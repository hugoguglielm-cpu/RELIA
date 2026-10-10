/* ============================================================
 * RELIA — jauges + reps + calibrage, visible sur iPhone (Bluetooth)
 *
 * App iPhone : "Bluefruit Connect" (Adafruit, gratuite)
 *   -> se connecter a "RELIA"
 *   -> module "Plotter" : courbe poids (kg) + reps
 *   -> module "UART"    : envoyer les commandes ci-dessous
 *
 * Commandes (UART de l'app, ou Moniteur serie USB) :
 *   t   = zero (goupille SANS charge)
 *   c20 = calibrer : poids connu de 20 kg pose -> "c20" (mettre ta valeur)
 *   x   = effacer le calibrage
 *   r   = reps a 0
 *   b   = diagnostic bruit (NE RIEN TOUCHER pendant ~4 s) :
 *         valeurs brutes moy/min/max/ecart-type des voies A et B
 *   ?   = etat (+ nombre d'erreurs de lecture I2C)
 * Le calibrage reste en memoire meme apres debranchement.
 * L'USB envoie toujours les memes lignes qu'avant (relia_reps.py marche encore).
 *
 * Reps : elles sont comptees avec l'ACCELEROMETRE (la barre doit bouger,
 * ~0.12 g au-dessus du repos). Plier la goupille sans bouger la carte
 * ne compte pas de rep.
 * ============================================================ */
#include <Wire.h>
#include <math.h>
#include <limits.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------- reglages ----------------
#define GAIN_CODE 5
#define RATE_CODE 3
#define LDO_CODE 5
#define DISCARD 3 // conversions jetees apres changement de voie (2 = trop court, melange A/B)
const float SEUIL_HAUT = 0.12; // g : debut de montee
const float SEUIL_BAS = -0.08; // g : fin de rep
const unsigned long ECART_MIN_MS = 600;
const unsigned long FIN_SERIE_MS = 6000;
const float SEUIL_CHARGE_KG = 1.0;
const int SAFE_PINS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21}; // jamais 11-19
const int NSAFE = sizeof(SAFE_PINS) / sizeof(SAFE_PINS[0]);
#define NAU_ADDR 0x2A
#define MPU_ADDR 0x68
// ---------------- BLE (service UART Nordic) ----------------
#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
BLECharacteristic *txChar = nullptr;
bool bleConnecte = false;
String bleCmd = "";
volatile bool bleCmdPrete = false;
class ServeurCb : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override { bleConnecte = true; }
  void onDisconnect(BLEServer *s) override { bleConnecte = false; BLEDevice::startAdvertising(); }
};
class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    bleCmd = String(c->getValue().c_str());
    bleCmdPrete = true;
  }
};
// ---------------- etat ----------------
Preferences prefs;
bool mpuOk = false;
long tareA = 0, tareB = 0;
double sumFK = 0, sumFF = 0; int nPts = 0; float k = 0; // calibrage
float histF[20]; int iF = 0, nF = 0; // dernieres forces
float lis[5]; int iL = 0, nL = 0; // lissage
float base = -1; bool montee = false;
unsigned long dernierRep = 0; int reps = 0;
double sumKgSerie = 0; long nKgSerie = 0;
long dernier[2] = {0, 0};          // derniere lecture valide par voie
long hist3[2][3]; bool hist3Pret[2] = {false, false}; // anti-pic (mediane de 3)
unsigned long nErrI2C = 0;
// ---------------- I2C / NAU7802 ----------------
void wr(uint8_t d, uint8_t r, uint8_t v) { Wire.beginTransmission(d); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
uint8_t rd(uint8_t d, uint8_t r) {
  Wire.beginTransmission(d); Wire.write(r); Wire.endTransmission(false);
  Wire.requestFrom((int)d, 1); return Wire.available() ? Wire.read() : 0;
}
bool present(uint8_t d) { Wire.beginTransmission(d); return Wire.endTransmission() == 0; }
bool findNAU() {
  for (int i = 0; i < NSAFE; i++) for (int j = 0; j < NSAFE; j++) {
    if (i == j) continue;
    Wire.end(); delay(2);
    Wire.begin(SAFE_PINS[i], SAFE_PINS[j]); Wire.setClock(100000); Wire.setTimeOut(20); delay(5);
    if (present(NAU_ADDR)) {
      Serial.printf("# NAU7802 trouve : SDA=%d SCL=%d\n", SAFE_PINS[i], SAFE_PINS[j]);
      return true;
    }
    delay(1);
  }
  return false;
}
bool nauInit() {
  wr(NAU_ADDR, 0x00, 0x01); delay(50);
  wr(NAU_ADDR, 0x00, 0x00); delay(10);
  wr(NAU_ADDR, 0x00, 0x02);
  unsigned long t0 = millis();
  uint8_t pu = 0;
  while (!((pu = rd(NAU_ADDR, 0x00)) & 0x08)) {
    if (millis() - t0 > 1500) {
      Serial.printf("# diag : PU_CTRL=0x%02X REV=0x%02X\n", pu, rd(NAU_ADDR, 0x1F));
      return false;
    }
    delay(5);
  }
  wr(NAU_ADDR, 0x00, 0x02 | 0x04 | 0x80);
  wr(NAU_ADDR, 0x01, (LDO_CODE << 3) | GAIN_CODE);
  wr(NAU_ADDR, 0x02, (RATE_CODE << 4));
  wr(NAU_ADDR, 0x15, rd(NAU_ADDR, 0x15) | 0x30);
  delay(300);
  wr(NAU_ADDR, 0x00, rd(NAU_ADDR, 0x00) | 0x10);
  return true;
}
// Lit une conversion. false si timeout ou lecture I2C incomplete.
bool readOne(long &v) {
  unsigned long t0 = millis();
  while (!(rd(NAU_ADDR, 0x00) & 0x20)) { if (millis() - t0 > 200) return false; }
  Wire.beginTransmission(NAU_ADDR); Wire.write(0x12);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)NAU_ADDR, 3) != 3) return false;
  // une lecture par instruction : l'ordre des Wire.read() est garanti
  long b2 = Wire.read();
  long b1 = Wire.read();
  long b0 = Wire.read();
  long x = (b2 << 16) | (b1 << 8) | b0;
  if (x & 0x800000) x |= 0xFF000000;
  v = x;
  return true;
}
// Valeur brute d'une voie ; en cas d'erreur, renvoie la derniere valeur valide.
long readChannel(uint8_t ch) {
  uint8_t c2 = rd(NAU_ADDR, 0x02);
  wr(NAU_ADDR, 0x02, ch ? (c2 | 0x80) : (c2 & 0x7F));
  long v;
  for (int i = 0; i < DISCARD; i++) readOne(v);
  if (readOne(v)) dernier[ch] = v;
  else nErrI2C++;
  return dernier[ch];
}
long med3(long x, long y, long z) {
  if (x > y) { long t = x; x = y; y = t; }
  if (y > z) y = z;
  return x > y ? x : y;
}
// Anti-pic : mediane des 3 dernieres lectures de la voie.
long filtre(uint8_t ch, long v) {
  if (!hist3Pret[ch]) { hist3[ch][0] = hist3[ch][1] = hist3[ch][2] = v; hist3Pret[ch] = true; }
  hist3[ch][0] = hist3[ch][1]; hist3[ch][1] = hist3[ch][2]; hist3[ch][2] = v;
  return med3(hist3[ch][0], hist3[ch][1], hist3[ch][2]);
}
// ---------------- envoi ----------------
void envoyer(const char *s) {
  Serial.print(s);
  if (bleConnecte && txChar) { txChar->setValue((uint8_t *)s, strlen(s)); txChar->notify(); delay(5); }
}
void msg(const char *s) { char b[120]; snprintf(b, sizeof(b), "# %s\n", s); envoyer(b); }
// ---------------- commandes ----------------
void tare() {
  long sa = 0, sb = 0;
  for (int i = 0; i < 16; i++) { sa += readChannel(0); sb += readChannel(1); }
  tareA = sa / 16; tareB = sb / 16;
  hist3Pret[0] = hist3Pret[1] = false;
  msg("zero fait");
}
// Bruit au repos : statistiques des valeurs BRUTES (sans zero, sans filtre).
void diagnostic() {
  const int N = 40;
  char b[120];
  msg("diagnostic : ne touche a rien ~4 s");
  for (int ch = 0; ch < 2; ch++) {
    double s = 0, s2 = 0; long mn = LONG_MAX, mx = LONG_MIN;
    for (int i = 0; i < N; i++) {
      long v = readChannel(ch);
      s += v; s2 += (double)v * v;
      if (v < mn) mn = v;
      if (v > mx) mx = v;
    }
    double m = s / N, var = s2 / N - m * m;
    snprintf(b, sizeof(b), "voie %c brut : moy %.0f min %ld max %ld ecart-type %.0f",
             ch ? 'B' : 'A', m, mn, mx, sqrt(var > 0 ? var : 0));
    msg(b);
  }
  snprintf(b, sizeof(b), "zero A %ld B %ld, erreurs I2C %lu", tareA, tareB, nErrI2C); msg(b);
}
void sauverCalib() {
  prefs.putDouble("fk", sumFK); prefs.putDouble("ff", sumFF); prefs.putInt("n", nPts);
}
void commande(String c) {
  c.trim();
  if (c.length() == 0) return;
  char b[120];
  char x = tolower(c[0]);
  if (x == 't') tare();
  else if (x == 'r') { reps = 0; sumKgSerie = 0; nKgSerie = 0; msg("reps a 0"); }
  else if (x == 'x') { sumFK = sumFF = 0; nPts = 0; k = 0; sauverCalib(); msg("calibrage efface"); }
  else if (x == 'b') diagnostic();
  else if (x == 'c') {
    float kg = c.substring(1).toFloat();
    float f = 0; for (int i = 0; i < nF; i++) f += histF[i]; f = nF ? f / nF : 0;
    if (kg <= 0) { msg("ecris le poids apres c, ex: c20"); return; }
    if (f < 1000) { msg("signal trop faible : pose le poids d'abord"); return; }
    sumFK += (double)f * kg; sumFF += (double)f * f; nPts++;
    k = sumFK / sumFF; sauverCalib();
    snprintf(b, sizeof(b), "point %d ok : %.1f kg -> relu %.2f kg", nPts, kg, f * k); msg(b);
  }
  else if (x == '?') {
    snprintf(b, sizeof(b), "calibrage %s (%d points), reps %d, erreurs I2C %lu",
             k > 0 ? "OK" : "absent", nPts, reps, nErrI2C); msg(b);
  }
}
// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  Serial.println("# RELIA jauges BLE");
  prefs.begin("relia", false);
  sumFK = prefs.getDouble("fk", 0); sumFF = prefs.getDouble("ff", 0); nPts = prefs.getInt("n", 0);
  k = (sumFF > 0) ? sumFK / sumFF : 0;
  // 1) capteur d'abord (avant le Bluetooth), avec 5 essais
  if (!findNAU()) { while (1) { Serial.println("# ERREUR : NAU7802 introuvable"); delay(2000); } }
  bool ok = false;
  for (int essai = 1; essai <= 5 && !ok; essai++) {
    ok = nauInit();
    if (!ok) { Serial.printf("# essai %d rate, je recommence\n", essai); delay(300); }
  }
  if (!ok) { while (1) { Serial.println("# ERREUR : NAU7802 ne demarre pas (voir ligne diag)"); delay(2000); } }
  Serial.println("# NAU7802 demarre");
  mpuOk = present(MPU_ADDR);
  if (mpuOk) wr(MPU_ADDR, 0x6B, 0x00);
  // 2) Bluetooth
  BLEDevice::init("RELIA");
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new ServeurCb());
  BLEService *svc = srv->createService(NUS_SERVICE);
  txChar = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());
  BLECharacteristic *rx = svc->createCharacteristic(NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCb());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  BLEAdvertisementData annonce; // le NOM dans l'annonce principale
  annonce.setFlags(0x06);
  annonce.setName("RELIA");
  BLEAdvertisementData reponse; // le service UART dans la reponse
  reponse.setCompleteServices(BLEUUID(NUS_SERVICE));
  adv->setAdvertisementData(annonce);
  adv->setScanResponseData(reponse);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("# Bluetooth actif : nom RELIA");
  delay(1500);
  tare();
  Serial.println("t_ms,A,B,force,angle_deg,ax,ay,az");
}
void loop() {
  if (Serial.available()) commande(Serial.readStringUntil('\n'));
  if (bleCmdPrete) { bleCmdPrete = false; commande(bleCmd); }
  long ra = readChannel(0), rb = readChannel(1);
  long a = filtre(0, ra) - tareA, b = filtre(1, rb) - tareB;
  float f = sqrtf((float)a * a + (float)b * b);
  float ang = atan2f((float)b, (float)a) * 180.0f / PI;
  histF[iF] = f; iF = (iF + 1) % 20; if (nF < 20) nF++;
  float ax = 0, ay = 0, az = 0;
  if (mpuOk) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
    if (Wire.requestFrom((int)MPU_ADDR, 6) == 6) {
      uint8_t r[6];
      for (int i = 0; i < 6; i++) r[i] = Wire.read();
      int16_t x = (int16_t)((r[0] << 8) | r[1]);
      int16_t y = (int16_t)((r[2] << 8) | r[3]);
      int16_t z = (int16_t)((r[4] << 8) | r[5]);
      ax = x / 16384.0; ay = y / 16384.0; az = z / 16384.0;
    }
  }
  // --- reps (accelerometre) ---
  unsigned long now = millis();
  float amag = sqrtf(ax * ax + ay * ay + az * az);
  if (amag > 0.2) {
    if (base < 0) base = amag;
    float dev = amag - base;
    if (fabsf(dev) < 0.05) base += 0.02 * (amag - base);
    if (!montee && dev > SEUIL_HAUT) montee = true;
    else if (montee && dev < SEUIL_BAS) {
      montee = false;
      if (now - dernierRep > ECART_MIN_MS) { reps++; dernierRep = now; }
    }
  }
  // --- poids ---
  float kg = (k > 0) ? f * k : f;
  lis[iL] = kg; iL = (iL + 1) % 5; if (nL < 5) nL++;
  float kgL = 0; for (int i = 0; i < nL; i++) kgL += lis[i]; kgL /= nL;
  if (k > 0 && kgL > SEUIL_CHARGE_KG && reps > 0) { sumKgSerie += kgL; nKgSerie++; }
  if (reps > 0 && now - dernierRep > FIN_SERIE_MS) {
    char b[80];
    snprintf(b, sizeof(b), "SERIE : %d reps a %.1f kg", reps, nKgSerie ? sumKgSerie / nKgSerie : 0.0);
    msg(b);
    reps = 0; sumKgSerie = 0; nKgSerie = 0;
  }
  if (labs(ra) > 8380000 || labs(rb) > 8380000) {
    static unsigned long dAlerte = 0;
    if (now - dAlerte > 3000) { msg("ATTENTION : capteur sature"); dAlerte = now; }
  }
  // --- USB (meme format qu'avant, pour relia_reps.py) ---
  Serial.printf("%lu,%ld,%ld,%.1f,%.1f,%.3f,%.3f,%.3f\n", now, a, b, f, ang, ax, ay, az);
  // --- iPhone : poids, reps (pour le Plotter) ---
  if (bleConnecte && txChar) {
    char l[32];
    snprintf(l, sizeof(l), "%.1f,%d\n", (k > 0 ? kgL : kgL / 100000.0), reps);
    txChar->setValue((uint8_t *)l, strlen(l)); txChar->notify();
  }
}
