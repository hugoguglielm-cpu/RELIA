/* ============================================================
 * RELIA — POIDS EN KG SUR IPHONE (Bluetooth, sans ordinateur)
 *
 * App iPhone : "Bluefruit Connect" -> se connecter a "RELIA" -> module UART
 *   Toutes les 0,5 s :   12.3 kg   (max 15.2)
 *
 * Commandes (a envoyer dans le module UART) :
 *   t     = zero  (tige SANS charge)
 *   c20   = calibrer : un poids connu de 20 kg est accroche -> envoyer "c20"
 *           (mettre ta valeur ; plusieurs poids differents = plus precis)
 *   x     = effacer le calibrage
 *   r     = remettre le max a 0
 *   p     = mode courbe (Plotter) / mode texte (UART)
 *   ?     = etat
 * Le calibrage reste en memoire meme apres debranchement.
 *
 * Au demarrage : NE TOUCHE PAS la tige ~3 s (zero automatique).
 * Par USB (moniteur serie 115200) : memes messages + lignes A_uV / B_uV.
 * ============================================================ */
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_system.h>

#define GAIN_CODE 5 // x32
#define RATE_CODE 3 // 80 mesures/s
#define LDO_CODE 5  // 3.0 V
#define DISCARD 3
#define NAU_ADDR 0x2A
#define SDA_PIN 0
#define SCL_PIN 1
// 1 point ADC = 0.5 * 3.0 V / 32 / 2^23  (~0.0056 uV)
const float UV_PAR_POINT = 0.5f * 3.0f / 32.0f / 8388608.0f * 1e6f;
const float SATURE_UV = 45000;
const float FORCE_MIN_CALIB_UV = 20;     // en dessous : poids trop leger pour calibrer
const unsigned long PERIODE_TEXTE_MS = 500;
#define NMOY 10                          // moyenne glissante (~1 s)

// ---------------- BLE (service UART Nordic, comme Bluefruit) ----------------
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
long tareA = 0, tareB = 0;
long dernier[2] = {0, 0};
long hist3[2][3]; bool hist3Pret[2] = {false, false};
unsigned long nEchecs = 0;
// calibrage : kg = k * force_uV  (moindres carres sur tous les points)
double sumFK = 0, sumFF = 0; int nPts = 0; float k = 0;
float moy[NMOY]; int iMoy = 0, nMoy = 0;  // dernieres forces (uV)
float kgMax = 0;
bool modeCourbe = false;

// ---------------- envoi ----------------
// Decoupe en paquets de 20 octets : marche meme si l'iPhone garde la
// taille de paquet BLE minimale.
void bleEnvoyer(const char *s) {
  if (!bleConnecte || !txChar) return;
  size_t n = strlen(s);
  for (size_t i = 0; i < n; i += 20) {
    size_t kk = (n - i < 20) ? n - i : 20;
    txChar->setValue((uint8_t *)(s + i), kk);
    txChar->notify();
    if (n > 20) delay(3);
  }
}
void msg(const char *s) {
  char b[160]; snprintf(b, sizeof(b), "# %s\n", s);
  Serial.print(b);
  bleEnvoyer(b);
}

// ---------------- I2C / NAU7802 ----------------
void wr(uint8_t r, uint8_t v) { Wire.beginTransmission(NAU_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
uint8_t rd(uint8_t r) {
  Wire.beginTransmission(NAU_ADDR); Wire.write(r); Wire.endTransmission(false);
  Wire.requestFrom((int)NAU_ADDR, 1); return Wire.available() ? Wire.read() : 0;
}
bool present() { Wire.beginTransmission(NAU_ADDR); return Wire.endTransmission() == 0; }

// Si l'ESP32 redemarre en pleine lecture, le NAU7802 (toujours alimente)
// peut garder SDA a 0 et bloquer le bus. On envoie des coups d'horloge
// jusqu'a ce qu'il relache SDA, puis une condition STOP.
void debloquerI2C() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  for (int i = 0; i < 18 && digitalRead(SDA_PIN) == LOW; i++) {
    digitalWrite(SCL_PIN, LOW); delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  }
  digitalWrite(SCL_PIN, LOW); delayMicroseconds(10);
  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA_PIN, LOW); delayMicroseconds(10);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(10);
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
}
bool findNAU() {
  for (int essai = 1; essai <= 5; essai++) {
    Wire.end(); delay(2);
    debloquerI2C();
    Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(100000); Wire.setTimeOut(20); delay(5);
    if (present()) { Serial.printf("# NAU7802 trouve : SDA=%d SCL=%d\n", SDA_PIN, SCL_PIN); return true; }
    delay(100);
  }
  return false;
}

bool nauInit() {
  wr(0x00, 0x01); delay(50);
  wr(0x00, 0x00); delay(10);
  wr(0x00, 0x02);
  unsigned long t0 = millis();
  while (!(rd(0x00) & 0x08)) { if (millis() - t0 > 1500) return false; delay(5); }
  wr(0x00, 0x02 | 0x04 | 0x80);
  wr(0x01, (LDO_CODE << 3) | GAIN_CODE);
  wr(0x02, (RATE_CODE << 4));
  wr(0x15, rd(0x15) | 0x30);
  delay(300);
  wr(0x00, rd(0x00) | 0x10);
  return true;
}

bool readOne(long &v) {
  unsigned long t0 = millis();
  while (!(rd(0x00) & 0x20)) { if (millis() - t0 > 200) return false; }
  Wire.beginTransmission(NAU_ADDR); Wire.write(0x12);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)NAU_ADDR, 3) != 3) return false;
  long b2 = Wire.read();
  long b1 = Wire.read();
  long b0 = Wire.read();
  long x = (b2 << 16) | (b1 << 8) | b0;
  if (x & 0x800000) x |= 0xFF000000;
  v = x;
  return true;
}

long readChannel(uint8_t ch) {
  uint8_t c2 = rd(0x02);
  wr(0x02, ch ? (c2 | 0x80) : (c2 & 0x7F));
  long v;
  for (int i = 0; i < DISCARD; i++) readOne(v);
  if (readOne(v)) dernier[ch] = v;
  else nEchecs++;
  return dernier[ch];
}

long med3(long x, long y, long z) {
  if (x > y) { long t = x; x = y; y = t; }
  if (y > z) y = z;
  return x > y ? x : y;
}
long filtre(uint8_t ch, long v) {
  if (!hist3Pret[ch]) { hist3[ch][0] = hist3[ch][1] = hist3[ch][2] = v; hist3Pret[ch] = true; }
  hist3[ch][0] = hist3[ch][1]; hist3[ch][1] = hist3[ch][2]; hist3[ch][2] = v;
  return med3(hist3[ch][0], hist3[ch][1], hist3[ch][2]);
}

float forceMoyenne() {
  float s = 0;
  for (int i = 0; i < nMoy; i++) s += moy[i];
  return nMoy ? s / nMoy : 0;
}

// ---------------- commandes ----------------
void tare() {
  long sa = 0, sb = 0;
  for (int i = 0; i < 16; i++) { sa += readChannel(0); sb += readChannel(1); }
  tareA = sa / 16; tareB = sb / 16;
  hist3Pret[0] = hist3Pret[1] = false;
  nMoy = 0; kgMax = 0;
  // tension entre A+ et A- (et B+ / B-) au repos : doit faire quelques mV au plus
  char b[120];
  snprintf(b, sizeof(b), "zero fait. Repos : A = %.2f mV, B = %.2f mV",
           tareA * UV_PAR_POINT / 1000.0f, tareB * UV_PAR_POINT / 1000.0f);
  msg(b);
  if (labs(tareA) > 8000000 || labs(tareB) > 8000000) msg("ATTENTION : une voie est SATUREE -> cablage");
}
void sauverCalib() {
  prefs.putDouble("fk", sumFK); prefs.putDouble("ff", sumFF); prefs.putInt("n", nPts);
}
void commande(String c) {
  c.trim();
  c.replace(',', '.'); // "c20,5" -> "c20.5"
  if (c.length() == 0) return;
  char b[160];
  char x = tolower(c[0]);
  if (x == 't') tare();
  else if (x == 'r') { kgMax = 0; msg("max remis a 0"); }
  else if (x == 'x') { sumFK = sumFF = 0; nPts = 0; k = 0; sauverCalib(); msg("calibrage efface"); }
  else if (x == 'p') {
    modeCourbe = !modeCourbe;
    msg(modeCourbe ? "mode courbe : ouvre le Plotter" : "mode texte : ouvre l'UART");
  }
  else if (x == 'c') {
    float kg = c.substring(1).toFloat();
    float f = forceMoyenne();
    if (kg <= 0) { msg("ecris le poids apres c, ex : c20"); return; }
    if (f < FORCE_MIN_CALIB_UV) {
      snprintf(b, sizeof(b), "signal trop faible (%.0f uV) : accroche le poids d'abord", f);
      msg(b); return;
    }
    sumFK += (double)f * kg; sumFF += (double)f * f; nPts++;
    k = sumFK / sumFF; sauverCalib();
    snprintf(b, sizeof(b), "point %d ok : %.1f kg -> relu %.2f kg", nPts, kg, f * k);
    msg(b);
  }
  else if (x == '?') {
    snprintf(b, sizeof(b), "calibrage %s (%d point%s). Zero : A %.2f mV, B %.2f mV",
             k > 0 ? "OK" : "ABSENT", nPts, nPts > 1 ? "s" : "",
             tareA * UV_PAR_POINT / 1000.0f, tareB * UV_PAR_POINT / 1000.0f);
    msg(b);
    msg("commandes : t zero, c20 calibrer, x effacer, r max, p courbe");
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(3000); // laisse le temps au moniteur serie de se reconnecter
  Serial.println("# RELIA poids (kg)");
  Serial.printf("# redemarrage precedent : %s\n",
                esp_reset_reason() == ESP_RST_BROWNOUT ? "BROWNOUT" : "normal");
  prefs.begin("relia_kg", false); // nouveau nom : les anciens calibrages (autres unites) sont ignores
  sumFK = prefs.getDouble("fk", 0); sumFF = prefs.getDouble("ff", 0); nPts = prefs.getInt("n", 0);
  k = (sumFF > 0) ? sumFK / sumFF : 0;

  // 1) Bluetooth d'abord (ordre qui evite les brownouts sur cette carte)
  Serial.println("# demarrage Bluetooth...");
  BLEDevice::init("RELIA");
  BLEDevice::setPower(ESP_PWR_LVL_N0); // 0 dBm : moins de pic de courant, portee ~5-10 m
  BLEDevice::setMTU(185);
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
  delay(500);

  // 2) capteur ensuite
  if (!findNAU()) { while (1) { msg("ERREUR : NAU7802 introuvable"); delay(2000); } }
  bool ok = false;
  for (int essai = 1; essai <= 5 && !ok; essai++) { ok = nauInit(); if (!ok) delay(300); }
  if (!ok) { while (1) { msg("ERREUR : NAU7802 ne demarre pas"); delay(2000); } }
  Serial.println("# NAU7802 demarre");
  Serial.println("# ne touche pas la tige...");
  delay(1000);
  tare();
  if (k <= 0) msg("pas encore calibre : accroche un poids connu puis envoie c<kg>, ex : c20");
}

void loop() {
  if (Serial.available()) commande(Serial.readStringUntil('\n'));
  if (bleCmdPrete) { bleCmdPrete = false; commande(bleCmd); }

  long a = filtre(0, readChannel(0)) - tareA;
  long b = filtre(1, readChannel(1)) - tareB;
  float aUv = a * UV_PAR_POINT, bUv = b * UV_PAR_POINT;
  float fUv = sqrtf(aUv * aUv + bUv * bUv);
  moy[iMoy] = fUv; iMoy = (iMoy + 1) % NMOY; if (nMoy < NMOY) nMoy++;
  float fMoy = forceMoyenne();
  float kg = (k > 0) ? fMoy * k : 0;
  if (kg > kgMax) kgMax = kg;

  // USB : meme format que relia_test_flexion (Traceur serie / relia_visu.py)
  Serial.printf("A_uV:%.1f B_uV:%.1f\n", aUv, bUv);

  static unsigned long dTexte = 0;
  if (modeCourbe) {
    // Plotter de Bluefruit : un nombre par ligne (kg, ou uV si pas calibre)
    char l[24];
    snprintf(l, sizeof(l), "%.2f\n", k > 0 ? kg : fMoy);
    bleEnvoyer(l);
  } else if (millis() - dTexte > PERIODE_TEXTE_MS) {
    dTexte = millis();
    char l[64];
    if (k > 0) snprintf(l, sizeof(l), "%.1f kg   (max %.1f)\n", kg, kgMax);
    else snprintf(l, sizeof(l), "non calibre : %.0f uV\n", fMoy);
    bleEnvoyer(l);
  }

  // sans mesure, les valeurs resteraient figees : on le dit clairement
  static unsigned long echecsVus = 0, dEchec = 0;
  if (nEchecs != echecsVus && millis() - dEchec > 3000) {
    msg("ERREUR : le NAU7802 ne donne pas de mesure (alim AVDD / cablage ?)");
    echecsVus = nEchecs; dEchec = millis();
  }
  if (fabsf(aUv) > SATURE_UV || fabsf(bUv) > SATURE_UV) {
    static unsigned long dAlerte = 0;
    if (millis() - dAlerte > 3000) { msg("ATTENTION : voie SATUREE (faux contact ?)"); dAlerte = millis(); }
  }
}
