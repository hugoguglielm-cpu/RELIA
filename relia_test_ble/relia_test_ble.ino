/* ============================================================
 * RELIA — TEST FLEXION SUR IPHONE (Bluetooth, sans ordinateur)
 *
 * App iPhone : "Bluefruit Connect" (Adafruit)
 *   -> se connecter a "RELIA"
 *   -> module "Plotter" : 3 courbes = A, B, force (en microvolts)
 *   -> module "UART"    : envoyer  t  = refaire le zero (tige SANS charge)
 *                                   ?  = etat
 *
 * Alimentation en salle : une batterie USB (powerbank) ou un chargeur
 * de telephone suffit.
 * Au demarrage : NE TOUCHE PAS la tige ~3 s (zero automatique).
 *
 * Par USB, le moniteur serie / relia_visu.py marchent toujours
 * (lignes "A_uV:.. B_uV:..").
 * ============================================================ */
#include <Wire.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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
long tareA = 0, tareB = 0;
long dernier[2] = {0, 0};
long hist3[2][3]; bool hist3Pret[2] = {false, false};

// ---------------- envoi ----------------
// Decoupe en paquets de 20 octets : marche meme si l'iPhone garde la
// taille de paquet BLE minimale.
void bleEnvoyer(const char *s) {
  if (!bleConnecte || !txChar) return;
  size_t n = strlen(s);
  for (size_t i = 0; i < n; i += 20) {
    size_t k = (n - i < 20) ? n - i : 20;
    txChar->setValue((uint8_t *)(s + i), k);
    txChar->notify();
    if (n > 20) delay(3);
  }
}
void msg(const char *s) {
  char b[140]; snprintf(b, sizeof(b), "# %s\n", s);
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

// ---------------- commandes ----------------
void tare() {
  long sa = 0, sb = 0;
  for (int i = 0; i < 16; i++) { sa += readChannel(0); sb += readChannel(1); }
  tareA = sa / 16; tareB = sb / 16;
  hist3Pret[0] = hist3Pret[1] = false;
  // tension entre A+ et A- (et B+ / B-) au repos : doit faire quelques mV au plus
  char b[120];
  snprintf(b, sizeof(b), "zero fait. Repos : A = %.2f mV, B = %.2f mV",
           tareA * UV_PAR_POINT / 1000.0f, tareB * UV_PAR_POINT / 1000.0f);
  msg(b);
  if (labs(tareA) > 8000000 || labs(tareB) > 8000000) msg("ATTENTION : une voie est SATUREE -> cablage");
}
void commande(String c) {
  c.trim();
  if (c.length() == 0) return;
  char x = tolower(c[0]);
  if (x == 't') tare();
  else if (x == '?') {
    char b[120];
    snprintf(b, sizeof(b), "zero A %.2f mV, B %.2f mV. Commandes : t = zero",
             tareA * UV_PAR_POINT / 1000.0f, tareB * UV_PAR_POINT / 1000.0f);
    msg(b);
  }
}

// ---------------- setup / loop ----------------
void setup() {
  // Anti-brownout : le CPU a 80 MHz consomme moins (largement assez ici)
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  Serial.println("# RELIA test flexion BLE");
  // 1) capteur d'abord
  if (!findNAU()) { while (1) { Serial.println("# ERREUR : NAU7802 introuvable"); delay(2000); } }
  bool ok = false;
  for (int essai = 1; essai <= 5 && !ok; essai++) { ok = nauInit(); if (!ok) delay(300); }
  if (!ok) { while (1) { Serial.println("# ERREUR : NAU7802 ne demarre pas"); delay(2000); } }
  // 2) Bluetooth
  delay(200); // laisse l'alimentation se stabiliser avant la radio
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
  Serial.println("# ne touche pas la tige...");
  delay(1000);
  tare();
}

void loop() {
  if (Serial.available()) commande(Serial.readStringUntil('\n'));
  if (bleCmdPrete) { bleCmdPrete = false; commande(bleCmd); }

  long a = filtre(0, readChannel(0)) - tareA;
  long b = filtre(1, readChannel(1)) - tareB;
  float aUv = a * UV_PAR_POINT, bUv = b * UV_PAR_POINT;
  float fUv = sqrtf(aUv * aUv + bUv * bUv);

  // USB : meme format que relia_test_flexion (Traceur serie / relia_visu.py)
  Serial.printf("A_uV:%.1f B_uV:%.1f\n", aUv, bUv);

  // iPhone (Plotter de Bluefruit) : A,B,force en uV entiers
  char l[40];
  snprintf(l, sizeof(l), "%ld,%ld,%ld\n", lroundf(aUv), lroundf(bUv), lroundf(fUv));
  bleEnvoyer(l);

  if (fabsf(aUv) > SATURE_UV || fabsf(bUv) > SATURE_UV) {
    static unsigned long dAlerte = 0;
    if (millis() - dAlerte > 3000) { msg("ATTENTION : voie SATUREE (faux contact ?)"); dAlerte = millis(); }
  }
}
