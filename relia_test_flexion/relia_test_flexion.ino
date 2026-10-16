/* ============================================================
 * RELIA — TEST FLEXION (USB seulement, pas de Bluetooth)
 *
 * Outils > Moniteur serie (ou Traceur serie), 115200 bauds.
 * Au demarrage : NE TOUCHE PAS la tige ~3 s (zero automatique).
 * Commande : t = refaire le zero
 * LIRE_A = false : on ne lit que la voie B (si la voie A est en panne)
 *
 * Affiche A et B en microvolts (uV) par rapport au zero :
 *   - au repos : quelques uV autour de 0
 *   - en pliant : A ou B bouge (dizaines a centaines de uV),
 *     le signe change selon le sens de la flexion
 * ============================================================ */
#include <Wire.h>

#define GAIN_CODE 5 // x32
#define RATE_CODE 3 // 80 mesures/s
#define LDO_CODE 5  // 3.0 V
#define DISCARD 3
#define NAU_ADDR 0x2A
const bool LIRE_A = false; // mettre true quand la voie A (J1) est reparee
// 1 point ADC = 0.5 * 3.0 V / 32 / 2^23  (~0.0056 uV)
const float UV_PAR_POINT = 0.5f * 3.0f / 32.0f / 8388608.0f * 1e6f;
const int SAFE_PINS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21}; // jamais 11-19
const int NSAFE = sizeof(SAFE_PINS) / sizeof(SAFE_PINS[0]);

long tareA = 0, tareB = 0;
long dernier[2] = {0, 0};
long hist3[2][3]; bool hist3Pret[2] = {false, false};

void wr(uint8_t r, uint8_t v) { Wire.beginTransmission(NAU_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
uint8_t rd(uint8_t r) {
  Wire.beginTransmission(NAU_ADDR); Wire.write(r); Wire.endTransmission(false);
  Wire.requestFrom((int)NAU_ADDR, 1); return Wire.available() ? Wire.read() : 0;
}
bool present() { Wire.beginTransmission(NAU_ADDR); return Wire.endTransmission() == 0; }

bool findNAU() {
  for (int i = 0; i < NSAFE; i++) for (int j = 0; j < NSAFE; j++) {
    if (i == j) continue;
    Wire.end(); delay(2);
    Wire.begin(SAFE_PINS[i], SAFE_PINS[j]); Wire.setClock(100000); Wire.setTimeOut(20); delay(5);
    if (present()) { Serial.printf("# NAU7802 trouve : SDA=%d SCL=%d\n", SAFE_PINS[i], SAFE_PINS[j]); return true; }
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

void tare() {
  long sa = 0, sb = 0;
  for (int i = 0; i < 16; i++) { if (LIRE_A) sa += readChannel(0); sb += readChannel(1); }
  tareA = sa / 16; tareB = sb / 16;
  hist3Pret[0] = hist3Pret[1] = false;
  // tension entre A+ et A- (et B+ / B-) au repos : doit faire quelques mV au plus
  Serial.printf("# zero fait. Ecart au repos : A = %.2f mV, B = %.2f mV\n",
                tareA * UV_PAR_POINT / 1000.0f, tareB * UV_PAR_POINT / 1000.0f);
  if (labs(tareA) > 8000000 || labs(tareB) > 8000000)
    Serial.println("# ATTENTION : une voie est SATUREE -> verifier le cablage");
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  Serial.println("# RELIA test flexion");
  if (!findNAU()) { while (1) { Serial.println("# ERREUR : NAU7802 introuvable"); delay(2000); } }
  bool ok = false;
  for (int essai = 1; essai <= 5 && !ok; essai++) { ok = nauInit(); if (!ok) delay(300); }
  if (!ok) { while (1) { Serial.println("# ERREUR : NAU7802 ne demarre pas"); delay(2000); } }
  Serial.println("# ne touche pas la tige...");
  delay(1000);
  tare();
}

void loop() {
  if (Serial.available()) {
    String c = Serial.readStringUntil('\n'); c.trim();
    if (c.length() && tolower(c[0]) == 't') tare();
  }
  long a = LIRE_A ? filtre(0, readChannel(0)) - tareA : 0;
  long b = filtre(1, readChannel(1)) - tareB;
  Serial.printf("A_uV:%.1f B_uV:%.1f\n", a * UV_PAR_POINT, b * UV_PAR_POINT);
}
