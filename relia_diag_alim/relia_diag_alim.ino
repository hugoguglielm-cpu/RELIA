/* ============================================================
 * RELIA — DIAGNOSTIC ALIMENTATION (brownout)
 *
 * Allume les elements UN PAR UN, avec 3 s d'attente entre chaque,
 * et affiche l'etape en cours. Si la carte redemarre, la premiere
 * ligne du demarrage suivant indique si c'etait un BROWNOUT, et la
 * derniere etape affichee avant indique ce qui l'a provoque.
 *
 * Moniteur serie : 115200 bauds.
 * Le detecteur de brownout reste ACTIF ici (c'est voulu).
 * ============================================================ */
#include <Wire.h>
#include <BLEDevice.h>
#include <esp_system.h>

#define NAU_ADDR 0x2A
#define SDA_PIN 0
#define SCL_PIN 1
#define LDO_CODE 5  // 3.0 V
#define GAIN_CODE 5 // x32

const char *raison(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "mise sous tension";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SW: return "logiciel (televersement / reset)";
    case ESP_RST_PANIC: return "plantage du programme (panic)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_EXT: return "bouton reset";
    default: return "autre";
  }
}

void etape(const char *s) {
  Serial.println(s);
  Serial.flush();
  delay(3000);
}

void wr(uint8_t r, uint8_t v) { Wire.beginTransmission(NAU_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
uint8_t rd(uint8_t r) {
  Wire.beginTransmission(NAU_ADDR); Wire.write(r); Wire.endTransmission(false);
  Wire.requestFrom((int)NAU_ADDR, 1); return Wire.available() ? Wire.read() : 0;
}

// meme deblocage du bus I2C que dans relia_test_ble
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

void setup() {
  Serial.begin(115200);
  delay(3000); // laisse le temps au moniteur serie de se reconnecter
  Serial.printf("# DIAG ALIM. Redemarrage precedent : %s\n", raison(esp_reset_reason()));

  etape("# 1/3 : rien d'allume (attente 3 s)");

  BLEDevice::init("RELIA-DIAG");
  BLEDevice::startAdvertising();
  etape("# 2/3 : Bluetooth allume (attente 3 s)");

  debloquerI2C();
  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(100000); Wire.setTimeOut(20);
  wr(0x00, 0x01); delay(50);           // reset du NAU7802
  wr(0x00, 0x00); delay(10);
  wr(0x00, 0x02); delay(10);           // mise en route numerique
  wr(0x01, (LDO_CODE << 3) | GAIN_CODE);
  wr(0x00, 0x02 | 0x04 | 0x80);        // + analogique + LDO : les jauges sont alimentees
  Serial.printf("# NAU7802 PU_CTRL = 0x%02X\n", rd(0x00));
  etape("# 3/3 : capteur + jauges allumes (attente 3 s)");

  Serial.println("# FIN : tout est allume, pas de brownout");
}

void loop() {
  Serial.println(millis());
  delay(1000);
}
