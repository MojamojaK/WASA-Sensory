// I2Cアドレス: 0x76

#include "Seeed_BME280.h"
#include <Wire.h>

#define BME_MULTI 100
#define STARTUP_WAIT 2000

#ifdef DEBUG_THP
uint32_t loop_time_thp = 0;
#endif

BME280 bme280;

boolean bme_enabled = false;
int16_t temperature = 0;
uint32_t pressure = 0;
int16_t p_altitude = 0;
uint16_t humidity = 0;

boolean connectionTHP(void);

void initTHP(void) {
#ifdef DEBUG_THP
  DEBUG_PORT.println("INIT THP");
#endif
  Wire.begin();
  if (connectionTHP()) {
    bme_enabled = bme280.init();
  }
}

// 温湿度・気圧センサーが接続されているか確認する関数
// ライブラリの init() メソッドだけだと接続されてない時に無限ループに入ってしまう
// (要はライブラリがクソ)
boolean connectionTHP(void) {
  boolean connectedTHP = false;
  uint32_t check_start = millis();
  // ここらへんの定数はライブラリ内から持ってきたものです
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(BME280_REG_CHIPID);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 1);
  while (millis() - check_start < 1000) {
    if (Wire.available()) {
      connectedTHP = true;
      break;
    }
    delay(1);
  }
  return connectedTHP;
}

void readTHP(void) {
#ifdef DEBUG_THP
  loop_time_thp = millis();
  DEBUG_PORT.println("READ THP");
#endif
  if (bme_enabled) {
    temperature = bme280.getTemperature() * BME_MULTI;
    pressure = bme280.getPressure();
    p_altitude = bme280.calcAltitude(pressure) * BME_MULTI;
    humidity = bme280.getHumidity();
  }
#ifdef DEBUG_THP
  DEBUG_PORT.print("THP Loop Time: ");
  DEBUG_PORT.print(millis() - loop_time_thp);
  DEBUG_PORT.print("\n");
  DEBUG_PORT.flush();
#endif
}

void packTHP(uint8_t *payload) {
  if (bme_enabled) {
    payload[45] = (uint8_t) (temperature & 0x00FF);
    payload[46] = (uint8_t) ((temperature & 0xFF00) >> 8);
    payload[47] = (uint8_t) (pressure & 0x000000FF);
    payload[48] = (uint8_t) ((pressure & 0x0000FF00) >> 8);
    payload[49] = (uint8_t) ((pressure & 0x00FF0000) >> 16);
    payload[50] = (uint8_t) ((pressure & 0xFF000000) >> 24);
    payload[51] = (uint8_t) (humidity & 0x000000FF);
    payload[52] = (uint8_t) ((humidity & 0x0000FF00) >> 8);
  }
}



