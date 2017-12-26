#include <XBee.h>                         //Xbee経由PC通信用
#include <AndroidAccessory.h>
#include "NineAxesMotion.h"
#include <Wire.h>

#define XBEE_SERIAL Serial  //XBee用シリアル
#define CONTROL_SERIAL Serial3

#define PROP_PIN 2          //プロペラ回転数計インタラプト専用ピン
#define TACH_PIN 3          //機速計インタラプト専用ピン
#define ALTI_PIN A0
#define MPU_PIN 18          //加速度ジャイロセンサーインタラプト専用ピン

#define PROP_SLITS 32       //回転数計スリット数
#define TACH_SLITS 100      //ロータリーエンコーダの分解能
#define TACH_COEFF 10       //[0.01回転/秒]に掛けて[m/s]を得るための係数

#define MPU_MAX_CHECKS 20   //MPU応答確認回数
#define ALTIMETER_SAMPLES 10//高度計の計測に利用する高度のサンプル数

#define PAYLOAD_SIZE 32     //APIモードXBee通信データパケットサイズ

#define SENSORY_COMM_MAX_BYTES 64

XBee xbee = XBee();         //APIモードXBee通信インスタンス

// 送信先のXbeeのシリアル番号
XBeeAddress64 addr64_boat = XBeeAddress64(0x0013A200, 0x40E7EAF6);
XBeeAddress64 addr64_land = XBeeAddress64(0x0013A200, 0x40E7EAF6);

#define transmit_cooldown 0
uint32_t last_transmit = 0;

uint8_t payload[PAYLOAD_SIZE] = {0}; //XBee通信データパケット

ZBTxRequest boat_packet = ZBTxRequest(addr64_boat, payload, PAYLOAD_SIZE);
ZBTxRequest land_packet = ZBTxRequest(addr64_land, payload, PAYLOAD_SIZE);

// AndroidAccessory
AndroidAccessory android("WASA", "Sensory001", "HPA Sensory Hardware", "0.0.2", "", "0000000114514810");

NineAxesMotion accelGyroMag;         //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 20;          //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))
unsigned long lastConfig = 0;
int16_t accel_offset[4] = { -23, 10, 5, 1000};
int16_t mag_offset[4] = {91, 255, 275, 615};
int16_t gyro_offset[3] = {1, -2, -1};
bool updateSensorData = true;

#define IMU_MULTI 100
int16_t heading = 0;
int16_t roll = 0;
int16_t pitch = 0;
int16_t accel_x = 0;
int16_t accel_y = 0;
int16_t accel_z = 0;
uint8_t calib_mag = 0;
uint8_t calib_accel = 0;
uint8_t calib_gyro = 0;
uint8_t calib_system = 0;

// 回転式インタラプト用時間管理引数
unsigned long rot_last_calc = 0;
unsigned long rot_curr_calc = 0;
unsigned long tach_delta = 0;
const unsigned long min_tach_delta = 150000;

volatile uint16_t prop_interrupts = 0;
uint16_t prop_rotation = 0;               //[回転/分]
#define PROP_PROPORTION 4                // prop_proportion回機速計の計測に対して1回回転数計の計測を行う
uint8_t prop_waits = 0;                   // 機速計の計測を待った回数
unsigned long prop_rot_last_calc = 0;
unsigned long prop_delta = 0;

volatile uint16_t tach_interrupts = 0;
uint16_t tach_rotation = 0; //[0.01回転/秒]

uint32_t alti_time = 0;
uint16_t alti = 0;
uint8_t alti_index = 0;
uint16_t alti_original[ALTIMETER_SAMPLES] = {0};
uint16_t alti_sample[ALTIMETER_SAMPLES] = {0};

uint8_t servo_rx_packet[SENSORY_COMM_MAX_BYTES] = {0};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  XBEE_SERIAL.begin(9600);                  // Xbee経由PC通信用シリアルの開始
  xbee.setSerial(XBEE_SERIAL);
  XBEE_SERIAL.print('B');

  CONTROL_SERIAL.begin(9600);

  I2C.begin();
  //Sensor Initialization
  accelGyroMag.initSensor();          //The I2C Address can be changed here inside this function in the library
  accelGyroMag.setOperationMode(OPERATION_MODE_CONFIG);
  accelGyroMag.writeAccelOffset(accel_offset);
  accelGyroMag.writeMagOffset(mag_offset);
  accelGyroMag.writeGyroOffset(gyro_offset);
  accelGyroMag.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  accelGyroMag.setUpdateMode(MANUAL);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor
  accelGyroMag.updateAccelConfig();

  pinMode(PROP_PIN, INPUT_PULLUP);
  pinMode(TACH_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PROP_PIN), prop_count_handle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tach_count_handle, FALLING);
  interrupts();
  rot_last_calc = micros();
  prop_rot_last_calc = rot_last_calc;

  pinMode(ALTI_PIN, INPUT);
}

void loop() {
  readAltitude();
  updateAccelGyroMag();
  readAccelGyroMag();
  readAltitude();
  stop_interrupts();
  calculateRotations();
  restart_interrupts();
  readAltitude();
  readServoInfo();
  readAltitude();
  transmitPayload();
}

void prop_count_handle() {
  prop_interrupts++;
}

void tach_count_handle() {
  tach_interrupts++;
}

void updateAccelGyroMag() {
  if (updateSensorData) {
    accelGyroMag.updateAccel();        //Update the Accelerometer data
    accelGyroMag.updateLinearAccel();  //Update the Linear Acceleration data
    accelGyroMag.updateGravAccel();    //Update the Gravity Acceleration data
    accelGyroMag.updateEuler();        //Update the Euler data into the structure of the object
    accelGyroMag.updateCalibStatus();  //Update the Calibration Status
    updateSensorData = false;
  }
}

void readAccelGyroMag() {
  if ((millis() - lastStreamTime) >= streamPeriod) {
    lastStreamTime = millis();
    heading = accelGyroMag.readEulerHeading();
    roll    = accelGyroMag.readEulerRoll() * IMU_MULTI;
    pitch   = accelGyroMag.readEulerPitch() * IMU_MULTI;
    accel_x = accelGyroMag.readLinearAcceleration(X_AXIS) * IMU_MULTI;
    accel_y = accelGyroMag.readLinearAcceleration(Y_AXIS) * IMU_MULTI;
    accel_z = accelGyroMag.readLinearAcceleration(Z_AXIS) * IMU_MULTI;
    calib_accel   = (uint8_t)accelGyroMag.readAccelCalibStatus();  //Accelerometer Calibration Status (0 - 3)
    calib_mag     = (uint8_t)accelGyroMag.readMagCalibStatus();    //Magnetometer Calibration Status (0 - 3)
    calib_gyro    = (uint8_t)accelGyroMag.readGyroCalibStatus();   //Gyroscope Calibration Status (0 - 3)
    calib_system  = (uint8_t)accelGyroMag.readSystemCalibStatus(); //System Calibration Status (0 - 3)
    payload[13] = lowByte(heading);
    payload[14] = highByte(heading);
    payload[15] = lowByte(roll);
    payload[16] = highByte(roll);
    payload[17] = lowByte(pitch);
    payload[18] = highByte(pitch);
    payload[19] = lowByte(accel_x);
    payload[20] = highByte(accel_x);
    payload[21] = lowByte(accel_y);
    payload[22] = highByte(accel_y);
    payload[23] = lowByte(accel_z);
    payload[24] = highByte(accel_z);
    payload[27] = 0;
    payload[27] |= calib_accel;
    payload[27] |= calib_mag << 2;
    payload[27] |= calib_gyro << 4;
    payload[27] |= calib_system << 6;
    updateSensorData = true;
  }
}

void readAltitude() {
  if ((uint32_t)(millis() - alti_time) > 10) {
    alti_original[alti_index] = analogRead(ALTI_PIN) * 2;
    alti_index = (alti_index + 1) % ALTIMETER_SAMPLES;
    for (uint8_t i = 0; i < ALTIMETER_SAMPLES; i++) alti_sample[i] = alti_original[i];
    sortAltSample(ALTIMETER_SAMPLES);
    if ((alti = getAltSampleMode(ALTIMETER_SAMPLES, true)) != getAltSampleMode(ALTIMETER_SAMPLES, false)) alti = alti_sample[ALTIMETER_SAMPLES / 2];
    payload[25] = lowByte(alti);
    payload[26] = highByte(alti);
    alti_time = millis();
  }
}

void sortAltSample(uint8_t sample_size) {
  for (int i = 1; i < sample_size; i++) {
    int j = alti_sample[i], k;
    for (k = i - 1; (k >= 0) && (j < alti_sample[k]); k--) alti_sample[k + 1] = alti_sample[k];
    alti_sample[k + 1] = j;
  }
}

uint16_t getAltSampleMode(uint8_t sample_size, boolean highest) {
  uint16_t mode = alti_sample[0];
  uint8_t mode_count = 1;
  uint8_t count = 1;
  for (int i = 1; i < sample_size; i++) {
    if (alti_sample[i] == alti_sample[i - 1]) count++;
    else count = 1;
    if (alti_sample[i] == mode) mode_count++;
    else if (!highest && count > mode_count || highest && count == mode_count) {
      mode_count = count;
      mode = alti_sample[i];
    }
  }
  return mode;
}

void restart_interrupts() {
  interrupts();
  rot_last_calc = micros();
  if (prop_waits == 0) prop_rot_last_calc = rot_last_calc;
}

void stop_interrupts() {
  if (micros() - rot_last_calc < min_tach_delta) delay((min_tach_delta - (micros() - rot_last_calc)) / 1000);
  if ((unsigned long)(micros() - rot_last_calc) < min_tach_delta) delayMicroseconds(min_tach_delta - (micros() - rot_last_calc) - 4);
  noInterrupts();
  rot_curr_calc = micros();
  tach_delta = (unsigned long)(rot_curr_calc - rot_last_calc);
  prop_delta = (unsigned long)(rot_curr_calc - prop_rot_last_calc);
}

void calculateRotations() {
  prop_waits++;
  if (prop_waits >= PROP_PROPORTION) {
    if (prop_interrupts > 5) {
      prop_rotation = (uint16_t)((60000000.0f * prop_interrupts) / (PROP_SLITS * 2) / prop_delta); //[rpm]
    }
    else {
      prop_rotation = 0;
    }
    prop_waits = 0;
    prop_interrupts = 0;
  }
  if (tach_interrupts > 5) {
    tach_rotation = (uint16_t)((100000000.0f * tach_interrupts) / TACH_SLITS / tach_delta); //[0.01rps]
  }
  else {
    tach_rotation = 0;
  }
  tach_interrupts = 0;
  // Serial.println("tac " + String(tach_rotation) + "\ttac_int " + String(tach_interrupts) + "\tprop " + String(prop_rotation) + "\tprop_int " + String(prop_interrupts) + "\tdelta " + String(delta));

  payload[1] = lowByte(prop_rotation);
  payload[2] = highByte(prop_rotation);
  payload[3] = lowByte(tach_rotation);
  payload[4] = highByte(tach_rotation);
  payload[5] = (uint8_t)(tach_delta & 0x000000FF);
  payload[6] = (uint8_t)((tach_delta & 0x0000FF00) >> 8);
  payload[7] = (uint8_t)((tach_delta & 0x00FF0000) >> 16);
  payload[8] = (uint8_t)((tach_delta & 0xFF000000) >> 24);
  payload[9] = (uint8_t)(prop_delta & 0x000000FF);
  payload[10] = (uint8_t)((prop_delta & 0x0000FF00) >> 8);
  payload[11] = (uint8_t)((prop_delta & 0x00FF0000) >> 16);
  payload[12] = (uint8_t)((prop_delta & 0xFF000000) >> 24);
}

void readServoInfo() {
  uint32_t wait_time;
  wait_time = millis();
  while (!CONTROL_SERIAL.available()) if (millis() - wait_time > 10UL) return;
  wait_time = millis();
  while ((servo_rx_packet[0] = CONTROL_SERIAL.read()) != 0x7C) if (millis() - wait_time > 25UL) return;
  wait_time = millis();
  for (uint8_t i = 1; i < 3; i++) {
    while (!CONTROL_SERIAL.available()) if ((uint32_t)(millis() - wait_time) > 3UL) return;
    servo_rx_packet[i] = CONTROL_SERIAL.read();
    wait_time = millis();
  }
  uint8_t len = servo_rx_packet[2] + 4;
  for (uint8_t i = 3; i < len; i++) {
    while (!CONTROL_SERIAL.available()) if ((uint32_t)(millis() - wait_time) > 4UL) return;
    servo_rx_packet[i] = CONTROL_SERIAL.read();
    wait_time = millis();
  }
  if (servo_rx_packet[0] == 0x7C && servo_rx_packet[1] == 0xC7 && servo_rx_packet[len - 1] == checksum(servo_rx_packet, len - 1)) return;
  for (uint8_t i = 0; i < servo_rx_packet[2]; i++) payload[i + 31] = servo_rx_packet[i + 3];
  Serial.println("Received!");
}

void transmitPayload() {
  if ((uint32_t)(millis() - last_transmit) > transmit_cooldown) {

    payload[28] = last_transmit & 0x000000FF;
    payload[29] = (last_transmit & 0x0000FF00) >> 8;
    payload[30] = (last_transmit & 0x00FF0000) >> 16;
    payload[31] = (last_transmit & 0xFF000000) >> 24;

    digitalWrite(LED_BUILTIN, HIGH);
    xbee.send(boat_packet);
    xbee.send(land_packet);

    /*Serial.print(String(payload[1] | (payload[2] << 8)));
      Serial.print("\t");
      Serial.print(String(payload[3] | (payload[4] << 8)));
      Serial.print("\t");
      Serial.print(String(((unsigned long)payload[5] | ((unsigned long)payload[6] << 8) | ((unsigned long)payload[7] << 16) | ((unsigned long)payload[8] << 24))));
      Serial.print("\t");
      Serial.print(String(((unsigned long)payload[9] | ((unsigned long)payload[10] << 8) | ((unsigned long)payload[11] << 16) | ((unsigned long)payload[12] << 24))));
      Serial.print("\tAL");
      Serial.print(String(payload[25] | (payload[26] << 8)));
      Serial.print("\tY");
      Serial.print(String(payload[13] | (payload[14] << 8)));
      Serial.print("\tR");
      Serial.print(String(payload[15] | (payload[16] << 8)));
      Serial.print("\tP");
      Serial.print(String(payload[17] | (payload[18] << 8)));
      Serial.print("\tAX");
      Serial.print("\tAY");
      Serial.print(String(payload[21] | (payload[22] << 8)));
      Serial.print("\tAZ");
      Serial.print(String(payload[23] | (payload[24] << 8)));
      Serial.print("\tCM");
      Serial.print(String(payload[27]));
      Serial.print("\tCA");
      Serial.print(String(payload[28]));
      Serial.print("\tCG");
      Serial.print(String(payload[29]));
      Serial.print("\tCS");
      Serial.print(String(payload[30]));
      Serial.print("\t");
      Serial.println();*/
    last_transmit = millis();
    digitalWrite(LED_BUILTIN, LOW);
  }
}

byte checksum(uint8_t * data, uint8_t len) {
  byte sum = 0;
  for (uint8_t i = 2; i < len; i++) sum ^= data[i];
  return sum;
}
