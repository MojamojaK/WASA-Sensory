// 9軸センサの制御ライブラリは以下のURLで配布しています。
// BOSCH社のライブラリを設定保存できるようにいじっています。
// Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
// https://github.com/MojamojaK/arduino-library-nine-axes-motion
// I2C アドレス 0x28 (設定によっては0x29に変えることもできます)
// (※注意)初期設定でデジタル4番ピンが使われます

#include "NineAxesMotion.h"               // 9軸センサ通信用ライブラリ
#include <Wire.h>                         // I2Cセンサ(温湿度・気圧センサと9軸センサ)用ライブラリ
#include <EEPROM.h>

#define IMU_MULTI 100

#define ID_ADDRESS 0x00; // ArduinoごとのIMUの設定を保管しておくので、

// 2018/03/10現在の磁気偏角 確認は www.magnetic-declination.com で行ってください (-7°37' = -7.67 (37/60 = 0.67))
// 時間と場所によって変わるので方角測定の際は気をつけてください
// また、強力な太陽風などの磁気嵐が来る際は極付近で30°、中緯度では約2°の誤差が出ます。
#define MAGENETIC_DECLINATION 7.67

#ifdef DEBUG_IMU
uint32_t loop_time_imu = 0;
#endif

NineAxesMotion accelGyroMag;         //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 20;          //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))
unsigned long lastConfig = 0;
int16_t accel_offset[4] = { -23, 10, 5, 1000};
int16_t mag_offset[4] = {100, 222, 279, 602};
int16_t gyro_offset[3] = {1, -2, -1};
bool updateSensorData = true;

int16_t heading = 0;
int16_t roll = 0;
int16_t pitch = 0;
int16_t accel_x = 0;
int16_t accel_y = 0;
int16_t accel_z = 0;
uint8_t calib = 0;

void initIMU() {
#ifdef DEBUG_IMU
  DEBUG_PORT.println("INIT IMU");
#endif
  // EEPROM 0x00番地 本番用センサ: 0x00 予備センサ: 0x01 (初期値:0xFF)
  uint8_t id = EEPROM.read(0x00);
#ifdef DEBUG_IMU
  DEBUG_PORT.print("Board ID: ");
  DEBUG_PORT.println(id);
#endif
  if (id == 0x00) {
    accel_offset[0] = -28;
    accel_offset[1] = -9;
    accel_offset[2] = 15;
    accel_offset[3] = 1000;
    mag_offset[0] = 139;
    mag_offset[1] = 212;
    mag_offset[2] = 299;
    mag_offset[3] = 606;
    gyro_offset[0] = 0;
    gyro_offset[1] = -1;
    gyro_offset[2] = -1;
  } else if (id == 0x01) {
    accel_offset[0] = -37;
    accel_offset[1] = 10;
    accel_offset[2] = 3;
    accel_offset[3] = 1000;
    mag_offset[0] = -172;
    mag_offset[1] = -19;
    mag_offset[2] = 263;
    mag_offset[3] = 1158;
    gyro_offset[0] = -1;
    gyro_offset[1] = -3;
    gyro_offset[2] = -1;
  }

  Wire.begin();
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
}

void updateIMU() {
  if (updateSensorData) {
    accelGyroMag.updateAccel();        // Update the Accelerometer data
    accelGyroMag.updateLinearAccel();  // Update the Linear Acceleration data
    accelGyroMag.updateGravAccel();    // Update the Gravity Acceleration data
    accelGyroMag.updateEuler();        // Update the Euler data into the structure of the object
    accelGyroMag.updateCalibStatus();  // Update the Calibration Status
    updateSensorData = false;
  }
}

void readIMU() {
#ifdef DEBUG_IMU
  loop_time_imu = millis();
  DEBUG_PORT.println("READ IMU");
#endif
  if ((millis() - lastStreamTime) >= streamPeriod) {
    lastStreamTime = millis();
    heading = accelGyroMag.readEulerHeading() - MAGENETIC_DECLINATION;
    roll    = accelGyroMag.readEulerRoll() * IMU_MULTI;
    pitch   = accelGyroMag.readEulerPitch() * IMU_MULTI;
    accel_x = -accelGyroMag.readLinearAcceleration(X_AXIS) * IMU_MULTI;
    accel_y = -accelGyroMag.readLinearAcceleration(Y_AXIS) * IMU_MULTI;
    accel_z = accelGyroMag.readLinearAcceleration(Z_AXIS) * IMU_MULTI;
    calib = 0;
    calib |=  (uint8_t)accelGyroMag.readAccelCalibStatus();       // Accelerometer Calibration Status (0 - 3)
    calib |=  (uint8_t)accelGyroMag.readMagCalibStatus() << 2;    // Magnetometer Calibration Status (0 - 3)
    calib |=  (uint8_t)accelGyroMag.readGyroCalibStatus() << 4;   // Gyroscope Calibration Status (0 - 3)
    calib |=  (uint8_t)accelGyroMag.readSystemCalibStatus() << 6; // System Calibration Status (0 - 3)
    updateSensorData = true;
  }
#ifdef DEBUG_IMU
  DEBUG_PORT.print("IMU Loop Time: ");
  DEBUG_PORT.print(millis() - loop_time_imu);
  DEBUG_PORT.print("\n");
  DEBUG_PORT.flush();
#endif
}

void packIMU(uint8_t *payload) {
  payload[9] = lowByte(heading);
  payload[10] = highByte(heading);
  payload[11] = lowByte(roll);
  payload[12] = highByte(roll);
  payload[13] = lowByte(pitch);
  payload[14] = highByte(pitch);
  payload[15] = lowByte(accel_x);
  payload[16] = highByte(accel_x);
  payload[17] = lowByte(accel_y);
  payload[18] = highByte(accel_y);
  payload[19] = lowByte(accel_z);
  payload[20] = highByte(accel_z);
  payload[21] = calib;
}



