// 9軸センサの制御ライブラリは以下のURLで配布しています。
// BOSCH社のライブラリを設定保存できるようにいじっています。
// Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
// https://github.com/MojamojaK/arduino-library-nine-axes-motion
// I2C アドレス 0x28 (設定によっては0x29に変えることもできます)
// (※注意)初期設定でデジタル4番ピンが使われます

#include "NineAxesMotion.h"               // 9軸センサ通信用ライブラリ
#include <Wire.h>                         // I2Cセンサ(温湿度・気圧センサと9軸センサ)用ライブラリ

#define IMU_MULTI 100

NineAxesMotion accelGyroMag;         //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 20;          //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))
unsigned long lastConfig = 0;
int16_t accel_offset[4] = { -23, 10, 5, 1000};
int16_t mag_offset[4] = {91, 255, 275, 615};
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
  DEBUG_PORT.println("READ IMU");
#endif
  if ((millis() - lastStreamTime) >= streamPeriod) {
    lastStreamTime = millis();
    heading = accelGyroMag.readEulerHeading() * IMU_MULTI;
    roll    = accelGyroMag.readEulerRoll() * IMU_MULTI;
    pitch   = accelGyroMag.readEulerPitch() * IMU_MULTI;
    accel_x = accelGyroMag.readLinearAcceleration(X_AXIS) * IMU_MULTI;
    accel_y = accelGyroMag.readLinearAcceleration(Y_AXIS) * IMU_MULTI;
    accel_z = accelGyroMag.readLinearAcceleration(Z_AXIS) * IMU_MULTI;
    calib = 0;
    calib |=  (uint8_t)accelGyroMag.readAccelCalibStatus();       // Accelerometer Calibration Status (0 - 3)
    calib |=  (uint8_t)accelGyroMag.readMagCalibStatus() << 2;    // Magnetometer Calibration Status (0 - 3)
    calib |=  (uint8_t)accelGyroMag.readGyroCalibStatus() << 4;   // Gyroscope Calibration Status (0 - 3)
    calib |=  (uint8_t)accelGyroMag.readSystemCalibStatus() << 6; // System Calibration Status (0 - 3)
    updateSensorData = true;
  }
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

