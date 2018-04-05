// GPS: green=RX white=TX
#include <TinyGPS++.h>

#define DEBUG_GPS_UPDATE

#define GPS_SERIAL Serial1                // GPS通信用シリアル

// GPSモジュールで最低限 GGA, RMC, GST メッセージの送信を有効にしてください

TinyGPSPlus gps;
TinyGPSCustom latitudeError(gps, "GNGST", 6);
TinyGPSCustom longitudeError(gps, "GNGST", 7);

uint8_t   valid = 0;
uint32_t  latitude = 359752780;
uint32_t  longitude = 1395238890;
uint16_t  lat_err = 1000;
uint16_t  lng_err = 1000;
uint16_t  groundSpeed = 0;
uint8_t   gpsTimeHour = 0;
uint8_t   gpsTimeMinute = 0;
uint8_t   gpsTimeSec = 0;
uint32_t  gpsAltitude = 0;
uint8_t   satelliteCount = 0;
uint16_t  hdopValue = 9999;
uint16_t  gpsCourse = 0;

char c;
int m_count = 0;
uint32_t  start = 0;

void initGPS(void) {
#ifdef DEBUG_GPS
  DEBUG_PORT.println("INIT GPS");
#endif
  GPS_SERIAL.begin(9600);
  GPS_SERIAL.flush();
}

void readGPS(void) {
#ifdef DEBUG_GPS
  DEBUG_PORT.println("READ GPS");
  DEBUG_PORT.flush();
#endif
  start = millis();
  do {
    while (GPS_SERIAL.available()) {
      c = GPS_SERIAL.read();
      gps.encode(c);
    }
  } while (millis() - start < 10);
}

void parseGPS(void) {
  if (gps.location.isUpdated() && gps.location.isValid()) {
    latitude = (uint32_t)(gps.location.lat() * 10000000); // 10000000倍
    longitude = (uint32_t)(gps.location.lng() * 10000000); // 10000000倍
#ifdef DEBUG_GPS && DEBUG_GPS_UPDATE
    DEBUG_PORT.print("LOCATION VALID (");
    DEBUG_PORT.print(latitude);
    DEBUG_PORT.print(", ");
    DEBUG_PORT.print(longitude);
    DEBUG_PORT.println(")");
#endif
  }
  if (gps.satellites.isUpdated() && gps.satellites.isValid()) {
    satelliteCount = (uint8_t)gps.satellites.value();
#ifdef DEBUG_GPS && DEBUG_GPS_UPDATE
    DEBUG_PORT.print("SATELLITES VALID (");
    DEBUG_PORT.print(satelliteCount);
    DEBUG_PORT.println(")");
#endif
  }
  if (gps.hdop.isUpdated() && gps.hdop.isValid()) {
    hdopValue = (uint16_t)gps.hdop.value(); // 水平精度 100倍
#ifdef DEBUG_GPS && DEBUG_GPS_UPDATE
    DEBUG_PORT.print("HDOP VALID (");
    DEBUG_PORT.print(hdopValue);
    DEBUG_PORT.println(")");
#endif
  }
  if (gps.time.isUpdated() && gps.time.isValid()) {
    gpsTimeHour = (uint8_t)gps.time.hour();
    gpsTimeMinute = (uint8_t)gps.time.minute();
    gpsTimeSec = (uint8_t)gps.time.second();
#ifdef DEBUG_GPS && DEBUG_GPS_UPDATE
    DEBUG_PORT.print("TIME VALID (");
    DEBUG_PORT.print(gpsTimeHour);
    DEBUG_PORT.print(":");
    DEBUG_PORT.print(gpsTimeMinute);
    DEBUG_PORT.print(":");
    DEBUG_PORT.print(gpsTimeSec);
    DEBUG_PORT.println(")");
#endif
  }
  if (gps.course.isUpdated() && gps.course.isValid()) {
    gpsCourse = (uint16_t)gps.course.value(); // 進行方向(deg) 100倍
#ifdef DEBUG_GPS && DEBUG_GPS_UPDATE
    DEBUG_PORT.print("COURSE VALID (");
    DEBUG_PORT.print(gpsCourse);
    DEBUG_PORT.println(")");
#endif
  }
  if (gps.altitude.isUpdated() && gps.altitude.isValid()) {
    gpsAltitude = (uint32_t)gps.altitude.value(); // 高度 センチ単位
#ifdef DEBUG_GPS && DEBUG_GPS_UPDATE
    DEBUG_PORT.print("ALTITUDE VALID (");
    DEBUG_PORT.print(gpsAltitude);
    DEBUG_PORT.println(")");
#endif
  }
  if (gps.speed.isUpdated() && gps.speed.isValid()) {
    groundSpeed = (uint16_t)(gps.speed.mps() * 1000); // 対地速度(m/s) 1000倍
#ifdef DEBUG_GPS && DEBUG_GPS_UPDATE
    DEBUG_PORT.print("SPEED VALID (");
    DEBUG_PORT.print(groundSpeed);
    DEBUG_PORT.println(")");
#endif
  }
  if (latitudeError.isUpdated() && latitudeError.isValid()) {
    lat_err = (uint16_t)latitudeError.value();
#ifdef DEBUG_GPS && DEBUG_GPS_UPDATE
    DEBUG_PORT.print("LAT_ERR VALID (");
    DEBUG_PORT.print(lat_err);
    DEBUG_PORT.println(")");
#endif
  }
  if (longitudeError.isUpdated() && longitudeError.isValid()) {
    lng_err = (uint16_t)longitudeError.value();
#ifdef DEBUG_GPS && DEBUG_GPS_UPDATE
    DEBUG_PORT.print("LNG_ERR VALID (");
    DEBUG_PORT.print(lng_err);
    DEBUG_PORT.println(")");
#endif
  }
}

void packGPS(uint8_t *payload) {
  parseGPS();
  payload[24] = (uint8_t) (longitude & 0x000000FF);
  payload[25] = (uint8_t)((longitude & 0x0000FF00) >> 8);
  payload[26] = (uint8_t)((longitude & 0x00FF0000) >> 16);
  payload[27] = (uint8_t)((longitude & 0xFF000000) >> 24);
  payload[28] = (uint8_t) (latitude & 0x000000FF);
  payload[29] = (uint8_t)((latitude & 0x0000FF00) >> 8);
  payload[30] = (uint8_t)((latitude & 0x00FF0000) >> 16);
  payload[31] = (uint8_t)((latitude & 0xFF000000) >> 24);
  payload[32] = (uint8_t) (groundSpeed & 0x00FF);
  payload[33] = (uint8_t)((groundSpeed & 0xFF00) >> 8);
  payload[34] = (uint8_t) gpsTimeHour;
  payload[35] = (uint8_t) gpsTimeMinute;
  payload[36] = (uint8_t) gpsTimeSec;
  payload[37] = (uint8_t) (gpsAltitude & 0x000000FF);
  payload[38] = (uint8_t)((gpsAltitude & 0x0000FF00) >> 8);
  payload[39] = (uint8_t)((gpsAltitude & 0x00FF0000) >> 16);
  payload[40] = (uint8_t)((gpsAltitude & 0xFF000000) >> 24);
  payload[41] = (uint8_t)satelliteCount;
  payload[42] = (uint8_t) (hdopValue & 0x000000FF);
  payload[43] = (uint8_t)((hdopValue & 0x0000FF00) >> 8);
  payload[44] = (uint8_t)((hdopValue & 0x00FF0000) >> 16);
  payload[45] = (uint8_t)((hdopValue & 0xFF000000) >> 24);
  payload[46] = (uint8_t) (gpsCourse & 0x000000FF);
  payload[47] = (uint8_t)((gpsCourse & 0x0000FF00) >> 8);

#ifdef DEBUG_GPS
  DEBUG_PORT.println("==============================");
  DEBUG_PORT.print("latitude: ");
  DEBUG_PORT.println(latitude);
  DEBUG_PORT.print("longitude: ");
  DEBUG_PORT.println(longitude);
  DEBUG_PORT.print("lat_err: ");
  DEBUG_PORT.println(lat_err);
  DEBUG_PORT.print("lng_err: ");
  DEBUG_PORT.println(lng_err);
  DEBUG_PORT.print("groundSpeed: ");
  DEBUG_PORT.println(groundSpeed);
  DEBUG_PORT.print("Time: ");
  DEBUG_PORT.print(gpsTimeHour);
  DEBUG_PORT.print(":");
  DEBUG_PORT.print(gpsTimeMinute);
  DEBUG_PORT.print(":");
  DEBUG_PORT.println(gpsTimeSec);
  DEBUG_PORT.print("gpsAltitude: ");
  DEBUG_PORT.println(gpsAltitude);
  DEBUG_PORT.print("satellites: ");
  DEBUG_PORT.println(satelliteCount);
  DEBUG_PORT.print("hdop: ");
  DEBUG_PORT.println(hdopValue);
  DEBUG_PORT.print("gpsCourse: ");
  DEBUG_PORT.println(gpsCourse);
#endif
}

