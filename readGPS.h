// GPS: green=RX white=TX
#include <NMEAGPS.h>

#define GPS_SERIAL Serial1                // GPS通信用シリアル

NMEAGPS gps;
gps_fix fix;

uint8_t   valid = 0;
uint32_t  longitude = 139523889;
uint32_t  latitude = 35975278;
uint16_t  lat_err = 1000;
uint16_t  lng_err = 1000;
uint16_t  groundSpeed = 0;
uint8_t   gpsTimeHour = 0;
uint8_t   gpsTimeMinute = 0;
uint8_t   gpsTimeSec = 0;
uint32_t  gpsAltitude = 0;
uint8_t   satellites = 0;
uint16_t  hdop = 9999;
uint16_t  gpsCourse = 0;
uint32_t  gpsLastReadTime = 0;
uint32_t  gpsLastParseTime = 0;

void initGPS(void) {
#ifdef DEBUG
  DEBUG_PORT.println("INIT GPS");
#endif
  GPS_SERIAL.begin(9600);
  GPS_SERIAL.flush();
}

void parseGPS(void) {
  if (fix.valid.location) {
    longitude       = (uint32_t)fix.longitudeL();    // 緯度(1000倍)         (uint32_t)
    latitude        = (uint32_t)fix.latitudeL();     // 経度(1000倍)         (uint32_t)
  }
  if (fix.valid.lon_err) {
    lat_err = (uint16_t)fix.lon_err_cm;
  }
  if (fix.valid.lat_err) {
    lat_err = (uint16_t)fix.lat_err_cm;
  }
  if (fix.valid.speed) {
    groundSpeed     = (uint16_t)((fix.spd.whole * 1000 + fix.spd.frac) * 0.5144447);  // 時間：1000メートル/秒  (uint16_t)
  }
  if (fix.valid.time) {
    gpsTimeHour     = fix.dateTime.hours;                          // 時間：時間            (uint8_t)
    gpsTimeMinute   = fix.dateTime.minutes;                        // 時間：分              (uint8_t)
    gpsTimeSec      = fix.dateTime.seconds;                        // 時間：秒              (uint8_t)
  }
  if (fix.valid.altitude) {
    gpsAltitude     = (uint32_t)(fix.altitude_cm());              // cm単位の高度（不正確）  (uint16_t)
  }
  if (fix.valid.satellites) {
    satellites      = (uint8_t)fix.satellites;                  // 衛星の数              (uint8_t)
  }
  if (fix.valid.hdop) {
    hdop            = (uint16_t)fix.hdop;                        // 精度(1000倍)          (uint16_t)
  }
  if (fix.valid.heading) {
    gpsCourse       = (uint16_t)fix.heading_cd();                 // 進行方角(100倍)       (uint16_t)
  }
}

void readGPS(void) {
#ifdef DEBUG
  DEBUG_PORT.println("READ GPS");
#endif
  gpsLastReadTime = millis();
  while (gps.available(GPS_SERIAL) && millis() - gpsLastReadTime < 50) {
    fix = gps.read();
    parseGPS();
  }
}

// TODO
void packGPS(uint8_t *payload) {
  payload[20] = (uint8_t) (longitude & 0x000000FF);
  payload[21] = (uint8_t)((longitude & 0x0000FF00) >> 8);
  payload[22] = (uint8_t)((longitude & 0x00FF0000) >> 16);
  payload[23] = (uint8_t)((longitude & 0xFF000000) >> 24);
  payload[24] = (uint8_t) (latitude & 0x000000FF);
  payload[25] = (uint8_t)((latitude & 0x0000FF00) >> 8);
  payload[26] = (uint8_t)((latitude & 0x00FF0000) >> 16);
  payload[27] = (uint8_t)((latitude & 0xFF000000) >> 24);
  payload[28] = (uint8_t) (groundSpeed & 0x00FF);
  payload[29] = (uint8_t)((groundSpeed & 0xFF00) >> 8);
  payload[30] = (uint8_t) gpsTimeHour;
  payload[31] = (uint8_t) gpsTimeMinute;
  payload[32] = (uint8_t) gpsTimeSec;
  payload[33] = (uint8_t) (gpsAltitude & 0x000000FF);
  payload[34] = (uint8_t)((gpsAltitude & 0x0000FF00) >> 8);
  payload[35] = (uint8_t)((gpsAltitude & 0x00FF0000) >> 16);
  payload[36] = (uint8_t)((gpsAltitude & 0xFF000000) >> 24);
  payload[37] = (uint8_t)satellites;
  payload[38] = (uint8_t) (hdop & 0x000000FF);
  payload[39] = (uint8_t)((hdop & 0x0000FF00) >> 8);
  payload[40] = (uint8_t)((hdop & 0x00FF0000) >> 16);
  payload[41] = (uint8_t)((hdop & 0xFF000000) >> 24);
  payload[42] = (uint8_t) (gpsCourse & 0x000000FF);
  payload[43] = (uint8_t)((gpsCourse & 0x0000FF00) >> 8);

}

