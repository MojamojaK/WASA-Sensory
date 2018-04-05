// デバッグ関係のものをいじるところです

//
// #define DEBUG  // シリアルデバッグを有効にするにはこの行のコメントアウトを外して下さい
// デバッグ有効時はxbee出力が自動的に無効にされるのでご注意

#ifdef DEBUG
//#define DEBUG_ANDROID
#define DEBUG_ALTI
//#define DEBUG_CONTROL
//#define DEBUG_GPS
//#define DEBUG_IMU
//#define DEBUG_ROTATION
//#define DEBUG_THP
#endif

#define DEBUG_PORT Serial  // デバッグ情報を出力するポートを指定できます。

#define DEBUG_LED 8

void initDebug() {
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
}


