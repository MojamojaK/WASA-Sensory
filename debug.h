// デバッグ関係のものをいじるところです

//#define DEBUG  // シリアルデバッグを有効にするにはこの行のコメントアウトを外して下さい
// デバッグ有効時はxbee出力が自動的に無効にされるのでご注意
// さらに、デバッガーによるシリアルの占有率が上がるのでxbee出力の頻度は一気に落ちます

// 以下でデバッグ情報を得たい項目のコメントアウトを外して下さい
#ifdef DEBUG
//#define DEBUG_ANDROID
//#define DEBUG_ALTI
//#define DEBUG_CONTROL
//#define DEBUG_GPS
//#define DEBUG_IMU
//#define DEBUG_ROTATION
//#define DEBUG_TACHOMETER
//#define DEBUG_THP
//#define DEBUG_XBEE
#endif

#define DEBUG_PORT Serial  // デバッグ情報を出力するポートを指定できます。

#define DEBUG_LED 8

void initDebug() {
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
}


void flashDebug(int times, int wait) {

  for (int i = 0; i < times; i++) {
    digitalWrite(DEBUG_LED, HIGH);
    delay(wait);
    digitalWrite(DEBUG_LED, LOW);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}
