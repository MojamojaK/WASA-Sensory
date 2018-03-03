// デバッグ関係のものをいじるところです

// #define DEBUG  // シリアルデバッグを有効にするにはこの行のコメントアウトを外して下さい
// デバッグ有効時はxbee出力が自動的に無効にされるのでご注意

#define DEBUG_PORT Serial  // デバッグ情報を出力するポートを指定できます。

#define DEBUG_LED LED_BUILTIN

void initDebug() {
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
}

