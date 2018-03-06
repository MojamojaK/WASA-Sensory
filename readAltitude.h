#define ALTI_PIN A0         // 高度用アナログピン

#define SNAP_MULTIPLIER 0.01
uint32_t alti_time = 0;
uint16_t alti = 0;

void initAltitude() {
#ifdef DEBUG_ALTI
  DEBUG_PORT.println("INIT ALTITUDE");
#endif
  pinMode(ALTI_PIN, INPUT);
}

// 高度から誤差を削除してうまく表示するための関数
// スナップ曲線を用いる
// http://damienclarke.me/code/posts/writing-a-better-noise-reducing-analogread
double snapCurve(uint16_t x) {
  double y = 1 / ((double)x + 1);
  y = (1 - y) * 2;
  if (y > 1) {
    return 1.0;
  }
  return y;
}

void readAltitude() {
#ifdef DEBUG_ALTI
  DEBUG_PORT.println("READ ALTITUDE");
#endif
  if ((uint32_t)(millis() - alti_time) > 10) {
    uint16_t newValue = analogRead(ALTI_PIN) * 2;
    uint16_t diff = abs(newValue - alti);
    double snap = snapCurve(diff * SNAP_MULTIPLIER);
    alti += (newValue - alti) * snap;
    alti_time = millis();
  }
}

void packAltitude(uint8_t *payload) {
  payload[22] = lowByte(alti);
  payload[23] = highByte(alti);
}

