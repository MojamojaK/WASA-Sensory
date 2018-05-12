#include <TimerOne.h> // 内部タイマー用ライブラリ/ライブラリマネージャから入手できます
// Timer1 はピン 11, 12を占有します。これらのピンではPWMによるanalogWriteができなくなります。

#define ALTI_PIN A0         // 高度用アナログピン

volatile uint8_t alti_curr_pulse = LOW;
volatile uint8_t alti_last_time_pulse = LOW;

uint32_t alti_last_time = 0;
volatile uint16_t alti_count = 0;
volatile uint16_t alti_tmp = 0;
uint16_t alti = 0;

#ifdef DEBUG_ALTI
uint32_t loop_time_alti = 0;
#endif

void calcAltitude();
void attachAltitude();

void initAltitude() {
#ifdef DEBUG_ALTI
  DEBUG_PORT.println("INIT ALTITUDE");
#endif
  pinMode(ALTI_PIN, INPUT);
  Timer1.initialize(58); // 58usごと
  attachAltitude();
}

void attachAltitude() {
  uint32_t t = millis();
  while (digitalRead(ALTI_PIN) == HIGH && (uint8_t)(millis() - t) < 62){}
  Timer1.attachInterrupt(calcAltitude);
}

void detachAltitude() {
  Timer1.detachInterrupt();
}

void calcAltitude() {
  alti_curr_pulse = digitalRead(ALTI_PIN);
  if (alti_curr_pulse == HIGH) { // HIGH
    alti_count++;
  } else if (alti_last_time_pulse == HIGH){ // HIGH -> LOW
    alti_tmp = alti_count;
    alti_count = 0;
  }
  alti_last_time_pulse = alti_curr_pulse;
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
  loop_time_alti = millis();
#endif
  if ((uint32_t)(millis() - alti_last_time) > 90) {
    // アナログ読取り(精度低いけど速い)
    // uint16_t newValue = analogRead(ALTI_PIN) * 2;
    // digitalWrite(DEBUG_LED, HIGH);
    // uint16_t diff = abs(newValue - alti);
    // double snap = snapCurve(diff * 0.01); // SNAP MULTIPLIER 0.01
    // alti += (newValue - alti) * snap;
    // digitalWrite(DEBUG_LED, LOW);

    // パルス幅読取り(精度高いけど遅い(1回の測定に最高124msの時間がかかる))
    // uint16_t newValue = pulseIn(ALTI_PIN, HIGH, 124000) / 58.0;
    // if (newValue > 0) {
    //  uint16_t diff = abs(newValue - alti);
    //  double snap = snapCurve(diff * 0.5); // SNAP MULTIPLIER 0.5 (計測間隔が大きいので)
    //  alti += (newValue - alti) * snap;
    // }
    
    detachAltitude();
    uint16_t diff = alti_tmp - alti;
    attachAltitude();
    double snap = snapCurve(abs(diff) * 0.5); // SNAP MULTIPLIER 0.5 (計測間隔が大きいので)
    alti += diff * snap;
    
#ifdef DEBUG_ALTI
    DEBUG_PORT.print("READ ALTITUDE: ");
    DEBUG_PORT.println(alti);
#endif
    alti_last_time = millis();
  }
#ifdef DEBUG_ALTI
  DEBUG_PORT.print("Altitude Loop Time: ");
  DEBUG_PORT.print(millis() - loop_time_alti);
  DEBUG_PORT.print("\n");
  DEBUG_PORT.flush();
#endif
}

void packAltitude(uint8_t *payload) {
  payload[22] = lowByte(alti);
  payload[23] = highByte(alti);
}


