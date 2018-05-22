// 使いません!

#define PROP_PIN 2          // プロペラ回転数計インタラプト専用ピン
#define TACH_PIN 3          // 機速計インタラプト専用ピン

// 回転式インタラプト用時間管理引数
unsigned long rot_last_calc = 0;
unsigned long rot_curr_calc = 0;
unsigned long tach_delta = 0;
const unsigned long min_tach_delta = 150000; // 0.15ms

volatile uint16_t prop_interrupts = 0;
uint32_t prop_rotation = 0;                 // [1000 interrupts per second] (1秒あたりのインタラプト回数を求めて送信。回転数は表示計の方で計算)
#define PROP_PROPORTION 2  //通常2 風洞試験1  // prop_proportion回 機速計の計測に対して1回回転数計の計測を行う (0.15ms * 2 = 0.30ms)
uint8_t prop_waits = 0;                     // 機速計の計測を待った回数
unsigned long prop_rot_last_calc = 0;
unsigned long prop_delta = 0;

volatile uint16_t tach_interrupts = 0;    // インタラプトで演算する引数は必ず "volatile" をつけること。理由はググって。
uint32_t tach_rotation = 0;               // [1000 interrupts per second]  (1秒あたりのインタラプト回数を求めて送信。機速は表示計の方で計算)

void prop_count_handle();
void tach_count_handle();

void initRotations() {
#ifdef DEBUG_ROTATION
  DEBUG_PORT.println("INIT ROTATION");
#endif
  pinMode(PROP_PIN, INPUT_PULLUP);
  pinMode(TACH_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PROP_PIN), prop_count_handle, CHANGE); // 通常CHANGE 風洞試験FALLING
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tach_count_handle, FALLING);
  interrupts();
  rot_last_calc = micros();
  prop_rot_last_calc = rot_last_calc;
}

void prop_count_handle() {
  prop_interrupts++;
}

void tach_count_handle() {
  tach_interrupts++;
}

void restartInterrupts() {
  interrupts();
  rot_last_calc = micros();
  if (prop_waits == 0) prop_rot_last_calc = rot_last_calc;
}

void stopInterrupts() {
  if (micros() - rot_last_calc < min_tach_delta) delay((min_tach_delta - (micros() - rot_last_calc)) / 1000);
  if ((unsigned long)(micros() - rot_last_calc) < min_tach_delta) delayMicroseconds(min_tach_delta - (micros() - rot_last_calc) - 4);
  noInterrupts();
  rot_curr_calc = micros();
  tach_delta = (unsigned long)(rot_curr_calc - rot_last_calc);
  prop_delta = (unsigned long)(rot_curr_calc - prop_rot_last_calc);
}

void readRotations() {
  stopInterrupts();
#ifdef DEBUG_ROTATION
  DEBUG_PORT.println("READ ROTATION");
#endif
  prop_waits++;
  if (prop_waits >= PROP_PROPORTION) {
    if (prop_interrupts > 5) {
      prop_rotation = (uint32_t)((double)1000000000.0 * ((double)prop_interrupts / prop_delta)); //[1000 interrupts per second]
    }
    else {
      prop_rotation = 0;
    }
    prop_waits = 0;
    prop_interrupts = 0;
#ifdef DEBUG_ROTATION
    DEBUG_PORT.print("Prop: ints:");
    DEBUG_PORT.print(prop_interrupts);
    DEBUG_PORT.print(" period: ");
    DEBUG_PORT.print(prop_delta / prop_interrupts);
    DEBUG_PORT.print(" delta_t: ");
    DEBUG_PORT.print(prop_delta);
    DEBUG_PORT.print(" i/s: ");
    DEBUG_PORT.println(prop_rotation);
#endif
  }
  if (tach_interrupts > 5) {
    tach_rotation = (uint32_t)((double)1000000000.0 * ((double)tach_interrupts / tach_delta)); //[1000 interrupts per second]
  }
  else {
    tach_rotation = 0;
  }
#ifdef DEBUG_ROTATION
  DEBUG_PORT.print("Tach: ints:");
  DEBUG_PORT.print(tach_interrupts);
  DEBUG_PORT.print(" period: ");
  DEBUG_PORT.print(tach_delta / tach_interrupts);
  DEBUG_PORT.print(" delta_t: ");
  DEBUG_PORT.print(tach_delta);
  DEBUG_PORT.print(" i/s: ");
  DEBUG_PORT.println(tach_rotation);
#endif
  tach_interrupts = 0;
  restartInterrupts();
}

void packRotations(uint8_t *payload) {
  payload[1] = (uint8_t)(prop_rotation & 0x000000FF);
  payload[2] = (uint8_t)((prop_rotation & 0x0000FF00) >> 8);
  payload[3] = (uint8_t)((prop_rotation & 0x00FF0000) >> 16);
  payload[4] = (uint8_t)((prop_rotation & 0xFF000000) >> 24);
  payload[5] = (uint8_t)(tach_rotation & 0x000000FF);
  payload[6] = (uint8_t)((tach_rotation & 0x0000FF00) >> 8);
  payload[7] = (uint8_t)((tach_rotation & 0x00FF0000) >> 16);
  payload[8] = (uint8_t)((tach_rotation & 0xFF000000) >> 24);
}
