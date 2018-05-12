// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!絶対読んで!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 機速計のインタラプト測定部は完全にバグっています
// 500マイクロ秒ごとにインタラプト起こしても600マイクロ秒ごとしか検知されていません
// 風洞試験を実施してから気づいたことなので(値が取れないので)修正はしませんが、機速に影響してくるので次年度以降は必ず修正してください
// 一度32代のソースコードで修正された問題ですが、うまく引き継がれなかったため発生しました。
// 32代のソースコードはドライブ(toridenso@gmail.com)にあるのでそれを参考にTimerとかをつかって修正してください
// なお、32代のソースコードで使われてるTimer1はすでに高度計で使用されているため、ここでは使用できません。
// Timer3(使えないかも), Timer4, Timer5(一番オススメ)をつかって修正してください
// https://oscarliang.com/arduino-timer-and-interrupt-tutorial/
// タイマーライブラリはこちら https://github.com/carlosrafaelgn/ArduinoTimer/
// Timer5: https://github.com/carlosrafaelgn/ArduinoTimer/blob/master/Timer5.zip?raw=true
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!絶対読んで!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// Arduino MEGA, MEGA2560, MEGA ADK ではピン番号2, 3, 18, 19, 20, 21がインタラプトとして使用できます
#define PROP_PIN 2          // プロペラ回転数計インタラプト専用ピン
#define TACH_PIN 3          // 機速計インタラプト専用ピン

// 回転式インタラプト用時間管理引数

volatile uint16_t prop_interrupts = 0;      // インタラプトで演算する引数は必ず "volatile" をつけること。理由はググって。
uint32_t prop_rotation = 0;                 // [1000 interrupts per second] (1秒あたりのインタラプト回数を求めて送信。回転数は表示計の方で計算)
uint32_t prop_curr_calc = 0;
uint32_t prop_last_calc = 0;
uint32_t prop_delta = 0;

const uint32_t min_prop_delta = 500000; // 0.50ms

volatile uint16_t tach_interrupts = 0;    // インタラプトで演算する引数は必ず "volatile" をつけること。理由はググって。
uint32_t tach_rotation = 0;               // [1000 interrupts per second]  (1秒あたりのインタラプト回数を求めて送信。機速は表示計の方で計算)
uint32_t tach_curr_calc = 0;
uint32_t tach_last_calc = 0;
uint32_t tach_delta = 0;

const uint32_t min_tach_delta = 150000; // 0.15ms

void prop_count_handle();
void tach_count_handle();
void attachPropeller();
void attachTachometer();
void detachPropeller();
void detachTachometer();

#ifdef DEBUG_ROTATION
uint32_t loop_time_rot = 0;
#endif

void initRotations() {
#ifdef DEBUG_ROTATION
  DEBUG_PORT.println("INIT ROTATION");
#endif
  pinMode(PROP_PIN, INPUT_PULLUP);
  pinMode(TACH_PIN, INPUT_PULLUP);

  attachPropeller();
  attachTachometer();

  interrupts();
  prop_last_calc = micros();
  tach_last_calc = prop_last_calc;
}

void attachPropeller() {
  attachInterrupt(digitalPinToInterrupt(PROP_PIN), prop_count_handle, CHANGE); // 通常CHANGE 風洞試験FALLING
}

void attachTachometer() {
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tach_count_handle, FALLING);
}

void detachPropeller() {
  detachInterrupt(digitalPinToInterrupt(PROP_PIN));
}

void detachTachometer() {
  detachInterrupt(digitalPinToInterrupt(TACH_PIN));
}

void prop_count_handle() {
  prop_interrupts++;
}

void tach_count_handle() {
  tach_interrupts++;
}

void readRotations() {
#ifdef DEBUG_ROTATION
  DEBUG_PORT.println("READ ROTATION");
  loop_time_rot = millis();
#endif
  if ((uint32_t)(micros() - prop_last_calc) > min_prop_delta) {
    detachPropeller();
    prop_delta = (prop_curr_calc = micros()) - prop_last_calc;
    if (prop_interrupts > 5) {
      prop_rotation = (uint32_t)((double)1000000000.0 * ((double)prop_interrupts / prop_delta)); //[1000 interrupts per second]
    } else {
      prop_rotation = 0;
    }
#ifdef DEBUG_ROTATION
    DEBUG_PORT.print("Prop: ints:");
    DEBUG_PORT.print(prop_interrupts);
    DEBUG_PORT.print(" period: ");
    DEBUG_PORT.print(prop_delta / prop_interrupts);
#endif
    prop_interrupts = 0;
    prop_last_calc = micros();
    attachPropeller();
#ifdef DEBUG_ROTATION
    DEBUG_PORT.print(" delta_t: ");
    DEBUG_PORT.print(prop_delta);
    DEBUG_PORT.print(" i/s: ");
    DEBUG_PORT.println(prop_rotation);
#endif
  }

  if ((uint32_t)(micros() - tach_last_calc) > min_tach_delta) {
    detachTachometer();
    tach_delta = (tach_curr_calc = micros()) - tach_last_calc;
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
#endif
    tach_interrupts = 0;
    tach_last_calc = micros();
    attachTachometer();
#ifdef DEBUG_ROTATION
    DEBUG_PORT.print(" delta_t: ");
    DEBUG_PORT.print(tach_delta);
    DEBUG_PORT.print(" i/s: ");
    DEBUG_PORT.println(tach_rotation);
#endif
  }

#ifdef DEBUG_ROTATION
  DEBUG_PORT.print("Rotation Loop Time: ");
  DEBUG_PORT.print(millis() - loop_time_rot);
  DEBUG_PORT.print("\n");
  DEBUG_PORT.flush();
#endif
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

