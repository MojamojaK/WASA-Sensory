// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!絶対読んで!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 機速計のインタラプト測定部は完全にバグっています(バグってなかったから大丈夫)
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

// 回転式インタラプト用時間管理引数

volatile uint16_t prop_interrupts = 0;      // インタラプトで演算する引数は必ず "volatile" をつけること。理由はググって。
uint32_t prop_rotation = 0;                 // [1000 interrupts per second] (1秒あたりのインタラプト回数を求めて送信。回転数は表示計の方で計算)
uint32_t prop_curr_calc = 0;
uint32_t prop_last_calc = 0;
uint32_t prop_delta = 0;

const uint32_t min_prop_delta = 500000; // 0.50ms

void prop_count_handle();
void attachPropeller();

#ifdef DEBUG_ROTATION
uint32_t loop_time_rot = 0;
#endif

void initPropeller() {
#ifdef DEBUG_ROTATION
  DEBUG_PORT.println("INIT PROPELLER");
#endif
  pinMode(PROP_PIN, INPUT_PULLUP);
  attachPropeller();
  interrupts();
  prop_last_calc = micros();
}

void attachPropeller() {
  attachInterrupt(digitalPinToInterrupt(PROP_PIN), prop_count_handle, CHANGE); // 通常CHANGE 風洞試験FALLING
}

void detachPropeller() {
  detachInterrupt(digitalPinToInterrupt(PROP_PIN));
}

void prop_count_handle() {
  prop_interrupts++;
}

void readPropeller() {
#ifdef DEBUG_ROTATION
  DEBUG_PORT.println("READ PROPELLER");
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

#ifdef DEBUG_ROTATION
  DEBUG_PORT.print("Propeller Loop Time: ");
  DEBUG_PORT.print(millis() - loop_time_rot);
  DEBUG_PORT.print("\n");
  DEBUG_PORT.flush();
#endif
}

void packPropeller(uint8_t *payload) {
  payload[1] = (uint8_t)(prop_rotation & 0x000000FF);
  payload[2] = (uint8_t)((prop_rotation & 0x0000FF00) >> 8);
  payload[3] = (uint8_t)((prop_rotation & 0x00FF0000) >> 16);
  payload[4] = (uint8_t)((prop_rotation & 0xFF000000) >> 24);
}

