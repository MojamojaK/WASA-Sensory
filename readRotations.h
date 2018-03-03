#define PROP_PIN 2          // プロペラ回転数計インタラプト専用ピン
#define TACH_PIN 3          // 機速計インタラプト専用ピン

#define PROP_SLITS 32       // 回転数計スリット数
#define TACH_SLITS 100      // ロータリーエンコーダの分解能
#define TACH_COEFF 10       // [0.01回転/秒]に掛けて[m/s]を得るための係数

// 回転式インタラプト用時間管理引数
unsigned long rot_last_calc = 0;
unsigned long rot_curr_calc = 0;
unsigned long tach_delta = 0;
const unsigned long min_tach_delta = 150000;

volatile uint16_t prop_interrupts = 0;
uint16_t prop_rotation = 0;               //[回転/分]
#define PROP_PROPORTION 4                // prop_proportion回 機速計の計測に対して1回回転数計の計測を行う
uint8_t prop_waits = 0;                   // 機速計の計測を待った回数
unsigned long prop_rot_last_calc = 0;
unsigned long prop_delta = 0;

volatile uint16_t tach_interrupts = 0;
uint16_t tach_rotation = 0; //[0.01回転/秒]

void prop_count_handle();
void tach_count_handle();

void initRotations() {
  pinMode(PROP_PIN, INPUT_PULLUP);
  pinMode(TACH_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PROP_PIN), prop_count_handle, CHANGE);
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
  prop_waits++;
  if (prop_waits >= PROP_PROPORTION) {
    if (prop_interrupts > 5) {
      prop_rotation = (uint16_t)((60000000.0f * prop_interrupts) / (PROP_SLITS * 2) / prop_delta); //[rpm]
    }
    else {
      prop_rotation = 0;
    }
    prop_waits = 0;
    prop_interrupts = 0;
  }
  if (tach_interrupts > 5) {
    tach_rotation = (uint16_t)((100000000.0f * tach_interrupts) / TACH_SLITS / tach_delta); //[0.01rps]
  }
  else {
    tach_rotation = 0;
  }
  tach_interrupts = 0;
  // Serial.println("tac " + String(tach_rotation) + "\ttac_int " + String(tach_interrupts) + "\tprop " + String(prop_rotation) + "\tprop_int " + String(prop_interrupts) + "\tdelta " + String(delta)); 
}

void packRotations(uint8_t *payload) {
  payload[1] = (uint8_t)(prop_rotation & 0x00FF);
  payload[2] = (uint8_t)((prop_rotation & 0xFF00) >> 8);
  payload[3] = (uint8_t)(tach_rotation & 0x00FF);
  payload[4] = (uint8_t)((tach_rotation & 0xFF00) >> 8);
}
