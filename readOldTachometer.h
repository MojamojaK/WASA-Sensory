// 使いません!

#include <Timer5.h>

#define TACH_PIN 3

const uint32_t min_tach_delta = 150000;
uint32_t tach_curr_calc = 0;
uint32_t tach_last_calc = 0;
uint32_t tach_delta = 0;

volatile uint8_t last_air_speed_state;
volatile uint16_t tach_interrupts = 0;

uint32_t tach_rotation;

void attachTachometer();

void initTachometer () {
  pinMode(TACH_PIN, INPUT_PULLUP);
  startTimer5(16);
}

void attachTachometer() {
  resumeTimer5();
}

void detachTachometer() {
  pauseTimer5();
}

void tach_count_handle() {
  tach_interrupts++;
}

void readTachometer() {
  if ((uint32_t)(micros() - tach_last_calc) > min_tach_delta) {
    detachTachometer();
    tach_delta = (tach_curr_calc = micros()) - tach_last_calc;
    if (tach_interrupts > 5) {
      tach_rotation = (uint32_t)((double)1000000000.0 * ((double)tach_interrupts / tach_delta)); //[1000 interrupts per second]
    }
    else {
      tach_rotation = 0;
    }

#ifdef DEBUG_TACHOMETER
    DEBUG_PORT.print("Tach: ints:");
    DEBUG_PORT.print(tach_interrupts);
    DEBUG_PORT.print(" period: ");
    DEBUG_PORT.print(tach_delta / tach_interrupts);
#endif
    tach_interrupts = 0;
    tach_last_calc = micros();
    attachTachometer();
#ifdef DEBUG_TACHOMETER
    DEBUG_PORT.print(" delta_t: ");
    DEBUG_PORT.print(tach_delta);
    DEBUG_PORT.print(" i/s: ");
    DEBUG_PORT.println(tach_rotation);
#endif
  }
}

void packTachometer(uint8_t *payload) {
  payload[5] = (uint8_t)(tach_rotation & 0x000000FF);
  payload[6] = (uint8_t)((tach_rotation & 0x0000FF00) >> 8);
  payload[7] = (uint8_t)((tach_rotation & 0x00FF0000) >> 16);
  payload[8] = (uint8_t)((tach_rotation & 0xFF000000) >> 24);
}

