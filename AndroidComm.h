// http://blog.srichakram.in/2015/09/arduino-mega-adk-and-android.html

#include <adk.h>             // Android通信用ライブラリ
#include <SPI.h>

#define ANDROID_DATA_HEAD 4
#define ANDROID_PAYLOAD_SIZE (PAYLOAD_SIZE + ANDROID_DATA_HEAD + 1)
#define ANDROID_CHECKSUM_BYTE (ANDROID_PAYLOAD_SIZE - 1)

USB Usb;
ADK android(&Usb,
            "WASA", // 製造者名
            "ArduinoADK", // 型番
            "WASA HPA Sensory Hardware", // 説明文
            "0.0.2", // バージョン
            "www.wasa-birdman.com", // URL
            "0000000114514810"); // シリアル番号

boolean USBInit = false;
boolean androidConnected = false;
uint32_t accTransmitLastTime = 0;
uint8_t androidPayload[ANDROID_PAYLOAD_SIZE];

#define RECEIVE_BUFFER_SIZE 128
uint8_t receiveBuffer[RECEIVE_BUFFER_SIZE];

uint8_t android_checksum(void);

#ifdef DEBUG_ANDROID
uint32_t loop_time_android = 0;
#endif

void initAccessory (void) {
#ifdef DEBUG_ANDROID
  DEBUG_PORT.print("INIT ANDROID");
#endif
  if (Usb.Init() != -1) {
    USBInit = true;
  }
  accTransmitLastTime = millis();
#ifdef DEBUG_ANDROID
  DEBUG_PORT.println(" COMPLETE");
#endif
}

void accessory(uint8_t *payload) {
#ifdef DEBUG_ANDROID
  loop_time_android = millis();
#endif
  if (USBInit) {
    Usb.Task();

    if (android.isReady()) {
      if (!androidConnected) {
        androidConnected = true;
      }

      // 受信
      uint16_t len = RECEIVE_BUFFER_SIZE;
      uint8_t rcode = android.RcvData(&len, receiveBuffer);
      if (rcode && rcode != hrNAK) {
#ifdef DEBUG_ANDROID
        DEBUG_PORT.print(F("\r\nData rcv: "));
        DEBUG_PORT.println(rcode, HEX);
#endif
      } else if (len > 0) {
#ifdef DEBUG_ANDROID
        DEBUG_PORT.print(F("\r\nData Packet: "));
        DEBUG_PORT.print((char*)receiveBuffer);
        DEBUG_PORT.println();
#endif
      }

      // 送信

      if (millis() - accTransmitLastTime > 250) {

        androidPayload[0] = 0x7C;         // ヘッダー上
        androidPayload[1] = 0xC7;         // ヘッダー下
        androidPayload[2] = 0x00;         // 予備
        androidPayload[3] = PAYLOAD_SIZE; // データサイズ
        for (uint8_t i = 0; i < PAYLOAD_SIZE; i++) androidPayload[i + ANDROID_DATA_HEAD] = payload[i];
        androidPayload[ANDROID_CHECKSUM_BYTE] = android_checksum();

        /*DEBUG_PORT.print("ANDROID PAYLOAD:\n");
          for (int i = 0; i < ANDROID_PAYLOAD_SIZE; i++) {
          DEBUG_PORT.print(androidPayload[i]);
          if (i % 30 == 0 && i != 0) DEBUG_PORT.print('\n');
          else DEBUG_PORT.print(' ');
          }
          DEBUG_PORT.println();*/

        rcode = android.SndData(ANDROID_PAYLOAD_SIZE, androidPayload);
        // 送った後はPayloadの中身が0になります

        accTransmitLastTime = millis();
        if (rcode && rcode != hrNAK) {
#ifdef DEBUG_ANDROID
          DEBUG_PORT.print(F("\r\nData send: "));
          DEBUG_PORT.print(rcode, HEX);
#endif
        } else if (rcode != hrNAK) {
#ifdef DEBUG_ANDROID
          DEBUG_PORT.print("SENT ANDROID PAYLOAD\n");
#endif
        }
      }
    } else {
      androidConnected = false;
    }
  }
#ifdef DEBUG_ANDROID
  DEBUG_PORT.print("Android Loop Time: ");
  DEBUG_PORT.print(millis() - loop_time_android);
  DEBUG_PORT.print("\n");
  DEBUG_PORT.flush();
#endif
}

uint8_t android_checksum(void) {
  uint8_t sum = 0;
  uint8_t i;
  for (i = ANDROID_DATA_HEAD; i < ANDROID_CHECKSUM_BYTE; i++) {
    sum ^= androidPayload[i];
  }
  return sum;
}



