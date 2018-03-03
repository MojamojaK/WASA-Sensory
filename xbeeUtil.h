#include <XBee.h>                         // Xbee経由PC通信用ライブラリ

#define XBEE_SERIAL Serial                // XBee用シリアル
#define PAYLOAD_SIZE 94     //APIモードXBee通信データパケットサイズ

#define transmit_cooldown 0
uint32_t last_transmit = 0;

uint8_t payload[PAYLOAD_SIZE] = {0}; //XBee通信データパケット

// 送信先のXbeeのシリアル番号
XBeeAddress64 addr64_boat = XBeeAddress64(0x0013A200, 0x40E7EAF6);
XBeeAddress64 addr64_land = XBeeAddress64(0x0013A200, 0x40E7EAF6);

ZBTxRequest boat_packet = ZBTxRequest(addr64_boat, payload, PAYLOAD_SIZE);
ZBTxRequest land_packet = ZBTxRequest(addr64_land, payload, PAYLOAD_SIZE);

XBee xbee = XBee();         //APIモードXBee通信インスタンス

void initXBee() {
  XBEE_SERIAL.begin(9600);                  // Xbee経由PC通信用シリアルの開始
  xbee.setSerial(XBEE_SERIAL);
  XBEE_SERIAL.print('B');
}

uint8_t *getPayload() {
  return payload;
}

void transmitPayload() {
#ifndef DEBUG
  if ((uint32_t)(millis() - last_transmit) > transmit_cooldown) {
    xbee.send(boat_packet);
    //xbee.send(land_packet);
    last_transmit = millis();
  }
#endif
}
