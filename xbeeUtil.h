#include <EEPROM.h>                       // EEPROMでID
#include <XBee.h>                         // Xbee経由PC通信用ライブラリ

#define XBEE_SERIAL Serial                // XBee用シリアル
#define PAYLOAD_SIZE 98     //APIモードXBee通信データパケットサイズ

#define transmit_cooldown 0
uint32_t last_transmit = 0;

uint8_t payload[PAYLOAD_SIZE] = {0}; //XBee通信データパケット

// 送信先のXbeeのシリアル番号
XBeeAddress64 addr64_boat = XBeeAddress64(0x0013A200, 0x40E7EAF6);
XBeeAddress64 addr64_land = XBeeAddress64(0x0013A200, 0x40E7EAF6);

ZBTxRequest boat_packet = ZBTxRequest(addr64_boat, payload, PAYLOAD_SIZE);
ZBTxRequest land_packet = ZBTxRequest(addr64_land, payload, PAYLOAD_SIZE);

XBee xbee = XBee();         //APIモードXBee通信インスタンス
XBeeResponse response = XBeeResponse();

ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

void initXBee() {
  XBEE_SERIAL.begin(9600);                  // Xbee経由PC通信用シリアルの開始
  xbee.setSerial(XBEE_SERIAL);
  XBEE_SERIAL.print('B');
}

void receivePayload() {
  /*xbee.readPacket(); // 受信パケットを読み込み
  if (xbee.getResponse().isAvailable()) { // レスポンスが存在する
    if (xbee.getResponse.getApiId == ZB_RX_REPSONSE) { // レスポンスがきた
      xbee.getResponse().getZBRxResponse(rx);
      if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
        
      } else {
        
      }
    } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) { // モデムステータスレスポンス
      xbee.getResponse().getModemStatusResponse(msr);
      if (msr.getStatus() == ASSOCIATED) {
        
      } else if (msr.getStatus() == DISASSOCIATED) {
        
      } else {
        
      }
    } else {
      
    }
  } else if (xbee.getResponse().isError()) { // レスポンスがエラー
    
  }*/
}

uint8_t *getPayload() {
  return payload;
}

void transmitPayload() {
#ifndef DEBUG
  if ((uint32_t)(millis() - last_transmit) > transmit_cooldown) {
    digitalWrite(DEBUG_LED, HIGH);
    xbee.send(boat_packet);
    xbee.send(land_packet);
    last_transmit = millis();
    digitalWrite(DEBUG_LED, LOW);
  }
#endif
}

