// XBeeに関するドキュメンテーション https://www.digi.com/resources/documentation/digidocs/PDFs/90000976.pdf
// XBeeのBaudrateに関する記事 https://diydrones.com/forum/topics/baud-rates-why-dont-we-use?id=705844%3ATopic%3A383022&page=1#comments

#include <XBee.h>                         // Xbee経由PC通信用ライブラリ

#define XBEE_SERIAL Serial                // XBee用シリアル
#define PAYLOAD_SIZE 77     //APIモードXBee通信データパケットサイズ broadcastはMAX:92 unicastはMAX:84(超えてもなんかできてた)

#ifdef DEBUG_XBEE
uint32_t loop_time_xbee = 0;
#endif

const uint16_t transmit_cooldown = 50;
uint32_t last_transmit = 0;

uint8_t payload[PAYLOAD_SIZE] = {0}; //XBee通信データパケット

XBee xbee = XBee();         //APIモードXBee通信インスタンス

// 接続予定のZigbeeの個数
const uint8_t XBEE_COUNT = 2;
const uint8_t frameId_offset = 2;

typedef struct XBeeProfile {
  uint8_t frameId;
  XBeeAddress64 address64;
  ZBTxRequest request;
  boolean responded = true;
  uint32_t last_sent_time = 0;
} XBeeProfile;

XBeeProfile xbeeProfile[XBEE_COUNT];

// 送信先のXbeeのシリアル番号
// 配信するとき(Broadcast)はアドレスを(0x00000000, 0x0000FFFF)にする
// Coordinatorに無条件で送信するときはアドレスを(0x00000000, 0x00000000)にする
// 配信しないとき(Unicast)はここを送信先のXBeeのMACアドレスにする (例: 0x0013A200, 0x40E7EAF6 / 0x0013A200, 0x40E7EDD6)
// BroadcastはUnicastと比べて性質上、めちゃくちゃ遅いのでパケットが大きかったり、距離が大きかったりすると正しく送受信できないかもです
XBeeAddress64 XBeeAddressArray[XBEE_COUNT] = {
  XBeeAddress64(0x0013A200, 0x40E7EDD6),
  XBeeAddress64(0x0013A200, 0x40E7EAF6)
};

ZBRxResponse rx = ZBRxResponse();
ZBTxStatusResponse tsr = ZBTxStatusResponse();

void initXBee() {
  payload[0] = 0xAA;
  for (uint8_t i = 0; i < XBEE_COUNT; i++) {
    xbeeProfile[i].frameId = i + 1;
    xbeeProfile[i].address64 = XBeeAddressArray[i];
    xbeeProfile[i].request = ZBTxRequest(xbeeProfile[i].address64, payload, PAYLOAD_SIZE);
    xbeeProfile[i].request.setFrameId(i + frameId_offset); // FrameIDを0以外の数字にすることで返答をオフにする(返答に依存するものを作りたくないだけ)
    xbeeProfile[i].request.setOption(0x01); // [超重要]送信失敗時に送信をリトライすることを無効にする, これがないと複数送信を保証できない
  }
  XBEE_SERIAL.begin(57600);                  // Xbee経由PC通信用シリアルの開始
  XBEE_SERIAL.write('B');                   // Xbee Programmable モデル使用時は電源を入れた後にBypassモードにするために'B'と送る必要がある
  xbee.setSerial(XBEE_SERIAL);
}

void receivePayload() {
  xbee.readPacket(); // 受信パケットを読み込み
  if (xbee.getResponse().isAvailable()) {
    // 何かが受信された
#ifdef DEBUG_XBEE
    DEBUG_PORT.print("Received API ID: 0x");
    DEBUG_PORT.println(xbee.getResponse().getApiId(), HEX);
#endif
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) { // 相手がデータを送ってきた
      xbee.getResponse().getZBRxResponse(rx);
      if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
        // 送信者が我々のACKを受け取った
      } else {
        // ACKが受信されなかった
        flashDebug(2, 20);
      }
      // ここで受信パケットの中身が見れる
      // 受信パケットの配列は　rx.getData(index); みたいな感じで参照することができる
      flashDebug(rx.getData(0) * 10, 0);
    } else if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(tsr);
      uint8_t frameId = tsr.getFrameId();
#ifdef DEBUG_XBEE
      DEBUG_PORT.print("Frame ID: 0x");
      DEBUG_PORT.print(frameId, HEX);
      DEBUG_PORT.print(" ");
      if (tsr.isSuccess()) DEBUG_PORT.println("Success!");
      else DEBUG_PORT.println("Failed");
#endif
      if (0 <= frameId - frameId_offset && frameId - frameId_offset < XBEE_COUNT) {
        if (tsr.isSuccess()) xbeeProfile[frameId - frameId_offset].responded = true;
      }
    } else {
      flashDebug(1, 25);
    }
  } else if (xbee.getResponse().isError()) { // レスポンスがエラー
  }
}

uint8_t *getPayload() {
  return payload;
}

uint8_t last_dest = 0;
uint8_t dest;

void transmitPayload() {
#ifdef DEBUG_XBEE
  loop_time_xbee = millis();
#endif

  if ((uint32_t)(millis() - last_transmit) > transmit_cooldown) {
    boolean has_connected = false;
    for (uint8_t i = 0, dest = last_dest; i < XBEE_COUNT; i++, dest = (last_dest + 1) % XBEE_COUNT) {
      if (xbeeProfile[dest].responded || (uint32_t)(millis() - xbeeProfile[dest].last_sent_time) > 10000) {
        // digitalWrite(DEBUG_LED, HIGH);
        xbee.send(xbeeProfile[dest].request);
        DEBUG_PORT.println();
        xbeeProfile[dest].last_sent_time = millis();
        xbeeProfile[dest].responded = false;
        last_dest = dest;
        // digitalWrite(DEBUG_LED, LOW);
        has_connected = true;
        last_transmit = millis();
        break;
      }
    }
    if (!has_connected) {
      for (uint8_t i = 0; i < XBEE_COUNT; i++) {
        xbeeProfile[i].responded = true;
      }
    }
    last_dest = (last_dest + 1) % XBEE_COUNT;
  }
  
#ifdef DEBUG_XBEE
  DEBUG_PORT.print("\nXBEE transmit Loop Time: ");
  DEBUG_PORT.print(millis() - loop_time_xbee);
  DEBUG_PORT.print("\n");
  DEBUG_PORT.flush();
#endif
}



