// XBeeに関するドキュメンテーション https://www.digi.com/resources/documentation/digidocs/PDFs/90000976.pdf
// XBeeのBaudrateに関する記事 https://diydrones.com/forum/topics/baud-rates-why-dont-we-use?id=705844%3ATopic%3A383022&page=1#comments

#include <XBee.h>                         // Xbee経由PC通信用ライブラリ

#define XBEE_SERIAL Serial                // XBee用シリアル
#define PAYLOAD_SIZE 73     //APIモードXBee通信データパケットサイズ broadcastはMAX:92 unicastはMAX:84(超えてもなんかできてた)

#ifdef DEBUG_XBEE
uint32_t loop_time_xbee = 0;
#endif

const uint16_t transmit_cooldown = 70;
uint32_t last_transmit = 0;

uint8_t payload[PAYLOAD_SIZE] = {0}; //XBee通信データパケット

XBee xbee = XBee();         //APIモードXBee通信インスタンス

// 接続予定のZigbeeの個数
const uint8_t XBEE_COUNT = 2;

// 送信先のXbeeのシリアル番号
// 配信するとき(Broadcast)はアドレスを(0x00000000, 0x0000FFFF)にする
// Coordinatorに無条件で送信するときはアドレスを(0x00000000, 0x00000000)にする
// 配信しないとき(Unicast)はここを送信先のXBeeのMACアドレスにする (例: 0x0013A200, 0x40E7EAF6 / 0x0013A200, 0x40E7EDD6)
// BroadcastはUnicastと比べて性質上、めちゃくちゃ遅いのでパケットが大きかったり、距離が大きかったりすると正しく送受信できないかもです
XBeeAddress64 XBeeAddressArray[XBEE_COUNT] = {
  XBeeAddress64(0x0013A200, 0x40E7EDD6),
  XBeeAddress64(0x0013A200, 0x40E7EAF6)
};

ZBTxRequest ZBTxRequestArray[XBEE_COUNT];

ZBRxResponse rx = ZBRxResponse();
ZBTxStatusResponse tsr = ZBTxStatusResponse();

void initXBee() {
  payload[0] = 0xAA;
  for (uint8_t i = 0; i < XBEE_COUNT; i++) {
    ZBTxRequestArray[i] = ZBTxRequest(XBeeAddressArray[i], payload, PAYLOAD_SIZE);
    ZBTxRequestArray[i].setFrameId(i+1); // FrameIDを0以外の数字にすることで返答をオフにする(返答に依存するものを作りたくないだけ)
    ZBTxRequestArray[i].setOption(0x01); // [超重要]送信失敗時に送信をリトライすることを無効にする, これがないと複数送信を保証できない
  }
  XBEE_SERIAL.begin(57600);                  // Xbee経由PC通信用シリアルの開始
  XBEE_SERIAL.write('B');                   // Xbee Programmable モデル使用時は電源を入れた後にBypassモードにするために'B'と送る必要がある
  xbee.setSerial(XBEE_SERIAL);
}

void receivePayload() {
  xbee.readPacket(); // 受信パケットを読み込み
  if (xbee.getResponse().isAvailable()) {
    // 何かが受信された
    DEBUG_PORT.print("Received API ID: 0x");
    DEBUG_PORT.println(xbee.getResponse().getApiId(), HEX);
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
      DEBUG_PORT.print("Frame ID: 0x");
      DEBUG_PORT.println(tsr.getFrameId(), HEX);
    } else {
      flashDebug(1, 25);
    }
  } else if (xbee.getResponse().isError()) { // レスポンスがエラー
  }
}

uint8_t *getPayload() {
  return payload;
}

uint8_t dest = 0;

void transmitPayload() {
#ifdef DEBUG_XBEE
  loop_time_xbee = millis();
#endif
  if ((uint32_t)(millis() - last_transmit) > transmit_cooldown) {
    digitalWrite(DEBUG_LED, HIGH);
//    xbee.send(ZBTxRequestArray[dest]); // まあとりあえず送信する
//    DEBUG_PORT.println(); // デバッグしやすくるためだけに改行
//    dest = (dest + 1) % XBEE_COUNT;
    for (uint8_t i = 0; i < XBEE_COUNT; i++) {
      xbee.send(ZBTxRequestArray[i]); // まあとりあえず送信する
      DEBUG_PORT.println(); // デバッグしやすくるためだけに改行
    }
    last_transmit = millis();
    digitalWrite(DEBUG_LED, LOW);
  }
#ifdef DEBUG_XBEE
  DEBUG_PORT.print("\nXBEE transmit Loop Time: ");
  DEBUG_PORT.print(millis() - loop_time_xbee);
  DEBUG_PORT.print("\n");
  DEBUG_PORT.flush();
#endif
}



