// Hardware シリアルのバッファーの大きさを64byteから256byteに変更する必要性有り
// 正しく設定しないとGPSの受信が失敗する(かもしれない)
// 設定方法はhttps://internetofhomethings.com/homethings/?p=927にて
// なお、HardwareSerial.hという名前のファイルは複数存在する可能性があるのでご注意
// バッファーサイズはSerial.println(SERIAL_RX_BUFFER_SIZE); で確認できます。

// includeをする順番があります。ちょっとだけ気をつけてください。
#include "debug.h"            // 一番上にする
#include "readAltitude.h"
#include "readGPS.h"
#include "readIMU.h"
#include "readTHP.h"
#include "readControl.h"
#include "xbeeUtil.h"
#include "readRotations.h"
#include "AndroidComm.h"

void setup() {
  initDebug();
  initXBee();
  initAccessory();
  initAltitude();
  initControl();
  initIMU();
  initTHP();
  initGPS();
  initRotations();
}

void loop() {

  readControl();                  // 操舵情報
  readGPS();                      // GPS情報の読取り

  readTHP();                      // 温度、湿度、気圧

  updateIMU();                    // 9軸センサ(更新)
  readIMU();                      // 9軸センサ(取得)

  readAltitude();                 // 高度 一回の実行に約86msかかります。気をつけて下さい。
  readGPS();                      // GPS情報の読取り

  readRotations();                // 回転数の計算

  readAltitude();                 // 高度の読取り
  readGPS();                      // GPS情報の読取り

  packRotations (getPayload());     // | [1~4]       | 回転数データ    |
  packIMU       (getPayload());     // | [5~17]      | 9軸センサデータ |
  packAltitude  (getPayload());     // | [18~19]     | 高度           |
  packGPS       (getPayload());     // | [20~43]     | GPS情報        |
  packTHP       (getPayload());     // | [44~55]     | 温度・湿度・気圧 |
  packControl   (getPayload());     // | [56~93]     | 操舵情報        |

  accessory(getPayload());        // androidパケット送受信
  transmitPayload();              // xbeeにパケットの送信
  receivePayload();               // PCからデータ受信
  receivePayload();               // PCからデータ受信
  receivePayload();               // PCからデータ受信
}


