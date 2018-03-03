// Hardware シリアルのバッファーの大きさを64byteから256byteに変更する必要性有り
// 正しく設定しないとGPSの受信が失敗する(かもしれない)
// 設定方法はhttps://internetofhomethings.com/homethings/?p=927にて
// なお、HardwareSerial.hという名前のファイルは複数存在する可能性があるのでご注意
// バッファーサイズはSerial.println(SERIAL_RX_BUFFER_SIZE); で確認できます。

#include "debug.h"
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
  initAltitude();
  initControl();
  initIMU();
  initTHP();
  initGPS();
  initRotations();
}

void loop() {
  readAltitude();                 // 高度

  readControl();                  // 操舵情報

  readAltitude();                 // 高度

  readTHP();                      // 温度、湿度、気圧

  readAltitude();                 // 高度

  updateIMU();                    // 9軸センサ(更新)
  readIMU();                      // 9軸センサ(取得)

  readAltitude();                 // 高度の読取り

  stopInterrupts();               // インタラプトの一時停止
  readGPS();                      // GPS情報の読取り
  readRotations();                // 回転数の計算
  restartInterrupts();            // インタラプトの再開

  readAltitude();                 // 高度の読取り

  // ----------------------------------+-------------+---------------+
  // 取得データの送信パケットへの積み込み     |配列インデックス|   情報         |
  // ----------------------------------+-------------+---------------+
  packRotations (getPayload());     // | [1~4]       | 回転数データ    |
  packIMU       (getPayload());     // | [5~17]      | 9軸センサデータ  |
  packAltitude  (getPayload());     // | [18~19]     | 高度           |
  packGPS       (getPayload());     // | [20~43]     | GPS情報        |
  packTHP       (getPayload());     // | [44~55]     | 温度・湿度・気圧 |
  packControl   (getPayload());     // | [56~93]     | 操舵情報        |
  // ----------------------------------+-------------+---------------+

  transmitPayload();              // パケットの送信
}
