/*
4輪オムニ試作機
ROS2から速度指令をRPMで受信
エンコーダーからRPMを求めPID制御をかける　
現時点ではPIDは１輪のみに実装
2024/07/04
*/

#include "EthernetInterface.h"
#include "QEI.h"
#include "mbed.h"
#include "rtos.h"
#include <cstdint>

/// QEI
QEI E1(D3, D2, NC, 2048, QEI::X4_ENCODING);
// QEI E2(PA_4, PB_0, NC, 2048, QEI::X2_ENCODING);

/*
QEI (A_ch, B_ch, index, int pulsesPerRev, QEI::X2_ENCODING)
index -> Xピン, １回転ごとに１パルス出力される？ 使わない場合はNCでok
pulsePerRev -> Resolution (PPR)を指す
X4も可,X4のほうが細かく取れる

データシート(

): https://jp.cuidevices.com/product/resource/amt10-v.pdf
*/
// end

using ThisThread::sleep_for;

void receive(UDPSocket *receiver);

DigitalOut MD1D(D4);
PwmOut MD1P(D5);

DigitalOut MD2D(D7);
PwmOut MD2P(D6);

DigitalOut MD3D(D8);
PwmOut MD3P(D9);

DigitalOut MD4D(D12);
PwmOut MD4P(D10);

DigitalOut MD5D(D13);
PwmOut MD5P(D11);

int E1_Pulse;
int last_E1_Pulse;
int dt = 0;
double dt_d = 0; // casted dt
double RPM;

double target;
double Kp;
double Ki;
double Kd;
double Error;
double last_Error;
double Integral;
double Differential;
double Output = 0;
double limit;

double mdd[6];
double mdp[6];

double safety; // PWM出力制限　絶対に消すな

int main() {

  // PWM Setting
  MD1P.period_us(50);
  MD2P.period_us(50);
  MD3P.period_us(50);
  MD4P.period_us(50);
  MD5P.period_us(50);
  /*
  50(us) = 1000(ms) / 20000(Hz)
  MDに合わせて調整
  CytronのMDはPWM周波数が20kHzなので上式になる
  */
  // end

  // 送信先情報(F7)
  const char *destinationIP = "192.168.8.205";
  const uint16_t destinationPort = 4000;

  // 自機情報
  const char *myIP = "192.168.8.215";
  const char *myNetMask = "255.255.255.0";
  const uint16_t receivePort = 5000;

  // イーサネット経由でインターネットに接続するクラス
  EthernetInterface net;
  // IPアドレスとPortの組み合わせを格納しておくクラス（構造体でいいのでは？）
  SocketAddress destination, source, myData;
  // UDP通信関係のクラス
  UDPSocket udp;
  // 受信用スレッド
  Thread receiveThread;

  /* マイコンのネットワーク設定 */
  // DHCPはオフにする（静的にIPなどを設定するため）
  net.set_dhcp(false);
  // IPなど設定
  net.set_network(myIP, myNetMask, "");

  printf("Start\n");

  // マイコンをネットワークに接続
  if (net.connect() != 0) {
    printf("Network connection Error (>_<)\n");
    return -1;
  } else {
    printf("Network connection success (^_^)\n");
  }

  // UDPソケットをオープン
  udp.open(&net);

  // portをバインドする
  udp.bind(receivePort);

  // 送信先の情報を入力
  destination.set_ip_address(destinationIP);
  destination.set_port(destinationPort);
  // 受信用のスレッドをスタート
  receiveThread.start(callback(receive, &udp));

  receiveThread.join();

  udp.close();
  net.disconnect();
  return 0;
}

void receive(UDPSocket *receiver) {

  using namespace std::chrono;

  Timer t;
  t.start();
  SocketAddress source;
  char buffer[64];

  int data[6] = {0, 0, 0, 0, 0, 0};
  while (1) {
    memset(buffer, 0, sizeof(buffer));
    if (const int result =
            receiver->recvfrom(&source, buffer, sizeof(buffer)) < 0) {
      printf("Receive Error : %d", result);
    } else {

      ///////////////////////////////////////////////////////////////////////////////////
      // 受信したパケット（文字列）を処理
      char *ptr;
      int ptr_counter = 1;
      // カンマを区切りに文字列を分割
      // 1回目
      ptr = strtok(buffer, ",");
      data[1] = atoi(ptr); // intに変換

      // 2回目以降
      while (ptr != NULL) {
        ptr_counter++;
        // strtok関数により変更されたNULLのポインタが先頭
        ptr = strtok(NULL, ",");
        data[ptr_counter] = atoi(ptr); // intに変換

        // ptrがNULLの場合エラーが発生するので対処
        if (ptr != NULL) {
          // printf("%s\n", ptr);
        }
      }
      ///////////////////////////////////////////////////////////////////////////////////
      // 0.0~1.0の範囲にマッピング
      /*printf("%d, %d, %d, %d, %d\n", data[1], data[2], data[3], data[4],
             data[5]);*/

      for (int i = 1; i <= 5; i++) {
        if (data[i] > 0) {
          mdd[i] = 1;
        } else if (data[i] < 0) {
          mdd[i] = 0;
        }
        mdp[i] = fabs(data[i]) / 255;
      }
      t.stop();
      dt = duration_cast<milliseconds>(t.elapsed_time()).count();
      //回転数の取得およびRPMの計算//////////////////////////////////////////////////////////////////
      E1_Pulse = E1.getPulses();
      RPM = 60000.0 / dt * E1_Pulse /
            8192; // 現在のRPM（1分間当たりの回転数）を求める
            E1.reset();
      // printf("%d\n", RPM);
      RPM = fabs(RPM);
      /////////////////////////////////////////////////////////////////////////////////////////////

      /*
      printf("%f, %d, %d, %d, %d, %d\n", RPM, data[1], data[2], data[3],
             data[4], data[5]);*/

      // PID///////////////////////////////////////////////////////////////////////////////////////

      // PID parameter
      Kp = 0.1;
      Ki = 0.0015;
      Kd = 0.000000001;
      limit = 60.0;
      safety = 0.6;
      // end

      dt_d = (double)dt / 1000000000; // cast to double
      target = abs((double)data[1]) / limit;
      Error = target - (RPM / limit);                // P
      Integral += ((Error + last_Error) * dt_d / 2); // I
      Differential = (Error - last_Error) / dt_d;    // D

      last_Error = Error;
      Output += ((Kp * Error) + (Ki * Integral) + (Kd * Differential)); // PID
      mdp[1] = Output;

      // PWM出力制限　絶対に消すな
      if (mdp[1] > safety) {
        mdp[1] = safety;
      } else if (mdp[1] < 0.0) {
        mdp[1] = 0.0;
      }
      // end

      /*
            //安全のため出力を制限　絶対に消すな
            if (mdp[1] >= safety) {
              mdp[1] = safety;
            }
            // end*/
      t.reset();
      t.start();
      printf("%lf, %lf, %d\n", RPM, mdp[1], data[1]);

      ////////////////////////////////////////////////////////////////////////////////////////////

      // Output////////////////////////////////////////////////////////////////////////////////////

      MD1D = mdd[1];
      MD2D = mdd[2];
      MD3D = mdd[3];
      MD4D = mdd[4];
      MD5D = mdd[5];

      MD1P = mdp[1];
      MD2P = mdp[2];
      MD3P = mdp[3];
      MD4P = mdp[4];
      MD5P = mdp[5];

      ///////////////////////////////////////////////////////////////////////////////////
    }
  }
}