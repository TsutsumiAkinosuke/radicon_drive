#include <M5Core2.h>

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

int pwm_pin[8] = {14, 26, 25, 13, 32, 27, 19, 33};

// MDに入力するduty比(0~255)を格納する配列(0:右前, 1:左前, 2:左後, 3:右後)
int duty[4] = {0};

// エラーメッセージをシリアル出力(RCCHECKマクロでエラー判定されたときに実行)
void error_loop(){
  while(1){
    M5.Lcd.println("An error occured during init micro-ROS");
    delay(10000);
  }
}

// メッセージを受信したときに実行するコールバック関数
void twist_callback(const void * msgin)
{
  // 受信したメッセージを格納
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // 各モータのduty比を計算
  duty[0] = msg->linear.x *  127 + msg->angular.z * 127;
  duty[1] = msg->linear.x * -127 + msg->angular.z * 127;
  duty[2] = msg->linear.x * -127 + msg->angular.z * 127;
  duty[3] = msg->linear.x *  127 + msg->angular.z * 127;

  for(int i = 0; i < 4; i++) {
    // ledcWrite(i*2, duty[i]*(duty[i]>0));
    // ledcWrite(i*2+1, -1*duty[i]*(duty[i]<0));
    if (duty[i] > 0) {
      ledcWrite(i*2, duty[i]);
      ledcWrite(i*2+1, 0);
    } else if(duty[i] < 0) {
      ledcWrite(i*2, 0);
      ledcWrite(i*2+1, -1*duty[i]);
    } else {
      ledcWrite(i*2, 0);
      ledcWrite(i*2+1, 0);
    }
  }
}

void setup() {

  // M5Stackのセットアップ
  M5.begin();

  // 画面の明るさを最大にして白色に塗りつぶす
  M5.Lcd.setBrightness(255);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);

  M5.Lcd.println("Initialize pwm pin");

  // PWM波生成に使用するタイマーのセットアップとピン割当て
  for(int i = 0; i < 8; i++) {
    ledcSetup(i, 20000, 8);       // i番チャンネルを20kHz・分解能8bitで初期化
    ledcAttachPin(pwm_pin[i], i); // i番チャンネルに配列pwm_pinのi番目のピンを割当て
    ledcWrite(i, 0);
  }

  M5.Lcd.println("Initialize micro-ROS");

  // micro-ROSをセットアップする関数
  // Wi-FI経由で通信する際の引数はSSID, パスワード, 接続先のPCのIPアドレス, ポート番号(適当でOK)
  set_microros_wifi_transports("SSID", "PASSWORD", "IP_ADDRESS", 2000);

  // セットアップが完了するまで少しの間待機
  delay(2000);

  // allocatorを宣言
  allocator = rcl_get_default_allocator();

  // 通信機能をサポートする構造体を宣言
  // 引数は サポートの構造体, argc, argv, allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // nodeを作成
  // 引数は nodeの構造体, nodeの名前, 名前空間, サポートの構造体
  RCCHECK(rclc_node_init_default(&node, "radicon_node", "", &support));

  // subscriptionを作成
  // 引数は subscriptionの構造体, nodeの構造体, メッセージサポート, topicの名前
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // executorを作成
  // 引数は executorの構造体, サポートのコンテクスト, ハンドル数, allocator
  // ハンドル数：マイコンで処理するtopic, service, timerなどのコールバックの数
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // executorにsubscriptionを追加
  // 引数は executor, subscription, msg, コールバック関数, 
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &twist_callback, ON_NEW_DATA));

  M5.Lcd.println("Successfully initialized micro-ROS");

  delay(1000);
}

void loop() {
  // 実行可能なハンドル(コールバック関数)があれば実行する
  // 指定した待機時間が経過した後に制御が返される
  // 引数は executor, 待機時間(ナノ秒)
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
