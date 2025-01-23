#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>  // サーボライブラリのインクルード

VL53L0X sensor;
Servo myServo;  // サーボオブジェクトの作成

const int servo_close = 89; // アームの閉じ具合　値を小さく→強くつかむ　大きく→緩くつかむ
const int servo_middle = 100; // 中間の開き具合　緩くつかみたいとき用　※値は仮置き
const int servo_open = 120; // アームの開き具合

void setup() {
  Serial.begin(9600);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // サーボの接続ピンを設定
  myServo.attach(9);  // サーボを9番ピンに接続

  // 距離センサを連続モードで開始
  sensor.startContinuous();


}

void loop() {

// シリアル通信からの入力によるアームの開閉制御
/*
  if (Serial.available() > 0) {
    char input = Serial.read();  // 入力を読み取る
    
    if (input == '0') {
      myServo.write(servo_close);  // サーボを閉じる
      Serial.println("Servo close");
    } 
    if (input == '1') {
      myServo.write(servo_open);  // サーボを開く
      Serial.println("Servo open");
    }
    else if (input == '2') {
      myServo.write(servo_middle);  // サーボを緩く閉じる
      Serial.println("Servo middle");
    } else {
      Serial.println("Invalid input. Enter 0 to close or 1 to open or 2 to middle.");
    }
  }
*/

// 以下距離センサの値によるアームの開閉制御

  int distance = sensor.readRangeContinuousMillimeters();  // 距離をmm単位で読み取る
  
  if (sensor.timeoutOccurred()) { 
    Serial.println(" TIMEOUT");
  }

  Serial.print("Distance: ");
  Serial.print(distance);

  // ブロックが遠い場合、アームを解放
  if (distance > 120) {
    myServo.write(servo_open);  
    Serial.println(" ");
  }
  
  // ブロックが近づいた場合、アームを閉じる
  if (distance <= 120 && distance >= 95) {
    myServo.write(servo_close); 
    Serial.println(" Servo close");
  }
  
  // ブロックが押し込まれた場合、アームを解放
  else if (distance < 95) {
    myServo.write(servo_open); 
    Serial.println(" Servo open");
    delay(5000);  //離して下がる間の待ち時間
  }

  delay(100);  // 次の測定までの待機時間

}
