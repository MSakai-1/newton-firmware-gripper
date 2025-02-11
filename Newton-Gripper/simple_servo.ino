#include <Servo.h>

Servo motor;  // PWM信号を出すオブジェクト
const int motorPin = 9;  // PWM出力ピン

void setup() {
  // put your setup code here, to run once:
  motor.attach(motorPin);  // モータのPWM制御ピンを設定

}

void loop() {
  // put your main code here, to run repeatedly:
  // ニュートラル（停止）
    motor.writeMicroseconds(1500);
    delay(2000);

    // 開く（正転）
    motor.writeMicroseconds(1700);
    delay(2000);

    // 停止
    motor.writeMicroseconds(1500);
    delay(2000);

    // 閉じる（逆転）
    motor.writeMicroseconds(1300);
    delay(2000);

    // 停止
    motor.writeMicroseconds(1500);
    delay(2000);

}