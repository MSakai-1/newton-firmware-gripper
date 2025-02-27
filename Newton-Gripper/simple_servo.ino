#include <Servo.h>

Servo motor;  // PWM信号を出すオブジェクト
const int motorPin = 9;  // PWM出力ピン

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  motor.attach(motorPin);  // モータのPWM制御ピンを設定
  Serial.println("モータ制御開始");

}

void loop() {
  // put your main code here, to run repeatedly:
    
    motor.writeMicroseconds(1300);
    Serial.println("1300µs");
    delay(5000);

    motor.writeMicroseconds(1500);
    Serial.println("1500µs");
    delay(3000);

    motor.writeMicroseconds(1600);
    Serial.println("1600µs");
    delay(5000);

    motor.writeMicroseconds(1500);
    Serial.println("1500µs");
    delay(3000);

    motor.writeMicroseconds(1300);
    Serial.println("1300µs");
    delay(5000);

    motor.writeMicroseconds(1800);
    Serial.println("1800µs");
    delay(5000);

    motor.writeMicroseconds(1500);
    Serial.println("1500µs");
    delay(3000);

    motor.writeMicroseconds(1300);
    Serial.println("1300µs");
    delay(5000);

    motor.writeMicroseconds(1520);
    Serial.println("1520µs");
    delay(1000);

    motor.writeMicroseconds(1500);
    Serial.println("1500µs");
    delay(3000);

    motor.writeMicroseconds(1300);
    Serial.println("1300µs");
    delay(5000);

}