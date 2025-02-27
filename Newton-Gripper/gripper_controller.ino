#include <Servo.h>

Servo motor;  // PWM信号を出すオブジェクト
const int motorPin = 9;  // PWM出力ピン
int PWM = 1600; //PWMの初期値
const unsigned long maxConstantActiveTime = 4000;//連続稼働は4秒まで

enum Mode {
  Opening,
  Closing,
  Opened,
  Closed
};

Mode mode = Opening;
unsigned long moveStartTime = 0;
int value = 0;

void readSerialValue() {
  if (Serial.available()) {  
        while (Serial.available()) {  // たまったデータを読み捨てる
            Serial.read();  
        }
        String received = Serial.readStringUntil('\n');  // 改行まで受信
        received.trim();  // 空白や改行を削除

        value = received.toInt();  // 文字列を整数に変換
  }
}
  

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);  // シリアル接続が確立するまで待つ
  motor.attach(motorPin);  // モータのPWM制御ピンを設定
}

void loop() {
    unsigned long now = millis();

    if (mode == Opening) {
      PWM = 1600;
      if (now - moveStartTime > maxConstantActiveTime) {
        mode = Opened;
        Serial.println("Max active time reached, switching to idle.");
      }
    } 

    else if (mode == Closing) {
      PWM = 1400;
      if (now - moveStartTime > maxConstantActiveTime) {
        mode = Closed;
        Serial.println("Max active time reached, switching to idle.");
      }   
    }

    else if (mode == Opened) {
      PWM = 1500;
      readSerialValue();
      if (value == -1) {
        mode = Closing;
        moveStartTime = now;
      }
    }
    
    else {//(mode == Closed) 
      PWM = 1500;
      readSerialValue();
      if (value == 1) {
        mode = Opening;
        moveStartTime = now;
      }
    
    }
    motor.writeMicroseconds(PWM);
    Serial.println(mode);
  }
}

