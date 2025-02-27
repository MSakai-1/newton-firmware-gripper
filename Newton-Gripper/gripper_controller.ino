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

int command = 0;
Mode mode = Opening;
unsigned long moveStartTime = 0;

void readSerialCommand() {
  unsigned long startTime = millis();
  command = 0;
  while (Serial.available()) {// たまったデータを読み捨てる
      Serial.read();  
  }
  //0.5秒間待機し、データが入ってきたら処理、なければ0を出力
    while (millis() - startTime < 500) {
        if (Serial.available()) {
            String received = Serial.readStringUntil('\n'); // 改行まで受信
            received.trim();// 空白や改行を削除
            command = received.toInt();  //文字列を整数に変換
            Serial.print("Received Command: "); Serial.println(command);
            return;//処理が終わったらリターン
        }
    }
}

void wait_ms(unsigned long t)
{
  unsigned long now=millis();
  while(millis()-now<t);
}
  

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //while (!Serial);  // シリアル接続が確立するまで待つ
  motor.attach(motorPin);  // モータのPWM制御ピンを設定
  motor.writeMicroseconds(1500);
  wait_ms(50);
  Serial.println("Start Control");
  moveStartTime=millis();
}

void loop() {
    unsigned long now = millis();

    if (mode == Opening) {
      PWM = 1600;
      if (now - moveStartTime > maxConstantActiveTime) {
        mode = Opened;
      }
    } 

    else if (mode == Closing) {
      PWM = 1400;
      if (now - moveStartTime > maxConstantActiveTime) {
        mode = Closed;
      }   
    }

    else if (mode == Opened) {
      PWM = 1500;
      readSerialCommand();
      if (command == -1) {
        mode = Closing;
        moveStartTime = now;
      }
    }
    
    else {//(mode == Closed) 
      PWM = 1500;
      readSerialCommand();
      if (command == 1) {
        mode = Opening;
        moveStartTime = now;
      }
    
    }
    motor.writeMicroseconds(PWM);
    Serial.println(mode);
    wait_ms(50);
  }


