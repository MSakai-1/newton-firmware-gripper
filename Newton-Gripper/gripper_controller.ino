#include <Servo.h>

Servo motor;  // PWM信号を出すオブジェクト
const int motorPin = 9;  // PWM出力ピン
int PWM = 1600; //PWMの初期値
const unsigned long maxConstantActiveTime = 4000;//連続稼働は4秒まで
const unsigned long observationWindow = 60000;  // 60秒間の監視ウィンドウ
const unsigned long maxActiveTime = 45000;  // 45秒以上動いていたら休む
const unsigned long restDuration = 20000;  // 20秒間の休止
const unsigned long activeTimeInLast60s = 0;

enum Status {//グリッパーの状況
  NEUTRAL,
  OPEN, 
  CLOSE
};

enum Mode {//動作しているか
  moving,
  idle,
  resting
};

Status status = OPEN;
Mode mode = idle;
unsigned long moveStartTime = 0;
unsigned long lastMovingTime = 0;
unsigned long lastRestTime = 0;
unsigned long activeTimeInLast60s = 0;

void printStatus(Status status) {
    switch (status) {
        case OPEN:
            Serial.println("Opening");
            break;
        case CLOSE:
            Serial.println("Closing");
            break;
        case NEUTRAL:
        default:
            Serial.println("NEUTRAL");
            break;
    }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);  // シリアル接続が確立するまで待つ
  motor.attach(motorPin);  // モータのPWM制御ピンを設定
  motor.writeMicroseconds(PWM);
  Serial.println(status);
  delay(4000);


}

void loop() {
    unsigned long now = millis();

    // 過去60秒間の累積時間を計算
    if (now - lastMovingTime < observationWindow) {
        activeTimeInLast60s += min(maxConstantActiveTime, now - lastMovingTime);
    }

    if (activeTimeInLast60s > maxActiveTime) {
        mode = resting;
        lastRestTime = now;
        Serial.println("Safety limit reached! Entering 20-second rest period.");
    }

    if (mode == moving) {
      motor.writeMicroseconds(PWM);
      printStatus(status);
      lastMovingTime = now;
      //if (now - moveStartTime > maxConstantActiveTime) {
      //  mode = idle;
      //}
      ////////or///////
      delay(4000);
      mode = idle;
      Serial.println("Max active time reached, switching to idle.");
      if (PWM == 1600) {
                status = OPEN;
            } else if (PWM == 1400) {
                status = CLOSE;
            }
    } 

    else if (mode == resting) {
        if (now - lastRestTime >= restDuration) {
            mode = idle;
            Serial.println("Rest period over. Resuming operation.");
        } 
        PWM = 1500;
        motor.writeMicroseconds(PWM);
        
    }


    else if (mode == idle) {
      if (Serial.available()) {  
        while (Serial.available()) {  // たまったデータを読み捨てる
            Serial.read();  
        }
        String received = Serial.readStringUntil('\n');  // 改行まで受信
        received.trim();  // 空白や改行を削除

        int value = received.toInt();  // 文字列を整数に変換

        // 入力値に応じてPWMを変更し、メッセージを送信
        if (value == 1) {
          if (status == OPEN) {
            PWM = 1500;
            Serial.println("idle");
          }
          else {
            PWM = 1600;
            Serial.println("Opening");
            mode = moving;
            moveStartTime = now;
          }
        } 
        else if (value == -1) {
          if (status == CLOSE) {
            PWM = 1500;
            Serial.println("idle");
          }
          else {
            PWM = 1400;
            Serial.println("Closing");
            mode = moving;
            moveStartTime = now;
          }
        } 
            
        } 
        else {
            PWM = 1500;
            Serial.println("Neutral");
        }
    }
    
    else {
      PWM = 1500;
      Serial.println("No Serial Input");
    }
    motor.writeMicroseconds(PWM);
    
  }
}

