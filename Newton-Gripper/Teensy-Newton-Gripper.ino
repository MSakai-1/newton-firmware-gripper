/*
Arduino(ATtiny84)用のもとのコードを、Teensy3.2用に変更
主な変更点
*割り込み処理を#include <util/atomic.h>	から　__disable_irq() / __enable_irq() を使う形に変更
*pwm出力をTCCR1AからanalogWriteFrequency() + analogWrite()に変更
*ADC 解像度 (1023)を　analogRead() / 1023.0f　から　analogReadResolution(12) を設定し、analogRead() / 4095.0f に変更
analogReadResolution(12)は12ビットなので1023を4095に設定することに。
*/



/* Blue Robotics Newton Gripper Firmware
-----------------------------------------------------

Title: Blue Robotics Newton Gripper Firmware

Description: This code is the default firmware for the Blue Robotics
Newton Gripper.  A PWM signal dictates the speed at which the gripper opens or
closes, with speeds above 1500 (plus a deadzone) closing the jaws and those
below 1500 (minus a deadzone) openning the jaws.  A current sensor detects when
the endstops are reached or when an object is in the way of the jaws, and
disables the motor until commanded to stop or reverse direction.

The current detection code adjusts for different stall currents at different
input voltages between 9 and 18 volts DC.  Smooth operation is not guaranteed
outside this range and input voltages exceeding 21 VDC should be avoided as this
can cause damage to the microcontroller.

This code requires the following library:
https://github.com/dheideman/DiscreteFilter

The code is designed for the ATtiny84 microcontroller and can be compiled and
uploaded via the Arduino 1.0+ software.

-------------------------------
The MIT License (MIT)

Copyright (c) 2020 Blue Robotics Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/

//#include <util/atomic.h>
//#include <Arduino.h>  //必要かも
#include <DiscreteFilter.h>  //フィルタ用ライブラリ、githubからインストール必要
#include "Newton-Gripper.h"

// Global Variables
volatile uint32_t pulsetime = 0;  //volatileは割りこみ処理用、pwm信号受信時刻
volatile int16_t  pulsein   = PWM_NEUTRAL;  //pwm信号width
uint32_t lastspeedfilterruntime   = 0;  //各フィルタの実行時間保存用
uint32_t lastcurrentfilterruntime = 0;
uint32_t lastOCLdetectionruntime = 0;
int      OCLcounter = 0;  //open close loop(OCL)のカウンタ、異常検知用
dir_t    limit    = NONE;  //dir_tはNewton-Gripper.hで定義されたユーザー定義型
dir_t    lastdir  = NONE;  //それぞれ制限、直前移動方向（フィルタリセット用）、現在の移動方向
dir_t    direction;
float    velocity = 0.0f;
DiscreteFilter speedfilter;  //速度フィルタ用オブジェクト
DiscreteFilter currentfilter;  //電流フィルタ用オブジェクト


///////////
// SETUP //
///////////

void setup() {
  // Set up pin modes
  pinMode(PWM_IN,     INPUT_PULLUP);  // Prevent floating input PWM w/ pullup
  pinMode(LED,        OUTPUT);
  pinMode(CURRENT_IN, INPUT);
  pinMode(SLEEPN,     OUTPUT);  //モータドライバ用制御ピン
  pinMode(OUT1,       OUTPUT);  //モータドライバ用制御ピン
  pinMode(OUT2,       OUTPUT);  //モータドライバ用制御ピン
  pinMode(VOLTAGE_IN, INPUT);
  pinMode(OCL, INPUT_PULLUP);//追加

  // Initialize PWM input reader
  //initializePWMReader();
  attachInterrupt(digitalPinToInterrupt(PWM_IN), pwmISR, CHANGE);//追加

  // Initialize PWM output generator
  //initializePWMOutput();
  // PWM出力設定 (Teensy用)
  analogWriteFrequency(OUT1, 20000);  // PWM周波数20kHz
  analogWriteFrequency(OUT2, 20000);
  analogWrite(OUT1, 0);
  analogWrite(OUT2, 0);

  // Initialize output low-pass filter（速度制御用のリードラグフィルタ）
  speedfilter.createLeadLagCompensator(FILTER_DT, TAUP_OUT, TAUN_OUT);

  // Initialize current input low-pass filter（電流制御用ローパスフィルタ）
  currentfilter.createFirstOrderLowPassFilter(CURRENT_DT, TAU_CURRENT);
  currentfilter.setSaturation(1.0f);

  // Wake up A4955(モータドライバスリープ解除)
  digitalWrite(SLEEPN, HIGH);

  // Turn off LED
  digitalWrite(LED, LOW);
}


//////////
// LOOP //
//////////

void loop() {
  // Save local version of pulsetime
  //割り込み中のpulsetimeを安全にコピー
  uint32_t tpulse;
  __disable_irq();
  tpulse = pulsetime;
  __enable_irq();

  // Make sure we're still receiving PWM inputs
  //PWM信号が途切れた場合 (INPUT_TOUT 秒以上経過) にモータを停止
  if ( (millis() - tpulse)/1000.0f > INPUT_TOUT ) {
    // If it has been too long since the last input, shut off motor
    __disable_irq();
    pulsein   = PWM_NEUTRAL;
    pulsetime = millis();
    __enable_irq();
  } // end pwm input check
    
  //Set endstop condition if Motor Driver detects a fault 
  //OCL (過電流) 検出処理
  if ( (millis() - lastOCLdetectionruntime)/1000.0f > OCL_DT ) {
    // Set next filter runtime
    lastOCLdetectionruntime = millis();
    //カウンタ処理、5回を超えたらモータの方向を制限しLEDオフに
    if (digitalRead(OCL) == LOW) {
        if (OCLcounter > 5) {
           limit = direction;
           digitalWrite(LED, LOW);
           setMotorOutput(direction, limit, velocity);
        } else {
          OCLcounter++;
        }
    } else {
      OCLcounter = 0;
    }
  }
  
  // Run speed filter at specified interval
  //速度フィルタ
  if ( (millis() - lastspeedfilterruntime)/1000.0f > FILTER_DT ) {
    // Set next filter runtime
    lastspeedfilterruntime = millis();

    // Update input filter
    runSpeedFilter();
  } // end run filters

  // Run current lp filter at specified interval
  //電流フィルタ
  if ( (micros() - lastcurrentfilterruntime)/1000000.0f > CURRENT_DT ) {
    // Set next current lp filter runtime
    lastcurrentfilterruntime = micros();

    // Update current filter
    currentfilter.step(readCurrent()/stallCurrent(readVoltage()));
  } // end current lp filter
}

///////////////
// Functions //
///////////////

/******************************************************************************
 * void runSpeedFilter()
 *
 * Calculates and generates output based on latest PWM input
 * Runs at 200 Hz
 ******************************************************************************/
void runSpeedFilter() {
  // Declare local variables
  static float rawvelocity;

  // Save pulsein locally
  //pwm入力を取得
  int16_t pulsewidth;
  __disable_irq();
  pulsewidth = pulsein;
  __enable_irq();

  // Reject signals that are way off (i.e. const. 0 V, const. +5 V, noise)
  //pwmの範囲チェック、異常値でなければNEWTRAL値を引いてpwを計算し、速度を±1で計算
  if ( pulsewidth >= INPUT_MIN && pulsewidth <= INPUT_MAX ) {
    // Remove neutral PWM bias & clamp to [-HALF_RANGE, HALF_RANGE]
    int16_t pw = constrain(pulsewidth - PWM_NEUTRAL, -HALF_RANGE, HALF_RANGE);

    if ( pw > INPUT_DZ ) {
      rawvelocity = 1.0f;
    } else if ( pw < -INPUT_DZ ) {
      rawvelocity = -1.0f;
    } else {
      // Stop motor if input is within input deadzone
      rawvelocity = 0.0f;
    }
  }

  // Filter velocity、速度フィルタをかける
  velocity = constrain(speedfilter.step(rawvelocity), -1.0f, 1.0f);

  // Figure out the current direction of travel、回転方向を決定
  if (velocity > 0.05) {
      direction = OPEN;
  } else if (velocity < -0.05) {
      direction = CLOSE;
  } else {
      direction = NONE;
  }

  // Clear current filter if we change directions
  //方向変わったら、フィルタをリセット
  if (direction != lastdir) {
    currentfilter.clear();
  }

  // Current sensor resolution: ~65 mA
  //過電流と判定されたとき、モータを制限しLED点灯
  if (direction == OPEN && currentfilter.getLastOutput() > I_LIMIT_OPEN) {
    limit = direction;
    digitalWrite(LED, HIGH);
  } else if (direction == CLOSE && currentfilter.getLastOutput() > I_LIMIT_CLOSE) {
    limit = direction;
    digitalWrite(LED, HIGH);
  } else if ( /*direction != NONE &&*/ limit != NONE && direction != limit ) {
    // We're going the opposite direction from the limit, so clear limit
    limit = NONE;
    // currentfilter.clear();
    digitalWrite(LED, LOW);
  }
  
  //計算した direction, limit, velocity を setMotorOutput() に渡す
  setMotorOutput(direction, limit, velocity);

  // Save last direction
  lastdir = direction;
}

/******************************************************************************
 * void setMotorOutput()
 *
 * Sets PWM output to motor controller
 ******************************************************************************/
void setMotorOutput(dir_t direction, dir_t limit, float velocity) {
    // Set output PWM timers
    //ATOMIC_BLOCK(ATOMIC_RESTORESTATE) を用いることで、割り込みの影響を受けずに OCR1A OCR1B を安全に設定
    //velocityの値にMAXのpwm幅をかけることでデューティー比を再現
  if ( direction == OPEN && limit != OPEN) {
    __disable_irq();
    analogWrite(OUT1, abs(velocity) * 255);
    analogWrite(OUT2, 0);
    __enable_irq();
  } else if ( direction == CLOSE && limit != CLOSE) {
    __disable_irq();
      analogWrite(OUT1, 0);
      analogWrite(OUT2, abs(velocity) * 255);
    __enable_irq();
  } else {
    __disable_irq();
      analogWrite(OUT1, 0);
      analogWrite(OUT2, 0);
    __enable_irq();
  }
}

/******************************************************************************
 * float readCurrent()
 *
 * Reads current consumption (Amperes)
 ******************************************************************************/
float readCurrent() {
  // Reported voltage is 10x actual drop across the sense resistor
  //電圧を測定し、V_IN / 4095.0f(5Vまたは3.3VのV_REF相当のAtoDデジタル値)で正規化
  //さらにモータドライバの電流検知による〈内蔵シャント抵抗による電圧降下、さらに10倍に増幅して出力する分を踏まえ10.0f*R_SENSEで割る
  return (analogRead(CURRENT_IN)*V_IN)/(4095.0f*10.0f*R_SENSE);
}

/******************************************************************************
 * float readVoltage()
 *
 * Reads input voltage (Volts)
 ******************************************************************************/
float readVoltage() {
  //電圧測定
  return (analogRead(VOLTAGE_IN)*V_IN)/(4095.0f*V_SENSE_DIV);
}

/******************************************************************************
 * float stallCurrent(float velocity, float voltage)
 *
 * Calculates stall current for given velocity and voltage (Amperes)
 ******************************************************************************/
float stallCurrent(float voltage) {
  //モータのストール電流 (停止時の最大電流) を計算
  //最大電流 (MOT_I_MAX) を超えないように制限
  return constrain((voltage/(R_MOT*MOT_FS)),0,MOT_I_MAX);
}

/******************************************************************************
 * void initializePWMOutput()
 *
 * Sets registers to run timers at the proper frequency for the PWM output
 ******************************************************************************/
void initializePWMOutput() {
  // Stop interrupts while changing timer settings
  //PWM 出力を初期化する関数
  c__disable_irq();
    analogWriteFrequency(OUT1, 20000);
    analogWriteFrequency(OUT2, 20000);
    analogWriteResolution(12);
    analogWrite(OUT1, 0);
    analogWrite(OUT2, 0);
    __enable_irq();
}


/******************************************************************************
 * void initializePWMReader()
 *
 * Sets up external interrupt for PWM reader
 ******************************************************************************/
void initializePWMReader() {
  // Enable INT0
  //PWM入力の割り込みを設定する関
  attachInterrupt(digitalPinToInterrupt(PWM_IN), pwmISR, CHANGE);
}

////////////////////////////////
// Interrupt Service Routines //
////////////////////////////////

// Define global variables only used for input timer
namespace {
  volatile uint32_t inputpulsestart = 0xFFFF;
}

/******************************************************************************
 * SIGNAL(INT0_vect)
 *
 * Watches external interrupt to read PWM input
 ******************************************************************************/
SIGNAL(INT0_vect) {
  //PWM信号の変化を検出する割り込みハンドラ
  if (digitalRead(PWM_IN)) {
    // Record start of input pulse
    __disable_irq();
    inputpulsestart = micros();
    __enable_irq();
  } else {
    // Measure width of input pulse
    __disable_irq();
    pulsein = micros() - inputpulsestart;
    pulsetime = millis();
    __enable_irq();
  }
}
