/*
 * File: Nexus_Omni4WD_Rosserial.ino
 * Purpose: ROSserial node for the Nexus Omni 4-wheeled Mecanum platform controller board 10011 (Duemilanove-328 based)
 * Version: 1.0.0
 * File Date: 21-03-2020
 * Release Date: 21-03-2020
 * URL: https://github.com/MartinStokroos/nexus_base_ros
 * License: MIT License
 *
 *
 * Copyright (c) M.Stokroos 2020
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *  
 *  
 * NOTE: The Sonars cannot be used simultaneously with the serial interface!
 *
 * This sketch is the maximum reasonable functionality that could be implemented with rosserial.
 * Because of the limited memory size of the 328, it is not possible to implement a cmd_vel subscriber
 * and four PID wheel controllers.
 *
 * commanding the wheels, PWM value signed int16
 * cmd_motor/motor0    left front
 * cmd_motor/motor1    left rear
 * cmd_motor/motor2    right rear
 * cmd_motor/motor3    right front
 *
 * raw velocity read back (number of encoder increments/decrements each sample period)
 * wheel_vel/enc0    left front
 * wheel_vel/enc1    left rear
 * wheel_vel/enc2    right rear
 * wheel_vel/enc3    right front
 *
 * Start communiction with:
 * rosrun rosserial_python serial_node.py /dev/ttyUSB0
 *
 * Sometimes it takes a moment to establish the communication because a false character has to be flushed
 * from the buffer first.
 *
 *
 *                           -----------------------------
 *                          |                             |
 *            left front M0 |                             | M3 right front
 *                          |                             |
 *                           -----------------------------
 *                          |                             |
 *                          |                             |
 *                          |                             |
 *                          |                             |
 *                          |                             |
 *                          |                             |
 *                          |                             |
 *                          |                             |
 *             left rear M1 |                             | M2 right rear
 *                          |                             |
 *                           -----------------------------
 *
 *
 */

#include <ros.h>
#include <nexus_base_ros/Encoders.h>
#include <nexus_base_ros/Motors.h>
#include <nexus_base_ros/EmergencyStopEnable.h>
#include <nexus_base_ros/ArmingEnable.h>
#include <PinChangeInt.h>       // https://github.com/GreyGnome/PinChangeInt (update to https://github.com/GreyGnome/EnableInterrupt ??)
#include <digitalWriteFast.h>   // library for high performance digital reads and writes by jrraines
                                // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                                // and http://code.google.com/p/digitalwritefast/

#define MSG_PUB_RATE 20 // publishing rate in Hz.
#define PWD_TIMEOUT 3 // motor power time-out in s.

// control loop timing constants
#define LOOPTIME 1000000 / MSG_PUB_RATE // loop time in us
#define PWD_TIMEOUT_VAL PWD_TIMEOUT * MSG_PUB_RATE // nof loop iterations before entering the low power mode when receiving no commands.

#define M0_PWM_PIN 3 // TIM2 OC2B
#define M0_DIR_PIN 2
#define M1_PWM_PIN 11 // TIM2 OC2A
#define M1_DIR_PIN 12
#define M2_PWM_PIN 9 // TIM1 OC1A
#define M2_DIR_PIN 8
#define M3_PWM_PIN 10 // TIM1 OC1B
#define M3_DIR_PIN 7
#define ENC0_A_PIN 4
#define ENC0_B_PIN 5
#define ENC1_A_PIN 14
#define ENC1_B_PIN 15
#define ENC2_A_PIN 16
#define ENC2_B_PIN 17
#define ENC3_A_PIN 18
#define ENC3_B_PIN 19
#define LED 13 //led is flashing when message received.

// encoders
volatile int encTicks0;
volatile int encTicks1;
volatile int encTicks2;
volatile int encTicks3;
int encTicks0_prev;
int encTicks1_prev;
int encTicks2_prev;
int encTicks3_prev;
volatile int pwmVal[4];
unsigned long loopTimer;
volatile int timeOutCnt;
volatile int ledCnt = 0;
volatile boolean stopmotors = false; //initially the controller is on.
// function prototype
void disableMotors(void);


nexus_base_ros::Encoders enc_msg;
ros::NodeHandle nh; // instantiate the node handle for ROS
ros::Publisher pub("wheel_vel", &enc_msg); // Using custom message for publishing array of wheel velocities as encoder raw data.


// ROS Callback Function
void cmdMotors_CallBack(const nexus_base_ros::Motors& msg) {

     pwmVal[0] = constrain(msg.motor0, -255, 255);
     pwmVal[1] = constrain(msg.motor1, -255, 255);
     pwmVal[2] = constrain(msg.motor2, -255, 255);
     pwmVal[3] = constrain(msg.motor3, -255, 255);

  if(!stopmotors){
     if(pwmVal[0] >= 0) {
       digitalWriteFast(M0_DIR_PIN, HIGH);
     }
     else {
       digitalWriteFast(M0_DIR_PIN, LOW);
     }

     if(pwmVal[1] >= 0) {
       digitalWriteFast(M1_DIR_PIN, HIGH);
     }
     else {
       digitalWriteFast(M1_DIR_PIN, LOW);
     }

     if(pwmVal[2] >= 0) {
       digitalWriteFast(M2_DIR_PIN, HIGH);
     }
     else {
       digitalWriteFast(M2_DIR_PIN, LOW);
     }

     if(pwmVal[3] >= 0) {
       digitalWriteFast(M3_DIR_PIN, HIGH);
     }
     else {
       digitalWriteFast(M3_DIR_PIN, LOW);
     }
     
     if(timeOutCnt==0) {
       analogWrite(M0_PWM_PIN, abs(pwmVal[0])); // first time write to start PWM
       analogWrite(M1_PWM_PIN, abs(pwmVal[1]));
       analogWrite(M2_PWM_PIN, abs(pwmVal[2]));
       analogWrite(M3_PWM_PIN, abs(pwmVal[3]));
     }
     else {
       OCR2B = abs(pwmVal[0]); // fast PWM update M0
       OCR2A = abs(pwmVal[1]); // fast PWM update M1
       OCR1A = abs(pwmVal[2]); // fast PWM update M2
       OCR1B = abs(pwmVal[3]); // fast PWM update M3
     }
    timeOutCnt = PWD_TIMEOUT_VAL; //set timer
    digitalWriteFast(LED, HIGH);
    ledCnt = 20;
  }
}
ros::Subscriber<nexus_base_ros::Motors> sub("cmd_motor", cmdMotors_CallBack);





// Emergency Stop Callback
void EmergencyStop_CallBack(const nexus_base_ros::EmergencyStopEnableRequest& req, nexus_base_ros::EmergencyStopEnableResponse& res) {
  // handle request
  if(req.enable) {
    disableMotors();
    stopmotors = true;
  }
  // generate response
  res.success = true;
}
ros::ServiceServer<nexus_base_ros::EmergencyStopEnableRequest, nexus_base_ros::EmergencyStopEnableResponse> halt_srv("emergency_stop_enable", &EmergencyStop_CallBack);





// Arming Enable CallBack
void ArmingEnable_CallBack(const nexus_base_ros::ArmingEnableRequest& req, nexus_base_ros::ArmingEnableResponse& res) {
  // handle request
  if(req.enable){
    stopmotors = false;
  }
  // generate response
  res.success = true;
}
ros::ServiceServer<nexus_base_ros::ArmingEnableRequest, nexus_base_ros::ArmingEnableResponse> arming_srv("arming_enable", &ArmingEnable_CallBack);





void disableMotors(){
  // turn motors off by writing a zero
  analogWrite(M0_PWM_PIN, 0);
  analogWrite(M1_PWM_PIN, 0);
  analogWrite(M2_PWM_PIN, 0);
  analogWrite(M3_PWM_PIN, 0);
}





void IsrEnc_0_A() {
  encTicks0 -= digitalReadFast(ENC0_B_PIN) ? -1 : +1; // adjust counter + if A leads B
}

void IsrEnc_1_A() {
  encTicks1 -= digitalReadFast(ENC1_B_PIN) ? -1 : +1; // adjust counter + if A leads B
}

void IsrEnc_2_A() {
  encTicks2 -= digitalReadFast(ENC2_B_PIN) ? -1 : +1; // adjust counter + if A leads B
}

void IsrEnc_3_A() {
  encTicks3 -= digitalReadFast(ENC3_B_PIN) ? -1 : +1; // adjust counter + if A leads B
}





void setup() {
  pinMode(M0_PWM_PIN, OUTPUT); //left front wheel
  pinMode(M0_DIR_PIN, OUTPUT);
  pinMode(M1_PWM_PIN, OUTPUT); //left bottom wheel
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT); //right bottom wheel
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M3_PWM_PIN, OUTPUT); //right front wheel
  pinMode(M3_DIR_PIN, OUTPUT);

  pinMode(ENC0_A_PIN, INPUT); //left front enc
  pinMode(ENC0_B_PIN, INPUT);
  pinMode(ENC1_A_PIN, INPUT); //left bottom enc
  pinMode(ENC1_B_PIN, INPUT);
  pinMode(ENC2_A_PIN, INPUT); //right bottom enc
  pinMode(ENC2_B_PIN, INPUT);
  pinMode(ENC3_A_PIN, INPUT); //right front enc
  pinMode(ENC3_B_PIN, INPUT);

  pinMode(LED, OUTPUT);

  PCattachInterrupt(ENC0_A_PIN, IsrEnc_0_A, RISING);
  PCattachInterrupt(ENC1_A_PIN, IsrEnc_1_A, RISING);
  PCattachInterrupt(ENC2_A_PIN, IsrEnc_2_A, RISING);
  PCattachInterrupt(ENC3_A_PIN, IsrEnc_3_A, RISING);

  // modify PWM frequency of motors
  TCCR1B = (TCCR1B & 0xF8) | 0x01;    // Pin9,Pin10 PWM 31250Hz
  TCCR2B = (TCCR2B & 0xF8) | 0x01;    // Pin3,Pin11 PWM 31250Hz

  nh.initNode();
  nh.subscribe(sub);
  // Publish wheel velocities in multi array, using custom msg.
  nh.advertise(pub);
  nh.advertiseService(halt_srv);
  nh.advertiseService(arming_srv);

  timeOutCnt = PWD_TIMEOUT_VAL;
  loopTimer = micros() + LOOPTIME; //Set the loopTimer variable.
}




void loop() {
  // uncomment for data loop-back test
  //enc_msg.enc0 = pwmVal[0];
  //enc_msg.enc1 = pwmVal[1];
  //enc_msg.enc2 = pwmVal[2];
  //enc_msg.enc3 = pwmVal[3];
  
  // calculate delta-counts for the current cycle.
  enc_msg.enc0 = encTicks0 - encTicks0_prev;
  enc_msg.enc1 = encTicks1 - encTicks1_prev;
  enc_msg.enc2 = encTicks2 - encTicks2_prev;
  enc_msg.enc3 = encTicks3 - encTicks3_prev;

  encTicks0_prev = encTicks0;
  encTicks1_prev = encTicks1;
  encTicks2_prev = encTicks2;
  encTicks3_prev = encTicks3;

  pub.publish(&enc_msg);

  timeOutCnt--;
  if(timeOutCnt <= 0) {
    disableMotors();
    timeOutCnt=0; //keep <=0
  }

  ledCnt--;
  if(ledCnt <= 0) {
    digitalWriteFast(LED, LOW);
  }

  nh.spinOnce();
  
  // Wait for the remaining time in the loop and set the new loopTimer value.
  while(loopTimer > micros()) {;}
  loopTimer += LOOPTIME;
}
