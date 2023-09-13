/**********************************************************************
 * RobotZero Peripheral Basics
 * This program illustrates the abilities of the RobotZero processor 
 * board by spinning any attached motors or servos, while reading the
 * accelerometer data of the onboard 9-Axis sensor and Color
 * Sensor data for a Color Sensor Wireling attached to port 0. 
 * 
 * NOTES: 
 *   - Serial Monitor must be open for program to run.
 *   - Battery must be plugged in to power motors.
 * 
 * Hardware by: TinyCircuits
 * Written by: Ben Rose & Laveréna Wienclaw for TinyCircuits
 * 
 * Initialized: June 2019
 * Last modified: Jan 2020
 **********************************************************************/

#include <Wire.h>
#include <ServoDriver.h> // Download latest here: https://github.com/TinyCircuits/TinyCircuits-TinyShield_Motor_Library/archive/master.zip
//#include <Wireling.h>
//#include "Adafruit_TCS34725.h"  // The library used for the Color Sensor Wireling

ServoDriver servo(15);// Value passed is the address- RobotZero is always address 15
#define SerialMonitorInterface SerialUSB // for SAMD21 processors

const int HALT_SPEED = 1500;

//int aX, aY, aZ, gX, gY, gZ, mX, mY, mZ, tempF; // 9-Axis variables 
//uint16_t r, g, b, c, colorTemp, lux; // Color Sensor Wireling variables
//
//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

void setup() {
//  // Enable and Power Wireling
//  Wireling.begin();
//  Wireling.selectPort(0);

  SerialMonitorInterface.begin(9600);
 // while (!SerialMonitorInterface); // Halt everything until Serial Monitor is opened
  Wire.begin();

  // Initialize servo driver
  if(servo.begin(20000)){
    while(1);
    SerialMonitorInterface.println("Servo driver not detected!");
  }
}

void loop() {
//  tcs.setInterrupt(false); // Turn onboard Color Sensor Wireling LEDs off
//  drive(HALT_SPEED, HALT_SPEED, 500);
//  delay(500);
//  drive(HALT_SPEED + 50, HALT_SPEED - 50, 50);
////  delay(500);
////  drive(HALT_SPEED, HALT_SPEED, 50);
////  delay(500);
//  drive(HALT_SPEED + 150, HALT_SPEED - 150, 50);
////  delay(500);
////  drive(HALT_SPEED, HALT_SPEED, 50);
////  delay(500);
//  drive(HALT_SPEED + 250, HALT_SPEED - 250, 50);
////  delay(500);
////  drive(HALT_SPEED, HALT_SPEED, 50);
////  delay(500);
//  drive(HALT_SPEED + 350, HALT_SPEED - 350, 50);
////  delay(500);
////  drive(HALT_SPEED, HALT_SPEED, 50);
//  //delay(500);
  drive(HALT_SPEED - 500, HALT_SPEED - 500, 50);
  //delay(5000);

//  drive(HALT_SPEED, HALT_SPEED, 50);
//  delay(500);
//  drive(HALT_SPEED - 50, HALT_SPEED + 50, 50);
//  delay(500);
//  drive(HALT_SPEED, HALT_SPEED, 50);
//  delay(500);
//  drive(HALT_SPEED - 150, HALT_SPEED + 150, 50);
//  delay(500);
//  drive(HALT_SPEED, HALT_SPEED, 50);
//  delay(500);
//  drive(HALT_SPEED - 250, HALT_SPEED + 250, 50);
//  delay(500);
//  drive(HALT_SPEED, HALT_SPEED, 50);
//  delay(500);
//  drive(HALT_SPEED - 350, HALT_SPEED + 350, 50);
//  delay(500);
//  drive(HALT_SPEED, HALT_SPEED, 50);
//  delay(500);
//  drive(HALT_SPEED - 450, HALT_SPEED + 450, 50);
//  delay(5000);
//  drive(2000, 1000, 50);
//  delay(500);

}

void drive(int servo_left_power, int servo_right_power, int duration)
{
  //Forward is left > 1500, right < 1500.  1500 is full stop.
  //Servo 1 is the Left
  //Servo 4 is the Right
  int servo_left_port  = 1;
  int servo_right_port = 2;

//  if(servo_left_power > HALT_SPEED || servo_left_power < HALT_SPEED)
//  {
//    servo_left_power = servo_left_power * 0.95;
//  }

  servo.setServo(servo_left_port, servo_left_power);
  servo.setServo(servo_right_port, servo_right_power);
  delay(duration);

}
