#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
#include "pid_v1.h"

// Motor control pins for left motor
#define LEFT_SV 5 // Left motor PWM speed control pin
#define LEFT_FR 6 // Left motor forward/reverse control pin
#define LEFT_EN 7 // Left motor enable pin

// Motor control pins for right motor
#define RIGHT_SV 10 // Right motor PWM speed control pin
#define RIGHT_FR 11 // Right motor forward/reverse control pin
#define RIGHT_EN 12 // Right motor enable pin

#define PI 3.1415926

// Custom structure to store calculated motor and servo target values
struct TwistControl
{
  int leftPulsePerInterval = 0;
  int rightPulsePerInterval = 0;
} twistTemp;

const double encoderResolution = 40000.0; // Encoder pulses per revolution (2*2*500*20 = 40000)
const double wheelDiameter = 0.120;       // Wheel diameter in meters
const double wheelBase = 0.36;            // Distance between rear wheels in meters


// Function to control the speed and direction of motors
void setSpeeds(int leftSpeed, int rightSpeed)
{
  // Control left motor
  if (leftSpeed > 0)
  {
    digitalWrite(LEFT_FR, HIGH);
    digitalWrite(LEFT_EN, LOW);
    analogWrite(LEFT_SV, leftSpeed);
  }
  else if (leftSpeed < 0)
  {
    digitalWrite(LEFT_FR, LOW);
    digitalWrite(LEFT_EN, LOW);
    analogWrite(LEFT_SV, -leftSpeed);
  }
  else
  {
    digitalWrite(LEFT_FR, LOW);
    digitalWrite(LEFT_EN, HIGH);
    analogWrite(LEFT_SV, 0);
  }

  // Control right motor
  if (rightSpeed > 0)
  {
    digitalWrite(RIGHT_FR, LOW);
    digitalWrite(RIGHT_EN, LOW);
    analogWrite(RIGHT_SV, rightSpeed);
  }
  else if (rightSpeed < 0)
  {
    digitalWrite(RIGHT_FR, HIGH);
    digitalWrite(RIGHT_EN, LOW);
    analogWrite(RIGHT_SV, -rightSpeed);
  }
  else
  {
    digitalWrite(RIGHT_FR, LOW);
    digitalWrite(RIGHT_EN, HIGH);
    analogWrite(RIGHT_SV, 0);
  }
}

// Function to initialize motor pins
void Init_Motors()
{
  pinMode(LEFT_SV, OUTPUT);
  pinMode(LEFT_FR, OUTPUT);
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_SV, OUTPUT);
  pinMode(RIGHT_FR, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);

  setSpeeds(0, 0);
}

// Set target pulses for the PID controller
void setTargetTicksPerFrame(int left, int right)
{
  if (left == 0 && right == 0)
  {
    setSpeeds(0, 0);
  }
  leftInfo.target = left;
  rightInfo.target = right;
  // Reset Twist temp
  twistTemp.leftPulsePerInterval = 0;
  twistTemp.rightPulsePerInterval = 0;
}
// Convert twist commands to motor target pulses
void twistToPulseAndServo(double linearVel, double angularVel)
{
  double vLeft = 0.0;
  double vRight = 0.0;

  // Case 1: Both linear and angular velocities are zero → stop motion
  if (linearVel == 0.0 && angularVel == 0.0) {
    setTargetTicksPerFrame(0, 0); // Set motor ticks to zero
    return;
  }

  // Case 2: In-place rotation (linearVel = 0 but angularVel ≠ 0)
  if (linearVel == 0.0) {
    vLeft = -angularVel * wheelBase / 2.0;
    vRight = angularVel * wheelBase / 2.0;
  } 
  else {
    // Case 3: Normal motion (including curved paths)
    vLeft = (1 - (wheelBase * angularVel) / (2.0 * linearVel)) * linearVel;
    vRight = (1 + (wheelBase * angularVel) / (2.0 * linearVel)) * linearVel;
  }

  // Convert wheel velocities to encoder ticks per control interval
  int leftTicks = static_cast<int>((vLeft * (encoderResolution / (PI * wheelDiameter))) / pidRate);
  int rightTicks = static_cast<int>((vRight * (encoderResolution / (PI * wheelDiameter))) / pidRate);

  // Send the target ticks to motor control
  setTargetTicksPerFrame(leftTicks, rightTicks);
}

#endif
