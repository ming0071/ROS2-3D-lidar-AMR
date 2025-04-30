#ifndef PID_V1_H
#define PID_V1_H

#include <PID_v1.h>
#include "motor_driver.h"

// Declaration for setting motor speeds
void setSpeeds(int m1Speed, int m2Speed);

// Structure to store PID-related information
typedef struct
{
  double target; // Desired target speed
  double input;  // Process variable for PID
  double output; // PID output value
} PIDInfo;

PIDInfo leftInfo, rightInfo; // PID information for left and right motors

// PID parameters configuration
double Kp_L = 0.6, Ki_L = 0.0, Kd_L = 0.0; // Left motor PID gains I = 0.007?
double Kp_R = 0.64, Ki_R = 0.0, Kd_R = 0.0;   // Right motor PID gains I = 0.007?

PID leftPID(&leftInfo.input, &leftInfo.output, &leftInfo.target, Kp_L, Ki_L, Kd_L, DIRECT);
PID rightPID(&rightInfo.input, &rightInfo.output, &rightInfo.target, Kp_R, Ki_R, Kd_R, DIRECT);

// PID calculation frequency and interval
double pidRate = 100.0;                // 100 Hz
double pidinterval = 1000.0 / pidRate; // 10 ms interval for PID computation

long nextMotion; // Time tracker for next PID computation

// PID initialization function
void Init_PID()
{
  // Configure left motor PID
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(pidinterval);
  leftPID.SetOutputLimits(-255, 255); // Set motor output limits

  // Configure right motor PID
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(pidinterval);
  rightPID.SetOutputLimits(-255, 255);

  leftInfo.input = 0;
  leftInfo.target = 0;
  leftInfo.output = 0;

  rightInfo.input = 0;
  rightInfo.target = 0;
  rightInfo.output = 0;

  nextMotion = millis();
}

// PID computation function, called at the specified interval
void compute_PID()
{
  // Update left motor PID
  leftInfo.input = leftEncoder.speedMsg.data; // Use current encoder speed as input
  leftPID.Compute();                          // Perform PID calculation

  // Update right motor PID
  rightInfo.input = rightEncoder.speedMsg.data; // Use current encoder speed as input
  rightPID.Compute();                           // Perform PID calculation

  // Set motor speeds based on PID output
  setSpeeds(leftInfo.output, rightInfo.output);

  // Update next motion timestamp
  nextMotion = millis() + pidinterval;
}

#endif