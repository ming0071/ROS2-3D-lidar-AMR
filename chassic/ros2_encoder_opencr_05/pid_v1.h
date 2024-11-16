#ifndef PID_V1_H
#define PID_V1_H

#include <PID_v1.h>
#include "motor_driver.h"

// Declaration for setting motor speeds
void setSpeeds(int m1Speed, int m2Speed);

// Structure to store PID-related information
typedef struct
{
  double target;         // Desired target speed
  double currentEncoder; // Current encoder reading
  double lastEncoder;    // Previous encoder reading
  double error;          // Error between target and input
  double input;          // Process variable for PID
  double output;         // PID output value
} PIDInfo;

PIDInfo leftInfo, rightInfo; // PID information for left and right motors

// PID parameters configuration
double Kp_L = 0.3, Ki_L = 0.007, Kd_L = 0.0;   // Left motor PID gains
double Kp_R = 0.315, Ki_R = 0.007, Kd_R = 0.0; // Right motor PID gains

PID leftPID(&leftInfo.input, &leftInfo.output, &leftInfo.target, Kp_L, Ki_L, Kd_L, DIRECT);
PID rightPID(&rightInfo.input, &rightInfo.output, &rightInfo.target, Kp_R, Ki_R, Kd_R, DIRECT);

// PID calculation frequency and interval
double pidRate = 100.0;                // 100 Hz
double pidinterval = 1000.0 / pidRate; // 10 ms interval for PID computation

long nextmotion; // Time tracker for next PID computation

// Function to reset encoder and PID-related variables
void resetPIDInfo()
{
  leftInfo.currentEncoder = 0;
  leftInfo.lastEncoder = 0;
  rightInfo.currentEncoder = 0;
  rightInfo.lastEncoder = 0;
}

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

  resetPIDInfo(); // Reset PID information before starting
}

// PID computation function, called at the specified interval
void compute_PID()
{
  // Update and calculate left motor PID
  leftInfo.currentEncoder = leftEncoder.speedMsg.data;
  leftInfo.input = leftInfo.currentEncoder - leftInfo.lastEncoder; // Compute input change
  leftInfo.error = leftInfo.target - leftInfo.input;               // Compute error
  leftPID.Compute();                                               // Perform PID calculation
  leftInfo.lastEncoder = leftEncoder.speedMsg.data;                // Store last encoder value

  // Update and calculate right motor PID
  rightInfo.currentEncoder = rightEncoder.speedMsg.data;
  rightInfo.input = rightInfo.currentEncoder - rightInfo.lastEncoder; // Compute input change
  rightInfo.error = rightInfo.target - rightInfo.input;               // Compute error
  rightPID.Compute();                                                 // Perform PID calculation
  rightInfo.lastEncoder = rightEncoder.speedMsg.data;                 // Store last encoder value

  // Set motor speeds based on PID output
  setSpeeds(leftInfo.output, rightInfo.output);

  // Update next motion timestamp
  nextmotion = millis() + pidinterval;
}

#endif