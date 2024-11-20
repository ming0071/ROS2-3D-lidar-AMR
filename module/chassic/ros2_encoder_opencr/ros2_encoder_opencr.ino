#include "encoder_driver.h" // Custom driver for handling encoder functions, such as reading encoder data
#include "motor_driver.h"   // Custom driver for controlling motors, including PWM and speed control
#include "ros_driver.h"     // Custom driver for initializing and managing ROS communication
#include "pid_driver.h"         // Custom driver for PID control to regulate motor speed

#define AUTO_STOP_INTERVAL 1500 // Time (in milliseconds) to stop motors if no new command is received

int moving; // Flag indicating whether PID is active

void setup()
{
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate for debugging or logging
  Init_Motors();        // Set up motor control, including configuring PWM channels
  Init_PID();           // Initialize and configure PID controllers for motor speed regulation
  Init_Encoder();       // Initialize encoders to monitor wheel/motor position and speed
  Init_ROS_Node();      // Initialize ROS node and configure publishers and subscribers
}

void loop()
{
  // Execute PID control if the scheduled time interval has passed and motion is active
  if (nextmotion <= millis() && moving == 1)
  {
    compute_PID(); // Perform PID computation and update motor outputs
  }

  // Stop the robot if no motor command is received within the defined AUTO_STOP_INTERVAL
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  {
    setTargetTicksPerFrame(0, 0); // Set target motor ticks to zero to stop motion
    moving = 0;
  }
  else
  {
    moving = 1;
  }

  // Process incoming ROS messages and execute callbacks
  // Equivalent to spin_once in ROS 1, with a 10 ms timeout for handling tasks
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
