#ifndef __ROS_DRIVER_H__
#define __ROS_DRIVER_H__

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include "motor_driver.h"

// Macro to check and handle RCL initialization errors
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

// ROS node components: publishers, subscribers, timers, and executor
rcl_subscription_t cmdVelSubscriber;
rcl_publisher_t encoderLeftPublisher;
rcl_publisher_t encoderRightPublisher;
rcl_publisher_t pwmLeftPublisher;
rcl_publisher_t pwmRightPublisher;
rcl_timer_t leftEncoderTimer;
rcl_timer_t rightEncoderTimer;
rcl_timer_t leftPwmTimer;
rcl_timer_t rightPwmTimer;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ROS messages for encoder and PWM data
std_msgs__msg__Float32 encoderLeftMsg;
std_msgs__msg__Float32 encoderRightMsg;
std_msgs__msg__Float32 pwmLeftMsg;
std_msgs__msg__Float32 pwmRightMsg;
std_msgs__msg__Float32 pwmMsg;

geometry_msgs__msg__Twist twist_msg;

long lastMotorCommand;

// Error loop to prevent further execution in case of setup failure
void error_loop()
{
  while (1)
  {
    delay(100); // Delay to avoid busy looping
  }
}

// Timer callback to subscribe cmd_vel topic
void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  double Velocity = msg->linear.x;
  double Angular = msg->angular.z;
  twistToPulseAndServo(Velocity, Angular);

  lastMotorCommand = millis(); // record the time when cmd_vel is received
}

// Timer callback to publish left encoder data
void leftEncoderTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    encoderLeftMsg.data = leftEncoder.speedMsg.data; // Retrieve left encoder data
    RCSOFTCHECK(rcl_publish(&encoderLeftPublisher, &encoderLeftMsg, NULL));
  }
}

// Timer callback to publish right encoder data
void rightEncoderTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    encoderRightMsg.data = rightEncoder.speedMsg.data; // Retrieve right encoder data
    RCSOFTCHECK(rcl_publish(&encoderRightPublisher, &encoderRightMsg, NULL));
  }
}

// Timer callback to publish left pwm data
void leftPwmTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    pwmLeftMsg.data = leftInfo.output;
    RCSOFTCHECK(rcl_publish(&pwmLeftPublisher, &pwmLeftMsg, NULL));
  }
}

// Timer callback to publish right pwm data
void rightPwmTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    pwmRightMsg.data = rightInfo.output;
    RCSOFTCHECK(rcl_publish(&pwmRightPublisher, &pwmRightMsg, NULL));
  }
}

// General function for publisher initialization
void initializePublisher(rcl_publisher_t &publisher, const char *topic_name)
{
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      topic_name));
}

// function for subscriber initialization
void twistSubscriber(rcl_subscription_t &subscriber, const char *topic_name,
                     void (*callback)(const void *))
{
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      topic_name));
  // Add the subscriber to the executor for handling received messages
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, callback, ON_NEW_DATA));
}

// General function for timer initialization
void initializeTimer(rcl_timer_t &timer, unsigned int timeout_ms, void (*callback)(rcl_timer_t *, int64_t))
{
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timeout_ms),
      callback));
  // Add the timer to the executor for periodic callbacks
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

// Initializes the ROS node, publishers, subscribers, timers, and executor
void Init_ROS_Node()
{
  Serial.begin(115200);
  set_microros_transports(); // Set up micro-ROS communication transport
  allocator = rcl_get_default_allocator();

  // Initialize ROS 2 support structure
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create the ROS 2 node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // Initialize executor with support context
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));

  // Initialize encoder data publishers
  initializePublisher(encoderLeftPublisher, "encoder_left");
  initializePublisher(encoderRightPublisher, "encoder_right");
  initializePublisher(pwmLeftPublisher, "pwm_left");
  initializePublisher(pwmRightPublisher, "pwm_right");

  // Initialize timers for encoder data publishing
  const unsigned int timerTimeout = 50; // Timer interval in ms
  initializeTimer(leftEncoderTimer, timerTimeout, leftEncoderTimerCallback);
  initializeTimer(rightEncoderTimer, timerTimeout, rightEncoderTimerCallback);
  initializeTimer(leftPwmTimer, timerTimeout, leftPwmTimerCallback);
  initializeTimer(rightPwmTimer, timerTimeout, rightPwmTimerCallback);

  // Initialize cmd_vel subscriber
  twistSubscriber(cmdVelSubscriber, "cmd_vel", cmd_vel_callback);
}

#endif