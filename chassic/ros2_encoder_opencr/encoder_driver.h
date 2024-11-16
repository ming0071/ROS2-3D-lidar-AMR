#ifndef __ENCODER_DRIVER_H__
#define __ENCODER_DRIVER_H__

#include <micro_ros_arduino.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>

// Pin definitions for encoders
#define EncoderLeftA 3  // Left encoder channel A pin
#define EncoderLeftB 4  // Left encoder channel B pin
#define EncoderRightA 8 // Right encoder channel A pin
#define EncoderRightB 9 // Right encoder channel B pin

// Constants for encoder calculations
const double PULSES_PER_REVOLUTION = 753982.23686; // Adjusted for gear ratio or resolution
const int PULSE_THRESHOLD = 10;                    // Minimum pulse count required for speed calculation
const int VELOCITY_DIVISOR = 20;                   // Divisor to scale computed velocity

// Structure to store encoder properties and calculated data
typedef struct
{
    int state;                       // Current state of the encoder pin
    int lastState;                   // Previous state of the encoder pin
    int value;                       // Encoder pulse count
    long previousMicros;             // Timestamp of the last valid pulse (in microseconds)
    long currentMicros;              // Current timestamp (in microseconds)
    double timeElapsed;              // Time interval between pulses (in microseconds)
    int direction;                   // Direction of rotation (+1 for forward, -1 for reverse)
    std_msgs__msg__Float32 speedMsg; // ROS message containing the calculated speed
    int directionFactor;             // Direction scaling factor (+1 for right, -1 for left)
} EncoderData;

// Encoder data instances for left and right encoders
EncoderData leftEncoder = {0, 0, 0, 0, 0, 0, 0, {0.0}, 1};   // Left encoder instance
EncoderData rightEncoder = {0, 0, 0, 0, 0, 0, 0, {0.0}, -1}; // Right encoder instance

// Generalized encoder interrupt service routine (ISR)
// Handles pulse counting, direction determination, and speed calculation
void encoderISR(EncoderData &encoder, int pinA, int pinB)
{
    encoder.state = digitalRead(pinA); // Read the current state of the encoder pin A

    // Detect state change (pulse detected)
    if (encoder.state != encoder.lastState)
    {
        // Update pulse count based on direction determined by pin B
        if (digitalRead(pinB) != encoder.state)
        {
            encoder.value += 1; // Increment pulse count
        }
        else
        {
            encoder.value -= 1; // Decrement pulse count
        }
    }

    // Record start time for speed calculation if a new pulse is detected
    if (encoder.value == 1 || encoder.value == -1)
    {
        encoder.previousMicros = micros(); // Save the time of the first pulse
    }

    // Calculate speed when pulse count reaches the threshold
    if (abs(encoder.value) > PULSE_THRESHOLD)
    {
        // Calculate direction based on the encoder value
        if (encoder.value > 0)
        {
            encoder.direction = encoder.directionFactor; // Set direction based on directionFactor for forward rotation
        }
        else
        {
            encoder.direction = -encoder.directionFactor; // Set direction based on directionFactor for reverse rotation
        }

        encoder.currentMicros = micros();                                     // Record the current time
        encoder.timeElapsed = encoder.currentMicros - encoder.previousMicros; // Compute time interval

        // Compute speed and update ROS message
        encoder.speedMsg.data = (encoder.direction * PULSES_PER_REVOLUTION) / encoder.timeElapsed / VELOCITY_DIVISOR;

        // Reset values for the next calculation
        encoder.value = 0;
        encoder.direction = 0;
    }

    // Update last state for the next ISR invocation
    encoder.lastState = encoder.state;
}

// Left encoder ISR function
// Triggered by a change in the left encoder's channel A pin
void leftEncoderISR()
{
    encoderISR(leftEncoder, EncoderLeftA, EncoderLeftB);
}

// Right encoder ISR function
// Triggered by a change in the right encoder's channel A pin
void rightEncoderISR()
{
    encoderISR(rightEncoder, EncoderRightA, EncoderRightB);
}

// Function to initialize encoders
// Sets up pin modes and attaches interrupts
void Init_Encoder()
{
    // Configure encoder pins as inputs
    pinMode(EncoderLeftA, INPUT);
    pinMode(EncoderLeftB, INPUT);
    pinMode(EncoderRightA, INPUT);
    pinMode(EncoderRightB, INPUT);

    // Attach interrupts to channel A pins for state change detection
    attachInterrupt(digitalPinToInterrupt(EncoderLeftA), leftEncoderISR, CHANGE);
    leftEncoder.lastState = digitalRead(EncoderLeftA); // Record initial state of left encoder

    attachInterrupt(digitalPinToInterrupt(EncoderRightA), rightEncoderISR, CHANGE);
    rightEncoder.lastState = digitalRead(EncoderRightA); // Record initial state of right encoder
}

#endif
