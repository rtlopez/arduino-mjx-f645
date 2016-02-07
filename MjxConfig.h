#ifndef MjxConfig_h
#define MjxConfig_h

#include "MPU6050.h"

#define MJX_DEBUG_CONTROLLER 0 //500     // debug controller values to serial, value in milliseconds, 0 to disable
#define MJX_DEBUG_PID 0 //50             // debug PID controller values to serial, value in milliseconds, 0 to disable
#define MJX_DEBUG_SERVO 0 //20           // debug SERVO

// radio pins
#define MJX_RADIO_CE_PIN 8
#define MJX_RADIO_CSN_PIN 7

// PWM output pins
#define MJX_CONTROLLER_THROTLE_PIN 3   // throttle
#define MJX_CONTROLLER_YAW_PIN 5       // tail rotate, (gyro_z)
#define MJX_CONTROLLER_PITCH_PIN 10    // move forward/backward, (gyro_y)
#define MJX_CONTROLLER_ROLL_PIN 9      // move left/right, (gyro_x)

#define MJX_SERVO_INTERVAL 19          // update interval [ms]
#define MJX_SERVO_THROTTLE_SPEED 0.0   // degrees per second, 0 max speed
#define MJX_SERVO_YAW_SPEED 0.0        // degrees per second, 0 max speed
#define MJX_SERVO_PITCH_SPEED 300.0    // degrees per second, 0 max speed
#define MJX_SERVO_ROLL_SPEED 300.0     // degrees per second, 0 max speed

#define MJX_CONTROLLER_YAW_PID_KP 1
#define MJX_CONTROLLER_YAW_PID_KI 0.4
#define MJX_CONTROLLER_YAW_PID_KD 0.15  // 0.2
#define MJX_CONTROLLER_YAW_PID_GYRO_RATE 0.1
#define MJX_CONTROLLER_YAW_PID_I_RATE 0.5

#define MJX_POSITION_DLPF MPU6050_DLPF_1  // gyro low pass filter

#endif
