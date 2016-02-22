#ifndef MjxConfig_h
#define MjxConfig_h


// radio pins
#define MJX_RADIO_CE_PIN 8
#define MJX_RADIO_CSN_PIN 7

// PWM output pins
#define MJX_CONTROLLER_THROTLE_PIN 3   // throttle
#define MJX_CONTROLLER_YAW_PIN 5       // tail rotate, (gyro_z)
#define MJX_CONTROLLER_PITCH_PIN 10    // move forward/backward, (gyro_y)
#define MJX_CONTROLLER_ROLL_PIN 9      // move left/right, (gyro_x)

#define MJX_UPDATE_INTERVAL 20         // update interval [ms]

#define MJX_SERVO_THROTTLE_SPEED 0.0   // degrees per second, 0 max speed
#define MJX_SERVO_YAW_SPEED 0.0        // degrees per second, 0 max speed
#define MJX_SERVO_PITCH_SPEED 300.0    // degrees per second, 0 max speed
#define MJX_SERVO_ROLL_SPEED 300.0     // degrees per second, 0 max speed

#define MJX_CONTROLLER_YAW_PID_KP 1
#define MJX_CONTROLLER_YAW_PID_KI 0.4
#define MJX_CONTROLLER_YAW_PID_KD 0.15  // 0.2

#define MJX_CONTROLLER_YAW_PID_GYRO_RATE 0.1
#define MJX_CONTROLLER_YAW_PID_I_RATE 0.5

#endif
