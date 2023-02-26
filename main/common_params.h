#ifndef _COMMON_PARAMS_H_
#define _COMMON_PARAMS_H_

#include <ArduinoSTL.h>
#include <AFMotor.h>


// for debugging
#define _PF_ __PRETTY_FUNCTION__

#define INIT_SPEED 150

// line sensors
#define RIGHT_LINE_SENSOR_PIN A5
#define LEFT_LINE_SENSOR_PIN A0

// ir sensor 
#define IR_SENSOR_PIN 9
// #define IR_DETECTED 0
// #define IR_NOT_DETECTED 1

// collision sensor
#define COLLISION_SENSOR_PIN -1

// motors
#define RIGHT_MOTOR_NUMBER 1
#define LEFT_MOTOR_NUMBER 4

#define DISPLAY_PIN -1

// line tracing parameters
#define LINE_SENSOR_THRESHOLD 500
#define NUMBER_OF_SAMPLES 10

// motor control parameters
#define HIGH_MOTOR_SPEED 150
#define LOW_MOTOR_SPEED 100

// state related paramters
#define START_WAIT_TIME_MS 5000
#define STOP_WAIT_TIME_MS 5000
#define NONE_LANE_TIME_MS 5000
#define DONE_TIME_MS 600000

// sensor fault
#define FAULT_DETECTION_THRESHOLD 1000

enum DIRECTION
{
  STRAGHIT,
  RIGHT,
  LEFT,
  NONE
};

enum STATE_TYPE
{
  INVALID_STATE = -1,
  INIT,
  STOP, // main decision state
  LINE_FOLLOW,
  OBSTACLE_AVOIDANCE,
  COLLISION,
  SYSTEM_FAULT,
  EMERGENCY_STOP,
  NORMAL_TERMINATION,
  ABNORMAL_TERMINATION,
  SYSTEM_RECOVERY,
  NUM_STATES
};

struct SensorData
{
  SensorData()
      : line_tracing_right_(0),
        line_tracing_left_(0),
        ir_value_(0),
        collision_value_(0),
        read_time_(0) {}

  uint16_t line_tracing_right_;
  uint16_t line_tracing_left_;

  uint16_t ir_value_;
  uint16_t collision_value_;

  uint32_t read_time_;
};

struct MotorOutput
{
  uint8_t right_motor_speed_;
  uint8_t left_motor_speed_;
  uint8_t right_motor_mode_;
  uint8_t left_motor_mode_;
};

struct DisplayOutput
{
  uint8_t right_motor_speed_;
  uint8_t left_motor_speed_;
  uint8_t right_motor_mode_;
  uint8_t left_motor_mode_;
};

#endif
