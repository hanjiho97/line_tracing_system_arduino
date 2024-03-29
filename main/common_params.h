#ifndef _COMMON_PARAMS_H_
#define _COMMON_PARAMS_H_

#include <ArduinoSTL.h>
#include <AFMotor.h>

// for debugging
#define _PF_ __PRETTY_FUNCTION__

#define INIT_SPEED 150

// line sensors
#define LEFT_LINE_SENSOR_PIN 2
#define RIGHT_LINE_SENSOR_PIN 3

// ir sensor
#define IR_SENSOR_PIN A0
#define IR_DETECTED 0
#define IR_NOT_DETECTED 1

// collision sensor
#define COLLISION_SENSOR_PIN A1
#define COLLISION_DETECTED_THRESHOLD 500

// motors
#define RIGHT_MOTOR_NUMBER 1
#define LEFT_MOTOR_NUMBER 4

#define DISPLAY_SDA_PIN A4
#define DISPLAY_SCL_PIN A5

// line tracing parameters
#define LINE_SENSOR_THRESHOLD 500
#define NUMBER_OF_SAMPLES 10

// motor control parameters
#define HIGH_MOTOR_SPEED 250
#define LOW_MOTOR_SPEED 200

// state related paramters
#define START_WAIT_TIME_MS 5000
#define STOP_WAIT_TIME_MS 5000
#define EMERGENCY_STOP_NONE_LINE_LIMIT_TIME_MS 5000
#define RECOVERY_NONE_LINE_LIMIT_TIME_MS 6000
#define AVOIDACNE_NONE_LINE_LIMIT_TIME_MS 1000
#define DONE_TIME_MS 600000

// avoidance
#define FIRST_CHECKPOINT_TIME_MS 1000
#define SECOND_CHECKPOINT_TIME_MS 3000
#define THRID_CHECKPOINT_TIME_MS 4000
#define FOURTH_CHECKPOINT_TIME_MS 8000
#define FIFTH_CHECKPOINT_TIME_MS 9000

// sensor fault
#define FAULT_DETECTION_THRESHOLD 1000
#define FAULT_COUNT_THRESHOLD 10

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
  INIT = 0,
  STOP = 1, // main decision state
  LINE_FOLLOW = 2,
  OBSTACLE_AVOIDANCE = 3,
  COLLISION = 4,
  SYSTEM_FAULT = 5,
  EMERGENCY_STOP = 6,
  NORMAL_TERMINATION = 7,
  ABNORMAL_TERMINATION = 8,
  RECOVERY = 9,
  NUM_STATES
};

// const std::string STATE_STR[] = 
// {
//   "INIT",
//   "STOP",
//   "LINE_FOLLOW",
//   "OBSTACLE_AVOIDANCE",
//   "COLLISION",
//   "SYSTEM_FAULT",
//   "EMERGENCY_STOP",
//   "NORMAL_TERMINATION",
//   "ABNORMAL_TERMINATION",
//   "RECOVERY"
// };

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
  MotorOutput()
    : right_motor_speed_(0)
    , left_motor_speed_(0)
    , right_motor_mode_(RELEASE)
    , left_motor_mode_(RELEASE) {}
  uint8_t right_motor_speed_;
  uint8_t left_motor_speed_;
  uint8_t right_motor_mode_;
  uint8_t left_motor_mode_;
};

struct DisplayOutput
{
  DisplayOutput()
    : right_motor_speed_(0)
    , left_motor_speed_(0)
    , right_motor_mode_(RELEASE)
    , left_motor_mode_(RELEASE) {}
  uint8_t right_motor_speed_;
  uint8_t left_motor_speed_;
  uint8_t right_motor_mode_;
  uint8_t left_motor_mode_;
};

#endif
