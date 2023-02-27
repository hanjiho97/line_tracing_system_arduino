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
#define IR_DETECTED_THRESHOLD 500

// collision sensor
#define COLLISION_SENSOR_PIN A1
#define COLLISION_DETECTED_THRESHOLD 600

// motors
#define RIGHT_MOTOR_NUMBER 3
#define LEFT_MOTOR_NUMBER 4

#define DISPLAY_SDA_PIN A4
#define DISPLAY_SCL_PIN A5

// line tracing parameters
#define LINE_SENSOR_THRESHOLD 500
#define NUMBER_OF_SAMPLES 10

// motor control parameters
#define HIGH_MOTOR_SPEED 200
#define LOW_MOTOR_SPEED 200

// state related paramters
#define START_WAIT_TIME_MS 5000
#define RECOVERY_NONE_LINE_LIMIT_TIME_MS 5000

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
  INIT,
  STOP, // main decision state
  LINE_FOLLOW,
  COLLISION,
  SYSTEM_FAULT,
  RECOVERY,
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

#endif
