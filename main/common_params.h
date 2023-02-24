#ifndef _COMMON_PARAMS_H_
#define _COMMON_PARAMS_H_

#include <ArduinoSTL.h>
#include <AFMotor.h>

// for debugging
#define _PF_ __PRETTY_FUNCTION__

#define INIT_SPEED 150

// both line sensors pin number
#define RIGHT_LINE_SENSOR_PIN A5
#define LEFT_LINE_SENSOR_PIN A0

// ir sensor pin number
#define IR_SENSOR_PIN 9

// both motors number
#define RIGHT_MOTOR_NUMBER 1
#define LEFT_MOTOR_NUMBER 4

#define CLOCK_PIN -1

// line tracing parameters
#define LINE_SENSOR_THRESHOLD 500
#define NUMBER_OF_SAMPLES 10

// motor control parameters
#define HIGH_MOTOR_SPEED 150
#define LOW_MOTOR_SPEED 100

// state related paramters
#define START_WAIT_TIME_MS 5000

enum STATE_TYPE
{
  INVALID_STATE = -1,
  INIT,
  STOP, // main decision state
  LINE_FOLLOW,
  PARKING,
  AVOIDANCE,
  COLLISION_STOP,
  THEFT_EMERGENCY,
  EMERGENCY_STOP,
  DONE,
  NUM_STATES
};

// const char* STATES_STRING[]
// {
//   "INIT_STATE",
//   "STOP_STATE",
//   "LINE_FOLLOW_STATE",
//   "PARKING_STATE",
//   "AVOIDANCE_STATE",
//   "COLLISION_STOP_STATE",
//   "THEFT_EMERGENCY_STATE",
//   "EMERGENCY_STOP_STATE",
//   "DONE_STATE"
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

struct MotorOuput
{
  uint8_t right_motor_speed_;
  uint8_t left_motor_speed_;
  uint8_t right_motor_mode_;
  uint8_t left_motor_mode_;
};

#endif
