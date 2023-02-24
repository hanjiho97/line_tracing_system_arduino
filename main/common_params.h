#ifndef _COMMON_PARAMS_H_
#define _COMMON_PARAMS_H_

#include <ArduinoSTL.h>

// for debugging
#define _PF_ (__PRETTY_FUNCTION__) 

#define INIT_SPEED 150

#define RIGHT_LINE_SENSOR_PIN A5
#define LEFT_LINE_SENSOR_PIN A0

#define RIGHT_MOTOR_NUMBER 1
#define LEFT_MOTOR_NUMBER 4

#define IR_SENSOR_PIN -1
#define CLOCK_PIN -1

#define LINE_SENSOR_THRESHOLD 500
#define HIGH_MOTOR_SPEED 150
#define LOW_MOTOR_SPEED 100

#define NUMBER_OF_SAMPLES 10

struct SensorData
{
  SensorData()
      : line_tracing_threshold_(0),
        line_tracing_right_(0),
        line_tracing_left_(0),
        ir_value_(0),
        collision_value_(0),
        current_time_(0) {}

  uint16_t line_tracing_threshold_;
  uint16_t line_tracing_right_;
  uint16_t line_tracing_left_;

  uint16_t ir_value_;
  uint16_t collision_value_;

  uint32_t current_time_;
};

#endif