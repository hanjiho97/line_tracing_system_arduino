#ifndef _COMMON_PARAMS_H_
#define _COMMON_PARAMS_H_

#include <ArduinoSTL.h>

struct Param
{
  const uint8_t IR_PIN = 0; // IR sensor pin
  const uint8_t LT_PIN = 0; // line tracing sensor pin
  const uint8_t CL_PIN = 0; // collision sensor pin
};

struct SensorData
{
  SensorData()
      : line_tracing_threshold_(0),
        line_tracing_right_(0),
        line_tracing_left_(0),
        ir_value_(0),
        collision_value_(0) {}

  uint16_t line_tracing_threshold_;
  uint16_t line_tracing_right_;
  uint16_t line_tracing_left_;

  uint16_t ir_value_;
  uint16_t collision_value_;
};

#endif