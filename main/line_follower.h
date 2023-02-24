#ifndef _LINE_FOLLOWER_H
#define _LINE_FOLLOWER_H

#include "common_params.h"
#include "moving_average_filter.h"

#include <Arduino.h>
#include <AFMotor.h>

class LineFollower
{
private:
  MovingAverageFilter *right_filter_ptr_;
  MovingAverageFilter *left_filter_ptr_;
  
  uint16_t right_sensor_value_;
  uint16_t left_sensor_value_;
  float right_sensor_mean_value_;
  float left_sensor_mean_value_;
  int8_t direction;
  int8_t previous_direction;

  MotorOuput motor_output_;

public:
  LineFollower();
  virtual ~LineFollower();
  void follow_line();
  void maintain_direction();
  void decide_direction();
  void go_straight();
  void turn_left();
  void turn_right();

  inline MotorOuput get_motor_output()
  {
    return motor_output_;
  }
};

#endif