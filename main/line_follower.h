#ifndef _LINE_FOLLOWER_H
#define _LINE_FOLLOWER_H

#include <Arduino.h>
#include <AFMotor.h>

#include "common_params.h"
#include "moving_average_filter.h"


class LineFollower
{
public:
  LineFollower();
  virtual ~LineFollower();
  void follow_line(uint16_t right_sensor_value, uint16_t left_sensor_value);
  void maintain_direction();
  void decide_direction(uint16_t right_sensor_value, uint16_t left_sensor_value);
  void go_straight();
  void turn_left();
  void turn_right();
  void go_back();

  inline MotorOutput get_motor_output()
  {
    return motor_output_;
  }

private:
  MovingAverageFilter *right_filter_ptr_;
  MovingAverageFilter *left_filter_ptr_;

  float right_sensor_mean_value_;
  float left_sensor_mean_value_;
  int8_t direction;
  int8_t previous_direction;

  MotorOutput motor_output_;
};

#endif
