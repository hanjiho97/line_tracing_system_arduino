#ifndef LINEFOLLOWER_H_
#define LINEFOLLOWER_H_

#include <Arduino.h>
#include <AFMotor.h>
#include "MovingAverageFilter.h"

#define RIGHT_LINE_SENSOR_PIN A5
#define LEFT_LINE_SENSOR_PIN A0
#define RIGHT_MOTOR_NUMBER 1
#define LEFT_MOTOR_NUMBER 4

#define LINE_SENSOR_THRESHOLD 500
#define HIGH_MOTOR_SPEED 150
#define LOW_MOTOR_SPEED 100

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
public:
  LineFollower();
  ~LineFollower() {};
  void follow_line();
  void decide_direction();
  void go_straight();
  void turn_left();
  void turn_right();
};

#endif
