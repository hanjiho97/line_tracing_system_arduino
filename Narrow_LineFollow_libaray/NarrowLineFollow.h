#ifndef NARROWLINEFOLLOWER_H_
#define NARROWLINEFOLLOWER_H_

#include <Arduino.h>
#include <AFMotor.h>

#define RIGHT_LINE_SENSOR_PIN A5
#define LEFT_LINE_SENSOR_PIN A0
#define RIGHT_MOTOR_NUMBER 1
#define LEFT_MOTOR_NUMBER 4

class NarrowLineFollower
{
private:
  uint16_t right_sensor_value_;
  uint16_t left_sensor_value_;
  bool straight_flag;
  bool right_flag;
  bool left_flag;
public:
  NarrowLineFollower();
  virtual ~NarrowLineFollower() {};
  void follow_line();
};

#endif
