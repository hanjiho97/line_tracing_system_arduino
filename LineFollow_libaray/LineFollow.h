#ifndef LINEFOLLOWER_H_
#define LINEFOLLOWER_H_

#include <Arduino.h>
#include <AFMotor.h>

#define RIGHT_LINE_SENSOR_PIN A5
#define LEFT_LINE_SENSOR_PIN A0
#define RIGHT_MOTOR_NUMBER 1
#define LEFT_MOTOR_NUMBER 2

class LineFollower
{
private:
  uint8_t right_sensor_value_;
  uint8_t left_sensor_value_;
public:
  LineFollower();
  virtual ~LineFollower() {};
  void follow_line();
};

#endif
