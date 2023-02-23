#include "LineFollow.h"

AF_DCMotor g_right_motor(RIGHT_MOTOR_NUMBER);
AF_DCMotor g_left_motor(LEFT_MOTOR_NUMBER);

LineFollower::LineFollower()
{
  g_right_motor.setSpeed(200);
  g_left_motor.setSpeed(200);
  g_right_motor.run(RELEASE);
  g_left_motor.run(RELEASE);
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT);
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT);
}

void LineFollower::follow_line()
{
  right_sensor_value_ = digitalRead(RIGHT_LINE_SENSOR_PIN);
  left_sensor_value_ = digitalRead(LEFT_LINE_SENSOR_PIN);
  if ((right_sensor_value_ == 0) && (left_sensor_value_ == 0))
  {
    g_right_motor.run(FORWARD);
    g_left_motor.run(FORWARD);
  }
  else if ((right_sensor_value_ == 1) && (left_sensor_value_ == 0))
  {
    g_right_motor.run(FORWARD);
    g_left_motor.run(RELEASE);
  }
  else if ((right_sensor_value_ == 0) && (left_sensor_value_ == 1))
  {
    g_right_motor.run(RELEASE);
    g_left_motor.run(FORWARD);
  }
  else if ((right_sensor_value_ == 1) && (left_sensor_value_ == 1))
  {
    g_right_motor.run(RELEASE);
    g_left_motor.run(RELEASE);
  }
}
