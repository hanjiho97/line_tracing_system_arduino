#include "NarrowLineFollow.h"

AF_DCMotor g_right_motor(RIGHT_MOTOR_NUMBER);
AF_DCMotor g_left_motor(LEFT_MOTOR_NUMBER);

NarrowLineFollower::NarrowLineFollower()
{
  straight_flag = false;
  right_flag = false;
  left_flag = false;
  g_right_motor.setSpeed(150);
  g_left_motor.setSpeed(150);
  g_right_motor.run(RELEASE);
  g_left_motor.run(RELEASE);
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT);
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT);
}

void NarrowLineFollower::follow_line()
{
  right_sensor_value_ = analogRead(RIGHT_LINE_SENSOR_PIN);
  left_sensor_value_ = analogRead(LEFT_LINE_SENSOR_PIN);
  bool left_state = left_sensor_value_ > 500;
  bool right_state = right_sensor_value_ > 500;
  if ((right_state) && (left_state))
  {
    // forward
    g_right_motor.setSpeed(150);
    g_left_motor.setSpeed(150);
    g_right_motor.run(FORWARD);
    g_left_motor.run(FORWARD);
    straight_flag = true;
    right_flag = false;
    left_flag = false;
  }
  else if ((right_state) && (!left_state))
  {
    // right
    g_right_motor.setSpeed(150);
    g_left_motor.setSpeed(100);
    g_right_motor.run(FORWARD);
    g_left_motor.run(BACKWARD);
    straight_flag = false;
    right_flag = true;
    left_flag = false;
  }
  else if ((!right_state) && (left_state))
  {
    g_right_motor.setSpeed(100);
    g_left_motor.setSpeed(150);
    g_right_motor.run(BACKWARD);
    g_left_motor.run(FORWARD);
    straight_flag = false;
    right_flag = false;
    left_flag = true;
  }
  else if ((!right_state) && (!left_state))
  {
    if (straight_flag)
    {
      g_right_motor.setSpeed(150);
      g_left_motor.setSpeed(150);
      g_right_motor.run(FORWARD);
      g_left_motor.run(FORWARD);
    }
    else if (right_flag)
    {
      g_right_motor.setSpeed(150);
      g_left_motor.setSpeed(150);
      g_right_motor.run(FORWARD);
      g_left_motor.run(BACKWARD);
    }
    else if (left_flag)
    {
      g_right_motor.setSpeed(150);
      g_left_motor.setSpeed(150);
      g_right_motor.run(BACKWARD);
      g_left_motor.run(FORWARD);
    }
  }
}