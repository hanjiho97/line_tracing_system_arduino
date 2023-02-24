#include "narrow_line_follower.h"

AF_DCMotor g_right_motor(RIGHT_MOTOR_NUMBER);
AF_DCMotor g_left_motor(LEFT_MOTOR_NUMBER);

NarrowLineFollower::NarrowLineFollower()
{
  straight_flag = false;
  right_flag = false;
  left_flag = false;
  g_right_motor.setSpeed(INIT_SPEED);
  g_left_motor.setSpeed(INIT_SPEED);
  g_right_motor.run(RELEASE);
  g_left_motor.run(RELEASE);
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT);
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT);
}
bool NarrowLineFollower::check_line()
{

  right_sensor_value_ = analogRead(RIGHT_LINE_SENSOR_PIN);
  left_sensor_value_ = analogRead(LEFT_LINE_SENSOR_PIN);
  return (left_sensor_value_ > 500) && (right_sensor_value_ > 500);
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
    g_right_motor.setSpeed(200);
    g_left_motor.setSpeed(200);
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
    g_left_motor.setSpeed(150);
    g_right_motor.run(FORWARD);
    g_left_motor.run(BACKWARD);
    straight_flag = false;
    right_flag = true;
    left_flag = false;
    while (true)
    {
      // delay(10);
      if (check_line())
        break;
    }
  }
  else if ((!right_state) && (left_state))
  {
    g_right_motor.setSpeed(150);
    g_left_motor.setSpeed(150);
    g_right_motor.run(BACKWARD);
    g_left_motor.run(FORWARD);
    straight_flag = false;
    right_flag = false;
    left_flag = true;
    while (true)
    {
      // delay(10);
      if (check_line())
        break;
    }
  }
  else if ((!right_state) && (!left_state))
  {
    int seed = random(3);
    if (seed == 0)
    {
      g_right_motor.setSpeed(200);
      g_left_motor.setSpeed(200);
      g_right_motor.run(BACKWARD);
      g_left_motor.run(BACKWARD);
    }
    else if (seed == 1)
    {
      g_right_motor.setSpeed(150);
      g_left_motor.setSpeed(150);
      g_right_motor.run(BACKWARD);
      g_left_motor.run(FORWARD);
    }
    else if (seed == 2)
    {
      g_right_motor.setSpeed(150);
      g_left_motor.setSpeed(150);
      g_right_motor.run(FORWARD);
      g_left_motor.run(BACKWARD);
    }
    delay(50);
  }
}
