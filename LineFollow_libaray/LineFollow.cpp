#include "LineFollow.h"

AF_DCMotor g_right_motor(RIGHT_MOTOR_NUMBER);
AF_DCMotor g_left_motor(LEFT_MOTOR_NUMBER);

LineFollower::LineFollower()
{
  g_right_motor.setSpeed(150);
  g_left_motor.setSpeed(150);
  g_right_motor.run(RELEASE);
  g_left_motor.run(RELEASE);
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT);
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT);
  direction = 0;
}

void LineFollower::follow_line()
{
  if (direction != -1)
  {
    switch(direction)
    {
      case 0:
        go_straight();
        break;
      case 1:
        turn_right();
        break;
      case 2:
        turn_left();
        break;
      default:
        break;
    }
  }
  else
  {
    switch(previous_direction)
    {
      case 0:
        go_straight();
        break;
      case 1:
        turn_right();
        break;
      case 2:
        turn_left();
        break;
      default:
        break;
    }
  }
}

void LineFollower::decide_direction()
{
  right_sensor_value_ = analogRead(RIGHT_LINE_SENSOR_PIN);
  left_sensor_value_ = analogRead(LEFT_LINE_SENSOR_PIN);
  if ((right_sensor_value_ <= LINE_SENSOR_THRESHOLD) && 
  (left_sensor_value_ <= LINE_SENSOR_THRESHOLD))
  {
    direction = 0;
  }
  else if ((right_sensor_value_ > LINE_SENSOR_THRESHOLD) && 
  (left_sensor_value_ <= LINE_SENSOR_THRESHOLD))
  {
    direction = 1;
  }
  else if ((right_sensor_value_ <= LINE_SENSOR_THRESHOLD) && 
  (left_sensor_value_ > LINE_SENSOR_THRESHOLD))
  {
    direction = 2;
  }
  else
  {
    direction = -1;
  }
}

void LineFollower::go_straight()
{
  g_right_motor.setSpeed(150);
  g_left_motor.setSpeed(150);
  g_right_motor.run(FORWARD);
  g_left_motor.run(FORWARD);
  previous_direction = 0;
}

void LineFollower::turn_right()
{
  g_right_motor.setSpeed(100);
  g_left_motor.setSpeed(150);
  g_right_motor.run(BACKWARD);
  g_left_motor.run(FORWARD);
  previous_direction = 1;
}

void LineFollower::turn_left()
{
  g_right_motor.setSpeed(150);
  g_left_motor.setSpeed(100);
  g_right_motor.run(FORWARD);
  g_left_motor.run(BACKWARD);
  previous_direction = 2;
}
