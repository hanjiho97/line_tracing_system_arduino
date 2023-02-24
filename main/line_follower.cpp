#include "line_follower.h"

enum DIRECTION
{
  STRAGHIT,
  RIGHT,
  LEFT,
  NONE
};

LineFollower::LineFollower()
{
  right_filter_ptr_ = new MovingAverageFilter(NUMBER_OF_SAMPLES);
  left_filter_ptr_ = new MovingAverageFilter(NUMBER_OF_SAMPLES);
  direction = 0;
}

LineFollower::~LineFollower()
{

  delete right_filter_ptr_;
  delete left_filter_ptr_;
}

void LineFollower::follow_line()
{
  decide_direction();
  switch (direction)
  {
  case DIRECTION::STRAGHIT:
    go_straight();
    break;
  case DIRECTION::RIGHT:
    turn_right();
    break;
  case DIRECTION::LEFT:
    turn_left();
    break;
  case DIRECTION::NONE:
    maintain_direction();
    break;
  default:
    break;
  }
}

void LineFollower::maintain_direction()
{
  switch (previous_direction)
  {
  case DIRECTION::STRAGHIT:
    go_straight();
    break;
  case DIRECTION::RIGHT:
    turn_right();
    break;
  case DIRECTION::LEFT:
    turn_left();
    break;
  default:
    break;
  }
}

void LineFollower::decide_direction()
{
  right_sensor_value_ = analogRead(RIGHT_LINE_SENSOR_PIN);
  left_sensor_value_ = analogRead(LEFT_LINE_SENSOR_PIN);
  right_filter_ptr_->add_sample(right_sensor_value_);
  left_filter_ptr_->add_sample(left_sensor_value_);
  right_sensor_mean_value_ = right_filter_ptr_->get_weighted_moving_average();
  left_sensor_mean_value_ = left_filter_ptr_->get_weighted_moving_average();
  if ((right_sensor_mean_value_ > LINE_SENSOR_THRESHOLD) &&
      (left_sensor_mean_value_ > LINE_SENSOR_THRESHOLD))
  {
    direction = DIRECTION::STRAGHIT;
  }
  else if ((right_sensor_mean_value_ > LINE_SENSOR_THRESHOLD) &&
           (left_sensor_mean_value_ <= LINE_SENSOR_THRESHOLD))
  {
    direction = DIRECTION::RIGHT;
  }
  else if ((right_sensor_mean_value_ <= LINE_SENSOR_THRESHOLD) &&
           (left_sensor_mean_value_ > LINE_SENSOR_THRESHOLD))
  {
    direction = DIRECTION::LEFT;
  }
  else
  {
    direction = DIRECTION::NONE;
  }
}

void LineFollower::go_straight()
{
  motor_output_.right_motor_speed_ = HIGH_MOTOR_SPEED;
  motor_output_.left_motor_speed_ = HIGH_MOTOR_SPEED;
  motor_output_.right_motor_mode_ = FORWARD;
  motor_output_.left_motor_mode_ = FORWARD;
  previous_direction = DIRECTION::STRAGHIT;
}

void LineFollower::turn_right()
{
  motor_output_.right_motor_speed_ = LOW_MOTOR_SPEED;
  motor_output_.left_motor_speed_ = HIGH_MOTOR_SPEED;
  motor_output_.right_motor_mode_ = BACKWARD;
  motor_output_.left_motor_mode_ = FORWARD;
  previous_direction = DIRECTION::RIGHT;
}

void LineFollower::turn_left()
{
  motor_output_.right_motor_speed_ = HIGH_MOTOR_SPEED;
  motor_output_.left_motor_speed_ = LOW_MOTOR_SPEED;
  motor_output_.right_motor_mode_ = FORWARD;
  motor_output_.left_motor_mode_ = BACKWARD;
  previous_direction = DIRECTION::LEFT;
}