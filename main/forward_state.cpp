#include "forward_state.h"

STATE_TYPE ForwardState::get_next_state()
{
  return (STATE_TYPE::LINE_FOLLOW);
}

bool ForwardState::run(DecisionMaker& decision_maker, MotorOuput& motor_output)
{
  // std::cout << _PF_ << "run" << std::endl;
  // line_follower.follow_line();
  // motor_output = line_follower.get_motor_output();
  LineFollower& line_follower = decision_maker.get_line_follower();
  line_follower.follow_line();
  motor_output = line_follower.get_motor_output();

  return true;
}
