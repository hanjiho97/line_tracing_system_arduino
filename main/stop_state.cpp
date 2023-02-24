#include "stop_state.h"

STATE_TYPE StopState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool StopState::run(const DecisionMaker& decision_maker, MotorOuput& motor_output)
{
  return true;
}
