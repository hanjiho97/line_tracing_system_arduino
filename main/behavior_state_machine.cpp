#include "behavior_state_machine.h"

void BehaviorStateMachine::init()
{
  std::cout << "init()" << std::endl;
  runtime_ = millis();
}

void BehaviorStateMachine::reset_timer()
{
  std::cout << "reset_timer()" << std::endl;
  runtime_ = millis();
}

void BehaviorStateMachine::insert_next_state(BehaviorStateMachine* next_state)
{
  if (next_state)
    p_next_states_.push_back(next_state);
}

STATE_TYPE BehaviorStateMachine::find_behavior_state(const STATE_TYPE& behavior)
{
  for (uint32_t i = 0; i < p_next_states_.size(); i++)
  {
    BehaviorStateMachine *p_state = p_next_states_.at(i);
    if (p_state && behavior == p_state->behavior_state_)
    {
      // UpdateLogCount(pState);
      // pState = FindBestState(decisionMakingCount);

      if (p_state == nullptr)
        return (STATE_TYPE::INVALID_STATE);

      // behavior_log_.clear();
      // p_state->reset_timer();
      // return p_state;
      std::cout << "Found: " << p_state->behavior_state_ << std::endl; 
      return p_state->behavior_state_;
    }
  }

  return STATE_TYPE::INVALID_STATE;
}

bool BehaviorStateMachine::run(DecisionMaker& decision_maker, MotorOuput& motor_output)
{
  // std::cout << _PF_ << "run" << std::endl;
  return true;
}