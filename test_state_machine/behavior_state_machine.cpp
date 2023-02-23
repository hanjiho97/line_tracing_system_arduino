#include "behavior_state_machine.h"

void BehaviorStateMachine::init()
{
  time_ = millis();
}

void BehaviorStateMachine::reset_timer()
{
  time_ = millis();
}

void BehaviorStateMachine::insert_next_state(BehaviorStateMachine *next_state)
{
  if (next_state)
    p_next_states_.push_back(next_state);
}

BehaviorStateMachine *BehaviorStateMachine::find_best_state(int n_min_count)
{
  for (uint32_t i = 0; i < behavior_log_.size(); i++)
  {
    if (behavior_log_.at(i).second >= n_min_count)
    {
      // std::cout << "Found Next Beh: " << behavior_log_.at(i).first->m_Behavior << ", Count: " << behavior_log_.at(i).second  << ", LogSize: " << behavior_log_.size() << std::endl;
      return behavior_log_.at(i).first;
    }
  }

  return nullptr;
}

BehaviorStateMachine *BehaviorStateMachine::find_behavior_state(const STATE_TYPE &behavior)
{
  for (uint32_t i = 0; i < p_next_states_.size(); i++)
  {
    BehaviorStateMachine *p_state = p_next_states_.at(i);
    if (p_state && behavior == p_state->behavior_state_)
    {
      // UpdateLogCount(pState);
      // pState = FindBestState(decisionMakingCount);

      if (p_state == 0)
        return this;

      behavior_log_.clear();
      p_state->reset_timer();
      return p_state;
    }
  }

  return nullptr;
}
