#include "stop_state.h"

STATE_TYPE StopState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::FORWARD);
}

bool StopState::run()
{
  return true;
}
