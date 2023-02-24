#include "forward_state.h"

STATE_TYPE ForwardState::get_next_state()
{
  return (STATE_TYPE::FORWARD);
}

bool ForwardState::run()
{
  std::cout << _PF_ << "run" << std::endl;
  return true;
}
