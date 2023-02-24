#include "init_state.h"

STATE_TYPE InitState::get_next_state()
{
  // std::cout << "runtime: " << static_cast<uint32_t>(runtime_) << std::endl;
  // std::cout << "millis(): " << static_cast<uint32_t>(millis()) << std::endl;
  uint32_t time_diff = millis() - runtime_;
  std::cout << "time_diff: " << time_diff << std::endl;
  if (time_diff < START_WAIT_TIME_MS)
    return find_behavior_state(behavior_state_);
  else
    return find_behavior_state(STATE_TYPE::STOP);
}

bool InitState::run(const DecisionMaker& decision_maker, MotorOuput& motor_output)
{
  // std::cout << _PF_ << " Ready..." << std::endl;
  // std::cout << " Ready..." << std::endl;
  // Serial.println("Ready...");
  std::cout << _PF_ << "run" << std::endl;
  return true;
}
