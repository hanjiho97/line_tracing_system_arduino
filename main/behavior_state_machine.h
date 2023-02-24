#ifndef _BEHAVIOR_STATE_MACHINE_H_
#define _BEHAVIOR_STATE_MACHINE_H_

#include "common_params.h"

#include <ArduinoSTL.h>
#include <vector>

class BehaviorStateMachine
{
public:
  BehaviorStateMachine(STATE_TYPE behavior_state)
      : behavior_state_(behavior_state)
  {
    p_next_states_.push_back(this);
    init();
  }
  virtual ~BehaviorStateMachine(){};

  virtual STATE_TYPE get_next_state() = 0;
  virtual void init();
  virtual void reset_timer();
  virtual void insert_next_state(BehaviorStateMachine* next_state);
  virtual bool run();

  STATE_TYPE find_behavior_state(const STATE_TYPE& behavior);

protected:
  STATE_TYPE behavior_state_;
  std::vector<BehaviorStateMachine *> p_next_states_;
  uint32_t runtime_;
};

#endif