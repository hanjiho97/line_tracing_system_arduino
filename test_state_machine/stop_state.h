#ifndef _STOP_STATE_H_
#define _STOP_STATE_H_

#include "behavior_state_machine.h"
#include <ArduinoSTL.h>
#include <vector>

class StopState : public BehaviorStateMachine
{
public:
  StopState()
      : BehaviorStateMachine(STATE_TYPE::STOP)
  {
  }
  virtual ~StopState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run();
};

#endif