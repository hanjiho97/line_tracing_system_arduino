#ifndef _FORWARD_STATE_H_
#define _FORWARD_STATE_H_

#include "behavior_state_machine.h"
#include <ArduinoSTL.h>
#include <vector>

class ForwardState : public BehaviorStateMachine
{
public:
  ForwardState()
      : BehaviorStateMachine(STATE_TYPE::FORWARD)
  {
  }
  virtual ~ForwardState() {}
  virtual BehaviorStateMachine *get_next_state();
  virtual bool run();
};

#endif