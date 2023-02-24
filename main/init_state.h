#ifndef _INIT_STATE_H_
#define _INIT_STATE_H_

#include "behavior_state_machine.h"
#include "common_params.h"

#include <ArduinoSTL.h>
#include <vector>

class InitState : public BehaviorStateMachine
{
public:
  InitState()
      : BehaviorStateMachine(STATE_TYPE::INIT)
  {
  }
  virtual ~InitState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker& decision_maker, MotorOuput &motor_output);
};

#endif
