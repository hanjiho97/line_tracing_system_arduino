#ifndef _FORWARD_STATE_H_
#define _FORWARD_STATE_H_

#include "behavior_state_machine.h"
#include "decision_maker.h"
#include <ArduinoSTL.h>
#include <vector>

class ForwardState : public BehaviorStateMachine
{
public:
  ForwardState()
      : BehaviorStateMachine(STATE_TYPE::LINE_FOLLOW)
  {
  }
  virtual ~ForwardState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(const DecisionMaker& decision_maker, MotorOuput& motor_output);
};

#endif