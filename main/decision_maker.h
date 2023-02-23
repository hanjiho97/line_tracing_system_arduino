#ifndef _DECISION_MAKER_H_
#define _DECISION_MAKER_H_

#include "behavior_state_machine.h"
#include "init_state.h"
#include "stop_state.h"
#include "forward_state.h"

#include <ArduinoSTL.h>
#include <vector>

class DecisionMaker
{
public:
  DecisionMaker(const STATE_TYPE initial_state);
  ~DecisionMaker() {};

protected:
  // STATE_TYPE current_state_;
  STATE_TYPE current_state_;
  std::vector<BehaviorStateMachine *> states_;
};

#endif
