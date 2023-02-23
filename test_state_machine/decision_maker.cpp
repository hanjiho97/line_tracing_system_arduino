#include "decision_maker.h"

DecisionMaker::DecisionMaker(const STATE_TYPE initial_state)
    : current_state_(initial_state), states_(STATE_TYPE::NUM_STATES)
{
  std::cout << "DecisionMaker Constructor" << std::endl;

  states_[STATE_TYPE::INIT] = new InitState;
  states_[STATE_TYPE::STOP] = new StopState;
  states_[STATE_TYPE::FORWARD] = new ForwardState;

  std::cout << "Constructor Done" << std::endl;

}
