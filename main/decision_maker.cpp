#include "decision_maker.h"

DecisionMaker::DecisionMaker(const STATE_TYPE initial_state)
    : current_state_(initial_state), states_(STATE_TYPE::NUM_STATES)
{

  states_[STATE_TYPE::INIT] = new InitState;
  states_[STATE_TYPE::STOP] = new StopState;
  states_[STATE_TYPE::FORWARD] = new ForwardState;

  std::cout << "DecisionMaker Constructor Done" << std::endl;
}

DecisionMaker::~DecisionMaker()
{
  for (uint32_t i = 0; i < static_cast<uint32_t>(STATE_TYPE::NUM_STATES); i++)
  {
    delete states_[i];
  }
}

void DecisionMaker::run()
{
  std::cout << "call current state run... " << static_cast<int>(current_state_) << std::endl;
  states_[current_state_]->run();
  
  std::cout << "call get_next_state..." << std::endl;
  STATE_TYPE new_state = states_[current_state_]->get_next_state(); 
  if (new_state != current_state_)
  {
    states_[new_state]->reset_timer();
    current_state_ = new_state;
  }
}