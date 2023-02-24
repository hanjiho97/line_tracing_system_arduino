#include "decision_maker.h"

DecisionMaker::DecisionMaker(const STATE_TYPE initial_state)
    : current_state_(initial_state), states_(STATE_TYPE::NUM_STATES)
{

  states_[STATE_TYPE::INIT] = new InitState;
  states_[STATE_TYPE::STOP] = new StopState;
  states_[STATE_TYPE::FORWARD] = new ForwardState;

  // define edges of INIT_STATE 
  states_[STATE_TYPE::INIT]->insert_next_state(states_[STATE_TYPE::STOP]);

  // define edges of STOP_STATE 
  states_[STATE_TYPE::STOP]->insert_next_state(states_[STATE_TYPE::FORWARD]);
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
  // std::cout << "call get_next_state..." << std::endl;
  STATE_TYPE new_state = states_[static_cast<uint32_t>(current_state_)]->get_next_state(); 
  std::cout << "new_state: " << new_state << std::endl;
  if (new_state != current_state_)
  {
    current_state_ = new_state;
    states_[static_cast<uint32_t>(current_state_)]->reset_timer();
  }

  // std::cout << "call current state run... " << static_cast<uint32_t>(current_state_) << std::endl;
  if (states_[static_cast<uint32_t>(current_state_)]->run())
    std::cout << "run success: " << static_cast<uint32_t>(current_state_) << std::endl;
  else
    std::cout << "run failure: " << static_cast<uint32_t>(current_state_) << std::endl;
}
