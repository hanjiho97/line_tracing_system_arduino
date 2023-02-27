#include "behavior_state_machine.h"
#include "decision_maker.h"

/*****************************************************************************************/
/******************************* BehaviorStateMachine*************************************/
/*****************************************************************************************/
void BehaviorStateMachine::init()
{
  // std::cout << "init()" << std::endl;
  runtime_ = millis();
}

void BehaviorStateMachine::reset_timer()
{
  // std::cout << "reset_timer()" << std::endl;
  runtime_ = millis();
}

void BehaviorStateMachine::insert_next_state(BehaviorStateMachine* next_state)
{
  if (next_state)
    p_next_states_.push_back(next_state);
}

STATE_TYPE BehaviorStateMachine::find_behavior_state(const STATE_TYPE& behavior)
{
  for (uint32_t i = 0U; i < p_next_states_.size(); ++i)
  {
    BehaviorStateMachine* p_state = p_next_states_.at(i);
    if (p_state && behavior == p_state->behavior_state_)
    {
      // UpdateLogCount(pState);
      // pState = FindBestState(decisionMakingCount);

      if (p_state == nullptr)
      {
        std::cout << "p_state == nullptr" << std::endl;
        return (STATE_TYPE::INVALID_STATE);
      }
      // behavior_log_.clear();
      // p_state->reset_timer();
      // return p_state;
      // std::cout << "Found: " << p_state->behavior_state_ << std::endl;
      return p_state->behavior_state_;
    }
  }

  return (STATE_TYPE::INVALID_STATE);
}

bool BehaviorStateMachine::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
  // std::cout << _PF_ << "run" << std::endl;
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/*********************************** InitState *******************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE InitState::get_next_state(DecisionMaker& decision_maker)
{
  uint32_t time_difference = millis() - runtime_;

  if (time_difference < START_WAIT_TIME_MS)
    return find_behavior_state(behavior_state_);
  else
    return find_behavior_state(STATE_TYPE::STOP);
}

bool InitState::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
  // std::cout << "******************************************" << std::endl;
  // std::cout << "*****************INIT_STATE***************" << std::endl;
  // std::cout << "******************************************" << std::endl;
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/*********************************** StopState *******************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE StopState::get_next_state(DecisionMaker& decision_maker)
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool StopState::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
  motor_output.right_motor_mode_ = RELEASE;
  motor_output.left_motor_mode_ = RELEASE;
  motor_output.right_motor_speed_ = 0;
  motor_output.left_motor_speed_ = 0;
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/********************************* LineFollowState ***************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE LineFollowState::get_next_state(DecisionMaker& decision_maker)
{
  return find_behavior_state(behavior_state_);
}

bool LineFollowState::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
  sensor_data_ = decision_maker.get_sensor_data();
  line_follower_.follow_line(sensor_data_.line_tracing_right_,
                             sensor_data_.line_tracing_left_);
  motor_output = line_follower_.get_motor_output();
  return true;
}

bool LineFollowState::exist_line()
{
  if ((sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD) ||
      (sensor_data_.line_tracing_left_ > LINE_SENSOR_THRESHOLD))
    return true;
  else
    return false;
}
