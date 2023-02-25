#include "behavior_state_machine.h"

/*****************************************************************************************/
/******************************* BehaviorStateMachine*************************************/
/*****************************************************************************************/
void BehaviorStateMachine::init()
{
  std::cout << "init()" << std::endl;
  runtime_ = millis();
}

void BehaviorStateMachine::reset_timer()
{
  std::cout << "reset_timer()" << std::endl;
  runtime_ = millis();
}

void BehaviorStateMachine::insert_next_state(BehaviorStateMachine *next_state)
{
  if (next_state)
    p_next_states_.push_back(next_state);
}

STATE_TYPE BehaviorStateMachine::find_behavior_state(const STATE_TYPE &behavior)
{
  for (uint32_t i = 0; i < p_next_states_.size(); i++)
  {
    BehaviorStateMachine *p_state = p_next_states_.at(i);
    if (p_state && behavior == p_state->behavior_state_)
    {
      // UpdateLogCount(pState);
      // pState = FindBestState(decisionMakingCount);

      if (p_state == nullptr)
        return (STATE_TYPE::INVALID_STATE);

      // behavior_log_.clear();
      // p_state->reset_timer();
      // return p_state;
      std::cout << "Found: " << p_state->behavior_state_ << std::endl;
      return p_state->behavior_state_;
    }
  }

  return (STATE_TYPE::INVALID_STATE);
}

bool BehaviorStateMachine::run(DecisionMaker &decision_maker, MotorOuput &motor_output)
{
  // std::cout << _PF_ << "run" << std::endl;
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/********************************* LineFollowState ***************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE LineFollowState::get_next_state()
{
  return (STATE_TYPE::LINE_FOLLOW);
}

bool LineFollowState::run(DecisionMaker &decision_maker, MotorOuput &motor_output)
{
  std::cout << _PF_ << "******************************************" << std::endl;
  std::cout << _PF_ << "************* LineFollowState ************" << std::endl;
  std::cout << _PF_ << "******************************************" << std::endl;

  decision_maker.get_sensor_data(sensor_data_);
  LineFollower &line_follower = decision_maker.get_line_follower();
  line_follower.follow_line(sensor_data_.line_tracing_right_,
                            sensor_data_.line_tracing_left_);
  motor_output = line_follower.get_motor_output();
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/*********************************** InitState *******************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE InitState::get_next_state()
{
  // std::cout << "runtime: " << static_cast<uint32_t>(runtime_) << std::endl;
  // std::cout << "millis(): " << static_cast<uint32_t>(millis()) << std::endl;
  uint32_t time_diff = millis() - runtime_;
  std::cout << "time_diff: " << time_diff << std::endl;
  if (time_diff < START_WAIT_TIME_MS)
    return find_behavior_state(behavior_state_);
  else
    return find_behavior_state(STATE_TYPE::STOP);
}

bool InitState::run(DecisionMaker &decision_maker, MotorOuput &motor_output)
{
  // std::cout << _PF_ << " Ready..." << std::endl;
  // std::cout << " Ready..." << std::endl;
  // Serial.println("Ready...");
  std::cout << _PF_ << "******************************************" << std::endl;
  std::cout << _PF_ << "*****************INIT_STATE***************" << std::endl;
  std::cout << _PF_ << "******************************************" << std::endl;
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/*********************************** StopState *******************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE StopState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool StopState::run(DecisionMaker &decision_maker, MotorOuput &motor_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/****************************** EmergencyStopState ***************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE EmergencyStopState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool EmergencyStopState::run(DecisionMaker &decision_maker, MotorOuput &motor_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/********************************* ParkingState ******************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE ParkingState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool ParkingState::run(DecisionMaker &decision_maker, MotorOuput &motor_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/**************************** ObstacleAvoidanceState *************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE ObstacleAvoidanceState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool ObstacleAvoidanceState::run(DecisionMaker &decision_maker, MotorOuput &motor_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/****************************** CollisionStopState ***************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE CollisionStopState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool CollisionStopState::run(DecisionMaker &decision_maker, MotorOuput &motor_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/****************************** TheftEmergencyState **************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE TheftEmergencyState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool TheftEmergencyState::run(DecisionMaker &decision_maker, MotorOuput &motor_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/************************************ DoneState ******************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE DoneState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool DoneState::run(DecisionMaker &decision_maker, MotorOuput &motor_output)
{
  return true;
}