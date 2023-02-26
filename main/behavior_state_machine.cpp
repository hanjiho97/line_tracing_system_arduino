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
  for (uint32_t i = 0U; i < p_next_states_.size(); ++i)
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

bool BehaviorStateMachine::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  // std::cout << _PF_ << "run" << std::endl;
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
  uint32_t time_differance = millis() - runtime_;
  std::cout << "time_diff: " << time_diff << std::endl;
  if (time_differance < START_WAIT_TIME_MS)
    return find_behavior_state(behavior_state_);
  else
    return find_behavior_state(STATE_TYPE::STOP);
}

bool InitState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
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
  time_differance = millis() - runtime_;
  // collision
  if (sensor_data_.collision_value_ == 0)
  {
    return find_behavior_state(STATE_TYPE::COLLISION);
  }
  // normal termination
  else if (millis() >= DONE_TIME_MS)
  {
    return find_behavior_state(STATE_TYPE::NORMAL_TERMINATION);
  }
  else
  {
    if((sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD) ||
    (sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD))
    {
      if (sensor_data_.ir_value == 0)
      {
        // obstacle avoidance
        if (time_differance >= STOP_WAIT_TIME_MS)
        {
          return find_behavior_state(STATE_TYPE::OBSTACLE_AVOIDANCE);
        }
        // stop
        else
        {
          return find_behavior_state(behavior_state_);
        }
      //line follow
      }
      else
      {
        return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
      }
    // abnormal termination
    }
    else
    {
      return find_behavior_state(STATE_TYPE::ABNORMAL_TERMINATION);
    }
  }
}

bool StopState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  sensor_data_ = decision_maker.get_sensor_data(sensor_data_);
  motor_output.right_motor_speed_ = 0;
  motor_output.left_motor_speed_ = 0;
  motor_output.right_motor_mode_ = RELEASE;
  motor_output.left_motor_mode_ = RELEASE;
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/********************************* LineFollowState ***************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE LineFollowState::get_next_state()
{
  if (sensor_data_.collision_value_ == 0)
  {
    return find_behavior_state(STATE_TYPE::COLLISION);
  }
  else if (sensor_data_.ir_value > 0)
  {
    return find_behavior_state(STATE_TYPE::EMERGENCY_STOP);
  }
  else if ((sensor_data_.ir_value == 0) && (check_lane_existance() == true))
  {
    return find_behavior_state(STATE_TYPE::STOP);
  }
  else if (((millis() - none_lane_start_time) > NONE_LANE_TIME_MS) &&
  (check_lane_existance() == false))
  {
    return find_behavior_state(STATE_TYPE::STOP);
  }
  else
  {
    return find_behavior_state(behavior_state_);
  }
}

bool LineFollowState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  std::cout << _PF_ << "******************************************" << std::endl;
  std::cout << _PF_ << "************* LineFollowState ************" << std::endl;
  std::cout << _PF_ << "******************************************" << std::endl;

  sensor_data_ = decision_maker.get_sensor_data(sensor_data_);
  line_follower_.follow_line(sensor_data_.line_tracing_right_,
                            sensor_data_.line_tracing_left_);
  motor_output = line_follower_.get_motor_output();
  return true;
}

bool LineFollowState::check_lane_existance()
{
  if((sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD) ||
  (sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD))
  {
    return true;
  }
  else
  {
    none_lane_start_time = millis();
    return false;
  }
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

bool ObstacleAvoidanceState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/********************************* CollisionState ****************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE CollisionState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool CollisionState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/******************************** SystemFaultState ***************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE SystemFaultState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::ABNORMAL_TERMINATION);
}

bool SystemFaultState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
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

bool EmergencyStopState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/***************************** NormalTerminationState ************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE NormalTerminationState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool NormalTerminationState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/**************************** AbnormalTerminationState ***********************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE AbnormalTerminationState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool AbnormalTerminationState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/****************************** SystemRecoveryState **************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE SystemRecoveryState::get_next_state()
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool SystemRecoveryState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  return true;
}
