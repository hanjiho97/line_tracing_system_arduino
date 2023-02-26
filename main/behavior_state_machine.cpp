#include "behavior_state_machine.h"
#include "decision_maker.h"

/*****************************************************************************************/
/******************************* BehaviorStateMachine*************************************/
/*****************************************************************************************/
void BehaviorStateMachine::init()
{
  std::cout << "init()" << std::endl;
  runtime_ = sensor_data_.read_time_;
}

void BehaviorStateMachine::reset_timer()
{
  std::cout << "reset_timer()" << std::endl;
  runtime_ = sensor_data_.read_time_;
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
      // std::cout << "Found: " << p_state->behavior_state_ << std::endl;
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

bool BehaviorStateMachine::display_state(DecisionMaker &decision_maker, DisplayOutput &display_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/*********************************** InitState *******************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE InitState::get_next_state(DecisionMaker &decision_maker)
{
  uint32_t time_difference = millis() - runtime_;

  if (time_difference < START_WAIT_TIME_MS)
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
STATE_TYPE StopState::get_next_state(DecisionMaker &decision_maker)
{
  uint32_t time_difference = millis() - runtime_;

  // collision
  if (sensor_data_.collision_value_ < COLLISION_DETECTED_THRESHOLD)
  {
    return find_behavior_state(STATE_TYPE::COLLISION);
  }
  else
  {
    if ((sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD) ||
        (sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD))
    {
      if (sensor_data_.ir_value_ == IR_DETECTED)
      {
        // obstacle avoidance
        if (time_difference >= STOP_WAIT_TIME_MS)
        {
          return find_behavior_state(STATE_TYPE::OBSTACLE_AVOIDANCE);
        }
        // stop
        else
        {
          return find_behavior_state(behavior_state_);
        }
        // line follow
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
  sensor_data_ = decision_maker.get_sensor_data();
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
STATE_TYPE LineFollowState::get_next_state(DecisionMaker &decision_maker)
{
  // collision detected
  if (sensor_data_.collision_value_ < COLLISION_DETECTED_THRESHOLD)
  {
    return find_behavior_state(STATE_TYPE::COLLISION);
  }
  else if (sensor_data_.ir_value_ == IR_DETECTED)
  {
    if (check_lane_existance() == true)
    {
      return find_behavior_state(STATE_TYPE::STOP);
    }
    else if ((sensor_data_.read_time_ - none_lane_start_time_) > NONE_LANE_STOP_TIME_MS)
    {
      return find_behavior_state(STATE_TYPE::EMERGENCY_STOP);
    }
    else
    {
      return find_behavior_state(behavior_state_);
    }
  }
  else if (((sensor_data_.read_time_ - none_lane_start_time_) > NONE_LANE_RECOVERY_TIME_MS) &&
           (check_lane_existance() == false))
  {
    return find_behavior_state(STATE_TYPE::RECOVERY);
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

  sensor_data_ = decision_maker.get_sensor_data();
  line_follower_.follow_line(sensor_data_.line_tracing_right_,
                             sensor_data_.line_tracing_left_);
  motor_output = line_follower_.get_motor_output();
  return true;
}

bool LineFollowState::check_lane_existance()
{
  if ((sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD) ||
      (sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD))
  {
    return true;
  }
  else
  {
    none_lane_start_time_ = millis();
    return false;
  }
}

/*****************************************************************************************/
/*****************************************************************************************/
/**************************** ObstacleAvoidanceState *************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE ObstacleAvoidanceState::get_next_state(DecisionMaker &decision_maker)
{
  sensor_data_ = decision_maker.get_sensor_data();
  measure_line_not_detected_time();

  if (sensor_data_.collision_value_ < COLLISION_DETECTED_THRESHOLD)
    return find_behavior_state(STATE_TYPE::COLLISION);

  if (avoidance_success_)
  {
    return find_behavior_state(STATE_TYPE::STOP);
  }

  if (!avoidance_success_ &&
      sensor_data_.collision_value_ >= COLLISION_DETECTED_THRESHOLD &&
      sensor_data_.ir_value_ == IR_DETECTED &&
      line_not_detected_time_ > AVOIDACNE_LINE_NOT_DETETED_TIME_MS)
    return find_behavior_state(STATE_TYPE::EMERGENCY_STOP);

  return find_behavior_state(behavior_state_);
}

bool ObstacleAvoidanceState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  return true;
}

bool ObstacleAvoidanceState::exist_line()
{
  if ((sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD) ||
      (sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD))
    return true;
  else
    return false;
}

void ObstacleAvoidanceState::measure_line_not_detected_time()
{
  if (!exist_line())
  {
    // firstly not detected
    if (previous_line_detected_)
    {
      line_not_detected_start_time_ = millis();
      line_not_detected_time_ = 0;
      previous_line_detected_ = false;
    }
    // continuously not detected
    else
    {
      // update not detected time
      line_not_detected_time_ = millis() - line_not_detected_start_time_;
    }
  }
  else
  {
    line_not_detected_time_ = 0;
    previous_line_detected_ = true;
  }
}

/*****************************************************************************************/
/*****************************************************************************************/
/********************************* CollisionState ****************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE CollisionState::get_next_state(DecisionMaker &decision_maker)
{
  (void)decision_maker;
  return find_behavior_state(STATE_TYPE::NORMAL_TERMINATION);
}

bool CollisionState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  sensor_data_ = decision_maker.get_sensor_data();
  motor_output.right_motor_speed_ = 0;
  motor_output.left_motor_speed_ = 0;
  motor_output.right_motor_mode_ = RELEASE;
  motor_output.left_motor_mode_ = RELEASE;

  /**
   * TODO: airbag exploded situation
   */

  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/******************************** SystemFaultState ***************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE SystemFaultState::get_next_state(DecisionMaker &decision_maker)
{
  if (fault_count_ > FAULT_COUNT_THRESHOLD)
  {
    return find_behavior_state(STATE_TYPE::ABNORMAL_TERMINATION);
  }
  else
  {
    return find_behavior_state(STATE_TYPE::INIT);
  }
}

bool SystemFaultState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  fault_count += 1;
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/****************************** EmergencyStopState ***************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE EmergencyStopState::get_next_state(DecisionMaker &decision_maker)
{
  uint32_t time_difference = millis() - runtime_;

  if (time_difference >= EMERGENCY_STOP_WAIT_TIME_MS)
    return find_behavior_state(STATE_TYPE::NORMAL_TERMINATION);

  return find_behavior_state(behavior_state_);
}

bool EmergencyStopState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  /**
   * TODO: alert emergency stop state
   */
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/***************************** NormalTerminationState ************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE NormalTerminationState::get_next_state(DecisionMaker &decision_maker)
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
STATE_TYPE AbnormalTerminationState::get_next_state(DecisionMaker &decision_maker)
{
  return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
}

bool AbnormalTerminationState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/****************************** RecoveryState **************************************/
/*****************************************************************************************/
/*****************************************************************************************/

STATE_TYPE SystemRecoveryState::get_next_state(DecisionMaker &decision_maker)
{
  if (sensor_data_.collision_value_ == 0)
  {
    return find_behavior_state(STATE_TYPE::COLLISION);
  }
  if (check_lane_existance() == true)
  {
    if ((sensor_data_.ir_value_ == 0))
    {
      return find_behavior_state(STATE_TYPE::STOP);
    }
    else
    {
      return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
    }
  }
  else if ((sensor_data_.read_time_ - none_lane_start_time) > NONE_LANE_RECOVERY_TIME_MS)
  {
    return find_behavior_state(STATE_TYPE::STOP);
  }
  else
  {
    return find_behavior_state(behavior_state_);
  }
}

bool RecoveryState::run(DecisionMaker &decision_maker, MotorOutput &motor_output)
{
  sensor_data_ = decision_maker.get_sensor_data();
  motor_output.right_motor_mode_ = BACKWARD;
  motor_output.left_motor_mode_ = BACKWARD;
  motor_output.right_motor_speed_ = HIGH_MOTOR_SPEED;
  motor_output.left_motor_speed_ = HIGH_MOTOR_SPEED;
  return true;
}

bool RecoveryState::check_lane_existance()
{
  if ((sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD) ||
      (sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD))
  {
    return true;
  }
  else
  {
    none_lane_start_time_ = sensor_data_.read_time_;
    return false;
  }
}
