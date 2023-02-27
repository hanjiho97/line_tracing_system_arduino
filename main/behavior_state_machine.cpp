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

bool BehaviorStateMachine::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
  // std::cout << _PF_ << "run" << std::endl;
  return true;
}

bool BehaviorStateMachine::display_state(DecisionMaker& decision_maker, DisplayOutput& display_output)
{
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
  std::cout << "******************************************" << std::endl;
  std::cout << "*****************INIT_STATE***************" << std::endl;
  std::cout << "******************************************" << std::endl;
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/*********************************** StopState *******************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE StopState::get_next_state(DecisionMaker& decision_maker)
{
  sensor_data_ = decision_maker.get_sensor_data();
  uint32_t time_difference = millis() - runtime_;

  // collision
  if (sensor_data_.collision_value_ < COLLISION_DETECTED_THRESHOLD)
  {
    return find_behavior_state(STATE_TYPE::COLLISION);
  }
  else
  {
    if ((sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD) ||
        (sensor_data_.line_tracing_left_ > LINE_SENSOR_THRESHOLD))
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
  sensor_data_ = decision_maker.get_sensor_data();
  measure_line_not_detected_time();
  // collision detected
  if (sensor_data_.collision_value_ < COLLISION_DETECTED_THRESHOLD)
  {
    return find_behavior_state(STATE_TYPE::COLLISION);
  }
  else if (sensor_data_.ir_value_ == IR_DETECTED)
  {
    if (exist_line() == true)
    {
      return find_behavior_state(STATE_TYPE::STOP);
    }
    else if (line_not_detected_time_ > EMERGENCY_STOP_NONE_LINE_LIMIT_TIME_MS)
    {
      return find_behavior_state(STATE_TYPE::EMERGENCY_STOP);
    }
    else
    {
      return find_behavior_state(behavior_state_);
    }
  }
  else if ((line_not_detected_time_ > RECOVERY_NONE_LINE_LIMIT_TIME_MS) &&
           (exist_line() == false))
  {
    return find_behavior_state(STATE_TYPE::RECOVERY);
  }
  else
  {
    return find_behavior_state(behavior_state_);
  }
}

bool LineFollowState::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
  // std::cout << _PF_ << "******************************************" << std::endl;
  // std::cout << _PF_ << "************* LineFollowState ************" << std::endl;
  // std::cout << _PF_ << "******************************************" << std::endl;
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

void LineFollowState::measure_line_not_detected_time()
{
  if (exist_line() == false)
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
/**************************** ObstacleAvoidanceState *************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE ObstacleAvoidanceState::get_next_state(DecisionMaker& decision_maker)
{
  sensor_data_ = decision_maker.get_sensor_data();
  measure_line_not_detected_time();

  if (sensor_data_.collision_value_ < COLLISION_DETECTED_THRESHOLD)
  {
    return find_behavior_state(STATE_TYPE::COLLISION);
    restore_time_ = 0;
  }
  else if (sensor_data_.ir_value_ == IR_DETECTED)
  {
    restore_time_ = millis() - runtime_;
    return find_behavior_state(STATE_TYPE::EMERGENCY_STOP);
  }
  else if (avoidance_success_ == true)
  {
    return find_behavior_state(STATE_TYPE::STOP);
    restore_time_ = 0;
  }
  else
  {
    return find_behavior_state(behavior_state_);
  }
}

bool ObstacleAvoidanceState::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
  uint32_t time_difference = millis() - runtime_ + restore_time_;
  //right turn
  if (time_difference < FIRST_CHECKPOINT_TIME_MS)
  {
    motor_output.right_motor_speed_ = LOW_MOTOR_SPEED;
    motor_output.left_motor_speed_ = LOW_MOTOR_SPEED;
    motor_output.right_motor_mode_ = BACKWARD;
    motor_output.left_motor_mode_ = FORWARD;
  }
  //go straight
  else if (time_difference < SECOND_CHECKPOINT_TIME_MS)
  {
    motor_output.right_motor_speed_ = HIGH_MOTOR_SPEED;
    motor_output.left_motor_speed_ = HIGH_MOTOR_SPEED;
    motor_output.right_motor_mode_ = FORWARD;
    motor_output.left_motor_mode_ = FORWARD;
  }
  //left turn
  else if (time_difference < THRID_CHECKPOINT_TIME_MS)
  {
    motor_output.right_motor_speed_ = LOW_MOTOR_SPEED;
    motor_output.left_motor_speed_ = LOW_MOTOR_SPEED;
    motor_output.right_motor_mode_ = FORWARD;
    motor_output.left_motor_mode_ = BACKWARD;
  }
  //go straight
  else if (time_difference < FOURTH_CHECKPOINT_TIME_MS)
  {
    motor_output.right_motor_speed_ = HIGH_MOTOR_SPEED;
    motor_output.left_motor_speed_ = HIGH_MOTOR_SPEED;
    motor_output.right_motor_mode_ = FORWARD;
    motor_output.left_motor_mode_ = FORWARD;
  }
  //left turn
  else if (time_difference < FIFTH_CHECKPOINT_TIME_MS)
  {
    motor_output.right_motor_speed_ = LOW_MOTOR_SPEED;
    motor_output.left_motor_speed_ = LOW_MOTOR_SPEED;
    motor_output.right_motor_mode_ = FORWARD;
    motor_output.left_motor_mode_ = BACKWARD;
  }
  else if (exist_line() == false)
  {
    motor_output.right_motor_speed_ = HIGH_MOTOR_SPEED;
    motor_output.left_motor_speed_ = HIGH_MOTOR_SPEED;
    motor_output.right_motor_mode_ = FORWARD;
    motor_output.left_motor_mode_ = FORWARD;
  }
  else
  {
    avoidance_success_ = true;
  }
  return true;
}

bool ObstacleAvoidanceState::exist_line()
{
  if ((sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD) ||
      (sensor_data_.line_tracing_left_ > LINE_SENSOR_THRESHOLD))
    return true;
  else
    return false;
}

void ObstacleAvoidanceState::measure_line_not_detected_time()
{
  if (exist_line() == false)
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
STATE_TYPE CollisionState::get_next_state(DecisionMaker& decision_maker)
{
  (void)decision_maker;
  return find_behavior_state(STATE_TYPE::NORMAL_TERMINATION);
}

bool CollisionState::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
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
STATE_TYPE SystemFaultState::get_next_state(DecisionMaker& decision_maker)
{
  sensor_data_ = decision_maker.get_sensor_data();
  if (fault_count_ > FAULT_COUNT_THRESHOLD)
  {
    return find_behavior_state(STATE_TYPE::ABNORMAL_TERMINATION);
  }
  else
  {
    return find_behavior_state(STATE_TYPE::INIT);
  }
}

bool SystemFaultState::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
  motor_output.right_motor_speed_ = 0;
  motor_output.left_motor_speed_ = 0;
  motor_output.right_motor_mode_ = RELEASE;
  motor_output.left_motor_mode_ = RELEASE;
  fault_count_ += 1;
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/****************************** EmergencyStopState ***************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE EmergencyStopState::get_next_state(DecisionMaker& decision_maker)
{
  uint32_t time_difference = millis() - runtime_;
  if (sensor_data_.collision_value_ < COLLISION_DETECTED_THRESHOLD)
  {
    return find_behavior_state(STATE_TYPE::COLLISION);
  }
  else if (avoidance_flag_ == true)
  {
    if (sensor_data_.ir_value_ == IR_NOT_DETECTED)
    {
      return find_behavior_state(STATE_TYPE::OBSTACLE_AVOIDANCE);
    }
    else
    {
      return find_behavior_state(behavior_state_);
    }
  }
  else if (time_difference >= EMERGENCY_STOP_NONE_LINE_LIMIT_TIME_MS)
  {
    return find_behavior_state(STATE_TYPE::RECOVERY);
  }
  else
  {
    return find_behavior_state(behavior_state_);
  }
}

bool EmergencyStopState::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
  motor_output.right_motor_speed_ = 0;
  motor_output.left_motor_speed_ = 0;
  motor_output.right_motor_mode_ = RELEASE;
  motor_output.left_motor_mode_ = RELEASE;

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
STATE_TYPE NormalTerminationState::get_next_state(DecisionMaker& decision_maker)
{
  return find_behavior_state(behavior_state_);
}

bool NormalTerminationState::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
  motor_output.right_motor_speed_ = 0;
  motor_output.left_motor_speed_ = 0;
  motor_output.right_motor_mode_ = RELEASE;
  motor_output.left_motor_mode_ = RELEASE;
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/**************************** AbnormalTerminationState ***********************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE AbnormalTerminationState::get_next_state(DecisionMaker& decision_maker)
{
  return find_behavior_state(behavior_state_);
}

bool AbnormalTerminationState::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
  motor_output.right_motor_speed_ = 0;
  motor_output.left_motor_speed_ = 0;
  motor_output.right_motor_mode_ = RELEASE;
  motor_output.left_motor_mode_ = RELEASE;
  return true;
}

/*****************************************************************************************/
/*****************************************************************************************/
/******************************** RecoveryState ******************************************/
/*****************************************************************************************/
/*****************************************************************************************/
STATE_TYPE RecoveryState::get_next_state(DecisionMaker& decision_maker)
{
  sensor_data_ = decision_maker.get_sensor_data();
  if (sensor_data_.collision_value_ == 0)
  {
    return find_behavior_state(STATE_TYPE::COLLISION);
  }
  if (exist_line() == true)
  {
    if ((sensor_data_.ir_value_ == IR_DETECTED))
    {
      return find_behavior_state(STATE_TYPE::STOP);
    }
    else
    {
      return find_behavior_state(STATE_TYPE::LINE_FOLLOW);
    }
  }
  else if (line_not_detected_time_ > RECOVERY_NONE_LINE_LIMIT_TIME_MS)
  {
    return find_behavior_state(STATE_TYPE::STOP);
  }
  else
  {
    return find_behavior_state(behavior_state_);
  }
}

bool RecoveryState::run(DecisionMaker& decision_maker, MotorOutput& motor_output)
{
  motor_output.right_motor_mode_ = BACKWARD;
  motor_output.left_motor_mode_ = BACKWARD;
  motor_output.right_motor_speed_ = HIGH_MOTOR_SPEED;
  motor_output.left_motor_speed_ = HIGH_MOTOR_SPEED;
  return true;
}

bool RecoveryState::exist_line()
{
  if ((sensor_data_.line_tracing_right_ > LINE_SENSOR_THRESHOLD) ||
      (sensor_data_.line_tracing_left_ > LINE_SENSOR_THRESHOLD))
    return true;
  else
    return false;
}

void RecoveryState::measure_line_not_detected_time()
{
  if (exist_line() == false)
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
