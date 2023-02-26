#ifndef _BEHAVIOR_STATE_MACHINE_H_
#define _BEHAVIOR_STATE_MACHINE_H_

#include <ArduinoSTL.h>
#include <vector>

#include "common_params.h"
#include "line_follower.h"

class DecisionMaker;

class BehaviorStateMachine
{
public:
  BehaviorStateMachine(STATE_TYPE behavior_state)
      : behavior_state_(behavior_state)
  {
    p_next_states_.push_back(this);
    init();
  }
  virtual ~BehaviorStateMachine(){};
  virtual STATE_TYPE get_next_state(DecisionMaker &decision_maker) = 0;
  virtual void init();
  virtual void reset_timer();
  virtual void insert_next_state(BehaviorStateMachine *next_state);
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
  virtual bool display_state(DecisionMaker &decision_maker, DisplayOutput &display_output); // TO DO
  virtual void reset_parameters()
  {
    runtime_ = 0;
  }

  STATE_TYPE find_behavior_state(const STATE_TYPE &behavior);

  void set_sensor_data(const SensorData &sensor_data)
  {
    sensor_data_ = sensor_data;
  }

protected:
  uint32_t runtime_;
  STATE_TYPE behavior_state_;
  std::vector<BehaviorStateMachine *> p_next_states_;
  SensorData sensor_data_;
};

/*****************************************************************************************/
/*****************************************************************************************/
/*********************************** InitState *******************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class InitState : public BehaviorStateMachine
{
public:
  InitState()
      : BehaviorStateMachine(STATE_TYPE::INIT) {}
  virtual ~InitState() {}
  virtual STATE_TYPE get_next_state(DecisionMaker &decision_maker);
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
  virtual void reset_parameters() {}
};

/*****************************************************************************************/
/*****************************************************************************************/
/*********************************** StopState *******************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class StopState : public BehaviorStateMachine
{
public:
  StopState()
      : BehaviorStateMachine(STATE_TYPE::STOP) {}
  virtual ~StopState() {}
  virtual STATE_TYPE get_next_state(DecisionMaker &decision_maker);
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
  virtual void reset_parameters() {}
};

/*****************************************************************************************/
/*****************************************************************************************/
/******************************** LineFollowState ****************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class LineFollowState : public BehaviorStateMachine
{
public:
  LineFollowState()
      : BehaviorStateMachine(STATE_TYPE::LINE_FOLLOW), none_lane_start_time_(0) {}
  virtual ~LineFollowState() {}
  virtual STATE_TYPE get_next_state(DecisionMaker &decision_maker);
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
  virtual void reset_parameters()
  {
    none_lane_start_time_ = 0;
  }

private:
  bool check_lane_existance();

  LineFollower line_follower_;
  uint32_t none_lane_start_time_;
};

/*****************************************************************************************/
/*****************************************************************************************/
/**************************** ObstacleAvoidanceState *************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class ObstacleAvoidanceState : public BehaviorStateMachine
{
public:
  ObstacleAvoidanceState()
      : BehaviorStateMachine(STATE_TYPE::OBSTACLE_AVOIDANCE)
      , avoidance_success_(false)
      , previous_line_detected_(false)
      , line_not_detected_time_(0)
      , line_not_detected_start_time_(0) {}
  virtual ~ObstacleAvoidanceState() {}
  virtual STATE_TYPE get_next_state(DecisionMaker &decision_maker);
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
  virtual void reset_parameters()
  {
    avoidance_success_ = false;
    previous_line_detected_ = false;
    line_not_detected_time_ = 0;
    line_not_detected_start_time_ = 0;
  }

private:
  bool exist_line();
  void measure_line_not_detected_time();

  bool avoidance_success_;
  bool previous_line_detected_;
  uint32_t line_not_detected_time_;
  uint32_t line_not_detected_start_time_;
};

/*****************************************************************************************/
/*****************************************************************************************/
/********************************* CollisionState ****************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class CollisionState : public BehaviorStateMachine
{
public:
  CollisionState()
      : BehaviorStateMachine(STATE_TYPE::COLLISION)
  {
  }
  virtual ~CollisionState() {}
  virtual STATE_TYPE get_next_state(DecisionMaker &decision_maker);
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
  virtual void reset_parameters() {}
};

/*****************************************************************************************/
/*****************************************************************************************/
/******************************** SystemFaultState ***************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class SystemFaultState : public BehaviorStateMachine
{
public:
  SystemFaultState()
      : BehaviorStateMachine(STATE_TYPE::SYSTEM_FAULT), fault_count(0) {}
  virtual ~SystemFaultState() {}
  virtual STATE_TYPE get_next_state(DecisionMaker &decision_maker);
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
  virtual void reset_parameters()
  {
    fault_count_ = 0;
  }

private:
  uint8_t fault_count_;
};

/*****************************************************************************************/
/*****************************************************************************************/
/******************************* EmergencyStopState **************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class EmergencyStopState : public BehaviorStateMachine
{
public:
  EmergencyStopState()
      : BehaviorStateMachine(STATE_TYPE::EMERGENCY_STOP) {}
  virtual ~EmergencyStopState() {}
  virtual STATE_TYPE get_next_state(DecisionMaker &decision_maker);
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
  virtual void reset_parameters() {}
};

/*****************************************************************************************/
/*****************************************************************************************/
/****************************** NormalTerminationState ***********************************/
/*****************************************************************************************/
/*****************************************************************************************/
class NormalTerminationState : public BehaviorStateMachine
{
public:
  NormalTerminationState()
      : BehaviorStateMachine(STATE_TYPE::NORMAL_TERMINATION) {}
  virtual ~NormalTerminationState() {}
  virtual STATE_TYPE get_next_state(DecisionMaker &decision_maker);
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
  virtual void reset_parameters() {}
};

/*****************************************************************************************/
/*****************************************************************************************/
/**************************** AbnormalTerminationState ***********************************/
/*****************************************************************************************/
/*****************************************************************************************/
class AbnormalTerminationState : public BehaviorStateMachine
{
public:
  AbnormalTerminationState()
      : BehaviorStateMachine(STATE_TYPE::ABNORMAL_TERMINATION) {}
  virtual ~AbnormalTerminationState() {}
  virtual STATE_TYPE get_next_state(DecisionMaker &decision_maker);
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
  virtual void reset_parameters() {}
};

/*****************************************************************************************/
/*****************************************************************************************/
/******************************** RecoveryState ************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class RecoveryState : public BehaviorStateMachine
{
public:
  RecoveryState()
      : BehaviorStateMachine(STATE_TYPE::RECOVERY), none_lane_start_time_(0) {}
  virtual ~RecoveryState() {}
  virtual STATE_TYPE get_next_state(DecisionMaker &decision_maker);
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
  bool check_lane_existance();
  virtual void reset_parameters()
  {
    none_lane_start_time_ = 0;
  }

private:
  uint32_t none_lane_start_time_;
};

#endif
