#ifndef _BEHAVIOR_STATE_MACHINE_H_
#define _BEHAVIOR_STATE_MACHINE_H_

#include <ArduinoSTL.h>
#include <vector>

#include "common_params.h"
#include "decision_maker.h"
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
  virtual ~BehaviorStateMachine() {};
  virtual STATE_TYPE get_next_state() = 0;
  virtual void init();
  virtual void reset_timer();
  virtual void insert_next_state(BehaviorStateMachine *next_state);
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);

  STATE_TYPE find_behavior_state(const STATE_TYPE &behavior);

protected:
  uint32_t runtime_;
  STATE_TYPE behavior_state_;
  std::vector<BehaviorStateMachine*> p_next_states_;
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
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
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
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);

private:
  uint32_t time_differance;
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
  : BehaviorStateMachine(STATE_TYPE::LINE_FOLLOW), none_lane_start_time(0) {}
  virtual ~LineFollowState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
  bool check_lane_existance();

private:
  LineFollower line_follower_;
  uint32_t none_lane_start_time;
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
  : BehaviorStateMachine(STATE_TYPE::OBSTACLE_AVOIDANCE) {}
  virtual ~ObstacleAvoidanceState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
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
  {}
  virtual ~CollisionState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
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
  : BehaviorStateMachine(STATE_TYPE::SYSTEM_FAULT) {}
  virtual ~SystemFaultState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
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
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
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
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
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
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
};

/*****************************************************************************************/
/*****************************************************************************************/
/******************************** SystemRecoveryState ************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class SystemRecoveryState : public BehaviorStateMachine
{
public:
  SystemRecoveryState()
  : BehaviorStateMachine(STATE_TYPE::SYSTEM_RECOVERY) {}
  virtual ~SystemRecoveryState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOutput &motor_output);
};

#endif
