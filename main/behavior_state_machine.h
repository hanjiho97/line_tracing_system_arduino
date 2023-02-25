#ifndef _BEHAVIOR_STATE_MACHINE_H_
#define _BEHAVIOR_STATE_MACHINE_H_

#include "common_params.h"
#include "decision_maker.h"

#include <ArduinoSTL.h>
#include <vector>

class DecisionMaker;
class LineFollower;

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

  virtual STATE_TYPE get_next_state() = 0;
  virtual void init();
  virtual void reset_timer();
  virtual void insert_next_state(BehaviorStateMachine *next_state);
  // virtual void print_state(BehaviorStateMachine *next_state);
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);

  STATE_TYPE find_behavior_state(const STATE_TYPE &behavior);

protected:
  // std::vector<std::pair<BehaviorStateMachine *, int>> behavior_log_;
  STATE_TYPE behavior_state_;
  std::vector<BehaviorStateMachine *> p_next_states_;
  uint32_t runtime_;
  SensorData sensor_data_;
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
      : BehaviorStateMachine(STATE_TYPE::LINE_FOLLOW)
  {
  }
  virtual ~LineFollowState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);
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
      : BehaviorStateMachine(STATE_TYPE::INIT)
  {
  }
  virtual ~InitState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);
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
      : BehaviorStateMachine(STATE_TYPE::STOP)
  {
  }
  virtual ~StopState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);
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
      : BehaviorStateMachine(STATE_TYPE::EMERGENCY_STOP)
  {
  }
  virtual ~EmergencyStopState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);
};

/*****************************************************************************************/
/*****************************************************************************************/
/********************************** ParkingState *****************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class ParkingState : public BehaviorStateMachine
{
public:
  ParkingState()
      : BehaviorStateMachine(STATE_TYPE::PARKING)
  {
  }
  virtual ~ParkingState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);
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
  {
  }
  virtual ~ObstacleAvoidanceState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);
};

/*****************************************************************************************/
/*****************************************************************************************/
/******************************* CollisionStopState **************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class CollisionStopState : public BehaviorStateMachine
{
public:
  CollisionStopState()
      : BehaviorStateMachine(STATE_TYPE::COLLISION_STOP)
  {
  }
  virtual ~CollisionStopState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);
};

/*****************************************************************************************/
/*****************************************************************************************/
/****************************** TheftEmergencyState **************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class TheftEmergencyState : public BehaviorStateMachine
{
public:
  TheftEmergencyState()
      : BehaviorStateMachine(STATE_TYPE::THEFT_EMERGENCY)
  {
  }
  virtual ~TheftEmergencyState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);
};

/*****************************************************************************************/
/*****************************************************************************************/
/*********************************** DoneState *******************************************/
/*****************************************************************************************/
/*****************************************************************************************/
class DoneState : public BehaviorStateMachine
{
public:
  DoneState()
      : BehaviorStateMachine(STATE_TYPE::DONE)
  {
  }
  virtual ~DoneState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);
};

#endif