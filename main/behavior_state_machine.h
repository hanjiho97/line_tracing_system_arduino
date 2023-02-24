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
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);

  STATE_TYPE find_behavior_state(const STATE_TYPE &behavior);

protected:
  // std::vector<std::pair<BehaviorStateMachine *, int>> behavior_log_;
  STATE_TYPE behavior_state_;
  std::vector<BehaviorStateMachine *> p_next_states_;
  uint32_t runtime_;
};

class ForwardState : public BehaviorStateMachine
{
public:
  ForwardState()
      : BehaviorStateMachine(STATE_TYPE::LINE_FOLLOW)
  {}
  virtual ~ForwardState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);
};

class InitState : public BehaviorStateMachine
{
public:
  InitState()
      : BehaviorStateMachine(STATE_TYPE::INIT)
  {}
  virtual ~InitState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);
};

class StopState : public BehaviorStateMachine
{
private:
  const SensorData& sensor_data;
public:
  StopState()
      : BehaviorStateMachine(STATE_TYPE::STOP)
  {}
  bool check
  virtual ~StopState() {}
  virtual STATE_TYPE get_next_state();
  virtual bool run(DecisionMaker &decision_maker, MotorOuput &motor_output);
};

#endif