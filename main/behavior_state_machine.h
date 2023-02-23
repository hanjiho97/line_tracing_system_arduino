#ifndef _BEHAVIOR_STATE_MACHINE_H_
#define _BEHAVIOR_STATE_MACHINE_H_

#include <ArduinoSTL.h>
#include <vector>

enum STATE_TYPE
{
  INIT,
  STOP, // main decision
  EMERGENCY_STOP,
  DONE,
  FORWARD,
  PARKING,
  AVOIDANCE,
  COLLISION_STOP,
  THEFT_EMERGENCY,
  NUM_STATES
};

class BehaviorStateMachine
{
public:
  BehaviorStateMachine(STATE_TYPE behavior_state)
      : behavior_state_(behavior_state)
  {
    p_next_states_.push_back(this);
  }
  virtual ~BehaviorStateMachine(){};

  virtual BehaviorStateMachine *get_next_state() = 0;
  virtual void init();
  virtual void reset_timer();
  virtual void insert_next_state(BehaviorStateMachine *next_state);
  virtual bool run() = 0;

  BehaviorStateMachine *find_behavior_state(const STATE_TYPE &behavior);
  BehaviorStateMachine *find_best_state(int n_min_count);

private:
  std::vector<std::pair<BehaviorStateMachine *, int>> behavior_log_;
  STATE_TYPE behavior_state_;
  std::vector<BehaviorStateMachine *> p_next_states_;
  uint64_t time_;
};

#endif