#ifndef _DECISION_MAKER_H_
#define _DECISION_MAKER_H_

#include "behavior_state_machine.h"
#include "line_follower.h"
#include "init_state.h"
#include "stop_state.h"
#include "forward_state.h"
#include "common_params.h"

#include <ArduinoSTL.h>
#include <vector>

class DecisionMaker
{
public:
  DecisionMaker(const STATE_TYPE initial_state = STATE_TYPE::INIT);
  ~DecisionMaker();

  void read_sensor_data();
  void write_control_signal();
  void run();

  inline void
  set_sensor_data(const SensorData &sensor_data)
  {
    sensor_data_ = sensor_data;
  }

protected:
  void init_sensor_pin();

  STATE_TYPE current_state_;
  std::vector<BehaviorStateMachine *> states_;
  SensorData sensor_data_;

  LineFollower line_follower_;
  AF_DCMotor *right_motor_;
  AF_DCMotor *left_motor_;
};

#endif
