#ifndef _DECISION_MAKER_H_
#define _DECISION_MAKER_H_

#include <ArduinoSTL.h>
#include <vector>

#include "behavior_state_machine.h"
#include "common_params.h"

class BehaviorStateMachine;

class DecisionMaker
{
public:
  DecisionMaker(const STATE_TYPE initial_state = STATE_TYPE::INIT);
  ~DecisionMaker();

  void run();

  SensorData& get_sensor_data()
  {
    return sensor_data_;
  }

  // std::string get_current_state_name()
  // {
  //   return states_[static_cast<uint32_t>(current_state_)]->get_state_name();
  // }

protected:
  void init_motors();
  void init_sensor_pin();
  bool check_sensor_data();
  void read_sensor_data();
  void write_control_signal(const MotorOutput& motor_output);
  void write_display_signal(const DisplayOutput& display_output);

  STATE_TYPE current_state_;
  std::vector<BehaviorStateMachine*> states_;
  SensorData sensor_data_;

  AF_DCMotor* right_motor_;
  AF_DCMotor* left_motor_;
};

#endif
