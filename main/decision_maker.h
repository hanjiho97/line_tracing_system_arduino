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

  void read_sensor_data();
  bool check_sensor_data();
  void write_control_signal(const MotorOutput& motor_output);
  void run();

  SensorData& get_sensor_data()
  {
    return sensor_data_;
  }

  void get_sensor_data(SensorData& sensor_data)
  {
    sensor_data = sensor_data_;
  }

protected:
  void init_motors();
  void init_sensor_pin();

  STATE_TYPE current_state_;
  std::vector<BehaviorStateMachine*> states_;
  SensorData sensor_data_;

  AF_DCMotor* right_motor_;
  AF_DCMotor* left_motor_;
};

#endif
