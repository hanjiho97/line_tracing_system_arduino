#include "decision_maker.h"


DecisionMaker::DecisionMaker(const STATE_TYPE initial_state)
    : current_state_(initial_state), states_(STATE_TYPE::NUM_STATES)
{
  init_sensor_pin();
  init_motors();

  // define nodes in state machine graph
  states_[STATE_TYPE::INIT] = new InitState;
  states_[STATE_TYPE::STOP] = new StopState;
  states_[STATE_TYPE::LINE_FOLLOW] = new LineFollowState;
  states_[STATE_TYPE::OBSTACLE_AVOIDANCE] = new ObstacleAvoidanceState;
  states_[STATE_TYPE::COLLISION] = new CollisionState;
  states_[STATE_TYPE::SYSTEM_FAULT] = new SystemFaultState;
  states_[STATE_TYPE::EMERGENCY_STOP] = new EmergencyStopState;
  states_[STATE_TYPE::NORMAL_TERMINATION] = new NormalTerminationState;
  states_[STATE_TYPE::ABNORMAL_TERMINATION] = new AbnormalTerminationState;
  states_[STATE_TYPE::SYSTEM_RECOVERY] = new SystemRecoveryState;

  // define edges of INIT_STATE
  states_[STATE_TYPE::INIT]->insert_next_state(states_[STATE_TYPE::STOP]);

  // define edges of STOP_STATE
  states_[STATE_TYPE::STOP]->insert_next_state(states_[STATE_TYPE::LINE_FOLLOW]);
  states_[STATE_TYPE::STOP]->insert_next_state(states_[STATE_TYPE::OBSTACLE_AVOIDANCE]);
  states_[STATE_TYPE::STOP]->insert_next_state(states_[STATE_TYPE::COLLISION]);
  states_[STATE_TYPE::STOP]->insert_next_state(states_[STATE_TYPE::NORMAL_TERMINATION]);
  states_[STATE_TYPE::STOP]->insert_next_state(states_[STATE_TYPE::SYSTEM_RECOVERY]);

  // define edges of LINE_FOLLOW_STATE
  states_[STATE_TYPE::LINE_FOLLOW]->insert_next_state(states_[STATE_TYPE::STOP]);
  states_[STATE_TYPE::LINE_FOLLOW]->insert_next_state(states_[STATE_TYPE::EMERGENCY_STOP]);
  states_[STATE_TYPE::LINE_FOLLOW]->insert_next_state(states_[STATE_TYPE::COLLISION]);

  // define edges of OBSTACLE_AVOIDANCE_STATE
  states_[STATE_TYPE::OBSTACLE_AVOIDANCE]->insert_next_state(states_[STATE_TYPE::STOP]);
  states_[STATE_TYPE::OBSTACLE_AVOIDANCE]->insert_next_state(states_[STATE_TYPE::EMERGENCY_STOP]);
  states_[STATE_TYPE::OBSTACLE_AVOIDANCE]->insert_next_state(states_[STATE_TYPE::COLLISION]);

  // define edges of COLLISION_STATE
  states_[STATE_TYPE::COLLISION]->insert_next_state(states_[STATE_TYPE::NORMAL_TERMINATION]);

  // define edges of SYSTEM_FAULT_STATE
  states_[STATE_TYPE::SYSTEM_FAULT]->insert_next_state(states_[STATE_TYPE::ABNORMAL_TERMINATION]);

  // define edges of EMERGENCY_STOP_STATE
  states_[STATE_TYPE::EMERGENCY_STOP]->insert_next_state(states_[STATE_TYPE::STOP]);

  // define edges of NORMAL_TERMINATION_STATE
  states_[STATE_TYPE::NORMAL_TERMINATION]->insert_next_state(states_[STATE_TYPE::ABNORMAL_TERMINATION]);

}

DecisionMaker::~DecisionMaker()
{
  for (uint32_t i = 0U; i < static_cast<uint32_t>(STATE_TYPE::NUM_STATES); ++i)
  {
    if (states_[i] != nullptr)
      delete states_[i];
  }

  delete right_motor_;
  delete left_motor_;
}

void DecisionMaker::init_motors()
{
  right_motor_ = new AF_DCMotor(RIGHT_MOTOR_NUMBER);
  left_motor_ = new AF_DCMotor(LEFT_MOTOR_NUMBER);
  right_motor_->setSpeed(HIGH_MOTOR_SPEED);
  left_motor_->setSpeed(HIGH_MOTOR_SPEED);
  right_motor_->run(RELEASE);
  left_motor_->run(RELEASE);
}

void DecisionMaker::init_sensor_pin()
{
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT_PULLUP);
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT_PULLUP);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(COLLISION_SENSOR_PIN, INPUT);
}

void DecisionMaker::read_sensor_data()
{
  sensor_data_.line_tracing_right_ = analogRead(RIGHT_LINE_SENSOR_PIN);
  sensor_data_.line_tracing_left_ = analogRead(LEFT_LINE_SENSOR_PIN);
  sensor_data_.ir_value_ = digitalRead(IR_SENSOR_PIN);
  sensor_data_.collision_value_ = digitalRead(COLLISION_SENSOR_PIN); // TODO
  sensor_data_.read_time_ = millis();
}

bool DecisionMaker::check_sensor_data()
{
  if ((sensor_data_.line_tracing_right_ > FAULT_DETECTION_THRESHOLD) ||
   (sensor_data_.line_tracing_left_ > FAULT_DETECTION_THRESHOLD))
  {
    return false;
  }
  else
  {
    return true;
  }
}

void DecisionMaker::write_control_signal(const MotorOutput &motor_output)
{
  right_motor_->setSpeed(motor_output.right_motor_speed_);
  left_motor_->setSpeed(motor_output.left_motor_speed_);

  right_motor_->run(motor_output.right_motor_mode_);
  left_motor_->run(motor_output.left_motor_mode_);
}

void write_display_signal(const DisplayOutput& display_output) // TO DO
{
  return;
}

void DecisionMaker::run()
{
  read_sensor_data();

  STATE_TYPE new_state = current_state_;

  if (check_sensor_data() == true)
  {
    new_state = states_[static_cast<uint32_t>(current_state_)]->get_next_state();
  }
  else
  {
    new_state = STATE_TYPE::SYSTEM_FAULT;
  }

  std::cout << "new_state: " << new_state << std::endl;
  if (new_state != current_state_)
  {
    current_state_ = new_state;
    states_[static_cast<uint32_t>(current_state_)]->reset_timer();
  }

  MotorOutput motor_output;
  if (states_[static_cast<uint32_t>(current_state_)]->run(*this, motor_output))
    std::cout << "Run Success: " << static_cast<uint32_t>(current_state_) << std::endl;
  else
    std::cout << "Run Failure: " << static_cast<uint32_t>(current_state_) << std::endl;

  write_control_signal(motor_output);
}
