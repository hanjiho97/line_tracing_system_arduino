#include "decision_maker.h"

DecisionMaker::DecisionMaker(const STATE_TYPE initial_state)
    : current_state_(initial_state), states_(STATE_TYPE::NUM_STATES)
{
  init_sensor_pin();
  init_motors();

  // define nodes in state machine graph
  states_[STATE_TYPE::INIT] = new InitState;
  states_[STATE_TYPE::STOP] = new StopState;
  states_[STATE_TYPE::LINE_FOLLOW] = new LineFollowState;;

  // define edges of INIT_STATE
  states_[STATE_TYPE::INIT]->insert_next_state(states_[STATE_TYPE::STOP]);

  // define edges of STOP_STATE
  states_[STATE_TYPE::STOP]->insert_next_state(states_[STATE_TYPE::LINE_FOLLOW]);

  // define edges of LINE_FOLLOW_STATE
  states_[STATE_TYPE::LINE_FOLLOW]->insert_next_state(states_[STATE_TYPE::STOP]);
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
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(COLLISION_SENSOR_PIN, INPUT_PULLUP);
}

void DecisionMaker::read_sensor_data()
{
  sensor_data_.line_tracing_right_ = digitalRead(RIGHT_LINE_SENSOR_PIN) * 1000;
  sensor_data_.line_tracing_left_ = digitalRead(LEFT_LINE_SENSOR_PIN) * 1000;
  sensor_data_.ir_value_ = -1;
  sensor_data_.collision_value_ = 1000; // TODO
  sensor_data_.read_time_ = millis();
}

bool DecisionMaker::check_sensor_data()
{
  return true;
}

void DecisionMaker::write_control_signal(const MotorOutput& motor_output)
{
  right_motor_->setSpeed(motor_output.right_motor_speed_);
  left_motor_->setSpeed(motor_output.left_motor_speed_);

  right_motor_->run(motor_output.right_motor_mode_);
  left_motor_->run(motor_output.left_motor_mode_);
}

void DecisionMaker::run()
{
  read_sensor_data();

  STATE_TYPE new_state = states_[static_cast<uint32_t>(current_state_)]->get_next_state(*this);
  if (new_state != current_state_)
  {
    current_state_ = new_state;
    states_[static_cast<uint32_t>(current_state_)]->reset_timer();
    states_[static_cast<uint32_t>(current_state_)]->reset_parameters();
  }

  MotorOutput motor_output;
  states_[static_cast<uint32_t>(current_state_)]->run(*this, motor_output);
  // if (states_[static_cast<uint32_t>(current_state_)]->run(*this, motor_output))

  //   // std::cout << "Run Success: " << STATE_STR[new_state] << std::endl;
  //   std::cout << "Run Success: " << static_cast<uint32_t>(current_state_) << std::endl;
  // else
  //   std::cout << "Run Failure: " << static_cast<uint32_t>(current_state_) << std::endl;
  write_control_signal(motor_output);
}
