#include "decision_maker.h"
#include "common_params.h"
#include "line_follower.h"

DecisionMaker *g_decision_maker_;
// NarrowLineFollower *g_line_follower_;

// AF_DCMotor g_right_motor_(RIGHT_MOTOR_NUMBER);
// AF_DCMotor g_left_motor_(LEFT_MOTOR_NUMBER);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  std::cout << "setup() start" << std::endl;

  // setConfigParam()
  g_decision_maker_ = new DecisionMaker;
  // g_line_follower_ = new NarrowLineFollower;
}

void loop()
{
  // Serial.println(".hello")

  // g_line_follower_->follow_line();
  g_decision_maker_->run();
  delay(100);
}
