#include "decision_maker.h"
#include "common_params.h"
#include "narrow_line_follower.h"

DecisionMaker *g_decision_maker_;
SensorData g_sensor_data_;
NarrowLineFollower *g_line_follower_;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  std::cout << "start" << std::endl;

  g_decision_maker_ = new DecisionMaker(STATE_TYPE::INIT);
  g_line_follower_ = new NarrowLineFollower;
}

void loop()
{
  // Serial.println(".hello")
  g_line_follower_->follow_line();
}
