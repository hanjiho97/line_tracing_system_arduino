#include "decision_maker.h"

DecisionMaker *g_decision_maker_;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  std::cout << "start" << std::endl;

  g_decision_maker_ = new DecisionMaker(STATE_TYPE::INIT);
}

void loop() 
{
  // Serial.println(".hello")
}
