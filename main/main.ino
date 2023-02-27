#include "decision_maker.h"
#include "common_params.h"

DecisionMaker* g_decision_maker;

void setup()
{
  Serial.begin(9600);
  
  g_decision_maker = new DecisionMaker;

}

void loop()
{
  g_decision_maker->run();
}
