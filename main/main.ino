#include "decision_maker.h"
#include "common_params.h"
#include "line_follower.h"

DecisionMaker *g_decision_maker;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  g_decision_maker = new DecisionMaker;
}

void loop()
{
  Serial.println("Loop Start...");
  g_decision_maker->run();
  delay(1000);
}
