#include "decision_maker.h"
#include "common_params.h"
#include "line_follower.h"

DecisionMaker *g_decision_maker;

void setup()
{
  Serial.begin(9600);

  g_decision_maker = new DecisionMaker;
}

void loop()
{
  Serial.println("Loop Start...");
  
  // get_sensor_data();

  g_decision_maker->run();

  delay(1000);
}
