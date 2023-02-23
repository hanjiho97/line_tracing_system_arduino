#include <NarrowLineFollow.h>

NarrowLineFollower line_module;

void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  line_module.follow_line();
}
