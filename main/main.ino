#include "decision_maker.h"
#include "common_params.h"

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

DecisionMaker* g_decision_maker;

LiquidCrystal_I2C g_lcd(0x27, 16, 2);

void setup()
{
  Serial.begin(9600);
  
  g_decision_maker = new DecisionMaker;

  g_lcd.begin();
  g_lcd.backlight();
}

void loop()
{
  g_decision_maker->run();
  g_decision_maker->display_current_state(g_lcd);
}
