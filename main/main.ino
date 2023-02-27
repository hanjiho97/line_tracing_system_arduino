#include "decision_maker.h"
#include "common_params.h"
#include "line_follower.h"

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

DecisionMaker *g_decision_maker;

LiquidCrystal_I2C g_lcd(0x27, 16, 2);

void print_lcd(uint32_t row, uint32_t col, const char* str)
{
  g_lcd.setCursor(row, col);
  g_lcd.print(str);
}

void setup()
{
  Serial.begin(9600);
  
  Serial.println("Setup..."); 
  g_decision_maker = new DecisionMaker;
  Serial.println("Decision Maker...");

  // g_lcd.begin();
  // g_lcd.backlight();
}

void loop()
{
  // Serial.println("Loop Start...");
  
  // get_sensor_data();

  g_decision_maker->run();
  // std::string display_message("STATE: " + g_decision_maker->get_current_state_name());
  // print_lcd(0, 0, display_message.c_str());

  // delay(1000);
}
