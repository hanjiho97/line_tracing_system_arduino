#include "decision_maker.h"
#include "common_params.h"
#include "line_follower.h"

DecisionMaker *g_decision_maker;
SensorData g_sensor_data;

void set_sensor_pin()
{
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT);
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
}

void get_sensor_data()
{
  g_sensor_data.line_tracing_right_ = analogRead(RIGHT_LINE_SENSOR_PIN);
  g_sensor_data.line_tracing_left_ = analogRead(LEFT_LINE_SENSOR_PIN);
  g_sensor_data.ir_value_ = digitalRead(IR_SENSOR_PIN);
}

void setup()
{
  Serial.begin(9600);
  
  set_sensor_pin();

  g_decision_maker = new DecisionMaker;
}

void loop()
{
  Serial.println("Loop Start...");
  
  get_sensor_data();

  // std::cout << "right : " << g_sensor_data.line_tracing_right_ << std::endl;
  // std::cout << "left : " << g_sensor_data.line_tracing_left_ << std::endl;
  // std::cout << "ir : " << g_sensor_data.ir_value_ << std::endl;
  g_decision_maker->set_sensor_data(g_sensor_data);
  g_decision_maker->run();
  delay(1000);
}