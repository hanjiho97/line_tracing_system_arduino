#include "common_params.h"
// #include "decision_maker.h"

SensorData* g_sensor_data;
// DecisionMaker* g_decision_maker;

void setup()
{
  Serial.begin(9600);

  std::cout << "setup() start" << std::endl;
  std::cout << "set sensor pin" << std::endl;
  set_sensor_pin()
  std::cout << "done!" << std::endl;

  // g_sensor_data = new SensorData;
  // g_decision_maker = new DecisionMaker;
}

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

void loop()
{
  get_sensor_data();
  std::cout << "right : " << g_sensor_data.line_tracing_right_ << std::endl;
  std::cout << "left : " << g_sensor_data.line_tracing_left_ << std::endl;
  std::cout << "ir : " << g_sensor_data.ir_value_ << std::endl;
  // g_decision_maker.setSensorData(g_sensor_data);
  // g_decision_maker->run();
}