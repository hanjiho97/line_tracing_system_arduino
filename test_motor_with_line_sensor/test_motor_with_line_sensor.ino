#include <AFMotor.h>
uint8_t RIGHT_LINE_SENSOR_PIN = A5;
uint8_t LEFT_LINE_SENSOR_PIN = A0;
uint16_t BOARD_RATE = 9600;

AF_DCMotor right_motor(1);
AF_DCMotor left_motor(2);

void setup() 
{
  Serial.begin(BOARD_RATE);
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT);
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT);
  right_motor.setSpeed(200);
  right_motor.run(RELEASE);
  left_motor.setSpeed(200);
  left_motor.run(RELEASE);
}

void loop() 
{
  uint8_t right_sensor_value = digitalRead(RIGHT_LINE_SENSOR_PIN);
  uint8_t left_sensor_value = digitalRead(LEFT_LINE_SENSOR_PIN);
  if ((right_sensor_value == 0) && (left_sensor_value == 0))
  {
    right_motor.run(FORWARD);
    left_motor.run(FORWARD);
  }
  else if ((right_sensor_value == 1) && (left_sensor_value == 0))
  {
    right_motor.run(FORWARD);
    left_motor.run(RELEASE);
  }
  else if ((right_sensor_value == 0) && (left_sensor_value == 1))
  {
    right_motor.run(RELEASE);
    left_motor.run(FORWARD);
  }
  else if ((right_sensor_value == 1) && (left_sensor_value == 1))
  {
    right_motor.run(RELEASE);
    left_motor.run(RELEASE);
  }
}
