uint8_t RIGHT_LINE_SENSOR_PIN = 9;
uint32_t BOARD_RATE = 9600;

void setup() 
{
  Serial.begin(BOARD_RATE);
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT);
}

void loop() 
{
  int right_sensor_value = digitalRead(RIGHT_LINE_SENSOR_PIN);
  Serial.println(right_sensor_value);
}
