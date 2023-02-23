int sensor_pin = A5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(sensor_pin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int value = analogRead(sensor_pin);

  Serial.println("Sensor Value: ");
  Serial.println(value);
//  delay(500);
}
