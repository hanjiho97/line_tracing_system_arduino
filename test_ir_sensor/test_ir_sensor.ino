int ir_pin = 7;

void setup() {
  
  pinMode(ir_pin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int value = digitalRead(ir_pin);

  if (value == HIGH)
  {
    Serial.println("value is high");
  }
}
