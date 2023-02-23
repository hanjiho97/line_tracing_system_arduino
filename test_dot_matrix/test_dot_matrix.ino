#include <LedControl.h>
LedControl dot = LedControl(12, 11, 10, 1);

int cnt = 0;
void setup() {
  // put your setup code here, to run once:
  dot.shutdown(0, false);
  dot.setIntensity(0, 3);
  dot.clearDisplay(0);

//  dot.setColumn(0, 0, B00011000);
//  dot.setColumn(0, 0, B00100100);
}

void loop() {
  // put your main code here, to run repeatedly:
  dot.clearDisplay(0);
  if (cnt % 2 == 0)
    dot.setColumn(0, 0, B00011000);
  else 
    dot.setColumn(0, 1, B00100100);

    cnt++;
    delay(1000);
  
}
