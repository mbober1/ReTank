#include <Arduino.h>
#include "motor.hpp"

motor motor1(16, 2, 4, 0);
// motor motor2(14, 15, 4, 1);

void setup() {
  Serial.begin(115200);


  motor1.direction(Direction::FORWARD);
  // motor2.direction(Direction::FORWARD);
  
  // motor1.power(255);
  // motor2.power(255);

}


void loop() {
  delay(10);
}