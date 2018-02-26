#include "SonarEZ0pw.h"
SonarEZ0pw Sonar(7); // pin D7
float value=0.00;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly: 
  value= Sonar.Distance(cm);
  Serial.print("Distance= " );
  Serial.print(value);
  Serial.println(" cm ");
  delay(100);
}
