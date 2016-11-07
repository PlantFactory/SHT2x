#include <Wire.h>
#include "SHT2x.h"

SHT2x sensor;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  sensor.begin();

  Serial.println("Reading...");
  Serial.println(sensor.readTemperature());
  Serial.println(sensor.readHumidity());

  Serial.println(sensor.heaterOn());
  Serial.println("Heater On");

  Serial.println(sensor.heaterOff());
  Serial.println("Heater Off");
}

void loop() {
  // put your main code here, to run repeatedly:

}
