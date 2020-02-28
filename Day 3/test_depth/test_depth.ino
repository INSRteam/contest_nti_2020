#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

void setup() {
  
  Serial.begin(9600);

  
  Wire.begin();

 
  while (!sensor.init()) {
    Serial.println("bad init!");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(1000);
}

void loop() {

  sensor.read();
  
  Serial.print("Глубина: "); 
  Serial.print(sensor.depth()); 
  Serial.println(" м");

  delay(100);
}
