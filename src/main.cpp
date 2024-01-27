#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("Hello!");
  delay(1000);
}