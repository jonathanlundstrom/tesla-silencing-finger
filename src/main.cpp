#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Pin definitions
#define IRL540_PIN        3
#define SERVO_PIN         A4
#define SAMPLE_RATE_MS    10
#define PLOTTING_OUTPUT   false
#define FINGER_DELAY      5000

Servo finger;
int position = 0;
float acceleration_x;
float acceleration_y;
float acceleration_z;
bool has_muted = false;
int finger_resting_position = 180;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup_adxl() {
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
}

void enable_servo() {
  digitalWrite(IRL540_PIN, HIGH);
}

void disable_servo() {
  digitalWrite(IRL540_PIN, LOW);
}

void move_finger_to(int degrees) {
  Serial.printf("Moving finger to position %d\n", degrees);
  finger.write(degrees);
}

void reset_finger() {
  enable_servo();
  delay(100);
  move_finger_to(finger_resting_position);
  delay(100);
  disable_servo();
}

void setup_servo() {
  pinMode(IRL540_PIN, OUTPUT);
  finger.attach(SERVO_PIN);
  reset_finger();
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  setup_adxl();
  setup_servo();
}

void loop() {
  if (!has_muted) {
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    acceleration_x = event.acceleration.x;
    acceleration_y = event.acceleration.y;
    acceleration_z = event.acceleration.z;

    if (PLOTTING_OUTPUT) {
      Serial.print("X:");
      Serial.print(event.acceleration.x, 4);
      Serial.print(",");
      Serial.print("Y:");
      Serial.print(event.acceleration.y, 4);
      Serial.print(",");
      Serial.print("Z:");
      Serial.print(event.acceleration.z, 4);
      Serial.println();
    }

    if (acceleration_y >= 4) {
      int position = finger_resting_position;
      Serial.println("Acceleration detected, preparing to deploy THE FINGER!");
      enable_servo();
      delay(FINGER_DELAY);
      
      Serial.println("Deploying the finger");
      while (position > -1) {
        move_finger_to(position);
        position -= 1;
        delay(5);
      }

      delay(500);

      Serial.println("Stowing the finger");
      while (position <= finger_resting_position) {
        move_finger_to(position);
        position += 1;
        delay(5);
      }

      disable_servo();
      has_muted = true;
    }

    delay(SAMPLE_RATE_MS);
  } else {
    Serial.println("Finger has already been deployed. Not doing anything until unit is restarted...");
    delay(5000);
  }
}