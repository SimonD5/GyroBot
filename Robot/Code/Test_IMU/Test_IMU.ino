
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

// Broches I2C de l'ESP32-C6
#define SDA_PIN 6
#define SCL_PIN 7

Adafruit_LSM6DSOX lsm6dsox;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nTest du LSM6DSOX");

  // Initialisation I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Recherche du capteur
  if (!lsm6dsox.begin_I2C(0x6A, &Wire)) {
    Serial.println("ERREUR : LSM6DSOX non detecte !");
    Serial.println("Verifier le cablage et l'adresse I2C.");

    // Arret du programme
    while (1) {
      delay(1000);
    }
  }

  Serial.println("LSM6DSOX detecte avec succes !");

  // Configuration du capteur
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

  Serial.println("Accel : m/s^2");
  Serial.println("Gyro  : rad/s");
  Serial.println("--------------------------------------");
}

void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  // Lecture des mesures
  lsm6dsox.getEvent(&accel, &gyro, &temp);

  // Accelerometre
  Serial.print("A: ");
  Serial.print(accel.acceleration.x, 2);
  Serial.print(" ");
  Serial.print(accel.acceleration.y, 2);
  Serial.print(" ");
  Serial.print(accel.acceleration.z, 2);

  // Gyroscope
  Serial.print(" | G: ");
  Serial.print(gyro.gyro.x, 3);
  Serial.print(" ");
  Serial.print(gyro.gyro.y, 3);
  Serial.print(" ");
  Serial.print(gyro.gyro.z, 3);

  // Temperature
  Serial.print(" | T: ");
  Serial.print(temp.temperature, 1);
  Serial.println(" C");

  delay(100);
}