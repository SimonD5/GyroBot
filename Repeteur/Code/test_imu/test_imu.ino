// Test rapide IMU (MPU6050)
// Ouvrir le moniteur serie a 115200 bauds
// ~90 deg quand il est droit, < 90 en avant, > 90 en arriere

#include <Wire.h>

#define SDA_PIN 2
#define SCL_PIN 3
#define STBY    22   // standby driver moteurs

const uint8_t MPU_ADDR = 0x68;
const float GYRO_SENSIBILITE = 65.5; // +-500 deg/s

// ecriture registre I2C
void mpuEcrire(uint8_t registre, uint8_t valeur) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(registre);
  Wire.write(valeur);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW); // moteurs coupes

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // config du MPU6050
  mpuEcrire(0x6B, 0x00);
  mpuEcrire(0x1A, 0x03);
  mpuEcrire(0x1B, 0x08);
  mpuEcrire(0x1C, 0x00);

  delay(100);
}

void loop() {
  // lecture registres MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("IMU absente – vérifier le câblage I2C");
    delay(500);
    return;
  }

  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  Wire.read(); Wire.read();

  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // calcul de l'angle
  float angleX = atan2((float)ay, (float)az) * RAD_TO_DEG;
  float angleY = atan2(-(float)ax, (float)az) * RAD_TO_DEG;

  float gx_dps = gx / GYRO_SENSIBILITE;
  float gy_dps = gy / GYRO_SENSIBILITE;
  float gz_dps = gz / GYRO_SENSIBILITE;

  Serial.printf("angleX:%.1f,angleY:%.1f,gx:%.1f,gy:%.1f,gz:%.1f\n",
                angleX, angleY, gx_dps, gy_dps, gz_dps);

  delay(50);
}
