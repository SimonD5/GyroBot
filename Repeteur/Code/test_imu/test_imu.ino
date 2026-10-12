// ============================================================
//  Test IMU seul – ESP32-C6 + MPU6050
//  Aucun moteur actif (driver en veille)
//
//  Ouvrir le Serial Monitor ou Serial Plotter à 115200 bauds
//  pour visualiser les angles et vitesses angulaires en temps réel.
//
//  Observations mesurées sur le gyrobot :
//    - Angle X ≈ 90° quand le robot est à l'équilibre (vertical)
//    - Accélération X négative quand le robot penche/avance vers l'avant
//    - Accélération X positive quand le robot penche/recule vers l'arrière
// ============================================================

#include <Wire.h>

// ---------- Broches I2C et driver moteurs ----------
#define SDA_PIN 2    // Broche SDA du bus I2C (relier à SDA du MPU6050)
#define SCL_PIN 3    // Broche SCL du bus I2C (relier à SCL du MPU6050)
#define STBY    22   // Broche STANDBY du driver moteurs – maintenue LOW pour désactiver les moteurs

// ---------- Adresse I2C du MPU6050 ----------
// AD0 relié à GND → adresse 0x68
// AD0 relié à 3.3V → adresse 0x69
const uint8_t MPU_ADDR = 0x68;

// ---------- Sensibilité du gyroscope ----------
// Dépend du réglage du registre 0x1B (GYRO_CONFIG) :
//   0x00 → ±250  °/s  → diviseur = 131.0
//   0x08 → ±500  °/s  → diviseur = 65.5   ← valeur utilisée ici
//   0x10 → ±1000 °/s  → diviseur = 32.8
//   0x18 → ±2000 °/s  → diviseur = 16.4
const float GYRO_SENSIBILITE = 65.5;

// ============================================================
//  Fonction utilitaire : écrire un octet dans un registre MPU
// ============================================================
void mpuEcrire(uint8_t registre, uint8_t valeur) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(registre);   // sélection du registre cible
  Wire.write(valeur);     // valeur à écrire
  Wire.endTransmission();
}

// ============================================================
//  SETUP – exécuté une seule fois au démarrage
// ============================================================
void setup() {
  Serial.begin(115200);

  // Mettre le driver moteurs en veille (sécurité : pas de mouvement pendant le test)
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);

  // Initialisation du bus I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // vitesse I2C : 400 kHz (mode rapide)

  // -- Configuration du MPU6050 --
  mpuEcrire(0x6B, 0x00);  // Sortir du mode veille (registre PWR_MGMT_1)
  mpuEcrire(0x1A, 0x03);  // Filtre passe-bas numérique : ~44 Hz (réduit le bruit)
  mpuEcrire(0x1B, 0x08);  // Gyroscope : plage ±500 °/s  → changer ici si nécessaire
  mpuEcrire(0x1C, 0x00);  // Accéléromètre : plage ±2 g   → changer ici si nécessaire

  delay(100); // laisser le capteur se stabiliser
}

// ============================================================
//  LOOP – exécuté en boucle
// ============================================================
void loop() {

  // --- Demander une lecture à partir du registre 0x3B (premier registre accéléro) ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("IMU absente – vérifier le câblage I2C");
    delay(500);
    return;
  }

  // Lire 14 octets : 6 accéléro + 2 température + 6 gyroscope
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return;

  // -- Accéléromètre (bruts, 16 bits signés) --
  int16_t ax = (Wire.read() << 8) | Wire.read();  // axe X
  int16_t ay = (Wire.read() << 8) | Wire.read();  // axe Y
  int16_t az = (Wire.read() << 8) | Wire.read();  // axe Z

  // -- Température (non utilisée ici, on vide les 2 octets) --
  Wire.read(); Wire.read();

  // -- Gyroscope (bruts, 16 bits signés) --
  int16_t gx = (Wire.read() << 8) | Wire.read();  // vitesse angulaire axe X
  int16_t gy = (Wire.read() << 8) | Wire.read();  // vitesse angulaire axe Y
  int16_t gz = (Wire.read() << 8) | Wire.read();  // vitesse angulaire axe Z

  // -- Calcul des angles à partir de l'accéléromètre (en degrés) --
  // angleX : inclinaison avant/arrière (≈ 90° à l'équilibre vertical)
  float angleX = atan2((float)ay, (float)az) * RAD_TO_DEG;

  // angleY : inclinaison gauche/droite
  float angleY = atan2(-(float)ax, (float)az) * RAD_TO_DEG;

  // -- Conversion gyroscope en °/s --
  float gx_dps = gx / GYRO_SENSIBILITE;
  float gy_dps = gy / GYRO_SENSIBILITE;
  float gz_dps = gz / GYRO_SENSIBILITE;

  // -- Envoi sur le port série (compatible Serial Plotter) --
  Serial.printf("angleX:%.1f,angleY:%.1f,gx:%.1f,gy:%.1f,gz:%.1f\n",
                angleX, angleY, gx_dps, gy_dps, gz_dps);

  delay(50); // fréquence d'envoi ≈ 20 Hz – augmenter si besoin de plus de réactivité
}