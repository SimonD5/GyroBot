// Reglage PID par port serie USB (921600 bauds)

#include <Wire.h>

#define SDA_PIN 2
#define SCL_PIN 3

#define PWMA 11
#define AIN1 7
#define AIN2 10
#define PWMB 23
#define BIN1 21
#define BIN2 20
#define STBY 22

const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int PWM_MAX  = 200;

// moteur b câblé à l'envers
const int SIGN_MOTEUR_A = 1;
const int SIGN_MOTEUR_B = -1;

float Kp = 15.0;
float Ki = 0.0;
float Kd = 1.0;
float consigne = 7.5;
int   pwmMin = 20;

// securite
const float ANGLE_CHUTE   = 18.0;
const float ANGLE_REPRISE = 6.0;
const float I_MAX         = 80.0;
const float ZONE_RAMPE    = 30.0;

// filtre gyro pour le d
const float ALPHA_D = 0.4;

// filtre complémentaire
const float ALPHA = 0.98;

// 200 hz = 5 ms
const uint32_t PERIODE_US = 5000;

// télémétrie 100 hz
const uint32_t TELEMETRIE_US = 10000;

// mpu6050
const uint8_t MPU_ADDR = 0x68;
float gyroBias = 0;
float gyroSign = -1.0;

float rateF = 0;
float angleAcc = 0;
float angle = 0;
float rate = 0;
float integrale = 0;
float sortie = 0;

bool armed = false;
bool actif = false;

uint32_t tPrec = 0;
uint32_t tTelem = 0;

uint32_t tDebutActif = 0;
float tenue = 0;
float tenueMax = 0;

int derniereChute = 0;
float dureeChute = 0;

int testPwm = 0;
uint32_t testFin = 0;

uint32_t dtMaxUs = 0;
uint32_t dtDernierUs = 0;

char serialBuf[64];
uint8_t serialPos = 0;

// mpu6050
void mpuEcrire(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuLire(float &accAngle, float &gyroRate) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0)
    return false;

  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14)
    return false;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  (void)ax;
  (void)gy;
  (void)gz;

// imu montée "debout"
  accAngle = atan2((float)az, (float)ay) * RAD_TO_DEG;

// ±500 °/s -> 65.5 lsb/(°/s)
  gyroRate = gyroSign * gx / 65.5;

  return true;
}

void mpuInit() {
  mpuEcrire(0x6B, 0x00);
  mpuEcrire(0x1A, 0x03);
  mpuEcrire(0x1B, 0x08);
  mpuEcrire(0x1C, 0x00);
}

void calibrationGyro() {
  armed = false;
  actif = false;
  commandeMoteurs(0);

  Serial.println("INFO,Calibration gyro : NE PAS BOUGER");

  float somme = 0;
  int n = 0;
  float a, g;

  for (int i = 0; i < 500; i++) {
    if (mpuLire(a, g)) {
      somme += g;
      n++;
    }
    delay(3);
  }

  gyroBias = (n > 0) ? somme / n : 0;

  Serial.print("INFO,GyroBias,");
  Serial.println(gyroBias, 5);
}

// moteurs
void moteur(int pwmPin, int in1, int in2, int vitesse) {
  vitesse = constrain(vitesse, -PWM_MAX, PWM_MAX);

  if (vitesse > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (vitesse < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  ledcWrite(pwmPin, abs(vitesse));
}

void commandeMoteurs(int v) {
  moteur(PWMA, AIN1, AIN2, SIGN_MOTEUR_A * v);
  moteur(PWMB, BIN1, BIN2, SIGN_MOTEUR_B * v);
}

void razEssai() {
  integrale = 0;
  tenueMax = 0;
  derniereChute = 0;
  dureeChute = 0;
}

void appliquerParam(char c, float v) {
  switch (c) {
    case 'p':
      Kp = max(v, 0.0f);
      razEssai();
      break;

    case 'i':
      Ki = max(v, 0.0f);
      razEssai();
      break;

    case 'd':
      Kd = max(v, 0.0f);
      razEssai();
      break;

    case 's':
      consigne = v;
      razEssai();
      break;

    case 'm':
      pwmMin = max((int)v, 0);
      razEssai();
      break;

    case 'r':
      Kp = 15.0;
      Ki = 0.0;
      Kd = 1.0;
      razEssai();
      break;

    case 'g':
      gyroSign = -gyroSign;
      gyroBias = -gyroBias;
      razEssai();
      break;

    case 'v':
      if (!armed) {
        testPwm = constrain((int)v, -PWM_MAX, PWM_MAX);
        testFin = millis() + 800;
      }
      break;

    case 'z':
      calibrationGyro();
      razEssai();
      break;

    case 'a':
      armed = (v > 0.5);
      integrale = 0;

      if (!armed) {
        actif = false;
        commandeMoteurs(0);
      }
      break;

    case 'q':
      armed = false;
      actif = false;
      integrale = 0;
      commandeMoteurs(0);
      break;

    case '?':
      break;
  }
}

void afficheParametres() {
  Serial.printf(
    "PARAM,Kp=%.3f,Ki=%.3f,Kd=%.3f,consigne=%.3f,pwmMin=%d,gyroSign=%.0f,armed=%d\n",
    Kp, Ki, Kd, consigne, pwmMin, gyroSign, armed ? 1 : 0
  );
}

// lecture série non bloquante.
void lireSerie() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r')
      continue;

    if (c == '\n') {
      serialBuf[serialPos] = '\0';

      if (serialPos > 0) {
        char commande = serialBuf[0];

        if (commande == '?' || commande == 'z' ||
            commande == 'g' || commande == 'r' ||
            commande == 'q') {
          appliquerParam(commande, 0);
        }
        else {
          float valeur = atof(&serialBuf[1]);
          appliquerParam(commande, valeur);
        }

        afficheParametres();
      }

      serialPos = 0;
    }
    else if (serialPos < sizeof(serialBuf) - 1) {
      serialBuf[serialPos++] = c;
    }
    else {
// ligne trop longue : on abandonne la ligne.
      serialPos = 0;
    }
  }
}

void envoyerTelemetrie() {
  Serial.printf(
    "DATA,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%d,%d,%.3f,%.3f,%.3f\n",
    millis(),
    angle,
    angleAcc,
    rate,
    rateF,
    angle - consigne,
    sortie,
    (int)round(sortie),
    dtDernierUs / 1000.0,
    actif ? 1 : 0,
    armed ? 1 : 0,
    Kp,
    Ki,
    Kd
  );
}

void setup() {
  Serial.begin(921600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);

  commandeMoteurs(0);
  digitalWrite(STBY, HIGH);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  mpuInit();
  delay(100);

  calibrationGyro();

  float a, g;
  if (mpuLire(a, g))
    angle = a;

  tPrec = micros();
  tTelem = tPrec;

  Serial.println("INFO,READY");
  afficheParametres();
}

void loop() {
// lecture série non bloquante.
// aucun wi-fi, aucun serveur http.
  lireSerie();

  uint32_t now = micros();

  if ((uint32_t)(now - tPrec) < PERIODE_US)
    return;

  uint32_t dus = now - tPrec;

// planification à période fixe pour limiter la dérive.
// si la boucle a pris trop de retard, on recale.
  tPrec += PERIODE_US;

  if ((uint32_t)(now - tPrec) > PERIODE_US * 4UL)
    tPrec = now;

  dtDernierUs = dus;

  if (dus > dtMaxUs)
    dtMaxUs = dus;

  float dt = dus / 1000000.0f;

  float accAngle, gyroRate;

  if (!mpuLire(accAngle, gyroRate)) {
    commandeMoteurs(0);
    armed = false;
    actif = false;
    return;
  }

  rate = gyroRate - gyroBias;
  angleAcc = accAngle;

// filtre gyro uniquement pour le terme d
  rateF += ALPHA_D * (rate - rateF);

// filtre complémentaire
  angle = ALPHA * (angle + rate * dt)
        + (1.0 - ALPHA) * accAngle;

  float erreur = angle - consigne;

// test roues : uniquement désarmé
  if (!armed && (int32_t)(testFin - millis()) > 0) {
    commandeMoteurs(testPwm);
    actif = false;
  }
  else {
// mémorisation d'une chute réelle
    if (actif && armed && fabs(angle) > ANGLE_CHUTE) {
      derniereChute = (angle > 0) ? 1 : -1;
      dureeChute = tenue;
    }

// sécurité
    if (!armed || fabs(angle) > ANGLE_CHUTE) {
      actif = false;
    }
    else if (!actif && fabs(erreur) < ANGLE_REPRISE) {
      actif = true;
      integrale = 0;
      tDebutActif = millis();
    }

    if (!actif) {
      commandeMoteurs(0);
      integrale = 0;
      sortie = 0;
      tenue = 0;
    }
    else {
      tenue = (millis() - tDebutActif) / 1000.0f;

      if (tenue > tenueMax)
        tenueMax = tenue;

// i
      integrale += Ki * erreur * dt;
      integrale = constrain(integrale, -I_MAX, I_MAX);

// important :
// le d s'oppose à la vitesse de chute.
// avec erreur = angle - consigne et gyro positif quand
// l'angle augmente, le terme d doit donc être -kd * ratef.
      sortie = Kp * erreur + integrale - Kd * rateF;

      sortie = constrain(sortie, -PWM_MAX, PWM_MAX);

// compensation progressive de la zone morte moteur.
      float absS = fabs(sortie);
      float offset = pwmMin * min(absS / ZONE_RAMPE, 1.0f);

      int cmd = (int)constrain(
        absS + offset,
        0.0f,
        (float)PWM_MAX
      );

      if (sortie < 0)
        cmd = -cmd;

      commandeMoteurs(cmd);
    }
  }

// télémétrie 100 hz.
// 100 hz est largement suffisant pour analyser le comportement
// d'un pid tournant à 200 hz, tout en gardant une marge usb.
  if ((uint32_t)(now - tTelem) >= TELEMETRIE_US) {
    tTelem += TELEMETRIE_US;
    envoyerTelemetrie();
  }
}
