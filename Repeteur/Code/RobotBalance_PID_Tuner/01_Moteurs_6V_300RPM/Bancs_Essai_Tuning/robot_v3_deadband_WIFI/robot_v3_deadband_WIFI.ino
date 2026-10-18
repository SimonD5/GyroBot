// ============================================================
// RobotBalance - Version 3 : Zone Morte + WiFi Persistant + 45s
// ESP32-C6 + MPU6050 GY-521 + TB6612FNG
//
// NOUVEAUTÉS V3 :
//   1. WiFi PERSISTANT : Le point d'accès SoftAP "RobotPID" reste allumé en continu.
//      Le PC ne se déconnecte PLUS JAMAIS du réseau WiFi pendant l'essai.
//      Aucune donnée n'est transmise pendant la régulation (zéro impact sur la boucle).
//      Dès la fin de l'essai, les données arrivent instantanément au PC.
//   2. ZONE MORTE ANTI-TREMBLEMENT : Si le PWM moyen est faible (|spd| <= 6),
//      la boucle 2 ne touche pas à la consigne. Elle n'intervient que si le robot
//      dérive réellement. Cela élimine le tremblement vers 10-15 secondes.
//   3. DURÉE 45 SECONDES : Tampon de 9000 échantillons à 200 Hz.
//
// RÉSEAU : SoftAP "RobotPID", mdp "robot1234", IP 192.168.4.1:3333
// ============================================================

#include <Wire.h>
#include <WiFi.h>

// ---------------- Réseau ----------------
const char*    AP_SSID    = "RobotPID";
const char*    AP_PASS    = "robot1234";
const uint8_t  AP_CHANNEL = 1;
const uint16_t TCP_PORT   = 3333;

// ---------------- Broches ----------------
#define SDA_PIN 2
#define SCL_PIN 3

#define PWMA 11
#define AIN1 7
#define AIN2 10
#define PWMB 23
#define BIN1 21
#define BIN2 20
#define STBY 22

// ---------------- Réglages matériels ----------------
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int PWM_MAX  = 200;

const int SIGN_MOTEUR_A = 1;
const int SIGN_MOTEUR_B = -1;

// ---------------- PID Boucle 1 (Angle) ----------------
float Kp = 15.0;
float Ki = 1.0;
float Kd = 0.30;
float consigne = 2.5;
int   pwmMin = 11;

// ---------------- Boucle 2 (Cascade avec Zone Morte) ----------------
float Kp2 = 0.05;    // Gain vitesse estimée
float Ki2 = 0.00;    // Gardé à 0 pour éviter le retard de phase
const float ALPHA_V = 0.03;          // Filtre passe-bas vitesse estimée (~200 ms)
float V_DEADBAND = 2.0;              // Zone morte anti-tremblement (reglable via 'b')
const float ANGLE_CORR_MAX = 3.0;    // Butée de sécurité : max +/- 3°

float vitesseEstimee = 0;
float correctionAngle = 0;
float consigneActive = 2.5;

// ---------------- Sécurité ----------------
const float ANGLE_CHUTE   = 18.0;
const float ANGLE_REPRISE = 2.0;    // Démarre uniquement si |angle - consigne| < 2.0°
const float I_MAX         = 80.0;
const float ZONE_RAMPE    = 30.0;

const uint32_t ARME_TIMEOUT_MS = 25000;
const float ALPHA_D = 0.4;
const float ALPHA = 0.98;
const uint32_t PERIODE_US = 5000;   // 200 Hz (5 ms)

// ---------------- Tampon d'enregistrement (45 secondes) ----------------
const uint32_t MAX_ECHANTILLONS = 9000;   // 45 s à 200 Hz
const size_t   RESERVE_WIFI     = 80000;

struct Echantillon {
  uint32_t t_ms;
  int16_t  angle;     // 0.01 °
  int16_t  acc;       // 0.01 °
  int16_t  gyro;      // 0.1 °/s
  int16_t  rateF;     // 0.1 °/s
  int16_t  pid;       // 0.1
  int16_t  pwm;       // commande moteurs
  int16_t  target;    // 0.01 ° consigne active
  int16_t  spd;       // 0.1 vitesse estimée
  uint16_t dt_us;     // durée tour
};

Echantillon* tampon = nullptr;
uint32_t capacite = 0;
uint32_t nEch = 0;

// ---------------- MPU6050 ----------------
const uint8_t MPU_ADDR = 0x68;
float gyroBias = 0;
float gyroSign = -1.0;

float rateF = 0;
float angleAcc = 0;
float angle = 0;
float rate = 0;
float integrale = 0;
float sortie = 0;

// ---------------- Machine d'états ----------------
enum Etat : uint8_t { ETAT_COMMS, ETAT_ARME, ETAT_ACTIF };
Etat etat = ETAT_COMMS;

uint32_t tPrec = 0;
uint32_t tDernier = 0;
uint32_t tArme = 0;
uint32_t tDebutActif = 0;

uint32_t essaiNum = 0;
bool trialPret = false;
const char* trialRaison = "";
float essaiKp = 0, essaiKi = 0, essaiKd = 0, essaiConsigne = 0;
int essaiPwmMin = 0;
float essaiKp2 = 0, essaiKi2 = 0;

const char* infoConnexion = "";

int testPwm = 0;
uint32_t testFin = 0;

// ---------------- WiFi / TCP ----------------
WiFiServer server(TCP_PORT);
WiFiClient client;
bool wifiActif = false;

char cmdBuf[64];
uint8_t cmdPos = 0;

void commandeMoteurs(int v);
void afficheParametres();
void calibrationGyro();
void wifiOn();
void envoyerEssai();

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

  accAngle = atan2((float)az, (float)ay) * RAD_TO_DEG;
  gyroRate = gyroSign * gx / 65.5;

  return true;
}

void mpuInit() {
  mpuEcrire(0x6B, 0x00);
  mpuEcrire(0x1A, 0x03);
  mpuEcrire(0x1B, 0x08);
  mpuEcrire(0x1C, 0x00);
}

void info(const char* texte) {
  Serial.print("INFO,");
  Serial.println(texte);
  if (client && client.connected()) {
    client.print("INFO,");
    client.println(texte);
  }
}

void calibrationGyro() {
  commandeMoteurs(0);
  info("Calibration gyro : NE PAS BOUGER");

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
  char msg[48];
  snprintf(msg, sizeof(msg), "GyroBias,%.5f", gyroBias);
  info(msg);
}

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

// Démarre le SoftAP (reste actif en permanence)
void wifiOn() {
  if (wifiActif)
    return;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 4); // autorise jusqu'à 4 clients
  server.begin();
  wifiActif = true;
  cmdPos = 0;
}

static inline int16_t q16(float v, float echelle) {
  float x = v * echelle;
  if (x > 32767.0f)  x = 32767.0f;
  if (x < -32768.0f) x = -32768.0f;
  return (int16_t)lroundf(x);
}

static inline void enregistrer(float pwmApplique, float pidVal, uint32_t dus, float targetVal, float spdVal) {
  if (nEch >= capacite || tampon == nullptr)
    return;

  Echantillon &e = tampon[nEch++];
  e.t_ms   = millis();
  e.angle  = q16(angle, 100.0f);
  e.acc    = q16(angleAcc, 100.0f);
  e.gyro   = q16(rate, 10.0f);
  e.rateF  = q16(rateF, 10.0f);
  e.pid    = q16(pidVal, 10.0f);
  e.pwm    = q16(pwmApplique, 1.0f);
  e.target = q16(targetVal, 100.0f);
  e.spd    = q16(spdVal, 10.0f);
  e.dt_us  = (dus > 65535UL) ? 65535 : (uint16_t)dus;
}

// Arme sans couper le WiFi : le PC reste connecté !
void armerSansCouperWifi() {
  integrale = 0;
  vitesseEstimee = 0;
  correctionAngle = 0;
  consigneActive = consigne;
  nEch = 0;
  testFin = 0;
  commandeMoteurs(0);

  if (client && client.connected()) {
    client.println("ARMING");
    // Pas de flush() bloquant ici : le TCP stack envoie asap.
  }

  // Le SoftAP reste actif, mais le mode devient ARME
  // Aucune communication TCP n'aura lieu pendant l'essai.
  etat = ETAT_ARME;
  tArme = millis();
  tPrec = micros();
  tDernier = tPrec;
}

void demarrerEssai() {
  essaiNum++;
  nEch = 0;
  integrale = 0;
  vitesseEstimee = 0;
  correctionAngle = 0;
  consigneActive = consigne;

  essaiKp = Kp;
  essaiKi = Ki;
  essaiKd = Kd;
  essaiConsigne = consigne;
  essaiPwmMin = pwmMin;
  essaiKp2 = Kp2;
  essaiKi2 = Ki2;

  tDebutActif = millis();
  etat = ETAT_ACTIF;
}

void finirEssai(const char* raison) {
  commandeMoteurs(0);
  sortie = 0;
  integrale = 0;
  vitesseEstimee = 0;

  etat = ETAT_COMMS;
  trialPret = true;
  trialRaison = raison;

  tPrec = micros();
  tDernier = tPrec;

  // Si le client est resté connecté, on envoie immédiatement l'essai !
  if (client && client.connected()) {
    envoyerEssai();
  }
}

void retourComms(const char* message) {
  commandeMoteurs(0);
  integrale = 0;
  etat = ETAT_COMMS;
  infoConnexion = message;

  tPrec = micros();
  tDernier = tPrec;

  if (client && client.connected()) {
    client.print("INFO,");
    client.println(message);
    afficheParametres();
  }
}

void appliquerParam(char c, float v) {
  switch (c) {
    case 'p':
      Kp = max(v, 0.0f);
      break;

    case 'i':
      Ki = max(v, 0.0f);
      break;

    case 'd':
      Kd = max(v, 0.0f);
      break;

    case 's':
      consigne = v;
      consigneActive = v;
      break;

    case 'm':
      pwmMin = max((int)v, 0);
      break;

    case 'k':
      Kp2 = max(v, 0.0f);
      break;

    case 'b':
      V_DEADBAND = max(v, 0.0f);   // Deadband reglable depuis le PC
      break;

    case 'j':
      Ki2 = max(v, 0.0f);
      break;

    case 'r':
      Kp = 15.0;
      Ki = 1.0;
      Kd = 0.30;
      consigne = 2.5;
      pwmMin = 11;
      Kp2 = 0.05;
      Ki2 = 0.00;
      V_DEADBAND = 2.0;
      break;

    case 'g':
      gyroSign = -gyroSign;
      gyroBias = -gyroBias;
      break;

    case 'v':
      testPwm = constrain((int)v, -PWM_MAX, PWM_MAX);
      testFin = millis() + 800;
      break;

    case 'z':
      calibrationGyro();
      break;

    case 'a':
      if (v > 0.5f) {
        // N'arme que si on est en COMMS (ignore si déjà armé / actif)
        if (etat == ETAT_COMMS) {
          if (trialPret) {
            info("Essai precedent non transmis : armement refuse");
          } else {
            armerSansCouperWifi();
          }
        }
      } else {
        // Désarmement : toujours accepté
        testFin = 0;
        commandeMoteurs(0);
        etat = ETAT_COMMS;
      }
      break;

    case 'q':
      testFin = 0;
      commandeMoteurs(0);
      etat = ETAT_COMMS;
      break;

    case '?':
      break;
  }
}

void afficheParametres() {
  if (!client || !client.connected())
    return;

  client.printf(
    "PARAM,Kp=%.3f,Ki=%.3f,Kd=%.3f,consigne=%.3f,pwmMin=%d,Kp2=%.4f,Ki2=%.5f,dead=%.2f,gyroSign=%.0f,armed=%d,pending=%d,cap=%lu\n",
    Kp, Ki, Kd, consigne, pwmMin, Kp2, Ki2, V_DEADBAND, gyroSign,
    (etat != ETAT_COMMS) ? 1 : 0,
    trialPret ? 1 : 0,
    (unsigned long)capacite
  );
}

void traiterCommande(const char* ligne) {
  char commande = ligne[0];
  if (commande == '?' || commande == 'z' ||
      commande == 'g' || commande == 'r' ||
      commande == 'q') {
    appliquerParam(commande, 0);
  } else {
    appliquerParam(commande, atof(ligne + 1));
  }
}

bool envoyerBloc(const char* data, size_t len) {
  if (!client || !client.connected())
    return false;
  size_t ecrit = client.write((const uint8_t*)data, len);
  return ecrit == len;
}

void envoyerEssai() {
  char ligne[140];

  int n = snprintf(
    ligne, sizeof(ligne),
    "TRIAL,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%d,%.4f,%.5f,%s\n",
    (unsigned long)essaiNum, (unsigned long)nEch,
    essaiKp, essaiKi, essaiKd, essaiConsigne, essaiPwmMin,
    essaiKp2, essaiKi2,
    trialRaison
  );

  if (!envoyerBloc(ligne, n))
    return;

  char out[1400];
  size_t pos = 0;

  for (uint32_t i = 0; i < nEch; i++) {
    const Echantillon &e = tampon[i];

    int k = snprintf(
      out + pos, sizeof(out) - pos,
      "D,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%u\n",
      (unsigned long)e.t_ms, e.angle, e.acc, e.gyro,
      e.rateF, e.pid, e.pwm, e.target, e.spd, (unsigned)e.dt_us
    );

    if (k <= 0)
      continue;

    pos += k;

    if (pos > sizeof(out) - 90) {
      if (!envoyerBloc(out, pos))
        return;
      pos = 0;
    }
  }

  if (pos > 0 && !envoyerBloc(out, pos))
    return;

  n = snprintf(ligne, sizeof(ligne), "END,%lu\n", (unsigned long)essaiNum);
  if (!envoyerBloc(ligne, n))
    return;

  trialPret = false;
}

void clientConnecte() {
  client.println("INFO,READY_V3");
  if (infoConnexion[0] != '\0') {
    client.print("INFO,");
    client.println(infoConnexion);
    infoConnexion = "";
  }
  afficheParametres();
  if (trialPret)
    envoyerEssai();
}

void lireClient() {
  while (client.connected() && client.available() > 0) {
    char c = (char)client.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdPos] = '\0';
      if (cmdPos > 0) {
        traiterCommande(cmdBuf);
        afficheParametres();
      }
      cmdPos = 0;
    }
    else if (cmdPos < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdPos++] = c;
    } else {
      cmdPos = 0;
    }
  }
}

void gererWifi() {
  if (!client || !client.connected()) {
    client.stop();
    WiFiClient nouveau = server.accept();
    if (nouveau) {
      client = nouveau;
      client.setNoDelay(true);
      cmdPos = 0;
      clientConnecte();
    }
    return;
  }
  lireClient();
}

void setup() {
  Serial.begin(115200);

  size_t libre = ESP.getFreeHeap();
  size_t dispo = (libre > RESERVE_WIFI) ? (libre - RESERVE_WIFI) : 0;
  size_t maxEch = dispo / sizeof(Echantillon);

  capacite = (maxEch < MAX_ECHANTILLONS) ? (uint32_t)maxEch : MAX_ECHANTILLONS;
  tampon = (Echantillon*)malloc(capacite * sizeof(Echantillon));
  if (tampon == nullptr)
    capacite = 0;

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

  char msg[64];
  snprintf(msg, sizeof(msg), "Tampon V3 : %lu echantillons (%.1f s a 200 Hz)",
           (unsigned long)capacite, capacite * PERIODE_US / 1e6);
  info(msg);

  wifiOn();

  tPrec = micros();
  tDernier = tPrec;
  info("READY_V3");
}

void loop() {
  // WiFi géré en COMMS et ARME (pour garder le socket TCP vivant).
  // Pendant ETAT_ACTIF (régulation active) : zéro appel réseau.
  if (etat != ETAT_ACTIF)
    gererWifi();

  uint32_t now = micros();
  if ((uint32_t)(now - tPrec) < PERIODE_US)
    return;

  uint32_t dus = now - tDernier;
  tDernier = now;

  tPrec += PERIODE_US;
  if ((uint32_t)(now - tPrec) > PERIODE_US * 4UL)
    tPrec = now;

  float dt = ((dus > 20000UL) ? 20000UL : dus) / 1000000.0f;

  float accAngle, gyroRate;
  if (!mpuLire(accAngle, gyroRate)) {
    commandeMoteurs(0);
    if (etat == ETAT_ACTIF)
      finirEssai("IMU");
    else if (etat == ETAT_ARME)
      retourComms("IMU");
    return;
  }

  rate = gyroRate - gyroBias;
  angleAcc = accAngle;

  rateF += ALPHA_D * (rate - rateF);
  angle = ALPHA * (angle + rate * dt) + (1.0 - ALPHA) * accAngle;

  float erreur = angle - consigneActive;

  switch (etat) {
    case ETAT_COMMS:
      if ((int32_t)(testFin - millis()) > 0)
        commandeMoteurs(testPwm);
      else
        commandeMoteurs(0);
      break;

    case ETAT_ARME:
      commandeMoteurs(0);
      consigneActive = consigne;
      vitesseEstimee = 0;

      // Déclenchement propre lorsque proche de la consigne
      if (fabs(angle) < ANGLE_CHUTE && fabs(angle - consigne) < ANGLE_REPRISE)
        demarrerEssai();
      else if (millis() - tArme > ARME_TIMEOUT_MS)
        retourComms("ARME_TIMEOUT");
      break;

    case ETAT_ACTIF: {
      if (fabs(angle) > ANGLE_CHUTE) {
        enregistrer(0, 0, dus, consigneActive, vitesseEstimee);
        finirEssai("FALL");
        break;
      }

      // --- BOUCLE EXTERNE : Vitesse estimée avec Zone Morte ---
      vitesseEstimee += ALPHA_V * (sortie - vitesseEstimee);

      // Zone morte anti-tremblement : si le PWM moyen est faible, pas de correction.
      float vActive = 0.0f;
      if (fabs(vitesseEstimee) > V_DEADBAND) {
        if (vitesseEstimee > 0)
          vActive = vitesseEstimee - V_DEADBAND;
        else
          vActive = vitesseEstimee + V_DEADBAND;
      }

      // Correction proportionnelle douce uniquement hors zone morte
      correctionAngle = constrain(Kp2 * vActive, -ANGLE_CORR_MAX, ANGLE_CORR_MAX);
      consigneActive = consigne - correctionAngle;

      erreur = angle - consigneActive;

      // --- BOUCLE INTERNE : Angle PID ---
      integrale += Ki * erreur * dt;
      integrale = constrain(integrale, -I_MAX, I_MAX);

      sortie = Kp * erreur + integrale + Kd * rateF;
      sortie = constrain(sortie, -PWM_MAX, PWM_MAX);

      float absS = fabs(sortie);
      float offset = pwmMin * min(absS / ZONE_RAMPE, 1.0f);

      int cmd = (int)constrain(absS + offset, 0.0f, (float)PWM_MAX);
      if (sortie < 0) cmd = -cmd;

      commandeMoteurs(cmd);
      enregistrer(cmd, sortie, dus, consigneActive, vitesseEstimee);

      if (nEch >= capacite)
        finirEssai("MAXTIME");
      break;
    }
  }
}
