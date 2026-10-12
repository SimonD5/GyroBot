// ============================================================
// RobotBalance - réglage PID par WiFi "hors boucle"
// ESP32-C6 + MPU6050 GY-521 + TB6612FNG
//
// PRINCIPE
//   Le WiFi n'est JAMAIS actif pendant que le robot s'équilibre :
//   la boucle PID tourne à 200 Hz sans aucune perturbation radio.
//
//   1. COMMS  : WiFi allumé (le robot est tombé / vient de démarrer).
//               Le PC se connecte, reçoit l'essai précédent, règle les
//               gains, puis envoie ARMER.
//   2. ARMÉ   : WiFi COUPÉ. Le robot attend d'être redressé
//               (|erreur| < ANGLE_REPRISE). Timeout 20 s -> retour COMMS.
//   3. ACTIF  : le PID tourne, la télémétrie est stockée en RAM.
//               Fin = chute, ou tampon plein (durée max), ou erreur IMU.
//   4. retour à COMMS : WiFi rallumé, l'essai est envoyé au PC.
//
// RÉSEAU
//   Le robot crée son propre point d'accès (SoftAP) :
//     SSID : RobotPID     mot de passe : robot1234
//     IP du robot : 192.168.4.1     port TCP : 3333
//
// COMMANDES (une par ligne, via TCP, uniquement en état COMMS)
//   p15  i0  d1  s7.5  m20   -> Kp, Ki, Kd, consigne, PWM min
//   a1                       -> armer (coupe le WiFi, attend le robot droit)
//   a0                       -> désarmer (déjà le cas en COMMS)
//   z                        -> recalibrer le gyro (robot IMMOBILE)
//   g                        -> inverser le sens du gyro
//   r                        -> gains de départ
//   v40                      -> test roues PWM 40 pendant 0,8 s
//   q                        -> arrêt moteurs
//   ?                        -> afficher les paramètres
//
// MESSAGES ROBOT -> PC
//   INFO,texte
//   PARAM,Kp=..,Ki=..,Kd=..,consigne=..,pwmMin=..,gyroSign=..,armed=..,pending=..,cap=..
//   ARMING
//   TRIAL,num,nbEchantillons,Kp,Ki,Kd,consigne,pwmMin,raison
//   D,t_ms,angle*100,acc*100,gyro*10,rateF*10,pid*10,pwm,dt_us
//   END,num
//   raison = FALL (chute) | MAXTIME (tampon plein) | IMU (erreur capteur)
// ============================================================

#include <Wire.h>
#include <WiFi.h>

// ---------------- Réseau ----------------
const char*    AP_SSID    = "RobotPID";
const char*    AP_PASS    = "robot1234";   // 8 caractères minimum
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

// Moteur B câblé à l'envers
const int SIGN_MOTEUR_A = 1;
const int SIGN_MOTEUR_B = -1;

// ---------------- PID ----------------
float Kp = 14.5;
float Ki = 1.0;
float Kd = 0.25;
float consigne = 3.1;
int   pwmMin = 11;

// ---------------- Sécurité ----------------
const float ANGLE_CHUTE   = 18.0;
const float ANGLE_REPRISE = 2.0;  // Démarre l'essai uniquement si très proche de la consigne (|angle - consigne| < 2.0°)
const float I_MAX         = 80.0;
const float ZONE_RAMPE    = 30.0;

// Armé mais jamais redressé : retour en COMMS après ce délai
const uint32_t ARME_TIMEOUT_MS = 20000;

// Filtre gyro pour le D
const float ALPHA_D = 0.4;

// Filtre complémentaire
const float ALPHA = 0.98;

// 200 Hz = 5 ms
const uint32_t PERIODE_US = 5000;

// ---------------- Tampon d'enregistrement ----------------
// 1 échantillon par tour de boucle (200 Hz).
// 6000 échantillons = 30 s d'essai. La taille réelle est réduite
// automatiquement si la RAM libre est insuffisante.
const uint32_t MAX_ECHANTILLONS = 6000;
const size_t   RESERVE_WIFI     = 90000;  // RAM laissée libre pour la pile WiFi

struct Echantillon {
  uint32_t t_ms;
  int16_t  angle;   // 0.01 °
  int16_t  acc;     // 0.01 °
  int16_t  gyro;    // 0.1 °/s
  int16_t  rateF;   // 0.1 °/s
  int16_t  pid;     // 0.1
  int16_t  pwm;     // commande réellement appliquée aux moteurs
  uint16_t dt_us;   // durée réelle depuis le tour précédent
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

uint32_t tPrec = 0;      // échéance planifiée (période fixe)
uint32_t tDernier = 0;   // instant réel du dernier tour
uint32_t tArme = 0;
uint32_t tDebutActif = 0;

// Essai en cours / dernier essai
uint32_t essaiNum = 0;
bool trialPret = false;            // essai terminé, pas encore transmis
const char* trialRaison = "";
float essaiKp = 0, essaiKi = 0, essaiKd = 0, essaiConsigne = 0;
int essaiPwmMin = 0;

const char* infoConnexion = "";    // message envoyé à la prochaine connexion

int testPwm = 0;
uint32_t testFin = 0;

// ---------------- WiFi / TCP ----------------
WiFiServer server(TCP_PORT);
WiFiClient client;
bool wifiActif = false;

char cmdBuf[64];
uint8_t cmdPos = 0;

// Déclarations anticipées
void commandeMoteurs(int v);
void afficheParametres();
void calibrationGyro();
void wifiOn();
void wifiOff();

// ---------------- MPU6050 ----------------
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
  Wire.read(); // température ignorée

  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  (void)ax;
  (void)gy;
  (void)gz;

  // IMU montée "debout"
  accAngle = atan2((float)az, (float)ay) * RAD_TO_DEG;

  // ±500 °/s -> 65.5 LSB/(°/s)
  gyroRate = gyroSign * gx / 65.5;

  return true;
}

void mpuInit() {
  mpuEcrire(0x6B, 0x00);  // sortie de veille
  mpuEcrire(0x1A, 0x03);  // DLPF ~44 Hz
  mpuEcrire(0x1B, 0x08);  // gyro ±500 °/s
  mpuEcrire(0x1C, 0x00);  // accéléro ±2 g
}

// Envoie un message INFO sur l'USB (debug) et au PC s'il est connecté.
void info(const char* texte) {
  Serial.print("INFO,");
  Serial.println(texte);
  if (wifiActif && client.connected()) {
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

// ---------------- Moteurs ----------------
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

// ---------------- WiFi ----------------
void wifiOn() {
  if (wifiActif)
    return;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 1);
  server.begin();
  wifiActif = true;
  cmdPos = 0;
}

void wifiOff() {
  if (!wifiActif)
    return;

  client.stop();
  server.end();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiActif = false;
}

// ---------------- Enregistrement ----------------
static inline int16_t q16(float v, float echelle) {
  float x = v * echelle;
  if (x > 32767.0f)  x = 32767.0f;
  if (x < -32768.0f) x = -32768.0f;
  return (int16_t)lroundf(x);
}

static inline void enregistrer(float pwmApplique, float pidVal, uint32_t dus) {
  if (nEch >= capacite || tampon == nullptr)
    return;

  Echantillon &e = tampon[nEch++];
  e.t_ms  = millis();
  e.angle = q16(angle, 100.0f);
  e.acc   = q16(angleAcc, 100.0f);
  e.gyro  = q16(rate, 10.0f);
  e.rateF = q16(rateF, 10.0f);
  e.pid   = q16(pidVal, 10.0f);
  e.pwm   = q16(pwmApplique, 1.0f);
  e.dt_us = (dus > 65535UL) ? 65535 : (uint16_t)dus;
}

// ---------------- Changements d'état ----------------
void armerEtCouperWifi() {
  integrale = 0;
  nEch = 0;
  testFin = 0;
  commandeMoteurs(0);

  // Laisse le temps à l'accusé de partir et à la connexion de se fermer
  // proprement avant de couper la radio.
  client.println("ARMING");
  delay(100);
  client.stop();
  delay(100);

  wifiOff();

  etat = ETAT_ARME;
  tArme = millis();

  tPrec = micros();
  tDernier = tPrec;
}

void demarrerEssai() {
  essaiNum++;
  nEch = 0;
  integrale = 0;

  // Gains figés pour toute la durée de l'essai
  essaiKp = Kp;
  essaiKi = Ki;
  essaiKd = Kd;
  essaiConsigne = consigne;
  essaiPwmMin = pwmMin;

  tDebutActif = millis();
  etat = ETAT_ACTIF;
}

// Fin d'essai : moteurs coupés, WiFi rallumé, essai prêt à être envoyé.
void finirEssai(const char* raison) {
  commandeMoteurs(0);
  sortie = 0;
  integrale = 0;

  etat = ETAT_COMMS;
  trialPret = true;
  trialRaison = raison;

  wifiOn();  // bloquant quelques centaines de ms : sans importance, moteurs coupés

  tPrec = micros();
  tDernier = tPrec;
}

// Armé mais jamais redressé : on rend la main au PC.
void retourComms(const char* message) {
  commandeMoteurs(0);
  integrale = 0;
  etat = ETAT_COMMS;
  infoConnexion = message;

  wifiOn();

  tPrec = micros();
  tDernier = tPrec;
}

// ---------------- Paramètres ----------------
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
      break;

    case 'm':
      pwmMin = max((int)v, 0);
      break;

    case 'r':
      Kp = 14.5;
      Ki = 1.0;
      Kd = 0.25;
      consigne = 3.1;
      pwmMin = 11;
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
        if (trialPret) {
          info("Essai precedent non transmis : armement refuse");
        } else {
          armerEtCouperWifi();
        }
      } else {
        testFin = 0;
        commandeMoteurs(0);
      }
      break;

    case 'q':
      testFin = 0;
      commandeMoteurs(0);
      break;

    case '?':
      break;
  }
}

void afficheParametres() {
  if (!wifiActif || !client.connected())
    return;

  client.printf(
    "PARAM,Kp=%.3f,Ki=%.3f,Kd=%.3f,consigne=%.3f,pwmMin=%d,gyroSign=%.0f,armed=%d,pending=%d,cap=%lu\n",
    Kp, Ki, Kd, consigne, pwmMin, gyroSign,
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
  }
  else {
    appliquerParam(commande, atof(ligne + 1));
  }
}

// ---------------- Envoi d'un essai ----------------
bool envoyerBloc(const char* data, size_t len) {
  if (!client.connected())
    return false;

  size_t ecrit = client.write((const uint8_t*)data, len);
  return ecrit == len;
}

void envoyerEssai() {
  char ligne[112];

  int n = snprintf(
    ligne, sizeof(ligne),
    "TRIAL,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%d,%s\n",
    (unsigned long)essaiNum, (unsigned long)nEch,
    essaiKp, essaiKi, essaiKd, essaiConsigne, essaiPwmMin,
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
      "D,%lu,%d,%d,%d,%d,%d,%d,%u\n",
      (unsigned long)e.t_ms, e.angle, e.acc, e.gyro,
      e.rateF, e.pid, e.pwm, (unsigned)e.dt_us
    );

    if (k <= 0)
      continue;

    pos += k;

    // Une ligne fait moins de 80 octets : on vide avant de déborder.
    if (pos > sizeof(out) - 80) {
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

// ---------------- Gestion du client TCP ----------------
void clientConnecte() {
  client.println("INFO,READY");

  if (infoConnexion[0] != '\0') {
    client.print("INFO,");
    client.println(infoConnexion);
    infoConnexion = "";
  }

  afficheParametres();

  if (trialPret)
    envoyerEssai();
}

// Lecture non bloquante des commandes du PC.
void lireClient() {
  while (client.connected() && client.available() > 0) {
    char c = (char)client.read();

    if (c == '\r')
      continue;

    if (c == '\n') {
      cmdBuf[cmdPos] = '\0';

      if (cmdPos > 0) {
        traiterCommande(cmdBuf);

        // Un armement coupe le WiFi : on s'arrête là.
        if (!wifiActif) {
          cmdPos = 0;
          return;
        }

        afficheParametres();
      }

      cmdPos = 0;
    }
    else if (cmdPos < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdPos++] = c;
    }
    else {
      // Ligne trop longue : on abandonne la ligne.
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

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);  // debug uniquement

  // Tampon d'enregistrement : à allouer AVANT de démarrer le WiFi.
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

  // Le robot doit être immobile ici (le WiFi n'est pas encore démarré).
  calibrationGyro();

  float a, g;
  if (mpuLire(a, g))
    angle = a;

  char msg[64];
  snprintf(msg, sizeof(msg), "Tampon : %lu echantillons (%.1f s a 200 Hz)",
           (unsigned long)capacite, capacite * PERIODE_US / 1e6);
  info(msg);

  wifiOn();

  tPrec = micros();
  tDernier = tPrec;

  info("READY");
}

// ---------------- Boucle principale ----------------
void loop() {
  // WiFi : uniquement en état COMMS (jamais pendant l'essai).
  if (wifiActif)
    gererWifi();

  uint32_t now = micros();

  if ((uint32_t)(now - tPrec) < PERIODE_US)
    return;

  // Durée réelle depuis le tour précédent.
  uint32_t dus = now - tDernier;
  tDernier = now;

  // Planification à période fixe pour limiter la dérive.
  // Si la boucle a pris trop de retard, on recale.
  tPrec += PERIODE_US;

  if ((uint32_t)(now - tPrec) > PERIODE_US * 4UL)
    tPrec = now;

  // Protège l'intégration contre un trou de plusieurs ms (envoi WiFi, calibration...).
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

  // Filtre gyro uniquement pour le terme D
  rateF += ALPHA_D * (rate - rateF);

  // Filtre complémentaire
  angle = ALPHA * (angle + rate * dt)
        + (1.0 - ALPHA) * accAngle;

  float erreur = angle - consigne;

  switch (etat) {

    case ETAT_COMMS:
      // Seul le test roues peut faire tourner les moteurs ici.
      if ((int32_t)(testFin - millis()) > 0)
        commandeMoteurs(testPwm);
      else
        commandeMoteurs(0);
      break;

    case ETAT_ARME:
      commandeMoteurs(0);

      if (fabs(angle) < ANGLE_CHUTE && fabs(erreur) < ANGLE_REPRISE)
        demarrerEssai();
      else if (millis() - tArme > ARME_TIMEOUT_MS)
        retourComms("ARME_TIMEOUT");
      break;

    case ETAT_ACTIF: {
      // Chute : on enregistre le dernier point puis on rend la main au PC.
      if (fabs(angle) > ANGLE_CHUTE) {
        enregistrer(0, 0, dus);
        finirEssai("FALL");
        break;
      }

      // I
      integrale += Ki * erreur * dt;
      integrale = constrain(integrale, -I_MAX, I_MAX);

      // IMPORTANT :
      // le D s'oppose à la vitesse de chute.
      // Avec erreur = angle - consigne et gyro positif quand
      // l'angle augmente, le terme D doit donc être -Kd * rateF. FAUX ??
      sortie = Kp * erreur + integrale + Kd * rateF;

      sortie = constrain(sortie, -PWM_MAX, PWM_MAX);

      // Compensation progressive de la zone morte moteur.
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

      enregistrer(cmd, sortie, dus);

      // Tampon plein : fin d'essai propre pour récupérer les données.
      if (nEch >= capacite)
        finirEssai("MAXTIME");
      break;
    }
  }
}
