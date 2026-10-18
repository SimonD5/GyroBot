// ============================================================
// RobotBalance - Banc d'Essais Relevage Automatique (Wi-Fi)
// ESP32-C6 + MPU6050 GY-521 + TB6612FNG
//
// OBJECTIF :
//   Tester et optimiser le redressement automatique du robot
//   depuis sa position de repos au sol (+/-20 deg) jusqu'a la
//   verticale, suivi de 10 secondes de maintien en equilibre stable.
//
// PHASES DU RELEVAGE :
//   1. KICK     : Impulsion franche de vitesse des roues (PWM Kick, Duree Kick)
//   2. CAPTURE  : Arrivee a l'Angle Capture -> freinage Kd boost
//   3. SETTLE   : Gel de la boucle 2 pendant SETTLE_MS : la boucle 1 seule
//                  stabilise le robot sans interference de la correction derive
//   4. EQUILIBRE: Maintien en double boucle cascade pendant 10 secondes.
//
// LIMITES STRICTES : PWM_MAX = 200 (Protection moteurs N20)
// RESEAU : SoftAP "RobotPID", mot de passe "robot1234", IP 192.168.4.1:3333
// ============================================================

#include <Wire.h>
#include <WiFi.h>

// ---------------- Réseau Wi-Fi ----------------
const char*    AP_SSID    = "RobotPID";
const char*    AP_PASS    = "robot1234";
const uint8_t  AP_CHANNEL = 1;
const uint16_t TCP_PORT   = 3333;

WiFiServer serveur(TCP_PORT);
WiFiClient client;

// ---------------- Broches matérielles (ESP32-C6) ----------------
#define SDA_PIN 2
#define SCL_PIN 3
#define PWMA 11
#define AIN1 7
#define AIN2 10
#define PWMB 23
#define BIN1 21
#define BIN2 20
#define STBY 22

// ---------------- Constantes matérielles ----------------
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int PWM_MAX  = 200; // LIMITE ABSOLUE : NE JAMAIS DEPASSER

const int SIGN_MOTEUR_A =  1;
const int SIGN_MOTEUR_B = -1;

// ---------------- Paramètres Relevage (modifiables par Wi-Fi) ----------------
int   pwmKick           = 180;   // Puissance impulsion kick (100 a 200 MAX)
int   dureeKickMs       = 350;   // Duree max du kick en ms (angle capture pris avant en general)
float angleCapture      = 2.0f;  // Angle d'approche pour couper le kick et freiner (deg)
float kdBoost           = 2.5f;  // Multiplicateur Kd pour freiner l'arrivee au sommet
float anglePoseMin      = 14.0f; // Angle min pour considerer le robot pose au sol
float anglePoseMax      = 28.0f; // Angle max pour considerer le robot pose au sol
float dureeEquilibreS   = 10.0f; // Duree de maintien d'equilibre visee (secondes)
int   settleMs          = 600;   // ms de gel de la boucle 2 apres capture (evite derive parasite)

// ---------------- PID Stabilisation (Boucle 1 & 2) ----------------
float Kp          = 16.0f;
float Ki          =  1.0f;
float Kd          =  0.30f;
float consigne    =  0.5f;
int   pwmMin      = 18;
float Kp2         =  0.25f;
float Ki2         =  0.0f;
float V_DEADBAND  =  2.0f;

const float ALPHA_V        = 0.03f;
const float ALPHA_D        = 0.40f;
const float ALPHA          = 0.98f;
const float I_MAX          = 80.0f;
const float ZONE_RAMPE     = 30.0f;
const float ANGLE_CORR_MAX = 3.0f;
const float ANGLE_CHUTE_MAX= 35.0f;
const uint32_t PERIODE_US  = 5000;  // 200 Hz

// ---------------- Tampon d'enregistrement (12 secondes = 2400 éch.) ----------------
const uint32_t MAX_ECHANTILLONS = 2400;

struct Echantillon {
  uint32_t t_ms;
  int16_t  angle;     // 0.01 deg
  int16_t  acc;       // 0.01 deg
  int16_t  gyro;      // 0.1 deg/s
  int16_t  rateF;     // 0.1 deg/s
  int16_t  pid;       // 0.1
  int16_t  pwm;       // commande moteurs
  int16_t  target;    // 0.01 deg consigne active
  int16_t  spd;       // 0.1 vitesse estimee
  uint16_t dt_us;     // duree cycle
  uint8_t  phase;     // 0=KICK, 1=CAPTURE, 2=SETTLE, 3=EQUILIBRE
};

Echantillon* tampon = nullptr;
uint32_t capacite = 0;
uint32_t nEch = 0;

// ---------------- MPU6050 ----------------
const uint8_t MPU_ADDR = 0x68;
float gyroBias = 0;
float gyroSign = -1.0f;

float rateF = 0, angleAcc = 0, angle = 0, rate = 0;
float integrale = 0, sortie = 0;
float vitesseEstimee = 0, correctionAngle = 0, consigneActive = 0.5f;

// ---------------- Machine d'états ----------------
enum EtatRobot : uint8_t {
  ETAT_COMMS,          // Wi-Fi ouvert, moteurs coupes, parametrage
  ETAT_ARME_ATTENTE,   // Arme : attend que le robot soit pose a ~20 deg au sol
  ETAT_KICK,           // Phase 1 : Impulsion violente de redressement
  ETAT_CAPTURE,        // Phase 2 : Coupure kick, freinage Kd Boost
  ETAT_SETTLE,         // Phase 3 : Gel boucle 2 le temps que le robot se calme
  ETAT_EQUILIBRE       // Phase 4 : Asservissement PID d'equilibre (10s)
};
EtatRobot etat = ETAT_COMMS;

uint32_t tPrec = 0, tDernier = 0;
uint32_t tArme = 0;
uint32_t tDebutRelevage = 0;
uint32_t tDebutKickUs = 0;
uint32_t tDebutCaptureMs = 0;
uint32_t tDebutSettleMs = 0;
int sensKick = 1;
int compteurPoseStable = 0;
const int SEUIL_POSE_STABLE = 80; // 80 * 5ms = 400 ms stable au sol

uint32_t essaiNum = 0;
bool trialPret = false;
const char* trialRaison = "";

int   essaiPwmKick    = 180;
int   essaiDureeKick  = 350;
float essaiAngleCap   = 2.0f;
float essaiKdBoost    = 2.5f;
int   essaiSettleMs   = 600;


// ============================================================
//                    COMMANDE DES MOTEURS
// ============================================================
void moteur(int pwmPin, int in1, int in2, int v) {
  v = constrain(v, -PWM_MAX, PWM_MAX);
  if (v > 0)       { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (v < 0)  { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else             { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
  ledcWrite(pwmPin, abs(v));
}

void commandeMoteurs(int cmd) {
  cmd = constrain(cmd, -PWM_MAX, PWM_MAX);
  moteur(PWMA, AIN1, AIN2, SIGN_MOTEUR_A * cmd);
  moteur(PWMB, BIN1, BIN2, SIGN_MOTEUR_B * cmd);
}


// ============================================================
//                    CAPTEUR MPU-6050
// ============================================================
void mpuEcrire(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

bool mpuLire(float &accAngle, float &gyroRate) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); Wire.read(); Wire.read();
  (void)ax;
  accAngle = atan2((float)az, (float)ay) * RAD_TO_DEG;
  gyroRate = gyroSign * gx / 65.5f;
  return true;
}

void mpuInit() {
  mpuEcrire(0x6B, 0x00);
  mpuEcrire(0x1A, 0x03);
  mpuEcrire(0x1B, 0x08);
  mpuEcrire(0x1C, 0x00);
}

void calibrationGyro() {
  commandeMoteurs(0);
  Serial.println("Calibration gyro...");
  float somme = 0; int n = 0; float a, g;
  for (int i = 0; i < 400; i++) {
    if (mpuLire(a, g)) { somme += g; n++; }
    delay(3);
  }
  gyroBias = (n > 0) ? somme / n : 0;
  Serial.printf("GyroBias = %.5f deg/s\n", gyroBias);
}


// ============================================================
//                    GESTION DU TAMPON
// ============================================================
static inline int16_t q16(float v, float ech) {
  float x = v * ech;
  if (x >  32767.0f) return  32767;
  if (x < -32768.0f) return -32768;
  return (int16_t)lroundf(x);
}

void enregistrer(int cmd, float sortiePid, uint16_t dus, uint8_t phase) {
  if (!tampon || nEch >= capacite) return;
  Echantillon &e = tampon[nEch++];
  e.t_ms   = millis() - tDebutRelevage;
  e.angle  = q16(angle, 100.0f);
  e.acc    = q16(angleAcc, 100.0f);
  e.gyro   = q16(rate, 10.0f);
  e.rateF  = q16(rateF, 10.0f);
  e.pid    = q16(sortiePid, 10.0f);
  e.pwm    = (int16_t)cmd;
  e.target = q16(consigneActive, 100.0f);
  e.spd    = q16(vitesseEstimee, 10.0f);
  e.dt_us  = dus;
  e.phase  = phase;
}


// ============================================================
//                    COMMUNICATIONS TCP
// ============================================================
void envoyerParametres() {
  if (!client || !client.connected()) return;
  client.printf("PARAM,Kp=%.2f,Ki=%.2f,Kd=%.2f,consigne=%.2f,pwmMin=%d,"
                "Kp2=%.3f,Ki2=%.3f,dead=%.1f,pKick=%d,tKick=%d,cKick=%.1f,"
                "fBoost=%.2f,tTest=%.1f,settle=%d,armed=%d\n",
                Kp, Ki, Kd, consigne, pwmMin,
                Kp2, Ki2, V_DEADBAND,
                pwmKick, dureeKickMs, angleCapture,
                kdBoost, dureeEquilibreS, settleMs,
                (etat != ETAT_COMMS) ? 1 : 0);
}

void demarrerRelevage() {
  nEch = 0;
  tDebutRelevage  = millis();
  tDebutKickUs    = micros();
  sensKick        = (angle > 0) ? 1 : -1;
  etat            = ETAT_KICK;
  integrale       = 0;
  vitesseEstimee  = 0;   // Propre à zéro en debut de relevage
  correctionAngle = 0;
  consigneActive  = consigne;
  sortie          = 0;

  essaiPwmKick   = pwmKick;
  essaiDureeKick = dureeKickMs;
  essaiAngleCap  = angleCapture;
  essaiKdBoost   = kdBoost;
  essaiSettleMs  = settleMs;

  essaiNum++;
  trialPret   = false;
  trialRaison = "";

  if (client && client.connected()) {
    client.printf("TRIAL_START,%lu,%s\n", essaiNum, (sensKick > 0) ? "KICK_AVANT" : "KICK_ARRIERE");
  }
  Serial.printf("\n>>> ESSAI #%lu : KICK sens=%d angle=%.2f pwm=%d t=%dms <<<\n",
                essaiNum, sensKick, angle, pwmKick, dureeKickMs);
}

void terminerEssai(const char* raison) {
  commandeMoteurs(0);
  trialRaison = raison;
  trialPret   = true;
  etat        = ETAT_COMMS;
  compteurPoseStable = 0;
  vitesseEstimee  = 0;
  correctionAngle = 0;
  integrale       = 0;
  sortie          = 0;
  Serial.printf("Fin essai #%lu : %s (%lu echantillons)\n", essaiNum, raison, nEch);
}

void viderDonneesVersClient() {
  if (!trialPret || !client || !client.connected()) return;
  Serial.printf("Transmission essai #%lu au PC (%lu pts)...\n", essaiNum, nEch);

  client.printf("TRIAL,%lu,%lu,Kp=%.2f,Ki=%.2f,Kd=%.2f,consigne=%.2f,pwmMin=%d,"
                "pKick=%d,tKick=%d,cKick=%.1f,fBoost=%.2f,settle=%d,raison=%s\n",
                essaiNum, nEch, Kp, Ki, Kd, consigne, pwmMin,
                essaiPwmKick, essaiDureeKick, essaiAngleCap, essaiKdBoost,
                essaiSettleMs, trialRaison);

  for (uint32_t i = 0; i < nEch; i++) {
    Echantillon &e = tampon[i];
    client.printf("D,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u\n",
                  e.t_ms, e.angle, e.acc, e.gyro, e.rateF,
                  e.pid, e.pwm, e.target, e.spd, e.phase, e.dt_us);
  }
  client.printf("END,%lu\n", essaiNum);
  trialPret = false;
}

void traiterCommandesReseau() {
  if (!client || !client.connected()) return;
  while (client.available()) {
    String ligne = client.readStringUntil('\n');
    ligne.trim();
    if (ligne.length() == 0) continue;

    char cmd = ligne.charAt(0);
    String arg = ligne.substring(1);
    float val = arg.toFloat();

    switch (cmd) {
      case 'p': Kp = constrain(val, 0.0f, 60.0f); break;
      case 'i': Ki = constrain(val, 0.0f, 20.0f); break;
      case 'd': Kd = constrain(val, 0.0f, 5.0f);  break;
      case 's': consigne = constrain(val, -10.0f, 10.0f); consigneActive = consigne; break;
      case 'm': pwmMin = constrain((int)val, 0, 80); break;
      case 'k': Kp2 = constrain(val, 0.0f, 2.0f); break;
      case 'b': V_DEADBAND = constrain(val, 0.0f, 20.0f); break;

      case 'u': pwmKick = constrain((int)val, 50, PWM_MAX); break;
      case 't': dureeKickMs = constrain((int)val, 20, 500); break;
      case 'c': angleCapture = constrain(val, 0.5f, 15.0f); break;
      case 'f': kdBoost = constrain(val, 1.0f, 4.0f); break;
      case 'h': dureeEquilibreS = constrain(val, 2.0f, 20.0f); break;
      case 'n': settleMs = constrain((int)val, 0, 2000); break; // n = settle duration

      case 'a':
        if (val == 1.0f) {
          etat = ETAT_ARME_ATTENTE;
          compteurPoseStable = 0;
          tArme = millis();
          Serial.println("ARME : Posez le robot a +/-20 deg au sol");
          if (client && client.connected()) client.println("INFO,ARME : Posez le robot a +/-20 deg...");
        } else {
          terminerEssai("DESARME_MANUEL");
        }
        break;

      case 'x': // Déclenchement forcé immédiat
        demarrerRelevage();
        break;

      case 'q':
        terminerEssai("ARRET_URGENCE");
        break;

      case '?':
        envoyerParametres();
        break;
    }
  }
}


// ============================================================
//                    INITIALISATION (SETUP)
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== RobotBalance - Relevage Automatique (Wi-Fi) ===");

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
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
  if (mpuLire(a, g)) angle = a;

  capacite = MAX_ECHANTILLONS;
  tampon = (Echantillon*)malloc(capacite * sizeof(Echantillon));
  if (!tampon) {
    Serial.println("Erreur allocation tampon, taille reduite...");
    capacite = 1500;
    tampon = (Echantillon*)malloc(capacite * sizeof(Echantillon));
  }
  Serial.printf("Tampon: %lu echantillons (%lu octets)\n",
                capacite, capacite * sizeof(Echantillon));

  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL);
  WiFi.setSleep(false);

  serveur.begin();
  serveur.setNoDelay(true);

  Serial.printf("SoftAP pret : SSID=%s IP=%s port=%d\n",
                AP_SSID, WiFi.softAPIP().toString().c_str(), TCP_PORT);

  tPrec = micros();
  tDernier = tPrec;
}


// ============================================================
//                    BOUCLE PRINCIPALE (LOOP)
// ============================================================
void loop() {
  // Gestion connexion client TCP
  if (etat == ETAT_COMMS) {
    if (!client || !client.connected()) {
      WiFiClient nouveau = serveur.accept();
      if (nouveau) {
        client = nouveau;
        client.setNoDelay(true);
        Serial.println("Client PC connecte !");
        envoyerParametres();
      }
    } else {
      traiterCommandesReseau();
      if (trialPret) viderDonneesVersClient();
    }
  }

  // Boucle temps réel 200 Hz
  uint32_t now = micros();
  if ((uint32_t)(now - tPrec) < PERIODE_US) return;

  uint32_t dus = now - tDernier;
  tDernier = now;
  tPrec   += PERIODE_US;
  if ((uint32_t)(now - tPrec) > PERIODE_US * 4UL) tPrec = now;

  float dt = ((dus > 20000UL) ? 20000UL : dus) / 1000000.0f;

  float a, gr;
  if (mpuLire(a, gr)) {
    rate     = gr - gyroBias;
    angleAcc = a;
  }
  rateF = ALPHA_D * rate + (1.0f - ALPHA_D) * rateF;
  angle = ALPHA * (angle + rate * dt) + (1.0f - ALPHA) * angleAcc;

  switch (etat) {

    // ── 0. COMMS ──────────────────────────────────────────────
    case ETAT_COMMS:
      commandeMoteurs(0);
      integrale = 0; vitesseEstimee = 0; correctionAngle = 0;
      sortie = 0; consigneActive = consigne;
      break;

    // ── 1. ARMÉ EN ATTENTE ─────────────────────────────────────
    case ETAT_ARME_ATTENTE:
      commandeMoteurs(0);
      integrale = 0; vitesseEstimee = 0; correctionAngle = 0; sortie = 0;
      {
        float absA = fabs(angle);
        if (absA >= anglePoseMin && absA <= anglePoseMax) {
          compteurPoseStable++;
          if (compteurPoseStable >= SEUIL_POSE_STABLE) demarrerRelevage();
        } else {
          compteurPoseStable = 0;
        }
        if (millis() - tArme > 30000) terminerEssai("TIMEOUT_POSE");
      }
      break;

    // ── 2. PHASE KICK ──────────────────────────────────────────
    case ETAT_KICK: {
      uint32_t tEcouleKickMs = (micros() - tDebutKickUs) / 1000UL;
      int cmdKick = sensKick * pwmKick;
      commandeMoteurs(cmdKick);
      enregistrer(cmdKick, (float)cmdKick, (uint16_t)dus, 0);

      // Angle capture : dépend du sens
      bool captureAtteinte = false;
      if (sensKick == 1 && angle <= consigne + angleCapture) captureAtteinte = true;
      if (sensKick == -1 && angle >= consigne - angleCapture) captureAtteinte = true;
      bool dureeDepassee = (tEcouleKickMs >= (uint32_t)dureeKickMs);

      if (captureAtteinte || dureeDepassee) {
        etat = ETAT_CAPTURE;
        tDebutCaptureMs = millis();
        // Remise à zéro PROPRE : l'intégrale et la vitesse estimée ne doivent pas
        // "se souvenir" de la phase Kick
        integrale      = 0;
        vitesseEstimee = 0;
        correctionAngle= 0;
        sortie         = 0;
        Serial.printf("KICK->CAPTURE apres %lu ms (angle=%.2f cap=%d timeout=%d)\n",
                      tEcouleKickMs, angle, captureAtteinte ? 1 : 0, dureeDepassee ? 1 : 0);
      }
      break;
    }

    // ── 3. PHASE CAPTURE (Freinage Kd Boost) ───────────────────
    case ETAT_CAPTURE: {
      if (fabs(angle) > ANGLE_CHUTE_MAX) {
        enregistrer(0, 0, (uint16_t)dus, 1);
        terminerEssai("CHUTE_CAPTURE");
        break;
      }

      float kdActif = Kd * kdBoost;
      float erreur  = angle - consigne;
      integrale    += Ki * erreur * dt;
      integrale     = constrain(integrale, -I_MAX, I_MAX);

      sortie = Kp * erreur + integrale + kdActif * rateF;
      sortie = constrain(sortie, -PWM_MAX, PWM_MAX);

      float absS   = fabs(sortie);
      float offset = pwmMin * min(absS / ZONE_RAMPE, 1.0f);
      int   cmd    = (int)constrain(absS + offset, 0.0f, (float)PWM_MAX);
      if (sortie < 0) cmd = -cmd;

      commandeMoteurs(cmd);
      enregistrer(cmd, sortie, (uint16_t)dus, 1);

      // Transition vers SETTLE : on passe dès que le robot est presque droit ET lent
      uint32_t tCapMs = millis() - tDebutCaptureMs;
      bool presque_droit = fabs(angle - consigne) < 2.0f;
      bool ralenti       = fabs(rateF) < 40.0f;

      if ((presque_droit && ralenti) || tCapMs >= 400) {
        etat = ETAT_SETTLE;
        tDebutSettleMs = millis();
        // REMISE A ZERO COMPLETE de la vitesse estimee et de l'integrale
        // pour eviter que la boucle 2 ne pousse le robot en arriere
        vitesseEstimee  = 0;
        correctionAngle = 0;
        integrale       = 0;
        sortie          = 0;
        consigneActive  = consigne;
        Serial.printf("CAPTURE->SETTLE apres %lu ms (angle=%.2f rateF=%.1f)\n",
                      tCapMs, angle, rateF);
      }
      break;
    }

    // ── 4. PHASE SETTLE (gel boucle 2, boucle 1 seule) ─────────
    // Le robot se stabilise sur la boucle 1 pendant settleMs
    // sans aucune correction de derive (evite le depart en arriere)
    case ETAT_SETTLE: {
      if (fabs(angle) > ANGLE_CHUTE_MAX) {
        enregistrer(0, 0, (uint16_t)dus, 2);
        terminerEssai("CHUTE_SETTLE");
        break;
      }

      // Boucle 1 uniquement (Kd = Kd normal, plus de boost)
      consigneActive = consigne; // Pas de correction derive
      float erreur   = angle - consigneActive;
      integrale += Ki * erreur * dt;
      integrale  = constrain(integrale, -I_MAX, I_MAX);

      sortie = Kp * erreur + integrale + Kd * rateF;
      sortie = constrain(sortie, -PWM_MAX, PWM_MAX);

      float absS   = fabs(sortie);
      float offset = pwmMin * min(absS / ZONE_RAMPE, 1.0f);
      int   cmd    = (int)constrain(absS + offset, 0.0f, (float)PWM_MAX);
      if (sortie < 0) cmd = -cmd;

      commandeMoteurs(cmd);
      enregistrer(cmd, sortie, (uint16_t)dus, 2);

      // Apres settleMs, on passe en equlibre complet avec la boucle 2
      if ((int32_t)(millis() - tDebutSettleMs) >= settleMs) {
        etat = ETAT_EQUILIBRE;
        // Initialisation propre de la boucle 2 depuis un etat calme
        vitesseEstimee  = 0;
        correctionAngle = 0;
        integrale       = 0; // Integrale remis a zero pour un demarrage propre
        sortie          = 0;
        Serial.printf("SETTLE->EQUILIBRE (angle=%.2f deg)\n", angle);
      }
      break;
    }

    // ── 5. PHASE EQUILIBRE (Double boucle cascade - 10s) ───────
    case ETAT_EQUILIBRE: {
      if (fabs(angle) > ANGLE_CHUTE_MAX) {
        enregistrer(0, 0, (uint16_t)dus, 3);
        terminerEssai("CHUTE_EQUILIBRE");
        break;
      }

      // Boucle 2 : Dérive lente avec zone morte
      vitesseEstimee += ALPHA_V * (sortie - vitesseEstimee);
      float vActive = 0.0f;
      if (fabs(vitesseEstimee) > V_DEADBAND) {
        vActive = (vitesseEstimee > 0)
            ? (vitesseEstimee - V_DEADBAND)
            : (vitesseEstimee + V_DEADBAND);
      }
      correctionAngle = constrain(Kp2 * vActive, -ANGLE_CORR_MAX, ANGLE_CORR_MAX);
      consigneActive  = consigne - correctionAngle;

      // Boucle 1 : Angle PID
      float erreur = angle - consigneActive;
      integrale += Ki * erreur * dt;
      integrale  = constrain(integrale, -I_MAX, I_MAX);

      sortie = Kp * erreur + integrale + Kd * rateF;
      sortie = constrain(sortie, -PWM_MAX, PWM_MAX);

      float absS   = fabs(sortie);
      float offset = pwmMin * min(absS / ZONE_RAMPE, 1.0f);
      int   cmd    = (int)constrain(absS + offset, 0.0f, (float)PWM_MAX);
      if (sortie < 0) cmd = -cmd;

      commandeMoteurs(cmd);
      enregistrer(cmd, sortie, (uint16_t)dus, 3);

      uint32_t echCible = (uint32_t)(dureeEquilibreS * 200.0f);
      if (nEch >= echCible || nEch >= capacite) terminerEssai("SUCCES_10S");
      break;
    }
  }
}
