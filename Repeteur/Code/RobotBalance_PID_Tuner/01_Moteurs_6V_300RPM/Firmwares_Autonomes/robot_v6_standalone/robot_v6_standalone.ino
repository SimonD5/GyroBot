// Robot balance v6 autonome (relevage auto + ESP-NOW)

#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>


// relevage
#define CONF_PWM_KICK           180
#define CONF_DUREE_KICK_MS      350
#define CONF_ANGLE_CAPTURE      2.0f
#define CONF_KD_BOOST           2.5f
#define CONF_SETTLE_MS          600
#define CONF_ANGLE_POSE_MIN     14.0f
#define CONF_ANGLE_POSE_MAX     28.0f

#define CONF_PWM_FLIP           120
#define CONF_DUREE_FLIP_MS      130
#define CONF_TIMEOUT_FLIP_MS    2500

// boucle pid angle
#define CONF_Kp                 16.0f
#define CONF_Ki                  1.0f
#define CONF_Kd                  0.30f
#define CONF_CONSIGNE            0.5f
#define CONF_PWM_MIN            18

// boucle vitesse cascade
#define CONF_Kp2                 0.25f
#define CONF_Ki2                 0.0f
#define CONF_DEADBAND            2.0f

// joystick
#define CONF_USE_JOYSTICK_DROIT  true
#define CONF_JOY_VITESSE_INIT    1.5f
#define CONF_JOY_VITESSE_MIN     0.8f
#define CONF_JOY_VITESSE_MAX     2.8f
#define CONF_JOY_ZONE_MORTE_Y    1.0f
#define CONF_JOY_Y_SIGN         -1.0f
#define CONF_TURN_GAIN           1.2f
#define CONF_JOY_ZONE_MORTE_X    1.0f
#define CONF_TURN_SIGN           1.0f

// securite
#define CONF_ANGLE_CHUTE        18.0f
#define CONF_ANGLE_REPRISE       2.0f
#define CONF_DEBOUNCE_ARME_MS    300
#define CONF_GYRO_SIGN          -1.0f

// com sans fil
#define CONF_MAC_TELE           {0x44, 0xB1, 0x76, 0xCD, 0xD1, 0x38}
#define CONF_WIFI_CHANNEL       1
#define CONF_WIFI_LONG_RANGE    true



#define SDA_PIN   2
#define SCL_PIN   3
#define PWMA     11
#define AIN1      7
#define AIN2     10
#define PWMB     23
#define BIN1     21
#define BIN2     20
#define STBY     22

const int PWM_FREQ      = 20000;
const int PWM_RES       = 8;
const int PWM_MAX       = 200;
const int SIGN_MOTEUR_A =  1;
const int SIGN_MOTEUR_B = -1;

const float ALPHA_V        = 0.03f;
const float ALPHA_D        = 0.40f;
const float ALPHA          = 0.98f;
const float ALPHA_JOY      = 0.04f;
const float I_MAX          = 80.0f;
const float ZONE_RAMPE     = 30.0f;
const float ANGLE_CORR_MAX = 3.0f;
const uint32_t PERIODE_US  = 5000;

float Kp = CONF_Kp, Ki = CONF_Ki, Kd = CONF_Kd;
float consigne = CONF_CONSIGNE;
int   pwmMin   = CONF_PWM_MIN;
float Kp2      = CONF_Kp2, Ki2 = CONF_Ki2;
float V_DEADBAND = CONF_DEADBAND;
float gyroSign   = CONF_GYRO_SIGN;

float inclinaisonMax = CONF_JOY_VITESSE_INIT;

float vitesseEstimee   = 0, correctionAngle = 0, consigneActive = CONF_CONSIGNE;
float rateF = 0, angleAcc = 0, angle = 0, rate = 0;
float integrale = 0, sortie = 0;
float gyroBias  = 0;

// commandes joystick lissées
float correctionJoyCible = 0.0f;
float correctionJoy      = 0.0f;
float turnOffsetCible    = 0.0f;
float turnOffset         = 0.0f;

enum EtatRobot : uint8_t {
  ETAT_ATTENTE,
  ETAT_RELEVAGE_FLIP_AV,
  ETAT_RELEVAGE_ATTENTE_POSE,
  ETAT_KICK,
  ETAT_CAPTURE,
  ETAT_SETTLE,
  ETAT_ACTIF
};
EtatRobot etat = ETAT_ATTENTE;

uint32_t tPrec = 0, tDernier = 0;
uint32_t tDernierEspNow = 0;
const uint32_t ESPNOW_TIMEOUT_MS = 500;

int compteurDebout = 0;
const int COMPTEUR_DEBOUT_CIBLE = CONF_DEBOUNCE_ARME_MS / 5;

// timers et variables specifiques au relevage
uint32_t tDebutFlip = 0;
uint32_t tDebutAttentePose = 0;
uint32_t tDebutKickUs = 0;
uint32_t tDebutCaptureMs = 0;
uint32_t tDebutSettleMs = 0;
int compteurPoseStable = 0;

// com sans fil
uint8_t macTelecommande[6] = CONF_MAC_TELE;

typedef struct {
  int P;
  int I;
  int D;
  int G;
  int EBtnAvD;
  int EBtnAvG;
  int EBtnAvH;
  int EBtnArD;
  int EBtnArG;
  int JGX;
  int JGY;
  int JGBtn;
  int JDX;
  int JDY;
  int JDBtn;
} MessageTelecommande;

volatile bool newDataEspNow = false;
MessageTelecommande msgRecu, msgSafe;

int precBtnAvH = 1;
int precBtnAvG = 1;
int precBtnArD = 1;
int precBtnArG = 1;


// commande des moteurs
void moteur(int pwmPin, int in1, int in2, int v) {
  v = constrain(v, -PWM_MAX, PWM_MAX);
  if (v > 0)       { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (v < 0)  { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else             { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
  ledcWrite(pwmPin, abs(v));
}

void commandeMoteurs(int cmd, int turn = 0) {
  int vA = constrain(cmd + turn, -PWM_MAX, PWM_MAX);
  int vB = constrain(cmd - turn, -PWM_MAX, PWM_MAX);
  moteur(PWMA, AIN1, AIN2, SIGN_MOTEUR_A * vA);
  moteur(PWMB, BIN1, BIN2, SIGN_MOTEUR_B * vB);
}


// capteur mpu-6050
void mpuEcrire(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

bool mpuLire(float &accAngle, float &gyroRate) {
  Wire.beginTransmission(0x68); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(0x68, 14) != 14) return false;
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
  Serial.println("Calibration Gyro (500 mesures) : NE PAS TOUCHER LE ROBOT...");
  float somme = 0; int n = 0; float a, g;
  for (int i = 0; i < 500; i++) {
    if (mpuLire(a, g)) { somme += g; n++; }
    delay(3);
  }
  gyroBias = (n > 0) ? somme / n : 0;
  Serial.printf(">> GyroBias = %.4f deg/s (n=%d) -- OK !\n", gyroBias, n);
}


// callback reception esp-now
void IRAM_ATTR onDataRecv(const esp_now_recv_info_t* info,
                          const uint8_t* data, int len) {
  if (len == (int)sizeof(MessageTelecommande)) {
    memcpy((void*)&msgRecu, data, sizeof(MessageTelecommande));
    newDataEspNow = true;
  }
}


// fonctions de relevage automatique
void demarrerKickAvant() {
  etat            = ETAT_KICK;
  tDebutKickUs    = micros();
  integrale       = 0;
  vitesseEstimee  = 0;
  correctionAngle = 0;
  consigneActive  = consigne;
  sortie          = 0;
  Serial.printf(">>> LANCEMENT KICK AVANT (angle=%.2f deg, PWM=%d) <<<\n", angle, CONF_PWM_KICK);
}

void declencherRelevageBouton() {
// verifier l'orientation du robot pose au sol
  if (angle >= 10.0f) {
// kick relevage
    Serial.printf("Relevage demande : robot couche en avant (%.1f deg) -> Kick direct\n", angle);
    demarrerKickAvant();
  }
  else if (angle <= -10.0f) {
    Serial.printf("Relevage demande : robot couche en arriere (%.1f deg) -> A-coup de basculement vers l'avant\n", angle);
    etat       = ETAT_RELEVAGE_FLIP_AV;
    tDebutFlip = millis();
  }
  else {
    Serial.println("Relevage demande : robot presque vertical -> Armement direct");
    etat           = ETAT_ACTIF;
    integrale      = 0;
    vitesseEstimee = 0;
  }
}


// decodage telecommande & commandes
void traiterEspNow() {
  noInterrupts();
  memcpy(&msgSafe, (const void*)&msgRecu, sizeof(MessageTelecommande));
  newDataEspNow = false;
  interrupts();

  tDernierEspNow = millis();

// choix du joystick (droit ou gauche)
  int valJoyX = CONF_USE_JOYSTICK_DROIT ? msgSafe.JDX : msgSafe.JGX;
  int valJoyY = CONF_USE_JOYSTICK_DROIT ? msgSafe.JDY : msgSafe.JGY;
  int btnJoy  = CONF_USE_JOYSTICK_DROIT ? msgSafe.JDBtn : msgSafe.JGBtn;

// relevage
  bool appuiBoutonArriere = (msgSafe.EBtnArD == 0 || msgSafe.EBtnArG == 0);
  bool clicJoystick       = (btnJoy == 0);

  if (etat == ETAT_ACTIF) {
// en vol : appui arriere ou clic stick = arret d'urgence
    if (appuiBoutonArriere || clicJoystick) {
      commandeMoteurs(0);
      etat = ETAT_ATTENTE;
      compteurDebout = 0;
      Serial.println(">> ARRET D'URGENCE (Bouton Arriere) -> Desarme");
    }
  } else if (etat == ETAT_ATTENTE) {
    bool frontArD = (msgSafe.EBtnArD == 0 && precBtnArD == 1);
    bool frontArG = (msgSafe.EBtnArG == 0 && precBtnArG == 1);
    if (frontArD || frontArG) {
      declencherRelevageBouton();
    }
  }

  precBtnArD = msgSafe.EBtnArD;
  precBtnArG = msgSafe.EBtnArG;

  if (msgSafe.EBtnAvH == 0 && precBtnAvH == 1) {
    inclinaisonMax = min(CONF_JOY_VITESSE_MAX, inclinaisonMax + 0.2f);
    Serial.printf(">> Bouton AvH : Inclinaison Max augmentee a %.1f deg\n", inclinaisonMax);
  }
  precBtnAvH = msgSafe.EBtnAvH;

  if (msgSafe.EBtnAvG == 0 && precBtnAvG == 1) {
    inclinaisonMax = max(CONF_JOY_VITESSE_MIN, inclinaisonMax - 0.2f);
    Serial.printf(">> Bouton AvG : Inclinaison Max reduite a %.1f deg\n", inclinaisonMax);
  }
  precBtnAvG = msgSafe.EBtnAvG;

// ── 3. axe y joystick : avancer / reculer (-15 a +15) ──
  float jdy = (float)valJoyY;
  if (fabs(jdy) <= CONF_JOY_ZONE_MORTE_Y) {
    correctionJoyCible = 0.0f;
  } else {
    float norm = (jdy > 0)
      ? (jdy - CONF_JOY_ZONE_MORTE_Y) / (15.0f - CONF_JOY_ZONE_MORTE_Y)
      : (jdy + CONF_JOY_ZONE_MORTE_Y) / (15.0f - CONF_JOY_ZONE_MORTE_Y);
    correctionJoyCible = norm * inclinaisonMax * CONF_JOY_Y_SIGN;
  }

  float jdx = (float)valJoyX;
  if (fabs(jdx) <= CONF_JOY_ZONE_MORTE_X) {
    turnOffsetCible = 0.0f;
  } else {
    float norm = (jdx > 0)
      ? (jdx - CONF_JOY_ZONE_MORTE_X) / (15.0f - CONF_JOY_ZONE_MORTE_X)
      : (jdx + CONF_JOY_ZONE_MORTE_X) / (15.0f - CONF_JOY_ZONE_MORTE_X);
    turnOffsetCible = norm * CONF_TURN_GAIN * (float)pwmMin * CONF_TURN_SIGN;
  }
}


// initialisation (setup)
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n=======================================================");
  Serial.println("=== RobotBalance V6 - Relevage Telecommande ESP-NOW ===");
  Serial.println("=======================================================");

// broches moteurs
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
  commandeMoteurs(0);
  digitalWrite(STBY, HIGH);

// delai strict de 3 secondes avant initialisation du gyroscope
  Serial.println("\n>>> Attente stricte de 3 secondes avant calibrage du gyro (stabilisation)... <<<");
  for (int s = 3; s > 0; s--) {
    Serial.printf("    %d seconde(s) restante(s)...\n", s);
    delay(1000);
  }

// initialisation i2c et capteur mpu-6050
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpuInit();
  delay(100);
  calibrationGyro();

  float a, g;
  if (mpuLire(a, g)) angle = a;

// wifi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CONF_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

// affichage de l'adresse mac du robot pour la telecommande
  uint8_t macRobot[6];
  esp_wifi_get_mac(WIFI_IF_STA, macRobot);
  Serial.println("\n----------------------------------------------------");
  Serial.printf(">>> MAC DU ROBOT : %02X:%02X:%02X:%02X:%02X:%02X <<<\n",
                macRobot[0], macRobot[1], macRobot[2], macRobot[3], macRobot[4], macRobot[5]);
  Serial.println("----------------------------------------------------\n");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERREUR: esp_now_init a echoue !");
  } else {
    esp_now_register_recv_cb(onDataRecv);

    if (CONF_WIFI_LONG_RANGE) {
      uint8_t protocole = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR;
      esp_wifi_set_protocol(WIFI_IF_STA, protocole);
      esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_LORA_250K);
      Serial.println("ESP-NOW : Mode Longue Portee (LR) active (250 kbps)");
    }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, macTelecommande, 6);
    peer.channel = CONF_WIFI_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
      Serial.printf("ESP-NOW : Telecommande (%02X:%02X:%02X:%02X:%02X:%02X) enregistree OK\n",
                    macTelecommande[0], macTelecommande[1], macTelecommande[2],
                    macTelecommande[3], macTelecommande[4], macTelecommande[5]);
    } else {
      Serial.println("AVERTISSEMENT : Ajout pair telecommande a echoue");
    }
  }

  Serial.println("\n--- Parametres V6 Actifs ---");
  Serial.printf("Relevage    : PWM Kick=%d  DureeKick=%dms  AngleCap=%.1f deg  KdBoost=%.1fx  Settle=%dms\n",
                CONF_PWM_KICK, CONF_DUREE_KICK_MS, CONF_ANGLE_CAPTURE, CONF_KD_BOOST, CONF_SETTLE_MS);
  Serial.printf("Bascule Ar  : PWM Flip=%d  DureeFlip=%dms\n", CONF_PWM_FLIP, CONF_DUREE_FLIP_MS);
  Serial.printf("Equilibre   : Kp=%.1f  Ki=%.1f  Kd=%.2f  Consigne=%.2f deg  Pmin=%d  Kp2=%.2f\n",
                Kp, Ki, Kd, consigne, pwmMin, Kp2);
  Serial.printf("Controle    : InclinaisonMax initiale = %.1f deg (ajustable via BtnAvH / BtnAvG)\n",
                inclinaisonMax);
  Serial.println("\nPret !");
  Serial.println(" - Pose a la verticale : Armement automatique");
  Serial.println(" - Pose au sol (avant ou arriere) : Appuyez sur Bouton Arriere pour vous relever !\n");

  tPrec    = micros();
  tDernier = tPrec;
}


// boucle principale (loop)
void loop() {
// traitement paquets esp-now
  if (newDataEspNow) {
    traiterEspNow();
  }

  if (millis() - tDernierEspNow > ESPNOW_TIMEOUT_MS) {
    correctionJoyCible = 0.0f;
    turnOffsetCible    = 0.0f;
  }

// cadencement a 200 hz (5 ms)
  uint32_t now = micros();
  if ((uint32_t)(now - tPrec) < PERIODE_US) return;

  uint32_t dus = now - tDernier;
  tDernier     = now;
  tPrec       += PERIODE_US;
  if ((uint32_t)(now - tPrec) > PERIODE_US * 4UL) tPrec = now;

  float dt = ((dus > 20000UL) ? 20000UL : dus) / 1000000.0f;

// lissage progressif de la consigne joystick
  correctionJoy += ALPHA_JOY * (correctionJoyCible - correctionJoy);
  turnOffset    += ALPHA_JOY * (turnOffsetCible - turnOffset);

// lecture imu et filtre complementaire
  float a, gr;
  if (mpuLire(a, gr)) {
    rate     = gr - gyroBias;
    angleAcc = a;
  }
  rateF = ALPHA_D * rate + (1.0f - ALPHA_D) * rateF;
  angle = ALPHA * (angle + rate * dt) + (1.0f - ALPHA) * angleAcc;


  switch (etat) {

// armement
    case ETAT_ATTENTE: {
      commandeMoteurs(0);
      integrale          = 0;
      vitesseEstimee     = 0;
      correctionAngle    = 0;
      sortie             = 0;
      consigneActive     = consigne;
      correctionJoyCible = 0;
      correctionJoy      = 0;
      turnOffsetCible    = 0;
      turnOffset         = 0;

// armement
      if (fabs(angle - consigne) < CONF_ANGLE_REPRISE && fabs(angle) < CONF_ANGLE_CHUTE) {
        compteurDebout++;
        if (compteurDebout >= COMPTEUR_DEBOUT_CIBLE) {
          etat = ETAT_ACTIF;
          compteurDebout = 0;
          integrale = 0;
          Serial.printf(">>> ARME AUTOMATIQUEMENT A LA VERTICALE ! Angle=%.2f deg <<<\n", angle);
        }
      } else {
        compteurDebout = 0;
      }
      break;
    }

    case ETAT_RELEVAGE_FLIP_AV: {
      uint32_t tFlip = millis() - tDebutFlip;
      if (tFlip < CONF_DUREE_FLIP_MS) {
        commandeMoteurs(-CONF_PWM_FLIP);
      } else {
        commandeMoteurs(0);
        etat = ETAT_RELEVAGE_ATTENTE_POSE;
        tDebutAttentePose  = millis();
        compteurPoseStable = 0;
        Serial.println("A-coup termine : attente amortissement et pose sur l'avant...");
      }
      break;
    }

// ── etat relevage 2 : attente de la pose stable en avant ──
    case ETAT_RELEVAGE_ATTENTE_POSE: {
      commandeMoteurs(0);
      uint32_t tPose = millis() - tDebutAttentePose;

      if (angle >= CONF_ANGLE_POSE_MIN && angle <= CONF_ANGLE_POSE_MAX && fabs(rateF) < 30.0f) {
        compteurPoseStable++;
        if (compteurPoseStable >= 40) {
          Serial.println("Robot bien pose sur l'avant -> Enchainement Kick de relevage !");
          demarrerKickAvant();
        }
      } else {
        compteurPoseStable = 0;
      }

// timeout si le robot n'a pas bascule ou ne s'est pas pose
      if (tPose > CONF_TIMEOUT_FLIP_MS) {
        commandeMoteurs(0);
        etat = ETAT_ATTENTE;
        Serial.println("Timeout bascule avant -> Retour attente");
      }
      break;
    }

// kick relevage
    case ETAT_KICK: {
      uint32_t tKickMs = (micros() - tDebutKickUs) / 1000UL;
      commandeMoteurs(CONF_PWM_KICK);

      bool captureAtteinte = (angle <= consigne + CONF_ANGLE_CAPTURE);
      bool timeoutKick     = (tKickMs >= CONF_DUREE_KICK_MS);

      if (captureAtteinte || timeoutKick) {
        etat = ETAT_CAPTURE;
        tDebutCaptureMs = millis();
        integrale       = 0;
        vitesseEstimee  = 0;
        correctionAngle = 0;
        sortie          = 0;
        Serial.printf("KICK->CAPTURE apres %lu ms (angle=%.2f)\n", tKickMs, angle);
      }
      break;
    }

// ── etat relevage 4 : freinage kd boost (2.5x) ──
    case ETAT_CAPTURE: {
// securite bascule
      if (angle < -CONF_ANGLE_CHUTE || angle > CONF_ANGLE_CHUTE + 8.0f) {
        commandeMoteurs(0);
        etat = ETAT_ATTENTE;
        Serial.println("Chute pendant Capture -> Moteurs coupes");
        break;
      }

      float kdActif = Kd * CONF_KD_BOOST;
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

// transition vers settle : robot proche de 0 deg et ralenti
      uint32_t tCapMs = millis() - tDebutCaptureMs;
      bool presque_droit = fabs(angle - consigne) < 2.0f;
      bool ralenti       = fabs(rateF) < 40.0f;

      if ((presque_droit && ralenti) || tCapMs >= 400) {
        etat = ETAT_SETTLE;
        tDebutSettleMs  = millis();
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

// ── etat relevage 5 : settle (gel boucle 2 pendant 600 ms) ──
    case ETAT_SETTLE: {
      if (fabs(angle) > CONF_ANGLE_CHUTE) {
        commandeMoteurs(0);
        etat = ETAT_ATTENTE;
        Serial.println("Chute pendant Settle -> Moteurs coupes");
        break;
      }

// boucle 1 seule, pas de boucle 2
      consigneActive = consigne;
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

// fin du settle -> passage direct en vol autonome actif !
      if ((int32_t)(millis() - tDebutSettleMs) >= CONF_SETTLE_MS) {
        etat = ETAT_ACTIF;
        vitesseEstimee  = 0;
        correctionAngle = 0;
        integrale       = 0;
        sortie          = 0;
        Serial.printf(">>> RELEVAGE REUSSI ! En equilibre actif (angle=%.2f deg) <<<\n", angle);
      }
      break;
    }

    case ETAT_ACTIF: {
// securite chute immediate
      if (fabs(angle) > CONF_ANGLE_CHUTE) {
        commandeMoteurs(0);
        etat = ETAT_ATTENTE;
        compteurDebout     = 0;
        integrale          = 0;
        vitesseEstimee     = 0;
        correctionJoyCible = 0;
        correctionJoy      = 0;
        sortie             = 0;
        Serial.printf("CHUTE DETECTEE (angle=%.2f deg) -> Moteurs coupes\n", angle);
        break;
      }

// boucle 2 : correction de derive lente avec zone morte
      if (fabs(correctionJoyCible) > 0.05f) {
        vitesseEstimee  = 0.0f;
        correctionAngle = 0.0f;
      } else {
        vitesseEstimee += ALPHA_V * (sortie - vitesseEstimee);
        float vActive = 0.0f;
        if (fabs(vitesseEstimee) > V_DEADBAND) {
          vActive = (vitesseEstimee > 0)
              ? (vitesseEstimee - V_DEADBAND)
              : (vitesseEstimee + V_DEADBAND);
        }
        correctionAngle = constrain(Kp2 * vActive, -ANGLE_CORR_MAX, ANGLE_CORR_MAX);
      }

      consigneActive = constrain(
          consigne - correctionAngle + correctionJoy,
          consigne - ANGLE_CORR_MAX - inclinaisonMax,
          consigne + ANGLE_CORR_MAX + inclinaisonMax
      );

// boucle 1 : pid de stabilisation sur l'angle
      float erreur = angle - consigneActive;
      integrale += Ki * erreur * dt;
      integrale  = constrain(integrale, -I_MAX, I_MAX);
      sortie     = Kp * erreur + integrale + Kd * rateF;
      sortie     = constrain(sortie, -PWM_MAX, PWM_MAX);

// rampe de demarrage pour vaincre le frottement statique
      float absS   = fabs(sortie);
      float offset = pwmMin * min(absS / ZONE_RAMPE, 1.0f);
      int   cmd    = (int)constrain(absS + offset, 0.0f, (float)PWM_MAX);
      if (sortie < 0) cmd = -cmd;

// joystick x : differentiel de virage
      int turn = (int)constrain(turnOffset, -(float)PWM_MAX, (float)PWM_MAX);

      commandeMoteurs(cmd, turn);
      break;
    }
  }
}
