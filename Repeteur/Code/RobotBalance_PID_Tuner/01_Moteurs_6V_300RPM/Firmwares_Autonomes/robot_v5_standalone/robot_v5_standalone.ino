// ============================================================
// RobotBalance V5 - Telecommande ESP-NOW Autonome
// ESP32-C6 + MPU6050 GY-521 + Driver TB6612FNG
//
// 100% AUTONOME (Pas de WiFi AP, pas de serveur TCP, pas de GUI Python)
// - Reglage : modifier facilement les parametres ci-dessous et reteleverser
// - Armement : automatique quand le robot est tenu droit (~300 ms)
// - Pilotage : 1 joystick (droit par defaut)
//     Axe Y : Avancer / Reculer (inclinaison de consigne progressive)
//     Axe X : Tourner Gauche / Droite (differentiel moteur)
// - Securite : arret automatique sur chute (> 18 deg)
//              arret d'urgence possible via bouton ArD ou clic joystick
// ============================================================

#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// [==============================================================]
// [               SECTION REGLAGES -- A MODIFIER                 ]
// [==============================================================]

// ---- PID Boucle 1 : Angle (stabilisation) ----
#define CONF_Kp          16.0f   // Gain proportionnel (15 - 20)
#define CONF_Ki           1.0f   // Gain integral (0.5 - 2.0)
#define CONF_Kd           0.30f  // Gain derive (0.20 - 0.40)
#define CONF_CONSIGNE     0.5f   // Angle d'equilibre naturel en degres (ajuster si le robot penche)
#define CONF_PWM_MIN     18      // PWM min pour vaincre le frottement statique (16 - 20)

// ---- PID Boucle 2 : Correction de derive lente ----
#define CONF_Kp2          0.25f  // Gain anti-derive (0.15 - 0.35) -- 0 = desactive
#define CONF_Ki2          0.0f   // Integrale boucle 2 (garder a 0.0)
#define CONF_DEADBAND     2.0f   // Zone morte anti-tremblement boucle 2 (unites PWM)

// ---- Pilotage par Joystick ----
#define CONF_USE_JOYSTICK_DROIT true // true = Joystick Droit, false = Joystick Gauche

// Axe Y (Avancer / Reculer)
#define CONF_JOY_VITESSE_MAX   1.5f  // Inclinaison max ajoutee a la consigne (degres) : 1.2 a 1.8 max pour eviter la chute
#define CONF_JOY_ZONE_MORTE_Y  1.0f  // Seuil de zone morte au centre (-15 a +15)
#define CONF_JOY_Y_SIGN       -1.0f  // -1.0f pour avancer en poussant vers l'avant, +1.0f si inverse

// Axe X (Tourner Gauche / Droite)
#define CONF_TURN_GAIN         1.2f  // Sensibilite de rotation
#define CONF_JOY_ZONE_MORTE_X  1.0f  // Seuil de zone morte au centre (-15 a +15)
#define CONF_TURN_SIGN         1.0f  // +1.0 ou -1.0 si besoin d'inverser gauche/droite

// ---- Gyroscope / MPU6050 ----
#define CONF_GYRO_SIGN       -1.0f  // -1.0f (inverser a +1.0f si le robot s'emballe immediatement)

// ---- Securite et Armement ----
#define CONF_ANGLE_CHUTE     18.0f  // Angle absolu en degres declenchant la coupure moteur
#define CONF_ANGLE_REPRISE    2.0f  // Ecart max a la consigne pour autoriser l'armement (degres)
#define CONF_DEBOUNCE_ARME_MS 300   // Temps stable a la verticale requis pour s'armer (ms)

// ---- Liaison ESP-NOW & WiFi ----
#define CONF_MAC_TELE        {0x44, 0xB1, 0x76, 0xCD, 0xD1, 0x38} // MAC de telecommandev7
#define CONF_WIFI_CHANNEL    1      // Doit correspondre au canal de la telecommande (1 par defaut)
#define CONF_WIFI_LONG_RANGE true   // Activer le mode Long Range (true = defaut de telecommandev7)

// [==================== FIN SECTION REGLAGES ====================]


// ---------------- Broches materielles (ESP32-C6) ----------------
#define SDA_PIN   2
#define SCL_PIN   3
#define PWMA     11
#define AIN1      7
#define AIN2     10
#define PWMB     23
#define BIN1     21
#define BIN2     20
#define STBY     22

// ---------------- Constantes materielles ----------------
const int PWM_FREQ      = 20000;
const int PWM_RES       = 8;
const int PWM_MAX       = 200;
const int SIGN_MOTEUR_A =  1;
const int SIGN_MOTEUR_B = -1;

// ---------------- Constantes algorithme ----------------
const float ALPHA_V        = 0.03f;
const float ALPHA_D        = 0.40f;
const float ALPHA          = 0.98f;
const float ALPHA_JOY      = 0.04f; // Lissage progressif de la consigne joystick (~120 ms)
const float I_MAX          = 80.0f;
const float ZONE_RAMPE     = 30.0f;
const float ANGLE_CORR_MAX = 3.0f;
const uint32_t PERIODE_US  = 5000; // 200 Hz

// ---------------- Variables de regulation ----------------
float Kp = CONF_Kp, Ki = CONF_Ki, Kd = CONF_Kd;
float consigne = CONF_CONSIGNE;
int   pwmMin   = CONF_PWM_MIN;
float Kp2      = CONF_Kp2, Ki2 = CONF_Ki2;
float V_DEADBAND = CONF_DEADBAND;
float gyroSign   = CONF_GYRO_SIGN;

float vitesseEstimee   = 0, correctionAngle = 0, consigneActive = CONF_CONSIGNE;
float rateF = 0, angleAcc = 0, angle = 0, rate = 0;
float integrale = 0, sortie = 0;
float gyroBias  = 0;

// Commandes issues du joystick
float correctionJoyCible = 0.0f;
float correctionJoy      = 0.0f; // Valeur lissee appliquee au PID
float turnOffsetCible    = 0.0f;
float turnOffset         = 0.0f;

// ---------------- Machine d'etats ----------------
enum Etat : uint8_t { ETAT_ATTENTE, ETAT_ACTIF };
Etat etat = ETAT_ATTENTE;

uint32_t tPrec = 0, tDernier = 0;
uint32_t tDernierEspNow = 0;
const uint32_t ESPNOW_TIMEOUT_MS = 500;
int compteurDebout = 0;
const int COMPTEUR_DEBOUT_CIBLE = CONF_DEBOUNCE_ARME_MS / 5; // e.g. 300 ms / 5 ms = 60 cycles

// ---------------- ESP-NOW ----------------
uint8_t macTelecommande[6] = CONF_MAC_TELE;

typedef struct {
  int P;
  int I;
  int D;
  int G;
  int EBtnAvD; // 1=relache, 0=appuye
  int EBtnAvG;
  int EBtnAvH;
  int EBtnArD;
  int EBtnArG;
  int JGX;     // -15 a +15
  int JGY;
  int JGBtn;   // 1=relache, 0=appuye
  int JDX;
  int JDY;
  int JDBtn;
} MessageTelecommande;

volatile bool newDataEspNow = false;
MessageTelecommande msgRecu, msgSafe;


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

void commandeMoteurs(int cmd, int turn = 0) {
  int vA = constrain(cmd + turn, -PWM_MAX, PWM_MAX);
  int vB = constrain(cmd - turn, -PWM_MAX, PWM_MAX);
  moteur(PWMA, AIN1, AIN2, SIGN_MOTEUR_A * vA);
  moteur(PWMB, BIN1, BIN2, SIGN_MOTEUR_B * vB);
}


// ============================================================
//                    CAPTEUR MPU-6050
// ============================================================
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
  Wire.read(); Wire.read(); // temperature
  int16_t gx = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); Wire.read(); Wire.read();
  (void)ax;
  accAngle = atan2((float)az, (float)ay) * RAD_TO_DEG;
  gyroRate = gyroSign * gx / 65.5f;
  return true;
}

void mpuInit() {
  mpuEcrire(0x6B, 0x00); // Reveil MPU
  mpuEcrire(0x1A, 0x03); // DLPF ~42 Hz
  mpuEcrire(0x1B, 0x08); // Gyro +/- 500 deg/s
  mpuEcrire(0x1C, 0x00); // Acc +/- 2g
}

void calibrationGyro() {
  commandeMoteurs(0);
  Serial.println("Calibration gyro : NE PAS BOUGER le robot...");
  float somme = 0; int n = 0; float a, g;
  for (int i = 0; i < 500; i++) {
    if (mpuLire(a, g)) { somme += g; n++; }
    delay(3);
  }
  gyroBias = (n > 0) ? somme / n : 0;
  Serial.printf("GyroBias = %.5f deg/s (n=%d)\n", gyroBias, n);
}


// ============================================================
//                    CALLBACK RECEPTION ESP-NOW
// ============================================================
void IRAM_ATTR onDataRecv(const esp_now_recv_info_t* info,
                          const uint8_t* data, int len) {
  if (len == (int)sizeof(MessageTelecommande)) {
    memcpy((void*)&msgRecu, data, sizeof(MessageTelecommande));
    newDataEspNow = true;
  }
}


// ============================================================
//                    INITIALISATION (SETUP)
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=========================================");
  Serial.println("=== RobotBalance V5 (ESP-NOW Autonome) ===");
  Serial.println("=========================================");

  // Broches moteurs
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
  commandeMoteurs(0);
  digitalWrite(STBY, HIGH);

  // I2C et Capteur IMU
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpuInit();
  delay(100);
  calibrationGyro();

  float a, g;
  if (mpuLire(a, g)) angle = a;

  // WiFi en mode Station pour ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CONF_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  // Affichage de la MAC du robot pour la telecommande
  uint8_t macRobot[6];
  esp_wifi_get_mac(WIFI_IF_STA, macRobot);
  Serial.println("\n----------------------------------------------------");
  Serial.printf(">>> MAC DU ROBOT : %02X:%02X:%02X:%02X:%02X:%02X <<<\n",
                macRobot[0], macRobot[1], macRobot[2], macRobot[3], macRobot[4], macRobot[5]);
  Serial.println("(A verifier dans la telecommande si le point n'est pas vert)");
  Serial.println("----------------------------------------------------\n");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERREUR: esp_now_init a echoue !");
  } else {
    esp_now_register_recv_cb(onDataRecv);

    // Activer Long Range si configure (protocole LR Espressif)
    if (CONF_WIFI_LONG_RANGE) {
      uint8_t protocole = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR;
      esp_wifi_set_protocol(WIFI_IF_STA, protocole);
      esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_LORA_250K);
      Serial.println("ESP-NOW : Mode Longue Portee (LR) active (250 kbps)");
    }

    // Enregistrer la telecommande pour que l'accuse de reception (ACK) fonctionne
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, macTelecommande, 6);
    peer.channel = CONF_WIFI_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
      Serial.printf("ESP-NOW : Telecommande (%02X:%02X:%02X:%02X:%02X:%02X) enregistree OK\n",
                    macTelecommande[0], macTelecommande[1], macTelecommande[2],
                    macTelecommande[3], macTelecommande[4], macTelecommande[5]);
    } else {
      Serial.println("AVERTISSEMENT : Ajout du pair telecommande a echoue");
    }
  }

  Serial.println("\n--- Parametres actifs ---");
  Serial.printf("PID Angle   : Kp=%.2f  Ki=%.2f  Kd=%.3f  Consigne=%.2f deg  PwmMin=%d\n",
                Kp, Ki, Kd, consigne, pwmMin);
  Serial.printf("PID Derive  : Kp2=%.3f  Deadband=%.1f\n", Kp2, V_DEADBAND);
  Serial.printf("Joystick    : VitesseMax=%.1f deg  SigneY=%.1f  TurnGain=%.1f\n",
                CONF_JOY_VITESSE_MAX, CONF_JOY_Y_SIGN, CONF_TURN_GAIN);
  Serial.printf("Securite    : Chute=%.1f deg  Reprise=%.1f deg  Debounce=%d ms\n",
                CONF_ANGLE_CHUTE, CONF_ANGLE_REPRISE, CONF_DEBOUNCE_ARME_MS);
  Serial.println("\nPret ! Maintenez le robot a la verticale pour l'armement automatique.\n");

  tPrec    = micros();
  tDernier = tPrec;
}


// ============================================================
//                    DECODAGE ESP-NOW & JOYSTICK
// ============================================================
void traiterEspNow() {
  noInterrupts();
  memcpy(&msgSafe, (const void*)&msgRecu, sizeof(MessageTelecommande));
  newDataEspNow = false;
  interrupts();

  tDernierEspNow = millis();

  // Choix du joystick (droit ou gauche)
  int valJoyX = CONF_USE_JOYSTICK_DROIT ? msgSafe.JDX : msgSafe.JGX;
  int valJoyY = CONF_USE_JOYSTICK_DROIT ? msgSafe.JDY : msgSafe.JGY;
  int btnJoy  = CONF_USE_JOYSTICK_DROIT ? msgSafe.JDBtn : msgSafe.JGBtn;

  // 1) Axe Y : Avancer / Reculer (-15 a +15)
  float jdy = (float)valJoyY;
  if (fabs(jdy) <= CONF_JOY_ZONE_MORTE_Y) {
    correctionJoyCible = 0.0f;
  } else {
    float norm = (jdy > 0)
      ? (jdy - CONF_JOY_ZONE_MORTE_Y) / (15.0f - CONF_JOY_ZONE_MORTE_Y)
      : (jdy + CONF_JOY_ZONE_MORTE_Y) / (15.0f - CONF_JOY_ZONE_MORTE_Y);
    // norm avec CONF_JOY_Y_SIGN applique : oriente la consigne vers l'avant ou l'arriere
    correctionJoyCible = norm * CONF_JOY_VITESSE_MAX * CONF_JOY_Y_SIGN;
  }

  // 2) Axe X : Tourner Gauche / Droite (-15 a +15)
  float jdx = (float)valJoyX;
  if (fabs(jdx) <= CONF_JOY_ZONE_MORTE_X) {
    turnOffsetCible = 0.0f;
  } else {
    float norm = (jdx > 0)
      ? (jdx - CONF_JOY_ZONE_MORTE_X) / (15.0f - CONF_JOY_ZONE_MORTE_X)
      : (jdx + CONF_JOY_ZONE_MORTE_X) / (15.0f - CONF_JOY_ZONE_MORTE_X);
    // Ecart PWM proportionnel au joystick
    turnOffsetCible = norm * CONF_TURN_GAIN * (float)pwmMin * CONF_TURN_SIGN;
  }

  // 3) Arret d'urgence manuel : Bouton Arriere Droit OU Clic sur Joystick (0 = appuye)
  if ((msgSafe.EBtnArD == 0 || btnJoy == 0) && etat == ETAT_ACTIF) {
    commandeMoteurs(0);
    etat = ETAT_ATTENTE;
    compteurDebout = 0;
    Serial.println("ARRET D'URGENCE (Bouton telecommande appuye) -> Desarme");
  }
}


// ============================================================
//                    BOUCLE PRINCIPALE (LOOP)
// ============================================================
void loop() {
  // Traitement ESP-NOW
  if (newDataEspNow) {
    traiterEspNow();
  }

  // Failsafe liaison : si aucun paquet recu depuis > 500 ms, remise au neutre du joystick
  if (millis() - tDernierEspNow > ESPNOW_TIMEOUT_MS) {
    correctionJoyCible = 0.0f;
    turnOffsetCible    = 0.0f;
  }

  // Cadencement strict a 200 Hz (5000 us)
  uint32_t now = micros();
  if ((uint32_t)(now - tPrec) < PERIODE_US) return;

  uint32_t dus = now - tDernier;
  tDernier     = now;
  tPrec       += PERIODE_US;
  if ((uint32_t)(now - tPrec) > PERIODE_US * 4UL) tPrec = now;

  float dt = ((dus > 20000UL) ? 20000UL : dus) / 1000000.0f;

  // Lissage progressif du joystick (filtre passe-bas : evite les a-coups et le decrochage)
  correctionJoy += ALPHA_JOY * (correctionJoyCible - correctionJoy);
  turnOffset    += ALPHA_JOY * (turnOffsetCible - turnOffset);

  // Lecture de la centrale inertielle
  float a, gr;
  if (mpuLire(a, gr)) {
    rate     = gr - gyroBias;
    angleAcc = a;
  }
  rateF = ALPHA_D * rate + (1.0f - ALPHA_D) * rateF;
  angle = ALPHA * (angle + rate * dt) + (1.0f - ALPHA) * angleAcc;

  // ---------------- Machine d'etats ----------------
  switch (etat) {

    // -- ETAT ATTENTE : Moteurs coupes, armement automatique --
    case ETAT_ATTENTE:
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

      // Condition d'armement : proche de la consigne et loin de l'angle de chute
      if (fabs(angle - consigne) < CONF_ANGLE_REPRISE && fabs(angle) < CONF_ANGLE_CHUTE) {
        compteurDebout++;
        if (compteurDebout >= COMPTEUR_DEBOUT_CIBLE) {
          etat = ETAT_ACTIF;
          compteurDebout = 0;
          integrale = 0;
          Serial.printf(">>> ARME AUTOMATIQUEMENT ! Angle=%.2f deg\n", angle);
        }
      } else {
        compteurDebout = 0;
      }
      break;

    // -- ETAT ACTIF : Asservissement PID + Conduite Joystick --
    case ETAT_ACTIF:
      // Securite chute immediate
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

      // Boucle 2 : Correction de derive lente avec deadband
      // Active uniquement lorsque le joystick est proche du neutre pour ne pas lutter contre le pilote
      if (fabs(correctionJoyCible) > 0.05f) {
        // En translation pilotee : on reinitialise la derive pour eviter le recul au lacher de stick
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

      // Consigne finale = Point d'equilibre - Anti-derive + Joystick Y lissé
      consigneActive = constrain(
          consigne - correctionAngle + correctionJoy,
          consigne - ANGLE_CORR_MAX - CONF_JOY_VITESSE_MAX,
          consigne + ANGLE_CORR_MAX + CONF_JOY_VITESSE_MAX
      );

      // Boucle 1 : PID de stabilisation sur l'angle
      float erreur = angle - consigneActive;
      integrale += Ki * erreur * dt;
      integrale  = constrain(integrale, -I_MAX, I_MAX);
      sortie     = Kp * erreur + integrale + Kd * rateF;
      sortie     = constrain(sortie, -PWM_MAX, PWM_MAX);

      // Rampe de demarrage pour vaincre le frottement statique
      float absS   = fabs(sortie);
      float offset = pwmMin * min(absS / ZONE_RAMPE, 1.0f);
      int   cmd    = (int)constrain(absS + offset, 0.0f, (float)PWM_MAX);
      if (sortie < 0) cmd = -cmd;

      // Joystick X : differentiel de virage
      int turn = (int)constrain(turnOffset, -(float)PWM_MAX, (float)PWM_MAX);

      commandeMoteurs(cmd, turn);
      break;
  }
}
