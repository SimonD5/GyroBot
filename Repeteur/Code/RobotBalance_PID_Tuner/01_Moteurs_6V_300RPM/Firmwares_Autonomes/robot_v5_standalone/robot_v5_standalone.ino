// Robot balance v5 autonome (telecommande ESP-NOW)

#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>


// boucle pid angle
#define CONF_Kp          16.0f
#define CONF_Ki           1.0f
#define CONF_Kd           0.30f
#define CONF_CONSIGNE     0.5f
#define CONF_PWM_MIN     18

// boucle vitesse cascade
#define CONF_Kp2          0.25f
#define CONF_Ki2          0.0f
#define CONF_DEADBAND     2.0f

// joystick
#define CONF_USE_JOYSTICK_DROIT true

// axe y (avancer / reculer)
#define CONF_JOY_VITESSE_MAX   1.5f
#define CONF_JOY_ZONE_MORTE_Y  1.0f
#define CONF_JOY_Y_SIGN       -1.0f

// axe x (tourner gauche / droite)
#define CONF_TURN_GAIN         1.2f
#define CONF_JOY_ZONE_MORTE_X  1.0f
#define CONF_TURN_SIGN         1.0f

// mpu6050
#define CONF_GYRO_SIGN       -1.0f

// securite
#define CONF_ANGLE_CHUTE     18.0f
#define CONF_ANGLE_REPRISE    2.0f
#define CONF_DEBOUNCE_ARME_MS 300

// com sans fil
#define CONF_MAC_TELE        {0x44, 0xB1, 0x76, 0xCD, 0xD1, 0x38}
#define CONF_WIFI_CHANNEL    1
#define CONF_WIFI_LONG_RANGE true



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

float vitesseEstimee   = 0, correctionAngle = 0, consigneActive = CONF_CONSIGNE;
float rateF = 0, angleAcc = 0, angle = 0, rate = 0;
float integrale = 0, sortie = 0;
float gyroBias  = 0;

// commandes issues du joystick
float correctionJoyCible = 0.0f;
float correctionJoy      = 0.0f;
float turnOffsetCible    = 0.0f;
float turnOffset         = 0.0f;

enum Etat : uint8_t { ETAT_ATTENTE, ETAT_ACTIF };
Etat etat = ETAT_ATTENTE;

uint32_t tPrec = 0, tDernier = 0;
uint32_t tDernierEspNow = 0;
const uint32_t ESPNOW_TIMEOUT_MS = 500;
int compteurDebout = 0;
const int COMPTEUR_DEBOUT_CIBLE = CONF_DEBOUNCE_ARME_MS / 5;

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
  Serial.println("Calibration gyro : NE PAS BOUGER le robot...");
  float somme = 0; int n = 0; float a, g;
  for (int i = 0; i < 500; i++) {
    if (mpuLire(a, g)) { somme += g; n++; }
    delay(3);
  }
  gyroBias = (n > 0) ? somme / n : 0;
  Serial.printf("GyroBias = %.5f deg/s (n=%d)\n", gyroBias, n);
}


// callback reception esp-now
void IRAM_ATTR onDataRecv(const esp_now_recv_info_t* info,
                          const uint8_t* data, int len) {
  if (len == (int)sizeof(MessageTelecommande)) {
    memcpy((void*)&msgRecu, data, sizeof(MessageTelecommande));
    newDataEspNow = true;
  }
}


// initialisation (setup)
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=========================================");
  Serial.println("=== RobotBalance V5 (ESP-NOW Autonome) ===");
  Serial.println("=========================================");

// broches moteurs
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
  commandeMoteurs(0);
  digitalWrite(STBY, HIGH);

// i2c et capteur imu
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

// affichage de la mac du robot pour la telecommande
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

// activer long range si configure (protocole lr espressif)
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


// decodage esp-now & joystick
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

// 1) axe y : avancer / reculer (-15 a +15)
  float jdy = (float)valJoyY;
  if (fabs(jdy) <= CONF_JOY_ZONE_MORTE_Y) {
    correctionJoyCible = 0.0f;
  } else {
    float norm = (jdy > 0)
      ? (jdy - CONF_JOY_ZONE_MORTE_Y) / (15.0f - CONF_JOY_ZONE_MORTE_Y)
      : (jdy + CONF_JOY_ZONE_MORTE_Y) / (15.0f - CONF_JOY_ZONE_MORTE_Y);
    correctionJoyCible = norm * CONF_JOY_VITESSE_MAX * CONF_JOY_Y_SIGN;
  }

// 2) axe x : tourner gauche / droite (-15 a +15)
  float jdx = (float)valJoyX;
  if (fabs(jdx) <= CONF_JOY_ZONE_MORTE_X) {
    turnOffsetCible = 0.0f;
  } else {
    float norm = (jdx > 0)
      ? (jdx - CONF_JOY_ZONE_MORTE_X) / (15.0f - CONF_JOY_ZONE_MORTE_X)
      : (jdx + CONF_JOY_ZONE_MORTE_X) / (15.0f - CONF_JOY_ZONE_MORTE_X);
// ecart pwm proportionnel au joystick
    turnOffsetCible = norm * CONF_TURN_GAIN * (float)pwmMin * CONF_TURN_SIGN;
  }

  if ((msgSafe.EBtnArD == 0 || btnJoy == 0) && etat == ETAT_ACTIF) {
    commandeMoteurs(0);
    etat = ETAT_ATTENTE;
    compteurDebout = 0;
    Serial.println("ARRET D'URGENCE (Bouton telecommande appuye) -> Desarme");
  }
}


// boucle principale (loop)
void loop() {
// traitement esp-now
  if (newDataEspNow) {
    traiterEspNow();
  }

  if (millis() - tDernierEspNow > ESPNOW_TIMEOUT_MS) {
    correctionJoyCible = 0.0f;
    turnOffsetCible    = 0.0f;
  }

// cadencement strict a 200 hz (5000 us)
  uint32_t now = micros();
  if ((uint32_t)(now - tPrec) < PERIODE_US) return;

  uint32_t dus = now - tDernier;
  tDernier     = now;
  tPrec       += PERIODE_US;
  if ((uint32_t)(now - tPrec) > PERIODE_US * 4UL) tPrec = now;

  float dt = ((dus > 20000UL) ? 20000UL : dus) / 1000000.0f;

  correctionJoy += ALPHA_JOY * (correctionJoyCible - correctionJoy);
  turnOffset    += ALPHA_JOY * (turnOffsetCible - turnOffset);

// lecture de la centrale inertielle
  float a, gr;
  if (mpuLire(a, gr)) {
    rate     = gr - gyroBias;
    angleAcc = a;
  }
  rateF = ALPHA_D * rate + (1.0f - ALPHA_D) * rateF;
  angle = ALPHA * (angle + rate * dt) + (1.0f - ALPHA) * angleAcc;

  switch (etat) {

// armement
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

// armement
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

// -- etat actif : asservissement pid + conduite joystick --
    case ETAT_ACTIF:
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

// boucle 2 : correction de derive lente avec deadband
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
          consigne - ANGLE_CORR_MAX - CONF_JOY_VITESSE_MAX,
          consigne + ANGLE_CORR_MAX + CONF_JOY_VITESSE_MAX
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
