// Test des moteurs - ESP32-C6 + TB6612FNG
// Moteurs 6V 300 rpm, lipo 7,2V -> PWM limité à 200/255

// ---- Broches driver ----
#define PWMA 11
#define AIN1 7
#define AIN2 10

#define PWMB 23
#define BIN1 21
#define BIN2 20

#define STBY 22

// ---- Réglages PWM ----
const int PWM_FREQ = 20000;  // 20 kHz (silencieux)
const int PWM_RES  = 8;      // 8 bits -> 0..255
const int PWM_MAX  = 200;    // limite à ne pas dépasser

// vitesse : -200 (arrière) ... 0 (stop) ... +200 (avant)
void moteurA(int vitesse) {
  vitesse = constrain(vitesse, -PWM_MAX, PWM_MAX); //limite de la vitesse 
  if (vitesse > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (vitesse < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  ledcWrite(PWMA, abs(vitesse));
}

void moteurB(int vitesse) {
  vitesse = constrain(vitesse, -PWM_MAX, PWM_MAX);  //limite de la vitesse
  if (vitesse > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else if (vitesse < 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  ledcWrite(PWMB, abs(vitesse));
}

void stopTout() {
  moteurA(0);
  moteurB(0);
}

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Core ESP32 Arduino 3.x : ledcAttach(pin, freq, résolution)
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);

  stopTout();
  digitalWrite(STBY, HIGH);  // active le driver

  Serial.println("Test moteurs : demarrage dans 3 s...");
  delay(3000);
}

void loop() {
  Serial.println("Moteur A avant");
  moteurA(PWM_MAX);
  delay(2000);
  stopTout();
  delay(1000);

  Serial.println("Moteur A arriere");
  moteurA(-PWM_MAX);
  delay(2000);
  stopTout();
  delay(1000);

  Serial.println("Moteur B avant");
  moteurB(PWM_MAX);
  delay(2000);
  stopTout();
  delay(1000);

  Serial.println("Moteur B arriere");
  moteurB(-PWM_MAX);
  delay(2000);
  stopTout();
  delay(1000);

  Serial.println("Les deux avant");
  moteurA(PWM_MAX);
  moteurB(PWM_MAX);
  delay(2000);
  stopTout();
  delay(1000);

  Serial.println("Les deux arriere");
  moteurA(-PWM_MAX);
  moteurB(-PWM_MAX);
  delay(2000);
  stopTout();
  delay(1000);

  Serial.println("Rampe progressive moteurs A+B");
  for (int v = 0; v <= PWM_MAX; v += 5) {
    moteurA(v);
    moteurB(v);
    delay(40);
  }
  delay(500);
  for (int v = PWM_MAX; v >= 0; v -= 5) {
    moteurA(v);
    moteurB(v);
    delay(40);
  }
  stopTout();

  Serial.println("--- Fin de cycle, pause 3 s ---");
  delay(3000);
}
