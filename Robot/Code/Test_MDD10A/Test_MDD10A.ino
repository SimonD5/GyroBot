
/*
  Test MDD10A REV 2.0 avec ESP32
  Deux moteurs synchronisés
  Rampe d'accélération et de décélération dans les deux sens

  DIR1 = GPIO 4
  PWM1 = GPIO 5
  DIR2 = GPIO 3
  PWM2 = GPIO 15
*/

const int DIR1 = 4;
const int PWM1 = 5;
const int DIR2 = 3;
const int PWM2 = 15;

// Paramètres de la rampe
const int PWM_MIN = 0;
const int PWM_MAX = 180;  // Puissance maximale (0 à 255)
const int PWM_STEP = 5;   // Incrément à chaque étape
const int STEP_DELAY = 100; // Délai en ms entre les étapes
const int PAUSE = 1000;     // Pause à l'arrêt

void stopMotors() {
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
}

void setMotors(int direction, int pwm) {
  // direction HIGH ou LOW pour les deux moteurs
  digitalWrite(DIR1, direction);
  digitalWrite(DIR2, direction);

  analogWrite(PWM1, pwm);
  analogWrite(PWM2, pwm);
}

void rampUp() {
  Serial.println("Acceleration...");

  for (int pwm = PWM_MIN; pwm <= PWM_MAX; pwm += PWM_STEP) {
    setMotors(HIGH, pwm);
    delay(STEP_DELAY);
  }
}

void rampDown() {
  Serial.println("Deceleration...");

  for (int pwm = PWM_MAX; pwm >= PWM_MIN; pwm -= PWM_STEP) {
    setMotors(HIGH, pwm);
    delay(STEP_DELAY);
  }

  stopMotors();
}

void rampUpReverse() {
  Serial.println("Acceleration sens inverse...");

  for (int pwm = PWM_MIN; pwm <= PWM_MAX; pwm += PWM_STEP) {
    setMotors(LOW, pwm);
    delay(STEP_DELAY);
  }
}

void rampDownReverse() {
  Serial.println("Deceleration sens inverse...");

  for (int pwm = PWM_MAX; pwm >= PWM_MIN; pwm -= PWM_STEP) {
    setMotors(LOW, pwm);
    delay(STEP_DELAY);
  }

  stopMotors();
}

void setup() {
  Serial.begin(115200);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  stopMotors();

  Serial.println("Test MDD10A : demarrage dans 2 secondes");
  delay(2000);
}

void loop() {
  // Sens 1
  Serial.println("\n=== SENS 1 ===");
  rampUp();

  delay(500);

  rampDown();
  delay(PAUSE);

  // Sens 2
  Serial.println("\n=== SENS 2 ===");
  rampUpReverse();

  delay(500);

  rampDownReverse();
  delay(PAUSE);
}
