
/*
  Test encodeur incrémental - ESP32-C6

  Voie A -> GPIO 1
  Voie B -> GPIO 10
  GND encodeur -> GND ESP32
*/

const int ENCODER_A = 1;
const int ENCODER_B = 10;

volatile long compteur = 0;
volatile int dernierEtatA = 0;

void IRAM_ATTR encoderISR() {
  int etatA = digitalRead(ENCODER_A);
  int etatB = digitalRead(ENCODER_B);

  if (etatA != dernierEtatA) {
    if (etatA == etatB) {
      compteur++;
    } else {
      compteur--;
    }
    dernierEtatA = etatA;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  dernierEtatA = digitalRead(ENCODER_A);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_A),
    encoderISR,
    CHANGE
  );

  Serial.println("Test encodeur ESP32-C6");
  Serial.println("Tourne le moteur et observe les valeurs.");
}

void loop() {
  static long ancienCompteur = 0;
  static unsigned long dernierAffichage = 0;

  if (millis() - dernierAffichage >= 200) {
    dernierAffichage = millis();

    noInterrupts();
    long valeur = compteur;
    interrupts();

    Serial.print("Impulsions : ");
    Serial.print(valeur);
    Serial.print(" | A = ");
    Serial.print(digitalRead(ENCODER_A));
    Serial.print(" | B = ");
    Serial.print(digitalRead(ENCODER_B));

    if (valeur != ancienCompteur) {
      Serial.println(" | Encodeur actif");
    } else {
      Serial.println();
    }

    ancienCompteur = valeur;
  }
}
