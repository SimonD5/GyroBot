// Telecommande Gyrobot v7 (ESP32-C6 + ecran ST7735 + ESP-NOW)

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <Preferences.h>
// ecran
const uint8_t TFT_SCK   = 2;
const uint8_t TFT_MOSI  = 42;
const uint8_t TFT_CS    = 39;
const uint8_t TFT_DC    = 41;
const uint8_t TFT_RST   = 40;
const uint8_t TFT_LED   = 1;
// bouton
const uint8_t BtnArD = 38;
const uint8_t BtnArG = 16;
const uint8_t BtnAvG = 21;
const uint8_t BtnAvD = 12;
const uint8_t BtnAvH = 48;
// potentiometre
const uint8_t PotG = 7;
const uint8_t PotP = 8;
const uint8_t PotI = 3;
const uint8_t PotD = 4;
// led
const uint8_t LED = 17;
// joystick gauche
const uint8_t JoyG_X = 6;
const uint8_t JoyG_Y = 5;
const uint8_t JoyG_btn = 11;
// joystick droit
const uint8_t JoyD_X = 9;
const uint8_t JoyD_Y = 10;
const uint8_t JoyD_btn = 18;
// variable
bool Verif_Capteur = true;
int P = 0;
int I = 1;
int D = 2;
int G = 3;
int EBtnAvD = -1;
int EBtnAvG = -1;
int EBtnAvH = -1;
int EBtnArD = -1;
int EBtnArG = -1;
int JGX = -1;
int JGY = -1;
int JGBtn = -1;
int JDX = -1;
int JDY = -1;
int JDBtn = -1;

// com sans fil
uint8_t wifiChannel = 1;
bool wifiLongRange = true;
int indexPuissance = 0;

Preferences preferences;
const char* PREFS_NAMESPACE = "config";

// com sans fil
enum EtatLiaison { LIAISON_INACTIVE, LIAISON_SANS_ACCUSE, LIAISON_OK };
EtatLiaison etatLiaison = LIAISON_INACTIVE;
float tauxReussiteEnvoi = 1.0;
const float SEUIL_TAUX_LIAISON_OK = 0.85;

const int NB_SECONDES_STATS = 60;
int statsEnvoyesParSeconde[NB_SECONDES_STATS] = {0};
int statsPerdusParSeconde[NB_SECONDES_STATS] = {0};
int indexSecondeCourante = 0;
unsigned long debutSecondeCourante = 0;

Adafruit_ST7735 tft = Adafruit_ST7735(&SPI, TFT_CS, TFT_DC, TFT_RST);
// esp now
uint8_t adresseBroadcast[] = {0x44, 0xB1, 0x76 , 0xCD , 0xD5 , 0x80};
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
} Message;
Message donneesEnvoyees;
void onEnvoiTermine(const uint8_t *mac_addr, esp_now_send_status_t status) {
  bool succes = (status == ESP_NOW_SEND_SUCCESS);
  Serial.println(succes ? "Envoi OK" : "Envoi ECHEC");

  tauxReussiteEnvoi = tauxReussiteEnvoi * 0.9f + (succes ? 1.0f : 0.0f) * 0.1f;
  etatLiaison = (tauxReussiteEnvoi >= SEUIL_TAUX_LIAISON_OK) ? LIAISON_OK : LIAISON_SANS_ACCUSE;

  if (!succes) {
    statsPerdusParSeconde[indexSecondeCourante]++;
  }
}
bool envoyerDonnees(const uint8_t *macDestinataire, const Message &msg) {
  esp_err_t resultat = esp_now_send(macDestinataire, (uint8_t *)&msg, sizeof(msg));
  return resultat == ESP_OK;
}

void mettreAJourFenetreStats() {
  unsigned long maintenant = millis();
  if (maintenant - debutSecondeCourante >= 1000) {
    debutSecondeCourante = maintenant;
    indexSecondeCourante = (indexSecondeCourante + 1) % NB_SECONDES_STATS;
    statsEnvoyesParSeconde[indexSecondeCourante] = 0;
    statsPerdusParSeconde[indexSecondeCourante] = 0;
  }
}

void calculerStatsPerte(int &totalEnvoyes, int &totalPerdus) {
  totalEnvoyes = 0;
  totalPerdus = 0;
  for (int i = 0; i < NB_SECONDES_STATS; i++) {
    totalEnvoyes += statsEnvoyesParSeconde[i];
    totalPerdus += statsPerdusParSeconde[i];
  }
}

// com sans fil
struct PuissanceWifi {
  wifi_power_t valeur;
  const char* label;
};

const PuissanceWifi tablePuissance[] = {
  {WIFI_POWER_19_5dBm,    "19.5 dBm"},
  {WIFI_POWER_19dBm,      "19 dBm"},
  {WIFI_POWER_18_5dBm,    "18.5 dBm"},
  {WIFI_POWER_17dBm,      "17 dBm"},
  {WIFI_POWER_15dBm,      "15 dBm"},
  {WIFI_POWER_13dBm,      "13 dBm"},
  {WIFI_POWER_11dBm,      "11 dBm"},
  {WIFI_POWER_8_5dBm,     "8.5 dBm"},
  {WIFI_POWER_7dBm,       "7 dBm"},
  {WIFI_POWER_5dBm,       "5 dBm"},
  {WIFI_POWER_2dBm,       "2 dBm"},
  {WIFI_POWER_MINUS_1dBm, "-1 dBm"},
};
const int NB_PUISSANCES = sizeof(tablePuissance) / sizeof(tablePuissance[0]);

// com sans fil
void appliquerCanalWifi(uint8_t canal) {
  esp_wifi_set_channel(canal, WIFI_SECOND_CHAN_NONE);

  esp_now_del_peer(adresseBroadcast);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, adresseBroadcast, 6);
  peerInfo.channel = canal;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erreur mise a jour peer (canal)");
  }
}

void appliquerLonguePortee(bool activer) {
  uint8_t protocole = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
  if (activer) {
    protocole |= WIFI_PROTOCOL_LR;
  }
  esp_wifi_set_protocol(WIFI_IF_STA, protocole);

// ci-dessus n'a aucun effet reel sur les paquets esp-now.
  if (activer) {
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_LORA_250K);
  } else {
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);
  }
}

void appliquerPuissanceWifi(int index) {
  WiFi.setTxPower(tablePuissance[index].valeur);
}

void chargerConfiguration() {
  preferences.begin(PREFS_NAMESPACE, true);

  wifiChannel = preferences.getUChar("canal", wifiChannel);
  wifiLongRange = preferences.getBool("lr", wifiLongRange);
  indexPuissance = preferences.getInt("puiss", indexPuissance);
  if (indexPuissance < 0 || indexPuissance >= NB_PUISSANCES) {
    indexPuissance = 0;
  }

  uint8_t adresseLue[6];
  size_t nbOctetsLus = preferences.getBytes("adresse", adresseLue, sizeof(adresseLue));
  if (nbOctetsLus == sizeof(adresseLue)) {
    memcpy(adresseBroadcast, adresseLue, sizeof(adresseBroadcast));
  }

  preferences.end();
}

void sauvegarderCanal(uint8_t canal) {
  preferences.begin(PREFS_NAMESPACE, false);
  preferences.putUChar("canal", canal);
  preferences.end();
}

void sauvegarderLonguePortee(bool activer) {
  preferences.begin(PREFS_NAMESPACE, false);
  preferences.putBool("lr", activer);
  preferences.end();
}

void sauvegarderPuissance(int index) {
  preferences.begin(PREFS_NAMESPACE, false);
  preferences.putInt("puiss", index);
  preferences.end();
}

void sauvegarderAdresse(const uint8_t *adresse) {
  preferences.begin(PREFS_NAMESPACE, false);
  preferences.putBytes("adresse", adresse, 6);
  preferences.end();
}

// affichage ecran
#define BG_COLOR      ST77XX_BLACK
#define ROBOT_COLOR   ST77XX_WHITE
#define ACCENT_COLOR  ST77XX_CYAN
#define TEXT_COLOR    ST77XX_WHITE
#define ROBOT_X 6
#define ROBOT_Y 8
#define ROBOT_W 74
#define INFO_X   92
#define SPEED_Y  14
#define BATT_Y   76

const int16_t STATUT_X = 7;
const int16_t STATUT_Y = 7;
const int16_t STATUT_R = 3;
const uint16_t COULEUR_STATUT_VERT   = 0x07E0;
const uint16_t COULEUR_STATUT_ORANGE = 0xFC00;
EtatLiaison dernierEtatPointPrincipal = LIAISON_INACTIVE;

void dessinerPointStatut(EtatLiaison etat) {
  uint16_t couleur;
  switch (etat) {
    case LIAISON_OK:          couleur = COULEUR_STATUT_VERT;   break;
    case LIAISON_SANS_ACCUSE: couleur = COULEUR_STATUT_ORANGE; break;
    default:                  couleur = ST77XX_RED;            break;
  }
  tft.fillCircle(STATUT_X, STATUT_Y, STATUT_R, couleur);
}

void mettreAJourPointStatutPrincipal() {
  if (etatLiaison != dernierEtatPointPrincipal) {
    dernierEtatPointPrincipal = etatLiaison;
    dessinerPointStatut(etatLiaison);
  }
}

const int16_t DIAG_BTN_W    = 26;
const int16_t DIAG_BTN_H    = 12;
const int16_t DIAG_ARG_X    = 4;
const int16_t DIAG_ARD_X    = 130;
const int16_t DIAG_BTN_TOP_Y = 16;

const int16_t DIAG_JOYG_CX = 25;
const int16_t DIAG_JOYD_CX = 135;
const int16_t DIAG_JOY_CY  = 50;
const int16_t DIAG_JOY_R   = 14;
const int16_t DIAG_JOY_DOT_R = 3;

const int16_t DIAG_AVH_X = 135, DIAG_AVH_Y = 86;
const int16_t DIAG_AVG_X = 120, DIAG_AVG_Y = 104;
const int16_t DIAG_AVD_X = 150, DIAG_AVD_Y = 104;
const int16_t DIAG_TRI_R = 7;

const int16_t DIAG_POT_TOP_Y  = 36;
const int16_t DIAG_POT_BAR_H  = 60;
const int16_t DIAG_POT_BAR_W  = 10;
const int16_t DIAG_POTP_X = 55;
const int16_t DIAG_POTI_X = 71;
const int16_t DIAG_POTD_X = 87;

const int16_t DIAG_POTG_CX = 25;
const int16_t DIAG_POTG_CY = 94;
const int16_t DIAG_POTG_R  = 13;

// reception données
int vitesse  = 12;
int batterie = 78;


int mapJoystickAxis(int valeur, int centre, int minBrut, int maxBrut, int sortieMin, int sortieMax, int zoneMorte = 200) {
// zone morte
  if (abs(valeur - centre) <= zoneMorte) {
    return 0;
  }

  if (valeur < centre) {
    return map(valeur, minBrut, centre, sortieMin, 0);
  } else {
    return map(valeur, centre, maxBrut, 0, sortieMax);
  }
}

// sortie > 0 = haut/droite, sortie < 0 = bas/gauche
const int SEUIL_JOYSTICK = 13;

bool joystickHaut() {
  int yG = mapJoystickAxis(analogRead(JoyG_Y), 1935, 0, 4095, -15, 15);
  int yD = mapJoystickAxis(analogRead(JoyD_Y), 1946, 0, 4095, 15, -15);
  return (yG > SEUIL_JOYSTICK) || (yD > SEUIL_JOYSTICK);
}

bool joystickBas() {
  int yG = mapJoystickAxis(analogRead(JoyG_Y), 1935, 0, 4095, -15, 15);
  int yD = mapJoystickAxis(analogRead(JoyD_Y), 1946, 0, 4095, 15, -15);
  return (yG < -SEUIL_JOYSTICK) || (yD < -SEUIL_JOYSTICK);
}

bool joystickDroite() {
  int xG = mapJoystickAxis(analogRead(JoyG_X), 1846, 0, 4095, 15, -15);
  int xD = mapJoystickAxis(analogRead(JoyD_X), 1925, 0, 4095, -15, 15);
  return (xG > SEUIL_JOYSTICK) || (xD > SEUIL_JOYSTICK);
}

bool joystickGauche() {
  int xG = mapJoystickAxis(analogRead(JoyG_X), 1846, 0, 4095, 15, -15);
  int xD = mapJoystickAxis(analogRead(JoyD_X), 1925, 0, 4095, -15, 15);
  return (xG < -SEUIL_JOYSTICK) || (xD < -SEUIL_JOYSTICK);
}

const int HYSTERESIS_POT = 25;
int brutPrecedentP = -1;
int brutPrecedentI = -1;
int brutPrecedentD = -1;
int brutPrecedentG = -1;

int lireAvecHysteresis(uint8_t broche, int &derniereValeurBrute) {
  int brut = analogRead(broche);
  if (derniereValeurBrute < 0 || abs(brut - derniereValeurBrute) >= HYSTERESIS_POT) {
    derniereValeurBrute = brut;
  }
  return derniereValeurBrute;
}

void setup() {
  Serial.begin(115200);
// led
  pinMode(LED, OUTPUT);
  analogWrite(LED, 10);
// bouton
  pinMode(BtnArD, INPUT_PULLDOWN);
  pinMode(BtnArG, INPUT_PULLDOWN);
  pinMode(BtnAvG, INPUT_PULLUP);
  pinMode(BtnAvD, INPUT_PULLUP);
  pinMode(BtnAvH, INPUT_PULLUP);
// potentiometre
  pinMode(PotG, INPUT);
  pinMode(PotP, INPUT);
  pinMode(PotI, INPUT);
  pinMode(PotD, INPUT);
// joystick gauche
  pinMode(JoyG_btn, INPUT_PULLUP);
  pinMode(JoyG_Y, INPUT);
  pinMode(JoyG_X, INPUT);
// joystick droit
  pinMode(JoyD_btn, INPUT_PULLUP);
  pinMode(JoyD_Y, INPUT);
  pinMode(JoyD_X, INPUT);

// ecran
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);

  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(BG_COLOR);

  chargerConfiguration();

// esp now
  WiFi.mode(WIFI_STA);

// wifi
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  appliquerPuissanceWifi(indexPuissance);

  debutSecondeCourante = millis();

  Serial.print("Adresse MAC de ce module : ");
  Serial.println(WiFi.macAddress());
  Serial.print("Canal WiFi actuel : ");
  Serial.println(WiFi.channel());
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erreur init ESP-NOW");
    return;
  }
  esp_now_register_send_cb(onEnvoiTermine);

  appliquerLonguePortee(wifiLongRange);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, adresseBroadcast, 6);
  peerInfo.channel = wifiChannel;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erreur ajout peer broadcast");
  }

// boot sequence;
  const char* messages[] = {
    "Demarrage systeme...",
    "Ne rien toucher",
    "Verif capteurs...",
  };
  int n = sizeof(messages) / sizeof(messages[0]);

  tft.setTextColor(TEXT_COLOR);
  tft.setTextSize(1);
  int y = 10;
  for (int i = 0; i < n; i++) {
    tft.setCursor(6, y);
    tft.println(messages[i]);
    y += 12;
    delay(400);
  }
  Verification_Capteur();
  if (Verif_Capteur) {
    tft.setCursor(6, y);
    tft.println("OK");
  }
  else {
    tft.setCursor(6, y);
    tft.println("HS");
    delay(10000);
  }
  delay(800);
  tft.fillScreen(BG_COLOR);
  drawMainScreen();
}

void loop() {
  if (digitalRead(BtnAvD) == 0) {
    showMenu();
  }
  P = lireAvecHysteresis(PotP, brutPrecedentP);
  I = lireAvecHysteresis(PotI, brutPrecedentI);
  D = lireAvecHysteresis(PotD, brutPrecedentD);
  G = lireAvecHysteresis(PotG, brutPrecedentG);
  EBtnAvD = digitalRead(BtnAvD);
  EBtnAvG = digitalRead(BtnAvG);
  EBtnAvH = digitalRead(BtnAvH);
  EBtnArD = digitalRead(BtnArD);
  EBtnArG = digitalRead(BtnArG);
  JGX = analogRead(JoyG_X);
  JGY = analogRead(JoyG_Y);
  JGBtn = digitalRead(JoyG_btn);
  JDX = analogRead(JoyD_X);
  JDY = analogRead(JoyD_Y);
  JDBtn = digitalRead(JoyD_btn);

  donneesEnvoyees.P  = map(P, 0, 4095, 0, 30);
  donneesEnvoyees.I = map(I, 0, 4095, 0, 30);
  donneesEnvoyees.D = map(D, 0, 4095, 0, 30);
  donneesEnvoyees.G = map(G, 4095, 0, 0, 30);
  donneesEnvoyees.EBtnAvD = EBtnAvD;
  donneesEnvoyees.EBtnAvG = EBtnAvG;
  donneesEnvoyees.EBtnAvH = EBtnAvH;
  donneesEnvoyees.EBtnArD = 1 - EBtnArD;
  donneesEnvoyees.EBtnArG = 1 - EBtnArG;
  donneesEnvoyees.JGX   = mapJoystickAxis(JGX, 1846, 0, 4095, 15, -15);
  donneesEnvoyees.JGY   = mapJoystickAxis(JGY, 1935, 0, 4095, 15, -15);
  donneesEnvoyees.JGBtn = JGBtn;
  donneesEnvoyees.JDX   = mapJoystickAxis(JDX, 1925, 0, 4095, -15, 15);
  donneesEnvoyees.JDY   = mapJoystickAxis(JDY, 1946, 0, 4095, -15, 15);
  donneesEnvoyees.JDBtn = JDBtn;

  mettreAJourFenetreStats();
  statsEnvoyesParSeconde[indexSecondeCourante]++;

  envoyerDonnees(adresseBroadcast, donneesEnvoyees);

  mettreAJourPointStatutPrincipal();

  Serial.println("");
  Serial.print("P:");
  Serial.print(P);
  Serial.print("I:");
  Serial.print(I);
  Serial.print("D:");
  Serial.print(D);
  Serial.print("G:");
  Serial.print(G);
  Serial.print("EBtnAvD:");
  Serial.print(EBtnAvD);
  Serial.print("EBtnAvG:");
  Serial.print(EBtnAvG);
  Serial.print("EBtnAvH:");
  Serial.print(EBtnAvH);
  Serial.print("EBtnArD:");
  Serial.print(EBtnArD);
  Serial.print("EBtnArG:");
  Serial.print(EBtnArG);
  Serial.print("JGX:");
  Serial.print(JGX);
  Serial.print("JGY:");
  Serial.print(JGY);
  Serial.print("JGBtn:");
  Serial.print(JGBtn);
  Serial.print("JDX:");
  Serial.print(JDX);
  Serial.print("JDY:");
  Serial.print(JDY);
  Serial.print("JDBtn:");
  Serial.println(JDBtn);
  Serial.println();
  delay(10);
}

// fonction de verification des capteurs
void Verification_Capteur() {
  if (digitalRead(BtnAvG) != 1) {
    Verif_Capteur = false;
    Serial.println("BtnAvG HS");
  }
  if (digitalRead(BtnAvD) != 1) {
    Verif_Capteur = false;
// serial.println("btnavd hs");
  }
  if (digitalRead(BtnAvH) != 1) {
    Verif_Capteur = false;
// serial.println("btnavh hs");
  }
  if (digitalRead(BtnArG) != 0) {
    Verif_Capteur = false;
// serial.println("btnarg hs");
  }
  if (digitalRead(BtnArD) != 0) {
    Verif_Capteur = false;
// serial.println("btnard hs");
  }
  if (digitalRead(JoyG_btn) != 1) {
    Verif_Capteur = false;
// serial.println("joyg_btn hs");
  }
  if (digitalRead(JoyD_btn) != 1) {
    Verif_Capteur = false;
// serial.println("joyd_btn hs");
  }
  if (analogRead(JoyG_X) > 1846 + 150 || analogRead(JoyG_X) < 1846 - 150 || analogRead(JoyG_Y) > 1935 + 150 || analogRead(JoyG_Y) < 1935 - 150) {
    Verif_Capteur = false;
// serial.println("joyg_x hs");
  }
  if (analogRead(JoyD_X) > 1925 + 150 || analogRead(JoyD_X) < 1925 - 150 || analogRead(JoyD_Y) > 1946 + 150 || analogRead(JoyD_Y) < 1946 - 150) {
    Verif_Capteur = false;
// serial.println("joyd_x hs");
  }
}


// affichage ecran
void showMenu() {
  etatLiaison = LIAISON_INACTIVE;

  const char* items[] = {
    "1. Diagnostic",
    "2. Reglages",
    "3. WiFi",
    "4. Quitter",
  };

  int n = sizeof(items) / sizeof(items[0]);
  int selected = 0;

  afficher_Menu(selected, n);
  unsigned long lastActivity = millis();

  while (millis() - lastActivity < 7000) {

    if (digitalRead(BtnAvG) == 0 || joystickBas()) {
      selected++;
      if (selected >= n) selected = 0;
      lastActivity = millis();
      afficher_Menu(selected, n);
      delay(200);
    }

    if (digitalRead(BtnAvH) == 0 || joystickHaut()) {
      selected--;
      if (selected < 0) selected = n - 1;
      lastActivity = millis();
      afficher_Menu(selected, n);
      delay(200);
    }

    if (digitalRead(BtnAvD) == 0 || joystickDroite()) {
      if (selected == 0) {
        diagnostic();
      }
      else if (selected == 1) {
        reglages();
      }
      else if (selected == 2) {
        menuWifi();
      }
// selected == 3 (quitter) : rien de plus a faire
      drawMainScreen();
      return;
    }

    if (joystickGauche()) {
// equivalent a "4. quitter"
      drawMainScreen();
      return;
    }

    delay(20);
  }

  drawMainScreen();
}

void diagnostic() {
  tft.fillScreen(BG_COLOR);
  tft.setTextColor(ACCENT_COLOR);
  tft.setTextSize(1);
  tft.setCursor(16, 2);
  tft.println("== DIAGNOSTIC ==");

  drawDiagStatic();

  delay(200);

  bool quitter = false;
  while (!quitter) {
    updateDiagButtons();
    updateDiagJoysticks();
    updateDiagPots();

    if (digitalRead(BtnAvD) == 0) {
      quitter = true;
    }
    delay(30);
  }
  while (digitalRead(BtnAvD) == 0) {
    delay(20);
  }
}
void drawDiagStatic() {
  tft.setTextColor(TEXT_COLOR);
  tft.setTextSize(1);

// boutons arriere
  tft.drawRect(DIAG_ARG_X, DIAG_BTN_TOP_Y, DIAG_BTN_W, DIAG_BTN_H, TEXT_COLOR);
  tft.setCursor(DIAG_ARG_X, DIAG_BTN_TOP_Y + DIAG_BTN_H + 2);
  tft.print("ArG");

  tft.drawRect(DIAG_ARD_X, DIAG_BTN_TOP_Y, DIAG_BTN_W, DIAG_BTN_H, TEXT_COLOR);
  tft.setCursor(DIAG_ARD_X, DIAG_BTN_TOP_Y + DIAG_BTN_H + 2);
  tft.print("ArD");

// joysticks
  tft.drawCircle(DIAG_JOYG_CX, DIAG_JOY_CY, DIAG_JOY_R, TEXT_COLOR);
  tft.setCursor(DIAG_JOYG_CX - 12, DIAG_JOY_CY + DIAG_JOY_R + 2);
  tft.print("JoyG");

  tft.drawCircle(DIAG_JOYD_CX, DIAG_JOY_CY, DIAG_JOY_R, TEXT_COLOR);
  tft.setCursor(DIAG_JOYD_CX - 12, DIAG_JOY_CY + DIAG_JOY_R + 2);
  tft.print("JoyD");

// boutons avant en triangle (haut / gauche / droite)
  tft.drawCircle(DIAG_AVH_X, DIAG_AVH_Y, DIAG_TRI_R, TEXT_COLOR);
  tft.drawCircle(DIAG_AVG_X, DIAG_AVG_Y, DIAG_TRI_R, TEXT_COLOR);
  tft.drawCircle(DIAG_AVD_X, DIAG_AVD_Y, DIAG_TRI_R, TEXT_COLOR);

// potentiometres lineaires p, i, d
  tft.drawRect(DIAG_POTP_X, DIAG_POT_TOP_Y, DIAG_POT_BAR_W, DIAG_POT_BAR_H, TEXT_COLOR);
  tft.setCursor(DIAG_POTP_X + 2, DIAG_POT_TOP_Y + DIAG_POT_BAR_H + 2);
  tft.print("P");

  tft.drawRect(DIAG_POTI_X, DIAG_POT_TOP_Y, DIAG_POT_BAR_W, DIAG_POT_BAR_H, TEXT_COLOR);
  tft.setCursor(DIAG_POTI_X + 2, DIAG_POT_TOP_Y + DIAG_POT_BAR_H + 2);
  tft.print("I");

  tft.drawRect(DIAG_POTD_X, DIAG_POT_TOP_Y, DIAG_POT_BAR_W, DIAG_POT_BAR_H, TEXT_COLOR);
  tft.setCursor(DIAG_POTD_X + 2, DIAG_POT_TOP_Y + DIAG_POT_BAR_H + 2);
  tft.print("D");

// potentiometre rotatif
  tft.drawCircle(DIAG_POTG_CX, DIAG_POTG_CY, DIAG_POTG_R, TEXT_COLOR);
  tft.setCursor(DIAG_POTG_CX - 3, DIAG_POTG_CY + DIAG_POTG_R + 2);
  tft.print("G");

  tft.setTextColor(ACCENT_COLOR);
  tft.setCursor(4, 120);
  tft.print("AvD=Quitter");

  dessinerPointStatut(etatLiaison);
}

void updateDiagButtons() {
  bool pArG = (digitalRead(BtnArG) == 1);
  bool pArD = (digitalRead(BtnArD) == 1);
  bool pAvH = (digitalRead(BtnAvH) == 0);
  bool pAvG = (digitalRead(BtnAvG) == 0);
  bool pAvD = (digitalRead(BtnAvD) == 0);

  tft.fillRect(DIAG_ARG_X + 1, DIAG_BTN_TOP_Y + 1, DIAG_BTN_W - 2, DIAG_BTN_H - 2, pArG ? ROBOT_COLOR : BG_COLOR);
  tft.fillRect(DIAG_ARD_X + 1, DIAG_BTN_TOP_Y + 1, DIAG_BTN_W - 2, DIAG_BTN_H - 2, pArD ? ROBOT_COLOR : BG_COLOR);

  tft.fillCircle(DIAG_AVH_X, DIAG_AVH_Y, DIAG_TRI_R - 1, pAvH ? ROBOT_COLOR : BG_COLOR);
  tft.fillCircle(DIAG_AVG_X, DIAG_AVG_Y, DIAG_TRI_R - 1, pAvG ? ROBOT_COLOR : BG_COLOR);
  tft.fillCircle(DIAG_AVD_X, DIAG_AVD_Y, DIAG_TRI_R - 1, pAvD ? ROBOT_COLOR : BG_COLOR);
}

void updateDiagJoysticks() {
  int maxOff = DIAG_JOY_R - DIAG_JOY_DOT_R - 2;

// joystick
  tft.fillCircle(DIAG_JOYG_CX, DIAG_JOY_CY, DIAG_JOY_R - 1, BG_COLOR);

  int gx = map(analogRead(JoyG_X), 0, 4095, maxOff, -maxOff);
  int gy = map(analogRead(JoyG_Y), 0, 4095, maxOff, -maxOff);

  tft.fillCircle(
    DIAG_JOYG_CX + gx,
    DIAG_JOY_CY + gy,
    DIAG_JOY_DOT_R,
    ACCENT_COLOR
  );

// appui joystick gauche
  bool joyGPressed = (digitalRead(JoyG_btn) == 0);

  if (joyGPressed) {
    tft.fillCircle(DIAG_JOYG_CX, DIAG_JOY_CY, 4, ROBOT_COLOR);
  }

// joystick
  tft.fillCircle(DIAG_JOYD_CX, DIAG_JOY_CY, DIAG_JOY_R - 1, BG_COLOR);

  int dx = map(analogRead(JoyD_X), 0, 4095, -maxOff, maxOff);
  int dy = map(analogRead(JoyD_Y), 0, 4095, -maxOff, maxOff);

  tft.fillCircle(
    DIAG_JOYD_CX + dx,
    DIAG_JOY_CY + dy,
    DIAG_JOY_DOT_R,
    ACCENT_COLOR
  );

// appui joystick droit
  bool joyDPressed = (digitalRead(JoyD_btn) == 0);

  if (joyDPressed) {
    tft.fillCircle(DIAG_JOYD_CX, DIAG_JOY_CY, 4, ROBOT_COLOR);
  }
}

void updateDiagPots() {
  int barInnerH = DIAG_POT_BAR_H - 2;

// potentiometres lineaires (0=petit, 4095=grand)
  tft.fillRect(DIAG_POTP_X + 1, DIAG_POT_TOP_Y + 1, DIAG_POT_BAR_W - 2, barInnerH, BG_COLOR);
  int fhP = map(analogRead(PotP), 0, 4095, 0, barInnerH);
  tft.fillRect(DIAG_POTP_X + 1, DIAG_POT_TOP_Y + 1 + (barInnerH - fhP), DIAG_POT_BAR_W - 2, fhP, ACCENT_COLOR);

  tft.fillRect(DIAG_POTI_X + 1, DIAG_POT_TOP_Y + 1, DIAG_POT_BAR_W - 2, barInnerH, BG_COLOR);
  int fhI = map(analogRead(PotI), 0, 4095, 0, barInnerH);
  tft.fillRect(DIAG_POTI_X + 1, DIAG_POT_TOP_Y + 1 + (barInnerH - fhI), DIAG_POT_BAR_W - 2, fhI, ACCENT_COLOR);

  tft.fillRect(DIAG_POTD_X + 1, DIAG_POT_TOP_Y + 1, DIAG_POT_BAR_W - 2, barInnerH, BG_COLOR);
  int fhD = map(analogRead(PotD), 0, 4095, 0, barInnerH);
  tft.fillRect(DIAG_POTD_X + 1, DIAG_POT_TOP_Y + 1 + (barInnerH - fhD), DIAG_POT_BAR_W - 2, fhD, ACCENT_COLOR);

// potentiometre rotatif (4095=petit, 0=grand)
  tft.fillCircle(DIAG_POTG_CX, DIAG_POTG_CY, DIAG_POTG_R - 2, BG_COLOR);
  float angleDeg = map(analogRead(PotG), 0, 4095, 135, -135);
  float angleRad = angleDeg * PI / 180.0;
  int nx = DIAG_POTG_CX + (int)((DIAG_POTG_R - 3) * sin(angleRad));
  int ny = DIAG_POTG_CY - (int)((DIAG_POTG_R - 3) * cos(angleRad));
  tft.drawLine(DIAG_POTG_CX, DIAG_POTG_CY, nx, ny, ACCENT_COLOR);
  tft.fillCircle(DIAG_POTG_CX, DIAG_POTG_CY, 2, TEXT_COLOR);
}

void reglages() {
  tft.fillScreen(BG_COLOR);
  tft.setCursor(16, 4);
  tft.setTextColor(ACCENT_COLOR);
  tft.println("== REGLAGES ==");
  tft.println("Version 1.0");
  tft.println("");
  tft.print("P :");
  tft.println(P);
  tft.print("I :");
  tft.println(I);
  tft.print("D :");
  tft.println(D);

  int totalEnvoyes, totalPerdus;
  calculerStatsPerte(totalEnvoyes, totalPerdus);
  int pourcentagePerte = (totalEnvoyes > 0) ? (totalPerdus * 100 / totalEnvoyes) : 0;
  tft.println("");
  tft.print("Perte 1min: ");
  tft.print(totalPerdus);
  tft.print("/");
  tft.print(totalEnvoyes);
  tft.print(" (");
  tft.print(pourcentagePerte);
  tft.println("%)");

  dessinerPointStatut(etatLiaison);

  delay(1000);
  while (digitalRead(BtnAvD) == 1) {
    delay(20);
  }
}

// affichage ecran
void menuWifi() {
  while (digitalRead(BtnAvD) == 0 || joystickDroite()) delay(20);

  const char* items[] = {
    "1. Canal",
    "2. Longue portee",
    "3. Puissance",
    "4. Adresse",
    "5. Retour",
  };
  int n = sizeof(items) / sizeof(items[0]);
  int selected = 0;

  afficherMenuWifi(selected, n, items);
  unsigned long lastActivity = millis();

  while (millis() - lastActivity < 15000) {

    if (digitalRead(BtnAvG) == 0 || joystickBas()) {
      selected++;
      if (selected >= n) selected = 0;
      lastActivity = millis();
      afficherMenuWifi(selected, n, items);
      delay(200);
    }

    if (digitalRead(BtnAvH) == 0 || joystickHaut()) {
      selected--;
      if (selected < 0) selected = n - 1;
      lastActivity = millis();
      afficherMenuWifi(selected, n, items);
      delay(200);
    }

    if (digitalRead(BtnAvD) == 0 || joystickDroite()) {
      while (digitalRead(BtnAvD) == 0) delay(20);

      if (selected == 0) {
        reglerCanal();
      } else if (selected == 1) {
        reglerLonguePortee();
      } else if (selected == 2) {
        reglerPuissance();
      } else if (selected == 3) {
        reglerAdresse();
      } else if (selected == 4) {
        return;
      }

      afficherMenuWifi(selected, n, items);
      lastActivity = millis();
    }

    if (joystickGauche()) {
      return;
    }

    delay(20);
  }
}

void afficherMenuWifi(int selected, int n, const char* items[]) {
  tft.fillScreen(BG_COLOR);
  tft.setTextSize(1);
  tft.setCursor(16, 4);
  tft.setTextColor(ACCENT_COLOR);
  tft.println("== PARAM. WIFI ==");

  int y = 24;
  for (int i = 0; i < n; i++) {
    if (i == selected) {
      tft.fillRect(2, y - 2, 156, 12, ACCENT_COLOR);
      tft.setTextColor(ST77XX_BLACK);
    } else {
      tft.setTextColor(TEXT_COLOR);
    }
    tft.setCursor(6, y);
    tft.println(items[i]);
    y += 14;
  }

  dessinerPointStatut(etatLiaison);
}

void dessinerTriangleBas(int16_t cx, int16_t cy, int16_t taille, uint16_t couleur) {
  tft.fillTriangle(cx - taille, cy - taille, cx + taille, cy - taille, cx, cy + taille, couleur);
}

void dessinerTriangleHaut(int16_t cx, int16_t cy, int16_t taille, uint16_t couleur) {
  tft.fillTriangle(cx - taille, cy + taille, cx + taille, cy + taille, cx, cy - taille, couleur);
}

void dessinerTriangleDroite(int16_t cx, int16_t cy, int16_t taille, uint16_t couleur) {
  tft.fillTriangle(cx - taille, cy - taille, cx - taille, cy + taille, cx + taille, cy, couleur);
}

void reglerCanal() {
  afficherReglageCanal();
  bool quitter = false;
  while (!quitter) {
    if (digitalRead(BtnAvH) == 0) {
      wifiChannel++;
      if (wifiChannel > 13) wifiChannel = 1;
      afficherReglageCanal();
      delay(200);
    }
    if (digitalRead(BtnAvG) == 0) {
      wifiChannel--;
      if (wifiChannel < 1) wifiChannel = 13;
      afficherReglageCanal();
      delay(200);
    }
    if (digitalRead(BtnAvD) == 0) {
      appliquerCanalWifi(wifiChannel);
      sauvegarderCanal(wifiChannel);
      quitter = true;
    }
    delay(20);
  }
  while (digitalRead(BtnAvD) == 0) delay(20);
}

void afficherReglageCanal() {
  tft.fillScreen(BG_COLOR);
  tft.setTextColor(ACCENT_COLOR);
  tft.setTextSize(1);
  tft.setCursor(16, 4);
  tft.println("== CANAL WIFI ==");

  tft.setTextColor(TEXT_COLOR);
  tft.setTextSize(3);
  tft.setCursor(60, 40);
  tft.print(wifiChannel);

  tft.setTextSize(1);
  tft.setTextColor(TEXT_COLOR);
  dessinerTriangleHaut(10, 92, 4, ACCENT_COLOR);
  tft.setCursor(18, 88);
  tft.print(": +");

  dessinerTriangleBas(70, 92, 4, ACCENT_COLOR);
  tft.setCursor(78, 88);
  tft.print(": -");

  dessinerTriangleDroite(10, 110, 4, ACCENT_COLOR);
  tft.setCursor(18, 106);
  tft.print(": Valider");

  dessinerPointStatut(etatLiaison);
}

void reglerLonguePortee() {
  afficherReglageLonguePortee();
  bool quitter = false;
  while (!quitter) {
    if (digitalRead(BtnAvG) == 0 || digitalRead(BtnAvH) == 0) {
      wifiLongRange = !wifiLongRange;
      afficherReglageLonguePortee();
      delay(200);
    }
    if (digitalRead(BtnAvD) == 0) {
      appliquerLonguePortee(wifiLongRange);
      sauvegarderLonguePortee(wifiLongRange);
      quitter = true;
    }
    delay(20);
  }
  while (digitalRead(BtnAvD) == 0) delay(20);
}

void afficherReglageLonguePortee() {
  tft.fillScreen(BG_COLOR);
  tft.setTextColor(ACCENT_COLOR);
  tft.setTextSize(1);
  tft.setCursor(16, 4);
  tft.println("== LONGUE PORTEE ==");

  tft.setTextColor(TEXT_COLOR);
  tft.setTextSize(2);
  tft.setCursor(20, 40);
  tft.print(wifiLongRange ? "ACTIVE" : "INACTIF");

  tft.setTextSize(1);
  tft.setCursor(6, 70);
  tft.println("Mode proprietaire ESP.");
  tft.setCursor(6, 80);
  tft.println("Emetteur+recepteur");
  tft.setCursor(6, 90);
  tft.println("doivent etre assortis.");

  tft.setTextColor(TEXT_COLOR);
  dessinerTriangleHaut(8, 108, 3, ACCENT_COLOR);
  dessinerTriangleBas(20, 108, 3, ACCENT_COLOR);
  tft.setCursor(28, 104);
  tft.print(": Basculer");

  dessinerTriangleDroite(8, 120, 3, ACCENT_COLOR);
  tft.setCursor(16, 116);
  tft.print(": Valider");

  dessinerPointStatut(etatLiaison);
}

void reglerPuissance() {
  afficherReglagePuissance();
  bool quitter = false;
  while (!quitter) {
    if (digitalRead(BtnAvH) == 0) {
      indexPuissance--;
      if (indexPuissance < 0) indexPuissance = NB_PUISSANCES - 1;
      afficherReglagePuissance();
      delay(200);
    }
    if (digitalRead(BtnAvG) == 0) {
      indexPuissance++;
      if (indexPuissance >= NB_PUISSANCES) indexPuissance = 0;
      afficherReglagePuissance();
      delay(200);
    }
    if (digitalRead(BtnAvD) == 0) {
      appliquerPuissanceWifi(indexPuissance);
      sauvegarderPuissance(indexPuissance);
      quitter = true;
    }
    delay(20);
  }
  while (digitalRead(BtnAvD) == 0) delay(20);
}

void afficherReglagePuissance() {
  tft.fillScreen(BG_COLOR);
  tft.setTextColor(ACCENT_COLOR);
  tft.setTextSize(1);
  tft.setCursor(16, 4);
  tft.println("== PUISSANCE WIFI ==");

  tft.setTextColor(TEXT_COLOR);
  tft.setTextSize(2);
  tft.setCursor(30, 40);
  tft.print(tablePuissance[indexPuissance].label);

  tft.setTextSize(1);
  tft.setTextColor(TEXT_COLOR);
  dessinerTriangleHaut(10, 92, 4, ACCENT_COLOR);
  tft.setCursor(18, 88);
  tft.print(": +");

  dessinerTriangleBas(70, 92, 4, ACCENT_COLOR);
  tft.setCursor(78, 88);
  tft.print(": -");

  dessinerTriangleDroite(10, 110, 4, ACCENT_COLOR);
  tft.setCursor(18, 106);
  tft.print(": Valider");

  dessinerPointStatut(etatLiaison);
}

const char* CLAVIER_TOUCHES[] = {
  "0", "1", "2", "3",
  "4", "5", "6", "7",
  "8", "9", "A", "B",
  "C", "D", "E", "F",
  "EFF", "OK"
};
const int NB_TOUCHES_CLAVIER = sizeof(CLAVIER_TOUCHES) / sizeof(CLAVIER_TOUCHES[0]);
const int CLAVIER_COLONNES = 4;

const int16_t CLAVIER_X0    = 8;
const int16_t CLAVIER_Y0    = 40;
const int16_t CLAVIER_CASE_W = 24;
const int16_t CLAVIER_CASE_H = 13;
const int16_t CLAVIER_ESPACE = 3;

void reglerAdresse() {
// avec l'adresse actuellement configuree.
  char buffer[12];
  for (int i = 0; i < 6; i++) {
    sprintf(&buffer[i * 2], "%02X", adresseBroadcast[i]);
  }

  int curseurTexte = 0;
  int curseurTouche = 0;

  afficherReglageAdresse(buffer, curseurTexte, curseurTouche);

  bool termine = false;
  while (!termine) {
    bool rafraichir = false;

    if (joystickDroite()) {
      if (curseurTouche < NB_TOUCHES_CLAVIER - 1) curseurTouche++;
      rafraichir = true;
      delay(180);
    }
    else if (joystickGauche()) {
      if (curseurTouche > 0) curseurTouche--;
      rafraichir = true;
      delay(180);
    }
    else if (joystickBas()) {
      if (curseurTouche + CLAVIER_COLONNES < NB_TOUCHES_CLAVIER) curseurTouche += CLAVIER_COLONNES;
      rafraichir = true;
      delay(180);
    }
    else if (joystickHaut()) {
      if (curseurTouche - CLAVIER_COLONNES >= 0) curseurTouche -= CLAVIER_COLONNES;
      rafraichir = true;
      delay(180);
    }

    if (digitalRead(BtnAvD) == 0) {
      const char* touche = CLAVIER_TOUCHES[curseurTouche];

      if (strcmp(touche, "EFF") == 0) {
// efface le caractere precedent et recule le curseur
        if (curseurTexte > 0) {
          curseurTexte--;
          buffer[curseurTexte] = '0';
        }
      }
      else if (strcmp(touche, "OK") == 0) {
        uint8_t nouvelleAdresse[6];
        char octetTxt[3];
        for (int i = 0; i < 6; i++) {
          octetTxt[0] = buffer[i * 2];
          octetTxt[1] = buffer[i * 2 + 1];
          octetTxt[2] = '\0';
          nouvelleAdresse[i] = (uint8_t)strtol(octetTxt, NULL, 16);
        }
        memcpy(adresseBroadcast, nouvelleAdresse, sizeof(adresseBroadcast));
        sauvegarderAdresse(adresseBroadcast);
        appliquerCanalWifi(wifiChannel);
        termine = true;
      }
      else {
        buffer[curseurTexte] = touche[0];
        if (curseurTexte < 11) curseurTexte++;
      }

      rafraichir = true;
      delay(200);
    }

    if (rafraichir && !termine) {
      afficherReglageAdresse(buffer, curseurTexte, curseurTouche);
    }

    delay(20);
  }

  while (digitalRead(BtnAvD) == 0) delay(20);
}

void afficherReglageAdresse(const char *buffer, int curseurTexte, int curseurTouche) {
  tft.fillScreen(BG_COLOR);
  tft.setTextColor(ACCENT_COLOR);
  tft.setTextSize(1);
  tft.setCursor(10, 4);
  tft.println("== ADRESSE MAC ==");

// caractere en cours d'edition surligne
  int x = 8;
  int y = 20;
  for (int i = 0; i < 6; i++) {
    for (int c = 0; c < 2; c++) {
      int posBuffer = i * 2 + c;
      bool estCurseur = (posBuffer == curseurTexte);
      if (estCurseur) {
        tft.fillRect(x - 1, y - 1, 8, 10, ACCENT_COLOR);
        tft.setTextColor(ST77XX_BLACK);
      } else {
        tft.setTextColor(TEXT_COLOR);
      }
      tft.setCursor(x, y);
      tft.print(buffer[posBuffer]);
      x += 7;
    }
    x += 4;
    if (i < 5) {
      tft.setTextColor(TEXT_COLOR);
      tft.setCursor(x, y);
      tft.print(":");
      x += 6;
    }
  }

// clavier virtuel
  for (int i = 0; i < NB_TOUCHES_CLAVIER; i++) {
    int col = i % CLAVIER_COLONNES;
    int row = i / CLAVIER_COLONNES;
    int cx = CLAVIER_X0 + col * (CLAVIER_CASE_W + CLAVIER_ESPACE);
    int cy = CLAVIER_Y0 + row * (CLAVIER_CASE_H + CLAVIER_ESPACE);

    bool estSelectionnee = (i == curseurTouche);
    uint16_t couleurFond = estSelectionnee ? ACCENT_COLOR : BG_COLOR;
    uint16_t couleurTexte = estSelectionnee ? ST77XX_BLACK : TEXT_COLOR;

    tft.drawRect(cx, cy, CLAVIER_CASE_W, CLAVIER_CASE_H, TEXT_COLOR);
    tft.fillRect(cx + 1, cy + 1, CLAVIER_CASE_W - 2, CLAVIER_CASE_H - 2, couleurFond);
    tft.setTextColor(couleurTexte);
    tft.setCursor(cx + 3, cy + 3);
    tft.print(CLAVIER_TOUCHES[i]);
  }

  dessinerPointStatut(etatLiaison);
}

void afficher_Menu(int selected, int n) {
  const char* items[] = {
    "1. Diagnostic",
    "2. Reglages",
    "3. WiFi",
    "4. Quitter",
  };
  tft.fillScreen(BG_COLOR);
  tft.setTextSize(1);
  tft.setCursor(16, 4);
  tft.setTextColor(ACCENT_COLOR);
  tft.println("== MENU CONFIG ==");

  int y = 24;
  for (int i = 0; i < n; i++) {
    if (i == selected) {
      tft.fillRect(2, y - 2, 156, 12, ACCENT_COLOR);
      tft.setTextColor(ST77XX_BLACK);
    } else {
      tft.setTextColor(TEXT_COLOR);
    }
    tft.setCursor(6, y);
    tft.println(items[i]);
    y += 14;
  }

  dessinerPointStatut(etatLiaison);

  delay(200);
}

// affichage ecran
void drawMainScreen() {
  tft.fillScreen(BG_COLOR);
  drawRobot();
  drawSpeedLabel();
  drawBatteryLabel();
  updateSpeed(vitesse);
  updateBattery(batterie);
  dessinerPointStatut(etatLiaison);
  dernierEtatPointPrincipal = etatLiaison;
  delay(200);
}

void drawRobot() {
  int cx = ROBOT_X + ROBOT_W / 2;

// antenne
  tft.drawLine(cx, ROBOT_Y, cx, ROBOT_Y + 8, ROBOT_COLOR);
  tft.fillCircle(cx, ROBOT_Y, 2, ACCENT_COLOR);

// tête
  tft.fillRoundRect(cx - 16, ROBOT_Y + 8, 32, 24, 4, ROBOT_COLOR);
  tft.fillCircle(cx - 7, ROBOT_Y + 20, 3, BG_COLOR);
  tft.fillCircle(cx + 7, ROBOT_Y + 20, 3, BG_COLOR);

// corps
  tft.fillRoundRect(cx - 20, ROBOT_Y + 34, 40, 44, 5, ROBOT_COLOR);
  tft.fillRect(cx - 10, ROBOT_Y + 46, 20, 14, ACCENT_COLOR);

// bras
  tft.fillRoundRect(cx - 30, ROBOT_Y + 38, 8, 28, 3, ROBOT_COLOR);
  tft.fillRoundRect(cx + 22, ROBOT_Y + 38, 8, 28, 3, ROBOT_COLOR);

// jambes
  tft.fillRoundRect(cx - 16, ROBOT_Y + 78, 12, 30, 3, ROBOT_COLOR);
  tft.fillRoundRect(cx + 4,  ROBOT_Y + 78, 12, 30, 3, ROBOT_COLOR);

// pieds
  tft.fillRoundRect(cx - 18, ROBOT_Y + 106, 16, 6, 2, ROBOT_COLOR);
  tft.fillRoundRect(cx + 2,  ROBOT_Y + 106, 16, 6, 2, ROBOT_COLOR);
}

void drawSpeedLabel() {
  tft.setTextColor(ACCENT_COLOR);
  tft.setTextSize(1);
  tft.setCursor(INFO_X, SPEED_Y);
  tft.println("VITESSE");
  tft.drawFastHLine(INFO_X, SPEED_Y + 10, 60, ACCENT_COLOR);
}

void updateSpeed(int value) {
  vitesse = value;
  tft.fillRect(INFO_X, SPEED_Y + 14, 64, 24, BG_COLOR);
  tft.setTextColor(TEXT_COLOR);
  tft.setTextSize(3);
  tft.setCursor(INFO_X, SPEED_Y + 16);
  tft.print(vitesse);
  tft.setTextSize(1);
  tft.setCursor(INFO_X + 40, SPEED_Y + 32);
  tft.print("km/h");
}

void drawBatteryLabel() {
  tft.setTextColor(ACCENT_COLOR);
  tft.setTextSize(1);
  tft.setCursor(INFO_X, BATT_Y);
  tft.println("BATTERIE");
  tft.drawFastHLine(INFO_X, BATT_Y + 10, 60, ACCENT_COLOR);
}

void updateBattery(int percent) {
  batterie = constrain(percent, 0, 100);

// efface l'ancien affichage
  tft.fillRect(INFO_X, BATT_Y + 14, 64, 30, BG_COLOR);

// icône de batterie (rectangle + plot)
  int bx = INFO_X;
  int by = BATT_Y + 16;
  int bw = 44, bh = 18;

  tft.drawRoundRect(bx, by, bw, bh, 2, TEXT_COLOR);
  tft.fillRect(bx + bw, by + 4, 3, bh - 8, TEXT_COLOR);

  int fillW = map(batterie, 0, 100, 0, bw - 4);
  uint16_t col = ACCENT_COLOR;
  if (batterie < 20) col = ST77XX_RED;
  else if (batterie < 50) col = ST77XX_YELLOW;
  tft.fillRect(bx + 2, by + 2, fillW, bh - 4, col);

  tft.setTextColor(TEXT_COLOR);
  tft.setTextSize(1);
  tft.setCursor(bx, by + bh + 4);
  tft.print(batterie);
  tft.print(" %");
}
