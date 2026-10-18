# RobotBalance — Gyrobot Deux Roues (ESP32-C6 + MPU6050 + TB6612FNG)

Banc complet d'asservissement d'équilibre pour robot gyroscopique bipode autonome et télécommandé.  
Développé sur **ESP32-C6**, centrale inertielle **MPU6050 (GY-521)**, pont en H **TB6612FNG**, alimentation **LiPo 2S (7.4V)** et liaison radio **ESP-NOW Long Range**.

---

## 1. Contexte & Historique des Motorisations (300 RPM vs 500 RPM)

> [!IMPORTANT]
> **Historique des essais (V1 à V7) :**  
> L'ensemble du développement initial, la calibration des boucles PID en cascade, les sessions de mesures Wi-Fi et les algorithmes de relevage automatique au sol ont été conçus et stabilisés pour des **moteurs N20 6V 300 RPM**.

### Pourquoi la transition vers les moteurs 6V 500 RPM ?
- **Limitation des 300 RPM** :  
  Sous 6V, un moteur 300 RPM tourne au maximum à ~5 tr/s, ce qui correspond à une vitesse d'avance linéaire d'environ 10 à 12 cm/s.  
  En équilibre statique ou à très basse vitesse, ce couple est excellent. Mais dès qu'on commande au joystick une inclinaison supérieure à **0.8° à 1.0°**, le robot commence à chuter en avant : le PID demande alors aux roues d'accélérer pour replacer la base sous le centre de gravité. Les moteurs 300 RPM atteignent immédiatement leur vitesse maximale de rotation (saturation PWM 200) et ne parviennent plus à rattraper la chute. Le robot décroche alors ou compense violemment en sens inverse.  
- **Avantages des moteurs 500 RPM** :  
  +67% de vitesse de rotation maximale

---

## 2. Nouvelle Arborescence du Projet

Le projet a été réorganisé de manière claire et modulaire pour différencier facilement les tests selon la motorisation :

```text
RobotBalance_PID_Tuner/
├── README.md                                 <- Documentation générale (ce fichier)
├── lancer_pid_tuner_v3.bat                   <- Lanceur direct Interface PC (Banc Wi-Fi V3)
├── lancer_relevage_tuner.bat                 <- Lanceur direct Interface PC (Relevage Auto)
│
├── 00_Telecommande/
│   └── telecommandev7/
│       └── telecommandev7.ino                <- Firmware ESP32-C6 Télécommande ESP-NOW (Inchangé)
│
├── 01_Moteurs_6V_300RPM/
│   ├── Firmwares_Autonomes/
│   │   ├── robot_v7_standalone/             <- VERSION DE RÉFÉRENCE 300 RPM (Relevage + Conduite fine)
│   │   ├── robot_v6_standalone/             <- Version V6 autonome (Relevage boutons arrière)
│   │   └── robot_v5_standalone/             <- Version V5 autonome (Équilibre + Joystick de base)
│   ├── Bancs_Essai_Tuning/
│   │   ├── robot_v3_deadband_WIFI/          <- Banc Wi-Fi V3 (Double boucle + Zone morte + 45s acquisition)
│   │   ├── robot_relevage_WIFI/             <- Banc Wi-Fi dédié au réglage du redressement au sol
│   │   ├── robot_cascade_pid_WIFI/          <- Banc Wi-Fi double boucle classique
│   │   ├── robot_reglage_pid_WIFI/          <- Banc Wi-Fi boucle simple
│   │   ├── robot_reglage_pid_USB/           <- Banc série USB (921600 bauds)
│   │   └── robot_reglage_pidAncien/         <- Archive code initial
│   └── Sessions_Tests_300RPM/                <- 17 sessions d'essais réels (JSON de télémétrie + CSV)
│
├── 02_Moteurs_6V_500RPM/                     <- NOUVELLE CAMPAGNE D'ESSAIS 500 RPM
│   ├── README_500RPM.md                      <- Guide spécifique aux 500 RPM
│   ├── Firmwares_Autonomes/
│   │   └── robot_v7_standalone_500rpm/      <- Firmware autonome adapté 500 RPM (Inclinaison init 1.20°)
│   ├── Bancs_Essai_Tuning/
│   │   └── robot_v3_deadband_500rpm/        <- Banc Wi-Fi prêt pour recalibrer Kp, Ki, Kd, Pmin des 500 RPM
│   └── Sessions_Tests_500RPM/                <- Dossier prêt pour sauvegarder les sessions d'essais 500 RPM
│
└── Outils_Tuning_PC/                         <- Interfaces graphiques Python & Scripts de capture
    ├── pid_tuner_v3_wifi.py                 <- Interface graphique complète V3 (Courbes, diagnostics, export)
    ├── relevage_tuner_wifi.py               <- Interface de réglage dynamique du relevage automatique
    ├── pid_tuner_cascade_wifi.py            <- Interface double boucle cascade
    ├── pid_tuner_wifi.py                    <- Interface simple boucle Wi-Fi
    ├── pid_tuner.py                         <- Interface série USB
    ├── lancer_pid_tuner_v3.bat              <- Lanceur V3 local
    └── lancer_relevage_tuner.bat            <- Lanceur Relevage local
```

---

## 3. Matériel, Câblage & Sens de Rotation des Moteurs (300 RPM)

### A. Brochage ESP32-C6 -> Drivers & Capteurs

| Composant | Signal matériel | Broche ESP32-C6 | Fonction / Rôle |
|---|---|---|---|
| **MPU-6050** | `SDA` | **GPIO 2** | Bus I2C (400 kHz) |
| **MPU-6050** | `SCL` | **GPIO 3** | Horloge I2C |
| **TB6612FNG** | `STBY` | **GPIO 22** | Standby (mis à `HIGH` pour activer le pont en H) |
| **Moteur A (Gauche)** | `PWMA` | **GPIO 11** | Vitesse PWM Moteur Gauche (20 kHz, 8 bits) |
| **Moteur A (Gauche)** | `AIN1` | **GPIO 7** | Pont logique Direction Moteur Gauche |
| **Moteur A (Gauche)** | `AIN2` | **GPIO 10** | Pont logique Direction Moteur Gauche |
| **Moteur B (Droit)** | `PWMB` | **GPIO 23** | Vitesse PWM Moteur Droit (20 kHz, 8 bits) |
| **Moteur B (Droit)** | `BIN1` | **GPIO 21** | Pont logique Direction Moteur Droit |
| **Moteur B (Droit)** | `BIN2` | **GPIO 20** | Pont logique Direction Moteur Droit |

---

### B. Sens de Rotation & Logique des Moteurs (Guide Câblage)

Dans le code Arduino, la fonction bas niveau commande chaque moteur via le TB6612FNG :
```cpp
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
```

#### Sens de rotation détaillé :
- **Moteur A = Moteur Gauche** :
  - Facteur logiciel : `SIGN_MOTEUR_A = +1`
  - **Marche Avant** (`v > 0`) : `AIN1 = HIGH`, `AIN2 = LOW`. La roue gauche tourne dans le sens de la marche avant.
  - **Marche Arrière** (`v < 0`) : `AIN1 = LOW`, `AIN2 = HIGH`. La roue gauche tourne en arrière.
- **Moteur B = Moteur Droit** :
  - Facteur logiciel : `SIGN_MOTEUR_B = -1`
  - **Marche Avant** (`v > 0`) : Avec `SIGN_MOTEUR_B = -1`, la commande réelle envoyée au pont est négative, soit `BIN1 = LOW`, `BIN2 = HIGH`. La roue droite tourne donc géométriquement vers l'AVANT (sens concordant avec la roue gauche).
  - **Marche Arrière** (`v < 0`) : `BIN1 = HIGH`, `BIN2 = LOW`. La roue droite tourne en arrière.
- **Virages (Différentiel)** :
  - **Tourner à Droite** (`turn > 0`) : La commande gauche augmente ($v_A = cmd + turn$) et la commande droite diminue ($v_B = cmd - turn$). Le robot pivote vers la droite.
  - **Tourner à Gauche** (`turn < 0`) : La commande gauche diminue et la commande droite augmente. Le robot pivote vers la gauche.

> [!TIP]
> **Si vous branchez de nouveaux moteurs ou changez le câblage :**
> - Si le moteur gauche tourne à l'envers : inverser les fils branchés sur `A01`/`A02`, ou changer `SIGN_MOTEUR_A` en `-1` dans le firmware.
> - Si le moteur droit tourne à l'envers : inverser les fils branchés sur `B01`/`B02`, ou changer `SIGN_MOTEUR_B` en `+1` dans le firmware.

---

## 4. Tableau Récapitulatif des Valeurs PID Validées (Moteurs 300 RPM)

Ce jeu de coefficients a été validé au banc d'essai sur plus de 17 sessions de mesures pour garantir un équilibre stationnaire stable, sans dérive et avec absorption des chocs :

| Paramètre | Macro dans le code | Valeur Validée (300 RPM) | Explication & Rôle Dynamique |
|---|---|---|---|
| **Gain Proportionnel (Angle)** | `CONF_Kp` | **16.0** | Raideur d'équilibre. Couple proportionnel à l'écart d'inclinaison. |
| **Gain Intégral (Angle)** | `CONF_Ki` | **1.0** | Annule l'erreur statique résiduelle. Anti-windup borné à $I_{max} = 80$. |
| **Gain Dérivé (Angle)** | `CONF_Kd` | **0.30** | Amortisseur angulaire appliqué sur la vitesse du gyroscope (`rateF`). |
| **Angle d'Équilibre Naturel** | `CONF_CONSIGNE` | **+0.50°** | Centre de masse mécanique réel du robot avec sa batterie. |
| **Friction / Seuil de Décollage** | `CONF_PWM_MIN` | **18** | PWM minimal pour vaincre le frottement statique des réducteurs N20 300 RPM. |
| **Plafond PWM Sécurité** | `PWM_MAX` | **200 / 255** | Bride de tension pour protéger les bobinages 6V sous LiPo 2S (7.4V - 8.4V). |
| **Gain Anti-Dérive (Vitesse)** | `CONF_Kp2` | **0.25** | Boucle 2 : ramène le robot vers sa position en inclinant légèrement la consigne selon la vitesse estimée. |
| **Intégrale Anti-Dérive** | `CONF_Ki2` | **0.00** | Laissée à 0 pour éliminer tout effet de pompage ou oscillation lente. |
| **Zone Morte Vitesse (Deadband)** | `CONF_DEADBAND` | **2.0 PWM** | Supprime le tremblement à 10-15s en ignorant le bruit de micro-correction. |
| **Filtre Vitesse Estimée** | `ALPHA_V` | **0.03** | Filtre passe-bas (~200 ms) dérivant la vitesse du robot à partir du PWM. |
| **Filtre Gyroscope** | `ALPHA_D` | **0.40** | Lissage de la vitesse angulaire pour un terme dérivé propre sans à-coups. |
| **Fréquence de Boucle** | `PERIODE_US` | **5000 µs (200 Hz)** | Période d'échantillonnage déterministe stricte. |
| **Angle de Chute (Coupure)** | `CONF_ANGLE_CHUTE` | **18.0°** | Coupe instantanément les moteurs en cas de perte d'équilibre. |
| **Armement Vertical Auto** | `CONF_ANGLE_REPRISE` | **2.0°** | S'arme automatiquement dès que le robot est maintenu vertical ~300 ms. |

### Paramètres du Relevage Automatique au sol (Self-Recovery 300 RPM)
- **Couché en avant ($> 14°$)** :
  - `CONF_PWM_KICK = 180` pendant `CONF_DUREE_KICK_MS = 350 ms` max.
  - Coupure du kick dès $	heta \le 	heta_{consigne} + 2.0°$.
  - Amortissement de capture avec `CONF_KD_BOOST = 2.5x` (Kd = 0.75).
  - Gel temporaire de la boucle 2 pendant `CONF_SETTLE_MS = 600 ms` pour caler l'assiette.
- **Couché en arrière ($< -14°$)** :
  - À-coup doux de bascule vers l'avant : `CONF_PWM_FLIP = 120` pendant `CONF_DUREE_FLIP_MS = 130 ms`.
  - Stabilisation au sol sur l'avant (200 ms), puis enchaînement automatique du kick de relevage avant.

### Paramètres Joystick (Conduite 300 RPM)
- `CONF_JOY_VITESSE_INIT = 0.50°` (amplitude restreinte adaptée à la vitesse maximale des 300 RPM).
- `CONF_JOY_VITESSE_MIN = 0.20°` / `CONF_JOY_VITESSE_MAX = 1.50°`.
- `CONF_JOY_Y_SIGN = +1.0f` (pousser en avant incline en avant).
- `CONF_TURN_GAIN = 1.2f`.

---

## 5. Guide des Essais avec les Nouveaux Moteurs 6V 500 RPM

Les fichiers dédiés se trouvent dans [`02_Moteurs_6V_500RPM/`](02_Moteurs_6V_500RPM/) :

### Étape 1 : Vérification au Banc Wi-Fi (`robot_v3_deadband_500rpm`)
1. Ouvrir [`02_Moteurs_6V_500RPM/Bancs_Essai_Tuning/robot_v3_deadband_500rpm/robot_v3_deadband_500rpm.ino`](02_Moteurs_6V_500RPM/Bancs_Essai_Tuning/robot_v3_deadband_500rpm/robot_v3_deadband_500rpm.ino) et le téléverser sur l'ESP32-C6.
2. Allumer le robot et connecter le PC au réseau Wi-Fi `RobotPID` (mot de passe : `robot1234`).
3. Lancer l'interface : double-cliquer sur `lancer_pid_tuner_v3.bat`.
4. Tester le frottement statique (`pwmMin`) :
   - Les moteurs 500 RPM ayant un réducteur différent, le frottement de décollage peut être légèrement différent (typiquement entre 18 et 24).
5. Ajuster `Kp` et `Kd` si nécessaire :
   - Si le robot semble trop "mou" pour contrer une pichenette : augmenter `Kp` vers 17 ou 18.
   - Si le robot présente des tremblements à haute fréquence : diminuer `Kd` vers 0.25.
6. Cliquer sur **« Exporter la session »** à la fin des tests pour enregistrer les courbes dans `02_Moteurs_6V_500RPM/Sessions_Tests_500RPM/`.

### Étape 2 : Utilisation Autonome Finale (`robot_v7_standalone_500rpm`)
---

## 6. Utilisation de la Télécommande ESP-NOW (`telecommandev7`)
### Commandes disponibles :
- **Joystick Droit** :
  - **Axe Y (Avancer / Reculer)** : modifie l'angle de consigne d'équilibre.
  - **Axe X (Gauche / Droite)** : braquage différentiel pour faire tourner le robot.
  - **Clic Joystick** : arrêt d'urgence immédiat (moteurs coupés).
- **Boutons Arrière (BtnArD / BtnArG)** :
  - **Si le robot est au sol (tombé en avant ou en arrière)** : déclenche le redressement automatique !
  - **Si le robot est debout en équilibre** : arrêt d'urgence immédiat.
- **Boutons Avant (BtnAvH / BtnAvG)** :
  - **BtnAvH** : augmente la vitesse maximale autorisée au joystick.
  - **BtnAvG** : réduit la vitesse maximale (mode ultra-doux / précision).
- **Délai Gyroscope 3 secondes** :
  - À l'allumage du robot, attendre 3 secondes immobiles pendant que le MPU-6050 calibre son zéro.
- **Armement automatique** :
  - Poser le robot à la verticale : il s'arme automatiquement et stabilise ses roues dès qu'il est droit !
