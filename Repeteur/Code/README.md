# Codes de test & Asservissement – Gyrobot Répéteur

Ce dossier contient les codes de test préliminaires et l'environnement complet d'asservissement / réglage PID pour le gyrobot répéteur.

---

## 1. `test_moteurs/` – Test des moteurs

Ce code permet de **tester le fonctionnement des moteurs** ainsi que leur **sens de rotation**.

### ⚠️ Important – Limitation PWM

Les moteurs fonctionnent en **6V**, tandis que la batterie LiPo délivre **7,4V**.  
Pour éviter d'endommager les moteurs, la valeur PWM est **limitée à 200/255** (≈ 78 % du duty cycle).

> **Ne pas dépasser 200/255 de PWM** sous peine de détériorer les moteurs.

### Fonctionnalités testées
- Vérification du **sens de rotation** (sens horaire / anti-horaire)
- Validation de la réponse PWM à vitesse réduite

---

## 2. `test_imu/` – Test de l'IMU

Ce code permet de **tester l'IMU** (Inertial Measurement Unit) et de lire les données de mouvement dans les différents axes.

### Fonctionnalités testées
- Lecture des **accélérations** (axes X, Y, Z) en m/s²
- Lecture des **vitesses angulaires** (gyroscope, axes X, Y, Z) en °/s
- Vérification de la cohérence des mesures selon les **différents sens de déplacement** et d'orientation du robot

### Observations relevées (axe X)

| Situation | Angle X | Accélération X |
|-----------|---------|----------------|
| Robot à l'équilibre | **90°** | ≈ 0 |
| Robot penché / accélère **vers l'avant** | < 90° | **négative (−)** |
| Robot penché / accélère **vers l'arrière** | > 90° | **positive (+)** |

> Les valeurs d'angle et d'accélération ci-dessus sont toutes relatives à l'**axe X**.

---

## 3. `RobotBalance_PID_Tuner/` – Banc d'équilibrage et réglage PID (Cascade V3 Wi-Fi & Zone Morte)

Ce sous-dossier regroupe le firmware d'asservissement en équilibre pour l'ESP32-C6 et l'interface graphique de télémétrie / réglage en temps réel.

### Architecture de contrôle (Double boucle cascade)
1. **Boucle 1 — Angle & Stabilité rapide (200 Hz / 5 ms) :**
   - Calcul de l'angle par filtre complémentaire (accéléromètre + gyroscope MPU6050).
   - PID complet avec Kp, Ki, Kd et compensation du seuil de frottement statique (`pwmMin`).
2. **Boucle 2 — Vitesse & Dérive (Cascade) :**
   - Estimation de la vitesse d'avancement par intégration filtrée de la commande PWM.
   - Ajustement de la consigne d'angle (`consigneActive = consigne - correctionAngle`) pour compenser la dérive sans intervention manuelle.

### Nouveautés de la Version V3 (`robot_v3_deadband_WIFI` & `pid_tuner_v3_wifi.py`)
- **Zone morte anti-tremblement réglable à chaud (`Deadband PWM`) :**
  - Ignore le bruit de vitesse résiduel quand la vitesse estimée est inférieure au seuil (ex: 2.0 PWM), ce qui supprime totalement les vibrations à haute fréquence tout en conservant la correction sur dérive continue.
  - Commande série/TCP dédiée `'b'` (`b<valeur>`), modifiable en direct depuis l'interface sans avoir à recompiler le microcontrôleur.
- **Connexion Wi-Fi persistant et non-bloquante :**
  - Le robot crée un point d'accès Wi-Fi (`RobotPID` / `robot1234`).
  - Le socket TCP reste ouvert en continu pour éviter que Windows ne se déconnecte/reconnecte intempestivement, sans jamais ralentir la boucle de régulation temps réel (200 Hz garanti).
- **Acquisition haute capacité (45 secondes) :**
  - Tampon mémoire dynamique de 9000 échantillons à 200 Hz pour capturer des essais longs et détaillés.
- **Interface graphique complète :**
  - Contrôle et envoi instantané des paramètres : `Kp`, `Ki`, `Kd`, `Consigne`, `PWM min`, `Kp2`, `Ki2`, `Deadband`.
  - Boutons d'armement/désarmement sécurisés, calibration gyro, inversion d'axe et tests de motricité directe.
  - Visualisation des graphiques (angle, vitesse estimée, PWM, consigne active) et export automatique des sessions au format JSON et CSV.

### Démarrage rapide V3

1. **Téléversement Arduino :**
   - Ouvrir `RobotBalance_PID_Tuner/robot_v3_deadband_WIFI/robot_v3_deadband_WIFI.ino`.
   - Téléverser sur l'ESP32-C6.
2. **Connexion Wi-Fi :**
   - Connecter le PC au réseau Wi-Fi : **`RobotPID`** (mot de passe : `robot1234`).
3. **Lancement de l'interface :**
   - Double-cliquer sur `RobotBalance_PID_Tuner/lancer_pid_tuner_v3.bat` (ou lancer `python pid_tuner_v3_wifi.py`).

---

## Structure du dossier

```
Code/
├── README.md                      ← ce fichier
├── test_moteurs/                  ← test PWM moteurs (max 200/255)
├── test_imu/                      ← test accélérations & vitesses angulaires IMU
└── RobotBalance_PID_Tuner/        ← banc d'équilibrage & réglage PID (Cascade V3 Wi-Fi)
    ├── lancer_pid_tuner_v3.bat    ← lanceur 1-clic pour l'interface V3
    ├── pid_tuner_v3_wifi.py       ← interface graphique Python V3 (deadband + cascade)
    ├── robot_v3_deadband_WIFI/    ← firmware ESP32-C6 V3 (zone morte + WiFi persistant)
    │   └── robot_v3_deadband_WIFI.ino
    ├── SessionTest/               ← historiques des sessions d'essais (JSON/CSV)
    └── README.md                  ← guide détaillé du banc de réglage
```
