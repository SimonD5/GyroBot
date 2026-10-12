# RobotBalance — Banc de réglage PID & Cascade V3 (Wi-Fi)

Environnement d'asservissement en équilibre pour gyrobot deux roues (ESP32-C6 + IMU MPU6050 + TB6612FNG + N20), avec interface graphique de télémétrie et réglage dynamique en direct.

---

## 1. Matériel & Câblage

- **Microcontrôleur** : ESP32-C6 (Seed Studio XIAO ou équivalent)
- **Centrale inertielle** : MPU6050 (GY-521, I2C à 400 kHz)
- **Pont en H** : TB6612FNG
- **Motorisation** : 2 moteurs N20 (PWM limité matériellement/logiciellement à 200/255)
- **Alimentation** : Batterie LiPo 2S (7,4 V)
- **Liaison PC** : Wi-Fi (Point d'accès créé par l'ESP32 : `RobotPID` / `robot1234`)

---

## 2. Versions disponibles

| Version | Firmware Arduino | Script Python / Lanceur | Spécificités |
|---|---|---|---|
| **V1 (USB)** | `robot_reglage_pid_USB/` | `pid_tuner.py` | Liaison série USB (921600 bauds), PID 1 boucle. |
| **V2 (Cascade Wi-Fi)** | `robot_cascade_pid_WIFI/` | `pid_tuner_cascade_wifi.py` | Double boucle cascade (Angle + Vitesse estimée), Wi-Fi TCP. |
| **V3 (Zone Morte + Wi-Fi Persistant)** | `robot_v3_deadband_WIFI/` | `pid_tuner_v3_wifi.py`<br>`lancer_pid_tuner_v3.bat` | **Recommandé** : Zone morte réglable en direct, pas de coupure Wi-Fi, acquisition 45s (9000 éch.). |

---

## 3. Fonctionnement de la Version V3

### A. Double boucle en cascade
- **Boucle 1 (Angle, 200 Hz / 5 ms)** :
  - Angle estimé par filtre complémentaire (gyroscope + accéléromètre).
  - Régulation PID : `Kp`, `Ki`, `Kd`, compensation de frottement `pwmMin`.
- **Boucle 2 (Vitesse / Dérive)** :
  - Vitesse estimée par filtre passe-bas sur la commande PWM (`ALPHA_V = 0.03`).
  - Action proportionnelle `Kp2` et intégrale `Ki2`.
  - Calcule une correction d'angle (`correctionAngle`) appliquée à la consigne d'équilibre :  
    $$	ext{consigneActive} = 	ext{consigne} - 	ext{correctionAngle}$$

### B. Zone morte anti-tremblement (`Deadband PWM`)
- La vitesse estimée comporte inévitablement du bruit lié aux micro-corrections rapides.
- **Principe** : Tant que $|	ext{vitesseEstimee}| \le 	ext{DEADBAND}$, la vitesse active est nulle ($v_{	ext{active}} = 0$). Dès qu'elle dépasse le seuil, seule la partie excédentaire est injectée dans la boucle 2.
- **Avantage** : Supprime les tremblements et oscillations à haute fréquence tout en corrigeant la dérive continue du robot (par exemple glissement vers l'avant de 50 cm en 20 s).
- **Réglage en direct** : La deadband est modifiable à chaud via le champ **`Deadband (PWM)`** de l'interface (commande `'b'`), sans recompiler.

### C. Wi-Fi persistant et non-bloquant
- Le socket TCP reste ouvert en continu pour éliminer les déconnexions/reconnexions intempestives de Windows.
- Les communications réseau s'exécutent en mode non-bloquant pour garantir le respect strict des 5 ms (200 Hz) de la boucle de contrôle.

### D. Enregistrement étendu (45 s)
- Tampon d'échantillons alloué dynamiquement (jusqu'à 9000 points à 200 Hz = 45 secondes).
- Données transmises au PC après chute ou fin d'essai.

---

## 4. Utilisation rapide (V3)

### Étape 1 : Téléversement du code Arduino
1. Ouvrir [`robot_v3_deadband_WIFI/robot_v3_deadband_WIFI.ino`](file:///c:/Users/nomis/Documents/Polytech/GPSE/4A/Projet/gyrobot/Repeteur/Code/RobotBalance_PID_Tuner/robot_v3_deadband_WIFI/robot_v3_deadband_WIFI.ino) dans l'IDE Arduino.
2. Sélectionner la carte ESP32-C6 et le port COM.
3. Téléverser le code (une seule fois nécessaire).

### Étape 2 : Connexion Wi-Fi
1. Allumer le robot.
2. Sur le PC, se connecter au réseau Wi-Fi :
   - **SSID** : `RobotPID`
   - **Mot de passe** : `robot1234`

### Étape 3 : Lancement de l'interface
- Double-cliquer sur `lancer_pid_tuner_v3.bat`  
  *(ou exécuter dans un terminal : `python pid_tuner_v3_wifi.py`)*

### Étape 4 : Réglage et Essai
1. Dans l'interface, vérifier que l'ESP32 est bien connecté (`192.168.4.1:8888`).
2. Ajuster les valeurs souhaitées :
   - `Kp`, `Ki`, `Kd`, `Consigne °`, `PWM min`
   - `Kp2 (Vitesse)`, `Ki2 (Centre)`
   - `Deadband (PWM)` (valeur typique : entre `1.5` et `3.0`)
3. Cliquer sur **« Appliquer PID »**.
4. Cliquer sur **« ARMER »**, placer le robot à la verticale (proche de sa consigne d'angle, ex: 2.5°).
5. Le robot démarre automatiquement l'essai dès qu'il est dans la plage de déclenchement.
6. À la fin de l'essai ou en cas de chute, la télémétrie complète (courbes, statistiques, diagnostic) s'affiche et s'enregistre automatiquement dans `SessionTest/`.

---

## 5. Protocole des commandes (Série / TCP)

| Commande | Action | Exemple |
|---|---|---|
| `p<val>` | Règle Kp | `p15.0` |
| `i<val>` | Règle Ki | `i1.0` |
| `d<val>` | Règle Kd | `d0.3` |
| `s<val>` | Règle la consigne d'angle (°) | `s2.5` |
| `m<val>` | Règle le PWM minimal (friction) | `m11` |
| `k<val>` | Règle Kp2 (gain boucle 2 vitesse) | `k0.05` |
| `j<val>` | Règle Ki2 (intégrale boucle 2) | `j0.0` |
| `b<val>` | Règle la zone morte vitesse (PWM) | `b2.0` |
| `a1` / `a0` | Armer / Désarmer | `a1` |
| `z` | Remet le zéro du gyroscope | `z` |
| `g` | Inverse le signe du gyroscope | `g` |
| `v<pwm>` | Test direct moteur (hors asservissement) | `v30` |
| `q` | Arrêt d'urgence moteurs | `q` |
| `?` | Demande l'état des paramètres (`PARAM,...`) | `?` |
