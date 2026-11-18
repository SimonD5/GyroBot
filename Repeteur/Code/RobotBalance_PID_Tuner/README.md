# Banc de réglage PID - Gyrobot deux roues

Tout ce qui sert à faire tenir le robot en équilibre sur deux roues avec l'ESP32-C6, l'IMU MPU6050 et le driver TB6612FNG, alimenté par une LiPo 2S (7.4V).

## Moteurs 300 RPM vs 500 RPM
Au début (versions v1 à v7) on utilisait des moteurs N20 6V 300 RPM.
Le souci c'est qu'ils manquaient pas mal de vitesse : dès qu'on voulait avancer un peu au joystick (inclinaison au-delà de 1°), les moteurs saturaient direct au PWM max (200) et n'arrivaient plus à rattraper la chute, donc le robot décrochait.
On est donc passés sur des moteurs 500 RPM qui tournent plus vite (+60% de vitesse max) et permettent au robot de se déplacer sans se casser la figure.

## Organisation des dossiers
- `00_Telecommande/` : firmware de la télécommande ESP32 avec écran SPI et transmission ESP-NOW.
- `01_Moteurs_6V_300RPM/` : les anciens tests et codes sur les 300 RPM, avec les fichiers de télémétrie enregistrés (JSON/CSV).
- `02_Moteurs_6V_500RPM/` : les codes pour les 500 RPM (firmware autonome v7 et banc de test v4).
- `Outils_Tuning_PC/` : interfaces graphiques en Python pour visualiser les courbes et ajuster les paramètres en direct via wifi.

## Câblage ESP32-C6
- **MPU-6050** : SDA sur GPIO 2, SCL sur GPIO 3 (I2C à 400 kHz).
- **TB6612FNG** :
  - STBY : GPIO 22 (mis à 1 pour activer les sorties).
  - Moteur A (gauche) : PWMA sur GPIO 11, AIN1 sur GPIO 7, AIN2 sur GPIO 10.
  - Moteur B (droit) : PWMB sur GPIO 23, BIN1 sur GPIO 21, BIN2 sur GPIO 20.

*Sens des moteurs :* comme les deux moteurs sont montés face à face, ils tournent en sens inverse pour aller en avant. Dans le code on gère ça avec `SIGN_MOTEUR_A = 1` et `SIGN_MOTEUR_B = -1` (ou l'inverse sur les 500 RPM car le bobinage interne est inversé).

## Valeurs PID validées
Pour les moteurs 500 RPM (après pas mal d'essais pour virer les vibrations) :
- `Kp = 13.0`
- `Ki = 1.0`
- `Kd = 0.10` (attention à pas trop monter sinon le robot vibre à ~10 Hz)
- `Consigne = +1.35°` (angle d'équilibre mécanique réel)
- `pwmMin = 28` (seuil de friction pour vaincre le frottement au décollage)
- `Kp2 = 0.25` et `Ki2 = ~0.006` (boucle cascade anti-dérive)
- `Deadband = 2.0` (zone morte pour laisser flotter un peu sans osciller)

Pour les 300 RPM c'était : Kp=16, Ki=1, Kd=0.30, consigne=+0.5°, pwmMin=18.

## Relevage automatique
Si le robot tombe par terre (couché en avant ou en arrière), on appuie sur les boutons arrière de la télécommande : ça envoie une grosse impulsion moteur (kick) pour le faire basculer et le redresser, et la boucle PID reprend la main dès qu'il repasse près de la verticale.

## Commandes télécommande (telecommandev7)
- Joystick droit : axe Y pour avancer/reculer (joue sur la consigne d'angle), axe X pour tourner (différentiel).
- Clic joystick droit : coupure d'urgence (moteurs à 0).
- Boutons arrière : redressement auto si tombé par terre, sinon coupure d'urgence.
- Boutons avant (haut/bas) : pour ajuster la vitesse max autorisée au joystick.
