# Moteurs 6V 300 RPM — Firmwares, Bancs & Sessions Validées

Ce dossier regroupe l'intégralité des développements, étalonnages et acquisitions réalisés avec les motoréducteurs **N20 6V 300 RPM**.

---

## 1. Organisation du dossier 300 RPM

- **`Firmwares_Autonomes/`** :
  - **`robot_v7_standalone/`** : Version autonome de référence pour les 300 RPM.
    - Redressement automatique au sol via les boutons arrière de la télécommande.
    - Équilibre double boucle stabilisé.
    - Conduite joystick lissée avec amplitude adaptée (0.50° par défaut) et réglage fin via boutons avant.
  - **`robot_v6_standalone/`** : Version autonome intermédiaire (relevage avec amplitude 1.5°).
  - **`robot_v5_standalone/`** : Version autonome basique (équilibre + joystick, sans relevage au sol).
- **`Bancs_Essai_Tuning/`** :
  - **`robot_v3_deadband_WIFI/`** : Banc d'essais Wi-Fi complet avec SoftAP continu et acquisition 45s.
  - **`robot_relevage_WIFI/`** : Banc d'optimisation du kick et de l'amortissement de capture pour le relevage automatique.
  - **`robot_cascade_pid_WIFI/`**, **`robot_reglage_pid_WIFI/`**, **`robot_reglage_pid_USB/`** : Bancs historiques.
- **`Sessions_Tests_300RPM/`** :
  - Contient les 17 enregistrements réels (fichiers JSON complets et résumés CSV) mesurés au banc d'essais.

---

## 2. Sens de Rotation des Moteurs 300 RPM & Brochage

- **Moteur A = Moteur Gauche** :
  - Commande : `PWMA` = GPIO 11, `AIN1` = GPIO 7, `AIN2` = GPIO 10
  - Signe : `SIGN_MOTEUR_A = +1`
  - Marche avant : `AIN1 = HIGH, AIN2 = LOW`
  - Marche arrière : `AIN1 = LOW, AIN2 = HIGH`
- **Moteur B = Moteur Droit** :
  - Commande : `PWMB` = GPIO 23, `BIN1` = GPIO 21, `BIN2` = GPIO 20
  - Signe : `SIGN_MOTEUR_B = -1` (câblage inversé tête-bêche)
  - Marche avant : `BIN1 = LOW, BIN2 = HIGH` (inversé par logiciel pour propulser en avant)
  - Marche arrière : `BIN1 = HIGH, BIN2 = LOW`
- **Virages** :
  - Droite : Moteur A accélère, Moteur B ralentit.
  - Gauche : Moteur A ralentit, Moteur B accélère.

---

## 3. Paramètres PID Validés (300 RPM)

- **Stabilisation (Boucle 1)** : `Kp = 16.0`, `Ki = 1.0` ($I_{max} = 80$), `Kd = 0.30`, `Consigne = +0.50°`, `pwmMin = 18`, `PWM_MAX = 200`.
- **Anti-Dérive (Boucle 2)** : `Kp2 = 0.25`, `Ki2 = 0.00`, `Deadband = 2.0 PWM`, `ALPHA_V = 0.03`.
- **Relevage Automatique** : `PWM_KICK = 180` (350 ms max), `Angle Capture = 2.0°`, `Kd Boost = 2.5x` (Kd = 0.75), `Settle = 600 ms`.
- **Joystick** : `Inclinaison Init = 0.50°` (plage 0.20° à 1.50°), `CONF_JOY_Y_SIGN = +1.0f`.
