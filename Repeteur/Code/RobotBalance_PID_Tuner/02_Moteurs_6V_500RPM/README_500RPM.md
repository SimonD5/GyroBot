# Moteurs 6V 500 RPM — Essais & Calibrations

Ce dossier est dédié aux tests du robot avec les **nouveaux moteurs N20 6V 500 RPM** (en remplacement des 6V 300 RPM).

---

## 1. Pourquoi passer à 500 RPM ?

Avec les moteurs 300 RPM :
- La vitesse de rotation maximale sous 6V est d'environ 5 tr/s (~12 cm/s).
- Dès que le robot s'incline au joystick au-delà de 0.8° à 1.0°, le PID demande aux roues d'accélérer pour replacer la base sous le centre de gravité. Les moteurs 300 RPM atteignent alors immédiatement leur vitesse maximale (saturation PWM 200) et ne peuvent plus suivre la vitesse de chute du robot.
- L'amplitude de pilotage était donc bridée à **0.50° max**.

Avec les moteurs 500 RPM :
- **+67% de vitesse de rotation maximale** (~8.3 tr/s au lieu de 5 tr/s).
- Réserve de vitesse permettant au robot de rouler beaucoup plus vite en translation.
- Capacité d'accepter des inclinaisons plus fortes (**1.2° à 2.5°**) sans saturation en vitesse.

---

## 2. Structure du dossier 500 RPM

- **`Firmwares_Autonomes/robot_v7_standalone_500rpm/`** :
  - Firmware final autonome intégrant le redressement automatique au sol (boutons arrière) et la conduite joystick ESP-NOW.
  - Pré-configuré avec une inclinaison initiale à **1.20°** (plage dynamique 0.30° à 2.50° via `BtnAvH` / `BtnAvG`).
- **`Bancs_Essai_Tuning/robot_v3_deadband_500rpm/`** :
  - Banc Wi-Fi pour ajuster `Kp`, `Ki`, `Kd`, `pwmMin`, `Kp2`, `Deadband` en direct depuis le PC (`lancer_pid_tuner_v3.bat`).
- **`Sessions_Tests_500RPM/`** :
  - Dossier destiné à recevoir les enregistrements télémétriques JSON et résumés CSV issus des essais avec les 500 RPM.

---

## 3. Guide de réglage PID pour les 500 RPM

1. **Point de départ** :
   - `Kp = 16.0`, `Ki = 1.0`, `Kd = 0.30`, `Consigne = +0.5°`
   - `pwmMin = 18` (à vérifier au décollage des roues)
   - `Kp2 = 0.25`, `Deadband = 2.0`
2. **Points d'attention** :
   - Le rapport d'engrenage étant plus faible sur les 500 RPM (ex: 1:50 au lieu de 1:100), le couple moteur par unité de PWM est légèrement inférieur.
   - Si le robot manque de réactivité pour tenir l'équilibre, augmenter légèrement `Kp` (ex: 17.0 à 19.0).
   - Si des micro-vibrations apparaissent dues à la vitesse de réponse plus rapide, réduire légèrement `Kd` (ex: 0.25 au lieu de 0.30).
