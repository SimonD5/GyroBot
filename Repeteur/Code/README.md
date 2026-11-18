# Codes de test & Asservissement - Gyrobot

Dans ce dossier y a tous les tests unitaires et le code pour l'équilibrage du robot avec le banc de réglage PID.

## 1. test_moteurs
Programme pour tester le branchement des moteurs et voir s'ils tournent bien dans le bon sens (moteur gauche et droit).
Attention : la batterie c'est une LiPo 2S (7.4V) alors que les petits moteurs N20 c'est du 6V. Du coup dans le code on bride le PWM à 200 max sur 255 pour pas cramer les moteurs.

## 2. test_imu
Pour tester la centrale inertielle MPU6050 en I2C.
On lit l'accéléromètre et le gyroscope sur la liaison série (115200 bauds).
En gros :
- Robot droit / debout : angle vers 90° et accélération vers 0.
- Si on le penche en avant : angle < 90°, accel négative.
- Si on le penche en arrière : angle > 90°, accel positive.

## 3. RobotBalance_PID_Tuner
Le gros dossier avec tout l'asservissement pour faire tenir le gyrobot debout :
- Le code ESP32-C6 avec la boucle de stabilisation (calcul d'angle par filtre complémentaire à 200 Hz + double boucle cascade PID pour corriger la dérive).
- L'interface graphique en Python pour afficher les courbes en direct et ajuster les gains Kp, Ki, Kd et la zone morte en wifi sans recompiler à chaque fois.

Pour lancer le tuner :
1. Téléverser le code Arduino sur l'ESP32.
2. Se connecter au wifi du robot (`RobotPID`, mot de passe : `robot1234`).
3. Lancer `lancer_pid_tuner_v3.bat` (ou faire `python pid_tuner_v3_wifi.py`).

## Arborescence :
```
Code/
├── test_moteurs/               <- test PWM moteurs (bridé à 200 max)
├── test_imu/                   <- test IMU (lecture I2C)
└── RobotBalance_PID_Tuner/     <- banc complet de réglage PID
    ├── 00_Telecommande/        <- code ESP32 télécommande ESP-NOW
    ├── 01_Moteurs_6V_300RPM/   <- anciens tests avec moteurs 300 RPM
    ├── 02_Moteurs_6V_500RPM/   <- tests avec moteurs 500 RPM
    └── Outils_Tuning_PC/       <- scripts python de télémétrie
```
