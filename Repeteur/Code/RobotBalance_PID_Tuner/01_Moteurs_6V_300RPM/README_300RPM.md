# Moteurs 6V 300 RPM - Anciens tests

Dossier d'archive des premiers essais réalisés avec les moteurs N20 6V 300 RPM.

## Organisation :
- `Firmwares_Autonomes/` :
  - `robot_v7_standalone` : la version finale 300 RPM avec le relevage auto et le pilotage joystick.
  - `robot_v5` et `v6` : versions intermédiaires.
- `Bancs_Essai_Tuning/` : firmwares de banc pour régler le PID par USB ou en wifi avec l'interface PC.
- `Sessions_Tests_300RPM/` : 17 sessions de tests enregistrées en JSON et résumés CSV.

## Réglages qui marchaient sur ces moteurs :
- Kp = 16.0
- Ki = 1.0
- Kd = 0.30
- Consigne = +0.50°
- pwmMin = 18
- Deadband = 2.0
- Kick relevage : PWM 180 pendant 350 ms max
