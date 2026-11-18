# Moteurs 6V 500 RPM - Essais et réglages

Dossier pour les essais avec les moteurs N20 500 RPM (qui remplacent les anciens 300 RPM).

## Pourquoi ce changement ?
Les 300 RPM saturaient trop vite : dès qu'on voulait avancer un peu vite au joystick, le robot penchait et les moteurs n'allaient pas assez vite pour replacer les roues sous le centre de gravité.
Avec les 500 RPM, on a beaucoup plus de réserve de vitesse (~8 tr/s au lieu de 5), ce qui permet au robot de rouler sans tomber.

## Arborescence :
- `Firmwares_Autonomes/robot_v7_standalone_500rpm/` : code autonome final avec relevage auto et contrôle joystick par ESP-NOW.
- `Bancs_Essai_Tuning/robot_v4_antiderive_500rpm/` : banc wifi v4 avec compensation de la dérive (Ki2) et gains ajustés.
- `Sessions_Tests_500RPM/` : sessions de télémétrie enregistrées.

Attention : sur ces moteurs la polarité interne est inversée par rapport aux 300 RPM (`SIGN_MOTEUR_A = -1`, `SIGN_MOTEUR_B = +1`).

## Réglages retenus (validés lors de la session du 20/10) :
- `Kp = 13.0`
- `Ki = 1.0`
- `Kd = 0.10` (à 0.30 le robot vibrait violemment vers 10 Hz à cause du couple des 500 RPM, 0.10 c'est nickel)
- `Consigne = +1.35°` (angle d'équilibre naturel)
- `pwmMin = 28`
- `Kp2 = 0.25` / `Ki2 = ~0.006` (élimine la dérive au sol sur la durée)
- `Deadband = 2.0` (zone morte pour laisser du jeu sans oscillation)

Le robot a une légère oscillation naturelle résiduelle d'avant en arrière (vers 0.7 Hz), mais c'est normal vu le jeu mécanique dans les réducteurs N20 et le fait qu'on n'a pas d'encodeurs physiques sur les roues.
