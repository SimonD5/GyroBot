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
- **`Bancs_Essai_Tuning/robot_v4_antiderive_500rpm/`** :
  - **Banc Wi-Fi V4 (NOUVEAU)** : intègre l'odométrie virtuelle de position (`Ki2`) pour bloquer la dérive de 3-4 m, l'adoucissement de rampe (`ZONE_RAMPE = 45`) et les gains pré-ajustés anti-tremblement 10 Hz (`Kp=12.5`, `Kd=0.08`, `consigne=1.35°`, `pwmMin=28`).
- **`Bancs_Essai_Tuning/robot_v3_deadband_500rpm/`** :
  - Banc Wi-Fi V3 historique pour moteurs 500 RPM.
- **`Sessions_Tests_500RPM/`** :
  - Contient la session `184348` (première tenue réussie de 45s sur les essais 15 à 18).

---

## 3. Sens de Rotation des Moteurs 500 RPM

Contrairement aux moteurs 300 RPM, les moteurs 500 RPM ont une polarité interne inversée.  
Le logiciel applique donc les signes suivants :
- **Moteur A (Gauche)** : `SIGN_MOTEUR_A = -1`
- **Moteur B (Droit)** : `SIGN_MOTEUR_B = +1`

---

## 4. Affichage de l'Angle en Direct (PID Tuner V3)

Lorsque le robot est **désarmé** (au repos) :
- L'angle d'inclinaison réel est diffusé en continu (10 Hz) et s'affiche dans l'interface PC :  
  `📐 Angle actuel : +0.72°`
- **Utilisation** : Maintenez le robot à la main à son point d'équilibre naturel. L'angle affiché vous donne la valeur exacte de consigne à renseigner dans **`Consigne °`**.
- Un bouton rapide **`[ 📐 Consigne = Angle ]`** permet de recopier directement l'angle mesuré comme nouvelle consigne d'équilibre en un seul clic !

---

## 5. Analyse de la Session 184348 & Réglages V4

La session `PID_session_v3_wifi_20261018_184348.json` a permis d'obtenir les 4 premiers essais complets de 45 secondes (`TRES BON`, essais 15 à 18).

### Diagnostic des 2 problèmes observés :
1. **Tremblement résiduel (vibration à ~10 Hz)** :
   - Sur les essais 3-5 avec $K_d = 0.30$, le robot chutait en 2s (sursaturations dérivées).
   - En réduisant $K_d$ à $0.10$ puis $0.05$ (essai 16), la puissance des tremblements a chuté de moitié.
   - **Piste prioritaire** : Les moteurs 500 RPM réagissent plus vivement avec un couple immédiat. Le terme $K_p = 15.0$ est encore légèrement trop rigide et entretient le cycle à 10 Hz.
   - **Solution V4** : Régler $K_p$ entre **11.5 et 13.0** avec $K_d$ entre **0.06 et 0.08**, et adoucir la rampe de friction (`ZONE_RAMPE = 45.0` pour `pwmMin = 28`).
2. **Dérive permanente en arrière (3 à 4 mètres en 45s)** :
   - **Cause 1 (Consigne désaxée)** : La consigne était à `0.30°`, or l'analyse télémétrique montre que le centre de masse réel du robot avec les 500 RPM est à **`+1.35°` en moyenne**. Le robot était forcé de rouler en arrière en permanence pour ne pas basculer sur le dos.
   - **Cause 2 (Absence de rappel de position dans la boucle 2)** : En V3, $K_{i2}$ n'était pas actif, la boucle 2 ne compensait que la vitesse instantanée ($K_{p2} \cdot v$). Un correcteur P pur a toujours une erreur statique permanente (qui se traduit ici par une vitesse constante en arrière).
   - **Solution V4** : Activer l'odométrie virtuelle de position via $K_{i2}$ (valeur typique : **`0.008` à `0.015`**). Dès que le robot recule, l'intégrateur de position virtuelle s'incrémente négativement et pousse la consigne d'angle vers l'avant pour ramener le robot à sa position initiale.
