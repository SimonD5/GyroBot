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
---

## 6. Analyse de la Session 20/10/2026 (Validation des Coefficients Optimaux)

La session `PID_session_v3_wifi_20261020_185039.json` (11 essais consécutifs de 45 secondes sans aucune chute, tous classés `TRES BON`) valide l'ensemble des réglages :

| Paramètre | Valeur Validée (500 RPM) | Performance mesurée |
|---|---|---|
| **$K_p$** | **13.0** | Précision d'équilibre sans le buzz 10 Hz |
| **$K_i$** | **1.0** | Élimination de l'erreur statique résiduelle |
| **$K_d$** | **0.10** | Amortissement fluide (vitesses angulaires max < 75°/s) |
| **Consigne** | **+1.35°** | Centre de masse réel (PWM moyen = -0.09, quasiment 0 net !) |
| **`pwmMin`** | **28** | Décollage mécanique des réducteurs 500 RPM |
| **$K_{p2}$** | **0.25** | Amortissement de la vitesse estimée |
| **$K_{i2}$** | **0.005 à 0.008** | **Dérive résiduelle quasiment nulle (-3.9 PWM*s sur 45s vs -805 PWM*s avant !)** |
| **Deadband** | **2.0 PWM** | Zone morte anti-tremblement |

### Comportement du léger balancement résiduel (~0.7 Hz) :
L'oscillation lente d'avant en arrière observée (fréquence mesurée à **0.67 Hz à 0.73 Hz**, amplitude $\pm 0.8°$) est le **cycle limite naturel incontournable** d'un robot équilibriste sans encodeurs optiques aux roues :
* La zone morte (`Deadband = 2.0`) laisse flotter le robot librement sur $\pm 0.8°$ sans intervention de la boucle 2.
* Dès que le robot dépasse ce seuil, le rappel $K_{i2}$ le ramène au centre.
* Compte tenu du jeu mécanique (backlash) des engrenages métalliques N20 et de l'absence d'encodeurs physiques, ce comportement est **optimal et représente la limite physique du système**.

---

## 7. Pilotage Télécommande : Comment Avancer / Reculer ?

1. **Jamais de PWM continu direct** : Imposer un PWM continu directement aux roues ferait chuter le pendule inversé instantanément.
2. **On incline l'angle de consigne ($\Delta \theta_{\text{joy}}$)** :
   $$\theta_{\text{cible}} = \theta_{\text{consigne}} + \Delta \theta_{\text{joy}}$$
   En poussant le joystick vers l'avant, le robot se penche en avant (jusqu'à $+1.2°$ ou $+2.5°$). La gravité l'attire vers l'avant et le PID fait tourner les roues pour le "rattraper", ce qui génère une avance fluide et continue.
3. **Gestion de l'intégrateur $I_2$ (Odométrie virtuelle)** :
   Pendant que le joystick est actionné ($|J_Y| > 0.05$), le firmware **réinitialise et gèle** l'odométrie virtuelle (`positionEstimee = 0`, `vitesseEstimee = 0`). Ainsi, l'intégrateur ne combat pas le joystick.
   Dès que le joystick revient au neutre, le robot freine, se redresse à $1.35°$, et le point où il s'est arrêté devient sa **nouvelle position d'ancrage** sans aucun effet ressort !
