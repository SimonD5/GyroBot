# Codes de test – Gyrobot Répéteur

Ce dossier contient **2 codes de test** développés pour valider les composants principaux du gyrobot.

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

## Structure du dossier

```
Code/
├── README.md          ← ce fichier
├── test_moteurs/      ← test PWM moteurs (max 200/255)
└── test_imu/          ← test accélérations & vitesses angulaires IMU
```
