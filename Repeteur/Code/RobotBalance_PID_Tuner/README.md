# RobotBalance — banc de réglage PID USB

## 1. Matériel

- ESP32-C6
- MPU6050 GY-521
- TB6612FNG
- 2 moteurs N20
- USB vers PC

Le Wi-Fi n'est plus utilisé.

## 2. Téléversement Arduino

Ouvre `robot_reglage_pid_USB.ino` dans Arduino IDE.

Baudrate série : **921600**.

Après téléversement :
1. laisse le robot parfaitement immobile ;
2. le gyro est calibré automatiquement ;
3. ouvre ensuite le programme Python.

## 3. Installer Python

Dans un terminal Windows :

```bash
py -m pip install pyserial matplotlib
```

Puis :

```bash
py pid_tuner.py
```

## 4. Première utilisation

Dans l'interface :

1. Choisir le COM de l'ESP32.
2. Cliquer `Connecter`.
3. Vérifier que l'ESP32 est bien détecté.
4. Poser le robot immobile.
5. Cliquer `GYRO ZÉRO` si nécessaire.
6. Commencer avec :
   - Ki = 0
   - Kd = 0
   - Kp progressivement
7. Armer uniquement lorsque le robot est prêt.

## 5. Méthode de réglage

### Phase A — P seul

Mettre :

```text
Ki = 0
Kd = 0
```

Augmenter Kp progressivement.

Objectif : trouver une valeur où le robot commence à réagir correctement mais devient proche de l'oscillation.

Ne pas chercher immédiatement 30 secondes de stabilité.

### Phase B — D

Garder Kp proche de la meilleure valeur trouvée.

Augmenter Kd par petits pas.

Le D doit réduire les oscillations.

Le code utilise :

```cpp
sortie = Kp * erreur + integrale - Kd * rateF;
```

Le signe `-` est volontaire : le gyro représente ici la vitesse de chute et le D doit s'y opposer.

### Phase C — I

Ne mettre Ki qu'en dernier.

Si Kp/Kd donnent déjà un robot stable, laisser :

```text
Ki = 0
```

L'intégrale est surtout utile si le robot garde une erreur statique persistante.

## 6. Données enregistrées

Chaque essai conserve :

- temps
- angle filtré
- angle accéléromètre
- gyro
- gyro filtré
- erreur
- sortie PID
- PWM
- dt de boucle
- état actif
- gains Kp/Ki/Kd

L'analyse calcule également :

- durée
- RMS de l'erreur
- erreur maximale
- vitesse angulaire maximale
- PWM maximum
- PWM moyen
- dt moyen
- dt maximum
- nombre d'oscillations
- diagnostic
- recommandation

## 7. Export

`Exporter session` crée :

```text
PID_session_YYYYMMDD_HHMMSS.json
PID_session_YYYYMMDD_HHMMSS_resume.csv
```

Le JSON contient les données brutes de tous les essais.

Le CSV contient le résumé.

Le JSON peut être réimporté plus tard avec `Importer session`.

Cela permet de continuer le réglage plusieurs jours plus tard sans perdre les essais précédents.
