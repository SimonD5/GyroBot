# ⚠️ ATTENTION : MOTEURS 6 V

**Avec une LiPo 3S (11,1 V), limiter le PWM à 138 maximum (sur 255) !**

Calcul : `PWM_max = 6 × 255 / 11,1 ≈ 138`

Les moteurs sont prévus pour **6 V**. Cette limite de PWM ne garantit toutefois pas que la tension instantanée reste sous 6 V. 

- Consommation au démarrage : **au moins 0,6 A pour les deux moteurs simultanément à 6v d'entré**.
