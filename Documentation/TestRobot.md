Test de la connexion et de l'accusé de réception
On envoie des commandes depuis la télécommande et on vérifie que l'ESP32 les reçoit et renvoie un accusé de réception. Le test est réussi si le point vert s'affiche sur la telecommande en continu.

Test du temps de réponse de la commande
La télécommande enregistre l'instant d'envoi d'une commande et l'instant d'arrivée de l'accusé de réception. La différence donne le temps de réponse, qui doit rester inférieur à 50 ms. On affiche les stat dans un onglet "Statistiques" de la télécommande et on controle la moyenne et l'ecart type. 

Test de portée de la télécommande
On éloigne progressivement la télécommande du robot, en intérieur puis en extérieur, avec et sans obstacle. On relève la distance une fois le contact perdu. Le test est validé si la liaison tient à plus de 30 m en intérieur et plus de 150 m en extérieur. On contrôle avec le point vert et le nombre de paquets perdus dans l'onglet stats de la télécommande.

Test de fiabilité des échanges
On laisse le robot fonctionner pendant 1 minutes, en le déplaçant, et on relève le nombre de paquets perdus dans l'onglet "Statistiques" de la télécommande, puis dans celui de l'IHM. Le test est réussi si le taux de perte reste sous de 5%.

Test du débit et de la portée vers l'ordinateur
On mesure le débit réel entre la Raspberry et l'ordinateur. Le débit doit rester supérieur à 2 Mbps.

Test de la portée du LiDAR
On place un obstacle à une distance croissante du robot, et on regarde s'il apparaît sur la carte de l'IHM. La portée maximale doit être de 10 m à ± 2 m. On contrôle la distance affichée sur l'IHM à la distance mesurée pour vérifier la fiabilité à chaque distance.

Test de précision de la carte
On fait cartographier une pièce ou un objet dont on a mesuré les dimensions au préalable. On compare ensuite les cotes de la carte affichée à celles de la réalité. Le test est validé si l'écart reste dans la tolérance du cahier des charges (5 cm, -3 / +5 cm).

Test de la latence d'affichage et du rafraîchissement de la carte
La Raspberry date chaque donnée à l'acquisition et l'ordinateur compare cet horodatage à l'heure d'affichage, après avoir synchronisé les deux. La latence doit être inférieure à 100 ms. Pour le rafraîchissement, on compte le nombre de mises à jour de la carte sur 10 s, qui doit donner 2 Hz ± 1.

Test d'équilibre
On laisse le robot à l'arrêt puis en mouvement pendant plusieurs minutes, en lui donnant de légères poussées. Le robot ne doit ni tomber ni glisser. On contrôle aussi qu'il monte et descende une pente supérieure à 30°.

Test de vitesse et de rayon de braquage
On trace une distance de 10 m et on chronomètre le parcours à vitesse maximale : 4 km/h correspond à environ 9 s, à ± 2 km/h. Pour le virage, on fait décrire un cercle au robot et on mesure son rayon (0,5 m ± 0,2 m). On contrôle l'absence de glissement de manière visuelle.

Test du poids
On pèse le robot complet, batterie comprise, avec une balance : il doit rester sous 1,5 kg. 

Test de remplacement de la batterie
Une personne qui n'a pas conçu le robot remplace la batterie en la chronométrant, sans outil. Le remplacement doit durer moins d'une minute. On contrôle que le robot redémarre correctement ensuite.

Test en simultané
On lance tout en même temps : équilibre, déplacement, commandes, LiDAR, caméra et envoi vers l'ordinateur. On surveille les onglets stats pendant plusieurs minutes. Le test est réussi si le robot ne tombe pas et si tous les niveaux précédents restent respectés.