import threading
import time
import math

from scanner import RPLidar


class Mapping:
    def __init__(self, port="COM6", baudrate=460800):
        self.port = port
        self.baudrate = baudrate

        self.lidar = None
        self.actif = False
        self.erreur = None

        self.points = []
        self.position = {
            "x": 0.0,
            "y": 0.0,
            "angle": 0.0
        }

        self.verrou = threading.Lock()
        self.thread = None

    def demarrer(self):
        """Démarre la lecture du LiDAR dans un thread séparé."""
        with self.verrou:
            if self.actif:
                return

            self.actif = True
            self.erreur = None

        self.thread = threading.Thread(
            target=self._lire_lidar,
            daemon=True
        )
        self.thread.start()

    def _lire_lidar(self):
        """Récupère continuellement les mesures du LiDAR."""
        try:
            self.lidar = RPLidar(
                self.port,
                baudrate=self.baudrate
            )

            for scan in self.lidar.iter_scans():
                with self.verrou:
                    if not self.actif:
                        break

                nouveaux_points = []

                for mesure in scan:
                    # Format habituel : qualité, angle en degrés,
                    # distance en millimètres.
                    qualite, angle, distance_mm = mesure

                    # Ignore les mesures nulles ou invalides.
                    if distance_mm <= 0:
                        continue

                    distance_m = distance_mm / 1000.0
                    angle_rad = math.radians(angle)

                    # Coordonnées dans le repère du LiDAR.
                    x = distance_m * math.cos(angle_rad)
                    y = distance_m * math.sin(angle_rad)

                    nouveaux_points.append({
                        "angle": round(float(angle), 2),
                        "distance": round(distance_m, 3),
                        "x": round(x, 3),
                        "y": round(y, 3)
                    })

                with self.verrou:
                    self.points = nouveaux_points

        except Exception as erreur:
            with self.verrou:
                self.erreur = str(erreur)

        finally:
            with self.verrou:
                self.actif = False

            if self.lidar is not None:
                try:
                    self.lidar.stop()
                except Exception:
                    pass

                try:
                    self.lidar.stop_motor()
                except Exception:
                    pass

                try:
                    self.lidar.disconnect()
                except Exception:
                    pass

                self.lidar = None

    def arreter(self):
        """Demande l'arrêt du scan."""
        with self.verrou:
            self.actif = False

    def obtenir_donnees(self):
        """Renvoie les dernières mesures pour l'interface web."""
        with self.verrou:
            return {
                "actif": self.actif,
                "erreur": self.erreur,
                "position": self.position.copy(),
                "points": list(self.points),
                "mise_a_jour": time.time()
            }


# Instance partagée avec le serveur Flask.
mapping = Mapping()