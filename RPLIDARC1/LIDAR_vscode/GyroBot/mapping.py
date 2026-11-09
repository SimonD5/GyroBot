
import threading
import time
import math
import asyncio

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
        try:
            asyncio.run(self._gerer_scan())

        except Exception as erreur:
            with self.verrou:
                self.erreur = repr(erreur)

        finally:
            if self.lidar is not None:
                try:
                    self.lidar.shutdown()
                except Exception as erreur:
                    print("Erreur fermeture LiDAR :", erreur)

                self.lidar = None

            with self.verrou:
                self.actif = False

    async def _gerer_scan(self):
        self.lidar = RPLidar(
            self.port,
            baudrate=self.baudrate
        )

        derniere_angle = None
        points_tour = []

        # Lance le scan asynchrone
        tache_scan = asyncio.create_task(
            self.lidar.simple_scan()
        )

        try:
            while self.actif:
                try:
                    mesure = await asyncio.wait_for(
                        self.lidar.output_queue.get(),
                        timeout=1.0
                    )
                except asyncio.TimeoutError:
                    if tache_scan.done():
                        exception = tache_scan.exception()
                        if exception:
                            raise exception
                        raise RuntimeError("La tâche de scan s'est terminée.")
                    continue

                angle = float(mesure["a_deg"])
                distance_mm = mesure["d_mm"]

                if distance_mm is None or distance_mm <= 0:
                    continue

                distance_m = distance_mm / 1000.0
                angle_rad = math.radians(angle)

                x = distance_m * math.cos(angle_rad)
                y = distance_m * math.sin(angle_rad)

                # Détection du passage de 360° à 0°
                if derniere_angle is not None:
                    if angle < derniere_angle - 180:
                        with self.verrou:
                            self.points = points_tour.copy()
                        points_tour = []

                points_tour.append({
                    "angle": round(angle, 2),
                    "distance": round(distance_m, 3),
                    "x": round(x, 3),
                    "y": round(y, 3)
                })

                derniere_angle = angle

        finally:
            self.lidar.stop_event.set()

            try:
                await asyncio.wait_for(tache_scan, timeout=3.0)
            except (asyncio.TimeoutError, Exception):
                tache_scan.cancel()
                try:
                    await tache_scan
                except BaseException:
                    pass

    def arreter(self):
        with self.verrou:
            self.actif = False

    def obtenir_donnees(self):
        with self.verrou:
            return {
                "actif": self.actif,
                "erreur": self.erreur,
                "position": self.position.copy(),
                "points": list(self.points),
                "mise_a_jour": time.time()
            }


mapping = Mapping()
