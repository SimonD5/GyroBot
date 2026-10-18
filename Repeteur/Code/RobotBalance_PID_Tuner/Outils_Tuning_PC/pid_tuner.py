"""
RobotBalance PID Tuner
======================

Interface graphique pour régler expérimentalement le PID du robot
ESP32-C6 + MPU6050 par USB.

Installation :
    pip install pyserial matplotlib

Lancement :
    python pid_tuner.py

Fonctions :
- connexion USB et détection du port
- réglage Kp / Ki / Kd / consigne / PWM min
- armement / désarmement
- affichage temps réel angle / gyro / PID / PWM / dt
- enregistrement de chaque essai
- analyse automatique
- recommandations de réglage
- historique de tous les essais
- export session JSON + CSV
- réimport d'une session précédente

IMPORTANT :
Le programme ne commande jamais automatiquement le robot pendant un essai.
Il propose des gains et attend que l'utilisateur les applique et lance le test.
"""

import csv
import json
import math
import os
import queue
import statistics
import threading
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import serial
from serial.tools import list_ports

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


BAUD = 921600
MAX_POINTS_PLOT = 2500


@dataclass
class Trial:
    number: int
    start_ms: int
    end_ms: int
    kp: float
    ki: float
    kd: float
    setpoint: float
    pwm_min: int
    duration: float
    max_angle_error: float
    rms_error: float
    max_rate: float
    max_pwm: float
    mean_pwm: float
    mean_dt: float
    max_dt: float
    oscillation_count: int
    outcome: str
    recommendation: str
    data: list


class PIDAnalyzer:
    """Analyse simple, volontairement robuste et lisible."""

    @staticmethod
    def analyze(data, kp, ki, kd, setpoint, pwm_min, trial_no):
        if not data:
            return None

        # Chaque ligne:
        # t, angle, acc, gyro, rateF, error, pid, pwm, dt,
        # active, armed, kp, ki, kd
        active_data = [x for x in data if x["active"]]
        if len(active_data) < 5:
            active_data = data

        duration = max(
            0.0,
            (active_data[-1]["t"] - active_data[0]["t"]) / 1000.0
        )

        errors = [x["error"] for x in active_data]
        rates = [x["gyro"] for x in active_data]
        pwms = [abs(x["pwm"]) for x in active_data]
        dts = [x["dt"] for x in active_data]

        rms = math.sqrt(sum(e * e for e in errors) / len(errors))
        max_error = max(abs(e) for e in errors)
        max_rate = max(abs(r) for r in rates)
        max_pwm = max(pwms)
        mean_pwm = statistics.mean(pwms)
        mean_dt = statistics.mean(dts)
        max_dt = max(dts)

        # Détection des passages de l'erreur par zéro.
        # On ignore les très petites variations autour de zéro.
        crossings = 0
        last_sign = 0
        for e in errors:
            if abs(e) < 0.25:
                continue
            sign = 1 if e > 0 else -1
            if last_sign and sign != last_sign:
                crossings += 1
            last_sign = sign

        # Diagnostic.
        recommendation = ""
        outcome = "INCONNU"

        # Chute = angle proche de la sécurité.
        last_angle = active_data[-1]["angle"]
        fell = max(abs(x["angle"]) for x in active_data) >= 17.0

        # Boucle lente / jitter.
        bad_dt = max_dt > 7.0 or mean_dt > 5.8

        if fell:
            outcome = "CHUTE"
        elif duration >= 8.0:
            outcome = "TRES BON"
        elif duration >= 5.0:
            outcome = "BON"
        elif duration >= 2.0:
            outcome = "MOYEN"
        else:
            outcome = "INSTABLE"

        # Logique de recommandation :
        # On privilégie d'abord la dynamique, puis la précision.
        if bad_dt:
            recommendation = (
                "Priorité : la boucle est trop lente/jittery "
                f"(moyenne {mean_dt:.2f} ms, max {max_dt:.2f} ms). "
                "Ne change pas encore le PID : corrige le timing."
            )
        elif ki > 0 and abs(statistics.mean(errors)) < 0.3 and duration < 3:
            recommendation = (
                "Ki n'apporte probablement rien ici. Remets Ki = 0 "
                "et règle d'abord Kp puis Kd."
            )
        elif crossings == 0 and duration < 5:
            recommendation = (
                "Pas assez d'oscillations observées. "
                "Si le robot tombe toujours du même côté, augmente Kp "
                "par petits pas."
            )
        elif crossings >= 3 and duration < 4:
            recommendation = (
                "Oscillations nombreuses/instables. "
                "Kp semble trop haut ou le D insuffisant. "
                "Premier essai conseillé : Kp -10 %, Kd inchangé."
            )
        elif crossings >= 2 and max_error > 4.0 and duration < 6:
            recommendation = (
                "Le robot corrige mais oscille encore fortement. "
                "Augmente Kd de 10 à 20 % avant de toucher à Ki."
            )
        elif duration >= 5 and rms > 1.5:
            recommendation = (
                "Bonne tenue mais erreur encore importante. "
                "Teste une petite hausse de Kd puis vérifie l'angle moyen."
            )
        elif duration >= 5 and rms <= 1.5:
            recommendation = (
                "Réglage prometteur. Fais 3 essais identiques pour vérifier "
                "la répétabilité avant de modifier les gains."
            )
        elif max_pwm >= 195:
            recommendation = (
                "Le contrôleur arrive presque à la saturation PWM. "
                "Avant d'augmenter Kp, vérifie la consigne, le sens moteur "
                "et la zone morte."
            )
        else:
            recommendation = (
                "Continue par petits pas. Change un seul gain à la fois "
                "et garde Ki = 0 pour l'instant."
            )

        return {
            "number": trial_no,
            "duration": duration,
            "max_error": max_error,
            "rms_error": rms,
            "max_rate": max_rate,
            "max_pwm": max_pwm,
            "mean_pwm": mean_pwm,
            "mean_dt": mean_dt,
            "max_dt": max_dt,
            "crossings": crossings,
            "outcome": outcome,
            "recommendation": recommendation,
            "fell": fell,
        }


class PIDTuner:
    def __init__(self, root):
        self.root = root
        self.root.title("RobotBalance — PID Tuner USB")
        self.root.geometry("1450x900")

        self.ser = None
        self.reader_thread = None
        self.reader_stop = threading.Event()
        self.rx_queue = queue.Queue()

        self.connected = False

        self.params = {
            "kp": 15.0,
            "ki": 0.0,
            "kd": 0.0,
            "setpoint": 7.5,
            "pwm_min": 20,
        }

        self.current_data = []
        self.current_trial_start = None
        self.current_trial_number = 0
        self.trials = []

        self.last_analysis = None
        self.last_active = False

        self.session_started = datetime.now().isoformat(timespec="seconds")

        self.build_ui()
        self.refresh_ports()
        self.update_gui()

        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # ---------------------------------------------------------
    # UI
    # ---------------------------------------------------------
    def build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="Port USB :").pack(side="left")

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            top, textvariable=self.port_var, width=20, state="readonly"
        )
        self.port_combo.pack(side="left", padx=5)

        ttk.Button(top, text="Actualiser", command=self.refresh_ports).pack(
            side="left"
        )
        ttk.Button(top, text="Connecter", command=self.connect).pack(
            side="left", padx=5
        )

        self.connection_label = ttk.Label(
            top, text="● Déconnecté", foreground="red"
        )
        self.connection_label.pack(side="left", padx=15)

        ttk.Button(top, text="Importer session", command=self.import_session).pack(
            side="right"
        )
        ttk.Button(top, text="Exporter session", command=self.export_session).pack(
            side="right", padx=5
        )

        # -----------------------------------------------------
        # Guide
        # -----------------------------------------------------
        guide = ttk.LabelFrame(self.root, text="Guide de réglage", padding=8)
        guide.pack(fill="x", padx=8, pady=4)

        self.step_label = ttk.Label(
            guide,
            text="ÉTAPE 1 — Connecte l'ESP32 en USB.",
            font=("Segoe UI", 12, "bold"),
        )
        self.step_label.pack(anchor="w")

        self.advice_label = ttk.Label(
            guide,
            text=(
                "Le robot doit être posé, roues libres. Ne l'arme pas encore. "
                "Le guide analysera chaque essai."
            ),
            wraplength=1300,
        )
        self.advice_label.pack(anchor="w", pady=4)

        self.recommendation_label = ttk.Label(
            guide,
            text="Aucune analyse pour l'instant.",
            wraplength=1300,
        )
        self.recommendation_label.pack(anchor="w", pady=4)

        # -----------------------------------------------------
        # Contrôles PID
        # -----------------------------------------------------
        controls = ttk.LabelFrame(self.root, text="Réglages", padding=8)
        controls.pack(fill="x", padx=8, pady=4)

        self.vars = {}
        fields = [
            ("Kp", "kp"),
            ("Ki", "ki"),
            ("Kd", "kd"),
            ("Consigne °", "setpoint"),
            ("PWM min", "pwm_min"),
        ]

        for label, key in fields:
            ttk.Label(controls, text=label).pack(side="left")
            var = tk.StringVar(value=str(self.params[key]))
            self.vars[key] = var
            ttk.Entry(controls, textvariable=var, width=9).pack(
                side="left", padx=(3, 12)
            )

        ttk.Button(
            controls, text="Appliquer PID", command=self.apply_params
        ).pack(side="left", padx=5)

        ttk.Button(
            controls, text="Réglage départ", command=self.reset_params
        ).pack(side="left")

        ttk.Button(
            controls, text="Gyro ZÉRO", command=lambda: self.send("z")
        ).pack(side="left", padx=5)

        ttk.Button(
            controls, text="Inverser gyro", command=lambda: self.send("g")
        ).pack(side="left")

        self.arm_button = ttk.Button(
            controls, text="ARMER", command=self.toggle_arm
        )
        self.arm_button.pack(side="right", padx=5)

        ttk.Button(
            controls, text="STOP MOTEURS", command=self.stop
        ).pack(side="right")

        # -----------------------------------------------------
        # Test roues
        # -----------------------------------------------------
        wheel = ttk.LabelFrame(
            self.root, text="Test roues — robot désarmé", padding=5
        )
        wheel.pack(fill="x", padx=8, pady=4)

        ttk.Label(wheel, text="PWM :").pack(side="left")
        for pwm in [20, 30, 40, 50, 60, 80]:
            ttk.Button(
                wheel,
                text=str(pwm),
                command=lambda p=pwm: self.send(f"v{p}"),
            ).pack(side="left", padx=2)

        # -----------------------------------------------------
        # Notebook
        # -----------------------------------------------------
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=8, pady=5)

        plot_tab = ttk.Frame(notebook)
        notebook.add(plot_tab, text="Courbes temps réel")

        history_tab = ttk.Frame(notebook)
        notebook.add(history_tab, text="Historique des essais")

        self.build_plots(plot_tab)
        self.build_history(history_tab)

    def build_plots(self, parent):
        self.figure = Figure(figsize=(11, 7), dpi=90)
        self.ax_angle = self.figure.add_subplot(311)
        self.ax_rate = self.figure.add_subplot(312)
        self.ax_pwm = self.figure.add_subplot(313)

        self.ax_angle.set_ylabel("Angle (°)")
        self.ax_rate.set_ylabel("Gyro (°/s)")
        self.ax_pwm.set_ylabel("PWM")
        self.ax_pwm.set_xlabel("Temps (s)")

        self.angle_line, = self.ax_angle.plot([], [], label="angle")
        self.setpoint_line, = self.ax_angle.plot([], [], label="consigne")
        self.rate_line, = self.ax_rate.plot([], [], label="gyro")
        self.pwm_line, = self.ax_pwm.plot([], [], label="PWM")

        self.ax_angle.legend(loc="upper right")

        self.figure.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.figure, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def build_history(self, parent):
        columns = (
            "N",
            "Kp",
            "Ki",
            "Kd",
            "durée",
            "RMS err",
            "err max",
            "gyro max",
            "PWM max",
            "dt moy",
            "dt max",
            "résultat",
        )

        self.tree = ttk.Treeview(
            parent, columns=columns, show="headings", height=15
        )

        widths = {
            "N": 40,
            "Kp": 70,
            "Ki": 70,
            "Kd": 70,
            "durée": 70,
            "RMS err": 80,
            "err max": 80,
            "gyro max": 85,
            "PWM max": 80,
            "dt moy": 70,
            "dt max": 70,
            "résultat": 110,
        }

        for c in columns:
            self.tree.heading(c, text=c)
            self.tree.column(c, width=widths[c], anchor="center")

        self.tree.pack(fill="both", expand=True)

        self.tree.bind("<<TreeviewSelect>>", self.select_trial)

        self.history_advice = ttk.Label(
            parent, text="Sélectionne un essai pour afficher son diagnostic.",
            wraplength=1300,
        )
        self.history_advice.pack(fill="x", padx=5, pady=8)

    # ---------------------------------------------------------
    # Serial
    # ---------------------------------------------------------
    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def connect(self):
        if self.connected:
            self.disconnect()
            return

        port = self.port_var.get()
        if not port:
            messagebox.showwarning("USB", "Sélectionne un port COM.")
            return

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=BAUD,
                timeout=0.05,
                write_timeout=0.2,
            )

            self.connected = True
            self.reader_stop.clear()

            self.reader_thread = threading.Thread(
                target=self.serial_reader, daemon=True
            )
            self.reader_thread.start()

            self.connection_label.config(
                text=f"● Connecté {port}", foreground="green"
            )

            # Début de la procédure : étape P seul.
            # On force réellement l'ESP32 à Ki=0 et Kd=0,
            # et on synchronise immédiatement l'interface.
            self.params["ki"] = 0.0
            self.params["kd"] = 0.0
            self.vars["ki"].set("0.0")
            self.vars["kd"].set("0.0")

            self.send(f"p{self.params['kp']}")
            self.send("i0")
            self.send("d0")
            self.send(f"s{self.params['setpoint']}")
            self.send(f"m{self.params['pwm_min']}")

            self.recommendation_label.config(
                text="PID initialisé automatiquement en P seul : Ki=0, Kd=0."
            )
            self.send("?")

        except Exception as e:
            messagebox.showerror("Connexion USB", str(e))

    def disconnect(self):
        self.reader_stop.set()
        self.connected = False

        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

        self.ser = None
        self.connection_label.config(
            text="● Déconnecté", foreground="red"
        )

    def serial_reader(self):
        while not self.reader_stop.is_set():
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode(
                        "utf-8", errors="ignore"
                    ).strip()

                    if line:
                        self.rx_queue.put(line)
                else:
                    time.sleep(0.001)

            except Exception as e:
                self.rx_queue.put(f"ERROR,{e}")
                break

    def send(self, command):
        if not self.connected or not self.ser:
            return

        try:
            self.ser.write((command + "\n").encode("ascii"))
        except Exception as e:
            messagebox.showerror("USB", str(e))

    # ---------------------------------------------------------
    # Paramètres
    # ---------------------------------------------------------
    def read_params_from_ui(self):
        try:
            return {
                "kp": float(self.vars["kp"].get()),
                "ki": float(self.vars["ki"].get()),
                "kd": float(self.vars["kd"].get()),
                "setpoint": float(self.vars["setpoint"].get()),
                "pwm_min": int(float(self.vars["pwm_min"].get())),
            }
        except ValueError:
            raise ValueError("Un des paramètres n'est pas un nombre.")

    def apply_params(self):
        try:
            p = self.read_params_from_ui()
        except ValueError as e:
            messagebox.showerror("Paramètres", str(e))
            return

        self.params = p

        # On force Ki=0 au début de la procédure.
        self.send(f"p{p['kp']}")
        self.send(f"i{p['ki']}")
        self.send(f"d{p['kd']}")
        self.send(f"s{p['setpoint']}")
        self.send(f"m{p['pwm_min']}")

        self.update_guide()

    def reset_params(self):
        self.params = {
            "kp": 15.0,
            "ki": 0.0,
            "kd": 0.0,
            "setpoint": 7.5,
            "pwm_min": 20,
        }

        for k, v in self.params.items():
            self.vars[k].set(str(v))

        self.apply_params()

    def toggle_arm(self):
        # Le robot est armé/désarmé explicitement.
        if self.arm_button["text"] == "ARMER":
            if not messagebox.askyesno(
                "Sécurité",
                "Robot prêt à tomber ?\n\n"
                "Tiens-le/assure-toi que les roues peuvent tourner librement "
                "et que rien ne peut être happé.\n\nARMER ?",
            ):
                return

            self.send("a1")
            self.arm_button.config(text="DÉSARMER")
        else:
            self.send("a0")
            self.arm_button.config(text="ARMER")

    def stop(self):
        self.send("q")
        self.arm_button.config(text="ARMER")

    # ---------------------------------------------------------
    # Données
    # ---------------------------------------------------------
    def parse_data(self, line):
        if not line.startswith("DATA,"):
            return

        parts = line.split(",")

        # DATA + 14 valeurs
        if len(parts) != 15:
            return

        try:
            x = {
                "t": int(parts[1]),
                "angle": float(parts[2]),
                "acc": float(parts[3]),
                "gyro": float(parts[4]),
                "rateF": float(parts[5]),
                "error": float(parts[6]),
                "pid": float(parts[7]),
                "pwm": int(parts[8]),
                "dt": float(parts[9]),
                "active": int(parts[10]),
                "armed": int(parts[11]),
                "kp": float(parts[12]),
                "ki": float(parts[13]),
                "kd": float(parts[14]),
            }
        except ValueError:
            return

        self.current_data.append(x)

        # Limite uniquement l'affichage, pas les données exportées.
        if len(self.current_data) > 20000:
            self.current_data = self.current_data[-20000:]

        # Détection du début d'un essai :
        # armed + active passe de faux à vrai.
        active = bool(x["active"])

        if active and not self.last_active:
            self.current_trial_start = len(self.current_data) - 1
            self.current_trial_number += 1

        # Fin d'essai : active -> false après avoir été actif.
        if not active and self.last_active:
            self.finish_trial()

        self.last_active = active

        # Gains réels venant de l'ESP32.
        self.params.update({
            "kp": x["kp"],
            "ki": x["ki"],
            "kd": x["kd"],
        })

    def finish_trial(self):
        if self.current_trial_start is None:
            return

        data = self.current_data[self.current_trial_start:]

        if len(data) < 5:
            self.current_trial_start = None
            return

        p = self.params

        analysis = PIDAnalyzer.analyze(
            data,
            p["kp"],
            p["ki"],
            p["kd"],
            p["setpoint"],
            p["pwm_min"],
            self.current_trial_number,
        )

        if not analysis:
            return

        trial = Trial(
            number=self.current_trial_number,
            start_ms=data[0]["t"],
            end_ms=data[-1]["t"],
            kp=p["kp"],
            ki=p["ki"],
            kd=p["kd"],
            setpoint=p["setpoint"],
            pwm_min=p["pwm_min"],
            duration=analysis["duration"],
            max_angle_error=analysis["max_error"],
            rms_error=analysis["rms_error"],
            max_rate=analysis["max_rate"],
            max_pwm=analysis["max_pwm"],
            mean_pwm=analysis["mean_pwm"],
            mean_dt=analysis["mean_dt"],
            max_dt=analysis["max_dt"],
            oscillation_count=analysis["crossings"],
            outcome=analysis["outcome"],
            recommendation=analysis["recommendation"],
            data=data,
        )

        self.trials.append(trial)
        self.last_analysis = analysis
        self.add_history_row(trial)

        self.recommendation_label.config(
            text=f"ESSAI {trial.number} — {trial.outcome}\n"
                 f"{trial.recommendation}"
        )

        self.current_trial_start = None
        self.update_guide()

    # ---------------------------------------------------------
    # Analyse / guide
    # ---------------------------------------------------------
    def update_guide(self):
        n = len(self.trials)

        if not self.connected:
            self.step_label.config(
                text="ÉTAPE 1 — Connecte l'ESP32 en USB."
            )
            self.advice_label.config(
                text="Choisis le port COM puis clique sur Connecter."
            )
            return

        if n == 0:
            self.step_label.config(
                text="ÉTAPE 2 — Vérifie l'IMU puis fais le premier essai P."
            )
            self.advice_label.config(
                text=(
                    "Clique sur GYRO ZÉRO avec le robot parfaitement immobile. "
                    "Ensuite commence avec Ki=0 et Kd=0. "
                    "Fais varier Kp progressivement. Le but est de trouver "
                    "la zone où le robot commence à osciller."
                )
            )
        elif n < 3:
            self.step_label.config(
                text="ÉTAPE 3 — Trouve le Kp critique."
            )
            self.advice_label.config(
                text=(
                    "Ne change qu'un seul paramètre. Pour l'instant Ki=0. "
                    "Si le robot tombe sans osciller : augmente Kp. "
                    "S'il oscille violemment : Kp est probablement trop haut."
                )
            )
        elif all(t.ki == 0 and t.kd == 0 for t in self.trials):
            self.step_label.config(
                text="ÉTAPE 3 — Continue le réglage de Kp avec P seul."
            )
            self.advice_label.config(
                text=(
                    "Ki et Kd sont maintenus automatiquement à 0. "
                    "Teste plusieurs Kp. Cherche la zone où le robot commence "
                    "à corriger sans partir immédiatement ni osciller violemment. "
                    "Ne passe au D qu'après avoir identifié une bonne valeur de Kp."
                )
            )
        elif all(t.ki == 0 for t in self.trials):
            self.step_label.config(
                text="ÉTAPE 4 — Ajoute progressivement le D."
            )
            self.advice_label.config(
                text=(
                    "Kp est maintenant à régler autour de la meilleure valeur P. "
                    "Augmente Kd par petits pas. Le D doit amortir les oscillations. "
                    "Ki reste automatiquement à 0."
                )
            )
        else:
            self.step_label.config(
                text="ÉTAPE 5 — Validation puis éventuellement Ki."
            )
            self.advice_label.config(
                text=(
                    "Si Kp/Kd donnent déjà plusieurs secondes de stabilité, "
                    "répète le même essai 3 fois. Ki ne doit être ajouté que "
                    "s'il reste une erreur statique persistante."
                )
            )

    def add_history_row(self, t):
        vals = (
            t.number,
            f"{t.kp:.2f}",
            f"{t.ki:.3f}",
            f"{t.kd:.2f}",
            f"{t.duration:.2f}",
            f"{t.rms_error:.2f}",
            f"{t.max_angle_error:.2f}",
            f"{t.max_rate:.1f}",
            f"{t.max_pwm:.0f}",
            f"{t.mean_dt:.2f}",
            f"{t.max_dt:.2f}",
            t.outcome,
        )
        self.tree.insert("", "end", values=vals)

    def select_trial(self, event=None):
        selection = self.tree.selection()
        if not selection:
            return

        index = self.tree.index(selection[0])
        if index >= len(self.trials):
            return

        trial = self.trials[index]

        self.history_advice.config(
            text=(
                f"Essai {trial.number} : {trial.outcome}\n"
                f"{trial.recommendation}"
            )
        )

        self.plot_data(trial.data)

    def plot_data(self, data):
        if not data:
            return

        d = data[-MAX_POINTS_PLOT:]
        t0 = d[0]["t"]
        x = [(v["t"] - t0) / 1000.0 for v in d]

        self.angle_line.set_data(x, [v["angle"] for v in d])
        self.setpoint_line.set_data(
            x, [self.params["setpoint"] for _ in d]
        )
        self.rate_line.set_data(x, [v["gyro"] for v in d])
        self.pwm_line.set_data(x, [v["pwm"] for v in d])

        for ax in [self.ax_angle, self.ax_rate, self.ax_pwm]:
            ax.relim()
            ax.autoscale_view()

        self.canvas.draw_idle()

    # ---------------------------------------------------------
    # Export / import
    # ---------------------------------------------------------
    def export_session(self):
        if not self.trials:
            messagebox.showinfo(
                "Export",
                "Aucun essai terminé à exporter."
            )
            return

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        path = filedialog.asksaveasfilename(
            title="Exporter la session",
            initialfile=f"PID_session_{stamp}.json",
            defaultextension=".json",
            filetypes=[("Session PID JSON", "*.json")],
        )

        if not path:
            return

        payload = {
            "format": "RobotBalance_PID_Tuner",
            "version": 1,
            "created": self.session_started,
            "exported": datetime.now().isoformat(timespec="seconds"),
            "baud": BAUD,
            "params": self.params,
            "trials": [asdict(t) for t in self.trials],
        }

        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)

        # CSV récapitulatif à côté du JSON.
        csv_path = os.path.splitext(path)[0] + "_resume.csv"

        with open(
            csv_path, "w", newline="", encoding="utf-8"
        ) as f:
            writer = csv.writer(f, delimiter=";")
            writer.writerow([
                "trial", "Kp", "Ki", "Kd", "setpoint", "pwmMin",
                "duration_s", "rms_error_deg", "max_error_deg",
                "max_rate_deg_s", "max_pwm", "mean_pwm",
                "mean_dt_ms", "max_dt_ms", "oscillations",
                "outcome", "recommendation"
            ])

            for t in self.trials:
                writer.writerow([
                    t.number, t.kp, t.ki, t.kd, t.setpoint, t.pwm_min,
                    t.duration, t.rms_error, t.max_angle_error,
                    t.max_rate, t.max_pwm, t.mean_pwm,
                    t.mean_dt, t.max_dt, t.oscillation_count,
                    t.outcome, t.recommendation
                ])

        messagebox.showinfo(
            "Export terminé",
            f"Session :\n{path}\n\nRésumé CSV :\n{csv_path}"
        )

    def import_session(self):
        path = filedialog.askopenfilename(
            title="Importer une session PID",
            filetypes=[("Session PID JSON", "*.json")],
        )

        if not path:
            return

        try:
            with open(path, "r", encoding="utf-8") as f:
                payload = json.load(f)

            if payload.get("format") != "RobotBalance_PID_Tuner":
                raise ValueError("Ce fichier n'est pas une session PID valide.")

            self.trials = [
                Trial(**t) for t in payload.get("trials", [])
            ]

            self.params.update(payload.get("params", {}))

            # Mettre les champs UI à jour.
            for k, v in self.params.items():
                if k in self.vars:
                    self.vars[k].set(str(v))

            # Reconstruire l'historique.
            for item in self.tree.get_children():
                self.tree.delete(item)

            for t in self.trials:
                self.add_history_row(t)

            if self.trials:
                self.plot_data(self.trials[-1].data)

                last = self.trials[-1]
                self.recommendation_label.config(
                    text=(
                        f"Dernier essai importé : {last.outcome}\n"
                        f"{last.recommendation}"
                    )
                )

            self.update_guide()

            messagebox.showinfo(
                "Import",
                f"{len(self.trials)} essai(s) importé(s)."
            )

        except Exception as e:
            messagebox.showerror("Import", str(e))

    # ---------------------------------------------------------
    # Update loop
    # ---------------------------------------------------------
    def update_gui(self):
        # Traiter les lignes reçues.
        while True:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break

            if line.startswith("DATA,"):
                self.parse_data(line)

            elif line.startswith("ERROR,"):
                self.connection_label.config(
                    text="● Erreur USB", foreground="red"
                )

        if self.current_data:
            self.plot_data(self.current_data)

        self.update_guide()

        self.root.after(50, self.update_gui)

    def close(self):
        self.stop()
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = PIDTuner(root)
    root.mainloop()


if __name__ == "__main__":
    main()
