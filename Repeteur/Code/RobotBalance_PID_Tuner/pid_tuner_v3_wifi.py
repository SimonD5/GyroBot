"""
RobotBalance PID Tuner — version WiFi
======================================

Interface graphique pour régler expérimentalement le PID du robot
ESP32-C6 + MPU6050 via WiFi (SoftAP du robot).

PRINCIPE DU FIRMWARE WIFI :
  Le WiFi n'est actif QUE lorsque le robot est tombé / en attente.
  La boucle PID tourne sans aucune activité radio (200 Hz propre).

  Cycle :
    1. COMMS  : WiFi allumé. Le PC se connecte, reçoit le dernier essai,
                règle les gains, puis envoie "a1" pour armer.
    2. ARMÉ   : WiFi COUPÉ. Le robot attend d'être redressé.
    3. ACTIF  : PID tourne, télémétrie stockée en RAM (pas de streaming).
    4. retour à COMMS : WiFi rallumé, l'essai complet est envoyé au PC.

RÉSEAU :
  SSID : RobotPID   mot de passe : robot1234
  IP du robot : 192.168.4.1   port TCP : 3333

PROTOCOLE ROBOT -> PC :
  INFO,texte
  PARAM,Kp=..,Ki=..,Kd=..,consigne=..,pwmMin=..,gyroSign=..,armed=..,pending=..,cap=..
  ARMING
  TRIAL,num,nbEchantillons,Kp,Ki,Kd,consigne,pwmMin,raison
  D,t_ms,angle*100,acc*100,gyro*10,rateF*10,pid*10,pwm,dt_us
  END,num

INSTALLATION :
    pip install matplotlib

LANCEMENT :
    python pid_tuner_wifi.py
    ou double-clic sur lancer_pid_tuner_wifi.bat
"""

import csv
import json
import math
import os
import queue
import socket
import statistics
import threading
import time
from dataclasses import dataclass, asdict
from datetime import datetime

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ── Réseau ────────────────────────────────────────────────────────────────────
ROBOT_IP   = "192.168.4.1"
ROBOT_PORT = 3333
TCP_TIMEOUT  = 5.0    # secondes pour la connexion initiale
RECV_TIMEOUT = 0.05   # secondes de lecture non bloquante

MAX_POINTS_PLOT = 9500


# ── Dataclasses ───────────────────────────────────────────────────────────────
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
    kp2: float = 0.0
    ki2: float = 0.0


# ── Analyseur PID ─────────────────────────────────────────────────────────────
class PIDAnalyzer:
    """Analyse simple, volontairement robuste et lisible."""

    @staticmethod
    def analyze(data, kp, ki, kd, setpoint, pwm_min, trial_no):
        if not data:
            return None

        duration = max(
            0.0,
            (data[-1]["t"] - data[0]["t"]) / 1000.0
        )

        errors = [x["error"] for x in data]
        rates  = [x["gyro"]  for x in data]
        pwms   = [abs(x["pwm"]) for x in data]
        dts    = [x["dt"]    for x in data]   # dt en ms

        rms       = math.sqrt(sum(e * e for e in errors) / len(errors))
        max_error = max(abs(e) for e in errors)
        max_rate  = max(abs(r) for r in rates)
        max_pwm   = max(pwms)
        mean_pwm  = statistics.mean(pwms)
        mean_dt   = statistics.mean(dts)
        max_dt    = max(dts)

        # Passages par zéro de l'erreur
        crossings = 0
        last_sign = 0
        for e in errors:
            if abs(e) < 0.25:
                continue
            sign = 1 if e > 0 else -1
            if last_sign and sign != last_sign:
                crossings += 1
            last_sign = sign

        fell   = max(abs(x["angle"]) for x in data) >= 17.0
        bad_dt = max_dt > 10.0 or mean_dt > 10.0

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
            "number":         trial_no,
            "duration":       duration,
            "max_error":      max_error,
            "rms_error":      rms,
            "max_rate":       max_rate,
            "max_pwm":        max_pwm,
            "mean_pwm":       mean_pwm,
            "mean_dt":        mean_dt,
            "max_dt":         max_dt,
            "crossings":      crossings,
            "outcome":        outcome,
            "recommendation": recommendation,
            "fell":           fell,
        }


# ── Application principale ────────────────────────────────────────────────────
class PIDTunerWifi:
    def __init__(self, root):
        self.root = root
        self.root.title("RobotBalance — PID Tuner WiFi")
        self.root.geometry("1450x950")

        # ── Socket TCP ──────────────────────────────────────────────────────
        self._sock = None
        self._reader_thread = None
        self._reader_stop = threading.Event()
        self.rx_queue = queue.Queue()

        self.connected = False

        # ── Paramètres PID ──────────────────────────────────────────────────
        self.params = {
            "kp":       15.0,
            "ki":       1.0,
            "kd":       0.30,
            "setpoint": 2.5,
            "pwm_min":  11,
            "kp2":      0.05,
            "deadband":  2.0,
            "ki2":      0.0,
        }

        # ── État interne ────────────────────────────────────────────────────
        self.current_data = []
        self.current_trial_number = 0
        self.trials = []
        self.last_analysis = None

        # Essai en cours de réception (entre TRIAL et END)
        self._incoming_trial = None

        # Indicateur "essai en attente" reçu dans PARAM
        self._pending_trial = False

        self.session_started = datetime.now().isoformat(timespec="seconds")

        self._build_ui()
        self._update_gui()

        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # ── Construction de l'interface ──────────────────────────────────────────
    def _build_ui(self):
        # --- Barre de connexion ---
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="IP robot :").pack(side="left")
        self._ip_var = tk.StringVar(value=ROBOT_IP)
        ttk.Entry(top, textvariable=self._ip_var, width=15).pack(side="left", padx=4)

        ttk.Label(top, text="Port :").pack(side="left")
        self._port_var = tk.StringVar(value=str(ROBOT_PORT))
        ttk.Entry(top, textvariable=self._port_var, width=7).pack(side="left", padx=4)

        ttk.Button(top, text="Connecter",    command=self._connect).pack(side="left", padx=8)
        ttk.Button(top, text="Déconnecter", command=self._disconnect).pack(side="left")

        self._conn_label = ttk.Label(top, text="● Déconnecté", foreground="red")
        self._conn_label.pack(side="left", padx=15)

        self._status_label = ttk.Label(top, text="État robot : —")
        self._status_label.pack(side="left", padx=10)

        ttk.Button(top, text="Importer session", command=self._import_session).pack(side="right")
        ttk.Button(top, text="Exporter session", command=self._export_session).pack(side="right", padx=5)

        # --- Guide ---
        guide = ttk.LabelFrame(self.root, text="Guide de réglage", padding=8)
        guide.pack(fill="x", padx=8, pady=4)

        self._step_label = ttk.Label(
            guide,
            text="ÉTAPE 1 — Connecte-toi au WiFi du robot (SSID : RobotPID  MDP : robot1234) puis clique Connecter.",
            font=("Segoe UI", 12, "bold"),
        )
        self._step_label.pack(anchor="w")

        self._advice_label = ttk.Label(
            guide,
            text=(
                "Le robot doit être posé, roues libres. Ne l'arme pas encore. "
                "Le guide analysera chaque essai après réception complète."
            ),
            wraplength=1350,
        )
        self._advice_label.pack(anchor="w", pady=4)

        self._recommendation_label = ttk.Label(
            guide,
            text="Aucune analyse pour l'instant.",
            wraplength=1350,
        )
        self._recommendation_label.pack(anchor="w", pady=4)

        # --- Contrôles PID ---
        controls = ttk.LabelFrame(self.root, text="Réglages", padding=8)
        controls.pack(fill="x", padx=8, pady=4)

        self._vars = {}
        fields = [
            ("Kp",          "kp"),
            ("Ki",          "ki"),
            ("Kd",          "kd"),
            ("Consigne °",  "setpoint"),
            ("PWM min",     "pwm_min"),
            ("Kp2 (Vitesse)","kp2"),
            ("Ki2 (Centre)","ki2"),
            ("Deadband (PWM)", "deadband"),
        ]

        for label, key in fields:
            ttk.Label(controls, text=label).pack(side="left")
            var = tk.StringVar(value=str(self.params[key]))
            self._vars[key] = var
            ttk.Entry(controls, textvariable=var, width=9).pack(side="left", padx=(3, 12))

        ttk.Button(controls, text="Appliquer PID",   command=self._apply_params).pack(side="left", padx=5)
        ttk.Button(controls, text="Réglage départ",  command=self._reset_params).pack(side="left")
        ttk.Button(controls, text="Gyro ZÉRO",       command=lambda: self._send("z")).pack(side="left", padx=5)
        ttk.Button(controls, text="Inverser gyro",   command=lambda: self._send("g")).pack(side="left")

        self._arm_button = ttk.Button(controls, text="ARMER", command=self._toggle_arm)
        self._arm_button.pack(side="right", padx=5)
        ttk.Button(controls, text="STOP MOTEURS", command=self._stop).pack(side="right")

        # --- Test roues rapide ---
        wheel = ttk.LabelFrame(self.root, text="Test roues rapide — robot désarmé", padding=5)
        wheel.pack(fill="x", padx=8, pady=4)

        ttk.Label(wheel, text="PWM rapide :").pack(side="left")
        for pwm in [20, 30, 40, 50, 60, 80]:
            ttk.Button(wheel, text=str(pwm),
                       command=lambda p=pwm: self._send(f"v{p}")).pack(side="left", padx=2)

        # --- Test PWM fin ---
        pwm_fine = ttk.LabelFrame(self.root, text="Test PWM fin — réglage précis", padding=5)
        pwm_fine.pack(fill="x", padx=8, pady=(0, 4))

        # Slider PWM
        ttk.Label(pwm_fine, text="PWM :").pack(side="left")
        self._pwm_fine_var = tk.IntVar(value=30)
        self._pwm_slider = ttk.Scale(
            pwm_fine, from_=0, to=200,
            orient="horizontal", length=300,
            variable=self._pwm_fine_var,
            command=self._on_pwm_slider,
        )
        self._pwm_slider.pack(side="left", padx=6)

        # Champ numérique
        self._pwm_entry_var = tk.StringVar(value="30")
        pwm_entry = ttk.Entry(pwm_fine, textvariable=self._pwm_entry_var, width=5)
        pwm_entry.pack(side="left", padx=4)
        pwm_entry.bind("<Return>", self._on_pwm_entry)
        pwm_entry.bind("<FocusOut>", self._on_pwm_entry)

        # Durée
        ttk.Label(pwm_fine, text="Durée (s) :").pack(side="left", padx=(12, 2))
        self._pwm_dur_var = tk.StringVar(value="1.0")
        ttk.Entry(pwm_fine, textvariable=self._pwm_dur_var, width=5).pack(side="left", padx=4)

        # Boutons
        ttk.Button(
            pwm_fine, text="▶ Lancer test",
            command=self._send_pwm_fine,
        ).pack(side="left", padx=8)

        ttk.Button(
            pwm_fine, text="■ Stop",
            command=lambda: self._send("q"),
        ).pack(side="left")

        self._pwm_fine_label = ttk.Label(pwm_fine, text="PWM : 30")
        self._pwm_fine_label.pack(side="left", padx=12)

        # --- Notebook ---
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=8, pady=5)

        plot_tab    = ttk.Frame(notebook)
        history_tab = ttk.Frame(notebook)
        log_tab     = ttk.Frame(notebook)

        notebook.add(plot_tab,    text="Courbes dernier essai")
        notebook.add(history_tab, text="Historique des essais")
        notebook.add(log_tab,     text="Log TCP brut")

        self._build_plots(plot_tab)
        self._build_history(history_tab)
        self._build_log(log_tab)

    def _build_plots(self, parent):
        self._figure   = Figure(figsize=(11, 7), dpi=90)
        self._ax_angle = self._figure.add_subplot(311)
        self._ax_rate  = self._figure.add_subplot(312)
        self._ax_pwm   = self._figure.add_subplot(313)

        self._ax_angle.set_ylabel("Angle (°)")
        self._ax_rate.set_ylabel("Gyro (°/s)")
        self._ax_pwm.set_ylabel("PWM")
        self._ax_pwm.set_xlabel("Temps (s)")

        self._angle_line,    = self._ax_angle.plot([], [], label="angle")
        self._setpoint_line, = self._ax_angle.plot([], [], label="consigne", linestyle="--")
        self._rate_line,     = self._ax_rate.plot([], [], label="gyro")
        self._pwm_line,      = self._ax_pwm.plot([], [], label="PWM")

        self._ax_angle.legend(loc="upper right")
        self._figure.tight_layout()

        self._canvas = FigureCanvasTkAgg(self._figure, master=parent)
        self._canvas.get_tk_widget().pack(fill="both", expand=True)

    def _build_history(self, parent):
        columns = (
            "N", "Kp", "Ki", "Kd", "Kp2", "Ki2", "durée", "RMS err",
            "err max", "gyro max", "PWM max", "dt moy", "dt max", "résultat",
        )
        self._tree = ttk.Treeview(parent, columns=columns, show="headings", height=15)

        widths = {
            "N": 35, "Kp": 55, "Ki": 55, "Kd": 55, "Kp2": 55, "Ki2": 55, "durée": 65,
            "RMS err": 70, "err max": 70, "gyro max": 75,
            "PWM max": 70, "dt moy": 65, "dt max": 65, "résultat": 100,
        }

        for c in columns:
            self._tree.heading(c, text=c)
            self._tree.column(c, width=widths[c], anchor="center")

        self._tree.pack(fill="both", expand=True)
        self._tree.bind("<<TreeviewSelect>>", self._select_trial)

        self._history_advice = ttk.Label(
            parent,
            text="Sélectionne un essai pour afficher son diagnostic.",
            wraplength=1350,
        )
        self._history_advice.pack(fill="x", padx=5, pady=8)

    def _build_log(self, parent):
        frame = ttk.Frame(parent)
        frame.pack(fill="both", expand=True)

        sb = ttk.Scrollbar(frame)
        sb.pack(side="right", fill="y")

        self._log_text = tk.Text(
            frame, yscrollcommand=sb.set, state="disabled",
            font=("Consolas", 9), wrap="none"
        )
        self._log_text.pack(fill="both", expand=True)
        sb.config(command=self._log_text.yview)

    # ── Connexion TCP ─────────────────────────────────────────────────────────
    def _connect(self):
        if self.connected:
            messagebox.showinfo("WiFi", "Déjà connecté.")
            return

        ip = self._ip_var.get().strip()
        try:
            port = int(self._port_var.get().strip())
        except ValueError:
            messagebox.showerror("WiFi", "Port invalide.")
            return

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(TCP_TIMEOUT)
            sock.connect((ip, port))
            sock.settimeout(RECV_TIMEOUT)
        except OSError as exc:
            messagebox.showerror(
                "Connexion WiFi",
                f"Impossible de joindre {ip}:{port}\n\n{exc}\n\n"
                "Vérifie que le PC est connecté au réseau WiFi du robot\n"
                "(SSID : RobotPID, MDP : robot1234)"
            )
            return

        self._sock = sock
        self.connected = True
        self._reader_stop.clear()

        self._reader_thread = threading.Thread(target=self._tcp_reader, daemon=True)
        self._reader_thread.start()

        self._conn_label.config(text=f"● Connecté {ip}:{port}", foreground="green")

        # Synchronisation avec les paramètres actuels de l'interface
        try:
            self.params = self._read_params_from_ui()
        except Exception:
            pass

        self._send(f"p{self.params['kp']}")
        self._send(f"i{self.params['ki']}")
        self._send(f"d{self.params['kd']}")
        self._send(f"s{self.params['setpoint']}")
        self._send(f"m{self.params['pwm_min']}")
        if "kp2" in self.params:
            self._send(f"k{self.params['kp2']}")
        if "ki2" in self.params:
            self._send(f"j{self.params['ki2']}")
        self._send("?")

        self._recommendation_label.config(
            text="Connecté au robot. Paramètres synchronisés."
        )

    def _disconnect(self):
        self._reader_stop.set()
        self.connected = False
        try:
            if self._sock:
                self._sock.close()
        except OSError:
            pass
        self._sock = None
        self._conn_label.config(text="● Déconnecté", foreground="red")
        self._status_label.config(text="État robot : —")

    def _tcp_reader(self):
        """Lit les lignes TCP en arrière-plan et les pousse dans rx_queue."""
        buf = b""
        while not self._reader_stop.is_set():
            try:
                chunk = self._sock.recv(4096)
                if not chunk:
                    self.rx_queue.put("ERROR,connexion fermée par le robot")
                    break
                buf += chunk
                while b"\n" in buf:
                    idx = buf.index(b"\n")
                    line = buf[:idx].rstrip(b"\r").decode("utf-8", errors="ignore")
                    buf = buf[idx + 1:]
                    if line:
                        self.rx_queue.put(line)
            except socket.timeout:
                continue
            except OSError as e:
                if not self._reader_stop.is_set():
                    self.rx_queue.put(f"ERROR,{e}")
                break

    def _send(self, command):
        """Envoie une commande au robot (ligne terminée par newline)."""
        if not self.connected or self._sock is None:
            return
        try:
            self._sock.sendall((command + "\n").encode("ascii"))
        except OSError as e:
            messagebox.showerror("WiFi TX", str(e))

    # ── Paramètres ────────────────────────────────────────────────────────────
    def _read_params_from_ui(self):
        try:
            return {
                "kp":       float(self._vars["kp"].get()),
                "ki":       float(self._vars["ki"].get()),
                "kd":       float(self._vars["kd"].get()),
                "setpoint": float(self._vars["setpoint"].get()),
                "pwm_min":  int(float(self._vars["pwm_min"].get())),
                "kp2":      float(self._vars["kp2"].get()),
                "ki2":      float(self._vars["ki2"].get()),
                "deadband":  float(self._vars["deadband"].get()),
            }
        except ValueError:
            raise ValueError("Un des paramètres n'est pas un nombre.")

    def _apply_params(self):
        try:
            p = self._read_params_from_ui()
        except ValueError as e:
            messagebox.showerror("Paramètres", str(e))
            return
        self.params = p
        self._send(f"p{p['kp']}")
        self._send(f"i{p['ki']}")
        self._send(f"d{p['kd']}")
        self._send(f"s{p['setpoint']}")
        self._send(f"m{p['pwm_min']}")
        self._send(f"k{p['kp2']}")
        self._send(f"j{p['ki2']}")
        self._send(f"b{p['deadband']}")
        self._update_guide()

    def _reset_params(self):
        self.params = {"kp": 15.0, "ki": 1.0, "kd": 0.30, "setpoint": 2.5, "pwm_min": 11, "kp2": 0.05, "ki2": 0.0, "deadband": 2.0}
        for k, v in self.params.items():
            self._vars[k].set(str(v))
        self._apply_params()

    def _toggle_arm(self):
        if self._arm_button["text"] == "ARMER":
            if self._pending_trial:
                messagebox.showwarning(
                    "Armement refusé",
                    "Le robot signale un essai non encore transmis.\n"
                    "Attends la réception complète de l'essai précédent."
                )
                return
            if not messagebox.askyesno(
                "Sécurité",
                "Robot prêt à tomber ?\n\n"
                "Tiens-le / assure-toi que les roues peuvent tourner librement "
                "et que rien ne peut être happé.\n\nARMER ?",
            ):
                return

            self._send("a1")
            self._arm_button.config(text="DÉSARMER")
            # En V3, le WiFi et le socket TCP restent OUVERTS en continu !
            self._status_label.config(text="État robot : ARMÉ — En attente mise à la verticale")
            self._recommendation_label.config(
                text="Robot armé avec WiFi persistant. Place le robot à la verticale (proche de 2.5°) pour démarrer l'essai."
            )
        else:
            self._send("a0")
            self._arm_button.config(text="ARMER")

    def _on_arming(self):
        """En V3, le WiFi reste connecté."""
        pass

    def _stop(self):
        self._send("q")
        self._arm_button.config(text="ARMER")

    # ── Test PWM fin ─────────────────────────────────────────────────────────
    def _on_pwm_slider(self, value):
        """Synchronise le champ numérique quand le slider bouge."""
        v = int(float(value))
        self._pwm_fine_var.set(v)
        self._pwm_entry_var.set(str(v))
        self._pwm_fine_label.config(text=f"PWM : {v}")

    def _on_pwm_entry(self, event=None):
        """Synchronise le slider quand on tape une valeur dans le champ."""
        try:
            v = int(float(self._pwm_entry_var.get()))
            v = max(0, min(200, v))
            self._pwm_fine_var.set(v)
            self._pwm_entry_var.set(str(v))
            self._pwm_fine_label.config(text=f"PWM : {v}")
        except ValueError:
            pass

    def _send_pwm_fine(self):
        """Envoie la commande v<PWM> avec la durée choisie.

        Le firmware accepte 'v<pwm>' pour 800 ms fixes.  Pour une durée
        personnalisée on envoie la commande plusieurs fois en séquence
        (chaque 'v' relance le timer interne à 800 ms), ou on utilise
        directement 'v<pwm>' si la durée <= 0.8 s.
        """
        try:
            dur = float(self._pwm_dur_var.get())
        except ValueError:
            dur = 0.8

        pwm = self._pwm_fine_var.get()

        # Nombre de rafales de 800 ms nécessaires pour couvrir la durée.
        # On envoie la première immédiatement ; les suivantes sont planifiées.
        n_shots = max(1, round(dur / 0.8))

        def _fire(remaining):
            if remaining <= 0:
                return
            self._send(f"v{pwm}")
            if remaining > 1:
                self.root.after(750, lambda: _fire(remaining - 1))

        _fire(n_shots)

    # ── Parsing du protocole WiFi ─────────────────────────────────────────────
    def _parse_line(self, line):
        """Traite une ligne reçue du robot."""
        self._log(line)

        if line.startswith("INFO,"):
            msg = line[5:]
            if "READY" in msg:
                self._status_label.config(text="État robot : PRÊT (COMMS)")
            return

        if line.startswith("PARAM,"):
            self._parse_param(line)
            return

        if line == "ARMING":
            self._status_label.config(text="État robot : ARMÉ")
            return

        if line.startswith("TRIAL,"):
            self._parse_trial_header(line)
            return

        if line.startswith("D,"):
            self._parse_data_point(line)
            return

        if line.startswith("END,"):
            self._finish_incoming_trial()
            return

        if line.startswith("ERROR,"):
            self._conn_label.config(text="● Erreur TCP", foreground="red")
            self.connected = False
            return

    def _parse_param(self, line):
        """PARAM,Kp=..,Ki=..,Kd=..,consigne=..,pwmMin=..,gyroSign=..,armed=..,pending=..,cap=.."""
        mapping = {
            "Kp":       ("kp",       float),
            "Ki":       ("ki",       float),
            "Kd":       ("kd",       float),
            "consigne": ("setpoint", float),
            "pwmMin":   ("pwm_min",  int),
            "Kp2":      ("kp2",      float),
            "Ki2":      ("ki2",      float),
            "dead":      ("deadband",  float),
        }

        parts = line[len("PARAM,"):].split(",")
        for part in parts:
            if "=" not in part:
                continue
            key, val = part.split("=", 1)
            key = key.strip()
            val = val.strip()

            if key in mapping:
                field, conv = mapping[key]
                try:
                    self.params[field] = conv(val)
                    self._vars[field].set(str(self.params[field]))
                except ValueError:
                    pass

            if key == "pending":
                self._pending_trial = (val == "1")

            if key == "armed":
                if val == "1":
                    self._status_label.config(text="État robot : ARMÉ")
                else:
                    self._status_label.config(text="État robot : COMMS (prêt)")

    def _parse_trial_header(self, line):
        """TRIAL,num,nbEchantillons,Kp,Ki,Kd,consigne,pwmMin,[Kp2,Ki2,]raison"""
        parts = line.split(",")
        if len(parts) < 9:
            return
        try:
            if len(parts) >= 11:
                kp2_val = float(parts[8])
                ki2_val = float(parts[9])
                raison_val = parts[10].strip()
            else:
                kp2_val = getattr(self, "params", {}).get("kp2", 0.0)
                ki2_val = getattr(self, "params", {}).get("ki2", 0.0)
                raison_val = parts[8].strip()

            self._incoming_trial = {
                "num":      int(parts[1]),
                "nb_ech":   int(parts[2]),
                "kp":       float(parts[3]),
                "ki":       float(parts[4]),
                "kd":       float(parts[5]),
                "setpoint": float(parts[6]),
                "pwm_min":  int(parts[7]),
                "kp2":      kp2_val,
                "ki2":      ki2_val,
                "raison":   raison_val,
                "raw_data": [],
            }
            self._status_label.config(
                text=f"État robot : réception essai #{parts[1]}…"
            )
        except (ValueError, IndexError):
            pass

    def _parse_data_point(self, line):
        """D,t_ms,angle*100,acc*100,gyro*10,rateF*10,pid*10,pwm,[target*100,spd*10,]dt_us"""
        if self._incoming_trial is None:
            return
        parts = line.split(",")
        if len(parts) < 9:
            return
        try:
            t_ms  = int(parts[1])
            angle = int(parts[2]) / 100.0
            acc   = int(parts[3]) / 100.0
            gyro  = int(parts[4]) / 10.0
            rateF = int(parts[5]) / 10.0
            pid   = int(parts[6]) / 10.0
            pwm   = int(parts[7])

            if len(parts) >= 11:
                target = int(parts[8]) / 100.0
                spd    = int(parts[9]) / 10.0
                dt_ms  = int(parts[10]) / 1000.0
            else:
                target = self._incoming_trial.get("setpoint", 0.0)
                spd    = 0.0
                dt_ms  = int(parts[8]) / 1000.0

            self._incoming_trial["raw_data"].append({
                "t":      t_ms,
                "angle":  angle,
                "acc":    acc,
                "gyro":   gyro,
                "rateF":  rateF,
                "error":  angle - target,
                "pid":    pid,
                "pwm":    pwm,
                "target": target,
                "spd":    spd,
                "dt":     dt_ms,
            })
        except (ValueError, IndexError):
            pass

    def _finish_incoming_trial(self):
        """END reçu : l'essai est complet."""
        if self._incoming_trial is None:
            return

        it   = self._incoming_trial
        data = it["raw_data"]
        self._incoming_trial = None
        self._pending_trial  = False

        if len(data) < 5:
            self._status_label.config(text="État robot : essai trop court, ignoré")
            return

        self.current_trial_number += 1
        tn = self.current_trial_number

        analysis = PIDAnalyzer.analyze(
            data,
            it["kp"], it["ki"], it["kd"],
            it["setpoint"], it["pwm_min"],
            tn,
        )

        if not analysis:
            return

        trial = Trial(
            number            = tn,
            start_ms          = data[0]["t"],
            end_ms            = data[-1]["t"],
            kp                = it["kp"],
            ki                = it["ki"],
            kd                = it["kd"],
            setpoint          = it["setpoint"],
            pwm_min           = it["pwm_min"],
            kp2               = it.get("kp2", 0.0),
            ki2               = it.get("ki2", 0.0),
            duration          = analysis["duration"],
            max_angle_error   = analysis["max_error"],
            rms_error         = analysis["rms_error"],
            max_rate          = analysis["max_rate"],
            max_pwm           = analysis["max_pwm"],
            mean_pwm          = analysis["mean_pwm"],
            mean_dt           = analysis["mean_dt"],
            max_dt            = analysis["max_dt"],
            oscillation_count = analysis["crossings"],
            outcome           = analysis["outcome"],
            recommendation    = analysis["recommendation"],
            data              = data,
        )

        self.trials.append(trial)
        self.last_analysis = analysis
        self.current_data  = data

        self._add_history_row(trial)
        self._plot_data(data)

        self._recommendation_label.config(
            text=f"ESSAI {trial.number} — {trial.outcome} "
                 f"(raison : {it['raison']})\n{trial.recommendation}"
        )
        self._status_label.config(
            text=f"État robot : essai #{it['num']} reçu — {trial.outcome}"
        )
        self._arm_button.config(text="ARMER")
        self._update_guide()

    # ── Graphiques ────────────────────────────────────────────────────────────
    def _plot_data(self, data):
        if not data:
            return

        d  = data[-MAX_POINTS_PLOT:]
        t0 = d[0]["t"]
        x  = [(v["t"] - t0) / 1000.0 for v in d]
        sp = self.trials[-1].setpoint if self.trials else self.params["setpoint"]

        self._angle_line.set_data(x, [v["angle"] for v in d])
        if d and "target" in d[0]:
            self._setpoint_line.set_data(x, [v["target"] for v in d])
        else:
            self._setpoint_line.set_data(x, [sp] * len(x))
        self._rate_line.set_data(x, [v["gyro"]  for v in d])
        self._pwm_line.set_data(x,  [v["pwm"]   for v in d])

        for ax in [self._ax_angle, self._ax_rate, self._ax_pwm]:
            ax.relim()
            ax.autoscale_view()

        self._canvas.draw_idle()

    # ── Historique ────────────────────────────────────────────────────────────
    def _add_history_row(self, t):
        vals = (
            t.number,
            f"{t.kp:.2f}", f"{t.ki:.3f}", f"{t.kd:.2f}", f"{getattr(t, 'kp2', 0.0):.3f}", f"{getattr(t, 'ki2', 0.0):.4f}",
            f"{t.duration:.2f}", f"{t.rms_error:.2f}",
            f"{t.max_angle_error:.2f}", f"{t.max_rate:.1f}",
            f"{t.max_pwm:.0f}", f"{t.mean_dt:.2f}", f"{t.max_dt:.2f}",
            t.outcome,
        )
        self._tree.insert("", "end", values=vals)

    def _select_trial(self, event=None):
        sel = self._tree.selection()
        if not sel:
            return
        idx = self._tree.index(sel[0])
        if idx >= len(self.trials):
            return
        trial = self.trials[idx]
        self._history_advice.config(
            text=f"Essai {trial.number} : {trial.outcome}\n{trial.recommendation}"
        )
        self._plot_data(trial.data)

    # ── Log brut TCP ──────────────────────────────────────────────────────────
    def _log(self, text):
        self._log_text.config(state="normal")
        self._log_text.insert("end", text + "\n")
        lines = int(self._log_text.index("end-1c").split(".")[0])
        if lines > 2000:
            self._log_text.delete("1.0", f"{lines - 2000}.0")
        self._log_text.see("end")
        self._log_text.config(state="disabled")

    # ── Guide de réglage ─────────────────────────────────────────────────────
    def _update_guide(self):
        n = len(self.trials)

        if not self.connected:
            self._step_label.config(
                text="ÉTAPE 1 — Connecte-toi au WiFi du robot puis clique sur Connecter."
            )
            self._advice_label.config(
                text="SSID : RobotPID  |  Mot de passe : robot1234  |  IP : 192.168.4.1"
            )
            return

        if n == 0:
            self._step_label.config(
                text="ÉTAPE 2 — Vérifie l'IMU puis fais le premier essai P."
            )
            self._advice_label.config(
                text=(
                    "Clique sur GYRO ZÉRO avec le robot parfaitement immobile. "
                    "Commence avec Ki=0 et Kd=0. Clique sur ARMER, redresse le robot, "
                    "puis reconnecte-toi pour récupérer l'essai."
                )
            )
        elif n < 3:
            self._step_label.config(text="ÉTAPE 3 — Trouve le Kp critique.")
            self._advice_label.config(
                text=(
                    "Ne change qu'un seul paramètre. Ki=0 pour l'instant. "
                    "Si le robot tombe sans osciller : augmente Kp. "
                    "S'il oscille violemment : Kp est probablement trop haut."
                )
            )
        elif all(t.ki == 0 and t.kd == 0 for t in self.trials):
            self._step_label.config(text="ÉTAPE 3 — Continue le réglage de Kp avec P seul.")
            self._advice_label.config(
                text=(
                    "Ki et Kd restent à 0. Cherche la zone où le robot commence "
                    "à corriger sans partir immédiatement ni osciller violemment."
                )
            )
        elif all(t.ki == 0 for t in self.trials):
            self._step_label.config(text="ÉTAPE 4 — Ajoute progressivement le D.")
            self._advice_label.config(
                text="Kp est stable. Augmente Kd par petits pas. Ki reste à 0."
            )
        else:
            self._step_label.config(text="ÉTAPE 5 — Validation puis éventuellement Ki.")
            self._advice_label.config(
                text=(
                    "Si Kp/Kd donnent déjà plusieurs secondes de stabilité, "
                    "répète le même essai 3 fois avant d'ajouter Ki."
                )
            )

    # ── Export / import ───────────────────────────────────────────────────────
    def _export_session(self):
        if not self.trials:
            messagebox.showinfo("Export", "Aucun essai terminé à exporter.")
            return

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = filedialog.asksaveasfilename(
            title="Exporter la session",
            initialfile=f"PID_session_v3_wifi_{stamp}.json",
            defaultextension=".json",
            filetypes=[("Session PID JSON", "*.json")],
        )
        if not path:
            return

        payload = {
            "format":   "RobotBalance_PID_Tuner_V3_WiFi",
            "version":  1,
            "created":  self.session_started,
            "exported": datetime.now().isoformat(timespec="seconds"),
            "params":   self.params,
            "trials":   [asdict(t) for t in self.trials],
        }

        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)

        csv_path = os.path.splitext(path)[0] + "_resume.csv"
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f, delimiter=";")
            writer.writerow([
                "trial", "Kp", "Ki", "Kd", "setpoint", "pwmMin",
                "duration_s", "rms_error_deg", "max_error_deg",
                "max_rate_deg_s", "max_pwm", "mean_pwm",
                "mean_dt_ms", "max_dt_ms", "oscillations",
                "outcome", "recommendation",
            ])
            for t in self.trials:
                writer.writerow([
                    t.number, t.kp, t.ki, t.kd, t.setpoint, t.pwm_min,
                    t.duration, t.rms_error, t.max_angle_error,
                    t.max_rate, t.max_pwm, t.mean_pwm,
                    t.mean_dt, t.max_dt, t.oscillation_count,
                    t.outcome, t.recommendation,
                ])

        messagebox.showinfo("Export terminé", f"Session :\n{path}\n\nRésumé CSV :\n{csv_path}")

    def _import_session(self):
        path = filedialog.askopenfilename(
            title="Importer une session PID",
            filetypes=[("Session PID JSON", "*.json")],
        )
        if not path:
            return

        try:
            with open(path, "r", encoding="utf-8") as f:
                payload = json.load(f)

            fmt = payload.get("format", "")
            if "RobotBalance_PID_Tuner" not in fmt:
                raise ValueError("Ce fichier n'est pas une session PID valide.")

            self.trials = [Trial(**t) for t in payload.get("trials", [])]
            self.params.update(payload.get("params", {}))

            for k, v in self.params.items():
                if k in self._vars:
                    self._vars[k].set(str(v))

            for item in self._tree.get_children():
                self._tree.delete(item)

            for t in self.trials:
                self._add_history_row(t)

            if self.trials:
                self._plot_data(self.trials[-1].data)
                last = self.trials[-1]
                self._recommendation_label.config(
                    text=f"Dernier essai importé : {last.outcome}\n{last.recommendation}"
                )

            self._update_guide()
            messagebox.showinfo("Import", f"{len(self.trials)} essai(s) importé(s).")

        except Exception as e:
            messagebox.showerror("Import", str(e))

    # ── Boucle de mise à jour GUI ─────────────────────────────────────────────
    def _update_gui(self):
        while True:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._parse_line(line)

        self._update_guide()
        self.root.after(100, self._update_gui)

    def close(self):
        self._stop()
        self._disconnect()
        self.root.destroy()


# ── Point d'entrée ────────────────────────────────────────────────────────────
def main():
    root = tk.Tk()
    app  = PIDTunerWifi(root)
    root.mainloop()


if __name__ == "__main__":
    main()
