"""
RobotBalance - Banc d'Essais Relevage Automatique (Wi-Fi)
=========================================================

Interface graphique spécialisée pour tester, analyser et optimiser
le redressement automatique du robot depuis +/-20 deg au sol jusqu'a
l'equilibre stable maintenu pendant 10 secondes.

PARAMETRES DU RELEVAGE :
  - PWM Kick         : Puissance de l'impulsion (50 a 200, limite stricte a 200)
  - Duree Kick       : Temps max d'impulsion motrice en ms (30 a 350 ms)
  - Angle Capture    : Angle a partir duquel le kick s'arrete et le freinage s'active (3 a 12 deg)
  - Kd Boost         : Facteur multiplicateur de Kd pendant le freinage (1.0x a 2.5x)
  - Duree Equilibre  : Temps de maintien en equilibre vise (10s)

RESEAU :
  SSID: RobotPID   Mot de passe: robot1234   IP: 192.168.4.1:3333
"""

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
from tkinter import ttk, messagebox

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ── Configuration Réseau ──────────────────────────────────────────────────────
ROBOT_IP     = "192.168.4.1"
ROBOT_PORT   = 3333
TCP_TIMEOUT  = 5.0
RECV_TIMEOUT = 0.05

MAX_POINTS_PLOT = 3500


# ── Dataclass Essai Relevage ──────────────────────────────────────────────────
@dataclass
class RelevageTrial:
    number: int
    start_ms: int
    end_ms: int
    pwm_kick: int
    duree_kick: int
    angle_capture: float
    kd_boost: float
    kp: float
    ki: float
    kd: float
    setpoint: float
    pwm_min: int
    kp2: float
    deadband: float
    duration: float
    t_redressement_ms: float
    max_overshoot: float
    max_gyro_rate: float
    mean_balance_error: float
    rms_error: float
    outcome: str
    recommendation: str
    data: list


# ── Analyseur Intelligent de Relevage ─────────────────────────────────────────
class RelevageAnalyzer:
    @staticmethod
    def analyze(data, p_kick, t_kick, c_kick, kd_boost, setpoint, trial_no):
        if not data:
            return None

        duration = max(0.0, (data[-1]["t"] - data[0]["t"]) / 1000.0)
        angles   = [x["angle"] for x in data]
        rates    = [abs(x["gyro"]) for x in data]
        pwms     = [abs(x["pwm"]) for x in data]
        phases   = [x.get("phase", 0) for x in data]

        start_angle = angles[0]
        sens = 1 if start_angle > 0 else -1

        # Temps pour franchir pour la première fois la consigne (redressement effectif)
        t_redressement = None
        for x in data:
            if sens == 1 and x["angle"] <= setpoint:
                t_redressement = x["t"]
                break
            elif sens == -1 and x["angle"] >= setpoint:
                t_redressement = x["t"]
                break

        # Calcul de l'overshoot (dépassement max de l'autre côté de la consigne)
        overshoot = 0.0
        if t_redressement is not None:
            post_crossing = [x["angle"] for x in data if x["t"] >= t_redressement]
            if post_crossing:
                if sens == 1: # Partait de +20°, dépassement dans les négatifs
                    overshoot = max(0.0, max(setpoint - a for a in post_crossing))
                else:         # Partait de -20°, dépassement dans les positifs
                    overshoot = max(0.0, max(a - setpoint for a in post_crossing))

        max_rate = max(rates) if rates else 0.0

        # Données en phase d'équilibre (phase 2)
        eq_data = [x for x in data if x.get("phase", 0) == 2]
        if eq_data:
            eq_errors = [abs(x["angle"] - setpoint) for x in eq_data]
            rms_err   = math.sqrt(statistics.mean(e * e for e in eq_errors))
            mean_err  = statistics.mean(eq_errors)
        else:
            eq_errors = [abs(a - setpoint) for a in angles]
            rms_err   = math.sqrt(statistics.mean(e * e for e in eq_errors)) if eq_errors else 0.0
            mean_err  = statistics.mean(eq_errors) if eq_errors else 0.0

        # Détermination du résultat
        if duration >= 9.8:
            outcome = "SUCCES 10S !"
            recommendation = (
                f"🎉 REDRESSEMENT PARFAIT ! Le robot s'est redressé en "
                f"{t_redressement if t_redressement else '?'} ms, "
                f"overshoot maîtrisé ({overshoot:.1f}°), et tenue en équilibre "
                f"impeccable pendant {duration:.1f} s."
            )
        elif t_redressement is None:
            outcome = "MANQUE DE KICK"
            recommendation = (
                f"Le robot n'a pas réussi à franchir la verticale (angle initial {start_angle:.1f}°). "
                f"L'impulsion était insuffisante. Conseils : "
                f"1) Augmenter le PWM Kick ({p_kick} -> {min(200, p_kick + 20)}) "
                f"2) Ou augmenter la Durée Kick ({t_kick} ms -> {t_kick + 30} ms)."
            )
        elif overshoot > 18.0:
            outcome = "CHUTE OVERSHOOT"
            recommendation = (
                f"Le robot s'est bien relevé mais avec trop de vitesse et a basculé de l'autre côté "
                f"(overshoot de {overshoot:.1f}°). Conseils : "
                f"1) Augmenter le Kd Boost pour freiner plus tôt ({kd_boost:.1f}x -> {min(2.5, kd_boost + 0.3):.1f}x) "
                f"2) Augmenter l'Angle Capture ({c_kick:.1f}° -> {min(10.0, c_kick + 1.5):.1f}°) pour déclencher le freinage plus vite "
                f"3) Ou réduire légèrement la durée du kick ({t_kick} ms -> {max(50, t_kick - 20)} ms)."
            )
        elif duration < 4.0:
            outcome = "CHUTE FREINAGE"
            recommendation = (
                f"Le robot s'est redressé (en {t_redressement} ms) mais est tombé pendant la phase "
                f"de transition (tenu {duration:.1f} s). Ajustez le Kd Boost ou vérifiez que pwmMin est suffisant."
            )
        else:
            outcome = "EQUILIBRE PARTIEL"
            recommendation = (
                f"Très encourageant ! Relevage réussi, tenue {duration:.1f} s sur 10s. "
                f"La stabilisation finale peut être affinée (ajuster Kp2/deadband)."
            )

        return RelevageTrial(
            number=trial_no,
            start_ms=data[0]["t"],
            end_ms=data[-1]["t"],
            pwm_kick=p_kick,
            duree_kick=t_kick,
            angle_capture=c_kick,
            kd_boost=kd_boost,
            kp=16.0, ki=1.0, kd=0.30, setpoint=setpoint,
            pwm_min=18, kp2=0.25, deadband=2.0,
            duration=duration,
            t_redressement_ms=t_redressement if t_redressement is not None else -1.0,
            max_overshoot=overshoot,
            max_gyro_rate=max_rate,
            mean_balance_error=mean_err,
            rms_error=rms_err,
            outcome=outcome,
            recommendation=recommendation,
            data=data,
        )


# ── Client TCP Non-Bloquant ───────────────────────────────────────────────────
class RelevageTCPClient:
    def __init__(self, msg_queue):
        self.queue = msg_queue
        self.sock = None
        self.running = False
        self._thread = None
        self._send_queue = queue.Queue()

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass

    def send(self, cmd_str):
        self._send_queue.put(cmd_str)

    def _run(self):
        while self.running:
            if not self.sock:
                try:
                    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.sock.settimeout(TCP_TIMEOUT)
                    self.sock.connect((ROBOT_IP, ROBOT_PORT))
                    self.sock.settimeout(RECV_TIMEOUT)
                    self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.queue.put(("STATUS", "CONNECTE"))
                    self._send_queue.put("?\n")
                except Exception:
                    self.sock = None
                    self.queue.put(("STATUS", "DECONNECTE"))
                    time.sleep(1.0)
                    continue

            # Envoi des commandes en attente
            while not self._send_queue.empty():
                try:
                    msg = self._send_queue.get_nowait()
                    if not msg.endswith("\n"):
                        msg += "\n"
                    self.sock.sendall(msg.encode("ascii"))
                except Exception:
                    self.sock = None
                    break

            if not self.sock:
                continue

            # Réception des données
            buffer = ""
            while self.running and self.sock:
                try:
                    # Envoi intercalé si nécessaire
                    while not self._send_queue.empty():
                        msg = self._send_queue.get_nowait()
                        if not msg.endswith("\n"):
                            msg += "\n"
                        self.sock.sendall(msg.encode("ascii"))

                    chunk = self.sock.recv(4096)
                    if not chunk:
                        self.sock = None
                        self.queue.put(("STATUS", "DECONNECTE"))
                        break

                    buffer += chunk.decode("ascii", errors="replace")
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            self.queue.put(("LINE", line))

                except socket.timeout:
                    pass
                except Exception:
                    self.sock = None
                    self.queue.put(("STATUS", "DECONNECTE"))
                    break


# ── Fenêtre Principale GUI ────────────────────────────────────────────────────
class RelevageTunerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RobotBalance — Banc d'Essais Relevage Automatique (Wi-Fi)")
        self.root.geometry("1400x900")
        self.root.minsize(1100, 750)

        # File de messages réseau
        self.msg_queue = queue.Queue()
        self.client = RelevageTCPClient(self.msg_queue)

        # Variables de relevage
        self.params = {
            "pKick": 180,
            "tKick": 350,
            "cKick": 2.0,
            "fBoost": 2.5,
            "settleMs": 600,
            "kp": 16.0,
            "ki": 1.0,
            "kd": 0.30,
            "setpoint": 0.5,
            "pwm_min": 18,
            "kp2": 0.25,
            "deadband": 2.0,
            "tTest": 10.0,
        }

        self.trials = []
        self.current_trial_data = []
        self.current_trial_info = {}
        self.is_armed = False

        # Dossier de sauvegarde des sessions
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.session_dir = os.path.join(script_dir, "SessionTest")
        os.makedirs(self.session_dir, exist_ok=True)
        session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_file = os.path.join(self.session_dir, f"Relevage_session_{session_id}.json")

        self._build_ui()
        self.client.start()
        self.root.after(50, self._process_network)

    def _build_ui(self):
        # 1. Barre d'état supérieure
        top_bar = ttk.Frame(self.root, padding=6)
        top_bar.pack(fill="x")

        self.status_indicator = tk.Label(top_bar, text="● DECONNECTE", fg="red", font=("Helvetica", 11, "bold"))
        self.status_indicator.pack(side="left", padx=6)

        self.phase_label = ttk.Label(top_bar, text="En attente de connexion...", font=("Helvetica", 11))
        self.phase_label.pack(side="left", padx=15)

        self.armed_label = tk.Label(top_bar, text="DESARME", bg="#ddd", fg="#333", font=("Helvetica", 10, "bold"), width=12)
        self.armed_label.pack(side="right", padx=6)

        # 2. Panneau de diagnostic et conseils
        advice_frame = ttk.LabelFrame(self.root, text="Diagnostic & Conseils de Réglage", padding=6)
        advice_frame.pack(fill="x", padx=8, pady=3)

        self.advice_title = ttk.Label(advice_frame, text="Prêt pour les essais de relevage.", font=("Helvetica", 10, "bold"))
        self.advice_title.pack(anchor="w")

        self.advice_desc = ttk.Label(
            advice_frame,
            text="Placez le robot posé au sol (~20°), cliquez sur 'ARMER RELEVAGE AUTO'. Le robot détectera la pose et se redressera tout seul !",
            wraplength=1350,
            font=("Helvetica", 9)
        )
        self.advice_desc.pack(anchor="w", pady=(2, 0))

        # 3. Panneau des contrôles Relevage
        ctrl_frame = ttk.LabelFrame(self.root, text="Paramètres de Relevage Automatique (Priorité)", padding=8)
        ctrl_frame.pack(fill="x", padx=8, pady=3)

        self.vars = {}

        # Ligne 1 : Les 4 paramètres clés du relevage
        r1 = ttk.Frame(ctrl_frame)
        r1.pack(fill="x", pady=2)

        ttk.Label(r1, text="PWM Kick (50-200) :", font=("Helvetica", 9, "bold")).pack(side="left", padx=(0, 4))
        self.vars["pKick"] = tk.IntVar(value=self.params["pKick"])
        s_p = ttk.Scale(r1, from_=80, to=200, variable=self.vars["pKick"], command=lambda v: self._sync_scale("pKick", v), length=140)
        s_p.pack(side="left", padx=4)
        e_p = ttk.Entry(r1, textvariable=self.vars["pKick"], width=5)
        e_p.pack(side="left", padx=(0, 8))
        tk.Label(r1, text="(MAX 200)", fg="#b00", font=("Helvetica", 8, "bold")).pack(side="left", padx=(0, 20))

        ttk.Label(r1, text="Durée Kick (ms) :", font=("Helvetica", 9, "bold")).pack(side="left", padx=(0, 4))
        self.vars["tKick"] = tk.IntVar(value=self.params["tKick"])
        s_t = ttk.Scale(r1, from_=30, to=350, variable=self.vars["tKick"], command=lambda v: self._sync_scale("tKick", v), length=140)
        s_t.pack(side="left", padx=4)
        e_t = ttk.Entry(r1, textvariable=self.vars["tKick"], width=5)
        e_t.pack(side="left", padx=(0, 20))

        ttk.Label(r1, text="Angle Capture (°) :", font=("Helvetica", 9, "bold")).pack(side="left", padx=(0, 4))
        self.vars["cKick"] = tk.DoubleVar(value=self.params["cKick"])
        s_c = ttk.Scale(r1, from_=2.0, to=12.0, variable=self.vars["cKick"], command=lambda v: self._sync_scale_f("cKick", v), length=120)
        s_c.pack(side="left", padx=4)
        e_c = ttk.Entry(r1, textvariable=self.vars["cKick"], width=5)
        e_c.pack(side="left", padx=(0, 20))

        ttk.Label(r1, text="Kd Boost (Freinage) :", font=("Helvetica", 9, "bold")).pack(side="left", padx=(0, 4))
        self.vars["fBoost"] = tk.DoubleVar(value=self.params["fBoost"])
        s_f = ttk.Scale(r1, from_=1.0, to=4.0, variable=self.vars["fBoost"], command=lambda v: self._sync_scale_f("fBoost", v), length=110)
        s_f.pack(side="left", padx=4)
        e_f = ttk.Entry(r1, textvariable=self.vars["fBoost"], width=5)
        e_f.pack(side="left", padx=(0, 12))

        ttk.Label(r1, text="Settle (ms) :", font=("Helvetica", 9, "bold"), foreground="#1565C0").pack(side="left", padx=(0, 4))
        self.vars["settleMs"] = tk.IntVar(value=self.params.get("settleMs", 600))
        s_set = ttk.Scale(r1, from_=0, to=1500, variable=self.vars["settleMs"], command=lambda v: self._sync_scale("settleMs", v), length=100)
        s_set.pack(side="left", padx=4)
        e_set = ttk.Entry(r1, textvariable=self.vars["settleMs"], width=5)
        e_set.pack(side="left", padx=(0, 4))
        ttk.Label(r1, text="(gel boucle 2)", font=("Helvetica", 8), foreground="#1565C0").pack(side="left")

        # Ligne 2 : PID d'équilibre et boutons d'action
        r2 = ttk.Frame(ctrl_frame)
        r2.pack(fill="x", pady=(6, 2))

        ttk.Label(r2, text="PID Equilibre : Kp=").pack(side="left")
        self.vars["kp"] = tk.DoubleVar(value=self.params["kp"])
        ttk.Entry(r2, textvariable=self.vars["kp"], width=5).pack(side="left", padx=2)

        ttk.Label(r2, text="Ki=").pack(side="left")
        self.vars["ki"] = tk.DoubleVar(value=self.params["ki"])
        ttk.Entry(r2, textvariable=self.vars["ki"], width=5).pack(side="left", padx=2)

        ttk.Label(r2, text="Kd=").pack(side="left")
        self.vars["kd"] = tk.DoubleVar(value=self.params["kd"])
        ttk.Entry(r2, textvariable=self.vars["kd"], width=5).pack(side="left", padx=2)

        ttk.Label(r2, text="Consigne=").pack(side="left")
        self.vars["setpoint"] = tk.DoubleVar(value=self.params["setpoint"])
        ttk.Entry(r2, textvariable=self.vars["setpoint"], width=5).pack(side="left", padx=2)

        ttk.Label(r2, text="Pmin=").pack(side="left")
        self.vars["pwm_min"] = tk.IntVar(value=self.params["pwm_min"])
        ttk.Entry(r2, textvariable=self.vars["pwm_min"], width=4).pack(side="left", padx=2)

        ttk.Label(r2, text="Kp2=").pack(side="left")
        self.vars["kp2"] = tk.DoubleVar(value=self.params["kp2"])
        ttk.Entry(r2, textvariable=self.vars["kp2"], width=5).pack(side="left", padx=2)

        ttk.Label(r2, text="Dead=").pack(side="left")
        self.vars["deadband"] = tk.DoubleVar(value=self.params["deadband"])
        ttk.Entry(r2, textvariable=self.vars["deadband"], width=4).pack(side="left", padx=2)

        ttk.Button(r2, text="Appliquer Réglages", command=self._apply_all_params).pack(side="left", padx=10)

        # Boutons principaux d'armement / déclenchement
        self.btn_arm = tk.Button(
            r2, text="ARMER RELEVAGE AUTO", bg="#2e7d32", fg="white",
            font=("Helvetica", 9, "bold"), padx=10, command=self._toggle_arm
        )
        self.btn_arm.pack(side="right", padx=4)

        self.btn_kick_now = tk.Button(
            r2, text="KICK IMMEDIAT", bg="#f57c00", fg="white",
            font=("Helvetica", 9, "bold"), padx=8, command=lambda: self.client.send("x")
        )
        self.btn_kick_now.pack(side="right", padx=4)

        self.btn_stop = tk.Button(
            r2, text="ARRET D'URGENCE", bg="#c62828", fg="white",
            font=("Helvetica", 9, "bold"), padx=8, command=lambda: self.client.send("q")
        )
        self.btn_stop.pack(side="right", padx=4)

        # 4. Onglets : Courbes, Historique, Log
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=8, pady=4)

        tab_plots = ttk.Frame(notebook)
        tab_history = ttk.Frame(notebook)
        tab_log = ttk.Frame(notebook)

        notebook.add(tab_plots, text="Courbes Télémétrie (Phases Relevage)")
        notebook.add(tab_history, text="Historique des Essais")
        notebook.add(tab_log, text="Log TCP Brut")

        self._build_plots(tab_plots)
        self._build_history(tab_history)
        self._build_log(tab_log)

    def _sync_scale(self, key, val):
        self.vars[key].set(int(float(val)))

    def _sync_scale_f(self, key, val):
        self.vars[key].set(round(float(val), 1))

    def _build_plots(self, parent):
        self.fig = Figure(figsize=(11, 7), dpi=95)
        self.ax_angle = self.fig.add_subplot(311)
        self.ax_rate  = self.fig.add_subplot(312, sharex=self.ax_angle)
        self.ax_pwm   = self.fig.add_subplot(313, sharex=self.ax_angle)

        self.ax_angle.set_ylabel("Angle (°)")
        self.ax_angle.grid(True, linestyle="--", alpha=0.5)

        self.ax_rate.set_ylabel("Gyro (°/s)")
        self.ax_rate.grid(True, linestyle="--", alpha=0.5)

        self.ax_pwm.set_ylabel("PWM Moteur")
        self.ax_pwm.set_xlabel("Temps (secondes)")
        self.ax_pwm.grid(True, linestyle="--", alpha=0.5)

        self.line_angle, = self.ax_angle.plot([], [], "b-", label="Angle (°)", linewidth=1.8)
        self.line_target,= self.ax_angle.plot([], [], "k--", label="Consigne (0.5°)", linewidth=1.2)
        self.line_acc,   = self.ax_angle.plot([], [], "c:", label="Acc (°)", alpha=0.6)

        self.line_rate,  = self.ax_rate.plot([], [], "m-", label="Gyro rate (°/s)", linewidth=1.4)
        self.line_ratef, = self.ax_rate.plot([], [], "r--", label="Rate filtre", linewidth=1.2)

        self.line_pwm,   = self.ax_pwm.plot([], [], "g-", label="PWM Moteurs", linewidth=1.5)

        self.ax_angle.legend(loc="upper right", fontsize=8)
        self.ax_rate.legend(loc="upper right", fontsize=8)
        self.ax_pwm.legend(loc="upper right", fontsize=8)

        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _build_history(self, parent):
        cols = ("N", "PWM Kick", "Durée Kick", "Angle Cap", "Kd Boost", "T Redresse", "Overshoot", "Durée Tenue", "Résultat")
        self.tree = ttk.Treeview(parent, columns=cols, show="headings", height=15)

        widths = {
            "N": 40, "PWM Kick": 90, "Durée Kick": 100, "Angle Cap": 90,
            "Kd Boost": 85, "T Redresse": 100, "Overshoot": 90, "Durée Tenue": 95, "Résultat": 160
        }
        for c in cols:
            self.tree.heading(c, text=c)
            self.tree.column(c, width=widths[c], anchor="center")

        self.tree.pack(fill="both", expand=True, padx=4, pady=4)
        self.tree.bind("<<TreeviewSelect>>", self._select_history_trial)

        self.history_info = ttk.Label(parent, text="Sélectionnez un essai pour afficher son analyse détaillée.", font=("Helvetica", 9))
        self.history_info.pack(fill="x", padx=6, pady=4)

    def _build_log(self, parent):
        self.log_text = tk.Text(parent, wrap="none", font=("Consolas", 9), height=15)
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

    def _apply_all_params(self):
        # Envoi de tous les paramètres au robot
        p_k = min(200, max(50, self.vars["pKick"].get()))
        t_k = min(400, max(20, self.vars["tKick"].get()))
        c_k = min(15.0, max(2.0, self.vars["cKick"].get()))
        f_b = min(3.0, max(1.0, self.vars["fBoost"].get()))

        kp  = self.vars["kp"].get()
        ki  = self.vars["ki"].get()
        kd  = self.vars["kd"].get()
        sp  = self.vars["setpoint"].get()
        pm  = self.vars["pwm_min"].get()
        kp2 = self.vars["kp2"].get()
        db  = self.vars["deadband"].get()

        self.client.send(f"u{p_k}")
        self.client.send(f"t{t_k}")
        self.client.send(f"c{c_k:.1f}")
        self.client.send(f"f{f_b:.2f}")
        settle = self.vars.get("settleMs", tk.IntVar(value=600)).get()
        self.client.send(f"n{settle}")

        self.client.send(f"p{kp:.2f}")
        self.client.send(f"i{ki:.2f}")
        self.client.send(f"d{kd:.2f}")
        self.client.send(f"s{sp:.2f}")
        self.client.send(f"m{pm}")
        self.client.send(f"k{kp2:.3f}")
        self.client.send(f"b{db:.1f}")

        self.phase_label.config(text=f"Paramètres envoyés : PWM={p_k}, Durée={t_k}ms, AngleCap={c_k}°, KdBoost={f_b}x")

    def _toggle_arm(self):
        if not self.is_armed:
            self._apply_all_params()
            self.client.send("a1")
            self.is_armed = True
            self.armed_label.config(text="ARME", bg="#2e7d32", fg="white")
            self.btn_arm.config(text="DESARMER", bg="#c62828")
            self.phase_label.config(text="Posez le robot à ~20° sur le sol (déclenchement auto)...")
        else:
            self.client.send("a0")
            self.is_armed = False
            self.armed_label.config(text="DESARME", bg="#ddd", fg="#333")
            self.btn_arm.config(text="ARMER RELEVAGE AUTO", bg="#2e7d32")
            self.phase_label.config(text="Robot désarmé.")

    def _process_network(self):
        while not self.msg_queue.empty():
            msg_type, payload = self.msg_queue.get_nowait()

            if msg_type == "STATUS":
                if payload == "CONNECTE":
                    self.status_indicator.config(text="● CONNECTE", fg="#2e7d32")
                    self.phase_label.config(text="Connecté au robot (SoftAP RobotPID).")
                else:
                    self.status_indicator.config(text="● DECONNECTE", fg="red")
                    self.phase_label.config(text="Recherche du robot sur 192.168.4.1:3333...")
                    self.is_armed = False
                    self.armed_label.config(text="DESARME", bg="#ddd", fg="#333")
                    self.btn_arm.config(text="ARMER RELEVAGE AUTO", bg="#2e7d32")

            elif msg_type == "LINE":
                self._handle_robot_line(payload)

        self.root.after(40, self._process_network)

    def _handle_robot_line(self, line):
        # Affichage dans le log
        self.log_text.insert("end", line + "\n")
        self.log_text.see("end")

        if line.startswith("PARAM,"):
            self._parse_params(line)

        elif line.startswith("TRIAL_START,"):
            parts = line.split(",")
            num = parts[1] if len(parts) > 1 else "?"
            sens = parts[2] if len(parts) > 2 else ""
            self.phase_label.config(text=f"ESSAI #{num} EN COURS ({sens})...")
            self.current_trial_data = []

        elif line.startswith("TRIAL,"):
            # Entête de fin d'essai
            self._parse_trial_header(line)

        elif line.startswith("D,"):
            self._parse_sample(line)

        elif line.startswith("END,"):
            self._finish_trial()

        elif line.startswith("INFO,"):
            self.phase_label.config(text=line[5:])

    def _parse_params(self, line):
        for token in line[6:].split(","):
            if "=" in token:
                k, v = token.split("=", 1)
                try:
                    if k in self.vars:
                        if isinstance(self.vars[k], tk.IntVar):
                            self.vars[k].set(int(float(v)))
                        else:
                            self.vars[k].set(float(v))
                except Exception:
                    pass

    def _parse_trial_header(self, line):
        parts = line.split(",")
        self.current_trial_info = {}
        for p in parts[3:]:
            if "=" in p:
                k, v = p.split("=", 1)
                self.current_trial_info[k] = v
        if len(parts) > 1:
            self.current_trial_info["num"] = parts[1]
        self.current_trial_data = []

    def _parse_sample(self, line):
        parts = line.split(",")
        if len(parts) >= 11:
            t_ms    = float(parts[1])
            angle   = float(parts[2]) / 100.0
            acc     = float(parts[3]) / 100.0
            gyro    = float(parts[4]) / 10.0
            ratef   = float(parts[5]) / 10.0
            pid     = float(parts[6]) / 10.0
            pwm     = float(parts[7])
            target  = float(parts[8]) / 100.0
            spd     = float(parts[9]) / 10.0
            phase   = int(parts[10])
            dt      = float(parts[11]) / 1000.0 if len(parts) > 11 else 5.0

            self.current_trial_data.append({
                "t": t_ms, "angle": angle, "acc": acc, "gyro": gyro,
                "rateF": ratef, "pid": pid, "pwm": pwm, "target": target,
                "spd": spd, "phase": phase, "dt": dt
            })

    def _finish_trial(self):
        if not self.current_trial_data:
            return

        trial_no = int(self.current_trial_info.get("num", len(self.trials) + 1))
        p_k = int(self.current_trial_info.get("pKick", self.vars["pKick"].get()))
        t_k = int(self.current_trial_info.get("tKick", self.vars["tKick"].get()))
        c_k = float(self.current_trial_info.get("cKick", self.vars["cKick"].get()))
        f_b = float(self.current_trial_info.get("fBoost", self.vars["fBoost"].get()))
        sp  = float(self.current_trial_info.get("consigne", self.vars["setpoint"].get()))

        trial = RelevageAnalyzer.analyze(
            self.current_trial_data, p_k, t_k, c_k, f_b, sp, trial_no
        )
        if not trial:
            return

        self.trials.append(trial)

        # Désarmement dans l'UI
        self.is_armed = False
        self.armed_label.config(text="DESARME", bg="#ddd", fg="#333")
        self.btn_arm.config(text="ARMER RELEVAGE AUTO", bg="#2e7d32")

        # Mise à jour des diagnostics
        self.advice_title.config(text=f"Bilan Essai #{trial_no} : {trial.outcome} (Tenue {trial.duration:.1f} s)")
        self.advice_desc.config(text=trial.recommendation)
        self.phase_label.config(text=f"Essai #{trial_no} reçu ({len(trial.data)} éch.).")

        # Mise à jour des graphiques et de l'historique
        self._plot_trial(trial)
        self._add_to_history(trial)
        self._save_session()

    def _plot_trial(self, trial):
        data = trial.data
        if not data:
            return

        t_sec  = [x["t"] / 1000.0 for x in data]
        angles = [x["angle"] for x in data]
        accs   = [x["acc"] for x in data]
        targets= [x["target"] for x in data]
        rates  = [x["gyro"] for x in data]
        ratefs = [x["rateF"] for x in data]
        pwms   = [x["pwm"] for x in data]
        phases = [x.get("phase", 0) for x in data]

        self.ax_angle.cla()
        self.ax_rate.cla()
        self.ax_pwm.cla()

        self.ax_angle.grid(True, linestyle="--", alpha=0.5)
        self.ax_rate.grid(True, linestyle="--", alpha=0.5)
        self.ax_pwm.grid(True, linestyle="--", alpha=0.5)

        self.ax_angle.set_ylabel("Angle (°)", fontsize=9)
        self.ax_rate.set_ylabel("Gyro (°/s)", fontsize=9)
        self.ax_pwm.set_ylabel("PWM Moteur", fontsize=9)
        self.ax_pwm.set_xlabel("Temps (secondes)", fontsize=9)

        # Coloration des zones de phases en arrière-plan
        # 0 = Kick (rouge clair), 1 = Capture (jaune clair), 2 = Equilibre (vert clair)
        color_map = {0: ("#ffebee", "Kick"), 1: ("#fff8e1", "Capture"), 2: ("#e3f2fd", "Settle"), 3: ("#e8f5e9", "Equilibre")}
        if phases:
            curr_phase = phases[0]
            start_t = t_sec[0]
            for i in range(1, len(phases)):
                if phases[i] != curr_phase:
                    c, lbl = color_map.get(curr_phase, ("#fff", ""))
                    self.ax_angle.axvspan(start_t, t_sec[i], color=c, alpha=0.7)
                    self.ax_rate.axvspan(start_t, t_sec[i], color=c, alpha=0.7)
                    self.ax_pwm.axvspan(start_t, t_sec[i], color=c, alpha=0.7)
                    curr_phase = phases[i]
                    start_t = t_sec[i]
            # Dernier tronçon
            c, lbl = color_map.get(curr_phase, ("#fff", ""))
            self.ax_angle.axvspan(start_t, t_sec[-1], color=c, alpha=0.7)
            self.ax_rate.axvspan(start_t, t_sec[-1], color=c, alpha=0.7)
            self.ax_pwm.axvspan(start_t, t_sec[-1], color=c, alpha=0.7)

        self.ax_angle.plot(t_sec, angles, "b-", label="Angle (°)", linewidth=1.8)
        self.ax_angle.plot(t_sec, targets, "k--", label="Consigne", linewidth=1.2)
        self.ax_angle.plot(t_sec, accs, "c:", label="Acc (°)", alpha=0.5)

        self.ax_rate.plot(t_sec, rates, "m-", label="Gyro brut (°/s)", linewidth=1.3)
        self.ax_rate.plot(t_sec, ratefs, "r--", label="Rate filtre", linewidth=1.1)

        self.ax_pwm.plot(t_sec, pwms, "g-", label="PWM Moteur", linewidth=1.5)
        self.ax_pwm.axhline(200, color="r", linestyle=":", alpha=0.7, label="Limite 200 PWM")
        self.ax_pwm.axhline(-200, color="r", linestyle=":", alpha=0.7)

        self.ax_angle.legend(loc="upper right", fontsize=8)
        self.ax_rate.legend(loc="upper right", fontsize=8)
        self.ax_pwm.legend(loc="upper right", fontsize=8)

        self.fig.tight_layout()
        self.canvas.draw_idle()

    def _add_to_history(self, trial):
        t_red = f"{trial.t_redressement_ms:.0f} ms" if trial.t_redressement_ms > 0 else "Jamais"
        vals = (
            trial.number,
            trial.pwm_kick,
            f"{trial.duree_kick} ms",
            f"{trial.angle_capture:.1f}°",
            f"{trial.kd_boost:.1f}x",
            t_red,
            f"{trial.max_overshoot:.1f}°",
            f"{trial.duration:.2f} s",
            trial.outcome
        )
        self.tree.insert("", 0, values=vals)

    def _select_history_trial(self, event):
        sel = self.tree.selection()
        if not sel:
            return
        item = self.tree.item(sel[0])
        trial_num = item["values"][0]
        for t in self.trials:
            if t.number == trial_num:
                self._plot_trial(t)
                self.history_info.config(text=f"Essai #{t.number} : {t.outcome} — {t.recommendation}")
                break

    def _save_session(self):
        try:
            data = {
                "date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "params": self.params,
                "trials": [asdict(t) for t in self.trials]
            }
            with open(self.session_file, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print("Erreur sauvegarde session:", e)


if __name__ == "__main__":
    root = tk.Tk()
    app = RelevageTunerGUI(root)
    root.mainloop()
