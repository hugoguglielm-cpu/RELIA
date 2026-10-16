#!/usr/bin/env python3
"""
RELIA — visualisation en direct de la flexion (voies A et B).

Installation (une seule fois) :
    pip install pyserial matplotlib

Lancement :
    python relia_visu.py                       (cherche la carte tout seul)
    python relia_visu.py COM5                  (Windows, port precis)
    python relia_visu.py /dev/cu.usbmodem1101  (Mac, port precis)
    python relia_visu.py --demo                (sans carte, donnees simulees)

IMPORTANT : ferme le Moniteur serie / Traceur serie d'Arduino avant,
sinon le port est occupe.

Dans la fenetre : touche t = refaire le zero, q = quitter.
Marche avec le sketch relia_test_flexion (lignes "A_uV:.. B_uV:..")
et avec le code complet relia_jauges_ble (lignes CSV t_ms,A,B,...).
"""
import math
import re
import sys
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

BAUD = 115200
FENETRE_S = 10            # secondes affichees
TRAINEE_S = 2             # trainee du graphe "direction"
UV_PAR_POINT = 0.5 * 3.0 / 32 / 8388608 * 1e6   # 1 point ADC en uV (gain 32, 3.0 V)
SATURE_UV = 45000         # au-dela : voie saturee (limite ~46 900 uV)
RE_TEST = re.compile(r"A_uV:\s*(-?[\d.]+)\s+B_uV:\s*(-?[\d.]+)")


def lire_ligne(ligne):
    """(A_uV, B_uV) ou None. Format du test, ou CSV du code complet."""
    m = RE_TEST.search(ligne)
    if m:
        return float(m.group(1)), float(m.group(2))
    p = ligne.split(",")
    if len(p) == 8:
        try:
            return int(p[1]) * UV_PAR_POINT, int(p[2]) * UV_PAR_POINT
        except ValueError:
            return None
    return None


def trouver_port():
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    mots = ("usbmodem", "usbserial", "ttyacm", "ttyusb", "usb serial",
            "usb jtag", "cp210", "ch340", "ch910", "esp")
    for p in ports:
        d = f"{p.device} {p.description} {p.manufacturer}".lower()
        if any(m in d for m in mots):
            return p.device
    if len(ports) == 1:
        return ports[0].device
    print("Carte introuvable. Ports disponibles :")
    for p in ports:
        print(f"   {p.device}   ({p.description})")
    print("Relance avec le port, ex :  python relia_visu.py COM5")
    sys.exit(1)


class Donnees:
    def __init__(self):
        self.lock = threading.Lock()
        self.t = deque(maxlen=5000)
        self.a = deque(maxlen=5000)
        self.b = deque(maxlen=5000)
        self.message = ""
        self.t0 = time.time()

    def ajouter(self, a, b):
        with self.lock:
            self.t.append(time.time() - self.t0)
            self.a.append(a)
            self.b.append(b)

    def copie(self):
        with self.lock:
            return list(self.t), list(self.a), list(self.b), self.message


class LecteurSerie(threading.Thread):
    def __init__(self, port, donnees):
        super().__init__(daemon=True)
        import serial
        self.serial = serial
        self.ser = serial.Serial(port, BAUD, timeout=0.5)
        self.d = donnees

    def run(self):
        while True:
            try:
                brut = self.ser.readline()
            except self.serial.SerialException:
                self.d.message = "# connexion perdue (carte debranchee ?)"
                print(self.d.message)
                return
            ligne = brut.decode("utf-8", "replace").strip()
            if not ligne:
                continue
            if ligne.startswith("#"):
                print(ligne)
                self.d.message = ligne
                continue
            v = lire_ligne(ligne)
            if v:
                self.d.ajouter(*v)

    def envoyer(self, texte):
        self.ser.write((texte + "\n").encode())


class Demo(threading.Thread):
    """Donnees simulees : appuis successifs dans plusieurs directions."""
    def __init__(self, donnees):
        super().__init__(daemon=True)
        self.d = donnees
        self.decalage = (0.0, 0.0)

    def run(self):
        t0 = time.time()
        while True:
            t = time.time() - t0
            cycle, phase = divmod(t, 4.0)
            force = 1000 * max(0.0, math.sin(math.pi * min(phase, 3.0) / 3.0))
            ang = math.radians(45 * (int(cycle) % 8))
            a = force * math.cos(ang) + 3 * math.sin(37 * t) - self.decalage[0]
            b = force * math.sin(ang) + 3 * math.cos(29 * t) - self.decalage[1]
            self.d.ajouter(a, b)
            time.sleep(0.1)

    def envoyer(self, texte):
        self.d.message = "# zero fait (demo)"


def main():
    demo = "--demo" in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    donnees = Donnees()
    if demo:
        source = Demo(donnees)
        print("Mode demo (donnees simulees)")
    else:
        port = args[0] if args else trouver_port()
        print(f"Connexion a {port} ... (ne touche pas la tige au demarrage)")
        try:
            source = LecteurSerie(port, donnees)
        except Exception as e:
            print(f"\nImpossible d'ouvrir {port} : {e}")
            if "busy" in str(e).lower() or "denied" in str(e).lower() or "acces" in str(e).lower():
                print("-> Le port est deja utilise par un autre programme :")
                print("   ferme le Moniteur serie ET le Traceur serie d'Arduino (ou quitte Arduino),")
                print("   et toute autre fenetre relia_visu.py encore ouverte, puis relance.")
            sys.exit(1)
    source.start()

    fig = plt.figure("RELIA - flexion", figsize=(12, 5.5))
    gs = fig.add_gridspec(1, 2, width_ratios=[2.2, 1])
    ax_t = fig.add_subplot(gs[0])
    ax_d = fig.add_subplot(gs[1])

    (ligne_a,) = ax_t.plot([], [], color="tab:blue", lw=1.8, label="A (uV)")
    (ligne_b,) = ax_t.plot([], [], color="tab:orange", lw=1.8, label="B (uV)")
    ax_t.axhline(0, color="gray", lw=0.8)
    ax_t.set_xlabel("temps (s)")
    ax_t.set_ylabel("microvolts")
    ax_t.legend(loc="upper left")
    ax_t.grid(alpha=0.3)
    titre = ax_t.set_title("En attente de donnees...")

    ax_d.set_title("Direction de la flexion")
    ax_d.set_xlabel("A (uV)")
    ax_d.set_ylabel("B (uV)")
    ax_d.set_aspect("equal")
    ax_d.grid(alpha=0.3)
    ax_d.axhline(0, color="gray", lw=0.8)
    ax_d.axvline(0, color="gray", lw=0.8)
    (trainee,) = ax_d.plot([], [], color="tab:green", alpha=0.4, lw=1)
    (fleche,) = ax_d.plot([], [], color="tab:green", lw=3)
    (point,) = ax_d.plot([], [], "o", color="tab:green", ms=10)
    info = ax_d.text(0.02, 0.02, "", transform=ax_d.transAxes, fontsize=11,
                     va="bottom", bbox=dict(boxstyle="round", fc="white", alpha=0.8))
    fig.text(0.01, 0.01, "t = refaire le zero    q = quitter", fontsize=9, color="gray")

    def maj(_):
        t, a, b, msg = donnees.copie()
        if not t:
            titre.set_text("En attente de donnees... " + msg)
            return
        tmax = t[-1]
        i0 = next((i for i, x in enumerate(t) if x >= tmax - FENETRE_S), 0)
        t, a, b = t[i0:], a[i0:], b[i0:]
        ligne_a.set_data(t, a)
        ligne_b.set_data(t, b)
        ax_t.set_xlim(max(0, tmax - FENETRE_S), max(FENETRE_S, tmax))
        lim = max(50.0, max(abs(v) for v in a + b)) * 1.2
        ax_t.set_ylim(-lim, lim)

        j0 = next((i for i, x in enumerate(t) if x >= tmax - TRAINEE_S), 0)
        trainee.set_data(a[j0:], b[j0:])
        fleche.set_data([0, a[-1]], [0, b[-1]])
        point.set_data([a[-1]], [b[-1]])
        ax_d.set_xlim(-lim, lim)
        ax_d.set_ylim(-lim, lim)

        force = math.hypot(a[-1], b[-1])
        angle = math.degrees(math.atan2(b[-1], a[-1]))
        sature = abs(a[-1]) > SATURE_UV or abs(b[-1]) > SATURE_UV
        info.set_text(f"force : {force:,.0f} uV\nangle : {angle:.0f} deg".replace(",", " "))
        if sature:
            titre.set_text("ATTENTION : voie SATUREE (faux contact / cablage ?)")
            titre.set_color("red")
        else:
            titre.set_text(f"A = {a[-1]:,.1f} uV    B = {b[-1]:,.1f} uV".replace(",", " ")
                           + (f"     {msg}" if msg else ""))
            titre.set_color("black")

    def touche(ev):
        if ev.key == "t":
            source.envoyer("t")
            donnees.message = "# zero demande..."

    fig.canvas.mpl_connect("key_press_event", touche)
    anim = FuncAnimation(fig, maj, interval=100, cache_frame_data=False)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    plt.show()
    return anim


if __name__ == "__main__":
    main()
