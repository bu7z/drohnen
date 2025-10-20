#!/usr/bin/env python3
import time
from collections import deque
from cflib import crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = 'radio://0/80/2M/1337691337'

# --- Parameter ---
DIST_THRESHOLD = 0.50      # m: Abstand, ab dem ausgewichen wird
HYSTERESIS     = 0.05      # m: Hysterese (verhindert Flattern)
V_MAX_XY       = 0.25      # m/s: max. seitliche/longitudinale Geschwindigkeit
EMA_ALPHA      = 0.2       # Glättung der Sensordaten
CTRL_DT        = 0.10      # s: Regelperiode (~10 Hz)
EMERGENCY_DIST = 0.15      # m: harter Notfallabstand
CEILING_THRES  = 0.60      # m: Deckennähe -> sinken
VZ_SINK        = 0.20      # m/s: sanftes Sinken bei Deckennähe

def ema(prev, new, alpha):
    if new is None:                # None (keine Messung) -> alten Wert behalten
        return prev
    if prev is None:
        return new
    return alpha * new + (1 - alpha) * prev

def cap_xy(vx, vy, vmax):
    mag = (vx*vx + vy*vy) ** 0.5
    if mag > vmax > 0:
        s = vmax / mag
        return vx * s, vy * s
    return vx, vy

def safe_land_blocking(mc, vz=0.35, timeout=6.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        mc.start_linear_motion(0.0, 0.0, -vz)
        time.sleep(0.1)
    mc.stop()

def hover_multiranger_avoid(duration_s=20):
    crtp.init_drivers(enable_debug_driver=False)
    print("Verbinde zum Crazyflie...")

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cfcache')) as scf:
        print("✅ Verbindung hergestellt.")
        cf = scf.cf
        # TOC-Cache aus – robuster bei wechselnden Decks/FW
        cf.log.use_toc_cache = False
        cf.param.use_toc_cache = False

        # MultiRanger starten
        mr = Multiranger(cf, rate_ms=int(CTRL_DT*1000))
        mr.start()
        time.sleep(0.3)

        # Gefilterte Distanzen
        f_front = f_back = f_left = f_right = f_up = None
        inside = False  # Hysterese-Zustand

        try:
            with MotionCommander(scf, default_height=1.0) as mc:
                print(f"🛫 Hover + Abstandshalter aktiv (Zielabstand {DIST_THRESHOLD:.2f} m) ...")
                t0 = time.time()
                while time.time() - t0 < duration_s:
                    # Rohwerte (m) holen (None wenn keine Messung)
                    d_front = mr.front
                    d_back  = mr.back
                    d_left  = mr.left
                    d_right = mr.right
                    d_up    = mr.up

                    # Glätten
                    f_front = ema(f_front, d_front, EMA_ALPHA)
                    f_back  = ema(f_back,  d_back,  EMA_ALPHA)
                    f_left  = ema(f_left,  d_left,  EMA_ALPHA)
                    f_right = ema(f_right, d_right, EMA_ALPHA)
                    f_up    = ema(f_up,    d_up,    EMA_ALPHA)

                    # --- Notfall: zu nah an irgendwas? ---
                    for dd in (f_front, f_back, f_left, f_right):
                        if dd is not None and dd < EMERGENCY_DIST:
                            print("⚠️  Notfall: Objekt < {:.0f} cm – sofort sinken & stoppen.".format(EMERGENCY_DIST*100))
                            safe_land_blocking(mc)
                            return

                    # --- Hysterese bestimmen ---
                    # „inside“ sobald etwas < (Schwelle) ist, verlasse erst wenn > (Schwelle + Hysterese)
                    min_d = min([x for x in (f_front, f_back, f_left, f_right) if x is not None], default=None)
                    if min_d is not None:
                        if not inside and min_d < DIST_THRESHOLD:
                            inside = True
                        elif inside and all((x is None or x > DIST_THRESHOLD + HYSTERESIS)
                                            for x in (f_front, f_back, f_left, f_right)):
                            inside = False

                    # --- Repulsions-Vektor berechnen ---
                    vx = vy = vz = 0.0
                    thr_use = DIST_THRESHOLD + (0.0 if inside else 0.0)  # (Hysterese ist already im inside-Flag)

                    def repulse(dist):
                        if dist is None or dist >= thr_use:
                            return 0.0
                        # quadratisch steigend bis V_MAX_XY
                        n = (thr_use - dist) / thr_use
                        return (n * n) * V_MAX_XY

                    # Koordinaten in MotionCommander:
                    # +X = vorwärts, -X = rückwärts, +Y = links, -Y = rechts
                    vx -= repulse(f_front)    # zu nah vorne -> rückwärts
                    vx += repulse(f_back)     # zu nah hinten -> vorwärts
                    vy -= repulse(f_left)     # zu nah links -> rechts
                    vy += repulse(f_right)    # zu nah rechts -> links

                    # Optional: Deckenvermeidung
                    if f_up is not None and f_up < CEILING_THRES:
                        vz = -min(VZ_SINK, (CEILING_THRES - f_up) * 0.6 + 0.05)

                    # Begrenzen & senden
                    vx, vy = cap_xy(vx, vy, V_MAX_XY)
                    if abs(vx) < 0.03: vx = 0.0
                    if abs(vy) < 0.03: vy = 0.0

                    mc.start_linear_motion(vx, vy, vz)

                    # Debug-Ausgabe sehr knapp halten (sonst Timing-Probleme)
                    # print(f"vx:{vx:+.2f} vy:{vy:+.2f} vz:{vz:+.2f} | F:{f_front} B:{f_back} L:{f_left} R:{f_right}")

                    time.sleep(CTRL_DT)

                print("Zeit abgelaufen – sichere Landung.")
                safe_land_blocking(mc)

        except KeyboardInterrupt:
            print("⌨️  Abbruch – sichere Landung.")
            try:
                with MotionCommander(scf, default_height=0.3) as mc:
                    safe_land_blocking(mc)
            except Exception:
                cf.commander.send_stop_setpoint()
                time.sleep(0.2)
        finally:
            mr.stop()
            print("🔚 Ende.")

if __name__ == "__main__":
    hover_multiranger_avoid(duration_s=30)
