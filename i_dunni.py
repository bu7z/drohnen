#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import time
from collections import defaultdict, deque

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
import cflib.crtp as crtp

# ============================
# URIs (gleicher Kanal/Bitrate für 1 Dongle)
# ============================
URI0 = 'radio://0/80/2M/1337691337'   # CF #1 (z.B. Leader)
URI1 = 'radio://0/80/2M/E7E7E7E7E5'   # CF #2

# ============================
# Flugparameter
# ============================
DEFAULT_TAKEOFF_Z0 = 0.35      # m
DEFAULT_TAKEOFF_Z1 = 0.55      # m (kleiner Höhenversatz für Sicherheit/Sicht)
SPIRAL_TURNS = 3.0             # Umdrehungen
R_START = 0.05                 # m
R_END = 0.80                   # m
TOTAL_TIME = 28.0              # s
STEP_DT = 0.05                 # s
VEL = 0.40                     # m/s
YAW_OFFSET_DEG0 = 0.0
YAW_OFFSET_DEG1 = 0.0
DZ_TOTAL = 0.00                # m Gesamter Höhenzuwachs in der Spirale (0.0 = flach)

# ============================
# Plot-Optionen
# ============================
HISTORY_SEC = 120.0
PLOT_RATE_HZ = 15
FLY = True                     # False => nur verbinden+loggen+plotten

# ============================
# Globals
# ============================
pos_buffers = defaultdict(lambda: deque(maxlen=int(HISTORY_SEC / STEP_DT) + 2000))
stop_logging = threading.Event()
flight_done = {URI0: threading.Event(), URI1: threading.Event()}

# ============================
# Helpers
# ============================
def setup_logconf():
    lc = LogConfig(name='stateEstimate', period_in_ms=int(STEP_DT * 1000))
    lc.add_variable('stateEstimate.x', 'float')
    lc.add_variable('stateEstimate.y', 'float')
    lc.add_variable('stateEstimate.z', 'float')
    lc.add_variable('stateEstimate.yaw', 'float')
    return lc

def logger_thread(scf, uri):
    lc = setup_logconf()
    try:
        with SyncLogger(scf, lc) as logger:
            for _, data, _ in logger:
                if stop_logging.is_set():
                    break
                pos_buffers[uri].append((
                    time.time(),
                    float(data['stateEstimate.x']),
                    float(data['stateEstimate.y']),
                    float(data['stateEstimate.z']),
                ))
    except Exception as e:
        print(f"[Logger {uri}] Fehler: {e}")

def reset_estimator(cf):
    """Kalman Estimator sauber resetten (Flow-Deck vorausgesetzt)."""
    try:
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(1.0)
    except Exception as e:
        print(f"[Estimator] Fehler beim Reset: {e}")

def turn_relative(mc, deg):
    if abs(deg) < 1e-3:
        return
    (mc.turn_right if deg > 0 else mc.turn_left)(abs(deg), rate=90)

def fly_spiral(scf, uri, default_z, yaw_offset_deg=0.0):
    """Spiralbewegung über kleine relative Segmente (MotionCommander)."""
    try:
        with MotionCommander(scf, default_height=default_z) as mc:
            time.sleep(0.8)
            turn_relative(mc, yaw_offset_deg)

            steps = max(1, int(TOTAL_TIME / STEP_DT))
            theta = 0.0
            dtheta = (2.0 * math.pi * SPIRAL_TURNS) / steps
            dz_per_step = (DZ_TOTAL / steps) if DZ_TOTAL else 0.0

            r = R_START
            dr = (R_END - R_START) / steps

            for _ in range(steps):
                arc = max(0.01, r) * dtheta
                dx = arc * (-math.sin(theta))
                dy = arc * ( math.cos(theta))
                dz = dz_per_step
                mc.move_distance(dx, dy, dz, velocity=VEL)
                theta += dtheta
                r += dr

            mc.start_linear_motion(0, 0, 0); time.sleep(0.8)
            mc.land()
    except Exception as e:
        print(f"[Flight {uri}] Fehler: {e}. Versuche zu landen …")
        try:
            with MotionCommander(scf, default_height=default_z) as mc:
                mc.land()
        except Exception as e2:
            print(f"[Flight {uri}] Landen fehlgeschlagen: {e2}")
    finally:
        flight_done[uri].set()

def set_axes_equal_3d(ax, xs, ys, zs, pad=0.12):
    if not xs or not ys or not zs:
        return
    xmin, xmax = min(xs), max(xs); ymin, ymax = min(ys), max(ys); zmin, zmax = min(zs), max(zs)
    if xmax == xmin: xmax += 0.01
    if ymax == ymin: ymax += 0.01
    if zmax == zmin: zmax += 0.01
    xmid = 0.5*(xmax+xmin); ymid = 0.5*(ymax+ymin); zmid = 0.5*(zmax+zmin)
    max_range = max(xmax-xmin, ymax-ymin, zmax-zmin) * (1.0 + pad)
    half = 0.5*max_range
    ax.set_xlim(xmid-half, xmid+half); ax.set_ylim(ymid-half, ymid+half); ax.set_zlim(zmid-half, zmid+half)

def open_with_retry(uri, tries=4, backoff=1.5):
    for k in range(tries):
        try:
            scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache'))
            scf.open_link()
            print(f"[OK] Link: {uri}")
            return scf
        except Exception as e:
            print(f"[FAIL] Link: {uri} ({k+1}/{tries}): {e}")
            time.sleep(backoff*(k+1))
    raise RuntimeError(f"Link fehlgeschlagen: {uri}")

# ============================
# Main
# ============================
def run():
    crtp.init_drivers(enable_debug_driver=False)

    # Links SEQUENZIELL öffnen (stabiler mit 1 Dongle)
    scf0 = open_with_retry(URI0); time.sleep(0.4)
    scf1 = open_with_retry(URI1)

    try:
        # Estimator-Reset (empfohlen bei Flow)
        reset_estimator(scf0.cf); reset_estimator(scf1.cf)

        # Logger starten
        t_log0 = threading.Thread(target=logger_thread, args=(scf0, URI0), daemon=True)
        t_log1 = threading.Thread(target=logger_thread, args=(scf1, URI1), daemon=True)
        t_log0.start(); t_log1.start()

        # Flug-Threads
        if FLY:
            t_fly0 = threading.Thread(target=fly_spiral, args=(scf0, URI0, DEFAULT_TAKEOFF_Z0, YAW_OFFSET_DEG0), daemon=True)
            t_fly1 = threading.Thread(target=fly_spiral, args=(scf1, URI1, DEFAULT_TAKEOFF_Z1, YAW_OFFSET_DEG1), daemon=True)
            t_fly0.start(); t_fly1.start()

        # ===== Live 3D Plot =====
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        line0, = ax.plot([], [], [], lw=2, label=URI0)
        line1, = ax.plot([], [], [], lw=2, label=URI1)
        ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
        ax.set_title('Crazyflie Live-Track (3D) – Dual Spiral')
        ax.legend(loc='upper left', fontsize=8)

        draw_interval = 1.0 / PLOT_RATE_HZ
        last_draw = 0.0

        try:
            while True:
                now = time.time()
                if now - last_draw >= draw_interval:
                    xs0 = [p[1] for p in pos_buffers[URI0]]; ys0 = [p[2] for p in pos_buffers[URI0]]; zs0 = [p[3] for p in pos_buffers[URI0]]
                    xs1 = [p[1] for p in pos_buffers[URI1]]; ys1 = [p[2] for p in pos_buffers[URI1]]; zs1 = [p[3] for p in pos_buffers[URI1]]

                    any_points = False
                    if xs0 and ys0 and zs0:
                        line0.set_data(xs0, ys0); line0.set_3d_properties(zs0); any_points = True
                    if xs1 and ys1 and zs1:
                        line1.set_data(xs1, ys1); line1.set_3d_properties(zs1); any_points = True

                    if any_points:
                        set_axes_equal_3d(ax, xs0+xs1, ys0+ys1, zs0+zs1)
                        plt.pause(0.001)

                    last_draw = now

                # sauber beenden, wenn beide Flüge fertig (falls FLY)
                if FLY and all(ev.is_set() for ev in flight_done.values()):
                    if now - last_draw > 1.0:
                        break
                time.sleep(0.005)
        except KeyboardInterrupt:
            print("Abbruch per Tastatur.")
        finally:
            stop_logging.set()
            plt.ioff()
            try:
                plt.show(block=False)
            except Exception:
                pass
    finally:
        # Links schließen
        try: scf1.close_link()
        except Exception: pass
        try: scf0.close_link()
        except Exception: pass

if __name__ == "__main__":
    run()
