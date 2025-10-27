#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (aktiviert 3D)
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
import cflib.crtp as crtp

# ============================
# Parameter
# ============================
URI = 'radio://0/80/2M/1337691337'   # <- anpassen!
DEFAULT_TAKEOFF_Z = 0.30             # m
SPIRAL_TURNS = 3.0                   # Umdrehungen
R_START = 0.05                       # m
R_END = 0.80                         # m
TOTAL_TIME = 30.0                    # s
STEP_DT = 0.05                       # s
VEL = 0.4                            # m/s
YAW_OFFSET_DEG = 0.0                 # relative Drehung zu Beginn (optional)

# Plot-Optionen
HISTORY_SEC = 60.0
PLOT_RATE_HZ = 15

# ============================
# Globals
# ============================
pos_buffer = deque(maxlen=int(HISTORY_SEC / STEP_DT) + 1000)
stop_logging = threading.Event()
flight_done = threading.Event()

def setup_logconf():
    logconf = LogConfig(name='stateEstimate', period_in_ms=int(STEP_DT * 1000))
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')
    return logconf

def logger_thread(scf):
    logconf = setup_logconf()
    try:
        with SyncLogger(scf, logconf) as logger:
            for log_entry in logger:
                if stop_logging.is_set():
                    break
                data = log_entry[1]
                pos_buffer.append((time.time(),
                                   float(data['stateEstimate.x']),
                                   float(data['stateEstimate.y']),
                                   float(data['stateEstimate.z'])))
    except Exception as e:
        print(f"[Logger] Fehler: {e}")

def turn_relative(mc, deg):
    if abs(deg) < 1e-3:
        return
    if deg > 0:
        mc.turn_right(abs(deg), rate=90)
    else:
        mc.turn_left(abs(deg), rate=90)

def fly_spiral(scf):
    """Spiralbewegung über kleine relative Segmente."""
    try:
        with MotionCommander(scf, default_height=DEFAULT_TAKEOFF_Z) as mc:
            time.sleep(1.0)
            turn_relative(mc, YAW_OFFSET_DEG)

            steps = max(1, int(TOTAL_TIME / STEP_DT))
            theta = 0.0
            theta_end = 2.0 * math.pi * SPIRAL_TURNS
            dtheta = theta_end / steps

            dz_total = max(0.0, R_END - R_START) * 0.25
            dz_per_step = dz_total / steps

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

            mc.land()
    except Exception as e:
        print(f"[Flight] Fehler: {e}. Versuche zu landen …")
        try:
            with MotionCommander(scf, default_height=DEFAULT_TAKEOFF_Z) as mc:
                mc.land()
        except Exception as e2:
            print(f"[Flight] Landen fehlgeschlagen: {e2}")
    finally:
        flight_done.set()

def set_axes_equal_3d(ax, xs, ys, zs, pad=0.1):
    """Sorgt für gleiche Skalierung in x/y/z (Würfel-View)."""
    if not xs or not ys or not zs:
        return
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)
    # Vermeide Null-Range
    if xmax == xmin: xmax += 0.01
    if ymax == ymin: ymax += 0.01
    if zmax == zmin: zmax += 0.01
    xmid = 0.5 * (xmax + xmin)
    ymid = 0.5 * (ymax + ymin)
    zmid = 0.5 * (zmax + zmin)
    max_range = max(xmax - xmin, ymax - ymin, zmax - zmin) * (1.0 + pad)
    half = 0.5 * max_range
    ax.set_xlim(xmid - half, xmid + half)
    ax.set_ylim(ymid - half, ymid + half)
    ax.set_zlim(zmid - half, zmid + half)

def run():
    crtp.init_drivers(enable_debug_driver=False)
    print(f"Verbinde zu {URI} ...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        t_log = threading.Thread(target=logger_thread, args=(scf,), daemon=True)
        t_log.start()

        t_fly = threading.Thread(target=fly_spiral, args=(scf,), daemon=True)
        t_fly.start()

        # ===== 3D Live-Plot =====
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        line3d, = ax.plot([], [], [], lw=2)
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title('Crazyflie Live-Track (3D) – Spiral')

        draw_interval = 1.0 / PLOT_RATE_HZ
        last_draw = 0.0

        try:
            while not flight_done.is_set() or len(pos_buffer) > 0:
                now = time.time()
                if now - last_draw >= draw_interval:
                    xs = [p[1] for p in pos_buffer]
                    ys = [p[2] for p in pos_buffer]
                    zs = [p[3] for p in pos_buffer]
                    if xs and ys and zs:
                        line3d.set_data(xs, ys)
                        line3d.set_3d_properties(zs)
                        set_axes_equal_3d(ax, xs, ys, zs, pad=0.15)
                        plt.pause(0.001)   # stabiler GUI-Loop
                    last_draw = now
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

if __name__ == "__main__":
    run()
