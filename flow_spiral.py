#!/usr/bin/env python3
import math
import time
from cflib import crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M/1337691337'

# --- Parameter ---
START_Z   = 0.30   # m: Start-Hover
END_Z     = 1.50   # m: Zielhöhe
R_MAX     = 1   # m: maximaler Spiralradius
TURNS     = 5.0    # Anzahl Umdrehungen bis zum Endradius
SEG_LEN   = 0.05   # m: Segmentlänge (feinere Bahn bei kleinerem Wert)
VEL_XY    = 2.0   # m/s: Seitwärts-/Vorwärtsgeschwindigkeit
VEL_Z     = 1.5   # m/s: Vertikal
PAUSE     = 0.00   # s: optional kleine Pause zwischen Segmenten

def set_kalman(scf):
    cf = scf.cf
    cf.param.set_value('stabilizer.estimator', '2')   # Kalman (Flow)
    time.sleep(0.2)
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.35)
    cf.param.set_value('kalman.resetEstimation', '0')
    # 2s ruhig stehen lassen, Flow stabilisieren
    time.sleep(2.0)

def move_rel(mc, dx, dy, dz, vxy=VEL_XY, vz=VEL_Z):
    v = min(vxy, vz)  # einheitliche konservative Geschwindigkeit
    mc.move_distance(dx, dy, dz, velocity=v)

def fly_spiral(mc, r_max=R_MAX, z0=START_Z, z1=END_Z, turns=TURNS, seg_len=SEG_LEN):
    """
    Archimedes-Spirale: r = k*theta, theta ∈ [0, theta_max], k = r_max/theta_max.
    x = r cos(theta), y = r sin(theta), z linear von z0 -> z1.
    """
    theta_max = 2.0 * math.pi * turns
    k = r_max / theta_max
    theta = 0.0

    # Startpunkt
    r = k * theta
    x_prev = r * math.cos(theta)
    y_prev = r * math.sin(theta)
    z_progress = 0.0  # 0..1

    x_sum = 0.0
    y_sum = 0.0

    while theta < theta_max:
        # Schrittweite so wählen, dass ~SEG_LEN zurückgelegt wird:
        # s ≈ sqrt((dr)^2 + (r dθ)^2) mit dr = k dθ -> dθ = SEG_LEN / sqrt(k^2 + r^2)
        dtheta = seg_len / max(1e-6, math.sqrt(k*k + r*r))
        theta_next = min(theta + dtheta, theta_max)

        # nächster Punkt
        r_next = k * theta_next
        x = r_next * math.cos(theta_next)
        y = r_next * math.sin(theta_next)

        dx = x - x_prev
        dy = y - y_prev

        # z linear über theta
        dz_total = z1 - z0
        dz = dz_total * ((theta_next - theta) / theta_max)

        # ausführen
        move_rel(mc, dx, dy, dz)
        if PAUSE > 0: time.sleep(PAUSE)

        # Summen für Rückflug
        x_sum += dx
        y_sum += dy

        # weiter
        theta = theta_next
        r = r_next
        x_prev, y_prev = x, y

    # Rückflug zur Mitte (horizontal)
    move_rel(mc, -x_sum, -y_sum, 0.0)

def main():
    crtp.init_drivers(enable_debug_driver=False)
    print("Verbinde zum Crazyflie...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cfcache')) as scf:
        print("✅ Verbindung hergestellt.")
        set_kalman(scf)

        with MotionCommander(scf, default_height=START_Z) as mc:
            print(f"🛫 Hover auf {START_Z:.2f} m …")
            time.sleep(1.0)  # kurze Stabilisierung

            print(f"🔁 Spiralaufstieg bis {END_Z:.2f} m, Radius ≤ {R_MAX:.2f} m …")
            fly_spiral(mc, r_max=R_MAX, z0=START_Z, z1=END_Z, turns=TURNS, seg_len=SEG_LEN)

            print("🛬 Landung …")
            mc.land(velocity=0.3)
            print("✅ Fertig.")

if __name__ == '__main__':
    main()
