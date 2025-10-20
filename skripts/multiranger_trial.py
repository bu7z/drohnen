#!/usr/bin/env python3
# pip install cflib
import time
import logging

from cflib import crtp
from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = 'radio://0/80/2M/1337691337'   # <-- deine URI

# Tuning
H_TARGET     = 0.5    # m Zielhöhe
V_MAX_XY     = 0.30    # m/s max seitl./vor/zurück
V_MAX_Z      = 0.20    # m/s max steigen/sinken
THR_XY       = 0.50    # m Hand wirkt bis zu dieser Distanz
THR_UP       = 2    # m Deckennähe
DEADBAND     = 0.03    # m/s kleine Geschwindigkeiten -> 0
ALPHA        = 0.35    # EMA-Glättung
PERIOD       = 0.07    # s ~14 Hz

init_drivers()


def ema(prev, new):
    if prev is None: return new
    if new is None:  return prev
    return ALPHA * new + (1 - ALPHA) * prev

def clamp(x, lo, hi): return max(lo, min(hi, x))

def repulse(dist, thr, vmax):
    # 0..thr -> 0..vmax (je näher, desto schneller)
    if dist is None or dist >= thr: return 0.0
    return (thr - dist) / thr * vmax

def main():
    logging.basicConfig(level=logging.INFO)
    logging.getLogger('cflib').setLevel(logging.WARNING)

    crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
        cf = scf.cf

        # Kalman aktivieren (Flow v2 braucht Estimator=2)
        try:
            if cf.param.get_value('stabilizer.estimator') != '2':
                cf.param.set_value('stabilizer.estimator', '2')
                time.sleep(0.2)
        except Exception:
            pass

        mr = Multiranger(cf, rate_ms=int(PERIOD * 1000))
        mr.start()
        time.sleep(0.2)

        # Takeoff mit definierter Höhe
        with MotionCommander(scf, default_height=H_TARGET) as mc:
            print(f"Hover @ ~{H_TARGET:.2f} m. Mit Hand 'schubsen'. Ctrl-C zum Landen.")
            time.sleep(0.6)

            fE=bE=lE=rE=uE=dE = None  # EMAs
            try:
                while True:
                
                    # EMA der Distanzen
                    fE = ema(fE, mr.front)
                    bE = ema(bE, mr.back)
                    lE = ema(lE, mr.left)
                    rE = ema(rE, mr.right)
                    uE = ema(uE, mr.up)
                    dE = ema(dE, mr.down)  # zrange vom Flow/ZRanger

                    # --- Höhe regeln (einfacher P auf H_TARGET) ---
                    vz = 0.0
                    if dE is not None:
                        err = H_TARGET - dE           # >0: zu niedrig
                        vz = clamp(1.0 * err, -V_MAX_Z, V_MAX_Z)
                    if uE is not None and uE < THR_UP:
                        vz = min(vz, -0.05)          # Decke nah -> nicht steigen

                    # --- Repulsives XY ---
                    vx = 0.0
                    vy = 0.0
                    vx += repulse(fE, THR_XY, V_MAX_XY)  # vorne nah -> rückwärts
                    vx -= repulse(bE, THR_XY, V_MAX_XY)  # hinten nah -> vorwärts
                    vy -= repulse(lE, THR_XY, V_MAX_XY)  # links nah -> rechts
                    vy += repulse(rE, THR_XY, V_MAX_XY)  # rechts nah -> links

                    if abs(vx) < DEADBAND: vx = 0.0
                    if abs(vy) < DEADBAND: vy = 0.0

                    mc.start_linear_motion(vx, vy, vz)
                    time.sleep(PERIOD)

            except KeyboardInterrupt:
                print("Landing…")
                mc.stop()
            finally:
                mr.stop()
                print("Done.")

if __name__ == '__main__':
    main()
