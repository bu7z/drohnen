# -*- coding: utf-8 -*-
#
# 2-Crazyflie Kreis/Formation – nur URI0 & URI1
#
# Voraussetzung: pip install cflib

import math
import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# -----------------------------
# URIs (anpassen!)
# Wenn gleicher USB-Radio verwendet wird: gleicher Kanal nötig.
URI0 = 'radio://0/80/2M/1337691337'
URI1 = 'radio://0/90/2M/E7E7E7E7E6'

uris = {
    URI0,
    URI1,
}

# d: Kreisdurchmesser [m]
# z: Flughöhe [m]
params = {
    URI0: [{'d': 1.0, 'z': 0.3}],
    URI1: [{'d': 1.0, 'z': 0.3}],
}

# -----------------------------
def poshold(cf, t_sec, z):
    steps = int(t_sec * 10)
    for _ in range(steps):
        cf.commander.send_hover_setpoint(0, 0, 0, z)
        time.sleep(0.1)

def reset_estimator(scf: SyncCrazyflie, _=None):
    """Kalman-Estimator sauber zurücksetzen (kompatibel ohne Swarm.reset_estimators())."""
    cf = scf.cf
    # Setzen
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    # Kurz warten, bis sich die Schätzung stabilisiert
    time.sleep(1.0)

def run_sequence(scf: SyncCrazyflie, p):
    cf = scf.cf

    # (Optional) Arming – nur für Plattformen, die es benötigen
    try:
        cf.platform.send_arming_request(True)
        time.sleep(1.0)
    except Exception:
        pass  # Bei CF2.x ohne Arming wird hier meist eine Exception geworfen -> ignorieren

    # Setpoint-Rate
    fs = 4
    fsi = 1.0 / fs

    # leichte Kompensation
    comp = 1.3

    base = 0.15  # Basis-Höhe
    d = p['d']
    z = p['z']

    poshold(cf, 2, base)

    # sanfter Steigflug auf z
    ramp = fs * 2
    for r in range(ramp):
        cf.commander.send_hover_setpoint(0, 0, 0, base + r * (z - base) / ramp)
        time.sleep(fsi)

    poshold(cf, 2, z)

    # 2 Kreise
    circle_time = 8  # s pro Umdrehung
    steps = circle_time * fs
    for _ in range(2):
        for _ in range(steps):
            cf.commander.send_hover_setpoint(
                d * comp * math.pi / circle_time,  # vx
                0,                                 # vy
                360.0 / circle_time,               # yawrate [deg/s]
                z                                  # alt
            )
            time.sleep(fsi)

    poshold(cf, 2, z)

    # sanft absenken
    for r in range(ramp):
        cf.commander.send_hover_setpoint(0, 0, 0, base + (ramp - r) * (z - base) / ramp)
        time.sleep(fsi)

    poshold(cf, 1, base)

    # Stop
    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        # Estimator-Reset für alle
        swarm.parallel(reset_estimator)

        # Flug-Sequenzen starten
        swarm.parallel(run_sequence, args_dict=params)
