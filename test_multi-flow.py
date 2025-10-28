#!/usr/bin/env python3
import time, os
from cflib import crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

URI = 'radio://0/90/2M/E7E7E7E7E7'   # <- anpassen!
READ_MS = 200

def fmt_range_mm(x):
    return f"{x/1000.0:4.2f}m" if x is not None else " n/a "

def run():
    crtp.init_drivers(enable_debug_driver=False)
    os.makedirs("./cfcache", exist_ok=True)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cfcache')) as scf:
        cf = scf.cf
        # Für wechselnde FW/Decks praktisch:
        cf.log.use_toc_cache = False
        cf.param.use_toc_cache = False

        # Ein LogConfig, Variablen „best effort“ hinzufügen
        lg = LogConfig(name='meas', period_in_ms=READ_MS)
        added = []  # <— wir merken uns die Namen hier

        def try_add(var, vartype=None):
            try:
                if vartype is not None:
                    lg.add_variable(var, vartype)
                else:
                    lg.add_variable(var)  # Typ aus TOC ableiten lassen
                added.append(var)
                return True
            except Exception:
                return False

        # MultiRanger (mm)
        for v in ('range.front','range.back','range.left','range.right','range.up'):
            try_add(v, 'uint16_t')

        # Down-Lidar (Flow v2 oder ZRanger) – optional
        has_zrange = try_add('range.zrange', 'uint16_t')

        # Estimator-Winkel – optional
        for v in ('stabilizer.roll', 'stabilizer.pitch', 'stabilizer.yaw'):
            try_add(v, 'float')

        if not added:
            print("Keine passenden Log-Variablen verfügbar – breche ab.")
            return

        print("Starte Logging:", ", ".join(added))

        with SyncLogger(scf, lg) as logger:
            try:
                for _ in range(100):
                    _, data, _ = next(logger)

                    parts = []

                    # MultiRanger-Ausgabe
                    F = data.get('range.front')
                    B = data.get('range.back')
                    L = data.get('range.left')
                    R = data.get('range.right')
                    U = data.get('range.up')
                    if any(v is not None for v in (F,B,L,R,U)):
                        parts.append(
                            f"MR F:{fmt_range_mm(F)}"
                            f" B:{fmt_range_mm(B)}"
                            f" L:{fmt_range_mm(L)}"
                            f" R:{fmt_range_mm(R)}"
                            f" U:{fmt_range_mm(U)}"
                        )

                    # zrange (Down)
                    if has_zrange and data.get('range.zrange') is not None:
                        parts.append(f"z:{data['range.zrange']/1000.0:4.2f}m")

                    # Stabilizer
                    roll  = data.get('stabilizer.roll')
                    pitch = data.get('stabilizer.pitch')
                    yaw   = data.get('stabilizer.yaw')
                    if roll is not None and pitch is not None and yaw is not None:
                        parts.append(f"RPY {roll:+6.2f}° {pitch:+6.2f}° {yaw:+6.2f}°")

                    print(" | ".join(parts) if parts else "(keine Daten)")

            except KeyboardInterrupt:
                print("Abbruch durch Benutzer.")

if __name__ == "__main__":
    run()
