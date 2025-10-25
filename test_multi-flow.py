#!/usr/bin/env python3
import time, os
from cflib import crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

URI = 'radio://0/80/2M/1337691337'
START_DELAY = 1.5
READ_MS = 200
STUCK_S = 1.0

def fmt(x): return f"{x:4.2f}m" if x is not None else " n/a "

def run():
    crtp.init_drivers(enable_debug_driver=False)
    os.makedirs("./cfcache", exist_ok=True)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cfcache')) as scf:
        cf = scf.cf
        cf.log.use_toc_cache = False
        cf.param.use_toc_cache = False

        """
        # Estimator passend & ruhig warten
        flow = int(cf.param.get_value('deck.bcFlow'))
        cf.param.set_value('stabilizer.estimator', '2' if flow else '1')
        if flow:
            time.sleep(0.2)
            cf.param.set_value('kalman.resetEstimation','1'); time.sleep(0.35)
            cf.param.set_value('kalman.resetEstimation','0')
            time.sleep(2.0)
        """

        mr_present = int(cf.param.get_value('deck.bcMultiranger') or 0)
        mr = None
        if not mr_present:
            print("MultiRanger nicht erkannt.")
        else:
            mr = Multiranger(cf, rate_ms=READ_MS)
            mr.start(); time.sleep(0.3)
            print("MultiRanger erkannt und gestartet.")

        flow_present = int(cf.param.get_value('deck.bcFlow') or 0)
        flow_logger = None
        if flow_present:
            lg = LogConfig(name='flow', period_in_ms=READ_MS)
            # Down-Lidar (mm), optischer Flow (Pixel/Frame), Qualitätswert (0..255)
            lg.add_variable('range.zrange', 'uint16_t')
            lg.add_variable('motion.deltaX', 'float')
            lg.add_variable('motion.deltaY', 'float')
            lg.add_variable('motion.squal',  'uint8_t')
            flow_logger = SyncLogger(scf, lg)  # einmalig anlegen


        flow_iter = None
        if flow_logger:
            flow_iter = iter(flow_logger)
            print("Flow-Deck erkannt (Flow-Logging aktiv).")
        else:
            print("Flow-Deck nicht erkannt.")

        time.sleep(START_DELAY)

        try:
            for _ in range(100):
                line = []

                # MR-Ausgabe (falls vorhanden)
                if mr is not None:
                    vals = (mr.front, mr.back, mr.left, mr.right, mr.up)
                    line.append(f"MR  F:{fmt(vals[0])}  B:{fmt(vals[1])}  L:{fmt(vals[2])}  R:{fmt(vals[3])}  U:{fmt(vals[4])}")
                else:
                    line.append("MR  — keine Messung")

                # FLOW-Ausgabe (falls vorhanden)
                if flow_iter is not None:
                    try:
                        _, data, _ = next(flow_iter)  # blockiert bis Sample
                        z_m  = data.get('range.zrange', 0)/1000.0
                        dX   = data.get('motion.deltaX', 0.0)
                        dY   = data.get('motion.deltaY', 0.0)
                        q    = int(data.get('motion.squal', 0))
                        line.append(f"FLOW z:{z_m:4.2f}m  dX:{dX:+.3f}  dY:{dY:+.3f}  q:{q}")
                    except StopIteration:
                        pass

                print(" | ".join(line))
                time.sleep(READ_MS/1000.0)

        except KeyboardInterrupt:
            print("Abbruch durch Benutzer.")
        finally:
            if mr:
                mr.stop()

if __name__ == "__main__":
    run()
