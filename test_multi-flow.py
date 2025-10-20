#!/usr/bin/env python3
import time, os
from cflib import crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.multiranger import Multiranger

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

        # Estimator passend & ruhig warten
        flow = int(cf.param.get_value('deck.bcFlow'))
        cf.param.set_value('stabilizer.estimator', '2' if flow else '1')
        if flow:
            time.sleep(0.2)
            cf.param.set_value('kalman.resetEstimation','1'); time.sleep(0.35)
            cf.param.set_value('kalman.resetEstimation','0')
            time.sleep(2.0)

        time.sleep(START_DELAY)

        mr = Multiranger(cf, rate_ms=READ_MS)
        mr.start(); time.sleep(0.3)

        last = (None,None,None,None,None)
        last_change = time.time()

        try:
            for _ in range(100):
                vals = (mr.front, mr.back, mr.left, mr.right, mr.up)
                print(f"F:{fmt(vals[0])}  B:{fmt(vals[1])}  L:{fmt(vals[2])}  R:{fmt(vals[3])}  U:{fmt(vals[4])}")
                # Reinit wenn alle 5 Werte unverändert bleiben
                if vals != last:
                    last = vals; last_change = time.time()
                elif time.time() - last_change > STUCK_S:
                    print("↻ MR stuck → re-init")
                    mr.stop(); time.sleep(0.25)
                    mr = Multiranger(cf, rate_ms=READ_MS)
                    mr.start(); time.sleep(0.35)
                    last_change = time.time()
                time.sleep(READ_MS/1000.0)
        finally:
            mr.stop()

if __name__ == "__main__":
    run()
