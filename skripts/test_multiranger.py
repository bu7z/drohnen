
#!/usr/bin/env python3
import time
from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig

URI = 'radio://0/80/2M/1337691337'  # anpassen

init_drivers()

# LogConfig anlegen (range.* sind uint16_t in Millimetern)
lg = LogConfig(name='ranges', period_in_ms=100)
for v in ('range.front','range.back','range.left','range.right','range.up','range.zrange'):
    lg.add_variable(v, 'uint16_t')

with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
    print("Verbunden. Starte Multiranger-Test in 10s ...")
    #time.sleep(10)
    with SyncLogger(scf, lg) as logger:
        try:
            for _, data, _ in logger:
                f = data.get('range.front',   0)/1000.0
                b = data.get('range.back',    0)/1000.0
                l = data.get('range.left',    0)/1000.0
                r = data.get('range.right',   0)/1000.0
                u = data.get('range.up',      0)/1000.0
                d = data.get('range.zrange',  0)/1000.0
                print(f"F:{f:5.2f}m  B:{b:5.2f}m  L:{l:5.2f}m  R:{r:5.2f}m  U:{u:5.2f}m  D:{d:5.2f}m")
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
