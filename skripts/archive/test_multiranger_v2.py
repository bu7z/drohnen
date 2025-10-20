#!/usr/bin/env python3
import time, os
from cflib import crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig

URI = 'radio://0/80/2M/1337691337'

def mm_to_m(v):
    if v is None or v == 0:
        return None
    # Manche FW liefern bei OOR 65535 -> als None behandeln
    if v >= 65000:
        return None
    return v / 1000.0

def fmt(v):
    return f"{v:5.2f}m" if v is not None else "  n/a "

crtp.init_drivers(enable_debug_driver=False)
os.makedirs("./cfcache", exist_ok=True)

# Schreibbaren Cache-Ordner setzen
with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cfcache')) as scf:
    # Optional: TOC-Cache hart deaktivieren, um „no LogEntry“/Mismatch zu vermeiden
    scf.cf.log.use_toc_cache = False
    scf.cf.param.use_toc_cache = False

    # TOC einlesen und vorhandene Variablen ermitteln
    toc = scf.cf.log.toc.toc  # Dict: group -> set/list of var names
    available = {f"{grp}.{name}" for grp, names in toc.items() for name in names}

    candidates = [
        "range.front","range.back","range.left","range.right","range.up",
        "range.zrange"  # nur mit ZRanger vorhanden
    ]
    present = [name for name in candidates if name in available]
    if not present:
        raise RuntimeError("Keine range.* Variablen im TOC gefunden. Deck erkannt/eingesteckt?")

    lg = LogConfig(name='ranges', period_in_ms=100)
    for name in present:
        lg.add_variable(name, 'uint16_t')

    print("Verbunden. Verfügbare Sensorvariablen:", ", ".join(present))
    time.sleep(0.3)

    # Logger paced automatisch mit period_in_ms – kein zusätzliches sleep nötig
    with SyncLogger(scf, lg) as logger:
        try:
            for _, data, _ in logger:
                f = mm_to_m(data.get('range.front'))
                b = mm_to_m(data.get('range.back'))
                l = mm_to_m(data.get('range.left'))
                r = mm_to_m(data.get('range.right'))
                u = mm_to_m(data.get('range.up'))
                d = mm_to_m(data.get('range.zrange')) if 'range.zrange' in present else None

                print(f"F:{fmt(f)}  B:{fmt(b)}  L:{fmt(l)}  R:{fmt(r)}  U:{fmt(u)}  D:{fmt(d)}")
        except KeyboardInterrupt:
            pass
