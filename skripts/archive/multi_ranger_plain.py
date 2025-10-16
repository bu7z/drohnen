#!/usr/bin/env python3
"""
Multiranger-Check für Crazyflie
 - verbindet sich zur Crazyflie
 - startet den Multiranger-Helper
 - gibt regelmäßig die 5/6 Richtungsabstände aus
Usage:
  pip install cflib
  python3 multiranger_check.py
Anpassen: URI (DEFAULT_URI)
"""
import time
import logging
from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.multiranger import Multiranger

logging.basicConfig(level=logging.INFO)
DEFAULT_URI = 'radio://0/80/2M/E7E7E7E7E5'  # an dein Crazyradio/URI anpassen
READ_INTERVAL = 0.1  # s, wie oft die Werte ausgegeben werden

def format_dist(d):
    """Gibt lesbar zurück: Zahl (m) oder 'N/A'."""
    if d is None:
        return 'N/A'
    try:
        # typischerweise float in Metern
        return f'{d:.3f} m'
    except Exception:
        return str(d)

def run(uri=DEFAULT_URI, runtime_s=10):
    init_drivers()  # cflib initialisieren (USB/Radio)
    print(f'Connecting to {uri} ...')

    # SyncCrazyflie sorgt für zuverlässiges open/close der Verbindung
    with SyncCrazyflie(uri, cf=Crazyflie()) as scf:
        cf = scf.cf

        # Multiranger helper (ruft intern Log-Configs auf)
        mr = Multiranger(cf, rate_ms=int(READ_INTERVAL*1000))
        try:
            mr.start()   # startet periodische Messungen/Updates
            print('Multiranger gestartet. Drücke CTRL-C zum Abbrechen.')
            t0 = time.time()
            while True:
                # mr.front/back/... sind Attribute, werden aktualisiert
                front = getattr(mr, 'front', None)
                back  = getattr(mr, 'back', None)
                left  = getattr(mr, 'left', None)
                right = getattr(mr, 'right', None)
                up    = getattr(mr, 'up', None)
                down  = getattr(mr, 'down', None)

                # Ausgabe (kurz): N/A bedeutet: keine Messung empfangen
                print(
                    f'F: {format_dist(front):>9}  '
                    f'B: {format_dist(back):>9}  '
                    f'L: {format_dist(left):>9}  '
                    f'R: {format_dist(right):>9}  '
                    f'U: {format_dist(up):>9}  '
                    f'D: {format_dist(down):>9}'
                )

                time.sleep(READ_INTERVAL)

                if runtime_s is not None and (time.time() - t0) > runtime_s:
                    break

        except KeyboardInterrupt:
            print('Abbruch per STRG-C')
        finally:
            mr.stop()
            print('Multiranger gestoppt.')

if __name__ == '__main__':
    # runtime_s = None -> läuft unendlich, sonst z.B. 30 (Sekunden)
    run(runtime_s=None)
