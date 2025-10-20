#!/usr/bin/env python3
import time
from cflib import crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = 'radio://0/80/2M/1337691337'

USE_FLOW = True   # auf False setzen, wenn KEIN Flow-Deck benutzt wird
RUN_PROP_TEST = False  # auf True, um vor Flug Motor-Check zu machen

def wait_for_param(scf, name, expected_value, timeout=3.0):
    """Wartet bis Parameter einen bestimmten Wert hat"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        val = scf.cf.param.get_value(name)
        if str(val) == str(expected_value):
            return True
        time.sleep(0.1)
    return False

def boot_sequence():
    crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cfcache')) as scf:
        cf = scf.cf

        # Cache-Nutzung ausschalten, um TOC-Fehler zu vermeiden
        cf.log.use_toc_cache = False
        cf.param.use_toc_cache = False

        print("➡️ Verbindung hergestellt. Prüfe Decks...")

        # Prüfe, ob Flow-Deck vorhanden ist
        flow_present = bool(int(cf.param.get_value('deck.bcFlow')))
        ranger_present = bool(int(cf.param.get_value('deck.bcMultiranger')))
        print(f"Flow-Deck: {'JA' if flow_present else 'nein'} | MultiRanger: {'JA' if ranger_present else 'nein'}")

        # Richtigen Estimator wählen
        estimator = '2' if USE_FLOW and flow_present else '1'
        cf.param.set_value('stabilizer.estimator', estimator)
        print(f"Estimator auf {'Kalman (2)' if estimator == '2' else 'Complementary (1)'} gesetzt")
        time.sleep(0.2)

        # Bei Kalman: Filter neu initialisieren
        if estimator == '2':
            print("Kalman Estimation wird zurückgesetzt...")
            cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.5)
            cf.param.set_value('kalman.resetEstimation', '0')
            print("✅ Kalman-Reset abgeschlossen.")

        # Optional: Propeller/Motor-Test
        if RUN_PROP_TEST:
            print("Starte kurzen Propeller-Test (1 Sekunde)...")
            cf.param.set_value('motorPowerSet.enable', '1')
            time.sleep(1.0)
            cf.param.set_value('motorPowerSet.enable', '0')
            print("✅ Propeller-Test abgeschlossen.")

        # Warten bis stabilisiert
        print("Warte 3 Sekunden auf Stabilisierung...")
        time.sleep(3.0)

        # Prüfen, ob Estimator stabil läuft
        try:
            est_state = cf.param.get_value('kalman.stateZ') if estimator == '2' else 'n/a'
            print(f"Estimator-Zustand: {est_state}")
        except Exception:
            print("Estimator-Zustand konnte nicht gelesen werden (nicht kritisch).")

        print("✅ Boot-Sequenz abgeschlossen, System bereit zum Flug.")

if __name__ == '__main__':
    boot_sequence()
