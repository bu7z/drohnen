#!/usr/bin/env python3
"""
Crazyflie 2.1+ | Flow2 + Multiranger
Tastatur + Hindernisvermeidung
NOTLANDUNG NUR BEI:
  → Multiranger: ALLE 32.77m
  → Flow Deck: zrange = 65.53m
"""

import logging
import time
import signal
import sys
import select
import tty
import termios
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

# === KONFIG ===
URI = 'radio://0/90/2M/E7E7E7E7E7'
OBSTACLE_DISTANCE = 0.6      # m
MANUAL_VELOCITY = 0.4        # m/s
AVOID_VELOCITY = 0.6         # m/s
KEEP_FLYING = True

# Kritische Werte
MR_INVALID = 32.77
ZRANGE_INVALID = 65.53
TOLERANCE = 0.1  # für Float-Vergleich

logging.basicConfig(level=logging.ERROR)


# === Hilfsfunktionen ===
def is_close(val):
    return val is not None and val < OBSTACLE_DISTANCE


def fmt(val):
    return f"{val:5.2f}" if val is not None and val < 10.0 else " --- "


def get_key():
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1).lower()
    return None


def emergency_stop(signum, frame):
    global KEEP_FLYING
    print("\n\nEMERGENCY STOP! (Strg+C)")
    KEEP_FLYING = False
    try:
        scf.cf.commander.send_stop_setpoint()
    except:
        pass


signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGTERM, emergency_stop)


# === Hauptprogramm ===
if __name__ == '__main__':
    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache='./cache')

    print("Verbinde mit Crazyflie...")
    with SyncCrazyflie(URI, cf=cf) as scf:
        print("Verbunden!")
        print("Arme Drohne...")
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        with MotionCommander(scf, default_height=1.0) as mc:
            with Multiranger(scf) as multiranger:
                print("\n" + "="*60)
                print("ABGEHOBEN! Steuerung:")
                print("   W/S = vor/zurück | A/D = links/rechts | R/F = hoch/runter")
                print("   X = Stop | Q = Landung | Strg+C = Notstopp")
                print("NOTLANDUNG NUR BEI:")
                print("  → Multiranger: F/B/L/R/U = 32.77m")
                print("  → Flow Deck: zrange = 65.53m")
                print("  → Sonst: weiterfliegen!")
                print("="*60 + "\n")

                old_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
                last_input_time = time.time()

                try:
                    while KEEP_FLYING:
                        key = get_key()
                        current_time = time.time()

                        # === Manuelle Eingabe ===
                        vx = vy = vz = 0.0
                        user_input = False

                        if key == 'w': vx = MANUAL_VELOCITY; user_input = True
                        elif key == 's': vx = -MANUAL_VELOCITY; user_input = True
                        elif key == 'a': vy = -MANUAL_VELOCITY; user_input = True
                        elif key == 'd': vy = MANUAL_VELOCITY; user_input = True
                        elif key == 'r': vz = MANUAL_VELOCITY; user_input = True
                        elif key == 'f': vz = -MANUAL_VELOCITY; user_input = True
                        elif key == 'x':
                            vx = vy = vz = 0
                            print("STOP (X gedrückt)")
                        elif key == 'q':
                            print("Landung angefordert (Q)...")
                            break

                        if user_input:
                            last_input_time = current_time
                        elif current_time - last_input_time > 0.3:
                            vx = vy = vz = 0

                        # === KRITISCHE NOTLANDUNG: NUR bei 32.77m + 65.53m ===
                        mr_values = [
                            multiranger.front,
                            multiranger.back,
                            multiranger.left,
                            multiranger.right,
                            multiranger.up
                        ]
                        zrange_val = multiranger.zrange if hasattr(multiranger, 'zrange') else None

                        # Prüfe: ALLE Multiranger = 32.77m?
                        all_mr_invalid = all(
                            v is not None and abs(v - MR_INVALID) < TOLERANCE
                            for v in mr_values
                        )

                        # Prüfe: zrange = 65.53m?
                        zrange_invalid = (
                            zrange_val is not None and
                            abs(zrange_val - ZRANGE_INVALID) < TOLERANCE
                        )

                        if all_mr_invalid and zrange_invalid:
                            print("\n" + "!"*60)
                            print("!!! KRITISCHER SENSOR-AUSFALL ERKANNT !!!")
                            print("Multiranger: F/B/L/R/U = 32.77m")
                            print(f"Flow Deck: zrange = {zrange_val:.2f}m")
                            print("→ SOFORTIGE NOTLANDUNG!")
                            print("!"*60 + "\n")
                            break

                        # === Hindernisvermeidung (32.77m = KEIN HINDERNIS!) ===
                        if is_close(multiranger.front):
                            vx = -AVOID_VELOCITY
                            print(f"FRONT BLOCK: {fmt(multiranger.front)} → Rückwärts!")
                        if is_close(multiranger.back):
                            vx = AVOID_VELOCITY
                            print(f"BACK BLOCK: {fmt(multiranger.back)} → Vorwärts!")
                        if is_close(multiranger.left):
                            vy = -AVOID_VELOCITY
                            print(f"LEFT BLOCK: {fmt(multiranger.left)} → Links!")
                        if is_close(multiranger.right):
                            vy = AVOID_VELOCITY
                            print(f"RIGHT BLOCK: {fmt(multiranger.right)} → Rechts!")
                        if is_close(multiranger.up):
                            print("OBEN ZU NAH → Notlandung!")
                            break

                        # === Status ===
                        status = (f"F:{fmt(multiranger.front)} "
                                  f"B:{fmt(multiranger.back)} "
                                  f"L:{fmt(multiranger.left)} "
                                  f"R:{fmt(multiranger.right)} "
                                  f"U:{fmt(multiranger.up)} "
                                  f"| z:{fmt(zrange_val)}")
                        print(status + "  " * 15, end='\r')

                        # === Bewegung ===
                        mc.start_linear_motion(vx, vy, vz)
                        time.sleep(0.05)

                finally:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        # === Landung ===
        print("\n\nLandung wird eingeleitet...")
        try:
            mc.land(velocity=0.2)
            time.sleep(3.0)
        except Exception as e:
            print(f"Landung fehlgeschlagen: {e}")
            try:
                scf.cf.commander.send_stop_setpoint()
            except:
                pass

    print("Programm sicher beendet.")