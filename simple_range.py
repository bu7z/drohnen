#!/usr/bin/env python3
"""
Crazyflie 2.1+ | Flow2 + Multiranger
Tastatur + SOFORTIGE Hindernisvermeidung
LINKS/RECHTS RICHTIG | STARK ausweichen | Flüssig
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
OBSTACLE_DISTANCE = 1.0   # m
MANUAL_VELOCITY = 0.5     # m/s
AVOID_VELOCITY = 0.8      # m/s ← STARK!
KEEP_FLYING = True

logging.basicConfig(level=logging.ERROR)


# === Hindernis-Check ===
def is_close(val):
    return val is not None and val < OBSTACLE_DISTANCE


# === Formatierung ===
def fmt(val):
    return f"{val:5.2f}" if val is not None and val < 10.0 else " --- "


# === Tastatur ===
def get_key():
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1).lower()
    return None


# === Emergency Stop ===
def emergency_stop(signum, frame):
    global KEEP_FLYING
    print("\nEMERGENCY STOP!")
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

        with MotionCommander(scf, default_height=0.4) as mc:
            with Multiranger(scf) as multiranger:
                print("Abgehoben! Steuerung: W/S/A/D | R/F hoch/runter | X=Stop | Q=Landung")
                print(f"Hindernisvermeidung: < {OBSTACLE_DISTANCE}m → SOFORT ausweichen!")

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
                            print("STOP")
                        elif key == 'q':
                            print("Landung...")
                            break

                        if user_input:
                            last_input_time = current_time
                        elif current_time - last_input_time > 0.3:
                            vx = vy = vz = 0

                        # === HINDERNISVERMEIDUNG HAT PRIORITÄT! ===
                        if is_close(multiranger.front):
                            vx = -AVOID_VELOCITY
                            print(f"FRONT BLOCK: {multiranger.front:.2f}m → Voll zurück!")
                        if is_close(multiranger.back):
                            vx = AVOID_VELOCITY
                            print(f"BACK BLOCK: {multiranger.back:.2f}m → Voll vor!")
                        if is_close(multiranger.left):
                            vy = -AVOID_VELOCITY  # ← von links weg → nach links!
                            print(f"LEFT BLOCK: {multiranger.left:.2f}m → Voll links!")
                        if is_close(multiranger.right):
                            vy = AVOID_VELOCITY   # ← von rechts weg → nach rechts!
                            print(f"RIGHT BLOCK: {multiranger.right:.2f}m → Voll rechts!")
                        if is_close(multiranger.up):
                            print("OBEN ZU NAH → Notlandung!")
                            break

                        # === Status ===
                        status = f"F:{fmt(multiranger.front)} B:{fmt(multiranger.back)} L:{fmt(multiranger.left)} R:{fmt(multiranger.right)} U:{fmt(multiranger.up)}"
                        print(status + "  " * 15, end='\r')

                        # === Bewegung ===
                        mc.start_linear_motion(vx, vy, vz)
                        time.sleep(0.05)

                finally:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        # === Landung ===
        print("\nLandung...")
        try:
            mc.land(velocity=0.2)
            time.sleep(3.0)
        except:
            pass

    print("Programm beendet.")