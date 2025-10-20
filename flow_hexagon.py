#!/usr/bin/env python3
import math
import time
from cflib import crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M/1337691337'

# Parameter
TAKEOFF_Z      = 1.0   # m
SHIFT_X        = 0.50  # m vorwärts verschieben
HEX_EDGE       = 0.50  # m Kantenlänge des Hexagons
VEL_XY         = 0.25  # m/s
VEL_Z          = 0.25  # m/s
PAUSE          = 0.1   # s kurze Pause zwischen Segmenten

def set_kalman(scf):
    """Flow-Setup: Kalman aktivieren & sauber resetten."""
    cf = scf.cf
    # Nur wenn Flow verbaut ist macht Kalman Sinn; wir erzwingen hier Kalman.
    cf.param.set_value('stabilizer.estimator', '2')
    time.sleep(0.2)
    cf.param.set_value('kalman.resetEstimation', '1'); time.sleep(0.3)
    cf.param.set_value('kalman.resetEstimation', '0')
    # 2 s ruhig stehen lassen, damit der Filter sauber einrastet
    time.sleep(2.0)

def move_xyz(mc, dx, dy, dz, vxy=VEL_XY, vz=VEL_Z):
    """Bewegt relativ; nutzt getrennte Geschw. für XY & Z."""
    # MotionCommander.move_distance nimmt einen Speed-Parameter für alle Achsen.
    # Wir splitten Z ggf. auf zwei Züge, wenn sich Geschwindigkeiten stark unterscheiden sollen.
    # Für Einfachheit: ein Zug mit konservativem Tempo.
    v = min(vxy, vz)
    mc.move_distance(dx, dy, dz, velocity=v)

def hex_vertices_yz(edge):
    """
    Erzeuge die 6 Eckpunkte eines regulären, vertikal stehenden Hexagons
    im YZ-Plane, zentriert um (y,z)=(0,0). Start bei "oben" (90°).
    """
    R = edge  # für reguläres Hexagon gilt: R = a (Kantenlänge)
    verts = []
    for k in range(6):
        angle = math.radians(90 + 60*k)  # 90°, 150°, 210°, ...
        y = R * math.cos(angle)
        z = R * math.sin(angle)
        verts.append((y, z))
    return verts

def fly_vertical_hexagon(mc, edge=HEX_EDGE):
    """
    Fliegt ein geschlossenes Hexagon im YZ-Plane am aktuellen X.
    Start: oben, dann im Uhrzeigersinn.
    """
    verts = hex_vertices_yz(edge)
    # Wir starten relativ am aktuellen Punkt. Also erst zum ersten Eckpunkt:
    y0, z0 = verts[0]
    move_xyz(mc, 0.0, y0, z0)  # hin zum ersten Eckpunkt
    time.sleep(PAUSE)

    # Sechs Kanten nacheinander
    for i in range(6):
        yA, zA = verts[i]
        yB, zB = verts[(i+1) % 6]
        dy = yB - yA
        dz = zB - zA
        move_xyz(mc, 0.0, dy, dz)
        time.sleep(PAUSE)

    # Zurück ins Zentrum (Gegenvektor zu y0,z0)
    move_xyz(mc, 0.0, -y0, -z0)
    time.sleep(PAUSE)

def main():
    crtp.init_drivers(enable_debug_driver=False)
    print("Verbinde zum Crazyflie...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cfcache')) as scf:
        print("✅ Verbindung hergestellt.")
        # Kalman sauber setzen
        set_kalman(scf)

        with MotionCommander(scf, default_height=TAKEOFF_Z) as mc:
            print(f"🛫 Hover auf {TAKEOFF_Z:.2f} m …")
            time.sleep(1.0)  # kurze Stabilisierung

            # 1) 0.3 m vor (X+)
            print(f"➡️ Vorwärts {SHIFT_X:.2f} m …")
            move_xyz(mc, SHIFT_X, 0.0, 0.0)
            time.sleep(0.2)

            # 2) Vertikal aufgerichtetes Hexagon abfliegen (im YZ-Plane)
            print(f"🔷 Hexagon (vertikal) mit Kante {HEX_EDGE:.2f} m …")
            fly_vertical_hexagon(mc, HEX_EDGE)

            # 3) Zur Ausgangsposition zurück (X-)
            print(f"↩️ Zurück {SHIFT_X:.2f} m …")
            move_xyz(mc, -SHIFT_X, 0.0, 0.0)
            time.sleep(0.2)

            # 4) Landen
            print("🛬 Landung …")
            mc.land(velocity=0.3)
            print("✅ Fertig.")

if __name__ == '__main__':
    main()
