# swarm_hover_lh.py
import time
import cflib.crtp
from cflib.crazyflie.swarm import Swarm

URIS = {
    'radio://0/80/2M/1337691337',
    'radio://0/80/2M/E7E7E7E7E5',
}

H = 0.6      # Höhe (m)
TO = 2.0     # Takeoff-Dauer (s)
HOVER = 3.0  # Hover-Zeit (s)
LD = 2.0     # Lande-Dauer (s)

def take_off(scf):
    scf.cf.high_level_commander.takeoff(H, TO)
    time.sleep(TO + 0.5)

def land(scf):
    scf.cf.high_level_commander.land(0.0, LD)
    time.sleep(LD + 0.5)
    scf.cf.high_level_commander.stop()

if __name__ == "__main__":
    cflib.crtp.init_drivers()
    with Swarm(URIS) as swarm:
        # Wichtig bei Lighthouse: Estimator sauber setzen/prüfen
        swarm.reset_estimators()             # wartet auf stabile Pose
        swarm.parallel_safe(take_off)        # beide gleichzeitig abheben
        time.sleep(HOVER)
        swarm.parallel_safe(land)
