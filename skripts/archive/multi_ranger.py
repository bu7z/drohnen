import logging, sys, time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = 'radio://0/80/2M/E7E7E7E7E5'
if len(sys.argv) > 1:
    URI = sys.argv[1]

logging.basicConfig(level=logging.ERROR)
logging.getLogger('cflib.crazyflie.mem').setLevel(logging.CRITICAL)

MIN_DISTANCE = 0.2
VELOCITY = 0.5
MAX_DURATION = 60.0

def is_close(r): return (r is not None) and (r < MIN_DISTANCE)

def wait_for_param(scf, name, expected='1', timeout=5.0, poll=0.1):
    end = time.time() + timeout
    while time.time() < end:
        try:
            if str(scf.cf.param.get_value(name)) == str(expected):
                return True
        except Exception:
            pass
        time.sleep(poll)
    return False

def wait_for_multiranger_ready(mr, timeout=3.0, poll=0.05):
    end = time.time() + timeout
    while time.time() < end:
        if any(s is not None for s in [mr.front, mr.back, mr.left, mr.right, mr.up]):
            return True
        time.sleep(poll)
    return False

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie(rw_cache='./cache')

    try:
        with SyncCrazyflie(URI, cf=cf) as scf:
            # Optional: Deck-Erkennung (nur Warnung)
            flow_ok = wait_for_param(scf, 'deck.bcFlow2', '1', timeout=3.0)
            mr_ok   = wait_for_param(scf, 'deck.bcMultiranger', '1', timeout=3.0)
            if not (flow_ok and mr_ok):
                missing = []
                if not flow_ok: missing.append('Flow v2 (deck.bcFlow2)')
                if not mr_ok:   missing.append('Multi-ranger (deck.bcMultiranger)')
                print(f'WARNUNG: Deck(s) nicht erkannt: {", ".join(missing)}.')

            # Auto-Takeoff durch default_height, KEIN zusätzliches take_off()
            with MotionCommander(scf, default_height=0.3) as mc:
                # Einmal explizit 0-Setpoint -> saubere Initialisierung
                mc.start_linear_motion(0.0, 0.0, 0.0)
                time.sleep(0.2)

                with Multiranger(scf) as mr:
                    if not wait_for_multiranger_ready(mr, timeout=3.0):
                        print('WARNUNG: Multiranger liefert noch keine Werte.')

                    start_t = time.time()
                    try:
                        while True:
                            vx = vy = 0.0
                            if is_close(mr.front): vx -= VELOCITY
                            if is_close(mr.back):  vx += VELOCITY
                            if is_close(mr.left):  vy -= VELOCITY
                            if is_close(mr.right): vy += VELOCITY
                            if is_close(mr.up):
                                print('UP-Hindernis erkannt – lande.')
                                break

                            # Wenn alle Sensoren None -> nicht „blind“ fahren
                            if all(v is None for v in [mr.front, mr.back, mr.left, mr.right, mr.up]):
                                vx = vy = 0.0

                            mc.start_linear_motion(float(vx), float(vy), 0.0)

                            if time.time() - start_t > MAX_DURATION:
                                print('Maximale Flugzeit erreicht – lande.')
                                break

                            time.sleep(0.1)

                    finally:
                        # Beim Verlassen des with-Blocks landet der Commander automatisch
                        mc.stop()

        print('Demo terminated!')

    except Exception as e:
        print(f'FEHLER: {e}')
        print('Tipp: Power-Cycle durchführen. Falls `deck.bcMultiranger` weiter 0 bleibt, Deck neu stecken/tauschen.')
