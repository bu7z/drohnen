# swarm_spiral_lighthouse.py
import time, math
import cflib.crtp
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig

URIS = {
    'radio://0/80/2M/1337691337',
    'radio://0/80/2M/E7E7E7E7E5',
}

# === Parameter ===
TAKEOFF_Z   = 0.4
CLIMB_Z     = 1.5
SPIRAL_TIME = 10.0
LOOPS       = 1.5
R_START     = 0.15
R_END       = 0.6
STEP_HZ     = 10
PHASE_OFF   = math.pi / 4

# ---------- Hilfsfunktionen ----------
def _enable_hl_and_reset(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('commander.enHighLevel', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(1.0)

def _read_pose_once(scf):
    pose = {'x': None, 'y': None, 'z': None}
    done = {'ok': False}

    def _cb(ts, data, logconf):
        pose['x'], pose['y'], pose['z'] = data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']
        done['ok'] = True

    lg = LogConfig(name='state', period_in_ms=100)
    for v in ('x', 'y', 'z'):
        lg.add_variable(f'stateEstimate.{v}', 'float')
    scf.cf.log.add_config(lg)
    lg.data_received_cb.add_callback(_cb)
    lg.start()
    t0 = time.time()
    while not done['ok'] and (time.time() - t0) < 0.5:
        time.sleep(0.01)
    lg.stop()
    return pose

def _get_center_and_roles(swarm):
    poses = {}
    for uri, scf in swarm._cfs.items():
        poses[uri] = _read_pose_once(scf)

    cx = sum(p['x'] for p in poses.values()) / len(poses)
    cy = sum(p['y'] for p in poses.values()) / len(poses)

    dists = {uri: math.hypot(p['x'] - cx, p['y'] - cy) for uri, p in poses.items()}
    leader_uri = min(dists, key=dists.get)
    follower_uri = [u for u in poses if u != leader_uri][0]
    return (cx, cy), leader_uri, follower_uri, poses

def _takeoff(scf, z, dur=2.0):
    hlc = scf.cf.high_level_commander
    hlc.takeoff(z, dur)
    time.sleep(dur + 0.2)

def _land(scf, dur=2.0):
    hlc = scf.cf.high_level_commander
    hlc.land(0.0, dur)
    time.sleep(dur + 0.2)
    hlc.stop()

def _spiral_points(center, z0, climb, loops, r0, r1, total_time, step_hz, theta0):
    n = max(1, int(total_time * step_hz))
    cx, cy = center
    for i in range(1, n + 1):
        t = i / n
        r = r0 + (r1 - r0) * t
        z = z0 + climb * t
        theta = theta0 - 2 * math.pi * loops * t  # CW
        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)
        yield x, y, z, 1.0 / step_hz

def _run_for_uri(scf, plan):
    hlc = scf.cf.high_level_commander
    for x, y, z, dt in plan:
        hlc.go_to(x, y, z, 0.0, relative=False)
        time.sleep(dt)

# ---------- Missionslogik ----------
def flight(swarm):
    for scf in swarm._cfs.values():
        _enable_hl_and_reset(scf.cf)

    swarm.parallel_safe(lambda scf: _takeoff(scf, TAKEOFF_Z, 2.0))

    center, leader_uri, follower_uri, poses = _get_center_and_roles(swarm)
    print(f"Center≈({center[0]:.2f},{center[1]:.2f})  Leader={leader_uri}  Follower={follower_uri}")

    def start_theta(p):
        return math.atan2(p['y'] - center[1], p['x'] - center[0])

    theta_leader   = start_theta(poses[leader_uri])
    theta_follower = start_theta(poses[follower_uri]) - PHASE_OFF

    plans = {
        leader_uri:   list(_spiral_points(center, TAKEOFF_Z, CLIMB_Z, LOOPS, R_START, R_END, SPIRAL_TIME, STEP_HZ, theta_leader)),
        follower_uri: list(_spiral_points(center, TAKEOFF_Z, CLIMB_Z, LOOPS, R_START, R_END, SPIRAL_TIME, STEP_HZ, theta_follower)),
    }

    # === Statt uris=[]: pro Drohne separat starten ===
    _run_for_uri(swarm._cfs[leader_uri],   plans[leader_uri])
    _run_for_uri(swarm._cfs[follower_uri], plans[follower_uri])

    time.sleep(0.5)
    swarm.parallel_safe(lambda scf: _land(scf, 2.0))

# ---------- Main ----------
if __name__ == "__main__":
    cflib.crtp.init_drivers()
    with Swarm(URIS) as swarm:
        flight(swarm)
