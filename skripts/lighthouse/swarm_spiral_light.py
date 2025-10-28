# swarm_intelligent_spiral.py
import time
import math
import random
import signal
import sys
import cflib.crtp
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig

URIS = {
    'radio://0/80/2M/1337691337',
    'radio://0/80/2M/E7E7E7E7E5',
}

# === Intelligente Parameter ===
INITIAL_HEIGHT = 0.6
FORMATION_HEIGHT = 1.0
SPIRAL_HEIGHT = 1.5
SPIRAL_TIME = 15.0
LOOPS = 2.0
MIN_SPACING = 0.8  # Mindestabstand zwischen Drohnen
MAX_SPACING = 1.5  # Maximalabstand für Formation

# Globale Variable für Notaus
notaus_aktiv = False

def notaus_handler(signum, frame):
    """Elegante Notlandung bei Ctrl+C"""
    global notaus_aktiv
    print(f"\n💫 NOTAUS: Elegante Landung wird eingeleitet...")
    notaus_aktiv = True
    sys.exit(1)

# Signal-Handler registrieren
signal.signal(signal.SIGINT, notaus_handler)
signal.signal(signal.SIGTERM, notaus_handler)

def _enable_hl_estimator(cf):
    """Optimierte Initialisierung"""
    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('commander.enHighLevel', '1')
    time.sleep(0.05)
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.05)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.5)

def _read_pose_quick(scf):
    """Schnelle Pose-Erfassung"""
    pose = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    pose_received = False
    
    def _pose_callback(timestamp, data, logconf):
        nonlocal pose_received
        pose['x'] = data['stateEstimate.x']
        pose['y'] = data['stateEstimate.y'] 
        pose['z'] = data['stateEstimate.z']
        pose_received = True
    
    lg_conf = LogConfig(name='Pose', period_in_ms=50)
    lg_conf.add_variable('stateEstimate.x', 'float')
    lg_conf.add_variable('stateEstimate.y', 'float')
    lg_conf.add_variable('stateEstimate.z', 'float')
    
    scf.cf.log.add_config(lg_conf)
    lg_conf.data_received_cb.add_callback(_pose_callback)
    lg_conf.start()
    
    for _ in range(20):
        if pose_received:
            break
        time.sleep(0.05)
    
    lg_conf.stop()
    return pose

def _calculate_optimal_formation(poses):
    """Berechnet intelligente Formation basierend auf aktuellen Positionen"""
    uris = list(poses.keys())
    
    if len(uris) != 2:
        raise ValueError("Nur für 2 Drohnen optimiert")
    
    # Aktuelle Positionen
    pos1 = poses[uris[0]]
    pos2 = poses[uris[1]]
    
    # Berechne Mittelpunkt
    center_x = (pos1['x'] + pos2['x']) / 2
    center_y = (pos1['y'] + pos2['y']) / 2
    
    # Aktueller Abstand
    current_distance = math.sqrt(
        (pos1['x'] - pos2['x'])**2 + 
        (pos1['y'] - pos2['y'])**2
    )
    
    # Optimalen Abstand berechnen (zwischen MIN und MAX Spacing)
    optimal_distance = max(MIN_SPACING, min(MAX_SPACING, current_distance))
    
    # Winkel zwischen den Drohnen berechnen
    angle = math.atan2(pos2['y'] - pos1['y'], pos2['x'] - pos1['x'])
    
    # Perpendikuläre Richtung für Spirale
    spiral_angle = angle + math.pi/2  # 90° gedreht
    
    # Startpositionen für Spirale berechnen
    start_radius = optimal_distance / 2
    
    # Positionen für Spirale (gegenüberliegend)
    start_positions = {
        uris[0]: {
            'x': center_x + start_radius * math.cos(spiral_angle),
            'y': center_y + start_radius * math.sin(spiral_angle),
            'angle': spiral_angle
        },
        uris[1]: {
            'x': center_x + start_radius * math.cos(spiral_angle + math.pi),
            'y': center_y + start_radius * math.sin(spiral_angle + math.pi),
            'angle': spiral_angle + math.pi
        }
    }
    
    print(f"🎯 Intelligente Formation berechnet:")
    print(f"   Mittelpunkt: ({center_x:.2f}, {center_y:.2f})")
    print(f"   Optimaler Abstand: {optimal_distance:.2f}m")
    print(f"   Startradius: {start_radius:.2f}m")
    
    return center_x, center_y, start_radius, start_positions

def _smooth_takeoff(scf, height, duration=2.5):
    """Sanfter Takeoff"""
    if notaus_aktiv:
        return
    scf.cf.high_level_commander.takeoff(height, duration)
    time.sleep(duration + 0.1)

def _graceful_land(scf, duration=3.0):
    """Sanfte Landung"""
    if notaus_aktiv:
        return
    hlc = scf.cf.high_level_commander
    hlc.land(0.0, duration)
    time.sleep(duration + 0.2)
    hlc.stop()

def _move_to_formation_position(scf, target_x, target_y, target_z, duration=4.0):
    """Bewegt Drohne zur Formationsposition"""
    if notaus_aktiv:
        return
        
    uri = scf.cf.link_uri
    current_pose = _read_pose_quick(scf)
    
    print(f"🔄 {uri[-4:]} bewegt sich zur Formation: "
          f"({current_pose['x']:.2f},{current_pose['y']:.2f}) → "
          f"({target_x:.2f},{target_y:.2f})")
    
    scf.cf.high_level_commander.go_to(target_x, target_y, target_z, 0, duration)
    time.sleep(duration + 0.5)

def _generate_adaptive_spiral(center_x, center_y, start_radius, start_angle, 
                            start_z, end_z, loops, total_time, steps_per_sec):
    """Generiert adaptive Spirale basierend auf Startposition"""
    num_points = int(total_time * steps_per_sec)
    
    # Endradius basierend auf Startradius berechnen
    end_radius = start_radius * 2.5  # Dynamisch skalierend
    
    for i in range(num_points):
        if notaus_aktiv:
            break
            
        t = i / num_points
        
        # Weiche Übergänge
        smooth_t = _ease_in_out_cubic(t)
        
        # Adaptive Radius- und Höhenentwicklung
        radius = start_radius + (end_radius - start_radius) * smooth_t
        height = start_z + (end_z - start_z) * smooth_t
        
        # Winkelentwicklung mit konstanter Winkelgeschwindigkeit
        angle = start_angle - 2 * math.pi * loops * smooth_t
        
        # Position berechnen
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        z = height
        
        dt = total_time / num_points
        
        yield x, y, z, dt

def _ease_in_out_cubic(t):
    """Kubischer Ease-In-Out für weiche Übergänge"""
    if t < 0.5:
        return 4 * t * t * t
    else:
        return 1 - math.pow(-2 * t + 2, 3) / 2

def _execute_adaptive_trajectory(scf, trajectory):
    """Führt adaptive Trajektorie aus"""
    if notaus_aktiv:
        return
        
    hlc = scf.cf.high_level_commander
    uri = scf.cf.link_uri
    
    print(f"🌀 {uri[-4:]} startet adaptive Spirale")
    
    trajectory_list = list(trajectory)  # In Liste umwandeln für Längenberechnung
    
    for i, (x, y, z, dt) in enumerate(trajectory_list):
        if notaus_aktiv:
            break
            
        hlc.go_to(x, y, z, 0.0, dt)
        
        # Fortschritt anzeigen
        if i % 15 == 0:  # Weniger häufige Updates
            progress = (i / len(trajectory_list)) * 100
            print(f"   {uri[-4:]}: {progress:.0f}% - Radius: {math.sqrt((x-center_x)**2 + (y-center_y)**2):.2f}m")
        
        time.sleep(dt)

def emergency_land_all(swarm, reason="Unbekannt"):
    """Elegante Notlandung aller Drohnen"""
    print(f"🆘 Notlandung: {reason}")
    try:
        swarm.parallel_safe(_graceful_land)
    except Exception as e:
        print(f"⚠️  Warnung bei Notlandung: {e}")

def intelligent_spiral_flight(swarm):
    """Intelligente Spiralflug mit adaptiver Formation"""
    global notaus_aktiv, center_x, center_y
    
    if notaus_aktiv:
        return
        
    print("🧠 Starte intelligenten adaptiven Spiralflug...")
    
    # Initialisiere alle Drohnen
    print("🔧 Initialisiere Drohnen...")
    for scf in swarm._cfs.values():
        _enable_hl_estimator(scf.cf)
    
    # Synchroner Takeoff auf Initialhöhe
    print("🛫 Synchroner Takeoff...")
    swarm.parallel_safe(lambda scf: _smooth_takeoff(scf, INITIAL_HEIGHT))
    
    if notaus_aktiv:
        return
    
    # Warte kurz für Stabilisierung
    time.sleep(1.0)
    
    # Aktuelle Positionen aller Drohnen erfassen
    print("📡 Erfasse aktuelle Positionen...")
    poses = {}
    for uri, scf in swarm._cfs.items():
        pose = _read_pose_quick(scf)
        poses[uri] = pose
        print(f"   {uri[-4:]}: ({pose['x']:.2f}, {pose['y']:.2f}, {pose['z']:.2f})")
    
    # Intelligente Formation berechnen
    print("🧮 Berechne optimale Formation...")
    center_x, center_y, start_radius, start_positions = _calculate_optimal_formation(poses)
    
    # Auf Formationshöhe steigen
    print("⬆️  Steige auf Formationshöhe...")
    swarm.parallel_safe(lambda scf: _smooth_takeoff(scf, FORMATION_HEIGHT, 2.0))
    
    if notaus_aktiv:
        return
    
    # Zur Formationsposition bewegen
    print("🎯 Positioniere Drohnen in Formation...")
    
    def position_in_formation(scf):
        uri = scf.cf.link_uri
        target = start_positions[uri]
        _move_to_formation_position(scf, target['x'], target['y'], FORMATION_HEIGHT)
    
    # Sequential positionieren für Sicherheit
    for uri, scf in swarm._cfs.items():
        if notaus_aktiv:
            break
        position_in_formation(scf)
    
    if notaus_aktiv:
        return
    
    # Kurze Stabilisierungspause
    print("⏳ Stabilisiere Formation...")
    time.sleep(1.5)
    
    # Adaptive Spiral-Trajektorien generieren
    print("📈 Generiere adaptive Spiral-Trajektorien...")
    trajectories = {}
    
    for uri, scf in swarm._cfs.items():
        start_pos = start_positions[uri]
        trajectories[uri] = _generate_adaptive_spiral(
            center_x, center_y, start_radius, start_pos['angle'],
            FORMATION_HEIGHT, SPIRAL_HEIGHT, LOOPS, SPIRAL_TIME, 12
        )
    
    # Spiralen parallel ausführen
    print("🌀 Starte adaptive Spiralen...")
    
    def fly_adaptive_spiral(scf):
        uri = scf.cf.link_uri
        _execute_adaptive_trajectory(scf, trajectories[uri])
    
    swarm.parallel_safe(fly_adaptive_spiral)
    
    if notaus_aktiv:
        return
        
    # Zurück zur Formation für sanfte Landung
    print("↩️  Rückkehr zur Formation...")
    
    def return_to_formation(scf):
        uri = scf.cf.link_uri
        target = start_positions[uri]
        _move_to_formation_position(scf, target['x'], target['y'], FORMATION_HEIGHT, 3.0)
    
    swarm.parallel_safe(return_to_formation)
    
    if notaus_aktiv:
        return
        
    # Elegante Landung
    print("🛬 Synchrones Landen...")
    swarm.parallel_safe(_graceful_land)
    
    print("✅ Intelligenter Spiralflug erfolgreich beendet!")

if __name__ == "__main__":
    cflib.crtp.init_drivers()
    
    try:
        with Swarm(URIS) as swarm:
            intelligent_spiral_flight(swarm)
    except KeyboardInterrupt:
        print("⏹️  Durch Benutzer abgebrochen")
    except Exception as e:
        print(f"❌ Fehler: {e}")
        emergency_land_all(swarm, f"Exception: {e}")
    finally:
        print("🎉 Programm beendet")
