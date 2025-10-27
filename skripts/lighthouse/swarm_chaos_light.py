import time
import random
import cflib.crtp
from cflib.crazyflie.swarm import Swarm
import math

URIS = {
    'radio://0/80/2M/1337691337',
    'radio://0/80/2M/E7E7E7E7E5',
}

# Flugparameter
MIN_HEIGHT = 0.5
MAX_HEIGHT = 1.2
FLUGDAUER = 30  # Sekunden
MIN_ABSTAND = 0.4  # Mindestabstand zwischen Drohnen
MAX_GESCHWINDIGKEIT = 0.4  # m/s

# Flugbereich definieren
RAUM_BREITE = 2.0
RAUM_TIEFE = 2.0

def chaotische_bewegung(scf):
    cf = scf.cf
    commander = cf.high_level_commander
    
    start_time = time.time()
    ziel_positions = {}  # Verfolge Zielpositionen für Kollisionsvermeidung
    
    # Initiale zufällige Positionen
    for uri in URIS:
        ziel_positions[uri] = (
            random.uniform(-RAUM_BREITE/2, RAUM_BREITE/2),
            random.uniform(-RAUM_TIEFE/2, RAUM_TIEFE/2),
            random.uniform(MIN_HEIGHT, MAX_HEIGHT)
        )
    
    while time.time() - start_time < FLUGDAUER:
        aktueller_uri = cf.link_uri
        
        # Neue zufällige Zielposition berechnen
        if random.random() < 0.3:  # 30% Chance für Richtungsänderung
            alt_x, alt_y, alt_z = ziel_positions[aktueller_uri]
            
            # Kleine, zufällige Bewegung
            neue_x = alt_x + random.uniform(-0.3, 0.3)
            neue_y = alt_y + random.uniform(-0.3, 0.3)
            neue_z = alt_z + random.uniform(-0.2, 0.2)
            
            # Innerhalb des Raums halten
            neue_x = max(-RAUM_BREITE/2, min(RAUM_BREITE/2, neue_x))
            neue_y = max(-RAUM_TIEFE/2, min(RAUM_TIEFE/2, neue_y))
            neue_z = max(MIN_HEIGHT, min(MAX_HEIGHT, neue_z))
            
            # Kollisionsvermeidung
            kollisionsfrei = False
            versuche = 0
            while not kollisionsfrei and versuche < 10:
                kollisionsfrei = True
                for andere_uri, andere_pos in ziel_positions.items():
                    if andere_uri != aktueller_uri:
                        abstand = math.sqrt(
                            (neue_x - andere_pos[0])**2 + 
                            (neue_y - andere_pos[1])**2 + 
                            (neue_z - andere_pos[2])**2
                        )
                        if abstand < MIN_ABSTAND:
                            kollisionsfrei = False
                            # Leichte Anpassung für Kollisionsvermeidung
                            neue_x += random.uniform(-0.2, 0.2)
                            neue_y += random.uniform(-0.2, 0.2)
                            break
                versuche += 1
            
            if kollisionsfrei:
                ziel_positions[aktueller_uri] = (neue_x, neue_y, neue_z)
                
                # Bewegung ausführen
                flugzeit = random.uniform(0.8, 2.0)  # Zufällige Flugzeit
                commander.go_to(neue_x, neue_y, neue_z, 0, flugzeit)
        
        time.sleep(0.1)  # Kurze Pause zwischen Bewegungen

def takeoff_sequence(scf):
    """Gleichmäßiger Start aller Drohnen"""
    cf = scf.cf
    commander = cf.high_level_commander
    
    # Auf gleiche Höhe starten
    start_hoehe = MIN_HEIGHT + 0.1
    commander.takeoff(start_hoehe, 2.0)
    time.sleep(2.5)

def land_sequence(scf):
    """Synchrones Landen"""
    cf = scf.cf
    commander = cf.high_level_commander
    
    commander.land(0.0, 2.0)
    time.sleep(2.5)
    commander.stop()

def spezielle_manoever(scf):
    """Spezielle chaotische Manöver"""
    cf = scf.cf
    commander = cf.high_level_commander
    
    manoeuver = random.choice(['kreis', 'zickzack', 'spirale', 'schnell'])
    
    if manoeuver == 'kreis':
        # Kleiner Kreis
        radius = 0.3
        for winkel in range(0, 360, 30):
            x = radius * math.cos(math.radians(winkel))
            y = radius * math.sin(math.radians(winkel))
            commander.go_to(x, y, MIN_HEIGHT + 0.3, 0, 1.0)
            time.sleep(0.3)
    
    elif manoeuver == 'zickzack':
        # Zickzack Bewegung
        for i in range(3):
            commander.go_to(0.4, 0, MIN_HEIGHT, 0, 0.8)
            time.sleep(0.5)
            commander.go_to(-0.4, 0, MIN_HEIGHT + 0.2, 0, 0.8)
            time.sleep(0.5)

if __name__ == "__main__":
    cflib.crtp.init_drivers()
    
    with Swarm(URIS) as swarm:
        print("=== Starte chaotischen Schwarmflug ===")
        
        # Estimator zurücksetzen
        swarm.reset_estimators()
        time.sleep(1.0)
        
        # Synchron starten
        print("Takeoff...")
        swarm.parallel_safe(takeoff_sequence)
        
        # Haupt-Chaos-Phase
        print("Starte chaotische Bewegungen...")
        start_zeit = time.time()
        
        while time.time() - start_zeit < FLUGDAUER:
            # Zufällig zwischen verschiedenen Bewegungsmustern wechseln
            if random.random() < 0.7:  # 70% normale chaotische Bewegung
                swarm.parallel_safe(chaotische_bewegung)
            else:  # 30% spezielle Manöver
                swarm.parallel_safe(spezielle_manoever)
            
            time.sleep(2.0)
        
        # Synchron landen
        print("Lande...")
        swarm.parallel_safe(land_sequence)
        
        print("=== Flug beendet ===")
