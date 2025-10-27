import time
import math
import random
import signal
import sys
import cflib.crtp
from cflib.crazyflie.swarm import Swarm

URIS = {
    'radio://0/80/2M/1337691337',  # Drohne 1
    'radio://0/80/2M/E7E7E7E7E5',  # Drohne 2
}

# Flugparameter
MIN_HEIGHT = 0.5
MAX_HEIGHT = 1.2
FLUGDAUER = 45  # Sekunden
MARGIN = 0.5  # Abstand zum Rand
GESCHWINDIGKEIT = 0.4  # m/s

# Flugbereich definieren
RAUM_BREITE = 3.0
RAUM_TIEFE = 3.0

# Globaler Zustand für Notaus
notaus_aktiv = False
swarm_instanz = None

def notaus_handler(signum, frame):
    """Notaus bei Ctrl+C oder System-Signal"""
    global notaus_aktiv
    print(f"\n!!! NOTAUS AKTIVIERT !!! Signal: {signum}")
    notaus_aktiv = True
    sofort_landen()
    sys.exit(1)

def sofort_landen():
    """Lande alle Drohnen sofort"""
    global swarm_instanz
    print("=== SOFORTLANDUNG WIRD EINGELEITET ===")
    
    if swarm_instanz:
        try:
            swarm_instanz.parallel_safe(lande_sofort)
            time.sleep(3)
        except Exception as e:
            print(f"Fehler bei Sofortlandung: {e}")
    print("=== ALLE DROHNEN GELANDET ===")

def lande_sofort(scf):
    """Lande eine einzelne Drohne sofort"""
    try:
        cf = scf.cf
        print(f"[NOTAUS] Lande {cf.link_uri}")
        cf.high_level_commander.stop()
        time.sleep(0.1)
        cf.high_level_commander.land(0.0, 1.0)
        time.sleep(2.0)
        cf.high_level_commander.stop()
    except Exception as e:
        print(f"[NOTAUS] Fehler bei {scf.cf.link_uri}: {e}")

# Signal-Handler registrieren
signal.signal(signal.SIGINT, notaus_handler)
signal.signal(signal.SIGTERM, notaus_handler)

# Drohnenzustände
drohnen_zustaende = {}

def init_drohne(uri):
    """Initialisiert den Zustand einer Drohne"""
    # Startposition an einem zufälligen Rand
    rand = random.choice(['links', 'rechts', 'unten', 'oben'])
    if rand == 'links':
        x = -RAUM_BREITE/2 + MARGIN
        y = random.uniform(-RAUM_TIEFE/2 + MARGIN, RAUM_TIEFE/2 - MARGIN)
        richtung = random.uniform(-math.pi/4, math.pi/4)  # Nach rechts
    elif rand == 'rechts':
        x = RAUM_BREITE/2 - MARGIN
        y = random.uniform(-RAUM_TIEFE/2 + MARGIN, RAUM_TIEFE/2 - MARGIN)
        richtung = random.uniform(math.pi*3/4, math.pi*5/4)  # Nach links
    elif rand == 'unten':
        x = random.uniform(-RAUM_BREITE/2 + MARGIN, RAUM_BREITE/2 - MARGIN)
        y = -RAUM_TIEFE/2 + MARGIN
        richtung = random.uniform(math.pi/4, math.pi*3/4)  # Nach oben
    else:  # oben
        x = random.uniform(-RAUM_BREITE/2 + MARGIN, RAUM_BREITE/2 - MARGIN)
        y = RAUM_TIEFE/2 - MARGIN
        richtung = random.uniform(math.pi*5/4, math.pi*7/4)  # Nach unten
    
    z = random.uniform(MIN_HEIGHT, MAX_HEIGHT)
    
    drohnen_zustaende[uri] = {
        'position': (x, y, z),
        'richtung': richtung,
        'zielhoehe': z
    }

def berechne_naechsten_abschnitt(uri):
    """Berechnet den nächsten Abschnitt bis zum Rand"""
    zustand = drohnen_zustaende[uri]
    x, y, z = zustand['position']
    richtung = zustand['richtung']
    
    # Berechne Zeiten zu den Rändern
    t_links = float('inf')
    t_rechts = float('inf')
    t_unten = float('inf')
    t_oben = float('inf')
    
    cos_theta = math.cos(richtung)
    sin_theta = math.sin(richtung)
    
    if cos_theta < 0:  # Bewegung nach links
        t_links = ((-RAUM_BREITE/2 + MARGIN) - x) / (cos_theta * GESCHWINDIGKEIT)
    elif cos_theta > 0:  # Bewegung nach rechts
        t_rechts = ((RAUM_BREITE/2 - MARGIN) - x) / (cos_theta * GESCHWINDIGKEIT)
    
    if sin_theta < 0:  # Bewegung nach unten
        t_unten = ((-RAUM_TIEFE/2 + MARGIN) - y) / (sin_theta * GESCHWINDIGKEIT)
    elif sin_theta > 0:  # Bewegung nach oben
        t_oben = ((RAUM_TIEFE/2 - MARGIN) - y) / (sin_theta * GESCHWINDIGKEIT)
    
    # Finde die kleinste positive Zeit
    t_min = min(t for t in [t_links, t_rechts, t_unten, t_oben] if t > 0)
    
    # Zielpunkt berechnen
    ziel_x = x + math.cos(richtung) * GESCHWINDIGKEIT * t_min
    ziel_y = y + math.sin(richtung) * GESCHWINDIGKEIT * t_min
    
    # Neue Richtung nach Aufprall
    if t_min == t_links or t_min == t_rechts:
        neue_richtung = math.pi - richtung  # Horizontal spiegeln
    else:
        neue_richtung = -richtung  # Vertikal spiegeln
    
    return ziel_x, ziel_y, neue_richtung, t_min

def berechne_min_abstand(uri1, uri2, t_max):
    """Berechnet den minimalen Abstand zweier Drohnen im Zeitintervall [0, t_max]"""
    zustand1 = drohnen_zustaende[uri1]
    zustand2 = drohnen_zustaende[uri2]
    
    x1, y1, z1 = zustand1['position']
    x2, y2, z2 = zustand2['position']
    richtung1 = zustand1['richtung']
    richtung2 = zustand2['richtung']
    
    # Relative Position
    x_rel = x1 - x2
    y_rel = y1 - y2
    
    # Geschwindigkeitsvektoren
    vx1 = math.cos(richtung1) * GESCHWINDIGKEIT
    vy1 = math.sin(richtung1) * GESCHWINDIGKEIT
    vx2 = math.cos(richtung2) * GESCHWINDIGKEIT
    vy2 = math.sin(richtung2) * GESCHWINDIGKEIT
    
    # Relative Geschwindigkeit
    vx_rel = vx1 - vx2
    vy_rel = vy1 - vy2
    
    # Zeitpunkt des minimalen Abstands
    if abs(vx_rel) < 1e-9 and abs(vy_rel) < 1e-9:
        # Parallele Bewegung - Abstand bleibt konstant
        t_min = 0
    else:
        t_min = -(x_rel * vx_rel + y_rel * vy_rel) / (vx_rel**2 + vy_rel**2)
    
    # Begrenze auf das Intervall [0, t_max]
    t_min = max(0, min(t_max, t_min))
    
    # Positionen zum Zeitpunkt t_min
    x1_t = x1 + vx1 * t_min
    y1_t = y1 + vy1 * t_min
    x2_t = x2 + vx2 * t_min
    y2_t = y2 + vy2 * t_min
    
    # Abstand berechnen
    abstand = math.sqrt((x1_t - x2_t)**2 + (y1_t - y2_t)**2)
    
    return abstand, t_min

def flug_abschnitt(scf):
    """Führt einen Flugabschnitt (von Rand zu Rand) durch"""
    global notaus_aktiv
    
    if notaus_aktiv:
        return
    
    cf = scf.cf
    commander = cf.high_level_commander
    uri = cf.link_uri
    
    # Nächsten Abschnitt berechnen
    ziel_x, ziel_y, neue_richtung, t_flug = berechne_naechsten_abschnitt(uri)
    
    # Zustand aktualisieren
    drohnen_zustaende[uri]['richtung'] = neue_richtung
    drohnen_zustaende[uri]['position'] = (ziel_x, ziel_y, drohnen_zustaende[uri]['zielhoehe'])
    
    # Zum Ziel fliegen
    commander.go_to(ziel_x, ziel_y, drohnen_zustaende[uri]['zielhoehe'], 0, t_flug)
    
    return t_flug

def takeoff_sequence(scf):
    """Startet die Drohnen auf sichere Höhe"""
    global notaus_aktiv
    if notaus_aktiv:
        return
        
    cf = scf.cf
    commander = cf.high_level_commander
    
    start_hoehe = MIN_HEIGHT + 0.2
    commander.takeoff(start_hoehe, 2.0)
    time.sleep(2.5)

def land_sequence(scf):
    """Synchrones Landen"""
    global notaus_aktiv
    if notaus_aktiv:
        return
        
    cf = scf.cf
    commander = cf.high_level_commander
    
    commander.land(0.0, 2.0)
    time.sleep(2.5)
    commander.stop()

if __name__ == "__main__":
    cflib.crtp.init_drivers()
    
    try:
        with Swarm(URIS) as swarm:
            swarm_instanz = swarm
            print("=== Starte DVD-Screensaver Schwarmflug ===")
            print(f"Flugbereich: {RAUM_BREITE}x{RAUM_TIEFE}m mit {MARGIN}m Randabstand")
            print("NOTAUS: Drücke Ctrl+C um sofort zu landen!")
            
            # Drohnen initialisieren
            for uri in URIS:
                init_drohne(uri)
            
            # Estimator zurücksetzen
            swarm.reset_estimators()
            time.sleep(1.0)
            
            # Synchron starten
            print("Takeoff...")
            swarm.parallel_safe(takeoff_sequence)
            
            if notaus_aktiv:
                sys.exit(1)
            
            # Hauptflugschleife
            start_zeit = time.time()
            while time.time() - start_zeit < FLUGDAUER and not notaus_aktiv:
                # Kollisionsprüfung für den nächsten Abschnitt
                uris_list = list(URIS)
                if len(uris_list) == 2:
                    uri1, uri2 = uris_list
                    
                    # Flugzeiten für nächsten Abschnitt berechnen
                    ziel_x1, ziel_y1, richtung1, t1 = berechne_naechsten_abschnitt(uri1)
                    ziel_x2, ziel_y2, richtung2, t2 = berechne_naechsten_abschnitt(uri2)
                    
                    t_max = min(t1, t2)
                    min_abstand, t_koll = berechne_min_abstand(uri1, uri2, t_max)
                    
                    print(f"Kollisionsprüfung: Min. Abstand = {min_abstand:.2f}m in {t_koll:.1f}s")
                    
                    # Höhen anpassen falls nötig
                    if min_abstand < 0.8:  # Puffer größer als MIN_ABSTAND
                        if uri1 < uri2:
                            drohnen_zustaende[uri1]['zielhoehe'] = MAX_HEIGHT
                            drohnen_zustaende[uri2]['zielhoehe'] = MIN_HEIGHT
                        else:
                            drohnen_zustaende[uri1]['zielhoehe'] = MIN_HEIGHT
                            drohnen_zustaende[uri2]['zielhoehe'] = MAX_HEIGHT
                        print(f"Kollisionsgefahr! Höhen angepasst: {uri1[-4:]} auf {drohnen_zustaende[uri1]['zielhoehe']:.1f}m, {uri2[-4:]} auf {drohnen_zustaende[uri2]['zielhoehe']:.1f}m")
                    else:
                        # Zufällige Höhen zuweisen
                        for uri in URIS:
                            drohnen_zustaende[uri]['zielhoehe'] = random.uniform(MIN_HEIGHT, MAX_HEIGHT)
                
                # Abschnitt fliegen
                flugzeiten = []
                for uri in URIS:
                    t_flug = flug_abschnitt(swarm._cfs[uri])
                    flugzeiten.append(t_flug)
                
                # Auf längste Flugzeit warten
                max_flugzeit = max(flugzeiten) if flugzeiten else 0
                time.sleep(max_flugzeit + 0.5)  # Kleine Pufferzeit
                
                print(f"Abschnitt completed. Verbleibende Zeit: {FLUGDAUER - (time.time() - start_zeit):.1f}s")
            
            if notaus_aktiv:
                sys.exit(1)
            
            # Synchron landen
            print("Lande...")
            swarm.parallel_safe(land_sequence)
            
            print("=== Flug beendet ===")
            
    except Exception as e:
        print(f"Fehler während des Flugs: {e}")
        sofort_landen()
    finally:
        swarm_instanz = None
