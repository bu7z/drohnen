#!/usr/bin/env python3
"""
Verbesserter Crazyflie Obstacle Avoidance Controller
Mit erweiterter Fehlerbehandlung, Parametrisierung und besseren Flugeigenschaften
"""

import time
import logging
import signal
import sys
from dataclasses import dataclass
from typing import Optional, Tuple

from cflib import crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

# ===== KONFIGURATION =====
URI = 'radio://0/80/2M/1337691337'  # Anpassen an deine Crazyflie

@dataclass
class FlightConfig:
    """Flugparameter-Konfiguration"""
    # Höheneinstellungen
    target_height: float = 0.5  # m - Zielhöhe
    max_climb_rate: float = 0.25  # m/s - max Steigen/Sinken
    ceiling_threshold: float = 1.5  # m - Deckennähe
    
    # XY-Bewegung
    max_xy_velocity: float = 0.35  # m/s - max seitlich/vor/zurück
    obstacle_threshold: float = 0.6  # m - Hinderniserkennung
    
    # Regelparameter
    deadband: float = 0.04  # m/s - Totzone für kleine Bewegungen
    ema_alpha: float = 0.3  # EMA-Glättungsfaktor
    control_period: float = 0.05  # s - Regelintervall (~20Hz)
    
    # Höhenregler
    height_p_gain: float = 1.5  # P-Verstärkung für Höhe
    
    # Sicherheit
    emergency_distance: float = 0.15  # m - Notfallabstand
    battery_check: bool = True  # Batterieüberwachung

class CrazyflieController:
    """Hauptcontroller für die Crazyflie mit Hindernisvermeidung"""
    
    def __init__(self, uri: str, config: FlightConfig):
        self.uri = uri
        self.config = config
        self.scf = None
        self.mc = None
        self.mr = None
        self._running = False
        
        # EMA-Werte für Sensordaten
        self.ema_distances = {
            'front': None, 'back': None, 
            'left': None, 'right': None,
            'up': None, 'down': None
        }
        
        self._setup_logging()
        
    def _setup_logging(self):
        """Logger konfigurieren"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        logging.getLogger('cflib').setLevel(logging.WARNING)
        self.logger = logging.getLogger('CrazyflieController')
    
    def _ema_filter(self, prev: Optional[float], new: Optional[float]) -> Optional[float]:
        """Exponentieller Moving Average Filter"""
        if prev is None or new is None:
            return new
        return self.config.ema_alpha * new + (1 - self.config.ema_alpha) * prev
    
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Begrenzt Wert auf Bereich"""
        return max(min_val, min(max_val, value))
    
    def _repulsion_force(self, distance: Optional[float], threshold: float, 
                        max_velocity: float) -> float:
        """
        Berechnet Repulsionskraft basierend auf Distanz
        Returns: Geschwindigkeit (0..max_velocity)
        """
        if distance is None or distance >= threshold:
            return 0.0
        
        # Nichtlineare Response für bessere Kontrolle nahe Hindernissen
        normalized = (threshold - distance) / threshold
        return normalized * max_velocity
    
    def _check_emergency_stop(self, distances: dict) -> bool:
        """Prüft auf Notfall-Stopp Bedingungen"""
        critical_distances = [
            distances['front'], distances['back'],
            distances['left'], distances['right']
        ]
        
        for dist in critical_distances:
            if dist is not None and dist < self.config.emergency_distance:
                self.logger.warning(f"Emergency stop! Distance: {dist:.3f}m")
                return True
        return False
    
    def _update_sensor_data(self) -> dict:
        """Aktualisiert und filtert Sensordaten"""
        current_distances = {
            'front': self.mr.front,
            'back': self.mr.back,
            'left': self.mr.left,
            'right': self.mr.right,
            'up': self.mr.up,
            'down': self.mr.down
        }
        
        # EMA anwenden
        filtered = {}
        for direction, current in current_distances.items():
            filtered[direction] = self._ema_filter(
                self.ema_distances[direction], current
            )
            self.ema_distances[direction] = filtered[direction]
            
        return filtered
    
    def _calculate_height_control(self, down_distance: Optional[float], 
                                up_distance: Optional[float]) -> float:
        """Berechnet vertikale Geschwindigkeit für Höhenregelung"""
        vz = 0.0
        
        # Höhenregelung basierend auf Bodenabstand
        if down_distance is not None:
            height_error = self.config.target_height - down_distance
            vz = self.config.height_p_gain * height_error
            vz = self._clamp(vz, -self.config.max_climb_rate, self.config.max_climb_rate)
        
        # Deckenvermeidung
        if up_distance is not None and up_distance < self.config.ceiling_threshold:
            vz = min(vz, -0.08)  # Leicht sinken bei Deckennähe
            
        return vz
    
    def _calculate_xy_control(self, distances: dict) -> Tuple[float, float]:
        """Berechnet XY-Geschwindigkeiten für Hindernisvermeidung"""
        vx, vy = 0.0, 0.0
        
        # Repulsive Kräfte in alle Richtungen
        vx += self._repulsion_force(
            distances['front'], self.config.obstacle_threshold, self.config.max_xy_velocity
        )
        vx -= self._repulsion_force(
            distances['back'], self.config.obstacle_threshold, self.config.max_xy_velocity
        )
        vy -= self._repulsion_force(
            distances['left'], self.config.obstacle_threshold, self.config.max_xy_velocity
        )
        vy += self._repulsion_force(
            distances['right'], self.config.obstacle_threshold, self.config.max_xy_velocity
        )
        
        # Deadband anwenden
        if abs(vx) < self.config.deadband:
            vx = 0.0
        if abs(vy) < self.config.deadband:
            vy = 0.0
            
        return vx, vy
    
    def _signal_handler(self, signum, frame):
        """Signal Handler für sauberes Beenden"""
        self.logger.info("Shutdown signal received...")
        self._running = False
    
    def _setup_crazyflie(self):
        """Crazyflie initialisieren und konfigurieren"""
        self.logger.info(f"Connecting to {self.uri}...")
        
        # Parameter setzen für stabilen Flug
        try:
            if self.scf.cf.param.get_value('stabilizer.estimator') != '2':
                self.scf.cf.param.set_value('stabilizer.estimator', '2')
                time.sleep(0.3)
                
            # Zusätzliche Einstellungen für bessere Performance
            self.scf.cf.param.set_value('locSrv.extPosStdDev', 0.1)
            self.scf.cf.param.set_value('kalman.initialX', 0.0)
            self.scf.cf.param.set_value('kalman.initialY', 0.0)
            self.scf.cf.param.set_value('kalman.initialZ', 0.5)
            
        except Exception as e:
            self.logger.warning(f"Parameter setting failed: {e}")
    
    def run(self):
        """Hauptflugroutine"""
        signal.signal(signal.SIGINT, self._signal_handler)
        
        try:
            crtp.init_drivers(enable_debug_driver=False)
            
            with SyncCrazyflie(self.uri, cf=Crazyflie()) as self.scf:
                self._setup_crazyflie()
                
                # Multiranger starten
                self.mr = Multiranger(
                    self.scf.cf, 
                    rate_ms=int(self.config.control_period * 1000)
                )
                self.mr.start()
                time.sleep(0.3)  # Warten auf erste Sensordaten
                
                with MotionCommander(self.scf, default_height=self.config.target_height) as self.mc:
                    self.logger.info(f"Hovering at {self.config.target_height:.2f}m")
                    self.logger.info("Use hand gestures to push - Ctrl+C to land")
                    time.sleep(0.8)  # Stabilisierungszeit
                    
                    self._running = True
                    cycle_count = 0
                    
                    while self._running:
                        try:
                            # Sensordaten aktualisieren
                            distances = self._update_sensor_data()
                            
                            # Notfallprüfung
                            if self._check_emergency_stop(distances):
                                self.mc.stop()
                                break
                            
                            # Regelung berechnen
                            vz = self._calculate_height_control(
                                distances['down'], distances['up']
                            )
                            vx, vy = self._calculate_xy_control(distances)
                            
                            # Bewegung ausführen
                            self.mc.start_linear_motion(vx, vy, vz)
                            
                            # Periodisches Logging
                            cycle_count += 1
                            if cycle_count % 20 == 0:
                                self.logger.debug(
                                    f"Control: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} | "
                                    f"Front: {distances['front'] or 'None':.2f}"
                                )
                            
                            time.sleep(self.config.control_period)
                            
                        except Exception as e:
                            self.logger.error(f"Control cycle error: {e}")
                            break
                            
        except Exception as e:
            self.logger.error(f"Flight failed: {e}")
            raise
        finally:
            self._cleanup()
    
    def _cleanup(self):
        """Ressourcen freigeben"""
        self.logger.info("Cleaning up...")
        self._running = False
        
        if self.mr:
            self.mr.stop()
        
        self.logger.info("Cleanup completed")

def main():
    """Hauptprogramm"""
    config = FlightConfig()
    controller = CrazyflieController(URI, config)
    
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\nLanding initiated by user...")
    except Exception as e:
        print(f"Flight failed: {e}")
        sys.exit(1)
    finally:
        print("Flight completed!")

if __name__ == '__main__':
    main()
