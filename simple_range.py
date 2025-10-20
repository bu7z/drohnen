#!/usr/bin/env python3
import time
from collections import deque
from cflib import crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig

URI = 'radio://0/80/2M/1337691337'

BAD_OOR_MAX = 5          # wie oft OOR/0 in Folge erlaubt
NOUPDATE_TIMEOUT = 1.0   # s ohne Update -> bad
STUCK_WINDOW = 1.5       # s innerhalb derer "stuck" erkannt wird
STUCK_EPS = 0.005        # m Änderung kleiner als EPS -> stuck

def safe_land(scf, mc, have_z=False, get_z=lambda: None, vdown=0.35, timeout=6.0):
    """Nicht-blockierendes, robustes Landen mit optionaler Bodenerkennung über zrange."""
    print("⚠️  Watchdog: Starte Sicherheitslandung ...")
    t0 = time.time()
    try:
        while time.time() - t0 < timeout:
            # sanft nach unten fahren
            mc.start_linear_motion(0.0, 0.0, -vdown)
            time.sleep(0.1)
            if have_z:
                z = get_z()
                if z is not None and z < 0.06:   # ~6 cm => Boden
                    break
        # Stop-Setpoint senden
        mc.stop()
        # sicherheitshalber Motoren stoppen
        scf.cf.commander.send_stop_setpoint()
        time.sleep(0.2)
    except Exception as e:
        print(f"Landungsroutine: {e}")
    finally:
        # Kontext verlässt später trotzdem ordentlich
        pass

def hover_with_watchdog():
    crtp.init_drivers(enable_debug_driver=False)
    print("Verbinde zum Crazyflie...")

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cfcache')) as scf:
        print("✅ Verbindung hergestellt.")
        cf = scf.cf
        cf.log.use_toc_cache = False
        cf.param.use_toc_cache = False

        # --- prüfen, ob zrange verfügbar ist
        toc = cf.log.toc.toc
        available = {f"{grp}.{name}" for grp, names in toc.items() for name in names}
        have_zrange = "range.zrange" in available

        # LogConfig
        lg = LogConfig(name="ranges", period_in_ms=100)
        if have_zrange:
            lg.add_variable("range.zrange", "uint16_t")
            print("Sensor: using range.zrange")
        else:
            print("Sensor: kein range.zrange verfügbar -> Watchdog nutzt nur Zeit/Heuristik")

        # shared state
        last_mm = [None]
        last_ts = [time.time()]
        bad_count = [0]
        samples = deque(maxlen=20)  # ~2 s bei 100 ms

        def mm_to_m(mm):
            if mm is None or mm == 0 or mm >= 65000:
                return None
            return mm / 1000.0

        def cb(ts, data, logconf):
            if have_zrange:
                mm = data.get("range.zrange")
                last_mm[0] = mm
                last_ts[0] = time.time()

        if have_zrange:
            cf.log.add_config(lg)
            lg.data_received_cb.add_callback(cb)
            lg.start()
            time.sleep(0.2)

        def read_z():
            return mm_to_m(last_mm[0])

        # --- Flug
        emergency = [False]
        try:
            with MotionCommander(scf, default_height=1.0) as mc:
                print("🛫 Hover auf 1.0 m ... (10 s)")

                t_start = time.time()
                while time.time() - t_start < 10.0:
                    time.sleep(0.1)

                    # --- Watchdog Checks
                    now = time.time()
                    z = read_z() if have_zrange else None

                    # 1) kein Update
                    if have_zrange and (now - last_ts[0]) > NOUPDATE_TIMEOUT:
                        print("Watchdog: kein Sensor-Update")
                        emergency[0] = True

                    # 2) OOR/0-Wert
                    if have_zrange and z is None:
                        bad_count[0] += 1
                        if bad_count[0] >= BAD_OOR_MAX:
                            print("Watchdog: zu viele OOR/0 Messungen")
                            emergency[0] = True
                    else:
                        bad_count[0] = 0

                    # 3) Stuck-Value (nahezu konstant)
                    if have_zrange and z is not None:
                        samples.append((now, z))
                        # Fenster
                        while samples and (now - samples[0][0]) > STUCK_WINDOW:
                            samples.popleft()
                        if len(samples) >= 5:
                            zmin = min(s for _, s in samples)
                            zmax = max(s for _, s in samples)
                            if (zmax - zmin) < STUCK_EPS:
                                print("Watchdog: Sensor scheint 'stuck'")
                                emergency[0] = True

                    # optional: Ausgabe alle 1 s
                    if int((now - t_start)*10) % 10 == 0:
                        out = f"{z:.2f} m" if z is not None else "n/a"
                        print(f"Aktueller Bodenabstand: {out}")

                    if emergency[0]:
                        safe_land(scf, mc, have_z=have_zrange, get_z=read_z)
                        break

                if not emergency[0]:
                    print("Leite Landung ein ...")
                    # eigene, nicht-blockierende Landung (vermeidet lange sleep()s in der Lib)
                    safe_land(scf, mc, have_z=have_zrange, get_z=read_z)

        except KeyboardInterrupt:
            print("⌨️  Abbruch durch Benutzer – sichere Landung ...")
            try:
                # sichere Landung, auch wenn Logger/Threads laufen
                with MotionCommander(scf, default_height=0.3) as mc:
                    safe_land(scf, mc, have_z=have_zrange, get_z=read_z)
            except Exception:
                # letzter Ausweg
                cf.commander.send_stop_setpoint()
                time.sleep(0.2)
        finally:
            if have_zrange:
                try:
                    lg.stop()
                except Exception:
                    pass
            print("🔚 Ende.")

if __name__ == "__main__":
    hover_with_watchdog()
