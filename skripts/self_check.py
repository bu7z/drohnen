#!/usr/bin/env python3
# cf_health_plus.py
# pip install cflib numpy
import time, math, statistics, logging, argparse, signal
from typing import List, Dict, Tuple, Optional
import numpy as np

from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

# ==================== Konfiguration ====================
DEF_URI = "radio://0/80/2M/E7E7E7E7E5"

# Kalman-Reset Kriterien
KALMAN_MAX_TRIES = 5
KALMAN_STD_OK    = 0.05   # m  -> ẑ muss so ruhig sein
KALMAN_MEAN_OK   = 0.30   # m  -> |ẑ| muss nahe Null liegen
KALMAN_SAMPLES_S = 1.0
KALMAN_RATE_HZ   = 15

# Vibrations-Test (global)
TEST_GLOBAL_THRUST = 14000   # 10000..20000 -> Boden bleiben
TEST_GLOBAL_TIME_S = 1.5     # Dauer pro Thrust-Stufe
TEST_GLOBAL_RATE_HZ= 200     # IMU-Logging-Frequenz

# Einzelmotor-Test (nur wenn Firmware-Parameter vorhanden)
# bekannter Param-Name (bei manchen Builds): motorPowerSet.m1..m4  (0..65535)
MOTOR_PARAM_GROUP = "motorPowerSet"   # wird dynamisch geprüft
MOTOR_KEYS        = ["m1", "m2", "m3", "m4"]
MOTOR_TEST_THRUST = 12000
MOTOR_TEST_TIME_S = 1.2
MOTOR_RATE_HZ     = 400

# Timeout für Sicherheit
TEST_TIMEOUT_S = 30
# =======================================================

logging.basicConfig(level=logging.INFO)
logging.getLogger('cflib').setLevel(logging.WARNING)

# Global variable for emergency stop
_current_cf = None

def emergency_stop(cf=None):
    """Emergency stop all motors"""
    target_cf = cf or _current_cf
    if target_cf is None:
        return
    
    print("🛑 EMERGENCY STOP")
    try:
        # Stop via commander
        commander_setpoint(target_cf, 0, 0, 0, 0)
        # Also try to stop via motor parameters if available
        for motor in MOTOR_KEYS:
            set_motor_param(target_cf, f"{MOTOR_PARAM_GROUP}.{motor}", 0)
        target_cf.platform.send_arming_request(False)
        time.sleep(0.1)
    except Exception as e:
        print(f"  Warning during emergency stop: {e}")

def timeout_handler(signum, frame):
    """Handle test timeout"""
    print("\n⏰ TEST TIMEOUT - Activating emergency stop!")
    emergency_stop()
    raise Exception("Test timeout!")

def signal_handler(signum, frame):
    """Handle Ctrl-C and other signals"""
    print(f"\n🔴 Signal {signum} received - Emergency stop!")
    emergency_stop()
    exit(1)

def pct_from_vbat(v):
    try:
        v=float(v)
    except Exception:
        return None
    return max(0,min(100,(v-3.5)/(4.2-3.5)*100))

def has_log_var(cf, name)->bool:
    try:
        return cf.log.toctree.get_element(name) is not None
    except Exception:
        return False

def collect(cf, vars_:List[str], hz:int, sec:float)->Dict[str,List[float]]:
    present = [v for v in vars_ if has_log_var(cf, v)]
    if not present: return {}
    lc = LogConfig(name="cap", period_in_ms=max(5,int(1000/hz)))
    for v in present:
        t = "uint16_t" if v.startswith("range.") else "float"
        lc.add_variable(v,t)
    buf = {v:[] for v in present}
    def on(ts, data, _):
        for v in present:
            if v in data:
                buf[v].append(float(data[v]))
    cf.log.add_config(lc)
    lc.data_received_cb.add_callback(on)
    lc.start()
    try:
        time.sleep(sec)
    finally:
        lc.stop()
    return buf

def collect_imu(cf, hz:int, sec:float):
    buf = collect(cf, ["acc.x","acc.y","acc.z"], hz, sec)
    ax, ay, az = (buf.get("acc.x",[]), buf.get("acc.y",[]), buf.get("acc.z",[]))
    return ax, ay, az

def rms(xs:List[float])->float:
    if not xs: return float('nan')
    return math.sqrt(sum(x*x for x in xs)/len(xs))

def dominant_freq(xs:List[float], fs:int):
    if len(xs) < 8: return float('nan')
    x = np.array(xs) - np.mean(xs)
    spec = np.abs(np.fft.rfft(x))
    freqs = np.fft.rfftfreq(len(x), 1.0/fs)
    idx = np.argmax(spec[1:]) + 1  # ignoriere DC
    return float(freqs[idx])

def smart_kalman_reset(cf)->bool:
    for i in range(KALMAN_MAX_TRIES):
        print(f"[Kalman] Reset {i+1}/{KALMAN_MAX_TRIES} …")
        try:
            cf.param.set_value('kalman.resetEstimation','1')
            time.sleep(0.1)
            cf.param.set_value('kalman.resetEstimation','0')
        except Exception:
            print("  Param kalman.resetEstimation nicht verfügbar.")
            return False
        # kurze Ruhephase
        time.sleep(0.7)
        # Stabilität prüfen
        buf = collect(cf, ["stateEstimate.z"], KALMAN_RATE_HZ, KALMAN_SAMPLES_S)
        zs  = buf.get("stateEstimate.z", [])
        if not zs:
            continue
        mean_z = statistics.mean(zs)
        std_z  = statistics.pstdev(zs) if len(zs)>1 else 0.0
        print(f"  ẑ mean={mean_z:.2f} m, std={std_z:.2f} m")
        if abs(mean_z) < KALMAN_MEAN_OK and std_z < KALMAN_STD_OK:
            print("  ✅ Kalman stabil.")
            return True
    print("  ⚠️ Kalman nach mehreren Resets nicht stabil.")
    return False

def commander_setpoint(cf, roll=0.0, pitch=0.0, yawrate=0.0, thrust=0):
    # Low-level Kommandos (keine HL-Commander nötig)
    cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

def global_vibration_test(cf):
    print("\n[Vibration] Globaltest (alle Motoren, niedriger Thrust)")
    print("  -> motors ON (low thrust)")
    commander_setpoint(cf, 0,0,0, TEST_GLOBAL_THRUST)
    time.sleep(0.2)
    ax, ay, az = collect_imu(cf, TEST_GLOBAL_RATE_HZ, TEST_GLOBAL_TIME_S)
    commander_setpoint(cf, 0,0,0, 0)
    print("  -> motors OFF")

    # RMS & dominierende Frequenz
    total = [math.sqrt(x*x + y*y + z*z) for x,y,z in zip(ax,ay,az)]
    rms_total = rms(total)
    f_dom = dominant_freq(total, TEST_GLOBAL_RATE_HZ)
    print(f"  RMS(total) = {rms_total:.3f} g,  f_dom ≈ {f_dom:.1f} Hz")

    # grobe Heuristik
    if rms_total > 0.12:
        print("  ⚠️ Hohe Vibrationen. Prüfe Propeller (Beschädigung, falsche Ausrichtung) & Motorlager.")
    elif rms_total > 0.06:
        print("  ⚠️ Mäßige Vibrationen. Leichte Unwucht wahrscheinlich.")
    else:
        print("  ✅ Vibrationen im grünen Bereich.")
    return rms_total, f_dom

def have_motor_params(cf):
    # Suche nach Parametern wie motorPowerSet.m1..m4 (0..65535)
    have = []
    for k in MOTOR_KEYS:
        name = f"{MOTOR_PARAM_GROUP}.{k}"
        try:
            cf.param.get_value(name)  # nur test
            have.append(name)
        except Exception:
            pass
    return have

def set_motor_param(cf, name, value):
    try:
        cf.param.set_value(name, str(int(value)))
        return True
    except Exception:
        return False

def single_motor_test(cf):
    names = have_motor_params(cf)
    if len(names) != 4:
        print("\n[Motor] Einzelmotor-Test wird übersprungen (Parameter nicht verfügbar).")
        return None
    print("\n[Motor] Einzelmotor-Test (niedriger Thrust, nacheinander m1..m4)")
    results = {}
    # Sicherheits-armed
    cf.platform.send_arming_request(True)
    time.sleep(0.2)
    for i, name in enumerate(names, start=1):
        print(f"  -> {name} ON")
        set_motor_param(cf, name, MOTOR_TEST_THRUST)
        ax, ay, az = collect_imu(cf, MOTOR_RATE_HZ, MOTOR_TEST_TIME_S)
        set_motor_param(cf, name, 0)
        print(f"  -> {name} OFF")
        total = [math.sqrt(x*x + y*y + z*z) for x,y,z in zip(ax,ay,az)]
        r = rms(total)
        f = dominant_freq(total, MOTOR_RATE_HZ)
        results[name] = (r, f)
        print(f"     RMS={r:.3f} g, f_dom≈{f:.1f} Hz")
    # disarm (optional – die Werte stehen auf 0)
    cf.platform.send_arming_request(False)
    # Bewertung relativ (Vergleich der vier)
    if results:
        vals = [r for (r, f) in results.values()]
        med  = statistics.median(vals)
        for name,(r,_) in results.items():
            if r > med * 1.6:
                print(f"  ⚠️ {name}: deutlich höhere Vibration als die anderen -> Kandidat für Unwucht/Schaden.")
    return results

def check_sensors(cf):
    """Check basic sensor functionality"""
    print("\n[Sensors] Basic sensor check...")
    data = collect(cf, ["gyro.x", "gyro.y", "gyro.z", "baro.pressure"], 10, 0.5)
    # Check if sensors are responding with reasonable values
    for sensor, values in data.items():
        if values:
            print(f"  {sensor}: {len(values)} samples, range: {min(values):.3f} to {max(values):.3f}")
            
    # Additional IMU sanity check
    ax, ay, az = collect_imu(cf, 50, 0.3)
    if ax and ay and az:
        # Check if accelerometer shows reasonable values (not stuck)
        ax_range = max(ax) - min(ax)
        ay_range = max(ay) - min(ay)
        az_range = max(az) - min(az)
        
        print(f"  Accelerometer ranges - X: {ax_range:.3f}g, Y: {ay_range:.3f}g, Z: {az_range:.3f}g")
        
        # If all ranges are very small, sensors might be stuck
        if max(ax_range, ay_range, az_range) < 0.01:
            print("  ⚠️ Very small accelerometer ranges - sensors might be stuck!")
        else:
            print("  ✅ Accelerometers show normal activity")
    
    return data

def check_temperature(cf):
    """Check motor and board temperatures"""
    print("\n[Temperature] Board temperature check...")
    try:
        temp = float(cf.param.get_value("pm.temperature"))
        print(f"  Board temperature: {temp:.1f}°C")
        if temp > 60:
            print("  ⚠️ High temperature warning!")
        elif temp > 45:
            print("  ⚠️ Moderate temperature - monitor during operation")
        else:
            print("  ✅ Temperature normal")
        return temp
    except Exception as e:
        print(f"  Temperature: n/a ({e})")
        return None

def check_range_sensors(cf):
    """Check if range sensors are working"""
    print("\n[Range] Sensor check...")
    range_vars = ["range.front", "range.back", "range.left", "range.right", "range.up", "range.zrange"]
    available = [v for v in range_vars if has_log_var(cf, v)]
    
    if not available:
        print("  No range sensors found in log variables")
        return {}
        
    print(f"  Found {len(available)} range sensors: {', '.join(available)}")
    data = collect(cf, available, 10, 0.5)
    
    sensor_status = {}
    for sensor, values in data.items():
        if values:
            avg = statistics.mean(values)
            std = statistics.pstdev(values) if len(values) > 1 else 0
            sensor_status[sensor] = (avg, std)
            
            # Evaluate sensor reading
            if avg < 0.01 and std < 0.001:
                status = "❓ (possibly stuck or out of range)"
            elif avg > 4.0:
                status = "❓ (possibly out of range)"
            else:
                status = "✅"
                
            print(f"  {sensor}: {avg:.3f} ± {std:.3f} m {status}")
        else:
            print(f"  {sensor}: no data")
            sensor_status[sensor] = (float('nan'), float('nan'))
    
    return sensor_status

def safety_prompt():
    """Prompt user for safety confirmation"""
    print("\n" + "="*60)
    print("🔒 SAFETY CHECK")
    print("="*60)
    print("BEFORE CONTINUING:")
    print("✓ Place Crazyflie on a CLEAR, FLAT surface")
    print("✓ Ensure NO OBSTACLES within 2 meters")
    print("✓ Remove ALL LOOSE OBJECTS from propellers")
    print("✓ Keep HANDS and FACE clear of propellers")
    print("✓ Be ready to use EMERGENCY STOP (Ctrl+C)")
    print("="*60)
    
    response = input("Type 'YES' to continue: ")
    if response.upper() != 'YES':
        print("Aborted by user.")
        exit(0)
    print("Safety confirmed. Starting tests...\n")

def main():
    global _current_cf
    
    ap = argparse.ArgumentParser(description="Crazyflie Health+, Smart Kalman Reset & Vibration Check")
    ap.add_argument("--uri", default=DEF_URI)
    ap.add_argument("--no-global-vibe", action="store_true", help="globalen Vibrationstest überspringen")
    ap.add_argument("--no-single-motor", action="store_true", help="Einzelmotor-Test überspringen")
    ap.add_argument("--thrust", type=int, default=TEST_GLOBAL_THRUST, help="globaler Test-Thrust (10000..20000)")
    ap.add_argument("--no-safety-prompt", action="store_true", help="Sicherheitsabfrage überspringen")
    ap.add_argument("--check-sensors", action="store_true", help="Grundlegende Sensorprüfung durchführen")
    ap.add_argument("--check-temperature", action="store_true", help="Temperaturprüfung durchführen")
    ap.add_argument("--check-range", action="store_true", help="Range-Sensor-Prüfung durchführen")
    ap.add_argument("--full-check", action="store_true", help="Alle verfügbaren Checks durchführen")
    args = ap.parse_args()

    # Setup signal handlers for safety
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Safety prompt
    if not args.no_safety_prompt:
        safety_prompt()

    init_drivers()
    print(f"Connecting to {args.uri} …")
    
    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie()) as scf:
            cf = scf.cf
            _current_cf = cf
            
            # Setup timeout for tests
            signal.signal(signal.SIGALRM, timeout_handler)
            signal.alarm(TEST_TIMEOUT_S)

            # Batterie
            try:
                vbat = float(cf.param.get_value("pm.vbat"))
                pct  = pct_from_vbat(vbat)
                print(f"Battery: {vbat:.2f} V (~{pct:.0f}%)" if pct is not None else f"Battery: {vbat:.2f} V")
            except Exception:
                print("Battery: n/a")

            # Estimator = Kalman
            try:
                if cf.param.get_value('stabilizer.estimator') != '2':
                    print("Setting stabilizer.estimator=2 (Kalman)")
                    cf.param.set_value('stabilizer.estimator', '2')
                    time.sleep(0.2)
            except Exception:
                pass

            # Smart Kalman Reset (mehrfach bis stabil)
            ok = smart_kalman_reset(cf)
            if not ok:
                print("Hinweis: Stelle die CF absolut ruhig auf texturierten Boden und versuche es erneut.")

            # Additional health checks
            if args.check_sensors or args.full_check:
                check_sensors(cf)
                
            if args.check_temperature or args.full_check:
                check_temperature(cf)
                
            if args.check_range or args.full_check:
                check_range_sensors(cf)

            # Globaler Vibrations-Test
            if not args.no_global_vibe:
                global TEST_GLOBAL_THRUST
                TEST_GLOBAL_THRUST = max(10000, min(20000, int(args.thrust)))
                print(f"\nGlobaler Vibrations-Test mit Thrust={TEST_GLOBAL_THRUST}")
                # arming anfordern
                cf.platform.send_arming_request(True)
                time.sleep(0.2)
                try:
                    global_vibration_test(cf)
                finally:
                    commander_setpoint(cf, 0,0,0, 0)
                    cf.platform.send_arming_request(False)

            # Einzelmotor-Test (nur wenn Param-Gruppe vorhanden)
            if not args.no_single_motor:
                single_motor_test(cf)

            # Disable timeout
            signal.alarm(0)
            _current_cf = None
            
            print("\n" + "="*50)
            print("✅ ALL TESTS COMPLETED SUCCESSFULLY")
            print("="*50)

    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        emergency_stop()
        raise

if __name__ == "__main__":
    main()
