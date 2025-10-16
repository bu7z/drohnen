#!/usr/bin/env python3
# cf_health_plus.py
# pip install cflib numpy
import time, math, statistics, logging, argparse, signal, sys
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
KALMAN_STD_OK    = 0.05   # m  -> ẑ muss ruhig sein
KALMAN_MEAN_OK   = 0.30   # m  -> |ẑ| nahe 0
KALMAN_SAMPLES_S = 1.0
KALMAN_RATE_HZ   = 15

# Vibrations-Test (global)
TEST_GLOBAL_THRUST = 14000   # 10000..20000 -> Boden bleiben
TEST_GLOBAL_TIME_S = 1.5
TEST_GLOBAL_RATE_HZ= 200     # Hz

# Einzelmotor-Test (nur wenn Param-Gruppe vorhanden)
MOTOR_PARAM_GROUP = "motorPowerSet"   # wird dynamisch geprüft
MOTOR_KEYS        = ["m1", "m2", "m3", "m4"]
MOTOR_TEST_THRUST = 12000
MOTOR_TEST_TIME_S = 1.2
MOTOR_RATE_HZ     = 400

# Timeout
TEST_TIMEOUT_S = 30
G0 = 9.80665  # m/s^2 -> g

# Vibrations-Bewertung (RMS in g):
#   <= 0.06 g: ok   | 0.06–0.12 g: moderat   | > 0.12 g: hoch
VIBRATION_LOW_THRESHOLD = 0.06
VIBRATION_HIGH_THRESHOLD = 0.12
# =======================================================

logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(levelname)s - %(message)s',
                   datefmt='%H:%M:%S')
logging.getLogger('cflib').setLevel(logging.WARNING)

_current_cf = None  # für Emergency-Stop

# ---------- Utility ----------
def pct_from_vbat(v: float) -> Optional[float]:
    """Convert battery voltage to percentage."""
    try: 
        v = float(v)
        return max(0, min(100, (v - 3.5) / (4.2 - 3.5) * 100.0))
    except (ValueError, TypeError): 
        return None

def has_log_var(cf, name: str) -> bool:
    """Check if log variable exists."""
    try:
        return cf.log.toctree.get_element(name) is not None
    except Exception:
        return False

def collect(cf, vars_: List[str], hz: int, sec: float) -> Dict[str, List[float]]:
    """Sammelt Logdaten. range.* wird mm->m konvertiert."""
    present = [v for v in vars_ if has_log_var(cf, v)]
    if not present: 
        return {}
    
    # Eindeutiger LogConfig-Name vermeidet Kollisionen
    lc = LogConfig(name=f"cap_{int(time.time()*1000)}", period_in_ms=max(5, int(1000/hz)))
    for v in present:
        t = "uint16_t" if v.startswith("range.") else "float"
        lc.add_variable(v, t)
    
    buf = {v: [] for v in present}
    
    def on(ts, data, _):
        for v in present:
            if v in data:
                val = float(data[v])
                if v.startswith("range."):   # mm -> m
                    val = val / 1000.0
                buf[v].append(val)
    
    cf.log.add_config(lc)
    lc.data_received_cb.add_callback(on)
    lc.start()
    try:
        time.sleep(sec)
    finally:
        lc.stop()
        # LogConfig sauber aufräumen (Memory/Handles)
        try:
            cf.log.delete_config(lc)   # Neuere cflib-Versionen
        except AttributeError:
            try:
                cf.log.remove_config(lc)  # Ältere cflib-Versionen  
            except Exception:
                pass  # Fallback falls keine der Methoden existiert
        except Exception:
            pass  # Silent fallback
    
    return buf

def collect_imu(cf, hz: int, sec: float) -> Tuple[List[float], List[float], List[float]]:
    """IMU sammeln und auf g umrechnen."""
    buf = collect(cf, ["acc.x", "acc.y", "acc.z"], hz, sec)
    ax = [x / G0 for x in buf.get("acc.x", [])]
    ay = [x / G0 for x in buf.get("acc.y", [])]
    az = [x / G0 for x in buf.get("acc.z", [])]
    return ax, ay, az

def rms(xs: List[float]) -> float:
    """Calculate RMS of signal."""
    if not xs: 
        return float('nan')
    return math.sqrt(sum(x * x for x in xs) / len(xs))

def dominant_freq(xs: List[float], fs: int) -> float:
    """Find dominant frequency using FFT with Hanning window for reduced leakage."""
    if len(xs) < 8: 
        return float('nan')
    
    x = np.array(xs) - np.mean(xs)
    w = np.hanning(len(x))          # Hanning-Fenster für reduzierte Spektralleckagen
    spec = np.abs(np.fft.rfft(x * w))
    freqs = np.fft.rfftfreq(len(x), 1.0 / fs)
    idx = np.argmax(spec[1:]) + 1   # DC ignorieren
    return float(freqs[idx])

def have_motor_params(cf) -> List[str]:
    """Check which motor parameters are available."""
    have = []
    for k in MOTOR_KEYS:
        name = f"{MOTOR_PARAM_GROUP}.{k}"
        try:
            cf.param.get_value(name)
            have.append(name)
        except Exception:
            pass
    return have

def set_motor_param(cf, name: str, value: int) -> bool:
    """Set motor parameter value."""
    try:
        cf.param.set_value(name, str(int(value)))
        return True
    except Exception:
        return False

def commander_setpoint(cf, roll: float = 0.0, pitch: float = 0.0, 
                       yawrate: float = 0.0, thrust: int = 0):
    """Send low-level setpoint to commander."""
    cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

def is_on_ground(cf, max_d: float = 0.06) -> bool:
    """Check if Crazyflie is clearly on ground using range sensor."""
    data = collect(cf, ["range.zrange"], 20, 0.2).get("range.zrange", [])
    if not data:
        print("  ❌ Ground check: no zrange data -> cannot verify ground safety")
        return False  # Sicherheitsorientiert: ohne Daten nicht fortfahren
    median_dist = statistics.median(data)
    print(f"  Ground check: zrange median = {median_dist:.3f} m")
    return median_dist < max_d

# ---------- Safety / Signals ----------
def emergency_stop(cf = None):
    """Alles stoppen (Commander + Motor-Params + Disarm)."""
    target_cf = cf or _current_cf
    if target_cf is None:
        return
    
    print("🛑 EMERGENCY STOP")
    try:
        # Multiple stop methods for safety - send multiple stop pulses
        for _ in range(3):
            try: 
                target_cf.commander.send_setpoint(0, 0, 0, 0)
            except Exception:
                pass
            time.sleep(0.02)
            
        try: 
            target_cf.commander.send_stop_setpoint()
        except Exception: 
            pass
        
        # Stop all motors via parameters
        for name in have_motor_params(target_cf):
            set_motor_param(target_cf, name, 0)
        
        target_cf.platform.send_arming_request(False)
        time.sleep(0.1)
    except Exception as e:
        print(f"  Warning during emergency stop: {e}")

def timeout_handler(signum, frame):
    """Handle test timeout."""
    print("\n⏰ TEST TIMEOUT - Activating emergency stop!")
    emergency_stop()
    raise Exception("Test timeout exceeded!")

def signal_handler(signum, frame):
    """Handle Ctrl-C and other signals."""
    print(f"\n🔴 Signal {signum} received - Emergency stop!")
    emergency_stop()
    sys.exit(1)

# ---------- Health Checks ----------
def smart_kalman_reset(cf) -> bool:
    """Reset wiederholen bis ẑ stabil und zrange plausibel."""
    print("\n[Kalman] Starting smart reset procedure...")
    
    for i in range(KALMAN_MAX_TRIES):
        print(f"  Attempt {i+1}/{KALMAN_MAX_TRIES}...")
        try:
            cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            cf.param.set_value('kalman.resetEstimation', '0')
        except Exception:
            print("  ❌ Param kalman.resetEstimation nicht verfügbar.")
            return False
        
        time.sleep(0.8)  # Einpendeln

        # Collect state estimate and range data
        buf = collect(cf, ["stateEstimate.z", "range.zrange"], KALMAN_RATE_HZ, KALMAN_SAMPLES_S)
        zs = buf.get("stateEstimate.z", [])
        dz = buf.get("range.zrange", [])
        
        if not zs:
            print("  ⚠️ No stateEstimate.z data received")
            continue
            
        mean_z = statistics.mean(zs)
        std_z = statistics.pstdev(zs) if len(zs) > 1 else 0.0
        
        # Check if range data is plausible
        zr_median = statistics.median(dz) if dz else float('nan')
        zr_ok = (dz and 0.02 <= zr_median <= 2.5)  # plausibel
        
        print(f"  ẑ: mean={mean_z:.3f} m, std={std_z:.3f} m")
        print(f"  zrange: median={zr_median:.3f} m, plausible={zr_ok}")
        
        if abs(mean_z) < KALMAN_MEAN_OK and std_z < KALMAN_STD_OK and zr_ok:
            print("  ✅ Kalman stabil.")
            return True
            
    print("  ❌ Kalman nach mehreren Resets nicht stabil.")
    return False

def global_vibration_test(cf, thrust: int) -> Tuple[float, float]:
    """Global vibration test with all motors - with commander keepalive."""
    print(f"\n[Vibration] Global test (thrust={thrust})")
    
    # Ground check - sicherheitsorientiert
    if not is_on_ground(cf):
        print("  ❌ Not clearly on ground - aborting vibration test!")
        return float('nan'), float('nan')
    
    # Arm and start motors
    cf.platform.send_arming_request(True)
    time.sleep(0.15)
    
    ax_all, ay_all, az_all = [], [], []
    
    try:
        print("  Starting motors with keepalive...")
        t_end = time.time() + TEST_GLOBAL_TIME_S
        period = 0.05  # 20 Hz keepalive für Commander
        
        while time.time() < t_end:
            # Send setpoint periodically to keep motors running
            commander_setpoint(cf, 0, 0, 0, thrust)
            
            # Collect IMU data in small chunks (collect_imu sleeps for 'period')
            ax, ay, az = collect_imu(cf, TEST_GLOBAL_RATE_HZ, period)
            ax_all.extend(ax)
            ay_all.extend(ay) 
            az_all.extend(az)
            
            # Kein zusätzliches Sleep - collect_imu hat bereits geblockt
        
        # Final stop
        commander_setpoint(cf, 0, 0, 0, 0)
        time.sleep(0.1)
        
    finally:
        cf.platform.send_arming_request(False)

    # Analyze vibrations
    if not ax_all:
        print("  ❌ No IMU data collected")
        return float('nan'), float('nan')
        
    total = [math.sqrt(x*x + y*y + z*z) for x, y, z in zip(ax_all, ay_all, az_all)]
    rms_total = rms(total)
    f_dom = dominant_freq(total, TEST_GLOBAL_RATE_HZ)
    
    print(f"  Results: RMS={rms_total:.3f} g, dominant_freq={f_dom:.1f} Hz")
    print(f"  Collected {len(total)} samples")

    # Vibration assessment mit dokumentierten Schwellwerten
    if rms_total > VIBRATION_HIGH_THRESHOLD:
        print("  ⚠️ High vibrations! Check propellers (damage, orientation) and motor bearings.")
    elif rms_total > VIBRATION_LOW_THRESHOLD:
        print("  ⚠️ Moderate vibrations. Slight imbalance likely.")
    else:
        print("  ✅ Vibrations in acceptable range.")
        
    return rms_total, f_dom

def single_motor_test(cf) -> Optional[Dict]:
    """Test each motor individually."""
    names = have_motor_params(cf)
    if len(names) != 4:
        print("\n[Motor] Single motor test skipped (parameters not available).")
        return None
        
    print("\n[Motor] Single motor test (low thrust, one by one)")
    
    # Ground check - sicherheitsorientiert
    if not is_on_ground(cf):
        print("  ❌ Not clearly on ground - aborting motor test!")
        return None
    
    results = {}
    
    cf.platform.send_arming_request(True)
    time.sleep(0.2)
    
    try:
        # Ensure no other controller is active
        commander_setpoint(cf, 0, 0, 0, 0)
        time.sleep(0.1)
        
        for name in names:
            print(f"  Testing {name}...")
            set_motor_param(cf, name, MOTOR_TEST_THRUST)
            
            # Collect vibration data
            ax, ay, az = collect_imu(cf, MOTOR_RATE_HZ, MOTOR_TEST_TIME_S)
            
            set_motor_param(cf, name, 0)
            time.sleep(0.2)  # Auslauf-Puffer für weniger Cross-Talk
            
            # Analyze results
            total = [math.sqrt(x*x + y*y + z*z) for x, y, z in zip(ax, ay, az)]
            r = rms(total)
            f = dominant_freq(total, MOTOR_RATE_HZ)
            results[name] = (r, f)
            
            print(f"    {name}: RMS={r:.3f} g, f_dom={f:.1f} Hz")
            
    finally:
        # Safety stop
        for name in names:
            set_motor_param(cf, name, 0)
        commander_setpoint(cf, 0, 0, 0, 0)
        cf.platform.send_arming_request(False)

    # Compare motor vibrations
    if results:
        vals = [r for (r, f) in results.values()]
        med = statistics.median(vals)
        
        print("\n  Motor comparison:")
        for name, (r, f) in results.items():
            ratio = r / med if med > 0 else float('inf')
            if ratio > 1.6:
                print(f"    ⚠️ {name}: {ratio:.1f}x higher vibration - check for imbalance/damage")
            else:
                print(f"    ✅ {name}: normal")
                
    return results

def check_sensors(cf) -> Dict:
    """Basic sensor functionality check."""
    print("\n[Sensors] Basic sensor check...")
    
    # Check gyro and baro
    data = collect(cf, ["gyro.x", "gyro.y", "gyro.z", "baro.pressure"], 10, 0.5)
    
    for sensor, values in data.items():
        if values:
            unit = "°/s" if "gyro" in sensor else "Pa"
            range_val = max(values) - min(values)
            print(f"  {sensor}: {len(values)} samples, Δ={range_val:.3f} {unit}")

    # Check accelerometer activity
    ax, ay, az = collect_imu(cf, 50, 0.3)
    if ax and ay and az:
        axr, ayr, azr = max(ax) - min(ax), max(ay) - min(ay), max(az) - min(az)
        print(f"  Accelerometer ranges [g]: X:{axr:.3f}, Y:{ayr:.3f}, Z:{azr:.3f}")
        
        if max(axr, ayr, azr) < 0.01:
            print("  ⚠️ Very small ranges - sensors might be stuck!")
        else:
            print("  ✅ Accelerometers show normal activity")
            
    return data

def check_temperature(cf) -> Optional[float]:
    """Check board temperature."""
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

def check_range_sensors(cf) -> Dict:
    """Check range sensor functionality."""
    print("\n[Range] Sensor check...")
    
    range_vars = ["range.front", "range.back", "range.left", "range.right", "range.up", "range.zrange"]
    available = [v for v in range_vars if has_log_var(cf, v)]
    
    if not available:
        print("  No range sensors found")
        return {}
        
    print(f"  Found {len(available)} sensors: {', '.join(available)}")
    data = collect(cf, available, 10, 0.5)
    
    for sensor, values in data.items():
        if values:
            avg = statistics.mean(values)
            std = statistics.pstdev(values) if len(values) > 1 else 0
            min_val = min(values)
            max_val = max(values)
            
            # Evaluate sensor status
            status = "✅ OK"
            if avg < 0.01 and std < 0.001: 
                status = "❓ Possibly stuck/out of range"
            elif avg > 4.0:                
                status = "❓ Possibly out of range"
            elif 32.5 < avg < 33.0 and std < 0.05:
                status = "❓ Saturated / no return"
            elif std < 0.002 and 0.05 < avg < 0.15:
                status = "❓ Near-constant -> check FOV/obstruction"
                
            if std > 0.2 and avg < 3.0:    
                status += " (unstable)"
                
            print(f"  {sensor}: {avg:.3f} ± {std:.3f} m [{min_val:.3f}..{max_val:.3f}] - {status}")
        else:
            print(f"  {sensor}: no data")
            
    return data

def safety_prompt():
    """Safety confirmation prompt."""
    print("\n" + "="*60)
    print("🔒 SAFETY CHECK")
    print("="*60)
    print("BEFORE CONTINUING:")
    print("✓ Place on flat, clear surface")
    print("✓ No obstacles within 2m")
    print("✓ Remove loose objects from propellers")  
    print("✓ Keep hands/face clear of props")
    print("✓ Be ready to use Ctrl+C (Emergency Stop)")
    print("="*60)
    
    response = input("Type 'YES' to continue: ").strip().upper()
    if response != 'YES':
        print("Aborted by user.")
        sys.exit(0)
        
    print("Safety confirmed. Starting tests...\n")

# ---------- Main ----------
def main():
    global _current_cf

    ap = argparse.ArgumentParser(
        description="Crazyflie Health+ - Comprehensive diagnostics and vibration analysis"
    )
    ap.add_argument("--uri", default=DEF_URI, help="Crazyflie radio URI")
    ap.add_argument("--no-global-vibe", action="store_true", help="Skip global vibration test")
    ap.add_argument("--no-single-motor", action="store_true", help="Skip single motor test")
    ap.add_argument("--thrust", type=int, default=TEST_GLOBAL_THRUST, 
                   help=f"Global test thrust ({10000}..{20000})")
    ap.add_argument("--no-safety-prompt", action="store_true", help="Skip safety prompt")
    ap.add_argument("--check-sensors", action="store_true", help="Basic sensor check")
    ap.add_argument("--check-temperature", action="store_true", help="Temperature check")  
    ap.add_argument("--check-range", action="store_true", help="Range sensor check")
    ap.add_argument("--full-check", action="store_true", help="Run all available checks")
    args = ap.parse_args()

    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    supports_alarm = hasattr(signal, "SIGALRM")
    if supports_alarm:
        signal.signal(signal.SIGALRM, timeout_handler)

    # Safety first
    if not args.no_safety_prompt:
        safety_prompt()

    init_drivers()
    print(f"Connecting to {args.uri}...")

    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie()) as scf:
            cf = scf.cf
            _current_cf = cf

            # Set timeout
            if supports_alarm:
                signal.alarm(TEST_TIMEOUT_S)

            # Battery status
            try:
                vbat = float(cf.param.get_value("pm.vbat"))
                pct = pct_from_vbat(vbat)
                status = f"Battery: {vbat:.2f} V"
                if pct is not None:
                    status += f" (~{pct:.0f}%)"
                print(status)
            except Exception:
                print("Battery: n/a")

            # Ensure Kalman estimator mit stabilerer Wartezeit
            try:
                if cf.param.get_value('stabilizer.estimator') != '2':
                    print("Setting stabilizer.estimator=2 (Kalman)")
                    cf.param.set_value('stabilizer.estimator', '2')
                    time.sleep(0.3)  # Etwas mehr Puffer für stabileren Übergang
            except Exception:
                print("⚠️ Could not set estimator")

            # Smart Kalman Reset
            ok = smart_kalman_reset(cf)
            if not ok:
                print("💡 Tip: Place CF on textured surface and retry")

            # Additional health checks
            if args.check_sensors or args.full_check:
                check_sensors(cf)
                
            if args.check_temperature or args.full_check:
                check_temperature(cf)
                
            if args.check_range or args.full_check:
                check_range_sensors(cf)

            # Vibration tests
            if not args.no_global_vibe:
                thrust = max(10000, min(20000, args.thrust))
                print(f"\nStarting global vibration test (thrust={thrust})")
                global_vibration_test(cf, thrust)

            if not args.no_single_motor:
                single_motor_test(cf)

            # Final safety: Mehrfach Stop-Pulses am Ende
            print("\nFinal safety cleanup...")
            for _ in range(2):
                try: 
                    cf.commander.send_setpoint(0, 0, 0, 0)
                except Exception:
                    pass
                time.sleep(0.02)

            # Cleanup
            if supports_alarm:
                signal.alarm(0)
            _current_cf = None

            print("\n" + "="*50)
            print("✅ ALL TESTS COMPLETED SUCCESSFULLY")
            print("="*50)

    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        emergency_stop()
        raise

if __name__ == "__main__":
    main()
