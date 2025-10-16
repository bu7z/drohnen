#!/usr/bin/env python3
# cf_health_check.py
# pip install cflib
import time
import logging
from statistics import median

from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

URI = 'radio://0/80/2M/1337691337' # anpassen
DO_RESET_ESTIMATOR = True              # True = Kalman-Reset einmal ausführen
LOG_PERIOD_MS = 100

logging.basicConfig(level=logging.INFO)
logging.getLogger('cflib').setLevel(logging.WARNING)

RANGE_VARS = [
    'range.front', 'range.back', 'range.left',
    'range.right', 'range.up', 'range.zrange'
]
STATE_VARS = ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
              'stateEstimate.yaw']

def get_param(cf, name, default=None):
    try:
        return cf.param.get_value(name)
    except Exception:
        return default

def has_log_var(cf, fullname):
    try:
        return cf.log.toctree.get_element(fullname) is not None
    except Exception:
        return False

def read_once(cf, variables, period_ms=LOG_PERIOD_MS, samples=10, dtype_hint='float'):
    """Sammelt kurz Logdaten und gibt Median je Variable zurück."""
    present = [v for v in variables if has_log_var(cf, v)]
    if not present:
        return {}, present

    lc = LogConfig(name='tmp', period_in_ms=period_ms)
    for v in present:
        # range.* sind uint16_t (mm), stateEstimate.* sind float
        t = 'uint16_t' if v.startswith('range.') else 'float'
        lc.add_variable(v, t)

    buf = {v: [] for v in present}

    def on_data(ts, data, _):
        for v in present:
            if v in data:
                buf[v].append(data[v])

    cf.log.add_config(lc)
    lc.data_received_cb.add_callback(on_data)
    lc.start()
    try:
        time.sleep(samples * period_ms / 1000.0 + 0.05)
    finally:
        lc.stop()

    med = {}
    for v in present:
        if buf[v]:
            # convert mm->m for range.*
            if v.startswith('range.'):
                med[v] = median(buf[v]) / 1000.0
            else:
                med[v] = float(median(buf[v]))
    return med, present

def battery_percent(vbat):
    # Sehr grobe Linearisierung für 1S LiPo: 3.5V leer, 4.2V voll
    try:
        v = float(vbat)
    except Exception:
        return None
    pct = (v - 3.5) / (4.2 - 3.5) * 100.0
    return max(0, min(100, pct))

def main():
    init_drivers()
    print(f'Connecting to {URI} ...')
    with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
        cf = scf.cf

        # --- Batterie ---
        vbat = get_param(cf, 'pm.vbat')
        pct = battery_percent(vbat) if vbat is not None else None
        if pct is not None:
            print(f'Battery: {float(vbat):.2f} V (~{pct:.0f}%)')
        else:
            print('Battery: n/a')

        # --- Decks / Params ---
        deck = {
            'bcFlow': get_param(cf, 'deck.bcFlow'),
            'bcFlow2': get_param(cf, 'deck.bcFlow2'),
            'bcZRanger': get_param(cf, 'deck.bcZRanger'),
            'bcMultiranger': get_param(cf, 'deck.bcMultiranger'),
        }
        print('Decks:', deck)

        # --- Estimator setzen ---
        est = get_param(cf, 'stabilizer.estimator')
        if est != '2':
            print('Setting stabilizer.estimator=2 (Kalman)')
            cf.param.set_value('stabilizer.estimator', '2')
            time.sleep(0.2)

        # --- Optional: Kalman-Reset ---
        if DO_RESET_ESTIMATOR:
            try:
                print('Resetting Kalman estimator …')
                cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                cf.param.set_value('kalman.resetEstimation', '0')
                time.sleep(0.6)  # kurz ruhen lassen
            except Exception:
                print('Kalman reset failed (param missing?).')

        # --- range.* vorhanden? ---
        present_ranges = [v for v in RANGE_VARS if has_log_var(cf, v)]
        print('range.* present:', present_ranges if present_ranges else 'none')

        # --- stateEstimate.* vorhanden? ---
        present_state = [v for v in STATE_VARS if has_log_var(cf, v)]
        print('stateEstimate.* present:', present_state if present_state else 'none')

        # --- Kurz messen & bewerten ---
        print('\nSampling sensors …')
        med_ranges, pr = read_once(cf, RANGE_VARS, samples=12)
        med_state, ps = read_once(cf, STATE_VARS, samples=12)

        # Ausgabe
        if med_ranges:
            def fmt(x):
                return 'n/a' if x is None else f'{x:.2f} m'
            print('Ranges (median):',
                  'F:', fmt(med_ranges.get('range.front')),
                  'B:', fmt(med_ranges.get('range.back')),
                  'L:', fmt(med_ranges.get('range.left')),
                  'R:', fmt(med_ranges.get('range.right')),
                  'U:', fmt(med_ranges.get('range.up')),
                  'D:', fmt(med_ranges.get('range.zrange')))
        else:
            print('No range data (check Multiranger/ZRanger & firmware).')

        if med_state:
            print('State (median):',
                  f"X:{med_state.get('stateEstimate.x', float('nan')):.2f} m",
                  f"Y:{med_state.get('stateEstimate.y', float('nan')):.2f} m",
                  f"Z:{med_state.get('stateEstimate.z', float('nan')):.2f} m",
                  f"Yaw:{med_state.get('stateEstimate.yaw', float('nan')):.1f}°")
        else:
            print('No stateEstimate.* available in log TOC.')

        # --- Heuristische Health-Checks ---
        issues = []

        # Decks
        if deck.get('bcFlow2') != '1':
            issues.append('Flow v2 deck not detected (bcFlow2!=1).')
        if deck.get('bcMultiranger') != '1':
            issues.append('Multiranger deck not detected (bcMultiranger!=1).')

        # Ranges plausibel?
        for k in ['range.front','range.back','range.left','range.right','range.up']:
            v = med_ranges.get(k)
            if v is not None and v >= 32.6:
                issues.append(f'{k} saturated (~32.77 m): no return (sensor sees nothing).')

        d = med_ranges.get('range.zrange')
        if d is not None and (d < 0.01 or d > 5.0):
            issues.append('range.zrange implausible; check surface/height/orientation of Flow/ZRanger.')

        # Z-Estimate plausibel?
        z = med_state.get('stateEstimate.z') if med_state else None
        if z is not None and abs(z) > 10:
            issues.append('stateEstimate.z very large; try reset estimator with CF stationary.')

        # Ergebnis
        if issues:
            print('\nHealth report: ⚠️')
            for it in issues:
                print(' -', it)
            print('\nSuggested actions:')
            print(' - Place CF stationary on a textured surface, run again.')
            print(' - Ensure Flow v2 (bottom) & Multiranger (top) are firmly stacked (no OneWire timeouts).')
            print(' - If ranges saturate: remove obstructions/protective films, check orientation.')
        else:
            print('\nHealth report: ✅ Looks good.')

if __name__ == '__main__':
    main()
