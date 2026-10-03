"""Spin each motor ON ITS OWN and measure how much the airframe shakes.

WHY NOT THE BUILT-IN TEST. health.startPropTest stopped after two motors on
both attempts, and both of those runs happened on boots where the decks had
failed, so even the numbers it did give are suspect. This drives the motors
directly through motorPowerSet, one at a time, at an identical PWM, and reads
the gyro itself. Four comparable numbers or none.

WHAT IT MEANS. A motor turning a true propeller shakes the frame a little. One
turning a bent, chipped or loose propeller shakes it a lot, and one that is
failing shakes it differently again. The absolute numbers mean nothing; the
comparison between the four is the whole point.

THE PROPELLERS SPIN. Flat surface, hands clear. The drone does NOT take off --
the PWM is well under hover thrust and only one motor runs at a time.

It always switches the motors off again, including if it fails partway.
"""
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = 'radio://0/80/2M'
PWM = 18000          # of 65535. Spins freely, nowhere near lifting.
SPIN_S = 2.5         # per motor
SETTLE_S = 1.0       # between motors, so one does not colour the next


def connect(tries=5):
    """The link has been intermittent all day. Keep trying before giving up."""
    cflib.crtp.init_drivers(enable_debug_driver=False)
    for i in range(tries):
        if not cflib.crtp.scan_interfaces():
            print('  attempt %d: nothing answered a scan' % (i + 1))
            time.sleep(2.0)
            continue
        try:
            scf = SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache'))
            scf.open_link()
            return scf
        except Exception as e:
            print('  attempt %d: %s' % (i + 1, e))
            time.sleep(2.0)
    return None


print('connecting...')
scf = connect()
if scf is None:
    print()
    print('COULD NOT CONNECT. The drone is off, hung, or the app has the link.')
    print('Power cycle it and run this again.')
    raise SystemExit(1)

cf = scf.cf
samples = {'x': [], 'y': [], 'z': []}
collecting = False


def on_data(ts, data, conf):
    if collecting:
        samples['x'].append(data['gyro.x'])
        samples['y'].append(data['gyro.y'])
        samples['z'].append(data['gyro.z'])


def variance(v):
    if len(v) < 2:
        return float('nan')
    m = sum(v) / len(v)
    return sum((a - m) ** 2 for a in v) / len(v)


results = {}
try:
    time.sleep(2.0)

    # A bad boot makes every number below meaningless, and that has happened
    # repeatedly today -- so the caller should run preflight.py first.
    lg = LogConfig(name='g', period_in_ms=10)
    for ax in ('gyro.x', 'gyro.y', 'gyro.z'):
        lg.add_variable(ax, 'float')
    lg.data_received_cb.add_callback(on_data)
    cf.log.add_config(lg)
    lg.start()
    time.sleep(0.5)

    cf.param.set_value('motorPowerSet.enable', '1')
    time.sleep(0.3)

    for m in (1, 2, 3, 4):
        name = 'motorPowerSet.m%d' % m
        print('motor %d spinning...' % m)
        samples['x'].clear(); samples['y'].clear(); samples['z'].clear()
        cf.param.set_value(name, str(PWM))
        time.sleep(0.8)              # let it come up to speed first
        collecting = True
        time.sleep(SPIN_S)
        collecting = False
        cf.param.set_value(name, '0')
        results[m] = (variance(samples['x']), variance(samples['y']),
                      variance(samples['z']), len(samples['x']))
        time.sleep(SETTLE_S)

finally:
    # Whatever happened, stop the motors.
    try:
        for m in (1, 2, 3, 4):
            cf.param.set_value('motorPowerSet.m%d' % m, '0')
        cf.param.set_value('motorPowerSet.enable', '0')
    except Exception:
        pass
    try:
        scf.close_link()
    except Exception:
        pass

print()
print('--- shake while each motor ran alone (same PWM for all four) ---')
print('%-8s %10s %10s %10s %8s' % ('motor', 'gyro X', 'gyro Y', 'gyro Z', 'samples'))
for m in (1, 2, 3, 4):
    if m in results:
        vx, vy, vz, n = results[m]
        print('%-8d %10.1f %10.1f %10.1f %8d' % (m, vx, vy, vz, n))
    else:
        print('%-8d %10s' % (m, 'not run'))

vals = [max(results[m][0], results[m][1]) for m in results if results[m][3] > 5]
if len(vals) == 4:
    worst = max(vals)
    best = min(vals)
    print()
    if best > 0 and worst / best >= 3.0:
        bad = [m for m in results
               if max(results[m][0], results[m][1]) == worst]
        print('MOTOR %s SHAKES %.1fx MORE THAN THE QUIETEST.' % (bad[0], worst / best))
        print('That is the one to look at: propeller, mount, or the motor itself.')
    else:
        print('All four are within %.1fx of each other -- no odd one out.' % (worst / best if best else 0))
print()
print('motors are off.')
