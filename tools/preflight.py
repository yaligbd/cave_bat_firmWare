"""Before every flight: did this boot find the decks?

Deck detection is intermittent on this aircraft. When it fails the boot log says

    OW: Cmd 0x22 timeout.
    DECK_INFO: Reading deck nr:0 [FAILED]. No driver will be initialized!
    DECK_CORE: 0 deck(s) found
    ESTIMATOR: Using Complementary (1) estimator

and that last line is the one that matters. No Flow deck means the firmware
falls back to an estimator with no position or velocity feedback, and a
Crazyflie in that state cannot hold station: it lifts off and goes over. The
firmware is identical either way, which is why the same binary flew two
complete missions in the morning and flipped on takeoff in the afternoon.

Run this after powering on, BEFORE sending a mission. It only listens.
"""
import time

import cflib.crtp

URI = 'radio://0/80/2M'

cflib.crtp.init_drivers(enable_debug_driver=False)
found = cflib.crtp.scan_interfaces()
if not found:
    print('NO DRONE. It is off, or the phone app is holding the link.')
    raise SystemExit(1)

link = cflib.crtp.get_link_driver(URI)
buf = ''
t0 = time.time()
while time.time() - t0 < 10:
    pk = link.receive_packet(0.2)
    if pk is not None and pk.port == 0:
        buf += bytes(pk.data).decode('utf8', 'replace')
link.close()

if not buf.strip():
    print('NO CONSOLE OUTPUT.')
    print('Either it booted a while ago, or the main processor is hung.')
    print('POWER CYCLE and run this again.')
    raise SystemExit(1)

decks = None
estimator = None
mr_ok = 0
for line in buf.splitlines():
    if 'deck(s) found' in line:
        try:
            decks = int(line.split('DECK_CORE:')[1].strip().split()[0])
        except Exception:
            pass
    if 'ESTIMATOR: Using' in line:
        estimator = line.split('Using', 1)[1].strip()
    if line.startswith('MR: Init') and '[OK]' in line:
        mr_ok += 1

print('decks found      :', decks)
print('multiranger ok   :', mr_ok, 'of 5 sensors')
print('estimator        :', estimator)
print()

ok = (decks is not None and decks >= 1
      and estimator is not None and 'Kalman' in estimator
      and mr_ok >= 5)

if ok:
    print('GOOD TO FLY.')
else:
    print('*** DO NOT FLY ***')
    if decks == 0 or decks is None:
        print('  The decks were not detected on this boot.')
    if estimator and 'Kalman' not in estimator:
        print('  No Flow deck, so there is no position feedback. It will flip.')
    if mr_ok < 5:
        print('  Only %d of 5 rangers came up, so wall following cannot work.' % mr_ok)
    print()
    print('  Power off, reseat both decks firmly, power on, run this again.')
