# Pull the drone's onboard flight recording over the Crazyradio.
#
# This is the same CRTP port 14 protocol the phone app uses, so it reads the
# recording the drone already has -- it does not command anything and never
# touches the motors.
#
# WHY: the console trace keeps being lost. A 20-byte BLE link shared with three
# log blocks drops whatever does not fit, and a wall-following crash came back
# with no takeoff line, no mode line and no trace at all. The recording does not
# have that problem: it is written to memory in flight and read afterwards over
# a quiet link, and it has arrived intact every time.
#
# Sample layout (firmware FlightSample, little-endian, 15 bytes):
#   int16  x, y, z      position, mm
#   int16  yaw          heading, whole degrees
#   uint8  front, back, left, right, up, down   ranges in 2cm units, 0 = none
#   uint8  wf           high nibble = flight mode, low nibble = follower state

import struct
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

PORT_BULK = 14
CHAN_CTRL = 0
CHAN_DATA = 1
CMD_DUMP_START = 0x01
CMD_EOF = 0xFF

# The corner script's steps, 1..7. A recorded 0 means the follower never ran.
STATE_NAMES = {
    1: 'follow', 2: 'stop', 3: 'turn', 4: 'verify',
    5: 'past-corner', 6: 'reacquire', 7: 'GAVE-UP', 8: 'back-off',
}
MODE_NAMES = {0: 'HOVER', 1: 'WALL-RIGHT', 2: 'WALL-LEFT'}

samples = {}
eof_count = None


def on_bulk(packet):
    global eof_count
    data = bytes(packet.data)
    if packet.channel == CHAN_CTRL:
        if data and data[0] == CMD_EOF:
            eof_count = data[1] | (data[2] << 8) if len(data) >= 3 else 0
        return
    if packet.channel != CHAN_DATA or len(data) < 2:
        return
    index = data[0] | (data[1] << 8)
    body = data[2:]
    if len(body) < 14:
        return
    x, y, z, yaw = struct.unpack_from('<hhhh', body, 0)
    front, back, left, right, up, down = struct.unpack_from('<BBBBBB', body, 8)
    wf = body[14] if len(body) >= 15 else None
    tilt = body[15] * 2 if len(body) >= 16 else None
    samples[index] = dict(
        x=x, y=y, z=z, yaw=yaw,
        front=front * 20, back=back * 20, left=left * 20,
        right=right * 20, up=up * 20, down=down * 20,
        wf=wf, tilt=tilt,
    )


cflib.crtp.init_drivers()
print('scanning...', flush=True)
found = cflib.crtp.scan_interfaces()
if not found:
    print('NO DRONE FOUND. It is either off, or the phone app is holding the')
    print('link -- the radio chip keeps only one connection at a time.')
    sys.exit(1)
uri = found[0][0]
print('found:', uri, flush=True)

cf = Crazyflie(rw_cache='./cache')
cf.add_port_callback(PORT_BULK, on_bulk)

with SyncCrazyflie(uri, cf=cf) as scf:
    print('connected, asking for the recording\n', flush=True)
    pk = CRTPPacket()
    pk.port = PORT_BULK
    pk.channel = CHAN_CTRL
    pk.data = bytes([CMD_DUMP_START])
    scf.cf.send_packet(pk)

    # Wait for EOF, or for the samples to stop arriving.
    deadline = time.time() + 20
    last_seen = 0
    while time.time() < deadline and eof_count is None:
        time.sleep(0.2)
        if len(samples) != last_seen:
            last_seen = len(samples)
            deadline = time.time() + 5   # still flowing, extend

if not samples:
    print('The drone sent nothing. Either it has no recording yet, or the')
    print('flight was cleared after the last download.')
    sys.exit(0)

print(f'{len(samples)} samples received'
      + (f' (drone claimed {eof_count})' if eof_count is not None else ' (no EOF)'))
print()
hdr = f"{'#':>3} {'x':>6} {'y':>6} {'z':>5} {'yaw':>5}  {'front':>5} {'left':>5} {'right':>5} {'UP':>5} {'down':>5} {'TILT':>5}  {'state':<18}"
print(hdr)
print('-' * len(hdr))

for i in sorted(samples):
    s = samples[i]
    wf = s['wf']
    mode = MODE_NAMES.get((wf >> 4) & 0x0f, '?') if wf is not None else '-'
    state = STATE_NAMES.get(wf & 0x0f, '?') if wf is not None else '-'
    print(f"{i:>3} {s['x']:>6} {s['y']:>6} {s['z']:>5} {s['yaw']:>5}  "
          f"{s['front']:>5} {s['left']:>5} {s['right']:>5} "
          f"{s['up']:>5} {s['down']:>5} "
          f"{(s['tilt'] if s['tilt'] is not None else -1):>5}  {state:<18}")

print()
zs = [s['z'] for s in samples.values()]
print(f"peak altitude {max(zs)}mm, final {samples[max(samples)]['z']}mm")
if any(s['wf'] is None for s in samples.values()):
    print('NOTE: this recording predates the follower state being stored.')
print('\ndone. no motor command was ever sent.')
