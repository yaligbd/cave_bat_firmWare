# known-good

Verified flight binaries, one per milestone. They are committed on purpose
(`build/` is ignored): if the source ever stops building or flying, the drone
can be put back on a build that flew without needing the toolchain.

Flash one in bootloader mode (drone off, hold power about 3 s until the blue
LEDs blink fast):

```bash
cfloader flash known-good/cavebat-v16-corners-work.bin stm32-fw
```

| File | Milestone |
|---|---|
| `cavebat-v1-flying.bin` | First flight on custom firmware (2026-08-28) |
| `cavebat-v2-full-mission.bin` | First full mission |
| `cavebat-v3-flying-noMR.bin` | Flying without the Multi-ranger deck |
| `cavebat-v4-flying-withMR.bin` | Flying with the Multi-ranger deck |
| `cavebat-v6-onboard-recording.bin` | Records the flight onboard for the phone to download |
| `cavebat-v7-record-robustdeck.bin` | Recording with the retrying Multi-ranger driver |
| `cavebat-v9-lifecycle.bin` | Full flight life cycle with recording |
| `cavebat-v10-wallfollow.bin` | Holds a set distance from a wall |
| `cavebat-v11-turning.bin` | Turns, and records its heading |
| `cavebat-option2-heading-hold.bin` | Alternative build: heading hold |
| `cavebat-v15-three-flights-great.bin` | Three good flights |
| `cavebat-v16-corners-work.bin` | Corners flown as a script: stop, turn 90° on the spot, verify, carry on. Four flights, four successes; one flew both corners of the arena and returned home (2026-09-29) |

Known faults at v16: a short altitude dip just before the return leg starts,
and the 3D view did not match the route closely.

To rebuild a binary from source:

```bash
rm -rf build && make && cp build/cf2.bin known-good/<name>.bin
```
