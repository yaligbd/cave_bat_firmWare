# known-good

`cavebat-v1-flying.bin` is the exact firmware that was verified flying on
2026-08-28 (tag `v1-flying`).

It is committed on purpose, even though `build/` is ignored. If the build ever
breaks again, you do not need it to work in order to get the drone flying --
you can flash this file directly.

Put the drone in bootloader mode (off, then hold power ~3s until the blue
lights blink fast), then from Ubuntu/WSL:

    cfloader flash known-good/cavebat-v1-flying.bin stm32-fw

To regenerate this file from source instead:

    make && cp build/cf2.bin known-good/cavebat-v1-flying.bin

## cavebat-v16-corners-work.bin -- the first build that turned corners

Verified 2026-09-29 (tag `v16-corners-work`). Four flights, four successes:
one hover, three wall-follow-right, and one of those flew **both corners of the
arena and returned home**. The first time that has ever happened.

Flies a corner as a script -- stop, turn 90 degrees on the spot, verify, carry
on -- instead of as a blend of states. Yaw only ever happens in discrete turns.

    cfloader flash known-good/cavebat-v16-corners-work.bin stm32-fw

Known faults at this tag: a dip in altitude just before the return leg starts,
and the 3D view does not match the route closely.
