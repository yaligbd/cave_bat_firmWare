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
