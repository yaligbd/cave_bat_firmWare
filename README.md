# CaveBat firmware

Custom firmware for a Bitcraze Crazyflie 2.x, written in C and running on the drone's STM32F405. It flies an autonomous mission that a phone starts and monitors over Bluetooth Low Energy.

This is the drone-side half of [CaveBat](https://github.com/yaligbd/cave-controller). The phone app lives in that repository.

<!-- Add a photo of the drone, or a short clip of a mission running. -->

## The mission

`cavebat_mission.c` is the current build. One run is a full life cycle:

1. Take off to `mission.height`
2. Fly out for half of `mission.timer`, following a wall if `mission.wallfollow` is on
3. Retrace the route home
4. Land

It records a sample throughout, and the app downloads the recording over CRTP port 14 on request.

Safety guards abort the mission on low battery (`mission.minvbat`) or an obstacle closer than `mission.minobst`, and report the reason through `tele.endwhy`.

## Parameters

The app reads and writes these over the CRTP parameter port.

### `mission` — what the drone should do

| Parameter | Type | Meaning |
|---|---|---|
| `state` | uint8 | 0 idle, 1 fly, 2 abort |
| `timer` | uint32 | mission length, seconds |
| `height` | uint32 | hover altitude, mm |
| `sampledist` | uint32 | distance between recorded samples |
| `minvbat` | uint32 | abort below this battery voltage |
| `minobst` | uint32 | abort below this obstacle distance |
| `guards` | uint8 | enable the safety guards |
| `wallfollow` | uint8 | 1 enables wall following |
| `walldist` | uint32 | target distance from the wall |

### `tele` — what the drone reports

| Parameter | Type | Meaning |
|---|---|---|
| `alive` | uint16 | counts up; proves `appMain` is running |
| `canfly` | uint8 | drone is ready to fly |
| `clear` | uint8 | path ahead is clear |
| `maxz` | uint16 | highest altitude reached |
| `samples` | uint16 | samples recorded |
| `endwhy` | uint8 | why the mission ended |
| `vbat` | uint16 | battery, millivolts |
| `front` `back` `left` `right` `up` `down` | uint16 | ranges, mm; 0 means no reading |
| `x` `y` `z` | int16 | position, mm |

The same values are also published as a `tele` log group.

## Modules

| File | Role |
|---|---|
| `cavebat_mission.c` | **Current build.** The full life cycle above. |
| `cavebat_record.c` | Flies and records, without the mission structure. The fallback if the mission build misbehaves. |
| `cavebat.c` | Flies, no recording. The version that first flew. |
| `cavebat_wall.c` | An earlier wall-following attempt using velocity setpoints. Superseded, never flown, kept as a record. |
| `cavebat_i2cscan.c` | Diagnostic. Does not fly. |
| `cavebat_multiranger.c` | Multi-ranger deck driver, replacing the stock one (`CONFIG_DECK_MULTIRANGER=n` in `app-config`). Always built. |
| `wallfollowing_multiranger_onboard.c` | Wall-following strategy from Bitcraze, by K. N. McGuire — see the header for the paper it comes from. Not currently built. |

Only one `cavebat_*.c` defining `appMain()` may be built at a time. `src/Kbuild` selects it.

## Building

Builds against the Bitcraze `crazyflie-firmware` tree as an out-of-tree app, with the ARM GCC toolchain.

```bash
make
```

Two things that cost real time when they go wrong:

- The app needs `CONFIG_APP_ENABLE=y` to come from the config file the build actually reads. If `appMain` never starts, check which config was picked up before looking for a bug in the code.
- `make` does not always rebuild changed sources. When a change seems to have no effect, `rm -rf build` and build again.

## Flashing

Over a Crazyradio PA.

- Hold the power button for 3 seconds to enter bootloader mode.
- Plug the Crazyradio straight into the machine. A USB hub causes packet loss.
- On WSL, `usbipd attach` has to be redone after every restart.
- Unplug the Crazyradio before testing over BLE — the nRF51 disables BLE when it sees radio traffic.

## Notes on BLE

The Crazyflie's BLE bridge runs on the nRF51, not the STM32, and two of its properties shape everything above it:

- Notifications are capped at 20 bytes, which truncates longer parameter names.
- The bridge is poll-driven — it needs a continuous stream of null packets to keep delivering data.

Stock firmware also gates CRTP log streaming behind a connection check that only reports a connected state for the radio link, so log blocks never start over BLE. This firmware bypasses that check, which is what lets the app receive live telemetry.

## Hardware

- Bitcraze Crazyflie 2.x (STM32F405)
- Flow deck v2
- Multi-ranger deck
- Crazyradio PA, for flashing

<!-- TODO: the three pending-cavebat-*.bin files in the repo root. If the app downloads them for
     over-the-air updates, say so here. If they are just build output, delete them and add *.bin to .gitignore. -->
