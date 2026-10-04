# current-firmware — a frozen copy of what is actually flying

**These files are not compiled.** `src/Kbuild` names the objects it builds and
this folder is not among them. It is a snapshot, kept here so the firmware on
the aircraft can be read without a checkout, a stash, or any risk of confusing
it with work in progress.

## What this is

An exact copy of the source behind **`d92a00e`**, tagged `v19-FLIES-BOTH-WAYS`,
which is the binary flashed to the drone and saved at:

    known-good/cavebat-morning-d92a00e.bin

To restore the aircraft to this, from WSL with the radio attached:

    cfloader flash known-good/cavebat-morning-d92a00e.bin stm32-fw
    python3 tools/preflight.py

## What it does

Takes off, follows a wall on either side for half the timer, retraces its own
breadcrumbs home and lands, recording a sample a second to its own memory
throughout. Confirmed on 2026-10-03/04 flying the complete mission in **both**
directions — outward corner, inward corner, return, accurate landing.

The four files:

| file | what it is |
|---|---|
| `cavebat_mission.c` | the mission: phases, breadcrumbs, the return, landing, the recording, the CRTP download, the guards |
| `wallfollowing_corners.c/.h` | the corner script: follow, stop, turn, verify, past-corner, reacquire, back-off |
| `cavebat_multiranger.c` | the Multi-ranger deck driver, replacing the stock one |
| `Kbuild`, `app-config` | which objects build, and the firmware options |

## Two things that are NOT in this copy

The branch has moved past this commit. Neither of the following is flying, and
both are a deliberate omission rather than an oversight:

- **the CF_BACKOFF sign fix** — `vy = dir * speed` rather than `-dir`. The copy
  here still eases *into* the wall during back-off, which a recorded flight
  caught: the wall went 280mm, 260mm, 240mm while the step ran.
- **the stricter return thresholds** — push at 300mm rather than 200 on the way
  home, after a return came within 120mm of a wall.

Both are on branch `flying-plus-signfix` and built, but were not flown before
the aircraft was returned to this version.

## The thing worth remembering

Most of 2026-10-03 was spent changing this firmware to chase crashes that were
never caused by it. **Motor 4 was shaking two hundred times more than the
quietest motor**, and separately the drone sometimes boots without detecting its
decks — in which case there is no position feedback and it flips on takeoff
regardless of what is flashed.

Six firmware changes were made before that was measured. Three made things
worse. `tools/preflight.py` and `tools/motortest.py` exist so it never has to
be guessed at again: **measure the aircraft before trusting what it does.**
