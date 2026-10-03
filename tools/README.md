# tools

Three scripts that talk to the drone over the Crazyradio. None of them send a
motor command except `motortest.py`, which says so loudly.

Run them from WSL with the radio attached:

    usbipd attach --wsl --busid <id>        # from Windows; the id moves, check `usbipd list`
    python3 tools/preflight.py

---

## preflight.py — run this before every flight

Reads the boot log and answers one question: **did this boot find the decks?**

Deck detection is intermittent on this aircraft. When it fails the drone says

    OW: Cmd 0x22 timeout.
    DECK_INFO: Reading deck nr:0 [FAILED]
    DECK_CORE: 0 deck(s) found
    ESTIMATOR: Using Complementary (1) estimator

and that last line is the one that matters. With no Flow deck the firmware
falls back to an estimator with no position or velocity feedback, and a
Crazyflie in that state cannot hold station — it lifts off and goes over,
whatever is flashed.

This cost most of 2026-10-03 to work out. The same binary flew two complete
missions in the morning and flipped on takeoff in the afternoon, and six
firmware changes were made chasing it. None of them could have helped.

A good boot prints `GOOD TO FLY`. Anything else: power off, reseat both decks,
power on, run it again.

## motortest.py — when it flips for no reason

Spins each motor on its own at an identical PWM and measures how much the
airframe shakes, from the gyro. Four comparable numbers, or none.

**The propellers spin.** Flat surface, hands clear. It never lifts off: only
one motor runs at a time and the PWM is well under hover thrust. It switches
the motors off again even if it fails partway.

The firmware's own `health.startPropTest` exists, but it stopped after two
motors on every attempt here, which is why this drives them directly instead.

What it found on 2026-10-03, after a day of unexplained flips:

    motor   gyro X   gyro Y
    1          0.1      0.1
    2          3.9      4.2
    3          0.0      0.0
    4         17.4     26.2     <- two hundred times the quietest

Motor 4 was replaced and the aircraft flew both directions immediately. Read
the comparison between the four, never the absolute numbers. A motor shaking on
the **Z** axis specifically usually means the propeller is the wrong rotation
for that position, or is not pushed fully down.

## pullflight.py — read the last flight out of the drone

Downloads the onboard recording over the radio and prints it as a table: every
sample's position, heading, six ranges, worst tilt, and which step the wall
follower was in.

The recording lives in RAM, so a crash that resets the drone takes it with it.
Download before power-cycling.
