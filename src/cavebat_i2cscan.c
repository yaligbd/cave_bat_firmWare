// cavebat_i2cscan.c - DIAGNOSTIC ONLY. This build does not fly.
//
// WHAT IT ANSWERS
//   The Multi-ranger's five sensors have failed to initialise on every boot,
//   which stops the whole firmware from starting. This scans the deck I2C bus
//   and prints every address that answers, so we know whether the deck is
//   electrically reachable rather than guessing.
//
// WHY ONE CHIP EXPLAINS ALL FIVE FAILURES
//   The five VL53L1x sensors all power up at the SAME address (0x29). The
//   driver keeps them in reset and releases them one at a time using a
//   separate chip on the deck: a PCA9554 I/O expander at 0x20. Only once a
//   sensor is released does it appear on the bus and get moved to its own
//   address.
//
//   multiranger.c calls pca95x4Init() / ConfigOutput() / ClearOutput() and
//   checks none of their return values. If the expander does not answer, every
//   sensor stays in reset, nothing appears at 0x29, and all five report
//   [FAIL] -- with nothing anywhere saying the expander was the problem.
//
// HOW TO READ THE RESULT
//   0x20 PRESENT  -> the deck is powered and reachable. The fault is in the
//                    sensors, the init sequence, or timing -- not the wiring.
//   0x20 ABSENT   -> nothing on the deck answers. No software change can fix
//                    that; it is power or connection.
//
// A scan is a write of zero bytes: the chip either ACKs its address or it does
// not. It reads no registers and changes no state.

#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2cdev.h"
#include "param.h"
#include "deck_core.h"

#define DEBUG_MODULE "I2CSCAN"
#include "debug.h"

// Chips we specifically care about, so the output names them rather than
// leaving a bare address to be looked up.
static const char *knownDevice(uint8_t addr) {
  switch (addr) {
    case 0x20: return "PCA9554 I/O expander  <-- MULTI-RANGER XSHUT CONTROL";
    case 0x29: return "VL53L1x at its default address (un-configured)";
    case 0x2A: return "VL53L1x moved to its own address";
    case 0x2B: return "VL53L1x moved to its own address";
    case 0x2C: return "VL53L1x moved to its own address";
    case 0x2D: return "VL53L1x moved to its own address";
    case 0x2E: return "VL53L1x moved to its own address";
    case 0x2F: return "VL53L1x moved to its own address";
    case 0x0C: return "AK8963 magnetometer";
    case 0x18: return "BMI088 accelerometer";
    case 0x50: return "deck 1-wire memory (deck ID)";
    case 0x53: return "deck 1-wire memory (deck ID)";
    case 0x68: return "BMI088 gyroscope / MPU";
    case 0x76: return "BMP388 barometer";
    case 0x77: return "BMP388 barometer";
    default:   return "";
  }
}

static bool probe(uint8_t addr) {
  // A one-byte READ, not a zero-length write. The first version used
  // i2cdevWrite(..., 0, NULL) and reported ZERO devices on a bus where the
  // Flow deck's sensor demonstrably works -- so a zero-length write does not
  // drive the address phase here. Reading one byte does.
  uint8_t b;
  return i2cdevRead(I2C1_DEV, addr, 1, &b);
}

void appMain(void) {
  // Let the rest of the system finish its own bus traffic first, otherwise a
  // scan can collide with a driver mid-transaction and mis-report.
  vTaskDelay(M2T(4000));

  // Which decks does the drone actually SEE?
  //
  // This is a different question from the I2C scan, and the two together are
  // what matter. A deck announces itself over a 1-wire ID memory on its own
  // dedicated pin; its sensors talk over I2C on two other pins. So:
  //
  //   deck listed + 0x20 answers    -> everything is connected
  //   deck listed + 0x20 silent     -> the deck is seated and identifies
  //                                    itself, but its two I2C pins are not
  //                                    getting through
  //   deck NOT listed               -> the drone cannot see the deck at all;
  //                                    this is not an I2C problem
  while (1) {
    DEBUG_PRINT("=== decks the drone can see ===\n");
    int nDecks = deckCount();
    DEBUG_PRINT("  deckCount = %d\n", nDecks);
    for (int i = 0; i < nDecks; i++) {
      DeckInfo *di = deckInfo(i);
      const char *nm = (di->driver && di->driver->name) ? di->driver->name : "(no driver)";
      DEBUG_PRINT("  deck %d: vid=0x%02X pid=0x%02X %s\n",
                  i, (int)di->vid, (int)di->pid, nm);
    }
    DEBUG_PRINT("  (Multi-ranger is vid=0xBC pid=0x0C)\n");

    DEBUG_PRINT("=== I2C1 scan (deck bus) ===\n");

    int found = 0;
    bool expanderPresent = false;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
      if (probe(addr)) {
        found++;
        if (addr == 0x20) expanderPresent = true;
        const char *name = knownDevice(addr);
        if (name[0]) {
          DEBUG_PRINT("  0x%02X answers - %s\n", (int)addr, name);
        } else {
          DEBUG_PRINT("  0x%02X answers\n", (int)addr);
        }
      }
      vTaskDelay(M2T(2));   // keep the bus unhurried; a rushed scan can miss
    }

    DEBUG_PRINT("=== %d device(s) answered ===\n", found);

    // Sanity check on the scan itself. The Flow deck works on this same bus,
    // so if NOTHING answers then the scan is broken, not the bus -- and
    // "nothing is reachable" would be a false and expensive conclusion.
    if (found == 0) {
      DEBUG_PRINT("SCAN INVALID: not even the Flow deck answered, and it is\n");
      DEBUG_PRINT("  known working. Distrust this scan, not the hardware.\n");
      vTaskDelay(M2T(10000));
      continue;
    }
    if (expanderPresent) {
      DEBUG_PRINT("VERDICT: multiranger expander 0x20 IS present.\n");
      DEBUG_PRINT("  The deck is powered and reachable. Look at the sensors,\n");
      DEBUG_PRINT("  the init sequence or timing -- not the connection.\n");
    } else {
      DEBUG_PRINT("VERDICT: multiranger expander 0x20 did NOT answer.\n");
      DEBUG_PRINT("  Nothing on the deck is reachable over I2C, so the five\n");
      DEBUG_PRINT("  sensors can never be released from reset. No firmware\n");
      DEBUG_PRINT("  change can work around this.\n");
    }

    // Repeat slowly. An intermittent contact shows up as an address that
    // appears in one pass and not the next, which a single scan would miss.
    vTaskDelay(M2T(10000));
  }
}
