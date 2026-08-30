// cavebat_record.c - the flying firmware, plus recording onto the drone.
//
// A copy of cavebat.c (tag v4-flying-withMR, the version that flies) with one
// capability added: it records a sample every second while airborne and hands
// the recording to the app over CRTP port 14 on request.
//
// Deliberately unchanged from the flying version: the 10Hz loop, high-level
// commander only, no velocity setpoints, no wall following. An earlier attempt
// changed all of those at once and the drone flipped on takeoff, leaving four
// suspects and no way to separate them. If this flips, recording is the cause,
// because it is the only difference.
#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "param.h"
#include "log.h"
#include "commander.h" 
#include "crtp_commander_high_level.h"
#include "stabilizer_types.h"
#include "crtp.h"
#include <string.h>

#define DEBUG_MODULE "CAVEBAT"
#include "debug.h"

// --- Telemetry Variables ---
static uint16_t tele_alive = 0;    // counts up, proves appMain is running
static uint16_t tele_vbat  = 0;    // millivolts
static uint16_t tele_front = 0;    // mm, 0 = no reading
static uint16_t tele_back  = 0;
static uint16_t tele_left  = 0;
static uint16_t tele_right = 0;
static uint16_t tele_up    = 0;
static uint16_t tele_down  = 0;
static int16_t  tele_x     = 0;    // mm
static int16_t  tele_y     = 0;
static int16_t  tele_z     = 0;

// --- Mission Parameters (Required by App) ---
static uint8_t  mission_state = 0; // 0=Idle, 1=Fly, 2=Abort
static uint32_t mission_timer = 10; // Seconds to hover
static uint32_t mission_height = 500; // Hover altitude in mm
static uint32_t mission_sampledist = 10; // cm

// --- Battery Safety ---
// Resting voltage below which takeoff is refused outright. A Crazyflie 2.x
// LiPo is ~4.2V full and ~3.0V empty; below roughly 3.7V at rest it no longer
// has the headroom to climb. That failure looks like "took off, never reached
// altitude, came down early" rather than like a flat battery.
static uint32_t mission_minvbat = 3700;   // mV, resting threshold, tunable
// Voltage sags hard under motor load, so the in-flight cutoff must sit below
// the resting threshold or every flight would abort the instant it lifted.
//
// 300mV was measured to be far too tight: a flight starting at 3890mV sagged
// to 3383mV during the takeoff climb and tripped a 3400mV cutoff, ending the
// mission at 2cm. Spin-up is the worst moment for sag, so the margin is now
// 600mV AND the check is suppressed until the climb is done. 3700-600 = 3100mV
// still sits above the ~3.0V where a Crazyflie cell is genuinely empty.
#define VBAT_INFLIGHT_MARGIN_MV 600

// 1 = healthy enough to attempt takeoff, 0 = too low. Published so the app can
// show it, instead of letting a doomed flight start and look like a bug.
static uint8_t  tele_canfly = 0;

// --- Obstacle Safety ---
// Side clearance below which flight is refused/aborted, in mm. Tunable,
// because 200mm is easy to trip indoors: a desk edge, a chair, or the pilot
// standing nearby all sit inside it.
static uint32_t mission_minobst = 200;

// 1 = clear to take off, 0 = something is inside mission_minobst. Published so
// the app can say WHY it will not fly, instead of the drone hopping a few
// centimetres and landing, which reads as a broken controller.
static uint8_t  tele_clear = 0;

// Master switch for CaveBat's own safety guards. OFF by default: every guard
// added so far has fired on a false positive and ended a healthy flight (a
// side ranger catching the floor during the climb; normal LiPo sag read as a
// collapsed battery). With this at 0 nothing in this file will abort a
// mission -- it flies the timer out and lands normally.
//
// tele_canfly and tele_clear are still computed and published either way, so
// the app can show battery and obstacle status as INFORMATION without any of
// it stopping a flight.
//
// This does NOT disable Bitcraze's supervisor. Tumble detection and the
// critical-battery cutoff live in the stock firmware and still apply.
//
// Set mission.guards = 1 to put the pre-flight refusals and in-flight aborts
// back on.
static uint8_t  mission_guards = 0;

// Highest point reached during the last flight, in mm, and why that flight
// ended. Published so a flight can be judged from the app alone -- no radio
// script, no second link. The drone reports on itself.
//   0 = never flown   1 = timer completed   2 = aborted
static uint16_t tele_maxz   = 0;
static uint8_t  tele_endwhy = 0;

// 0 means "no reading" (out of range) and must NOT count as an obstacle,
// otherwise open space would read as blocked.
static bool sideBlocked(uint16_t mm) {
  return (mm > 0) && (mm < (uint16_t)mission_minobst);
}

// logGetFloat() ASSERTs on an invalid id, which halts the drone. A sensor that
// is not present -- because its deck driver is not in the build, or the deck
// did not initialise -- yields exactly such an id. Reading through this helper
// means the firmware runs with or without the Multi-ranger fitted instead of
// dying on the first read.
static float safeLogFloat(logVarId_t id) {
  return logVarIdIsValid(id) ? logGetFloat(id) : 0.0f;
}

// --- Recording ------------------------------------------------------------
//
// SIZE IS LOAD-BEARING. A download packet is 1 CRTP header + 2 index bytes +
// sizeof(FlightSample), and the whole thing must be at most 19 bytes so the BLE
// layer sends it as ONE 20-byte notification. Split packets arrive corrupted on
// this hardware -- the nRF51's s130 stack caps ATT_MTU at 23 and it cannot be
// raised. At 12 bytes the packet totals 15, with room to spare.
//
// That budget is why the ranges are single bytes in 2cm units rather than
// millimetres: six uint16 ranges plus position would be 18 bytes and push the
// packet to 21, which fragments. 2cm units cover 0-5.1m, comfortably past the
// multiranger's 3m ceiling, and 2cm resolution is far finer than anything a
// cave survey needs. Position stays full-resolution millimetres.
typedef struct __attribute__((packed)) {
  int16_t x, y, z;                              // position, mm
  uint8_t front, back, left, right, up, down;   // ranges, 2cm units, 0 = none
} FlightSample;

// 1Hz sampling, so 180 samples is three minutes -- well past the one minute we
// fly. 180 * 12 = 2.1KB of static RAM.
#define MAX_SAMPLES 180
static FlightSample flight_log[MAX_SAMPLES];
static uint16_t sample_count = 0;
static uint16_t tele_samples = 0;   // published so the app can see the count

// mm -> 2cm units, saturating. 0 stays 0 and keeps meaning "nothing in range".
static uint8_t rangeTo2cm(uint16_t mm) {
  if (mm == 0) return 0;
  uint16_t u = mm / 20;
  return (u > 255) ? 255 : (uint8_t)u;
}

// --- Download protocol, CRTP port 14 --------------------------------------
#define CRTP_PORT_BULK  14
#define BULK_CHAN_CTRL  0
#define BULK_CHAN_DATA  1
#define CMD_DUMP_START  0x01   // app -> drone: send me the recording
#define CMD_CLEAR_MEM   0x02   // app -> drone: discard it
#define CMD_EOF         0xFF   // drone -> app: that was the last sample

static bool dump_requested  = false;
static bool clear_requested = false;

// Runs in CRTP task context, so it only raises a flag. The transfer itself
// happens in the main loop, where blocking is safe.
static void bulkCrtpCb(CRTPPacket *pk) {
  if (pk->channel == BULK_CHAN_CTRL && pk->size >= 1) {
    if (pk->data[0] == CMD_DUMP_START) dump_requested = true;
    if (pk->data[0] == CMD_CLEAR_MEM)  clear_requested = true;
  }
}

// One sample per packet, index included so the app can tell a dropped packet
// from a shifted one rather than silently mis-aligning every later sample.
static void sendRecording(void) {
  DEBUG_PRINT("CAVEBAT: sending %d samples\n", (int)sample_count);
  CRTPPacket pk;
  for (uint16_t i = 0; i < sample_count; i++) {
    pk.port = CRTP_PORT_BULK;
    pk.channel = BULK_CHAN_DATA;
    pk.size = 2 + sizeof(FlightSample);
    pk.data[0] = i & 0xff;
    pk.data[1] = (i >> 8) & 0xff;
    memcpy(&pk.data[2], &flight_log[i], sizeof(FlightSample));
    crtpSendPacketBlock(&pk);
  }
  pk.port = CRTP_PORT_BULK;
  pk.channel = BULK_CHAN_CTRL;
  pk.size = 3;
  pk.data[0] = CMD_EOF;
  pk.data[1] = sample_count & 0xff;
  pk.data[2] = (sample_count >> 8) & 0xff;
  crtpSendPacketBlock(&pk);
  DEBUG_PRINT("CAVEBAT: send complete\n");
}

static uint16_t clampRange(float mm) {
  if (mm <= 0.0f || mm > 3000.0f) return 0;
  return (uint16_t)mm;
}

void appMain(void) {
  vTaskDelay(M2T(3000));
  
  logVarId_t idVbat  = logGetVarId("pm", "vbat");
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idBack  = logGetVarId("range", "back");
  logVarId_t idLeft  = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idUp    = logGetVarId("range", "up");
  logVarId_t idDown  = logGetVarId("range", "zrange");
  logVarId_t idX     = logGetVarId("stateEstimate", "x");
  logVarId_t idY     = logGetVarId("stateEstimate", "y");
  logVarId_t idZ     = logGetVarId("stateEstimate", "z");

  DEBUG_PRINT("CAVEBAT: Flight & Telemetry starting\n");
  
  uint32_t flight_start_time = 0;
  uint32_t hover_deadline = 0;  // tick at which a hovering flight should land
  uint8_t  obstacle_streak = 0; // consecutive cycles seeing an obstacle
  uint8_t  lowbat_streak = 0;   // consecutive cycles seeing a collapsed battery
  // Tick at which the climb finishes. Obstacle checking is suppressed until
  // then: while climbing, the drone tilts and the side-facing rangers catch
  // the floor, producing readings well under the limit with nothing actually
  // in the way. Measured in flight: sides flicked between 32766 ("nothing")
  // and ~450mm during a climb that was steady at 400-800mm at rest.
  uint32_t climb_done_tick = 0;
  bool is_flying = false;
  uint32_t last_sample_tick = 0;

  crtpRegisterPortCB(CRTP_PORT_BULK, bulkCrtpCb);

  crtpCommanderHighLevelInit();

  while (1) {
    // 1. Update Telemetry
    tele_alive++;
    tele_vbat  = (uint16_t)(safeLogFloat(idVbat) * 1000.0f);
    tele_canfly = (tele_vbat >= mission_minvbat) ? 1 : 0;
    tele_clear = (sideBlocked(tele_front) || sideBlocked(tele_back) ||
                  sideBlocked(tele_left)  || sideBlocked(tele_right)) ? 0 : 1;
    tele_front = clampRange(safeLogFloat(idFront));
    tele_back  = clampRange(safeLogFloat(idBack));
    tele_left  = clampRange(safeLogFloat(idLeft));
    tele_right = clampRange(safeLogFloat(idRight));
    tele_up    = clampRange(safeLogFloat(idUp));
    tele_down  = clampRange(safeLogFloat(idDown));
    tele_x     = (int16_t)(safeLogFloat(idX) * 1000.0f);
    tele_y     = (int16_t)(safeLogFloat(idY) * 1000.0f);
    tele_z     = (int16_t)(safeLogFloat(idZ) * 1000.0f);

    // Download requests. Safe at any time: this only reads the buffer, and the
    // app is only ever connected while the drone is on the ground.
    if (clear_requested) {
      clear_requested = false;
      sample_count = 0;
      tele_samples = 0;
      DEBUG_PRINT("CAVEBAT: recording cleared\n");
    }
    if (dump_requested) {
      dump_requested = false;
      sendRecording();
    }

    // One sample per second while airborne. Driven off is_flying rather than a
    // phase machine, because there is no phase machine here -- that is the
    // point of this version.
    if (is_flying && (xTaskGetTickCount() - last_sample_tick) >= M2T(1000)) {
      last_sample_tick = xTaskGetTickCount();
      if (sample_count < MAX_SAMPLES) {
        FlightSample *fs = &flight_log[sample_count];
        fs->x = tele_x;  fs->y = tele_y;  fs->z = tele_z;
        fs->front = rangeTo2cm(tele_front);
        fs->back  = rangeTo2cm(tele_back);
        fs->left  = rangeTo2cm(tele_left);
        fs->right = rangeTo2cm(tele_right);
        fs->up    = rangeTo2cm(tele_up);
        fs->down  = rangeTo2cm(tele_down);
        sample_count++;
        tele_samples = sample_count;
      }
    }

    // 2. Flight State Machine
    if (mission_guards && mission_state == 1 && !is_flying && !tele_canfly) {
        // Too low to climb. Refuse rather than half-fly: an underpowered
        // takeoff looks like a software fault but is really a flat battery.
        DEBUG_PRINT("CAVEBAT: Takeoff REFUSED, battery %d mV < %d mV min\n",
                    (int)tele_vbat, (int)mission_minvbat);
        mission_state = 0; // clear the request so the app sees it rejected

    } else if (mission_guards && mission_state == 1 && !is_flying && !tele_clear) {
        // Something is already inside the clearance limit. Taking off here just
        // trips the in-flight abort ~100ms later, so the drone hops a few
        // centimetres and lands -- which looks like a broken controller rather
        // than an obstacle. Refuse up front and say so.
        DEBUG_PRINT("CAVEBAT: Takeoff REFUSED, obstacle within %d mm (f=%d b=%d l=%d r=%d)\n",
                    (int)mission_minobst, (int)tele_front, (int)tele_back,
                    (int)tele_left, (int)tele_right);
        mission_state = 0;

    } else if (mission_state == 1 && !is_flying) {
        // App requested Takeoff
        DEBUG_PRINT("CAVEBAT: Initiating Takeoff to %d mm, battery %d mV\n",
                    (int)mission_height, (int)tele_vbat);
        flight_start_time = xTaskGetTickCount();
        is_flying = true;
        // Each flight starts a fresh recording, and the drone never parks
        // waiting for the app to acknowledge a download. An earlier version did
        // wait, on a command the app could not send, and silently refused every
        // mission after the first.
        sample_count = 0;
        tele_samples = 0;
        last_sample_tick = xTaskGetTickCount();
        
        float target_height_m = mission_height / 1000.0f;
        float takeoff_duration = target_height_m / 0.3f; // safe velocity 0.3 m/s
        if (takeoff_duration < 1.0f) takeoff_duration = 1.0f;
        
        crtpCommanderHighLevelTakeoff(target_height_m, takeoff_duration);

        // The timer is meant to be HOVER time. Counting it from the moment the
        // climb starts meant a short timer expired mid-climb: a 1s mission
        // began descending before it ever reached altitude, which looked like
        // "it never got off the ground". Land only once the climb has finished
        // AND the requested hover time has elapsed on top of it.
        obstacle_streak = 0;
        lowbat_streak = 0;
        tele_maxz = 0;
        tele_endwhy = 0;
        climb_done_tick = xTaskGetTickCount()
                        + M2T((uint32_t)(takeoff_duration * 1000.0f));
        hover_deadline = xTaskGetTickCount()
                       + M2T((uint32_t)(takeoff_duration * 1000.0f))
                       + M2T(mission_timer * 1000);
        
    } else if (mission_state == 1 && is_flying) {
        // Peak altitude, so the flight can be judged after the fact.
        if (tele_z > 0 && (uint16_t)tele_z > tele_maxz) tele_maxz = (uint16_t)tele_z;

        // App is in Fly mode -> Check Timer
        if ((int32_t)(xTaskGetTickCount() - hover_deadline) >= 0) {
            tele_endwhy = 1;
            DEBUG_PRINT("CAVEBAT: FLIGHT OK, peak %d of %d mm, %d samples\n",
                        (int)tele_maxz, (int)mission_height, (int)sample_count);
            
            float target_height_m = mission_height / 1000.0f;
            float land_duration = target_height_m / 0.3f;
            if (land_duration < 1.0f) land_duration = 1.0f;
            
            crtpCommanderHighLevelLand(0.0f, land_duration);
            vTaskDelay(M2T((uint32_t)(land_duration * 1000) + 500));
            
            mission_state = 0; // Reset to idle
            is_flying = false;
        } else {
            // High-level commander automatically maintains position (hovers)
            // after the takeoff trajectory is complete. No explicit API call needed.
            
            // Abort if the battery collapses mid-flight. A controlled landing
            // beats the supervisor cutting the motors at altitude. Skipped
            // during the climb, where spin-up sag is worst, and requires the
            // reading to persist so one dip cannot end a flight.
            if (!mission_guards) {
                lowbat_streak = 0;
            } else if ((int32_t)(xTaskGetTickCount() - climb_done_tick) < 0) {
                lowbat_streak = 0;
            } else if (tele_vbat > 0 &&
                       tele_vbat < (mission_minvbat - VBAT_INFLIGHT_MARGIN_MV)) {
                lowbat_streak++;
            } else {
                lowbat_streak = 0;
            }
            if (lowbat_streak >= 3) {
                DEBUG_PRINT("CAVEBAT: Battery %d mV collapsed in flight, landing\n",
                            (int)tele_vbat);
                mission_state = 2; // Trigger Abort
            }

            // Abort if an obstacle gets closer than 200mm (0 means no reading)
            // Only once the climb is done -- see climb_done_tick. Requiring the
            // obstacle to persist as well: a single stray short reading from a
            // ToF sensor should not end a flight, but three in a row at 10Hz is
            // 0.3s, still fast enough to be useful.
            if (!mission_guards) {
                obstacle_streak = 0;
            } else if ((int32_t)(xTaskGetTickCount() - climb_done_tick) < 0) {
                obstacle_streak = 0;
            } else if (sideBlocked(tele_front) || sideBlocked(tele_back) ||
                       sideBlocked(tele_left)  || sideBlocked(tele_right)) {
                obstacle_streak++;
            } else {
                obstacle_streak = 0;
            }
            if (obstacle_streak >= 3) {
                DEBUG_PRINT("CAVEBAT: Obstacle within %d mm (f=%d b=%d l=%d r=%d), aborting\n",
                            (int)mission_minobst, (int)tele_front, (int)tele_back,
                            (int)tele_left, (int)tele_right);
                mission_state = 2; // Trigger Abort
            }
        }
        
    } else if (mission_state == 2 && !is_flying) {
        // Abort pressed while already on the ground. Commanding a landing here
        // ran a full land trajectory from zero height, spinning the motors for
        // over a second for no reason. Just clear the request.
        DEBUG_PRINT("CAVEBAT: Abort ignored, not flying\n");
        mission_state = 0;

    } else if (mission_state == 2) {
        // App requested Abort
        tele_endwhy = 2;
        DEBUG_PRINT("CAVEBAT: FLIGHT ABORTED, peak %d mm of %d mm asked\n",
                    (int)tele_maxz, (int)mission_height);
        
        float target_height_m = mission_height / 1000.0f;
        float land_duration = target_height_m / 0.3f;
        if (land_duration < 1.0f) land_duration = 1.0f;
        
        crtpCommanderHighLevelLand(0.0f, land_duration);
        vTaskDelay(M2T((uint32_t)(land_duration * 1000) + 500));
        
        mission_state = 0; // Reset to idle
        is_flying = false;
    }

    if (!is_flying && mission_state == 0) {
       // Reset flight flag if landed
       is_flying = false; 
    }

    // Every 10s, not every 1s. The loop runs at 10Hz, so "% 10" printed a
    // ~60-char status line every second. Over BLE that saturates the link and
    // queues real replies (param/log TOC) behind console text until the app
    // times out waiting for them. This is a heartbeat, not telemetry - the
    // app reads tele.* directly.
    if ((tele_alive % 100) == 0) {
      DEBUG_PRINT("CB state=%d bat=%d f=%d b=%d l=%d r=%d u=%d d=%d\n",
                  (int)mission_state, (int)tele_vbat, (int)tele_front,
                  (int)tele_back, (int)tele_left, (int)tele_right,
                  (int)tele_up, (int)tele_down);
    }
    
    // Stream data at 10Hz (100ms) to ensure app captures 1 sample per second cleanly
    vTaskDelay(M2T(100));
  }
}

// --- Parameter Registration ---
PARAM_GROUP_START(tele)
  PARAM_ADD(PARAM_UINT16, alive, &tele_alive)
  PARAM_ADD(PARAM_UINT8,  canfly, &tele_canfly)
  PARAM_ADD(PARAM_UINT8,  clear,  &tele_clear)
  PARAM_ADD(PARAM_UINT16, maxz,   &tele_maxz)
  PARAM_ADD(PARAM_UINT16, samples, &tele_samples)
  PARAM_ADD(PARAM_UINT8,  endwhy, &tele_endwhy)
  PARAM_ADD(PARAM_UINT16, vbat,  &tele_vbat)
  PARAM_ADD(PARAM_UINT16, front, &tele_front)
  PARAM_ADD(PARAM_UINT16, back,  &tele_back)
  PARAM_ADD(PARAM_UINT16, left,  &tele_left)
  PARAM_ADD(PARAM_UINT16, right, &tele_right)
  PARAM_ADD(PARAM_UINT16, up,    &tele_up)
  PARAM_ADD(PARAM_UINT16, down,  &tele_down)
  PARAM_ADD(PARAM_INT16,  x,     &tele_x)
  PARAM_ADD(PARAM_INT16,  y,     &tele_y)
  PARAM_ADD(PARAM_INT16,  z,     &tele_z)
PARAM_GROUP_STOP(tele)

PARAM_GROUP_START(mission)
  PARAM_ADD(PARAM_UINT8,  state,      &mission_state)
  PARAM_ADD(PARAM_UINT32, timer,      &mission_timer)
  PARAM_ADD(PARAM_UINT32, height,     &mission_height)
  PARAM_ADD(PARAM_UINT32, sampledist, &mission_sampledist)
  PARAM_ADD(PARAM_UINT32, minvbat,    &mission_minvbat)
  PARAM_ADD(PARAM_UINT32, minobst,    &mission_minobst)
  PARAM_ADD(PARAM_UINT8,  guards,     &mission_guards)
PARAM_GROUP_STOP(mission)

// --- Log Registration ---
LOG_GROUP_START(tele)
  LOG_ADD(LOG_UINT16, alive, &tele_alive)
  LOG_ADD(LOG_UINT8,  canfly, &tele_canfly)
  LOG_ADD(LOG_UINT8,  clear,  &tele_clear)
  LOG_ADD(LOG_UINT16, maxz,   &tele_maxz)
  LOG_ADD(LOG_UINT16, samples, &tele_samples)
  LOG_ADD(LOG_UINT8,  endwhy, &tele_endwhy)
  LOG_ADD(LOG_UINT16, vbat,  &tele_vbat)
  LOG_ADD(LOG_UINT16, front, &tele_front)
  LOG_ADD(LOG_UINT16, back,  &tele_back)
  LOG_ADD(LOG_UINT16, left,  &tele_left)
  LOG_ADD(LOG_UINT16, right, &tele_right)
  LOG_ADD(LOG_UINT16, up,    &tele_up)
  LOG_ADD(LOG_UINT16, down,  &tele_down)
  LOG_ADD(LOG_INT16,  x,     &tele_x)
  LOG_ADD(LOG_INT16,  y,     &tele_y)
  LOG_ADD(LOG_INT16,  z,     &tele_z)
LOG_GROUP_STOP(tele)