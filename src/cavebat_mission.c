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
#include <math.h>

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
  // Which way the drone was pointing, in whole degrees, -180..180.
  //
  // Without this the six ranges cannot be placed on a map. "Front says 800mm"
  // means nothing on its own once the drone can turn -- it is 800mm north on
  // one sample and 800mm east two seconds later. The path was always drawable
  // from position alone, which is why this was not needed while yaw was pinned
  // at zero; the walls never were.
  //
  // Two bytes takes the sample to 14 and the download packet to 17, still
  // inside the 19 that fits one BLE notification.
  int16_t yaw;
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


// ============================================================================
// WALL FOLLOWING AND THE RETURN LEG
// ============================================================================
//
// This is the full life cycle: take off, find a wall, follow it out for half
// the timer, then retrace the route home and land, recording throughout.
//
// THREE DECISIONS, ALL MADE FOR SAFETY OVER ELEGANCE
//
// 1. The high-level commander does EVERYTHING. No velocity setpoints anywhere.
//    Wall following moves in discrete goTo steps and waits for each to finish.
//    A previous attempt at wall following swapped in velocity setpoints, and
//    the drone flipped on takeoff with four changes in flight at once and no
//    way to tell which was at fault. Stepping is less smooth than velocity
//    control and it does not matter: the drone samples once a second, and
//    holding still between steps makes those samples better, not worse.
//
// 2. Yaw stays at zero for the entire flight. The drone never rotates; it
//    strafes. That makes the body frame and the world frame the same thing, so
//    "front" really is +X and "right" really is -Y, the recorded path needs no
//    rotation to be drawn, and the return leg is exact.
//
//    The cost is real and worth stating plainly: a drone that cannot turn
//    cannot round a ninety-degree corner. It follows a wall that curves or
//    slants, and ends the outbound leg at one that cuts across it. For a first
//    flying version that is the right trade, and turning can be added later
//    against a baseline that works.
//
// 3. Breadcrumbs, not dead reckoning. Every completed step records where the
//    drone actually ended up, and the return flies those points in reverse.
//    "Retrace the route" is then literally what happens.
//
// WALL FOLLOWING IS OFF BY DEFAULT (mission.wallfollow = 0)
//
// With it off this firmware flies exactly like the version before it: take
// off, hover out the timer, land. So flashing this cannot regress a working
// drone -- verify the hover still works, THEN turn wall following on.

// 0 = hover in place for the timer, as before. 1 = seek and follow a wall.
static uint8_t mission_wallfollow = 0;

// Distance to hold from the wall, in mm, and how far it may drift before the
// drone corrects. A deadband matters: without one it strafes on every single
// step chasing sensor noise, and the path comes out as a zigzag.
static uint32_t mission_walldist = 400;
#define WALL_BAND_MM     120

// Beyond this a "wall" is too far to be the one we are following. The
// multiranger reads to about 3m; anything past 1.2m is the far side of a room,
// not the surface we are tracking.
#define WALL_MAX_MM     1200

// How far to move per step, forward and sideways. Small steps mean the drone
// re-reads its sensors often and can never commit far to a bad decision.
#define STEP_FWD_MM      200
#define STEP_SIDE_MM     150

// Something this close ahead stops the outbound leg advancing.
#define FRONT_STOP_MM    500

// GEOFENCE. The single most important number here. However confused the wall
// following gets, the drone turns for home once it is this far from where it
// started. Straight-line distance from the origin, in mm.
#define GEOFENCE_MM     4000

// Step speed, m/s. Matches the climb rate the takeoff already uses.
// Slowed from 0.3. Every metre now gets half as much again in sensor readings
// and gives the estimator more time to keep up, and nothing about this mission
// benefits from being quick.
#define STEP_SPEED_MS    0.2f

// --- Turning ---------------------------------------------------------------
//
// The first flying version held yaw at zero and strafed, which followed a wall
// that curved gently and gave up at a corner. Following walls "even if they
// turn left or right, inwards or outwards" needs the drone to actually turn,
// so it now steers with its heading the way a person walking a wall would.
//
// Three cases cover every corner:
//
//   wall ahead            -> the wall turns AWAY from the drone's right, an
//                            inward corner. Rotate left, on the spot, until
//                            the way ahead is clear.
//   wall gone on the right-> the wall turned out from under it, an outward
//                            corner. Rotate right and edge forward to come
//                            around it.
//   wall there but wrong  -> trim the heading in proportion to the error and
//     distance               keep flying. This is what tracks a curve.
//
// Rotating on the spot at a corner rather than while moving is deliberate: it
// keeps translation and rotation from being commanded at once, which is where
// a stepping controller would get untidy.

// Degrees turned per step at a corner. Small enough that the drone re-reads
// its sensors several times through a right angle instead of committing to
// the whole turn on one reading.
#define TURN_IN_DEG      25.0f
#define TURN_OUT_DEG     20.0f

// A corner turn is only believed after this many decisions in a row agree.
// Turning right means turning TOWARD the wall, so it is held to the stricter
// count: the cost of a wrong left turn is a wasted step, the cost of a wrong
// right turn is the wall.
#define CONFIRM_IN       2
#define CONFIRM_OUT      3
static uint8_t confirm_in = 0;
static uint8_t confirm_out = 0;

// The most the heading may be trimmed on a following step, and how hard to
// trim per millimetre of error. 200mm off target gives 15 degrees, so the
// drone converges over a few steps rather than swinging back and forth.
// Halved from 20. At 20 degrees a single reading 400mm off target swung the
// drone a fifth of a right angle, and with the wall only 400mm away there is
// no room for a correction that large to be wrong.
#define MAX_TRIM_DEG     10.0f
#define TRIM_DEG_PER_MM  0.04f

// Closer than this to the wall is an emergency, and the only allowed response
// is to turn away from it. No reading, however confident, may turn the drone
// toward a wall this close.
#define WALL_PANIC_MM    220

// Beyond this the wall on the right has gone, and the drone treats it as an
// outward corner rather than as a wall it is merely far from. Sits above
// WALL_MAX_MM so ordinary drift does not read as a corner.
#define WALL_LOST_MM    1400

// How far to creep forward while coming around an outward corner. Shorter than
// a normal step, because the drone is turning into space it cannot see yet.
#define STEP_CORNER_MM   150

#define DEG2RAD          0.017453292f
#define RAD2DEG          57.29578f

// The heading the drone is being flown at, radians. Tracked rather than read
// back, because this is the value being COMMANDED -- reading the estimate and
// steering from it would feed the estimator's own error back into the
// controller.
static float mission_yaw = 0.0f;

// Keep a heading in -pi..pi so it can be stored in an int16 of degrees and so
// repeated turning in one direction never runs away.
// Steering decisions are made from a MEDIAN of recent readings, never from one.
//
// This is why wall following flew into the wall. Every decision came from a
// single instantaneous reading and could command up to 25 degrees of turn, and
// a VL53L1x returns 0 both for "nothing in range" and for a read that simply
// failed. One failed read on the right-hand sensor was therefore indis-
// tinguishable from the wall ending, and the response to the wall ending is to
// turn RIGHT -- into the wall. Two or three of those in a row and the drone is
// pointed at the wall and flying.
//
// A median of five throws away any pair of outliers outright. It costs half a
// second of history, which is nothing against a step that takes about one.
#define RANGE_HIST 5
static uint16_t hist_front[RANGE_HIST];
static uint16_t hist_right[RANGE_HIST];
static uint8_t  hist_pos = 0;
static uint8_t  hist_fill = 0;

static void pushRanges(uint16_t f, uint16_t r) {
  hist_front[hist_pos] = f;
  hist_right[hist_pos] = r;
  hist_pos = (uint8_t)((hist_pos + 1) % RANGE_HIST);
  if (hist_fill < RANGE_HIST) hist_fill++;
}

static uint16_t medianOf(const uint16_t *buf) {
  uint16_t t[RANGE_HIST];
  uint8_t n = hist_fill;
  for (uint8_t i = 0; i < n; i++) t[i] = buf[i];
  for (uint8_t i = 1; i < n; i++) {          // insertion sort, n is 5
    uint16_t v = t[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  return n ? t[n / 2] : 0;
}

static float wrapYaw(float y) {
  while (y >  3.14159265f) y -= 6.28318531f;
  while (y < -3.14159265f) y += 6.28318531f;
  return y;
}

// A step that has not reported finished by this long past its planned duration
// is treated as finished anyway. Without it, one goTo that never completes
// would strand the drone in the air until the timer ran out.
#define STEP_GRACE_MS    1500

// Hold still for this long after a turn before flying anywhere.
//
// This is the one difference between the flights that work and the flights
// that do not. Hover has not crashed in a long time; wall following crashes
// about half the time. Hover never turns. Wall following turns.
//
// The Flow deck measures optical flow, and rotating over a floor produces
// apparent translation that the estimator has to unpick using the gyro. For a
// moment after a turn its position estimate is at its least trustworthy -- and
// the old code commanded the next translation immediately, from exactly that
// estimate. Every target is computed as "where I am, plus a step", so a
// position estimate disturbed by rotation sends the drone to the wrong place
// at speed.
//
// Standing still costs a second and lets the flow measurement settle before it
// is trusted again. Not proven: no log of a crash was ever captured, so this
// is reasoning from what distinguishes the two cases, not from a recording of
// one going wrong.
#define TURN_SETTLE_MS   1000

// A yaw change smaller than this is a trim, not a turn, and does not need the
// settle. Otherwise every following step would pause and the drone would take
// all day to get anywhere.
#define TURN_SETTLE_DEG  8.0f

static TickType_t settle_until = 0;
static float last_turn_deg = 0.0f;

// Breadcrumbs. 64 covers a 60-second outbound leg at roughly a step a second,
// with room to spare, for 256 bytes of RAM.
#define MAX_WAYPOINTS 64
typedef struct { int16_t x, y; } Waypoint;
static Waypoint waypoints[MAX_WAYPOINTS];
static uint16_t waypoint_count = 0;

// Mission phases. Published as tele.phase so the app can say what the drone is
// doing rather than only whether it is flying.
#define PHASE_IDLE      0
#define PHASE_CLIMB     1
#define PHASE_OUTBOUND  2
#define PHASE_RETURN    3
#define PHASE_LANDING   4
static uint8_t tele_phase = PHASE_IDLE;

// Why the outbound leg ended, published for the same reason.
//   0 = still going  1 = half the timer elapsed  2 = geofence
//   3 = blocked ahead with nowhere to go  4 = out of breadcrumb space
static uint8_t tele_outwhy = 0;

// The commanded heading in whole degrees, published so the phone-side recorder
// can capture it too. The drone's own recording carries yaw in every sample;
// without this the phone's copy of the same flight would still draw all its
// walls facing one way, and the two recordings of one flight would disagree.
static int16_t tele_yaw = 0;

// The step currently in the air, if any.
static bool       step_active = false;
static TickType_t step_deadline = 0;
static TickType_t outbound_deadline = 0;
static uint16_t   return_index = 0;

// Straight-line distance from the origin, for the geofence. Compares squared
// distances so there is no square root in the flight loop.
static bool beyondGeofence(void) {
  // 64-bit on purpose. tele_x and tele_y are millimetres in an int16, so each
  // square reaches 1.07e9 and the SUM can pass INT32_MAX. That needs a
  // diverged estimator to happen -- and a diverged estimator is exactly when
  // this check has to work, since it is the last thing standing between a
  // confused drone and the far end of the room. This one has already reported
  // a 6401mm altitude in a 500mm hover, so it is not a hypothetical.
  int64_t x = tele_x, y = tele_y;
  return (x * x + y * y) > ((int64_t)GEOFENCE_MM * (int64_t)GEOFENCE_MM);
}

// Issue one absolute goTo and arm the timeout that covers it.
//
// Absolute rather than relative on purpose. A relative goTo is relative to the
// commander's own setpoint, which is not necessarily where the drone actually
// is; over dozens of steps that difference accumulates and the breadcrumb
// trail stops matching the flight. Every target here is computed from the
// estimated position and sent in world coordinates.
static void issueStep(float tx_m, float ty_m, float tz_m, float yaw_rad) {
  float dx = tx_m - (tele_x / 1000.0f);
  float dy = ty_m - (tele_y / 1000.0f);
  float dist = sqrtf(dx * dx + dy * dy);
  float dur  = dist / STEP_SPEED_MS;
  if (dur < 0.7f) dur = 0.7f;   // a floor, or short hops are commanded violently
  // And a ceiling. Every target here is derived from the estimated position,
  // so if the estimate jumps the computed distance jumps with it, and without
  // this the drone would be commanded on one long uninterrupted flight to a
  // place it was never at. Capping the duration caps how far a single bad
  // reading can carry it before the sensors are consulted again.
  if (dur > 4.0f) dur = 4.0f;
  // Turning on the spot covers no distance, so the distance-derived duration
  // would be the 0.7s floor no matter how far it has to rotate. Give a turn
  // time proportional to its size instead, or the commander is asked to snap
  // round faster than the aircraft can follow.
  float dyaw = wrapYaw(yaw_rad - mission_yaw);
  if (dyaw < 0) dyaw = -dyaw;
  last_turn_deg = dyaw * RAD2DEG;
  float turn_dur = last_turn_deg / 25.0f;   // 25 deg/s, deliberately slow
  if (turn_dur > dur) dur = turn_dur;
  if (dur > 4.0f) dur = 4.0f;
  mission_yaw = wrapYaw(yaw_rad);

  crtpCommanderHighLevelGoTo(tx_m, ty_m, tz_m, yaw_rad, dur, false);
  step_active   = true;
  step_deadline = xTaskGetTickCount()
                + M2T((uint32_t)(dur * 1000.0f)) + M2T(STEP_GRACE_MS);
}

// True once the current step is done, or has taken so long that waiting
// further is worse than moving on.
static bool stepFinished(void) {
  if (!step_active) return true;
  if (crtpCommanderHighLevelIsTrajectoryFinished()) return true;
  if ((int32_t)(xTaskGetTickCount() - step_deadline) >= 0) {
    DEBUG_PRINT("CAVEBAT: step timed out, continuing\n");
    return true;
  }
  return false;
}

static void dropBreadcrumb(void) {
  if (waypoint_count < MAX_WAYPOINTS) {
    waypoints[waypoint_count].x = tele_x;
    waypoints[waypoint_count].y = tele_y;
    waypoint_count++;
  }
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
// Sends one packet without ever blocking indefinitely.
//
// crtpSendPacketBlock() waits forever for queue space. If the link is
// congested or has dropped, that hangs this task permanently -- and this task
// is the one running the mission and updating telemetry, so the whole drone
// goes silent and unreachable until it is power cycled. Retry a bounded number
// of times instead and give up, because a failed download is recoverable and a
// hung flight controller is not.
static bool sendPacketBounded(CRTPPacket *pk) {
  for (int attempt = 0; attempt < 20; attempt++) {
    if (crtpSendPacket(pk)) return true;
    vTaskDelay(M2T(5));   // let the radio drain
  }
  return false;
}

static void sendRecording(void) {
  DEBUG_PRINT("CAVEBAT: sending %d samples\n", (int)sample_count);

  // Zeroed, not left on the stack. CRTPPacket's header is a bitfield union, so
  // assigning .port and .channel leaves the reserved bits as whatever happened
  // to be on the stack -- a malformed header on every packet.
  CRTPPacket pk;
  memset(&pk, 0, sizeof(pk));

  for (uint16_t i = 0; i < sample_count; i++) {
    pk.port = CRTP_PORT_BULK;
    pk.channel = BULK_CHAN_DATA;
    pk.size = 2 + sizeof(FlightSample);
    pk.data[0] = i & 0xff;
    pk.data[1] = (i >> 8) & 0xff;
    memcpy(&pk.data[2], &flight_log[i], sizeof(FlightSample));
    if (!sendPacketBounded(&pk)) {
      DEBUG_PRINT("CAVEBAT: send stalled at sample %d, aborting\n", (int)i);
      return;   // no EOF: the app times out and keeps what arrived
    }
    // Pace the transfer. Filling the radio queue as fast as this loop can is
    // what makes it stall in the first place.
    vTaskDelay(M2T(5));
  }

  memset(&pk, 0, sizeof(pk));
  pk.port = CRTP_PORT_BULK;
  pk.channel = BULK_CHAN_CTRL;
  pk.size = 3;
  pk.data[0] = CMD_EOF;
  pk.data[1] = sample_count & 0xff;
  pk.data[2] = (sample_count >> 8) & 0xff;
  sendPacketBounded(&pk);
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
    tele_yaw   = (int16_t)(mission_yaw * RAD2DEG);
    // Sampled at the full 10Hz loop rate rather than once per step, so a step
    // that lasts a second is decided on ten readings instead of one.
    pushRanges(tele_front, tele_right);
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
        fs->yaw = (int16_t)(mission_yaw * RAD2DEG);
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

        // Reset the position estimator and let it settle BEFORE lifting off.
        //
        // This is why takeoff was a coin flip. Roughly half of flights flipped
        // within two seconds of leaving the ground -- and the giveaway was the
        // down-facing sensor reading 2413mm while the up-facing one read
        // nothing: the drone was inverted, looking at the ceiling. That flight
        // then reported climbing to 6401mm of a requested 500mm.
        //
        // It was NOT the battery. The crashed flight sagged 497mV and the good
        // one straight after it sagged 592mV, both from a full charge. Nor the
        // floor: every flight is flown over the same towel.
        //
        // The drone sits on the ground for tens of seconds between boot and
        // launch and the Kalman filter accumulates drift the whole time -- the
        // boot log prints "ESTKALMAN: State out of bounds, resetting" before it
        // has been asked to do anything at all. Taking off on that stale
        // estimate means the controller's first act can be a violent correction
        // toward a position the drone was never in. Sometimes the estimate
        // happens to be good and it flies. That is the coin flip.
        //
        // Every one of Bitcraze's own autonomous examples resets the estimator
        // and waits before flying. This firmware never did.
        {
          paramVarId_t resetId = paramGetVarId("kalman", "resetEstimation");
          if (PARAM_VARID_IS_VALID(resetId)) {
            paramSetInt(resetId, 1);
            vTaskDelay(M2T(100));
            paramSetInt(resetId, 0);
            // Convergence takes about a second with a Flow deck. Two is the
            // figure Bitcraze use, and it costs nothing but a pause on the pad.
            // Telemetry freezes for this long because this task is the one that
            // updates it. That is expected, not a stall.
            vTaskDelay(M2T(2000));
            DEBUG_PRINT("CAVEBAT: estimator reset, settled\n");
          } else {
            // Fly anyway. A missing parameter is a reason to warn, not to ground
            // the aircraft -- unreset is exactly how it behaved until now.
            DEBUG_PRINT("CAVEBAT: kalman.resetEstimation NOT FOUND, flying unreset\n");
          }
        }

        // Started after the settle, so the recording clock and the first sample
        // line up with the moment the drone actually leaves the ground.
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
        tele_phase = PHASE_CLIMB;
        tele_outwhy = 0;
        // Whatever way the drone is physically pointing becomes zero. Every
        // heading in the mission is relative to how it was placed, which is
        // what a pilot expects and what the estimator assumes.
        mission_yaw = 0.0f;
        confirm_in = 0;
        confirm_out = 0;
        hist_pos = 0;
        hist_fill = 0;
        settle_until = 0;
        last_turn_deg = 0.0f;
        waypoint_count = 0;
        step_active = false;
        climb_done_tick = xTaskGetTickCount()
                        + M2T((uint32_t)(takeoff_duration * 1000.0f));
        hover_deadline = xTaskGetTickCount()
                       + M2T((uint32_t)(takeoff_duration * 1000.0f))
                       + M2T(mission_timer * 1000);
        
    } else if (mission_state == 1 && is_flying) {
        // Peak altitude, so the flight can be judged after the fact.
        if (tele_z > 0 && (uint16_t)tele_z > tele_maxz) tele_maxz = (uint16_t)tele_z;

        // THE BACKSTOP, ABOVE EVERY OTHER RULE.
        //
        // The phase machine below decides when the mission is done. If it ever
        // fails to -- an unfinished trajectory, a wall that confuses the
        // follower, a bug not yet found -- this brings the drone down anyway.
        // Nothing else is allowed to keep it airborne past the timer plus a
        // margin generous enough that a healthy mission never sees it.
        //
        // A drone that will not land is the one failure this project cannot
        // recover from, so it gets a check that does not depend on any of the
        // logic that might be wrong.
        if ((int32_t)(xTaskGetTickCount() - flight_start_time)
              >= (int32_t)M2T(mission_timer * 1000 + 30000)) {
            if (tele_phase != PHASE_LANDING) {
                DEBUG_PRINT("CAVEBAT: HARD TIME LIMIT in phase %d, landing now\n",
                            (int)tele_phase);
                tele_phase = PHASE_LANDING;
            }
        }

        // --- Mission phases ------------------------------------------------
        //
        // Nothing here blocks. Every phase does a little work per 10Hz tick and
        // returns, so recording keeps sampling at 1Hz and the battery and
        // obstacle guards keep running no matter what the mission is doing.
        if (tele_phase == PHASE_CLIMB) {
            // Wait out the climb before doing anything clever. The estimator is
            // least trustworthy here and the guards are suppressed, so this is
            // the worst possible moment to start reacting to sensors.
            if ((int32_t)(xTaskGetTickCount() - climb_done_tick) >= 0) {
                if (mission_wallfollow) {
                    tele_phase = PHASE_OUTBOUND;
                    tele_outwhy = 0;
                    waypoint_count = 0;
                    // Half the timer out, half back. The return is never given
                    // less time than the outbound leg took.
                    outbound_deadline = xTaskGetTickCount()
                                      + M2T((mission_timer * 1000) / 2);
                    // Where we started, so the trail always ends at the pad.
                    dropBreadcrumb();
                    DEBUG_PRINT("CAVEBAT: outbound, wall following\n");
                } else {
                    // Wall following off: behave exactly as the previous
                    // firmware did. Hover until the timer expires.
                    tele_phase = PHASE_OUTBOUND;
                    tele_outwhy = 0;
                    outbound_deadline = hover_deadline;
                }
            }

        } else if (tele_phase == PHASE_OUTBOUND) {
            if (!mission_wallfollow) {
                // Plain hover. The high-level commander holds position on its
                // own once the takeoff trajectory finishes.
                if ((int32_t)(xTaskGetTickCount() - hover_deadline) >= 0) {
                    tele_outwhy = 1;
                    tele_phase = PHASE_LANDING;
                }
            } else if (!stepFinished()) {
                // A step is in the air. Let it land before deciding anything --
                // reading the sensors mid-move and re-commanding on top of an
                // unfinished trajectory is how a follower starts oscillating.
            } else {
                if (step_active) {
                    step_active = false;
                    dropBreadcrumb();
                    // A step that turned earns a pause before the next one.
                    if (last_turn_deg >= TURN_SETTLE_DEG) {
                        settle_until = xTaskGetTickCount() + M2T(TURN_SETTLE_MS);
                    }
                }

                // Standing still after a turn. The high-level commander holds
                // position on its own, so doing nothing here IS the hold.
                if ((int32_t)(xTaskGetTickCount() - settle_until) < 0) {
                    goto skip_step;
                }

                // Reasons to turn for home, checked before committing to
                // another step outward. Order matters: the geofence outranks
                // the clock, because too far is a safety limit while time is up
                // is only a plan.
                if (beyondGeofence()) {
                    tele_outwhy = 2;
                    DEBUG_PRINT("CAVEBAT: geofence at %d,%d mm, returning\n",
                                (int)tele_x, (int)tele_y);
                    tele_phase = PHASE_RETURN;
                } else if (waypoint_count >= MAX_WAYPOINTS) {
                    tele_outwhy = 4;
                    DEBUG_PRINT("CAVEBAT: breadcrumb trail full, returning\n");
                    tele_phase = PHASE_RETURN;
                } else if ((int32_t)(xTaskGetTickCount() - outbound_deadline) >= 0) {
                    tele_outwhy = 1;
                    DEBUG_PRINT("CAVEBAT: half timer, returning over %d points\n",
                                (int)waypoint_count);
                    tele_phase = PHASE_RETURN;
                } else {
                    // --- Decide one step ---------------------------------
                    //
                    // The drone steers rather than strafes, so everything here
                    // is relative to where it is currently pointing. Forward in
                    // the world is (cos yaw, sin yaw); the wall is on the
                    // right, 90 degrees clockwise from that.
                    //
                    // Every reading used here is a MEDIAN of the last half
                    // second, never a single sample. The first version steered
                    // on instantaneous values and flew into the wall, because a
                    // VL53L1x returning 0 for a failed read is indistinguish-
                    // able from one returning 0 for "nothing there", and the
                    // response to "nothing there" is to turn toward the wall.
                    float cx = tele_x / 1000.0f;
                    float cy = tele_y / 1000.0f;
                    float cz = mission_height / 1000.0f;

                    uint16_t f = medianOf(hist_front);
                    uint16_t r = medianOf(hist_right);

                    float newYaw = mission_yaw;
                    float advance = 0.0f;   // metres along the NEW heading

                    bool frontClose = (f > 0) && (f < FRONT_STOP_MM);
                    bool wallGone   = (r == 0) || (r > WALL_LOST_MM);
                    bool tooClose   = (r > 0) && (r < WALL_PANIC_MM);

                    // Streaks, so one odd decision cannot turn the aircraft.
                    if (frontClose) confirm_in++;  else confirm_in = 0;
                    if (wallGone)   confirm_out++; else confirm_out = 0;

                    if (tooClose) {
                        // TOO CLOSE. This outranks everything, including a
                        // wall detected ahead, because whatever else is true
                        // the drone is about to touch the thing it is
                        // following. Turn away, do not advance.
                        newYaw = wrapYaw(mission_yaw + TURN_IN_DEG * DEG2RAD);
                        advance = 0.0f;
                        confirm_in = 0;
                        confirm_out = 0;
                        DEBUG_PRINT("CAVEBAT: too close r=%d, turning away\n", (int)r);

                    } else if (confirm_in >= CONFIRM_IN) {
                        // INWARD CORNER. The wall has turned across the path,
                        // so the way on is to the left. Rotate on the spot and
                        // look again -- several small turns through a right
                        // angle, each re-read, rather than one blind ninety.
                        newYaw = wrapYaw(mission_yaw + TURN_IN_DEG * DEG2RAD);
                        advance = 0.0f;
                        DEBUG_PRINT("CAVEBAT: corner in, f=%d, turning left\n", (int)f);

                    } else if (confirm_out >= CONFIRM_OUT && waypoint_count > 1) {
                        // OUTWARD CORNER. The wall turned away from under the
                        // drone, so it follows it round by turning right.
                        //
                        // Turning right means turning TOWARD where the wall
                        // was, which is the one turn that can end in a
                        // collision, so it needs three agreeing decisions and
                        // it does not advance while turning. The step after
                        // this one either finds the wall again and resumes
                        // following, or turns again.
                        newYaw = wrapYaw(mission_yaw - TURN_OUT_DEG * DEG2RAD);
                        advance = 0.0f;
                        DEBUG_PRINT("CAVEBAT: corner out, turning right\n");

                    } else if (wallGone) {
                        // No wall on the right, and not yet confirmed as a
                        // corner. Hold the heading and fly on. This is both the
                        // search that opens the mission and the safe answer to
                        // a single dropped reading: carrying straight on cannot
                        // put the drone into a wall it has lost track of.
                        advance = STEP_FWD_MM / 1000.0f;

                    } else {
                        // FOLLOWING. Trim the heading in proportion to how far
                        // off the target distance the wall is, then fly on.
                        // Too far turns toward it, too close turns away.
                        //
                        // Proportional so a gentle curve gets a gentle
                        // correction, and capped hard: with the wall 400mm away
                        // there is no room for a large correction to be wrong.
                        float err = (float)r - (float)mission_walldist;
                        float trim = -err * TRIM_DEG_PER_MM;
                        if (trim >  MAX_TRIM_DEG) trim =  MAX_TRIM_DEG;
                        if (trim < -MAX_TRIM_DEG) trim = -MAX_TRIM_DEG;
                        newYaw = wrapYaw(mission_yaw + trim * DEG2RAD);
                        advance = STEP_FWD_MM / 1000.0f;
                    }

                    float nx = cx + advance * cosf(newYaw);
                    float ny = cy + advance * sinf(newYaw);
                    issueStep(nx, ny, cz, newYaw);
                }
                skip_step: ;
            }

            // Entering the return leg from any of the branches above: rewind to
            // the newest breadcrumb and start walking the trail backwards.
            if (tele_phase == PHASE_RETURN) {
                step_active = false;
                return_index = waypoint_count;   // decremented before first use
            }

        } else if (tele_phase == PHASE_RETURN) {
            if (!stepFinished()) {
                // let the current leg finish
            } else {
                step_active = false;
                if (return_index == 0) {
                    DEBUG_PRINT("CAVEBAT: home, landing\n");
                    tele_phase = PHASE_LANDING;
                } else {
                    return_index--;
                    // Heading held, not recomputed. The drone is retracing
                    // space it has just flown through, so there is nothing to
                    // look at that it has not already seen, and holding the
                    // heading keeps rotation out of the return entirely. It
                    // flies home sideways or backwards, which the aircraft
                    // does perfectly well, and every sample still carries the
                    // heading so the map stays correct.
                    issueStep(waypoints[return_index].x / 1000.0f,
                              waypoints[return_index].y / 1000.0f,
                              mission_height / 1000.0f,
                              mission_yaw);
                }
            }
        }

        if (tele_phase == PHASE_LANDING) {
            tele_endwhy = 1;
            DEBUG_PRINT("CAVEBAT: FLIGHT OK, peak %d of %d mm, %d samples, %d pts, why %d\n",
                        (int)tele_maxz, (int)mission_height, (int)sample_count,
                        (int)waypoint_count, (int)tele_outwhy);

            float target_height_m = mission_height / 1000.0f;
            float land_duration = target_height_m / 0.3f;
            if (land_duration < 1.0f) land_duration = 1.0f;

            crtpCommanderHighLevelLand(0.0f, land_duration);
            vTaskDelay(M2T((uint32_t)(land_duration * 1000) + 500));

            mission_state = 0; // Reset to idle
            is_flying = false;
            tele_phase = PHASE_IDLE;
            step_active = false;
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
                       sideBlocked(tele_left)  ||
                       // The right-hand sensor is excluded while wall
                       // following, because being close to the wall on the
                       // right IS the mission. Left in, this guard would abort
                       // every successful wall follow the moment it worked --
                       // a guard that fires on the intended behaviour.
                       (!mission_wallfollow && sideBlocked(tele_right))) {
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
        tele_phase = PHASE_LANDING;
        step_active = false;
        DEBUG_PRINT("CAVEBAT: FLIGHT ABORTED, peak %d mm of %d mm asked\n",
                    (int)tele_maxz, (int)mission_height);
        
        float target_height_m = mission_height / 1000.0f;
        float land_duration = target_height_m / 0.3f;
        if (land_duration < 1.0f) land_duration = 1.0f;
        
        crtpCommanderHighLevelLand(0.0f, land_duration);
        vTaskDelay(M2T((uint32_t)(land_duration * 1000) + 500));
        
        mission_state = 0; // Reset to idle
        is_flying = false;
        tele_phase = PHASE_IDLE;
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
  // 0 = hover out the timer exactly as the previous firmware did.
  // 1 = seek a wall, follow it for half the timer, retrace the route home.
  // Defaults to 0 so flashing this cannot regress a drone that already flies.
  PARAM_ADD(PARAM_UINT8,  wallfollow, &mission_wallfollow)
  PARAM_ADD(PARAM_UINT32, walldist,   &mission_walldist)
PARAM_GROUP_STOP(mission)

// --- Log Registration ---
LOG_GROUP_START(tele)
  LOG_ADD(LOG_UINT16, alive, &tele_alive)
  LOG_ADD(LOG_UINT8,  canfly, &tele_canfly)
  LOG_ADD(LOG_UINT8,  clear,  &tele_clear)
  LOG_ADD(LOG_UINT16, maxz,   &tele_maxz)
  LOG_ADD(LOG_UINT16, samples, &tele_samples)
  LOG_ADD(LOG_UINT8,  endwhy, &tele_endwhy)
  // What the drone is doing right now, so the app can say so:
  // 0 idle, 1 climbing, 2 outbound, 3 returning, 4 landing.
  LOG_ADD(LOG_UINT8,  phase,  &tele_phase)
  // Why the outbound leg ended:
  // 0 still going, 1 half the timer, 2 geofence, 3 blocked, 4 trail full.
  LOG_ADD(LOG_UINT8,  outwhy, &tele_outwhy)
  LOG_ADD(LOG_INT16,  yaw,    &tele_yaw)
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