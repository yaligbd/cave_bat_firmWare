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
#include "wallfollowing_multiranger_onboard.h"
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
// WALL FOLLOWING - SETTINGS
// ============================================================================
// Mission: take off, follow a wall on the RIGHT for half the timer, fly back
// along the same route, land. Records the whole time.
//
// How it moves: small hops ("steps") using goTo, never continuous speed
// commands. Each hop: read sensors, decide, move, wait until it arrives.
// Home: every hop drops a "breadcrumb" (its position). Going home = visiting
// the breadcrumbs in reverse.

// Flight mode, set by the app.
//   0 = hover in place (unchanged behaviour)
//   1 = follow a wall on the RIGHT
//   2 = follow a wall on the LEFT
//
// Left following is the exact mirror of right following, not a second copy of
// the logic. One sign variable flips every turn and every steering correction,
// so a fix to one is automatically a fix to the other and the two cannot drift
// apart.
static uint8_t mission_wallfollow = 0;

// Distance to keep from the wall, mm. Set by the app.
static uint32_t mission_walldist = 400;

// SAFETY: this far from the takeoff spot = stop and fly home. mm.
#define GEOFENCE_MM     4000

// Speed, m/s, for both the return hops and the wall follower. Slow on
// purpose: more sensor readings per metre travelled.
#define STEP_SPEED_MS    0.2f

// Unit conversions: degrees <-> radians.
#define DEG2RAD          0.017453292f
#define RAD2DEG          57.29578f

// --- How the wall gets followed ---------------------------------------------
//
// This is NOT our own controller any more. It is Bitcraze's, from
// wallfollowing_multiranger_onboard.c -- the one published with McGuire et al.,
// "Minimal navigation solution for a swarm of tiny flying robots to explore an
// unknown environment", Science Robotics 4(35), 2019. It has been flown on real
// swarms. We call it and fly what it says.
//
// WHY WE REPLACED OUR OWN, because this is the whole lesson of the project:
//
//   Our controller could only STEER. To fix its distance from the wall it had
//   to turn, and turning changes where it is going, so a small distance error
//   became a heading error, which produced a LARGER distance error a moment
//   later. The loop fed itself. Seven-second flights were fine because there
//   was no time for it to build up. A fourteen-second flight crept in to 20cm
//   and touched the wall. That is not a tuning problem, it is a controller
//   that was never stable -- it just diverged slower than the flights were
//   long.
//
//   Bitcraze's controller SLIDES SIDEWAYS to hold the distance while keeping
//   the nose pointed along the wall. Distance and heading stay independent, so
//   an error in one cannot grow the other.
//
// That difference is why this needs velocity setpoints and the old one did
// not: strafing is something you can only ask for as a velocity.
//
// ONE DELIBERATE DEVIATION from the Bitcraze demo: their loop decides at
// 100Hz. Ours decides at 10Hz, because this loop also carries the recording,
// the battery guard and the obstacle guard, all of which are tuned for 10Hz
// and none of which should be disturbed. The setpoint is still REFRESHED at
// 100Hz (see the foot of the main loop), so the aircraft is never flying an
// aging command. The correction is bang-bang at half of WF_SPEED_MS, so a
// 10Hz decision overshoots by at most 1cm before it is reconsidered -- well
// inside the 10cm deadband the controller works to.
#define WF_SPEED_MS      0.2f

// Drop a breadcrumb every time the drone has moved this far since the last
// one. The old controller dropped one per completed hop; there are no hops
// any more, so distance is what marks the trail now.
#define WF_CRUMB_MM      300

// True while velocity setpoints are driving the aircraft. It gates the 100Hz
// refresh at the foot of the loop, and it is how the return leg knows it has
// to hand control back to the high-level commander.
static bool vel_active = false;

// The one setpoint struct, reused. Held at file scope because the 100Hz
// refresh re-sends it without rebuilding it.
static setpoint_t wf_setpoint;

// Which state the follower is in, published so a crash log says what the
// aircraft was doing rather than leaving us to guess again.
static StateWF wf_state = forward;
static uint8_t tele_wfstate = 0;

// The sensors the follower reads. File scope because it runs outside the
// block where appMain keeps its own log handles.
static logVarId_t wf_idFront, wf_idLeft, wf_idRight, wf_idYaw;

// Where the last breadcrumb was dropped, mm.
static int16_t crumb_x = 0, crumb_y = 0;

// The heading the RETURN leg is holding, in radians. Bookkeeping for
// issueStep, which needs to know the heading it last commanded so it can work
// out how long a turn should take. 0 = the way the drone faced at takeoff.
static float mission_yaw = 0.0f;

// The heading the drone ACTUALLY has, in radians, read from the estimator.
//
// This is what gets recorded, not mission_yaw. Under the wall follower the
// heading is commanded as a RATE, so there is no commanded angle to record --
// and a measured heading is the better thing to draw the map from anyway,
// because it is what the aircraft really did.
static float meas_yaw = 0.0f;

// Keep an angle between -180 and +180 degrees (in radians), so turning the
// same way many times never grows the number forever.
static float wrapYaw(float y) {
  while (y >  3.14159265f) y -= 6.28318531f;
  while (y < -3.14159265f) y += 6.28318531f;
  return y;
}

// SAFETY: if a hop hasn't finished this long after it should have, move on
// anyway. Stops one stuck hop from leaving the drone hanging in the air.
#define STEP_GRACE_MS    1500

static float last_turn_deg = 0.0f;

// --- Breadcrumbs --------------------------------------------------------------
// One saved position per hop. 64 is enough for about a minute of flying out.
#define MAX_WAYPOINTS 64
typedef struct { int16_t x, y; } Waypoint;
static Waypoint waypoints[MAX_WAYPOINTS];
static uint16_t waypoint_count = 0;

// --- Mission phase (the app can read this) ----------------------------------
#define PHASE_IDLE      0
#define PHASE_CLIMB     1
#define PHASE_OUTBOUND  2
#define PHASE_RETURN    3
#define PHASE_LANDING   4
static uint8_t tele_phase = PHASE_IDLE;

// Why the drone stopped going out and turned home (the app can read this):
//   0 = still going  1 = half the timer  2 = geofence
//   3 = blocked      4 = out of breadcrumbs
static uint8_t tele_outwhy = 0;

// Current heading in whole degrees, for the app.
static int16_t tele_yaw = 0;

// The hop currently in progress, if any, and its time limits.
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
  crumb_x = tele_x;
  crumb_y = tele_y;
}

// --- Flying on velocity, for the wall follower ------------------------------
//
// Height stays an ABSOLUTE setpoint while x, y and yaw are velocities. That
// mixture is what lets the follower strafe and rotate without ever having an
// opinion about altitude, which the barometer and the down ranger hold on
// their own. Copied from Bitcraze's own demo, deliberately unchanged.
//
// velocity_body = true means vx is "forward" and vy is "left" as the DRONE is
// pointing, not as the room is laid out. The follower thinks entirely in its
// own frame, so anything else would need a rotation here and would be one more
// place to get a sign wrong.
//
// Priority 3 beats the high-level commander's 1, so the first of these calls
// takes the aircraft off the trajectory planner automatically.
static void sendBodyVelocity(float vx, float vy, float z_m, float yawRateDeg) {
  memset(&wf_setpoint, 0, sizeof(wf_setpoint));
  wf_setpoint.mode.z   = modeAbs;       wf_setpoint.position.z = z_m;
  wf_setpoint.mode.yaw = modeVelocity;  wf_setpoint.attitudeRate.yaw = yawRateDeg;
  wf_setpoint.mode.x   = modeVelocity;  wf_setpoint.mode.y = modeVelocity;
  wf_setpoint.velocity.x = vx;          wf_setpoint.velocity.y = vy;
  wf_setpoint.velocity_body = true;
  commanderSetSetpoint(&wf_setpoint, 3);
}

// Run the follower once and fly what it says.
//
// The ranges handed over are the RAW log values in metres, NOT tele_front and
// friends. That is not a shortcut, it is required: clampRange() turns anything
// out of range into 0, and this controller reads 0 as "the wall is touching my
// nose" and turns immediately. The driver reports 32767mm for nothing-in-range,
// which divided by 1000 is 32.7 metres -- exactly the "no wall anywhere" the
// controller expects, and what Bitcraze's own demo feeds it.
//
// direction is +1 with the wall on the RIGHT and -1 with it on the LEFT, which
// is the pairing Bitcraze's demo uses with the matching sensor.
static void wfTick(void) {
  float frontRange = safeLogFloat(wf_idFront) / 1000.0f;
  float sideRange  = (mission_wallfollow == 2 ? safeLogFloat(wf_idLeft)
                                              : safeLogFloat(wf_idRight)) / 1000.0f;
  float yawRad     = safeLogFloat(wf_idYaw) * DEG2RAD;
  int   direction  = (mission_wallfollow == 2) ? -1 : 1;

  // The follower measures its own timeouts in seconds against this, so it has
  // to keep counting up across the whole flight. Ticks since boot, as seconds.
  float now_s = (float)xTaskGetTickCount() / (float)configTICK_RATE_HZ;

  float vx = 0.0f, vy = 0.0f, yawRateRad = 0.0f;
  wf_state = wallFollower(&vx, &vy, &yawRateRad,
                          frontRange, sideRange, yawRad, direction, now_s);
  tele_wfstate = (uint8_t)wf_state;

  sendBodyVelocity(vx, vy, mission_height / 1000.0f, yawRateRad * RAD2DEG);
}

// Give the aircraft back to the high-level commander.
//
// MUST be called before any goTo or land that follows wall following, and this
// is the one piece of the port that is a safety matter rather than a behaviour
// one. Velocity setpoints are sent at priority 3 and the high-level commander
// sits at 1, so while the 100Hz refresh is running a land command is accepted
// and then immediately overridden -- the drone would simply keep flying.
// Clearing vel_active stops the refresh; commanderRelaxPriority() then tells
// the planner the live state estimate and lowers the priority so its own
// setpoints take effect again.
static void releaseToHighLevel(void) {
  if (!vel_active) return;
  vel_active = false;
  // Stop moving, and hold still long enough to actually be stopped. Relaxing
  // mid-slide hands over a velocity the planner does not know it must arrest.
  sendBodyVelocity(0.0f, 0.0f, mission_height / 1000.0f, 0.0f);
  vTaskDelay(M2T(400));
  commanderRelaxPriority();
  // Whatever heading it ended up on becomes the heading to hold from here.
  mission_yaw = wrapYaw(safeLogFloat(wf_idYaw) * DEG2RAD);
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
  logVarId_t idYaw   = logGetVarId("stabilizer", "yaw");

  // The same handles again at file scope, for the wall follower. It runs from
  // the 100Hz refresh at the foot of the loop, outside this function's scope.
  wf_idFront = idFront;
  wf_idLeft  = idLeft;
  wf_idRight = idRight;
  wf_idYaw   = idYaw;

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
    // The heading the aircraft really has. Recorded, and used to draw the map.
    meas_yaw   = safeLogFloat(idYaw) * DEG2RAD;
    tele_yaw   = (int16_t)(meas_yaw * RAD2DEG);
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
        fs->yaw = (int16_t)(meas_yaw * RAD2DEG);
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
        meas_yaw = 0.0f;
        last_turn_deg = 0.0f;
        waypoint_count = 0;
        step_active = false;
        // Every flight starts the follower from scratch. It keeps its state
        // machine, its heading reference and its corner bookkeeping in statics,
        // so a second flight on a warm drone would otherwise begin halfway
        // through the previous one's corner.
        vel_active = false;
        wf_state = forward;
        tele_wfstate = 0;
        crumb_x = 0;
        crumb_y = 0;
        if (mission_wallfollow) {
          wallFollowerInit(mission_walldist / 1000.0f, WF_SPEED_MS, forward);
        }
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
                    DEBUG_PRINT("CAVEBAT: outbound, following wall on the %s\n",
                                mission_wallfollow == 2 ? "LEFT" : "RIGHT");
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
            } else {
                // --- Following a wall ------------------------------------
                //
                // No hops and no waiting for a trajectory to finish. The
                // aircraft is under continuous velocity control, so every tick
                // simply asks the follower what to do now and does it.
                //
                // The reasons to turn for home are checked FIRST, and they are
                // checked every tick rather than only between hops, so the
                // geofence now catches a runaway within 100ms instead of
                // whenever the current hop happened to end. Order matters: the
                // geofence outranks the clock, because too far is a safety
                // limit while time is up is only a plan.
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
                    // Fly the wall. wfTick() reads the sensors, runs Bitcraze's
                    // state machine and sends the setpoint; the refresh at the
                    // foot of the loop keeps that setpoint alive at 100Hz.
                    vel_active = true;
                    wfTick();

                    // Mark the trail by distance travelled. The return leg
                    // retraces these, so they have to be close enough together
                    // that flying straight between two of them cannot cut a
                    // corner the drone went round.
                    // 64-bit for the same reason beyondGeofence() is: these are
                    // millimetres from an estimator that has already been seen
                    // to report 6401mm in a 500mm hover, and a squared
                    // difference of two diverged readings passes INT32_MAX.
                    int64_t dx = (int64_t)tele_x - (int64_t)crumb_x;
                    int64_t dy = (int64_t)tele_y - (int64_t)crumb_y;
                    if (dx * dx + dy * dy
                          >= (int64_t)WF_CRUMB_MM * (int64_t)WF_CRUMB_MM) {
                        dropBreadcrumb();
                    }

                    // One line per second, not per tick.
                    //
                    // Three sessions of wall-following crashes were lost
                    // because nothing recorded what the aircraft was thinking
                    // in the two seconds before it went over. This is that
                    // record -- and it is short and slow on purpose, because a
                    // long line printed ten times a second queues behind
                    // itself on a 20-byte BLE link and arrives after the crash
                    // it was meant to explain.
                    //
                    // st is the follower's state: 0 forward, 1 hover,
                    // 2 turnToFindWall, 3 turnToAlignToWall, 4 forwardAlongWall,
                    // 5 rotateAroundWall, 6 rotateInCorner, 7 findCorner.
                    if ((tele_alive % 10) == 0) {
                        DEBUG_PRINT("WF%c st=%d f=%d s=%d\n",
                                    (mission_wallfollow == 2) ? 'L' : 'R',
                                    (int)tele_wfstate,
                                    (int)tele_front,
                                    (int)(mission_wallfollow == 2 ? tele_left
                                                                  : tele_right));
                    }
                }
            }

            // Entering the return leg from any of the branches above: rewind to
            // the newest breadcrumb and start walking the trail backwards.
            if (tele_phase == PHASE_RETURN) {
                // Hand the aircraft back to the trajectory planner first.
                //
                // The return leg flies goTo hops, which belong to the
                // high-level commander -- but the follower has just spent the
                // whole outbound leg overriding it at a higher priority, and
                // the planner's idea of where the drone is has been frozen
                // since takeoff. Handing back without saying so would make the
                // first hop start from the takeoff pad and fly the difference
                // as fast as it could.
                //
                // commanderRelaxPriority() is the firmware's own answer to
                // exactly this: it tells the planner the live state estimate
                // and drops the priority in one go, so the next goTo plans
                // from where the aircraft actually is.
                releaseToHighLevel();
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
            // Before anything else: a land command is worthless while velocity
            // setpoints are still being refreshed over the top of it.
            releaseToHighLevel();
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
            // The sensor pointed at the wall being followed is excluded,
            // because being close to that wall IS the mission. Left in, this
            // guard would abort every successful wall follow the moment it
            // started working. The other three sides still count.
            } else if (sideBlocked(tele_front) || sideBlocked(tele_back) ||
                       (mission_wallfollow != 2 && sideBlocked(tele_left)) ||
                       (mission_wallfollow != 1 && sideBlocked(tele_right))) {
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
        releaseToHighLevel();   // or the land below is overridden, see above
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
    
    // The loop stays at 10Hz. Everything above is tuned for it: the guards
    // count ticks, the heartbeat divides by it, and the app expects telemetry
    // at this rate.
    //
    // But a velocity setpoint cannot be left to age for 100ms -- the aircraft
    // goes on obeying the last one it was given, so at 10Hz it would fly each
    // command in 10cm lurches. So the delay is spent in ten 10ms slices and the
    // setpoint is re-sent on each, which is the 100Hz Bitcraze's follower was
    // proven at. commanderSetSetpoint() re-stamps the timestamp, so re-sending
    // the same struct is exactly what keeps it alive.
    //
    // While not wall following this is one unchanged 100ms delay, so the hover
    // path behaves precisely as it did before.
    for (uint8_t i = 0; i < 10; i++) {
      vTaskDelay(M2T(10));
      if (vel_active) {
        commanderSetSetpoint(&wf_setpoint, 3);
      }
    }
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
  // NOT exposed: tele_wfstate. The follower's state goes out in the WF trace
  // line instead. Adding a parameter changes the parameter table, which
  // invalidates the app's cached copy and needs KNOWN_PARAM_NAMES updated to
  // match -- a mismatch there already cost one flight. The console line
  // carries the same information at no such cost.
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
  // 1 = follow a wall on the RIGHT for half the timer, then retrace home.
  // 2 = the same, following a wall on the LEFT.
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