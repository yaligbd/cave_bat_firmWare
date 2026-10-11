// cavebat_mission.c - CaveBat mission app, the only appMain in this build.
//
// One flight per mission.state = 1 request. Phases, as published in tele.phase:
//   0 IDLE      on the ground. On a fly request: pre-flight checks (only if
//               mission.guards = 1), estimator reset (about 2.1 s, blocking),
//               high-level takeoff
//   1 CLIMB     wait out the takeoff trajectory
//   2 OUTBOUND  hover mode (wallfollow 0): hold position for the timer, then
//               land. Wall mode (1 right, 2 left): follow the wall with
//               wallfollowing_corners for half the timer, dropping breadcrumbs
//   5 BRAKE     wall mode: ramp the forward speed to zero over 800 ms
//   3 RETURN    wall mode: fly the breadcrumbs back in reverse, heading held
//   4 LANDING   hand back to the high-level commander, land, back to IDLE
// Aborts (phone, over-tilt, battery, optional obstacle) and a hard time limit
// (timer + 30 s) end any flight with a landing.
//
// Timing: the main loop decides at 10 Hz. While velocity setpoints are in use
// (wall-mode OUTBOUND, BRAKE, RETURN) the setpoint is re-sent every 10 ms
// during the loop's wait, a 100 Hz refresh.
//
// Phone interface: the app writes mission.timer, height, sampledist and
// wallfollow, then mission.state (1 = fly, 2 = abort). walldist, minvbat,
// minobst and guards are tunables with safe defaults. Status goes back in the
// tele.* log group (some also as params): position, the six ranges, battery,
// yaw, canfly, clear, samples, phase, outwhy, endwhy and maxz.
//
// Recorder: one 16-byte FlightSample per second while flying (up to 180, in
// RAM), downloaded on request over CRTP port 14 (see "Download protocol").
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
#include "wallfollowing_corners.h"
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

// --- Mission parameters (mission.*) ---
static uint8_t  mission_state = 0; // 0=Idle, 1=Fly, 2=Abort
// Seconds. Hover mode: hover time after the climb. Wall mode: the outbound
// leg gets half of it; the return is bounded only by the hard time limit.
static uint32_t mission_timer = 10;
static uint32_t mission_height = 500; // Flight altitude in mm
static uint32_t mission_sampledist = 10; // cm; unused (recording is 1 Hz)

// --- Battery safety ---
// Resting voltage below which takeoff is refused (only with
// mission.guards = 1). A Crazyflie 2.x LiPo is ~4.2 V full and ~3.0 V empty;
// below about 3.7 V at rest it lacks the headroom to climb, which looks like a
// weak takeoff rather than a flat battery.
static uint32_t mission_minvbat = 3700;   // mV, resting threshold, tunable
// The in-flight cutoff is minvbat minus this margin, because voltage sags hard
// under motor load. Spin-up sag can exceed 500 mV (3890 mV at rest has been
// seen at 3383 mV in the climb), so a 300 mV margin is too tight; the check is
// also suppressed until the climb is done. 3700 - 600 = 3100 mV is still above
// the ~3.0 V where a cell is empty.
#define VBAT_INFLIGHT_MARGIN_MV 600

// 1 = battery at or above minvbat, 0 = too low. Published so the app can show
// it before a flight starts.
static uint8_t  tele_canfly = 0;

// --- Obstacle safety ---
// Clearance in mm on front, back, left and right below which takeoff is
// refused and an in-flight abort is triggered (both only with
// mission.guards = 1). Tunable because 200 mm is easy to trip indoors: a desk
// edge, a chair, or the pilot standing nearby.
static uint32_t mission_minobst = 200;

// 1 = nothing inside mission_minobst, 0 = something is. Published so the app
// can say why the drone will not fly.
static uint8_t  tele_clear = 0;

// Master switch for the pre-flight refusals (battery, obstacle) and the
// in-flight obstacle abort. Off by default, because these have fired on false
// positives and ended healthy flights (a side ranger seeing the floor during
// the climb, normal LiPo sag). tele_canfly and tele_clear are still published
// either way, as information.
//
// Not affected by this switch: the in-flight battery abort, the over-tilt
// abort, the hard time limit, and the stock supervisor (tumble detection,
// critical-battery cutoff).
static uint8_t  mission_guards = 0;

// Peak altitude of the last flight in mm, and how that flight ended:
//   0 = never flown
//   1 = landed normally (hover timer done, home reached, or hard time limit)
//   2 = aborted (phone, over-tilt, battery or obstacle)
static uint16_t tele_maxz   = 0;
static uint8_t  tele_endwhy = 0;

// 0 means "no reading" (out of range) and must not count as an obstacle,
// otherwise open space would read as blocked.
static bool sideBlocked(uint16_t mm) {
  return (mm > 0) && (mm < (uint16_t)mission_minobst);
}

// logGetFloat() asserts on an invalid id, which halts the drone. A variable
// whose deck driver is not built, or whose deck did not initialise, yields
// such an id. This returns 0 instead, so the firmware runs with or without the
// Multi-ranger fitted.
static float safeLogFloat(logVarId_t id) {
  return logVarIdIsValid(id) ? logGetFloat(id) : 0.0f;
}

// --- Onboard flight recorder ----------------------------------------------
//
// Size limit: a download packet is 1 CRTP header byte + 2 index bytes +
// sizeof(FlightSample) and must be at most 19 bytes, so the BLE bridge sends it
// as one 20-byte notification (the nRF51 s130 stack caps ATT_MTU at 23, and
// split packets arrive corrupted). The sample is 16 bytes, so the packet is
// exactly 19: there is no room to grow this struct.
//
// That budget is why the ranges are single bytes in 2 cm units (0-5.1 m, past
// the Multi-ranger's ~3 m limit) while position stays in full millimetres.
// Packed, in native STM32 (little-endian) order: the field order is the wire
// format the app parses.
typedef struct __attribute__((packed)) {
  int16_t x, y, z;                              // position, mm
  // Measured heading in whole degrees, -180..180. Needed to place the ranges
  // on a map once the drone turns.
  int16_t yaw;
  uint8_t front, back, left, right, up, down;   // ranges, 2cm units, 0 = none

  // Follower state, one nibble each:
  //   low  4 bits  follower step, CF_* from wallfollowing_corners.h (0 = has
  //                not run this flight)
  //   high 4 bits  flight mode, mission.wallfollow (0 hover, 1 right, 2 left)
  // Sampled before the state machine runs, so the step is the previous tick's.
  // The follower only runs in OUTBOUND, so during BRAKE and RETURN the step
  // stays at the last outbound value.
  //
  // Recorded rather than printed: console text is dropped when the 20-byte BLE
  // link is busy with log blocks, while the recording is downloaded later over
  // a quiet link.
  uint8_t wf;

  // Worst of |roll| and |pitch| since the previous sample, in 2-degree units,
  // saturating at 255. A peak rather than a snapshot, because a flip is over
  // well within the one-second sample interval. Tilt climbing over several
  // samples points to the controller fighting a bad position estimate; level
  // and then suddenly large points to an impact.
  uint8_t tilt;
} FlightSample;

// 1 Hz sampling, so 180 samples is three minutes; once full, further samples
// are dropped. 180 * 16 = 2880 bytes of static RAM.
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
// Wall mode: after the climb, follow a wall for half the timer while dropping
// a breadcrumb every WF_CRUMB_MM, brake, then fly the breadcrumbs back in
// reverse and land. All of it is flown on body-frame velocity setpoints.

// Flight mode, set by the app:
//   0 = hover in place
//   1 = follow a wall on the RIGHT
//   2 = follow a wall on the LEFT
//
// Left is the mirror of right, not a second copy of the logic: wfTick() reads
// the other side ranger and passes direction = -1 to the follower, which flips
// every turn and steering correction.
static uint8_t mission_wallfollow = 0;

// Distance to keep from the wall, mm. Tunable; read once, at takeoff.
static uint32_t mission_walldist = 400;

// SAFETY: horizontal distance from the takeoff point (mm) at which the
// outbound leg ends and the drone turns home. Checked only in wall-mode
// OUTBOUND.
#define GEOFENCE_MM     4000

// RETURN speed toward each breadcrumb, m/s. Slow on purpose: more sensor
// readings per metre travelled.
#define STEP_SPEED_MS    0.2f

// Close enough to a breadcrumb to call it reached, mm.
//
// Crumbs are at least WF_CRUMB_MM apart (equal to this, 150 mm, at present).
// A radius larger than the spacing would let one arrival satisfy several
// crumbs and cut corners on the way home.
#define RETRACE_ARRIVE_MM  150

// Give up on a breadcrumb not reached within this time, ms, and aim at the
// next one. The obstacle clamp can hold the drone off a crumb; every crumb
// leads home, so skipping one costs little.
#define RETRACE_STUCK_MS  5000
static TickType_t crumb_since = 0;   // when the current RETURN target was set

// Unit conversions: degrees <-> radians.
#define DEG2RAD          0.017453292f
#define RAD2DEG          57.29578f

// --- How the wall gets followed ---------------------------------------------
//
// The follower is wallfollowing_corners.c: a script of discrete steps (CF_*
// codes in wallfollowing_corners.h). It holds the wall distance by strafing
// sideways with the nose along the wall, and yaws only in 90-degree turns on
// the spot. Correcting distance by steering instead couples distance and
// heading errors so that each grows the other; strafing keeps them independent,
// and strafing needs velocity setpoints.
//
// The follower decides at 10 Hz, in the main loop alongside the recorder and
// the guards (all tuned for 10 Hz). Its setpoint is re-sent every 10 ms at the
// foot of the loop, so the commander never flies a stale command.
//
// Forward cruise speed handed to the follower, m/s.
#define WF_SPEED_MS      0.2f

// Ceiling on the commanded yaw rate, deg/s.
//
// Yawing over the Flow deck corrupts the position estimate: the optical-flow
// sensor sees the floor sweep past and the estimator reads part of it as
// translation, which the controller then turns into real drift. Near a wall
// that drift is a collision, so turns are kept slow.
//
// The corners follower already turns slowly (30 deg/s in a corner turn, at
// most 10 deg/s of heading hold while following), so this 45 deg/s cap does
// not engage with it. It is a backstop applied outside the follower; see
// wfTick().
#define WF_MAX_YAWRATE_DEG  45.0f

// How fast the commanded velocity may change, m/s^2. Used by the ramps in
// wfTick() and RETURN.
#define WF_ACCEL_MS2        0.4f

// The ramped command, carried between ticks. Reset at each takeoff, or a
// second flight on a warm drone begins at the speed the last one ended on.
static float vx_cmd = 0.0f;
static float vy_cmd = 0.0f;

// Drop a breadcrumb whenever the drone is this far (mm) from the previous one.
// The return flies straight lines between crumbs, so this spacing is the
// resolution of the path home; at 300 mm the return cut across corners the
// outbound leg had flown round.
#define WF_CRUMB_MM      150

// True while velocity setpoints are driving the aircraft. Gates the 100 Hz
// refresh at the foot of the loop, and tells releaseToHighLevel() there is
// control to hand back before landing.
static bool vel_active = false;

// The one setpoint struct, reused. Held at file scope because the 100Hz
// refresh re-sends it without rebuilding it.
static setpoint_t wf_setpoint;

// The follower's current step (CF_* code). tele_wfstate is its copy for the
// recorder and the WF console line: 0 until the follower first runs in a
// flight. It is not a tele.* param or log variable (see the param table).
static int wf_state = CF_FOLLOW;
static uint8_t tele_wfstate = 0;

// Worst tilt since the last sample, 2-degree units. See FlightSample.tilt.
static uint8_t tilt_peak = 0;

// Consecutive ticks with roll or pitch beyond OVER_TILT_DEG. At that angle the
// aircraft is down, not flying (the stock supervisor normally cuts the motors
// first), so the flight is ended through the abort path rather than recording
// a drone lying on the floor until the timer runs out.
#define OVER_TILT_DEG    70.0f
#define OVER_TILT_TICKS  10      // one second at 10Hz
static uint8_t over_ticks = 0;

// Copies of appMain's log handles, for the helpers outside it (wfTick,
// clampAwayFromObstacles, commandedHeight, releaseToHighLevel).
static logVarId_t wf_idFront, wf_idLeft, wf_idRight, wf_idYaw, wf_idBack, wf_idUp;

// --- Obstacle clamp (clampAwayFromObstacles) --------------------------------
//
// The last layer on the velocity command, in wall-mode OUTBOUND and in RETURN.
// Within WF_STOP_MM it only removes motion toward an obstacle; within
// WF_PUSH_MM it backs away. It never invents a manoeuvre of its own (no turns,
// no handover), so it cannot oscillate against the follower or fight it for
// control.
//
// 300 mm is below the follower's working distances with the defaults (400 mm
// off the wall; a corner is called at walldist + 150 mm = 550 mm ahead), so in
// normal following it stays out of the way. It catches what the follower gets
// wrong, such as a corner that never triggered.
// Stop moving towards anything inside this, on any of the four sides.
#define WF_STOP_MM       300

// Inside this, actively back away at WF_PUSH_MS instead of only stopping. A
// drone that merely stops commanding motion toward a wall keeps drifting
// toward it on momentum and estimator error. Applied to the final command, so
// it holds in every follower step, including corner turns where nothing else
// watches the front. Gentle on purpose: it only has to beat a slow drift, and
// a hard shove away from one wall can put the drone into another.
#define WF_PUSH_MM       200
#define WF_PUSH_MS       0.12f

// A low ceiling pushes the flight down this far at most, mm. Anything more and
// a bad up-reading could fly the aircraft into the floor.
#define WF_CEIL_DROP_MM  250

// Where the ceiling starts to matter, mm.
#define WF_CEIL_MM       400

// Defined further down, next to the rest of the velocity control; declared here
// because dropBreadcrumb() records the height that is actually being flown.
static float commandedHeight(void);

// Where the last breadcrumb was dropped, mm.
static int16_t crumb_x = 0, crumb_y = 0;

// Tick at which the braking phase began, so the ramp knows how far along it is.
static TickType_t brake_start = 0;

// Consecutive in-flight ticks on which all six tele ranges read 0 ("no
// reading"). One sensor reading 0 is open space, but all of them at once means
// the ranging hardware has stopped answering (the Multi-ranger's I2C expander
// has failed this way), and the follower would see open space everywhere and
// fly on blind. In wall-mode OUTBOUND this ends the leg (tele.outwhy = 7).
//
// The down range is part of the test because there is always a floor. Note
// that it comes from the Flow deck (range.zrange), not the Multi-ranger, so a
// failed Multi-ranger with a working Flow deck does not by itself trip this.
static uint8_t deck_silent_ticks = 0;
#define DECK_SILENT_LIMIT 10   // one second at 10Hz

// The forward speed the brake ramps down from: vx_cmd at the moment the
// outbound leg ends, which can be well below WF_SPEED_MS (for example when
// stopped mid-corner). Ramping from WF_SPEED_MS instead would command a
// forward lurch.
static float brake_v0 = 0.0f;

// Heading recorded when BRAKE ends and again in releaseToHighLevel(), radians
// (0 = the heading at takeoff). For reference only; nothing reads it. RETURN
// holds its heading by commanding a yaw rate of 0.
static float mission_yaw = 0.0f;

// Measured heading (stabilizer.yaw) in radians, updated every tick. Recorded
// in each sample and used by RETURN to turn world-frame directions into the
// body frame. Under the follower the heading is commanded as a rate, so there
// is no commanded angle to record.
static float meas_yaw = 0.0f;

// Keep an angle between -180 and +180 degrees (in radians), so turning the
// same way many times never grows the number forever.
static float wrapYaw(float y) {
  while (y >  3.14159265f) y -= 6.28318531f;
  while (y < -3.14159265f) y += 6.28318531f;
  return y;
}

// --- Breadcrumbs --------------------------------------------------------------
//
// The trail home: x, y and the commanded height (mm) every WF_CRUMB_MM of
// outbound travel, crumb 0 at the start of the leg. RETURN uses only x and y;
// its height comes from commandedHeight() at the time, so the stored z is
// currently unused.
//
// 128 crumbs at 150 mm or more apart cover at least 19 m of outbound path
// (96 s at 0.2 m/s); a full trail ends the leg (tele.outwhy = 4). 768 bytes of
// static RAM.
#define MAX_WAYPOINTS 128
typedef struct { int16_t x, y, z; } Waypoint;
static Waypoint waypoints[MAX_WAYPOINTS];
static uint16_t waypoint_count = 0;

// --- Mission phase (tele.phase) ---------------------------------------------
#define PHASE_IDLE      0
#define PHASE_CLIMB     1
#define PHASE_OUTBOUND  2
#define PHASE_RETURN    3
#define PHASE_LANDING   4
// Wall mode only, between OUTBOUND and RETURN: ramp the forward speed to zero
// over BRAKE_MS. A quadrotor stops by pitching, which costs vertical thrust;
// an instant stop on a sagging pack loses height and has rolled the aircraft
// over. It is a phase rather than a blocking wait, so the recorder and the
// tilt watch keep running through it.
#define PHASE_BRAKE     5

// How long to spend slowing down. Long enough that the aircraft never has to
// pitch hard to lose 0.2 m/s.
#define BRAKE_MS        800
static uint8_t tele_phase = PHASE_IDLE;

// Why the outbound leg ended (tele.outwhy):
//   0 = still going    1 = timer (half the timer in wall mode, the full hover
//   time in hover mode)    2 = geofence    4 = breadcrumb trail full
//   7 = ranger deck silent
// 3 ("blocked") is not set by this build.
static uint8_t tele_outwhy = 0;

// Measured heading in whole degrees, for the app.
static int16_t tele_yaw = 0;

// Tick at which the outbound leg ends, and the breadcrumb RETURN is flying to.
static TickType_t outbound_deadline = 0;
static uint16_t   return_index = 0;

// Straight-line distance from the origin (the takeoff point, after the
// estimator reset), for the geofence. Compares squared distances so there is
// no square root in the flight loop.
static bool beyondGeofence(void) {
  // 64-bit on purpose: tele_x and tele_y are int16 mm, so each square reaches
  // 1.07e9 and the sum can exceed INT32_MAX. That takes a diverged estimate
  // (6401 mm has been reported in a 500 mm hover), which is exactly when the
  // geofence has to work.
  int64_t x = tele_x, y = tele_y;
  return (x * x + y * y) > ((int64_t)GEOFENCE_MM * (int64_t)GEOFENCE_MM);
}

// Append the current position to the trail (if there is room) and make it
// the reference point for the next drop.
static void dropBreadcrumb(void) {
  if (waypoint_count < MAX_WAYPOINTS) {
    waypoints[waypoint_count].x = tele_x;
    waypoints[waypoint_count].y = tele_y;
    // The height actually being flown, which a low ceiling may have lowered.
    waypoints[waypoint_count].z = (int16_t)(commandedHeight() * 1000.0f);
    waypoint_count++;
  }
  crumb_x = tele_x;
  crumb_y = tele_y;
}

// --- Flying on velocity (wall-mode OUTBOUND, BRAKE, RETURN) -----------------
//
// Height is an absolute setpoint while x, y and yaw are velocities, so the
// follower can strafe and rotate without commanding altitude.
//
// velocity_body = true means vx is forward and vy is left as the drone is
// pointing, not as the room is laid out. The follower works in its own frame,
// so no rotation is needed here.
//
// Priority 3 outranks the high-level commander, so the first of these calls
// takes the aircraft off the trajectory planner; releaseToHighLevel() gives it
// back.
static void sendBodyVelocity(float vx, float vy, float z_m, float yawRateDeg) {
  memset(&wf_setpoint, 0, sizeof(wf_setpoint));
  wf_setpoint.mode.z   = modeAbs;       wf_setpoint.position.z = z_m;
  wf_setpoint.mode.yaw = modeVelocity;  wf_setpoint.attitudeRate.yaw = yawRateDeg;
  wf_setpoint.mode.x   = modeVelocity;  wf_setpoint.mode.y = modeVelocity;
  wf_setpoint.velocity.x = vx;          wf_setpoint.velocity.y = vy;
  wf_setpoint.velocity_body = true;
  commanderSetSetpoint(&wf_setpoint, 3);
}

// Cancel any commanded motion that heads into something close, and back away
// from anything very close. See WF_STOP_MM and WF_PUSH_MM.
//
// Body frame: vx forward, vy LEFT. So a positive vy is checked against the
// left sensor and a negative one against the right.
//
// Only motion toward an obstacle is removed; motion away is untouched. When
// the follower is hard against its wall it strafes away from it, and a clamp
// that blocked that would trap the aircraft against the wall.
//
// Reads the raw log ranges in mm. The driver reports 32767 for nothing in
// range, so a real 0 is almost unheard of; but safeLogFloat() returns 0 for a
// sensor that is not fitted, and treating that as an obstacle would freeze a
// Multi-ranger-less aircraft in place. So 0 counts as no reading.
static void clampAwayFromObstacles(float *vx, float *vy) {
  float f = safeLogFloat(wf_idFront);
  float b = safeLogFloat(wf_idBack);
  float l = safeLogFloat(wf_idLeft);
  float r = safeLogFloat(wf_idRight);

  // 0 = no reading (see above), not an obstacle.
  bool fNear = f > 0.0f && f < (float)WF_STOP_MM;
  bool bNear = b > 0.0f && b < (float)WF_STOP_MM;
  bool lNear = l > 0.0f && l < (float)WF_STOP_MM;
  bool rNear = r > 0.0f && r < (float)WF_STOP_MM;

  // First: never move towards anything close.
  if (*vx > 0.0f && fNear) *vx = 0.0f;
  if (*vx < 0.0f && bNear) *vx = 0.0f;
  if (*vy > 0.0f && lNear) *vy = 0.0f;
  if (*vy < 0.0f && rNear) *vy = 0.0f;

  // Then: if something is closer still, back away from it. See WF_PUSH_MM.
  //
  // The opposite side is checked before pushing: in a gap narrower than two
  // push distances both sides are close at once, and backing away from one
  // means flying into the other. Pinned on both sides, that axis holds still.
  bool fPush = f > 0.0f && f < (float)WF_PUSH_MM;
  bool bPush = b > 0.0f && b < (float)WF_PUSH_MM;
  bool lPush = l > 0.0f && l < (float)WF_PUSH_MM;
  bool rPush = r > 0.0f && r < (float)WF_PUSH_MM;

  if (fPush && !bPush)      *vx = -WF_PUSH_MS;   // wall ahead, ease back
  else if (bPush && !fPush) *vx =  WF_PUSH_MS;
  else if (fPush && bPush)  *vx =  0.0f;

  if (lPush && !rPush)      *vy = -WF_PUSH_MS;   // wall to the left, ease right
  else if (rPush && !lPush) *vy =  WF_PUSH_MS;
  else if (lPush && rPush)  *vy =  0.0f;

  // And halve forward speed while a side push is active. Easing sideways at
  // 0.12 m/s while flying forward at 0.2 m/s is a 31-degree crab, which can
  // carry the drone diagonally into a corner at full speed. Slower forward
  // flight cuts the crab angle and shortens the stopping distances that follow.
  if ((lPush || rPush) && *vx > 0.0f) *vx *= 0.5f;
}

// How high to fly right now, in metres: mission.height, lowered by
// (WF_CEIL_MM - up) when the up ranger reads under WF_CEIL_MM. The drop is
// capped at WF_CEIL_DROP_MM so one bad reading cannot drive the aircraft into
// the floor. Used for every velocity setpoint.
static float commandedHeight(void) {
  float h = mission_height / 1000.0f;
  float up = safeLogFloat(wf_idUp);
  if (up > 0.0f && up < (float)WF_CEIL_MM) {
    float drop = ((float)WF_CEIL_MM - up) / 1000.0f;
    if (drop > (float)WF_CEIL_DROP_MM / 1000.0f) {
      drop = (float)WF_CEIL_DROP_MM / 1000.0f;
    }
    h -= drop;
  }
  return h;
}

// One follower tick (10 Hz, wall-mode OUTBOUND only): run the corners follower,
// cap the yaw rate, clamp against obstacles, ramp, and send the setpoint.
//
// The follower gets the raw log ranges in metres, not tele_*: the driver's
// 32767 mm for nothing in range arrives as 32.7 m, which the follower treats as
// open space (it also treats 0 or less as open). direction is +1 with the wall
// on the right and -1 with it on the left.
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
  wf_state = wallFollowerCorners(&vx, &vy, &yawRateRad,
                                 frontRange, sideRange, yawRad, direction, now_s);
  tele_wfstate = (uint8_t)wf_state;

  // Cap the rotation, and slow the translation by the same factor so that any
  // arc (a turn rate paired with forward speed) keeps its radius instead of
  // running wide. See WF_MAX_YAWRATE_DEG; with the corners follower (30 deg/s
  // at most, and on the spot) the cap does not engage.
  float yawRateDeg = yawRateRad * RAD2DEG;
  float yawMag = yawRateDeg < 0.0f ? -yawRateDeg : yawRateDeg;
  if (yawMag > WF_MAX_YAWRATE_DEG) {
    float scale = WF_MAX_YAWRATE_DEG / yawMag;
    yawRateDeg *= scale;
    vx *= scale;
    vy *= scale;
  }

  // Last word before the setpoint goes out: never fly into anything.
  // Applied to the target, so the ramp below decelerates into the stop rather
  // than snapping to it.
  clampAwayFromObstacles(&vx, &vy);

  // Ease onto the new speed rather than stepping onto it. The controller
  // answers a step change in velocity by pitching hard, which shows as a dip
  // in height. The follower relies on this too: CF_STOP commands zero and waits
  // CF_STOP_S (0.7 s); 0.4 m/s^2 takes the cruise speed to zero in 0.5 s,
  // inside that wait.
  float maxStep = WF_ACCEL_MS2 * 0.1f;     // decisions run at 10Hz
  float dvx = vx - vx_cmd;
  float dvy = vy - vy_cmd;
  if (dvx >  maxStep) dvx =  maxStep;
  if (dvx < -maxStep) dvx = -maxStep;
  if (dvy >  maxStep) dvy =  maxStep;
  if (dvy < -maxStep) dvy = -maxStep;
  vx_cmd += dvx;
  vy_cmd += dvy;

  sendBodyVelocity(vx_cmd, vy_cmd, commandedHeight(), yawRateDeg);
}

// Give the aircraft back to the high-level commander.
//
// SAFETY: must run before any land command that follows velocity flight.
// Velocity setpoints go out at priority 3, above the high-level commander, so
// while the 100 Hz refresh runs a land command is accepted and then overridden
// and the drone keeps flying. Clearing vel_active stops the refresh;
// commanderRelaxPriority() then hands the planner the live state estimate and
// lowers the priority so its setpoints take effect again. No-op in hover mode.
static void releaseToHighLevel(void) {
  if (!vel_active) return;
  vel_active = false;
  // One last command to hold still, so the aircraft is stopped by an
  // instruction rather than by the supervisor: a setpoint older than 500 ms
  // trips supervisorStateWarningLevelOut, which disables x and y control and
  // forces the aircraft level.
  //
  // No blocking wait here, so the recorder and the tilt watch never go blind.
  // The drone may still be moving (RETURN flies right up to crumb 0, and an
  // abort or the hard time limit can come at any point), so this zero
  // setpoint is the stop.
  sendBodyVelocity(0.0f, 0.0f, commandedHeight(), 0.0f);
  commanderRelaxPriority();
  // Final heading, for reference (see mission_yaw).
  mission_yaw = wrapYaw(safeLogFloat(wf_idYaw) * DEG2RAD);
}

// --- Download protocol, CRTP port 14 --------------------------------------
// App -> drone, channel 0: 0x01 = send the recording, 0x02 = clear it.
// Drone -> app: one packet per sample on channel 1, [idx_lo, idx_hi,
// FlightSample], then [0xFF, count_lo, count_hi] on channel 0. A stalled send
// ends the transfer without the EOF packet.
#define CRTP_PORT_BULK  14
#define BULK_CHAN_CTRL  0
#define BULK_CHAN_DATA  1
#define CMD_DUMP_START  0x01   // app -> drone: send me the recording
#define CMD_CLEAR_MEM   0x02   // app -> drone: discard it
#define CMD_EOF         0xFF   // drone -> app: that was the last sample

static bool dump_requested  = false;
static bool clear_requested = false;

// Runs in CRTP task context, so it only raises a flag. The transfer itself
// runs in the main loop.
static void bulkCrtpCb(CRTPPacket *pk) {
  if (pk->channel == BULK_CHAN_CTRL && pk->size >= 1) {
    if (pk->data[0] == CMD_DUMP_START) dump_requested = true;
    if (pk->data[0] == CMD_CLEAR_MEM)  clear_requested = true;
  }
}

// Send one packet, retrying up to 20 times 5 ms apart, then give up.
//
// crtpSendPacketBlock() waits forever for queue space. On a congested or
// dropped link that would hang this task, which also runs the mission and the
// telemetry, until a power cycle. A failed download is recoverable; a hung
// flight task is not.
static bool sendPacketBounded(CRTPPacket *pk) {
  for (int attempt = 0; attempt < 20; attempt++) {
    if (crtpSendPacket(pk)) return true;
    vTaskDelay(M2T(5));   // let the radio drain
  }
  return false;
}

// Send the recording, one sample per packet. The index is included so the app
// can tell a dropped packet from a shifted one rather than mis-aligning every
// later sample. Blocks this loop for at least 5 ms per sample.
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

// Log range in mm -> tele_* value. 0 means no reading: <= 0, or beyond 3000 mm
// (which includes the driver's 32767 for nothing in range).
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
  // Roll and pitch: for the over-tilt abort, the tilt peak in each sample and
  // the TILT console line.
  logVarId_t idRoll  = logGetVarId("stabilizer", "roll");
  logVarId_t idPitch = logGetVarId("stabilizer", "pitch");

  // The same handles at file scope, for the helpers called from the 10 Hz
  // state machine (wfTick, the obstacle clamp, commandedHeight,
  // releaseToHighLevel).
  wf_idFront = idFront;
  wf_idLeft  = idLeft;
  wf_idRight = idRight;
  wf_idYaw   = idYaw;
  wf_idBack  = idBack;
  wf_idUp    = idUp;

  DEBUG_PRINT("CAVEBAT: Flight & Telemetry starting\n");
  
  uint32_t flight_start_time = 0;
  uint32_t hover_deadline = 0;  // tick at which a hovering flight should land
  uint8_t  obstacle_streak = 0; // consecutive cycles seeing an obstacle
  uint8_t  lowbat_streak = 0;   // consecutive cycles seeing a collapsed battery
  // Tick at which the climb finishes: CLIMB ends here, and the in-flight
  // battery and obstacle aborts are suppressed until then. While climbing the
  // drone tilts and the side rangers catch the floor (sides have been seen
  // flicking between 32766 and ~450 mm with nothing in the way), and spin-up
  // battery sag is at its worst.
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
    // Uses the previous tick's ranges (refreshed just below), a one-tick lag.
    tele_clear = (sideBlocked(tele_front) || sideBlocked(tele_back) ||
                  sideBlocked(tele_left)  || sideBlocked(tele_right)) ? 0 : 1;
    tele_front = clampRange(safeLogFloat(idFront));
    tele_back  = clampRange(safeLogFloat(idBack));
    tele_left  = clampRange(safeLogFloat(idLeft));
    tele_right = clampRange(safeLogFloat(idRight));
    tele_up    = clampRange(safeLogFloat(idUp));
    tele_down  = clampRange(safeLogFloat(idDown));
    // The heading the aircraft really has: recorded, and used by RETURN.
    meas_yaw   = safeLogFloat(idYaw) * DEG2RAD;
    tele_yaw   = (int16_t)(meas_yaw * RAD2DEG);
    tele_x     = (int16_t)(safeLogFloat(idX) * 1000.0f);
    tele_y     = (int16_t)(safeLogFloat(idY) * 1000.0f);
    tele_z     = (int16_t)(safeLogFloat(idZ) * 1000.0f);

    // --- Tilt and deck watch (while flying, every mode) ----------------------
    //
    // Over-tilt abort, deck-silent counter, tilt peak for the recorder, and a
    // TILT console line at the full 10 Hz whenever roll or pitch exceeds 15
    // degrees. A healthy flight stays under 15, so the line costs nothing on
    // the 20-byte BLE link until something goes wrong. Reading it: tilt
    // climbing over several lines means the controller is winding up against
    // a bad estimate; level, then over within one line, points to a motor, a
    // prop or a sudden estimator jump.
    if (is_flying) {
      float roll_now  = safeLogFloat(idRoll);
      float pitch_now = safeLogFloat(idPitch);
      if (roll_now < 0) roll_now = -roll_now;
      if (pitch_now < 0) pitch_now = -pitch_now;
      // Over on its side or back? See over_ticks. Fires once, when the count
      // reaches OVER_TILT_TICKS with mission.state = 1; the abort branch of
      // the state machine below then runs in this same tick.
      if (roll_now > OVER_TILT_DEG || pitch_now > OVER_TILT_DEG) {
        if (over_ticks < 255) over_ticks++;
      } else {
        over_ticks = 0;
      }
      if (over_ticks == OVER_TILT_TICKS && mission_state == 1) {
        DEBUG_PRINT("CAVEBAT: over on its side, ending the flight\n");
        tele_endwhy = 2;
        mission_state = 2;   // the ordinary abort path: stop, tidy up, idle
      }

      // Have all six ranges gone silent? See deck_silent_ticks.
      if (tele_front == 0 && tele_back == 0 && tele_left == 0 &&
          tele_right == 0 && tele_up == 0 && tele_down == 0) {
        if (deck_silent_ticks < 255) deck_silent_ticks++;
      } else {
        deck_silent_ticks = 0;
      }

      // Worst tilt this second, held until the next sample takes it. Checked
      // every tick because a tumble is over long before the next sample.
      {
        float worst = (roll_now > pitch_now) ? roll_now : pitch_now;
        uint16_t units = (uint16_t)(worst / 2.0f);
        if (units > 255) units = 255;
        if ((uint8_t)units > tilt_peak) tilt_peak = (uint8_t)units;
      }
      if (roll_now > 15.0f || pitch_now > 15.0f) {
        DEBUG_PRINT("TILT r=%d p=%d z=%d\n",
                    (int)safeLogFloat(idRoll), (int)safeLogFloat(idPitch),
                    (int)tele_z);
      }
    }

    // Clear and dump requests, flagged by bulkCrtpCb(). A dump only reads the
    // buffer, but it blocks this loop (at least 5 ms per sample), so it is
    // meant for the ground; nothing here refuses it in flight.
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

    // One sample per second while is_flying, taken before the state machine
    // runs this tick. Nothing is recorded during the blocking land wait.
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
        // The follower's step and the flight mode, one nibble each.
        fs->wf    = (uint8_t)((mission_wallfollow & 0x0f) << 4)
                  | (uint8_t)(tele_wfstate & 0x0f);
        fs->tilt  = tilt_peak;
        tilt_peak = 0;   // each sample owns the second before it
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
        // Something is already inside minobst. Taking off would trip the
        // in-flight obstacle abort as soon as the climb ends, which looks like
        // a broken controller rather than an obstacle. Refuse up front and say
        // so.
        DEBUG_PRINT("CAVEBAT: Takeoff REFUSED, obstacle within %d mm (f=%d b=%d l=%d r=%d)\n",
                    (int)mission_minobst, (int)tele_front, (int)tele_back,
                    (int)tele_left, (int)tele_right);
        mission_state = 0;

    } else if (mission_state == 1 && !is_flying) {
        // Takeoff requested, and guards off or both pre-flight checks passed.
        DEBUG_PRINT("CAVEBAT: Initiating Takeoff to %d mm, battery %d mV\n",
                    (int)mission_height, (int)tele_vbat);

        // Reset the Kalman estimator and let it settle before lifting off.
        // tele.phase stays IDLE during this.
        //
        // The estimate drifts while the drone sits on the pad between boot and
        // launch. Taking off on a stale estimate makes the controller's first
        // act a violent correction toward a position the drone is not at,
        // which has flipped the aircraft within seconds of leaving the ground.
        // Bitcraze's autonomous examples reset and wait the same way.
        {
          paramVarId_t resetId = paramGetVarId("kalman", "resetEstimation");
          if (PARAM_VARID_IS_VALID(resetId)) {
            paramSetInt(resetId, 1);
            vTaskDelay(M2T(100));
            paramSetInt(resetId, 0);
            // Convergence takes about a second with a Flow deck; 2 s is the
            // figure Bitcraze use. Telemetry freezes for this long because
            // this task is the one that updates it. Expected, not a stall.
            vTaskDelay(M2T(2000));
            DEBUG_PRINT("CAVEBAT: estimator reset, settled\n");
          } else {
            // Fly anyway: a missing parameter is a reason to warn, not to
            // ground the aircraft.
            DEBUG_PRINT("CAVEBAT: kalman.resetEstimation NOT FOUND, flying unreset\n");
          }
        }

        // Taken after the settle, so the hard time limit and the recording
        // count from the moment the drone actually leaves the ground.
        flight_start_time = xTaskGetTickCount();
        is_flying = true;
        // Each takeoff starts a fresh recording, discarding any samples not
        // yet downloaded. Takeoff never waits for a download.
        sample_count = 0;
        tele_samples = 0;
        last_sample_tick = xTaskGetTickCount();
        
        float target_height_m = mission_height / 1000.0f;
        float takeoff_duration = target_height_m / 0.3f; // safe velocity 0.3 m/s
        if (takeoff_duration < 1.0f) takeoff_duration = 1.0f;
        
        crtpCommanderHighLevelTakeoff(target_height_m, takeoff_duration);

        // Per-flight resets, so nothing carries over from the previous flight.
        obstacle_streak = 0;
        lowbat_streak = 0;
        tele_maxz = 0;
        tele_endwhy = 0;
        tele_phase = PHASE_CLIMB;
        tele_outwhy = 0;
        // Headings are relative to the takeoff heading, which the estimator
        // reset makes 0. meas_yaw is re-read from the estimator next tick.
        mission_yaw = 0.0f;
        meas_yaw = 0.0f;
        waypoint_count = 0;
        vel_active = false;
        wf_state = CF_FOLLOW;
        vx_cmd = 0.0f;
        vy_cmd = 0.0f;
        brake_v0 = 0.0f;
        deck_silent_ticks = 0;
        over_ticks = 0;
        tele_wfstate = 0;
        tilt_peak = 0;
        crumb_x = 0;
        crumb_y = 0;
        if (mission_wallfollow) {
          // Reset the follower's own statics (step, held heading, corner
          // bookkeeping), or a second flight on a warm drone would resume the
          // previous one mid-corner. It starts in CF_FOLLOW, because the drone
          // is placed with the wall already beside it and open space ahead. If
          // the wall is further than walldist + 0.4 m, FOLLOW sees the side as
          // gone and goes to CF_PAST, which handles it as an outward corner:
          // a short drive on, then a 90-degree turn toward the wall side.
          wallFollowerCornersInit(mission_walldist / 1000.0f, WF_SPEED_MS);
          wf_state = CF_FOLLOW;
        }
        climb_done_tick = xTaskGetTickCount()
                        + M2T((uint32_t)(takeoff_duration * 1000.0f));
        // Hover time counts from the end of the climb, so a short timer never
        // expires before the drone reaches altitude.
        hover_deadline = xTaskGetTickCount()
                       + M2T((uint32_t)(takeoff_duration * 1000.0f))
                       + M2T(mission_timer * 1000);
        
    } else if (mission_state == 1 && is_flying) {
        // Peak altitude, so the flight can be judged after the fact.
        if (tele_z > 0 && (uint16_t)tele_z > tele_maxz) tele_maxz = (uint16_t)tele_z;

        // SAFETY: hard time limit, checked before the phases. At timer + 30 s
        // after takeoff, land whatever the phase machine is doing (a confused
        // follower, a return that never reaches home, an unknown bug). It
        // depends on none of the phase logic, because a drone that will not
        // land is the one failure that cannot be recovered. Normal landings
        // and this one both report tele.endwhy = 1.
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
        // None of these phases blocks: each does one tick of work and returns,
        // so the recorder keeps sampling at 1 Hz and the guards keep running.
        // LANDING, handled after this chain in the same tick, does block.
        if (tele_phase == PHASE_CLIMB) {
            // Wait out the climb before doing anything clever. The estimator is
            // least trustworthy here and the guards are suppressed, so this is
            // the worst possible moment to start reacting to sensors.
            if ((int32_t)(xTaskGetTickCount() - climb_done_tick) >= 0) {
                if (mission_wallfollow) {
                    tele_phase = PHASE_OUTBOUND;
                    tele_outwhy = 0;
                    waypoint_count = 0;
                    // Outbound gets half the timer, counted from the end of the
                    // climb. RETURN has no deadline of its own; the hard time
                    // limit bounds it.
                    outbound_deadline = xTaskGetTickCount()
                                      + M2T((mission_timer * 1000) / 2);
                    // Where we started, so the trail always ends at the pad.
                    dropBreadcrumb();
                    DEBUG_PRINT("CAVEBAT: outbound, following wall on the %s\n",
                                mission_wallfollow == 2 ? "LEFT" : "RIGHT");
                } else {
                    // Hover mode: OUTBOUND just waits for hover_deadline
                    // (climb end + timer). No BRAKE or RETURN follows.
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
                // Every tick, the reasons to turn for home are checked first,
                // in priority order: deck silent, geofence, trail full, half
                // timer. Each one goes to BRAKE, so a runaway is caught within
                // one tick (~100 ms). The geofence outranks the clock: too far
                // is a safety limit, time up is only a plan. Otherwise run one
                // follower tick and drop a breadcrumb if one is due.
                if (deck_silent_ticks >= DECK_SILENT_LIMIT) {
                    // All ranges silent (see deck_silent_ticks): following is
                    // impossible, so stop and fly the breadcrumbs home on the
                    // position estimate.
                    tele_outwhy = 7;
                    DEBUG_PRINT("CAVEBAT: ranger deck silent 1s, returning\n");
                    tele_phase = PHASE_BRAKE;
                    brake_start = xTaskGetTickCount();
                    brake_v0 = vx_cmd;
                } else if (beyondGeofence()) {
                    tele_outwhy = 2;
                    DEBUG_PRINT("CAVEBAT: geofence at %d,%d mm, returning\n",
                                (int)tele_x, (int)tele_y);
                    tele_phase = PHASE_BRAKE;
                    brake_start = xTaskGetTickCount();
                    brake_v0 = vx_cmd;   // ramp down from the real speed
                } else if (waypoint_count >= MAX_WAYPOINTS) {
                    tele_outwhy = 4;
                    DEBUG_PRINT("CAVEBAT: breadcrumb trail full, returning\n");
                    tele_phase = PHASE_BRAKE;
                    brake_start = xTaskGetTickCount();
                    brake_v0 = vx_cmd;   // ramp down from the real speed
                } else if ((int32_t)(xTaskGetTickCount() - outbound_deadline) >= 0) {
                    tele_outwhy = 1;
                    DEBUG_PRINT("CAVEBAT: half timer, returning over %d points\n",
                                (int)waypoint_count);
                    tele_phase = PHASE_BRAKE;
                    brake_start = xTaskGetTickCount();
                    brake_v0 = vx_cmd;   // ramp down from the real speed
                } else {
                    // Fly the wall. wfTick() is the only place the follower
                    // runs: it reads the sensors, runs one step of the corners
                    // follower and sends the setpoint, which the foot of the
                    // loop re-sends at 100 Hz.
                    vel_active = true;
                    wfTick();

                    // Drop a breadcrumb every WF_CRUMB_MM of travel. 64-bit
                    // for the same reason as beyondGeofence(): the squared
                    // difference of two diverged readings exceeds INT32_MAX.
                    int64_t dx = (int64_t)tele_x - (int64_t)crumb_x;
                    int64_t dy = (int64_t)tele_y - (int64_t)crumb_y;
                    if (dx * dx + dy * dy
                          >= (int64_t)WF_CRUMB_MM * (int64_t)WF_CRUMB_MM) {
                        dropBreadcrumb();
                    }

                    // WF console trace, one line every 10 ticks (~1 s). Short
                    // and slow on purpose: a long line at 10 Hz queues behind
                    // itself on the 20-byte BLE link and arrives late.
                    //   st    follower step (CF_* in wallfollowing_corners.h):
                    //         1 follow, 2 stop, 3 turn, 4 verify, 5 past,
                    //         6 reacq, 7 gave up, 8 back off
                    //   f, s  front and followed-side range, mm (0 = none)
                    //   r, p  roll and pitch, degrees
                    if ((tele_alive % 10) == 0) {
                        DEBUG_PRINT("WF%c st=%d f=%d s=%d r=%d p=%d\n",
                                    (mission_wallfollow == 2) ? 'L' : 'R',
                                    (int)tele_wfstate,
                                    (int)tele_front,
                                    (int)(mission_wallfollow == 2 ? tele_left
                                                                  : tele_right),
                                    (int)safeLogFloat(idRoll),
                                    (int)safeLogFloat(idPitch));
                    }
                }
            }

        } else if (tele_phase == PHASE_BRAKE) {
            // --- Slowing down before flying home -------------------------
            //
            // Ramp forward speed linearly from brake_v0 to zero over BRAKE_MS
            // (see PHASE_BRAKE), with vy and yaw rate at 0, spread over
            // ordinary 10 Hz ticks. No obstacle clamp is applied here, and
            // vx_cmd/vy_cmd are not updated, so RETURN's ramp starts from the
            // pre-brake command.
            int32_t elapsed = (int32_t)(xTaskGetTickCount() - brake_start);
            if (elapsed < (int32_t)M2T(BRAKE_MS)) {
                float f = 1.0f - ((float)elapsed / (float)M2T(BRAKE_MS));
                if (f < 0.0f) f = 0.0f;
                vel_active = true;
                sendBodyVelocity(brake_v0 * f, 0.0f,
                                 commandedHeight(), 0.0f);
            } else {
                // Stopped: start the return at the newest crumb. There is no
                // handover here. RETURN flies on velocity setpoints like the
                // outbound leg, so it keeps the obstacle clamp, and control
                // goes back to the high-level commander only for the landing.
                return_index = waypoint_count;
                if (return_index > 0) return_index--;
                crumb_since = xTaskGetTickCount();
                // RETURN holds the current heading (yaw rate 0) and flies
                // backwards and sideways as needed: not yawing protects the
                // flow-based position estimate the crumbs are expressed in.
                // mission_yaw records that heading for reference only.
                mission_yaw = wrapYaw(safeLogFloat(wf_idYaw) * DEG2RAD);
                tele_phase = PHASE_RETURN;
                DEBUG_PRINT("CAVEBAT: stopped, home over %d pts\n",
                            (int)waypoint_count);

            }

        } else if (tele_phase == PHASE_RETURN) {
            // The breadcrumbs lead, the rangers veto. Fly at STEP_SPEED_MS to
            // waypoints[return_index], then the one before it, down to crumb 0
            // at the start of the outbound leg. The follower does not run here;
            // this is a reverse replay of the trail on the position estimate.
            //
            // The command passes through clampAwayFromObstacles, which removes
            // only the component into an obstacle, so the drone slides along a
            // wall toward the crumb instead of pressing into it. A crumb not
            // reached within RETRACE_STUCK_MS is skipped. There is no geofence
            // or deck-silent check here; the exits are home, the hard time
            // limit and an abort.
            float tx = waypoints[return_index].x / 1000.0f;
            float ty = waypoints[return_index].y / 1000.0f;
            float dx = tx - (tele_x / 1000.0f);
            float dy = ty - (tele_y / 1000.0f);
            float dist = sqrtf(dx * dx + dy * dy);

            bool reached = dist < (float)RETRACE_ARRIVE_MM / 1000.0f;
            bool stuck = (int32_t)(xTaskGetTickCount() - crumb_since)
                           >= (int32_t)M2T(RETRACE_STUCK_MS);

            if (reached || stuck) {
                if (stuck && !reached) {
                    // Something is between the drone and that crumb. Every
                    // crumb leads home, so take the next one rather than push.
                    DEBUG_PRINT("CAVEBAT: crumb %d blocked, skipping\n",
                                (int)return_index);
                }
                if (return_index == 0) {
                    DEBUG_PRINT("CAVEBAT: home, landing\n");
                    tele_phase = PHASE_LANDING;
                } else {
                    return_index--;
                    crumb_since = xTaskGetTickCount();
                }
            } else {
                // The world-frame direction to the crumb, rotated into the body
                // frame by the measured heading. Heading is held throughout,
                // so this comes out as flying backwards and sideways rather
                // than as a turn.
                float ux = dx / dist, uy = dy / dist;
                float c = cosf(meas_yaw), sn = sinf(meas_yaw);
                float vx =  (ux * c + uy * sn) * STEP_SPEED_MS;
                float vy = (-ux * sn + uy * c) * STEP_SPEED_MS;

                // Never fly into anything (WF_STOP_MM, WF_PUSH_MM).
                clampAwayFromObstacles(&vx, &vy);

                // Same WF_ACCEL_MS2 ramp as wfTick().
                float maxStep = WF_ACCEL_MS2 * 0.1f;
                float dvx = vx - vx_cmd;
                float dvy = vy - vy_cmd;
                if (dvx >  maxStep) dvx =  maxStep;
                if (dvx < -maxStep) dvx = -maxStep;
                if (dvy >  maxStep) dvy =  maxStep;
                if (dvy < -maxStep) dvy = -maxStep;
                vx_cmd += dvx;
                vy_cmd += dvy;

                vel_active = true;
                sendBodyVelocity(vx_cmd, vy_cmd, commandedHeight(), 0.0f);
            }
        }

        if (tele_phase == PHASE_LANDING) {
            // Normal landing (timer done, home reached, or hard time limit).
            // Blocks for the land trajectory plus 0.5 s, then back to IDLE.
            // Release first: a land command is worthless while velocity
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
        } else {
            // Not landing: run the in-flight aborts. They set
            // mission.state = 2, and the abort branch below acts on it in the
            // next tick. (In hover mode the high-level commander holds
            // position by itself once the takeoff trajectory completes.)

            // In-flight battery abort: vbat under minvbat -
            // VBAT_INFLIGHT_MARGIN_MV (3100 mV by default) on 3 consecutive
            // ticks, ignored until the climb is done (spin-up sag is worst
            // there). A controlled landing beats the supervisor cutting the
            // motors at altitude.
            //
            // SAFETY: deliberately not gated on mission.guards. Flying on a
            // collapsing battery is never better than landing, and this check
            // has no false-positive mode that would justify switching it off.
            // Gating it once left it off by default, and flights then flipped
            // mid-turn, at peak current draw, on a pack resting at 3.80 V.
            if ((int32_t)(xTaskGetTickCount() - climb_done_tick) < 0) {
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

            // In-flight obstacle abort, only with mission.guards = 1 and only
            // after the climb (see climb_done_tick): something within minobst
            // (0 = no reading) on 3 consecutive ticks. One stray short ToF
            // reading should not end a flight; three in a row is 0.3 s.
            if (!mission_guards) {
                obstacle_streak = 0;
            } else if ((int32_t)(xTaskGetTickCount() - climb_done_tick) < 0) {
                obstacle_streak = 0;
            // The ranger facing the followed wall is excluded, because being
            // close to that wall is the mission. Front, back and the other side
            // still count (all four in hover mode).
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
        // Abort while already on the ground: just clear the request. A land
        // command here would run a full land trajectory from zero height and
        // spin the motors for over a second for no reason.
        DEBUG_PRINT("CAVEBAT: Abort ignored, not flying\n");
        mission_state = 0;

    } else if (mission_state == 2) {
        // Abort in flight: requested by the phone (mission.state = 2), or set
        // by the over-tilt check earlier in this tick, or by the battery or
        // obstacle check in the previous tick. Lands from wherever the drone
        // is (no BRAKE, no RETURN) with tele.endwhy = 2; blocks like LANDING.
        releaseToHighLevel();   // or the land below is overridden, see above
        tele_endwhy = 2;
        tele_phase = PHASE_LANDING;
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
       // Redundant: is_flying is already false whenever this runs.
       is_flying = false;
    }

    // Heartbeat console line every 100 ticks (~10 s). Printing it every
    // second saturates the BLE link and queues param/log TOC replies behind
    // console text until the app times out. Live values come from tele.*.
    if ((tele_alive % 100) == 0) {
      DEBUG_PRINT("CB state=%d bat=%d f=%d b=%d l=%d r=%d u=%d d=%d\n",
                  (int)mission_state, (int)tele_vbat, (int)tele_front,
                  (int)tele_back, (int)tele_left, (int)tele_right,
                  (int)tele_up, (int)tele_down);
    }
    
    // The loop decides at 10 Hz (a little slower in practice: the work above
    // adds to the 100 ms). Everything above is tuned for that: the guards
    // count ticks and the heartbeat and trace divide by it.
    //
    // While vel_active, the wait is spent in ten 10 ms slices and the velocity
    // setpoint is re-sent on each, a 100 Hz refresh. commanderSetSetpoint()
    // re-stamps the timestamp, so re-sending the same struct keeps the command
    // fresh for the commander and the supervisor. Otherwise (hover mode, on
    // the ground) this is a plain 100 ms wait.
    for (uint8_t i = 0; i < 10; i++) {
      vTaskDelay(M2T(10));
      if (vel_active) {
        commanderSetSetpoint(&wf_setpoint, 3);
      }
    }
  }
}

// --- Parameter Registration ---
// tele.*: status values, also in the log group below (phase, outwhy and yaw
// are log-only). mission.*: the mission settings, described at the top.
PARAM_GROUP_START(tele)
  PARAM_ADD(PARAM_UINT16, alive, &tele_alive)
  PARAM_ADD(PARAM_UINT8,  canfly, &tele_canfly)
  PARAM_ADD(PARAM_UINT8,  clear,  &tele_clear)
  PARAM_ADD(PARAM_UINT16, maxz,   &tele_maxz)
  PARAM_ADD(PARAM_UINT16, samples, &tele_samples)
  PARAM_ADD(PARAM_UINT8,  endwhy, &tele_endwhy)
  // tele_wfstate is deliberately not exposed (as a param or a log). Adding a
  // param changes the param table, which invalidates the app's cached copy and
  // requires the app's KNOWN_PARAM_NAMES to be updated to match. The follower
  // step goes out in the WF console line and in the recording instead.
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
  // 0 = hover for the timer, then land.
  // 1 = follow a wall on the RIGHT for half the timer, then retrace home.
  // 2 = the same, following a wall on the LEFT.
  // Defaults to 0, plain hover, if the app never sets it.
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
  // 0 idle, 1 climbing, 2 outbound, 3 returning, 4 landing, 5 braking.
  LOG_ADD(LOG_UINT8,  phase,  &tele_phase)
  // Why the outbound leg ended: 0 still going, 1 timer, 2 geofence,
  // 4 trail full, 7 ranger deck silent (3 is not set by this build).
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