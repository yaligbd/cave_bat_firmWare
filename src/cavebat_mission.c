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

  // What the wall follower was doing, packed into one byte:
  //   low  4 bits  the follower's state (0 forward .. 7 findCorner)
  //   high 4 bits  the flight mode (0 hover, 1 wall right, 2 wall left)
  //
  // WHY THIS IS RECORDED RATHER THAN PRINTED. The state was going out on the
  // console, and the console does not survive. A wall-following flight came
  // back missing "Initiating Takeoff", missing "following wall on the LEFT"
  // and missing every line of the trace, with the drone reporting "LOG packets
  // drop detected" -- the crash it was meant to explain left no evidence at
  // all. The console shares a 20-byte BLE link with three log blocks and loses
  // whatever does not fit.
  //
  // The recording does not have that problem: it is written to memory during
  // the flight and downloaded afterwards over a quiet link, and every sample
  // of every flight so far has arrived intact. The samples already carry the
  // position, the heading and all six ranges -- the decision made from them
  // was the only thing missing.
  //
  // One byte takes the sample to 15 and the download packet to 18, still
  // inside the 19 that fits a single BLE notification.
  uint8_t wf;

  // The WORST tilt seen since the previous sample, in 2-degree units.
  //
  // A peak, not a snapshot, because samples are one second apart and a flip
  // takes a fraction of that -- an instantaneous reading would miss it
  // entirely. Recorded rather than printed for the same reason the state is:
  // the console drops exactly the lines that matter, and two crashes have now
  // gone unexplained because of it.
  //
  // This is the one measurement that separates the two remaining explanations.
  // Tilt CLIMBING across several samples means the controller is fighting a
  // position estimate it cannot trust, which is a software problem. Tilt near
  // zero and then suddenly enormous means nothing was fighting anything and
  // the aircraft hit something, which is not.
  //
  // One byte takes the sample to 16 and the packet to 19, the most that fits
  // in a single BLE notification.
  uint8_t tilt;
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

// --- Which side is being followed RIGHT NOW ---------------------------------
//
// mission_wallfollow is what the operator asked for and never changes during a
// flight. This is what the follower is actually doing, and it FLIPS halfway.
//
// WHY. The way home used to be dead reckoning: stop, hand control to the
// trajectory planner, and fly backwards through recorded positions without
// looking at anything. That handover is where the aircraft sagged, every
// single time, and the last recording caught it losing 214mm at 2 degrees of
// tilt -- level, under command, not a tumble.
//
// Turning round and following the same wall home removes the handover
// entirely. The aircraft stays under velocity control for the whole flight,
// and it keeps looking at the wall instead of trusting an estimate that has
// been drifting since takeoff. It is also just what a person would do.
//
// After 180 degrees the wall that was on the left is on the right, so the
// follower is restarted with the direction inverted -- which is all "follow it
// back" means.
static uint8_t active_follow = 0;

// Heading to reach before the turnaround is complete, radians.
static float turn_target = 0.0f;

// Set when following the wall home has failed, so the braking phase falls back
// to the breadcrumb trail instead of trying to turn round again.
//
// The way home is a cascade, not a single plan: follow the wall back, and if
// that fails fly the recorded positions back. Landing where it happens to be
// is not one of the options -- an aircraft that gives up in the middle of a
// cave is no use, even if the ground it picks is flat.
static bool force_breadcrumbs = false;

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

// Ceiling on how fast the aircraft is allowed to rotate, degrees per second.
//
// WHY THIS EXISTS. Turning is where this aircraft kept failing: it tilted
// through every corner, crashed at the turning point of a 45s flight, and
// bumped the wall while pivoting at an inward corner. What makes that telling
// is that a corner turn commands ZERO velocity -- commandTurn() sets cmdVelX to
// 0 and the action clears cmdVelY, so nothing asks the aircraft to move at all.
// Drifting anyway means the position estimate is wrong, not the controller.
//
// That is the Flow deck. The optical flow sensor watches the floor sweep past
// during a yaw, the estimator reads part of that sweep as real translation, and
// the velocity controller then works hard to cancel a motion that was never
// happening. The tilt it uses to do that becomes drift, and 40cm from a wall
// drift is a collision.
//
// Bitcraze set maxTurnRate to 0.5 rad/s (about 29 deg/s) alongside a forward
// speed of 0.5 m/s. We fly at 0.2 m/s -- 40% of their speed at 100% of their
// turn rate. Halving the rotation roughly halves how fast the flow estimate is
// corrupted, at the cost of a 46-degree corner taking about 3 seconds instead
// of 1.6. That trade was unaffordable while flights were 15 seconds long and
// corners never completed; with a 45s timer there is room for it.
//
// Capped here in our own layer rather than by editing Bitcraze's file, so their
// state machine stays exactly as published. This is safe because every one of
// its transitions is driven by a MEASURED heading change or a range reading,
// never by assuming a turn took a particular time -- so a slower turn simply
// takes more ticks to satisfy the same condition.
// 20, raised from 15 after flying it. 15 stopped the aircraft tilting through
// corners, which was the point, but it turned visibly slower than it needed to
// and every corner manoeuvre spends that time inside the outbound budget. 20 is
// still well under Bitcraze's 29 and keeps most of the margin.
// 25, raised from 20 once the velocity was scaled along with it.
//
// The cap was originally there to stop the aircraft tilting through turns, and
// 15 then 20 were chosen while the cap was being applied to the rotation ALONE
// -- which quietly widened every corner arc and crashed two flights. Now that
// speed scales with it the arc radius is right at any cap, so the only thing
// the number controls is how long a corner takes. 25 is close to Bitcraze's
// own 28.6 and gives most of that time back.
#define WF_MAX_YAWRATE_DEG  25.0f

// Drop a breadcrumb every time the drone has moved this far since the last
// one. The old controller dropped one per completed hop; there are no hops
// any more, so distance is what marks the trail now.
// Halved from 300mm. The return flies straight lines between breadcrumbs, so
// the spacing IS the resolution of the path home: at 300mm a flight that
// rounded a corner was retraced as a shortcut across it, which is the "it went
// a direct route instead of the exact way" that was reported. At 150mm the
// trail bends where the flight bent.
#define WF_CRUMB_MM      150

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
static logVarId_t wf_idFront, wf_idLeft, wf_idRight, wf_idYaw, wf_idBack, wf_idUp, wf_idDown;

// --- Refusing to fly into things --------------------------------------------
//
// Anything closer than this in the direction of travel cancels movement that
// way. Nothing else: the commanded velocity is only ever reduced, never
// redirected.
//
// That restraint is the entire point. Every crash in this project came from
// logic that decided to DO something clever near a wall -- turn toward it,
// stop dead, hand over control -- and a layer that can only subtract cannot
// invent a manoeuvre, cannot oscillate against the follower, and cannot fight
// it for control. It composes with whatever the follower wants instead of
// arguing with it.
//
// 250mm sits below everything the follower works to: it keeps 400mm off the
// wall and calls a corner at 600mm ahead. So in normal flight this never
// engages at all. It is what catches the case the follower got wrong -- like
// flying into a dead end because the corner never triggered.
#define WF_STOP_MM       250

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

// --- Smoothing the commanded speed ------------------------------------------
//
// How fast the commanded velocity is allowed to change, m/s per second.
//
// The wall follower's states hand back step changes: forwardAlongWall asks for
// 0.2 m/s, rotateInCorner asks for 0, and the switch between them lands in a
// single tick. A quadrotor obeys "stop now" by pitching up, which trades away
// the vertical component of its thrust, so every one of those steps costs
// altitude. That is the dip that remained at direction changes after the
// braking phase fixed it for the return leg -- same fault, the rest of the
// flight.
//
// Rate-limiting the commanded velocity fixes all of them at once instead of
// special-casing each transition. At 0.4 m/s^2 a full stop from 0.2 m/s takes
// half a second, which the aircraft can do on a gentle pitch.
//
// The obstacle clamp is applied to the TARGET, before this, so an obstacle
// still stops the drone -- it just decelerates into the stop rather than
// snapping. From 0.2 m/s that costs about 5cm, which is why WF_STOP_MM is 250
// and not 150.
#define WF_ACCEL_MS2     0.4f

// The commanded velocity as last sent, which the ramp works from.
static float vx_cmd = 0.0f, vy_cmd = 0.0f;

// Speed when braking began, so the brake ramps down from whatever the aircraft
// was actually doing rather than jumping back up to full speed first.
static float brake_v0 = 0.0f;

// True once the aircraft has turned back to its takeoff heading at the end of
// the return leg.
static bool home_turn_done = false;

// Tick at which the braking phase began, so the ramp knows how far along it is.
static TickType_t brake_start = 0;

// --- Giving up on a corner ---------------------------------------------------
//
// Tick at which the follower was last actually following a wall, and how long
// it may spend away from that before the mission gives up and flies home.
//
// WHY. A downloaded flight showed the whole failure. Five seconds of
// forwardAlongWall holding the wall at 280-420mm, exactly as intended. Then the
// wall was lost at one corner -- and the follower never reached
// forwardAlongWall again for the remaining twenty-one seconds. It cycled
// findCorner, rotateAroundWall, turnToAlignToWall, rotateInCorner, back to
// rotateAroundWall, yaw swinging 8 -> 33 -> 84 -> 7 -> 66 degrees, turning
// continuously until it turned into something.
//
// The corner states have no collective timeout. Individually each is
// reasonable; together they can form a loop with no exit, because every one of
// them is waiting for a wall that searching is not finding. Altitude held at
// ~500mm through all of it and only collapsed at the very end, so the sag that
// looks like the fault is a symptom of twenty seconds of manoeuvring, not its
// cause.
//
// Rather than tune the corner geometry again -- twice now that has fixed one
// case and broken another -- this makes losing the wall SAFE. A corner that
// cannot be completed in WF_LOST_MS stops being a corner and becomes a reason
// to come home, which is the right answer for an aircraft whose job is to map
// a cave and get back.
//
// 10s is comfortably longer than a real corner, which needs about 5 to 6: a 46
// degree rotation at 25 deg/s, a one second measurement pause, then finding
// and re-aligning to the wall.
#define WF_LOST_MS 10000
static TickType_t last_following_tick = 0;

// --- Stop flying the mission when the aircraft starts losing it -------------
//
// Tilt past this many degrees, for two ticks running, ends the flight: the
// aircraft stops being asked to go anywhere and lands.
//
// WHY THIS RATHER THAN MORE TUNING. Every crash in this project has had the
// same shape -- the aircraft gets into trouble and the firmware keeps flying
// the mission at it until it is upside down. The last one went from 16 degrees
// of yaw to 136 degrees per second in under a second while still being told to
// strafe toward a wall it had lost.
//
// A cave mapper that follows a wall for twenty seconds, gets confused and
// comes home has done its job. One that tries heroically to round every corner
// and ends on its back has not, and it cannot be downloaded from a wall.
//
// So the demand is removed at the first sign of trouble. If the controller is
// chasing a position estimate it cannot trust, taking away the target is
// exactly what stops the chase; if it has hit something, nothing was going to
// help anyway and landing is still the least bad answer.
//
// 30 degrees is far beyond normal -- healthy flights record 0 to 2, and even
// aggressive corners stay under 15 -- but well short of the 65 to 74 seen once
// a tumble is already unrecoverable.
#define TILT_ABORT_DEG   30.0f
#define TILT_ABORT_TICKS 2
static uint8_t tilt_streak = 0;

// Worst tilt since the last sample, 2-degree units. See FlightSample.tilt.
static uint8_t tilt_peak = 0;

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
//
// The trail home. One saved position every WF_CRUMB_MM of travel.
//
// The height is stored with each one, not assumed. The outbound leg ducks under
// a low ceiling, so a return flown at the requested altitude would climb back
// into the roof it had just avoided. Retracing the height that was actually
// flown is the only version of "go back the way you came" that is true in a
// cave.
//
// Doubled to 128 when the spacing was halved, so the reachable range is
// unchanged at about 19m of outbound path -- far more than any timer the app
// offers can fly at 0.2 m/s. Costs 768 bytes of static RAM, not heap.
#define MAX_WAYPOINTS 128
typedef struct {
  int16_t x, y, z;
  // How far the followed wall was when this crumb was dropped, mm, or 0 if it
  // was out of range. This is what makes the return self-correcting: see
  // wallCorrection().
  int16_t side;
} Waypoint;
static Waypoint waypoints[MAX_WAYPOINTS];
static uint16_t waypoint_count = 0;

// --- Mission phase (the app can read this) ----------------------------------
#define PHASE_IDLE      0
#define PHASE_CLIMB     1
#define PHASE_OUTBOUND  2
#define PHASE_RETURN    3
#define PHASE_LANDING   4
// Slowing down, between following the wall and flying home.
//
// This phase exists because its absence crashed the aircraft. The outbound leg
// ended by stepping the commanded velocity straight from full speed to zero,
// and stopping a quadrotor means pitching up, which trades away the vertical
// component of its thrust. With the pack sagging to 3219mV there was no
// headroom left to brake and hold height at the same time, so it sank -- about
// 20cm on a good day, and on a bad one the attitude went with it and the
// aircraft ended on its back. The log is unambiguous: seven healthy lines at
// st=4 holding 400mm, then "returning", then 65 degrees of roll.
//
// So the speed is ramped down over BRAKE_MS instead, as a phase rather than a
// blocking wait -- the old code slept 400ms inside the loop, which also blinded
// the recording and the tilt watch for exactly the window where the failure
// began.
#define PHASE_BRAKE     5

// Turning round, and then following the wall back.
//
// PHASE_RETURN still exists and still flies breadcrumbs, because a HOVER
// mission and any flight with no wall to follow has nothing else to go home
// by. But a wall-following flight uses these instead: it turns 180 degrees on
// the spot and follows the same wall back, never leaving velocity control and
// never trusting a position estimate it has not checked against anything.
#define PHASE_TURNAROUND 6
#define PHASE_HOMEBOUND  7

// How close to the takeoff point counts as home, mm.
//
// Generous on purpose. After a minute of flying, the position estimate is the
// least trustworthy thing on the aircraft, and insisting on precision would
// have it hunting for a spot it cannot find. Landing half a metre off is fine;
// circling is not.
#define HOME_RADIUS_MM  500

// Give up turning round after this long and land where it is. A 180 at
// 25 deg/s takes about seven seconds, so this only fires if the turn is not
// progressing at all.
#define TURNAROUND_MS  12000
static TickType_t turnaround_start = 0;

// How long to spend slowing down. Long enough that the aircraft never has to
// pitch hard to lose 0.2 m/s.
#define BRAKE_MS        800
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
  // A floor, or short hops are commanded violently.
  //
  // 0.7s was too low once the trail became dense. A 300mm hop flown in 0.7s is
  // 0.43 m/s -- more than TWICE the speed the outbound leg flies at -- so the
  // aircraft left every breadcrumb by accelerating hard, and accelerating means
  // tilting, and tilting costs altitude on a sagging pack. That is the drop
  // still seen at the start of the return after the braking fix.
  //
  // At 1.5s a WF_CRUMB_MM hop is flown at the same speed as the outbound leg,
  // which is what "go back the way you came" ought to mean.
  if (dur < 1.5f) dur = 1.5f;
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
    // The height actually being flown, which a low ceiling may have lowered.
    waypoints[waypoint_count].z = (int16_t)(commandedHeight() * 1000.0f);
    // The raw followed-side reading, NOT tele_*, because clampRange() turns
    // out-of-range into 0 and this needs to tell "far" from "no reading".
    {
      float sideRaw = (active_follow == 2) ? safeLogFloat(wf_idLeft)
                                                : safeLogFloat(wf_idRight);
      waypoints[waypoint_count].side =
        (sideRaw > 0.0f && sideRaw < 3000.0f) ? (int16_t)sideRaw : 0;
    }
    waypoint_count++;
  }
  crumb_x = tele_x;
  crumb_y = tele_y;
}

// Which breadcrumb is closest to where the aircraft is now.
//
// The trail runs from the pad (index 0) outwards, and the way home is to walk
// it backwards. Starting that walk at the LAST crumb is right when the
// fallback happens at the far end of the outbound leg -- but wrong if it
// happens halfway home, because the newest crumbs are then behind the aircraft
// and it would fly back out to the far end before turning round again.
//
// Rejoining at the nearest point is what a person reading a map would do.
static uint16_t nearestWaypoint(void) {
  uint16_t best = 0;
  int64_t bestDist = -1;
  for (uint16_t i = 0; i < waypoint_count; i++) {
    int64_t dx = (int64_t)tele_x - (int64_t)waypoints[i].x;
    int64_t dy = (int64_t)tele_y - (int64_t)waypoints[i].y;
    int64_t d = dx * dx + dy * dy;
    if (bestDist < 0 || d < bestDist) {
      bestDist = d;
      best = i;
    }
  }
  return best;
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
// Cancel any commanded motion that heads into something close.
//
// Body frame: vx forward, vy LEFT. So a positive vy is checked against the
// left sensor and a negative one against the right.
//
// Only motion TOWARD an obstacle is removed. Motion away is untouched, which
// matters more than it sounds: when the follower is hard against its wall it
// answers by strafing away from it, and a clamp that blocked that would trap
// the aircraft against the very thing it was trying to escape.
//
// A reading of exactly 0 is treated as "no reading" rather than "touching".
// The raw sensors report 32767 when nothing is in range, so a real 0 is
// almost unheard of -- but safeLogFloat() also returns 0 for a sensor that is
// not fitted, and reading that as an obstacle would freeze a Multi-ranger-less
// aircraft in place. Losing the 0mm case is the cheaper mistake by far.
static void clampAwayFromObstacles(float *vx, float *vy) {
  float f = safeLogFloat(wf_idFront);
  float b = safeLogFloat(wf_idBack);
  float l = safeLogFloat(wf_idLeft);
  float r = safeLogFloat(wf_idRight);

  if (*vx > 0.0f && f > 0.0f && f < (float)WF_STOP_MM) *vx = 0.0f;
  if (*vx < 0.0f && b > 0.0f && b < (float)WF_STOP_MM) *vx = 0.0f;
  if (*vy > 0.0f && l > 0.0f && l < (float)WF_STOP_MM) *vy = 0.0f;
  if (*vy < 0.0f && r > 0.0f && r < (float)WF_STOP_MM) *vy = 0.0f;
}

// How high to fly right now, in metres.
//
// Normally the height the app asked for. Under a low ceiling, less: a cave
// roof does not care what altitude was requested on the pad. Taken from
// Bitcraze's own wall-following demo, which does exactly this with the up
// ranger, and capped so that one bad reading cannot drive the aircraft into
// the floor.
// --- Terrain, and why stairs break a Crazyflie ------------------------------
//
// The altitude controller holds stateEstimate.z, and the estimator builds that
// almost entirely from the downward laser. That works because it assumes the
// floor is flat and at zero. A staircase breaks the assumption directly: fly
// over a riser and the laser shortens by 170mm in one reading, the estimator
// concludes the aircraft just DROPPED 170mm, and the controller hauls it up
// to correct a fall that never happened. Fly off the top and the reverse.
//
// So the aircraft already terrain-follows -- badly. The problem is not that it
// ignores the step, it is that it reacts to the whole step instantly, and the
// estimator reports "State out of bounds, resetting" when the discontinuity is
// large enough. A reset mid-flight throws away the position the breadcrumbs
// and the way home depend on.
//
// This converts the step into a ramp. When the laser jumps by more than a
// stair-sized amount in one tick, that is terrain rather than noise, and the
// commanded height is moved by the SAME amount immediately -- which cancels
// the error the controller was about to chase. The offset then decays back to
// zero at a limited rate, so the aircraft re-acquires its requested clearance
// smoothly over about half a second instead of lurching.
//
// The threshold sits above sensor noise (tens of mm) and below a stair riser
// (about 170mm), so ordinary drift is left to the estimator and only real
// terrain moves the setpoint.
// THE AMBIGUITY THAT MADE THIS DANGEROUS, and what is done about it.
//
// The downward laser cannot tell "the floor came up" from "I went down". Both
// shorten the reading by the same amount. The first version treated any drop
// over 80mm in a tick as terrain and lowered the target to match -- so when the
// aircraft sagged at a phase change, the reading fell, the target followed it
// down, and the aircraft obediently chased itself into the floor. A recorded
// flight caught it exactly: 214mm lost in one second at 2 degrees of tilt,
// level and under command the whole way.
//
// Two things make that impossible now.
//
// The threshold is 140mm rather than 80. A stair riser is about 170mm and
// crosses the beam within a single tick, so a real step still clears it; an
// aircraft would have to be falling at 1.4 m/s to fake one.
//
// And a step may only be taken once per AGL_COOLDOWN_MS. A staircase presents
// one edge at a time, so nothing real is lost -- but a sustained descent
// cannot ratchet the target down tick after tick, which is precisely how the
// crash happened.
#define AGL_STEP_MM       140.0f  // a change this big in ONE tick is terrain
#define AGL_SLEW_MS         0.30f // how fast the offset is given back, m/s
#define AGL_MAX_M           0.40f // never chase more terrain than this at once
#define AGL_COOLDOWN_MS  1200     // minimum gap between accepted steps
static TickType_t last_step_tick = 0;

static float terrain_offset_m = 0.0f;
static float last_zrange_mm = 0.0f;

static void terrainTick(void) {
  float zr = safeLogFloat(wf_idDown);
  // 0 means no reading, not zero height. Forget the history too, or the next
  // valid reading looks like an enormous step.
  if (zr <= 0.0f || zr > 3000.0f) {
    last_zrange_mm = 0.0f;
    return;
  }

  if (last_zrange_mm > 0.0f) {
    float jump = zr - last_zrange_mm;
    float mag = jump < 0.0f ? -jump : jump;
    if (mag > AGL_STEP_MM
        && (int32_t)(xTaskGetTickCount() - last_step_tick) >= (int32_t)M2T(AGL_COOLDOWN_MS)) {
      last_step_tick = xTaskGetTickCount();
      // Cancel the transient. Floor rises -> laser shortens -> jump negative
      // -> commanded height drops by the same amount, so the controller sees
      // no error to fight.
      terrain_offset_m += jump / 1000.0f;
      if (terrain_offset_m >  AGL_MAX_M) terrain_offset_m =  AGL_MAX_M;
      if (terrain_offset_m < -AGL_MAX_M) terrain_offset_m = -AGL_MAX_M;
      DEBUG_PRINT("TERRAIN step %dmm\n", (int)jump);
    }
  }
  last_zrange_mm = zr;

  // Give the offset back gradually, so the aircraft climbs the step rather
  // than being snapped up it.
  float step = AGL_SLEW_MS * 0.1f;          // per 10Hz tick, metres
  if (terrain_offset_m >  step)      terrain_offset_m -= step;
  else if (terrain_offset_m < -step) terrain_offset_m += step;
  else                               terrain_offset_m = 0.0f;
}

static float commandedHeight(void) {
  float h = mission_height / 1000.0f + terrain_offset_m;
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

static void wfTick(void) {
  float frontRange = safeLogFloat(wf_idFront) / 1000.0f;
  float sideRange  = (active_follow == 2 ? safeLogFloat(wf_idLeft)
                                              : safeLogFloat(wf_idRight)) / 1000.0f;
  float yawRad     = safeLogFloat(wf_idYaw) * DEG2RAD;
  int   direction  = (active_follow == 2) ? -1 : 1;

  // The follower measures its own timeouts in seconds against this, so it has
  // to keep counting up across the whole flight. Ticks since boot, as seconds.
  float now_s = (float)xTaskGetTickCount() / (float)configTICK_RATE_HZ;

  float vx = 0.0f, vy = 0.0f, yawRateRad = 0.0f;
  wf_state = wallFollower(&vx, &vy, &yawRateRad,
                          frontRange, sideRange, yawRad, direction, now_s);
  tele_wfstate = (uint8_t)wf_state;

  // The one state meaning "a wall is beside me and I am following it".
  // Everything else is searching for one, and searching has a time limit.
  if (wf_state == forwardAlongWall) last_following_tick = xTaskGetTickCount();

  // Cap the rotation -- and slow the TRANSLATION by the same factor.
  //
  // Capping the turn rate alone was a real bug, and it cost two crashed
  // corners. The follower does not pick a turn rate independently; around an
  // outside corner it DERIVES one from the forward speed to fly a particular
  // arc:
  //
  //     cmdVelX = maxForwardSpeed;
  //     cmdAngW = direction * (-cmdVelX / radius);
  //
  // The radius is what matters -- it is set to the wall distance, so the
  // aircraft curves around the corner at the range it was already holding.
  // Clamping the yaw rate while leaving the speed alone changes that radius:
  // 0.2 m/s against a 20 deg/s cap arcs at 0.57m instead of the 0.4m intended,
  // so the aircraft runs wide and stops tracking the wall it is turning around.
  //
  // Scaling the velocity by the same factor keeps vx/omega, and therefore the
  // radius, exactly as the controller intended. The corner is flown on the same
  // path, just more slowly. States that rotate on the spot already have vx = 0,
  // so scaling costs them nothing.
  float yawRateDeg = yawRateRad * RAD2DEG;
  float yawMag = yawRateDeg < 0.0f ? -yawRateDeg : yawRateDeg;
  if (yawMag > WF_MAX_YAWRATE_DEG) {
    float scale = WF_MAX_YAWRATE_DEG / yawMag;
    yawRateDeg *= scale;
    vx *= scale;
    vy *= scale;
  }

  // Never fly into anything. Applied to the TARGET, so the ramp below
  // decelerates into the stop instead of snapping to it.
  clampAwayFromObstacles(&vx, &vy);

  // Ease onto the new speed rather than stepping onto it. See WF_ACCEL_MS2:
  // this is what stops the aircraft sagging every time the follower changes
  // its mind about where to go.
  float maxStep = WF_ACCEL_MS2 * 0.1f;   // decisions run at 10Hz
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

// --- Using the wall on the way home -----------------------------------------
//
// How far sideways to shift the next return target, in metres, so that the
// aircraft ends up the same distance from the wall as it was on the way out.
//
// WHY THE RETURN NEEDED THIS. Until now the way home was pure dead reckoning:
// fly to a list of positions the Flow deck reported earlier, without looking at
// anything. That trusts the position estimate completely, and the Flow deck
// drifts -- so the aircraft faithfully replays a path that no longer matches
// the room, and has no way to notice, because it is not looking.
//
// But it IS still beside the wall it followed out. The heading is held through
// the whole return, so the wall stays on the same side, and each breadcrumb
// recorded how far away it was at that point. Comparing the live reading with
// the recorded one measures the drift perpendicular to the wall directly, and
// that is the component dead reckoning gets wrong first.
//
// Conservative on purpose: it does nothing unless both readings are real,
// ignores differences under 50mm as noise, and never shifts a target by more
// than WF_FIX_MAX_MM. A correction is only ever a nudge toward what the wall
// says -- a wrong one must not be able to fly the aircraft into it.
#define WF_FIX_MIN_MM   50
#define WF_FIX_MAX_MM  250

static float wallCorrection(int16_t recordedSide) {
  if (!active_follow) return 0.0f;          // hover has no wall
  if (recordedSide <= 0) return 0.0f;            // nothing was seen back then

  float live = (active_follow == 2) ? safeLogFloat(wf_idLeft)
                                         : safeLogFloat(wf_idRight);
  if (live <= 0.0f || live >= 3000.0f) return 0.0f;   // nothing seen now

  float err = live - (float)recordedSide;        // +ve = further away than before
  float mag = err < 0.0f ? -err : err;
  if (mag < (float)WF_FIX_MIN_MM) return 0.0f;   // noise
  if (err >  (float)WF_FIX_MAX_MM) err =  (float)WF_FIX_MAX_MM;
  if (err < -(float)WF_FIX_MAX_MM) err = -(float)WF_FIX_MAX_MM;

  // Body +y is LEFT. Following on the left, being too far away means moving
  // +y to close up; following on the right it means moving -y. Returned in
  // metres along body y.
  float sign = (active_follow == 2) ? 1.0f : -1.0f;
  return sign * err / 1000.0f;
}

// Is the way to the next return target clear?
//
// The return leg flies goTo hops, which belong to the trajectory planner, so
// the velocity clamp that protects wall following cannot reach them. That left
// the whole way home with no obstacle protection at all -- and the aircraft
// flies it sideways or backwards, so its front sensor is not even pointed where
// it is going.
//
// This checks the sensor that faces the direction of travel before committing
// to a hop. It does not steer around anything; it refuses to set off. Given the
// path was flown minutes earlier, anything in the way now is either new or
// means the position estimate has drifted enough that the path is wrong -- and
// in both cases not moving is the right answer.
static bool returnPathClear(float dx_m, float dy_m) {
  // The intended move, rotated from world into the body frame the sensors live
  // in. Heading is held through the return, so this is mission_yaw.
  float c = cosf(mission_yaw), s = sinf(mission_yaw);
  float fwd  =  dx_m * c + dy_m * s;   // +ve = forwards
  float left = -dx_m * s + dy_m * c;   // +ve = to the left

  // Only the dominant axis is checked. A hop is one short straight line, and
  // the sensor facing most nearly along it is the one that can see what is
  // there.
  float range;
  if ((fwd < 0 ? -fwd : fwd) >= (left < 0 ? -left : left)) {
    range = (fwd >= 0.0f) ? safeLogFloat(wf_idFront) : safeLogFloat(wf_idBack);
  } else {
    range = (left >= 0.0f) ? safeLogFloat(wf_idLeft) : safeLogFloat(wf_idRight);
  }
  // A reading of exactly 0 means the sensor is absent, not touching -- see
  // clampAwayFromObstacles for why that distinction matters.
  return !(range > 0.0f && range < (float)WF_STOP_MM);
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
  // One last command to hold still, so the aircraft is stopped by an
  // instruction rather than by a supervisor that has given up on us: a setpoint
  // older than 500ms trips supervisorStateWarningLevelOut, which disables x and
  // y control and forces the aircraft level.
  //
  // There is no wait here any more. PHASE_BRAKE has already ramped the speed
  // down over BRAKE_MS before this is called, so by now the aircraft is
  // stationary. The 400ms blocking wait this replaces was doing real harm: it
  // stopped the loop, so the recording and the tilt watch went blind for
  // exactly the window in which the aircraft started falling.
  sendBodyVelocity(0.0f, 0.0f, commandedHeight(), 0.0f);
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
  // Tilt, for the crash trace. The aircraft has been flipping rather than
  // colliding, and these two numbers are what tell those apart: a controller
  // winding up against a bad estimate tilts further over several seconds,
  // while something mechanical or an estimator jump goes from level to over
  // inside one or two lines.
  logVarId_t idRoll  = logGetVarId("stabilizer", "roll");
  logVarId_t idPitch = logGetVarId("stabilizer", "pitch");

  // The same handles again at file scope, for the wall follower. It runs from
  // the 100Hz refresh at the foot of the loop, outside this function's scope.
  wf_idFront = idFront;
  wf_idLeft  = idLeft;
  wf_idRight = idRight;
  wf_idYaw   = idYaw;
  wf_idBack  = idBack;
  wf_idUp    = idUp;
  wf_idDown  = idDown;

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

    // --- The tilt watch -----------------------------------------------------
    //
    // The aircraft has been FLIPPING, not colliding, and no log has ever caught
    // it happening. This is the one line that can settle why.
    //
    // It prints at the full 10Hz, but ONLY past 15 degrees, which a healthy
    // flight never reaches -- so it costs nothing until something is going
    // wrong and then it is dense exactly when it matters. A line this short at
    // 10Hz is affordable on a 20-byte link; the full WF line at that rate would
    // not be, which is how earlier evidence was lost.
    //
    // WHAT THE ANSWER LOOKS LIKE, either way:
    //   tilt climbing over several lines  -> the controller is winding up
    //                                        against an estimate it cannot
    //                                        trust, and the fix is in software
    //   level, then over inside one line  -> nothing was chasing anything. A
    //                                        motor, a prop or a sudden
    //                                        estimator jump, and no amount of
    //                                        controller work will help
    //
    // Deliberately not restricted to wall following: hover flipped earlier in
    // this project too, and a flight already lost is not one whose console
    // traffic needs protecting.
    if (is_flying) {
      float roll_now  = safeLogFloat(idRoll);
      float pitch_now = safeLogFloat(idPitch);
      if (roll_now < 0) roll_now = -roll_now;
      if (pitch_now < 0) pitch_now = -pitch_now;
      // Worst tilt this second, held until the next sample takes it.
      float worst = (roll_now > pitch_now) ? roll_now : pitch_now;
      {
        uint16_t units = (uint16_t)(worst / 2.0f);
        if (units > 255) units = 255;
        if ((uint8_t)units > tilt_peak) tilt_peak = (uint8_t)units;
      }

      // Follow the terrain. Suppressed until the climb is over: the aircraft
      // is changing height on purpose up there, and a climb looks exactly like
      // a floor falling away.
      if ((int32_t)(xTaskGetTickCount() - climb_done_tick) >= 0) {
        terrainTick();
      }

      // Losing it. Stop flying the mission and put it down.
      //
      // Suppressed until the climb is over, like the other guards: the
      // aircraft tilts on the way up and the estimator is least trustworthy
      // there, which is the worst possible moment to react to it.
      if (worst > TILT_ABORT_DEG
          && (int32_t)(xTaskGetTickCount() - climb_done_tick) >= 0) {
        if (tilt_streak < 255) tilt_streak++;
      } else {
        tilt_streak = 0;
      }
      if (tilt_streak >= TILT_ABORT_TICKS && tele_phase != PHASE_LANDING) {
        DEBUG_PRINT("CAVEBAT: tilt %d deg, abandoning mission and landing\n",
                    (int)worst);
        tele_endwhy = 3;
        tele_phase = PHASE_LANDING;
      }
      // 5Hz, not the full 10.
      //
      // The console shares a 20-byte BLE link with two 5Hz log blocks and a
      // 1Hz one, and it is already losing lines: a flight arrived with no
      // "Initiating Takeoff", no "following wall on the LEFT" and none of the
      // WF trace, alongside the drone's own "LOG packets drop detected". A
      // tilt line ten times a second is the heaviest thing here and it fires
      // exactly when the other lines matter most.
      //
      // Halving it still gives two or three lines across a flip, which is
      // enough to tell a controller winding up from something letting go.
      if ((roll_now > 15.0f || pitch_now > 15.0f) && (tele_alive % 2) == 0) {
        DEBUG_PRINT("TILT r=%d p=%d z=%d\n",
                    (int)safeLogFloat(idRoll), (int)safeLogFloat(idPitch),
                    (int)tele_z);
      }
    }

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
        // The follower's state and the flight mode, one nibble each.
        fs->wf    = (uint8_t)((active_follow & 0x0f) << 4)
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
        vx_cmd = 0.0f;
        tilt_streak = 0;
        tilt_peak = 0;
        terrain_offset_m = 0.0f;
        last_zrange_mm = 0.0f;
        last_step_tick = 0;
        // Starts as what was asked for; flips at the turnaround.
        active_follow = mission_wallfollow;
        force_breadcrumbs = false;
        turn_target = 0.0f;
        vy_cmd = 0.0f;
        brake_v0 = 0.0f;
        home_turn_done = false;
        if (mission_wallfollow) {
          // Start ALREADY FOLLOWING, not in `forward`.
          //
          // This matters more than it looks. The only way out of the `forward`
          // state is the FRONT sensor seeing something within 60cm -- it does
          // not look at the side sensor at all. Bitcraze's demo is launched in
          // the middle of a room, so it flies forward, meets a wall ahead,
          // turns to find it and then follows.
          //
          // CaveBat is launched with the wall already beside the drone and open
          // space ahead, which is what the app's own instructions ask for. Told
          // to start in `forward`, the machine therefore never left it: it flew
          // straight at full speed past the wall it was supposed to follow
          // until it hit something. That was three crashed flights.
          //
          // `forwardAlongWall` is the state that holds the distance and flies
          // on, which is exactly the situation on the pad. If the wall turns
          // out to be further than 70cm the machine drops into findCorner and
          // searches for it, so a sloppy placement degrades into a search
          // rather than a crash.
          wallFollowerInit(mission_walldist / 1000.0f, WF_SPEED_MS,
                           forwardAlongWall);
          wf_state = forwardAlongWall;
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
                    // Starts now, not at takeoff: the climb is not searching.
                    last_following_tick = xTaskGetTickCount();
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

        } else if (tele_phase == PHASE_OUTBOUND || tele_phase == PHASE_HOMEBOUND) {
            // Both legs fly the same follower. Only the reasons to stop differ,
            // which is the whole point of turning round rather than switching
            // to a different way of navigating.
            bool homebound = (tele_phase == PHASE_HOMEBOUND);
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
                // GAVE UP ON A CORNER. Checked first, because an aircraft
                // that has stopped following a wall is not doing the mission
                // any more -- it is turning in circles looking for one, and
                // every other reason to go home assumes it is still flying a
                // sensible path. See WF_LOST_MS.
                //
                // HOME. Checked before everything, because arriving ends the
                // flight whatever else is true.
                //
                // Half a metre counts. After a minute of flying the position
                // estimate is the least trustworthy thing on the aircraft, and
                // demanding precision would leave it hunting for a spot it
                // cannot find. Landing slightly off beats circling.
                int64_t hx = tele_x, hy = tele_y;
                if (homebound &&
                    (hx * hx + hy * hy) <= (int64_t)HOME_RADIUS_MM * (int64_t)HOME_RADIUS_MM) {
                    tele_outwhy = 6;
                    DEBUG_PRINT("CAVEBAT: home at %d,%d mm, landing\n",
                                (int)tele_x, (int)tele_y);
                    tele_phase = PHASE_LANDING;
                } else if ((int32_t)(xTaskGetTickCount() - last_following_tick)
                      >= (int32_t)M2T(WF_LOST_MS)) {
                    // Lost the wall. On the way out that means turn round; on
                    // the way home there is nothing left to turn round FOR, so
                    // it lands where it is rather than hunting.
                    tele_outwhy = 5;
                    DEBUG_PRINT("CAVEBAT: lost the wall for %ds, %s\n",
                                (int)(WF_LOST_MS / 1000),
                                homebound ? "breadcrumbs home" : "going home");
                    // Losing the wall on the way home does not end the
                    // flight -- it drops to the next way of getting home.
                    if (homebound) force_breadcrumbs = true;
                    tele_phase = PHASE_BRAKE;
                    brake_v0 = vx_cmd;
                    brake_start = xTaskGetTickCount();
                } else if (beyondGeofence()) {
                    tele_outwhy = 2;
                    DEBUG_PRINT("CAVEBAT: geofence at %d,%d mm, returning\n",
                                (int)tele_x, (int)tele_y);
                    tele_phase = PHASE_BRAKE;
                    brake_start = xTaskGetTickCount();
                    brake_v0 = vx_cmd;   // ramp down from the real speed
                } else if (!homebound && waypoint_count >= MAX_WAYPOINTS) {
                    tele_outwhy = 4;
                    DEBUG_PRINT("CAVEBAT: breadcrumb trail full, returning\n");
                    tele_phase = PHASE_BRAKE;
                    brake_start = xTaskGetTickCount();
                    brake_v0 = vx_cmd;   // ramp down from the real speed
                } else if (!homebound
                           && (int32_t)(xTaskGetTickCount() - outbound_deadline) >= 0) {
                    tele_outwhy = 1;
                    DEBUG_PRINT("CAVEBAT: half timer, returning over %d points\n",
                                (int)waypoint_count);
                    tele_phase = PHASE_BRAKE;
                    brake_start = xTaskGetTickCount();
                    brake_v0 = vx_cmd;   // ramp down from the real speed
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
                    // Twice a second through a corner, once a second along a
                    // wall. Corners are where this keeps failing and they last
                    // three to five seconds, so four lines at 1Hz was not
                    // enough to watch one go wrong -- but the straight
                    // stretches are long and uneventful, and printing those
                    // faster would crowd the link for nothing.
                    uint16_t traceEvery = (wf_state == forwardAlongWall) ? 10 : 5;
                    if ((tele_alive % traceEvery) == 0) {
                        DEBUG_PRINT("WF%c st=%d f=%d s=%d r=%d p=%d\n",
                                    (active_follow == 2) ? 'L' : 'R',
                                    (int)tele_wfstate,
                                    (int)tele_front,
                                    (int)(active_follow == 2 ? tele_left
                                                                  : tele_right),
                                    (int)safeLogFloat(idRoll),
                                    (int)safeLogFloat(idPitch));
                    }
                }
            }

        } else if (tele_phase == PHASE_BRAKE) {
            // --- Slowing down before flying home -------------------------
            //
            // Ramp the commanded forward speed to zero over BRAKE_MS rather
            // than dropping it in one tick. A quadrotor stops by pitching up,
            // which trades away part of the thrust holding it in the air, so an
            // instant stop demands a hard pitch and costs altitude at the worst
            // possible moment. That is what put the aircraft on its back.
            //
            // Nothing blocks here: the ramp is spread over ordinary 10Hz ticks,
            // so the recording and the tilt watch keep running right through
            // the transition. The version this replaces slept 400ms inside the
            // loop and went blind for exactly the window where it fell.
            int32_t elapsed = (int32_t)(xTaskGetTickCount() - brake_start);
            if (elapsed < (int32_t)M2T(BRAKE_MS)) {
                float f = 1.0f - ((float)elapsed / (float)M2T(BRAKE_MS));
                if (f < 0.0f) f = 0.0f;
                vel_active = true;
                // From brake_v0, not WF_SPEED_MS. If the follower was already
                // slowing -- mid-corner, say -- ramping from full speed would
                // accelerate the aircraft before stopping it.
                vx_cmd = brake_v0 * f;
                vy_cmd = 0.0f;
                sendBodyVelocity(vx_cmd, vy_cmd, commandedHeight(), 0.0f);
            } else {
                // Stopped. Hand the aircraft back to the trajectory planner.
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
                if (active_follow && !force_breadcrumbs) {
                    // TURN ROUND AND FOLLOW THE WALL BACK.
                    //
                    // No handover at all: the aircraft stays under velocity
                    // control, so the sag that happened at every single
                    // handover cannot happen here.
                    turn_target = wrapYaw(safeLogFloat(wf_idYaw) * DEG2RAD + 3.14159265f);
                    turnaround_start = xTaskGetTickCount();
                    tele_phase = PHASE_TURNAROUND;
                    DEBUG_PRINT("CAVEBAT: turning round to follow the wall home\n");
                } else {
                    // No wall to follow home -- a hover mission, or wall
                    // following was never on. Breadcrumbs are all there is.
                    releaseToHighLevel();
                    step_active = false;
                    // Rejoin the trail where the aircraft actually is, then
                    // walk it back to the pad. +1 because the step below
                    // decrements before using it.
                    return_index = (uint16_t)(nearestWaypoint() + 1);
                    if (return_index > waypoint_count) return_index = waypoint_count;
                    tele_phase = PHASE_RETURN;
                    DEBUG_PRINT("CAVEBAT: stopped, home over %d pts\n",
                                (int)waypoint_count);
                }
            }

        } else if (tele_phase == PHASE_TURNAROUND) {
            // --- Turning round on the spot ------------------------------
            //
            // Zero translation, pure rotation, until the heading has come
            // round by 180 degrees. Rotating in place is the one manoeuvre
            // the Flow deck copes with worst, so it is done at the capped
            // rate and with nothing else going on at the same time.
            float now = wrapYaw(safeLogFloat(wf_idYaw) * DEG2RAD);
            float err = wrapYaw(turn_target - now);
            float mag = err < 0.0f ? -err : err;

            bool timedOut = (int32_t)(xTaskGetTickCount() - turnaround_start)
                              >= (int32_t)M2T(TURNAROUND_MS);

            if (mag < 0.15f || timedOut) {          // within ~9 degrees
                if (timedOut) {
                    // Could not get round. Still going home, just the other way.
                    DEBUG_PRINT("CAVEBAT: turnaround stalled, breadcrumbs home\n");
                    force_breadcrumbs = true;
                    brake_v0 = 0.0f;
                    brake_start = xTaskGetTickCount();
                    tele_phase = PHASE_BRAKE;
                } else {
                    // The wall that was on the left is now on the right.
                    active_follow = (active_follow == 2) ? 1 : 2;
                    wallFollowerInit(mission_walldist / 1000.0f, WF_SPEED_MS,
                                     forwardAlongWall);
                    wf_state = forwardAlongWall;
                    last_following_tick = xTaskGetTickCount();
                    vx_cmd = 0.0f;
                    vy_cmd = 0.0f;
                    tele_phase = PHASE_HOMEBOUND;
                    DEBUG_PRINT("CAVEBAT: round, following wall on the %s home\n",
                                active_follow == 2 ? "LEFT" : "RIGHT");
                }
            } else {
                // Rotate toward the target, at the capped rate.
                float rate = (err > 0.0f) ? WF_MAX_YAWRATE_DEG : -WF_MAX_YAWRATE_DEG;
                vel_active = true;
                vx_cmd = 0.0f;
                vy_cmd = 0.0f;
                sendBodyVelocity(0.0f, 0.0f, commandedHeight(), rate);
            }

        } else if (tele_phase == PHASE_RETURN) {
            if (!stepFinished()) {
                // let the current leg finish
            } else {
                step_active = false;
                if (return_index == 0) {
                    // Home. Turn back to the heading it took off on before
                    // landing.
                    //
                    // The return leg deliberately holds whatever heading the
                    // outbound leg ended on, because rotating is what upsets
                    // the Flow deck and rotating repeatedly along the way home
                    // would cost position on every hop. The cost of that choice
                    // is that the aircraft lands facing an arbitrary direction.
                    //
                    // So the rotation happens once, here, at the end: on the
                    // pad, with nothing left to navigate to and nothing to hit.
                    // Drift during this turn costs nothing, because the next
                    // thing that happens is landing.
                    if (!home_turn_done && waypoint_count > 0) {
                        home_turn_done = true;
                        DEBUG_PRINT("CAVEBAT: home, turning to takeoff heading\n");
                        issueStep(waypoints[0].x / 1000.0f,
                                  waypoints[0].y / 1000.0f,
                                  waypoints[0].z / 1000.0f,
                                  0.0f);
                    } else {
                        DEBUG_PRINT("CAVEBAT: home, landing\n");
                        tele_phase = PHASE_LANDING;
                    }
                } else {
                    return_index--;
                    // Heading held, not recomputed. The drone is retracing
                    // space it has just flown through, so there is nothing to
                    // look at that it has not already seen, and holding the
                    // heading keeps rotation out of the return entirely. It
                    // flies home sideways or backwards, which the aircraft
                    // does perfectly well, and every sample still carries the
                    // heading so the map stays correct.
                    {
                        float tx = waypoints[return_index].x / 1000.0f;
                        float ty = waypoints[return_index].y / 1000.0f;

                        // Correct the target sideways by what the wall says.
                        // Dead reckoning alone replays the Flow deck's drift;
                        // the wall is the one thing out here that has not
                        // moved.
                        float fix = wallCorrection(waypoints[return_index].side);
                        if (fix != 0.0f) {
                            // Body +y (left) expressed in world coordinates.
                            tx += fix * -sinf(mission_yaw);
                            ty += fix *  cosf(mission_yaw);
                            DEBUG_PRINT("RTN fix %dmm\n", (int)(fix * 1000.0f));
                        }

                        float dx = tx - (tele_x / 1000.0f);
                        float dy = ty - (tele_y / 1000.0f);

                        if (!returnPathClear(dx, dy)) {
                            // Something is in the way. Do not set off: hold
                            // position and look again next tick. return_index
                            // is put back so this waypoint is retried rather
                            // than skipped -- skipping it would cut a corner
                            // through whatever is blocking the path.
                            return_index++;
                            DEBUG_PRINT("RTN blocked, holding\n");
                        } else {
                            issueStep(tx, ty,
                                      waypoints[return_index].z / 1000.0f,
                                      mission_yaw);
                        }
                    }
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
                       (active_follow != 2 && sideBlocked(tele_left)) ||
                       (active_follow != 1 && sideBlocked(tele_right))) {
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