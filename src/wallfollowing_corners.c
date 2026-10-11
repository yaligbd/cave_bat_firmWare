/*
 * wallfollowing_corners.c
 *
 * Wall follower that flies corners as a fixed script rather than a blend of
 * states. cavebat_mission.c calls it once per 10 Hz tick in wall-follow mode
 * and turns its output into a body-velocity setpoint.
 *
 *   wall ahead    -> stop, turn 90 degrees away from it, verify, carry on
 *   wall vanished -> drive out past the corner, turn 90 degrees into it, fly
 *                    on until the wall is back, carry on
 *
 * Design rules:
 *
 *   1. The heading changes only in discrete 90 degree turns, on the spot,
 *      with no translation. No yaw rate is ever computed from a range
 *      reading: that loop oscillates against the wall. FOLLOW does hold its
 *      heading, with a gentle yaw-rate loop on the estimated yaw (see
 *      CF_HOLD_GAIN). Turning on the spot also suits the Flow deck: rotating
 *      while translating is what it cannot separate, so turning stopped
 *      protects the position estimate the return leg depends on.
 *
 *   2. No step commands full speed toward something close. The steps that
 *      fly forward (FOLLOW, PAST, REACQ) check frontBlocked first on every
 *      tick and hand over to a corner: STOP, or BACKOFF when the side wall is
 *      too close to turn.
 *
 *   3. No step commands zero for long. The still waits are short and fixed:
 *      STOP 0.7 s while the caller's ramp brings the aircraft to rest, then
 *      the 0.4 s settle in VERIFY after a turn. GAVEUP is the exception.
 *
 *   4. Every corner step is bounded. TURN ends on a heading, not on a range
 *      threshold, and gives up after CF_STEP_TIMEOUT_S; BACKOFF ends within
 *      CF_BACKOFF_MS; PAST and REACQ end within a set distance. FOLLOW has no
 *      timeout and GAVEUP is terminal. At most CF_MAX_TURNS turns are flown
 *      in a row without following before the follower gives up.
 *
 * Giving up means holding position: GAVEUP commands zero until the next
 * wallFollowerCornersInit(). The mission still turns for home at half the
 * timer and the hard time limit lands it, whatever step the follower is in,
 * so holding still is the safe response to a room the script cannot handle.
 */

#include <math.h>

#include "wallfollowing_corners.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG2RAD_F  0.017453292f
#define RAD2DEG_F  57.29578f
#define QUARTER    1.5707963f

// Thresholds are margins on the holding distance (ref) rather than bare
// numbers, so changing the wall distance moves them all together.

// Stop further out than the holding distance.
//
// After an inward corner the wall that was ahead becomes the wall alongside, so
// the distance the aircraft stops at is its starting distance from the next
// wall. Stopping at ref would start every leg too close (the caller's ramp
// coasts it about 50 mm further, and the turn itself drifts), and the next
// corner would then be turned too close: turning within about 300 mm of a wall
// can flip the aircraft. Stopping 150 mm further out puts the next wall outside
// the target, so the trim eases in toward it. An error toward open space is
// harmless; one toward the wall is not.
#define CF_STOP_MARGIN      0.15f  // stop this much further out than ref

// Ahead counts as clear beyond ref + this.
//
// Keeps a 150 mm band between "clear enough to fly" (VERIFY) and "close enough
// to stop" (CF_STOP_MARGIN), wider than ranger noise, so a turn VERIFY accepts
// is not immediately followed by another stop.
//
// Assumes the wall ahead after a turn is more than ref + 0.30 m away (0.70 m
// at the default 0.4 m). In a narrower space VERIFY reads a dead end and turns
// another 90 degrees: wrong, but safe.
#define CF_CLEAR_MARGIN     0.30f
#define CF_LOST_MARGIN      0.40f  // side past ref + this: the wall ended
#define CF_FOUND_MARGIN     0.25f  // side inside ref + this: a wall is there

#define CF_TURN_RATE_DEG   30.0f   // 90 degrees in three seconds
#define CF_TURN_TOL_DEG     6.0f   // turn is done within this of the goal
#define CF_SETTLE_S         0.4f   // let the rangers catch up after a rotation
#define CF_STOP_S           0.7f   // long enough for the ramp to reach zero
#define CF_PAST_M           0.30f  // travel past an outward corner before turning
// How far REACQ searches for the wall before giving up.
//
// A distance, not a time, because distance decides whether the wall comes into
// view. After an outward corner the new wall comes alongside only after about
// ref of travel (about 700 mm from the corner, counting CF_PAST_M), so this is
// comfortably more than that without letting the aircraft wander across a
// room. PAST and REACQ integrate distance from the commanded speed at an
// assumed 10 Hz; it is not measured travel.
#define CF_REACQ_MAX_M      1.50f
#define CF_STEP_TIMEOUT_S   5.0f   // TURN gives up (GAVEUP) after this long
#define CF_MAX_TURNS        4      // max turns in a row without following
#define CF_TRIM_GAIN        0.6f   // strafe (m/s) per metre of distance error

// Heading hold in FOLLOW, seeded from the turn.
//
// With nothing holding the heading the nose wanders (about 2.5 deg/s) and
// curves the path into the wall faster than the sideways trim can correct. But
// a hold is only as good as the heading it holds: one that is off parallel
// makes the aircraft crab, and the trim spends its authority cancelling that
// instead of closing the gap.
//
// After a 90 degree turn the right heading is known: the wall about to be
// followed is the one that was ahead, so for a square corner goal_heading is
// parallel to it, and TURN seeds the hold with it. The opening leg
// has only the heading the aircraft was placed at, and CF_ALIGN_RATE walks that
// into place: a strafe command that will not go away means the nose is off
// parallel, so the held heading leans into it until the strafe stops. That is
// at most 2 deg/s (full trim at 10 Hz), slow enough not to oscillate, and it
// fades out as the distance error does.
#define CF_HOLD_GAIN        0.5f   // deg/s of yaw rate per degree of error
#define CF_HOLD_MAX_DEG    10.0f   // cap on that yaw rate, deg/s
#define CF_ALIGN_RATE     0.035f   // rad/tick of held heading per m/s of strafe

// How close to the side wall is too close to turn.
//
// Stopping pitches the aircraft up and turning sweeps its rotors; within about
// 300 mm of the wall alongside the two together can flip it. CF_STOP_MARGIN
// keeps the wall ahead at a distance; this covers the wall beside. A corner
// that starts closer than this is postponed, not refused: BACKOFF eases out to
// the holding distance first, then STOP and TURN follow.
#define CF_TURN_MIN_SIDE   0.35f   // metres; under this, back off before turning
#define CF_BACKOFF_MS      3.0f    // BACKOFF time limit, in s despite the name
#define CF_BACKOFF_VX      0.08f   // m/s; gentle reverse, opens the gap ahead

static float ref_distance = 0.4f;
static float max_speed = 0.2f;

static int   state = CF_FOLLOW;
static float state_start = 0.0f;
static float goal_heading = 0.0f;
static float hold_heading = 0.0f;   // what FOLLOW steers to, see CF_HOLD_GAIN
static float past_travelled = 0.0f;
static int   turns_without_following = 0;
static bool  first_run = true;

void wallFollowerCornersInit(float refDistanceFromWall, float maxSpeed)
{
  ref_distance = refDistanceFromWall;
  max_speed = maxSpeed;
  state = CF_FOLLOW;
  state_start = 0.0f;
  goal_heading = 0.0f;
  hold_heading = 0.0f;
  past_travelled = 0.0f;
  turns_without_following = 0;
  first_run = true;
}

// Wrap to [-pi, pi] so heading errors take the shortest way round. Without it a
// 10 degree error across the +/-180 seam reads as 350 and the turn goes the
// long way.
static float wrapToPi(float a)
{
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

// Which step a corner starts in: BACKOFF if the wall alongside is closer than
// CF_TURN_MIN_SIDE, otherwise STOP. FOLLOW, PAST and REACQ can all meet a wall
// ahead, so all three go through here and the side-clearance check applies
// whichever step meets it.
static int startCorner(float sideRange)
{
  if (sideRange > 0.0f && sideRange < CF_TURN_MIN_SIDE) return CF_BACKOFF;
  return CF_STOP;
}

// Switch step and restart its clock (inState counts from here).
static int enter(int newState, float now)
{
  state_start = now;
  return newState;
}

int wallFollowerCorners(float *velX, float *velY, float *velW,
                        float frontRange, float sideRange,
                        float currentHeading, int direction, float now)
{
  if (first_run) {
    state_start = now;
    // The opening leg has nothing better to go on than how it was placed.
    // CF_ALIGN_RATE corrects it from there.
    hold_heading = currentHeading;
    first_run = false;
  }

  const float dir = (float)direction;

  // Zero or less means no reading and counts as open space, never as touching.
  // (cavebat_multiranger.c reports an invalid reading as 32767 mm, which
  // arrives here as 32.8 m and also reads as open.)
  bool frontBlocked = frontRange > 0.0f && frontRange < ref_distance + CF_STOP_MARGIN;
  bool frontClear   = frontRange <= 0.0f || frontRange > ref_distance + CF_CLEAR_MARGIN;
  bool sideGone     = sideRange  <= 0.0f || sideRange  > ref_distance + CF_LOST_MARGIN;
  bool sideFound    = sideRange  >  0.0f && sideRange  < ref_distance + CF_FOUND_MARGIN;

  float vx = 0.0f, vy = 0.0f, wDeg = 0.0f;
  float inState = now - state_start;

  switch (state) {

  case CF_FOLLOW: {
    if (frontBlocked) { state = enter(startCorner(sideRange), now); break; }
    if (sideGone)     { past_travelled = 0.0f; state = enter(CF_PAST, now); break; }

    // Hold the asked-for distance by strafing, never by turning. Positive
    // error means too far off the wall, so move toward it: the wall is +y in
    // the body frame when it is on the left (direction -1), -y on the right.
    float err = sideRange - ref_distance;
    float trim = err * CF_TRIM_GAIN;
    float lim = max_speed * 0.5f;
    if (trim >  lim) trim =  lim;
    if (trim < -lim) trim = -lim;

    // Steer back to the held heading. Closed on the estimated yaw, not on a
    // range, so it cannot fight the wall or oscillate against the corner
    // geometry.
    float hErrDeg = wrapToPi(hold_heading - currentHeading) * RAD2DEG_F;
    wDeg = hErrDeg * CF_HOLD_GAIN;
    if (wDeg >  CF_HOLD_MAX_DEG) wDeg =  CF_HOLD_MAX_DEG;
    if (wDeg < -CF_HOLD_MAX_DEG) wDeg = -CF_HOLD_MAX_DEG;

    vx = max_speed;
    vy = -dir * trim;
    turns_without_following = 0;

    // Walk the held heading towards parallel. See CF_ALIGN_RATE: the sign
    // works out the same on both walls, because strafing right always wants a
    // lower heading and strafing left a higher one.
    hold_heading = wrapToPi(hold_heading + vy * CF_ALIGN_RATE);
    break;
  }

  case CF_BACKOFF:
    // Ease away from the side wall and back from the wall ahead, with zero yaw
    // rate, until the side reading reaches ref (or drops out), then STOP.
    //
    // Gives up after CF_BACKOFF_MS and goes to STOP anyway, deliberately: if
    // the gap will not open the space is too tight to fix, and turning from a
    // bad position beats hovering at a wall until the timer runs out.
    if (sideRange <= 0.0f || sideRange >= ref_distance ||
        inState > CF_BACKOFF_MS) {
      state = enter(CF_STOP, now);
      break;
    }
    // Reverse gently as well. The aircraft arrives here at cruise and coasts
    // while the caller's ramp slows it, so vx = 0 would still close on the
    // wall ahead; backing off opens both gaps at once.
    vx = -CF_BACKOFF_VX;
    // Away from the wall is +dir. Body +y is left (clampAwayFromObstacles
    // blocks +vy when the left reading is near), so a right wall (dir +1)
    // needs +y and a left wall (dir -1) needs -y. FOLLOW's trim uses -dir
    // because its error is already negative when too close.
    vy = dir * (max_speed * 0.5f);
    break;

  case CF_STOP:
    // Everything stays zero: this step asks for a halt and waits for the ramp
    // in the caller to deliver one, so the nose eases down instead of dipping.
    if (inState >= CF_STOP_S) {
      // Do not re-check side clearance here (for example by going back to
      // BACKOFF). The aircraft coasts toward the wall ahead while easing
      // sideways, so postponing the turn fixes the side gap at the cost of the
      // front one, and a turn that close to the wall ahead can flip it. Once
      // stopped, turning is what takes the nose off the wall ahead.
      //
      // Inward corner: turn away from the wall beside, which puts the wall that
      // was ahead alongside. Right when following on the left.
      goal_heading = wrapToPi(currentHeading + dir * QUARTER);
      turns_without_following++;
      state = enter(CF_TURN, now);
    }
    break;

  case CF_TURN: {
    float errDeg = wrapToPi(goal_heading - currentHeading) * RAD2DEG_F;
    if (errDeg < CF_TURN_TOL_DEG && errDeg > -CF_TURN_TOL_DEG) {
      // Seed the heading hold. The wall about to be followed is the one that
      // was ahead, so the angle this turn aimed at is parallel to it. Taking
      // the aim rather than the achieved heading also discards the turn's
      // overshoot instead of holding it for the whole next leg.
      hold_heading = goal_heading;
      state = enter(CF_VERIFY, now);
      break;
    }
    if (inState > CF_STEP_TIMEOUT_S) { state = enter(CF_GAVEUP, now); break; }
    wDeg = errDeg > 0.0f ? CF_TURN_RATE_DEG : -CF_TURN_RATE_DEG;
    break;
  }

  case CF_VERIFY:
    // Believe the rangers only once they have had a moment to settle: they are
    // looking at a different part of the room than they were a second ago.
    // Short, and entered from a standstill -- see rule 3.
    if (inState < CF_SETTLE_S) break;

    if (frontClear && sideFound) {
      state = enter(CF_FOLLOW, now);          // corner done
    } else if (!frontClear) {
      // Still blocked. Not a corner: a dead end, or a gap narrower than the
      // turn. Another 90 makes a 180, which is the way back out: the right
      // answer to a dead end, and cheap to be wrong about. The GAVEUP branch
      // is a backstop; the check after the switch already gives up as soon
      // as the count passes CF_MAX_TURNS.
      if (turns_without_following > CF_MAX_TURNS) {
        state = enter(CF_GAVEUP, now);
      } else {
        goal_heading = wrapToPi(currentHeading + dir * QUARTER);
        turns_without_following++;
        state = enter(CF_TURN, now);
      }
    } else {
      // Clear ahead but no wall alongside: the turn came round short of it, or
      // it is further off than expected. Go and look.
      past_travelled = 0.0f;
      state = enter(CF_REACQ, now);
    }
    break;

  case CF_PAST:
    // The wall ended. Turning here would turn into open air and lose it, so
    // carry on until the corner is behind the aircraft.
    if (sideFound) {
      // It came back: a doorway or a gap, not a corner. Keep following.
      state = enter(CF_FOLLOW, now);
      break;
    }
    if (frontBlocked) { state = enter(startCorner(sideRange), now); break; }

    vx = max_speed;
    past_travelled += max_speed * 0.1f;        // commanded, assumes 10Hz ticks
    if (past_travelled >= CF_PAST_M) {
      // Outward corner: turn into where the wall went.
      goal_heading = wrapToPi(currentHeading - dir * QUARTER);
      turns_without_following++;
      state = enter(CF_TURN, now);
    }
    break;

  case CF_REACQ:
    // Search at cruise speed, not a creep. That is safe: at cruise and 10 Hz
    // the aircraft moves about 20 mm between side readings, far less than the
    // sideFound window, so it cannot pass the wall unseen; and frontBlocked
    // (ref + CF_STOP_MARGIN, 550 mm by default) leaves room to stop from
    // cruise. It ends when the wall reappears alongside, something is close
    // ahead, or CF_REACQ_MAX_M is used up.
    if (sideFound)    { state = enter(CF_FOLLOW, now); break; }
    if (frontBlocked) { state = enter(startCorner(sideRange), now); break; }
    // Bounded by distance rather than time, so a slower cruise speed searches
    // the same ground instead of less of it.
    if (past_travelled >= CF_REACQ_MAX_M) { state = enter(CF_GAVEUP, now); break; }
    vx = max_speed;
    past_travelled += max_speed * 0.1f;   // commanded, assumes 10Hz ticks
    break;

  case CF_GAVEUP:
  default:
    // Hold station with a zero command. Terminal: nothing leaves this step
    // until the next wallFollowerCornersInit(). The mission turns for home at
    // half the timer and the hard time limit lands it regardless, so standing
    // still is both safe and self-clearing.
    break;
  }

  // Going in circles. turns_without_following counts every turn started (from
  // STOP, VERIFY or PAST) and is cleared only by a FOLLOW tick that actually
  // flies, not by entering FOLLOW. Once it passes CF_MAX_TURNS the turn just
  // started is dropped and the follower gives up: the room is not what this
  // script assumes, and another turn will not help.
  if (turns_without_following > CF_MAX_TURNS && state != CF_GAVEUP) {
    state = enter(CF_GAVEUP, now);
    vx = 0.0f; vy = 0.0f; wDeg = 0.0f;
  }

  *velX = vx;
  *velY = vy;
  *velW = wDeg * DEG2RAD_F;
  return state;
}
