/*
 * wallfollowing_corners.c
 *
 * WHAT THIS REPLACES, AND WHAT THE EVIDENCE FOR IT IS.
 *
 * Three implementations of McGuire's state machine have now flown this
 * aircraft and all three fail at corners:
 *
 *   Bitcraze's C demo          corners fail roughly one time in two
 *   Bitcraze's Python twin     crashed at the first corner, flown from the PC
 *                              with none of our firmware in the loop
 *   TU Delft's research build  worse than either, crashed every attempt
 *
 * The Python flight is the one that matters, because nothing of ours was
 * running: it rules our firmware out as the cause. Two defects were measured
 * in its log, and both are in the design rather than in any one port:
 *
 *   A FULL SECOND COMMANDING NOTHING. Entering the align state it sends
 *   vx = 0, vy = 0, yaw rate = 0 for ten consecutive ticks. The aircraft is
 *   not being flown during that second. Measured drift: 40cm, half a metre
 *   from a wall.
 *
 *   A LUNGE AT A WALL IT HAS ALREADY SEEN. One tick of forward-along-wall
 *   commanding the full 0.30 m/s with the wall 0.30m ahead -- a speed that
 *   closes the whole gap in a second -- then straight back to a corner state.
 *
 * So this file abandons the blend. A corner becomes a script:
 *
 *   wall ahead    -> stop, turn 90 degrees away from it, verify, carry on
 *   wall vanished -> drive out past the corner, turn 90 into it, creep until
 *                    the wall is back, carry on
 *
 * FOUR RULES, each aimed at something that actually went wrong.
 *
 *   1. YAW ONLY EVER HAPPENS IN DISCRETE 90 DEGREE TURNS, on the spot, with
 *      no translation. There is no continuous yaw feedback anywhere in this
 *      file. Every version that has oscillated -- theirs and my own earlier
 *      attempt -- did so through a yaw rate computed continuously from a
 *      range reading. Removing that loop removes the oscillation by
 *      construction, not by tuning. It also suits the hardware: rotating
 *      while translating is precisely what a Flow deck cannot separate, so
 *      turning stopped is what keeps the position estimate the return leg
 *      depends on.
 *
 *   2. NO STEP MAY COMMAND FULL SPEED TOWARD SOMETHING CLOSE. Forward flight
 *      happens only in FOLLOW, which gives way to STOP the moment the wall
 *      ahead is inside the holding distance.
 *
 *   3. NO STEP MAY COMMAND NOTHING FOR LONG. The only still step is the
 *      settle inside VERIFY, it lasts 0.4s not 1.0, and the aircraft has
 *      already been brought to a stop before it begins -- rather than
 *      arriving at it in the middle of a turn at speed.
 *
 *   4. EVERY STEP ENDS. Each has a timeout, and a turn finishes on a heading
 *      rather than on a range threshold happening to agree. No step hands
 *      back to the step it came from, so the machine cannot cycle.
 *
 * GIVING UP IS HOLDING POSITION. If the script runs out of ideas it holds
 * station rather than landing or improvising. That is safe here: the mission
 * turns for home at half the timer whatever the follower is doing, and the
 * hard time limit lands it regardless. A drone holding still beside a wall is
 * the safest thing this code can do when it does not understand the room.
 */

#include <math.h>

#include "wallfollowing_corners.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG2RAD_F  0.017453292f
#define RAD2DEG_F  57.29578f
#define QUARTER    1.5707963f

// Margins on the holding distance rather than bare numbers, so changing the
// wall distance moves them all together.
// STOP FURTHER OUT THAN WE MEAN TO HOLD.
//
// This is the difference between a corner that works and one that puts the
// aircraft on its back, and it took a crash to see it.
//
// The wall ahead becomes the wall alongside the moment the turn finishes. So
// whatever distance the aircraft stops at IS its starting distance from the
// next wall. Stopping at the holding distance therefore guarantees arriving too
// close: the ramp carries it in another 50mm, and the turn itself drifts. Two
// flights in a row measured the result -- stopping with the wall 300mm ahead,
// then following the next wall at 240-300mm against a 400mm target, with the
// trim crawling outward at 0.06 m/s and the walls of a 2m arena too short to
// finish the job. The second corner was then taken 280mm from a wall, and that
// is where it went over: level at one sample, 118 degrees at the next, on a
// healthy 4.02V battery.
//
// Stopping 150mm further out puts the next wall OUTSIDE the target instead of
// inside it, so the trim eases in rather than clawing out. Easing in is the
// safe direction: an error toward open space costs nothing, an error toward the
// wall costs the aircraft.
#define CF_STOP_MARGIN      0.15f  // stop this much further out than ref

// Ahead counts as clear past ref + this.
//
// Raised with CF_STOP_MARGIN to keep a band between "clear enough to fly" and
// "close enough to stop". Without one, verifying a turn at 600mm and then
// stopping again at 550mm leaves 50mm of daylight, and ranger noise alone could
// bounce the machine between the two. 150mm is wider than the noise.
//
// It does assume the wall ahead after a turn is further than 700mm, which in
// this 2m arena it is -- the flown path is a 1.2m square. In a narrower
// corridor this would read as a dead end, and the answer to a dead end is
// another 90 degrees and back out, which is wrong but safe.
#define CF_CLEAR_MARGIN     0.30f
#define CF_LOST_MARGIN      0.40f  // side past ref + this: the wall ended
#define CF_FOUND_MARGIN     0.25f  // side inside ref + this: a wall is there

#define CF_TURN_RATE_DEG   30.0f   // 90 degrees in three seconds
#define CF_TURN_TOL_DEG     6.0f
#define CF_SETTLE_S         0.4f   // let the rangers catch up after a rotation
#define CF_STOP_S           0.7f   // long enough for the ramp to reach zero
#define CF_PAST_M           0.30f  // travel past an outward corner before turning
#define CF_CREEP_MS         0.08f
#define CF_STEP_TIMEOUT_S   5.0f   // any one step taking longer than this failed
#define CF_MAX_TURNS        4      // turns without following = going in circles
#define CF_TRIM_GAIN        0.6f   // strafe per metre of distance error

// HOLD THE HEADING WHILE FOLLOWING.
//
// This closes the last hole, and it is one I put there. Rule 1 said yaw only
// ever happens in discrete turns, because every version that oscillated
// computed a yaw rate continuously from a RANGE reading -- a loop through the
// wall geometry. That reasoning was right, and removing that loop is what made
// corners work. But "no yaw at all" went too far: with nothing holding the
// heading, the aircraft yaws slowly on its own, and a nose that drifts curves
// the flight path into the wall faster than a sideways trim can push it out.
//
// Measured, on the flight that prompted this: over four seconds of ordinary
// following the heading wandered from -167 to -157 degrees, and the wall came
// in from 380mm to 280mm with the trim pushing the other way the whole time.
// It then tried to stop for the next corner at 280mm and went over -- the same
// too-close condition that the standoff fix was meant to have removed.
//
// The distinction that matters: this loop is closed on the ESTIMATOR'S OWN YAW,
// not on a range. It answers "am I pointing where I was pointing", which has
// nothing to do with where the wall is, so it cannot fight the wall or oscillate
// against the corner geometry. It is a heading hold, not a wall follower.
//
// Deliberately weak and capped. Correcting ten degrees takes a couple of
// seconds, which is all the authority needed to cancel a 2.5 deg/s drift, and
// far too little to throw the aircraft at anything.
#define CF_HOLD_GAIN        0.5f   // deg/s per degree of heading error
#define CF_HOLD_MAX_DEG    10.0f   // ceiling on the correction

// TURN THE HELD HEADING TOWARDS PARALLEL, slowly.
//
// Holding a heading fixed the drift. It did not fix being pointed the wrong way
// in the first place, and the flight that prompted this shows the cost. The
// aircraft was placed about 13 degrees off parallel to the wall, and the hold
// faithfully kept it there for the whole leg -- so it crabbed: flying forward
// at 13 degrees pushes it sideways at 45mm/s, and the trim's whole authority
// went on cancelling that instead of closing the 100mm gap it had started with.
// Net progress was 12mm/s. Reaching the target distance would have taken 13
// seconds and the corner arrived in six, so it turned at 300mm and went over.
//
// The correction needs no new sensor. A persistent sideways command IS the
// symptom: if the aircraft must keep strafing away from the wall to hold
// station, its nose is pointed into the wall. So the held heading is nudged in
// proportion to that command, which drives the strafe towards zero and the
// aircraft towards parallel.
//
// This is a loop through a range reading, which rule 1 warned about -- but at
// two degrees per second, against a corner turn of thirty. It cannot oscillate
// at any timescale the aircraft can fly, and it stops as soon as the strafe
// does.
#define CF_ALIGN_RATE     0.035f   // radians of held heading per (m/s) of strafe

// HOW CLOSE IS TOO CLOSE TO TURN.
//
// Three crashes have now happened the same way: a corner taken 280-300mm from
// the wall alongside. Stopping pitches the aircraft up, turning sweeps its
// rotors, and that close to a wall the two together are what puts it on its
// back. The standoff fix moved the stop further from the wall AHEAD; this is
// the wall BESIDE, which it never addressed.
//
// So a corner that arrives while too close is not refused, it is postponed:
// ease out to the holding distance first, then stop and turn. Costs a second.
#define CF_TURN_MIN_SIDE   0.35f   // metres; under this, back off before turning
#define CF_BACKOFF_MS      3.0f    // and give up easing out after this long

static float ref_distance = 0.4f;
static float max_speed = 0.2f;

static int   state = CF_FOLLOW;
static float state_start = 0.0f;
static float goal_heading = 0.0f;
static float hold_heading = 0.0f;   // the heading FOLLOW keeps, see CF_HOLD_GAIN
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

// Shortest way round. Without this a turn across the +/-180 seam reads as a
// 350 degree error and the aircraft spins the long way to gain 10.
static float wrapToPi(float a)
{
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

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
    hold_heading = currentHeading;
    first_run = false;
  }

  // Track the heading whenever we are NOT following, and freeze it the moment
  // we are. So the heading FOLLOW holds is whatever the last turn ended on,
  // with no extra bookkeeping to get out of step.
  if (state != CF_FOLLOW) hold_heading = currentHeading;

  const float dir = (float)direction;

  // A zero reading means the laser saw nothing, which at these ranges means
  // far away rather than touching. Everything below reads zero as open space.
  bool frontBlocked = frontRange > 0.0f && frontRange < ref_distance + CF_STOP_MARGIN;
  bool frontClear   = frontRange <= 0.0f || frontRange > ref_distance + CF_CLEAR_MARGIN;
  bool sideGone     = sideRange  <= 0.0f || sideRange  > ref_distance + CF_LOST_MARGIN;
  bool sideFound    = sideRange  >  0.0f && sideRange  < ref_distance + CF_FOUND_MARGIN;

  float vx = 0.0f, vy = 0.0f, wDeg = 0.0f;
  float inState = now - state_start;

  switch (state) {

  case CF_FOLLOW: {
    if (frontBlocked) {
      // Too close to turn here. See CF_TURN_MIN_SIDE.
      if (sideRange > 0.0f && sideRange < CF_TURN_MIN_SIDE) {
        state = enter(CF_BACKOFF, now);
      } else {
        state = enter(CF_STOP, now);
      }
      break;
    }
    if (sideGone)     { past_travelled = 0.0f; state = enter(CF_PAST, now); break; }

    // Hold the asked-for distance by strafing, never by turning. Positive
    // error means too far off the wall, so move toward it: the wall is +y in
    // the body frame when it is on the left (direction -1), -y on the right.
    float err = sideRange - ref_distance;
    float trim = err * CF_TRIM_GAIN;
    float lim = max_speed * 0.5f;
    if (trim >  lim) trim =  lim;
    if (trim < -lim) trim = -lim;

    // Hold the heading the last turn ended on. Closed on our own yaw, never
    // on a range -- see CF_HOLD_GAIN for why that distinction is the whole
    // point.
    float hErrDeg = wrapToPi(hold_heading - currentHeading) * RAD2DEG_F;
    wDeg = hErrDeg * CF_HOLD_GAIN;
    if (wDeg >  CF_HOLD_MAX_DEG) wDeg =  CF_HOLD_MAX_DEG;
    if (wDeg < -CF_HOLD_MAX_DEG) wDeg = -CF_HOLD_MAX_DEG;

    vx = max_speed;
    vy = -dir * trim;
    turns_without_following = 0;

    // Towards parallel. See CF_ALIGN_RATE: a sideways command that will not go
    // away means the nose is wrong, so lean the held heading into it. The sign
    // works out the same on both walls -- strafing right wants a lower heading,
    // strafing left a higher one, whichever side the wall is on.
    hold_heading = wrapToPi(hold_heading + vy * CF_ALIGN_RATE);
    break;
  }

  case CF_BACKOFF:
    // Ease straight out from the wall, holding heading, until there is room to
    // turn. No forward speed: the wall ahead is already inside the stopping
    // distance, and the only thing wanted here is the other axis.
    //
    // It gives up after CF_BACKOFF_MS and turns anyway. That is deliberate. If
    // the gap will not open, the aircraft is in a space too tight to fix, and
    // turning from a bad position still beats hovering into the timer with a
    // wall in front of it.
    if (sideRange <= 0.0f || sideRange >= ref_distance ||
        inState > CF_BACKOFF_MS) {
      state = enter(CF_STOP, now);
      break;
    }
    vx = 0.0f;
    vy = -dir * (max_speed * 0.5f);   // away from the wall, half speed
    break;

  case CF_STOP:
    // Everything stays zero: this step asks for a halt and waits for the ramp
    // in the caller to deliver one, so the nose eases down instead of dipping.
    if (inState >= CF_STOP_S) {
      // Inward corner. Turn AWAY from the wall beside us, which puts the wall
      // that was ahead alongside instead: right when following on the left.
      goal_heading = wrapToPi(currentHeading + dir * QUARTER);
      turns_without_following++;
      state = enter(CF_TURN, now);
    }
    break;

  case CF_TURN: {
    float errDeg = wrapToPi(goal_heading - currentHeading) * RAD2DEG_F;
    if (errDeg < CF_TURN_TOL_DEG && errDeg > -CF_TURN_TOL_DEG) {
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
      state = enter(CF_FOLLOW, now);          // the corner is behind us
    } else if (!frontClear) {
      // Still blocked. Not a corner: a dead end, or a gap narrower than the
      // turn. Another 90 makes a 180, which is the way back out -- the right
      // answer to a dead end, and cheap to be wrong about.
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
      // It came back: that was a doorway or a gap, not a corner. The most
      // common thing in a real room, and the blend treated every one of them
      // as a corner to be rounded.
      state = enter(CF_FOLLOW, now);
      break;
    }
    if (frontBlocked) { state = enter(CF_STOP, now); break; }

    vx = max_speed;
    past_travelled += max_speed * 0.1f;        // decisions run at 10Hz
    if (past_travelled >= CF_PAST_M) {
      // Outward corner: turn INTO where the wall went.
      goal_heading = wrapToPi(currentHeading - dir * QUARTER);
      turns_without_following++;
      state = enter(CF_TURN, now);
    }
    break;

  case CF_REACQ:
    // Creep, do not fly. If a wall is about to appear alongside, speed is the
    // last thing wanted.
    if (sideFound)      { state = enter(CF_FOLLOW, now); break; }
    if (frontBlocked)   { state = enter(CF_STOP, now);   break; }
    if (inState > CF_STEP_TIMEOUT_S) { state = enter(CF_GAVEUP, now); break; }
    vx = CF_CREEP_MS;
    break;

  case CF_GAVEUP:
  default:
    // Hold station. The mission turns for home at half the timer whatever the
    // follower is doing, and the hard time limit lands it regardless, so
    // standing still is both safe and self-clearing.
    break;
  }

  // Going in circles. Several turns without once settling back into following
  // means the room is not what this script assumes, and another turn will not
  // help.
  if (turns_without_following > CF_MAX_TURNS && state != CF_GAVEUP) {
    state = enter(CF_GAVEUP, now);
    vx = 0.0f; vy = 0.0f; wDeg = 0.0f;
  }

  *velX = vx;
  *velY = vy;
  *velW = wDeg * DEG2RAD_F;
  return state;
}
