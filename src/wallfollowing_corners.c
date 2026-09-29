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
#define CF_CLEAR_MARGIN     0.20f  // ahead counts as clear past ref + this
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

static float ref_distance = 0.4f;
static float max_speed = 0.2f;

static int   state = CF_FOLLOW;
static float state_start = 0.0f;
static float goal_heading = 0.0f;
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
    first_run = false;
  }

  const float dir = (float)direction;

  // A zero reading means the laser saw nothing, which at these ranges means
  // far away rather than touching. Everything below reads zero as open space.
  bool frontBlocked = frontRange > 0.0f && frontRange < ref_distance;
  bool frontClear   = frontRange <= 0.0f || frontRange > ref_distance + CF_CLEAR_MARGIN;
  bool sideGone     = sideRange  <= 0.0f || sideRange  > ref_distance + CF_LOST_MARGIN;
  bool sideFound    = sideRange  >  0.0f && sideRange  < ref_distance + CF_FOUND_MARGIN;

  float vx = 0.0f, vy = 0.0f, wDeg = 0.0f;
  float inState = now - state_start;

  switch (state) {

  case CF_FOLLOW: {
    if (frontBlocked) { state = enter(CF_STOP, now); break; }
    if (sideGone)     { past_travelled = 0.0f; state = enter(CF_PAST, now); break; }

    // Hold the asked-for distance by strafing, never by turning. Positive
    // error means too far off the wall, so move toward it: the wall is +y in
    // the body frame when it is on the left (direction -1), -y on the right.
    float err = sideRange - ref_distance;
    float trim = err * CF_TRIM_GAIN;
    float lim = max_speed * 0.5f;
    if (trim >  lim) trim =  lim;
    if (trim < -lim) trim = -lim;

    vx = max_speed;
    vy = -dir * trim;
    wDeg = 0.0f;               // rule 1: no continuous yaw, ever
    turns_without_following = 0;
    break;
  }

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
