/*
 * wallfollowing_tudelft.c
 *
 * TU Delft's wall follower, from https://github.com/tudelft/SGBA_CF2_App_layer
 * -- the code that flew the Science Robotics 2019 experiments: real Crazyflies
 * with a Flow deck and a Multi-ranger exploring a real office building and
 * returning to where they took off.
 *
 * Ported faithfully. The only change to behaviour is that `now` is passed in
 * rather than read from usecTimestamp(), so the caller owns the time source.
 *
 * ---------------------------------------------------------------------------
 * HOW THIS DIFFERS FROM wallfollowing_multiranger_onboard.c, WHICH IS ALSO HERE
 *
 * Both descend from the same work and most of the file is identical -- same
 * eight states, same transitions, same command functions, character for
 * character in places. I diffed them. There are exactly three differences, and
 * two of them are the reason this file is worth flying.
 *
 * 1. THE ESCAPE OUT OF "TURN TO FIND WALL" IS FAR STRICTER.
 *
 *      this file:  side < 1.0m  AND  front > 2.0m
 *      the other:  side < ref+0.2  AND  front > ref+0.3   (0.6m and 0.7m)
 *
 *    That escape drops the machine into FIND_CORNER, and FIND_CORNER ->
 *    ROTATE_AROUND_WALL -> TURN_TO_FIND_WALL is exactly the loop our recordings
 *    keep catching: a corner it circles instead of turning. The other file
 *    fires that escape whenever there is 70cm of space ahead, which near a
 *    corner is most of the time.
 *
 *    In a 2m arena, "front > 2.0m" is the entire width of the room, so this
 *    escape effectively cannot fire -- the loop is structurally unreachable.
 *    That is a happy accident of arena size, not a fix, and it is worth being
 *    honest that it would behave differently in a corridor.
 *
 * 2. IT IS SLOWER TO DECIDE THE WALL IS LOST WHILE ROUNDING A CORNER.
 *
 *      this file:  side > ref + 0.5
 *      the other:  side > ref + 0.3
 *
 *    20cm more patience before abandoning a corner it is halfway round.
 *
 * 3. Its default speed is 0.5 m/s against the other's 0.2. We keep 0.2 -- half
 *    a metre per second across a two-metre arena is four seconds wall to wall.
 *
 * WHAT IS NOT DIFFERENT, and should be said plainly: the one-second hover at
 * the start of TURN_TO_ALIGN is in BOTH. Flying Bitcraze's own Python version
 * on this aircraft, that second was measured commanding nothing at all -- no
 * velocity, no yaw rate -- while the aircraft drifted 40cm. This file will do
 * the same. It is the known remaining flaw going in.
 */

#include <math.h>

#include "wallfollowing_tudelft.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Their thresholds, named. The originals are bare numbers in the middle of the
// state machine; the values are unchanged.
#define TUD_FRONT_TURN_MARGIN   0.2f  // wall ahead this much past ref = corner
#define TUD_SIDE_LOST_MARGIN    0.3f  // side past ref + this = wall ended
#define TUD_CORNER_LOST_MARGIN  0.5f  // ... but 0.5 while rounding a corner
#define TUD_ESCAPE_SIDE_M       1.0f  // see note 1 at the top
#define TUD_ESCAPE_FRONT_M      2.0f  // see note 1 at the top
#define TUD_IN_CORNER_ANGLE     0.8f  // radians of turn that completes a corner
#define TUD_ANGLE_MARGIN        0.1f
#define TUD_ALIGN_SETTLE_S      1.0f  // the hover at the start of TURN_TO_ALIGN

static float ref_distance_from_wall = 0.0f;
static float max_speed = 0.5f;
static float max_rate = 0.5f;
static float direction = 1.0f;
static bool  first_run = false;
static int   state = TUD_FORWARD;
static float state_start_time = 0.0f;

// Set once per tick so transition() can stamp the state without reading a clock
// of its own. The original called usecTimestamp() here.
static float time_now = 0.0f;

void wallFollowerTudelftInit(float refDistanceFromWall, float maxSpeed, int initState)
{
  ref_distance_from_wall = refDistanceFromWall;
  max_speed = maxSpeed;
  first_run = true;
  state = initState;
  state_start_time = 0.0f;
}

static bool logicIsCloseTo(float real_value, float checked_value, float margin)
{
  return (real_value > checked_value - margin && real_value < checked_value + margin);
}

static float wraptopi(float number)
{
  if (number > (float)M_PI) {
    return (number - (float)(2 * M_PI));
  } else if (number < (float)(-1 * M_PI)) {
    return (number + (float)(2 * M_PI));
  } else {
    return number;
  }
}

static void commandTurn(float *vel_x, float *vel_w, float ref_rate)
{
  *vel_x = 0.0f;
  *vel_w = direction * ref_rate;
}

static void commandAlignCorner(float *vel_y, float *vel_w, float ref_rate, float range,
                               float wanted_distance_from_corner)
{
  if (range > wanted_distance_from_corner + 0.3f) {
    *vel_w = direction * ref_rate;
    *vel_y = 0.0f;
  } else {
    if (range > wanted_distance_from_corner) {
      *vel_y = direction * (-1.0f * max_speed / 3.0f);
    } else {
      *vel_y = direction * (max_speed / 3.0f);
    }
    *vel_w = 0.0f;
  }
}

static void commandHover(float *vel_x, float *vel_y, float *vel_w)
{
  *vel_x = 0.0f;
  *vel_y = 0.0f;
  *vel_w = 0.0f;
}

static void commandForwardAlongWall(float *vel_x, float *vel_y, float range)
{
  *vel_x = max_speed;
  bool check_distance_wall = logicIsCloseTo(ref_distance_from_wall, range, 0.1f);
  *vel_y = 0.0f;
  if (!check_distance_wall) {
    if (range > ref_distance_from_wall) {
      *vel_y = direction * (-1.0f * max_speed / 2.0f);
    } else {
      *vel_y = direction * (max_speed / 2.0f);
    }
  }
}

static void commandTurnAroundCornerAndAdjust(float *vel_x, float *vel_y, float *vel_w,
                                             float radius, float range)
{
  *vel_x = max_speed;
  *vel_w = direction * (-1.0f * (*vel_x) / radius);
  bool check_distance_to_wall = logicIsCloseTo(ref_distance_from_wall, range, 0.1f);
  if (!check_distance_to_wall) {
    if (range > ref_distance_from_wall) {
      *vel_y = direction * (-1.0f * max_speed / 3.0f);
    } else {
      *vel_y = direction * (max_speed / 3.0f);
    }
  }
}

static void commandTurnAndAdjust(float *vel_y, float *vel_w, float rate, float range)
{
  (void)range;
  *vel_w = direction * rate;
  *vel_y = 0.0f;
}

static int transition(int new_state)
{
  state_start_time = time_now;
  return new_state;
}

int wallFollowerTudelft(float *vel_x, float *vel_y, float *vel_w,
                        float front_range, float side_range,
                        float current_heading, int direction_turn, float now)
{
  static float previous_heading = 0.0f;
  static float angle = 0.0f;
  static bool around_corner_go_back = false;

  direction = (float)direction_turn;
  time_now = now;

  if (first_run) {
    previous_heading = current_heading;
    around_corner_go_back = false;
    state_start_time = now;
    first_run = false;
  }

  /* ---- state transitions ---------------------------------------------- */

  if (state == TUD_FORWARD) {
    if (front_range < ref_distance_from_wall + TUD_FRONT_TURN_MARGIN) {
      state = transition(TUD_TURN_TO_FIND_WALL);
    }

  } else if (state == TUD_HOVER) {
    /* nothing */

  } else if (state == TUD_TURN_TO_FIND_WALL) {
    bool side_range_check  = side_range  < ref_distance_from_wall / (float)cos(0.78f) + 0.2f;
    bool front_range_check = front_range < ref_distance_from_wall / (float)cos(0.78f) + 0.2f;
    if (side_range_check && front_range_check) {
      previous_heading = current_heading;
      angle = direction * (1.57f - (float)atan(front_range / side_range) + TUD_ANGLE_MARGIN);
      state = transition(TUD_TURN_TO_ALIGN);
    }
    // See note 1 at the top of this file: this is the escape that the other
    // implementation opens far too readily.
    if (side_range < TUD_ESCAPE_SIDE_M && front_range > TUD_ESCAPE_FRONT_M) {
      around_corner_go_back = false;
      previous_heading = current_heading;
      state = transition(TUD_FIND_CORNER);
    }

  } else if (state == TUD_TURN_TO_ALIGN) {
    bool allign_wall_check =
        logicIsCloseTo(wraptopi(current_heading - previous_heading), angle, TUD_ANGLE_MARGIN);
    if (allign_wall_check) {
      state = transition(TUD_FORWARD_ALONG_WALL);
    }

  } else if (state == TUD_FORWARD_ALONG_WALL) {
    if (side_range > ref_distance_from_wall + TUD_SIDE_LOST_MARGIN) {
      state = transition(TUD_FIND_CORNER);
    }
    if (front_range < ref_distance_from_wall + TUD_FRONT_TURN_MARGIN) {
      previous_heading = current_heading;
      state = transition(TUD_ROTATE_IN_CORNER);
    }

  } else if (state == TUD_ROTATE_AROUND_WALL) {
    if (front_range < ref_distance_from_wall + TUD_FRONT_TURN_MARGIN) {
      state = transition(TUD_TURN_TO_FIND_WALL);
    }

  } else if (state == TUD_ROTATE_IN_CORNER) {
    bool check_heading_corner = logicIsCloseTo(
        fabsf(wraptopi(current_heading - previous_heading)), TUD_IN_CORNER_ANGLE, TUD_ANGLE_MARGIN);
    if (check_heading_corner) {
      state = transition(TUD_TURN_TO_FIND_WALL);
    }

  } else if (state == TUD_FIND_CORNER) {
    if (side_range <= ref_distance_from_wall) {
      state = transition(TUD_ROTATE_AROUND_WALL);
    }
  }

  /* ---- state actions --------------------------------------------------- */

  float temp_vel_x = 0.0f;
  float temp_vel_y = 0.0f;
  float temp_vel_w = 0.0f;

  if (state == TUD_FORWARD) {
    temp_vel_x = max_speed;

  } else if (state == TUD_HOVER) {
    commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);

  } else if (state == TUD_TURN_TO_FIND_WALL) {
    commandTurn(&temp_vel_x, &temp_vel_w, max_rate);

  } else if (state == TUD_TURN_TO_ALIGN) {
    // The second that commands nothing. Theirs and Bitcraze's both do this.
    if (now - state_start_time < TUD_ALIGN_SETTLE_S) {
      commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
    } else {
      commandTurn(&temp_vel_x, &temp_vel_w, max_rate);
    }

  } else if (state == TUD_FORWARD_ALONG_WALL) {
    commandForwardAlongWall(&temp_vel_x, &temp_vel_y, side_range);

  } else if (state == TUD_ROTATE_AROUND_WALL) {
    if (side_range > ref_distance_from_wall + TUD_CORNER_LOST_MARGIN) {
      if (wraptopi(fabsf(current_heading - previous_heading)) > TUD_IN_CORNER_ANGLE) {
        around_corner_go_back = true;
      }
      if (around_corner_go_back) {
        commandTurnAndAdjust(&temp_vel_y, &temp_vel_w, -1.0f * max_rate, side_range);
      } else {
        commandTurnAndAdjust(&temp_vel_y, &temp_vel_w, max_rate, side_range);
      }
    } else {
      previous_heading = current_heading;
      around_corner_go_back = false;
      commandTurnAroundCornerAndAdjust(&temp_vel_x, &temp_vel_y, &temp_vel_w,
                                       ref_distance_from_wall, side_range);
    }

  } else if (state == TUD_ROTATE_IN_CORNER) {
    commandTurn(&temp_vel_x, &temp_vel_w, max_rate);

  } else if (state == TUD_FIND_CORNER) {
    commandAlignCorner(&temp_vel_y, &temp_vel_w, -1.0f * max_rate, side_range,
                       ref_distance_from_wall);

  } else {
    commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
  }

  *vel_x = temp_vel_x;
  *vel_y = temp_vel_y;
  *vel_w = temp_vel_w;

  return state;
}
