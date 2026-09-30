/*
 * wallfollowing_corners.h
 *
 * A corner is a script with an end, not a blend of states that hand off to one
 * another. See the .c for what this replaces and why.
 */

#ifndef SRC_WALLFOLLOWING_CORNERS_H_
#define SRC_WALLFOLLOWING_CORNERS_H_

#include <stdint.h>
#include <stdbool.h>

// Numbered from 1 so a recorded 0 still reads as "never ran".
#define CF_FOLLOW  1  // wall beside me, flying along it
#define CF_STOP    2  // wall ahead: stopping before I turn
#define CF_TURN    3  // turning 90 degrees on the spot
#define CF_VERIFY  4  // did that turn actually work?
#define CF_PAST    5  // wall ended: driving out past the corner
#define CF_REACQ   6  // creeping forward until the wall comes back
#define CF_GAVEUP  7  // holding position, see the .c
#define CF_BACKOFF 8  // too close to the wall to turn: easing out first

void wallFollowerCornersInit(float refDistanceFromWall, float maxSpeed);

/**
 * One tick. Returns the step it is now in, and writes body-frame velocities
 * and a yaw rate in RADIANS/s, matching the other two followers in this tree.
 *
 * `direction` is +1 with the wall on the right, -1 with the wall on the left,
 * the same convention the other followers use. `now` is seconds, counting up.
 */
int wallFollowerCorners(float *velX, float *velY, float *velW,
                        float frontRange, float sideRange,
                        float currentHeading, int direction, float now);

#endif /* SRC_WALLFOLLOWING_CORNERS_H_ */
