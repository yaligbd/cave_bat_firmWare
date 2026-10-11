/*
 * wallfollowing_corners.h
 *
 * Corner-script wall follower, called by cavebat_mission.c in wall-follow
 * mode. Design rules and tuning are in the .c.
 */

#ifndef SRC_WALLFOLLOWING_CORNERS_H_
#define SRC_WALLFOLLOWING_CORNERS_H_

#include <stdint.h>
#include <stdbool.h>

// Step codes returned by wallFollowerCorners() and recorded in each flight
// sample. Numbered from 1 so a recorded 0 still reads as "never ran".
#define CF_FOLLOW  1  // flying along the wall, holding distance and heading
#define CF_STOP    2  // wall ahead: halting before a turn
#define CF_TURN    3  // turning 90 degrees on the spot
#define CF_VERIFY  4  // settling after a turn, then checking it worked
#define CF_PAST    5  // wall ended: driving out past the corner
#define CF_REACQ   6  // flying forward until the wall comes back
#define CF_GAVEUP  7  // holding position; terminal until the next init
#define CF_BACKOFF 8  // too close to the side wall to turn: easing out first

/**
 * Reset to CF_FOLLOW with the holding distance (m) and cruise speed (m/s).
 * Call before each wall-following flight.
 */
void wallFollowerCornersInit(float refDistanceFromWall, float maxSpeed);

/**
 * One decision tick, expected at 10 Hz (PAST and REACQ integrate distance on
 * that assumption). Returns the step it is now in, and writes body-frame
 * velocities in m/s (+x forward, +y left) and a yaw rate in rad/s (not deg/s).
 *
 * Ranges are in metres; zero or less means no reading (open space).
 * `currentHeading` is the yaw in radians. `direction` is +1 with the wall on
 * the right, -1 with the wall on the left. `now` is seconds, counting up; only
 * differences between calls matter.
 */
int wallFollowerCorners(float *velX, float *velY, float *velW,
                        float frontRange, float sideRange,
                        float currentHeading, int direction, float now);

#endif /* SRC_WALLFOLLOWING_CORNERS_H_ */
