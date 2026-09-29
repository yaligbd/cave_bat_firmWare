/*
 * wallfollowing_tudelft.h
 *
 * The wall follower from TU Delft's SGBA_CF2_App_layer, the implementation used
 * for the real-world flights in:
 *
 *   K.N. McGuire, C. De Wagter, K. Tuyls, H.J. Kappen, G.C.H.E. de Croon,
 *   "Minimal navigation solution for a swarm of tiny flying robots to explore
 *   an unknown environment", Science Robotics 4(35), 23 October 2019.
 *
 * Source: https://github.com/tudelft/SGBA_CF2_App_layer
 *
 * This is NOT the same file as wallfollowing_multiranger_onboard.c, although
 * the algorithm is the same and the authors overlap. See the notes at the top
 * of the .c for the three places they differ and why those differences matter
 * in a small arena.
 */

#ifndef SRC_WALLFOLLOWING_TUDELFT_H_
#define SRC_WALLFOLLOWING_TUDELFT_H_

#include <stdint.h>
#include <stdbool.h>

// States, as the original numbers them. Kept as integers rather than an enum
// because the recording stores the state in four bits and the numbering is
// what makes a downloaded flight readable.
//
//   1 forward            5 forward along wall
//   2 hover              6 rotate around wall
//   3 turn to find wall  7 rotate in corner
//   4 turn to align      8 find corner
#define TUD_FORWARD            1
#define TUD_HOVER              2
#define TUD_TURN_TO_FIND_WALL  3
#define TUD_TURN_TO_ALIGN      4
#define TUD_FORWARD_ALONG_WALL 5
#define TUD_ROTATE_AROUND_WALL 6
#define TUD_ROTATE_IN_CORNER   7
#define TUD_FIND_CORNER        8

void wallFollowerTudelftInit(float refDistanceFromWall, float maxSpeed, int initState);

/**
 * One tick of the state machine.
 *
 * Returns the state it is now in, and writes the body-frame velocities and the
 * yaw rate it wants. `now` is seconds, counting up, and only differences
 * between calls matter. The original read the clock itself; it is passed in
 * here so the caller owns the time source, which changes no behaviour.
 */
int wallFollowerTudelft(float *velX, float *velY, float *velW,
                        float frontRange, float sideRange,
                        float currentHeading, int directionTurn, float now);

#endif /* SRC_WALLFOLLOWING_TUDELFT_H_ */
