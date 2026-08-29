// cavebat_v2.c - autonomous survey flight with onboard recording.
//
// THE FLIGHT
//   1. App writes mission.timer and mission.height, then mission.state = 1.
//   2. App may disconnect. Nothing below needs the link -- the whole flight
//      runs on the drone. See "WHY NO LINK IS NEEDED" at the bottom.
//   3. Take off to mission.height.
//   4. OUTBOUND for timer/2: follow walls, keeping mission.walldist from the
//      wall on the chosen side, recording one sample per second.
//   5. RETURN: fly the recorded positions back in reverse.
//   6. Land. mission.state becomes 3 (data ready).
//   7. App reconnects and pulls the samples over CRTP port 14.
//
// TWO COMMANDERS, HANDED OVER CAREFULLY
//   Takeoff and landing use the high-level commander (trajectories). Wall
//   following and the return leg use low-level velocity setpoints. These are
//   not additive: commanderSetSetpoint() at a priority above the high-level
//   one calls crtpCommanderHighLevelStop(), cancelling any trajectory. So each
//   phase must own the drone exclusively, and the handover happens only at a
//   phase boundary -- never both in the same control cycle.
//
// HOW ACCURATE IS "RETURN TO START"
//   Not exact, and it cannot be. Position comes from the Flow deck's optical
//   flow, which drifts continuously with no external reference to correct it.
//   Over a minute of moving flight the error is realistically tens of
//   centimetres. The return leg replays recorded waypoints in reverse, so the
//   drone lands NEAR where it started, not on the spot.

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "param.h"
#include "log.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "stabilizer_types.h"
#include "crtp.h"
#include "usec_time.h"

#include "supervisor.h"

#include "wallfollowing_multiranger_onboard.h"

#define DEBUG_MODULE "CAVEBAT"
#include "debug.h"

// --- Control rates --------------------------------------------------------
// The wall follower is a velocity controller and needs to be fed steadily;
// the demo it comes from runs at 100Hz and so do we.
#define LOOP_PERIOD_MS   10
#define SAMPLE_PERIOD_MS 1000   // one recorded measurement per second

// --- Recording ------------------------------------------------------------
// 14 bytes, and that number is not arbitrary. A download packet is
// 1 header + 2 index + sizeof(FlightSample). At 14 that totals 17 bytes, which
// fits inside a single 20-byte BLE notification. Anything larger is split
// across two notifications, and split packets arrive corrupted on this
// hardware (the nRF51's s130 stack caps ATT_MTU at 23 and cannot be raised).
// If you add a field here, check that total again.
typedef struct __attribute__((packed)) {
  int16_t  x, y, z;                   // position, mm
  uint16_t front, back, left, right;  // wall distances, mm (0 = out of range)
} FlightSample;

// 180 samples at 1Hz is three minutes of flight, comfortably over the one
// minute we actually fly. 180 * 14 = 2.5KB of static RAM.
#define MAX_SAMPLES 180
static FlightSample flight_log[MAX_SAMPLES];
static uint16_t sample_count = 0;

// --- Download protocol, CRTP port 14 --------------------------------------
#define CRTP_PORT_BULK  14
#define BULK_CHAN_CTRL  0
#define BULK_CHAN_DATA  1

#define CMD_DUMP_START  0x01   // app -> drone: send me everything
#define CMD_CLEAR_MEM   0x02   // app -> drone: discard the recording
#define CMD_EOF         0xFF   // drone -> app: that was the last sample

static bool dump_requested  = false;
static bool clear_requested = false;

// --- Mission parameters, written by the app -------------------------------
// 0 = idle, 1 = start mission, 2 = abort, 3 = landed with data ready
static uint8_t  mission_state  = 0;
static uint32_t mission_timer  = 30;    // seconds, whole flight, out and back
static uint32_t mission_height = 500;   // mm
static uint32_t mission_walldist = 400; // mm to hold from a wall
static uint8_t  mission_goleft = 1;     // 1 = wall on the left, 0 = right
// 0 = hold position during the outbound leg, 1 = follow walls.
//
// Defaults to OFF. The first v2 flight crashed during takeoff, and with wall
// following enabled there is no way to tell whether the fault is in the
// takeoff, the handover between the two commanders, or the wall follower. With
// it off the flight is takeoff -> hold -> retrace -> land: the same shape as
// the real mission with one fewer thing that can be wrong.
static uint8_t  mission_wallfollow = 0;

// --- Telemetry, readable by the app ---------------------------------------
static uint16_t tele_alive  = 0;
static uint16_t tele_vbat   = 0;
static uint16_t tele_front  = 0;
static uint16_t tele_back   = 0;
static uint16_t tele_left   = 0;
static uint16_t tele_right  = 0;
static uint16_t tele_up     = 0;
static uint16_t tele_down   = 0;
static int16_t  tele_x      = 0;
static int16_t  tele_y      = 0;
static int16_t  tele_z      = 0;
static uint16_t tele_samples = 0;   // how many samples are stored
static uint8_t  tele_phase   = 0;   // see Phase below
// Bit 0 crashed, bit 1 tumbled, bit 2 can fly, bit 3 is flying. The drone's own
// verdict on itself, so a bad flight can be read back rather than guessed at.
static uint8_t  tele_sup     = 0;

typedef enum {
  PHASE_IDLE     = 0,
  PHASE_TAKEOFF  = 1,
  PHASE_OUTBOUND = 2,
  PHASE_RETURN   = 3,
  PHASE_LANDING  = 4,
  PHASE_READY    = 5,   // landed, data waiting to be downloaded
} Phase;

// --- Helpers --------------------------------------------------------------

static uint16_t clampRange(float mm) {
  // 0 means "nothing in range". The multiranger reports ~32766 for no echo,
  // and treating that as an obstacle at 32 metres would be worse than useless.
  if (mm <= 0.0f || mm > 3000.0f) return 0;
  return (uint16_t)mm;
}

// Velocity in the drone's OWN frame (forward / left), height held absolutely.
// This is what the wall follower produces.
static void setBodyVelocity(setpoint_t *sp, float vx, float vy, float z, float yawRateDeg) {
  memset(sp, 0, sizeof(setpoint_t));
  sp->mode.z = modeAbs;
  sp->position.z = z;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = yawRateDeg;
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->velocity.x = vx;
  sp->velocity.y = vy;
  sp->velocity_body = true;
}

// Velocity in WORLD coordinates, which is what the return leg needs: it steers
// toward a remembered x/y, and those were recorded in world space.
static void setWorldVelocity(setpoint_t *sp, float vx, float vy, float z) {
  memset(sp, 0, sizeof(setpoint_t));
  sp->mode.z = modeAbs;
  sp->position.z = z;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = 0.0f;
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->velocity.x = vx;
  sp->velocity.y = vy;
  sp->velocity_body = false;
}

static void recordSample(void) {
  if (sample_count >= MAX_SAMPLES) return;   // full: keep the earliest flight
  FlightSample *s = &flight_log[sample_count];
  s->x = tele_x;
  s->y = tele_y;
  s->z = tele_z;
  s->front = tele_front;
  s->back  = tele_back;
  s->left  = tele_left;
  s->right = tele_right;
  sample_count++;
  tele_samples = sample_count;
}

// Runs in CRTP task context, so it only raises a flag. The actual transfer
// happens in the main loop where blocking is safe.
static void bulkCrtpCb(CRTPPacket *p) {
  if (p->channel == BULK_CHAN_CTRL && p->size >= 1) {
    if (p->data[0] == CMD_DUMP_START) dump_requested = true;
    if (p->data[0] == CMD_CLEAR_MEM)  clear_requested = true;
  }
}

// One sample per packet, index included so a dropped packet is detectable by
// the app rather than silently shifting every later sample.
static void sendRecording(void) {
  DEBUG_PRINT("CAVEBAT: sending %d samples\n", (int)sample_count);
  CRTPPacket p;

  for (uint16_t i = 0; i < sample_count; i++) {
    p.port = CRTP_PORT_BULK;
    p.channel = BULK_CHAN_DATA;
    p.size = 2 + sizeof(FlightSample);
    p.data[0] = i & 0xff;
    p.data[1] = (i >> 8) & 0xff;
    memcpy(&p.data[2], &flight_log[i], sizeof(FlightSample));
    crtpSendPacketBlock(&p);
  }

  p.port = CRTP_PORT_BULK;
  p.channel = BULK_CHAN_CTRL;
  p.size = 3;
  p.data[0] = CMD_EOF;
  p.data[1] = sample_count & 0xff;
  p.data[2] = (sample_count >> 8) & 0xff;
  crtpSendPacketBlock(&p);

  DEBUG_PRINT("CAVEBAT: send complete\n");
}

// --- Main -----------------------------------------------------------------

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

  crtpRegisterPortCB(CRTP_PORT_BULK, bulkCrtpCb);

  DEBUG_PRINT("CAVEBAT v2: ready\n");

  Phase phase = PHASE_IDLE;
  setpoint_t setpoint;
  StateWF wfState = forward;

  uint32_t phase_started    = 0;   // tick the current phase began
  uint32_t last_sample_tick = 0;
  uint32_t outbound_ms      = 0;   // half the mission timer
  float    target_height_m  = 0.0f;
  int16_t  return_index     = 0;   // walks backwards through flight_log
  uint32_t waypoint_started = 0;

  while (1) {
    vTaskDelay(M2T(LOOP_PERIOD_MS));
    const uint32_t now = xTaskGetTickCount();

    // --- telemetry, every cycle ------------------------------------------
    tele_alive++;
    tele_vbat  = (uint16_t)(logGetFloat(idVbat) * 1000.0f);
    tele_front = clampRange(logGetFloat(idFront));
    tele_back  = clampRange(logGetFloat(idBack));
    tele_left  = clampRange(logGetFloat(idLeft));
    tele_right = clampRange(logGetFloat(idRight));
    tele_up    = clampRange(logGetFloat(idUp));
    tele_down  = clampRange(logGetFloat(idDown));
    tele_x     = (int16_t)(logGetFloat(idX) * 1000.0f);
    tele_y     = (int16_t)(logGetFloat(idY) * 1000.0f);
    tele_z     = (int16_t)(logGetFloat(idZ) * 1000.0f);
    tele_phase = (uint8_t)phase;
    tele_sup = (supervisorIsCrashed() ? 1 : 0)
             | (supervisorIsTumbled() ? 2 : 0)
             | (supervisorCanFly()    ? 4 : 0)
             | (supervisorIsFlying()  ? 8 : 0);

    // The supervisor's verdict overrides this state machine. Without this the
    // mission carried on regardless: the first v2 flight logged "Crashed,
    // recovery required" during takeoff, then flew the outbound leg, turned
    // around and recorded six samples while lying on the floor.
    if ((phase == PHASE_TAKEOFF || phase == PHASE_OUTBOUND ||
         phase == PHASE_RETURN  || phase == PHASE_LANDING) &&
        (supervisorIsCrashed() || supervisorIsTumbled())) {
      DEBUG_PRINT("CAVEBAT: CRASH in phase %d, stopping. %d samples kept\n",
                  (int)phase, (int)sample_count);
      crtpCommanderHighLevelStop();
      phase = PHASE_READY;
      mission_state = 3;
      continue;
    }

    // --- download requests, safe in any phase ----------------------------
    if (clear_requested) {
      clear_requested = false;
      sample_count = 0;
      tele_samples = 0;
      if (phase == PHASE_READY) { phase = PHASE_IDLE; mission_state = 0; }
      DEBUG_PRINT("CAVEBAT: recording cleared\n");
    }
    if (dump_requested) {
      dump_requested = false;
      sendRecording();
    }

    // --- abort, from any flying phase ------------------------------------
    if (mission_state == 2 && phase != PHASE_IDLE && phase != PHASE_READY) {
      DEBUG_PRINT("CAVEBAT: aborted, landing\n");
      crtpCommanderHighLevelLand(0.0f, 2.0f);
      vTaskDelay(M2T(2500));
      phase = PHASE_READY;
      mission_state = 3;
      continue;
    }

    switch (phase) {

    case PHASE_IDLE:
      if (mission_state == 1) {
        sample_count = 0;
        tele_samples = 0;
        target_height_m = mission_height / 1000.0f;
        outbound_ms = (mission_timer * 1000) / 2;

        // Wall follower wants metres. Re-initialised per flight so a previous
        // mission's state cannot leak into this one.
        wfState = forward;
        wallFollowerInit(mission_walldist / 1000.0f, 0.3f, wfState);

        float climb = target_height_m / 0.3f;      // ~0.3 m/s
        if (climb < 1.0f) climb = 1.0f;
        DEBUG_PRINT("CAVEBAT: takeoff to %d mm, mission %d s, battery %d mV\n",
                    (int)mission_height, (int)mission_timer, (int)tele_vbat);
        crtpCommanderHighLevelTakeoff(target_height_m, climb);

        phase = PHASE_TAKEOFF;
        phase_started = now;
        last_sample_tick = now;
      }
      break;

    case PHASE_TAKEOFF: {
      // Hand over to velocity control once at altitude, or after a generous
      // timeout so a slightly-short climb cannot strand the mission.
      float z = logGetFloat(idZ);
      bool at_height = (z > target_height_m - 0.1f);
      bool timed_out = ((now - phase_started) > M2T(6000));
      if (at_height || timed_out) {
        DEBUG_PRINT("CAVEBAT: outbound, %s\n",
                    mission_wallfollow
                      ? (mission_goleft ? "following wall on the left"
                                        : "following wall on the right")
                      : "holding position (wall following off)");
        phase = PHASE_OUTBOUND;
        phase_started = now;
      }
      break;
    }

    case PHASE_OUTBOUND: {
      if (!mission_wallfollow) {
        // Hold station. Zero velocity still has to be sent every cycle: once we
        // have taken over from the high-level commander, our setpoint is what
        // keeps the supervisor's watchdog fed.
        setWorldVelocity(&setpoint, 0.0f, 0.0f, target_height_m);
        commanderSetSetpoint(&setpoint, 3);
        if ((now - phase_started) > M2T(outbound_ms)) {
          DEBUG_PRINT("CAVEBAT: turning back, %d samples out\n", (int)sample_count);
          phase = PHASE_RETURN;
          phase_started = now;
          return_index = (int16_t)sample_count - 2;
          waypoint_started = now;
        }
        break;
      }

      float frontRange = (float)tele_front / 1000.0f;
      float sideRange  = (float)(mission_goleft ? tele_left : tele_right) / 1000.0f;
      // The wall follower treats 0 as "touching a wall". Our 0 means "nothing
      // within range", so hand it a large number instead.
      if (tele_front == 0) frontRange = 3.0f;
      if ((mission_goleft ? tele_left : tele_right) == 0) sideRange = 3.0f;

      float yawRad = logGetFloat(idYaw) * (float)M_PI / 180.0f;
      float vx = 0.0f, vy = 0.0f, yawRateRad = 0.0f;
      float timeNow = usecTimestamp() / 1e6f;

      wfState = wallFollower(&vx, &vy, &yawRateRad, frontRange, sideRange,
                             yawRad, mission_goleft ? 1 : -1, timeNow);

      setBodyVelocity(&setpoint, vx, vy, target_height_m,
                      yawRateRad * 180.0f / (float)M_PI);
      commanderSetSetpoint(&setpoint, 3);

      if ((now - phase_started) > M2T(outbound_ms)) {
        DEBUG_PRINT("CAVEBAT: turning back, %d samples out\n", (int)sample_count);
        phase = PHASE_RETURN;
        phase_started = now;
        // Retrace from the most recent sample backwards. The last one is where
        // we are standing right now, so start one before it.
        return_index = (int16_t)sample_count - 2;
        waypoint_started = now;
      }
      break;
    }

    case PHASE_RETURN: {
      if (return_index < 0) {
        DEBUG_PRINT("CAVEBAT: back at the start, landing\n");
        phase = PHASE_LANDING;
        phase_started = now;
        break;
      }

      float tx = flight_log[return_index].x / 1000.0f;
      float ty = flight_log[return_index].y / 1000.0f;
      float cx = logGetFloat(idX);
      float cy = logGetFloat(idY);
      float dx = tx - cx, dy = ty - cy;
      float dist = sqrtf(dx * dx + dy * dy);

      // Close enough, or spent too long on this one. The timeout matters:
      // drift means a waypoint can be unreachable, and without it the drone
      // would hover at that spot until the battery died.
      if (dist < 0.15f || (now - waypoint_started) > M2T(4000)) {
        return_index--;
        waypoint_started = now;
        break;
      }

      const float RETURN_SPEED = 0.3f;
      float vx = (dx / dist) * RETURN_SPEED;
      float vy = (dy / dist) * RETURN_SPEED;

      // Retracing a path we already flew, so it should be clear -- but drift
      // can put us into a wall that was not there on the way out. Refuse to
      // drive forward into something close; the waypoint timeout above then
      // moves us on rather than leaving us stuck against it.
      if (tele_front > 0 && tele_front < 250) {
        vx = 0.0f;
        vy = 0.0f;
      }

      setWorldVelocity(&setpoint, vx, vy, target_height_m);
      commanderSetSetpoint(&setpoint, 3);
      break;
    }

    case PHASE_LANDING: {
      float land = target_height_m / 0.3f;
      if (land < 1.0f) land = 1.0f;
      crtpCommanderHighLevelLand(0.0f, land);
      vTaskDelay(M2T((uint32_t)(land * 1000) + 500));
      DEBUG_PRINT("CAVEBAT: FLIGHT DONE, %d samples ready to download\n",
                  (int)sample_count);
      phase = PHASE_READY;
      mission_state = 3;
      break;
    }

    case PHASE_READY:
      // Landed with data. Waits here until the app clears the recording, so a
      // flight is never silently overwritten before it has been collected.
      break;
    }

    // --- one sample per second, while airborne ---------------------------
    if ((phase == PHASE_OUTBOUND || phase == PHASE_RETURN) &&
        (now - last_sample_tick) >= M2T(SAMPLE_PERIOD_MS)) {
      last_sample_tick = now;
      recordSample();
    }

    // Heartbeat every 10s. Deliberately not every second: over BLE a status
    // line every second saturates the link and queues real replies behind it.
    if ((tele_alive % 1000) == 0) {
      DEBUG_PRINT("CB phase=%d bat=%d n=%d x=%d y=%d z=%d\n",
                  (int)phase, (int)tele_vbat, (int)sample_count,
                  (int)tele_x, (int)tele_y, (int)tele_z);
    }
  }
}

// WHY NO LINK IS NEEDED
// The stabilizer pulls a setpoint every cycle and that refreshes the
// supervisor's watchdog, so nothing here depends on the phone or the radio
// being connected. Verified in the firmware source: stabilizer.c calls
// crtpCommanderHighLevelGetSetpoint() each loop, and while a trajectory is
// active the planner keeps returning one; during the velocity phases this file
// calls commanderSetSetpoint() at 100Hz itself. Either way the watchdog stays
// fed with the app disconnected.

// --- Parameters -----------------------------------------------------------
PARAM_GROUP_START(mission)
  PARAM_ADD(PARAM_UINT8,  state,    &mission_state)
  PARAM_ADD(PARAM_UINT32, timer,    &mission_timer)
  PARAM_ADD(PARAM_UINT32, height,   &mission_height)
  PARAM_ADD(PARAM_UINT32, walldist, &mission_walldist)
  PARAM_ADD(PARAM_UINT8,  goleft,   &mission_goleft)
  PARAM_ADD(PARAM_UINT8,  wallfollow, &mission_wallfollow)
PARAM_GROUP_STOP(mission)

PARAM_GROUP_START(tele)
  PARAM_ADD(PARAM_UINT16, alive,   &tele_alive)
  PARAM_ADD(PARAM_UINT16, vbat,    &tele_vbat)
  PARAM_ADD(PARAM_UINT16, front,   &tele_front)
  PARAM_ADD(PARAM_UINT16, back,    &tele_back)
  PARAM_ADD(PARAM_UINT16, left,    &tele_left)
  PARAM_ADD(PARAM_UINT16, right,   &tele_right)
  PARAM_ADD(PARAM_UINT16, up,      &tele_up)
  PARAM_ADD(PARAM_UINT16, down,    &tele_down)
  PARAM_ADD(PARAM_INT16,  x,       &tele_x)
  PARAM_ADD(PARAM_INT16,  y,       &tele_y)
  PARAM_ADD(PARAM_INT16,  z,       &tele_z)
  PARAM_ADD(PARAM_UINT16, samples, &tele_samples)
  PARAM_ADD(PARAM_UINT8,  phase,   &tele_phase)
  PARAM_ADD(PARAM_UINT8,  sup,     &tele_sup)
PARAM_GROUP_STOP(tele)

// --- Log variables --------------------------------------------------------
// Keep any log block the app builds from these to FIVE variables or fewer:
// a create packet is 3 + 3*N bytes and must fit one 20-byte BLE notification.
LOG_GROUP_START(tele)
  LOG_ADD(LOG_UINT16, vbat,    &tele_vbat)
  LOG_ADD(LOG_UINT16, samples, &tele_samples)
  LOG_ADD(LOG_UINT8,  phase,   &tele_phase)
  LOG_ADD(LOG_UINT8,  sup,     &tele_sup)
  LOG_ADD(LOG_INT16,  x,       &tele_x)
  LOG_ADD(LOG_INT16,  y,       &tele_y)
  LOG_ADD(LOG_INT16,  z,       &tele_z)
LOG_GROUP_STOP(tele)
