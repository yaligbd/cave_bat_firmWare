// cavebat.c - Telemetry and Autonomous Hover
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

  DEBUG_PRINT("CAVEBAT: Flight & Telemetry starting\n");
  
  uint32_t flight_start_time = 0;
  bool is_flying = false;

  crtpCommanderHighLevelInit();

  while (1) {
    // 1. Update Telemetry
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

    // 2. Flight State Machine
    if (mission_state == 1 && !is_flying) {
        // App requested Takeoff
        DEBUG_PRINT("CAVEBAT: Initiating Takeoff to %d mm\n", (int)mission_height);
        flight_start_time = xTaskGetTickCount();
        is_flying = true;
        
        float target_height_m = mission_height / 1000.0f;
        float takeoff_duration = target_height_m / 0.3f; // safe velocity 0.3 m/s
        if (takeoff_duration < 1.0f) takeoff_duration = 1.0f;
        
        crtpCommanderHighLevelTakeoff(target_height_m, takeoff_duration);
        
    } else if (mission_state == 1 && is_flying) {
        // App is in Fly mode -> Check Timer
        uint32_t current_time = xTaskGetTickCount();
        uint32_t elapsed_sec = (current_time - flight_start_time) / configTICK_RATE_HZ;
        
        if (elapsed_sec >= mission_timer) {
            DEBUG_PRINT("CAVEBAT: Timer complete. Landing.\n");
            
            float target_height_m = mission_height / 1000.0f;
            float land_duration = target_height_m / 0.3f;
            if (land_duration < 1.0f) land_duration = 1.0f;
            
            crtpCommanderHighLevelLand(0.0f, land_duration);
            vTaskDelay(M2T((uint32_t)(land_duration * 1000) + 500));
            
            mission_state = 0; // Reset to idle
            is_flying = false;
        } else {
            // High-level commander automatically maintains position (hovers)
            // after the takeoff trajectory is complete. No explicit API call needed.
            
            // Abort if an obstacle gets closer than 200mm (0 means no reading)
            if ((tele_front > 0 && tele_front < 200) ||
                (tele_back  > 0 && tele_back  < 200) ||
                (tele_left  > 0 && tele_left  < 200) ||
                (tele_right > 0 && tele_right < 200)) {
                DEBUG_PRINT("CAVEBAT: Obstacle detected < 200mm! Aborting.\n");
                mission_state = 2; // Trigger Abort
            }
        }
        
    } else if (mission_state == 2) {
        // App requested Abort
        DEBUG_PRINT("CAVEBAT: Mission Aborted! Landing immediately.\n");
        
        float target_height_m = mission_height / 1000.0f;
        float land_duration = target_height_m / 0.3f;
        if (land_duration < 1.0f) land_duration = 1.0f;
        
        crtpCommanderHighLevelLand(0.0f, land_duration);
        vTaskDelay(M2T((uint32_t)(land_duration * 1000) + 500));
        
        mission_state = 0; // Reset to idle
        is_flying = false;
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
    
    // Stream data at 10Hz (100ms) to ensure app captures 1 sample per second cleanly
    vTaskDelay(M2T(100));
  }
}

// --- Parameter Registration ---
PARAM_GROUP_START(tele)
  PARAM_ADD(PARAM_UINT16, alive, &tele_alive)
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
PARAM_GROUP_STOP(mission)

// --- Log Registration ---
LOG_GROUP_START(tele)
  LOG_ADD(LOG_UINT16, alive, &tele_alive)
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