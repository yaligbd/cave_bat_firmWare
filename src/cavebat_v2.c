// cavebat_v2.c - Store-and-Forward Autonomous Telemetry
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "param.h"
#include "log.h"
#include "commander.h" 
#include "high_level_commander.h"
#include "crtp.h"

#define DEBUG_MODULE "CAVEBAT_V2"
#include "debug.h"

// --- Custom CRTP Port 14 Constants ---
#define CRTP_PORT_BULK 14
#define BULK_CHAN_CTRL 0
#define BULK_CHAN_DATA 1

#define CMD_DUMP_START 0x01
#define CMD_CLEAR_MEM  0x02
#define CMD_EOF        0xFF

// --- Data Structure (24 bytes total) ---
// We use __attribute__((packed)) so the byte order perfectly matches the React Native parser
typedef struct __attribute__((packed)) {
    uint32_t timestamp;
    uint16_t vbat;
    uint16_t front;
    uint16_t back;
    uint16_t left;
    uint16_t right;
    uint16_t up;
    uint16_t zrange;
    int16_t x;
    int16_t y;
    int16_t z;
} FlightSample;

// --- Memory Allocation ---
// 400 samples at 10Hz = 40 seconds of recording time. 
// 400 * 24 bytes = 9.6 KB (Easily fits in STM32 RAM)
#define MAX_SAMPLES 400
static FlightSample flight_log[MAX_SAMPLES];
static uint16_t sample_count = 0;

// --- State Variables ---
static uint8_t mission_state = 0; // 0=Idle, 1=Fly/Log, 2=Abort, 3=Landed/DataReady
static uint32_t mission_timer = 10; 
static uint32_t mission_height = 500; 

static bool dump_requested = false;
static bool clear_requested = false;

// --- CRTP Callback for Port 14 ---
// This interrupts to catch commands from the app, but sets a flag so we do the heavy lifting in the main loop
void bulkCrtpCb(CRTPPacket *p) {
    if (p->channel == BULK_CHAN_CTRL && p->size >= 1) {
        uint8_t cmd = p->data[0];
        if (cmd == CMD_DUMP_START) dump_requested = true;
        if (cmd == CMD_CLEAR_MEM) clear_requested = true;
    }
}

static uint16_t clampRange(float mm) {
    if (mm <= 0.0f || mm > 3000.0f) return 0;
    return (uint16_t)mm;
}

void appMain(void) {
    vTaskDelay(M2T(3000));
    
    // Register custom CRTP port listener
    crtpRegisterPortCB(CRTP_PORT_BULK, bulkCrtpCb);

    // Fetch internal log IDs
    logVarId_t idVbat  = logGetVarId("pm", "vbatMV");
    logVarId_t idFront = logGetVarId("range", "front");
    logVarId_t idBack  = logGetVarId("range", "back");
    logVarId_t idLeft  = logGetVarId("range", "left");
    logVarId_t idRight = logGetVarId("range", "right");
    logVarId_t idUp    = logGetVarId("range", "up");
    logVarId_t idDown  = logGetVarId("range", "zrange");
    logVarId_t idX     = logGetVarId("stateEstimate", "x");
    logVarId_t idY     = logGetVarId("stateEstimate", "y");
    logVarId_t idZ     = logGetVarId("stateEstimate", "z");

    DEBUG_PRINT("CaveBat V2: Store-and-Forward Ready.\n");
    
    uint32_t flight_start_time = 0;
    bool is_flying = false;

    highLevelCommanderInit();

    while (1) {
        // --- 1. Process CRTP Bulk Requests ---
        if (dump_requested) {
            DEBUG_PRINT("App requested data dump. Sending %d samples...\n", sample_count);
            CRTPPacket p;
            p.port = CRTP_PORT_BULK;
            p.channel = BULK_CHAN_DATA;
            
            for (uint16_t i = 0; i < sample_count; i++) {
                p.size = sizeof(FlightSample);
                memcpy(p.data, &flight_log[i], p.size);
                crtpSendPacket(&p);
                
                // Throttle transmission to 10ms per packet so the BLE queue doesn't overflow
                vTaskDelay(M2T(10)); 
            }
            
            // Send EOF Sentinel packet
            p.size = 1;
            p.data[0] = CMD_EOF;
            crtpSendPacket(&p);
            
            dump_requested = false;
            DEBUG_PRINT("Dump complete.\n");
        }

        if (clear_requested) {
            sample_count = 0;
            mission_state = 0; // Reset to Idle
            clear_requested = false;
            DEBUG_PRINT("Memory cleared. Ready for next flight.\n");
        }

        // --- 2. Flight & Logging State Machine ---
        if (mission_state == 1 && !is_flying) {
            DEBUG_PRINT("Takeoff initiated.\n");
            sample_count = 0; // Ensure clean slate
            flight_start_time = xTaskGetTickCount();
            is_flying = true;
            
            float target_height_m = mission_height / 1000.0f;
            highLevelCommanderTakeoff(target_height_m, 2.0f);
        } 
        else if (mission_state == 1 && is_flying) {
            uint32_t current_time = xTaskGetTickCount();
            
            // Log Data at 10Hz
            if (sample_count < MAX_SAMPLES) {
                flight_log[sample_count].timestamp = current_time;
                flight_log[sample_count].vbat      = (uint16_t)logGetUint(idVbat);
                flight_log[sample_count].front     = clampRange(logGetFloat(idFront));
                flight_log[sample_count].back      = clampRange(logGetFloat(idBack));
                flight_log[sample_count].left      = clampRange(logGetFloat(idLeft));
                flight_log[sample_count].right     = clampRange(logGetFloat(idRight));
                flight_log[sample_count].up        = clampRange(logGetFloat(idUp));
                flight_log[sample_count].zrange    = clampRange(logGetFloat(idDown));
                flight_log[sample_count].x         = (int16_t)(logGetFloat(idX) * 1000.0f);
                flight_log[sample_count].y         = (int16_t)(logGetFloat(idY) * 1000.0f);
                flight_log[sample_count].z         = (int16_t)(logGetFloat(idZ) * 1000.0f);
                sample_count++;
            }

            // Check Timer
            uint32_t elapsed_sec = (current_time - flight_start_time) / configTICK_RATE_HZ;
            if (elapsed_sec >= mission_timer) {
                DEBUG_PRINT("Timer complete. Landing.\n");
                highLevelCommanderLand(0.0f, 2.0f);
                vTaskDelay(M2T(2500));
                
                is_flying = false;
                mission_state = 3; // Landed & Data Ready
            }
        } 
        else if (mission_state == 2) {
            DEBUG_PRINT("Abort requested! Landing.\n");
            highLevelCommanderLand(0.0f, 1.0f);
            vTaskDelay(M2T(1500));
            is_flying = false;
            mission_state = 3; 
        }

        vTaskDelay(M2T(100)); // 10Hz Main Loop
    }
}

// --- App Parameter Registration ---
PARAM_GROUP_START(mission)
  PARAM_ADD(PARAM_UINT8,  state,  &mission_state)
  PARAM_ADD(PARAM_UINT32, timer,  &mission_timer)
  PARAM_ADD(PARAM_UINT32, height, &mission_height)
PARAM_GROUP_STOP(mission)