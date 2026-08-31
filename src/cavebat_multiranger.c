// cavebat_multiranger.c - Bitcraze's Multi-ranger driver with the init made
// robust. Built INSTEAD of the stock one (CONFIG_DECK_MULTIRANGER=n).
//
// WHY THIS EXISTS
//   The stock driver failed to initialise its five sensors on maybe a third of
//   boots. That is not a cosmetic failure: deckTest() returning false makes
//   system.c skip systemStart() entirely, so the whole firmware never runs --
//   no flight, no telemetry, every command silently ignored. It blocked work
//   repeatedly and a power cycle sometimes cleared it and sometimes did not.
//
// WHAT CHANGED, and why each is a plausible cause of an INTERMITTENT failure
//   1. The expander writes are checked. pca95x4ConfigOutput/ClearOutput return
//      a status and the stock driver discards it. That chip gates all five
//      sensors' reset lines, so if a write is lost every sensor stays in reset
//      and all five fail -- with nothing saying the expander was at fault.
//   2. The sensor boot delay is 10ms rather than 2ms. A VL53L1x needs time
//      after its reset line is released before it answers on I2C, and 2ms sits
//      close enough to the edge to explain a failure that comes and goes.
//   3. Each sensor gets three attempts. A transient I2C failure should not
//      end the boot of the entire aircraft.
//
// Everything else is unchanged from upstream.

/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright 2021, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* multiranger.c: Multiranger deck driver */
#include "deck.h"
#include "param.h"

#define DEBUG_MODULE "MR"

#include "system.h"
#include "debug.h"
#include "log.h"
#include "pca95x4.h"
#include "vl53l1x.h"
#include "range.h"
#include "static_mem.h"

#include "i2cdev.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

static bool isInit = false;
static bool isTested = false;
static bool isPassed = false;
static uint16_t filterMask = 1 << VL53L1_RANGESTATUS_RANGE_VALID;

#define MR_PIN_UP PCA95X4_P0
#define MR_PIN_FRONT PCA95X4_P4
#define MR_PIN_BACK PCA95X4_P1
#define MR_PIN_LEFT PCA95X4_P6
#define MR_PIN_RIGHT PCA95X4_P2

NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devFront;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devBack;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devUp;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devLeft;
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L1_Dev_t devRight;

static bool mrInitSensor(VL53L1_Dev_t *pdev, uint32_t pca95pin, char *name)
{
    // Three attempts. A transient I2C failure should not end the boot of the
    // entire aircraft, which is what one attempt meant in practice.
    for (int attempt = 1; attempt <= 3; attempt++)
    {
        // Release this sensor's reset line, via the expander. Checked, unlike
        // upstream: if this write is lost the sensor never leaves reset and no
        // amount of waiting will make it answer.
        if (!pca95x4SetOutput(PCA95X4_DEFAULT_ADDRESS, pca95pin))
        {
            DEBUG_PRINT("Init %s: expander write failed (try %d)\n", name, attempt);
            vTaskDelay(M2T(20));
            continue;
        }

        // 10ms, not 2ms. The VL53L1x needs time after reset is released before
        // it answers on I2C, and 2ms is close enough to the edge to explain a
        // failure that appears on some boots and not others.
        vTaskDelay(M2T(10));

        if (vl53l1xInit(pdev, I2C1_DEV))
        {
            if (attempt > 1)
            {
                DEBUG_PRINT("Init %s sensor [OK] (took %d tries)\n", name, attempt);
            }
            else
            {
                DEBUG_PRINT("Init %s sensor [OK]\n", name);
            }
            return true;
        }

        DEBUG_PRINT("Init %s sensor retry %d\n", name, attempt);
        vTaskDelay(M2T(20));
    }

    DEBUG_PRINT("Init %s sensor [FAIL] after 3 tries\n", name);
    return false;
}

static uint16_t mrGetMeasurementAndRestart(VL53L1_Dev_t *dev)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range;

    while (dataReady == 0)
    {
        status = VL53L1_GetMeasurementDataReady(dev, &dataReady);
        vTaskDelay(M2T(1));
    }

    status = VL53L1_GetRangingMeasurementData(dev, &rangingData);

    if (filterMask & (1 << rangingData.RangeStatus))
    {
        range = rangingData.RangeMilliMeter;
    }
    else
    {
        range = 32767;
    }

    VL53L1_StopMeasurement(dev);
    status = VL53L1_StartMeasurement(dev);
    status = status;

    return range;
}

static void mrTask(void *param)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;

    systemWaitStart();

    // Restart all sensors
    status = VL53L1_StopMeasurement(&devFront);
    status = VL53L1_StartMeasurement(&devFront);
    status = VL53L1_StopMeasurement(&devBack);
    status = VL53L1_StartMeasurement(&devBack);
    status = VL53L1_StopMeasurement(&devUp);
    status = VL53L1_StartMeasurement(&devUp);
    status = VL53L1_StopMeasurement(&devLeft);
    status = VL53L1_StartMeasurement(&devLeft);
    status = VL53L1_StopMeasurement(&devRight);
    status = VL53L1_StartMeasurement(&devRight);
    status = status;

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, M2T(100));
        rangeSet(rangeFront, mrGetMeasurementAndRestart(&devFront) / 1000.0f);
        rangeSet(rangeBack, mrGetMeasurementAndRestart(&devBack) / 1000.0f);
        rangeSet(rangeUp, mrGetMeasurementAndRestart(&devUp) / 1000.0f);
        rangeSet(rangeLeft, mrGetMeasurementAndRestart(&devLeft) / 1000.0f);
        rangeSet(rangeRight, mrGetMeasurementAndRestart(&devRight) / 1000.0f);
    }
}

static void mrInit()
{
    if (isInit)
    {
        return;
    }

    pca95x4Init();

    // The expander gates every sensor's reset line, so if it is not answering
    // nothing below can possibly work. Say so plainly rather than letting five
    // sensors fail one after another with no indication of the real cause.
    if (!pca95x4Test(PCA95X4_DEFAULT_ADDRESS))
    {
        DEBUG_PRINT("MR: expander 0x20 not answering - retrying\n");
        for (int i = 0; i < 5 && !pca95x4Test(PCA95X4_DEFAULT_ADDRESS); i++)
        {
            vTaskDelay(M2T(50));
        }
        if (!pca95x4Test(PCA95X4_DEFAULT_ADDRESS))
        {
            DEBUG_PRINT("MR: expander 0x20 STILL not answering\n");
        }
        else
        {
            DEBUG_PRINT("MR: expander 0x20 answered on retry\n");
        }
    }

    pca95x4ConfigOutput(PCA95X4_DEFAULT_ADDRESS,
                        ~(MR_PIN_UP |
                          MR_PIN_RIGHT |
                          MR_PIN_LEFT |
                          MR_PIN_FRONT |
                          MR_PIN_BACK));

    pca95x4ClearOutput(PCA95X4_DEFAULT_ADDRESS,
                       MR_PIN_UP |
                       MR_PIN_RIGHT |
                       MR_PIN_LEFT |
                       MR_PIN_FRONT |
                       MR_PIN_BACK);

    isInit = true;

    xTaskCreate(mrTask, MULTIRANGER_TASK_NAME, MULTIRANGER_TASK_STACKSIZE, NULL,
                MULTIRANGER_TASK_PRI, NULL);
}

static bool mrTest()
{
    if (isTested)
    {
        return isPassed;
    }

    isPassed = isInit;

    isPassed &= mrInitSensor(&devFront, MR_PIN_FRONT, "front");
    isPassed &= mrInitSensor(&devBack, MR_PIN_BACK, "back");
    isPassed &= mrInitSensor(&devUp, MR_PIN_UP, "up");
    isPassed &= mrInitSensor(&devLeft, MR_PIN_LEFT, "left");
    isPassed &= mrInitSensor(&devRight, MR_PIN_RIGHT, "right");

    isTested = true;

    return isPassed;
}

static const DeckDriver multiranger_deck = {
    .vid = 0xBC,
    .pid = 0x0C,
    .name = "bcMultiranger",

    .usedGpio = 0,
    .usedPeriph = DECK_USING_I2C,

    .init = mrInit,
    .test = mrTest,
};

DECK_DRIVER(multiranger_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Multi-ranger deck](%https://store.bitcraze.io/collections/decks/products/multi-ranger-deck) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &isInit)

PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(multiranger)
/**
 * @brief Filter mask determining which range measurements is to be let through based on the range status of the VL53L1 chip
 */
PARAM_ADD(PARAM_UINT16, filterMask, &filterMask)

PARAM_GROUP_STOP(multiranger)
