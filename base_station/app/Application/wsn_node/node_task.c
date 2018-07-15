/******************************************************************************

 @file  node_task.c

 @brief Node task module

 Group: WCS, BTS
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2017-2018, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc13x2_sdk_2_20_00_
 Release Date: 2018-07-02 19:40:58
 *****************************************************************************/


/***** Includes *****/

#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/PIN.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayExt.h>

/* Board Header files */
#include "board.h"

#include <wsn_node/node_radio_task.h>
#include <wsn_node/node_task.h>

#include <ble_remote_display/app/remote_display.h>

#ifdef FEATURE_BLE_ADV
#include "ble_adv/BleAdv.h"
#endif

/***** Defines *****/

#define NODE_TASK_STACK_SIZE 1024
#define NODE_TASK_PRIORITY   1

#define NODE_EVENT_ALL                  0xFFFFFFFF
#define NODE_EVENT_NEW_ADC_VALUE    (uint32_t)(1 << 0)
#define NODE_EVENT_UPDATE_LCD       (uint32_t)(1 << 1)

/* Sensor stub data will change every 1 seconds */
#define NODE_ADCTASK_SENSOR_STUB_DURIATION_MS       1000

/* Sensor stub max data value */
#define NODE_ADCTASK_SENSOR_STUB_MAX_VALUE          0xFF

#define NUM_EDDYSTONE_URLS      5

/***** Variable declarations *****/
static Task_Params nodeTaskParams;
Task_Struct nodeTask;    /* Not static so you can see in ROV */
static uint8_t nodeTaskStack[NODE_TASK_STACK_SIZE];
Event_Struct nodeEvent;  /* Not static so you can see in ROV */
static Event_Handle nodeEventHandle;
static uint16_t latestSensorData;

/* Clock for sensor stub */
Clock_Struct sensorStubClock;     /* Not static so you can see in ROV */
static Clock_Handle sensorStubClockHandle;

#ifdef WSN_USE_DISPLAY
/* Display driver handles */
static Display_Handle hDisplayLcd;
static Display_Handle hDisplaySerial;
#endif //WSN_USE_DISPLAY

#ifndef RF_PROFILING
/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/* Enable the 3.3V power domain used by the LCD */
PIN_Config pinTable[] = {
    NODE_ACTIVITY_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
#endif //!RF_PROFILING

static uint8_t nodeAddress = 0;

/***** Prototypes *****/
static void nodeTaskFunction(UArg arg0, UArg arg1);
static void updateLcd(void);
static void sensorStubCb(UArg arg0);
static void setReportIntervalCb(uint8_t reportInterval);
static void setConcLedCb(uint8_t concLed);
static void setNodeAddressCb(uint8_t newNodeAddress);

RemoteDisplay_nodeCbs_t DmmSimplePeripheral_nodeCbs =
{
    setReportIntervalCb,
    setConcLedCb,
    setNodeAddressCb
};


static void nodeTaskFunction(UArg arg0, UArg arg1)
{
#ifdef WSN_USE_DISPLAY
    /* Initialize display and try to open both UART and LCD types of display. */
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;

    /* Open both an available LCD display and an UART display.
     * Whether the open call is successful depends on what is present in the
     * Display_config[] array of the board file.
     *
     * Note that for SensorTag evaluation boards combined with the SHARP96x96
     * Watch DevPack, there is a pin conflict with UART such that one must be
     * excluded, and UART is preferred by default. To display on the Watch
     * DevPack, add the precompiler define BOARD_DISPLAY_EXCLUDE_UART.
     */
    hDisplayLcd = Display_open(Display_Type_LCD, &params);
    hDisplaySerial = Display_open(Display_Type_UART, &params);

    /* Check if the selected Display type was found and successfully opened */
    if (hDisplaySeria;l)
    {
        Display_printf(hDisplaySerial, 0, 0, "Waiting for reading...");
    }

    /* Check if the selected Display type was found and successfully opened */
    if (hDisplayLcd)
    {
        Display_printf(hDisplayLcd, 0, 0, "Waiting for Data...");
    }
#endif //WSN_USE_DISPLAY

#ifndef RF_PROFILING
    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (!ledPinHandle)
    {
        System_abort("Error initializing board 3.3V domain pins\n");
    }
#endif //!RF_PROFILING

    RemoteDisplay_registerNodeCbs(DmmSimplePeripheral_nodeCbs);

    /* Setup timeout for sensor stub timeout */
    Clock_setTimeout(sensorStubClockHandle,
            NODE_ADCTASK_SENSOR_STUB_DURIATION_MS * 1000 / Clock_tickPeriod);

    /* Start sensor stub clock */
    Clock_start(sensorStubClockHandle);

    while (1)
    {
        /* Wait for event */
        uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If new ADC value, send this data */
        if (events & NODE_EVENT_NEW_ADC_VALUE) {
#ifndef RF_PROFILING
            /* Toggle activity LED */
            PIN_setOutputValue(ledPinHandle, NODE_ACTIVITY_LED,!PIN_getOutputValue(NODE_ACTIVITY_LED));
#endif //!RF_PROFILING

            /* Send ADC value to concentrator and simple peripheral */
            NodeRadioTask_sendSensorData(latestSensorData);
            RemoteDisplay_updateNodeData(latestSensorData);

            /* Update display */
            updateLcd();
        }
        /* If new ADC value, send this data */
        if (events & NODE_EVENT_UPDATE_LCD) {
            /* update display */
            updateLcd();
        }
    }
}

static void updateLcd(void)
{
    /* get node address if not already done */
    if (nodeAddress == 0)
    {
        nodeAddress = nodeRadioTask_getNodeAddr();
        /* update node address in simple peripheral */
        RemoteDisplay_setNodeAddress(nodeAddress);
    }
#ifdef WSN_USE_DISPLAY
    /* print to LCD */
    Display_clear(hDisplayLcd);
    Display_printf(hDisplayLcd, 0, 0, "NodeID: 0x%02x", nodeAddress);
    Display_printf(hDisplayLcd, 1, 0, "ADC: %04d", latestSensorData);

    /* Print to UART clear screen, put cuser to beggining of terminal and print the header */
    Display_printf(hDisplaySerial, 0, 0, "\033[2J \033[0;0HNode ID: 0x%02x", nodeAddress);
    Display_printf(hDisplaySerial, 0, 0, "Node ADC Reading: %04d", latestSensorData);
#endif //WSN_USE_DISPLAY
}

/*
 *  ======== sensorStubCallback ========
 *  sensor Stub function to simulate reading an ADC.
 */
static void sensorStubCb(UArg arg0)
{
    static uint8_t sensorStubData = 0;

    /* data ramp to simulate data reading */
    sensorStubData++;

    /* Save latest value */
    latestSensorData = sensorStubData;

    /* Post event */
    Event_post(nodeEventHandle, NODE_EVENT_NEW_ADC_VALUE);
}

/** @brief  Sets the sensor reporting interval
 *
 *  @param  reportInterval  report interval in seconds
 */
static void setReportIntervalCb(uint8_t reportInterval)
{
    /* Start sensor stub clock */
    Clock_stop(sensorStubClockHandle);

    /* Setup timeout for sensor stub timeout */
    Clock_setPeriod(sensorStubClockHandle,
                     reportInterval * 1000000 / Clock_tickPeriod);

    /* Start sensor stub clock */
    Clock_start(sensorStubClockHandle);
}

/** @brief  Sets the sensor reporting interval
 *
 *  @param  nodeAddress  new node address
 */
static void setNodeAddressCb(uint8_t newNodeAddress)
{
    nodeRadioTask_setNodeAddr(newNodeAddress);
    nodeAddress = newNodeAddress;
}

/** @brief  Sends concetrator LED toggle request
 *
 *  @param  concLed  concentrator LED toggle (1 or 0)
 */
static void setConcLedCb(uint8_t concLed)
{
    /* set the concentrator LED toggle bit for next message */
    nodeRadioTask_setConcLedToggle((bool) concLed);
}

/***** Function definitions *****/

/** @brief  Initializes the Node Task and creates all TI-RTOS objects
 */
void NodeTask_init(void)
{
    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&nodeEvent, &eventParam);
    nodeEventHandle = Event_handle(&nodeEvent);

    /* Clock to report stub sensor data periodically */
    Clock_Params clkParams;
    Clock_Params_init(&clkParams);

    clkParams.period = ((NODE_ADCTASK_SENSOR_STUB_DURIATION_MS * 1000) / Clock_tickPeriod);
    clkParams.startFlag = FALSE;
    Clock_construct(&sensorStubClock, sensorStubCb, 1, &clkParams);
    sensorStubClockHandle = Clock_handle(&sensorStubClock);

    /* Create the node task */
    Task_Params_init(&nodeTaskParams);
    nodeTaskParams.stackSize = NODE_TASK_STACK_SIZE;
    nodeTaskParams.priority = NODE_TASK_PRIORITY;
    nodeTaskParams.stack = &nodeTaskStack;
    Task_construct(&nodeTask, nodeTaskFunction, &nodeTaskParams, NULL);
}


