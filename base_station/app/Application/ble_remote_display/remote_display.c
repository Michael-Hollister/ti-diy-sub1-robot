/******************************************************************************

 @file  remote_display.c

 @brief This file contains the Remote Display sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2013-2018, Texas Instruments Incorporated
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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/display/Display.h>

#if !(defined __TI_COMPILER_VERSION__)
#include <intrinsics.h>
#endif

#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include <remote_display_gatt_profile.h>

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

#include <board.h>
#include <board_key.h>

#include "remote_display.h"

#include <dmm/dmm_policy_blesp_wsnnode.h>

#ifdef RF_PROFILING
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include "Board.h"
#endif

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * CONSTANTS
 */

// Address mode of the local device
#define DEFAULT_ADDRESS_MODE                  ADDRMODE_PUBLIC

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Minimum connection interval (units of 1.25ms, 80=100ms) for parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 88=110ms) for  parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     88

// Slave latency to use for parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 300=3s) for parameter update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          300

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_PASS_TO_APP

// How often to perform periodic event (in ms)
#define RD_PERIODIC_EVT_PERIOD               5000

// How often to read current current RPA (in ms)
#define RD_READ_RPA_EVT_PERIOD               3000

// Delay (in ms) after connection establishment before sending a parameter update request
#define RD_SEND_PARAM_UPDATE_DELAY           1000

// Number of times to attempt to configure params, some apps will reject the
// update request if they are still discovering the attributes
#define MAX_PARAM_UPDATE_ATTEMPTS            3

// Task configuration
#define RD_TASK_PRIORITY                     1

#ifndef RD_TASK_STACK_SIZE
#define RD_TASK_STACK_SIZE                   1024
#endif

// Application events
#define RD_STATE_CHANGE_EVT                  0
#define RD_CHAR_CHANGE_EVT                   1
#define RD_KEY_CHANGE_EVT                    2
#define RD_ADV_EVT                           3
#define RD_PAIR_STATE_EVT                    4
#define RD_PASSCODE_EVT                      5
#define RD_READ_RPA_EVT                      6
#define RD_SEND_PARAM_UPDATE_EVT             7
#define RD_UPDATE_NODE_ADDR_EVT              8
#define RD_UPDATE_CONC_LED_EVT               9
#define RD_UPDATE_NODE_DATA_EVT              10
#define RD_UPDATE_NODE_STATS_EVT             11

// Internal Events for RTOS application
#define RD_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define RD_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define RD_ALL_EVENTS                        (RD_ICALL_EVT             | \
                                              RD_QUEUE_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define RD_ADDR_STR_SIZE     15

// Row numbers for display
#define RD_ROW_TITLE         0
#define RD_ROW_SEPARATOR_1   1
#define RD_ROW_STATUS_1      2
#define RD_ROW_STATUS_2      3
#define RD_ROW_CONNECTION    4
#define RD_ROW_ADVSTATE      5
#define RD_ROW_IDA           6
#define RD_ROW_RPA           7
#define RD_ROW_WSN_DATA      8
#define RD_ROW_WSN_STATS_1   9
#define RD_ROW_WSN_STATS_2   10
#define RD_ROW_WSN_STATS_3   11
#define RD_ROW_DEBUG         12

// For storing the active connections
#define RD_RSSI_TRACK_CHNLS        1            // Max possible channels can be GAP_BONDINGS_MAX
#define RD_MAX_RSSI_STORE_DEPTH    5
#define RD_INVALID_HANDLE          0xFFFF
#define RSSI_2M_THRSHLD           -30           // -80 dB rssi
#define RSSI_1M_THRSHLD           -40           // -90 dB rssi
#define RSSI_S2_THRSHLD           -50           // -100 dB rssi
#define RSSI_S8_THRSHLD           -60           // -120 dB rssi
#define RD_PHY_NONE                LL_PHY_NONE  // No PHY set
#define AUTO_PHY_UPDATE            0xFF

// Spin if the expression is not true
#define REMOTEDISPLAY_ASSERT(expr) if (!(expr)) HAL_ASSERT_SPINLOCK;
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} rdEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} rdPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} rdPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} rdGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;                //
  uint8_t data[];
} rdClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
  List_Elem elem;
  uint16_t connHandle;
} rdConnHandleEntry_t;

// Connected device information
typedef struct
{
  uint16_t         connHandle;                        // Connection Handle
  Clock_Struct*    pUpdateClock;                      // pointer to clock struct
} rdConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

// Task configuration
Task_Struct rdTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(rdTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t rdTaskStack[RD_TASK_STACK_SIZE];

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkPeriodic;
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass RPA read event ID to clock handler
rdClockEventData_t argRpaRead =
{ .event = RD_READ_RPA_EVT };

// Per-handle connection info
static rdConnRec_t connList[MAX_NUM_BLE_CONNS];

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Node Remote Display";

// Advertisement data
static uint8_t advertData[] =
{
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(RDPROFILE_SERV_UUID),
  HI_UINT16(RDPROFILE_SERV_UUID)
};

// Scan Response Data
static uint8_t scanRspData[] =
{
 // complete name
 15,   // length of this data
 GAP_ADTYPE_LOCAL_NAME_COMPLETE,
 'D',
 'M',
 'M',
 ' ',
 'W',
 's',
 'n',
 'N',
 'o',
 'd',
 'e',
 ' ',
 'R',
 'D',
  // connection interval range
  5,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  2,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// Advertising handles
static uint8 advHandleLegacy;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

#ifdef RF_PROFILING
/* Pin driver handle */
static PIN_Handle rfBleProfilingPinHandle;
static PIN_State rfBleProfilingPinState;

#define BLE_CONNECTED_GPIO  Board_DIO22

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config rfBleProfilingPinTable[] =
{
    BLE_CONNECTED_GPIO | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
#endif

static RemoteDisplay_nodeCbs_t nodeCallbacks;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void RemoteDisplay_init( void );
static void RemoteDisplay_taskFxn(UArg a0, UArg a1);
static uint8_t RemoteDisplay_processStackMsg(ICall_Hdr *pMsg);
static uint8_t RemoteDisplay_processGATTMsg(gattMsgEvent_t *pMsg);
static void RemoteDisplay_processGapMessage(gapEventHdr_t *pMsg);
static void RemoteDisplay_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void  RemoteDisplay_processAdvEvent(rdGapAdvEventData_t *pEventData);
static void RemoteDisplay_processAppMsg(rdEvt_t *pMsg);
static void RemoteDisplay_processCharValueChangeEvt(uint8_t paramId);
static void RemoteDisplay_updateRPA(void);
static void RemoteDisplay_clockHandler(UArg arg);
static void RemoteDisplay_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);
static void RemoteDisplay_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void RemoteDisplay_processPairState(rdPairStateData_t *pPairState);
static void RemoteDisplay_processPasscode(rdPasscodeData_t *pPasscodeData);
static void RemoteDisplay_charValueChangeCB(uint8_t paramId);
static status_t RemoteDisplay_enqueueMsg(uint8_t event, void *pData);
static void RemoteDisplay_keyChangeHandler(uint8 keys);
static void RemoteDisplay_handleKeys(uint8_t keys);
static uint8_t RemoteDisplay_addConn(uint16_t connHandle);
static uint8_t RemoteDisplay_getConnIndex(uint16_t connHandle);
static uint8_t RemoteDisplay_removeConn(uint16_t connHandle);
static void RemoteDisplay_processParamUpdate(uint16_t connHandle);
static uint8_t RemoteDisplay_clearConnListEntry(uint16_t connHandle);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t RemoteDisplay_BondMgrCBs =
{
  RemoteDisplay_passcodeCb,       // Passcode callback
  RemoteDisplay_pairStateCb       // Pairing/Bonding state Callback
};

// Remote Display GATT Profile Callbacks
static remoteDisplayProfileCBs_t RemoteDisplay_ProfileCBs =
{
  RemoteDisplay_charValueChangeCB // Remote Display GATT Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      RemoteDisplay_registerNodeCbs
 *
 * @brief   Register the wsn node callbacks
 */
void RemoteDisplay_registerNodeCbs(RemoteDisplay_nodeCbs_t nodeCbs)
{
    nodeCallbacks = nodeCbs;
}

/*********************************************************************
 * @fn      RemoteDisplay_createTask
 *
 * @brief   Task creation function for the Remote Display.
 */
void RemoteDisplay_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = rdTaskStack;
  taskParams.stackSize = RD_TASK_STACK_SIZE;
  taskParams.priority = RD_TASK_PRIORITY;

  Task_construct(&rdTask, RemoteDisplay_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      RemoteDisplay_setNodeAddress
 *
 * @brief   Sets the nodes address characteristic
 */
void RemoteDisplay_setNodeAddress(uint8_t nodeAddress)
{
    uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

    if (pValue)
    {
      *pValue = nodeAddress;
      RemoteDisplay_enqueueMsg(RD_UPDATE_NODE_ADDR_EVT, pValue);
    }
}

/*********************************************************************
 * @fn      RemoteDisplay_setNodeAddress
 *
 * @brief   Sets the nodes address characteristic
 */
void RemoteDisplay_setConcLed(uint8_t concLed)
{
    uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

    if (pValue)
    {
      *pValue = concLed;
      RemoteDisplay_enqueueMsg(RD_UPDATE_CONC_LED_EVT, pValue);
    }
}

/*********************************************************************
 * @fn      RemoteDisplay_updateNodeData
 *
 * @brief   Sets the nodes data reading characteristic
 */
void RemoteDisplay_updateNodeData(uint8_t sensorData)
{
    uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

    if (pValue)
    {
      *pValue = sensorData;
      RemoteDisplay_enqueueMsg(RD_UPDATE_NODE_DATA_EVT, pValue);
    }
}

/*********************************************************************
 * @fn      RemoteDisplay_updateNodeData
 *
 * @brief   Sets the nodes data reading characteristic
 */
void RemoteDisplay_updateNodeWsnStats(RemoteDisplay_nodeWsnStats_t stats)
{
#ifdef RD_DISPLAY_WSN_STATS
  RemoteDisplay_nodeWsnStats_t *pValue = ICall_malloc(sizeof(RemoteDisplay_nodeWsnStats_t));

  if (pValue)
  {
    *pValue = stats;
    RemoteDisplay_enqueueMsg(RD_UPDATE_NODE_STATS_EVT, pValue);
  }
#endif //RD_DISPLAY_WSN_STATS
}

/*********************************************************************
 * @fn      RemoteDisplay_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void RemoteDisplay_init(void)
{
#ifdef RF_PROFILING
    /* Open LED pins */
    rfBleProfilingPinHandle = PIN_open(&rfBleProfilingPinState, rfBleProfilingPinTable);
    /* Clear Debug pins */
    PIN_setOutputValue(rfBleProfilingPinHandle, BLE_CONNECTED_GPIO, 0);
#endif

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {
    // Don't send a pairing request after connecting; the peer device must
    // initiate pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    // Use authenticated pairing: require passcode.
    uint8_t mitm = TRUE;
    // This device only has display capabilities. Therefore, it will display the
    // passcode during pairing. However, since the default passcode is being
    // used, there is no need to display anything.
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    // Request bonding (storing long-term keys for re-encryption upon subsequent
    // connections without repairing)
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service
  RemoteDisplay_AddService(GATT_ALL_SERVICES); // Remote Display GATT Profile

  // Setup the Remote Display Characteristic Values
  // For more information, see the GATT and GATTServApp sections in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {
    uint8_t nodeReportIntervalDefaultValue = 1;
    uint8_t concLedDefaultValue = 0;
    uint8_t nodeDataDefaultValue = 1;
    uint8_t nodeAdressDefaultValue[RDPROFILE_NODE_ADDR_CHAR_LEN] = { 0 };

    RemoteDisplay_SetParameter(RDPROFILE_NODE_REPORT_INTERVAL_CHAR, sizeof(uint8_t),
                               &nodeReportIntervalDefaultValue);
    RemoteDisplay_SetParameter(RDPROFILE_CONC_LED_CHAR, sizeof(uint8_t),
                               &concLedDefaultValue);
    RemoteDisplay_SetParameter(RDPROFILE_NODE_DATA_CHAR, sizeof(uint8_t),
                               &nodeDataDefaultValue);
    RemoteDisplay_SetParameter(RDPROFILE_NODE_ADDR_CHAR, RDPROFILE_NODE_ADDR_CHAR_LEN,
                               nodeAdressDefaultValue);
  }

  // Register callback with Remote Display GATT profile
  RemoteDisplay_RegisterAppCBs(&RemoteDisplay_ProfileCBs);

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&RemoteDisplay_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  GATT_InitClient();

  // Init key debouncer
  Board_initKeys(RemoteDisplay_keyChangeHandler);

  // Initialize Connection List
  RemoteDisplay_clearConnListEntry(LL_CONNHANDLE_ALL);

  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, NULL);

  // The type of display is configured based on the BOARD_DISPLAY_USE...
  // preprocessor definitions
  dispHandle = Display_open(Display_Type_ANY, NULL);

  // update display
  Display_printf(dispHandle, RD_ROW_TITLE, 0, "Wsn Node Remote Display");
  Display_printf(dispHandle, RD_ROW_SEPARATOR_1, 0, "====================");
}

/*********************************************************************
 * @fn      RemoteDisplay_taskFxn
 *
 * @brief   Application task entry point for the Remote Display.
 *
 * @param   a0, a1 - not used.
 */
static void RemoteDisplay_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  RemoteDisplay_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, RD_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for non-BLE stack events
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = RemoteDisplay_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & RD_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          rdEvt_t *pMsg = (rdEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            RemoteDisplay_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t RemoteDisplay_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      RemoteDisplay_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = RemoteDisplay_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

        default:
          break;
      }

      break;
    }

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      RemoteDisplay_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t RemoteDisplay_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_printf(dispHandle, RD_ROW_STATUS_1, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_printf(dispHandle, RD_ROW_STATUS_1, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      RemoteDisplay_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void RemoteDisplay_processAppMsg(rdEvt_t *pMsg)
{
  bool dealloc = TRUE;

  switch (pMsg->event)
  {
    case RD_CHAR_CHANGE_EVT:
      RemoteDisplay_processCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;

    case RD_KEY_CHANGE_EVT:
      RemoteDisplay_handleKeys(*(uint8_t*)(pMsg->pData));
      break;

    case RD_ADV_EVT:
      RemoteDisplay_processAdvEvent((rdGapAdvEventData_t*)(pMsg->pData));
      break;

    case RD_PAIR_STATE_EVT:
      RemoteDisplay_processPairState((rdPairStateData_t*)(pMsg->pData));
      break;

    case RD_PASSCODE_EVT:
      RemoteDisplay_processPasscode((rdPasscodeData_t*)(pMsg->pData));
      break;

    case RD_READ_RPA_EVT:
      RemoteDisplay_updateRPA();
      break;

    case RD_SEND_PARAM_UPDATE_EVT:
    {
      // Extract connection handle from data
      uint16_t connHandle = *(uint16_t *)(((rdClockEventData_t *)pMsg->pData)->data);

      RemoteDisplay_processParamUpdate(connHandle);

      // This data is not dynamically allocated
      dealloc = FALSE;
      break;
    }

    case RD_UPDATE_NODE_ADDR_EVT:
    {
      RemoteDisplay_SetParameter( RDPROFILE_NODE_ADDR_CHAR, RDPROFILE_NODE_ADDR_CHAR_LEN, (uint8_t*)(pMsg->pData) );
      break;
    }

    case RD_UPDATE_CONC_LED_EVT:
    {
      RemoteDisplay_SetParameter( RDPROFILE_CONC_LED_CHAR, 1, (uint8_t*)(pMsg->pData) );
      break;
    }

    case RD_UPDATE_NODE_DATA_EVT:
    {

      Display_printf(dispHandle, RD_ROW_WSN_DATA, 0, "WSN Node Data: %02x", *((uint8_t*)(pMsg->pData)));

      // Note that if notifications of the fourth characteristic have been
      // enabled by a GATT client device, then a notification will be sent
      // every time this function is called.
      RemoteDisplay_SetParameter(RDPROFILE_NODE_DATA_CHAR, sizeof(uint8_t),
                                   (uint8_t*)(pMsg->pData));

      break;
    }
#ifdef RD_DISPLAY_WSN_STATS
    case RD_UPDATE_NODE_STATS_EVT:
    {
      Display_printf(dispHandle, RD_ROW_WSN_STATS_1, 0, "WSN Node Statistics:");
      Display_printf(dispHandle, RD_ROW_WSN_STATS_2, 0, "dataSendSuccess:%d, dataSendFail:%d dataTxSchError:%d, ",
                     ((RemoteDisplay_nodeWsnStats_t*)(pMsg->pData))->dataSendSuccess,
                     ((RemoteDisplay_nodeWsnStats_t*)(pMsg->pData))->dataSendFail,
                     ((RemoteDisplay_nodeWsnStats_t*)(pMsg->pData))->dataTxSchError);
      Display_printf(dispHandle, RD_ROW_WSN_STATS_3, 0, "ackRxTimeout:%d, ackRxSchError:%d, ackRxAbort:%d ",
                     ((RemoteDisplay_nodeWsnStats_t*)(pMsg->pData))->ackRxTimeout,
                     ((RemoteDisplay_nodeWsnStats_t*)(pMsg->pData))->ackRxSchError,
                     ((RemoteDisplay_nodeWsnStats_t*)(pMsg->pData))->ackRxAbort);

      break;
    }
#endif //RD_DISPLAY_WSN_STATS

    default:
      // Do nothing.
      break;
  }

  // Free message data if it exists and we are to dealloc
  if ((dealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void RemoteDisplay_processGapMessage(gapEventHdr_t *pMsg)
{
  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      bStatus_t status = FAILURE;

      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      if(pPkt->hdr.status == SUCCESS)
      {
        // Store the system ID
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = pPkt->devAddr[0];
        systemId[1] = pPkt->devAddr[1];
        systemId[2] = pPkt->devAddr[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = pPkt->devAddr[5];
        systemId[6] = pPkt->devAddr[4];
        systemId[5] = pPkt->devAddr[3];

        // Set Device Info Service Parameter
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        Display_printf(dispHandle, RD_ROW_STATUS_1, 0, "Initialized");

        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        // Temporary memory for advertising parameters for set #1. These will be copied
        // by the GapAdv module
        GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;
        advParamLegacy.primIntMin = DEFAULT_ADVERTISING_INTERVAL;
        advParamLegacy.primIntMax = DEFAULT_ADVERTISING_INTERVAL;

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&RemoteDisplay_advCallback, &advParamLegacy,
                               &advHandleLegacy);
        REMOTEDISPLAY_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advertData), advertData);
        REMOTEDISPLAY_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanRspData), scanRspData);
        REMOTEDISPLAY_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        REMOTEDISPLAY_ASSERT(status == SUCCESS);

        // Display device address
        Display_printf(dispHandle, RD_ROW_IDA, 0, "%s Addr: %s",
                       (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
                       Util_convertBdAddr2Str(pPkt->devAddr));

        if (addrMode > ADDRMODE_RANDOM)
        {
          RemoteDisplay_updateRPA();

          // Create one-shot clock for RPA check event.
          Util_constructClock(&clkRpaRead, RemoteDisplay_clockHandler,
                              RD_READ_RPA_EVT_PERIOD, 0, true,
                              (UArg) &argRpaRead);
        }
      }
      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, RD_ROW_STATUS_2, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      if (pPkt->hdr.status == SUCCESS)
      {
        // Add connection to list and start RSSI
        RemoteDisplay_addConn(pPkt->connectionHandle);

        // Display the address of this connection
        Display_printf(dispHandle, RD_ROW_STATUS_1, 0, "Connected to %s",
                       Util_convertBdAddr2Str(pPkt->devAddr));

        // Start Periodic Clock.
        Util_startClock(&clkPeriodic);
      }

      if (numActive < MAX_NUM_BLE_CONNS)
      {
        // Start advertising since there is room for more connections
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }
      else
      {
        // Stop advertising since there is no room for more connections
        GapAdv_disable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, RD_ROW_STATUS_1, 0, "Device Disconnected!");
      Display_printf(dispHandle, RD_ROW_STATUS_2, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      // Remove the connection from the list and disable RSSI if needed
      RemoteDisplay_removeConn(pPkt->connectionHandle);

      // If no active connections
      if (numActive == 0)
      {
        // Stop periodic clock
        Util_stopClock(&clkPeriodic);
      }

      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

      // Clear remaining lines
      Display_clearLine(dispHandle, RD_ROW_CONNECTION);

      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;

      gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;

      rsp.connectionHandle = pReq->req.connectionHandle;
      rsp.signalIdentifier = pReq->req.signalIdentifier;

      // Only accept connection intervals with slave latency of 0
      // This is just an example of how the application can send a response
      if(pReq->req.connLatency == 0)
      {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

      break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;
      static uint8_t paramUpdateFailCnt = 0;

      // Get the address from the connection handle
      linkDBInfo_t linkInfo;
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      if(pPkt->status == SUCCESS)
      {
        paramUpdateFailCnt = 0;
        // Display the address of the connection update
        Display_printf(dispHandle, RD_ROW_STATUS_2, 0, "Link Param Updated: %s",
                       Util_convertBdAddr2Str(linkInfo.addr));
      }
      else
      {
        paramUpdateFailCnt++;

        // Display the address of the connection update failure
        Display_printf(dispHandle, RD_ROW_STATUS_2, 0,
                       "Link Param Update Failed 0x%x: %s", pPkt->opcode,
                       Util_convertBdAddr2Str(linkInfo.addr));

        if(paramUpdateFailCnt < MAX_PARAM_UPDATE_ATTEMPTS)
        {
          uint8_t connIndex = RemoteDisplay_getConnIndex(pPkt->connectionHandle);
          rdClockEventData_t *paramUpdateEventData;

          // Allocate data to send through clock handler
          paramUpdateEventData = ICall_malloc(sizeof(rdClockEventData_t) +
                                              sizeof (uint16_t));
          if(paramUpdateEventData)
          {
            paramUpdateEventData->event = RD_SEND_PARAM_UPDATE_EVT;
            *((uint16_t *)paramUpdateEventData->data) = pPkt->connectionHandle;

            // Create a clock object and start
            connList[connIndex].pUpdateClock
              = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

            if (connList[connIndex].pUpdateClock)
            {
              Util_constructClock(connList[connIndex].pUpdateClock,
                                RemoteDisplay_clockHandler,
                                (RD_SEND_PARAM_UPDATE_DELAY*paramUpdateFailCnt),
                                 0, true, (UArg) paramUpdateEventData);
            }
          }
        }
      }

      // Check if there are any queued parameter updates
      rdConnHandleEntry_t *connHandleEntry = (rdConnHandleEntry_t *)List_get(&paramUpdateList);
      if (connHandleEntry != NULL)
      {
          // Attempt to send queued update now
          RemoteDisplay_processParamUpdate(connHandleEntry->connHandle);

          // Free list element
          ICall_free(connHandleEntry);
      }

      break;
    }

    default:
      Display_clearLines(dispHandle, RD_ROW_STATUS_1, RD_ROW_STATUS_2);
      break;
  }
}

/*********************************************************************
 *  @fn      RemoteDisplay_bleFastStateUpdateCb
 *
 * @brief   Callback from BLE link layer to indicate a state change
 */
void RemoteDisplay_bleFastStateUpdateCb(uint32_t stackType, uint32_t stackState)
{
  if(stackType == DMMPolicy_StackType_BlePeripheral)
  {
    static uint32_t prevStackState = 0;

    if( !(prevStackState & LL_TASK_ID_SLAVE) && (stackState & LL_TASK_ID_SLAVE))
    {
      //We just connected
#ifdef RF_PROFILING
        PIN_setOutputValue(rfBleProfilingPinHandle, BLE_CONNECTED_GPIO, 1);
#endif

        /* update DMM policy */
        DMMPolicy_updateStackState(DMMPolicy_StackType_BlePeripheral, DMMPOLICY_STACKSTATE_BLEPERIPH_CONNECTED);
    }
    else if( (prevStackState & LL_TASK_ID_SLAVE) && !(stackState & LL_TASK_ID_SLAVE))
    {
      //We just disconnected
#ifdef RF_PROFILING
        PIN_setOutputValue(rfBleProfilingPinHandle, BLE_CONNECTED_GPIO, 0);
#endif

        /* update DMM policy */
        DMMPolicy_updateStackState(DMMPolicy_StackType_BlePeripheral, DMMPOLICY_STACKSTATE_BLEPERIPH_ADV);
    }

    prevStackState = stackState;
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_charValueChangeCB
 *
 * @brief   Callback from REmote Display Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void RemoteDisplay_charValueChangeCB(uint8_t paramId)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = paramId;

    if (RemoteDisplay_enqueueMsg(RD_CHAR_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_freeMsg(pValue);
    }
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_processCharValueChangeEvt
 *
 * @brief   Process a pending Remote Display Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void RemoteDisplay_processCharValueChangeEvt(uint8_t paramId)
{
  uint8_t newReportIntreval;
  uint8_t newConLed;
  uint8_t newNodeAddr[RDPROFILE_NODE_ADDR_CHAR_LEN];

  switch(paramId)
  {
    case RDPROFILE_NODE_REPORT_INTERVAL_CHAR:
      RemoteDisplay_GetParameter(RDPROFILE_NODE_REPORT_INTERVAL_CHAR, &newReportIntreval);

      Display_printf(dispHandle, RD_ROW_STATUS_1, 0, "Report Interval Char: %d", (uint16_t)newReportIntreval);

      if(nodeCallbacks.setReportIntervalCb != NULL)
      {
        nodeCallbacks.setReportIntervalCb(newReportIntreval);
      }

      break;

    case RDPROFILE_CONC_LED_CHAR:
      RemoteDisplay_GetParameter(RDPROFILE_CONC_LED_CHAR, &newConLed);

      Display_printf(dispHandle, RD_ROW_STATUS_1, 0, "Conc LED Toggle Char: %d", (uint16_t)newConLed);

      if(nodeCallbacks.setConcLedCb != NULL)
      {
        nodeCallbacks.setConcLedCb(newConLed);
      }

      break;

    case RDPROFILE_NODE_ADDR_CHAR:
      RemoteDisplay_GetParameter(RDPROFILE_NODE_ADDR_CHAR, newNodeAddr);

      if(nodeCallbacks.setNodeAddressCb != NULL)
      {
        nodeCallbacks.setNodeAddressCb(newNodeAddr[0]);
      }

      break;

    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void RemoteDisplay_updateRPA(void)
{
  uint8_t* pRpaNew;

  // Read the current RPA.
  pRpaNew = GAP_GetDevAddress(FALSE);

  if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
  {
    // If the RPA has changed, update the display
    Display_printf(dispHandle, RD_ROW_RPA, 0, "RP Addr: %s",
                   Util_convertBdAddr2Str(pRpaNew));
    memcpy(rpa, pRpaNew, B_ADDR_LEN);
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void RemoteDisplay_clockHandler(UArg arg)
{
  rdClockEventData_t *pData = (rdClockEventData_t *)arg;

  if (pData->event == RD_READ_RPA_EVT)
  {
    // Start the next period
    Util_startClock(&clkRpaRead);

    // Post event to read the current RPA
    RemoteDisplay_enqueueMsg(RD_READ_RPA_EVT, NULL);
  }
  else if (pData->event == RD_SEND_PARAM_UPDATE_EVT)
  {
    // Send message to app
    RemoteDisplay_enqueueMsg(RD_SEND_PARAM_UPDATE_EVT, pData);
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys - bitmap of pressed keys
 *
 * @return  none
 */
static void RemoteDisplay_keyChangeHandler(uint8_t keys)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = keys;

    if(RemoteDisplay_enqueueMsg(RD_KEY_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_freeMsg(pValue);
    }
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 */
static void RemoteDisplay_handleKeys(uint8_t keys)
{
  /* add button precessing here
   */
}

/*********************************************************************
 * @fn      RemoteDisplay_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void RemoteDisplay_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  rdGapAdvEventData_t *pData = ICall_malloc(sizeof(rdGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(RemoteDisplay_enqueueMsg(RD_ADV_EVT, pData) != SUCCESS)
    {
      ICall_freeMsg(pData);
    }
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void RemoteDisplay_processAdvEvent(rdGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      Display_printf(dispHandle, RD_ROW_ADVSTATE, 0, "Adv Set %d Enabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
      Display_printf(dispHandle, RD_ROW_ADVSTATE, 0, "Adv Set %d Disabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_START:
      break;

    case GAP_EVT_ADV_END:
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);

      Display_printf(dispHandle, RD_ROW_ADVSTATE, 0, "Adv Set %d disabled after conn %d",
                     advSetTerm->handle, advSetTerm->connHandle );
    }
    break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
      break;

    default:
      break;
  }
  
  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}


/*********************************************************************
 * @fn      RemoteDisplay_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void RemoteDisplay_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  rdPairStateData_t *pData = ICall_malloc(sizeof(rdPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(RemoteDisplay_enqueueMsg(RD_PAIR_STATE_EVT, pData) != SUCCESS)
    {
      ICall_freeMsg(pData);
    }
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void RemoteDisplay_passcodeCb(uint8_t *pDeviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  rdPasscodeData_t *pData = ICall_malloc(sizeof(rdPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if(RemoteDisplay_enqueueMsg(RD_PASSCODE_EVT, pData) != SUCCESS)
    {
      ICall_freeMsg(pData);
    }
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void RemoteDisplay_processPairState(rdPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
      Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "Pairing started");
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "Pairing success");
      }
      else
      {
        Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "Pairing fail: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "Encryption success");
      }
      else
      {
        Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "Encryption failed: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "Bond save success");
      }
      else
      {
        Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "Bond save failed: %d", status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      RemoteDisplay_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void RemoteDisplay_processPasscode(rdPasscodeData_t *pPasscodeData)
{
  // Display passcode to user
  if (pPasscodeData->uiOutputs != 0)
  {
    Display_printf(dispHandle, RD_ROW_CONNECTION, 0, "Passcode: %d",
                   B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      RemoteDisplay_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static status_t RemoteDisplay_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  rdEvt_t *pMsg = ICall_malloc(sizeof(rdEvt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }
  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      RemoteDisplay_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t RemoteDisplay_addConn(uint16_t connHandle)
{
  uint8_t i;
  uint8_t status = bleNoResources;
  rdClockEventData_t *paramUpdateEventData;

  // Try to find an available entry
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == LL_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;

      // Allocate data to send through clock handler
      paramUpdateEventData = ICall_malloc(sizeof(rdClockEventData_t) +
                                          sizeof (uint16_t));
      if(paramUpdateEventData)
      {
        paramUpdateEventData->event = RD_SEND_PARAM_UPDATE_EVT;
        *((uint16_t *)paramUpdateEventData->data) = connHandle;

        // Create a clock object and start
        connList[i].pUpdateClock
          = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

        if (connList[i].pUpdateClock)
        {
          Util_constructClock(connList[i].pUpdateClock,
                            RemoteDisplay_clockHandler,
                            RD_SEND_PARAM_UPDATE_DELAY, 0, true,
                            (UArg) paramUpdateEventData);
        }
      }
      else
      {
        status = bleMemAllocError;
      }

      break;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      RemoteDisplay_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t RemoteDisplay_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      RemoteDisplay_clearConnListEntry
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t RemoteDisplay_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != LL_CONNHANDLE_ALL)
  {
    // Get connection index from handle
    connIndex = RemoteDisplay_getConnIndex(connHandle);
    if(connIndex >= MAX_NUM_BLE_CONNS)
	{
	  return(bleInvalidRange);
	}
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == LL_CONNHANDLE_ALL))
    {
      connList[i].connHandle = LL_CONNHANDLE_INVALID;
    }
  }

  return(SUCCESS);
}

/*********************************************************************
 * @fn      RemoteDisplay_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t RemoteDisplay_removeConn(uint16_t connHandle)
{
  uint8_t connIndex = RemoteDisplay_getConnIndex(connHandle);

  if(connIndex != MAX_NUM_BLE_CONNS)
  {
    Clock_Struct* pUpdateClock = connList[connIndex].pUpdateClock;

    if (pUpdateClock != NULL)
    {
      // Stop and destruct the RTOS clock if it's still alive
      if (Util_isActive(pUpdateClock))
      {
        Util_stopClock(pUpdateClock);
      }

      // Destruct the clock object
      Clock_destruct(pUpdateClock);
      // Free clock struct
      ICall_free(pUpdateClock);
    }
    // Clear Connection List Entry
    RemoteDisplay_clearConnListEntry(connHandle);
  }

  return connIndex;
}

/*********************************************************************
 * @fn      RemoteDisplay_processParamUpdate
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static void RemoteDisplay_processParamUpdate(uint16_t connHandle)
{
  gapUpdateLinkParamReq_t req;
  uint8_t connIndex;

  req.connectionHandle = connHandle;
  req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
  req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
  req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
  req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

  connIndex = RemoteDisplay_getConnIndex(connHandle);
  REMOTEDISPLAY_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Deconstruct the clock object
  Clock_destruct(connList[connIndex].pUpdateClock);
  // Free clock struct
  ICall_free(connList[connIndex].pUpdateClock);
  connList[connIndex].pUpdateClock = NULL;

  // Send parameter update
  bStatus_t status = GAP_UpdateLinkParamReq(&req);

  // If there is an ongoing update, queue this for when the udpate completes
  if (status == bleAlreadyInRequestedMode)
  {
    rdConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(rdConnHandleEntry_t));
    if (connHandleEntry)
    {
        connHandleEntry->connHandle = connHandle;

        List_put(&paramUpdateList, (List_Elem *)&connHandleEntry);
    }
  }
}

/*********************************************************************
*********************************************************************/
