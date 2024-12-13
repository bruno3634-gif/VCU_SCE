/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    can_send_task.c

  Summary:
    Contains source code for the MPLAB Harmony application.

  Description:
    Implements the logic of the application's state machine and may call API
    routines of other MPLAB Harmony modules. This file does not call system
    interfaces (e.g., "Initialize" and "Tasks" functions) or make assumptions
    about when those functions are called.
 *******************************************************************************/

// *****************************************************************************
// Section: Included Files
// *****************************************************************************

#include "can_send_task.h"

#include <stdio.h>

#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
#include "definitions.h"
#include "queue.h"
#include "semphr.h"

// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************

CAN_SEND_TASK_DATA can_send_taskData;
static uint8_t message[8];
float voltage;
float temp;
static uint32_t id = 0;
static uint8_t length = 8;
CANFD_MSG_RX_ATTRIBUTE msgAttr = CANFD_MSG_RX_DATA_FRAME;

// *****************************************************************************
// Section: Local Functions
// *****************************************************************************

bool CanSend_task(uint32_t id, uint8_t length, uint8_t *buffer) {
    return CAN1_MessageTransmit(id, length, buffer, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
}

// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************

void CAN_SEND_TASK_Initialize(void) {
    // Initialize application state machine to its initial state
    can_send_taskData.state = CAN_SEND_TASK_STATE_INIT;

    // Clear message buffer
    memset(message, 0, sizeof(message));

    // Initialize CAN module
    CAN1_Initialize();
}

void CAN_SEND_TASK_Tasks(void) {
    switch (can_send_taskData.state) {
        case CAN_SEND_TASK_STATE_INIT: {
            bool appInitialized = true;

            if (appInitialized) {
                can_send_taskData.state = CAN_SEND_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case CAN_SEND_TASK_STATE_SERVICE_TASKS: {
            // Receive data from queues
            if (xQueueReceive(Bat_Voltage_Queue, &voltage, pdMS_TO_TICKS(300)) == pdPASS &&
                xQueueReceive(Temperature_Queue, &temp, portMAX_DELAY) == pdPASS) {

                // Convert voltage and temperature to integer representation
                int voltage_int = (int)(voltage * 10);
                int temp_int = (int)(temp * 10);

                // Toggle LED
                LED_RB13_Toggle();

                // Prepare CAN message
                message[0] = (voltage_int >> 8) & 0xFF;
                message[1] = voltage_int & 0xFF;
                message[2] = (temp_int >> 8) & 0xFF;
                message[3] = temp_int & 0xFF;

                // Set CAN message ID
                id = 0x54;

                // Transmit CAN message
                CanSend_task(id, length, message);
            }
            break;
        }

        default:
            // Handle unexpected states
            break;
    }
}

/*******************************************************************************
 End of File
 *******************************************************************************/
