/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    inverter_task.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    Implements the logic of the application's state machine and may call
    API routines of other MPLAB Harmony modules. This file does not call
    system interfaces (e.g., "Initialize" and "Tasks" functions) or make
    assumptions about when those functions are called. That is the responsibility
    of the configuration-specific system files.
 *******************************************************************************/

// *****************************************************************************
// Section: Included Files
// *****************************************************************************

#include "inverter_task.h"

#include <stdio.h>

#include "apps_task.h"
#ifndef FREERTOS_H
#include "FreeRTOS.h"
#endif
#include "../../APPS.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
#include "definitions.h"
#include "queue.h"
#include "semphr.h"

// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************

INVERTER_TASK_DATA inverter_taskData;

// Define a structure to hold the ADC values
typedef struct {
    uint16_t adc0value;
    uint16_t adc3value;
} ADCValues_t;

// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************

bool CanSend_inverter(uint32_t id, uint8_t length, uint8_t *buffer) {
    return CAN1_MessageTransmit(id, length, buffer, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************

void INVERTER_TASK_Initialize(void) {
    // Initialize application state machine to its initial state
    inverter_taskData.state = INVERTER_TASK_STATE_INIT;

    CAN1_Initialize();
    APPS_Init(0.5, 4.5, 0.1, 0);
}

void INVERTER_TASK_Tasks(void) {
    switch (inverter_taskData.state) {
        case INVERTER_TASK_STATE_INIT: {
            bool appInitialized = true;

            if (appInitialized) {
                inverter_taskData.state = INVERTER_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case INVERTER_TASK_STATE_SERVICE_TASKS: {
            static ADCValues_t receivedValues;
            static BaseType_t xStatus;

            // Take semaphore to proceed
            xSemaphoreTake(R2D_semaphore, portMAX_DELAY);

            // Wait to receive data from the queue
            xStatus = xQueueReceive(Inverter_control_Queue, &receivedValues, pdMS_TO_TICKS(20));
            if (xStatus == pdPASS) {
                static int power = 0;
                static int power_mean = 0;

                uint16_t adc0value = receivedValues.adc0value;
                uint16_t adc3value = receivedValues.adc3value;

                printf("\n\rADC0 Value: %u\n", adc0value);
                printf("\n\rADC3 Value: %u\n", adc3value);

                // Calculate mean value and map to power
                power_mean = (adc0value + adc3value) / 2;
                power = map(power_mean, 0, 4095, 0, 1000);

                printf("\n\rPower: %u\n", power);

                // Prepare CAN message
                uint32_t id = 0x24;
                uint8_t length = 6;
                uint8_t message[8];

                message[0] = adc0value & 0xFF;
                message[1] = (adc0value >> 8) & 0xFF;
                message[2] = adc3value & 0xFF;
                message[3] = (adc3value >> 8) & 0xFF;
                message[4] = power & 0xFF;
                message[5] = (power >> 8) & 0xFF;

                CanSend_inverter(id, length, message);
            }

            break;
        }

        default: {
            // Handle unexpected states
            break;
        }
    }
}

/*******************************************************************************
 End of File
 ******************************************************************************/
