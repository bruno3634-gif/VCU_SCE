/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    inverter_task.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
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
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the INVERTER_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
 */

INVERTER_TASK_DATA inverter_taskData;

// Define a structure to hold the ADC values

typedef struct {
    uint16_t adc0value;
    uint16_t adc3value;
} ADCValues_t;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
 */
bool CanSend_inverter(uint32_t id, uint8_t length, uint8_t *buffer) {
    return CAN1_MessageTransmit(id, length, buffer, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void INVERTER_TASK_Initialize ( void )

  Remarks:
    See prototype in inverter_task.h.
 */

void INVERTER_TASK_Initialize(void) {
    /* Place the App state machine in its initial state. */
    inverter_taskData.state = INVERTER_TASK_STATE_INIT;

    CAN1_Initialize();
    APPS_Init(0.5, 4.5, 0.1, 0);
}

/******************************************************************************
  Function:
    void INVERTER_TASK_Tasks ( void )

  Remarks:
    See prototype in inverter_task.h.
 */

void INVERTER_TASK_Tasks(void) {
    /* Check the application's current state. */
    switch (inverter_taskData.state) {
            /* Application's initial state. */
        case INVERTER_TASK_STATE_INIT:
        {
            bool appInitialized = true;

            if (appInitialized) {
                inverter_taskData.state = INVERTER_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case INVERTER_TASK_STATE_SERVICE_TASKS:
        {
            static ADCValues_t receivedValues;
            static BaseType_t xStatus;
            xSemaphoreTake(R2D_semaphore, portMAX_DELAY);
            // Wait to receive data from the queue
            xStatus = xQueueReceive(Inverter_control_Queue, &receivedValues, portMAX_DELAY);
            if (xStatus == pdPASS) {
                static int power = 0;
                static int power_mean = 0;

                // Process the received data
                uint16_t adc0value = receivedValues.adc0value;
                uint16_t adc3value = receivedValues.adc3value;
                // printf("Received ADC0 Value: %u\n", adc0value);
                // printf("Received ADC3 Value: %u\n", adc3value);

                printf("\n\rADC0 Value: %u\n", adc0value);
                printf("\n\rADC3 Value: %u\n", adc3value);
                // power = APPS_Function(adc0value, adc3value);
                // calculate mean value
                power_mean = (adc0value + adc3value) / 2;
                power = map(power_mean, 0, 4095, 0, 1000);

                printf("\n\rPower: %u\n", power);

                // Send the data over CAN
                uint32_t id = 0x24;
                uint8_t length = 2;
                uint8_t message[8];
                message[0] = 0x01; // send drive enable signal
                // send drive enable signal
               // xSemaphoreTake(CAN_Mutex, portMAX_DELAY);
                //{
                    CanSend_inverter(id, length, message);
                //}
               // xSemaphoreGive(CAN_Mutex);

                // send the data to the inverter control
                id = 0x24;
                length = 6;
                message[0] = adc0value & 0xFF;
                message[1] = (adc0value >> 8) & 0xFF;
                message[2] = adc3value & 0xFF;
                message[3] = (adc3value >> 8) & 0xFF;
                message[4] = power & 0xFF;
                message[5] = (power >> 8) & 0xFF;

               // xSemaphoreTake(CAN_Mutex, portMAX_DELAY); {
                    CanSend_inverter(id, length, message);
                //}
                //xSemaphoreGive(CAN_Mutex);
            }

            break;
        }

            /* TODO: implement your application state machine.*/

            /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

/*******************************************************************************
 End of File
 */
