#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include <stdio.h>
#include "semphr.h"

#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main(void) {
    /* Initialize all modules */
    SYS_Initialize(NULL);

    // Create queues for inter-task communication
    Inverter_control_Queue = xQueueCreate(1, sizeof (long)); // Queue for inverter control messages
    AS_Emergency_Queue = xQueueCreate(1, sizeof (CANMessage)); // Queue for AS emergency messages
    Bat_Voltage_Queue = xQueueCreate(1, sizeof (float)); // Queue for battery voltage readings
    Temperature_Queue = xQueueCreate(1, sizeof (float)); // Queue for temperature readings

    // Create a mutex for CAN resource protection
    CAN_Mutex = xSemaphoreCreateMutex();

    // Create a binary semaphore for R2D (Ready-to-Drive) signaling
    vSemaphoreCreateBinary(R2D_semaphore);

    while (true) {
        SYS_Tasks();
    }

    return (EXIT_FAILURE);
}