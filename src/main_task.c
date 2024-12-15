
#include "main_task.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
#include "definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

MAIN_TASK_DATA main_taskData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

void MAIN_TASK_Initialize(void) {
    // Initialize the application state to the initial state
    main_taskData.state = MAIN_TASK_STATE_INIT;
}

void MAIN_TASK_Tasks(void) {
    // Check the application's current state
    switch (main_taskData.state) {
            // Application's initial state
        case MAIN_TASK_STATE_INIT:
        {
            // Assume the application is initialized successfully
            bool appInitialized = true;
            // If initialization is successful, transition to service tasks state
            if (appInitialized) {
                main_taskData.state = MAIN_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }
        case MAIN_TASK_STATE_SERVICE_TASKS:
        {
            // Toggle GPIO pin RC11 to indicate activity
            GPIO_RC11_Toggle();
            break;
        }

        default:
        {
            break;
        }
    }
}
