/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include <stdio.h>
//#include <FreeRTOS.h>
//#include "../../u8g2-master/csrc/u8g2.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

//QueueHandle_t Inverter_control_Queue;


int main(void) {
    /* Initialize all modules */
    SYS_Initialize(NULL);
    

    
    
    while (true) {
        Inverter_control_Queue = xQueueCreate(10, sizeof ( long));
        SYS_Tasks();
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE);
}


/*******************************************************************************
 End of File
 */

