/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main_task.c

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

#include "main_task.h"
#include "definitions.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"


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
    This structure should be initialized by the MAIN_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

MAIN_TASK_DATA main_taskData;

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


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MAIN_TASK_Initialize ( void )

  Remarks:
    See prototype in main_task.h.
 */

void MAIN_TASK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    main_taskData.state = MAIN_TASK_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void MAIN_TASK_Tasks ( void )

  Remarks:
    See prototype in main_task.h.
 */

void MAIN_TASK_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( main_taskData.state )
    {
        /* Application's initial state. */
        case MAIN_TASK_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized)
            {

                main_taskData.state = MAIN_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case MAIN_TASK_STATE_SERVICE_TASKS:
        {
            GPIO_RC11_Toggle();
           // printf("\r\n ola led");

            //printf("\n\rMain\n\r");

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
