/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    voltage_measurement_task.c

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

#include "voltage_measurement_task.h"

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
    This structure should be initialized by the VOLTAGE_MEASUREMENT_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

VOLTAGE_MEASUREMENT_TASK_DATA voltage_measurement_taskData;



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
    void VOLTAGE_MEASUREMENT_TASK_Initialize ( void )

  Remarks:
    See prototype in voltage_measurement_task.h.
 */

void VOLTAGE_MEASUREMENT_TASK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    voltage_measurement_taskData.state = VOLTAGE_MEASUREMENT_TASK_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void VOLTAGE_MEASUREMENT_TASK_Tasks ( void )

  Remarks:
    See prototype in voltage_measurement_task.h.
 */

void VOLTAGE_MEASUREMENT_TASK_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( voltage_measurement_taskData.state )
    {
        /* Application's initial state. */
        case VOLTAGE_MEASUREMENT_TASK_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized)
            {

                voltage_measurement_taskData.state = VOLTAGE_MEASUREMENT_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case VOLTAGE_MEASUREMENT_TASK_STATE_SERVICE_TASKS:
        {

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
