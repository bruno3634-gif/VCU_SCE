/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    can_send_task.c

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

#include <stdio.h>

#include "can_send_task.h"
//#include "peripheral/canfd/plib_canfd1.h"
#include "definitions.h"

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
    This structure should be initialized by the CAN_SEND_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

CAN_SEND_TASK_DATA can_send_taskData;

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
CANFD_MSG_RX_ATTRIBUTE msgAttr = CANFD_MSG_RX_DATA_FRAME;


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void CAN_SEND_TASK_Initialize ( void )

  Remarks:
    See prototype in can_send_task.h.
 */

void CAN_SEND_TASK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    can_send_taskData.state = CAN_SEND_TASK_STATE_INIT;
    


    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    CAN1_Initialize();
    
}


/******************************************************************************
  Function:
    void CAN_SEND_TASK_Tasks ( void )

  Remarks:
    See prototype in can_send_task.h.
 */

void CAN_SEND_TASK_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( can_send_taskData.state )
    {
        /* Application's initial state. */
        case CAN_SEND_TASK_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized)
            {

                can_send_taskData.state = CAN_SEND_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case CAN_SEND_TASK_STATE_SERVICE_TASKS:
        {
          //  printf("\n\rCAN task\n\r");
            uint8_t message[64];
      
            for (int count = 8; count >=1; count--){
                message[count - 1] = count;
            }
            if(CAN1_MessageTransmit(0x69, 8, message, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME)){
                
                
            }else{
            //    printf("Failed to transmit message");
            }
            LED_RB13_Toggle();
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
