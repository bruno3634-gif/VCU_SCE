/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    as_emergency_task.c

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
#include "FreeRTOS.h"
#include "as_emergency_task.h"
#include "portmacro.h"
#include "queue.h"
#include"../SCE_VCU_FreeRTOS.X/queue_manager.h"
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
    This structure should be initialized by the AS_EMERGENCY_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

AS_EMERGENCY_TASK_DATA as_emergency_taskData;

typedef struct {
    uint32_t id;         // CAN ID
    uint8_t data[8];     // CAN data (up to 8 bytes)
    uint8_t dlc;         // Data length code
}Received_CANMessage;

int time_counter = 0; 

Received_CANMessage CAN_R_Q;
int value = 0;
int prev_value = 0;
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
    void AS_EMERGENCY_TASK_Initialize ( void )

  Remarks:
    See prototype in as_emergency_task.h.
 */

void AS_EMERGENCY_TASK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    as_emergency_taskData.state = AS_EMERGENCY_TASK_STATE_INIT;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void AS_EMERGENCY_TASK_Tasks ( void )

  Remarks:
    See prototype in as_emergency_task.h.
 */

void AS_EMERGENCY_TASK_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( as_emergency_taskData.state )
    {
        /* Application's initial state. */
        case AS_EMERGENCY_TASK_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized)
            {

                as_emergency_taskData.state = AS_EMERGENCY_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case AS_EMERGENCY_TASK_STATE_SERVICE_TASKS:
        {
            static BaseType_t xStatus;

            // Wait to receive data from the queue
            buzzer_Set();
            xStatus = xQueueReceive(AS_Emergency_Queue,&CAN_R_Q , portMAX_DELAY);
            value = CAN_R_Q.data[0] & 0b00000111;
            if(xStatus == pdPASS){
                if(prev_value != 4){
                    if(value == 4)
                    {
                        //emergency
                        for(int i = 0; i<32;i++){
                            buzzer_Toggle();
                            vTaskDelay(pdMS_TO_TICKS(250));
                        }
                        buzzer_Set();
                    }
                }
                prev_value = value;
                
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
