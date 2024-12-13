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
#include "queue.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
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
    This structure should be initialized by the CAN_SEND_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

CAN_SEND_TASK_DATA can_send_taskData;

static uint8_t message[8];
float voltage;
float temp;
static uint32_t id = 0;
static uint8_t length = 8;

CANFD_MSG_RX_ATTRIBUTE msgAttr = CANFD_MSG_RX_DATA_FRAME;
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
bool CanSend_task(uint32_t id, uint8_t length, uint8_t *buffer) {
    return CAN1_MessageTransmit(id, length, buffer, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
}

/* TODO:  Add any necessary local functions.
*/



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
    
    for(int i = 0; i<8;i++){
        message[i] = 0;
    }

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
            
            
            xQueueReceive(Bat_Voltage_Queue,&voltage,pdMS_TO_TICKS(300));
            xQueueReceive(Temperature_Queue,&temp,portMAX_DELAY);
            temp = 17;
            //voltage = 240;
            printf("\n\n\n\r Queue temp : %f \r\n\n\n\r",temp);
            int voltage_int = voltage*10;
            uint8_t MSB_voltage = (voltage_int >> 8) & 0xF;
            uint8_t LSB_voltage = (voltage_int << 8) & 0xF;
            int temp_int = temp*10;
            uint8_t MSB_temp = (temp_int >> 8) & 0xF;
            uint8_t LSB_temp = (temp_int << 8) & 0xF;
            LED_RB13_Toggle();
            printf("\n\n\rMSD : %d  \tLSB: %d\n\n\n\\r",MSB_temp,LSB_temp);
            message[0] = MSB_voltage;
            message[1] = LSB_voltage;
            message[2] = MSB_temp;
            message[3] = LSB_temp;
            id = 0x54;
            
           // xSemaphoreTake(CAN_Mutex, portMAX_DELAY);
            //{
                CanSend_task(id, length, message);
            //}
           // xSemaphoreGive(CAN_Mutex);
            //*** Data to send to can ***//
            //Low Voltage battery
            //Ignition            uint8_t LSB_temp = (temp_int << 8) & 0xF;

            //VCU state
            //Temperatures
            //Fan control
            //**************************//         
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
