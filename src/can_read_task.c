/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    can_read_task.c

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
#include "can_read_task.h"
#include "peripheral/canfd/plib_canfd1.h"
#include "peripheral/adchs/plib_adchs_common.h"
#include "peripheral/gpio/plib_gpio.h"
#include "definitions.h"
#include "queue.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
uint8_t state;
#define AS_EMERGENGENCY 0x502

typedef struct {
    uint32_t id;         // CAN ID
    uint8_t data[8];     // CAN data (up to 8 bytes)
    uint8_t dlc;         // Data length code
}Received_CANMessage;


Received_CANMessage message_to_send;
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the CAN_READ_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
 */

CAN_READ_TASK_DATA can_read_taskData;

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
void task_function();

void task_function(){

   // printf("\n\rCAN read\n\r");
    LED_RA10_Toggle();
    CANFD_MSG_RX_ATTRIBUTE msgAttr;
    uint8_t lenght;
    uint8_t rx_message[8];



    if (CAN1_MessageReceive(&can_read_taskData.id, &lenght, rx_message, 0, 2, &msgAttr) == true) {
        //CAN1_MessageTransmit(0x200,lenght,can_read_taskData.rx_message,0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
        switch (can_read_taskData.id) {
            case AS_EMERGENGENCY:
                message_to_send.id = can_read_taskData.id;
                message_to_send.dlc = lenght;
                for(int i = 0; i<8;i++){
                    message_to_send.data[i] = rx_message[i];
                }
                xQueueSend(AS_Emergency_Queue,&message_to_send,portMAX_DELAY);
                printf("\r\n\n\nReceived 0x502\r\n\n\n");
                break;
            default:
                break;
        }
    } else {
        taskYIELD();
    }
}




// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void CAN_READ_TASK_Initialize ( void )

  Remarks:
    See prototype in can_read_task.h.
 */

void CAN_READ_TASK_Initialize(void) {
    /* Place the App state machine in its initial state. */
    can_read_taskData.state = CAN_READ_TASK_STATE_INIT;

    CAN1_Initialize();

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

/******************************************************************************
  Function:
    void CAN_READ_TASK_Tasks ( void )

  Remarks:
    See prototype in can_read_task.h.
 */

void CAN_READ_TASK_Tasks(void) {

    /* Check the application's current state. */
    switch (can_read_taskData.state) {
            /* Application's initial state. */
        case CAN_READ_TASK_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized) {

                can_read_taskData.state = CAN_READ_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case CAN_READ_TASK_STATE_SERVICE_TASKS:
        {

            xSemaphoreTake(CAN_Mutex, portMAX_DELAY);
            {
                task_function();
            }
            xSemaphoreGive(CAN_Mutex);

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
