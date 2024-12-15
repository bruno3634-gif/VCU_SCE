#include "can_read_task.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
#include "FreeRTOS.h"
#include "definitions.h"
#include "peripheral/adchs/plib_adchs_common.h"
#include "peripheral/canfd/plib_canfd1.h"
#include "peripheral/gpio/plib_gpio.h"
#include "queue.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

uint8_t state; // State of the task

#define AS_EMERGENGENCY 0x502

typedef struct {
    uint32_t id; // CAN ID
    uint8_t data[8]; // CAN data (up to 8 bytes)
    uint8_t dlc; // Data length code
} Received_CANMessage;

Received_CANMessage message_to_send;

CAN_READ_TASK_DATA can_read_taskData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void task_function(void) {
    // Toggle LED RA10 to indicate task activity
    LED_RA10_Toggle();

    // Variables to store message attributes
    CANFD_MSG_RX_ATTRIBUTE msgAttr;
    uint8_t length;
    uint8_t rx_message[8];

    if (CAN1_MessageReceive(&can_read_taskData.id, &length, rx_message, 0, 2, &msgAttr) == true) {
        // Process the received message based on its ID
        switch (can_read_taskData.id) {
            case AS_EMERGENGENCY:
                // Prepare the message to send to the AS_Emergency_Queue
                message_to_send.id = can_read_taskData.id;
                message_to_send.dlc = length;
                for (int i = 0; i < 8; i++) {
                    message_to_send.data[i] = rx_message[i];
                }
                // Send the message to the AS_Emergency_Queue
                xQueueSend(AS_Emergency_Queue, &message_to_send, portMAX_DELAY);
                // Print a debug message indicating the receipt of the emergency message
                printf("\r\n\n\nReceived 0x502\r\n\n\n");
                break;
            default:
                break;
        }
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

void CAN_READ_TASK_Initialize(void) {
    can_read_taskData.state = CAN_READ_TASK_STATE_INIT;
    CAN1_Initialize();
}

void CAN_READ_TASK_Tasks(void) {
    switch (can_read_taskData.state) {
        case CAN_READ_TASK_STATE_INIT:
        {
            // Initialize the application state
            bool appInitialized = true;
            if (appInitialized) {
                // Transition to service tasks state if initialization is successful
                can_read_taskData.state = CAN_READ_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }
        case CAN_READ_TASK_STATE_SERVICE_TASKS:
        {
            // Take the CAN mutex to ensure exclusive access to CAN resources
            xSemaphoreTake(CAN_Mutex, portMAX_DELAY);
            {
                // Execute the task function to handle CAN message reception
                task_function();
            }
            // Release the CAN mutex after processing
            xSemaphoreGive(CAN_Mutex);
            break;
        }
        default:
        {
            break;
        }
    }
}
/*******************************************************************************
 End of File
 */
