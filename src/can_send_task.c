#include "can_send_task.h"
#include <stdio.h>
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
#include "definitions.h"
#include "queue.h"
#include "semphr.h"


CAN_SEND_TASK_DATA can_send_taskData;
static uint8_t message[8];
float voltage;
float temp;
static uint32_t id = 0;
static uint8_t length = 8;
CANFD_MSG_RX_ATTRIBUTE msgAttr = CANFD_MSG_RX_DATA_FRAME;


bool CanSend_task(uint32_t id, uint8_t length, uint8_t *buffer) {
    return CAN1_MessageTransmit(id, length, buffer, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
}

void CAN_SEND_TASK_Initialize(void) {
    // Initialize application state machine to its initial state
    can_send_taskData.state = CAN_SEND_TASK_STATE_INIT;

    // Clear message buffer
    memset(message, 0, sizeof (message));

    // Initialize CAN module
    CAN1_Initialize();
}

void CAN_SEND_TASK_Tasks(void) {
    switch (can_send_taskData.state) {
        case CAN_SEND_TASK_STATE_INIT:
        {
            // Initialize the application state
            bool appInitialized = true;

            // If initialization is successful, transition to service tasks state
            if (appInitialized) {
                can_send_taskData.state = CAN_SEND_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }
        case CAN_SEND_TASK_STATE_SERVICE_TASKS:
        {
            // Receive data from the battery voltage queue with a timeout of 300 ms
            if (xQueueReceive(Bat_Voltage_Queue, &voltage, pdMS_TO_TICKS(300)) == pdPASS &&
                    // Receive data from the temperature queue indefinitely
                    xQueueReceive(Temperature_Queue, &temp, portMAX_DELAY) == pdPASS) {
                // Convert voltage and temperature to integer representation
                int voltage_int = (int) (voltage * 10);
                int temp_int = (int) (temp * 10);

                // Toggle LED to indicate activity
                LED_RB13_Toggle();

                // Prepare CAN message with voltage and temperature data
                message[0] = (voltage_int >> 8) & 0xFF;
                message[1] = voltage_int & 0xFF;
                message[2] = (temp_int >> 8) & 0xFF;
                message[3] = temp_int & 0xFF;

                // Set CAN message ID
                id = 0x54;

                // Transmit CAN message
                CanSend_task(id, length, message);
            }
            break;
        }
        default:

            break;
    }
}
