#include "as_emergency_task.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
#include "FreeRTOS.h"
#include "definitions.h"
#include "portmacro.h"
#include "queue.h"

AS_EMERGENCY_TASK_DATA as_emergency_taskData;

typedef struct {
    uint32_t id; // CAN ID
    uint8_t data[8]; // CAN data (up to 8 bytes)
    uint8_t dlc; // Data length code
} Received_CANMessage;

int time_counter = 0; // used to count the time for the emergency buzzer

Received_CANMessage CAN_R_Q; // received CAN message  from the queue
int value = 0;
int prev_value = 0;

void AS_EMERGENCY_TASK_Initialize(void) {
    as_emergency_taskData.state = AS_EMERGENCY_TASK_STATE_INIT;
}

void AS_EMERGENCY_TASK_Tasks(void) {
    switch (as_emergency_taskData.state) {
        case AS_EMERGENCY_TASK_STATE_INIT:
        {
            bool appInitialized = true;

            if (appInitialized) {
                as_emergency_taskData.state = AS_EMERGENCY_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case AS_EMERGENCY_TASK_STATE_SERVICE_TASKS:
        {
            static BaseType_t xStatus;

            // Set the buzzer initially
            buzzer_Set();

            // Wait to receive data from the AS_Emergency_Queue indefinitely
            xStatus = xQueueReceive(AS_Emergency_Queue, &CAN_R_Q, portMAX_DELAY);

            // Extract the relevant value from the received CAN message
            value = CAN_R_Q.data[0] & 0b00000111;

            // Check if data was received successfully
            if (xStatus == pdPASS) {
                // Check if the previous value was not an emergency
                if (prev_value != 4) {
                    // Check if the current value indicates an emergency
                    if (value == 4) {
                        // Emergency detected, toggle the buzzer 32 times with a delay
                        for (int i = 0; i < 32; i++) {
                            buzzer_Toggle();
                            vTaskDelay(pdMS_TO_TICKS(250));
                        }
                        // Ensure the buzzer is set after toggling
                        buzzer_Set();
                    }
                }
                // Update the previous value with the current value
                prev_value = value;
            }
            break;
        }
        default:
        {
            break;
        }
    }
}