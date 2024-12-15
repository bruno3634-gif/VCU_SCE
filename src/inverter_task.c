#include "inverter_task.h"
#include <stdio.h>
#include "apps_task.h"
#ifndef FREERTOS_H
#include "FreeRTOS.h"
#endif
#include "../../APPS.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
#include "definitions.h"
#include "queue.h"
#include "semphr.h"


INVERTER_TASK_DATA inverter_taskData;

// Define a structure to hold the ADC values

typedef struct {
    uint16_t adc0value;
    uint16_t adc3value;
} ADCValues_t;


bool CanSend_inverter(uint32_t id, uint8_t length, uint8_t *buffer) {
    return CAN1_MessageTransmit(id, length, buffer, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void INVERTER_TASK_Initialize(void) {
    // Initialize application state machine to its initial state
    inverter_taskData.state = INVERTER_TASK_STATE_INIT;

    // Initialize CAN1 module
    CAN1_Initialize();

    // Initialize APPS (Accelerator Pedal Position Sensor) with calibration values
    APPS_Init(0.5, 4.5, 0.1, 0);
}

void INVERTER_TASK_Tasks(void) {
    switch (inverter_taskData.state) {
        case INVERTER_TASK_STATE_INIT:
        {
            // Initialize the application state
            bool appInitialized = true;

            // If initialization is successful, transition to service tasks state
            if (appInitialized) {
                inverter_taskData.state = INVERTER_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }
        case INVERTER_TASK_STATE_SERVICE_TASKS:
        {
            static ADCValues_t receivedValues; // Structure to hold received ADC values
            static BaseType_t xStatus; // Variable to hold queue receive status

            // Take semaphore to proceed
            xSemaphoreTake(R2D_semaphore, portMAX_DELAY);

            // Wait to receive data from the Inverter control queue with a timeout of 20 ms
            xStatus = xQueueReceive(Inverter_control_Queue, &receivedValues, pdMS_TO_TICKS(20));
            if (xStatus == pdPASS) {
                static int power = 0; // Variable to hold calculated power
                static int power_mean = 0; // Variable to hold mean ADC value

                // Extract ADC values from the received structure
                uint16_t adc0value = receivedValues.adc0value;
                uint16_t adc3value = receivedValues.adc3value;

                // Print ADC values for debugging
                printf("\n\rADC0 Value: %u\n", adc0value);
                printf("\n\rADC3 Value: %u\n", adc3value);

                // Calculate mean value of the two ADC readings
                power_mean = (adc0value + adc3value) / 2;

                // Map the mean ADC value to a power value (0 to 1000)
                power = map(power_mean, 0, 4095, 0, 1000);

                // Print calculated power for debugging
                printf("\n\rPower: %u\n", power);

                // Prepare CAN message with ADC values and calculated power
                uint32_t id = 0x24; // CAN message ID
                uint8_t length = 6; // CAN message length
                uint8_t message[8]; // CAN message buffer

                // Fill CAN message buffer with ADC values and power
                message[0] = adc0value & 0xFF;
                message[1] = (adc0value >> 8) & 0xFF;
                message[2] = adc3value & 0xFF;
                message[3] = (adc3value >> 8) & 0xFF;
                message[4] = power & 0xFF;
                message[5] = (power >> 8) & 0xFF;

                // Send CAN message
                CanSend_inverter(id, length, message);
            }
            break;
        }
        default:
        {
            break;
        }
    }
}