/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    voltage_measurement_task.c

  Summary:
    Source code for the MPLAB Harmony application.

  Description:
    Implements the logic of the application's state machine and calls API routines
    of other MPLAB Harmony modules. It does not call system interfaces (e.g.,
    "Initialize" and "Tasks" functions) or make assumptions about their calls.
 *******************************************************************************/

// *****************************************************************************
// Section: Included Files
// *****************************************************************************

#include "voltage_measurement_task.h"

#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
#include "definitions.h"
#include "queue.h"
#include "toolchain_specifics.h"

// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************

VOLTAGE_MEASUREMENT_TASK_DATA voltage_measurement_taskData;

xSemaphoreHandle voltageMeasurementSemaphore;

__COHERENT uint16_t voltageMeasurementValue;

// Function prototype
float MeasureVoltage(uint16_t bits);

// *****************************************************************************
// Section: Callback Functions
// *****************************************************************************

void ADCHS_CH8_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    ADCHS_ChannelResultGet(ADCHS_CH8);
    xSemaphoreGiveFromISR(voltageMeasurementSemaphore, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD();
    }
}

// *****************************************************************************
// Section: Utility Functions
// *****************************************************************************

unsigned int millis(void) {
    return (unsigned int)(CORETIMER_CounterGet() / (CORE_TIMER_FREQUENCY / 1000));
}

// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************

void VOLTAGE_MEASUREMENT_TASK_Initialize(void) {
    // Initialize state machine
    voltage_measurement_taskData.state = VOLTAGE_MEASUREMENT_TASK_STATE_INIT;

    // Register ADC callback and enable interrupts
    ADCHS_CallbackRegister(ADCHS_CH8, ADCHS_CH8_Callback, (uintptr_t)NULL);
    ADCHS_ChannelResultInterruptEnable(ADCHS_CH8);
    ADCHS_ChannelConversionStart(ADCHS_CH8);

    // Create and take semaphore
    vSemaphoreCreateBinary(voltageMeasurementSemaphore);
    xSemaphoreTake(voltageMeasurementSemaphore, 0);
}

void VOLTAGE_MEASUREMENT_TASK_Tasks(void) {
    switch (voltage_measurement_taskData.state) {
        case VOLTAGE_MEASUREMENT_TASK_STATE_INIT: {
            if (true) {  // Initialization complete
                ADCHS_ChannelConversionStart(ADCHS_CH8);
                voltage_measurement_taskData.state = VOLTAGE_MEASUREMENT_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case VOLTAGE_MEASUREMENT_TASK_STATE_SERVICE_TASKS: {
            xSemaphoreTake(voltageMeasurementSemaphore, portMAX_DELAY);

            voltageMeasurementValue = MeasureVoltage(ADCHS_ChannelResultGet(ADCHS_CH8));
            ADCHS_ChannelConversionStart(ADCHS_CH8);
            break;
        }

        default: {
            // Handle unexpected states
            break;
        }
    }
}

float MeasureVoltage(uint16_t bits) {
    const float SCALE_FACTOR = 0.1155f;
    const float ADC_MAX = 4095.0f;
    const float V_REF = 3.3f;

    float PDM_Voltage = ((float)bits * V_REF / ADC_MAX) / SCALE_FACTOR;

    if (PDM_Voltage >= 25.0f) {
        GPIO_RG9_LV_ON_Set();
    } else if (PDM_Voltage < 25.0f) {
        static uint16_t previousMillis = 0;
        uint16_t currentMillis = millis();
        uint16_t interval = currentMillis - previousMillis;

        if (PDM_Voltage < 25.0f && PDM_Voltage >= 24.0f && interval >= 500) {
            GPIO_RG9_LV_ON_Toggle();
            previousMillis = currentMillis;
        } else if (PDM_Voltage < 24.0f && PDM_Voltage >= 23.0f && interval >= 100) {
            GPIO_RG9_LV_ON_Toggle();
            previousMillis = currentMillis;
        } else if (PDM_Voltage < 23.0f && interval >= 0) {
            GPIO_RG9_LV_ON_Toggle();
            previousMillis = currentMillis;
        }
    }

    // Send voltage to the queue
    xQueueOverwrite(Bat_Voltage_Queue, &PDM_Voltage);

    return PDM_Voltage;
}

/*******************************************************************************
 End of File
 ******************************************************************************/
