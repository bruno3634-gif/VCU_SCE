#include "voltage_measurement_task.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
#include "definitions.h"
#include "queue.h"
#include "toolchain_specifics.h"

VOLTAGE_MEASUREMENT_TASK_DATA voltage_measurement_taskData; // Task data structure

xSemaphoreHandle voltageMeasurementSemaphore; // Semaphore for ADC synchronization

__COHERENT uint16_t voltageMeasurementValue; // Variable to store ADC result

// Function prototype
float MeasureVoltage(uint16_t bits);

void ADCHS_CH8_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    // Retrieve ADC result (even if unused here)
    ADCHS_ChannelResultGet(ADCHS_CH8);
    // Give semaphore to indicate ADC data is ready
    xSemaphoreGiveFromISR(voltageMeasurementSemaphore, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) {
        // Request context switch if a higher priority task was woken
        portYIELD();
    }
}


unsigned int millis(void) {
    // Calculate milliseconds from core timer count
    return (unsigned int) (CORETIMER_CounterGet() / (CORE_TIMER_FREQUENCY / 1000));
}

void VOLTAGE_MEASUREMENT_TASK_Initialize(void) {
    // Initialize state machine
    voltage_measurement_taskData.state = VOLTAGE_MEASUREMENT_TASK_STATE_INIT;

    // Register ADC callback and enable interrupts
    ADCHS_CallbackRegister(ADCHS_CH8, ADCHS_CH8_Callback, (uintptr_t) NULL);
    ADCHS_ChannelResultInterruptEnable(ADCHS_CH8);
    ADCHS_ChannelConversionStart(ADCHS_CH8);

    // Create and take semaphore
    vSemaphoreCreateBinary(voltageMeasurementSemaphore);
    xSemaphoreTake(voltageMeasurementSemaphore, 0);
}

void VOLTAGE_MEASUREMENT_TASK_Tasks(void) {
    switch (voltage_measurement_taskData.state) {
        case VOLTAGE_MEASUREMENT_TASK_STATE_INIT:
        {
            // If initialization is complete, transition to service tasks state
            if (true) {
                ADCHS_ChannelConversionStart(ADCHS_CH8);
                voltage_measurement_taskData.state = VOLTAGE_MEASUREMENT_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }
        case VOLTAGE_MEASUREMENT_TASK_STATE_SERVICE_TASKS:
        {
            // Wait for ADC conversion result
            xSemaphoreTake(voltageMeasurementSemaphore, portMAX_DELAY);

            // Measure voltage and start next ADC conversion
            voltageMeasurementValue = MeasureVoltage(ADCHS_ChannelResultGet(ADCHS_CH8));
            ADCHS_ChannelConversionStart(ADCHS_CH8);
            break;
        }
        default:
        {
            break;
        }
    }
}

float MeasureVoltage(uint16_t bits) {
    const float SCALE_FACTOR = 0.1155f; // Scale factor for voltage measurement
    const float ADC_MAX = 4095.0f; // Maximum ADC value (12-bit ADC)
    const float V_REF = 3.3f; // Reference voltage

    // Calculate PDM voltage from ADC bits
    float PDM_Voltage = ((float) bits * V_REF / ADC_MAX) / SCALE_FACTOR;

    // Control GPIO based on voltage levels
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