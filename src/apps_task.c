// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "apps_task.h"

#include <stdio.h>

#include "inverter_task.h"
#include "peripheral/adchs/plib_adchs.h"
#ifndef FREERTOS_H
#include "FreeRTOS.h"
#endif
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
#include "definitions.h"
#include "queue.h"
#include "semphr.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

APPS_TASK_DATA apps_taskData;

static SemaphoreHandle_t ADC0_SEMAPHORE;
static SemaphoreHandle_t ADC3_SEMAPHORE;

typedef struct {
    uint16_t adc0value;
    uint16_t adc3value;
} ADCValues_t;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void ADC0_callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ADCHS_ChannelResultGet(ADCHS_CH0);

    xSemaphoreGiveFromISR(ADC0_SEMAPHORE, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void ADC3_callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ADCHS_ChannelResultGet(ADCHS_CH3);

    xSemaphoreGiveFromISR(ADC3_SEMAPHORE, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

void APPS_TASK_Initialize(void) {
    apps_taskData.state = APPS_TASK_STATE_INIT;

    ADCHS_CallbackRegister(ADCHS_CH0, ADC0_callback, (uintptr_t) NULL);
    ADCHS_CallbackRegister(ADCHS_CH3, ADC3_callback, (uintptr_t) NULL);

    vSemaphoreCreateBinary(ADC0_SEMAPHORE);
    xSemaphoreTake(ADC0_SEMAPHORE, portMAX_DELAY);

    vSemaphoreCreateBinary(ADC3_SEMAPHORE);
    xSemaphoreTake(ADC3_SEMAPHORE, portMAX_DELAY);
}

void APPS_TASK_Tasks(void) {
    switch (apps_taskData.state) {
        case APPS_TASK_STATE_INIT:
        {
            bool appInitialized = true;

            if (appInitialized) {
                apps_taskData.state = APPS_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }
        case APPS_TASK_STATE_SERVICE_TASKS:
        {
            static portBASE_TYPE xStatus;

            ADCHS_ChannelConversionStart(ADCHS_CH3);
            ADCHS_ChannelConversionStart(ADCHS_CH0);

            // Wait for the ADC to finish
            if (xSemaphoreTake(ADC0_SEMAPHORE, portMAX_DELAY) == pdTRUE) {
            }
            // wait for the ADC to finish
            if (xSemaphoreTake(ADC3_SEMAPHORE, portMAX_DELAY) == pdTRUE) {
            }

            // get the ADC values
            uint16_t adc0value = ADCHS_ChannelResultGet(ADCHS_CH0);
            uint16_t adc3value = ADCHS_ChannelResultGet(ADCHS_CH3);

            // convert the ADC values to voltage for debug
            // float voltage0 = adc0value * 3.3 / 4096;
            // float voltage3 = adc3value * 3.3 / 4096;

            LED_RB10_Toggle(); // indicator of the task running

            ADCValues_t adcValues;
            adcValues.adc0value = adc0value;
            adcValues.adc3value = adc3value;

            // send the values to the inverter control task
            xStatus = xQueueOverwrite(Inverter_control_Queue, &adcValues);
            if (xStatus != pdPASS) {
            }

            break;
        }
        default:
        {
            break;
        }
    }
}