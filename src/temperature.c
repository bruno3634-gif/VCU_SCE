#include "temperature.h"
#include <math.h>
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
#include "definitions.h"
#include "queue.h"
#include "semphr.h"

// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************

TEMPERATURE_DATA temperatureData;
SemaphoreHandle_t ADC9_Temp_SEMAPHORE;

// *****************************************************************************
// Section: Callback Functions
// *****************************************************************************

void ADCHS_CH9_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    ADCHS_ChannelResultGet(ADCHS_CH9);
    xSemaphoreGiveFromISR(ADC9_Temp_SEMAPHORE, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD();
    }
}

// *****************************************************************************
// Section: Utility Functions
// *****************************************************************************

float MeasureTemperature(uint16_t bits) {
    const float SERIES_RESISTOR = 10000.0f; // 10kΩ resistor
    const float NOMINAL_RESISTANCE = 10000.0f; // NTC 10kΩ at 25°C
    const float NOMINAL_TEMPERATURE = 25.0f; // 25°C
    const float B_COEFFICIENT = 3976.0f; // B coefficient
    const float ADC_MAX = 4095.0f; // 12-bit ADC
    const float V_REF = 3.3f; // Reference voltage

    // Handle invalid ADC values
    if (bits == 0) {
        return -273.0f; // Absolute zero for invalid ADC value
    }

    // Calculate voltage from ADC bits
    float voltage = (float) bits * V_REF / ADC_MAX;

    // Calculate thermistor resistance
    float resistance = SERIES_RESISTOR * (V_REF / voltage - 1.0f);

    // Apply the Steinhart-Hart equation
    float steinhart = resistance / NOMINAL_RESISTANCE; // R/Ro
    steinhart = log(steinhart); // ln(R/Ro)
    steinhart /= B_COEFFICIENT; // (1/B) * ln(R/Ro)
    steinhart += 1.0f / (NOMINAL_TEMPERATURE + 273.15f); // + (1/To)
    steinhart = 1.0f / steinhart; // Invert
    steinhart -= 273.15f; // Convert to Celsius

    return roundf(steinhart);
}

// *****************************************************************************
// Section: Application Functions
// *****************************************************************************
// Initialize the temperature task

void TEMPERATURE_Initialize(void) {
    // Initialize state machine
    temperatureData.state = TEMPERATURE_STATE_INIT;

    // Register ADC callback and enable interrupts for ADC channel 9
    ADCHS_CallbackRegister(ADCHS_CH9, ADCHS_CH9_Callback, (uintptr_t) NULL);
    ADCHS_ChannelResultInterruptEnable(ADCHS_CH9);
    ADCHS_ChannelConversionStart(ADCHS_CH9);

    // Create semaphore for ADC conversion synchronization
    vSemaphoreCreateBinary(ADC9_Temp_SEMAPHORE);
    xSemaphoreTake(ADC9_Temp_SEMAPHORE, 0);
}

// Temperature task state machine

void TEMPERATURE_Tasks(void) {
    switch (temperatureData.state) {
        case TEMPERATURE_STATE_INIT:
        {
            // Transition to service tasks state after initialization
            if (true) { // Initialization complete
                temperatureData.state = TEMPERATURE_STATE_SERVICE_TASKS;
            }
            break;
        }

        case TEMPERATURE_STATE_SERVICE_TASKS:
        {
            static float lastTemperature1 = 0.0f; // Last temperature reading 1
            static float lastTemperature2 = 0.0f; // Last temperature reading 2

            // Wait for ADC conversion result
            xSemaphoreTake(ADC9_Temp_SEMAPHORE, portMAX_DELAY);
            uint16_t adcValue = ADCHS_ChannelResultGet(ADCHS_CH9);

            // Calculate temperature and mean value
            float newTemperature = MeasureTemperature(adcValue);
            float meanTemperature = (lastTemperature1 + lastTemperature2 + newTemperature) / 3.0f;

            // Print and queue the mean temperature
            printf("\n\rMean Temperature = %.f C", roundf(meanTemperature));
            xQueueSend(Temperature_Queue, &meanTemperature, portMAX_DELAY);

            // Update last temperature values
            lastTemperature1 = lastTemperature2;
            lastTemperature2 = newTemperature;

            // Start next ADC conversion
            ADCHS_ChannelConversionStart(ADCHS_CH9);

            break;
        }

        default:
        {
            // Handle invalid states
            break;
        }
    }
}