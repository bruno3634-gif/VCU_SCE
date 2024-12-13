/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    temperature.c

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

#include "temperature.h"

#include "definitions.h"

#include <math.h>
#include "queue.h"
#include "semphr.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
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
    This structure should be initialized by the TEMPERATURE_Initialize function.

    Application strings and buffers are be defined outside this structure.
 */

TEMPERATURE_DATA temperatureData;

SemaphoreHandle_t ADC9_Temp_SEMAPHORE;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 *
 */

void ADCHS_CH9_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    ADCHS_ChannelResultGet(ADCHS_CH9);

    xSemaphoreGiveFromISR(ADC9_Temp_SEMAPHORE, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD();
    }
}

float MeasureTemperature(uint16_t bits) {
    const float SERIES_RESISTOR = 10000.0f;       // 10k? resistor
    const float NOMINAL_RESISTANCE = 10000.0f;    // NTC 10k? at 25°C
    const float NOMINAL_TEMPERATURE = 25.0f;      // 25°C
    const float B_COEFFICIENT = 3976.0f;          // B coefficient
    const float ADC_MAX = 4095.0f;                // 12-bit ADC
    const float V_REF = 3.3f;                     // Reference voltage

    // Prevent division by zero
    if (bits == 0) {
        return -273.0f; // Return absolute zero (rounded) for invalid ADC value
    }

    // Calculate voltage from ADC bits
    float voltage = (float)bits * V_REF / ADC_MAX;
    
    // Calculate thermistor resistance
    float resistance = SERIES_RESISTOR * (V_REF / voltage - 1.0f);

    // Apply the Steinhart-Hart equation
    float steinhart = resistance / NOMINAL_RESISTANCE;     // R/Ro
    steinhart = log(steinhart);                            // ln(R/Ro)
    steinhart /= B_COEFFICIENT;                            // (1/B) * ln(R/Ro)
    steinhart += 1.0f / (NOMINAL_TEMPERATURE + 273.15f);   // + (1/To)
    steinhart = 1.0f / steinhart;                         // Invert
    steinhart -= 273.15f;                                 // Convert to Celsius

    // Round to the nearest whole number
    return roundf(steinhart);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void TEMPERATURE_Initialize ( void )

  Remarks:
    See prototype in temperature.h.
 */

void TEMPERATURE_Initialize(void) {
    /* Place the App state machine in its initial state. */
    temperatureData.state = TEMPERATURE_STATE_INIT;

    ADCHS_CallbackRegister(ADCHS_CH9, ADCHS_CH9_Callback, (uintptr_t) NULL); // Voltage Measurement 
    ADCHS_ChannelResultInterruptEnable(ADCHS_CH9);
    ADCHS_ChannelConversionStart(ADCHS_CH9);

    vSemaphoreCreateBinary(ADC9_Temp_SEMAPHORE);
    xSemaphoreTake(ADC9_Temp_SEMAPHORE, 0);

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

/******************************************************************************
  Function:
    void TEMPERATURE_Tasks ( void )

  Remarks:
    See prototype in temperature.h.
 */

void TEMPERATURE_Tasks(void) {
    /* Check the application's current state. */
    switch (temperatureData.state) {
            /* Application's initial state. */
        case TEMPERATURE_STATE_INIT:
        {
            bool appInitialized = true;

            if (appInitialized) {
                temperatureData.state = TEMPERATURE_STATE_SERVICE_TASKS;
            }
            break;
        }

        case TEMPERATURE_STATE_SERVICE_TASKS:
        {
            static float lastTemperature1 = 0.0;
            static float lastTemperature2 = 0.0;

            xSemaphoreTake(ADC9_Temp_SEMAPHORE, portMAX_DELAY);
            uint16_t adcValue = ADCHS_ChannelResultGet(ADCHS_CH9);

            float newTemperature = MeasureTemperature(adcValue);

            // Calculate the mean temperature with the last two values and the new one
            float meanTemperature = (lastTemperature1 + lastTemperature2 + newTemperature) / 3.0;
            printf("\n\rMean Temperature = %.f C", roundf(meanTemperature));
            xQueueSend(Temperature_Queue,&meanTemperature,portMAX_DELAY);
            //xQueueOverwrite(Temperature_Queue,&meanTemperature);
            // Update the last temperature values
            lastTemperature1 = lastTemperature2;
            lastTemperature2 = newTemperature;

            // Start the next conversion
            ADCHS_ChannelConversionStart(ADCHS_CH9);

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
