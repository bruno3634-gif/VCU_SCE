/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    voltage_measurement_task.c

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

#include "voltage_measurement_task.h"
#include "definitions.h" 
#include "toolchain_specifics.h"

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
    This structure should be initialized by the VOLTAGE_MEASUREMENT_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

VOLTAGE_MEASUREMENT_TASK_DATA voltage_measurement_taskData;

 
xSemaphoreHandle voltageMeasurementSemaphore; 
 
__COHERENT uint16_t voltageMeasurementValue; 

float MeasureVoltage(uint16_t);

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 * 
*/


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
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


// TODO:  Add any necessary local functions.
unsigned int millis(void);

unsigned int millis(void){
  return (unsigned int)(CORETIMER_CounterGet() / (CORE_TIMER_FREQUENCY / 1000));
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void VOLTAGE_MEASUREMENT_TASK_Initialize ( void )

  Remarks:
    See prototype in voltage_measurement_task.h.
 */

void VOLTAGE_MEASUREMENT_TASK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    voltage_measurement_taskData.state = VOLTAGE_MEASUREMENT_TASK_STATE_INIT;
    
    ADCHS_CallbackRegister(ADCHS_CH8, ADCHS_CH8_Callback, (uintptr_t)NULL);  // Voltage Measurement 
    ADCHS_ChannelResultInterruptEnable(ADCHS_CH8); 
    ADCHS_ChannelConversionStart(ADCHS_CH8); 
 
    vSemaphoreCreateBinary(voltageMeasurementSemaphore); 
    xSemaphoreTake(voltageMeasurementSemaphore, 0); 



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void VOLTAGE_MEASUREMENT_TASK_Tasks ( void )

  Remarks:
    See prototype in voltage_measurement_task.h.
 */

void VOLTAGE_MEASUREMENT_TASK_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( voltage_measurement_taskData.state )
    {
        /* Application's initial state. */
        case VOLTAGE_MEASUREMENT_TASK_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized)
            {

                voltage_measurement_taskData.state = VOLTAGE_MEASUREMENT_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case VOLTAGE_MEASUREMENT_TASK_STATE_SERVICE_TASKS:
        {
            xSemaphoreTake(voltageMeasurementSemaphore, portMAX_DELAY); 
            voltageMeasurementValue = MeasureVoltage(ADCHS_ChannelResultGet(ADCHS_CH8)); 
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

/// @brief Measure the voltage from a ADC channel
/// @param channel  ADC channel to measure the voltage
float MeasureVoltage(uint16_t bits) {
    float PDM_Voltage;
//    float LV_SOC;
    PDM_Voltage = ((float)bits * 3.30 / 4095.000) / 0.1155;
    //24.0 = 0% and 28.0 = 100%
    //LV_SOC = (uint16_t)((PDM_Voltage - 24.0) * 1000 / 4.0);
    printf("\n\rPDM VALUE = %f",PDM_Voltage);
    if (PDM_Voltage >= 25) {
        // set pin
        GPIO_RG9_LV_ON_Set();
    } else if (PDM_Voltage < 25) {
        static uint16_t previousMillis = 0;
        uint16_t currentMillis = 0;
        uint16_t interval = 0;

        currentMillis = millis();
        interval = currentMillis - previousMillis;

        if (PDM_Voltage < 25 && PDM_Voltage >= 24) {
            if (interval >= 1000) {
                GPIO_RG9_LV_ON_Toggle();
                previousMillis = currentMillis;
            }
        } else if (PDM_Voltage < 24 && PDM_Voltage >= 23) {
            if (interval >= 500) {
                GPIO_RG9_LV_ON_Toggle();
                previousMillis = currentMillis;
            }
        } else if (PDM_Voltage < 23) {
            if (interval >= 150) {
                GPIO_RG9_LV_ON_Toggle();
                previousMillis = currentMillis;
            }
        }
    }

    return PDM_Voltage;
}


/*******************************************************************************
 End of File
 */
