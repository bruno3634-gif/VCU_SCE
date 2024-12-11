/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    r2d_task.c

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

#include "r2d_task.h"
#include "definitions.h"
#include "peripheral/adchs/plib_adchs_common.h"
#include "semphr.h"
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
    This structure should be initialized by the R2D_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

R2D_TASK_DATA r2d_taskData;

unsigned int Time = 0;
volatile int r2d = 0;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/

xSemaphoreHandle ADC15_BP_SEMAPHORE; 
xSemaphoreHandle R2D_BTN_SEMAPHORE; 

unsigned int millis1(void){
  return (unsigned int)(CORETIMER_CounterGet() / (CORE_TIMER_FREQUENCY / 1000));
}


void SOUND_R2DS(void);

void SOUND_R2DS(void) {
    /*if(r2d == 1){
        if(buzzer_Get() == 0){
            buzzer_Set();
        }
        else{
            if(Time + 1000 < millis1()){
                buzzer_Clear();
                r2d_taskData.state = R2D_TASK_R2D_STATE;
            }
        }
    }else{
        Time = millis1();
        buzzer_Clear();
    }*/buzzer_Clear();
    
   // LED_F1_Toggle();
}

/// @brief Measure the brake pressure from an ADC channel
/// @param bits ADC channel to measure the brake pressure
/// @return Measured brake pressure
float MeasureBrakePressure(uint16_t bits) {
    /*(28.57mV/bar  + 500mv)*/
    float volts = 0;
    float pressure = 0;
    volts = (float)bits * 3.300 / 4095.000;
    volts = volts / 0.667;  // conversion from 3.3V to 5V

    pressure = (volts - 0.5) / 0.02857;

    return pressure;
}

void ADCHS_CH15_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) { 
    static BaseType_t xHigherPriorityTaskWoken; 
    xHigherPriorityTaskWoken = pdFALSE; 
    
    ADCHS_ChannelResultGet(ADCHS_CH15); 
    
    
    xSemaphoreGiveFromISR(ADC15_BP_SEMAPHORE, &xHigherPriorityTaskWoken); 
    
    if (xHigherPriorityTaskWoken == pdTRUE) { 
        portYIELD(); 
    } 
} 



void r2d_int(GPIO_PIN pin, uintptr_t context){
    static BaseType_t xHigherPriorityTaskWoken; 
    xHigherPriorityTaskWoken = pdFALSE; 
    
    //LED_F1_Toggle(); 
    xSemaphoreGiveFromISR(R2D_BTN_SEMAPHORE, &xHigherPriorityTaskWoken); 
    
    if (xHigherPriorityTaskWoken == pdTRUE) { 
        portYIELD(); 
    } 
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void R2D_TASK_Initialize ( void )

  Remarks:
    See prototype in r2d_task.h.
 */

void R2D_TASK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    r2d_taskData.state = R2D_TASK_STATE_INIT;

    GPIO_PinInterruptCallbackRegister(IGNITION_PIN,r2d_int,0);
    GPIO_PinIntDisable(IGNITION_PIN);
    LED_F1_Set();
    
    ADCHS_CallbackRegister(ADCHS_CH15, ADCHS_CH15_Callback, (uintptr_t)NULL);  // Voltage Measurement 
    ADCHS_ChannelResultInterruptEnable(ADCHS_CH15); 
    ADCHS_ChannelConversionStart(ADCHS_CH15);   
    vSemaphoreCreateBinary(ADC15_BP_SEMAPHORE); 
    xSemaphoreTake(ADC15_BP_SEMAPHORE, 0); 
    vSemaphoreCreateBinary(R2D_BTN_SEMAPHORE);
    xSemaphoreTake(R2D_BTN_SEMAPHORE,0);
 

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void R2D_TASK_Tasks ( void )

  Remarks:
    See prototype in r2d_task.h.
 */

void R2D_TASK_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( r2d_taskData.state )
    {
        /* Application's initial state. */
        case R2D_TASK_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized)
            {

                r2d_taskData.state = R2D_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case R2D_TASK_STATE_SERVICE_TASKS:
        {
            LED_F1_Clear();
            if(R2D_S_Get()==1){
                GPIO_PinIntEnable(IGNITION_PIN, GPIO_INTERRUPT_ON_RISING_EDGE);
                ADCHS_ChannelConversionStart(ADCHS_CH15);
            
                if(xSemaphoreTake(R2D_BTN_SEMAPHORE,pdMS_TO_TICKS(50)) == pdTRUE){
                    ADCHS_ChannelConversionStart(ADCHS_CH15);
                    xSemaphoreTake(ADC15_BP_SEMAPHORE, portMAX_DELAY);
                    if(MeasureBrakePressure(ADCHS_ChannelResultGet(ADCHS_CH15)) >= -1){
                        r2d_taskData.state = R2D_TASK_BUZZING;
                    }

                    printf("\n\n\rbp: %f\n\r", MeasureBrakePressure(ADCHS_ChannelResultGet(ADCHS_CH15)));
                    GPIO_PinIntDisable(IGNITION_PIN);
                    }
                }

            else{
                GPIO_PinIntDisable(IGNITION_PIN);
                r2d_taskData.state = R2D_TASK_STATE_SERVICE_TASKS;
                //change states
            }
            break;
        }
        case R2D_TASK_BUZZING:
            GPIO_PinIntDisable(IGNITION_PIN);
            SOUND_R2DS();
            r2d_taskData.state = R2D_TASK_R2D_STATE;
            break;
        case R2D_TASK_R2D_STATE:
            GPIO_PinIntDisable(IGNITION_PIN);
            if(R2D_S_Get() == 1){
                LED_F1_Set();
                buzzer_Set();
            }
            else
            {
                buzzer_Set();
                r2d_taskData.state = R2D_TASK_STATE_SERVICE_TASKS;
            }
            break;
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
