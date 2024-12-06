/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    apps_task.c

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

#include <stdio.h>
#include "apps_task.h"
#include "inverter_task.h"
#include "peripheral/adchs/plib_adchs.h"
#ifndef FREERTOS_H
#include"FreeRTOS.h"
#endif
#include "semphr.h"
#include "definitions.h"
#include "queue.h"



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
    This structure should be initialized by the APPS_TASK_Initialize function.

    Application strings and buffers are be defined outside this structure.
 */

APPS_TASK_DATA apps_taskData;

static SemaphoreHandle_t ADC0_SEMAPHORE;
static SemaphoreHandle_t ADC3_SEMAPHORE;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */



void ADC0_callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
     
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //printf("\n\n\rADC0 int\n\n\r");
    ADCHS_ChannelResultGet(ADCHS_CH0);
    xSemaphoreGiveFromISR(ADC0_SEMAPHORE, &xHigherPriorityTaskWoken);

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);


    //taskYIELD();
}

void ADC3_callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // printf("\n\n\rADC3 int\n\n\r");
    ADCHS_ChannelResultGet(ADCHS_CH3);
    //IFS3CLR = _IFS3_AD1D3IF_MASK;
    //IFS3bits.AD1D3IF = 0;
    xSemaphoreGiveFromISR(ADC3_SEMAPHORE, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    //taskYIELD();
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
    void APPS_TASK_Initialize ( void )

  Remarks:
    See prototype in apps_task.h.
 */

void APPS_TASK_Initialize(void) {
    /* Place the App state machine in its initial state. */
    apps_taskData.state = APPS_TASK_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */

    ADCHS_CallbackRegister(ADCHS_CH0, ADC0_callback, (uintptr_t) NULL);
    ADCHS_CallbackRegister(ADCHS_CH3, ADC3_callback, (uintptr_t) NULL);

    vSemaphoreCreateBinary(ADC0_SEMAPHORE);
    xSemaphoreTake(ADC0_SEMAPHORE, portMAX_DELAY);
    
        vSemaphoreCreateBinary(ADC3_SEMAPHORE);
    xSemaphoreTake(ADC3_SEMAPHORE, portMAX_DELAY);
}

/******************************************************************************
  Function:
    void APPS_TASK_Tasks ( void )

  Remarks:
    See prototype in apps_task.h.
 */

void APPS_TASK_Tasks(void) {

    /* Check the application's current state. */
    switch (apps_taskData.state) {
            /* Application's initial state. */
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
            ADCHS_ChannelConversionStart(ADCHS_CH3);
            ADCHS_ChannelConversionStart(ADCHS_CH0);
            
            if (xSemaphoreTake(ADC0_SEMAPHORE, portMAX_DELAY) == pdTRUE) {
                // Task unblocks here when semaphore is given
                
            }
            
            if (xSemaphoreTake(ADC3_SEMAPHORE, portMAX_DELAY) == pdTRUE) {
                //Task unblocks here when semaphore is given

                
            }


            

            //        printf("\n\n\r Ended conversion\n\n\r");

            uint16_t adc0value = ADCHS_ChannelResultGet(ADCHS_CH0);
            uint16_t adc3value = ADCHS_ChannelResultGet(ADCHS_CH3);

            float voltage0 = adc0value * 3.3 / 4096;
            float voltage3 = adc3value * 3.3 / 4096;
            voltage0 = voltage0 + 0;
            voltage3 = voltage3 + 0;
            printf("APPS 1:%f,APPS 3:%f\r\n", voltage0, voltage3);
//            uint8_t message_to_queue = voltage0 * 100/3.3;
            //LED_F1_Toggle();
            LED_RB10_Toggle();
            // printf("\n\rAPPS\n\r");
 //           xQueueSend(Inverter_control_Queue,&message_to_queue,portMAX_DELAY);
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
