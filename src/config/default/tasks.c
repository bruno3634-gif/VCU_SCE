/*******************************************************************************
 System Tasks File

  File Name:
    tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled tasks.

  Description:
    This file contains source code necessary to maintain system's polled tasks.
    It implements the "SYS_Tasks" function that calls the individual "Tasks"
    functions for all polled MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "configuration.h"
#include "definitions.h"
#include "sys_tasks.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"

//QueueHandle_t Inverter_control_Queue;
// *****************************************************************************
// *****************************************************************************
// Section: RTOS "Tasks" Routine
// *****************************************************************************
// *****************************************************************************
/* Handle for the CAN_SEND_TASK_Tasks. */
TaskHandle_t xCAN_SEND_TASK_Tasks;

static void lCAN_SEND_TASK_Tasks(  void *pvParameters  )
{   
    while(true)
    {
        CAN_SEND_TASK_Tasks();
        vTaskDelay(100U / portTICK_PERIOD_MS);
    }
}
/* Handle for the APPS_TASK_Tasks. */
TaskHandle_t xAPPS_TASK_Tasks;

static void lAPPS_TASK_Tasks(  void *pvParameters  )
{   
    while(true)
    {
        APPS_TASK_Tasks();
        vTaskDelay(500U / portTICK_PERIOD_MS);
    }
}
/* Handle for the VOLTAGE_MEASUREMENT_TASK_Tasks. */
TaskHandle_t xVOLTAGE_MEASUREMENT_TASK_Tasks;

static void lVOLTAGE_MEASUREMENT_TASK_Tasks(  void *pvParameters  )
{   
    while(true)
    {
        VOLTAGE_MEASUREMENT_TASK_Tasks();
        vTaskDelay(1000U / portTICK_PERIOD_MS);
    }
}
/* Handle for the INVERTER_TASK_Tasks. */
TaskHandle_t xINVERTER_TASK_Tasks;

static void lINVERTER_TASK_Tasks(  void *pvParameters  )
{   
    while(true)
    {
        INVERTER_TASK_Tasks();
        vTaskDelay(50U / portTICK_PERIOD_MS);
    }
}
/* Handle for the R2D_TASK_Tasks. */
TaskHandle_t xR2D_TASK_Tasks;

static void lR2D_TASK_Tasks(  void *pvParameters  )
{   
    while(true)
    {
        R2D_TASK_Tasks();
        vTaskDelay(500U / portTICK_PERIOD_MS);
    }
}
/* Handle for the MAIN_TASK_Tasks. */
TaskHandle_t xMAIN_TASK_Tasks;

static void lMAIN_TASK_Tasks(  void *pvParameters  )
{   
    while(true)
    {
        MAIN_TASK_Tasks();
        vTaskDelay(500U / portTICK_PERIOD_MS);
    }
}
/* Handle for the CAN_READ_TASK_Tasks. */
TaskHandle_t xCAN_READ_TASK_Tasks;

static void lCAN_READ_TASK_Tasks(  void *pvParameters  )
{   
    while(true)
    {
        CAN_READ_TASK_Tasks();
        vTaskDelay(50U / portTICK_PERIOD_MS);
    }
}




// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/
void SYS_Tasks ( void )
{
    
    /* Maintain system services */
    

    /* Maintain Device Drivers */
    

    /* Maintain Middleware & Other Libraries */
    

    /* Maintain the application's state machine. */
        /* Create OS Thread for CAN_SEND_TASK_Tasks. */
    (void) xTaskCreate((TaskFunction_t) lCAN_SEND_TASK_Tasks,
                "CAN_SEND_TASK_Tasks",
                1024,
                NULL,
                3,
                &xCAN_SEND_TASK_Tasks);

    /* Create OS Thread for APPS_TASK_Tasks. */
    (void) xTaskCreate((TaskFunction_t) lAPPS_TASK_Tasks,
                "APPS_TASK_Tasks",
                1024,
                NULL,
                5,
                &xAPPS_TASK_Tasks);

    /* Create OS Thread for VOLTAGE_MEASUREMENT_TASK_Tasks. */
    (void) xTaskCreate((TaskFunction_t) lVOLTAGE_MEASUREMENT_TASK_Tasks,
                "VOLTAGE_MEASUREMENT_TASK_Tasks",
                1024,
                NULL,
                1,
                &xVOLTAGE_MEASUREMENT_TASK_Tasks);

    /* Create OS Thread for INVERTER_TASK_Tasks. */
    (void) xTaskCreate((TaskFunction_t) lINVERTER_TASK_Tasks,
                "INVERTER_TASK_Tasks",
                1024,
                NULL,
                3,
                &xINVERTER_TASK_Tasks);

    /* Create OS Thread for R2D_TASK_Tasks. */
    (void) xTaskCreate((TaskFunction_t) lR2D_TASK_Tasks,
                "R2D_TASK_Tasks",
                1024,
                NULL,
                3,
                &xR2D_TASK_Tasks);

    /* Create OS Thread for MAIN_TASK_Tasks. */
    (void) xTaskCreate((TaskFunction_t) lMAIN_TASK_Tasks,
                "MAIN_TASK_Tasks",
                1024,
                NULL,
                -2,
                &xMAIN_TASK_Tasks);

    /* Create OS Thread for CAN_READ_TASK_Tasks. */
    (void) xTaskCreate((TaskFunction_t) lCAN_READ_TASK_Tasks,
                "CAN_READ_TASK_Tasks",
                1024,
                NULL,
                5,
                &xCAN_READ_TASK_Tasks);




    /* Start RTOS Scheduler. */
    
     /**********************************************************************
     * Create all Threads for APP Tasks before starting FreeRTOS Scheduler *
     ***********************************************************************/
    vTaskStartScheduler(); /* This function never returns. */

}

/*******************************************************************************
 End of File
 */

