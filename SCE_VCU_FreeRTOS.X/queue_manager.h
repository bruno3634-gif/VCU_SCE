/* 
 * File:   queue_manager.h
 * Author: root
 *
 * Created on December 7, 2024, 7:37 PM
 */

#ifndef QUEUE_MANAGER_H
#define QUEUE_MANAGER_H

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

// Declare the queue handle as extern
extern QueueHandle_t Inverter_control_Queue;
extern QueueHandle_t AS_Emergency_Queue;



//Declare semaphores

extern xSemaphoreHandle R2D_semaphore;
extern xSemaphoreHandle CAN_Mutex;

#endif // QUEUE_MANAGER_H