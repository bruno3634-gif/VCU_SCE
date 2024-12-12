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



typedef struct {
    uint32_t id;         // CAN ID
    uint8_t data[8];     // CAN data (up to 8 bytes)
    uint8_t dlc;         // Data length code
} CANMessage;

// Declare the queue handle as extern
extern QueueHandle_t Inverter_control_Queue;
extern QueueHandle_t AS_Emergency_Queue;



//Declare semaphores

extern xSemaphoreHandle R2D_semaphore;
extern xSemaphoreHandle CAN_Mutex;

#endif // QUEUE_MANAGER_H