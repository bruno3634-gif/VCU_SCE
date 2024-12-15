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

// Structure to represent a CAN message
typedef struct {
    uint32_t id;      // CAN ID
    uint8_t data[8];  // CAN data (up to 8 bytes)
    uint8_t dlc;      // Data length code
} CANMessage;

// Declare the queue handles as extern for inter-task communication
extern QueueHandle_t Inverter_control_Queue;  // Queue for inverter control messages
extern QueueHandle_t AS_Emergency_Queue;      // Queue for AS emergency messages
extern QueueHandle_t Bat_Voltage_Queue;       // Queue for battery voltage readings
extern QueueHandle_t Temperature_Queue;       // Queue for temperature readings

// Declare semaphore handles as extern
extern xSemaphoreHandle R2D_semaphore;  // Semaphore for Ready-to-Drive signaling
extern xSemaphoreHandle CAN_Mutex;      // Mutex for CAN resource protection

#endif  // QUEUE_MANAGER_H