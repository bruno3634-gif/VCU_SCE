
#include "queue_manager.h"

// Define the queue handle
QueueHandle_t Inverter_control_Queue = NULL;
QueueHandle_t AS_Emergency_Queue = NULL;

SemaphoreHandle_t R2D_semaphore = NULL;
SemaphoreHandle_t CAN_Mutex = NULL;