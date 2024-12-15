#include "queue_manager.h"

// Define the queue handles for inter-task communication
QueueHandle_t Inverter_control_Queue = NULL;  // Queue for inverter control messages
QueueHandle_t AS_Emergency_Queue = NULL;      // Queue for AS emergency messages
QueueHandle_t Bat_Voltage_Queue = NULL;       // Queue for battery voltage readings
QueueHandle_t Temperature_Queue = NULL;       // Queue for temperature readings

// Define the semaphore handles
SemaphoreHandle_t R2D_semaphore = NULL;  // Semaphore for Ready-to-Drive signaling
SemaphoreHandle_t CAN_Mutex = NULL;      // Mutex for CAN resource protection