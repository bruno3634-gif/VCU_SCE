#include "r2d_task.h"

#include "definitions.h"
#include "peripheral/adchs/plib_adchs_common.h"
#include "../SCE_VCU_FreeRTOS.X/queue_manager.h"
#include "semphr.h"
#include "stdio.h"

// Global Variables
R2D_TASK_DATA r2d_taskData;
unsigned int Time = 0;
volatile int r2d = 0;

uint32_t id = 0x14;
uint8_t length = 8;
uint8_t message[8];
float bp = 0.0f;

// Semaphores
xSemaphoreHandle ADC15_BP_SEMAPHORE;
xSemaphoreHandle R2D_BTN_SEMAPHORE;

// Function Prototypes
unsigned int millis1(void);
bool CanSend(uint32_t id, uint8_t length, uint8_t *buffer);
float MeasureBrakePressure(uint16_t bits);
void ADCHS_CH15_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context);
void r2d_int(GPIO_PIN pin, uintptr_t context);

// Get current time in milliseconds
unsigned int millis1(void) {
    return (unsigned int)(CORETIMER_CounterGet() / (CORE_TIMER_FREQUENCY / 1000));
}

// Send CAN message
bool CanSend(uint32_t id, uint8_t length, uint8_t *buffer) {
    return CAN1_MessageTransmit(id, length, buffer, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
}

// Measure brake pressure from ADC value
float MeasureBrakePressure(uint16_t bits) {
    float volts = (float)bits * 3.3f / 4095.0f;
    volts /= 0.667f;  // Convert from 3.3V to 5V scale
    float pressure = (volts - 0.5f) / 0.02857f;
    return pressure;
}

// ADC Channel 15 Callback
void ADCHS_CH15_Callback(ADCHS_CHANNEL_NUM channel, uintptr_t context) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ADCHS_ChannelResultGet(ADCHS_CH15);
    xSemaphoreGiveFromISR(ADC15_BP_SEMAPHORE, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD();
    }
}

// R2D Interrupt Handler
void r2d_int(GPIO_PIN pin, uintptr_t context) {
    GPIO_PinIntDisable(IGNITION_PIN);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(R2D_BTN_SEMAPHORE, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD();
    }
}

// Application Initialization
void R2D_TASK_Initialize(void) {
    // Initialize state machine
    r2d_taskData.state = R2D_TASK_STATE_INIT;

    // Register callbacks
    GPIO_PinInterruptCallbackRegister(IGNITION_PIN, r2d_int, 0);
    GPIO_PinIntDisable(IGNITION_PIN);
    ADCHS_CallbackRegister(ADCHS_CH15, ADCHS_CH15_Callback, (uintptr_t)NULL);
    ADCHS_ChannelResultInterruptEnable(ADCHS_CH15);
    ADCHS_ChannelConversionStart(ADCHS_CH15);

    // Initialize semaphores
    vSemaphoreCreateBinary(ADC15_BP_SEMAPHORE);
    xSemaphoreTake(ADC15_BP_SEMAPHORE, 0);
    vSemaphoreCreateBinary(R2D_BTN_SEMAPHORE);
    xSemaphoreTake(R2D_BTN_SEMAPHORE, 0);

    // Initialize message buffer
    memset(message, 0x00, sizeof(message));

    // Set initial LED state
    LED_F1_Set();
}

// Application Tasks
void R2D_TASK_Tasks(void) {
    switch (r2d_taskData.state) {
        case R2D_TASK_STATE_INIT: {
            message[0] = 0x00;
            r2d_taskData.state = R2D_TASK_STATE_SERVICE_TASKS;
            break;
        }

        case R2D_TASK_STATE_SERVICE_TASKS: {
            LED_F1_Clear();
            if (R2D_S_Get() == 1) {
                message[0] = 0x01;
                message[1] = 0x00;
                GPIO_PinIntEnable(IGNITION_PIN, GPIO_INTERRUPT_ON_RISING_EDGE);
                ADCHS_ChannelConversionStart(ADCHS_CH15);

                if (xSemaphoreTake(R2D_BTN_SEMAPHORE, pdMS_TO_TICKS(50)) == pdTRUE) {
                    ADCHS_ChannelConversionStart(ADCHS_CH15);
                    xSemaphoreTake(ADC15_BP_SEMAPHORE, portMAX_DELAY);
                    float brakePressure = MeasureBrakePressure(ADCHS_ChannelResultGet(ADCHS_CH15));
                    if (brakePressure >= 1.0f) {
                        r2d_taskData.state = R2D_TASK_BUZZING;
                    }
                    printf("\n\rBrake Pressure: %.2f\n\r", brakePressure);
                }
            } else {
                GPIO_PinIntDisable(IGNITION_PIN);
                r2d_taskData.state = R2D_TASK_STATE_INIT;
                message[1] = 0x00;
            }
            break;
        }

        case R2D_TASK_BUZZING: {
            GPIO_PinIntDisable(IGNITION_PIN);
            r2d_taskData.state = R2D_TASK_R2D_STATE;
            LED_F1_Set();
            buzzer_Clear();
            vTaskDelay(pdMS_TO_TICKS(1000));
            buzzer_Set();
            break;
        }

        case R2D_TASK_R2D_STATE: {
            GPIO_PinIntDisable(IGNITION_PIN);
            if (R2D_S_Get() == 1) {
                message[0] = 0x01;
                message[1] = 0x01;
                buzzer_Set();
                xSemaphoreGive(R2D_semaphore);
            } else {
                message[0] = 0x00;
                message[1] = 0x00;
                buzzer_Set();
                r2d_taskData.state = R2D_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        default: {
            // Handle unexpected state
            break;
        }
    }

    // Measure brake pressure and send CAN message
    ADCHS_ChannelConversionStart(ADCHS_CH15);
    xSemaphoreTake(ADC15_BP_SEMAPHORE, portMAX_DELAY);
    bp = MeasureBrakePressure(ADCHS_ChannelResultGet(ADCHS_CH15));
    int pressure = (int)(bp * 10);
    message[2] = (pressure >> 8) & 0xFF;
    message[3] = pressure & 0xFF;
    CanSend(id, length, message);
}