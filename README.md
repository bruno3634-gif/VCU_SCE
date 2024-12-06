# SCE_VCU_FreeRTOS

SCE_VCU_FreeRTOS is a project that implements a real-time operating system (RTOS) for the Vehicle Control Unit (VCU) of a Formula Student car. The system is designed to run on a **PIC32MK1024MCM064** microcontroller and leverages **FreeRTOS** for managing multiple tasks in real time. The project handles key functionalities like CAN communication, voltage measurements, and the control of various subsystems such as the inverter, dashboard, and I2C display.

---

## Features

- **Real-Time Task Management**: Utilizes FreeRTOS to handle input, output, and processing tasks efficiently.
- **CAN Bus Communication**: Reads and processes CAN messages for real-time data exchange.
- **Voltage Measurement**: Monitors and processes voltage levels using ADCs.
- **APPS Monitoring**: Reads and processes the accelerator pedal position sensor (APPS) values.
- **Subsystem Control**:
  - Inverter
  - Dashboard
  - I2C Display
  - R2D (Ready-to-Drive) system
  - AS Emergency system

---

## System Overview

The system architecture is depicted in the diagram below:
![vcu-Queues drawio](https://github.com/user-attachments/assets/58bbf85e-b0a2-4867-b6c7-dd76e5be1465)
- **Input Tasks**:
  - Reads data from CAN bus.
  - Starts ADC conversions and processes APPS data.
  - Measures voltage levels.

- **Main Task**:
  - Central processing unit that receives data from input tasks via FreeRTOS queues.
  - Processes data and sends control signals to subsystems.

- **Output Tasks**:
  - Controls various car subsystems such as the inverter, dashboard, and emergency systems.

Refer to the **Figure 3: System Overview** in the project documentation for a visual representation.

---

## Software Architecture

- **FreeRTOS**: The real-time operating system used to manage tasks, semaphores, and queues.
- **Queues**: Used to communicate between input tasks and the main brain task.
- **Semaphores**: Ensure synchronization between tasks, especially for ADC conversions.

---

## Getting Started

### Prerequisites

1. **Hardware**:
   - PIC32MK1024MCM064 microcontroller.
   - Formula Student VCU hardware setup.
2. **Software**:
   - MPLAB X IDE.
   - FreeRTOS library.
   - XC32 Compiler.

