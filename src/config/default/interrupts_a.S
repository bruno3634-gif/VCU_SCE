/*******************************************************************************
  System Interrupts File

  Company:
    Microchip Technology Inc.

  File Name:
    interrupt_a.S

  Summary:
    Raw ISR definitions.

  Description:
    This file maps all the interrupt vectors to their corresponding
    implementations. If a particular module interrupt is used, then its ISR
    definition can be found in corresponding PLIB source file. If a module
    interrupt is not used, then its ISR implementation is mapped to dummy
    handler.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
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

#include <xc.h>
#include "ISR_Support.h"

    .extern  EXTERNAL_3_Handler

    .section   .vector_18,code, keep
    .equ     __vector_dispatch_18, IntVectorEXTERNAL_3_Handler
    .global  __vector_dispatch_18
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorEXTERNAL_3_Handler

IntVectorEXTERNAL_3_Handler:
    portSAVE_CONTEXT
    la    s6,  EXTERNAL_3_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorEXTERNAL_3_Handler
    .extern  ADC_EOS_Handler

    .section   .vector_101,code, keep
    .equ     __vector_dispatch_101, IntVectorADC_EOS_Handler
    .global  __vector_dispatch_101
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorADC_EOS_Handler

IntVectorADC_EOS_Handler:
    portSAVE_CONTEXT
    la    s6,  ADC_EOS_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorADC_EOS_Handler
    .extern  ADC_DATA0_Handler

    .section   .vector_106,code, keep
    .equ     __vector_dispatch_106, IntVectorADC_DATA0_Handler
    .global  __vector_dispatch_106
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorADC_DATA0_Handler

IntVectorADC_DATA0_Handler:
    portSAVE_CONTEXT
    la    s6,  ADC_DATA0_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorADC_DATA0_Handler
    .extern  ADC_DATA3_Handler

    .section   .vector_109,code, keep
    .equ     __vector_dispatch_109, IntVectorADC_DATA3_Handler
    .global  __vector_dispatch_109
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorADC_DATA3_Handler

IntVectorADC_DATA3_Handler:
    portSAVE_CONTEXT
    la    s6,  ADC_DATA3_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorADC_DATA3_Handler
    .extern  ADC_DATA8_Handler

    .section   .vector_114,code, keep
    .equ     __vector_dispatch_114, IntVectorADC_DATA8_Handler
    .global  __vector_dispatch_114
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorADC_DATA8_Handler

IntVectorADC_DATA8_Handler:
    portSAVE_CONTEXT
    la    s6,  ADC_DATA8_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorADC_DATA8_Handler
    .extern  ADC_DATA15_Handler

    .section   .vector_121,code, keep
    .equ     __vector_dispatch_121, IntVectorADC_DATA15_Handler
    .global  __vector_dispatch_121
    .set     nomicromips
    .set     noreorder
    .set     nomips16
    .set     noat
    .ent  IntVectorADC_DATA15_Handler

IntVectorADC_DATA15_Handler:
    portSAVE_CONTEXT
    la    s6,  ADC_DATA15_Handler
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end   IntVectorADC_DATA15_Handler

