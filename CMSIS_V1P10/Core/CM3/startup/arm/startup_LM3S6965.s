;/*****************************************************************************
; * @file:    startup_LM3S.s
; * @purpose: CMSIS Cortex-M3 Core Device Startup File for the
; *           Luminary 'LM3S' Device Series 
; * @version: V0.01 (Preliminary)
; * @date:    15. Jan. 2009
; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------
; *
; * Copyright (C) 2009 ARM Limited. All rights reserved.
; * ARM Limited (ARM) is supplying this software for use with Cortex-Mx 
; * processor based microcontrollers.  This file can be freely distributed 
; * within development tools that are supporting such ARM based processors. 
; *
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; *****************************************************************************/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000200

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     GPIOPortA_IRQHandler      ;  0: GPIO Port A
                DCD     GPIOPortB_IRQHandler      ;  1: GPIO Port B
                DCD     GPIOPortC_IRQHandler      ;  2: GPIO Port C
                DCD     GPIOPortD_IRQHandler      ;  3: GPIO Port D
                DCD     GPIOPortE_IRQHandler      ;  4: GPIO Port E
                DCD     UART0_IRQHandler          ;  5: UART0
                DCD     UART1_IRQHandler          ;  6: UART1
                DCD     SSI0_IRQHandler           ;  7: SSI0
                DCD     I2C0_IRQHandler           ;  8: I2C0
                DCD     PWMFault_IRQHandler       ;  9: PWM Fault
                DCD     PWMGen0_IRQHandler        ; 10: PWM Generator 0
                DCD     PWMGen1_IRQHandler        ; 11: PWM Generator 1
                DCD     PWMGen2_IRQHandler        ; 12: PWM Generator 2
                DCD     QEI0_IRQHandler           ; 13: Quadrature Encoder
                DCD     ADCSeq0_IRQHandler        ; 14: ADC Sequence 0
                DCD     ADCSeq1_IRQHandler        ; 15: ADC Sequence 1
                DCD     ADCSeq2_IRQHandler        ; 16: ADC Sequence 2
                DCD     ADCSeq3_IRQHandler        ; 17: ADC Sequence 3
                DCD     Watchdog_IRQHandler       ; 18: Watchdog
                DCD     Timer0A_IRQHandler        ; 19: Timer 0A
                DCD     Timer0B_IRQHandler        ; 20: Timer 0B
                DCD     Timer1A_IRQHandler        ; 21: Timer 1A
                DCD     Timer1B_IRQHandler        ; 22: Timer 1B
                DCD     Timer2A_IRQHandler        ; 23: Timer 2A
                DCD     Timer2B_IRQHandler        ; 24: Timer 2B
                DCD     Comp0_IRQHandler          ; 25: Comp 0
                DCD     Comp1_IRQHandler          ; 26: Comp 1
                DCD     Comp2_IRQHandler          ; 27: Comp 2
                DCD     SysCtrl_IRQHandler        ; 28: System Control
                DCD     FlashCtrl_IRQHandler      ; 29: Flash Control
                DCD     GPIOPortF_IRQHandler      ; 30: GPIO Port F
                DCD     GPIOPortG_IRQHandler      ; 31: GPIO Port G
                DCD     GPIOPortH_IRQHandler      ; 32: GPIO Port H
                DCD     USART2_IRQHandler         ; 33: UART2 Rx and Tx
                DCD     SSI1_IRQHandler           ; 34: SSI1 Rx and Tx
                DCD     Timer3A_IRQHandler        ; 35: Timer 3 subtimer A
                DCD     Timer3B_IRQHandler        ; 36: Timer 3 subtimer B
                DCD     I2C1_IRQHandler           ; 37: I2C1 Master and Slave
                DCD     QEI1_IRQHandler           ; 38: Quadrature Encoder 1
                DCD     CAN0_IRQHandler           ; 39: CAN0
                DCD     CAN1_IRQHandler           ; 40: CAN1
                DCD     CAN2_IRQHandler           ; 41: CAN2
                DCD     Ethernet_IRQHandler       ; 42: Ethernet
                DCD     Hibernate_IRQHandler      ; 43:Hibernate
                DCD     USB0_IRQHandler           ; 44: USB0
                DCD     PWMGen3_IRQHandler        ; 45: PWM Generator 3
                DCD     uDMA_IRQHandler           ; 46: uDMA Software Transfer
                DCD     uDMAErr_IRQHandler        ; 47: uDMA Error


                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  __main
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  INT0_IRQHandler           [WEAK]
                EXPORT  GPIOPortA_IRQHandler      [WEAK]
                EXPORT  GPIOPortB_IRQHandler      [WEAK]
                EXPORT  GPIOPortC_IRQHandler      [WEAK]
                EXPORT  GPIOPortD_IRQHandler      [WEAK]
                EXPORT  GPIOPortE_IRQHandler      [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  SSI0_IRQHandler           [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  PWMFault_IRQHandler       [WEAK]
                EXPORT  PWMGen0_IRQHandler        [WEAK]
                EXPORT  PWMGen1_IRQHandler        [WEAK]
                EXPORT  PWMGen2_IRQHandler        [WEAK]
                EXPORT  QEI0_IRQHandler           [WEAK]
                EXPORT  ADCSeq0_IRQHandler        [WEAK]
                EXPORT  ADCSeq1_IRQHandler        [WEAK]
                EXPORT  ADCSeq2_IRQHandler        [WEAK]
                EXPORT  ADCSeq3_IRQHandler        [WEAK]
                EXPORT  Watchdog_IRQHandler       [WEAK]
                EXPORT  Timer0A_IRQHandler        [WEAK]
                EXPORT  Timer0B_IRQHandler        [WEAK]
                EXPORT  Timer1A_IRQHandler        [WEAK]
                EXPORT  Timer1B_IRQHandler        [WEAK]
                EXPORT  Timer2A_IRQHandler        [WEAK]
                EXPORT  Timer2B_IRQHandler        [WEAK]
                EXPORT  Comp0_IRQHandler          [WEAK]
                EXPORT  Comp1_IRQHandler          [WEAK]
                EXPORT  Comp2_IRQHandler          [WEAK]
                EXPORT  SysCtrl_IRQHandler        [WEAK]
                EXPORT  FlashCtrl_IRQHandler      [WEAK]
                EXPORT  GPIOPortF_IRQHandler      [WEAK]
                EXPORT  GPIOPortG_IRQHandler      [WEAK]
                EXPORT  GPIOPortH_IRQHandler      [WEAK]
                EXPORT  USART2_IRQHandler         [WEAK]
                EXPORT  SSI1_IRQHandler           [WEAK]
                EXPORT  Timer3A_IRQHandler        [WEAK]
                EXPORT  Timer3B_IRQHandler        [WEAK]
                EXPORT  I2C1_IRQHandler           [WEAK]
                EXPORT  QEI1_IRQHandler           [WEAK]
                EXPORT  CAN0_IRQHandler           [WEAK]
                EXPORT  CAN1_IRQHandler           [WEAK]
                EXPORT  CAN2_IRQHandler           [WEAK]
                EXPORT  Ethernet_IRQHandler       [WEAK]
                EXPORT  Hibernate_IRQHandler      [WEAK]
                EXPORT  USB0_IRQHandler           [WEAK]
                EXPORT  PWMGen3_IRQHandler        [WEAK]
                EXPORT  uDMA_IRQHandler           [WEAK]
                EXPORT  uDMAErr_IRQHandler        [WEAK]

INT0_IRQHandler
GPIOPortA_IRQHandler
GPIOPortB_IRQHandler
GPIOPortC_IRQHandler
GPIOPortD_IRQHandler
GPIOPortE_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
SSI0_IRQHandler
I2C0_IRQHandler
PWMFault_IRQHandler
PWMGen0_IRQHandler
PWMGen1_IRQHandler
PWMGen2_IRQHandler
QEI0_IRQHandler
ADCSeq0_IRQHandler
ADCSeq1_IRQHandler
ADCSeq2_IRQHandler
ADCSeq3_IRQHandler
Watchdog_IRQHandler
Timer0A_IRQHandler
Timer0B_IRQHandler
Timer1A_IRQHandler
Timer1B_IRQHandler
Timer2A_IRQHandler
Timer2B_IRQHandler
Comp0_IRQHandler
Comp1_IRQHandler
Comp2_IRQHandler
SysCtrl_IRQHandler
FlashCtrl_IRQHandler
GPIOPortF_IRQHandler
GPIOPortG_IRQHandler
GPIOPortH_IRQHandler
USART2_IRQHandler
SSI1_IRQHandler
Timer3A_IRQHandler
Timer3B_IRQHandler
I2C1_IRQHandler
QEI1_IRQHandler
CAN0_IRQHandler
CAN1_IRQHandler
CAN2_IRQHandler
Ethernet_IRQHandler
Hibernate_IRQHandler
USB0_IRQHandler
PWMGen3_IRQHandler
uDMA_IRQHandler
uDMAErr_IRQHandler

                B       .

                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB
                
                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit
                
                ELSE
                
                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF


                END
