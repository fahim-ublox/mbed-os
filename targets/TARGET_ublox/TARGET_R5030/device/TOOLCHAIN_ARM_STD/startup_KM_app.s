;/**************************************************************************//**
; * @file     startup_KM_app.s
; * @brief    CMSIS Cortex-M7 Core Device Startup File for
; *           Device KM
; * @version  V3.10
; * @date     23. November 2012
; *
; * @note
; *
; ******************************************************************************/
;/* Copyright (c) 2012 ARM LIMITED
;
;   All rights reserved.
;   Redistribution and use in source and binary forms, with or without
;   modification, are permitted provided that the following conditions are met:
;   - Redistributions of source code must retain the above copyright
;     notice, this list of conditions and the following disclaimer.
;   - Redistributions in binary form must reproduce the above copyright
;     notice, this list of conditions and the following disclaimer in the
;     documentation and/or other materials provided with the distribution.
;   - Neither the name of ARM nor the names of its contributors may be used
;     to endorse or promote products derived from this software without
;     specific prior written permission.
;   *
;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
;   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;   POSSIBILITY OF SUCH DAMAGE.
;   ---------------------------------------------------------------------------*/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/

//#define PSRAM_CONFIG_BINARY //this macro needs to be defined when compiling the psram_config binary. For other applications, comment it out.

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000100

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

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
    DCD APP_CPU_APP_IRQ_CTIIRQ0_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_CTIIRQ1_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_I2C1_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_I2C2_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_I2C3_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_I2S_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_UART1_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_UART2_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_SPI1_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_SPI2_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_TIMER1_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_TIMER2_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_MDM_WDG_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_PHY_WDG_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_RF_WDG_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_SYSCTRL_ERR_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_SYSCTRL_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_DMAC_ERR_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_DMAC_TC_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_DMAC_COMB_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_SDIO_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_OSPI_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_PSRAM_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_CRYPTO_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_DVS_GLB_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_DVS_CORE_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_USB_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_USB_DMA_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_PIO_1_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_PIO_2_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_PIO_WAKEUP_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_PWRCTRL_ERR_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_PWRCTRL_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_BB_IPC_MBX_TX0_IRQHandler
    DCD APP_CPU_APP_IRQ_BB_IPC_MBX_TX1_IRQHandler
    DCD APP_CPU_APP_IRQ_BB_IPC_MBX_TX2_IRQHandler
    DCD APP_CPU_APP_IRQ_BB_IPC_MBX_RX0_IRQHandler
    DCD APP_CPU_APP_IRQ_BB_IPC_MBX_RX1_IRQHandler
    DCD APP_CPU_APP_IRQ_BB_IPC_MBX_RX2_IRQHandler
    DCD APP_CPU_APP_IRQ_GNSS_TIM_IRQHandler
    DCD APP_CPU_APP_IRQ_OTP_FAULT_IRQHandler
    DCD APP_CPU_APP_IRQ_ONFI_CAL_IRQHandler
    DCD APP_CPU_APP_IRQ_PWRCTRL_WAKEUP_IRQHandler
    DCD APP_CPU_APP_IRQ_ETR_LIMITER_THRESHOLD_IRQHandler
    DCD APP_CPU_APP_IRQ_USB_WAKEUP_INT_IRQHandler
    DCD APP_CPU_APP_IRQ_AC_PWR_ALERT_IRQHandler


__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY
                ENTRY

; Reset Handler

__reset         ; compatibility with MAT
                EXPORT  __reset             [WEAK]

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
#ifdef PSRAM_CONFIG_BINARY
                LDR 	R1, = 0x4960002C ; address width register of PSRAM. It is 0 when PSRAM in uninitialized
                LDR 	R2,[R1] ; read value of address width register
                CMP 	R2, #0 ; check if it is zero
                BEQ		Init_PSRAM ; if zero, proceed executing the program. This program initailizes the PSRAM and in the end performs a core reset
                B       . ; if non-zero, it means PSRAM is already initialized so loop here and do nothing. At this stage, we can load the second program
Init_PSRAM
#endif
      			LDR     R0, =SystemInit
                BLX     R0
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
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

APP_CPU_APP_IRQ_CTIIRQ0_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_CTIIRQ0_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_CTIIRQ1_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_CTIIRQ1_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_I2C1_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_I2C1_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_I2C2_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_I2C2_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_I2C3_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_I2C3_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_I2S_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_I2S_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_UART1_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_UART1_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_UART2_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_UART2_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_SPI1_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_SPI1_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_SPI2_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_SPI2_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_TIMER1_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_TIMER1_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_TIMER2_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_TIMER2_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_MDM_WDG_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_MDM_WDG_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_PHY_WDG_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_PHY_WDG_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_RF_WDG_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_RF_WDG_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_SYSCTRL_ERR_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_SYSCTRL_ERR_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_SYSCTRL_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_SYSCTRL_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_DMAC_ERR_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_DMAC_ERR_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_DMAC_TC_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_DMAC_TC_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_DMAC_COMB_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_DMAC_COMB_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_SDIO_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_SDIO_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_OSPI_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_OSPI_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_PSRAM_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_PSRAM_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_CRYPTO_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_CRYPTO_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_DVS_GLB_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_DVS_GLB_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_DVS_CORE_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_DVS_CORE_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_USB_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_USB_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_USB_DMA_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_USB_DMA_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_PIO_1_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_PIO_1_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_PIO_2_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_PIO_2_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_PIO_WAKEUP_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_PIO_WAKEUP_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_PWRCTRL_ERR_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_PWRCTRL_ERR_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_PWRCTRL_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_PWRCTRL_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_BB_IPC_MBX_TX0_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_BB_IPC_MBX_TX0_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_BB_IPC_MBX_TX1_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_BB_IPC_MBX_TX1_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_BB_IPC_MBX_TX2_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_BB_IPC_MBX_TX2_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_BB_IPC_MBX_RX0_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_BB_IPC_MBX_RX0_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_BB_IPC_MBX_RX1_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_BB_IPC_MBX_RX1_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_BB_IPC_MBX_RX2_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_BB_IPC_MBX_RX2_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_GNSS_TIM_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_GNSS_TIM_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_OTP_FAULT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_OTP_FAULT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_ONFI_CAL_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_ONFI_CAL_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_PWRCTRL_WAKEUP_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_PWRCTRL_WAKEUP_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_ETR_LIMITER_THRESHOLD_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_ETR_LIMITER_THRESHOLD_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_USB_WAKEUP_INT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_USB_WAKEUP_INT_IRQHandler           [WEAK]
                B       .
                ENDP
APP_CPU_APP_IRQ_AC_PWR_ALERT_IRQHandler\
                PROC
                EXPORT  APP_CPU_APP_IRQ_AC_PWR_ALERT_IRQHandler           [WEAK]
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

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
