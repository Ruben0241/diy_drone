/*
 * FreeRTOSConfig.h - Einstellungen des Echtzeitbetriebssystems
 */
#pragma once

#include <stdint.h>
extern uint32_t SystemCoreClock;

#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configCPU_CLOCK_HZ                      (216000000UL)
#define configTICK_RATE_HZ                      (1000)
#define configMAX_PRIORITIES                    (6)
#define configMINIMAL_STACK_SIZE                (256)
#define configMAX_TASK_NAME_LEN                 (12)
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_MUTEXES                       1
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_MALLOC_FAILED_HOOK            1
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_TIMERS                        0
#define configUSE_CO_ROUTINES                   0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configTOTAL_HEAP_SIZE                   (48 * 1024)
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configENABLE_FPU                        1

#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetTickCount               1
#define INCLUDE_uxTaskGetStackHighWaterMark     1

/* Cortex-M7: 4 Prioritaetsbits. Interrupts mit Prioritaet >= 5 duerfen
 * FreeRTOS-Funktionen (...FromISR) aufrufen. */
#define configPRIO_BITS                                4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY        15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY   5
#define configKERNEL_INTERRUPT_PRIORITY      (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Bei einem Fehler: Motoren aus und anhalten (siehe main.c) */
void fc_fatal_error(const char *file, int line);
#define configASSERT(x) do { if (!(x)) fc_fatal_error(__FILE__, __LINE__); } while (0)

/* Interrupt-Handler des Ports auf die CMSIS-Namen abbilden */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler
