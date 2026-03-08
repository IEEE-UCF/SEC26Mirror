/**
 * @file Teensy4FreeRTOSConfig.h
 * @brief FreeRTOS configuration overrides for SEC26 robot (Teensy 4.1).
 *
 * The FreeRTOS-Teensy4 library's FreeRTOSConfig.h uses __has_include() to
 * find this file.  library.json sets includeDir to ../../include/ which
 * resolves to mcu_ws/include/, so placing this file here is sufficient.
 *
 * This file REPLACES FreeRTOSConfig_default.h entirely when found.
 */

#ifndef TEENSY4_FREERTOS_CONFIG_H
#define TEENSY4_FREERTOS_CONFIG_H

#include <Arduino.h>

#define F_CPU 600000000

/* --- Scheduler -----------------------------------------------------------*/
#define configUSE_PREEMPTION                    1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      (F_CPU)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    ( 10 )
#define configMINIMAL_STACK_SIZE                ((unsigned short)90)
#define configMINIMAL_SECURE_STACK_SIZE         ((unsigned short)120)
#define configMAX_TASK_NAME_LEN                 20
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_ALTERNATIVE_API               0
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  1   /* round-robin for same-priority tasks */
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5
#define configUSE_APPLICATION_TASK_TAG          0

/* --- Memory allocation ---------------------------------------------------*/
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ((size_t)(300000))  /* 21 tasks */
#define configAPPLICATION_ALLOCATED_HEAP        0

/* --- Hook functions ------------------------------------------------------*/
#define configUSE_IDLE_HOOK                     1   /* idle hook calls loop() */
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2   /* catch stack issues */
#define configUSE_MALLOC_FAILED_HOOK            1   /* catch heap exhaustion */
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* --- Run time and task stats ---------------------------------------------*/
#define configGENERATE_RUN_TIME_STATS           0   /* MUST stay 0: FLEXPWM2 conflicts */
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* --- Co-routines ---------------------------------------------------------*/
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         2

/* --- Software timers -----------------------------------------------------*/
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE * 2)

/* --- Optional functions --------------------------------------------------*/
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1   /* debug stack usage */
#define INCLUDE_xTaskGetIdleTaskHandle          0
#define INCLUDE_eTaskGetState                   0
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 0
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1
#define INCLUDE_vTaskEndScheduler               0

/* --- Interrupt priority --------------------------------------------------*/
#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 8
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY ((1U << (configPRIO_BITS)) - 1)
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 2

#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#endif /* TEENSY4_FREERTOS_CONFIG_H */
