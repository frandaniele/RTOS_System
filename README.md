## Sistemas Operativos II - Laboratorio VII RTOS - Francisco Daniele
###  Ingeniería en Computación - FCEFyN - UNC
# Real Time Operating Systems

## Desarrollo
Este sistema de Tiempo Real que diseñé para cumplir con este laboratorio consta de 5 tareas, 4 colas para comunicarse y 2 servicios de interrupción.
-   Filter Task:
-   UART Task:
-   Sensor Task:
-   Graph Task:
-   Stats Task:
-   Temps Queue:
-   Print Queue:
-   N Queue:
-   UART Queue:
-   UART ISR:
-   Timer0 ISR:

Además, se usan varias configuraciones del demo proporcionador por FreeRTOS 'CORTEX_LM3S811_GCC' que es un programa para correr sobre la placa Stellaris LM3S811, la misma sobre la cual se emula este trabajo.
Se destacan los siguientes cambios realizados a dichas configuraciones para poder cumplir con las consignas del laboratorio:
-   FreeRTOSConfig.h:

INCLUDE_uxTaskGetStackHighWaterMark (stack poner algo)
configUSE_TRACE_FACILITY
configGENERATE_RUN_TIME_STATS
portCONFIGURE_TIMER_FOR_RUN_TIME_STATS
timer use 0 por  lmi timer

-   Makefile:

heap la cambie por el pv free -> heap_1 - the very simplest, does not permit memory to be freed.  heap_2 is now considered legacy as heap_4 is preferred.
agregue driver en makefile, -g 

-   init/startup.c:

Timer0IntHandler 

Además se agregaron algunas librerías para facilitar el trabajo:

-   Timer.c:
-   printf-stdarg.c
