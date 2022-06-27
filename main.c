/* Environment includes. */
#include "DriverLib.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Delay between cycles of the 'sensor' task. Frequency 10 Hz */
#define mainSENSOR_DELAY						pdMS_TO_TICKS ( 100 )

/* Delay between cycles of the 'graph' task. Frequency 1 Hz / every 1 sec */
#define mainGRAPH_DELAY							pdMS_TO_TICKS ( 1000 )

/* Delay between cycles of the 'stats' task. Frequency 0.5 Hz / every 2 sec */
#define mainSTATS_DELAY							pdMS_TO_TICKS ( 2000 )

/* UART configuration - note this does not use the FIFO so is not very
efficient. */
#define mainBAUD_RATE				( 19200 )
#define mainFIFO_SET				( 0x10 )

/* Demo task priorities. */
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )

/* Misc. */
#define mainQUEUE_SIZE				( 1 )
#define mainNO_DELAY				( ( TickType_t ) 0 )
#define MAX_ARRAY 					( 50 ) 

static void prvSetupHardware( void );

static void vSensorTask( void *pvParameters );

static void vFilterTask( void *pvParameters );

static void vGraphTask( void *pvParameters );

static void vStatsTask( void *pvParameters );

static void vUART_NTask( void *pvParameters );

int filtrar(int temps[MAX_ARRAY], int N);

void shift_array_int(int * temps, int tamanio);

void shift_array_char(char * pixels, int tamanio);

/* The queue used to send measured temperature to filter. */
QueueHandle_t xTempsQueue;

/* The queue used to send received data from UART to task */
QueueHandle_t xUARTQueue;

/* The queue used to send the number N to filter */
QueueHandle_t xNQueue;

/* The queue used to send strings to the print task for display on the LCD. */
QueueHandle_t xPrintQueue;

/*-----------------------------------------------------------*/

int main( void )
{
	/* Configure the clocks, UART and GPIO. */
	prvSetupHardware();

	xTempsQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( int ) );
	if(xTempsQueue == NULL) return EXIT_FAILURE; 

	xPrintQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( int ) );
	if(xPrintQueue == NULL) return EXIT_FAILURE; 

	xNQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( int ) );
	if(xNQueue == NULL) return EXIT_FAILURE; 

	xUARTQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( char ) );
	if(xUARTQueue == NULL) return EXIT_FAILURE; 

    /* Start the tasks defined within the file. */
	BaseType_t xReturned;

	xReturned = xTaskCreate( vSensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY , NULL );
	if(xReturned != pdPASS) return EXIT_FAILURE;

	xReturned = xTaskCreate( vFilterTask, "Filter", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );
	if(xReturned != pdPASS) return EXIT_FAILURE;

	xReturned = xTaskCreate( vGraphTask, "Graph", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 3, NULL );
	if(xReturned != pdPASS) return EXIT_FAILURE;

	xReturned = xTaskCreate( vStatsTask, "Stats", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 2, NULL );
	if(xReturned != pdPASS) return EXIT_FAILURE;

	xReturned = xTaskCreate( vUART_NTask, "UART_N", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 2, NULL );
	if(xReturned != pdPASS) return EXIT_FAILURE;

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient heap to start the
	scheduler. */

	return 0;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Setup the PLL. */
	SysCtlClockSet( SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ );

	/* Enable the UART.  */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	/* Set GPIO A0 and A1 as peripheral function.  They are used to output the
	UART signals. */
	GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_HW );

	/* Configure the UART for 8-N-1 operation. */
	UARTConfigSet( UART0_BASE, mainBAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE );

	/* We don't want to use the fifo.  This is for test purposes to generate
	as many interrupts as possible. */
	HWREG( UART0_BASE + UART_O_LCR_H ) &= ~mainFIFO_SET;

	/* Enable rx interrupts. */
	HWREG( UART0_BASE + UART_O_IM ) |= UART_INT_RX;
	IntPrioritySet( INT_UART0, configKERNEL_INTERRUPT_PRIORITY );
	IntEnable( INT_UART0 );

	/* Initialise the LCD> */
    OSRAMInit( false );
    OSRAMStringDraw("www.FreeRTOS.org", 0, 0);
	OSRAMStringDraw("LM3S811 demo", 16, 1);
}
/*-----------------------------------------------------------*/

static void vSensorTask( void *pvParameters ){
    TickType_t xLastExecutionTime = xTaskGetTickCount();

    int temperatura = 0;
	int op = 1;
	for( ;; )
	{
		/* Perform this check every mainSENSOR_DELAY milliseconds. */
		vTaskDelayUntil( &xLastExecutionTime, mainSENSOR_DELAY );

		//envio la medicion
        xQueueSend( xTempsQueue, &temperatura, 0 );
        
        temperatura += 3 * op;
		if(temperatura == 42) op = -1;
		if(temperatura == -6) op = 1;
    }
}

static void vFilterTask( void *pvParameters ){
	int temps[MAX_ARRAY] = {0};
    int temp_recibida;
	int N = 10;

	for( ;; )
	{
		/* Wait for a message to arrive. */
		xQueueReceive( xTempsQueue, &temp_recibida, portMAX_DELAY );

		temps[0] = temp_recibida;

		//chequear nuevo N

		int media_movil = filtrar(temps, N);

		//enviar media movil a print task
        xQueueSend( xPrintQueue, &media_movil, 0 );

		shift_array_int(temps, MAX_ARRAY);
	}
}

static void vGraphTask( void *pvParameters ){
    TickType_t xLastExecutionTime = xTaskGetTickCount();

	int temp_recibida;
	//REVISAR TAMANIO SEGUN CUANTOS VALORES PUEDO DIBUJAR
	unsigned char pixels_to_graph[MAX_ARRAY] = {0};
	/* Write the message to the LCD. */
	for( ;; )
	{
		vTaskDelayUntil( &xLastExecutionTime, mainGRAPH_DELAY );
		
		xQueueReceive( xPrintQueue, &temp_recibida, portMAX_DELAY );
		pixels_to_graph[0] = temp_recibida;

		//PASAR DE TEMPERATURAS A COORDENADAS PA DIBUJAR

		OSRAMClear();
		OSRAMStringDraw( "recibi filtracion", 0, 0);
		//OSRAMImageDraw(pixels_to_graph)

		shift_array_char(pixels_to_graph, MAX_ARRAY);
	}
}

static void vStatsTask( void *pvParameters ){
	TickType_t xLastExecutionTime = xTaskGetTickCount();

	for( ;; )
	{
		vTaskDelayUntil( &xLastExecutionTime, mainSTATS_DELAY );
		
	}
}

static void vUART_NTask( void *pvParameters ){

	for( ;; )
	{
		//recibir char por uart isr
		//chequear char de inicio
		//chequear final
		//si se inicio, chequear q sea num y cual y guardarlo
		//enviaarlo a filter

	}
}

// REVISAR
void vUART_ISR(void){
	unsigned long ulStatus;
	char charRecibido;

	/* What caused the interrupt. */
	ulStatus = UARTIntStatus( UART0_BASE, pdTRUE );

	/* Clear the interrupt. */
	UARTIntClear( UART0_BASE, ulStatus );

	/* Was a Rx interrupt pending? */
	if( ulStatus & UART_INT_RX )
	{
		/* Si hay algo lo recibo */
		if( ( HWREG( UART0_BASE + UART_O_FR ) & UART_FR_RXFF ) )
		{
			charRecibido = (char) HWREG( UART0_BASE + UART_O_DR );				
			
			xQueueSendFromISR( xUARTQueue, &charRecibido, NULL );
		}
	}
}
/*-----------------------------------------------------------*/

// hace la media movil simple 
int filtrar(int temps[MAX_ARRAY], int N){
    int temperatura = 0;
    for(int i = 0; i < N; i++){
        temperatura += temps[i];
    }

    return temperatura/N;
}

// mueve los valores del array a la derecha y deja lugar para uno nuevo
void shift_array_int(int * temps, int tamanio){
    for(int i = tamanio - 1; i > 0; i--){
        temps[i] = temps[i-1];
    }
}

void shift_array_char(char * pixels, int tamanio){
    for(int i = tamanio - 1; i > 0; i--){
        pixels[i] = pixels[i-1];
    }
}