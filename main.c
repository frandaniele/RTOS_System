/* Environment includes. */
#include "DriverLib.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <stdio.h>

/* Demo app includes. */
#include "integer.h"
#include "PollQ.h"
#include "semtest.h"
#include "BlockQ.h"

/* Delay between cycles of the 'sensor' task. Frequency 10 Hz */
#define mainSENSOR_DELAY						( ( TickType_t ) 100 / portTICK_PERIOD_MS )

/* UART configuration - note this does not use the FIFO so is not very
efficient. */
#define mainBAUD_RATE				( 19200 )
#define mainFIFO_SET				( 0x10 )

/* Demo task priorities. */
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )

/* Misc. */
#define mainQUEUE_SIZE				( 3 )
#define mainDEBOUNCE_DELAY			( ( TickType_t ) 150 / portTICK_PERIOD_MS )
#define mainNO_DELAY				( ( TickType_t ) 0 )
#define MAX_ARRAY 			( 50 ) 

static void prvSetupHardware( void );

static void vSensorTask( void *pvParameters );

static void vFilterTask( void *pvParameters );

static void vGraphTask( void *pvParameters );

static void vStatsTask( void *pvParameters );

int filtrar(int temps[MAX_ARRAY], int N);

void shift_array(int temps[MAX_ARRAY], int tamanio);

/* The queue used to send strings to the print task for display on the LCD. */
QueueHandle_t xTempsQueue;
QueueHandle_t xPrintQueue;

/*-----------------------------------------------------------*/

int main( void )
{
	/* Configure the clocks, UART and GPIO. */
	prvSetupHardware();

	xTempsQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( int ) );
	xPrintQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( int ) );

	/* Start the standard demo tasks. */
	vStartIntegerMathTasks( tskIDLE_PRIORITY );
	vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
	vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
	vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );

    /* Start the tasks defined within the file. */
	xTaskCreate( vSensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
	xTaskCreate( vFilterTask, "Filter", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );
	xTaskCreate( vGraphTask, "Graph", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 2, NULL );
	//xTaskCreate( vStatsTask, "Stats", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 3, NULL );

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

	/* Enable Tx interrupts. */
	HWREG( UART0_BASE + UART_O_IM ) |= UART_INT_TX;
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
        xQueueSendToFront( xTempsQueue, &temperatura, 0 );
        
        temperatura += 3 * op;
		if(temperatura == 42) op = -1;
		if(temperatura == -6) op = 1;
    }
}

static void vFilterTask( void *pvParameters ){
	int temps[MAX_ARRAY] = {20};
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
        xQueueSendToFront( xPrintQueue, &media_movil, 0 );

		shift_array(temps, MAX_ARRAY);
	}
}

static void vGraphTask( void *pvParameters ){
	int temp_recibida;
    unsigned portBASE_TYPE uxLine = 0, uxRow = 0;
	/* Write the message to the LCD. */
	for( ;; )
	{
		xQueueReceive( xPrintQueue, &temp_recibida, portMAX_DELAY );

		uxRow++;
		uxLine++;
		OSRAMClear();
		OSRAMStringDraw( "recibi filtracion", uxLine & 0x3f, uxRow & 0x01);
	}
}

static void vStatsTask( void *pvParameters ){

}

void vUART_ISR(void)
{
unsigned long ulStatus;

	/* What caused the interrupt. */
	ulStatus = UARTIntStatus( UART0_BASE, pdTRUE );

	/* Clear the interrupt. */
	UARTIntClear( UART0_BASE, ulStatus );

	/* Was a Tx interrupt pending? */
	if( ulStatus & UART_INT_TX )
	{
		/* Send the next character in the string.  We are not using the FIFO. */
		/*if( *pcNextChar != 0 )
		{
			if( !( HWREG( UART0_BASE + UART_O_FR ) & UART_FR_TXFF ) )
			{
				HWREG( UART0_BASE + UART_O_DR ) = *pcNextChar;
			}
			pcNextChar++;
		}*/
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
void shift_array(int temps[MAX_ARRAY], int tamanio){
    for(int i = tamanio - 1; i > 0; i--){
        temps[i] = temps[i-1];
    }
}