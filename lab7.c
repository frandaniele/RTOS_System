#include "DriverLib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Delay para obtener la frecuencia de 10 hz del sensor */
#define mainSENSOR_DELAY ( ( TickType_t ) 100 / portTICK_PERIOD_MS )

/* UART configuration - note this does not use the FIFO so is not very
efficient. */
#define mainBAUD_RATE	( 19200 )
#define mainFIFO_SET	( 0x10 )

#define mainQUEUE_SIZE	( 50 )

#define mainTASK_PRIORITY		( tskIDLE_PRIORITY + 3 )

static void prvSetupHardware( void );

static void vSensorTask( void *pvParameters );

static void vFilterTask( void *pvParameters );

static void vGraphTask( void *pvParameters );

static void vStatsTask( void *pvParameters );

/* The queue used to send strings to the print task for display on the LCD. */
QueueHandle_t xTempsQueue;

int main(){
	prvSetupHardware();

	xTempsQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( uint32_t * ) );

    /* Start the tasks defined within the file. */
	xTaskCreate( vSensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, mainTASK_PRIORITY, NULL );
	xTaskCreate( vFilterTask, "Filter", configMINIMAL_STACK_SIZE, NULL, mainTASK_PRIORITY - 1, NULL );
	xTaskCreate( vGraphTask, "Graph", configMINIMAL_STACK_SIZE, NULL, mainTASK_PRIORITY - 2, NULL );
	xTaskCreate( vStatsTask, "Stats", configMINIMAL_STACK_SIZE, NULL, mainTASK_PRIORITY - 3, NULL );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient heap to start the
	scheduler. */

	return 0;
}

static void prvSetupHardware( void )
{
	/* Setup the PLL. */
	SysCtlClockSet( SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ );
    
	/* Enable the UART.  */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	/* Set GPIO A0 and A1 as peripheral function.  They are used for input
	UART signals. */
	GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_IN );

	/* Configure the UART for 8-N-1 operation. */
	UARTConfigSet( UART0_BASE, mainBAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE );

	/* We don't want to use the fifo.  This is for test purposes to generate
	as many interrupts as possible. */
	HWREG( UART0_BASE + UART_O_LCR_H ) &= ~mainFIFO_SET;

	/* Enable Rx interrupts. */
	HWREG( UART0_BASE + UART_O_IM ) |= UART_INT_RX;
	IntPrioritySet( INT_UART0, configKERNEL_INTERRUPT_PRIORITY );
	IntEnable( INT_UART0 );


	/* Initialise the LCD> */
    OSRAMInit( false );
    OSRAMStringDraw("www.FreeRTOS.org", 0, 0);
	OSRAMStringDraw("LM3S811 demo", 16, 1);
}

static void vSensorTask( void *pvParameters ){
    TickType_t xLastExecutionTime = xTaskGetTickCount();

    uint32_t temperatura;

	for( ;; )
	{
		/* Perform this check every mainSENSOR_DELAY milliseconds. */
		vTaskDelayUntil( &xLastExecutionTime, mainSENSOR_DELAY );

        //obtengo una temperatura entre -3 y 42 grados
        temperatura = rand() % 45;
        temperatura -= 3; 

        xQueueSendToFront( xTempsQueue, &temperatura, 0 );
    }
}

static void vFilterTask( void *pvParameters ){
    uint32_t *temp_recibida;
    unsigned portBASE_TYPE uxLine = 0, uxRow = 0;

	for( ;; )
	{
		/* Wait for a message to arrive. */
		xQueueReceive( xTempsQueue, &temp_recibida, portMAX_DELAY );
		/* Write the message to the LCD. */
		uxRow++;
		uxLine++;
		OSRAMClear();
		OSRAMStringDraw( "recibi una medicion", uxLine & 0x3f, uxRow & 0x01);
	}
}

static void vGraphTask( void *pvParameters ){

}

static void vStatsTask( void *pvParameters ){

}

void vUART_ISR(void){
    unsigned long ulStatus;

	/* What caused the interrupt. */
	ulStatus = UARTIntStatus( UART0_BASE, pdTRUE );

	/* Clear the interrupt. */
	UARTIntClear( UART0_BASE, ulStatus );

	/* Was a Tx interrupt pending? */
	if( ulStatus & UART_INT_RX )
	{
		/* receive N */
		
	}
}