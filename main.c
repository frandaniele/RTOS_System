/* Environment includes. */
#include "DriverLib.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
//#include "printf-stdarg.h"

#include <stdlib.h>

/* Delay between cycles of the 'sensor' task. Frequency 10 Hz */
#define mainSENSOR_DELAY						( ( TickType_t ) 100 / portTICK_PERIOD_MS )

/* Delay between cycles of the 'filter' task. Frequency 20 Hz */
#define mainFILTER_DELAY						( ( TickType_t ) 50 / portTICK_PERIOD_MS )

/* Delay between cycles of the 'graph' task. Frequency 1 Hz / every 1 sec */
#define mainGRAPH_DELAY							( ( TickType_t ) 1000 / portTICK_PERIOD_MS )

/* Delay between cycles of the 'stats' task. Frequency 0.5 Hz / every 2 sec */
#define mainSTATS_DELAY							( ( TickType_t ) 2000 / portTICK_PERIOD_MS )

/* UART configuration - note this does not use the FIFO so is not very
efficient. */
#define mainBAUD_RATE				( 19200 )
#define mainFIFO_SET				( 0x10 )

/* task priorities. 0 is highest*/
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )

/* Misc. */
#define mainQUEUE_SIZE				( 1 )
#define mainNO_DELAY				( ( TickType_t ) 0 )
#define MAX_ARRAY 					( 50 ) 
#define N_PIXELES 					( 96 ) 

static void prvSetupHardware( void );

static void vSensorTask( void *pvParameters );

static void vFilterTask( void *pvParameters );

static void vGraphTask( void *pvParameters );

static void vStatsTask( void *pvParameters );

static void vUART_NTask( void *pvParameters );

int filtrar(int temps[MAX_ARRAY], int N);

void shiftR_array_int(int * temps, int tamanio);

void shiftL_array_char(char * pixels, int tamanio);

unsigned char temp_to_pixel(int temp);

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

	xReturned = xTaskCreate( vSensorTask, "Sensor", configMINIMAL_STACK_SIZE + 100, NULL, mainCHECK_TASK_PRIORITY, NULL );
	if(xReturned != pdPASS) return EXIT_FAILURE;

	xReturned = xTaskCreate( vFilterTask, "Filter", configMINIMAL_STACK_SIZE + 100, NULL, mainCHECK_TASK_PRIORITY, NULL );
	if(xReturned != pdPASS) return EXIT_FAILURE;

	xReturned = xTaskCreate( vGraphTask, "Graph", configMINIMAL_STACK_SIZE + 100, NULL, mainCHECK_TASK_PRIORITY, NULL );
	if(xReturned != pdPASS) return EXIT_FAILURE;

	//xReturned = xTaskCreate( vStatsTask, "Stats", configMINIMAL_STACK_SIZE + 100, NULL, mainCHECK_TASK_PRIORITY - 2, NULL );
	//if(xReturned != pdPASS) return EXIT_FAILURE;

	xReturned = xTaskCreate( vUART_NTask, "UART_N", configMINIMAL_STACK_SIZE + 100, NULL, mainCHECK_TASK_PRIORITY - 2, NULL );
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
	//volatile UBaseType_t uxHighWaterMark;
	BaseType_t xReturned;

    TickType_t xLastExecutionTime = xTaskGetTickCount();
    
	int temperatura = 0;
	int op = 1;
	for( ;; )
	{
		/* Perform this check every mainSENSOR_DELAY milliseconds. */
		vTaskDelayUntil( &xLastExecutionTime, mainSENSOR_DELAY );

		//envio la medicion
        xReturned = xQueueSend( xTempsQueue, &temperatura, 0 );
		if(xReturned == pdTRUE){
			temperatura += op;
			if(temperatura == 36) op = -1;
			if(temperatura == 4) op = 1;
		}

    	//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		//int a = (int) uxHighWaterMark;
    }
}

static void vFilterTask( void *pvParameters ){
	//volatile UBaseType_t uxHighWaterMark;
	
	int temps[MAX_ARRAY] = {0};
    int temp_recibida;
	int N = 1;
	int nuevo_N = 1;
	BaseType_t xReturned;

	vTaskDelay(mainSENSOR_DELAY);

	for( ;; )
	{
		/* Wait for a message to arrive. */

		xReturned = xQueueReceive( xTempsQueue, &temp_recibida, mainFILTER_DELAY );
		if(xReturned == pdTRUE){ //recibi algo
			temps[0] = temp_recibida;

			//chequear nuevo N
			xReturned = xQueueReceive( xNQueue, &nuevo_N, 0 );
			if(xReturned == pdTRUE){
				N = nuevo_N;
			}

			int media_movil = filtrar(temps, N);

			//enviar media movil a print task
			xQueueSend( xPrintQueue, &media_movil, 0 );

			shiftR_array_int(temps, MAX_ARRAY);
		}
    	
		//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		//int a = (int) uxHighWaterMark;
	}
}

static void vGraphTask( void *pvParameters ){
    //volatile UBaseType_t uxHighWaterMark;
	
	TickType_t xLastExecutionTime = xTaskGetTickCount();
	BaseType_t xReturned;

	int temp_recibida;
	int count = 0;
	unsigned char pixels_to_graph[N_PIXELES] = {0};

	/* Write the message to the LCD. */
	for( ;; )
	{
		vTaskDelayUntil( &xLastExecutionTime, mainGRAPH_DELAY );

		xReturned = xQueueReceive( xPrintQueue, &temp_recibida, portMAX_DELAY );
		if(xReturned == pdTRUE){
			
			if(count < N_PIXELES - 1) {
				pixels_to_graph[count] = temp_to_pixel(temp_recibida);
				count++;
			}
			else{
				pixels_to_graph[N_PIXELES - 1] = temp_to_pixel(temp_recibida);
				shiftL_array_char(pixels_to_graph, N_PIXELES);
			}

			OSRAMClear();
			OSRAMImageDraw(pixels_to_graph, 0, 0, N_PIXELES, 1);

		}
		//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		//int a = (int) uxHighWaterMark;
	}
}

static void vStatsTask( void *pvParameters ){
	//volatile UBaseType_t uxHighWaterMark;
	
	TickType_t xLastExecutionTime = xTaskGetTickCount();

	for( ;; )
	{
		vTaskDelayUntil( &xLastExecutionTime, mainSTATS_DELAY );
		
    	//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	}
}


static void vUART_NTask( void *pvParameters ){
	//volatile UBaseType_t uxHighWaterMark;
	BaseType_t xReturned;

	char charRecibido;

	int flag = 0, nuevo_N = 1, digitos = 0, N_a_enviar = 1;

	for( ;; )
	{
		xReturned = xQueueReceiveFromISR( xUARTQueue, &charRecibido, NULL );
		if(xReturned == pdTRUE){
			if (charRecibido == '<'){//comienzo
				flag = 1;
				nuevo_N = 1;
			} 
			else if (charRecibido >= '0' && charRecibido <= '9' && flag){//recibo 2 digitos
				digitos++;
				if (digitos == 1) 		nuevo_N += (int) (charRecibido - 0x30)*10;
				else if (digitos == 2) 	nuevo_N += (int) (charRecibido - 0x30);
				else if (digitos > 2){
					digitos = 0;
					flag = 0;
				}
			}
			else if (charRecibido == '>' && flag){//final
                if(digitos == 1) {	        
                    N_a_enviar = N_a_enviar;
                    }
                else{
                    if(nuevo_N <= 1) 				N_a_enviar = 1;
                    else if(nuevo_N >= MAX_ARRAY) 	N_a_enviar = MAX_ARRAY - 1;
    				else 							N_a_enviar = nuevo_N - 1; //xq inicia en 1

					xQueueSend( xNQueue, &N_a_enviar, 0 );
                }

				flag = 0;
				digitos = 0;
			}
			else{//otra cosa, empiezo secuencia de nuevo
				flag = 0;
				digitos = 0;
			}
		}
		//recibir char por uart isr
		//chequear char de inicio
		//chequear final
		//si se inicio, chequear q sea num y cual y guardarlo
		//enviaarlo a filter

    	//uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	}
}

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

// mueve los valores del array a la derecha y deja lugar para uno nuevo al inicio
void shiftR_array_int(int * temps, int tamanio){
    for(int i = tamanio - 1; i > 0; i--){
        temps[i] = temps[i-1];
    }
}

// mueve los valores del array a la izquierda y deja lugar para uno nuevo al final
void shiftL_array_char(char * pixels, int tamanio){
    for(int i = 0; i < tamanio - 1; i++){
        pixels[i] = pixels[i+1];
    }
}

//mapea los valores de temperatura a pixeles a dibujar segun osram 96x16
unsigned char temp_to_pixel(int temp){
    if(temp < 5)        return 128; //0b10000000
    else if(temp < 10)  return 64;
    else if(temp < 15)  return 32;
    else if(temp < 20)  return 16;  
    else if(temp < 25)  return 8;
    else if(temp < 30)  return 4;
    else if(temp < 35)  return 2;
    else if(temp >= 35) return 1; //0b00000001
}