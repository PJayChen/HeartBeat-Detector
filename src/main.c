#include "stm32f4xx.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "queue.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "hw_config.h"  //all hardware configuration was setted here
#include "main.h"
#include "shell.h"
#include "stm32f4_discovery_l3g4200d.h"
#include "hw_it.h"

#include "string-util.c"


/* Private macro -------------------------------------------------------------*/

/* semaphores, queues declarations */
xQueueHandle xQueueUARTSend;
xQueueHandle xQueueUARTRecvie;

xSemaphoreHandle serial_tx_wait_sem;

/* software Timers */
xTimerHandle xTimerNoSignal;

/* Queue structure used for passing messages. */
typedef struct {
	char str[50];
} serial_str_msg;

typedef struct {
	char ch;
} serial_ch_msg;

/* Private functions ---------------------------------------------------------*/
char receive_byte()
{
serial_ch_msg msg;

/* Wait for a byte to be queued by the receive interrupts handler. */
while (!xQueueReceive(xQueueUARTRecvie, &msg, portMAX_DELAY));
return msg.ch;
}

/* Task functions ------------------------------------------------- */


/* Uart Task functions ------------------------------------------------- */
//Task For Sending Data Via USART
void vUsartSendTask(void *pvParameters)
{
	
	uint8_t curr_char;	

	while(1) {
		serial_str_msg msg;

		while (!xQueueReceive(xQueueUARTSend , &msg, portMAX_DELAY));

		/* Write each character of the message to the RS232 port. */
		curr_char = 0;
		while (msg.str[curr_char] != '\0') {
			//Wait till the flag resets
			while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
			//Send the data
			USART_SendData(USART2, msg.str[curr_char]); // Send Char from queue
			curr_char++;
		}
	}
	while(1);
}

void vTestTask(void *pvParameters)
{
	while(1){

	}
}

/* Software Timer Function ----------------------------------------------------*/

/* 40 sec idle time pass ... trun off moto */
void vTimerSystemIdle( xTimerHandle pxTimer )
{

	qprintf(xQueueUARTSend, "10 sec trun off motor\n\r");
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{ 
	int timerID = 1;
	int timerID1 = 2;

	/*A Timer used to count how long there is no signal come in*/
	xTimerNoSignal = xTimerCreate("TurnOffTime", 10000 / portTICK_RATE_MS, pdFALSE,  (void *) timerID, vTimerSystemIdle);

	/*a queue for tansfer the senddate to USART task*/
	xQueueUARTSend = xQueueCreate(15, sizeof(serial_str_msg));
   	xQueueUARTRecvie = xQueueCreate(15, sizeof(serial_ch_msg));

   	vSemaphoreCreateBinary(serial_tx_wait_sem);

	/* initialize hardware... */
	prvSetupHardware();

	xTimerStart(xTimerNoSignal, 0);

	xTaskCreate(vUsartSendTask, ( signed portCHAR * ) "USART", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL);
	xTaskCreate(shell, ( signed portCHAR * ) "shell", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY + 5, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;  
}

