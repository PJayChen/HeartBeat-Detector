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
#include "hw_it.h"

#include "string-util.c"


/* Private macro -------------------------------------------------------------*/

/* semaphores, queues declarations */
xQueueHandle xQueueUARTSend;
xQueueHandle xQueueUARTRecvie;

xSemaphoreHandle serial_tx_wait_sem;

/* software Timers */
xTimerHandle xTimerNoSignal;
xTimerHandle xTimerADC;
xTimerHandle xTimer10ms;
xTimerHandle xTimerFindMax;


uint16_t tick1sec = 0;
uint8_t tick10Hz = 0;

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

enum{  
	FIND_MAX = 0,
	FIND_SLOPE,
	INCR,
	DECR

}STATE;

void vHeartBeatTask(void *pvParameters)
{
	uint16_t Maximum = 0, curr_ADC = 0, last_ADC = 0;
	uint8_t state = FIND_MAX;
	uint16_t HeartBeat = 0;

	uint16_t Max_int[2];
	double Max_float;

	

	qprintf(xQueueUARTSend, "Start Sampling...\n");
	while(1){
		switch(state){
			case FIND_MAX:
				if(xTimerIsTimerActive(xTimerFindMax) == pdTRUE){
					if(ADCConvertedValue[0] > Maximum) Maximum = ADCConvertedValue[0];
					if(tick1sec > 0) {
						state = FIND_SLOPE;
						last_ADC = ADCConvertedValue[0];
						xTimerStop(xTimerFindMax, 0);
						tick1sec = 0;
						Max_float = (Maximum * 2.96f) / 4096.0f;
						Max_int[0] = (uint16_t) Max_float;
						Max_int[1] = (Max_float - Max_int[0]) * 1000;
						qprintf(xQueueUARTSend, "\nMaximum is: %d.%d v\n", Max_int[0], Max_int[1]);
					}
				}else{
					xTimerStart(xTimerFindMax, 0);
				}
				break;
			case FIND_SLOPE:
				curr_ADC = ADCConvertedValue[0];
				if(curr_ADC > last_ADC) state = INCR;
				else if(curr_ADC < last_ADC) state = DECR;
				last_ADC = curr_ADC;
				break;
			case INCR:
				curr_ADC = ADCConvertedValue[0];
				//qprintf(xQueueUARTSend, "\rHeart Beat: %d                   I", HeartBeat);
				if( (curr_ADC < last_ADC) && (last_ADC >= Maximum - 50)){
					if(xTimerIsTimerActive(xTimer10ms) == pdTRUE){
						xTimerStop(xTimer10ms, 0);
						HeartBeat = (tick10Hz != 0)?(int)(60 * 100 / tick10Hz):0;
						qprintf(xQueueUARTSend, "\r                     \r");
						qprintf(xQueueUARTSend, "Heart Beat: %d", HeartBeat);
						//qprintf(xQueueUARTSend, "tick10Hz: %d", tick10Hz);
						tick10Hz = 0;
						state = FIND_MAX;
					}else{
						xTimerStart(xTimer10ms, 0);
						state = DECR;
					}
				}
				last_ADC = curr_ADC;
				break;
			case DECR:
				curr_ADC = ADCConvertedValue[0];
				//qprintf(xQueueUARTSend, "\rHeart Beat: %d                   D", HeartBeat);
				if(curr_ADC > last_ADC){
					state = INCR;
				}
				last_ADC = curr_ADC;
		}
		vTaskDelay(50 / portTICK_RATE_MS);
	}
}

/* Software Timer Function ----------------------------------------------------*/



/* 40 sec idle time pass ... trun off moto */
void vTimerSystemIdle( xTimerHandle pxTimer )
{

	qprintf(xQueueUARTSend, "10 sec...\n\r");
}

void vTimer10HzTick(xTimerHandle pxTimer)
{
	tick10Hz++;
}

#define BUFFERSIZE 8
#define THRESHOLD 800
uint8_t tick = 0, flag_t, flag;
uint16_t deltaBuffer[BUFFERSIZE] = {0};
uint16_t recentTotal = 0, newADC = 0, lastADC = 0, delta = 0, index = 0;
uint16_t heartBeat = 0;

void vTimerReadADCValue(xTimerHandle pxTimer)
{
	double AD_value;
	uint16_t value[3];
	
	value[0] = ADCConvertedValue[0];
	AD_value = (value[0] * 2.96f) / 4096.0f;
	value[1] = (uint16_t) AD_value;
	value[2] = (AD_value - value[1]) * 1000;
	//qprintf(xQueueUARTSend, "\r                     \r");
	//qprintf(xQueueUARTSend, "voltage:%d.%d", value[1], value[2]);
	
	newADC = value[2];
	delta = newADC - lastADC;
	lastADC = newADC;
	
	recentTotal = recentTotal - deltaBuffer[index] + delta;
	if(recentTotal >= 65000) recentTotal = 0;

	deltaBuffer[index] = delta;
	index = (index + 1) % BUFFERSIZE;
	//qprintf(xQueueUARTSend, "recentTotal: %d\n", recentTotal);
	if(recentTotal >= THRESHOLD){
		if(flag == 0){
			flag_t = 1;
			flag = 1;
		}else {
			heartBeat = (int)(60 * (10 / tick));
			qprintf(xQueueUARTSend, "\r                     \r");
			qprintf(xQueueUARTSend, "Heart Beat: %d", heartBeat);
			flag = 0;
			flag_t = 0;
		}
	}
		//qprintf(xQueueUARTSend, "\r                     \r");
		//	qprintf(xQueueUARTSend, "Tick: %d", tick);
	if(flag_t == 1) tick++;
	else tick = 0;
	
}


void vTimer1secTick(xTimerHandle pxTimer){
	tick1sec++;
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{ 
	int timerID[3] = {0, 1, 2};

	/*A Timer used to count how long there is no signal come in*/
	xTimerNoSignal = xTimerCreate("TurnOffTime", 10000 / portTICK_RATE_MS, pdFALSE,  (void *) timerID[0], vTimerSystemIdle);

	//xTimerADC = xTimerCreate("ReadADC", 100 / portTICK_RATE_MS, pdTRUE, (void *) timerID[1], vTimerReadADCValue);
	
	xTimerFindMax = xTimerCreate("1sec tick", 1000/ portTICK_RATE_MS, pdTRUE,  (void *) timerID[1], vTimer1secTick);
	xTimer10ms = xTimerCreate("100 Hz tick", 10 / portTICK_RATE_MS, pdTRUE, (void *) timerID[2], vTimer10HzTick);

	/*a queue for tansfer the senddate to USART task*/
	xQueueUARTSend = xQueueCreate(15, sizeof(serial_str_msg));
   	xQueueUARTRecvie = xQueueCreate(15, sizeof(serial_ch_msg));

   	vSemaphoreCreateBinary(serial_tx_wait_sem);

	/* initialize hardware... */
	prvSetupHardware();

	//xTimerStart(xTimerNoSignal, 0);
	//xTimerStart(xTimerADC, 0);
	//xTimerStart(xTimer100ms, 0);

	xTaskCreate(vUsartSendTask, ( signed portCHAR * ) "USART", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL);
	xTaskCreate(vHeartBeatTask, ( signed portCHAR * ) "HeartBeat", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL);
	//xTaskCreate(shell, ( signed portCHAR * ) "shell", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY + 5, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;  
}

