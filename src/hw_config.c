/**
  ******************************************************************************
  * @file    STM32F4-Discovery FreeRTOS demo\hw_config.c
  * @author  T.O.M.A.S. Team
  * @version V1.0.0
  * @date    05-October-2011
  * @brief   Hardware initialization
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Library includes. */
#include "hw_config.h"
#include "FreeRTOSConfig.h"


/*-----------------------------------------------------------*/
void prvSetupHardware( void )
{
	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure LED IOs as output push-pull */
    /* Initialize LEDs on STM32F4_Discovery board */
	//prvLED_Config(GPIO);
	/* Configure User button pin (PA0) as external interrupt -> modes switching */
	//STM_EVAL_PBInit(BUTTON_USER,BUTTON_MODE_EXTI);

	/* Configuration of Timer4 to control LEDs based on MEMS data */
	//prvTIM4_Config();

	/* Configure LIS302 in order to produce data used for TIM4 reconfiguration and LED control */

  	RCC_Configuration();
  	GPIO_Configuration();
  //	TIM_Configuration();
  	USART_Configuration();
    
    DMA_Configuration();
    ADC_Configuration();
	 
    ADC_SoftwareStartConv(ADC1); // Start conversion by software.
}


/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
  
 // RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOD , ENABLE );
  //RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );
  
  //--Enable DMA1 clock--
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  
  //Enable ADC1 clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN, ENABLE);

  //Enable GPIO Clocks For USART2
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  //Enable Clocks for USART2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);    
}

/**
  * @brief  
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //ADC
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // PC0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; //analog mode
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
    /*----------------------------------------------------------------------*/

  //GPIO Configuration for USART - PA2, PA3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //Connect USART pins to AF
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

}



void DMA_Configuration(void){
  DMA_InitTypeDef DMA_InitStructure;

  DMA_DeInit(DMA2_Stream4);
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_Address;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &ADCConvertedValue[0];

  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;

  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;

  DMA_Init(DMA2_Stream4, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream4, ENABLE);
}

void ADC_Configuration(void){
  ADC_InitTypeDef ADC_InitStructure;

  

  //ADC Sturcture configuration
  ADC_DeInit(); // Reset all parameters to their default values
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE; // No scan?
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //continuous conversion
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // no external trigger for conversion
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // converted data will be shifted to the right
  ADC_InitStructure.ADC_NbrOfConversion = 1; // Number of used ADC channels
  ADC_Init(ADC1, &ADC_InitStructure);  

  // use channel 10 from ADC1, with sample time 15 cycles
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
  
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  
  //Enable ADC1 use DMA
  ADC_DMACmd(ADC1, ENABLE);
  //Enable ADC1
  ADC_Cmd(ADC1, ENABLE);
/*
  //Calibrate ADC1
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
*/
}

/**
  * @brief  configure the TIM4 for PWM mode
  * @param  None
  * @retval None
  */
void TIM_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    // Let PWM frequency equal 400Hz.
    // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
    // Solving for prescaler gives 240.
    TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInitStruct.TIM_Period = 2500 - 1;//6720 - 1;//3360 - 1;   
    TIM_TimeBaseInitStruct.TIM_Prescaler = 84 - 1;//40 - 1;//500 - 1; 
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;    
    TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStruct );
    
    TIM_OCStructInit( &TIM_OCInitStruct );
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    
    // Initial duty cycle equals 0%. Value can range from zero to 65535.
    //TIM_Pulse = TIM4_CCR1 register (16 bits)
    TIM_OCInitStruct.TIM_Pulse = 0; //(0=Always Off, 65535=Always On)
 
    TIM_OC1Init( TIM4, &TIM_OCInitStruct ); // Channel 1  LED
    TIM_OC2Init( TIM4, &TIM_OCInitStruct ); // Channel 2  LED
    TIM_OC3Init( TIM4, &TIM_OCInitStruct ); // Channel 3  LED
    TIM_OC4Init( TIM4, &TIM_OCInitStruct ); // Channel 4  LED
 
    TIM_Cmd( TIM4, ENABLE );
}

/**
  * @brief  configure the USART
  * @param  None
  * @retval None
  */
void USART_Configuration(void)
{
  //Structure With Data For USART Configuration
  USART_InitTypeDef USART_InitStructure;
  
  //USART Parameters
  USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;

  //Configuring And Enabling USART2
  USART_Init(USART2, &USART_InitStructure);
  

  //Enable USART Interrupt ----
  NVIC_InitTypeDef NVIC_InitStructure;

  USART_ClearFlag(USART2, USART_FLAG_TC);
  /* Enable transmit and receive interrupts for the USART2. */
  USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  /* Enable the USART2 IRQ in the NVIC module (so that the USART2 interrupt
   * handler is enabled). */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY + 0x10;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_Cmd(USART2, ENABLE);
}

/*------------below is original code------------*/




/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
