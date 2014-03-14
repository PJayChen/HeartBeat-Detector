/* Library includes. */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

#ifndef HW_CONFIG_H
#define HW_CONFIG_H

extern GPIO_InitTypeDef GPIO_InitStructure;
extern NVIC_InitTypeDef NVIC_InitStructure;

/* TIM4 Autoreload and Capture Compare register values */
#define TIM_ARR                          ((uint16_t)1900)
#define TIM_CCR                          ((uint16_t)1000)

/* definitions used by prvLED_Config() to reconfigure LED pins GPIO<->TIM4 */
#define GPIO	0
#define TIMER	1

/** 
 * ADC1 base address is 0x40012000
 * The offset value of ADC_DR register is 4Ch
 */
#define ADC1_DR_Address ((uint32_t)0x4001204C) 
 __IO uint16_t ADCConvertedValue[1]; //Array that is used to store the calculated DMA values for ADC1

void prvSetupHardware( void );
//void prvTIM4_Config(void);
//void prvMEMS_Config(void);
//void prvGRYO_Config(void);
//void prvLED_Config(char state);
//uint32_t LIS3DSH_TIMEOUT_UserCallback(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void TIM_Configuration(void);
void USART_Configuration(void);

void ADC_Configuration(void);
void DMA_Configuration(void);

#endif /*HW_CONFIG_H*/
