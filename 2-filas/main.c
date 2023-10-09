/*
 *
 * Desenvolver exemplos usando buffer de comuniação (Queue)
 * com propósito de comunicação entre duas ou mais tarefas.
 *
 *
 * Pinos utilizados: 
 *		PA0 -> PWM Ch0 -> TIM2 
 *		PA1 -> ADC1 configurado e não utilizado
 *
 * USART: 
 *		A9 -> TX
 *		A10 RX
 *
 *
 *		PC13 -> Led Onboard
 *
 * */

//Lib. Auxiliares:
#include "stm32f10x.h"
#include <math.h>
#include "Me.h"
// #include <string.h>
/* #include "stm32f10x_adc.h" */

// FreeRTOS: 
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define __delay_ms( TEMPO ) vTaskDelay(pdMS_TO_TICKS( TEMPO ))
#define __delay( TEMPO ) vTaskDelay( TEMPO )

#define pwm_control(VAR) TIM2->CCR1 = (VAR)
#define value_ADC1	ADC1->DR

// void ADC1_configure_lib();
void configure_PWM_TIM2();
void usart_init(void);
void USARTSend(char *pucBuffer);


static QueueHandle_t xQueue; // Declara a variável com tipo
									  // QueueHandle_t

static void task1(void *args __attribute((unused))) {
	GPIOB->CRH &= ~(0xf << 8 ); // ip analogico PA10

	u16 vpwm = 0x3ff /2; 
	while(1){
			 if((GPIOA->IDR & (1 << 10)))
				 vpwm +=3;

		if(vpwm >= 0x3ff)
			vpwm = 0;
		
		TIM2->CCR1 = vpwm; 
		xQueueSend(xQueue, &vpwm, 1);

		__delay_ms(10);


	}
}

#define con 0xfff
static void task2(void *args __attribute ((unused))){
	u16 dd; 
	char info[10] ;

	while(1){
		if(xQueueReceive(xQueue, &dd, pdMS_TO_TICKS(10))){
			floatToString((float) dd/con *100, info, 4); 
			USARTSend("%)\r\nValor de Duty Cicle= (");
			USARTSend((char * ) &info);
		}

	}
}

static void task3(void *args __attribute ((unused))){

	GPIOC->CRH |=(0x3<<20);
	while(1){
		GPIOC->ODR	^= (1<<13);
		__delay(50);
	}
}



/*
 * ----
 *  1 -> Usar uma tarefa para capturar dados e atuar no PWM
 *  (por enquanto tudo junto) e enviar para o buffer e
 *  mostrar a faixa do atual valor.
 *
 *  2 -> a tarefa receberá e mostrará com pouca precisão o
 *  valor do duty cicle atual via UART
 *
 * */


int main(void) {
	xQueue = xQueueCreate(5 /* Tamanho máximo da fila. */,
								sizeof(u32) /* Largura máxima da fila. */);

	set_system_clock_to_72Mhz();

	RCC->APB2ENR |= 0x7<<2 ; // Hab. GPIO A,B,C
	GPIOC->CRH |= 0b11 << 20;

	RCC->APB2ENR |= 0b1<<4 ;
	GPIOC->CRH |= 0b11 << 20;

	configure_PWM_TIM2();
	 usart_init();

	xTaskCreate(task1,"PWM_Control", 100 ,NULL, 4 ,NULL);
	xTaskCreate(task2, "ADC_Read", 200 , NULL, 4 , NULL);
	xTaskCreate(task3, "Led_C13", 100 , NULL, 4 , NULL);

	vTaskStartScheduler(); // inicia o escalonador de
								  // processos.
	while(1);
	return 0;
}


void configure_PWM_TIM2(){
/* Configurado com o exemplo do datasheet pag "387 / 1136" */

   RCC->APB2ENR |= 0b101 << 0 ; /*Habilita o AFIO GPIOA */
   RCC->APB1ENR |= 1 << 0; /*Habilita o TIMER 2 - CH na GPIO_A */

   GPIOA->CRL |= (0x0B << 0) ; /* AF O_PP pino PA0 */
	AFIO->MAPR &= ~( 0b11 << 6);
   AFIO->MAPR &= ~( 0b11 << 8); /* [bit 26:24] SWJ_CFG [nota_2]
                 [bit 8] TIM3_REMAP[1:0]: TIM2 remapping [nota_3]*/ 

	TIM2->CCMR1 |= 0b110 << 4; /*Bits 6:4 OC1M: Output compare 1 mode
                                  110: PWM mode 1 */
   TIM2->CCMR1 |= 1UL << 3; /* Bit 3 OC1PE: Output compare 1 preload enable**/


   TIM2->PSC = 709; /* 249;  25/250 - 1 => 0.1KHz  */
   TIM2->ARR = 999; /* PWM period = (999 + 1) * 100KHz = 0.01 */
   TIM2->CR1 &= ~(1<<4);
   TIM2->CR1 |= 1<<7;/*Bit 7 ARPE: Auto-reload preload enable */
   TIM2->CCMR1 |= 1UL << 3; /* Bit 3 OC1PE: Output compare 1 preload enable - ---- CONSEGUIIIIIII **/
   TIM2->EGR |=1<<0;

   TIM2->CCER |= 1<<0; /*Bit 0 CC1E: Capture/Compare 1 output enable*/
   TIM2->CCER &= ~(1<<1); /*Bit 1 CC1P: Capture/Compare 1 output polarity
                            0: OC1 active high. */

   TIM2->CR1 |= 1<<0; //Habilita o clock
   TIM2->CCR1 = 00;
}


void usart_init(void) {
	/* Enable USART1 and GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure the GPIOs */
	//GPIO_Configuration();
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the USART1 */
	//USART_Configuration();
	USART_InitTypeDef USART_InitStructure;

	/* USART1 configuration ------------------------------------------------------*/
	/* USART1 configured as follow:
	          - BaudRate = 115200 baud
	          - Word Length = 8 Bits
	          - One Stop Bit
	          - No parity
	          - Hardware flow control disabled (RTS and CTS signals)
	          - Receive and transmit enabled
	          - USART Clock disabled
	          - USART CPOL: Clock is active low
	          - USART CPHA: Data is captured on the middle
	          - USART LastBit: The clock pulse of the last data bit is not output to
	                           the SCLK pin
	 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART1 */
	USART_Cmd(USART1, ENABLE);

	/* Enable the USART1 Receive interrupt: this interrupt is generated when the
		USART1 receive data register is not empty */
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USARTSend(char *pucBuffer) {
    while (*pucBuffer)
    {
        USART_SendData(USART1, *pucBuffer++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}
// https://github.com/avislab/STM32F103/blob/master/Example_ADC_Temperature/main.c#L74
