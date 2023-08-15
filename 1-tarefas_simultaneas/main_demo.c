/*
 * Objetivo: Destacar o uso de tarefas de 3 tarefas
 * simultâneas em execução.
 * 
 * */

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Me.h"

#define __delay( TEMPO ) vTaskDelay(pdMS_TO_TICKS( TEMPO ))

void configure_PWM_TIM2();// Saída mapeada no pino A0/CH1

static void task1(void *args __attribute((unused))) {
	while(1){
		GPIOC->ODR ^= (1<<13);
		vTaskDelay(pdMS_TO_TICKS(100)); /* A macro pdMS_TO_TICKS() 
														converte milissegundos em ticks */
	}
}

static void task2(void *args __attribute ((unused))){

u16 c =0x10;
	while(1){
		/* ADC - 12 bits de resolução 
		 * PWM - 10 bits */

		/*
		 * Configurar um botão ou 2 para almentar e diminuir a
		 * intensidade;  Parâmetrizar limites __IO u16: Max
		 * <= 0xff min>=0x8
		 *
		 * Posteriormente usar conversor ADC para determinar
		 * usando transição de dados em porcentarem (conversão
		 * PWM 100 = 0x0fff e ADC (Bits de resolução max) =
		 * 100% e controle do PWD = CCR1 (capture compare
		 * register))
		 * */
/*	
		for (u16 i=0 ; i<=0x03ff; i++){
			TIM2->CCR1 = i;
			__delay(1);
		} 
*/
			if((GPIOA->IDR & (1 << 10))){
				c +=7;
			}
			if (c >= 0x3ff)
				c = 3;

			TIM2->CCR1 = c;
			__delay(10);
	}
}

static void task3(void *args __attribute ((unused))){
		GPIOB->CRH |= (0x07 << 16); // PB12 -> Out Dig.

		// GPIOA->CRH |= (0x8 << 8); // Pin A10 -> I_up_down
			// GPIOA->ODR &=~(1<<10); // pA10 -> Pull-Down
			GPIOA->ODR |=(1<<10); // pA10 -> Pull-up

	while(1){
			if((GPIOA->IDR & (1 << 10)))
				GPIOB->ODR |= (1<<12);

			if(!(GPIOA->IDR & (1 << 10)))
				GPIOB->ODR &= ~(1<<12);
	}
}

int main(void) {
	set_system_clock_to_72Mhz();

	RCC->APB2ENR |= 0x7<<2 ; // Hab. GPIO A,B,C
	GPIOC->CRH |= 0b11 << 20;

	RCC->APB2ENR |= 0b1<<4 ;
	GPIOC->CRH |= 0b11 << 20;

	configure_PWM_TIM2();

	xTaskCreate(task1,"LED", 100 ,NULL, 4 ,NULL);
	xTaskCreate(task2, "PWM", 100 , NULL, 4 , NULL);
	xTaskCreate(task3, "Botao", 100 , NULL, 4 , NULL);


	vTaskStartScheduler(); // inicia o escalonador de
								  // processos.

	while(1);
	return 0;
}


#define psc 71
#define arr 3000

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

// https://embarcados.com.br/blue-pill-stm32f103c8t6-introducao-ao-freertos/
