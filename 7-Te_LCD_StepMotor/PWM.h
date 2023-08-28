
void configure_PWM_TIM2(); // Canal 1 -> PA0
void configure_PWM_TIM4(); // Canal 1 -> PB6

void PWM_T2_out(u8 valor_atual);
void PWM_T4_out(u8 valor_atual);

/*=========================================================*/
/*=========================================================*/
/*=========================================================*/

void PWM_T2_out(u8 valor_atual){
	TIM2->CCR1 = valor_atual;
}

void PWM_T4_out(u8 valor_atual){
	TIM4->CCR1 = valor_atual;
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

void configure_PWM_TIM4(){

	RCC->APB1ENR	|= (1<<2); //TIM4 En
	RCC->APB2ENR	|= (1<<0 | 1<< 3); // Hab AFIO e GPIOB

	GPIOB->CRL		|= (0b1011<<24); // AF_PP -> PB6
	AFIO->MAPR		&= ~(1<<12); // No Remap
	
	TIM4->CCMR1		|= 0b110 << 4;
	TIM4->CCMR1		|= (1UL<<3);

	TIM4->PSC		= 719;
	TIM4->ARR		= 999;
	TIM4->CR1		&= ~(1<<4); // UpCounter
	TIM4->CR1		|= (1<<7); 
	TIM4->CCMR1		|= (1UL << 3); // Hab. o Preloar Register
	TIM4->EGR		|= 1<<0; // Reinicializa o contador e gera
									// um "evento".
	TIM4->CCER		|= (1<<0); 

	TIM4->CR1		|= (1<<0); 
	TIM4->CCR1		= 0x0fff/2; // Inicializa com
												 // metade da potencia
}
