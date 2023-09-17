#include <math.h>
#define __IO volatile

void SystemInit (void);
void nvic_IntDisble(u8 IRQn);
void nvic_intEnable(u8 IRQn);
void set_system_clock_to_25Mhz (void);
void set_system_clock_to_72Mhz(void);
void Delay ( __IO u32 T );
void enable_TIM3_delay (void);
void ADC1_configure();

void ADC1_configure(){
	RCC->APB2ENR |= 1UL<<2 ; // Hab. GPIOA
	GPIOA->CRL &= ~(0x0F << 4); // Define A1 Input Analogic
	
	RCC->APB2ENR |= (1<<9); // Bit 9 ADC1EN: ADC 1 
									// interface clock enable

	RCC->CFGR	|= (0b10 << 14); // Bits 15:14 ADCPRE:
										  // ADC prescaler 10:
										  //    PCLK2 divided by 6
	// Enable End of Conversion (EOC) interrupt
	ADC1->CR1 |= (1 << 5);

	// One conversion
	ADC1->SQR1 = 0x00000000;

	// Choose the analog channel to read
	// Since we want channel 10 to be the first
	//   conversion we write 10 to SQ1 bits (3:0)
	//   which is in SQR3 register. For multiple conversions
	//   keep writing the channel numbers to SQx bits.
	 ADC1->SQR3 = (10 << 0);

	// Set up software trigger to start conversion
	ADC1->CR2 |= (7 << 17);  // Select SWSTART as trigger
	ADC1->CR2 |= (1 << 20);  // Enable external trigger

	// Enable continuous conversion
	ADC1->CR2 |= (1 << 1);

	// enable_interrupt(ADC1_2_IRQn);

	// Enable A/D conversion
	ADC1->CR2 |= (1 << 0);

	// Calibration reset and start
	//    Optional for better accuracy.
	ADC1->CR2 |= (1 << 3);
	while((ADC1->CR2 & (1 << 3)));
	ADC1->CR2 |= (1 << 2);
	while((ADC1->CR2 & (1 << 2)));

	// Start conversion with software trigger
	ADC1->CR2 |= (1<<22);
	//
	// Leitura  ADC1->DR  -> Data Register
}

void set_system_clock_to_72Mhz(void)
{
   // Necessary wait states for Flash for high speeds
   FLASH->ACR = 0x12;
   // Enable HSE
   RCC->CR |= (1 << 16);
   // Wait untill HSE settles down
   while (!(RCC->CR & (1 << 17)));
   // Set PREDIV2 division factor to 5
   RCC->CFGR2 |= (0b0100 << 4);
   // Set PLL2 multiplication factor to 8
   RCC->CFGR2 |= (0b0110 << 8);
   // Enable PLL2
   RCC->CR |= (1 << 26);
   // Wait untill PLL2 settles down
	// while (!(RCC->CR & (1 << 27)));
   // Set PLL2 as PREDIV1 clock source
   RCC->CFGR2 |= (1 << 16);
   // Set PREDIV1 division factor to 5
   RCC->CFGR2 |= (0b0100 << 0);
   // Select Prediv1 as PLL source
   RCC->CFGR |= (1 << 16);
   // Set PLL1 multiplication factor to 9
   RCC->CFGR |= (0b0111 << 18);
   // Set APB1 to 36MHz
   RCC->CFGR |= 1 << 10;
   // Enable PLL
   RCC->CR |= (1 << 24);
   // Wait untill PLL settles down
   while (!(RCC->CR & (1 << 25)));
   // Finally, choose PLL as the system clock
   RCC->CFGR |= (0b10 << 0);
} 

void Delay ( __IO u32 T ) { // T=1 ;  868µs -> T=1 ~ 526µs
  for ( T ; T > 0 ; T-- ){
   TIM3->EGR |= ( 1 << 0 );
   while ( TIM3->CNT < 5 ) ; /* 5 * 10-- Configurar para IR */
  }
}

void enable_TIM3_delay (void) {
   RCC->APB1ENR |= 1<<1 ;
   TIM3->CR1 = 0x0000;
   //TIM3->ARR = 21699;
   TIM3->ARR = 250-1; // 10µs
   // para obter o período de 868µs
/*https://www.vishay.com/docs/80071/dataform.pdf PAG 2
 * https://controllerstech.com/ir-remote-with-stm32/
 * Per=(1+ARR)/Clock_system */
   TIM3->CR1 |= 1<<0;
}

void enable_interrupt(IRQn_Type IRQn)
{
   NVIC->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}  
   
/* 
 * Disable given interrupt
 */
void disable_interrupt(IRQn_Type IRQn)
{
   NVIC->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/* Exemplo: 
 		    float num = 123.456789;
			 char str[20];
			 // num -> ponto flutuante
			 // str -> Var String 
			 // 4 -> Precisão do de casas decimais
			 floatToString(num, str, 4);
 
 * */
void floatToString(float num, char *str, int precision) {
    // Lida com números negativos
    if (num < 0) {
        num = -num;
        *str++ = '-';
    }

    // Extrai a parte inteira do número
    int inteira = (int)num;
    intToStr(inteira, str);
    while (*str != '\0') {
        str++;
    }

    // Adiciona o ponto decimal
    *str++ = '.';

    // Calcula e adiciona a parte fracionária
    float fracionaria = num - inteira;
    for (int i = 0; i < precision; i++) {
        fracionaria *= 10;
        int digito = (int)fracionaria;
        *str++ = '0' + digito;
        fracionaria -= digito;
    }

    // Adiciona o terminador nulo
    *str = '\0';
}

void intToStr(int num, char *str) {
    // Converte um número inteiro para uma string
    if (num == 0) {
        *str++ = '0';
        *str = '\0';
        return;
    }

    int len = 0;
    int temp = num;
    while (temp > 0) {
        temp /= 10;
        len++;
    }

    str += len;
    *str = '\0';

    while (num > 0) {
        int digito = num % 10;
        *(--str) = '0' + digito;
        num /= 10;
    }
}


/* Definições de bits "CFG" e "Mode" da configuração das GPIOs.
 *		   Pin_Hardw	
 *   CFG     MODE    H	 L   
 * |31|30|  |29|28| 15 ; 7		
 * |27|26|  |25|24| 14 ; 6	
 * |23|22|  |21|20| 13 ; 5
 * |19|18|  |17|16| 12 ; 4
 * |15|14|  |13|12| 11 ; 3
 * |11|10|  | 9| 8| 10 ; 2
 * | 7| 6|  | 5| 4|  9 ; 1
 * | 3| 2|  | 1| 0|  8 ; 0
 *
 *
 * __________________________________________
 * HexaCode -> 50MHz			↓ - PxODR
 * 0x03 | 0| 0|  |01..11| |x| <- Out Push-pull
 * 0x07 | 0| 1|  |01..11| |x| <- Out Open-drain
 * 0x0B | 1| 0|  |01..11| |x| <- Alt Out *pull
 * 0x0B | 1| 0|  |01..11| |x| <- Alt Out *drain
 * __________________________________________
 * 0x00 | 0| 0|  |  00  | |x| -< Input Analogig
 * 0x04 | 0| 1|  |  00  | |x| -< Input Floating  
 * 0x08 | 1| 0|  |  00  | |1| -< Input Pull-up
 * 0x0B | 1| 0|  |  00  | |0| -< Input Pull-down
 * __________________________________________
 * */

/* tabela_2 -> Configuração do Registrador AFIO->EXTICRx
 *     EXTI[0..15] 3    2    1    0 
 *
 *     Shift     <<12  <<8  <<4  <<0 	
 *	              xxxx xxxx xxxx xxxx 
 * GPIO[a..g] pin  3    2    1    0   	AFIO_EXTI_CR_1
 * GPIO[a..g] pin  7    6    5    4  	AFIO_EXTI_CR_2 
 * GPIO[a..g] pin  11   10   9    8  	AFIO_EXTI_CR_3 
 * GPIO[a..g] pin  15   14   13   12  	AFIO_EXTI_CR_4 
 *
 *Hex  xxxx
 *0x00 0000: PA[x] pin 
 *0x01 0001: PB[x] pin 
 *0x02 0010: PC[x] pin 
 *0x03 0011: PD[x] pin 
 *0x04 0100: PE[x] pin 
 *0x05 0101: PF[x] pin 
 *0x06 0110: PG[x] pin
 *
 * */

/* link_1 https://developer.arm.com/documentation/dui0662/b/Cortex-M0--Peripherals/Nested-Vectored-Interrupt-Controller/Interrupt-Set-Enable-Register
 *
 *
 *
 * https://github.com/98zam98/arm_nvic_driver/blob/main/hardware_arm_nvic_driver.h
 */

/*
 * Neste código, fora resolvido o problema com a interrupção
 * acredito que definitivamente usando o projeto:
 * "stm32f1-bare-metal/ext_int/ext_int.c"
 *		Usando uma estrutura com as definições de  vetores.
 *		Fora usado também, o manual do programador:
 * 
 * dm00046982-stm32-cortex-m4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf 
 */
