/*
 * Projeto LCD
 *		Modificações necessárias: 
 *		  FreeRTOSConfig:
 *			 TICK_RATE_HZ definido em (100000)Hz -> 1µs
 *
 *	Clock do sistema: 
 *		72Mhz
 *	
 *	Lib construída para Arduíno UNO:
 *		myLcd.h
 *			#define delayMicroseconds(X)   vTaskDelay(X)
 *			#define delay(X)  vTaskDelay(X)
 *
 * Pinos do LCD 16x2:
 *		GPIOA
 *			RS -> P2		| EN -> P3
 *			D4	-> P4		| D5 -> P5
 *			D6 -> P6		| D7 -> P7
 * */
//
// FreeRTOS: 
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <string.h>
#include <stdlib.h>

#define __delay_ms( TEMPO ) vTaskDelay(pdMS_TO_TICKS( TEMPO ))
#define __delay( TEMPO ) vTaskDelay( TEMPO )

// LCD 16x2
#include "stm32f10x.h"
#include "Me.h"
#include "myLcd.h"
#include "Serial.h"

//#include "adc.h"

#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16

static QueueHandle_t fila_ADC; 

static void task1(void *args __attribute((unused))) {
	RCC->APB2ENR |= (1<<2);
	GPIOA->CRL	 = 0x33333300; // Hab PA(2-7) out PP

	set_bit(IO,RS);
	set_bit(IO,RS);
	set_bit(IO,EN);
	set_bit(IO,D4);
	set_bit(IO,D5);
	set_bit(IO,D6);
	set_bit(IO,D7);

  disp_init();
  disp_cmd(0x0C);

  disp_text("Teste De\\ !&(#@!%@",0,0);
  u8 cont = 0;
	while(1){

		cont ++;
		cont &= 0x00ff;
		disp_text("CONTADOR: ", 1, 0);
		disp_number( cont, 1, 11);
		__delay_ms(10);
		
	}
}

static void task2(void *args __attribute ((unused))){
	RCC->APB2ENR |= (1<<4); // En GPIOC
	GPIOC->CRH	|= (0b11<< 20); // PC13 out PP 

	while(1){
		GPIOC->ODR	^= (1<<13);
		__delay_ms(50);
	}
}

static void task3(void *args __attribute ((unused))){
	USART_init();

	u32 dados_recebidos = 0;

	char num_char[8]; // tamanho max de um numero é de 32 bits
	u8 status;
			

	while(1){
		status = xQueueReceive(fila_ADC,
			  	&dados_recebidos, 
				pdMS_TO_TICKS(1));

		if (status != NULL ){
			USARTSend("\r\nValor Lido: ");

			//	itoa numero -> String , Tipo output
			itoa((u32) dados_recebidos, num_char, HEX); // Base: 2 8 10 16

			USARTSend( num_char );
			__delay_ms(100);
		}
		else
			USARTSend("\r\nValor não amostrado ou transmitido!! \r\n");
	}
}

static void task4(void *args __attribute ((unused))){
	//ADC8_Init();
	
	u32 adc_dados_lidos = 0; 
	while(1){

		// adc_dados_lidos = ADC8_Read();
		adc_dados_lidos ++;

		xQueueSend( fila_ADC,
				&adc_dados_lidos, 
				0);

		__delay_ms(1);
	}
}

int main(void) {
	fila_ADC = xQueueCreate(5 /* Tamanho máximo da fila. */,
								sizeof(u32) /* Largura máxima da fila. */);

	set_system_clock_to_72Mhz();

	xTaskCreate(task1,"LCD_16x2", 100 ,NULL, 4 ,NULL);
	xTaskCreate(task2, "LED_13", 100 , NULL, 4 , NULL);
	xTaskCreate(task3, "USArt", 100 , NULL, 4 , NULL);
	xTaskCreate(task4, "ADC", 100 , NULL, 4 , NULL);

	vTaskStartScheduler(); // inicia o escalonador de
								  // processos.
	while(1);
	return 0;
}

/*==========================================================*/
/*==========================================================*/
/*==========================================================*/
