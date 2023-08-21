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
 * */
//
// FreeRTOS: 

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// #include <string.h>
#include <stdlib.h>

#define __delay_ms( TEMPO ) vTaskDelay(pdMS_TO_TICKS( TEMPO ))
#define __delay( TEMPO ) vTaskDelay( TEMPO )

#include "stm32f10x.h"
#include "Me.h"

// LCD 16x2
#include "myLcd.h"
// #include "Serial.h"
#include "PWM.h"


#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16

static QueueHandle_t fila_ADC; 
static QueueHandle_t LCD_Flag_queue; 

static void task1(void *args __attribute((unused))) {
	/* [2---]
	 *
	 * Tarefa dedicada em mostrar informações no LCD 16x2.
	 * Informações de temperatura, ficando aberto para mais
	 * funcionalidades
	 * 
	 * */

	RCC->APB2ENR |= (1<<2);
	GPIOA->CRL	 |= 0x33333300; // Hab PA(2-7) out PP

	set_bit(IO,RS); 	set_bit(IO,RS);
	set_bit(IO,EN); 	set_bit(IO,D4);
	set_bit(IO,D5); 	set_bit(IO,D6);
	set_bit(IO,D7);   disp_init();

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
	/* [1---]
	 *
	 * Tarefa dedicada a leitura do sensor de temperatura e
	 * humidade do ar.
	 *
	 * O valor amostrado deve ser transmitido a cada 500ms
	 * usando área de memória compartilhada. -> [2---]
	 *
	 * */

	while(1){
		
	}
}

static void task4(void *args __attribute ((unused))){
	/* [3---]
	 * Tarefa dedicada a leitura de botões para controlar o
	 * PWM -> ajuste de contraste.
	 * */
	while(1){
		
	}
}


int main(void) {
	fila_ADC = xQueueCreate(5 /* Tamanho máximo da fila. */,
								sizeof(u32) /* Largura máxima da fila. */);
	LCD_Flag_queue = xQueueCreate(2, sizeof(u8));
	
	set_system_clock_to_72Mhz();

	xTaskCreate(task1,"LCD_16x2", 100 ,NULL, 4 ,NULL);
	xTaskCreate(task2, "LED_13", 100 , NULL, 4 , NULL);
	xTaskCreate(task3, "USArt", 100 , NULL, 4 , NULL);
	xTaskCreate(task4, "Contrast", 100 , NULL, 4 , NULL);

	vTaskStartScheduler(); // inicia o escalonador de
								  // processos.
	while(1);
	return 0;
}

/*==========================================================*/
/*==========================================================*/
/*==========================================================*/
