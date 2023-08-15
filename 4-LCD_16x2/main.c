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
 * */

// FreeRTOS: 
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define __delay_ms( TEMPO ) vTaskDelay(pdMS_TO_TICKS( TEMPO ))
#define __delay( TEMPO ) vTaskDelay( TEMPO )

// LCD 16x2
#include "stm32f10x.h"
#include "Me.h"
#include "myLcd.h"

static QueueHandle_t xQueue; 

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

  disp_text("abcdABCDEF012",0,0);
  u8 cont = (char) 1;
	while(1){

		cont &= 0xffff;
		cont ++;
		disp_text("CONTADOR: ", 1, 0);
		disp_number( cont, 1, 11);
		__delay_ms(1000);
		
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
	while(1){
	}
}

int main(void) {
	xQueue = xQueueCreate(5 /* Tamanho máximo da fila. */,
								sizeof(u32) /* Largura máxima da fila. */);

	set_system_clock_to_72Mhz();
//	enable_TIM3_delay();

	xTaskCreate(task1,"LCD_16x2", 100 ,NULL, 4 ,NULL);
	xTaskCreate(task2, "LED_13", 100 , NULL, 4 , NULL);
	//xTaskCreate(task3, "3", 100 , NULL, 4 , NULL);

	vTaskStartScheduler(); // inicia o escalonador de
								  // processos.
	while(1);
	return 0;
}
