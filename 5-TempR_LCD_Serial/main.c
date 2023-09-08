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

// FreeRTOS: 

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <string.h>
#include <stdlib.h>

#define __delay_ms( TEMPO ) vTaskDelay(pdMS_TO_TICKS( TEMPO ))
#define __delay( TEMPO ) vTaskDelay( TEMPO )

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "Me.h"

// LCD 16x2
#include "myLcd.h"
#include "Serial.h"
#include "adc.h"

#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16

// Vadc * 3300 (mV) / 4096 (12 bits) 
#define ADC_Const 0.8056


void LCD16x2_init(){

	RCC->APB2ENR |= (1<<2);
	GPIOA->CRL	 = 0x33333300; // Hab PA(2-7) out PP

	set_bit(IO,RS);   set_bit(IO,RS);
	set_bit(IO,EN);   set_bit(IO,D4);
	set_bit(IO,D5);   set_bit(IO,D6);
	set_bit(IO,D7);   disp_init();
	disp_cmd(0x0C);

// Animação no visor:
	disp_text("----------------", 0 ,0);
	disp_text("0123456789abcdef", 1, 0);

	for (u8 i=0 ; i<16 ; i++){
		__delay_ms(200);
		disp_text("X", 0 ,i);
		disp_text("#", 1 , 16 - i);
	}
	__delay_ms(500); 
	disp_clear();
}

static QueueHandle_t LCD_Show_queue; 
static void task1(void *args __attribute((unused))) {

	/* [2---]
	 *
	 * Tarefa dedicada em mostrar informações no LCD 16x2.
	 * Informações de temperatura, ficando aberto para mais
	 * funcionalidades
	 * 
	 * */

	LCD16x2_init();
	int msg_buf_rev;

	while(1){
		xQueueReceive(LCD_Show_queue, &msg_buf_rev, pdMS_TO_TICKS(1));

		disp_text("Temp (C): ", 0, 0);
		disp_number(msg_buf_rev * ADC_Const , 0,16-5);

		__delay_ms(500);
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


 void task3(void *args __attribute ((unused))){
	/* [1---]
	 *
	 * Tarefa dedicada a leitura do sensor de temperatura
	 * usando ADC9 -> PB1
	 *
	 * O valor amostrado deve ser transmitido a cada 500ms
	 * usando área de memória compartilhada. -> [2---]
	 *
	 * */

	ADC9_Configuration();
	u32 Valor_ADC_normalizado; 

	while(1){

		Valor_ADC_normalizado = ADC2->DR;

		xQueueSend(LCD_Show_queue, &Valor_ADC_normalizado, 0);
		__delay_ms(500);
		
	}
}

int main(void) {

	LCD_Show_queue = xQueueCreate(2, sizeof(u32));
	
	set_system_clock_to_72Mhz();

	xTaskCreate(task1,"LCD_16x2", 100 ,NULL, 2 ,NULL);
	xTaskCreate(task2, "LED_13", 100 , NULL, 2, NULL);
	xTaskCreate(task3, "LM35_read", 100 , NULL, 2 , NULL);

	vTaskStartScheduler(); 

	while(1);
	return 0;
}

/*==========================================================*/
