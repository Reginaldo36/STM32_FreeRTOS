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

// #include <string.h>

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

// Vadc * 3.3 * 10 / 4096 (12 bits) 
#define ADC_Const 0.008056
#define VEL_ANIMACAO 15


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
	disp_text("================", 1, 0);

	for (u8 i=0 ; i<16 ; i++){
		__delay_ms(VEL_ANIMACAO);
		disp_text("=", 0 ,i);
		disp_text("-", 1 , 16 - i);
	}
	__delay_ms(VEL_ANIMACAO); 
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
	 *		Modificações necessárias: 
	 *		  FreeRTOSConfig:
	 *			 TICK_RATE_HZ definido em (100000)Hz -> 1µs
	 *
	 *	Clock do sistema: 
	 *		72Mhz
	 * */

	LCD16x2_init();
	RCC->APB2ENR |= 1<<3;
	GPIOB->CRH |= 0x4<<0; // Input Pull
	GPIOB->ODR |= 1<<8; // Pull-up
	int msg_buf_rev, BUFF = 0;

	char Float_point[6];
	u8 ent_atual=1;

#define MAX_MENU 2

	while(1){
		if ((GPIOB->IDR & (1<<8))){
			ent_atual ++;

		if (ent_atual > MAX_MENU )
			ent_atual = 1;
		disp_clear();
		}

		if (xQueueReceive(LCD_Show_queue, 
					&msg_buf_rev, pdMS_TO_TICKS(1)))
			BUFF = msg_buf_rev;

		if (ent_atual == 1 ){
				disp_text("Temperatura - C", 0, 0); 

				floatToString(BUFF * ADC_Const, Float_point, 2);
				disp_text(Float_point, 1, 6);
		} 
		if (ent_atual == 2 ){
			disp_text("----------------", 0 ,0);
			disp_text("================", 1, 0);

			for (u8 i=0 ; i<16 ; i++){
				__delay_ms(10);
				disp_text("=", 0 ,i);
				disp_text("-", 1 , 16 - i);
			}
		}
	}
}

void LCD_Menu( u8 ent_atual ){
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

	xTaskCreate(task1,"LCD_16x2", 100 ,NULL, 4 ,NULL);
	xTaskCreate(task2, "LED_13", 100 , NULL, 4, NULL);
	xTaskCreate(task3, "LM35_read", 100 , NULL, 4 , NULL);

	vTaskStartScheduler(); 

	while(1);
	return 0;
}
