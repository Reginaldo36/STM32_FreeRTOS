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

#include <stdlib.h> // funçao itoa

#define __delay_ms( TEMPO ) vTaskDelay(pdMS_TO_TICKS( TEMPO )) // ms
#define __delay( TEMPO ) vTaskDelay( TEMPO ) // µs
#define TICKS_Q pdMS_TO_TICKS(10)

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "Me.h"

#include "myLcd.h"
#include "Serial.h"
#include "adc.h"
#include "infraRed.h"
#include "PWM.h"

#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16

// Vadc * 3.3 * 10 / 4096 (12 bits) 
#define ADC_Const 0.008056
#define VEL_ANIMACAO 15


// Codigos IR - Botoes:
#define PWM_No_2 0xf70ffe30
#define PWM_No_4 0xf80ffe30
#define PWM_No_5 0xf07ffe30
#define PWM_No_6 0xc26ffe30
#define PWM_No_9 0xc4affe30

u32 CONTROLE_IR[] = {0xf70ffe30 }; 

// Quantidade max de elementos fila:
#define ELEMENTOS_FILA_LCD 2
#define ELEMENTOS_FILA_ServoM 1
#define ELEMENTOS_FILA_PWM 1

static QueueHandle_t LCD_Show_queue; 
static QueueHandle_t SERVO_queue; 

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

static void task1(void *args __attribute((unused))) {

	/* [2---]
	 *
	 * Tarefa dedicada em mostrar informações no LCD 16x2.
	 * Informações de temperatura, ficando aberto para mais
	 * funcionalidades
	 * 
	 * */

	LCD16x2_init();
	RCC->APB2ENR |= 1<<3;
	GPIOB->CRH |= 0x4<<0; // Input Pull
	GPIOB->ODR |= 1<<8; // Pull-up
	int msg_buf_rev, buffer_de_fila = 0;

	char Float_point[6];
	u8 menu_View=1;

#define MAX_MENU 3

	u8 RD = 0;

	while(1){

		if (xQueueReceive(LCD_Show_queue, 
					&msg_buf_rev, pdMS_TO_TICKS(10)))
			buffer_de_fila = msg_buf_rev;

		while (RD > 12){ // força atualização do LCD :
			RD = 0;
			disp_clear();
		} RD ++;

		// Verifica se houve atualização no botão ou 
		// recebeu dados do infravermelho

		if ((GPIOB->IDR & (1<<8))){
			menu_View ++;

		if (menu_View > MAX_MENU )
			menu_View = 1;
		disp_clear();
		}

		if (menu_View == 1 ){
				disp_text("Temperatura - C", 0, 0); 
				floatToString((float) buffer_de_fila * ADC_Const, Float_point, 2);
				disp_text(Float_point, 1, 6);
		} 
		if (menu_View == 2 ){
			disp_text("----------------", 0 ,0);
			disp_text("================", 1, 0);

			for (u8 i=0 ; i<16 ; i++){
				__delay_ms(10);
				disp_text("=", 0 ,i);
				disp_text("-", 1 , 15 - i);
			}
		}

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

		if(uxQueueMessagesWaiting(LCD_Show_queue) <= ELEMENTOS_FILA_LCD){
				xQueueSend(LCD_Show_queue, &Valor_ADC_normalizado, pdMS_TO_TICKS(10));
		}
		__delay_ms(500); // Bloqueia por 500ms
		
	}
}

static void task4(void *args __attribute ((unused))){
	
	PWM_PB9_config();
	u32 code_ir = 0; 
	u16 atua_PWM = 1000;

	while(1){

		if (xQueueReceive(SERVO_queue, &code_ir, pdMS_TO_TICKS(10))) {
			if ( PWM_No_6 == code_ir )
				atua_PWM -=75;

			else if ( PWM_No_4 == code_ir )
				atua_PWM +=75;

			else if ( PWM_No_5 == code_ir )
				atua_PWM =1500;
		}
		
	atua_PWM = (atua_PWM >= 1000) ? atua_PWM : 1000 ;
	atua_PWM = (atua_PWM <= 2000) ? atua_PWM : 2000 ;

	TIM4->CCR4 = atua_PWM ; 
	}
}

static void task5(void *args __attribute ((unused))){

	USART_init();
	IR_Init();
	char word_IR[8];
	u32 code_ir;

	while(1){

		code_ir = 0xFFFFFFFF;
		code_ir = IR_Read();
		
		if (code_ir != (0xFFFFFF << 1)) {

			USARTSend("\r\nCodigo: 0x");
			itoa (code_ir, word_IR, 16);
			USARTSend(word_IR);
			
			if( uxQueueMessagesWaiting( SERVO_queue ) <= ELEMENTOS_FILA_ServoM ){
				xQueueSend(SERVO_queue, &code_ir, pdMS_TO_TICKS(10));
			}


			
		}
		__delay(250);

	}
}


int main(void) {

	//											↓ O tamanho da fila é x - 1 !!
	LCD_Show_queue = xQueueCreate(ELEMENTOS_FILA_LCD, sizeof(u32));
	SERVO_queue  = xQueueCreate(ELEMENTOS_FILA_ServoM, sizeof(u32)); 
	// PWM_data_queue = xQueueCreate(ELEMENTOS_FILA_PWM, sizeof(u32));
	
	set_system_clock_to_72Mhz();

	xTaskCreate(task1,"LCD_16x2", 100 ,NULL, 4 ,NULL);
	xTaskCreate(task2, "LED_13", 100 , NULL, 4, NULL);
	xTaskCreate(task3, "LM35_read", 100 , NULL, 4 , NULL);
	xTaskCreate(task4, "ServoMec", 100 , NULL, 4 , NULL);
	xTaskCreate(task5, "IR_Rev", 100 , NULL, 4 , NULL);

	vTaskStartScheduler(); 

	while(1);
	return 0;
}
