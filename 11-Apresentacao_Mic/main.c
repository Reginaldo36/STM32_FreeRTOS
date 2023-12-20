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

const float ADC_Const = (float) 3 * 10 / 4095;
#define OFFSET_TEMP 0
#define VEL_ANIMACAO 15


// Codigos IR - Botoes:

#define SM_horario	 0x7f07fe30
#define SM_anti_horario 0x7f3ffc30 

#define SM_180				0x7f0ffe30
#define SM_90				0x7f17fe30
#define SM_0				0x7f47fe30

#define LCD_b0				0x7f27fe30 // Botão info 
#define LCD_b1				0x7f2ffe30 // botão Mute

// Quantidade max de elementos fila:
#define ELEMENTOS_FILA_LCD 1
#define ELEMENTOS_FILA_ServoM 1
#define ELEMENTOS_FILA_disp_menu 1
#define ELEMENTOS_FILA_Mic_Reader 2

static QueueHandle_t LM32_LCD_queue; 
static QueueHandle_t Display_menu_select_queue; 
static QueueHandle_t SERVO_queue; 
// static QueueHandle_t Mic_Reader_queue; 

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
	char * LCD_string = "---Reginaldo---";

	u32 code_ir_menu_receive;
	u32 code_ir_menu_select =   LCD_b0;

	while(1){

		if (xQueueReceive(LM32_LCD_queue, 
					&msg_buf_rev, pdMS_TO_TICKS(10)))
			buffer_de_fila = msg_buf_rev;

		// Verifica se houve atualização no botão ou 
		// recebeu dados do infravermelho

		if (xQueueReceive(Display_menu_select_queue, 
					&code_ir_menu_receive, pdMS_TO_TICKS(10))){

			if (code_ir_menu_receive == LCD_b0 
					|| code_ir_menu_receive == LCD_b1){

				code_ir_menu_select = code_ir_menu_receive; 		
				disp_clear();
			}
		}
		// code_ir_menu_select = LCD_b0; 

		switch (code_ir_menu_select) {
			case LCD_b0: 

				floatToString((float) (TIM4->CCR4 - 500) / 11.11, Float_point, 1);

				disp_text("Angulo:", 0, 0); 
				disp_text(Float_point, 0, 10); 

				floatToString((float) buffer_de_fila * ADC_Const - OFFSET_TEMP , Float_point, 2);
				disp_text(Float_point, 1, 10);
				disp_text("Temp. C:" , 1, 0); 
				// disp_number( buffer_de_fila , 1, 0); 

				break; 

			case LCD_b1: 

				// disp_text("----------------", 0 ,0);
				disp_text("================", 1, 0);
				for (u8 i=0 ; i<16 ; i++){
					disp_text(LCD_string, 0 ,0);
					disp_text("-", 1 , 15 - i);

					// __delay_ms(100);
				}

				code_ir_menu_select = LCD_b0;
				disp_clear();

				break;

			default: 
				continue; 
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

		if(uxQueueMessagesWaiting(LM32_LCD_queue) <= ELEMENTOS_FILA_LCD){
			xQueueSend(LM32_LCD_queue, &Valor_ADC_normalizado, pdMS_TO_TICKS(10));
		}
		__delay_ms(500); // Bloqueia por 500ms

	}
}

static void task4(void *args __attribute ((unused))){

	PWM_PB9_config();
	u32 code_ir = 0; 
	u16 atua_PWM = 1500;

	char word_IR[8];

	TIM4->CCR4 = atua_PWM ; 
	while(1){

		if (xQueueReceive(SERVO_queue, &code_ir, pdMS_TO_TICKS(10))) {
			if (SM_anti_horario == code_ir)
				atua_PWM -=111;

			else if (SM_horario == code_ir)
				atua_PWM +=111;

			else if (SM_90 == code_ir)
				atua_PWM =1500;

			else if (SM_0 == code_ir)
				atua_PWM =500;

			else if (SM_180 == code_ir)
				atua_PWM =2500;

			// USARTSend("\r\nNa Função: 0x");
			// itoa (code_ir, word_IR, 16);
			// USARTSend(word_IR);


				// atua_PWM = (atua_PWM >= 1000) ? atua_PWM : 1000 ;
				// atua_PWM = (atua_PWM <= 2000) ? atua_PWM : 2000 ;

			atua_PWM = (!(atua_PWM <= 500)) ? atua_PWM : 500 ;
			atua_PWM = (!(atua_PWM >= 2500)) ? atua_PWM : 2500 ;

			 // USARTSend(", PWM Var: "); 
			 // itoa(atua_PWM, word_IR, 10);
			 // USARTSend(word_IR);



			TIM4->CCR4 = atua_PWM ; 
		}
	}
}

static void task5(void *args __attribute ((unused))){

	USART_init();
	IR_Init();
	// char word_IR[8];
	u32 code_ir;
	u32 anterior;

	while(1){

		code_ir = 0xFFFFFFFF;
		code_ir = IR_Read();


		if (code_ir == 0xffffffd0){
			if( uxQueueMessagesWaiting( SERVO_queue ) <= ELEMENTOS_FILA_ServoM ){
				xQueueSend(SERVO_queue, &anterior, pdMS_TO_TICKS(10));
			}
		}

		else {
			if( uxQueueMessagesWaiting( SERVO_queue ) <= ELEMENTOS_FILA_ServoM ){
				xQueueSend(SERVO_queue, &code_ir, pdMS_TO_TICKS(10));
			}
			anterior = code_ir; 
		}

			if( uxQueueMessagesWaiting( Display_menu_select_queue ) <= ELEMENTOS_FILA_disp_menu ){
				xQueueSend(Display_menu_select_queue, &code_ir, pdMS_TO_TICKS(10));

		}

	}
}

void task6_MIC(void *args __attribute ((unused))){

	ADC8_Configuration(); // PB0  One conversion mode

	u32 ADC8_data = 0;
	char str_ADC[2]; 

	u32 conta_ciclo = 0;
	u8 inicio_fim = 0 ; 
	GPIOB->CRH |= (0x03 << (4*4)) | (0x03 << (4*5)); 
	GPIOB->ODR ^= 1<<12;

	while(1){

		// ADC8_data = ADC8_GetValue();
		ADC8_data = ADC1->DR;
		
		itoa(ADC8_data, str_ADC, 10);
		// itoa(conta_ciclo, str_ADC, 10);

		 USARTSend(str_ADC);
		 USARTSend("\n\r"); 


		// Verifica se a fila está cheia
		// if( uxQueueMessagesWaiting(Mic_Reader_queue) <= ELEMENTOS_FILA_Mic_Reader ){ 
		// xQueueSend(Mic_Reader_queue, &ADC8_data, pdMS_TO_TICKS(10));
		// }

		// __delay(10);

#define VALOR_ADC_COMANDO 3600 // Nível da entrada
#define comando_1 15 // 500 ms
#define TMOUT 2500 // valor co conta_ciclo
#define PB13_set  (GPIOB->ODR &= ~(1<<13))
#define PB13_reset	(GPIOB->ODR |= 1<<13) 

#define PB12_set  (GPIOB->ODR &= ~(1<<12))
#define PB12_reset	(GPIOB->ODR |= 1<<12) 
								 
		if (ADC8_data  > VALOR_ADC_COMANDO ){

			if (inicio_fim == 1 && conta_ciclo >= comando_1){
				inicio_fim = 0 ; 
				// executar o comando
				PB12_set;
			}
			else if (inicio_fim == 1 && conta_ciclo < comando_1){
				inicio_fim = 0;
				// executar o comando

				PB12_reset;
			}
			else {
				inicio_fim = 1;
				conta_ciclo = 0;
			}
		} 


		if (inicio_fim == 1 ){
			conta_ciclo ++; 
			PB13_set ; 
		}
		else 
			PB13_reset;

		if (conta_ciclo >= TMOUT){
			conta_ciclo = 0; 
			inicio_fim = 0 ;
		}
		// __delay(3);
	}
}


int main(void) {

	//											↓ O tamanho da fila é x - 1 !!
	LM32_LCD_queue = xQueueCreate(ELEMENTOS_FILA_LCD, sizeof(u32));
	SERVO_queue  = xQueueCreate(ELEMENTOS_FILA_ServoM, sizeof(u32)); 
	Display_menu_select_queue  = xQueueCreate(ELEMENTOS_FILA_disp_menu, sizeof(u32)); 

	set_system_clock_to_72Mhz();

	xTaskCreate(task1,"LCD_16x2", 100 ,NULL, 4 ,NULL);
	xTaskCreate(task2, "LED_13", 100 , NULL, 4, NULL);
	xTaskCreate(task3, "LM35_read", 100 , NULL, 4 , NULL);
	xTaskCreate(task4, "ServoMec", 100 , NULL, 4 , NULL);
	xTaskCreate(task5, "IR_Rev", 100 , NULL, 4 , NULL);
	xTaskCreate(task6_MIC, "MIC_READER", 200 , NULL, 4 , NULL);
	// xTaskCreate(task7_M_acao, "MIC_command", 100 , NULL, 4 , NULL);

	vTaskStartScheduler(); 

	while(1);
	return 0;
}
