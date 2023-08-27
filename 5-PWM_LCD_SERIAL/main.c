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
// #include "PWM.h"
// #include "dht11.h"
#include "DHT11.h"

#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16



#define CKSUM_FAIL_DHT11 -2 	
#define TMOUT_DHT11 -1

static QueueHandle_t LCD_DHT_queue; 
static void task1(void *args __attribute((unused))) {

	/* [2---]
	 *
	 * Tarefa dedicada em mostrar informações no LCD 16x2.
	 * Informações de temperatura, ficando aberto para mais
	 * funcionalidades
	 * 
	 * */

	RCC->APB2ENR |= (1<<2);
	GPIOA->CRL	 = 0x33333300; // Hab PA(2-7) out PP

	set_bit(IO,RS);   set_bit(IO,RS);
	set_bit(IO,EN);   set_bit(IO,D4);
	set_bit(IO,D5);   set_bit(IO,D6);
	set_bit(IO,D7);   disp_init();

	disp_cmd(0x0C);


	/*
	disp_text("Iniciando 12345",0,0);
	disp_text("0123456789ABCDEF: ",1,0);
	__delay_ms(5000);
*/

	disp_text("                ", 0, 0);
	disp_text("                ", 1, 0);
	__delay_ms(500); 

	int msg_buf_rev;

	while(1){
		xQueueReceive(LCD_DHT_queue, &msg_buf_rev, pdMS_TO_TICKS(1));

		disp_number(msg_buf_rev & (0XF << 0), 0,0);
		disp_number(msg_buf_rev & (0XF << 8), 1,0);

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
	 * Tarefa dedicada a leitura do sensor de temperatura e
	 * humidade do ar.
	 *
	 * O valor amostrado deve ser transmitido a cada 500ms
	 * usando área de memória compartilhada. -> [2---]
	 *
	 * Problemas: A configuração do TIMER está errada nas
	 * linhas: 15 e 93 do arquivo dht11.c !!!!!
	 *
	 * Solução: 
	 *		Comparar com a biblioteca funcional do outro
	 *		projeto!!!
	 *
	 *
	 *
	 * */

	USART_init();
	DHT11_start();

	u16 dados_leitura=0xFFFF;

	int j = 0; 

	j++;

	char t[4], h[4], env[20];

	USARTSend("\r\nINICIANDO....\r\n");

	while(1){

		dados_leitura = DHT11_read();
	USARTSend("\r\nINICIANDO....\r\n");

		itoa(dados_leitura & (0xF << 8), t, 10);
		itoa(dados_leitura & (0xF << 0), h, 10);

		
		strcpy(env, "\n\rTemperatura: ");
		strcat(env, t);
		USARTSend(env);

		strcpy(env, "\n\rHumidade: ");
		strcat(env, h);
		USARTSend(env);


		__delay_ms(500);
		xQueueSend(LCD_DHT_queue, &dados_leitura, 0);

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

	LCD_DHT_queue = xQueueCreate(17, sizeof(int));
	
	set_system_clock_to_72Mhz();

	xTaskCreate(task1,"LCD_16x2", 100 ,NULL, 3 ,NULL);
	xTaskCreate(task2, "LED_13", 100 , NULL, 3, NULL);
	xTaskCreate(task3, "DHT_read", 300 , NULL, 4 , NULL);
	// xTaskCreate(task4, "Contrast", 100 , NULL, 4 , NULL);

	vTaskStartScheduler(); 

	while(1);
	return 0;
}

/*==========================================================*/
