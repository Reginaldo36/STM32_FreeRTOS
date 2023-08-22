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
#include "Me.h"

// LCD 16x2
#include "myLcd.h"
#include "Serial.h"
#include "PWM.h"
#include "dht11.h"

#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16

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
	GPIOA->CRL	 |= 0x33333300; // Hab PA(2-7) out PP

	set_bit(IO,RS); 	set_bit(IO,RS);
	set_bit(IO,EN); 	set_bit(IO,D4);
	set_bit(IO,D5); 	set_bit(IO,D6);
	set_bit(IO,D7);   disp_init();

  disp_cmd(0x0C);
/*============================================================*/

  disp_text("Temperatura: ",0,0);

  char msg_buf_rev[16];
	while(1){
		//
		// testar para ver se recebe !!!
		__delay_ms(1000);
		xQueueReceive(LCD_DHT_queue, &msg_buf_rev, pdMS_TO_TICKS(1));

		disp_text(msg_buf_rev, 1, 0);
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

	struct DHT11_Dev dev;
	DHT11_init(&dev, GPIOB, GPIO_Pin_6);

	char msg_buf_env[16];
	char  Humidade[4];
	char 	Temperatura[4];

	while(1){
		__delay_ms(500); // << Rever isso !!

		int leitura_DHT = DHT11_read( &dev );

		if(leitura_DHT == DHT11_SUCCESS) {
			
			// Converte em string 
			itoa(dev.temparature, Temperatura , DEC);
			itoa(dev.humidity, Humidade, DEC);

			// Concatena  a variável "msg_buf_env"
			strcpy(msg_buf_env, "T: ");
			strcat(msg_buf_env, Temperatura );
			strcat(msg_buf_env, "H: " );
			strcat(msg_buf_env, Humidade );
		}

		else if(leitura_DHT == DHT11_ERROR_CHECKSUM ) 
			strcpy(msg_buf_env, "ERR_checksum"); 
		
		else 
			strcpy(msg_buf_env, "TIMOUT"); 

		xQueueSend(LCD_DHT_queue, &msg_buf_env, 0);
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

	LCD_DHT_queue = xQueueCreate(2, sizeof(char));
	
	set_system_clock_to_72Mhz();

	xTaskCreate(task1,"LCD_16x2", 100 ,NULL, 4 ,NULL);
	xTaskCreate(task2, "LED_13", 100 , NULL, 4 , NULL);
	xTaskCreate(task3, "DHT_read", 100 , NULL, 4 , NULL);
	xTaskCreate(task4, "Contrast", 100 , NULL, 4 , NULL);

	vTaskStartScheduler(); 

	while(1);
	return 0;
}

/*==========================================================*/
