#include "stm32f10x.h"
#include "Me.h"

// FreeRTOS: 
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// LCD 16x2
#include "delay.h"
#include "lcd16x2.h"

#define __delay_ms( TEMPO ) vTaskDelay(pdMS_TO_TICKS( TEMPO ))
#define __delay( TEMPO ) vTaskDelay( TEMPO )

static QueueHandle_t xQueue; 

static void task1(void *args __attribute((unused))) {
	while(1){
		lcd16x2_put_custom_char(0, 0, 0);
		lcd16x2_puts(" Battery Low");
		DelayMs(500);
		// Clear display
		lcd16x2_clrscr();
		DelayMs(500);
	}
}

static void task2(void *args __attribute ((unused))){
	RCC->APB2ENR |= (1<<4); // En GPIOC
	GPIOC->CRH	|= (0b11<< 20); // PC13 out PP 

	while(1){
		GPIOC->ODR	^= (1<<13);
		__delay(50);
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
// display Init 

	lcd16x2_init(LCD16X2_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);

	xTaskCreate(task1,"1", 100 ,NULL, 4 ,NULL);
	xTaskCreate(task2, "2", 100 , NULL, 4 , NULL);
	xTaskCreate(task3, "3", 100 , NULL, 4 , NULL);

	vTaskStartScheduler(); // inicia o escalonador de
								  // processos.
	while(1);
	return 0;
}
