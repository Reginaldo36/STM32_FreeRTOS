#include "stm32f10x.h"
#include "Me.h"
/* #include "stm32f10x_adc.h" */

// FreeRTOS: 
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define __delay_ms( TEMPO ) vTaskDelay(pdMS_TO_TICKS( TEMPO ))
#define __delay( TEMPO ) vTaskDelay( TEMPO )


static QueueHandle_t xQueue; 

static void task1(void *args __attribute((unused))) {
	while(1){

	}
}

static void task2(void *args __attribute ((unused))){
	while(1){
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

	xTaskCreate(task1,"PWM_Control", 100 ,NULL, 4 ,NULL);
	xTaskCreate(task2, "ADC_Read", 200 , NULL, 4 , NULL);
	xTaskCreate(task3, "Led_C13", 100 , NULL, 4 , NULL);

	vTaskStartScheduler(); // inicia o escalonador de
								  // processos.
	while(1);
	return 0;
}
