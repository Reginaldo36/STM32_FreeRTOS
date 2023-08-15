#include "FreeRTOS.h"
#include "task.h"
#include "Me.h"

extern void vApplicationStackOverflowHook( // Protótipo da
														  // função abaixo
	xTaskHandle *pxTask,
	signed portCHAR *pcTaskName);


void vApplicationStackOverflowHook(   xTaskHandle *pxTask __attribute((unused)), 
	 	signed portCHAR *pcTaskName __attribute((unused)))  {
	for(;;);	// Função não muito útil, usado quando há um
				// estouro de pilha, servindo como alerta.
}

static void task1(void *args __attribute((unused))) {
	for (;;) {
		GPIOC->ODR ^= (1<<13);
		vTaskDelay(pdMS_TO_TICKS(500)); /* A macro pdMS_TO_TICKS() 
														converte milissegundos em ticks */
	}
}

int main(void) {

	// rcc_clock_setup_in_hse_8mhz_out_72mhz(); // For "blue pill"
	enable_TIM3_delay();

	RCC->APB2ENR |= 0b1<<4 ;
	GPIOC->CRH |= 0b11 << 20;
	Delay(2000);
	GPIOC->ODR ^= (1<<13);
	Delay(2000);

	set_system_clock_to_72Mhz();
	RCC->APB2ENR |= 0b1<<4 ;
	GPIOC->CRH |= 0b11 << 20;

	xTaskCreate(task1,"LED",100,NULL,configMAX_PRIORITIES-1,NULL);
	vTaskStartScheduler();

	while(1);
	return 0;
}

// https://embarcados.com.br/blue-pill-stm32f103c8t6-introducao-ao-freertos/
