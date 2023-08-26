#include "stm32f10x.h"

// Define o pino de entrada do receptor IR
#define IR_PIN GPIO_Pin_8
#define IR_PORT GPIOB

// Função para configurar o GPIO para o pino de entrada IR
void IR_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = IR_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(IR_PORT, &GPIO_InitStructure);

}

// Função para receber o código NEC IR
uint32_t IR_Read(void) {
  uint32_t code = 0;
  uint32_t bit = 0;
  uint32_t data = 0;

  // Aguarda o sinal de início (pulso LOW)
  while (GPIO_ReadInputDataBit(IR_PORT, IR_PIN) != 0) {}
  
  // Aguarda o sinal de início (pulso HIGH)
  while (GPIO_ReadInputDataBit(IR_PORT, IR_PIN) == 0) {}
  
  // Aguarda o sinal de 9 ms (pulso LOW)
  while (GPIO_ReadInputDataBit(IR_PORT, IR_PIN) != 0) {}

  // Recebe o código NEC IR
  for (bit = 0; bit < 32; bit++) {
    // Aguarda o próximo bit
    // while (GPIO_ReadInputDataBit(IR_PORT, IR_PIN) == 0) {}
    
    // Aguarda o próximo espaço
    // while (GPIO_ReadInputDataBit(IR_PORT, IR_PIN) != 0) {}
	 __delay(200);

    data = GPIO_ReadInputDataBit(IR_PORT, IR_PIN);
    
    // Se o espaço tiver uma duração maior que 1.2 ms o bit é 1, caso contrário é 0
    if (data > 0) {
      code |= (1 << bit);
    }
	// Aguarda o próximo espaço
	 __delay(300);
  }

  return code;
}

