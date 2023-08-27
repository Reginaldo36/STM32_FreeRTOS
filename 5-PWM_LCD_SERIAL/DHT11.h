#define DHT11_DATA_PIN GPIO_Pin_12
#define DHT11_GPIO GPIOA

#define DELAY_VALUE 1000    // Atraso necessário para a leitura do sensor

void DHT11_start(void) {
	RCC->APB2ENR |= 0x3 <<2;

    // Configurar o pino como saída
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = DHT11_DATA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DHT11_GPIO, &GPIO_InitStruct);

    // Enviar sinal de inicialização do sensor
    GPIO_ResetBits(DHT11_GPIO, DHT11_DATA_PIN);
    __delay(18000);
    GPIO_SetBits(DHT11_GPIO, DHT11_DATA_PIN);
    __delay(20);

    // Configurar o pino como entrada para leitura dos dados do sensor
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(DHT11_GPIO, &GPIO_InitStruct);

}

u16 DHT11_read( void ){
	u16 RETORNO;
	u8 K =0 ;

    uint8_t data[5] = {0};

    // Esperar o sensor iniciar a transmissão de dados
    while (!GPIO_ReadInputDataBit(DHT11_GPIO, DHT11_DATA_PIN)){
		 __delay(1);
			K++;
				if (K > 1000)
					return 1;
		  } K =0;

    // Ler os 40 bits de dados enviados pelo sensor
    for (uint8_t i = 0; i < 40; i++) {
        // Esperar até que o bit atual seja transmitido completamente
/*
        while (GPIO_ReadInputDataBit(DHT11_GPIO, DHT11_DATA_PIN)){
		 __delay(1);
			K++;
				if (K > 1000)
					return 2;
		  } K =0;
*/
	//USARTSend("\r\n___RUNNING....");
        // Aguardarle (GPIO_ReadInputDataBit(DHT11_GPIO, DHT11_DATA_PIN));
		  //
		  //     // Ler os 40 bits de dados enviados pelo
		  //     sensor)) 30us (LOW) e verificar se o bit atual é um 0 ou 1
        __delay(30);
        if (GPIO_ReadInputDataBit(DHT11_GPIO, DHT11_DATA_PIN)) {
            // Se o nível for alto, o bit atual é 1
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
/*
        // Esperar até que o bit atual seja transmitido completamente
        while (GPIO_ReadInputDataBit(DHT11_GPIO, DHT11_DATA_PIN)){
		 __delay(1);
			K++;
				if (K > 1000)
					return 3;
		  } K =0;
*/
    }

    // Verificar a integridade dos dados recebidos
    if (data[0] + data[1] + data[2] + data[3] == data[4]) {
		return RETORNO = data[0] << 8| data[2] <<0;
    }

  return 0xFFFF;
}
