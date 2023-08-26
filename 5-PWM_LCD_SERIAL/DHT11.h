#define GPIOx GPIOB
#define PP (1<<12)

#define OUT_P (0x3 << 16)
#define IN_P (0x4 << 16)

//#define __delay xTime....

#define ERRO 0xFF

void DHT11_START(){
	RCC->APB2ENR |= (1<<3); //GPGPIOx
	GPIOx->CRH	|= OUT_P;

	GPIOx->ODR	&=~ PP;
	__delay(1800);
	GPIOx->ODR	^= PP;
	__delay(20);

	GPIOx->CRH &= ~(0xF << 16);
	GPIOx->CRH |= IN_P;
}

u16 DHT11_read( void ){
			 USARTSend("\r\n------------1------------\r\n");
	u16 RETORNO;
	u8 K =0 ;

	uint8_t i, j;
	uint8_t data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

	 GPIOx->CRH |= (0x3<<16 );


	 GPIOx->ODR |= 1<<12;
	 __delay_ms(18);

	 GPIOx->ODR &=~(1<<12);

	 __delay(30);


	 GPIOx->CRH &= ~(0xF<<16 ); // reset -> Analo
	 // GPIOx->CRH |= ~(0x4<<16 );

	 while((GPIOx->IDR & (1<<12))) {
	  __delay(1);
	  K++;
	  if(K > 100)
		return 3;
	 }

	 while(!((GPIOx->IDR & (1<<12)))) {
	  __delay(1);
	  K++;
	  if(K > 100)
		return 4;
	 }
	 K =0 ;





	 for(j = 0; j < 5; ++j) {
		  for(i = 0; i < 8; ++i) {
				while(!((GPIOx->IDR & (1<<12))));
				while((GPIOx->IDR & (1<<12)));
				data[j] = data[j] << 1;

			  __delay(1);
			  K++;
			  if(K > 40)
				 data[j] = data[j]+1;

		  }
	 }


  if(data[4] == (data[0] + data[2]))
		return RETORNO |= data[0] << 8| data[2] <<0;

  return 0xFFFF;
}
