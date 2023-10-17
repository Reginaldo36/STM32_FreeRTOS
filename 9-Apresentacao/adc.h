
void ADC9_Configuration()
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
    RCC->APB2ENR |= (1<<3);
    GPIOB->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);

    // Configurar as seguintes opções para o ADC9:
    // - Modo de operação: modo independente
    // - Modo de conversão: Contínua
    // - Resolução: 12 bits
    // - Fator de divisão do clock: 4 (seleciona um clock ADC de 14 MHz)
    ADC2->CR1 &= ~(ADC_CR1_DISCEN | ADC_CR1_SCAN);
    ADC2->CR1 |= ADC_CR1_EOCIE;
    ADC2->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_ALIGN | ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG);

    ADC2->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_RSTCAL;

    ADC2->SMPR2 |= ADC_SMPR2_SMP9_0 | ADC_SMPR2_SMP9_1;
    ADC2->SQR3 |= 9;

	while(ADC2->CR2 & ADC_CR2_RSTCAL);
    ADC2->CR2 |= ADC_CR2_ADON;
}

uint16_t ADC9_GetValue()
{
    ADC2->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC2->SR & ADC_SR_EOC));
    return ADC2->DR;

}



void ADC8_Configuration()
{
    // Habilitar o clock do ADC8
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB2ENR |= (1<<3);
    GPIOB->CRL &= ~(0xF << 0 ); // Pino PB0

    // Configurar as seguintes opções para o ADC8:
    // - Modo de operação: modo independente
    // - Modo de conversão: Contínua
    // - Resolução: 12 bits
    // - Fator de divisão do clock: 4 (seleciona um clock ADC de 14 MHz)
    ADC1->CR1 &= ~(ADC_CR1_DISCEN | ADC_CR1_SCAN);
    ADC1->CR1 |= ADC_CR1_EOCIE;
    ADC1->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_ALIGN | ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG);

    ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_RSTCAL;

    ADC1->SMPR2 |= ADC_SMPR2_SMP9_0 | ADC_SMPR2_SMP9_1;
    ADC1->SQR3 |= 9;

	while(ADC1->CR2 & ADC_CR2_RSTCAL);

    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t ADC8_GetValue()
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;

}
