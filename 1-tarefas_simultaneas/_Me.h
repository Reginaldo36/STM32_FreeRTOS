#define __IO volatile  //Habilita RW
#define uint32_t unsigned int
#define u32 unsigned int

#define uint8_t unsigned char
#define u8 unsigned char

#define int32_t int

#define uint16_t unsigned short
#define u16 unsigned short

#define STACKINT  0x20000000

//--------------------  GPIOx Def --------------------
#define GPIOA ((GPIO_TypeDef * ) 0x40010800) /* GPIOx Addr Base */
#define GPIOB ((GPIO_TypeDef * ) 0x40010c00) 
#define GPIOC ((GPIO_TypeDef * ) 0x40011000) 
#define GPIOD ((GPIO_TypeDef * ) 0x40011400) 
#define GPIOE ((GPIO_TypeDef * ) 0x40011800) 
#define GPIOF ((GPIO_TypeDef * ) 0x40011c00) 
#define GPIOG ((GPIO_TypeDef * ) 0x40012000) 

//-------------------- TIMx Def -------------------- 
#define TIM1 ((TIM_TypeDef * ) 0x40012c00) /* TIMx Addr Base */
#define TIM2 ((TIM_TypeDef * ) 0x40000000) 
#define TIM3 ((TIM_TypeDef * ) 0x40000400) 
#define TIM4 ((TIM_TypeDef * ) 0x40000800) 
#define TIM5 ((TIM_TypeDef * ) 0x40000c00) 
#define TIM6 ((TIM_TypeDef * ) 0x40001000) 
#define TIM7 ((TIM_TypeDef * ) 0x40001400) 
#define TIM8 ((TIM_TypeDef * ) 0x40013400) 
#define TIM9 ((TIM_TypeDef * ) 0x40014C00) 
#define TIM10 ((TIM_TypeDef * ) 0x40015000) 
#define TIM11 ((TIM_TypeDef * ) 0x40015400) 
#define TIM12 ((TIM_TypeDef * ) 0x40001800) 
#define TIM13 ((TIM_TypeDef * ) 0x40001c00) 
#define TIM14 ((TIM_TypeDef * ) 0x40002000) 
#define FLASH (( FLASH_TypeDef * ) 0x40022000)

//-------------------- AFIO 
#define AFIO (( AFIO_TypeDef *) 0x40010000 )

//-------------------- EXTI 
#define EXTI (( EXTI_TypeDef *) 0x40010400 )

//-------------------- RCC Def -------------------- 
#define RCC (( RCC_TypeDef *) 0x40021000) /* RCC Addr Base */

/*-------------------- NVIC --------------------*/
#define NVIC_BASE (( __IO u32 *) 0xE000E100) 
#define NVIC            ((NVIC_type  *)  NVIC_BASE)


//-------------------- Area de funções ---------------  
typedef struct {
   uint32_t   ISER[8];     /* Address offset: 0x000 - 0x01C */
   uint32_t  RES0[24];     /* Address offset: 0x020 - 0x07C */
   uint32_t   ICER[8];     /* Address offset: 0x080 - 0x09C */
   uint32_t  RES1[24];     /* Address offset: 0x0A0 - 0x0FC */
   uint32_t   ISPR[8];     /* Address offset: 0x100 - 0x11C */
   uint32_t  RES2[24];     /* Address offset: 0x120 - 0x17C */
   uint32_t   ICPR[8];     /* Address offset: 0x180 - 0x19C */
   uint32_t  RES3[24];     /* Address offset: 0x1A0 - 0x1FC */
   uint32_t   IABR[8];     /* Address offset: 0x200 - 0x21C */
   uint32_t  RES4[56];     /* Address offset: 0x220 - 0x2FC */
   uint8_t   IPR[240];     /* Address offset: 0x300 - 0x3EC */
   uint32_t RES5[644];     /* Address offset: 0x3F0 - 0xEFC */
   uint32_t       STIR;    /* Address offset:         0xF00 */
} NVIC_type;


typedef struct{
   __IO u32 CRL ;       // 0x00 MODO de operação Port configuration register low
   __IO u32 CRH ;       // 0x04 Port configuration register high
   __IO u32 IDR ;    // 0x08 Port input data register
   __IO u32 ODR ;    // 0x0c Port output data register
   __IO u32 BSRR ;      // 0x10 Port bit set/reset register
   __IO u32 BRR ;    // 0x14 Port bit reset register
   __IO u32 LCKR ;      // 0x18 Port configuration lock register

} GPIO_TypeDef ;

typedef struct{

   __IO u32 EVCR;    // 0x00
   __IO u32 MAPR;    // 0x04
   __IO u16 EXTICR1;    // 0X08
   __IO u16 RES1 ;   // 
   __IO u16 EXTICR2;    // 0x0c
   __IO u16 RES2 ;   // 
   __IO u16 EXTICR3;    // 0x10
   __IO u16 RES3 ;   //
   __IO u16 EXTICR4;    // 0x14
   __IO u16 RES4 ;   // 
   __IO u32 reservado;  // 0x18
   __IO u32 MAPR2;      // 0x1c

} AFIO_TypeDef ;

typedef struct{

   __IO u32 CR1;     // 0x00
   __IO u32 CR2;     // 0x04
   __IO u32 SMCR;    // 0x08
   __IO u32 DIER;    // 0x0c
   __IO u32 SR;      // 0x10
   __IO u32 EGR;     // 0x14
   __IO u32 CCMR1;      // 0x18 ->  input/output capture mode
   __IO u32 CCMR2;      // 0x1c ->  input/output capture mode
   __IO u32 CCER;       // 0x20 <--- 
   __IO u32 CNT;     // 0x24
   __IO u32 PSC;     // 0x28
   __IO u32 ARR;     // 0x2c
   __IO u32 RCR;     // 0x30
   __IO u32 CCR1;       // 0x34
   __IO u32 CCR2;    // 0x38 
   __IO u32 CCR3;    // 0x3c
   __IO u32 CCR4;    // 0x40
   __IO u32 BDTR;    // 0x44 -> Descritivo na pagina 344 do RM0008
   __IO u32 DCR;     // 0x48
   __IO u32 DMAR;    // 0x4c

} TIM_TypeDef ;

typedef struct{
//                  Offset
   __IO u32 CR;      // 0x00
   __IO u32 CFGR;    // 0x04 
   __IO u32 CIR;     // 0x08
   __IO u32 APB2RSTR;   // 0x0c
   __IO u32 APB1RSTR;   // 0x10
   __IO u32 AHBENR;  // 0x14
   __IO u32 APB2ENR; // 0x18
   __IO u32 APB1ENR; // 0x1c
   __IO u32 BDCR;    // 0x20
   __IO u32 CSR;     // 0x24
   __IO u32 AHBRSTR; // 0x28
   __IO u32 CFGR2;      // 0x2c

} RCC_TypeDef;

typedef struct{

   __IO u32 GOTGCTL;       // 0x00
   __IO u32 GOTGINT;    // 0x04
   __IO u32 GAHBCFG;    /* 0x08 <<- Responsavel por habilitar 
                 interrupções AHB ou USB */
   __IO u32 GUSBCFG;    // 0x0c
   __IO u32 GRSTCTL;    // 0x10
   __IO u32 GINTSTS;    // 0x14
   __IO u32 GINTMSK;    // 0x18
   __IO uint16_t GRXSTSR_1_H; // 0x1c Host/Device Mode
   __IO uint16_t GRXSTSR_1_D; // 0x1c Host/Device Mode
   __IO uint16_t GRXSTSR_2_H; // 0x20 Host/Device Mode
   __IO uint16_t GRXSTSRPR_2_D;  // 0x20 Host/Device Mode
   __IO u32 GRXFSIZ;    // 0x24
   __IO uint16_t  HNPTXFSIZ;  // 0x28 
   __IO uint16_t  DIEPTXF0;   // 0x28
   __IO u32 HNPTXSTS;      // 0x2c

} OTG_FS_TypeDef;

typedef struct {

   __IO u32 IMR ;    // 0x00
   __IO u32 EMR ;    // 0x04  
   __IO u32 RTSR ;      // 0x08  
   __IO u32 FTSR ;      // 0x0c  
   __IO u32 SWIER ;  // 0x10  
   __IO u32 PR ;     // 0x14  

} EXTI_TypeDef;

typedef struct {

   __IO u32 ACR;
   __IO u32 KEYR;
   __IO u32 OPTKEYR;
   __IO u32 SR;
   __IO u32 CR;
   __IO u32 AR;
   __IO u32 RESERVED;
   __IO u32 OBR;
   __IO u32 WRPR;

} FLASH_TypeDef;
typedef enum IRQn
{
   NonMaskableInt_IRQn         = -14,    /* 2 Non Maskable Interrupt                             */
   MemoryManagement_IRQn       = -12,    /* 4 Cortex-M3 Memory Management Interrupt              */
   BusFault_IRQn               = -11,    /* 5 Cortex-M3 Bus Fault Interrupt                      */
   UsageFault_IRQn             = -10,    /* 6 Cortex-M3 Usage Fault Interrupt                    */
   SVCall_IRQn                 = -5,     /* 11 Cortex-M3 SV Call Interrupt                       */
   DebugMonitor_IRQn           = -4,     /* 12 Cortex-M3 Debug Monitor Interrupt                 */
   PendSV_IRQn                 = -2,     /* 14 Cortex-M3 Pend SV Interrupt                       */
   SysTick_IRQn                = -1,     /* 15 Cortex-M3 System Tick Interrupt                   */
   WWDG_IRQn                   = 0,      /* Window WatchDog Interrupt                            */
   PVD_IRQn                    = 1,      /* PVD through EXTI Line detection Interrupt            */
   TAMPER_IRQn                 = 2,      /* Tamper Interrupt                                     */
   RTC_IRQn                    = 3,      /* RTC global Interrupt                                 */
   FLASH_IRQn                  = 4,      /* FLASH global Interrupt                               */
   RCC_IRQn                    = 5,      /* RCC global Interrupt                                 */
   EXTI0_IRQn                  = 6,      /* EXTI Line0 Interrupt                                 */
   EXTI1_IRQn                  = 7,      /* EXTI Line1 Interrupt                                 */
   EXTI2_IRQn                  = 8,      /* EXTI Line2 Interrupt                                 */
   EXTI3_IRQn                  = 9,      /* EXTI Line3 Interrupt                                 */
   EXTI4_IRQn                  = 10,     /* EXTI Line4 Interrupt                                 */
   DMA1_Channel1_IRQn          = 11,     /* DMA1 Channel 1 global Interrupt                      */
   DMA1_Channel2_IRQn          = 12,     /* DMA1 Channel 2 global Interrupt                      */
   DMA1_Channel3_IRQn          = 13,     /* DMA1 Channel 3 global Interrupt                      */
   DMA1_Channel4_IRQn          = 14,     /* DMA1 Channel 4 global Interrupt                      */
   DMA1_Channel5_IRQn          = 15,     /* DMA1 Channel 5 global Interrupt                      */
   DMA1_Channel6_IRQn          = 16,     /* DMA1 Channel 6 global Interrupt                      */
   DMA1_Channel7_IRQn          = 17,     /* DMA1 Channel 7 global Interrupt                      */
   ADC1_2_IRQn                 = 18,     /* ADC1 and ADC2 global Interrupt                       */
   CAN1_TX_IRQn                = 19,     /* USB Device High Priority or CAN1 TX Interrupts       */
   CAN1_RX0_IRQn               = 20,     /* USB Device Low Priority or CAN1 RX0 Interrupts       */
   CAN1_RX1_IRQn               = 21,     /* CAN1 RX1 Interrupt                                   */
   CAN1_SCE_IRQn               = 22,     /* CAN1 SCE Interrupt                                   */
   EXTI9_5_IRQn                = 23,     /* External Line[9:5] Interrupts                        */
   TIM1_BRK_IRQn               = 24,     /* TIM1 Break Interrupt                                 */
   TIM1_UP_IRQn                = 25,     /* TIM1 Update Interrupt                                */
   TIM1_TRG_COM_IRQn           = 26,     /* TIM1 Trigger and Commutation Interrupt               */
   TIM1_CC_IRQn                = 27,     /* TIM1 Capture Compare Interrupt                       */
   TIM2_IRQn                   = 28,     /* TIM2 global Interrupt                                */
   TIM3_IRQn                   = 29,     /* TIM3 global Interrupt                                */
   TIM4_IRQn                   = 30,     /* TIM4 global Interrupt                                */
   I2C1_EV_IRQn                = 31,     /* I2C1 Event Interrupt                                 */
   I2C1_ER_IRQn                = 32,     /* I2C1 Error Interrupt                                 */
   I2C2_EV_IRQn                = 33,     /* I2C2 Event Interrupt                                 */
   I2C2_ER_IRQn                = 34,     /* I2C2 Error Interrupt                                 */
   SPI1_IRQn                   = 35,     /* SPI1 global Interrupt                                */
   SPI2_IRQn                   = 36,     /* SPI2 global Interrupt                                */
   USART1_IRQn                 = 37,     /* USART1 global Interrupt                              */
   USART2_IRQn                 = 38,     /* USART2 global Interrupt                              */
   USART3_IRQn                 = 39,     /* USART3 global Interrupt                              */
   EXTI15_10_IRQn              = 40,     /* External Line[15:10] Interrupts                      */
   RTCAlarm_IRQn               = 41,     /* RTC Alarm through EXTI Line Interrupt                */
   OTG_FS_WKUP_IRQn            = 42,     /* USB OTG FS WakeUp from suspend through EXTI Line Int */
   TIM5_IRQn                   = 50,     /* TIM5 global Interrupt                                */
   SPI3_IRQn                   = 51,     /* SPI3 global Interrupt                                */
   UART4_IRQn                  = 52,     /* UART4 global Interrupt                               */
   UART5_IRQn                  = 53,     /* UART5 global Interrupt                               */
   TIM6_IRQn                   = 54,     /* TIM6 global Interrupt                                */
   TIM7_IRQn                   = 55,     /* TIM7 global Interrupt                                */
   DMA2_Channel1_IRQn          = 56,     /* DMA2 Channel 1 global Interrupt                      */
   DMA2_Channel2_IRQn          = 57,     /* DMA2 Channel 2 global Interrupt                      */
   DMA2_Channel3_IRQn          = 58,     /* DMA2 Channel 3 global Interrupt                      */
   DMA2_Channel4_IRQn          = 59,     /* DMA2 Channel 4 global Interrupt                      */
   DMA2_Channel5_IRQn          = 60,     /* DMA2 Channel 5 global Interrupt                      */
   ETH_IRQn                    = 61,     /* Ethernet global Interrupt                            */
   ETH_WKUP_IRQn               = 62,     /* Ethernet Wakeup through EXTI line Interrupt          */
   CAN2_TX_IRQn                = 63,     /* CAN2 TX Interrupt                                    */
   CAN2_R_IRQn                 = 67      /* USB OTG FS global Interrupt                          */
} IRQn_type;


void SystemInit (void);
void nvic_IntDisble(u8 IRQn);
void nvic_intEnable(u8 IRQn);
void set_system_clock_to_25Mhz (void);
void set_system_clock_to_72Mhz(void);
void Delay ( __IO u32 T );
void enable_TIM3_delay (void);


void set_system_clock_to_72Mhz(void)
{
   // Necessary wait states for Flash for high speeds
   FLASH->ACR = 0x12;
   // Enable HSE
   RCC->CR |= (1 << 16);
   // Wait untill HSE settles down
   while (!(RCC->CR & (1 << 17)));
   // Set PREDIV2 division factor to 5
   RCC->CFGR2 |= (0b0100 << 4);
   // Set PLL2 multiplication factor to 8
   RCC->CFGR2 |= (0b0110 << 8);
   // Enable PLL2
   RCC->CR |= (1 << 26);
   // Wait untill PLL2 settles down
	// while (!(RCC->CR & (1 << 27)));
   // Set PLL2 as PREDIV1 clock source
   RCC->CFGR2 |= (1 << 16);
   // Set PREDIV1 division factor to 5
   RCC->CFGR2 |= (0b0100 << 0);
   // Select Prediv1 as PLL source
   RCC->CFGR |= (1 << 16);
   // Set PLL1 multiplication factor to 9
   RCC->CFGR |= (0b0111 << 18);
   // Set APB1 to 36MHz
   RCC->CFGR |= 1 << 10;
   // Enable PLL
   RCC->CR |= (1 << 24);
   // Wait untill PLL settles down
   while (!(RCC->CR & (1 << 25)));
   // Finally, choose PLL as the system clock
   RCC->CFGR |= (0b10 << 0);
} 

void Delay ( __IO u32 T ) { // T=1 ;  868µs -> T=1 ~ 526µs
  for ( T ; T > 0 ; T-- ){
   TIM3->EGR |= ( 1 << 0 );
   while ( TIM3->CNT < 5 ) ; /* 5 * 10-- Configurar para IR */
  }
}

void enable_TIM3_delay (void) {
   RCC->APB1ENR |= 1<<1 ;
   TIM3->CR1 = 0x0000;
   //TIM3->ARR = 21699;
   TIM3->ARR = 250-1; // 10µs
   // para obter o período de 868µs
/*https://www.vishay.com/docs/80071/dataform.pdf PAG 2
 * https://controllerstech.com/ir-remote-with-stm32/
 * Per=(1+ARR)/Clock_system */
   TIM3->CR1 |= 1<<0;
}

void enable_interrupt(IRQn_type IRQn)
{
   NVIC->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}  
   
/* 
 * Disable given interrupt
 */
void disable_interrupt(IRQn_type IRQn)
{
   NVIC->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}


/* Definições de bits "CFG" e "Mode" da configuração das GPIOs.
 *		   Pin_Hardw	
 *   CFG     MODE    H	 L   
 * |31|30|  |29|28| 15 ; 7		
 * |27|26|  |25|24| 14 ; 6	
 * |23|22|  |21|20| 13 ; 5
 * |19|18|  |17|16| 12 ; 4
 * |15|14|  |13|12| 11 ; 3
 * |11|10|  | 9| 8| 10 ; 2
 * | 7| 6|  | 5| 4|  9 ; 1
 * | 3| 2|  | 1| 0|  8 ; 0
 *
 *
 * __________________________________________
 * HexaCode -> 50MHz			↓ - PxODR
 * 0x03 | 0| 0|  |01..11| |x| <- Out Push-pull
 * 0x07 | 0| 1|  |01..11| |x| <- Out Open-drain
 * 0x0B | 1| 0|  |01..11| |x| <- Alt Out *pull
 * 0x0B | 1| 0|  |01..11| |x| <- Alt Out *drain
 * __________________________________________
 * 0x00 | 0| 0|  |  00  | |x| -< Input Analogig
 * 0x04 | 0| 1|  |  00  | |x| -< Input Floating  
 * 0x08 | 1| 0|  |  00  | |1| -< Input Pull-up
 * 0x0B | 1| 0|  |  00  | |0| -< Input Pull-down
 * __________________________________________
 * */

/* tabela_2 -> Configuração do Registrador AFIO->EXTICRx
 *     EXTI[0..15] 3    2    1    0 
 *
 *     Shift     <<12  <<8  <<4  <<0 	
 *	              xxxx xxxx xxxx xxxx 
 * GPIO[a..g] pin  3    2    1    0   	AFIO_EXTI_CR_1
 * GPIO[a..g] pin  7    6    5    4  	AFIO_EXTI_CR_2 
 * GPIO[a..g] pin  11   10   9    8  	AFIO_EXTI_CR_3 
 * GPIO[a..g] pin  15   14   13   12  	AFIO_EXTI_CR_4 
 *
 *Hex  xxxx
 *0x00 0000: PA[x] pin 
 *0x01 0001: PB[x] pin 
 *0x02 0010: PC[x] pin 
 *0x03 0011: PD[x] pin 
 *0x04 0100: PE[x] pin 
 *0x05 0101: PF[x] pin 
 *0x06 0110: PG[x] pin
 *
 * */

/* link_1 https://developer.arm.com/documentation/dui0662/b/Cortex-M0--Peripherals/Nested-Vectored-Interrupt-Controller/Interrupt-Set-Enable-Register
 *
 *
 *
 * https://github.com/98zam98/arm_nvic_driver/blob/main/hardware_arm_nvic_driver.h
 */

/*
 * Neste código, fora resolvido o problema com a interrupção
 * acredito que definitivamente usando o projeto:
 * "stm32f1-bare-metal/ext_int/ext_int.c"
 *		Usando uma estrutura com as definições de  vetores.
 *		Fora usado também, o manual do programador:
 * 
 * dm00046982-stm32-cortex-m4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf 
 */
