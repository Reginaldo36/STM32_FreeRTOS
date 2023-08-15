/* ========================================================================= 

  Controle para display LCD 20x2 JHD202C
  myLcd.h

  Autor: Dr. Eng. Wagner Rambo
  Data:  Dezembro de 2022

========================================================================= */

#ifndef   MYLCD_H
#define   MYLCD_H
/* ========================================================================= */
/* --- Bibliotecas --- */
// #include <Arduino.h>
#include "stm32f10x.h"

// Gambiarra para não ter que fazer diversas mudanças de
// codigos no *.c *.h

#define PD2	2
#define PD3	3
#define PD4	4
#define PD5	5
#define PD6	6
#define PD7	7

/* ========================================================================= */
/* --- Mapeamento de Hardware --- */
//#define      reg1       PORTD                  //registrador 1 utilizado nas IOs
#define		 IO			reg1
#define      reg1       GPIOA->ODR                  //registrador 1 utilizado nas IOs

#define      RS         2                    //pino register select do LCD
#define      EN         3                    //pino enable do LCD
#define      D4         4                    //pino de dados DB4 do LCD
#define      D5	    	5                    //pino de dados DB5 do LCD
#define      D6		   6                    //pino de dados DB6 do LCD
#define      D7		   7                    //pino de dados DB7 do LCD

#define delayMicroseconds(X)	 vTaskDelay(X)
#define delay(X)	vTaskDelay(X)

/* ========================================================================= */
/* --- Macros --- */
#define   set_bit(reg,bit)		  (reg |= (1<<bit))		       /* seta um bit de determinado registrador */
#define   clr_bit(reg,bit)	  	(reg &= ~(1<<bit))		     /* limpa um bit de determinado registrador */


/* ========================================================================= */
/* --- Protótipo das Funções --- */
void disp_number(unsigned num, char row, char col);        /* converte um inteiro de até 5 dígitos para exibir no display, remove zeros à esquerda */
void disp_wr_po(unsigned char chr, char row, char col);    /* função para escrever caracteres no LCD na posição indicada */
void disp_text(char *str, char row, char col);             /* função para escrever uma string no LCD */
void disp_write(unsigned char chr);                        /* função para escrever caracteres no LCD */
void disp_cmd(unsigned char cmd);                          /* função para enviar comandos para o LCD*/
void disp_init(void);                                      /* função para inicializar o LCD */
void disp_clear(void);                                     /* função para limpar o LCD */
void send_nibble(unsigned char nib, char rsel);            /* envia cada nibble separadamente e gera pulso em enable */

#endif //MYLCD_H
		 
/* ========================================================================= */
/* --- Fim --- */
