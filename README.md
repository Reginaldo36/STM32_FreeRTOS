# Objetivo:
Este projeto faz parte de trabalho de conclusão de curso
de Engenharia da Computação. Sendo uma continuação do
projeto de pesquisa (2021 - 2022) de tema: "Introdução
ao desenvolvimento de sistemas embarcados baseados nos
microcontroladores ARM STM32".

Trabalho de conclusão de curso (2023): "Estudo da
aplicação de um sistema operacional de tempo real em
microcontroladores ARM".

### Universidade: UNILINS.
### Aluno: Reginaldo Junior.

### Licensa: GNU GPL_V2 quando possível.

Sistema operacional: Debian 12 - sid 
Suíte de compilação: GNU Toolchain 

#### Opções do Makefile 
```
$ make
$ make clean
$ make flash
$ make erase

```

#### st-link Depuração: 
execute no diretório source
```
$ st-util

$ arm-none-eabi-gdb --eval-command="target extended-remote :4242" *.elf
```


GNU gdb (GNU Tools for ARM Embedded Processors) 
Copyright (C) 2015 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.  Type "show copying"
and "show warranty" for details.
This GDB was configured as "--host=x86_64-apple-darwin10 --target=arm-none-eabi".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
<http://www.gnu.org/software/gdb/documentation/>.
For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from z2_cmcc_b_app.elf...done.
mcu: STM32F103C8T6


Notas e referências:
A estrutura do projeto tem como base:
    <https://github.com/tuanhe/Stm32_GCC_FreeRTOS10.2.1/tree/master>
