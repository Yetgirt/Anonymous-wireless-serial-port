#ifndef __MY_USART_H
#define __MY_USART_H

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "Ano_DT.h"


#define NVIC_UART2_P			3			//串口2中断配置  //数传
#define NVIC_UART2_S			0


void Usart2_init(u32 bound);
void Usart2_Send ( unsigned char *DataToSend , u8 data_num );

#endif


