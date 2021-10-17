#ifndef __MY_USART_H
#define __MY_USART_H

#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "Ano_DT.h"
#include "tf_minis.h"
#include "nano.h" 

#define NVIC_UART2_P			3			//串口2中断配置  //数传
#define NVIC_UART2_S			0

#define NVIC_UART3_P			1			//串口3中断配置  //nano
#define NVIC_UART3_S			0

#define NVIC_UART5_P			2			//串口5中断配置  //激光雷达/TFmini
#define NVIC_UART5_S			0

void Usart2_init(u32 bound);
void Usart2_Send ( unsigned char *DataToSend , u8 data_num );
void Usart3_Init(u32 bound);
void Uart5_Init(u32 bound);
#endif


