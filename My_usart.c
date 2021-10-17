
#include "My_usart.h"	


//////////////////////////////////////////////////////////////////////////////////	 

/*
UART2 : PA2 PA3 ------>用于与上位机通信---->波特率使用：500000  TX RX 
*/

void Usart2_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
 
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
	
	//USART2端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; //GPIOA2  TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA3
	
	/////////////
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3; //GPIOA3  RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //开漏
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA3

	//USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//收发模式
	//配置USART2时钟
  USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
  USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
  USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
  USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
	//Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=NVIC_UART2_P;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =NVIC_UART2_S;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	////////////////////////////////////////////////////////
	USART_Init(USART2, &USART_InitStructure); //初始化串口2
	USART_ClockInit ( USART2, &USART_ClockInitStruct );
	////////////////////////////////////////////////////////
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启接收相关中断
	USART_Cmd(USART2, ENABLE);  //使能串口2
}

u8 TxBuffer[256];
u8 TxCounter = 0;
u8 count = 0;

void USART2_IRQHandler(void)                	//串口2中断服务程序
{
	u8 com_data;
	if ( USART2->SR & USART_SR_ORE ) //ORE中断
  {
		com_data = USART2->DR;
  }
		
	//接收中断
  if ( USART_GetITStatus ( USART2, USART_IT_RXNE ) )
  {
		USART_ClearITPendingBit ( USART2, USART_IT_RXNE ); //清除中断标志

    com_data = USART2->DR;
    ANO_DT_Data_Receive_Prepare ( com_data );
  }
	
	//发送（进入移位）中断
  if ( USART_GetITStatus ( USART2, USART_IT_TXE ) )
  {
    USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志
    if ( TxCounter == count )
    {
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
    }
  }
} 

void Usart2_Send( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        TxBuffer[count++] = * ( DataToSend + i ); //自定义储存器
    }

    if ( ! ( USART2->CR1 & USART_CR1_TXEIE ) ) //如果没在发送
    {
        USART_ITConfig ( USART2, USART_IT_TXE, ENABLE ); //打开发送中断，进入发送中断
    }
}


