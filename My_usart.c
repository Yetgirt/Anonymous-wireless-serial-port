
#include "My_usart.h"	


//////////////////////////////////////////////////////////////////////////////////	 

/*
UART2 : PA2 PA3 ------>��������λ��ͨ��---->������ʹ�ã�500000  TX RX 
*/

void Usart2_init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
 
	//����2��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2����ΪUSART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2
	
	//USART2�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; //GPIOA2  TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA3
	
	/////////////
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3; //GPIOA3  RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //��©
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA3

	//USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//�շ�ģʽ
	//����USART2ʱ��
  USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //ʱ�ӵ͵�ƽ�
  USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK������ʱ������ļ���->�͵�ƽ
  USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //ʱ�ӵڶ������ؽ������ݲ���
  USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //���һλ���ݵ�ʱ�����岻��SCLK���
	//Usart2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=NVIC_UART2_P;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =NVIC_UART2_S;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	////////////////////////////////////////////////////////
	USART_Init(USART2, &USART_InitStructure); //��ʼ������2
	USART_ClockInit ( USART2, &USART_ClockInitStruct );
	////////////////////////////////////////////////////////
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//������������ж�
	USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
}

u8 TxBuffer[256];
u8 TxCounter = 0;
u8 count = 0;

void USART2_IRQHandler(void)                	//����2�жϷ������
{
	u8 com_data;
	if ( USART2->SR & USART_SR_ORE ) //ORE�ж�
  {
		com_data = USART2->DR;
  }
		
	//�����ж�
  if ( USART_GetITStatus ( USART2, USART_IT_RXNE ) )
  {
		USART_ClearITPendingBit ( USART2, USART_IT_RXNE ); //����жϱ�־

    com_data = USART2->DR;
    ANO_DT_Data_Receive_Prepare ( com_data );
  }
	
	//���ͣ�������λ���ж�
  if ( USART_GetITStatus ( USART2, USART_IT_TXE ) )
  {
    USART2->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־
    if ( TxCounter == count )
    {
			USART2->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
    }
  }
} 

void Usart2_Send( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        TxBuffer[count++] = * ( DataToSend + i ); //�Զ��崢����
    }

    if ( ! ( USART2->CR1 & USART_CR1_TXEIE ) ) //���û�ڷ���
    {
        USART_ITConfig ( USART2, USART_IT_TXE, ENABLE ); //�򿪷����жϣ����뷢���ж�
    }
}
/*
UART5 :	PC12 PD2 ------>�����뼤����/�����״�---->�����ʣ�115200	TX RX---Y_TFminis
*/
void Uart5_Init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIODʱ��
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);//ʹ��UART5ʱ��
 
	//����5��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5); //GPIOC12����ΪUSART5
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); //GPIOD2����ΪUSART5
	
	//USART5�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init ( GPIOC, &GPIO_InitStructure );
	GPIO_Init ( GPIOD, &GPIO_InitStructure );
  //USART5 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//�շ�ģʽ
	//Usart6 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;//����6�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=NVIC_UART5_P;//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =NVIC_UART5_S;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	////////////////////////////////////////////////////////
	USART_Init(UART5, &USART_InitStructure); //��ʼ������5
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//������������ж�
	USART_Cmd(UART5, ENABLE);  //ʹ�ܴ���5
}
/*����5�����жϺ���*/
void UART5_IRQHandler(void)                	//����5�жϷ������
{
	u8 com_data;
	//�����ж�
  if ( USART_GetITStatus ( UART5, USART_IT_RXNE ) )
  {
		USART_ClearITPendingBit ( UART5, USART_IT_RXNE ); //����жϱ�־
    com_data = UART5->DR;
    Data_Receive_Prepare( com_data );
  }
} 
/*
UART3 : PC10 PC11 ----->����nano����------->������: 115200  TX  RX
*/
void Usart3_Init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
 
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3); //GPIOC10����ΪUSART3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3); //GPIOC11����ΪUSART3
	
	//USART3�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; //GPIOC10  TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC10
	
	/////////////
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11; //GPIOC11  RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //��©
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC11

	//USART3 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//�շ�ģʽ
	//����USART3ʱ��
  USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //ʱ�ӵ͵�ƽ�
  USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK������ʱ������ļ���->�͵�ƽ
  USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //ʱ�ӵڶ������ؽ������ݲ���
  USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //���һλ���ݵ�ʱ�����岻��SCLK���
	//Usart3 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=NVIC_UART3_P;//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =NVIC_UART3_S;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	////////////////////////////////////////////////////////
	USART_Init(USART3, &USART_InitStructure); //��ʼ������3
	USART_ClockInit ( USART3, &USART_ClockInitStruct );
	////////////////////////////////////////////////////////
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//������������ж�
	USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���3
}
u8 com_data;
/*����3�����жϺ���*/
void USART3_IRQHandler(void)                	//����3�жϷ������
{

	//�����ж�
  if ( USART_GetITStatus ( USART3, USART_IT_RXNE ) )
  {
		USART_ClearITPendingBit ( USART3, USART_IT_RXNE ); //����жϱ�־
    com_data = USART3->DR;
    Receive_Nano_Data( com_data );
  }
} 

