/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：数据传输
**********************************************************************************/
/*============================================================================
更新：
201907272159-Jyoun：增加飞控状态界面传感器状态数据发送。
201908031126-茶不思：增加mv相关数据发送，可在上位机查看mv寻色块数据。


===========================================================================*/

#include "Ano_DT.h"

/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

#define MYHWADDR	0x05
#define SWJADDR		0xAF

#define PARNUM		100
s32 ParValList[100];		//参数列表

dt_flag_t f;					//需要发送数据的标志
u8 data_to_send[50];	//发送数据缓存


/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	Usart2_Send(data_to_send, length);
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//此函数应由用户每1ms调用一次	

void ANO_DT_Data_Exchange(u8 dT_ms)
{
	static u16 cnt = 0;  //cnt会++，然后通过设定下面的static的值，控制发送速度
	static u16 mpu_cnt = 15;
	static u16 sensor_cnt = 10;
	static u16 user_cnt 	= 10;
	static u16 motor_cnt 	= 10;
	static u16 dist_cnt 	= 15;
	
	if((cnt % user_cnt) == (user_cnt-2))		//发送用户自定义
		f.send_user = 1;
	
	if((cnt % mpu_cnt) == (mpu_cnt-1))			//发送MPU6050 ，
		f.send_mpu = 1;
	
	if((cnt % sensor_cnt) == (sensor_cnt-1))			//发送MPU6050
		f.send_senser = 1;
	
	if((cnt % motor_cnt) == (motor_cnt-1))			//发送PWM
		f.send_motopwm = 1;
	
	if((cnt % dist_cnt) == (dist_cnt-1))			//发送距离
		f.send_dist = 1;
	
	if(++cnt>1000) cnt = 0;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.paraToSend < 0xffff)
	{
		ANO_DT_SendParame(f.paraToSend);
		f.paraToSend = 0xffff;
	}
	else if(f.send_user)  //自定义用户发送
	{
		f.send_user = 0;
		ANODT_SendF1(speed.encoder[m1], speed.encoder[m2], speed.encoder[m3], speed.encoder[m4], speed.encoder[4], 
								 step.same_type, step.step_sta, kalman.gyroz2, expect_value,
								 category, rubbish_x, rubbish_y, bottom);
	}
	else if(f.send_mpu)
	{
		f.send_mpu = 0;
		ANODT_Send_MPU(0,imu.yaw2,imu.yaw,0,0,0);
	}
	else if(f.send_senser)
	{
		f.send_senser = 0;
		ANO_DT_Send_Senser(0,0,0,0,imu.gyroz2,imu.gyroz);					//第二项为云台yaw,第五项为云台角加速度,第六项为小车角加速度gyroz
	}
	else if(f.send_motopwm)
	{
		f.send_motopwm = 0;
		ANO_DT_Send_MotoPWM(step.rubbish_type[0],step.rubbish_type[1],step.rubbish_type[2],step.rubbish_type[3],step.point_type[4]);
	}
	else if(f.send_dist)
	{
		f.send_dist = 0;
		ANO_DT_Send_Dist(X_TF_distance,Y_TF_distance);
	}

/////////////////////////////////////////////////////////////////////////////////////
	ANO_DT_Data_Receive_Anl_Task();			
/////////////////////////////////////////////////////////////////////////////////////
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
static u8 DT_RxBuffer[256],DT_data_cnt = 0,ano_dt_data_ok;

void ANO_DT_Data_Receive_Anl_Task()
{
	if(ano_dt_data_ok)
	{
		ANO_DT_Data_Receive_Anl(DT_RxBuffer,DT_data_cnt+6);
		ano_dt_data_ok = 0;
	}
}

void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 _data_len = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)	//帧头0xAA
	{
		state=1;
		DT_RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)	//数据源，0xAF表示数据来自上位机
	{
		state=2;
		DT_RxBuffer[1]=data;
	}
	else if(state==2)		//数据目的地
	{
		state=3;
		DT_RxBuffer[2]=data;
	}
	else if(state==3)		//功能字
	{
		state=4;
		DT_RxBuffer[3]=data;
	}
	else if(state==4)		//数据长度
	{
		state = 5;
		DT_RxBuffer[4]=data;
		_data_len = data;
		DT_data_cnt = 0;
	}
	else if(state==5&&_data_len>0&&_data_len<80)
	{
		_data_len--;
		DT_RxBuffer[5+DT_data_cnt++]=data;
		if(_data_len==0)
			state = 6;
	}
	else if(state==6)
	{
		state = 0;
		DT_RxBuffer[5+DT_data_cnt]=data;
		ano_dt_data_ok = 1;//ANO_DT_Data_Receive_Anl(DT_RxBuffer,DT_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
u16 flash_save_en_cnt = 0;
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	u8 i;
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==MYHWADDR)
	{
		if(*(data_buf+3)==0XE0)			//命令E0
		{
			switch(*(data_buf+5))		//FUN
			{
				case 0x02:
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xAA)	//恢复默认PID
					{
						PID_Rest();
						data_save();
					}
					break;
				case 0xE1:
					f.paraToSend = (u16)(*(data_buf+6)<<8)|*(data_buf+7);	//读取参数
					break;
				case 0x11:
					break;
				default:
					break;
			}
			ANO_DT_SendCmd(SWJADDR,*(data_buf+5),(u16)(*(data_buf+6)<<8)|*(data_buf+7),(u16)(*(data_buf+8)<<8)|*(data_buf+9),(u16)(*(data_buf+10)<<8)|*(data_buf+11),(u16)(*(data_buf+12)<<8)|*(data_buf+13),(u16)(*(data_buf+14)<<8)|*(data_buf+15));
		}
		else if(*(data_buf+3)==0XE1)	//设置参数
		{
			u16 _paraNum = (u16)(*(data_buf+5)<<8)|*(data_buf+6);
			s32 _paraVal = (s32)(((*(data_buf+7))<<24) + ((*(data_buf+8))<<16) + ((*(data_buf+9))<<8) + (*(data_buf+10)));
			ANO_DT_GetParame(_paraNum,_paraVal);
		}
	}
}
void ANO_DT_SendCmd(u8 dest, u8 fun, u16 cmd1, u16 cmd2, u16 cmd3, u16 cmd4, u16 cmd5)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=dest;
	data_to_send[_cnt++]=0xE0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=fun;
	data_to_send[_cnt++]=BYTE1(cmd1);
	data_to_send[_cnt++]=BYTE0(cmd1);
	data_to_send[_cnt++]=BYTE1(cmd2);
	data_to_send[_cnt++]=BYTE0(cmd2);
	data_to_send[_cnt++]=BYTE1(cmd3);
	data_to_send[_cnt++]=BYTE0(cmd3);
	data_to_send[_cnt++]=BYTE1(cmd4);
	data_to_send[_cnt++]=BYTE0(cmd4);
	data_to_send[_cnt++]=BYTE1(cmd5);
	data_to_send[_cnt++]=BYTE0(cmd5);
	
	data_to_send[4] = _cnt-5;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_SendParame(u16 num)
{
	u8 _cnt=0;
	int32_t data;
	u8 sum = 0;
	u8 i;
	
	if(num > PARNUM)
		return;
	ANO_DT_ParUsedToParList();
	data = ParValList[num];
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0xE1;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(num);
	data_to_send[_cnt++]=BYTE0(num);
	data_to_send[_cnt++]=BYTE3(data);
	data_to_send[_cnt++]=BYTE2(data);
	data_to_send[_cnt++]=BYTE1(data);
	data_to_send[_cnt++]=BYTE0(data);
	
	data_to_send[4] = _cnt-5;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_GetParame(u16 num,s32 data)
{
	if(num > PARNUM)
		return;
	ParValList[num] = data;
	ANO_DT_ParListToParUsed();
	f.paraToSend = num;	//将接收到的参数发回上位机进行双向验证
	data_save();
}
void ANO_DT_ParListToParUsed(void)
{
	Ano_Parame.set.pid_att_1level[ROL][KP] = (float) ParValList[PAR_PID_1_P] / 1000;
	Ano_Parame.set.pid_att_1level[ROL][KI] = (float) ParValList[PAR_PID_1_I] / 1000;
	Ano_Parame.set.pid_att_1level[ROL][KD] = (float) ParValList[PAR_PID_1_D] / 1000;
	Ano_Parame.set.pid_att_1level[PIT][KP] = (float) ParValList[PAR_PID_2_P] / 1000;
	Ano_Parame.set.pid_att_1level[PIT][KI] = (float) ParValList[PAR_PID_2_I] / 1000;
	Ano_Parame.set.pid_att_1level[PIT][KD] = (float) ParValList[PAR_PID_2_D] / 1000;
	Ano_Parame.set.pid_att_1level[YAW][KP] = (float) ParValList[PAR_PID_3_P] / 1000;
	Ano_Parame.set.pid_att_1level[YAW][KI] = (float) ParValList[PAR_PID_3_I] / 1000;
	Ano_Parame.set.pid_att_1level[YAW][KD] = (float) ParValList[PAR_PID_3_D] / 1000;
	
	Ano_Parame.set.pid_att_2level[ROL][KP] = (float) ParValList[PAR_PID_4_P] / 1000;
	Ano_Parame.set.pid_att_2level[ROL][KI] = (float) ParValList[PAR_PID_4_I] / 1000;
	Ano_Parame.set.pid_att_2level[ROL][KD] = (float) ParValList[PAR_PID_4_D] / 1000;
	Ano_Parame.set.pid_att_2level[PIT][KP] = (float) ParValList[PAR_PID_5_P] / 1000;
	Ano_Parame.set.pid_att_2level[PIT][KI] = (float) ParValList[PAR_PID_5_I] / 1000;
	Ano_Parame.set.pid_att_2level[PIT][KD] = (float) ParValList[PAR_PID_5_D] / 1000;
	Ano_Parame.set.pid_att_2level[YAW][KP] = (float) ParValList[PAR_PID_6_P] / 1000;
	Ano_Parame.set.pid_att_2level[YAW][KI] = (float) ParValList[PAR_PID_6_I] / 1000;
	Ano_Parame.set.pid_att_2level[YAW][KD] = (float) ParValList[PAR_PID_6_D] / 1000;
	
	Ano_Parame.set.pid_alt_1level[KP] = (float) ParValList[PAR_PID_7_P] / 1000;
	Ano_Parame.set.pid_alt_1level[KI] = (float) ParValList[PAR_PID_7_I] / 1000;
	Ano_Parame.set.pid_alt_1level[KD] = (float) ParValList[PAR_PID_7_D] / 1000;
	Ano_Parame.set.pid_alt_2level[KP] = (float) ParValList[PAR_PID_8_P] / 1000;
	Ano_Parame.set.pid_alt_2level[KI] = (float) ParValList[PAR_PID_8_I] / 1000;
	Ano_Parame.set.pid_alt_2level[KD] = (float) ParValList[PAR_PID_8_D] / 1000;
	
	Ano_Parame.set.pid_loc_1level[KP] = (float) ParValList[PAR_PID_9_P] / 1000; 
	Ano_Parame.set.pid_loc_1level[KI] = (float) ParValList[PAR_PID_9_I] / 1000; 
	Ano_Parame.set.pid_loc_1level[KD] = (float) ParValList[PAR_PID_9_D] / 1000; 
	Ano_Parame.set.pid_loc_2level[KP] = (float) ParValList[PAR_PID_10_P] / 1000; 
	Ano_Parame.set.pid_loc_2level[KI] = (float) ParValList[PAR_PID_10_I] / 1000; 
	Ano_Parame.set.pid_loc_2level[KD] = (float) ParValList[PAR_PID_10_D] / 1000; 
	
	Ano_Parame.set.pid_gps_loc_1level[KP] = (float) ParValList[PAR_PID_11_P] / 1000; 
	Ano_Parame.set.pid_gps_loc_1level[KI] = (float) ParValList[PAR_PID_11_I] / 1000; 
	Ano_Parame.set.pid_gps_loc_1level[KD] = (float) ParValList[PAR_PID_11_D] / 1000; 
	Ano_Parame.set.pid_gps_loc_2level[KP] = (float) ParValList[PAR_PID_12_P] / 1000; 
	Ano_Parame.set.pid_gps_loc_2level[KI] = (float) ParValList[PAR_PID_12_I] / 1000; 
	Ano_Parame.set.pid_gps_loc_2level[KD] = (float) ParValList[PAR_PID_12_D] / 1000; 
	
	Ano_Parame.set.pid_velocity[KP] = (float) ParValList[PAR_PID_13_P] / 100; 
	Ano_Parame.set.pid_velocity[KI] = (float) ParValList[PAR_PID_13_I] / 100; 
	Ano_Parame.set.pid_velocity[KD] = (float) ParValList[PAR_PID_13_D] / 100; 
	
	Ano_Parame.set.pid_gyroz_11evel[KP] = (float) ParValList[PAR_PID_14_P] / 1000; 
	Ano_Parame.set.pid_gyroz_11evel[KI] = (float) ParValList[PAR_PID_14_I] / 1000; 
	Ano_Parame.set.pid_gyroz_11evel[KD] = (float) ParValList[PAR_PID_14_D] / 1000; 
	
	Ano_Parame.set.pid_gyroz_21evel[KP] = (float) ParValList[PAR_PID_15_P] / 1000; 
	Ano_Parame.set.pid_gyroz_21evel[KI] = (float) ParValList[PAR_PID_15_I] / 1000; 
	Ano_Parame.set.pid_gyroz_21evel[KD] = (float) ParValList[PAR_PID_15_D] / 1000; 
	
	Ano_Parame.set.pid_stable_11evel[KP] = (float) ParValList[PAR_PID_16_P] / 100; 
	Ano_Parame.set.pid_stable_11evel[KI] = (float) ParValList[PAR_PID_16_I] / 100; 
	Ano_Parame.set.pid_stable_11evel[KD] = (float) ParValList[PAR_PID_16_D] / 100; 
	
	Ano_Parame.set.pid_stable_21evel[KP] = (float) ParValList[PAR_PID_17_P] / 100; 
	Ano_Parame.set.pid_stable_21evel[KI] = (float) ParValList[PAR_PID_17_I] / 100; 
	Ano_Parame.set.pid_stable_21evel[KD] = (float) ParValList[PAR_PID_17_D] / 100; 
	
	Ano_Parame.set.pid_laser_level[KP] = (float) ParValList[PAR_PID_18_P] / 1000; 
	Ano_Parame.set.pid_laser_level[KI] = (float) ParValList[PAR_PID_18_I] / 1000; 
	Ano_Parame.set.pid_laser_level[KD] = (float) ParValList[PAR_PID_18_D] / 1000; 
	
}
void ANO_DT_ParUsedToParList(void)
{
	ParValList[PAR_PID_1_P] = Ano_Parame.set.pid_att_1level[ROL][KP] * 1000;
	ParValList[PAR_PID_1_I] = Ano_Parame.set.pid_att_1level[ROL][KI] * 1000;
	ParValList[PAR_PID_1_D] = Ano_Parame.set.pid_att_1level[ROL][KD] * 1000;
	ParValList[PAR_PID_2_P] = Ano_Parame.set.pid_att_1level[PIT][KP] * 1000;
	ParValList[PAR_PID_2_I] = Ano_Parame.set.pid_att_1level[PIT][KI] * 1000;
	ParValList[PAR_PID_2_D] = Ano_Parame.set.pid_att_1level[PIT][KD] * 1000;
	ParValList[PAR_PID_3_P] = Ano_Parame.set.pid_att_1level[YAW][KP] * 1000;
	ParValList[PAR_PID_3_I] = Ano_Parame.set.pid_att_1level[YAW][KI] * 1000;
	ParValList[PAR_PID_3_D] = Ano_Parame.set.pid_att_1level[YAW][KD] * 1000;
	
	ParValList[PAR_PID_4_P] = Ano_Parame.set.pid_att_2level[ROL][KP] * 1000;
	ParValList[PAR_PID_4_I] = Ano_Parame.set.pid_att_2level[ROL][KI] * 1000;
	ParValList[PAR_PID_4_D] = Ano_Parame.set.pid_att_2level[ROL][KD] * 1000;
	ParValList[PAR_PID_5_P] = Ano_Parame.set.pid_att_2level[PIT][KP] * 1000;
	ParValList[PAR_PID_5_I] = Ano_Parame.set.pid_att_2level[PIT][KI] * 1000;
	ParValList[PAR_PID_5_D] = Ano_Parame.set.pid_att_2level[PIT][KD] * 1000;
	ParValList[PAR_PID_6_P] = Ano_Parame.set.pid_att_2level[YAW][KP] * 1000;
	ParValList[PAR_PID_6_I] = Ano_Parame.set.pid_att_2level[YAW][KI] * 1000;
	ParValList[PAR_PID_6_D] = Ano_Parame.set.pid_att_2level[YAW][KD] * 1000;
	
	ParValList[PAR_PID_7_P] = Ano_Parame.set.pid_alt_1level[KP] * 1000;
	ParValList[PAR_PID_7_I] = Ano_Parame.set.pid_alt_1level[KI] * 1000;
	ParValList[PAR_PID_7_D] = Ano_Parame.set.pid_alt_1level[KD] * 1000;
	ParValList[PAR_PID_8_P] = Ano_Parame.set.pid_alt_2level[KP] * 1000;
	ParValList[PAR_PID_8_I] = Ano_Parame.set.pid_alt_2level[KI] * 1000;
	ParValList[PAR_PID_8_D] = Ano_Parame.set.pid_alt_2level[KD] * 1000;
	
	ParValList[PAR_PID_9_P] = Ano_Parame.set.pid_loc_1level[KP] * 1000;
	ParValList[PAR_PID_9_I] = Ano_Parame.set.pid_loc_1level[KI] * 1000;
	ParValList[PAR_PID_9_D] = Ano_Parame.set.pid_loc_1level[KD] * 1000;
	ParValList[PAR_PID_10_P] = Ano_Parame.set.pid_loc_2level[KP] * 1000;
	ParValList[PAR_PID_10_I] = Ano_Parame.set.pid_loc_2level[KI] * 1000;
	ParValList[PAR_PID_10_D] = Ano_Parame.set.pid_loc_2level[KD] * 1000;

	ParValList[PAR_PID_11_P] = Ano_Parame.set.pid_gps_loc_1level[KP] * 1000;
	ParValList[PAR_PID_11_I] = Ano_Parame.set.pid_gps_loc_1level[KI] * 1000;
	ParValList[PAR_PID_11_D] = Ano_Parame.set.pid_gps_loc_1level[KD] * 1000;
	ParValList[PAR_PID_12_P] = Ano_Parame.set.pid_gps_loc_2level[KP] * 1000;
	ParValList[PAR_PID_12_I] = Ano_Parame.set.pid_gps_loc_2level[KI] * 1000;
	ParValList[PAR_PID_12_D] = Ano_Parame.set.pid_gps_loc_2level[KD] * 1000;
	
	ParValList[PAR_PID_13_P] = Ano_Parame.set.pid_velocity[KP] * 100;
	ParValList[PAR_PID_13_I] = Ano_Parame.set.pid_velocity[KI] * 100;
	ParValList[PAR_PID_13_D] = Ano_Parame.set.pid_velocity[KD] * 100;
	
	ParValList[PAR_PID_14_P] = Ano_Parame.set.pid_gyroz_11evel[KP] * 1000;
	ParValList[PAR_PID_14_I] = Ano_Parame.set.pid_gyroz_11evel[KI] * 1000;
	ParValList[PAR_PID_14_D] = Ano_Parame.set.pid_gyroz_11evel[KD] * 1000;
	
	ParValList[PAR_PID_15_P] = Ano_Parame.set.pid_gyroz_21evel[KP] * 1000;
	ParValList[PAR_PID_15_I] = Ano_Parame.set.pid_gyroz_21evel[KI] * 1000;
	ParValList[PAR_PID_15_D] = Ano_Parame.set.pid_gyroz_21evel[KD] * 1000;
	
	ParValList[PAR_PID_16_P] = Ano_Parame.set.pid_stable_11evel[KP] * 100;
	ParValList[PAR_PID_16_I] = Ano_Parame.set.pid_stable_11evel[KI] * 100;
	ParValList[PAR_PID_16_D] = Ano_Parame.set.pid_stable_11evel[KD] * 100;
	
	ParValList[PAR_PID_17_P] = Ano_Parame.set.pid_stable_21evel[KP] * 100;
	ParValList[PAR_PID_17_I] = Ano_Parame.set.pid_stable_21evel[KI] * 100;
	ParValList[PAR_PID_17_D] = Ano_Parame.set.pid_stable_21evel[KD] * 100;
	
	ParValList[PAR_PID_18_P] = Ano_Parame.set.pid_laser_level[KP] * 1000;
	ParValList[PAR_PID_18_I] = Ano_Parame.set.pid_laser_level[KI] * 1000;
	ParValList[PAR_PID_18_D] = Ano_Parame.set.pid_laser_level[KD] * 1000;
}


void ANO_DT_SendString(const char *str)
{
	u8 _cnt=0;
	u8 j = 0;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0xA0;
	data_to_send[_cnt++]=0;

	while(*(str+j) != '\0')
	{
		data_to_send[_cnt++] = *(str+j++);
		if(_cnt > 50)
			break;
	}
	data_to_send[4] = _cnt-5;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

/////////////////////////////////MPU6050
void ANODT_Send_MPU(float angle_rol, float angle_pit, float angle_yaw, s16 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE1(alt);
	data_to_send[_cnt++]=BYTE0(alt);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[4] = _cnt-5;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//加速度计以及角速度
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[4] = _cnt-5;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}



//电机PWM
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	
	data_to_send[4] = _cnt-5;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//坐标dist_x,dist_y
void ANO_DT_Send_Dist(u16 dist_x,u16 dist_y)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0xf2;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(dist_x);
	data_to_send[_cnt++]=BYTE0(dist_x);
	data_to_send[_cnt++]=BYTE1(dist_y);
	data_to_send[_cnt++]=BYTE0(dist_y);

	data_to_send[4] = _cnt-5;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//用户自定义数据发送

void ANODT_SendF1(s16 speeda, s16 speedb, s16 speedc, s16 speedd, s16 xspeed, 
									s16 expect_value, u8 step_sta, s16 record_yaw, s16 kalmanyaw2,
									u8 rubbish_type, u16 x_pix, u16 y_pix, u16 bottom)  //13
{
	u8 _cnt = 0;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = MYHWADDR;
	data_to_send[_cnt++] = SWJADDR;
	data_to_send[_cnt++] = 0xf1;		//功能帧
	data_to_send[_cnt++] = 0;
	/*speed*/
	data_to_send[_cnt++] = BYTE1(speeda);
	data_to_send[_cnt++] = BYTE0(speeda);
	
	data_to_send[_cnt++] = BYTE1(speedb);
	data_to_send[_cnt++] = BYTE0(speedb);

	data_to_send[_cnt++] = BYTE1(speedc);
	data_to_send[_cnt++] = BYTE0(speedc);
	
	data_to_send[_cnt++] = BYTE1(speedd);
	data_to_send[_cnt++] = BYTE0(speedd);
	
	data_to_send[_cnt++] = BYTE1(xspeed);
	data_to_send[_cnt++] = BYTE0(xspeed);
	/*speed*/
	
	/*record*/
	data_to_send[_cnt++] = BYTE1(expect_value);
	data_to_send[_cnt++] = BYTE0(expect_value);
	
	data_to_send[_cnt++] = BYTE0(step_sta);
	
	data_to_send[_cnt++] = BYTE1(record_yaw);
	data_to_send[_cnt++] = BYTE0(record_yaw);
	
	data_to_send[_cnt++] = BYTE1(kalmanyaw2);
	data_to_send[_cnt++] = BYTE0(kalmanyaw2);
	/*record*/
	
	/*nano.h*/
	data_to_send[_cnt++] = BYTE0(rubbish_type);
	
	data_to_send[_cnt++] = BYTE1(x_pix);
	data_to_send[_cnt++] = BYTE0(x_pix);
	
	data_to_send[_cnt++] = BYTE1(y_pix);
	data_to_send[_cnt++] = BYTE0(y_pix);
	
	data_to_send[_cnt++] = BYTE1(bottom);
	data_to_send[_cnt++] = BYTE0(bottom);
	/*nano.h*/
	data_to_send[4] = _cnt-5;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
