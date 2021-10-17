/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
  * 作者   ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：参数配置等
**********************************************************************************/
#include "Ano_Parameter.h"

union Parameter Ano_Parame;
_parameter_state_st para_sta;

void data_save(void)
{
	para_sta.save_en = 1;
	para_sta.save_trig = 1;
}

void Para_Data_Init()
{
	Ano_Parame_Read();
}

void PID_Rest()
{
//---	姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[ROL][KP] = 0.0f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[ROL][KI] = 0.0f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[ROL][KD] = 0.0f; //姿态控制角速度环PID参数
	
	Ano_Parame.set.pid_att_1level[PIT][KP] = 0.0f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[PIT][KI] = 0.0f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[PIT][KD] = 0.0f; //姿态控制角速度环PID参数
	
	Ano_Parame.set.pid_att_1level[YAW][KP] = 0.0f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[YAW][KI] = 0.0f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[YAW][KD] = 0.0f; //姿态控制角速度环PID参数
//---	姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[ROL][KP] = 0.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[ROL][KI] = 0.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[ROL][KD] = 0.0f; //姿态控制角度环PID参数
	
	Ano_Parame.set.pid_att_2level[PIT][KP] = 0.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[PIT][KI] = 0.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[PIT][KD] = 0.0f; //姿态控制角度环PID参数
	
	Ano_Parame.set.pid_att_2level[YAW][KP] = 0.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[YAW][KI] = 0.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[YAW][KD] = 0.0f; //姿态控制角度环PID参数	
//---	高度控制高度速度环PID参数	
	Ano_Parame.set.pid_alt_1level[KP] = 0.0f;          //高度控制高度速度环PID参数
	Ano_Parame.set.pid_alt_1level[KI] = 0.0f;          //高度控制高度速度环PID参数
	Ano_Parame.set.pid_alt_1level[KD] = 0.0f;          //高度控制高度速度环PID参数
//---	高度控制高度环PID参数
	Ano_Parame.set.pid_alt_2level[KP] = 0.13f;           //高度控制高度环PID参数
	Ano_Parame.set.pid_alt_2level[KI] = 0.0f;           //高度控制高度环PID参数(NULL)
	Ano_Parame.set.pid_alt_2level[KD] = 0.5f;           //高度控制高度环PID参数(NULL)
//---	位置控制位置速度环PID参数	
	Ano_Parame.set.pid_loc_1level[KP] = 0.13f;          //位置控制位置速度环PID参数1.0
	Ano_Parame.set.pid_loc_1level[KI] = 0.0f;          //位置控制位置速度环PID参数
	Ano_Parame.set.pid_loc_1level[KD] = 0.5f;          //位置控制位置速度环PID参数0.23
//---	位置控制位置环PID参数
	Ano_Parame.set.pid_loc_2level[KP] = 2.4f;           //位置控制位置环PID参数(NULL)
	Ano_Parame.set.pid_loc_2level[KI] = 0.0f;           //位置控制位置环PID参数(NULL)
	Ano_Parame.set.pid_loc_2level[KD] = 0.5f;           //位置控制位置环PID参数(NULL)
//---	GPS位置控制位置速度环PID参数	
	Ano_Parame.set.pid_gps_loc_1level[KP] = 2.0f;          //位置控制位置速度环PID参数 1.2
	Ano_Parame.set.pid_gps_loc_1level[KI] = 0.0f;          //位置控制位置速度环PID参数 00
	Ano_Parame.set.pid_gps_loc_1level[KD] = 2.0f;          //位置控制位置速度环PID参数0.01
//---	GPS位置控制位置环PID参数
	Ano_Parame.set.pid_gps_loc_2level[KP] = 1.2f;           //位置控制位置环PID参数
	Ano_Parame.set.pid_gps_loc_2level[KI] = 0.0f;           //位置控制位置环PID参数(NULL)0.1
	Ano_Parame.set.pid_gps_loc_2level[KD] = 1.3f;           //位置控制位置环PID参数(NULL)
//--- 编码器控制小车速度环PID参数
	Ano_Parame.set.pid_velocity[KP] = 40.0f;			//旋转杆位置控制位置环PID参数		60 12.5 	0.85  12  3.5  1.4
	Ano_Parame.set.pid_velocity[KI] = 1.8f;			//旋转杆位置控制位置环PID参数			7.0             7
	Ano_Parame.set.pid_velocity[KD] = 90.0f;			//旋转杆位置控制位置环PID参数                   0.85
//--- 角度外环PID参数
	Ano_Parame.set.pid_gyroz_11evel[KP] = 4.05f;			//旋转杆位置控制位置环PID参数				8.0   4.5
	Ano_Parame.set.pid_gyroz_11evel[KI] = 0.0f;					//旋转杆位置控制位置环PID参数
	Ano_Parame.set.pid_gyroz_11evel[KD] = 0.3f;					//旋转杆位置控制位置环PID参数
//---	角速度内环PID参数
	Ano_Parame.set.pid_gyroz_21evel[KP] = 1.6f;
	Ano_Parame.set.pid_gyroz_21evel[KI] = 0.17f;				//
	Ano_Parame.set.pid_gyroz_21evel[KD] = 1.8f;
//---	云台角度环PID参数
	Ano_Parame.set.pid_stable_11evel[KP] = 5.0f; // 		14.0  0.0  1.2
	Ano_Parame.set.pid_stable_11evel[KI] = 0.0f;
	Ano_Parame.set.pid_stable_11evel[KD] = 5.5f;
//---	云台角速度环PID参数
	Ano_Parame.set.pid_stable_21evel[KP] = 13.01f; // 8.0 0.0 0.05
	Ano_Parame.set.pid_stable_21evel[KI] = 0.0f;
	Ano_Parame.set.pid_stable_21evel[KD] = 25.50f;
//---	定杆位置环PID参数
	Ano_Parame.set.pid_laser_level[KP] = 4.05f;
	Ano_Parame.set.pid_laser_level[KI] = 0.0f;				//0.52
	Ano_Parame.set.pid_laser_level[KD] = 0.3f;
	
	ANO_DT_SendString("PID reset!");
}

static void Ano_Parame_Write(void)
{	
	Ano_Parame.set.frist_init = SOFT_VER;
	
	Flash_SectorErase ( 0x000000, 1 );							//擦除第一扇区
	Flash_SectorsWrite ( 0x000000, &Ano_Parame.byte[0], 1 );	//将参数写入第一扇区
}

void Ano_Parame_Read(void)
{
	Flash_SectorsRead ( 0x000000, &Ano_Parame.byte[0], 1 );		//读取第一扇区内的参数
	
	if(Ano_Parame.set.frist_init != SOFT_VER)	//内容没有被初始化，则进行参数初始化工作
	{		
		PID_Rest();
		Ano_Parame_Write();
	}
}

void Ano_Parame_Write_task(u16 dT_ms)
{
	//因为写入flash耗时较长，我们飞控做了一个特殊逻辑，在解锁后，是不进行参数写入的，此时会置一个需要写入标志位，等飞机降落锁定后，再写入参数，提升飞行安全性
	//为了避免连续更新两个参数，造成flash写入两次，我们飞控加入一个延时逻辑，参数改变后三秒，才进行写入操作，可以一次写入多项参数，降低flash擦写次数
	if(para_sta.save_en )				//允许存储
	{
		if(para_sta.save_trig == 1) 	//如果触发存储标记1
		{
			GPIO_ResetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10设置低，灯亮
			para_sta.time_delay = 0;  	//计时复位
			para_sta.save_trig = 2;   	//触发存储标记2
		}
		
		if(para_sta.save_trig == 2) 	//如果触发存储标记2
		{
			if(para_sta.time_delay<3000) //计时小于3000ms
			{
				para_sta.time_delay += dT_ms; //计时
			}
			else
			{
				para_sta.save_trig = 0;  //存储标记复位
				Ano_Parame_Write();      //执行存储
				ANO_DT_SendString("Set save OK!");
				GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10设置高，灯灭
			}
		}
		else
		{
			para_sta.time_delay = 0;
		}
		
	}
	else
	{
		para_sta.time_delay = 0;
		para_sta.save_trig = 0;
	}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
