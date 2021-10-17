#ifndef _PARAMETER_H
#define	_PARAMETER_H

#include "sys.h"
#include "Ano_DT.h"
#include "Drv_w25qxx.h"
#include "led.h"

#define SOFT_VER 17
enum
{
	KP = 0,
	KI = 1,
	KD = 2,
	PID,
};

enum
{
	ROL = 0,
	PIT = 1,
	YAW = 2,
	VEC_RPY,
};

__packed struct Parameter_s
{
	u16 frist_init;	//飞控第一次初始化，需要做一些特殊工作，比如清空flash
	
	float 	pid_att_1level[VEC_RPY][PID]; //姿态控制角速度环PID参数
	float 	pid_att_2level[VEC_RPY][PID]; //姿态控制角度环PID参数
	float 	pid_alt_1level[PID];          //高度控制高度速度环PID参数
	float 	pid_alt_2level[PID];           //高度控制高度环PID参数
	float 	pid_loc_1level[PID];          //位置控制位置速度环PID参数
	float 	pid_loc_2level[PID];           //位置控制位置环PID参数

	float 	pid_gps_loc_1level[PID];          //tiny位置控制位置速度环PID参数
	float 	pid_gps_loc_2level[PID];           //tiny位置控制位置环PID参数

	float		pid_velocity[PID];									//编码器控制小车速度环PID参数
	float		pid_gyroz_11evel[PID];							//角度外环控制PID参数
	float		pid_gyroz_21evel[PID];							//角速度内环控制PID参数
	
	float 	pid_stable_11evel[PID];							//云台角度外环控制PID参数
	float 	pid_stable_21evel[PID];							//云台角度内环控制PID参数
	
	float 	pid_laser_level[PID];										  //激光测距位置PID参数
};

union Parameter
{
	//这里使用联合体，长度是4KByte，联合体内部是一个结构体，该结构体内是需要保存的参数
	struct Parameter_s set;
	u8 byte[4096];
};
extern union Parameter Ano_Parame;

typedef struct
{
	u8 save_en;
	u8 save_trig;
	u16 time_delay;
}_parameter_state_st ;
extern _parameter_state_st para_sta;

/////////////////////////////////////////////////////////////////////////////
void data_save(void);
void Para_Data_Init(void);
void Ano_Parame_Read(void);
void Ano_Parame_Write_task(u16 dT_ms);
void PID_Rest(void);
void Parame_Reset(void);

#endif 

