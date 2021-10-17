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
	u16 frist_init;	//�ɿص�һ�γ�ʼ������Ҫ��һЩ���⹤�����������flash
	
	float 	pid_att_1level[VEC_RPY][PID]; //��̬���ƽ��ٶȻ�PID����
	float 	pid_att_2level[VEC_RPY][PID]; //��̬���ƽǶȻ�PID����
	float 	pid_alt_1level[PID];          //�߶ȿ��Ƹ߶��ٶȻ�PID����
	float 	pid_alt_2level[PID];           //�߶ȿ��Ƹ߶Ȼ�PID����
	float 	pid_loc_1level[PID];          //λ�ÿ���λ���ٶȻ�PID����
	float 	pid_loc_2level[PID];           //λ�ÿ���λ�û�PID����

	float 	pid_gps_loc_1level[PID];          //tinyλ�ÿ���λ���ٶȻ�PID����
	float 	pid_gps_loc_2level[PID];           //tinyλ�ÿ���λ�û�PID����

	float		pid_velocity[PID];									//����������С���ٶȻ�PID����
	float		pid_gyroz_11evel[PID];							//�Ƕ��⻷����PID����
	float		pid_gyroz_21evel[PID];							//���ٶ��ڻ�����PID����
	
	float 	pid_stable_11evel[PID];							//��̨�Ƕ��⻷����PID����
	float 	pid_stable_21evel[PID];							//��̨�Ƕ��ڻ�����PID����
	
	float 	pid_laser_level[PID];										  //������λ��PID����
};

union Parameter
{
	//����ʹ�������壬������4KByte���������ڲ���һ���ṹ�壬�ýṹ��������Ҫ����Ĳ���
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

