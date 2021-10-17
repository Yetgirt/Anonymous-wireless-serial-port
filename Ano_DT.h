#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "sys.h"
#include "encoder.h"
#include "control.h"
#include "my_mpu6050.h"
#include "My_usart.h"
#include "tf_minis.h"
#include "Kalman.h"
#include "nano.h"

typedef struct
{
  u8 send_mpu;
  u8 send_senser;
  u8 send_motopwm;
  u8 send_dist;
  u8 send_user;
  u8 send_speed;
	u16 send_parame;
	u16 paraToSend;
} dt_flag_t;

#define PAR_PID_1_P		1
#define PAR_PID_1_I		2
#define PAR_PID_1_D		3
#define PAR_PID_2_P		4
#define PAR_PID_2_I		5
#define PAR_PID_2_D		6
#define PAR_PID_3_P		7
#define PAR_PID_3_I		8
#define PAR_PID_3_D		9
#define PAR_PID_4_P		10
#define PAR_PID_4_I		11
#define PAR_PID_4_D		12
#define PAR_PID_5_P		13
#define PAR_PID_5_I		14
#define PAR_PID_5_D		15
#define PAR_PID_6_P		16
#define PAR_PID_6_I		17
#define PAR_PID_6_D		18
#define PAR_PID_7_P		19
#define PAR_PID_7_I		20
#define PAR_PID_7_D		21
#define PAR_PID_8_P		22
#define PAR_PID_8_I		23
#define PAR_PID_8_D		24
#define PAR_PID_9_P		25
#define PAR_PID_9_I		26
#define PAR_PID_9_D		27
#define PAR_PID_10_P		28
#define PAR_PID_10_I		29
#define PAR_PID_10_D		30
#define PAR_PID_11_P		31
#define PAR_PID_11_I		32
#define PAR_PID_11_D		33
#define PAR_PID_12_P		34
#define PAR_PID_12_I		35
#define PAR_PID_12_D		36
#define PAR_PID_13_P		37
#define PAR_PID_13_I		38
#define PAR_PID_13_D		39
#define PAR_PID_14_P		40
#define PAR_PID_14_I		41
#define PAR_PID_14_D		42
#define PAR_PID_15_P		43
#define PAR_PID_15_I		44
#define PAR_PID_15_D		45
#define PAR_PID_16_P		46
#define PAR_PID_16_I		47
#define PAR_PID_16_D		48
#define PAR_PID_17_P		49
#define PAR_PID_17_I		50
#define PAR_PID_17_D		51
#define PAR_PID_18_P		52
#define PAR_PID_18_I		53
#define PAR_PID_18_D		54

extern s32 ParValList[100];

extern dt_flag_t f;


void ANO_DT_Data_Exchange(u8 dT_ms);

void ANO_DT_Data_Receive_Prepare(u8 data);
void ANO_DT_Data_Receive_Anl_Task(void);
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);

void ANO_DT_SendCmd(u8 dest, u8 fun, u16 cmd1, u16 cmd2, u16 cmd3, u16 cmd4, u16 cmd5);
void ANO_DT_SendString(const char *str);

void ANO_DT_SendParame(u16 num);
void ANO_DT_GetParame(u16 num,s32 data);
void ANO_DT_ParListToParUsed(void);
void ANO_DT_ParUsedToParList(void);
/////////////////////////////////////////////////////////
void ANODT_Send_MPU(float angle_rol, float angle_pit, float angle_yaw, s16 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z);
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5);
void ANO_DT_Send_Dist(u16 dist_x,u16 dist_y);
void ANODT_SendF1(s16 speeda, s16 speedb, s16 speedc, s16 speedd, s16 xspeed, 
									s16 expect_value, u8 step_sta, s16 record_yaw, s16 kalmanyaw2,
									u8 rubbish_type, u16 x_pix, u16 y_pix, u16 bottom);
/////////////////////////////////////////////////////////

#endif

