/*---------------------------------
*   File name: Control.h
*	
*	Author: LaZuli
*
*	Created time: 2024年4月11日
---------------------------------*/
#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"

#define K1 -0.000045//125
#define K2 0.00009//125
/*-----------结构体初始化-----------*/
/*NaN*/
/*--------------------------------*/

/*-----------枚举数组初始化-----------*/
/*NaN*/
/*--------------------------------*/

/*-----------函数声明-----------*/
void Vec_Ctrl(void);
void Speed_loop(void);
float firstOrderFilter(float in_data);
void Angle_Speed_Loop(void);
void Angle_Loop(void);
void Divert_Speed_Loop(void);
void Divert_Angle_Loop(void);
void Divert_Loop(void);
void Motor_PID_Control(void);
char Attitude_stability_judgment(float roll, float pitch, float yaw);
float Dynamic_Zeroing_Pitch(float vec,float angle);
float Dynamic_Zeroing_roll(float vec,float angle);
//float limit(float val, float lower, float upper);
void Turn_Angle(void);
void distance_loop();//
void PID_Freeze_Init(void);
//void PID_Straight_Init(void);
//void PID_Divert_Init(void);
void Speed_ctrl_loop(void);


/*--------------------------------*/

extern PID V;
extern PID VA;
extern PID A;
extern PID G;
extern PID G_Y;
extern PID Speed1;
extern PID Speed2;

extern PID zhuanxiang;
extern PID Turn;
extern PID Divert_Speed;
extern PID Divert_Angle;
extern PID Speed_ctrl;

extern float Vec_Precious;
extern float Vec_BLZ,Vec_BLY;
extern short Duty_L,Duty_R,Duty_X;
extern float Encoder_Total;
extern float Vec_Original;
extern float aim_Encoder;


#endif /* CODE_CONTROL_H_ */
