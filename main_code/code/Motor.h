/*---------------------------------
*   File name: Motor.h
*
*   Author: LaZuli
*
*   Created time: 2024年4月10日
---------------------------------*/
#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"

#define DIN1 P11_2
#define DIN2 P11_3
#define DIN_ON  gpio_set_level(DIN1, 0); gpio_set_level(DIN2, 0);        // 启动（刹车线）
#define DIN_OFF gpio_set_level(DIN1, 1); gpio_set_level(DIN2, 1);        // 停止（刹车线）

void Motor_Init(void);
void Encoder_Init(void);

void Motor_Control(sint32 motor1, sint32 motor2, sint32 motor3);
void Test_Motor(sint32 motor1, sint32 motor2, sint32 motor3);

#endif
