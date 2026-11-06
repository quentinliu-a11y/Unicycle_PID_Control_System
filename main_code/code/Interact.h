/*---------------------------------
*   File name: Interact.h
*	
*	Author: LaZuli
*
*	Created time: 2024年4月11日
---------------------------------*/
#ifndef CODE_INTERACT_H_
#define CODE_INTERACT_H_

#include "zf_common_headfile.h"

/*-----------结构体初始化-----------*/
typedef struct PID_Changed{
        float Kp;
        float Ki;
        float Kd;
        float Err;
        float Err_last;
        float Err_l2st;  //前前次误差
        float Integral_lim; //积分限幅
        float Err_Int;  //误差积分
        float P_out;
        float I_out;
        float D_out;
        float Out;
        float Out_lim;

}PID_Changed;
/*--------------------------------*/

/*-----------枚举数组初始化-----------*/
enum Key_Mode{
    Click,
    Hold,
    None
};

enum Modify_Mode{
    Wireless,
    Button
};
/*--------------------------------*/

/*-----------函数声明-----------*/
void Go(void);
void Menu_Init(void);
void Buzzer(uint8 value);
void Menu_Display(void);
enum Key_Mode GetKeyMode(gpio_pin_enum pin);
/*--------------------------------*/

extern char Start_Flag;

#endif /* CODE_INTERACT_H_ */
