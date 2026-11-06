/*---------------------------------
*   File name: UI.h
*
*   Author: LaZuli
*
*   Created time: 2024年4月12日
---------------------------------*/
#ifndef CODE_UI_H_
#define CODE_UI_H_

#include "zf_common_headfile.h"

/*-----------结构体初始化-----------*/
typedef struct UI_page                 //定义页面结构体UI_page的别名为menu
{
    uint16 row;
    char date[20];

    void (*Fanction)(void);

    struct UI_page* sibling;
    struct UI_page* child;
} menu;
/*--------------------------------*/

/*-----------外部变量-----------*/
//extern float Angle_X_zero,Angle_Y_zero;
extern float Angle_X_Final,Angle_Y_Final,Angle_Z_Final,Gyro_x;
//extern float gx,gz,gy;
//extern float eulerAngle_yaw,eulerAngle_roll,eulerAngle_pitch;
//extern long Vec_BLZ,Vec_BLY;
//extern float Vec_Precious;
extern PID V;
extern PID VA;
extern PID A;
extern PID G;
extern PID G_Y;
extern PID Speed2;
extern PID zhuanxiang;
extern PID distance;

extern float Vec_Aim;
extern int zhuan;

extern float Angle_Y_zero,Angle_X_zero;

/*--------------------------------*/

/*-----------按键定义-----------*/
#define BEEP_PIN P33_10
#define Key5 P23_1
#define Key4 P22_0
#define Key3 P22_1
#define Key2 P22_2
#define Key1 P22_3
#define Key0 P21_3

/*--------------------------------*/


/*-----------枚举数组初始化-----------*/

/*--------------------------------*/

/*-----------函数声明-----------*/
void menu_init(void);
void choose_menu(void);
void remote_control(void );
char navigation(void);
void navigation_path();

/*--------------------------------*/


#endif /* CODE_UI_H_ */
