/*---------------------------------
*   File name: Init.c
*
*   Author: LaZuli
*
*   Created time: 2024年4月10日
---------------------------------*/

#include "Init.h"

void Init(){
    //LED初始化
    gpio_init(P21_4, GPO, 0, GPO_PUSH_PULL);
    gpio_init(P20_9, GPO, 1, GPO_PUSH_PULL);
    //按键初始化
    gpio_init(Key1, GPI, 0, GPI_PULL_DOWN);
    gpio_init(Key2, GPI, 0, GPI_PULL_DOWN);
    gpio_init(Key3, GPI, 0, GPI_PULL_DOWN);
    gpio_init(Key4, GPI, 0, GPI_PULL_DOWN);
    gpio_init(Key5, GPI, 0, GPI_PULL_DOWN);
    //蜂鸣器初始化
    gpio_init(P33_10, GPO, 0, GPO_PUSH_PULL);
    Buzzer(0);
    //菜单初始化
    menu_init();
    //陀螺仪初始化
//    imu963ra_init();
//    low_pass_filter_init();
    imu660ra_init();
    IMU_gyro_Offset_Init();
    //电机初始化
    Motor_Init();
    Motor_Control(0,0,0);
    //编码器初始化
    Encoder_Init();
    //无线串口初始化
//    wireless_uart_init();
    //1ms中断初始化
    pit_ms_init(CCU60_CH0, 1);
    pit_ms_init(CCU61_CH0, 1);                            // GPS数据解析中断/霍尔编码器中断
//    pit_ms_init(CCU60_CH1, 1);                              // 遥控器
    //逐飞助手初始化
//    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    //延时函数初始化
    system_delay_init();
    //GPS初始化
//    gnss_init(TAU1201);
    //PID初始化
    PID_Freeze_Init();
    //卡尔曼滤波器初始化
    KalmanCreate(&Kalman1,0.5,85);
    KalmanCreate(&Kalman2,10,156);
    //tof初始化
    dl1b_init();
    // 点阵屏幕初始化
    dot_matrix_screen_init();       // 点阵屏幕初始化
    //遥控器初始化
    lora3a22_init();

}

void Core_Ready(){

    for(float i = 0;i < 5;i++){
        Buzzer(1);
        gpio_set_level(P20_9,1);
        system_delay_ms(100);
        Buzzer(0);
        gpio_set_level(P20_9,0);
        system_delay_ms(100);
    }
}
