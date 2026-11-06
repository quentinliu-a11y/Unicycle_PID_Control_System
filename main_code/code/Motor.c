/*---------------------------------
*   File name: Motor.c
*
*   Author: LaZuli
*
*   Created time: 2024年4月10日
---------------------------------*/
#include "Motor.h"

void Motor_Init(){

    gpio_init(P02_4, GPO, 0, GPO_PUSH_PULL);
    gpio_init(P02_6, GPO, 0, GPO_PUSH_PULL);
//    gpio_init(P21_4, GPO, 0, GPO_PUSH_PULL);
    pwm_init(ATOM1_CH5_P02_5,12500,0);
    pwm_init(ATOM1_CH7_P02_7,12500,0);
    pwm_init(ATOM0_CH0_P21_2,30000,0);
    pwm_init(ATOM0_CH2_P21_4,30000,0);

}
void Encoder_Init(void){

    encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7,TIM2_ENCODER_CH2_P33_6 );  // 前无刷电机
    encoder_dir_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8 , TIM4_ENCODER_CH2_P00_9);  // 有刷电机
    encoder_quad_init(TIM5_ENCODER, TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);  // 后无刷电机

    encoder_clear_count(TIM2_ENCODER);
    encoder_clear_count(TIM4_ENCODER);
    encoder_clear_count(TIM5_ENCODER);
}

void Motor_Control(sint32 motor1, sint32 motor2, sint32 motor3){
           if (motor1 > 0)
           {
                pwm_set_duty(ATOM1_CH5_P02_5, 10000-motor1);//P02.5
                gpio_set_level(P02_4,0);
           }
           else
           {
               pwm_set_duty(ATOM1_CH5_P02_5, (10000+motor1));
               gpio_set_level(P02_4,1);
           }

           if (motor2 > 0)
           {
               pwm_set_duty(ATOM1_CH7_P02_7, 10000-motor2);
               gpio_set_level(P02_6,0);
           }
           else
           {
               pwm_set_duty(ATOM1_CH7_P02_7, (10000+motor2));
               gpio_set_level(P02_6,1);
           }

           if(motor3>3500)motor3=3500;
           if(motor3<-3500)motor3=-3500;
           if (motor3 > 0)
           {
               pwm_set_duty(ATOM0_CH0_P21_2, motor3);
               pwm_set_duty(ATOM0_CH2_P21_4, 0);
//               gpio_set_level(P21_4,1);
    //           IfxPort_setPinLow(&MODULE_P23, 1);
           }
           else
           {
               pwm_set_duty(ATOM0_CH0_P21_2, 0);
               pwm_set_duty(ATOM0_CH2_P21_4, (0-motor3));
    //           IfxPort_setPinLow(&MODULE_P23, 1);
           }
}
void Test_Motor(sint32 motor1, sint32 motor2, sint32 motor3){

    pwm_set_duty(ATOM1_CH5_P02_5, -motor1);//P02.5
    gpio_set_level(P02_4,0);

    pwm_set_duty(ATOM1_CH7_P02_7, motor2);
    gpio_set_level(P02_6,0);

}
