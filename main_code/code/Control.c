/*---------------------------------
*   File name: Control.c
*	
*	Author: LaZuli
*
*	Created time: 2024年4月11日
---------------------------------*/
#include "Control.h"

float Angle_X_zero = 0;         //动量轮零点
float Angle_Y_zero = 0;
float Vec_BLZ,Vec_BLY;
int PWM_R,PWM_Y,PWMA_accel,PWMB_accel,PWMC_accel;
float Encoder_Total=0;
float aim_Encoder=0;
/*--------------------------------------------------*/
short Duty_L;
short Duty_R;
short Duty_X;
/*---------------*/

/*-------------串级PID相关--------------*/
float encoder_x = 0;
float Vec_Original = 300;
float Vec_Precious = 0;
float Vec_Aim = 0;
float VF = 0;
PID V;
PID VA;
PID A;
PID G;
PID G_Y;
PID Speed1;
PID Speed2;

PID Divert_Speed;

PID Divert_Angle;
PID Turn;

PID Speed_ctrl;



/*----------------------------------*/

void Angle_Speed_Loop(){

    Pid_Position(A.Out,RAD_TO_ANGLE(IMU_Data.gyro_x),&G);//角速度环
    Pid_Position(VA.Out,RAD_TO_ANGLE(IMU_Data.gyro_y),&G_Y);//行进电机与角速度的闭环PID


    Duty_L = G.Out - Turn.Out;           //左轮占空比
    Duty_R = G.Out + Turn.Out;           //右轮占空比
    Duty_X = G_Y.Out;//-QIAN ;//+ Vec.Out;

    if(Duty_L < -0)Duty_L-=400;     //死区
    else if(Duty_L > 0)Duty_L +=400;//死区
    if(Duty_R < -0)Duty_R-=400;     //死区
    else if(Duty_R > 0)Duty_R +=400;//死区
    if(Duty_X < -0)Duty_X-=40;     //死区
    else if(Duty_X > 0)Duty_X +=40;//死区

}

void Angle_Loop(){
    Pid_Position((Speed1.Out+Speed2.Out)/2+Angle_Y_zero + Dynamic_Zeroing_roll(Vec_Precious,Delta_Yaw-T_N),first_complement.angle.pitch,&A);//角度环
    Pid_Position(V.Out+Angle_X_zero,first_complement.angle.roll,&VA);

//    Pid_Position((Speed1.Out+Speed2.Out)/2+Angle_Y_zero ,first_complement.angle.pitch,&A);//角度环
//    Pid_Position(V.Out+Angle_X_zero,first_complement.angle.roll,&VA);

}

void Vec_Ctrl(void)  //行进轮速度环
{
    float Encoder_Least2;
    static float Encoder2;
    Vec_Precious = -encoder_get_count(TIM4_ENCODER);
    Encoder_Least2 = Vec_Precious;
    Encoder2 *= 0.7;                             //一阶互补滤波
    Encoder2 += Encoder_Least2*0.3;
    encoder_x = Encoder2;
    Pid_Position(Vec_Aim-My_abs(Speed_ctrl.Out)*Encoder2/500,Encoder2,&V);
}

void Speed_loop(void)  //速度环    PI
{
    float Encoder_Least1,Encoder_Least2;
    static float Encoder1,Encoder2;
    Vec_BLZ = encoder_get_count(TIM2_ENCODER); // 动量轮反馈 母板编码器左
    Vec_BLY = -encoder_get_count(TIM5_ENCODER);
//    Encoder_Least = (Vec_BLZ+Vec_BLY)/2;
    Encoder_Least2 = Vec_BLZ;
    Encoder_Least1 = Vec_BLY;
    Encoder1 *= 0.7;             //一阶互补滤波
    Encoder1 += Encoder_Least1*0.3;
    Pid_Position(0,Encoder1,&Speed1);               //期望值为0  输入为编码器采集值
    Pid_Position(0,Encoder2,&Speed2);               //期望值为0  输入为编码器采集值

}

void Divert_Speed_Loop(){
    float Encoder_Least;
    static float Encoder;
    Vec_BLZ = encoder_get_count(TIM2_ENCODER); // 动量轮反馈 母板编码器左
    Vec_BLY = -encoder_get_count(TIM5_ENCODER);
    Encoder_Least = (Vec_BLZ+Vec_BLY)/2;
//    Encoder_Least = ECPULSE2;
//    Encoder_Least = ECPULSE1;
    Encoder *= 0.7;             //一阶互补滤波
    Encoder += Encoder_Least*0.3;
    Pid_Position(0,Encoder,&Divert_Speed);               //期望值为0  输入为编码器采集值



//    float Encoder_Least1,Encoder_Least2;
//    static float Encoder1,Encoder2;
//    Vec_BLZ = encoder_get_count(TIM2_ENCODER); // 动量轮反馈 母板编码器左
//    Vec_BLY = -encoder_get_count(TIM5_ENCODER);
////    Encoder_Least = (Vec_BLZ+Vec_BLY)/2;
//    Encoder_Least2 = Vec_BLZ;
//    Encoder_Least1 = Vec_BLY;
//    Encoder1 *= 0.7;             //一阶互补滤波
//    Encoder1 += Encoder_Least1*0.3;
//    Pid_Position(0,Encoder1,&Divert_Speed1);               //期望值为0  输入为编码器采集值
//    Pid_Position(0,Encoder2,&Divert_Speed2);               //期望值为0  输入为编码器采集值
//


}

void Divert_Angle_Loop(){
    Pid_Position(Delta_Yaw,T_N,&Divert_Angle);
}

void Divert_Loop(){
    Pid_Position(Divert_Angle.Out,imu660ra_gyro_transition(imu660ra_gyro_z),&Turn);

}
/*------------------------------------------------
 * 函数简介     距离pid
 * 函数名称     distance_pid(float aim_position,float now_position)
 * 返回参数     float
 * 使用示例     NaN
 * ----------------------------------------------*/
//void distance_loop(){
//    Pid_Position(aim_Encoder,Encoder_Total,&distance);
//}

/*------------------------------------------------
 * 函数简介     控速pid
 * 函数名称     distance_pid(float aim_position,float now_position)
 * 返回参数     float
 * 使用示例     NaN
 * ----------------------------------------------*/
void Speed_ctrl_loop(){
    Pid_Position(point_data[i_Track+1][1],T_N,&Speed_ctrl);
}

/*------------------------------------------------
 * 函数简介     电机控制
 * 函数名称     Motor_PID_Control()
 * 返回参数     void
 * 使用示例     NaN
 * ----------------------------------------------*/

void Motor_PID_Control(){
    if(Start_Flag==0)
    {
        DIN_OFF;                                        //刹车
        Motor_Control(0,0,0);                             //飞轮 独轮都停止
    }
    else if(Start_Flag==1)
    {
        DIN_ON;                                        //刹车
        Motor_Control(0,0,-Duty_X);                       //飞轮刹车 独轮启动
    }
    else if(Start_Flag==2)
    {
        DIN_ON;                                         //启动
        Motor_Control(Duty_L,-Duty_R,-Duty_X);         //飞轮 独轮都启动
    }
}

char flag_pose = 0;//0表示稳定，1表示不稳定，车体姿态标志位
/*------------------------------------------------
 * 函数简介     车体姿态判断函数
 * 函数名称     Attitude_stability_judgment(float roll, float pitch, float yaw)
 * 返回参数     char
 * 使用示例     NaN
 * ----------------------------------------------*/
char Attitude_stability_judgment(float roll, float pitch, float yaw) {

    if(Max_Delta_Pitch > 10 || Max_Delta_Roll > 10){
        flag_pose = 2;//2：极度不稳定
    }
    else if(Max_Delta_Pitch > 3 || Max_Delta_Roll > 3){
        flag_pose = 1;//1：不稳定
    }
    else{
        flag_pose = 0;//0：稳定
    }

    if(flag_pose == 2){
        if(abs(first_complement.angle.roll) - abs(Angle_Y_zero) <= 4){
            Pitch_Min = first_complement.angle.pitch;
            Pitch_Max = first_complement.angle.pitch;
            Roll_Min = first_complement.angle.roll;
            Roll_Max = first_complement.angle.roll;
        }
    }

    if(flag_pose == 1){
        if(abs(first_complement.angle.roll) - abs(Angle_Y_zero) <= 2){
             Pitch_Min = first_complement.angle.pitch;
             Pitch_Max = first_complement.angle.pitch;
             Roll_Min = first_complement.angle.roll;
             Roll_Max = first_complement.angle.roll;
         }
    }
//    if(yaw){
//
//    }
    return flag_pose;
}

/*------------------------------------------------
 * 函数简介     Roll轴动态零点函数（压弯）
 * 函数名称     Dynamic_Zeroing_roll(float vec,float angle)
 * 返回参数     float
 * 使用示例     NaN
 * ----------------------------------------------*/
float Dynamic_Zeroing_roll(float vec,float angle){

    float zero = 0;
    signed int static last_error_1;      //上次偏差
    signed int static last_error_2;      //上上次偏差

    zero= K2 * angle + vec * vec * angle * K1;   //获取最终数

    last_error_2=last_error_1;            //记录上上次误差
    last_error_1=angle;             //记录上次


    zero = firstOrderFilter(zero);

    if(zero<-4)  zero=-4;
    if(zero>4)   zero=4;

    return zero;

}
/*------------------------------------------------
 * 函数简介     突变离散角度值连续化函数
 * 函数名称     Turn_Angle()
 * 返回参数     void
 * 使用示例     Turn_Angle();
 * ----------------------------------------------*/
void Turn_Angle(){


    if(Delta_Yaw_Temp - Delta_Yaw > 0){
        Delta_Yaw += 0.1;
        if(Delta_Yaw > Delta_Yaw_Temp){// && Flag.Angle_Decrease == 0){
            Delta_Yaw = Delta_Yaw_Temp;

        }
    }
    if(Delta_Yaw_Temp - Delta_Yaw < 0){
        Delta_Yaw -= 0.1;
        if(Delta_Yaw < Delta_Yaw_Temp){
            Delta_Yaw = Delta_Yaw_Temp;
        }
    }

//    Delta_Yaw = Delta_Yaw_Temp;
}
/************一阶滤波************/
float firstOrderFilter(float in_data)
{
    float final=0;
    float a=0.25;
    final = a*in_data + (1-a)*final;
    return(final);
}

/***************
 * 静止PID参数调整
 ***************/
void PID_Freeze_Init()
{
    /*-----速度环参数-----*/
        V.Kp = 0.02;//-0.000425;
        V.Ki = 0.0002;//-0.000125;
        V.Kd = 0.00;
        V.Err = 0;
        V.Err_last = 0;
        V.Err_l2st = 0;
        V.Integral_lim = 4000; //积分限幅
        V.Err_Int = 0;  //误差积分
        V.P_out = 0;
        V.I_out = 0;
        V.D_out = 0;
    //    V.Out = 0;
        V.Out_lim = 60;

    /*-----速度角度环参数-----*/
        VA.Kp = -4 ;//-1.28;//0.58+
        VA.Ki = -2;
        VA.Kd = -0.4;//-52;//-52
        VA.Err = 0;
        VA.Err_last = 0;
        VA.Err_l2st = 0;
        VA.Integral_lim = 0; //积分限幅
        VA.Err_Int = 0;  //误差积分
        VA.P_out = 0;
        VA.I_out = 0;
        VA.D_out = 0;
    //    VA.Out = 0;
        VA.Out_lim = 3000;

    /*-----行进角速度环参数-----*/
        G_Y.Kp = -90;//55;-
        G_Y.Ki = -7;//8.5;
        G_Y.Kd = 0;
        G_Y.Err = 0;
        G_Y.Err_last = 0;
        G_Y.Err_l2st = 0;
        G_Y.Integral_lim = 400; //积分限幅
        G_Y.Err_Int = 0;  //误差积
        G_Y.P_out = 0;
        G_Y.I_out = 0;
        G_Y.D_out = 0;
    //    G_Y.Out = 0;
        G_Y.Out_lim = 4000;

    /*-----动量轮速度环参数-----*/
        Speed2.Kp =-1;
                Speed1.Kp = -1;//-0.235;
        Speed2.Ki =-3;
                Speed1.Ki = -3;
        Speed2.Kd =Speed1.Kd = 0;
        Speed2.Err =Speed1.Err = 0;
        Speed2.Err_last =0;
        Speed1.Err_last = 0;
        Speed2.Err_l2st =0;
                Speed1.Err_l2st = 0;
        Speed2.Integral_lim = 200; //积分限幅

        Speed2.Integral_lim =0;
                Speed1.Err_Int = 0;  //误差积分
        Speed2.P_out =0;
                Speed1.P_out = 0;
        Speed2.I_out =0;
                Speed1.I_out = 0;
        Speed2.D_out =0;
                Speed1.D_out = 0;
       //  Speed.Out = 0;
        Speed2.Out_lim =Speed1.Out_lim = 3000;


    /*-----角度环参数-----*/
        A.Kp = 1.3;//5.25;//-1.25;//-1.25;//4.25f;
        A.Ki = 1.7;//4.55;//1;//3.5;
        A.Kd =-1.3;
        A.Err = 0;
        A.Err_last = 0;
        A.Err_l2st = 0;
        A.Integral_lim = 0; //积分限幅
        A.Err_Int = 0;  //误差积分
        A.P_out = 0;
        A.I_out = 0;
        A.D_out = 0;
    //    A.Out = 0;
        A.Out_lim = 3000;

    /*-----角速度环参数-----*/
        G.Kp = 120.5f;//150
        G.Ki = 8;//12
        G.Kd = 0;
        G.Err = 0;
        G.Err_last = 0;
        G.Err_l2st = 0;
        G.Integral_lim = 4000; //积分限幅
        G.Err_Int = 0;  //误差积分
        G.P_out = 0;
        G.I_out = 0;
        G.D_out = 0;
    //    G.Out = 0;
        G.Out_lim = 9000;

/*-----转向环参数-----*/
    Divert_Speed.Kp = 5;
    Divert_Speed.Ki = -1;
    Divert_Speed.Kd = -1;
    Divert_Speed.Err = 0;
    Divert_Speed.Err_last = 0;
    Divert_Speed.Err_l2st = 0;
    Divert_Speed.Integral_lim = 0; //积分限幅
    Divert_Speed.Err_Int = 0;  //误差积分
    Divert_Speed.P_out = 0;
    Divert_Speed.I_out = 0;
    Divert_Speed.D_out = 0;
//  Divert_Speed.Out = 0;
    Divert_Speed.Out_lim = 0;

    Divert_Angle.Kp = -9.5;
    Divert_Angle.Ki = -0;
    Divert_Angle.Kd = 3.8;
    Divert_Angle.Err = 0;
    Divert_Angle.Err_last = 0;
    Divert_Angle.Err_l2st = 0;
    Divert_Angle.Integral_lim = 0; //积分限幅
    Divert_Angle.Err_Int = 0;  //误差积分
    Divert_Angle.P_out = 0;
    Divert_Angle.I_out = 0;
    Divert_Angle.D_out = 0;
//  Divert.Out = 0;
    Divert_Angle.Out_lim = 800;

    Turn.Kp = -50;
    Turn.Ki = 0;
    Turn.Kd = 9;
    Turn.Err = 0;
    Turn.Err_last = 0;
    Turn.Err_l2st = 0;
    Turn.Integral_lim = 3000; //积分限幅
    Turn.Err_Int = 0;  //误差积分
    Turn.P_out = 0;
    Turn.I_out = 0;
    Turn.D_out = 0;
//  Divert.Out = 0;
    Turn.Out_lim = 7000;

//    printf("V.Kp:%f\n",V.Kp);

    /*-----控速环参数-----*/
    Speed_ctrl.Kp = 12 ;//-1.28;//0.58+
    Speed_ctrl.Ki = 0.00;
    Speed_ctrl.Kd = 0;//-52;//-52
    Speed_ctrl.Err = 0;
    Speed_ctrl.Err_last = 0;
    Speed_ctrl.Err_l2st = 0;
    Speed_ctrl.Integral_lim = 0; //积分限幅
    Speed_ctrl.Err_Int = 0;  //误差积分
    Speed_ctrl.P_out = 0;
    Speed_ctrl.I_out = 0;
    Speed_ctrl.D_out = 0;
//    Speed_ctrl.Out = 0;
    Speed_ctrl.Out_lim = 450;

}
