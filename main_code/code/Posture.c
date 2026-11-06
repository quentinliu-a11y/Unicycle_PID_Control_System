/*---------------------------------
*   File name: Posture.c
*
*	Author: LaZuli
*
*	Created time: 2024年4月10日
---------------------------------*/
#include "Posture.h"

gyro_param_t Gyro_Offset;//陀螺仪零飘结构体
IMU_param_t  IMU_Data;   //减去零飘数据结构体
extKalman_t Kalman1;
extKalman_t Kalman2;

float gyro_Offset_flag = 0;
float Daty_Z = 0;
float Daty_X = 0;
float Daty_Y = 0;
float T_M = 0;
float T_N = 0;

/*
 * @brief 四元数姿态解算
 * NULL
 * */

/*
 * @brief 一阶互补滤波姿态解算
 * */
float Pitch_Max = 0;
float Pitch_Min = 0;
float Roll_Max = 0;
float Roll_Min = 0;
float Max_Delta_Pitch = 0;
float Max_Delta_Roll = 0;

First_Complement first_complement;

void First_complement(){

    float k = 0.65;
    static float Pitch_Temp = 0;
    static float Roll_Temp = 0;
    static float Delta_Roll = 0;
    static char Pitch_Flag = 0;
    static char Roll_Flag = 0;
//    static float Integral_Pitch = 0;
//    static float Integral_Roll = 0;
/*----------------------------------------角度获取----------------------------------------*/

    Pitch_Temp = (atan(IMU_Data.acc_y/IMU_Data.acc_z))*180/3.1415926535f;
    Roll_Temp = (atan(IMU_Data.acc_x/IMU_Data.acc_z))*180/3.1415926535f;

    Pitch_Temp = Pitch_Temp + IMU_Data.gyro_x * 0.005f;
    Roll_Temp = Roll_Temp + IMU_Data.gyro_y * 0.005f;

//    Integral_Pitch += RAD_TO_ANGLE(IMU_Data.gyro_x) * 0.005f;
//    Integral_Roll += RAD_TO_ANGLE(IMU_Data.gyro_y) * 0.005f;

    first_complement.angle.pitch = k * Pitch_Temp + (1 - k) * (first_complement.angle.pitch + IMU_Data.gyro_x * 0.005f);
    first_complement.angle.roll = k * Roll_Temp + (1 - k) * (first_complement.angle.roll + IMU_Data.gyro_y * 0.005f);

//    first_complement.angle.pitch = k * Pitch_Temp + (1 - k) * Integral_Pitch;
//    first_complement.angle.roll = k * Roll_Temp + (1 - k) * Integral_Roll;

    if(Delta_Roll < 0.1 && Delta_Roll > -0.1){
        first_complement.angle.roll = first_complement.angle.roll - Delta_Roll;
    }
//    else if(Delta_Roll > 1 || i > 10){
//        first_complement.angle.roll = 0.5 * first_complement.angle.roll;
//    }
    first_complement.angle.roll = KalmanFilter(&Kalman1,first_complement.angle.roll);//卡尔曼滤波
    first_complement.angle.pitch = KalmanFilter(&Kalman2,first_complement.angle.pitch);//卡尔曼滤波
//    first_complement.angle.pitch = firstOrderFilter(first_complement.angle.pitch);
//    first_complement.angle.roll = firstOrderFilter(first_complement.angle.roll);

/*----------------------------------------角度获取----------------------------------------*/

    if(Pitch_Flag == 0 && Start_Flag == 2){
        Pitch_Min = first_complement.angle.pitch;
        Pitch_Max = first_complement.angle.pitch;

        Pitch_Flag = 1;
    }

    if(Roll_Flag == 0 && Start_Flag == 2){
        Roll_Min = first_complement.angle.roll;
        Roll_Max = first_complement.angle.roll;

        Roll_Flag = 1;
    }

    if(My_abs(Pitch_Min) > My_abs(first_complement.angle.pitch)){
        Pitch_Min = My_abs(first_complement.angle.pitch);
    }
    else if(My_abs(Pitch_Max) < My_abs(first_complement.angle.pitch)){
        Pitch_Max = My_abs(first_complement.angle.pitch);
    }

    if(My_abs(Roll_Min) > My_abs(first_complement.angle.roll)){
        Roll_Min = My_abs(first_complement.angle.roll);
    }
    else if(My_abs(Roll_Max) < My_abs(first_complement.angle.roll)){
        Roll_Max = My_abs(first_complement.angle.roll);
    }
    Max_Delta_Pitch = Pitch_Max - Pitch_Min;
    Max_Delta_Roll = Roll_Max - Roll_Min;
}


/**
 * @brief 陀螺仪零漂初始化
 * 通过采集一定数据求均值计算陀螺仪零点偏移值。
 * 后续 陀螺仪读取的数据 - 零飘值，即可去除零点偏移量。
 */
float IMU_gyro_Offset_Init()
{
    Gyro_Offset.Xdata = 0;
    Gyro_Offset.Ydata = 0;
    Gyro_Offset.Zdata = 0;
    for (uint16_t i = 0; i < 1000; i++)
    {
//        Gyro_Offset.Xdata += imu660ra_gyro_x;
//        Gyro_Offset.Ydata += imu660ra_gyro_y;
//        Gyro_Offset.Zdata += imu660ra_gyro_z;

        Gyro_Offset.Xdata += imu660ra_gyro_x;
        Gyro_Offset.Ydata += imu660ra_gyro_y;
        Gyro_Offset.Zdata += imu660ra_gyro_z;
        system_delay_ms(5);   // 最大 1Khz
    }

    Gyro_Offset.Xdata /= 1000;
    Gyro_Offset.Ydata /= 1000;
    Gyro_Offset.Zdata /= 1000;

    return gyro_Offset_flag=1;
}

void IMU_GetValues()//将采集的数值转化为实际物理值, 并对陀螺仪进行去零漂处理
{
//2000dps:IMU660--16.4
//2000dps:IMU660--14.3
        imu660ra_get_gyro();
        imu660ra_get_acc();
    //! 陀螺仪角速度必须转换为弧度制角速度: deg/s -> rad/s
//    IMU_Data.gyro_x = ((float) imu660ra_gyro_x - Gyro_Offset.Xdata) * PI / 180 / 16.4f;
//    IMU_Data.gyro_y = ((float) imu660ra_gyro_y - Gyro_Offset.Ydata) * PI / 180 / 16.4f;
//    IMU_Data.gyro_z = ((float) imu660ra_gyro_z - Gyro_Offset.Zdata) * PI / 180 / 16.4f;

        IMU_Data.gyro_x = ((float) imu660ra_gyro_x - Gyro_Offset.Xdata) * PI / 180 / 16.4f;
        IMU_Data.gyro_y = ((float) imu660ra_gyro_y - Gyro_Offset.Ydata) * PI / 180 / 16.4f;
        IMU_Data.gyro_z = ((float) imu660ra_gyro_z - Gyro_Offset.Zdata) * PI / 180 / 16.4f;

        IMU_Data.acc_x = (((float)imu660ra_acc_x) * 0.3f) + IMU_Data.acc_x * 0.7f;
        IMU_Data.acc_y = (((float)imu660ra_acc_y) * 0.3f) + IMU_Data.acc_y * 0.7f;
        IMU_Data.acc_z = (((float)imu660ra_acc_z) * 0.3f) + IMU_Data.acc_z * 0.7f;


}

void IMU_YAW_integral()//对角速度进行积分
{
//        IMU_GetValues();

//        Daty_X-=RAD_TO_ANGLE(IMU_Data.gyro_x*0.005);
//        Daty_Y-=RAD_TO_ANGLE(IMU_Data.gyro_y*0.005);
    //    if(IMU_Data.gyro_z<0.0045&&IMU_Data.gyro_z>-0.0045)
        if(IMU_Data.gyro_z<0.015&&IMU_Data.gyro_z>-0.015)//滤波
        {
            Daty_Z-=0;

        }
        else
        {
            IMU_Handle_180();//规划为0-180和0-(-180)   Daty_Z
            IMU_Handle_360();//规划为0-360和0-(-360)   T_M
            IMU_Handle_0();  //规划为0-正无穷和0-负无穷  T_N
         }

        if(IMU_Data.gyro_x<0.015&&IMU_Data.gyro_x>-0.015)//滤波
        {
            Daty_X-=0;

        }
        if(IMU_Data.gyro_y<0.015&&IMU_Data.gyro_y>-0.015)//滤波
        {
            Daty_Y-=0;

        }



}

void IMU_Handle_180()
{
    Daty_Z-=RAD_TO_ANGLE(IMU_Data.gyro_z*0.005);//(积分过程)本来是逆时针为正,现在改为顺时针为正

    if((Daty_Z>0&&Daty_Z<=180)  ||   (Daty_Z<0&&Daty_Z>=(-180)))//顺时针
    {
        Daty_Z= +Daty_Z;
    }
    else if(Daty_Z>180 && Daty_Z<=360)
    {
        Daty_Z-=360;
    }
    else if(Daty_Z<(-180) && Daty_Z>=(-360))
    {
        Daty_Z+=360;
    }

}


char Round = 0;

void IMU_Handle_360()
{
    if(Round == 0){
        T_M -= RAD_TO_ANGLE(IMU_Data.gyro_z*0.005);
        if(T_M > 360){
            T_M = 360;
            Round = 1;
        }
        else if(T_M < -360){
            T_M = -360;
            Round = 1;
        }
    }
    if(Round == 1){
        if(T_M <= 360 && T_M >= -360){
            T_M += RAD_TO_ANGLE(IMU_Data.gyro_z*0.005);
            if(T_M > 360){
                T_M = 360;
                Round = 0;
            }
        }
        else if(T_M >= -360){
            T_M -= RAD_TO_ANGLE(IMU_Data.gyro_z*0.005);
            if(T_M < -360){

                T_M = -360;
                Round = 0;
            }
        }
//        if(T_M < 0){
//            T_M = 0;
//            Round = 0;
//        }
    }
}
void IMU_Handle_0()
{
    T_N-=RAD_TO_ANGLE(IMU_Data.gyro_z*0.005);//(积分过程)本来是逆时针为正,现在改为顺时针为正

}


float fast_sqrt(float num) {
    float halfx = 0.5f * num;
    float y = num;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i >> 1);

    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
    // float y = sqrtf(num);
    // return y;
}

float My_abs(float x) {
    // 返回x的绝对值，用三元运算符
    return x < 0 ? -x : x;
}
/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *
  * @retval none
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *             反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  */

void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶H'即为它本身,否则为转置矩阵
  */

float KalmanFilter(extKalman_t* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A*p->P_last+p->Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;                              //输出预测结果x(k|k)
}
