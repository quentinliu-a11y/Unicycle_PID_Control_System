/*---------------------------------
*   File name: Combined_Navigation.c
*
*   Author: LaZuli
*
*   Created time: 2024年4月12日
---------------------------------*/
#include "Combined_Navigation.h"

/*
 * 1、启动时默认打点模式，不进行任何姿态以及点位逼近判断；
 * 2、start navigate返回值为1时，循迹点初始化；
 * 3、打点模式之下，使用i来判断点位切换，保存到达这个点位时的yaw和encoder值（点位的yaw值存在问题，也许是正负号反了）；
 * 4、当start flag置1时，打点模式关闭，进入循迹模式，使用i_Track进行行进中点位切换；
 * */

flag Flag;
float Point_Size = 0;



void Flag_Init(){
    Flag.Start = 0;
    Flag.Start_Navigate = 0;
    Flag.Start_Pin_Point = 1;
    Flag.Distance_Approaching = 0;
    Flag.Angle_Approaching = 0;
}

/*------------------------------------------------
 * 函数简介     坐标点数据获取
 * 返回参数     Point_Data_Strcut point
 * 使用示例     gps_point[1][1] = *point().gps.Latitude;    //获取该点的经度值
 * 备注   内部调用
 * ----------------------------------------------*/
Point_Data_Strcut *point(){

    Point_Data_Strcut *point = (Point_Data_Strcut *)malloc(sizeof(Point_Data_Strcut));
    // 处理内存分配失败的情况
    if (point == NULL) {
        printf("Navigate initialize error!!!");
        Buzzer(1);
        return NULL;
    }

    // 初始化结构体成员
//    point->gps.Latitude = 0.0;
//    point->gps.Longitude = 0.0;
//    point->lnu.Delta_Yaw = 0.0;
//    point->lnu.Yaw = 0.0;
//    point->lnu.Last_Yaw = 0.0;
//    point->gps.First_Latitude = gnss.latitude;
//    point->gps.First_Longitude = gnss.longitude;

//    point->gps.Latitude = gnss.latitude; // @suppress("Field cannot be resolved")
//    point->gps.Longitude = gnss.longitude; // @suppress("Field cannot be resolved")

    point->lnu.Yaw = T_N;
    point->lnu.Delta_Yaw = point->lnu.Last_Yaw - point->lnu.Yaw;


    point->lnu.Last_Yaw = point->lnu.Yaw;
    Point_Size += sizeof(point);
    return (point);
}

/*------------------------------------------------
 * 函数简介     导航开始
 * 函数名称     Start_Navigate()
 * 返回参数     void
 * 使用示例     Start_Navigate()
 * ---------------------------------------+-------*/
void Start_Navigate(){
    if(Flag.Start_Navigate == 0){

        system_delay_ms(10);
        Buzzer(1);
        while(gpio_get_level(Key4) == 0);
        Buzzer(0);
        i_Track = 0;//循迹点初始化
        point_data[0][0] = 0;//point()->gps.First_Latitude;
        point_data[0][1] = point()->lnu.Yaw;//point()->gps.First_Longitude;
//        point_data[0][2] = point()->lnu.Yaw;
//        point_data[0][3] = 0;
//        point_data[0][4] = 0;
//        point_data[0][5] = 0;
//        point_data[0][6] = 0;
//        Delta_Yaw_Temp = point_data[0][1];

        Flag.Start_Navigate = 1;
        Flag.Start_Pin_Point = 1;
    }
}

/*
 * 1.判断按键是否触发获取当前信息，保存到flash中
 * 2.全部的点保存到一个二维数组中，确定的内存大小，以免内存泄露溢出
 * 3.当在外部调用这个函数时，
 * */
float point_data[Point_Num][Data];
float Delta_Yaw = 0;
float Delta_Yaw_Temp = 0;
int i_cb = 1;
char j = 0;
/*------------------------------------------------
 * 函数简介     坐标点数据存储
 * 返回参数     void
 * 使用示例     Data_Collect();
 * ----------------------------------------------*/
void Data_Collect(){

//    Start_Navigate();
    if(gpio_get_level(Key4) == 0 && Flag.Start_Navigate == 1){
        system_delay_ms(10);
        Buzzer(1);
        while(gpio_get_level(Key4) == 0);
        Buzzer(0);
            point_data[i_cb][j] = Encoder_Total;//point()->gps.Latitude;
            point_data[i_cb][j+1] = point()->lnu.Yaw;//point()->gps.Longitude;
//            point_data[i_cb][j+2] = point()->lnu.Yaw;//获取到达这个点时的Yaw轴角度值
//
//            point_data[i_cb][j+3] = point()->lnu.Yaw - point_data[i_cb-1][j+2];
//            //Delta_Yaw = point_data[i_cb][j+3];
//            point_data[i_cb][j+4] = get_two_points_distance(point_data[i_cb-1][j],point_data[i_cb-1][j+1],point_data[i_cb][j],point_data[i_cb][j+1]);
//            point_data[i_cb][j+5] = get_two_points_azimuth(point_data[i_cb-1][j],point_data[i_cb-1][j+1],point_data[i_cb][j],point_data[i_cb][j+1]);
//            point_data[i_cb][j+6] = Encoder_Total;
            Encoder_Total = 0;
            i_cb++;
//        }
    }
    if(i_cb == 100){
        printf("Full data!");
    }

}

Car_Global_Mode global_mode;
Car_Operate_Mode operate_mode;
/*------------------------------------------------
 * 函数简介     主动状态机切换
 * 函数名称     Spontaneous_Car_Status()
 * 返回参数     void
 * 使用示例     Spontaneous_Car_Status();
 * 对于车体姿态判断函数的回传数据做处理，从而调整现在车的行进状态
 * ----------------------------------------------*/
void Spontaneous_Car_Status(){

    switch(Attitude_stability_judgment(first_complement.angle.roll,first_complement.angle.pitch,T_N)){

//        case 0:operate_mode.now_mode = Accelerate;Buzzer(0);break;
//        case 1:operate_mode.now_mode = Decelerate;Buzzer(1);break;
//        case 2:operate_mode.now_mode = Freeze;Buzzer(1);break;
//        case 3:operate_mode.now_mode = Divert;break;
        case 0:operate_mode.now_mode = Accelerate;break;
        case 1:operate_mode.now_mode = Decelerate;break;
        case 2:operate_mode.now_mode = Freeze;break;
        case 3:operate_mode.now_mode = Divert;break;

        default:operate_mode.now_mode = Freeze;break;
    }
}

/*------------------------------------------------
 * 函数简介     从动状态机切换
 * 函数名称     Passive_Car_Status()
 * 返回参数     void
 * 使用示例     Passive_Car_Status();
 * 对于主动状态机的回传数据做处理，从而调整现在车的行进状态
 * ----------------------------------------------*/
void Passive_Car_Status(){
    if(global_mode.now_mode == Freeze){

//        Vec_Aim -= 5;
//
//        if(Vec_Aim <= 0){
//            Vec_Aim = 0;
//        }

        global_mode.last_mode = Freeze;
    }
    else if(global_mode.now_mode == Navigation){
        global_mode.last_mode = Navigation;
//        if(operate_mode.now_mode == Straight){
//
//            operate_mode.last_mode = Straight;
//
            //如果姿态为稳定，进入加速模式

            if(operate_mode.now_mode == Accelerate){
//                Buzzer(1);
                Vec_Aim += 0.2;
                if(Vec_Aim > 1000){
                    Vec_Aim = 1000;
                }
            }

            //如果姿态为不稳定，进入减速模式
            else if(operate_mode.now_mode == Decelerate){
//                Buzzer(0);
//                Vec_Aim -= 0.5;
//                if(Vec_Aim <= 350){
//                    Vec_Aim = 350;
//                }

            }

//        }
        else if(operate_mode.now_mode == Divert){

            operate_mode.last_mode = Divert;

        }


    }
    else if(global_mode.now_mode == Garage){
        //如果方位角归正，进入加速模式

        if(Delta_Yaw - T_N < 5){
            Duty_L += 0.2;
        }

        //如果方位角误差过大，进入减速模式
        else if(Delta_Yaw - T_N > 7 || T_N - Delta_Yaw < -7){
            Duty_L -= 0.2;
        }
    }
}

//储存点的数据后，将对于下一个点来说的delta_yaw与Distance传给速度环与转向环
//
/*------------------------------------------------
 * 函数简介     导航模式切换
 * 函数名称     Mode()
 * 返回参数     Navigate_Mode Mode
 * 使用示例     Car_Status();
 * ----------------------------------------------*/
Navigate_Mode Mode(){
    Navigate_Mode Mode;

    //进入导航菜单
    if(navigation()){
        char mode_temp = 0;

        //切换导航模式
        if(gpio_get_level(Key3) == 0){
            system_delay_ms(10);
            Buzzer(1);
            while(gpio_get_level(Key4) == 0);
            Buzzer(0);

            mode_temp ++;
            if(mode_temp > 2){
                mode_temp = 0;
            }
        }

        switch(mode_temp){
            case 0:Mode.mode = Pure_Navigate;

            break;

            case 1:Mode.mode = GPS_Used_Flag;

            break;

            case 2:Mode.mode = AHRs_Used_Flag;

            break;
        }

    }

    return Mode;
}

unsigned char i_Track = 0;
char judgement_temp = 0;

/*------------------------------------------------
 * 函数简介     点位逼近判断函数
 * 函数名称     Point_Approaching_Judgement()
 * 返回参数     char
 * 使用示例     if(Point_Approaching_Judgement() == 1)i_Track++;
 * 内部调用
 * ----------------------------------------------*/
char Point_Approaching_Judgement(){
    //需要加一个标志位，用以限制刚开始打点的姿态判断（但其实只加打点模式判断是不是也可以……？）
    if(Flag.Start_Pin_Point == 0){
        if(point_data[i_Track][0] - Encoder_Total < 100){
            Flag.Distance_Approaching = 1;

//            Vec_Aim -= 0.05;
//
//            if(Vec_Aim <= 0){
//                Vec_Aim = 0;
//            }

            //之前以下判断代码因为使用abs数进行判断，故在3和-3类似两个点处会误判,所以要防止出现两个角度值符号相反但是经过相同处理判断的情况
           /*--------------------*/
            if((point_data[i_Track][1] > 0 && Delta_Yaw < 0) || (point_data[i_Track][1] < 0 && Delta_Yaw > 0)){
                if(point_data[i_Track][1] + Delta_Yaw < 3){
                    Flag.Angle_Approaching = 1;
                }
            }
            else{
                if(My_abs(point_data[i_Track][1]) - My_abs(Delta_Yaw) < 3){
                    Flag.Angle_Approaching = 1;
                }
            }
           /*--------------------*/

        }

        if(Flag.Angle_Approaching == 1 && Flag.Distance_Approaching == 1){
            Flag.Angle_Approaching = 0;
            Flag.Distance_Approaching = 0;

//            Buzzer(1);
            judgement_temp++;
            return 3;

        }

        else{
            judgement_temp = 0;
//            Buzzer(0);
            return 0;
        }
    }
    else{
        return 0;
    }
}

//通过先前按键获取到的yaw值与distance值，依次传入该函数
/*------------------------------------------------
 * 函数简介     循迹主函数
 * 函数名称     Track()
 * 返回参数     void
 * 使用示例     Track();
 * ----------------------------------------------*/
void Track(){
    //点位逼近判断，需要增加一个*辅助判断条件*，或者逻辑补偿，否则将会自加到定义的最大数值
    //可以使用TOF当作辅助判断条件

    /*？？？？？怎么写啊？？？？？？*/

    if(Point_Approaching_Judgement() == 3 && judgement_temp == 1){
            i_Track ++;
            Encoder_Total = 0;

//            if(point_data[i_Track+1][0]>5000)
//                {
//                    Vec_Aim=Vec_Original+point_data[i_Track-1][0]*0.01;
//                            if(Vec_Aim>1300)Vec_Aim=1000;
//                }


             Vec_Aim = Vec_Original;


            aim_Encoder=point_data[i_Track][0];
            Delta_Yaw_Temp = point_data[i_Track][1];//使当前角度依次变为点位的角度值
    }
}

