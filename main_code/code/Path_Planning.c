/*---------------------------------
*   File name: Path_Planning.c
*	
*	Author: LaZuli
*
*	Created time: 2024年4月23日
---------------------------------*/
#include "Path_Planning.h"
char run_flag=0;
char Start_Collect=0;



void remote_Start_Navigate(){
    if(Flag.Start_Navigate == 0 ){

        system_delay_ms(10);
        i_Track = 0;//循迹点初始化
        point_data[0][0] = 0;
        point_data[0][1] = 0;
        Delta_Yaw_Temp = point_data[0][1];
//        Start_Collect=1;
        Flag.Start_Navigate = 1;
//        Flag.Start_Pin_Point = 1;
//        Start_Collect=1;
        system_delay_ms(10);
    }
}
//pid走距离环?
void Auto_Collect(){

    if(  i_cb<=Point_Num&&Start_Collect == 1){
        Buzzer(1);
        system_delay_ms(1);
        Buzzer(0);

            point_data[i_cb][j] = Encoder_Total;//point()->gps.Latitude;
            point_data[i_cb][j+1] = T_N;//point()->gps.Longitude;
            Encoder_Total = 0;
            i_cb++;
    }

}




//利用贪婪算法或者遗传算法得到给定点的最优解
