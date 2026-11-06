/*---------------------------------
*   File name: Combined_Navigation.h
*
*   Author: LaZuli
*
*   Created time: 2024年4月12日
---------------------------------*/
#ifndef CODE_COMBINED_NAVIGATION_H_
#define CODE_COMBINED_NAVIGATION_H_

#include "zf_common_headfile.h"
/*-----------结构体初始化-----------*/

/*Navigate*/
typedef struct GPS_Data_Strcut{
        double Longitude;
        double Latitude;
        double First_Longitude;
        double First_Latitude;
}GPS_Data_Strcut;

typedef struct LNU_Data{
    float Delta_Yaw;
    float Last_Yaw;
    float Yaw;
    float Distance;
}LNU_Data;

typedef struct Point_Data_Strcut{
//        GPS_Data_Strcut gps;
        LNU_Data lnu;
}Point_Data_Strcut;

typedef enum{
    Pure_Navigate,
    GPS_Used_Flag,
    AHRs_Used_Flag,
}Mode_Enum;

typedef struct Navigate_Mode{
    Mode_Enum mode;
}Navigate_Mode;

typedef struct flag{
    unsigned char Start_Navigate;
    unsigned char Start_Pin_Point;
    unsigned char Start;
    unsigned char Distance_Approaching;
    unsigned char Angle_Approaching;

}flag;

/*Status*/

//Global_Status
enum{
    Bump,
    Navigation,
    Garage,
};

//Operate_Status
enum{
    Straight,
    Divert,
    Freeze,
    Accelerate,
    Decelerate,
};

typedef struct {
    unsigned char last_mode;//车当前状态
    unsigned char now_mode;//车一状态
}Car_Global_Mode;

typedef struct{
    unsigned char last_mode;//车当前状态
    unsigned char now_mode;//车一状态
}Car_Operate_Mode;

/*--------------------------------*/

/*-----------函数声明-----------*/
Point_Data_Strcut *point(void);
void Data_Collect(void);
Navigate_Mode Mode(void);
char Point_Approaching_Judgement(void);
void Track(void);
void Flag_Init(void);
void Spontaneous_Car_Status(void);
void Passive_Car_Status(void);
/*--------------------------------*/

#define Point_Num 10000
#define Data 2

/*变量外部声明*/
extern float point_data[Point_Num][Data];

extern float Delta_Yaw;
extern float Delta_Yaw_Temp;
extern float Point_Size;
extern Point_Data_Strcut *point();
extern unsigned char i_Track;
extern int i_cb;

extern Car_Global_Mode global_mode;
extern Car_Operate_Mode operate_mode;;
extern flag Flag;

#endif /* CODE_COMBINED_NAVIGATION_H_ */
