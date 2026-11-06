/*---------------------------------
*   File name: UI.c
*
*   Author: LaZuli
*
*   Created time: 2024年4月12日
---------------------------------*/
#include "UI.h"
/*--------------------------------*/
//菜单定义
menu* father1;  //菜单一
menu* father2;
menu* father3;
menu* father4;
menu* Cursor;   //光标
menu* nowmenu;  //现菜单
menu* father5;

char display_on=1;

static uint16 i=20;//光标起点
bool menu_flag=0;   //是否调用结构体中的指针函数,1为是
static int L_Y,L_X,R_Y,R_X;

//函数声明
void Basis_Factor_Check_1(void);
void menu_display(void);
void parameter_debug(void);
void remote_track(void);
void Value_change(float *Value,float Unit);
void Cam(void);
void navigation_path();
void nofun(){}  //空函数
char navigation(){return 1;}



//添加菜单
menu* Menu_create(uint8 row,char date[20],void* Fanction)//行数,菜单名,指针函数
{
    menu* node=(menu*)malloc (sizeof(menu));    //分配节点内存

    node->row=row;                  //填入行数
    strcpy(node->date, date);       //填入菜单名
    node->Fanction=Fanction;        //填入执行函数

    node->sibling=NULL;

    return node;
}

//添加兄弟菜单
void addsibling(menu *older, menu *youth)
{
    if (older->sibling == NULL)
    {
        older->sibling = youth;
    }
    else    //其他情况还没考虑
    {

    }
}

void tft_init(void) //屏幕初始化
{
    tft180_init();
    tft180_clear();
    tft180_set_dir(TFT180_PORTAIT);     //屏幕方向
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_full(RGB565_BLACK);
}

void menu_init()
{
    tft_init();

    Cursor = Menu_create(20,">>",*nofun);
        father1 = Menu_create(20,"1.Status",*Basis_Factor_Check_1);
        father2 = Menu_create(40,"2.Parameter",*parameter_debug);
        father3 = Menu_create(60,"3.Camera",*Cam);
        father4 = Menu_create(80,"4.Navigation",*navigation_path);
        father5 = Menu_create(100,"5.remote_T",*remote_track);

    nowmenu=father1;

    addsibling(father1,father2);
    addsibling(father2,father3);
    addsibling(father3,father4);
    addsibling(father4,father5);
    addsibling(father5,father1);
//    addsibling(father4,father1);

    menu_display();
}

void menu_display() //主菜单显示
{
    tft180_full(RGB565_BLACK);
    tft180_set_font(TFT180_8X16_FONT);


    tft180_show_string(20,father1->row,father1->date);
    tft180_show_string(20,father2->row,father2->date);
    tft180_show_string(20,father3->row,father3->date);
    tft180_show_string(20,father4->row,father4->date);
    tft180_show_string(20,father5->row,father5->date);
    tft180_show_string(0,Cursor->row,Cursor->date);
}
void choose_menu()
{
    if(gpio_get_level(Key1) == 0)//光标移动
    {
            system_delay_ms(10);
            while(gpio_get_level(Key1) == 0);
            system_delay_ms(10);
            nowmenu=nowmenu->sibling;
            menu_flag=0;

        if(Cursor->row==100)    //光标回到第一个
        {
            Cursor->row =20;
            i=80;
            for(i;i>Cursor->row;i--)
            {
                tft180_show_string(0,i,Cursor->date);
                i+=(Cursor->row-i)/32;
                system_delay_ms(1);
            }
            i=20;
        }
        else
        {Cursor->row +=20;
            for(i;i<Cursor->row;i++)
            {
                 tft180_show_string(0,i,Cursor->date);
                 i+=(Cursor->row-i)/8;
                 system_delay_ms(10);
            }
        }
        menu_display();

    }
        if(gpio_get_level(Key2) == 0)   //确定,返回,调用结构体中的指针函数
            {
                system_delay_ms(10);
                while(gpio_get_level(Key2) == 0);
                system_delay_ms(10);

                menu_flag=!menu_flag;
                tft180_full(RGB565_BLACK);
                if(menu_flag==0)menu_display();
            }
        if(menu_flag==1)
            {
                nowmenu->Fanction();
            }
        else {}
}

/*-----------结构体中的指针函数(需提前声明)-----------*/

///*-------------------子菜单-------------------*/

/*------------------------------------------------
 * 函数简介     状态查看
 * 函数名称     Basis_Factor_Check_1(void )
 * 返回参数     void
 * 使用示例     NaN
 * ----------------------------------------------*/
void Basis_Factor_Check_1(void)
{
//    tft180_full(RGB565_BLACK);
if(display_on==1){
                    tft180_show_string(15,128,"Successful?");
                    tft180_show_string(15,0,"Rol:");
                    tft180_show_float(50,0,first_complement.angle.roll,2,4);
                    tft180_show_string(15,16,"Pit:");
                    tft180_show_float(50,16,first_complement.angle.pitch,2,4);
                    tft180_show_string(15,32,"yaw_error:");
                    tft180_show_float(50,32,Delta_Yaw-T_N,4,2);
                    tft180_show_string(15,48,"Spe1:");
                    tft180_show_float(60,48,Vec_BLZ,4,1);
                    tft180_show_string(15,64,"Spe2");
                    tft180_show_float(60,64,Vec_BLY,4,1);
                    tft180_show_string(15,80,"Sp_r:");
                    tft180_show_float(60,80,Vec_Precious,4,1);
                    tft180_show_string(15,96,"tof:");
                    tft180_show_float(55,96,dl1b_distance_mm,5,2);

                    tft180_show_string(15,112,"lora");
                    tft180_show_float(60,112,VA.Out,3,2);
                  }

remote_control();
}
/*------------------------------------------------
 * 函数简介     调参todo
 * 函数名称     parameter_debug(void )
 * 返回参数     void
 * 使用示例     NaN
 * ----------------------------------------------*/

void parameter_debug()
{
 static uint16 cursor = 0;
 if(display_on==1){
                    tft180_show_string(15,32,"GZ:");
                    tft180_show_float(50,32,T_N,3,4);
                    tft180_show_string(15,128,"zhuan");
                    tft180_show_float(60,128,Delta_Yaw,3,1);
                    tft180_show_string(15,144,"R");
                    tft180_show_float(60,144,Vec_Aim,4,1);
                  }
    //
    if(gpio_get_level(Key3) == 0)        //光标移动(向下)
    {
        system_delay_ms(12);
        while(gpio_get_level(Key3) == 0);
        system_delay_ms(12);
        tft180_full(RGB565_BLACK);
        cursor+=16;
        if(cursor >144) cursor = 0;
        tft180_show_string(0,cursor,">>");

    }
//    switch(cursor)      //调整参数对应的光标,执行调整函数
//    {
//
//        case 0:Value_change(&G.Kp,10);break;
//        case 16:Value_change(&G.Ki,0.2);break;
//        case 32:Value_change(&G.Kd,0.2);break;
//        case 48:Value_change(&A.Kp,0.1);break;
//
//        default:break;
//    }
    remote_control();

}
/*------------------------------------------------
 * 函数简介     Todo
 * 函数名称     remote_control(void )
 * 返回参数     void
 * 使用示例     NaN
 * ----------------------------------------------*/
void Cam()
{
if(display_on==1){
                tft180_full(RGB565_BLACK);
            //    tft180_draw_line(5,31,50,31,RGB565_PINK);
            //    tft180_draw_line(5,86,50,86,RGB565_PINK);

                L_Y=(lora3a22_uart_transfer.joystick[1])/100;
                L_X=(lora3a22_uart_transfer.joystick[0])/100;
                R_Y=(lora3a22_uart_transfer.joystick[3])/100;
                R_X=(lora3a22_uart_transfer.joystick[2])/100;

                tft180_show_string(25-L_X,50-L_Y,"+");
                tft180_show_string(85-R_X,50-R_Y,"+");
                    }
    remote_control();

    dot_matrix_screen_set_brightness(6000);

    dot_matrix_screen_show_string("OK");

}
/*------------------------------------------------
 * 函数简介     路径导航
 * 函数名称     remote_control(void )
 * 返回参数     void
 * 使用示例     NaN
 * ----------------------------------------------*/
void navigation_path(){
    static int i=0;

if(display_on==1){
                tft180_set_font(TFT180_6X8_FONT);

            //    tft180_show_float( 0, 0, point_data[1][0], 3, 4);
                tft180_show_float( 58, 0, point_data[1][1], 3, 4);

                tft180_show_float( 0, 24, point_data[2][0], 3, 4);
                tft180_show_float( 58, 24, point_data[2][1], 3, 4);

                tft180_show_string(15,144,"RR");
                tft180_show_float(60,120,Vec_Original,4,1);

                tft180_show_string(15,144,"R");
                tft180_show_float(60,144,Vec_Aim,4,1);

                tft180_show_float( 5, 48, T_N , 4, 3);
                tft180_show_float( 50, 48, Turn.Out , 4, 2);
                }

if(i<1){
//    Vec_Aim = Vec_Original;
    Delta_Yaw_Temp=0;
        Vec_Aim = 0 ;

    Delta_Yaw=0;
    Delta_Yaw_Temp=0;
    i++;
}

if(gpio_get_level(Key3) == 0)
    {
        system_delay_ms(10);
        while(gpio_get_level(Key3) == 0);
        system_delay_ms(10);
        run_flag=1;
        Encoder_Total=0;
        Vec_Aim = Vec_Original;
//        aim_Encoder=point_data[1][0];
//        system_delay_ms(500);
        i_Track=0 ;
        T_N=0;Delta_Yaw=0;
    }

Value_change(&Vec_Original,10);

    Speed_ctrl_loop();

    Track();
    if(dl1b_distance_mm<800&&Vec_Aim>=300){Vec_Aim=300;}
//    else Vec_Aim=Vec_Original;
    Turn_Angle();
    Passive_Car_Status();
    Spontaneous_Car_Status();

   }
/*------------------------------------------------
 * 函数简介     遥控打点
 * 函数名称     remote_track(void )
 * 返回参数     void
 * 使用示例     NaN
 * ----------------------------------------------*/

void remote_track ()    //
{

if(display_on==1){
                tft180_set_font(TFT180_6X8_FONT);

            //    tft180_show_float( 0, 0, point_data[1][0], 6, 1);
                tft180_show_float( 58, 0, point_data[1][1],3, 4);

                tft180_show_float( 0, 24, point_data[2][0], 6, 1);
                tft180_show_float( 58, 24, point_data[2][1], 3, 4);
                tft180_show_string(15,144,"R");
                tft180_show_float(60,144,Vec_Aim,4,1);

            //    tft180_show_float( 5, 48, T_M , 4, 3);
                tft180_show_float(5,48,T_N,4,6);

                tft180_show_float( 50, 60, Turn.Out , 4, 2);
                tft180_show_float( 50, 75, i_cb , 4, 2);
                }

    remote_control();

    remote_Start_Navigate();
    //
    if(gpio_get_level(Key3) == 0)
        {
            system_delay_ms(10);
            while(gpio_get_level(Key3) == 0);
            system_delay_ms(10);

            T_N=0;Delta_Yaw=0;
            system_delay_ms(10);
        }
    else Start_Collect=0;
    //
    if(lora3a22_uart_transfer.joystick[2]==0){
        Start_Collect=0;
        if(lora3a22_uart_transfer.joystick[3]>1700){
            Start_Collect=1;}

    }
    else Start_Collect=1;
    //
    if(gpio_get_level(Key5) == 0)
    {
        system_delay_ms(10);
        while(gpio_get_level(Key5) == 0);
        system_delay_ms(10);

        i_cb=1;
    }

}


///*-------------------功能函数-------------------*/

/*------------------------------------------------
 * 函数简介     遥控
 * 函数名称     remote_control(void )
 * 返回参数     void
 * 使用示例     NaN
 * ----------------------------------------------*/

void remote_control(void )
{
    //
    Vec_Aim=lora3a22_uart_transfer.joystick[1]/4;
    if(dl1b_distance_mm<500&&Vec_Aim>250)Vec_Aim=250;
    if(lora3a22_uart_transfer.joystick[2]!=0&&Vec_Aim>250){
        do{Vec_Aim-=0.08;}while(Vec_Aim>250);
    }



//        if(Vec_Aim!=0)    distance.Out_lim = 0; //积分限幅
//        else distance.Out_lim=450;
//        if(Vec_Aim==0)Vec_Aim=lora3a22_uart_transfer.joystick[3]/6;
        Delta_Yaw-=0.001*lora3a22_uart_transfer.joystick[2]/12;
       if( lora3a22_uart_transfer.joystick[0]>1700){T_N=0;Delta_Yaw=0;}
       if( lora3a22_uart_transfer.joystick[0]<-1700){tft_init();}

//       Encoder_Total=0;
//       aim_Encoder=Encoder_Total;
//       aim_Encoder=lora3a22_uart_transfer.joystick[1]*10;

}
/*------------------------------------------------
 * 函数简介     按键调参
 * 函数名称     Value_change(float *Value,float Unit )
 * 返回参数     void
 * 使用示例     Value----需要改变的值
 *            Unit ----单位数值
 * ----------------------------------------------*/
void Value_change(float *Value,float Unit)  //调整值   单位值
{
    if(gpio_get_level(Key4) == 0)    //值加一个单位
    {
        system_delay_ms(12);
        while(gpio_get_level(Key4) == 0);
        system_delay_ms(12);
        *Value=*Value+Unit;
    }
    if(gpio_get_level(Key5) == 0)    //值减一个单位
    {
        system_delay_ms(12);
        while(gpio_get_level(Key5)== 0);
        system_delay_ms(12);
        *Value=*Value-Unit;
    }

}



/*--------------------------------*/


