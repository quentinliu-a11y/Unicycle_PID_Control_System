/*---------------------------------
*   File name: Interact.c
*	
*	Author: LaZuli
*
*	Created time: 2024年4月11日
---------------------------------*/
#include "Interact.h"

char Start_Flag = 0;

void Go(void)
{
    if(gpio_get_level(Key0) == 0)
    {
        system_delay_ms(10);
        Buzzer(1);
        while(gpio_get_level(Key0) == 0);
        system_delay_ms(10);
        Buzzer(0);

        if(i_Track == 0){
            i_Track ++;
        }
        Start_Flag += 1;
        Flag.Start_Pin_Point = 0;
        global_mode.now_mode = Navigation;

        if(Start_Flag >= 3){
            Start_Flag = 0;
        }
    }

}


void Menu_Init(void)
{
    tft180_init();
    tft180_clear();
    tft180_set_dir(TFT180_PORTAIT);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_set_font(TFT180_6X8_FONT);
    tft180_full(RGB565_BLACK);

//    tft180_show_string(15,0,"1.Factor");
//    tft180_show_string(15,32,"2.Parameter");
//    tft180_show_string(15,64,"3.Camera");
//    tft180_show_string(15,96,"4.Navigate");
}

void Menu_Display(){
    Data_Collect();

    tft180_show_float( 0, 0, point_data[0][0], 3, 4);
    tft180_show_float( 58, 0, point_data[0][1], 3, 4);
    tft180_show_float( 0, 8, point_data[0][4], 4, 2);
    tft180_show_float( 58, 8, point_data[0][3], 4, 2);
    tft180_show_float( 0, 16, point_data[0][5], 4, 2);
//    tft180_show_float( 50, 0, point_data[0][4] , 4, 2);

    tft180_show_float( 0, 24, point_data[1][0], 3, 4);
    tft180_show_float( 58, 24, point_data[1][1], 3, 4);
    tft180_show_float( 0, 32, point_data[1][4], 4, 2);
    tft180_show_float( 58, 32, point_data[1][3], 4, 2);
    tft180_show_float( 0, 40, point_data[1][5], 4, 2);

//    tft180_show_float( 50, 17, point_data[0][4] , 4, 2);
}

/*------------------------------------------------
 * 函数简介     脱机调参
 * 函数输入     enum Modify_Mode mode
 * 返回参数     PID_Changed *pid_changed
 * 使用示例     if(GetKeyMode(Key0) == Click)
 * ----------------------------------------------*/
PID_Changed *Parameter_Change(enum Modify_Mode mode){
    //1.参数初始化 将PID结构体+上次按键的改变量打包发送给PID_Changed
    PID_Changed *pid_changed = (PID_Changed *)malloc(sizeof(PID_Changed));
    if(pid_changed == NULL){
        Buzzer(1);
        printf("PID_Changed initialize error!!!");
    }
    if(mode == Button){
        //1.将加减量写入flash
        FlashAutoSet(0);
        FlashWrite(Key1,PID_PARAMETER_PAGE1,1);
        //2.show
    }

    if(mode == Wireless){

    }
    return(pid_changed);
}

/*------------------------------------------------
 * 函数简介     获取按键模式
 * 函数输入     gpio_pin_enum pin
 * 返回参数     enum Key_Mode其中的一个值
 * 使用示例     if(GetKeyMode(Key0) == Click)
 * ----------------------------------------------*/
enum Key_Mode GetKeyMode(gpio_pin_enum pin){
    unsigned char i = 0;
    unsigned char last_mode = 10;
    //如果press_time < 20,
    if(gpio_get_level(pin) == 0){


        last_mode = gpio_get_level(pin);
        do{
            i++;
        }while(gpio_get_level(pin) != last_mode);


        if(i < 20 && gpio_get_level(pin) != last_mode){
            return Click;
        }
        else if(i >= 20 && gpio_get_level(pin) != last_mode){
            return Hold;
        }
    }
    return None;
}

unsigned char press_time = 0;
/*------------------------------------------------
 * 函数简介     按键按下时间记录
 * 返回参数     unsigned char
 * 使用示例     if(Press_Time < 20)
 * ----------------------------------------------*/
unsigned char Press_Time(){
//    if(key0_flag == 1 || key1_flag == 1 || key2_flag == 1 || key3_flag == 1 || key4_flag == 1 || key5_flag == 1 ){
//        press_time++;
//    }
        return press_time;
}

void Buzzer(uint8 value) { gpio_set_level(BEEP_PIN, value); }
