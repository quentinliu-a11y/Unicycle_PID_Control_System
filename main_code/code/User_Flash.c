/*---------------------------------
*   File name: User_Flash.c
*	
*	Author: LaZuli
*
*	Created time: 2024年4月18日
---------------------------------*/
#include "User_Flash.h"

/*------------------------------------------------
 * 函数简介     指定按键按下后将数据写入指定flash页面中
 * 输入参数     按键pin   页码page  数据data
 * 返回参数     void
 * 使用示例     FlashWrite(Key1,POINT_DATA_PAGE1,point_data[0][0])    //获取该点的经度值
 * 备注
 * ----------------------------------------------*/
float FlashWrite(gpio_pin_enum pin, char page, float data){
    if(gpio_get_level(pin) == 0){
        system_delay_ms(10);
        Buzzer(1);
        while(gpio_get_level(pin) == 0);
        Buzzer(0);

        //1.清除数据缓冲区
        flash_buffer_clear();
        //2.把数据存入缓冲区
        flash_union_buffer[page].float_type = data;
        //3.将数据写入flash中
        flash_write_page_from_buffer(0,page);
    }

    return data;
}

void FlashAutoSet(char page){
    if(flash_check(0,page)){
        flash_erase_page(0,page);
    }
}
