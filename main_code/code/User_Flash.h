/*---------------------------------
*   File name: User_Flash.h
*	
*	Author: LaZuli
*
*	Created time: 2024年4月18日
---------------------------------*/
#ifndef CODE_USER_FLASH_H_
#define CODE_USER_FLASH_H_

#include "zf_common_headfile.h"

#define POINT_DATA_PAGE1 0
#define POINT_DATA_PAGE2 1
#define POINT_DATA_PAGE3 2
#define PID_PARAMETER_PAGE1 3
#define PID_PARAMETER_PAGE2 4

/*-----------结构体初始化-----------*/

/*--------------------------------*/

/*-----------枚举数组初始化-----------*/

/*--------------------------------*/

/*-----------函数声明-----------*/
float FlashWrite(gpio_pin_enum pin, char page, float data);
void FlashAutoSet(char page);
/*--------------------------------*/

#endif /* CODE_USER_FLASH_H_ */
