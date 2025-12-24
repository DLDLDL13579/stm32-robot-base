/**
* @par Copyright (C): 2016-2026, Shenzhen Yahboom Tech
* @file         // ALLHeader.h
* @author       // lly
* @version      // V1.0
* @date         // 240628
* @brief        // 相关所有的头文件
* @details      
* @par History  //
*               
*/


#ifndef __ALLHEADER_H
#define __ALLHEADER_H


//头文件
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

extern uint8_t times;

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

#include "myenum.h"

#include "delay.h"
#include "bsp.h"
#include "usart.h"
#include "IOI2C.h"
#include "bsp_motor_iic.h"
#include "bsp_hard_i2c.h"
#include "bsp_usart.h"

#include "mpu6050.h"     // MPU6050椹卞姩
#include "bsp_timer.h"
void Car_Move(void);



#endif


