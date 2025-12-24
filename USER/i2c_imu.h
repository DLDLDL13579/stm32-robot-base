#ifndef __I2C_IMU_H
#define __I2C_IMU_H

#include "AllHeader.h"

// MPU6050地址定义
#define MPU6050_ADDRESS 0xD0  // 7位地址左移1位
#define MPU6050_WHO_AM_I 0x75
#define ERROR 0
#define SUCCESS 1
// I2C引脚定义
#define I2C_IMU_PORT GPIOB
#define I2C_IMU_SCL_PIN GPIO_Pin_6
#define I2C_IMU_SDA_PIN GPIO_Pin_7
#define I2C_IMU_CLK RCC_APB2Periph_GPIOB
#define I2C_IMU I2C1
#define I2C_IMU_CLK_CMD RCC_APB1PeriphClockCmd
#define I2C_IMU_CLK_Periph RCC_APB1Periph_I2C1

// 函数声明
void I2C_Configuration(void);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_WaitAck(void);
void I2C_Ack(void);
void I2C_NAck(void);
void I2C_SendByte(uint8_t data);
uint8_t I2C_ReadByte(void);

#endif