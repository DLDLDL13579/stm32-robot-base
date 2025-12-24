#include "bsp_hard_i2c.h"
#include "mpu6050.h"
#include "bsp_usart.h"

// 删除或注释掉这行
// #include "./led/bsp_led.h"

void WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);

// I2C配置函数
void MPU_I2C_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_HARD_GPIO_PORT, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_HARD_I2C_PORT, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin   = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);
    
    I2C_DeInit(MPU_I2C);
    
    I2C_InitStructure.I2C_Ack                   = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress   = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed            = 400000;
    I2C_InitStructure.I2C_DutyCycle             = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_Mode                  = I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1           = I2Cx_OWN_ADDRESS7;
    I2C_Init(MPU_I2C, &I2C_InitStructure);
    I2C_Cmd(MPU_I2C, ENABLE);
}

// 字节写函数
void I2C_ByteWrite(uint8_t pBuffer, uint8_t WriteAddr)
{
    while(I2C_GetFlagStatus(MPU_I2C, I2C_FLAG_BUSY));
    I2C_GenerateSTART(MPU_I2C, ENABLE);
    while(I2C_CheckEvent(MPU_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
    
    I2C_Send7bitAddress(MPU_I2C, MPU6050_SLAVE_ADDRESS, I2C_Direction_Transmitter);
    WaitEvent(MPU_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    
    I2C_SendData(MPU_I2C, WriteAddr);
    WaitEvent(MPU_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
    
    I2C_SendData(MPU_I2C, pBuffer);
    WaitEvent(MPU_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    
    I2C_GenerateSTOP(MPU_I2C, ENABLE);
}

// 缓冲区读函数
void I2C_BufferRead(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
    while(I2C_GetFlagStatus(MPU_I2C, I2C_FLAG_BUSY));
    I2C_GenerateSTART(MPU_I2C, ENABLE);
    while(I2C_CheckEvent(MPU_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
    
    I2C_Send7bitAddress(MPU_I2C, MPU6050_SLAVE_ADDRESS, I2C_Direction_Transmitter);
    while(I2C_CheckEvent(MPU_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR);
    
    I2C_SendData(MPU_I2C, ReadAddr);
    while(I2C_CheckEvent(MPU_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);
    
    I2C_GenerateSTART(MPU_I2C, ENABLE);
    while(I2C_CheckEvent(MPU_I2C, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
    
    I2C_Send7bitAddress(MPU_I2C, MPU6050_SLAVE_ADDRESS, I2C_Direction_Receiver);
    while(I2C_CheckEvent(MPU_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR);

    while(NumByteToRead)
    {
        if(NumByteToRead == 1)
        {
            I2C_AcknowledgeConfig(MPU_I2C, DISABLE);
            I2C_GenerateSTOP(MPU_I2C, ENABLE);
        }

        WaitEvent(MPU_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED);
        {
            *pBuffer = I2C_ReceiveData(MPU_I2C);
            pBuffer++;
            NumByteToRead--;
        }
    }

    I2C_AcknowledgeConfig(MPU_I2C, ENABLE);
}

// 等待事件函数（删除LED相关代码）
void WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
    uint32_t timecount = 40000;
    while(I2C_CheckEvent(I2Cx, I2C_EVENT) == ERROR)
    {
        timecount--;
        if(timecount == 0)
        {
            // 删除LED1_ON;
            break;
        }
    }
}