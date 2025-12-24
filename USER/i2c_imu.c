#include "i2c_imu.h"

// 添加超时机制
#define I2C_TIMEOUT 40000



// 带超时的等待函数
void WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
    uint32_t timecount = I2C_TIMEOUT;
    while(I2C_CheckEvent(I2Cx, I2C_EVENT) == ERROR)
    {
        timecount--;
        if(timecount == 0) {
            printf("I2C event timeout: 0x%08lX\r\n", I2C_EVENT);  // 改为英文避免编码问题
            // LED1_ON;  // 暂时注释掉，如果需要请先定义LED控制
            break;
        }
    }
}

// I2C初始化
void I2C_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
    
    // 使能GPIOB和I2C1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    I2C_IMU_CLK_CMD(I2C_IMU_CLK_Periph, ENABLE);
    
    // 配置I2C引脚：PB6(SCL), PB7(SDA)
    GPIO_InitStructure.GPIO_Pin = I2C_IMU_SCL_PIN | I2C_IMU_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_IMU_PORT, &GPIO_InitStructure);
    
    // I2C配置
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;  // 降低到100kHz提高稳定性
    
    I2C_Init(I2C_IMU, &I2C_InitStructure);
    I2C_Cmd(I2C_IMU, ENABLE);
    
    printf("I2C1 initialized (PB6-SCL, PB7-SDA, 100kHz)\r\n");
}

// 其他函数保持不变...
void I2C_Start(void)
{
    I2C_GenerateSTART(I2C_IMU, ENABLE);
    WaitEvent(I2C_IMU, I2C_EVENT_MASTER_MODE_SELECT);
}

uint8_t I2C_WaitAck(void)
{
    WaitEvent(I2C_IMU, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    return 0;
}

void I2C_SendByte(uint8_t data)
{
    I2C_SendData(I2C_IMU, data);
    WaitEvent(I2C_IMU, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
}

uint8_t I2C_ReadByte(void)
{
    WaitEvent(I2C_IMU, I2C_EVENT_MASTER_BYTE_RECEIVED);
    return I2C_ReceiveData(I2C_IMU);
}

void I2C_Stop(void)
{
    I2C_GenerateSTOP(I2C_IMU, ENABLE);
}

void I2C_Ack(void)
{
    I2C_AcknowledgeConfig(I2C_IMU, ENABLE);
}

void I2C_NAck(void)
{
    I2C_AcknowledgeConfig(I2C_IMU, DISABLE);
}