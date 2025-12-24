/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   MPU6050����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F103-MINI STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */


#include "mpu6050.h"
#include "bsp_usart.h"



#ifdef   soft_IIC
#include "bsp_soft_i2c.h"

/**
  * @brief   д���ݵ�MPU6050�Ĵ���
  * @param   
  * @retval  
  */
void MPU6050_WriteReg(u8 reg_add,u8 reg_dat)
{
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);
	i2c_WaitAck();
	i2c_SendByte(reg_add);
	i2c_WaitAck();
	i2c_SendByte(reg_dat);
	i2c_WaitAck();
	i2c_Stop();
}


/**
  * @brief   ��MPU6050�Ĵ�����ȡ����
  * @param   
  * @retval  
  */
void MPU6050_ReadData(u8 reg_add,unsigned char*Read,u8 num)
{
	unsigned char i;
	
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS);
	i2c_WaitAck();
	i2c_SendByte(reg_add);
	i2c_WaitAck();
	
	i2c_Start();
	i2c_SendByte(MPU6050_SLAVE_ADDRESS+1);
	i2c_WaitAck();
	
	for(i=0;i<(num-1);i++){
		*Read=i2c_ReadByte(1);
		Read++;
	}
	*Read=i2c_ReadByte(0);
	i2c_Stop();
}


#else
#include "./iic/bsp_hard_i2c.h"

/**
 * @brief  ���Ĵ����������ṩ���ϲ�Ľӿ�
 * @param  slave_addr: �ӻ���ַ
 * @param 	reg_addr:�Ĵ�����ַ
 * @param len��Ҫ��ȡ�ĳ���
 *	@param data_ptr:ָ��Ҫ�洢���ݵ�ָ��
 * @retval ����Ϊ0��������Ϊ��0
 */
void MPU6050_ReadData(u8 reg_add,unsigned char*Read,u8 num)
{
    I2C_BufferRead(Read,reg_add,num);
}

/**
 * @brief  д�Ĵ����������ṩ���ϲ�Ľӿ�
 * @param  slave_addr: �ӻ���ַ
 * @param 	reg_addr:�Ĵ�����ַ
 * @param len��д��ĳ���
 *	@param data_ptr:ָ��Ҫд�������
 * @retval ����Ϊ0��������Ϊ��0
 */
void MPU6050_WriteReg(u8 reg_add,u8 reg_dat)
{
    I2C_ByteWrite(reg_dat,reg_add);
}


#endif



/**
  * @brief   ��ʼ��MPU6050оƬ
  * @param   
  * @retval  
  */
void MPU6050_Init(void)
{
    int i, j;
    
    printf("开始MPU6050初始化...\r\n");
    
    // 确保I2C GPIO已配置
    i2c_GPIO_Config();
    
    // 更长的延时等待电源稳定
    for(i = 0; i < 3000; i++) {
        for(j = 0; j < 1000; j++) {
            ;
        }
    }
    
    // 尝试多次读取WHO_AM_I
    uint8_t whoami = 0;
    int retry_count = 5;
    
    for(i = 0; i < retry_count; i++) {
        printf("尝试 %d/%d: 读取WHO_AM_I...", i+1, retry_count);
        
        MPU6050_ReadData(MPU6050_RA_WHO_AM_I, &whoami, 1);
        
        if(whoami == 0x68 || whoami == 0x69) {
            printf("✅ 成功: 0x%02X\r\n", whoami);
            break;
        } else {
            printf("❌ 失败: 0x%02X\r\n", whoami);
            delay_ms(100);
        }
    }
    
    if(whoami != 0x68 && whoami != 0x69) {
        printf("MPU6050初始化失败，无法读取正确ID\r\n");
        return;
    }
    
    // 继续正常初始化流程
    MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);
    delay_ms(100);
    
    MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV, 0x07);
    MPU6050_WriteReg(MPU6050_RA_CONFIG, 0x06);
    MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);
    MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG, 0x00);
    
    printf("✅ MPU6050初始化成功\r\n");
}

/**
  * @brief   ��ȡMPU6050��ID
  * @param   
  * @retval  
  */
uint8_t MPU6050ReadID(void)
{
    unsigned char Re = 0;
    MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);

    if(Re != 0x68)
    {
        printf("MPU6050 detected error!\r\n未检测到 MPU6050 模块，请检查连接。\r\n");
        return 0;
    }
    else
    {
        printf("MPU6050 ID = %d\r\n",Re);
        return 1;
    }
}

/**
  * @brief   ��ȡMPU6050�ļ��ٶ�����
  * @param   
  * @retval  
  */
void MPU6050ReadAcc(short *accData)
{
    u8 buf[6];
    MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   ��ȡMPU6050�ĽǼ��ٶ�����
  * @param   
  * @retval  
  */
void MPU6050ReadGyro(short *gyroData)
{
    u8 buf[6];
    MPU6050_ReadData(MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}


/**
  * @brief   ��ȡMPU6050��ԭʼ�¶�����
  * @param   
  * @retval  
  */
void MPU6050ReadTemp(short *tempData)
{
	u8 buf[2];
    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
    *tempData = (buf[0] << 8) | buf[1];
}


/**
  * @brief   ��ȡMPU6050���¶����ݣ�ת�������϶�
  * @param   
  * @retval  
  */
void MPU6050_ReturnTemp(float *Temperature)
{
	short temp3;
	u8 buf[2];
	
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
  temp3= (buf[0] << 8) | buf[1];	
	*Temperature=((double) temp3/340.0)+36.53;

}
