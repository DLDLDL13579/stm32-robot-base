#include "ioi2c.h"

/**************************************************************************
Function: Simulate IIC start signal
Input   : none
Output  : 1
º¯Êý¹¦ÄÜ£ºÄ£ÄâIICÆðÊ¼ÐÅºÅ
Èë¿Ú²ÎÊý£ºÎÞ
·µ»Ø  Öµ£º1
**************************************************************************/
int IIC_Start(void)
{
	SDA_OUT();     //sdaÏßÊä³ö	sda line output
	IIC_SDA=1;
	if(!READ_SDA)return 0;	
	IIC_SCL=1;
	delay_us(1);
 	IIC_SDA=0; //START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return 0;
	delay_us(1);
	IIC_SCL=0;//Ç¯×¡I2C×ÜÏß£¬×¼±¸·¢ËÍ»ò½ÓÊÕÊý¾Ý 	Clamp the I2C bus and prepare to send or receive data
	return 1;
}

/**************************************************************************
Function: Analog IIC end signal
Input   : none
Output  : none
º¯Êý¹¦ÄÜ£ºÄ£ÄâIIC½áÊøÐÅºÅ
Èë¿Ú²ÎÊý£ºÎÞ
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sdaÏßÊä³ö	sda line output
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	IIC_SCL=1; 
	IIC_SDA=1;//·¢ËÍI2C×ÜÏß½áÊøÐÅºÅ	Send I2C bus end signal
	delay_us(1);							   	
}

/**************************************************************************
Function: IIC wait the response signal
Input   : none
Output  : 0£ºNo response received£»1£ºResponse received
º¯Êý¹¦ÄÜ£ºIICµÈ´ýÓ¦´ðÐÅºÅ
Èë¿Ú²ÎÊý£ºÎÞ
·µ»Ø  Öµ£º0£ºÃ»ÓÐÊÕµ½Ó¦´ð£»1£ºÊÕµ½Ó¦´ð
**************************************************************************/
int IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDAÉèÖÃÎªÊäÈë  SDA is set as input
	IIC_SDA=1;
	delay_us(1);	   
	IIC_SCL=1;
	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL=0;//Ê±ÖÓÊä³ö0 	 Clock output 0  
	return 1;  
} 

/**************************************************************************
Function: IIC response
Input   : none
Output  : none
º¯Êý¹¦ÄÜ£ºIICÓ¦´ð
Èë¿Ú²ÎÊý£ºÎÞ
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
	
/**************************************************************************
Function: IIC don't reply
Input   : none
Output  : none
º¯Êý¹¦ÄÜ£ºIIC²»Ó¦´ð
Èë¿Ú²ÎÊý£ºÎÞ
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
/**************************************************************************
Function: IIC sends a byte
Input   : txd£ºByte data sent
Output  : none
º¯Êý¹¦ÄÜ£ºIIC·¢ËÍÒ»¸ö×Ö½Ú
Èë¿Ú²ÎÊý£ºtxd£º·¢ËÍµÄ×Ö½ÚÊý¾Ý
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/	  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  SDA_OUT(); 	    
    IIC_SCL=0;//À­µÍÊ±ÖÓ¿ªÊ¼Êý¾Ý´«Êä	Pull the clock low to start data transmission
    for(t=0;t<8;t++)
    {              
			IIC_SDA=(txd&0x80)>>7;
			txd<<=1; 	  
			delay_us(1);   
			IIC_SCL=1;
			delay_us(1); 
			IIC_SCL=0;	
			delay_us(1);
    }	 
} 	 
  
/**************************************************************************
Function: IIC write data to register
Input   : addr£ºDevice address£»reg£ºRegister address£»len;Number of bytes£»data£ºData
Output  : 0£ºWrite successfully£»1£ºFailed to write
º¯Êý¹¦ÄÜ£ºIICÐ´Êý¾Ýµ½¼Ä´æÆ÷
Èë¿Ú²ÎÊý£ºaddr£ºÉè±¸µØÖ·£»reg£º¼Ä´æÆ÷µØÖ·£»len;×Ö½ÚÊý£»data£ºÊý¾Ý
·µ»Ø  Öµ£º0£º³É¹¦Ð´Èë£»1£ºÃ»ÓÐ³É¹¦Ð´Èë
**************************************************************************/
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
		int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1 );
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
		for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}
/**************************************************************************
Function: IIC read register data
Input   : addr£ºDevice address£»reg£ºRegister address£»len;Number of bytes£»*buf£ºData read out
Output  : 0£ºRead successfully£»1£ºFailed to read
º¯Êý¹¦ÄÜ£ºIIC¶Á¼Ä´æÆ÷µÄÊý¾Ý
Èë¿Ú²ÎÊý£ºaddr£ºÉè±¸µØÖ·£»reg£º¼Ä´æÆ÷µØÖ·£»len;×Ö½ÚÊý£»*buf£º¶Á³öÊý¾Ý»º´æ
·µ»Ø  Öµ£º0£º³É¹¦¶Á³ö£»1£ºÃ»ÓÐ³É¹¦¶Á³ö
**************************************************************************/

int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1)+1);
    IIC_Wait_Ack();
    while (len) {
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}

/**************************************************************************
Function: IIC reads a byte
Input   : ack£ºSend response signal or not£»1£ºSend£»0£ºDo not send
Output  : receive£ºData read
º¯Êý¹¦ÄÜ£ºIIC¶ÁÈ¡Ò»¸öÎ»
Èë¿Ú²ÎÊý£ºack£ºÊÇ·ñ·¢ËÍÓ¦´ðÐÅºÅ£»1£º·¢ËÍ£»0£º²»·¢ËÍ
·µ»Ø  Öµ£ºreceive£º¶ÁÈ¡µÄÊý¾Ý
**************************************************************************/ 
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDAÉèÖÃÎªÊäÈë	SDA is set as input
    for(i=0;i<8;i++ )
	 {
			IIC_SCL=0; 
			delay_us(2);
			IIC_SCL=1;
			receive<<=1;
			if(READ_SDA)receive++;   
			delay_us(2); 
    }					 
    if (ack)
        IIC_Ack(); //·¢ËÍACK 	Send ACK
    else
        IIC_NAck();//·¢ËÍnACK 	Send nACK 
    return receive;
}
// åœ¨æ–‡ä»¶æœ«å°¾æ·»åŠ ä»¥ä¸‹å‡½æ•°
int i2cWriteWithRetry(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data, uint8_t retries)
{
    int result;
    for(uint8_t i = 0; i < retries; i++) {
        result = i2cWrite(addr, reg, len, data);
        if(result == 0) {
            return 0; // æˆåŠŸ
        }
        delay_ms(1); // é‡è¯•é—´éš”
    }
    return result; // è¿”å›žæœ€åŽä¸€æ¬¡é”™è¯¯
}

int i2cReadWithRetry(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf, uint8_t retries)
{
    int result;
    for(uint8_t i = 0; i < retries; i++) {
        result = i2cRead(addr, reg, len, buf);
        if(result == 0) {
            return 0; // æˆåŠŸ
        }
        delay_ms(1); // é‡è¯•é—´éš”
    }
    return result; // è¿”å›žæœ€åŽä¸€æ¬¡é”™è¯¯
}