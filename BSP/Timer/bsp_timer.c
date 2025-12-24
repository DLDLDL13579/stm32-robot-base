#include "bsp_timer.h"


static u16 stop_time = 0;//ÑÓ³ÙÊ±¼ä  delay time
static uint32_t monotonic_time_ms = 0;


//¶¨Ê±Æ÷6×öÑÓ³Ù 10msµÄÑÓ³Ù ´Ë·½·¨±Èdelay×¼È·
//Timer 6 has a delay of 10ms. This method is more accurate than delay
void delay_time(u16 time)
{
	stop_time = time;
	while(stop_time);//ËÀµÈ Wait
}

//ÑÓ³Ù1s  Unit second
void my_delay(u16 s)//s
{
	for(int i = 0;i<s;i++)
	{
		delay_time(100);
	}
}


/**************************************************************************
Function function: TIM3 initialization, timed for 10 milliseconds
Entrance parameters: None
Return value: None
º¯Êý¹¦ÄÜ£ºTIM3³õÊ¼»¯£¬¶¨Ê±10ºÁÃë
Èë¿Ú²ÎÊý£ºÎÞ
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/
void TIM3_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //Ê¹ÄÜ¶¨Ê±Æ÷µÄÊ±ÖÓ  Enable the clock of the timer
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;			 // Ô¤·ÖÆµÆ÷  Prescaler
	TIM_TimeBaseStructure.TIM_Period = 99;				 //Éè¶¨¼ÆÊýÆ÷×Ô¶¯ÖØ×°Öµ  Set the automatic reset value of the counter
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);                //Çå³ýTIMµÄ¸üÐÂ±êÖ¾Î» Clear the update flag of TIM
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	//ÖÐ¶ÏÓÅÏÈ¼¶NVICÉèÖÃ
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;			  //TIM6ÖÐ¶Ï	TIM6 interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; //ÏÈÕ¼ÓÅÏÈ¼¶4¼¶	Preempts priority level 4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  //´ÓÓÅÏÈ¼¶2¼¶	From priority level 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQÍ¨µÀ±»Ê¹ÄÜ	IRQ channel is enabled
	NVIC_Init(&NVIC_InitStructure);							  //³õÊ¼»¯NVIC¼Ä´æÆ÷	Initializes NVIC registers

	TIM_Cmd(TIM3, ENABLE);
}


// TIM3ÖÐ¶Ï //TIM3 Interrupt service
// åœ¨TIM3ä¸­æ–­ä¸­æ›´æ–°
void TIM3_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        
        __disable_irq();
        monotonic_time_ms += 10; // æ¯10mså¢žåŠ 
        times++;
        __enable_irq();
        
        if(stop_time > 0) stop_time--;
    }
}

