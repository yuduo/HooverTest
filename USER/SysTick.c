#include "sys.h"

volatile uint32_t	msTmr=0,msledTmr=0; 
unsigned short sWorkTimeMinCt,sWorkTimeMin,sSleepTimeMin;

void SysTick_Init(void)
{
   	while(SysTick_Config(SystemCoreClock / 1000));
}

void SysTick_Handler()
{	
	msTmr++;
	msledTmr++;

	if(sWorkTimeMinCt++ > 60000)
	{
		sWorkTimeMinCt = 0;
		sWorkTimeMin ++;
		sSleepTimeMin++;
	}
}

uint8_t mstimeout(uint32_t *timep,uint32_t msec)
{
	uint32_t time,diff;
	uint8_t result=0;
	time=msTmr;
	diff=time- *timep;
	if((msec==0)||(diff>=msec))
	{
		*timep=time;
		result=1;
	}
	return result;
}

void delay(uint32_t msec)
{
	uint32_t LastTO;
	mstimeout(&LastTO,0);
	while(!mstimeout(&LastTO,msec))
	{
	}
}

void delay_sensor(uint32_t msec)
{
	uint32_t LastTO;
	mstimeout(&LastTO,0);
	while(!mstimeout(&LastTO,msec)) 
	{
		get_sensers(&syst.gSta);
	}
}

void timer5_config(void)//Main(loop) Timer configuration
{
	//Timer2 Config
	TIM_DeInit(TIM5);
	TIM_InternalClockConfig(TIM5);
	TIM5->PSC = 71;	// Set prescaler (PSC + 1)
	TIM5->ARR = 65535;//TIM2_AUTORELOADVALUE;           // Auto reload value 2000
	TIM5->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	TIM5->CNT=0;
	TIM5->CR1 = TIM_CR1_CEN;   // Enable timer
//	NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt from TIM2 (NVIC level)
}

void delay_ms_tm5(uint32_t ms)
{

	
	for(int i=0;i<ms;i++)
	{
		TIM5->CNT = 0;
		while(TIM5->CNT < 1000);
	}

}

void delay_ms(uint32_t ms)
{
	int c=0,k;
	k = (ms * 10) / 3;
	while(1)
	{
		c++;
		if(c>=k)
		{
			return ;
		}			
	}
}

void delay_ms_sensers(uint32_t ms)
{
	int c=0,k;
	k = (ms * 10) / 3;
	while(1)
	{
		c++;
		if(c>=k)
		{
			return ;
		}		
	}
}

