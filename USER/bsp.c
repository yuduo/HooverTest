#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_it.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_bkp.h"
#include "sys.h"

#define ADC1_DR_Address    ((u32)0x4001244C)

__IO u16 adc_converted_value[ADC_CHANNEL_NUM+1];

void RTC_Init(void);
void RCC_Config(void);
void GPIO_Config(void);
void NVIC_Config(void);
void EXTI_Config(void);
void EXTI_cfg(void);

void Periph_Init(void)
{
	/*********时钟使能*************/
	RCC_Config();
	/********中断优先级设置********/
	NVIC_Config();
	EXTI_cfg();
	GPIO_Config();
	senser_gpio_config();
	key_io_init();
	RTC_Init();
}

void EXTI_cfg()
{
}

void RCC_Config(void)
{
	SystemInit();

	/* 使能GPIO(A~C)|ADC1|USART1 时钟 */
	RCC_APB2PeriphClockCmd(   RCC_APB2Periph_GPIOA 
							| RCC_APB2Periph_GPIOB
			 	 	 	 	| RCC_APB2Periph_GPIOC 
			 	 	 	 	| RCC_APB2Periph_GPIOD
			 	 	 	 	| RCC_APB2Periph_GPIOE
							| RCC_APB2Periph_AFIO
							| RCC_APB2Periph_TIM1
							| RCC_APB2Periph_ADC1
							| RCC_APB2Periph_TIM8  
							| RCC_APB2Periph_USART1
 							, ENABLE );
//	 RCC_APB1PeriphClockCmd(  RCC_APB1Periph_USART2 
//	 						, ENABLE );
/******************CAN时钟使能**************************/
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
 	RCC_APB1PeriphClockCmd(   RCC_APB1Periph_TIM3
							| RCC_APB1Periph_TIM4
							| RCC_APB1Periph_TIM5  
							| RCC_APB1Periph_TIM2
							, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);	
	
 TIM_ARRPreloadConfig(TIM3, ENABLE);
}

void  GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	//PE9 前照灯控制
	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//PC9 前撞红外控制
	GPIO_InitStructure.GPIO_Pin =   PIN_IR_CTRL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_IR_CTRL, &GPIO_InitStructure);

	//PD0 IN�?
	GPIO_InitStructure.GPIO_Pin = PIN_AC;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_AC,&GPIO_InitStructure);

	//PE2 电机控制
	GPIO_InitStructure.GPIO_Pin = PIN_MOTOR_POWER;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_MOTOR_POWER, &GPIO_InitStructure);

	//PC12 PC13 风机控制
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void NVIC_Config(void)
{
}

void Delayus(int t)
{
   while(t--)
   {
   }
}

void TargetSysReset(void)
{
	 __set_FAULTMASK(1);
	 SCB->AIRCR = 0x05FA0000 | (u32)0x04;
	 while(1);
}

void RTC_RCCCfg(void)
{
	//启用PWR和BKP的时钟（from APB1）
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	//后备域解锁
	PWR_BackupAccessCmd(ENABLE);

	//备份寄存器模块复位
	BKP_DeInit();

	//外部32.768K其哟偶那个
	RCC_LSEConfig(RCC_LSE_ON);
	//等待稳定
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

	//RTC时钟源配置成LSE（外部32.768K）
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//这里使用LSI作RTC时钟源，因此工程用不到RTC,只用到后备RAM
	//RTC开启
	RCC_RTCCLKCmd(ENABLE);

	//开启后需要等待APB1时钟与RTC时钟同步，才能读写寄存器
	RTC_WaitForSynchro();

	//读写寄存器前，要确定上一个操作已经结束
	RTC_WaitForLastTask();

	//设置RTC分频器，使RTC时钟为1Hz
	//RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1)
	RTC_SetPrescaler(32767);

	//等待寄存器写入完成
	RTC_WaitForLastTask();

	RCC_RTCCLKCmd(ENABLE);
	//等待RTC时钟与APB1时钟同步
	RTC_WaitForSynchro();

	//使能秒中断
	//RTC_ITConfig(RTC_IT_SEC, ENABLE);
	//使能闹钟中断
	//RTC_ITConfig(RTC_IT_ALR, ENABLE);// | RTC_IT_SEC  

	//等待写入完成
	RTC_WaitForLastTask();

	return;
}

void RTC_Config(void)
{
	//第一次上电或后备电源掉电后，该寄存器数据丢失，
	//表明RTC数据丢失，需要重新配置
	if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
	{
		//重新配置RTC
		RTC_RCCCfg();
		//配置完成后，向后备寄存器中写特殊字符0xA5A5
		BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
	}
	else
	{
		//若后备寄存器没有掉电，则无需重新配置RTC
		//这里我们可以利用RCC_GetFlagStatus()函数查看本次复位类型
		if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
		{
			//这是上电复位
		}
		else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
		{
			//这是外部RST管脚复位
		}
		//清除RCC中复位标志
		RCC_ClearFlag();

		//虽然RTC模块不需要重新配置，且掉电后依靠后备电池依然运行
		//但是每次上电后，还是要使能RTCCLK???????
		RCC_RTCCLKCmd(ENABLE);
		//等待RTC时钟与APB1时钟同步
		RTC_WaitForSynchro();

		//使能秒中断
		//RTC_ITConfig(RTC_IT_SEC, ENABLE);
		//使能闹钟中断
		//RTC_ITConfig(RTC_IT_ALR, ENABLE);// | RTC_IT_SEC
		//等待操作完成
		RTC_WaitForLastTask();
	}

	return;
}
 
void RTC_Init(void)
{
	RTC_Config();
}

static void ADC1_GPIO_Config(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);

	/* Configure PC.01  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);			

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

}


static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)adc_converted_value;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = ADC_CHANNEL_NUM;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;    
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
     
	/* ADC1 configuration */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = ADC_CHANNEL_NUM;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 16, ADC_SampleTime_55Cycles5);	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 15, ADC_SampleTime_55Cycles5);	
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 14, ADC_SampleTime_55Cycles5);	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 13, ADC_SampleTime_55Cycles5);	
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 12, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 11, ADC_SampleTime_55Cycles5);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_55Cycles5);	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_55Cycles5);	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_55Cycles5); 

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 8, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 10, ADC_SampleTime_55Cycles5);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));
	 
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}


void kio_init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}

void sensers_timr_init(void)
{
}

void senser_gpio_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//PB15 PE7 PC8 回充检测
	GPIO_InitStructure.GPIO_Pin = IR_L_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IR_L_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = IR_M_GPIO_PIN;
	GPIO_Init(IR_M_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = IR_R_GPIO_PIN;
	GPIO_Init(IR_R_GPIO_PORT, &GPIO_InitStructure);

	//PD8 PD9 PE6 PE12 碰撞检测
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_12;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	//PE10 左后轮凌空
	GPIO_InitStructure.GPIO_Pin = PIN_LEFT_MOTOR_LEAVE;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_LEFT_MOTOR_LEAVE,&GPIO_InitStructure);  

	//PB12 右后轮凌空
	GPIO_InitStructure.GPIO_Pin = PIN_RIGHT_MOTOR_LEAVE;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_RIGHT_MOTOR_LEAVE,&GPIO_InitStructure);	

	//PD7 虚拟墙检测
	GPIO_InitStructure.GPIO_Pin = PIN_VWALL_DET;
	GPIO_Init(PORT_VWALL_DET,&GPIO_InitStructure);	

	//PB5 尘盒
	GPIO_InitStructure.GPIO_Pin = PIN_DUSTBOX_DET;
	GPIO_Init(PORT_DUSTBOX_DET,&GPIO_InitStructure);  

	//PD10 开关检测
	GPIO_InitStructure.GPIO_Pin = PIN_PSW;
	GPIO_Init(PORT_PSW,&GPIO_InitStructure);  

	kio_init();
}

