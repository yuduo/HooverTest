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
	/*********ʱ��ʹ��*************/
	RCC_Config();
	/********�ж����ȼ�����********/
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

	/* ʹ��GPIO(A~C)|ADC1|USART1 ʱ�� */
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
/******************CANʱ��ʹ��**************************/
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

	//PE9 ǰ�յƿ���
	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//PC9 ǰײ�������
	GPIO_InitStructure.GPIO_Pin =   PIN_IR_CTRL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_IR_CTRL, &GPIO_InitStructure);

	//PD0 IN�?
	GPIO_InitStructure.GPIO_Pin = PIN_AC;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_AC,&GPIO_InitStructure);

	//PE2 �������
	GPIO_InitStructure.GPIO_Pin = PIN_MOTOR_POWER;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_MOTOR_POWER, &GPIO_InitStructure);

	//PC12 PC13 �������
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
	//����PWR��BKP��ʱ�ӣ�from APB1��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	//�������
	PWR_BackupAccessCmd(ENABLE);

	//���ݼĴ���ģ�鸴λ
	BKP_DeInit();

	//�ⲿ32.768K��Ӵż�Ǹ�
	RCC_LSEConfig(RCC_LSE_ON);
	//�ȴ��ȶ�
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

	//RTCʱ��Դ���ó�LSE���ⲿ32.768K��
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//����ʹ��LSI��RTCʱ��Դ����˹����ò���RTC,ֻ�õ���RAM
	//RTC����
	RCC_RTCCLKCmd(ENABLE);

	//��������Ҫ�ȴ�APB1ʱ����RTCʱ��ͬ�������ܶ�д�Ĵ���
	RTC_WaitForSynchro();

	//��д�Ĵ���ǰ��Ҫȷ����һ�������Ѿ�����
	RTC_WaitForLastTask();

	//����RTC��Ƶ����ʹRTCʱ��Ϊ1Hz
	//RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1)
	RTC_SetPrescaler(32767);

	//�ȴ��Ĵ���д�����
	RTC_WaitForLastTask();

	RCC_RTCCLKCmd(ENABLE);
	//�ȴ�RTCʱ����APB1ʱ��ͬ��
	RTC_WaitForSynchro();

	//ʹ�����ж�
	//RTC_ITConfig(RTC_IT_SEC, ENABLE);
	//ʹ�������ж�
	//RTC_ITConfig(RTC_IT_ALR, ENABLE);// | RTC_IT_SEC  

	//�ȴ�д�����
	RTC_WaitForLastTask();

	return;
}

void RTC_Config(void)
{
	//��һ���ϵ��󱸵�Դ����󣬸üĴ������ݶ�ʧ��
	//����RTC���ݶ�ʧ����Ҫ��������
	if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
	{
		//��������RTC
		RTC_RCCCfg();
		//������ɺ���󱸼Ĵ�����д�����ַ�0xA5A5
		BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
	}
	else
	{
		//���󱸼Ĵ���û�е��磬��������������RTC
		//�������ǿ�������RCC_GetFlagStatus()�����鿴���θ�λ����
		if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
		{
			//�����ϵ縴λ
		}
		else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
		{
			//�����ⲿRST�ܽŸ�λ
		}
		//���RCC�и�λ��־
		RCC_ClearFlag();

		//��ȻRTCģ�鲻��Ҫ�������ã��ҵ���������󱸵����Ȼ����
		//����ÿ���ϵ�󣬻���Ҫʹ��RTCCLK???????
		RCC_RTCCLKCmd(ENABLE);
		//�ȴ�RTCʱ����APB1ʱ��ͬ��
		RTC_WaitForSynchro();

		//ʹ�����ж�
		//RTC_ITConfig(RTC_IT_SEC, ENABLE);
		//ʹ�������ж�
		//RTC_ITConfig(RTC_IT_ALR, ENABLE);// | RTC_IT_SEC
		//�ȴ��������
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

	//PB15 PE7 PC8 �س���
	GPIO_InitStructure.GPIO_Pin = IR_L_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IR_L_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = IR_M_GPIO_PIN;
	GPIO_Init(IR_M_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = IR_R_GPIO_PIN;
	GPIO_Init(IR_R_GPIO_PORT, &GPIO_InitStructure);

	//PD8 PD9 PE6 PE12 ��ײ���
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_12;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	//PE10 ��������
	GPIO_InitStructure.GPIO_Pin = PIN_LEFT_MOTOR_LEAVE;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_LEFT_MOTOR_LEAVE,&GPIO_InitStructure);  

	//PB12 �Һ������
	GPIO_InitStructure.GPIO_Pin = PIN_RIGHT_MOTOR_LEAVE;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_RIGHT_MOTOR_LEAVE,&GPIO_InitStructure);	

	//PD7 ����ǽ���
	GPIO_InitStructure.GPIO_Pin = PIN_VWALL_DET;
	GPIO_Init(PORT_VWALL_DET,&GPIO_InitStructure);	

	//PB5 ����
	GPIO_InitStructure.GPIO_Pin = PIN_DUSTBOX_DET;
	GPIO_Init(PORT_DUSTBOX_DET,&GPIO_InitStructure);  

	//PD10 ���ؼ��
	GPIO_InitStructure.GPIO_Pin = PIN_PSW;
	GPIO_Init(PORT_PSW,&GPIO_InitStructure);  

	kio_init();
}

