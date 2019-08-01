#include "sys.h"

__IO uint16_t *left_pwm,*right_pwm;

void timer1_init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_DeInit(TIM1);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOE  | RCC_APB2Periph_AFIO, ENABLE);  
	
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); 
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = 1000; 
	TIM_TimeBaseStructure.TIM_Prescaler =(4-1); 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ�� 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_Pulse =0;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC1

	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
 	TIM_ARRPreloadConfig(TIM1, ENABLE);                //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//����PMW�����
}

void timer2_init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_DeInit(TIM2);//��ʼ��TIM1�Ĵ���
 	RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE); //Timer2��ȫ��ӳ��
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO

	// 72M /72 / 500 = 2k
	TIM_TimeBaseStructure.TIM_Period = 500; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =(72-1); //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ�� 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_Pulse =0;
	
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC1

	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
 	TIM_ARRPreloadConfig(TIM2, ENABLE);                //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM1	
}


void motor_timer_init(void)
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure; 

	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* GPIOD clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);           

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = (4-1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Down;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 800;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* TIM4 enable counter */
	TIM_Cmd(TIM4, ENABLE);

	DIS_CH1();
	DIS_CH2();
	DIS_CH3();
	DIS_CH4();
}

void motor_wheel_backward(u8 wheel,u16 speed)
{	
	TIM_OCInitTypeDef  TIM_OCInitStruct;	
	TIM_OCInitStruct.TIM_Pulse = speed;	
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High ;	
	
	if( READ_BIT(wheel,RIGHT_WHEEL))
	{		
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;		
		TIM_OC4Init(TIM4,&TIM_OCInitStruct);		
		TIM_OCInitStruct.TIM_OCMode = TIM_ForcedAction_Active;		
		TIM_OC3Init(TIM4,&TIM_OCInitStruct);		
		TIM_CCxCmd(TIM4,TIM_Channel_3,TIM_CCx_Enable);		
		TIM_CCxCmd(TIM4,TIM_Channel_4,TIM_CCx_Enable);		
		right_pwm= &(TIM4->CCR4);	
	}	
	if(READ_BIT(wheel,LEFT_WHEEL))
	{		
		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;		
		TIM_OC2Init(TIM4,&TIM_OCInitStruct);				
		TIM_OCInitStruct.TIM_OCMode = TIM_ForcedAction_Active;		
		TIM_OC1Init(TIM4,&TIM_OCInitStruct);		
		TIM_CCxCmd(TIM4,TIM_Channel_1,TIM_CCx_Enable);		
		TIM_CCxCmd(TIM4,TIM_Channel_2,TIM_CCx_Enable);		
		left_pwm  = &(TIM4->CCR2);	
	}
}

void motor_wheel_forward(u8 wheel,u16 speed)
{
	TIM_OCInitTypeDef  TIM_OCInitStruct;	
	TIM_OCInitStruct.TIM_Pulse = speed;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High ;	

	if( READ_BIT(wheel,RIGHT_WHEEL)){

		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OC3Init(TIM4,&TIM_OCInitStruct);

		TIM_OCInitStruct.TIM_OCMode = TIM_ForcedAction_Active;
		TIM_OC4Init(TIM4,&TIM_OCInitStruct);
		TIM_CCxCmd(TIM4,TIM_Channel_3,TIM_CCx_Enable);
		TIM_CCxCmd(TIM4,TIM_Channel_4,TIM_CCx_Enable);
		 right_pwm= &(TIM4->CCR3);
	}

	if(READ_BIT(wheel,LEFT_WHEEL)){

		TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OC1Init(TIM4,&TIM_OCInitStruct);
		
		TIM_OCInitStruct.TIM_OCMode = TIM_ForcedAction_Active;
		TIM_OC2Init(TIM4,&TIM_OCInitStruct);
		TIM_CCxCmd(TIM4,TIM_Channel_1,TIM_CCx_Enable);
		TIM_CCxCmd(TIM4,TIM_Channel_2,TIM_CCx_Enable);
		left_pwm = &(TIM4->CCR1);
	}
}


void motor_wheel_stop(u8 wheel)
{
	TIM_OCInitTypeDef  TIM_OCInitStruct;

	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High ;	

	if( READ_BIT(wheel,RIGHT_WHEEL)){

		TIM_OCInitStruct.TIM_OCMode = TIM_ForcedAction_Active;
		TIM_OC3Init(TIM4,&TIM_OCInitStruct);

		TIM_OCInitStruct.TIM_OCMode = TIM_ForcedAction_Active;
		TIM_OC4Init(TIM4,&TIM_OCInitStruct);
		TIM_CCxCmd(TIM4,TIM_Channel_3,TIM_CCx_Enable);
		TIM_CCxCmd(TIM4,TIM_Channel_4,TIM_CCx_Enable);
	}

	if(READ_BIT(wheel,LEFT_WHEEL)){

		TIM_OCInitStruct.TIM_OCMode = TIM_ForcedAction_Active;
		TIM_OC1Init(TIM4,&TIM_OCInitStruct);
		
		TIM_OCInitStruct.TIM_OCMode = TIM_ForcedAction_Active;
		TIM_OC2Init(TIM4,&TIM_OCInitStruct);
		TIM_CCxCmd(TIM4,TIM_Channel_1,TIM_CCx_Enable);
		TIM_CCxCmd(TIM4,TIM_Channel_2,TIM_CCx_Enable);
	}
}

void r_motor_set_pwm(uint8_t HL,int pwm)
{
	EN_CH1();
	EN_CH2();
	if(HL==GO_FORWARD)
	{
		TIM4->CCR2=0;
		TIM4->CCR1=1000 - pwm;
	}else
	{
		TIM4->CCR1=0;
		TIM4->CCR2=1000 - pwm;
	}
} 

