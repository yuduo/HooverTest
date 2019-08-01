/********************************************************************
* ��Ȩ �� -------------micony--------------
* �ļ� :motor.c
* �汾 ��
* ���� ��
* ���� : Τ����
* ʱ�� ��2015-2-13
* ˵�� ��
*/
#include "sys.h"





/*
��ɨ���
*/

//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void timer1_init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_DeInit(TIM1);//��ʼ��TIM1�Ĵ���
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA , ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); //Timer1��ȫ��ӳ��  TIM1_CH1->E9    
 
       //���ø�����Ϊ�����������,���TIM1 CH1��PWM���岨��	GPIOE.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //TIM1_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO


 
       //��ʼ��TIM1
	TIM_TimeBaseStructure.TIM_Period = MAX_PWM; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =(1-1); //����Ƶ 72M / 2K = 36K
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM1 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ�� 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_Pulse =800;
	
	//TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC1
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC1

	//TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
 	TIM_ARRPreloadConfig(TIM1, ENABLE);                //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//����PMW�����
	//TIM1->CCR2=700;	
	//TIM1->CCR4=700;	


}

void timer1_pwm_init(void)
{

	#if 0
	RCC->APB2ENR|=1<<11; 		//TIM1ʱ��ʹ�� 
	//�������仰����ʼ��PA8��PA9��PA10��PA11Ϊ���
	//GPIOA->CRH&=0XFFFFFFF0;		
	//GPIOA->CRH|=0X0000000B;	
	//GPIOA->ODR|=1<<8;			



	
	TIM1->ARR=MAX_PWM;			//�趨�������Զ���װֵ 	Ϊ 1000
	TIM1->PSC=0;//(2-1);			// ����2��Ƶ����ʱ��Ƶ��Ϊ 72M /4 = 18M
	
	TIM1->CCMR1|=6<<4; //CH1 PWM2ģʽ 
	TIM1->CCMR1|=1<<3; //CH1Ԥװ��ʹ��
	
	TIM1->CCMR1|=6<<12; //CH2 PWM2ģʽ 
	TIM1->CCMR1|=1<<11; //CH2Ԥװ��ʹ��
	
	TIM1->CCMR2|=6<<4; //CH3 PWM2ģʽ
	TIM1->CCMR2|=1<<3; //CH3Ԥװ��ʹ�� 

	TIM1->CCMR2|=6<<12; //CH4 PWM2ģʽ 
	TIM1->CCMR2|=1<<11; //CH4Ԥװ��ʹ�� 


	TIM1->CR1|=1<<7; //ARPEʹ���Զ���װ��Ԥװ������λ 
	TIM1->CR1|=1<<4; //���¼���ģʽ 

	TIM1->BDTR |=1<<15; //����OC��OCN�� 

	TIM1->CCR1 = 500; //�趨ռ�ձ�0~100%
	TIM1->CCR2 = 500; //�趨ռ�ձ�0~100%


	TIM1->CCR3 = 500; //�趨ռ�ձ�0~100%
	TIM1->CCR4 = 500; //�趨ռ�ձ�0~100%  ȫ�͵�ƽ
	


	TIM1->EGR |= 1<<0; //��ʼ�����еļĴ��� 
	TIM1->CR1|=1<<0; //ʹ�ܶ�ʱ��3 

	EN_CH1();		
	//EN_CH2();
	//EN_CH3();
	//EN_CH4();
	#else
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM1_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM1_OCInitStructure;
	/* TIM1 Peripheral Configuration */
	TIM_DeInit(TIM1);

	/* Time Base configuration */
	TIM1_TimeBaseStructure.TIM_Prescaler = 100;//1500;
	TIM1_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM1_TimeBaseStructure.TIM_Period = 1000;//0x390; //0x3b0

	TIM1_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM1_TimeBaseStructure.TIM_RepetitionCounter = 0x0;

	TIM_TimeBaseInit(TIM1,&TIM1_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM1,ENABLE);


	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_OCMode_PWM1;//TIM1_ForcedAction_Active;// TIM1_OCMode_PWM2;
	TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM1_OCInitStructure.TIM_Pulse =  0;
	TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_High;
	//TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

#if UI_HW_VERSION >= 130
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); //Timer1��ȫ��ӳ��  TIM1_CH1->E9  
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO , ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); //Timer1��ȫ��ӳ��  TIM1_CH1->E9    
 
       //���ø�����Ϊ�����������,���TIM1 CH1��PWM���岨��	GPIOE.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; //TIM1_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIO  
	//GPIOE->ODR |= GPIO_Pin_14;
	
	TIM_OC4Init(TIM1,&TIM1_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
#else
	TIM_OC1Init(TIM1,&TIM1_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
#endif
	
	/* TIM1 counter enable */
	TIM_Cmd(TIM1,ENABLE);
	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	#endif
}

void timer2_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;  //ʱ��Ƶ�ʵ�Ԥ��Ƶֵ 35999->17999 250us
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=10000 - 1;// - (36*10);  //100us
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;// 1��Ƶ 36M
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0x0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);  //��ʼ��TIM2��ʱ���������

	TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);     //���ж�
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	TIM_Cmd(TIM2,ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);      //ʱ������
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                     //NVIC����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


#define PWM_VALUE_INIT	0
void timer3_pwm_init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM3_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM3_OCInitStructure;
	/* TIM1 Peripheral Configuration */
	TIM_DeInit(TIM3);

	/* Time Base configuration */
	TIM3_TimeBaseStructure.TIM_Prescaler = 500;//1500;
	TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM3_TimeBaseStructure.TIM_Period = 2000;//0x390; //0x3b0

	TIM3_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM3_TimeBaseStructure.TIM_RepetitionCounter = 0x0;

	TIM_TimeBaseInit(TIM3,&TIM3_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM3,ENABLE);


	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM3_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_OCMode_PWM1;//TIM1_ForcedAction_Active;// TIM1_OCMode_PWM2;
	TIM3_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM3_OCInitStructure.TIM_Pulse =  PWM_VALUE_INIT;
	TIM3_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_High;
	//TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM3_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM3_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;


 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); //Timer1��ȫ��ӳ��  TIM1_CH1->E9    
 
       //���ø�����Ϊ�����������,���TIM1 CH1��PWM���岨��	GPIOE.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //TIM1_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO  
	//GPIOE->ODR |= GPIO_Pin_14;
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7; //TIM1_CH1
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO  
	
	TIM_OC1Init(TIM3,&TIM3_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM3,&TIM3_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM3,&TIM3_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM3,&TIM3_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	
	/* TIM1 counter enable */
	TIM_Cmd(TIM3,ENABLE);
	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
}

//extern __IO uint16_t *pledpwms[2];	
void timer4_pwm_init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM4_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM4_OCInitStructure;
	/* TIM1 Peripheral Configuration */
	TIM_DeInit(TIM4);

	/* Time Base configuration */
	TIM4_TimeBaseStructure.TIM_Prescaler = 1500;//1500;
	TIM4_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM4_TimeBaseStructure.TIM_Period = 200;//0x390; //0x3b0

	TIM4_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM4_TimeBaseStructure.TIM_RepetitionCounter = 0x0;

	TIM_TimeBaseInit(TIM4,&TIM4_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM4,ENABLE);


	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM4_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_OCMode_PWM1;//TIM1_ForcedAction_Active;// TIM1_OCMode_PWM2;
	TIM4_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM4_OCInitStructure.TIM_Pulse =  PWM_VALUE_INIT;
	TIM4_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_High;
	//TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM4_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM4_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;


 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); //Timer1��ȫ��ӳ��  TIM1_CH1->E9    
 
       //���ø�����Ϊ�����������,���TIM1 CH1��PWM���岨��	GPIOE.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; //TIM1_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO  
	//GPIOE->ODR |= GPIO_Pin_14;
	
	TIM_OC3Init(TIM4,&TIM4_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM4,&TIM4_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
	
	
	/* TIM1 counter enable */
	TIM_Cmd(TIM4,ENABLE);
	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	//*pledpwms[0] = *pledpwms[1] = 100;
}

#if 0
void motor_timer_init(void)
{

 TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
 //  TIM_ICInitTypeDef  	   TIM_ICInitStructure;
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
	
	TIM_TimeBaseStructure.TIM_Period = MAX_PWM;	
  
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
  


}

#endif

