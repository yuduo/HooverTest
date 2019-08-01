//#include "includes.h"
#include "stdio.h"
#include "string.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_it.h"
#include "bsp.h"
#include "BLDC.h"
#include "usart.h"
/**********��������**********/


int	bldc_dir;
uint16_t 	bldcPWM;

#define IDLE    0
#define	START	1
#define	RUN	    2
#define	STOP    3
#define FAULT   4
#define HIGH	1480
#define LOW     3
/*********ȫ�ֱ���***********/
u8 state;				   //��״̬
FlagStatus Direction = SET;//��ʼ����Ϊ��ת
uint8_t stalling_count = 0;		   //��ת������
FlagStatus zheng_fan = RESET;//��ʼ����Ϊ��ת


/**********************************************************************
* Description    : �Զ�ʱ��1�Ͷ�ʱ��3��GPIO����
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void BLDC_GPIOConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
 	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;  //TIM1���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;						   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
			 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;//TIM1���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;						   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;              //TIM3�Ļ�������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;                         //TIM3�Ļ�������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB,GPIO_Pin_3) ;
	
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;                         //TIM1_BKIN
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
	   
}
/**********************************************************************
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void BLDC_TIM1Config(void)
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;			   //�����ṹ���������
   TIM_OCInitTypeDef  		TIM_OCInitStructure;               //����ṹ���������
   TIM_BDTRInitTypeDef  	TIM_BDTRInitStructure;			   //����ɲ���ṹ���������

   TIM_DeInit(TIM1);

/*
  PWM T=1/(72M/3) *1500
  PWM =	72M/3/1500 =72M/(3*1500)=16K
*/
   TIM_TimeBaseStructure.TIM_Prescaler = 2;					   //Ԥ��Ƶֵ
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned2;//����������ģʽ,����Ƚϱ�־λֻ���ڱȽ������ϼ��㱻����
   TIM_TimeBaseStructure.TIM_Period = 1500 - 1;					   //PWM 16K   ������һ������ʱ��װ�����Զ���װ�ؼĴ������ڵ�ֵ
   TIM_TimeBaseStructure.TIM_ClockDivision = 0;						//����ʱ�ӷָ�
   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;				   

   TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
 /*
   	PWMģʽ2��TIM1_CNT > TIM1_CCR1ʱΪ��Ч����
 */
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 		   //TIM���ͨ����ʼ��
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; 
   TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
   TIM_OCInitStructure.TIM_Pulse =200; 
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
   TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         
   TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//����������͵�ƽ��TIM_OCIdleState_Set;
   TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;          
  
   /*
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 		   //TIM���ͨ����ʼ��
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; 
   TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
   TIM_OCInitStructure.TIM_Pulse =300; 
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
   TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         
   TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//TIM_OCIdleState_Set;
   TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;          
  
  TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;                    //����Ϊpwm1���ģʽ
  TIM_OCInitStructure.TIM_Pulse=300;                   //����ռ�ձ�ʱ��
  TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;                 //�����������
  TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;        //ʹ�ܸ�ͨ�����
  TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_Low;        //���û������������
  TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;//ʹ�ܻ��������
  TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;        //���������״̬
  TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//�����󻥲������״̬
   TIM_OC1Init(TIM1,&TIM_OCInitStructure); 
  */ 
  // TIM_OCInitStructure.TIM_Pulse =300;
   TIM_OC1Init(TIM1,&TIM_OCInitStructure);
   TIM_OC2Init(TIM1,&TIM_OCInitStructure);
   TIM_OC3Init(TIM1,&TIM_OCInitStructure);

/*
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 		   //TIM���ͨ��4��ʼ������������ADע�����
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;                   
   TIM_OCInitStructure.TIM_Pulse =1495; 
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;           
  
   TIM_OC4Init(TIM1,&TIM_OCInitStructure); 

   */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//TIM_OCMode_PWM1;////CCR4������ʹ�ܺ������������ADC1��ע��ͨ������
	TIM_OCInitStructure.TIM_Pulse = 200;    
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//TIM_OCPolarity_High; 	//����Ƚϼ��Ըߡ�
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

 
   TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;	//����ɲ����ʼ��
   TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
   TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
   TIM_BDTRInitStructure.TIM_DeadTime = 100;
   TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;	 //��򿪣������������״̬���ң�������
   TIM_BDTRInitStructure.TIM_BreakPolarity =  TIM_BreakPolarity_Low ;
   TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

   TIM_BDTRConfig(TIM1,&TIM_BDTRInitStructure);


   TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);  //ʹ�ܲ���ȽϼĴ���Ԥװ�أ�ͨ��1��

   TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);	 //ʹ�ܲ���ȽϼĴ���Ԥװ�أ�ͨ��2��

   TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);	 //ʹ�ܲ���ȽϼĴ���Ԥװ�أ�ͨ��3��

 //  TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);	 //ʹ�ܲ���ȽϼĴ���Ԥװ�أ�ͨ��3��
   
   TIM_SelectInputTrigger(TIM1, TIM_TS_ITR2);        //���봥��Դѡ��TIM3   
  
   //TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Trigger);//��ģʽѡ�� ����	  

   TIM_CtrlPWMOutputs(TIM1,ENABLE);		//����ʲô��ģʽ��ò��PWM��Ե����顣

//   TIM_ClearITPendingBit(TIM1, TIM_IT_Break|TIM_IT_COM);
   TIM_ITConfig(TIM1, TIM_IT_Break | TIM_IT_COM ,ENABLE);
  //  TIM_ITConfig(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3, ENABLE);
 //  TIM_ClearITPendingBit( TIM1, TIM_IT_COM);
 //  TIM_ITConfig(TIM1, TIM_IT_COM ,ENABLE);
  // TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);	 //CCR4���жϣ����ͨ������CCR4��pulse�����Ʋ����ж��൱��PWM-ON��λ��


   TIM_Cmd(TIM1,ENABLE);


/*
ʹ���ⲿʱ�ӵ�����
TIM_ITRxExternalClockConfig(TIM3, TIM_TS_ITR1);//�ⲿʱ��ģʽ1������ԴΪTIMER2
TIMx->SMCR=TIM_TS_ITR1
TIMx->SMCR=	��=TIM_SlaveMode_External1��

*/
   
}
/**********************************************************************
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void BLDC_TIM3Config(void)
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;			   //�����ṹ���������
   TIM_ICInitTypeDef  TIM_ICInitStructure;                      //����ṹ�����
   TIM_OCInitTypeDef  TIM_OCInitStructure;                     //����ṹ���������
      
   TIM_DeInit(TIM3);
  /*
	һȦ���ʱ��

	1/(72M/72)*65535=64ms
	�ǲ�����ζ��
   һ�Լ������Ϊ64*3=200ms,���Ի�һȦ400ms	��һ���� 60000/400=	 150Ȧ������ٶȣ�����һ����Ȧ�������ٶȡ�
  */
   TIM_TimeBaseStructure.TIM_Prescaler = 71;				   //Ԥ��Ƶ
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseStructure.TIM_Period =65535;	   			//
   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

   TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure); 
     
    //�������� 
   TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;            //ѡ��ͨ��1
   TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //���������ز���  
   TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC;  //����ͨ��Ϊ���룬��ӳ�䵽����
   TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;       //���벶��Ԥ��Ƶֵ
   TIM_ICInitStructure.TIM_ICFilter = 10;                      //�����˲�����������

   TIM_ICInit(TIM3, &TIM_ICInitStructure);                     //����ͨ������

   //ͨ��2���������TIM1��COM
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 		    //TIM���ͨ����ʼ��
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;             
   TIM_OCInitStructure.TIM_Pulse =1023;   		//TIMx->CCR2
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
  
   TIM_OC2Init(TIM3,&TIM_OCInitStructure);
   
   //Ӧ���� ���������Ƚ�/�����¼���
   //TIMx->CCMR2   =  TIM_OCMode_Timing << 8
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing; 		    //TIM���ͨ����ʼ��
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;             
   TIM_OCInitStructure.TIM_Pulse =65535; 
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
  
   TIM_OC4Init(TIM3,&TIM_OCInitStructure);
  /*
 ���� >CR2 �� TIM_CR2_TI1S��
 CH1��CH2��CH3���������ӵ�II1������
  */
   TIM_SelectHallSensor(TIM3,ENABLE);                          //ʹ��TIMx�Ļ����������ӿ�
 
 /*
    SMCR = 	 TIM_TS_TI1F_ED; TS���ó�TIFI_ED
 */  
   TIM_SelectInputTrigger(TIM3, TIM_TS_TI1F_ED);               //���봥��Դѡ��   
 
 /*
 SMCR |= TIM_SlaveMode_Reset;
 SMCR��SMS������Ϊ100.��λģʽ��ѡ�еĳ�������(TRGI�������������³�ʼ����������������һ�����¼Ĵ������ź�
 */ 
   TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);             //��λģʽ��һ�������������ź�����ʱ�򣬸�λ��

  /*
 SMCR BIT7 MSM ����Ϊ1
�������루TRGI)�ϵ��¼����ӳ��ˣ��������ڵ�ǰ��ʱ����ͨ��TRGO)�����ĴӶ�ʱ����������ͬ��
���Ҫ��Ѽ�����ʱ��ͬ����һ����һ���ⲿ�¼�ʱ�Ƿǳ����õġ�
 */ 
   TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);//����ģʽѡ��        
 
 /*
 CR2= 0x50 MMS=101 OC2REF�źű���Ϊ�������
 */  
   TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC2Ref);      //ѡ���������ģʽ(TRGO��)
// TIMx->DIER |= TIM_IT;
   TIM_ITConfig(TIM3, TIM_IT_Trigger|TIM_IT_CC4, ENABLE);      //����ʱ���ж� 
   //TIM_Cmd(TIM3,ENABLE);
}
/*
void timer4_cfg(void)
{
 TIM_TimeBaseInitType ;

 TIM_DeInit(TIM4);

 TIM_TimeBaseStructure.TIM_Period=2000;		 //ARR��ֵ
 TIM_TimeBaseStructure.TIM_Prescaler=71;
 TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //������Ƶ
 TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
// TIM_PrescalerConfig(TIM2,0x8C9F,TIM_PSCReloadMode_Immediate);//ʱ�ӷ�Ƶϵ��36000�����Զ�ʱ��ʱ��Ϊ2K
 TIM_ARRPreloadConfig(TIM4, DISABLE);//��ֹARRԤװ�ػ�����
 TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

// TIM_Cmd(TIM2, ENABLE);	//����ʱ��

}
*/

void Flaghuanxiang(void)
{
	zheng_fan=~zheng_fan;

}
void set_direction(FlagStatus dir)
{
	zheng_fan= dir;
}
void bdlc_set_pwm(uint16_t pwm)
{
	   TIM1->CCR1=pwm;	   //�����TIM1��PWM�Ĵ���
  	   TIM1->CCR2=pwm;
  	   TIM1->CCR3=pwm; 
	   TIM1->CCR4=pwm;
	   bldcPWM=pwm;
}
/**********************************************************************
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
/*
���� �Ƕ�  A	B	C |	 A	B	C  		
1 	   0   0    0   1 |  +   -	 		ab
2	   60  0	1	1 |  +   	-	    ac
3	   120 0	1	0 |	  	+	-	    bc
4	   180 1	1	0 |  -	+	 	    ba
5	   240 1	0	0 |	 -	 	+	  	ca
6	   300 1	0	1 |	  	-	+	    cb

A - PA6
B - PA7
C - PB0
���		����		IO
A��ɫ		C			PB0
B��ɫ		B			PA7
C��ɫ		A			PA6

CBA

����1
����:264513
462315

154

1 	   0   		0   0   1 |   ab	4	0x3180
2	   60  		0	1	1 |   ac	6	0x3108
3	   120 		0	1	0 |   bc	2	0x3018
4	   180 		1	1	0 |   ba	3	0x3810
5	   240 		1	0	0 |   ca	1	0x3801
6	   300 		1	0	1 |   cb	5	0x3081
��ת
1 	   0   		0   0   1 |   ba	4	0x3810
2	   60  		0	1	1 |   ca	6	0x3801
3	   120 		0	1	0 |   cb	2	0x3081
4	   180 		1	1	0 |   ab	3	0x3180
5	   240 		1	0	0 |   ac	1	0x3108
6	   300 		1	0	1 |   bc	5	0x3018
*/


			//			 		1     2      3       4     5      6
//uint16_t	zarycced[]={0x00,0x3018,0x3180,0x3108,0x3801,0x3810,0x3081,0x00};
uint16_t	zarycced[]={0x00,0x3801,0x3018,0x3810,0x3180,0x3081,0x3108,0x00};
uint16_t	fzarycced[]={0x00,0x3108,0x3081,0x3180,0x3810,0x3018,0x3801,0x00};
//u8			mBlPwmAry[]={0,4,2,2,1,4,1,0};

uint8_t	hwDIRary[]={4,6,2,3,1,5};

uint8_t	l_hw[3];

void dirCala(uint8_t hw)
{
	u8 i,k,j;
   	 //���㷽��
	  for(i=0;i<6;i++)
	  {
		   if(hwDIRary[i]==hw)
		   		break;
	  }
	  if(i<6)
	  {
	  	  k=i;
	  	  for(j=0;j<2;j++)
		  {
			  k++;
			  k=k>=6?0:k;
			  if(hwDIRary[k]!=l_hw[j])
			  	break;
		  }
		  if(j==2)	 //����
		  {
			 // t_printf("B");
			 bldc_dir=1;
		  }else
		  {
		  	  k=i;
		  	  for(j=0;j<2;j++)
			  {
			  	  if(k==0)
				  	k=5;
				   else
				  	k--;
				  
				  if(hwDIRary[k]!=l_hw[j])
				  	break;
			  }
			  if(j==2)	 //�Դ˷����Ƕ�Ϊ��ʱ��ȡ����
			  {
			  	bldc_dir=-1;
			  	/*
				  t_printf("T");
				  set_direction(RESET); //����
				  */
			  }

		  }
		  
	  }


		
		l_hw[2]=l_hw[1];
		l_hw[1]=l_hw[0];
		l_hw[0]=hw;
		
}

void huanxiang(void)
{
  u8 hwValue; 
#if HW_PRINTF
  static u8 lst=0;
#endif
  //hwValue = CBA
  hwValue=(u8)((GPIOA->IDR&0x000000c0)>>6);	//��ת��λ��
  hwValue|=(u8)((GPIOB->IDR&0x00000001)<<2);
 //  log_printf("%d",hwValue);
#if HW_PRINTF
 	if(lst!=hwValue)
	   write_hw(hwValue);
	lst = hwValue;
#endif
#if HW_DEBUG
//		recod_bldc(0,hwValue,0,0);
#endif
	  dirCala(hwValue);
	  if(zheng_fan==RESET)
	  	TIM1->CCER=zarycced[hwValue];
	  else								 //��ת
		TIM1->CCER=fzarycced[hwValue]; //����ת��λ�ã�����CCER�����λ��ת����ƫ����


}
/*
void check_run_ok(void)
{
  u8 hwValue,last,index=0; 
  int i;
  last=(u8)((GPIOA->IDR&0x000000c0)>>6);	//��ת��λ��
  last|=(u8)((GPIOB->IDR&0x00000001)<<2);
  index=last;
  for(i=0;i<80;i++)
  {
  	   Delayus(1000);
	  hwValue=(u8)((GPIOA->IDR&0x000000c0)>>6);	//��ת��λ��
	  hwValue|=(u8)((GPIOB->IDR&0x00000001)<<2);
	  log_printf("%d",hwValue);
	  if(last!=hwValue)
	  	return ;
	  last=hwValue;	 
   }
   log_printf("run error,%d\r\n",hwValue);
   index= hwValue+2;
   if(index==7)
   	index=1;
   TIM1->CCER=zarycced[index];
   log_printf("set to %d\r\n",index);

}
*/
/**************����******************/
void BLDC_Start(void)
{
   TIM1->SMCR|=0x0006;        //��TIM1�����봥��	
   TIM1->DIER=0x0040;         //��TIM1�Ĵ����ж�
   huanxiang();			      //���û�����������			  					 
   TIM3->CR1|=0x0001;		  //��TIM3
   TIM3->DIER|=0x0050;		  //��TIM3�ж�  
 //  Delayus(40);
//	check_run_ok();
}
void BLDC_Stop(void)
{
   TIM1->SMCR&=0xfff8;		  //�ر�TIM1�����봥��
   TIM1->CCER=0;              //�ر�TIM1����·���
   Delayus(40);			      //��ʱ����������
   TIM1->CCER=0x0ccc;         //����·�¹ܣ������ܺ��ƶ�
   while(stalling_count<9);    //�ȴ����ֹͣ��TIM3�������10�Σ�����Ϊ�����ͣת��   
   TIM1->CCER=0;              //�ر�TIM1����·�������ɲ��		  
   TIM3->CR1&=0xfffe;         //�ر�TIM3						  
   TIM3->CNT=0;               //��TIM3�ļ�����				   
   TIM3->DIER&=0xffaf;		  //��TIM3�ж�
}

/**************ֹͣ******************/
/*void stop(void)
{
   TIM1->SMCR&=0xfff8;		  //�ر�TIM1�����봥��
   TIM1->CCER=0;              //�ر�TIM1����·���
   Delay(20);			      //��ʱ����������
   TIM1->CCER=0x0ccc;         //����·�¹ܣ������ܺ��ƶ�
   while(duzhuan_value<1);    //�ȴ����ֹͣ��TIM3�������10�Σ�����Ϊ�����ͣת��   
   TIM1->CCER=0;              //�ر�TIM1����·�������ɲ��		  
   TIM3->CR1&=0xfffe;         //�ر�TIM3						  
   TIM3->CNT=0;               //��TIM3�ļ�����				   
   TIM3->DIER&=0xffaf;		  //��TIM3�ж�
   data_reset();     	      //��λ���в���
}*/
