#ifndef _BSP_H
#define _BSP_H

//IN�?
#define PIN_AC		GPIO_Pin_0
#define PORT_AC		GPIOD

//��շ������
#define DUST_MOTOR_OFF()		{GPIO_ResetBits(GPIOC,GPIO_Pin_12);GPIO_ResetBits(GPIOC,GPIO_Pin_13);}
#define DUST_MOTOR_RANK2()		{GPIO_SetBits(GPIOC,GPIO_Pin_12);GPIO_ResetBits(GPIOC,GPIO_Pin_13);}
#define DUST_MOTOR_RANK1()		{GPIO_ResetBits(GPIOC,GPIO_Pin_12);GPIO_SetBits(GPIOC,GPIO_Pin_13);}
#define DUST_MOTOR_RANK3()		{GPIO_SetBits(GPIOC,GPIO_Pin_12);GPIO_SetBits(GPIOC,GPIO_Pin_13);}

//�����Դ����
#define PORT_MOTOR_POWER			GPIOE
#define PIN_MOTOR_POWER				GPIO_Pin_2

#define MOTOR_POWER_ON()			{GPIO_SetBits(PORT_MOTOR_POWER, PIN_MOTOR_POWER);}
#define MOTOR_POWER_OFF()			{GPIO_ResetBits(PORT_MOTOR_POWER, PIN_MOTOR_POWER);}	

#define STOP_ALL_MOTOR()			{/*MOTOR_POWER_OFF();*/motor_wheel_stop(LEFT_WHEEL);motor_wheel_stop(RIGHT_WHEEL);}	

/*
	�������ռ�⣨H/L��ƽ��⣩	����
	�Һ�����ռ�⣨H/L��ƽ��⣩	����
*/
#define PORT_LEFT_MOTOR_LEAVE				GPIOE
#define PIN_LEFT_MOTOR_LEAVE				GPIO_Pin_10

#define PORT_RIGHT_MOTOR_LEAVE				GPIOB
#define PIN_RIGHT_MOTOR_LEAVE				GPIO_Pin_12

//����ǽ,�͵�ƽΪ����ǽ
#define	PIN_VWALL_DET		GPIO_Pin_7
#define PORT_VWALL_DET		GPIOD
#define READ_VWALL_DET()	((PORT_VWALL_DET->IDR & PIN_VWALL_DET) == 0)

//���м��,�͵�ƽΪ���н���
#define PIN_DUSTBOX_DET		GPIO_Pin_5
#define PORT_DUSTBOX_DET	GPIOB
#define READ_DUSTBOX_DET()	((PORT_DUSTBOX_DET->IDR & PIN_DUSTBOX_DET) != 0)

//��Դ�����Ƿ�򿪵ļ��ܽ�
#define PIN_PSW			GPIO_Pin_10
#define PORT_PSW		GPIOD
#define READ_PSW_DET()	((PORT_PSW->IDR & PIN_PSW) == 0)

//ǰײ����Ŀ�����
#define PORT_IR_CTRL					GPIOC
#define PIN_IR_CTRL						GPIO_Pin_9

//������ռ��
#define LEFT_MOTOR_LEAVE()					GPIO_ReadInputDataBit(PORT_LEFT_MOTOR_LEAVE, PIN_LEFT_MOTOR_LEAVE) 
#define RIGHT_MOTOR_LEAVE()					GPIO_ReadInputDataBit(PORT_RIGHT_MOTOR_LEAVE, PIN_RIGHT_MOTOR_LEAVE) 

//IN�?
#define EXTERAL_AC_DETECT()	GPIO_ReadInputDataBit(PORT_AC, PIN_AC) 

void Periph_Init(void);
void Delayus(int t);
void TargetSysReset(void);
#endif

