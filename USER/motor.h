#ifndef _MOTOR_H_
#define _MOTOR_H_

#define  ENABLE_MOTOER()		GPIO_WriteBit(GPIOE, GPIO_Pin_2,   Bit_SET);

#define GO_FORWARD		1

#define LEFT_WHEEL	0x1
#define RIGHT_WHEEL	0x2

#define DIS_CH1()			(TIM4->CCER &=0xFFFC)
#define DIS_CH2()			(TIM4->CCER &=0xFFCF)
#define DIS_CH3()			(TIM4->CCER &=0xFCFF)
#define DIS_CH4()			(TIM4->CCER &=0xCFFF)

#define EN_CH1()			(TIM4->CCER|=3<<0)
#define EN_CH2()			(TIM4->CCER|=3<<4)
#define EN_CH3()			(TIM4->CCER|=3<<8)
#define EN_CH4()			(TIM4->CCER|=3<<12)

#define DIS_TM1_CH1()		(TIM1->CCER &=0xFFFC)
#define DIS_TM1_CH2()		(TIM1->CCER &=0xFFCF)
#define DIS_TM1_CH3()		(TIM1->CCER &=0xFCFF)
#define DIS_TM1_CH4()		(TIM1->CCER &=0xCFFF)

#define EN_TM1_CH1()		(TIM1->CCER|=3<<0)
#define EN_TM1_CH2()		(TIM1->CCER|=3<<4)
#define EN_TM1_CH3()		(TIM1->CCER|=3<<8)
#define EN_TM1_CH4()		(TIM1->CCER|=3<<12)

//¹öË¢µ÷ËÙ¿ØÖÆ
#define SET_MID_MOTER(PWM)	(TIM1->CCR4 = PWM)

extern __IO uint16_t *left_pwm,*right_pwm;

void timer1_init(void);
void motor_timer_init(void);
void r_motor_set_pwm(uint8_t HL,int pwm);
void motor_wheel_forward(u8 wheel,u16 speed);
void motor_wheel_backward(u8 wheel,u16 speed);
void motor_wheel_stop(u8 wheel);
void timer2_init(void);
#endif
