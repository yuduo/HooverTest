#include "sys.h"

struct sys_t syst;

void test_go_line(void)
{
	TIM5->CNT = 0;
		
	delay_ms_sensers(5);

	if(1 == syst.test_go_forward_back)
	{
		motor_wheel_forward(LEFT_WHEEL, 600);
		motor_wheel_forward(RIGHT_WHEEL, 600);
	}
	else if(2 == syst.test_go_forward_back)
	{
		motor_wheel_backward(LEFT_WHEEL, 600);
		motor_wheel_backward(RIGHT_WHEEL, 600);
	}

	while(1)
	{
		get_sensers(&syst.gSta);
		proc_key_task(syst.sState);

		if(0 == syst.sState)
		{
			return;
		}

		if(TIM5->CNT >=5000)				
		{
			TIM5->CNT = 0;

			if(1 == syst.test_go_forward_back)
			{
				motor_wheel_forward(LEFT_WHEEL, 600);
				motor_wheel_forward(RIGHT_WHEEL, 600);
			}
			else if(2 == syst.test_go_forward_back)
			{
				motor_wheel_backward(LEFT_WHEEL, 600);
				motor_wheel_backward(RIGHT_WHEEL, 600);
			}
		}

		if(0 != (syst.gSta & 0x01))
		{
			STOP_ALL_MOTOR();
			return;
		}
	}
}


int main(void)
{
	disable_irq();
	Periph_Init();			//IO�ڵȳ�ʼ��
	//usartx_init();			//���ڳ�ʼ��

	SysTick_Init();
	motor_timer_init();		//���PWM��ʼ��

	timer1_init();			//��ˢ
	//timer2_init();		//��Դ������
	timer5_config();

	MOTOR_POWER_OFF();		//�ص����е����Դ
	enable_irq();
	
	syst.sState = 0;
	syst.midd_motor_level = 0;
	STOP_ALL_MOTOR();

	while(1)
	{
		proc_key_task(syst.sState);	
		if(1 == syst.sState)
		{
			test_go_line();
		}
	}
}

