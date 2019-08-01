#include "sys.h"

/**********************************************************************
* Function Name  : key_io_init
* 按键及LED相关IO口的初始化
**********************************************************************/
void key_io_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = KEY_RIGHT_1_PIN |KEY_RIGHT_2_PIN|KEY_LEFT_1_PIN|KEY_LEFT_2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(KEY_RIGHT_1_PORT,&GPIO_InitStructure); 
}

static uint32_t n_key_read_tmr = 0;
static uint16_t n_key_pressed_ct[4] = {0xffff};
static uint8_t key_read(void)
{
	if(n_key_read_tmr != msTmr)
	{
		n_key_read_tmr = msTmr;
	}
	else
		return 0;

	if(KEY_LEFT_1_RD() == 1)//按键被按下
	{
		if(n_key_pressed_ct[KEY_LEFT_1] < 0xfff0)
		{
			n_key_pressed_ct[KEY_LEFT_1] ++;
		}
		else if(n_key_pressed_ct[KEY_LEFT_1] == 0xffff)n_key_pressed_ct[KEY_LEFT_1] = 0;
	}
	else
	{
		if(n_key_pressed_ct[KEY_LEFT_1] < 0xfff0 && n_key_pressed_ct[KEY_LEFT_1] > 5)
		{
			n_key_pressed_ct[KEY_LEFT_1] = 0xffff;
			return KEY_LEFT_1 + 1;
		}
		n_key_pressed_ct[KEY_LEFT_1] = 0xffff;
	}

	if(KEY_RIGHT_1_RD() == 1)//按键被按下
	{
		if(n_key_pressed_ct[KEY_RIGHT_1] < 0xfff0)
		{
			n_key_pressed_ct[KEY_RIGHT_1]++;

		}
		else if(n_key_pressed_ct[KEY_RIGHT_1] == 0xffff)n_key_pressed_ct[KEY_RIGHT_1] = 0;
	}
	else
	{
		if(n_key_pressed_ct[KEY_RIGHT_1] < 0xfff0 && n_key_pressed_ct[KEY_RIGHT_1] > 5)
		{
			n_key_pressed_ct[KEY_RIGHT_1] = 0xffff;
			return KEY_RIGHT_1 + 1;
		}
		n_key_pressed_ct[KEY_RIGHT_1] = 0xffff;
	}

	if(KEY_LEFT_2_RD() == 1)//按键被按下
	{
		if(n_key_pressed_ct[KEY_LEFT_2] < 0xfff0)
		{
			n_key_pressed_ct[KEY_LEFT_2] ++;
		}
		else if(n_key_pressed_ct[KEY_LEFT_2] == 0xffff)n_key_pressed_ct[KEY_LEFT_2] = 0;
	}
	else
	{
		if(n_key_pressed_ct[KEY_LEFT_2] < 0xfff0 && n_key_pressed_ct[KEY_LEFT_2] > 5)
		{
			n_key_pressed_ct[KEY_LEFT_2] = 0xffff;
			return KEY_LEFT_2 + 1;
		}
		n_key_pressed_ct[KEY_LEFT_2] = 0xffff;
	}

	if(KEY_RIGHT_2_RD() == 1)
	{
		if(n_key_pressed_ct[KEY_RIGHT_2] < 0xfff0)
		{
			n_key_pressed_ct[KEY_RIGHT_2] ++;

		}
		else if(n_key_pressed_ct[KEY_RIGHT_2] == 0xffff)n_key_pressed_ct[KEY_RIGHT_2] = 0;
	}
	else
	{
		if(n_key_pressed_ct[KEY_RIGHT_2] < 0xfff0 && n_key_pressed_ct[KEY_RIGHT_2] > 5)
		{
			n_key_pressed_ct[KEY_RIGHT_2] = 0xffff;
			return KEY_RIGHT_2 + 1;
		}
		n_key_pressed_ct[KEY_RIGHT_2] = 0xffff;
	}
	return 0;
}

uint8_t proc_key_task(uint8_t sys_state)
{
	uint8_t key_value;
	static unsigned char fan_level = 0;
	
	key_value = key_read();
	
	if(key_value--)
	{
		sSleepTimeMin = 0;
		switch(key_value)
		{
			case KEY_LEFT_1:
				if(5 < ++syst.midd_motor_level)
				{
					syst.midd_motor_level = 0;
					MOTOR_POWER_OFF();
				}
				else
				{
					MOTOR_POWER_ON();
				}

				SET_MID_MOTER(syst.midd_motor_level * 200);	//滚刷转速PWM 占空比 0% 20% 40% 60% 80% 100%
				break;
			case KEY_RIGHT_1:
				break;
			case KEY_LEFT_2:
				if(0 == syst.sState)
				{
					MOTOR_POWER_ON();
					if(1 == syst.test_go_forward_back)
					{
						syst.test_go_forward_back = 2;
					}
					else
					{
						syst.test_go_forward_back = 1;
					}
					syst.sState = 1;
				}
				else if(1 == syst.sState)
				{					
					syst.sState = 0;
					STOP_ALL_MOTOR();
				}
				break;
			case KEY_RIGHT_2:
				if(4 <= ++fan_level)
				{
					fan_level = 0;
				}

				if(1 == fan_level)
				{
					MOTOR_POWER_ON();
					delay_ms(4);
					DUST_MOTOR_RANK1();
				}
				else if(2 == fan_level)
				{
					DUST_MOTOR_RANK2();
				}
				else if(3 == fan_level)
				{
					DUST_MOTOR_RANK3();
				}
				else
				{
					MOTOR_POWER_OFF();
				}
				break;
			default:
				break;
		}
	}
	return key_value;
}

