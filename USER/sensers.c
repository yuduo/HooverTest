#include "sys.h"

char get_sensers(uint16_t *g_sta)
{
	uint16_t i;

	//ǰ����ײ
	if(( (GPIOD->IDR) & MASK_BUM_LEFT)==0 )
	{
		for(i=0;i<2;i++)
		{
			if((GPIOD->IDR) & MASK_BUM_LEFT)
				break;
		}
		if(i>=2)
		{
			*g_sta |= 0x01;
		}
	}

	//ǰ����ײ
	if( ((GPIOD->IDR) & MASK_BUM_RIGHT) ==0 )
	{
		for(i=0;i<2;i++)
		{
			if((GPIOD->IDR) & MASK_BUM_RIGHT)
				break;
		}
		if(i>=2)
		{
			*g_sta |= 0x01;
		}
	}	

	//�����ײ
	if(((GPIOE->IDR) & MASK_BUM_LEFT2) ==0)
	{
		for(i=0;i<2;i++)
		{
			if((GPIOE->IDR) & MASK_BUM_LEFT2)
				break;
		}
		if(i>=2)
		{
			*g_sta |= 0x01;
		}
	}

	//�Ҳ���ײ
	if(((GPIOE->IDR) & MASK_BUM_RIGHT2) == 0)
	{
		for(i=0;i<2;i++)
		{
			if((GPIOE->IDR) & MASK_BUM_RIGHT2)
				break;
		}
		if(i>=2)
		{
			*g_sta |= 0x01;
		}
	}

	if(READ_VWALL_DET())
	{
		for(i=0;i<20;i++)
		{
			if(!READ_VWALL_DET())
				break;
		}
		if(i>=10)
		{
		}
	}
	return 1;
}


