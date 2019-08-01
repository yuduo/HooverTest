#ifndef _ICLEAN_H
#define _ICLEAN_H

#define PRINTF		log_printf

extern unsigned char m_stack[4096];
extern unsigned char m_stack2[4];

struct sys_t
{
	unsigned char 	test_go_forward_back;
	unsigned char	sState;
	unsigned char	power_on;	
	unsigned short 	gSta;
	unsigned char	motor_power_off;
	unsigned char	midd_motor_level;
};

extern unsigned short sWorkTimeMinCt,sWorkTimeMin,sSleepTimeMin;
extern struct sys_t syst;

void init_sys(void);
#endif
