#ifndef _CFG_H
#define _CFG_H

struct cfg_t
{
	uint16_t	magic;
};

struct robot_msg_t
{
	uint8_t	 	msg;
};

struct sys_debug_t
{
	uint8_t		msg;	
};

struct h_cfg_t
{
	uint16_t magic;
};
extern struct cfg_t *cfg;
extern uint16_t NOMORL_PWM,SLOW_PWM;
extern char cfg_buf[];
extern struct sys_debug_t sys_debug;

void proc_rx_uart(uint8_t *buff,int len);
#endif

