#ifndef _TASK_RX_H_
#define _TASK_RX_H_

typedef struct h_ctrl_t
{
	uint8_t msg;
}ctrl_t;

typedef struct h_pid_set_t
{
	uint8_t 	msg;
}pid_set_t;
typedef struct h_rx_cfg_t
{
	uint8_t 	msg;
}rx_cfg_t;
void rx_usart(uint8_t chr);
void tx_msg(uint8_t *buff,int len);
void test_seft_task(uint8_t type);
#endif

