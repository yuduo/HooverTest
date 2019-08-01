#ifndef _KEY_H
#define _KEY_H

#define KEY_LEFT_1		0
#define KEY_RIGHT_1		1
#define KEY_LEFT_2		2
#define KEY_RIGHT_2		3

//×ó1¼ü
#define KEY_LEFT_1_PORT		GPIOA
#define KEY_LEFT_1_PIN		GPIO_Pin_15
//×ó2¼ü
#define KEY_LEFT_2_PORT		GPIOA
#define KEY_LEFT_2_PIN		GPIO_Pin_8
//ÓÒ1¼ü
#define KEY_RIGHT_1_PORT	GPIOA
#define KEY_RIGHT_1_PIN		GPIO_Pin_11
//ÓÒ2¼ü
#define KEY_RIGHT_2_PORT	GPIOA
#define KEY_RIGHT_2_PIN		GPIO_Pin_12

#define KEY_LEFT_1_RD()		(((KEY_LEFT_1_PORT->IDR) & KEY_LEFT_1_PIN) != KEY_LEFT_1_PIN)
#define KEY_RIGHT_1_RD()	(((KEY_RIGHT_1_PORT->IDR) & KEY_RIGHT_1_PIN) != KEY_RIGHT_1_PIN)
#define KEY_LEFT_2_RD()		(((KEY_LEFT_2_PORT->IDR) & KEY_LEFT_2_PIN) != KEY_LEFT_2_PIN)
#define KEY_RIGHT_2_RD()	(((KEY_RIGHT_2_PORT->IDR) & KEY_RIGHT_2_PIN) != KEY_RIGHT_2_PIN)

void key_io_init(void);
uint8_t proc_key_task(uint8_t sys_state);
#endif

