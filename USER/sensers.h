#ifndef _SENSERS_H_
#define _SENSERS_H_

//回充传感器
#define IR_L_LINE			EXTI_Line15
#define IR_L_GPIO_PORT		GPIOB	
#define IR_L_GPIO_PIN		GPIO_Pin_15

#define IR_R_LINE			EXTI_Line8
#define IR_R_GPIO_PORT		GPIOC
#define IR_R_GPIO_PIN		GPIO_Pin_8

#define IR_M_LINE			EXTI_Line7
#define IR_M_GPIO_PORT		GPIOE	
#define IR_M_GPIO_PIN		GPIO_Pin_7

//碰撞开关传感器
#define INDEX_BUM_RIGHT2	6
#define INDEX_BUM_LEFT2		12
#define INDEX_BUM_RIGHT		9
#define INDEX_BUM_LEFT		8

#define MASK_BUM_LEFT		(1 << INDEX_BUM_LEFT)
#define MASK_BUM_LEFT2		(1 << INDEX_BUM_LEFT2)
#define MASK_BUM_RIGHT		(1 << INDEX_BUM_RIGHT)
#define MASK_BUM_RIGHT2		(1 << INDEX_BUM_RIGHT2)

void  senser_gpio_config(void);
char get_sensers(uint16_t *g_sta);
#endif
