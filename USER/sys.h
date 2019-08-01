#ifndef _SYS_H_
#define _SYS_H_

#include "stm32f10x.h"
#include "usart.h"
#include <stdarg.h>
#include <stdlib.h>
#include "stm32f10x_bkp.h"
#include "stm32f10x_pwr.h"

#include "pid.h"
#include "bsp.h"
#include "string.h"
#include "SysTick.h"
#include "iclean.h"
#include "ringbuffer.h"
#include "motor.h"
#include "cfg.h"
#include "sensers.h"
#include "task_rx.h"
#include "key.h"
#include "adc.h"
#include "math.h"
#include "key.h"

#include "stdio.h"

#define TRUE	0x01
#define FALSE	0x00

#define MAX(a,b) ((a) > (b) ? (a) : (b)) 
#define MIN(a,b) ((a) < (b) ? (a) : (b)) 

#define disable_irq()  {__ASM  volatile ("cpsid i");}
#define enable_irq()   {__ASM  volatile ("cpsie i");}
float disfloat(float x,float y);
void init_sys_sta(uint8_t sta);
uint32_t disxy_32(uint32_t x,uint32_t y);

#endif

