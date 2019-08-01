#ifndef USART_H
#define USART_H

typedef char OUT[64];

extern int u1_len;
extern uint8_t g_buf[32];
extern int g_len;

void usartx_init(void);
void usart_write(uint8_t *buf,int len);
void log_printf(const char *format,...);
int read_uart1(uint8_t *str);
void proc_uart_task(void);
void uasrt_write(uint8_t *buff,int len);
#endif
