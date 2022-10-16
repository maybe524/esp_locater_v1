#ifndef ___HARDWARE___
#define ___HARDWARE___

#define ARRAY_LEN(array)    (sizeof(array) / sizeof(array[0]))
#define UART_EVENT_QUE_DETH 128
#define UART_EVENT_QUE_CONTEN_SIZE 256

typedef struct uart_event_que_s {
    bool is_valid;
    char content[UART_EVENT_QUE_CONTEN_SIZE];
} uart_event_que_t;

typedef struct uart_event_str_s {
    char *p_event_str;
} uart_event_str_t;

void uart_set_buff_clean(void);
unsigned int uart_get_recv_cnt(void);
char *uart_get_recv_buff_head(void);

int uart_event_put_item(char *p_event_str);
int uart_event_get_busy_item_count(void);
struct uart_event_que_s *uart_event_get_item(void);

#endif