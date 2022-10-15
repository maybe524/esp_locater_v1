#include "common_esp32.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
/*
 *
 *
 *
output:
CAT1ģ��Ĺ���ʹ�ܽ�	GPIO7	���߹��磬���Ͷϵ�
CAT1ģ��Ŀ�����		GPIO10	����CAT1ģ���SPC����������ʱ��

���ᴫ�����ն˹ܽ�		GPIO6
2��ָʾ�ƣ�			GPIO8 GPIO9	������ô���Ĵ���

input:
���ָʾ��			GPIO2	ƽʱ���ߣ���������
��ص�������		GPIO3	����ص�ѹ

mode:
��4Gģ��ͨѶ�ô���TX	GPIO4
��4Gģ��ͨѶ�ô���RX	GPIO5

I2C��SCL��		GPIO18
I2C��SDA��		GPIO19
 *
 */
//output
#define GPIO_OUTPUT_UART_CAT1_EN    7
#define GPIO_OUTPUT_UART_CAT1_POWER    10
#define GPIO_OUTPUT_LED_0    8
#define GPIO_OUTPUT_LED_1    9
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_UART_CAT1_EN) | (1ULL<<GPIO_OUTPUT_UART_CAT1_POWER) \
			|(1ULL<<GPIO_OUTPUT_LED_0) |(1ULL<<GPIO_OUTPUT_LED_1))

#define GPIO_OUTPUT_PIN_SEL1  ((1ULL<<GPIO_OUTPUT_LED_0) |(1ULL<<GPIO_OUTPUT_LED_1) | (1ULL<<GPIO_OUTPUT_UART_CAT1_POWER))
#define GPIO_OUTPUT_PIN_SEL2  ((1ULL<<GPIO_OUTPUT_UART_CAT1_EN))

//input
#define GPIO_INPUT_CHARGET_DETECT    2
#define GPIO_INPUT_BATTERY_DETECT    3

#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_CHARGET_DETECT) | (1ULL<<GPIO_INPUT_BATTERY_DETECT))

#define ESP_INTR_FLAG_DEFAULT 0

#define RX_TASK_TAG "rx_task"


#define port_tick_rate_ms    portTICK_RATE_MS
#define v_task_delay  vTaskDelay

static xQueueHandle gpio_evt_queue = NULL;
static char s_locater_uart_recv_buff[256] = {0};
static unsigned int s_locater_uart_recv_count = 0;
static pthread_mutex_t s_locater_uart_data_mutex;


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


static struct uart_event_que_s s_uart_event_que[UART_EVENT_QUE_DETH] = {0};
static unsigned int s_uart_event_que_busy_count = 0;
static unsigned int s_uart_event_que_head_idx = 0, s_uart_event_que_tail_idx = 0;
static pthread_mutex_t s_uart_event_mutex;

static struct uart_event_str_s s_uart_event_str_array[] = {
    {.p_event_str = "+QMTRECV:"},   ///< 服务器下发的消息
};

char *uart_get_recv_buff_head(void)
{
    return s_locater_uart_recv_buff;
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    printf("send the isr from gpio %d\r\n", gpio_num);
}

static void gpio_task(void* arg)
{
    uint32_t io_num = 0;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

void gpio_set_output()
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    printf("%s %d\r\n", __FUNCTION__, __LINE__);
}

void gpio_set_input()
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

void gpio_set_intr()
{
    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_CHARGET_DETECT, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task, "charger_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_CHARGET_DETECT, gpio_isr_handler, (void*) GPIO_INPUT_CHARGET_DETECT);
#if 0
    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_CHARGET_DETECT);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_CHARGET_DETECT, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
#endif
}

void uart_4g_gpio_init()
{
	gpio_set_level(GPIO_OUTPUT_UART_CAT1_EN, 1);
	v_task_delay(50 / port_tick_rate_ms);

	gpio_set_level(GPIO_OUTPUT_UART_CAT1_POWER, 1);
	v_task_delay(1000 / port_tick_rate_ms);
	gpio_set_level(GPIO_OUTPUT_UART_CAT1_POWER, 0);
}

void gpio_init(void)
{
	gpio_set_output();
    gpio_set_input();

    gpio_set_intr();
    printf("gpio init finish\r\n");
}

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_5)
#define RXD_PIN (GPIO_NUM_4)



int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

void uart_set_buff_clean(void)
{
    pthread_mutex_lock(&s_locater_uart_data_mutex);
    s_locater_uart_recv_count = 0;
    memset(s_locater_uart_recv_buff, 0, sizeof(s_locater_uart_recv_buff));
    pthread_mutex_unlock(&s_locater_uart_data_mutex);
}

unsigned int uart_get_recv_cnt(void)
{
    return s_locater_uart_recv_count;
}


struct uart_event_que_s *uart_event_get_item(void)
{
    struct uart_event_que_s *p_event = NULL;

    printf("uart_event_get_item, cout: %03d, head: %03d\n", s_uart_event_que_busy_count, s_uart_event_que_head_idx);
    if (!s_uart_event_que_busy_count)
        return -1;

    pthread_mutex_lock(&s_uart_event_mutex);
    // 出队
    s_uart_event_que[s_uart_event_que_head_idx].is_valid = false;
    p_event = &s_uart_event_que[s_uart_event_que_head_idx];

    // head指针往前
    s_uart_event_que_head_idx++;
    if (s_uart_event_que_head_idx >= UART_EVENT_QUE_DETH) {
        s_uart_event_que_head_idx = 0;
    }

    // 总个数减去1
    s_uart_event_que_busy_count--;
    pthread_mutex_unlock(&s_uart_event_mutex);

    return p_event;
}

int uart_event_put_item(char *p_event_str)
{
    if (!p_event_str)

    printf("uart_event_put_item, cout: %03d, tail: %03d, event: %s\n", \
        s_uart_event_que_busy_count, s_uart_event_que_tail_idx, p_event_str ? p_event_str : "nul");
    if (s_uart_event_que_busy_count >= UART_EVENT_QUE_DETH || !p_event_str)
        return -1;

    pthread_mutex_lock(&s_uart_event_mutex);
    // 进队
    strncpy(s_uart_event_que[s_uart_event_que_tail_idx].content, p_event_str, UART_EVENT_QUE_CONTEN_SIZE);
    s_uart_event_que[s_uart_event_que_tail_idx].is_valid = true;

    // tail指针往前移动
    s_uart_event_que_tail_idx++;
    if (s_uart_event_que_tail_idx >= UART_EVENT_QUE_DETH) {
        s_uart_event_que_tail_idx = 0;
    }

    // 总个数增加
    s_uart_event_que_busy_count++;
    pthread_mutex_unlock(&s_uart_event_mutex);

    return 0;
}

int uart_event_get_busy_item_count(void)
{
    return s_uart_event_que_busy_count;
}

static void uart_rx_task(void *arg)
{
    int i = 0, j = 0;
    int rx_bytes = 0, rx_final = 0;
    unsigned int remain = 0;
    unsigned char buff[512] = {0};
    char *p_match_one = NULL, *p_get_one = NULL;
    struct uart_event_que_s *p_event_que = NULL;
    char event_buff[UART_EVENT_QUE_CONTEN_SIZE];

    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    while (true) {
        rx_bytes = uart_read_bytes(UART_NUM_1, buff, sizeof(buff), 10 / port_tick_rate_ms);
        /*
        *  一直接收串口数据，在Buff缓冲大小之内一直接收。
        */
        if (rx_bytes > 0 && \
                s_locater_uart_recv_count < (sizeof(s_locater_uart_recv_buff) - 1))
        {
            pthread_mutex_lock(&s_locater_uart_data_mutex);
            remain = (sizeof(s_locater_uart_recv_buff) - 1) - s_locater_uart_recv_count;
            rx_final = rx_bytes > remain ? remain : rx_bytes;
            memcpy(s_locater_uart_recv_buff + s_locater_uart_recv_count, buff, rx_final);
            s_locater_uart_recv_count += rx_final;
            s_locater_uart_recv_buff[s_locater_uart_recv_count] = 0;

            /*
            *  在此检测4G模块的事件，例如服务器下发的事件等。
            *  如果是一个完整的句子，那么在处理，否则继续等待。
            */
            if (s_locater_uart_recv_count >= 2 && \
                    s_locater_uart_recv_buff[s_locater_uart_recv_count - 1] == '\n' && \
                    s_locater_uart_recv_buff[s_locater_uart_recv_count - 2] == '\r')
            {
                for (i = 0; i < ARRAY_LEN(s_uart_event_str_array); i++) {
                    p_match_one = strstr(s_locater_uart_recv_buff, s_uart_event_str_array[i].p_event_str);
                    if (!p_match_one)
                        continue;
                    printf("uart_rx_task, p_match_one: %s\n", p_match_one);
                    /*
                    *  把事件提取出来，并且把这个事件在s_locater_uart_recv_buff中抹茶干净，
                    *  即把s_locater_uart_recv_buff的字符串替换成XXX。
                    */
                    j = 0;
                    memset(event_buff, 0, sizeof(event_buff));
                    p_get_one = p_match_one;
                    while (true) {
                        if (*p_get_one == '\r' || *p_get_one == '\n' || j >= UART_EVENT_QUE_CONTEN_SIZE)
                            break;
                        event_buff[j] = *p_get_one;
                        p_get_one++;
                        j++;
                    }
                    uart_event_put_item(event_buff);
                }
            }

            pthread_mutex_unlock(&s_locater_uart_data_mutex);
        }

        v_task_delay(10 / portTICK_PERIOD_MS);
    }
}

void uart_4g_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // uart_4g_gpio_init();
    printf("uart init \r\n");
}


void uart_4g_main(void)
{
    int ret = 0;
    
    ret = pthread_mutex_init(&s_locater_uart_data_mutex, 0);
    ret = pthread_mutex_init(&s_uart_event_mutex, 0);
    printf("pthread_mutex_init init ret: %d\r\n", ret);

    xTaskCreate(uart_rx_task, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreate(locater_uart_misc_task, "locater_uart_misc_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}

void led_work()
{
	int cnt = 0;
	v_task_delay(1000 / port_tick_rate_ms);
    gpio_set_level(GPIO_OUTPUT_LED_0, cnt % 2);
    gpio_set_level(GPIO_OUTPUT_LED_1, cnt % 2);
}

void gpio_test()
{
#if 0
    int cnt = 0;

    while(1) {
        printf("cnt: %d\n", cnt++);
        v_task_delay(1000 / port_tick_rate_ms);
        gpio_set_level(GPIO_OUTPUT_LED_0, cnt % 2);
        gpio_set_level(GPIO_OUTPUT_LED_1, cnt % 2);
        //gpio_set_level(GPIO_OUTPUT_UART_CAT1_EN, cnt % 2);
        //printf("GPIO[%d] intr, val: %d\n", GPIO_OUTPUT_UART_CAT1_EN, gpio_get_level(GPIO_OUTPUT_UART_CAT1_EN));
    }
#endif
}

void hareware_main()
{
	gpio_init();//gpio2 3 13 8 9
	uart_4g_init();//gpio15 10,uart 14 12
	uart_4g_main();
	//i2c_init();//gpio 18,19

    pthread_attr_t attr;
    pthread_t thread1, thread2;
    esp_pthread_cfg_t esp_pthread_cfg;
    int res;

    // Create a pthread with the default parameters
    res = pthread_create(&thread1, NULL, gpio_test, NULL);
    if(!res)
    {
        printf("Created thread 0x%x\n", thread1);
    	printf("message init success\r\n");
    }
    else
    {
    	//assert(res == 0);
    	printf("message init fail\r\n");
    }

	//todo u
	printf("hardware init finish\r\n");
}
