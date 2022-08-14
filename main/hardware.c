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

static xQueueHandle gpio_evt_queue = NULL;
static char s_locater_uart_recv_buff[256] = {0};
static unsigned int s_locater_uart_recv_count = 0;
static pthread_mutex_t s_locater_uart_data_mutex;

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
	vTaskDelay(50 / portTICK_RATE_MS);

	gpio_set_level(GPIO_OUTPUT_UART_CAT1_POWER, 1);
	vTaskDelay(1000 / portTICK_RATE_MS);
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

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)



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

static void rx_task(void *arg)
{
    int rx_bytes = 0, rx_final = 0;
    unsigned int remain = 0;
    unsigned char buff[1024] = {0};

    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        rx_bytes = uart_read_bytes(UART_NUM_1, buff, sizeof(buff), 10 / portTICK_RATE_MS);
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
            pthread_mutex_unlock(&s_locater_uart_data_mutex);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
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

    uart_4g_gpio_init();
    printf("uart init \r\n");
}


void uart_4g_main(void)
{
    int ret = 0;
    
    ret = pthread_mutex_init(&s_locater_uart_data_mutex, 0);
    printf("pthread_mutex_init init ret: %d\r\n", ret);

    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreate(locater_uart_misc_task, "locater_uart_misc_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}

void led_work()
{
	int cnt = 0;
	vTaskDelay(1000 / portTICK_RATE_MS);
    gpio_set_level(GPIO_OUTPUT_LED_0, cnt % 2);
    gpio_set_level(GPIO_OUTPUT_LED_1, cnt % 2);
}

void gpio_test()
{
#if 0
    int cnt = 0;

    while(1) {
        printf("cnt: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_RATE_MS);
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
