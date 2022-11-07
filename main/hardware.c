#include "common_esp32.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "hardware.h"

//temp sensor
#include "driver/temp_sensor.h"

//gpio
#include <stdlib.h>
#include "freertos/task.h"
#include "freertos/queue.h"

//i2c
#include "driver/i2c.h"

//adc
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"

#define CONFIG_IDF_TARGET_ESP32C3 1

#define TIMES              256
#define GET_UNIT(x)        ((x>>3) & 0x1)

#if CONFIG_IDF_TARGET_ESP32
#define ADC_RESULT_BYTE     2
#define ADC_CONV_LIMIT_EN   1                       //For ESP32, this should always be set to 1
#define ADC_CONV_MODE       ADC_CONV_SINGLE_UNIT_1  //ESP32 only supports ADC1 DMA mode
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE1
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_RESULT_BYTE     2
#define ADC_CONV_LIMIT_EN   0
#define ADC_CONV_MODE       ADC_CONV_BOTH_UNIT
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE2
#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32H2
#define ADC_RESULT_BYTE     4
#define ADC_CONV_LIMIT_EN   0
#define ADC_CONV_MODE       ADC_CONV_ALTER_UNIT     //ESP32C3 only supports alter mode
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE2
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_RESULT_BYTE     4
#define ADC_CONV_LIMIT_EN   0
#define ADC_CONV_MODE       ADC_CONV_BOTH_UNIT
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE2
#endif

#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32H2
static uint16_t adc1_chan_mask = BIT(3);
static adc_channel_t channel[1] = {ADC1_CHANNEL_3};
#endif
#if CONFIG_IDF_TARGET_ESP32S2
static uint16_t adc1_chan_mask = BIT(2) | BIT(3);
static uint16_t adc2_chan_mask = BIT(0);
static adc_channel_t channel[3] = {ADC1_CHANNEL_2, ADC1_CHANNEL_3, (ADC2_CHANNEL_0 | 1 << 3)};
#endif
#if CONFIG_IDF_TARGET_ESP32
static uint16_t adc1_chan_mask = BIT(7);
static uint16_t adc2_chan_mask = 0;
static adc_channel_t channel[1] = {ADC1_CHANNEL_7};
#endif

//static const char *TAG = "ADC DMA";

/*
 *  i2c
*/
static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           18
#define I2C_MASTER_SDA_IO           19
//#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
//#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT                   7
#define GPIO_OUTPUT_UART_CAT1_EN    7
#define GPIO_OUTPUT_UART_CAT1_POWER    10
#define GPIO_OUTPUT_LED_0    15//Red
#define GPIO_OUTPUT_LED_1    14//Yellow
#define GPIO_OUTPUT_WAKEUP_4G    13//esp32 wake up 4g

#define GPIO_TEST0 0
#define GPIO_TEST1 1
#if 0
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_UART_CAT1_EN) | (1ULL<<GPIO_OUTPUT_UART_CAT1_POWER) \
			|(1ULL<<GPIO_OUTPUT_LED_0) |(1ULL<<GPIO_OUTPUT_LED_1))
#else
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_UART_CAT1_EN) | (1ULL<<GPIO_OUTPUT_UART_CAT1_POWER) \
		|(1ULL<<GPIO_OUTPUT_WAKEUP_4G) |(1ULL<<GPIO_TEST0) |(1ULL<<GPIO_TEST1))
#endif
#define GPIO_OUTPUT_PIN_SEL1  ((1ULL<<GPIO_OUTPUT_LED_0) |(1ULL<<GPIO_OUTPUT_LED_1) | (1ULL<<GPIO_OUTPUT_UART_CAT1_POWER))
#define GPIO_OUTPUT_PIN_SEL2  ((1ULL<<GPIO_OUTPUT_UART_CAT1_EN))

//input
#define GPIO_INPUT_CHARGET_DETECT    16
#define GPIO_INPUT_BATTERY_DETECT    3
#define GPIO_INPUT_SENSOR_DETECT1    6 //17
#define GPIO_INPUT_4G_WAKEUP_ESP32_DETECT    2 //17
//#define GPIO_INPUT_SENSOR_DETECT2    17
#if 0
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_CHARGET_DETECT) | (1ULL<<GPIO_INPUT_BATTERY_DETECT) \
		| (1ULL<<GPIO_INPUT_SENSOR_DETECT1) | (1ULL<<GPIO_INPUT_4G_WAKEUP_ESP32_DETECT))
#else
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_SENSOR_DETECT1) | (1ULL<<GPIO_INPUT_BATTERY_DETECT) \
		| (1ULL<<GPIO_INPUT_4G_WAKEUP_ESP32_DETECT))
#endif
#define ESP_INTR_FLAG_DEFAULT 0

#define RX_TASK_TAG "rx_task"

#define TXD_PIN (GPIO_NUM_5)
#define RXD_PIN (GPIO_NUM_4)


#define port_tick_rate_ms    portTICK_RATE_MS
#define v_task_delay  vTaskDelay

static xQueueHandle gpio_evt_queue = NULL;
static char s_locater_uart_recv_buff[1024] = {0};
static unsigned int s_locater_uart_recv_count = 0;
static pthread_mutex_t s_locater_uart_data_mutex;
static bool s_is_locater_uart_recv_ready = false;
static const int RX_BUF_SIZE = 1024;


static struct uart_event_que_s s_uart_event_que[UART_EVENT_QUE_DETH] = {0};
static unsigned int s_uart_event_que_busy_count = 0;
static unsigned int s_uart_event_que_head_idx = 0, s_uart_event_que_tail_idx = 0;
static pthread_mutex_t s_uart_event_mutex;
static unsigned int s_locater_uart_debug_mode = 0;

static struct uart_event_str_s s_uart_event_str_array[] = {
    {.p_event_str = "+QMTRECV:"},   ///< 服务器下发的消息
    {.p_event_str = "+QMTSTAT:"},   ///< 服务器下发的消息
    {.p_event_str = "+QMTPING:"},   ///< 服务器下发的消息

};

static unsigned int uart_check_flags_32(unsigned int *p_flags, unsigned int mask)
{
    return (*p_flags) & mask;
}

static void uart_marks_flags_32(unsigned int *p_flags, unsigned int mask)
{
    (*p_flags) |= mask;
}

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
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_CHARGET_DETECT, gpio_isr_handler, (void*) GPIO_INPUT_CHARGET_DETECT);

#if 0
    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_4G_WAKEUP_ESP32_DETECT, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue1 = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task1, "gpio_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_4G_WAKEUP_ESP32_DETECT, gpio_isr_handler, (void*) GPIO_INPUT_4G_WAKEUP_ESP32_DETECT);
#endif

#if 0
    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_CHARGET_DETECT);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_CHARGET_DETECT, gpio_isr_handler, (void*) GPIO_INPUT_CHARGET_DETECT);

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

    //gpio_set_intr();
    printf("gpio init finish\r\n");
}

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
    s_is_locater_uart_recv_ready = false;
    memset(s_locater_uart_recv_buff, 0, sizeof(s_locater_uart_recv_buff));
    pthread_mutex_unlock(&s_locater_uart_data_mutex);
}

unsigned int uart_get_recv_cnt(void)
{
    return s_locater_uart_recv_count;
}

int uart_chk_recv_buff_ready_condition(uart_chk_recv_buff_ready_user_condiction_fuc_t p_user_condiction, 
        void *p_argv, unsigned int flags)
{
    int ret;

    if (!s_is_locater_uart_recv_ready && \
            uart_check_flags_32(&flags, LOCATER_CHK_RECV_BUFF_FLAG_NO_MUST_READY))
    {
        return -1;
    }
    else if (!p_user_condiction) {
        return 0;
    }

    pthread_mutex_lock(&s_locater_uart_data_mutex);
    ret = p_user_condiction(s_locater_uart_recv_buff, s_locater_uart_recv_count, p_argv, flags);
    pthread_mutex_unlock(&s_locater_uart_data_mutex);

    return ret;
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

void uart_set_debug_mode(unsigned int debug_mode)
{
    s_locater_uart_debug_mode = debug_mode;
}

static void uart_rx_task(void *arg)
{
    int i = 0, j = 0;
    int rx_bytes = 0, rx_final = 0;
    unsigned int remain = 0;
    unsigned char buff[512] = {0};
    char *p_match_one = NULL, *p_get_one = NULL, *p_next_one = NULL, *p_prev_one = NULL, *p_first_one = NULL;
    struct uart_event_que_s *p_event_que = NULL;
    char event_buff[UART_EVENT_QUE_CONTEN_SIZE];
    unsigned int curr_event_len = 0, all_event_len = 0;
    unsigned int process_idx = 0;

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
            *  如果是一个消息，那么把消息提取出来。并且把消息移除掉。
            */
            printf("uart_rx_task, uart_recv_count: %04d (+%04d)\n", s_locater_uart_recv_count, rx_final);

            if (s_locater_uart_debug_mode) {
                printf("uart_rx_task, recv_buff:\n");
                printf("/*********************\n");
                printf("%s\n", s_locater_uart_recv_buff);
                printf("*********************/\n");
            }

            all_event_len = 0;
            curr_event_len = 0;
            process_idx = 0;
            p_first_one = NULL;
            p_get_one = s_locater_uart_recv_buff;

            if (s_locater_uart_recv_count >= 2 && \
                    s_locater_uart_recv_buff[s_locater_uart_recv_count - 1] == '\n' && \
                s_locater_uart_recv_buff[s_locater_uart_recv_count - 2] == '\r')
            {
                // 依次遍历所有的消息关键字
                s_is_locater_uart_recv_ready = false;
                for (i = 0; i < ARRAY_LEN(s_uart_event_str_array); i++) {
uart_rx_task_retry_get_one_event:
                    p_match_one = strstr(p_get_one, s_uart_event_str_array[i].p_event_str);
                    if (!p_match_one)
                        continue;
                    
                    // 记录第一个event的位置
                    if (!p_first_one)
                        p_first_one = p_match_one;

                    // 把事件提取出来
                    j = 0;
                    memset(event_buff, 0, sizeof(event_buff));
                    p_get_one = p_match_one;
                    while (true) {
                        // 考虑到后续还有一个字符\n
                        if (*p_get_one == '\r') {
                            p_get_one++;
                            continue;
                        }
                        // 找到一个完整的消息
                        else if (*p_get_one == '\n' || j >= UART_EVENT_QUE_CONTEN_SIZE) {
                            p_get_one++;
                            break;
                        }
                        event_buff[j] = *p_get_one++;
                        j++;
                    }

                    // 记录事件的总长度等，方便后续从接收buff中减去这些消息
                    curr_event_len = p_get_one - p_match_one;
                    all_event_len += curr_event_len;
                    printf("uart_rx_task, event_%04d curr_event_len: %d, all_event_len: %d\n", \
                        process_idx, curr_event_len, all_event_len);

#if 1 //def LOCATOR_DEBUG_MODE
                    printf("uart_rx_task, event_%04d dump start\n", process_idx);
                    ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, p_match_one, curr_event_len, ESP_LOG_INFO);
                    printf("uart_rx_task, event_%04d dump done!\n", process_idx);
#endif

                    // 事件入队
                    uart_event_put_item(event_buff);
                    process_idx++;

                    // 尝试在匹配一下当前类型的消息
                    goto uart_rx_task_retry_get_one_event;
                }

                // 如果接收的buff中存在消息，那么把消息给擦除掉
                if (p_first_one) {
                    // 把消息从buff中擦除
                    p_next_one = p_get_one;
                    p_prev_one = p_first_one;
                    while (true) {
                        if (!(*p_next_one))
                            break;
                        *p_prev_one++ = *p_next_one++;
                    }
                    *p_prev_one = '\0';
                    s_locater_uart_recv_count -= all_event_len;
                    printf("uart_rx_task, event_%04d, recv_count update: %d\n", \
                        process_idx, s_locater_uart_recv_count);
                }
            }
            pthread_mutex_unlock(&s_locater_uart_data_mutex);
            s_is_locater_uart_recv_ready = true;
        }

        v_task_delay(10 / port_tick_rate_ms);
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

    printf("uart init \r\n");
}


void uart_4g_main(void)
{
    int ret = 0;
    
    ret = pthread_mutex_init(&s_locater_uart_data_mutex, 0);
    ret = pthread_mutex_init(&s_uart_event_mutex, 0);
    printf("pthread_mutex_init init ret: %d\r\n", ret);

    xTaskCreate(uart_rx_task, "uart_rx_task", 1024*5, NULL, configMAX_PRIORITIES, NULL);
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

void temp_init()
{
	// Initialize touch pad peripheral, it will start a timer to run a filter
	//ESP_LOGI(TAG, "Initializing Temperature sensor");
	printf("Initializing Temperature sensor\r\n");
	temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
	temp_sensor_get_config(&temp_sensor);
	//ESP_LOGI(TAG, "default dac %d, clk_div %d", temp_sensor.dac_offset, temp_sensor.clk_div);
	printf("default dac %d, clk_div %d\r\n", temp_sensor.dac_offset, temp_sensor.clk_div);
	temp_sensor.dac_offset = TSENS_DAC_DEFAULT;
	temp_sensor_set_config(temp_sensor);
	temp_sensor_start();
	//ESP_LOGI(TAG, "Temperature sensor started");
	printf("Temperature sensor started\r\n");
}

float hardware_temp_get()
{
	float tsens_out;
	//while (1) {
		//vTaskDelay(1000 / portTICK_RATE_MS);
		temp_sensor_read_celsius(&tsens_out);
		//ESP_LOGI(TAG, "Temperature out celsius %f", tsens_out);
		printf("Temperature out celsius %f\r\n", tsens_out);
	//}
	return tsens_out;
}


static void continuous_adc_init(uint16_t adc1_chan_mask, uint16_t adc2_chan_mask, adc_channel_t *channel, uint8_t channel_num)
{
    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 1024,
        .conv_num_each_intr = TIMES,
        .adc1_chan_mask = adc1_chan_mask,
        .adc2_chan_mask = adc2_chan_mask,
    };
    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = ADC_CONV_LIMIT_EN,
        .conv_limit_num = 250,
        .sample_freq_hz = 10 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        uint8_t unit = GET_UNIT(channel[i]);
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_11;//ADC_ATTEN_DB_0;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
}

#if !CONFIG_IDF_TARGET_ESP32
static bool check_valid_data(const adc_digi_output_data_t *data)
{
    const unsigned int unit = data->type2.unit;
    if (unit > 2) return false;
    if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(unit)) return false;

    return true;
}
#endif

uint32_t get_channel(int channel_num)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[TIMES] = {0};
    memset(result, 0xcc, TIMES);

    //continuous_adc_init(adc1_chan_mask, adc2_chan_mask, channel, sizeof(channel) / sizeof(adc_channel_t));
    continuous_adc_init(adc1_chan_mask, 0, channel, sizeof(channel) / sizeof(adc_channel_t));
    adc_digi_start();

    ret = adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);
    adc_digi_output_data_t *p = (void*)&result[channel_num];

    adc_digi_stop();
    ret = adc_digi_deinitialize();
    assert(ret == ESP_OK);

    ESP_LOGI(TAG, "Unit: %d,_Channel: %d, Value: %x", p->type2.unit+1, p->type2.channel, p->type2.data);
    return p->type2.data;
}


/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void hareware_main()
{
	temp_init();
	gpio_init();//gpio2 3 13 8 9
	uart_4g_init();//gpio15 10,uart 14 12
	uart_4g_main();
	i2c_master_init();//gpio 18,19

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

int hardware_leds_ctl(int ledidx, int onoff)
{
	int gpio = ledidx;
    gpio_set_level(gpio, onoff);
    return 0;
}

int hardware_battery_get_cap()
{
	int slcap = 0;
	int slcap_adc = get_channel(0);
	if(slcap_adc <= 0xbab)
		slcap = 100;
	else if(slcap_adc > 0xbab)
		slcap = 99;
	else if(slcap_adc > 0xb67)
		slcap = 88;
	else if(slcap_adc > 0xb1c)
		slcap = 77;
	else if(slcap_adc > 0xad8)
		slcap = 66;
	else if(slcap_adc > 0xa48)
		slcap = 55;
	else if(slcap_adc > 0x9fc)
		slcap = 44;
	else if(slcap_adc > 0x9b8)
		slcap = 33;
	else if(slcap_adc > 0x96d)
		slcap = 22;
	else if(slcap_adc > 0x91e)
		slcap = 11;
	else
		slcap = 1;
	printf("channel data is %x, cap %d\r\n", slcap_adc, slcap);
	return slcap;
}


//参考peripherals/wave_gen
int hardware_voice_record()
{
	return 0;
}

int hardware_wakeup_4g_module()
{
	gpio_set_level(GPIO_OUTPUT_WAKEUP_4G, 0);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	gpio_set_level(GPIO_OUTPUT_WAKEUP_4G, 1);
	return 0;
}
