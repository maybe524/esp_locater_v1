#include "common_esp32.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

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

/*
 *
 *
 *
output:
CAT1A£?eµA?©µcE?AU½A	GPIO7	A­???©µc£¬A­µI¶Iµc
CAT1A£?eµA??»u½A		GPIO10	°/OOCAT1A£?eµASPCA/¶¨OaA­??E±?o

EyOa/«????OO¶E?U½A		GPIO6
2?oO?E?µ?£¬			GPIO8 GPIO9	??IaOoA/AAµA/y¶¨

input:
³aµcO?E?½A			GPIO2	?½E±A­??£¬³aAuA­µI
µc³OµcA??i²a½A		GPIO3	?i²aµc³OµcN?

mode:
?I4GA£?eI¨N¶OA/®?UTX	GPIO4
?I4GA£?eI¨N¶OA/®?URX	GPIO5

I2CµASCL½A		GPIO18
I2CµASDA½A		GPIO19
 *
 */

//output
#define GPIO_OUTPUT_UART_CAT1_EN    7
#define GPIO_OUTPUT_UART_CAT1_POWER    10
#define GPIO_OUTPUT_LED_0    8
#define GPIO_OUTPUT_LED_1    9
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_UART_CAT1_EN) | (1ULL<<GPIO_OUTPUT_UART_CAT1_POWER) \
			|(1ULL<<GPIO_OUTPUT_LED_0) |(1ULL<<GPIO_OUTPUT_LED_1))
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_UART_CAT1_EN) | (1ULL<<GPIO_OUTPUT_UART_CAT1_POWER) \
			|(1ULL<<GPIO_OUTPUT_LED_0))

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
uint32_t get_channel(int channel_num);
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
    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_CHARGET_DETECT);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_CHARGET_DETECT, gpio_isr_handler, (void*) GPIO_INPUT_CHARGET_DETECT);

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
#endif
}

void gpio_init(void)
{
	gpio_set_output();
	gpio_set_input();

	gpio_set_intr();
}

void uart_4g_gpio_init()
{
	gpio_set_level(GPIO_OUTPUT_UART_CAT1_EN, 1);
	vTaskDelay(50 / portTICK_RATE_MS);

	gpio_set_level(GPIO_OUTPUT_UART_CAT1_POWER, 1);
	vTaskDelay(1000 / portTICK_RATE_MS);
	gpio_set_level(GPIO_OUTPUT_UART_CAT1_POWER, 0);
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
        //gpio_set_level(GPIO_OUTPUT_UART_CAT1_EN, cnt % 2);
        printf("GPIO[%d] time out, val: %d\n", GPIO_INPUT_CHARGET_DETECT, gpio_get_level(GPIO_INPUT_CHARGET_DETECT));

        printf("channel data is %x\r\n", get_channel(0));
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
	//gpio_init();
#if 0
	uint8_t data[2];
	//ESP_ERROR_CHECK(i2c_master_init());
	ESP_LOGI(TAG, "I2C initialized successfully");
	/* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
	//ESP_ERROR_CHECK(mpu9250_register_read(MPU9250_WHO_AM_I_REG_ADDR, data, 1));

	int cnt = 0;
	while(1)
	{
		printf("cnt: %d\n", cnt++);
		//printf("temp is %f\r\n", hardware_temp_get());
		//printf("channel data is %x\r\n", get_channel(0));


	    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

	    /* Demonstrate writing by reseting the MPU9250 */
	    //ESP_ERROR_CHECK(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
	ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
	ESP_LOGI(TAG, "I2C unitialized successfully");
#endif
	gpio_init();//gpio2 3 13 8 9
	uart_4g_init();//gpio15 10,uart 14 12
	uart_4g_main();
	//i2c_init();//gpio 18,19

//    //pthread_attr_t attr;
//    pthread_t thread1;// thread2;
//    //esp_pthread_cfg_t esp_pthread_cfg;
//    int res;
//
//    // Create a pthread with the default parameters
//    res = pthread_create(&thread1, NULL, gpio_test, NULL);
//    if(!res)
//    {
//        printf("Created thread 0x%x\n", thread1);
//    	printf("message init success\r\n");
//    }
//    else
//    {
//    	//assert(res == 0);
//    	printf("message init fail\r\n");
//    }

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
	return 0;
}


//²I??peripherals/wave_gen
int hardware_voice_record()
{
	return 0;
}
