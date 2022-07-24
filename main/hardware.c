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

static void uart_set_buff_clean(void);
static unsigned int uart_get_recv_cnt(void);

#if 1
static xQueueHandle gpio_evt_queue = NULL;
static char s_locater_uart_recv_buff[2048] = {0};
static unsigned int s_locater_uart_recv_count = 0;
static pthread_mutex_t s_locater_uart_data_mutex;


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
#endif

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

#if 1
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

#define LOCATER_MAX_AT_RESP_LEN (215)
#define LOCATOR_STEP_ENTRY(s)   if (step == (s))
#define LOCATOR_DEBUG_MODE
#define ARRAY_LEN(a)    (sizeof(a) / sizeof(a[0]))

typedef struct locater_atres_fmt_s {
    char atreq_content[LOCATER_MAX_AT_RESP_LEN];
};

typedef struct locater_atres_csq_fmt_s {
    unsigned int rssi;
    unsigned int ber;
} locater_atres_csq_fmt_t;

typedef struct locater_str_split_fmt_s {
    char data[LOCATER_MAX_AT_RESP_LEN];
} locater_str_split_fmt_t;

typedef struct locater_atres_creg_fmt_s {
    unsigned int result_code;
    unsigned int status;
} locater_atres_creg_fmt_t;

typedef struct locater_atres_cgreg_fmt_s {

} locater_atres_cgreg_fmt_t;

typedef struct locater_atres_cpin_fmt_s {
    bool is_ready;
} locater_atres_cpin_fmt_t;

/*
*  //TBD: 经常发生栈溢出，临时把变量放在外边
*/
static char buff[1024];
static struct locater_atres_fmt_s s_locater_atres_array[32] = {0};
static struct locater_str_split_fmt_s s_locater_str_split_array[32] = {0};
static struct locater_atres_creg_fmt_s s_locater_atres_creg = {0};
static struct locater_atres_csq_fmt_s s_locater_csq = {0};
static struct locater_atres_cpin_fmt_s s_locater_atres_cpin = {0};

/**
 * @brief 截取字符串
 * 
 */
static int locater_uart_str_split_bychar(char *p_str, char *p_split_char_s, \
        struct locater_str_split_fmt_s *p_split_array, unsigned int split_array_size)
{
    char *p_head = p_str, *p_tail = p_str;
    char *p_split_char = p_split_char_s;
    bool is_found_one_split = false;
    struct locater_str_split_fmt_s *p_locater_str_split = p_split_array;
    unsigned int final_byte = 0, want_byte = 0;
    bool is_need_update_tail = false;
    bool is_need_split_exit = false;
    unsigned int split_size = 0;

    if (!p_str || !p_split_char_s || !p_split_array || !split_array_size)
        return -1;

    while (1) {
        if (split_size > split_array_size)
            break;
        else if (!(*p_head) || (*p_head == '\r') || (*p_head == '\n'))
            is_need_split_exit = true;
        
        /*
        *  检查是否是遇到切割的字符
        */
        is_found_one_split = false;
        p_split_char = p_split_char_s;
        while (1) {
            if (!(*p_split_char) || is_need_split_exit)
                break;
            if (*p_head == *p_split_char) {
                is_found_one_split = true;
                break;
            }
            p_split_char++;
        }

        /*
        *  截取字符串
        */
        is_need_update_tail = false;
        if (is_found_one_split || is_need_split_exit) {
            want_byte = p_head - p_tail;
            final_byte = want_byte > (LOCATER_MAX_AT_RESP_LEN - 1) ? (LOCATER_MAX_AT_RESP_LEN - 1) : want_byte;
            memcpy(p_locater_str_split->data, p_tail, final_byte);
            p_locater_str_split->data[final_byte] = 0;
            is_need_update_tail = true;
            p_locater_str_split++;
            split_size++;
        }

        p_head++;
        /*
        *  更新tail指针到下一个字节
        */
        if (is_need_split_exit)
            break;
        else if (is_need_update_tail) {
            p_tail = p_head;
        }
    }

    return split_size;
}

/**
 * @brief 通过字符串，以回车换行符为间隔获取每字符串
 * 
 * @param p_at_res_str 
 * @param p_at_rep 
 * @param rep_num 
 * @return int 
 */
static int locater_uart_process_atres(char *p_at_res_str, struct locater_atres_fmt_s *p_at_rep, unsigned int rep_num)
{
    char *p_head = p_at_res_str, *p_tail = NULL;
    struct locater_atres_fmt_s *p_at_rep_head = p_at_rep, *p_at_rep_iter = NULL;
    unsigned int idx = 0;
    unsigned int final_byte = 0, want_byte = 0;
    bool is_need_ignore_rn = false;

    if (!p_at_res_str || !p_at_rep || !rep_num)
        return -1;

#ifdef LOCATOR_DEBUG_MODE
    ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, p_at_res_str, strlen(p_at_res_str), ESP_LOG_INFO);
#endif

    /*
    *  去掉字符串首的回车换行符
    */
    while (1) {
        if (!(*p_head) || (*p_head != '\r' && *p_head != '\n'))
            break;
        p_head++;
    }
    p_tail = p_head;

    while (1) {
#ifdef LOCATOR_DEBUG_MODE
        printf("process_atres handle: 0x%02x\n", *p_head);
#endif
        if (!(*p_head) || idx >= rep_num)
            break;
        
        is_need_ignore_rn = false;
        if (*p_head == '\r' || *p_head == '\n') {
            want_byte = p_head - p_tail;
            final_byte = want_byte > (LOCATER_MAX_AT_RESP_LEN - 1) ? (LOCATER_MAX_AT_RESP_LEN - 1) : want_byte;
            /*
            *  去除\r字符等
            */
#ifdef LOCATOR_DEBUG_MODE
            printf("process_atres head: 0x%08x, p_tail: 0x%08x, final_byte: %d\n", \
                (unsigned int)p_at_res_str, (unsigned int)p_tail, final_byte);
#endif
            p_at_rep_iter = p_at_rep_head + idx;
            memcpy(p_at_rep_iter->atreq_content, p_tail, final_byte);
            p_at_rep_iter->atreq_content[final_byte] = 0;
            idx++;
            is_need_ignore_rn = true;
        }

        /*
        *  过滤掉后续的回车换行符，并且更新tail指针
        */
        if (is_need_ignore_rn) {
            while (1) {
                p_head++;
                if (!(*p_head) || (*p_head != '\r' && *p_head != '\n')) {
                    p_tail = p_head;
                    break;
                }
            }
        }
        else {
            p_head++;
        }
    }

    return idx;
}

/**
 * @brief 与4G模块的AT指令，并且获取返回的字符串
 * 
 * @param p_at_cmd_str 
 * @param p_result_buff 
 * @param result_buff_size 
 * @param timeout 
 * @return int 
 */
static int locater_uart_send_atcmd_2_4g_module(char *p_at_cmd_str, char *p_result_buff, \
        unsigned int result_buff_size, unsigned int timeout)
{
    bool is_recv_fail = false;
    unsigned int recv_byte = 0, final_copy_byte = 0;
    char *p_uart_buff_head = s_locater_uart_recv_buff;

    if (!p_at_cmd_str || !p_result_buff || \
            !result_buff_size || !timeout)
    {
        printf("atcmd_2_4g_module, check argv is invalid\n");
        return -1;
    }

    uart_set_buff_clean();
    uart_write_bytes(UART_NUM_1, p_at_cmd_str, strlen(p_at_cmd_str));
    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        recv_byte = uart_get_recv_cnt();
        timeout--;
        if (!timeout) {
            printf("atcmd_2_4g_module, wait atres timeout\n");
            is_recv_fail = true;
            break;
        }
        else if (!recv_byte)
            continue;
        else if ((recv_byte > 4 && \
                p_uart_buff_head[recv_byte - 4] == 'O'  && \
                p_uart_buff_head[recv_byte - 3] == 'K'  && \
                p_uart_buff_head[recv_byte - 2] == '\r' && \
                p_uart_buff_head[recv_byte - 1] == '\n') || \
            (recv_byte > 2 && 
                p_uart_buff_head[recv_byte - 2] == '\r' && \
                p_uart_buff_head[recv_byte - 1] == '\n'))
        {
            /*
            *  认为一个完整的AT回复是最后有一个OK的字符串，
            *  否则继续等待！
            */
            printf("atcmd_2_4g_module, wait atcmd done: %s\n", p_at_cmd_str);
            break;
        }
    }
    
    final_copy_byte = recv_byte > result_buff_size ? result_buff_size : recv_byte;
    memcpy(p_result_buff, p_uart_buff_head, final_copy_byte);
    printf("rx(%d):\n", recv_byte);
    printf("%s\n", p_uart_buff_head);
    printf("\n\n");

    return final_copy_byte;
}

/**
 * @brief 获取AT+CSQ的值
 * 
 * @param p_csq 
 * @return int 
 */
static int locater_uart_get_csq(struct locater_atres_csq_fmt_s *p_csq)
{
    int ret;
    int i = 0, j = 0;
    int at_res_line = 0;
    char *p_atcmd = NULL;
    int split_count = 0;

    if (!p_csq)
        return -1;

    p_atcmd = "AT+CSQ\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000);
    if (ret < 0) {
        printf("csq, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_atres(buff, s_locater_atres_array, 32);

    /*
    * Response
    * 1) +CSQ: <rssi>,<ber>
    *    OK
    * 2) ERROR
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("csq, line %02d, len: %02d, %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CSQ", s_locater_atres_array[i].atreq_content, 4))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_str_split_bychar(
            s_locater_atres_array[i].atreq_content, ",:", s_locater_str_split_array, ARRAY_LEN(s_locater_atres_array));
        for (j = 0; j < split_count; j++) {
            printf("csq, elem %02d: %s\n", j, s_locater_str_split_array[j].data);
        }
        if (split_count < 3) {
            printf("csq, split_count is less than 3\n");
            return -2;
        }

        p_csq->rssi = atoi(s_locater_str_split_array[1].data);
        p_csq->ber = atoi(s_locater_str_split_array[2].data);
        printf("csq, result, rssi: %d, ber: %d\n", p_csq->rssi, p_csq->ber);
        ret = 0;
        break;
    }

    return ret;
}

/**
 * @brief 获取AT+CREG的值
 * 
 * @param p_creg 
 * @return int 
 */
static int locater_uart_get_creg(struct locater_atres_creg_fmt_s *p_creg)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;

    if (!p_creg)
        return -1;

    p_atcmd = "AT+CREG?\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000);
    if (ret < 0) {
        printf("creg, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_atres(buff, s_locater_atres_array, 32);
    /*
    * Response
    *  1) +CREG: <n>,<stat>[,<lac>,<ci>] OK
    *  2) ERROR 
    *  3) +CME ERROR: <err>
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("creg, line %02d, len: %02d, %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CREG", s_locater_atres_array[i].atreq_content, 5))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_str_split_bychar(
            s_locater_atres_array[i].atreq_content, ",:", s_locater_str_split_array, ARRAY_LEN(s_locater_atres_array));
        for (j = 0; j < split_count; j++) {
            printf("creg, elem %02d: %s\n", j, s_locater_str_split_array[j].data);
        }
        if (split_count < 3) {
            printf("creg, split_count is less than 3\n");
            return -2;
        }

        p_creg->result_code = atoi(s_locater_str_split_array[1].data);
        p_creg->status = atoi(s_locater_str_split_array[2].data);
        printf("creg, result, result_code: %d, status: %d\n", p_creg->result_code, p_creg->status);
        ret = 0;
        break;
    }

    return ret;
}

static int locater_uart_get_cgreg(struct locater_atres_cgreg_fmt_s *p_creg)
{
    return 0;
}

/**
 * @brief 获取CPIN的值，查看SIM卡是否在位
 * 
 * @param p_cpin 
 * @return int 
 */
static int locater_uart_get_cpin(struct locater_atres_cpin_fmt_s *p_cpin)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;

    if (!p_cpin)
        return -1;

    p_atcmd = "AT+CPIN?\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000);
    if (ret < 0) {
        printf("cpin, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_atres(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  1) +CPIN: READY
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("cpin, line %02d, len: %02d, %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CPIN", s_locater_atres_array[i].atreq_content, 5))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_str_split_bychar(
            s_locater_atres_array[i].atreq_content, ",:", s_locater_str_split_array, ARRAY_LEN(s_locater_atres_array));
        for (j = 0; j < split_count; j++) {
            printf("creg, elem %02d: %s\n", j, s_locater_str_split_array[j].data);
        }
        if (split_count < 2) {
            printf("creg, split_count is error: %d\n", split_count);
            return -2;
        }

        p_cpin->is_ready = false;
        if (!strncmp(&s_locater_str_split_array[1].data[1], "READY", 5))
            p_cpin->is_ready = true;
        printf("creg, result, is_ready: %d\n", p_cpin->is_ready);
        ret = 0;
        break;
    }

    return ret;
}

static int locater_uart_set_mqtt_start(void)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;

    p_atcmd = "AT+CMQTTSTART\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000);
    if (ret < 0) {
        printf("mqtt_start, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_atres(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_start, line %02d, len: %02d, %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字OK为止
        */
        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 5))
            continue;
        
        printf("mqtt_start, result is ok\n");
        ret = 0;
        break;
    }

    return ret;
}

static int locater_uart_set_mqtt_stop(void)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;

    p_atcmd = "AT+CMQTTSTOP\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000);
    if (ret < 0) {
        printf("mqtt_stop, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_atres(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_stop, line %02d, len: %02d, %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字OK为止
        */
        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 5))
            continue;
        
        printf("mqtt_stop, result is ok\n");
        ret = 0;
        break;
    }

    return ret;
}


static int locater_uart_set_mqtt_accq(void)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;

    p_atcmd = "AT+CMQTTACCQ=0,\"client\"\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000);
    if (ret < 0) {
        printf("mqtt_accq, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_atres(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_accq, line %02d, len: %02d, %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字OK为止
        */
        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 5))
            continue;
        
        printf("mqtt_accq, result is ok\n");
        ret = 0;
        break;
    }

    return ret;
}

static void tx_task(void *arg)
{
    int ret = 0, idx = 0, i = 0;
    char *p_atcmd = NULL;
    static const char *TX_TASK_TAG = "TX_TASK";
    unsigned int timeout = 0;
    unsigned int step = 0;

    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

start_que:
    vTaskDelay(5000 / portTICK_PERIOD_MS);

LOCATOR_STEP_ENTRY(0) {
        printf("//////////////////%04d//////////////////\n", idx++);

        /*
        *  检查SIM在位情况
        */
        ret = locater_uart_get_cpin(&s_locater_atres_cpin);
        printf("detect sim ready: %d(%d)\n", s_locater_atres_cpin.is_ready, ret);
        if (ret < 0 || !s_locater_atres_cpin.is_ready) {
            printf("detect sim is not ready!\n");
            goto start_que;
        }

        /*
        *  检查信号质量
        */
        ret = locater_uart_get_csq(&s_locater_csq);
        printf("detect query signal quality, rssi: %d, ber: %d(%d)\n", s_locater_csq.rssi, s_locater_csq.ber, ret);
        if (ret) {
            printf("detect query signal quality fail!\n");
            goto start_que;
        }

        /*
        *  检查网络注册情况
        */
        ret = locater_uart_get_creg(&s_locater_atres_creg);
        printf("detect network registration\n");

        /*
        *  启动MQTT
        */
        ret = locater_uart_set_mqtt_start();
        printf("detect mqtt start result: %d\n", ret);
        if (ret) {
            ret = locater_uart_set_mqtt_stop();
            printf("detect mqtt stop result: %d\n", ret);
            goto start_que;
        }

        /*
        *  创建一个MQTT客户端
        */
        ret = locater_uart_set_mqtt_accq();
        printf("detect mqtt create client result: %d\n", ret);
        if (ret) {
            ret = locater_uart_set_mqtt_stop();
            printf("detect mqtt stop result: %d\n", ret);
            goto start_que;
        }

        goto start_que;
    }

LOCATOR_STEP_ENTRY(8) {
        ret = locater_uart_set_mqtt_stop();
        printf("mqtt stop result: %d\n", ret);
        if (ret) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        else {
            step = 0;
        }
    }

    printf("// check SIMCOMATI\n");
    p_atcmd = "AT+SIMCOMATI\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000);

    // Access to MQTT server not SSL/TLS
    // p_atcmd = "ATE0\r\n";
    // ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff));

    printf("// check CPIN\n");
    p_atcmd = "AT+CPIN?\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000);

    printf("// start MQTT service, activate PDP context\n");
    p_atcmd = "AT+CMQTTSTART\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 10000000);
    while (1) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    printf("// Acquire one client which will connect to a MQTT server not SSL/TLS\n");
    p_atcmd = "AT+CMQTTACCQ=0,\"client test0\"";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 100000);

    printf("// Set the will topic for the CONNECT message\n");
    p_atcmd = "AT+CMQTTWILLTOPIC=0,10";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 100000);

    printf("// Set the will message for the CONNECT message\n");
    p_atcmd = "AT+CMQTTWILLMSG=0,6,1";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 100000);

    printf("// Connect to a MQTT server\n");
    p_atcmd = "AT+CMQTTCONNECT=0,\"tcp://1.117.221.12:1883\",60,1,\"ABCABCABC\",\"ABCABCABC\"\r";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 100000);

    while (1) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void uart_set_buff_clean(void)
{
    pthread_mutex_lock(&s_locater_uart_data_mutex);
    s_locater_uart_recv_count = 0;
    pthread_mutex_unlock(&s_locater_uart_data_mutex);
}

static unsigned int uart_get_recv_cnt(void)
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
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
#endif

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
