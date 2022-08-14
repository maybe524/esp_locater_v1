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
#define LOCATOR_UART_STEP_ENTRY(s)   if (step == (s))
// #define LOCATOR_DEBUG_MODE
#define ARRAY_LEN(a)    (sizeof(a) / sizeof(a[0]))
#define LOCATER_IMEI_SIZE   (15)
#define locater_assert(cond)    do { \
                if (cond) { \
                    unsigned int timeout = 0; \
                    while (1) {  \
                        if (++timeout < 10000000) \
                            continue;  \
                        timeout = 0;    \
                        printf("assert, bug! %s %d\n", __func__, __LINE__); \
                    } \
                } \
            } while (0)

#define LOCATER_DEVICE_SERIAL_SIZE  (8)

typedef enum {
    LOCATER_SEND_AT_CMD_WITHOUT_CLEAN   = (1 << 0),
} locater_send_atcmd_2_4g_module_flags_e;

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

typedef struct locater_atres_cereg_fmt_s {
    unsigned int result_code;
    unsigned int status;
} locater_atres_cereg_fmt_t;

typedef struct locater_atres_cgreg_fmt_s {

} locater_atres_cgreg_fmt_t;

typedef struct locater_atres_cpin_fmt_s {
    bool is_ready;
} locater_atres_cpin_fmt_t;

typedef struct locater_atres_cgact_fmt_s {
    unsigned int cid;
    unsigned int status;
};

typedef struct locater_atres_cgatt_fmt_s {
    unsigned int status;
};

/**
 * @brief 4G模块上电OK后会回复PB Done
 * 
 */
typedef struct locater_atres_pbdone_fmt_s {
    bool is_power_on_done;
};

/**
 * @brief 获取IMEI号
 * 
 */
typedef struct locater_atres_cgsn_fmt_s {
    char imei_buff[LOCATER_IMEI_SIZE + 1];
    unsigned long long imei_64;
};

typedef struct locater_atres_cpsi_fmt_s {
    unsigned int system_mode;
    unsigned int operation_mode;
    unsigned int mcc;
    unsigned int mnc;
    unsigned int lac;
    unsigned int cell_id;
    unsigned int absolute_rf_ch_num;
    unsigned int rx_lev;
    unsigned int track_lo_adjust;
    unsigned int c1;
    unsigned int c2;
} locater_atres_cpsi_fmt_t;


/*
*  //TBD: 经常发生栈溢出，临时把变量放在外边
*/
static char buff[1024];
static struct locater_atres_fmt_s s_locater_atres_array[32] = {0};
static struct locater_str_split_fmt_s s_locater_str_split_array[32] = {0};
static struct locater_atres_creg_fmt_s s_locater_atres_creg = {0};
static struct locater_atres_csq_fmt_s s_locater_csq = {0};
static struct locater_atres_cpin_fmt_s s_locater_atres_cpin = {0};
static struct locater_atres_cgact_fmt_s s_locater_atres_cgact = {0};
static struct locater_atres_pbdone_fmt_s s_locater_atres_pbdone = {0};
static struct locater_atres_cgsn_fmt_s s_locater_atres_cgsn = {0};
static struct locater_atres_cgatt_fmt_s s_locater_atres_cgatt = {0};
static struct locater_atres_cereg_fmt_s s_locater_atres_cereg = {0};
static struct locater_atres_cpsi_fmt_s s_locater_atres_cpsi = {0};
static char s_locater_device_serial_buff[LOCATER_DEVICE_SERIAL_SIZE + 1] = {0};
static bool s_is_locater_online = false;
static unsigned int s_locater_temperature_threshold_high = 0, s_locater_temperature_threshold_low = 0;

static unsigned int locater_check_flags_32(unsigned int *p_flags, unsigned int mask)
{
    return (*p_flags) & mask;
}

static void locater_marks_flags_32(unsigned int *p_flags, unsigned int mask)
{
    (*p_flags) |= mask;
}

/**
 * @brief 截取字符串
 * 
 */
static int locater_uart_split_str_bychar(char *p_str, char *p_split_char_s, \
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
            /*
            *  去掉空格
            */
            while (true) {
                if (!(*p_head) || (*p_head != ' '))
                    break;
                p_head++;
            }
            p_tail = p_head;
        }
    }

    return split_size;
}

/**
 * @brief 检查字符串是否是10进制数值
 * 
 * @param p_dec_num_str 
 * @return int 
 */
static int locater_uart_check_is_dec_num_str(char *p_dec_num_str)
{
    char *p = p_dec_num_str;

    if (!p_dec_num_str)
        return -1;

    /*
    *  去掉字符串前面的回车换行符，和空格
    */
    while (1) {
        if (!(*p) || (*p != '\r' && *p != '\n' && *p != ' '))
            break;
        p++;
    }

    /*
    *  判断字符是否是数值，中间遇到空格则判定不合格。
    */
    while (1) {
        if (!(*p))
            break;
        else if ((*p) < '0' || (*p) > '9')
            return -2;
        else {
            p++;
        }
    }

    return 0;
}

// #define LOCATOR_DEBUG_MODE

/**
 * @brief 通过字符串，以回车换行符为间隔获取每字符串
 * 
 * @param p_at_res_str 
 * @param p_at_rep 
 * @param rep_num 
 * @return int 
 */
static int locater_uart_process_at_response(char *p_at_res_str, struct locater_atres_fmt_s *p_at_rep, unsigned int rep_num)
{
    char *p_head = p_at_res_str, *p_tail = NULL;
    struct locater_atres_fmt_s *p_at_rep_head = p_at_rep, *p_at_rep_iter = NULL;
    unsigned int idx = 0;
    unsigned int final_byte = 0, want_byte = 0;
    bool is_need_ignore_rn = false;
    bool is_detect_last_str = false;

    if (!p_at_res_str || !p_at_rep || !rep_num)
        return -1;

#ifdef LOCATOR_DEBUG_MODE
    ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, p_at_res_str, strlen(p_at_res_str), ESP_LOG_INFO);
#endif

    /*
    *  去掉字符串首的回车换行符
    */
    while (true) {
        if (!(*p_head) || (*p_head != '\r' && *p_head != '\n'))
            break;
        p_head++;
    }
    p_tail = p_head;

    while (true) {
#ifdef LOCATOR_DEBUG_MODE
        printf("process_atres handle: 0x%02x\n", *p_head);
#endif
        /*
        *  如果是最后一个字符串不是以回车换行符结束，那么需要处理
        */
        if (!(*p_head) && (p_head - p_tail))
            is_detect_last_str = true;
        else if (!(*p_head) || idx >= rep_num)
            break;
        
        is_need_ignore_rn = false;
        if ((*p_head == '\r' || *p_head == '\n') || is_detect_last_str) {
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
            memset(p_at_rep_iter, 0, sizeof(struct locater_atres_fmt_s));
            memcpy(p_at_rep_iter->atreq_content, p_tail, final_byte);
            p_at_rep_iter->atreq_content[final_byte] = 0;
            idx++;
            is_need_ignore_rn = true;
        }

        /*
        *  过滤掉后续的回车换行符，并且更新tail指针
        */
        if (is_detect_last_str)
            break;
        else if (is_need_ignore_rn) {
            while (true) {
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
 * @brief 去除回车换行符的打印
 * 
 * @param p 
 */
static void locater_uart_send_atcmd_2_4g_module_utils_print(char *p)
{
    bool is_need_printf_n = false;

    /*
    *  去掉字符串首的回车换行符
    */
    while (true) {
        if (!(*p) || (*p != '\r' && *p != '\n'))
            break;
        p++;
    }

    /*
    *  去掉多个回车换行符，只保留其中一个回车换行符，并且打印
    */
    while (true) {
        if (!(*p))
            break;
        else if ((*p) == '\r' || (*p) == '\n') {
            p++;
            is_need_printf_n = 1;
            continue;
        }
        if (is_need_printf_n) {
            is_need_printf_n = 0;
            printf("\n");
        }
        printf("%c", *p);
        p++;
    }
    printf("\n");
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
        unsigned int result_buff_size, unsigned int timeout, unsigned int flags)
{
    int i = 0;
    bool is_recv_fail = false;
    unsigned int recv_byte = 0, final_copy_byte = 0;
    char *p_uart_buff_head = s_locater_uart_recv_buff;
    unsigned int uart_buff_head_len = 0;
    unsigned int detect_unused_char = 0;

    if (!p_at_cmd_str || !p_result_buff || \
            !result_buff_size || !timeout)
    {
        printf("atcmd_2_4g_module, check argv is invalid\n");
        return -1;
    }

    /*
    *  清除uart buff里边的数据
    */
    if (!locater_check_flags_32(&flags, LOCATER_SEND_AT_CMD_WITHOUT_CLEAN)) {
        uart_set_buff_clean();
    }

    memset(p_result_buff, 0, result_buff_size);
    uart_write_bytes(UART_NUM_1, p_at_cmd_str, strlen(p_at_cmd_str));
    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        recv_byte = uart_get_recv_cnt();
        timeout--;
        if (!timeout) {
            printf("at_cmd_2_4g_module, wait at_response timeout!\n");
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
            printf("atcmd_2_4g_module, wait atcmd done\n");
            break;
        }
    }

    final_copy_byte = recv_byte > result_buff_size ? result_buff_size : recv_byte;
    memcpy(p_result_buff, p_uart_buff_head, final_copy_byte);
    uart_buff_head_len = strlen(p_uart_buff_head);
    printf("/----------------------\n");
    printf("tx %04d byte:\n", strlen(p_at_cmd_str));
    locater_uart_send_atcmd_2_4g_module_utils_print(p_at_cmd_str);
    printf("rx %04d byte:\n", recv_byte);
    locater_uart_send_atcmd_2_4g_module_utils_print(p_uart_buff_head);
    printf("----------------------/\n");
    printf("\n");

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
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("csq, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    * 1) +CSQ: <rssi>,<ber>
    *    OK
    * 2) ERROR
    */
    ret = -2;
    memset(p_csq, 0, sizeof(struct locater_atres_csq_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("csq, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CSQ", s_locater_atres_array[i].atreq_content, 4))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_split_str_bychar(
            s_locater_atres_array[i].atreq_content, ",:", s_locater_str_split_array, ARRAY_LEN(s_locater_atres_array));
        for (j = 0; j < split_count; j++) {
            printf("csq, elem_%02d: %s\n", j, s_locater_str_split_array[j].data);
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

static int locater_uart_get_cpsi(struct locater_atres_cpsi_fmt_s *p_cpsi)
{
    int ret;
    int i = 0, j = 0;
    int at_res_line = 0;
    char *p_atcmd = NULL;
    int split_count = 0;

    if (!p_cpsi)
        return -1;

    p_atcmd = "AT+CPSI?\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("get_cpsi, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    Response 
    1)If camping on a gsm cell: 
    +CPSI: <System Mode>,<Operation Mode>,<MCC>-<MNC>,<LAC>,<Cell ID>,<Absolute RF Ch Num>,<RxLev>,<Track LO Adjust>,<C1-C2>
    */
    ret = -2;
    memset(p_cpsi, 0, sizeof(struct locater_atres_csq_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("get_cpsi, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CPSI", s_locater_atres_array[i].atreq_content, 5))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_split_str_bychar(
            s_locater_atres_array[i].atreq_content, ",:", s_locater_str_split_array, ARRAY_LEN(s_locater_atres_array));
        for (j = 0; j < split_count; j++) {
            printf("get_cpsi, elem_%02d: %s\n", j, s_locater_str_split_array[j].data);
        }
        if (split_count < 9) {
            printf("get_cpsi, split_count is less than 9\n");
            return -2;
        }

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
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("creg, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);
    /*
    * Response
    *  1) +CREG: <n>,<stat>[,<lac>,<ci>] OK
    *  2) ERROR 
    *  3) +CME ERROR: <err>
    */
    ret = -2;
    memset(p_creg, 0, sizeof(struct locater_atres_creg_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("creg, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CREG", s_locater_atres_array[i].atreq_content, 5))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_split_str_bychar(
            s_locater_atres_array[i].atreq_content, ",:", s_locater_str_split_array, ARRAY_LEN(s_locater_atres_array));
        for (j = 0; j < split_count; j++) {
            printf("creg, elem_%02d: %s\n", j, s_locater_str_split_array[j].data);
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


/**
 * @brief 获取AT+CEREG的值
 * 
 * @param p_cereg 
 * @return int 
 */
static int locater_uart_get_cereg(struct locater_atres_cereg_fmt_s *p_cereg)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;

    if (!p_cereg)
        return -1;

    p_atcmd = "AT+CEREG?\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("get_cereg, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);
    /*
    * Response
    *  1) +CREG: <n>,<stat>[,<lac>,<ci>] OK
    *  2) ERROR 
    *  3) +CME ERROR: <err>
    */
    ret = -2;
    memset(p_cereg, 0, sizeof(struct locater_atres_cereg_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("get_cereg, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CEREG", s_locater_atres_array[i].atreq_content, 6))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_split_str_bychar(
            s_locater_atres_array[i].atreq_content, ",:", s_locater_str_split_array, ARRAY_LEN(s_locater_atres_array));
        for (j = 0; j < split_count; j++) {
            printf("get_cereg, elem_%02d: %s\n", j, s_locater_str_split_array[j].data);
        }
        if (split_count < 3) {
            printf("get_cereg, split_count is less than 3\n");
            return -2;
        }

        p_cereg->result_code = atoi(s_locater_str_split_array[1].data);
        p_cereg->status = atoi(s_locater_str_split_array[2].data);
        printf("get_cereg, result_code: %d, status: %d\n", p_cereg->result_code, p_cereg->status);
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
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("cpin, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  1) +CPIN: READY
    *  OK
    */
    ret = -2;
    memset(p_cpin, 0, sizeof(struct locater_atres_cpin_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("cpin, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CPIN", s_locater_atres_array[i].atreq_content, 5))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_split_str_bychar(
            s_locater_atres_array[i].atreq_content, ",:", s_locater_str_split_array, ARRAY_LEN(s_locater_atres_array));
        for (j = 0; j < split_count; j++) {
            printf("cpin, elem_%02d: %s\n", j, s_locater_str_split_array[j].data);
        }
        if (split_count < 2) {
            printf("cpin, split_count is error: %d\n", split_count);
            return -2;
        }

        p_cpin->is_ready = false;
        if (!strncmp(s_locater_str_split_array[1].data, "READY", 5))
            p_cpin->is_ready = true;
        printf("cpin, result, is_ready: %d\n", p_cpin->is_ready);
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
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 100000, 0);
    if (ret < 0) {
        printf("mqtt_start, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_start, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字OK为止
        */
        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 2))
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
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_stop, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_stop, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字OK为止
        */
        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 2))
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
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_accq, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_accq, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字OK为止
        */
        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 2))
            continue;
        
        printf("mqtt_accq, result is ok\n");
        ret = 0;
        break;
    }

    return ret;
}

/**
 * @brief 输入will topic
 * 
 * @param p_will_topic 
 * @return int 
 */
static int locater_uart_set_mqtt_will_topic(char *p_will_topic)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};

    p_atcmd = "AT+CMQTTWILLTOPIC=0,%d\r\n";
    sprintf(at_cmd_buf, p_atcmd, strlen(p_will_topic));
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_will_topic, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_will_topic, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关">"为止，要输入will topic的字符串
        */
        if (strncmp(">", s_locater_atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_will_topic, result is ok\n");
        ret = 0;
        break;
    }

    ret = locater_uart_send_atcmd_2_4g_module(p_will_topic, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_will_topic, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_will_topic, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关"OK"为止，要输入will topic的字符串
        */
        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 2))
            continue;

        printf("mqtt_will_topic, result is ok\n");
        ret = 0;
        break;
    }

    return ret;
}

static int locater_uart_set_cgdcont_cmnet(void)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};

    p_atcmd = "AT+CGDCONT=1,\"ip\",\"cmnet\"\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("cgdcont_cmnet, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("cgdcont_cmnet, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 2))
            continue;
        
        printf("cgdcont_cmnet, result is ok\n");
        ret = 0;
        break;
    }

    return ret;
}


static int locater_uart_set_cgact(void)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};

    p_atcmd = "AT+CGACT=1,1\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 500000, 0);
    if (ret < 0) {
        printf("cgact, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("cgact, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (!strncmp(s_locater_atres_array[i].atreq_content, "+CME ERROR", 10)) {
            printf("cgact, result is ok\n");
            ret = -3;
            break;
        }
        else if (strncmp(s_locater_atres_array[i].atreq_content, "OK", 2)) {
            printf("cgact, result is ok\n");
            ret = 0;
            break;
        }
    }

    return ret;
}


static int locater_uart_get_cgact(struct locater_atres_cgact_fmt_s *p_cgact)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};
    char *p_cid = NULL;
    char *p_status = NULL;

    p_atcmd = "AT+CGACT?\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("get_cgact, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    memset(p_cgact, 0, sizeof(struct locater_atres_cgact_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("get_cgact, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("+CGACT", s_locater_atres_array[i].atreq_content, 1))
            continue;
        split_count = locater_uart_split_str_bychar(
                s_locater_atres_array[i].atreq_content, ",:", s_locater_str_split_array, ARRAY_LEN(s_locater_atres_array));
        for (j = 0; j < split_count; j++) {
            printf("get_cgact, elem_%02d: %s\n", j, s_locater_str_split_array[j].data);
        }
        if (split_count < 2) {
            printf("get_cgact, split_count is error: %d\n", split_count);
            return -2;
        }

        /*
        *  检查是否是合理的十进制数值
        */
        p_cid = s_locater_str_split_array[1].data;
        p_status = s_locater_str_split_array[2].data;
        ret = locater_uart_check_is_dec_num_str(p_cid);
        if (ret) {
            printf("get_cgact, dec_num_str is error: %s\n", p_cid);
            return ret;
        }
        ret = locater_uart_check_is_dec_num_str(p_status);
        if (ret) {
            printf("get_cgact, dec_num_str is error: %s\n", p_status);
            return ret;
        }

        p_cgact->cid = atoi(p_cid);
        p_cgact->status = atoi(p_status);
        printf("get_cgact, result ok, cid: %d, status: %d\n", p_cgact->cid, p_cgact->status);
        ret = 0;
        break;
    }

    return ret;
}

/**
 * @brief AT+CGATT?
 * 
 * @param p_cgatt 
 * @return int 
 */
static int locater_uart_get_cgatt(struct locater_atres_cgatt_fmt_s *p_cgatt)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};
    char *p_cid = NULL;
    char *p_status = NULL;

    p_atcmd = "AT+CGATT?\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("get_cgatt, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    memset(p_cgatt, 0, sizeof(struct locater_atres_cgatt_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("get_cgatt, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("+CGATT", s_locater_atres_array[i].atreq_content, 1))
            continue;
        split_count = locater_uart_split_str_bychar(
                s_locater_atres_array[i].atreq_content, ",:", s_locater_str_split_array, ARRAY_LEN(s_locater_atres_array));
        for (j = 0; j < split_count; j++) {
            printf("get_cgatt, elem_%02d: %s\n", j, s_locater_str_split_array[j].data);
        }
        if (split_count < 2) {
            printf("get_cgatt, split_count is error: %d\n", split_count);
            return -2;
        }

        /*
        *  检查是否是合理的十进制数值
        */
        p_status = s_locater_str_split_array[1].data;
        ret = locater_uart_check_is_dec_num_str(p_cid);
        if (ret) {
            printf("get_cgatt, dec_num_str is error: %s\n", p_cid);
            return ret;
        }

        p_cgatt->status = atoi(p_status);
        printf("get_cgatt, result ok, status: %d", p_cgatt->status);
        ret = 0;
        break;
    }

    return ret;
}

static int locater_uart_set_netopen(void)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};
    unsigned int err = 0;

    p_atcmd = "AT+NETOPEN\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("netopen, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("netopen, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("+NETOPEN:", s_locater_atres_array[i].atreq_content, 8))
            continue;
        
        printf("netopen, result is ok\n");
        ret = 0;
        break;
    }

    split_count = locater_uart_split_str_bychar(
            s_locater_atres_array[i].atreq_content, ":", s_locater_str_split_array, ARRAY_LEN(s_locater_atres_array));
    for (j = 0; j < split_count; j++) {
        printf("netopen, elem_%02d: %s\n", j, s_locater_str_split_array[j].data);
    }
    if (split_count < 2) {
        printf("netopen, split_count is error: %d\n", split_count);
        return -2;
    }

    err = atoi(s_locater_str_split_array[i].data);
    printf("netopen, result, netopen_result: %d\n", err);
    ret = err ? -3 : 0;

    return ret;
}

static int locater_uart_set_mqtt_connect(void)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};
    unsigned int err = 0;
    unsigned int client = 0;
    unsigned int connect_err = 0;

    p_atcmd = "AT+CMQTTCONNECT=0,\"tcp://1.117.221.12:1883\",60,1,\"ABCABCABC\",\"ABCABCABC\"\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_connect, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_connect, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("+CMQTTCONNECT", s_locater_atres_array[i].atreq_content, 13))
            continue;
        
        printf("mqtt_connect, result is ok\n");
        ret = 0;
        break;
    }

    split_count = locater_uart_split_str_bychar(
            s_locater_atres_array[i].atreq_content, ":,", s_locater_str_split_array, ARRAY_LEN(s_locater_atres_array));
    for (j = 0; j < split_count; j++) {
        printf("mqtt_connect, elem_%02d: %s\n", j, s_locater_str_split_array[j].data);
    }
    if (split_count < 3) {
        printf("mqtt_connect, split_count is error: %d\n", split_count);
        return -2;
    }

    client = atoi(s_locater_str_split_array[1].data);
    connect_err = atoi(s_locater_str_split_array[2].data);
    printf("mqtt_connect, client: %d, connect_err: %d\n", client, connect_err);

    return ret;
}

static int locater_uart_get_pb_done(struct locater_atres_pbdone_fmt_s *p_pb_done)
{
    int ret;
    int i = 0, j = 0;
    int at_res_line = 0;
    char *p_atcmd = NULL;
    int split_count = 0;

    if (!p_pb_done)
        return -1;

    p_atcmd = "\r\n\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, LOCATER_SEND_AT_CMD_WITHOUT_CLEAN);
    if (ret < 0) {
        printf("pb_done, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    * PB DONE
    */
    ret = -2;
    p_pb_done->is_power_on_done = false;
    for (i = 0; i < at_res_line; i++) {
        printf("pb_done, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
#if 1
        if (strncmp("PB DONE", s_locater_atres_array[i].atreq_content, 7) && \
                strncmp("*ATREADY: 1", s_locater_atres_array[i].atreq_content, 11))
#else
        if (strncmp("PB DONE", s_locater_atres_array[i].atreq_content, 7))
#endif
            continue;

        printf("pb_done, pb is done\n");
        p_pb_done->is_power_on_done = true;
        ret = 0;
        break;
    }

    return ret;
}

int locater_uart_get_device_serial_by_imei(unsigned long long imei, char *p_serial_buff, unsigned int serial_buff_len)
{
    int serial_length = 9;
    char code_serial[] = {
        'G', 'i', 'E', 'N', 'P', 's', 'q', 'T', 'h', 'v',
        'u', 'X', 'e', 'p', 'S', 'z', 'g', 'j', 'n', 'y',
        'W', 'M', 'a', 'Q', 'k', 'L', 'b', 't', 'R', 'B',
        'Z', 'H', 'D', 'K', 'r', 'f', 'm', 'V', 'J', 'C',
        'U', 'd', 'F', 'A', 'c', 'w', 'x', 'Y'
    };
    unsigned long long  temp;
	unsigned int length = sizeof(code_serial) / sizeof(code_serial[0]);

	if (serial_buff_len < serial_length || \
            !imei || !p_serial_buff || !serial_buff_len)
		return -1;

    memset(p_serial_buff, 0, serial_buff_len);
    for (int i = 0; i < serial_length; i++) {
        if (imei < length) {
            p_serial_buff[i] = code_serial[(int) imei];
            break;
        }
		else {
            temp = imei;
            imei = imei / length;
            p_serial_buff[i] = code_serial[(int) (temp - imei * length)];
        }
    }

    // 序列号字符数组转换为字符串
    return 0;
}


static int locater_uart_get_imei_str(struct locater_atres_cgsn_fmt_s *p_cgsn)
{
    int ret;
    int i = 0, j = 0;
    int at_res_line = 0;
    char *p_atcmd = NULL;
    int split_count = 0;
    unsigned int imei_len = 0;
    unsigned long long imei_base = 100000000000000;

    if (!p_cgsn)
        return -1;

    p_atcmd = "AT+CGSN\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("imei_str, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  AT+CIMI
    *  460082215204124
    *  OK
    */
    ret = -2;
    p_cgsn->imei_64 = 0;
    memset(p_cgsn->imei_buff, 0, LOCATER_IMEI_SIZE + 1);
    for (i = 0; i < at_res_line; i++) {
        printf("imei_str, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        imei_len = 0;
        for (j = 0; j < LOCATER_IMEI_SIZE; j++) {
            if (s_locater_atres_array[i].atreq_content[j] && \
                (s_locater_atres_array[i].atreq_content[j] >= '0') && \
                (s_locater_atres_array[i].atreq_content[j] <= '9'))
            {
                imei_len++;
            }
        }

        if (imei_len != LOCATER_IMEI_SIZE)
            continue;

        strncpy(p_cgsn->imei_buff, s_locater_atres_array[i].atreq_content, LOCATER_IMEI_SIZE);
        for (j = 0; j < LOCATER_IMEI_SIZE; j++) {
            p_cgsn->imei_64 += (s_locater_atres_array[i].atreq_content[j] - '0') * imei_base;
            imei_base /= 10;
        }

        printf("imei_str, str : %s, val: %lld\n", p_cgsn->imei_buff, p_cgsn->imei_64);
        ret = 0;
        break;
    }

    return ret;
}


static int locater_uart_set_mqtt_subtopic(void)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};
    unsigned int err = 0;
    unsigned int client = 0;
    unsigned int connect_err = 0;
    unsigned int recv_cnt = 0;

    ///< 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    ///< AT+CMQTTSUBTOPIC=0,11,1		///< 参数依次含义：client_0, 11个字节，QoS=1
    ///< > AzRxBWxbZ/#	

    p_atcmd = "AT+CMQTTSUBTOPIC=0,11,1\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_subtopic, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_subtopic, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp(">", s_locater_atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_subtopic, wait input comunication ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_subtopic, result is error\n");
        return ret;
    }

    ///< 订阅主题
    sprintf(at_cmd_buf, "%s/#\r\n", s_locater_device_serial_buff);
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, buff, sizeof(buff), 1000, 0);
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_subtopic, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 13))
            continue;
        
        printf("mqtt_subtopic, input content result is ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_subtopic, result is fail\n");
        return ret;
    }

    printf("mqtt_subtopic, done\n");

    return ret;
}

/**
 * @brief AT+CMQTTSUB=0,14,1			///< 查询是否有这个主题
 * 
 * @return int 
 */
static int locater_uart_set_mqtt_sub(int client_handle, char *p_topic_str, unsigned int flags)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};
    unsigned int err = 0;
    unsigned int client = 0;
    unsigned int connect_err = 0;
    unsigned int recv_cnt = 0;

    ///< 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    ///< AT+CMQTTSUBTOPIC=0,11,1		///< 参数依次含义：client_0, 11个字节，QoS=1
    ///< > AzRxBWxbZ/#	

    if (client_handle < 0 || !p_topic_str) {
        printf("mqtt_sub, detect argv is error\n");
        return -1;
    }

    p_atcmd = "AT+CMQTTSUB=%d,%d,1\r\n";
    sprintf(at_cmd_buf, p_atcmd, client_handle, strlen(p_topic_str));
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_sub, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_sub, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp(">", s_locater_atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_sub, wait input comunication ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_sub, result is error\n");
        return ret;
    }

    ///< 订阅主题
    sprintf(at_cmd_buf, "%s\r\n", p_topic_str);
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, buff, sizeof(buff), 1000, 0);
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_sub, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 13))
            continue;
        
        printf("mqtt_sub, input content result is ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_sub, result is fail\n");
        return ret;
    }

    printf("mqtt_sub, done\n");

    return ret;
}


/**
 * @brief AT+CMQTTTOPIC=0,14			///< 指定发布主题
 * 
 * @return int 
 */
static int locater_uart_set_mqtt_topic(int client_handle, char *p_topic_str, unsigned int flags)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};
    unsigned int err = 0;
    unsigned int client = 0;
    unsigned int connect_err = 0;
    unsigned int recv_cnt = 0;

    ///< 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    ///< AT+CMQTTSUBTOPIC=0,11,1		///< 参数依次含义：client_0, 11个字节，QoS=1
    ///< > AzRxBWxbZ/#	

    p_atcmd = "AT+CMQTTSUB=%d,%d,1\r\n";
    sprintf(at_cmd_buf, p_atcmd, client_handle, strlen(p_topic_str));
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_topic, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_topic, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp(">", s_locater_atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_topic, wait input comunication ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_topic, result is error\n");
        return ret;
    }

    ///< 订阅主题
    sprintf(at_cmd_buf, "%s\r\n", p_topic_str);
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, buff, sizeof(buff), 1000, 0);
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_topic, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 13))
            continue;
        
        printf("mqtt_topic, input content result is ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_topic, result is fail\n");
        return ret;
    }

    printf("mqtt_topic, done\n");

    return ret;
}


/**
 * @brief AT+CMQTTPAYLOAD=0，1编辑payload。
 * 
 * @return int 
 */
static int locater_uart_set_mqtt_payload(int client_handle, char *p_payload_str, unsigned int flags)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};
    unsigned int err = 0;
    unsigned int client = 0;
    unsigned int connect_err = 0;
    unsigned int recv_cnt = 0;

    ///< 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    ///< AT+CMQTTSUBTOPIC=0,11,1		///< 参数依次含义：client_0, 11个字节，QoS=1
    ///< > AzRxBWxbZ/#	

    p_atcmd = "AT+CMQTTPAYLOAD=%d,%d\r\n";
    sprintf(at_cmd_buf, p_atcmd, client_handle, strlen(p_payload_str));
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_payload, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_payload, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp(">", s_locater_atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_payload, wait input comunication ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_payload, result is error\n");
        return ret;
    }

    ///< 要发布的数据字符串
    sprintf(at_cmd_buf, "%s\r\n", s_locater_device_serial_buff);
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, buff, sizeof(buff), 1000, 0);
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_payload, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 13))
            continue;
        
        printf("mqtt_payload, input content result is ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_payload, result is fail\n");
        return ret;
    }

    printf("mqtt_payload, done\n");

    return ret;
}


/**
 * @brief AT+CMQTTPUB=0,1,60,1，发布payload
 *      参数依次含义：client_0, QoS=1, pub_timeout=60, Retain=1
 * 
 * @param client_handle 
 * @param flags 
 * @return int 
 */
static int locater_uart_set_mqtt_pub(int client_handle, unsigned int flags)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};
    unsigned int err = 0;
    unsigned int client = 0;
    unsigned int connect_err = 0;
    unsigned int recv_cnt = 0;

    ///< 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    ///< AT+CMQTTSUBTOPIC=0,11,1		///< 参数依次含义：client_0, 11个字节，QoS=1
    ///< > AzRxBWxbZ/#	

    p_atcmd = "AT+CMQTTPUB=0,1,60,1\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_pub, failed\n");
        return ret;
    }

    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_pub, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 13))
            continue;
        
        printf("mqtt_pub, input content result is ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_pub, result is fail\n");
        return ret;
    }

    printf("mqtt_pub, done\n");

    return ret;
}

/**
 * @brief AT+CMQTTSUB=0，订阅设置好的主题的消息
 * 
 * @return int 
 */
static int locater_uart_set_mqtt_sub_confirm(void)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};
    unsigned int err = 0;
    unsigned int client = 0;
    unsigned int connect_err = 0;
    unsigned int recv_cnt = 0;

    ///< 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    ///< AT+CMQTTSUBTOPIC=0,11,1		///< 参数依次含义：client_0, 11个字节，QoS=1
    ///< > AzRxBWxbZ/#	

    p_atcmd = "AT+CMQTTSUB=0\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_sub_confirm, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_sub_confirm, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 2))
            continue;
        
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_sub_confirm, result is error\n");
        return ret;
    }

    printf("mqtt_sub_confirm, done\n");

    return ret;
}


static int locater_uart_set_mqtt_willtopic(void)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};
    unsigned int err = 0;
    unsigned int client = 0;
    unsigned int connect_err = 0;
    unsigned int recv_cnt = 0;

    ///< 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    ///< AT+CMQTTSUBTOPIC=0,11,1		///< 参数依次含义：client_0, 11个字节，QoS=1
    ///< > AzRxBWxbZ/#	

    p_atcmd = "AT+CMQTTWILLTOPIC=0,15\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_willtopic, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_willtopic, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp(">", s_locater_atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_willtopic, wait input comunication ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_willtopic, result is error\n");
        return ret;
    }

    ///< 订阅主题
    sprintf(at_cmd_buf, "%sD/0/0\r\n", s_locater_device_serial_buff);
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, buff, sizeof(buff), 1000, 0);
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_willtopic, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 13))
            continue;
        
        printf("mqtt_willtopic, input content result is ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_willtopic, result is fail\n");
        return ret;
    }

    printf("mqtt_willtopic, done\n");

    return ret;
}


static int locater_uart_set_mqtt_willmsg(void)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[64] = {0};
    unsigned int err = 0;
    unsigned int client = 0;
    unsigned int connect_err = 0;
    unsigned int recv_cnt = 0;

    ///< 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    ///< AT+CMQTTSUBTOPIC=0,11,1		///< 参数依次含义：client_0, 11个字节，QoS=1
    ///< > AzRxBWxbZ/#	

    p_atcmd = "AT+CMQTTWILLMSG=0,1,1\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_willmsg, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_willmsg, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp(">", s_locater_atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_willmsg, wait input comunication ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_willmsg, result is error\n");
        return ret;
    }

    ///< 订阅主题
    sprintf(at_cmd_buf, "0\r\n");
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, buff, sizeof(buff), 1000, 0);
    at_res_line = locater_uart_process_at_response(buff, s_locater_atres_array, 32);
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_willmsg, line_%02d, len_%02d: %s\n", i, strlen(s_locater_atres_array[i].atreq_content), \
            s_locater_atres_array[i].atreq_content);

        if (strncmp("OK", s_locater_atres_array[i].atreq_content, 13))
            continue;
        
        printf("mqtt_willmsg, input content result is ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_willmsg, result is fail\n");
        return ret;
    }

    printf("mqtt_willmsg, done\n");

    return ret;
}

static void locater_uart_device_online_conf_task(void *arg)
{
    int ret = 0, idx = 0, i = 0;
    char *p_atcmd = NULL;
    static const char *TX_TASK_TAG = "TX_TASK";
    unsigned int timeout = 0;
    unsigned int step = 0, step_bakup = 0;
    char *p_at_cmd_str = NULL;
    unsigned int retry_cnt = 0;
    unsigned int at_res_line = 0;
    int client_handle = 0;

    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

start_que:
    vTaskDelay(5000 / portTICK_PERIOD_MS);
LOCATOR_UART_STEP_ENTRY(0) {
        printf("\n\n");
        printf("//////////////////%04d//////////////////\n", idx++);

        s_is_locater_online = false;
        
        /*
        *  习惯性的输入回车换行
        */
        p_at_cmd_str = "\r\n\r\n";
        uart_write_bytes(UART_NUM_1, p_at_cmd_str, strlen(p_at_cmd_str));

        ret = locater_uart_get_pb_done(&s_locater_atres_pbdone);
        printf("pb_done, detect pb_done ready: %d (%d)\n", s_locater_atres_pbdone.is_power_on_done, ret);
        if (ret < 0 || !s_locater_atres_pbdone.is_power_on_done) {
            printf("pb_done, retry\n");
            goto start_que;
        }
        step_bakup = 0;
        step++;
    }

LOCATOR_UART_STEP_ENTRY(1) {
        /*
        *  检查SIM在位情况
        */
        printf("\r\n\r\n");
        ret = locater_uart_get_cpin(&s_locater_atres_cpin);
        printf("cpin, detect sim ready: %d (%d)\n", s_locater_atres_cpin.is_ready, ret);
        if (!s_locater_atres_cpin.is_ready) {
            printf("cpin, retry!\n");
            goto start_que;
        }

        /*
        *  检查信号质量
        */
        printf("\r\n\r\n");
        retry_cnt = 500;
        ret = locater_uart_get_csq(&s_locater_csq);
        printf("csq, detect query signal quality, rssi: %d, ber: %d (ret: %d)\n", s_locater_csq.rssi, s_locater_csq.ber, ret);
        if (ret || (s_locater_csq.rssi < 0) || (s_locater_csq.rssi > 31)) {
            printf("csq, retry!\n");
            goto start_que;
        }

        printf("\r\n\r\n");
        ret = locater_uart_get_cpsi(&s_locater_atres_cpsi);
        printf("get_cpsi, detect cpsi, (ret: %d)\n", ret);

        /*
        *  检查网络注册情况
        */
        printf("\r\n\r\n");
        ret = locater_uart_get_cereg(&s_locater_atres_cereg);
        printf("get_cereg, detect n: %d, status: %d\n", s_locater_atres_cereg.result_code, s_locater_atres_cereg.status);
        if (s_locater_atres_cereg.result_code == 0 && s_locater_atres_cereg.status == 1)
            printf("get_cereg, registered home network\n");
        else if (retry_cnt) {
            retry_cnt--;
            printf("get_cereg, retry %d!\n", retry_cnt);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            goto start_que;
        }
        else {
            printf("get_cereg, final fail after retry!\n", ret);
            goto start_que;
        }

        /*
        *  设置移动卡接入网络
        */
        printf("\r\n\r\n");
        ret = locater_uart_set_cgdcont_cmnet();
        printf("cgdcont_cmnet, detect cgdcont cmnet ret: %d\n", ret);
        if (ret) {
            printf("cgdcont_cmnet, retry\n", ret);
            goto start_que;
        }

        printf("\r\n\r\n");
        retry_cnt = 500;
        ret = locater_uart_get_cgact(&s_locater_atres_cgact);
        printf("get_cgact, detect get cgact ret: %d\n", ret);
        if ((ret && retry_cnt) || s_locater_atres_cgact.status != 1) {
            retry_cnt--;
            printf("get_cgact, retry: %d\n", retry_cnt);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            goto start_que;
        }
        else if (ret && !retry_cnt) {
            printf("get_cgact, final fail after retry!\n", ret);
            goto start_que;
        }

        /*
        *  CGACT激活网络。AT+CGACT之前先查询一下，返回为1才执行CGACT
        */
        printf("\r\n\r\n");
        ret = locater_uart_set_cgact();
        printf("cgact, detect set cgact ret: %d\n", ret);
        if (ret) {
            printf("set_cgact, retry!\n", ret);
            goto start_que;
        }

        if (step_bakup)
            step = step_bakup;
        else
            step++;
    }

LOCATOR_UART_STEP_ENTRY(2) {
        char mqtt_topic_str_buf[32] = {0};
        char mqtt_payload_str_buf[32] = {0};

        /*
        *  启动MQTT
        */
        printf("\r\n\r\n");
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
        printf("\r\n\r\n");
        ret = locater_uart_set_mqtt_accq();
        printf("mqtt_accq, detect mqtt create client result: %d\n", ret);
        if (ret) {
            ret = locater_uart_set_mqtt_stop();
            printf("mqtt_accq, detect mqtt stop result: %d\n", ret);
            goto start_que;
        }

        printf("\r\n\r\n");
        ret = locater_uart_get_imei_str(&s_locater_atres_cgsn);
        printf("imei_str, str result: %s\n", s_locater_atres_cgsn.imei_buff);
        locater_assert(ret);
        locater_uart_get_device_serial_by_imei(s_locater_atres_cgsn.imei_64, s_locater_device_serial_buff, sizeof(s_locater_device_serial_buff));
        printf("device_serial, str result: %s\n", s_locater_device_serial_buff);

        /*
        *  MQTT连接
        */
        printf("\r\n\r\n");
        ret = locater_uart_set_mqtt_connect();
        printf("mqtt_connect, detect mqtt connect result: %d\n", ret);
        if (ret) {
            ret = locater_uart_set_mqtt_stop();
            printf("mqtt_connect, retry mqtt connect\n");
            goto start_que;
        }

        ret = locater_uart_set_mqtt_subtopic();
        ret = locater_uart_set_mqtt_sub_confirm();

        ret = locater_uart_set_mqtt_willtopic();
        ret = locater_uart_set_mqtt_willmsg();

        /*
        *  上线通知服务器
        */
       sprintf(mqtt_topic_str_buf, "%sD/0/0", s_locater_device_serial_buff);
       ret = locater_uart_set_mqtt_sub(client_handle, mqtt_topic_str_buf, 0);

       ret = locater_uart_set_mqtt_topic(client_handle, mqtt_topic_str_buf, 0);

       sprintf(mqtt_payload_str_buf, "%s", "1");
       ret = locater_uart_set_mqtt_payload(client_handle, mqtt_payload_str_buf, 0);

       ret = locater_uart_set_mqtt_pub(client_handle, 0);
       s_is_locater_online = true;
       step++;
    }

LOCATOR_UART_STEP_ENTRY(3) {
        /*
        *  间隔几分钟检查一次网络
        */
        vTaskDelay(100000 / portTICK_PERIOD_MS);
        step = 1;
        step_bakup = 3;
        goto start_que;
    }
}

static unsigned int locater_uart_get_temperature(void)
{
    return 0;
}

static unsigned int locater_uart_get_collision(void)
{
    return 0;
}

static unsigned int locater_uart_get_battery_level(void)
{
    return 0;
}

static void locater_uart_misc_task(void *arg)
{
    int ret;
    unsigned int step = 0;
    unsigned int temperature = 0;
    char mqtt_topic_str_buf[32] = {0};
    char mqtt_payload_str_buf[32] = {0};
    int client_handle = 0;
    unsigned int collision_alarm = 0;
    unsigned int battery_level = 0;
    bool is_need_upload_service = false;

LOCATOR_UART_STEP_ENTRY(0) {
        if (s_is_locater_online)
            step++;
        else {
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
    }

    /*
    *  此步骤仅仅处理温度
    */
LOCATOR_UART_STEP_ENTRY(1) {
        is_need_upload_service = false;
        temperature = locater_uart_get_temperature();

        if (temperature > s_locater_temperature_threshold_high) {
            sprintf(mqtt_topic_str_buf, "%sD/%d/122", s_locater_device_serial_buff, 251);
            is_need_upload_service = true;
        }
        else if (temperature < s_locater_temperature_threshold_low) {
            sprintf(mqtt_topic_str_buf, "%sD/%d/222", s_locater_device_serial_buff, 251);
            is_need_upload_service = true;
        }

        if (is_need_upload_service) {
            ret = locater_uart_set_mqtt_sub(client_handle, mqtt_topic_str_buf, 0);
            ret = locater_uart_set_mqtt_topic(client_handle, mqtt_topic_str_buf, 0);
            sprintf(mqtt_payload_str_buf, "%s", "1");
            ret = locater_uart_set_mqtt_payload(client_handle, mqtt_payload_str_buf, 0);
            ret = locater_uart_set_mqtt_pub(client_handle, 0);
        }

        step++;
    }

    /*
    *  此步骤仅仅处理碰撞预警
    */
LOCATOR_UART_STEP_ENTRY(2) {
        collision_alarm = locater_uart_get_collision();
        if (collision_alarm) {
            sprintf(mqtt_topic_str_buf, "%sD/%d/42", s_locater_device_serial_buff, 188);
            ret = locater_uart_set_mqtt_sub(client_handle, mqtt_topic_str_buf, 0);
            ret = locater_uart_set_mqtt_topic(client_handle, mqtt_topic_str_buf, 0);
            sprintf(mqtt_payload_str_buf, "%s", "1");
            ret = locater_uart_set_mqtt_payload(client_handle, mqtt_payload_str_buf, 0);
            ret = locater_uart_set_mqtt_pub(client_handle, 0);
        }
        step++;
    }

    /*
    *  此步骤仅仅处理电池电量上报服务器
    */
LOCATOR_UART_STEP_ENTRY(3) {
        is_need_upload_service = false;
        battery_level = locater_uart_get_battery_level();
        if (battery_level < 10) {
            sprintf(mqtt_topic_str_buf, "%sD/%d/42", s_locater_device_serial_buff, 188);
            is_need_upload_service = true;
        }
    }
}

static void uart_set_buff_clean(void)
{
    pthread_mutex_lock(&s_locater_uart_data_mutex);
    s_locater_uart_recv_count = 0;
    memset(s_locater_uart_recv_buff, 0, sizeof(s_locater_uart_recv_buff));
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
    xTaskCreate(locater_uart_device_online_conf_task, "uart_tlocater_uart_device_online_conf_taskx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    // xTaskCreate(locater_uart_misc_task, "locater_uart_misc_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
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
