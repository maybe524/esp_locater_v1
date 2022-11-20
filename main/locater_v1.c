#include "common_esp32.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "locater_v1.h"
#include "hardware.h"
#include "protocol.h"

#define GPIO_OUTPUT_UART_CAT1_EN    7
#define GPIO_OUTPUT_UART_CAT1_POWER    10

#define port_tick_rate_ms   portTICK_RATE_MS
#define port_tick_period_ms  portTICK_PERIOD_MS
#define v_task_delay  vTaskDelay
#define config_max_priorities configMAX_PRIORITIES
#define LOCATER_SERVER_TIME (1640995200)

/*
*  //TBD: 经常发生栈溢出，临时把变量放在外边
*/
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
static bool s_is_locater_device_ready = false;
static unsigned int s_locater_temperature_threshold_high = 0, s_locater_temperature_threshold_low = 0;
static unsigned int s_locater_uart_curr_app_idx = 0;
static bool s_is_locater_uart_curr_app_online = false;
static bool s_is_locater_uart_device_need_suspend = false;
static bool s_is_locater_uart_need_compare_distances = false;
static bool s_is_locater_uart_once_positioning = false;
static bool s_is_locater_uart_continuous_positioning = false;
static bool s_is_locater_uart_need_wifi_scan_rssi_by_mac_array = false;
static bool s_is_locater_uart_need_wifi_scan_rssi_by_mac = false;
static unsigned int s_is_locater_uart_need_wifi_scan_rssi_by_mac_app_idx = 0;
static pthread_mutex_t s_locater_uart_4g_module_mutex;
static char *p_serial = "QAZWSXQAZ";
static bool s_is_locater_uart_need_ota = false;
static bool s_is_locater_uart_ota_broadcast_stop_task = false;
static unsigned s_locater_uart_ota_stop_task_count = 0;
static char s_is_locater_uart_ota_wifi_mac[32] = {0};
static char s_is_locater_uart_ota_wifi_password[32] = {0};
static char s_is_locater_uart_ota_firmware_url[1024] = {0};
static char *sp_locater_firware_version = "0.0.1";
static struct locater_enclosure_conf_fmt_s s_locater_enclosure_conf = {0};
static struct locater_local_time_info_fmt_s s_ocater_local_time_info = {0};

static unsigned int locater_uart_get_temperature(void);
static unsigned int locater_uart_get_collision(void);
static unsigned int locater_uart_get_battery_level(void);


static unsigned int locater_check_flags_32(unsigned int *p_flags, unsigned int mask)
{
    return (*p_flags) & mask;
}

static void locater_marks_flags_32(unsigned int *p_flags, unsigned int mask)
{
    (*p_flags) |= mask;
}


#define CODE_BASE (217)
#define INT_LENGTH (4)

static char s_locater_86_code_char_array[CODE_BASE] = {
    33, 35, 36, 37, 38, 39, 40, 41, 42, 43, 45, 47, 48, 49, 50, 51, 52, 53, 54, 55,
    56, 57, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76,
    77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96,
    97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116,
    117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137,
    138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157,
    158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177,
    178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197,
    198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217,
    218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237,
    238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254
};

static int locater_code86_to_int(const char *p_code_str, int code_size)
{
    int my_int = 0;
    int position = 0;

    if (code_size != 1 && code_size != 2 && code_size != 4)
        return -1;

    for (int i = code_size; i > 0; i--) {
        for (int k = 0; k < CODE_BASE; k++) {
            if (p_code_str[i - 1] == s_locater_86_code_char_array[k]) {
                my_int = my_int * CODE_BASE + k;
                break;
            }
        }
    }

    return my_int;
}


/**
 * @brief 将数值（int、short、char）转换成code86.
 * 
 * @param my_int  [in ] 
 * @param int_type  [in ] 
 * @param str_buff  [in ] 
 * @param buff_len  [in ] 
 * @return char* 
 * 
 * @details 
 */
static char *locater_int_to_code86(int my_int, int int_type, char *str_buff, int buff_len)
{
    int the_rest = my_int;
    int position = 0;

    if (int_type != sizeof(char) && \
            int_type != sizeof(short) && \
        int_type != sizeof(int))
        return NULL;

    for (int i = 0; i < int_type; i++) {
        str_buff[position] = s_locater_86_code_char_array[the_rest % CODE_BASE];
        the_rest = the_rest / CODE_BASE;
        position++;
        if (position >= buff_len) {
            break;
        }
    }

    return the_rest ? NULL : str_buff;
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

    while (true) {
        if (split_size > split_array_size)
            break;
        else if (!(*p_head) || (*p_head == '\r') || (*p_head == '\n'))
            is_need_split_exit = true;
        
        /*
        *  检查是否是遇到切割的字符
        */
        is_found_one_split = false;
        p_split_char = p_split_char_s;
        while (true) {
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
            p_locater_str_split->data_len = final_byte;
            p_locater_str_split->data[final_byte] = 0;
            p_locater_str_split++;
            is_need_update_tail = true;
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
    while (true) {
        if (!(*p) || (*p != '\r' && *p != '\n' && *p != ' '))
            break;
        p++;
    }

    /*
    *  判断字符是否是数值，中间遇到空格则判定不合格。
    */
    while (true) {
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

    if (!p)
        return;

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

static int locater_uart_chk_recv_buff_ready_user_condiction(char *p_uart_recv_buff, \
        unsigned int uart_rev_buff_size, void *p_argv, unsigned int flags)
{
    int ret = 0, i = 0;
    char *p_strstr_found = NULL;
    struct locater_uart_chk_recv_buff_ready_misc_param_s *p_param = \
        (struct locater_uart_chk_recv_buff_ready_misc_param_s *)p_argv;
    unsigned int recv_byte = 0, final_copy_byte = 0;
    bool is_strstr_found = false;

    if (!uart_rev_buff_size || !p_param)
        return -1;
    else if (!p_param->p_strstr_array) {
        goto check_skip_strstr_entry;
    }

    // 匹配关键字
    for (i = 0; i < p_param->strstr_array_size; i++) {
        if (!p_param->p_strstr_array[i])
            continue;
        p_strstr_found = strstr(p_uart_recv_buff, p_param->p_strstr_array[i]);
        if (p_strstr_found) {
            is_strstr_found = true;
            break;
        }
    }
    if (!is_strstr_found && \
            !locater_check_flags_32(&flags, LOCATER_CHK_RECV_BUFF_FLAG_NO_MUST_READY))
    {
        return -2;
    }

check_skip_strstr_entry:
    final_copy_byte = uart_rev_buff_size > p_param->user_buff_size ? p_param->user_buff_size : uart_rev_buff_size;
    memcpy(p_param->p_user_buff, p_uart_recv_buff, final_copy_byte);
    if (p_param->p_final_copy_size)
        *p_param->p_final_copy_size = final_copy_byte;
    if (p_param->p_uart_buff_size) {
        *p_param->p_uart_buff_size = uart_rev_buff_size;
    }

    return 0;
}

/**
 * @brief 与4G模块的AT指令，并且获取返回的字符串
 * 
 * @param p_at_cmd_str 
 * @param p_want_result_str 
 * @param p_raw_result_buff 
 * @param raw_result_buff_size 
 * @param timeout 
 * @return int 
 */
static int locater_uart_send_atcmd_2_4g_module(char *p_at_cmd_str, \
        char *p_want_result_str, \
        char *p_raw_result_buff, unsigned int raw_result_buff_size, \
        unsigned int timeout, unsigned int flags)
{
    int ret;
    unsigned int all_recv_byte = 0, final_copy_byte = 0;
    struct locater_uart_chk_recv_buff_ready_misc_param_s misc_param = {0};
    static unsigned long dump_idx = 0;
    unsigned int chk_flags = 0;
    char *p_strstr_array[] = {p_want_result_str, "ERROR"};

    if ((!locater_check_flags_32(&flags, LOCATER_SEND_AT_CMD_DIRECT_WAIT_RESULT) && !p_at_cmd_str) || \
            !p_raw_result_buff || !raw_result_buff_size)
    {
        printf("atcmd_2_4g_module, check argv is invalid\n");
        return -1;
    }

    pthread_mutex_lock(&s_locater_uart_4g_module_mutex);
    /*
    *  清除uart buff里边的数据
    *  即标志了LOCATER_SEND_AT_CMD_WITHOUT_CLEAN就不清除
    */
    if (!locater_check_flags_32(&flags, LOCATER_SEND_AT_CMD_WITHOUT_CLEAN)) {
        uart_set_buff_clean();
    }

    timeout = !timeout ? 1000 : timeout;
    memset(p_raw_result_buff, 0, raw_result_buff_size);

    /*
    *  直接等待结果不需要发送AT指令
    *  即标志了LOCATER_SEND_AT_CMD_DIRECT_WAIT_RESULT就不发送AT指令
    */
    if (!locater_check_flags_32(&flags, LOCATER_SEND_AT_CMD_DIRECT_WAIT_RESULT)) {
        uart_write_bytes(UART_NUM_1, p_at_cmd_str, strlen(p_at_cmd_str));
    }

    chk_flags = 0;
    misc_param.p_strstr_array = p_strstr_array;
    misc_param.strstr_array_size = ARRAY_LEN(p_strstr_array);
    misc_param.p_user_buff = p_raw_result_buff;
    misc_param.user_buff_size = raw_result_buff_size;
    misc_param.p_final_copy_size = &final_copy_byte;
    misc_param.p_uart_buff_size = &all_recv_byte;

    while (true) {
        v_task_delay(10 / port_tick_period_ms);
        ret = uart_chk_recv_buff_ready_condition(locater_uart_chk_recv_buff_ready_user_condiction, \
            (void *)&misc_param, chk_flags);
        if (!ret) {
            printf("at_cmd_2_4g_module, wait at_response %s!\n", timeout ? "done" : "timeout");
            break;
        }
        else if (!timeout) {
            chk_flags = LOCATER_CHK_RECV_BUFF_FLAG_NO_MUST_READY;
            continue;
        }
        timeout--;
    }

    printf("/----------------------\n");
    printf("idx: %04d\n", dump_idx++);
    printf("tx %04d byte:\n", p_at_cmd_str ? strlen(p_at_cmd_str) : 0);
    locater_uart_send_atcmd_2_4g_module_utils_print(p_at_cmd_str);
    printf("rx %04d byte:\n", all_recv_byte);
    locater_uart_send_atcmd_2_4g_module_utils_print(p_raw_result_buff);
    printf("----------------------/\n");

    pthread_mutex_unlock(&s_locater_uart_4g_module_mutex);

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
    char *p_want_at_respone_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    if (!p_csq)
        return -1;

    p_atcmd = "AT+CSQ\r\n";
    p_want_at_respone_str = "+CSQ:";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_at_respone_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("csq, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, ARRAY_LEN(atres_array));

    /*
    * Response
    * 1) +CSQ: <rssi>,<ber>
    *    OK
    * 2) ERROR
    */
    ret = -2;
    memset(p_csq, 0, sizeof(struct locater_atres_csq_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("csq, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CSQ", atres_array[i].atreq_content, 4))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_split_str_bychar(
            atres_array[i].atreq_content, ",:", split_array, ARRAY_LEN(atres_array));
        for (j = 0; j < split_count; j++) {
            printf("csq, elem_%02d: %s\n", j, split_array[j].data);
        }
        if (split_count < 3) {
            printf("csq, split_count is less than 3\n");
            return -2;
        }

        p_csq->rssi = atoi(split_array[1].data);
        p_csq->ber = atoi(split_array[2].data);
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
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    if (!p_cpsi)
        return -1;

    p_atcmd = "AT+CPSI?\r\n";
    p_want_ack_str = "+CPSI:";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("get_cpsi, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, ARRAY_LEN(atres_array));

    /*
    Response 
    1)If camping on a gsm cell: 
    +CPSI: <System Mode>,<Operation Mode>,<MCC>-<MNC>,<LAC>,<Cell ID>,<Absolute RF Ch Num>,<RxLev>,<Track LO Adjust>,<C1-C2>
    */
    ret = -2;
    memset(p_cpsi, 0, sizeof(struct locater_atres_csq_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("get_cpsi, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CPSI", atres_array[i].atreq_content, 5))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_split_str_bychar(
            atres_array[i].atreq_content, ",:", split_array, ARRAY_LEN(atres_array));
        for (j = 0; j < split_count; j++) {
            printf("get_cpsi, elem_%02d: %s\n", j, split_array[j].data);
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    if (!p_creg)
        return -1;

    p_atcmd = "AT+CREG?\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("creg, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, ARRAY_LEN(atres_array));
    /*
    * Response
    *  1) +CREG: <n>,<stat>[,<lac>,<ci>] OK
    *  2) ERROR 
    *  3) +CME ERROR: <err>
    */
    ret = -2;
    memset(p_creg, 0, sizeof(struct locater_atres_creg_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("creg, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CREG", atres_array[i].atreq_content, 5))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_split_str_bychar(
            atres_array[i].atreq_content, ",:", split_array, ARRAY_LEN(atres_array));
        for (j = 0; j < split_count; j++) {
            printf("creg, elem_%02d: %s\n", j, split_array[j].data);
        }
        if (split_count < 3) {
            printf("creg, split_count is less than 3\n");
            return -2;
        }

        p_creg->result_code = atoi(split_array[1].data);
        p_creg->status = atoi(split_array[2].data);
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
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    if (!p_cereg)
        return -1;

    p_atcmd = "AT+CEREG?\r\n";
    p_want_ack_str = "+CEREG:";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("get_cereg, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, ARRAY_LEN(atres_array));
    /*
    * Response
    *  1) +CREG: <n>,<stat>[,<lac>,<ci>] OK
    *  2) ERROR 
    *  3) +CME ERROR: <err>
    */
    ret = -2;
    memset(p_cereg, 0, sizeof(struct locater_atres_cereg_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("get_cereg, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CEREG", atres_array[i].atreq_content, 6))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_split_str_bychar(
            atres_array[i].atreq_content, ",:", split_array, ARRAY_LEN(atres_array));
        for (j = 0; j < split_count; j++) {
            printf("get_cereg, elem_%02d: %s\n", j, split_array[j].data);
        }
        if (split_count < 3) {
            printf("get_cereg, split_count is less than 3\n");
            return -2;
        }

        p_cereg->result_code = atoi(split_array[1].data);
        p_cereg->status = atoi(split_array[2].data);
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
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    if (!p_cpin)
        return -1;

    p_atcmd = "AT+CPIN?\r\n";
    p_want_ack_str = "+CPIN:";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("cpin, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, ARRAY_LEN(atres_array));

    /*
    * Response
    *  1) +CPIN: READY
    *  OK
    */
    ret = -2;
    memset(p_cpin, 0, sizeof(struct locater_atres_cpin_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("cpin, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        if (strncmp("+CPIN", atres_array[i].atreq_content, 5))
            continue;
        
        /*
        *  切割字符串，理论上有3个元素，否则判定失败
        */
        split_count = locater_uart_split_str_bychar(
            atres_array[i].atreq_content, ",:", split_array, ARRAY_LEN(atres_array));
        for (j = 0; j < split_count; j++) {
            printf("cpin, elem_%02d: %s\n", j, split_array[j].data);
        }
        if (split_count < 2) {
            printf("cpin, split_count is error: %d\n", split_count);
            return -2;
        }

        p_cpin->is_ready = false;
        if (!strncmp(split_array[1].data, "READY", 5))
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    p_atcmd = "AT+CMQTTSTART\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, NULL, buff, sizeof(buff), 100000, 0);
    if (ret < 0) {
        printf("mqtt_start, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_start, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        /*
        *  找到关键字OK为止
        */
        if (strncmp("OK", atres_array[i].atreq_content, 2))
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    p_atcmd = "AT+CMQTTSTOP\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_stop, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_stop, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        /*
        *  找到关键字OK为止
        */
        if (strncmp("OK", atres_array[i].atreq_content, 2))
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    p_atcmd = "AT+CMQTTACCQ=0,\"client\"\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_accq, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_accq, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        /*
        *  找到关键字OK为止
        */
        if (strncmp("OK", atres_array[i].atreq_content, 2))
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    p_atcmd = "AT+CMQTTWILLTOPIC=0,%d\r\n";
    sprintf(at_cmd_buf, p_atcmd, strlen(p_will_topic));
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_will_topic, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_will_topic, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        /*
        *  找到关">"为止，要输入will topic的字符串
        */
        if (strncmp(">", atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_will_topic, result is ok\n");
        ret = 0;
        break;
    }

    ret = locater_uart_send_atcmd_2_4g_module(p_will_topic, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_will_topic, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_will_topic, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        /*
        *  找到关"OK"为止，要输入will topic的字符串
        */
        if (strncmp("OK", atres_array[i].atreq_content, 2))
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
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    p_atcmd = "AT+CGDCONT=1,\"ip\",\"cmnet\"\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("cgdcont_cmnet, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("cgdcont_cmnet, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp("OK", atres_array[i].atreq_content, 2))
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
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    p_atcmd = "AT+CGACT=1,1\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("cgact, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("cgact, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (!strncmp(atres_array[i].atreq_content, "+CME ERROR", 10)) {
            printf("cgact, result is ok\n");
            ret = -3;
            break;
        }
        else if (strncmp(atres_array[i].atreq_content, "OK", 2)) {
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
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    p_atcmd = "AT+CGACT?\r\n";
    p_want_ack_str = "+CGACT:";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("get_cgact, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    memset(p_cgact, 0, sizeof(struct locater_atres_cgact_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("get_cgact, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp("+CGACT", atres_array[i].atreq_content, 1))
            continue;
        split_count = locater_uart_split_str_bychar(
                atres_array[i].atreq_content, ",:", split_array, ARRAY_LEN(atres_array));
        for (j = 0; j < split_count; j++) {
            printf("get_cgact, elem_%02d: %s\n", j, split_array[j].data);
        }
        if (split_count < 2) {
            printf("get_cgact, split_count is error: %d\n", split_count);
            return -2;
        }

        /*
        *  检查是否是合理的十进制数值
        */
        p_cid = split_array[1].data;
        p_status = split_array[2].data;
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    p_atcmd = "AT+CGATT?\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("get_cgatt, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    memset(p_cgatt, 0, sizeof(struct locater_atres_cgatt_fmt_s));
    for (i = 0; i < at_res_line; i++) {
        printf("get_cgatt, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp("+CGATT", atres_array[i].atreq_content, 1))
            continue;
        split_count = locater_uart_split_str_bychar(
                atres_array[i].atreq_content, ",:", split_array, ARRAY_LEN(atres_array));
        for (j = 0; j < split_count; j++) {
            printf("get_cgatt, elem_%02d: %s\n", j, split_array[j].data);
        }
        if (split_count < 2) {
            printf("get_cgatt, split_count is error: %d\n", split_count);
            return -2;
        }

        /*
        *  检查是否是合理的十进制数值
        */
        p_status = split_array[1].data;
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    p_atcmd = "AT+NETOPEN\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("netopen, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("netopen, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp("+NETOPEN:", atres_array[i].atreq_content, 8))
            continue;
        
        printf("netopen, result is ok\n");
        ret = 0;
        break;
    }

    split_count = locater_uart_split_str_bychar(
            atres_array[i].atreq_content, ":", split_array, ARRAY_LEN(atres_array));
    for (j = 0; j < split_count; j++) {
        printf("netopen, elem_%02d: %s\n", j, split_array[j].data);
    }
    if (split_count < 2) {
        printf("netopen, split_count is error: %d\n", split_count);
        return -2;
    }

    err = atoi(split_array[i].data);
    printf("netopen, result, netopen_result: %d\n", err);
    ret = err ? -3 : 0;

    return ret;
}

/**
 * @brief 打开MQTT之前的准备
 * 
 * @param mqtt_handle  [in ] 
 * @return int 
 * 
 * @details 
 */
static int locater_uart_set_mqtt_open(int mqtt_handle)
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
    char *lp_res_want_ack_str = NULL, *lp_res_want_ret_str = NULL;
    unsigned int at_res_want_ack_str_len = 0, at_res_want_ret_str_len = 0;
    bool is_res_want_ack_str_ok = false, is_res_want_ret_str_ok = false;
    char *p_want_result_str = NULL;
    int client_id = 0;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    // 配置 MQTT SSL 模式和SSL 上下文索引
    // AT+QMTCFG="ssl",<client_idx>[,<SSL_enable>[,<SSL_ctx_idx>]]
    // <SSL_enable> 整型。配置 MQTT SSL 模式。
    // 0 使用普通 TCP 连接
    // 1 使用 SSL TCP 安全连接
    p_atcmd = "AT+QMTCFG=\"ssl\",0,0,0\r\n";
    p_want_result_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_result_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_open, failed\n");
        return ret;
    }

    /*
    * Response
    * 响应
    * 若省略可选参数，则查询当前 MQTT SSL 模式以及 SSL 上下
    * 文索引配置情况：
    * +QMTCFG: "ssl",<SSL_enable>[,<SSL_ctx_idx>]
    * OK
    */
    ret = -2;
    lp_res_want_ack_str = "AT+QMTCFG";
    lp_res_want_ret_str = "OK";
    at_res_want_ack_str_len = strlen(lp_res_want_ack_str);
    at_res_want_ret_str_len = strlen(lp_res_want_ret_str);
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_open, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (!strncmp(lp_res_want_ack_str, atres_array[i].atreq_content, at_res_want_ack_str_len))
            is_res_want_ack_str_ok = true;
        else if (!strncmp(lp_res_want_ret_str, atres_array[i].atreq_content, at_res_want_ret_str_len))
            is_res_want_ret_str_ok = true;
    }
    if (!is_res_want_ack_str_ok || !is_res_want_ret_str_ok) {
        printf("mqtt_open, failed ack: %d, ret: %d\n", is_res_want_ack_str_ok, is_res_want_ret_str_ok);
        return ret;
    }

    // MQTT 客户端打开网络。
    // AT+QMTOPEN=0,"iot-as-mqtt.cn-shanghai.aliyuncs.com",1883
    // OK
    p_atcmd = "AT+QMTOPEN=0,\"1.117.221.12\",1883\r\n";
    p_want_result_str = "+QMTOPEN:";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_result_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_open, failed\n");
        return ret;
    }

    ret = -2;
    lp_res_want_ack_str = "+QMTOPEN:";
    at_res_want_ack_str_len = strlen(lp_res_want_ack_str);
    is_res_want_ack_str_ok = false;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_open, line_%02d, len_%02d: %s\n", i, \
            strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(lp_res_want_ack_str, atres_array[i].atreq_content, at_res_want_ack_str_len))
            continue;
        is_res_want_ack_str_ok = true;
        break;
    }

    if (!is_res_want_ack_str_ok) {
        printf("mqtt_open, not found str: %s\n", lp_res_want_ack_str);
        return ret;
    }
    split_count = locater_uart_split_str_bychar(
            atres_array[i].atreq_content, ",:", split_array, ARRAY_LEN(atres_array));
    for (j = 0; j < split_count; j++) {
        printf("mqtt_open, elem_%02d: %s\n", j, split_array[j].data);
    }
    if (split_count < 3) {
        printf("mqtt_open, split_count is error: %d\n", split_count);
        return -2;
    }

    client_id = atoi(split_array[1].data);
    err = atoi(split_array[2].data);
    printf("mqtt_open, client_id: %d, err: %d\n", client_id, err);
    ret = err ? -3 : 0;

    printf("mqtt_open, done\n");

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
    char *lp_res_want_ack_str = NULL, *lp_res_want_ret_str = NULL;
    unsigned int at_res_want_ack_str_len = 0, at_res_want_ret_str_len = 0;
    bool is_res_want_ack_str_ok = false, is_res_want_ret_str_ok = false;
    int client_id = 0, result = 0, ret_value = 0;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    // AT+QMTCONN=<client_idx>,<clientid>[,<username>,<password>]
    // 响应
    // OK
    // +QMTCONN: <client_idx>,<result>[,<ret_code>]
    // 若出现任何错误：
    // ERROR
    lp_res_want_ack_str = "+QMTCONN:";
    p_atcmd = "AT+QMTCONN=0,\"wPkWCaQVZ\",\"ABCABCABC\",\"ABCABCABC\"\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, lp_res_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_connect, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  OK
    */
    ret = -2;
    is_res_want_ack_str_ok = false;
    at_res_want_ack_str_len = strlen(lp_res_want_ack_str);
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_connect, line_%02d, len_%02d: %s\n", i, \
            strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(lp_res_want_ack_str, atres_array[i].atreq_content, at_res_want_ack_str_len))
            continue;
        is_res_want_ack_str_ok = true;
        break;
    }
    if (!is_res_want_ack_str_ok) {
        printf("mqtt_connect, do not found key str!\n");
        return ret;
    }

    // +QMTCONN: <TCP_connectID>,<result>[,<ret_code>]
    // <result> 整型。命令执行结果。
    // 0 数据包发送成功且从服务器接收到 ACK
    // 1 数据包重传
    // 2 数据包发送失败
    // <ret_code> 整型。连接返回码。
    // 0 接受连接
    // 1 拒绝连接：不支持的协议版本
    // 2 拒绝连接：拒绝标识符
    // 3 拒绝连接：服务器不可用
    // 4 拒绝连接：用户名或密码错误
    // 5 拒绝连接：未授权
    split_count = locater_uart_split_str_bychar(
            atres_array[i].atreq_content, ",:", split_array, ARRAY_LEN(atres_array));
    for (j = 0; j < split_count; j++) {
        printf("mqtt_connect, elem_%02d: %s\n", j, split_array[j].data);
    }
    if (split_count < 4) {
        printf("mqtt_connect, split_count is error: %d\n", split_count);
        return -2;
    }

    client_id = atoi(split_array[1].data);
    result = atoi(split_array[2].data);
    ret_value = atoi(split_array[3].data);
    printf("mqtt_connect, client_id: %d, result: %d, ret_value: %d\n", client_id, result, ret_value);
    if (result || ret_value) {
        printf("mqtt_connect, error result: %d, ret_value: %d\n", result, ret_value);
        return -3;
    }

    ret = 0;
    printf("mqtt_connect, done\n");

    return ret;
}

static int locater_uart_get_pb_done(struct locater_atres_pbdone_fmt_s *p_pb_done)
{
    int ret;
    int i = 0, j = 0;
    int at_res_line = 0;
    char *p_atcmd = NULL;
    int split_count = 0;
    char *lp_res_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    if (!p_pb_done)
        return -1;

    p_atcmd = "\r\n\r\n";
    lp_res_want_ack_str = "RDY";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, lp_res_want_ack_str, buff, sizeof(buff), 1000, LOCATER_SEND_AT_CMD_WITHOUT_CLEAN);
    if (ret < 0) {
        printf("pb_done, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    * PB DONE
    */
    ret = -2;
    p_pb_done->is_power_on_done = false;
    for (i = 0; i < at_res_line; i++) {
        printf("pb_done, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
#if 0
        if (strncmp("PB DONE", atres_array[i].atreq_content, 7) && \
                strncmp("*ATREADY: 1", atres_array[i].atreq_content, 11))
#else
        if (strncmp("RDY", atres_array[i].atreq_content, 3))
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
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    if (!p_cgsn)
        return -1;

    p_atcmd = "AT+CGSN\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("imei_str, failed\n");
        return ret;
    }
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

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
        printf("imei_str, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        /*
        *  找到关键字+CSQ为止
        */
        imei_len = 0;
        for (j = 0; j < LOCATER_IMEI_SIZE; j++) {
            if (atres_array[i].atreq_content[j] && \
                (atres_array[i].atreq_content[j] >= '0') && \
                (atres_array[i].atreq_content[j] <= '9'))
            {
                imei_len++;
            }
        }

        if (imei_len != LOCATER_IMEI_SIZE)
            continue;

        strncpy(p_cgsn->imei_buff, atres_array[i].atreq_content, LOCATER_IMEI_SIZE);
        for (j = 0; j < LOCATER_IMEI_SIZE; j++) {
            p_cgsn->imei_64 += (atres_array[i].atreq_content[j] - '0') * imei_base;
            imei_base /= 10;
        }

        printf("imei_str, str: %s, val: %lld\n", p_cgsn->imei_buff, p_cgsn->imei_64);
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    // 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    // AT+CMQTTSUBTOPIC=0,11,1		
    // 参数依次含义：client_0, 11个字节，QoS=1
    // > AzRxBWxbZ/#
    p_atcmd = "AT+CMQTTSUBTOPIC=0,11,1\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_subtopic, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_subtopic, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(">", atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_subtopic, wait input comunication ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_subtopic, result is error\n");
        return ret;
    }

    // 订阅主题
    sprintf(at_cmd_buf, "%s/#\r\n", s_locater_device_serial_buff);
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, NULL, buff, sizeof(buff), 1000, 0);
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_subtopic, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp("OK", atres_array[i].atreq_content, 13))
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
 * @brief AT+CMQTTSUB=0,14,1			// 查询是否有这个主题
 * 
 * @return int 
 */
static int locater_uart_set_mqtt_sub(int client_handle, char *p_topic_str, int qos, int retain, unsigned int flags)
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
    char *p_res_want_ack_str = NULL, *p_res_want_ret_str = NULL;
    int client_id = 0, msg_id = 0, result = 0, value = 0;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    // 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    // AT+CMQTTSUBTOPIC=0,11,1		
    // 参数依次含义：client_0, 11个字节，QoS=1
    // > AzRxBWxbZ/#	
    // 订阅主题。
    // AT+QMTSUB=0,1,"topic/example",2
    // OK
    // +QMTSUB: 0,1,0,2	
    if (client_handle < 0 || !p_topic_str) {
        printf("mqtt_sub, detect argv is error\n");
        return -1;
    }

    p_res_want_ack_str = "+QMTSUB:";
    p_atcmd = "AT+QMTSUB=%d,1,\"%s\",%d\r\n";
    sprintf(at_cmd_buf, p_atcmd, client_handle, p_topic_str, qos);
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, p_res_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_sub, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, ARRAY_LEN(atres_array));

    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_sub, line_%02d, len_%02d: %s\n", \
            i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(p_res_want_ack_str, atres_array[i].atreq_content, strlen(p_res_want_ack_str)))
            continue;
        
        printf("mqtt_sub, wait key str ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_sub, result is error\n");
        return ret;
    }
    split_count = locater_uart_split_str_bychar(
            atres_array[i].atreq_content, ",:", split_array, ARRAY_LEN(atres_array));
    for (j = 0; j < split_count; j++) {
        printf("mqtt_sub, elem_%02d: %s\n", j, split_array[j].data);
    }
    if (split_count < 5) {
        printf("mqtt_sub, split_count is error: %d\n", split_count);
        return -2;
    }

    // +QMTSUB: <TCP_connectID>,<msgID>,<result>[,<value>]
    // <msgID> 整型。数据包的消息标识符。范围： 1~65535。
    // <result> 整型。命令执行结果。
    // 0 数据包发送成功且从服务器接收到 ACK
    // 1 数据包重传
    // 2 数据包发送失败
    // <value> 若<result>=0，则为已确认 QoS 等级的矢量；参数取值 128 表示服务器拒绝订阅；
    // 若<result>=1，则表示数据包重传次数；
    // 若<result>=2，则不显示。
    client_id = atoi(split_array[1].data);
    msg_id = atoi(split_array[2].data);
    result = atoi(split_array[3].data);
    value = atoi(split_array[4].data);

    printf("mqtt_sub, client_id: %d, msg_id: %d, result: %d, value: %d\n", client_id, msg_id, result, value);
    if (client_id != client_handle || \
            msg_id != 1 || result == 2)
    {
        printf("mqtt_sub, detect error\n");
        return -5;
    }

    printf("mqtt_sub, done\n");

    return ret;
}


/**
 * @brief AT+CMQTTSUB=0,14,1			// 查询是否有这个主题
 * 
 * @return int 
 */
static int locater_uart_set_mqtt_will(int client_handle, 
        char *p_will_topic_str, char *p_will_payload_str, int qos, int retain, unsigned int flags)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[128] = {0};
    unsigned int err = 0;
    unsigned int client = 0;
    unsigned int connect_err = 0;
    unsigned int recv_cnt = 0;
    char *p_res_want_ack_str = NULL, *p_res_want_ret_str = NULL;
    int client_id = 0, msg_id = 0, result = 0, value = 0;
    bool is_res_want_ack_ok = false, is_res_want_ret_ok = false;
    unsigned int will_fg = 1;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    // 1.2　Will消息设置：消息主题：SERIAL+“D/0/0” ，Payload=‘0’，QoS=1，Retain=1。
    if (client_handle < 0 || !p_will_topic_str || !p_will_payload_str) {
        printf("mqtt_will, detect argv is error\n");
        return -1;
    }

    p_atcmd = "AT+QMTCFG=\"will\",%d,%d,%d,%d,\"%s\",\"%s\"\r\n";
    p_res_want_ack_str = "+QMTCFG:";
    sprintf(at_cmd_buf, p_atcmd, client_handle, will_fg, qos, retain, p_will_topic_str, p_will_payload_str);
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, p_res_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_will, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    ret = -2;
    p_res_want_ret_str = "OK";
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_will, line_%02d, len_%02d: %s\n", \
            i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (!strncmp(p_res_want_ack_str, atres_array[i].atreq_content, strlen(p_res_want_ack_str)))
            is_res_want_ack_ok = true;
        if (!strncmp(p_res_want_ret_str, atres_array[i].atreq_content, strlen(p_res_want_ret_str)))
            is_res_want_ret_ok = true;
    }
    if (!is_res_want_ack_ok || !is_res_want_ret_ok) {
        printf("mqtt_will, result is error\n");
        return ret;
    }

    printf("mqtt_will, done\n");

    return ret;
}

/**
 * @brief AT+CMQTTTOPIC=0,14			// 指定发布主题
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    // 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    // AT+CMQTTSUBTOPIC=0,11,1		// 参数依次含义：client_0, 11个字节，QoS=1
    // > AzRxBWxbZ/#	

    p_atcmd = "AT+CMQTTSUB=%d,%d,1\r\n";
    sprintf(at_cmd_buf, p_atcmd, client_handle, strlen(p_topic_str));
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_topic, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_topic, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(">", atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_topic, wait input comunication ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_topic, result is error\n");
        return ret;
    }

    // 订阅主题
    sprintf(at_cmd_buf, "%s\r\n", p_topic_str);
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, NULL, buff, sizeof(buff), 1000, 0);
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_topic, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp("OK", atres_array[i].atreq_content, 13))
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    // 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    // AT+CMQTTSUBTOPIC=0,11,1		// 参数依次含义：client_0, 11个字节，QoS=1
    // > AzRxBWxbZ/#	

    p_atcmd = "AT+CMQTTPAYLOAD=%d,%d\r\n";
    sprintf(at_cmd_buf, p_atcmd, client_handle, strlen(p_payload_str));
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_payload, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_payload, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(">", atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_payload, wait input comunication ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_payload, result is error\n");
        return ret;
    }

    // 要发布的数据字符串
    sprintf(at_cmd_buf, "%s\r\n", s_locater_device_serial_buff);
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, NULL, buff, sizeof(buff), 1000, 0);
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_payload, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp("OK", atres_array[i].atreq_content, 13))
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
static int locater_uart_set_mqtt_pub(int client_handle, char *p_topic_str, char *p_msg_str, int qos, int retain, unsigned int flags)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0;
    char at_cmd_buf[256] = {0};
    unsigned int err = 0;
    unsigned int client_id = 0;
    unsigned int connect_err = 0;
    unsigned int recv_cnt = 0;
    char *p_res_want_ack_str = NULL, *p_res_want_ret_str = NULL;
    unsigned int msg_id = 0, result = 0, value = 0;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    if (!p_topic_str) {
        printf("mqtt_pub, detect argv error\n");
        return -1;
    }

    // 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    // AT+CMQTTSUBTOPIC=0,11,1		
    // 参数依次含义：client_0, 11个字节，QoS=1
    // > AzRxBWxbZ/#	

    // AT+QMTPUB=<TCP_connectID>,<msgID>,<qos>,<retain>,<topic>,<msg>
    // <msgID> 整型。数据包的消息标识符。 范围： 0~65535。只有当<qos>=0 时，该参数值为 0。
    // <qos> 整型。客户端想要发布消息的 QoS 等级。
    // 0 最多发送一次
    // 1 最少发送一次
    // 2 只发送一次
    // <retain> 整型。消息发送到当前订阅者后，服务器是否保存该消息。
    // 0 消息发送到当前订阅者后，服务器不保存消息。
    // 1 消息发送到当前订阅者后，服务器保存消息。
    // <topic> 字符串类型。待发布主题。 最大长度： 255 字节。
    // <msg> 字符串类型。待发布消息。 最大长度： 700 字节； 若是在数据模式，最大长度： 1024字节。
    p_atcmd = "AT+QMTPUB=0,1,%d,%d,\"%s\",\"%s\"\r\n";
    p_res_want_ack_str = "+QMTPUB:";
    sprintf(at_cmd_buf, p_atcmd, qos, retain, p_topic_str, p_msg_str);
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, p_res_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_pub, failed\n");
        return ret;
    }

    ret = -2;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_pub, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(p_res_want_ack_str, atres_array[i].atreq_content, strlen(p_res_want_ack_str)))
            continue;
        
        printf("mqtt_pub, input content result is ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_pub, result is fail\n");
        return ret;
    }
    split_count = locater_uart_split_str_bychar(
            atres_array[i].atreq_content, ",:", split_array, ARRAY_LEN(atres_array));
    for (j = 0; j < split_count; j++) {
        printf("mqtt_pub, elem_%02d: %s\n", j, split_array[j].data);
    }
    if (split_count < 4) {
        printf("mqtt_pub, split_count is error: %d\n", split_count);
        return -2;
    }

    client_id = atoi(split_array[1].data);
    msg_id = atoi(split_array[2].data);
    result = atoi(split_array[3].data);
    value = atoi(split_array[4].data);

    printf("mqtt_pub, client_id: %d, msg_id: %d, result: %d, value: %d\n", client_id, msg_id, result, value);
    if (client_id != client_handle || msg_id != 1 || \
            result == 2)
    {
        printf("mqtt_pub, detect error\n");
        return -5;
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    // 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    // AT+CMQTTSUBTOPIC=0,11,1		// 参数依次含义：client_0, 11个字节，QoS=1
    // > AzRxBWxbZ/#	

    p_atcmd = "AT+CMQTTSUB=0\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_sub_confirm, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_sub_confirm, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp("OK", atres_array[i].atreq_content, 2))
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    // 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    // AT+CMQTTSUBTOPIC=0,11,1		// 参数依次含义：client_0, 11个字节，QoS=1
    // > AzRxBWxbZ/#	

    p_atcmd = "AT+CMQTTWILLTOPIC=0,15\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_willtopic, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_willtopic, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(">", atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_willtopic, wait input comunication ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_willtopic, result is error\n");
        return ret;
    }

    // 订阅主题
    sprintf(at_cmd_buf, "%sD/0/0\r\n", s_locater_device_serial_buff);
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, NULL, buff, sizeof(buff), 1000, 0);
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_willtopic, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp("OK", atres_array[i].atreq_content, 13))
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
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    // 1.1　上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
    // AT+CMQTTSUBTOPIC=0,11,1		// 参数依次含义：client_0, 11个字节，QoS=1
    // > AzRxBWxbZ/#	

    p_atcmd = "AT+CMQTTWILLMSG=0,1,1\r\n";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, NULL, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("mqtt_willmsg, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    /*
    * Response
    *  <
    */
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_willmsg, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(">", atres_array[i].atreq_content, 1))
            continue;
        
        printf("mqtt_willmsg, wait input comunication ok\n");
        ret = 0;
        break;
    }
    if (ret) {
        printf("mqtt_willmsg, result is error\n");
        return ret;
    }

    // 订阅主题
    sprintf(at_cmd_buf, "0\r\n");
    ret = locater_uart_send_atcmd_2_4g_module(at_cmd_buf, NULL, buff, sizeof(buff), 1000, 0);
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);
    ret = -2;
    for (i = 0; i < at_res_line; i++) {
        printf("mqtt_willmsg, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp("OK", atres_array[i].atreq_content, 13))
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


static int locater_uart_get_network_status(void)
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
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    p_atcmd = "AT+QPING=1,\"www.baidu.com\"\r\n";
    p_want_ack_str = "+QPING:";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("network_status, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    ret = -2;
    p_want_ack_str = "+QPING:";
    for (i = 0; i < at_res_line; i++) {
        printf("network_status, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(p_want_ack_str, atres_array[i].atreq_content, strlen(p_want_ack_str)))
            continue;
        
        printf("network_status, get one: %s\n", atres_array[i].atreq_content);
        ret = 0;
        break;
    }

    return ret;
}

static int locater_uart_get_wifi_list(void)
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
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    uart_set_debug_mode(1);
    // 发送AT指令，等待返回OK
    p_atcmd = "AT+QWIFISCAN\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("wifi_scan, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    // 第二次直接等待扫描结果，等到关键字OK表明搜索完成
    p_want_ack_str = "\r\n\r\nOK";
    ret = locater_uart_send_atcmd_2_4g_module(NULL, p_want_ack_str, buff, sizeof(buff), 8000, \
        LOCATER_SEND_AT_CMD_WITHOUT_CLEAN | LOCATER_SEND_AT_CMD_DIRECT_WAIT_RESULT);
    if (ret < 0) {
        printf("wifi_scan, get wifi list failed\n");
        return ret;
    }
    uart_set_debug_mode(0);

    ret = -2;
    p_want_ack_str = "+QWIFISCAN:";
    for (i = 0; i < at_res_line; i++) {
        printf("wifi_scan, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(p_want_ack_str, atres_array[i].atreq_content, strlen(p_want_ack_str)))
            continue;
        
        printf("wifi_scan, get wifi: %s\n", atres_array[i].atreq_content);
        ret = 0;
        break;
    }

    return ret;
}

static int locater_uart_set_config(void)
{
    return 0;
}

static int locater_uart_get_config(void)
{
    return 0;
}


static int locater_uart_set_location_init(void)
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
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    //获取GNSS版本号
    p_atcmd = "AT+QGPSINFO\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("locate_info, get gnss version failed\n");
        return ret;
    }

    //获取查询当前配置
    p_atcmd = "AT+QGPSCFG=\"outport\"\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("locate_info, get outport failed\n");
        return ret;
    }

    //获取当前输出端口
    p_atcmd = "AT+QGPSCFG=\"outport\",\"uartdebug\"\r\n";
    // p_atcmd = "AT+QGPSCFG=\"outport\",\"uartmain\"\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("locate_info, set outport failed\n");
        return ret;
    }

    //获取当前原语
    p_atcmd = "AT+QGPSCFG=\"gpsnmeatype\"\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("locate_info, get gps nmea type failed\n");
        return ret;
    }

    //设置输出原语
    p_atcmd = "AT+QGPSCFG=\"gpsnmeatype\",63\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("locate_info, set gps nmea type failed\n");
        return ret;
    }

    /*
    *  选择混合定位
    *  0 单GPS
    *  3 GPS + GLONASS + Galileo 混合定位（仅 EC200U-EU 和 EC600U-EU 模块支持）
    *  5 GPS + BeiDou混合定位（当模块为非EC200U-EU或EC600U-EU时）
    *  GPS + BeiDou + Galileo混合定位（当模块为EC200U-EU或EC600U-EU时）
    *  7 单BeiDou
    *  <errcode> 操作错误码。详细
    */
    p_atcmd = "AT+QGPSCFG=\"gnssconfig\",5\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("locate_info, set gps_beidou_mix location failed\n");
        return ret;
    }

    //打开AGPS，加快定位
    p_atcmd = "AT+QAGPS=1\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("locate_info, open qgps failed\n");
        return ret;
    }

    //打开GNSS
    p_atcmd = "AT+QGPS=1\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("locate_info, open qgps failed\n");
        return ret;
    }

    return 0;
}

static int locater_uart_set_location_exit(void)
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
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    //关闭GNSS
    p_atcmd = "AT+QGPS=0\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("locate_info, failed\n");
        return ret;
    }
    return 0;
}

/**
 * @brief 通过4G模块获取GPS定位信息、
 *      +QGPSLOC: 094518.00,2235.4651N,11358.6896E,1.25,79.1,3,0.000,1.644,0.888,061122,02
 * 
 * @return int 
 * 
 * @details 
 */
static int locater_uart_get_location_info(struct locater_location_info_fmt_s *p_location_info)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    char at_cmd_buf[64] = {0};
    unsigned int err = 0;
    unsigned int client = 0;
    unsigned int connect_err = 0;
    unsigned int recv_cnt = 0;
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char *p_utc = NULL, *p_latitude = NULL, *p_longitude = NULL, *p_hdop = NULL, *p_altitude = NULL, *p_fix = NULL;
    char *p_cog = NULL, *p_spkm = NULL, *p_date = NULL, *p_errcode = NULL, *p_nsat = NULL, *p_spkn = NULL;
    unsigned int split_count = 0, split_index = 0;
    struct locater_location_utc_info_fmt_s *p_utc_fmt = NULL;
    char buff[1024];
    unsigned int hh = 0, mi = 0, ss = 0, yy = 0, mo = 0, dd = 0;
    int lat_dd = 0, lat_mm_h = 0, lat_mm_l = 0;
    int lon_dd = 0, lon_mm_h = 0, lon_mm_l = 0;
    char lat_ns = 0, lon_ew = 0;
    unsigned int time_stamp = 0;
    struct tm info;

    // 获取定位信息
    p_atcmd = "AT+QGPSLOC=0\r\n";
    p_want_ack_str = "+QGPSLOC:";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("locate_info, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    ret = -2;
    p_want_ack_str = "+QGPSLOC:";
    for (i = 0; i < at_res_line; i++) {
        printf("locate_info, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(p_want_ack_str, atres_array[i].atreq_content, strlen(p_want_ack_str)))
            continue;
        
        printf("locate_info, get locate_info: %s\n", atres_array[i].atreq_content);
        ret = 0;
        break;
    }
    if (ret) {
        printf("locate_info, get locate_info failed\n");
        return ret;
    }

    split_count = locater_uart_split_str_bychar(atres_array[i].atreq_content, ":,", split_array, ARRAY_LEN(split_array));
    for (split_index = 0; split_index < split_count; split_index++) {
        printf("locate_info, elem_%02d: %s\n", split_index, split_array[split_index].data);
    }
    if (split_count < 12) {
        printf("locate_info, get split failed\n");
        return -1;
    }

    // UTC 时间。格式：hhmmss.ss（引自 GPGGA 语句）。
    p_utc = split_array[1].data;
    // 纬度。
    p_latitude = split_array[2].data;
    // 经度。
    p_longitude = split_array[3].data;
    // 水平精度因子
    p_hdop = split_array[4].data;
    // 天线的海拔高度
    p_altitude = split_array[5].data;
    // 整型。GNSS 定位模式
    p_fix = split_array[6].data;
    // 字符串类型。以真北方向为基准的地面航向。测速失败或静态场景速度极小时，输出为空。
    p_cog = split_array[7].data;
    // 地面速率。单位：千米/时
    p_spkm = split_array[8].data;
    p_spkn = split_array[9].data;
    // UTC 日期。格式：ddmmyy
    p_date = split_array[10].data;
    // 本系统可见卫星数量。固定两位数，前导位数不足则补 0（引自 GSV 语句）。
    p_nsat = split_array[11].data;

    // 转换成时间戳_时间
    sprintf(buff, "%c%c", p_utc[0], p_utc[1]);
    hh = atoi(buff);
    sprintf(buff, "%c%c", p_utc[2], p_utc[3]);
    mi = atoi(buff);
    sprintf(buff, "%c%c", p_utc[4], p_utc[5]);
    ss = atoi(buff);
    printf("locate_info, get hh: %d, mi: %d, ss: %d\n", hh, mi, ss);
    // 转换成时间戳_日期
    sprintf(buff, "%c%c", p_date[4], p_date[5]);
    yy = atoi(buff);
    sprintf(buff, "%c%c", p_date[2], p_date[3]);
    mo = atoi(buff);
    sprintf(buff, "%c%c", p_date[0], p_date[1]);
    dd = atoi(buff);
    printf("locate_info, get yy: %d, mo: %d, dd: %d\n", yy, mo, dd);

    info.tm_year = (2000 + yy) - 1970;
    info.tm_mon = mo - 1;
    info.tm_mday = dd;
    info.tm_hour = hh;
    info.tm_min = mi;
    info.tm_sec = ss;
    info.tm_isdst = -1;
    time_stamp = mktime(&info);
    // p_location_info->time_stamp = time_stamp - 1640995200;
    p_location_info->time_stamp = time_stamp;
    printf("locate_info, get p_utc: %s, p_date: %s, time_stamp_before: 0x%08x, time_stamp_after: 0x%08x\n", \
            p_utc, p_date, time_stamp, p_location_info->time_stamp);

    /*
    *  转换成纬度
    *  +QGPSLOC: 030832.00,2235.5077N,11358.6698E,2.55,596.5,3,0.000,2.027,1.094,131122,07
    *  N/S: 北纬/南纬
    *  E/W: 东经/西经
    */
    sprintf(buff, "%c%c", p_latitude[0], p_latitude[1]);
    lat_dd = atoi(buff);
    sprintf(buff, "%c%c", p_latitude[2], p_latitude[3]);
    lat_mm_h = atoi(buff);
    sprintf(buff, "%c%c%c%c", p_latitude[5], p_latitude[6], p_latitude[7], p_latitude[8]);
    lat_mm_l = atoi(buff);
    p_location_info->latitude = (lat_dd * 60 * 10000) + (lat_mm_h * 10000) + lat_mm_l;
    lat_ns = p_latitude[9];
    printf("locate_info, lat_dd: %d, lat_mm_h: %d, lat_mm_l: %d\n", lat_dd, lat_mm_h, lat_mm_l);
    if (lat_ns == 'S')
        p_location_info->latitude *= -1;
    else if (lat_ns == 'N') {
        // do nothing
    }
    else {
        printf("locate_info, get lat_ns fail: %c\n", lat_ns);
    }
    printf("locate_info, get p_latitude_str: %s, hex: 0x%08x, dec: %d\n", p_latitude, \
        p_location_info->latitude, p_location_info->latitude);

    // 转换成经度
    sprintf(buff, "%c%c%c", p_longitude[0], p_longitude[1], p_longitude[2]);
    lon_dd = atoi(buff);
    sprintf(buff, "%c%c", p_longitude[3], p_longitude[4]);
    lon_mm_h = atoi(buff);
    sprintf(buff, "%c%c%c%c", p_longitude[6], p_longitude[7], p_longitude[8], p_longitude[9]);
    lon_mm_l = atoi(buff);
    p_location_info->longitude = (lon_dd * 60 * 10000) + (lon_mm_h * 10000) + lon_mm_l;
    printf("locate_info, lon_dd: %d, lon_mm_h: %d, lon_mm_l: %d\n", lon_dd, lon_mm_h, lon_mm_l);
    lon_ew = p_longitude[10];
    if (lon_ew == 'W')
        p_location_info->longitude *= -1;
    else if (lon_ew == 'E') {
        // do nothing
    }
    else {
        printf("locate_info, get lon_ew fail: %c\n", lon_ew);
    }
    printf("locate_info, get p_longitude_str: %s, hex: 0x%08x, dec: %d\n", p_longitude, \
        p_location_info->longitude, p_location_info->longitude);

    printf("locate_info, get p_hdop: %s\n", p_hdop);
    printf("locate_info, get p_altitude: %s\n", p_altitude);
    printf("locate_info, get p_fix: %s\n", p_fix);
    printf("locate_info, get p_cog: %s\n", p_cog);
    printf("locate_info, get p_spkm: %s\n", p_spkm);
    printf("locate_info, get p_spkn: %s\n", p_spkn);
    printf("locate_info, get p_date: %s\n", p_date);
    printf("locate_info, get p_nsat: %s\n", p_nsat);    

    return ret;
}

static int locater_uart_get_ati(void)
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
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];

    // 发送AT指令，等待返回OK
    p_atcmd = "ATI\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("product_id, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    return ret;
}

/**
 * @brief 从4G模块获取SNTP时间
 * 
 * @return int 
 * 
 * @details 
 */
static int locater_uart_get_sntp_time(struct locater_local_time_info_fmt_s *p_local_time)
{
    int ret;
    int i = 0, j = 0;
    char *p_atcmd = NULL;
    int at_res_line = 0;
    int split_count = 0, split_index = 0;
    char at_cmd_buf[64] = {0};
    unsigned int err = 0;
    unsigned int client = 0;
    unsigned int connect_err = 0;
    unsigned int recv_cnt = 0;
    char *p_want_ack_str = NULL;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};
    char buff[1024];
    char *p_zone = NULL, *p_year = NULL, *p_month = NULL, *p_day = NULL;
    char *p_hour = NULL, *p_minute = NULL, *p_second = NULL;

    if (!p_local_time) {
        printf("get_sntp_time, argv is error\n");
        return -1;
    }

    // +CCLK: "22/11/19,05:28:27+32"
    p_atcmd = "AT+CCLK?\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("get_sntp_time, failed\n");
        return ret;
    }
    recv_cnt = ret;
    at_res_line = locater_uart_process_at_response(buff, atres_array, 32);

    ret = -2;
    p_want_ack_str = "+CCLK:";
    for (i = 0; i < at_res_line; i++) {
        printf("get_sntp_time, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(p_want_ack_str, atres_array[i].atreq_content, strlen(p_want_ack_str)))
            continue;
        
        printf("get_sntp_time, get time: %s\n", atres_array[i].atreq_content);
        ret = 0;
        break;
    }
    if (ret) {
        printf("get_sntp_time, get time failed\n");
        return ret;
    }

    split_count = locater_uart_split_str_bychar(atres_array[i].atreq_content, ":,/\"+", split_array, ARRAY_LEN(split_array));
    for (split_index = 0; split_index < split_count; split_index++) {
        printf("get_sntp_time, elem_%02d: %s\n", split_index, split_array[split_index].data);
    }
    if (split_count < 11) {
        printf("get_sntp_time, get split failed\n");
        return -1;
    }

    p_zone = split_array[9].data;
    p_year = split_array[3].data;
    p_month = split_array[4].data;
    p_day = split_array[5].data;
    p_hour = split_array[6].data;
    p_minute = split_array[7].data;
    p_second = split_array[8].data;
    
    p_local_time->zone = atoi(p_zone) / 4;
    p_local_time->year = atoi(p_year) + 2000;
    p_local_time->month = atoi(p_month);
    p_local_time->day = atoi(p_day);
    p_local_time->hour = atoi(p_hour);
    p_local_time->minute = atoi(p_minute);
    p_local_time->second = atoi(p_second);
    printf("get_sntp_time, zone: %d, year: %d, month: %d, day: %d, hour: %d, minute: %d, second: %d\n", \
            p_local_time->zone, p_local_time->year, p_local_time->month, \
            p_local_time->day, p_local_time->hour, p_local_time->minute, \
            p_local_time->second);

    return 0;
}


/**
 * @brief 定位的所有任务
 * 
 * @param arg  [in ] 
 * 
 * @details 
 */
static void locater_uart_app_location_misc_task(void *arg)
{
    int ret;
    unsigned int step = 0, step_bakup = 0;
    char mqtt_topic_str_buf[256] = {0}, mqtt_payload_str_buf[256] = {0};
    int client_handle = 0;
    unsigned int qos = 0, remain = 0;
    bool is_locator_init = false;
    unsigned int curr_app_idx = 0;
    unsigned int positioning_idx = 0;

    while (true) {
LOCATOR_UART_FSM_COM_STEP_ENTRY(0) {
        // 循环等待标志位
        if (s_is_locater_uart_once_positioning)
            step = 1;
        else if (s_is_locater_uart_continuous_positioning)
            step = 5;
        else {
            v_task_delay(1000 / port_tick_rate_ms);
            continue;
        }
        curr_app_idx = s_locater_uart_curr_app_idx;
      }

LOCATOR_UART_FSM_COM_STEP_ENTRY(1) {
        // 标志位置位false
        printf("location_misc, detect device once positioning\n");
        s_is_locater_uart_once_positioning = false;
        step++;

        /*
        *  如果主程序告知设备没有准备好，那么放弃执行任务。
        */
        if (!s_is_locater_device_ready) {
            printf("location_misc, detect device is not ready\n");
            continue;
        }

        /*
        *  单次定位在没有GPS、WIFI定位信息的情况下，提供LBS定位信息。
        *  按照协议发布到指定的主题。
        */
        // sprintf(mqtt_topic_str_buf, "%s/07", p_serial);
        // locater_uart_set_mqtt_pub(client_handle, mqtt_topic_str_buf, mqtt_payload_str_buf, qos, remain, 0);
        // printf("location_misc, upload once positioning done\n");
        step = 0;
        continue;
      }

    ///< 连续定位的初始化
LOCATOR_UART_FSM_COM_STEP_ENTRY(5) {
        printf("location_misc, continuous positioning init\n");
        locater_uart_set_location_init();
        wifi_init();
        step++;
    }

LOCATOR_UART_FSM_COM_STEP_ENTRY(6) {
        struct locater_location_info_fmt_s location_info = {0};
        struct locator_uart_protocol_dev_upload_location_gps_fmt_s *p_location_gps = NULL;
        struct locator_uart_protocol_dev_upload_location_wifi_fmt_s *p_location_wifi = NULL;
        struct locator_uart_protocol_dev_upload_location_mult_payload_fmt_s *p_location_mul = NULL;
        unsigned int payload_size = 1;
        int check_payload_size = 0;
        int check_longitude = 0, check_latitude = 0;
        int location_res = 0;
        unsigned int wifi_cnt = 0, wifi_idx = 0;
	    STRU_WIFI_INFO *p_wifi_info  = NULL;
        char mac_rssi = 0;
        char *p_offs = NULL;
        unsigned int time_stamp = 0;
        unsigned int location_count = 0;

        printf("location_misc, device continuous positioning: %08d\n", positioning_idx);
        positioning_idx++;
        // 检测到用户换了手机，或者同一个手机可能重新登录，那么结束本次任务
        if (curr_app_idx != s_locater_uart_curr_app_idx || \
                !s_is_locater_uart_continuous_positioning)
        {
            printf("location_misc, detect app idx is update\n");
            s_is_locater_uart_continuous_positioning = false;
            locater_uart_set_location_exit();
            step = 0;
            positioning_idx = 0;
            continue;
        }

        memset(mqtt_payload_str_buf, 0, sizeof(mqtt_payload_str_buf));
        p_location_mul = (struct locator_uart_protocol_dev_upload_location_mult_payload_fmt_s *)mqtt_payload_str_buf;
        p_offs = (char *)&p_location_mul->data;
        time_stamp = time(NULL);
        printf("location_misc, time_stamp1: %ld\n", time_stamp);
        locater_assert(time_stamp < LOCATER_SERVER_TIME);
        time_stamp -= LOCATER_SERVER_TIME;
        printf("location_misc, time_stamp2: %ld\n", time_stamp);

        ///< 获取WIFI信号
        wifi_scan();
        wifi_cnt = wifi_get_info(&p_wifi_info);
        printf("location_misc, detect wifi_cnt: %d\n", wifi_cnt);
        if (wifi_cnt) {
            p_location_wifi = (struct locator_uart_protocol_dev_upload_location_wifi_fmt_s *)p_offs;
            p_location_wifi->type = 0x32;
            locater_int_to_code86(wifi_cnt, sizeof(char), (char *)&p_location_wifi->count, 1);
            locater_int_to_code86(time_stamp, sizeof(int), (char *)&p_location_wifi->time_stamp, 4);
            p_offs = p_location_wifi->info->mac;
            for (wifi_idx = 0; wifi_idx < wifi_cnt; wifi_idx++) {
                struct locator_uart_location_wifi_item_s *p_item = \
                    (struct locator_uart_location_wifi_item_s *)p_offs;
                ///< 按协议。取反+100 范围0-216
                mac_rssi = p_wifi_info[wifi_idx].rssi < 0 ? -(p_wifi_info[wifi_idx].rssi) : p_wifi_info[wifi_idx].rssi;
                mac_rssi += 70;
                printf("location_misc, wifi_idx: %04d, mac: %02x-%02x-%02x-%02x-%02x-%02x, ssid: %s, "
                    "rssi: %d(dec: %d, hex: 0x%02x)\r\n", wifi_idx, \
                    p_wifi_info[wifi_idx].mac[0], p_wifi_info[wifi_idx].mac[1], \
                    p_wifi_info[wifi_idx].mac[2], p_wifi_info[wifi_idx].mac[3], \
                    p_wifi_info[wifi_idx].mac[4], p_wifi_info[wifi_idx].mac[5], \
                    p_wifi_info[wifi_idx].ssid, p_wifi_info[wifi_idx].rssi, mac_rssi, mac_rssi);
                sprintf(p_item->mac, "%x%x:%x%x:%x%x:%x%x:%x%x:%x%x", \
                   (p_wifi_info[wifi_idx].mac[0] >> 4) & 0x0f, (p_wifi_info[wifi_idx].mac[0] >> 0) & 0x0f, \
                   (p_wifi_info[wifi_idx].mac[1] >> 4) & 0x0f, (p_wifi_info[wifi_idx].mac[1] >> 0) & 0x0f, \
                   (p_wifi_info[wifi_idx].mac[2] >> 4) & 0x0f, (p_wifi_info[wifi_idx].mac[2] >> 0) & 0x0f, \
                   (p_wifi_info[wifi_idx].mac[3] >> 4) & 0x0f, (p_wifi_info[wifi_idx].mac[3] >> 0) & 0x0f, \
                   (p_wifi_info[wifi_idx].mac[4] >> 4) & 0x0f, (p_wifi_info[wifi_idx].mac[4] >> 0) & 0x0f, \
                   (p_wifi_info[wifi_idx].mac[5] >> 4) & 0x0f, (p_wifi_info[wifi_idx].mac[5] >> 0) & 0x0f);
                locater_int_to_code86(mac_rssi, sizeof(char), (char *)&p_item->rssi, 1);
                ///< 计算下一次偏移
                p_offs += sizeof(struct locator_uart_location_wifi_item_s);
                location_count++;
            }
        }

        ///< 获取定位
        memset(&location_info, 0, sizeof(location_info));
        location_res = locater_uart_get_location_info(&location_info);
        if (!location_res) {
            p_location_gps = (struct locator_uart_protocol_dev_upload_location_gps_fmt_s *)p_offs;
            // 定位数据类型是31
            p_location_gps->type = 0x31;
            printf("location_misc, gps_time_stamp: %ld, local_time_stamp\n", location_info.time_stamp, time_stamp);
            locater_int_to_code86(time_stamp, sizeof(int), (char *)&p_location_gps->time_stamp, 4);

            // 表示纬度时负数+180转为正数
            if (location_info.latitude < 0)
                location_info.latitude += 180;
            locater_int_to_code86(location_info.latitude, sizeof(int), (char *)&p_location_gps->latitude, 4);
            // 表示经度时，负数+360转为正数
            if (location_info.longitude < 0)
                location_info.longitude += 360;
            locater_int_to_code86(location_info.longitude, sizeof(int), (char *)&p_location_gps->longitude, 4);
            printf("location_misc, final latitude: %d, longitude: %d\n", location_info.latitude, location_info.longitude);

            // 检验
            check_payload_size = locater_code86_to_int(mqtt_payload_str_buf, 2);
            check_longitude = locater_code86_to_int((char *)&p_location_gps->longitude, 4);
            check_latitude = locater_code86_to_int((char *)&p_location_gps->latitude, 4);
            printf("location_misc, check payload_size: %d, latitude: %d, longitude: %d\n", \
                check_payload_size, check_latitude, check_longitude);
            p_offs += sizeof(struct locator_uart_protocol_dev_upload_location_gps_fmt_s);
            location_count++;
        }

        if (!location_count) {
            v_task_delay(8000 / port_tick_period_ms);
            continue;
        }
        locater_int_to_code86(location_count, sizeof(short), (char *)&p_location_mul->count, 2);

        qos = 1;
        remain = 0;
        ESP_LOG_BUFFER_HEXDUMP("locater_mul_lc", mqtt_payload_str_buf, sizeof(mqtt_payload_str_buf), ESP_LOG_INFO);
        sprintf(mqtt_topic_str_buf, "%sD/0/B", p_serial);
        locater_uart_set_mqtt_pub(client_handle, mqtt_topic_str_buf, (char *)mqtt_payload_str_buf, qos, remain, 0);
        printf("location_misc, upload once positioning done\n");
        printf("\n\n");

        // s_is_locater_uart_continuous_positioning = false;
        v_task_delay(8000 / port_tick_period_ms);
        continue;
      }
    }

    return;
}

typedef struct locator_uart_wifi_mac_info_s {
    char is_valid;
    char idx;
    char mac[17];
    char ssid[256];
    int rssi;
} locator_uart_wifi_mac_info_t;

static struct locator_uart_wifi_mac_info_s s_locator_uart_wifi_info_array[64] = {0};
static unsigned int s_locator_uart_wifi_info_size = 0;
static unsigned char s_locator_uart_wifi_scan_mac[32] = {0};

static int locater_uart_app_set_wifi_scan_mac_array(char *p_mqtt_payload)
{
    int i = 0;
    struct locator_uart_protocol_app_set_wifi_mac_array_payload_fmt_s *p_mac_array = \
            (struct locator_uart_protocol_app_set_wifi_mac_array_payload_fmt_s *)p_mqtt_payload;

    memset(s_locator_uart_wifi_info_array, 0, sizeof(s_locator_uart_wifi_info_array));
    printf("wifi_set_mac_array, update wifi mac array, wifi_count: %d\n", p_mac_array->wifi_count);
    for (i = 0; i < p_mac_array->wifi_count; i++) {
        s_locator_uart_wifi_info_array[i].is_valid = 1;
        s_locator_uart_wifi_info_array[i].idx = p_mac_array->idx_mac_array[i].idx;
        memcpy(&s_locator_uart_wifi_info_array[i].mac, p_mac_array->idx_mac_array[i].mac, 17);
        printf("wifi_set_mac_array, update wifi mac_%02d: 0x%02x-0x%02x-0x%02x-0x%02x-"
            "0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-"
            "0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-0x%02x\n", s_locator_uart_wifi_info_array[i].idx, \
            p_mac_array->idx_mac_array[i].mac[ 0], 
            p_mac_array->idx_mac_array[i].mac[ 1], 
            p_mac_array->idx_mac_array[i].mac[ 2],
            p_mac_array->idx_mac_array[i].mac[ 3],
            p_mac_array->idx_mac_array[i].mac[ 4], 
            p_mac_array->idx_mac_array[i].mac[ 5], 
            p_mac_array->idx_mac_array[i].mac[ 6], 
            p_mac_array->idx_mac_array[i].mac[ 7],
            p_mac_array->idx_mac_array[i].mac[ 8], 
            p_mac_array->idx_mac_array[i].mac[ 9], 
            p_mac_array->idx_mac_array[i].mac[10], 
            p_mac_array->idx_mac_array[i].mac[11],
            p_mac_array->idx_mac_array[i].mac[12], 
            p_mac_array->idx_mac_array[i].mac[13], 
            p_mac_array->idx_mac_array[i].mac[14], 
            p_mac_array->idx_mac_array[i].mac[15],
            p_mac_array->idx_mac_array[i].mac[16]);
    }

    s_locator_uart_wifi_info_size = p_mac_array->wifi_count;
    return 0;
}

static int locater_uart_app_set_user_setting(char *p_mqtt_payload)
{
    struct locator_uart_protocol_app_set_user_setting_payload_fmt_s *p_user_setting = \
            (struct locator_uart_protocol_app_set_user_setting_payload_fmt_s *)p_mqtt_payload;



    return 0;
}

/**
 * @brief WIFI的所有任务
 * 
 * @param arg  [in ] 
 * 
 * @details 
 */
static void locater_uart_app_wifi_misc_task(void *arg)
{
    unsigned int step = 0, step_bakup = 0;
	STRU_WIFI_INFO *p_wifi_info  = NULL;
    unsigned int wifi_cnt = 0, wifi_idx = 0;
    char mqtt_topic_str_buf[256] = {0}, mqtt_payload_str_buf[256] = {0};
    unsigned int qos = 0, remain = 0;
    unsigned int i = 0, client_handle = 0;

    while (true) {
LOCATOR_UART_FSM_COM_STEP_ENTRY(0) {
        if (s_is_locater_uart_need_wifi_scan_rssi_by_mac_array)
            step = 1;
        else if (s_is_locater_uart_need_wifi_scan_rssi_by_mac)
            step = 2;
        else {
            v_task_delay(1000 / port_tick_rate_ms);
            continue;
        }
      }

    // 按照协议上报wifi的rssi值
LOCATOR_UART_FSM_COM_STEP_ENTRY(1) {
        struct locator_uart_protocol_dev_set_wifi_rssi_array_payload_fmt_s *p_wifi_rssi_payload_array = NULL;
        printf("uart_app_wifi_scan, detect device wifi scan by mac array\n");
        wifi_init();
        wifi_scan();
        wifi_cnt = wifi_get_info(&p_wifi_info);
        printf("uart_app_wifi_scan, detect wifi_cnt: %d\n", wifi_cnt);
        for (wifi_idx = 0; wifi_idx < wifi_cnt; wifi_idx++) {
            printf("uart_app_wifi_scan, wifi_idx: %02d, ssid: %s, rssi: %d, mac: %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\r\n", \
                wifi_idx, p_wifi_info[wifi_idx].ssid, p_wifi_info[wifi_idx].rssi, \
                p_wifi_info[wifi_idx].mac[0], p_wifi_info[wifi_idx].mac[1], p_wifi_info[wifi_idx].mac[2],
                p_wifi_info[wifi_idx].mac[3], p_wifi_info[wifi_idx].mac[4], p_wifi_info[wifi_idx].mac[5],
                p_wifi_info[wifi_idx].mac[6]);
            // 匹配相同的mac地址，获取rssi信号强度
            for (i = 0; i < s_locator_uart_wifi_info_size; i++) {
                if (!memcmp(p_wifi_info[wifi_idx].mac, s_locator_uart_wifi_info_array[i].mac, 17)) {
                    s_locator_uart_wifi_info_array[i].rssi = p_wifi_info[wifi_idx].rssi;
                    break;
                }
            }
        }

        // 按照协议上报wifi的rssi值
        sprintf(mqtt_topic_str_buf, "%sD/7N", p_serial);
        memset(mqtt_payload_str_buf, 0, sizeof(mqtt_payload_str_buf));
        p_wifi_rssi_payload_array = (struct locator_uart_protocol_dev_set_wifi_rssi_array_payload_fmt_s *)mqtt_payload_str_buf;
        p_wifi_rssi_payload_array->wifi_count = wifi_cnt;
        for (wifi_idx = 0; wifi_idx < wifi_cnt; wifi_idx++) {
            p_wifi_rssi_payload_array->idx_rssi_array[wifi_idx].idx = s_locator_uart_wifi_info_array[i].idx;
            p_wifi_rssi_payload_array->idx_rssi_array[wifi_idx].rssi = s_locator_uart_wifi_info_array[i].rssi;
        }
        locater_uart_set_mqtt_pub(client_handle, mqtt_topic_str_buf, mqtt_payload_str_buf, qos, remain, 0);
        s_is_locater_uart_need_wifi_scan_rssi_by_mac_array = false;
        step = 0;
      }

    // 按照协议上报指定wifi的rssi值
LOCATOR_UART_FSM_COM_STEP_ENTRY(2) {
        char mac_rssi = 0;
        struct locator_uart_protocol_dev_set_wifi_rssi_payload_fmt_s *p_wifi_rssi_payload = NULL;
        printf("uart_app_wifi_scan, detect device wifi scan by mac\n");
        wifi_init();
        wifi_scan();
        wifi_cnt = wifi_get_info(&p_wifi_info);
        printf("uart_app_wifi_scan, detect wifi_cnt: %d\n", wifi_cnt);
        for (wifi_idx = 0; wifi_idx < wifi_cnt; wifi_idx++) {
            printf("uart_app_wifi_scan, wifi_idx: %02d, ssid: %s, rssi: %d, mac: %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\r\n", \
                wifi_idx, p_wifi_info[wifi_idx].ssid, p_wifi_info[wifi_idx].rssi, \
                p_wifi_info[wifi_idx].mac[0], p_wifi_info[wifi_idx].mac[1], p_wifi_info[wifi_idx].mac[2],
                p_wifi_info[wifi_idx].mac[3], p_wifi_info[wifi_idx].mac[4], p_wifi_info[wifi_idx].mac[5],
                p_wifi_info[wifi_idx].mac[6]);
            // 匹配相同的mac地址，获取rssi信号强度
            if (!memcmp(p_wifi_info[wifi_idx].mac, s_locator_uart_wifi_scan_mac, 17)) {
                mac_rssi = p_wifi_info[wifi_idx].rssi;
                break;
            }
        }

        // 按照协议上报指定wifi的rssi值
        sprintf(mqtt_topic_str_buf, "%sD/signal4N", p_serial);
        memset(mqtt_payload_str_buf, 0, sizeof(mqtt_payload_str_buf));
        p_wifi_rssi_payload = (struct locator_uart_protocol_dev_set_wifi_rssi_payload_fmt_s *)mqtt_payload_str_buf;
        p_wifi_rssi_payload->id = s_is_locater_uart_need_wifi_scan_rssi_by_mac_app_idx;
        p_wifi_rssi_payload->rssi = mac_rssi;
        locater_uart_set_mqtt_pub(client_handle, mqtt_topic_str_buf, mqtt_payload_str_buf, qos, remain, 0);
        s_is_locater_uart_need_wifi_scan_rssi_by_mac_app_idx = 0;
        s_is_locater_uart_need_wifi_scan_rssi_by_mac = false;
        step = 0;
      }
    }

    return;
}


/**
 * @brief 在topic里边获取mac地址和密码，在payload获取下载固件的地址链接
 * 
 * @param p_wifi_mac_password_str  [in ] 
 * @param p_mqtt_payload  [in ] 
 * @return int 
 * 
 * @details 
 */
static int locater_uart_app_set_ota_conf(char *p_wifi_mac_password_str, char *p_mqtt_payload)
{
    struct locator_uart_protocol_app_set_ota_conf_payload_fmt_s *p_url = \
            (struct locator_uart_protocol_app_set_ota_conf_payload_fmt_s *)p_mqtt_payload;
    char *p_mac = NULL;

    printf("app_set_ota_config, start\n");

    // 获取WIFI的MAC地址
    p_mac = strstr(p_wifi_mac_password_str, "/");
    if (!p_mac) {
        printf("app_set_ota_config, not found mac str\n");
        return -1;
    }
    p_mac++;
    if (!(*p_mac)) {
        printf("app_set_ota_config, mac is error\n");
        return -2;
    }
    memset(s_is_locater_uart_ota_wifi_mac, 0, sizeof(s_is_locater_uart_ota_wifi_mac));
    strncpy(s_is_locater_uart_ota_wifi_mac, p_mac, 17);

    // 获取WIFI的密码
    p_mac += 17;
    memset(s_is_locater_uart_ota_wifi_password, 0, sizeof(s_is_locater_uart_ota_wifi_password));
    strncpy(s_is_locater_uart_ota_wifi_password, p_mac, 17);

    // 获取固件的URL
    memset(s_is_locater_uart_ota_firmware_url, 0, sizeof(s_is_locater_uart_ota_firmware_url));
    strncpy(s_is_locater_uart_ota_wifi_password, p_url->url, p_url->url_len);

    return 0;
}

static void locater_uart_app_ota_misc_task(void *arg)
{
    unsigned int step = 0, step_bakup = 0;
	STRU_WIFI_INFO *p_wifi_info  = NULL;
    unsigned int wifi_cnt = 0, wifi_idx = 0;
    char mqtt_topic_str_buf[256] = {0}, mqtt_payload_str_buf[256] = {0};
    unsigned int qos = 0, remain = 0;
    unsigned int i = 0, client_handle = 0;
    unsigned int locater_uart_ota_stop_task_count_bak = 0;

    while (true) {
    // 开机启动，检查OTA的状态
LOCATOR_UART_FSM_COM_STEP_ENTRY(0) {
        if (s_is_locater_uart_need_ota)
            step = 1;
        else {
            v_task_delay(1000 / port_tick_rate_ms);
            continue;
        }
      }

    // 广播所有任务停止
LOCATOR_UART_FSM_COM_STEP_ENTRY(1) {
        printf("uart_app_ota, detect ota start\n");
        s_is_locater_uart_need_ota = false;
        s_is_locater_uart_ota_broadcast_stop_task = true;
        step++;
      }

    // 等待所有的任务停止
LOCATOR_UART_FSM_COM_STEP_ENTRY(2) {
        if (s_locater_uart_ota_stop_task_count < 10 && \
                locater_uart_ota_stop_task_count_bak != s_locater_uart_ota_stop_task_count)
        {
            printf("uart_app_ota, detect curr stop task count: %d\n", s_locater_uart_ota_stop_task_count);
            locater_uart_ota_stop_task_count_bak = s_locater_uart_ota_stop_task_count;
            v_task_delay(1000 / port_tick_rate_ms);
            continue;
        }

        sys_ota_set_url(s_is_locater_uart_ota_firmware_url);
        step = 0;
      }
    }

    return;
}

static void locater_uart_app_temp_misc_task(void *arg)
{
    return;
}

/**
 * @brief 设置围栏的名字和是否使能。"QAZWSXQAZ/0/可观看该剧阿达LJL15m"
 * 
 * @param p_conf  [in ] 
 * @return int 
 * 
 * @details 
 */
static int locater_uart_app_set_enclosure(char *p_conf)
{
#define LOCATER_ENCLOSURE_CONF_SPLIT_ARRAY_NAME_IDX  3
    struct locater_str_split_fmt_s split_array[32] = {0};
    unsigned int split_count = 0, split_index = 0, conf_len = 0;
    unsigned int enclosure_idx = 0, enclosure_name_len = 0;
    char *p_enclosure_name = NULL;

    if (!p_conf) {
        printf("enclosure: detect p_conf fail\n");
        return -1;
    }

    conf_len = strlen(p_conf);
    if (conf_len < 3) {
        printf("enclosure,s detect conf_len fail: %d\n", conf_len);
        return -2;
    }

    enclosure_idx = p_conf[conf_len - 3];

    split_count = locater_uart_split_str_bychar(p_conf, "/\"", \
            split_array, ARRAY_LEN(split_array));
    for (split_index = 0; split_index < split_count; split_index++) {
        printf("enclosure, elem_%02d: %s\n", split_index, split_array[split_index].data);
    }

    if (split_count < (LOCATER_ENCLOSURE_CONF_SPLIT_ARRAY_NAME_IDX + 1)) {
        printf("enclosure, detect split_count fail: %d\n", split_count);
        return -3;
    }
    else if (split_array[LOCATER_ENCLOSURE_CONF_SPLIT_ARRAY_NAME_IDX].data_len < 3) {
        printf("enclosure, detect split_data_len fail: %d\n", split_array[LOCATER_ENCLOSURE_CONF_SPLIT_ARRAY_NAME_IDX].data_len);
        return -4;
    }

    s_locater_enclosure_conf.idx = enclosure_idx;
    p_enclosure_name = split_array[LOCATER_ENCLOSURE_CONF_SPLIT_ARRAY_NAME_IDX].data;
    enclosure_name_len = split_array[LOCATER_ENCLOSURE_CONF_SPLIT_ARRAY_NAME_IDX].data_len;
    
    memset(s_locater_enclosure_conf.name, 0, LOCATER_ENCLOSURE_CNF_NAME_LEN);
    strncpy(s_locater_enclosure_conf.name, p_enclosure_name, enclosure_name_len - 3);
    printf("enclosure, conf done\n");
    printf("enclosure, conf name: %s\n", s_locater_enclosure_conf.name);
    printf("enclosure, conf id: 0x%02x\n", s_locater_enclosure_conf.idx);

    return 0;
}

/**
 * @brief 设置围栏的更多参数
 * 
 * @param p_conf_ext  [in ] 
 * @return int 
 * 
 * @details 
 */
static int locater_uart_app_set_enclosure_ext(char *p_conf_ext)
{
    struct locater_enclosure_conf_ext_fmt_s *p_conf = NULL;
    
    char *p = p_conf_ext;
    while (1) {
        if (*p == '\0')
            break;
        printf("0x%02x\n", *p++);
    }

    if (!p_conf_ext) {
        return -1;
    }
    p_conf = (struct locater_enclosure_conf_ext_fmt_s *)&p_conf_ext[1];

    printf("enclosure_ext, conf enable: 0x%02x\n", p_conf->is_enable);
    printf("enclosure_ext, conf enable: 0x%02x\n", p_conf->accurate);
    printf("enclosure_ext, conf center_dimension_value: 0x%04x\n", p_conf->center_dimension_value);
    printf("enclosure_ext, conf center_longitude_value: 0x%04x\n", p_conf->center_longitude_value);
    printf("enclosure_ext, conf comparative_value: 0x%08llx\n", p_conf->comparative_value);
    printf("enclosure_ext, conf start_time: 0x%04x\n", p_conf->start_time);
    printf("enclosure_ext, conf stop_time: 0x%04x\n", p_conf->stop_time);
    printf("enclosure_ext, conf type: 0x%04x\n", p_conf->type);

    memcpy(&s_locater_enclosure_conf.ext, p_conf, sizeof(struct locater_enclosure_conf_ext_fmt_s));

    return 0;
}


/**
 * @brief 主程序
 * 
 * @param arg  [in ] 
 * 
 * @details 
 */
static void locater_uart_app_main_task(void *arg)
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
    unsigned int collision_alarm = 0;
    unsigned int battery_level = 0;
    unsigned int temperature = 0;
    bool is_need_upload_service = false;
    char mqtt_topic_str_buf[256] = {0}, mqtt_payload_str_buf[256] = {0};
    unsigned int mqtt_payload_qos = 0, mqtt_payload_retain = 0, mqtt_flags = 0;
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};

    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

    while (true) {
    // 处理4G模块上电过程
LOCATOR_UART_FSM_COM_STEP_ENTRY(0) {
        time_t x;
        printf("\n\n");
        printf("firware_version: %s, build_time: %s %s\n", sp_locater_firware_version, __DATE__, __TIME__);
        
        time(&x);
        printf("app_main, time = %ld, %ld\n", time(NULL), x);

        printf("//////////////////%04d//////////////////\n", idx++);
        s_is_locater_device_ready = false;

        printf("app_main, close 4g module\n");
        // 不给4G模块供电
        v_task_delay(5000 / port_tick_period_ms);
        gpio_set_level(GPIO_OUTPUT_UART_CAT1_EN, 0);
        gpio_set_level(GPIO_OUTPUT_UART_CAT1_POWER, 0);
        v_task_delay(1000 / port_tick_rate_ms);

        // 给4G模块供电
        gpio_set_level(GPIO_OUTPUT_UART_CAT1_EN, 1);
        v_task_delay(1000 / port_tick_rate_ms);

        // 4G模块上电
        printf("app_main, open 4g module\n");
        gpio_set_level(GPIO_OUTPUT_UART_CAT1_POWER, 1);
        v_task_delay(1000 / port_tick_rate_ms);
        gpio_set_level(GPIO_OUTPUT_UART_CAT1_POWER, 0);
        v_task_delay(1000 / port_tick_rate_ms);

        // 习惯性的输入回车换行，检查上电是否OK
        p_at_cmd_str = "\r\n\r\n";
        uart_write_bytes(UART_NUM_1, p_at_cmd_str, strlen(p_at_cmd_str));

        printf("app_main, wait 4g module power on\n");
        ret = locater_uart_get_pb_done(&s_locater_atres_pbdone);
        printf("pb_done, detect pb_done ready: %d (%d)\n", s_locater_atres_pbdone.is_power_on_done, ret);
        if (ret < 0 || !s_locater_atres_pbdone.is_power_on_done) {
            printf("pb_done, retry\n");
            continue;
        }

        step_bakup = 0;
        step++;
      }

    // 检查网络情况
LOCATOR_UART_FSM_COM_STEP_ENTRY(1) {
        // 获取4G模块的产品ID号
        ret = locater_uart_get_ati();

        // 检查SIM在位情况
        printf("\r\n\r\n");
        printf("app_main, check sim card is ready\n");
        ret = locater_uart_get_cpin(&s_locater_atres_cpin);
        printf("cpin, detect sim ready: %d (%d)\n", s_locater_atres_cpin.is_ready, ret);
        if (!s_locater_atres_cpin.is_ready) {
            printf("cpin, retry!\n");
            v_task_delay(5000 / port_tick_period_ms);
            continue;
        }

        // 检查信号质量
        printf("\r\n\r\n");
        retry_cnt = 500;
        ret = locater_uart_get_csq(&s_locater_csq);
        printf("csq, detect query signal quality, rssi: %d, ber: %d (ret: %d)\n", s_locater_csq.rssi, s_locater_csq.ber, ret);
        if (ret || (s_locater_csq.rssi < 0) || (s_locater_csq.rssi > 31)) {
            printf("csq, retry!\n");
            v_task_delay(5000 / port_tick_period_ms);
            continue;
        }

        printf("\r\n\r\n");
        ret = locater_uart_get_cpsi(&s_locater_atres_cpsi);
        printf("get_cpsi, detect cpsi, (ret: %d)\n", ret);

        // 检查网络注册情况
        printf("\r\n\r\n");
        ret = locater_uart_get_cereg(&s_locater_atres_cereg);
        printf("get_cereg, detect n: %d, status: %d\n", s_locater_atres_cereg.result_code, s_locater_atres_cereg.status);
        if (s_locater_atres_cereg.result_code == 0 && s_locater_atres_cereg.status == 1)
            printf("get_cereg, registered home network\n");
        else if (retry_cnt) {
            retry_cnt--;
            printf("get_cereg, retry %d!\n", retry_cnt);
            v_task_delay(5000 / port_tick_period_ms);
            continue;
        }
        else {
            printf("get_cereg, final fail after retry!\n", ret);
            continue;
        }

        // 设置移动卡接入网络
        printf("\r\n\r\n");
        ret = locater_uart_set_cgdcont_cmnet();
        printf("cgdcont_cmnet, detect cgdcont cmnet ret: %d\n", ret);
        if (ret) {
            printf("cgdcont_cmnet, retry\n", ret);
            continue;
        }

        printf("\r\n\r\n");
        retry_cnt = 500;
        ret = locater_uart_get_cgact(&s_locater_atres_cgact);
        printf("get_cgact, detect get cgact ret: %d\n", ret);
        if ((ret && retry_cnt) || s_locater_atres_cgact.status != 1) {
            retry_cnt--;
            printf("get_cgact, retry: %d\n", retry_cnt);
            v_task_delay(5000 / port_tick_period_ms);
            continue;
        }
        else if (ret && !retry_cnt) {
            printf("get_cgact, final fail after retry!\n", ret);
            continue;
        }

        // CGACT激活网络。AT+CGACT之前先查询一下，返回为1才执行CGACT
        printf("\r\n\r\n");
        ret = locater_uart_set_cgact();
        printf("cgact, detect set cgact ret: %d\n", ret);
        if (ret) {
            printf("set_cgact, retry!\n", ret);
            continue;
        }

        if (step_bakup)
            step = step_bakup;
        else
            step++;
      }

    // 订阅消息
LOCATOR_UART_FSM_COM_STEP_ENTRY(2) {
        struct timeval stime = {0};
        struct tm info = {0};
        struct locater_local_time_info_fmt_s local_time_info = {0};
        unsigned int time_stamp = 0;
        time_t x;

        printf("\r\n\r\n");

        ///< 获取IMEI号
        ret = locater_uart_get_imei_str(&s_locater_atres_cgsn);
        printf("imei_str, str result: %s\n", s_locater_atres_cgsn.imei_buff);
        locater_assert(ret);

        ///< 获取SNTP时间
        ret = locater_uart_get_sntp_time(&local_time_info);
        locater_assert(ret);
        ///< 配置时区
        setenv("TZ", "CST-8", 1);
        tzset();

        info.tm_year = local_time_info.year - 1900;
        info.tm_mon = local_time_info.month - 1;
        info.tm_mday = local_time_info.day;
        info.tm_hour = local_time_info.hour;
        info.tm_min = local_time_info.minute;
        info.tm_sec = local_time_info.second;
        info.tm_isdst = -1;
        time_stamp = mktime(&info);
        printf("app_main, time_stamp from 1900: %u\n", time_stamp);
        // 设置本地时间
        stime.tv_sec = time_stamp;
        settimeofday(&stime, NULL);
        // 获取本地时间
        time(&x);
        printf("app_main, time: %u, x: %u\n", time(NULL), x);

        locater_uart_get_device_serial_by_imei(s_locater_atres_cgsn.imei_64, s_locater_device_serial_buff, \
            sizeof(s_locater_device_serial_buff));
        printf("device_serial, str result: %s\n", s_locater_device_serial_buff);

        printf("\r\n\r\n");
        printf("app_main, open mqtt\n");
        ret = locater_uart_set_mqtt_open(0);

        //  MQTT连接
        printf("\r\n\r\n");
        printf("app_main, connect mqtt server\n");
        ret = locater_uart_set_mqtt_connect();
        printf("mqtt_connect, detect mqtt connect result: %d\n", ret);
        if (ret) {
            printf("mqtt_connect, retry mqtt connect\n");
            continue;
        }

        // 上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
        printf("\r\n\r\n");
        printf("app_main, sub mqtt topic\n");
        sprintf(mqtt_topic_str_buf, "%s/#", p_serial);
        ret = locater_uart_set_mqtt_sub(client_handle, mqtt_topic_str_buf, 1, 0, 0);

        // 上线订阅will消息， Will消息设置：消息主题：SERIAL+“D/0/0” ，Payload=‘0’，QoS=1，Retain=1。
        printf("\r\n\r\n");
        sprintf(mqtt_topic_str_buf, "%s/#", p_serial);
        sprintf(mqtt_payload_str_buf, "%s", "0");
        mqtt_payload_qos = 1;
        mqtt_payload_retain = 1;
        ret = locater_uart_set_mqtt_will(client_handle, mqtt_topic_str_buf, mqtt_payload_str_buf, \
            mqtt_payload_qos, mqtt_payload_retain, 0);

        // 上线通知服务器
        printf("\r\n\r\n");
        printf("app_main, pub mqtt device online\n");
        sprintf(mqtt_topic_str_buf, "%sD/0/0", p_serial);
        ret = locater_uart_set_mqtt_pub(client_handle, mqtt_topic_str_buf, "1", 1, 1, 0);

        printf("app_main, clear uart recv buff data\n");
        uart_set_buff_clean();

        s_is_locater_uart_continuous_positioning = true;
        retry_cnt = 0;
        s_is_locater_device_ready = true;
        step++;
      }

    /*
    *  主程序处理消息事件，对于一些事件进行处理
    */
LOCATOR_UART_FSM_COM_STEP_ENTRY(3) {
        struct uart_event_que_s *uart_event = NULL;
        unsigned int event_count = 0, split_count = 0, split_index = 0, payload_len = 0;
        unsigned int client_idx = 0, msg_id = 0, recv_id = 0, err_code = 0;
        char *p_topic = NULL, *p_payload = NULL;
        unsigned int str_len = 0;

        event_count = uart_event_get_busy_item_count();
        if (!event_count) {
            printf("main_event_centre, wait event...\n");
            v_task_delay(1000 / port_tick_period_ms);
            continue;
        }

        uart_event = uart_event_get_item();
        if (!uart_event) {
            printf("main_event_centre, detect event_event is error\n");
            v_task_delay(1000 / port_tick_period_ms);
            continue;
        }

        /*
        *  [1] +QMTSTAT: <client_idx>,<err_code>当 MQTT 链路层状态改变，客户端会断开MQTT 连接并上报 URC。
        *  [2] +QMTRECV: <client_idx>,<msgid>,<topic>[,<payload_len>],<payload>当客户端接收到 MQTT 服务器的数据包会上报 URC。 
        *  [3] +QMTRECV: <client_idx>,<recv_id>当从 MQTT 服务器接收的消息存储到缓存时上报 URC。 
        *  [4] +QMTPING: <client_idx>,<result>当 MQTT 链层状态变化时，客户端会关闭MQTT 连接并上报此 URC。
        */
        if (!strncmp(uart_event->content, "+QMTRECV:", 9)) {
            split_count = locater_uart_split_str_bychar(uart_event->content, ",:", \
                    split_array, ARRAY_LEN(split_array));
            for (split_index = 0; split_index < split_count; split_index++) {
                printf("main_event_centre, elem_%02d: %s\n", split_index, split_array[split_index].data);
            }

            if (split_count == 3) {
                printf("main_event_centre, detect msg type: 3\n");
                client_idx = atoi(split_array[1].data);
                recv_id = atoi(split_array[2].data);
            }
            else if (split_count == 5) {
                printf("main_event_centre, detect msg type: 2, no payload len\n");
                client_idx = atoi(split_array[1].data);
                msg_id = atoi(split_array[2].data);
                p_topic = split_array[3].data;
                payload_len = 0;
                p_payload = split_array[4].data;
            }
            else if (split_count == 6) {
                printf("main_event_centre, detect msg type: 1, with payload len\n");
                client_idx = atoi(split_array[1].data);
                msg_id = atoi(split_array[2].data);
                p_topic = split_array[3].data;
                payload_len = split_array[4].data;
                p_payload = split_array[5].data;
            }

            // 解析这个消息内容，APP上线/下线通知
            sprintf(mqtt_topic_str_buf, "\"%s/0/5\"", p_serial);
            if (!strncmp(p_topic, mqtt_topic_str_buf, strlen(mqtt_topic_str_buf))) {
                struct locator_uart_protocol_app_on_line_fmt_s app_online_info;
                app_online_info.is_on_line = p_payload[0];
                app_online_info.app_idx = locater_code86_to_int(&p_payload[1], 4);
                printf("main_event_centre, app online info: 0x%02x, app_new_idx: 0x%04x, app_old_idx: 0x%04x\n", \
                        app_online_info.is_on_line, app_online_info.app_idx, s_locater_uart_curr_app_idx);
                /*
                *  记录App在线情况。
                *  所有的业务都需要检测是否APP上线，以及app_idx是否发生变化！
                */
                if (app_online_info.is_on_line == 0x31) {
                    s_is_locater_uart_curr_app_online = true;
                    s_locater_uart_curr_app_idx = app_online_info.app_idx;
                }
                else {
                    s_is_locater_uart_curr_app_online = false;
                }
                continue;
            }

            // 接收休眠指令消息，QOS=1
            sprintf(mqtt_topic_str_buf, "\"%s/0/6\"", p_serial);
            if (!strncmp(p_topic, mqtt_topic_str_buf, strlen(mqtt_topic_str_buf))) {
                printf("main_event_centre, device need suspend\n");
                s_is_locater_uart_device_need_suspend = true;
                continue;
            }

            // 接收退出休眠指令消息，QOS=1
            sprintf(mqtt_topic_str_buf, "\"%s/5/6\"", p_serial);
            if (!strncmp(p_topic, mqtt_topic_str_buf, strlen(mqtt_topic_str_buf))) {
                printf("main_event_centre, device need wakeup\n");
                s_is_locater_uart_device_need_suspend = false;
                continue;
            }

            // 接收连续定位指令，QOS=0
            sprintf(mqtt_topic_str_buf, "\"%s/07\"", p_serial);
            if (!strncmp(p_topic, mqtt_topic_str_buf, strlen(mqtt_topic_str_buf))) {
                printf("main_event_centre, device open continuous positioning\n");
                s_is_locater_uart_continuous_positioning = true;
                continue;
            }

            // 接收连续定位指令，QOS=0
            sprintf(mqtt_topic_str_buf, "\"%s/57\"", p_serial);
            if (!strncmp(p_topic, mqtt_topic_str_buf, strlen(mqtt_topic_str_buf))) {
                printf("main_event_centre, device close continuous positioning\n");
                s_is_locater_uart_continuous_positioning = false;
                continue;
            }

            // 接收Factor_T参数:用于GPS定位时，计算比较距离，QOS=0
            sprintf(mqtt_topic_str_buf, "\"%s/9\"", p_serial);
            if (!strncmp(p_topic, mqtt_topic_str_buf, strlen(mqtt_topic_str_buf))) {
                printf("main_event_centre, device need compare distances\n");
                s_is_locater_uart_need_compare_distances = true;
                continue;
            }

            // 单次定位（立即定位一次） QOS=0
            sprintf(mqtt_topic_str_buf, "\"%s/A\"", p_serial);
            if (!strncmp(p_topic, mqtt_topic_str_buf, strlen(mqtt_topic_str_buf))) {
                printf("main_event_centre, device open once positioning\n");
                s_is_locater_uart_once_positioning = true;
                continue;
            }

            // 10.1，WIFI扫描
            sprintf(mqtt_topic_str_buf, "\"%s/0N\"", p_serial);
            if (!strncmp(p_topic, mqtt_topic_str_buf, strlen(mqtt_topic_str_buf))) {
                printf("main_event_centre, device need wifi scanning\n");
                s_is_locater_uart_need_wifi_scan_rssi_by_mac_array = true;
                continue;
            }

            // 10.3，收到手机端下发的wifi的mac列表
            sprintf(mqtt_topic_str_buf, "\"%s/0/8N\"", p_serial);
            if (!strncmp(p_topic, mqtt_topic_str_buf, strlen(mqtt_topic_str_buf))) {
                printf("main_event_centre, device recv wifi mac list\n");
                locater_uart_app_set_wifi_scan_mac_array(p_payload);
                continue;
            }

            // 10.3，收到手机端下发指令，针对某个wifi的mac扫描wifi信号
            sprintf(mqtt_topic_str_buf, "%s", "3N");
            if (strstr(p_topic, mqtt_topic_str_buf)) {
                printf("main_event_centre, device recv wifi mac\n");
                s_is_locater_uart_need_wifi_scan_rssi_by_mac = true;
                s_is_locater_uart_need_wifi_scan_rssi_by_mac_app_idx = *(unsigned int *)p_payload;
                continue;
            }

            // 11.1，OTA升级，主题的随后一个字节为J时，认为是OTA升级
            str_len = strlen(p_topic);
            if (str_len >= 1 && p_topic[str_len - 1] == 'J') {
                printf("main_event_centre, device ota command\n");
                locater_uart_app_set_ota_conf(p_topic, p_payload);
                s_is_locater_uart_need_ota = true;
                continue;
            }

            sprintf(mqtt_topic_str_buf, "\"%s/0/1m\"", p_serial);
            if (!strncmp(p_topic, mqtt_topic_str_buf, strlen(mqtt_topic_str_buf))) {
                printf("main_event_centre, device recv user setting\n");
                locater_uart_app_set_user_setting(p_payload);
                continue;
            }

            // 设置围栏，关键字“5m”
            str_len = strlen(p_topic);
            if (str_len >= 2 && p_topic[str_len - 2] == 'm' && p_topic[str_len - 3] == '5') {
                printf("main_event_centre, device set enclosure\n");
                locater_uart_app_set_enclosure(p_topic);
                locater_uart_app_set_enclosure_ext(p_payload);
                continue;
            }

        }
        else if (!strncmp(uart_event->content, "+QMTSTAT:", 9)) {
            split_count = locater_uart_split_str_bychar(uart_event->content, ",:", split_array, \
                    ARRAY_LEN(atres_array));
            for (split_index = 0; split_index < split_count; split_index++) {
                printf("main_event_centre, elem_%02d: %s\n", split_index, split_array[split_index].data);
            }
            client_idx = atoi(split_array[1].data);
            err_code = atoi(split_array[2].data);
        }
        else {
            printf("main_event_centre, no msg %04d!\n", retry_cnt);
        }

        /*
        *  间隔几分钟检查一次网络
        */
        v_task_delay(1000 / port_tick_period_ms);
        continue;
      }
    }
}

int locater_uart_init(void)
{
    int ret;

    ret = pthread_mutex_init(&s_locater_uart_4g_module_mutex, 0);

    xTaskCreate(locater_uart_app_main_task, "app_main_task", 1024 * 30, NULL, config_max_priorities - 1, NULL);
    xTaskCreate(locater_uart_app_location_misc_task, "app_location_misc_task", 1024 * 30, NULL, config_max_priorities - 1, NULL);
    xTaskCreate(locater_uart_app_wifi_misc_task, "app_wifi_misc_task", 1024 * 30, NULL, config_max_priorities - 1, NULL);
    xTaskCreate(locater_uart_app_ota_misc_task, "app_ota_misc_task", 1024 * 30, NULL, config_max_priorities - 1, NULL);
    // xTaskCreate(locater_uart_app_temp_misc_task, "app_temp_misc_task", 1024 * 30, NULL, config_max_priorities - 1, NULL);

    return 0;
}