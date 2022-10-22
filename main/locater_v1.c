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

/*
*  //TBD: 经常发生栈溢出，临时把变量放在外边
*/
static char buff[1024];
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
static unsigned int s_locater_uart_curr_app_idx = 0;
static bool s_is_locater_uart_curr_app_online = false;
static bool s_is_locater_uart_device_need_suspend = false;
static bool s_is_locater_uart_continuous_positioning = false;
static bool s_is_locater_uart_need_compare_distances = false;
static bool s_is_locater_uart_once_positioning = false;
static bool s_is_locater_uart_need_wifi_scan = false;
static pthread_mutex_t s_locater_uart_4g_module_mutex;

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

static int locater_uart_get_locate_info(void)
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

    //获取GNSS版本号
    p_atcmd = "AT+QGPSINFO\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("locate_info, get gnss version failed\n");
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

    //关闭GNSS
    p_atcmd = "AT+QGPS=0\r\n";
    p_want_ack_str = "OK";
    ret = locater_uart_send_atcmd_2_4g_module(p_atcmd, p_want_ack_str, buff, sizeof(buff), 1000, 0);
    if (ret < 0) {
        printf("locate_info, failed\n");
        return ret;
    }

    ret = -2;
    p_want_ack_str = "+QGPSLOC:";
    for (i = 0; i < at_res_line; i++) {
        printf("wifi_scan, line_%02d, len_%02d: %s\n", i, strlen(atres_array[i].atreq_content), \
            atres_array[i].atreq_content);

        if (strncmp(p_want_ack_str, atres_array[i].atreq_content, strlen(p_want_ack_str)))
            continue;
        
        printf("locate_info, get locate_info: %s\n", atres_array[i].atreq_content);
        ret = 0;
        break;
    }

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

static void locater_uart_main_task(void *arg)
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
    char *p_serial = "ABCABCABC";
    struct locater_atres_fmt_s atres_array[32] = {0};
    struct locater_str_split_fmt_s split_array[32] = {0};

    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

    while (true) {
    // 处理4G模块上电过程
LOCATOR_UART_FSM_COM_STEP_ENTRY(0) {
        printf("\n\n");
        printf("//////////////////%04d//////////////////\n", idx++);
        s_is_locater_online = false;

        printf("main, close 4g module\n");
        // 不给4G模块供电
        v_task_delay(5000 / port_tick_period_ms);
        gpio_set_level(GPIO_OUTPUT_UART_CAT1_EN, 0);
        gpio_set_level(GPIO_OUTPUT_UART_CAT1_POWER, 0);
        v_task_delay(1000 / port_tick_rate_ms);

        // 给4G模块供电
        gpio_set_level(GPIO_OUTPUT_UART_CAT1_EN, 1);
        v_task_delay(1000 / port_tick_rate_ms);

        // 4G模块上电
        printf("main, open 4g module\n");
        gpio_set_level(GPIO_OUTPUT_UART_CAT1_POWER, 1);
        v_task_delay(1000 / port_tick_rate_ms);
        gpio_set_level(GPIO_OUTPUT_UART_CAT1_POWER, 0);
        v_task_delay(1000 / port_tick_rate_ms);

        // 习惯性的输入回车换行，检查上电是否OK
        p_at_cmd_str = "\r\n\r\n";
        uart_write_bytes(UART_NUM_1, p_at_cmd_str, strlen(p_at_cmd_str));

        printf("main, wait 4g module power on\n");
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
        printf("main, check sim card is ready\n");
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
        printf("\r\n\r\n");
        ret = locater_uart_get_imei_str(&s_locater_atres_cgsn);

        printf("imei_str, str result: %s\n", s_locater_atres_cgsn.imei_buff);
        locater_assert(ret);
        locater_uart_get_device_serial_by_imei(s_locater_atres_cgsn.imei_64, s_locater_device_serial_buff, \
            sizeof(s_locater_device_serial_buff));
        printf("device_serial, str result: %s\n", s_locater_device_serial_buff);

        printf("\r\n\r\n");
        printf("main, open mqtt\n");
        ret = locater_uart_set_mqtt_open(0);

        //  MQTT连接
        printf("\r\n\r\n");
        printf("main, connect mqtt server\n");
        ret = locater_uart_set_mqtt_connect();
        printf("mqtt_connect, detect mqtt connect result: %d\n", ret);
        if (ret) {
            printf("mqtt_connect, retry mqtt connect\n");
            continue;
        }

        // 上线订阅主题：订阅主题为SERIAL/#，QoS=1，Retain=0。
        printf("\r\n\r\n");
        printf("main, sub mqtt topic\n");
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
        printf("main, pub mqtt device online\n");
        sprintf(mqtt_topic_str_buf, "%sD/0/0", p_serial);
        ret = locater_uart_set_mqtt_pub(client_handle, mqtt_topic_str_buf, "1", 1, 1, 0);

        printf("main, clear uart recv buff data\n");
        uart_set_buff_clean();


        // locater_uart_get_wifi_list();
        locater_uart_get_locate_info();
        retry_cnt = 0;
        s_is_locater_online = true;
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

        event_count = uart_event_get_busy_item_count();
        if (!event_count) {
            printf("main_event_centre, detect curr event count is zero\n");
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
        *  [1] +QMTSTAT: <client_idx>,<err_code>当 MQTT 链路层状态改变，客户端会断开
        *  MQTT 连接并上报 URC。
        *  [2] +QMTRECV: <client_idx>,<msgid>,<topic>[,<payload_len>],<payload>
        *  当客户端接收到 MQTT 服务器的数据包会上报 URC。 
        *  [3] +QMTRECV: <client_idx>,<recv_id>
        *  当从 MQTT 服务器接收的消息存储到缓存
        *  时上报 URC。 [4] +QMTPING: <client_idx>,<result>
        *  当 MQTT 链层状态变化时，客户端会关闭MQTT 连接并上报此 URC。
        */
        if (!strncmp(uart_event->content, "+QMTRECV:", 9)) {
            split_count = locater_uart_split_str_bychar(uart_event->content, ",:", \
                    split_array, ARRAY_LEN(atres_array));
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
                struct locator_uart_protocol_app_on_line_fmt_s *p_app_online_fmt = \
                        (struct locator_uart_protocol_app_on_line_fmt_s *)&p_payload[1];
                printf("main_event_centre, app online info: 0x%02x, app_new_idx: 0x%04x, app_old_idx: 0x%04x\n", \
                        p_app_online_fmt->is_on_line, p_app_online_fmt->app_idx, s_locater_uart_curr_app_idx);
                /*
                *  记录App在线情况。
                *  所有的业务都需要检测是否APP上线，以及app_idx是否发生变化！
                */
                if (p_app_online_fmt->is_on_line == 0x31) {
                    s_is_locater_uart_curr_app_online = true;
                    s_locater_uart_curr_app_idx = p_app_online_fmt->app_idx;
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

            // 接收不安比连续定位指令，QOS=0
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

            // WIFI扫描
            sprintf(mqtt_topic_str_buf, "\"%s/X0N\"", p_serial);
            if (!strncmp(p_topic, mqtt_topic_str_buf, strlen(mqtt_topic_str_buf))) {
                printf("main_event_centre, device need wifi scanning\n");
                s_is_locater_uart_need_wifi_scan = true;
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

    xTaskCreate(locater_uart_main_task, "locater_uart_main_task", 1024 * 30, NULL, configMAX_PRIORITIES - 1, NULL);

    return 0;
}