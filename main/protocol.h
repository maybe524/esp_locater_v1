#pragma once

#pragma pack (1)

/**
 * @brief 3.1　APP上线消息：消息主题为SERIAL/+5，payload=’1’+4字节APP ID，
 *      每个APP客户端上线、离线的APP ID相同， 但和下次登录时的APP ID可能不同；通过消息ID排除重复消息；
 */
typedef struct locator_uart_protocol_app_on_line_fmt_s {
    char is_on_line;
    unsigned int app_idx;
} locator_uart_protocol_app_on_line_fmt_t;


/**
 * @brief 10.3 接收 WIFI 热点设置消息：消息主题为 SERIAL/0/8N，消息 payload =4 字节保留数据+1 字节热点数量+【1
 *  字节热点编号+17 字节 MAC】*热点数量+（保留字节，长度待定），热点编号依次为 1、2、3、。。。。。 ，QOS=1； 接
 *  收该消息后，删除以前的 wifi 设置数据
 * @details 
 */
typedef struct locator_uart_protocol_app_set_wifi_mac_array_payload_fmt_s {
    char reserv[4];
    char wifi_count;
    struct {
        char idx;
        char mac[17];
    } idx_mac_array[64];

} locator_uart_protocol_app_set_wifi_mac_array_payload_fmt_s;

typedef struct locator_uart_protocol_dev_set_wifi_rssi_array_payload_fmt_s {
    char wifi_count;
    struct {
        char idx;
        char rssi;
    } idx_rssi_array[64];

} locator_uart_protocol_dev_set_wifi_rssi_array_payload_fmt_t;

typedef struct locator_uart_protocol_dev_set_wifi_rssi_payload_fmt_s {
    unsigned int id;
    char rssi;
} locator_uart_protocol_dev_set_wifi_rssi_payload_fmt_t;

typedef struct locator_uart_protocol_app_set_ota_conf_payload_fmt_s {
    unsigned int id;
    char url_len;
    char url[1024];
} locator_uart_protocol_app_set_ota_conf_payload_fmt_t;

typedef struct locator_uart_protocol_app_set_user_setting_payload_fmt_s {
    char reserv1[4];
    // 1
    char is_temp_low_alarm_enable;
    // 2
    short temp_low_threshold;
    // 3
    char is_temp_high_alarm_enable;
    // 4
    short temp_high_threshold;
    // 5
    char is_touch_alarm_enable;
    // 6
    short touch_alarm_in_0_1_g;
    // 7
    char is_steps_enable;
    // 8
    char is_temp_alarm_enable;
    // 9
    char is_led_enable;
    // 10
    char reserv2[4];
} locator_uart_protocol_app_set_user_setting_payload_fmt_t;

typedef struct locator_uart_protocol_dev_upload_location_mult_payload_fmt_s {
    char type;
    unsigned int time_stamp;
    unsigned int latitude;
    unsigned int longitude;
} locator_uart_protocol_dev_upload_location_mult_payload_fmt_t;

#pragma pack ()