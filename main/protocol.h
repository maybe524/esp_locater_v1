#pragma once

/**
 * @brief 3.1　APP上线消息：消息主题为SERIAL/+5，payload=’1’+4字节APP ID，
 *      每个APP客户端上线、离线的APP ID相同， 但和下次登录时的APP ID可能不同；通过消息ID排除重复消息；
 */
typedef struct locator_uart_protocol_app_on_line_fmt_s {
    char is_on_line;
    unsigned int app_idx;
} locator_uart_protocol_app_on_line_fmt_t;