#define LOCATER_MAX_AT_RESP_LEN (215)
#define LOCATOR_UART_FSM_COM_STEP_ENTRY(s)   if (step == (s))
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

int locater_uart_init(void);