#define LOCATER_MAX_AT_RESP_LEN (215)
#define LOCATOR_UART_FSM_COM_STEP_ENTRY(s)   if (step == (s))
// #define LOCATOR_DEBUG_MODE
#define ARRAY_LEN(a)    (sizeof(a) / sizeof(a[0]))
#define LOCATER_IMEI_SIZE   (15)
#define locater_assert(cond)    do { \
                if (cond) { \
                    unsigned int ____timeout = 0; \
                    while (1) {  \
                        if (____timeout > 10000000) \
                            break;  \
                        printf("assert, bug! %s %d\n", __func__, __LINE__); \
                        v_task_delay(1000 / port_tick_period_ms); \
                        ____timeout++; \
                    } \
                } \
            } while (0)

#define LOCATER_DEVICE_SERIAL_SIZE  (8)

typedef enum {
    LOCATER_SEND_AT_CMD_WITHOUT_CLEAN   = (1 << 0),

    // 直接等待结果
    LOCATER_SEND_AT_CMD_DIRECT_WAIT_RESULT     = (1 << 1),
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
    unsigned int data_len;
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

#define LOCATER_ENCLOSURE_CNF_NAME_LEN  256

typedef struct locater_enclosure_conf_ext_fmt_s {
    union {
        char byte[27];
    
        struct {
            // 1 字节围栏开关（0=不启用，1=启用）
            char is_enable;
            // 1 字节围栏精准/标准设置（0=标准，1=精准）
            char accurate;
            // 2 字节开始时间分钟
            short start_time;
            short stop_time;
            // 1 字节围栏类型，1=出栏报警，2=入栏报警，3=跨栏报警
            char type;
            // 4 字节围栏中心维度值，为角度值分*10^5;
            unsigned int center_dimension_value;
            // 4 字节围栏中心经度值，为角度值分*10^5，如 30.30º=（30*60+30）*100000=183000000
            unsigned int center_longitude_value;
            // 8 字节围栏半径比较值 S, 如果(△Lat^2*Factor_T + △Lon^2*10^6)>S,表示为围栏外面，反之，表示在围栏里面
            unsigned long long comparative_value;
        };
    };
} locater_enclosure_conf_ext_fmt_t;

typedef struct locater_enclosure_conf_fmt_s {
    char name[LOCATER_ENCLOSURE_CNF_NAME_LEN];
    char idx;
    struct locater_enclosure_conf_ext_fmt_s ext;
} locater_enclosure_conf_fmt_t;

typedef struct locater_location_utc_info_fmt_s {
    unsigned short hh;
    unsigned short mm;
    unsigned short ss_high;
    unsigned char resver;
    unsigned short ss_low;
} locater_location_utc_info_fmt_t;

typedef struct locater_location_info_fmt_s {
    unsigned int time_stamp;
    int latitude;
    int longitude;
} locater_location_info_fmt_t;

typedef struct locater_local_time_info_fmt_s {
    unsigned int zone;
    unsigned int year;
    unsigned int month;
    unsigned int day;
    unsigned int hour;
    unsigned int minute;
    unsigned int second;
} locater_local_time_info_fmt_t;

int locater_uart_init(void);