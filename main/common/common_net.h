#ifndef __COMMON_NET_H_
#define __COMMON_NET_H_

#include "common.h"
#include "esp_event.h"
#include "esp_dpp.h"

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void dpp_enrollee_event_cb(esp_supp_dpp_event_t event, void *data);
void dpp_enrollee_init(void);
void nvs_init();
int wifi_init();
int wifi_connect();
int wifi_scan();
void sys_ota();
//int32_t storage_read(char *key);
//void storage_write(char *key, int value);

#endif


