#ifndef __COMMON_NET_H_
#define __COMMON_NET_H_

#include "common.h"
#include "esp_event.h"
#include "esp_dpp.h"
//scan
#define DEFAULT_SCAN_LIST_SIZE 10
extern char strWifiMac[DEFAULT_SCAN_LIST_SIZE][32];

void nvs_init();
int wifi_init();
int wifi_connect();
int wifi_scan();
int sys_ota();
//int32_t storage_read(char *key);
//void storage_write(char *key, int value);

#endif


