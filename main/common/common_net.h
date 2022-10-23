#ifndef __COMMON_NET_H_
#define __COMMON_NET_H_

#include "common.h"
#include "esp_event.h"
#include "esp_dpp.h"
//scan
#define DEFAULT_SCAN_LIST_SIZE 10
extern char strWifiMac[DEFAULT_SCAN_LIST_SIZE][32];

typedef struct{
	int time;
	int round;
	int max_bssid_num;
	int scan_timeout;
	int priority;
	char ecn[12];
	char ssid[48];
	int rssi;
	char mac[24];
	int channel;
}STRU_WIFI_INFO;
STRU_WIFI_INFO stWifi[12];
void nvs_init();
int wifi_init();
int wifi_connect();
int wifi_scan();
int wifi_set_user_pwd(char *struser, char *strpwd);
int sys_ota();
int wifi_get_info(STRU_WIFI_INFO **stRet);
//int32_t storage_read(char *key);
//void storage_write(char *key, int value);
int wifi_test();
#endif


