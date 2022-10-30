#ifndef __COMMON_NET_H_
#define __COMMON_NET_H_

#include "common.h"
#include "esp_event.h"
#include "esp_dpp.h"
//scan
#define DEFAULT_SCAN_LIST_SIZE 10
extern char strWifiMac[DEFAULT_SCAN_LIST_SIZE][32];

typedef enum{
	ENUM_OTA_STS_NO_RUNNING = 0,
	ENUM_OTA_STS_BEGIN,
	ENUM_OTA_STS_CONNECTED,
	ENUM_OTA_STS_CHECKED,
	ENUM_OTA_STS_DOWNLOADING,
	ENUM_OTA_STS_DOWNLOAD_FAIL,
	ENUM_OTA_STS_DOWNLOAD_FINISH,
	ENUM_OTA_STS_EARSE_FINISH,
	ENUM_OTA_STS_READY_TO_REBOOT,
	ENUM_OTA_STS_EARSE_FAIL_IMAGE_CORRUPTED,
	ENUM_OTA_STS_EARSE_FAIL,
	ENUM_OTA_STS_DEFAULT,
}ENUM_OTA_STA;

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
int wifi_disconnect();
int wifi_scan();
int wifi_set_user_pwd(char *struser, char *strpwd);
int sys_ota();
int sys_ota_set_url(char *strurl);
int sys_ota_sts_get();
int sys_ota_reboot_set(int _reboot);
int wifi_get_info(STRU_WIFI_INFO **stRet);
//int32_t storage_read(char *key);
//void storage_write(char *key, int value);
#endif


