//read and write storage
#include "nvs_flash.h"
#include "nvs.h"

//wifi
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

//ota
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "../example/common_components/protocol_examples_common/include/protocol_examples_common.h"

#include "esp_eth.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "common.h"
static const char *TAG = "system";
unsigned int gulWifiConnect = 0;
//unsigned int gulRunning = 0;
unsigned int gulOTAReboot = 0;
unsigned int gulOTAsts = 0;


//connect
#define CONFIG_EXAMPLE_CONNECT_WIFI   1
#define CONFIG_EXAMPLE_WIFI_SSID	"CMCC-NtKi"
#define CONFIG_EXAMPLE_WIFI_PASSWORD	"3d687272"

#define WIFI_TRY_TIME 5
#define WIFI_LEN 64
char gstrssid[WIFI_LEN];//CONFIG_EXAMPLE_WIFI_SSID,
char gstrpwd[WIFI_LEN];//CONFIG_EXAMPLE_WIFI_PASSWORD,
unsigned int gulWifiApcnt = 0;
//ota
uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");
//extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
//extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define OTA_URL_SIZE 256
#define ESP_ERR_WIFI_NOT_STARTED 5
//#define CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL "http://192.168.1.6:8080/hello_world.bin"
#define CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL "http://1.117.221.12:8080/ota/hello_world.bin"
#define CONFIG_EXAMPLE_OTA_RECV_TIMEOUT 10000

#define FIRMWARE_URL_LEN 320
char frimwareURL[FIRMWARE_URL_LEN];

/* Common functions for protocol examples, to establish Wi-Fi or Ethernet connection.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */


#ifdef CONFIG_EXAMPLE_CONNECT_IPV6
#define MAX_IP6_ADDRS_PER_NETIF (5)
#define NR_OF_IP_ADDRESSES_TO_WAIT_FOR (s_active_interfaces*2)

#if defined(CONFIG_EXAMPLE_CONNECT_IPV6_PREF_LOCAL_LINK)
#define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE ESP_IP6_ADDR_IS_LINK_LOCAL
#elif defined(CONFIG_EXAMPLE_CONNECT_IPV6_PREF_GLOBAL)
#define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE ESP_IP6_ADDR_IS_GLOBAL
#elif defined(CONFIG_EXAMPLE_CONNECT_IPV6_PREF_SITE_LOCAL)
#define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE ESP_IP6_ADDR_IS_SITE_LOCAL
#elif defined(CONFIG_EXAMPLE_CONNECT_IPV6_PREF_UNIQUE_LOCAL)
#define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE ESP_IP6_ADDR_IS_UNIQUE_LOCAL
#endif // if-elif CONFIG_EXAMPLE_CONNECT_IPV6_PREF_...

#else
#define NR_OF_IP_ADDRESSES_TO_WAIT_FOR (s_active_interfaces)
#endif

#define EXAMPLE_DO_CONNECT CONFIG_EXAMPLE_CONNECT_WIFI || CONFIG_EXAMPLE_CONNECT_ETHERNET

#if CONFIG_EXAMPLE_WIFI_SCAN_METHOD_FAST
#define EXAMPLE_WIFI_SCAN_METHOD WIFI_FAST_SCAN
#elif CONFIG_EXAMPLE_WIFI_SCAN_METHOD_ALL_CHANNEL
#define EXAMPLE_WIFI_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#endif

#if CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SIGNAL
#define EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#elif CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SECURITY
#define EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
#endif

#if CONFIG_EXAMPLE_WIFI_AUTH_OPEN
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_EXAMPLE_WIFI_AUTH_WEP
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA2_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA_WPA2_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA2_ENTERPRISE
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_ENTERPRISE
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA3_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA2_WPA3_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WAPI_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

static int s_active_interfaces = 0;
static xSemaphoreHandle s_semph_get_ip_addrs;
static esp_netif_t *s_example_esp_netif = NULL;

#ifdef CONFIG_EXAMPLE_CONNECT_IPV6
static esp_ip6_addr_t s_ipv6_addr;

/* types of ipv6 addresses to be displayed on ipv6 events */
static const char *s_ipv6_addr_types[] = {
    "ESP_IP6_ADDR_IS_UNKNOWN",
    "ESP_IP6_ADDR_IS_GLOBAL",
    "ESP_IP6_ADDR_IS_LINK_LOCAL",
    "ESP_IP6_ADDR_IS_SITE_LOCAL",
    "ESP_IP6_ADDR_IS_UNIQUE_LOCAL",
    "ESP_IP6_ADDR_IS_IPV4_MAPPED_IPV6"
};
#endif

#if CONFIG_EXAMPLE_CONNECT_WIFI
static esp_netif_t *wifi_start(void);
static void wifi_stop(void);
#endif
#if CONFIG_EXAMPLE_CONNECT_ETHERNET
static esp_netif_t *eth_start(void);
static void eth_stop(void);
#endif
static void on_got_ip(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data);
static void on_got_ipv6(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data);
static void on_wifi_disconnect(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
static void on_wifi_connect(void *esp_netif, esp_event_base_t event_base,
                            int32_t event_id, void *event_data);
//返回系统总运行时间。参考system/startup_time
int sys_runtime()
{
	return 0;
}


//
int sys_chipinfo()
{
#if 0
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    //light power mode
    //common_power_enter_light_sleep();

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);

    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
#endif
    return 0;
}

//
void sys_deviceops()
{
	//shutdown


	//reboot

	//cleandata
}


static void print_auth_mode(int authmode)
{
    switch (authmode) {
    case WIFI_AUTH_OPEN:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OPEN");
        break;
    case WIFI_AUTH_WEP:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WEP");
        break;
    case WIFI_AUTH_WPA_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_PSK");
        break;
    case WIFI_AUTH_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA2_ENTERPRISE:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_ENTERPRISE");
        break;
    case WIFI_AUTH_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_PSK");
        break;
    case WIFI_AUTH_WPA2_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_WPA3_PSK");
        break;
    default:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_UNKNOWN");
        break;
    }
}

static void print_cipher_type(int pairwise_cipher, int group_cipher)
{
    switch (pairwise_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_NONE");
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP40");
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP104");
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP");
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_CCMP");
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
        break;
    default:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
        break;
    }

    switch (group_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_NONE");
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP40");
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP104");
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP");
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_CCMP");
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
        break;
    default:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
        break;
    }
}

void nvs_init()
{
	// Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
}

int wifi_init()
{
	static unsigned int gulWifiInit = 0;
	if(!gulWifiInit)
	{
		ESP_ERROR_CHECK(esp_netif_init());
    	ESP_ERROR_CHECK(esp_event_loop_create_default());
    	gulWifiInit = 1;
    	ESP_LOGI(TAG, "Wifi Module Init!");
	}
	else
	{
		ESP_LOGI(TAG, "Wifi Module already Init!");
	}

    return 0;
}
char strWifiMac[DEFAULT_SCAN_LIST_SIZE][32] = {0};
/* Initialize Wi-Fi as sta and set scan method */
int wifi_scan()
{
	printf("Wifi Scan \r\n");
	static esp_netif_t *sta_netif = NULL;
    if(!sta_netif)
    {
    	sta_netif = esp_netif_create_default_wifi_sta();
    	assert(sta_netif);
    }
	if(!gulWifiConnect)
	{
		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		ESP_ERROR_CHECK(esp_wifi_init(&cfg));
		gulWifiConnect = 1;
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
		ESP_ERROR_CHECK(esp_wifi_start());
	}
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    esp_wifi_scan_start(NULL, true);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));//error
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_ERROR_CHECK(esp_wifi_scan_stop());
//    while(1)
//    {
		ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);
		if(ap_count >= DEFAULT_SCAN_LIST_SIZE)
			gulWifiApcnt = DEFAULT_SCAN_LIST_SIZE;
		else
			gulWifiApcnt = ap_count;
		for (int i = 0; i < gulWifiApcnt; i++) {
			ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
			//memcpy(&stWifi[i].ssid, ap_info[i].ssid, strlen(&ap_info[i].ssid[0]));
			strcpy((char *)&stWifi[i].ssid[0], (char *)&ap_info[i].ssid[0]);
			//memcpy(&strWifiMac[i], "%s", ap_info[i].ssid);
			//printf("strWifiMac[%d]=%s", i, strWifiMac[i]);

			printf("%c%c:%c%c:%c%c:%c%c:%c%c:%c%c\0",
					ap_info[i].bssid[0]&0xf0, ap_info[i].bssid[0]&0x0f,
					ap_info[i].bssid[1]&0xf0, ap_info[i].bssid[1]&0x0f,
					ap_info[i].bssid[2]&0xf0, ap_info[i].bssid[2]&0x0f,
					ap_info[i].bssid[3]&0xf0, ap_info[i].bssid[3]&0x0f,
					ap_info[i].bssid[4]&0xf0, ap_info[i].bssid[4]&0x0f,
					ap_info[i].bssid[5]&0xf0, ap_info[i].bssid[5]&0x0f);

			sprintf(stWifi[i].mac, "%c%c:%c%c:%c%c:%c%c:%c%c:%c%c\0",
					ap_info[i].bssid[0]&0xf0, ap_info[i].bssid[0]&0x0f,
					ap_info[i].bssid[1]&0xf0, ap_info[i].bssid[1]&0x0f,
					ap_info[i].bssid[2]&0xf0, ap_info[i].bssid[2]&0x0f,
					ap_info[i].bssid[3]&0xf0, ap_info[i].bssid[3]&0x0f,
					ap_info[i].bssid[4]&0xf0, ap_info[i].bssid[4]&0x0f,
					ap_info[i].bssid[5]&0xf0, ap_info[i].bssid[5]&0x0f);

			//ESP_LOGI(TAG, "BSSID \t\t%s:%s:%s", ap_info[i].bssid[0], ap_info[i].bssid[1],ap_info[i].bssid[2]);
			ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
			ESP_LOGI(TAG, "BSSID \t\t%s", ap_info[i].bssid);
			stWifi[i].rssi = ap_info[i].rssi;
			ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
			print_auth_mode(ap_info[i].authmode);
			if (ap_info[i].authmode != WIFI_AUTH_WEP) {
				print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
			}
			ESP_LOGI(TAG, "Channel \t\t%d\n", ap_info[i].primary);

			//printf("stWifi SSID: %s\n", stWifi[i].ssid);
			ESP_LOGI(TAG, "stWifi RSSI: %d\n", stWifi[i].rssi);
		}
//		vTaskDelay(5000 / portTICK_PERIOD_MS);
//    }

//    esp_err_t err = esp_wifi_stop();
//    if (err == ESP_ERR_WIFI_NOT_INIT) {
//        return;
//    }
	if(!gulWifiConnect)
	{
		//ESP_ERROR_CHECK(esp_wifi_stop());
		//ESP_ERROR_CHECK(esp_wifi_deinit());
		//ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(sta_netif));
		//esp_netif_destroy(sta_netif);
	}
	return 0;
}

void wifi_scan_stop(void)
{
    esp_netif_t *wifi_netif = get_example_netif_from_desc("sta");
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip));
#ifdef CONFIG_EXAMPLE_CONNECT_IPV6
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_GOT_IP6, &on_got_ipv6));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &on_wifi_connect));
#endif
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) {
        return;
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(wifi_netif));
    esp_netif_destroy(wifi_netif);
    s_example_esp_netif = NULL;
}

int wifi_connect()
{
	//ESP_ERROR_CHECK(esp_netif_init());
	//ESP_ERROR_CHECK(esp_event_loop_create_default());

#if EXAMPLE_DO_CONNECT
    if (s_semph_get_ip_addrs != NULL) {
    	ESP_LOGI(TAG, "Wifi is already connnectd.\r\n");
        return ESP_ERR_INVALID_STATE;
    }
#endif
	if(!gulWifiConnect)
	{

		/* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
		 * Read "Establishing Wi-Fi or Ethernet Connection" section in
		 * examples/protocols/README.md for more information about this function.
		*/
		ESP_ERROR_CHECK(example_connect());

		gulWifiConnect = 1;
	}
#if 0
	else //if(example_connect() == ESP_ERR_INVALID_STATE)
	{

		ESP_LOGI(TAG, "Wifi is connecting.\r\n");
		ESP_ERROR_CHECK(example_disconnect());
		vTaskDelay(2000 / portTICK_PERIOD_MS);
		ESP_ERROR_CHECK(example_connect());

	}
#endif
	return 0;
}

#if 1
int32_t storage_read(char *key)
{
//	// Initialize NVS
//	esp_err_t err = nvs_flash_init();
//	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//		// NVS partition was truncated and needs to be erased
//		// Retry nvs_flash_init
//		ESP_ERROR_CHECK(nvs_flash_erase());
//		err = nvs_flash_init();
//	}
//	ESP_ERROR_CHECK( err );

	int32_t value = 0; // value will default to 0, if not set yet in NVS
	esp_err_t err;

	// Open
	printf("\n");
	printf("Opening Non-Volatile Storage (NVS) handle... ");
	nvs_handle_t my_handle;
	err = nvs_open("storage", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
		printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	} else {
		printf("Done\n");

		// Read
		printf("Reading %s from NVS ... ", key);

		err = nvs_get_i32(my_handle, key, &value);
		switch (err) {
			case ESP_OK:
				printf("Done\n");
				printf("%S = %d\n", key, value);
				break;
			case ESP_ERR_NVS_NOT_FOUND:
				printf("The value is not initialized yet!\n");
				break;
			default :
				printf("Error (%s) reading!\n", esp_err_to_name(err));
		}

		// Close
		nvs_close(my_handle);
	}

	printf("\n");
	return value;
}

void storage_write(char *key, int value)
{
	// Initialize NVS
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// NVS partition was truncated and needs to be erased
		// Retry nvs_flash_init
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK( err );

	// Open
	printf("\n");
	printf("Opening Non-Volatile Storage (NVS) handle... ");
	nvs_handle_t my_handle;
	err = nvs_open("storage", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
		printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	} else {
		printf("Done\n");

		//int32_t value = 0; // value will default to 0, if not set yet in NVS

		// Write
		printf("Updating %s to %d in NVS ... ", key, value);

		//restart_counter++;
		err = nvs_set_i32(my_handle, key, value);
		printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

		// Commit written value.
		// After setting any values, nvs_commit() must be called to ensure changes are written
		// to flash storage. Implementations may write to storage at other times,
		// but this is not guaranteed.
		printf("Committing updates in NVS ... ");
		err = nvs_commit(my_handle);
		printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

		// Close
		nvs_close(my_handle);
	}

	printf("\n");
}
#endif

/**
 * @brief Checks the netif description if it contains specified prefix.
 * All netifs created withing common connect component are prefixed with the module TAG,
 * so it returns true if the specified netif is owned by this module
 */
static bool is_our_netif(const char *prefix, esp_netif_t *netif)
{
    return strncmp(prefix, esp_netif_get_desc(netif), strlen(prefix) - 1) == 0;
}

/* set up connection, Wi-Fi and/or Ethernet */
static void start(void)
{

#if CONFIG_EXAMPLE_CONNECT_WIFI
    s_example_esp_netif = wifi_start();
    s_active_interfaces++;
#endif

#if CONFIG_EXAMPLE_CONNECT_ETHERNET
    s_example_esp_netif = eth_start();
    s_active_interfaces++;
#endif

#if CONFIG_EXAMPLE_CONNECT_WIFI && CONFIG_EXAMPLE_CONNECT_ETHERNET
    /* if both intefaces at once, clear out to indicate that multiple netifs are active */
    s_example_esp_netif = NULL;
#endif

#if EXAMPLE_DO_CONNECT
    /* create semaphore if at least one interface is active */
    s_semph_get_ip_addrs = xSemaphoreCreateCounting(NR_OF_IP_ADDRESSES_TO_WAIT_FOR, 0);
#endif

}

/* tear down connection, release resources */
static void stop(void)
{
#if CONFIG_EXAMPLE_CONNECT_WIFI
    wifi_stop();
    s_active_interfaces--;
#endif

#if CONFIG_EXAMPLE_CONNECT_ETHERNET
    eth_stop();
    s_active_interfaces--;
#endif
}

#if EXAMPLE_DO_CONNECT
static esp_ip4_addr_t s_ip_addr;

static void on_got_ip(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    if (!is_our_netif(TAG, event->esp_netif)) {
        ESP_LOGW(TAG, "Got IPv4 from another interface \"%s\": ignored", esp_netif_get_desc(event->esp_netif));
        return;
    }
    ESP_LOGI(TAG, "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));
    memcpy(&s_ip_addr, &event->ip_info.ip, sizeof(s_ip_addr));
    xSemaphoreGive(s_semph_get_ip_addrs);
}
#endif

#ifdef CONFIG_EXAMPLE_CONNECT_IPV6

static void on_got_ipv6(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
    if (!is_our_netif(TAG, event->esp_netif)) {
        ESP_LOGW(TAG, "Got IPv6 from another netif: ignored");
        return;
    }
    esp_ip6_addr_type_t ipv6_type = esp_netif_ip6_get_addr_type(&event->ip6_info.ip);
    ESP_LOGI(TAG, "Got IPv6 event: Interface \"%s\" address: " IPV6STR ", type: %s", esp_netif_get_desc(event->esp_netif),
             IPV62STR(event->ip6_info.ip), s_ipv6_addr_types[ipv6_type]);
    if (ipv6_type == EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE) {
        memcpy(&s_ipv6_addr, &event->ip6_info.ip, sizeof(s_ipv6_addr));
        xSemaphoreGive(s_semph_get_ip_addrs);
    }
}

#endif // CONFIG_EXAMPLE_CONNECT_IPV6

esp_err_t example_connect(void)
{
#if EXAMPLE_DO_CONNECT
    if (s_semph_get_ip_addrs != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
#endif
    start();
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&stop));
    ESP_LOGI(TAG, "Waiting for IP(s)");
    for (int i = 0; i < NR_OF_IP_ADDRESSES_TO_WAIT_FOR; ++i) {
        xSemaphoreTake(s_semph_get_ip_addrs, portMAX_DELAY);
    }
    // iterate over active interfaces, and print out IPs of "our" netifs
    esp_netif_t *netif = NULL;
    esp_netif_ip_info_t ip;
    for (int i = 0; i < esp_netif_get_nr_of_ifs(); ++i) {
        netif = esp_netif_next(netif);
        if (is_our_netif(TAG, netif)) {
            ESP_LOGI(TAG, "Connected to %s", esp_netif_get_desc(netif));
            ESP_ERROR_CHECK(esp_netif_get_ip_info(netif, &ip));
            ESP_LOGI(TAG, "- IPv4 address: " IPSTR, IP2STR(&ip.ip));
#ifdef CONFIG_EXAMPLE_CONNECT_IPV6
            esp_ip6_addr_t ip6[MAX_IP6_ADDRS_PER_NETIF];
            int ip6_addrs = esp_netif_get_all_ip6(netif, ip6);
            for (int j = 0; j < ip6_addrs; ++j) {
                esp_ip6_addr_type_t ipv6_type = esp_netif_ip6_get_addr_type(&(ip6[j]));
                ESP_LOGI(TAG, "- IPv6 address: " IPV6STR ", type: %s", IPV62STR(ip6[j]), s_ipv6_addr_types[ipv6_type]);
            }
#endif
        }
    }
    return ESP_OK;
}

esp_err_t example_disconnect(void)
{
    if (s_semph_get_ip_addrs == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    stop();
    ESP_ERROR_CHECK(esp_unregister_shutdown_handler(&stop));
    vSemaphoreDelete(s_semph_get_ip_addrs);
    s_semph_get_ip_addrs = NULL;
    return ESP_OK;
}

#ifdef CONFIG_EXAMPLE_CONNECT_WIFI

static void on_wifi_disconnect(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_NOT_STARTED) {
        return;
    }
    ESP_ERROR_CHECK(err);
}

#ifdef CONFIG_EXAMPLE_CONNECT_IPV6

static void on_wifi_connect(void *esp_netif, esp_event_base_t event_base,
                            int32_t event_id, void *event_data)
{
    esp_netif_create_ip6_linklocal(esp_netif);
}

#endif // CONFIG_EXAMPLE_CONNECT_IPV6

static esp_netif_t *wifi_start(void)
{

    char *desc;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    // Prefix the interface description with the module TAG
    // Warning: the interface desc is used in tests to capture actual connection details (IP, gw, mask)
    asprintf(&desc, "%s: %s", TAG, esp_netif_config.if_desc);
    esp_netif_config.if_desc = desc;
    esp_netif_config.route_prio = 128;
    esp_netif_t *netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
    free(desc);
    esp_wifi_set_default_wifi_sta_handlers();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));
#ifdef CONFIG_EXAMPLE_CONNECT_IPV6
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &on_wifi_connect, netif));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6, &on_got_ipv6, NULL));
#endif

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
#if 1
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config_t));
    memcpy(wifi_config.sta.ssid,  gstrssid, WIFI_LEN);
    memcpy(wifi_config.sta.password,  gstrpwd, WIFI_LEN);
#else
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_EXAMPLE_WIFI_SSID,
            .password = CONFIG_EXAMPLE_WIFI_PASSWORD,
            //.scan_method = EXAMPLE_WIFI_SCAN_METHOD,
            //.sort_method = EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD,
            //.threshold.rssi = CONFIG_EXAMPLE_WIFI_SCAN_RSSI_THRESHOLD,
            //.threshold.authmode = EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD,
        },
    };
#endif

    ESP_LOGI(TAG, "Connecting to %s...%s", wifi_config.sta.ssid, wifi_config.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();
    return netif;
}

static void wifi_stop(void)
{
    esp_netif_t *wifi_netif = get_example_netif_from_desc("sta");
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip));
#ifdef CONFIG_EXAMPLE_CONNECT_IPV6
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_GOT_IP6, &on_got_ipv6));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &on_wifi_connect));
#endif
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) {
        return;
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(wifi_netif));
    esp_netif_destroy(wifi_netif);
    s_example_esp_netif = NULL;
}
#endif // CONFIG_EXAMPLE_CONNECT_WIFI


esp_netif_t *get_example_netif(void)
{
    return s_example_esp_netif;
}

esp_netif_t *get_example_netif_from_desc(const char *desc)
{
    esp_netif_t *netif = NULL;
    char *expected_desc;
    asprintf(&expected_desc, "%s: %s", TAG, desc);
    while ((netif = esp_netif_next(netif)) != NULL) {
        if (strcmp(esp_netif_get_desc(netif), expected_desc) == 0) {
            free(expected_desc);
            return netif;
        }
    }
    free(expected_desc);
    return netif;
}

/*****Advanced HTTPS OTA*****/

static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
{
    if (new_app_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
    }

#ifdef CONFIG_EXAMPLE_SKIP_VERSION_CHECK
    if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0) {
        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
        return ESP_FAIL;
    }
#endif

#ifdef CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK
    /**
     * Secure version check from firmware image header prevents subsequent download and flash write of
     * entire firmware image. However this is optional because it is also taken care in API
     * esp_https_ota_finish at the end of OTA update procedure.
     */
    const uint32_t hw_sec_version = esp_efuse_read_secure_version();
    if (new_app_info->secure_version < hw_sec_version) {
        ESP_LOGW(TAG, "New firmware security version is less than eFuse programmed, %d < %d", new_app_info->secure_version, hw_sec_version);
        return ESP_FAIL;
    }
#endif

    return ESP_OK;
}

static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
{
    esp_err_t err = ESP_OK;
    /* Uncomment to add custom headers to HTTP request */
    // err = esp_http_client_set_header(http_client, "Custom-Header", "Value");
    return err;
}

void advanced_ota_example_task(void *pvParameter)
{
	gulOTAsts = ENUM_OTA_STS_BEGIN;

    ESP_LOGI(TAG, "Starting Advanced OTA example [%d]!!", gulOTAsts);
    printf("Firmware url: %s\r\n", frimwareURL); //CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL
    esp_err_t ota_finish_err = ESP_OK;
    esp_http_client_config_t config = {
        //.url = CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL,
        .cert_pem = (char *)server_cert_pem_start,
        .timeout_ms = CONFIG_EXAMPLE_OTA_RECV_TIMEOUT,
        .keep_alive_enable = true,
    };

	//memcpy(config.url,  frimwareURL, FIRMWARE_URL_LEN);
	config.url = frimwareURL;
#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL_FROM_STDIN
    char url_buf[OTA_URL_SIZE];
    if (strcmp(config.url, "FROM_STDIN") == 0) {
        example_configure_stdin_stdout();
        fgets(url_buf, OTA_URL_SIZE, stdin);
        int len = strlen(url_buf);
        url_buf[len - 1] = '\0';
        config.url = url_buf;
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong firmware upgrade image url");
        abort();
    }
#endif

#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
    config.skip_cert_common_name_check = true;
#endif

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
        .http_client_init_cb = _http_client_init_cb, // Register a callback to be invoked after esp_http_client is initialized
#ifdef CONFIG_EXAMPLE_ENABLE_PARTIAL_HTTP_DOWNLOAD
        .partial_http_download = true,
        .max_http_request_size = CONFIG_EXAMPLE_HTTP_REQUEST_SIZE,
#endif
    };
    //gulOTAsts = ENUM_OTA_STS_BEGIN;
    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);

    if (err != ESP_OK) {
    	gulOTAsts = ENUM_OTA_STS_NO_RUNNING;
        ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed, reset the ota thread sts:%d", gulOTAsts);
        vTaskDelete(NULL);
        return;
    }
    gulOTAsts = ENUM_OTA_STS_CONNECTED;
    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed");
        goto ota_end;
    }
    err = validate_image_header(&app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "image header verification failed");
        goto ota_end;
    }
    gulOTAsts = ENUM_OTA_STS_CHECKED;
    while (1) {
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }

        gulOTAsts = ENUM_OTA_STS_DOWNLOADING;
        // esp_https_ota_perform returns after every read operation which gives user the ability to
        // monitor the status of OTA upgrade by calling esp_https_ota_get_image_len_read, which gives length of image
        // data read so far.
        ESP_LOGD(TAG, "Image bytes read: %d", esp_https_ota_get_image_len_read(https_ota_handle));
    }

    if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
        // the OTA image was not completely received and user can customise the response to this situation.
        ESP_LOGE(TAG, "Complete data was not received.");
        gulOTAsts = ENUM_OTA_STS_DOWNLOAD_FAIL;
    } else {
    	ESP_LOGI(TAG, "Complete data was received.");
    	gulOTAsts = ENUM_OTA_STS_DOWNLOAD_FINISH;
        ota_finish_err = esp_https_ota_finish(https_ota_handle);
        if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
        	gulOTAsts = ENUM_OTA_STS_EARSE_FINISH;
            ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            if(gulOTAReboot)
            {
            	gulOTAsts = ENUM_OTA_STS_READY_TO_REBOOT;
            	esp_restart();
            }
        } else {
            if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
                ESP_LOGE(TAG, "Image validation failed, image is corrupted");
                gulOTAsts = ENUM_OTA_STS_EARSE_FAIL_IMAGE_CORRUPTED;
            }
            ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
            gulOTAsts = ENUM_OTA_STS_EARSE_FAIL;
            vTaskDelete(NULL);
        }
    }

ota_end:
    esp_https_ota_abort(https_ota_handle);
    gulOTAsts = ENUM_OTA_STS_NO_RUNNING;
    ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed, %d", gulOTAsts);
    vTaskDelete(NULL);


}

//system/ota
int sys_ota(int reboot)
{
	printf("sys ota\r\n");

	wifi_connect();

	if(!gulOTAsts)
	{
		xTaskCreate(&advanced_ota_example_task, "advanced_ota_example_task", 1024 * 8, NULL, 5, NULL);
	}
	else
	{
		ESP_LOGE(TAG, "OTA already Running! %d", gulOTAsts);
	}


	return gulOTAsts;
}

int sys_ota_set_url(char *strurl)
{
	if(!strurl)
	{
		ESP_LOGE(TAG, "Input url Is Null");
		return -1;
	}
	ESP_LOGI(TAG, "set ota URL %s", strurl);

	if(strlen(strurl) > FIRMWARE_URL_LEN)
	{
		ESP_LOGE(TAG, "url is too long than %d", FIRMWARE_URL_LEN);
		return -2;
	}

	if(!strcmp(strurl, frimwareURL))
	{
		ESP_LOGW(TAG, "URL %s %s is samme!", strurl, frimwareURL);
		return -1;
	}

	strcpy(frimwareURL, strurl);
	//memcpy(frimwareURL, strurl, FIRMWARE_URL_LEN);
	return 0;
}

int sys_ota_sts_get()
{
	return gulOTAsts;
}

int sys_ota_reboot_set(int _reboot)
{
	gulOTAReboot = !!_reboot;
	return gulOTAReboot;
}

int wifi_set_user_pwd(char *struser, char *strpwd)
{
	ESP_LOGI(TAG, "set wifi uername %s pwd %s", struser, strpwd);
	if(!struser || !strpwd) return -1;

	if(!strcmp(struser, gstrssid) && !strcmp(strpwd, gstrpwd))
	{
		ESP_LOGW(TAG, "ssid %s %s is samme and pwd %s %s is same too!", struser, gstrssid, strpwd, gstrpwd);
		return -1;
	}

	memcpy(gstrssid, struser, WIFI_LEN);
	memcpy(gstrpwd, strpwd, WIFI_LEN);
	return 0;
}

//*stRet = &stWifi[0];

int wifi_get_info(STRU_WIFI_INFO **stRet)
{
	*stRet = &stWifi[0];
	return gulWifiApcnt;
}
