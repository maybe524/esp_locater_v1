#include "../common_esp32.h"
#include "../common/common.h"
static void *example_thread(void * arg);

extern void lis2dw12_activity(void);
/*
 *
broker.emqx.io
 *
 * ABCABCABC¡¡XYZXYZXYZ¡¡QAZWSXQAZ
 *
 * int SERIAL_LENGTH = 9;
    char[] CODE_SERIAL = {
            'G', 'i', 'E', 'N', 'P', 's', 'q', 'T', 'h', 'v',
            'u', 'X', 'e', 'p', 'S', 'z', 'g', 'j', 'n', 'y',
            'W', 'M', 'a', 'Q', 'k', 'L', 'b', 't', 'R', 'B',
            'Z', 'H', 'D', 'K', 'r', 'f', 'm', 'V', 'J', 'C',
            'U', 'd', 'F', 'A', 'c', 'w', 'x', 'Y'
    };

//port 1883
 */

#include "nvs_flash.h"
#include "nvs.h"

#include "driver/temp_sensor.h"


float temp_read()
{
	float tsens_out;
	//while (1) {
		//vTaskDelay(1000 / portTICK_RATE_MS);
		temp_sensor_read_celsius(&tsens_out);
		//ESP_LOGI(TAG, "Temperature out celsius %f", tsens_out);
		printf("Temperature out celsius %f\r\n", tsens_out);
	//}
	return tsens_out;
}

int message_recv()
{
	//return (1+rand()%22);
	return 100;
}

void wifi_sts_thread()
{
	static int ulWifiThread = 0;
	if(!ulWifiThread)
	{
		while(1)
		{
			ulWifiThread = 1;
			printf("[%20s]%d\r\n", "OTA STS", sys_ota_sts_get());
			vTaskDelay(5000 / portTICK_PERIOD_MS);

			if(ENUM_OTA_STS_EARSE_FINISH == sys_ota_sts_get())
			{
				esp_restart();
			}
		}
	}
}

void message_handle()
{
	int restart_time = 0;

	//read and write storage
	int key_val = 123;

	vTaskDelay(5000 / portTICK_PERIOD_MS);
	STRU_WIFI_INFO *pstWifiInfo  = NULL;
	//wifi
	nvs_init();

	unsigned int ulIdx = 0;
	unsigned int ulapcnt = 0;

//	wifi_init();
//	wifi_scan();

	while(1)
	{
		int slRecv = message_recv();
		//printf("recv %d message\r\n", slRecv);
		printf("*****[tese case%d]*****\r\n", slRecv);

		switch(slRecv)
		{
		case 1:

			//storage_read();
			//restart_time++;
			//storage_write(restart_time);

			//read and write storage
			//printf("*****[tese case1]*****\r\n");
			key_val = storage_read("key");
			printf("[storage key]read key =%d\r\n", key_val++);
			storage_write("key", key_val);

			//dpp_enrollee_init();

			break;
		case 2:
			wifi_init();
			wifi_scan();
			ulapcnt = wifi_get_info(&pstWifiInfo);
			printf("Get Ap Cnt %d\r\n", ulapcnt);
			for(ulIdx = 0;ulIdx < ulapcnt; ulIdx++)
			{
				printf("SSID:%s\r\n", pstWifiInfo[ulIdx].ssid);
				printf("MAC:%s\r\n", pstWifiInfo[ulIdx].mac);
				printf("RSSI:%d\r\n", pstWifiInfo[ulIdx].rssi);
			}
			//message_alarm_report(); //6 message
			break;
		case 3:
			printf("[cap]%d\r\n", hardware_battery_get_cap());
			//message_app_online_recv();//2 message
			break;
		case 4:
			printf("[temp sensor]%f\r\n", temp_read());
			//messsage_power_sleeep_recv();//2 message
			break;

		case 5:
			wifi_init();
			wifi_set_user_pwd("CMCC-NtKi", "3d687272");
			sys_ota_set_url("http://192.168.1.6:8080/hello_world.bin");
			sys_ota();
			//messsage_location_report_continus();//5 message
			break;
		case 6:
			hardware_wakeup_4g_module();
			//messsage_fense_report();//6 message
			break;
		case 7:
			wifi_init();
			sys_ota();
			//messsage_location_report();//7 message
			break;
		case 8:
			lis2dw12_activity();
			//messsage_devices_record_report();//8 message
			break;
		case 9:
			power_init();
			//messsage_app_account_set();//9 message
			break;
		case 10:
			wifi_init();
			wifi_connect();
			wifi_disconnect();
			wifi_scan();
			//   xTaskCreate(&wifi_sts_thread, "wifi_sts_thread", 1024 * 8, NULL, 5, NULL);
			//sys_ota();
			//messsage_wifi_location_report_continus();//10 message
			break;
		case 11:
		    xTaskCreate(&wifi_sts_thread, "wifi_sts_thread", 1024 * 8, NULL, 5, NULL);
			//messsage_device_update();//11 message
			break;
		case 12:
			wifi_init();
			wifi_scan();
			sys_ota();
			//messsage_device_hardware_data_report();//12 message
			break;
		case 13:
			wifi_init();
			wifi_scan();
			wifi_connect();
			wifi_disconnect();
			//messsage_device_data_report();//13 message
			break;
		case 14:
			//messsage_device_voices_report();//14 message
			break;
		case 15:
			//messsage_device_run_date_report();//15 message
			break;
		case 16:
			//messsage_device_leds_set();//16 message
			break;
		case 17:
			//messsage_device_ops_set();//17 message
			break;
		case 18:
			//messsage_app_config_set();//18 message
			break;
		case 19:
			//messsage_app_device_config_set();//19 message
			break;
		case 20:
			key_val = storage_read("key");
			printf("[storage key]read key =%d\r\n", key_val++);
			storage_write("key", key_val);

			printf("[%20s]%f\r\n", "temp sensor", temp_read());
			printf("[%20s]%d\r\n", "cap", hardware_battery_get_cap());

			wifi_init();
			wifi_scan();
			ulapcnt = wifi_get_info(&pstWifiInfo);
			printf("[%20s]%d\r\n", "Wifi Cnt", ulapcnt);
			for(ulIdx = 0;ulIdx < ulapcnt; ulIdx++)
			{
				printf("[SSID]%s\r\n", pstWifiInfo[ulIdx].ssid);
			}
			//messsage_temp_report_set();//20 message
			break;
		case 21:

			//messsage_battery_set();//21 message
			break;
		case 22:
			//messsage_runtime();//22 message
			break;
		default:
			break;


		}

		vTaskDelay(5000 / portTICK_PERIOD_MS);
	    //message_location_continous_recv();
	}
}


//send
void message_send_init()
{
	char *str = "SERIAL/#";


}

void message_send_alarm_temp()
{

}

//receive
void message_receive_init()
{

}

void message_init()
{
    pthread_attr_t attr;
    pthread_t thread1, thread2;
    esp_pthread_cfg_t esp_pthread_cfg;
    int res;
#if 0
    // Create a pthread with the default parameters
    res = pthread_create(&thread1, NULL, message_handle, NULL);
    if(!res)
    {
        printf("Created thread 0x%x\n", thread1);
    	printf("message init success\r\n");
    }
    else
    {
    	//assert(res == 0);
    	printf("message init fail\r\n");
    }
#endif
    xTaskCreate(&message_handle, "message_handle", 1024 * 8, NULL, 5, NULL);

}
