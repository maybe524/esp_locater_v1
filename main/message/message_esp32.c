#include "../common_esp32.h"

static void *example_thread(void * arg);

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

void temp_init()
{
	// Initialize touch pad peripheral, it will start a timer to run a filter
	//ESP_LOGI(TAG, "Initializing Temperature sensor");
	printf("Initializing Temperature sensor\r\n");
	temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
	temp_sensor_get_config(&temp_sensor);
	//ESP_LOGI(TAG, "default dac %d, clk_div %d", temp_sensor.dac_offset, temp_sensor.clk_div);
	printf("default dac %d, clk_div %d\r\n", temp_sensor.dac_offset, temp_sensor.clk_div);
	temp_sensor.dac_offset = TSENS_DAC_DEFAULT;
	temp_sensor_set_config(temp_sensor);
	temp_sensor_start();
	//ESP_LOGI(TAG, "Temperature sensor started");
	printf("Temperature sensor started\r\n");
}

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

void storage_read()
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

		// Read
		printf("Reading restart counter from NVS ... ");
		int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
		err = nvs_get_i32(my_handle, "restart_counter", &restart_counter);
		switch (err) {
			case ESP_OK:
				printf("Done\n");
				printf("Restart counter = %d\n", restart_counter);
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
}

void storage_write(int new_restart_counter)
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

		int32_t restart_counter = new_restart_counter; // value will default to 0, if not set yet in NVS

		// Write
		printf("Updating restart counter in NVS ... ");
		//restart_counter++;
		err = nvs_set_i32(my_handle, "restart_counter", restart_counter);
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

int message_recv()
{
	//return (1+rand()%22);
	return 1;
}

void message_handle()
{
	int restart_time = 0;
	temp_init();

	while(1)
	{
		int slRecv = message_recv();
		printf("recv %d message\r\n", slRecv);

		switch(slRecv)
		{
		case 1:
		    //message_init();
			//printf("temp sensor %f\r\n", temp_read());
			//storage_read();
			//restart_time++;
			//storage_write(restart_time);
			break;
		case 2:
			//message_alarm_report(); //6 message
			break;
		case 3:
			//message_app_online_recv();//2 message
			break;
		case 4:
			//messsage_power_sleeep_recv();//2 message
			break;
		case 5:
			//messsage_location_report_continus();//5 message
			break;
		case 6:
			//messsage_fense_report();//6 message
			break;
		case 7:
			//messsage_location_report();//7 message
			break;
		case 8:
			//messsage_devices_record_report();//8 message
			break;
		case 9:
			//messsage_app_account_set();//9 message
			break;
		case 10:
			//messsage_wifi_location_report_continus();//10 message
			break;
		case 11:
			//messsage_device_update();//11 message
			break;
		case 12:
			//messsage_device_hardware_data_report();//12 message
			break;
		case 13:
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
}
