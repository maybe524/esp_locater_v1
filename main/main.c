/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "common.h"



static int app_locator_init(void)
{
    return 0;
}

void app_main(void)
{
    printf("Hello world! 123\n");

    common_init();

    //��ʼ��Ӳ��
    hareware_main();

    //prop_init();

    //power_init();

    //net_init();

    message_init();

    while(1)
    {
    	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

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
}
