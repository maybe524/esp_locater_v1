/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "common_esp32.h"
#include "locater_v1.h"



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
    locater_uart_init();

    while(1)
    {
    	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }


}
