#include "common_esp32.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

/* "Boot" button is active low */
#define BUTTON_WAKEUP_LEVEL_DEFAULT     0
int gulOnoff = 0;
void power_test(void)
{
    /* Configure the button GPIO as input, enable wakeup */
    const int button_gpio_num = 9;
    const int wakeup_level = BUTTON_WAKEUP_LEVEL_DEFAULT;
    gpio_config_t config = {
            .pin_bit_mask = BIT64(button_gpio_num),
            .mode = GPIO_MODE_INPUT
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    gpio_wakeup_enable(button_gpio_num,
            wakeup_level == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);

    while (1) {
        /* Wake up in 2 seconds, or when button is pressed */

        //esp_sleep_enable_timer_wakeup(10000*1000);
        //esp_sleep_enable_gpio_wakeup();
        esp_sleep_enable_uart_wakeup(0);
        //esp_sleep_enable_uart_wakeup(1);
        //printf("gpio_get_level(%d)%d\r\n", button_gpio_num, gpio_get_level(button_gpio_num));

    	if(!gulOnoff) break;

#if 0
        /* Wait until GPIO goes high */
        if (gpio_get_level(button_gpio_num) == wakeup_level) {
            printf("Waiting for GPIO%d to go high...\n", button_gpio_num);
            do {
                vTaskDelay(pdMS_TO_TICKS(10));
            } while (gpio_get_level(button_gpio_num) == wakeup_level);
        }
#endif
        printf("Entering light sleep\n");
        /* To make sure the complete line is printed before entering sleep mode,
         * need to wait until UART TX FIFO is empty:
         */
        uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

    	/* Get timestamp before entering sleep */
        int64_t t_before_us = esp_timer_get_time();

        /* Enter sleep mode */
        esp_light_sleep_start();
        /* Execution continues here after wakeup */

        /* Get timestamp after waking up from sleep */
        int64_t t_after_us = esp_timer_get_time();

        /* Determine wake up reason */
        const char* wakeup_reason;
        switch (esp_sleep_get_wakeup_cause()) {
            case ESP_SLEEP_WAKEUP_TIMER:
                wakeup_reason = "timer";
                break;
            case ESP_SLEEP_WAKEUP_GPIO:
                wakeup_reason = "pin";
                break;
            case ESP_SLEEP_WAKEUP_UART:
                wakeup_reason = "uart";
                break;
            default:
                wakeup_reason = "other";
                break;
        }

        printf("Returned from light sleep, reason: %s, t=%lld ms, slept for %lld ms\n",
                wakeup_reason, t_after_us / 1000, (t_after_us - t_before_us) / 1000);
    }

}


void common_power_enter_light_sleep_ext()
{
//	esp_sleep_enable_uart_wakeup(0);
	esp_light_sleep_start();
}

void power_enable(int onoff)
{
	gulOnoff = onoff;
	printf("%s:set power onoff %d\r\n", __FUNCTION__, onoff);
}

void power_init()
{
	power_test();
}
