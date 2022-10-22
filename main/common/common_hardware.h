/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef __COMMON_HARDWARE_H_
#define __COMMON_HARDWARE_H_
#include "common.h"

/*
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
*/
/*external function*/
inline void common_init(){ printf("common_init\r\n"); }
void hareware_main(void);
int hardware_battery_get_cap();//return 99,%
int hardware_charger_get();//0 - no charger 1-charger
int hardware_leds_ctl(int ledidx, int onoff);//onoff 0-off 1-on
float hardware_temp_get(); // return 99.c
int hardware_touch_get();//pengzhuang
int hardware_weilan_get();//weilan
int hardware_voice_record();
#endif
