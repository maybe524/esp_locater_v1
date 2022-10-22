#ifndef __COMMON_POWER_H_
#define __COMMON_POWER_H_

extern void common_power_enter_light_sleep_ext();
#include "common.h"

void power_init();
#if 0
void common_power_enter_light_sleep()
{
	common_power_enter_light_sleep_ext();
//	printf("enter light sleep\r\n");
}
#endif
#endif
