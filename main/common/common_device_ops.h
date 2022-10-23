#ifndef __COMMON_DEVICESOPS_H_
#define __COMMON_DEVICESOPS_H_
#include "common.h"

int32_t storage_read(char *key);
void storage_write(char *key, int value);
#endif
