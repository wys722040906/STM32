#ifndef __TEST_TASK_H_
#define __TEST_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"

extern xTaskHandle TEST_Handler;
/*----------------函数声明区---------------------*/
void TEST_Task(void *pvParameters);

#endif