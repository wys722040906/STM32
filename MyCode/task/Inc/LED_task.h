#ifndef __LED_TASK_H_
#define __LED_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"

extern xTaskHandle LED_Handler;
/*----------------函数声明区---------------------*/
void LED_Task(void *pvParameters);

#endif