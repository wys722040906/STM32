#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_


// #include "CANDrive.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"

#include "LED_task.h"
#include "TEST_task.h"

extern xTaskHandle LED_Handler;
extern xTaskHandle TEST_Handler;
void taskInit(void);



#endif


