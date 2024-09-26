#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_


#include "CANDrive.h"
#include "freertos.h"
#include "task.h"
#include "stdio.h"

#include "PID_task.h"
#include "LED_task.h"
#include "ROS_task.h"
extern xTaskHandle PID_Handler;
extern xTaskHandle LED_Handler;

#ifdef __cplusplus
extern "C"{
#endif
void taskInit(void);
#ifdef __cplusplus
}
#endif		



#endif


