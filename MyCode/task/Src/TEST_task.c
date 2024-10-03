#include "TEST_task.h"


xTaskHandle TEST_Handler;

void TEST_Task(void *pvParameters){
		// int a = 0;
	TickType_t xLastWakeTime;	
    // 初始化上次唤醒时间为当前时间
    xLastWakeTime = xTaskGetTickCount();
	for(;;){
		printf("TEST task\r\n");
		// a += 1;
	//   if(a >= 100)a=0;
		// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		// vTaskDelayUntil(&xLastWakeTime ,500);
		// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); 		
		vTaskDelayUntil(&xLastWakeTime ,500);
	}
}
