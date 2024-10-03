#include "LED_task.h"

/*
-----流水灯变量定义区-------------
*/
xTaskHandle LED_Handler;

void LED_Task(void *pvParameters){
		// int a = 0;
	HAL_Delay(200);
	OLED_Init();
//	OLED_Test();
	TickType_t xLastWakeTime;	
    // 初始化上次唤醒时间为当前时间
    xLastWakeTime = xTaskGetTickCount();

	for(;;){
	  static char c = '0';
	  c++; 
		OLED_NewFrame();
	  OLED_PrintASCIIString(64,24,&c , &afont8x6, OLED_COLOR_REVERSED);
		OLED_ShowFrame();
//		printf("LED task\r\n");
		// a += 1;
	//   if(a >= 100)a=0;
		// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime ,10);
		// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); 		
//        vTaskDelayUntil(&xLastWakeTime ,500);
	}
}




