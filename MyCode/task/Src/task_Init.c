#include "task_Init.h"


void taskInit(void){

	/*USART2 DMA_init*/
vPortEnterCritical();
    printf("TaskInit\r\n");
    xTaskCreate(LED_Task,
         "LED_task",
          100,
          NULL,
          2,
          &LED_Handler); 
    xTaskCreate(TEST_Task,
         "TEST_task",
          100,
          NULL,
          2,
          &TEST_Handler); 
vPortExitCritical();
}

