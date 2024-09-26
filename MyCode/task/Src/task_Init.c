#include "task_Init.h"


void taskInit(void){
	/*CAN1_Init*/
    CanFilter_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	
	
	/*CAN2_init*/
    CanFilter_Init(&hcan2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);	
	
	/*USART2 DMA_init*/

vPortEnterCritical();

    xTaskCreate(PID_Task,
         "PID_task",
          300,
          NULL,
          2,
          &PID_Handler); 

    xTaskCreate(LED_Task,
         "LED_task",
          100,
          NULL,
          2,
          &LED_Handler); 
	node_init();
vPortExitCritical();
}

