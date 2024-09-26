#include "dma_interface.h"

//uint8_t rx_buff[BUFF_SIZE];

///*
//	usart.h
//	extern DMA_HandleTypeDef hdma_usart2_rx;

//	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buff, BUFF_SIZE);
//	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断	
//	加入到main.c init
//*/

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
//{

//    if(huart->Instance == USART2)
//    {
//		HAL_UART_Transmit(&huart2, rx_buff, Size, 0xffff);         // 将接收到的数据再发出
//        if (Size <= BUFF_SIZE)
//        {	
//			if(rx_buff[0] == 0xAA){
//				if(rx_buff[1] == Size){
//					uint8_t sum = 0;
//					for(int i = 0; i<Size - 1; i++){
//						sum += rx_buff[i];
//					}
//					if(sum == rx_buff[Size - 1]){
//						for(int i = 2; i < Size-1; i+=2){
//							int status = 0;
//							if(rx_buff[i+=1]){
//								status = 1;
//							}
//							switch(rx_buff[i]){
//								case 0x01 :
//									break;
//								default : break;
//							}
//						}
//					}
//				}
//			}
//        }
//        else  // 接收数据长度大于BUFF_SIZE，错误处理
//        {
//            fprintf(stderr, "DMA transmit is overflowed");
////			return -1; //异常退出
//        }
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buff, BUFF_SIZE); // 接收完毕后重启
//		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
//		memset(rx_buff, 0, BUFF_SIZE);							   // 清除接收缓存
//    }
//}

//void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
//{
//    if(huart->Instance == USART2)
//    {
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buff, BUFF_SIZE); // 接收发生错误后重启
//		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
//		memset(rx_buff, 0, BUFF_SIZE);							   // 清除接收缓存
//        
//    }
//}