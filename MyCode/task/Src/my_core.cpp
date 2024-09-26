#include "ROS_task.h"
#include "cmsis_os.h"
#include "main.h" //main函数的头文件
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include "stdio.h"

extern TIM_HandleTypeDef htim2; //定时器中断相关

TaskHandle_t loop_task_handle;	//同步任务的任务句柄
TaskHandle_t deal_task_handle;	//接收任务的任务句柄
void loop_task(void);//周期性地执行ROS节点的任务  检查新的订阅者或发布者连接、处理消息传递、更新节点状态 
void deal_task(void);//每次通过串口接收到数据后会自动触发 等待接收通知，并在接收到通知后处理ROS数据

ros::NodeHandle nh; // 创建节点句柄
std_msgs::UInt32 msg; // 创建话题消息
ros::Publisher pub("stm32_pub_chatter",&msg);// 创建发布者
void sub_callback(const std_msgs::String& msg);// 接收完成回调
ros::Subscriber<std_msgs::String> sub("stm32_sub_chatter",&sub_callback);//创建接收者#include "cmsis_os.h"


// 中断接收回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	// 由于在STM32Hardware.h中默认rosserial相关串口为huart2，因此这样进行判断
	if(huart->Instance == USART2){
		//接收完全回调
		if(huart->RxXferSize == Size){
			nh.getHardware()->reset_rbuf(nh.getHardware()->rbuflen);
			BaseType_t xHigherPriorityTaskWoken;
			// 任务通知
			vTaskNotifyGiveFromISR(deal_task_handle,&xHigherPriorityTaskWoken);
			//切换上下文
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		// 判断UART IDLE中断是否关闭，从而判断是否由IDLE引起的中断
		else if(!__HAL_UART_GET_IT_SOURCE(huart,UART_IT_IDLE))
		{
			nh.getHardware()->reset_rbuf(huart->RxXferSize - Size);
			BaseType_t xHigherPriorityTaskWoken;
			// 任务通知
			vTaskNotifyGiveFromISR(deal_task_handle,&xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

//发送回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	// 由于在STM32Hardware.h中默认rosserial相关串口为huart2，因此这样进行判断
	if(huart->Instance == USART2){
		nh.getHardware()->flush();
	}
}

void node_init(){
	// 初始化节点
	nh.initNode();
	// 注册发布者
	nh.advertise(pub);
	// 注册订阅者
	nh.subscribe(sub);
	// 创建任务
	xTaskCreate((TaskFunction_t)loop_task,"loop_task",256,NULL,1,&loop_task_handle);
	xTaskCreate((TaskFunction_t)deal_task,"deal_task",256,NULL,1,&deal_task_handle);
	// 启动定时器2，开始update中断
	HAL_TIM_Base_Start_IT(&htim2);
}

// 同步任务，一段时间同步一次
void loop_task(){
	
	for(;;){
//		int i=0;
		/*
检查是否有新的订阅者或发布者连接到当前节点；connect
如果有新的消息到达，将它们传递给相应的回调函数进行处理； callback
如果节点有需要发布的数据，将其发送给相应的发布者；pub
更新节点的状态，例如检查是否收到了预期的响应或完成了某个任务 receive_finish		
		*/
//		i++;
		nh.spinOnce();
		vTaskDelay(400);
		//确保系统的时间得到正确的更新，以便在需要的时候能够准确地获取当前时间
		// 这里回旋用于同步时间，时间不短于1s，但不建议过短以免影响实际使用
	}
}

// 处理接收的ros数据任务
void deal_task(){
	for(;;){
		// 等待接收通知
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		// 接收到通知处理数据
		nh.spinOnce();
	}
}

//定时发送消息
uint32_t count = 0;
void pub_msg(){
		// 判断是否连接PC端ROS
	if(nh.connected()){
		// 加载消息
		msg.data = count++;
		// 发布话题消息
		pub.publish(&msg);
	}
}

// 订阅话题消息回调函数
void sub_callback(const std_msgs::String& msg){
	char p_buff[50];
	sprintf(p_buff,"[stm32]: sub data:%s",msg.data);
	// 接收后打印，PC端会打印出来
	nh.loginfo(p_buff);
}
