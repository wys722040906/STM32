#ifndef __PID_TASK_
#define __PID_TASK_

#include "freertos.h"
#include "task.h"
#include "motor.h"
#include "PID.h"
#include "stdio.h"

extern xTaskHandle PID_Handler;	//任务句柄

#define SPEED_MODE 0
#define POSITION_MODE 1
#define M3510_MODE 0

typedef struct motor_data_send{
	int16_t _CAN1[4];  //can口向can总线发送的数据 电机1-4
	int16_t _CAN2[4];
}MDATA_SEND;

typedef struct motor_start_flag{
	uint8_t M2006;		//是否上电标志位
	uint8_t M3508;
	uint8_t M3510;
	uint8_t M_offset;	//是否已经记录了偏移量的标志位
}MSTART_FLAG;
typedef struct target_value{  //各电机目标值结构体
	float M3510_v1;	//*_rmap 变量为斜坡逼近过程的中间目标值
	float M3510_v2;
	float M3510_v3;
	float M3510_v4;
	float M3510_v1_ramp;
	float M3510_v2_ramp;
	float M3510_v3_ramp;
	float M3510_v4_ramp;
	float M3508_v1;	//*_rmap 变量为斜坡逼近过程的中间目标值
	float M3508_v2;
	float M3508_v3;
	float M3508_v4;
	float M3508_v1_ramp;
	float M3508_v2_ramp;
	float M3508_v3_ramp;
	float M3508_v4_ramp;
	float M2006_v1;
	float M2006_v2;
	float M2006_v3;
	float M2006_v4;
	float M2006_v1_ramp;
	float M2006_v2_ramp;
	float M2006_v3_ramp;
	float M2006_v4_ramp;
}TARGET;

typedef struct motor_state{  //含各种电机的状态--转角，速度
	RM3510_TypeDef M3510[4];  //3510电机状态
	RM3508_TypeDef M3508[4];  //存储3508电机状态
	M2006_TypeDef M2006[4]; 	 //2006电机状态
	int32_t M3510_offset[4];
	int32_t M3508_offset[4];
	int32_t M2006_offset[4];
}MOTOR_STATE;

typedef struct ramp{		//斜坡结构体,设置各种斜坡参数
	int32_t M3510[4];
	int32_t M3508[4];
	int32_t M2006[4];
}RAMP;
						

#ifdef __cplusplus
extern "C"{
#endif
/*------------------函数实现-------------------------*/
void PID_Task(void *pvParameters);
int32_t RAMP_self( int32_t final, int32_t now, int32_t ramp );
#ifdef __cplusplus
}
#endif	



#endif
	
											
					
					