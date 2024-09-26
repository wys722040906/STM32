#include "PID_task.h"

xTaskHandle PID_Handler;	//任务句柄

TARGET motor_target = {.M3510_v1 = 0, .M3510_v2=0, .M3510_v3 = 0, .M3510_v4 = 0,
				.M3510_v1_ramp = 0, .M3510_v2_ramp = 0, .M3510_v3_ramp = 0, .M3510_v4_ramp=0,
				.M3508_v1 = 0, .M3508_v2=0, .M3508_v3 = 0, .M3508_v4 = 0,
				.M3508_v1_ramp = 0, .M3508_v2_ramp = 0, .M3508_v3_ramp = 0, .M3508_v4_ramp=0,
				.M2006_v1 = 0, .M2006_v2 = 0, .M2006_v3= 0, .M2006_v4 = 0,
				.M2006_v1_ramp = 0, .M2006_v2_ramp = 0, .M2006_v3_ramp = 0, .M2006_v4_ramp=0};

RAMP motor_ramp = {.M3510[0] = 200, .M3510[1] = 200, .M3510[2] = 200, .M3510[3] = 200,
	.M3508[0] = 200, .M3508[1] = 200, .M3508[2] = 200, .M3508[3] = 200,
	.M2006[0] = 367, .M2006[1] = 367, .M2006[2] = 367, .M2006[3] = 367};
//PID参数结构体
PID_TypeDef Cascaed_PID_position[4], //串联PID位置环
			Cascaed_PID_speed[4],	//串联PID速度环
			M2006_PID_position[4],
			M2006_PID_Speed[4];  //2006PID速度环

MOTOR_STATE motor_state ;	//电机状态

MDATA_SEND motor_data_send; //can口数据发送

MSTART_FLAG motor_start_flag;  //电机标志位

#if SPEED_MODE
			
void PID_Task(void *pvParameters)
{
//	portTickType xLastWakeTime = xTaskGetTickCount();		
	
	#if M3510_MODE
	PID_Init(&Cascaed_PID_speed[0], 4.3, 0.001, 4.5, 5000,5, RM3510_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_speed[1], 4.3, 0.0001, 4.5, 5000,5, RM3510_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_speed[2], 4.3, 0.0001, 4.5, 5000,5, RM3510_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_speed[3], 4.3, 0.0001, 4.5, 5000,5, RM3510_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);	
		
	#else
	PID_Init(&Cascaed_PID_speed[0], 4.3, 0.0001, 4.5, 5000,5, RM3508_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_speed[1], 4.3, 0.0001, 4.5, 5000,5, RM3508_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_speed[2], 4.3, 0.0001, 4.5, 5000,5, RM3508_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_speed[3], 4.3, 0.0001, 4.5, 5000,5, RM3508_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);		
	#endif
	
	PID_Init(&M2006_PID_Speed[0], 2.5, 0.4, 5, 500,5, M2006_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&M2006_PID_Speed[1], 2.5, 0.4, 5, 500,5, M2006_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&M2006_PID_Speed[2], 2.5, 0.4, 5, 500,5, M2006_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&M2006_PID_Speed[3], 2.5, 0.4, 5, 500,5, M2006_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);	
	
	for(;;){
		
	#if M3510_MODE
	motor_data_send._CAN1[0] = PID_Calculate(motor_state.M3510[0].Speed,motor_target.M3510_v1, &Cascaed_PID_speed[0]);
	motor_data_send._CAN1[1] = PID_Calculate(motor_state.M3510[1].Speed,motor_target.M3510_v2, &Cascaed_PID_speed[1]);
	motor_data_send._CAN1[2] = PID_Calculate(motor_state.M3510[2].Speed,motor_target.M3510_v3, &Cascaed_PID_speed[2]);
	motor_data_send._CAN1[3] = PID_Calculate(motor_state.M3510[3].Speed,motor_target.M3510_v4, &Cascaed_PID_speed[3]);	

	#else
	motor_data_send._CAN1[0] = PID_Calculate(motor_state.M3508[0].Speed,motor_target.M3508_v1, &Cascaed_PID_speed[0]);
	motor_data_send._CAN1[1] = PID_Calculate(motor_state.M3508[1].Speed,motor_target.M3508_v2, &Cascaed_PID_speed[1]);
	motor_data_send._CAN1[2] = PID_Calculate(motor_state.M3508[2].Speed,motor_target.M3508_v3, &Cascaed_PID_speed[2]);
	motor_data_send._CAN1[3] = PID_Calculate(motor_state.M3508[3].Speed,motor_target.M3508_v4, &Cascaed_PID_speed[3]);		
		
	#endif
		
	//2006速度模式控制		
	motor_data_send._CAN2[0]= PID_Calculate(motor_state.M2006[0].Speed,motor_target.M2006_v1, &M2006_PID_Speed[0]);
	motor_data_send._CAN2[1]= PID_Calculate(motor_state.M2006[1].Speed,motor_target.M2006_v2, &M2006_PID_Speed[1]);
	motor_data_send._CAN2[2]= PID_Calculate(motor_state.M2006[2].Speed,motor_target.M2006_v3, &M2006_PID_Speed[2]);
	motor_data_send._CAN2[3]= PID_Calculate(motor_state.M2006[3].Speed,motor_target.M2006_v4, &M2006_PID_Speed[3]);		
	
	
	MotorSend(&hcan2, 0x200, motor_data_send._CAN2);// ID:201~204	
#if VOFA_EN
	printf("%f,%f\n",motor_target.M2006_v4,(float)motor_state.M2006[3].Speed);
#endif
	vTaskDelay(1);
//	vTaskDelayUntil(&xLastWakeTime,2);		
	}
};

#elif  POSITION_MODE//若为位置模式
void PID_Task(void *pvParameters){ //电机pid控制任务
	
	portTickType xLastWakeTime = xTaskGetTickCount();
	
	#if M3510_MODE
	PID_Init(&Cascaed_PID_speed[0], 2, 0.001, 0, 500,5, RM3510_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_speed[1], 2, 0.001, 0, 500,5, RM3510_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_speed[2], 2, 0.001, 0, 500,5, RM3510_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_speed[3], 2, 0.001, 0, 500,5, RM3510_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);

	PID_Init(&Cascaed_PID_position[0], 2, 0, -2.5, 500,40, RM3510_LIMIT, 20000, 10000, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_position[1], 2, 0, -2.5, 500,40, RM3510_LIMIT, 20000, 10000, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_position[2], 2, 0, -2.5, 500,40, RM3510_LIMIT, 20000, 10000, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_position[3], 2, 0, -2.5, 500,40, RM3510_LIMIT, 20000, 10000, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	#else 
	PID_Init(&Cascaed_PID_speed[0], 2, 0.001, 0, 500,5, RM3508_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_speed[1], 2, 0.001, 0, 500,5, RM3508_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_speed[2], 2, 0.001, 0, 500,5, RM3508_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_speed[3], 2, 0.001, 0, 500,5, RM3508_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);		

	PID_Init(&Cascaed_PID_position[0], 2, 0, -2.5, 500,40, RM3508_LIMIT, 20000, 10000, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_position[1], 2, 0, -2.5, 500,40, RM3508_LIMIT, 20000, 10000, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_position[2], 2, 0, -2.5, 500,40, RM3508_LIMIT, 20000, 10000, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&Cascaed_PID_position[3], 2, 0, -2.5, 500,40, RM3508_LIMIT, 20000, 10000, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	#endif 
	
	
	PID_Init(&M2006_PID_Speed[0], 2.5, 0.01, 5, 500,5, M2006_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&M2006_PID_Speed[1], 2.5, 0.01, 5, 500,5, M2006_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&M2006_PID_Speed[2], 2.5, 0.01, 5, 500,5, M2006_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&M2006_PID_Speed[3], 2.5, 0.01, 5, 500,5, M2006_LIMIT, 200, 100, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);

	PID_Init(&M2006_PID_position[0], 3, 0, -3, 0, 40, M2006_LIMIT, 20000, 10000, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&M2006_PID_position[1], 3, 0, -3, 0, 40, M2006_LIMIT, 20000, 10000, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&M2006_PID_position[2], 3, 0, -3, 0, 40, M2006_LIMIT, 20000, 10000, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
	PID_Init(&M2006_PID_position[3], 3, 0, -3, 0, 40, M2006_LIMIT, 20000, 10000, 100, 100,Integral_Limit | Derivative_On_Measurement | Trapezoid_Intergral | ChangingIntegralRate);
    
	for(;;){
		
		if((motor_start_flag.M2006 & 0x01) && (!(motor_start_flag.M_offset & 0x10))){ motor_state.M2006_offset[0] = motor_state.M2006[0].Angle;motor_start_flag.M_offset  |=0x10;}
		if((motor_start_flag.M2006 & 0x02) && (!(motor_start_flag.M_offset & 0x20))){ motor_state.M2006_offset[1] = motor_state.M2006[1].Angle;motor_start_flag.M_offset  |=0x20;}
		if((motor_start_flag.M2006 & 0x04) && (!(motor_start_flag.M_offset & 0x40))){ motor_state.M2006_offset[2] = motor_state.M2006[2].Angle;motor_start_flag.M_offset  |=0x40;}
		if((motor_start_flag.M2006 & 0x08) && (!(motor_start_flag.M_offset & 0x80))){ motor_state.M2006_offset[3] = motor_state.M2006[3].Angle;motor_start_flag.M_offset  |=0x80;}
	#if M3510_MODE
		//3510位置模式控制
		if((motor_start_flag.M3510 & 0x01) && (!(motor_start_flag.M_offset & 0x01))){ motor_state.M3510_offset[0] = motor_state.M3510[0].Angle;motor_start_flag.M_offset  |=0x01;}//   纠偏
		if((motor_start_flag.M3510 & 0x02) && (!(motor_start_flag.M_offset & 0x02))){ motor_state.M3510_offset[1] = motor_state.M3510[1].Angle;motor_start_flag.M_offset  |=0x02;}
		if((motor_start_flag.M3510 & 0x04) && (!(motor_start_flag.M_offset & 0x04))){ motor_state.M3510_offset[2] = motor_state.M3510[2].Angle;motor_start_flag.M_offset  |=0x04;}
		if((motor_start_flag.M3510 & 0x08) && (!(motor_start_flag.M_offset & 0x08))){ motor_state.M3510_offset[3] = motor_state.M3510[3].Angle;motor_start_flag.M_offset  |=0x08;}
				
		motor_target.M3510_v1_ramp = RAMP_self(motor_target.M3510_v1,motor_target.M3510_v1_ramp,motor_ramp.M3510[0]);
		PID_Control_Smis(motor_state.M3510[0].Angle - motor_state.M3510_offset[0],motor_target.M3510_v1_ramp,&Cascaed_PID_position[0],motor_state.M3510[0].Speed);
		PID_Calculate(motor_state.M3510[0].Speed, Cascaed_PID_position[0].Output, &Cascaed_PID_speed[0]);
		motor_data_send._CAN1[0]=Cascaed_PID_speed[0].Output;

		motor_target.M3510_v2_ramp = RAMP_self(motor_target.M3510_v2,motor_target.M3510_v2_ramp,motor_ramp.M3510[1]);
		PID_Control_Smis(motor_state.M3510[1].Angle - motor_state.M3510_offset[1],motor_target.M3510_v2_ramp,&Cascaed_PID_position[1],motor_state.M3510[1].Speed);
		PID_Calculate(motor_state.M3510[1].Speed, Cascaed_PID_position[1].Output, &Cascaed_PID_speed[1]);
		motor_data_send._CAN1[1]=Cascaed_PID_speed[1].Output;
		
		motor_target.M3510_v3_ramp = RAMP_self(motor_target.M3510_v3,motor_target.M3510_v3_ramp,motor_ramp.M3510[2]);
		PID_Control_Smis(motor_state.M3510[2].Angle - motor_state.M3510_offset[2],motor_target.M3510_v3_ramp,&Cascaed_PID_position[2],motor_state.M3510[2].Speed);
		PID_Calculate(motor_state.M3510[2].Speed, Cascaed_PID_position[2].Output, &Cascaed_PID_speed[2]);
		motor_data_send._CAN1[2]=Cascaed_PID_speed[2].Output;
		
		motor_target.M3510_v4_ramp = RAMP_self(motor_target.M3510_v4,motor_target.M3510_v4_ramp,motor_ramp.M3510[3]);
		PID_Control_Smis(motor_state.M3510[3].Angle - motor_state.M3510_offset[3],motor_target.M3510_v4_ramp,&Cascaed_PID_position[3],motor_state.M3510[3].Speed);
		PID_Calculate(motor_state.M3510[3].Speed, Cascaed_PID_position[3].Output, &Cascaed_PID_speed[3]);
		motor_data_send._CAN1[3]=Cascaed_PID_speed[3].Output;
		
		if(motor_start_flag.M3510) MotorSend(&hcan1,0x200,motor_data_send._CAN1);	
	#else
		//3508位置模式控制
		if((motor_start_flag.M3508 & 0x01) && (!(motor_start_flag.M_offset & 0x01))){ motor_state.M3508_offset[0] = motor_state.M3508[0].Angle;motor_start_flag.M_offset  |=0x01;}//   纠偏
		if((motor_start_flag.M3508 & 0x02) && (!(motor_start_flag.M_offset & 0x02))){ motor_state.M3508_offset[1] = motor_state.M3508[1].Angle;motor_start_flag.M_offset  |=0x02;}
		if((motor_start_flag.M3508 & 0x04) && (!(motor_start_flag.M_offset & 0x04))){ motor_state.M3508_offset[2] = motor_state.M3508[2].Angle;motor_start_flag.M_offset  |=0x04;}
		if((motor_start_flag.M3508 & 0x08) && (!(motor_start_flag.M_offset & 0x08))){ motor_state.M3508_offset[3] = motor_state.M3508[3].Angle;motor_start_flag.M_offset  |=0x08;}

				
		motor_target.M3508_v1_ramp = RAMP_self(motor_target.M3508_v1,motor_target.M3508_v1_ramp,motor_ramp.M3508[0]);
		PID_Control_Smis(motor_state.M3508[0].Angle - motor_state.M3508_offset[0],motor_target.M3508_v1_ramp,&Cascaed_PID_position[0],motor_state.M3508[0].Speed);
		PID_Calculate(motor_state.M3508[0].Speed, Cascaed_PID_position[0].Output, &Cascaed_PID_speed[0]);
		motor_data_send._CAN1[0]=Cascaed_PID_speed[0].Output;

		motor_target.M3508_v2_ramp = RAMP_self(motor_target.M3508_v2,motor_target.M3508_v2_ramp,motor_ramp.M3508[1]);
		PID_Control_Smis(motor_state.M3508[1].Angle - motor_state.M3508_offset[1],motor_target.M3508_v2_ramp,&Cascaed_PID_position[1],motor_state.M3508[1].Speed);
		PID_Calculate(motor_state.M3508[1].Speed, Cascaed_PID_position[1].Output, &Cascaed_PID_speed[1]);
		motor_data_send._CAN1[1]=Cascaed_PID_speed[1].Output;
		
		motor_target.M3508_v3_ramp = RAMP_self(motor_target.M3508_v3,motor_target.M3508_v3_ramp,motor_ramp.M3508[2]);
		PID_Control_Smis(motor_state.M3508[2].Angle - motor_state.M3508_offset[2],motor_target.M3508_v3_ramp,&Cascaed_PID_position[2],motor_state.M3508[2].Speed);
		PID_Calculate(motor_state.M3508[2].Speed, Cascaed_PID_position[2].Output, &Cascaed_PID_speed[2]);
		motor_data_send._CAN1[2]=Cascaed_PID_speed[2].Output;
		
		motor_target.M3508_v4_ramp = RAMP_self(motor_target.M3508_v4,motor_target.M3508_v4_ramp,motor_ramp.M3508[3]);
		PID_Control_Smis(motor_state.M3508[3].Angle - motor_state.M3508_offset[3],motor_target.M3508_v4_ramp,&Cascaed_PID_position[3],motor_state.M3508[3].Speed);
		PID_Calculate(motor_state.M3508[3].Speed, Cascaed_PID_position[3].Output, &Cascaed_PID_speed[3]);
		motor_data_send._CAN1[3]=Cascaed_PID_speed[3].Output;
		
		if(motor_start_flag.M3508) MotorSend(&hcan1,0x200,motor_data_send._CAN1);
	
	#endif		
			
		//2006位置模式控制
		motor_target.M2006_v1_ramp = RAMP_self(motor_target.M2006_v1,motor_target.M2006_v1_ramp,motor_ramp.M2006[0]);
		PID_Control_Smis(motor_state.M2006[0].Angle - motor_state.M2006_offset[0],motor_target.M2006_v1_ramp,&M2006_PID_position[0],motor_state.M2006[0].Speed);
		PID_Calculate(motor_state.M2006[0].Speed, M2006_PID_position[0].Output, &M2006_PID_Speed[0]);
		motor_data_send._CAN2[0]=M2006_PID_Speed[0].Output;

		motor_target.M2006_v2_ramp = RAMP_self(motor_target.M2006_v2,motor_target.M2006_v2_ramp,motor_ramp.M2006[1]);
		PID_Control_Smis(motor_state.M2006[1].Angle - motor_state.M2006_offset[1],motor_target.M2006_v2_ramp,&M2006_PID_position[1],motor_state.M2006[1].Speed);
		PID_Calculate(motor_state.M2006[1].Speed, M2006_PID_position[1].Output, &M2006_PID_Speed[1]);
		motor_data_send._CAN2[1]=M2006_PID_Speed[1].Output;
		
		motor_target.M2006_v3_ramp = RAMP_self(motor_target.M2006_v3,motor_target.M2006_v3_ramp,motor_ramp.M2006[2]);
		PID_Control_Smis(motor_state.M2006[2].Angle - motor_state.M2006_offset[2],motor_target.M2006_v3_ramp,&M2006_PID_position[2],motor_state.M2006[2].Speed);
		PID_Calculate(motor_state.M2006[2].Speed, M2006_PID_position[2].Output, &M2006_PID_Speed[2]);
		motor_data_send._CAN2[2]=M2006_PID_Speed[2].Output;
		
		motor_target.M2006_v4_ramp = RAMP_self(motor_target.M2006_v4,motor_target.M2006_v4_ramp,motor_ramp.M2006[3]);
		PID_Control_Smis(motor_state.M2006[3].Angle - motor_state.M2006_offset[3],motor_target.M2006_v4_ramp,&M2006_PID_position[3],motor_state.M2006[3].Speed);
		PID_Calculate(motor_state.M2006[3].Speed, M2006_PID_position[3].Output, &M2006_PID_Speed[3]);
		motor_data_send._CAN2[3]=M2006_PID_Speed[3].Output;
		
		if(motor_start_flag.M2006) MotorSend(&hcan2, 0x200, motor_data_send._CAN2);
		#if VOFA_EN
		printf("%f,%f,%f\n",motor_target.M2006_v4, motor_target.M2006_v4_ramp, (float)motor_state.M2006[3].Angle);
		#endif
		vTaskDelay(2);
//		vTaskDelayUntil(&xLastWakeTime,2);
  }
}
#else
	#program message("You do not select SPEED_MODE or POSITION_MDOE")
#endif

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){     //can回调
	uint16_t ID1 = CAN_Receive_DataFrame(&hcan1,CAN1_buff);     //拷贝消息，不清空
		switch(ID1){
#if M3510_MODE
			case 0x201: 
				RM3510_Receive(&motor_state.M3510[0],CAN1_buff);
				motor_start_flag.M3510 |= 0x01; 
				break;
			case 0x202: 
				RM3510_Receive(&motor_state.M3510[1],CAN1_buff);
				motor_start_flag.M3510 |= 0x02; 
				break;
			case 0x203: 
				RM3508_Receive(&motor_state.M3508[2],CAN1_buff);
				motor_start_flag.M3510 |= 0x04; 
				break;
			case 0x204: 
				RM3508_Receive(&motor_state.M3508[3],CAN1_buff);
				motor_start_flag.M3510 |= 0x08; 
				break;
		
#else		
			case 0x201: 
				RM3508_Receive(&motor_state.M3508[0],CAN1_buff);
				motor_start_flag.M3508 |= 0x01; 
				break;
			case 0x202: 
				RM3508_Receive(&motor_state.M3508[1],CAN1_buff);
				motor_start_flag.M3508 |= 0x02; 
				break;
			case 0x203: 
				RM3508_Receive(&motor_state.M3508[2],CAN1_buff);
				motor_start_flag.M3508 |= 0x04; 
				break;
			case 0x204: 
				RM3508_Receive(&motor_state.M3508[3],CAN1_buff);
				motor_start_flag.M3508 |= 0x08; 
				break;  
#endif
			default: break;
		}				
	
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	
	uint16_t ID1 = CAN_Receive_DataFrame(&hcan2,CAN2_buff);     
	switch(ID1){             
		case 0x201: 
			M2006_Receive(&motor_state.M2006[0], CAN2_buff);
			if(motor_state.M2006[0].Angle != 0 && motor_state.M2006[0].MchanicalAngle != 0)
				motor_start_flag.M2006 |= 0x01; 
			break;
		case 0x202:
			M2006_Receive(&motor_state.M2006[1], CAN2_buff);
			if(motor_state.M2006[1].Angle != 0 && motor_state.M2006[1].MchanicalAngle != 0)
				motor_start_flag.M2006 |= 0x02; 
			break;
		case 0x203: 
			M2006_Receive(&motor_state.M2006[2], CAN2_buff);
			if(motor_state.M2006[2].Angle != 0 && motor_state.M2006[2].MchanicalAngle != 0)
				motor_start_flag.M2006 |= 0x04; 
			break;
		case 0x204: 
			M2006_Receive(&motor_state.M2006[3], CAN2_buff);
			if(motor_state.M2006[3].Angle != 0 && motor_state.M2006[3].MchanicalAngle != 0)
				motor_start_flag.M2006 |= 0x08;
			break;  
		default: break;
	}				
}




int32_t RAMP_self( int32_t final, int32_t now, int32_t ramp ){ //斜坡函数
    float buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)  
                now += ramp;  
        else
                now += buffer;
    }		
    else
    {
        if (buffer < -ramp)
                now += -ramp;
        else
                now += buffer;
    }
    return now;
}
