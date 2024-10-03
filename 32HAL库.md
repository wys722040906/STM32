# 串口通信

### 简介

### 超时收发

**HAL_UART_Transmit** **(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout超时退出)**;

串口发送数据，一直发则程序一直卡在这里调用该函数

**HAL_UART_Receive()**;

串口接收数据，一直收则程序一直卡在这里调用该函数

### 中断收发

- **HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)**

  );

  串口中断模式发送 

  将数据地址+固定长度数据放入变量缓冲区--调用中断发送数据+

- **HAL_UART_TxCpltCallback()**

​	数据发送完成调用回调

- **HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)**;

  串口中断模式接收--中断接收使能

- **HAL_UART_RxCpltCallback()**

  单字节中断，循环多收，设定缓存区
  
  **固定字符接收**
  
  ---------------------------------------------f4不可用----------------------------------------------------

```c

#define BUFFER_SIZE 100

uint8_t rxBuffer[BUFFER_SIZE];	//接受缓存区
uint8_t receivedString[] = "OK";  //校验字符串
uint8_t receivedIndex = 0;		//接受区索引
uint8_t stringMatched = 0;		//完全接收标志位

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
      //丢包检测
  	if(receivedIndex >= BUFFER_SIZE){
  		receivedIndex = 0;
  		memset(rxBuffer,0x00,RxBuffer);
  		HAL_UART_Transmit(&huart1, (uint8_t *)"数据溢出", 10,0xFFFF);
  	}
	//固定字符串解析
    if (rxBuffer[receivedIndex] == receivedString[receivedIndex])
    {
      receivedIndex++; 
      if (receivedIndex == sizeof(receivedString) - 1)
      {
        stringMatched = 1;
      }
    }
    else
    {
      receivedIndex = 0;
    }
    //清除寄存器标志位
    HAL_UART_Receive_IT(&huart1, &rxBuffer[receivedIndex], 1);  
  }
}


int main(void){
	 HAL_UART_Receive_IT(&huart1, &rxBuffer[receivedIndex], 1);
	while{
	    if (stringMatched)
    {
      // 接收到了预定的字符串 "OK"
      // 执行相应的操作
      stringMatched = 0;
      receivedIndex = 0;
    }
	}
}
```

**不定长中断接受**

```
char c;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
	Receive(c);
    //清除寄存器标志位
    HAL_UART_Receive_IT(&huart1, &rxBuffer[receivedIndex], 1);  
  }
}

int main(void){
	 HAL_UART_Receive_IT(&huart1, &c, 1);
	while{
	}
}
```



### DMA收发数据

**HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)**;串口DMA模式 

**HAL_UART_TxCpltCallback()**:发送完成回调

**HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)**;/发送一半回调函数

发送缓冲区一般的数据被发送

**不定长数据接收**  

- 空闲中断模式

**HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)**;*//接收完成回调函数

**HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)**;*//接收一半回调函数*

**注：**

//main.c

```
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>

#define BUFF_SIZE	100

uint8_t rx_buff[BUFF_SIZE];

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* 需要在初始化时调用一次否则无法接收到内容 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buff, BUFF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断	
  while (1)
  {

  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if(huart->Instance == USART1)
    {
        if (Size <= BUFF_SIZE)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buff, BUFF_SIZE); // 接收完毕后重启
            HAL_UART_Transmit(&huart1, rx_buff, Size, 0xffff);         // 将接收到的数据再发出
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT过半中断
            memset(rx_buff, 0, BUFF_SIZE);							   // 清除接收缓存
        }
        else  // 接收数据长度大于BUFF_SIZE，错误处理
        {
//            fprintf(stderr, "DMA transmit is overflowed");
//			  return -1; //异常退出
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == USART1)
    {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buff, BUFF_SIZE); // 接收发生错误后重启
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
		memset(rx_buff, 0, BUFF_SIZE);							   // 清除接收缓存
        
    }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }

}

```

//usart.h

```
/* USER CODE BEGIN Private defines */
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END Private defines */

```



### printf重定向



#### IAR编译器

重定义printf-scanf-getchar函数 

- 勾选microLib

##### stm32f1xx_hal.c

```
#include "stm32f1xx_hal.h"
#include <stdio.h>
extern UART_HandleTypeDef huart1;   //声明串口
/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx -----------------micro//
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：硬件抽象层，可移植性好，开销略大
  */
int fputc(int ch, FILE *f)
{
 
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
 
  return ch;
 
}
/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx-----------------------F4不可用//
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：硬件底层，可移植性差，高效
  */
int fputc(int ch, FILE *p)
{
	while(!(USART1->SR & (1<<7)));    //检查SR标志位是否清零
	USART1->DR =(uint8_t) ch;                  //将内容赋给DR位输出
	
	return ch;
}
/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}
/*-------------------------------------------------------*/
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{
 
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
 
  return ch;
 
}
#endif 
```





##### main.c

```
#define RXBUFFERSIZE 256
char RxBuffer[RXBUFFERSIZE];
while(1){
	printf("yes");
	HAL_Delay(1000);
}
```

##### printf变参函数

```
/*char *itoa( int value, char *string,int radix);
value：欲转换的数据。
string：目标字符串的地址。
radix：转换后的进制数，可以是10进制、16进制等。
*/
void print(char *Data, ... )
{
	 const char *s;
	int d;
	char buf[16];
	uint8_t txdata;
	va_list ap;
	va_start(ap, Data);
	while (*Data != 0) // 判断是否到达字符串结束符
	{
		if (*Data == 0x5c) //'\'
		{
				switch (*++Data)
				{
				case 'r': // 回车符
					txdata = 0x0d;
					HAL_UART_Transmit(&huart1, &txdata, 1, 0xFF);
					Data++;
					break;
				case 'n': // 换行符
					txdata = 0x0a;
					HAL_UART_Transmit(&huart1, &txdata, 1, 0xFF);
					Data++;
					break;
				default:
					Data++;
					break;
				}
}
		else if (*Data == '%')
		{
			switch (*++Data)
			{
				case 's': // 字符串
				s = va_arg(ap, const char *);
				for (; *s; s++)
				{
					HAL_UART_Transmit(&huart1, (uint8_t *)s, 1, 0xFF);
					while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == 0);
				}
				Data++;
				break;
				case 'd': // 十进制
				d = va_arg(ap, int);
				itoa(d, buf, 10);
				for (s = buf; *s; s++)
				{
					HAL_UART_Transmit(&huart1, (uint8_t *)s, 1, 0xFF);
					while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == 0);
				}
				Data++;
				break;
				default:
				Data++;
				break;
				}
			}
			else
			{
					HAL_UART_Transmit(&huart1, (uint8_t *)Data, 1, 0xFF);
					Data++;
					while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == 0);
			}
		}
}


```

- 调用

```
usart.c
#include "retarget.h"
RetargetInit(&huart1);
//main
char buf[100];
printf("\r\nYour name: ");
scanf("%s", buf);
printf("\r\nHello, %s!\r\n", buf);
//断言机制
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
    while(1);
}
#end

```

#### gcc编译器

##### stm32f1xx_hal.c

- 法一：重写_write() Vscode

```
int _write(int fd, char *pBuffer, int size)
{
    // for (int i = 0; i < size; i++)
    // {
    //     while ((USART1->SR & 0X40) == 0)
    //         ;                     //等待上一次串口数据发送完成
    //     USART1->DR = (u8)pBuffer; //写DR,串口1将发送数据
    // }
    HAL_UART_Transmit(&huart1, (uint8_t *)pBuffer, size, 0xff);
    return size;
 }
```

- 法二：通用CLion 

```
ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker-Libraries-Small printf
     set to Yes) calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}
#if defined(__GNUC__)
int _write(int fd, char * ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  return len;
}
#endif
```

- #### cmake/gcc-arm-none-eabi.cmake

```
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs -Wl,--gc-sections -u _printf_float")
```



### CRC校验

```
//------------------------------数据发送
void Send(const uint8_t *data,uint8_t len)
{
	uint8_t i;
	for (i = 0; i < len; i++)
	{
		SendByte(data[i]);//发送一个字节
	}
}
//CRC16校验代码
uint16_t CRC16_Check(const uint8_t *data,uint8_t len)
{
    uint16_t CRC16 = 0xFFFF;
    uint8_t state,i,j;
    for(i = 0; i < len; i++ )
    {
        CRC16 ^= data[i];
        for( j = 0; j < 8; j++)
        {
            state = CRC16 & 0x01;
            CRC16 >>= 1;
            if(state)
            {
                CRC16 ^= 0xA001;
            }
        }
    }
    return CRC16;
}
//数据帧打包代码
void Send_Cmd_Data(uint8_t cmd,const uint8_t *datas,uint8_t len)
{
    uint8_t buf[300],i,cnt=0;
    uint16_t crc16;
    buf[cnt++] = 0xA5;
    buf[cnt++] = 0x5A;
    buf[cnt++] = len;
    buf[cnt++] = cmd;
    for(i=0;i<len;i++)
    {
        buf[cnt++] = datas[i];
    }
    crc16 = CRC16_Check(buf,len+4);
    buf[cnt++] = crc16>>8;
    buf[cnt++] = crc16&0xFF;
    buf[cnt++] = 0xFF;
    Send(buf,cnt);//调用数据帧发送函数将打包好的数据帧发送出去
}

//-----------------------------------接受
//数据处理
void Data_Analysis(uint8_t cmd,const uint8_t *datas,uint8_t len)
{
	//根据需要处理数据
}
//CRC16校验代码
uint16_t CRC16_Check(const uint8_t *data,uint8_t len)
{
    uint16_t CRC16 = 0xFFFF;
    uint8_t state,i,j;
    for(i = 0; i < len; i++ )
    {
        CRC16 ^= data[i];
        for( j = 0; j < 8; j++)
        {
            state = CRC16 & 0x01;
            CRC16 >>= 1;
            if(state)
            {
                CRC16 ^= 0xA001;
            }
        }
    }
    return CRC16;
}
// 状态机解析数据
//接收数据
void Receive(uint8_t bytedata)
{
	static uint8_t step=0,//状态变量初始化为0 在函数中必须为静态变量
	static uint8_t cnt=0,Buf[300],len,cmd,*data_ptr;
	static uint16_t crc16;
	//进行数据解析 状态机
	switch(step)
	{
	    case 0://接收帧头1状态
	        if(bytedata== 0xA5)
	        {
	            step++;
	            cnt = 0;
	            Buf[cnt++] = bytedata;
	        }break;
	    case 1://接收帧头2状态
	        if(bytedata== 0x5A)
	        {
	            step++;
	            Buf[cnt++] = bytedata;
	        }
	        else if(bytedata== 0xA5)
	        {
	            step = 1;
	        }
	        else
	        {
	            step = 0;
	        }
	        break;
	    case 2://接收数据长度字节状态
	        step++;
	        Buf[cnt++] = bytedata;
	        len = bytedata;
	        break;
	    case 3://接收命令字节状态
	        step++;
	        Buf[cnt++] = bytedata;
	        cmd = bytedata;
	        data_ptr = &Buf[cnt];//记录数据指针首地址
	        if(len == 0)step++;//数据字节长度为0则跳过数据接收状态
	        break;
	    case 4://接收len字节数据状态
	        Buf[cnt++] = bytedata;
	        if(data_ptr + len == &Buf[cnt])//利用指针地址偏移判断是否接收完len位数据
	        {
	            step++;
	        }
	        break;
	    case 5://接收crc16校验高8位字节
	        step++;
	        crc16 = bytedata;
	        break;
	    case 6://接收crc16校验低8位字节
	        crc16 <<= 8;
	        crc16 += bytedata;
	        if(crc16 == CRC16_Check(Buf,cnt))//校验正确进入下一状态
	        {
	            step ++;
	        }
	        else if(bytedata == 0xA5)
	        {
	            step = 1;
	        }
	        else
	        {
	            step = 0;
	        }
	        break;
	    case 7://接收帧尾
	        if(bytedata== 0xFF)//帧尾接收正确
	        {
	        	Data_Analysis(cmd,data_ptr,len);//数据解析
	            step = 0;
	        }
	        else if(bytedata == 0xA5)
	        {
	            step = 1;
	        }
	        else
	        {
	            step = 0;
	        }
	        break;
	    default:step=0;break;//多余状态，正常情况下不可能出现
	}
}

```

### 数据包解析

- 任务中进行--保证传输速率
- 创建任务轮询解析缓冲区--轮询完置0--发布信号量 命令

```
void data_analysis( , uint16_t Size)
{
        if (Size <= BUFF_SIZE)
        {	
			if(rx_buff[0] == 0xAA){
				if(rx_buff[1] == Size){
					uint8_t sum = 0;
					for(int i = 0; i<Size - 1; i++){
						sum += rx_buff[i];
					}
					if(sum == rx_buff[Size - 1]){
						for(int i = 2; i < Size-1; i+=2){
							int status = 0;
							if(rx_buff[i+=1]){
								status = 1;
							}
							switch(rx_buff[i]){
								case 0x01 :
									break;
								default : break;
							}
						}
					}
				}
			}
        }
    }
}

```



# 延时函数

### Arduino的micros

非阻塞（波形较不稳定，周期准确度较高）

#### Microsoft.c

```
#include "micros.h"
 
 
__STATIC_INLINE uint32_t GXT_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}
 
 
static uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  GXT_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (GXT_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}
 
 
//获取系统时间，单位us
uint32_t micros(void)
{
  return getCurrentMicros();
}
 
 
 
//微秒级延时函数
void Delay_us(uint16_t nus)
{
    int temp = micros();
    while(micros() - temp < nus){}
}
```

#### micros.h

```
#ifndef __MICROS_H__
#define __MICROS_H__
 
#include "stm32f4xx_hal.h"
 
uint32_t micros(void);
void Delay_us(uint16_t nus);
 
#endif
```

### 定时器中断

波形较稳定，周期稳定地偏长0.3-0.4us

```
/**
  * @brief  定时器延时us，Prescaler -> 168-1
  * @param  us: <= 65535
  * @retval None
  */
 
void Delay_us(uint16_t nus)
{
	__HAL_TIM_SetCounter(&htim1,0);
	__HAL_TIM_ENABLE(&htim1);
	while (__HAL_TIM_GET_COUNTER(&htim1) < nus)
	{
	}
	__HAL_TIM_DISABLE(&htim1);
}
```

### 纯代码

#### 1  Delay_us.c(波形较稳定，周期稳定地偏长0.3-0.4us)

```
__IO float usDelayBase;
 
void PY_usDelayTest(void)
{
  __IO uint32_t firstms, secondms;
  __IO uint32_t counter = 0;
 
  firstms = HAL_GetTick()+1;
  secondms = firstms+1;
 
  while(uwTick!=firstms) ;
 
  while(uwTick!=secondms) counter++;
 
  usDelayBase = ((float)counter)/1000;
}
 
 
 
void PY_Delay_us_t(uint32_t Delay)
{
  __IO uint32_t delayReg;
  __IO uint32_t usNum = (uint32_t)(Delay*usDelayBase);
 
  delayReg = 0;
  while(delayReg!=usNum) delayReg++;
}
 
 
 
void PY_usDelayOptimize(void)
{
  __IO uint32_t firstms, secondms;
  __IO float coe = 1.0;
 
  firstms = HAL_GetTick();
  PY_Delay_us_t(1000000) ;
  secondms = HAL_GetTick();
 
  coe = ((float)1000)/(secondms-firstms);
  usDelayBase = coe*usDelayBase;
}
 
 
 
void Delay_us(uint16_t nus)
{
  __IO uint32_t delayReg;
 
  __IO uint32_t msNum = nus/1000;
  __IO uint32_t usNum = (uint32_t)((nus%1000)*usDelayBase);
 
  if(msNum>0) HAL_Delay(msNum);
 
  delayReg = 0;
  while(delayReg!=usNum) delayReg++;
}
```

#### 2 Delay_us.c（波形较稳定，偏差稳定0.5us）

```
#define CPU_FREQUENCY_MHZ    168        //STM32主频
 
void Delay_us(__IO uint32_t delay)
{
    int last, curr, val;
    int temp;
    while (delay != 0)
    {
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0){
            do{
                val = SysTick->VAL;
            }
            while ((val < last) && (val >= curr));
        }
        else{
            curr += CPU_FREQUENCY_MHZ * 1000;
            do{
                val = SysTick->VAL;
            }
            while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }
}
```

### HAL库

#### Delay_us.c

```
#define CPU_FREQUENCY_MHZ 168				/* CPU主频，根据实际进行修改 */
 
 
/**
 * 此延时函数代码适用于HAL库
 */
void delay_us(uint32_t delay)
{
    int last, curr, val;
    int temp;
    while (delay != 0){
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0){
            do{
                val = SysTick->VAL;
            }
            while ((val < last) && (val >= curr));
        }
        else{
            curr += CPU_FREQUENCY_MHZ * 1000;
            do{
                val = SysTick->VAL;
            }
            while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }
}
```





# 舵机控制

一个20ms左右的时基脉冲，该脉冲的高电平部分（即PWM的正占空比）一般在0.5ms-2.5ms范围内，决定了舵机转动的角度范围为0°到180

pwm占空比与舵机控制占空比即可对应为50——0度  150——90度 250——180度

占空比：高电平时间的时基单元个数占脉冲宽度(Counter Period)的比

- **__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,150);**

```
 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
 
/* USER CODE BEGIN 4 */
void Servo_Control(uint16_t angle)   //参数为舵机转动角度 0-180
 {
    float temp;
    temp =angle/9*10+50 ;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t )temp);
 }
/* USER CODE END 4 */
```

# CAN通信

- 传输方式

**要在单片机上获取CAN信号，确实需要通过CAN收发器将单片机的CAN收发引脚与CAN总线的高低线连接起来**。

对于单片机来说，其内部集成的CAN控制器负责处理CAN协议的相关通信功能，但在与物理总线交换信号时，由于单片机的电气特性与CAN总线的电气特性不匹配，直接连接可能会造成信号不匹配甚至损坏单片机。因此，必须通过专门的CAN收发器来实现这一接口转换。



- 输出信号

**CAN收发器的收发引脚实际传输的是TTL信号（逻辑电平）**。

CAN收发器在CAN通信系统中起着至关重要的接口转换作用，它将单片机或微控制器的TTL电平信号转换为CAN总线上的差分信号，从而实现与总线上其他节点的通信。同时，它还能将来自总线的差分信号转换回TTL信号，供单片机处理

can控制器(ttl信号)--总线差分信号



- CAN总线连接多个设备。

CAN总线采用**多主机**工作方式，通过报文标识符来传输数据，在传输数据时，发送方会将数据的标识符和数据一同发送出去，而接收方则会根据标识符来确定是否需要接收这个数据。

因此，CAN总线上可以连接多个设备，这些设备可以是微控制器、传感器、执行器等，它们通过标准或扩展格式的标识符来进行数据的发送和接收。

此外，CAN总线上还可以连接网关，用于实现不同网络之间的数据交换。

总之，CAN总线的多主机工作方式和报文传输特性使得它可以方便地连接多个设备，并且具有较高的实时性和可靠性。



- CAN总线一发多收

CAN总线作为一种多主机系统的串行通讯总线，其设计初衷就是允许多个节点在网络上进行通信。在CAN总线的传输过程中，确实只能由一个节点占用总线来发送消息，但是同时其他所有节点仍然能够接收总线上的数据。



- 通讯帧--CAN总线通信的基本单位--总线上传输数据

1. **帧类型和作用**
   - **数据帧**：数据帧的主要功能是传输数据。它由发射端发送到接收端，由七个部分组成。
   - **远程帧**：远程帧用于请求其他节点发送数据。它没有数据场，其他部分与数据帧相似。
     - 由需要数据的节点发送到总线上，用于请求其他节点发送具有相同标识符的数据帧。
     - 帧起始、仲裁场、控制场、CRC场、应答场和帧结束--无数据场
     - 区别：
       - 远程帧的RTR位为隐性状态，而数据帧的RTR位为显性状态
       - 远程帧不包含数据场，仅有帧起始、仲裁场、控制场、CRC场、应答场和帧结束
     - 紧急数据请求
   - **错误帧**：错误帧用于指示检测到总线错误的节点。它由两个字段组成，错误标志和错误界定符。
     - 不是应答场--CAN总线通信中两种不同的信号类型，用于不同的目的和场景
     - 由错误标志和错误界定符组成：在接收和发送报文时检测出错误而通知错误的帧
     - 总线上的任何节点检测到数据传输错误时，便可以发送错误帧以通知其他节点发生了错误。错误帧的存在对于保证数据传输的准确性和可靠性至关重要，它帮助节点识别并处理通信过程中出现的问题
     - 应答场是数据帧和远程帧的一部分，用于确认接收节点是否成功接收到了报文
     - **错误帧**不是由接收节点发出的，而是由**检测到错误的节点**发送的。自定义的**错误标志**指明了错误类型，**界定符**指明了错误帧的结尾
       - 总线上的任何节点检测到数据传输错误时，该节点会发送一个错误帧，以通知其他节点发生了错误
     - **错误标志**根据节点的错误状态不同，分为主动错误标志和被动错误标志。主动错误标志由6个连续的显性位组成，而被动错误标志由6个连续的隐性位组成。这种区分主要是为了识别是处于活动错误状态还是被动错误状态的节点发现了错误
     - **错误界定符**则是由8个连续的隐性位组成，它标志着错误帧的结束，并提供了错误帧与其后续帧之间的分隔。这一部分对于恢复总线的正常通信流程至关重要。通过这种方式，一旦错误被标记并处理，总线可以迅速恢复正常数据传输
   - **过载帧**：过载帧用于提供总线的过载保护。它在连续的数据帧之间发送，由两个字段组成，过载标志和过载界定符。
     - 过载标志和过载界定符：接收节点在未准备好接收下一帧数据时通知发送节点，以延迟数据帧的发送
     - **过载标志**由6个连续的显性位组成，这种显性位的形式破坏了间歇场的固定格式，导致其他所有节点也检测到一个出错状态，从而各自送出一个过载标志。
     - **过载界定符**则由8个连续的隐性位组成，其形式与错误界定符相似。当过载标志发送完毕后，每个节点都会对总线进行监察，直到检测到一个隐性位为止。此时，每个节点均已发送完各自的过载标志，接着所有节点还要同时开始发送7个隐性位，配齐长达8位的过载界定符。
2. **数据帧的结构**
   - **帧起始**：帧起始（SOF）是数据帧开始的标志，通常由一个显性位表示。它用于同步总线上的其他节点。
   - **仲裁场**：仲裁场包括标识符和远程发送请求（RTR）位。标识符用于确定数据的优先级和识别接收者，而RTR位则用于区分数据帧和远程帧。
   - **控制场**：控制场包括数据长度代码（DLC），它指示数据场中的字节数。保留位通常为隐性，用于未来扩展。
   - **数据场**：数据场包含实际传输的数据，长度由DLC决定。
   - **CRC场**：CRC场包括循环冗余校验值和界定符，用于检测数据传输错误。
   - **应答场**：应答场用于确认数据是否正常接收。发送节点通过监测应答位来了解是否所有接收节点都正确接收了数据。
   - **帧结尾**：帧结尾（EOF）标志着数据帧的结束，通常由连续的隐性位组成。
     - **CRC场的作用**
       1. **校验原理**：CRC场用于存储循环冗余校验值，这个值是由发送节点根据数据帧的内容计算得出的。它采用特定的生成多项式$X^{15} + X^{14} + X^{10} + X^{8} + X^{7} + X^{4} + X^{3} + 1$，通过模-2除法计算出一个15位的余数，作为CRC序列发送到总线上。
       2. **错误检测**：接收节点会对接收到的数据帧执行相同的CRC计算过程。如果计算出的CRC值与接收到的CRC场中的值不一致，则说明数据在传输过程中可能发生了错误，接收节点可以据此请求重发或采取其他错误处理措施。
       3. **界定符**：CRC场的最后一位是CRC界定符，它是一个隐性位，标志着CRC场的结束，同时也是应答场的开始。
     - **应答场的作用**
       1. **应答机制**：应答场主要用于接收节点向发送节点确认数据是否正常接收。在应答场中，发送节点会发送一个隐性的应答位和应答结束符到总线上。如果接收节点正确接收并验证了数据帧，它会立即发送一个显性位到总线上，这样，在总线上观察到的应答位应为显性。
       2. **错误处理**：如果发送节点在应答位上观察到隐性位，即表明没有接收节点正确接收数据，它将认为是数据传输错误并尝试重新发送数据帧。这个过程将持续直到数据被正确接收，或者达到一定的重发次数限制。
       3. **结束标志**：应答场之后是帧结尾，由7个连续的隐性位组成，标志着数据帧的结束。
3. **标准帧与扩展帧的区别**
   - **标识符长度**：标准帧的标识符长度为11位，而扩展帧的标识符长度可达29位，提供了更大的地址空间。
   - **格式区别**：标准帧和扩展帧在仲裁段和控制段的构成上有所不同。扩展帧增加了扩展标识符位和替代远程请求位。
4. **帧的传输和接收**
   - **发送条件**：当CAN总线空闲时，任何节点都可以开始发送数据帧。发送节点通过发送帧起始位来标志着数据帧的开始。
   - **接收过滤**：节点通过硬件过滤器对接收到的帧进行筛选，只处理与自身标识符匹配的帧。这确保了只有目标节点才会对数据帧作出响应。
5. **错误处理和过载控制**
   - **错误检测**：CAN总线具有强大的错误检测机制。错误帧用于指示总线上的节点发生了错误，而CRC场则用于检测数据帧内的错误。
   - **过载保护**：在总线负载过高时，过载帧可以提供必要的延迟，以防止总线拥塞。

- 工作模式
  - normal
    - 正常收发
  - silent
    - 正常接收，发送无效，用于监测总线状态
  - circle
    - 正常发送，接收无效，用于自检
  - silent + circle
    - 自发自收，自我检查
  
- 报文模式

  - can模块：两个接收fifo,每个fifo三个邮箱（0，1，2）

    - 接收到的报文>邮箱数，覆盖最早的报文
    - 及时处理，避免溢出

  - 接收中断、FIFO满中断和FIFO溢出中断：及时处理报文，防止数据丢失或溢出

  - **软件循环查询**：主循环或定时器中断服务程序中，定期检查CAN的接收状态，读取FIFO中的报文并清空的报文

  - 配置过滤器

  - ID越小，报文优先级越高

  - 清除：

    读取报文后将其从相应邮箱中移除即可：读取FIFO输出邮箱，这相当于从消息队列中取出队尾元素

    - **轮询**：在主循环中不断检查是否有新报文到达，如果有则立即处理。
    - **中断**：利用上述提到的各种接收中断，当有报文到达或FIFO状态改变时立即进入中断服务程序进行处理。
    - **DMA（Direct Memory Access）**：虽然STM32的CAN模块不支持DMA传输，但其他一些高级微控制器的CAN模块可能支持DMA，这样可以在不经过CPU的情况下直接存储报文到内存，减轻CPU负担并提高处理速度


```
HAL_CAN_Start	开启CAN通讯
HAL_CAN_Stop	关闭CAN通讯
HAL_CAN_RequestSleep	尝试进入休眠模式
HAL_CAN_WakeUp	从休眠模式中唤醒
HAL_CAN_IsSleepActive	检查是否成功进入休眠模式
HAL_CAN_AddTxMessage	向 Tx 邮箱中增加一个消息,并且激活对应的传输请求
HAL_CAN_AbortTxRequest	请求中断传输
HAL_CAN_GetTxMailboxesFreeLevel	Return Tx mailboxes free level
HAL_CAN_IsTxMessagePending	检查是否有传输请求在指定的 Tx 邮箱上等待
HAL_CAN_GetRxMessage	从Rx FIFO 收取一个 CAN 帧
HAL_CAN_GetRxFifoFillLevel	Return Rx FIFO fill level获取指定CAN控制器的接收FIFO（First-In First-Out）水平的当前值。这个值表示接收FIFO中当前存储了多少个消息
```

#### can_app.h

```
#ifndef _can_H
#define _can_H

#include "stm32f1xx_hal.h"
#include "can.h"

#define CAN_RX0_INT_ENABLE 0   //不使用中断

void CAN_Mode_Init(void);

uint8_t CAN_Send_Msg(uint8_t* msg,uint8_t len);						//发送数据

int16_t CAN_Receive_Msg(uint8_t *buf);							//接收数据

#endif
```

#### can_app.c

```
#include "can_app.h"

/*************************************************
*函数名：CAN_Mode_Init
*函数功能：can过滤器设置并打开can
*输入：无
*返回值：无
**************************************************/
void CAN_Mode_Init()
{
	CAN_FilterTypeDef filter = {0};
	filter.FilterActivation = ENABLE;//激活过滤器
	filter.FilterMode = CAN_FILTERMODE_IDMASK;//标识符屏蔽，选择掩码
	filter.FilterScale = CAN_FILTERSCALE_32BIT;//筛选器位宽
	filter.FilterBank = 0; //过滤器组，最多14组
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;//关联fifo
	filter.FilterIdLow = 0x0000;//接受任意ID数据
	filter.FilterIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x0000;
	filter.SlaveStartFilterBank = 14;//滤波器的起始滤波器组设置为14
	HAL_CAN_ConfigFilter(&hcan, &filter);


	HAL_CAN_Start(&hcan);

	#if CAN_RX0_INT_ENABLE
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//自动清除该中断标志位,不需要重启中断使能或重新设置CAN_IT_RX_FIFO0_MSG_PENDING标志位
	#endif
}

/*************************************************
*函数名：CAN_Send_Msg
*函数功能：can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)
*输入：len:数据长度(最大为8),msg:数据指针,最大为8个字节.
*返回值：0,成功;其他,失败;
**************************************************/
uint8_t CAN_Send_Msg(uint8_t* msg,uint8_t len)
{
	uint16_t i=0;
	uint32_t msg_box;  //存储发送函数的返回值
	uint8_t send_buf[8] = {0};
	
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.StdId=0X12;	 // 标准标识符为0
	TxMessage.ExtId=0X12;	 // 设置扩展标示符（29位）
	TxMessage.IDE=CAN_ID_STD;		  // 使用扩展标识符
	TxMessage.RTR=CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
	TxMessage.DLC=len;							 // 发送两帧信息
	TxMessage.TransmitGlobalTime = DISABLE;
	for(i=0;i<len;i++)
		send_buf[i]=msg[i];				 // 第一帧信息          
	HAL_CAN_AddTxMessage(&hcan,&TxMessage,send_buf,&msg_box);。
	return 0;		
}

/*************************************************
*函数名：CAN_Receive_Msg
*函数功能：can口接收数据查询
*输入：buf:数据缓存区;
*返回值：0,无数据被收到;-1,接收错误;其他>0,接收的数据长度;
**************************************************/
int16_t CAN_Receive_Msg(uint8_t *buf)
{
	uint32_t i;
	uint8_t recv_data[8];
	CAN_RxHeaderTypeDef rxHeader;
	
	while(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) != 0) //循环等待接收
	{
		if (__HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_FOV0) != RESET) //接收超时退出
      return -1;
		
		HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, recv_data);
		for(i=0;i<rxHeader.DLC;i++)
			buf[i]=recv_data[i];
	}
 
	return rxHeader.DLC;
}

#if CAN_RX0_INT_ENABLE
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//中断内容
    /* 获取接收到的CAN消息 */
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    
    if(hcan->pRxBuffPtr != NULL)
    {
        rxHeader = hcan->pRxBuffPtr->RxHeader;
        memcpy(rxData, hcan->pRxBuffPtr->Data, rxHeader.DLC);
        
        /* 判断ID并从数据帧中提取数据 */
        if(rxHeader.StdId == 0x123)
        {
            /* 提取数据 */
            uint8_t data = rxData[0]; // 假设我们需要的数据在第一个字节
            
            /* 处理数据... */
        }
    }
}
#endif
```

#### main.c

```
#include "can_app.h"

CAN_Mode_Init();
uint8_t msg_send[1], msg_receive[1];


while(1){
/************1s循环，当接收的信息等于发送的信息时，LED灯取反否则常亮************/
	HAL_Delay(1000);
	
	CAN_Send_Msg(msg_send,1);
	CAN_Receive_Msg(msg_recv);
	if(msg_send[0] == msg_recv[0]+1)
	{
		HAL_GPIO_TogglePin(GPIOC, LED_Pin);
	}
	else
		HAL_GPIO_WritePin(GPIOC,LED_Pin,GPIO_PIN_RESET);
	
	msg_send[0] += 1;
}
```

# OLED

①屏幕初始化
②开启屏幕显示
③清屏（不清屏的话屏幕将会出现一片雪花）
④发送要显示的字符串、数字、汉字等

### 普通驱动

#### oled.h

```
#ifndef _oled_H
#define _oled_H

#include "stm32f1xx_hal.h"
unsigned char Hzk[][16]=
{
 {0x00,0x80,0xC0,0xBC,0x84,0x84,0x84,0xF4,0x82,0x82,0x83,0x82,0x80,0xC0,0x80,0x00},
 {0x00,0x40,0x20,0x10,0x0C,0x40,0x80,0x7F,0x00,0x00,0x04,0x08,0x30,0x60,0x00,0x00},
 {0x00,0xFE,0x22,0xFE,0x00,0xFE,0x22,0xFE,0x00,0xFC,0x06,0x55,0x84,0x7E,0x04,0x00},
 {0x40,0x3F,0x02,0x3F,0x40,0x3F,0x42,0x7F,0x10,0x13,0x12,0x12,0x5A,0x92,0x7E,0x00},
};

//写命令
void OLED_WR_CMD(uint8_t cmd); 
//写数据
void OLED_WR_DATA(uint8_t data);
//屏幕初始化部分
void OLED_Init()；
//开启OLED显示    
void OLED_Display_On(void);
//关闭OLED显示     
void OLED_Display_Off(void);
//清屏操作
void OLED_Clear();

//输入内容为oled屏幕的显示坐标，要显示的字符、字符大小
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size);
//显示一个字符号串
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size);
//显示汉字
//hzk 用取模软件得出的数组
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);
//水平滚动
void OLED_ver_scroll();

#endif
```

#### oled.c

```
#include "oled.h"
//写命令
void OLED_WR_CMD(uint8_t cmd)
{
	HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,&cmd,1,0x100);
}
//写数据
void OLED_WR_DATA(uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,&data,1,0x100);
}
//屏幕初始化
void OLED_Init()；
{
    OLED_WR_CMD(0xAE);//--turn off oled panel
	OLED_WR_CMD(0x00);//---set low column address
	OLED_WR_CMD(0x10);//---set high column address
	OLED_WR_CMD(0x40);//--set start line address  Set Mapping RAM Display Start Line 
	OLED_WR_CMD(0x81);//--set contrast control register
	OLED_WR_CMD(0xCF);// Set SEG Output Current Brightness
	OLED_WR_CMD(0xA1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	OLED_WR_CMD(0xC8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	OLED_WR_CMD(0xA6);//--set normal display
	OLED_WR_CMD(0xA8);//--set multiplex ratio(1 to 64)
	OLED_WR_CMD(0x3f);//--1/64 duty
	OLED_WR_CMD(0xD3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WR_CMD(0x00);//-not offset
	OLED_WR_CMD(0xd5);//--set display clock divide ratio/oscillator frequency
	OLED_WR_CMD(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WR_CMD(0xD9);//--set pre-charge period
	OLED_WR_CMD(0xF1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WR_CMD(0xDA);//--set com pins hardware configuration
	OLED_WR_CMD(0x12);
	OLED_WR_CMD(0xDB);//--set vcomh
	OLED_WR_CMD(0x40);//Set VCOM Deselect Level
	OLED_WR_CMD(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WR_CMD(0x02);//
	OLED_WR_CMD(0x8D);//--set Charge Pump enable/disable
	OLED_WR_CMD(0x14);//--set(0x10) disable
	OLED_WR_CMD(0xA4);// Disable Entire Display On (0xa4/0xa5)
	OLED_WR_CMD(0xA6);// Disable Inverse Display On (0xa6/a7) 
	OLED_Clear();
	OLED_WR_CMD(0xAF);
}
//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_CMD(0X8D);  //SET DCDC命令
	OLED_WR_CMD(0X14);  //DCDC ON
	OLED_WR_CMD(0XAF);  //DISPLAY ON
}
 
//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_CMD(0X8D);  //SET DCDC命令
	OLED_WR_CMD(0X10);  //DCDC OFF
	OLED_WR_CMD(0XAE);  //DISPLAY OFF
}		
//清屏操作
void OLED_Clear()
{
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_CMD(0xb0+i);
		OLED_WR_CMD (0x00); 
		OLED_WR_CMD (0x10); 
		for(n=0;n<128;n++)
			OLED_WR_DATA(0x00);
	} 
}
/*
输入内容为oled屏幕的显示坐标，要显示的字符、字符大小
*/
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size)
{      	
	unsigned char c=0,i=0;	
		c=chr-' ';//得到偏移后的值			
		if(x>128-1){x=0;y=y+2;}
		if(Char_Size ==16)
			{
			OLED_Set_Pos(x,y);	
			for(i=0;i<8;i++)
			OLED_WR_DATA(F8x16[c*16+i]);
			OLED_Set_Pos(x,y+1);
			for(i=0;i<8;i++)
			OLED_WR_DATA(F8x16[c*16+i+8]);
			}
			else {	
				OLED_Set_Pos(x,y);
				for(i=0;i<6;i++)
				OLED_WR_DATA(F6x8[c][i]);
				
			}
}
 
//显示一个字符号串
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar(x,y,chr[j],Char_Size);
			x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}
 
 
unsigned char Hzk[][16]=
{
 
 {0x00,0x80,0xC0,0xBC,0x84,0x84,0x84,0xF4,0x82,0x82,0x83,0x82,0x80,0xC0,0x80,0x00},
 {0x00,0x40,0x20,0x10,0x0C,0x40,0x80,0x7F,0x00,0x00,0x04,0x08,0x30,0x60,0x00,0x00},
 
 {0x00,0xFE,0x22,0xFE,0x00,0xFE,0x22,0xFE,0x00,0xFC,0x06,0x55,0x84,0x7E,0x04,0x00},
 {0x40,0x3F,0x02,0x3F,0x40,0x3F,0x42,0x7F,0x10,0x13,0x12,0x12,0x5A,0x92,0x7E,0x00},
};
 
//显示汉字
//hzk 用取模软件得出的数组
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no)
{      			    
	uint8_t t,adder=0;
	OLED_Set_Pos(x,y);	
    for(t=0;t<16;t++)
		{
				OLED_WR_DATA(Hzk[2*no][t]);
				adder+=1;
     }	
		OLED_Set_Pos(x,y+1);	
    for(t=0;t<16;t++)
			{	
				OLED_WR_DATA(Hzk[2*no+1][t]);
				adder+=1;
      }					
}
 
//水平滚动
void OLED_ver_scroll()
{
  OLED_WR_CMD(0x2e);//关滚动
  OLED_WR_CMD(0x27);//29向右，2a向左
  OLED_WR_CMD(0x00);//A:空字节
  OLED_WR_CMD(0x02);//B:水平起始页
  OLED_WR_CMD(0x07);//C:水平滚动速度
  OLED_WR_CMD(0x03);//D:水平结束页
  OLED_WR_CMD(0x00);//E:每次垂直滚动位移
  OLED_WR_CMD(0xFF);//E:每次垂直滚动位移
  OLED_WR_CMD(0x2f);//开滚动
}
```

#### main.c

```
int main(){
  MX_I2C1_Init();
  uint8_t A[]="hellow world !!!!";
  uint8_t B[]="hellow world !!";
  OLED_Init();
  HAL_Delay(500);
  OLED_Display_On();
  OLED_Clear();
 
  /*OLED_ShowString(0,0,A,sizeof(A));
	OLED_ShowString(0,4,A,sizeof(B));*/
  OLED_ShowCHinese(0,2,0);
  OLED_ShowCHinese(16,2,1);
  OLED_ver_scroll();
  while (1)
  {
  }
}
```

### u8g2库函数

#### stm32_u8g2.h

```
#ifndef __STM32_U8G2_H
#define __STM32_U8G2_H
 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "u8g2.h"
/* USER CODE BEGIN Includes */
 
/* USER CODE END Includes */
 
 
 
/* USER CODE BEGIN Private defines */
 
/* USER CODE END Private defines */
#define u8         unsigned char  // ?unsigned char ????
#define MAX_LEN    128  //
#define OLED_ADDRESS  0x78 // oled
#define OLED_CMD   0x00  // 
#define OLED_DATA  0x40  // 
 
/* USER CODE BEGIN Prototypes */
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void u8g2Init(u8g2_t *u8g2);
void draw(u8g2_t *u8g2);
void testDrawPixelToFillScreen(u8g2_t *u8g2);
 
#endif
```

#### stm32_u8g2.c

```
#include "stm32_u8g2.h"
#include "tim.h"
#include "i2c.h"
 
 
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
    static uint8_t buffer[128];
    static uint8_t buf_idx;
    uint8_t *data;
 
    switch (msg)
    {
    case U8X8_MSG_BYTE_INIT:
    {
        /* add your custom code to init i2c subsystem */
        MX_I2C2_Init(); //I2C初始化
    }
    break;
 
    case U8X8_MSG_BYTE_START_TRANSFER:
    {
        buf_idx = 0;
    }
    break;
 
    case U8X8_MSG_BYTE_SEND:
    {
        data = (uint8_t *)arg_ptr;
 
        while (arg_int > 0)
        {
            buffer[buf_idx++] = *data;
            data++;
            arg_int--;
        }
    }
    break;
 
    case U8X8_MSG_BYTE_END_TRANSFER:
    {
        if (HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDRESS, buffer, buf_idx, 1000) != HAL_OK)
            return 0;
    }
    break;
 
    case U8X8_MSG_BYTE_SET_DC:
        break;
 
    default:
        return 0;
    }
 
    return 1;
}
 
 
 
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
    case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
        __NOP();
        break;
    case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
        for (uint16_t n = 0; n < 320; n++)
        {
            __NOP();
        }
        break;
    case U8X8_MSG_DELAY_MILLI: // delay arg_int * 1 milli second
        HAL_Delay(1);
        break;
    case U8X8_MSG_DELAY_I2C: // arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
        Tims_delay_us(5);
        break;                    // arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
    case U8X8_MSG_GPIO_I2C_CLOCK: // arg_int=0: Output low at I2C clock pin
        break;                    // arg_int=1: Input dir with pullup high for I2C clock pin
    case U8X8_MSG_GPIO_I2C_DATA:  // arg_int=0: Output low at I2C data pin
        break;                    // arg_int=1: Input dir with pullup high for I2C data pin
    case U8X8_MSG_GPIO_MENU_SELECT:
        u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_NEXT:
        u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_PREV:
        u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_HOME:
        u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
        break;
    default:
        u8x8_SetGPIOResult(u8x8, 1); // default return value
        break;
    }
    return 1;
}
 
//U8g2的初始化，需要调用下面这个u8g2_Setup_ssd1306_128x64_noname_f函数，该函数的4个参数含义：
//u8g2：传入的U8g2结构体
//U8G2_R0：默认使用U8G2_R0即可（用于配置屏幕是否要旋转）
//u8x8_byte_sw_i2c：使用软件IIC驱动，该函数由U8g2源码提供
//u8x8_gpio_and_delay：就是上面我们写的配置函数
 
void u8g2Init(u8g2_t *u8g2)
{
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay); // 初始化u8g2 结构体
	u8g2_InitDisplay(u8g2);                                                                       // 
	u8g2_SetPowerSave(u8g2, 0);                                                                   // 
	u8g2_ClearBuffer(u8g2);
}
 
 
void draw(u8g2_t *u8g2)
{
	u8g2_ClearBuffer(u8g2); 
	
    u8g2_SetFontMode(u8g2, 1); /*字体模式选择*/
    u8g2_SetFontDirection(u8g2, 0); /*字体方向选择*/
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf); /*字库选择*/
    u8g2_DrawStr(u8g2, 0, 20, "U");
    
    u8g2_SetFontDirection(u8g2, 1);
    u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
    u8g2_DrawStr(u8g2, 21,8,"8");
        
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 51,30,"g");
    u8g2_DrawStr(u8g2, 67,30,"\xb2");
    
    u8g2_DrawHLine(u8g2, 2, 35, 47);
    u8g2_DrawHLine(u8g2, 3, 36, 47);
    u8g2_DrawVLine(u8g2, 45, 32, 12);
    u8g2_DrawVLine(u8g2, 46, 33, 12);
  
    u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
    u8g2_DrawStr(u8g2, 1,54,"github.com/olikraus/u8g2");
		
	u8g2_SendBuffer(u8g2);
	HAL_Delay(1000);
}
 
//画点填充
void testDrawPixelToFillScreen(u8g2_t *u8g2)
{
  int t = 1000;
	u8g2_ClearBuffer(u8g2);
 
  for (int j = 0; j < 64; j++)
  {
    for (int i = 0; i < 128; i++)
    {
      u8g2_DrawPixel(u8g2,i, j);
    }
  }
  HAL_Delay(1000);
}
```

#### test.h--测试

```
#ifndef __TEST_H
#define __TEST_H
 
#include "main.h"
#include "u8g2.h"
 
void testDrawProcess(u8g2_t *u8g2);
void testShowFont(u8g2_t *u8g2);
void testDrawFrame(u8g2_t *u8g2);
void testDrawRBox(u8g2_t *u8g2);
void testDrawCircle(u8g2_t *u8g2);
void testDrawFilledEllipse(u8g2_t *u8g2);
void testDrawMulti(u8g2_t *u8g2);
void testDrawXBM(u8g2_t *u8g2);
 
void u8g2DrawTest(u8g2_t *u8g2);
 
#endif
```

#### test.c--测试

```
#include "test.h"
 
//---------------U8g2测试函数
 
#define SEND_BUFFER_DISPLAY_MS(u8g2, ms)\
  do {\
    u8g2_SendBuffer(u8g2); \
    HAL_Delay(ms);\
  }while(0);
 
 
//进度条显示
void testDrawProcess(u8g2_t *u8g2)
{
	for(int i=10;i<=80;i=i+2)
	{
		u8g2_ClearBuffer(u8g2); 
			
		char buff[20];
		sprintf(buff,"%d%%",(int)(i/80.0*100));
		
		u8g2_SetFont(u8g2,u8g2_font_ncenB12_tf);
		u8g2_DrawStr(u8g2,16,32,"STM32 U8g2");//字符显示
		
		u8g2_SetFont(u8g2,u8g2_font_ncenB08_tf);
		u8g2_DrawStr(u8g2,100,49,buff);//当前进度显示
		
		u8g2_DrawRBox(u8g2,16,40,i,10,4);//圆角填充框矩形框
		u8g2_DrawRFrame(u8g2,16,40,80,10,4);//圆角矩形
		
		u8g2_SendBuffer(u8g2);
	}
	HAL_Delay(500);
}
 
 
//字体测试 数字英文可选用 u8g2_font_ncenB..(粗) 系列字体
//u8g2_font_unifont_t_symbols/u8g2_font_unifont_h_symbols(细 圆润)
void testShowFont(u8g2_t *u8g2)
{
	int t = 1000;
	char testStr[14] = "STM32F103C8T6";
	
	u8g2_ClearBuffer(u8g2);
	
	u8g2_SetFont(u8g2,u8g2_font_u8glib_4_tf);
	u8g2_DrawStr(u8g2,0,5,testStr);
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	
	u8g2_SetFont(u8g2,u8g2_font_ncenB08_tf);
	u8g2_DrawStr(u8g2,0,30,testStr);
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	
    u8g2_SetFont(u8g2,u8g2_font_ncenB10_tr);
	u8g2_DrawStr(u8g2,0,60,testStr);
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
}
 
//画空心矩形
void testDrawFrame(u8g2_t *u8g2)
{
	int t = 1000;
	int x = 16;
	int y = 32;
	int w = 50;
	int h = 20;
	u8g2_ClearBuffer(u8g2);
	u8g2_DrawStr(u8g2,0, 15, "DrawFrame");
 
	u8g2_DrawFrame(u8g2, x, y, w, h);
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	u8g2_DrawFrame(u8g2, x+w+5, y-10, w-20, h+20);
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
}
 
//画实心圆角矩形
void testDrawRBox(u8g2_t *u8g2)
{
	int t = 1000;
	int x = 16;
	int y = 32;
	int w = 50;
	int h = 20;
	int r = 3;
	u8g2_ClearBuffer(u8g2);
	u8g2_DrawStr(u8g2,0, 15, "DrawRBox");
 
	u8g2_DrawRBox(u8g2, x, y, w, h, r);
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	u8g2_DrawRBox(u8g2, x+w+5, y-10, w-20, h+20, r);
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
}
 
//画空心圆
void testDrawCircle(u8g2_t *u8g2)
{
	int t = 600;
	int stx = 0;  //画图起始x
	int sty = 16; //画图起始y
	int with = 16;//一个图块的间隔
	int r = 15;   //圆的半径
	u8g2_ClearBuffer(u8g2);
	u8g2_DrawStr(u8g2, 0, 15, "DrawCircle");
 
	u8g2_DrawCircle(u8g2, stx, sty - 1 + with, r, U8G2_DRAW_UPPER_RIGHT); //右上
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	u8g2_DrawCircle(u8g2, stx + with, sty, r, U8G2_DRAW_LOWER_RIGHT); //右下
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	u8g2_DrawCircle(u8g2, stx - 1 + with * 3, sty - 1 + with, r, U8G2_DRAW_UPPER_LEFT); //左上
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	u8g2_DrawCircle(u8g2, stx - 1 + with * 4, sty, r, U8G2_DRAW_LOWER_LEFT); //左下
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	u8g2_DrawCircle(u8g2, stx - 1 + with * 2, sty - 1 + with * 2, r, U8G2_DRAW_ALL);//整个圆
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	
    u8g2_DrawCircle(u8g2, 32*3, 32, 31, U8G2_DRAW_ALL);//右侧整个圆
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
}
 
//画实心椭圆
void testDrawFilledEllipse(u8g2_t *u8g2)
{
	int t = 800;
	int with = 16;//一个图块的间隔
	int rx = 27;  //椭圆x方向的半径
	int ry = 22;  //椭圆y方向的半径
	u8g2_ClearBuffer(u8g2);
	u8g2_DrawStr(u8g2,0, 14, "DrawFilledEllipse");
 
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	u8g2_DrawFilledEllipse(u8g2, 0, with, rx, ry, U8G2_DRAW_LOWER_RIGHT);//右下
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	u8g2_DrawFilledEllipse(u8g2, with * 4 - 1, with, rx, ry, U8G2_DRAW_LOWER_LEFT); //左下
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	u8g2_DrawFilledEllipse(u8g2, 0, with * 4 - 1, rx, ry, U8G2_DRAW_UPPER_RIGHT); //右上
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	u8g2_DrawFilledEllipse(u8g2, with * 4 - 1, with * 4 - 1, rx, ry, U8G2_DRAW_UPPER_LEFT); //左上
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
	u8g2_DrawFilledEllipse(u8g2, with * 6, with * 2.5, rx, ry, U8G2_DRAW_ALL);//整个椭圆
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
}
 
//环形测试
void testDrawMulti(u8g2_t *u8g2)
{
	u8g2_ClearBuffer(u8g2);
	for (int j = 0; j < 64; j+=16)
	{
		for (int i = 0; i < 128; i+=16)
		{
			u8g2_DrawPixel(u8g2, i, j);
			u8g2_SendBuffer(u8g2);
		}
	}
  
	//实心矩形逐渐变大
    u8g2_ClearBuffer(u8g2);
	for(int i=30; i>0; i-=2)
	{
		u8g2_DrawBox(u8g2,i*2,i,128-i*4,64-2*i);
		u8g2_SendBuffer(u8g2);
	}
	//空心矩形逐渐变小
	u8g2_ClearBuffer(u8g2);
	for(int i=0; i<32; i+=2)
	{
		u8g2_DrawFrame(u8g2,i*2,i,128-i*4,64-2*i);
		u8g2_SendBuffer(u8g2);
	}
	
	//实心圆角矩形逐渐变大
	u8g2_ClearBuffer(u8g2);
	for(int i=30; i>0; i-=2)
	{
		u8g2_DrawRBox(u8g2,i*2,i,128-i*4,64-2*i,10-i/3);
		u8g2_SendBuffer(u8g2);
	}
    //空心圆角矩形逐渐变小
	u8g2_ClearBuffer(u8g2);
	for(int i=0; i<32; i+=2)
	{
		u8g2_DrawRFrame(u8g2,i*2,i,128-i*4,64-2*i,10-i/3);
		u8g2_SendBuffer(u8g2);
	}
	
	//实心圆逐渐变大
	u8g2_ClearBuffer(u8g2);
	for(int i=2; i<64; i+=3)
	{
		u8g2_DrawDisc(u8g2,64,32,i, U8G2_DRAW_ALL);
		u8g2_SendBuffer(u8g2);
	}
	//空心圆逐渐变小
	u8g2_ClearBuffer(u8g2);
	for(int i=64; i>0; i-=3)
	{
		u8g2_DrawCircle(u8g2,64,32,i, U8G2_DRAW_ALL);
		u8g2_SendBuffer(u8g2);
	}
	
	//实心椭圆逐渐变大
    u8g2_ClearBuffer(u8g2);
	for(int i=2; i<32; i+=3)
	{
		u8g2_DrawFilledEllipse(u8g2,64,32, i*2, i, U8G2_DRAW_ALL);
		u8g2_SendBuffer(u8g2);
	}
    //空心椭圆逐渐变小
    u8g2_ClearBuffer(u8g2);
	for(int i=32; i>0; i-=3)
	{
		u8g2_DrawEllipse(u8g2,64,32, i*2, i, U8G2_DRAW_ALL);
		u8g2_SendBuffer(u8g2);
	}
}
 
 
// width: 128, height: 48
const unsigned char bilibili[] U8X8_PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0xe0, 0x03, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0xf0, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0x01, 0xfc, 0x00, 0x00, 0x00, 0xc0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x03, 0xfc, 0x00, 0x00, 0x3c, 0xc0, 0x0f, 0x00, 0x80, 0x03, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0x07, 0xfc, 0x00, 0x00, 0x3c, 0xc0, 0x0f, 0x00, 0xc0, 0x07, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xfc, 0x00, 0x00, 0x3c, 0x80, 0x0f, 0x00, 0xc0, 0x07, 0x00, 0xc0, 0x0f, 0x00, 0x00, 0x80, 0x0f, 0xf8, 0x00, 0x00, 0x3c, 0x80, 0x0f, 0x00, 0x80, 0x07, 0x00, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x78, 0x80, 0x0f, 0x00, 0x80, 0x07, 0x00, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x78, 0x80, 0x0f, 0x00, 0x80, 0x07, 0x00, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x80, 0x79, 0x80, 0x0f, 0x00, 0x98, 0x07, 0x00, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0xe0, 0x79, 0x9f, 0x0f, 0x00, 0xbe, 0xe7, 0x01, 0xc0, 0x07, 0x10, 0x40, 0x00, 0x1f, 0xf8, 0x00, 0xe0, 0x7b, 0x1f, 0x0f, 0x00, 0xbe, 0xe7, 0x01, 0xc0, 0x87, 0x1f, 0xe0, 0x0f, 0x1f, 0xf8, 0x00, 0xe0, 0x7b, 0x1e, 0x0f, 0x00, 0x3e, 0xe7, 0x01, 0xc0, 0xe7, 0x3f, 0xe0, 0x3f, 0x1f, 0xf0, 0x00, 0xe0, 0x7b, 0x1e, 0x0f, 0x00, 0x3e, 0xe7, 0x01, 0xc0, 0xe7, 0x3f, 0xe0, 0x3f, 0x1f, 0xf0, 0x00, 0x60, 0x71, 0x1e, 0x0f, 0x00, 0x34, 0xe7, 0x01, 0xc0, 0xe7, 0x07, 0x00, 0x3f, 0x1f, 0xf0, 0x00, 0x00, 0x70, 0x00, 0x1f, 0x00, 0x00, 0x07, 0x00, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0xc0, 0x73, 0x1e, 0x1f, 0x00, 0x3c, 0xc7, 0x01, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0xc0, 0x73, 0x1e, 0x1f, 0x00, 0x7c, 0xe7, 0x01, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0xc0, 0x73, 0x1e, 0x1f, 0x00, 0x7c, 0xef, 0x01, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x01, 0xc0, 0x77, 0x1e, 0x1e, 0x00, 0x7c, 0xef, 0x01, 0xc0, 0x07, 0x00, 0x03, 0x00, 0x1f, 0xf0, 0xff, 0xc1, 0xf7, 0x1e, 0xfe, 0x1f, 0x78, 0xef, 0x01, 0xc0, 0x07, 0x70, 0x37, 0x00, 0x1f, 0xe0, 0xff, 0x87, 0xf7, 0x1e, 0xfe, 0xff, 0x78, 0xee, 0x01, 0xc0, 0x07, 0xe0, 0x3f, 0x00, 0x1f, 0xe0, 0xff, 0x9f, 0xf7, 0x1e, 0xfe, 0xff, 0x79, 0xce, 0x01, 0xc0, 0x07, 0xc0, 0x18, 0x00, 0x1f, 0xe0, 0xff, 0xbf, 0xe7, 0x1e, 0xfe, 0xff, 0x7b, 0xce, 0x01, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0xc7, 0xbf, 0xe7, 0x1e, 0xfe, 0xf8, 0x77, 0xce, 0x01, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x0f, 0x3f, 0xe7, 0x1c, 0xfe, 0xf0, 0x77, 0xce, 0x03, 0xc0, 0x07, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0xcf, 0x3f, 0xe7, 0x1c, 0xfe, 0xf8, 0xf3, 0xce, 0x03, 0xc0, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xe0, 0xef, 0x1f, 0xe7, 0x1c, 0xfe, 0xfe, 0xf1, 0xce, 0x03, 0x80, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xe0, 0xff, 0x0f, 0xcf, 0x1c, 0xfc, 0xff, 0xf0, 0xc0, 0x03, 0x00, 0xff, 0xff, 0xff, 0xff, 0x07, 0xe0, 0xff, 0x03, 0x06, 0x1c, 0xfc, 0x7f, 0x60, 0xc0, 0x01, 0x00, 0xfe, 0xff, 0xff, 0xff, 0x03, 0xe0, 0xff, 0x00, 0x00, 0x00, 0xfc, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0x00, 0xc0, 0x0f, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// width: 128, height: 48
const unsigned char three_support[] U8X8_PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 0xfc, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 0xfe, 0x7f, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x00, 0x00, 0x80, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0xc0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x03, 0x00, 0x00, 0x80, 0x0f, 0xf0, 0x01, 0x00, 0x00, 0xfc, 0xff, 0x01, 0x00, 0x00, 0xc0, 0xfd, 0xff, 0x00, 0x00, 0xc0, 0x7f, 0xfe, 0x01, 0x00, 0x00, 0xff, 0xff, 0x0f, 0x00, 0x00, 0xe0, 0xfd, 0xff, 0x01, 0x00, 0xc0, 0x1f, 0xf8, 0x03, 0x00, 0x00, 0xff, 0xff, 0x0f, 0x00, 0x00, 0xe0, 0xfd, 0xff, 0x01, 0x00, 0xc0, 0x0f, 0xf0, 0x03, 0x00, 0x00, 0xfe, 0xff, 0x07, 0x00, 0x00, 0xe0, 0xfd, 0xff, 0x01, 0x00, 0xc0, 0x67, 0xe6, 0x03, 0x00, 0x00, 0xfc, 0xff, 0x03, 0x00, 0x00, 0xe0, 0xfd, 0xff, 0x01, 0x00, 0xc0, 0x67, 0xe6, 0x03, 0x00, 0x00, 0xf8, 0xff, 0x01, 0x00, 0x00, 0xe0, 0xfd, 0xff, 0x00, 0x00, 0xc0, 0x67, 0xe6, 0x03, 0x00, 0x00, 0xf0, 0x7f, 0x00, 0x00, 0x00, 0xe0, 0xfd, 0xff, 0x00, 0x00, 0xc0, 0x67, 0xee, 0x03, 0x00, 0x00, 0xe0, 0x7f, 0x00, 0x00, 0x00, 0xe0, 0xfd, 0xff, 0x00, 0x00, 0x80, 0x7f, 0xfe, 0x01, 0x00, 0x00, 0xe0, 0xff, 0x00, 0x00, 0x00, 0xe0, 0xfd, 0xff, 0x00, 0x00, 0x80, 0x7f, 0xfe, 0x01, 0x00, 0x00, 0xf0, 0xff, 0x00, 0x00, 0x00, 0xe0, 0xfd, 0x7f, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0xf0, 0xff, 0x00, 0x00, 0x00, 0xe0, 0xfd, 0x7f, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x00, 0x00, 0x00, 0xf8, 0xf9, 0x01, 0x00, 0x00, 0xe0, 0xfd, 0x7f, 0x00, 0x00, 0x00, 0xfe, 0x7f, 0x00, 0x00, 0x00, 0xf8, 0xf0, 0x00, 0x00, 0x00, 0xe0, 0xfd, 0x1f, 0x00, 0x00, 0x00, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0x30, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
 
void testDrawXBM(u8g2_t *u8g2)
{
	int t = 1000;
	u8g2_ClearBuffer(u8g2);
	u8g2_DrawStr(u8g2,0, 14, "DrawXBM");
 
	u8g2_DrawXBM(u8g2,0, 16, 128, 48, bilibili);
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
 
	u8g2_ClearBuffer(u8g2);
	u8g2_DrawStr(u8g2,0, 14, "bilibili");
	u8g2_DrawXBM(u8g2,0, 16, 128, 48, three_support);
	SEND_BUFFER_DISPLAY_MS(u8g2,t);
}
 
void u8g2DrawTest(u8g2_t *u8g2)
{
	testDrawProcess(u8g2);
	testDrawMulti(u8g2);
	//testDrawFrame(u8g2);
	//testDrawRBox(u8g2);
	//testDrawCircle(u8g2);
	//testDrawFilledEllipse(u8g2);
	testShowFont(u8g2);
	testDrawXBM(u8g2);
 
}
```

#### main.c

```
int main(void)
{
  /* USER CODE BEGIN 2 */
  u8g2_t u8g2;
  u8g2Init(&u8g2);	
  while (1)
  {
       u8g2_FirstPage(&u8g2);
       do
       {
				 draw(&u8g2);
				 u8g2DrawTest(&u8g2);
       } while (u8g2_NextPage(&u8g2));
  }
}
```

### SSD1315驱动

#### main.c

```
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(200);
	OLED_Init();
	OLED_Test();	
  while (1)
  {
	  static char c = '0';
	  c++; 
	  OLED_NewFrame();
//		  OLED_DrawCircle(64, 32, i, OLED_COLOR_REVERSED);
//		  OLED_DrawCircle(64, 32, 2*i, OLED_COLOR_REVERSED);
//		  OLED_DrawCircle(64, 32, 3*i, OLED_COLOR_REVERSED);
//		  OLED_SetPixel(2*i, i, OLED_COLOR_NORMAL);
//		  OLED_DrawCircle(64,32, i,OLED_COLOR_NORMAL);
//		  OLED_DrawCircle(64,32, 2*i, OLED_COLOR_NORMAL);		  
//		  OLED_DrawCircle(64,32, 3*i,OLED_COLOR_NORMAL);	
//	  OLED_PrintASCIIString(64,24,&c , &afont8x6, OLED_COLOR_REVERSED);		
	  OLED_ShowFrame();
	  HAL_Delay(500);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
```





# 电机驱动

高精度的转动

- 高精度编码器 + 控制算法

  - BLDC--无刷直流电机--霍尔编码器(方波驱动,低精度即可)--低分辨率

    **pwm控制**(脉宽调制)--电流大小调速

  - PMSM--永磁同步电机--旋转变压器或光码盘(正弦波精度要求高)--高分辨率--

    **foc矢量控制**(磁场定向控制)--控电流波形--减小转矩波动

  - 高精度编码器（正交编码器）获取位置

  - 启动时初始定位

- 先进反馈机制

  - 闭环控制：读取编码器，传感器，电机位置闭环控制
  - 软件算法：**卡尔曼滤波**，优化控制精度响应速度

- 步进电机--脉冲控制步进角(1.8)--pwm调制减少起步停止是的丢步震荡，增加行走时电流的稳定性

- 有刷直流电机--仅pwm

- 伺服电机--高精度 含编码器 可准确控制 与 特殊设计算法优化 后的电机。

**PID**控制

- 读取编码器脉冲 数字信号，得出距离，速度，加速度-->闭环控制

**三伺服**

- 位置环 控角  变速度 调距离  编码器反馈--位置环的设定值(距离->脉冲)+反馈值->(偏差计数)位置环的**P**调节->速度环给定值
- 速度环 控速  变电流 调速度  编码器反馈--位置环的输出
- 电流环  控流 变电流 调扭矩  电流传感器反馈--速度环的输出（即期望转速对应的电流值|参考

**pwm频率** 周期 影响电机稳定性，不宜调节

**级联控制** 速度控制器(PI或PID控制器)的输出通常作为电流环的参考值

- **设置目标位置**：首先，需要知道移动固定距离对应的脉冲数量。这可以通过实验或计算得到。
- **初始化霍尔编码器**：在系统启动时，需要对霍尔编码器进行初始化，以确定初始位置。这通常通过寻找一个参考点（如机械原点）来实现。
- **读取脉冲数**：当电机开始运动时，控制器不断读取霍尔编码器产生的脉冲数。每个脉冲代表电机转动一定的角度。
- **计算当前位置**：通过对脉冲进行计数，可以计算出电机的当前位置。这通常是相对于初始位置的相对位移。
- **比较目标位置**：将当前位置与目标位置进行比较。如果当前位置小于目标位置，则继续驱动电机；如果当前位置等于或大于目标位置，则停止电机。
- **控制电机启停**：根据比较结果，控制器发出指令控制电机启动、停止或反转。这确保了电机能够精确地移动到目标位置
- **没有了电流反馈，电流环不再进行积分和微分运算，只能进行比例运算调节电流**：控制精度和动态性能可能会受到影响，无法实现精确的电流控制

**位置环性能**：

- **位置传感器**
- **PID调节器**
- **机械传动部件**

### 电机测速

- **提高不同响应速度**
  - 到指定位置变速
    - 位置环PD--P定->D变
    - 速度环--targey变 + 位置环检测（编码器)PID置0
  - 匀速
    - 速度环PID--target变

- **HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);**
- **HAL_TIM_Base_Start_IT(&htim2);**
- **HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);**
- **__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,(controler_data[0]+100));**
- **void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)**--中断回调

#### main.c

```
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
unsigned int MotorSpeed;  // 电机当前速度数值，从编码器中获取 rpm
int motor_rate = xx;  //(0-100)
int MotorOutput = 72*100;		  // 电机输出 0-7200
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	    // TIM1_CH1(pwm)
HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1); // 开启编码器A
HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2); // 开启编码器B	
HAL_TIM_Base_Start_IT(&htim2);                // 使能定时器2中断
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static unsigned char i = 0;
    if (htim == (&htim2))
    {
        //1.获取电机速度
        MotorSpeed = (short)(__HAL_TIM_GET_COUNTER(&htim4)/18);   
        // TIM4计数器获得电机脉冲，该电机在10ms采样的脉冲/18则为实际转速的rpm
        __HAL_TIM_SET_COUNTER(&htim4,0);  // 计数器清零
        
      
        //2.将占空比导入至电机控制函数
        MotorOutput=3600; // 3600即为50%的占空比
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorOutput);
        MotorOutput=MotorOutput*100/7200; 
        // 占空比（按最高100%计算，串口打印）
        i++;
        if(i>100)
        {
          // 打印定时器4的计数值，short（-32768——32767）
          printf("Encoder = %d moto = %d \r\n",MotorSpeed,MotorOutput);	
          i=0;
        }
    }
}
/* USER CODE END 4 */

int main(){
	HAL_TIM_PWM_Start(&htm1,TIM_CHANNEL_1);
}
```

### 电机驱动

#### Motor.h

```
#ifndef __MOTOR_H
#define __MOTOR_H
#include <stdint.h>
//A4,A5连接AIN1,AIN2,控制AO1,AO2，控制右边两电机;A6,A7连接BIN1,BIN2,控制BO1,BO2，控制左边两电机。

void Motor_SetLeftSpeed(int8_t Speed);
void Motor_SetRightSpeed(int8_t Speed);
#endif
```

#### Motor.c

```
#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void Motor_SetLeftSpeed(int8_t Speed)//右边电机的速度，为什么会反过来连接，因为在主视图上，BIN在左边，AIN在右边
{
	if (Speed >0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		PWM_SetCompare3(Speed);
	}
	else if(Speed==0){
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		GPIO_SetBits(GPIOA, GPIO_Pin_5);//达到一个制动的效果
		PWM_SetCompare3(Speed);         //占空比为0
	}else{   							//速度<0小车往后走，电机反转
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
		PWM_SetCompare3(-Speed);
	}
}
void Motor_SetRightSpeed(int8_t Speed)//左边电机的速度
{
	if (Speed >0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_6);
		GPIO_ResetBits(GPIOA, GPIO_Pin_7);
		PWM_SetCompare3(Speed);
	}
	else if(Speed==0){
		GPIO_SetBits(GPIOA, GPIO_Pin_6);
		GPIO_SetBits(GPIOA, GPIO_Pin_7);
		PWM_SetCompare3(Speed);
	}else{
		GPIO_ResetBits(GPIOA, GPIO_Pin_6);
		GPIO_SetBits(GPIOA, GPIO_Pin_7);
		PWM_SetCompare3(-Speed);
	}
}
```

### foc开环控制

#### open_foc.h

```
#ifndef __OPEN_FOC_H_
#define __OPEN_FOC_H_
 
#include "stm32f1xx_hal.h"
#include "math.h"
 
void ClarkAndPark(float Uq,float Ud,float angle_el);
float electricalAngle(float shaft_angle, int pole_pairs) ;
float normalizeAngle(float angle);
void test(void);
 
#endif
```

#### open_foc.c

```
#include "open_foc.h"
#define voltage_power_supply 12.6
#define Pi 3.1415926
 
float Ualpha,Ubeta,Ua,Ub,Uc;
float APWM,BPWM,CPWM;
/**********************系统运行时间***************************/
__STATIC_INLINE uint32_t GXT_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}
static uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  GXT_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (GXT_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}
//获取系统时间，单位us
uint32_t micros(void)
{
  return getCurrentMicros();
}
/**************************************************************/
/* Clark与Park变换*/
void ClarkAndPark(float Uq,float Ud,float angle_el)
{
	Ualpha = -Uq*sin(angle_el);
	Ubeta = Uq*cos(angle_el);
	
	Ua = Ualpha + voltage_power_supply / 2;
	Ub = (sqrt(3)*Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
	Uc = -(Ualpha + sqrt(3)*Ubeta) / 2 + voltage_power_supply / 2;
	APWM=Ua/voltage_power_supply;
	BPWM=Ub/voltage_power_supply;
	CPWM=Uc/voltage_power_supply;
	printf("%lf,%lf,%lf\n",APWM,BPWM,CPWM);
}
float electricalAngle(float shaft_angle, int pole_pairs) 
{
  return (shaft_angle * pole_pairs);
}
// 归一化角度到 [0,2PI]
float normalizeAngle(float angle){
  float a = fmod(angle, 2*Pi);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*Pi);  
}
uint32_t now=0,last=0;
float E_A,m_a=0;
double Tval;
void test(void)
{
	now=micros();
	Tval=(double)(now-last)/1000000;
	m_a=normalizeAngle(m_a+(10*Pi)*Tval*7);
//	printf("%lf\n",m_a);
	E_A= electricalAngle(m_a, 7);
	ClarkAndPark(5,0,m_a);
	last=now;
}
```

#### main.c

```
#include "open_foc.h"
extern float APWM,BPWM,CPWM;
int main(void)
{
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
  /* USER CODE END 2 *
  while (1)
  {
		test();
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,4799*APWM); 
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,4799*BPWM); 
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,4799*CPWM); 
  }
}

```



# PID

## VOFA+

### **FireWater**：

- 适合调试阶段或者低速少量数据传输。
- 支持绘制多通道波形
- ASCII字符串，会占用更多带宽
- printf，但解析消耗资源较大
- CSV风格的字符串流，数据以逗号分隔，需加换行符

```
printf("%.2f,%.2f\n",chanel1,chanel2);//这里打印了多少个就是多少个通道，上位机会识别
```



### **JustFloat**：

- 优选于高速大量数据传输的环境。
- 支持多通道数据，适合动态显示波形
- 一定的编程知识来处理浮点数到字节的转换
- 高效传输，使用小端浮点数和帧尾校验，节省带宽
- 小端浮点数组形式的字节流，小端浮点数组形式的字节流，

### **RawData**：

- 适用于仅需简单串口功能的情况。



- 参数整定找最佳，从小到大顺序查
- 先是比例后积分，最后再把微分加
- 曲线振荡很频繁，比例度盘要放大
- 曲线漂浮绕大湾，比例度盘往小扳
- 曲线偏离回复慢，积分时间往下降
- 曲线波动周期长，积分时间再加长
- 曲线振荡频率快，先把微分降下来
- 动差大来波动慢，微分时间应加长
- 理想曲线两个波，前高后低四比一
- 一看二调多分析，调节质量不会低

**局限**--非线性和复杂系统的控制效果不佳、参数调整困难、对系统参数变化和外部扰动的鲁棒性差

## **优化算法**

#### 结构体

```
typedef struct {
    float Kp; // 比例系数
    float Ki; // 积分系数
    float Kd; // 微分系数
    float max_out; // 输出上限
    float min_out; // 输出下限
    float max_iout; // 积分项输出的最大值 -->误差持续存在(积分饱和)，达饱和，需反调，需积分退饱和，才响应，调控滞后，严重超调|震荡
    float set; // 目标值
    float fdb; // 当前值或反馈值
    float out; // PID控制器的最终输出值
    float Pout; // 比例环节的输出值
    float Iout; // 积分环节的输出值
    float Dout; // 微分环节的输出值
    float error[3]; // 存储过去几次的误差值
    float Dbuf[2]; // 存储过去几次微分项的计算结果
} PIDController;

```

#### **初始化**

```
void initPIDController(PIDController *pid, float Kp, float Ki, float Kd, float max_out, float min_out, float max_iout) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->max_out = max_out;
    pid->min_out = min_out;
    pid->max_iout = max_iout;
    pid->set = 0.0;
    pid->fdb = 0.0;
    pid->out = 0.0;
    pid->Pout = 0.0;
    pid->Iout = 0.0;
    pid->Dout = 0.0;
    for (int i = 0; i < 3; i++) {
        pid->error[i] = 0.0;
    }
    for (int i = 0; i < 2; i++) {
        pid->Dbuf[i] = 0.0;
    }
}
```

注：

- 位置式需引入初始阀位值，增量式不需要

- 位置式需防积分饱和，增量式不会产生积分饱和
- 只有存在偏差时，增量式才会有输出
- 增量式容易实现从手动到自动的切换
- 位置式PID控制的输出与整个过去的状态有关，用到了误差的累加值;而增量式PID的输出只与当前拍和前两拍的误差有关，因此位置式PID控制的累积误差相对更大;



### 增量PID(少截断 误差影响小|静差，时长，超调)

**控制量绝对值**

```
/*
* 增量式PID计算C语言代码
* 输入参数:
* 返回值：
*        PID计算增量值        
*/
float PID_Update_Increase(int CurrentPoint,int SetPoint,PID* sptr) 
{
  float	iIncpid;
  sptr->iError=SetPoint-CurrentPoint;                                     // 计算当前误差
  iIncpid=sptr->Proportion * (sptr->iError-sptr->Error1)                  // P
         +sptr->Integral * sptr->iError                                   // I
         +sptr->Derivative * (sptr->iError-2*sptr->Error1+sptr->Error2);  // D
             
  sptr->Error2=sptr->Error1;                          // 存储误差，用于下次计算
  sptr->Error1=sptr->iError;										
		
  return(iIncpid);                                    // 返回增量值
}
```

### 位置PID(静差小，溢出少|计算量大 积分易饱和 调参费时间 误差响应不灵敏 非线性效果差 -->无积分部件，位置直达)

**控制量增量**

```c
// 更新PID控制器（绝对式PID）
double PID_Update_Absolute(PIDController *pid, double current_value, double dt) {
    double error = pid->setpoint - current_value; // 计算误差
    pid->integral += error * dt; // 计算积分项
    double derivative = (error - pid->last_error) / dt; // 计算微分项
    double output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative; // 计算输出值
    pid->last_error = error; // 更新上一次的误差
    return output;
}
```



### 微分先行（PD--舵机快速响应）

```
// 积分先行PID控制器函数
double integral_lead_PID(double setpoint, double process_value) {
    double error = setpoint - process_value; // 计算误差
    integral += error; // 更新积分项
    double derivative = error - previous_error; // 计算微分项
    previous_error = error; // 更新上一次误差
    return Kp * error + Ki * integral + Kd * derivative; // 返回控制输出
}
```

### PI控制（消除静差）

### PD控制

```c
// 更新PID控制器（仅含PD控制器）
double PID_Update_PD(PIDController *pid, double current_value, double dt) {
    double error = pid->setpoint - current_value; // 计算误差
    double derivative = (error - pid->last_error) / dt; // 计算微分项
    double output = pid->Kp * error + pid->Kd * derivative; // 计算输出值
    pid->last_error = error; // 更新上一次的误差
    return output;
}

```

### 积分分离（启动+结束+大幅福调整设定值）

```
// 更新PID控制器（积分分离）
double PID_Update_IntegralSeparation(PIDController *pid, double current_value, double dt, int condition) {
    double error = pid->setpoint - current_value; // 计算误差
    if (condition == 1) {
        pid->integral += error * dt; // 条件1：正常积分
    } else if (condition == 2) {
        pid->integral = 0; // 条件2：积分清零
    } else {
        pid->integral += error * dt / 2; // 条件3：积分减半
    }
    double derivative = (error - pid->last_error) / dt; // 计算微分项
    double output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative; // 计算输出值
    pid->last_error = error; // 更新上一次的误差
    return output;
}
```

### 积分限幅（偏差越大，积分越慢; 偏差越小，积分越快）

#### **饱和处理,停止积分**

-->防止不稳定|超调

```
void saturate(float *integral, float max_integral) {
    if (*integral > max_integral) {
        *integral = max_integral; // 将积分项设置为最大值
    } else if (*integral < -max_integral) {
        *integral = -max_integral; // 将积分项设置为负的最大值
    }
}
```

#### **重置积分器**

-->饱和后需从零计算

```
void resetIntegral(foat *integral, float max_integral) {
    float error = target_value - current_value;
    if (*integral > max_integral || *integral < -max_integral) {
    	*integral = 0.0; // 重置积分项为零
    }
}
```

#### 积分饱和保护

-->避免积分饱和并保持系统的稳定

```
//gain--缩放误差对积分的贡献 
//过大-->过冲
//过小-->响应缓慢
void integralSaturationProtection(float *integral, float error, float max_integral, float gain) {
    // 判断误差是否为正，若为正则进行积分累积
    if (error > 0) {
        *integral += error * gain;
    } else {
        *integral -= error * gain;
    }
    // 检查积分项是否超过最大值
    if (*integral > max_integral) {
        *integral = max_integral; // 将积分项设置为最大值
    } else if (*integral < -max_integral) {
        *integral = -max_integral; // 将积分项设置为负的最大值
    }
}
```

#### 积分饱和修正

达饱和后，动态调参，快速响应！！！--可能破环稳定性

```
void adjustControllerParameters(float *Kp, float *Ki, float *Kd, float target_value, float current_value，float *integral, float max_integral) {
    // 根据当前值与目标值的差异调整PID参数
    float error = target_value - current_value;
    if (*integral > max_integral || *integral < -max_integral) {
    *Kp += error * 0.1; // 假设比例系数调整因子为0.1
    *Ki += error * 0.05; // 假设积分系数调整因子为0.05
    *Kd += error * 0.01; // 假设微分系数调整因子为0.01
    }
}
```



### 模糊PID(微分泛化)

模糊逻辑 + PID控制 -->模糊逻辑处理不确定性和非线性，根据系统状态调整PID参数-->适应+鲁棒

```
void fuzzyPIDUpdate(FuzzyPIDController *pid, float current_value) {
    // 计算误差
    float error = pid->setpoint - current_value;
    
    // 更新误差数组
    for (int i = 2; i > 0; i--) {
        pid->error[i] = pid->error[i - 1];
    }
    pid->error[0] = error;
    
    // 计算比例项
    pid->Pout = pid->Kp * error;
    
    // 计算积分项
    pid->Iout += pid->Ki * error;
    if (pid->Iout > pid->max_out) {
        pid->Iout = pid->max_out;
    } else if (pid->Iout < pid->min_out) {
        pid->Iout = pid->min_out;
    }
    
    // 计算微分项
    float derivative = (pid->error[0] - pid->error[1]) / (pid->error[1] - pid->error[2]);
    pid->Dout = pid->Kd * derivative;
    
    // 更新输出值
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    if (pid->out > pid->max_out) {
        pid->out = pid->max_out;
    } else if (pid->out < pid->min_out) {
        pid->out = pid->min_out;
    }
    
    // 更新反馈值
    pid->fdb = current_value;
}
```



### 神经网络PID

**数据采集**：

1. **理解系统动态**：首先，对您要控制的系统进行深入理解。了解系统的动态行为，包括其对控制输入的响应时间、稳定性和可能的非线性特性。
2. **初始PID参数设置**：基于系统的理论知识或经验规则（如Ziegler-Nichols方法），设置初始的PID参数。
3. **系统辨识**：可以通过系统辨识的方法来估计系统的数学模型。这些模型可以帮助您理解系统的行为，并预测不同PID参数下的系统响应。
4. **实验和调优**：
   - **手动调优**：在安全的条件下，逐步调整PID参数，观察系统的响应。记录下每次调整后的系统性能指标，如超调量、稳态误差、上升时间和调节时间。
   - **自动调优**：使用自动调谐方法（如软件工具或自整定调节器）来找到最佳的PID参数。这些方法通常会通过优化某个性能指标（如ITAE、ISE等）来自动调整参数。
5. **数据收集**：在调优过程中，收集系统的输入和输出数据。确保在每个PID参数设置下，系统都运行足够长的时间，以观察到稳定的响应。
6. **标记数据**：记录每次实验的PID参数值，以及系统的响应。这些数据应标记为“最佳”或“非最佳”，取决于系统的响应是否符合预期的性能标准。
7. **数据分析**：分析收集到的数据，比较不同PID参数下的系统性能。最佳PID参数下的数据应该显示出良好的系统响应，如快速且无超调地达到设定点，以及稳定的稳态行为。
8. **训练数据准备**：从收集的数据中选择最佳PID参数下的数据作为训练数据，用于训练神经网络PID控制器。确保训练数据覆盖了不同的操作条件和扰动情况。
9. **验证和测试**：使用独立的验证数据集来测试神经网络的性能。如果可能，使用实际系统中的数据进行测试，以确保模型的泛化能力。

**交叉验证**：

```
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import KFold

class NeuralNetworkPID(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(NeuralNetworkPID, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, output_size)
        self.relu = nn.ReLU()

    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.fc3(x)
        return x

# 超参数设置
input_size = 3  # 输入维度，例如：误差、误差积分、误差微分
hidden_size = 64  # 隐藏层神经元数量
output_size = 3  # 输出维度，例如：PID系数Kp、Ki、Kd
learning_rate = 0.001
epochs = 1000
k_folds = 5  # 设置交叉验证的折数

# 创建模型实例
model = NeuralNetworkPID(input_size, hidden_size, output_size)
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=learning_rate)

# 训练数据准备（这里使用随机生成的数据作为示例）
train_data = torch.randn((1000, input_size))
train_labels = torch.randn((1000, output_size))

# 使用K折交叉验证进行模型优化
kf = KFold(n_splits=k_folds)
best_loss = float('inf')
best_model = None

for fold, (train_index, val_index) in enumerate(kf.split(train_data)):
    print(f"Training fold {fold + 1}")
    train_subset = train_data[train_index]
    train_labels_subset = train_labels[train_index]
    val_subset = train_data[val_index]
    val_labels_subset = train_labels[val_index]

    # 重置模型权重
    model.apply(lambda m: m.reset_parameters())

    # 训练模型
    for epoch in range(epochs):
        optimizer.zero_grad()
        outputs = model(train_subset)
        loss = criterion(outputs, train_labels_subset)
        loss.backward()
        optimizer.step()
        if (epoch+1) % 100 == 0:
            print(f'Epoch [{epoch+1}/{epochs}], Loss: {loss.item():.4f}')

    # 验证模型
    with torch.no_grad():
        outputs = model(val_subset)
        val_loss = criterion(outputs, val_labels_subset)
        print(f'Validation Loss: {val_loss.item():.4f}')

    # 更新最佳模型
    if val_loss < best_loss:
        best_loss = val_loss
        best_model = model

print("Best model found with validation loss:", best_loss)

```



### 自适应PID控制(参数自修正)

```
// 更新自适应PID控制器
double update_pid(PIDController *pid, double current_value) {
    double error = pid->setpoint - current_value; // 计算误差
    pid->integral += error; // 更新积分项
    double derivative = error - pid->last_error; // 计算微分项
    pid->last_error = error; // 更新上一次的误差

    // 计算控制输出
    double output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return output;
}
// 自适应调整参数的模块
void adapt_parameters(PIDController *pid, double current_value, double control_output) {
    // 根据系统响应和控制输出来调整PID参数
    // 这里只是一个示例，实际应用中需要根据具体情况进行调整
    if (control_output > 100) {
        pid->kp *= 0.9;
        pid->ki *= 0.9;
        pid->kd *= 0.9;
    } else if (control_output < -100) {
        pid->kp *= 1.1;
        pid->ki *= 1.1;
        pid->kd *= 1.1;
    }
}

int main() {
    PIDController pid;
    double kp = 1.0, ki = 0.5, kd = 0.1, setpoint = 10.0;
    double current_value = 0.0;
    for (int i = 0; i < 100; i++) {
        double control_output = update_pid(&pid, current_value);
        printf("Control output: %f
", control_output);
        // 在这里，您可以将控制输出应用于您的系统，并获取新的当前值
        // current_value = ...;
        // 自适应调整参数
        adapt_parameters(&pid, current_value, control_output);
    }
    return 0;
}

```



### 遗传算法优化PID(自适应的PID控制器，通过遗传算法来优化PID参数)

**遗传算法**(优化搜索算法)--在给定的搜索空间中寻找最优解

  -->**遗传算法得出的PID参数确实是相对于各种PID参数情况下最符合系统的参数，而不是仅仅针对特定输入数据的最优参数**。

PID--调整系统的输出以达到期望的目标值

**种群大小、代数、变异率**等，这些参数需要根据具体问题进行调整

**参数准备**：

1. 目标值（Target）：这是你希望系统达到的目标状态或输出。例如，如果你正在控制一个温度控制系统，目标值可能是你希望达到的特定温度。
2. 输入数据序列（Input）：这是一系列实际测量到的系统状态或输出数据。这些数据将用于计算适应度函数和评估PID控制器的性能。例如，如果你正在控制一个温度控制系统，输入数据可能是一系列实际测量到的温度值。
3. 种群大小（Population size）：这是遗传算法中初始种群的大小。种群中的每个个体都表示一个可能的PID参数组合。较大的种群可以提高算法的多样性和搜索能力，但也会增加计算成本。
4. 迭代次数（Generations）：这是遗传算法运行的总代数。每一代都会进行选择、交叉和变异操作来生成新的种群。较多的代数可以增加算法找到最优解的机会，但也可能导致更长的计算时间。
5. 变异率（Mutation rate）：这是遗传算法中变异操作发生的概率。较高的变异率可以增加种群的多样性，有助于探索新的解决方案空间，但也可能导致算法不稳定。较低的变异率则可能导致算法陷入局部最优解。

注：多次运行遗传算法并比较不同结果，以获得更好的性能。

```
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// 定义PID结构体
typedef struct {
    double kp;
    double ki;
    double kd;
} PID;

// 计算误差平方和（适应度函数）
//
double fitness_function(PID pid, double target, double *input, int length) {
    double error = 0.0;
    double integral = 0.0;
    double last_error = 0.0;
    double output = 0.0;
    int i;

    for (i = 0; i < length; i++) {
        error = target - input[i];
        integral += error;
        double derivative = error - last_error;
        output = pid.kp * error + pid.ki * integral + pid.kd * derivative;
        last_error = error;
    }

    return output * output; // 返回误差平方和作为适应度函数
}

// 遗传算法中的交叉操作
void crossover(PID *parent1, PID *parent2, PID *child1, PID *child2) {
    child1->kp = (parent1->kp + parent2->kp) / 2;
    child1->ki = (parent1->ki + parent2->ki) / 2;
    child1->kd = (parent1->kd + parent2->kd) / 2;

    child2->kp = (parent1->kp + parent2->kp) / 2;
    child2->ki = (parent1->ki + parent2->ki) / 2;
    child2->kd = (parent1->kd + parent2->kd) / 2;
}

// 遗传算法中的变异操作
void mutation(PID *individual, double mutation_rate) {
    if (rand() < RAND_MAX * mutation_rate) {
        individual->kp += (rand() % 100 - 50) / 100.0;
    }
    if (rand() < RAND_MAX * mutation_rate) {
        individual->ki += (rand() % 100 - 50) / 100.0;
    }
    if (rand() < RAND_MAX * mutation_rate) {
        individual->kd += (rand() % 100 - 50) / 100.0;
    }
}

// 遗传算法主函数
void genetic_algorithm(PID *population, int population_size, double target, double *input, int length, int generations, double mutation_rate) {
    int generation;
    int i, j;
    double best_fitness = INFINITY;
    PID best_individual;

    for (generation = 0; generation < generations; generation++) {
        for (i = 0; i < population_size; i++) {
            double fitness = fitness_function(population[i], target, input, length);
            if (fitness < best_fitness) {
                best_fitness = fitness;
                best_individual = population[i];
            }
        }

        // 选择、交叉和变异操作
        for (i = 0; i < population_size; i++) {
            int parent1_index = rand() % population_size;
            int parent2_index = rand() % population_size;
            while (parent1_index == parent2_index) {
                parent2_index = rand() % population_size;
            }

            crossover(&population[parent1_index], &population[parent2_index], &population[i], &population[i]);
            mutation(&population[i], mutation_rate);
        }
    }

    printf("Best PID parameters: kp=%f, ki=%f, kd=%f\n", best_individual.kp, best_individual.ki, best_individual.kd);
}

int main() {
    // 示例用法
    int population_size = 100;
    int generations = 1000;
    double mutation_rate = 0.1;
    PID population[population_size];
    double target = 1.0; // 目标值
    double input[] = {0.5, 0.6, 0.7, 0.8, 0.9}; // 输入数据序列
    int length = sizeof(input) / sizeof(input[0]); // 输入数据长度

    // 初始化种群
    for (int i = 0; i < population_size; i++) {
        population[i].kp = (rand() % 100) / 100.0;
        population[i].ki = (rand() % 100) / 100.0;
        population[i].kd = (rand() % 100) / 100.0;
    }

    // 运行遗传算法优化PID参数
    genetic_algorithm(population, population_size, target, input, length, generations, mutation_rate);

    return 0;
}
```

### 智能PID控制(dt--杂糅)

```
// 计算智能PID控制器输出
double smart_pid_compute(SmartPIDController *pid, double setpoint, double process_value, double dt) {
    double error = setpoint - process_value; // 计算误差
    pid->integral += error * dt; // 更新积分项
    double derivative = (error - pid->prev_error) / dt; // 计算微分项
    double output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative; // 计算输出
    pid->prev_error = error; // 更新上一次的误差

    // 限制输出范围
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
    }
    return output*dt;
}
```

### 级联PID控制(多级控制)

```
        double control_output1 = cascade_pid_compute(&pid1, setpoint, process_value1, dt);
        double control_output2 = cascade_pid_compute(&pid2, control_output1, process_value2, dt);
        printf("Control output: %f
", control_output2);
        process_value1 += control_output1 * dt; // 假设第一个子系统的响应是线性的
        process_value2 += control_output2 * dt; // 假设第二个子系统的响应是线性的
```

### 混合PID控制（多种控制+模糊）

```
// 计算混合PID控制器输出
double mixed_pid_compute(MixedPIDController *pid, double setpoint, double process_value, double dt) {
    double error = setpoint - process_value; // 计算误差
    pid->integral += error * dt; // 更新积分项
    double derivative = (error - pid->prev_error) / dt; // 计算微分项
    double output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative; // 计算输出
    pid->prev_error = error; // 更新上一次的误差

    // 限制输出范围
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
    }
    return output;
}
```

# LQR

## 简介

**线性二次型调节器，是一种用于设计控制器以使线性系统在给定性能指标下表现最优的经典控制理论方法**

- 核心思想

最小化一个**二次型目标函数**(状态变量的加权平方和   控制输入的加权平方和 )来设计状态反馈控制器.

选择适当的**权重矩阵 Q*Q* 和 R*R***，可以平衡系统性能和控制能量之间的关系

- 注意

1. 仅适用于线性系统，对于非线性系统需要结合其他方法进行处理
2. Q R靠经验整定

```

```

## 力矩控制

### 概念

- **不同负载条件下调整电流以维持所需的力矩**
  - 负载增加时，电机需要更大的力矩来克服阻力。在力矩控制模式下，电机会自动增加电流以提供所需的力矩
- **负载增加时，电机能够通过调节电流来提升输出力矩，从而在最大负载情况下继续转动**
- 

### LQR优势

- **力矩控制系统通常可以近似为线性系统**
  - 电机的转矩输出与其电流输入之间的关系在工作点附近可以近似为线性关系
- **在优化性能和稳定性方面具有显著优势**
  - 最小化一个二次型目标函数来设计控制  包括状态变量和控制输入的加权平方和   系统稳定性的同时，优化系统的动态响应和控制能耗
- **考虑不确定性和外部扰动，使系统鲁棒**
  - 参数变化和外部干扰，如负载变化和摩擦等
- 用Matlab等工具进行仿真和优：根据具体需求调整权重矩阵 Q*Q* 和 R*R*



### 



# MPU6050

## 使用

--返回16进制数据

- 编译器+drc  target+inc

- 变量声名

  ```
  /* USER CODE BEGIN PV */
  short x;
  short y;
  short z;
  /* USER CODE END PV */
  ```

- 初始化

```
  /* USER CODE BEGIN 2 */
	MPU_Init();
  /* USER CODE END 2 */
```

- 打印

```
    /* USER CODE BEGIN 3 */
	MPU_Get_Gyroscope(&x,&y,&z);
	HAL_Delay(100);
	printf("\r\nx=%8x,y=%8x,z=%8x",x,y,z);
  }
  /* USER CODE END 3 */
```

- 重定向

```
/* USER CODE BEGIN 0 */
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);   
	return ch;
}
/* USER CODE END 0 */
```

## 补充

### 滤波

注：减噪，预测

#### (Mahony)互补

- 低频-简单

#### 卡尔曼

##### Q,R整定

- 先验估计
  - 经验法则来估计Q和R。知道测量噪声的标准差，可以将其平方作为R的对角线元素
- 调优方法
  - 交叉验证或其他调优方法来确定最佳的Q和R值，在不同的数据集上训练和测试滤波器，以找到最优的参数组合

##### kalman.h

```
/**
 * @file    kalman.h
 * @author  Liu heng
 * @date    27-August-2013
 * @brief   一阶卡尔曼滤波器模块
 * @details
 *   实现过程完全与硬件无关，可直接调用，任意移植。
 *   使用时先定义一个kalman对象，然后调用kalmanCreate()创建一个滤波器
 *   每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波。
 *
 *          kalman p;
 *          float SersorData;
 *          kalmanCreate(&p,20,200);
 *          while(1)
 *          {
 *             SersorData = sersor();
 *             SersorData = KalmanFilter(&p,SersorData);
 *             printf("%2.2f",SersorData);
 *          }
 *
 *          MPU6050的卡尔曼滤波器参考参数 Q：10 R：400
 */

#ifndef _KALMAN_H
#define _KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一阶卡尔曼滤波器
 */
typedef struct {
    float X_last; //上一时刻的最优结果
    float X_mid;  //当前时刻的预测结果
    float X_now;  //当前时刻的最优结果
    float P_mid;  //当前时刻预测结果的协方差
    float P_now;  //当前时刻最优结果的协方差
    float P_last; //上一时刻最优结果的协方差
    float kg;     //kalman增益
    float A;      //系统参数
    float Q;
    float R;
    float H;
} kalman_filter_t;

/**
  * @brief 初始化一个卡尔曼滤波器
  * @param[out] p 滤波器
  * @param[in] T_Q 系统噪声协方差
  * @param[in] T_R 测量噪声协方差
  */
void kalman_Init(kalman_filter_t *p, float T_Q, float T_R);

/**
  * @brief 卡尔曼滤波器
  * @param[in] p 滤波器
  * @param[in] dat 待滤波信号
  * @retval 滤波后的信号
  */
float Kalman_Filter(kalman_filter_t *p, float dat);

#ifdef __cplusplus
}
#endif

#endif

```

##### kalmax.c

```
#include "kalman.h"

void kalman_Init(kalman_filter_t *p, float T_Q, float T_R) {
    p->X_last = 0.0f;
    p->P_last = 0.0f;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1.0f;
    p->H = 1.0f;
    p->X_mid = p->X_last;
}

float Kalman_Filter(kalman_filter_t *p, float dat) {
    p->X_mid = p->A * p->X_last;                    //x(k|k-1) = AX(k-1|k-1)+BU(k)
    p->P_mid = p->A * p->P_last + p->Q;             //p(k|k-1) = Ap(k-1|k-1)A'+Q
    p->kg = p->P_mid / (p->P_mid + p->R);           //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    p->X_now = p->X_mid + p->kg * (dat - p->X_mid); //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p->P_now = (1 - p->kg) * p->P_mid;              //p(k|k) = (I-kg(k)H)P(k|k-1)
    p->P_last = p->P_now;                           //状态更新
    p->X_last = p->X_now;
    return p->X_now;
}

```



#### **低通滤波器**

来去除高频噪声，保留低频信号；

#### **移动平均滤波器**

来平滑短期内的波动

### 高频采样

### 温度补偿

- 读取温度，对照查找表（温度 & 输出误差Look-Up Table, LUT），确定误差

### 周期性校准陀螺仪

安装过程中，由于机械应力或制造公差等原因，陀螺仪可能会产生零点偏移或灵敏度误差。

通过校准过程，可以确定这些误差并将其从后续的测量结果中扣除

- 静止状态下记录一段时间内的输出值，然后计算平均值作为零点偏移
- 施加已知角速度并记录输出值，以此来标定灵敏度系数

时间推移，陀螺仪可能漂移，即其输出与实际运动状态之间存在持续的差异。定期进行再校准可以纠正这种漂移

### **软件滤波**



### **电磁干扰**

- 远离强磁场
- 接地和屏蔽措施也

### **专用接口**

- I2C或SPI接口可以减少中间环节带来的延迟和误差，从而提高数据传输的效率和可靠性



## mpu6050.h

- 加速度灵敏度
  - short--两字节(short)-- 量程 -32768~+32767
  - 量程2g-->16384 LSB/g
- 陀螺仪灵敏度
  - short--两字节(short)-- 量程 -32768~+32767
  - 量程±2000°/s，最小分辨率变为16.384 LSB/(°/s)

```c
#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx_hal.h"		
//#include "stm32f1xx_hal.h"	用什么系列就是什么									  	  


//#define MPU_ACCEL_OFFS_REG		0X06	//accel_offs寄存器,可读取版本号,寄存器手册未提到
//#define MPU_PROD_ID_REG			0X0C	//prod id寄存器,在寄存器手册未提到
#define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU_SELF_TESTA_REG		0X10	//自检寄存器A
#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG			0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	        0X25	//IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	        0X27	//IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	        0X28	//IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	        0X2A	//IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	        0X2B	//IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	        0X2D	//IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	        0X2E	//IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	        0X30	//IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	        0X31	//IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	        0X34	//IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG	        0X67	//IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	        0X69	//运动检测控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器,who am i寄存器
 
//如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
//如果接V3.3,则IIC地址为0X69(不包含最低位).
#define MPU_ADDR				0X68


//因为MPU6050的AD0接GND,所以则读写地址分别为0XD1和0XD0
//            (如果AD0接VCC,则读写地址分别为0XD3和0XD2)  
#define MPU_READ    0XD1
#define MPU_WRITE   0XD0

uint8_t MPU_Init(void); 						//初始化MPU6050
uint8_t MPU_Write_Len(uint8_t reg,uint8_t len,uint8_t *buf);                           //IIC连续写
uint8_t MPU_Read_Len(uint8_t reg,uint8_t len,uint8_t *buf);                         //IIC连续读 
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data);				//IIC写一个字节
uint8_t MPU_Read_Byte(uint8_t reg);					//IIC读一个字节

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Set_LPF(uint16_t lpf);
uint8_t MPU_Set_Rate(uint16_t rate);
uint8_t MPU_Set_Fifo(uint8_t sens);


float MPU_Get_Temperature(void);
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az);

#endif
```

## mpu6050.c

```
#include "mpu6050.h" 
#include "stdio.h"

//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Init(void)
{ 
  uint8_t res;
  extern I2C_HandleTypeDef hi2c1;
  HAL_I2C_Init(&hi2c1);
  MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
  MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
  MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
  MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
  MPU_Set_Rate(50);						//设置采样率50Hz
  MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
  MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
  MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
  MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
  res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	printf("\r\nMPU6050:0x%2x\r\n",res);
  if(res==MPU_ADDR)//器件ID正确
  {
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
    MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
    MPU_Set_Rate(50);						//设置采样率为50Hz
  }else 
		return 1;
  return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
float MPU_Get_Temperature(void)
{
  unsigned char  buf[2]; 
  short raw;
  float temp;
  
  MPU_Read_Len(MPU_TEMP_OUTH_REG,2,buf); 
  raw=(buf[0]<<8)| buf[1];  
  temp=(36.53+((double)raw)/340)*100;  
//  temp = (long)((35 + (raw / 340)) * 65536L);
  return temp/100.0f;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}


//IIC连续写
uint8_t MPU_Write_Len(uint8_t reg,uint8_t len,uint8_t *buf)
{
  extern I2C_HandleTypeDef hi2c1;
  HAL_I2C_Mem_Write(&hi2c1, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
  HAL_Delay(100);
  
  return 0;
}
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t reg,uint8_t len,uint8_t *buf)
{ 
  extern I2C_HandleTypeDef hi2c1;
  HAL_I2C_Mem_Read(&hi2c1, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
  HAL_Delay(100);
  
  return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data) 				 
{ 
  extern I2C_HandleTypeDef hi2c1;
  unsigned char W_Data=0;

  W_Data = data;
  HAL_I2C_Mem_Write(&hi2c1, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &W_Data, 1, 0xfff);
  HAL_Delay(100);
  
  return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
  extern I2C_HandleTypeDef hi2c1;
  unsigned char R_Data=0;
  
  HAL_I2C_Mem_Read(&hi2c1, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, &R_Data, 1, 0xfff);
  HAL_Delay(100);
  
  return R_Data;		
}
```





# IIC通讯

两线式串行总线，连接微控制器及其外围设备-->主从通信，小数据量，传输距离短，任意时刻只能有一个主机

**低速**通信，v(IIC)<v(SPI)

单字节--仅外设

-  **HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);**

  - ***hi2c** 设置使用的是那个IIC 例：&hi2c2

  - **DevAddress** 写入的地址 设置写入数据的地址 例 0xA0

  - ***pData** 需要写入的数据

  - **Size** 要发送的字节数

  - **Timeout** **最大传输时间**，超过传输时间将自动退出传输函数

```
HAL_I2C_Master_Transmit(&hi2c1,0xA1,(uint8_t*)TxData,2,1000) ；;
```

- **HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);**

  - ***hi2c：** 设置使用的是那个IIC 例：&hi2c2

  - **DevAddress：** 写入的地址 设置写入数据的地址 例 0xA0

  - ***pDat：a** **存储读取到的数据**

  - **Size：** 发送的字节数

  - **Timeout：** **最大读取时间**，超过时间将自动退出读取函数

多字节--字地址

**HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);**

*    第1个参数为I2C操作句柄
     第2个参数为从机设备地址
     第3个参数为从机寄存器地址
     第4个参数为从机寄存器地址长度(

  **MemAddSize：** 从机寄存器地址字节长度 8位或16位

  - - 写入数据的字节类型 **8位还是16位**
  - - I2C_MEMADD_SIZE_8BIT
  - - I2C_MEMADD_SIZE_16BIT

  )

     第5个参数为发送的数据的起始地址
     第6个参数为传输数据的大小
     第7个参数为操作超时时间 　　

注：使用**HAL_I2C_Mem_Write**等于先使用**HAL_I2C_Master_Transmit**传输第一个寄存器地址，再用**HAL_I2C_Master_Transmit**传输写入第一个寄存器的数据。可以传输多个数据

**传输过程，寄存器地址和源数据地址是会自加的**

eg:

8位

```
HAL_I2C_Mem_Write(&hi2c2, ADDR, i, I2C_MEMADD_SIZE_8BIT,&(I2C_Buffer_Write[i]),8, 1000);

HAL_I2C_Mem_Read(&hi2c2, ADDR, i, I2C_MEMADD_SIZE_8BIT,&(I2C_Buffer_Write[i]),8, 1000);
```

16位

```
HAL_I2C_Mem_Write(&hi2c2, ADDR, i, I2C_MEMADD_SIZE_16BIT,&(I2C_Buffer_Write[i]),8, 1000);

HAL_I2C_Mem_Read(&hi2c2, ADDR, i, I2C_MEMADD_SIZE_16BIT,&(I2C_Buffer_Write[i]),8, 1000);
```

注意事项：

- AT24C02的IIC每次写之后要delay 5ms左右 不管硬件IIC采用何种形式（DMA，IT），都要确保两次写入的间隔大于5ms;

- 读写函数最后一个超时调整为1000以上 因为我们一次写8个字节，延时要久一点
- AT24C02页写入只支持8个byte，所以需要分32次写入。这不是HAL库的bug，而是AT24C02的限制，其他的EEPROM可以支持更多byte的写入。

printf重定向

### main.c

```
/* USER CODE BEGIN PV */
#include <string.h>
#define ADDR_24LCxx_Write 0xA0
#define ADDR_24LCxx_Read 0xA1
#define BufferSize 256
uint8_t WriteBuffer[BufferSize],ReadBuffer[BufferSize];
uint16_t i;


/* USER CODE END PV */
  /* USER CODE BEGIN 2 */
	for(i=0; i<256; i++)
    WriteBuffer[i]=i;    /* WriteBuffer init */


		printf("\r\n***************I2C Example Z小旋测试*******************************\r\n");
			for (int j=0; j<32; j++)
        {
                if(HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, 8*j, I2C_MEMADD_SIZE_8BIT,WriteBuffer+8*j,8, 1000) == HAL_OK)
                {
                                printf("\r\n EEPROM 24C02 Write Test OK \r\n");
                        HAL_Delay(20);
                }
                else
                {
                         HAL_Delay(20);
                                printf("\r\n EEPROM 24C02 Write Test False \r\n");
                }
		}
		/*
		// wrinte date to EEPROM   如果要一次写一个字节，写256次，用这里的代码
		for(i=0;i<BufferSize;i++)
		{
		    HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, i, I2C_MEMADD_SIZE_8BIT,&WriteBuffer[i],1，0xff);//使用I2C块读，出错。因此采用此种方式，逐个单字节写入
		  HAL_Delay(5);//此处延时必加，与AT24C02写时序有关
		}
		printf("\r\n EEPROM 24C02 Write Test OK \r\n");
		*/

		HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, 0, I2C_MEMADD_SIZE_8BIT,ReadBuffer,BufferSize, 0xff);

		for(i=0; i<256; i++)
			printf("0x%02X  ",ReadBuffer[i]);
			
  /* USER CODE END 2 */
```

# FreeRtos

注：

- xTask -- 功能丰富，原生接口--底层

·      osThread -- 功能受限， CMSIS-rtos兼容--中间层

- 定时器不可阻塞-->挂起

- 任务执行顺序：高优先级>低优先级 同优先级等分时间片

- 任务终止：

  - 使用标志变量：任务完成时将其设置为True或False、
  - 异常处理：try-except语句来捕获异常并终止循环 捕获到异常 设置一个标志变量|直接退出循环
  - 条件判断
  - 线程或进程：任务封装在一个独立的线程或进程，主程序中使用线程或进程的API来等待任务完成，

- 调度器调度状态：

  - 创建任务

  ```
  BaseType_t xTaskCreate(	TaskFunction_t pxTaskCode, //函数指针
  							const char * const pcName,//任务名
  							const configSTACK_DEPTH_TYPE usStackDepth,//栈深度
  							void * const pvParameters,//所传参数
  							UBaseType_t uxPriority,//优先级
  							TaskHandle_t * const pxCreatedTask )//任务句柄-->后期操作
                              
  //宏定义--静态线程			
  osThreadDef(myTask02 -- 名, StartTask02--函数指针, osPriorityIdle 优先级, 1--线程实例数, 128--线程栈大小，buffer--（TCB）线程控制块缓冲区大小，control--（TCB -- task control block）线程控制块指针（句柄），存储线程状态信息);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);
  ```
  
  
  
  - 获取线程句柄-->指向控制块的指针
  
  ```
  osThreadId_t myTask02Handle = osThreadGetId();
  
  TaskHandle_t xTaskHandle = xTaskGetHandle("Task1");  
  ```
  
  - 启动
  
  ```
  // 启动任务调度器 -- 管理已存在线程--启动前必至少存在一个线程
  vTaskStartScheduler();
  osKernelStart();
  ```
  
  - 就绪（Ready）：可运行，等CPU资源，抢占低优先级资源
  
    - `osThreadCreate`：创建一个新的线程并将其设置为就绪状态。
  
    ```
      osThreadId_t myTask02Handle; 
      osThreadDef(myTask02, StartTask02--函数, osPriorityIdle, 1--实例, 128);
      myTask02Handle = osThreadCreate(osThread(myTask02), NULL);
    ```
    
    - `osThreadYield`：主动让出CPU给其他就绪状态的线程，重新就绪
    
    ```
    osThreadYield();
    ```
    
  - 运行（Runing）： 正运行，等高优先级，一旦高优先级准备就绪，中断自己
  
    - `osThreadGetId`：获取当前正在运行的线程ID--一旦创建，即可运行
  
    osThreadId_t currentThreadId = osThreadGetId();
    printf("Current thread ID: %u\n", currentThreadId);
    
    - `osThreadExit`：结束当前线程的执行。
  - TaskHandle_t xHandle = xTaskGetCurrentTaskHandle();
    - vTaskDelete(xHandle);
  
  - 挂起（Suspend）： 停运行（内存保留），让低优先级，即使就绪，不抢占
  
    - `vTaskSuspend(xTaskHandle xTaskToSuspend)`  --线程句柄->指向控制块的指针
    - vTaskResume() 恢复
    
    - `osThreadSuspend`（）挂起
    
    - `osThreadResume` （）恢复
    
  - 删除
  
  ```
  NULL
  
  vTaskDelete(TaskHandle_t xTaskToDelete);
  ```
  
  - 阻塞态（blocked）等待信号量、消息队列、事件标准组、系统延时时，被称为阻塞态，如果等待的事件到了，就会自动退出阻塞态，准备运行

### 延时

**void vTaskDelay( const TickType_t xTicksToDelay )**

挂起(Suspend)指定时钟节拍，并就绪(Ready),(Runing)时     从停止处执行|被打断，挂起时间变长

```
void vTaskFunction(void *pvParameters) {
    for (;;) {
        // 延迟500毫秒（假设系统时钟频率为1000Hz）
        vTaskDelay(500);
        // 在这里执行任务相关的代码
    }
}
```

**void vTaskDelayUntil( TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement )**

上次唤醒的时间（时钟节拍）+ 延迟的时间增量（始终节拍） 从停止处执行|绝对延时，会被打断

```
TickType_t xLastWakeTime;
void vTaskFunction(void *pvParameters) {
    // 初始化上次唤醒时间为当前时间
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        // 延迟1秒（假设系统时钟频率为1000Hz）
        vTaskDelayUntil(&xLastWakeTime, 1000);
        // 在这里执行任务相关的代码
    }
}
```

### 

### 消息队列

#### **创建**

```
**QueueHandle_t xQueueGenericCreate( const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, const uint8_t ucQueueType )**
```

- `xQueueGenericCreate`函数用于创建一个动态队列。以下是各个参数的作用：

  1. `uxQueueLength`：队列的长度，即队列中可以容纳的最大元素数量。

  2. `uxItemSize`：每个队列元素的字节大小。

  3. ```
     ucQueueType
     ```

     ：队列的类型，可以是以下值之一：

     - `queueQUEUE_TYPE_BASE`：基本类型队列，不支持优先级。
     - `queueQUEUE_TYPE_BINARY_SEMAPHORE`：二进制信号量队列，主要用于同步和互斥。
     - `queueQUEUE_TYPE_COUNTING_SEMAPHORE`：计数信号量队列，主要用于资源管理。
     - `queueQUEUE_TYPE_MUTEX`：互斥锁队列，主要用于保护共享资源的访问。
     - `queueQUEUE_TYPE_RECURSIVE_MUTEX`：递归互斥锁队列，允许同一任务多次获取同一个互斥锁。
     - `queueQUEUE_TYPE_BLOCKING_ON_QUEUE_FULL`：当队列满时阻塞写入操作。
     - `queueQUEUE_TYPE_BLOCKING_ON_QUEUE_EMPTY`：当队列为空时阻塞读取操作。

- return NULL --失败 | 非0，成功

- 动态--内存可调--资源丰富

```
**QueueHandle_t xQueueGenericCreateStatic( const UBaseType_t uxQueueLength -- 长度, const UBaseType_t uxItemSize  -- 元素大小, uint8_t *pucQueueStorage -- 缓存区指针, StaticQueue_t *pxStaticQueue -- 队列信息结构体, const uint8_t ucQueueType )**
```

- `ucQueueType`：队列的类型，可以是以下值之一：
  - `queueQUEUE_TYPE_BASE`：基本类型队列，不支持优先级。
  - `queueQUEUE_TYPE_BINARY_SEMAPHORE`：二进制信号量队列，主要用于同步和互斥。
  - `queueQUEUE_TYPE_COUNTING_SEMAPHORE`：计数信号量队列，主要用于资源管理。
  - `queueQUEUE_TYPE_MUTEX`：互斥锁队列，主要用于保护共享资源的访问。
  - `queueQUEUE_TYPE_RECURSIVE_MUTEX`：递归互斥锁队列，允许同一任务多次获取同一个互斥锁。
  - `queueQUEUE_TYPE_BLOCKING_ON_QUEUE_FULL`：当队列满时阻塞写入操作。
  - `queueQUEUE_TYPE_BLOCKING_ON_QUEUE_EMPTY`：当队列为空时阻塞读取操作

#### **队列复位**

```
**xQueueReset(QueueHandle_t  xQueue)**

**xQueueGenericReset(QueueHandle_t xQueue, UBaseType_t uxQueueLength = 10, uint8_t ucQueueType = queueQUEUE_TYPE_BASE)**
```

#### **队列删除**

```
**vQueueDelete(QueueHandle_t xQueue);**

**vQueueDelete(QueueHandle_t xQueue, uint8_t ucQueueType = queueQUEUE_TYPE_BASE, UBaseType_t uxQueueLength = 10);** 
```

#### **队列写入**

fifo--后入前出

//基本写入

```
xQueueSend( xQueue_Handle队列句柄, 数据指针, xTicksToWait（0--不等待，立即发送）)；
```

- 任务中向队列发送数据--非中断服务
- 队列满--可溢出

```
xQueueSendToFront(xQueue, &data, (TickType_t)0);*// 将数据写入队列的前端*

xQueueSendToBack(xQueue, &data, (TickType_t)0);*// 将数据写入队列的后端*

 xQueueSendFromISR(xQueue, &data, &xHigherPriorityTaskWoken)//中断前端--实时性高--传感器采集
```

- `&xHigherPriorityTaskWoken`：这是一个指向布尔值的指针，用于指示是否有更高优先级的任务被唤醒。如果设置为`pdTRUE`，则表示有更高优先级的任务被唤醒，需要重新调度。如果设置为`pdFALSE`，则表示没有更高优先级的任务被唤醒

```
BaseType_t xHigherPriorityTaskWoken = pdFALSE;
uint32_t data = getSensorData(); // 获取传感器数据
xQueueGenericSendFromISR(xQueue, &data, pdFALSE, &xHigherPriorityTaskWoken);
portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
```

- bool--是否有更高优先级的任务被唤醒--
  - pdTRUE:有更高优先级的任务被唤醒，需要重新调度
  - pdFALSE:没有更高优先级的任务被唤醒,不需要进行任务调度

```
xQueueSendToFrontFromISR( xQueue,  &data, pxHigherPriorityTaskWoken )

xQueueSendToBackFromISR( xQueue,  &data, pxHigherPriorityTaskWoken )
```

//通用写入--后端写入

```
xQueueGenericSend(QueueHandle_t xQueue--句柄, &data--数据指针, (TickType_t)0延迟时间（可选）, uint8_t ucQueueType = queueQUEUE_TYPE_BASE--队列类型);  

xQueueGenericSendFromISR(QueueHandle_t  xQueue, &data, pdFALSE, (TickType_t)0, ucQueueType);
```

#### **队列读取**

**普通**

xQueueReceive( xQueue_Handle队列句柄， 数据指针，  (TickType_t)0（0--不等待，立即接收）);// 从队列后端读取数据，并移除元素

- 任务中从队列接收数据--非中断服务
- 队列空--阻塞等待

```
xQueuePeek(xQueue, &data, (TickType_t)0); // 从队列前端读取数据，但不移除元素

xQueuePeekFromISR(xQueue, &data); // 从队列前端读取数据，但不移除元素
```

**中断**

xQueueReceiveFromISR(xQueue, &data, &xHigherPriorityTaskWoken); // 从队列后端读取数据，并移除元素

- `&xHigherPriorityTaskWoken`：这是一个指向布尔值的指针，用于指示是否有更高优先级的任务被唤醒。如果设置为`pdTRUE`，则表示有更高优先级的任务被唤醒，需要重新调度。如果设置为`pdFALSE`，则表示没有更高优先级的任务被唤醒

```
        复制代码运行    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t data;
    xQueueGenericReceiveFromISR(xQueue, &data, pdFALSE, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    
```

- bool--是否有更高优先级的任务被唤醒--
  - pdTRUE:有更高优先级的任务被唤醒，需要重新调度
  - pdFALSE:没有更高优先级的任务被唤醒，不需要进行任务调度

**查询**

```
**uxQueueMessagesWaiting(Handle)**  当前队列中有多少个队列元素

 **uxQueueSpacesAvailable(Handle)**   队列中还有多少可用空间
```

**偷窥**  ---- 队列首元素

```
**BaseType_t xQueuePeek( QueueHandle_t xQueue, void * const pvBuffer -- 保存缓冲区, TickType_t xTicksToWait )**

**BaseType_t xQueuePeekFromISR( QueueHandle_t xQueue,  void * const pvBuffer )**
```



### 二值信号量--同步机制，消息访问--0或1的资源数量

- **信号量**

  - 信号量的计数值都有限制：限定[最大值](https://so.csdn.net/so/search?q=最大值&spm=1001.2101.3001.7020)。

    - 如果最大值被限定为1，那么它就是二值信号量；

    - 如果最大值不是1，它就是计数型信号量。

  - 当计数值大于0，代表有信号量资源

    - 当释放信号量，信号量计数值（资源数）加一

    - 当获取信号量，信号量计数值（资源数）减一

  - 信号量用于传递状态

#### 创建

```
SemaphoreHandle_t xSemaphoreCreateBinary(void);  //动态创建

SemaphoreHandle_t xSemaphoreCreateBinary(void);	 //静态创建
```

#### **获取**

注：如果信号量处于空闲状态，获取成功并变为占用状态；如果已被占用，则根据指定的阻塞时间等待或立即返回

```
BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t blockTime);

BaseType_t xSemaphoreTakeFromISR(SemaphoreHandle_t xSemaphore, TickType_t blockTime);
```

同步刷新--避免轮询--参数简单

#### **释放**

注：将信号量从占用状态转变为空闲状态，允许其他任务获取

```
BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);

BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t xSemaphore);   
```

#### **检查**

注：查询二值信号量的当前状态（空闲或占用），但不改变其状态

```
UBaseType_t xSemaphoreGenericGetCount(SemaphoreHandle_t xSemaphore);
```

#### 删除

注：删除一个不再需要的信号量，释放其占用的资源

```
void vSemaphoreDelete(SemaphoreHandle_t xSemaphore);
```



#### 实例

```
#include "freertos_demo.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/KEY/key.h"
/*FreeRTOS*********************************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//FreeRTOS配置
//1.任务配置
//1.1 START_TASK 任务 配置

#define START_TASK_PRIO 1                   /* 任务优先级 */
#define START_STK_SIZE  128                 /* 任务堆栈大小 */
TaskHandle_t            StartTask_Handler;  /* 任务句柄 */
void start_task(void *pvParameters);        /* 任务函数 */

//1.2 TASK1 任务 配置

#define TASK1_PRIO      2                   /* 任务优先级 */
#define TASK1_STK_SIZE  128                 /* 任务堆栈大小 */
TaskHandle_t            Task1Task_Handler;  /* 任务句柄 */
void task1(void *pvParameters);             /* 任务函数 */

//1.2 TASK2 任务 配置

#define TASK2_PRIO      3                   /* 任务优先级 */
#define TASK2_STK_SIZE  128                 /* 任务堆栈大小 */
TaskHandle_t            Task2Task_Handler;  /* 任务句柄 */
void task2(void *pvParameters);             /* 任务函数 */

//1.3 二值信号量句柄定义
SemaphoreHandle_t        binarySemaphore;  

//2.在freertos_demo函数中创建start_task任务
void freertos_demo(void)
{

    xTaskCreate((TaskFunction_t )start_task,            /* 任务函数 */
                (const char*    )"start_task",          /* 任务名称 */
                (uint16_t       )START_STK_SIZE,        /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )START_TASK_PRIO,       /* 任务优先级 */
                (TaskHandle_t*  )&StartTask_Handler);   /* 任务句柄 */
    vTaskStartScheduler();
}

//3.在start_task函数中创建task1、task2任务
void start_task(void *pvParameters)
{
    
    //进入临界区
    taskENTER_CRITICAL();
    //创建二值信号量
     binarySemaphore = xSemaphoreCreateBinary(); 
    if(binarySemaphore != NULL)
    {
        printf("二值信号量创建成功！！！\r\n");
    }
    
    //创建任务1
      xTaskCreate((TaskFunction_t )task1,            /* 任务函数 */
                (const char*    )"task1",          /* 任务名称 */
                (uint16_t       )TASK1_STK_SIZE,        /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )TASK1_PRIO,       /* 任务优先级 */
                (TaskHandle_t*  )&Task1Task_Handler);   /* 任务句柄 */
     //创建任务2          
     xTaskCreate((TaskFunction_t )task2,            /* 任务函数 */
                (const char*    )"task2",          /* 任务名称 */
                (uint16_t       )TASK2_STK_SIZE,        /* 任务堆栈大小 */
                (void*          )NULL,                  /* 传入给任务函数的参数 */
                (UBaseType_t    )TASK2_PRIO,       /* 任务优先级 */
                (TaskHandle_t*  )&Task1Task_Handler);   /* 任务句柄*/
                
      //删除开始任务        
      vTaskDelete(StartTask_Handler);   
                
     //退出临界区
     taskEXIT_CRITICAL();
}


//4.在task1函数中释放二值信号量
void task1(void *pvParameters)
{
    uint8_t key = 0;
    BaseType_t err = 0;
    
    while (1)
    {
        //按键KEY0控制二值信号释放
        key = key_scan(0);
        switch (key)
        {
            case KEY0_PRES:
            {
                //如果二值信号量创建成功
                if(binarySemaphore != NULL)
                {
                     err = xSemaphoreGive(binarySemaphore);                //二值释放信号量
                    if(err == pdPASS)
                    {
                       printf("binarySemaphore释放成功\r\n");
                    }
                    else
                    {
                        printf("binarySemaphore释放失败\r\n");
                    }
               
                }
                 break;
               
            }
            default:
            {
                break;
            }
        }
        vTaskDelay(10);
    }
}

//5.在task2函数中获取二值信号量
void task2(void *pvParameters)
{
    BaseType_t err = 0;
    while(1)
    {
         err = xSemaphoreTake(binarySemaphore,portMAX_DELAY);
        if(err == pdPASS)
        {
            printf("binarySemaphore获取成功\r\n");
        }
        else printf("binarySemaphore获取失败\r\n");
    
    }
   
}
```



### 计时信号量--0或无穷的资源数量

#### 创建

```
SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t maxCount, UBaseType_t initialCount);

SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t maxCount, UBaseType_t initialCount);	 //静态创建
```

#### 获取

```
BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t blockTime);

BaseType_t xSemaphoreTakeFromISR(SemaphoreHandle_t xSemaphore, TickType_t blockTime);
```

#### 释放

```
BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);    

BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t xSemaphore);    
```

#### 检查

```
UBaseType_t xSemaphoreGenericGetCount(SemaphoreHandle_t xSemaphore);
```

#### 删除

```
void vSemaphoreDelete(SemaphoreHandle_t xSemaphore);
```

### 互斥量--狐假虎威--完事释放

背景：低优先级的任务执行后高优先级任务才能执行，但是中优先级会插低优先级任务的队，等于是插了高优先级任务的队，把高优先级任务和低优先级任务绑定起来，让中优先级的任务等低优先级的任务，不会插到高优先级任务的队。

注：互斥信号量是用于实现任务间的互斥，确保任意时刻只有一个任务能够访问共享资源。以下是互斥信号量的操作函数

**会被中断打断**

1. **创建互斥信号量**：使用`xSemaphoreCreateMutex()`函数可以创建一个互斥信号量。该函数返回一个互斥信号量的句柄，可以用于后续的操作。

   ```
   SemaphoreHandle_t xSemaphoreCreateMutex(void);
   ```

2. **获取互斥信号量**：`xSemaphoreTake()`函数用于获取互斥信号量。如果信号量处于未锁定状态，获取成功并将其设置为锁定状态；如果已被锁定，则根据指定的阻塞时间等待或立即返回。

   ```
   BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t blockTime);    
   ```

3. **释放互斥信号量**：`xSemaphoreGive()`函数用于释放互斥信号量。它将信号量从锁定状态转变为未锁定状态，允许其他任务获取。

   ```
   BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);    
   ```

4. **尝试获取互斥信号量**：`xSemaphoreTryTake()`函数尝试获取互斥信号量，但不会阻塞等待。如果信号量未锁定，则获取成功并锁定；如果已锁定，则立即返回。

   ```
   BaseType_t xSemaphoreTryTake(SemaphoreHandle_t xSemaphore);   
   ```

5. **检查互斥信号量**：`ulTaskNotifyTake()`函数通常用于检查任务是否持有特定的互斥信号量。

   ```
   TickType_t xTaskNotifyWait(uint32_t clearBits, TickType_t ticksToWait, const BaseType_t *const pxClearedBits);  
   ```

6. **删除互斥信号量**：`vSemaphoreDelete()`函数用于删除一个不再需要的互斥信号量，释放其占用的资源。

   ```
   void vSemaphoreDelete(SemaphoreHandle_t xSemaphore);
   ```

### 事件组--执行条件（多条件 & |）

注：用于在多任务环境中同步任务，允许一个任务等待多个事件的发生。以下是事件组的操作函数

1. **创建事件组**：使用`xEventGroupCreate()`函数可以创建一个事件组。该函数返回一个事件组的句柄，用于后续的操作。

   ```
   EventGroupHandle_t xEventGroupCreate(void);
   ```

2. **设置位集合中的位**：`xEventGroupSetBits()`函数用于在事件组中设置一个或多个位。这表示相关的事件已经发生，可以被其他任务检测到。

   ```
   EventBits_t xEventGroupSetBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet);   
   ```

3. **清除位集合中的位**：`xEventGroupClearBits()`函数用于在事件组中清除一个或多个位。这表示某些事件已经不再有效或被处理。

   ```
   EventBits_t xEventGroupClearBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToClear);   
   ```

4. **等待事件位集合中的位**：`xEventGroupWaitBits()`函数允许一个任务等待事件组中指定的位被设置。任务可以等待一个或多个位，在事件发生前会被阻塞。

   ```
   EventBits_t xEventGroupWaitBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor, const BaseType_t xClearOnExit, TickType_t ticksToWait);    
   ```

5. **获取当前事件位集合的状态**：`xEventGroupGetBits()`函数用于获取当前事件组的状态，即哪些位已经被设置。

   ```
   EventBits_t xEventGroupGetBits(EventGroupHandle_t xEventGroup);    
   ```

6. **删除事件组**：`vEventGroupDelete()`函数用于删除一个不再需要的事件组，释放其占用的资源。

   ```
    void vEventGroupDelete(EventGroupHandle_t xEventGroup);
   ```

### 任务通知--少用

任务通知是一种机制，允许一个任务向另一个任务发送消息或通知。以下是任务通知的实际操作函数：

1. **创建任务通知**：使用`xTaskNotifyCreate()`函数可以创建一个任务通知。该函数返回一个句柄，用于后续的操作。

   ```
   TaskHandle_t xTaskNotifyCreate(TaskFunction_t pxTaskCode, const char *pcName, uint32_t usStackDepth, void *pvParameters, UBaseType_t uxPriority, StackType_t *pxStackBuffer, StaticTask_t *pxTaskBuffer);
   ```

2. **发送任务通知**：`xTaskNotify()`函数用于发送任务通知给指定的任务。

   ```
   BaseType_t xTaskNotify(TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction);    
   ```

3. **接收任务通知**：`ulTaskNotifyTake()`函数用于接收任务通知。如果任务没有收到通知，它将阻塞等待直到有通知到来。

   ```
   uint32_t ulTaskNotifyTake(BaseType_t xClearCountOnExit, TickType_t xTicksToWait);  
   ```

4. **清除任务通知值**：`xTaskNotifyClear()`函数用于清除任务的通知值。这通常在任务处理完通知后调用，以便下一次接收通知时不会受到之前的通知影响。

   ```
   void xTaskNotifyClear(TaskHandle_t xTaskToNotify);   
   ```

5. **获取任务通知值**：`ulTaskNotifyValue()`函数用于获取任务的通知值。它返回最近接收到的通知值。

   ```
   uint32_t ulTaskNotifyValue(TaskHandle_t xTaskToQuery);
   ```

6. **中断服务程序（ISR）中通知任务**：`ulTaskNotifyValue()`函数用于获取任务的通知值。它返回最近接收到的通知值。

- `deal_task_handle`: 这是你想要通知的任务的句柄
- `ulNotificationValue`: 这是你想要传递给任务的通知值。通常是一个无符号长整型变量，可以是任何有效的数值。
- `eSetBits` 是一个枚举值，表示我们要设置通知的值
  - `eSetBits`: 将通知值设置为指定的值。
  - `eIncrement`: 将通知值增加指定的值。
  - `eSetValueWithOverwrite`: 如果任务尚未收到通知，则将通知值设置为指定的值；如果任务已经收到通知，则忽略此操作。
- `&xHigherPriorityTaskWoken`: 这是一个指向布尔变量的指针，用于指示是否有更高优先级的任务被唤醒,值为真，那么调度器将会重新评估当前运行的任务，以确保最高优先级的任务能够运行

```
// 假设你已经定义了一个任务句柄和一个布尔变量
TaskHandle_t deal_task_handle;
uint32_t ulNotificationValue = 10;
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

// 在某个中断服务程序中调用 vTaskNotifyGiveFromISR
void ISR_Handler(void) {
    // 检查是否有必要唤醒任务
    if (deal_task_handle != NULL) {
        // 使用 vTaskNotifyGiveFromISR 发送通知
        xTaskNotifyFromISR(deal_task_handle, ulNotificationValue, eSetBits, &xHigherPriorityTaskWoken);
    }
}
```



### 软件定时器--基于时钟节拍

1. **创建定时器**：`xTimerCreate()` 函数用于创建一个软件定时器。

   ```
   TimerHandle_t xTimerCreate(const TimerCallbackFunction_t *pxCallbackFunction, const char *pcName, TickType_t xTimerPeriodInTicks, void *pvTimerID, TimerType_t xTimerType); 
   ```

2. **启动定时器**：`xTimerStart()` 函数用于启动一个已经被创建但停止的定时器，使其开始计时并在到期时调用回调函数。

   ```
   BaseType_t xTimerStart(TimerHandle_t xTimer, TickType_t xTicksToWait);    
   ```

3. **停止定时器**：`xTimerStop()` 函数用于停止一个正在运行的定时器，阻止其到期时调用回调函数。

   ```
   BaseType_t xTimerStop(TimerHandle_t xTimer, BaseType_t xIndefinitely);    
   ```

4. **重启定时器**：`xTimerReset()` 函数用于重启一个正在运行或已经到期的定时器，重新计时。

   ```
   BaseType_t xTimerReset(TimerHandle_t xTimer, TickType_t xTicksToWait);   
   ```

5. **更改定时器周期**：`xTimerChangePeriod()` 函数用于更改一个定时器的周期。

   ```
   BaseType_t xTimerChangePeriod(TimerHandle_t xTimer, TickType_t xNewPeriodInTicks, TickType_t xTicksToWait);   
   ```

6. **定时器控制**：`xTimerCtrl()` 函数是一个通用的控制函数，可以执行多种定时器相关的操作，如检查定时器的状态、获取剩余时间等。

   ```
   TimerControlReturn_t xTimerGenericCommand(TimerHandle_t xTimer, TimerCmd_t xCmd, void *pvData, uint32_t xDataLength, void *pxReturnedValue, uint32_t xReturnedValueLength, TickType_t xTicksToWait);    
   ```

7. **删除定时器**：`xTimerDelete()` 函数用于删除一个定时器，释放其占用的资源。

   ```
   void vTimerDelete(TimerHandle_t xTimer); 
   ```

8. **定时器到期回调函数**：当定时器到期时，将自动调用创建定时器时指定的回调函数。

```
void vTimerCallbackFunction(void *pvTimerID);
```

### 中断

#### 中断任务切换--ISR(中断回调)释放信号量、发送事件、直接切换任务

 **xTaskGetSchedulerState()**

此函数用于获取当前调度器的状态，即是否正在运行由任务调度器管理的任务。

- **原型**：`BaseType_t xTaskGetSchedulerState(void);`

- `pdTRUE`：如果调度器正在执行任务。
- `pdFALSE`：如果调度器未在执行任务（例如，因为在临摹或中断上下文中）。

**xTaskIncrementTick()**

此函数用于手动增加系统节拍计数器，通常在中断服务例程（ISR）中使用，以触发任务调度。

- **原型**：`void xTaskIncrementTick(void);`

```
// 假设在ISR中使用
void vAnInterruptServiceRoutine(void)
{
    // 处理中断相关的工作...
    // 增加系统节拍以触发任务切换（如果需要）
    xTaskIncrementTick();
} 
```

**任务切换判断**

```
// 创建二值信号量
SemaphoreHandle_t xBinarySemaphore = NULL;
void vTaskFunction(void *pvParameters) {
    // 任务主体，尝试获取信号量--阻塞等待 || 信号量可用
    if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
        // 成功获取信号量，执行任务逻辑
        // ...
    }
}
void ISR_Handler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // 只想bool的指针，指示是否有更高优先级的任务因为释放信号量而被唤醒。如果释放信号量导致高优先级任务变为就绪状态，那么这个变量会被设置为 pdTRUE
    // 检查特定条件是否满足（如外设事件完成）
    if (condition_met) {
        // 释放信号量，通知任务可以继续执行
        xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
    }
    // 如果需要切换任务，调用portYIELD_FROM_ISR宏进行上下文切换
    //检查 xHigherPriorityTaskWoken 是否为 pdTRUE，即是否有更高优先级的任务被唤醒
    //如果条件成立，说明有高优先级任务需要执行，因此调用 portYIELD_FROM_ISR() 宏来触发上下文切换
    //ortYIELD_FROM_ISR() 是 FreeRTOS 中的一个宏，用于在中断服务程序中请求操作系统进行任务切换。当调用此宏时，操作系统会暂停当前正在运行的任务，保存其上下文，然后调度最高优先级的就绪任务开始执行
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}
int main(void) {
    // 初始化FreeRTOS系统
    // ...
    // 创建二值信号量
    xBinarySemaphore = xSemaphoreCreateBinary();
    // 创建任务并启动调度器
    xTaskCreate(vTaskFunction, "Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();

    // 主循环（在任务中不需要）
    while (1) {
        // ...
    }
}
```



### 中断优先级

- 任何中断优先级>任务
- 必须使用的符合中断的任务任务函数
- 时间短

### 任务中的中断屏蔽

用于保护临界区代码段，确保在执行临界区代码时不会被中断打断

```
void criticalSectionFunction(void) {
    // 进入临界区，禁用中断
    vPortEnterCritical();
    // 在这里执行临界区的代码，例如访问共享资源或执行关键任务
    // ...
    // 退出临界区，重新启用中断
    vPortExitCritical();
}
```

### 中断中的中断屏蔽函数

```
void criticalSectionFunction(void) {
    // 进入临界区，禁用中断
    vPortEnterCritical();
    // 在这里执行临界区的代码，例如访问共享资源或执行关键任务
    // ...
    // 退出临界区，重新启用中断
    vPortExitCritical();
}
```



### 创建MyTask

- 清空freeRTOS.c
- 工程文件 + 魔法棒-->导入.c  .h

#### ourTask.h

```
#ifndef __OUR_TASK_H
#define __OUR_TASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#endig
```

#### ourTask.c

# ADC电压采集

## 独立模式单通道采集

电位器电压的采集，并通过串口打印至 PC 端串口调试助手。单 通道采集适用 AD 转换完成中断，在中断服务函数中读取数据，不使用 DMA 传输，在多通道采 集时才使用 DMA 传输

**main.c**

```
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t ADC_ConverteValue = 0; //定义全局变量，存储数值
/* USER CODE END PD */

/* USER CODE BEGIN 4 */ 
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) //ADC采集中断回调服务函数
{
	ADC_ConverteValue = HAL_ADC_GetValue(&hadc1);
} 
/* USER CODE END 4 */

/* USER CODE BEGIN 2 */
    HAL_ADC_Start(&hadc1);   	//手动开启
    
 	char buffer[64]; // 定义一个足够大的缓冲区来存储格式化后的字符串
    int data = 42; // 假设这是你要发送的数据
    while (1)
    {
        // 使用sprintf函数将数据格式化为字符串
        sprintf(buffer, "Data: %d
    ", data);
        // 使用HAL库的串口发送函数将字符串发送到上位机
        HAL_UART_Transmit(&huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
        HAL_Delay(500);
    }
/* USER CODE END 2 */
```

## 独立模式多通道采集

开发板已通过排针接口把部分 ADC 通道引脚引出，其中电位器通过跳线帽默认接了一个 ADC 的 IO，其他的 ADC IO 在做实验的时候可以用杜邦线连接到开发板中的 GND 或者 3V3 来获取模 拟信号

做 ADC 输入的 IO **不能被复用**，否则会导致采集到的**信号不准确**

1) 初始化 ADC GPIO；
2) 初始化 ADC 工作参数；
3) 配置 DMA 工作参数；
4) 读取 ADC 采集的数据；

**HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_ConverteValue, 6);**

- `ADC_HandleTypeDef* hadc`: 这是一个指向ADC句柄的指针，该句柄包含了ADC的配置信息。
- `uint32_t* pData`: 这是一个指向32位无符号整型的指针，用于存储ADC转换后的数据。
- `uint32_t Length`: 表示要获取的ADC数据样本的数量--通道数

**main.c**

```
/* USER CODE BEGIN PD */
uint16_t ADC_ConverteValue[6] = {0}; //定义全局变量，存储数值
/* USER CODE END PD */

int main(){
  ADC_Enable(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_ConverteValue, 6);
	while(1){
	printf("\r\n CH0_PA0 value = %f V \r\n", (float)ADC_ConvertedValueLocal[0]/4096.0*3.3);
	//4096 = 2^12
	}
}

```

**usart.c**

```
//printf重写
```

**Keil开发STM32时开启微库（MicroLIB）的主要作用是优化代码大小和内存使用**

## 双重 ADC 同步规则模式采集编程要点

1) 初始化 ADC GPIO；
2) 初始化 DMA 配置；
3) 初始化 ADC 参数；
4) 读取 ADC 采集的数据，并打印出来校正；

原来一个ADC采集一个通道最快速率是0.85Mps，如果采用两个ADC外设交替采集同一个通道，速率可以提升两倍为1.7Mps

##### MDK程序完善

如果使用三个ADC交替读取，我们需要定义一个全局数组用于保存ADC采样的结果，数组要定义大一点，因为数组要作为DAM的缓冲区，如果定义得很小，那么会频繁地进入DMA的中断处理函数，这样会导致CPU花大量时间在中断切换上

**HAL_ADCEx_Calibration_Start(&hadc1)**

用于启动ADC校准的函数:ADC初始化后，通常需要进行一次校准来消除内部偏移误差

**main.c**

```
uint32_t ADC123_Buff[3000];
uint32_t ADC_ConvertedValue;
int main(){
//添加启动ADC采样器的校准代码并启动 AD 转换并使能 DMA 传输和中断
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);
 
  HAL_ADC_Start(&hadc2);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, &ADC_ConverteValue, 4);
	while(1){
	printf("\r\n ADC1_l:%f V \r\n", (float)(uint16_t)ADC_ConvertedValue/4096.0*3.3);
	printf("\r\n ADC2_H:%f V \r\n", (float)(ADC_ConvertedValue>>16)/4096.0*3.3);
	//4096 = 2^12
	}
}
```

**usart.c**

```
//printf重写
```

# 看门狗

## 独立看门狗

- **HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg)** //初始化
- **HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg)**  //喂狗

## 窗口看门狗

- HAL_WWDG_Init(WWDG_HandleTypeDef *hwwdg);//窗口看门狗软件初始化
- HAL_WWDG_MspInit(WWDG_HandleTypeDef *hwwdg);//窗口看门狗硬件（MCU）初始化
- HAL_WWDG_Refresh(WWDG_HandleTypeDef *hwwdg);//窗口看门狗喂狗
- HAL_WWDG_IRQHandler(WWDG_HandleTypeDef *hwwdg);//窗口看门狗中断服务函数
- HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg);//窗口看门狗回调函数，弱定义函数可自定义开发

**main.c**

```
void MX_WWDG_Init(void)
{
  hwwdg.Instance = WWDG;																		//句柄
  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;//1分频（不分频）
  hwwdg.Init.Window = 0x4f;															//上窗口值
  hwwdg.Init.Counter = 0x7f;														//计数值
  hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;			//提前中断使能（计数器递减到0x40时将提前进入中断做复位前重要数据保存等处理）
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)				//初始化使能进入该函数将会调用 HAL_WWDG_MspInit
  {
    Error_Handler();
  }

}


int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_WWDG_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_5);//当看门狗递减到0x40时将会反转该LED
		HAL_WWDG_Refresh(hwwdg);//复位前喂狗
}

```

# 编码器

## 定时器--encoder

```
int count = 0;
HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
int main(){
count = __HAL_TIM_GET_COUNTER(&htim1);
}
```



# C嵌入式常识

## RAM|ROM|FLASH

### RAM

- 易失性存储--断电后数据丢失
- 存储的是程序运行时的数据，如变量、栈和动态分配的内存等
- 写速度快
- 容量小，成本高，功耗高

### FLASH

- 非易失性存储--断电不丢失--存储固件或程序代码
- 读取速度快于RAM的写入速度，适合存储不经常变化的程序代码
- 容量大，成本低，功耗低



# C嵌入式语法

## attribute

GCC编译器的扩展语法，指定变量、函数、类型等的属性。-->优化程序的性能、可移植性

### aligned(n)--指定变量的对齐方式

```
struct __attribute__((aligned(4))) mystruct {
    int a;
    char b;
    short c;
};
```

注:变量的内存地址是4的倍数,内存是以字节为单位进行寻址的，而某些硬件平台要求数据访问必须从特定的地址开始，这个地址通常是某个数值（如2、4、8等）的倍数

处理器访问内存时会一次读取一定数量的数据（称为缓存行或内存块），如果变量的起始地址与缓存行的边界对齐，那么处理器可以一次性加载更多的数据，从而提高性能

### packed--指定结构体或联合体的成员按照1字节对齐

避免不同编译器对结构体内存对齐的差异

```
struct __attribute__((packed)) mystruct {
    int a;
    char b;
    short c;
};
```

### unused-变量 函数未使用，避免警告

```
int myfunc(int a, int b) __attribute_
```

### format--指定函数的参数格式

```
int myprintf(const char *format, ...) __attribute__((format(printf, 1, 2)));
```

- 格式字符串+自定义参数类型
  - `%d`：表示整数。
  - `%f`：表示浮点数。
  - `%s`：表示字符串。
  - `%c`：表示单个字符。
  - `%x`：表示十六进制整数。
  - `%o`：表示八进制整数。
  - `%u`：表示无符号整数。
  - `%e`：表示科学计数法表示的浮点数。
  - `%g`：表示根据值的大小自动选择使用`%f`或`%e`。
  - `%p`：表示指针地址。
  - `%%`：表示一个百分号本身。

### constructor--构造函数 程序启动自执行

```
void myinit() __attribute__((constructor));
```

### destructor--析构函数 程序结束自执行

### deprecated--变量 函数已过时 避免警告

```
[[deprecated("该变量已过时，请使用新的变量替代")]]
int old_variable;

nt __attribute__((deprecated("该变量已过时，请使用新的变量替代"))) my_variable;
```

### noreturn--函数不会返回，避免警告。

```
[[noreturn]] void my_function() {
    // 函数实现
}

void __attribute__((noreturn)) my_function() {
    // 函数实现
}
```

### section(“name”)--指定变量 函数所在段名

```
int __attribute__((section("my_section"))) my_variable;

void __attribute__((section("my_section"))) my_function() {
    // 函数实现
}
```

- 改变变量 函数存储得内存段--操作系统|系统内核编程

### weak--弱--编译链接被忽略

```
void __attribute__((weak)) my_function() {
    // 函数实现
}
```

## #program once

写在head.c文件开始处，跨平台性不好，防重定义

```
#pragma once
```

## extern c

c＋＋的特性支持函数重载，就不会重定向printf了

```
#ifdef __cplusplus
extern "C" {
#endif

xxx自定义函数头文件(C/C++)--同意按C语言函数编译，避免重载，正确链接
C++编译器遇到这段代码时，它会将其中的C++内容视为C语言代码进行编译。而当C编译器遇到这段代码时，它会忽略掉extern "C"部分，只编译C语言部分
//C语言中使用C++库，建议使用C++编译器来编译整个项目，而不是混合使用C和C++编译器。这样可以确保类型兼容性和正确的链接。
#include "c_header.h"

#ifdef __cplusplus
}
#endif

int main() {
    // 现在可以在 C++ 代码中调用 C 语言的函数--不会被C++编译器优化
    c_function();
    return 0;
}
```



## sizeof|strlen

### sizeof--stdlib.h

- 获取任何数据类型或对象的大小--字节数

```
int a; double b; char str[] = "hello";//-----str字节6‘\0’
printf("%zu, %zu, %zu", sizeof(a), sizeof(b), sizeof(str));
int arr[] = {1, 2, 3};
printf("%zu
", sizeof(arr) / sizeof(arr[0]));  // 输出数组的长度
```

- 结构体大小--对齐影响

```
struct example {
    char a;
    int b;
} ex;
printf("%zu
", sizeof(ex));
```

- 指针大小--返回指针本身大小

```
int *ptr = NULL;
printf("%zu
", sizeof(ptr));  // 输出指针的大小，通常是4或8字节  | 32位或64位

```

- 动态分配内存--指针大小

```
int *arr = (int*) malloc(5 * sizeof(int));
printf("%zu
", sizeof(arr));  // 错误！输出的不是分配的内存大小，而是指针的大小
```

- 结构体内存对齐
  - 按照结构体中最大的成员变量的字节数进行对齐
  - 提高内存访问的效率：计算机访问内存时是以字（word）为单位进行的，如果一个数据类型的大小不是字的整数倍，那么就需要额外的填充字节来满足对齐要求

```
struct Example {
    char a;     // 1 byte
    int b;      // 4 bytes
    short c;    // 2 bytes
};
```

编译器在进行内存对齐时会确保整个结构体的起始地址是4的倍数

实际存储结构体时，会在`a`和`c`之间插入3个填充字节，使得整个结构体的大小为8个字节

### strlen--string.h

```
    char a[] = "hello";
    char b[10] = "ok";
    std::cout << "a size :" << strlen(a) <<std::endl;//5
```

## 内存操作

### 分配--stdlib.h

##### malloc

- 堆区分配指定大小的内存块
- 内存区域字节数
  - 分配成功，返回指针；如果失败，返回NULL

```
int *ptr = (int*)malloc(sizeof(int) * 10);
```

##### calloc--stdlib.h

- 分配内存之后将其初始化为0
- 元素数量和每个元素的大小

```
int *ptr = (int*)calloc(10, sizeof(int));
```

##### relloc--stdlib.h

- 调整已分配内存块的大小
- 之前分配的内存指针和新的大小(字节)
  - 成功，realloc返回新的内存地址，否则返回NULL

```
int *temp = (int*)realloc(ptr, sizeof(int) * 20);
```

##### new--iostrem--new

- 动态分配内存，同时初始化对象或数组

  - 对象

    - ```
      Point *p = new Point(3, 4); // 假设Point有一个接受两个参数的构造函数
      ```

  - 数组

    - ```
      int *arr = new int[5]; // 分配一个大小为5的整数数组
      ```

- `<new>`: 声明了 `std::nothrow`，可与 `new` 使用，内存分配失败时返回 `nullptr` 而不是抛出异常

```
#include <iostream>
#include <new>
int main() {
    try {
        int *arr = new int[5];
    } catch (const std::bad_alloc &BA) {
        std::cerr << "Allocation failed: " << BA.what() << '
';
    }
    return 0;
}
```

### 释放

##### free

```
int* ptr = new int; // 分配一个整数
delete ptr;         // 释放内存

int* arr = new int[5]; // 分配一个包含5个整数的数组
delete[] arr;          // 释放数组内存
```

### 智能指针--memory

- **自动管理动态分配内存的工具**
- **通过RAII机制确保在对象离开作用域时释放内存，从而避免内存泄漏**

#### **unique_ptr**

- 同一时刻只有一个智能指针指向给定的动态分配对象。它不支持拷贝赋值，但可以通过移动语义转移所有权

  --不复制底层对象

```
#include <iostream>
#include <memory>
int main() {
    // 创建一个unique_ptr，指向一个动态分配的整数
    std::unique_ptr<int> ptr1(new int(42));

    // 创建另一个unique_ptr，使用移动语义从ptr1获取所有权
    std::unique_ptr<int> ptr2(std::move(ptr1));

    // 现在ptr1不再拥有任何资源，而ptr2拥有原来的资源
    if (ptr1) {
        std::cout << "ptr1 owns the resource" << std::endl;
    } else {
        std::cout << "ptr1 does not own the resource" << std::endl;
    }

    if (ptr2) {
        std::cout << "ptr2 owns the resource" << std::endl;
    } else {
        std::cout << "ptr2 does not own the resource" << std::endl;
    }

    return 0;
}
```

#### **shared_ptr**

- 这种智能指针允许多个指针共享同一个动态分配对象的所有权。

- 它使用引用计数来跟踪有多少个`shared_ptr`指向同一个对象，当最后一个`shared_ptr`被销毁时，它会自动释放对象。

  - 多次引用，多次delete
  - 函数执行完，自定delete，自动管理生命周期

- 创建`shared_ptr`对象：

  ```
  std::shared_ptr<int> ptr(new int(42)); // 创建一个指向整数的shared_ptr
  ```

- 使用`make_shared`函数创建`shared_ptr`对象：

  ```
  auto ptr = std::make_shared<int>(42); // 使用make_shared创建shared_ptr
  ```

- 访问`shared_ptr`所指向的对象：

  ```
  *ptr = 50; // 修改shared_ptr所指向的整数值
  int value = *ptr; // 获取shared_ptr所指向的整数值
  ```

- 拷贝和赋值`shared_ptr`对象：

  ```
  std::shared_ptr<int> ptr1 = std::make_shared<int>(42);
  std::shared_ptr<int> ptr2 = ptr1; // 拷贝构造，增加引用计数
  ```

- 重置`shared_ptr`对象：

  ```
  ptr1.reset(); // 重置ptr1，使其不再指向任何对象
  ```

- 检查`shared_ptr`是否为空：

  ```
  if (!ptr) {
      // shared_ptr为空的处理逻辑
  }  
  ```

- 获取`shared_ptr`的引用计数：

  ```
  std::cout << "Reference count: " << ptr.use_count() << std::endl; // 输出引用计数
  ```

#### **weak_ptr**

这种智能指针被设计为配合`shared_ptr`使用，用来解决循环引用问题，它不会增加引用计数，因此不拥有对象。这允许你检查一个`shared_ptr`是否仍然存在，而不会阻止对象的删除

```
#include <iostream>
#include <memory>

class B; // 前向声明B类

class A {
public:
    std::shared_ptr<B> bPtr;
};

class B {
public:
    std::weak_ptr<A> aWeakPtr;
};

int main() {
    std::shared_ptr<A> a = std::make_shared<A>();
    std::shared_ptr<B> b = std::make_shared<B>();

    a->bPtr = b;
    b->aWeakPtr = a;
    // 此时，A和B之间的循环引用已经被打破，因为B持有一个弱引用而不是强引用。
    // 当A和B都不再被使用时，它们会被正确地释放。
}
```

**实例**

```
#include <memory>

struct Resource {
    Resource() { std::cout << "Resource acquired
"; }
    ~Resource() { std::cout << "Resource destroyed
"; }
};

int main() {
    std::unique_ptr<Resource> res1(new Resource); // unique ownership
    std::shared_ptr<Resource> res2(new Resource); // shared ownership with reference counting
    std::weak_ptr<Resource> weakRes(res2); // weak reference to the shared resource

    if (auto sp = weakRes.lock()) { // Check if the shared pointer still exists
        // Use *sp here
    }

    return 0;
}
```



### 访问--指针运算

#### strcpy--string.h

- 字符串赋值函数，仅字符串

```
char* strcpy(char* dest, const char* src);
```

- 会遇到 '\0'停止

#### memcpy--string.h

- 内存复制函数,任何数据类型

```
void *memcpy(void *dest, const void *src, size_t n)
```

- 不会遇到 '\0'停止 

#### memset--string.h

- 一个字节序列（通常是数组或结构体）的内容设置为指定的**字符**--unsigned char
  - 返回一个指向填充后内存区域的指针

- 快速地初始化或清除内存区域,或者在不再需要时清除包含敏感数据的内存区域
- 注意其类型转换的行为
  - `memset` 来初始化非字符类型的数组（如 `int` 或 `float` 数组）:类型长度不匹配而导致结果不符合预期
    - `memset` 来初始化一个整数数组，传入的值 `'A'` 会被转换为其ASCII值（即65），然后将每个字节都设为这个值
  - 快
  - 地址 + 字符 + 字节数


```
#include <stdio.h>
#include <string.h>
int main() {
    char str[50];
    // 使用 memset 将 str 的前 10 个字节设置为 'A'
    memset(str, 'A', 10);
    str[10] = '\0'; // 确保字符串以 null 结尾
    printf("After memset, string is: %s
", str);
    return 0;
}
```

## 指针

### 函数名

- 该函数的首地址

### 普通数组

```
struct Student {
  char id[20];
  char name[10];
  int age;
  float score;
};
struct Student class1[20]; // 定义一个包含20个Student结构体的数组
struct Student class1[2] = {{"20242356", "小敏", 18, 88.9}, {"20242345", "小鹤", 19, 85.9}};

int var[] = {10, 20, 30};
int *ptrArray[3];
```

### 函数指针数组

- 元素是函数的指针，元素都指向一个函数，定义需要**指定函数返回类型**和**参数类型**
- 类似策略模式的设计模式，不同的情况调用不同的函数

```
void (*funcArray[3])(void) = {func1, func2, func3};

void (*sortingAlgorithms[2])(int arr[], int len) = {bubbleSort, quickSort};
```

### 指针函数数组

- ##### 返回值为指针的函数

- 动态内存分配、数据结构如链表、树等的构建过程中

- 通过返回指针，实现对同一块内存的多次操作，从而提高效率

```
int *getIntPointer() {
   static int i = 10;
   return &i;
}
```

## 预编译

```
#define PI 3.14159
#include <stdio.h>
#undef PI
```

### **#undef**

- 取消宏定义，删除先前定义的宏

### #if

- 决定是否编译某段代码

```
#ifdef、#ifndef、#else、#elif、#endif
```

### **#pragma**

- 向编译器发送特定于实现的指令

```
#pragma message("This is a compiler-specific message")
```

### #line

- 改变编译器报告的行号

```
#line 100 "new_file.c"
```

### **#**

- 用于将宏参数转换为字符串

```
#define STRINGIFY(x) #x
printf("%s
```



## 重定义

- 头文件（.h）中

```
#incldue <xxx.h> //头文件包含
typedef struct xxx{}xxx; //重命名
#ifdef __CPLUS_PLUS //条件编译
#endif
void xxx(void); //函数声明
```

注：不可加变量定义和变量初始化--变量原型不能在头文件中

若加，则（.c）中

```
xTaskHandle PID_Handler;	//任务句柄
```

（.h）中

```
extern xTaskHandle PID_Handler;	//任务句柄
```

## 函数无实现

- 将头文件加入工程--嵌入到自己工程--添加编译源码

```
Add Existing file to project
```

- C/C++  头文件路径

## 位操作

### 某位变1,其余不变

```
uint8_t flag = 0;
switch(n){
	case 0x201:
		if(xx)flag |= 0x01;  //
		break;
	default:break;
}
```

### 某位变1，其余归零

```
uint8_t flag = 0;
switch(n){
	case 0x201:
		if(xx){flag &= 0x01;flah |= 0x01;}  
		break;
	default:break;
}
```

### 检验某位，其余不变

```
uint8_t flag = 0;
switch(n){
	case 0x201:
		if(flag & 0x01){}  
		break;
	default:break;
}
```

### 记录1个数

```
int count_ones(unsigned char byte) {
    int count = 0;
    for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
            count++;
        }
    }
    return count;
}
```

## 不同类型变量前缀

- unsigned char byte = 0b10101010; *// 二进制（Binary）表示的字节*  0b/0B
- unsigned char byte = 0172; //八进制（Octal）：0
- unsigned char byte = 172;//十进制（Decimal）
- unsigned char byte = 0xAC;//十六进制（Hexadecimal）

## 流操作

- 标准库函数`printf()`
  - 输出流
  - 字符串（含格式占位符）
  - 格式字符串

### stderr--错误信息输出流

```
#include <stdio.h>
int main() {
    // 假设发生了一个错误
    int error_code = 1;
    if (error_code != 0) {
        fprintf(stderr, "发生错误： %d\n", error_code);
        return 1; // 返回非零值表示程序异常退出
    }
    // 正常执行的代码...
    return 0; // 返回零表示程序正常退出
}

```

- 定义了一个错误码`error_code`
- `fprintf()`函数将错误信息打印到标准错误输出（`stderr`）
- 返回相应的退出状态码

### stdout--标准输出流

### sprintf--格式化输出

```
char messade[20] = "";
sprintf(message, "count: %d",count);
```



## 预定义宏

### 构建指令

```
target_compile_definitions(target
PRIVATE
-Dtrue=(__LINE__%2==0);
)
```

### 宏判断

```
#define IS_TRUE(condition) （condition）

if(IS_TRUE(condition)){
}
```

# CMake+Vscode开发

## 编译优化

防止编译优化--正确断点--保持结构

```
# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O0")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -O0")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O0")
```

高代码执行效率和减少生成的二进制文件大小的过程：去除未使用的代码、重排指令以提高效率、函数内联等等



# txt

```

```

