# stm32_roslib库

 基本描述：**stm32_roslib**是[rosserial](https://github.com/ros-drivers/rosserial)在stm32上运行的库。它本身包括了硬件通讯部分、ROS基本功能部分以及官方消息库，同时我将部分常用的第三方消息库整合进来。该库将stm32控制器作为一个节点接入ROS，使机器人控制更加高效、灵活。

该库配合PC端的**rosserial**一起使用，已经将其整合为一个功能包[rosserial_self_launch](https://www.kdrobot.top/git/ROS/rosserial_self_launch)，直接使用启动文件即可

由于官方原始库的`STM32Hardware.h`有错误，本库已经解决了相关问题，因此直接使用本库即可，无需参考官方教程生成**stm32_roslib**库

---

## 1.安装库及配置

### 1.1 下载库

首先将该库克隆。克隆完成后，将该库放入需要使用`rosserial`的工程目录下，例如工程目录为`hello_rosserial`则放入该目录下

<img src="http://m.qpic.cn/psc?/V53QFqXB1uOKk40V3XHe2aEJLv2ByRIm/bqQfVz5yrrGYSXMvKr.cqVeokxP*ZcSGiPIi0LvlWI7YOoVZ0.sRa9DWmqsuSO7g0Xoy2oLaYQAGb8pm6GOp6BWanlfTfHorKDcBrWiSDb0!/b&bo=mgZHBJoGRwQDByI!&rf=viewer_4" style="zoom:50%;" />

### 1.2 配置STM32CUBEMX

由于stm32与ROS(PC端)需要硬件通讯，因此我们需要指定一个串口作为与上位机进行通讯。

打开配置文件。由于在**stm32_roslib**库中的`STM32Hardware.h`默认指定USART2作为通讯串口，这里我也指定了`USART2`作为与上位机通讯串口，**波特率**设置为`115200`。

> 这里波特率和指定串口都可以根据实际情况更改。其中，若波特率改变，PC端的rosserial启动文件将波特率指定为与STM32相同的值。若指定串口更改了，则需要更改**stm32_roslib**库中的`STM32Hardware.h`指定串口，修改方法下文会具体说。

在硬件通讯实现中，使用了UART_DMA方式进行数据接发。因此将串口的DMA接收发送开启（默认设置即可），同时打开串口中断和DMA中断。

<img src="http://m.qpic.cn/psc?/V53QFqXB1uOKk40V3XHe2aEJLv2ByRIm/bqQfVz5yrrGYSXMvKr.cqRmV2613VbYzQSckytzg8Unr.3B7RMW5..U*iTEswD6BOuXPq.EdRU*198ljNAv.e9VD*xVOTohxW4f7S4X7nYY!/b&bo=hwOKAYcDigEDFzI!&rf=viewer_4" style="zoom:50%;" />

<img src="http://m.qpic.cn/psc?/V53QFqXB1uOKk40V3XHe2aEJLv2ByRIm/bqQfVz5yrrGYSXMvKr.cqbJZJwwHvvDZs*ZnOUXw7wlvgHgqGz5ovxZZ764GImmzwnnpWYyiGkYET4p9a89Y17GIImoYYezuO6bfKyb5lcE!/b&bo=iAOiAYgDogEDFzI!&rf=viewer_4" style="zoom:50%;" />

**至此配置CUBE完成，生成代码。**

### 1.3 配置Keil工程

打开keil工程，首先需要更改编译器为AC6，以及将c/c++标准更改为c99和c++11。

修改流程之前的文档已经写过了https://www.kdrobot.top/git/luoyangcheng/solve_keil_AC6_FreeRTOS_problem

> **如果没有使用FreeRTOS，只需要看修改编译器部分即可**

修改为AC6后，点击**魔法棒**，添加`stm32_roslib`库头文件路径。

<img src="http://m.qpic.cn/psc?/V53QFqXB1uOKk40V3XHe2aEJLv2ByRIm/bqQfVz5yrrGYSXMvKr.cqQnfDT5RhrsnaYmSW2gJZePo5KB*qNlNc4pMw5xAIJvvLWk*j.8RNR.6oFZgjRa5I4qJGHFPFic9vCGXUWU9xLY!/b&bo=UAJNAFACTQADFzI!&rf=viewer_4" style="zoom:100%;" />



<img src="http://m.qpic.cn/psc?/V53QFqXB1uOKk40V3XHe2aEJLv2ByRIm/bqQfVz5yrrGYSXMvKr.cqe*Ach9q.fNg4DxO0SWLTKgu6UGcWWgdRGe9CDAUkJjwmtI0zsA1*KXbsqqg7fQzlXoqx6d7cVED7ugaAlBkPis!/b&bo=cQLVAXEC1QEDFzI!&rf=viewer_4" style="zoom:100%;" />

<img src="http://m.qpic.cn/psc?/V53QFqXB1uOKk40V3XHe2aEJLv2ByRIm/bqQfVz5yrrGYSXMvKr.cqULVwByQ3tHHEwSP6YxqP68kouQgt.p7oLWW9NBR4OE.dl*7UXKzjbtlW8KuVCbNO1KTAHX5TrGZp8RRGwE8c80!/b&bo=*QFWAf0BVgEDFzI!&rf=viewer_4" style="zoom:100%;" />

点击OK之后，接着在`C/C++`中的`Define`编译定义中添加`,__USE_C99_MATH`，保存即可

> 由于stm32_roslib中使用到math.h，不添加会报错

<img src="http://m.qpic.cn/psc?/V53QFqXB1uOKk40V3XHe2aEJLv2ByRIm/bqQfVz5yrrGYSXMvKr.cqRglPpgLK0ikCbqdcWSNs0FtH6ZLAfUF5sYIWq9xOwlQl0GSeG2IkDrb3Y6h0EBerCbEOq5ymrwD*iNPhHKK8B0!/b&bo=cAJ2AHACdgADFzI!&rf=viewer_4" style="zoom:100%;" />

在工程组别管理中添加`stm32_roslib`源文件，文件在stm32_roslib文件夹下的Inc中，把另两个头文件也添加了。共四个文件(duration.cpp、ros.h、STM32Hardware.h、time.cpp)

<img src="http://m.qpic.cn/psc?/V53QFqXB1uOKk40V3XHe2aEJLv2ByRIm/bqQfVz5yrrGYSXMvKr.cqSnh.zlesagFb.u5nlowTjYqgBejuhdOO0PHNxyxe25pwxbDJi3mJ6mN16i6veq08ThcAojDRrMoP*KRz5FPazY!/b&bo=SgI8AEoCPAADByI!&rf=viewer_4" style="zoom:75%;" />

<img src="http://m.qpic.cn/psc?/V53QFqXB1uOKk40V3XHe2aEJLv2ByRIm/bqQfVz5yrrGYSXMvKr.cqZQSpcNsRIds8fBJ5u*Aicsz5WxCS.9jrGtLTcZo1KlO1qorkjSYADrIgWX2QA6X*CZFZTaq.vnz4Dt*9mJCXNE!/b&bo=dALVAXQC1QEDFzI!&rf=viewer_4" style="zoom:75%;" />

至此配置完成，然后编译即可。

## 2.使用

### 2.1 注意事项

这里说明一下使用的注意事项，由于stm32_roslib使用了C++来实现，编译器编译时采用C C++混合编译。

需要注意之后我们自己写的头文件被C++源文件包含时需要添加以下指令，否则头文件代码会被当做C语言来编译。

```c++
#ifdef __cplusplus
extern "C" {
#endif

...(自己写的代码放在这个'{'和'}'中间)

#ifdef __cplusplus
}
#endif
```

**!!!这里我建议接入stm32_roslib库后，以后所有自己写的头文件都添加类似如下指令，这样保证编译不会出错。**

例如我自己写的头文件代码`my_core.h`：

```c++
#ifndef __MY_CORE_H_
#define __MY_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif

void setup(void);
void loop(void);	

#ifdef __cplusplus
}
#endif

#endif
```

c++源文件为`my_core.cpp`,头文件包含如下

> **c++源文件后缀不用.c，而用.cpp**。因为这样做源文件后缀为cpp会被自动被keil识别为c++源文件，这样编译时keil会自动将源文件添加`__cplusplus`定义，编译不会出错

```c++
#include "my_core.h"
#include "main.h"
#include "ros.h"

...
```

## 2.2 STM32Hardware.h文件配置

**在通讯指定串口为USART2以及不想修改接收发送缓冲长度的情况下，不需要对`STM32Hardware.h`文件做任何修改。**

[STM32Hardware.h](./Inc/STM32Hardware.h)文件功能：在stm32_roslib中，如果非要修改的也就是这个文件。这个文件规定了最基础也时最重要的部分，stm32与上位机通讯。

通常情况下，我们只需要修改指定串口，若是使用USART3则将`huart2`替换成`huart3`即可。

```c++
extern UART_HandleTypeDef huart2;
UART_HandleTypeDef *_huart = &huart2;
```



### 2.3 接收部分代码

必须有硬件接发数据才能保证stm32_roslib正常工作，硬件接发数据部分在`STM32Hardware.h`。



### 2.4 节点初始化以及回调

与PC端ROS类似，都是需要初始化节点，创建节点句柄以及调用回旋，不过stm32_roslib中只有spinOnce()，没有spin()

（毕竟没有多核处理器，也没有线程概念O(∩_∩)O））

```c++
// 节点句柄
ros::NodeHandle nh;

/* 初始化 */
void setup(void)
{
	// 初始化节点
	nh.initNode();
    ...
}
...
void loop(void)
{
    nh.spinOnce();	//回旋
    /*	
    	调用回旋时时间说明：
    	用FreeRTOS的话换成FreeRTOS相关的时间函数。	
    	//TODO
    */
	HAL_Delay(1);	
}
...
```



### 2.5 生成自定义消息的`rosserial`头文件，并将其载入stm32_roslib中

在平常使用中，官方消息库以及第三方消息库，一般不能满足我们使用实际使用需要，例如上位机给下位传递一些指令时，需要使用自定义的消息。

`rosserial_client`支持生成自定义消息的`rossserial`头文件。

**这一点在ROS终端中完成。**

示例做法如下：

**确保在ROS中，你的自己工作空间下已经编译生成了自定义消息。**

例如我的自定义消息为`test.msg`，位于功能包`rosserial_self_msg`下

然后将工作空间`source`，使工作空间的包添加进ROS包工作环境中。

```bash
source devel/setup.bash
```

接着指定一个文件夹在其目录下生成当前ROS环境下所有的消息

```bash
rosrun rosserial_client make_libraries <生成的文件夹目录>
```

例如我指定了`~/temp_roslib`，那么就写为

```bash
rosrun rosserial_client make_libraries ~/temp_roslib
```

那么在`~/temp_roslib/roslib`下就可以找到功能包消息头文件的文件夹，这里我的为`rosserial_self_msg`，将其拷贝到**STM32**工程目录的`stm32_roslib/Inc`下。

这样自定义消息头文件在`stm32_roslib`中就添加完成了



要使用时，包含头文件即可

```c++
#include "rosserial_self_msg/test.h"
```

这样在**STM32**中可以使用自己编写的自定义消息

---

*这里只介绍了重要的内容，具体使用可以使用看源代码和范例。*

## 3.使用教程范例

[ROS/stm32_roslib_tutorial - stm32_roslib_tutorial - A&T机器人队 (kdrobot.top)](https://www.kdrobot.top/git/ROS/stm32_roslib_tutorial)
