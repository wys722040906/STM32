#ifndef __SERIAL_ANALYSIS_H
#define __SERIAL_ANALYSIS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "./CRC.h" 

#define MAX_DATA_LENGTH 249  // 定义最大数据长度

// 常量定义 -- const 多文件包含 每个文件生成一个实例 -- 建议宏
// const uint8_t FRAME_START = 0xAA;
// const uint8_t FRAME_END = 0x55;
#define FRAME_START 0xAA
#define FRAME_END 0x55



// 数据帧结构
typedef struct {
    uint8_t frame[MAX_DATA_LENGTH + 7]; // 帧的最大长度为数据长度 + 7
    size_t length;                       // 当前帧的实际长度
} Frame;

// 数据帧打包函数
/*
uint16_t functionCode: 功能码，通常用于标识操作的类型。
const uint8_t* data: 指向数据的指针，包含需要打包的数据。
size_t dataLength: 数据的长度，表示 data 中有效数据的字节数。
Frame* frame: 指向 Frame 结构的指针，保存打包后的帧。
*/
size_t CreateFrame(uint16_t functionCode, const uint8_t* data, size_t dataLength, Frame* frame);

// 数据帧解包函数
bool ParseFrame(const Frame* frame);

#endif // __SERIAL_ANALYSIS_H
