#include "../Inc/Serial_Analysis.h"


// 数据帧打包函数
size_t CreateFrame(uint16_t functionCode, const uint8_t* data, size_t dataLength, Frame* frame) {
    if (dataLength > MAX_DATA_LENGTH) {
        return 0; // 数据长度超出最大限制
    }

    size_t index = 0;

    // 添加帧头
    frame->frame[index++] = FRAME_START;

    // 添加功能码（2字节）
    frame->frame[index++] = (functionCode >> 8) & 0xFF; // 功能码高字节
    frame->frame[index++] = functionCode & 0xFF;        // 功能码低字节

    // 添加数据长度
    frame->frame[index++] = (uint8_t)dataLength;

    // 添加数据区
    for (size_t i = 0; i < dataLength; i++) {
        frame->frame[index++] = data[i];
    }

    // 计算CRC16校验和（从功能码到数据区的部分进行校验）
    uint16_t crc16 = Get_CRC16_Check_Sum(&frame->frame[1], 3 + dataLength, 0xFFFF);

    // 添加CRC16校验码
    frame->frame[index++] = crc16 & 0xFF;         // CRC16低字节
    frame->frame[index++] = (crc16 >> 8) & 0xFF;  // CRC16高字节

    // 添加帧尾
    frame->frame[index++] = FRAME_END;

    // 设置帧的实际长度
    frame->length = index;

    return frame->length; // 返回打包后的帧长度
}

// 数据帧解包函数
bool ParseFrame(const Frame* frame) {
    // 检查帧长度
    if (frame->length < 5) { // 需要最少5个字节来包含帧头、功能码、数据长度和帧尾
        printf("帧长度不足！\n");
        return false;
    }

    // 检查帧头和帧尾
    if (frame->frame[0] != FRAME_START || frame->frame[frame->length - 1] != FRAME_END) {
        printf("帧头或帧尾错误！\n");
        return false;
    }

    // 提取功能码
    uint16_t functionCode = (frame->frame[1] << 8) | frame->frame[2];
    printf("功能码: 0x%04X\n", functionCode);

    // 提取数据长度
    size_t dataLength = frame->frame[3];
    printf("数据长度: %d\n", dataLength);

    // 检查数据长度是否有效
    if (frame->length < 5 + dataLength + 2) { // 5是帧头和功能码、数据长度，2是CRC16
        printf("数据长度无效！\n");
        return false;
    }

    // 提取数据区
    printf("数据内容: ");
    for (size_t i = 0; i < dataLength; i++) {
        printf("0x%02X ", frame->frame[4 + i]);
    }
    printf("\n");

    // 提取接收到的CRC16校验码
    uint16_t receivedCRC = (frame->frame[4 + dataLength + 1] << 8) | frame->frame[4 + dataLength];
    printf("接收到的CRC16校验码: 0x%04X\n", receivedCRC);

    // 计算CRC16校验和（从功能码到数据区）
    uint16_t calculatedCRC = Get_CRC16_Check_Sum(&frame->frame[1], 3 + dataLength, 0xFFFF);
    printf("计算得到的CRC16校验码: 0x%04X\n", calculatedCRC);

    // 校验CRC16
    if (receivedCRC == calculatedCRC) {
        printf("数据帧校验成功！\n");
        return true;
    } else {
        printf("数据帧校验失败！\n");
        return false;
    }
}
