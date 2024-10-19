/**
 * @file CRC.h
 * @author yao
 * @date 2021年1月13日
 * @brief C实现的CRC校验
 */

#ifndef __CRC_H_
#define __CRC_H_

#include <stdint.h>
#include <stddef.h>

/**
 * @brief CRC16校验和函数
 * @param pchMessage 要校验的数据
 * @param dwLength 数据流的长度
 * @param wCRC 初始化的CRC16校验和
 * @return 计算得到的CRC16校验和
 */
uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, size_t dwLength, uint16_t wCRC);

/**
 * @brief CRC16校验函数
 * @param pchMessage 要校验的数据
 * @param dwLength 数据流的长度 = 数据 + 校验和
 * @return 校验结果
 */
uint16_t Verify_CRC16_Check_Sum(const uint8_t *pchMessage, size_t dwLength);

#endif // __CRC_H_