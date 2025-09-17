/*
* Copyright (c) 2015.11，南京华乘电气科技有限公司
* All rights reserved.
*
* 文件名称：CRC8.h
*
*
* 初始版本：1.0
* 作者：吴昌盛
* 创建日期：2016年1月13日
* 摘要：检查CRC8校验
*
* 
*/

#ifndef _CRC8_H_
#define _CRC8_H_

#include "DataDefine.h"

//添加CRC校验
void  add_crc8(UINT8* pucData, UINT32 uiDataLen);

//检查CRC校验 返回检查结果，输入包含CRC8的数组
INT32 check_crc8(UINT8* pucData, UINT32 uiDataLen);

//计算CRC8值 返回CRC8校验码
UINT8 cal_crc8(UINT8* pucData, UINT32 uiDataLen);
/************************************************
 * 函数名   ：cal_crc16

 * 输入参数 ：UINT8 *pucData输入报文  UINT32 uiDataLen 报文长度

 * 输出参数 ：

 * 返回值   ：0:HC_SUCCESS; -1:HC_FAILURE

 * 功能     ：计算CRC16检验码

 * 描述     : 查表法计算CRC16检验码
 ************************************************/
UINT16 cal_crc16(UINT8* pucData, UINT32 uiDataLen);
/*************************************************
函数名： cal_crc32
输入参数： pucData：待计算数据缓冲
          uiCount：缓冲长度
输出参数： NULL
返回值： 32位CRC值
功能： 计算数据的CRC
*************************************************************/
UINT32 cal_crc32(const UINT8 *pucData, UINT32 uiCount);

#endif

