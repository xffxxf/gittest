/*
* Copyright (c) 2017.04，华乘电气科技(股份)有限公司
* All rights reserved.
*
* 文件名称：dft.c
* 
* 初始版本：V1.0.0.0

* 作者：李遥

* 创建日期：2017/05/17

* 摘要：离散傅里叶变换
* 
*/

#include "dft.h"

#ifndef PI
# define PI	3.14159265358979323846264338327950288
#endif

/************************************************
 * 输入参数：float *pafInBuf:  原始数据缓存
  			 int iLen:	   	   原始数据长度
  			 int iWantedNode:  需要计算的节点

 * 返回值   ：succeed: 当前节点对应的值; negative:错误码

 * 功能     :用于计算当前节点对应的离散傅里叶变换结果
 ************************************************/
float dft_calc( float *pafInputBuf, int iLen, int iWantedNode )
{
	complex stOutput;
	int iIndex = 0;
	float fResult;

	stOutput.real = stOutput.imag = 0.0;

	for( iIndex = 0; iIndex < iLen; iIndex++ )
	{
		stOutput.real += pafInputBuf[iIndex] * cos( 2 * PI * iWantedNode * iIndex / iLen );
		stOutput.imag += -1 * pafInputBuf[iIndex] * sin( 2 * PI * iWantedNode * iIndex / iLen );
	}

	stOutput.real = stOutput.real * 2 / iLen;
	stOutput.imag = stOutput.imag * 2 / iLen;

	fResult = sqrt( stOutput.real * stOutput.real + stOutput.imag * stOutput.imag );

	return fResult;
}
