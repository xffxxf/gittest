/*
* Copyright (c) 2017.04，华乘电气科技(股份)有限公司
* All rights reserved.
*
* 文件名称：dft.h
* 
* 初始版本：V1.0.0.0

* 作者：李遥

* 创建日期：2017/05/17

* 摘要：离散傅里叶变换
* 
*/
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#ifndef DFT_H_
#define DFT_H_

//自定义复数类型
typedef struct _complex
{
	float real;
	float imag;
}complex;

#define MALLOC_ERROR     (-1)

/************************************************
 * 输入参数：float *pafInBuf:  原始数据缓存
  			 int iLen:	   	   原始数据长度
  			 int iWantedNode:  需要计算的节点

 * 返回值   ：succeed: 当前节点对应的值; negative:错误码

 * 功能     :用于计算当前节点对应的离散傅里叶变换结果
 ************************************************/
float dft_calc( float *pafInputBuf, int iLen, int iWantedNode );

#endif