/*
 * Copyright (c) 2017.04，华乘电气科技(股份)有限公司
 * All rights reserved.
 *
 * 文件名称：ae_view_convert.h
 * 
 * 初始版本：V1.0.0.0

 * 作者：李遥

 * 创建日期：2018/03/30

 * 摘要：增加对AE四大图谱的数据处理
 * 
 */

/*
 * Copyright (c) 2017.04，华乘电气科技(股份)有限公司
 * All rights reserved.
 *
 * 文件名称：DataConvert.h
 * 
 * 初始版本：V1.0.0.0

 * 作者：李遥

 * 创建日期：2017/04/17

 * 摘要：增加对AE四大图谱的数据处理
 * 
 */

#ifndef __AE_VIEW_CONVERT_H
#define __AE_VIEW_CONVERT_H

#include "UHFHFCTAETEVApi.h"
#include "UHFHFCTAETEV.h"


#define 	PI 								(3.14159265358979323846264338327950288)//π
#define     GRID_FREQUENCY                  (50)//工频周期

/*  AE返回值类型  */
typedef enum _AEStatus
{
    SUCCEED = 0, //成功
    AE_INPUT_ERROR = -1, //输入错误
    AE_CALC_ERROR = -2, //计算错误
    AE_MALLOC_ERROR = -3, //malloc错误
    AE_PULSE_MAX_ERROR = -4,//AE脉冲达到最大
    AE_FAILED = -5,//综合错误
} AEStatus;

/************************************************
 * 功能    : 将原始数据转成ae四图谱格式数据,并填入输出缓存
 * 
 * 描述    : 使用前需填写pstAE中的输入参数( important!!! )
 * 
 * 输入参数：
 *          pfRaw -- 原始数据
 *          usLen -- 原始数据长度
 *          pstParam -- 参数
 *          pstAE -- AE多图谱参数及数据缓存
 * 输出参数:
 *          pstAE -- AE多图谱参数及数据缓存
 * 
 * 返回值  ：succeed:0; fail:negative
 ************************************************/
AEStatus ae_raw_to_multi_views( float *pfRaw, UINT16 usLen, AEWorkSetForUp* pstParam, AEAllView* pstAE );


#endif//__AE_VIEW_CONVERT_H
