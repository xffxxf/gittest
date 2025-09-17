/*
 * Copyright (c) 2017.04，华乘电气科技(股份)有限公司
 * All rights reserved.
 *
 * 文件名称：ae_view_convert.c
 * 
 * 初始版本：V1.0.0.0

 * 作者：李遥

 * 创建日期：2018/03/30

 * 摘要：增加对AE四大图谱的数据处理
 * 
 */

#include "ae_view_convert.h"
#include "dft.h"

/************************************************
 * 输入参数：
 *          pfRaw -- 原始数据
 *          usLen -- 原始数据长度
 * 返回值: 
 *          原始数据中的最大值
 ************************************************/
static float get_max_value(float *pfRaw, UINT16 usLen)
{
    int iIndex = 0;
    float fMaxValue = 0;

    if (pfRaw == NULL)
    {
        UHFHFCTAETEV_dbg("error in get_max_value: input error!\r\n");
        return 0;
    }

    /*找到最大值*/
    for (iIndex = 0; iIndex < usLen; iIndex++)
    {
        fMaxValue = (fMaxValue > pfRaw[iIndex]) ? fMaxValue : pfRaw[iIndex];
    }

    return fMaxValue;
}

/************************************************
 * 输入参数：
 *          pfRaw -- 原始数据
 *          usLen -- 原始数据长度
 * 返回值: 
 *          原始数据的有效值
 ************************************************/
static float get_rms(float *pfRaw, UINT16 usLen)
{
    int iIndex = 0;
    double dSum = 0;
    float fRms = 0;

    /*找到最大值*/
    for (iIndex = 0; iIndex< usLen; iIndex++)
    {
        dSum += pfRaw[iIndex] * pfRaw[iIndex];
    }

    fRms = sqrt(dSum / usLen);

    return fRms;
}

/***********************************************************
 输入参数:iAdcValue  mV值
 输出参数：无
 返回值：无
 功能：   转换成dB值
 伪代码：无
 算法：无
 *************************************************************/
int adc_driver_value2db(UINT32 iAdcValue)
{
    if (iAdcValue)
    {
        return (int) (20 * log10(((double) iAdcValue) / 100));
    }
    else if(iAdcValue == 0)
    {
        return -15;
    }
    else
    {
        return 0;
    }
}

#define AE_RANGE_60DB_MV_MIN 0
#define AE_RANGE_60DB_MV_MAX 1000
#define AE_RANGE_80DB_MV_MIN 0
#define AE_RANGE_80DB_MV_MAX 200
#define AE_RANGE_100DB_MV_MIN 0
#define AE_RANGE_100DB_MV_MAX 20

#define AE_RANGE_60DB_DB_MIN 10
#define AE_RANGE_60DB_DB_MAX 70
#define AE_RANGE_80DB_DB_MIN 0
#define AE_RANGE_80DB_DB_MAX 50
#define AE_RANGE_100DB_DB_MIN -15
#define AE_RANGE_100DB_DB_MAX 30

/***********************************************************
 函数名：static s32 ae_driver_value_range
 输入参数:
 ucUnit     单位
 fGain      增益
 fValue    比较值
 输出参数：无
 返回值：无
 功能： AE值范围限制
 伪代码：无
 算法：无
 *************************************************************/
float ae_driver_value_range(UINT8 ucUnit, float fGain, float fValue)
{
    float fResult;

    if (ucUnit == dB)   //DB模式
    {
        if (fGain == AE_GAIN60)
        {
            fResult =
                    fValue < AE_RANGE_60DB_DB_MAX ?
                            fValue : AE_RANGE_60DB_DB_MAX;
            fResult =
                    fValue > AE_RANGE_60DB_DB_MIN ?
                            fValue : AE_RANGE_60DB_DB_MIN;
        }
        else if (fGain == AE_GAIN80)
        {
            fResult =
                    fValue < AE_RANGE_80DB_DB_MAX ?
                            fValue : AE_RANGE_80DB_DB_MAX;
            fResult =
                    fValue > AE_RANGE_80DB_DB_MIN ?
                            fValue : AE_RANGE_80DB_DB_MIN;
        }
        else if (fGain == AE_GAIN100)
        {
            fResult =
                    fValue < AE_RANGE_100DB_DB_MAX ?
                            fValue : AE_RANGE_100DB_DB_MAX;
            fResult =
                    fValue > AE_RANGE_100DB_DB_MIN ?
                            fValue : AE_RANGE_100DB_DB_MIN;
        }
    }

    else
    {
        if (fGain == AE_GAIN60)
        {
            fResult =
                    fValue < AE_RANGE_60DB_MV_MAX ?
                            fValue : AE_RANGE_60DB_MV_MAX;
            fResult =
                    fValue > AE_RANGE_60DB_MV_MIN ?
                            fValue : AE_RANGE_60DB_MV_MIN;
        }
        else if (fGain == AE_GAIN80)
        {
            fResult =
                    fValue < AE_RANGE_80DB_MV_MAX ?
                            fValue : AE_RANGE_80DB_MV_MAX;
            fResult =
                    fValue > AE_RANGE_80DB_MV_MIN ?
                            fValue : AE_RANGE_80DB_MV_MIN;
        }
        else if (fGain == AE_GAIN100)
        {
            fResult =
                    fValue < AE_RANGE_100DB_MV_MAX ?
                            fValue : AE_RANGE_100DB_MV_MAX;
            fResult =
                    fValue > AE_RANGE_100DB_MV_MIN ?
                            fValue : AE_RANGE_100DB_MV_MIN;
        }
    }

    return fResult;
}

/************************************************************
 * 功能:    根据单位转换ae幅值数据
 * 说明:    
 *          1.将数据转为mv
 *          2.如果单位为db,则以mv值为基础转为db
 *          3.对数据限制范围
 * 输入参数:
 *          pstAE -- AE幅值数据(原始值)
 *          ucUnit -- 单位
 *          fGain -- 增益
 * 返回值:
 *          SUCCEED : 成功; else :失败
 *************************************************************/
static AEStatus convert_amp_value( AEAmpData *pstAE, UINT8 ucUnit, float fGain )
{
    if( NULL == pstAE )
    {
        UHFHFCTAETEV_dbg("error in [%s]: pstAE = null\r\n",__func__);
        return AE_INPUT_ERROR;
    }

    //根据单位做转换
    if( dB == ucUnit )
    {
        pstAE->fPeakValue = (float)adc_driver_value2db( (UINT32)pstAE->fPeakValue * 100 );
        pstAE->fRMS = (float)adc_driver_value2db( (UINT32)pstAE->fRMS * 100 );
        pstAE->fH1 = (float)adc_driver_value2db( (UINT32)pstAE->fH1 * 100 );
        pstAE->fH2 = (float)adc_driver_value2db( (UINT32)pstAE->fH2 * 100 );

        //规避增益为100时 傅里叶变换时频率成分为0的情况
        if(( pstAE->fRMS < 0 )&&( fabs(fGain - AE_GAIN100) < 0.00001 ))
        {
            pstAE->fH1 = ( fabs( pstAE->fH1 ) < 0.00001 ) ? -15 : pstAE->fH1;
            pstAE->fH2 = ( fabs( pstAE->fH2 ) < 0.00001 ) ? -15 : pstAE->fH2;
        }
    }

    //限制数据范围
    pstAE->fPeakValue = ae_driver_value_range( ucUnit, fGain,  (int)pstAE->fPeakValue );
    pstAE->fRMS = ae_driver_value_range( ucUnit, fGain,  (int)pstAE->fRMS );
    pstAE->fH1 = ae_driver_value_range( ucUnit, fGain,  (int)pstAE->fH1 );
    pstAE->fH2 = ae_driver_value_range( ucUnit, fGain,  (int)pstAE->fH2 );

    return SUCCEED;
}

/************************************************
 * 功能    : 将ad原始数据转换成wave图谱对应的格式
 * 
 * 描述    : 使用前需要填写@pstAE中的参数部分
 * 
 * 输入参数： 
 *          pfRaw -- 原始数据
 *          usLen  -- 原始数据长度
 * 
 * 输出参数：
 *          stAEMultiView -- ae多图谱数据类型
 * 
 * 返回值  ：succeed:0; fail:negative
 ************************************************/
AEStatus raw_convert_to_wave( float *pfRaw, UINT16 usLen, AEAllView* pstAE )
{
    if(( NULL == pfRaw )||( NULL == pstAE ))
    {
        UHFHFCTAETEV_dbg("error in [%s]: input = null!\r\n", __func__);
        return AE_INPUT_ERROR;
    }    

    UINT16 usCopyLen = usLen > MAX_WAVE_POINT_PER_MSG ? MAX_WAVE_POINT_PER_MSG * sizeof(float) : usLen * sizeof(float);

    memcpy( &pstAE->faWaveData, pfRaw, usCopyLen );

    pstAE->usWavePointNum = usCopyLen / sizeof(float);

    return SUCCEED;
}

/************************************************
 * 功能    : 将ad原始数据转换成amplitude图谱对应的格式
 * 
 * 描述    : 使用前需要填写@pstAE中的参数部分
 * 
 * 输入参数： 
 *          pusRaw -- 原始数据
 *          usLen  -- 原始数据长度
 *          iFreq -- 系统频率
 * 
 * 输出参数：
 *          stAEMultiView -- ae多图谱数据类型
 * 
 * 返回值  ：succeed:0; fail:negative
 ************************************************/
AEStatus raw_convert_to_amplitude( float *pfRaw, UINT16 usLen, UINT16 usFreq, AEAllView* pstAE )
{
    UINT16 usDFTFreq = 0;

    if(( NULL == pfRaw )||( NULL == pstAE ))
    {
        UHFHFCTAETEV_dbg("error in [%s]: input = null!\r\n", __func__);
        return AE_INPUT_ERROR;
    }

    //计算AE幅值数据
    pstAE->stAeAmp.fPeakValue = get_max_value( pfRaw, usLen );
    pstAE->stAeAmp.fRMS = get_rms( pfRaw, usLen );

    usDFTFreq = usFreq * usLen / pstAE->uiSamplingRate;
    pstAE->stAeAmp.fH1 = dft_calc( pfRaw, usLen, usDFTFreq);	//一次谐波分量
    pstAE->stAeAmp.fH2 = dft_calc( pfRaw, usLen, usDFTFreq * 2);	//二次谐波分量

    //将幅值数据按单位转换
    convert_amp_value( &pstAE->stAeAmp, pstAE->eAE_Unit, pstAE->fGain );

    return SUCCEED;
}

//将关门时间转换成点数
#define    US_IN_ONE_SECOND     (1000000)
#define    MS_IN_ONE_SECOND     (1000)
#define    US_IN_ONE_MS         (1000)
static UINT32 get_point_num_from_block_time( float fBlockTime, UINT32 uiSampleRate )
{
    float fSampleInterval = US_IN_ONE_SECOND / uiSampleRate;//每个采样点之间的间隔
    UINT32 uiClosingTime = fBlockTime * US_IN_ONE_MS / fSampleInterval;
    uiClosingTime = ( uiClosingTime == 0 ) ? 1 : uiClosingTime;

    return uiClosingTime;
}

/************************************************
 * 功能    : 将原始数据转换成pulse图谱对应的格式
 * 描述    : 1.统计原始数据中的峰值点
 *           2.对每个峰值点记录脉冲间隔和幅值
 * 输入参数：
 *          pfRaw -- 原始数据
 *          usLen -- 数据长度
 *          pstParam -- 参数
 * 输出参数:
 *          pstAE -- 数据缓存
 * 
 * 返回值  ：succeed:0; fail:negative
 ************************************************/
AEStatus raw_convert_to_pulse( float *pfRaw, UINT16 usLen, AEWorkSetForUp* pstParam, AEAllView* pstAE )
{
    if(( NULL == pfRaw )||( NULL == pstAE ))
    {
        UHFHFCTAETEV_dbg("error in [%s]: input = null!\r\n", __func__);
        return AE_INPUT_ERROR;
    }

    float fThreshold = pstParam->fPulseThd;

    UINT32 uiSampleInterval = US_IN_ONE_SECOND / pstAE->uiSamplingRate;//每个采样点之间的间隔,以μs为单位
    UINT32 uiClosingTime = get_point_num_from_block_time(pstParam->usPulseBlocking, pstAE->uiSamplingRate);//脉冲关门时间
    UINT16 usPulseOpeningTime = get_point_num_from_block_time( (float)pstParam->usPulseGating / 1000, pstAE->uiSamplingRate );
    float fMaxValue = 0;

    UHFHFCTAETEV_dbg("AE Wireless Prps Info:\nPoint Interval:%d, Closing Time = %d\n",uiSampleInterval, uiClosingTime);

    int iIndex = 0;
    int iLastPointIndex = 0;
    UINT32 uiPulseInterval = 0.0;
    int iPointCount = 0;
    for( iIndex = 0; iIndex < usLen - usPulseOpeningTime; )
	{
		if( pfRaw[iIndex] > fThreshold )//对大于阈值的点判断
		{
            //找到峰值点
			while(( iIndex < usLen )
				&&( pfRaw[iIndex] < pfRaw[iIndex + 1] ))
			{
				iIndex++;
			}

            //存储峰值点
            if( iLastPointIndex == 0 )
            {
                iLastPointIndex = iIndex;
                iIndex += uiClosingTime;
            }
            else
            {
                uiPulseInterval = ( iIndex - iLastPointIndex ) * uiSampleInterval; 
                iLastPointIndex = iIndex;

                if( iPointCount < MAX_PHASE_PULSE_NUM_PER_MSG )
                {
                    fMaxValue = get_max_value( &pfRaw[iIndex], usPulseOpeningTime );
                    pstAE->staPulseData[iPointCount].fPeakValue = fMaxValue;
                    pstAE->staPulseData[iPointCount].uiPulseInterval = uiPulseInterval;
                    iPointCount++;
                    iIndex += usPulseOpeningTime;
                }
                iIndex += uiClosingTime;
            }

        }
        else
        {
            iIndex++;
        }
    }

    pstAE->usPulsePointNum = iPointCount;

    return SUCCEED;
}

/************************************************
 * 功能    : 将原始数据转换成phase图谱对应的格式
 * 描述    : 1.统计原始数据中的峰值点
 *           2.对每个峰值点记录相位和幅值
 * 输入参数：
 *          pfRaw -- 原始数据
 *          usLen -- 数据长度
 *          ucFreq -- 工频
 *          pstParam -- 参数
 * 输出参数:
 *          pstAE -- 数据缓存
 * 
 * 返回值  ：succeed:0; fail:negative
 ************************************************/
AEStatus raw_convert_to_phase( float *pfRaw, UINT16 usLen, UINT8 ucFreq, AEWorkSetForUp* pstParam, AEAllView* pstAE )
{
    if(( NULL == pfRaw )||( NULL == pstAE ))
    {
        UHFHFCTAETEV_dbg("error in [%s]: input = null!\r\n", __func__);
        return AE_INPUT_ERROR;
    }

    float fThreshold = pstParam->fPrpdThd;
    int iPointNumPerCycle = pstAE->uiSamplingRate / ucFreq;//每周期的采样点数
    int iCycleNum = usLen / iPointNumPerCycle;
    float fPhaseInterVal = (float)360 / iPointNumPerCycle;//每个点之间的相位差
    UINT32 uiClosingTime = get_point_num_from_block_time( pstParam->usPrpdBlocking, pstAE->uiSamplingRate );//相位关门时间
    
    UHFHFCTAETEV_dbg("Prpd Info:\nSample Rate:%d, Point Num Per Cycle:%d, Closing Time = %d\n",pstAE->uiSamplingRate, iPointNumPerCycle, uiClosingTime);

    int iIndex = 0;
    int iPointCount = 0;
    UINT32 uiPhase = 0;//峰值点的相位值
    int iCycle = 0;
    //按周期处理数据
    for( iCycle = 0; iCycle < iCycleNum; iCycle++ )
    {
        for( iIndex = iCycle * iPointNumPerCycle; iIndex < iCycle * iPointNumPerCycle + iPointNumPerCycle; )
        {
            if( pfRaw[iIndex] > fThreshold )//对大于阈值的点判断
            {
                //找到峰值点
                while(( iIndex < iCycle * iPointNumPerCycle + iPointNumPerCycle )
                    &&( pfRaw[iIndex] < pfRaw[iIndex + 1] ))
                {
                    iIndex++;
                }

                //存储峰值点的相位和幅值
                uiPhase =( iIndex % iPointNumPerCycle ) * fPhaseInterVal + 0.5;
                if( iPointCount < MAX_PHASE_PULSE_NUM_PER_MSG )
                {
                    pstAE->staPhaseData[iPointCount].fPeakValue = pfRaw[iIndex];
                    pstAE->staPhaseData[iPointCount].fPhaseValue = uiPhase;
                    iPointCount++;
                }
                iIndex += uiClosingTime;
            }
            else
            {
                iIndex++;
            }
        }
    }

    pstAE->usPhasePointNum = iPointCount;

    return SUCCEED;    
}

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
AEStatus ae_raw_to_multi_views( float *pfRaw, UINT16 usLen, AEWorkSetForUp* pstParam, AEAllView* pstAE )
{
    if(( NULL == pfRaw )||( NULL == pstAE ))
    {
        UHFHFCTAETEV_dbg("error in [%s]: input = null!\r\n", __func__);
        return AE_INPUT_ERROR;
    }

    AEStatus eAmpStatus, eWaveStatus, ePhaseStatus, ePulseStatus;

    eAmpStatus = raw_convert_to_amplitude( pfRaw, usLen, GRID_FREQUENCY, pstAE );
    eWaveStatus = raw_convert_to_wave( pfRaw, usLen, pstAE );
    ePhaseStatus = raw_convert_to_phase( pfRaw, usLen, GRID_FREQUENCY, pstParam, pstAE );
    ePulseStatus = raw_convert_to_pulse( pfRaw, usLen, pstParam, pstAE );   

    if(( SUCCEED == eAmpStatus )
     &&( SUCCEED == eWaveStatus )
     &&( SUCCEED == ePhaseStatus )
     &&( SUCCEED == ePulseStatus ))
    {
        return SUCCEED;
    }
    else 
    {
        UHFHFCTAETEV_dbg("Convert Failed!\r\n");
        return AE_FAILED;
    }

}

