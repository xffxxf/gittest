/*
 * Copyright (c) 2015.11，南京华乘电气科技有限公司
 * All rights reserved.
 *
 * 文件名称：UHFHFCTAETEV.h
 *
 *
 * 初始版本：1.0
 * 作者：吴昌盛
 * 创建日期：2016年1月13日
 * 摘要：UHFHFCTAETEV.c内部使用定义
 *
 */

#ifndef _UHFHFCTAETEV_H_
#define _UHFHFCTAETEV_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include "DataDefine.h"
#include "protocol.h"
#include "uart.h"
#include "gpio.h"
#include "UHFHFCTAETEVApi.h"
#include <stdbool.h>

#define T90_WIRELESS_AE//T90原版无线AE调理器处理.影响到参数的格式定义

#define STM32Dev  				"/dev/ttyO4"
#define STM32BaudRate      		460800
#define STM32DataBits      		8
#define STM32StopBits      		1
#define STM32Parity        		'N'

#define ZigbeeDev  				"/dev/ttyO2"
#define ZigbeeBaudRate      	115200
#define ZigbeeDataBits      	8
#define ZigbeeStopBits      	1
#define ZigbeeParity        	'N'

#define GPSDev  				"/dev/ttyO0"
#define GPSBaudRate      		115200
#define GPSDataBits      		8
#define GPSStopBits      		1
#define GPSParity        		'N'

#define LOADPRAGRAMFRAMENUM     (1024)      	//调理器每帧发送字节数
#define LOADPRAGRAMDATANUM      (1028)      	//调理器数据长度
#define LOADPRAGRAMSTARADDR     (0x800C000) 	//调理器数据起始地址
#define LOAD_ZIGBEE_FRAMESIZE   (100)           //zigbee每帧发送字节数

typedef enum
{
	CONDITIONWHERE = 1,               			//调理器子命令查询当前所在区域
	CONDITIONEGOBOOT = 2,             			//调理器子命令跳转到boot区
	CONDITIONERASE = 3,               			//调理器子命令握手擦除FLASH
	CONDITIONDATA = 4,                			//调理器子命令发送程序数据
	CONDITIONGOAPP = 5,               			//调理器子命令跳转到APP区
} UPDATECMD;

#define AE_GAIN60      1
#define AE_GAIN80      10
#define AE_GAIN100     100

/* 报文通讯类 */
typedef struct
{
	UINT8 ucaRxBuf[COMM_BUF_MAX_NUM];  			//接收缓存指针
	UINT8 ucaTxBuf[COMM_BUF_MAX_NUM];  			//发送缓存指针
	UINT16 usCmdNum;                    		//发送字节数
	UINT32 uiDelay;                     		//发送 延时后 读数据 单位us
} FrameTransfrom;

/* 本地缓存的UHF HFCT参数 */
typedef struct _UHFCTParam
{
	INT8 cUHFGain;
	INT8 cUHFFilter;
	INT8 cHFCTGain;
}UHFCTParam;

/* 下发STM32的AE参数定义 */
typedef struct
{
	UINT8 ucWorkMode;                   		//BIT[6:0]:工作模式; 0-幅值检测；1-PRPD;2-脉冲模式；3波形检测； BIT[7]:0正常工作；1自检
	UINT8 ucUnit;                       		//单位 0未知 1为dB 2为dBm 3为mV 4为pC 5为%
	float fGain;                        		//增益  1,10,100
	UINT8 ucSyncSource;                 		//同步源
	UINT16 usFComponent;                 		//FFT所需的频率分量，有效值 （10,20,30… 500） 步长10Hz
	UINT8 ucNgrid;                      		//波形图谱检测时显示的电网周期数据（1-10）
	float fAmplitudeThd;                		//幅值阈值
	float fPrpdThd;                     		//PRPD阈值
	float fPulseThd;                    		//脉冲阈值
	float fWaveThd;                     		//波形阈值
	UINT16 usPrpdBlocking;               		//PRPD的关门时间
	UINT16 usPulseBlocking;              		//脉冲模式的关门时间
	UINT16 usPulseGating;                		//脉冲模式的开门时间
#ifdef AE_FILTER_ENABLE
    UINT8 ucAEFilter;//AE带宽模式
#endif
}__attribute__((packed))
AEWorkSettings;

/* STM32返回的AE数据中的工作参数定义 */
typedef struct
{
	UINT8 ucWorkMode;                   		//BIT[6:0]:工作模式; 0-幅值检测；1-PRPD;2-脉冲模式；3波形检测； BIT[7]:0正常工作；1自检
	UINT8 ucUnit;                       		//单位 0未知 1为dB 2为dBm 3为mV 4为pC 5为%
	float fGain;                        		//增益  1,10,100
	UINT8 ucSyncSource;                 		//同步源
	UINT8 ucSyncState;                  		//同步状态
	UINT8 ucAEType;                     		//传感器类型
	UINT32 uiSamplingRate;               		//采样率
	UINT32 uiADRange;                    		//AD量程
	UINT8 ucADSampling;                 		//AD采样位数，例如13代表13位AD采样精度
	UINT8 ucADSampDataFormat;           		//AD采样数据格式（0：标准二进制，1：二进制补码）
	UINT16 usPulseNum;                   		//脉冲计数 单帧的脉冲数只有PRPD、脉冲模式有效
}__attribute__((packed))
AEDataParameter;

/* 下发无线调理器的AE工作参数定义 */
typedef struct _WirelessAEParam
{
    UINT8 ucWorkMode;
    UINT8 ucGain;
#ifdef T90_WIRELESS_AE
	UINT8 ucFComponent;
#else
    UINT16 usFComponent;
#endif
    UINT8 ucNgrid;
    UINT16 usPrpdThreshold;
    UINT16 usPrpdBlocking;
    UINT16 usPulseBlocking;
    UINT16 usPulseThreshold;
    unsigned long ulPulseGating;
}__attribute__((packed))
WirelessAEParam;

//AE调理器返回的幅值数据格式
typedef struct _AEWirelessAmpData
{
	UINT16 usPeak;//峰值
	UINT16 usRms;//有效值
	UINT16 usH1;//一次谐波
	UINT16 usH2;//二次谐波
}__attribute__((packed))
AEWirelessAmpData;

//AE调理器返回的相位数据
typedef struct _AEWirelessPrpdData
{
	UINT16 usPhase;
	UINT16 usAmp;
}__attribute__((packed))
AEWirelessPrpdData;

//AE调理器返回的飞行数据 
typedef struct _AEWirelessPulseData
{
	unsigned long ulDeltaT;
	UINT16 usAmp;
}
AEWirelessPulseData;

//AE调理器返回的参数
typedef struct _AEWirelessMsgParam
{
	UINT8 ucWorkMode;//AE工作模式
	UINT8 ucGainLevel;//增益等级
	UINT8 ucSyncSource;//同步源
}AEWirelessMsgParam;

//AE返回的多图谱参数数据
#pragma pack(1)
typedef struct _AEMultiViewParam
{
    UINT8 ucSyncState; //同步状态
    UINT8 ucSyncSource; //同步源
    UINT8 ucChannel; //传感器类型
    float fGain; //增益
    UINT32 uiSamplingRate; //采样率
    UINT32 uiADRange; //AD量程
    UINT8 ucADSampling; //AD采样位数，例如13代表13位AD采样精度
    UINT8 ucADSampDataFormat; //AD采样数据格式（0：标准二进制，1：二进制补码）
    UINT8 ucWorkMode; //工作模式
    UINT8 ucUnit; //单位 1:dB  3:mV

    UINT16 usPhasePointNum;//数据中的相位图谱点数
    UINT16 usPulsePointNum;//数据中的脉冲图谱点数
    UINT16 usWavePointNum;//数据中的波形图谱点数
}AEMultiViewParam;

//调理器地址维护
typedef struct _ConditionerAddr
{
	WLDeviceAddr stAddr;
	bool bAddrSet;
}ConditionerAddr;

#pragma pack()

#define HFCTDB_COUNT            (22)    		//HFCT dB表大小
#define HFCTDB_MAX              (20)    		//HFCT dB最大值

#define SET_TEV_DELAY20ms       (4) 			//延时10ms 单位5ms
#define READ_DATA_DELAY30ms     (6) 			//延时30ms 单位5ms
#define READ_DATA_DELAY50ms     (10) 			//延时50ms 单位5ms
#define READ_DATA_DELAY85ms     (17) 			//延时80ms 单位5ms
#define SEND_CONTROL_DELAY100ms (20)			//延时100ms 单位5ms
#define SEND_CONTROL_DELAY200ms (40)			//延时100ms 单位5ms
#define DEV_VERSION_DELAY500ms  (100)			//延时500ms 单位5ms
#define START_ADDR_DELAY1s      (200)     		//延时1s 单位5ms
#define START_ADDR_DELAY10s     (2000)    		//延时10s 单位5ms

#define UHF_TRANSFORM           (0.48) 			//UHF的AD值转换成频谱值 系数

#define RETRY_NUM               (3)    			//报文通讯失败 重复次数
#define STM32VERBYTENUM			(24)  			//STM32版本字节数最大值
#define STM32DEVICEIDNUM		(8)  			//设备序列号字节数最大值

#define M16TO8_L(x)             ((x)&0xff)
#define M16TO8_H(x)             (((x)>>8)&0xff)
#define M8TO16(l,h)             ((l)|((h)<<8))
#define M32TO8_L1(x)            ((x)&0xff)
#define M32TO8_L2(x)            (((x)>>8)&0xff)
#define M32TO8_L3(x)            (((x)>>16)&0xff)
#define M32TO8_L4(x)            (((x)>>24)&0xff)

#define RETRYTXRX(x)            		do{ (x)--;usleep(20000);}while(0);continue;

#define BIT(x)                          (1<<(x))
#define GET_BIT(data,x)                 (((data)&BIT(x))>>x)
#define GET_BIT_RANGE(data,min,max)     (((data)&(BIT((max)+1)-BIT(min)))>>min)

#define GPIO_TO_PIN(bank, gpio)  		(32 * (bank) + (gpio))

#define GPS_STANDBY         			GPIO_TO_PIN(0,4)
#define GPS_RESET           			GPIO_TO_PIN(5,6)
#define GPS_FORCEON         			GPIO_TO_PIN(0,5)
#define GPS_PPS             			GPIO_TO_PIN(0,0)

#endif
