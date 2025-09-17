/*
 * Copyright (c) 2015.11，南京华乘电气科技有限公司
 * All rights reserved.
 *
 * 文件名称：UHFHFCTAETEVApi.c
 *
 *
 * 初始版本：1.0
 * 作者：吴昌盛
 * 创建日期：2016年1月13日
 * 摘要：UHF HFCT AE TEV通讯的API接口
 *
 * 说明:接收器接UHF、HFCT、同步器，接收器通过Zigbee无线与ARM板通讯，ARM通过串口与Zigbee连接；
 *      AE、TEV直接串口至ARM。
 *     新增电缆电流检测调理器外设支持,数据链路如下：arm<--uart-->zigbee《--cc2510--〉 cable current terminal device
 *      用户连接无线调理器读取采集数据控制流程：
 *      1.start_search_wireless（eDeviceType）--扫描指定类型的无线调理器
 *      2.get_wireless_addrlist（eDeviceType）--获取调理器地址列表
 *      3.set_wireless_addrlist（eDeviceType）--设置调理器地址
 *      <4...>设置调理器参数<可选>
*      5.get_xxx_data读取采集数据
 */

#include "UHFHFCTAETEV.h"
#include "protocol.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include "ae_view_convert.h"
#include "crc.h"

static INT32 g_iFd_STM32 = -1; //STM32串口文件描述符
static WorkMode g_eWorkMode = Waveform; //保存超声波工作模式 用以确定超声波读写延时
static Frequency g_eFrequency = FREQ_UNKNOW; //保存频率状态
static AE_Unit g_eAEWirelessUnit = mV;//AE无线调理器单位,默认为毫伏
static AEWorkSetForUp s_stAeWirelessWorkParam;//AE无线调理器参数
static UHFCTParam s_stUHFCTParam;//本地缓存的UHF HFCT参数
#ifdef HARDEARENEWV2
static INT32 g_iFd_Zigbee = -1; //Zigbee串口文件描述符
#endif
/**记录主机设置的调理器地址缓冲区&调理器版本缓冲区
 * 作用：在读取调理器采集数据之前，获取调理器版本号，确保能够解析到正确的数据版本
 * 步骤：
 * 1、set_wireless_addrlist 接口保存调理器地址到 gstWlDevAddr
 * 2、set_wireless_addrlist 接口调用get_conditioner_data_version 获取数据版本并保存到g_connectedWlDevDataVerArray
 * 3、get_uhfhfct_data 读取数据前 判断g_connectedWlDevDataVerArray 是否有效，有效则直接获取采集数据，无效则调用get_conditioner_data_version
 *    确保获取到正确的数据版本信息！！
*/
static WLDeviceAddr gstWlDevAddr[MAX_TERMINAL_NUM];
#define INVALI_DATA_VER (0x55)
static INT8 g_connectedWlDevDataVerArray[MAX_TERMINAL_NUM];
/**
 * libUHFHFCTAETEVApi.so 版本号 修改原因 修改时间 修改人
 * v1.00 初次发布  2016.5.13  鲍贺川
 * v1.01 修改gpcVerStr为静态变量，否则会导致多个库文件导出相同的
 *       gpcVerStr 符号，无法查看每个库文件的正确版本号。  2016.5.17 鲍贺川
 * V1.0.0.2 增加STM32的版本获取 2016.7.11 吴昌盛
 * V1.0.3.0 增加对第二版硬件支持 2016.7.11 吴昌盛
 * V1.0.4.0 增加获取Stm32版本及设备序列号及通讯改为5ms定时查询 2016.7.11 吴昌盛
 * V1.0.4.1 修改超时 2016.7.11 吴昌盛
 * V1.0.4.2 获取设备序列号bug 吴昌盛
 * V1.0.4.3 增加不回复的-2返回，固件升级的进度更新 吴昌盛
 * V1.0.4.4 HFCT最大最下不显示增益偏高偏低 2016.8.8 吴昌盛
 * V1.0.5.0 增加TEV校准参数 	2016.8.9 吴昌盛
 * V1.0.5.1 TEV校准系数改为UINT32 	2016.8.9 吴昌盛
 * V1.0.5.2 增加读取Stm32状态及跳转至App接口，停止原来返回的-2 	2016.8.11 吴昌盛
 * V1.0.5.3 修复增加读取Stm32状态及跳转至App接口，停止原来返回的-2的bug 	2016.8.11 吴昌盛
 * V1.0.5.4 1)add gComMutex 线程锁，串行化对send_read_frame接口的访问
 *          2）修改conditioner_cmd返回值的定义，修改get_stm32_inapp返回值定义
 * V1.0.5.5 修改获取外设地址的流程，开始等10秒后再获取地址 	2016.8.20 吴昌盛
 * V1.1.0.0 arm/stm32 串口通讯报文优化 2016.8.23 鲍贺川
 * V2.0.1.0 修改了get_ae_data函数的返回值,对不同的错误进行区分,方便view层判断 2016.10.9 李遥
 * V2.0.2.0 修改了AE波形和幅值数据获取的超时 2016.10.9 李遥
 * V2.0.3.0 修正了连接调理器卡死的问题 2016.10.12 李遥
 * V2.0.4.0 修正了UHFHFCT设置工作参数容易失败的问题 2016.10.13 李遥
 * V2.0.4.1 去除get ae data打印 2016.10.18 吴昌盛
 * V2.0.5.0 修正TEV校准的报文解析,现在能正确反映是否校准成功 2016.10.20 李遥
 * V2.0.6.0 对ZIGBEE的通讯接口增加重发机制,避免了UHF HFCT的PRPS图谱卡顿现象 2016.11.1 李遥
 * V3.0.0.1 添加对Z159 R3版本硬件无线AE和TEV PRPS的支持 2018.3.15 李遥
 * V3.0.1.1 增加对无线AE的参数重发,解决参数配置有可能失败的问题 2018.3.16 李遥
 * V3.0.2.1 修复TEV PRPS采集数据后三个字节为空的bug           2018.3.21 李遥
 * V3.0.3.1 修改读电量接口,修复TEV校准命令返回报文解析错位的问题 2018.3.26 李遥
 * V3.1.0.1 新增AE四图谱同时采集接口,修复无线AE测试问题         2018.4.10 李遥
 * V3.1.1.1 修改ae采集数据的逻辑为,无线状态才向zigbee发送报文 2018.4.18 李遥
 * V3.1.2.1 对无线UHF HFCT调理器增加参数校验,参数不匹配则不处理数据 2018.4.19 李遥
 * V3.1.3.1 对一次采集四次图谱接口限制波形数据最大长度,防止异常情况引起的波形取数据越界 2018.4.26 李遥
 * V3.1.4.1 修复无线AE的mv值总是返回整型值的问题 2018.4.26 李遥
 * V3.2.0.1 增加对无线AE调理器的一次上送四图谱的处理 2018.5.9 李遥
 * V3.2.1.1 修复参数不一致时HFCT下发参数异常的问题 2018.5.10 李遥
 * V3.2.2.1 增加对UHF的滤波参数的检查 2018.5.13 李遥
 * V3.2.3.1 修复表贴状态不对AE读数据命令处理的bug 2018.6.1 李遥
 * V3.2.4.1 修改所有报文的超时为10s，对报文新增命令字校验操作 2018.6.6 李遥
 * V3.2.5.1 对所有AE读数据均向zigbee发送改变数据类型的报文，保证同步脉冲发送频率 2018.6.28 李遥
 * V3.2.6.1 超时10秒可能导致界面卡住，恢复原来超时时间 2018.8.8 吴昌盛
 * V3.2.7.0 在命令字判断中对AE四图谱采集特殊处理；修复命令字判断返回值错误的问题 2018.11.8 李遥
 * V3.2.8.0 修复AE无线调理器解析增益状态错误的问题；修改TEV读取mv值接口为同时读取三个通道 2019.3.22 李遥
 * V3.2.9.0 修复UHF、HFCT在信号发生器断电情况下图谱刷新卡顿的问题 2019.4.30 鲍贺川
 * V3.2.10.0  init_uhfhfctaetev 初始化接口支持重复调用  2019.6.5 鲍贺川
 * V3.2.11.0 修复串口通信错误码定义错误，导致程序崩溃的问题 2019.7.3 鲍贺川
 * V3.2.12.0  1)增加电缆电流调理器支持  2019.7.4 鲍贺川
 *            2)修复无线AE读数据过程中，插入切换uhf/hfct图谱操作时导致AE读数据失败的问题 2019.7.9 鲍贺川
 * V3.2.13.0 修正TEV校准参数结构体定义错误  2019.7.16 鲍贺川
 * V3.2.14.0 修正AE波形数据转换公式计算错误 20191028 鲍贺川
 * V4.0.0.0 更新TEV校准流程                2020.1.15 鲍贺川
 * V4.0.1.0 修复UHF增益定义错误问题         2020.9.9 鲍贺川
 * V4.0.2.0 更新UHF ADC码值到DB值的映射关系，UHF 显示范围变更为 0~80dB  2021.11.29 鲍贺川
 * V4.1.2.0 兼容支持UHF R13P1 HFCT 新版本调理器硬件支持        2024.5.26 鲍贺川
 * V4.1.3.0 更新ae_adtomv 函数接口 支持负ADC码值转mv;增加电子标签测试读写测试接口           2024.6.28 鲍贺川
 *  *  **/

/************************************************
 * 函数名   ：wireless_ae_adc_to_mv
 ************************************************/
static float wireless_ae_adc_to_mv( UINT16 usData, UINT8 ucGainLevel );

/************************************************
 * 函数名   ：wireless_ae_mv_to_adc
 ************************************************/
static int wireless_ae_mv_to_adc( float fData, UINT8 ucGainLevel );


static const INT8 *stm32VerStr = "V4.1.3.0";
const INT8 * UHFHFCTAETEVApi_version(void)
{
    return stm32VerStr;
}

/*用于保护串口通信的串行化：当线程A通过串口与STM32通信时，线程B必须等待线程A完成通信时才能开始与STM32通信*/
static pthread_mutex_t gComMutex; /*stale variable ,do not use it any more*/

static pthread_mutex_t gStm32ComMutex;
static pthread_mutex_t gZigbeeComMutex;

/*用于保存调理器地址，在ae工作时清空uhf和hfct地址，保证ae的同步连接*/
ConditionerAddr s_stUHFAddr = {0};
ConditionerAddr s_stHFCTAddr = {0};

//清除uhf和hfct调理器地址
//因95版本程序中没有关闭AE电源操作，暂时注释
void clear_uhf_hfct_conditioner_addr( void )
{
    // UINT8 ucEmptyAddr[8] = {0};
    // set_wireless_addrlist( ucEmptyAddr, UHF_TERMINAL );
    // set_wireless_addrlist( ucEmptyAddr, HFCT_TERMINAL );
}

//重新设置uhf和hfct调理器地址
//因95版本程序中没有关闭AE电源操作，暂时注释
void reset_uhf_hfct_conditioner_addr( void )
{
    // if( s_stUHFAddr.bAddrSet )
    // {
    //     set_wireless_addrlist( s_stUHFAddr.stAddr.caAddr, UHF_TERMINAL );   
    // }

    // if( s_stHFCTAddr.bAddrSet )
    // {
    //     set_wireless_addrlist( s_stHFCTAddr.stAddr.caAddr, HFCT_TERMINAL );   
    // }
}

/************************************************
 * 函数名   ：comm_with_stm32_handle

 * 输入参数 ：iComPort --串口设备描述符
 *         pucBuf --发送报文数据段缓冲区指针
 *         usDataLen  --发送报文数据段字节长度,
 为0时不执行发送命令,仅执行接收命令
 *         pstFrame->uiRcvTimeout --接收报文超时时间（毫秒）
 *
 * 输出参数 ：pstFrame->ucaRcvBuf --接收缓冲区指针
 *         pstFrame->UIRcvNum  --接收报文字节长度
 *
 * 返回值   ： 参见CommErr定义
 *
 * 功能     ：arm与stm32之间串口通讯的报文发送/接收处理
 ************************************************/
INT32 comm_with_stm32_handle(INT32 iComPort, FrameBuf *pstFrame, UINT8 *pucBuf, UINT16 usDataLen)
{
    INT32 iRet = 0;
    pthread_mutex_lock(&gStm32ComMutex);
    iRet = com_comm_handle(iComPort, pstFrame, pucBuf, usDataLen);
    UINT8 ucCommand = pucBuf[0];
    if (iRet == COM_OK)
    {
        UINT8 *pucDataBuf = NULL;
        UINT16 usDataLen = 0;
        pucDataBuf = get_frame_data(pstFrame, &usDataLen);
        //报文响应状态字校验
        if (pucDataBuf[CMD_RESPONSE_STATE_POS] != CMD_RESPONSE_STATE_OK)
        {
            iRet = -COM_RESPONSE_STATE_ERR;
        }
        if((ucCommand!=COM_AE_READ_MULTI_VIEWS)&&(pucDataBuf[CMD_RESPONSE_CMD_POS] != ucCommand))
        {
            iRet = -COM_COMMAND_BYTE_ERR;
        }
    }
    pthread_mutex_unlock(&gStm32ComMutex);
    return iRet;
}

/************************************************
 * 函数名   ：comm_with_zigbee_handle

 * 输入参数 ：iComPort --串口设备描述符
 *         pucBuf --发送报文数据段缓冲区指针
 *         usDataLen  --发送报文数据段字节长度,
 为0时不执行发送命令,仅执行接收命令
 *         pstFrame->uiRcvTimeout --接收报文超时时间（毫秒）
 *
 * 输出参数 ：pstFrame->ucaRcvBuf --接收缓冲区指针
 *         pstFrame->UIRcvNum  --接收报文字节长度
 *
 * 返回值   ： 参见CommErr定义
 *
 * 功能     ：arm与zigbee之间串口通讯的报文发送/接收处理
 ************************************************/
INT32 comm_with_zigbee_handle(INT32 iComPort, FrameBuf *pstFrame, UINT8 *pucBuf, UINT16 usDataLen)
{
    INT32 iRet = 0;
    UINT8 ucCommand = pucBuf[0];
    pthread_mutex_lock(&gZigbeeComMutex);
    iRet = com_comm_handle(iComPort, pstFrame, pucBuf, usDataLen);
    
    if (iRet == COM_OK)
    {
        UINT8 *pucDataBuf = NULL;
        UINT16 usDataLen = 0;
        pucDataBuf = get_frame_data(pstFrame, &usDataLen);
        if (pucDataBuf[CMD_RESPONSE_CMD_POS] != ucCommand) 
        {
            iRet = -COM_COMMAND_BYTE_ERR;
        }
        //报文响应状态字校验
        if (pucDataBuf[CMD_RESPONSE_STATE_POS] == CMD_RESPONSE_STATE_ERR)
        {
            iRet = -COM_RESPONSE_STATE_ERR;
        }
       
    }
    pthread_mutex_unlock(&gZigbeeComMutex);
    return iRet;
}

/************************************************
 * 函数名   ：comm_zigbee_iap_handle

 * 输入参数 ：iComPort --串口设备描述符
 *         pucBuf --发送报文数据段缓冲区指针
 *         usDataLen  --发送报文数据段字节长度,
 为0时不执行发送命令,仅执行接收命令
 *         pstFrame->uiRcvTimeout --接收报文超时时间（毫秒）
 *
 * 输出参数 ：pstFrame->ucaRcvBuf --接收缓冲区指针
 *         pstFrame->UIRcvNum  --接收报文字节长度
 *
 * 返回值   ： 参见CommErr定义
 *
 * 功能     ：arm与zigbee之间固件升级的报文发送/接收处理
 ************************************************/
INT32 comm_zigbee_iap_handle(INT32 iComPort, FrameBuf *pstFrame, UINT8 *pucBuf, UINT16 usDataLen)
{
    INT32 iRet = 0;
    pthread_mutex_lock(&gZigbeeComMutex);
    iRet = zigbee_iap(iComPort, pstFrame, pucBuf, usDataLen);
    pthread_mutex_unlock(&gZigbeeComMutex);
    if (iRet == COM_OK)
    {
        UINT8 *pucDataBuf = NULL;
        UINT16 usDataLen = 0;
        pucDataBuf = get_frame_data(pstFrame, &usDataLen);
        //报文响应状态字校验
        if (pucDataBuf[CMD_ZIGBEE_IAP_STATUS] != CMD_RESPONSE_STATE_OK)
        {
            iRet = -COM_RESPONSE_STATE_ERR;
        }
    }
    return iRet;
}

/************************************************
 * 函数名   ：init_uhfhfctaetev

 * 输入参数 ：

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：初始化串口

 * 描述     :
 ************************************************/
INT32 init_uhfhfctaetev(void)
{
    UARTPortSettings stUARTPortSettings;
    INT32 iRet = 0;

#ifdef NODEVSIMDATA//模拟测试
    return HC_SUCCESS;
#endif
    if(g_iFd_STM32 > 0)
    {
        printf("[warning]:init_uhfhfctaetev->module already initialized!\n");
        return HC_SUCCESS;
    }
    //初始化STM32串口
    stUARTPortSettings.pcDeviceNode = STM32Dev; //设备节点
    stUARTPortSettings.ucDatabits = STM32DataBits; //数据位
    stUARTPortSettings.ucStopbits = STM32StopBits; //停止位
    stUARTPortSettings.uiBaudRate = STM32BaudRate; //波特率
    stUARTPortSettings.cParity = STM32Parity; //奇偶校验
    g_iFd_STM32 = init_uart(&stUARTPortSettings);
    if (g_iFd_STM32 < 0)
    {
        printferr("g_iFd_STM32 failed \n");
        return HC_FAILURE;
    }

#ifdef HARDEARENEWV2
    //初始化zigbee串口
    stUARTPortSettings.pcDeviceNode = ZigbeeDev;//设备节点
    stUARTPortSettings.ucDatabits = ZigbeeDataBits;//数据位
    stUARTPortSettings.ucStopbits = ZigbeeStopBits;//停止位
    stUARTPortSettings.uiBaudRate = ZigbeeBaudRate;//波特率
    stUARTPortSettings.cParity = ZigbeeParity;//奇偶校验
    g_iFd_Zigbee = init_uart(&stUARTPortSettings);
    if (g_iFd_Zigbee < 0)
    {
        printferr("g_iFd_Zigbee failed \n");
        return HC_FAILURE;
    }

#endif
    pthread_mutex_init(&gComMutex, NULL); //初始化gComMutex
    pthread_mutex_init(&gStm32ComMutex, NULL);
//	//初始化GPS控制引脚
//	iRet += gpio_export(GPS_STANDBY);
//	iRet += gpio_direction_output(GPS_STANDBY, ACTIVE_HIGH);
//	iRet += gpio_export(GPS_RESET);
//	iRet += gpio_direction_output(GPS_RESET, ACTIVE_HIGH);
//	iRet += gpio_export(GPS_FORCEON);
//	iRet += gpio_direction_output(GPS_FORCEON, ACTIVE_LOW);
//	iRet += gpio_export(GPS_PPS);
//	iRet += gpio_direction_input(GPS_PPS);
//	if (iRet < 0)
//	{
//		printferr("iRet < 0\n");
//		//return HC_FAILURE;
//	}

    return HC_SUCCESS;
}
/************************************************
 * 函数名   ：exit_uhfhfctaetev

 * 输入参数 ：

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：退出

 * 描述     :
 ************************************************/
void exit_uhfhfctaetev(void)
{
#ifdef NODEVSIMDATA//模拟测试
    return;
#endif

//	//撤销GPS导出
//	gpio_unexport(GPS_STANDBY);
//	gpio_unexport(GPS_RESET);
//	gpio_unexport(GPS_FORCEON);
//	gpio_unexport(GPS_PPS);

    close(g_iFd_STM32);
    g_iFd_STM32 = -1;
#ifdef HARDEARENEWV2
    close(g_iFd_Zigbee);
    g_iFd_Zigbee = -1;
#endif
}

#ifdef UHFHFCTAETEV_DEBUG
/************************************************
 * 函数名   ：printrxtxframe

 * 输入参数 ：UINT8 *pucData报文起始指针,UINT16 usDataLen报文长度

 * 输出参数 ：

 * 返回值   ：

 * 功能     ：打印报文

 * 描述     : 只有在UHFHFCTAETEVApi.h头文件定义UHFHFCTAETEV_DEBUG有效
 ************************************************/
static void printrxtxframe(UINT8 *pucData, INT32 iDataLen)
{
    UINT16 usTemp = 0;

    for (usTemp = 0; usTemp < iDataLen; usTemp++)
    {
        if (usTemp != 0 && usTemp % 25 == 0)
        {
            printf("\n");
        }
        printf("%x  ", pucData[usTemp]);
    }
    printf("\n");
}
#endif

/************************************************
 * 函数名   ：change_data_type

 * 输入参数 ：WLDeviceType eDeviceType设备类型

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：切换数据类型

 * 描述     : 只有UHF HFCT用到
 ************************************************/
static INT32 change_data_type(WLDeviceType eDeviceType)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;

    static WLDeviceType eDataChangCtl = 0xfe; //每次切换数据类型，需要设置
    //无需切换数据类型
    if (eDataChangCtl == eDeviceType)
        return HC_SUCCESS;
    //AE时需要清空uhf和hfct的调理器，避免同步器延迟发送脉冲的时间
    if( AE_TERMINAL == eDeviceType )
    {
        clear_uhf_hfct_conditioner_addr();
    }

    ucaDataBuf[usDataLen++] = DATA_TYPE_COMMAND;

    //数据类型
    switch (eDeviceType)
    {
    case UHF_TERMINAL:
        ucaDataBuf[usDataLen++] = BIT(DATATYPE_UHFBIT);
        break;
    case HFCT_TERMINAL:
        ucaDataBuf[usDataLen++] = BIT(DATATYPE_CTBIT);
        break;
    case AE_TERMINAL:
        ucaDataBuf[usDataLen++] = BIT(DATATYPE_AEBIT);
        break;
    case NODEVICEType:
        ucaDataBuf[usDataLen++] = 0;
        break;
    case CABLE_CURRENT_TERMINAL:
        ucaDataBuf[usDataLen++] = BIT(DATATYPE_CABLEBIT);
        break;
    default:
        printferr("\n");
        return HC_FAILURE;
    }

    stFrame.uiRcvTimeout = TIMEOUT_100MS;

    //发送报文
    iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(DATA_TYPE_COMMAND, (CommErr)(-iRet));
        return HC_FAILURE;
    }

    //保存数据类型
    eDataChangCtl = eDeviceType;

    return HC_SUCCESS;
}

/************************************************
 * 输入参数 ：
 *          eDeviceType -- 设备类型
 *          cGain -- 增益的值
 * 返回值   ：
 *          增益的枚举
 * 功能     ：
 *          将增益的值转成增益的枚举
 ************************************************/
static UHFHFCTGain gain_value_to_enum( WLDeviceType eDeviceType, INT8 cGain )
{
    UHFHFCTGain eGain = GAIN_0;

    switch( eDeviceType )
    {
        case UHF_TERMINAL:
        {
            switch( cGain )
            {
                case 0:
                eGain = GAIN_0;
                break;

                case 20:
                eGain = GAIN_20;
                break;

                default:
                printf("error in %s : input error!\n");
                break;
            }
        }
        break;

        case HFCT_TERMINAL:
        {
            switch( cGain )
            {
                case 0:
                eGain = ATTENUATE_0;
                break;

                case 20:
                eGain = ATTENUATE_20;
                break;

                case 40:
                eGain = ATTENUATE_40;
                break;

                case 60:
                eGain = ATTENUATE_60;
                break;

                default:
                printf("error in %s : input error!\n");
                break;
            }
        }
        break;

        default:
        printf("error in %s : input error!\n");
        break;
    }
    return eGain;
}
#define UHF_OFFSET_VAL (70)
/**UHF幅值上下限，适用于新版本UHF调理器硬件*/
#define MAX_UHF_DB_VAL (90)
#define MIN_UHF_DB_VAL (-10)
#define MAX_HFCT_DB_VAL (80)
#define MIN_HFCT_DB_VAL (0)
/**UHF幅值上下限，适用于老版本UHF调理器硬件*/
#define MAX_UHF_DB_VAL_V1  (80)
#define MIN_UHF_DB_VAL_V1  (0)
#define MAX_HFCT_DB_VAL_V1  (20)
#define MIN_HFCT_DB_VAL_V1  (0)
typedef struct _cali_coef
{
    float k;
    float b;
}CaliCoef;
/**
 * UHF调理器默认校准系数
 * 
*/
CaliCoef gUhfCoef[2][3] = 
{   //all pass,       low pass,         high pass
    { {0.3681,-92.557},{0.3694,-91.5483},{0.3566,-87.0695} }, //20dB 增益
    { {0.3604,-66.973},{0.3595,-67.0177},{0.349,-61.9593}  }, //0dB 增益
};

/**
 * HFCT调理器默认校准系数
 * 
*/
CaliCoef gHFCTCoef = {0.5081,-35.928};

void get_HFCT_cali_coef(CaliCoef *coef)
{
    coef->k = gHFCTCoef.k;
    coef->b = gHFCTCoef.b;
}
void get_UHF_cali_coef(CaliCoef *coef,unsigned char gain,unsigned char bandwidth)
{
	coef->k = gUhfCoef[gain][bandwidth].k;
	coef->b = gUhfCoef[gain][bandwidth].b;
}

static void calc_uhf_dB_val(UINT8 *pucAdcVal,UHFHFCTData *pstUHFHFCTData)
{
    UINT16 i ;
    UINT16 usMax_uhf_index = 0;
    INT8 cDataVer = DATA_VERSION_V2;
    cDataVer = pstUHFHFCTData->cDataVersion;
    switch(cDataVer)
    {
        case DATA_VERSION_V1:
        {
            INT16 sTemp_uhf_Value = 0;
            for(i=0;i<SPECTTRUMNUM;i++)
            {
                sTemp_uhf_Value = pucAdcVal[i] * UHF_TRANSFORM;
                if (sTemp_uhf_Value < 10)
                {
                    pstUHFHFCTData->caSpectrum[i] = -1;
                }
                else
                {
                    pstUHFHFCTData->caSpectrum[i] = sTemp_uhf_Value - 9;
                }
                //记录最大原始值索引
                if (pucAdcVal[ usMax_uhf_index] < pucAdcVal[i])
                {
                    usMax_uhf_index = i;
                }
            }
            pstUHFHFCTData->cMaxSpectrum = pstUHFHFCTData->caSpectrum[usMax_uhf_index];
            /**UHF幅值上下限*/
            pstUHFHFCTData->cAmpLowerLimit = MIN_UHF_DB_VAL_V1;
            pstUHFHFCTData->cAmpUpperLimit = MAX_UHF_DB_VAL_V1;
        #ifdef  UHFHFCTAETEV_DEBUG
            printf("UHF ADC val:(DataVer = %d)\n",cDataVer);
            data_print(pucAdcVal,SPECTTRUMNUM);
            printf("UHF DB val:\n");
            signed_data_print(pstUHFHFCTData->caSpectrum,SPECTTRUMNUM);
            printf("Max DB val: %d\n",pstUHFHFCTData->cMaxSpectrum);
        #endif

            break;
        }
        case DATA_VERSION_V2:
        {
            //计算实际UHF频谱值
            CaliCoef coef;
            get_UHF_cali_coef(&coef,pstUHFHFCTData->cGain,pstUHFHFCTData->eBandWidth);
            printf("uhf gain = %d, filter = %d,k = %f,b = %f\n",pstUHFHFCTData->cGain, pstUHFHFCTData->eBandWidth,coef.k,coef.b);
            for(i=0;i<pstUHFHFCTData->usPhaseNum;i++)
            {
                pstUHFHFCTData->fPrpsDataPerGridPeriod[i] = coef.k*pucAdcVal[i] +coef.b + UHF_OFFSET_VAL;
            //记录最大原始值索引
                if (pucAdcVal[usMax_uhf_index] < pucAdcVal[i])
                {
                    usMax_uhf_index = i;
                }
            }
            pstUHFHFCTData->fMaxSpectrum = pstUHFHFCTData->fPrpsDataPerGridPeriod[usMax_uhf_index];
            /**UHF幅值上下限*/
            pstUHFHFCTData->cAmpLowerLimit = MIN_UHF_DB_VAL;
            pstUHFHFCTData->cAmpUpperLimit = MAX_UHF_DB_VAL;
        #ifdef UHFHFCTAETEV_DEBUG
            printf("UHF ADC val:(DataVer = %d)\n",cDataVer);
            data_print(pucAdcVal,pstUHFHFCTData->usPhaseNum);
            printf("UHF DB val:\n");
            float_data_print(pstUHFHFCTData->fPrpsDataPerGridPeriod,pstUHFHFCTData->usPhaseNum);
            printf("Max DB val: %.1f\n",pstUHFHFCTData->fMaxSpectrum);
        #endif
            
            break;
        }
        default:
        {

        }
       
    }

}

/************************************************
 * 函数名   ：parse_uhf_frame

 * 输入参数 ：UINT8 *pucSrcData报文数据

 * 输出参数 ：UHFHFCTData *pstUHFHFCTData解析数据

 * 返回值   ：HC_SUCCESS:成功; HC_FAILURE:失败

 * 功能     ：解析UHF报文

 * 描述     :
 ************************************************/
static INT32 parse_uhf_frame(UHFHFCTData *pstUHFHFCTData, UINT8 *pucSrcData)
{
    //参数检查
    if (pstUHFHFCTData == NULL || pucSrcData == NULL)
    {
        printf("in parse_uhf_frame pstUHFHFCTData==NULL || pucSrcData==NULL\n");
        return HC_FAILURE;
    }

    //状态字解析
    pstUHFHFCTData->eBandWidth = (UHFFilterControl) GET_BIT_RANGE(pucSrcData[STATE_BYTE], BIT0, BIT1); //带宽
    if (pstUHFHFCTData->eBandWidth == LOW_PASS_FILTER) //上下枚举变量不同  转换
    {
        pstUHFHFCTData->eBandWidth = HIGH_PASS_FILTER;
    }
    else if (pstUHFHFCTData->eBandWidth == HIGH_PASS_FILTER)
    {
        pstUHFHFCTData->eBandWidth = LOW_PASS_FILTER;
    }
    pstUHFHFCTData->cGain = GET_BIT(pucSrcData[STATE_BYTE], BIT2); //增益
    pstUHFHFCTData->eSyncSource = (SyncSource)(GET_BIT(pucSrcData[STATE_BYTE], BIT3) + 1); //同步源  +1上下枚举变量不同  转换
    pstUHFHFCTData->eSyncState = (SyncState) GET_BIT(pucSrcData[STATE_BYTE], BIT4); //同步状态

    /**TODO: PRPS相位分辨率应支持用户设置，目前固定为60*/
    pstUHFHFCTData->usPhaseNum  = SPECTTRUMNUM;
    
    //计算实际UHF频谱值 
    calc_uhf_dB_val(&pucSrcData[DATA_BYTE],pstUHFHFCTData);


    return HC_SUCCESS;

}
static void calc_hfct_dB_val(UINT8 *pucAdcVal,UHFHFCTData *pstUHFHFCTData)
{
    UINT16 i ;
    UINT16 usMaxdBIndex = 0;
    INT8 cDataVer = DATA_VERSION_V2;
    cDataVer = pstUHFHFCTData->cDataVersion;
    switch(cDataVer)
    {
        case DATA_VERSION_V1:
        {
            INT8 cTempdBVal = 0;
            static const UINT8 HFCT_dB[HFCTDB_COUNT] =
            { 10, 13, 18, 22, 27, 34, 42, 52, 61, 70, 79, 88, 98, 108, 117, 127, 138, 147, 150, 157, 158, 162 }; //频谱 dB查表
            for (i = 0; i < SPECTTRUMNUM; i++)
            {
                //当前dB值
                cTempdBVal = HFCTDB_MAX;
                while (pucAdcVal[i] <= HFCT_dB[cTempdBVal] && cTempdBVal > 0) //查表转换dB
                {
                    cTempdBVal--;
                }
                if (pucAdcVal[i] == HFCT_dB[HFCTDB_MAX]) //HFCTDB_MAX时不同，AD==HFCT_dB[HFCTDB_MAX],结果为HFCTDB_MAX
                {
                    cTempdBVal = HFCTDB_MAX;
                }

                //加上衰减
                cTempdBVal += pstUHFHFCTData->cGain;
                //保存频谱值
                pstUHFHFCTData->caSpectrum[i] = cTempdBVal;
                //记录最大原始值索引
                if (pucAdcVal[usMaxdBIndex] < pucAdcVal[i])
                {
                    usMaxdBIndex = i;
                }
            }
            pstUHFHFCTData->cMaxSpectrum = pstUHFHFCTData->caSpectrum[usMaxdBIndex];
            //判断频谱状态
            if (pucAdcVal[usMaxdBIndex] < HFCT_dB[0] && pstUHFHFCTData->cGain != 0) //低于范围
            {
                pstUHFHFCTData->eSpectrumState = SPECTRUM_UNDER_LOW;
            }
            else if (pucAdcVal[usMaxdBIndex] > HFCT_dB[HFCTDB_MAX] && pstUHFHFCTData->cGain != 60) //大于范围
            {
                pstUHFHFCTData->eSpectrumState = SPECTRUM_ABOVE_HIGH;
            }
            else
            {
                pstUHFHFCTData->eSpectrumState = SPECTRUM_INSIDE_RANGE; //范围内
            } 
            /**HFCT幅值上下限*/
            pstUHFHFCTData->cAmpLowerLimit = MIN_HFCT_DB_VAL_V1 + pstUHFHFCTData->cGain;
            pstUHFHFCTData->cAmpUpperLimit = MAX_HFCT_DB_VAL_V1 + pstUHFHFCTData->cGain;

        #ifdef UHFHFCTAETEV_DEBUG
            printf("HFCT ADC val:(DataVer = %d)\n",cDataVer);
            data_print(pucAdcVal,SPECTTRUMNUM);
            printf("HFCT DB val:\n");
            signed_data_print(pstUHFHFCTData->caSpectrum,SPECTTRUMNUM);
            printf("Max DB val: %d\n",pstUHFHFCTData->cMaxSpectrum);
        #endif
            break;
        }
        case DATA_VERSION_V2:
        {
            CaliCoef coef;
            get_HFCT_cali_coef(&coef);
            for(i=0;i<pstUHFHFCTData->usPhaseNum;i++)
            {
                pstUHFHFCTData->fPrpsDataPerGridPeriod[i] = coef.k * pucAdcVal[i] + coef.b;
                //记录最大原始值索引
                if (pucAdcVal[usMaxdBIndex] < pucAdcVal[i])
                {
                    usMaxdBIndex = i;
                }
            }
            pstUHFHFCTData->fMaxSpectrum = pstUHFHFCTData->fPrpsDataPerGridPeriod[usMaxdBIndex];
            pstUHFHFCTData->eSpectrumState = SPECTRUM_INSIDE_RANGE;
            /**HFCT幅值上下限*/
            pstUHFHFCTData->cAmpLowerLimit = MIN_HFCT_DB_VAL;
            pstUHFHFCTData->cAmpUpperLimit = MAX_HFCT_DB_VAL;
            /**DATA_VERSION_V2 版本HFCT调理器仅支持0dB增益档位*/
            pstUHFHFCTData->cGain = 0;
        #ifdef UHFHFCTAETEV_DEBUG
            printf("HFCT ADC val:(DataVer = %d)\n",cDataVer);
            data_print(pucAdcVal,pstUHFHFCTData->usPhaseNum);
            printf("HFCT DB val:\n");
            float_data_print(pstUHFHFCTData->fPrpsDataPerGridPeriod,pstUHFHFCTData->usPhaseNum);
            printf("Max DB val: %d\n",pstUHFHFCTData->fMaxSpectrum);
        #endif

            break;
        }
        default:
        {

        }
    }
}

/************************************************
 * 函数名   ：parse_hfct_frame

 * 输入参数 ：UINT8 *pucSrcData报文数据

 * 输出参数 ：UHFHFCTData *pstUHFHFCTData解析数据

 * 返回值   ：HC_SUCCESS:成功; HC_FAILURE:失败

 * 功能     ：解析HFCT报文

 * 描述     :
 ************************************************/
static INT32 parse_hfct_frame(UHFHFCTData *pstUHFHFCTData, UINT8 *pucSrcData)
{
    //参数检查
    if (pstUHFHFCTData == NULL || pucSrcData == NULL)
    {
        printf("in parse_hfct_frame pstUHFHFCTData==NULL || pucSrcData==NULL\n");
        return HC_FAILURE;
    }

    //状态字解析
    pstUHFHFCTData->cGain = GET_BIT_RANGE(pucSrcData[STATE_BYTE], BIT0, BIT2) * 20; //衰减
    pstUHFHFCTData->eSyncSource = (SyncSource)(GET_BIT(pucSrcData[STATE_BYTE], BIT3) + 1); //同步源 +1上下枚举变量不同  转换
    pstUHFHFCTData->eSyncState = (SyncState) GET_BIT(pucSrcData[STATE_BYTE], BIT4); //同步状态

    /**TODO: PRPS相位分辨率应支持用户设置，目前固定为60*/
    pstUHFHFCTData->usPhaseNum  = SPECTTRUMNUM;

    calc_hfct_dB_val(&pucSrcData[DATA_BYTE],pstUHFHFCTData);

    return HC_SUCCESS;       
}

static INT32 parse_cable_current_data(CableCurrentData *pstCableData, UINT8 *pucData)
{
    pstCableData->fGroundCurrent = *(float*)(pucData+CMD_RESPONSE_CABEL_GROUD_CURRENT_POS);
    pstCableData->fLoadCurrent   = *(float*)(pucData+CMD_RESPONSE_CABEL_LOAD_CURRENT_POS);
    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：parse_zigbee_conditioner_version

 * 输入参数 ：pVerString

 * 输出参数 ：pcDataVer

 * 返回值   ：null

 * 功能     ：解析获取的zigbee调理器版本号，并根据调理器版本号，确认支持的数据版本

 * 描述     :
 ************************************************/
void parse_zigbee_conditioner_version(UINT8 *pVerString,INT8 *pcDataVer)
{
    UINT8 ucMajorVer,ucMinorVer,ucPatchVer;
    if(!pVerString)
        return;
    printf("zigbee conditioner version:%s\n",pVerString);
    /**
     * pVerString format: "Vx.x.x"
    */
    ucMajorVer = pVerString[1] - '0';
    ucMinorVer = pVerString[2] - '0';
    ucPatchVer = pVerString[3] - '0';
    if(ucMajorVer >= 3)
    {
        *pcDataVer = DATA_VERSION_V2;
    }
    else
    {
        *pcDataVer = DATA_VERSION_V1;
    }
}

/**
 *  获取UHF、HFCT 调理器支持的采集数据版本类型
 *  DATA_VERSION_V1：老版本数据类型
 *  DATA_VERSION_V2：新版本数据类型
 *  输入参数 ：eType   调理器设备类型
 *            pstAddr 调理器设备地址
 *  输出参数 ：version 调理器数据版本号
 *  返回值： HC_SUCCESS ：成功； HC_FAILURE:失败
*/
INT32 get_conditioner_data_version(WLDeviceType eType, WLDeviceAddr *pstAddr,INT8 *version)
{
    INT32 ret = HC_SUCCESS;
    WLDeviceVer ver;
    UINT32 tryCnt = 10;
    while(tryCnt--)
    {
        if((ret=get_wireless_version(&ver,eType,pstAddr))==HC_SUCCESS)
        {
            parse_zigbee_conditioner_version(ver.caVersion,version); 
            break;
        }
       // usleep(100000); /**延时等待100ms 重新获取版本号*/
    }
    if(ret != HC_SUCCESS)
    {
        printf("get zigbee conditioner(Type:%d) data version Failed\n",eType);
    }
    else
    {
        printf("get zigbee conditioner(Type:%d) data version:%d\n",eType,*version);
    }
    
    return ret;
}

/************************************************
 * 
 * 函数名   ：get_uhfhfct_data

 * 输入参数 ：WLDeviceType eDeviceType设备类型

 * 输出参数 ：UHFHFCTData *pstUHFHFCTData数据类

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：读uhfhfct数据

 * 描述     :
 ************************************************/
INT32 get_uhfhfct_data(UHFHFCTData *pstUHFHFCTData, WLDeviceType eDeviceType)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    int i;
    int min = 2;
    int max = 30;
    int maxdB = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;
    int timeuse  = 0;
    static int cnt=0;
    static struct timeval lasttime,currtime;
    gettimeofday(&currtime,NULL);
    timeuse = (currtime.tv_sec-lasttime.tv_sec) *1000000 + currtime.tv_usec-lasttime.tv_usec;
    memcpy(&lasttime,&currtime,sizeof(struct timeval));
    printf("%s is called Timer interval: %d ms\n",__func__,timeuse/1000);
    //static UINT8 ucGetDataVersionSuccess = 0;
    memset(&stFrame, 0, sizeof(stFrame));
    cnt++;
    INT32 (*parse_frame)(UHFHFCTData *, UINT8 *);

    //参数检查
    if (pstUHFHFCTData == NULL)
    {
        printf("get_uhfhfct_data param is wrong pstUHFHFCTData=NULL\n");
        return PARAMETER_ERROR;
    }
    /**根据UHF、HFCT调理器硬件版本确认数据版本类型*/
    if( g_connectedWlDevDataVerArray[eDeviceType] != DATA_VERSION_V1 && 
        g_connectedWlDevDataVerArray[eDeviceType] != DATA_VERSION_V2 )
    {
        if(get_conditioner_data_version(eDeviceType,&gstWlDevAddr[eDeviceType],&g_connectedWlDevDataVerArray[eDeviceType])==HC_FAILURE)
            return HC_FAILURE;
    }
    pstUHFHFCTData->cDataVersion = g_connectedWlDevDataVerArray[eDeviceType];
    
    //根据设备类型 选择命令 解析函数
    switch (eDeviceType)
    {
    case UHF_TERMINAL:
        ucaDataBuf[usDataLen++] = READ_UHF_COMMAND;
        parse_frame = &parse_uhf_frame;
        break;
    case HFCT_TERMINAL:
        ucaDataBuf[usDataLen++] = READ_HFCT_COMMAND;
        parse_frame = &parse_hfct_frame;
        break;
    default:
        return PARAMETER_ERROR;
    }

    //切换数据类型
    if (change_data_type(eDeviceType) != HC_SUCCESS)
    {
        printferr("change data type\n");
        return HC_FAILURE;
    }

    //组读数据命令报文
    stFrame.uiRcvTimeout = TIMEOUT_100MS;

    //发送报文
    iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);

    if (iRet<0) //串口通信错误
    {
        comm_err_print(READ_UHF_COMMAND, (CommErr)(-iRet));
        // return HC_FAILURE;
    }
	pucData = get_frame_data(&stFrame, &usDataLen);


	// //解析报文
	iRet = (*parse_frame)(pstUHFHFCTData, pucData);
    for(i=0;i<60;i++)
    {
        // pstUHFHFCTData->caSpectrum[i] = gstUhfCoefs[0].fK*i + gstUhfCoefs[0].fB + 20;
        // pstUHFHFCTData->caSpectrum[i] = i%10 + 2;
        // // printf(" %d ",pstUHFHFCTData->caSpectrum[i]);
        pstUHFHFCTData->caSpectrum[i] = i%10 + 2*cnt%50;
        // // printf(" %d ",pstUHFHFCTData->caSpectrum[i]);
        // pstUHFHFCTData->caSpectrum[i] = min + rand() % (max - min + 1);
        maxdB = maxdB>pstUHFHFCTData->caSpectrum[i]?maxdB:pstUHFHFCTData->caSpectrum[i];
    }   
    pstUHFHFCTData->cMaxSpectrum = maxdB; 
    pstUHFHFCTData->cMaxSpectrum = maxdB;
    pstUHFHFCTData->cDataVersion = DATA_VERSION_V1;
    pstUHFHFCTData->usPhaseNum = 60;
    pstUHFHFCTData->cAmpLowerLimit= 5;
    pstUHFHFCTData->cAmpUpperLimit = 50;
  #ifdef UHFHFCTAETEV_DEBUG
    printf("DataVer:%d\n",pstUHFHFCTData->cDataVersion);
    printf("cGain:%d\n",pstUHFHFCTData->cGain);
    printf("spectrum state:%d(only for HFCT)\n",pstUHFHFCTData->eSpectrumState);
    printf("upper Limit:%ddB\n",pstUHFHFCTData->cAmpUpperLimit);
    printf("lower Limit:%ddB\n",pstUHFHFCTData->cAmpLowerLimit);
    printf("prps PhaseNum:%d\n",pstUHFHFCTData->usPhaseNum);
  #endif   

    return iRet;
}

/************************************************
 * 函数名   ：get_cable_current_data

 * 输入参数 ：NULL

 * 输出参数 ：CabelCurrentData *pstCableData数据类

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：读电缆电流数据

 * 描述     :
 ************************************************/
INT32 get_cable_current_data(CableCurrentData *pstCableData)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;

    memset(&stFrame, 0, sizeof(stFrame));

    //参数检查
    if (pstCableData == NULL)
    {
        printf("get_cable_current_data param is wrong pstCableData=NULL\n");
        return PARAMETER_ERROR;
    }


    //切换数据类型
    if (change_data_type(CABLE_CURRENT_TERMINAL) != HC_SUCCESS)
    {
        printferr("change data type\n");
        return HC_FAILURE;
    }

    //组读数据命令报文
    stFrame.uiRcvTimeout = TIMEOUT_100MS;
    ucaDataBuf[usDataLen++] = READ_CABLE_CURRENT_COMMAND;
    //发送报文
    iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);

    if (iRet<0) //串口通信错误
    {
        comm_err_print(READ_CABLE_CURRENT_COMMAND, (CommErr)(-iRet));
        return HC_FAILURE;
    }
	pucData = get_frame_data(&stFrame, &usDataLen);


	//解析报文
	iRet = parse_cable_current_data(pstCableData,pucData);


    return iRet;
}
/************************************************
 * 函数名   ：start_addr_find

 * 输入参数 ：WLDeviceType eDeviceType设备类型

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：开始枚举设备

 * 描述     : 只有UHF HFCT 同步器用到
 ************************************************/
static INT32 start_addr_find(WLDeviceType eDeviceType)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 command;

    memset(&stFrame, 0, sizeof(stFrame));

    //根据设备类型 选择命令
    switch (eDeviceType)
    {
    case UHF_TERMINAL:
        ucaDataBuf[usDataLen++] = START_UHF_ADDR;
        break;
    case HFCT_TERMINAL:
        ucaDataBuf[usDataLen++] = START_HFCT_ADDR;
        break;
    case CABLE_CURRENT_TERMINAL:
        ucaDataBuf[usDataLen++] = ENUMERATE_CABLE_CURRENT_ADDR;
        break;
    case SYNCHRONIZER:
        ucaDataBuf[usDataLen++] = START_SYNCER_ADDR;
        break;
#ifdef AETOCONDITIONER
        case AE_TERMINAL:
        ucaDataBuf[usDataLen++] = START_AE_ADDR;
        break;
#endif
    default:
        return PARAMETER_ERROR;
    }
    command = ucaDataBuf[COMMAND_BYTE];

    //组命令
    stFrame.uiRcvTimeout = TIMEOUT_100MS;

    //发送报文
    iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(command, (CommErr)(-iRet));
        return HC_FAILURE;
    }
    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：get_first_addrlist

 * 输入参数 ：WLDeviceType eDeviceType设备类型

 * 输出参数 ：WLDeviceAddr *pstAddr 设备地址列表

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：获取第一个地址

 * 描述     :
 ************************************************/
INT32 get_first_addrlist(WLDeviceAddr *pstAddr, WLDeviceType eDeviceType)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;
    UINT8 command;

    //参数检查
    if (pstAddr == NULL)
    {
        printf("get_first_addrlist param is wrong pstAddr=%x\n", pstAddr);
        return PARAMETER_ERROR;
    }

    //根据设备类型 选择命令
    switch (eDeviceType)
    {
    case UHF_TERMINAL:
        ucaDataBuf[usDataLen++] = FIRST_UHF_ADDR;
        break;
    case HFCT_TERMINAL:
        ucaDataBuf[usDataLen++] = FIRST_HFCT_ADDR;
        break;
    case SYNCHRONIZER:
        ucaDataBuf[usDataLen++] = FIRST_SYNCER_ADDR;
        break;
    case CABLE_CURRENT_TERMINAL:
        ucaDataBuf[usDataLen++] = GET_FIRST_CABLE_CURRENT_ADDR;
        break;
#ifdef AETOCONDITIONER
        case AE_TERMINAL:
        ucaDataBuf[usDataLen++] = FIRST_AE_ADDR;
        break;
#endif
    default:
        return PARAMETER_ERROR;
    }
    command = ucaDataBuf[COMMAND_BYTE];

    //组命令
    stFrame.uiRcvTimeout = TIMEOUT_100MS;

    //发送报文
    iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(command, (CommErr)(-iRet));
        return HC_FAILURE;
    }

    pucData = get_frame_data(&stFrame, &usDataLen);

    //保存设备地址 状态
    memcpy(pstAddr->caAddr, pucData + ADDR_BYTE, ADDRBYTENUM); //地址
    pstAddr->eAddrState = (AddrState) pucData[ADDR_STATE_BYTE]; //地址状态

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：get_next_addrlist

 * 输入参数 ：WLDeviceType eDeviceType设备类型

 * 输出参数 ：WLDeviceAddr *pstAddr 设备地址列表

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：获取下一个地址

 * 描述     :
 ************************************************/
INT32 get_next_addrlist(WLDeviceAddr *pstAddr, WLDeviceType eDeviceType)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;
    UINT8 command;

    //参数检查
    if (pstAddr == NULL)
    {
        printf("get_next_addrlist param is wrong pstAddr=%x\n", pstAddr);
        return PARAMETER_ERROR;
    }

    //根据设备类型 选择命令
    switch (eDeviceType)
    {
    case UHF_TERMINAL:
        ucaDataBuf[usDataLen++] = NEXT_UHF_ADDR;
        break;
    case HFCT_TERMINAL:
        ucaDataBuf[usDataLen++] = NEXT_HFCT_ADDR;
        break;
    case SYNCHRONIZER:
        ucaDataBuf[usDataLen++] = NEXT_SYNCER_ADDR;
        break;
    case CABLE_CURRENT_TERMINAL:
        ucaDataBuf[usDataLen++] = GET_NEXT_CABLE_CURRENT_ADDR;
        break;
#ifdef AETOCONDITIONER
        case AE_TERMINAL:
        ucaDataBuf[usDataLen++] = NEXT_AE_ADDR;
        break;
#endif
    default:
        return PARAMETER_ERROR;
    }
    command = ucaDataBuf[COMMAND_BYTE];

    //组命令
    stFrame.uiRcvTimeout = TIMEOUT_100MS;

    //发送报文
    iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(command, (CommErr)(-iRet));
        return HC_FAILURE;
    }

    pucData = get_frame_data(&stFrame, &usDataLen);
    //保存设备地址 状态
    memcpy(pstAddr->caAddr, pucData + ADDR_BYTE, ADDRBYTENUM); //地址
    pstAddr->eAddrState = (AddrState) pucData[ADDR_STATE_BYTE]; //地址状态

    return HC_SUCCESS;
}

/************************************************
 * 功能       ： 开始搜索无线设备
 * 描述       ： 开始搜索后，等待10秒，调get_wireless_addrlist获取无线设备地址
 *        针对设备：UHF、HFCT、同步器、超声波(保留超声波接口、但一般没有超声波)
 * 入参       ：
 *          eDeviceType 设备类型
 * 返回值   ：
 *          0    ：成功
 *          else ：失败
 ************************************************/
#define GETADDRRETRYNUM  (10)
INT32 start_search_wireless(WLDeviceType eDeviceType)
{
    UINT8 ucRetry = 0;

    //发送开始枚举设备命令
    for (ucRetry = 0; ucRetry < GETADDRRETRYNUM; ucRetry++)
    {
        if (start_addr_find(eDeviceType) != HC_SUCCESS)
        {
            continue;
        }
        ucRetry = 0;
        break;
    }

    //失败
    if (ucRetry == GETADDRRETRYNUM)
    {
        printferr("\n");
        return HC_FAILURE;
    }

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：get_wireless_addrlist

 * 输入参数 ：WLDeviceType eDeviceType设备类型

 * 输出参数 ：WLDeviceAddr *pstAddr 设备地址列表 , UINT16 *usAddrNum 设备个数

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：枚举设备地址

 * 描述     :
 ************************************************/
INT32 get_wireless_addrlist(WLDeviceAddr *pstAddr, UINT16 *pusAddrNum, WLDeviceType eDeviceType)
{
    WLDeviceAddr *pstDevAddr = pstAddr;

    //参数检查
    if (pstDevAddr == NULL || pusAddrNum == NULL)
    {
        printf("get_wireless_addrlist param is wrong pstAddr=%x pusAddrNum=%d\n", pstDevAddr, pusAddrNum);
        return PARAMETER_ERROR;
    }
    *pusAddrNum = 0;

    //获取第一个地址
    if (get_first_addrlist(pstDevAddr, eDeviceType) != HC_SUCCESS)
    {
        printferr("\n");
        return HC_FAILURE;
    }
    UHFHFCTAETEV_dbg("a****************0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x  state=%d\n", pstDevAddr->caAddr[0], pstDevAddr->caAddr[1],
            pstDevAddr->caAddr[2], pstDevAddr->caAddr[3], pstDevAddr->caAddr[4], pstDevAddr->caAddr[5], pstDevAddr->caAddr[6], pstDevAddr->caAddr[7],
            pstDevAddr->eAddrState);
    pstDevAddr++;
    *pusAddrNum = 1; //地址个数
    
    //获取剩余地址
    while (get_next_addrlist(pstDevAddr, eDeviceType) == HC_SUCCESS && *pusAddrNum < MAXWIRELESSDEVICENUM)
    {
        UHFHFCTAETEV_dbg("b****************0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x  state=%d\n", pstDevAddr->caAddr[0], pstDevAddr->caAddr[1],
                pstDevAddr->caAddr[2], pstDevAddr->caAddr[3], pstDevAddr->caAddr[4], pstDevAddr->caAddr[5], pstDevAddr->caAddr[6], pstDevAddr->caAddr[7],
                pstDevAddr->eAddrState);
        pstDevAddr++;
        (*pusAddrNum) += 1;
    }

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：get_wireless_version

 * 输入参数 ：WLDeviceAddr *pstAddr设备地址

 * 输出参数 ：WLDeviceVer *pstWLDeviceVer设备版本

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：获取设备版本

 * 描述     : WLDeviceAddr *pstAddr为设备地址,接收器时该参数为NULL
 ************************************************/
INT32 get_wireless_version(WLDeviceVer *pstWLDeviceVer, WLDeviceType eDeviceType, WLDeviceAddr *pstAddr)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;

    //参数检查
    if (pstWLDeviceVer == NULL)
    {
        printf("get_wireless_version param is wrong pstWLDeviceVer=%x\n", pstWLDeviceVer);
        return PARAMETER_ERROR;
    }

    //根据设备类型 选择命令
    if (pstAddr == NULL) //接收器
    {
        ucaDataBuf[usDataLen++] = RECEIVER_VERSION;
        stFrame.uiRcvTimeout = TIMEOUT_100MS;
    }
    else //外部设备
    {
        ucaDataBuf[usDataLen++] = EXTERN_DEV_VERSION;
        ucaDataBuf[usDataLen++] = 0;
        memcpy(ucaDataBuf + usDataLen, pstAddr->caAddr, ADDRBYTENUM);
        usDataLen += ADDRBYTENUM;
        stFrame.uiRcvTimeout = TIMEOUT_200MS;
    }

    //发送报文
    iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(ucaDataBuf[0], (CommErr)(-iRet));
        return HC_FAILURE;
    }
    pucData = get_frame_data(&stFrame, &usDataLen);

    //保存版本信息
    memset(pstWLDeviceVer->caVersion, 0, VERSIONBYTENUM);
    memcpy(pstWLDeviceVer->caVersion, pucData + 2, usDataLen - 2); //地址
    pstWLDeviceVer->ucVerLen = usDataLen - 2;

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：set_uhf_filter

 * 输入参数 ：UHFFilterControl eFilterControl

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：UHF滤波设置

 * 描述     : 即是带宽设置
 ************************************************/
INT32 set_uhf_filter(UHFFilterControl eFilterControl)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;

    //组命令
    ucaDataBuf[usDataLen++] = CONTROL_CMMAND; //命令
    ucaDataBuf[usDataLen++] = UHF_FILTER_CMMAND; //子命令
    ucaDataBuf[usDataLen] = (UINT8) eFilterControl; //数据

    if (eFilterControl == LOW_PASS_FILTER) //上下枚举变量不同  转换
    {
        ucaDataBuf[usDataLen] = 0x02;
    }
    else if (eFilterControl == HIGH_PASS_FILTER)
    {
        ucaDataBuf[usDataLen] = 0x01;
    }

    //更新本地缓存的参数,用于报文验证
    s_stUHFCTParam.cUHFFilter = (INT8)eFilterControl;
    UHFHFCTAETEV_dbg("cache uhf param : filter = %d\n",s_stUHFCTParam.cUHFFilter);

    usDataLen += 1;
    stFrame.uiRcvTimeout = TIMEOUT_500MS;

    //发送报文
    iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(CONTROL_CMMAND, (CommErr)(-iRet));
        return HC_FAILURE;
    }
    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：set_uhfhfct_gain

 * 输入参数 ：UHFHFCTGain eUHFHFCTGain增益,WLDeviceType eDeviceType设备类型

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：设置增益

 * 描述     :
 ************************************************/
INT32 set_uhfhfct_gain(UHFHFCTGain eUHFHFCTGain, WLDeviceType eDeviceType)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;

    //参数检查
    if ((eDeviceType == UHF_TERMINAL && eUHFHFCTGain > (UHFHFCTGain) 1) || (eDeviceType == HFCT_TERMINAL && eUHFHFCTGain < (UHFHFCTGain) 2)
            || eDeviceType > HFCT_TERMINAL)
    {
        printf("set_uhfhfct_gain param is wrong eUHFHFCTGain=%d WLDeviceType=%d\n", (UINT8) eUHFHFCTGain, (UINT8) eDeviceType);
        return PARAMETER_ERROR;
    }

    ucaDataBuf[usDataLen++] = CONTROL_CMMAND;
    //根据设备类型 选择命令
    if (eDeviceType == UHF_TERMINAL) //UHF
    {
        ucaDataBuf[usDataLen++] = UHF_GAIN_CMMAND;
        ucaDataBuf[usDataLen++] = (UINT8) eUHFHFCTGain;
        //更新本地缓存的参数,用于报文验证
        s_stUHFCTParam.cUHFGain = (INT8)eUHFHFCTGain;
        UHFHFCTAETEV_dbg("cache uhf param : gain = %d\n",s_stUHFCTParam.cUHFGain);
    }
    else //HFCT
    {
        ucaDataBuf[usDataLen++] = HFCT_GAIN_CMMAND;
        ucaDataBuf[usDataLen++] = (UINT8) eUHFHFCTGain - 2; //HFCT衰减类型
        //更新本地缓存的参数,用于报文验证
        s_stUHFCTParam.cHFCTGain = (INT8)eUHFHFCTGain;
        UHFHFCTAETEV_dbg("cache ct param : gain = %d\n",s_stUHFCTParam.cHFCTGain);
    }

    //组命令
    stFrame.uiRcvTimeout = TIMEOUT_500MS;

    //发送报文
    iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(CONTROL_CMMAND, (CommErr)(-iRet));
        return HC_FAILURE;
    }
    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：set_wireless_syncsource

 * 输入参数 ：UHFHFCTSyncSource eUHFHFCTSync同步源,WLDeviceType eDeviceType设备类型

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：设置同步源

 * 描述     : 设置UHF、HFCT、AE同步
 ************************************************/
INT32 set_wireless_syncsource(SyncSource eUHFHFCTSync, WLDeviceType eDeviceType)
{
    FrameBuf stFrame;
    INT32 usDataLen = 0;
    UINT8 ucaDataBuf[10];
    INT32 iRet = 0;

    ucaDataBuf[usDataLen++] = CONTROL_CMMAND; //命令
    //根据设备类型 选择命令
    switch (eDeviceType)
    {
    case UHF_TERMINAL:
        ucaDataBuf[usDataLen++] = UHF_SYNC_CMMAND;
        break;
    case HFCT_TERMINAL:
        ucaDataBuf[usDataLen++] = HFCT_SYNC_CMMAND;
        break;
    case AE_TERMINAL:
        ucaDataBuf[usDataLen++] = AE_SYNC_CMMAND;
        break;
    default:
        return PARAMETER_ERROR;
    }

    //组命令
    ucaDataBuf[usDataLen++] = (UINT8) eUHFHFCTSync - 1; //数据  -1上下枚举变量不同  转换
    stFrame.uiRcvTimeout = TIMEOUT_100MS;

    //发送报文
    iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(CONTROL_CMMAND, (CommErr)(-iRet));
        return HC_FAILURE;
    }
    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：set_wireless_frequency

 * 输入参数 ：Frequency eFrequency电网频率

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：设置UHF HFCT电网频率

 * 描述     :
 ************************************************/
static INT32 set_wireless_frequency(Frequency eFrequency)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;

    //组命令
    ucaDataBuf[usDataLen++] = SET_FREQUENCE;
    ucaDataBuf[usDataLen++] = (UINT8) eFrequency;
    stFrame.uiRcvTimeout = TIMEOUT_100MS;

    //发送报文
    iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(SET_FREQUENCE, (CommErr)(-iRet));
        return HC_FAILURE;
    }
    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：set_wireless_addrlist

 * 输入参数 ：WLDeviceAddr *pstAddr设备地址,WLDeviceType eDeviceType设备类型

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：设置设备地址

 * 描述     :
 ************************************************/
INT32 set_wireless_addrlist(WLDeviceAddr *pstAddr, WLDeviceType eDeviceType)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;
    UINT8 command;


    //参数检查
    if (pstAddr == NULL)
    {
        printf("set_wireless_addrlist param is wrong pstAddr==NULL\n");
        return PARAMETER_ERROR;
    }

    /**记录主机设置的调理器地址*/
    memcpy(&gstWlDevAddr[eDeviceType],pstAddr,sizeof(WLDeviceAddr));
    //根据设备类型 选择命令
    switch (eDeviceType)
    {
    case UHF_TERMINAL:
        ucaDataBuf[usDataLen++] = SET_UHF_ADDR;
        s_stUHFAddr.bAddrSet = true;
        memcpy( s_stUHFAddr.stAddr.caAddr, pstAddr, ADDRBYTENUM );
        break;
    case HFCT_TERMINAL:
        ucaDataBuf[usDataLen++] = SET_HFCT_ADDR;
        s_stHFCTAddr.bAddrSet = true;
        memcpy( s_stHFCTAddr.stAddr.caAddr, pstAddr, ADDRBYTENUM );
        break;
    case SYNCHRONIZER:
        ucaDataBuf[usDataLen++] = SET_SYNCER_ADDR;
        break;
    case CABLE_CURRENT_TERMINAL:
        ucaDataBuf[usDataLen++] = SET_CABLE_CURRENT_ADDR;
#ifdef AETOCONDITIONER
        case AE_TERMINAL:
        ucaDataBuf[usDataLen++] = SET_AE_ADDR;
        break;
#endif
    default:
        return PARAMETER_ERROR;
    }
    command = ucaDataBuf[COMMAND_BYTE];

    //组命令
    memcpy(ucaDataBuf + 1, pstAddr->caAddr, ADDRBYTENUM);
    usDataLen += ADDRBYTENUM;
    stFrame.uiRcvTimeout = TIMEOUT_100MS;

    //发送报文
    iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(CONTROL_CMMAND, (CommErr)(-iRet));
        return HC_FAILURE;
    }
   
   g_connectedWlDevDataVerArray[eDeviceType] = INVALI_DATA_VER;
   iRet = get_conditioner_data_version(eDeviceType,pstAddr,&g_connectedWlDevDataVerArray[eDeviceType]);
   return iRet;
}

/************************************************
 * 函数名   ：set_aetev_frequency

 * 输入参数 ：Frequency eFrequency电网频率

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：设置AE TEV电网频率

 * 描述     :
 ************************************************/
static INT32 set_aetev_frequency(Frequency eFrequency)
{
    FrameBuf stFrame;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    INT32 iRet = 0;

    //组命令
    ucaDataBuf[usDataLen++] = AETEV_FREQUENCE;
    ucaDataBuf[usDataLen++] = (UINT8) eFrequency;
    stFrame.uiRcvTimeout = TIMEOUT_100MS;

    //发送报文
    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(AETEV_FREQUENCE, (CommErr)(-iRet));
        return HC_FAILURE;
    }

    return HC_SUCCESS;
}

/************************************************
 * 功能     ：从增益获取增益等级
 *
 * 描述     : 增益等级为0,1,2,分别对应60,80,100dB
 * 
 * 输入参数 ：fGain -- 增益
 *
 * 返回值   ：增益等级
 ************************************************/
UINT8 get_gain_level_from_gain( float fGain )
{
    UINT8 ucGainLevel = 0;

    if( fabs(fGain - AE_GAIN100) < 0.00001 )
    {
        ucGainLevel = AE_GAIN_LEVEL_100;
    }
    else if( fabs(fGain - AE_GAIN80) < 0.00001 )
    {
        ucGainLevel = AE_GAIN_LEVEL_80;
    }
    else if( fabs(fGain - AE_GAIN60) < 0.00001 )
    {
        ucGainLevel = AE_GAIN_LEVEL_60;
    }
    else
    {
        ucGainLevel = AE_GAIN_LEVEL_INVALID;
    }

    return ucGainLevel;
}

/************************************************
 * 功能     ：从增益等级获取增益
 *
 * 描述     : 增益等级为0,1,2,分别对应60,80,100dB
 * 
 * 输入参数 : ucGainLevel -- 增益等级
 *
 * 返回值   ：增益
 ************************************************/
float get_gain_from_gain_level( UINT8 ucGainLevel )
{
    float fGain = 0.0;

    if( AE_GAIN_LEVEL_100 == ucGainLevel )
    {
        fGain = AE_GAIN100;
    }
    else if( AE_GAIN_LEVEL_80 == ucGainLevel )
    {
        fGain = AE_GAIN80;
    }
    else if( AE_GAIN_LEVEL_60 == ucGainLevel )
    {
        fGain = AE_GAIN60;
    }
    else
    {
        fGain = AE_GAIN100;
    }

    return fGain;
}

/************************************************
 * 函数名   ：set_ae_settings

 * 输入参数 ：AEWorkSettings *pstWorkSettings工作参数

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：设置超声波工作参数

 * 描述     :
 ************************************************/
#define WIRELESS_AE_MULTI_VIEW_WAVE_GRID_COUNT  (5)
INT32 set_ae_settings(AEWorkSetForUp *pstAEWorkSetForUp, AE_CHANNEL eAE_CHANNEL)
{
    FrameBuf stFrame;
    UINT8 ucDataBuf[1024];
    UINT16 usDataLen = 0;
    INT32 iRet = 0;

    //参数检查
    if (pstAEWorkSetForUp == NULL)
    {
        printf("in set_ae_settings pstAEWorkSetForUp==NULL \n");
        return HC_FAILURE;
    }

    // printf("----------set ae param, channel: %d\n",eAE_CHANNEL);
    // printf("work mode:%d\nunit:%d\ngain:%f\nsync source:%d\nfcomponent:%d\nngrid:%d\namp thd:%f\nprpd thd:%f\npulse thd:%f\nwave thd:%f\nprpd blocking:%d\npulse blocking:%d\npulse gating:%d\nfilter:%d\n",\
    // pstAEWorkSetForUp->eWorkMode,pstAEWorkSetForUp->eAE_Unit,pstAEWorkSetForUp->fGain,pstAEWorkSetForUp->eSyncSource,pstAEWorkSetForUp->usFComponent,pstAEWorkSetForUp->ucNgrid,pstAEWorkSetForUp->fAmplitudeThd,pstAEWorkSetForUp->fPrpdThd,\
    // pstAEWorkSetForUp->fPulseThd,pstAEWorkSetForUp->fWaveThd,pstAEWorkSetForUp->usPrpdBlocking,pstAEWorkSetForUp->usPulseBlocking,pstAEWorkSetForUp->usPulseGating,pstAEWorkSetForUp->eAEFilter);

#ifdef WIRELESS_AE_TEST
    WLDeviceAddr stAddr = {
    {0xC1, 0x91, 0x54, 0x09, 0x00, 0x4B, 0x12, 0x00},
    0
    };
    set_wireless_addrlist(&stAddr, AE_TERMINAL);
    usleep(100000);

    eAE_CHANNEL = AE_Wireless;
#endif

    //非无线传感器情况下,向STM32下发参数
    if( AE_Wireless != eAE_CHANNEL )
    {
        AEWorkSettings stAEWorkSettings;//对STM32的AE工作参数结构体

        //组数据
        stAEWorkSettings.ucWorkMode = (UINT8) pstAEWorkSetForUp->eWorkMode;
        stAEWorkSettings.ucUnit = (UINT8) pstAEWorkSetForUp->eAE_Unit;
        stAEWorkSettings.fGain = pstAEWorkSetForUp->fGain;
        stAEWorkSettings.ucSyncSource = (UINT8) pstAEWorkSetForUp->eSyncSource;
        stAEWorkSettings.usFComponent = pstAEWorkSetForUp->usFComponent;
        stAEWorkSettings.ucNgrid = pstAEWorkSetForUp->ucNgrid;
        stAEWorkSettings.fAmplitudeThd = pstAEWorkSetForUp->fAmplitudeThd;
        stAEWorkSettings.fPrpdThd = pstAEWorkSetForUp->fPrpdThd;
        stAEWorkSettings.fPulseThd = pstAEWorkSetForUp->fPulseThd;
        stAEWorkSettings.fWaveThd = pstAEWorkSetForUp->fWaveThd;
        stAEWorkSettings.usPrpdBlocking = pstAEWorkSetForUp->usPrpdBlocking;
        stAEWorkSettings.usPulseBlocking = pstAEWorkSetForUp->usPulseBlocking;
        stAEWorkSettings.usPulseGating = pstAEWorkSetForUp->usPulseGating;
#ifdef AE_FILTER_ENABLE
        stAEWorkSettings.ucAEFilter = (UINT8)pstAEWorkSetForUp->eAEFilter;//AE带宽模式
#endif

        //组命令
        ucDataBuf[usDataLen++] = AE_SETTINGS; //命令
        memcpy(&ucDataBuf[usDataLen], &stAEWorkSettings, sizeof(AEWorkSettings)); //数据
        usDataLen += sizeof(AEWorkSettings);
        stFrame.uiRcvTimeout = TIMEOUT_100MS;

        //发送报文
        iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucDataBuf, usDataLen);
        if (iRet != COM_OK)
        {
            comm_err_print(AE_SETTINGS, (CommErr)(-iRet));
            return HC_FAILURE;
        }

        //保存超声波工作模式
        g_eWorkMode = pstAEWorkSetForUp->eWorkMode;
    }
    //无线传感器情况下,向ZIGBEE下发参数
    else
    {
        WirelessAEParam stParam;//对ZIGBEE的AE工作参数结构体

        //无线AE的多图谱采集以波形命令下发,波形周期数固定
        if( pstAEWorkSetForUp->eWorkMode == AllViews )
        {
            pstAEWorkSetForUp->ucNgrid = WIRELESS_AE_MULTI_VIEW_WAVE_GRID_COUNT;
            pstAEWorkSetForUp->eWorkMode = Waveform;
        }

        //保存无线传感器参数,用于对数据中的参数进行验证
        memcpy( &s_stAeWirelessWorkParam, pstAEWorkSetForUp, sizeof(s_stAeWirelessWorkParam) );

        //组数据
        stParam.ucWorkMode = (UINT8) pstAEWorkSetForUp->eWorkMode;
        stParam.ucGain = get_gain_level_from_gain(pstAEWorkSetForUp->fGain);
#ifdef T90_WIRELESS_AE
        stParam.ucFComponent = pstAEWorkSetForUp->usFComponent;
#else
        stParam.usFComponent = pstAEWorkSetForUp->usFComponent;
#endif
        stParam.ucNgrid = pstAEWorkSetForUp->ucNgrid;
        
        stParam.usPrpdThreshold = (UINT16) wireless_ae_mv_to_adc( pstAEWorkSetForUp->fPrpdThd, stParam.ucGain );
        stParam.usPrpdBlocking = pstAEWorkSetForUp->usPrpdBlocking;
        stParam.usPulseBlocking = pstAEWorkSetForUp->usPulseBlocking;
        stParam.usPulseThreshold = (UINT16)wireless_ae_mv_to_adc( pstAEWorkSetForUp->fPulseThd, stParam.ucGain );
        stParam.ulPulseGating = (unsigned long)pstAEWorkSetForUp->usPulseGating;

        g_eAEWirelessUnit = pstAEWorkSetForUp->eAE_Unit;

#ifdef WIRELESS_AE_TEST

        printf("-----------param input---------------------\n");
        printf("work mode = %d, unit = %d, gain = %f, SyncSource = %d, FComponent = %d, ngrid = %d, prpd threshold = %f, prpd blocking = %d, pulse blocking = %d, \
        pulse threshold = %f, pulse gateing = %d\n",pstAEWorkSetForUp->eWorkMode, pstAEWorkSetForUp->eAE_Unit, pstAEWorkSetForUp->fGain, pstAEWorkSetForUp->eSyncSource, pstAEWorkSetForUp->usFComponent, pstAEWorkSetForUp->ucNgrid, \
        pstAEWorkSetForUp->fPrpdThd, pstAEWorkSetForUp->usPrpdBlocking, pstAEWorkSetForUp->usPulseBlocking, pstAEWorkSetForUp->fPulseThd, pstAEWorkSetForUp->usPulseGating);

#endif

        //组命令
        ucDataBuf[usDataLen++] = AE_SETTINGS; //命令
        memcpy(&ucDataBuf[usDataLen], &stParam, sizeof(stParam)); //数据
        usDataLen += sizeof(stParam);
        stFrame.uiRcvTimeout = TIMEOUT_500MS;
        //发送报文
        iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucDataBuf, usDataLen);
        if (iRet != COM_OK)
        {
            comm_err_print(AE_SETTINGS, (CommErr)(-iRet));
            return HC_FAILURE;
        }
        //保存超声波工作模式
        g_eWorkMode = pstAEWorkSetForUp->eWorkMode;
    }

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：adc_driver_value210xuv

 * 输入参数 ：INT16 ：adc值 (无637检波电路采集，ADC码值可能为负值) chOplevel：增益等级

 * 输出参数 ：FrameTransfrom *pstFrameTransfrom报文通讯类指针

 * 返回值   ：0.1mV float

 * 功能     ： ADC 采样值转换为0.1mV

 * 描述     :
 ************************************************/
static float ae_adtomv(INT16 sValue, float fGain)
{
    static const INT32 iaOptlb[] =
    { AE_GAIN100, AE_GAIN80, AE_GAIN60 };
    UINT8 ucIndex = 0;

    //选择对应的放大倍数
    if (fGain == AE_GAIN60)
    {
        ucIndex = 0;
    }
    else if (fGain == AE_GAIN80)
    {
        ucIndex = 1;
    }
    else if (fGain == AE_GAIN100)
    {
        ucIndex = 2;
    }
    else
    {
        printf("fGain is wrong fGain=%f\n", fGain);
        return 0;
    }

    //计算实际值
    return ((sValue * 708 * iaOptlb[ucIndex] * 1.1 / 8192 / 10));
}

/************************************************
 * 函数名   ：wireless_ae_adc_to_mv
 ************************************************/
static float wireless_ae_adc_to_mv( UINT16 usData, UINT8 ucGainLevel )
{
    static int iCoefficient[] = {100, 10, 1};

    if( ucGainLevel > AE_GAIN_LEVEL_100)
    {
        return 0;
    }

    return ( (float)usData * WIRELESSAE_COEFFICIENT * 71 * iCoefficient[ucGainLevel] / 8192 );
}

/************************************************
 * 函数名   ：wireless_ae_mv_to_adc
 ************************************************/
static int wireless_ae_mv_to_adc( float fData, UINT8 ucGainLevel )
{
    static int iCoefficient[] = {1, 10, 100};

    if( ucGainLevel > AE_GAIN_LEVEL_100)
    {
        return 0;
    }

    return ( fData * 296 * iCoefficient[ucGainLevel] / 256 / WIRELESSAE_COEFFICIENT );
}

/************************************************
 * 函数名   ：wireless_ae_mv_to_db
 * 功能:    mv值转为dB值
 * 说明:    当mv值为0时,dB值置为-15dB
 *          其他情况下dB值为20log10(x)
 ************************************************/
static float wireless_ae_mv_to_db( float fMv )
{
    float fDB = 0.0;
    if( fabs(fMv) > 0.00000000001 )
    {
        fDB = 20*log10( fMv );
    }
    else
    {
        fDB = -15;//DB模式的最小值
    }

    return fDB;
}

/************************************************
 * 功能:    判断无线ae参数是否匹配
 * 说明:    需检测工作模式和增益,对相位和波形模式还需要检查同步源
 * 入参:    
 *          pMsgParam -- ae无线数据报文中返回的参数
 *          pLocalParam -- 本地保存的ae参数
 ************************************************/
static bool is_wireless_ae_param_match( AEWirelessMsgParam* pMsgParam, AEWorkSetForUp* pLocalParam )
{
    bool bParamMatch = true;
    SyncSource eWirelessAESyncSource = WIRELESS_SYNC;

    if(( pMsgParam->ucWorkMode != (UINT8)pLocalParam->eWorkMode )
    || ( pMsgParam->ucGainLevel != get_gain_level_from_gain(pLocalParam->fGain)))
    {
        UHFHFCTAETEV_dbg("Wireless Param Mismatch: msg work mode = %d, local work mode = %d, msg gain level = %d, local gain level = %d\n",\
        pMsgParam->ucWorkMode, pLocalParam->eWorkMode, pMsgParam->ucGainLevel, get_gain_level_from_gain(pLocalParam->fGain));
        bParamMatch = false;
    }

    if(( pLocalParam->eWorkMode == PRPD )
     ||( pLocalParam->eWorkMode == Waveform ))
    {
        eWirelessAESyncSource = (pMsgParam->ucSyncSource == 0) ? WIRELESS_SYNC : INTER_SYNC;//无线AE返回值中,0代表电源同步,1代表内同步
        if( eWirelessAESyncSource != (UINT8)pLocalParam->eSyncSource )
        {
            UHFHFCTAETEV_dbg("Wireless Param Mismatch: msg sync source = %d, local sync source = %d\n",
            eWirelessAESyncSource, pLocalParam->eSyncSource);
            bParamMatch = false;
        }
    }

    return bParamMatch;
}

/************************************************
 * 功能     ：获取无线AE采样率
 * 
 * 输入参数 : eWorkMode -- 工作模式
 *
 * 返回值   ：sample rate
 ************************************************/
#define  AE_WIRELESS_DEFAULT_SAMPLE_RATE    (2560)
static float get_wireless_sample_rate( WorkMode eWorkMode )
{
    if( Amplitude == eWorkMode || PRPD == eWorkMode || Pulse == eWorkMode )
    {
        return AE_WIRELESS_DEFAULT_SAMPLE_RATE;
    }
    else if( Waveform == eWorkMode )
    {
        return (float)AEWAVENUM * 1000 / (s_stAeWirelessWorkParam.ucNgrid * 1000 / GRID_FREQUENCY);
    }
}

/************************************************
 * 功能     ：解析无线AE发送的数据
 * 
 * 输入参数 ：pAeData -- AE无线数据缓存
 * 
 *           usDataLen -- AE数据长度
 *
 * 输出参数 ：pstAEReadData -- 保存解析后数据的结构体
 *
 * 返回值   ：0:succeed; negtive:failed
 ************************************************/
static INT32 parse_wireless_ae_message( UINT8* pAeData, UINT16 usDataLen, AEReadData* pstAEReadData )
{
    AEWirelessMsgParam stWirelessMsgParam;//无线调理器返回的参数定义
    CommErr eRet = COM_OK;

    // struct tm* t;
    // time_t lTime;

    // time(&lTime);
    // t = localtime( &lTime );
    // printf("local time in parse ae wireless msg: %4d%02d%02d %02d:%02d:%02d\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);  

    //入参检查
    if(( NULL == pAeData )||( 0 == usDataLen )||( NULL == pstAEReadData ))
    {
        printf("error in [%s]: input error!\n",__func__);
        return -COM_INPUTPARAM_ERR;
    }

    UINT8 ucCommand = pAeData[POS_WIRELESSAE_CMD];//命令字
    if( ucCommand != AE_READ_DATA )
    {
        printf("error in [%s]: command byte mismatch!\n",__func__);
        return -COM_COMMAND_BYTE_ERR;
    }

    UINT8 ucDataStatus = pAeData[POS_WIRELESSAE_STATUS];//数据状态
    if( !ucDataStatus )
    {
        printf("error in [%s]: data invalid!\n",__func__);
        return -COM_RESPONSE_STATE_ERR;
    }

    //填写数据前的参数

    //无线传感器参数,包括同步源,同步状态,增益
    //                       ↓  ↓增益
    //  0  0  0  0     0  0  0  0
    //           ↑同步状态 ↑同步源
    UINT8 ucParamBit = pAeData[POS_WIRELESSAE_PARAM];

    UINT8 ucWorkMode = pAeData[POS_WIRELESSAE_WORK_MODE];//无线传感器工作模式

    UINT8 ucGainLevel = ucParamBit & 0x03; //增益等级定义参见UHFHFCTAETEVApi.h的AEGainLevel枚举
    UINT8 ucSyncSource = ucParamBit & 0x08;//无线AE调理器中,该位为0:电源同步,否则:内同步
    UINT8 ucSyncStatus = (ucParamBit & 0x10) >> 4;//同步状态定义参见UHFHFCTAETEVApi.h的SyncState枚举

    stWirelessMsgParam.ucWorkMode = ucWorkMode;
    stWirelessMsgParam.ucGainLevel = ucGainLevel;
    stWirelessMsgParam.ucSyncSource = ucSyncSource;

    //检查数据中的参数和本地的参数是否匹配,如果不匹配则需要重新下发
    //该逻辑是为了1.避免AE调理器关闭重启后运行参数重置;2.规避向调理器下发参数失败的风险
    if( is_wireless_ae_param_match( &stWirelessMsgParam, &s_stAeWirelessWorkParam ) )
    {
        pstAEReadData->eSyncState = (SyncState) ucSyncStatus;
        pstAEReadData->eSyncSource = (ucSyncSource == 0) ? WIRELESS_SYNC : INTER_SYNC;
        pstAEReadData->eAE_CHANNEL = AE_Wireless;
        pstAEReadData->fGain = get_gain_from_gain_level( ucGainLevel );
        pstAEReadData->eWorkMode = (WorkMode)ucWorkMode;
        pstAEReadData->eAE_Unit = g_eAEWirelessUnit;
        pstAEReadData->usPulseNum = AE_WIRELESS_PULSE_NUM;
        pstAEReadData->uiSamplingRate = get_wireless_sample_rate(ucWorkMode);
        pstAEReadData->uiADRange = AE_WIRELESS_AD_RANGE;
        pstAEReadData->ucADSampling = AE_WIRELESS_AD_SAMPLE_BIT;
        pstAEReadData->ucADSampDataFormat = AE_WIRELESS_AD_SAMPLE_MODE;


        //填写数据
        UINT8 *pData = pAeData+POS_WIRELESSAE_DATA;

        switch( ucWorkMode )
        {
            //幅值数据,需要对所有数值乘以系数
            case Amplitude:
            {
                AEWirelessAmpData stAmpData;
                memcpy( (UINT8*)&stAmpData, pData, sizeof(stAmpData) );
                pstAEReadData->Data.AmpData.fPeakValue = wireless_ae_adc_to_mv( stAmpData.usPeak, ucGainLevel );
                pstAEReadData->Data.AmpData.fRMS = wireless_ae_adc_to_mv( stAmpData.usRms, ucGainLevel );
                pstAEReadData->Data.AmpData.fFirstFreqComValue = wireless_ae_adc_to_mv( stAmpData.usH1, ucGainLevel );
                pstAEReadData->Data.AmpData.fScendFreqComValue = wireless_ae_adc_to_mv( stAmpData.usH2, ucGainLevel );

                if( g_eAEWirelessUnit == dB )
                {
                    pstAEReadData->Data.AmpData.fPeakValue = wireless_ae_mv_to_db(pstAEReadData->Data.AmpData.fPeakValue);
                    pstAEReadData->Data.AmpData.fRMS = wireless_ae_mv_to_db(pstAEReadData->Data.AmpData.fRMS);
                    pstAEReadData->Data.AmpData.fFirstFreqComValue = wireless_ae_mv_to_db(pstAEReadData->Data.AmpData.fFirstFreqComValue);
                    pstAEReadData->Data.AmpData.fScendFreqComValue = wireless_ae_mv_to_db(pstAEReadData->Data.AmpData.fScendFreqComValue);
                }
            }
            break;
            
            //PRPD数据,需要对峰值乘以系数
            case PRPD:
            {
                AEWirelessPrpdData stPrpdData;
                memcpy( (UINT8*)&stPrpdData, pData, sizeof(stPrpdData) );
                pstAEReadData->Data.PRPDData[0].fPhaseValue = stPrpdData.usPhase;
                pstAEReadData->Data.PRPDData[0].fPeakValue = wireless_ae_adc_to_mv( stPrpdData.usAmp, ucGainLevel );

                if( g_eAEWirelessUnit == dB )
                {
                    pstAEReadData->Data.PRPDData[0].fPeakValue = wireless_ae_mv_to_db(pstAEReadData->Data.PRPDData[0].fPeakValue);
                }
            }
            break;

            //Pulse数据,需要对峰值乘以系数
            case Pulse:
            {
                AEWirelessPulseData stPulseData;
                memcpy( (UINT8*)&stPulseData, pData, sizeof(stPulseData) );
                pstAEReadData->Data.PulseData[0].uiPulseInterval = stPulseData.ulDeltaT;
                pstAEReadData->Data.PulseData[0].fPeakValue = wireless_ae_adc_to_mv( stPulseData.usAmp, ucGainLevel );

                if( g_eAEWirelessUnit == dB )
                {
                    pstAEReadData->Data.PulseData[0].fPeakValue = wireless_ae_mv_to_db(pstAEReadData->Data.PulseData[0].fPeakValue);
                }
            }
            break;

            //wave数据,需要对所有数据乘以系数
            case Waveform:
            {
                UINT16 i = 0;
                for( i = 0; i < WIRELESSAE_WAVE_DATA_LEN; i++ )
                {
                    pstAEReadData->Data.WaveData.faWaveValue[i] = wireless_ae_adc_to_mv( pData[i], ucGainLevel );
                    if( g_eAEWirelessUnit == dB )
                    {
                        pstAEReadData->Data.WaveData.faWaveValue[i] = wireless_ae_mv_to_db(pstAEReadData->Data.WaveData.faWaveValue[i]);
                    }
                }
            }
            break;

            default:
            printf("error in [%s]:work mode invalid!\n",__func__);
            eRet = COM_INPUTPARAM_ERR;
            break;
        }
    }
    else
    {
        //resend param
        set_ae_settings( &s_stAeWirelessWorkParam, AE_Wireless );
        eRet = COM_RCV_ERR;
    }

    return -eRet;
}

/************************************************
 * 函数名   ：get_ae_data

 * 输入参数 ：AEReadData *pstAEReadData     
 AE_CHANNEL eAE_CHANNEL通道    
 AE_CMD eAE_CMD

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：获取AE数据

 * 描述     :
 ************************************************/
INT32 get_ae_data(AEReadData *pstAEReadData, AE_CHANNEL eAE_CHANNEL, AE_CMD eAE_CMD)
{

    FrameBuf stFrame;
    UINT8 aucBuf[10];
    UINT8 *pucData = NULL;
    UINT16 *pusDataAD = NULL;
    UINT16 usDataLen = 0;
    INT32 iRet = 0;
    AEDataParameter stAEDataParameter;
    WLDeviceType eWLDeviceType;
    float fGain = 1;
    memset(&stFrame, 0, sizeof(stFrame));

    //参数检查
    if ((pstAEReadData == NULL)&&(eAE_CMD == AE_GET_DATA))
    {
        printf("in get_ae_data pstAEReadData==NULL \n");
        return -COM_INPUTPARAM_ERR;
    }

#ifdef WIRELESS_AE_TEST
    eAE_CHANNEL = AE_Wireless;
#endif

    

    //组命令
    aucBuf[usDataLen++] = AE_READ_DATA;
    aucBuf[usDataLen++] = (UINT8) eAE_CMD;

    if ((g_eWorkMode == Amplitude && eAE_CMD == AE_GET_DATA) || (g_eWorkMode == Waveform && eAE_CMD == AE_GET_DATA))
    {
        stFrame.uiRcvTimeout = TIMEOUT_500MS;
    }
    else
    {
        stFrame.uiRcvTimeout = TIMEOUT_50MS;
    }

    //非AE无线调理器
    if( AE_Wireless != eAE_CHANNEL )
    {
        //发送报文
        iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, aucBuf, usDataLen);
        if (iRet != COM_OK)
        {
            comm_err_print(AE_READ_DATA, (CommErr)(-iRet));
            if (eAE_CMD == AE_OPEN_POWER || eAE_CMD == AE_CLOSE_POWER)
            {
                printf("*********************************************************\n");
            }
            return iRet;
        }

        //控制电源完成
        if (eAE_CMD == AE_OPEN_POWER || eAE_CMD == AE_CLOSE_POWER || eAE_CMD == AE_START_SAMPLE || eAE_CMD == AE_STOP_SAMPLE)
        {
            return HC_SUCCESS;
        }

        //解析状态
        pucData = get_frame_data(&stFrame, &usDataLen); //获取接收到的数据缓冲区指针和数据长度
        memcpy(&stAEDataParameter, &pucData[CMD_RESPONSE_AE_PARAM_POS], sizeof(stAEDataParameter));
        pstAEReadData->eSyncState = (SyncState) stAEDataParameter.ucSyncState;
        pstAEReadData->eSyncSource = (SyncSource) stAEDataParameter.ucSyncSource;
        pstAEReadData->eAE_CHANNEL = (AE_CHANNEL) stAEDataParameter.ucAEType;
        pstAEReadData->fGain = stAEDataParameter.fGain;
        pstAEReadData->eWorkMode = (WorkMode) stAEDataParameter.ucWorkMode;
        pstAEReadData->eAE_Unit = (AE_Unit) stAEDataParameter.ucUnit;
        pstAEReadData->usPulseNum = stAEDataParameter.usPulseNum;
        pstAEReadData->uiSamplingRate = stAEDataParameter.uiSamplingRate;
        pstAEReadData->uiADRange = stAEDataParameter.uiADRange;
        pstAEReadData->ucADSampling = stAEDataParameter.ucADSampling;
        pstAEReadData->ucADSampDataFormat = stAEDataParameter.ucADSampDataFormat;

        printf("Sync Status = %d\r\n",pstAEReadData->eSyncState);

        // printf("ae param size:%d\n",sizeof(stAEDataParameter));

        //解析数据 波形需计算实际值
        if (g_eWorkMode != Waveform)
        {
            memset(&pstAEReadData->Data, 0, sizeof(pstAEReadData->Data));
            memcpy(&pstAEReadData->Data, &pucData[sizeof(stAEDataParameter) + CMD_RESPONSE_AE_PARAM_POS],
                    usDataLen - sizeof(stAEDataParameter) - CMD_RESPONSE_AE_COMMON_LEN);
        }
        else //波形数据最后一个数据为增益等级
        {
            pusDataAD = (UINT16*) &pucData[sizeof(stAEDataParameter) + CMD_RESPONSE_AE_PARAM_POS];
            memcpy(&fGain, &pusDataAD[AEWAVENUM], sizeof(fGain));
            for (usDataLen = 0; usDataLen < AEWAVENUM; usDataLen++)
            {
                pstAEReadData->Data.WaveData.faWaveValue[usDataLen] = ae_adtomv(pusDataAD[usDataLen], fGain);
            }
        }
    }
    //AE无线调理器
    else
    { 
        if( AE_OPEN_POWER == eAE_CMD || AE_CLOSE_POWER == eAE_CMD 
        || AE_STOP_SAMPLE == eAE_CMD || AE_START_SAMPLE == eAE_CMD )
        {
            return HC_SUCCESS;
        } /*无线AE 无需处理上述命令 */
        if (change_data_type(AE_TERMINAL) != HC_SUCCESS)
        {
            printferr("\n");
            return HC_FAILURE;
        } 
        //关闭AE电源时重新设置uhf和hfct地址
        if( AE_CLOSE_POWER == eAE_CMD )
        {
            reset_uhf_hfct_conditioner_addr();
        }  
        
        //发送报文
        INT32 iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, aucBuf, usDataLen);
        if (iRet != COM_OK)
        {
            comm_err_print(AE_READ_DATA, (CommErr)(-iRet));
            return iRet;
        }

        //解析AE调理器数据
        pucData = get_frame_data(&stFrame, &usDataLen); //获取接收到的数据缓冲区指针和数据长度

        iRet = parse_wireless_ae_message( pucData, usDataLen, pstAEReadData );
        if( iRet != COM_OK )
        {
            return iRet;
        }
    }        

    return HC_SUCCESS;
}

/************************************************
 * 输出参数 : *peAEStatus 无线AE连接状态

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：获取超声波连接状态

 * 描述     : 无线AE时使用
 ************************************************/
INT32 get_ae_connectstate( AE_ConnectState *peAE_ConnectState )
{
    FrameBuf stFrame;
    INT32 iReadLen = 0;
    UINT8 ucaDataBuf[10];
    UINT8 *pucData = NULL;
    UINT16 usDataLen = 0;

    //参数检查
    if (peAE_ConnectState == NULL)
    {
        printf("in get_ae_connect_status peAE_ConnectState==NULL \n");
        return HC_FAILURE;
    }

    memset(&stFrame, 0, sizeof(stFrame));

    //组命令
    ucaDataBuf[usDataLen++] = AE_CONNECT_STATE;
    stFrame.uiRcvTimeout = TIMEOUT_50MS;

    //发送报文
    INT32 iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(AE_CONNECT_STATE, (CommErr)(-iRet));
        return HC_FAILURE;
    }
    pucData = get_frame_data(&stFrame, &usDataLen);

    //解析报文
    *peAE_ConnectState = (AE_ConnectState) pucData[CMD_AE_CONNECT_STATUS_POS];

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：set_tev_settings

 * 输入参数 ：TEVWorkSettings *pstTEVWorkSettings  TEV工作参数

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：设置TEV工作参数

 * 描述     :
 ************************************************/
INT32 set_tev_settings(TEVWorkSettings *pstTEVWorkSettings)
{
    //FrameTransfrom stFrameTransfrom;
    INT32 iRet = 0;
    FrameBuf stFrame;
    UINT8 ucDataBuf[1024];
    UINT16 usDataLen = 0;
    //参数检查
    if (pstTEVWorkSettings == NULL)
    {
        printf("in set_tev_settings pstTEVWorkSettings==NULL \n");
        return HC_FAILURE;
    }
    memset(&stFrame, 0, sizeof(stFrame));
    //组命令
    ucDataBuf[usDataLen++] = TEV_SETTINGS;
    memcpy(&ucDataBuf[usDataLen], pstTEVWorkSettings, sizeof(TEVWorkSettings));
    usDataLen += sizeof(TEVWorkSettings);

    stFrame.uiRcvTimeout = TIMEOUT_20MS;

    //发送报文
    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(TEV_SETTINGS, (CommErr)(-iRet));
        return HC_FAILURE;
    }

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：get_tev_data

 * 输入参数 ：TEV_CMD eTEV_CMD

 * 输出参数 ：TEVReadData *pstTEVReadData TEV数据

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：获取TEV数据

 * 描述     :
 ************************************************/
INT32 get_tev_data(TEVReadData *pstTEVReadData, TEV_CMD eTEV_CMD)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucRcvData = NULL;

    memset(&stFrame, 0, sizeof(stFrame));
    //组命令
    ucaDataBuf[usDataLen++] = TEV_READ_DATA;
    ucaDataBuf[usDataLen++] = (UINT8) eTEV_CMD;
    stFrame.uiRcvTimeout = (eTEV_CMD == TEV_GET_DATA) ? TIMEOUT_5S : TIMEOUT_50MS;

    //发送报文
    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(TEV_READ_DATA, (CommErr)(-iRet));
        return HC_FAILURE;
    }

    //控制电源完成
    if (eTEV_CMD == TEV_OPEN_POWER || eTEV_CMD == TEV_CLOSE_POWER)
    {
        return HC_SUCCESS;
    }

    //参数检查
    if ((pstTEVReadData == NULL)&&(eTEV_CMD == TEV_GET_DATA))
    {
        printf("in get_tev_data pstTEVReadData==NULL\n");
        return HC_FAILURE;
    }


    pucRcvData = get_frame_data(&stFrame, &usDataLen);
    //解析数据
    pstTEVReadData->ucWorkMode = pucRcvData[CMD_RESPONSE_TEV_WORKMOD_POS];
    memset(&pstTEVReadData->Data, 0, sizeof(pstTEVReadData->Data));

    memcpy(&pstTEVReadData->Data, &pucRcvData[CMD_RESPONSE_TEV_DATA_POS], usDataLen - CMD_RESPONSE_TEV_COMMON_LEN);

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：get_tev_calibrate_param

 * 输入参数 ：TEVCalibrateParam *pstTEVCalibrateParam

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：获取TEV系数

 * 描述     :
 ************************************************/
INT32 get_tev_calibrate_param(TEVCalibrateParam *pstTEVCalibrateParam)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucRcvData = NULL;
    TevChannel i;
    UINT32 j;
    UINT32 uiLen = 0;
    memset(&stFrame, 0, sizeof(stFrame));
    //组命令
    ucaDataBuf[usDataLen++] = STM32TEVPARAM;
    stFrame.uiRcvTimeout = TIMEOUT_100MS;

    //发送报文
    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(STM32TEVPARAM, (CommErr)(-iRet));
        return HC_FAILURE;
    }
    pucRcvData = get_frame_data(&stFrame, &usDataLen);
    for(i=TEV_CHANNEL_0dB;i<TEV_CHANNEL_COUNT;i++)
    {
        pstTEVCalibrateParam->CaliParam[i].fK = *((float*)&pucRcvData[CMD_RESPONSE_TEV_PARAM_POS+uiLen]);
        uiLen += sizeof(float);
        pstTEVCalibrateParam->CaliParam[i].fB = *((float*)&pucRcvData[CMD_RESPONSE_TEV_PARAM_POS+uiLen]);
        uiLen += sizeof(float);
        pstTEVCalibrateParam->CaliParam[i].eCaliSta = *((UINT8*)&pucRcvData[CMD_RESPONSE_TEV_PARAM_POS+uiLen]);
        uiLen += sizeof(UINT8);
        memcpy((void*)pstTEVCalibrateParam->ausDacRef[i],(void*)(&pucRcvData[CMD_RESPONSE_TEV_PARAM_POS+uiLen]),sizeof(pstTEVCalibrateParam->ausDacRef[i]));
        uiLen += sizeof(pstTEVCalibrateParam->ausDacRef[i]);
    }
    return HC_SUCCESS;

}

/************************************************
 * 函数名   ：set_frequency

 * 输入参数 ：Frequency eFrequency电网频率

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：设置电网频率

 * 描述     :
 ************************************************/
INT32 set_frequency(Frequency eFrequency)
{
    //设置无线电网频率
    if (set_wireless_frequency(eFrequency) != HC_SUCCESS)
    {
        return HC_FAILURE;
    }

    //设置TEV AE电网频率
    if (set_aetev_frequency(eFrequency) != HC_SUCCESS)
    {
        return HC_FAILURE;
    }

    g_eFrequency = eFrequency;
    return HC_SUCCESS;
}

Frequency get_frequency(void)
{
    return g_eFrequency;
}

/************************************************
 * 函数名   ：Calibrate_ae

 * 输入参数 ：float *fValue输出值,CalibrateAECmd eCalibrateAECmd命令

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：校准AE

 * 描述     :
 ************************************************/
INT32 Calibrate_ae(float *fValue, CalibrateAECmd eCalibrateAECmd)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;
    //参数检查
    if (fValue == NULL)
    {
        printf("in Calibrate_ae fValue==NULL \n");
        return HC_FAILURE;
    }
    memset(&stFrame, 0, sizeof(stFrame));

    //组命令
    ucaDataBuf[usDataLen++] = Calibrate_AE;
    ucaDataBuf[usDataLen++] = (UINT8) eCalibrateAECmd;
    stFrame.uiRcvTimeout = TIMEOUT_500MS;

    //发送报文
    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(Calibrate_AE, (CommErr)(-iRet));
        return HC_FAILURE;
    }
    pucData = get_frame_data(&stFrame, &usDataLen);
    //解析数据
    memcpy((UINT8 *) fValue, &pucData[CMD_RESPONSE_AE_CALIBRATE_DATA_POS], sizeof(float));

    return HC_SUCCESS;
}
void print_CaliParam(AutoCali *pstCali)
{
    UINT8 i;
    printf("k:%f B:%f\n",pstCali->stParam.fK,pstCali->stParam.fB);
    printf("Dac Ref Code:\n");
    for(i=0;i<sizeof(pstCali->ausDacRef)/sizeof(pstCali->ausDacRef[0]);i++)
    {
        if(i!=0 && i%10==0)
        {
            printf("\n");
        }
        printf("%4d ",pstCali->ausDacRef[i]);
    }
    printf("\n");
    return ;
}
void print_TevSampleData(TevSample *p)
{
    int i,j;
    printf("Tev DB Val:%d\n",p->ucDB);
    for(i=0;i<3;i++)
    {
        printf("TEV channel:%d--Dac Value:%d\n",i+1,p->ausDacVal[i]);
        printf("Dac Ref Array:\n");
        for(j=0;j<20;j++)
        {
            if(j!=0 && j%10==0)
            {
                printf("\n");
            }

            printf("%2d ",p->ausDacRef[i][j]);
        }
        printf("\n");

    }
    return ;
}
/************************************************
 * 函数名   ：Calibrate_tev

 * 输入参数 ：eCmd - TEV 校准子命令
 *                   TEV_RESET_CALI_PARAM -恢复默认参数
 * 
	                 TEV_CALI_GAIN1, - 40dB档位校准


	                 TEV_CALI_GAIN5, - 20dB档位校准

	                 TEV_CALI_GAIN50, - 0dB系数校准

 * 输出参数 ：pstCali - 校准命令返回参数信息
 *           pstCali -> usaDacRef --校准档位 DAC 码值参考数组
 *           pstCali -> stParam.fK   --当前档位计算校准系数 K （仅当校准成功时有效，否则返回默认系数）
 *           pstCali -> stParam.fB   --当前档位计算校准系数 B （仅当校准成功时有效，否则返回默认系数）
 *           pstCali -> stParam.eCaliSta   --当前校准状态

 * 返回值   ：0:succeed; negtive:failed 

 * 功能     ：校准TEV

 * 描述     : 
 * 1.校准算法更新：
 *   每个档位一个校准点，使用二分法采集10次校准点对应的DAC码值取均值，作为该校准点对应的最终码值，
 *   并根据如下公式（DAC[i]=DAC[i-1]*1.112)更新当前校准档位的码值参考数组
 *   档位    校准点    校准电压mVpp
 *   0~19dB    18dB       321
 *   20~39dB   25dB       718
 *   40~59dB   43dB       5700
 *                     
 ************************************************/
INT32 Calibrate_tev(TevCaliCommand eCmd,AutoCali *pstCali)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;

    //参数检查
    if (pstCali == NULL)
    {
        printf("in Calibrate_tev pstCali==NULL \n");
        return HC_FAILURE;
    }
    memset(&stFrame, 0, sizeof(stFrame));


    ucaDataBuf[usDataLen++] = Calibrate_TEV;
    ucaDataBuf[usDataLen++] = (UINT8)eCmd;
    stFrame.uiRcvTimeout = TIMEOUT_10S;

    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaDataBuf, usDataLen); //发送报文

    if (iRet != COM_OK)
    {
        comm_err_print(Calibrate_TEV, (CommErr)(-iRet));
        return HC_FAILURE;
    }
    pucData = get_frame_data(&stFrame, &usDataLen);

    printf("%s: cmd: %d,subcmd:%d,com status:%d\n",__func__,pucData[0],pucData[2],pucData[1]);

    
    pstCali->stParam.eCaliSta = pucData[3];
    memcpy((char*)(&pstCali->stParam.fK),(char*)&pucData[4],sizeof(float));
    memcpy((char*)(&pstCali->stParam.fB),(char*)&pucData[8],sizeof(float));
    memcpy(pstCali->ausDacRef,&pucData[12],sizeof(pstCali->ausDacRef));

    print_CaliParam(pstCali);
    return HC_SUCCESS;
}
/************************************************
 * 功能     ：设置TEV档位校准系数 
 * 
 * 描述     : 用于手动微调TEV档位校准系数K,B

 * 输入参数 ： eCmd - TEV 校准子命令
 *              TEV_SET_GAIN1_CALI_COEFF -- 40dB档位校准系数设置
 *              TEV_SET_GAIN5_CALI_COEFF -- 20dB档位校准系数设置
 *              TEV_SET_GAIN50_CALI_COEFF -- 0dB档位校准系数设置
 *   
 *           
 * 
 * 输出参数 : 无
 *          
 ************************************************/
INT32 set_tev_cali_coeff(TevCaliCommand eCmd,float fK,float fB)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[20];
    UINT16 usDataLen = 0;



    memset(&stFrame, 0, sizeof(stFrame));


    ucaDataBuf[usDataLen++] = Calibrate_TEV;
    ucaDataBuf[usDataLen++] = (UINT8)eCmd;
    memcpy(&ucaDataBuf[usDataLen],(void*)&fK,sizeof(fK));
    usDataLen += sizeof(fK);
    memcpy(&ucaDataBuf[usDataLen],(void*)&fB,sizeof(fB));
    usDataLen += sizeof(fB);
    stFrame.uiRcvTimeout = TIMEOUT_5S;
    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaDataBuf, usDataLen); //发送报文

    if (iRet != COM_OK)
    {
        comm_err_print(Calibrate_TEV, (CommErr)(-iRet));
        return HC_FAILURE;
    }


    return HC_SUCCESS;
}
/************************************************
 * 功能     ：读取TEV测量值
 * 
 * 描述     : 用于测试TEV采集值信息

 * 输入参数 ：TevSample 结构体指针
 *   
        
 * 输出参数 : TevSample 结构体指针
 *          
 ************************************************/
INT32 test_tev_sample(TevSample *stTev)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[20];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;

    memset(&stFrame,0, sizeof(stFrame));
    ucaDataBuf[usDataLen++] = Calibrate_TEV;
    ucaDataBuf[usDataLen++] = TEV_TEST_GET_SAMPLE_DATA;
    stFrame.uiRcvTimeout = TIMEOUT_10S;
    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaDataBuf, usDataLen); //发送报文
    if (iRet != COM_OK)
    {
        comm_err_print(Calibrate_TEV, (CommErr)(-iRet));
        return HC_FAILURE;
    }
    //todo get data
    pucData = get_frame_data(&stFrame,&usDataLen);
    printf("datalen:%d sizeof(TEVsample):%d\n",usDataLen,sizeof(TevSample));
    memcpy(stTev,pucData+3,sizeof(TevSample));
    print_TevSampleData(stTev);
    return iRet;
}


/************************************************
 * 函数名   ：conditioner_cmd

 * 输入参数 ：UPDATECMD eUPDATECMD命令
 
 * 输出参数 ：

 * 返回值   ：0:succeed   negtive:failed
 *        1:stm32 运行在APP;2：stm32运行在BOOT（仅当eUPDATECMD = CONDITIONWHERE时返回值有效）

 * 功能     ：查询调理器是否在BOOT

 * 描述     : 
 ************************************************/
#define STM32_BOOT_RUN  2
#define STM32_APP_RUN  1
static INT32 conditioner_cmd(UPDATECMD eUPDATECMD)
{
    UINT8 ucApp;
    FrameBuf stFrame;
    UINT8 ucaBuf[10];
    UINT16 usDataLen = 0;
    INT32 iRet = 0;
    UINT8 *pucData = NULL;
    memset(&stFrame, 0, sizeof(stFrame));

    //组命令
    ucaBuf[usDataLen++] = UPDATECONDITINEER; //command
    ucaBuf[usDataLen++] = (UINT8) eUPDATECMD; //subcommand
    ucaBuf[usDataLen++] = 0; //datalenH8
    ucaBuf[usDataLen++] = 0; //datalenL8

    stFrame.uiRcvTimeout = (eUPDATECMD == CONDITIONERASE) ? TIMEOUT_10S : TIMEOUT_100MS;
    //发送报文
    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(UPDATECONDITINEER, (CommErr)(-iRet));
        return HC_FAILURE;
    }
    pucData = get_frame_data(&stFrame, &usDataLen);
    //返回结果
    switch (eUPDATECMD)
    {
    case CONDITIONERASE:
        return HC_SUCCESS; //握手擦除 返回即是成功
    case CONDITIONWHERE:
    {
        ucApp = pucData[CMD_RESPONSE_IAP_DATA_POS];
        return ucApp ? STM32_APP_RUN : STM32_BOOT_RUN;
    }
    case CONDITIONEGOBOOT:
    case CONDITIONGOAPP:
        return (pucData[CMD_RESPONSE_IAP_DATA_POS] == IAP_BOOT_APP_SWITCH_OK) ? HC_SUCCESS : HC_FAILURE;
    default:
        return HC_FAILURE;
    }
}

/************************************************
 * 函数名   ：cond_load_pragram

 * 输入参数 ：UPDATECMD eUPDATECMD命令
 
 * 输出参数 ：

 * 返回值   ：0:succeed   negtive:failed

 * 功能     ：查询调理器是否在BOOT

 * 描述     : 
 ************************************************/
static INT32 cond_load_program(void)
{
    INT32 iFd = -1;
    FrameBuf stFrame;
    UINT16 usDataLen = 0;
    INT8 ucaSndData[2048];
    INT8 *pucRcvData = NULL;
    INT32 iReadLen = 0;
    INT32 iRet = 0;
    UINT32 uiDataToBeRead = 0;
    UINT32 uiStartAddr = LOADPRAGRAMSTARADDR;
    struct stat stStat;

    //打开程序
    iFd = open(CONDITIONERROGRM, O_RDWR);
    if (iFd < 0)
    {
        printf("failed cond_load_program program file open iFd=%d\n", iFd);
        return HC_FAILURE;
    }
    if (fstat(iFd, &stStat))
    {
        printf("%s:fail to get "CONDITIONERROGRM" file info\n",__func__);
        return HC_FAILURE;
    }

    memset(&stFrame, 0, sizeof(stFrame));
    //pucSndData = get_send_frame_data(&stFrame);
    //组命令
    ucaSndData[IAP_COMMAND_POS] = UPDATECONDITINEER;
    ucaSndData[IAP_SUBCOMMAND_POS] = (UINT8) CONDITIONDATA;
    ucaSndData[IAP_DATALENHI_POS] = HI8(IAP_DATA_LEN);
    ucaSndData[IAP_DATALENLO_POS] = LO8(IAP_DATA_LEN);
    stFrame.uiRcvTimeout = TIMEOUT_100MS;
    //发送程序
    uiDataToBeRead = stStat.st_size;
    while (uiDataToBeRead)
    {
        ucaSndData[IAP_PROGRAME_ADDR_POS] = HH8(uiStartAddr);
        ucaSndData[IAP_PROGRAME_ADDR_POS + 1] = HL8(uiStartAddr);
        ucaSndData[IAP_PROGRAME_ADDR_POS + 2] = LH8(uiStartAddr);
        ucaSndData[IAP_PROGRAME_ADDR_POS + 3] = LL8(uiStartAddr);
        iReadLen = read(iFd, &ucaSndData[IAP_PROGRAME_DATA_POS], IAP_PROGRAME_DATA_LEN);
        if (iReadLen < 0)
        {
            printf("%s:read upgrade file failed:%s!\n", __func__, strerror(errno));
            return HC_FAILURE;
        }
        uiDataToBeRead -= iReadLen;
        if (iReadLen < IAP_PROGRAME_DATA_LEN) //长度不足，补0
        {
            memset(&ucaSndData[IAP_PROGRAME_DATA_POS + iReadLen], 0, IAP_PROGRAME_DATA_LEN - iReadLen);
        }
        usDataLen = IAP_PROGRAME_FRAME_LEN;
        iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaSndData, usDataLen);
        if (iRet != COM_OK)
        {
            comm_err_print(UPDATECONDITINEER, (CommErr)(-iRet));
            return HC_FAILURE;
        }
        pucRcvData = get_frame_data(&stFrame, &usDataLen);
        if (pucRcvData[CMD_RESPONSE_IAP_DATA_POS] != IAP_PROGRAME_OK)
        {
            printf("%s:iap programe data failed\n", __func__);
            return HC_FAILURE;
        }

        uiStartAddr += IAP_PROGRAME_DATA_LEN;
    }

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：wait_stm32_state

 * 输入参数 ：UPDATECMD eUPDATECMD
 
 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：等待Stm32进入相应的工作状态

 * 描述     : 超时等待10S
 ************************************************/
static INT32 wait_stm32_state(UPDATECMD eUPDATECMD)
{
    UINT16 usStateVal = 0;
    INT32 iTimeOut = 0;

    //选择期待值
    switch (eUPDATECMD)
    {
    case CONDITIONEGOBOOT:
        usStateVal = STM32_BOOT_RUN;
        break;
    case CONDITIONGOAPP:
        usStateVal = STM32_APP_RUN;
        break;
    default:
        printferr("input wrong eUPDATECMD=%d\n", eUPDATECMD);
        return HC_FAILURE;
    }

    //等待Stm32在期待区
    while (1)
    {
        if (conditioner_cmd(CONDITIONWHERE) == usStateVal) //查询调理器在期待区域
        {
            break;
        }
        if (iTimeOut++ > 100) //超时
        {
            printferr("iTimeOut\n");
            return HC_FAILURE;
        }
        usleep(100000);
    }

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：update_conditioner

 * 输入参数 ：UINT8 *pucPercentage

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：升级调理器固件

 * 描述     : 程序路径CONDITIONERROGRM
 ************************************************/
INT32 update_conditioner(UINT8 *pucPercentage)
{
    UPDATECMD eUPDATECMD;

    //参数检查
    if (NULL == pucPercentage)
    {
        printferr("NULL==pucPercentage\n");
        return HC_FAILURE;
    }

    *pucPercentage = 0; //0%
    //调理器至BOOT区
    if (conditioner_cmd(CONDITIONWHERE) != STM32_BOOT_RUN) //查询调理器是否在BOOT区域
    {
//发送跳转到BOOT区命令
        if (conditioner_cmd(CONDITIONEGOBOOT) != HC_SUCCESS)
        {
            printf("IAP jump to boot failed!\n");
            return HC_FAILURE;
        }
        *pucPercentage = 10; //10%

//等待调理气在Boot区
        if (wait_stm32_state(CONDITIONEGOBOOT) != HC_SUCCESS)
        {
            printf("wait for stm32 state jump boot failed\n");
            return HC_FAILURE;
        }

    }
    *pucPercentage = 25; //25%

    //擦除FLASH
    if (conditioner_cmd(CONDITIONERASE) != HC_SUCCESS)
    {
        printf("erase flash failed\n");
        return HC_FAILURE;
    }
    *pucPercentage = 60; //60%

    //下载程序
    if (cond_load_program() != HC_SUCCESS)
    {
        printf("down program failed\n");
        return HC_FAILURE;
    }
    *pucPercentage = 70; //70%

    //跳转至APP区
    if (conditioner_cmd(CONDITIONGOAPP) != HC_SUCCESS)
    {
        printf("jump to app failed!\n");
        return HC_FAILURE;
    }
    //等待调理气在APP区
    if (wait_stm32_state(CONDITIONGOAPP) != HC_SUCCESS)
    {
        printf("wait for stm32 state jump  app failed\n");
        return HC_FAILURE;
    }
    *pucPercentage = 100; //100%

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：get_stm32_inapp

 * 输入参数 ：

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：读取Stm32是否在App

 * 描述     : 在App返回成功  ，仅当stm32有响应报文，且显示stm32运行在BOOT时返回失败。
 ************************************************/
INT32 get_stm32_inapp(void)
{
    if (conditioner_cmd(CONDITIONWHERE) == STM32_BOOT_RUN)
    {
        return HC_FAILURE;
    }
    else
    {
        return HC_SUCCESS;
    }
}

/************************************************
 * 函数名   ：stm32_goto_app

 * 输入参数 ：

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：Stm32跳转至App

 * 描述     :
 ************************************************/
INT32 stm32_goto_app(void)
{
    //跳转至APP区
    if (conditioner_cmd(CONDITIONGOAPP) != HC_SUCCESS)
    {
        return HC_FAILURE;
    }
    //等待调理气在APP区
    if (wait_stm32_state(CONDITIONGOAPP) != HC_SUCCESS)
    {
        return HC_FAILURE;
    }
    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：check_zigbee_boot

 * 输入参数 ：

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：检查zigbee在boot区

 * 描述     :
 ************************************************/
static INT32 check_zigbee_boot(void)
{
    FrameBuf stFrame;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;

    UHFHFCTAETEV_dbg("\n");

    //查询是否在boot状态
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_ADDR;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_CMD;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_SELF;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_CMD;

    stFrame.uiRcvTimeout = TIMEOUT_1S;
    if (comm_zigbee_iap_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen) != COM_OK)
    {
        printferr("comm_zigbee_iap_handle\n");
        return HC_FAILURE;
    }
    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：erase_zigbee_flash

 * 输入参数 ：

 * 输出参数 ：

 * 返回值   ：文件描述符; NULL:failed

 * 功能     ：擦除zigbee程序

 * 描述     : 打开文件
 ************************************************/
static FILE* erase_zigbee_flash(void)
{
    FrameBuf stFrame;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    FILE* pFile = NULL;
    struct stat stStatBuff;

    UHFHFCTAETEV_dbg("\n");

    //打开文件
    pFile = fopen(ZIGBEEPROGRM, "r");
    if (pFile == NULL)
    {
        printferr("failed in fopen pcFilePath=%x errno:%s\n", ZIGBEEPROGRM, strerror(errno));
        return NULL;
    }

    //获取文件大小
    if (stat(ZIGBEEPROGRM, &stStatBuff) < 0)
    {
        printferr("failed in stat errno:%s\n", strerror(errno));
        fclose(pFile);
        return NULL;
    }

    //擦除zigbee程序
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_ADDR;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_CMD;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_SELF;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_ERASE;
    ucaDataBuf[usDataLen++] = M32TO8_L1(stStatBuff.st_size);
    ucaDataBuf[usDataLen++] = M32TO8_L2(stStatBuff.st_size);
    ucaDataBuf[usDataLen++] = M32TO8_L3(stStatBuff.st_size);
    ucaDataBuf[usDataLen++] = M32TO8_L4(stStatBuff.st_size);

    stFrame.uiRcvTimeout = TIMEOUT_10S;

    //发送命令
    if (comm_zigbee_iap_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen) != COM_OK)
    {
        printferr("comm_zigbee_iap_handle\n");
        fclose(pFile);
        return NULL;
    }
    return pFile;
}

/************************************************
 * 函数名   ：send_zigbee_program

 * 输入参数 ：

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：下载zigbee程序

 * 描述     :
 ************************************************/
static INT32 send_zigbee_program(FILE* pFile)
{
    UINT32 uiSendLen = 0;
    INT32 iReadLen = 0;
    INT32 iWrongTime = 0;
    FrameBuf stFrame;
    UINT8 ucaDataBuf[128];
    UINT16 usDataLen = 0;

    UHFHFCTAETEV_dbg("\n");

    while (1)
    {
        //读数据
        iReadLen = fread(&ucaDataBuf[UPDATE_PROGRAM_BYTE], sizeof(UINT8), LOAD_ZIGBEE_FRAMESIZE, pFile);
        if (iReadLen < LOAD_ZIGBEE_FRAMESIZE)
        {
            if (!feof(pFile))
            {
                printferr("fread\n");
                return HC_FAILURE;
            }
            UHFHFCTAETEV_dbg("file is over\n");
        }

        //组命令
        usDataLen = 0;
        ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_ADDR;
        ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_CMD;
        ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_SELF;
        ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_DATA;
        ucaDataBuf[usDataLen++] = M32TO8_L1(uiSendLen); //数据地址
        ucaDataBuf[usDataLen++] = M32TO8_L2(uiSendLen);
        ucaDataBuf[usDataLen++] = M32TO8_L3(uiSendLen);
        ucaDataBuf[usDataLen++] = M32TO8_L4(uiSendLen);
        ucaDataBuf[usDataLen++] = M32TO8_L1(iReadLen); //数据长度
        ucaDataBuf[usDataLen++] = M32TO8_L2(iReadLen);
        ucaDataBuf[usDataLen++] = M32TO8_L3(iReadLen);
        ucaDataBuf[usDataLen++] = M32TO8_L4(iReadLen);

        stFrame.uiRcvTimeout = TIMEOUT_500MS;
        usDataLen += LOAD_ZIGBEE_FRAMESIZE;

        //发送命令
        if (comm_zigbee_iap_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen) != COM_OK)
        {
            if (iWrongTime++ > 3) //连续3帧错误 返回错误
            {
                printferr("comm_zigbee_iap_handle\n");
                return HC_FAILURE;
            }
            if (fseek(pFile, -iReadLen, SEEK_CUR) < 0) //文件位置移动回原来位置
            {
                printferr("fseek");
                return HC_FAILURE;
            }
        }
        iWrongTime = 0; //错误计数清0
        uiSendLen += iReadLen; //发送数据计数

        if (feof(pFile))
        {
            return HC_SUCCESS;
        }
    }

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：check_zigbee_program

 * 输入参数 ：

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：校验程序 并升级

 * 描述     :
 ************************************************/
static INT32 check_zigbee_program(void)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    INT32 iRetry = 0;

    UHFHFCTAETEV_dbg("\n");

    //校验程序
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_ADDR;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_CMD;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_SELF;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_CHECK;
    stFrame.uiRcvTimeout = TIMEOUT_1S;

    //发送命令
    if (comm_zigbee_iap_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen) != COM_OK)
    {
        printferr("comm_zigbee_iap_handle\n");
        return HC_FAILURE;
    }

    //升级程序
    memset(ucaDataBuf, 0, sizeof(ucaDataBuf));
    usDataLen = 0;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_ADDR;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_CMD;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_SELF;
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_OVER;
    stFrame.uiRcvTimeout = TIMEOUT_50MS;
    comm_zigbee_iap_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);

    //等待升级完成
    while (iRetry++ < 20)
    {
        memset(ucaDataBuf, 0, sizeof(ucaDataBuf));
        usDataLen = 0;
        ucaDataBuf[usDataLen++] = RECEIVER_VERSION;
        stFrame.uiRcvTimeout = TIMEOUT_50MS;

        iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);
        if (iRet != COM_OK)
        {
            usleep(500000);
            continue;
        }
        break;
    }

    if (iRetry < 20)
    {
        return HC_SUCCESS;
    }
    return HC_FAILURE;
}

/************************************************
 * 函数名   ：update_zigbee

 * 输入参数 ：UINT8 *pucPercentage

 * 输出参数 ：

 * 返回值   ：0:succeed; negtive:failed

 * 功能     ：升级zigbee固件

 * 描述     : 程序路径CONDITIONERROGRM
 ************************************************/
INT32 update_zigbee(UINT8 *pucPercentage)
{
    FrameBuf stFrame;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;

    FILE* pFile = NULL;

    UHFHFCTAETEV_dbg("\n");
    *pucPercentage = 0; //0%

    //进入boot状态
    ucaDataBuf[usDataLen++] = UPDATE_ZIGBEE_CMD;
    stFrame.uiRcvTimeout = TIMEOUT_100MS;
    comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, ucaDataBuf, usDataLen);

    *pucPercentage = 10; //10%
    //等待进入boot
    if (check_zigbee_boot() < 0)
    {
        if (check_zigbee_boot() < 0)
        {
            printferr("check_zigbee_boot\n");
            return HC_FAILURE;
        }
    }
    *pucPercentage = 25; //25%

    //擦除zigbee程序
    pFile = erase_zigbee_flash();
    if (NULL == pFile)
    {
        printferr("erase_zigbee_flash\n");
        return HC_FAILURE;
    }
    *pucPercentage = 40; //40%

    //下载zigbee程序
    if (send_zigbee_program(pFile) < 0)
    {
        printferr("send_zigbee_program\n");
        fclose(pFile);
        return HC_FAILURE;
    }
    fclose(pFile);
    *pucPercentage = 90; //90%

    //校验程序 并升级
    if (check_zigbee_program() < 0)
    {
        printferr("check_zigbee_program\n");
        return HC_FAILURE;
    }
    *pucPercentage = 100; //100%

    return HC_SUCCESS;
}

/************************************************
 * 函数名   ：get_bat_power

 * 输入参数 ：
 
 * 输出参数 ：

 * 返回值   ：0~100%; negtive:failed

 * 功能     ：获取电池电量

 * 描述     : 
 ************************************************/
INT32 get_bat_power(void)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;

    //组命令
    ucaDataBuf[usDataLen++] = BATTEERY_POWER;
    stFrame.uiRcvTimeout = TIMEOUT_500MS;

    //发送报文
    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(BATTEERY_POWER, (CommErr)(-iRet));
        return iRet;
    }
    pucData = get_frame_data(&stFrame, &usDataLen);

    //解析数据
    return (INT32) pucData[CMD_RESPONSE_BATTERA_POS];
}

/************************************************
 * 函数名   ：stm32_version

 * 输入参数 ：

 * 输出参数 ：

 * 返回值   ：版本号指针; NULL:failed

 * 功能     ：获取stm32版本

 * 描述     :
 ************************************************/
static INT8 gcaStm32VerStr[STM32VERBYTENUM];
const INT8 * stm32_version(void)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaData[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;

    //组命令
    ucaData[usDataLen++] = STM32VERSION;
    stFrame.uiRcvTimeout = TIMEOUT_50MS;

    //发送报文
    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaData, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(STM32VERSION, (CommErr)(-iRet));
        return NULL;
    }
    pucData = get_frame_data(&stFrame, &usDataLen);
    //解析数据
    snprintf(gcaStm32VerStr, STM32VERBYTENUM, "V%d.%d.%d.%d", pucData[CMD_RESPONSE_STM32VER_POS], pucData[CMD_RESPONSE_STM32VER_POS + 1],
            pucData[CMD_RESPONSE_STM32VER_POS + 2], pucData[CMD_RESPONSE_STM32VER_POS + 3]);
    return gcaStm32VerStr;
}

/************************************************
 * 函数名   ：device_id_num

 * 输入参数 ：

 * 输出参数 ：

 * 返回值   ：设备序列号指针; NULL:failed

 * 功能     ：获取设备序列号

 * 描述     :
 ************************************************/
static UINT8 gucaDeviceID[STM32DEVICEIDNUM];
const UINT8 * device_id_num(void)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaData[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;

    //组命令
    ucaData[usDataLen++] = STM32DEVICEID;
    stFrame.uiRcvTimeout = TIMEOUT_100MS;

    //发送报文
    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaData, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(STM32DEVICEID, (CommErr)(-iRet));
        return NULL;
    }
    pucData = get_frame_data(&stFrame, &usDataLen);
    //解析数据
    memcpy(gucaDeviceID, &pucData[CMD_RESPONSE_DEVICEID_POS], STM32DEVICEIDNUM);
    return gucaDeviceID;
}

/************************************************
 * 功能:获取当前TEV mv值
 ************************************************/
INT32 get_tev_voltage_value(INT32 *iVol1, INT32 *iVol2, INT32 *iVol3)
{
    FrameBuf stFrame;
    INT32 iRet = 0;
    UINT8 ucaDataBuf[10];
    UINT16 usDataLen = 0;
    UINT8 *pucData = NULL;
    UINT32 uiIndex = 0;
    UINT32 uiResult[3] = {0};

    //组命令
    ucaDataBuf[usDataLen++] = TEV_VOLTAGE_VALUE;
    stFrame.uiRcvTimeout = TIMEOUT_1S*15;
    
    printf("Get Tev Mv:\n");

    //发送报文
    iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, ucaDataBuf, usDataLen);
    if (iRet != COM_OK)
    {
        comm_err_print(TEV_VOLTAGE_VALUE, (CommErr)(-iRet));
        return HC_FAILURE;
    }

    printf("Data Ready:\n");

    pucData = get_frame_data(&stFrame, &usDataLen);

    pucData += CMD_RESPONSE_TEV_VOLTAGE_VALUE_POS;

    memcpy(&uiResult[0], pucData, sizeof(INT32));

    pucData += sizeof(INT32);

    memcpy(&uiResult[1], pucData, sizeof(INT32));

    pucData += sizeof(INT32);

    memcpy(&uiResult[2], pucData, sizeof(INT32));

    *iVol1 = uiResult[0];
    *iVol2 = uiResult[1];
    *iVol3 = uiResult[2];

    printf("%d, %d, %d\n", *iVol1, *iVol2, *iVol3);

    return HC_SUCCESS;
}

/************************************************
 * 功能     ：一次获取AE四种图谱数据
 * 
 * 描述     :使用方法:
 *           1.调用set_ae_settings设置参数,需要填写所有项,工作模式需要填写成AllViews
 *           2.调用get_ae_data接口,下发开始采样命令
 *           2.调用get_all_ae_views获取数据并解析
 *              幅值数据 -- 固定长度
 *              相位数据 -- 有效点数为usPhasePointNum,其他为空
 *              脉冲数据 -- 有效点数为usPulsePointNum,其他为空
 *              波形数据 -- 有效点数为usWavePointNum,其他为空
 *           3.如果需要关闭采集,则调用原有的get_ae_data接口即可
 * 
 * 输入参数 ：
 *           eCaliCmd -- TEV校准命令
 *           eAE_CHANNEL -- AE通道
 * 输出参数 : 
 *           pstCaliInfo -- 底层返回的校准信息,使用方法见描述
 * 返回值  : 0 -- success; else -- fail
 ************************************************/
INT32 get_all_ae_views(AEAllView *pstAEViewData,  AE_CHANNEL eAE_CHANNEL)
{
    FrameBuf stFrame;
    UINT8 aucBuf[10];
    UINT8 *pucData = NULL;
    UINT16 *pusDataAD = NULL;
    UINT16 usDataLen = 0;
    INT32 iRet = 0;
    AEMultiViewParam stAEMultiViewParam;
    WLDeviceType eWLDeviceType;
    float fGain = 1;
    memset(&stFrame, 0, sizeof(stFrame));
    UINT32 uiResponseLen = 0;
    AEReadData stAERead;

    //参数检查
    if (pstAEViewData == NULL)
    {
        printf("in get_ae_data pstAEViewData==NULL \n");
        return -COM_INPUTPARAM_ERR;
    }

    if (change_data_type( AE_TERMINAL ) != HC_SUCCESS)
    {
        printferr("change data type\n");
        return HC_FAILURE;
    }

    if( AE_Wireless == eAE_CHANNEL )
    {
        //组命令
        aucBuf[usDataLen++] = AE_READ_DATA;
        aucBuf[usDataLen++] = (UINT8) AE_GET_DATA;
        stFrame.uiRcvTimeout = TIMEOUT_100MS;

        //发送报文
        iRet = comm_with_zigbee_handle(g_iFd_Zigbee, &stFrame, aucBuf, usDataLen);
        if (iRet != COM_OK)
        {
            comm_err_print(AE_READ_DATA, (CommErr)(-iRet));
            return iRet;
        }

        //解析状态
        pucData = get_frame_data(&stFrame, &usDataLen); //获取接收到的数据缓冲区指针和数据长度

        //复用之前的接口,获取波形数据,作为原始值输入
        iRet = parse_wireless_ae_message( pucData, usDataLen, &stAERead );
        if(( iRet != COM_OK )||( Waveform != stAERead.eWorkMode ))//无线AE一次采集四图谱时工作模式需设置成波形
        {
            return HC_FAILURE;
        }

        //读参数
        pstAEViewData->eSyncState = stAERead.eSyncState;
        pstAEViewData->eSyncSource = stAERead.eSyncSource;
        pstAEViewData->eAE_CHANNEL = stAERead.eAE_CHANNEL;
        pstAEViewData->fGain = stAERead.fGain;
        pstAEViewData->eWorkMode = stAERead.eWorkMode;
        pstAEViewData->eAE_Unit = stAERead.eAE_Unit;
        pstAEViewData->uiSamplingRate = stAERead.uiSamplingRate;
        pstAEViewData->uiADRange = stAERead.uiADRange;
        pstAEViewData->ucADSampling = stAERead.ucADSampling;
        pstAEViewData->ucADSampDataFormat = stAERead.ucADSampDataFormat;

        //获取四图谱数据
        if( SUCCEED != ae_raw_to_multi_views( &stAERead.Data.WaveData.faWaveValue[0], AEWAVENUM, &s_stAeWirelessWorkParam, pstAEViewData ))
        {
            printferr("ae wireless convert to multi views error!\n");
            return HC_FAILURE;
        }
    }
    else
    {
        //组命令
        aucBuf[usDataLen++] = COM_AE_READ_MULTI_VIEWS;
        stFrame.uiRcvTimeout = TIMEOUT_1S;

        //发送报文
        iRet = comm_with_stm32_handle(g_iFd_STM32, &stFrame, aucBuf, usDataLen);
        if (iRet != COM_OK)
        {
            comm_err_print(AE_READ_DATA, (CommErr)(-iRet));
            return iRet;
        }

        //解析状态
        pucData = get_frame_data(&stFrame, &usDataLen); //获取接收到的数据缓冲区指针和数据长度

        //读参数
        memcpy(&stAEMultiViewParam, &pucData[CMD_AE_MULTIVIEW_DATA_POS], sizeof(stAEMultiViewParam));
        uiResponseLen += sizeof(stAEMultiViewParam);

        pstAEViewData->eSyncState = (SyncState) stAEMultiViewParam.ucSyncState;
        pstAEViewData->eSyncSource = (SyncSource) stAEMultiViewParam.ucSyncSource;
        pstAEViewData->eAE_CHANNEL = (AE_CHANNEL) stAEMultiViewParam.ucChannel;
        pstAEViewData->fGain = stAEMultiViewParam.fGain;
        pstAEViewData->eWorkMode = (WorkMode) stAEMultiViewParam.ucWorkMode;
        pstAEViewData->eAE_Unit = (AE_Unit) stAEMultiViewParam.ucUnit;
        pstAEViewData->uiSamplingRate = stAEMultiViewParam.uiSamplingRate;
        pstAEViewData->uiADRange = stAEMultiViewParam.uiADRange;
        pstAEViewData->ucADSampling = stAEMultiViewParam.ucADSampling;
        pstAEViewData->ucADSampDataFormat = stAEMultiViewParam.ucADSampDataFormat;
        pstAEViewData->usPhasePointNum = stAEMultiViewParam.usPhasePointNum;
        pstAEViewData->usPulsePointNum = stAEMultiViewParam.usPulsePointNum;
        pstAEViewData->usWavePointNum = stAEMultiViewParam.usWavePointNum;

        //获取AE的幅值数据
        memcpy(&pstAEViewData->stAeAmp, &pucData[CMD_AE_MULTIVIEW_DATA_POS + uiResponseLen], sizeof( AEAmpData ));
        // ("AMP offset: %d\n", CMD_AE_MULTIVIEW_DATA_POS + uiResponseLen);
        uiResponseLen += sizeof( AEAmpData );

        //获取AE的相位数据
        memcpy(&pstAEViewData->staPhaseData, &pucData[CMD_AE_MULTIVIEW_DATA_POS + uiResponseLen], MAX_PHASE_PULSE_NUM_PER_MSG * sizeof( AEPhaseData ));
        // DBG_INFO("PHASE offset: %d\n", CMD_AE_MULTIVIEW_DATA_POS + uiResponseLen);
        uiResponseLen += MAX_PHASE_PULSE_NUM_PER_MSG * sizeof( AEPhaseData );

        //获取AE的飞行数据
        memcpy(&pstAEViewData->staPulseData, &pucData[CMD_AE_MULTIVIEW_DATA_POS + uiResponseLen], MAX_PHASE_PULSE_NUM_PER_MSG * sizeof( AEPulseData ));
        // DBG_INFO("PULSE offset: %d\n", CMD_AE_MULTIVIEW_DATA_POS + uiResponseLen);
        uiResponseLen += MAX_PHASE_PULSE_NUM_PER_MSG * sizeof( AEPulseData );

        //获取AE的波形数据
        pusDataAD = (UINT16*) &pucData[CMD_RESPONSE_AE_PARAM_POS + uiResponseLen];

        pstAEViewData->usWavePointNum = pstAEViewData->usWavePointNum > MAX_WAVE_POINT_PER_MSG ? MAX_WAVE_POINT_PER_MSG : pstAEViewData->usWavePointNum;

        // DBG_INFO("WAVE offset: %d\n", CMD_AE_MULTIVIEW_DATA_POS + uiResponseLen);
        UINT16 usIndex = 0;
        for( usIndex = 0; usIndex < pstAEViewData->usWavePointNum; usIndex++ )
        {
            pstAEViewData->faWaveData[usIndex] = ae_adtomv( pusDataAD[usIndex], pstAEViewData->fGain );
        }        
    }

    return HC_SUCCESS;
}



//测试AE无线数据读取
//usInterval -- 读取时间间隔,单位ms
//eWorkMode -- 工作模式
void get_wireless_ae_data_test( UINT16 usInterval, WorkMode eWorkMode )
{
    AEReadData stData;
    AEWorkSetForUp stWorkParam;

    //设置调理器地址
    WLDeviceAddr stAddr = {
    // {0xC1, 0x91, 0x54, 0x09, 0x00, 0x4B, 0x12, 0x00},//设备1
    {0x8B, 0x91, 0x54, 0x09, 0x00, 0x4B, 0x12, 0x00},//设备2
    0
    };
    set_wireless_addrlist(&stAddr, AE_TERMINAL);
    usleep(100000);

    //打开电源
    get_ae_data( &stData, AE_Wireless, AE_OPEN_POWER );

    //设置参数
    stWorkParam.eAE_Unit = mV;
    stWorkParam.eSyncSource = WIRELESS_SYNC;
    stWorkParam.eWorkMode = eWorkMode;
    stWorkParam.fAmplitudeThd = 1;
    stWorkParam.fGain = 100;
    stWorkParam.fPrpdThd = 0.1;
    stWorkParam.fPulseThd = 0.1;
    stWorkParam.fWaveThd = 0.1;
    stWorkParam.ucNgrid = 1;
    stWorkParam.usFComponent = 10;
    stWorkParam.usPrpdBlocking = 0;
    stWorkParam.usPulseBlocking = 0;
    stWorkParam.usPulseGating = 0;
    set_ae_settings( &stWorkParam, AE_Wireless );

    //开始采样
    get_ae_data( &stData, AE_Wireless, AE_START_SAMPLE );

    //读取数据
    while(1)
    {
        get_ae_data( &stData, AE_Wireless, AE_GET_DATA );
        usleep( usInterval*1000 );
    }
}


//测试TEV数据读取
//usInterval -- 读取时间间隔,单位ms
//eWorkMode -- 工作模式 0
void get_tev_data_test( UINT16 usInterval, TEVWorkMode eWorkMode )
{
    TEVWorkSettings stTevWorkParam;
    TEVReadData stTevData;
    
    //打开TEV电源
    get_tev_data( &stTevData, TEV_OPEN_POWER );

    //设置TEV参数
    stTevWorkParam.ucWorkMode = (UINT8)eWorkMode;
    stTevWorkParam.ucTime = 1;
    stTevWorkParam.ucBackGround = 10;
    stTevWorkParam.ucAlarm = 30;
    set_tev_settings( &stTevWorkParam );

    printf("Get TEV Data Test:\n");

    //读取TEV数据
    while(1)
    {
        get_tev_data( &stTevData, TEV_GET_DATA );
        switch( stTevData.ucWorkMode )
        {
            case MODE_AMP:
                printf("Mode: AMP, amp value = %d, MV value = %d\n",\
                stTevData.Data.AmpData.cAmpValue, stTevData.Data.AmpData.usTevMv);
            break;

            case MODE_PULSE:
                printf("Mode: PULSE, pulse num = %d, amp value = %d\n",\
                stTevData.Data.PulseData.uiPulseNum, stTevData.Data.PulseData.cAmpValue);
            break;

            case MODE_PRPS:
                printf("Mode: PRPS, max spectrum = %d, spectrum state = %d\n",\
                stTevData.Data.PrpsData.cMaxSpectrum, stTevData.Data.PrpsData.eSpectrumState);
                UINT16 i = 0;
                for(i = 0; i< TEV_PRPS_COUNT; i++)
                {
                    printf("%d ",stTevData.Data.PrpsData.cSpectrum[i]);
                    if( i % 10 == 0 )
                    {
                        printf("\n");
                    }
                }
                printf("\n");
            break;

            default:
            break;
        }

        usleep( usInterval*1000 );
    }
}


/**
 * @brief  读取电子标签信息
 * 
 * 该函数从EEPROM中读取电子标签的信息，并将其存储在提供的结构体中。
 * 
 * @param pElectronic_tag 指向电子标签结构体的指针，该结构体将被填充上从EEPROM中读取的数据。
 * @return 返回一个INT32类型的值，表示读取操作的结果。成功则返回正值，失败则返回负值。
 */
INT32 read_electronic_tag(EepromTag *pElectronic_tag)
{
    INT32 ret = 0;
    FrameBuf uart_comm_frame;
    UINT8 send_data_len = 0;
    UINT8 send_data_buf[10];
    UINT8 *pFrame_data;
    UINT16 recv_data_len = 0;

    if(NULL==pElectronic_tag)
    {
        return HC_FAILURE;
    }

    //组命令
    send_data_buf[send_data_len++] = COM_ELECTRONIC_TAG_READ;
    uart_comm_frame.uiRcvTimeout = TIMEOUT_1S;
    
    printf("Get Electronic Tag Info...\n");

    //发送报文
    ret = comm_with_stm32_handle(g_iFd_STM32, &uart_comm_frame, send_data_buf, send_data_len);
    if (ret != COM_OK)
    {
        comm_err_print(COM_ELECTRONIC_TAG_READ, (CommErr)(-ret));
        return HC_FAILURE;
    }
    //解析报文
    pFrame_data = get_frame_data(&uart_comm_frame, &recv_data_len);
  //  printf("Electronic Tag Info size:%d,receive real size:%d\n",sizeof(EepromTag),recv_data_len);
    if(pFrame_data[CMD_RESPONSE_CMD_POS] == COM_ELECTRONIC_TAG_READ)
    {
        memcpy(pElectronic_tag,pFrame_data+CMD_ELECTRONIC_TAG_DATA_POS,sizeof(EepromTag));
    }
    else
    {
        printf("Get Wrong Response Command:%d\n",pFrame_data[CMD_RESPONSE_CMD_POS]);
        return HC_FAILURE;
    }
    
    return HC_SUCCESS;
}
/**
 * @brief 写入电子标签信息到EEPROM
 * 
 * 该函数用于将电子标签（EepromTag）结构体中的数据写入到EEPROM中。
 * 
 * @param pElectronic_tag 指向电子标签结构体的指针，结构体中包含了需要写入EEPROM的数据。
 * @return INT32 返回写入操作的结果，成功返回0，失败返回非0值。
 */
INT32 write_electronic_tag(EepromTag *pElectronic_tag)
{
    INT32 ret = 0;
    FrameBuf uart_comm_frame;
    UINT8 send_data_len = 0;
    UINT8 send_data_buf[256];
    UINT8 *pFrame_data;
    UINT16 recv_data_len = 0;

    if(NULL==pElectronic_tag)
    {
        return HC_FAILURE;
    }

    //组命令
    send_data_buf[send_data_len++] = COM_ELECTRONIC_TAG_WRITE;
    memcpy(send_data_buf+send_data_len,pElectronic_tag,sizeof(EepromTag));
    send_data_len += sizeof(EepromTag);
    uart_comm_frame.uiRcvTimeout = TIMEOUT_1S;
    
    printf("Write Electronic Tag Info...\n");
    ret = comm_with_stm32_handle(g_iFd_STM32, &uart_comm_frame, send_data_buf, send_data_len);
    if (ret != COM_OK)
    {
        comm_err_print(COM_ELECTRONIC_TAG_WRITE, (CommErr)(-ret));
        return HC_FAILURE;
    }

    return HC_SUCCESS;
}
/**
 * @brief 计算电子标签的CRC校验值
 * 
 * 该函数用于根据电子标签（EepromTag）的内容计算并返回一个CRC校验值。
 * 
 * @param pElectronic_tag 指向电子标签结构体的指针。该结构体包含了需要进行CRC校验的数据。
 * @return 返回计算得到的CRC校验值，类型为UINT32。
 */
UINT32 calc_electronic_tag_crc(EepromTag *pElectronic_tag)
{
    UINT32 crc32 = 0;
    UINT32 electronic_tag_size = 0;
    electronic_tag_size = sizeof(EepromTag)-sizeof(pElectronic_tag->checksum);

    crc32 = cal_crc32((UINT8 *)pElectronic_tag,electronic_tag_size);
    return crc32;
}
