/*
* Copyright (c) 2015.11，南京华乘电气科技有限公司
* All rights reserved.
*
* 文件名称：protocol.h
*
*
* 初始版本：1.0
* 作者：吴昌盛
* 创建日期：2016年1月13日
* 摘要：UHF HFCT AE TEV通讯协议的相关定义头文件
* 
*/

#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "DataDefine.h"

/***************************************************/
/******************* 命令字定义 *******************/

#define READ_UHF_COMMAND        (0)     //读UHF数据

#define CONTROL_CMMAND          (1)     //控制命令

#define SET_UHF_ADDR            (2)     //设置UHF地址
#define START_UHF_ADDR          (3)     //开始枚举UHF地址
#define FIRST_UHF_ADDR          (4)     //获取第一个UHF地址
#define NEXT_UHF_ADDR           (5)     //获取下一个UHF地址

#define SET_SYNCER_ADDR         (6)     //设置同步器地址
#define START_SYNCER_ADDR       (7)     //开始枚举同步器地址
#define FIRST_SYNCER_ADDR       (8)     //获取第一个同步器地址
#define NEXT_SYNCER_ADDR        (9)     //获取下一个同步器地址

#define DATA_TYPE_COMMAND       (10)    //控制数据类型

#define READ_HFCT_COMMAND       (11)    //读HFCT数据

#define SET_HFCT_ADDR           (12)    //设置HFCT地址
#define START_HFCT_ADDR         (13)    //开始枚举HFCT地址
#define FIRST_HFCT_ADDR         (14)    //获取第一个HFCT地址
#define NEXT_HFCT_ADDR          (15)    //获取下一个HFCT地址

#define SET_FREQUENCE           (16)    //设置电网周期

#define SET_AE_ADDR             (17)    //设置AE地址
#define START_AE_ADDR           (18)    //开始枚举AE地址
#define FIRST_AE_ADDR           (19)    //获取第一个AE地址
#define NEXT_AE_ADDR            (20)    //获取下一个AE地址

#define AE_SETTINGS             (21)    //设置超声波工作参数
#define AE_READ_DATA            (22)    //读超声波数据
#define AE_CONNECT_STATE        (23)    //获取超声波连接状态

#define TEV_SETTINGS            (28)    //设置低电波工作参数
#define TEV_READ_DATA           (29)    //获取低电波数据

#define AETEV_FREQUENCE         (30)    //设置AE TEV电网周期

#define Calibrate_AE            (31)    //校准AE
#define Calibrate_TEV           (32)    //校准TEV

#define BATTEERY_POWER          (33)    //获取电池电量

#define TEV_VOLTAGE_VALUE       (40)    //获取TEV毫伏值

#define STM32VERSION          	(42)    //stm32版本
#define STM32DEVICEID          	(43)    //设备序列号

#define STM32TEVPARAM          	(44)    //TEV参数

#define COM_AE_READ_MULTI_VIEWS (45)    //读AE多图谱数据


#define READ_CABLE_CURRENT_COMMAND     (46) //读取电缆电流命令
#define SET_CABLE_CURRENT_ADDR         (47)  //设置电缆电流调理器地址
#define ENUMERATE_CABLE_CURRENT_ADDR   (48) //开始枚举电缆电流调理器地址
#define GET_FIRST_CABLE_CURRENT_ADDR   (49) //获取第一个电缆电流调理器地址
#define GET_NEXT_CABLE_CURRENT_ADDR    (50)  //获取下一个电缆电流调理器地址

#define COM_ELECTRONIC_TAG_READ       (51)  // 读电子标签数据
#define COM_ELECTRONIC_TAG_WRITE      (52)  // 写电子标签数据
  



#define UPDATECONDITINEER       (128)   //调解器升级 

#define UPDATE_ZIGBEE_CMD       (0xE0)  //升级zigbee命令
#define UPDATE_ZIGBEE_SELF      (0x01)  //自定义命令类型
#define UPDATE_ZIGBEE_ERASE     (0xE1)  //擦除子命令
#define UPDATE_ZIGBEE_DATA      (0xE2)  //传输数据子命令
#define UPDATE_ZIGBEE_CHECK     (0xE3)  //校验程序
#define UPDATE_ZIGBEE_OVER      (0xE4)  //结束
#define UPDATE_ZIGBEE_ADDR      (0)     //升级zigbee设备地址

#define RECEIVER_VERSION        (0xf0)  //获取接收器版本
#define EXTERN_DEV_VERSION      (0xf2)  //获取外部设备版本

/***************************************************/
/******************* 子命令字定义 *******************/

#define UHF_FILTER_CMMAND       (1)     //UHF滤波控制
#define UHF_GAIN_CMMAND         (2)     //UHF增益设置
#define UHF_SYNC_CMMAND         (3)     //UHF同步设置
#define HFCT_GAIN_CMMAND        (0x12)  //HFCT增益设置
#define HFCT_SYNC_CMMAND        (0x13)  //HFCT同步设置
#define AE_SYNC_CMMAND          (0x23)  //AE同步设置


/***************************************************/
/******************* 报文字节定义 *******************/

#define COMMAND_BYTE            (0)     //命令字 字节位置
#define SUB_COMMAND_BYTE        (1)     //子命令 字节位置
#define DATA_LEN_BYTE           (2)     //数据长度 字节位置
#define UPDATE_DATA_BYTE        (4)     //升级数据 字节位置
#define UPDATE_PROGRAM_BYTE     (12)    //升级数据 字节位置
#define TYPECTL_BYTE            (1)     //数据类型控制 字节位置
#define DATA_BYTE               (3)     //数据开始 字节位置
#define CMD_DATA_BYTE           (2)     //命令中数据 字节位置
#define ADDR_BYTE               (2)     //地址 字节位置
#define VERSION_BYTE            (2)     //版本 字节位置
#define CMD_ADDR_BYTE           (1)     //命令中地址 字节位置
#define ADDR_STATE_BYTE         (10)    //地址状态 字节位置
#define RECEIVE_STATE_BYTE      (1)     //接收返回结果 字节位置
#define STATE_BYTE              (2)     //状态 字节位置
#define AE_PARAM_BYTE           (2)     //AE参数字节

/***************************************************/
/******************* 常量 **************************/

#define COMM_BUF_MAX_NUM        (8192)  //报文通讯缓存最大
#define SEND_COMMMAND_NUM       (2)     //发送命令帧长度
#define SEND_DATATYPE_NUM       (3)     //发送数据类型帧长度
#define SUB_COMMMAND_NUM        (4)     //发送子命令帧长度
#define AE_CONSTATE_NUM         (4)     //超声波发送连接状态命令帧长度
#define TEV_COMMON_NUM          (4)     //低电波接收帧除数据外的长度
#define UPDATE_NUM              (5)     //低电波接收帧除数据外的长度
#define UPDATE_LOAD_NUM         (6)     //低电波接收帧除数据外的长度
#define TEV_CALIDATA_NUM        (8)     //低电波接收帧除数据外的长度
#define TEV_CALI_NUM            (9)     //低电波接收帧除数据外的长度
#define READ_DATA_NUM           (64)    //读数据接收帧长度
#define RECEIVE_CMD_NUM         (3)     //接收命令帧长度
#define RECEIVE_ADDR_NUM        (12)    //接收地址帧长度
#define RECEIVE_SUCCESS         (1)     //接收帧返回结果成功
#define INCALIBRATE             (1)     //校准中
#define CALIBRATESUCCESS        (2)     //校准成功
#define INCALIBRATEBAD          (3)     //校准失败
#define CONDITION_ADDR_NUM      (4)     //调理器程序地址字节数
#define TEV_CALIBRATE_NUM      	(28)    //TEV校准参数字节数

/***************************************************/
/******************* 位定义 ***********************/

#define DATATYPE_UHFBIT         (0)     //UHF数据类型位
#define DATATYPE_CTBIT          (1)     //HFCT数据类型位
#define DATATYPE_AEBIT          (2)     //AE数据类型位
#define DATATYPE_CABLEBIT       (3)     //电缆电流数据类型位
#define BIT0                    (0)
#define BIT1                    (1)
#define BIT2                    (2)
#define BIT3                    (3)
#define BIT4                    (4)
#define BIT5                    (5)
#define BIT6                    (6)
#define BIT7                    (7)


/****added for new uart communication protocol*******/


#define U8TOU32(ucLL,ucLH,ucHL,ucHH) ( (ucLL) + ((ucLH)<<8) + ((ucHL)<<16) + ((ucHH)<<24) )
#define U8TOU16(ucL,ucH) ( ( (ucL) & 0xFF ) + ( ( (ucH) & 0xFF ) << 8 ) )

#define LO8(u16) ((u16)&0xFF)
#define HI8(u16) (((u16)>>8)&0xFF)
#define LL8(u32) ((u32)&0xFF)
#define LH8(u32) (((u32)>>8)&0xFF)
#define HL8(u32) (((u32)>>16)&0xFF)
#define HH8(u32) (((u32)>>24)&0xFF)

typedef struct _FrameBuf
{
    UINT8 ucaSndBuf[COMM_BUF_MAX_NUM];//报文发送缓冲区
    UINT8 ucaRcvBuf[COMM_BUF_MAX_NUM];//报文接收缓冲区
    UINT16 usSndNum;//报文发送字节数
    UINT16 usRcvNum;//报文接收字节数
    UINT32 uiRcvTimeout;//报文接收超时时间（毫秒）
}FrameBuf;
typedef enum _CommErr
{
    COM_OK = 0,//报文正确发送/接收
    COM_SND_ERR = 1,//发送报文失败
    COM_RCV_TIMEOUT,//接收报文超时
    COM_POLL_ERR,//select系统调用错误
    COM_HEADER_ERR,//报文帧头错误
    COM_RESPONSE_STATE_ERR,//响应报文状态字错误
    COM_CRC_ERR,//crc校验错误
    COM_INPUTPARAM_ERR,//入参错误
    COM_RCV_ERR,//接收错误
    /*add any new err code here*/
    COM_COMMAND_BYTE_ERR,//命令字错误

    /*err code end */
    COM_MAX_ERR,
    
}CommErr;
/*响应报文定义*/
#define CMD_RESPONSE_CMD_POS         (0)
#define CMD_RESPONSE_STATE_POS       (CMD_RESPONSE_CMD_POS+1)
	#define CMD_RESPONSE_STATE_OK        (1)
	#define CMD_RESPONSE_STATE_ERR   (0)
#define CMD_RESPONSE_STATE_PARAM_POS (CMD_RESPONSE_STATE_POS+1)
	#define STATE_PARAM_NO_DATA   (0XFF)
	#define STATE_PARAM_DEV_CONFLICT (1)
//#define CMD_RESPONSE_STATE_ERR       (2)
#define CMD_RESPONSE_COMMON_LEN      (2)
#define CMD_ZIGBEE_IAP_STATUS        (4)
#define CMD_RESPONSE_AE_PARAM_POS    (CMD_RESPONSE_STATE_POS+1)
#define CMD_RESPONSE_AE_COMMON_LEN   (CMD_RESPONSE_AE_PARAM_POS)
#define CMD_RESPONSE_TEV_WORKMOD_POS (CMD_RESPONSE_STATE_POS+1)
#define CMD_RESPONSE_TEV_DATA_POS    (CMD_RESPONSE_TEV_WORKMOD_POS+1)
#define CMD_RESPONSE_TEV_CALI_SUBCMD_POS   (CMD_RESPONSE_STATE_POS+1)
#define CMD_RESPONSE_TEV_CALI_DATA_POS     (CMD_RESPONSE_TEV_CALI_SUBCMD_POS+1)
#define CMD_RESPONSE_TEV_COMMON_LEN  (CMD_RESPONSE_TEV_DATA_POS)
#define CMD_RESPONSE_AE_CALIBRATE_DATA_POS (CMD_RESPONSE_STATE_POS+1)
#define CMD_RESPONSE_BATTERA_POS     (CMD_RESPONSE_STATE_POS+1)
#define CMD_RESPONSE_STM32VER_POS    (CMD_RESPONSE_STATE_POS+1)
#define CMD_RESPONSE_DEVICEID_POS    (CMD_RESPONSE_STATE_POS+1)
#define CMD_RESPONSE_TEV_PARAM_POS   (CMD_RESPONSE_STATE_POS+1)
#define CMD_RESPONSE_TEV_COEFFICIENT_POS (CMD_RESPONSE_STATE_POS+2)
#define CMD_RESPONSE_TEV_CALI_VALUE_POS  (CMD_RESPONSE_TEV_COEFFICIENT_POS+4)
#define CMD_RESPONSE_TEV_CALI_STATUS_POS (CMD_RESPONSE_TEV_CALI_VALUE_POS+1)
#define CMD_RESPONSE_TEV_VOLTAGE_VALUE_POS  (CMD_RESPONSE_STATE_POS + 1)
#define CMD_AE_CONNECT_STATUS_POS    (CMD_RESPONSE_STATE_POS+1)
#define CMD_AE_MULTIVIEW_DATA_POS    ( CMD_RESPONSE_STATE_POS + 1 )
#define CMD_ELECTRONIC_TAG_DATA_POS  (CMD_RESPONSE_STATE_POS+1)

#define CMD_RESPONSE_CABEL_GROUD_CURRENT_POS (CMD_RESPONSE_STATE_POS+2)
#define CMD_RESPONSE_CABEL_LOAD_CURRENT_POS (CMD_RESPONSE_CABEL_GROUD_CURRENT_POS+4)

//stm32固件升级命令相关定义
#define IAP_BOOT_APP_SWITCH_OK       (1)
#define IAP_BOOT_APP_SWITCH_ERR      (2)
#define IAP_COMMAND_POS              (0)
#define IAP_SUBCOMMAND_POS           (1)
#define IAP_DATALENHI_POS            (2)
#define IAP_DATALENLO_POS            (3)
#define IAP_COMMON_DATA_LEN          (4)
#define IAP_PROGRAME_ADDR_POS        (4)
#define IAP_PROGRAME_ADDR_LEN        (4)
#define IAP_PROGRAME_DATA_POS        (IAP_PROGRAME_ADDR_POS+IAP_PROGRAME_ADDR_LEN)
#define IAP_PROGRAME_DATA_LEN        (1024)
#define IAP_DATA_LEN                 (IAP_PROGRAME_ADDR_LEN+IAP_PROGRAME_DATA_LEN)
#define IAP_PROGRAME_FRAME_LEN       (IAP_COMMON_DATA_LEN+IAP_DATA_LEN)
#define CMD_RESPONSE_IAP_DATA_POS    (5)
#define IAP_PROGRAME_OK              (1)
#define IAP_PROGRAME_ERR             (2)


#define TIMEOUT_20MS (20)
#define TIMEOUT_50MS (50)
#define TIMEOUT_100MS (100)
#define TIMEOUT_200MS (200)
#define TIMEOUT_500MS (500)
#define TIMEOUT_800MS (800)
#define TIMEOUT_1S    (1000)
#define TIMEOUT_5S    (5000)
#define TIMEOUT_10S   (10000)

/************************************************
 * 函数名   ：get_frame_data

 * 输入参数 ：pstFrame --帧结构指针
 *
 * 输出参数 ：
 *         *pusDataLen  --报文帧数据段字节长度
 *
 * 返回值   ： 指向报文帧数据段缓冲区的指针
 *
 * 功能     ：获取接收报文帧的数据段长度和缓冲区指针
 ************************************************/
UINT8 * get_frame_data(FrameBuf *pstFrame,UINT16 *pusDataLen);

/************************************************
 * 函数名   ：com_comm_handle

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
 * 功能     ：串口通讯的报文发送/接收处理
 ************************************************/
INT32 com_comm_handle(INT32 iComPort,FrameBuf *pstFrame,UINT8 *pucBuf,UINT16 usDataLen);
/************************************************
 * 函数名   ：com_zigbee_iap_handle

 * 输入参数 :
 *         pucBuf --发送报文数据段缓冲区指针
 *         usDataLen  --发送报文数据段字节长度, 
 *         pstFrame->uiRcvTimeout --接收报文超时时间（毫秒）
 *
 * 输出参数 ：pstFrame->ucaRcvBuf --接收缓冲区指针
 *         pstFrame->UIRcvNum  --接收报文字节长度
 *
 * 返回值   ： 参见CommErr定义
 *
 * 功能     ：ZIGBEE固件升级的报文发送/接收处理
 ************************************************/
 INT32 zigbee_iap(INT32 iComPort,FrameBuf *pstFrame,UINT8 *pucBuf,UINT16 usDataLen);
/************************************************
 * 函数名   ：comm_err_print

 * 输入参数 ：ucCmd --报文命令字
 *         eErr --报文传输错误码
 * 输出参数 ：NULL
 *
 * 返回值   ： NULL
 *
 * 功能     ：打印报文传输错误信息
 ************************************************/
void comm_err_print(UINT8 ucCmd,CommErr eErr);

void float_data_print(float *pBuf,UINT16 usLen);
void signed_data_print(INT8 *pBuf,UINT16 usLen);
void data_print(UINT8 *pucBuf,UINT16 usLen);
void data16_print(UINT16 *pusBuf,UINT16 usLen);

/****end of new uart communication protocol*********/
#endif
