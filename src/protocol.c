/*
 * protoco.c
 *
 *  Created on: 2016年8月20日
 *      Author: Administrator
 */


#include "protocol.h"
#include "DataDefine.h"
#include "UHFHFCTAETEVApi.h"
#include <sys/select.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "UHFHFCTAETEV.h"
#include "crc.h"



//与STM32通讯相关协议定义
typedef enum _ComWithArmProtocol
{
    //报文位置
    CWA_POS_HEADER_1 = 0,//头
    CWA_POS_HEADER_2,//头
    CWA_POS_LEN_H,//帧长高字节
    CWA_POS_LEN_L,//帧长低字节(注：帧长度不包括帧头)
    CWA_POS_DATA,//帧数据

    CWA_LEN_HEADER = CWA_POS_DATA,//帧头长度

    CWA_LEN_CRC = 1,
    CWA_LEN_CRC16 = 2,

    CWA_HEADER_1 = 0xA5,//报文头1
    CWA_HEADER_2 = 0x5A,//报文头2
}ComWithArmProtocol;

UINT8 * pucaCommErrStr[COM_MAX_ERR] = {
        "com ok",
        "com send error",
        "com recv timeout",
        "com poll error",
        "com frame header error",
        "com frame response state error",
        "com frame crc check error",
        "input parameter error",
        "com recevie data error",
        "com frame response command error"
};//串口通信错误码对应说明字符串数组

//check is frame header is valid
INT32 frame_header_valid(UINT8 *pucBuf)
{
    return(pucBuf[CWA_POS_HEADER_1]==CWA_HEADER_1 &&
            pucBuf[CWA_POS_HEADER_2]==CWA_HEADER_2);
}
//组装发送报文帧
void assemble_snd_frame(FrameBuf *pstFrame,UINT8 *pucBuf,UINT16 usDataLen)
{
    /*add frame data segment*/
    memcpy(&pstFrame->ucaSndBuf[CWA_POS_DATA],pucBuf,usDataLen);
    pstFrame->usSndNum = usDataLen;
    /*add frame header and crc*/
    pstFrame->ucaSndBuf[CWA_POS_HEADER_1] = CWA_HEADER_1;
    pstFrame->ucaSndBuf[CWA_POS_HEADER_2] = CWA_HEADER_2;
    pstFrame->ucaSndBuf[CWA_POS_LEN_H] = HI8(pstFrame->usSndNum+CWA_LEN_CRC);
    pstFrame->ucaSndBuf[CWA_POS_LEN_L] = LO8(pstFrame->usSndNum+CWA_LEN_CRC);
    pstFrame->usSndNum += CWA_LEN_HEADER;
    pstFrame->usSndNum += CWA_LEN_CRC;
    add_crc8(pstFrame->ucaSndBuf,pstFrame->usSndNum);
}

void data16_print(UINT16 *pusBuf,UINT16 usLen)
{
    UINT16 i;
    UINT16 max = 0;
    for(i=0;i<usLen;i++)
    {
        if(i!=0 && i%16==0)
            printf("\n");
        printf(" %04d",pusBuf[i]);
        max = max > pusBuf[i] ? max : pusBuf[i];
    }
    printf("\n");
    printf("max:%d\n",max);
}
void data_print(UINT8 *pucBuf,UINT16 usLen)
{
    UINT16 i;
    for(i=0;i<usLen;i++)
    {
        if(i!=0 && i%16==0)
            printf("\n");
        printf(" %03d",pucBuf[i]);
    }
    printf("\n");
}
void signed_data_print(INT8 *pBuf,UINT16 usLen)
{
    UINT16 i;
    for(i=0;i<usLen;i++)
    {
        if(i!=0 && i%16==0)
            printf("\n");
        printf(" %03d",pBuf[i]);
    }
    printf("\n");
}
void float_data_print(float *pBuf,UINT16 usLen)
{
    UINT16 i;
    for(i=0;i<usLen;i++)
    {
        if(i!=0 && i%16==0)
            printf("\n");
        printf(" %.1f",pBuf[i]);
    }
    printf("\n");
}
void frame_header_print(UINT8 *pucBuf,UINT16 usLen)
{
#ifdef UHFHFCTAETEV_DEBUG
    printf("Frame Header:\n");
    data_print(pucBuf,usLen);
#endif
}
void frame_recv_print(UINT8 *pucBuf,UINT16 usLen)
{
#ifdef UHFHFCTAETEV_DEBUG
    printf("Recv Frame:\n");
    data_print(pucBuf,usLen);
#endif
}
void frame_snd_print(FrameBuf *pstFrame)
{
#ifdef UHFHFCTAETEV_DEBUG
    printf("Send Frame:\n");
    data_print(pstFrame->ucaSndBuf,pstFrame->usSndNum);
  //  printf("Wait Response Timeout:%d ms\n",pstFrame->uiRcvTimeout);
#endif
}
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
void comm_err_print(UINT8 ucCmd,CommErr eErr)
{
    printf("Command:0x%x Err-->%s\n",ucCmd,pucaCommErrStr[eErr]);
}
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
UINT8 * get_frame_data(FrameBuf *pstFrame,UINT16 *pusDataLen)
{

    *pusDataLen = pstFrame->usRcvNum - CWA_LEN_CRC-CWA_LEN_HEADER;
    return &pstFrame->ucaRcvBuf[CWA_POS_DATA];
}


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
INT32 com_comm_handle(INT32 iComPort,FrameBuf *pstFrame,UINT8 *pucBuf,UINT16 usDataLen)
{
    fd_set rdfds;
    struct timeval timeout;
    INT32 iRet=0;
    INT32 iReadLen=0;
    UINT16 usWantedLen = 0;
    UINT16 usDataLeft = 0;
    UINT8 ucIsFrameHeader = 1;
    UINT16 usFrameLen = 0;
    UINT8 *pucData;
    if(!pstFrame || !pucBuf)
        return -COM_INPUTPARAM_ERR;
    if(usDataLen)//判断是否执行发送命令
    {
        //清除缓存
        read_uart_clear(iComPort, pstFrame->ucaRcvBuf, 1);
        //组装报文帧
        assemble_snd_frame(pstFrame,pucBuf,usDataLen);
        //发送报文
        frame_snd_print(pstFrame);
        if(write_uart(iComPort,pstFrame->ucaSndBuf,pstFrame->usSndNum) != HC_SUCCESS)
        {
            return -COM_SND_ERR;
        }

    }
    usDataLeft = usWantedLen = CWA_LEN_HEADER;
    pucData = pstFrame->ucaRcvBuf;

    while(usDataLeft)
    {
        timeout.tv_sec  = (pstFrame->uiRcvTimeout/1000);
        timeout.tv_usec = (pstFrame->uiRcvTimeout%1000)*1000;
        FD_ZERO(&rdfds);
        FD_SET(iComPort,&rdfds);
        iRet = select(iComPort+1,&rdfds,NULL,NULL,&timeout);
        if(0>iRet)
        {
            if(errno == EINTR)
                continue;//忽略信号中断系统调用错误
#ifdef UHFHFCTAETEV_DEBUG
            printf("%s:select error %s!!\n",__func__,strerror(errno));
#endif
            return -COM_POLL_ERR;
        }
        if(0==iRet)
        {
#ifdef UHFHFCTAETEV_DEBUG
            printf("Wait Response Timeout:%d ms\n",pstFrame->uiRcvTimeout);
#endif
            return -COM_RCV_TIMEOUT;
        }
        iReadLen = read_uart_nbyte(iComPort,pucData,usWantedLen);

        if(iReadLen >= 0 && iReadLen <= usWantedLen)
        {
            usDataLeft -= iReadLen;
            usWantedLen -= iReadLen;
            pucData  += iReadLen;

            if(0 == usWantedLen)//header or data receive completed
            {
                if(ucIsFrameHeader)
                {

                    if(frame_header_valid(pstFrame->ucaRcvBuf))
                    {
                        usWantedLen = U8TOU16(pstFrame->ucaRcvBuf[CWA_POS_LEN_L],pstFrame->ucaRcvBuf[CWA_POS_LEN_H]);
                        usDataLeft  = usWantedLen;
                        usFrameLen = CWA_LEN_HEADER + usWantedLen;
                        ucIsFrameHeader = 0;
                    }
                    else
                    {
                        frame_header_print(pstFrame->ucaRcvBuf,CWA_LEN_HEADER);
                        return -COM_HEADER_ERR;
                    }
                }
                else
                {
                    //data receive completed,jump out loop...
                }
            }
            else
            {
                //header or data receive is not completed,continue...
            }

        }
        else
        {
            return -COM_RCV_ERR;
            //error
        }
    }

    pstFrame->usRcvNum = usFrameLen<COMM_BUF_MAX_NUM?usFrameLen:COMM_BUF_MAX_NUM;
    frame_recv_print(pstFrame->ucaRcvBuf,pstFrame->usRcvNum);
    //报文crc校验
    if(check_crc8(pstFrame->ucaRcvBuf,pstFrame->usRcvNum) != HC_SUCCESS)
    {
        return -COM_CRC_ERR;
    }

    return COM_OK;
}

/************************************************
 * 函数名   ：zigbee_iap

 * 输入参数 :iComPort -- 串口描述符
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
INT32 zigbee_iap(INT32 iComPort,FrameBuf *pstFrame,UINT8 *pucBuf,UINT16 usDataLen)
{
    fd_set rdfds;
    struct timeval timeout;
    INT32 iRet=0;
    INT32 iReadLen=0;
    UINT16 usWantedLen = 0;
    UINT16 usDataLeft = 0;
    UINT8 ucIsFrameHeader = 1;
    UINT16 usFrameLen = 0;
    UINT8 *pucData;
    UINT16 crc = 0;
    if(!pstFrame || !pucBuf)
        return -COM_INPUTPARAM_ERR;

    //清除缓存
    read_uart_clear(iComPort, pstFrame->ucaRcvBuf, 1);
    //组装报文帧
    /*add frame data segment*/
    memcpy(&pstFrame->ucaSndBuf[CWA_POS_DATA],pucBuf,usDataLen);
    pstFrame->usSndNum = usDataLen;
    /*add frame header and crc*/
    pstFrame->ucaSndBuf[CWA_POS_HEADER_1] = CWA_HEADER_1;
    pstFrame->ucaSndBuf[CWA_POS_HEADER_2] = CWA_HEADER_2;
    pstFrame->ucaSndBuf[CWA_POS_LEN_H] = HI8(pstFrame->usSndNum+CWA_LEN_CRC16);
    pstFrame->ucaSndBuf[CWA_POS_LEN_L] = LO8(pstFrame->usSndNum+CWA_LEN_CRC16);
    pstFrame->usSndNum += CWA_LEN_HEADER;
    pstFrame->usSndNum += CWA_LEN_CRC16;
    crc = cal_crc16(pstFrame->ucaSndBuf, pstFrame->usSndNum-CWA_LEN_CRC16);//最后两个字节不参与CRC16计算
    pstFrame->ucaSndBuf[pstFrame->usSndNum-2] = M16TO8_L(crc);
    pstFrame->ucaSndBuf[pstFrame->usSndNum-1] = M16TO8_H(crc);
    //发送报文
    frame_snd_print(pstFrame);
    if(write_uart(iComPort,pstFrame->ucaSndBuf,pstFrame->usSndNum) != HC_SUCCESS)
    {
        return -COM_SND_ERR;
    }

    //接收返回报文
    usDataLeft = usWantedLen = CWA_LEN_HEADER;
    pucData = pstFrame->ucaRcvBuf;

    while(usDataLeft)
    {
        timeout.tv_sec  = (pstFrame->uiRcvTimeout/1000);
        timeout.tv_usec = (pstFrame->uiRcvTimeout%1000)*1000;
        FD_ZERO(&rdfds);
        FD_SET(iComPort,&rdfds);
        iRet = select(iComPort+1,&rdfds,NULL,NULL,&timeout);
        if(0>iRet)
        {
            if(errno == EINTR)
                continue;//忽略信号中断系统调用错误
#ifdef UHFHFCTAETEV_DEBUG
            printf("%s:select error %s!!\n",__func__,strerror(errno));
#endif
            return -COM_POLL_ERR;
        }
        if(0==iRet)
        {
#ifdef UHFHFCTAETEV_DEBUG
            printf("Wait Response Timeout:%d ms\n",pstFrame->uiRcvTimeout);
#endif
            return -COM_RCV_TIMEOUT;
        }
        iReadLen = read_uart_nbyte(iComPort,pucData,usWantedLen);

        if(iReadLen >= 0 && iReadLen <= usWantedLen)
        {
            usDataLeft -= iReadLen;
            usWantedLen -= iReadLen;
            pucData  += iReadLen;

            if(0 == usWantedLen)//header or data receive completed
            {
                if(ucIsFrameHeader)
                {

                    if(frame_header_valid(pstFrame->ucaRcvBuf))
                    {
                        usWantedLen = U8TOU16(pstFrame->ucaRcvBuf[CWA_POS_LEN_L],pstFrame->ucaRcvBuf[CWA_POS_LEN_H]);
                        usDataLeft  = usWantedLen;
                        usFrameLen = CWA_LEN_HEADER + usWantedLen;
                        ucIsFrameHeader = 0;
                    }
                    else
                    {
                        frame_header_print(pstFrame->ucaRcvBuf,CWA_LEN_HEADER);
                        return -COM_HEADER_ERR;
                    }
                }
                else
                {
                    //data receive completed,jump out loop...
                }
            }
            else
            {
                //header or data receive is not completed,continue...
            }

        }
        else
        {
            return -COM_RCV_ERR;
            //error
        }
    }

    pstFrame->usRcvNum = usFrameLen<COMM_BUF_MAX_NUM?usFrameLen:COMM_BUF_MAX_NUM;
    frame_recv_print(pstFrame->ucaRcvBuf,pstFrame->usRcvNum);

    //CRC16校验
    crc = cal_crc16(pstFrame->ucaRcvBuf,pstFrame->usRcvNum - 2);
    if(M8TO16(pstFrame->ucaRcvBuf[pstFrame->usRcvNum-2],pstFrame->ucaRcvBuf[pstFrame->usRcvNum-1])!=crc)
    {
        return -COM_CRC_ERR;
    }
    return COM_OK;
}
