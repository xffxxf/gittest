/*
 * Copyright (c) 2015.11，南京华乘电气科技有限公司
 * All rights reserved.
 *
 * 文件名称：test.c
 *
 *
 * 初始版本：1.0
 * 作者：吴昌盛
 * 创建日期：2016年1月13日
 * 摘要：UHFHFCTAETEVApi.so的测试程序
 *
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "DataDefine.h"
#include "UHFHFCTAETEVApi.h"

#define  NEEDPRINTF

/****************************************/

void printfUHFHFCTData(UHFHFCTData *pstUHFHFCTData)
{
	UINT16 usi = 0;
	INT16 sTemp = 0;
	INT8 *pcTemp = NULL;

	printf("eBandWidth=%d eSpectrumState=%d eSyncSource=%d eSyncState=%d\n", (UINT8) pstUHFHFCTData->eBandWidth,
			(UINT8) pstUHFHFCTData->eSpectrumState, (UINT8) pstUHFHFCTData->eSyncSource,
			(UINT8) pstUHFHFCTData->eSyncState);
	printf("cMaxSpectrum=%d  cGain=%d\n", pstUHFHFCTData->cMaxSpectrum, pstUHFHFCTData->cGain);
	pcTemp = pstUHFHFCTData->caSpectrum;
	printf("\t\t\t");
	for (usi = 0; usi < SPECTTRUMNUM; usi++)
	{
		if ((usi + 3) % 5 == 0)
			printf("\n");
		printf("%d\t", *pcTemp++);
	}
	printf("\n");
}

void printfWLDeviceAddr(WLDeviceAddr *pstWLDeviceAddr, UINT16 usAddrNum)
{
	UINT16 usi = 0;
	UINT16 usj = 0;
	UINT8 *pucTemp = NULL;
	WLDeviceAddr *pstTemp = NULL;
	printf("get wireless Device Number:%d\n",usAddrNum);
	for (usj = 0; usj < usAddrNum; usj++)
	{
		pstTemp = (WLDeviceAddr*) (pstWLDeviceAddr + usj);
		pucTemp = pstTemp->caAddr;
		printf("	Device[%d] Addr:",usj);
		for (usi = 0; usi < ADDRBYTENUM; usi++)
		{
			printf("%3x ", *pucTemp++);
		}
		printf("\teAddrState=%d\n", (UINT8) pstTemp->eAddrState);
	}
}

void printfAEReadData(AEReadData *pstAEReadData)
{
	UINT16 usi = 0;
	UINT16 usLen = 0;
	UINT8 *pucTemp = NULL;
	float *fpTemp = NULL;

	printf("eSyncState=%d eSyncSource=%d eAE_CHANNEL=%d eAE_Gain=%f eAE_WorkMode=%d eAE_Unit=%d usPulseNum=%d\n",
			(UINT8) pstAEReadData->eSyncState, (UINT8) pstAEReadData->eSyncSource, pstAEReadData->eAE_CHANNEL,
			pstAEReadData->fGain, (UINT8) pstAEReadData->eWorkMode, (UINT8) pstAEReadData->eAE_Unit,
			pstAEReadData->usPulseNum);

	printf("uiSampleRate=%d uiADRange=%d ucADBits=%d ucADFormat=%d  ucPulseCount=%d\n", pstAEReadData->uiSamplingRate,
			pstAEReadData->uiADRange, pstAEReadData->ucADSampling, pstAEReadData->ucADSampDataFormat,
			pstAEReadData->usPulseNum);
	fpTemp = (float*) &pstAEReadData->Data;
	switch (pstAEReadData->eWorkMode)
	{
	case Amplitude:
		printf("fPeakValue=%f  fRMS=%f  fFirstFreqComValue=%f  fScendFreqComValue=%f \n", *fpTemp, *(fpTemp + 1),
				*(fpTemp + 2), *(fpTemp + 3));
		break;
	case PRPD:
		for (usi = 0; usi < pstAEReadData->usPulseNum; usi++)
		{
			printf("fPhaseValue=%f  fPeakValue=%f\n", *fpTemp, *(fpTemp + 1));
			fpTemp += 2;
		}
		break;
	case Pulse:
		for (usi = 0; usi < pstAEReadData->usPulseNum; usi++)
		{
			printf("uiPulseInterval=%d  fPeakValue=%f\n", *((UINT32*) fpTemp), *(fpTemp + 1));
			fpTemp += 2;
		}
		break;
	case Waveform:
		for (usi = 0; usi < AEWAVENUM; usi++)
		{
			if (usi % 5 == 0)
				printf("\n");
			printf("%4f\t", *fpTemp++);
		}
		break;
	}
	printf("\n");
}

INT32 get_ae_single_view( UINT8 ucWorkMode, UINT16 usSleepTime )
{
	AEReadData stAEView;
	AEWorkSetForUp stAEParam;
	int iRet = 0;
	float* fpTemp = (float*) &stAEView.Data;
	UINT16 usi = 0;

	//start ae power
	iRet = get_ae_data( NULL, AirSound, AE_OPEN_POWER );
	if( iRet )
	{
		printf("error in [%s]-start ae power: error code = %d\n",__func__, iRet);
		return iRet;		
	}

	//set ae param
	stAEParam.eWorkMode = ucWorkMode;
	stAEParam.eAE_Unit = mV;
	stAEParam.fGain = 100.0;
	stAEParam.eSyncSource = WIRELESS_SYNC;
	stAEParam.usFComponent = 50;
	stAEParam.ucNgrid = 2;
	stAEParam.fAmplitudeThd = 1;
	stAEParam.fPrpdThd = 1;
	stAEParam.fPulseThd = 1;
	stAEParam.fWaveThd = 1;
	stAEParam.usPrpdBlocking = 1;
	stAEParam.usPulseBlocking = 1;
	stAEParam.usPulseGating = 1;
#ifdef AE_FILTER_ENABLE
	stAEParam.eAEFilter = FILTER_80_300_kHZ;
#endif

	iRet = set_ae_settings(&stAEParam, AirSound);
	if( iRet )
	{
		printf("error in [%s]-set ae param: error code = %d\n",__func__, iRet);
		return iRet;
	}

	//start sample
	iRet = get_ae_data( NULL, AirSound, AE_START_SAMPLE );
	if( iRet )
	{
		printf("error in [%s]-start ae sample: error code = %d\n",__func__, iRet);
		// return iRet;
	}	

	while(1)
	{
		//read ae data
		iRet = get_ae_data( &stAEView, AirSound, AE_GET_DATA );
		if( iRet )
		{
			printf("error in [%s]-get ae single view data: error code = %d\n",__func__, iRet);
			continue;
		}		

		//add print here..
		switch (stAEView.eWorkMode)
		{
			case Amplitude:
			printf("fPeakValue=%f  fRMS=%f  fFirstFreqComValue=%f  fScendFreqComValue=%f \n", *fpTemp, *(fpTemp + 1),
					*(fpTemp + 2), *(fpTemp + 3));
			break;
			case PRPD:
			for (usi = 0; usi < stAEView.usPulseNum; usi++)
			{
				printf("fPhaseValue=%f  fPeakValue=%f\n", *fpTemp, *(fpTemp + 1));
				fpTemp += 2;
			}
			break;
			case Pulse:
			for (usi = 0; usi < stAEView.usPulseNum; usi++)
			{
				printf("uiPulseInterval=%d  fPeakValue=%f\n", *((UINT32*) fpTemp), *(fpTemp + 1));
				fpTemp += 2;
			}
			break;
			case Waveform:
			for (usi = 0; usi < AEWAVENUM; usi++)
			{
				if (usi % 5 == 0)
					printf("\n");
				printf("%4f\t", *fpTemp++);
			}
			break;
		}
		usleep( 1000 * usSleepTime );
	}

	return iRet;
	
}


INT32 get_ae_multi_views_test( UINT16 usSleepTime )
{
	AEAllView stAEView;
	AEWorkSetForUp stAEParam;
	int iRet = 0;
	UINT16 i = 0;

	//start ae power
	iRet = get_ae_data( NULL, AirSound, AE_OPEN_POWER );
	if( iRet )
	{
		printf("error in [%s]-start ae power: error code = %d\n",__func__, iRet);
		return iRet;		
	}

	//set ae param
	stAEParam.eWorkMode = AllViews;
	stAEParam.eAE_Unit = mV;
	stAEParam.fGain = 100.0;
	stAEParam.eSyncSource = WIRELESS_SYNC;
	stAEParam.usFComponent = 50;
	stAEParam.ucNgrid = 2;
	stAEParam.fAmplitudeThd = 1;
	stAEParam.fPrpdThd = 1;
	stAEParam.fPulseThd = 1;
	stAEParam.fWaveThd = 1;
	stAEParam.usPrpdBlocking = 2;
	stAEParam.usPulseBlocking = 2;
	stAEParam.usPulseGating = 100;
#ifdef AE_FILTER_ENABLE
	stAEParam.eAEFilter = 0;
#endif

	iRet = set_ae_settings(&stAEParam, AirSound);
	if( iRet )
	{
		printf("error in [%s]-set ae param: error code = %d\n",__func__, iRet);
		return iRet;
	}

	//start sample
	iRet = get_ae_data( NULL, AirSound, AE_START_SAMPLE );
	if( iRet )
	{
		printf("error in [%s]-start ae sample: error code = %d\n",__func__, iRet);
		return iRet;
	}	

	while(1)
	{
		//read ae data
		iRet = get_all_ae_views( &stAEView, AirSound );
		if( iRet )
		{
			printf("error in [%s]-get ae multi view data: error code = %d\n",__func__, iRet);
			continue;
		}		

		//add print here..
		// printf("Read AE Multi Views Print:\n");
		// printf("Gain = %f, Unit = %d, WorkMode = %d\nPhasePointNum = %d, PulsePointNum = %d, WavePointNum = %d\n",\
		// stAEView.fGain, stAEView.eAE_Unit, stAEView.eWorkMode, stAEView.usPhasePointNum, stAEView.usPulsePointNum, stAEView.usWavePointNum );
		// printf("Amp Data: peak = %f, rms = %f, h1 = %f, h2 = %f\n",\
		// stAEView.stAeAmp.fPeakValue, stAEView.stAeAmp.fRMS, stAEView.stAeAmp.fH1, stAEView.stAeAmp.fH2);
		
		for( i = 0; i < stAEView.usPulsePointNum; i++ )
		{
			printf("%d,%f\n",stAEView.staPulseData[i].uiPulseInterval,stAEView.staPulseData[i].fPeakValue);
		}

		usleep( 1000 * usSleepTime );
	}

	return iRet;

}


TevCaliCommand get_tev_cali_cmd(unsigned char ucStall)
{
	TevCaliCommand eCmd;

	switch(ucStall)
	{
		case 1://0~20dB
		{

			eCmd = TEV_CALI_GAIN50;
			break;
		}
		case 2://20~40dB
		{

			eCmd = TEV_CALI_GAIN5;

			break;
		}
		case 3://40~60dB
		{

			eCmd = TEV_CALI_GAIN1;

			break;
		}
	}
	return eCmd;
}



static void  print_dac_ref(UINT16 *pucDacRef)
{

	int i, j;
	for(i=TEV_CHANNEL_0dB;i<TEV_CHANNEL_COUNT;i++)
	{
		printf("TEV Channel:%d\n",i);
		for(j=0;j<20;j++)
		{
			if(j!=0 && j%10==0)
			{
				printf("\n");
			}
			printf("%2d ",pucDacRef[i*20 + j]);
		}
		printf("\n");
	}
	return;
}

static void print_electronic_tag(EepromTag *pElectronic_tag)
{
	char string_buf[32];
	
	printf("Electronic Tag:\n");
	printf("magicNumber:%x\n",pElectronic_tag->magicNumber);

	memset(string_buf,0,sizeof(string_buf));
	snprintf(string_buf,sizeof(string_buf),"%s",pElectronic_tag->hardwareModel);
	printf("hardwareModel:%s\n",string_buf);

	printf("hardwareVersion:V%d.%d\n",pElectronic_tag->hardwareVersion[1],pElectronic_tag->hardwareVersion[0]);
	printf("manufacturerID:0x%4x\n",pElectronic_tag->manufacturerID);

	memset(string_buf,0,sizeof(string_buf));
	snprintf(string_buf,sizeof(string_buf),"%s",pElectronic_tag->serialNumber);
	printf("serialNumber:%s\n",string_buf);

	memset(string_buf,0,sizeof(string_buf));
	snprintf(string_buf,sizeof(string_buf),"%s",pElectronic_tag->manufactureDate);
	printf("manufactureDate:%s\n",string_buf);
	//printf("checksum:%x\n",pElectronic_tag->checksum);
}
static void test_read_electronic_tag(void)
{
	EepromTag electronic_tag;
	memset(&electronic_tag, 0, sizeof(EepromTag));
	if(read_electronic_tag(&electronic_tag) == 0)
	{
		print_electronic_tag(&electronic_tag);
	}
	else
	{
		printf("Error in read electronic tag\n");
		
	}
	return ;
}

static void test_write_electronic_tag(int is_ae_637_hardware)
{
	EepromTag electronic_tag;
	memset(&electronic_tag, 0, sizeof(EepromTag));
	electronic_tag.magicNumber = 0xa5a5;
	if(is_ae_637_hardware)
	{
		electronic_tag.hardwareVersion[0] = 1;
		electronic_tag.hardwareVersion[1] = 0; // V1.0
		electronic_tag.manufacturerID = 0x1234;
		strncpy(electronic_tag.hardwareModel,"HW_Z159_SIG_R4",sizeof(electronic_tag.hardwareModel));
		strncpy(electronic_tag.serialNumber,"SN20190101",sizeof(electronic_tag.serialNumber));
		strncpy(electronic_tag.manufactureDate,"20190101",sizeof(electronic_tag.manufactureDate));
	}
	else
	{
		electronic_tag.hardwareVersion[0] = 1;
		electronic_tag.hardwareVersion[1] = 1; // V1.1
		electronic_tag.manufacturerID = 0x1234;

		strncpy(electronic_tag.hardwareModel,"HW_Z159_SIG_R4P1",sizeof(electronic_tag.hardwareModel));
		strncpy(electronic_tag.serialNumber,"SN20240101",sizeof(electronic_tag.serialNumber));
		strncpy(electronic_tag.manufactureDate,"20240101",sizeof(electronic_tag.manufactureDate));
	}
	
	electronic_tag.checksum = calc_electronic_tag_crc(&electronic_tag);

	print_electronic_tag(&electronic_tag);
  
	if((write_electronic_tag(&electronic_tag) == 0))
	{
		printf("Success in write electronic tag\n");
	}
	else
	{
		printf("Error in write electronic tag\n");
	}
}




int main(int argc, void *argv[])
{
	//调试用AE调理器地址
	WLDeviceAddr stAddr = {
	{0xC1, 0x91, 0x54, 0x09, 0x00, 0x4B, 0x12, 0x00},
	0
	};
	WLDeviceType eWLDeviceType;
	WLDeviceAddr *pstWLDeviceAddr = NULL;
	WLDeviceVer stWLDeviceVer;
	AEWorkSetForUp stAEWorkSetForUp;
	UHFHFCTData stUHFHFCTData;
	AEReadData stAEReadData;
	TEVWorkSettings stTEVWorkSettings;
	TEVReadData stTEVReadData;
	AE_ConnectState eAE_ConnectState;
	TEVCalibrateParam stTEVCalibrateParam;
	AE_ConnectState eConnectState;
	WLDeviceAddr staWireLessAddrList[10];
	WLDeviceAddr stAEWireLessAddr;
	UINT16 usWireLessAddrCount = 0;
	UINT16 usi = 0;
	UINT16 usLen = 0;
	float fTemp = 0;
	INT8 cTemp = 0;
	UINT16 ucTestTest = 0;
	UINT8 *pucTemp = NULL;
	const INT8 *pcTemp = NULL;
	INT32 iRet = 0;
	UINT8 ucaTest[100];
	INT32 fff = 0;
	float eee = 0;
	INT8 cTTT = 0;
	char* cSTM32Version;
	INT32 iBatValue;
	INT32 iTevMv1, iTevMv2, iTevMv3;
	R3CaliInfo stR3CaliInfo;

	if (argc == 1)
	{
		printf("testtest [function] [praram] [device type]\n");
		printf("[device type] ***0:UHF ***1:HFCT ***2:SYNCHRONIZER ***3:RECEIVER ***4:AE ***5:TEV ***6:cable current\n");
		printf("***1:get receiver version \n");
		printf("***2:get set receiver addr\n");
		printf("***3:set UHF gain0-20 1-0  HFCT gain 2-0 3-20 4-40 5-60\n");
		printf("***4:get UHf/HFCT data\n");
		printf("***5:set ae parram 0-amp 1-prpd 2-pluse 3-Waveform\n");
		printf("***6:set frequency\n");
		printf("***7:set syncsource 2-power 3-light\n");
		printf("***8:set UHF filer 0-all pass 1-high pass 2-low pass\n");
		printf("***9:calibrate ae\n");
		printf("**11:calibrate tev \n");
		printf("**12:update stm32 firmware\n");
		printf("**13:ae power 0-open 4-close 	\n");
		printf("**20:get stm32 version\n");
		printf("**21:get Device ID\n");
		printf("**22:get tev calibrate param\n");
		printf("**23:get ae connect status\n");
		printf("**24:read ae data\n");
		printf("**25:get stm32 version\n");
		printf("**26:get bat value\n");
		printf("**27:get tev value, 0-amp, 1-pulse, 2-prps:\n");
		printf("**30:r3 tev calibration:\n");
		printf("**31:get_ae_multi_views_test\n");
		printf("**32:get battery power\n");
		printf("**33:get_cable_current_data \n");
		printf("**34:update zigbee firmware\n");
		printf("**35:enumerate device address\n");
		printf("**36:read electronic tag info\n");
		printf("**37:write (AE 637 sample)electronic tag info\n");
		printf("**38:write (No AE 637 sample)electronic tag info\n");
		printf("libUHFHFCTAETEVApi.so version:%s\n",UHFHFCTAETEVApi_version());
		
		return 0;
	}

	switch (atoi(argv[3]))
	{
	case 0:
		eWLDeviceType = UHF_TERMINAL;
		break;
	case 1:
		eWLDeviceType = HFCT_TERMINAL;
		break;
	case 2:
		eWLDeviceType = SYNCHRONIZER;
		break;
	case 4:
		eWLDeviceType = AE_TERMINAL;
		break;
	case 5:
		eWLDeviceType = TEV_TERMINAL;
		break;
	case 6:
		eWLDeviceType = CABLE_CURRENT_TERMINAL;
		break;
	default:
		break;
	}

	if (init_uhfhfctaetev() < 0)
	{
		printf("faile init_uhfhfctaetev\n");
		return 0;
	}

	switch (atoi(argv[1]))
	{
	case 1:
		if (argc < 2)
			goto GORETURN;
		pstWLDeviceAddr = (WLDeviceAddr*) calloc(10, sizeof(WLDeviceAddr));
		//版本信息
		printf(" get_wireless_version RECEIVER \n\n");
		eWLDeviceType = RECEIVER;
		memset(stWLDeviceVer.caVersion, 0, VERSIONBYTENUM);
		iRet = get_wireless_version(&stWLDeviceVer, eWLDeviceType, NULL);
		for (cTemp = 0; iRet == 0 && cTemp < stWLDeviceVer.ucVerLen; cTemp++)
			printf("%c", stWLDeviceVer.caVersion[cTemp]);
		printf("\n");
		free(pstWLDeviceAddr);
		goto GORETURN;
	case 2:
		if (argc < 2)
			goto GORETURN;
		pstWLDeviceAddr = (WLDeviceAddr*) calloc(10, sizeof(WLDeviceAddr));
		//获取地址
		printf(" get_wireless_addrlist(&stWLDeviceAddr,&usi,HFCT_TERMINAL) \n\n");
		if (get_wireless_addrlist(pstWLDeviceAddr, &usi, eWLDeviceType) < 0)
		{
			goto GORETURN;
		}
		printfWLDeviceAddr(pstWLDeviceAddr, usi);

		for (ucTestTest = 0; ucTestTest < 3; ucTestTest++)
		{
			//设置地址
			printf(" set_wireless_addrlist(&stWLDeviceAddr1~8,HFCT_TERMINAL) \n\n");
			if (set_wireless_addrlist(pstWLDeviceAddr, eWLDeviceType) < 0)
				continue;
			break;
		}

		for (ucTestTest = 0; ucTestTest < 3; ucTestTest++)
		{
			//获取版本信息
			printf(" get_wireless_version \n\n");
			eWLDeviceType = EXTERNAL_DEVICE;
			memset(stWLDeviceVer.caVersion, 0, VERSIONBYTENUM);
			if (get_wireless_version(&stWLDeviceVer, eWLDeviceType, pstWLDeviceAddr) < 0)
				continue;
			for (cTemp = 0; cTemp < stWLDeviceVer.ucVerLen; cTemp++)
				printf("%d=%x \t", cTemp, stWLDeviceVer.caVersion[cTemp]);
			printf("\n");
			break;
		}

		free(pstWLDeviceAddr);
		goto GORETURN;
	case 3:
		//设置增益
		printf(" set_uhfhfct_gain((UHFHFCTGain)3,HFCT_TERMINAL) \n\n");
		set_uhfhfct_gain((UHFHFCTGain) atoi(argv[2]), eWLDeviceType);
		goto GORETURN;
	case 4:
		printf(" get_uhfhfct_data(&stUHFHFCTData,HFCT_TERMINAL) \n\n");
		UINT16 TTT = 0;
		for (ucTestTest = 0; ucTestTest < 1000; ucTestTest++)
		{
			if (get_uhfhfct_data(&stUHFHFCTData, eWLDeviceType) < 0)
			{
				TTT++;
				printf("********************failed ucTestTest=%d  AllNum=%d\n", ucTestTest, TTT);
			}
		}
		printfUHFHFCTData(&stUHFHFCTData);
		goto GORETURN;
	case 6:
		printf("set_frequency(FREQ_%d) \n\n", atoi(argv[2]));
		set_frequency((Frequency) atoi(argv[2]));
		goto GORETURN;
	case 7:
		printf("set_wireless_syncsource\n\n");
		set_wireless_syncsource((UHFHFCTGain) atoi(argv[2]), eWLDeviceType);
		goto GORETURN;
	case 8:
		printf("set_wireless_syncsource\n\n");
		set_uhf_filter((UHFFilterControl) atoi(argv[2]));
		goto GORETURN;
	case 5:
		printf("set_ae_settings\n\n");
		stAEWorkSetForUp.eAE_Unit = mV;
		stAEWorkSetForUp.eSyncSource = WIRELESS_SYNC;
		stAEWorkSetForUp.eWorkMode = atoi(argv[2]);
		stAEWorkSetForUp.fAmplitudeThd = 1;
		stAEWorkSetForUp.fGain = 100;
		stAEWorkSetForUp.fPrpdThd = 1;
		stAEWorkSetForUp.fPulseThd = 1;
		stAEWorkSetForUp.fWaveThd = 1;
		stAEWorkSetForUp.ucNgrid = 1;
		stAEWorkSetForUp.usFComponent = 10;
		stAEWorkSetForUp.usPrpdBlocking = 2000;
		stAEWorkSetForUp.usPulseBlocking = 100;
		stAEWorkSetForUp.usPulseGating = 0;
		set_ae_settings(&stAEWorkSetForUp, AirSound);
		//sleep(1);
		memset(&stAEReadData, 0, sizeof(stAEReadData));

		if (atoi(argv[2]) == (UINT8) PRPD || atoi(argv[2]) == (UINT8) Pulse)
		{
			get_ae_data(&stAEReadData, AirSound, 0);
			get_ae_data(&stAEReadData, AirSound, 1);
			for (ucTestTest = 0; ucTestTest < 100; ucTestTest++)
			{
				get_ae_data(&stAEReadData, AirSound, 2);
				//printfAEReadData(&stAEReadData);
			}
			get_ae_data(&stAEReadData, AirSound, 3);
			get_ae_data(&stAEReadData, AirSound, 4);
		}
		else
		{
			for (ucTestTest = 0; ucTestTest < 100; ucTestTest++)
			{
				get_ae_data(&stAEReadData, AirSound, 0);
				//printfAEReadData(&stAEReadData);
			}
			printfAEReadData(&stAEReadData);
		}

		goto GORETURN;
	case 10:

		goto GORETURN;
	case 9:
		for (ucTestTest = 1; ucTestTest < 10; ucTestTest++)
		{
			Calibrate_ae(&eee, ucTestTest);
			printf("fValue=%f  \n", eee);
			usleep(100000);
		}
		goto GORETURN;
	case 11:
	{
		char ucStall;
		char ucCaliDot;
		AutoCali stCali;
		TevCaliCommand eCmd;
		int i;
		char *pucStrCaliSta[] = 
		{
			"unCalibrate",
			"Calibrating",
			"Calibrate Success",
			"Calibrate Failed",
		};

		printf("Please Input Calibrate Stall(1:(0~20db)2:(20~40db)3:(40~60db))4:reset Calibrate params:5:read DAC val");
		ucStall = getchar();
		while(getchar()!='\n');
		ucStall -= '0';
		switch(ucStall)
		{
			case 1:
			case 2:
			case 3:
			{
					eCmd = get_tev_cali_cmd(ucStall);
					printf("ucStall:%d cali cmd:%d\n",ucStall,eCmd);
					if(Calibrate_tev(eCmd,&stCali)<0)
					{
					    printf("Calibrate failed!!\n");
					}
					else
					{

						printf("Calibrate Stall:%d-> state:%s,K:%f,B:%f\n",
						      ucStall,pucStrCaliSta[stCali.stParam.eCaliSta],
							  stCali.stParam.fK,stCali.stParam.fB);
					}
			
				break;
			}
			case 4:
			{
				if(Calibrate_tev(TEV_RESET_CALI_PARAM,&stCali)<0)
				{
					printf("Calibrate_tev: reset cali param failed\n");
				}
				else
				{
					printf("Reset Cali Param succeed!\n");
				}
				break;
			}
			case 5:
			{
				TevSample stTev;
				if(test_tev_sample(&stTev)<0)
				{
					printf("Calibrate_tev:get sample data failed\n");
				}
				break;
			}
			default:
			{
				printf("error calibrate stall:%c\n",ucStall+'0');
			}
		}

		goto GORETURN;
	}
	case 12:
	{
		printf("begin to update stm32 firmware:%s\n",CONDITIONERROGRM);
		update_conditioner(&cTemp);
		goto GORETURN;
	}
	case 13:
		get_ae_data(&stAEReadData, AirSound, atoi(argv[2]));
		goto GORETURN;
	case 20:
		printf("stm32 version: %s\n", stm32_version());
		goto GORETURN;
	case 21:
		pcTemp = device_id_num();
		if (pcTemp == NULL)
		{
			goto GORETURN;
		}
		for (ucTestTest = 0; ucTestTest < 8; ucTestTest++)
		{
			printf("%d ", pcTemp[ucTestTest]);
		}
		goto GORETURN;
	case 22:
		if (get_tev_calibrate_param(&stTEVCalibrateParam) == 0)
		{

			printf(" 0~20dB-> K:%f,B:%f,calbrate state:%d\n",stTEVCalibrateParam.CaliParam[TEV_CHANNEL_0dB].fK,
			               stTEVCalibrateParam.CaliParam[TEV_CHANNEL_0dB].fB,stTEVCalibrateParam.CaliParam[TEV_CHANNEL_0dB].eCaliSta);
			printf("20~40dB-> K:%f,B:%f,calbrate state:%d\n",stTEVCalibrateParam.CaliParam[TEV_CHANNEL_20dB].fK,
			               stTEVCalibrateParam.CaliParam[TEV_CHANNEL_20dB].fB,stTEVCalibrateParam.CaliParam[TEV_CHANNEL_20dB].eCaliSta);
			printf("40~60dB-> K:%f,B:%f,calbrate state:%d\n",stTEVCalibrateParam.CaliParam[TEV_CHANNEL_40dB].fK,
			               stTEVCalibrateParam.CaliParam[TEV_CHANNEL_40dB].fB,stTEVCalibrateParam.CaliParam[TEV_CHANNEL_40dB].eCaliSta);
			printf("Calibrate State:1->Calibrating 2->Calibrate Succeed,3->Calibrate Failed,0->Not Calibrate\n");

			UINT16 *pusDacRef = stTEVCalibrateParam.ausDacRef[TEV_CHANNEL_0dB];
			print_dac_ref(pusDacRef);
		}
		goto GORETURN;

	case 23:
		get_ae_connectstate( &eConnectState );
		if( AE_NotConnect == eConnectState )
		{
			printf("AE not connect\r\n");
		}
		else if( AE_Connected == eConnectState )
		{
			printf("AE connected!\r\n");
		}	
		goto GORETURN;

	case 24://读无线AE数据测试
		get_wireless_ae_data_test( 1000, atoi(argv[2]) );
		goto GORETURN;

	case 25://读STM32版本号测试
		cSTM32Version = malloc(24);
		if( cSTM32Version != NULL )
		{
			strncpy( cSTM32Version ,stm32_version(), 24 );
			printf("STM32 Version : %s\n", cSTM32Version);
		}
		else
		{
			printf("read stm32 version, pointer == NULL!\n");
		}
		free(cSTM32Version);
		goto GORETURN;

	case 26://读电量
		iBatValue = get_bat_power();
		printf("bat power = %d\n",iBatValue);
		goto GORETURN;

	case 27://读TEV数据
		get_tev_data_test( 1000, atoi(argv[2]) );
		goto GORETURN;

	case 29://读AE数据
		get_ae_single_view( atoi(argv[2]), 50 );
		goto GORETURN;

	case 31://读AE多图谱数据
		get_ae_multi_views_test( 50 );
	 	goto GORETURN;
	case 32:
	{
		INT32  iPwr;
		iPwr = get_bat_power();
		if(iPwr < 0)
		{
			printf("get battery power failed %d\n",iPwr);
		}
		else
		{
			printf("get battery power:%d\%\n",iPwr);
		}
		
		goto GORETURN;
	}
	case 33:
	{
		CableCurrentData stCableData;
		INT32 iRet = 0;
		printf("get cable current data test\n");

		iRet = get_cable_current_data(&stCableData);
		if(iRet < 0)
		{
			printf("get_cable_current_data failed :%d\n",iRet);
		}
		else
		{
			printf(" Cable Ground Current:%f\n",stCableData.fGroundCurrent);
			printf(" Cable Load Current :%f\n",stCableData.fLoadCurrent);
		}
		break;
	}
	case 34:
	{
		UINT8 ucPercent;
		printf("begin to update zigbee firmware:%s\n",ZIGBEEPROGRM);
		update_zigbee(&ucPercent);
		break;
	}
	case 35:
	{
		printf("begin to search wireless device,type:%d\n",eWLDeviceType);
		start_search_wireless(eWLDeviceType);
		break;
	}
	case 36:
	{
		test_read_electronic_tag();
		break;
	}
	case 37:
	{
		test_write_electronic_tag(1);
		break;
	}
	case 38:
	{
		test_write_electronic_tag(0);
		break;
	}

	default:
		break;
	}

	GORETURN: exit_uhfhfctaetev();
	return 0;
}
