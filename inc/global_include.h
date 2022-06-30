/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file global_include.h
 *@author gonghao
 *@date 2020/03/18
 *@brief ��ͷ�ļ������õ�ͷ�ļ������������������ļ��������Ҫ������Щ
 *ͷ�ļ���ֻ��Ҫinclude��ͷ�ļ�����
 *@version
 */

#ifndef INC_GLOBAL_INCLUDE_H_
#define INC_GLOBAL_INCLUDE_H_

using namespace std;

//C++��׼��
#include <stdio.h>
#include <stddef.h>
#include <assert.h>
#include <unistd.h>
#include <vector>
#include <map>
#include <stack>
#include <stdarg.h>
#include <pthread.h>
#include <sys/time.h>
#include <math.h>
#include <set>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <stdint.h>     //uint_32��ͷ�ļ�
#include <stdarg.h>     //�ɱ䳤�Ȳ���ͷ�ļ�
#include <errno.h>
#include <wchar.h>
#include <stddef.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <netinet/in.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <sys/stat.h>


//������ͷ�ļ�
#include "global_definition.h"
#include "global_structure.h"
#include "data_stack.h"
#include "list_buffer.h"
#include "circular_buffer.h"
#include "geometry_data.h"
#include "hmi_shared_data.h"
#include "trace.h"
#include "parm_manager.h"

//ǰ����������
class HMICommunication;
class MICommunication;
class MCCommunication;
class MCArmCommunication;
class ChannelEngine;
class ParmManager;
class TraceInfo;
class AlarmProcessor;
class ADCommunication;


//ȫ�ֱ������ⲿ����
extern SystemState  g_sys_state;   //ϵͳ״̬
extern SystemInfo g_sys_info;      //ϵͳ��Ϣ�������汾��CPUռ���ʵ�
extern int g_test;

extern HMICommunication *g_ptr_hmi_comm;    //HMIͨѶ�ӿ�
extern MICommunication *g_ptr_mi_comm; 		//MIͨѶ�ӿ�
extern MCCommunication *g_ptr_mc_comm;		//MCͨѶ�ӿ�
extern MCArmCommunication *g_ptr_mc_arm_comm;  //MC-ARMͨѶ�ӿ�
extern ChannelEngine *g_ptr_chn_engine;    //ͨ������
extern ParmManager *g_ptr_parm_manager;    //����������
extern TraceInfo *g_ptr_trace;             //��־���Խӿ�
extern AlarmProcessor *g_ptr_alarm_processor;        //�澯�������ӿ�

extern ADCommunication *g_ptr_ad_comm;     //�����豸ͨѶ�ӿ�

//ȫ�ֺ���
/*
 * @brief ����������Ϣ��
 * @param uint16_t axis_index, ���
 * @param ErrorType error_type, ��������
 * @param ErrorLevel error_level, ���󼶱�
 * @param channel_index, ͨ��������
 * @param ErrorClearType clear_type, �������
 * @param int32_t error_info, ��������
 * @return ��
 */
extern void CreateError(uint16_t error_code, uint8_t error_level,
		uint8_t clear_type, int32_t error_info = 0, uint8_t channel_index = CHANNEL_ENGINE_INDEX, uint8_t axis_index = NO_AXIS);

//�������״̬�������Ƿ����
extern bool CheckNetState(const char *net_name);

//��ȡ����ָ�����ӵ�IP��ַ
extern bool GetLocalIP(const char *eth_name, char *local_ip_addr);

//У��CRC
extern bool CheckCrc(const int16_t *data, const int len, const int16_t crc);

//����MI
extern bool StartMi();

//�ͷ�Linux�Ļ���
extern int DropCaches(int level);

//�ļ���������
extern int CopyFile(const char *src, const char *des);

//��ȡ����ʣ��ռ�
extern uint64_t GetFreeDisk();

//���ַ����е���ĸ��ת��Ϊ��д
extern void StrUpper(char *str);

#endif /* INC_GLOBAL_INCLUDE_H_ */
