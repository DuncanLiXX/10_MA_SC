/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file global_include.h
 *@author gonghao
 *@date 2020/03/18
 *@brief 本头文件将共用的头文件集中起来管理，其他文件中如果需要包含这些
 *头文件，只需要include此头文件即可
 *@version
 */

#ifndef INC_GLOBAL_INCLUDE_H_
#define INC_GLOBAL_INCLUDE_H_

using namespace std;

//C++标准库
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
#include <stdint.h>     //uint_32等头文件
#include <stdarg.h>     //可变长度参数头文件
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


//工程内头文件
#include "global_definition.h"
#include "global_structure.h"
#include "data_stack.h"
#include "list_buffer.h"
#include "circular_buffer.h"
#include "geometry_data.h"
#include "hmi_shared_data.h"
#include "trace.h"
#include "parm_manager.h"

//前置类型声明
class HMICommunication;
class MICommunication;
class MCCommunication;
class MCArmCommunication;
class ChannelEngine;
class ParmManager;
class TraceInfo;
class AlarmProcessor;
class ADCommunication;


//全局变量的外部声明
extern SystemState  g_sys_state;   //系统状态
extern SystemInfo g_sys_info;      //系统信息，包括版本和CPU占用率等
extern int g_test;

extern HMICommunication *g_ptr_hmi_comm;    //HMI通讯接口
extern MICommunication *g_ptr_mi_comm; 		//MI通讯接口
extern MCCommunication *g_ptr_mc_comm;		//MC通讯接口
extern MCArmCommunication *g_ptr_mc_arm_comm;  //MC-ARM通讯接口
extern ChannelEngine *g_ptr_chn_engine;    //通道引擎
extern ParmManager *g_ptr_parm_manager;    //参数管理器
extern TraceInfo *g_ptr_trace;             //日志调试接口
extern AlarmProcessor *g_ptr_alarm_processor;        //告警处理器接口

extern ADCommunication *g_ptr_ad_comm;     //辅助设备通讯接口

//全局函数
/*
 * @brief 创建错误信息包
 * @param uint16_t axis_index, 轴号
 * @param ErrorType error_type, 错误类型
 * @param ErrorLevel error_level, 错误级别
 * @param channel_index, 通道索引号
 * @param ErrorClearType clear_type, 清除级别
 * @param int32_t error_info, 错误内容
 * @return 无
 */
extern void CreateError(uint16_t error_code, uint8_t error_level,
		uint8_t clear_type, int32_t error_info = 0, uint8_t channel_index = CHANNEL_ENGINE_INDEX, uint8_t axis_index = NO_AXIS);

//检测网络状态，网络是否可用
extern bool CheckNetState(const char *net_name);

//获取本地指定连接的IP地址
extern bool GetLocalIP(const char *eth_name, char *local_ip_addr);

//校验CRC
extern bool CheckCrc(const int16_t *data, const int len, const int16_t crc);

//启动MI
extern bool StartMi();

//释放Linux的缓存
extern int DropCaches(int level);

//文件拷贝函数
extern int CopyFile(const char *src, const char *des);

//获取磁盘剩余空间
extern uint64_t GetFreeDisk();

//将字符串中的字母都转换为大写
extern void StrUpper(char *str);

#endif /* INC_GLOBAL_INCLUDE_H_ */
