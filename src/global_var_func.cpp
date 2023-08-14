/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file global_alg.h
 *@author gonghao
 *@date 2020/03/04
 *@brief 本源文件为全局变量和函数的定义
 *@version
 */

#include "global_include.h"
#include "channel_engine.h"
#include "parm_manager.h"
#include "trace.h"
#include "alarm_processor.h"
#include "mi_communication.h"
#include "mc_communication.h"
#include "ad_communication.h"
#include "mc_communication_arm.h"								 
#include <linux/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/statfs.h>

SystemState g_sys_state;    //系统状态
SystemInfo g_sys_info;      //系统信息，包括版本和CPU占用率等

double pmc_period1;  // 1级pmc时间
double pmc_period2;  // 2级pmc时间

HMICommunication *g_ptr_hmi_comm = nullptr;   //HMI通讯接口
MICommunication *g_ptr_mi_comm = nullptr; 		//MI通讯接口
MCCommunication *g_ptr_mc_comm = nullptr;		//MC通讯接口
MCArmCommunication *g_ptr_mc_arm_comm = nullptr;   //MC-ARM通讯接口																	   
ChannelEngine *g_ptr_chn_engine = nullptr;    //通道引擎
ParmManager *g_ptr_parm_manager = nullptr;    //参数管理器
TraceInfo *g_ptr_trace = nullptr;             //日志调试接口
AlarmProcessor *g_ptr_alarm_processor = nullptr;        //告警处理器接口
TraceLogProcess *g_ptr_tracelog_processor = nullptr;    //操作记录
ADCommunication *g_ptr_ad_comm = nullptr;     //辅助设备通讯接口



/**
 * @brief 创建错误信息包
 * @param error_code : 错误码
 * @param error_level ：错误级别
 * @param clear_type ： 清除方法
 * @param error_info ：附带信息
 * @param channel_index ： 通道号
 * @param axis_index ：轴号
 */
void CreateError(uint16_t error_code, uint8_t error_level,
    uint8_t clear_type, int32_t error_info, uint8_t channel_index, uint8_t axis_index) { 

    if (error_code == 0)
        return g_ptr_alarm_processor->ClearTips();
    //if(axis_index != 0xffff) {
    //    axis_index++;	//0,1,2,3,4 --> 1,2,3,4,5
    //}
    if(g_ptr_alarm_processor->ContainErrorInfo(error_code, error_level, clear_type, error_info, channel_index, axis_index)){
        return;  //重复告警
    }

    if (g_ptr_chn_engine->GetPoweroffFlag())
        return;

    if(error_level < WARNING_LEVEL){
        g_ptr_chn_engine->SetChnMachineState(channel_index, MS_WARNING);
    }

    ErrorInfo err_info;
    memset(&err_info, 0x00, sizeof(ErrorInfo));
    time_t cur_time;
    err_info.channel_index = channel_index;
    err_info.axis_index = axis_index;
    err_info.error_code = (uint16_t) error_code;
    err_info.error_level = (uint8_t) error_level;
    err_info.clear_type = (uint8_t) clear_type;
    err_info.error_info = error_info;
    struct tm* time_info;
    cur_time = time(NULL);
    if (cur_time != -1) {
        time_info = localtime(&cur_time);
        if (time_info != NULL) {
            err_info.time = (uint64_t) time_info->tm_sec & (uint64_t) 0xff;
            err_info.time |= ((uint64_t) time_info->tm_min & (uint64_t) 0xff)
                << 8;
            err_info.time |= ((uint64_t) time_info->tm_hour & (uint64_t) 0xff)
                << 16;
            err_info.time |= ((uint64_t) time_info->tm_mday & (uint64_t) 0xff)
                << 24;
            err_info.time |= ((uint64_t) (time_info->tm_mon + 1)
                & (uint64_t) 0xff) << 32;
            err_info.time |= ((uint64_t) (time_info->tm_year + 1900)
                & (uint64_t) 0xffff) << 40;
        } else {
            err_info.time = 0;
        }
    } else {
        err_info.time = 0;
    }

    g_ptr_alarm_processor->SetErrorInfo(&err_info);


	//保存告警行号，PLC报警时由于error_info存的是PLC报警号信息，所以PLC报警时不保存
//	if (err_info.error_level <= ERROR_LEVEL && err_info.error_type != PLC_ALARM_ERR) {
//		if(channel_index < g_ncc_sm_config.general_config.channel_number)
//			g_channel_state[channel_index].alarm_line_no = err_info.error_info;
//		else if(channel_index == CHANNEL_ENGINE_INDEX){
//			for(int i = 0; i < g_ncc_sm_config.general_config.channel_number; i++)
//				g_channel_state[i].alarm_line_no = err_info.error_info;
//		}
//	}
}

/**
 * @brief 检测网络状态，网络是否可用
 * @param net_name : 网络连接名称
 * @return true--可用   false--不可用
 */
bool CheckNetState(const char *net_name){
	int skfd = 0;
	struct ifreq ifr;
	skfd = socket(AF_INET, SOCK_DGRAM, 0);

	if(skfd < 0){
		printf("%s:%d open socket error!errno = %d\n", __FILE__, __LINE__, errno);
		return false;
	}

	strcpy(ifr.ifr_name, net_name);

	if(ioctl(skfd, SIOCGIFFLAGS, &ifr) < 0){
		printf("%s:%d IOCTL error!\n", __FILE__, __LINE__);
		printf("Maybe ethernet interface %s is not valid!\n", ifr.ifr_name);
		close(skfd);
		return false;
	}

	if(ifr.ifr_flags & IFF_RUNNING){
		if(!g_sys_state.eth0_running)
			printf("%s is running :)\n", ifr.ifr_name);
	}else{
		if(g_sys_state.eth0_running)
			printf("%s is not running :(\n", ifr.ifr_name);
		close(skfd);
		return false;
	}
	close(skfd);  //关闭socket

	return true;
}

/**
 * @brief 获取本地指定连接的IP地址
 * @param eth_name : 本地连接名称
 * @param local_ip_addr[out] ：获取的IP地址字符串
 * @return true--成功   false--失败
 */
bool GetLocalIP(const char *eth_name, char *local_ip_addr){
	bool ret = false;
	register int fd;
	struct ifreq ifr;
	if(local_ip_addr == nullptr || eth_name == nullptr)
		return ret;

	if((fd = socket(AF_INET, SOCK_DGRAM, 0)) > 0){
		strcpy(ifr.ifr_name, eth_name);
		if(!(ioctl(fd, SIOCGIFADDR, &ifr))){
			ret = true;
			strcpy(local_ip_addr, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
		}
	}
	if(fd > 0)
		close(fd);
	return ret;
}


/**
 * @brief CRC校验函数
 * @param data ：待校验数据指针
 * @param len ：数据个数
 * @param crc ：原CRC值
 * @return
 */
bool CheckCrc(const int16_t *data, const int len, const int16_t crc){
	if(data == nullptr)
		return false;

	int16_t cc = 0xffff;
	for(int i = 0; i < len; i++){
		cc ^= data[i];
	}

	if(cc != crc){
		printf("check crc failed, 0x%hx, 0x%hx\n", cc, crc);
		return false;
	}

	return true;
}

/**
 * @brief 启动MI程序
 */

bool StartMi(){
	printf("start MI, ready, pagesize = %d\n", PAGE_SIZE);

	fflush(stdout);

	//打开设备文件
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1) {
		g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "MICommunication::StartMI() failed to open file! errno = %d", errno);
		return false;
	}
	//映射内存地址，得到基地址
	int8_t *map = (int8_t *) mmap(nullptr, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, MI_START_REG & (~PAGE_MASK));
	if (map == MAP_FAILED) {
		g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "MICommunication::StartMI() failed to strat MI! errno = %d", errno);
		return false;
	}

	volatile int32_t *virt_addr = (volatile int32_t*) (map + ( MI_START_REG & PAGE_MASK));

//	*virt_addr = 0x8000000;    //256M内存
	*virt_addr = 0x18000000;	//512M内存

	//取消内存映射
	if(-1 == munmap(map, PAGE_SIZE)){
		g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "取消映射文件[/dev/mem]失败！map = %p, errno=%d", map, errno);
		//关闭设备文件
		if(fd != -1) {
			close(fd);
		}
		return false;
	}

	//关闭设备文件
	if(fd != -1) {
		close(fd);
	}
	printf("start Mi over!\n");

	usleep(1000000);  //延时1s
	printf("sleep 1s over\n");

	return true;
}

/**
 * @brief 释放Linux cache
 * @param level : 1--仅清除页面缓存  2--清除目录项和inode    3--清除页面缓存，目录项和inode
 * @return 0--成功    -1 -- 失败
 */
int DropCaches(int level){
	int ret = 0;

#ifdef WIN32

#else
	int fd = 0;
	fd = open("/proc/sys/vm/drop_caches", O_RDWR);
	if(fd < 0){
		return -1;
	}

	char drop_data[32] = {0};
	int drop_size = snprintf(drop_data, sizeof(drop_data), "%d", level);

	ret = write(fd, drop_data, drop_size);
	close(fd);

#endif
	return ret;
}

/**
 * @brief 文件拷贝函数
 * @param des ：目的文件路径
 * @param src ：源文件路径
 * @return  0--拷贝成功   其它--失败
 */
int CopyFile(const char *src, const char *des){
	int res = 0;

	char buffer[1024] = {0};

	FILE *file_src = fopen(src, "r+");
	if (nullptr == file_src)
	{
		g_ptr_trace->PrintLog(LOG_ALARM, "拷贝文件失败，源文件打开失败[%s]！errno = %d", src, errno);
		return 1;
	}

	FILE *file_des = fopen(des, "w+");
	if (nullptr == file_des)
	{
		g_ptr_trace->PrintLog(LOG_ALARM, "拷贝文件失败，目标文件创建失败[%s]！errno = %d", des, errno);
		fclose(file_src);
		return 2;
	}

	int size = 0;
	fseek(file_src, 0L, SEEK_END);
	size = ftell(file_src);   //获取文件长度

	fseek(file_src, 0L, SEEK_SET);   //回到文件头

	int count_wr = 0;
	while(!feof(file_src))
	{
		fread(buffer, 1024, 1, file_src);
		if(size <= 1024){
			count_wr = fwrite(buffer, size, 1, file_des);
		}else{
			count_wr = fwrite(buffer, 1024, 1, file_des);
			size -= 1024;
		}

		if (count_wr == 0)
		{
			g_ptr_trace->PrintLog(LOG_ALARM, "拷贝文件失败，写入目标文件失败[%s]！errno = %d", des, errno);
			fclose(file_src);
			fclose(file_des);
			return 3;
		}
		memset(buffer, 0, 1024);
	}

	fclose(file_src);
	fclose(file_des);

	//修改目标文件属性
	if(-1 == chmod(des, 0x7777)){
		g_ptr_trace->PrintLog(LOG_ALARM, "修改文件[%s]属性失败！errno = %d", des, errno);
		return 4;
	}
	return res;
}


//获取磁盘剩余空间
uint64_t GetFreeDisk(){
	uint64_t free = 0;

	struct statfs diskInfo;
	statfs("/cnc/nc_files/", &diskInfo);
	uint64_t blocksize = diskInfo.f_bsize;     //每个block包含的字节数

    free = diskInfo.f_bfree  * blocksize;

	return free;
}

//将字符串中的字母都转换为大写
void StrUpper(char *str){
	if(str == nullptr)
		return;

	char *c = str;
	while(*c != '\0'){
		if(islower(*c))
			*c = toupper(*c);   //小写变大写
		c++;
	}
}
