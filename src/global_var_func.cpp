/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file global_alg.h
 *@author gonghao
 *@date 2020/03/04
 *@brief ��Դ�ļ�Ϊȫ�ֱ����ͺ����Ķ���
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

SystemState g_sys_state;    //ϵͳ״̬
SystemInfo g_sys_info;      //ϵͳ��Ϣ�������汾��CPUռ���ʵ�

double pmc_period1;  // 1��pmcʱ��
double pmc_period2;  // 2��pmcʱ��

HMICommunication *g_ptr_hmi_comm = nullptr;   //HMIͨѶ�ӿ�
MICommunication *g_ptr_mi_comm = nullptr; 		//MIͨѶ�ӿ�
MCCommunication *g_ptr_mc_comm = nullptr;		//MCͨѶ�ӿ�
MCArmCommunication *g_ptr_mc_arm_comm = nullptr;   //MC-ARMͨѶ�ӿ�																	   
ChannelEngine *g_ptr_chn_engine = nullptr;    //ͨ������
ParmManager *g_ptr_parm_manager = nullptr;    //����������
TraceInfo *g_ptr_trace = nullptr;             //��־���Խӿ�
AlarmProcessor *g_ptr_alarm_processor = nullptr;        //�澯�������ӿ�
TraceLogProcess *g_ptr_tracelog_processor = nullptr;    //������¼
ADCommunication *g_ptr_ad_comm = nullptr;     //�����豸ͨѶ�ӿ�



/**
 * @brief ����������Ϣ��
 * @param error_code : ������
 * @param error_level �����󼶱�
 * @param clear_type �� �������
 * @param error_info ��������Ϣ
 * @param channel_index �� ͨ����
 * @param axis_index �����
 */
void CreateError(uint16_t error_code, uint8_t error_level,
    uint8_t clear_type, int32_t error_info, uint8_t channel_index, uint8_t axis_index) { 

    if (error_code == 0)
        return g_ptr_alarm_processor->ClearTips();
    //if(axis_index != 0xffff) {
    //    axis_index++;	//0,1,2,3,4 --> 1,2,3,4,5
    //}
    if(g_ptr_alarm_processor->ContainErrorInfo(error_code, error_level, clear_type, error_info, channel_index, axis_index)){
        return;  //�ظ��澯
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


	//����澯�кţ�PLC����ʱ����error_info�����PLC��������Ϣ������PLC����ʱ������
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
 * @brief �������״̬�������Ƿ����
 * @param net_name : ������������
 * @return true--����   false--������
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
	close(skfd);  //�ر�socket

	return true;
}

/**
 * @brief ��ȡ����ָ�����ӵ�IP��ַ
 * @param eth_name : ������������
 * @param local_ip_addr[out] ����ȡ��IP��ַ�ַ���
 * @return true--�ɹ�   false--ʧ��
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
 * @brief CRCУ�麯��
 * @param data ����У������ָ��
 * @param len �����ݸ���
 * @param crc ��ԭCRCֵ
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
 * @brief ����MI����
 */

bool StartMi(){
	printf("start MI, ready, pagesize = %d\n", PAGE_SIZE);

	fflush(stdout);

	//���豸�ļ�
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1) {
		g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "MICommunication::StartMI() failed to open file! errno = %d", errno);
		return false;
	}
	//ӳ���ڴ��ַ���õ�����ַ
	int8_t *map = (int8_t *) mmap(nullptr, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, MI_START_REG & (~PAGE_MASK));
	if (map == MAP_FAILED) {
		g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "MICommunication::StartMI() failed to strat MI! errno = %d", errno);
		return false;
	}

	volatile int32_t *virt_addr = (volatile int32_t*) (map + ( MI_START_REG & PAGE_MASK));

//	*virt_addr = 0x8000000;    //256M�ڴ�
	*virt_addr = 0x18000000;	//512M�ڴ�

	//ȡ���ڴ�ӳ��
	if(-1 == munmap(map, PAGE_SIZE)){
		g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "ȡ��ӳ���ļ�[/dev/mem]ʧ�ܣ�map = %p, errno=%d", map, errno);
		//�ر��豸�ļ�
		if(fd != -1) {
			close(fd);
		}
		return false;
	}

	//�ر��豸�ļ�
	if(fd != -1) {
		close(fd);
	}
	printf("start Mi over!\n");

	usleep(1000000);  //��ʱ1s
	printf("sleep 1s over\n");

	return true;
}

/**
 * @brief �ͷ�Linux cache
 * @param level : 1--�����ҳ�滺��  2--���Ŀ¼���inode    3--���ҳ�滺�棬Ŀ¼���inode
 * @return 0--�ɹ�    -1 -- ʧ��
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
 * @brief �ļ���������
 * @param des ��Ŀ���ļ�·��
 * @param src ��Դ�ļ�·��
 * @return  0--�����ɹ�   ����--ʧ��
 */
int CopyFile(const char *src, const char *des){
	int res = 0;

	char buffer[1024] = {0};

	FILE *file_src = fopen(src, "r+");
	if (nullptr == file_src)
	{
		g_ptr_trace->PrintLog(LOG_ALARM, "�����ļ�ʧ�ܣ�Դ�ļ���ʧ��[%s]��errno = %d", src, errno);
		return 1;
	}

	FILE *file_des = fopen(des, "w+");
	if (nullptr == file_des)
	{
		g_ptr_trace->PrintLog(LOG_ALARM, "�����ļ�ʧ�ܣ�Ŀ���ļ�����ʧ��[%s]��errno = %d", des, errno);
		fclose(file_src);
		return 2;
	}

	int size = 0;
	fseek(file_src, 0L, SEEK_END);
	size = ftell(file_src);   //��ȡ�ļ�����

	fseek(file_src, 0L, SEEK_SET);   //�ص��ļ�ͷ

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
			g_ptr_trace->PrintLog(LOG_ALARM, "�����ļ�ʧ�ܣ�д��Ŀ���ļ�ʧ��[%s]��errno = %d", des, errno);
			fclose(file_src);
			fclose(file_des);
			return 3;
		}
		memset(buffer, 0, 1024);
	}

	fclose(file_src);
	fclose(file_des);

	//�޸�Ŀ���ļ�����
	if(-1 == chmod(des, 0x7777)){
		g_ptr_trace->PrintLog(LOG_ALARM, "�޸��ļ�[%s]����ʧ�ܣ�errno = %d", des, errno);
		return 4;
	}
	return res;
}


//��ȡ����ʣ��ռ�
uint64_t GetFreeDisk(){
	uint64_t free = 0;

	struct statfs diskInfo;
	statfs("/cnc/nc_files/", &diskInfo);
	uint64_t blocksize = diskInfo.f_bsize;     //ÿ��block�������ֽ���

    free = diskInfo.f_bfree  * blocksize;

	return free;
}

//���ַ����е���ĸ��ת��Ϊ��д
void StrUpper(char *str){
	if(str == nullptr)
		return;

	char *c = str;
	while(*c != '\0'){
		if(islower(*c))
			*c = toupper(*c);   //Сд���д
		c++;
	}
}
