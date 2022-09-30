/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file trace.cpp
 *@author gonghao
 *@date 2020/05/14
 *@brief 本头文件包含调试跟踪信息类声明
 *@version
 */
#include "trace.h"
#include "global_include.h"


//文件保存目录
#define LOG_PATH "/cnc/trace/log/"
#define TRACE_PATH "/cnc/trace/trace/"
#define ALARM_PATH "/cnc/trace/alarm/"

//文件名
#define LOG_FILE_NAME "logfile.txt"
#define LOG_BAK_FILE_NAME "logfile_bak.txt"
#define TRACE_FILE_NAME "tracefile.txt"
#define TRACE_BAK_FILE_NAME "tracefile_bak.txt"
#define ALARM_FILE_NAME "alarmfile.txt"
#define ALARM_BAK_FILE_NAME "alarmfile_bak.txt"

//#define ALARM_ITEM_START 0x


const int kMaxTimeLen = 21;      //时间字符串最大长度，包括年月日时分秒
const int kMaxMsgLen = 256;      //log消息字符串最大长度

const int kMaxFileSize = 1024*1024*5;	    //日志/调试信息记录文件最大长度5M

TraceInfo* TraceInfo::m_instance = NULL;

TraceInfo::TraceInfo() :
	m_log_type_setting(0xFFFFFFFF), //日志类型配置默认打印
	m_trace_level(TRACE_WARNING),	//调试信息输出级别默认0
	m_trace_output_setting(1) {     //调试信息输出默认采用文本记录
	//初始化文件句柄
	m_log_handler = -1;
	m_trace_handler = -1;
	m_alarm_handler = -1;

	strcpy(m_serial_port_setting, "/dev/ttyS0");

	pthread_mutex_init(&m_mutex, NULL);

	//创建保存目录
	char log_file_name[kMaxPathLen] = {0};
	const char *paths[] = {LOG_PATH, TRACE_PATH, ALARM_PATH};
	int size_len = 0;
	for(int index = 0; index < 3; index++){
		strcpy(log_file_name, paths[index]);                  //获取路径
		if (log_file_name[strlen(log_file_name) - 1] != '/') {
			strcat(log_file_name, "/");                       //在路径最后添加字符‘/’
		}
		size_len = strlen(log_file_name);
		for (int i = 1; i < size_len; i++) {       //注意：i=1 而不是 i=0  创建多级目录
			if (log_file_name[i] == '/') {
				log_file_name[i] = '\0';
				if (access(log_file_name, F_OK) != 0) {
					if (mkdir(log_file_name, 0755) == -1) {
						perror("Create log directory failed!");
						break;
					}
				}
				log_file_name[i] = '/';
			}
		}
	}

	//打开各文件
	//LOG文件
	memset(log_file_name, 0x00, kMaxPathLen);
	strcpy(log_file_name, LOG_PATH);
	strcat(log_file_name, LOG_FILE_NAME);
	m_log_handler = open(log_file_name, O_WRONLY|O_CREAT|O_APPEND, 0644);
	if(m_log_handler == -1){
		perror("Failed to open log file!");
		return;
	}

	//TRACE文件
	memset(log_file_name, 0x00, kMaxPathLen);
	strcpy(log_file_name, TRACE_PATH);
	strcat(log_file_name, TRACE_FILE_NAME);
	m_trace_handler = open(log_file_name, O_WRONLY|O_CREAT|O_APPEND, 0644);
	if(m_trace_handler == -1){
		perror("Failed to open trace file!");
		return;
	}

	//ALARM文件
	memset(log_file_name, 0x00, kMaxPathLen);
	strcpy(log_file_name, ALARM_PATH);
	strcat(log_file_name, ALARM_FILE_NAME);
	m_alarm_handler = open(log_file_name, O_WRONLY|O_CREAT|O_APPEND, 0644);
	if(m_alarm_handler == -1){
		perror("Failed to open alarm file!");
		return;
	}

    // topic 形式的数据监控
    //mosquitto_lib_init();
    //m_topic_mosq.connect(m_topic_dest.c_str());
	//m_topic_mosq.loop_start();
}

//由于m_instance为静态变量，所以系统会自动析构
TraceInfo::~TraceInfo() {

	//关闭文件
	if(m_log_handler > 0){
		fsync(m_log_handler);
		close(m_log_handler);
		m_log_handler = -1;
	}
	if(m_trace_handler > 0){
		fsync(m_trace_handler);
		close(m_trace_handler);
		m_trace_handler = -1;
	}
	if(m_alarm_handler > 0){
		fsync(m_alarm_handler);
		close(m_alarm_handler);
		m_alarm_handler = -1;
	}

    // topic 形式的数据监控
    //mosquitto_lib_cleanup();
}

//GetInstance是获得此类实例的唯一全局访问点
TraceInfo* TraceInfo::GetInstance() {
	if (m_instance == NULL) {
		m_instance = new TraceInfo();
	}
	return m_instance;
}

//记录系统日志
void TraceInfo::PrintLog(LogType log_type, const char* log_message, ...) {
	pthread_mutex_lock(&m_mutex);
	char current_time[kMaxTimeLen] = {0};
	char log_message_buf[kMaxMsgLen] = {0};  //保存log_message

	va_list arg_ptr;                        //可变长度参数
	struct stat log_size_buf;              //获取文件大小
	int log_str_len = 0;					//消息字符串长度
	if ((0x01 << log_type) & m_log_type_setting) {    //是否需要记录该日志信息   //直接调用成员变量
		time_t timep;
		struct tm *p;
		time(&timep);
		p = gmtime(&timep);
		sprintf(current_time, "%04d/%02d/%02d %02d:%02d:%02d ",
				1900 + p->tm_year, 1 + p->tm_mon, p->tm_mday, p->tm_hour,
				p->tm_min, p->tm_sec);
		//处理可变长度参数
		va_start(arg_ptr, log_message);
		//vsprintf(log_message_buf, log_message, arg_ptr);              //内存可能会溢出，不够安全
		vsnprintf(log_message_buf, sizeof(log_message_buf), log_message,
				arg_ptr); //限制最大长度，防止内存溢出
		va_end(arg_ptr);
		log_str_len = strlen(log_message_buf);
		if(log_message_buf[log_str_len-1] != '\n')
			log_message_buf[log_str_len] = '\n';	//换行

		printf(current_time);
		printf(log_message_buf);  //for test

		//写入日志文件
		if(m_log_handler > 0){
			if(fstat(m_log_handler, &log_size_buf) != 0){
				perror("Failed to get log file size!");
				goto END;
			}

			if (log_size_buf.st_size >= kMaxFileSize) { //大于等于文件最大值 kMaxFileSize
				ResetFileName(LOG_FILE);
			}

			write(m_log_handler, current_time, kMaxTimeLen-1);
			write(m_log_handler, log_message_buf, strlen(log_message_buf));
			fsync(m_log_handler);

		}
	}

	END:
	pthread_mutex_unlock(&m_mutex);
}

//记录调试信息，根据调试信息设置以及输入参数，决定记录哪些调试信息
void TraceInfo::PrintTrace(TraceLevel trace_level, TraceModule trace_module,
		const char* trace_message, ...) {
	char trace_message_buf[kMaxMsgLen] = { 0 };       //用于保存trace_message
	va_list arg_ptr;                                    //可变长度参数
	int str_len = 0;		//消息字符串长度
	if (trace_level <= m_trace_level) {

		//处理可变长度参数
		va_start(arg_ptr, trace_message);

		vsnprintf(trace_message_buf, sizeof(trace_message_buf),
				trace_message, arg_ptr);            //限制最大长度，防止内存溢出
		va_end(arg_ptr);
		str_len = strlen(trace_message_buf);
		if(trace_message_buf[str_len-1] != '\n')
			trace_message_buf[str_len] = '\n';	//换行

		//测试用
		printf("%s", trace_message_buf);

		TraceOutput(trace_message_buf);
    }
}

/*void TraceInfo::PrintTopic(const std::string &topic, const char *trace_message, ...)
{
    if (m_topic_mosq.isConnected())
    {
        //处理可变长度参数
        char trace_message_buf[kMaxMsgLen] = { 0 };       //用于保存trace_message
        va_list arg_ptr;
        va_start(arg_ptr, trace_message);
        vsnprintf(trace_message_buf, sizeof(trace_message_buf),
                trace_message, arg_ptr);
        va_end(arg_ptr);

        //主题发布
        m_topic_mosq.publish(nullptr, topic.c_str(), strlen(trace_message_buf), trace_message_buf);
    }
    return;
}*/

//调试信息输出函数
void TraceInfo::TraceOutput(const char* trace_message) {
	pthread_mutex_lock(&m_mutex);
//	int32_t trace_file_des;                                          //调试信息文本描述符
//	int32_t trace_serial_des;                                          //串口描述符
//	char trace_file_name[kMaxPathLen] = { 0 };                  //保存调试文件路径

	struct stat trace_size_buf;                                       //用于获取文件大小
	char current_time[kMaxTimeLen] = {0};
	time_t timep;
	struct tm *p;
	time(&timep);
	p = gmtime(&timep);
	sprintf(current_time, "%04d/%02d/%02d %02d:%02d:%02d ", 1900 + p->tm_year,
			1 + p->tm_mon, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
	if (m_trace_output_setting) {                              //1: 写入文本； 0：串口输出
		//写入调试信息文件
		if(m_trace_handler > 0){
			if(fstat(m_trace_handler, &trace_size_buf) != 0){
				perror("Failed to get trace file size!");
				goto END;
			}

			if (trace_size_buf.st_size >= kMaxFileSize) { //大于等于文件最大值 kMaxFileSize
				ResetFileName(TRACE_FILE);
			}

			write(m_trace_handler, current_time, kMaxTimeLen-1);
			write(m_trace_handler, trace_message, strlen(trace_message));
			fsync(m_trace_handler);

		}


	} else {
//		trace_serial_des = open(m_serial_port_setting, O_WRONLY);
//		if (trace_serial_des == -1) {
//			perror("Please check your serial port setting!");
//		} else {
//			//串口输出
//			write(trace_serial_des, current_time, kMaxTimeLen);
//			//write(trace_serial_des, trace_module, strlen(trace_module));
//			write(trace_serial_des, trace_message, strlen(trace_message));
//			close(trace_serial_des);
//		}

		printf(current_time);
		printf(trace_message);
	}

	END:
	pthread_mutex_unlock(&m_mutex);
}

//记录告警信息
void TraceInfo::PrintAlarm(const ErrorInfo* error_info) {
	pthread_mutex_lock(&m_mutex);

	struct stat alarm_size_buf;              //获取文件大小
	int err_item_size = sizeof(ErrorInfo);   //一条错误记录的大小

	//写入文件
	if(m_alarm_handler > 0){
		if(fstat(m_alarm_handler, &alarm_size_buf) != 0){
			perror("Failed to get alarm file size!");
			goto END;
		}

		if (alarm_size_buf.st_size >= kMaxFileSize) { //大于等于文件最大值 kMaxFileSize
			ResetFileName(ALARM_FILE);
		}
		else{
			int offset = alarm_size_buf.st_size % err_item_size;
			if(offset != 0){//存在不完整记录，则覆盖掉不完整记录
				lseek(m_alarm_handler, -offset, SEEK_END);
			}

		}

		//告警信息起始标识
		write(m_alarm_handler, (char*) error_info, sizeof(ErrorInfo));
		fsync(m_alarm_handler);
	}

	END: pthread_mutex_unlock(&m_mutex);
}


//获取当前时间
char* TraceInfo::GetCurrentTime(void) {
	time_t timeval;
	(void) time(&timeval);
	return ctime(&timeval);
}

//重新设置文件名
void TraceInfo::ResetFileName(LogFileType type) {
	char bak_path_name[kMaxPathLen] = { 0 };
	char cur_path_name[kMaxPathLen] = { 0 };
	const char *bak_files[] = {ALARM_BAK_FILE_NAME, LOG_BAK_FILE_NAME, TRACE_BAK_FILE_NAME};
	const char *cur_files[] = {ALARM_FILE_NAME, LOG_FILE_NAME, TRACE_FILE_NAME};
	const char *paths[] = {ALARM_PATH, LOG_PATH, TRACE_PATH};
	int *fd = nullptr;

	strcpy(bak_path_name, paths[type]);
	if (bak_path_name[strlen(bak_path_name) - 1] != '/') {
		strcat(bak_path_name, "/");                       //在路径最后添加字符‘/’
	}

	strcpy(cur_path_name, bak_path_name);
	strcat(bak_path_name, bak_files[type]);
	strcat(cur_path_name, cur_files[type]);

	if(access(bak_path_name, F_OK) == 0){
		if(0 != remove(bak_path_name)){   //备份文件已存在，则先删除备份文件
			perror("Failed to remove trace file!");
			goto END;
		}

	}

	switch (type) {
	case ALARM_FILE:
		fd = &m_alarm_handler;
		break;
	case LOG_FILE:
		fd = &m_log_handler;
		break;
	case TRACE_FILE:
		fd = &m_trace_handler;
		break;
	default:
		goto END;
		break;
	}
	if(*fd > 0){
		fsync(*fd);
		close(*fd);
	}

	if(0 != rename(cur_path_name, bak_path_name)){  //将当前文件重命名为备份文件
		perror("Failed to rename trace file!");
	}

	*fd = open(cur_path_name, O_WRONLY|O_CREAT|O_APPEND, 0644);
	if(*fd < 0){
		printf("Failed to reopen log file[%s], errno = %d!", cur_path_name, errno);
	}

	END: return;
}

/**
 * @brief 告警历史文件是否存在
 * @param type : 0x10--代表文件"alarmfile.txt",   0x20--代表文件"alarmfile_bak.txt"
 * @return false--文件不存在    true--文件存在
 */
bool TraceInfo::IsAlarmFileExist(uint8_t type){
	char path[kMaxPathLen] = {0};
	strcpy(path, ALARM_PATH);
	if(type == 0x10)
		strcat(path, ALARM_FILE_NAME);
	else if(type == 0x20)
		strcat(path, ALARM_BAK_FILE_NAME);
	else
		return false;

	if(access(path, F_OK) == -1){	//不存在，返回失败
		return false;
	}
	return true;
}

/*
 * @brief 获取告警历史文件路径
 * @param type[in] : 告警历史文件类型
 * @param path[out] : 告警历史文件路径
 */
void TraceInfo::GetAlarmFilePath(uint8_t type, char *path){
	if(path == nullptr)
		return;
	memset(path, 0x00, kMaxPathLen);

	strcpy(path, ALARM_PATH);
	if(type == 0x10)
		strcat(path, ALARM_FILE_NAME);
	else if(type == 0x20)
		strcat(path, ALARM_BAK_FILE_NAME);
	else
		memset(path, 0x00, kMaxPathLen);
}

/**
 * @brief 获取告警文件总大小，包括alarmfile.txt和alarmfile_bak.txt
 * @return 告警数据总大小
 */
uint64_t TraceInfo::GetAlarmTotalSize(){
	uint64_t file_size = 0;
	struct stat statbuf;
	char path[kMaxPathLen] = {0};
	strcpy(path, ALARM_PATH);
	strcat(path, ALARM_FILE_NAME);

	if(access(path, F_OK) == 0){	//存在
		if(stat(path, &statbuf) == 0)
			file_size += statbuf.st_size;  //获取文件总大小
	}

	memset(path, 0, kMaxPathLen);
	strcpy(path, ALARM_PATH);
	strcat(path, ALARM_BAK_FILE_NAME);

	if(access(path, F_OK) == 0){	//存在
		if(stat(path, &statbuf) == 0)
			file_size += statbuf.st_size;  //获取文件总大小
	}

	return file_size;
}

//同步文件到磁盘
//void TraceInfo::SyncFile() {
//	pthread_mutex_lock(&m_mutex);
//	int fd;
//	char file_name[kMaxFileNameLen] = { 0 };
//	//同步日志文件
//	strcpy(file_name, LOG_PATH);
//	strcat(file_name, "logfile.txt");
//	fd = open(file_name, O_WRONLY);
//	if (fd != -1) {
//		fsync(fd);
//		close(fd);
//	}
//
//
//	//同步trace文件
//	strcpy(file_name, TRACE_PATH);
//	strcat(file_name, "tracefile.txt");
//	fd = open(file_name, O_WRONLY);
//	if (fd != -1) {
//		fsync(fd);
//		close(fd);
//	}
//
//	//同步告警文件
//	strcpy(file_name, ALARM_PATH);
//	strcat(file_name, "alarmfile.txt");
//	fd = open(file_name, O_WRONLY);
//	if (fd != -1) {
//		fsync(fd);
//		close(fd);
//	}
//
//	pthread_mutex_unlock(&m_mutex);
//}
//输出NCCCI信息
/*void TraceInfo::SetNCCCIInfo(NCCCICmd* cmd) {
	LogTraceInfo log_trace_info;
	switch (cmd->cmd) {
	case SEND_TRACE_TO_NCC:
		memcpy(&log_trace_info, cmd->data, sizeof(LogTraceInfo));
		PrintTrace((TraceLevel) log_trace_info.trace_level,
				log_trace_info.trace_module, log_trace_info.message);
		break;
	case SEND_LOG_TO_NCC:
		memcpy(&log_trace_info, cmd->data, sizeof(LogTraceInfo));
		PrintLog(log_trace_info.trace_level, log_trace_info.message);
		break;
	default:
		break;
	}
	return;
}*/



