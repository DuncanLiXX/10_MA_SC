/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file trace.h
 *@author gonghao
 *@date 2020/05/14
 *@brief 本头文件包含调试跟踪信息类声明
 *@version
 */

#ifndef INC_TRACE_H_
#define INC_TRACE_H_

//#include "global_include.h"
#include <pthread.h>
#include <string.h>
#include <string>
#include <vector>
#include "mosquittopp.h"
#include "hmi_shared_data.h"

//TraceLevel数据结构用于设置调试信息打印级别
enum TraceLevel {
	TRACE_NONE = 0, //不记录调试信息
	TRACE_ERROR, 	//严重错误Error
	TRACE_WARNING, 	//警告Warning，例如内存分配失败
	TRACE_INFO, 	//函数调用和退出，Entry-Exit
	TRACE_DETAIL	//详细调用过程，每一个步骤
};

//LogType日志类型
enum LogType {
	LOG_SYSTEM_INFO = 0,	//系统信息
	LOG_MACHINING_INFO,		//加工信息
	LOG_ALARM,				//告警
	LOG_MCP_OPERATION,		//MCP操作
	LOG_HMI_MENU,			//HMI菜单操作
	LOG_CONFIG_MODIFY		//参数修改

};

/*
 * @brief 文件类型
 */
enum LogFileType {
	ALARM_FILE = 0,
	LOG_FILE,
	TRACE_FILE
};


//TraceModule数据结构为调试信息类型
enum TraceModule {
	//NCC-SC
	MAIN_ENTRANCE_SC = 0,       //SC模块main入口
	HMI_COMMUNICATION,	    	//HMI数据通信模块
	COMMAND_TRANSFER,           //命令转发
	CHANNEL_ENGINE_SC,          //通道引擎
	CHANNEL_CONTROL_SC,     	//上位机通道控制
	ERROR_PROCESS_SC,	        //上位机错误处理
	STATE_MONITOR_SC,       	//上位机状态监控
	ALARM_DIAGNOSIS_SC,         //上位机系统告警与诊断
	TOOL_COMPENSATION,          //刀补
	COORD_TRANSFER,             //坐标变换
	PROTECTION_ZONE_LIMIT,		//安全区域限制模块
	MATRIX_CALCULATION,         //矩阵运算
	MC_COMMUNICATION_SC,	    //SC-MC通讯模块
	MI_COMMUNICATION_SC,        //SC-MI通讯模块
	PARAM_MANAGER,				//参数管理器
	COMPILER_LEXER,				//词法解析器
	COMPILER_PARSER,			//语法解析器
	COMPILER_CHN,				//编译模块
	PMC_REGISTER,				//PMC寄存器模块
	AD_COMMUNICATION,			//辅助设备通讯模块
	MACRO_VARIABLE,				//宏变量
	USER_MACRO_VARIABLE			//用户宏变量

};

enum PrintType{
      TypeNone =                  0,//不打印
      TypeChnStatus =             1,//通道状态
      TypeMcStatus =              2,//MC状态
      TypeRealtimeData =          3,//实时数据
      TypeSpindle =               4,//主轴状态
      TypeSysConfig =             5,//系统配置
      TypeChnConfig =             6,//通道配置
      TypeAxisConfig =            7,//轴参配置
      TypeCoordConfig =           8,//工件坐标系偏置
      TypeExCoordConfig =         9,//拓展工件坐标系偏置
      TypeGrbCoordConfig =        10,//全局工件坐标系偏置
      TypeTooOffsetlConfig =      11,//刀具偏置配置
      TypeToolPotConfig =         12,//刀具信息
      TypeFiveAxisConfig =        13,//五轴参数
      TypeMode =                  14,//模态
      TypeWarning =               15,//警告
      TypeFRegState =             16,//F寄存器
      TypeGRegState =             17,//G寄存器
      TypeMax,
  };

namespace mosqpp {

struct mosquitto_message{
    int mid;
    char *topic;
    void *payload;
    int payloadlen;
    int qos;
    bool retain;
};

class TraceMesSend : public mosquittopp {
    private:
        void ProcessRecvTopic(const std::string &str);
        std::vector<int> vecTypeSwitch;
    public:
        TraceMesSend(const char *id=NULL, bool clean_session=true);
        virtual ~TraceMesSend();
        bool NeedPublish(int i);
        void on_connect(int) override;
        void on_message(const mosquitto_message * msg) override;
};
}

/**
 * @brief 日志、跟踪消息输出类
 */
class TraceInfo {
public:
	/*
	 * @brief 获得此类实例的唯一全局访问点
	 */
	static TraceInfo* GetInstance();

	/*
	 * @brief 日志类析构函数
	 */
	~TraceInfo();

	/*
	 * @brief 输出日志信息
	 * @param log_type 日志类型
	 * @param log_message 日志内容
	 * @return 无
	 */
	void PrintLog(LogType log_type, const char* log_message, ...);

	/*
	 * @brief 输出调试信息
	 * @param trace_level 调试信息等级
	 * @param trace_module 调试信息模块ID
	 * @param trace_message 调试信息内容
	 * @return 无
	 */
	void PrintTrace(TraceLevel trace_level, TraceModule trace_module,
            const char* trace_message, ...);

    /*
     * @brief 输出调试信息,按主题进行分类
     * @param topic 当前调试信息主题
     * @param trace_message 调试信息内容
     * @return 无
     */
    void PrintTopic(int topic, std::string content);

	/*
	 * @brief 输出告警信息
	 * @param error_info 告警内容指针
	 * @return 无
	 */
	void PrintAlarm(const ErrorInfo* error_info);



	//m_log_type_setting存取函数
	uint32_t log_type_setting() const {
		return m_log_type_setting;
	}
	void set_log_type_setting(uint32_t log_type_setting) {
		m_log_type_setting = log_type_setting;
	}

	//m_trace_level存取函数
	unsigned char trace_level() const {
		return m_trace_level;
	}
	void set_trace_level(unsigned char trace_level) {
		m_trace_level = trace_level;
	}

	//m_trace_output_setting存取函数,1:文本，0：串口
	unsigned char trace_output_setting() const {
		return m_trace_output_setting;
	}
	void set_trace_output_setting(unsigned char trace_output_setting) {
		m_trace_output_setting = trace_output_setting;
	}


	//m_serial_port_setting存取函数
	void serial_port_setting(char* serial_port_setting) {
		strcpy(serial_port_setting, m_serial_port_setting);
	}
	void set_serial_port_setting(char* serial_port_setting) {
		strcpy(m_serial_port_setting, serial_port_setting);
	}

	/*
	 * @brief 调试信息输出
	 * @param trace_message 调试信息内容
	 */
	void TraceOutput(const char* trace_message);

	/*
	 * @brief 获取当前时间
	 */
	char* GetCurrentTime(void);

	/*
	 * @brief 重新设置日志/调试/告警文件名
	 * @param type 文件类型
	 * @param name 文件名
	 */
	void ResetFileName(LogFileType type);


	/*
	 * @brief 告警历史文件是否存在
	 */
	static bool IsAlarmFileExist(uint8_t type);

	/*
	 * @brief 获取告警历史文件路径
	 * @param type[in] : 告警历史文件类型
	 * @param path[out] : 告警历史文件路径
	 */
	static void GetAlarmFilePath(uint8_t type, char *path);

	/**
	 * @brief 获取告警文件总大小，包括alarmfile.txt和alarmfile_bak.txt
	 * @return 总告警数据大小
	 */
	static uint64_t GetAlarmTotalSize();

	/*
	 * @brief 同步文件到磁盘
	 */
//	void SyncFile();

	//void FileAppendStr(const char*file_name, const char* str);
	//void MyPrintLog(uint32_t type, const char* msg, ...);//将trace输出到文件
private:
	/*
	 * @brief 日志类构造函数
	 */
	TraceInfo();
	static TraceInfo* m_instance;              //单例类
	uint32_t m_log_type_setting;			    //日志类型配置
	unsigned char m_trace_level;			    //调试信息输出级别
	unsigned char m_trace_output_setting;	    //调试信息输出形式配置
	char m_serial_port_setting[kMaxPathLen];   //串口配置
	pthread_mutex_t m_mutex;    //读写互斥量

	int32_t m_log_handler;	//操作日志文件句柄
	int32_t m_trace_handler;	//跟踪信息文件句柄
	int32_t m_alarm_handler;	//告警信息文件句柄

    std::string m_topic_dest = "192.168.100.155";
    mosqpp::TraceMesSend m_topic_mosq;
};




#endif /* INC_TRACE_H_ */
