/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file trace.h
 *@author gonghao
 *@date 2020/05/14
 *@brief ��ͷ�ļ��������Ը�����Ϣ������
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

//TraceLevel���ݽṹ�������õ�����Ϣ��ӡ����
enum TraceLevel {
	TRACE_NONE = 0, //����¼������Ϣ
	TRACE_ERROR, 	//���ش���Error
	TRACE_WARNING, 	//����Warning�������ڴ����ʧ��
	TRACE_INFO, 	//�������ú��˳���Entry-Exit
	TRACE_DETAIL	//��ϸ���ù��̣�ÿһ������
};

//LogType��־����
enum LogType {
	LOG_SYSTEM_INFO = 0,	//ϵͳ��Ϣ
	LOG_MACHINING_INFO,		//�ӹ���Ϣ
	LOG_ALARM,				//�澯
	LOG_MCP_OPERATION,		//MCP����
	LOG_HMI_MENU,			//HMI�˵�����
	LOG_CONFIG_MODIFY		//�����޸�

};

/*
 * @brief �ļ�����
 */
enum LogFileType {
	ALARM_FILE = 0,
	LOG_FILE,
	TRACE_FILE
};


//TraceModule���ݽṹΪ������Ϣ����
enum TraceModule {
	//NCC-SC
	MAIN_ENTRANCE_SC = 0,       //SCģ��main���
	HMI_COMMUNICATION,	    	//HMI����ͨ��ģ��
	COMMAND_TRANSFER,           //����ת��
	CHANNEL_ENGINE_SC,          //ͨ������
	CHANNEL_CONTROL_SC,     	//��λ��ͨ������
	ERROR_PROCESS_SC,	        //��λ��������
	STATE_MONITOR_SC,       	//��λ��״̬���
	ALARM_DIAGNOSIS_SC,         //��λ��ϵͳ�澯�����
	TOOL_COMPENSATION,          //����
	COORD_TRANSFER,             //����任
	PROTECTION_ZONE_LIMIT,		//��ȫ��������ģ��
	MATRIX_CALCULATION,         //��������
	MC_COMMUNICATION_SC,	    //SC-MCͨѶģ��
	MI_COMMUNICATION_SC,        //SC-MIͨѶģ��
	PARAM_MANAGER,				//����������
	COMPILER_LEXER,				//�ʷ�������
	COMPILER_PARSER,			//�﷨������
	COMPILER_CHN,				//����ģ��
	PMC_REGISTER,				//PMC�Ĵ���ģ��
	AD_COMMUNICATION,			//�����豸ͨѶģ��
	MACRO_VARIABLE,				//�����
	USER_MACRO_VARIABLE			//�û������

};

enum PrintType{
      TypeNone =                  0,//����ӡ
      TypeChnStatus =             1,//ͨ��״̬
      TypeMcStatus =              2,//MC״̬
      TypeRealtimeData =          3,//ʵʱ����
      TypeSpindle =               4,//����״̬
      TypeSysConfig =             5,//ϵͳ����
      TypeChnConfig =             6,//ͨ������
      TypeAxisConfig =            7,//�������
      TypeCoordConfig =           8,//��������ϵƫ��
      TypeExCoordConfig =         9,//��չ��������ϵƫ��
      TypeGrbCoordConfig =        10,//ȫ�ֹ�������ϵƫ��
      TypeTooOffsetlConfig =      11,//����ƫ������
      TypeToolPotConfig =         12,//������Ϣ
      TypeFiveAxisConfig =        13,//�������
      TypeMode =                  14,//ģ̬
      TypeWarning =               15,//����
      TypeFRegState =             16,//F�Ĵ���
      TypeGRegState =             17,//G�Ĵ���
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
 * @brief ��־��������Ϣ�����
 */
class TraceInfo {
public:
	/*
	 * @brief ��ô���ʵ����Ψһȫ�ַ��ʵ�
	 */
	static TraceInfo* GetInstance();

	/*
	 * @brief ��־����������
	 */
	~TraceInfo();

	/*
	 * @brief �����־��Ϣ
	 * @param log_type ��־����
	 * @param log_message ��־����
	 * @return ��
	 */
	void PrintLog(LogType log_type, const char* log_message, ...);

	/*
	 * @brief ���������Ϣ
	 * @param trace_level ������Ϣ�ȼ�
	 * @param trace_module ������Ϣģ��ID
	 * @param trace_message ������Ϣ����
	 * @return ��
	 */
	void PrintTrace(TraceLevel trace_level, TraceModule trace_module,
            const char* trace_message, ...);

    /*
     * @brief ���������Ϣ,��������з���
     * @param topic ��ǰ������Ϣ����
     * @param trace_message ������Ϣ����
     * @return ��
     */
    void PrintTopic(int topic, std::string content);

	/*
	 * @brief ����澯��Ϣ
	 * @param error_info �澯����ָ��
	 * @return ��
	 */
	void PrintAlarm(const ErrorInfo* error_info);



	//m_log_type_setting��ȡ����
	uint32_t log_type_setting() const {
		return m_log_type_setting;
	}
	void set_log_type_setting(uint32_t log_type_setting) {
		m_log_type_setting = log_type_setting;
	}

	//m_trace_level��ȡ����
	unsigned char trace_level() const {
		return m_trace_level;
	}
	void set_trace_level(unsigned char trace_level) {
		m_trace_level = trace_level;
	}

	//m_trace_output_setting��ȡ����,1:�ı���0������
	unsigned char trace_output_setting() const {
		return m_trace_output_setting;
	}
	void set_trace_output_setting(unsigned char trace_output_setting) {
		m_trace_output_setting = trace_output_setting;
	}


	//m_serial_port_setting��ȡ����
	void serial_port_setting(char* serial_port_setting) {
		strcpy(serial_port_setting, m_serial_port_setting);
	}
	void set_serial_port_setting(char* serial_port_setting) {
		strcpy(m_serial_port_setting, serial_port_setting);
	}

	/*
	 * @brief ������Ϣ���
	 * @param trace_message ������Ϣ����
	 */
	void TraceOutput(const char* trace_message);

	/*
	 * @brief ��ȡ��ǰʱ��
	 */
	char* GetCurrentTime(void);

	/*
	 * @brief ����������־/����/�澯�ļ���
	 * @param type �ļ�����
	 * @param name �ļ���
	 */
	void ResetFileName(LogFileType type);


	/*
	 * @brief �澯��ʷ�ļ��Ƿ����
	 */
	static bool IsAlarmFileExist(uint8_t type);

	/*
	 * @brief ��ȡ�澯��ʷ�ļ�·��
	 * @param type[in] : �澯��ʷ�ļ�����
	 * @param path[out] : �澯��ʷ�ļ�·��
	 */
	static void GetAlarmFilePath(uint8_t type, char *path);

	/**
	 * @brief ��ȡ�澯�ļ��ܴ�С������alarmfile.txt��alarmfile_bak.txt
	 * @return �ܸ澯���ݴ�С
	 */
	static uint64_t GetAlarmTotalSize();

	/*
	 * @brief ͬ���ļ�������
	 */
//	void SyncFile();

	//void FileAppendStr(const char*file_name, const char* str);
	//void MyPrintLog(uint32_t type, const char* msg, ...);//��trace������ļ�
private:
	/*
	 * @brief ��־�๹�캯��
	 */
	TraceInfo();
	static TraceInfo* m_instance;              //������
	uint32_t m_log_type_setting;			    //��־��������
	unsigned char m_trace_level;			    //������Ϣ�������
	unsigned char m_trace_output_setting;	    //������Ϣ�����ʽ����
	char m_serial_port_setting[kMaxPathLen];   //��������
	pthread_mutex_t m_mutex;    //��д������

	int32_t m_log_handler;	//������־�ļ����
	int32_t m_trace_handler;	//������Ϣ�ļ����
	int32_t m_alarm_handler;	//�澯��Ϣ�ļ����

    std::string m_topic_dest = "192.168.100.155";
    mosqpp::TraceMesSend m_topic_mosq;
};




#endif /* INC_TRACE_H_ */
