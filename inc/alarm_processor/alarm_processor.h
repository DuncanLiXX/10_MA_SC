/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file alarm_processor.h
 *@author gonghao
 *@date 2020/05/15
 *@brief 本头文件为告警处理类的声明
 *@version
 */

#ifndef INC_ALARM_PROCESSOR_ALARM_PROCESSOR_H_
#define INC_ALARM_PROCESSOR_ALARM_PROCESSOR_H_

#include "global_include.h"

//前置类声明
class HMICommunication;
class ChannelEngine;

/**
 * @brief 告警处理类
 */
class AlarmProcessor {
public:
	/*
	 * @brief 获得此类实例的唯一全局访问点
	 * @return 类实例指针
	 */
	static AlarmProcessor* GetInstance() {
		if (m_instance == nullptr) {
			m_instance = new AlarmProcessor();
		}
		return m_instance;
	}

	/*
	 * @brief 告警处理类析构函数
	 * @param 无
	 * @return 无
	 */
	~AlarmProcessor();

	//设置接口
	void SetInterfaces();

	/*
	 * @brief 写入错误信息
	 * @param error_info 需要写入第错误信息指针
	 * @return 无
	 */
	void SetErrorInfo(ErrorInfo* error_info);

	/*
	 * @brief 读取错误信息
	 * @param error_info 错误信息数据结构指针，用来存放读取第错误信息
	 * @return 错误信息个数
	 */
	int GetErrorInfo(ErrorInfo* error_info);

	/*
	 * @brief 判断是否有出错信息
	 * @param 无
	 * @return 处理结果,true-有未处理的出错信息;false-无未处理的出错信息
	 */
	bool HasErrorInfo(int level = ERROR_LEVEL);

	/**
	 * @brief 判断是否有属于指定通道的告警，通道引擎的告警属于任意通道
	 * @param chn_index ： 指定通道
	 * @return 处理结果,true-有未处理的出错信息;false-无未处理的出错信息
	 */
	bool HasErrorInfo(uint8_t chn_index);

	//判断是否有重复告警
	bool HasErrorInfo(uint16_t error_code, uint8_t error_level,
		uint8_t clear_type, int32_t error_info, uint8_t channel_index, uint8_t axis_index);

	/*
	 * @brief 读取最新的错误信息
	 * @param error_info 错误信息数据结构指针，用来存放读取第错误信息
	 * @return 处理结果,0--处理成功;非0--错误码
	 */
	int GetLatestErrorInfo(ErrorInfo* error_info);

	uint16_t GetLatestErrorCode();   //获取最近的错误代码

	void SendToHmi();   //将当前错误都输出到HMI

	void PrintError();	//打印输出所有错误信息

	void Clear();   //清空告警队列

	void ClearWarning(uint8_t chn);  //清空告警即以下等级的消息

	void RemoveWarning(uint8_t chn, uint16_t error_code);   //清空指定告警

    CircularBuffer<ErrorInfo>* GetWarningList(){return m_error_info_input_list;}

private:  //私有接口函数
	/**
	 * @brief 错误处理类构造函数
	 * 构造函数为私有，防止外界使用new创建该类实例
	 * @param 无
	 * @return 无
	 */
	AlarmProcessor();

	bool SendToHmi(ErrorInfo *err); //将告警信息发送至HMI


	void ProcessAlarm(ErrorInfo *err); //告警的统一处理函数

private:

	static AlarmProcessor* m_instance; 	//单实例指针
	pthread_mutex_t m_mutex;    //读写互斥量
	ErrorInfo m_latest_error;   //最近一次的告警信息
	CircularBuffer<ErrorInfo>* m_error_info_input_list;    //告警信息列表

	HMICommunication *m_p_hmi_comm;   //HMI通讯接口
	ChannelEngine *m_p_chn_engine;		//通道引擎指针
};


#endif /* INC_ALARM_PROCESSOR_ALARM_PROCESSOR_H_ */
