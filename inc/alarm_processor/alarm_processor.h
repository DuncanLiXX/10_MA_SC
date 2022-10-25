/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file alarm_processor.h
 *@author gonghao
 *@date 2020/05/15
 *@brief ��ͷ�ļ�Ϊ�澯�����������
 *@version
 */

#ifndef INC_ALARM_PROCESSOR_ALARM_PROCESSOR_H_
#define INC_ALARM_PROCESSOR_ALARM_PROCESSOR_H_

#include "global_include.h"

//ǰ��������
class HMICommunication;
class ChannelEngine;

/**
 * @brief �澯������
 */
class AlarmProcessor {
public:
	/*
	 * @brief ��ô���ʵ����Ψһȫ�ַ��ʵ�
	 * @return ��ʵ��ָ��
	 */
	static AlarmProcessor* GetInstance() {
		if (m_instance == nullptr) {
			m_instance = new AlarmProcessor();
		}
		return m_instance;
	}

	/*
	 * @brief �澯��������������
	 * @param ��
	 * @return ��
	 */
	~AlarmProcessor();

	//���ýӿ�
	void SetInterfaces();

	/*
	 * @brief д�������Ϣ
	 * @param error_info ��Ҫд��ڴ�����Ϣָ��
	 * @return ��
	 */
	void SetErrorInfo(ErrorInfo* error_info);

	/*
	 * @brief ��ȡ������Ϣ
	 * @param error_info ������Ϣ���ݽṹָ�룬������Ŷ�ȡ�ڴ�����Ϣ
	 * @return ������Ϣ����
	 */
	int GetErrorInfo(ErrorInfo* error_info);

	/*
	 * @brief �ж��Ƿ��г�����Ϣ
	 * @param ��
	 * @return ������,true-��δ����ĳ�����Ϣ;false-��δ����ĳ�����Ϣ
	 */
	bool HasErrorInfo(int level = ERROR_LEVEL);

	/**
	 * @brief �ж��Ƿ�������ָ��ͨ���ĸ澯��ͨ������ĸ澯��������ͨ��
	 * @param chn_index �� ָ��ͨ��
	 * @return ������,true-��δ����ĳ�����Ϣ;false-��δ����ĳ�����Ϣ
	 */
	bool HasErrorInfo(uint8_t chn_index);

	//�ж��Ƿ����ظ��澯
	bool HasErrorInfo(uint16_t error_code, uint8_t error_level,
		uint8_t clear_type, int32_t error_info, uint8_t channel_index, uint8_t axis_index);

	/*
	 * @brief ��ȡ���µĴ�����Ϣ
	 * @param error_info ������Ϣ���ݽṹָ�룬������Ŷ�ȡ�ڴ�����Ϣ
	 * @return ������,0--����ɹ�;��0--������
	 */
	int GetLatestErrorInfo(ErrorInfo* error_info);

	uint16_t GetLatestErrorCode();   //��ȡ����Ĵ������

	void SendToHmi();   //����ǰ���������HMI

	void PrintError();	//��ӡ������д�����Ϣ

	void Clear();   //��ո澯����

	void ClearWarning(uint8_t chn);  //��ո澯�����µȼ�����Ϣ

	void RemoveWarning(uint8_t chn, uint16_t error_code);   //���ָ���澯

    CircularBuffer<ErrorInfo>* GetWarningList(){return m_error_info_input_list;}

private:  //˽�нӿں���
	/**
	 * @brief �������๹�캯��
	 * ���캯��Ϊ˽�У���ֹ���ʹ��new��������ʵ��
	 * @param ��
	 * @return ��
	 */
	AlarmProcessor();

	bool SendToHmi(ErrorInfo *err); //���澯��Ϣ������HMI


	void ProcessAlarm(ErrorInfo *err); //�澯��ͳһ������

private:

	static AlarmProcessor* m_instance; 	//��ʵ��ָ��
	pthread_mutex_t m_mutex;    //��д������
	ErrorInfo m_latest_error;   //���һ�εĸ澯��Ϣ
	CircularBuffer<ErrorInfo>* m_error_info_input_list;    //�澯��Ϣ�б�

	HMICommunication *m_p_hmi_comm;   //HMIͨѶ�ӿ�
	ChannelEngine *m_p_chn_engine;		//ͨ������ָ��
};


#endif /* INC_ALARM_PROCESSOR_ALARM_PROCESSOR_H_ */
