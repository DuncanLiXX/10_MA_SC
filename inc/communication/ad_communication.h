/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file ad_communication.h
 *@author gonghao
 *@date 2020/11/03
 *@brief ��ͷ�ļ�Ϊ�����豸ͨѶ�������
 *@version
 */

#ifndef SRC_COMMUNICATION_AD_COMMUNICATION_H_
#define SRC_COMMUNICATION_AD_COMMUNICATION_H_

#include <circular_buffer.h>
#include <list_buffer.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <semaphore.h>

#include "comm_data_definition.h"

class ChannelEngine;


#define PORT_AD_UDP_CMD   (9550)   //UDP����ͨѶ�˿�

/**
 * @brief  ���帨���豸�������
 */
enum ADCmdCode{
	CMD_AD_GET_MACRO_VAR = 0x10,	//��ȡ�����ֵ
	CMD_AD_SET_MACRO_VAR = 0x20,	//���ú����ֵ
	CMD_AD_GET_PMC_REG = 0x30,		//����PMC�Ĵ���ֵ
	CMD_AD_SET_PMC_REG = 0x40,		//����PMC�Ĵ���ֵ

	CMD_AD_GUARD
};


class ADCommunication {
public:

	virtual ~ADCommunication();

	static ADCommunication *GetInstance();   //����ģʽ����ȡ����ʵ����Ψһ���ʵ�

	bool SendCmd(HMICmdFrame &cmd, sockaddr_in &addr);           //����UDP����

	void SetInterface();  //����ͨ������ָ��

	ErrorType GetErrorCode(){return m_error_code;}	//���ش�����


private:
	ADCommunication();
	int Initialize();

//	static void *UdpResendCmdThread(void *args);  //UDP�����ط������̺߳���
	static void *UdpProcessCmdThread(void *args); //UDP����������̺߳���

	int Clean();     //�����ƺ�����


//	int ResendHmiCmd();                  //HMI�����ط�����
	int ProcessADCmd();                 //������յ�HMI����
	void RecvADCmd();					//����HMI��UDP����

//	int ResendCmd(HMICmdFrame &cmd);        //�ط�UDP��������뷢�Ͷ���

//	bool DelCmd(uint16_t frame_index);   //�ڷ��Ͷ�����ɾ����Ӧ����

	uint16_t GetFrameNumber(){return  (++m_n_frame_number&0x8000) ? (m_n_frame_number=0) : m_n_frame_number;}   //��ȡ����֡��

	void ProcessGetMacroVarCmd(HMICmdFrame &cmd, sockaddr_in &addr);   	//�����ȡ���������
	void ProcessSetMacroVarCmd(HMICmdFrame &cmd, sockaddr_in &addr);		//�������ú��������
	void ProcessGetPmcRegCmd(HMICmdFrame &cmd, sockaddr_in &addr);		//�����ȡPMC�Ĵ�������
	void ProcessSetPmcRegCmd(HMICmdFrame &cmd, sockaddr_in &addr);		//��������PMC�Ĵ�������


private:
	static ADCommunication *m_p_instance;    //��ʵ������
	ChannelEngine *m_p_channel_engine;   //ͨ������ָ��

	static int m_soc_udp_recv;    //UDP�������socket
	int m_soc_udp_send;    //UDP�����socket

	pthread_mutex_t m_mutex_udp_send;   //UDP�������ݻ�����,���ڷ�ֹ�ڲ�ͬ�ĵط�ͬʱ��ɾ������

	//�����豸UDPͨѶ�ӿ�
	struct sockaddr_in m_addr_udp_recv;      //�����豸��UDP�����ַ
	static CircularBuffer<HMICmdRecvNode> *m_list_recv_ad;   	//���յ��ĸ����豸��������λ���
//	ListBuffer<HMICmdResendNode *> *m_list_send;   			//���͵������豸�����������

	pthread_t m_thread_process_cmd;    //UDP������߳�
//	pthread_t m_thread_resend_cmd;     //UDP�����ط��߳�

	uint16_t m_n_frame_number;         //��ǰ֡��

	ErrorType m_error_code;           //������
};



#endif /* SRC_COMMUNICATION_AD_COMMUNICATION_H_ */
