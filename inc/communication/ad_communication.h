/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file ad_communication.h
 *@author gonghao
 *@date 2020/11/03
 *@brief 本头文件为辅助设备通讯类的声明
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


#define PORT_AD_UDP_CMD   (9550)   //UDP命令通讯端口

/**
 * @brief  定义辅助设备命令代码
 */
enum ADCmdCode{
	CMD_AD_GET_MACRO_VAR = 0x10,	//获取宏变量值
	CMD_AD_SET_MACRO_VAR = 0x20,	//设置宏变量值
	CMD_AD_GET_PMC_REG = 0x30,		//请求PMC寄存器值
	CMD_AD_SET_PMC_REG = 0x40,		//设置PMC寄存器值

	CMD_AD_GUARD
};


class ADCommunication {
public:

	virtual ~ADCommunication();

	static ADCommunication *GetInstance();   //单例模式，获取此类实例的唯一访问点

	bool SendCmd(HMICmdFrame &cmd, sockaddr_in &addr);           //发送UDP命令

	void SetInterface();  //设置通道引擎指针

	ErrorType GetErrorCode(){return m_error_code;}	//返回错误码


private:
	ADCommunication();
	int Initialize();

//	static void *UdpResendCmdThread(void *args);  //UDP命令重发处理线程函数
	static void *UdpProcessCmdThread(void *args); //UDP接收命令处理线程函数

	int Clean();     //处理善后事宜


//	int ResendHmiCmd();                  //HMI命令重发函数
	int ProcessADCmd();                 //处理接收的HMI命令
	void RecvADCmd();					//接收HMI的UDP命令

//	int ResendCmd(HMICmdFrame &cmd);        //重发UDP命令，不加入发送队列

//	bool DelCmd(uint16_t frame_index);   //在发送队列中删除对应报文

	uint16_t GetFrameNumber(){return  (++m_n_frame_number&0x8000) ? (m_n_frame_number=0) : m_n_frame_number;}   //获取最新帧号

	void ProcessGetMacroVarCmd(HMICmdFrame &cmd, sockaddr_in &addr);   	//处理获取宏变量命令
	void ProcessSetMacroVarCmd(HMICmdFrame &cmd, sockaddr_in &addr);		//处理设置宏变量命令
	void ProcessGetPmcRegCmd(HMICmdFrame &cmd, sockaddr_in &addr);		//处理获取PMC寄存器命令
	void ProcessSetPmcRegCmd(HMICmdFrame &cmd, sockaddr_in &addr);		//处理设置PMC寄存器命令


private:
	static ADCommunication *m_p_instance;    //单实例对象
	ChannelEngine *m_p_channel_engine;   //通道引擎指针

	static int m_soc_udp_recv;    //UDP命令接收socket
	int m_soc_udp_send;    //UDP命令发送socket

	pthread_mutex_t m_mutex_udp_send;   //UDP发送数据互斥锁,用于防止在不同的地方同时做删除动作

	//辅助设备UDP通讯接口
	struct sockaddr_in m_addr_udp_recv;      //辅助设备的UDP命令地址
	static CircularBuffer<HMICmdRecvNode> *m_list_recv_ad;   	//接收到的辅助设备命令包环形缓冲
//	ListBuffer<HMICmdResendNode *> *m_list_send;   			//发送到辅助设备的命令包队列

	pthread_t m_thread_process_cmd;    //UDP命令处理线程
//	pthread_t m_thread_resend_cmd;     //UDP命令重发线程

	uint16_t m_n_frame_number;         //当前帧号

	ErrorType m_error_code;           //错误码
};



#endif /* SRC_COMMUNICATION_AD_COMMUNICATION_H_ */
