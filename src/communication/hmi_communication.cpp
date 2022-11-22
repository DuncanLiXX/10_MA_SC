/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file HMICommunication.cpp
 *@author gonghao
 *@date 2020/03/19
 *@brief 本文件为HMI通讯类的实现
 *@version
 */

#include <hmi_communication.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <dirent.h>
#include <sys/socket.h>
#include <sys/types.h>
#include "global_include.h"
#include "channel_engine.h"
#include "channel_control.h"
#include "alarm_processor.h"
#include "license_interface.h"
#include "backup_info.h"

struct timeval tv_hmi_heart1, tv_hmi_heart2, tv_hmi_heart3, tv_hmi_cmd1, tv_hmi_cmd2;

//template<> int ListNode<HMICmdResendNode *>::new_count = 0;

unsigned int HMICommunication::m_n_hmi_heartbeat = 0;      //初始化HMI心跳计数
HMICommunication* HMICommunication::m_p_instance = nullptr;  //初始化单例对象指针为空
int HMICommunication::m_soc_udp_recv = -1;       //初始化UDP命令接受套接字
//int HMICommunication::m_soc_tcp_monitor = -1;    //初始化TCP监控数据监听套接字
//int HMICommunication::m_soc_monitor_send = -1;  //初始化监控数据发送socket
CircularBuffer<HMICmdRecvNode> *HMICommunication::m_list_recv = nullptr;  //初始化udp命令接收缓冲指针
pthread_mutex_t HMICommunication::m_mutex_udp_recv;    //初始化udp命令接受互斥量
sem_t HMICommunication::m_sem_udp_recv;   //初始化静态信号量

struct sockaddr_in HMICommunication::m_addr_udp_hmi;  //初始化HMI地址结构
socklen_t HMICommunication::m_n_addr_len;       //初始化HMI地址结构长度
int HMICommunication::m_thread_run_flag = 0;   //线程运行标志


/**
 * @brief HMI通讯类构造函数
 */
HMICommunication::HMICommunication() {
	// TODO Auto-generated constructor stub

	m_error_code = static_cast<ErrorType>(Initialize()); //初始化

}


/**
 * @brief HMI通讯类析构函数
 */
HMICommunication::~HMICommunication() {
	// TODO Auto-generated destructor stub

	Clean();

}

/**
 * @brief 获取类实例的唯一访问接口函数，单例模式
 */
HMICommunication* HMICommunication::GetInstance(){
	if(NULL == m_p_instance){
		m_p_instance = new HMICommunication();
	}
	return m_p_instance;
}

/**
 * @brief 初始化函数
 * @return
 */
int HMICommunication::Initialize(){
	int res = ERR_NONE;
//	const int on= 1;
	//sigset_t zeromask, newmask,oldmask;    //信号集

	sockaddr_in addr;
	int reuse = 1;    //地址可重用，防止SC主动退出后，端口TIME_WAIT状态

	pthread_attr_t attr;
	struct sched_param param;
	struct timeval tcp_sock_timeout = {3, 0};  //3秒  TCP连接recv数据超时时间
	int flag = 0;

	int KeepAliveProbes = 2;		//重试次数
	int KeepAliveIntvl=1;			//两次心跳的间隔，单位：秒
	int KeepAliveTime = 10;			//连接建立后多久开始心跳，单位：秒

	this->m_n_hmi_heartbeat = 0;
	this->m_n_frame_number = 0;
	this->m_b_trans_file = false;
	this->m_file_type = FILE_NO_TYPE;
	this->m_b_recv_file = true;  //默认接收文件
	this->m_mask_config_pack = 0x00;

	this->m_p_channel_engine = nullptr;   //初始化通道引擎指针为空

	memset(m_str_file_name, 0x00, kMaxFileNameLen);
//	m_n_alarm_file_type = 0;

	m_soc_udp_recv = socket(AF_INET, SOCK_DGRAM, 0);
	m_soc_udp_send = socket(AF_INET, SOCK_DGRAM, 0);
	m_soc_tcp_monitor = socket(AF_INET, SOCK_STREAM, 0);
	m_soc_tcp_file = socket(AF_INET, SOCK_STREAM, 0);
	m_soc_monitor_send = -1;
	m_soc_file_send = -1;
	if(m_soc_udp_recv == -1 || m_soc_udp_send == -1 ||
		m_soc_tcp_monitor == -1 || m_soc_tcp_file == -1){
//		printf("ERROR! Init socket failed!\n");
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口创建套接字失败！");
		res = ERR_SC_INIT;
		goto END;
	}

	//设置TCP连接心跳机制
	setsockopt(m_soc_tcp_monitor, IPPROTO_TCP, TCP_KEEPCNT, (void *)&KeepAliveProbes, sizeof(KeepAliveProbes));
	setsockopt(m_soc_tcp_monitor, IPPROTO_TCP, TCP_KEEPIDLE, (void *)&KeepAliveTime, sizeof(KeepAliveTime));
	setsockopt(m_soc_tcp_monitor, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&KeepAliveIntvl, sizeof(KeepAliveIntvl));
	setsockopt(m_soc_tcp_file, IPPROTO_TCP, TCP_KEEPCNT, (void *)&KeepAliveProbes, sizeof(KeepAliveProbes));
	setsockopt(m_soc_tcp_file, IPPROTO_TCP, TCP_KEEPIDLE, (void *)&KeepAliveTime, sizeof(KeepAliveTime));
	setsockopt(m_soc_tcp_file, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&KeepAliveIntvl, sizeof(KeepAliveIntvl));


	//设置监控连接accept函数超时时间
	if( 0 != setsockopt(m_soc_tcp_monitor, SOL_SOCKET, SO_RCVTIMEO, (char *)&tcp_sock_timeout, sizeof(timeval))){
		g_ptr_trace->PrintLog(LOG_ALARM, "设置TCP监控套接字超时时间失败！");
		res = ERR_SC_INIT;
		goto END;
	}
	if( 0 != setsockopt(m_soc_tcp_file, SOL_SOCKET, SO_RCVTIMEO, (char *)&tcp_sock_timeout, sizeof(timeval))){
		g_ptr_trace->PrintLog(LOG_ALARM, "设置TCP文件传输套接字超时时间失败！");
		res = ERR_SC_INIT;
		goto END;
	}

	//设置TCP连接为可重用
	if(0 != setsockopt(m_soc_tcp_monitor, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse))){
		g_ptr_trace->PrintLog(LOG_ALARM, "设置TCP监控套接字可重用属性失败！");
		res = ERR_SC_INIT;
		goto END;
	}
	if(0 != setsockopt(m_soc_tcp_file, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse))){
		g_ptr_trace->PrintLog(LOG_ALARM, "设置TCP文件传输套接字可重用属性失败！");
		res = ERR_SC_INIT;
		goto END;
	}


	m_list_recv = new CircularBuffer<HMICmdRecvNode>(kCmdRecvBufLen);  //分配接收命令缓冲区
	m_list_send = new ListBuffer<HMICmdResendNode *>();   //初始化重发命名链表
	if(m_list_recv == nullptr || m_list_send == nullptr){
//		printf("ERROR! List buffer init failed!\n");
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口命令缓冲内存分配失败！");
		res = ERR_SC_INIT;
		goto END;
	}

	//初始化UDP接收地址及端口
	bzero(&m_addr_udp_recv, sizeof(m_addr_udp_recv));
	m_addr_udp_recv.sin_family = AF_INET;
	m_addr_udp_recv.sin_addr.s_addr = INADDR_ANY;
	m_addr_udp_recv.sin_port = htons(PORT_UDP_CMD);

	bind(m_soc_udp_recv, (sockaddr *)&m_addr_udp_recv, sizeof(m_addr_udp_recv));  //绑定UDP命令接收端口

	//初始化HMI端地址
	bzero(&m_addr_udp_hmi, sizeof(m_addr_udp_hmi));
	m_addr_udp_hmi.sin_family = AF_INET;
	this->m_n_addr_len = sizeof(struct sockaddr);

//#ifdef USES_MODBUS_PROTOCAL
//	//初始化modbus协议下的文件传输IP
//	bzero(&m_addr_file_trans, sizeof(m_addr_file_trans));
//	m_addr_file_trans.sin_family = AF_INET;
//#endif


	bzero(&addr, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(PORT_TCP_MONITOR);
	bind(m_soc_tcp_monitor, (struct sockaddr *)&addr, sizeof(sockaddr));   //绑定monitro TCP监听端口

	bzero(&addr, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(PORT_TCP_FILE);
	bind(m_soc_tcp_file, (struct sockaddr *)&addr, sizeof(sockaddr));   //绑定文件传输 TCP监听端口

	//初始化互斥量
	pthread_mutex_init(&m_mutex_udp_recv, nullptr);
	pthread_mutex_init(&m_mutex_udp_send, nullptr);
	pthread_mutex_init(&m_mutex_tcp_monitor, nullptr);



	//注册SIGIO信号处理函数
//	struct sigaction act;
//	act.sa_handler = HMICommunication::SigioHandler;
//	act.sa_flags = SA_RESTART;
//	sigaction(SIGIO, &act, nullptr);

//	fcntl(m_soc_udp_recv, F_SETOWN, getpid());   //设置socket属主
//	ioctl(m_soc_udp_recv, FIOASYNC, &on);        //设置为异步非阻塞方式
//	ioctl(m_soc_udp_recv, FIONBIO, &on);         //有效SIGIO信号

	flag = fcntl(m_soc_udp_recv, F_GETFL, 0);
	if(flag < 0){
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口命令获取UDP接口属性失败！");
		res = ERR_SC_INIT;
		goto END;
	}
	if(fcntl(m_soc_udp_recv, F_SETFL, flag|O_NONBLOCK) < 0){
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口命令获取UDP接口设置非阻塞模式失败！");
		res = ERR_SC_INIT;
		goto END;
	}


	//初始化信号量
	sem_init(&m_sem_udp_recv, 0, 0);
	sem_init(&m_sem_tcp_file, 0, 0);
//	sem_init(&m_sem_tcp_send_test, 0, 0);
    sem_init(&m_sem_background, 0, 0);

	this->m_thread_saveas = 0;  //另存为线程初始化为0

	//开启线程
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 39; //99;
	pthread_attr_setschedparam(&attr, &param);
	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
	if (res) {
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口命令处理线程设置线程继承模式失败！");
		m_error_code = ERR_SC_INIT;
		goto END;
	}
	res = pthread_create(&m_thread_process_cmd, &attr,
			HMICommunication::UdpProcessCmdThread, this);    //开启HMI命令处理线程
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口命令处理线程启动失败！");
		res = ERR_SC_INIT;
		goto END;
	}

	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 36; //96;
	pthread_attr_setschedparam(&attr, &param);
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
//	if (res) {
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口命令重发线程设置线程继承模式失败！");
//		m_error_code = ERR_SC_INIT;
//		goto END;
//	}
	res = pthread_create(&m_thread_resend_cmd, &attr,
			HMICommunication::UdpResendCmdThread, this);    //开启HMI命令重发线程
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口命令重发线程启动失败！");
		res = ERR_SC_INIT;
		goto END;
	}

	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 38; //98;
	pthread_attr_setschedparam(&attr, &param);
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
//	if (res) {
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口数据监控线程设置线程继承模式失败！");
//		m_error_code = ERR_SC_INIT;
//		goto END;
//	}
	res = pthread_create(&m_thread_monitor, &attr,
			HMICommunication::TcpMonitorThread, this);    //开启数据刷新线程
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口数据监控线程启动失败！");
		res = ERR_SC_INIT;
		goto END;
	}

	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 30; //90;
	pthread_attr_setschedparam(&attr, &param);
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
//	if (res) {
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口文件传输线程设置线程继承模式失败！");
//		m_error_code = ERR_SC_INIT;
//		goto END;
//	}
    res = pthread_create(&m_thread_trans_file, &attr,
			HMICommunication::TcpTransFileThread, this);    //开启文件传输线程
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口文件传输线程启动失败！");
		res = ERR_SC_INIT;
		goto END;
	}

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//
    param.__sched_priority = 29; //90;
    pthread_attr_setschedparam(&attr, &param);
    res = pthread_create(&m_thread_background, &attr,
            HMICommunication::BackgroundThread, this);    //开启文件传输线程
    if (res != 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口文件传输线程启动失败！");
        res = ERR_SC_INIT;
        goto END;
    }
END:
	pthread_attr_destroy(&attr);
	return res;
}

/**
 * @brief 设置接口指针
 * @param 无
 */
void HMICommunication::SetInterface(){
	this->m_p_channel_engine = ChannelEngine::GetInstance();
}

/**
 * @brief 执行复位
 */
void HMICommunication::Reset(){

	this->m_error_code = ERR_NONE;

	//清空命令缓冲
	pthread_mutex_lock(&m_mutex_udp_send);
	m_list_send->Clear();	//清空重发队列
	pthread_mutex_unlock(&m_mutex_udp_send);
	m_list_recv->ClearBuf();	//清空接收命令队列

	//取消当前的文件传输
	if(m_b_trans_file){

	}
}

/**
 * @brief 处理善后事宜
 * @return
 */
int HMICommunication::Clean(){

	int res = ERR_NONE;
//	void* thread_result;

//	//退出命令处理线程
//	res = pthread_cancel(m_thread_process_cmd);
//	if (res != ERR_NONE) {
// //		printf("Cmd process thread cancel failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口命令处理线程退出失败！");
//	}
//
//	usleep(1000);
//	res = pthread_join(m_thread_process_cmd, &thread_result);//等待命令处理线程退出完成
//	if (res != ERR_NONE) {
// //		printf("Cmd process thread join failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口等待命令处理线程退出失败！");
//	}
//	m_thread_process_cmd = 0;
//
//	//退出重发线程
//	res = pthread_cancel(m_thread_resend_cmd);
//	if (res != ERR_NONE) {
// //		printf("Cmd resend thread cancel failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口命令重发线程退出失败！");
//	}
//	usleep(1000);
//
//	res = pthread_join(m_thread_resend_cmd, &thread_result);//等待重发线程退出完成
//	if (res != ERR_NONE) {
// //		printf("Cmd resend thread join failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口等待命令重发线程退出失败！");
//	}
//	m_thread_resend_cmd = 0;
//
//	//退出数据刷新线程
//	res = pthread_cancel(m_thread_monitor);
//	if (res != ERR_NONE) {
// //		printf("Monitor thread cancel failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口数据监控线程退出失败！");
//	}
//	usleep(1000);
//	res = pthread_join(m_thread_monitor, &thread_result);//等待数据刷新线程退出完成
//	if (res != ERR_NONE) {
// //		printf("Monitor thread join failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口等待数据监控线程退出失败！");
//	}
//	m_thread_monitor = 0;
//
//	//退出文件传输线程
//	res = pthread_cancel(m_thread_trans_file);
//	if (res != ERR_NONE) {
// //		printf("File trans thread cancel failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口文件传输线程退出失败！");
//	}
//	usleep(1000);
//	res = pthread_join(m_thread_trans_file, &thread_result);//等待文件传输线程 退出完成
//	if (res != ERR_NONE) {
//		printf("File trans thread join failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口等待文件传输线程退出失败！");
//	}
//	m_thread_trans_file = 0;
//
//
//	if(m_thread_saveas != 0){
//		res = pthread_cancel(m_thread_saveas);
//		if (res != ERR_NONE) {
//	//		printf("Monitor thread cancel failed\n");
//			g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口文件另存为线程退出失败！");
//		}
//		usleep(1000);
//		res = pthread_join(m_thread_saveas, &thread_result);//等待文件另存为线程 退出完成
//		if (res != ERR_NONE) {
//	//		printf("File save as thread join failed\n");
//			g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口文件另存为线程退出失败！");
//		}
//		m_thread_saveas = 0;
//	}

	int i = 0;
	while(this->m_thread_run_flag != 0){
		usleep(100000);
		if(i++>10)
		{
			printf("wait thread timeout:%x\n", this->m_thread_run_flag);
			i = 0;
			break;
		}
	}

	if(m_soc_monitor_send > 0){
		printf("close m_soc_monitor_send\n");
		close(m_soc_monitor_send);
		m_soc_monitor_send = -1;
	}

	if(m_soc_file_send > 0){
		close(m_soc_file_send);
		m_soc_file_send = -1;
	}


	//关闭网络套接字
	close(m_soc_tcp_file);
	close(m_soc_tcp_monitor);
	close(m_soc_udp_recv);
	close(m_soc_udp_send);

	if(m_list_recv){
		delete m_list_recv;
		m_list_recv = nullptr;
	}
	if(m_list_send){
		delete m_list_send;
		m_list_send = nullptr;
	}

	pthread_mutex_destroy(&m_mutex_udp_recv);
	pthread_mutex_destroy(&m_mutex_udp_send);
	pthread_mutex_destroy(&m_mutex_tcp_monitor);

	printf("exit HMICommunication::Clean()!\n");

	return res;
}

/**
 * @brief 发送UDP命令
 * @param HMICmdFrame &cmd: 待发送的命令包
 * @return true--发送成功  false--发送失败
 */
bool HMICommunication::SendCmd(HMICmdFrame &cmd){

	int res = 0;

	if(!g_sys_state.hmi_comm_ready)  //HMI连接尚未准备好
		return false;

	//确认帧号
	if((cmd.frame_number & 0x8000) == 0){
		cmd.frame_number = this->GetFrameNumber();  //非回复数据包，自动赋予帧号
	}

	//发送UDP包
	int len = kHmiCmdHeadLen+cmd.data_len;
	res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&m_addr_udp_hmi, m_n_addr_len);
	if(len != res){
		//printf("ERROR! Failed to send udp cmd to hmi!%d, %d\n", len,res);
		g_ptr_trace->PrintLog(LOG_ALARM, "发送HMI命令包失败，应发%d字节，实发%d字节，errno = %d！", len, res, errno);
//		return false;
	}

    //将UDP命令包（响应包除外）放入重发队列
	if((cmd.frame_number & 0x8000) == 0){//非响应包
		HMICmdResendNode *pNode = new HMICmdResendNode();
		if(pNode != NULL){
			pNode->frame = cmd;
			pNode->resend_count = kHmiCmdResend;
			pNode->timeout = kHmiCmdTimeout;

			m_list_send->Append(pNode);  //此处只做添加，可以不用m_mutex_udp_send互斥，因为内部已互斥
//			if(ListNode<HMICmdResendNode *>::new_count > 0)
//				printf("HMICmdResendNode new count :%d\n", ListNode<HMICmdResendNode *>::new_count);
		}
		
	}


	return true;
}

/**
 * @brief 向源IP发送命令
 * @param cmd_node :  命令数据节点
 * @return  true--发送成功  false--发送失败
 */
bool HMICommunication::SendCmdToIp(HMICmdRecvNode &cmd_node){
	int res = 0;

	if(!g_sys_state.hmi_comm_ready)  //HMI连接尚未准备好
		return false;

	HMICmdFrame &cmd = cmd_node.cmd;
	//确认帧号
	if((cmd.frame_number & 0x8000) == 0){
		cmd.frame_number = this->GetFrameNumber();  //非回复数据包，自动赋予帧号
	}

//#ifdef USES_MODBUS_PROTOCAL
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //更改端口
//#endif
	cmd_node.ip_addr.sin_port = this->m_addr_udp_hmi.sin_port;  //更改端口

	//发送UDP包
	int len = kHmiCmdHeadLen+cmd.data_len;
	res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
	if(len != res){
		//printf("ERROR! Failed to send udp cmd to hmi!%d, %d\n", len,res);
		g_ptr_trace->PrintLog(LOG_ALARM, "发送HMI命令包失败，应发%d字节，实发%d字节，errno = %d！", len, res, errno);
//		return false;
	}

#ifndef USES_MODBUS_PROTOCAL
    //将UDP命令包（响应包除外）放入重发队列
	if((cmd.frame_number & 0x8000) == 0){//非响应包
		HMICmdResendNode *pNode = new HMICmdResendNode();
		if(pNode != NULL){
			pNode->frame = cmd;
			pNode->resend_count = kHmiCmdResend;
			pNode->timeout = kHmiCmdTimeout;

			m_list_send->Append(pNode);  //此处只做添加，可以不用m_mutex_udp_send互斥，因为内部已互斥
//			if(ListNode<HMICmdResendNode *>::new_count > 0)
//				printf("HMICmdResendNode new count :%d\n", ListNode<HMICmdResendNode *>::new_count);
		}

	}
#endif

	return true;
}

/**
 * @brief 发送监控数据
 * @param buf ：待发送数据缓冲
 * @param size ：发送数据字节数
 * @return 成功发送字节数
 */
int HMICommunication::SendMonitorData(char *buf, int size){
	if(this->m_soc_monitor_send == -1 || buf == nullptr)
		return 0;

	pthread_mutex_lock(&m_mutex_tcp_monitor);

	int res = send(m_soc_monitor_send, buf, size, MSG_NOSIGNAL);

	if(res == -1){
		if(errno == 32 || errno == 104){
			g_ptr_trace->PrintLog(LOG_ALARM, "HMI监控连接中断！errno=%d", errno);
		//	CreateError(ERR_HMI_MONITOR_DISCON, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON);  //生成HMI监控断链告警
		}
	}else if(res != size){
//		printf("warnning: send = %d, res = %d\n", size, res);
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI监控连接发送数据异常[send=%d, res=%d]！errno=%d", size, res, errno);
	}


	pthread_mutex_unlock(&m_mutex_tcp_monitor);
	return res;
}

/**
 * @brief 重发UDP命令，不将命令包加入发送队列
 * @param HMICmdFrame &cmd: 待发送的命令包
 * @return 实际发送字节数
 */
int HMICommunication::ResendCmd(HMICmdFrame &cmd){
	int res = 0;

	if(!g_sys_state.hmi_comm_ready)  //HMI连接尚未准备好
		return res;

	//发送UPD包
	int len = kHmiCmdHeadLen+cmd.data_len;
	res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&m_addr_udp_hmi, m_n_addr_len);
	if(len != res){
	//	printf("ERROR! Failed to resend udp cmd to hmi!%d, %d\n", len,res);
		g_ptr_trace->PrintLog(LOG_ALARM, "重发HMI命令包失败，应发%d字节，实发%d字节！errno = %d", len, res, errno);
	}

	return res;
}

/**
 * @brief SIGIO信号处理函数
 * @param int signo: 收到的信号
 */
//void HMICommunication::SigioHandler(int signo) {
//
//	if(signo != SIGIO){
//		printf("other signal %d:%d\n", signo, SIGIO);
//		return;
//	}
//
//	static int sigio_mutex = 0;   //防止嵌套
//
//	if(sigio_mutex == 0){
//		sigio_mutex = 1;
//	}else if(sigio_mutex == 1){
//		printf("sigio handdler nested!\n");
//		return;
//	}
//
////	pthread_mutex_lock(&m_mutex_udp_recv);
//
//
//	if(m_list_recv->EmptyBufLen() == 0){  //接收缓冲已满
//		printf("ERROR！HMI CMD buffer overflow!!!\n");
////		pthread_mutex_unlock(&m_mutex_udp_recv);
//		sigio_mutex = 0;
//		return;
//	}
//
////	ssize_t read_len = 0;   //实际读取字节数
//
//	HMICmdRecvNode cmd_node;
//	HMICmdFrame &data = cmd_node.cmd;
//	ssize_t res = 0;   //接收数据返回值
//
//	while(1){
//		bzero((char *)&data, kMaxHmiCmdFrameLen);
//
//		res = recvfrom(m_soc_udp_recv, &data, kMaxHmiCmdFrameLen, 0, (struct sockaddr *)&cmd_node.ip_addr, &m_n_addr_len);
//
//		if(res == -1 && errno == EAGAIN){	//无数据可读
//			break;
//		}else if (res == 0){
//			printf("recvfrom error, errno = %d\n", errno);
//			break;
//		}
//
//
//		if(data.cmd == CMD_HMI_HEART_BEAT){  //在此处处理心跳，保证优先处理，不会因为处理耗时命令而误发心跳丢失
//			if(g_sys_state.hmi_comm_ready){
//				if(strcmp(g_sys_state.hmi_host_addr, inet_ntoa(cmd_node.ip_addr.sin_addr)) == 0){
//		//			printf("get hmi heartbeat cmd :%d\n", m_n_hmi_heartbeat);
//					m_n_hmi_heartbeat = 0;  //心跳计数归零
//		//			sigio_mutex = 0;
//		//			pthread_mutex_unlock(&m_mutex_udp_recv);
//					continue;
//				}
//			}
//			else{//未连接状态，丢弃心跳包
//				printf("no hmi host connected, drop the heartbeat package!\n");
//		//		sigio_mutex = 0;
//		//		pthread_mutex_unlock(&m_mutex_udp_recv);
//				continue;
//			}
//		}
//
//		m_list_recv->WriteData(&cmd_node, 1);
//	}
//
//
//
//	sigio_mutex = 0;
//
//
////	pthread_mutex_unlock(&m_mutex_udp_recv);
//
//
//
//	//发送信号
//	if(m_list_recv->BufLen() > 0)
//		sem_post(&m_sem_udp_recv);
//}

/**
 * @brief HMI命令重发函数, 每100ms轮询一次
 * @param 无
 */
int HMICommunication::ResendHmiCmd(){
	int res = ERR_NONE;

	ListNode<HMICmdResendNode *> *pNode = nullptr, *del_node = nullptr;
	while(!g_sys_state.system_quit){
		if(g_sys_state.eth0_running && g_sys_state.hmi_comm_ready){
			//遍历HMI命令发送队列
			if(!m_list_send->IsEmpty()){
				pthread_mutex_lock(&m_mutex_udp_send);
				pNode = m_list_send->HeadNode();
				while(pNode != NULL){
					if( 0 == --(pNode->data->timeout) ){//接收超时
						if(pNode->data->resend_count == 0){
							//TODO 已经到达最大重发次数，此处需要增加告警处理
							g_ptr_trace->PrintLog(LOG_ALARM,"ERROR! HMI can not be reached!cmd=%d, cmd_ex=%d, frame_index = %hu", pNode->data->frame.cmd,
									pNode->data->frame.cmd_extension, pNode->data->frame.frame_number);
							del_node = pNode;
							pNode = pNode->next;
							m_list_send->Delete(del_node);
							del_node = nullptr;
							continue;
						}
						ResendCmd(pNode->data->frame);
						pNode->data->timeout = kHmiCmdTimeout;
						pNode->data->resend_count--;
					}
					pNode = pNode->next;
				}
				pthread_mutex_unlock(&m_mutex_udp_send);
			}
		}

		usleep(100000);  //休眠100ms
	}



	return res;
}

/**
 * @brief 接收HMI的UDP命令
 */
void HMICommunication::RecvHmiCmd(){
	if(m_list_recv->EmptyBufLen() == 0){  //接收缓冲已满
		printf("WARNING！HMI CMD buffer overflow!!!\n");
		return;
	}

//	static struct timeval tvLast;
//	static	struct timeval tvNow;
//	unsigned int nTimeDelay = 0;

//	ssize_t read_len = 0;   //实际读取字节数

	HMICmdRecvNode cmd_node;
	HMICmdFrame &data = cmd_node.cmd;
	ssize_t res = 0;   //接收数据返回值

	while(1){

		if(m_list_recv->EmptyBufLen() == 0){  //接收缓冲已满
			break;
		}

		bzero((char *)&data, kMaxHmiCmdFrameLen);

		res = recvfrom(m_soc_udp_recv, &data, kMaxHmiCmdFrameLen, 0, (struct sockaddr *)&cmd_node.ip_addr, &m_n_addr_len);

		if(res == -1 && errno == EAGAIN){	//无数据可读

			break;
		}else if (res == 0){
			printf("recvfrom error, errno = %d\n", errno);
			break;
		}



        if(data.cmd == CMD_HMI_HEART_BEAT){  //在此处处理心跳，保证优先处理，不会因为处理耗时命令而误发心跳丢失

			if(g_sys_state.hmi_comm_ready){
				if(strcmp(g_sys_state.hmi_host_addr, inet_ntoa(cmd_node.ip_addr.sin_addr)) == 0){

					m_n_hmi_heartbeat = 0;  //心跳计数归零

					gettimeofday(&tv_hmi_heart1, NULL);
					int nn = (tv_hmi_heart1.tv_sec-tv_hmi_heart3.tv_sec)*1000000+tv_hmi_heart1.tv_usec-tv_hmi_heart3.tv_usec;
					if(nn > 900000){
						printf("receive hmi heartbeat timeout: %d\n", nn);
					}

					tv_hmi_heart3 = tv_hmi_heart1;
					continue;
				}else{
					printf("unknow hmi heartbeat\n");
				}
			}
			else{//未连接状态，丢弃心跳包
				printf("no hmi host connected, drop the heartbeat package!\n");

				continue;
			}
		}

		if(1 != m_list_recv->WriteData(&cmd_node, 1)){
			g_ptr_trace->PrintLog(LOG_ALARM, "HMI命令缓冲队列溢出！");
			break;
		}

		if(m_list_recv->EmptyBufLen() == 0){  //接收缓冲已满
			break;
		}
	}
}

/**
 * @brief HMI命令处理函数
 * @param 无
 */
int HMICommunication::ProcessHmiCmd(){
	int res = ERR_NONE;
//	struct timespec time_out= {0, 10000000};   //信号等待超时时间


	HMICmdRecvNode cmd_node;
	HMICmdFrame &cmd = cmd_node.cmd;
	uint16_t frame_index = 0;
	while(!g_sys_state.system_quit){
//		sem_wait(&m_sem_udp_recv);  //等待UDP命令到达信号
//		if(0 != sem_timedwait(&m_sem_udp_recv, &time_out) && errno == ETIMEDOUT){  //等待UDP命令到达信号, sem_timedwait函数会阻塞
//	//		usleep(1000);
//			continue;
//		}

		gettimeofday(&tv_hmi_cmd1, NULL);

		int nTimeDelay = (tv_hmi_cmd1.tv_sec-tv_hmi_cmd2.tv_sec)*1000000+tv_hmi_cmd1.tv_usec-tv_hmi_cmd2.tv_usec;
		if(nTimeDelay > 100000){
			printf("receive hmi cmd out; %d\n", nTimeDelay);
		}

		//接收udp数据
        this->RecvHmiCmd();
		tv_hmi_cmd2 = tv_hmi_cmd1;

		//遍历HMI命令接收队列
		while(m_list_recv->ReadData(&cmd_node, 1) > 0){
//			if(g_sys_state.hmi_comm_ready && cmd.cmd != CMD_HMI_DEVICE_SCAN){  //已连接HMI后丢弃其它IP发来的非设备扫描指令
//#ifdef USES_MODBUS_PROTOCAL
//				if(cmd.cmd != CMD_HMI_GET_FILE && cmd.cmd != CMD_HMI_SEND_FILE && cmd.cmd != CMD_HMI_NC_FILE_COUNT &&
//						cmd.cmd != CMD_HMI_NC_FILE_INFO && cmd.cmd != CMD_HMI_FILE_OPERATE && cmd.cmd != CMD_HMI_UPDATE){   //Modbus协议下，允许HMI直连传输文件
//#endif
//				if(strcmp(g_sys_state.hmi_host_addr, inet_ntoa(cmd_node.ip_addr.sin_addr)) != 0){
//					continue;  //
//				}
//#ifdef USES_MODBUS_PROTOCAL
//				}
//#endif
//			}

			//如果是响应报文，则在发送队列中删除对应报文
			if((cmd.frame_number & 0x8000) != 0){
				frame_index = (cmd.frame_number & 0x7FFF);
				this->DelCmd(frame_index);
			}

//			printf("receive hmi cmd[%04X %02X %02X %02X %s]\n", cmd.frame_number, cmd.cmd, cmd.cmd_extension,cmd.data_len, cmd.data);
			//对命令进行处理
			switch(cmd.cmd){
//			case CMD_HMI_HEART_BEAT: //HMI心跳  //转移到SigioHandle()中优先处理
//				m_n_hmi_heartbeat = 0;  //心跳计数归零
//				break;
			case CMD_HMI_DEVICE_SCAN:  //扫描指令
				this->ProcessHmiDeviceScan(cmd_node);
				break;
			case CMD_HMI_SHAKEHAND://HMI握手
				this->ProcessHmiShakehand(cmd_node);
				break;
			case CMD_HMI_SET_CUR_CHANNEL:   //设置当前通道
				this->ProcessHmiSetCurChnCmd(cmd);
				break;
			case CMD_HMI_GET_FILE:	//HMI获取文件
//#ifdef USES_MODBUS_PROTOCAL
//				this->ProcessHmiGetFileCmd(cmd_node);
//#else
				this->ProcessHmiGetFileCmd(cmd);
//#endif
				break;
			case CMD_HMI_SEND_FILE:	//HMI传送文件
//#ifdef USES_MODBUS_PROTOCAL
//				this->ProcessHmiSendFileCmd(cmd_node);
//#else
				this->ProcessHmiSendFileCmd(cmd);
//#endif
				break;
			case CMD_HMI_DISCONNECT:  //HMI关闭连接
				this->ProcessHmiDisconnectCmd(cmd);
				break;
			case CMD_HMI_NC_FILE_COUNT: //HMI获取SC模块存储的NC加工文件个数
				this->ProcessHmiNcFileCountCmd(cmd_node);
				break;
			case CMD_HMI_NC_FILE_INFO:  //HMI获取NC文件名称列表
				this->ProcessHmiNcFileInfoCmd(cmd_node);
				break;
			case CMD_HMI_FILE_OPERATE:   //处理HMI发来的文件操作命令
				this->ProcessHmiFileOperateCmd(cmd_node);
				break;
			case CMD_HMI_READY:   //HMI准备就绪
				this->ProcessHmiReadyCmd(cmd);
				break;
			case CMD_HMI_NEW_ALARM: //HMI告警
				ProcessHmiNewAlarmCmd(cmd);
				break;
			case CMD_HMI_GET_VERSION:  //获取版本信息
				ProcessHmiGetVersionCmd(cmd);
				break;
			case CMD_HMI_GET_FILE_SIGNATURE:  //获取NC文件签名
				ProcessHmiFileSignatureCmd(cmd_node);
				break;
			case CMD_HMI_GET_CHN_COUNT: //HMI获取通道数量
				this->ProcessHmiChnCountCmd(cmd);
				break;
			case CMD_HMI_GET_ESB_INFO:            //HMI向SC获取ESB文件数据   0x2A
				this->ProcessHmiGetEsbInfoCmd(cmd);
				break;
			case CMD_HMI_ESB_OPERATE:            //HMI通知SC对指定ESB文件进行操作  0x2B
                this->ProcessHmiOperateEsbCmd(cmd);
				break;
			case CMD_HMI_SYNC_TIME:               //HMI向SC查询当前系统时间  0x36
				this->ProcessHmiSyncTimeCmd(cmd);
				break;
//			case CMD_HMI_GET_CHN_STATE: //HMI获取通道当前状态
//				ProcessHmiGetChnStateCmd(cmd);
//				break;
				//以下指令传递给通道引擎处理
			case CMD_HMI_SET_NC_FILE:			 //设置当前加工文件 13
				//需要判断打开文件是否正在传输中
				if(m_b_trans_file && this->m_b_recv_file && m_file_type == FILE_G_CODE
						&& strcmp(m_str_file_name, cmd.data) == 0){//文件正在传输中
					cmd.frame_number |= 0x8000;
					cmd.cmd_extension = FAILED;
					this->SendCmd(cmd);
					printf("file[%s] is transfering! \n", cmd.data);
					break;
				}
			case CMD_HMI_SET_PARA:				 //参数设置
			case CMD_HMI_GET_PARA:				 //参数获取
			case CMD_HMI_GET_CHN_STATE: 		 //HMI获取通道当前状态
			case CMD_HMI_RESTART:               //加工复位 11
			case CMD_HMI_SIMULATE:				 //仿真 12
			case CMD_HMI_FIND_REF_POINT:		 //确定参考点 14   针对增量式编码器
			case CMD_SC_MDA_DATA_REQ:		 	//MDA代码请求 115
			case CMD_HMI_UPDATE:      			//HMI通知SC请求进行升级文件传送
			case CMD_HMI_GET_PMC_REG:			//获取PMC寄存器
			case CMD_HMI_SET_PMC_REG:			//设置PMC寄存器
			case CMD_HMI_GET_PMC_UUID:			//获取PMC的UUID
			case CMD_HMI_SET_REF_POINT:			//设置轴原点，针对绝对值编码器
			case CMD_HMI_GET_MACRO_VAR:			//HMI向SC请求宏变量的值
			case CMD_HMI_SET_MACRO_VAR:        //HMI向SC设置宏变量寄存器的值
			case CMD_HMI_SET_CALIBRATION:     //HMI向SC发出调高器标定指令 32
			case CMD_HMI_AXIS_MANUAL_MOVE:     //HMI指令SC轴移动指令
            case CMD_HMI_CLEAR_WORKPIECE:      //HMI请求SC将加工计数清零,临时计数(区分白夜班)
            case CMD_HMI_CLEAR_TOTAL_PIECE:    //HMI请求SC将总共件数
			case CMD_HMI_GET_LIC_INFO:            //HMI向SC请求授权信息   0x27
			case CMD_HMI_SEND_LICENSE:            //HMI向SC发送授权码     0x28
			case CMD_HMI_MANUAL_TOOL_MEASURE:     //HMI向SC发起手动对刀操作  0x29
			case CMD_HMI_GET_IO_REMAP:			//HMI向SC获取IO重定向数据  0x2C
			case CMD_HMI_SET_IO_REMAP:         //HMI向SC设置IO重定向数据  0x2D
			case CMD_HMI_SET_PROC_PARAM:      //HMI向SC设置工艺相关参数  0x2E
			case CMD_HMI_GET_PROC_PARAM:          //HMI向SC获取工艺相关参数  0x2F
			case CMD_HMI_SET_PROC_GROUP:          //HMI向SC设置当前工艺参数组号  0x30
			case CMD_HMI_GET_PROC_GROUP:          //HMI向SC获取当前工艺参数组号  0x31
			case CMD_HMI_SET_CUR_MACH_POS:        //HMI向SC重设指定轴的机械坐标  0x32
			case CMD_HMI_CLEAR_MSG:               //HMI通知SC清除消息  0x33
			case CMD_HMI_SYNC_AXIS_OPT:           //HMI通知HMI进行同步轴使能操作 0x34
			case CMD_HMI_NOTIFY_GRAPH:            //HMI通知SC进入图形模式    0x35
			case CMD_HMI_CHECK_SYNC_EN:           //HMI向SC查询同步轴状态 0x37
            case CMD_HMI_CLEAR_MACHINETIME_TOTAL: //HMI向SC请求清除累计时间
            case CMD_HMI_GET_HANDWHEEL_INFO:      //HMI向SC获取手轮信息
            case CMD_HMI_SET_HANDWHEEL_INFO:      //HMI向SC设置手轮信息
            case CMD_HMI_GET_ERROR_INFO:          //HMI向SC获取错误信息
            case CMD_HMI_SET_ALL_COORD:           //HMI向SC设置当前通道的所有工件坐标系
#ifdef USES_GRIND_MACHINE
			case CMD_SC_MECH_ARM_ERR:         //HMI响应机械手告警指令
#endif
				m_p_channel_engine->ProcessHmiCmd(cmd);
				break;

			case CMD_HMI_MOP_KEY:				//HMI虚拟MOP按键消息
				this->ProcessHmiVirtualMOPCmd(cmd);
				break;
			case CMD_SC_REQ_START:   //SC向HMI请求开始加工
				break;

			case CMD_SC_NEW_ALARM:
//				printf("get hmi reply alarm response:0x%hx\n", cmd.frame_number);
			case CMD_SC_WORK_STATE:    //HMI响应指令，无需做任何处理
			case CMD_SC_WORK_MODE:
			case CMD_SC_STATUS_CHANGE:

			case CMD_SC_CLEAR_ALARM:
			case CMD_SC_RESET:
			case CMD_SC_DISPLAY_MODE:
			case CMD_SC_EMERGENCY:              //急停
			case CMD_SC_RESTART_LINE:            //通知HMI当前加工复位操作扫描的行数
			case CMD_SC_MODE_CHANGE:
			case CMD_SC_DISPLAY_FILE:         //通知HMI切换显示的文件
			case CMD_SC_WORKPIECE_COUNT:			//通知HMI更新工件计数值
			case CMD_SC_READY:
			case CMD_SC_MESSAGE:
			case CMD_SC_UPDATE_MODULE_STATUS:  //通知HMI升级状态命令的响应
			case CMD_SC_TRANS_FILE_OVER:
			case CMD_SC_TOOL_MEASURE_OVER:    //手动对刀结束命令的响应
			case CMD_SC_PARAM_CHANGED:
			case CMD_SC_NOTIFY_MACH_OVER:     //加工结束通知消息的响应
            case CMD_SC_NOTIFY_ALARM_CHANGE:
            case CMD_SC_BACKUP_STATUS:        //SC通知HMI当前备份状态
				break;
			case CMD_HMI_GET_SYS_INFO:
				m_p_channel_engine->ProcessHmiCmd(cmd);
				break;
			case CMD_SC_NOTIFY_MCODE:
				break;
            case CMD_HMI_BACKUP_REQUEST:
                ProcessHmiSysBackupCmd(cmd);
                break;
            case CMD_HMI_RECOVER_REQUEST:
                ProcessHmiSysRecoverCmd(cmd);
                break;
			default:
				g_ptr_trace->PrintLog(LOG_ALARM, "收到不支持的HMI指令cmd=%d", cmd.cmd);
				break;
			}
		}

        usleep(10000);   //休眠10ms
	}


	return res;
}


/**
 * @brief 在发送队列中删除对应报文
 * @param frame_index: 待删除报文的帧号
 * @return
 */
bool HMICommunication::DelCmd(uint16_t frame_index){
	bool res = false;

	pthread_mutex_lock(&m_mutex_udp_send);

	//遍历发送队列
	ListNode<HMICmdResendNode *> *pNode = m_list_send->HeadNode();
	while(pNode != NULL){
		if(pNode->data->frame.frame_number == frame_index){
			res = m_list_send->Delete(pNode);
			break;
		}
		pNode = pNode->next;
	}


	pthread_mutex_unlock(&m_mutex_udp_send);
	return res;
}

/**
 * @brief 文件传输函数
 * @return
 */
int HMICommunication::TransFile(){
	int res = ERR_NONE;
	sockaddr_in addr_hmi;
	socklen_t len = sizeof(sockaddr);
//	struct timespec time_out= {0, 10000000};


//	while(!g_sys_state.system_ready){  //等待系统启动
//		usleep(100000);
//	}
	printf("trans file thread: id = %ld\n", syscall(SYS_gettid));

	if(listen(m_soc_tcp_file, 2)!= 0){
		//TODO 错误处理
		printf("ERROR! Failed to listen file trans port!err=%d\n", errno);
		res = ERR_SC_INIT;
		return res;
	}


	while(!g_sys_state.system_quit){

		sem_wait(&m_sem_tcp_file);//等待tcp文件传输信号

	//	sem_post(&this->m_sem_tcp_send_test);  //自发自收测试

		if(g_sys_state.eth0_running && g_sys_state.hmi_comm_ready){
			if(m_soc_file_send < 0){
				printf("waitting for file trans connect\n");
				m_soc_file_send = accept(m_soc_tcp_file, (struct sockaddr *)&addr_hmi, &len);
				if(m_soc_file_send < 0){//连接出错
					//TODO 错误处理
				//	printf("ERROR! Failed to accept file trans link!err=%d\n", errno);
					this->m_b_trans_file = false;   //复位文件传输标志
					g_ptr_trace->PrintLog(LOG_ALARM, "接受文件传输连接失败！errno = %d", errno);
                    CreateError(ERR_FILE_TRANS, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno);
					continue;
				}
				else
					printf("tcp file socket: %d, %s\n", m_soc_file_send, inet_ntoa(addr_hmi.sin_addr));
			}

			//TODO 传输文件
			if(this->m_b_recv_file){
				res = this->RecvFile();
			}
			else{
				res = this->SendFile();
			}
			if(res != ERR_NONE){
				g_ptr_trace->PrintLog(LOG_ALARM, "文件传输异常中断！errno = %d", errno);
                CreateError(res, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno);
			}

		}
		this->m_b_trans_file = false;   //复位文件传输标志
	}

	return res;
}

//void HMICommunication::SendMonitorData(int sig){

//	static uint64_t monitor_count = 0;
//	static time_t t_last = time(nullptr);
//
//
//	if(++monitor_count%20 == 0)
//	{
//		time_t t1 = time(nullptr);
//		if(t1 - t_last != 1)
//			printf("monitor time %d,%d,%lld\n", (int)t1, (int)t_last,monitor_count );
//		t_last = t1;
//		printf("send monitor data id = %ld\n", syscall(SYS_gettid));
//	}

//
//	if(!g_sys_state.hmi_comm_ready){  //HMI未连接
//		g_sys_state.eth0_running = CheckNetState("eth0");
//	//	printf("monitor check net state:%d\n", g_sys_state.eth0_running);
//		return;
//	}
//
//	if(m_soc_monitor_send == -1){
//		struct sockaddr_in addr_hmi;
//		socklen_t len = sizeof(struct sockaddr);
//		printf("waitting for monitor connect\n");
//		m_soc_monitor_send = accept(m_soc_tcp_monitor, (struct sockaddr *)&addr_hmi, &len);
//		if(m_soc_monitor_send < 0){//连接出错
//			//TODO 错误处理
//			printf("ERROR! Failed to accept monitor link!err=%d\n", errno);
//			return;
//		}else
//			printf("accept monitor connect\n");
//	}
//
//	if(g_sys_state.hmi_comm_ready){
//		//处理心跳计数
//		if(++m_n_hmi_heartbeat >= kHeartbeatTimeout){//心跳掉线，1秒
//			g_sys_state.hmi_comm_ready = false;
//
//			close(m_soc_monitor_send);
//			m_soc_monitor_send = -1;  //关闭连接
//			printf("close monitor socket\n");
//
//
//			g_ptr_trace->PrintLog(LOG_ALARM, "HMI心跳丢失！");
//
//			CreateError(ERR_HEARTBEAT_HMI_LOST, ERROR_LEVEL, CLEAR_BY_CLEAR_BUTTON);  //生成HMI心跳丢失告警
//			return;
//		}
//
//
//		//TODO 发送监控数据
//		bool bAxisSend = (monitor_count%6==0?true:false);
//		g_ptr_chn_engine->SendMonitorData(m_soc_monitor_send, bAxisSend);   //发送通道数据
//
//	}

//}

/**
 * @brief 监控数据传输函数
 * @return
 */
static int count1 = 0;
int HMICommunication::Monitor(){
	int res = ERR_NONE;

    static uint64_t monitor_count = 0;
//   	static time_t t_last = time(nullptr);


   	printf("thread id = %ld\n", syscall(SYS_gettid));

   	if(listen(m_soc_tcp_monitor, 5)!= 0){
		//TODO 错误处理
		printf("ERROR! Failed to listen monitor port!err=%d\n", errno);
		res = ERR_SC_INIT;
		pthread_exit((void*) EXIT_FAILURE);
	}

   	//测试刷新时间间隔
//	struct timeval tvLast;
//	struct timeval tvNow;
//	bool first = true;
//	int nTimeInterval;

   	while(!g_sys_state.system_quit){

   		monitor_count++;
//	    if(++monitor_count%20 == 0)
//		{
//			time_t t1 = time(nullptr);
//			if(t1 - t_last != 1)
//				printf("monitor time %d,%d,%lld\n", (int)t1, (int)t_last,monitor_count );
//			t_last = t1;
//		}

	//    printf("monitor process:%lld\n", monitor_count);

	   if(!g_sys_state.hmi_comm_ready){  //HMI未连接
	   		g_sys_state.eth0_running = CheckNetState("eth0");
	   	//	printf("monitor check net state:%d\n", g_sys_state.eth0_running);
	   		usleep(50000);
	   		continue;
	   	}


	   	if(g_sys_state.hmi_comm_ready){
	   		//处理心跳计数
	//   		printf("m_n_hmi_heartbeat : %u\n", m_n_hmi_heartbeat);
	   		if(++m_n_hmi_heartbeat > kHeartbeatTimeout){//心跳掉线，1秒
	   			gettimeofday(&tv_hmi_heart2, NULL);
	   			int nTimeDelay = (tv_hmi_heart2.tv_sec-tv_hmi_heart1.tv_sec)*1000000+tv_hmi_heart2.tv_usec-tv_hmi_heart1.tv_usec;
	   			int nTimeDelay2 = (tv_hmi_heart2.tv_sec-tv_hmi_cmd1.tv_sec)*1000000+tv_hmi_heart2.tv_usec-tv_hmi_cmd1.tv_usec;

	   			g_ptr_trace->PrintLog(LOG_ALARM, "HMI心跳丢失！%d, %d, %d", m_n_hmi_heartbeat, nTimeDelay, nTimeDelay2);

	   			this->DisconnectToHmi();  //关闭HMI连接
	   			m_n_hmi_heartbeat = 0;
	   			continue;
	   		}

		   	if(m_soc_monitor_send == -1){
		   		struct sockaddr_in addr_hmi;
		   		socklen_t len = sizeof(struct sockaddr);
		   		printf("waitting for monitor connect\n");
		   		m_soc_monitor_send = accept(m_soc_tcp_monitor, (struct sockaddr *)&addr_hmi, &len);
		   		if(m_soc_monitor_send < 0){//连接出错
		   			//TODO 错误处理
		   			printf("ERROR! Failed to accept monitor link!err=%d\n", errno);
		   			g_ptr_trace->PrintLog(LOG_ALARM, "HMI监控连接中断！errno=%d", errno);
		   			this->DisconnectToHmi();  //关闭HMI连接
		   		//	CreateError(ERR_HMI_MONITOR_DISCON, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON);  //生成HMI监控断链告警
		   		}else{
		   			printf("accept monitor connect\n");
		   		}
		   	}else{
		   		//TODO 发送监控数据
				bool bAxisSend = (monitor_count%6==0?true:false);
				g_ptr_chn_engine->SendMonitorData(bAxisSend, true);   //发送通道数据
		   	}

	   		//测试发送监控数据间隔
//	   		gettimeofday(&tvNow, NULL);
//	   		if(first){
//	   			first = false;
//	   			tvLast = tvNow;
//	   		}else{
//	   			nTimeInterval = (tvNow.tv_sec-tvLast.tv_sec)*1000000+tvNow.tv_usec-tvLast.tv_usec;
//	   			if(nTimeInterval > 50100)
//	   				printf("$$$$$$$$$monitor data send time out: %d\n", nTimeInterval);
//	   			tvLast = tvNow;
//	   		}



	   	}

	   	// @test zk
	   	count1 ++;

	   	if(count1 >= 100){
			count1 = 0;
	   		//获取内存
			//(MemTotal - MemFree)/ MemTotal
			get_memoccupy((MEM_OCCUPY *)&mem_stat);
			/*printf("[MemTotal] = %lu\n "
					"[MemFree] = %lu\n "
					"[Buffers] = %lu\n "
					"[Cached] = %lu\n "
					"[SwapCached] = %lu\n ",
					mem_stat.MemTotal, mem_stat.MemFree, mem_stat.Buffers, mem_stat.Cached, mem_stat.SwapCached);*/

			 //printf("%.3f\n", mem_stat.MemFree * 1.0 / ( mem_stat.MemTotal * 1.0  ) );
			 //第一次获取cpu使用情况
			 CPU_OCCUPY cpu_cur;
			 get_cpuoccupy((CPU_OCCUPY *)&cpu_cur);

			 //计算cpu使用率
			 cal_cpuoccupy((CPU_OCCUPY *)&cpu_stat, (CPU_OCCUPY *)&cpu_cur);
			 cpu_stat = cpu_cur;
	   	}
		 // @test zk
	   usleep(50000);
	 //  	usleep(200000);

	}

	return res;
}

/*
void HMICommunication::SendTestFunc(){

	int tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
	if(tcp_socket == -1){
		perror("create send tcp socket fail\n");
		return;
	}

	char filepath[100] = PATH_NC_FILE;
	int fd = -1;
	uint64_t filesize = 0;

	uint64_t sendsize = 0;

	char buf[1024];

	struct sockaddr_in dest_addr = {0};
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons(PORT_TCP_FILE);
	dest_addr.sin_addr.s_addr = inet_addr("192.168.1.100");

	int ret = -1;
	while(!g_sys_state.system_quit){
		sem_wait(&this->m_sem_tcp_send_test);

		ret = connect(tcp_socket, (struct sockaddr *)&dest_addr, sizeof(sockaddr));
		if(ret != 0){
			perror("connect error\n");
			return;
		}
		printf("send test connect to host!\n");

		fd = open(filepath, O_RDWR);
		if(fd == -1){
			perror("open file failed\n");
			return;
		}

		filesize = lseek(fd, 0, SEEK_END);

		lseek(fd, 0, SEEK_SET);

		send(tcp_socket, (char *)&filesize, sizeof(filesize), 0);

		sendsize = 0;
		while(sendsize < filesize){
			bzero(buf, 1024);
			ret = read(fd, buf, 1024);

			send(tcp_socket, buf, ret, 0);

			sendsize += ret;
		}

		close(fd);

		close(tcp_socket);


	}
}
*/
/**
 * @brief UDP命令重发线程函数
 * @param void *args: HMICommunication对象指针
 */
void *HMICommunication::UdpResendCmdThread(void *args){
	printf("Start HMICommunication::UdpResendCmdThread! id = %ld\n", syscall(SYS_gettid));
	HMICommunication *hmi_comm = static_cast<HMICommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::UdpResendCmdThread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::UdpResendCmdThread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	m_thread_run_flag |= THREAD_CMD_RESEND;

	while(!g_sys_state.hmi_comm_ready)  //等待连接就绪
		usleep(50000);  //等待50ms

	res = hmi_comm->ResendHmiCmd();

	m_thread_run_flag &= (~THREAD_CMD_RESEND);

	printf("Quit HMICommunication::UdpResendCmdThread!\n");
	pthread_exit(NULL);
}

/**
 * @brief UDP接收命令处理线程函数
 * @param void *args: HMICommunication对象指针
 */
void *HMICommunication::UdpProcessCmdThread(void *args){
	printf("Start HMICommunication::UdpProcessCmdThread! id = %ld\n", syscall(SYS_gettid));
	HMICommunication *hmi_comm = static_cast<HMICommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::UdpProcessCmdThread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::UdpProcessCmdThread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	m_thread_run_flag |= THREAD_CMD_PROCESS;

	while((g_sys_state.module_ready_mask & SC_READY) == 0) //等待SC模块初始化完成
		usleep(50000);  //等待50ms

	res = hmi_comm->ProcessHmiCmd();

	m_thread_run_flag &= (~THREAD_CMD_PROCESS);
	printf("Quit HMICommunication::UdpProcessCmdThread!\n");
	pthread_exit(NULL);
}


/**
 * @brief TCP监控数据发送线程
 * @param void *args: HMICommunication对象指针
 */
void *HMICommunication::TcpMonitorThread(void *args){
	printf("Start HMICommunication::TcpMonitorThread!thread id = %ld\n", syscall(SYS_gettid));

	HMICommunication *hmi_comm = static_cast<HMICommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::TcpMonitorThread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::TcpMonitorThread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	m_thread_run_flag |= THREAD_MONITOR;

	g_ptr_trace->PrintTrace(TRACE_INFO, HMI_COMMUNICATION, "等待网络连接。。。。。。");
	while(!g_sys_state.eth0_running){
		usleep(200000);   //等待200ms
		g_sys_state.eth0_running = CheckNetState("eth0");
	}

	g_ptr_trace->PrintTrace(TRACE_INFO, HMI_COMMUNICATION, "等待SC模块准备好。。。。。。");

	while((g_sys_state.module_ready_mask & SC_READY) == 0){  //等待SC模块初始化完成
		usleep(100000);
	}

	g_ptr_trace->PrintTrace(TRACE_INFO, HMI_COMMUNICATION, "初始化监控TCP连接。。。。。。");
//	signal(SIGALRM, SendMonitorData);
//
//
//	struct itimerval tick;
//	tick.it_value.tv_sec = 0;
//	tick.it_value.tv_usec = 50000;
//	tick.it_interval.tv_sec = 0;
//	tick.it_interval.tv_usec = 50000;
//
//	res = setitimer(ITIMER_REAL, &tick, nullptr);
//	if(res != 0){
//		printf("Failed to set montior timer!\n");
//		res = ERR_SC_INIT;
//		pthread_exit((void*) EXIT_FAILURE);
//	}


//	this->SendTestFunc();


//   while(!g_sys_state.system_quit){
//		pause();
//		printf("monitor pause\n");
//	}


	res = hmi_comm->Monitor();

	m_thread_run_flag &= (~THREAD_MONITOR);
	printf("Quit HMICommunication::TcpMonitorThread!\n");
	pthread_exit(NULL);
}

/**
 * @brief TCP文件传输线程，包括参数文件、加工文件，升级文件等
 * @param void *args: HMICommunication对象指针
 */
void *HMICommunication::TcpTransFileThread(void *args){
	printf("Start HMICommunication::TcpTransFileThread! id = %ld\n", syscall(SYS_gettid));
	HMICommunication *hmi_comm = static_cast<HMICommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::TcpTransFileThread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::TcpTransFileThread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	m_thread_run_flag |= THREAD_FILE_TRANS;

	while((g_sys_state.module_ready_mask & SC_READY) == 0) //等待SC模块初始化完成
		usleep(50000);  //等待50ms

	res = hmi_comm->TransFile();

	m_thread_run_flag &= (~THREAD_FILE_TRANS);
	printf("Quit HMICommunication::TcpTransFileThread!\n");
	pthread_exit(NULL);
}

/**
 * @brief 文件另存为线程
 * @param void *args: HMICommunication对象指针
 */
void *HMICommunication::SaveAsFileThread(void *args){
	printf("Start HMICommunication::SaveAsFileThread!id = %ld\n", syscall(SYS_gettid));
	SaveAsParam *pp = static_cast<SaveAsParam *>(args);
	HMICommunication *hmi_comm = pp->pComm;

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::SaveAsFileThread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::SaveAsFileThread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	m_thread_run_flag |= (THREAD_FILE_SAVEAS);

	res = hmi_comm->SaveasFileFunc(pp->path_old, pp->path_new);

	if(res != ERR_NONE){
		printf("save as res = %d\n", res);
        CreateError(res, WARNING_LEVEL, CLEAR_BY_MCP_RESET, errno);
	}


	m_thread_run_flag &= (~THREAD_FILE_SAVEAS);
	printf("Quit HMICommunication::SaveAsFileThread!\n");
    pthread_exit(NULL);
}

/**
 * @brief 用于处理耗时命令
 * @param void *args: HMICommunication对象指针
 */
void *HMICommunication::BackgroundThread(void *args)
{
    printf("Start HMICommunication::BackgroundThread!id = %ld\n", syscall(SYS_gettid));
    HMICommunication *hmi_comm = static_cast<HMICommunication *>(args);

    int res = ERR_NONE;
    res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
    if (res != ERR_NONE) {
        printf("Quit HMICommunication::BackgroundThread with error 1!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }
    res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
    if (res != ERR_NONE) {
        printf("Quit HMICommunication::SaveAsFileThread with error 2!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }

    m_thread_run_flag |= (THREAD_BACKGROUND);

    hmi_comm->BackgroundFunc();

    m_thread_run_flag &= (~THREAD_BACKGROUND);
    printf("Quit HMICommunication::BackgroundThread!\n");
    pthread_exit(NULL);
}

/**
 * @brief nc文件另存为
 * @return
 */
int HMICommunication::SaveasFileFunc(const char *path_old, const char *path_new){
	int ret = ERR_NONE;
	char buf[10240];
	uint64_t file_size = 0;    //文件大小
	uint64_t remains = 0;  //剩余字节数
	int read_num = 10240;  //每次读取字节数
	mode_t file_mode;    //文件属性

	printf("Start HMICommunication::SaveasNcFileFunc!thread id = %ld\n", syscall(SYS_gettid));


	//打开文件
	int fd = open(path_old, O_RDONLY);
	int fd_new = open(path_new, O_WRONLY|O_CREAT|O_TRUNC);
	if(fd == -1 || fd_new == -1){
		g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "打开或创建文件失败！");
		ret = ERR_SAVEAS_FILE;
		goto END;
	}

	//获取文件大小
	if(!GetNcFileInfo(path_old, file_size, nullptr, &file_mode)){
		g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "获取文件[%s]大小失败！", path_old);
		ret = ERR_SAVEAS_FILE;
		goto END;
	}
	remains = file_size;


	while(remains > 0 && !g_sys_state.system_quit){
		memset(buf, 0x00, 10240);
		if((uint64_t)read_num > remains)
			read_num = remains;
		ret = read(fd, buf, read_num);
		if(ret != read_num){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "读取文件出错！应读取%d字节，实际读取%d字节。", read_num, ret);
			ret = ERR_SAVEAS_FILE;
			goto END;
		}
		ret = write(fd_new, buf, read_num);
		if(ret != read_num){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "写入文件出错！应写入%d字节，实际写入%d字节。", read_num, ret);
			ret = ERR_SAVEAS_FILE;
			goto END;
		}
		remains -= read_num;
	}

	END:
	close(fd);
	close(fd_new);
	chmod(path_new, file_mode);   //修改新文件操作属性同原文件
	this->m_thread_saveas = 0;
	ret = ERR_NONE;

	sync();

	return ret;

}

/**
 * @brief 耗时命令后台处理
 * @return
 */
void HMICommunication::BackgroundFunc()
{
    while(!g_sys_state.system_quit){

        sem_wait(&m_sem_background);//等待tcp文件传输信号
        std::cout << "HMICommunication::BackgroundFunc()" << std::endl;
        if(g_sys_state.eth0_running && g_sys_state.hmi_comm_ready){
            switch (m_background_type) {
            case Background_None:
                std::cout << "Warning: HMICommunication::BackgroundFunc Background_None" << std::endl;
                break;
            case Background_Pack:
                PackageSysBackupFile();
                break;
            case Background_UnPack:
                UnPackageBackupFile();
                break;
            }
            m_background_type = Background_None;
        }
    }
}


/**
 * @brief 处理HMI发出的设备扫描指令
 * @param cmd_node : 命令数据
 * @return
 */
void HMICommunication::ProcessHmiDeviceScan(HMICmdRecvNode &cmd_node){
	int res = ERR_NONE;
	HMICmdFrame resp;
	memset(resp.data, 0x00, kMaxHmiDataLen);


	if(cmd_node.cmd.cmd_extension == 0)  //传统tcp/udp协议
		cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD);  //更改端口
	else
		cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //更改端口
//#ifdef USES_MODBUS_PROTOCAL
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //更改端口
//#else
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD);  //更改端口
//#endif

	//发送回复
	resp.frame_number = (cmd_node.cmd.frame_number | 0x8000);  //回复帧号
	resp.channel_index = cmd_node.cmd.channel_index;
	resp.cmd = cmd_node.cmd.cmd;


	if(!g_sys_state.hmi_comm_ready){
		resp.cmd_extension = APPROVE;

	}else{
		resp.cmd_extension = REFUSE;   //设备忙
	}
	strcpy(resp.data, g_sys_state.local_host_addr);
	resp.data_len = strlen(resp.data);


	//发送回复包
	int len = kHmiCmdHeadLen+resp.data_len;
	res = sendto(this->m_soc_udp_send, &resp, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
	if(len != res){
		//printf("ERROR! Failed to send scan response cmd to hmi!%d, %d\n", len,res);
		g_ptr_trace->PrintLog(LOG_ALARM, "回复设备扫描指令失败[host:%s]！", inet_ntoa(cmd_node.ip_addr.sin_addr));
		return;
	}

	//printf("receive hmi device scan cmd, host[%s]!\n", inet_ntoa(cmd_node.ip_addr.sin_addr));
	g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "收到来自[%s]的设备扫描指令！", inet_ntoa(cmd_node.ip_addr.sin_addr));
}


/**
 * @brief 处理HMI发出的握手指令
 * @param cmd_node: 命令数据
 */
void HMICommunication::ProcessHmiShakehand(HMICmdRecvNode &cmd_node){

	HMICmdFrame resp;
	memset(resp.data, 0x00, kMaxHmiDataLen);

	HMICmdFrame &cmd = cmd_node.cmd;

	if(cmd.data_len == strlen(STR_HMI_SHAKE) &&
			strcmp(cmd.data, STR_HMI_SHAKE) == 0){
		if(!g_sys_state.hmi_comm_ready){
			m_addr_udp_hmi = cmd_node.ip_addr;
			if(cmd.cmd_extension == 0)  //传统tcp/udp协议
				m_addr_udp_hmi.sin_port = htons(PORT_UDP_CMD);  //更改端口
			else
				m_addr_udp_hmi.sin_port = htons(PORT_UDP_CMD_SEND);  //更改端口
//#ifdef USES_MODBUS_PROTOCAL
//			m_addr_udp_hmi.sin_port = htons(PORT_UDP_CMD_SEND);  //更改端口
//#else
//			m_addr_udp_hmi.sin_port = htons(PORT_UDP_CMD);  //更改端口
//#endif

			this->m_n_hmi_heartbeat = 0;   //心跳计数归零



			resp.cmd_extension = APPROVE;
			resp.data_len = strlen(STR_HMI_SHAKE_RESP);
			strcpy(resp.data, STR_HMI_SHAKE_RESP);
			g_sys_state.hmi_comm_ready = true;  //hmi通讯准备好

		}else{
			resp.cmd_extension = REFUSE;   //设备忙
			resp.data_len = 0x00;

		}


		//发送回复
		resp.frame_number = (cmd.frame_number | 0x8000);  //回复帧号
		resp.channel_index = cmd.channel_index;
		resp.cmd = cmd.cmd;

		this->SendCmd(resp);

		if(resp.cmd_extension == APPROVE){
			strcpy(g_sys_state.hmi_host_addr, inet_ntoa(m_addr_udp_hmi.sin_addr));
			g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "建立与HMI[%s:%hu]的连接！", g_sys_state.hmi_host_addr, ntohs(m_addr_udp_hmi.sin_port));

#ifdef USES_LICENSE_FUNC
		m_p_channel_engine->CheckLicense(true);
#endif

			g_ptr_alarm_processor->SendToHmi();  //将现有错误发送给HMI
		}
		else
			g_ptr_trace->PrintLog(LOG_ALARM, "系统已连接HMI[%s]，拒绝来自[%s]的HMI 连接, %d！",
					g_sys_state.hmi_host_addr, inet_ntoa(cmd_node.ip_addr.sin_addr), this->m_n_hmi_heartbeat);
	}

}

/**
 * @brief 处理HMI获取文件指令
 * @param HMICmdFrame &cmd: 命令数据
 */
void HMICommunication::ProcessHmiGetFileCmd(HMICmdFrame &cmd){

	if(this->m_b_trans_file){
		//当前已经处于文件传输中，拒绝
		cmd.data[cmd.data_len] = REFUSE;  //拒绝
		printf("In file transferring , refuse to send file, file type = %hu\n", cmd.cmd_extension);
	}
	else{
		if(cmd.cmd_extension == FILE_PMC_LADDER){  //请求梯图文件则先检查梯图文件是否存在
			char path[kMaxPathLen];
			strcpy(path, PATH_PMC_LDR);
			if(access(path, F_OK) == -1){	//不存在，返回失败
				cmd.data[cmd.data_len] = REFUSE;  //拒绝
				printf("ladder file do not exist, refuse to send file\n");
				goto END;
			}
		}else if(cmd.cmd_extension == FILE_WARN_HISTORY){  //请求告警历史文件则先检查文件是否存在
			if(!TraceInfo::IsAlarmFileExist(0x10)){	//不存在，返回失败
				cmd.data[cmd.data_len] = NOTEXIST;  //拒绝
				printf("alarm file do not exist, refuse to send file\n");
				goto END;
			}
		}else if(cmd.cmd_extension == FILE_ESB_DEV){ //请求伺服描述文件，检查文件是否存在
			char path[kMaxPathLen];
			strcpy(path, PATH_ESB_FILE);
			strcat(path, cmd.data);
			if(access(path, F_OK) == -1){	//不存在，返回失败
				cmd.data[cmd.data_len] = REFUSE;  //拒绝
				printf("esb file[%s] do not exist, refuse to send file\n", cmd.data);
				goto END;
			}
		}
		//接受请求
		this->m_b_trans_file = true;
		this->m_b_recv_file = false;

		cmd.data[cmd.data_len] = APPROVE;  //接受

		m_file_type = (FileTransType)cmd.cmd_extension;
		if(m_file_type == FILE_G_CODE){
			memset(m_str_file_name, 0x00, kMaxFileNameLen);
			strcpy(m_str_file_name, cmd.data);

			printf("HMI get file: %s\n", m_str_file_name);
//			for(int i=0; i < cmd.data_len; i++)
//				printf("[%d]= 0x%x ", i, cmd.data[i]);
//			printf("\n");
		}else if(m_file_type == FILE_CONFIG_PACK){  //配置文件打包文件
			memcpy(&this->m_mask_config_pack, cmd.data, cmd.data_len);   //参数mask
            printf("pack config: mask = 0x%x\n", m_mask_config_pack);
		}else if(m_file_type == FILE_WARN_HISTORY){  //告警历史文件
//			this->m_n_alarm_file_type = cmd.data[0];
//			printf("HMI get alarm file, type = %hhu\n", m_n_alarm_file_type);
		}else if(cmd.cmd_extension == FILE_ESB_DEV){ //请求伺服描述文件
			memset(m_str_file_name, 0x00, kMaxFileNameLen);
			strcpy(m_str_file_name, cmd.data);
        }

		sem_post(&m_sem_tcp_file);  //发送信号，激活文件传输线程

	}

	END:
	cmd.frame_number |= 0x8000;
	cmd.data_len++;
	this->SendCmd(cmd);  //发送响应包

}

//#ifdef USES_MODBUS_PROTOCAL
///**
// * @brief 处理HMI获取文件指令
// * @param cmd_node : 指令节点，包含发送方地址信息
// */
//void HMICommunication::ProcessHmiGetFileCmd(HMICmdRecvNode &cmd_node){
//	HMICmdFrame &cmd = cmd_node.cmd;
//	if(this->m_b_trans_file){
//		//当前已经处于文件传输中，拒绝
//		cmd.data[cmd.data_len] = REFUSE;  //拒绝
//		printf("In file transferring , refuse to send file, file type = %hu\n", cmd.cmd_extension);
//	}
//	else{
//		if(cmd.cmd_extension == FILE_PMC_LADDER){  //请求梯图文件则先检查梯图文件是否存在
//			char path[kMaxPathLen];
//			strcpy(path, PATH_PMC_LDR);
//			if(access(path, F_OK) == -1){	//不存在，返回失败
//				cmd.data[cmd.data_len] = REFUSE;  //拒绝
//				printf("ladder file do not exist, refuse to send file\n");
//				goto END;
//			}
//		}
//		//接受请求
//		this->m_b_trans_file = true;
//		this->m_b_recv_file = false;
//
//		cmd.data[cmd.data_len] = APPROVE;  //接受
//
//		m_file_type = (FileTransType)cmd.cmd_extension;
//		if(m_file_type == FILE_G_CODE){
//			memset(m_str_file_name, 0x00, kMaxFileNameLen);
//			strcpy(m_str_file_name, cmd.data);
//
//			printf("HMI get file2: %s\n", m_str_file_name);
////			for(int i=0; i < cmd.data_len; i++)
////				printf("[%d]= 0x%x ", i, cmd.data[i]);
////			printf("\n");
//		}else if(m_file_type == FILE_CONFIG_PACK){  //配置文件打包文件
//			memcpy(&this->m_mask_config_pack, cmd.data, cmd.data_len);   //参数mask
//			printf("pack config: mask = 0x%x\n", m_mask_config_pack);
//
//		}
//		sem_post(&m_sem_tcp_file);  //发送信号，激活文件传输线程
//
//	}
//
//	END:
//	cmd.frame_number |= 0x8000;
//	cmd.data_len++;
//
//
//	//发送回复包
//	int len = kHmiCmdHeadLen+cmd.data_len;
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //更改端口
//	m_addr_file_trans = cmd_node.ip_addr;   //记录文件传输目的IP
//	int res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
//	if(len != res){
//		//printf("ERROR! Failed to send scan response cmd to hmi!%d, %d\n", len,res);
//		g_ptr_trace->PrintLog(LOG_ALARM, "回复设备文件获取指令失败[host:%s]！", inet_ntoa(cmd_node.ip_addr.sin_addr));
//		return;
//	}else{
//		printf("send resp to hmi get file cmd:ip[%s:%hu]\n", inet_ntoa(cmd_node.ip_addr.sin_addr), ntohs(cmd_node.ip_addr.sin_port));
//	}
//}
//#endif


/**
 * @brief 处理HMI发送文件指令
 * @param HMICmdFrame &cmd: 命令数据
 */
void HMICommunication::ProcessHmiSendFileCmd(HMICmdFrame &cmd){

	uint64_t free = GetFreeDisk() - g_ptr_parm_manager->GetSystemConfig()->free_space_limit*1024;
	uint64_t filesize = 0;

	m_file_type = (FileTransType)cmd.cmd_extension;
	if(m_file_type == FILE_G_CODE || m_file_type == FILE_ESB_DEV){
		memset(m_str_file_name, 0x00, kMaxFileNameLen);
		strcpy(m_str_file_name, cmd.data);
		if(cmd.data_len == strlen(m_str_file_name)+9){  //有文件大小
			memcpy(&filesize, &cmd.data[cmd.data_len-8], 8);
			printf("has file size : %llu\n", filesize);
		}else{
			printf("no file size: data_len=%d, filename=%d\n", cmd.data_len, strlen(m_str_file_name));
		}
	}
	if(this->m_b_trans_file|| filesize > free){
		//当前已经处于文件传输中，拒绝
		cmd.frame_number |= 0x8000;

		memcpy(&cmd.data[cmd.data_len], &free, 8);
		cmd.data_len += 8;

		cmd.data[cmd.data_len] = REFUSE;  //拒绝
		printf("refuse to recv file, flag = %hhu, filesize=%llu, free=%llu\n", m_b_trans_file, filesize, free);
	}
	else{
		//接受请求
		this->m_b_trans_file = true;
		this->m_b_recv_file = true;

		sem_post(&m_sem_tcp_file);  //发送信号，激活文件传输线程

//		m_file_type = (FileTransType)cmd.cmd_extension;
		if(m_file_type == FILE_G_CODE){

			printf("hmi send nc file: %s\n", m_str_file_name);
//			for(int i=0; i < cmd.data_len; i++)
//				printf("[%d]= 0x%x ", i, cmd.data[i]);
//			printf("\n");
			this->m_p_channel_engine->RefreshFile(m_str_file_name);
		}else{
			printf("hmi send file[type=%hu]: %s\n", cmd.cmd_extension, m_str_file_name);
		}

		memcpy(&cmd.data[cmd.data_len], &free, 8);
		cmd.data_len += 8;

		cmd.frame_number |= 0x8000;
		cmd.data[cmd.data_len] = APPROVE;  //接受

	}
	cmd.data_len++;

	this->SendCmd(cmd);  //发送响应包
}

//#ifdef USES_MODBUS_PROTOCAL
///**
// * @brief 处理HMI发送文件指令
// * @param HMICmdRecvNode &cmd_node: 指令节点，包含发送方地址信息
// */
//void HMICommunication::ProcessHmiSendFileCmd(HMICmdRecvNode &cmd_node){
//	HMICmdFrame &cmd = cmd_node.cmd;
//	if(this->m_b_trans_file){
//		//当前已经处于文件传输中，拒绝
//		cmd.frame_number |= 0x8000;
//	//	cmd.cmd_extension = REFUSE;  //拒绝
//		cmd.data[cmd.data_len] = REFUSE;  //拒绝
//		printf("refuse to recv file\n");
//	}
//	else{
//		//接受请求
//		this->m_b_trans_file = true;
//		this->m_b_recv_file = true;
//
//		sem_post(&m_sem_tcp_file);  //发送信号，激活文件传输线程
//
//
//		m_file_type = (FileTransType)cmd.cmd_extension;
//		if(m_file_type == FILE_G_CODE){
//			memset(m_str_file_name, 0x00, kMaxFileNameLen);
//			strcpy(m_str_file_name, cmd.data);
//			printf("hmi send file2: %s\n", m_str_file_name);
////			for(int i=0; i < cmd.data_len; i++)
////				printf("[%d]= 0x%x ", i, cmd.data[i]);
////			printf("\n");
//			this->m_p_channel_engine->RefreshFile(m_str_file_name);
//		}
//
//
//
//		cmd.frame_number |= 0x8000;
//		cmd.data[cmd.data_len] = APPROVE;  //接受
//
//
//	}
//	cmd.data_len++;
//
//	//发送回复包
//	int len = kHmiCmdHeadLen+cmd.data_len;
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //更改端口
//	m_addr_file_trans = cmd_node.ip_addr;   //记录文件传输目的IP
//	int res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
//	if(len != res){
//		//printf("ERROR! Failed to send scan response cmd to hmi!%d, %d\n", len,res);
//		g_ptr_trace->PrintLog(LOG_ALARM, "回复设备文件发送指令失败[host:%s]！", inet_ntoa(cmd_node.ip_addr.sin_addr));
//		return;
//	}else{
//		printf("send resp to hmi send file cmd:ip[%s:%hu]\n", inet_ntoa(cmd_node.ip_addr.sin_addr), ntohs(cmd_node.ip_addr.sin_port));
//	}
//}
//#endif

/**
 * @brief 处理HMI中断连接指令
 * @param cmd : 命令数据
 * @return
 */
void HMICommunication::ProcessHmiDisconnectCmd(HMICmdFrame &cmd){

	if(g_sys_state.hmi_comm_ready)
		this->DisconnectToHmi();   //关闭HMI连接

	//发送响应包
	cmd.frame_number |= 0x8000;
	this->SendCmd(cmd);
}

/**
 * @brief 处理HMI设置当前通道命令
 * @param cmd : 命令数据
 */
void HMICommunication::ProcessHmiSetCurChnCmd(HMICmdFrame &cmd){
	bool res = this->m_p_channel_engine->SetCurWorkChanl(cmd.channel_index);

	//发送响应包
	cmd.frame_number |= 0x8000;
	cmd.cmd_extension = res?SUCCEED:FAILED;
	this->SendCmd(cmd);
}

/**
 * @brief 处理HMI就绪命令
 * @param cmd : 命令数据
 */
void HMICommunication::ProcessHmiReadyCmd(HMICmdFrame &cmd){
//	g_sys_state.hmi_ready = true;
	g_sys_state.module_ready_mask |= HMI_READY;

	if((g_sys_state.module_ready_mask & SC_READY) == 0)
		cmd.cmd_extension = 1;  // 未就绪

	//发送响应包
	cmd.frame_number |= 0x8000;
	this->SendCmd(cmd);

	if((g_sys_state.module_ready_mask & NC_READY) == NC_READY)
		g_sys_state.system_ready = true;
}

/**
 * @brief 处理HMI告警指令
 * @param cmd : 命令数据
 */
void HMICommunication::ProcessHmiNewAlarmCmd(HMICmdFrame &cmd){

	if(cmd.data_len == 4)
		memcpy((char *)&g_sys_state.hmi_alarm_code, cmd.data, 4);
//	g_sys_state.hmi_ready = false;
	g_sys_state.module_ready_mask &= ~(HMI_READY);

	//发送响应包
	cmd.frame_number |= 0x8000;
	cmd.data_len = 0;
	this->SendCmd(cmd);
}

/**
 * @brief 处理HMI获取版本信息指令
 * @param cmd : 命令数据
 */
void HMICommunication::ProcessHmiGetVersionCmd(HMICmdFrame &cmd){

	if(cmd.cmd_extension == SOFTWARE_VER){
		cmd.data_len = sizeof(SwVerInfoCollect);
		memcpy(cmd.data, (char *)&g_sys_info.sw_version_info, cmd.data_len);
	}
	else if(cmd.cmd_extension == HARDWARE_VER){
		//TODO 增加硬件版本处理
		cmd.data_len = 0;
	}

	//发送响应包
	cmd.frame_number |= 0x8000;

	this->SendCmd(cmd);
}

/**
 * @brief 处理HMI获取通道数命令
 * @param cmd : 命令数据
 * @return
 */
void HMICommunication::ProcessHmiChnCountCmd(HMICmdFrame &cmd){

	//发送响应包
	cmd.frame_number |= 0x8000;
	if(g_sys_state.system_boot_stage > STEP_INIT_PARM_MANAGER)
		cmd.cmd_extension = g_ptr_parm_manager->GetSystemConfig()->chn_count;
	else
		cmd.cmd_extension = 0;  //初始化未完成

	this->SendCmd(cmd);
}

void HMICommunication::ProcessHmiGetChnStateCmd(HMICmdFrame &cmd){
	HmiChannelStatus status;
	if(g_ptr_chn_engine == nullptr){
		cmd.cmd_extension = FAILED;
	}
	else if(g_ptr_chn_engine->GetChnStatus(cmd.channel_index, status)){
		cmd.cmd_extension = SUCCEED;
	}
	else
		cmd.cmd_extension = FAILED;

	//发送响应包
	cmd.frame_number |= 0x8000;
	cmd.data_len = (uint8_t)sizeof(HmiChannelStatus);
	memcpy(cmd.data, (char *)&status, cmd.data_len);
	this->SendCmd(cmd);
}

/**
 * @brief 处理HMI获取NC文件个数指令
 * @param cmd : 命令数据
 * @return
 */
void HMICommunication::ProcessHmiNcFileCountCmd(HMICmdRecvNode &cmd_node){
	HMICmdFrame &cmd = cmd_node.cmd;
	char path[kMaxPathLen] = {0};
	strcpy(path, PATH_NC_FILE);
	if(cmd.data_len > 0){
		strcat(path, cmd.data);
		memcpy(cmd.data+4, cmd.data, cmd.data_len);
	}

	//发送响应包
	cmd.frame_number |= 0x8000;
	cmd.data_len += 4;
	uint32_t count = this->GetNcFileCount(path);
	memcpy(cmd.data, (char *)&count, sizeof(count));

//#ifdef USES_MODBUS_PROTOCAL
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //更改端口
//	int len = kHmiCmdHeadLen+cmd.data_len;
//	int res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
//	if(len != res){
//		//printf("ERROR! Failed to send scan response cmd to hmi!%d, %d\n", len,res);
//		g_ptr_trace->PrintLog(LOG_ALARM, "回复设备获取加工文件数量指令失败[host:%s]！", inet_ntoa(cmd_node.ip_addr.sin_addr));
//		return;
//	}
//#else
	this->SendCmd(cmd);
//#endif
}

/**
 * @brief 处理HMI获取nc文件详细信息命令
 * @param cmd : 命令数据
 */
void HMICommunication::ProcessHmiNcFileInfoCmd(HMICmdRecvNode &cmd_node){
	HMICmdFrame &cmd = cmd_node.cmd;
	char path[kMaxPathLen] = {0};
	strcpy(path, PATH_NC_FILE);
	if(cmd.data_len > 0){
		strcat(path, cmd.data);
		if(path[strlen(path)-1] != '/')
			strcat(path, "/");
	}

	DIR *dir = opendir(PATH_NC_FILE);
	if(dir == nullptr){
	//	printf("Failed to open dir [%s]!\n", PATH_NC_FILE);
		g_ptr_trace->PrintLog(LOG_ALARM, "打开文件目录失败！[%s]", PATH_NC_FILE);
        CreateError(ERR_OPEN_DIR, WARNING_LEVEL, CLEAR_BY_MCP_RESET, errno);
		return;
	}

	uint32_t file_index = 0;
	struct dirent *dir_obj = nullptr;
	char *dir_name = nullptr;
	uint64_t file_size = 0;
	char mtime[20] = {0};
	char file_path[kMaxPathLen] = {0};
//#ifdef USES_MODBUS_PROTOCAL
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //更改端口
//#endif
	//循环读取目录下的文件
	while(nullptr != (dir_obj = readdir(dir))){
		dir_name = dir_obj->d_name;
		if(strcmp(dir_name, ".") == 0 || strcmp(dir_name, "..") == 0){
			continue;
		}
		else if(dir_obj->d_type == DT_REG){   //普通文件

			strcpy(file_path, path);
			strcat(file_path, dir_name);
			if(this->GetNcFileInfo(file_path, file_size, mtime)){
				cmd.cmd_extension = FILE_REG;
				cmd.data_len = 31+strlen(dir_name);
				memcpy(cmd.data+4, (char *)&file_size, 8);
				memcpy(cmd.data+12, mtime, 19);
				strcpy(cmd.data+31, dir_name);
		//		printf("file-%d[%s] info: size = %lld, time[%s]\n", file_index, dir_name, file_size, mtime);
			}
		}
		else if(dir_obj->d_type == DT_DIR){ 	//文件夹
			cmd.cmd_extension = FILE_DIR;
			cmd.data_len = 4+strlen(dir_name);
			strcpy(cmd.data+4, dir_name);
		//	printf("dir-%d[%s]\n", file_index, dir_name);
		}
		//发送响应包
		cmd.frame_number |= 0x8000;
		memcpy(cmd.data, (char *)&file_index, 4);

//#ifdef USES_MODBUS_PROTOCAL
//		int len = kHmiCmdHeadLen+cmd.data_len;
//		int res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
//		if(len != res){
//			//printf("ERROR! Failed to send scan response cmd to hmi!%d, %d\n", len,res);
//			g_ptr_trace->PrintLog(LOG_ALARM, "回复设备获取加工文件信息指令失败[host:%s]！", inet_ntoa(cmd_node.ip_addr.sin_addr));
//			return;
//		}
//#else
		this->SendCmd(cmd);
//#endif
		file_index++;

	}
	closedir(dir);

}


/**
 * @brief 处理HMI获取NC文件签名的命令
 * @param cmd : 命令数据
 */
void HMICommunication::ProcessHmiFileSignatureCmd(HMICmdRecvNode &cmd_node){

	HMICmdFrame &cmd = cmd_node.cmd;
	char file_name[kMaxFileNameLen] = {0};
	char file_path[kMaxPathLen] = {0};
	strcpy(file_name, cmd.data);
//	printf("data_len = %hu, data=%s, file=%s\n", cmd.data_len, cmd.data, file_name);

	char *pSplit = strchr(file_name, '.');
	if(pSplit != nullptr){
		memset(pSplit, 0x00, kMaxFileNameLen-(pSplit-file_name));
	}
	strcat(file_name, "_md5");

	strcpy(file_path, PATH_NC_SIGN_FILE);
	strcat(file_path, file_name);


//	printf("hmi get file md5:%s\n", file_path);

	int fd = open(file_path, O_RDONLY);
	if(fd > 0){//文件打开成功
		read(fd, cmd.data+cmd.data_len+1, kNcFileSignLen);
		cmd.data_len += (kNcFileSignLen+1);
	}else
	{
		printf("Failed to open file[%s] in ProcessHmiFileSignatureCmd\n", file_path);
	}

	//发送响应包
	cmd.frame_number |= 0x8000;

//#ifdef USES_MODBUS_PROTOCAL
//	this->SendCmdToIp(cmd_node);
//#else
	this->SendCmd(cmd);
//#endif
}


/**
 * @brief 处理HMI发来的文件操作命令
 * @param cmd : 命令数据
 */
void HMICommunication::ProcessHmiFileOperateCmd(HMICmdRecvNode &cmd_node){
	HMICmdFrame &cmd = cmd_node.cmd;
	char name_old[kMaxPathLen] = {0};
	char name_new[kMaxPathLen] = {0};
	char *pSplit = nullptr;  //分隔符指针
	bool res = true;
	switch(cmd.cmd_extension){
	case FILE_OPT_DELETE:
		res = this->DeleteNcFile(cmd.data);
//		this->m_p_channel_engine->StartUpdateProcess();
		break;
	case FILE_OPT_RENAME:
		strcpy(name_old, cmd.data);
		pSplit = strchr(name_old, '?');
		if(pSplit == nullptr){
			res = false;
			break;
		}
		*pSplit = '\0';
		strcpy(name_new, ++pSplit);

		res = this->RenameNcFile(name_old, name_new);
		break;
	case FILE_OPT_SAVEAS:
		strcpy(name_old, cmd.data);
		pSplit = strchr(name_old, '?');
		if(pSplit == nullptr){
			res = false;
			break;
		}
		*pSplit = '\0';
		strcpy(name_new, ++pSplit);
		res = this->SaveasNcFile(name_old, name_new);
		break;
	}

	//发送响应包
	cmd.frame_number |= 0x8000;
	cmd.data[cmd.data_len] = res?SUCCEED:FAILED;
	cmd.data_len++;

//#ifdef USES_MODBUS_PROTOCAL
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //更改端口
//	int len = kHmiCmdHeadLen+cmd.data_len;
//	int send_count = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
//	if(len != send_count){
//		//printf("ERROR! Failed to send scan response cmd to hmi!%d, %d\n", len,res);
//		g_ptr_trace->PrintLog(LOG_ALARM, "回复设备文件操作指令失败[host:%s]！", inet_ntoa(cmd_node.ip_addr.sin_addr));
//		return;
//	}
//#else
	this->SendCmd(cmd);
//#endif
}

/**
 * @brief 处理HMI获取ESB文件信息的命令
 * @param cmd
 */
void HMICommunication::ProcessHmiGetEsbInfoCmd(HMICmdFrame &cmd){
	cmd.frame_number |= 0x8000;
	int count = this->GetEsbFileCount(PATH_ESB_FILE);

	if(count == 0){  //没有esb文件
		//发送回复
		cmd.data_len = 9;
		cmd.cmd_extension = 0;
		this->SendCmd(cmd);

		return;
	}

	DIR *dir = opendir(PATH_ESB_FILE);
	if(dir == nullptr){
		//发送回复
		cmd.data_len = 9;
		cmd.cmd_extension = 0;
		this->SendCmd(cmd);

		return;
	}

	uint32_t file_index = 0;
	struct dirent *dir_obj = nullptr;
	char *dir_name = nullptr;
	uint64_t file_size = 0;
	char mtime[20] = {0};
	char file_path[kMaxPathLen] = {0};
	int len = 0;
	char *pt = nullptr;

	//循环读取目录下的文件
	while(nullptr != (dir_obj = readdir(dir))){
		dir_name = dir_obj->d_name;
		if(strcmp(dir_name, ".") == 0 || strcmp(dir_name, "..") == 0){
			continue;
		}
		else if(dir_obj->d_type == DT_REG){   //普通文件
			//校验文件后缀是不是.esb
			len = strlen(dir_name);
			if(len > 4){
				pt = strrchr(dir_name, '.');
				if(pt && strcasecmp(pt, "esb")){  //后缀匹配
					strcpy(file_path, PATH_ESB_FILE);
					strcat(file_path, dir_name);
					if(this->GetNcFileInfo(file_path, file_size, mtime)){
						cmd.cmd_extension = count;
						cmd.data_len = 9+strlen(dir_name);
						cmd.data[0] = file_index;   //顺序号
						memcpy(cmd.data+1, (char *)&file_size, 8);  //文件字节数
						strcpy(cmd.data+9, dir_name);  //文件名称

					}
					file_index++;
				}
			}

		}
		else if(dir_obj->d_type == DT_DIR){ 	//文件夹
			continue;
		}
		//发送响应包
		this->SendCmd(cmd);

	}
	closedir(dir);
}

/**
 * @brief 处理HMI操作ESB文件的命令
 * @param cmd
 */
void HMICommunication::ProcessHmiOperateEsbCmd(HMICmdFrame &cmd){
	char name_old[kMaxPathLen] = {0};
	char name_new[kMaxPathLen] = {0};
	char *pSplit = nullptr;  //分隔符指针
	bool res = true;
	switch(cmd.cmd_extension){
	case FILE_OPT_DELETE:
		res = this->DeleteEsbFile(cmd.data);
		break;
	case FILE_OPT_RENAME:
		strcpy(name_old, cmd.data);
		pSplit = strchr(name_old, '?');
		if(pSplit == nullptr){
			res = false;
			break;
		}
		*pSplit = '\0';
		strcpy(name_new, ++pSplit);

		res = this->RenameEsbFile(name_old, name_new);
		break;
	case FILE_OPT_SAVEAS:
		strcpy(name_old, cmd.data);
		pSplit = strchr(name_old, '?');
		if(pSplit == nullptr){
			res = false;
			break;
		}
		*pSplit = '\0';
		strcpy(name_new, ++pSplit);
		res = this->SaveasEsbFile(name_old, name_new);
		break;
	}

	//发送响应包
	cmd.frame_number |= 0x8000;
	cmd.data[cmd.data_len] = res?SUCCEED:FAILED;
	cmd.data_len++;

	this->SendCmd(cmd);
}

/**
 * @brief 处理HMI同步系统时间命令
 * @param cmd : 命令数据
 */
void HMICommunication::ProcessHmiSyncTimeCmd(HMICmdFrame &cmd){
	//获取当前时间
	time_t timep;
	struct tm *p;
	time(&timep);
	p = gmtime(&timep);
	uint16_t year = p->tm_year;
	cmd.data[0] = ((year>>8)&0xFF);
	cmd.data[1] = (year&0xFF);
	cmd.data[2] = p->tm_mon;
	cmd.data[3] = p->tm_mday;
	cmd.data[4] = p->tm_hour;
	cmd.data[5] = p->tm_min;
	cmd.data[6] = p->tm_sec;

	//发送响应包
	cmd.frame_number |= 0x8000;
	cmd.data_len = 7;

    this->SendCmd(cmd);
}

/**
 * @brief 处理HMI备份命令
 * @param cmd : 命令数据
 */
void HMICommunication::ProcessHmiSysBackupCmd(HMICmdFrame &cmd)
{
    cmd.frame_number |= 0x8000;
    if (m_background_type != Background_None || m_b_trans_file)
    {
        std::cout << "busy backup" << std::endl;
        cmd.cmd_extension = -1;
        SendCmd(cmd);
    }
    else
    {
        std::cout << "begin backup" << std::endl;
        m_sysbackup_status = SysUpdateStatus();
        m_sysbackup_status.m_type = SysUpdateStatus::Backup;
        memcpy(&m_maks_sys_backup, cmd.data, cmd.data_len);

        cmd.data_len = sizeof(SysUpdateStatus);
        cmd.cmd_extension = 0;
        memset(cmd.data, 0x00, kMaxHmiDataLen);
        memcpy(cmd.data, &m_sysbackup_status, cmd.data_len);

        SendCmd(cmd);

        m_background_type = Background_Pack;
        sem_post(&m_sem_background);//通知执行后台程序
    }
}

/**
 * @brief 处理HMI恢复命令
 * @param cmd : 命令数据
 */
void HMICommunication::ProcessHmiSysRecoverCmd(HMICmdFrame &cmd)
{
    cmd.frame_number |= 0x8000;
    if (m_background_type != Background_None || m_b_trans_file)
    {
        std::cout << "busy recover" << std::endl;
        cmd.cmd_extension = -1;
        SendCmd(cmd);
    }
    else
    {
        std::cout << "begin recover" << std::endl;
        m_sysbackup_status = SysUpdateStatus();
        m_sysbackup_status.m_type = SysUpdateStatus::Recover;
        memcpy(&m_maks_sys_backup, cmd.data, cmd.data_len);

        cmd.data_len = sizeof(SysUpdateStatus);
        cmd.cmd_extension = 0;

        memset(cmd.data, 0x00, kMaxHmiDataLen);
        memcpy(cmd.data, &m_sysbackup_status, cmd.data_len);

        SendCmd(cmd);
    }
}

/**
 * @brief 处理HMI虚拟MOP按键命令
 * @param cmd : 命令数据
 */
void HMICommunication::ProcessHmiVirtualMOPCmd(HMICmdFrame &cmd){
//	static bool kk = false;
	printf("Get Virtual MOP Key : %d, len = %d, channel_index = %d,  cmd = %d, data= %hhu\n", cmd.cmd_extension,cmd.data_len,cmd.channel_index,cmd.cmd, cmd.data[0]);

	//发送回复数据
    cmd.frame_number |= 0x8000;
    this->SendCmd(cmd);

    uint8_t chan_index = cmd.channel_index;
    if(false == m_p_channel_engine->SetCurWorkChanl(chan_index)){
    	return;
    }
       
	switch(cmd.cmd_extension){
	case MOP_KEY_AUTO:
		m_p_channel_engine->SetWorkMode(AUTO_MODE);
		break;
	case MOP_KEY_MDA:
		m_p_channel_engine->SetWorkMode(MDA_MODE);
		break;
	case MOP_KEY_JOG:
		m_p_channel_engine->SetWorkMode(MANUAL_MODE);
		break;
	case MOP_KEY_JOG_STEP:
		m_p_channel_engine->SetWorkMode(MANUAL_STEP_MODE);
		break;
	case MOP_KEY_HANDWHEEL:
		m_p_channel_engine->SetWorkMode(MPG_MODE);
		break;
	case MOP_KEY_START:
		m_p_channel_engine->Start();
		break;
	case MOP_KEY_PAUSE:
	//	printf("key pause\n");
		m_p_channel_engine->Pause();
		break;
	case MOP_KEY_SINGLE_BLOCK:    //单段设置
		m_p_channel_engine->SetFuncState(CHANNEL_ENGINE_INDEX, FS_SINGLE_LINE);
		break;
	case MOP_KEY_JUMP:    //跳段设置
		m_p_channel_engine->SetFuncState(CHANNEL_ENGINE_INDEX, FS_BLOCK_SKIP);
		break;
	case MOP_KEY_M01:    //选停设置
		m_p_channel_engine->SetFuncState(CHANNEL_ENGINE_INDEX, FS_OPTIONAL_STOP);
		break;
	case MOP_KEY_HW_TRACE:     //手轮跟踪设置
		m_p_channel_engine->SetFuncState(CHANNEL_ENGINE_INDEX, FS_HANDWHEEL_CONTOUR);
		break;
	case MOP_KEY_G00_RATIO:    //快速倍率
        // lidianqiang: 去掉倍率控制的接口，快速倍率只由PMC信号ROV来控制
		break;
	case MOP_KEY_FEED_RATIO:
        // lidianqiang: 去掉倍率控制的接口，进给倍率只由PMC信号_FV来控制
        // m_p_channel_engine->SetAutoRatio(cmd.data[0]);
        // todo: 手动倍率也应该由pmc信号_JV来控制
		m_p_channel_engine->SetManualRatio(cmd.data[0]);
		break;
	case MOP_KEY_SPD_RATIO:
        // lidianqiang: 去掉倍率控制的接口，主轴倍率只由PMC信号SOV来控制
		break;
	case MOP_KEY_STEP_LEN:{
		uint16_t step;
		memcpy(&step, cmd.data, 2);
		m_p_channel_engine->SetManualStep(step);

		break;
	}
	case MOP_KEY_DIR_FOREWARD:
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();

		break;
	case MOP_KEY_DIR_REVERSE:
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_RAPID:
		m_p_channel_engine->SetManualRapidMove();
		break;
	case MOP_KEY_KEY_X_AXIS:			//X轴
		m_p_channel_engine->SetCurAxis(0);
		break;
	case MOP_KEY_KEY_Y_AXIS:			//Y轴
		m_p_channel_engine->SetCurAxis(1);
		break;
	case MOP_KEY_KEY_Z_AXIS:			//Z轴
		m_p_channel_engine->SetCurAxis(2);
		break;
	case MOP_KEY_KEY_4_AXIS:			//4轴
		m_p_channel_engine->SetCurAxis(3);
		break;
	case MOP_KEY_KEY_5_AXIS:			//5轴
		m_p_channel_engine->SetCurAxis(4);
		break;
	case MOP_KEY_KEY_6_AXIS:			//6轴
		m_p_channel_engine->SetCurAxis(5);
		break;
	case MOP_KEY_CUR_AXIS:
		m_p_channel_engine->SetCurAxis(cmd.data[0]);   //设置当前轴
		break;
	case MOP_KEY_SPD_FOREWARD:			//主轴正转
        // lidianqiang: 去掉主轴控制的接口，主轴由PMC来控制
        // m_p_channel_engine->SpindleOut(SPD_DIR_POSITIVE);
		break;
	case MOP_KEY_SPD_STOP:				//主轴停转
        // lidianqiang: 去掉主轴控制的接口，主轴由PMC来控制
        // m_p_channel_engine->SpindleOut(SPD_DIR_STOP);
		break;
	case MOP_KEY_SPD_REVERSE:			//主轴反转
        // lidianqiang: 去掉主轴控制的接口，主轴由PMC来控制
        // m_p_channel_engine->SpindleOut(SPD_DIR_NEGATIVE);
		break;
    case MOP_KEY_EMERGENCY:				//急停
        std::cout << "mop energency" << std::endl;
		this->m_p_channel_engine->Emergency();
		break;
	case MOP_KEY_CLEAR_ALARM:			//清除告警
		this->m_p_channel_engine->ClearAlarm();
		break;
	case MOP_KEY_SYS_RESET:				//系统复位

		this->m_p_channel_engine->SystemReset();
		break;
	case MOP_KEY_LIGHT:	{				//照明
	//	this->m_p_channel_engine->ProcessHmiSetRefCmd();  //测试设置参考点功能
#ifdef USES_PEITIAN_SMALL_FIVE
		if(cmd.data[0] == 1)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 0, true);
		else if(cmd.data[0] == 2)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 0, false);
#endif

		this->m_p_channel_engine->PrintDebugInfo();
		break;
	}
	case MOP_KEY_CHIP:					//排屑
#ifdef USES_LASER_MACHINE
		this->m_p_channel_engine->StartLaserCalib(true);   //测试调高器标定
#endif
//		if(cmd.data[0] == 1)
//			this->m_p_channel_engine->SetPmcOutSignal(17, true);
//		else
//			this->m_p_channel_engine->SetPmcOutSignal(17, false);
//		this->m_p_channel_engine->ProcessHmiIOCmd(cmd);

#ifdef USES_PEITIAN_SMALL_FIVE
		if(cmd.data[0] == 1)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 1, true);
		else if(cmd.data[0] == 2)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 1, false);
#endif
		break;
	case MOP_KEY_GAS:					//吹气
//		this->m_p_channel_engine->StartLaserCalib(false);   //测试调高器标定
//		printf("MOP_KEY_GAS : %hhu\n", cmd.data[0]);
//		if(cmd.data[0] == 1){
//			this->m_p_channel_engine->SetPmcOutSignal(16, true);
//
//		}else{
//			this->m_p_channel_engine->SetPmcOutSignal(16, false);
//		}

//		this->m_p_channel_engine->ProcessHmiIOCmd(cmd);
//		if(cmd.data[0] == 1){  //测试激活对刀
//			cmd.cmd_extension = 0;
//			cmd.data_len = 2*sizeof(int);
//			int h_value = 2;
//			int times = 3;
//			memcpy(cmd.data, &h_value, sizeof(int));
//			memcpy(cmd.data+sizeof(int), &times, sizeof(int));
//			this->m_p_channel_engine->GetChnControl(0)->ProcessHmiManualToolMeasureCmd(cmd);   //测试手动对刀
//		}

#ifdef USES_PEITIAN_SMALL_FIVE
		if(cmd.data[0] == 1)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 2, true);
		else if(cmd.data[0] == 2)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 2, false);
#endif

		break;
	case MOP_KEY_COOL:					//冷却
//		this->m_p_channel_engine->EnableLaserHeightTrace();  //测试使能跟随
		//测试增量编码器回零
//		cmd.cmd_extension = 0x00;
//		cmd.channel_index = 0;
//		this->m_p_channel_engine->ProcessHmiFindRefCmd(cmd);    //测试增量编码器回零

//		this->m_p_channel_engine->ProcessHmiIOCmd(cmd);

//		if(cmd.data[0] == 1){//测试取消对刀
//		    cmd.cmd_extension = 1;   //取消对刀
//			cmd.data_len = 2*sizeof(int);
//			int h_value = 2;
//			int times = 3;
//			memcpy(cmd.data, &h_value, sizeof(int));
//			memcpy(cmd.data+sizeof(int), &times, sizeof(int));
//			this->m_p_channel_engine->GetChnControl(0)->ProcessHmiManualToolMeasureCmd(cmd);   //测试手动对刀
//		}

#ifdef USES_PEITIAN_SMALL_FIVE
		if(cmd.data[0] == 1)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 3, true);
		else if(cmd.data[0] == 2)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 3, false);
#endif

		break;
	case MOP_KEY_AXIS:                 //PMC轴选定
		this->m_p_channel_engine->SetCurPmcAxis(cmd.data[0]);   //选择PMC轴
		break;
#ifdef USES_GRIND_MACHINE
	case MOP_KEY_Z_AXIS_POS:			//Z轴正向手动
		m_p_channel_engine->SetCurAxis(2);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_C_AXIS_POS:			//C轴正向手动
		m_p_channel_engine->SetCurAxis(3);
		if(cmd.data[0] == 1){
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		}
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_C_AXIS_NEG:			//C轴负向手动
		m_p_channel_engine->SetCurAxis(3);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_Z_AXIS_NEG:			//Z轴负向手动
		m_p_channel_engine->SetCurAxis(2);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_X_AXIS_NEG:           //X轴负向手动
		m_p_channel_engine->SetCurAxis(0);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_X_AXIS_POS:			//X轴正向手动
		m_p_channel_engine->SetCurAxis(0);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_X_AXIS_POS:          //PMC轴X正向
		m_p_channel_engine->SetCurPmcAxis(5);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_X_AXIS_NEG:       //PMC轴X负向
		m_p_channel_engine->SetCurPmcAxis(5);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_Y_AXIS_POS:       //PMC轴Y正向
		m_p_channel_engine->SetCurPmcAxis(6);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_Y_AXIS_NEG:       //PMC轴Y负向
		m_p_channel_engine->SetCurPmcAxis(6);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_LEFT_AXIS_POS:    //左侧托盘轴正向
		m_p_channel_engine->SetCurPmcAxis(7);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_LEFT_AXIS_NEG:    //左侧托盘轴负向
		m_p_channel_engine->SetCurPmcAxis(7);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_RIGHT_AXIS_POS:   //右侧托盘轴正向
		m_p_channel_engine->SetCurPmcAxis(8);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_RIGHT_AXIS_NEG:   //右侧托盘轴负向
		m_p_channel_engine->SetCurPmcAxis(8);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_WORKPIECE_CLAMP:		//工件夹紧
	case MOP_KEY_VACUUM:     			//真空
	case MOP_KEY_NEW_TRAY:				//新料盘
	case MOP_KEY_PAUSE_CARRY:          //暂停上下料
		this->m_p_channel_engine->ProcessHmiIOCmd(cmd);
		break;
	case MOP_KEY_IO_0:
	case MOP_KEY_IO_1:
	case MOP_KEY_IO_2:
	case MOP_KEY_IO_3:
	case MOP_KEY_IO_4:
	case MOP_KEY_IO_5:
	case MOP_KEY_IO_6:
	case MOP_KEY_IO_7:
	case MOP_KEY_IO_8:
	case MOP_KEY_IO_9:
	case MOP_KEY_IO_10:
	case MOP_KEY_IO_11:
	case MOP_KEY_IO_12:
	case MOP_KEY_IO_13:
	case MOP_KEY_IO_14:
	case MOP_KEY_IO_15:
	case MOP_KEY_IO_16:
	case MOP_KEY_IO_17:
	case MOP_KEY_IO_18:
	case MOP_KEY_IO_19:
	case MOP_KEY_IO_20:
	case MOP_KEY_IO_21:
	case MOP_KEY_IO_22:
	case MOP_KEY_IO_23:
	case MOP_KEY_IO_24:
	case MOP_KEY_IO_25:
	case MOP_KEY_IO_26:
	case MOP_KEY_IO_27:
	case MOP_KEY_IO_28:
	case MOP_KEY_IO_29:
	case MOP_KEY_IO_30:
	case MOP_KEY_IO_31:
//		if(cmd.data[0] == 1){//按下
//			this->m_p_channel_engine->SetPmcOutSignal(cmd.cmd_extension-MOP_KEY_IO_0, true);
//		}
//		else if(cmd.data[0] == 2){//弹起
//			this->m_p_channel_engine->SetPmcOutSignal(cmd.cmd_extension-MOP_KEY_IO_0, false);
//		}else if(cmd.data[0] == 0){//开关
			this->m_p_channel_engine->SetPmcOutSignal(cmd.cmd_extension-MOP_KEY_IO_0, cmd.data[0]);
//		}
		break;
	case MOP_KEY_SPINDLE_START:  		//主轴启动
//		if(cmd.data[1] == 1)
			m_p_channel_engine->SpindleOut(SPD_DIR_POSITIVE);
//		else
//			m_p_channel_engine->SpindleOut(SPD_DIR_NEGATIVE);
		break;
	case MOP_KEY_SPINDLE_STOP:			//主轴停止
		m_p_channel_engine->SpindleOut(SPD_DIR_STOP);
		break;

	case MOP_KEY_FETCHING_PIECE:   //取料中
	case MOP_KEY_START_WORKING:    //取料结束
		this->m_p_channel_engine->ProcessHmiIOCmd(cmd);
		break;

#endif
	default:
		break;
	}
}

/**
 * @brief 删除文件
 * @param name
 * @return true--成功   false--失败
 */
bool HMICommunication::DeleteNcFile(const char *name){
	char path[kMaxPathLen] = {0};
	strcpy(path, PATH_NC_FILE);
	strcat(path, name);

	int res = remove(path);
	if(res == 0)
		return true;
	return false;
}

/**
 * @brief 重命名文件
 * @param old_name : 原文件名称
 * @param new_name : 新文件名称
 * @return true--成功   false--失败
 */
bool HMICommunication::RenameNcFile(const char *old_name, const char *new_name){
	char path_old[kMaxPathLen] = {0};
	char path_new[kMaxPathLen] = {0};
	strcpy(path_old, PATH_NC_FILE);
	strcat(path_old, old_name);
	strcpy(path_new, PATH_NC_FILE);
	strcat(path_new, new_name);

	int res = rename(path_old, path_new);
	if(res == 0)
		return true;
	return false;
}

/**
 * @brief 另存为文件,开启一个线程进行文件另存为操作
 * @param old_name : 原文件名称
 * @param new_name : 新文件名称
 * @return true--成功   false--失败
 */
bool HMICommunication::SaveasNcFile(const char *old_name, const char *new_name){

	pthread_attr_t attr;
	struct sched_param param;
	int res = 0;

	SaveAsParam *pParam = new SaveAsParam();
	pParam->pComm = this;
	strcpy(pParam->path_old, PATH_NC_FILE);
	strcat(pParam->path_old, old_name);
	strcpy(pParam->path_new, PATH_NC_FILE);
	strcat(pParam->path_new, new_name);

	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 31; //91;
	pthread_attr_setschedparam(&attr, &param);
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
//	if (res) {
//		g_ptr_trace->PrintLog(LOG_ALARM, "文件另存为线程设置线程继承模式失败！");
//		pthread_attr_destroy(&attr);
//		return false;
//	}
    res = pthread_create(&m_thread_saveas, &attr,
            HMICommunication::SaveAsFileThread, pParam);    //开启文件另存为处理线程

	pthread_attr_destroy(&attr);
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "文件另存为线程启动失败！");
		return false;
	}

	return true;
}

/**
 * @brief 删除文件
 * @param name
 * @return true--成功   false--失败
 */
bool HMICommunication::DeleteEsbFile(const char *name){
	char path[kMaxPathLen] = {0};
	strcpy(path, PATH_ESB_FILE);
	strcat(path, name);

	int res = remove(path);
	if(res == 0)
		return true;
	return false;
}

/**
 * @brief 重命名文件
 * @param old_name : 原文件名称
 * @param new_name : 新文件名称
 * @return true--成功   false--失败
 */
bool HMICommunication::RenameEsbFile(const char *old_name, const char *new_name){
	char path_old[kMaxPathLen] = {0};
	char path_new[kMaxPathLen] = {0};
	strcpy(path_old, PATH_ESB_FILE);
	strcat(path_old, old_name);
	strcpy(path_new, PATH_ESB_FILE);
	strcat(path_new, new_name);

	int res = rename(path_old, path_new);
	if(res == 0)
		return true;
	return false;
}


/**
 * @brief 另存为文件
 * @param old_name : 原文件名称
 * @param new_name : 新文件名称
 * @return true--成功   false--失败
 */
bool HMICommunication::SaveasEsbFile(const char *old_name, const char *new_name){
	pthread_attr_t attr;
	struct sched_param param;
	int res = 0;

	SaveAsParam *pParam = new SaveAsParam();
	pParam->pComm = this;
	strcpy(pParam->path_old, PATH_ESB_FILE);
	strcat(pParam->path_old, old_name);
	strcpy(pParam->path_new, PATH_ESB_FILE);
	strcat(pParam->path_new, new_name);

	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 31; //91;
	pthread_attr_setschedparam(&attr, &param);
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
//	if (res) {
//		g_ptr_trace->PrintLog(LOG_ALARM, "文件另存为线程设置线程继承模式失败！");
//		pthread_attr_destroy(&attr);
//		return false;
//	}
	res = pthread_create(&m_thread_saveas, &attr,
			HMICommunication::SaveAsFileThread, pParam);    //开启文件另存为处理线程

	pthread_attr_destroy(&attr);
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "文件另存为线程启动失败！");
		return false;
	}

	return true;
}

/**
 * @brief 关闭与HMI的链接
 */
void HMICommunication::DisconnectToHmi(){

	//关闭监视数据连接
	if(this->m_soc_monitor_send > 0){
		close(m_soc_monitor_send);
		m_soc_monitor_send = -1;
		g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "关闭监视数据连接！");
	}

	//关闭文件传输连接
	if(m_soc_file_send > 0){
		close(m_soc_file_send);
	//	closesocket(m_soc_file_send);
		m_soc_file_send = -1;
		g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "关闭文件传输连接！");
	}

	g_sys_state.hmi_comm_ready = false;
}

/**
 * @brief 发送文件
 * @return
 */
int HMICommunication::SendFile(){
	int res = ERR_NONE;
	uint64_t file_size = 0;  //文件大小
	int send_size = 0;      //单次发送的大小
	uint64_t send_total = 0;  //已发送的文件大小
	char buffer[kTcpBufferSize] = {0};	//缓冲
	char filepath[kMaxPathLen] = {0};	//文件路径
	char sign_path[kMaxPathLen] = {0};    //签名文件路径
	char file_sign[kNcFileSignLen] = {0};  //签名
	int fd = -1;                    //文件句柄
	uint8_t file_type = m_file_type;  //文件传输类型
	uint8_t filename_size = 0;         //文件名长度
	char *pSplit = nullptr;
	uint8_t alarm_type = 0x20;   // 初始传送文件alarmfile_bak.txt
	uint64_t total_size = 0;
	int send_count = 0;

	//测试耗时
	struct timeval tvStart;
	struct timeval tvNow;
	unsigned int nTimeDelay = 0;

	HMICmdFrame cmd;

	bzero((char *)&cmd, kMaxHmiCmdFrameLen);
	cmd.cmd = CMD_SC_TRANS_FILE_OVER;   //结果响应命令包
	cmd.cmd_extension = file_type;
	cmd.data_len = 1;

	gettimeofday(&tvStart, NULL);

	if(file_type == FILE_G_CODE){
		filename_size = strlen(this->m_str_file_name);
		if(strcmp(this->m_str_file_name, "mda.nc") == 0){
			uint8_t chn = cmd.channel_index;
			sprintf(filepath, PATH_MDA_FILE, chn);
		}else{
			strcpy(filepath, PATH_NC_FILE);
			cmd.data_len += filename_size;
			strcpy(cmd.data, m_str_file_name);
			strcat(filepath, m_str_file_name);  //组合出文件绝对路径
		}

		strcpy(sign_path, PATH_NC_SIGN_FILE);
		strcat(sign_path, m_str_file_name);
		pSplit = strchr(sign_path, '.');
		if(pSplit != nullptr)
			*pSplit = '\0';
		strcat(sign_path, "_md5");   //组合出签名文件绝对路径
		fd = open(sign_path, O_RDONLY);
		if(fd > 0){
			read(fd, file_sign, kNcFileSignLen);
			close(fd);
		}

	}else if(file_type == FILE_PMC_LADDER){		//梯图文件
		strcpy(filepath, PATH_PMC_LDR);


	}else if(file_type == FILE_UPDATE){   //模块更新文件

	}else if(file_type == FILE_CONFIG_PACK){  //配置打包文件
		return this->SendConfigPackFile();

	}else if(file_type == FILE_WARN_HISTORY){  //告警历史文件
		total_size = TraceInfo::GetAlarmTotalSize();
		if(!TraceInfo::IsAlarmFileExist(alarm_type))
			alarm_type = 0x10;
		TraceInfo::GetAlarmFilePath(alarm_type, filepath);
	}else if(file_type == FILE_ESB_DEV){  //伺服描述文件
		filename_size = strlen(this->m_str_file_name);
		strcpy(filepath, PATH_ESB_FILE);
		strcat(filepath, m_str_file_name);
	}else if(file_type == FILE_BACK_DISK) { //一键备份
		strcpy(filepath, "/cnc/back.zip");
		char cmd_buf[100];
		sprintf(cmd_buf, "chmod a+x %s", PATH_BACK_DISK_CMD);
		system(cmd_buf);
		int ret = system(PATH_BACK_DISK_CMD);
		if (system(PATH_BACK_DISK_CMD) != 0) { //运行备份脚本
			printf("error in SendFile fun, failed to run back cmd, return %d %d\n", ret & 0xFF, ret>>8);
			res = ERR_FILE_TRANS;
			goto END;
		}
    }else if(file_type == FILE_BACKUP_CNC) { //备份
        strcpy(filepath, BACKUP_DIR.c_str());
    }
//	else if(file_type == FILE_SYSTEM_CONFIG){
//
//	}else if(file_type == FILE_CHANNEL_CONFIG){
//
//	}else if(file_type == FILE_AXIS_CONFIG){
//
//	}else if(file_type == FILE_TOOL_CONFIG){
//
//	}


	//打开文件
	fd = open(filepath, O_RDONLY);
	if(fd == -1){
		printf("error in SendFile fun, failed to open file:%s\n", filepath);
		res = ERR_FILE_TRANS;
		goto END;
	}


	//首先读取文件大小
	file_size = lseek(fd, 0, SEEK_END);

	lseek(fd, 0, SEEK_SET);

	//发送文件类型,文件名以及文件大小
	memcpy(buffer, (char *)&file_type, 1);
	if(file_type == FILE_G_CODE){
		memcpy(buffer+1, (char *)&filename_size, 1);
		memcpy(buffer+2, m_str_file_name, filename_size);
		memcpy(buffer+2+filename_size, file_sign, kNcFileSignLen);
		memcpy(buffer+2+filename_size+kNcFileSignLen, (char *)&file_size, sizeof(uint64_t));
		send_size = 10+kNcFileSignLen+filename_size;
	}else if(file_type == FILE_WARN_HISTORY){  //发送告警历史文件类型，及文件大小
		memcpy(buffer+1, (char *)&total_size, sizeof(uint64_t));
		send_size = 9;
	}else if(file_type == FILE_ESB_DEV){
		memcpy(buffer+1, (char *)&filename_size, 1);
		memcpy(buffer+2, m_str_file_name, filename_size);
		memcpy(buffer+2+filename_size, (char *)&file_size, sizeof(uint64_t));
		send_size = 10+filename_size;
	}
	else{
		memcpy(buffer+1, (char *)&file_size, sizeof(uint64_t));
		send_size = 9;
	}



	if( -1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
		g_ptr_trace->PrintLog(LOG_ALARM, "文件大小发送异常！errno = %d", errno);
		res = ERR_FILE_TRANS;
		close(fd);
		goto END;
	}

TRAN:
	send_total = 0;
	while(send_total < file_size){
		bzero(buffer, kTcpBufferSize);
		send_size = read(fd, buffer, kTcpBufferSize);

		if(-1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
			g_ptr_trace->PrintLog(LOG_ALARM, "文件数据发送异常！已发送%lld字节，总大小%lld字节, errno = %d",send_total, file_size, errno);
			res = ERR_FILE_TRANS;
			close(fd);
			goto END;
		}

		//printf("---------------> send_total %llu\n", send_total);
		send_total += send_size;

//		printf("send file: total = %lld, send= %lld\n", file_size, send_total);
		if(send_count++ >= 50){
			usleep(1000);
			send_count = 0;
		}
	}

	close(fd);

	if(file_type == FILE_WARN_HISTORY && alarm_type == 0x20){
		alarm_type = 0x10;
		TraceInfo::GetAlarmFilePath(alarm_type, filepath);

		fd = open(filepath, O_RDONLY);
		if(fd == -1){
			printf("error in SendFile fun, failed to open file:%s\n", filepath);
			res = ERR_FILE_TRANS;
			goto END;
		}


		//读取文件大小
		file_size = lseek(fd, 0, SEEK_END);

		lseek(fd, 0, SEEK_SET);
		goto TRAN;
	}

	gettimeofday(&tvNow, NULL);
	nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;
	printf("total time = %u us, filesize = %lld\n", nTimeDelay, file_size);

	printf("send file over, sendsize = %lld, filesize = %lld\n", send_total, file_size);


    if(file_type == FILE_BACKUP_CNC && m_sysbackup_status.m_type == SysUpdateStatus::Backup) { //备份
        m_sysbackup_status.m_status = SysUpdateStatus::Idle;
        SendHMIBackupStatus(m_sysbackup_status);
    }
	END:
#ifndef USES_TCP_FILE_TRANS_KEEP
	if(m_soc_file_send > 0){
		close(m_soc_file_send);
		m_soc_file_send = -1;
	}
#endif

	if(res != ERR_NONE)
		cmd.data[cmd.data_len-1] = FAILED;

//#ifdef USES_MODBUS_PROTOCAL
//	//发送回复包
//	int len = kHmiCmdHeadLen+cmd.data_len;
//	int rb = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&m_addr_file_trans, m_n_addr_len);
//	if(len != rb){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "发送文件传输指令完成指令失败[host:%s]！", inet_ntoa(m_addr_file_trans.sin_addr));
//	}else{
//		printf("send file over cmd:ip[%s:%hu]\n", inet_ntoa(m_addr_file_trans.sin_addr), ntohs(m_addr_file_trans.sin_port));
//	}
//#else
	this->SendCmd(cmd);   //发送响应
//#endif

	return res;
}

/**
 * @brief 获取文件长度
 * @return 数据长度
 */
uint64_t HMICommunication::GetFileLength(const char *path){
	uint64_t len = 0;
	FILE *file = fopen(path, "r+");
	if (nullptr == file)
	{
		return len;
	}

	fseek(file, 0L, SEEK_END);
	len = ftell(file);   //获取文件长度

	fclose(file);
	return len;
}

/**
 * @brief 获取配置文件路径
 * @param path[out] : 返回文件路径
 * @param type ： 参数类型
 * @return true--成功   false--失败
 */
bool HMICommunication::GetConfigFilePath(char *path, uint8_t type){
	bool res = true;
	switch(type){
    case CONFIG_SYSTEM:
		strcpy(path, SYS_CONFIG_FILE);
		break;
    case CONFIG_CHANNEL:
		strcpy(path, CHN_CONFIG_FILE);
		break;
    case CONFIG_AXIS:
		strcpy(path, AXIS_CONFIG_FILE);
		break;
    case CONFIG_COORD:
		strcpy(path, WORK_COORD_FILE);
		break;
    case CONFIG_EX_COORD:
		strcpy(path, EX_WORK_COORD_FILE);
		break;
    case CONFIG_TOOL_INFO:
		strcpy(path, TOOL_CONFIG_FILE);
		break;
#ifdef USES_FIVE_AXIS_FUNC
    case CONFIG_FIVE_AXIS:
		strcpy(path, FIVE_AXIS_CONFIG_FILE);
		break;
#endif
#ifdef USES_GRIND_MACHINE
	case CONFIG_GRIND:
		strcpy(path, GRIND_CONFIG_FILE);
		break;
#endif
	case CONFIG_MACRO_VAR:
	//	strcpy(path, PATH_MACRO_VAR_KEEP);
		sprintf(path, PATH_MACRO_VAR_KEEP, 0);  //返回第一通道的宏变量文件名
		break;
	case CONFIG_PMC_REG:
		strcpy(path, PATH_PMC_REG);
		break;
	case CONFIG_PMC_LDR:
		strcpy(path, PATH_PMC_LDR);
		break;
	case CONFIG_IO_REMAP:
		strcpy(path, IO_REMAP_FILE);
		break;
	default:
		res = false;
		break;
	}
	return res;
}

/**
 * @brief 计算配置打包文件总字节数
 * @return 总字节数
 */
uint64_t HMICommunication::GetConfigPackFileSize(){
	uint64_t total_size = 4;  //4字节mask

	if(this->m_mask_config_pack & (0x01<<CONFIG_SYSTEM)){  //系统参数
		total_size += 10;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度
		total_size += this->GetFileLength(SYS_CONFIG_FILE); //加上文件大小
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_CHANNEL)){  //通道参数
		total_size += 10;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度
		total_size += this->GetFileLength(CHN_CONFIG_FILE); //加上文件大小
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_AXIS)){  //轴参数
		total_size += 10;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度
		total_size += this->GetFileLength(AXIS_CONFIG_FILE); //加上文件大小
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_COORD)){  //工件坐标系配置
		total_size += 10;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度
		total_size += this->GetFileLength(WORK_COORD_FILE); //加上文件大小
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_EX_COORD)){  //扩展工件坐标系
		total_size += 10;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度
		total_size += this->GetFileLength(EX_WORK_COORD_FILE); //加上文件大小
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_TOOL_INFO)){  //刀具信息，包括刀具偏置及刀具位置
		total_size += 10;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度
		total_size += this->GetFileLength(TOOL_CONFIG_FILE); //加上文件大小
	}

#ifdef USES_FIVE_AXIS_FUNC
	if(this->m_mask_config_pack & (0x01<<CONFIG_FIVE_AXIS)){  //五轴配置
		total_size += 10;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度
		total_size += this->GetFileLength(FIVE_AXIS_CONFIG_FILE); //加上文件大小
	}
#endif

#ifdef USES_GRIND_MACHINE
	if(this->m_mask_config_pack & (0x01<<CONFIG_GRIND)){  //磨削配置
		total_size += 10;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度
		total_size += this->GetFileLength(GRIND_CONFIG_FILE); //加上文件大小
	}
#endif

	if(this->m_mask_config_pack & (0x01<<CONFIG_MACRO_VAR)){  //宏变量
		total_size += 11;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度 + 1字节通道数
		char path[kMaxPathLen] = {0};		//目的文件路径
		sprintf(path, PATH_MACRO_VAR_KEEP, 0);
		total_size += this->GetFileLength(path); //加上文件大小
		for(uint8_t i = 1; i < g_ptr_parm_manager->GetSystemConfig()->chn_count; i++){
			memset(path, 0x00, kMaxPathLen);
			sprintf(path, PATH_MACRO_VAR_KEEP, i);
			total_size += 9;
			total_size += this->GetFileLength(path); //加上文件大小
		}
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_PMC_REG)){  //PMC寄存器
		total_size += 10;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度
		total_size += this->GetFileLength(PATH_PMC_REG); //加上文件大小
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_PMC_LDR)){  //PMC梯图
		total_size += 10;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度(梯图文件）
		total_size += this->GetFileLength(PATH_PMC_LDR); //加上梯图文件大小
		total_size += 9;    //1字节分隔符‘#’+ 8字节数据长度(PMC执行文件）
		total_size += this->GetFileLength(PATH_PMC_DATA); //加上PMC执行文件大小
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_IO_REMAP)){ //io重映射配置
		total_size += 10;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度
		total_size += this->GetFileLength(IO_REMAP_FILE); //加上文件大小
	}
    return total_size;
}

/**
 * @brief 压缩备份文件
 * @return
 */
void HMICommunication::PackageSysBackupFile()
{
    BackUp_Manager manager;
    manager.Init_Pack_Info(m_maks_sys_backup);
    m_sysbackup_status.m_cur_step = 0;
    m_sysbackup_status.m_total_step = manager.Info_Cnt();
    m_sysbackup_status.m_status = SysUpdateStatus::Backupping;
    SendHMIBackupStatus(m_sysbackup_status);
    struct zip_t *zip = zip_open(BACKUP_DIR.c_str(), 1, 'w');
    if (!zip)
    {
        m_sysbackup_status.m_err_code = -1;
        SendHMIBackupStatus(m_sysbackup_status);
        return;
    }

    bool ret = true;
    auto now = chrono::steady_clock::now();
    for (auto itr = manager.begin(); itr != manager.end(); ++itr)
    {
        if (!(*itr)->Package(zip))
        {
            ret = false;
            break;
        }
        m_sysbackup_status.m_cur_step++;
        if (chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - now).count() > 300)
        {//300ms向HMI更新压缩状态
            SendHMIBackupStatus(m_sysbackup_status);
            now = chrono::steady_clock::now();
        }
    }
    zip_close(zip);

    if (!ret)
    {
        m_sysbackup_status.m_err_code = -2;
        SendHMIBackupStatus(m_sysbackup_status);
        return;
    }

    m_sysbackup_status.m_status = SysUpdateStatus::FileTransing;//通知hmi准备开始文件传输
    SendHMIBackupStatus(m_sysbackup_status);
}

/**
 * @brief 解压备份文件
 * @return
 */
void HMICommunication::UnPackageBackupFile()
{
    m_sysbackup_status.m_status = SysUpdateStatus::Recovering;
    m_sysbackup_status.m_type = SysUpdateStatus::Recover;
    m_sysbackup_status.m_err_code = 0;

    if (access((RECOVER_FILE).c_str(), F_OK))
    {
        m_sysbackup_status.m_err_code = -1;
        SendHMIBackupStatus(m_sysbackup_status);
        return;
    }

    string command;
    if (!access(RECOVER_TEMP.c_str(), F_OK))
    {//存在就先删除目录
        command = "rm -rf " + RECOVER_TEMP;
        system(command.c_str());
    }
    command = "mkdir " + RECOVER_TEMP;
    system(command.c_str());

    struct zip_t *zip = zip_open(RECOVER_FILE.c_str(), 0, 'r');
    m_sysbackup_status.m_cur_step = 0;
    m_sysbackup_status.m_total_step = zip_entries_total(zip);

    bool ret = true;
    BackUp_Manager manager;
    manager.Init_UnPack_Info(m_maks_sys_backup, zip);
    auto now = chrono::steady_clock::now();
    for (auto itr = manager.begin(); itr != manager.end(); ++itr)
    {
        if (!(*itr)->UnPackage(zip, RECOVER_TEMP+'/'))
        {
            ret = false;
            break;
        }
        m_sysbackup_status.m_cur_step++;
        if (chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - now).count() > 300)
        {//300ms向HMI更新压缩状态
            SendHMIBackupStatus(m_sysbackup_status);
            now = chrono::steady_clock::now();
        }
     }
     zip_close(zip);


    //提取出现错误，删除恢复文件
    if (!ret)
    {
        command = "rm -rf " + RECOVER_TEMP;
        system(command.c_str());
        m_sysbackup_status.m_err_code = -2;
        SendHMIBackupStatus(m_sysbackup_status);
        return;
    }

    //SC
    string scPath = RECOVER_TEMP + SC_DIR;
    if (!access(scPath.c_str(), F_OK))
    {
        std::cout << "sc chmod" << std::endl;
        command = "chmod +x " + scPath;
        system(command.c_str());
    }

    //PL

    //MC

    std::cout << "UnPack Finished" << std::endl;

    m_sysbackup_status.m_status = SysUpdateStatus::Idle;
    SendHMIBackupStatus(m_sysbackup_status);
}

/**
 * @brief 向HMI发送当前备份/恢复状态
 * @return
 */
void HMICommunication::SendHMIBackupStatus(SysUpdateStatus status)
{
    HMICmdFrame cmd;
    cmd.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.cmd = CMD_SC_BACKUP_STATUS;

    cmd.data_len = sizeof(status);
    std::cout << "cur " << status.m_cur_step << " total " << status.m_total_step
              << " status " << status.m_status<< " error " << status.m_err_code << std::endl;
    memcpy(cmd.data, &status, cmd.data_len);
    SendCmd(cmd);
}

/**
 * @brief 发送配置打包文件
 * @return
 */
int HMICommunication::SendConfigPackFile(){
	int res = ERR_NONE;
	int send_size = 0;      //单次发送的大小
	int fd = -1;                    //文件句柄
	char buffer[kTcpBufferSize] = {0};	//缓冲
	char filepath[kMaxPathLen] = {0};	//文件路径
	uint8_t file_type = m_file_type;  //文件传输类型
	uint64_t send_total = 0;  //已发送的文件大小
	uint64_t total_size = this->GetConfigPackFileSize(); //备份文件总大小
	uint64_t file_size = 0;  //当前文件大小
	uint8_t i = 0, j = 0;
	uint8_t chn_count = 0;


	this->m_p_channel_engine->SyncKeepVar();   //同步保存文件

	//发送文件类型以及文件大小
	memcpy(buffer, (char *)&file_type, 1);
	memcpy(buffer+1, (char *)&total_size, sizeof(uint64_t));
	memcpy(buffer+9, &m_mask_config_pack, 4);
	send_size = 13;
	total_size -= 4;

	if( -1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
		g_ptr_trace->PrintLog(LOG_ALARM, "文件大小发送异常！errno = %d", errno);
		res = ERR_FILE_TRANS;
		goto END;
	}

	//开始发送文件
	for(i = 0; i < CONFIG_TYPE_COUNT; i++){
		if(this->m_mask_config_pack & (0x01<<i)){  //通道参数
			bzero(filepath, kMaxPathLen);
			if(!this->GetConfigFilePath(filepath, i)){
				printf("error in SendConfigPackFile, failed to get config filepath:%hhu\n", i);
				res = ERR_FILE_TRANS;
				goto END;
			}
			send_total = 0;
			send_size = 10;   //1字节分隔符‘#’+ 1字节参数类型 + 8字节数据长度
			fd = open(filepath, O_RDONLY);
			if(fd == -1){
				printf("error in SendConfigPackFile, failed to open file:%s\n", filepath);
				res = ERR_FILE_TRANS;
				goto END;
			}

			//首先读取文件大小
			file_size = lseek(fd, 0, SEEK_END);
			lseek(fd, 0, SEEK_SET);
			bzero(buffer, kTcpBufferSize);
			buffer[0] = '#'; //分隔符
			memcpy(buffer+1, &i, 1);  //参数类型
			memcpy(buffer+2, &file_size, 8);  //文件大小

			if(i == CONFIG_MACRO_VAR){//宏变量是每个通道一个文件，因此需要通道数
				chn_count = g_ptr_parm_manager->GetSystemConfig()->chn_count;
				buffer[10] = chn_count;
				send_size++;
			}

			if( -1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
				g_ptr_trace->PrintLog(LOG_ALARM, "文件大小发送异常！errno = %d", errno);
				res = ERR_FILE_TRANS;
				close(fd);
				fd = -1;
				goto END;
			}
			total_size -= send_size;

			while(send_total < file_size){
				bzero(buffer, kTcpBufferSize);
				send_size = read(fd, buffer, kTcpBufferSize);

				if(-1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
					g_ptr_trace->PrintLog(LOG_ALARM, "文件数据发送异常！已发送%lld字节，总大小%lld字节, errno = %d",send_total, file_size, errno);
					res = ERR_FILE_TRANS;
					close(fd);
					goto END;
				}

				send_total += send_size;
			}
			close(fd);
			fd = -1;
			total_size -= send_total;

			if(i == CONFIG_PMC_LDR){ //梯图还要发送执行文件
				bzero(filepath, kMaxPathLen);
				strcpy(filepath, PATH_PMC_DATA);
				send_total = 0;
				send_size = 9;   //1字节分隔符'#' + 8字节数据长度
				fd = open(filepath, O_RDONLY);
				if(fd == -1){
					printf("error in SendConfigPackFile, failed to open file:%s\n", filepath);
					res = ERR_FILE_TRANS;
					goto END;
				}

				//首先读取文件大小
				file_size = lseek(fd, 0, SEEK_END);
				lseek(fd, 0, SEEK_SET);
				bzero(buffer, kTcpBufferSize);
				buffer[0] = '#'; //分隔符
				memcpy(buffer+1, &file_size, 8);  //文件大小

				if( -1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
					g_ptr_trace->PrintLog(LOG_ALARM, "文件大小发送异常！errno = %d", errno);
					res = ERR_FILE_TRANS;
					close(fd);
					fd = -1;
					goto END;
				}
				total_size -= send_size;

				while(send_total < file_size){
					bzero(buffer, kTcpBufferSize);
					send_size = read(fd, buffer, kTcpBufferSize);

					if(-1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
						g_ptr_trace->PrintLog(LOG_ALARM, "文件数据发送异常！已发送%lld字节，总大小%lld字节, errno = %d",send_total, file_size, errno);
						res = ERR_FILE_TRANS;
						close(fd);
						goto END;
					}

					send_total += send_size;
				}
				close(fd);
				fd = -1;
				total_size -= send_total;
			}else if(i == CONFIG_MACRO_VAR && chn_count > 1){  //多通道时保存宏变量参数
				for(j = 1; j < chn_count; j++){
					bzero(filepath, kMaxPathLen);
					sprintf(filepath, PATH_MACRO_VAR_KEEP, j);

					send_total = 0;
					send_size = 9;   //1字节分隔符'#' + 8字节数据长度
					fd = open(filepath, O_RDONLY);
					if(fd == -1){
						printf("error in SendConfigPackFile, failed to open file:%s\n", filepath);
						res = ERR_FILE_TRANS;
						goto END;
					}

					//首先读取文件大小
					file_size = lseek(fd, 0, SEEK_END);
					lseek(fd, 0, SEEK_SET);
					bzero(buffer, kTcpBufferSize);
					buffer[0] = '#'; //分隔符
					memcpy(buffer+1, &file_size, 8);  //文件大小

					if( -1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
						g_ptr_trace->PrintLog(LOG_ALARM, "文件大小发送异常！errno = %d", errno);
						res = ERR_FILE_TRANS;
						close(fd);
						fd = -1;
						goto END;
					}
					total_size -= send_size;

					while(send_total < file_size){
						bzero(buffer, kTcpBufferSize);
						send_size = read(fd, buffer, kTcpBufferSize);

						if(-1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
							g_ptr_trace->PrintLog(LOG_ALARM, "文件数据发送异常！已发送%lld字节，总大小%lld字节, errno = %d",send_total, file_size, errno);
							res = ERR_FILE_TRANS;
							close(fd);
							goto END;
						}

						send_total += send_size;
					}
					close(fd);
					fd = -1;
					total_size -= send_total;
				}

			}
		}
	}

	if(total_size != 0){
		g_ptr_trace->PrintLog(LOG_ALARM, "文件数据发送异常！剩余字节数%lld字节，errno = %d",total_size, errno);
		res = ERR_FILE_TRANS;
	}


	END:
#ifndef USES_TCP_FILE_TRANS_KEEP
	if(m_soc_file_send > 0){
		close(m_soc_file_send);
		m_soc_file_send = -1;
	}
#endif

	return res;
}

/**
 * @brief 解压处理配置打包文件
 * @return true--成功  false--失败
 */
int HMICommunication::UnpackConfigBakFile(){
	int res = ERR_NONE;

	uint64_t file_size = 0;  //文件大小
	int read_size = 0;      //读取包的大小
	int read_block = 0;		//当前读取数据大小
//	uint64_t read_total = 0;  //已读取的文件大小
	char buffer[kTcpBufferSize] = {0};	//缓冲
	char filepath[kMaxPathLen] = {0};		//目的文件路径
	uint32_t cfg_mask = 0;    //配置mask
	uint8_t i = 0, j = 0;
	char split_char;  //分隔符
	uint8_t cfg_type;  //配置参数类型
	uint64_t cfg_file_size = 0;  //配置文件大小

	uint8_t chn_count = 0;

	int fd = -1, fdd = -1;

	fd = open(PATH_CONFIG_PACK_TMP, O_RDONLY);
	if(fd == -1){
		printf("error in UnpackConfigBakFile, failed to open file:%s\n", PATH_CONFIG_PACK_TMP);
		res = ERR_FILE_TRANS;
		goto END;
	}

	//首先读取文件大小
	file_size = lseek(fd, 0, SEEK_END);
	lseek(fd, 0, SEEK_SET);

	//读取配置mask
	read_size = read(fd, &cfg_mask, sizeof(uint32_t));
	if(read_size != sizeof(uint32_t)){
		printf("error in UnpackConfigBakFile, failed to read config mask, readsize=%d\n", read_size);
		res = ERR_FILE_TRANS;
		goto END;
	}
	printf("read pack config file mask: 0x%x\n", cfg_mask);

	file_size -= sizeof(uint32_t);

	for(i = 0; i < CONFIG_TYPE_COUNT  && file_size > 0; i++){
		if(cfg_mask & (0x01<<i)){  //通道参数
			if(file_size < 10){
				printf("error in UnpackConfigBakFile, invalid file format\n");
				res = ERR_FILE_TRANS;
				goto END;
			}
			printf("unpack config:%hhu\n", i);
			//读取分隔符，配置类型及配置文件长度
			read(fd, &split_char, 1);
			read(fd, &cfg_type, 1);
			read(fd, &cfg_file_size, 8);
			if(split_char != '#' || cfg_type != i){
				printf("error in UnpackConfigBakFile, invalid file format2, type = %hhu\n", i);
				res = ERR_FILE_TRANS;
				goto END;
			}
			file_size -= 10;

			if(i == CONFIG_MACRO_VAR){//宏变量是每个通道一个文件，因此需要通道数
				read(fd, &chn_count, 1);
				file_size--;
			}

			bzero(filepath, kMaxPathLen);
			if(!this->GetConfigFilePath(filepath, i)){
				printf("error in UnpackConfigBakFile, failed to get config filepath:%hhu\n", i);
				res = ERR_FILE_TRANS;
				goto END;
			}
			fdd = open(filepath, O_WRONLY|O_CREAT|O_TRUNC);
			if(fdd == -1){
				printf("error in UnpackConfigBakFile, failed to create file:%s\n", filepath);
				res = ERR_FILE_TRANS;
				goto END;
			}

			//读取文件并写入配置
			while(0 < cfg_file_size){
				bzero(buffer, kTcpBufferSize);
				read_block = cfg_file_size > kTcpBufferSize?kTcpBufferSize:cfg_file_size;
				read_size = read(fd, buffer, read_block);
				if(read_size != read_block){
					printf("error in UnpackConfigBakFile, read file error:%hhu\n", i);
					res = ERR_FILE_TRANS;
					goto END;
				}

				write(fdd, buffer, read_size);  //写入文件
				cfg_file_size -= read_size;
				file_size -= read_size;

			}
			fsync(fdd);
			close(fdd);
			fdd = -1;

			if(i == CONFIG_PMC_LDR){//还要读取pmc运行文件
				if(file_size < 9){
					printf("error in UnpackConfigBakFile, invalid file format\n");
					res = ERR_FILE_TRANS;
					goto END;
				}
				//读取分隔符，及配置文件长度
				read(fd, &split_char, 1);
				read(fd, &cfg_file_size, 8);
				if(split_char != '#'){
					printf("error in UnpackConfigBakFile, invalid file format3, split_char = %c \n", split_char);
					res = ERR_FILE_TRANS;
					goto END;
				}
				file_size -= 9;

				bzero(filepath, kMaxPathLen);
				strcpy(filepath, PATH_PMC_DATA);

				fdd = open(filepath, O_WRONLY|O_CREAT|O_TRUNC);
				if(fdd == -1){
					printf("error in UnpackConfigBakFile, failed to create file:%s\n", filepath);
					res = ERR_FILE_TRANS;
					goto END;
				}

				//读取文件并写入配置
				while(0 < cfg_file_size){
					bzero(buffer, kTcpBufferSize);
					read_block = cfg_file_size > kTcpBufferSize?kTcpBufferSize:cfg_file_size;
					read_size = read(fd, buffer, read_block);
					if(read_size != read_block){
						printf("error in UnpackConfigBakFile, read file error:%hhu\n", i);
						res = ERR_FILE_TRANS;
						goto END;
					}

					write(fdd, buffer, read_size);  //写入文件
					cfg_file_size -= read_size;
					file_size -= read_size;

				}
				fsync(fdd);
				close(fdd);
				fdd = -1;
			}else if(i == CONFIG_MACRO_VAR && chn_count > 1){  //多通道时保存宏变量参数
				for(j = 1; j < chn_count; j++){
					if(file_size < 9){
						printf("error in UnpackConfigBakFile, invalid file format\n");
						res = ERR_FILE_TRANS;
						goto END;
					}
					//读取分隔符，及配置文件长度
					read(fd, &split_char, 1);
					read(fd, &cfg_file_size, 8);
					if(split_char != '#'){
						printf("error in UnpackConfigBakFile, invalid file format3, split_char = %c \n", split_char);
						res = ERR_FILE_TRANS;
						goto END;
					}
					file_size -= 9;

					bzero(filepath, kMaxPathLen);
					sprintf(filepath, PATH_MACRO_VAR_KEEP, j);

					fdd = open(filepath, O_WRONLY|O_CREAT|O_TRUNC);
					if(fdd == -1){
						printf("error in UnpackConfigBakFile, failed to create file:%s\n", filepath);
						res = ERR_FILE_TRANS;
						goto END;
					}

					//读取文件并写入配置
					while(0 < cfg_file_size){
						bzero(buffer, kTcpBufferSize);
						read_block = cfg_file_size > kTcpBufferSize?kTcpBufferSize:cfg_file_size;
						read_size = read(fd, buffer, read_block);
						if(read_size != read_block){
							printf("error in UnpackConfigBakFile, read file error:%hhu\n", i);
							res = ERR_FILE_TRANS;
							goto END;
						}

						write(fdd, buffer, read_size);  //写入文件
						cfg_file_size -= read_size;
						file_size -= read_size;

					}
					fsync(fdd);
					close(fdd);
					fdd = -1;
				}
			}
			this->m_p_channel_engine->SetParamImportMask(i);   //置位标志
		}
	}

	if(file_size != 0){
		printf("error in UnpackConfigBakFile, error file format, left bytes:%lld\n", file_size);
		res = ERR_FILE_TRANS;
	}




	END:
	if(fd > 0){
		close(fd);
		fd = -1;
	}
	if(fdd > 0){
		close(fdd);
		fdd = -1;
	}
	return res;
}



/**
 * @brief 接收文件
 * @return
 */
int HMICommunication::RecvFile(){
	int res = ERR_NONE;
	uint64_t file_size = 0;  //文件大小
	int read_size = 0;      //读取包的大小
	int read_block = 0;		//当前读取数据大小
	uint64_t read_total = 0;  //已读取的文件大小
	char buffer[kTcpBufferSize];	//缓冲
	char filepath[kMaxPathLen];		//文件路径
	char sign_path[kMaxPathLen] = {0};    //签名文件路径
	char file_sign[kNcFileSignLen] = {0};  //签名
	char file_name[kMaxFileNameLen] = {0};  //文件名
	int fd = -1;                    //文件句柄
	uint8_t file_type = FILE_NO_TYPE;  //文件传输类型
	uint8_t filename_size = 0;         //文件名长度
	int error_count = 0;              //出错计数
	char *pSplit = nullptr;
	int recv_count = 0;

	uint8_t update_file_count = 0;   //升级文件个数
	uint8_t module_type = 0;	//模块类型

	uint8_t err_count = 0;   //错误次数，连续三次失败则告警

	HMICmdFrame cmd;

	bzero((char *)&cmd, kMaxHmiCmdFrameLen);
	cmd.cmd = CMD_SC_TRANS_FILE_OVER;   //结果响应命令包
	cmd.data_len = 1;

//	FILE *file_bak = nullptr;


	//测试耗时
	struct timeval tvStart;
	struct timeval tvNow;
	unsigned int nTimeDelay = 0;

	gettimeofday(&tvStart, NULL);


	//读取传输类型
	read_size = recv(m_soc_file_send, (char *)&file_type, 1, MSG_NOSIGNAL);  //阻塞模式
//	if(read_size != 1){
//		printf("error in recvFile fun, failed to get file type, read = %d, error = %d\n", read_size, errno);
//		g_ptr_trace->PrintLog(LOG_ALARM, "接收文件类型异常！error = %d", errno);
//		res = ERR_FILE_TRANS;
//		goto END;
//	}
//	else
//		printf("file type %d\n", file_type);

	while(read_size != 1){
		printf("error in recvFile fun, failed to get file type, read_size = %d, error = %d\n",
				read_size, errno);

		if(++err_count > 3){
			g_ptr_trace->PrintLog(LOG_ALARM, "接收文件类型异常！error = %d, errcount=%hhu", errno, err_count);
			res = ERR_FILE_TRANS;
			goto END;
		}else{
			usleep(10000);   //休眠10ms
			read_size = recv(m_soc_file_send, (char *)&file_type, 1, MSG_NOSIGNAL);  //阻塞模式
		}
	}
	printf("file type %d\n", file_type);
	cmd.cmd_extension = file_type;


	if(file_type == FILE_G_CODE){
		//读取文件名称长度
		read_size = recv(m_soc_file_send, (char *)&filename_size, 1, MSG_NOSIGNAL);  //阻塞模式
		if(read_size != 1){
			printf("error in recvFile fun, failed to get file name length, read = %d, error = %d\n", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}
		else
			printf("file name length %d\n", filename_size);

		//读取文件名称
		bzero(buffer, kTcpBufferSize);
		read_size = recv(m_soc_file_send, buffer, filename_size, MSG_NOSIGNAL);  //阻塞模式
		if(read_size != filename_size){
			printf("error in recvFile fun, failed to get file name length, read = %d, error = %d\n", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}
		else
			printf("file name: %s\n", buffer);

		strcpy(file_name, buffer);
		cmd.data_len = strlen(buffer)+1;
		strcpy(cmd.data, buffer);

		bzero(filepath, kMaxPathLen);
		strcpy(filepath, PATH_NC_FILE);
		strcat(filepath, buffer);  //组合出文件绝对路径

		//组合签名文件绝对路径
		strcpy(sign_path, PATH_NC_SIGN_FILE);
		pSplit = strchr(buffer, '.');
		if(pSplit != nullptr)
			*pSplit = '\0';
		strcat(buffer, "_md5");
		strcat(sign_path, buffer);


		printf("file path = %s, signpath = %s\n", filepath, sign_path);

//		if(access(filepath, F_OK) == 0){	//文件存在则删除文件
//			if(!DeleteNcFile(filepath)){
//				printf("Failed to delete nc file[%s]\n", filepath);
//			}
//
//		}

		//读取文件签名
		read_size = recv(m_soc_file_send, (char *)&file_sign, kNcFileSignLen, MSG_NOSIGNAL);  //阻塞模式
		if(read_size != kNcFileSignLen){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "error in recvFile fun, failed to get file signature, read = %d, error = %d", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}
	}else if(file_type == FILE_UPDATE){//各模块升级文件
		//首先读取文件个数
		read_size = recv(m_soc_file_send, (char *)&update_file_count, 1, MSG_NOSIGNAL);  //阻塞模式
		if(read_size != 1){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "error in recvFile fun, failed to get update file count, read = %d, error = %d", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}
		printf("update file count: %hhu\n", update_file_count);

		//读取升级模块类型
		read_size = recv(m_soc_file_send, (char *)&module_type, 1, MSG_NOSIGNAL);  //阻塞模式
		if(read_size != 1){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "error in recvFile fun, failed to get update module type, read = %d, error = %d", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}

		printf("update module type : %hhu\n", module_type);

		//生成文件存储路径
		bzero(filepath, kMaxPathLen);
		if(!GetModuleUpdatePath(module_type, filepath)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "error in recvFile fun, invalid update module type, module_type = %hhu", module_type);
			res = ERR_FILE_TRANS;
			goto END;
		}


	}else if(file_type == FILE_PMC_LADDER){	//PMC梯图文件

	}else if(file_type == FILE_CONFIG_PACK){  //参数打包文件
		bzero(filepath, kMaxPathLen);
		strcpy(filepath, PATH_CONFIG_PACK_TMP);
	}else if(file_type == FILE_PC_DATA){  //螺补数据文件
		bzero(filepath, kMaxPathLen);
		strcpy(filepath, PATH_PC_DATA_TMP);
	}else if(file_type == FILE_ESB_DEV){  //伺服描述文件
		//读取文件名称长度
		read_size = recv(m_soc_file_send, (char *)&filename_size, 1, MSG_NOSIGNAL);  //阻塞模式
		if(read_size != 1){
			printf("error in recvFile fun, failed to get file name length, read = %d, error = %d\n", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}
		else
			printf("file name length %d\n", filename_size);

		//读取文件名称
		bzero(buffer, kTcpBufferSize);
		read_size = recv(m_soc_file_send, buffer, filename_size, MSG_NOSIGNAL);  //阻塞模式
		if(read_size != filename_size){
			printf("error in recvFile fun, failed to get file name length, read = %d, error = %d\n", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}
		else
			printf("file name: %s\n", buffer);

		strcpy(file_name, buffer);
		cmd.data_len = strlen(buffer)+1;
		strcpy(cmd.data, buffer);

		bzero(filepath, kMaxPathLen);
		strcpy(filepath, PATH_ESB_FILE);
		strcat(filepath, buffer);  //组合出文件绝对路径

		printf("ESB file path = %s\n", filepath);
    }else if (file_type == FILE_BACKUP_CNC) {
        m_sysbackup_status.m_status = SysUpdateStatus::FileTransing;
        SendHMIBackupStatus(m_sysbackup_status);
        bzero(filepath, kMaxPathLen);
        strcpy(filepath, RECOVER_FILE.c_str());
    }
//	else if(file_type == FILE_SYSTEM_CONFIG){
//
//	}else if(file_type == FILE_CHANNEL_CONFIG){
//
//	}else if(file_type == FILE_AXIS_CONFIG){
//
//	}else if(file_type == FILE_TOOL_CONFIG){
//
//	}

	READFILE:

	//读取文件大小
	read_size = recv(m_soc_file_send, (char *)&file_size, 8, MSG_NOSIGNAL);  //阻塞模式
	if(read_size != 8){
		printf("error in recvFile fun, failed to get file size, read = %d, error = %d\n", read_size, errno);
		res = ERR_FILE_TRANS;
		goto END;
	}
	else
		printf("file size %lld\n", file_size);

	//打开文件
	fd = open(filepath, O_WRONLY|O_CREAT|O_TRUNC);
//	file_bak = fopen("/cnc/nc_files/nc_bak.nc", "w+");
	if(fd == -1/* || file_bak == nullptr*/){
		g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "error in recvFile fun, failed to open file[%s]", filepath);
		res = ERR_FILE_TRANS;
		goto END;
	}

	read_total = 0;

	while(read_total < file_size){
		memset(buffer, 0x00, kTcpBufferSize);
		read_block = file_size-read_total;
		if(read_block > kTcpBufferSize-10)
			read_block = kTcpBufferSize-10;

		read_size = recv(m_soc_file_send, buffer, read_block, MSG_NOSIGNAL);
		if(read_size > 0){
			write(fd, buffer, read_size);

		//	fwrite(buffer, read_size, 1, file_bak);

			//printf("---------------------> recv total: %llu\n", read_total);
			read_total += read_size;
			error_count = 0;
		//	printf("read size :%lld\n", read_total);

			if(recv_count++ > 50){
				usleep(1000);
				recv_count = 0;
			}
		}
		else if(m_soc_file_send == -1){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "文件传输TCP连接异常中断！errno = %d", errno);
			break;
		}
		else if(error_count < 3){
			error_count++;
			usleep(5000);   //没读到数据等待5ms
		}
		else{
			printf("receive file length is zero: %d\n", read_size);
			break;
		}

	}

	gettimeofday(&tvNow, NULL);
	nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;
	printf("total time = %u us, filesize = %lld\n", nTimeDelay, file_size);

//	sync();
//	fclose(file_bak);
	close(fd);
	sync();

	if(read_total == file_size){
		if(file_type == FILE_G_CODE){
			//生成签名文件
			fd = open(sign_path, O_CREAT|O_WRONLY|O_TRUNC);
			if(fd > 0){
				write(fd, file_sign, kNcFileSignLen);
				close(fd);
				sync();
			}
			else
				g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "保存NC文件签名失败！");


			this->m_p_channel_engine->RemapFile(file_name);
		}
//		else if(file_type == FILE_MDA_CODE){
//			//收到完整MDA代码，启动运行程序
//			m_p_channel_engine->Start();
//		}


		printf("Succeed to receive file over, filesize = %llu\n", file_size);
	}
	else{
		g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "接收文件[类型%d]失败！已读取%lld字节, 总长%lld字节。", file_type, read_total, file_size);
		if(file_type == FILE_UPDATE)
			printf("update file module:%hhu\n", module_type);
		res = ERR_FILE_TRANS;
	}

	if(file_type == FILE_UPDATE && --update_file_count>0){//各模块升级文件
		//读取升级模块类型
		read_size = recv(m_soc_file_send, (char *)&module_type, 1, MSG_NOSIGNAL);  //阻塞模式
		if(read_size != 1){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "error in recvFile fun, failed to get update module type, read = %d, error = %d", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}

		printf("update module type : %hhu\n", module_type);

		//生成文件存储路径
		bzero(filepath, kMaxPathLen);
		if(!GetModuleUpdatePath(module_type, filepath)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "error in recvFile fun, invalid update module type, module_type = %hhu", module_type);
			res = ERR_FILE_TRANS;
			goto END;
		}
		goto READFILE;
	}

    if (file_type == FILE_BACKUP_CNC) {
        std::cout << "notify unpack background" << std::endl;
        m_background_type = Background_UnPack;
        sem_post(&m_sem_background);//通知执行后台程序
    }

	END:
#ifdef USES_TCP_FILE_TRANS_KEEP
	sync();
#else
	if(m_soc_file_send > 0){
		close(m_soc_file_send);
		m_soc_file_send = -1;
		sync();
		printf("close file trans socket!\n");
	}
#endif

	if(file_type == FILE_UPDATE && res == ERR_NONE){//接收升级文件结束，开始升级操作
		this->m_p_channel_engine->StartUpdateProcess();
	}else if(file_type == FILE_CONFIG_PACK && res ==ERR_NONE){  //配置打包文件，开始解压配置
		this->UnpackConfigBakFile();
	}else if(file_type == FILE_PC_DATA && res == ERR_NONE){//处理螺补导入文件数据
		this->m_p_channel_engine->ProcessPcDataImport();
	}

	if(res != ERR_NONE){
		cmd.data[cmd.data_len-1] = FAILED;
		res = ERR_NONE;
	}

//#ifdef USES_MODBUS_PROTOCAL
//	//发送回复包
//	int len = kHmiCmdHeadLen+cmd.data_len;
//	int rb = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&m_addr_file_trans, m_n_addr_len);
//	if(len != rb){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "发送文件传输指令完成指令失败[host:%s]！", inet_ntoa(m_addr_file_trans.sin_addr));
//	}else{
//		printf("send recv file over cmd:ip[%s:%hu]\n", inet_ntoa(m_addr_file_trans.sin_addr), ntohs(m_addr_file_trans.sin_port));
//	}
//#else
	this->SendCmd(cmd);   //发送响应
//#endif


	return res;
}

/**
 * @brief 获取模块升级文件路径
 * @param module_type : 模块类型
 * @param file_path[out] ： 文件路径
 * @return
 */
bool HMICommunication::GetModuleUpdatePath(uint8_t module_type, char *file_path){
	bool res = true;

	if(file_path == nullptr){
		res = false;
		goto END;
	}

	strcpy(file_path, PATH_UPDATE_PATH);
	switch(module_type){
	case MODULE_UPDATE_SC:
		strcat(file_path, "module_sc.elf");
		break;
	case MODULE_UPDATE_MC:
		strcat(file_path, "module_mc.ldr");
		break;
	case MODULE_UPDATE_PMC:
		strcat(file_path, "module_pmc.upd");
		break;
	case MODULE_UPDATE_MI:
		strcat(file_path, "module_mi.bin");
		break;
	case MODULE_UPDATE_PL:
		strcat(file_path, "module_pl.bin");
		break;
	case MODULE_UPDATE_FPGA:
		strcat(file_path, "module_spartan.bin");
		break;
	case MODULE_UPDATE_MODBUS:
		strcat(file_path, "module_modbus.elf");
		break;
	case MODULE_UPDATE_DISK:
		strcat(file_path, "module_disk.zip");
		break;
	default:
		g_ptr_trace->PrintTrace(TRACE_WARNING, HMI_COMMUNICATION, "不支持的模块类型[%hhu]", module_type);
		res = false;
		break;
	}
END:
	return res;
}

/**
 * @brief 获取NC加工文件个数
 * @return 文件个数
 */
int HMICommunication::GetNcFileCount(const char *path){
	int count = 0;

	DIR *dir = opendir(path);
	if(dir == nullptr){
	//	printf("Failed to open dir [%s]!\n", PATH_NC_FILE);
		g_ptr_trace->PrintLog(LOG_ALARM, "打开文件目录失败！[%s]", path);
        CreateError(ERR_OPEN_DIR, WARNING_LEVEL, CLEAR_BY_MCP_RESET, errno);
		return count;
	}

	struct dirent *dir_obj = nullptr;
	char *dir_name = nullptr;

	//循环读取目录下的文件
	while(nullptr != (dir_obj = readdir(dir))){
		dir_name = dir_obj->d_name;
//		printf("read file %s\n", dir_name);
		if(strcmp(dir_name, ".") == 0 || strcmp(dir_name, "..") == 0){
			continue;
		}
		else if(dir_obj->d_type == DT_REG){   //普通文件
//			printf("file[%s], size[%d], %d\n", dir_name, dir_obj->d_reclen, (int)dir_obj->d_off);
			count++;
		}
		else if(dir_obj->d_type == DT_DIR){
//			printf("dir[%s]\n", dir_name);

			count++;
		}
	}

	closedir(dir);

	return count;
}

/**
 * @brief 获取ESB文件数量
 * @param path : 路径
 * @return ESB文件个数
 */
int HMICommunication::GetEsbFileCount(const char *path){
	int count = 0;
	DIR *dir = opendir(path);
	if(dir == nullptr){
		//创建目录
		if(mkdir(PATH_ESB_FILE, 0755) == -1){//创建目录失败
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "创建目录失败！[%s]", PATH_ESB_FILE);
            CreateError(ERR_OPEN_DIR, WARNING_LEVEL, CLEAR_BY_MCP_RESET, errno);
		}else{
			g_ptr_trace->PrintTrace(TRACE_INFO, HMI_COMMUNICATION, "打开目录[%s]失败！自动创建改目录！", PATH_ESB_FILE);
		}
		return count;
	}

	struct dirent *dir_obj = nullptr;
	char *dir_name = nullptr, *pt = nullptr;
	int len = 0;

	//循环读取目录下的文件
	while(nullptr != (dir_obj = readdir(dir))){
		dir_name = dir_obj->d_name;

		if(dir_obj->d_type == DT_REG){   //普通文件
			//校验文件后缀是不是.esb
			len = strlen(dir_name);
			if(len > 4){
				pt = strrchr(dir_name, '.');
				if(pt && strcasecmp(pt, "esb")){  //后缀匹配
					count++;
				}
			}

		}
		else if(dir_obj->d_type == DT_DIR){

			continue;
		}
	}

	closedir(dir);
	return count;
}


/**
 * @brief 遍历获取path目录下文件的详细信息
 * @param path[in] : 目录
 * @param size[out] : 文件大小
 * @param time[out] : 文件最后修改时间
 * @param mode[out] : 文件属性
 * @return true--成功   false--失败
 */
bool HMICommunication::GetNcFileInfo(const char *path, uint64_t &size, char *time, mode_t *mode){

	struct stat statbuf;
	if(stat(path, &statbuf) == -1){
		g_ptr_trace->PrintLog(LOG_ALARM, "打开文件失败！[%s]", path);
        CreateError(ERR_OPEN_FILE, WARNING_LEVEL, CLEAR_BY_MCP_RESET, errno);
		return false;
	}

	if(S_ISDIR(statbuf.st_mode))
		return false;
	else if(S_ISREG(statbuf.st_mode)){
		size = statbuf.st_size;
		if(time != nullptr){
			struct tm *ptm = localtime(&statbuf.st_mtime);
			sprintf(time, "%04d/%02d/%02d %02d:%02d:%02d", ptm->tm_year+1900, ptm->tm_mon+1, ptm->tm_mday,
					ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
		}
		if(mode != nullptr){
			*mode = statbuf.st_mode;
		}
	}

	return true;
}

/**
 * @brief 处理文件传输异常
 */
void HMICommunication::ProcessFileTransError(){
	this->m_b_trans_file = false;  //复位文件传输标志
}

//@test zk
void get_memoccupy(MEM_OCCUPY *mem) //对无类型get函数含有一个形参结构体类弄的指针O
{
    FILE *fd;
    char buff[256];
    MEM_OCCUPY *m;
    m = mem;

    fd = fopen("/proc/meminfo", "r");
    //MemTotal: 515164 kB
    //MemFree: 7348 kB
    //Buffers: 7892 kB
    //Cached: 241852  kB
    //SwapCached: 0 kB
    //从fd文件中读取长度为buff的字符串再存到起始地址为buff这个空间里
    fgets(buff, sizeof(buff), fd);
    sscanf(buff, "%s %lu ", m->name1, &m->MemTotal);
    fgets(buff, sizeof(buff), fd);
    sscanf(buff, "%s %lu ", m->name2, &m->MemFree);
    fgets(buff, sizeof(buff), fd);
    sscanf(buff, "%s %lu ", m->name3, &m->Buffers);
    fgets(buff, sizeof(buff), fd);
    sscanf(buff, "%s %lu ", m->name4, &m->Cached);
    fgets(buff, sizeof(buff), fd);
    sscanf(buff, "%s %lu", m->name5, &m->SwapCached);

    fclose(fd);     //关闭文件fd
}


int get_cpuoccupy(CPU_OCCUPY *cpust) //对无类型get函数含有一个形参结构体类弄的指针O
{
    FILE *fd;
    char buff[256];
    CPU_OCCUPY *cpu_occupy;
    cpu_occupy = cpust;

    fd = fopen("/proc/stat", "r");
    fgets(buff, sizeof(buff), fd);

    sscanf(buff, "%s %u %u %u %u %u %u %u", cpu_occupy->name, &cpu_occupy->user, &cpu_occupy->nice, &cpu_occupy->system, &cpu_occupy->idle, &cpu_occupy->lowait, &cpu_occupy->irq, &cpu_occupy->softirq);

    fclose(fd);

    return 0;
}


void cal_cpuoccupy(CPU_OCCUPY *o, CPU_OCCUPY *n)
{
    unsigned long od, nd;
    double cpu_use = 0;

    od = (unsigned long)(o->user + o->nice + o->system + o->idle + o->lowait + o->irq + o->softirq);//第一次(用户+优先级+系统+空闲)的时间再赋给od
    nd = (unsigned long)(n->user + n->nice + n->system + n->idle + n->lowait + n->irq + n->softirq);//第二次(用户+优先级+系统+空闲)的时间再赋给od
    double sum = nd - od;
    double idle = n->idle - o->idle;
    cpu_use = idle / sum;
    idle = n->user + n->system + n->nice - o->user - o->system - o->nice;
    cpu_use = idle / sum;
    //printf("---cpu use---: %.3f\n",cpu_use);
}
//@test zk
