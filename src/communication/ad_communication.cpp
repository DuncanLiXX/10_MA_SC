/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file ad_communication.cpp
 *@author gonghao
 *@date 2020/11/03
 *@brief 本头文件为辅助设备通讯类的实现
 *@version
 */
#include <netinet/in.h>
#include <arpa/inet.h>
#include <dirent.h>

#include "ad_communication.h"
#include "channel_engine.h"
#include "global_include.h"


ADCommunication* ADCommunication::m_p_instance = nullptr;  //初始化单例对象指正为空
int ADCommunication::m_soc_udp_recv = -1;       //初始化UDP命令接受套接字
CircularBuffer<HMICmdRecvNode> *ADCommunication::m_list_recv_ad = nullptr;  //初始化udp命令接收缓冲指针

/**
 * @brief 构造函数
 */
ADCommunication::ADCommunication() {
	// TODO Auto-generated constructor stub

	m_error_code = static_cast<ErrorType>(Initialize());
}


/**
 * @brief 析构函数
 */
ADCommunication::~ADCommunication() {
	// TODO Auto-generated destructor stub

	this->Clean();
}


/**
 * @brief 获取类实例的唯一访问接口函数，单例模式
 */
ADCommunication* ADCommunication::GetInstance(){
	if(NULL == m_p_instance){
		m_p_instance = new ADCommunication();
	}
	return m_p_instance;
}

/**
 * @brief 设置接口指针
 * @param 无
 */
void ADCommunication::SetInterface(){
	this->m_p_channel_engine = ChannelEngine::GetInstance();
}

/**
 * @brief 处理善后事宜
 * @return
 */
int ADCommunication::Clean(){

	int res = ERR_NONE;
	void* thread_result;

	//退出命令处理线程
	res = pthread_cancel(m_thread_process_cmd);
	if (res != ERR_NONE) {
 //		printf("Cmd process thread cancel failed\n");
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口命令处理线程退出失败！");
	}

	usleep(1000);
	res = pthread_join(m_thread_process_cmd, &thread_result);//等待命令处理线程退出完成
	if (res != ERR_NONE) {
 //		printf("Cmd process thread join failed\n");
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口等待命令处理线程退出失败！");
	}
	m_thread_process_cmd = 0;

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


	//关闭网络套接字
	close(m_soc_udp_recv);
	close(m_soc_udp_send);



	if(m_list_recv_ad){
		delete m_list_recv_ad;
		m_list_recv_ad = nullptr;
	}
//	if(m_list_send){
//		delete m_list_send;
//		m_list_send = nullptr;
//	}

	pthread_mutex_destroy(&m_mutex_udp_send);



	printf("exit ADCommunication::Clean()!\n");

	return res;
}


/**
 * @brief 初始化函数
 * @return
 */
int ADCommunication::Initialize(){
	int res = ERR_NONE;

	pthread_attr_t attr;
	struct sched_param param;

	int flag = 0;

	this->m_n_frame_number = 0;

	this->m_p_channel_engine = ChannelEngine::GetInstance();   //初始化通道引擎指针为空

	m_soc_udp_recv = socket(AF_INET, SOCK_DGRAM, 0);	//接收套接字
	m_soc_udp_send = socket(AF_INET, SOCK_DGRAM, 0);	//发送套接字

	if(m_soc_udp_recv == -1 || m_soc_udp_send == -1){
		g_ptr_trace->PrintTrace(TRACE_ERROR, AD_COMMUNICATION, "辅助设备通讯接口创建套接字失败！");
		res = ERR_SC_INIT;
		goto END;
	}


	m_list_recv_ad = new CircularBuffer<HMICmdRecvNode>(kCmdRecvBufLen/2);  //分配接收命令缓冲区
//	m_list_send = new ListBuffer<HMICmdResendNode *>();   //初始化重发命名链表
	if(m_list_recv_ad == nullptr/* || m_list_send == nullptr*/){
		g_ptr_trace->PrintTrace(TRACE_ERROR, AD_COMMUNICATION, "辅助设备通讯接口命令缓冲内存分配失败！");
		res = ERR_SC_INIT;
		goto END;
	}

	//初始化UDP接收地址及端口
	bzero(&m_addr_udp_recv, sizeof(m_addr_udp_recv));
	m_addr_udp_recv.sin_family = AF_INET;
	m_addr_udp_recv.sin_addr.s_addr = INADDR_ANY;
	m_addr_udp_recv.sin_port = htons(PORT_AD_UDP_CMD);

	bind(m_soc_udp_recv, (sockaddr *)&m_addr_udp_recv, sizeof(m_addr_udp_recv));  //绑定UDP命令接收端口


	//初始化互斥量
	pthread_mutex_init(&m_mutex_udp_send, nullptr);

	flag = fcntl(m_soc_udp_recv, F_GETFL, 0);
	if(flag < 0){
		g_ptr_trace->PrintTrace(TRACE_ERROR, AD_COMMUNICATION, "辅助设备通讯接口命令获取UDP接口属性失败！");
		res = ERR_SC_INIT;
		goto END;
	}
	if(fcntl(m_soc_udp_recv, F_SETFL, flag|O_NONBLOCK) < 0){
		g_ptr_trace->PrintTrace(TRACE_ERROR, AD_COMMUNICATION, "辅助设备通讯接口命令获取UDP接口设置非阻塞模式失败！");
		res = ERR_SC_INIT;
		goto END;
	}


	//开启线程
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 39; //99;
	pthread_attr_setschedparam(&attr, &param);
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
//	if (res) {
//		g_ptr_trace->PrintTrace(TRACE_ERROR, AD_COMMUNICATION, "辅助设备通讯接口命令处理线程设置线程继承模式失败！");
//		m_error_code = ERR_SC_INIT;
//		goto END;
//	}
	res = pthread_create(&m_thread_process_cmd, &attr,
			ADCommunication::UdpProcessCmdThread, this);    //开启HMI命令处理线程
	if (res != 0) {
		g_ptr_trace->PrintTrace(TRACE_ERROR, AD_COMMUNICATION, "HMI通讯接口命令处理线程启动失败！");
		res = ERR_SC_INIT;
		goto END;
	}

//	pthread_attr_init(&attr);
//	pthread_attr_setschedpolicy(&attr, SCHED_RR);
//	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
//	param.__sched_priority = 36; //96;
//	pthread_attr_setschedparam(&attr, &param);
////	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
////	if (res) {
////		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口命令重发线程设置线程继承模式失败！");
////		m_error_code = ERR_SC_INIT;
////		goto END;
////	}
//	res = pthread_create(&m_thread_resend_cmd, &attr,
//			HMICommunication::UdpResendCmdThread, this);    //开启HMI命令重发线程
//	if (res != 0) {
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMI通讯接口命令重发线程启动失败！");
//		res = ERR_SC_INIT;
//		goto END;
//	}


END:
	pthread_attr_destroy(&attr);
	return res;
}


/**
 * @brief 发送UDP命令
 * @param cmd: 待发送的命令包
 * @param addr: 发送的目的地址
 * @return true--发送成功  false--发送失败
 */
bool ADCommunication::SendCmd(HMICmdFrame &cmd, sockaddr_in &addr){

	int res = 0;

	//确认帧号
	if((cmd.frame_number & 0x8000) == 0){
		cmd.frame_number = this->GetFrameNumber();  //非回复数据包，自动赋予帧号
	}

	//发送UDP包
	int len = kHmiCmdHeadLen+cmd.data_len;
	res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&addr, sizeof(struct sockaddr));
	if(len != res){
		g_ptr_trace->PrintTrace(TRACE_WARNING, AD_COMMUNICATION, "发送UDP命令包失败，应发%d字节，实发%d字节，errno = %d！", len, res, errno);
		return false;
	}

	return true;
}

/**
 * @brief UDP接收命令处理线程函数
 * @param void *args: ADCommunication对象指针
 */
void *ADCommunication::UdpProcessCmdThread(void *args){
	printf("Start ADCommunication::UdpProcessCmdThread! id = %ld\n", syscall(SYS_gettid));
	ADCommunication *ad_comm = static_cast<ADCommunication *>(args);

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

	while(!g_sys_state.system_ready) //等待系统准备好
			usleep(100000);  //等待100ms

	res = ad_comm->ProcessADCmd();

	printf("Quit ADCommunication::UdpProcessCmdThread!\n");
	pthread_exit(NULL);
}

/**
 * @brief AD命令处理函数
 * @param 无
 */
int ADCommunication::ProcessADCmd(){
	int res = ERR_NONE;
//	struct timespec time_out= {0, 10000000};   //信号等待超时时间

	printf("~~~~start process ad cmd\n");

	HMICmdRecvNode cmd_node;
	HMICmdFrame &cmd = cmd_node.cmd;
	while(!g_sys_state.system_quit){

		//接收udp数据
		this->RecvADCmd();


		//遍历HMI命令接收队列
		while(m_list_recv_ad->ReadData(&cmd_node, 1) > 0){
//			if(g_sys_state.hmi_comm_ready && cmd.cmd != CMD_HMI_DEVICE_SCAN){  //已连接HMI后丢弃其它IP发来的非设备扫描指令
//				if(strcmp(g_sys_state.hmi_host_addr, inet_ntoa(cmd_node.ip_addr.sin_addr)) != 0){
//					continue;  //
//				}
//			}


	//		printf("receive AD cmd[%04X %02X %02X %02X]\n", cmd.frame_number, cmd.cmd, cmd.cmd_extension,cmd.data_len);
			//对命令进行处理
			switch(cmd.cmd){
			case CMD_AD_GET_MACRO_VAR:  //获取宏变量值
				ProcessGetMacroVarCmd(cmd, cmd_node.ip_addr);
				break;
			case CMD_AD_SET_MACRO_VAR:	//设置宏变量值
				ProcessSetMacroVarCmd(cmd, cmd_node.ip_addr);
				break;
			case CMD_AD_GET_PMC_REG:	//获取PMC寄存器
				this->ProcessGetPmcRegCmd(cmd, cmd_node.ip_addr);
				break;
			case CMD_AD_SET_PMC_REG:	//设置PMC寄存器
				this->ProcessSetPmcRegCmd(cmd, cmd_node.ip_addr);
				break;
			default:
				g_ptr_trace->PrintTrace(TRACE_WARNING, AD_COMMUNICATION, "收到不支持的AD指令cmd=%d", cmd.cmd);
				break;
			}
		}

		usleep(20000);   //休眠20ms
	}


	return res;
}

/**
 * @brief 接收辅助设备的UDP命令
 */
void ADCommunication::RecvADCmd(){
	if(m_list_recv_ad->EmptyBufLen() == 0){  //接收缓冲已满
//		printf("WARNING！AD CMD buffer overflow!!!\n");
		g_ptr_trace->PrintTrace(TRACE_WARNING, AD_COMMUNICATION, "辅助设备UDP命令缓冲满！");
		return;
	}


	HMICmdRecvNode cmd_node;
	HMICmdFrame &data = cmd_node.cmd;
	ssize_t res = 0;   //接收数据返回值
	socklen_t addr_len = sizeof(struct sockaddr);

	while(1){
		bzero((char *)&data, kMaxHmiCmdFrameLen);

		res = recvfrom(m_soc_udp_recv, &data, kMaxHmiCmdFrameLen, 0, (struct sockaddr *)&cmd_node.ip_addr, &addr_len);

//		if(res > 0)
//			printf("ad recv cmd[%hu], addr(%s:%hu)\n", data.cmd, inet_ntoa(cmd_node.ip_addr.sin_addr), ntohs(cmd_node.ip_addr.sin_port));

		if(res == -1 && errno == EAGAIN){	//无数据可读

			break;
		}else if (res == 0){
			printf("recvfrom error, errno = %d\n", errno);
			break;
		}

		m_list_recv_ad->WriteData(&cmd_node, 1);
	}
}


/**
 * @brief 处理获取宏变量命令
 * @param cmd
 * @param addr
 */
void ADCommunication::ProcessGetMacroVarCmd(HMICmdFrame &cmd, sockaddr_in &addr){

//	printf("ProcessGetMacroVarCmd\n");
	double value = 0;
	bool init = false;

	uint8_t chn = cmd.channel_index;	//通道号
	uint32_t index = 0;			//宏变量索引号
	memcpy(&index, cmd.data, 4);

//	addr.sin_port = htons(PORT_AD_UDP_CMD);   //重设端口为9550

//	printf("ProcessGetMacroVarCmd chn= %hhu, addr(%s:%d)\n", chn, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
	if(chn == CHANNEL_ENGINE_INDEX)
		chn = 0;

	if(this->m_p_channel_engine->GetMacroVarValue(chn, index, init, value)){//成功
		cmd.cmd_extension = SUCCEED;
		cmd.data_len += sizeof(double) + 1;
		cmd.data[4] = init?1:0;
		memcpy(&cmd.data[5], &value, sizeof(double));

	//	printf("chn = %hhu, index = %u, value= %lf\n", chn, index, value);
	}else{//失败
		cmd.cmd_extension = FAILED;
		printf("get failed\n");
	}


	cmd.frame_number |= 0x8000;
	this->SendCmd(cmd, addr);
//	printf("exit ProcessGetMacroVarCmd\n");

}


/**
 * @brief 处理设置宏变量命令
 * @param cmd
 * @param addr
 */
void ADCommunication::ProcessSetMacroVarCmd(HMICmdFrame &cmd, sockaddr_in &addr){

	double value = 0;
	bool init = false;


	uint8_t chn = cmd.channel_index;	//通道号
	uint32_t index = 0;			//宏变量索引号
	memcpy(&index, cmd.data, 4);
	memcpy(&init, &cmd.data[4], 1);
	memcpy(&value, &cmd.data[5], sizeof(double));

//	addr.sin_port = htons(PORT_AD_UDP_CMD);   //重设端口为9550

	if(chn == CHANNEL_ENGINE_INDEX)
		chn = 0;

	if(this->m_p_channel_engine->SetMacroVarValue(chn, index, init, value)){//成功
		printf("set macro value: #%u = %lf\n", index, value);
		cmd.cmd_extension = SUCCEED;
		cmd.data_len = 4;
	}else{//失败
		cmd.cmd_extension = FAILED;
		cmd.data_len = 4;
		printf("failed to set macro value, #%u\n", index);
	}

	cmd.frame_number |= 0x8000;
	this->SendCmd(cmd, addr);

}

/**
 * @brief 处理获取PMC寄存器命令
 * @param cmd
 * @param addr
 */
void ADCommunication::ProcessGetPmcRegCmd(HMICmdFrame &cmd, sockaddr_in &addr){
	bool res = false;
	if(cmd.data_len != 4){
		cmd.cmd_extension = FAILED;
	}else{
		uint16_t reg_sec = 0;
		uint16_t reg_index = 0;
		uint8_t value_8;
#ifndef USES_PMC_2_0
		uint16_t value_16;
#endif
		memcpy(&reg_sec, cmd.data, 2);
		memcpy(&reg_index, &cmd.data[2], 2);

		switch(reg_sec){
		case PMC_REG_X:
		case PMC_REG_Y:
		case PMC_REG_F:
		case PMC_REG_G:
		case PMC_REG_K:
		case PMC_REG_R:
		case PMC_REG_A://单字节寄存器
#ifdef USES_PMC_2_0
		case PMC_REG_D:
		case PMC_REG_C:
		case PMC_REG_T:
		case PMC_REG_E:
#endif
			res = this->m_p_channel_engine->GetPmcRegValue_8(reg_sec, reg_index, value_8);
			cmd.data_len = 5;
			memcpy(&cmd.data[4], &value_8, 1);
			break;
#ifndef USES_PMC_2_0
		case PMC_REG_D:
		case PMC_REG_C:
		case PMC_REG_T:
		case PMC_REG_DC:
		case PMC_REG_DT://双字节寄存器
			res = this->m_p_channel_engine->GetPmcRegValue_16(reg_sec, reg_index, value_16);
			cmd.data_len = 6;
			memcpy(&cmd.data[4], &value_16, 2);
			break;
#endif
		default:
			break;
		}
		if(res)
			cmd.cmd_extension = SUCCEED;
		else
			cmd.cmd_extension = FAILED;
	}


//	addr.sin_port = htons(PORT_AD_UDP_CMD);   //重设端口为9550
	cmd.frame_number |= 0x8000;
	this->SendCmd(cmd, addr);
}

/**
 * @brief 处理设置PMC寄存器命令
 * @param cmd
 * @param addr
 */
void ADCommunication::ProcessSetPmcRegCmd(HMICmdFrame &cmd, sockaddr_in &addr){
	bool res = false;
	uint16_t reg_sec = 0;
	uint16_t reg_index = 0;
	uint8_t value_8 = 0;
	uint16_t value_16 = 0;

	memcpy(&reg_sec, cmd.data, 2);
	memcpy(&reg_index, &cmd.data[2], 2);

	if(cmd.data_len == 5){ //单字节
		memcpy(&value_8, &cmd.data[4], 1);
		res = this->m_p_channel_engine->SetPmcRegValue_8(reg_sec, reg_index, value_8);

	}else if(cmd.data_len == 6){//双字节
		memcpy(&value_16, &cmd.data[4], 2);
		res = this->m_p_channel_engine->SetPmcRegValue_16(reg_sec, reg_index, value_16);
	}else{
		cmd.cmd_extension = FAILED;
	}
	if(res)
		cmd.cmd_extension = SUCCEED;
	else
		cmd.cmd_extension = FAILED;

//	addr.sin_port = htons(PORT_AD_UDP_CMD);   //重设端口为9550
	cmd.frame_number |= 0x8000;
	this->SendCmd(cmd, addr);

}
