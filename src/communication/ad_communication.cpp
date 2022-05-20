/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file ad_communication.cpp
 *@author gonghao
 *@date 2020/11/03
 *@brief ��ͷ�ļ�Ϊ�����豸ͨѶ���ʵ��
 *@version
 */
#include <netinet/in.h>
#include <arpa/inet.h>
#include <dirent.h>

#include "ad_communication.h"
#include "channel_engine.h"
#include "global_include.h"


ADCommunication* ADCommunication::m_p_instance = nullptr;  //��ʼ����������ָ��Ϊ��
int ADCommunication::m_soc_udp_recv = -1;       //��ʼ��UDP��������׽���
CircularBuffer<HMICmdRecvNode> *ADCommunication::m_list_recv_ad = nullptr;  //��ʼ��udp������ջ���ָ��

/**
 * @brief ���캯��
 */
ADCommunication::ADCommunication() {
	// TODO Auto-generated constructor stub

	m_error_code = static_cast<ErrorType>(Initialize());
}


/**
 * @brief ��������
 */
ADCommunication::~ADCommunication() {
	// TODO Auto-generated destructor stub

	this->Clean();
}


/**
 * @brief ��ȡ��ʵ����Ψһ���ʽӿں���������ģʽ
 */
ADCommunication* ADCommunication::GetInstance(){
	if(NULL == m_p_instance){
		m_p_instance = new ADCommunication();
	}
	return m_p_instance;
}

/**
 * @brief ���ýӿ�ָ��
 * @param ��
 */
void ADCommunication::SetInterface(){
	this->m_p_channel_engine = ChannelEngine::GetInstance();
}

/**
 * @brief �����ƺ�����
 * @return
 */
int ADCommunication::Clean(){

	int res = ERR_NONE;
	void* thread_result;

	//�˳�������߳�
	res = pthread_cancel(m_thread_process_cmd);
	if (res != ERR_NONE) {
 //		printf("Cmd process thread cancel failed\n");
		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ�������߳��˳�ʧ�ܣ�");
	}

	usleep(1000);
	res = pthread_join(m_thread_process_cmd, &thread_result);//�ȴ�������߳��˳����
	if (res != ERR_NONE) {
 //		printf("Cmd process thread join failed\n");
		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿڵȴ�������߳��˳�ʧ�ܣ�");
	}
	m_thread_process_cmd = 0;

//	//�˳��ط��߳�
//	res = pthread_cancel(m_thread_resend_cmd);
//	if (res != ERR_NONE) {
// //		printf("Cmd resend thread cancel failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ������ط��߳��˳�ʧ�ܣ�");
//	}
//	usleep(1000);
//
//	res = pthread_join(m_thread_resend_cmd, &thread_result);//�ȴ��ط��߳��˳����
//	if (res != ERR_NONE) {
// //		printf("Cmd resend thread join failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿڵȴ������ط��߳��˳�ʧ�ܣ�");
//	}
//	m_thread_resend_cmd = 0;
//


	//�ر������׽���
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
 * @brief ��ʼ������
 * @return
 */
int ADCommunication::Initialize(){
	int res = ERR_NONE;

	pthread_attr_t attr;
	struct sched_param param;

	int flag = 0;

	this->m_n_frame_number = 0;

	this->m_p_channel_engine = ChannelEngine::GetInstance();   //��ʼ��ͨ������ָ��Ϊ��

	m_soc_udp_recv = socket(AF_INET, SOCK_DGRAM, 0);	//�����׽���
	m_soc_udp_send = socket(AF_INET, SOCK_DGRAM, 0);	//�����׽���

	if(m_soc_udp_recv == -1 || m_soc_udp_send == -1){
		g_ptr_trace->PrintTrace(TRACE_ERROR, AD_COMMUNICATION, "�����豸ͨѶ�ӿڴ����׽���ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}


	m_list_recv_ad = new CircularBuffer<HMICmdRecvNode>(kCmdRecvBufLen/2);  //��������������
//	m_list_send = new ListBuffer<HMICmdResendNode *>();   //��ʼ���ط���������
	if(m_list_recv_ad == nullptr/* || m_list_send == nullptr*/){
		g_ptr_trace->PrintTrace(TRACE_ERROR, AD_COMMUNICATION, "�����豸ͨѶ�ӿ�������ڴ����ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}

	//��ʼ��UDP���յ�ַ���˿�
	bzero(&m_addr_udp_recv, sizeof(m_addr_udp_recv));
	m_addr_udp_recv.sin_family = AF_INET;
	m_addr_udp_recv.sin_addr.s_addr = INADDR_ANY;
	m_addr_udp_recv.sin_port = htons(PORT_AD_UDP_CMD);

	bind(m_soc_udp_recv, (sockaddr *)&m_addr_udp_recv, sizeof(m_addr_udp_recv));  //��UDP������ն˿�


	//��ʼ��������
	pthread_mutex_init(&m_mutex_udp_send, nullptr);

	flag = fcntl(m_soc_udp_recv, F_GETFL, 0);
	if(flag < 0){
		g_ptr_trace->PrintTrace(TRACE_ERROR, AD_COMMUNICATION, "�����豸ͨѶ�ӿ������ȡUDP�ӿ�����ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}
	if(fcntl(m_soc_udp_recv, F_SETFL, flag|O_NONBLOCK) < 0){
		g_ptr_trace->PrintTrace(TRACE_ERROR, AD_COMMUNICATION, "�����豸ͨѶ�ӿ������ȡUDP�ӿ����÷�����ģʽʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}


	//�����߳�
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 39; //99;
	pthread_attr_setschedparam(&attr, &param);
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
//	if (res) {
//		g_ptr_trace->PrintTrace(TRACE_ERROR, AD_COMMUNICATION, "�����豸ͨѶ�ӿ�������߳������̼̳߳�ģʽʧ�ܣ�");
//		m_error_code = ERR_SC_INIT;
//		goto END;
//	}
	res = pthread_create(&m_thread_process_cmd, &attr,
			ADCommunication::UdpProcessCmdThread, this);    //����HMI������߳�
	if (res != 0) {
		g_ptr_trace->PrintTrace(TRACE_ERROR, AD_COMMUNICATION, "HMIͨѶ�ӿ�������߳�����ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}

//	pthread_attr_init(&attr);
//	pthread_attr_setschedpolicy(&attr, SCHED_RR);
//	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
//	param.__sched_priority = 36; //96;
//	pthread_attr_setschedparam(&attr, &param);
////	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
////	if (res) {
////		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ������ط��߳������̼̳߳�ģʽʧ�ܣ�");
////		m_error_code = ERR_SC_INIT;
////		goto END;
////	}
//	res = pthread_create(&m_thread_resend_cmd, &attr,
//			HMICommunication::UdpResendCmdThread, this);    //����HMI�����ط��߳�
//	if (res != 0) {
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ������ط��߳�����ʧ�ܣ�");
//		res = ERR_SC_INIT;
//		goto END;
//	}


END:
	pthread_attr_destroy(&attr);
	return res;
}


/**
 * @brief ����UDP����
 * @param cmd: �����͵������
 * @param addr: ���͵�Ŀ�ĵ�ַ
 * @return true--���ͳɹ�  false--����ʧ��
 */
bool ADCommunication::SendCmd(HMICmdFrame &cmd, sockaddr_in &addr){

	int res = 0;

	//ȷ��֡��
	if((cmd.frame_number & 0x8000) == 0){
		cmd.frame_number = this->GetFrameNumber();  //�ǻظ����ݰ����Զ�����֡��
	}

	//����UDP��
	int len = kHmiCmdHeadLen+cmd.data_len;
	res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&addr, sizeof(struct sockaddr));
	if(len != res){
		g_ptr_trace->PrintTrace(TRACE_WARNING, AD_COMMUNICATION, "����UDP�����ʧ�ܣ�Ӧ��%d�ֽڣ�ʵ��%d�ֽڣ�errno = %d��", len, res, errno);
		return false;
	}

	return true;
}

/**
 * @brief UDP����������̺߳���
 * @param void *args: ADCommunication����ָ��
 */
void *ADCommunication::UdpProcessCmdThread(void *args){
	printf("Start ADCommunication::UdpProcessCmdThread! id = %ld\n", syscall(SYS_gettid));
	ADCommunication *ad_comm = static_cast<ADCommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::UdpProcessCmdThread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::UdpProcessCmdThread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	while(!g_sys_state.system_ready) //�ȴ�ϵͳ׼����
			usleep(100000);  //�ȴ�100ms

	res = ad_comm->ProcessADCmd();

	printf("Quit ADCommunication::UdpProcessCmdThread!\n");
	pthread_exit(NULL);
}

/**
 * @brief AD�������
 * @param ��
 */
int ADCommunication::ProcessADCmd(){
	int res = ERR_NONE;
//	struct timespec time_out= {0, 10000000};   //�źŵȴ���ʱʱ��

	printf("~~~~start process ad cmd\n");

	HMICmdRecvNode cmd_node;
	HMICmdFrame &cmd = cmd_node.cmd;
	while(!g_sys_state.system_quit){

		//����udp����
		this->RecvADCmd();


		//����HMI������ն���
		while(m_list_recv_ad->ReadData(&cmd_node, 1) > 0){
//			if(g_sys_state.hmi_comm_ready && cmd.cmd != CMD_HMI_DEVICE_SCAN){  //������HMI��������IP�����ķ��豸ɨ��ָ��
//				if(strcmp(g_sys_state.hmi_host_addr, inet_ntoa(cmd_node.ip_addr.sin_addr)) != 0){
//					continue;  //
//				}
//			}


	//		printf("receive AD cmd[%04X %02X %02X %02X]\n", cmd.frame_number, cmd.cmd, cmd.cmd_extension,cmd.data_len);
			//��������д���
			switch(cmd.cmd){
			case CMD_AD_GET_MACRO_VAR:  //��ȡ�����ֵ
				ProcessGetMacroVarCmd(cmd, cmd_node.ip_addr);
				break;
			case CMD_AD_SET_MACRO_VAR:	//���ú����ֵ
				ProcessSetMacroVarCmd(cmd, cmd_node.ip_addr);
				break;
			case CMD_AD_GET_PMC_REG:	//��ȡPMC�Ĵ���
				this->ProcessGetPmcRegCmd(cmd, cmd_node.ip_addr);
				break;
			case CMD_AD_SET_PMC_REG:	//����PMC�Ĵ���
				this->ProcessSetPmcRegCmd(cmd, cmd_node.ip_addr);
				break;
			default:
				g_ptr_trace->PrintTrace(TRACE_WARNING, AD_COMMUNICATION, "�յ���֧�ֵ�ADָ��cmd=%d", cmd.cmd);
				break;
			}
		}

		usleep(20000);   //����20ms
	}


	return res;
}

/**
 * @brief ���ո����豸��UDP����
 */
void ADCommunication::RecvADCmd(){
	if(m_list_recv_ad->EmptyBufLen() == 0){  //���ջ�������
//		printf("WARNING��AD CMD buffer overflow!!!\n");
		g_ptr_trace->PrintTrace(TRACE_WARNING, AD_COMMUNICATION, "�����豸UDP���������");
		return;
	}


	HMICmdRecvNode cmd_node;
	HMICmdFrame &data = cmd_node.cmd;
	ssize_t res = 0;   //�������ݷ���ֵ
	socklen_t addr_len = sizeof(struct sockaddr);

	while(1){
		bzero((char *)&data, kMaxHmiCmdFrameLen);

		res = recvfrom(m_soc_udp_recv, &data, kMaxHmiCmdFrameLen, 0, (struct sockaddr *)&cmd_node.ip_addr, &addr_len);

//		if(res > 0)
//			printf("ad recv cmd[%hu], addr(%s:%hu)\n", data.cmd, inet_ntoa(cmd_node.ip_addr.sin_addr), ntohs(cmd_node.ip_addr.sin_port));

		if(res == -1 && errno == EAGAIN){	//�����ݿɶ�

			break;
		}else if (res == 0){
			printf("recvfrom error, errno = %d\n", errno);
			break;
		}

		m_list_recv_ad->WriteData(&cmd_node, 1);
	}
}


/**
 * @brief �����ȡ���������
 * @param cmd
 * @param addr
 */
void ADCommunication::ProcessGetMacroVarCmd(HMICmdFrame &cmd, sockaddr_in &addr){

//	printf("ProcessGetMacroVarCmd\n");
	double value = 0;
	bool init = false;

	uint8_t chn = cmd.channel_index;	//ͨ����
	uint32_t index = 0;			//�����������
	memcpy(&index, cmd.data, 4);

//	addr.sin_port = htons(PORT_AD_UDP_CMD);   //����˿�Ϊ9550

//	printf("ProcessGetMacroVarCmd chn= %hhu, addr(%s:%d)\n", chn, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
	if(chn == CHANNEL_ENGINE_INDEX)
		chn = 0;

	if(this->m_p_channel_engine->GetMacroVarValue(chn, index, init, value)){//�ɹ�
		cmd.cmd_extension = SUCCEED;
		cmd.data_len += sizeof(double) + 1;
		cmd.data[4] = init?1:0;
		memcpy(&cmd.data[5], &value, sizeof(double));

	//	printf("chn = %hhu, index = %u, value= %lf\n", chn, index, value);
	}else{//ʧ��
		cmd.cmd_extension = FAILED;
		printf("get failed\n");
	}


	cmd.frame_number |= 0x8000;
	this->SendCmd(cmd, addr);
//	printf("exit ProcessGetMacroVarCmd\n");

}


/**
 * @brief �������ú��������
 * @param cmd
 * @param addr
 */
void ADCommunication::ProcessSetMacroVarCmd(HMICmdFrame &cmd, sockaddr_in &addr){

	double value = 0;
	bool init = false;


	uint8_t chn = cmd.channel_index;	//ͨ����
	uint32_t index = 0;			//�����������
	memcpy(&index, cmd.data, 4);
	memcpy(&init, &cmd.data[4], 1);
	memcpy(&value, &cmd.data[5], sizeof(double));

//	addr.sin_port = htons(PORT_AD_UDP_CMD);   //����˿�Ϊ9550

	if(chn == CHANNEL_ENGINE_INDEX)
		chn = 0;

	if(this->m_p_channel_engine->SetMacroVarValue(chn, index, init, value)){//�ɹ�
		printf("set macro value: #%u = %lf\n", index, value);
		cmd.cmd_extension = SUCCEED;
		cmd.data_len = 4;
	}else{//ʧ��
		cmd.cmd_extension = FAILED;
		cmd.data_len = 4;
		printf("failed to set macro value, #%u\n", index);
	}

	cmd.frame_number |= 0x8000;
	this->SendCmd(cmd, addr);

}

/**
 * @brief �����ȡPMC�Ĵ�������
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
		case PMC_REG_A://���ֽڼĴ���
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
		case PMC_REG_DT://˫�ֽڼĴ���
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


//	addr.sin_port = htons(PORT_AD_UDP_CMD);   //����˿�Ϊ9550
	cmd.frame_number |= 0x8000;
	this->SendCmd(cmd, addr);
}

/**
 * @brief ��������PMC�Ĵ�������
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

	if(cmd.data_len == 5){ //���ֽ�
		memcpy(&value_8, &cmd.data[4], 1);
		res = this->m_p_channel_engine->SetPmcRegValue_8(reg_sec, reg_index, value_8);

	}else if(cmd.data_len == 6){//˫�ֽ�
		memcpy(&value_16, &cmd.data[4], 2);
		res = this->m_p_channel_engine->SetPmcRegValue_16(reg_sec, reg_index, value_16);
	}else{
		cmd.cmd_extension = FAILED;
	}
	if(res)
		cmd.cmd_extension = SUCCEED;
	else
		cmd.cmd_extension = FAILED;

//	addr.sin_port = htons(PORT_AD_UDP_CMD);   //����˿�Ϊ9550
	cmd.frame_number |= 0x8000;
	this->SendCmd(cmd, addr);

}
