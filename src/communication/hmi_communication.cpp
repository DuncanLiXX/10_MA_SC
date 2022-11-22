/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file HMICommunication.cpp
 *@author gonghao
 *@date 2020/03/19
 *@brief ���ļ�ΪHMIͨѶ���ʵ��
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

unsigned int HMICommunication::m_n_hmi_heartbeat = 0;      //��ʼ��HMI��������
HMICommunication* HMICommunication::m_p_instance = nullptr;  //��ʼ����������ָ��Ϊ��
int HMICommunication::m_soc_udp_recv = -1;       //��ʼ��UDP��������׽���
//int HMICommunication::m_soc_tcp_monitor = -1;    //��ʼ��TCP������ݼ����׽���
//int HMICommunication::m_soc_monitor_send = -1;  //��ʼ��������ݷ���socket
CircularBuffer<HMICmdRecvNode> *HMICommunication::m_list_recv = nullptr;  //��ʼ��udp������ջ���ָ��
pthread_mutex_t HMICommunication::m_mutex_udp_recv;    //��ʼ��udp������ܻ�����
sem_t HMICommunication::m_sem_udp_recv;   //��ʼ����̬�ź���

struct sockaddr_in HMICommunication::m_addr_udp_hmi;  //��ʼ��HMI��ַ�ṹ
socklen_t HMICommunication::m_n_addr_len;       //��ʼ��HMI��ַ�ṹ����
int HMICommunication::m_thread_run_flag = 0;   //�߳����б�־


/**
 * @brief HMIͨѶ�๹�캯��
 */
HMICommunication::HMICommunication() {
	// TODO Auto-generated constructor stub

	m_error_code = static_cast<ErrorType>(Initialize()); //��ʼ��

}


/**
 * @brief HMIͨѶ����������
 */
HMICommunication::~HMICommunication() {
	// TODO Auto-generated destructor stub

	Clean();

}

/**
 * @brief ��ȡ��ʵ����Ψһ���ʽӿں���������ģʽ
 */
HMICommunication* HMICommunication::GetInstance(){
	if(NULL == m_p_instance){
		m_p_instance = new HMICommunication();
	}
	return m_p_instance;
}

/**
 * @brief ��ʼ������
 * @return
 */
int HMICommunication::Initialize(){
	int res = ERR_NONE;
//	const int on= 1;
	//sigset_t zeromask, newmask,oldmask;    //�źż�

	sockaddr_in addr;
	int reuse = 1;    //��ַ�����ã���ֹSC�����˳��󣬶˿�TIME_WAIT״̬

	pthread_attr_t attr;
	struct sched_param param;
	struct timeval tcp_sock_timeout = {3, 0};  //3��  TCP����recv���ݳ�ʱʱ��
	int flag = 0;

	int KeepAliveProbes = 2;		//���Դ���
	int KeepAliveIntvl=1;			//���������ļ������λ����
	int KeepAliveTime = 10;			//���ӽ������ÿ�ʼ��������λ����

	this->m_n_hmi_heartbeat = 0;
	this->m_n_frame_number = 0;
	this->m_b_trans_file = false;
	this->m_file_type = FILE_NO_TYPE;
	this->m_b_recv_file = true;  //Ĭ�Ͻ����ļ�
	this->m_mask_config_pack = 0x00;

	this->m_p_channel_engine = nullptr;   //��ʼ��ͨ������ָ��Ϊ��

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
		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿڴ����׽���ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}

	//����TCP������������
	setsockopt(m_soc_tcp_monitor, IPPROTO_TCP, TCP_KEEPCNT, (void *)&KeepAliveProbes, sizeof(KeepAliveProbes));
	setsockopt(m_soc_tcp_monitor, IPPROTO_TCP, TCP_KEEPIDLE, (void *)&KeepAliveTime, sizeof(KeepAliveTime));
	setsockopt(m_soc_tcp_monitor, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&KeepAliveIntvl, sizeof(KeepAliveIntvl));
	setsockopt(m_soc_tcp_file, IPPROTO_TCP, TCP_KEEPCNT, (void *)&KeepAliveProbes, sizeof(KeepAliveProbes));
	setsockopt(m_soc_tcp_file, IPPROTO_TCP, TCP_KEEPIDLE, (void *)&KeepAliveTime, sizeof(KeepAliveTime));
	setsockopt(m_soc_tcp_file, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&KeepAliveIntvl, sizeof(KeepAliveIntvl));


	//���ü������accept������ʱʱ��
	if( 0 != setsockopt(m_soc_tcp_monitor, SOL_SOCKET, SO_RCVTIMEO, (char *)&tcp_sock_timeout, sizeof(timeval))){
		g_ptr_trace->PrintLog(LOG_ALARM, "����TCP����׽��ֳ�ʱʱ��ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}
	if( 0 != setsockopt(m_soc_tcp_file, SOL_SOCKET, SO_RCVTIMEO, (char *)&tcp_sock_timeout, sizeof(timeval))){
		g_ptr_trace->PrintLog(LOG_ALARM, "����TCP�ļ������׽��ֳ�ʱʱ��ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}

	//����TCP����Ϊ������
	if(0 != setsockopt(m_soc_tcp_monitor, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse))){
		g_ptr_trace->PrintLog(LOG_ALARM, "����TCP����׽��ֿ���������ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}
	if(0 != setsockopt(m_soc_tcp_file, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse))){
		g_ptr_trace->PrintLog(LOG_ALARM, "����TCP�ļ������׽��ֿ���������ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}


	m_list_recv = new CircularBuffer<HMICmdRecvNode>(kCmdRecvBufLen);  //��������������
	m_list_send = new ListBuffer<HMICmdResendNode *>();   //��ʼ���ط���������
	if(m_list_recv == nullptr || m_list_send == nullptr){
//		printf("ERROR! List buffer init failed!\n");
		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ�������ڴ����ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}

	//��ʼ��UDP���յ�ַ���˿�
	bzero(&m_addr_udp_recv, sizeof(m_addr_udp_recv));
	m_addr_udp_recv.sin_family = AF_INET;
	m_addr_udp_recv.sin_addr.s_addr = INADDR_ANY;
	m_addr_udp_recv.sin_port = htons(PORT_UDP_CMD);

	bind(m_soc_udp_recv, (sockaddr *)&m_addr_udp_recv, sizeof(m_addr_udp_recv));  //��UDP������ն˿�

	//��ʼ��HMI�˵�ַ
	bzero(&m_addr_udp_hmi, sizeof(m_addr_udp_hmi));
	m_addr_udp_hmi.sin_family = AF_INET;
	this->m_n_addr_len = sizeof(struct sockaddr);

//#ifdef USES_MODBUS_PROTOCAL
//	//��ʼ��modbusЭ���µ��ļ�����IP
//	bzero(&m_addr_file_trans, sizeof(m_addr_file_trans));
//	m_addr_file_trans.sin_family = AF_INET;
//#endif


	bzero(&addr, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(PORT_TCP_MONITOR);
	bind(m_soc_tcp_monitor, (struct sockaddr *)&addr, sizeof(sockaddr));   //��monitro TCP�����˿�

	bzero(&addr, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(PORT_TCP_FILE);
	bind(m_soc_tcp_file, (struct sockaddr *)&addr, sizeof(sockaddr));   //���ļ����� TCP�����˿�

	//��ʼ��������
	pthread_mutex_init(&m_mutex_udp_recv, nullptr);
	pthread_mutex_init(&m_mutex_udp_send, nullptr);
	pthread_mutex_init(&m_mutex_tcp_monitor, nullptr);



	//ע��SIGIO�źŴ�����
//	struct sigaction act;
//	act.sa_handler = HMICommunication::SigioHandler;
//	act.sa_flags = SA_RESTART;
//	sigaction(SIGIO, &act, nullptr);

//	fcntl(m_soc_udp_recv, F_SETOWN, getpid());   //����socket����
//	ioctl(m_soc_udp_recv, FIOASYNC, &on);        //����Ϊ�첽��������ʽ
//	ioctl(m_soc_udp_recv, FIONBIO, &on);         //��ЧSIGIO�ź�

	flag = fcntl(m_soc_udp_recv, F_GETFL, 0);
	if(flag < 0){
		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ������ȡUDP�ӿ�����ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}
	if(fcntl(m_soc_udp_recv, F_SETFL, flag|O_NONBLOCK) < 0){
		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ������ȡUDP�ӿ����÷�����ģʽʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}


	//��ʼ���ź���
	sem_init(&m_sem_udp_recv, 0, 0);
	sem_init(&m_sem_tcp_file, 0, 0);
//	sem_init(&m_sem_tcp_send_test, 0, 0);
    sem_init(&m_sem_background, 0, 0);

	this->m_thread_saveas = 0;  //���Ϊ�̳߳�ʼ��Ϊ0

	//�����߳�
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 39; //99;
	pthread_attr_setschedparam(&attr, &param);
	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
	if (res) {
		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ�������߳������̼̳߳�ģʽʧ�ܣ�");
		m_error_code = ERR_SC_INIT;
		goto END;
	}
	res = pthread_create(&m_thread_process_cmd, &attr,
			HMICommunication::UdpProcessCmdThread, this);    //����HMI������߳�
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ�������߳�����ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}

	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 36; //96;
	pthread_attr_setschedparam(&attr, &param);
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
//	if (res) {
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ������ط��߳������̼̳߳�ģʽʧ�ܣ�");
//		m_error_code = ERR_SC_INIT;
//		goto END;
//	}
	res = pthread_create(&m_thread_resend_cmd, &attr,
			HMICommunication::UdpResendCmdThread, this);    //����HMI�����ط��߳�
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ������ط��߳�����ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}

	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 38; //98;
	pthread_attr_setschedparam(&attr, &param);
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
//	if (res) {
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ����ݼ���߳������̼̳߳�ģʽʧ�ܣ�");
//		m_error_code = ERR_SC_INIT;
//		goto END;
//	}
	res = pthread_create(&m_thread_monitor, &attr,
			HMICommunication::TcpMonitorThread, this);    //��������ˢ���߳�
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ����ݼ���߳�����ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}

	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 30; //90;
	pthread_attr_setschedparam(&attr, &param);
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
//	if (res) {
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ��ļ������߳������̼̳߳�ģʽʧ�ܣ�");
//		m_error_code = ERR_SC_INIT;
//		goto END;
//	}
    res = pthread_create(&m_thread_trans_file, &attr,
			HMICommunication::TcpTransFileThread, this);    //�����ļ������߳�
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ��ļ������߳�����ʧ�ܣ�");
		res = ERR_SC_INIT;
		goto END;
	}

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//
    param.__sched_priority = 29; //90;
    pthread_attr_setschedparam(&attr, &param);
    res = pthread_create(&m_thread_background, &attr,
            HMICommunication::BackgroundThread, this);    //�����ļ������߳�
    if (res != 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ��ļ������߳�����ʧ�ܣ�");
        res = ERR_SC_INIT;
        goto END;
    }
END:
	pthread_attr_destroy(&attr);
	return res;
}

/**
 * @brief ���ýӿ�ָ��
 * @param ��
 */
void HMICommunication::SetInterface(){
	this->m_p_channel_engine = ChannelEngine::GetInstance();
}

/**
 * @brief ִ�и�λ
 */
void HMICommunication::Reset(){

	this->m_error_code = ERR_NONE;

	//��������
	pthread_mutex_lock(&m_mutex_udp_send);
	m_list_send->Clear();	//����ط�����
	pthread_mutex_unlock(&m_mutex_udp_send);
	m_list_recv->ClearBuf();	//��ս����������

	//ȡ����ǰ���ļ�����
	if(m_b_trans_file){

	}
}

/**
 * @brief �����ƺ�����
 * @return
 */
int HMICommunication::Clean(){

	int res = ERR_NONE;
//	void* thread_result;

//	//�˳�������߳�
//	res = pthread_cancel(m_thread_process_cmd);
//	if (res != ERR_NONE) {
// //		printf("Cmd process thread cancel failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ�������߳��˳�ʧ�ܣ�");
//	}
//
//	usleep(1000);
//	res = pthread_join(m_thread_process_cmd, &thread_result);//�ȴ�������߳��˳����
//	if (res != ERR_NONE) {
// //		printf("Cmd process thread join failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿڵȴ�������߳��˳�ʧ�ܣ�");
//	}
//	m_thread_process_cmd = 0;
//
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
//	//�˳�����ˢ���߳�
//	res = pthread_cancel(m_thread_monitor);
//	if (res != ERR_NONE) {
// //		printf("Monitor thread cancel failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ����ݼ���߳��˳�ʧ�ܣ�");
//	}
//	usleep(1000);
//	res = pthread_join(m_thread_monitor, &thread_result);//�ȴ�����ˢ���߳��˳����
//	if (res != ERR_NONE) {
// //		printf("Monitor thread join failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿڵȴ����ݼ���߳��˳�ʧ�ܣ�");
//	}
//	m_thread_monitor = 0;
//
//	//�˳��ļ������߳�
//	res = pthread_cancel(m_thread_trans_file);
//	if (res != ERR_NONE) {
// //		printf("File trans thread cancel failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ��ļ������߳��˳�ʧ�ܣ�");
//	}
//	usleep(1000);
//	res = pthread_join(m_thread_trans_file, &thread_result);//�ȴ��ļ������߳� �˳����
//	if (res != ERR_NONE) {
//		printf("File trans thread join failed\n");
//		g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿڵȴ��ļ������߳��˳�ʧ�ܣ�");
//	}
//	m_thread_trans_file = 0;
//
//
//	if(m_thread_saveas != 0){
//		res = pthread_cancel(m_thread_saveas);
//		if (res != ERR_NONE) {
//	//		printf("Monitor thread cancel failed\n");
//			g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ��ļ����Ϊ�߳��˳�ʧ�ܣ�");
//		}
//		usleep(1000);
//		res = pthread_join(m_thread_saveas, &thread_result);//�ȴ��ļ����Ϊ�߳� �˳����
//		if (res != ERR_NONE) {
//	//		printf("File save as thread join failed\n");
//			g_ptr_trace->PrintLog(LOG_ALARM, "HMIͨѶ�ӿ��ļ����Ϊ�߳��˳�ʧ�ܣ�");
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


	//�ر������׽���
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
 * @brief ����UDP����
 * @param HMICmdFrame &cmd: �����͵������
 * @return true--���ͳɹ�  false--����ʧ��
 */
bool HMICommunication::SendCmd(HMICmdFrame &cmd){

	int res = 0;

	if(!g_sys_state.hmi_comm_ready)  //HMI������δ׼����
		return false;

	//ȷ��֡��
	if((cmd.frame_number & 0x8000) == 0){
		cmd.frame_number = this->GetFrameNumber();  //�ǻظ����ݰ����Զ�����֡��
	}

	//����UDP��
	int len = kHmiCmdHeadLen+cmd.data_len;
	res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&m_addr_udp_hmi, m_n_addr_len);
	if(len != res){
		//printf("ERROR! Failed to send udp cmd to hmi!%d, %d\n", len,res);
		g_ptr_trace->PrintLog(LOG_ALARM, "����HMI�����ʧ�ܣ�Ӧ��%d�ֽڣ�ʵ��%d�ֽڣ�errno = %d��", len, res, errno);
//		return false;
	}

    //��UDP���������Ӧ�����⣩�����ط�����
	if((cmd.frame_number & 0x8000) == 0){//����Ӧ��
		HMICmdResendNode *pNode = new HMICmdResendNode();
		if(pNode != NULL){
			pNode->frame = cmd;
			pNode->resend_count = kHmiCmdResend;
			pNode->timeout = kHmiCmdTimeout;

			m_list_send->Append(pNode);  //�˴�ֻ����ӣ����Բ���m_mutex_udp_send���⣬��Ϊ�ڲ��ѻ���
//			if(ListNode<HMICmdResendNode *>::new_count > 0)
//				printf("HMICmdResendNode new count :%d\n", ListNode<HMICmdResendNode *>::new_count);
		}
		
	}


	return true;
}

/**
 * @brief ��ԴIP��������
 * @param cmd_node :  �������ݽڵ�
 * @return  true--���ͳɹ�  false--����ʧ��
 */
bool HMICommunication::SendCmdToIp(HMICmdRecvNode &cmd_node){
	int res = 0;

	if(!g_sys_state.hmi_comm_ready)  //HMI������δ׼����
		return false;

	HMICmdFrame &cmd = cmd_node.cmd;
	//ȷ��֡��
	if((cmd.frame_number & 0x8000) == 0){
		cmd.frame_number = this->GetFrameNumber();  //�ǻظ����ݰ����Զ�����֡��
	}

//#ifdef USES_MODBUS_PROTOCAL
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //���Ķ˿�
//#endif
	cmd_node.ip_addr.sin_port = this->m_addr_udp_hmi.sin_port;  //���Ķ˿�

	//����UDP��
	int len = kHmiCmdHeadLen+cmd.data_len;
	res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
	if(len != res){
		//printf("ERROR! Failed to send udp cmd to hmi!%d, %d\n", len,res);
		g_ptr_trace->PrintLog(LOG_ALARM, "����HMI�����ʧ�ܣ�Ӧ��%d�ֽڣ�ʵ��%d�ֽڣ�errno = %d��", len, res, errno);
//		return false;
	}

#ifndef USES_MODBUS_PROTOCAL
    //��UDP���������Ӧ�����⣩�����ط�����
	if((cmd.frame_number & 0x8000) == 0){//����Ӧ��
		HMICmdResendNode *pNode = new HMICmdResendNode();
		if(pNode != NULL){
			pNode->frame = cmd;
			pNode->resend_count = kHmiCmdResend;
			pNode->timeout = kHmiCmdTimeout;

			m_list_send->Append(pNode);  //�˴�ֻ����ӣ����Բ���m_mutex_udp_send���⣬��Ϊ�ڲ��ѻ���
//			if(ListNode<HMICmdResendNode *>::new_count > 0)
//				printf("HMICmdResendNode new count :%d\n", ListNode<HMICmdResendNode *>::new_count);
		}

	}
#endif

	return true;
}

/**
 * @brief ���ͼ������
 * @param buf �����������ݻ���
 * @param size �����������ֽ���
 * @return �ɹ������ֽ���
 */
int HMICommunication::SendMonitorData(char *buf, int size){
	if(this->m_soc_monitor_send == -1 || buf == nullptr)
		return 0;

	pthread_mutex_lock(&m_mutex_tcp_monitor);

	int res = send(m_soc_monitor_send, buf, size, MSG_NOSIGNAL);

	if(res == -1){
		if(errno == 32 || errno == 104){
			g_ptr_trace->PrintLog(LOG_ALARM, "HMI��������жϣ�errno=%d", errno);
		//	CreateError(ERR_HMI_MONITOR_DISCON, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON);  //����HMI��ض����澯
		}
	}else if(res != size){
//		printf("warnning: send = %d, res = %d\n", size, res);
		g_ptr_trace->PrintLog(LOG_ALARM, "HMI������ӷ��������쳣[send=%d, res=%d]��errno=%d", size, res, errno);
	}


	pthread_mutex_unlock(&m_mutex_tcp_monitor);
	return res;
}

/**
 * @brief �ط�UDP���������������뷢�Ͷ���
 * @param HMICmdFrame &cmd: �����͵������
 * @return ʵ�ʷ����ֽ���
 */
int HMICommunication::ResendCmd(HMICmdFrame &cmd){
	int res = 0;

	if(!g_sys_state.hmi_comm_ready)  //HMI������δ׼����
		return res;

	//����UPD��
	int len = kHmiCmdHeadLen+cmd.data_len;
	res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&m_addr_udp_hmi, m_n_addr_len);
	if(len != res){
	//	printf("ERROR! Failed to resend udp cmd to hmi!%d, %d\n", len,res);
		g_ptr_trace->PrintLog(LOG_ALARM, "�ط�HMI�����ʧ�ܣ�Ӧ��%d�ֽڣ�ʵ��%d�ֽڣ�errno = %d", len, res, errno);
	}

	return res;
}

/**
 * @brief SIGIO�źŴ�����
 * @param int signo: �յ����ź�
 */
//void HMICommunication::SigioHandler(int signo) {
//
//	if(signo != SIGIO){
//		printf("other signal %d:%d\n", signo, SIGIO);
//		return;
//	}
//
//	static int sigio_mutex = 0;   //��ֹǶ��
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
//	if(m_list_recv->EmptyBufLen() == 0){  //���ջ�������
//		printf("ERROR��HMI CMD buffer overflow!!!\n");
////		pthread_mutex_unlock(&m_mutex_udp_recv);
//		sigio_mutex = 0;
//		return;
//	}
//
////	ssize_t read_len = 0;   //ʵ�ʶ�ȡ�ֽ���
//
//	HMICmdRecvNode cmd_node;
//	HMICmdFrame &data = cmd_node.cmd;
//	ssize_t res = 0;   //�������ݷ���ֵ
//
//	while(1){
//		bzero((char *)&data, kMaxHmiCmdFrameLen);
//
//		res = recvfrom(m_soc_udp_recv, &data, kMaxHmiCmdFrameLen, 0, (struct sockaddr *)&cmd_node.ip_addr, &m_n_addr_len);
//
//		if(res == -1 && errno == EAGAIN){	//�����ݿɶ�
//			break;
//		}else if (res == 0){
//			printf("recvfrom error, errno = %d\n", errno);
//			break;
//		}
//
//
//		if(data.cmd == CMD_HMI_HEART_BEAT){  //�ڴ˴�������������֤���ȴ���������Ϊ�����ʱ�������������ʧ
//			if(g_sys_state.hmi_comm_ready){
//				if(strcmp(g_sys_state.hmi_host_addr, inet_ntoa(cmd_node.ip_addr.sin_addr)) == 0){
//		//			printf("get hmi heartbeat cmd :%d\n", m_n_hmi_heartbeat);
//					m_n_hmi_heartbeat = 0;  //������������
//		//			sigio_mutex = 0;
//		//			pthread_mutex_unlock(&m_mutex_udp_recv);
//					continue;
//				}
//			}
//			else{//δ����״̬������������
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
//	//�����ź�
//	if(m_list_recv->BufLen() > 0)
//		sem_post(&m_sem_udp_recv);
//}

/**
 * @brief HMI�����ط�����, ÿ100ms��ѯһ��
 * @param ��
 */
int HMICommunication::ResendHmiCmd(){
	int res = ERR_NONE;

	ListNode<HMICmdResendNode *> *pNode = nullptr, *del_node = nullptr;
	while(!g_sys_state.system_quit){
		if(g_sys_state.eth0_running && g_sys_state.hmi_comm_ready){
			//����HMI����Ͷ���
			if(!m_list_send->IsEmpty()){
				pthread_mutex_lock(&m_mutex_udp_send);
				pNode = m_list_send->HeadNode();
				while(pNode != NULL){
					if( 0 == --(pNode->data->timeout) ){//���ճ�ʱ
						if(pNode->data->resend_count == 0){
							//TODO �Ѿ���������ط��������˴���Ҫ���Ӹ澯����
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

		usleep(100000);  //����100ms
	}



	return res;
}

/**
 * @brief ����HMI��UDP����
 */
void HMICommunication::RecvHmiCmd(){
	if(m_list_recv->EmptyBufLen() == 0){  //���ջ�������
		printf("WARNING��HMI CMD buffer overflow!!!\n");
		return;
	}

//	static struct timeval tvLast;
//	static	struct timeval tvNow;
//	unsigned int nTimeDelay = 0;

//	ssize_t read_len = 0;   //ʵ�ʶ�ȡ�ֽ���

	HMICmdRecvNode cmd_node;
	HMICmdFrame &data = cmd_node.cmd;
	ssize_t res = 0;   //�������ݷ���ֵ

	while(1){

		if(m_list_recv->EmptyBufLen() == 0){  //���ջ�������
			break;
		}

		bzero((char *)&data, kMaxHmiCmdFrameLen);

		res = recvfrom(m_soc_udp_recv, &data, kMaxHmiCmdFrameLen, 0, (struct sockaddr *)&cmd_node.ip_addr, &m_n_addr_len);

		if(res == -1 && errno == EAGAIN){	//�����ݿɶ�

			break;
		}else if (res == 0){
			printf("recvfrom error, errno = %d\n", errno);
			break;
		}



        if(data.cmd == CMD_HMI_HEART_BEAT){  //�ڴ˴�������������֤���ȴ���������Ϊ�����ʱ�������������ʧ

			if(g_sys_state.hmi_comm_ready){
				if(strcmp(g_sys_state.hmi_host_addr, inet_ntoa(cmd_node.ip_addr.sin_addr)) == 0){

					m_n_hmi_heartbeat = 0;  //������������

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
			else{//δ����״̬������������
				printf("no hmi host connected, drop the heartbeat package!\n");

				continue;
			}
		}

		if(1 != m_list_recv->WriteData(&cmd_node, 1)){
			g_ptr_trace->PrintLog(LOG_ALARM, "HMI�������������");
			break;
		}

		if(m_list_recv->EmptyBufLen() == 0){  //���ջ�������
			break;
		}
	}
}

/**
 * @brief HMI�������
 * @param ��
 */
int HMICommunication::ProcessHmiCmd(){
	int res = ERR_NONE;
//	struct timespec time_out= {0, 10000000};   //�źŵȴ���ʱʱ��


	HMICmdRecvNode cmd_node;
	HMICmdFrame &cmd = cmd_node.cmd;
	uint16_t frame_index = 0;
	while(!g_sys_state.system_quit){
//		sem_wait(&m_sem_udp_recv);  //�ȴ�UDP������ź�
//		if(0 != sem_timedwait(&m_sem_udp_recv, &time_out) && errno == ETIMEDOUT){  //�ȴ�UDP������ź�, sem_timedwait����������
//	//		usleep(1000);
//			continue;
//		}

		gettimeofday(&tv_hmi_cmd1, NULL);

		int nTimeDelay = (tv_hmi_cmd1.tv_sec-tv_hmi_cmd2.tv_sec)*1000000+tv_hmi_cmd1.tv_usec-tv_hmi_cmd2.tv_usec;
		if(nTimeDelay > 100000){
			printf("receive hmi cmd out; %d\n", nTimeDelay);
		}

		//����udp����
        this->RecvHmiCmd();
		tv_hmi_cmd2 = tv_hmi_cmd1;

		//����HMI������ն���
		while(m_list_recv->ReadData(&cmd_node, 1) > 0){
//			if(g_sys_state.hmi_comm_ready && cmd.cmd != CMD_HMI_DEVICE_SCAN){  //������HMI��������IP�����ķ��豸ɨ��ָ��
//#ifdef USES_MODBUS_PROTOCAL
//				if(cmd.cmd != CMD_HMI_GET_FILE && cmd.cmd != CMD_HMI_SEND_FILE && cmd.cmd != CMD_HMI_NC_FILE_COUNT &&
//						cmd.cmd != CMD_HMI_NC_FILE_INFO && cmd.cmd != CMD_HMI_FILE_OPERATE && cmd.cmd != CMD_HMI_UPDATE){   //ModbusЭ���£�����HMIֱ�������ļ�
//#endif
//				if(strcmp(g_sys_state.hmi_host_addr, inet_ntoa(cmd_node.ip_addr.sin_addr)) != 0){
//					continue;  //
//				}
//#ifdef USES_MODBUS_PROTOCAL
//				}
//#endif
//			}

			//�������Ӧ���ģ����ڷ��Ͷ�����ɾ����Ӧ����
			if((cmd.frame_number & 0x8000) != 0){
				frame_index = (cmd.frame_number & 0x7FFF);
				this->DelCmd(frame_index);
			}

//			printf("receive hmi cmd[%04X %02X %02X %02X %s]\n", cmd.frame_number, cmd.cmd, cmd.cmd_extension,cmd.data_len, cmd.data);
			//��������д���
			switch(cmd.cmd){
//			case CMD_HMI_HEART_BEAT: //HMI����  //ת�Ƶ�SigioHandle()�����ȴ���
//				m_n_hmi_heartbeat = 0;  //������������
//				break;
			case CMD_HMI_DEVICE_SCAN:  //ɨ��ָ��
				this->ProcessHmiDeviceScan(cmd_node);
				break;
			case CMD_HMI_SHAKEHAND://HMI����
				this->ProcessHmiShakehand(cmd_node);
				break;
			case CMD_HMI_SET_CUR_CHANNEL:   //���õ�ǰͨ��
				this->ProcessHmiSetCurChnCmd(cmd);
				break;
			case CMD_HMI_GET_FILE:	//HMI��ȡ�ļ�
//#ifdef USES_MODBUS_PROTOCAL
//				this->ProcessHmiGetFileCmd(cmd_node);
//#else
				this->ProcessHmiGetFileCmd(cmd);
//#endif
				break;
			case CMD_HMI_SEND_FILE:	//HMI�����ļ�
//#ifdef USES_MODBUS_PROTOCAL
//				this->ProcessHmiSendFileCmd(cmd_node);
//#else
				this->ProcessHmiSendFileCmd(cmd);
//#endif
				break;
			case CMD_HMI_DISCONNECT:  //HMI�ر�����
				this->ProcessHmiDisconnectCmd(cmd);
				break;
			case CMD_HMI_NC_FILE_COUNT: //HMI��ȡSCģ��洢��NC�ӹ��ļ�����
				this->ProcessHmiNcFileCountCmd(cmd_node);
				break;
			case CMD_HMI_NC_FILE_INFO:  //HMI��ȡNC�ļ������б�
				this->ProcessHmiNcFileInfoCmd(cmd_node);
				break;
			case CMD_HMI_FILE_OPERATE:   //����HMI�������ļ���������
				this->ProcessHmiFileOperateCmd(cmd_node);
				break;
			case CMD_HMI_READY:   //HMI׼������
				this->ProcessHmiReadyCmd(cmd);
				break;
			case CMD_HMI_NEW_ALARM: //HMI�澯
				ProcessHmiNewAlarmCmd(cmd);
				break;
			case CMD_HMI_GET_VERSION:  //��ȡ�汾��Ϣ
				ProcessHmiGetVersionCmd(cmd);
				break;
			case CMD_HMI_GET_FILE_SIGNATURE:  //��ȡNC�ļ�ǩ��
				ProcessHmiFileSignatureCmd(cmd_node);
				break;
			case CMD_HMI_GET_CHN_COUNT: //HMI��ȡͨ������
				this->ProcessHmiChnCountCmd(cmd);
				break;
			case CMD_HMI_GET_ESB_INFO:            //HMI��SC��ȡESB�ļ�����   0x2A
				this->ProcessHmiGetEsbInfoCmd(cmd);
				break;
			case CMD_HMI_ESB_OPERATE:            //HMI֪ͨSC��ָ��ESB�ļ����в���  0x2B
                this->ProcessHmiOperateEsbCmd(cmd);
				break;
			case CMD_HMI_SYNC_TIME:               //HMI��SC��ѯ��ǰϵͳʱ��  0x36
				this->ProcessHmiSyncTimeCmd(cmd);
				break;
//			case CMD_HMI_GET_CHN_STATE: //HMI��ȡͨ����ǰ״̬
//				ProcessHmiGetChnStateCmd(cmd);
//				break;
				//����ָ��ݸ�ͨ�����洦��
			case CMD_HMI_SET_NC_FILE:			 //���õ�ǰ�ӹ��ļ� 13
				//��Ҫ�жϴ��ļ��Ƿ����ڴ�����
				if(m_b_trans_file && this->m_b_recv_file && m_file_type == FILE_G_CODE
						&& strcmp(m_str_file_name, cmd.data) == 0){//�ļ����ڴ�����
					cmd.frame_number |= 0x8000;
					cmd.cmd_extension = FAILED;
					this->SendCmd(cmd);
					printf("file[%s] is transfering! \n", cmd.data);
					break;
				}
			case CMD_HMI_SET_PARA:				 //��������
			case CMD_HMI_GET_PARA:				 //������ȡ
			case CMD_HMI_GET_CHN_STATE: 		 //HMI��ȡͨ����ǰ״̬
			case CMD_HMI_RESTART:               //�ӹ���λ 11
			case CMD_HMI_SIMULATE:				 //���� 12
			case CMD_HMI_FIND_REF_POINT:		 //ȷ���ο��� 14   �������ʽ������
			case CMD_SC_MDA_DATA_REQ:		 	//MDA�������� 115
			case CMD_HMI_UPDATE:      			//HMI֪ͨSC������������ļ�����
			case CMD_HMI_GET_PMC_REG:			//��ȡPMC�Ĵ���
			case CMD_HMI_SET_PMC_REG:			//����PMC�Ĵ���
			case CMD_HMI_GET_PMC_UUID:			//��ȡPMC��UUID
			case CMD_HMI_SET_REF_POINT:			//������ԭ�㣬��Ծ���ֵ������
			case CMD_HMI_GET_MACRO_VAR:			//HMI��SC����������ֵ
			case CMD_HMI_SET_MACRO_VAR:        //HMI��SC���ú�����Ĵ�����ֵ
			case CMD_HMI_SET_CALIBRATION:     //HMI��SC�����������궨ָ�� 32
			case CMD_HMI_AXIS_MANUAL_MOVE:     //HMIָ��SC���ƶ�ָ��
            case CMD_HMI_CLEAR_WORKPIECE:      //HMI����SC���ӹ���������,��ʱ����(���ְ�ҹ��)
            case CMD_HMI_CLEAR_TOTAL_PIECE:    //HMI����SC���ܹ�����
			case CMD_HMI_GET_LIC_INFO:            //HMI��SC������Ȩ��Ϣ   0x27
			case CMD_HMI_SEND_LICENSE:            //HMI��SC������Ȩ��     0x28
			case CMD_HMI_MANUAL_TOOL_MEASURE:     //HMI��SC�����ֶ��Ե�����  0x29
			case CMD_HMI_GET_IO_REMAP:			//HMI��SC��ȡIO�ض�������  0x2C
			case CMD_HMI_SET_IO_REMAP:         //HMI��SC����IO�ض�������  0x2D
			case CMD_HMI_SET_PROC_PARAM:      //HMI��SC���ù�����ز���  0x2E
			case CMD_HMI_GET_PROC_PARAM:          //HMI��SC��ȡ������ز���  0x2F
			case CMD_HMI_SET_PROC_GROUP:          //HMI��SC���õ�ǰ���ղ������  0x30
			case CMD_HMI_GET_PROC_GROUP:          //HMI��SC��ȡ��ǰ���ղ������  0x31
			case CMD_HMI_SET_CUR_MACH_POS:        //HMI��SC����ָ����Ļ�е����  0x32
			case CMD_HMI_CLEAR_MSG:               //HMI֪ͨSC�����Ϣ  0x33
			case CMD_HMI_SYNC_AXIS_OPT:           //HMI֪ͨHMI����ͬ����ʹ�ܲ��� 0x34
			case CMD_HMI_NOTIFY_GRAPH:            //HMI֪ͨSC����ͼ��ģʽ    0x35
			case CMD_HMI_CHECK_SYNC_EN:           //HMI��SC��ѯͬ����״̬ 0x37
            case CMD_HMI_CLEAR_MACHINETIME_TOTAL: //HMI��SC��������ۼ�ʱ��
            case CMD_HMI_GET_HANDWHEEL_INFO:      //HMI��SC��ȡ������Ϣ
            case CMD_HMI_SET_HANDWHEEL_INFO:      //HMI��SC����������Ϣ
            case CMD_HMI_GET_ERROR_INFO:          //HMI��SC��ȡ������Ϣ
            case CMD_HMI_SET_ALL_COORD:           //HMI��SC���õ�ǰͨ�������й�������ϵ
#ifdef USES_GRIND_MACHINE
			case CMD_SC_MECH_ARM_ERR:         //HMI��Ӧ��е�ָ澯ָ��
#endif
				m_p_channel_engine->ProcessHmiCmd(cmd);
				break;

			case CMD_HMI_MOP_KEY:				//HMI����MOP������Ϣ
				this->ProcessHmiVirtualMOPCmd(cmd);
				break;
			case CMD_SC_REQ_START:   //SC��HMI����ʼ�ӹ�
				break;

			case CMD_SC_NEW_ALARM:
//				printf("get hmi reply alarm response:0x%hx\n", cmd.frame_number);
			case CMD_SC_WORK_STATE:    //HMI��Ӧָ��������κδ���
			case CMD_SC_WORK_MODE:
			case CMD_SC_STATUS_CHANGE:

			case CMD_SC_CLEAR_ALARM:
			case CMD_SC_RESET:
			case CMD_SC_DISPLAY_MODE:
			case CMD_SC_EMERGENCY:              //��ͣ
			case CMD_SC_RESTART_LINE:            //֪ͨHMI��ǰ�ӹ���λ����ɨ�������
			case CMD_SC_MODE_CHANGE:
			case CMD_SC_DISPLAY_FILE:         //֪ͨHMI�л���ʾ���ļ�
			case CMD_SC_WORKPIECE_COUNT:			//֪ͨHMI���¹�������ֵ
			case CMD_SC_READY:
			case CMD_SC_MESSAGE:
			case CMD_SC_UPDATE_MODULE_STATUS:  //֪ͨHMI����״̬�������Ӧ
			case CMD_SC_TRANS_FILE_OVER:
			case CMD_SC_TOOL_MEASURE_OVER:    //�ֶ��Ե������������Ӧ
			case CMD_SC_PARAM_CHANGED:
			case CMD_SC_NOTIFY_MACH_OVER:     //�ӹ�����֪ͨ��Ϣ����Ӧ
            case CMD_SC_NOTIFY_ALARM_CHANGE:
            case CMD_SC_BACKUP_STATUS:        //SC֪ͨHMI��ǰ����״̬
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
				g_ptr_trace->PrintLog(LOG_ALARM, "�յ���֧�ֵ�HMIָ��cmd=%d", cmd.cmd);
				break;
			}
		}

        usleep(10000);   //����10ms
	}


	return res;
}


/**
 * @brief �ڷ��Ͷ�����ɾ����Ӧ����
 * @param frame_index: ��ɾ�����ĵ�֡��
 * @return
 */
bool HMICommunication::DelCmd(uint16_t frame_index){
	bool res = false;

	pthread_mutex_lock(&m_mutex_udp_send);

	//�������Ͷ���
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
 * @brief �ļ����亯��
 * @return
 */
int HMICommunication::TransFile(){
	int res = ERR_NONE;
	sockaddr_in addr_hmi;
	socklen_t len = sizeof(sockaddr);
//	struct timespec time_out= {0, 10000000};


//	while(!g_sys_state.system_ready){  //�ȴ�ϵͳ����
//		usleep(100000);
//	}
	printf("trans file thread: id = %ld\n", syscall(SYS_gettid));

	if(listen(m_soc_tcp_file, 2)!= 0){
		//TODO ������
		printf("ERROR! Failed to listen file trans port!err=%d\n", errno);
		res = ERR_SC_INIT;
		return res;
	}


	while(!g_sys_state.system_quit){

		sem_wait(&m_sem_tcp_file);//�ȴ�tcp�ļ������ź�

	//	sem_post(&this->m_sem_tcp_send_test);  //�Է����ղ���

		if(g_sys_state.eth0_running && g_sys_state.hmi_comm_ready){
			if(m_soc_file_send < 0){
				printf("waitting for file trans connect\n");
				m_soc_file_send = accept(m_soc_tcp_file, (struct sockaddr *)&addr_hmi, &len);
				if(m_soc_file_send < 0){//���ӳ���
					//TODO ������
				//	printf("ERROR! Failed to accept file trans link!err=%d\n", errno);
					this->m_b_trans_file = false;   //��λ�ļ������־
					g_ptr_trace->PrintLog(LOG_ALARM, "�����ļ���������ʧ�ܣ�errno = %d", errno);
                    CreateError(ERR_FILE_TRANS, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno);
					continue;
				}
				else
					printf("tcp file socket: %d, %s\n", m_soc_file_send, inet_ntoa(addr_hmi.sin_addr));
			}

			//TODO �����ļ�
			if(this->m_b_recv_file){
				res = this->RecvFile();
			}
			else{
				res = this->SendFile();
			}
			if(res != ERR_NONE){
				g_ptr_trace->PrintLog(LOG_ALARM, "�ļ������쳣�жϣ�errno = %d", errno);
                CreateError(res, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno);
			}

		}
		this->m_b_trans_file = false;   //��λ�ļ������־
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
//	if(!g_sys_state.hmi_comm_ready){  //HMIδ����
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
//		if(m_soc_monitor_send < 0){//���ӳ���
//			//TODO ������
//			printf("ERROR! Failed to accept monitor link!err=%d\n", errno);
//			return;
//		}else
//			printf("accept monitor connect\n");
//	}
//
//	if(g_sys_state.hmi_comm_ready){
//		//������������
//		if(++m_n_hmi_heartbeat >= kHeartbeatTimeout){//�������ߣ�1��
//			g_sys_state.hmi_comm_ready = false;
//
//			close(m_soc_monitor_send);
//			m_soc_monitor_send = -1;  //�ر�����
//			printf("close monitor socket\n");
//
//
//			g_ptr_trace->PrintLog(LOG_ALARM, "HMI������ʧ��");
//
//			CreateError(ERR_HEARTBEAT_HMI_LOST, ERROR_LEVEL, CLEAR_BY_CLEAR_BUTTON);  //����HMI������ʧ�澯
//			return;
//		}
//
//
//		//TODO ���ͼ������
//		bool bAxisSend = (monitor_count%6==0?true:false);
//		g_ptr_chn_engine->SendMonitorData(m_soc_monitor_send, bAxisSend);   //����ͨ������
//
//	}

//}

/**
 * @brief ������ݴ��亯��
 * @return
 */
static int count1 = 0;
int HMICommunication::Monitor(){
	int res = ERR_NONE;

    static uint64_t monitor_count = 0;
//   	static time_t t_last = time(nullptr);


   	printf("thread id = %ld\n", syscall(SYS_gettid));

   	if(listen(m_soc_tcp_monitor, 5)!= 0){
		//TODO ������
		printf("ERROR! Failed to listen monitor port!err=%d\n", errno);
		res = ERR_SC_INIT;
		pthread_exit((void*) EXIT_FAILURE);
	}

   	//����ˢ��ʱ����
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

	   if(!g_sys_state.hmi_comm_ready){  //HMIδ����
	   		g_sys_state.eth0_running = CheckNetState("eth0");
	   	//	printf("monitor check net state:%d\n", g_sys_state.eth0_running);
	   		usleep(50000);
	   		continue;
	   	}


	   	if(g_sys_state.hmi_comm_ready){
	   		//������������
	//   		printf("m_n_hmi_heartbeat : %u\n", m_n_hmi_heartbeat);
	   		if(++m_n_hmi_heartbeat > kHeartbeatTimeout){//�������ߣ�1��
	   			gettimeofday(&tv_hmi_heart2, NULL);
	   			int nTimeDelay = (tv_hmi_heart2.tv_sec-tv_hmi_heart1.tv_sec)*1000000+tv_hmi_heart2.tv_usec-tv_hmi_heart1.tv_usec;
	   			int nTimeDelay2 = (tv_hmi_heart2.tv_sec-tv_hmi_cmd1.tv_sec)*1000000+tv_hmi_heart2.tv_usec-tv_hmi_cmd1.tv_usec;

	   			g_ptr_trace->PrintLog(LOG_ALARM, "HMI������ʧ��%d, %d, %d", m_n_hmi_heartbeat, nTimeDelay, nTimeDelay2);

	   			this->DisconnectToHmi();  //�ر�HMI����
	   			m_n_hmi_heartbeat = 0;
	   			continue;
	   		}

		   	if(m_soc_monitor_send == -1){
		   		struct sockaddr_in addr_hmi;
		   		socklen_t len = sizeof(struct sockaddr);
		   		printf("waitting for monitor connect\n");
		   		m_soc_monitor_send = accept(m_soc_tcp_monitor, (struct sockaddr *)&addr_hmi, &len);
		   		if(m_soc_monitor_send < 0){//���ӳ���
		   			//TODO ������
		   			printf("ERROR! Failed to accept monitor link!err=%d\n", errno);
		   			g_ptr_trace->PrintLog(LOG_ALARM, "HMI��������жϣ�errno=%d", errno);
		   			this->DisconnectToHmi();  //�ر�HMI����
		   		//	CreateError(ERR_HMI_MONITOR_DISCON, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON);  //����HMI��ض����澯
		   		}else{
		   			printf("accept monitor connect\n");
		   		}
		   	}else{
		   		//TODO ���ͼ������
				bool bAxisSend = (monitor_count%6==0?true:false);
				g_ptr_chn_engine->SendMonitorData(bAxisSend, true);   //����ͨ������
		   	}

	   		//���Է��ͼ�����ݼ��
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
	   		//��ȡ�ڴ�
			//(MemTotal - MemFree)/ MemTotal
			get_memoccupy((MEM_OCCUPY *)&mem_stat);
			/*printf("[MemTotal] = %lu\n "
					"[MemFree] = %lu\n "
					"[Buffers] = %lu\n "
					"[Cached] = %lu\n "
					"[SwapCached] = %lu\n ",
					mem_stat.MemTotal, mem_stat.MemFree, mem_stat.Buffers, mem_stat.Cached, mem_stat.SwapCached);*/

			 //printf("%.3f\n", mem_stat.MemFree * 1.0 / ( mem_stat.MemTotal * 1.0  ) );
			 //��һ�λ�ȡcpuʹ�����
			 CPU_OCCUPY cpu_cur;
			 get_cpuoccupy((CPU_OCCUPY *)&cpu_cur);

			 //����cpuʹ����
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
 * @brief UDP�����ط��̺߳���
 * @param void *args: HMICommunication����ָ��
 */
void *HMICommunication::UdpResendCmdThread(void *args){
	printf("Start HMICommunication::UdpResendCmdThread! id = %ld\n", syscall(SYS_gettid));
	HMICommunication *hmi_comm = static_cast<HMICommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::UdpResendCmdThread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::UdpResendCmdThread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	m_thread_run_flag |= THREAD_CMD_RESEND;

	while(!g_sys_state.hmi_comm_ready)  //�ȴ����Ӿ���
		usleep(50000);  //�ȴ�50ms

	res = hmi_comm->ResendHmiCmd();

	m_thread_run_flag &= (~THREAD_CMD_RESEND);

	printf("Quit HMICommunication::UdpResendCmdThread!\n");
	pthread_exit(NULL);
}

/**
 * @brief UDP����������̺߳���
 * @param void *args: HMICommunication����ָ��
 */
void *HMICommunication::UdpProcessCmdThread(void *args){
	printf("Start HMICommunication::UdpProcessCmdThread! id = %ld\n", syscall(SYS_gettid));
	HMICommunication *hmi_comm = static_cast<HMICommunication *>(args);

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

	m_thread_run_flag |= THREAD_CMD_PROCESS;

	while((g_sys_state.module_ready_mask & SC_READY) == 0) //�ȴ�SCģ���ʼ�����
		usleep(50000);  //�ȴ�50ms

	res = hmi_comm->ProcessHmiCmd();

	m_thread_run_flag &= (~THREAD_CMD_PROCESS);
	printf("Quit HMICommunication::UdpProcessCmdThread!\n");
	pthread_exit(NULL);
}


/**
 * @brief TCP������ݷ����߳�
 * @param void *args: HMICommunication����ָ��
 */
void *HMICommunication::TcpMonitorThread(void *args){
	printf("Start HMICommunication::TcpMonitorThread!thread id = %ld\n", syscall(SYS_gettid));

	HMICommunication *hmi_comm = static_cast<HMICommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::TcpMonitorThread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::TcpMonitorThread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	m_thread_run_flag |= THREAD_MONITOR;

	g_ptr_trace->PrintTrace(TRACE_INFO, HMI_COMMUNICATION, "�ȴ��������ӡ�����������");
	while(!g_sys_state.eth0_running){
		usleep(200000);   //�ȴ�200ms
		g_sys_state.eth0_running = CheckNetState("eth0");
	}

	g_ptr_trace->PrintTrace(TRACE_INFO, HMI_COMMUNICATION, "�ȴ�SCģ��׼���á�����������");

	while((g_sys_state.module_ready_mask & SC_READY) == 0){  //�ȴ�SCģ���ʼ�����
		usleep(100000);
	}

	g_ptr_trace->PrintTrace(TRACE_INFO, HMI_COMMUNICATION, "��ʼ�����TCP���ӡ�����������");
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
 * @brief TCP�ļ������̣߳����������ļ����ӹ��ļ��������ļ���
 * @param void *args: HMICommunication����ָ��
 */
void *HMICommunication::TcpTransFileThread(void *args){
	printf("Start HMICommunication::TcpTransFileThread! id = %ld\n", syscall(SYS_gettid));
	HMICommunication *hmi_comm = static_cast<HMICommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::TcpTransFileThread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::TcpTransFileThread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	m_thread_run_flag |= THREAD_FILE_TRANS;

	while((g_sys_state.module_ready_mask & SC_READY) == 0) //�ȴ�SCģ���ʼ�����
		usleep(50000);  //�ȴ�50ms

	res = hmi_comm->TransFile();

	m_thread_run_flag &= (~THREAD_FILE_TRANS);
	printf("Quit HMICommunication::TcpTransFileThread!\n");
	pthread_exit(NULL);
}

/**
 * @brief �ļ����Ϊ�߳�
 * @param void *args: HMICommunication����ָ��
 */
void *HMICommunication::SaveAsFileThread(void *args){
	printf("Start HMICommunication::SaveAsFileThread!id = %ld\n", syscall(SYS_gettid));
	SaveAsParam *pp = static_cast<SaveAsParam *>(args);
	HMICommunication *hmi_comm = pp->pComm;

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
	if (res != ERR_NONE) {
		printf("Quit HMICommunication::SaveAsFileThread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
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
 * @brief ���ڴ����ʱ����
 * @param void *args: HMICommunication����ָ��
 */
void *HMICommunication::BackgroundThread(void *args)
{
    printf("Start HMICommunication::BackgroundThread!id = %ld\n", syscall(SYS_gettid));
    HMICommunication *hmi_comm = static_cast<HMICommunication *>(args);

    int res = ERR_NONE;
    res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
    if (res != ERR_NONE) {
        printf("Quit HMICommunication::BackgroundThread with error 1!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }
    res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
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
 * @brief nc�ļ����Ϊ
 * @return
 */
int HMICommunication::SaveasFileFunc(const char *path_old, const char *path_new){
	int ret = ERR_NONE;
	char buf[10240];
	uint64_t file_size = 0;    //�ļ���С
	uint64_t remains = 0;  //ʣ���ֽ���
	int read_num = 10240;  //ÿ�ζ�ȡ�ֽ���
	mode_t file_mode;    //�ļ�����

	printf("Start HMICommunication::SaveasNcFileFunc!thread id = %ld\n", syscall(SYS_gettid));


	//���ļ�
	int fd = open(path_old, O_RDONLY);
	int fd_new = open(path_new, O_WRONLY|O_CREAT|O_TRUNC);
	if(fd == -1 || fd_new == -1){
		g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "�򿪻򴴽��ļ�ʧ�ܣ�");
		ret = ERR_SAVEAS_FILE;
		goto END;
	}

	//��ȡ�ļ���С
	if(!GetNcFileInfo(path_old, file_size, nullptr, &file_mode)){
		g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "��ȡ�ļ�[%s]��Сʧ�ܣ�", path_old);
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
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "��ȡ�ļ�����Ӧ��ȡ%d�ֽڣ�ʵ�ʶ�ȡ%d�ֽڡ�", read_num, ret);
			ret = ERR_SAVEAS_FILE;
			goto END;
		}
		ret = write(fd_new, buf, read_num);
		if(ret != read_num){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "д���ļ�����Ӧд��%d�ֽڣ�ʵ��д��%d�ֽڡ�", read_num, ret);
			ret = ERR_SAVEAS_FILE;
			goto END;
		}
		remains -= read_num;
	}

	END:
	close(fd);
	close(fd_new);
	chmod(path_new, file_mode);   //�޸����ļ���������ͬԭ�ļ�
	this->m_thread_saveas = 0;
	ret = ERR_NONE;

	sync();

	return ret;

}

/**
 * @brief ��ʱ�����̨����
 * @return
 */
void HMICommunication::BackgroundFunc()
{
    while(!g_sys_state.system_quit){

        sem_wait(&m_sem_background);//�ȴ�tcp�ļ������ź�
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
 * @brief ����HMI�������豸ɨ��ָ��
 * @param cmd_node : ��������
 * @return
 */
void HMICommunication::ProcessHmiDeviceScan(HMICmdRecvNode &cmd_node){
	int res = ERR_NONE;
	HMICmdFrame resp;
	memset(resp.data, 0x00, kMaxHmiDataLen);


	if(cmd_node.cmd.cmd_extension == 0)  //��ͳtcp/udpЭ��
		cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD);  //���Ķ˿�
	else
		cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //���Ķ˿�
//#ifdef USES_MODBUS_PROTOCAL
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //���Ķ˿�
//#else
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD);  //���Ķ˿�
//#endif

	//���ͻظ�
	resp.frame_number = (cmd_node.cmd.frame_number | 0x8000);  //�ظ�֡��
	resp.channel_index = cmd_node.cmd.channel_index;
	resp.cmd = cmd_node.cmd.cmd;


	if(!g_sys_state.hmi_comm_ready){
		resp.cmd_extension = APPROVE;

	}else{
		resp.cmd_extension = REFUSE;   //�豸æ
	}
	strcpy(resp.data, g_sys_state.local_host_addr);
	resp.data_len = strlen(resp.data);


	//���ͻظ���
	int len = kHmiCmdHeadLen+resp.data_len;
	res = sendto(this->m_soc_udp_send, &resp, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
	if(len != res){
		//printf("ERROR! Failed to send scan response cmd to hmi!%d, %d\n", len,res);
		g_ptr_trace->PrintLog(LOG_ALARM, "�ظ��豸ɨ��ָ��ʧ��[host:%s]��", inet_ntoa(cmd_node.ip_addr.sin_addr));
		return;
	}

	//printf("receive hmi device scan cmd, host[%s]!\n", inet_ntoa(cmd_node.ip_addr.sin_addr));
	g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "�յ�����[%s]���豸ɨ��ָ�", inet_ntoa(cmd_node.ip_addr.sin_addr));
}


/**
 * @brief ����HMI����������ָ��
 * @param cmd_node: ��������
 */
void HMICommunication::ProcessHmiShakehand(HMICmdRecvNode &cmd_node){

	HMICmdFrame resp;
	memset(resp.data, 0x00, kMaxHmiDataLen);

	HMICmdFrame &cmd = cmd_node.cmd;

	if(cmd.data_len == strlen(STR_HMI_SHAKE) &&
			strcmp(cmd.data, STR_HMI_SHAKE) == 0){
		if(!g_sys_state.hmi_comm_ready){
			m_addr_udp_hmi = cmd_node.ip_addr;
			if(cmd.cmd_extension == 0)  //��ͳtcp/udpЭ��
				m_addr_udp_hmi.sin_port = htons(PORT_UDP_CMD);  //���Ķ˿�
			else
				m_addr_udp_hmi.sin_port = htons(PORT_UDP_CMD_SEND);  //���Ķ˿�
//#ifdef USES_MODBUS_PROTOCAL
//			m_addr_udp_hmi.sin_port = htons(PORT_UDP_CMD_SEND);  //���Ķ˿�
//#else
//			m_addr_udp_hmi.sin_port = htons(PORT_UDP_CMD);  //���Ķ˿�
//#endif

			this->m_n_hmi_heartbeat = 0;   //������������



			resp.cmd_extension = APPROVE;
			resp.data_len = strlen(STR_HMI_SHAKE_RESP);
			strcpy(resp.data, STR_HMI_SHAKE_RESP);
			g_sys_state.hmi_comm_ready = true;  //hmiͨѶ׼����

		}else{
			resp.cmd_extension = REFUSE;   //�豸æ
			resp.data_len = 0x00;

		}


		//���ͻظ�
		resp.frame_number = (cmd.frame_number | 0x8000);  //�ظ�֡��
		resp.channel_index = cmd.channel_index;
		resp.cmd = cmd.cmd;

		this->SendCmd(resp);

		if(resp.cmd_extension == APPROVE){
			strcpy(g_sys_state.hmi_host_addr, inet_ntoa(m_addr_udp_hmi.sin_addr));
			g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "������HMI[%s:%hu]�����ӣ�", g_sys_state.hmi_host_addr, ntohs(m_addr_udp_hmi.sin_port));

#ifdef USES_LICENSE_FUNC
		m_p_channel_engine->CheckLicense(true);
#endif

			g_ptr_alarm_processor->SendToHmi();  //�����д����͸�HMI
		}
		else
			g_ptr_trace->PrintLog(LOG_ALARM, "ϵͳ������HMI[%s]���ܾ�����[%s]��HMI ����, %d��",
					g_sys_state.hmi_host_addr, inet_ntoa(cmd_node.ip_addr.sin_addr), this->m_n_hmi_heartbeat);
	}

}

/**
 * @brief ����HMI��ȡ�ļ�ָ��
 * @param HMICmdFrame &cmd: ��������
 */
void HMICommunication::ProcessHmiGetFileCmd(HMICmdFrame &cmd){

	if(this->m_b_trans_file){
		//��ǰ�Ѿ������ļ������У��ܾ�
		cmd.data[cmd.data_len] = REFUSE;  //�ܾ�
		printf("In file transferring , refuse to send file, file type = %hu\n", cmd.cmd_extension);
	}
	else{
		if(cmd.cmd_extension == FILE_PMC_LADDER){  //������ͼ�ļ����ȼ����ͼ�ļ��Ƿ����
			char path[kMaxPathLen];
			strcpy(path, PATH_PMC_LDR);
			if(access(path, F_OK) == -1){	//�����ڣ�����ʧ��
				cmd.data[cmd.data_len] = REFUSE;  //�ܾ�
				printf("ladder file do not exist, refuse to send file\n");
				goto END;
			}
		}else if(cmd.cmd_extension == FILE_WARN_HISTORY){  //����澯��ʷ�ļ����ȼ���ļ��Ƿ����
			if(!TraceInfo::IsAlarmFileExist(0x10)){	//�����ڣ�����ʧ��
				cmd.data[cmd.data_len] = NOTEXIST;  //�ܾ�
				printf("alarm file do not exist, refuse to send file\n");
				goto END;
			}
		}else if(cmd.cmd_extension == FILE_ESB_DEV){ //�����ŷ������ļ�������ļ��Ƿ����
			char path[kMaxPathLen];
			strcpy(path, PATH_ESB_FILE);
			strcat(path, cmd.data);
			if(access(path, F_OK) == -1){	//�����ڣ�����ʧ��
				cmd.data[cmd.data_len] = REFUSE;  //�ܾ�
				printf("esb file[%s] do not exist, refuse to send file\n", cmd.data);
				goto END;
			}
		}
		//��������
		this->m_b_trans_file = true;
		this->m_b_recv_file = false;

		cmd.data[cmd.data_len] = APPROVE;  //����

		m_file_type = (FileTransType)cmd.cmd_extension;
		if(m_file_type == FILE_G_CODE){
			memset(m_str_file_name, 0x00, kMaxFileNameLen);
			strcpy(m_str_file_name, cmd.data);

			printf("HMI get file: %s\n", m_str_file_name);
//			for(int i=0; i < cmd.data_len; i++)
//				printf("[%d]= 0x%x ", i, cmd.data[i]);
//			printf("\n");
		}else if(m_file_type == FILE_CONFIG_PACK){  //�����ļ�����ļ�
			memcpy(&this->m_mask_config_pack, cmd.data, cmd.data_len);   //����mask
            printf("pack config: mask = 0x%x\n", m_mask_config_pack);
		}else if(m_file_type == FILE_WARN_HISTORY){  //�澯��ʷ�ļ�
//			this->m_n_alarm_file_type = cmd.data[0];
//			printf("HMI get alarm file, type = %hhu\n", m_n_alarm_file_type);
		}else if(cmd.cmd_extension == FILE_ESB_DEV){ //�����ŷ������ļ�
			memset(m_str_file_name, 0x00, kMaxFileNameLen);
			strcpy(m_str_file_name, cmd.data);
        }

		sem_post(&m_sem_tcp_file);  //�����źţ������ļ������߳�

	}

	END:
	cmd.frame_number |= 0x8000;
	cmd.data_len++;
	this->SendCmd(cmd);  //������Ӧ��

}

//#ifdef USES_MODBUS_PROTOCAL
///**
// * @brief ����HMI��ȡ�ļ�ָ��
// * @param cmd_node : ָ��ڵ㣬�������ͷ���ַ��Ϣ
// */
//void HMICommunication::ProcessHmiGetFileCmd(HMICmdRecvNode &cmd_node){
//	HMICmdFrame &cmd = cmd_node.cmd;
//	if(this->m_b_trans_file){
//		//��ǰ�Ѿ������ļ������У��ܾ�
//		cmd.data[cmd.data_len] = REFUSE;  //�ܾ�
//		printf("In file transferring , refuse to send file, file type = %hu\n", cmd.cmd_extension);
//	}
//	else{
//		if(cmd.cmd_extension == FILE_PMC_LADDER){  //������ͼ�ļ����ȼ����ͼ�ļ��Ƿ����
//			char path[kMaxPathLen];
//			strcpy(path, PATH_PMC_LDR);
//			if(access(path, F_OK) == -1){	//�����ڣ�����ʧ��
//				cmd.data[cmd.data_len] = REFUSE;  //�ܾ�
//				printf("ladder file do not exist, refuse to send file\n");
//				goto END;
//			}
//		}
//		//��������
//		this->m_b_trans_file = true;
//		this->m_b_recv_file = false;
//
//		cmd.data[cmd.data_len] = APPROVE;  //����
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
//		}else if(m_file_type == FILE_CONFIG_PACK){  //�����ļ�����ļ�
//			memcpy(&this->m_mask_config_pack, cmd.data, cmd.data_len);   //����mask
//			printf("pack config: mask = 0x%x\n", m_mask_config_pack);
//
//		}
//		sem_post(&m_sem_tcp_file);  //�����źţ������ļ������߳�
//
//	}
//
//	END:
//	cmd.frame_number |= 0x8000;
//	cmd.data_len++;
//
//
//	//���ͻظ���
//	int len = kHmiCmdHeadLen+cmd.data_len;
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //���Ķ˿�
//	m_addr_file_trans = cmd_node.ip_addr;   //��¼�ļ�����Ŀ��IP
//	int res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
//	if(len != res){
//		//printf("ERROR! Failed to send scan response cmd to hmi!%d, %d\n", len,res);
//		g_ptr_trace->PrintLog(LOG_ALARM, "�ظ��豸�ļ���ȡָ��ʧ��[host:%s]��", inet_ntoa(cmd_node.ip_addr.sin_addr));
//		return;
//	}else{
//		printf("send resp to hmi get file cmd:ip[%s:%hu]\n", inet_ntoa(cmd_node.ip_addr.sin_addr), ntohs(cmd_node.ip_addr.sin_port));
//	}
//}
//#endif


/**
 * @brief ����HMI�����ļ�ָ��
 * @param HMICmdFrame &cmd: ��������
 */
void HMICommunication::ProcessHmiSendFileCmd(HMICmdFrame &cmd){

	uint64_t free = GetFreeDisk() - g_ptr_parm_manager->GetSystemConfig()->free_space_limit*1024;
	uint64_t filesize = 0;

	m_file_type = (FileTransType)cmd.cmd_extension;
	if(m_file_type == FILE_G_CODE || m_file_type == FILE_ESB_DEV){
		memset(m_str_file_name, 0x00, kMaxFileNameLen);
		strcpy(m_str_file_name, cmd.data);
		if(cmd.data_len == strlen(m_str_file_name)+9){  //���ļ���С
			memcpy(&filesize, &cmd.data[cmd.data_len-8], 8);
			printf("has file size : %llu\n", filesize);
		}else{
			printf("no file size: data_len=%d, filename=%d\n", cmd.data_len, strlen(m_str_file_name));
		}
	}
	if(this->m_b_trans_file|| filesize > free){
		//��ǰ�Ѿ������ļ������У��ܾ�
		cmd.frame_number |= 0x8000;

		memcpy(&cmd.data[cmd.data_len], &free, 8);
		cmd.data_len += 8;

		cmd.data[cmd.data_len] = REFUSE;  //�ܾ�
		printf("refuse to recv file, flag = %hhu, filesize=%llu, free=%llu\n", m_b_trans_file, filesize, free);
	}
	else{
		//��������
		this->m_b_trans_file = true;
		this->m_b_recv_file = true;

		sem_post(&m_sem_tcp_file);  //�����źţ������ļ������߳�

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
		cmd.data[cmd.data_len] = APPROVE;  //����

	}
	cmd.data_len++;

	this->SendCmd(cmd);  //������Ӧ��
}

//#ifdef USES_MODBUS_PROTOCAL
///**
// * @brief ����HMI�����ļ�ָ��
// * @param HMICmdRecvNode &cmd_node: ָ��ڵ㣬�������ͷ���ַ��Ϣ
// */
//void HMICommunication::ProcessHmiSendFileCmd(HMICmdRecvNode &cmd_node){
//	HMICmdFrame &cmd = cmd_node.cmd;
//	if(this->m_b_trans_file){
//		//��ǰ�Ѿ������ļ������У��ܾ�
//		cmd.frame_number |= 0x8000;
//	//	cmd.cmd_extension = REFUSE;  //�ܾ�
//		cmd.data[cmd.data_len] = REFUSE;  //�ܾ�
//		printf("refuse to recv file\n");
//	}
//	else{
//		//��������
//		this->m_b_trans_file = true;
//		this->m_b_recv_file = true;
//
//		sem_post(&m_sem_tcp_file);  //�����źţ������ļ������߳�
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
//		cmd.data[cmd.data_len] = APPROVE;  //����
//
//
//	}
//	cmd.data_len++;
//
//	//���ͻظ���
//	int len = kHmiCmdHeadLen+cmd.data_len;
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //���Ķ˿�
//	m_addr_file_trans = cmd_node.ip_addr;   //��¼�ļ�����Ŀ��IP
//	int res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
//	if(len != res){
//		//printf("ERROR! Failed to send scan response cmd to hmi!%d, %d\n", len,res);
//		g_ptr_trace->PrintLog(LOG_ALARM, "�ظ��豸�ļ�����ָ��ʧ��[host:%s]��", inet_ntoa(cmd_node.ip_addr.sin_addr));
//		return;
//	}else{
//		printf("send resp to hmi send file cmd:ip[%s:%hu]\n", inet_ntoa(cmd_node.ip_addr.sin_addr), ntohs(cmd_node.ip_addr.sin_port));
//	}
//}
//#endif

/**
 * @brief ����HMI�ж�����ָ��
 * @param cmd : ��������
 * @return
 */
void HMICommunication::ProcessHmiDisconnectCmd(HMICmdFrame &cmd){

	if(g_sys_state.hmi_comm_ready)
		this->DisconnectToHmi();   //�ر�HMI����

	//������Ӧ��
	cmd.frame_number |= 0x8000;
	this->SendCmd(cmd);
}

/**
 * @brief ����HMI���õ�ǰͨ������
 * @param cmd : ��������
 */
void HMICommunication::ProcessHmiSetCurChnCmd(HMICmdFrame &cmd){
	bool res = this->m_p_channel_engine->SetCurWorkChanl(cmd.channel_index);

	//������Ӧ��
	cmd.frame_number |= 0x8000;
	cmd.cmd_extension = res?SUCCEED:FAILED;
	this->SendCmd(cmd);
}

/**
 * @brief ����HMI��������
 * @param cmd : ��������
 */
void HMICommunication::ProcessHmiReadyCmd(HMICmdFrame &cmd){
//	g_sys_state.hmi_ready = true;
	g_sys_state.module_ready_mask |= HMI_READY;

	if((g_sys_state.module_ready_mask & SC_READY) == 0)
		cmd.cmd_extension = 1;  // δ����

	//������Ӧ��
	cmd.frame_number |= 0x8000;
	this->SendCmd(cmd);

	if((g_sys_state.module_ready_mask & NC_READY) == NC_READY)
		g_sys_state.system_ready = true;
}

/**
 * @brief ����HMI�澯ָ��
 * @param cmd : ��������
 */
void HMICommunication::ProcessHmiNewAlarmCmd(HMICmdFrame &cmd){

	if(cmd.data_len == 4)
		memcpy((char *)&g_sys_state.hmi_alarm_code, cmd.data, 4);
//	g_sys_state.hmi_ready = false;
	g_sys_state.module_ready_mask &= ~(HMI_READY);

	//������Ӧ��
	cmd.frame_number |= 0x8000;
	cmd.data_len = 0;
	this->SendCmd(cmd);
}

/**
 * @brief ����HMI��ȡ�汾��Ϣָ��
 * @param cmd : ��������
 */
void HMICommunication::ProcessHmiGetVersionCmd(HMICmdFrame &cmd){

	if(cmd.cmd_extension == SOFTWARE_VER){
		cmd.data_len = sizeof(SwVerInfoCollect);
		memcpy(cmd.data, (char *)&g_sys_info.sw_version_info, cmd.data_len);
	}
	else if(cmd.cmd_extension == HARDWARE_VER){
		//TODO ����Ӳ���汾����
		cmd.data_len = 0;
	}

	//������Ӧ��
	cmd.frame_number |= 0x8000;

	this->SendCmd(cmd);
}

/**
 * @brief ����HMI��ȡͨ��������
 * @param cmd : ��������
 * @return
 */
void HMICommunication::ProcessHmiChnCountCmd(HMICmdFrame &cmd){

	//������Ӧ��
	cmd.frame_number |= 0x8000;
	if(g_sys_state.system_boot_stage > STEP_INIT_PARM_MANAGER)
		cmd.cmd_extension = g_ptr_parm_manager->GetSystemConfig()->chn_count;
	else
		cmd.cmd_extension = 0;  //��ʼ��δ���

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

	//������Ӧ��
	cmd.frame_number |= 0x8000;
	cmd.data_len = (uint8_t)sizeof(HmiChannelStatus);
	memcpy(cmd.data, (char *)&status, cmd.data_len);
	this->SendCmd(cmd);
}

/**
 * @brief ����HMI��ȡNC�ļ�����ָ��
 * @param cmd : ��������
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

	//������Ӧ��
	cmd.frame_number |= 0x8000;
	cmd.data_len += 4;
	uint32_t count = this->GetNcFileCount(path);
	memcpy(cmd.data, (char *)&count, sizeof(count));

//#ifdef USES_MODBUS_PROTOCAL
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //���Ķ˿�
//	int len = kHmiCmdHeadLen+cmd.data_len;
//	int res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
//	if(len != res){
//		//printf("ERROR! Failed to send scan response cmd to hmi!%d, %d\n", len,res);
//		g_ptr_trace->PrintLog(LOG_ALARM, "�ظ��豸��ȡ�ӹ��ļ�����ָ��ʧ��[host:%s]��", inet_ntoa(cmd_node.ip_addr.sin_addr));
//		return;
//	}
//#else
	this->SendCmd(cmd);
//#endif
}

/**
 * @brief ����HMI��ȡnc�ļ���ϸ��Ϣ����
 * @param cmd : ��������
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
		g_ptr_trace->PrintLog(LOG_ALARM, "���ļ�Ŀ¼ʧ�ܣ�[%s]", PATH_NC_FILE);
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
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //���Ķ˿�
//#endif
	//ѭ����ȡĿ¼�µ��ļ�
	while(nullptr != (dir_obj = readdir(dir))){
		dir_name = dir_obj->d_name;
		if(strcmp(dir_name, ".") == 0 || strcmp(dir_name, "..") == 0){
			continue;
		}
		else if(dir_obj->d_type == DT_REG){   //��ͨ�ļ�

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
		else if(dir_obj->d_type == DT_DIR){ 	//�ļ���
			cmd.cmd_extension = FILE_DIR;
			cmd.data_len = 4+strlen(dir_name);
			strcpy(cmd.data+4, dir_name);
		//	printf("dir-%d[%s]\n", file_index, dir_name);
		}
		//������Ӧ��
		cmd.frame_number |= 0x8000;
		memcpy(cmd.data, (char *)&file_index, 4);

//#ifdef USES_MODBUS_PROTOCAL
//		int len = kHmiCmdHeadLen+cmd.data_len;
//		int res = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
//		if(len != res){
//			//printf("ERROR! Failed to send scan response cmd to hmi!%d, %d\n", len,res);
//			g_ptr_trace->PrintLog(LOG_ALARM, "�ظ��豸��ȡ�ӹ��ļ���Ϣָ��ʧ��[host:%s]��", inet_ntoa(cmd_node.ip_addr.sin_addr));
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
 * @brief ����HMI��ȡNC�ļ�ǩ��������
 * @param cmd : ��������
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
	if(fd > 0){//�ļ��򿪳ɹ�
		read(fd, cmd.data+cmd.data_len+1, kNcFileSignLen);
		cmd.data_len += (kNcFileSignLen+1);
	}else
	{
		printf("Failed to open file[%s] in ProcessHmiFileSignatureCmd\n", file_path);
	}

	//������Ӧ��
	cmd.frame_number |= 0x8000;

//#ifdef USES_MODBUS_PROTOCAL
//	this->SendCmdToIp(cmd_node);
//#else
	this->SendCmd(cmd);
//#endif
}


/**
 * @brief ����HMI�������ļ���������
 * @param cmd : ��������
 */
void HMICommunication::ProcessHmiFileOperateCmd(HMICmdRecvNode &cmd_node){
	HMICmdFrame &cmd = cmd_node.cmd;
	char name_old[kMaxPathLen] = {0};
	char name_new[kMaxPathLen] = {0};
	char *pSplit = nullptr;  //�ָ���ָ��
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

	//������Ӧ��
	cmd.frame_number |= 0x8000;
	cmd.data[cmd.data_len] = res?SUCCEED:FAILED;
	cmd.data_len++;

//#ifdef USES_MODBUS_PROTOCAL
//	cmd_node.ip_addr.sin_port = htons(PORT_UDP_CMD_SEND);  //���Ķ˿�
//	int len = kHmiCmdHeadLen+cmd.data_len;
//	int send_count = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&cmd_node.ip_addr, m_n_addr_len);
//	if(len != send_count){
//		//printf("ERROR! Failed to send scan response cmd to hmi!%d, %d\n", len,res);
//		g_ptr_trace->PrintLog(LOG_ALARM, "�ظ��豸�ļ�����ָ��ʧ��[host:%s]��", inet_ntoa(cmd_node.ip_addr.sin_addr));
//		return;
//	}
//#else
	this->SendCmd(cmd);
//#endif
}

/**
 * @brief ����HMI��ȡESB�ļ���Ϣ������
 * @param cmd
 */
void HMICommunication::ProcessHmiGetEsbInfoCmd(HMICmdFrame &cmd){
	cmd.frame_number |= 0x8000;
	int count = this->GetEsbFileCount(PATH_ESB_FILE);

	if(count == 0){  //û��esb�ļ�
		//���ͻظ�
		cmd.data_len = 9;
		cmd.cmd_extension = 0;
		this->SendCmd(cmd);

		return;
	}

	DIR *dir = opendir(PATH_ESB_FILE);
	if(dir == nullptr){
		//���ͻظ�
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

	//ѭ����ȡĿ¼�µ��ļ�
	while(nullptr != (dir_obj = readdir(dir))){
		dir_name = dir_obj->d_name;
		if(strcmp(dir_name, ".") == 0 || strcmp(dir_name, "..") == 0){
			continue;
		}
		else if(dir_obj->d_type == DT_REG){   //��ͨ�ļ�
			//У���ļ���׺�ǲ���.esb
			len = strlen(dir_name);
			if(len > 4){
				pt = strrchr(dir_name, '.');
				if(pt && strcasecmp(pt, "esb")){  //��׺ƥ��
					strcpy(file_path, PATH_ESB_FILE);
					strcat(file_path, dir_name);
					if(this->GetNcFileInfo(file_path, file_size, mtime)){
						cmd.cmd_extension = count;
						cmd.data_len = 9+strlen(dir_name);
						cmd.data[0] = file_index;   //˳���
						memcpy(cmd.data+1, (char *)&file_size, 8);  //�ļ��ֽ���
						strcpy(cmd.data+9, dir_name);  //�ļ�����

					}
					file_index++;
				}
			}

		}
		else if(dir_obj->d_type == DT_DIR){ 	//�ļ���
			continue;
		}
		//������Ӧ��
		this->SendCmd(cmd);

	}
	closedir(dir);
}

/**
 * @brief ����HMI����ESB�ļ�������
 * @param cmd
 */
void HMICommunication::ProcessHmiOperateEsbCmd(HMICmdFrame &cmd){
	char name_old[kMaxPathLen] = {0};
	char name_new[kMaxPathLen] = {0};
	char *pSplit = nullptr;  //�ָ���ָ��
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

	//������Ӧ��
	cmd.frame_number |= 0x8000;
	cmd.data[cmd.data_len] = res?SUCCEED:FAILED;
	cmd.data_len++;

	this->SendCmd(cmd);
}

/**
 * @brief ����HMIͬ��ϵͳʱ������
 * @param cmd : ��������
 */
void HMICommunication::ProcessHmiSyncTimeCmd(HMICmdFrame &cmd){
	//��ȡ��ǰʱ��
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

	//������Ӧ��
	cmd.frame_number |= 0x8000;
	cmd.data_len = 7;

    this->SendCmd(cmd);
}

/**
 * @brief ����HMI��������
 * @param cmd : ��������
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
        sem_post(&m_sem_background);//ִ֪ͨ�к�̨����
    }
}

/**
 * @brief ����HMI�ָ�����
 * @param cmd : ��������
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
 * @brief ����HMI����MOP��������
 * @param cmd : ��������
 */
void HMICommunication::ProcessHmiVirtualMOPCmd(HMICmdFrame &cmd){
//	static bool kk = false;
	printf("Get Virtual MOP Key : %d, len = %d, channel_index = %d,  cmd = %d, data= %hhu\n", cmd.cmd_extension,cmd.data_len,cmd.channel_index,cmd.cmd, cmd.data[0]);

	//���ͻظ�����
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
	case MOP_KEY_SINGLE_BLOCK:    //��������
		m_p_channel_engine->SetFuncState(CHANNEL_ENGINE_INDEX, FS_SINGLE_LINE);
		break;
	case MOP_KEY_JUMP:    //��������
		m_p_channel_engine->SetFuncState(CHANNEL_ENGINE_INDEX, FS_BLOCK_SKIP);
		break;
	case MOP_KEY_M01:    //ѡͣ����
		m_p_channel_engine->SetFuncState(CHANNEL_ENGINE_INDEX, FS_OPTIONAL_STOP);
		break;
	case MOP_KEY_HW_TRACE:     //���ָ�������
		m_p_channel_engine->SetFuncState(CHANNEL_ENGINE_INDEX, FS_HANDWHEEL_CONTOUR);
		break;
	case MOP_KEY_G00_RATIO:    //���ٱ���
        // lidianqiang: ȥ�����ʿ��ƵĽӿڣ����ٱ���ֻ��PMC�ź�ROV������
		break;
	case MOP_KEY_FEED_RATIO:
        // lidianqiang: ȥ�����ʿ��ƵĽӿڣ���������ֻ��PMC�ź�_FV������
        // m_p_channel_engine->SetAutoRatio(cmd.data[0]);
        // todo: �ֶ�����ҲӦ����pmc�ź�_JV������
		m_p_channel_engine->SetManualRatio(cmd.data[0]);
		break;
	case MOP_KEY_SPD_RATIO:
        // lidianqiang: ȥ�����ʿ��ƵĽӿڣ����ᱶ��ֻ��PMC�ź�SOV������
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
	case MOP_KEY_KEY_X_AXIS:			//X��
		m_p_channel_engine->SetCurAxis(0);
		break;
	case MOP_KEY_KEY_Y_AXIS:			//Y��
		m_p_channel_engine->SetCurAxis(1);
		break;
	case MOP_KEY_KEY_Z_AXIS:			//Z��
		m_p_channel_engine->SetCurAxis(2);
		break;
	case MOP_KEY_KEY_4_AXIS:			//4��
		m_p_channel_engine->SetCurAxis(3);
		break;
	case MOP_KEY_KEY_5_AXIS:			//5��
		m_p_channel_engine->SetCurAxis(4);
		break;
	case MOP_KEY_KEY_6_AXIS:			//6��
		m_p_channel_engine->SetCurAxis(5);
		break;
	case MOP_KEY_CUR_AXIS:
		m_p_channel_engine->SetCurAxis(cmd.data[0]);   //���õ�ǰ��
		break;
	case MOP_KEY_SPD_FOREWARD:			//������ת
        // lidianqiang: ȥ��������ƵĽӿڣ�������PMC������
        // m_p_channel_engine->SpindleOut(SPD_DIR_POSITIVE);
		break;
	case MOP_KEY_SPD_STOP:				//����ͣת
        // lidianqiang: ȥ��������ƵĽӿڣ�������PMC������
        // m_p_channel_engine->SpindleOut(SPD_DIR_STOP);
		break;
	case MOP_KEY_SPD_REVERSE:			//���ᷴת
        // lidianqiang: ȥ��������ƵĽӿڣ�������PMC������
        // m_p_channel_engine->SpindleOut(SPD_DIR_NEGATIVE);
		break;
    case MOP_KEY_EMERGENCY:				//��ͣ
        std::cout << "mop energency" << std::endl;
		this->m_p_channel_engine->Emergency();
		break;
	case MOP_KEY_CLEAR_ALARM:			//����澯
		this->m_p_channel_engine->ClearAlarm();
		break;
	case MOP_KEY_SYS_RESET:				//ϵͳ��λ

		this->m_p_channel_engine->SystemReset();
		break;
	case MOP_KEY_LIGHT:	{				//����
	//	this->m_p_channel_engine->ProcessHmiSetRefCmd();  //�������òο��㹦��
#ifdef USES_PEITIAN_SMALL_FIVE
		if(cmd.data[0] == 1)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 0, true);
		else if(cmd.data[0] == 2)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 0, false);
#endif

		this->m_p_channel_engine->PrintDebugInfo();
		break;
	}
	case MOP_KEY_CHIP:					//��м
#ifdef USES_LASER_MACHINE
		this->m_p_channel_engine->StartLaserCalib(true);   //���Ե������궨
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
	case MOP_KEY_GAS:					//����
//		this->m_p_channel_engine->StartLaserCalib(false);   //���Ե������궨
//		printf("MOP_KEY_GAS : %hhu\n", cmd.data[0]);
//		if(cmd.data[0] == 1){
//			this->m_p_channel_engine->SetPmcOutSignal(16, true);
//
//		}else{
//			this->m_p_channel_engine->SetPmcOutSignal(16, false);
//		}

//		this->m_p_channel_engine->ProcessHmiIOCmd(cmd);
//		if(cmd.data[0] == 1){  //���Լ���Ե�
//			cmd.cmd_extension = 0;
//			cmd.data_len = 2*sizeof(int);
//			int h_value = 2;
//			int times = 3;
//			memcpy(cmd.data, &h_value, sizeof(int));
//			memcpy(cmd.data+sizeof(int), &times, sizeof(int));
//			this->m_p_channel_engine->GetChnControl(0)->ProcessHmiManualToolMeasureCmd(cmd);   //�����ֶ��Ե�
//		}

#ifdef USES_PEITIAN_SMALL_FIVE
		if(cmd.data[0] == 1)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 2, true);
		else if(cmd.data[0] == 2)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 2, false);
#endif

		break;
	case MOP_KEY_COOL:					//��ȴ
//		this->m_p_channel_engine->EnableLaserHeightTrace();  //����ʹ�ܸ���
		//������������������
//		cmd.cmd_extension = 0x00;
//		cmd.channel_index = 0;
//		this->m_p_channel_engine->ProcessHmiFindRefCmd(cmd);    //������������������

//		this->m_p_channel_engine->ProcessHmiIOCmd(cmd);

//		if(cmd.data[0] == 1){//����ȡ���Ե�
//		    cmd.cmd_extension = 1;   //ȡ���Ե�
//			cmd.data_len = 2*sizeof(int);
//			int h_value = 2;
//			int times = 3;
//			memcpy(cmd.data, &h_value, sizeof(int));
//			memcpy(cmd.data+sizeof(int), &times, sizeof(int));
//			this->m_p_channel_engine->GetChnControl(0)->ProcessHmiManualToolMeasureCmd(cmd);   //�����ֶ��Ե�
//		}

#ifdef USES_PEITIAN_SMALL_FIVE
		if(cmd.data[0] == 1)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 3, true);
		else if(cmd.data[0] == 2)
			this->m_p_channel_engine->SetPmcRegBitValue_8(PMC_REG_F, 72, 3, false);
#endif

		break;
	case MOP_KEY_AXIS:                 //PMC��ѡ��
		this->m_p_channel_engine->SetCurPmcAxis(cmd.data[0]);   //ѡ��PMC��
		break;
#ifdef USES_GRIND_MACHINE
	case MOP_KEY_Z_AXIS_POS:			//Z�������ֶ�
		m_p_channel_engine->SetCurAxis(2);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_C_AXIS_POS:			//C�������ֶ�
		m_p_channel_engine->SetCurAxis(3);
		if(cmd.data[0] == 1){
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		}
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_C_AXIS_NEG:			//C�Ḻ���ֶ�
		m_p_channel_engine->SetCurAxis(3);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_Z_AXIS_NEG:			//Z�Ḻ���ֶ�
		m_p_channel_engine->SetCurAxis(2);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_X_AXIS_NEG:           //X�Ḻ���ֶ�
		m_p_channel_engine->SetCurAxis(0);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_X_AXIS_POS:			//X�������ֶ�
		m_p_channel_engine->SetCurAxis(0);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_X_AXIS_POS:          //PMC��X����
		m_p_channel_engine->SetCurPmcAxis(5);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_X_AXIS_NEG:       //PMC��X����
		m_p_channel_engine->SetCurPmcAxis(5);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_Y_AXIS_POS:       //PMC��Y����
		m_p_channel_engine->SetCurPmcAxis(6);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_Y_AXIS_NEG:       //PMC��Y����
		m_p_channel_engine->SetCurPmcAxis(6);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_LEFT_AXIS_POS:    //�������������
		m_p_channel_engine->SetCurPmcAxis(7);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_LEFT_AXIS_NEG:    //��������Ḻ��
		m_p_channel_engine->SetCurPmcAxis(7);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_RIGHT_AXIS_POS:   //�Ҳ�����������
		m_p_channel_engine->SetCurPmcAxis(8);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_POSITIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_PMC_RIGHT_AXIS_NEG:   //�Ҳ������Ḻ��
		m_p_channel_engine->SetCurPmcAxis(8);
		if(cmd.data[0] == 1)
			m_p_channel_engine->ManualMove(DIR_NEGATIVE);
		else if(cmd.data[0] == 2)
			m_p_channel_engine->ManualMoveStop();
		break;
	case MOP_KEY_WORKPIECE_CLAMP:		//�����н�
	case MOP_KEY_VACUUM:     			//���
	case MOP_KEY_NEW_TRAY:				//������
	case MOP_KEY_PAUSE_CARRY:          //��ͣ������
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
//		if(cmd.data[0] == 1){//����
//			this->m_p_channel_engine->SetPmcOutSignal(cmd.cmd_extension-MOP_KEY_IO_0, true);
//		}
//		else if(cmd.data[0] == 2){//����
//			this->m_p_channel_engine->SetPmcOutSignal(cmd.cmd_extension-MOP_KEY_IO_0, false);
//		}else if(cmd.data[0] == 0){//����
			this->m_p_channel_engine->SetPmcOutSignal(cmd.cmd_extension-MOP_KEY_IO_0, cmd.data[0]);
//		}
		break;
	case MOP_KEY_SPINDLE_START:  		//��������
//		if(cmd.data[1] == 1)
			m_p_channel_engine->SpindleOut(SPD_DIR_POSITIVE);
//		else
//			m_p_channel_engine->SpindleOut(SPD_DIR_NEGATIVE);
		break;
	case MOP_KEY_SPINDLE_STOP:			//����ֹͣ
		m_p_channel_engine->SpindleOut(SPD_DIR_STOP);
		break;

	case MOP_KEY_FETCHING_PIECE:   //ȡ����
	case MOP_KEY_START_WORKING:    //ȡ�Ͻ���
		this->m_p_channel_engine->ProcessHmiIOCmd(cmd);
		break;

#endif
	default:
		break;
	}
}

/**
 * @brief ɾ���ļ�
 * @param name
 * @return true--�ɹ�   false--ʧ��
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
 * @brief �������ļ�
 * @param old_name : ԭ�ļ�����
 * @param new_name : ���ļ�����
 * @return true--�ɹ�   false--ʧ��
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
 * @brief ���Ϊ�ļ�,����һ���߳̽����ļ����Ϊ����
 * @param old_name : ԭ�ļ�����
 * @param new_name : ���ļ�����
 * @return true--�ɹ�   false--ʧ��
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
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
//	if (res) {
//		g_ptr_trace->PrintLog(LOG_ALARM, "�ļ����Ϊ�߳������̼̳߳�ģʽʧ�ܣ�");
//		pthread_attr_destroy(&attr);
//		return false;
//	}
    res = pthread_create(&m_thread_saveas, &attr,
            HMICommunication::SaveAsFileThread, pParam);    //�����ļ����Ϊ�����߳�

	pthread_attr_destroy(&attr);
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "�ļ����Ϊ�߳�����ʧ�ܣ�");
		return false;
	}

	return true;
}

/**
 * @brief ɾ���ļ�
 * @param name
 * @return true--�ɹ�   false--ʧ��
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
 * @brief �������ļ�
 * @param old_name : ԭ�ļ�����
 * @param new_name : ���ļ�����
 * @return true--�ɹ�   false--ʧ��
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
 * @brief ���Ϊ�ļ�
 * @param old_name : ԭ�ļ�����
 * @param new_name : ���ļ�����
 * @return true--�ɹ�   false--ʧ��
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
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
//	if (res) {
//		g_ptr_trace->PrintLog(LOG_ALARM, "�ļ����Ϊ�߳������̼̳߳�ģʽʧ�ܣ�");
//		pthread_attr_destroy(&attr);
//		return false;
//	}
	res = pthread_create(&m_thread_saveas, &attr,
			HMICommunication::SaveAsFileThread, pParam);    //�����ļ����Ϊ�����߳�

	pthread_attr_destroy(&attr);
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "�ļ����Ϊ�߳�����ʧ�ܣ�");
		return false;
	}

	return true;
}

/**
 * @brief �ر���HMI������
 */
void HMICommunication::DisconnectToHmi(){

	//�رռ�����������
	if(this->m_soc_monitor_send > 0){
		close(m_soc_monitor_send);
		m_soc_monitor_send = -1;
		g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "�رռ����������ӣ�");
	}

	//�ر��ļ���������
	if(m_soc_file_send > 0){
		close(m_soc_file_send);
	//	closesocket(m_soc_file_send);
		m_soc_file_send = -1;
		g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "�ر��ļ��������ӣ�");
	}

	g_sys_state.hmi_comm_ready = false;
}

/**
 * @brief �����ļ�
 * @return
 */
int HMICommunication::SendFile(){
	int res = ERR_NONE;
	uint64_t file_size = 0;  //�ļ���С
	int send_size = 0;      //���η��͵Ĵ�С
	uint64_t send_total = 0;  //�ѷ��͵��ļ���С
	char buffer[kTcpBufferSize] = {0};	//����
	char filepath[kMaxPathLen] = {0};	//�ļ�·��
	char sign_path[kMaxPathLen] = {0};    //ǩ���ļ�·��
	char file_sign[kNcFileSignLen] = {0};  //ǩ��
	int fd = -1;                    //�ļ����
	uint8_t file_type = m_file_type;  //�ļ���������
	uint8_t filename_size = 0;         //�ļ�������
	char *pSplit = nullptr;
	uint8_t alarm_type = 0x20;   // ��ʼ�����ļ�alarmfile_bak.txt
	uint64_t total_size = 0;
	int send_count = 0;

	//���Ժ�ʱ
	struct timeval tvStart;
	struct timeval tvNow;
	unsigned int nTimeDelay = 0;

	HMICmdFrame cmd;

	bzero((char *)&cmd, kMaxHmiCmdFrameLen);
	cmd.cmd = CMD_SC_TRANS_FILE_OVER;   //�����Ӧ�����
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
			strcat(filepath, m_str_file_name);  //��ϳ��ļ�����·��
		}

		strcpy(sign_path, PATH_NC_SIGN_FILE);
		strcat(sign_path, m_str_file_name);
		pSplit = strchr(sign_path, '.');
		if(pSplit != nullptr)
			*pSplit = '\0';
		strcat(sign_path, "_md5");   //��ϳ�ǩ���ļ�����·��
		fd = open(sign_path, O_RDONLY);
		if(fd > 0){
			read(fd, file_sign, kNcFileSignLen);
			close(fd);
		}

	}else if(file_type == FILE_PMC_LADDER){		//��ͼ�ļ�
		strcpy(filepath, PATH_PMC_LDR);


	}else if(file_type == FILE_UPDATE){   //ģ������ļ�

	}else if(file_type == FILE_CONFIG_PACK){  //���ô���ļ�
		return this->SendConfigPackFile();

	}else if(file_type == FILE_WARN_HISTORY){  //�澯��ʷ�ļ�
		total_size = TraceInfo::GetAlarmTotalSize();
		if(!TraceInfo::IsAlarmFileExist(alarm_type))
			alarm_type = 0x10;
		TraceInfo::GetAlarmFilePath(alarm_type, filepath);
	}else if(file_type == FILE_ESB_DEV){  //�ŷ������ļ�
		filename_size = strlen(this->m_str_file_name);
		strcpy(filepath, PATH_ESB_FILE);
		strcat(filepath, m_str_file_name);
	}else if(file_type == FILE_BACK_DISK) { //һ������
		strcpy(filepath, "/cnc/back.zip");
		char cmd_buf[100];
		sprintf(cmd_buf, "chmod a+x %s", PATH_BACK_DISK_CMD);
		system(cmd_buf);
		int ret = system(PATH_BACK_DISK_CMD);
		if (system(PATH_BACK_DISK_CMD) != 0) { //���б��ݽű�
			printf("error in SendFile fun, failed to run back cmd, return %d %d\n", ret & 0xFF, ret>>8);
			res = ERR_FILE_TRANS;
			goto END;
		}
    }else if(file_type == FILE_BACKUP_CNC) { //����
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


	//���ļ�
	fd = open(filepath, O_RDONLY);
	if(fd == -1){
		printf("error in SendFile fun, failed to open file:%s\n", filepath);
		res = ERR_FILE_TRANS;
		goto END;
	}


	//���ȶ�ȡ�ļ���С
	file_size = lseek(fd, 0, SEEK_END);

	lseek(fd, 0, SEEK_SET);

	//�����ļ�����,�ļ����Լ��ļ���С
	memcpy(buffer, (char *)&file_type, 1);
	if(file_type == FILE_G_CODE){
		memcpy(buffer+1, (char *)&filename_size, 1);
		memcpy(buffer+2, m_str_file_name, filename_size);
		memcpy(buffer+2+filename_size, file_sign, kNcFileSignLen);
		memcpy(buffer+2+filename_size+kNcFileSignLen, (char *)&file_size, sizeof(uint64_t));
		send_size = 10+kNcFileSignLen+filename_size;
	}else if(file_type == FILE_WARN_HISTORY){  //���͸澯��ʷ�ļ����ͣ����ļ���С
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
		g_ptr_trace->PrintLog(LOG_ALARM, "�ļ���С�����쳣��errno = %d", errno);
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
			g_ptr_trace->PrintLog(LOG_ALARM, "�ļ����ݷ����쳣���ѷ���%lld�ֽڣ��ܴ�С%lld�ֽ�, errno = %d",send_total, file_size, errno);
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


		//��ȡ�ļ���С
		file_size = lseek(fd, 0, SEEK_END);

		lseek(fd, 0, SEEK_SET);
		goto TRAN;
	}

	gettimeofday(&tvNow, NULL);
	nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;
	printf("total time = %u us, filesize = %lld\n", nTimeDelay, file_size);

	printf("send file over, sendsize = %lld, filesize = %lld\n", send_total, file_size);


    if(file_type == FILE_BACKUP_CNC && m_sysbackup_status.m_type == SysUpdateStatus::Backup) { //����
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
//	//���ͻظ���
//	int len = kHmiCmdHeadLen+cmd.data_len;
//	int rb = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&m_addr_file_trans, m_n_addr_len);
//	if(len != rb){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "�����ļ�����ָ�����ָ��ʧ��[host:%s]��", inet_ntoa(m_addr_file_trans.sin_addr));
//	}else{
//		printf("send file over cmd:ip[%s:%hu]\n", inet_ntoa(m_addr_file_trans.sin_addr), ntohs(m_addr_file_trans.sin_port));
//	}
//#else
	this->SendCmd(cmd);   //������Ӧ
//#endif

	return res;
}

/**
 * @brief ��ȡ�ļ�����
 * @return ���ݳ���
 */
uint64_t HMICommunication::GetFileLength(const char *path){
	uint64_t len = 0;
	FILE *file = fopen(path, "r+");
	if (nullptr == file)
	{
		return len;
	}

	fseek(file, 0L, SEEK_END);
	len = ftell(file);   //��ȡ�ļ�����

	fclose(file);
	return len;
}

/**
 * @brief ��ȡ�����ļ�·��
 * @param path[out] : �����ļ�·��
 * @param type �� ��������
 * @return true--�ɹ�   false--ʧ��
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
		sprintf(path, PATH_MACRO_VAR_KEEP, 0);  //���ص�һͨ���ĺ�����ļ���
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
 * @brief �������ô���ļ����ֽ���
 * @return ���ֽ���
 */
uint64_t HMICommunication::GetConfigPackFileSize(){
	uint64_t total_size = 4;  //4�ֽ�mask

	if(this->m_mask_config_pack & (0x01<<CONFIG_SYSTEM)){  //ϵͳ����
		total_size += 10;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ���
		total_size += this->GetFileLength(SYS_CONFIG_FILE); //�����ļ���С
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_CHANNEL)){  //ͨ������
		total_size += 10;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ���
		total_size += this->GetFileLength(CHN_CONFIG_FILE); //�����ļ���С
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_AXIS)){  //�����
		total_size += 10;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ���
		total_size += this->GetFileLength(AXIS_CONFIG_FILE); //�����ļ���С
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_COORD)){  //��������ϵ����
		total_size += 10;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ���
		total_size += this->GetFileLength(WORK_COORD_FILE); //�����ļ���С
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_EX_COORD)){  //��չ��������ϵ
		total_size += 10;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ���
		total_size += this->GetFileLength(EX_WORK_COORD_FILE); //�����ļ���С
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_TOOL_INFO)){  //������Ϣ����������ƫ�ü�����λ��
		total_size += 10;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ���
		total_size += this->GetFileLength(TOOL_CONFIG_FILE); //�����ļ���С
	}

#ifdef USES_FIVE_AXIS_FUNC
	if(this->m_mask_config_pack & (0x01<<CONFIG_FIVE_AXIS)){  //��������
		total_size += 10;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ���
		total_size += this->GetFileLength(FIVE_AXIS_CONFIG_FILE); //�����ļ���С
	}
#endif

#ifdef USES_GRIND_MACHINE
	if(this->m_mask_config_pack & (0x01<<CONFIG_GRIND)){  //ĥ������
		total_size += 10;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ���
		total_size += this->GetFileLength(GRIND_CONFIG_FILE); //�����ļ���С
	}
#endif

	if(this->m_mask_config_pack & (0x01<<CONFIG_MACRO_VAR)){  //�����
		total_size += 11;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ��� + 1�ֽ�ͨ����
		char path[kMaxPathLen] = {0};		//Ŀ���ļ�·��
		sprintf(path, PATH_MACRO_VAR_KEEP, 0);
		total_size += this->GetFileLength(path); //�����ļ���С
		for(uint8_t i = 1; i < g_ptr_parm_manager->GetSystemConfig()->chn_count; i++){
			memset(path, 0x00, kMaxPathLen);
			sprintf(path, PATH_MACRO_VAR_KEEP, i);
			total_size += 9;
			total_size += this->GetFileLength(path); //�����ļ���С
		}
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_PMC_REG)){  //PMC�Ĵ���
		total_size += 10;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ���
		total_size += this->GetFileLength(PATH_PMC_REG); //�����ļ���С
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_PMC_LDR)){  //PMC��ͼ
		total_size += 10;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ���(��ͼ�ļ���
		total_size += this->GetFileLength(PATH_PMC_LDR); //������ͼ�ļ���С
		total_size += 9;    //1�ֽڷָ�����#��+ 8�ֽ����ݳ���(PMCִ���ļ���
		total_size += this->GetFileLength(PATH_PMC_DATA); //����PMCִ���ļ���С
	}

	if(this->m_mask_config_pack & (0x01<<CONFIG_IO_REMAP)){ //io��ӳ������
		total_size += 10;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ���
		total_size += this->GetFileLength(IO_REMAP_FILE); //�����ļ���С
	}
    return total_size;
}

/**
 * @brief ѹ�������ļ�
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
        {//300ms��HMI����ѹ��״̬
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

    m_sysbackup_status.m_status = SysUpdateStatus::FileTransing;//֪ͨhmi׼����ʼ�ļ�����
    SendHMIBackupStatus(m_sysbackup_status);
}

/**
 * @brief ��ѹ�����ļ�
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
    {//���ھ���ɾ��Ŀ¼
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
        {//300ms��HMI����ѹ��״̬
            SendHMIBackupStatus(m_sysbackup_status);
            now = chrono::steady_clock::now();
        }
     }
     zip_close(zip);


    //��ȡ���ִ���ɾ���ָ��ļ�
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
 * @brief ��HMI���͵�ǰ����/�ָ�״̬
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
 * @brief �������ô���ļ�
 * @return
 */
int HMICommunication::SendConfigPackFile(){
	int res = ERR_NONE;
	int send_size = 0;      //���η��͵Ĵ�С
	int fd = -1;                    //�ļ����
	char buffer[kTcpBufferSize] = {0};	//����
	char filepath[kMaxPathLen] = {0};	//�ļ�·��
	uint8_t file_type = m_file_type;  //�ļ���������
	uint64_t send_total = 0;  //�ѷ��͵��ļ���С
	uint64_t total_size = this->GetConfigPackFileSize(); //�����ļ��ܴ�С
	uint64_t file_size = 0;  //��ǰ�ļ���С
	uint8_t i = 0, j = 0;
	uint8_t chn_count = 0;


	this->m_p_channel_engine->SyncKeepVar();   //ͬ�������ļ�

	//�����ļ������Լ��ļ���С
	memcpy(buffer, (char *)&file_type, 1);
	memcpy(buffer+1, (char *)&total_size, sizeof(uint64_t));
	memcpy(buffer+9, &m_mask_config_pack, 4);
	send_size = 13;
	total_size -= 4;

	if( -1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
		g_ptr_trace->PrintLog(LOG_ALARM, "�ļ���С�����쳣��errno = %d", errno);
		res = ERR_FILE_TRANS;
		goto END;
	}

	//��ʼ�����ļ�
	for(i = 0; i < CONFIG_TYPE_COUNT; i++){
		if(this->m_mask_config_pack & (0x01<<i)){  //ͨ������
			bzero(filepath, kMaxPathLen);
			if(!this->GetConfigFilePath(filepath, i)){
				printf("error in SendConfigPackFile, failed to get config filepath:%hhu\n", i);
				res = ERR_FILE_TRANS;
				goto END;
			}
			send_total = 0;
			send_size = 10;   //1�ֽڷָ�����#��+ 1�ֽڲ������� + 8�ֽ����ݳ���
			fd = open(filepath, O_RDONLY);
			if(fd == -1){
				printf("error in SendConfigPackFile, failed to open file:%s\n", filepath);
				res = ERR_FILE_TRANS;
				goto END;
			}

			//���ȶ�ȡ�ļ���С
			file_size = lseek(fd, 0, SEEK_END);
			lseek(fd, 0, SEEK_SET);
			bzero(buffer, kTcpBufferSize);
			buffer[0] = '#'; //�ָ���
			memcpy(buffer+1, &i, 1);  //��������
			memcpy(buffer+2, &file_size, 8);  //�ļ���С

			if(i == CONFIG_MACRO_VAR){//�������ÿ��ͨ��һ���ļ��������Ҫͨ����
				chn_count = g_ptr_parm_manager->GetSystemConfig()->chn_count;
				buffer[10] = chn_count;
				send_size++;
			}

			if( -1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
				g_ptr_trace->PrintLog(LOG_ALARM, "�ļ���С�����쳣��errno = %d", errno);
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
					g_ptr_trace->PrintLog(LOG_ALARM, "�ļ����ݷ����쳣���ѷ���%lld�ֽڣ��ܴ�С%lld�ֽ�, errno = %d",send_total, file_size, errno);
					res = ERR_FILE_TRANS;
					close(fd);
					goto END;
				}

				send_total += send_size;
			}
			close(fd);
			fd = -1;
			total_size -= send_total;

			if(i == CONFIG_PMC_LDR){ //��ͼ��Ҫ����ִ���ļ�
				bzero(filepath, kMaxPathLen);
				strcpy(filepath, PATH_PMC_DATA);
				send_total = 0;
				send_size = 9;   //1�ֽڷָ���'#' + 8�ֽ����ݳ���
				fd = open(filepath, O_RDONLY);
				if(fd == -1){
					printf("error in SendConfigPackFile, failed to open file:%s\n", filepath);
					res = ERR_FILE_TRANS;
					goto END;
				}

				//���ȶ�ȡ�ļ���С
				file_size = lseek(fd, 0, SEEK_END);
				lseek(fd, 0, SEEK_SET);
				bzero(buffer, kTcpBufferSize);
				buffer[0] = '#'; //�ָ���
				memcpy(buffer+1, &file_size, 8);  //�ļ���С

				if( -1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
					g_ptr_trace->PrintLog(LOG_ALARM, "�ļ���С�����쳣��errno = %d", errno);
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
						g_ptr_trace->PrintLog(LOG_ALARM, "�ļ����ݷ����쳣���ѷ���%lld�ֽڣ��ܴ�С%lld�ֽ�, errno = %d",send_total, file_size, errno);
						res = ERR_FILE_TRANS;
						close(fd);
						goto END;
					}

					send_total += send_size;
				}
				close(fd);
				fd = -1;
				total_size -= send_total;
			}else if(i == CONFIG_MACRO_VAR && chn_count > 1){  //��ͨ��ʱ������������
				for(j = 1; j < chn_count; j++){
					bzero(filepath, kMaxPathLen);
					sprintf(filepath, PATH_MACRO_VAR_KEEP, j);

					send_total = 0;
					send_size = 9;   //1�ֽڷָ���'#' + 8�ֽ����ݳ���
					fd = open(filepath, O_RDONLY);
					if(fd == -1){
						printf("error in SendConfigPackFile, failed to open file:%s\n", filepath);
						res = ERR_FILE_TRANS;
						goto END;
					}

					//���ȶ�ȡ�ļ���С
					file_size = lseek(fd, 0, SEEK_END);
					lseek(fd, 0, SEEK_SET);
					bzero(buffer, kTcpBufferSize);
					buffer[0] = '#'; //�ָ���
					memcpy(buffer+1, &file_size, 8);  //�ļ���С

					if( -1 == send(m_soc_file_send, buffer, send_size, MSG_NOSIGNAL)){
						g_ptr_trace->PrintLog(LOG_ALARM, "�ļ���С�����쳣��errno = %d", errno);
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
							g_ptr_trace->PrintLog(LOG_ALARM, "�ļ����ݷ����쳣���ѷ���%lld�ֽڣ��ܴ�С%lld�ֽ�, errno = %d",send_total, file_size, errno);
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
		g_ptr_trace->PrintLog(LOG_ALARM, "�ļ����ݷ����쳣��ʣ���ֽ���%lld�ֽڣ�errno = %d",total_size, errno);
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
 * @brief ��ѹ�������ô���ļ�
 * @return true--�ɹ�  false--ʧ��
 */
int HMICommunication::UnpackConfigBakFile(){
	int res = ERR_NONE;

	uint64_t file_size = 0;  //�ļ���С
	int read_size = 0;      //��ȡ���Ĵ�С
	int read_block = 0;		//��ǰ��ȡ���ݴ�С
//	uint64_t read_total = 0;  //�Ѷ�ȡ���ļ���С
	char buffer[kTcpBufferSize] = {0};	//����
	char filepath[kMaxPathLen] = {0};		//Ŀ���ļ�·��
	uint32_t cfg_mask = 0;    //����mask
	uint8_t i = 0, j = 0;
	char split_char;  //�ָ���
	uint8_t cfg_type;  //���ò�������
	uint64_t cfg_file_size = 0;  //�����ļ���С

	uint8_t chn_count = 0;

	int fd = -1, fdd = -1;

	fd = open(PATH_CONFIG_PACK_TMP, O_RDONLY);
	if(fd == -1){
		printf("error in UnpackConfigBakFile, failed to open file:%s\n", PATH_CONFIG_PACK_TMP);
		res = ERR_FILE_TRANS;
		goto END;
	}

	//���ȶ�ȡ�ļ���С
	file_size = lseek(fd, 0, SEEK_END);
	lseek(fd, 0, SEEK_SET);

	//��ȡ����mask
	read_size = read(fd, &cfg_mask, sizeof(uint32_t));
	if(read_size != sizeof(uint32_t)){
		printf("error in UnpackConfigBakFile, failed to read config mask, readsize=%d\n", read_size);
		res = ERR_FILE_TRANS;
		goto END;
	}
	printf("read pack config file mask: 0x%x\n", cfg_mask);

	file_size -= sizeof(uint32_t);

	for(i = 0; i < CONFIG_TYPE_COUNT  && file_size > 0; i++){
		if(cfg_mask & (0x01<<i)){  //ͨ������
			if(file_size < 10){
				printf("error in UnpackConfigBakFile, invalid file format\n");
				res = ERR_FILE_TRANS;
				goto END;
			}
			printf("unpack config:%hhu\n", i);
			//��ȡ�ָ������������ͼ������ļ�����
			read(fd, &split_char, 1);
			read(fd, &cfg_type, 1);
			read(fd, &cfg_file_size, 8);
			if(split_char != '#' || cfg_type != i){
				printf("error in UnpackConfigBakFile, invalid file format2, type = %hhu\n", i);
				res = ERR_FILE_TRANS;
				goto END;
			}
			file_size -= 10;

			if(i == CONFIG_MACRO_VAR){//�������ÿ��ͨ��һ���ļ��������Ҫͨ����
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

			//��ȡ�ļ���д������
			while(0 < cfg_file_size){
				bzero(buffer, kTcpBufferSize);
				read_block = cfg_file_size > kTcpBufferSize?kTcpBufferSize:cfg_file_size;
				read_size = read(fd, buffer, read_block);
				if(read_size != read_block){
					printf("error in UnpackConfigBakFile, read file error:%hhu\n", i);
					res = ERR_FILE_TRANS;
					goto END;
				}

				write(fdd, buffer, read_size);  //д���ļ�
				cfg_file_size -= read_size;
				file_size -= read_size;

			}
			fsync(fdd);
			close(fdd);
			fdd = -1;

			if(i == CONFIG_PMC_LDR){//��Ҫ��ȡpmc�����ļ�
				if(file_size < 9){
					printf("error in UnpackConfigBakFile, invalid file format\n");
					res = ERR_FILE_TRANS;
					goto END;
				}
				//��ȡ�ָ������������ļ�����
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

				//��ȡ�ļ���д������
				while(0 < cfg_file_size){
					bzero(buffer, kTcpBufferSize);
					read_block = cfg_file_size > kTcpBufferSize?kTcpBufferSize:cfg_file_size;
					read_size = read(fd, buffer, read_block);
					if(read_size != read_block){
						printf("error in UnpackConfigBakFile, read file error:%hhu\n", i);
						res = ERR_FILE_TRANS;
						goto END;
					}

					write(fdd, buffer, read_size);  //д���ļ�
					cfg_file_size -= read_size;
					file_size -= read_size;

				}
				fsync(fdd);
				close(fdd);
				fdd = -1;
			}else if(i == CONFIG_MACRO_VAR && chn_count > 1){  //��ͨ��ʱ������������
				for(j = 1; j < chn_count; j++){
					if(file_size < 9){
						printf("error in UnpackConfigBakFile, invalid file format\n");
						res = ERR_FILE_TRANS;
						goto END;
					}
					//��ȡ�ָ������������ļ�����
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

					//��ȡ�ļ���д������
					while(0 < cfg_file_size){
						bzero(buffer, kTcpBufferSize);
						read_block = cfg_file_size > kTcpBufferSize?kTcpBufferSize:cfg_file_size;
						read_size = read(fd, buffer, read_block);
						if(read_size != read_block){
							printf("error in UnpackConfigBakFile, read file error:%hhu\n", i);
							res = ERR_FILE_TRANS;
							goto END;
						}

						write(fdd, buffer, read_size);  //д���ļ�
						cfg_file_size -= read_size;
						file_size -= read_size;

					}
					fsync(fdd);
					close(fdd);
					fdd = -1;
				}
			}
			this->m_p_channel_engine->SetParamImportMask(i);   //��λ��־
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
 * @brief �����ļ�
 * @return
 */
int HMICommunication::RecvFile(){
	int res = ERR_NONE;
	uint64_t file_size = 0;  //�ļ���С
	int read_size = 0;      //��ȡ���Ĵ�С
	int read_block = 0;		//��ǰ��ȡ���ݴ�С
	uint64_t read_total = 0;  //�Ѷ�ȡ���ļ���С
	char buffer[kTcpBufferSize];	//����
	char filepath[kMaxPathLen];		//�ļ�·��
	char sign_path[kMaxPathLen] = {0};    //ǩ���ļ�·��
	char file_sign[kNcFileSignLen] = {0};  //ǩ��
	char file_name[kMaxFileNameLen] = {0};  //�ļ���
	int fd = -1;                    //�ļ����
	uint8_t file_type = FILE_NO_TYPE;  //�ļ���������
	uint8_t filename_size = 0;         //�ļ�������
	int error_count = 0;              //�������
	char *pSplit = nullptr;
	int recv_count = 0;

	uint8_t update_file_count = 0;   //�����ļ�����
	uint8_t module_type = 0;	//ģ������

	uint8_t err_count = 0;   //�����������������ʧ����澯

	HMICmdFrame cmd;

	bzero((char *)&cmd, kMaxHmiCmdFrameLen);
	cmd.cmd = CMD_SC_TRANS_FILE_OVER;   //�����Ӧ�����
	cmd.data_len = 1;

//	FILE *file_bak = nullptr;


	//���Ժ�ʱ
	struct timeval tvStart;
	struct timeval tvNow;
	unsigned int nTimeDelay = 0;

	gettimeofday(&tvStart, NULL);


	//��ȡ��������
	read_size = recv(m_soc_file_send, (char *)&file_type, 1, MSG_NOSIGNAL);  //����ģʽ
//	if(read_size != 1){
//		printf("error in recvFile fun, failed to get file type, read = %d, error = %d\n", read_size, errno);
//		g_ptr_trace->PrintLog(LOG_ALARM, "�����ļ������쳣��error = %d", errno);
//		res = ERR_FILE_TRANS;
//		goto END;
//	}
//	else
//		printf("file type %d\n", file_type);

	while(read_size != 1){
		printf("error in recvFile fun, failed to get file type, read_size = %d, error = %d\n",
				read_size, errno);

		if(++err_count > 3){
			g_ptr_trace->PrintLog(LOG_ALARM, "�����ļ������쳣��error = %d, errcount=%hhu", errno, err_count);
			res = ERR_FILE_TRANS;
			goto END;
		}else{
			usleep(10000);   //����10ms
			read_size = recv(m_soc_file_send, (char *)&file_type, 1, MSG_NOSIGNAL);  //����ģʽ
		}
	}
	printf("file type %d\n", file_type);
	cmd.cmd_extension = file_type;


	if(file_type == FILE_G_CODE){
		//��ȡ�ļ����Ƴ���
		read_size = recv(m_soc_file_send, (char *)&filename_size, 1, MSG_NOSIGNAL);  //����ģʽ
		if(read_size != 1){
			printf("error in recvFile fun, failed to get file name length, read = %d, error = %d\n", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}
		else
			printf("file name length %d\n", filename_size);

		//��ȡ�ļ�����
		bzero(buffer, kTcpBufferSize);
		read_size = recv(m_soc_file_send, buffer, filename_size, MSG_NOSIGNAL);  //����ģʽ
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
		strcat(filepath, buffer);  //��ϳ��ļ�����·��

		//���ǩ���ļ�����·��
		strcpy(sign_path, PATH_NC_SIGN_FILE);
		pSplit = strchr(buffer, '.');
		if(pSplit != nullptr)
			*pSplit = '\0';
		strcat(buffer, "_md5");
		strcat(sign_path, buffer);


		printf("file path = %s, signpath = %s\n", filepath, sign_path);

//		if(access(filepath, F_OK) == 0){	//�ļ�������ɾ���ļ�
//			if(!DeleteNcFile(filepath)){
//				printf("Failed to delete nc file[%s]\n", filepath);
//			}
//
//		}

		//��ȡ�ļ�ǩ��
		read_size = recv(m_soc_file_send, (char *)&file_sign, kNcFileSignLen, MSG_NOSIGNAL);  //����ģʽ
		if(read_size != kNcFileSignLen){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "error in recvFile fun, failed to get file signature, read = %d, error = %d", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}
	}else if(file_type == FILE_UPDATE){//��ģ�������ļ�
		//���ȶ�ȡ�ļ�����
		read_size = recv(m_soc_file_send, (char *)&update_file_count, 1, MSG_NOSIGNAL);  //����ģʽ
		if(read_size != 1){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "error in recvFile fun, failed to get update file count, read = %d, error = %d", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}
		printf("update file count: %hhu\n", update_file_count);

		//��ȡ����ģ������
		read_size = recv(m_soc_file_send, (char *)&module_type, 1, MSG_NOSIGNAL);  //����ģʽ
		if(read_size != 1){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "error in recvFile fun, failed to get update module type, read = %d, error = %d", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}

		printf("update module type : %hhu\n", module_type);

		//�����ļ��洢·��
		bzero(filepath, kMaxPathLen);
		if(!GetModuleUpdatePath(module_type, filepath)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "error in recvFile fun, invalid update module type, module_type = %hhu", module_type);
			res = ERR_FILE_TRANS;
			goto END;
		}


	}else if(file_type == FILE_PMC_LADDER){	//PMC��ͼ�ļ�

	}else if(file_type == FILE_CONFIG_PACK){  //��������ļ�
		bzero(filepath, kMaxPathLen);
		strcpy(filepath, PATH_CONFIG_PACK_TMP);
	}else if(file_type == FILE_PC_DATA){  //�ݲ������ļ�
		bzero(filepath, kMaxPathLen);
		strcpy(filepath, PATH_PC_DATA_TMP);
	}else if(file_type == FILE_ESB_DEV){  //�ŷ������ļ�
		//��ȡ�ļ����Ƴ���
		read_size = recv(m_soc_file_send, (char *)&filename_size, 1, MSG_NOSIGNAL);  //����ģʽ
		if(read_size != 1){
			printf("error in recvFile fun, failed to get file name length, read = %d, error = %d\n", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}
		else
			printf("file name length %d\n", filename_size);

		//��ȡ�ļ�����
		bzero(buffer, kTcpBufferSize);
		read_size = recv(m_soc_file_send, buffer, filename_size, MSG_NOSIGNAL);  //����ģʽ
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
		strcat(filepath, buffer);  //��ϳ��ļ�����·��

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

	//��ȡ�ļ���С
	read_size = recv(m_soc_file_send, (char *)&file_size, 8, MSG_NOSIGNAL);  //����ģʽ
	if(read_size != 8){
		printf("error in recvFile fun, failed to get file size, read = %d, error = %d\n", read_size, errno);
		res = ERR_FILE_TRANS;
		goto END;
	}
	else
		printf("file size %lld\n", file_size);

	//���ļ�
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
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "�ļ�����TCP�����쳣�жϣ�errno = %d", errno);
			break;
		}
		else if(error_count < 3){
			error_count++;
			usleep(5000);   //û�������ݵȴ�5ms
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
			//����ǩ���ļ�
			fd = open(sign_path, O_CREAT|O_WRONLY|O_TRUNC);
			if(fd > 0){
				write(fd, file_sign, kNcFileSignLen);
				close(fd);
				sync();
			}
			else
				g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "����NC�ļ�ǩ��ʧ�ܣ�");


			this->m_p_channel_engine->RemapFile(file_name);
		}
//		else if(file_type == FILE_MDA_CODE){
//			//�յ�����MDA���룬�������г���
//			m_p_channel_engine->Start();
//		}


		printf("Succeed to receive file over, filesize = %llu\n", file_size);
	}
	else{
		g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "�����ļ�[����%d]ʧ�ܣ��Ѷ�ȡ%lld�ֽ�, �ܳ�%lld�ֽڡ�", file_type, read_total, file_size);
		if(file_type == FILE_UPDATE)
			printf("update file module:%hhu\n", module_type);
		res = ERR_FILE_TRANS;
	}

	if(file_type == FILE_UPDATE && --update_file_count>0){//��ģ�������ļ�
		//��ȡ����ģ������
		read_size = recv(m_soc_file_send, (char *)&module_type, 1, MSG_NOSIGNAL);  //����ģʽ
		if(read_size != 1){
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "error in recvFile fun, failed to get update module type, read = %d, error = %d", read_size, errno);
			res = ERR_FILE_TRANS;
			goto END;
		}

		printf("update module type : %hhu\n", module_type);

		//�����ļ��洢·��
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
        sem_post(&m_sem_background);//ִ֪ͨ�к�̨����
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

	if(file_type == FILE_UPDATE && res == ERR_NONE){//���������ļ���������ʼ��������
		this->m_p_channel_engine->StartUpdateProcess();
	}else if(file_type == FILE_CONFIG_PACK && res ==ERR_NONE){  //���ô���ļ�����ʼ��ѹ����
		this->UnpackConfigBakFile();
	}else if(file_type == FILE_PC_DATA && res == ERR_NONE){//�����ݲ������ļ�����
		this->m_p_channel_engine->ProcessPcDataImport();
	}

	if(res != ERR_NONE){
		cmd.data[cmd.data_len-1] = FAILED;
		res = ERR_NONE;
	}

//#ifdef USES_MODBUS_PROTOCAL
//	//���ͻظ���
//	int len = kHmiCmdHeadLen+cmd.data_len;
//	int rb = sendto(this->m_soc_udp_send, &cmd, len, 0, (sockaddr *)&m_addr_file_trans, m_n_addr_len);
//	if(len != rb){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "�����ļ�����ָ�����ָ��ʧ��[host:%s]��", inet_ntoa(m_addr_file_trans.sin_addr));
//	}else{
//		printf("send recv file over cmd:ip[%s:%hu]\n", inet_ntoa(m_addr_file_trans.sin_addr), ntohs(m_addr_file_trans.sin_port));
//	}
//#else
	this->SendCmd(cmd);   //������Ӧ
//#endif


	return res;
}

/**
 * @brief ��ȡģ�������ļ�·��
 * @param module_type : ģ������
 * @param file_path[out] �� �ļ�·��
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
		g_ptr_trace->PrintTrace(TRACE_WARNING, HMI_COMMUNICATION, "��֧�ֵ�ģ������[%hhu]", module_type);
		res = false;
		break;
	}
END:
	return res;
}

/**
 * @brief ��ȡNC�ӹ��ļ�����
 * @return �ļ�����
 */
int HMICommunication::GetNcFileCount(const char *path){
	int count = 0;

	DIR *dir = opendir(path);
	if(dir == nullptr){
	//	printf("Failed to open dir [%s]!\n", PATH_NC_FILE);
		g_ptr_trace->PrintLog(LOG_ALARM, "���ļ�Ŀ¼ʧ�ܣ�[%s]", path);
        CreateError(ERR_OPEN_DIR, WARNING_LEVEL, CLEAR_BY_MCP_RESET, errno);
		return count;
	}

	struct dirent *dir_obj = nullptr;
	char *dir_name = nullptr;

	//ѭ����ȡĿ¼�µ��ļ�
	while(nullptr != (dir_obj = readdir(dir))){
		dir_name = dir_obj->d_name;
//		printf("read file %s\n", dir_name);
		if(strcmp(dir_name, ".") == 0 || strcmp(dir_name, "..") == 0){
			continue;
		}
		else if(dir_obj->d_type == DT_REG){   //��ͨ�ļ�
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
 * @brief ��ȡESB�ļ�����
 * @param path : ·��
 * @return ESB�ļ�����
 */
int HMICommunication::GetEsbFileCount(const char *path){
	int count = 0;
	DIR *dir = opendir(path);
	if(dir == nullptr){
		//����Ŀ¼
		if(mkdir(PATH_ESB_FILE, 0755) == -1){//����Ŀ¼ʧ��
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "����Ŀ¼ʧ�ܣ�[%s]", PATH_ESB_FILE);
            CreateError(ERR_OPEN_DIR, WARNING_LEVEL, CLEAR_BY_MCP_RESET, errno);
		}else{
			g_ptr_trace->PrintTrace(TRACE_INFO, HMI_COMMUNICATION, "��Ŀ¼[%s]ʧ�ܣ��Զ�������Ŀ¼��", PATH_ESB_FILE);
		}
		return count;
	}

	struct dirent *dir_obj = nullptr;
	char *dir_name = nullptr, *pt = nullptr;
	int len = 0;

	//ѭ����ȡĿ¼�µ��ļ�
	while(nullptr != (dir_obj = readdir(dir))){
		dir_name = dir_obj->d_name;

		if(dir_obj->d_type == DT_REG){   //��ͨ�ļ�
			//У���ļ���׺�ǲ���.esb
			len = strlen(dir_name);
			if(len > 4){
				pt = strrchr(dir_name, '.');
				if(pt && strcasecmp(pt, "esb")){  //��׺ƥ��
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
 * @brief ������ȡpathĿ¼���ļ�����ϸ��Ϣ
 * @param path[in] : Ŀ¼
 * @param size[out] : �ļ���С
 * @param time[out] : �ļ�����޸�ʱ��
 * @param mode[out] : �ļ�����
 * @return true--�ɹ�   false--ʧ��
 */
bool HMICommunication::GetNcFileInfo(const char *path, uint64_t &size, char *time, mode_t *mode){

	struct stat statbuf;
	if(stat(path, &statbuf) == -1){
		g_ptr_trace->PrintLog(LOG_ALARM, "���ļ�ʧ�ܣ�[%s]", path);
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
 * @brief �����ļ������쳣
 */
void HMICommunication::ProcessFileTransError(){
	this->m_b_trans_file = false;  //��λ�ļ������־
}

//@test zk
void get_memoccupy(MEM_OCCUPY *mem) //��������get��������һ���βνṹ����Ū��ָ��O
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
    //��fd�ļ��ж�ȡ����Ϊbuff���ַ����ٴ浽��ʼ��ַΪbuff����ռ���
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

    fclose(fd);     //�ر��ļ�fd
}


int get_cpuoccupy(CPU_OCCUPY *cpust) //��������get��������һ���βνṹ����Ū��ָ��O
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

    od = (unsigned long)(o->user + o->nice + o->system + o->idle + o->lowait + o->irq + o->softirq);//��һ��(�û�+���ȼ�+ϵͳ+����)��ʱ���ٸ���od
    nd = (unsigned long)(n->user + n->nice + n->system + n->idle + n->lowait + n->irq + n->softirq);//�ڶ���(�û�+���ȼ�+ϵͳ+����)��ʱ���ٸ���od
    double sum = nd - od;
    double idle = n->idle - o->idle;
    cpu_use = idle / sum;
    idle = n->user + n->system + n->nice - o->user - o->system - o->nice;
    cpu_use = idle / sum;
    //printf("---cpu use---: %.3f\n",cpu_use);
}
//@test zk
