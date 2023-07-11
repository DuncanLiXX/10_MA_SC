/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file HMICommunication.h
 *@author gonghao
 *@date 2020/03/19
 *@brief ��ͷ�ļ�ΪHMIͨѶ�������
 *@version
 */

#ifndef HMICOMMUNICATION_H_
#define HMICOMMUNICATION_H_

#include <circular_buffer.h>
#include <list_buffer.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <semaphore.h>
#include <dirent.h>
#include <map>
#include <memory>
#include <list>


//#include "hmi_shared_data.h"
#include "comm_data_definition.h"

class ChannelEngine;
class HMICommunication;

/**
 * @brief ���׵��ļ�ϵͳ������
 * @details ��ȡָ��Ŀ¼�¡��ļ�ϵͳ��Ϣ��
 *
 * ���ļ�ϵͳ��Ϣ����һ���б�ķ�ʽ���棬����Ϊָ����Ŀ¼�µ������ļ���Ŀ¼��Ϣ
 *
 * ��Ϣ���������
 *  - ���� (�ļ�: 0 �� Ŀ¼: 1)
 *  - ��С (��λ: byte)
 *  - ʱ�� (���һ���޸�ʱ��)
 *  - ���� (����·��)
 */
class FileSystemManager
{
public:
    FileSystemManager(HMICommunication *hmiCommmunication);

    /**
     * @brief SetRoot ���ø�Ŀ¼·��
     * @param root ��Ŀ¼·��
     * @return void
     */
    void SetRoot(string root);

    /**
     * @brief �����ļ��б���Ϣ
     * @return
     * @retval true    �ɹ�
     * @retval false   ʧ��
     */
    bool SendInfo();

    /**
     * @brief �����ļ��б�
     * @return
     * @retval true    �ɹ�
     * @retval false   ʧ��
     */
    bool UpdateInfo();

    /**
     * @brief �����ļ��б�
     * @return list<FS_Entity> �ļ��б�
     */
    std::list<FS_Entity> GetEntity() const;

private:
    FS_Entity GetInfo(const string &path);  // ��ȡ�ļ����������Ϣ
    bool ScanDir(string path = "");         // ɨ��Ŀ¼����ȡĿ¼�µ��ļ��б�

    HMICommunication *pCommunication;
    string root_path;                       // ��Ŀ¼
    std::list<FS_Entity> file_tree;         // �ļ��б�
};

// @test zk
typedef struct MEMPACKED         //����һ��mem occupy�Ľṹ��
{
    char name1[20];      //����һ��char���͵�������name��20��Ԫ��
    unsigned long MemTotal;
    char name2[20];
    unsigned long MemFree;
    char name3[20];
    unsigned long Buffers;
    char name4[20];
    unsigned long Cached;
    char name5[20];
    unsigned long SwapCached;
}MEM_OCCUPY;

//proc/stat�ļ��ṹ
//cpu  633666 46912 249878 176813696 782884 2859 19625 0
//cpu0 633666 46912 249878 176813696 782884 2859 19625 0
//intr 5812844
//ctxt 265816063
//btime 1455203832
//processes 596625
//procs_running 1
//procs_blocked 0

typedef struct CPUPACKED         //����һ��cpu occupy�Ľṹ��
{
    char name[20];      //����һ��char���͵�������name��20��Ԫ��
    unsigned int user; //����һ���޷��ŵ�int���͵�user
    unsigned int nice; //����һ���޷��ŵ�int���͵�nice
    unsigned int system;//����һ���޷��ŵ�int���͵�system
    unsigned int idle; //����һ���޷��ŵ�int���͵�idle
    unsigned int lowait;
    unsigned int irq;
    unsigned int softirq;
}CPU_OCCUPY;

enum BackgroundTask {
    Background_None,        //����
    Background_Pack,        //ѹ��
    Background_UnPack,      //��ѹ
};


double get_memoccupy(MEM_OCCUPY *mem); //��������get��������һ���βνṹ����Ū��ָ��O


int get_cpuoccupy(CPU_OCCUPY *cpust); //��������get��������һ���βνṹ����Ū��ָ��O


double cal_cpuoccupy(CPU_OCCUPY *o, CPU_OCCUPY *n);
// @test zk


/**
 * @brief SCģ����HMIģ����ͨѶ�ӿڣ�֧��UDP��TCP����
 */
class HMICommunication {
public:

	virtual ~HMICommunication();

	static HMICommunication *GetInstance();   //����ģʽ����ȡ����ʵ����Ψһ���ʵ�

	ErrorType GetErrorCode(){return m_error_code;}

	bool SendCmd(HMICmdFrame &cmd);           //����UDP����
	bool SendCmdToIp(HMICmdRecvNode &cmd_node);   //��ԴIP��������

	int SendMonitorData(char *buf, int size);  //���ͼ������

	void DisconnectToHmi();              //�ر���HMI������

	void Reset();                    //ִ�и�λ����

	void SetInterface();  //����ͨ������ָ��

	void ProcessFileTransError();  //�����ļ������쳣

private:
	HMICommunication();
	int Initialize();                  //��ʼ������
	static void SigioHandler(int signo);          //SIGIO�źŴ�����
	static void *UdpResendCmdThread(void *args);  //UDP�����ط������̺߳���
	static void *UdpProcessCmdThread(void *args); //UDP����������̺߳���
	static void *TcpMonitorThread(void *args);    //TCP������ݷ����̺߳���
	static void *TcpTransFileThread(void *args);   //TCP�ļ������̺߳���
//	static void SendMonitorData(int sig);     //���ͼ������
	static void *SaveAsFileThread(void *args);   //�ļ����Ϊ�̺߳���
    static void *BackgroundThread(void *args);      //���ڴ���HMI�·��ĺ�ʱ����

	int Clean();     //�����ƺ�����

	int ResendHmiCmd();                  //HMI�����ط�����
	int ProcessHmiCmd();                 //������յ�HMI����
	void RecvHmiCmd();					//����HMI��UDP����
	int Monitor();                       //������ݷ���
	int TransFile();                     //�����ļ�
	int SaveasFileFunc(const char *path_old, const char *path_new);             //�ļ����Ϊ
    void BackgroundFunc();               //��ʱ�����̨����

//	void SendTestFunc();   //�Է����ղ���
	int SendFile();                      //�����ļ�
	int RecvFile();                       //�����ļ�
	int SendConfigPackFile();             //�������ô���ļ�
	uint64_t GetConfigPackFileSize();             //�������ô���ļ����ֽ���
    void PackageSysBackupFile();             //ѹ�������ļ�
    void UnPackageBackupFile();              //��ѹ�����ļ�
    void SendHMIBackupStatus(SysUpdateStatus status);   //��HMI���͵�ǰ����/�ָ�״̬

	int ResendCmd(HMICmdFrame &cmd);        //�ط�UDP��������뷢�Ͷ���

	bool DelCmd(uint16_t frame_index);   //�ڷ��Ͷ�����ɾ����Ӧ����

	void ProcessHmiDeviceScan(HMICmdRecvNode &cmd_node);   //����HMI�������豸ɨ��ָ��
	void ProcessHmiShakehand(HMICmdRecvNode &cmd_node);    //����HMI����������ָ��
	void ProcessHmiGetFileCmd(HMICmdFrame &cmd);   		  //����HMI��ȡ�ļ�ָ��
	void ProcessHmiSetCurChnCmd(HMICmdFrame &cmd);        //����HMI���õ�ǰͨ������
//#ifdef USES_MODBUS_PROTOCAL
//	void ProcessHmiGetFileCmd(HMICmdRecvNode &cmd_node);   //����HMI��ȡ�ļ�ָ��
//#endif
	void ProcessHmiSendFileCmd(HMICmdFrame &cmd);  		  //����HMI�����ļ�ָ��
//#ifdef USES_MODBUS_PROTOCAL
//	void ProcessHmiSendFileCmd(HMICmdRecvNode &cmd_node);  //����HMI�����ļ�ָ��
//#endif
	void ProcessHmiDisconnectCmd(HMICmdFrame &cmd);  	  //����HMI�ж�����ָ��
	void ProcessHmiReadyCmd(HMICmdFrame &cmd);             //����HMI��������
	void ProcessHmiNewAlarmCmd(HMICmdFrame &cmd);          //����HMI�澯ָ��
	void ProcessHmiGetVersionCmd(HMICmdFrame &cmd);       //����HMI��ȡ�汾��Ϣָ��
	void ProcessHmiChnCountCmd(HMICmdFrame &cmd);    	  //����HMI��ȡͨ��������
	void ProcessHmiGetChnStateCmd(HMICmdFrame &cmd);   	  //����HMI��ȡͨ��״ָ̬��
	void ProcessHmiNcFileCountCmd(HMICmdRecvNode &cmd_node);   	  //����HMI��ȡNC�ļ�����ָ��
	void ProcessHmiNcFileInfoCmd(HMICmdRecvNode &cmd_node);    	  //����HMI��ȡnc�ļ���ϸ��Ϣ����

    void ProcessHmiNcFileSystemCmd(HMICmdRecvNode &cmd_node);     //����HMI��ȡnc�ļ�ϵͳ
    void ProcessHmiMkdirCmd(HMICmdRecvNode &cmd_node);            //����HMI��ȡ����Ŀ¼����

	void ProcessHmiFileOperateCmd(HMICmdRecvNode &cmd_node);      //����HMI�������ļ���������
	void ProcessHmiFileSignatureCmd(HMICmdRecvNode &cmd_node);		//����HMI��ȡNC�ļ�ǩ��������
	void ProcessHmiVirtualMOPCmd(HMICmdFrame &cmd);			//����HMI����MOP��������
	void ProcessHmiSyncTimeCmd(HMICmdFrame &cmd);          //����HMIͬ��ϵͳʱ������
    void ProcessHmiSysBackupCmd(HMICmdFrame &cmd);         //����HMI��������
    void ProcessHmiSysRecoverCmd(HMICmdFrame &cmd);        //����HMI�ָ�����
    void ProcessHmiClearAlarmFile(HMICmdFrame &cmd);       //����HMI��������ļ�����
    void ProcessHmiGetCPUInfo(HMICmdFrame &cmd);					   //����HMI��ȡCPU �ڴ�ռ����Ϣ
    void ProcessHmiServoDataRequest(HMICmdFrame &cmd);     //����HMI����ʼ�ŷ�����
    void ProcessHmiServoDataReset(HMICmdFrame &cmd);       //����HMI�����ŷ�������λ
    void ProcessHmiReady();
	uint16_t GetFrameNumber(){return  (++m_n_frame_number&0x8000) ? (m_n_frame_number=0) : m_n_frame_number;}   //��ȡ����֡��

	int GetNcFileCount(const char *path);    //��ȡNC�ӹ��ļ�����
	bool GetNcFileInfo(const char *path, uint64_t &size, char *time=nullptr, mode_t *mode=nullptr);    //������ȡpathĿ¼���ļ�����ϸ��Ϣ
    bool DeleteNc(const string &name);
    bool DeleteNcFile(const char *name);		//ɾ���ļ�
    bool DeleteNcDir(string name);
	bool RenameNcFile(const char *old_name, const char *new_name);		//�������ļ�
	bool SaveasNcFile(const char *old_name, const char *new_name);		//���Ϊ�ļ�


	int GetEsbFileCount(const char *path);   //��ȡESB�ļ�����
	void ProcessHmiGetEsbInfoCmd(HMICmdFrame &cmd);      //����HMI��ȡESB�ļ���Ϣ������
	void ProcessHmiOperateEsbCmd(HMICmdFrame &cmd);      //����HMI����ESB�ļ�������
	bool DeleteEsbFile(const char *name);		//ɾ���ļ�
	bool RenameEsbFile(const char *old_name, const char *new_name);		//�������ļ�
	bool SaveasEsbFile(const char *old_name, const char *new_name);		//���Ϊ�ļ�

	bool GetModuleUpdatePath(uint8_t module_type, char *file_path);     //��ȡģ�������ļ�·��

	uint64_t GetFileLength(const char *path);    //��ȡ�ļ�����
	bool GetConfigFilePath(char *path, uint8_t type);  //��ȡ�����ļ�·��

	int UnpackConfigBakFile();      //��ѹ�������ô���ļ�

private:
	static HMICommunication *m_p_instance;    //��ʵ������
	static sem_t m_sem_udp_recv;     //���յ�UDP������ź�
	sem_t m_sem_tcp_file;             //����tcp�ļ������ź�
    sem_t m_sem_background;           //��̨�̹߳����ź�
//	sem_t m_sem_tcp_send_test;        //tcp��������ź�

	ChannelEngine *m_p_channel_engine;   //ͨ������ָ��

	static int m_thread_run_flag;         //�߳����б�־

	static unsigned int m_n_hmi_heartbeat;           //HMI��������

	static int m_soc_udp_recv;    //UDP�������socket
	int m_soc_udp_send;    //UDP�����socket
	int m_soc_tcp_monitor;   //TCP������ݼ���socket
	int m_soc_monitor_send;  //������ݷ���socket
	int m_soc_tcp_file;      //TCP�ļ��������socket
	int m_soc_file_send;     //�ļ�����socket

	struct sockaddr_in m_addr_udp_recv;     //����HMI����ĵ�ַ
    static struct sockaddr_in m_addr_udp_hmi;      //HMI��UDP�����ַ
    static socklen_t m_n_addr_len;            //HMI��ַ����
//#ifdef USES_MODBUS_PROTOCAL
//    struct sockaddr_in m_addr_file_trans;   //modbusЭ�����ļ�����ip
//#endif

	static pthread_mutex_t m_mutex_udp_recv;   //UDP�������ݻ�����
	pthread_mutex_t m_mutex_udp_send;   //UDP�������ݻ�����,���ڷ�ֹ�ڲ�ͬ�ĵط�ͬʱ��ɾ������

	pthread_mutex_t m_mutex_tcp_monitor;  //����TCP���ݻ�����

	static CircularBuffer<HMICmdRecvNode> *m_list_recv;   //���յ���HMI��������λ���
	ListBuffer<HMICmdResendNode *> *m_list_send;   //���͵�HMI�����������

	pthread_t m_thread_process_cmd;    //HMI������߳�
	pthread_t m_thread_resend_cmd;     //HMI�����ط��߳�
	pthread_t m_thread_monitor;        //TCP������ݷ����߳�
	pthread_t m_thread_trans_file;     //TCP�ļ������̣߳����������ļ����ӹ��ļ��������ļ���
	pthread_t m_thread_saveas;         //NC�ļ����Ϊ�߳�
    pthread_t m_thread_background;     //����HMI�����ĺ�ʱ����

	uint16_t m_n_frame_number;         //��ǰ֡��
	bool m_b_trans_file;              //��ǰ�Ƿ��ڴ����ļ�
	FileTransType m_file_type;              //�ļ���������
	char m_str_file_name[kMaxFileNameLen];             //�������ļ�����
	bool m_b_recv_file;              //��־�ļ����������Ƿ��ͻ��ǽ��գ�true��ʾ�����ļ���false��ʾ�����ļ�
	uint32_t m_mask_config_pack;      //�����ļ����mask
    int m_maks_sys_backup = 0;                      //ϵͳ����mask
    int m_background_type = Background_None;        //��̨��������
    SysUpdateStatus m_sysbackup_status = SysUpdateStatus(); //����/�ָ�״̬
//	uint8_t m_n_alarm_file_type;      //�����͵ĸ澯��ʷ�ļ�����

	ErrorType m_error_code;           //������
	MEM_OCCUPY mem_stat;
	CPU_OCCUPY cpu_stat;
	double cpu_percent;  //cpuռ����
	double mem_percent;  //�ڴ�ռ����
	char big_frame_buffer[10240];

    std::map<int, std::shared_ptr<FileSystemManager>> fileSystem;
};

//�����ļ����Ϊ�̴߳��ݲ���
struct SaveAsParam{
	HMICommunication *pComm;		//HMIͨѶ�ӿڶ���ָ��
	char path_old[kMaxPathLen];   //ԭ�ļ�����·��
	char path_new[kMaxPathLen];   //���ļ�����·��
};


#endif /* HMICOMMUNICATION_H_ */
