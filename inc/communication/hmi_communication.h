/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file HMICommunication.h
 *@author gonghao
 *@date 2020/03/19
 *@brief 本头文件为HMI通讯类的声明
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
 * @brief 简易的文件系统管理类
 * @details 获取指定目录下‘文件系统信息’
 *
 * ‘文件系统信息’以一个列表的方式保存，内容为指定根目录下的所有文件和目录信息
 *
 * 信息包括如下项：
 *  - 类型 (文件: 0 或 目录: 1)
 *  - 大小 (单位: byte)
 *  - 时间 (最后一次修改时间)
 *  - 名称 (包括路径)
 */
class FileSystemManager
{
public:
    FileSystemManager(HMICommunication *hmiCommmunication);

    /**
     * @brief SetRoot 设置根目录路径
     * @param root 根目录路径
     * @return void
     */
    void SetRoot(string root);

    /**
     * @brief 发送文件列表信息
     * @return
     * @retval true    成功
     * @retval false   失败
     */
    bool SendInfo();

    /**
     * @brief 更新文件列表
     * @return
     * @retval true    成功
     * @retval false   失败
     */
    bool UpdateInfo();

    /**
     * @brief 返回文件列表
     * @return list<FS_Entity> 文件列表
     */
    std::list<FS_Entity> GetEntity() const;

private:
    FS_Entity GetInfo(const string &path);  // 获取文件属性相关信息
    bool ScanDir(string path = "");         // 扫描目录，获取目录下的文件列表

    HMICommunication *pCommunication;
    string root_path;                       // 根目录
    std::list<FS_Entity> file_tree;         // 文件列表
};

// @test zk
typedef struct MEMPACKED         //定义一个mem occupy的结构体
{
    char name1[20];      //定义一个char类型的数组名name有20个元素
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

//proc/stat文件结构
//cpu  633666 46912 249878 176813696 782884 2859 19625 0
//cpu0 633666 46912 249878 176813696 782884 2859 19625 0
//intr 5812844
//ctxt 265816063
//btime 1455203832
//processes 596625
//procs_running 1
//procs_blocked 0

typedef struct CPUPACKED         //定义一个cpu occupy的结构体
{
    char name[20];      //定义一个char类型的数组名name有20个元素
    unsigned int user; //定义一个无符号的int类型的user
    unsigned int nice; //定义一个无符号的int类型的nice
    unsigned int system;//定义一个无符号的int类型的system
    unsigned int idle; //定义一个无符号的int类型的idle
    unsigned int lowait;
    unsigned int irq;
    unsigned int softirq;
}CPU_OCCUPY;

enum BackgroundTask {
    Background_None,        //空闲
    Background_Pack,        //压缩
    Background_UnPack,      //解压
};


double get_memoccupy(MEM_OCCUPY *mem); //对无类型get函数含有一个形参结构体类弄的指针O


int get_cpuoccupy(CPU_OCCUPY *cpust); //对无类型get函数含有一个形参结构体类弄的指针O


double cal_cpuoccupy(CPU_OCCUPY *o, CPU_OCCUPY *n);
// @test zk


/**
 * @brief SC模块与HMI模块间的通讯接口，支持UDP及TCP连接
 */
class HMICommunication {
public:

	virtual ~HMICommunication();

	static HMICommunication *GetInstance();   //单例模式，获取此类实例的唯一访问点

	ErrorType GetErrorCode(){return m_error_code;}

	bool SendCmd(HMICmdFrame &cmd);           //发送UDP命令
	bool SendCmdToIp(HMICmdRecvNode &cmd_node);   //向源IP发送命令

	int SendMonitorData(char *buf, int size);  //发送监控数据

	void DisconnectToHmi();              //关闭与HMI的链接

	void Reset();                    //执行复位动作

	void SetInterface();  //设置通道引擎指针

	void ProcessFileTransError();  //处理文件传输异常

private:
	HMICommunication();
	int Initialize();                  //初始化函数
	static void SigioHandler(int signo);          //SIGIO信号处理函数
	static void *UdpResendCmdThread(void *args);  //UDP命令重发处理线程函数
	static void *UdpProcessCmdThread(void *args); //UDP接收命令处理线程函数
	static void *TcpMonitorThread(void *args);    //TCP监控数据发送线程函数
	static void *TcpTransFileThread(void *args);   //TCP文件传输线程函数
//	static void SendMonitorData(int sig);     //发送监控数据
	static void *SaveAsFileThread(void *args);   //文件另存为线程函数
    static void *BackgroundThread(void *args);      //用于处理HMI下发的耗时命令

	int Clean();     //处理善后事宜

	int ResendHmiCmd();                  //HMI命令重发函数
	int ProcessHmiCmd();                 //处理接收的HMI命令
	void RecvHmiCmd();					//接收HMI的UDP命令
	int Monitor();                       //监控数据发送
	int TransFile();                     //传输文件
	int SaveasFileFunc(const char *path_old, const char *path_new);             //文件另存为
    void BackgroundFunc();               //耗时命令后台处理

//	void SendTestFunc();   //自发自收测试
	int SendFile();                      //发送文件
	int RecvFile();                       //接收文件
	int SendConfigPackFile();             //发送配置打包文件
	uint64_t GetConfigPackFileSize();             //计算配置打包文件总字节数
    void PackageSysBackupFile();             //压缩备份文件
    void UnPackageBackupFile();              //解压备份文件
    void SendHMIBackupStatus(SysUpdateStatus status);   //向HMI发送当前备份/恢复状态

	int ResendCmd(HMICmdFrame &cmd);        //重发UDP命令，不加入发送队列

	bool DelCmd(uint16_t frame_index);   //在发送队列中删除对应报文

	void ProcessHmiDeviceScan(HMICmdRecvNode &cmd_node);   //处理HMI发出的设备扫描指令
	void ProcessHmiShakehand(HMICmdRecvNode &cmd_node);    //处理HMI发出的握手指令
	void ProcessHmiGetFileCmd(HMICmdFrame &cmd);   		  //处理HMI获取文件指令
	void ProcessHmiSetCurChnCmd(HMICmdFrame &cmd);        //处理HMI设置当前通道命令
//#ifdef USES_MODBUS_PROTOCAL
//	void ProcessHmiGetFileCmd(HMICmdRecvNode &cmd_node);   //处理HMI获取文件指令
//#endif
	void ProcessHmiSendFileCmd(HMICmdFrame &cmd);  		  //处理HMI发送文件指令
//#ifdef USES_MODBUS_PROTOCAL
//	void ProcessHmiSendFileCmd(HMICmdRecvNode &cmd_node);  //处理HMI发送文件指令
//#endif
	void ProcessHmiDisconnectCmd(HMICmdFrame &cmd);  	  //处理HMI中断连接指令
	void ProcessHmiReadyCmd(HMICmdFrame &cmd);             //处理HMI就绪命令
	void ProcessHmiNewAlarmCmd(HMICmdFrame &cmd);          //处理HMI告警指令
	void ProcessHmiGetVersionCmd(HMICmdFrame &cmd);       //处理HMI获取版本信息指令
	void ProcessHmiChnCountCmd(HMICmdFrame &cmd);    	  //处理HMI获取通道数命令
	void ProcessHmiGetChnStateCmd(HMICmdFrame &cmd);   	  //处理HMI获取通道状态指令
	void ProcessHmiNcFileCountCmd(HMICmdRecvNode &cmd_node);   	  //处理HMI获取NC文件个数指令
	void ProcessHmiNcFileInfoCmd(HMICmdRecvNode &cmd_node);    	  //处理HMI获取nc文件详细信息命令

    void ProcessHmiNcFileSystemCmd(HMICmdRecvNode &cmd_node);     //处理HMI获取nc文件系统
    void ProcessHmiMkdirCmd(HMICmdRecvNode &cmd_node);            //处理HMI获取创建目录命令

	void ProcessHmiFileOperateCmd(HMICmdRecvNode &cmd_node);      //处理HMI发来的文件操作命令
	void ProcessHmiFileSignatureCmd(HMICmdRecvNode &cmd_node);		//处理HMI获取NC文件签名的命令
	void ProcessHmiVirtualMOPCmd(HMICmdFrame &cmd);			//处理HMI虚拟MOP按键命令
	void ProcessHmiSyncTimeCmd(HMICmdFrame &cmd);          //处理HMI同步系统时间命令
    void ProcessHmiSysBackupCmd(HMICmdFrame &cmd);         //处理HMI备份命令
    void ProcessHmiSysRecoverCmd(HMICmdFrame &cmd);        //处理HMI恢复命令
    void ProcessHmiClearAlarmFile(HMICmdFrame &cmd);       //处理HMI清除报警文件命令
    void ProcessHmiGetCPUInfo(HMICmdFrame &cmd);					   //处理HMI获取CPU 内存占用信息
    void ProcessHmiServoDataRequest(HMICmdFrame &cmd);     //处理HMI请求开始伺服引导
    void ProcessHmiServoDataReset(HMICmdFrame &cmd);       //处理HMI请求伺服引导复位
    void ProcessHmiReady();
	uint16_t GetFrameNumber(){return  (++m_n_frame_number&0x8000) ? (m_n_frame_number=0) : m_n_frame_number;}   //获取最新帧号

	int GetNcFileCount(const char *path);    //获取NC加工文件个数
	bool GetNcFileInfo(const char *path, uint64_t &size, char *time=nullptr, mode_t *mode=nullptr);    //遍历获取path目录下文件的详细信息
    bool DeleteNc(const string &name);
    bool DeleteNcFile(const char *name);		//删除文件
    bool DeleteNcDir(string name);
	bool RenameNcFile(const char *old_name, const char *new_name);		//重命名文件
	bool SaveasNcFile(const char *old_name, const char *new_name);		//另存为文件


	int GetEsbFileCount(const char *path);   //获取ESB文件数量
	void ProcessHmiGetEsbInfoCmd(HMICmdFrame &cmd);      //处理HMI获取ESB文件信息的命令
	void ProcessHmiOperateEsbCmd(HMICmdFrame &cmd);      //处理HMI操作ESB文件的命令
	bool DeleteEsbFile(const char *name);		//删除文件
	bool RenameEsbFile(const char *old_name, const char *new_name);		//重命名文件
	bool SaveasEsbFile(const char *old_name, const char *new_name);		//另存为文件

	bool GetModuleUpdatePath(uint8_t module_type, char *file_path);     //获取模块升级文件路径

	uint64_t GetFileLength(const char *path);    //获取文件长度
	bool GetConfigFilePath(char *path, uint8_t type);  //获取配置文件路径

	int UnpackConfigBakFile();      //解压处理配置打包文件

private:
	static HMICommunication *m_p_instance;    //单实例对象
	static sem_t m_sem_udp_recv;     //接收到UDP命令的信号
	sem_t m_sem_tcp_file;             //开启tcp文件传输信号
    sem_t m_sem_background;           //后台线程工作信号
//	sem_t m_sem_tcp_send_test;        //tcp传输测试信号

	ChannelEngine *m_p_channel_engine;   //通道引擎指针

	static int m_thread_run_flag;         //线程运行标志

	static unsigned int m_n_hmi_heartbeat;           //HMI心跳计数

	static int m_soc_udp_recv;    //UDP命令接收socket
	int m_soc_udp_send;    //UDP命令发送socket
	int m_soc_tcp_monitor;   //TCP监控数据监听socket
	int m_soc_monitor_send;  //监控数据发送socket
	int m_soc_tcp_file;      //TCP文件传输监听socket
	int m_soc_file_send;     //文件传输socket

	struct sockaddr_in m_addr_udp_recv;     //接收HMI命令的地址
    static struct sockaddr_in m_addr_udp_hmi;      //HMI的UDP命令地址
    static socklen_t m_n_addr_len;            //HMI地址长度
//#ifdef USES_MODBUS_PROTOCAL
//    struct sockaddr_in m_addr_file_trans;   //modbus协议下文件传输ip
//#endif

	static pthread_mutex_t m_mutex_udp_recv;   //UDP接收数据互斥锁
	pthread_mutex_t m_mutex_udp_send;   //UDP发送数据互斥锁,用于防止在不同的地方同时做删除动作

	pthread_mutex_t m_mutex_tcp_monitor;  //发送TCP数据互斥锁

	static CircularBuffer<HMICmdRecvNode> *m_list_recv;   //接收到的HMI命令包环形缓冲
	ListBuffer<HMICmdResendNode *> *m_list_send;   //发送到HMI的命令包队列

	pthread_t m_thread_process_cmd;    //HMI命令处理线程
	pthread_t m_thread_resend_cmd;     //HMI命令重发线程
	pthread_t m_thread_monitor;        //TCP监控数据发送线程
	pthread_t m_thread_trans_file;     //TCP文件传输线程，包括参数文件、加工文件，升级文件等
	pthread_t m_thread_saveas;         //NC文件另存为线程
    pthread_t m_thread_background;     //出来HMI发来的耗时命令

	uint16_t m_n_frame_number;         //当前帧号
	bool m_b_trans_file;              //当前是否在传输文件
	FileTransType m_file_type;              //文件传输类型
	char m_str_file_name[kMaxFileNameLen];             //待传输文件名称
	bool m_b_recv_file;              //标志文件传输类型是发送还是接收，true表示接收文件，false表示发送文件
	uint32_t m_mask_config_pack;      //配置文件打包mask
    int m_maks_sys_backup = 0;                      //系统备份mask
    int m_background_type = Background_None;        //后台任务类型
    SysUpdateStatus m_sysbackup_status = SysUpdateStatus(); //备份/恢复状态
//	uint8_t m_n_alarm_file_type;      //待发送的告警历史文件类型

	ErrorType m_error_code;           //错误码
	MEM_OCCUPY mem_stat;
	CPU_OCCUPY cpu_stat;
	double cpu_percent;  //cpu占用率
	double mem_percent;  //内存占用率
	char big_frame_buffer[10240];

    std::map<int, std::shared_ptr<FileSystemManager>> fileSystem;
};

//用于文件另存为线程传递参数
struct SaveAsParam{
	HMICommunication *pComm;		//HMI通讯接口对象指针
	char path_old[kMaxPathLen];   //原文件绝对路径
	char path_new[kMaxPathLen];   //新文件绝对路径
};


#endif /* HMICOMMUNICATION_H_ */
