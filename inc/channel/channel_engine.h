/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file channel_engine.h
 *@author gonghao
 *@date 2020/03/19
 *@brief 本头文件为通道引擎类的声明
 *@version
 */

#ifndef INC_CHANNEL_CHANNEL_ENGINE_H_
#define INC_CHANNEL_CHANNEL_ENGINE_H_


#include "global_definition.h"
#include "global_structure.h"
#include "hmi_shared_data.h"
#include "comm_data_definition.h"
#include "pmc_axis_ctrl.h"
#include "channel_data.h"
#include "parm_manager.h"

#include "license_interface.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <signal.h>

#include "pmc_register.h"


//前置声明
class ChannelControl;   //通道控制类
class HMICommunication; //HMI通讯类
class MICommunication;	//MI通讯类
class MCCommunication;	//MC通讯类
class MCArmCommunication;  //MC-ARM通讯类
class ParmManager;      //配置管理类
class PmcRegister;		//PMC寄存器类
class ChannelModeGroup;   //通道方式组类
class SyncAxisCtrl;     //同步轴控制类
struct SCSystemConfig; //SC通道配置
struct SCChannelConfig;  //SC通道配置
struct SCAxisConfig;	//SC轴配置
struct ParamUpdate;		//参数升级结构



/**
 * @brief 通道引擎类，负责统筹处理通道间的交互以及外部交互
 */
class ChannelEngine {
public:

	static ChannelEngine *GetInstance();   //单例模式，获取此类实例的唯一访问点
	virtual ~ChannelEngine();  //析构函数

	void Initialize(HMICommunication *hmi_comm, MICommunication *mi_comm,
			MCCommunication *mc_comm, ParmManager *parm);   //设置接口

void SetMcArmComm(MCArmCommunication *comm){this->m_p_mc_arm_comm = comm;}   //设置MC-ARM通讯接口
	bool IsMcArmChn(uint8_t chn){return this->m_mc_run_on_arm[chn];}   //是否通道MC运行于ARM上
	void InitBdioDev();   //初始化SD-LINK从站设备配置

#ifdef USES_PMC_2_0
    void ReadIoDev_pmc2();                               //PMC2.0版本，读取SD_LINK从站设备配置
    bool CheckIoDev_pmc2(const BdioDevInfo &info);       //检测SD_LINK参数是否合理
    bool SelectHandleWheel(int indexId, int channelId);  //配置手轮通道映射
#else
	void ReadIoDev_pmc1();     //PMC1.0版本，读取SD_LINK从站设备配置
#endif

	void SendMonitorData(bool bAxis, bool btime);   //发送监控数据
	void SendPmcAxisToHmi();    //发送PMC轴位置给HMI

	void ProcessHmiCmd(HMICmdFrame &cmd);  //处理HMI指令
	void ProcessMcCmdRsp(McCmdFrame &rsp);	//处理MC模块的指令回复
	void ProcessMiCmd(MiCmdFrame &cmd);		//处理MI模块的指令


	int GetChnCount(){return this->m_p_general_config->chn_count;}     //返回当前通道数量
	ChannelControl *GetChnControl(){return this->m_p_channel_control;}   //获取所有通道控制对象数组的的指针
	ChannelControl *GetChnControl(uint8_t index);		//获取指定的通道控制对象指针
	bool GetChnStatus(uint8_t chn_index, HmiChannelStatus &status);  //获取通道状态
	uint8_t GetChnAxistoPhyAixs(uint8_t chn_index, uint8_t chn_axis);  //获取通道轴对应的物理轴号

//	uint16_t GetMcAutoBufMax(){return m_n_mc_auto_buf_max;}   //获取MC单通道自动数据缓冲数量


	bool Start();   //启动程序，用于响应循环启动
	bool Pause();   //暂停程序，用于响应进给保持
	bool Stop(bool reset);	 //停止程序执行，用于处理系统告警下的程序停止
	bool Stop(uint8_t chn, bool reset);  //停止程序执行，用于处理系统告警下的程序停止
	bool SetCurWorkChanl(uint8_t work_chan);   //设置当前通道号
    void ServoOn();     //对除主轴外的所有轴上使能
    void ServoOff();	//对除主轴外的所有轴下使能

#ifdef USES_EMERGENCY_DEC_STOP
    void DelayToServoOff(uint8_t chn_index);  //延迟下伺服

    void SetChnStoppedMask(uint8_t chn_index);   //设置通道停止到位标志
#endif

	bool SetWorkMode(uint8_t work_mode);   //设置工作模式

	void ShakeHandWithMc();    	//向MC模块发送握手命令
	void ShakeHandWithMi();		//向MI模块发送握手命令

	void SetFuncState(uint8_t chn, int state, uint8_t mode = 10);		//设置功能状态，例如：单段，选停等等
	void SetAutoRatio(uint8_t ratio);    //设置自动倍率
	void SetAutoRatio(uint8_t chn, uint8_t ratio);  //设置自动倍率
	void SetManualRatio(uint8_t ratio);	//设置手动倍率
	void SetManualRatio(uint8_t chn, uint8_t ratio);	//设置手动倍率
	void SetRapidRatio(uint8_t ratio);	//设置快速进给倍率
	void SetRapidRatio(uint8_t chn, uint8_t ratio);	//设置快速进给倍率
	void SetManualStep(uint16_t step);	//设置手动步长
	void SetManualStep(uint8_t chn, uint8_t step);  //设置手动步长
	void SetManualRapidMove(uint8_t mode = 10);			//设置手动快速移动状态
	void SetCurAxis(uint8_t axis);		//设置当前轴
	void SetCurAxis(uint8_t chn, uint8_t axis);  //设置当前轴
	void ChangeChnGroupIndex(uint8_t chn, uint8_t group_old, uint8_t group_new);   //改变指定通道的所属方式组

	void SetChnMachineState(uint8_t chn, uint8_t mach_state);   //设置通道加工状态

	void EnableHWTraceToMi();     //向MI更新手轮反向跟踪使能

//	void SetAxisOn(uint8_t index);			//设置轴使能，以及插补模式（1--NC轴插补   2--PMC轴插补）
//	void SetAxisBaseParam(uint8_t index);		//设置指定轴的螺距及最大速度等基本信息
//	void SetAxisSpeedParam(uint8_t index);		//设置指定轴的速度相关信息
//	void SetAxisAccParam(uint8_t index);			//设置指定轴加速度相关信息


	void ManualMove(int8_t dir);		//手动移动
	void ManualMoveStop();			//手动停止
	void ManualMove(uint8_t phy_axis, int8_t dir, double vel, double inc_dis);  //手动移动，向dir方向移动dis距离
	void ManualMoveAbs(uint8_t phy_axis, double vel, double pos);   //手动以目标速度vel移动至绝对目标位置

	void ManualMoveStop(uint8_t phy_axis);			//手动停止

	void ManualMovePmc(int8_t dir);		//手动移动
	void ManualMovePmc(uint8_t phy_axis, int8_t dir);   //指定轴手动移动
	void ManualMovePmc(uint8_t phy_axis, double tar_pos, double vel, bool inc);  //指定轴以指定速度移动到指定位置
	void ManualMovePmcStop();			//手动停止


	void PausePmcAxis(uint8_t phy_axis, bool flag);          //暂停PMC轴移动

	double GetPhyAxisMachPosFeedback(uint8_t index){return m_df_phy_axis_pos_feedback[index];}  //获取指定物理轴的当前反馈机械坐标

#ifdef USES_SPEED_TORQUE_CTRL
	double GetPhyAxisMachSpeedFeedback(uint8_t index){return m_df_phy_axis_speed_feedback[index];}  //获取指定物理轴的当前反馈速度
	double GetPhyAxisMachTorqueFeedback(uint8_t index){return m_df_phy_axis_torque_feedback[index];}  //获取指定物理轴的当前反馈力矩
#endif
	
	double GetPhyAxisMachPosIntp(uint8_t index){return m_df_phy_axis_pos_intp[index];}   //返回指定物理轴的当前插补机械坐标
	void StartUpdateProcess();		//开始模块升级操作

	void ClearAlarm();				//清除告警
	void SystemReset();				//系统复位
	void Emergency(uint8_t chn = CHANNEL_ENGINE_INDEX);					//急停处理


	bool GetMacroVarValue(uint8_t chn, uint32_t index, bool &init, double &value);   	//获取宏变量的值
	bool SetMacroVarValue(uint8_t chn, uint32_t index, bool init, double value);		//设置宏变量的值

	bool GetPmcRegValue_8(uint16_t reg_sec, uint16_t reg_index, uint8_t &value);			//获取PMC寄存器的值
	bool GetPmcRegValue_16(uint16_t reg_sec, uint16_t reg_index, uint16_t &value);			//获取PMC寄存器的值
	bool SetPmcRegValue_8(uint16_t reg_sec, uint16_t reg_index, uint8_t value);           //设置PMC寄存器的值
	bool SetPmcRegBitValue_8(uint16_t reg_sec, uint16_t reg_index, uint8_t bit_index, bool value);    //按位设置寄存器值
	bool SetPmcRegValue_16(uint16_t reg_sec, uint16_t reg_index, uint16_t value);           //设置PMC寄存器的值


	FRegBits *GetChnFRegBits(uint8_t chn_index);     //获取指定通道的F寄存器指针
	const GRegBits *GetChnGRegBits(uint8_t chn_index);      //获取指定通道的G寄存器指针

	static void InitPoweroffHandler();			//初始关联掉电信号和掉电处理函数


//	void SetAxisSoftLimit(uint8_t axis);   //设置指定轴的软限位开关
//	void SetAxisSoftLimitValue(uint8_t axis, uint8_t index);   //设置指定轴的软限位值

	uint32_t GetDaPrecision(){return this->m_n_da_prec;}  //返回DA精度值

	bool CheckAxisHardLimit(uint8_t phy_axis, int8_t dir);   //检查物理轴限位告警情况

	void SetPmcSignal(uint8_t chn_index, int signal, bool flag);	//设置通道的PMC信号

	void SetPoweoffFlag(bool flag){m_b_power_off = flag;}  //设置掉电标志

	void RefreshFile(char *file);   //刷新nc文件file
	void RemapFile(char *file);		//重新加载文件file

	void ProcessHmiIOCmd(HMICmdFrame &cmd);   	//处理HMI的IO请求

	bool IsEmergency(){return m_b_emergency;}    //是否处于急停状态

	void ResetOPSignal(uint8_t chn_index);   //复位OP信号
	void SetALSignal(uint8_t chn_index, bool value);  //设置告警信号


    void SetMiWorkMode(uint8_t value);   //设置MI模块工作模式
    void SetMiHandwheelTrace(bool flag, uint8_t chn);   //设置MI模块手轮跟踪模式
    void SetMiCurChannel();   //设置MI当前通道号

    bool CheckAxisRefBaseSignal(uint8_t phy_axis, int8_t dir);   //检查轴原点粗基准信号
    bool CheckAxisRefSignal(uint8_t phy_axis);      //检查轴原点精基准信号（IO信号）
    
    uint8_t GetAxisChannel(uint8_t phy_axis, uint8_t &chn_axis);     //获取物理轴所属通道及通道轴编号

    void SetRetRefMask(uint8_t phy_axis);    //设置回参考点轴mask
    bool IsRefReturnning(uint8_t phy_axis);    //指定轴是否正在回零中

    void SetRetRefFlag(uint8_t phy_axis, bool flag);   //设置轴回参考点完成标志
	void SetInRetRefFlag(uint8_t phy_axis, bool flag);  //设置轴回零中信号
	
    void SendPmcAxisCmd(PmcCmdFrame &cmd);     //发送PMC轴运动指令
    uint8_t GetPmcAxis(uint8_t pmc_chn);        //获取指定PMC通道的PMC轴序号
    uint32_t GetPmcAxisRapidSpeed(uint8_t axis);   //获取pmc轴的设定定位速度
    void SetCurPmcAxis(uint8_t axis);     //设置当前PMC轴

    void SetPmcOutSignal(uint8_t index, uint8_t value);    //设置PMC中F寄存器的OUT输出点位

    void ProcessHmiFindRefCmd(HMICmdFrame &cmd); 	//处理HMI回参考点指令
    void ProcessHmiSetRefCmd(HMICmdFrame &cmd);		//处理HMI设置参考点指令
    void ProcessPmcRefRet(uint8_t phy_axis);         //处理PMC指令的物理轴回零

    void ProcessPmcAxisFindRef(uint8_t phy_axis);   //处理PMC轴确认参考点指令

    void ClearPmcAxisMoveData();   //清空MI中的PMC轴移动数据

    int CheckLicense(bool force = false);     //授权校验

    void SetAxisNameEx(bool flag);   //设置轴名称扩展下标使能

    void SendMiPcData(uint8_t axis);    //向MI发送螺补数据表
    void SendMiPcParam(uint8_t axis);   //向MI发送指定轴螺补参数设置
    void SendMiPcParam2(uint8_t axis);  //向MI发送指定轴螺补参数设置2
    void SendMiRefMachPos(uint8_t axis_index);   //向MI发送指定轴的参考点位置机械坐标
    void SendMiBacklash(uint8_t axis);  //向MI发送指定轴的反向间隙参数

    void SendMiIoRemapInfo(IoRemapInfo &info);   //向MI发送设置IO重定向的配置

    void SendMiAxisFilterParam(uint8_t phy_axis);   //向MI发送指定轴的滤波器相关参数


#ifdef USES_TIANJIN_PROJ
    double TransMachToWorkCoord(double &pos, uint8_t pmc_axis_index);   //PMC轴机械坐标系到工件坐标系的转换
    double TransWorkToMachCoord(double &pos, uint8_t pmc_axis_index);   //PMC轴工件坐标系到机械坐标系的转换
#endif

#ifdef USES_LASER_MACHINE
    void StartLaserCalib(bool start);				//开始激光调高器标定
    void EnableLaserHeightTrace();       //激活激光头高度跟随功能
#endif

    void DoIdle();     //空闲处理函数

    void SyncKeepVar();   //同步保存文件

    void SetParamImportMask(int param_type);    //设置参数导入标志
    bool IsParamImported(int param_type){return (this->m_mask_import_param & (0x01<<param_type)) == 0?false:true;}       //配置是否导入过

    bool NotifyHmiAxisRefBaseDiffChanged(uint8_t axis, double diff);     //通知HMI轴参考点粗精基准位置偏差发生变更

    bool NotifyHmiAxisRefChanged(uint8_t phy_axis);     //通知HMI轴参考点对应的编码器值变更

    bool NotifyHmiPitchCompDataChanged();        //通知HMI螺补数据更改

    bool ProcessPcDataImport();      //处理螺补导入数据

#ifdef USES_WOOD_MACHINE
    void SaveToolInfo();    //保存刀具信息数据
#endif

#ifdef USES_GRIND_MACHINE
	void RefreshMechArmParam();    //刷新机械手参数
	void RefreshMechArmSpeedDelay();  //刷新机械手速度和延迟时间
	void InitMechArmState();       //初始化机械手状态
	void StopMechArmRun(bool flag);      //停止机械手动作流程
	void PauseMechArmRun(bool flag);     //暂停机械手动作
	void ProcessNewTray(uint8_t chn);      //处理新料盘动作
	void SendHMIMechArmException(uint8_t err_code);   //向HMI发送机械臂异常代码
	void ProcessHmiMechArmExcepRsp(HMICmdFrame &cmd);   //处理HMI返回的机械手异常数据包

	StateMechArm *GetMechArmState(){return &this->m_mech_arm_state;}   //获取机械手状态
	ParamMechArm *GetMechArmParam(){return &this->m_mech_arm_param;}   //获取机械手参数
#endif

    void PrintDebugInfo();		//输出调试数据

    PmcAxisCtrl *GetPmcAxisCtrl(){return m_pmc_axis_ctrl;}
    SyncAxisCtrl *GetSyncAxisCtrl(){return  m_sync_axis_ctrl;}

private:	//私有成员函数
	ChannelEngine();   //构造函数

	static void *RefreshMiStatusThread(void *args); //MI状态更新线程函数
	bool RefreshMiStatusFun();  //更新MI的状态

	void SendIntepolateCycle();			//向MC发送插补周期参数
	void InitMcDataBuffer();				//初始化MC的数据缓冲区
	void InitMcCoord();					//初始化设置MC工件坐标系
	void SendGetMcVersionCmd();			//获取MC模块的版本信息
	void InitMcParam();					//设置MC的相关参数

	void InitChnModeGroup();                //初始化方式组数据

	void InitPcAllocList();           //初始化螺补数据分布表

#ifdef USES_GRIND_MACHINE
	void SendMcGrindParam();				//发送磨削参数
	void RunMechArm();                //执行机械数上下料动作

	void ManualMoveMechArm();      //机械手手动定位
#endif

	void ProcessMcVersionCmd(McCmdFrame &cmd);	//处理MC模块返回的版本信息
	void ProcessMiVersionCmd(MiCmdFrame &cmd);	//处理MI模块返回的版本信息


	static void PoweroffHandler(int signo, siginfo_t *info, void *context); 	//掉电处理函数

	void SaveDataPoweroff();			//掉电时保存数据

	void ProcessMiShakehand(MiCmdFrame &cmd); 	//处理MI握手指令
	void ProcessMiPmcLadderReq(MiCmdFrame &cmd);   //处理PMC梯形图请求命令
	void ProcessMiGetESBCmd(MiCmdFrame &cmd);    //处理MI获取伺服描述文件命令
	void ProcessMiAlarm(MiCmdFrame &cmd);			//处理MI告警信息
	int32_t LoadPmcLadderData(uint16_t index, uint16_t &flag);		//加载PMC梯形图至共享区
	int32_t LoadEsbData(uint16_t index, uint16_t &flag);    //加载ESB文件数据，伺服描述文件
	void ProcessMiSetRefCurRsp(MiCmdFrame &cmd);		//处理MI返回的编码器值
	void ProcessMiBusError(MiCmdFrame &cmd);			//处理MI返回的总线错误

	void ProcessMiClearPosRsp(MiCmdFrame &cmd);		//处理MI返回的清整数圈位置指令回复

	void ProcessMiSyncAxis(MiCmdFrame &cmd);		//处理MI返回的同步轴同步结果
	void ProcessMiHWTraceStateChanged(MiCmdFrame &cmd);   //处理MI发送的手轮跟踪状态切换指令

	void ProcessHmiUpdateReq(HMICmdFrame &cmd);		//处理HMI升级请求
	void ProcessHmiGetParam(HMICmdFrame &cmd);   //处理HMI获取参数指令
	void ProcessHmiSetParam(HMICmdFrame &cmd);	//处理HMI设置参数指令

	bool UpdateHmiPitchCompData(HMICmdFrame &cmd);   //处理HMI更新螺补数据

	void ProcessHmiGetPmcReg(HMICmdFrame &cmd);	//处理HMI获取PMC寄存器值得指令
	void ProcessHmiSetPmcReg(HMICmdFrame &cmd);	//处理HMI设置PMC寄存器值得指令
	void ProcessHmiGetPmcUuid(HMICmdFrame &cmd); //处理HMI请求PMC的UUID指令
	void ProcessHmiAxisMoveCmd(HMICmdFrame &cmd);        //处理HMI轴移动指令

	void ProcessHmiGetLicInfoCmd(HMICmdFrame &cmd);     //处理HMI获取授权信息指令
	void ProcessHmiRegLicCmd(HMICmdFrame &cmd);          //处理HMI注册授权指令

	void ProcessHmiGetIoRemapInfoCmd(HMICmdFrame &cmd);  //处理HMI获取IO重映射信息命令
	void ProcessHmiSetIoRemapInfoCmd(HMICmdFrame &cmd);  //处理HMI设置IO重映射信息命令

	void ProcessHmiSetProcParamCmd(HMICmdFrame &cmd);    //处理HMI设置工艺相关参数的命令
	void ProcessHmiGetProcParamCmd(HMICmdFrame &cmd);    //处理HMI获取工艺相关参数的命令
	void ProcessHmiSetCurProcIndex(HMICmdFrame &cmd);    //处理HMI设置通道当前工艺组号的命令
	void ProcessHmiGetCurProcIndex(HMICmdFrame &cmd);    //处理HMI获取工艺参数组号的命令

	void ProcessHmiClearMsgCmd(HMICmdFrame &cmd);     //处理HMI清除消息命令
    void ProcessHmiGetErrorCmd(HMICmdFrame &cmd);       //处理HMI获取SC错误命令
	
	void ProcessHmiNotifyGraphCmd(HMICmdFrame &cmd);    //处理HMI通知图形模式命令
    void ProcessHmiHandWheelCmd(HMICmdFrame &cmd);

	void SendHmiUpdateStatus(uint8_t total_step, uint8_t cur_step);  //给HMI发送升级状态



	void ProcessPmcSignal();		//处理PMC的G变量
	void ProcessPmcAxisCtrl();		//处理PMC轴控制信号
	void ProcessPmcDataWnd();       //处理PMC数据窗口



	void ProcessPmcAlarm();    //处理PMC告警

	void ProcessAxisHardLimit(uint8_t dir);   //处理轴硬限位告警

	void GetParamValueFromCmd(ParamUpdate *data, char *src);  //从命令保重获取参数数据

    static void *UpdateThread(void *args);  //模块升级执行线程函数
    int UpdateProcess();		//模块升级执行函数
    int UpdateMC();				//升级MC模块
    int UpdateMI();				//升级MI模块
    int UpdateMI_2();				//升级MI模块，与boot分开加载
    int UpdateSC();				//升级SC模块
    int UpdatePL();				//升级PL模块
    int UpdatePMC();				//升级PMC模块
    int UpdateSpartan();			//升级Spartan模块
    int UpdateModbus();            //升级Modbus模块
	int UpdateDisk();			//一键升级

    void SendMcResetCmd();		//发送MC复位指令
    void ClearMcAlarm();			//发送MC清除告警指令
    bool SendMcUpdateStartCmd();		//发送MC升级开始指令
    bool SendMcUpdateFileSize(uint32_t size);		//发送MC升级文件大小
    bool SendMcUpdateFileCrc(uint16_t frame_count, uint16_t crc);		//发送MC升级文件CRC
    bool QueryMcUpdateStatus();	//查询MC升级状态

    bool SendMessageToHmi(uint16_t msg_type, uint32_t msg_id, uint32_t msg_param = 0);		//向HMI发送提示信息
    uint16_t GetBusAxisCount();        //获取实际总线轴数量
    void InitMiParam();     			//初始化MI参数
    void InitPmcReg();				//初始化PMC的非易失性寄存器

    void SendPmcRegValue(PmcRegSection sec, uint16_t index, uint16_t data);  //向MI设置PMC寄存器
    void SendMiReset();		//发送MI复位指令
 //   void SendMiGetVerCmd();		//发送获取MI版本指令

//    template<typename T>
//    void SendMiParam(uint8_t axis, uint32_t para_no, T data);  //发送MI参数

    void CheckBattery();		//电压相关告警检查

    void InitPhyAxisChn();		//初始化物理轴与通道的映射
    void SendMiPhyAxisEncoder();     //向MI发送物理轴的反馈
    void SetAxisRetRefFlag();    //向MI发送各轴回参考点结束标志

    void SaveCurPhyAxisEncoder();  //掉电保存当前所有物理轴的编码器反馈
    void SaveKeepMacroVar();		//掉电保存非易失性宏变量

    void CheckAxisSrvOn();     //检查轴上伺服信号

    void SetSlaveInfo();		//设置SD-LINK从站数据

    void SetMcAutoBufMax(uint16_t count);     //设置MC单通道自动数据缓冲数量

    void SendMiAxisZCaptureCmd(uint8_t phy_axis, bool active);  // 向MI发送激活捕获Z信号命令
	void ReturnRefPoint();      //回参考点执行函数
	void PulseAxisFindRefWithZeroSignal(uint8_t phy_axis);      // 脉冲输出有基准轴回参考点，根据粗基准 精基准回参考点
	void PulseAxisFindRefNoZeroSignal(uint8_t phy_axis);         // 脉冲输出无基准轴回参考点，仅根据精基准回参考点
	void EcatAxisFindRefWithZeroSignal(uint8_t phy_axis);     // ethcat轴有基准轴回参考点，根据粗基准 精基准回参考点
	void EcatAxisFindRefNoZeroSignal(uint8_t phy_axis);        // ethcat轴无基准轴回参考点，仅根据精基准回参考点
	void EcatAxisFindRefWithZeroSignal2(uint8_t phy_axis);     //ethcat轴有基准回参考点，粗基准与精基准均为IO信号，直线电机
	void AxisFindRefWithZeroSignal(uint8_t phy_axis);            // 仅根据原点信号设置参考点 包括步进电机根据原点信号设置参考点
	void AxisFindRefNoZeroSignal(uint8_t phy_axis);              // 当前位置设置为参考点   虚拟轴回参考点  步进电机设置参考点
    void EcatIncAxisFindRefWithZeroSignal(uint8_t phy_axis);     // 总线增量式轴有基准回参考点，根据Z信号作为精基准
    void EcatIncAxisFindRefNoZeroSignal(uint8_t phy_axis);       // 总线增量式无基准回参考点，仅根据Z信号作为精基准回零


	void PmcAxisRunOver(MiCmdFrame &cmd);    //PMC轴运行到位

	void ProcessGetCurAxisEncodeRsp(MiCmdFrame &cmd);    //处理MI返回的当前编码器单圈绝对值，回参考点时使用
	void ProcessSetAxisRefRsp(MiCmdFrame &cmd);      //处理MI返回的设置轴参考点回复指令，回参考点时使用
	void ProcessGetAxisZeroEncoderRsp(MiCmdFrame &cmd);    //处理MI返回的获取机械零点编码器值指令，回参考点时使用
	void ProcessSkipCmdRsp(MiCmdFrame &cmd);    //处理MI返回的跳转命令响应
	void ProcessRefreshAxisZeroEncoder(MiCmdFrame &cmd);   //处理MI刷新轴零点编码器值命令
	void ProcessMiEnSyncAxisRsp(MiCmdFrame &cmd);      //处理MI使能同步轴指令的响应
    void ProcessMiSpdLocateRsp(MiCmdFrame &cmd);    //处理MI主轴定位的响应
    void ProcessMiOperateCmdRsp(MiCmdFrame &cmd);  //处理MI操作指令
    void ProcessMiAxisCtrlModeRsp(MiCmdFrame &cmd);     //处理MI轴模式切换响应
    void ProcessMiSpindleSpeedRsp(MiCmdFrame &cmd);     //处理MI DA输出的响应
    void ProcessMiActiveAxisZCaptureRsp(MiCmdFrame &cmd);   //处理MI捕获Z信号指令响应
	
	void ProcessSetAxisCurMachPosRsp(MiCmdFrame &cmd);   //处理MI对设置轴当前位置机械坐标命令的响应

	void ProcessHmiGetPcDataCmd(HMICmdFrame &cmd);    //处理HMI获取螺补数据指令

	void CheckTmpDir();    //测试tmp目录是否存在，不存在则创建



private:  //私有成员变量
	static ChannelEngine *m_p_instance;    //唯一实例指针
	ChannelModeGroup *m_p_channel_mode_group;  //通道方式组
	ChannelControl *m_p_channel_control;    //通道控制
	HMICommunication *m_p_hmi_comm;        //HMI通讯接口
	MICommunication *m_p_mi_comm;		   //MI通讯接口
	MCCommunication *m_p_mc_comm;		   //MC通讯接口
	MCArmCommunication *m_p_mc_arm_comm;   //MC-ARM通讯接口
	SCSystemConfig *m_p_general_config;    //系统配置
	SCChannelConfig *m_p_channel_config;   //通道配置
	SCAxisConfig *m_p_axis_config;			//物理轴配置
	//螺补数据表
	AxisPitchCompTable *m_p_pc_table;   //分轴存储，动态分配，先正向螺补，后负向螺补
	IoRemapList *m_p_io_remap;     //IO重定向数据

	//工艺参数
	ChnProcParamGroup *m_p_chn_proc_param;    //工艺相关通道参数
	AxisProcParamGroup *m_p_axis_proc_param;   //工艺相关轴参数

	LitronucLicInfo m_lic_info;       //授权数据
	long m_ln_local_time;            //本地计时  -1：表示文件丢失   -2：表示文件损坏    -3：表示非本机文件   >0:表示正常计时
	char m_device_sn[SN_COUNT+1];    //设备序列号

	PcDataAllocList m_list_pc_alloc;     //螺补数据分布表

#ifdef USES_FIVE_AXIS_FUNC
	FiveAxisConfig *m_p_chn_5axis_config;    //五轴配置
#endif

#ifdef USES_GRIND_MACHINE
	GrindConfig *m_p_grind_config;       //磨床参数
	ParamMechArm m_mech_arm_param;       //上下料机械手参数
	StateMechArm m_mech_arm_state;       //机械手状态
	MechArmManualState m_mech_arm_manual_state;    //机械手手动状态
	uint8_t m_mech_arm_manual_step;    //机械手手动定位步骤
	double m_mech_arm_manual_target_x;  //手动X轴目标位置
	double m_mech_arm_manual_target_y;  //手动Y轴目标位置
#endif

	ErrorType m_error_code;  		//错误码

	uint8_t m_n_cur_channle_index;   //当前通道索引号
	uint8_t m_n_cur_chn_group_index;   //当前方式组号

//	uint16_t m_n_mc_auto_buf_max;    //MC中单通道自动模式数据缓冲最大数量

	double *m_df_phy_axis_pos_feedback;		//物理轴当前反馈机械坐标
	double *m_df_phy_axis_pos_intp;        //物理轴当前插补机械坐标
	double *m_df_phy_axis_pos_intp_after;   //物理轴当前插补后加减速输出的机械坐标,还加上了螺补，反向间隙等补偿值
	double *m_df_pmc_axis_remain;    //PMC轴余移动量

//#ifdef USES_SPEED_TORQUE_CTRL	
	double *m_df_phy_axis_speed_feedback;        //物理轴当前反馈速度
	double *m_df_phy_axis_torque_feedback;        //物理轴当前反馈力矩
//#endif	

	int64_t m_n_phy_axis_svo_on;		//物理轴使能标志

	uint8_t m_n_pmc_axis_count;     //PMC轴数量

	pthread_t m_thread_refresh_mi_status;   //刷新MI状态数据线程
	pthread_t m_thread_update;		//模块升级线程
	uint8_t m_n_update_state;     //升级状态   0--非升级状态    1--升级SC   2--升级MC   3--升级PMC梯形图   4--升级MI    5--升级PL     6--升级FPGA

	uint16_t m_n_mc_update_status;   //MC模块升级状态
	bool m_b_get_mc_update_status;  //接收到MC升级状态

	bool m_b_recv_mi_heartbeat;     //标志首次收到MI的心跳包，此时可以开始给ＭＩ发送初始化参数
	bool m_b_init_mi_over;			//标志给MI发送初始化参数完成

	PmcRegister *m_p_pmc_reg;			//PMC寄存器
	GRegister m_g_reg_last;             //上一周期的G寄存器
    FRegister m_f_reg_last;             //上一周期的F寄存器

	bool m_b_emergency;				//急停状态标志  true--急停状态    false--非急停状态
	uint64_t m_hard_limit_postive;	//硬限位触发标志，正向   64bit代表64轴
	uint64_t m_hard_limit_negative; //硬限位触发标志，负向

	uint32_t m_n_da_prec;			//DA精度，转换成了拆分精度，即：如果DA为16位，则为32768

	bool m_b_power_off;				//掉电标志

	uint8_t m_map_phy_axis_chn[kMaxAxisNum];      //物理轴所属通道

	bool m_b_reset_rst_signal;    //复位RST信号标志
	struct timeval m_time_rst_over;   //复位操作结束时间

	uint32_t m_n_idle_count;      //空闲周期计数

	//同步轴变量
    SyncAxisCtrl *m_sync_axis_ctrl;

	BdioDevList m_list_bdio_dev;    //SD-LINK设备队列

	//回参考点相关参数
	bool m_b_ret_ref;                 //回参考点状态
	bool m_b_ret_ref_auto;            //自动回参考点标志
	uint64_t m_n_mask_ret_ref_over;   //已回参考点标志
	uint64_t m_n_mask_ret_ref;         			//回参考点的轴MASK
	int m_n_ret_ref_step[kMaxAxisNum];    	//回参考点当前步骤
	uint8_t m_n_ret_ref_auto_cur; 				//自动回参考点时的当前顺序号
	struct timeval m_time_ret_ref[kMaxAxisNum];       //回参考延时计时器
	double m_df_ret_ref_tar_pos[kMaxAxisNum];  //回参考点中间移动目标位置
//	uint8_t m_n_get_cur_encoder_count;    //获取MI当前编码器计数，保护措施，多次验证
//	uint64_t m_n_ret_ref_encoder;   //粗基准编码器值


	//PMC轴运动相关
	uint64_t m_n_run_axis_mask;  //当前运行的轴的mask
	uint64_t m_n_runover_axis_mask;   //当前运行完成的轴的mask
	PmcAxisCtrl m_pmc_axis_ctrl[kMaxPmcAxisCtrlGroup];    //PMC轴控制
	uint8_t m_n_cur_pmc_axis;       //当前PMC轴  0xFF表示当前没有选择PMC轴

	uint16_t m_mask_import_param;    //导入参数标志
	
	bool m_mc_run_on_arm[kMaxChnCount];   //MC通道是否运行在ARM上            0 -- dsp    1--mi
	bool m_mc_run_dsp_flag;    // 有通道的MC运行在DSP的标识   
	bool m_mc_run_mi_flag;     // 有通道的MC运行在MI的标识  

#ifdef USES_EMERGENCY_DEC_STOP
	bool m_b_delay_servo_off;      //延迟断伺服标志，等待减速停止后再断伺服，木工机速度太快不能直接断伺服
	uint8_t m_mask_delay_svo_off;   //待延迟下伺服的通道mask
	uint8_t m_mask_delay_svo_off_over;   //已经停止到位的通道的mask
#endif

    const int HANDWHEEL_BYTES = 3;
    const static map<int, SDLINK_SPEC> m_SDLINK_MAP;

};

#endif /* INC_CHANNEL_CHANNEL_ENGINE_H_ */
