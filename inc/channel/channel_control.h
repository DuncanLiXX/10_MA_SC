/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file channel_control.h
 *@author gonghao
 *@date 2020/03/19
 *@brief 本头文件为通道控制类的声明
 *通道控制类的主要作用是管理通道状态，控制通道处理流程
 *@version
 */

#ifndef INC_CHANNEL_CHANNEL_CONTROL_H_
#define INC_CHANNEL_CHANNEL_CONTROL_H_

#include "compiler.h"
#include "channel_data.h"
#include "variable.h"
#include <pthread.h>

//前置声明
class ChannelEngine;   //通道引擎类
class HMICommunication; //HMI通讯类
class ParmManager;      //配置管理类
class MICommunication;  //MI通讯类
class MCCommunication;  //MC通讯类
class MCArmCommunication;  //MC-ARM通讯类
class PmcRegister;		//PMC寄存器类
class SpindleControl;   //主轴控制类
struct SCSystemConfig; //SC通道配置
struct SCChannelConfig;  //SC通道配置
struct SCAxisConfig;     //SC轴配置
struct FRegBits;		//F寄存器点位定义
struct GRegBits;		//G寄存器点位定义

/**
 * @brief 通道控制类
 */
class ChannelControl {
public:
	ChannelControl();	//构造函数
	virtual ~ChannelControl(); //析构函数

	// @test zk
    //void test();
	// @test zk

	bool Initialize(uint8_t chn_index, ChannelEngine *engine, HMICommunication *hmi_comm,
			MICommunication *mi_comm, MCCommunication *mc_comm, ParmManager *parm, PmcRegister *reg);  //初始化函数
	
	void SetMcArmComm(MCArmCommunication *comm){this->m_p_mc_arm_comm = comm;}   //设置MC-ARM通讯接口

    SpindleControl *GetSpdCtrl(){return m_p_spindle;}
    const GRegBits *GetGReg(){return m_p_g_reg;}
    PmcAxisCtrl *GetPmcAxisCtrl();
    void RefreshPmcAxis();


//	bool OpenFile(const char *file_name);   //打开待编译文件

	ErrorType GetErrorCode(){return m_error_code;}   //返回当前错误码

	const ChannelStatusCollect &GetChnStatus(){return this->m_channel_status;}  //获取当前通道状态结构体
    const ChannelRealtimeStatus &GetRealtimeStatus(){return m_channel_rt_status;} //获取实时状态结构体

	uint8_t GetChnAxisCount(){return this->m_p_channel_config->chn_axis_count;}   //获取通道轴数量
	uint8_t GetChnAxisName(uint8_t idx){return this->m_p_channel_config->chn_axis_name[idx];}   //获取通道轴名称

	void Pause(); 			//停止G代码

	void StartRunGCode();  //开始G代码运行
	void StopRunGCode(bool reset = true);	//停止G代码运行
	void StopCompilerRun();		//停止编译


	void SendMonitorData(bool bAxis, bool btime);   //发送监控数据

	void SendSimulateDataToHmi(MonitorDataType type, SimulateData &data);   //发送仿真数据给HMI

	void ProcessHmiCmd(HMICmdFrame &cmd);  //处理HMI指令

	void GetChnStatus(HmiChannelStatus &status);  //获取通道状态

	uint8_t GetChnWorkMode(){return m_channel_status.chn_work_mode;}   //获取通道当前工作模式

	void SetWorkMode(uint8_t work_mode);  //切换工作模式

	bool IsRapidManualMove(){return m_channel_status.func_state_flags.CheckMask(FS_MANUAL_RAPID);}       //是否快速手动移动

	uint8_t GetChnIndex(){return  m_n_channel_index;}   //返回通道索引号

    void CompileOver();		//编译结束处理

    void ResetMcLineNo();    //复位MC模块的当前加工行号，置零

    bool IsBlockRunOver();   //是否块运行到位

    bool IsMachinRunning();    //是否在运行中，包括自动运行，加工仿真，刀路仿真

	bool IsStepMode(); 		//是否单段模式,只有在自动模式下，单段才有效

    void SetCurLineNo(uint32_t line_no);   	//设置当前行号
    void SetCurLineNoFromMc();   //设置当前行号从MC模块获取


    CompilerState GetCompileState(){return m_n_run_thread_state;}    //获取G代码编译运行状态，编译/暂停/停止

//	bool StartCompile(const char *file);		//开始编译
//	void ContinueCompile(){m_n_thread_state = RUN;}	//继续编译

	bool SetSimulateMode(const SimulateMode mode);   //设置仿真模式
	SimulateMode GetSimulateMode(){return m_simulate_mode;}  	//返回仿真模式
	SimulateMode *GetSimulateModePtr(){return &m_simulate_mode;}  //返回仿真模式变量指针


	bool OutputData(RecordMsg *msg, bool flag_block = false);     //向MC发送编译G代码数据
	bool OutputSimulateData(RecordMsg *msg);   //向HMI发送仿真数据

	uint8_t GetPhyAxis(uint8_t chn_axis);  //获取通道轴对应的物理轴索引号，0开始
	uint8_t GetSpdChnAxis(uint8_t spd_idx);  //获取指定序号的主轴对应的通道轴号，0开始

	double GetToolCompRadius(int d_index);    //获取对应D值得刀具半径补偿值   半径+磨损

	void SetFuncState(int state, uint8_t mode = 10);		//设置功能状态，例如：单段，选停等等
	void SetAutoRatio(uint8_t ratio);    //设置自动倍率
	void SetManualRatio(uint8_t ratio);	//设置手动倍率
	void SetRapidRatio(uint8_t ratio);	//设置快速进给倍率
    void SetSpindleRatio(uint8_t ratio);	//设置主轴倍率
    void SetManualStep(uint8_t step);	//设置手动步长
	void SetManualRapidMove(uint8_t mode = 10);			//设置手动快速移动状态
	void SetCurAxis(uint8_t axis);		//设置当前轴

	uint8_t GetAutoRatio(){return this->m_channel_status.auto_ratio;}   //返回当前自动倍率
	uint8_t GetManualRatio(){return this->m_channel_status.manual_ratio;}     //返回当前手动倍率
	uint8_t GetRapidRatio(){return this->m_channel_status.rapid_ratio;}       //返回当前快速进给倍率
	uint8_t GetSpindleRatio(){return this->m_channel_status.spindle_ratio;}   //返回当前主轴倍率
	uint8_t GetManualStep(){return this->m_channel_status.manual_step;}       //返回当前手动步长

	void SetMcAutoBufMax(uint16_t count){this->m_n_mc_auto_buf_max = count;}   //设置MC自动模式运动数据缓冲数量
	uint16_t GetMcAutoBufMax(){return this->m_n_mc_auto_buf_max;}   //获取MC单通道自动数据缓冲数量

	void ManualMove(int8_t dir);		//手动移动
    //void ManualMove2(uint8_t axis, int8_t dir, double vel, double inc_dis);  //手动移动，向dir方向移动dis距离
	void ManualMoveStop();			//停止当前轴手动移动
	void ManualMoveStop(uint16_t axis_mask);   //停止轴的手动移动

	void ManualMove(uint8_t axis, double pos, double vel, bool workcoord_flag = false);   //手动指令某个轴以指定速度运动到指定位置

	void ManualMovePmc(int8_t dir);		//手动移动PMC轴
	void ManualMovePmc2(uint8_t axis, int8_t dir, double vel, double inc_dis);   //指定PMC轴手动移动
	void ManualMovePmc(uint8_t axis, double pos, double vel);   //手动指令某个PMC轴以指定速度运动到指定位置
	void ManualMovePmcStop();			//停止当前PMC轴手动移动
	void ManualMovePmcStop(uint8_t phy_axis);   //停止PMC轴的手动移动
	void PausePmcAxis(uint8_t phy_axis, bool flag);          //暂停PMC轴移动
	void StopPmcAxis(uint8_t phy_axis);       //停止PMC轴的移动

    // 检测是否软限位超限
    // dir: 方向
    // phy_axis: 物理轴号 从0开始
    // pos: 目标位置 mm
    // 返回值： 0：没有超限   1：超限
    bool CheckSoftLimit(ManualMoveDir dir, uint8_t phy_axis, double pos);
    // 获取限位位置
    bool GetSoftLimt(ManualMoveDir dir, uint8_t phy_axis, double &limit);

	void HandwheelMove(int32_t hw_count);   //手轮移动指令

	void ManualToolMeasure(int h_code, int times);      //手动对刀处理

	void InitMcIntpMdaBuf();		//通知MC进行通道MDA插补缓冲初始化
	void InitMcIntpAutoBuf();	//通知MC进行通道自动数据缓冲初始化

	void SetMcStepMode(bool flag);   //设置MC单段模式

	Variable *GetMacroVar(){return &m_macro_variable;}   //返回宏变量对象指针
	int GetMacroVar(const int index, double &value);   //返回指定宏变量的值
	bool GetSysVarValue(const int index, double &value);      //获取系统变量值
	bool SetSysVarValue(const int index, const double &value);      //设置系统变量

	bool IsOutputMsgRunover();   //返回是否还有待运行的指令


	bool EmergencyStop();		//急停处理

	void Reset();               //复位通道状态

	void ProcessHmiSetRefCmd(HMICmdFrame &cmd);			//处理设置参考点命令

	void SetMcCoord(bool flag);  	//设置MC的坐标系原点
	void SetChnAxisName();		//设置通道轴名称

	void SetChnAxisOn(uint8_t chn_axis);			//设置轴使能，以及插补模式（1--NC轴插补   2--PMC轴插补）
	void SetChnAxisBaseParam(uint8_t chn_axis);		//设置指定轴的螺距及最大速度等基本信息
	void SetChnAxisSpeedParam(uint8_t chn_axis);		//设置指定轴的速度相关信息
	void SetChnAxisAccParam(uint8_t chn_axis);			//设置指定轴加速度相关信息
	void SetChnAxisSoftLimit(uint8_t chn_axis);   //设置指定轴的软限位开关
	void CloseChnAxisSoftLimit(uint8_t chn_axis);  //强制关闭指定轴的软限位开关
	void SetChnAxisSoftLimitValue(uint8_t chn_axis, uint8_t index);   //设置指定轴的软限位值
	void SetChnAllAxisParam(void);
	void SetChnAxisPosThreshold(uint8_t chn_axis);   //设置指定轴的位置指令
	
	void SetMachineState(uint8_t mach_state);     //设置加工状态

	void SendMcSysResetCmd();	//给MC发送系统复位指令

	void SetMcChnPlanMode();			//设置加工速度规划方式
	void SetMcChnPlanParam();			//设置通道加工速度规划参数
	void SetMcChnPlanFun();			//设置通道加工速度功能开关
	void SetMcChnCornerStopParam();		//设置拐角准停参数

#ifdef USES_WOOD_MACHINE
	void SetMcFlipCompParam();     //设置MC的挑角补偿值
	void SetMcDebugParam(uint16_t index);   //设置MC的调试参数

#endif


	double GetAxisCurIntpTarPos(uint8_t axis_index, bool bMachCoord);  //获取单轴的当前插补目标位置
	double GetAxisCurWorkPos(uint8_t axis_index);   //获取单轴的当前工件坐标系位置
	double GetAxisCurMachPos(uint8_t axis_index);	//获取单轴的当前机械坐标系位置
	double GetAxisCurFedBckAxisSpeed(uint8_t axis_index);	//获取单轴的当前运行速度
	double GetAxisCurFedBckAxisTorque(uint8_t axis_index);	//获取单轴的当前运行力矩
	int GetCurManualStep();	//获取当前手动单步步长设置

	bool SendOpenFileCmdToHmi(char *filename); 	//向HMI发送切换NC文件显示命令

	void SpindleOut(int dir, int speed=0);			//主轴输出

	void UpdateCoord(uint8_t index, HmiCoordConfig &cfg);  //更新工件坐标系
	void UpdateExCoord(uint8_t index, HmiCoordConfig &cfg);  //更新扩展工件坐标系
    bool UpdateAllCoord(double val);//更新所有工件坐标系为设定值
    bool UpdateAllExCoord(double val, int count);//更新所有的扩展工件坐标系设定值

	void UpdateToolOffset(uint8_t index, HmiToolOffsetConfig &cfg);   //更新刀具偏置
    bool UpdateAllToolOffset(const HmiToolOffsetConfig &cfg);

	bool IsCurNcFile(char *file_name);    //file_name是否为当前加工文件

	void RefreshFile(char *file);   //刷新nc文件file
	bool RemapFile(char *file);			//重新映射当前加载的NC文件

	void SetClearPosAxisMask(uint8_t axis_index);   //修改清整数圈位置的轴mask

	void SendMiChnAxisMap();   //发送通道轴-物理轴映射给MI
#ifdef USES_WUXI_BLOOD_CHECK
	bool ReturnRef(uint8_t axis_mask);		//指定轴回参考点
#endif

	void SaveKeepMacroVar();      //保存非易失性宏变量

    bool CheckAxisSrvOn(uint64_t &flag);   //检查轴上伺服状态

	void ResetOPSignal();   //复位OP信号
	void ResetRSTSignal();	//复位RST信号
	void SetALSignal(bool value);  //设置告警信号

	void TransMachCoordToWorkCoord(DPointChn &pos, uint16_t coord_idx, uint16_t h_code, uint32_t axis_mask);   //将坐标由机械坐标系转换为工件坐标系
	void TransMachCoordToWorkCoord(DPointChn &pos, uint16_t coord_idx, uint32_t axis_mask);   //将坐标由机械坐标系转换为工件坐标系
	void TransMachCoordToWorkCoord(DPoint &pos, uint16_t coord_idx, uint32_t axis_mask);   //将坐标由机械坐标系转换为工件坐标系
	void TransWorkCoordToMachCoord(DPointChn &pos, uint16_t coord_idx, uint32_t axis_mask);   //将坐标由工件坐标系转换为机械坐标系
	void TransWorkCoordToMachCoord(DPoint &pos, uint16_t coord_idx, uint32_t axis_mask);   //将坐标由工件坐标系转换为机械坐标系
    void TransMachCoordToWorkCoord(double &pos, uint16_t coord_idx, uint8_t axis);    //将单轴坐标由机械坐标系转换为工件坐标系
    void TransWorkCoordToMachCoord(double &pos, uint16_t coord_idx, uint8_t axis);    //将单轴坐标由工件坐标系转换为机械坐标系
	void TransWorkCoordToMachCoord(double &pos, uint16_t coord_idx, uint16_t h_code, uint8_t axis);  //将单轴坐标由工件坐标系转换为机械坐标系

	void ProcessHmiReturnRefCmd(bool flag);                   //回参考点执行函数
	void SetRefPointFlag(uint8_t chn_axis, bool flag);    //设置通道轴回参考点标志

	bool CheckFuncState(int func);      //通道是否处于func状态

	bool CheckGRegState(int sec, int bit);  //获取G地址寄存器状态
	bool CheckFRegState(int sec, int bit);  //获取F地址寄存器状态
	bool SetFRegValue(int sec, int bit, bool value);  //设置F地址寄存器的值
	uint32_t GetMacroVar1032();    //获取#1032的值
	uint16_t GetMacroVar1132();   //获取#1132的值
	uint32_t GetMacroVar1133();     //获取#1133的值

	void SetMacroVar1132(uint16_t value);  //设置#1132的值
	void SetMacroVar1133(uint32_t value);  //设置#1133的值

	bool IsSkipCaptured(){return m_b_pos_captured;}    //SKIP信号是否捕获

#ifdef USES_GRIND_MACHINE
	void SetMechArmParam(ParamMechArm *para){this->m_p_mech_arm_param = para;}      //设置机械手参数
	void SetMechArmState(StateMechArm *state){this->m_p_mech_arm_state = state;}    //设置机械手状态

	void RunM66_slave();      //运行从机上下料
#endif

#ifdef USES_WUXI_BLOOD_CHECK
	bool ReturnHome();		//回参考点   无锡项目定制
	void SetRetHomeFlag();    //设置回零标志
	bool IsReturnHome(){return m_b_returnning_home;}   	//返回回零标志
#endif

#ifdef USES_LASER_MACHINE
	void ProcessHmiCalibCmd(bool bstart);			//处理HMI调高器标定指令
	void ProcessMiInputSignal(MiCmdFrame &cmd);   //处理MI返回的激光调高器输入信号值
	void ProcessMiPosCapture(MiCmdFrame &cmd);		//处理Mi返回的轴位置捕获结果

	void EnableHeightRegulatorFollow(bool enable);   //调高器跟随使能
#endif

#ifdef USES_FIVE_AXIS_FUNC
	void UpdateFiveAxisRotParam();      //更新五轴旋转轴的相关参数
	bool IsFiveAxisMach(){return m_p_chn_5axis_config->five_axis_type==NO_FIVEAXIS?false:true;}   //是否开启五轴
	uint8_t GetFiveAxisRot1(){return this->m_n_5axis_rot1;}    //返回第一旋转轴的通道轴索引号
	uint8_t GetFiveAxisRot2(){return this->m_n_5axis_rot2;}    //返回第二旋转轴的通道轴索引号
	uint8_t GetFiveAxisRotMaskNoLimt(){return this->m_mask_5axis_rot_nolimit;}  //返回五轴无线旋转轴的mask

	void SetMcChnFiveAxisParam();			//初始化通道五轴相关参数
	void UpdateFiveAxisParam(FiveAxisParamType type);   //设置五轴参数
#endif


#ifdef USES_WOOD_MACHINE
	bool IsToolLifeChanged(){return this->m_b_save_tool_info;}   //是否刀具寿命改变
#endif

	void DoIdle(uint32_t count);     //空闲处理函数

	void SyncMacroVar();       //同步保存宏变量

	void ProcessSkipCmdRsp(MiCmdFrame &cmd);    //处理MI返回的跳转命令响应

	void SetAxisNameEx(bool flag);   //设置轴名称扩展下标使能

	bool PmcAxisRunOver(MiCmdFrame cmd);    //PMC轴运行到位

	void DPoint2DPointChn(const DPoint &src, DPointChn &tar);  //将DPoint对象变换为DPointChn对象
	void DPointChn2DPoint(const DPointChn &src, DPoint &tar);  //将DPointChn对象变换为DPoint对象

	void PrintDebugInfo();		//输出DSP-MC调试数据

	void PrintDebugInfo1();		//输出MI-MC调试数据

	void ProcessHmiManualToolMeasureCmd(HMICmdFrame &cmd);   //处理手动对刀指令

	void ProcessMiHWTraceStateChanged(MiCmdFrame &cmd);   //处理MI发送的手轮跟踪状态切换指令
	bool ChangeHwTraceState(HWTraceState state);    //切换手轮跟踪状态

    bool CallMacroProgram(uint16_t macro_index);      //调用宏程序

	uint8_t GetCurProcParamIndex(){return this->m_n_cur_proc_group;}   //返回当前工艺参数组号
	bool SetCurProcParamIndex(uint8_t index);  //设置当前工艺参数组号

	void GetMdaFilePath(char *path);   //返回通道MDA文件名称
	
	void SetHmiGraphMode(bool flag){this->m_b_hmi_graph = flag;}   //设置HMI图形显示模式

    void ClearMachineTimeTotal();

    void UpdateModeData(uint16_t mode_type, int value);//由SC直接更新D模态数据

#ifdef USES_ADDITIONAL_PROGRAM
	bool CallAdditionalProgram(AddProgType type);  //调用附加程序（前置/后置）
#endif

	void test(); // @test zk
	// @ add zk
	void StraightFeed(int chn, double x, double y, double z, int feed);
	void StraightTraverse(int chn, double x, double y, double z);
	void g73_func();
	// @ add zk

    uint8_t GetPhyAxisFromName(uint8_t axis_name);   //获取对应通道轴名称的物理轴索引号，0开始
    uint8_t GetChnAxisFromName(uint8_t axis_name);   //获取对应通道轴名称的通道轴索引号，0开始
    uint8_t GetChnAxisFromPhyAxis(uint8_t phy_axis); //获得对应物理轴的通道轴索引号，0开始

    void SyncMcPosition();  // 同步位置

private:
	void InitialChannelStatus();		//初始化通道状态
    static void *CompileThread(void *args);  //G代码运行线程函数
    int Run();       //G代码运行行函数


    static void *BreakContinueThread(void *args);  //断点继续执行线程函数
    int BreakContinueProcess();		//断点继续执行函数

    void ResetMode();       //复位通道模态

//  bool AddOutputData(const GCodeFrame &data);   //将运动控制数据帧压入缓冲
//	bool OutputLastBlockItem();   	//将运动数据缓冲中的数据都输出，置位块结束标志
//	bool OutputAllData();    			//将运动数据缓冲中的数据都输出,不置位块结束标志

    void SaveAutoScene(bool flag = true);		//保存自动模式加工暂停时的状态
    void ReloadAutoScene();		//恢复自动加工暂停时的状态
    bool StartBreakpointContinue();	//启动自动模式断点继续处理流程

	bool ExecuteMessage();       //命令消息执行模块
	bool ExecuteAuxMsg(RecordMsg *msg);    	//实际执行辅助指令消息
	bool ExecuteLineMsg(RecordMsg *msg, bool flag_block);   	//实际执行直线切削指令消息
	bool ExecuteRapidMsg(RecordMsg *msg, bool flag_block);  	//实际执行快速定位指令消息
	bool ExecuteArcMsg(RecordMsg *msg, bool flag_block);  		//实际执行圆弧切削指令消息
	bool ExecuteCoordMsg(RecordMsg *msg);  	//实际执行坐标系指定指令消息
	bool ExecuteToolMsg(RecordMsg *msg);  	//实际执行刀具指令消息
	bool ExecuteModeMsg(RecordMsg *msg);  	//实际执行一般模态指令消息，不带参数的模态指令
	bool ExecuteFeedMsg(RecordMsg *msg);  	//实际执行进给速度指定指令消息
	bool ExecuteSpeedMsg(RecordMsg *msg);  	//实际执行主轴转速指令消息
	bool ExecuteLoopMsg(RecordMsg *msg);  	//实际执行循环指令消息
	bool ExecuteCompensateMsg(RecordMsg *msg);  //实际执行刀补指令消息
	bool ExecuteErrorMsg(RecordMsg *msg);  	//实际执行错误指令消息
	bool ExecuteSubProgCallMsg(RecordMsg *msg);  //实际执行子程序调用消息
	bool ExecuteMacroCmdMsg(RecordMsg *msg);     //执行宏指令消息
	bool ExecuteSubProgReturnMsg(RecordMsg *msg);	//执行子程序退出消息
	bool ExecutePolarIntpMsg(RecordMsg *msg);	//执行极坐标插补及磨削类消息
	bool ExecuteClearCirclePosMsg(RecordMsg *msg);    //执行清整数圈消息
	bool ExecuteTimeWaitMsg(RecordMsg *msg);		//执行延时消息
	bool ExecuteRefReturnMsg(RecordMsg *msg);	//执行参考点返回消息
	bool ExecuteSkipMsg(RecordMsg *msg);	//执行参考点返回消息
	bool ExecuteMacroProgCallMsg(RecordMsg *msg); //实际执行宏程序调用消息
	bool ExecuteAutoToolMeasureMsg(RecordMsg *msg);   //实际执行自动对刀指令消息
	bool ExecuteRestartOverMsg(RecordMsg *msg);   //实际执行加工复位完成指令消息
	bool ExecuteInputMsg(RecordMsg *msg);		//执行G10 输入指令消息
	bool ExecuteExactStopMsg(RecordMsg *msg);   // G09

#ifdef USES_SPEED_TORQUE_CTRL	
	bool ExecuteSpeedCtrlMsg(RecordMsg *msg);   //执行速度控制消息
	bool ExecuteTorqueCtrlMsg(RecordMsg *msg);   //执行力矩控制消息
#endif	

	void PauseRunGCode();  //暂停G代码运行

	void InitGCodeMode();    //将G代码模态初始化到默认状态

	bool RefreshOuputMovePos(DPointChn &pos);   //同步已编译的轴移动指令的位置
//	bool RefreshOutputMovePos(DPoint &pos);  //同步已编译的轴移动指令的位置
	bool IsMoveMsgLine();   //同行是否有轴移动指令

	bool IsNcFileExist(char *file_name);    //NC文件是否存在


	void ProcessHmiGetChnStateCmd(HMICmdFrame &cmd);   //处理HMI获取通道状态指令
	void ProcessHmiSimulateCmd(HMICmdFrame &cmd);      //处理HMI仿真指令
	void ProcessHmiSetNcFileCmd(HMICmdFrame &cmd);     //处理HMI设置加工文件命令
	void ProcessMdaData(HMICmdFrame &cmd);				//处理MDA数据
	void ProcessHmiFindRefCmd(HMICmdFrame &cmd);			//处理回参考点命令
	void ProcessHmiGetMacroVarCmd(HMICmdFrame &cmd);	//处理获取宏变量命令
	void ProcessHmiSetMacroVarCmd(HMICmdFrame &cmd);	//处理设置宏变量命令
	void ProcessHmiClearWorkPieceCmd(HMICmdFrame &cmd);  //处理加工计数清零命令
    void ProcessHmiClearTotalPieceCmd(HMICmdFrame &cmd); //处理总共件数清零命令
	void ProcessHmiRestartCmd(HMICmdFrame &cmd);    //处理加工复位命令
    void ProcessHmiSetRequirePieceCmd(HMICmdFrame &cmd);    //处理

	void ProcessHmiSetCurMachPosCmd(HMICmdFrame &cmd);      //处理HMI设置轴当前位置的机械坐标命令

	void ProcessHmiClearMsgCmd(HMICmdFrame &cmd);     //处理HMI清除消息命令


#ifdef USES_LASER_MACHINE
	void ProcessLaserCalibration();        //处理激光调高器标定


    void ReadHeightRegulatorInput();			//读取调高器输入信号
	void SetHeightRegulatorOutput(uint8_t idx, bool value);  //设置调高器输出信号
	void SendCatchPosCmd(uint64_t axis_mask, uint16_t io_idx, bool level);         //向MI发送轴位置捕获指令
#endif


    bool SendMachineStateCmdToHmi(uint8_t mach_state);     //向HMI发送加工状态命令
	bool SendWorkModeCmdToHmi(uint8_t chn_work_mode); 		//向HMI发送工作模式命令
	bool SendChnStatusChangeCmdToHmi(uint16_t status_type);	//向HMI发送通道状态改变命令
	bool SendModeChangToHmi(uint16_t mode_type);			//向HMI发送非G模态状态改变命令，包含T/D/H/F/S
	bool SendMdaDataReqToHmi();			//向HMI发送MDA数据请求
	bool SendMessageToHmi(uint16_t msg_type, uint32_t msg_id, uint32_t msg_param = 0);		//向HMI发送提示信息
    bool SendWorkCountToHmi(uint32_t count, uint32_t totalCount);   //向HMI更新加工计数
	bool SendMachOverToHmi();    //通知HMI加工完成

	bool SendManualToolMeasureResToHmi(bool res);  //发送手动对刀结果给HMI

	bool ClearHmiInfoMsg();    //清空当前提示信息


	//设置MC状态接口
	void SendIntpModeCmdToMc(uint16_t intp_mode);      //向MC模块发送插补工作模式命令
	void SendWorkModeToMc(uint16_t work_mode);    //向MC模块发送加工模式命令，主要用于区分插补目标位置，方便计算不同模式的余移动量
    void SendMachineStateToMc(uint8_t state); //向MC模块发送SC当前加工状态
    void PauseMc();		//暂停MC模块的运行
	void SetMcAxisOrigin(uint8_t axis_index);		//向MC设置轴的工件坐标系原点
	void SetMcAxisOrigin(uint8_t axis_index, int64_t origin_pos); //向MC设置轴的工件坐标系原点
	void ActiveMcOrigin(bool active_flag);		//激活设置的工件坐标系
	void SetMcAxisToolOffset(uint8_t axis_index); 	//向MC设置各轴刀具偏置
	void ActiveMcToolOffset(bool active_flag);		//激活设置的刀具偏置
	void SendMcG31Stop();    //向MC发送G31停止命令
    //void SendMcRigidTapFlag(bool tap_flag);    //向MC发送刚性攻丝开关

	void SendMcResetCmd();		//发送MC复位指令

	uint16_t ReadMcMoveDataCount();   //读取当前MC中运动数据的数量
	bool CheckBlockOverFlag();	//当前分块到位标志是否有效
	bool CheckStepOverFlag();	//当前单段到位标志是否有效

	bool IsMcNeedStart();   //MC是否需要发送启动命令

	void StartMcIntepolate();    			//启动MC开始插补

	static void *RefreshStatusThread(void *args); //状态更新线程函数
	bool RefreshStatusFun();  	//状态更新函数

	void InitMcModeStatus();		//初始化传递给MC的模态信息
	bool IsMcModeCmd(int cmd);	//是否需要发送到MC的模态信息命令

	void RefreshModeInfo(const McModeStatus &mode);   //更新当前通道模态数据
	
	void ReadGraphAxisPos();      //读取图形模式下的高频轴位置数据
	void SendHmiGraphPosData();     //给HMI发送绘图位置数据

	void SetMiSimMode(bool flag);   //设置MI仿真模式


	void SetMcToolOffset(bool flag);      //设置MC的刀具偏置
	void SetMcRatio();		//向MC更新倍率信
	void SetMcSpecialIntpMode(uint16_t intp_type);   //设置特殊插补模式，包括极坐标插补G12.1和磨削专用指令G12.2，以及取消指令G13.1
	void SetMcRtcpMode(uint16_t cmd_type, uint16_t switch_type, int32_t h_val);   //设置五轴RTCP状态
#ifdef USES_GRIND_MACHINE
	void SetMcGrindParamDynamic(uint16_t intp_type, uint16_t data_idx, int data1, int data2);		//设置G12.2/G12.3磨削动态参数
	void EnableGrindShock(bool enable);    //使能震荡磨
	void ReturnSafeState();    //执行复位安全位置动作

	void ProcessGrindM66(AuxMsg *msg);   //处理M66指令，主机上下料请求
	void ProcessGrindM66_slave(AuxMsg *msg);   //处理从机M66指令，从机上下料请求
#endif

#ifdef USES_WOOD_MACHINE
	void SetMcToolLife();   //向MC设置当前刀具寿命    木工机刀具寿命以切削长度计算

	void PreExecSpindleCmd(uint64_t line);    //执行主轴预启动流程
	bool ExecuteAuxMsg_wood(RecordMsg *msg);    	//实际执行辅助指令消息, 木工机定制版本
#endif

    void SetChnCurWorkPiece(int newCnt);   //设置当前加工件数
	int GetCurToolLife();    //获取当前刀具寿命

	uint8_t GetCurPhyAxis();		//获取当前物理轴号

	void SetAxisRefCur(uint8_t axis);	//设定指定轴的当前编码器值为零点
	void SetAxisRefEncoder(uint8_t axis, int64_t encoder);		//设置指定轴的零点位置对应的编码器值
	void ActiveAxisZCapture(uint8_t axis);		//激活指定轴的Z信号捕获功能

	void SendMiClearPosCmd(uint8_t axis, int mode); //发送轴位置清整数圈指令

	void SendSpdSpeedToMi(uint8_t axis, int16_t da_value);   //发送主轴转速对应的DA值到MI

	bool PmcRapidMove(uint8_t chn_axis, double tar_pos, bool inc);    //PMC轴定位移动
	bool PmcLineMove(uint8_t total, uint8_t idx, uint8_t chn_axis, double tar_pos, double vel, bool inc);   //PMC轴切削移动

#ifdef USES_SPEED_TORQUE_CTRL
	void SendAxisCtrlModeSwitchCmdToMi(uint8_t axis, uint8_t value);   //发送轴控制模式切换命令到MI
	
	void SendAxisSpeedCmdToMi(uint8_t axis, int32_t value);   //发送速度轴运行速度值到MI
	
	void SendAxisTorqueCmdToMi(uint8_t axis, int16_t value, int16_t speed_lmt,int16_t flag);   //发送力矩轴运行力矩值到MI

	void ProcessCtrlModeSwitch(AuxMsg *msg, uint8_t index);   //处理控制模式动态切换M指令
	bool ResetAllAxisOutZero(void);
	bool ResetAllAxisCtrlMode(uint8_t flag);

	void ProcessSpdModeSwitch(AuxMsg *msg, uint8_t index);  //处理主轴CS模式切换流程
#endif

    void SendAxisMapCmdToMi(uint8_t phy_axis,uint8_t chan,uint8_t axis);   //发送物理轴与通道轴映射关系设置命令

	void SendMCodeToPmc(uint32_t m_code, uint8_t m_index);  //将M代码发送给PMC
	void SetMFSig(uint8_t index, bool value);      //设置MFn选通信号
	bool GetMFINSig(uint8_t index);    //返回指定序号的MFIN信号
	bool GetMExcSig(uint8_t index);    //返回指定序号的MExc信号

	void SendTCodeToPmc(uint32_t t_code, uint8_t index);			//将T指令发送至PMC寄存器
	void SetTFSig(uint8_t index, bool value);		//设置TF选通信号
	bool GetTFINSig(uint8_t index);     //返回指定序号的TFIN信号

	void SpindleSpeedDaOut(int32_t speed);		//将主轴转速转换为电平值


	void RefreshAxisIntpPos();   //将MC的各轴插补位置刷新到通道实时状态中

	bool CancelBreakContinueThread();	//退出断点继续线程
	
	bool IsBufClearMsg(RecordMsg *msg);   //判断是否缓冲清空消息
	bool IsRunmoveMsg(RecordMsg *msg);    //是否轴移动消息
	ListNode<RecordMsg *> *FindMsgNode(uint16_t index);   //在自动数据缓冲中查找指定帧号的数据

	void ProcessAxisMapSwitch(AuxMsg *msg, uint8_t index);   //处理轴映射动态切换M指令

	void GetHmiToolOffset(const uint8_t idx, HmiToolOffsetConfig &cfg);     //获取指定刀偏的数据
	bool NotifyHmiToolOffsetChanged(uint8_t h_idx);     //通知HMI刀具偏置参数值发生变更
	bool NotifyHmiWorkcoordChanged(uint8_t coord_idx);   //通知HMI工件坐标系设置发生变更
	bool NotifyHmiWorkcoordExChanged(uint8_t coor_idx);  //通知HMI 扩展坐标系设置发生变更
    bool NotifyHmiToolPotChanged();    //通知HMI刀具信息发生改变
    bool NotifyHmiMCode(int mcode);

	void DoRestart(uint64_t line_no);     //加工复位执行函数
	void InitRestartMode();         //初始化加工复位中间模态

	void ExecMCode(AuxMsg *msg, uint8_t index);     //具体执行M指令系统动作

//	void SendMiTapAxisCmd(uint16_t spd, uint16_t zAxis);   //发送攻丝轴号给MI
//	void SendMiTapParamCmd();      //发送攻丝参数给MI
//	void SendMiTapRatioCmd(int32_t ratio);   //发送攻丝比例给MI
//	void SendMiTapStateCmd(bool state);   //发送攻丝状态给MI

    //void ProcessM29Reset();     //处理M29状态复位流程

	void MiDebugFunc(int mcode);      //发送MI调试指令



#ifdef USES_TWINING_FUNC
	void ActiveTwiningFunc(int active);   //开关缠绕功能
#endif


private://私有成员变量
	uint8_t m_n_channel_index;   //通道索引号

	Compiler *m_p_compiler;     //nc代码编译器
	ChannelStatusCollect m_channel_status;   //通道状态
	ChannelRealtimeStatus m_channel_rt_status;  //通道实时状态

	ChannelMcStatus m_channel_mc_status;   //通道MC状态

	ChannelEngine *m_p_channel_engine;   //通道引擎指针
	HMICommunication *m_p_hmi_comm;    //HMI通讯接口
	MICommunication *m_p_mi_comm;		//MI通讯接口
	MCCommunication *m_p_mc_comm;		//MC通讯接口
    MCArmCommunication *m_p_mc_arm_comm;   //MC-ARM通讯接口
    SpindleControl *m_p_spindle;        //主轴模块
	SCSystemConfig *m_p_general_config;   //系统配置
	SCChannelConfig *m_p_channel_config;  //通道配置
	SCAxisConfig *m_p_axis_config;        //轴配置
    SCCoordConfig *m_p_chn_coord_config;  //工件坐标系配置
	SCCoordConfig *m_p_chn_ex_coord_config; //扩展工件坐标系配置
	SCCoordConfig *m_p_chn_g92_offset;    //@add zk G92 全局坐标系偏移

	SCToolOffsetConfig *m_p_chn_tool_config;		//刀具偏置配置
	SCToolPotConfig *m_p_chn_tool_info;     //刀具信息，包括刀具类型、刀套号、寿命等

	ChnProcParamGroup *m_p_chn_proc_param;    //工艺相关通道参数
	AxisProcParamGroup *m_p_axis_proc_param;   //工艺相关轴参数

#ifdef USES_FIVE_AXIS_FUNC
	FiveAxisConfig *m_p_chn_5axis_config;    //五轴配置
	uint8_t m_n_5axis_rot1;      //第一旋转轴通道轴索引号，0开始
	uint8_t m_n_5axis_rot2;      //第二旋转轴通道轴索引号，0开始
	uint8_t m_mask_5axis_rot_nolimit;   //可以无限旋转的通道轴的mask
	uint64_t m_mask_5axis_rot_nolimit_phyaxis;  //可以无限旋转的通道轴的物理轴mask,
	bool m_b_5axis_clear_rot;    //五轴旋转轴清整数圈标志，不需要提前初始化，使用时会提前初始化   true--清位置   false--不清位置
#endif

#ifdef USES_GRIND_MACHINE
	GrindConfig *m_p_grind_config;       //磨床参数
	bool m_b_ret_safe_state;         //各轴回到安全状态
	uint8_t m_n_ret_safe_step;     //各轴复位步骤
	double m_df_c_pos_tar;        //C轴回正目标位置
	struct timeval m_time_ret_safe_delay;   //用于回安全位置延时执行
	struct timeval m_time_mcode_start;   //M指令执行起始时间，用于M指令执行超时告警

	StateMechArm *m_p_mech_arm_state;   //上下料机械手状态
	ParamMechArm *m_p_mech_arm_param;   //上下料机械手参数
#endif

	uint8_t m_n_spindle_count;				//主轴个数
	uint8_t m_spd_axis_phy[kMaxChnSpdCount];	//主轴对应的物理轴号,从1开始

	uint16_t m_n_mc_auto_buf_max;     //MC中自动模式运动数据缓冲大小

    uint64_t m_n_real_phy_axis;          //通道内实际物理轴Mask

	uint32_t m_mask_intp_axis;      //通道插补轴mask，按通道轴顺序
	uint8_t m_n_intp_axis_count;    //通道插补轴个数

	ErrorType m_error_code;      	//错误码

	uint8_t m_n_cur_proc_group;     //当前工艺参数组号

//	char m_str_cur_nc_file[kMaxPathLen];  //当前NC加工文件,相对路径，相对目录"/cnc/nc_files/"

	pthread_t m_thread_refresh_status;    //状态更新线程
	pthread_t m_thread_compiler;   			//G代码编译运行线程
	pthread_t m_thread_breakcontinue;		//断点继续线程

	bool m_b_init_compiler_pos;    	//是否初始化编译器当前位置，工件坐标系

	bool m_b_lineno_from_mc;    //是否从MC模块更新当前运行代码行号,true表示从MC获取，false表示不从MC获取

	CompilerState m_n_run_thread_state;   //G代码编译运行线程运行状态
//	CompilerState m_n_run_state_auto_bak;	//保存自动模式的G代码运行线程状态备份

	OutputMsgList *m_p_output_msg_list;   		//待输出至MC的指令消息队列，根据模式指向AUTO和MDA的队列
	OutputMsgList *m_p_output_msg_list_auto;   //AUTO模式待输出至MC的指令消息队列
	OutputMsgList *m_p_output_msg_list_mda;   //MDA模式待输出至MC的指令消息队列
//	GCodeFrameBuffer *m_p_output_buffer;    //运动控制数据，此缓冲区的数据只存放轴运动指令，只有来了后续指令时才把前一条指令发送出去

	ListNode<RecordMsg *> *m_p_last_output_msg;   //最近发送的运动数据
	HWTraceState m_n_hw_trace_state;        //手轮跟踪状态
	HWTraceState m_n_hw_trace_state_change_to;    //将要切换的手轮跟踪状态
	bool m_b_change_hw_trace_state;       //切换手轮状态标志   true--正在等待MC运动停止到位

	uint16_t m_n_frame_index;   //运动数据帧号 1~65535循环，0留给非运动数据

//	bool m_b_quit;   //G代码编译运行线程退出标志
	pthread_mutex_t m_mutex_change_state;    //编译运行状态切换互斥量

	SimulateMode m_simulate_mode;  //仿真模式
	ChnSimulateZone m_simulate_zone;   //切削范围

	uint8_t m_change_work_mode_to;	//即将切换的目标工作模式，等待系统停止G代码运行

	int16_t m_n_cur_tcode;		//当前T代码，M06激活,-1表示未初始化
    int32_t m_n_cur_scode;		//当前S代码，M03/M04激活,-1表示未初始化   单位：rpm
//	int16_t m_n_cur_dcode;		//当前D代码，G41/G42激活,-1表示未初始化
//	int16_t m_n_cur_hcode;		//当前H代码，G43/G44激活,-1表示未初始化

	char m_str_mda_path[kMaxPathLen];        //通道所属MDA文件绝对路径

	bool m_b_mc_need_start;			//MC运行需要发送启动插补命令，true--需要   false--不需要

	bool m_b_need_change_to_pause;   //单段暂停标志，true--等待mc运行完即切换暂停状态      false--不需要切换

	uint8_t m_n_step_change_mode_count ;    //用于单段模式状态切换确认计数，由于mc的响应延时一定要连续2次达到条件才切换状态
//	bool m_b_step_exec;    //单步执行标志

	McModeStatus m_mc_mode_exec;   //指令消息执行时修改的MC模态组信息
	McModeStatus m_mc_mode_cur;	   //最近一次从MC获取的MC模态信息

	ChannelAutoScene m_scene_auto;		//自动模式的运行状态
	int m_n_breakcontinue_segment;    //断点继续的执行阶段
	bool m_b_cancel_breakcontinue_thread;   //退出断点继续线程标志

	uint64_t m_n_send_mc_data_err;	//向MC发送数据失败计数

	uint8_t m_n_subprog_count;     	//子程序嵌套层数计数,包括宏程序调用
	uint8_t m_n_macroprog_count;    //宏程序嵌套层数计数，不包括子程序
	bool m_b_ret_from_macroprog;    //宏程序返回标志

	//用于保存循环指令参数值
	double m_df_loop_param[kMaxTagCount];  //保存数据字段的值

	bool m_b_manual_tool_measure;   //是否处于手动对刀中
	bool m_b_cancel_manual_tool_measure;   //取消手动对刀

	bool m_b_manual_call_macro;           //是否处于手动宏程序调用
	bool m_b_cancel_manual_call_macro;    //取消手动宏程序调用

#ifdef USES_ADDITIONAL_PROGRAM
	AddProgType m_n_add_prog_type;        //当前附加程序执行状态
	uint8_t m_n_sub_count_bak;            //执行附加程序时的子程序嵌套计数备份，用于返回时决定是否需要复位附加程序状态标志
#endif

	struct timeval m_time_start_mc;    //最后一次发送MC启动命令的时间，用于读取MC运行到位标志时的延时判断

	Variable m_macro_variable;          //宏变量

	PmcRegister *m_p_pmc_reg;			//PMC寄存器
	FRegBits *m_p_f_reg;			    //本通道F寄存器指针
	const GRegBits *m_p_g_reg;			//本通道G寄存器指针

	struct timeval m_time_m_start[kMaxMCodeInLine];     //M指令执行的开始时间
	struct timeval m_time_t_start[kMaxTCodeInLine];     //T指令执行的开始时间
	struct timeval m_time_start_maching;     //开始自动加工的时间，用于加工计时
    int32_t        m_time_remain;            //剩余加工时间

	uint64_t m_n_mask_clear_pos;		//位置清整数圈轴标志

	//IO触发位置捕获功能相关
	uint64_t m_n_mask_pos_capture;      //轴位置捕获Mask
	uint64_t m_n_mask_captured;         //轴位置捕获成功Mask
	DPointChn m_point_capture;		    //捕获的轴位置，机械坐标系
	bool m_b_pos_captured;             //轴位置捕获完成标志

	//PMC轴运行mask
	uint64_t m_mask_run_pmc;  //当前运行的轴的mask
	uint64_t m_mask_runover_pmc;   //当前运行完成的轴的mask

	//加工复位相关变量
    uint8_t m_n_restart_mode;     //加工复位模式，0--非加工复位   1--正常加工复位    2--快速加工复位
    uint8_t m_n_restart_step;     //加工复位步骤
    uint64_t m_n_restart_line;    //加工复位目的行号
    ChnRestartMode m_mode_restart;    //加工复位中间模态
	
	bool m_b_mc_on_arm;    //本通道所属mc通道是否运行于ARM上
    bool m_b_delay_to_reset;   //延迟复位动作，等待减速停到位后，再执行复位动作

    DPointChn m_pos_simulate_cur_work;    //仿真模式下的当前工件坐标

//	uint8_t m_n_M29_flag;   //  M29 M28 flag;   0--M28  1--M29
//	uint8_t m_n_G843_flag;  //  G843 flag;   0--非刚攻跟随状态         1--刚攻跟随状态
//	int32_t m_n_G843_ratio; // G843 ratio; 跟随刚性攻丝的比例值， 该值*10000倍，

	bool m_b_hmi_graph;    //HMI是否处于图形模式，此模式需要发送实时高频位置数据
	uint8_t m_n_xyz_axis_phy[3];    //XYZ轴对应的物理轴号，0开始
	CoordAxisPos m_pos_graph_array[kGraphPosBufCount];   //缓冲图形模式位置数据
	uint16_t m_n_graph_pos_write_idx;     //当前空闲的位置缓冲写入索引
	uint16_t m_n_graph_pos_read_idx;      //当前位置数据读取索引
    uint16_t m_n_graph_pos_count;   //当前位置缓冲数量
	

#ifdef USES_SPEED_TORQUE_CTRL
    uint8_t m_n_need_reset_axis_ctrlmode;				// 各轴复位计数值，当为0时，标识不需要复位，从某个数递减，减为0，标识复位成功
#endif

#ifdef USES_WOOD_MACHINE
	//刀具寿命刷新延时标志
	uint8_t m_n_ref_tool_life_delay;    //用于切换当前刀号时，延时刷新刀具寿命
	bool m_b_save_tool_info;         //掉电时是否保存刀具信息
	
	bool m_b_prestart_spd;    //主轴预启动执行标志    true--当前正在执行
	int m_n_spd_prestart_step;   //当前主轴预启动执行步骤， 从0开始，结束后为0xFF
	SpindleStartOffset m_spd_prestart_offset;   //当前所执行的主轴预启动数据
#endif


#ifdef USES_LASER_MACHINE
	bool m_b_laser_calibration;			//进行激光调高器标定
	bool m_b_cancel_calib;				//退出标定流程
	uint8_t m_n_calib_step;				//标定步骤
	uint8_t m_n_calib_ref_count;		//标定点数
	uint8_t m_n_cur_ref_point;		    //当前标定位置点索引
	uint8_t m_n_laser_input_signal;          //调高器输入信号
	uint8_t m_n_follow_range;           //调高器跟随高度范围      单位:mm
	struct timeval m_time_delay_start;   //等待起始时间
#endif

#ifdef USES_WUXI_BLOOD_CHECK
	bool m_b_returnning_home;      //回参考点中
	uint8_t m_n_ret_home_step;     //回零步骤
	struct timeval m_time_start_wait;   //等待起始时间
	uint32_t m_n_warn_status;       //告警状态
#endif

#ifdef USES_SIMULATION_TEST
	int m_file_sim_data;      //仿真数据保存文件句柄
#endif

	double G52offset[kMaxAxisChn];
	bool G52Active = false;

};

#endif /* INC_CHANNEL_CHANNEL_CONTROL_H_ */
