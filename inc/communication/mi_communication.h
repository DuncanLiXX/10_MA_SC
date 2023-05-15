/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file mi_communication.h
 *@author gonghao
 *@date 2020/06/15
 *@brief 本头文件为SC-MI通讯类的声明
 *@version
 */

#ifndef SRC_COMMUNICATION_MI_COMMUNICATION_H_
#define SRC_COMMUNICATION_MI_COMMUNICATION_H_

#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include "hmi_shared_data.h"
#include "comm_data_definition.h"
#include "list_buffer.h"

class ChannelEngine;   //通道引擎

//命令通道FIFO深度
#define MI_CMD_FIFO_DEPTH  (32)	//32个命令帧深度

//命令帧字节数
#define MI_CMD_FRAME_SIZE (24)

//PMC运动指令通道FIFO深度
#define PMC_CMD_FIFO_DEPTH (32)   //32个命令帧深度

//PMC运动指令帧字节数
#define PMC_CMD_FRAME_SIZE (24)

//PMC文件数据区大小
#define MI_PMC_DATA_SIZE (131072)	//128KB

//共享内存地址定义
#define SHARED_MEM_BASE		(0x1FE00000)		//共享内存首地址，512M内存
//#define SHARED_MEM_BASE		(0x0FE00000)		//共享内存首地址，256M内存

#define SHARE_MEM_MUTEX        (SHARED_MEM_BASE + 0x0)             //访问互斥量
#define SHARED_MEM_AXIS_POS    (SHARED_MEM_BASE + 0x100)           //通道轴位置

//各数据段基地址
#define SHARED_MEM_CMD_SEND_BASE	(SHARED_MEM_BASE + 0x0)			//发送命令通道基地址	1K
#define SHARED_MEM_CMD_RECV_BASE   (SHARED_MEM_BASE + 0x400)		//接收命令通道基地址	1K
#define SHARED_MEM_MI_STATUS_BASE  (SHARED_MEM_BASE + 0x75000)		//MI系统状态区基地址  	4K
#define SHARED_MEM_AXIS_STATUS_BASE	(SHARED_MEM_BASE + 0x76000)	//通道轴状态区基地址  	4K
#define SHARED_MEM_SC_STATUS_BASE   (SHARED_MEM_BASE + 0x78000)    //SC系统状态区基地址    4K
#define SHARED_MEM_PMC_AXIS_POS_BASE (SHARED_MEM_BASE + 0x7FC00)   //PMC运动指令通道基地址 1K
#define SHARED_MEM_PMC_FREG_BASE	(SHARED_MEM_BASE + 0x80000)		//PMC寄存器基地址		16K
#define SHARED_MEM_PMC_GREG_BASE   (SHARED_MEM_BASE + 0x84000)		//PMC寄存器基地址		16K
#define SHARED_MEM_PMC_LADDER_BASE (SHARED_MEM_BASE + 0x90000)		//PMC梯形图基地址		128K

//物理轴位置寄存器
#define SHARED_MEM_AXIS_MAC_POS_INTP(n) (SHARED_MEM_AXIS_STATUS_BASE+0x40*(n)) 			//第n轴的插补机械坐标
#define SHARED_MEM_AXIS_MAC_POS_FB(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x08)   			//第n轴的反馈机械坐标
#define SHARED_MEM_AXIS_SPEED(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x10)   					//第n轴的速度
#define SHARED_MEM_AXIS_TRK_ERR(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x14)   				//第n轴的跟随误差
#define SHARED_MEM_AXIS_ENCODER(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x18)                  //第n轴的当前编码器反馈值
#define SHARED_MEM_PMC_AXIS_REMAIN_DIS(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x20)           //第n轴的余移动量（PMC轴有效）
#define SHARED_MEM_AXIS_MAC_POS_INTP_AFTER(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x28)       //第n轴的插补后加减速之后输出的机械坐标
#define SHARED_MEM_AXIS_TORQUE(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x30)   			    //第n轴的实时力矩反馈
#define SHARED_MEM_AXIS_SPD_ANGLE(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x32)   			    //第n轴的实时主轴角度反馈
#define SHARED_MEM_AXIS_READ_OVER (SHARED_MEM_AXIS_STATUS_BASE+0x1000)   					//轴数据的读取完成标志
#define SHARED_MEM_AXIS_WRITE_OVER (SHARED_MEM_AXIS_STATUS_BASE+0x1004)   				    //轴数据的写入完成标志
#define SHARED_MEM_TAP_ERR(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x1040)                  //第n个攻丝组的刚性攻丝误差
#define SHARED_MEM_TAP_ERR_NOW(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x1044)                  //第n个攻丝组的刚性攻丝误差
#define SHARED_MEM_TAP_READ_OVER (SHARED_MEM_AXIS_STATUS_BASE+0x1240)                  //第n个攻丝组的刚性攻丝误差
#define SHARED_MEM_TAP_WRITE_OVER (SHARED_MEM_AXIS_STATUS_BASE+0x1244)                  //第n个攻丝组的刚性攻丝误差
//通道状态寄存器


//系统状态寄存器
#define SHARED_MEM_MI_STATUS_WARN      (SHARED_MEM_MI_STATUS_BASE)				//告警分类标志，uint8_t类型
#define SHARED_MEM_MI_STATUS_HLIMIT_POS  (SHARED_MEM_MI_STATUS_BASE + 0x08)     //轴正限位告警
#define SHARED_MEM_MI_STATUS_HLIMIT_NEG  (SHARED_MEM_MI_STATUS_BASE + 0x10)     //轴负限位告警
#define SHARED_MEM_MI_STATUS_ENCODER_WARN (SHARED_MEM_MI_STATUS_BASE + 0x18)    //编码器告警标志,uint64_t类型
#define SHARED_MEM_MI_STATUS_SVO_WARN  (SHARED_MEM_MI_STATUS_BASE + 0x20)       //伺服告警标志,uint64_t类型


#define SHARED_MEM_MI_STATUS_SVO_ON     (SHARED_MEM_MI_STATUS_BASE + 0x28)              //轴使能标志，uint64_t类型，一个bit代表一个轴
#define SHARED_MEM_MI_STATUS_SVO_TRACK_ERR (SHARED_MEM_MI_STATUS_BASE + 0x30)      //轴跟随误差超限告警
#define SHARED_MEM_MI_STATUS_SVO_SYNC_POS_ERR   (SHARED_MEM_MI_STATUS_BASE + 0x38)      //同步轴位置偏差过大告警
#define SHARED_MEM_MI_STATUS_SVO_INTP_POS_ERR  (SHARED_MEM_MI_STATUS_BASE + 0x40)      //位置指令过大告警
#define SHARED_MEM_MI_STATUS_SVO_SYNC_TORQUE_ERR  (SHARED_MEM_MI_STATUS_BASE + 0x48)    //同步轴力矩偏差过大告警
#define SHARED_MEM_MI_STATUS_SVO_SYNC_MACH_ERR  (SHARED_MEM_MI_STATUS_BASE + 0x50)      //同步轴机床坐标偏差过大告警

#define SHARED_MEM_MI_STATUS_SVO_WARN_CODE(n)  (SHARED_MEM_MI_STATUS_BASE + 0x100 + 0x04*n)       //第n轴的伺服告警码


//SC状态寄存器
#define SHARED_MEM_SC_STATUS_HLIMIT_POS   (SHARED_MEM_SC_STATUS_BASE)      //物理轴正向限位标志
#define SHARED_MEM_SC_STATUS_HLIMIT_NEG   (SHARED_MEM_SC_STATUS_BASE + 0x08)     //物理轴负向限位标志

//命令上行通道寄存器  MI->SC
#define SHARED_MEM_CMD_RECV_WF(n)	(SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n)		//第n个命令帧的写标志
#define SHARED_MEM_CMD_RECV_RF(n)	(SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n + 0x02) //第n个命令帧的读标志
#define SHARED_MEM_CMD_RECV_DATA0(n)  (SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n + 0x04) 	//第n个命令帧的第0个32位数据
#define SHARED_MEM_CMD_RECV_DATA1(n)  (SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n + 0x08) 	//第n个命令帧的第1个32位数据
#define SHARED_MEM_CMD_RECV_DATA2(n)  (SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n + 0x0C) 	//第n个命令帧的第2个32位数据
#define SHARED_MEM_CMD_RECV_DATA3(n)  (SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n + 0x10) 	//第n个命令帧的第3个32位数据
#define SHARED_MEM_CMD_RECV_DATA4(n)  (SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n + 0x14) 	//第n个命令帧的第4个32位数据


//命令下行通道寄存器  SC->MI
#define SHARED_MEM_CMD_SEND_WF(n)	(SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n)			//第n个命令帧的写标志
#define SHARED_MEM_CMD_SEND_RF(n)	(SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n + 0x02) 	//第n个命令帧的读标志
#define SHARED_MEM_CMD_SEND_DATA0(n)  (SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n + 0x04) 	//第n个命令帧的第0个32位数据
#define SHARED_MEM_CMD_SEND_DATA1(n)  (SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n + 0x08) 	//第n个命令帧的第1个32位数据
#define SHARED_MEM_CMD_SEND_DATA2(n)  (SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n + 0x0C) 	//第n个命令帧的第2个32位数据
#define SHARED_MEM_CMD_SEND_DATA3(n)  (SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n + 0x10) 	//第n个命令帧的第3个32位数据
#define SHARED_MEM_CMD_SEND_DATA4(n)  (SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n + 0x14) 	//第n个命令帧的第4个32位数据

//PMC轴运动指令通道
#define SHARED_MEM_CMD_PMC_WF(n)	(SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n)		//第n个命令帧的写标志
#define SHARED_MEM_CMD_PMC_RF(n)	(SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n + 0x02) //第n个命令帧的读标志
#define SHARED_MEM_CMD_PMC_DATA0(n)  (SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n + 0x04) 	//第n个命令帧的第0个32位数据
#define SHARED_MEM_CMD_PMC_DATA1(n)  (SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n + 0x08) 	//第n个命令帧的第1个32位数据
#define SHARED_MEM_CMD_PMC_DATA2(n)  (SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n + 0x0C) 	//第n个命令帧的第2个32位数据
#define SHARED_MEM_CMD_PMC_DATA3(n)  (SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n + 0x10) 	//第n个命令帧的第3个32位数据
#define SHARED_MEM_CMD_PMC_DATA4(n)  (SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n + 0x14) 	//第n个命令帧的第4个32位数据

//PMC寄存器数据地址定义
//NC->PMC
#define SHARED_MEM_REG_TO_PMC_BASE 	(SHARED_MEM_BASE + 0x80000)				//NC->PMC寄存器首地址
#define SHARED_MEM_REG_TO_PMC_F		(SHARED_MEM_REG_TO_PMC_BASE + 0x0)		//F寄存器地址

//PMC->NC
#define SHARED_MEM_REG_TO_NC_BASE  	(SHARED_MEM_BASE + 0x84000)				//PMC->NC寄存器首地址
#ifndef USES_PMC_2_0

#define SHARED_MEM_REG_TO_NC_G			(SHARED_MEM_REG_TO_NC_BASE + 0x0)		//G寄存器地址
#define SHARED_MEM_REG_TO_NC_X			(SHARED_MEM_REG_TO_NC_BASE + 0x0800)	//X寄存器地址
#define SHARED_MEM_REG_TO_NC_Y			(SHARED_MEM_REG_TO_NC_BASE + 0x0900)	//Y寄存器地址
#define SHARED_MEM_REG_TO_NC_R			(SHARED_MEM_REG_TO_NC_BASE + 0x0A00)	//R寄存器地址
#define SHARED_MEM_REG_TO_NC_K			(SHARED_MEM_REG_TO_NC_BASE + 0x0E00)	//K寄存器地址
#define SHARED_MEM_REG_TO_NC_A			(SHARED_MEM_REG_TO_NC_BASE + 0x0E80)	//A寄存器地址
#define SHARED_MEM_REG_TO_NC_D			(SHARED_MEM_REG_TO_NC_BASE + 0x1000)	//D寄存器地址
#define SHARED_MEM_REG_TO_NC_C			(SHARED_MEM_REG_TO_NC_BASE + 0x2000)	//C寄存器地址
#define SHARED_MEM_REG_TO_NC_T			(SHARED_MEM_REG_TO_NC_BASE + 0x2200)	//T寄存器地址
#define SHARED_MEM_REG_TO_NC_DC		(SHARED_MEM_REG_TO_NC_BASE + 0x2400)	//DC寄存器地址
#define SHARED_MEM_REG_TO_NC_DT		(SHARED_MEM_REG_TO_NC_BASE + 0x2600)	//DT寄存器地址

#else

#define SHARED_MEM_REG_TO_NC_G			(SHARED_MEM_REG_TO_NC_BASE + 0x0)		//G寄存器地址
#define SHARED_MEM_REG_TO_NC_X			(SHARED_MEM_REG_TO_NC_BASE + 0x0800)	//X寄存器地址
#define SHARED_MEM_REG_TO_NC_Y			(SHARED_MEM_REG_TO_NC_BASE + 0x0A00)	//Y寄存器地址
#define SHARED_MEM_REG_TO_NC_R			(SHARED_MEM_REG_TO_NC_BASE + 0x0D00)	//R寄存器地址
#define SHARED_MEM_REG_TO_NC_K			(SHARED_MEM_REG_TO_NC_BASE + 0x01500)	//K寄存器地址
#define SHARED_MEM_REG_TO_NC_A			(SHARED_MEM_REG_TO_NC_BASE + 0x01600)	//A寄存器地址
#define SHARED_MEM_REG_TO_NC_D			(SHARED_MEM_REG_TO_NC_BASE + 0x1700)	//D寄存器地址
#define SHARED_MEM_REG_TO_NC_C			(SHARED_MEM_REG_TO_NC_BASE + 0x2700)	//C寄存器地址
#define SHARED_MEM_REG_TO_NC_T			(SHARED_MEM_REG_TO_NC_BASE + 0x2800)	//T寄存器地址
#define SHARED_MEM_REG_TO_NC_E		    (SHARED_MEM_REG_TO_NC_BASE + 0x2900)	//E寄存器地址
//#define SHARED_MEM_REG_TO_NC_TC         (SHARED_MEM_REG_TO_NC_BASE + 0x2900)    //T寄存器当前值
//#define SHARED_MEM_REG_TO_NC_TM         (SHARED_MEM_REG_TO_NC_BASE + 0x2A00)    //T寄存器Mask
//#define SHARED_MEM_REG_TO_NC_E		    (SHARED_MEM_REG_TO_NC_BASE + 0x2B00)	//E寄存器地址

#endif



typedef ListBuffer<MiCmdFrame> MiCmdBuffer;   //待输出的指令消息队列
typedef ListBuffer<PmcCmdFrame> PmcCmdBuffer;  //待输出的PMC运动指令队列


/**
 * @brief SC-MI通讯类，主要负责SC模块与MI模块建的通讯，采用共享内存方式
 */
class MICommunication {
public:

	virtual ~MICommunication();	//析构函数

	static MICommunication *GetInstance();   //单例模式，获取此类实例的唯一访问点

	void SetInterface();   //设置接口

	ErrorType GetErrorCode(){return m_error_code;}	//返回最近的错误码

    void ReadPhyAxisCurFedBckPos(double *pos_fb, double *pos_intp,double *speed,double *torque,double *angle, uint8_t count);    //读取物理轴位置、速度、力矩
	void ReadPhyAxisFbPos(double *pos_fb, uint8_t *phy_axis, uint8_t count); //读取指定物理轴的反馈位置
#ifdef USES_SPEED_TORQUE_CTRL
	void ReadPhyAxisCurFedBckSpeedTorque(double *speed, double *torque, uint8_t count);    //读取物理轴速度和力矩
#endif
    void ReadSpindleAndle(uint16_t *angle,uint8_t count);   //读取主轴实时角度反馈

//	void ReadPhyAxisIntpPos(double *pos, uint8_t count);   //读取物理轴机械插补位置
	bool ReadPhyAxisSpeed(int32_t *speed, uint8_t index);  	//读取指定物理轴速度
    void ReadPhyAxisEncoder(int64_t *encoder, uint8_t count);   //读取物理轴当前编码器反馈
	void ReadPmcAxisRemainDis(double *pos, uint8_t count);    //读取PMC轴余移动量
    void ReadTapErr(int32_t *err, int32_t *err_now,uint8_t cnt = 8);  // 读取特定攻丝组的攻丝误差(最大值)
	bool ReadCmd(MiCmdFrame &data);    //读取指令
	bool WriteCmd(MiCmdFrame &data, bool resend = false);   //发送指令
	bool SendPmcCmd(PmcCmdFrame &data, bool resend = false);     //发送PMC轴运动指令
	bool GetPmcRegByte(const int reg_type, uint8_t &value);  	//获取PMC单字节寄存器的值
	bool SetPmcRegByte(const int reg_type, const uint8_t value);	//设置PMC单字节寄存器的值

	bool GetPmcRegWord(const int reg_type, uint16_t &value);  	//获取PMC双字节字节寄存器的值
	bool SetPmcRegWord(const int reg_type, const uint16_t value);	//设置PMC双字节寄存器的值

	bool ReadPmcReg(int sec, uint8_t *reg);		//读取PMC寄存器，按地址段读取
    bool WritePmcReg(int sec, uint8_t *reg);    //写入PMC寄存器，按地址段写入
	bool ReadPmcPeriod();
    //bool ReadESPSignal();                       //读取X急停信号

    bool SetAxisRef(uint8_t axis, int64_t encoder);		//设置轴参考点
    void SetAxisRefCur(uint8_t axis, double mach_pos);		//设置指定轴的参考点

	bool StartMI();   //启动MI程序

	char *GetPmcDataAddr();		//获取PMC梯形图数据区的地址

    // opt:操作类型，详细可查看[SC-MI命令参数]文档
    // axis:轴号，从1开始
    // enable: 0:功能关 1:功能开
    void SendOperateCmd(uint16_t opt, uint8_t axis, uint16_t enable);   //发送轴操作指令
    void SendAxisEnableCmd(uint8_t axis, bool enable, uint8_t pos_req = 0);   // 轴上使能
    void SendAxisMLK(uint8_t axis, bool MLK);    // 轴机械锁住

    // spd_aixs: 主轴轴号，从1开始
    // z_axis: z轴轴号，从1开始
    void SendTapAxisCmd(uint8_t chn,uint8_t spd_axis,uint8_t z_axis); // 发送攻丝轴号

    void SendTapRatioCmd(uint8_t chn,int32_t ratio); // 发送攻丝比例
    // 发送同步误差增益，轴速度前馈增益，轴位置比例增益
    void SendTapParams(uint8_t chn,uint32_t error_gain,
                       uint32_t feed_gain,uint32_t ratio_gain);
    void SendTapStateCmd(uint8_t chn,bool enable, bool rtnt = false); // 发送攻丝状态给MI
    // axis: 主轴轴号，从1开始
    void SendSpdLocateCmd(uint8_t chn, uint8_t axis, bool enable); // 发送主轴定位命令
    // axis: 轴号，从1开始
    // type: 0—直线型位置控制输出；1—旋转型位置控制输出；2—速度指令输出；3—力矩指令输出
    void SendAxisCtrlModeSwitchCmd(uint8_t axis,uint8_t type);
    // enable: 0:MI使用默认的步长逻辑  1:自定义手轮步长
    // step: 手轮步长 单位:um
    void SendMpgStep(uint8_t chn,bool enable,uint16_t step);
    // mask: 64bit对应最多64个轴,需要同步的轴对应位置1
    void SendSyncAxis(int64_t mask);
    // master: 主动轴号，从1开始
    // slave: 从动轴号，从1开始
    // enable: 0:同步无效  1:同步有效
    void SendEnSyncAxis(uint8_t master, uint8_t slave, bool enable);
    // chn: 通道号
    // enable: 是否开启手轮跟踪
    // reverse: 是否反向跟踪
    void SendHandwheelState(uint8_t chn, bool trace_enable, bool trace_reverse,
                            bool insert_enable);
    // chn: 通道号
    // enable: 是否开启手轮插入
    void SendHandwheelInsert(uint8_t chn, bool enable);
    // chn: 通道号
    // axis: 手轮插入轴选
    void SendHandwheelInsertAxis(uint8_t chn, uint8_t axis);
    // is_positive: 是否为正限位
    // mask: 告警掩码 每一bit代表一个轴是否处于告警状态
    void SendHardLimitState(bool is_positive,uint64_t mask);

	bool ReadEncoderWarn(uint64_t &value);		//读取轴编码器告警标志
	bool ReadServoHLimitFlag(bool pos_flag, uint64_t &value);   //读取伺服限位告警信息
    bool ReadServoWarn(uint64_t &value);			//读取轴伺服告警标志
    bool ReadServoWarnCode(uint8_t axis, uint32_t &value); 	//读取指定轴的伺服告警码
    bool ReadAxisWarnFlag(uint64_t &warn);        //读取轴告警标志
    bool ReadServoOnState(uint64_t &value);		//读取轴使能状态
    bool ReadTrackErr(uint64_t &value);        //读取轴跟踪误差过大告警
    bool ReadSyncPosErr(uint64_t &value);     //读取同步轴指令偏差过大告警
    bool ReadIntpPosErr(uint64_t &value);      //读取轴位置指令过大告警
    bool ReadSyncTorqueErr(uint64_t &value);   //读取同步轴力矩偏差报警
    bool ReadSyncMachErr(uint64_t &value);     //读取同步轴机床坐标偏差报警

    bool WriteAxisHLimitFlag(bool pos_flag, const uint64_t value);   //写入物理轴硬限位标志

    template<typename T>
    void SendMiParam(uint8_t axis, uint32_t para_no, T data)  //发送MI参数
    {
        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.cmd = CMD_MI_SET_PARAM;
        cmd.data.axis_index = axis;
        memcpy(cmd.data.data, &para_no, 4);
        memcpy(&cmd.data.data[2], &data, sizeof(T));
        WriteCmd(cmd);
    }


private://私有接口
	MICommunication();	//构造函数

	int InitThread();    //初始化线程
	int QuitThread();		//退出线程

	void InitCmdChannel();   //初始化命令通道

	static void *ProcessCmdThread(void *args); //接收命令处理线程函数


	bool ReadRegister64(const uint32_t addr, int64_t& value);	 	//寄存器读函数
	bool ReadRegister32(const uint32_t addr, int32_t& value);		//寄存器读取函数
	bool WriteRegister32(const uint32_t addr, const int32_t value);	//寄存器写函数
	bool WriteRegister64(const uint32_t addr, const int64_t value);	//寄存器写函数


	bool InitSharedMemory();   	//初始化共享内存
	bool CloseSharedMemory();      //关闭共享内存

	bool ProcessCmdFun();  	//命令处理函数

	void CalMiCmdCrc(MiCmdFrame &cmd);     //计算MI命令包的CRC
	void CalPmcCmdCrc(PmcCmdFrame &cmd);   //计算PMC命令包的CRC

private://私有成员变量
	static MICommunication *m_p_instance;    //单实例对象

	ChannelEngine *m_p_channel_engine;   //通道引擎指针

	int m_n_threshold;      //缓冲预警阈值

	uint8_t *m_p_shared_base;    //共享区基地址
	int m_n_mem_file;         //文件打开句柄

	ErrorType m_error_code;    //最近一次的错误码

	pthread_t m_thread_process_cmd;    //命令处理线程

	uint8_t m_n_cur_send_cmd_index;    //当前写命令帧序号   0~MI_CMD_FIFO_DEPTH循环
	uint8_t m_n_cur_recv_cmd_index;		//当前读命令帧序号  0~MI_CMD_FIFO_DEPTH循环

	pthread_mutex_t m_mutex_cmd_down;   //下行命令通道互斥量,写入命令互斥
	pthread_mutex_t m_mutex_pmc_cmd;    //pmc运动指令通道互斥量，写入指令互斥

	MiCmdBuffer *m_list_cmd;		//待输出指令缓冲
	PmcCmdBuffer *m_list_pmc;       //待输出的PMC运动指令缓冲

	uint8_t m_n_cur_send_pmc_index;  //当前写PMC运动指令帧序号   0~PMC_CMD_FIFO_DEPTH循环
};

#endif /* SRC_COMMUNICATION_MI_COMMUNICATION_H_ */
