/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file hmi_shared_data.h
 *@author gonghao
 *@date 2020/03/19
 *@brief 本头文件包含与HMI共用的数据的声明，此文件必须与HMI保持同步更新
 *@version
 */

#ifndef INC_HMI_SHARED_DATA_H_
#define INC_HMI_SHARED_DATA_H_

#include <stdint.h>
#include <string>
#include <list>
#include "global_definition.h"

//HMI命令包相关常量
const int kMaxHmiDataLen = 1024;   //HMI命令包中固定数据区的长度
const int kHmiCmdHeadLen = 10;     //HMI命令包头的长度
const int kMaxHmiCmdFrameLen = kMaxHmiDataLen + kHmiCmdHeadLen;   //HMI命令包总长度

//版本字符串最大长度
const int kMaxVersionLen = 32;

//系统最大轴数
const int kMaxAxisNum = 32;    //12;


const uint8_t kHmiCmdTimeout = 10;    //HMI命令超时时间，单位100ms
const uint8_t kHmiCmdResend = 2;     //HMI命令包重发次数

const uint16_t kMaxHmiCmdFrameIndex = 0X7FFF;    //HMI命令包最大帧号

//通道相关
const int kMaxChnCount = 4;   //最大通道数量
const int kMaxAxisChn = 8;   //每个通道的最大轴数量
const int kMaxSpindleChn = 6;   //每个通道支持的主轴最大数量
const int kMaxGModeCount = 40;   //G代码模态组数


//注意: 此处定义与windows的定义不同，windows为256字节
const int kMaxPathLen = 128;          //路径最大长度
const int kMaxFileNameLen = 128;      //文件名最大长度

//NC文件签名串长度
const int kNcFileSignLen = 16;   //签名串长度

//刀具相关
const int kMaxToolCount = 60;    //系统支持最大刀具容量

//工艺参数最大组数
const int kMaxProcParamCount = 10;    //系统支持的工艺参数最大组数

//HMI通讯
#define PORT_UDP_CMD   (9100)   //UDP命令通讯端口
#define PORT_UDP_CMD_SEND (9101)   //UDP命令发送目的端口
#define PORT_TCP_MONITOR     (9210)  //TCP监控数据端口，用于发送
#define PORT_TCP_FILE        (9220)   //TCP文件传输端口
#define PORT_TCP_DATA        (9230)   //TCP数据传输

#define STR_HMI_SHAKE  "HELLO ARADEX CNC"  //握手字符串
#define STR_HMI_SHAKE_RESP "HELLO ARADEX HMI"  //握手回复字符串


//回复结果定义
#define APPROVE   (0x00)    //接受请求
#define REFUSE    (0x01)    //拒绝请求
#define NOTEXIST  (0x02)    //文件不存在，拒绝
#define SUCCEED   (0x00)    //执行成功
#define FAILED    (0x01)    //执行失败

//文件类型
#define FILE_UNKOWN (-1)
#define FILE_REG  (0x00)   //普通文件
#define FILE_DIR  (0x01)   //文件目录

//版本类别定义
#define SOFTWARE_VER (0x00)    //软件版本
#define HARDWARE_VER (0x01)    //硬件版本

//主轴控制模式定义
#define SPD_CTRL_SPEED   (0x00) 		//主轴速度控制
#define SPD_CTRL_POS     (0x01)		//主轴位置控制


//仿真类型
#define SIMULATE_OUTLINE (0x01)    	//轮廓仿真，绘制工件轮廓
#define SIMULATE_TOOLPATH  (0x02)		//刀路检查，加工时的实际刀路轨迹
#define SIMULATE_MACHINING (0x03)	    //加工仿真，进行虚拟加工，除了轴不实际运动，内部其它处理一致

//定义各寄存器段的长度,总长度6528bytes
#ifndef USES_PMC_2_0
#define X_REG_COUNT (128)			//X寄存器个数
#define Y_REG_COUNT (128)			//Y寄存器个数
#define F_REG_COUNT (kMaxChnCount*256)			//F寄存器个数
#define G_REG_COUNT (kMaxChnCount*256)			//G寄存器个数
#define K_REG_COUNT (64)			//K寄存器个数
#define A_REG_COUNT (64)			//A寄存器个数
#define R_REG_COUNT (512)			//R寄存器个数
#define D_REG_COUNT (1024)			//D寄存器个数
#define T_REG_COUNT (128)			//T/DT寄存器个数
#define C_REG_COUNT (128)			//C/DC寄存器个数
#else
#define X_REG_COUNT (265)			//X寄存器个数   X0~X127(通道1）+ X200~X327（通道2）+ 硬地址X1000~X1007 + 高速DI X2000  HD1~8
#define Y_REG_COUNT (260)			//Y寄存器个数   Y0~Y127(通道1）+ Y200~Y327（通道2）+ 硬地址Y1000~Y1003
#define F_REG_COUNT (kMaxChnCount*256)			//F寄存器个数   单通道F0~F255,4通道
#define G_REG_COUNT (kMaxChnCount*256)			//G寄存器个数   单通道G0~G255,4通道
#define K_REG_COUNT (20)			//K寄存器个数
#define A_REG_COUNT (25)			//A寄存器个数
#define R_REG_COUNT (2048)			//R寄存器个数
#define D_REG_COUNT (2048)			//D寄存器个数
#define T_REG_COUNT (128)			//T寄存器个数   单个寄存器占2字节，总字节数为256字节
#define C_REG_COUNT (80)			//C寄存器个数   单个寄存器占4字节，前两字节为预置值，后两字节为当前值，总字节数为320字节
//#define TC_REG_COUNT (256)          //TC寄存器个数
//#define TM_REG_COUNT (16)           //TM寄存器，16个字节
#define E_REG_COUNT (8000)         //E寄存器个数
#endif


/**
 * @brief HMICmdFrame结构用于定义HMI与SC之间的命令帧
 */
struct HMICmdFrame {
	HMICmdFrame(){frame_number = 0;}
	~HMICmdFrame(){}
	uint16_t frame_number;   //帧号，0-32767循环，bit15用于标志回复数据包
	uint16_t channel_index;  //通道号
	uint16_t cmd;            //命令号
	uint16_t cmd_extension;  //扩展命令号
	uint16_t data_len;       //数据区字节数,0~255
	char data[kMaxHmiDataLen];    //短数据区
//	char *data_huge;            //大数据区指针
};


/**
 * @brief 定义HMI与SC之间的命令编号
 */
enum HMICmdCode {
	//HMI-->SC
	CMD_HMI_HEART_BEAT = 0,		    //心跳命令 0
	CMD_HMI_DEVICE_SCAN,			//设备扫描命令 1
	CMD_HMI_SHAKEHAND,				//HMI发起的握手命令 2
	CMD_HMI_GET_FILE,             	//HMI向SC请求文件,不同的文件用扩展命令号区分 3
	CMD_HMI_SEND_FILE,              //HMI向SC发送文件，不同的文件用扩展命令号区分 4
	CMD_HMI_DISCONNECT,             //HMI关闭连接 5
	CMD_HMI_SET_PARA,               //HMI参数设置命令 6
	CMD_HMI_GET_PARA,               //HMI获取参数命令 7
    CMD_HMI_CLEAR_WORKPIECE,        //HMI请求清空加工计数,临时计数(区分白夜班) 8
	CMD_HMI_SET_CUR_CHANNEL,	    //HMI设置当前通道号 9
	CMD_HMI_NEW_ALARM,              //HMI产生新的告警 10
	CMD_HMI_RESTART,                //加工复位 11
	CMD_HMI_SIMULATE,				//加工仿真 12
	CMD_HMI_SET_NC_FILE,		    //设置当前加工文件 13
	CMD_HMI_FIND_REF_POINT = 14,	//确定参考点 14
//	CMD_HMI_MDA_CODE_TRANSFER,		//MDA代码传送 15
	CMD_HMI_GET_VERSION = 16,       //HMI获取版本信息  16
	CMD_HMI_GET_FILE_SIGNATURE,     //HMI获取指定文件的数字签名 17
	CMD_HMI_NC_FILE_COUNT,           //HMI获取SC模块存储的NC加工文件个数 18
	CMD_HMI_NC_FILE_INFO,            //HMI获取NC文件信息 19
	CMD_HMI_GET_CHN_COUNT,           //HMI获取通道数量   20
	CMD_HMI_GET_CHN_STATE,           //HMI获取通道当前状态 21
	CMD_HMI_READY,                   //HMI准备就绪
	CMD_HMI_UPDATE,					 //HMI通知SC请求进行升级文件传送
	CMD_HMI_FILE_OPERATE,            //HMI通知对NC文件进行操作，具体操作类型由扩展命令号给定
	CMD_HMI_MOP_KEY,                 //HMI虚拟MOP按键消息
	CMD_HMI_GET_PMC_REG,			 //HMI向SC请求PMC寄存器的值
	CMD_HMI_SET_PMC_REG,			 //HMI向SC设置PMC寄存器的值
	CMD_HMI_GET_PMC_UUID,			 //HMI向SC请求PMC的UUID
	CMD_HMI_SET_REF_POINT,			 //HMI通知SC将当前位置设置为当前轴的原点
	CMD_HMI_GET_MACRO_VAR,			 //HMI向SC请求宏变量的值
	CMD_HMI_SET_MACRO_VAR,			 //HMI向SC设置宏变量寄存器的值
    CMD_HMI_CLEAR_TOTAL_PIECE,       //HMI请求清空总共件数
    CMD_HMI_SET_REQUIRE_PIECE,       //HMI请求设置需求件数

//	CMD_HMI_SET_CUR_PMC_AXIS,        //HMI设置当前PMC轴  0x20
	CMD_HMI_SET_CALIBRATION = 35,	 //HMI向SC发出激光调高器标定指令  0x23
	CMD_HMI_AXIS_MANUAL_MOVE = 37,   //HMI指令SC的某轴进行手动移动   0x25
	CMD_HMI_GET_LIC_INFO = 39,       //HMI向SC请求授权信息   0x27
	CMD_HMI_SEND_LICENSE,            //HMI向SC发送授权码     0x28
	CMD_HMI_MANUAL_TOOL_MEASURE,     //HMI向SC发起手动对刀操作  0x29
	CMD_HMI_GET_ESB_INFO,            //HMI向SC获取ESB文件数据   0x2A
	CMD_HMI_ESB_OPERATE,             //HMI通知SC对指定ESB文件进行操作  0x2B
	CMD_HMI_GET_IO_REMAP,            //HMI向SC获取IO重定向数据  0x2C
	CMD_HMI_SET_IO_REMAP,            //HMI向SC设置IO重定向数据  0x2D
	CMD_HMI_SET_PROC_PARAM,          //HMI向SC设置工艺相关参数  0x2E
	CMD_HMI_GET_PROC_PARAM,          //HMI向SC获取工艺相关参数  0x2F
	CMD_HMI_SET_PROC_GROUP,          //HMI向SC设置当前工艺参数组号  0x30
	CMD_HMI_GET_PROC_GROUP,          //HMI向SC获取当前工艺参数组号  0x31
	CMD_HMI_SET_CUR_MACH_POS,        //HMI向SC重设指定轴的机械坐标   0x32
	CMD_HMI_CLEAR_MSG,               //HMI通知SC清除消息（包括错误、警告和提示)0x33
    CMD_HMI_NOTIFY_GRAPH = 0x35,      //HMI通知SC进入图形模式    0x35
	CMD_HMI_SYNC_TIME,               //HMI向SC查询当前系统时间  0x36
	CMD_HMI_CHECK_SYNC_EN,           //HMI向SC查询同步轴状态 0x37
	CMD_HMI_GET_SYS_INFO,
    CMD_HMI_CLEAR_MACHINETIME_TOTAL, //HMI向SC请求清除累计时间 0x39
    CMD_HMI_GET_HANDWHEEL_INFO,      //HMI向SC获取手轮信息 0x3A
    CMD_HMI_SET_HANDWHEEL_INFO,      //HMI向SC设置手轮信息 0x3B
    CMD_HMI_GET_ERROR_INFO,          //HMI向SC获取错误信息 0x3C
    CMD_HMI_SET_ALL_COORD,           //HMI向SC设置当前通道的所有工件坐标系 0x3D
    CMD_HMI_BACKUP_REQUEST,          //HMI向SC请求备份 0x3E
    CMD_HMI_RECOVER_REQUEST,         //HMI向SC请求恢复 0x3F
    CMD_HMI_CLEAR_ALARMFILE,         //HMI向SC请求清空报警文件 0x40
    CMD_HMI_ABSOLUTE_REF_SET,        //HMI向SC请求绝对式编码器设零 0x41
    CMD_HMI_SET_ALL_TOOL_OFFSET,     //HMI向SC请求设置所有刀偏值 0x42
    CMD_HMI_GET_CPU_INFO,            //HMI向SC获取 CPU 内存 占用信息
    CMD_HMI_SERVE_DATA_REQUEST,      //HMI向SC请求准备伺服引导
    CMD_HMI_SERVE_DATA_RESET,        //HMI向SC请求伺服引导复位
    CMD_HMI_CLEAR_IO_MAP,            //HMI向SC请求清除IO重映射数据
    CMD_HMI_GET_FILE_SYSTEM,         //HMI向SC请求文件系统
    CMD_HMI_GET_MKDIR,               //HMI向SC请求创建目录
	// 木工专用
	CMD_HMI_APPEND_ORDER_LIST,	     //HMI添加排程加工文件
	CMD_HMI_CLEAR_ORDER_LIST,		 //HMI清空排程文件列表

	//SC-->HMI
	CMD_SC_NEW_ALARM = 100,			//SC向HMI发送新的告警信息 100
	CMD_SC_CLEAR_ALARM,			    //清除告警
	CMD_SC_RESET,                   //复位
	CMD_SC_WORK_STATE,				//通知HMI更新加工状态，加工中、暂停、完成
    CMD_SC_WORK_MODE,               //工作模式切换，AUTO/MDA/MANUAL/HANDWHEEL
	CMD_SC_DISPLAY_MODE,     		//显示模式切换，加工、程序、设置、信息、调试
	CMD_SC_STATUS_CHANGE,			//状态变化，例如单段、选停、跳段、空运行、手轮跟踪等等
	CMD_SC_EMERGENCY,               //急停
	CMD_SC_RESTART_LINE,            //通知HMI当前加工复位操作扫描的行数
	CMD_SC_MODE_CHANGE,             //通知HMI非G指令模态值发生变化，包括T、D、H、F、S
	CMD_SC_DISPLAY_FILE,            //通知HMI切换显示的文件
	CMD_SC_WORKPIECE_COUNT,			//通知HMI更新工件计数值  0x6F
//	CMD_SC_REF_POINT_STATUS,        //通知HMI轴参考点状态
	CMD_SC_READY = 0x71,			//通知HMI下位机各模块已经准备就绪
	CMD_SC_MESSAGE,                 //通知HMI消息
	CMD_SC_MDA_DATA_REQ,			//向HMI请求MDA数据
	CMD_SC_UPDATE_MODULE_STATUS,	//向HMI更新模块升级状态 0x74
	CMD_SC_REQ_START,               //SC向HMI请求开始加工
	CMD_SC_TOOL_CHANGE,             //SC通知HMI刀库
	CMD_SC_MECH_ARM_ERR,            //SC通知HMI机械手上下料异常 0x77  鲁匠定制
	CMD_SC_TRANS_FILE_OVER,         //SC通知HMI文件传输完成   0x78
	CMD_SC_TOOL_MEASURE_OVER,       //SC通知HMI手动对刀结果  0x79
	CMD_SC_PARAM_CHANGED,           //SC通知HMI参数发生更改   0x7A
	CMD_SC_NOTIFY_MACH_OVER,        //SC通知HMI加工完成       0x7B
    CMD_SC_NOTIFY_MCODE,			//SC通知HMI M代码执行
    CMD_SC_NOTIFY_ALARM_CHANGE,     //SC通知HMI报警信息改变
    CMD_SC_BACKUP_STATUS,           //SC通知HMI当前备份状态     0x7E
    CMD_SC_NOTIFY_TRACELOG,         //SC通知HMI操作记录        0x7F
    CMD_SC_NOTIFY_PROTECT_STATUS,   //SC通知HMI保护状态        0x80
    CMD_SC_NOTIFY_GATHER_FINISH,    //SC通知HMI采集完成        0x81
	CMD_HMI_GUARD = 255       //HMI命令字卫兵 0xFF
};

//虚拟MOP键值定义
enum VirtualMOPKeyValue{
	MOP_KEY_AUTO = 1,					//自动
	MOP_KEY_MDA = 2,					//MDA
	MOP_KEY_JOG = 3,					//手动连续
	MOP_KEY_JOG_STEP = 4,				//手动单步
	MOP_KEY_HANDWHEEL = 5,				//手轮

	MOP_KEY_SINGLE_BLOCK = 10,			//单段
	MOP_KEY_JUMP = 11,					//跳段
	MOP_KEY_M01 = 12,					//选停
	MOP_KEY_HW_TRACE = 13,				//手轮跟踪


	MOP_KEY_G00_RATIO = 20,				//G00倍率
	MOP_KEY_FEED_RATIO = 21,			//手动/进给倍率
	MOP_KEY_SPD_RATIO = 22,				//主轴倍率

	MOP_KEY_STEP_LEN = 30,				//步长
	MOP_KEY_DIR_FOREWARD = 31,			//+
	MOP_KEY_DIR_REVERSE = 32,			//-
	MOP_KEY_RAPID = 33,					//快速
	MOP_KEY_KEY_X_AXIS = 34,			//X轴
	MOP_KEY_KEY_Y_AXIS = 35,			//Y轴
	MOP_KEY_KEY_Z_AXIS = 36,			//Z轴
	MOP_KEY_KEY_4_AXIS = 37,			//4轴
	MOP_KEY_KEY_5_AXIS = 38,			//5轴
	MOP_KEY_KEY_6_AXIS = 39,			//6轴

	MOP_KEY_LIGHT = 40,					//照明
	MOP_KEY_CHIP = 41,					//排屑
	MOP_KEY_GAS = 42,					//吹气
	MOP_KEY_COOL = 43,					//冷却

	MOP_KEY_TOOL_D = 50,				//刀具偏置
	MOP_KEY_SPD_TOOL = 51,				//刀号设定
	MOP_KEY_TOOL_MEASURE = 52,			//自动对刀
	MOP_KEY_SPD_SPEED = 55,				//主轴转速

	MOP_KEY_MAGAZINE_FOREWARD = 60,		//正选刀
	MOP_KEY_MAGAZINE_REVERSE = 61,		//逆选刀
	MOP_KEY_MAGAZINE_RESET = 62,		//刀库回零
	MOP_KEY_SPD_CLAMP = 63,				//主轴夹头
	MOP_KEY_SPD_FOREWARD = 65,			//主轴正转
	MOP_KEY_SPD_STOP = 66,				//主轴停转
	MOP_KEY_SPD_REVERSE = 67,			//主轴反转

	MOP_KEY_EMERGENCY = 70,				//急停
	MOP_KEY_CLEAR_ALARM = 80,			//清除告警
	MOP_KEY_SYS_RESET = 81,				//系统复位

	MOP_KEY_START = 100,				//循环启动
	MOP_KEY_PAUSE = 101,				//进给保持

	MOP_KEY_AXIS = 102,                 //PMC轴选定
	MOP_KEY_CUR_AXIS = 103,             //当前轴选定

#ifdef USES_GRIND_MACHINE
	MOP_KEY_Z_AXIS_POS = 500,			//Z轴正向手动
	MOP_KEY_C_AXIS_POS = 501,			//C轴正向手动
	MOP_KEY_C_AXIS_NEG = 502,			//C轴负向手动
	MOP_KEY_Z_AXIS_NEG = 503,			//Z轴负向手动
	MOP_KEY_X_AXIS_NEG = 504,           //X轴负向手动
	MOP_KEY_X_AXIS_POS = 505,			//X轴正向手动

	MOP_KEY_PMC_X_AXIS_POS = 510,       //PMC轴X正向
	MOP_KEY_PMC_X_AXIS_NEG = 511,       //PMC轴X负向
	MOP_KEY_PMC_Y_AXIS_POS = 512,       //PMC轴Y正向
	MOP_KEY_PMC_Y_AXIS_NEG = 513,       //PMC轴Y负向
	MOP_KEY_PMC_LEFT_AXIS_POS = 514,    //左侧托盘轴正向
	MOP_KEY_PMC_LEFT_AXIS_NEG = 515,    //左侧托盘轴负向
	MOP_KEY_PMC_RIGHT_AXIS_POS = 516,   //右侧托盘轴正向
	MOP_KEY_PMC_RIGHT_AXIS_NEG = 517,   //右侧托盘轴负向


	MOP_KEY_WORKPIECE_CLAMP = 600,		//工件夹紧
	MOP_KEY_VACUUM = 601,				//真空
	MOP_KEY_SPINDLE_START = 602,		//主轴启动
	MOP_KEY_SPINDLE_STOP = 603,			//主轴停止
	MOP_KEY_NEW_TRAY = 604,				//新料盘
	MOP_KEY_PAUSE_CARRY = 605,          //暂停上下料


	MOP_KEY_IO_0 = 610,			//
	MOP_KEY_IO_1 = 611,
	MOP_KEY_IO_2 = 612,
	MOP_KEY_IO_3 = 613,
	MOP_KEY_IO_4 = 614,
	MOP_KEY_IO_5 = 615,
	MOP_KEY_IO_6 = 616,
	MOP_KEY_IO_7 = 617,
	MOP_KEY_IO_8 = 618,
	MOP_KEY_IO_9 = 619,
	MOP_KEY_IO_10 = 620,
	MOP_KEY_IO_11 = 621,
	MOP_KEY_IO_12 = 622,
	MOP_KEY_IO_13 = 623,
	MOP_KEY_IO_14 = 624,
	MOP_KEY_IO_15 = 625,
	MOP_KEY_IO_16 = 626,			//
	MOP_KEY_IO_17 = 627,
	MOP_KEY_IO_18 = 628,
	MOP_KEY_IO_19 = 629,
	MOP_KEY_IO_20 = 630,
	MOP_KEY_IO_21 = 631,
	MOP_KEY_IO_22 = 632,
	MOP_KEY_IO_23 = 633,
	MOP_KEY_IO_24 = 634,
	MOP_KEY_IO_25 = 635,
	MOP_KEY_IO_26 = 636,
	MOP_KEY_IO_27 = 637,
	MOP_KEY_IO_28 = 638,
	MOP_KEY_IO_29 = 639,
	MOP_KEY_IO_30 = 640,
	MOP_KEY_IO_31 = 641,

	MOP_KEY_FETCHING_PIECE = 700,    //取料中
	MOP_KEY_START_WORKING = 701,     //取料完成


#endif

	MOP_KEY_GUARD						//卫兵
};

//错误类型定义
enum ErrorType {
	ERR_NONE = 0,        //无错误
	//系统SC内部错误  1~999
    ERR_SC_INIT         = 1,    //SC模块初始化出错
    ERR_MI_COM_INIT     = 2,    //MI通讯模块初始化出错
    ERR_MC_COM_INIT     = 3,	//MC通讯模块初始化出错
    ERR_MEMORY_NEW      = 4,    //内存分配失败
    ERR_OPEN_DIR        = 5,	//打开文件目录失败
    ERR_OPEN_FILE       = 6,	//打开文件失败
    ERR_READ_FILE       = 7,    //读取NC文件失败
    ERR_SAVEAS_FILE     = 8,    //文件另存为失败
    ERR_LOAD_MDA_DATA   = 9,    //加载MDA数据失败
    ERR_INIT_BREAKCONTINUE = 10,    //初始化断点继续线程失败
    ERR_INIT_UPDATE     = 11,   //初始化模块升级线程失败
    ERR_LOAD_SP6        = 12,   //初始化加载SPARTAN6代码失败
    ERR_LOAD_MI         = 13,   //初始化加载MI模块失败
    ERR_MC_INIT         = 14,   //MC启动失败
    ERR_MI_INIT         = 15,   //MI启动失败
    ERR_POWER_OFF       = 16,   //掉电告警
    ERR_IMPORT_CONFIG   = 17,   //配置导入后请先重启
    ERR_CHN_SPINDLE_OVERRUN = 18,   //通道主轴数量超限
    ERR_PMC_SDLINK_CONFIG   = 19,   //SDLINK配置信息错误


    ERR_SYSTEM_TIME         = 80,   //系统时间异常
    ERR_SYSTEM_FILE         = 81,   //系统文件异常
    ERR_NO_LIC              = 82,   //无授权
    ERR_LICENSE_INVALID     = 83,   //非法授权
    ERR_LIC_OVERTIME        = 84,   //系统授权已过期
    ERR_LIC_DUE_SOON        = 85,   //系统授权即将到期

    ERR_UPDATE_MC           = 100,  //MC模块升级失败
    ERR_UPDATE_MI           = 101,  //MI模块升级失败
    ERR_UPDATE_PL           = 102,  //PL模块升级失败
    ERR_UPDATE_SC           = 103,  //SC模块升级失败
    ERR_UPDATE_PMC          = 104,  //PMC升级失败
    ERR_UPDATE_SPARTAN      = 105,  //SPARTAN升级失败
    ERR_UPDATE_SCMODBUS     = 106,  //SCModbus模块升级失败

    ERR_IMPORT_PC_DATA      = 107,  //螺补数据导入失败
    ERR_UPDATE_DISK         = 108,  //一键升级失败

    ERR_EN_SYNC_AXIS        = 150,  //同步轴建立同步失败
    ERR_DIS_SYNC_AXIS       = 151,  //同步轴解除同步失败
    ERR_INIT_SYNC_AXIS      = 152,  //同步轴初始化同步失败

    ERR_VOLTAGE_OVER        = 500,  //过压告警
    ERR_VOLTAGE_UNDER       = 501,  //欠压告警
    ERR_BATTERY_VOL_UNDER   = 502,  //电池电压低
    ERR_AXIS_CURRENT_OVER   = 503,  //轴过流告警
    ERR_USB_CURRENT_OVER    = 504,  //USB过流告警

    ERR_RET_REF_FAILED      = 600,  //回参考点失败
    ERR_AXIS_REF_NONE       = 601,  //轴未回参考点
    ERR_RET_REF_NOT_RUN     = 602,  //系统处于错误状态，禁止回零
    ERR_NC_FILE_NOT_EXIST   = 603,  //文件不存在
    ERR_SWITCH_MODE_IN_RET_REF = 604,   //回零中禁止切换工作模式
    ERR_RET_REF_Z_ERR       = 605,  //搜索Z脉冲位置超限，原点建立失败
    ERR_RET_SYNC_ERR        = 606,  //从动轴禁止回零
    ERR_RET_NOT_SUPPORT     = 607,  //不支持的回零方式

	//加工告警	1000~1999
    ERR_EMERGENCY = 1000,               //紧急停止
    ERR_HARDLIMIT_POS = 1001,           //轴正限位告警
    ERR_HARDLIMIT_NEG = 1002,			//轴负限位告警
    ERR_ENCODER = 1003,                 //编码器错误
    ERR_SERVO = 1004,					//伺服告警
    ERR_SYNC_AXIS = 1005,				//同步轴尚未同步
    ERR_SOFTLIMIT_POS = 1006,			//轴正向软限位告警
    ERR_SOFTLIMIT_NEG = 1007,			//轴负向软限位告警
    ERR_TRACK_LIMIT = 1008,             //轴跟随误差过大告警
    ERR_SYNC_POS = 1009,                //同步轴位置指令偏差过大告警
    ERR_INTP_POS = 1010,                //轴位置指令过大告警

    ERR_AXIS_CTRL_MODE_SWITCH = 1011,   //轴控制模式切换超时
    ERR_AXIS_TORQUE_OVERTIME = 1012,    //轴力矩控制超时

    ERR_M_CODE = 1013,                  //不支持的M指令
    ERR_SPD_PRESTART_M05 = 1014,        //主轴预启动期间不能调用M05
    ERR_NO_SPD_Z_AXIS = 1015,		    //系统未指定主轴或Z轴

    ERR_SYNC_TORQUE = 1016,             //同步轴力矩偏差过大告警
    ERR_SYNC_MACH = 1017,               //同步轴机床坐标偏差过大告警

    ERR_SYNC_INVALID_OPT = 1018,        //同步轴非法操作

    ERR_EXEC_FORWARD_PROG = 1100,       //执行前置程序失败
    ERR_EXEC_BACKWARD_PROG = 1101,      //执行后置程序失败

    ERR_NO_CUR_RUN_DATA = 1500,         //找不到当前运行数据

    ERR_TOOL_MEAS_POS = 1600,           //对刀位置未设置，未使用
    ERR_AUTO_TOOL_MEASURE = 1601,       //自动对刀失败，未使用

    ERR_HW_REV_OVER = 1650,             //手轮反向跟踪无更多数据   级别：警告
    ERR_HW_INSERT_INVALID = 1651,       //手轮插入无效
    ERR_HW_INSERT_INFO = 1652,          //提示信息：使用手轮插入后需用坐标系指令清除

    ERR_TOOL_LIFE_OVER = 1660,          //刀具寿命到达
    ERR_TOOL_LIFE_COMING = 1661,        //刀具寿命即将到达     级别：警告

    ERR_REACH_WORK_PIECE = 1662,        //已到达加工件数   级别：警告

    //主轴告警
    ERR_SPD_TAP_START_FAIL  = 1700, //刚性攻丝失败(没有进入位置模式)
    ERR_SPD_RUN_IN_TAP      = 1701, //刚攻状态发送速度指令
    ERR_SPD_TAP_RATIO_FAULT = 1702, //刚性攻丝比例异常
    ERR_SPD_LOCATE_SPEED    = 1703, //定位时主轴转速异常
    ERR_SPD_SW_LEVEL_FAIL   = 1704, //主轴换挡超时
    ERR_SPD_TAP_POS_ERROR   = 1705, //刚攻位置错误
    ERR_SPD_RESET_IN_TAP    = 1706, //刚性攻丝中不能复位
    ERR_SPD_RTNT_INVALID    = 1707, //刚性攻丝回退无效
    ERR_SPD_RTNT_FAIL       = 1708, //刚性攻丝回退失败
    ERR_SPD_RTNT_IN_AUTO    = 1709, //自动模式禁止攻丝回退
    ERR_SPD_MULTI_NUM       = 1710, //多主轴告警
    ERR_SPD_LOCATE_FAIL     = 1711, //主轴定向失败

	//PMC轴告警
    ERR_PMC_AXIS_CTRL_CHANGE    = 1900, //PMC轴控制非法切换
    ERR_PMC_SPEED_ERROR         = 1901, //PMC轴速度异常
    ERR_PMC_IVALID_USED         = 1902, //PMC轴非法使用

#ifdef USES_GRIND_MACHINE
	//玻璃磨边机定制M指令告警
	ERR_WORK_VAC_OPEN = 1950,  //工位吸真空超时
	ERR_WORK_VAC_CLOSE,      //工位破真空超时
	ERR_WORK_CYLINDER_UP,    //工位气缸上升超时
	ERR_WORK_CYLINDER_DOWN,   //工位气缸下降超时
#endif


	//编译告警	2000~3999
    ERR_COMPILER_INTER      = 2000, //编译器内部错误
    ERR_NO_START            = 2001, //无%开头
    ERR_NO_END              = 2002, //无结束命令
    ERR_NC_FORMAT           = 2003, //无法解析
    ERR_MACRO_FORMAT        = 2004, //无法解析的宏指令格式
    ERR_MACRO_OPT           = 2005, //无法解析的宏运算符
    ERR_INVALID_CODE        = 2006, //非法代码，不支持的代码
    ERR_TOO_MANY_G_CODE     = 2007, //单行NC代码中G指令个数超限
    ERR_TOO_MANY_M_CODE     = 2008, //单行NC代码中M指令个数超限
    ERR_TOO_MANY_MACRO_EXP  = 2009, //单行NC代码中宏表达式个数超限
    ERR_BRACKET_NOT_MATCH   = 2010, //括号不匹配
    ERR_INVALID_MACRO_EXP   = 2011, //非法的宏表达式
    ERR_MACRO_OPT_VALUE     = 2012, //非法的宏函数操作数
    ERR_MACRO_EXP_CAL       = 2013, //宏表达式计算错误
    ERR_MACRO_DIV_ZERO      = 2014, //除0错误
    ERR_MACRO_DO_OUT        = 2015, //DO指令识别号超范围
    ERR_MACRO_DO_END_MATCH  = 2016, //DO-END指令不匹配
    ERR_NOT_IN_ONE_LINE     = 2017, //存在不可同行的指令
    ERR_ARC_NO_DATA         = 2018, //圆弧指令缺少参数
    ERR_ARC_INVALID_DATA    = 2019, //圆弧数据错误，无法自洽
    ERR_NO_D_DATA           = 2020, //缺少D值指定
    ERR_NO_H_DATA           = 2021, //缺少H值指定
    ERR_NO_F_DATA           = 2022, //G01、G02、G03缺少F值指定
    ERR_SUB_BACK            = 2023, //子程序返回，恢复现场异常
    ERR_MAIN_OVER           = 2024, //主程序结束异常，子程序调用堆栈非空
    ERR_SUB_END             = 2025, //错误的子程序结束指令，子程序必须以M99结束
    ERR_MDA_OVER            = 2026, //MDA程序结束后，调用堆栈为空
    ERR_NO_SUB_PROG_P       = 2027, //无P字段指定子程序号
    ERR_MCODE_SEPARATED     = 2028, //存在不能同行的M指令     注：部分M指令（需要NC内部处理的M指令）不能与其它M指令同行
    ERR_PRE_SCAN            = 2029, //预扫描执行失败
    ERR_QUIT_PRESCAN        = 2030, //取消预扫描失败
    ERR_SAME_SUB_INDEX      = 2031, //子程序同名
    ERR_SAME_SEQ_NO         = 2032, //重复的顺序号
    ERR_NO_SUB_PROG         = 2033, //找不到指定的子程序
    ERR_JUMP_SUB_PROG       = 2034, //跳转子程序失败
    ERR_SUB_NESTED_COUNT    = 2035, //子程序嵌套层数过多
    ERR_NO_JUMP_LABEL       = 2036, //没有找到跳转点位置
    ERR_JUMP_GOTO           = 2037, //执行GOTO指令，跳转失败
    ERR_JUMP_END            = 2038, //END指令执行失败
    ERR_LACK_OF_PARAM       = 2039, //指令缺少参数
    ERR_JUMP_GOTO_ILLIGAL   = 2040,	//非法跳转
    ERR_NO_P_DATA           = 2041, //缺少P值指定
    ERR_INVALID_L_DATA      = 2042, //非法L值指定
    ERR_TOO_MANY_T_CODE     = 2043, //单行NC代码中T指令个数超限
    ERR_TOO_LONG_WORD       = 2044, //超长词语，无法解析
    ERR_TOO_LONG_DIGIT      = 2045, //超长数值，无法解析
    ERR_G_EXP_NULL          = 2046, //G指令表达式为空值
    ERR_M_EXP_NULL          = 2047, //M指令表达式为空值
    ERR_T_EXP_NULL          = 2048, //T指令表达式为空值
    IF_ELSE_MATCH_FAILED    = 2049, // if else 匹配失败 栈中没有if 却碰到了 else
    COMP_LIST_NOT_INIT      = 2050, // 刀补模块 输出消息队列指针未初始化
    ARC_RADIUS_TOO_SMALL    = 2051, // 圆弧半径过小 无法计算刀补
    ARC_NOT_VALID           = 2052, // 圆弧数据错误
    TOOL_RADIUS_BIGGER_THAN_ARC     = 2053, // 刀补半径大于圆弧半径
    ARC_NOT_ALLOWED         = 2054, // 关闭刀补后第一段移动不能是圆弧
    TOOL_RADIUS_COMP_BUG    = 2055,
    CONCAVE_CORNER_ERROR    = 2056,
    ARC_TO_ARC_SAME_CENTER  = 2057, //圆弧接圆弧刀补不能计算同心圆弧
    MOVE_SMALLER_THAN_CMPRADIUS     = 2058, //移动距离小于刀补半径 无法计算刀补
    ERR_CAL_SPD_TAP_POS     = 2059,   // 无法计算刚性攻丝时的主轴位置
    F_VALUE_ERROR           = 2060,   // F不能指定为负数


	//刀补告警	5000~6999
    ERR_CANCEL_R_COMP_WITHOUT_MOVE      = 5000, //取消刀具半径补偿时没有轴移动
    ERR_NO_MOVE_DATA_EXCEED             = 5001, //刀具半径补偿非加工数据超限
    ERR_C_L_COMP_AXIS_EXCEED            = 5002, //C类刀具长度补偿轴个数超限,针对任意指定轴向进行长度补偿
    ERR_TOOL_RADIUS_EXCEED              = 5003, //刀具半径过大
    ERR_CHANGE_R_COMP_SURFACE           = 5004, //刀具半径补偿过程中不可改变加工平面
    ERR_CHANGE_R_COMP_STAT_CMD          = 5005, //建立或取消刀具半径补偿指令错误
    ERR_END_WITHOUT_CANCEL_L_COMP       = 5006, //程序结束时没有取消刀具长度补偿
    ERR_R_COMP_UNDER_MACHINE_COORD      = 5007, //在机床坐标系下不能使用刀具半径补偿
    ERR_WRONG_MOV_CMD_FOR_L_COMP_CHANGE = 5008, //刀具长度补偿变化时指定插补指令错误
    ERR_R_COMP_NOT_SUPPORTED_FOR_CURVE_TYPE = 5009,	//不支持指定曲线类型的刀具半径补偿
    ERR_R_COMP_CHANGE_AFTER_ESTABLISH   = 5010, //刀具半径补偿不能在建立后立即改变
    ERR_R_COMP_UNDER_FIXED_CYCLE        = 5011, //固定循环中不能使用刀具半径补偿
    ERR_R_COMP_GENERATE_FAIL            = 5012, //刀补轨迹生成失败
	INVALID_COMPENSATE_PLANE 			= 5013,  // G18/G19平面暂不支持刀补
	ARC_NOT_ALLOWED2		 			= 5014,  // 刀补建立第一段不支持圆弧轨迹
	G31_NOT_ALLOWED			 			= 5015,  // 刀补过程中不支持G31跳段
	DWORD_INVALID			 			= 5016,  // 刀补中无法执行D指令
	G41_G42_CHANGE			 			= 5017,  // 不支持 G41_G42 中途切换


	//通讯告警	7000~7999
    ERR_HEARTBEAT_HMI_LOST  = 7000, //HMI心跳丢失
    ERR_HEARTBEAT_MC_LOST   = 7001, //MC模块心跳丢失
    ERR_HEARTBEAT_MI_LOST   = 7002, //MI模块心跳丢失
    ERR_HMI_MONITOR_DISCON  = 7003, //HMI监控tcp连接中断
    ERR_FILE_TRANS = 7004,  //文件传输失败
    ERR_MC_CMD_CRC = 7005,  //MC命令包CRC校验错
    ERR_MI_CMD_CRC = 7006,  //MI命令包CRC校验错


	//MC模块告警 10000~19999
    ERR_SOFT_LIMIT_NEG = 10000, //负向软限位告警
    ERR_SOFT_LIMIT_POS = 10001, //正向软限位告警
    ERR_POS_ERR     = 10002,    //位置指令过大告警
    ERR_ARC_DATA    = 10003,    //圆弧数据错误告警
    ERR_CMD_CRC     = 10004,    //指令校验错
    ERR_DATA_CRC    = 10005,    //数据校验错


	//PMC告警	20000~29999



	//总线告警	30000~39999



	//用户自定义告警   40000~49999

};


//G指令枚举定义
enum GCode{
	G00_CMD = 0,
	G01_CMD = 10,
	G02_CMD = 20,
	G03_CMD = 30,
	G04_CMD = 40,
	G05_1_CMD = 51,
	G06_2_CMD = 62,
	G09_CMD = 90,
	G10_CMD = 100,
	G11_CMD = 110,
	G12_1_CMD = 121,
	G12_2_CMD = 122,
	G12_3_CMD = 123,
	G12_4_CMD = 124,
	G13_1_CMD = 131,
	G15_CMD = 150,
	G16_CMD = 160,
	G17_CMD = 170,
	G18_CMD = 180,
	G19_CMD = 190,
	G20_CMD = 200,
	G21_CMD = 210,
	G25_CMD = 250,
	G26_CMD = 260,
	G27_CMD = 270,
	G28_CMD = 280,
	G29_CMD = 290,
	G30_CMD = 300,
	G31_CMD = 310,
	G37_CMD = 370,         //自动对刀指令
	G40_CMD = 400,
	G41_CMD = 410,
	G42_CMD = 420,
	G43_CMD = 430,
	G43_4_CMD = 434,
	G44_CMD = 440,
	G49_CMD = 490,
	G50_CMD = 500,
	G51_CMD = 510,
	G50_1_CMD = 501,
	G51_1_CMD = 511,
	G52_CMD = 520,
	G53_CMD = 530,
	G54_CMD = 540,
	G55_CMD = 550,
	G56_CMD = 560,
	G57_CMD = 570,
	G58_CMD = 580,
	G59_CMD = 590,
	G60_CMD = 600,
	G61_CMD = 610,
	G62_CMD = 620,
	G63_CMD = 630,
	G64_CMD = 640,
	G65_CMD = 650,
	G68_CMD = 680,
	G69_CMD = 690,
	G73_CMD = 730,
	G74_CMD = 740,
	G76_CMD = 760,
	G80_CMD = 800,
	G81_CMD = 810,
	G82_CMD = 820,
	G83_CMD = 830,
	G84_CMD = 840,
	G84_3_CMD = 843,       //发送刚攻比例值
	G85_CMD = 850,
	G86_CMD = 860,
	G87_CMD = 870,
	G88_CMD = 880,
	G89_CMD = 890,
	G90_CMD = 900,
	G91_CMD = 910,
	G92_CMD = 920,
	G94_CMD = 940,
	G95_CMD = 950,
	G98_CMD = 980,
	G99_CMD = 990,
	G112_CMD = 1120,
	G113_CMD = 1130,
	G120_CMD = 1200,     //G120  告警、提示信息命令
	G200_CMD = 2000,     //G200清整数圈命令

	G1000_CMD = 10000,   //速度控制指令，不等待速度到达，速度单位：mm/min
	G1001_CMD = 10010,   //速度控制指令，不等待速度到达，速度单位：rpm
	G1002_CMD = 10020,   //速度控制指令，等待速度到达，速度单位：mm/min
	G1003_CMD = 10030,   //速度控制指令，等待速度到达，速度单位：rpm

	G2000_CMD = 20000,    // 力矩指令，不用等待结束条件
	G2001_CMD = 20010,    // 力矩指令，结束条件为转速到达0
	G2002_CMD = 20020,    // 力矩指令，结束条件为力矩到达
	G2003_CMD = 20030,    // 力矩指令，结束条件为力矩到达，超时也正常结束，不告警

	G5401_CMD = 54010,  //扩展工件坐标系
	G5402_CMD = 54020,
	G5403_CMD = 54030,
	G5404_CMD = 54040,
	G5405_CMD = 54050,
	G5406_CMD = 54060,
	G5407_CMD = 54070,
	G5408_CMD = 54080,
	G5409_CMD = 54090,
	G5410_CMD = 54100,
	G5411_CMD = 54110,
	G5412_CMD = 54120,
	G5413_CMD = 54130,
	G5414_CMD = 54140,
	G5415_CMD = 54150,
	G5416_CMD = 54160,
	G5417_CMD = 54170,
	G5418_CMD = 54180,
	G5419_CMD = 54190,
	G5420_CMD = 54200,
	G5421_CMD = 54210,
	G5422_CMD = 54220,
	G5423_CMD = 54230,
	G5424_CMD = 54240,
	G5425_CMD = 54250,
	G5426_CMD = 54260,
	G5427_CMD = 54270,
	G5428_CMD = 54280,
	G5429_CMD = 54290,
	G5430_CMD = 54300,
	G5431_CMD = 54310,
	G5432_CMD = 54320,
	G5433_CMD = 54330,
	G5434_CMD = 54340,
	G5435_CMD = 54350,
	G5436_CMD = 54360,
	G5437_CMD = 54370,
	G5438_CMD = 54380,
	G5439_CMD = 54390,
	G5440_CMD = 54400,
	G5441_CMD = 54410,
	G5442_CMD = 54420,
	G5443_CMD = 54430,
	G5444_CMD = 54440,
	G5445_CMD = 54450,
	G5446_CMD = 54460,
	G5447_CMD = 54470,
	G5448_CMD = 54480,
	G5449_CMD = 54490,
	G5450_CMD = 54500,
	G5451_CMD = 54510,
	G5452_CMD = 54520,
	G5453_CMD = 54530,
	G5454_CMD = 54540,
	G5455_CMD = 54550,
	G5456_CMD = 54560,
	G5457_CMD = 54570,
	G5458_CMD = 54580,
	G5459_CMD = 54590,
	G5460_CMD = 54600,
	G5461_CMD = 54610,
	G5462_CMD = 54620,
	G5463_CMD = 54630,
	G5464_CMD = 54640,
	G5465_CMD = 54650,
	G5466_CMD = 54660,
	G5467_CMD = 54670,
	G5468_CMD = 54680,
	G5469_CMD = 54690,
	G5470_CMD = 54700,
	G5471_CMD = 54710,
	G5472_CMD = 54720,
	G5473_CMD = 54730,
	G5474_CMD = 54740,
	G5475_CMD = 54750,
	G5476_CMD = 54760,
	G5477_CMD = 54770,
	G5478_CMD = 54780,
	G5479_CMD = 54790,
	G5480_CMD = 54800,
	G5481_CMD = 54810,
	G5482_CMD = 54820,
	G5483_CMD = 54830,
	G5484_CMD = 54840,
	G5485_CMD = 54850,
	G5486_CMD = 54860,
	G5487_CMD = 54870,
	G5488_CMD = 54880,
	G5489_CMD = 54890,
	G5490_CMD = 54900,
	G5491_CMD = 54910,
	G5492_CMD = 54920,
	G5493_CMD = 54930,
	G5494_CMD = 54940,
	G5495_CMD = 54950,
	G5496_CMD = 54960,
	G5497_CMD = 54970,
	G5498_CMD = 54980,
	G5499_CMD = 54990
};

/**
 * @brief 定义文件传输时的文件类型
 */
enum FileTransType{
	FILE_NO_TYPE = 0x00,        //非法类型
	FILE_G_CODE = 0x01,         //加工文件
	FILE_CONFIG_PACK = 0x02,	//配置打包文件
//	FILE_SYSTEM_CONFIG,         //系统配置文件
//	FILE_CHANNEL_CONFIG,        //通道配置文件
//	FILE_AXIS_CONFIG,           //轴配置文件
//	FILE_TOOL_CONFIG,          //刀具配置文件
	FILE_UPDATE = 0x06,		    //升级文件
	FILE_PMC_LADDER,			//PMC梯形图文件
	FILE_WARN_HISTORY,          //告警历史文件
	FILE_ESB_DEV = 0x10,         //ESB设备描述文件
	FILE_PC_DATA,                //螺补数据文件
	FILE_BACK_DISK,				//一键备份文件
    FILE_BACKUP_CNC,            //全盘备份
};

/**
 * @brief 模块升级类型
 */
enum ModuleUpdateType{
	MODULE_UPDATE_NONE = 0,			//非升级状态
	MODULE_UPDATE_SC,				//SC模块升级
	MODULE_UPDATE_MC,				//MC模块升级
	MODULE_UPDATE_PMC,				//PMC梯形图升级
	MODULE_UPDATE_MI,				//MI模块升级
	MODULE_UPDATE_PL,				//PL模块升级
	MODULE_UPDATE_FPGA,				//FPGA模块升级
	MODULE_UPDATE_MODBUS,            //SCModbus模块升级
	MODULE_UPDATE_DISK,             //一键升级
};

/**
 * @brief 文件操作定义
 */
enum FileOptType{
	FILE_OPT_INVALID = 0X00,   //非法操作
	FILE_OPT_DELETE,           //删除操作
	FILE_OPT_RENAME,           //重命名操作
	FILE_OPT_SAVEAS            //另存为操作
};

/**
 * @brief 打包升级文件Mask定义
 */
enum ConfigMaskBits{
	CONFIG_SYSTEM = 0,	    //系统参数
	CONFIG_CHANNEL = 1,	    //通道参数
	CONFIG_AXIS = 2,		//轴参数
	CONFIG_COORD = 3,	    //工件坐标系
	CONFIG_EX_COORD = 4,	//扩展工件坐标系
	CONFIG_TOOL_INFO = 5,	//刀具信息，包括刀具偏置和刀位信息
	CONFIG_FIVE_AXIS = 6,	//五轴配置
	CONFIG_GRIND = 7,		//磨边机配置
	CONFIG_MACRO_VAR = 8,	//宏变量
	CONFIG_PMC_REG = 9,		//PMC寄存器
	CONFIG_PMC_LDR = 10,	//PMC梯图
	CONFIG_IO_REMAP = 11,   //IO重映射数据
	CONFIG_ESB_FILE = 12,   //伺服配置esb文件

	CONFIG_TYPE_COUNT      //打包文件类型总数
};


/**
 * @brief ErrorInfo数据结构为错误处理模块第输入/输出数据格式
 */
//#pragma pack(1)  //单字节对齐
struct ErrorInfo {
    uint64_t time; 	    //错误产生时间，由高到低分别为：年（2字节）月（1字节）日（1字节）时（1字节）分（1字节）秒（1字节），最高字节保留
    int32_t error_info;     //错误信息
    uint16_t error_code;    //错误码
    uint8_t channel_index;  //通道号
    uint8_t axis_index;     //通道内轴序号
    uint8_t error_level;    //错误级别
    uint8_t clear_type;	    //清除告警类别

    bool operator==(const ErrorInfo &info) const {
        if(info.axis_index == axis_index && info.channel_index == channel_index &&
            info.clear_type == clear_type && info.error_code == error_code &&
            info.error_level == error_level && info.error_info == error_info){
            return true;
        }
        else
            return false;
    }
};
//#pragma pack()

//错误级别
enum ErrorLevel {
    FATAL_LEVEL = 0, 	//严重错误
    ERROR_LEVEL,        //错误
    WARNING_LEVEL,      //警告
    INFO_LEVEL,	        //提示信息
    MAX_LEVEL,
};

/*
 * @brief 告警清除类别
 */
enum ErrorClearType {
    CLEAR_BY_RESET_POWER = 0, 	//重启电源后清除
    CLEAR_BY_MCP_RESET,         //复位清除
    CLEAR_BY_AUTO,              //自动清除
};

//32bit的mask
struct Mask32 {
	uint32_t mask;
	void Init() {
		mask = 0;
	}
	void InitF() {
		mask = (uint32_t) -1;
	}
	bool CheckMask(int i) const {
		if(mask & ((uint32_t)0x01 << i))
			return true;

		return false;
	}
	void SetMask(int i) {	//标志位置位
		mask |= (uint32_t)0x01 << i;
	}
	void ResetMask(int i){  //标志位清零
		mask &= ~((uint32_t)0x01 << i);
	}
	void SetMask(int i, int value) {
		if (value == 0) {
			mask &= ~((uint32_t)0x01 << i);

		} else {
			SetMask(i);
		}
	}
	bool IsZero() {
		return (mask) ? false : true;
	}
	bool IsAllF() {
		return (mask == (uint32_t) -1 ) ?
				true : false;
	}
};

/*
 * @brief HMI使用的通道实时状态
 */
struct HmiChannelRealtimeStatus {
	double cur_pos_machine[kMaxAxisChn];	//当前实际位置，机械坐标系，单位：mm
	double cur_pos_work[kMaxAxisChn];		//当前实际位置，工件坐标系，单位：mm
	double tar_pos_work[kMaxAxisChn];		//当前目标位置，工件坐标系，单位：mm
	int32_t spindle_cur_speed; 		        //主轴当前转速，单位：转/分钟
    int32_t cur_feed; 						//当前进给速度，单位：um/s
	uint32_t machining_time; 				//加工时间，单位：秒
	uint32_t machining_time_remains; 		//剩余加工时间，单位：秒
    uint32_t machining_time_total;          //累计加工时间，单位：秒
	uint32_t line_no;						//当前行号

    double cur_feedbck_velocity[kMaxAxisChn];			//当前各轴实际速度，单位：mm/min
    double cur_feedbck_torque[kMaxAxisChn];			//当前各轴实际力矩，单位：0.001额定力矩

    int32_t tap_err;                        //刚性攻丝误差(最大值) 单位：um
    int32_t tap_err_now;                    //刚性攻丝误差(当前值) 单位：um
    double spd_angle;                      //主轴当前角度 单位：度
};


/*
 * @brief HMI使用的通道状态
 */
struct HmiChannelStatus{
	double rated_manual_speed; 		//手动速度，单位：mm/s
	uint32_t workpiece_count;       //工件计数，最大99999999
    uint32_t workpiece_require;     //需求件数
    uint32_t workpiece_count_total; //总共件数
    uint32_t machinetime_total;         //累计加工时间
    Mask32 func_state_flags;        //系统功能状态标志，包括单段、跳段、选停、空运行、手轮跟踪、机床锁，辅助锁
	int32_t rated_spindle_speed;    //用户设定主轴转速，单位：转/分钟
	uint32_t rated_feed; 					//用户设定进给速度，单位：um/s
	uint16_t g_code_mode[kMaxGModeCount];	//G指令模态信息
	uint8_t chn_work_mode;			//通道工作模式, 自动、MDA、手动、手轮
	uint8_t machining_state;       	//加工状态
	uint8_t cur_tool;               //当前刀号，从1开始编号，0号刀为主轴刀号
	uint8_t preselect_tool_no;		//预选刀刀号
	uint8_t cur_h_code;				//当前H补偿号
	uint8_t cur_d_code;				//当前D补偿号
	uint8_t auto_ratio;            	//自动进给倍率
	uint8_t spindle_ratio;         	//主轴倍率
	uint8_t rapid_ratio;			//快速进给倍率
	uint8_t manual_ratio;          	//手动倍率
	uint8_t manual_step;           	//手动步长，0:INVALID;1:X1;2:X10;3:X100;4:X1000
	int8_t cur_manual_move_dir;		//当前手动移动方向，0--停止   1--正向   -1--负向
	uint8_t cur_axis;				//当前轴号,通道轴号
	uint8_t spindle_mode;           //主轴工作方式，0：速度控制方式；1：位置控制方式
	int8_t spindle_dir;				//主轴方向，-1：反转；0：停转；1：正转
	uint8_t returned_to_ref_point;	//是否已回参考点，bit0-bit7分别表示轴0-7是否已回参考点
	uint8_t es_button_on;			//急停按键是否处于按下状态，每一个bit表示一个急停按键
	char cur_nc_file_name[kMaxFileNameLen];	//当前加工主程序文件名
	uint8_t cur_chn_axis_phy[kMaxAxisChn];	    //当前通道内轴对应的物理轴索引
	int mcode; // @add zk 暂时没用
};

/**
 * @brief 定义通道状态类型
 */
enum ChannelStatusType{
	FUNC_STATE = 0,			//系统功能状态标志，包括单段、跳段、选停、空运行、手轮跟踪、机床锁，辅助锁
	AUTO_RATIO,				//自动倍率
	SPINDLE_RATIO,			//主轴倍率
	RAPID_RATIO,			//快速进给倍率
	MANUAL_RATIO,			//手动倍率
	G_MODE,					//G代码模式
	MANUAL_STEP,			//手动步长
	REF_POINT_FLAG,			//回参考点标志
	CUR_AXIS,				//通道当前轴

	CHN_STATUS_GUARD		//卫兵

};

/*
 * @brief PLC数据
 */
struct PmcData {
	uint8_t address;	//段地址
	uint16_t index;		//段内地址号
	uint16_t data;		//值
};

/**
 * @brief CPU运行情况信息
 */
struct CPURunningInfo{
	double board_temperature;	//主板温度
	uint8_t arm1_cpu_load;		//ARM1 CPU占用率
	uint8_t arm1_max_cpu_load;	//ARM1 CPU占用率（系统启动后最大值）
	uint8_t arm1_mem_load;		//ARM1 内存占用率
	uint8_t arm1_max_mem_load;	//ARM1 内存占用率（系统启动后最大值）
	uint8_t dsp_cpu_load;		//DSP CPU占用率
	uint8_t dsp_max_cpu_load;	//DSP CPU占用率（系统启动后最大值）

	uint8_t arm2_cpu_load;		//ARM2 CPU占用率
	uint8_t arm2_max_cpu_load;	//ARM2 CPU占用率（系统启动后最大值）
	uint8_t arm2_mem_load;		//ARM2 内存占用率
	uint8_t arm2_max_mem_load;	//ARM2 内存占用率（系统启动后最大值）

	uint16_t plc_fst_stage_run_time;		//PLC一级运行时间
	uint16_t plc_max_fst_stage_run_time;	//PLC一级运行时间（系统启动后最大值）
	uint16_t plc_snd_stage_run_time;		//PLC二级运行时间
	uint16_t plc_max_snd_stage_run_time;	//PLC二级运行时间（系统启动后最大值）
	uint16_t plc_scan_interval;	//PLC扫描周期
};

//版本信息集合
struct SwVerInfoCollect{
	char platform[kMaxVersionLen];   	//平台版本
	char sc[kMaxVersionLen];			//SC模块
	char mc[kMaxVersionLen];			//MC模块
	char mi[kMaxVersionLen];			//MI模块
	char mop[kMaxVersionLen];			//操作面板
	char keyboard[kMaxVersionLen];		//键盘
	char fpga_pl[kMaxVersionLen];		//ZYNQ_PL
	char fpga_spartan[kMaxVersionLen];	//SPARTAN
};


//系统信息结构体
struct SystemInfo{
	SwVerInfoCollect sw_version_info;    //版本信息
	CPURunningInfo cpu_info;		//CPU运行信息
};

/**
 * @brief 坐标轴位置结构
 */
struct CoordAxisPos{
	double x;		//X轴
	double y;		//Y轴
	double z;		//Z轴
};

/**
 * @brief PMC轴位置结构
 */
struct PmcAxisPos{
	int axis_no;		//物理轴号
	double mach_pos;	//机械坐标，单位：mm
	double work_pos;   //工件坐标，单位: mm
	double remain_dis;	//剩余移动量，单位：mm
};


/**
 * @brief HMI模块系统配置数据
 */
struct HmiSystemConfig{
	uint8_t cnc_mode;			        //设备类型		//由授权码给定，不可修改
	uint8_t max_chn_count;				//最大通道数	//由授权码给定，不可修改
	uint8_t max_axis_count;				//最大轴数		//由授权码给定，不可修改
	uint8_t chn_count;					//通道数量
	uint8_t axis_count;			     	//系统物理轴数量
	uint8_t axis_name_ex;				//是否允许轴名称扩展下标    0--关闭		1--打开
	uint8_t fix_ratio_find_ref;			//回参考点时倍率固定	0--关闭		1--打开
//	uint8_t pc_type;					//螺距补偿方式		0--单向螺补  1--双向螺补
//	uint8_t mcp_count;                  //MCP数量

	uint8_t hw_code_type;               //手轮编码方式     0--二进制编码    1--格雷码

	uint32_t io_filter_time;			//普通IO滤波时间	//单位：微秒
	uint32_t fast_io_filter_time;		//快速IO滤波时间	//单位：微秒
	uint32_t free_space_limit;			//剩余空间限制      //单位：KB
	uint16_t backlight_delay_time;		//背光延时时间		//单位：秒

	uint8_t bus_cycle;					//总线通讯周期		0-8:125us/250us/500us/1ms/2ms/4ms/5ms/8ms/10ms

	uint8_t save_lineno_poweroff;		//掉电保存当前运行行号		0--关闭		1--打开
	uint8_t manual_ret_ref_mode;		//手动回参考点模式		0-首次 		1--每次

	uint8_t beep_time;					//蜂鸣时间			//单位：秒

	uint8_t da_ocp;						//DA过流保护	0--关闭		1--打开
	uint8_t da_prec;				    //DA器件精度（位数）  10-64

	uint8_t alarm_temperature;			//主板告警温度   单位：摄氏度
	uint8_t trace_level;				//调试跟踪信息记录级别
	uint8_t debug_mode;					//调试模式		0--关闭    1--模式1    2--模式2    3--模式3

	uint8_t hw_rev_trace;               //手轮反向引导功能   0--关闭   1--打开

    /**********************位置开关***************************/
    uint8_t pos_check_id_1;             //位置开关1
    uint8_t pos_check_id_2;             //位置开关2
    uint8_t pos_check_id_3;             //位置开关3
    uint8_t pos_check_id_4;             //位置开关4
    uint8_t pos_check_id_5;             //位置开关5
    uint8_t pos_check_id_6;             //位置开关6
    uint8_t pos_check_id_7;             //位置开关7
    uint8_t pos_check_id_8;             //位置开关8

    double pos_check_min_1;             //位置开关最小值1
    double pos_check_min_2;             //位置开关最小值2
    double pos_check_min_3;             //位置开关最小值3
    double pos_check_min_4;             //位置开关最小值4
    double pos_check_min_5;             //位置开关最小值5
    double pos_check_min_6;             //位置开关最小值6
    double pos_check_min_7;             //位置开关最小值7
    double pos_check_min_8;             //位置开关最小值8

    double pos_check_max_1;             //位置开关最大值1
    double pos_check_max_2;             //位置开关最大值2
    double pos_check_max_3;             //位置开关最大值3
    double pos_check_max_4;             //位置开关最大值4
    double pos_check_max_5;             //位置开关最大值5
    double pos_check_max_6;             //位置开关最大值6
    double pos_check_max_7;             //位置开关最大值7
    double pos_check_max_8;             //位置开关最大值8
    /**********************位置开关***************************/
};

/**
 * @brief HMI模块通道配置数据
 */
struct HmiChnConfig{
	uint8_t chn_index;  		//通道索引号
	uint8_t chn_axis_count;     //通道轴数量
	uint8_t chn_group_index;    //通道所属方式组编号
//	uint8_t chn_axis_x;			//基本轴X对应的物理轴索引
//	uint8_t chn_axis_y;			//基本轴Y对应的物理轴索引
//	uint8_t chn_axis_z;			//基本轴Z对应的物理轴索引
	uint8_t chn_axis_phy[kMaxAxisChn];	//其它轴对应的物理轴索引

	uint8_t chn_axis_name[kMaxAxisChn];	//轴名称
	uint8_t chn_axis_name_ex[kMaxAxisChn];	//轴名称下标

	uint8_t intep_mode;				//插补模式   0：XYZ三轴插补  1：所有轴插补
	uint8_t intep_cycle;			//插补周期	0-4:125us/250us/400us/500us/1000us
	uint16_t chn_precision;			//加工精度，单位：微米
	uint8_t chn_look_ahead;			//前瞻功能 	0--关闭   1--打开
	uint8_t chn_feed_limit_level;	//加工速度调整等级  0--关闭   1~10表示等级1~等级10
	uint8_t zmove_stop;				//下刀准停  0--关闭    1--打开
	uint8_t corner_stop_enable;		//拐角准停功能开关
	uint8_t corner_stop;			//拐角准停角度  0-180度
	uint8_t corner_stop_angle_min;  //拐角准停下限角度 0-180度
	uint8_t corner_acc_limit;		//拐角加速度限制	0--关闭   1--打开
	uint8_t long_line_acc;			//长直线运行加速    0--关闭   1--打开

	uint8_t arc_err_limit;			//圆弧误差限制      0-255  单位：um

	uint8_t chn_spd_limit_on_axis;  	//基于轴速度差的转角速度钳制		0--关闭   1--打开
	uint8_t chn_spd_limit_on_acc;		//基于加速度的小线段进给速度钳制		0--关闭   1--打开
	uint8_t chn_spd_limit_on_curvity;	//基于曲率及向心加速的小线段进给速度钳制		0--关闭   1--打开
	uint8_t chn_rapid_overlap_level;	//G00重叠等级	0--关闭  1~3--等级1~等级3

	double chn_max_vel;					//通道允许最大速度	//单位：mm/min
	double chn_max_acc;					//通道最大加速度	//单位：mm/s^2
	double chn_max_dec;					//通道最大减速度	//单位：mm/s^2
	double chn_max_corner_acc;          //最大拐角加速度		//单位：mm/s^2
	double chn_max_arc_acc;				//最大向心加速度	//单位：mm/s^2
	uint16_t chn_s_cut_filter_time;		//切削进给S型规划时间常数  //单位：ms

    double tap_max_acc;                 //刚性攻丝最大加速度	//单位：mm/s^2
    double tap_max_dec;                 //刚性攻丝最大减速度	//单位：mm/s^2
    int8_t tap_plan_mode;              //刚性攻丝规划模式  0：T型  1：S型
    uint16_t tap_s_cut_filter_time;     //刚性攻丝S型规划时间常数  //单位：ms

    uint8_t default_plane;			//默认平面	   0：G17  1：G18  2：G19
    uint8_t default_cmd_mode;		//默认指令模式   0：G90  1：G91
    uint8_t default_feed_mode;      //默认进给方式   0：G00  1：G01  2：G02  3：G03

	uint8_t rapid_mode;				//G00模式    0：定位     1：直线定位
	uint8_t cut_plan_mode;			//切削规划模式	0：T型   1：S型
	uint8_t rapid_plan_mode;		//定位规划模式  0：T型   1：S型
	uint8_t ex_coord_count;			//扩展工件坐标系个数  0-99

	uint8_t change_tool_mode;		//换刀方式     0--手动    1--自动
	uint8_t tool_live_check;		//刀具寿命检测  0--关闭   1--打开
	uint8_t auto_tool_measure;		//自动对刀		0--关闭   1--打开

	uint8_t gcode_trace;			//加工代码跟踪   0--关闭   1--打开
	uint8_t gcode_unit;				//G代码编程单位		0--公制    1--英制

	uint8_t timing_mode;			//加工计时方式    0--全部计时    1--切削计时

	uint16_t chn_small_line_time;   //小线段执行时间常数   单位：0.1ms   范围[0, 10000]

	uint32_t g31_skip_signal;       //G31跳转信号，例如X1004.7保存为10047，即乘10倍
	uint8_t g31_sig_level;          //G31跳转信号有效电平    0--低电平    1--高电平
    uint16_t rst_hold_time;         //复位时间 单位:ms
    uint8_t rst_mode;               //复位时运行数据是否保留  0:不保留 1：保留
    uint32_t g01_max_speed;         //G01最高进给速度 单位：mm/min
    uint16_t mpg_level3_step;       //手轮3档的自定义步长 单位：um
    uint16_t mpg_level4_step;       //手轮4档的自定义步长 单位：um

    double G73back;		// G73回退距离
    double G83back;		// G83回退距离

#ifdef USES_WOOD_MACHINE
	int debug_param_1;             //调试参数1
	int debug_param_2;             //调试参数2
	int debug_param_3;             //调试参数3
	int debug_param_4;             //调试参数4
	int debug_param_5;             //调试参数5
	
	int flip_comp_value;           //挑角补偿    -10000~10000    单位：um
#endif
};


/**
 * @brief HMI模块轴配置数据
 */
struct HmiAxisConfig{
    uint8_t axis_index;						//轴索引号	0-11，系统自动生成
    uint8_t axis_type;						//轴类型,    0--直线轴     1--旋转轴     2--主轴/速度轴    3--力矩轴
    uint8_t axis_interface;					//轴接口类型	0--虚拟轴    1-总线轴    2--非总线轴
    uint8_t axis_port;						//从站号（总线轴）/对应轴口号（非总线轴）1~12
    uint8_t axis_linear_type;				//直线轴类型
    uint8_t axis_pmc;						//是否PMC轴


    double kp1;								//自动比例参数
    double kp2;								//手动比例参数
    double ki;								//积分参数
    double kd;								//微分参数
    double kil;								//积分饱和限
    uint16_t kvff;							//速度前馈系数
    uint16_t kaff;							//加速度前馈系数

    uint32_t track_err_limit;				//跟随误差限制 单位：um
    uint16_t location_err_limit;			//定位误差限制 单位：um

//	uint8_t soft_limit_check;				//软限位检测	0--关闭   1--打开
    uint8_t save_pos_poweroff;				//掉电坐标记忆		0--关闭   1--打开
//	uint8_t axis_alarm_level;               //轴告警信号电平    0--低电平  1--高电平

	uint32_t motor_count_pr;				//电机每转计数
	uint32_t motor_speed_max;				//电机最大转速   单位：rpm
	double move_pr;						    //每转移动量，即丝杆螺距    单位：mm(deg)
	uint8_t motor_dir;						//电机旋转方向    0--正转    1--反转
	uint8_t feedback_mode;					//电机反馈类型    0--增量式    1--绝对式(安川)     2--绝对式(松下)    3--光栅尺        4--无反馈

    uint8_t ret_ref_mode;					//0--无挡块回零  1--有挡块回零
    uint8_t absolute_ref_mode;              //绝对式电机回零方式 0--回零标记点设定方式    1--无挡块回零方式
    uint8_t ret_ref_dir;					//回参考点方向		0--负向    1--正向
	uint8_t ret_ref_change_dir;				//回参考点换向      0--反向    1--同向
	uint8_t ref_signal;						//参考点信号类型    0--零信号    1--Z信号
	uint8_t ret_ref_index;					//回参考点顺序		0-11:第一~第十二
	uint8_t ref_base_diff_check;			//粗精基准位置偏差检测		0--关闭   1--打开
	double ref_base_diff;					//粗精基准位置偏差		单位：mm（deg）此值为检测值，不可更改
	int64_t ref_encoder;					//零点编码器值，绝对式编码器有效
    double ref_offset_pos;                  //回参考点后的偏移量 单位:mm
    double ref_z_distance_max;              //搜索Z脉冲最大移动距离

	double manual_speed;					//手动进给速度，单位:mm/min
	double rapid_speed;						//定位速度，单位:mm/min
	double reset_speed;						//复位速度，单位：mm/min
	double ret_ref_speed;					//回参考点速度，单位：mm/min
    double ret_ref_speed_second;            //回参考点低速度，单位：mm/min

    double rapid_acc;   					//定位加速度，单位:mm/s^2
    double manual_acc;						//手动加速度，单位:mm/s^2
    double start_acc;						//起步加速度，单位:mm/s^2

    double corner_acc_limit;				//拐角速度差限制，单位：mm/min

    uint16_t rapid_s_plan_filter_time;			//G00 S型速度规划时间常数

    uint8_t post_filter_type;		    //插补后滤波器类型		0--关闭    1--平滑滤波     2--均值滤波     3--S型均值滤波器
    uint16_t post_filter_time_1;		//插补后滤波器一级时间常数  单位：ms
    uint16_t post_filter_time_2;        //插补后滤波器二级时间常数  单位：ms

    uint8_t backlash_enable;					//反向间隙是否生效
    //int16_t backlash_forward;					//正向反向间隙，单位：um(deg)
    //int16_t backlash_negative;					//负向反向间隙，单位：um(deg)
    double backlash_forward;					//正向反向间隙，单位：mm(deg)
    double backlash_negative;                   //负向反向间隙，单位：mm(deg)
    int16_t backlash_step;                      //反向间隙步长，单位：um(20-300)


    uint8_t  pc_type;							//螺补类型  0 单向螺补  1 双向螺补
    uint8_t  pc_enable;							//螺补是否生效
    uint16_t pc_offset;                         //螺补起始点   1-10000
    uint16_t pc_count;							//螺距误差补偿点数  0-10000
    uint16_t pc_ref_index;						//参考点补偿位置 1-10000
    double pc_inter_dist;						//螺距补偿间隔，单位：mm(deg)

    double soft_limit_max_1;					//正向软限位1
    double soft_limit_min_1;					//负向软限位1
    uint8_t soft_limit_check_1;					//软限位1有效
    double soft_limit_max_2;					//正向软限位2
    double soft_limit_min_2;					//负向软限位2
    uint8_t soft_limit_check_2;					//软限位2有效
    double soft_limit_max_3;					//正向软限位3
    double soft_limit_min_3;					//负向软限位3
    uint8_t soft_limit_check_3;					//软限位3有效



    uint8_t off_line_check;						//轴断线检测       0--关闭   1--打开

    uint8_t ctrl_mode;							//轴控制方式	   0--脉冲方向    1--正交脉冲    2--CW/CCW    3--模拟量
    uint32_t pulse_count_pr;					//每转输出脉冲数
    uint8_t encoder_lines;						//编码器单圈线数   10~31
    uint16_t encoder_max_cycle;					//编码器整圈最大值


    //主轴相关参数
    int16_t zero_compensation;					//零漂补偿(DA电平值) 0~4095
    uint8_t spd_gear_ratio;						//主轴变速比
    uint32_t spd_max_speed;						//主轴最高转速	单位：rpm
    uint16_t spd_start_time;					//主轴启动时间  单位：0.1s
    uint16_t spd_stop_time;						//主轴制动时间  单位：0.1s
    uint8_t spd_vctrl_mode;						//主轴模拟电压控制方式    0--禁止    1 -- 0V~10V     2-- -10V~10V
//	uint8_t spd_set_dir;						//主轴设置方向    0--反转    1--正转  //使用电机旋转方向替代
    uint32_t spd_set_speed;						//主轴设置转速  单位：rpm
    uint8_t spd_speed_check;					//主轴转速检测   0--关闭   1--打开
    uint8_t spd_speed_match;					//转速匹配检测   0--关闭   1--打开
    uint8_t spd_speed_diff_limit;				//转速误差允许比例   0~100
    uint8_t spd_encoder_location;				//编码器位置     0--电机端     1--主轴端

    uint8_t spd_ctrl_GST;                       //SOR信号用途  0：主轴定向  1：齿轮换挡
    uint8_t spd_ctrl_SGB;                       //主轴换挡方式 0：A方式  1：B方式
    uint8_t spd_ctrl_SFA;                       //齿轮换挡时是否输出SF信号 0：输出  1：不输出
    uint8_t spd_ctrl_ORM;                       //主轴定向时的转向 0：正  1：负
    uint8_t spd_ctrl_TCW;                       //主轴转向是否受M03/M04影响 0：不受影响 1：受影响
    uint8_t spd_ctrl_CWM;                       //主轴转向取反 0：否  1：是
    uint8_t spd_ctrl_TSO;                       //螺纹切削和刚性攻丝时，主轴倍率设置  0：强制100%  1：有效
    uint16_t spd_analog_gain;                   //模拟输出增益 700-1250 单位：0.1%
    uint16_t spd_sor_speed;                     //主轴齿轮换档/定向时的主轴转速 rpm
    uint16_t spd_motor_min_speed;               //主轴电机最小钳制速度
    uint16_t spd_motor_max_speed;               //主轴电机最大钳制速度
    uint16_t spd_gear_speed_low;                //齿轮低档位最高转速 rpm
    uint16_t spd_gear_speed_middle;             //齿轮中档位最高转速 rpm
    uint16_t spd_gear_speed_high;               //齿轮高档位最高转速 rpm
    uint16_t spd_gear_switch_speed1;            //B方式档1->档2电机转速
    uint16_t spd_gear_switch_speed2;            //B方式档2->档3电机转速
    double spd_sync_error_gain;               //同步误差增益  范围为0~1000，默认为200
    double spd_speed_feed_gain;               //轴速度前馈增益 范围为0~100000，默认为60000
    double spd_pos_ratio_gain;                //轴位置比例增益 范围0~200000，默认为100000
    uint8_t spd_rtnt_rate_on;                   //攻丝回退期间，倍率是否有效 0：强制100%  1：有效
    uint8_t spd_rtnt_rate;                      //攻丝回退倍率 单位：1%
    int32_t spd_rtnt_distance;                 //攻丝回退的额外回退值 单位：um

    double spd_locate_ang;                     //主轴定向角度 单位：度

    //旋转轴相关参数
    uint8_t fast_locate;						//快速定位    0--关闭   1--打开
    uint8_t pos_disp_mode;						//位置显示模式   0--循环模式（0~360）    1--非循环模式
    uint8_t pos_work_disp_mode;                 //工件坐标是否循环显示  0--否  1--是
    uint8_t pos_rel_disp_mode;                  //相对坐标是否循环显示  0--否  1--是
    uint8_t rot_abs_dir;                        //旋转轴绝对指令的旋转方向  0--快捷方向  1--取决于指令符号

    //同步轴相关参数
    uint8_t sync_axis;							//是否同步轴  0--否   1--是
    uint8_t series_ctrl_axis;                   //是否串联控制 0--否   1--同向   2--反向
    uint8_t master_axis_no;						//主动轴号
    uint8_t disp_coord;							//是否显示坐标  0--否   1--是
    uint8_t auto_sync;							//从动轴回参考点后自动同步校准   0--否   1--是
    uint32_t sync_err_max_pos;					//位置同步误差报警阈值	 单位：um
    double benchmark_offset;					//主从轴基准位置偏差  单位：mm
    uint8_t sync_pre_load_torque;               //预载电流偏置 单位：1%
    uint8_t sync_err_max_torque;                //扭矩同步误差报警阈值 单位：1%
    uint32_t sync_err_max_mach;                 //坐标同步误差报警阈值 单位：um
    uint8_t sync_pos_detect;                    //是否进行位置同步误差检测 0--否   1：是
    uint8_t sync_mach_detect;                   //是否进行坐标同步误差检测 0--否   1：是
    uint8_t sync_torque_detect;                 //是否进行扭矩同步误差检测 0--否   1：是
    uint16_t serial_torque_ratio;               //串联力矩系数 单位: 1%
    uint16_t serial_pre_speed;                  //预载串联速度 单位：rpm

    double axis_home_pos[10];				//参考点位置  单位：mm
    double ref_mark_err;                   //参考点基准误差   单位：mm    有效范围：0~10.0
    uint32_t spd_min_speed;                 //主轴最低转速   单位：rpm   0~100000

    uint8_t pmc_g00_by_EIFg;                //PMC倍率控制
    uint16_t pmc_min_speed;                 //最小PMC移动速度
    uint16_t pmc_max_speed;                 //最大PMC移动速度

    int    ref_complete;                    //参考点是否建立
    bool init_backlash_dir;                 //螺补初始化方向

    uint8_t decelerate_numerator;           //减速比例分子
    uint8_t decelerate_denominator;         //减速比例分母
};

/**
 * @brief HMI模块刀具偏置配置数据
 */
struct HmiToolOffsetConfig{
	double geometry_compensation[3];    //刀具几何偏置，包括长度补偿
	double geometry_wear;				//刀具长度磨损补偿
	double radius_compensation;			//刀具半径补偿
	double radius_wear;					//刀具半径磨损补偿
};

/**
 * @brief HMI模块刀位配置数据，刀号与刀套映射表
 */
struct HmiToolPotConfig{
//	uint8_t tool_index;		//刀号
	uint8_t tool_type[kMaxToolCount];		//刀具类型
	uint8_t huge_tool_flag[kMaxToolCount];           //是否大刀柄，大刀柄意味着一把刀需要占三个刀位
	uint8_t tool_pot_index[kMaxToolCount];			//刀套号
	int tool_life_max[kMaxToolCount];								//刀具寿命，单位：min
	int tool_life_cur[kMaxToolCount];								//已使用寿命，单位：min
	int tool_threshold[kMaxToolCount];                             //刀具寿命预警阈值
	uint8_t tool_life_type[kMaxToolCount];                          //刀具计寿类型，0--换刀次数，1--切削时间，2--切削距离
};

/**
 * @brief 磨削专用参数
 */
struct GrindConfig{
	double wheel_radius;				//砂轮半径
	double safe_pos_x;					//X轴安全位置坐标
	double safe_pos_c;					//C轴安全位置坐标
	double shock_z_max_dis;				//砂轮震荡磨削Z轴最大长度
	double shock_x_compen_ratio;		//砂轮震荡磨削X轴补偿比例
	double shock_speed;					//砂轮震荡磨削速度
	uint8_t shock_manual_ratio_ctrl;	//震荡速度受手动倍率控制
	uint8_t shock_init_dir;				//砂轮震荡磨削初始方向
	uint8_t grind_dir;					//切削加工方向
	double corner_vec_speed;			//转角向量角速度
	uint8_t grind_smooth_fun;			//磨削平滑功能
	uint16_t grind_smooth_time;			//磨削平滑时间常数
	double wheel_raidus_multi[6];		//备用砂轮半径
};

/**
 * @brief 五轴配置参数
 */
struct FiveAxisConfig{
	uint8_t five_axis_type;				//五轴联动机床类型
	double x_offset_1;					//第一转轴中心相对刀尖点X坐标
	double y_offset_1;					//第一转轴中心相对刀尖点Y坐标
	double z_offset_1;					//第一转轴中心相对刀尖点Z坐标
	uint8_t post_dir_1;					//第一转轴旋转正方向  0--逆时针    1--顺时针
	uint8_t range_limit_1;				//第一转轴旋转受限  0--受限    1--不受限
	double x_offset_2;					//第二转轴中心相对刀尖点X坐标
	double y_offset_2;					//第二转轴中心相对刀尖点Y坐标
	double z_offset_2;					//第二转轴中心相对刀尖点Z坐标
	uint8_t post_dir_2;					//第二转轴旋转正方向
	uint8_t range_limit_2;				//第二转轴旋转受限
	double rot_swing_arm_len;			//刀具旋转摆臂长度
	uint8_t five_axis_coord;			//五轴编程坐标系
	double speed_limit_x;				//五轴联动X轴速度限制
	double speed_limit_y;				//五轴联动Y轴速度限制
	double speed_limit_z;				//五轴联动Z轴速度限制
	double speed_limit_a;				//五轴联动A轴速度限制
	double speed_limit_b;				//五轴联动B轴速度限制
	double speed_limit_c;				//五轴联动C轴速度限制
	double intp_angle_step;				//五轴联动粗插补角度步距
	double intp_len_step;				//五轴联动粗插补最小长度
	uint8_t five_axis_speed_plan;		//五轴联动速度规划
	uint8_t five_axis_plan_mode;		//五轴速度规划方式

};

/**
 * @brief 新五轴配置参数
 */
struct HmiFiveAxisV2Config
{
    uint8_t machine_type;               //五轴结构类型 0:非五轴  1:双摆头  2:摇篮  3:混合
    uint8_t pivot_master_axis;          //第一旋转轴 0:X  1:Y  2:Z
    uint8_t pivot_slave_axis;           //第二旋转轴 0:X  1:Y  2:Z
    double table_x_position;            //旋转台x坐标 单位:mm
    double table_y_position;            //旋转台y坐标 单位:mm
    double table_z_position;            //旋转台z坐标 单位:mm
    double table_x_offset;              //第二旋转轴x偏移 单位:mm
    double table_y_offset;              //第二旋转轴y偏移 单位:mm
    double table_z_offset;              //第二旋转轴z偏移 单位:mm
    uint8_t tool_dir;                   //刀具轴向 0:X  1:Y  2:Z
    double tool_holder_offset_x;        //刀柄x偏移 单位:mm
    double tool_holder_offset_y;        //刀柄y偏移 单位:mm
    double tool_holder_offset_z;        //刀柄z偏移 单位:mm
    uint8_t table_master_dir;           //第一旋转轴方向  0:正  1:负
    uint8_t table_slave_dir;            //第二旋转轴  0:正  1:负
    double master_ref_angle_crc;        //第一旋转轴初始角度 单位:度
    double slave_ref_angle_crc;         //第二旋转轴初始角度 单位:度
    double tool_holder_length;          //刀柄长度 单位:mm
};

/**
 * @brief 工艺相关通道参数
 */
struct ProcessParamChn{
	uint8_t rapid_mode;				//G00模式    0：定位     1：直线定位
	uint8_t cut_plan_mode;			//切削规划模式	0：T型   1：S型
	uint8_t rapid_plan_mode;		//定位规划模式  0：T型   1：S型
	uint8_t corner_stop_enable;		//拐角准停功能开关
	uint8_t corner_stop;			//拐角准停角度  0-180度
	uint8_t corner_stop_angle_min;  //拐角准停下限角度 0-180度
	uint8_t corner_acc_limit;		//拐角加速度限制	0--关闭   1--打开
	uint8_t chn_spd_limit_on_axis;  	//基于轴速度差的转角速度钳制		0--关闭   1--打开
	uint8_t chn_spd_limit_on_acc;		//基于加速度的小线段进给速度钳制		0--关闭   1--打开
	uint8_t chn_spd_limit_on_curvity;	//基于曲率及向心加速的小线段进给速度钳制		0--关闭   1--打开
	uint8_t chn_rapid_overlap_level;	//G00重叠等级	0--关闭  1~3--等级1~等级3
	uint16_t chn_s_cut_filter_time;		//切削进给S型规划时间常数  //单位：ms
	uint16_t chn_small_line_time;   //小线段执行时间常数   单位：0.1ms   范围[0, 10000]
	double chn_max_vel;					//通道允许最大速度	//单位：mm/min
	double chn_max_acc;					//通道最大加速度	//单位：mm/s^2
	double chn_max_dec;					//通道最大减速度	//单位：mm/s^2
	double chn_max_corner_acc;          //最大拐角加速度		//单位：mm/s^2
	double chn_max_arc_acc;				//最大向心加速度	//单位：mm/s^2
#ifdef USES_WOOD_MACHINE
	int flip_comp_value;            //挑角补偿    -10000~10000    单位：um
#endif
};

/**
 * @brief 工艺相关轴参数
 */
struct ProcessParamAxis{
	double rapid_acc;   					//定位加速度，单位:mm/s^2
	double manual_acc;						//手动加速度，单位:mm/s^2
	double start_acc;						//起步加速度，单位:mm/s^2

	double corner_acc_limit;				//拐角速度差限制，单位：mm/min

	uint16_t rapid_s_plan_filter_time;			//G00 S型速度规划时间常数

	uint8_t post_filter_type;		    //插补后滤波器类型		0--关闭    1--平滑滤波     2--均值滤波     3--S型均值滤波器
	uint16_t post_filter_time_1;		//插补后滤波器一级时间常数  单位：ms
	uint16_t post_filter_time_2;        //插补后滤波器二级时间常数  单位：ms
};

/**
 * @brief 激光切割参数
 */
struct LaserConfig{
	double follow_height;       //跟随高度   单位：mm
};


/**
 * @brief HMI工件坐标系配置
 */
struct HmiCoordConfig{
	double offset[kMaxAxisChn];    //轴工件坐标系偏置
};

//仿真数据
struct SimulateData{
	int type;   //指令类型: -3-仿真范围下限   -2-仿真范围上限    -1--初始位置   00-G00目标位置    01--G01目标位置  02--G02目标位置
	                       //03--G03目标位置   05--圆弧中心位置   06--圆弧半径   10--结束指令
	double pos[3];    //坐标数据，当数据类型为圆弧半径时，pos[0]用于存放半径，pos[1]用于存放当前平面（170--G17  180--G18   190--G19）
};

//仿真数据类型
enum SimDataType{
	ZONE_LIMIT_MIN = -3,   //仿真范围负向限制
	ZONE_LIMIT_MAX = -2,   //仿真范围正向限制
	ORIGIN_POINT = -1,     //初始位置
	RAPID_TARGET = 0,      //G00目标位置
	LINE_TARGET = 1,       //G01目标位置
	ARC2_TARGET = 2,       //G02圆弧目标位置
	ARC3_TARGET = 3,	   //G03圆弧目标位置
	ARC_CENTER = 5,        //圆弧圆心坐标
	ARC_RADIUS = 6         //圆弧半径
};


/**
 * @brief 通道工作模式
 */
enum ChnWorkMode{
	INVALID_MODE = 0,	//非法模式
	AUTO_MODE,			//自动模式
	MDA_MODE,			//MDA模式
	MANUAL_STEP_MODE,	//手动单步模式
	MANUAL_MODE,		//手动连续模式
	MPG_MODE,		    //手轮模式
	EDIT_MODE,          //编辑模式
	REF_MODE            //原点模式
};

/*
 * @brief 加工状态
 */
enum MachiningState {
	MS_READY = 0, 				//准备好（空闲）状态
	MS_RUNNING, 				//运行状态
	MS_OUTLINE_SIMULATING, 		//轮廓仿真中
	MS_TOOL_PATH_SIMULATING, 	//刀路仿真中
	MS_MACHIN_SIMULATING,       //加工仿真中
	MS_PAUSING, 				//暂停过程中
	MS_PAUSED, 					//暂停状态
	MS_STOPPING,				//停止过程中
	MS_REF_POINT_RETURNING, 	//参考点返回中
	MS_WARNING					//系统告警
};


/**
 * @brief 系统功能状态mask定义
 */
enum FuncStateMask{
	FS_SINGLE_LINE = 0,			//单段
	FS_DRY_RUN,					//空运行
	FS_OPTIONAL_STOP,			//选停
	FS_HANDWHEEL_CONTOUR,		//手轮跟踪
	FS_BLOCK_SKIP,				//跳段
	FS_EDIT_LOCKED,				//编辑锁
	FS_MACHINE_LOCKED,			//机床锁
	FS_AUXILIARY_LOCKED, 		//机床辅助锁
	FS_MANUAL_RAPID				//手动快速
};




/*
 * @brief PLC寄存器地址类型
 */
enum PmcAddress {
	NULL_ADDR = 0,
	X_ADDR,
	Y_ADDR,
	F_ADDR,
	G_ADDR,
	R_ADDR,
	K_ADDR,
	D_ADDR,
	A_ADDR,
	T_ADDR,
	C_ADDR,
	DC_ADDR,
	DT_ADDR
};


/**
 * @brief 监控数据类型
 */
enum MonitorDataType{
	MDT_INVALID = 0,  		//未定义数据       0
	MDT_CHN_RT_STATUS = 1,  //通道实时数据     1
	MDT_CHN_AXIS_POS,       //通道实时轴位置，刷新频率高于实时数据，图形模式使用    2
	MDT_CHN_PMC_AXIS_POS,   //PMC轴位置数据    3
	MDT_CHN_SIM_GRAPH,      //图形仿真数据     4
	MDT_CHN_SIM_TOOL_PATH,  //刀路仿真数据     5
	MDT_CHN_SIM_MACH,       //加工仿真数据     6
	MDT_CHN_DEBUG_INFO		//通道调试信息     7
};

//模块就绪bit定义
enum ModuleReadyFlag{
	NONE_READY = 0x00,	//无就绪模块
	SC_READY = 0x01,	//SC模块
	MC_READY = 0x02,	//MC模块
    MI_READY = 0x04,    //MI启动完成
	PL_READY = 0x08,	//PL模块
	SPANTAN_READY = 0x10,	//SPANTAN模块
	HMI_READY = 0x20,	//HMI模块
    MI_START = 0x40,	//MI模块（MI启动前即可开始刷新G信号，否则无法处理复位类型的请求）



#ifdef	USES_MC_B_BOARD
	NC_READY = 0x0F,    //NC模块都就绪，即除HMI模块外都就绪,MC-B没有SPANTAN
#else
	NC_READY = 0x1F,	//NC模块都就绪，即除HMI模块外都就绪
#endif
	ALL_READY = 0x3F    //所有模块就绪
};

//手动步长定义
enum ManualStep{
	MANUAL_STEP_INVALID = 0,	//非法值
    MANUAL_STEP_1,				//1um       //0.001mm
    MANUAL_STEP_10,				//10um      //0.01mm
    MANUAL_STEP_100,			//100um     //0.1mm
    MANUAL_STEP_500,            //500um     //0.5mm
    MANUAL_STEP_1000,			//1000um    //1mm
    MANUAL_STEP_5000,           //5000um    //5mm
    MANUAL_STEP_10000,          //10000um   //10mm
    MANUAL_STEP_50000,          //50000um   //50mm
};

//参数激活类型定义
enum ParamActiveType{
	ACTIVE_BY_POWEROFF = 0,	//重启生效
	ACTIVE_BY_RESET,        //复位生效
	ACTIVE_BY_RESTART,		//新一次加工生效
	ACTIVE_IMMEDIATE		//立即生效
};

//参数值类型定义
enum ParamValueType{
	VALUE_UINT8 = 0,	//uint8_t
	VALUE_INT8,			//int8_t
	VALUE_UINT16,		//uint16_t
	VALUE_INT16,		//int16_t
	VALUE_UINT32,		//uint32_t
	VALUE_INT32,		//int32_t
	VALUE_UINT64,		//uint64_t
	VALUE_INT64,		//int64_t
	VALUE_DOUBLE		//double
};

//HMI提示信息类别定义
enum HmiMsgType{
	MSG_INIT = 0,		//初始化信息
	MSG_TIPS 			//提示信息
};

//HMI提示信息ID定义
enum HmiMsgId{
	MSG_ID_AXIS_REF = 100,	//轴未确定参考点
	MSG_ID_LIC_OVER = 160,  //系统授权即将到期

	MSG_ID_RESTARTING = 200,   //程序再启动进行中

	MSG_ID_TOOL_CHANGING =210, //换刀进行中

	MSG_ID_GUARD

};

enum ToolPotLifeType {
    ToolPot_Close,      //关闭计次
    ToolPot_Cnt,        //换刀次数计次
};

enum SysUpdateType {
    BackUp_System_Param  = 0x01,        //系统参数
    Backup_Tool_Param    = 0x02,        //刀具信息
    Backup_Coord_Param   = 0x04,        //工件坐标系
    Backup_Pmc_Data      = 0x08,        //梯形图
    Backup_Macro_Param   = 0x10,        //宏变量
    Backup_Esb_Data      = 0x20,        //Esb文件
    Backup_Gcode_Data    = 0x40,        //G代码
    Backup_IO_Remap      = 0x80,        //IO重映射
    Backup_All           = 0xFFFF,      //全盘备份
};

// 备份/恢复当前状态
struct SysUpdateStatus {
    enum Status {
        Idle            = 0,    //未开始
        Ready,                  //准备开始
        Backupping,             //开始备份
        Recovering,             //开始恢复
        FileTransing,           //文件传输
    };
    enum Type {
        Unkown,                 //未指定
        Backup,                 //备份
        Recover,                //恢复
    };
    SysUpdateStatus() = default;
    SysUpdateStatus(int cur_step, int total_step, Status status, Type type)
        : m_cur_step(cur_step), m_total_step(total_step), m_status(status), m_type(type)
    { }

    int m_cur_step = 0;
    int m_total_step = 0;
    int m_err_code = 0;
    Status m_status = Idle;
    Type m_type = Unkown;
};

struct TraceLogInfo {
    int             opt;    //日志类型
    int             type;   //日志消息类型
    std::string     msg;    //日志内容
};

enum ScStatusType {
    Type_ProtectKey = 0,    //程序保护
};


/*****************************************************
** 伺服引导 *******************************************/

/**
 * @brief 伺服引导类型
 */
enum class SG_Config_Type
{
    Rect = 1,
    Circle,
    RecCir,
    Tapping
};

/**
 * @brief 方型轨迹
 */
struct SG_Rect_Config
{
    uint8_t axis_one = -1;
    uint8_t axis_two = -1;
    uint8_t interval = 8;       //采样周期
};

/**
 * @brief 圆型轨迹
 */
struct SG_Circle_Config
{
    uint8_t axis_one = -1;
    uint8_t axis_two = -1;
    uint8_t interval = 8;       //采样周期
    double radius = 5;          //半径
};

/**
 * @brief 方圆轨迹
 */
struct SG_RecCir_Config
{
    uint8_t axis_one = -1;
    uint8_t axis_two = -1;
    uint8_t interval = 8;       //采样周期
    double width = 10;          //宽度(不包括倒角半径)
    double height = 20;         //高度(不包括倒角半径)
    double radius = 5;          //倒角半径
};

/**
 * @brief 刚性攻丝轨迹
 */
struct SG_Tapping_Config
{
    //uint8_t axis_one = -1;      //主轴轴号
    //uint8_t axis_two = -1;      //Z轴轴号
    uint8_t interval = 8;
};

/* 伺服引导 *******************************************
******************************************************/



struct FS_Entity
{
    int type_ = FILE_UNKOWN;
    //std::string name_;
    int size_ = 0;
    time_t time_;
    char name_[128] ;
    //std::string current_dir;
};

class FS_File : public FS_Entity
{
public:
    std::string GetFileName();      //获取文件名称
private:
};

class FS_Dir : public FS_Entity
{
public:
    std::string GetDirName();       //获取目录名称
private:
};

class FS_FileSystem {
private:
    std::list<FS_File> file_entities_;  // 文件列表
    std::list<FS_Dir>  dir_entities_;   // 目录列表
    std::string root_path_;
};


#endif /* INC_HMI_SHARED_DATA_H_ */

