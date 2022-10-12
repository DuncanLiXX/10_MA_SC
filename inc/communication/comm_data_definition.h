/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file mc_communication.h
 *@author gonghao
 *@date 2020/06/29
 *@brief 本头文件为通讯相关数据结构的声明
 *@version
 */

#ifndef INC_COMMUNICATION_COMM_DATA_DEFINITION_H_
#define INC_COMMUNICATION_COMM_DATA_DEFINITION_H_

#include <stdint.h>
#include <netinet/in.h>
#include "circular_buffer.h"
#include "hmi_shared_data.h"

/*#####################################################################SC<-->MC############################################################################*/
//常量定义
const int kMaxCmdFifoCount = 15;  //  30;        //DSP改双通道，FIFO深度调整为16（20210415）  //命令行通道FIFO最大深度，实际深度为32，保险起见不写满。上行通道深度为16
const int kMaxMcStatusCount = 128;      //MC状态数据32bits数据字数量
const int kMaxGCodeDataCount = 30;      //28;  //GCode运动状态数据所占的32bits数据字数量
const int kMaxGCodeFifoCount = 15;  // 30;      //DSP改双通道，FIFO深度调整为16（20210415）		//下行运动指令数据通道FIFO深度，实际深度为32，保险起见不写满
const int kMaxCmdCount = 5;			//命令通道数据帧所占32bits数据字数量
const int kMaxDebugFifoCount = 254;	//调试数据通道FIFO最大深度，实际深度为256，保险起见不写满

//轴运动代码类型
enum MoveCmdType{
	MOVE_G00 = 0,
	MOVE_G01 = 1,
	MOVE_G02 = 2,
	MOVE_G03 = 3,

};

//SC向MC发送的运动控制数据包
struct GCodeData{
	uint16_t crc;			//CRC校验
	uint16_t frame_index;	//帧号，0-65535循环
	uint16_t cmd;			//bit0-bit7:轴运动代码类型：G00=0/G01=1/G02=2/G03=3, bit8-bit15:Reserved
	uint16_t ext_type;		//扩展类型
	uint32_t line_no;		//行号
	uint32_t feed;			//进给速度
	uint32_t mode;			//模式mask
	uint32_t reserved;		//预留
	int64_t pos0;			//轴位置坐标0（单位：0.1nm）
	int64_t pos1;			//轴位置坐标1（单位：0.1nm）
	int64_t pos2;			//轴位置坐标2（单位：0.1nm）
	int64_t pos3;			//轴位置坐标3（单位：0.1nm）
	int64_t pos4;			//轴位置坐标4（单位：0.1nm）
	int64_t pos5;			//轴位置坐标5（单位：0.1nm）
	int64_t pos6;			//轴位置坐标6（单位：0.1nm）
	int64_t pos7;			//轴位置坐标7（单位：0.1nm）
	int64_t arc_center0;	//圆心坐标X（单位：0.1nm）
	int64_t arc_center1;	//圆心坐标Y（单位：0.1nm）
	int64_t arc_center2;	//圆心坐标Z（单位：0.1nm）
	int64_t arc_radius;		//圆弧半径（单位：0.1nm）
};

//运动控制数据帧
union GCodeFrame{
	GCodeData data;								//用于数据访问
	int32_t data_w[kMaxGCodeDataCount];		//用于寄存器写入
	uint16_t data_crc[kMaxGCodeDataCount*2];	//用于crc计算
};

typedef CircularBuffer<GCodeFrame> GCodeFrameBuffer;  //运动控制数据帧缓冲，循环缓冲


//SC-MC命令结构
struct McCmdData{
	uint16_t cmd; 		//指令号
	uint8_t axis_index;	//轴号
	uint8_t channel_index;	//通道号
	uint16_t data[7];	//数据
	uint16_t crc;		//crc校验码
};

//SC-MC命令帧结构
union McCmdFrame{
	McCmdData data;						//指令包，用于数据访问
	int32_t data_w[kMaxCmdCount];		//用于寄存器写入与读出
	int16_t data_crc[kMaxCmdCount*2];	//用于crc计算
};

enum McCmdCode{
	//系统控制命令
	CMD_MC_SET_HARDWARE_INFO = 0x0001,		//设置硬件信息
	CMD_MC_SET_CNC_TYPE	= 0x0002,			//设置CNC加工模式
	CMD_MC_SET_CHN_AXIS_NUM = 0x0003,		//设置各通道轴数据
	CMD_MC_RESET_ALARM = 0x0004,			//告警复位清除
//	CMD_MC_CLEAR_DATA_BUF = 0x0005, 		//清空加工数据缓冲区
	CMD_MC_SET_INIT_PARAM = 0x0005,			//设置插补周期等初始化参数
	CMD_MC_SET_CHN_AXIS_NAME = 0x0006,		//设置通道轴名称
	CMD_MC_SYS_RESET = 0x0007,				//系统复位

	CMD_MC_SET_CHN_INTP_MODE = 0x0010,		//设置各通道插补模式（自动，MDA，手动）
	CMD_MC_INIT_CHN_AUTO_BUF = 0x0011,		//初始化各通道自动加工数据缓冲
	CMD_MC_INIT_CHN_MDA_BUF = 0x0012,		//初始化各通道MDA数据缓冲
//	CMD_MC_CLEAR_CHN_STEP_ARRIVED = 0x0013,	//
	CMD_MC_CLEAR_CHN_LINE_NO = 0x0014,		//清除运行加工行号
	CMD_MC_MULTI_AXIS_STOP = 0x0015,		//手动各轴停止
	CMD_MC_SET_CHN_WORK_MODE = 0x0016,      //设置通道的加工模式（自动、MDA、手动）
	CMD_MC_SET_G84_INTP_MODE = 0x0017,      //设置MC的特殊插补模式G84（刚性攻丝）
	CMD_MC_STOP_G31_CMD = 0x0018,           //通知MC停止G31移动命令
	CMD_MC_SET_CHN_PLAN_MODE = 0x0020,		//设置通道加工速度规划模式
	CMD_MC_SET_CHN_PLAN_PARAM = 0x0021,		//设置通道加工速度规划参数
	CMD_MC_SET_CHN_PLAN_SWITCH = 0x0022,	//设置通道加工速度功能开关
	CMD_MC_SET_CHN_CORNER_STOP_PARAM = 0x0023,	//设置拐角准停参数
	CMD_MC_SET_CHN_RATIO = 0x0024,			//设置通道加工倍率
	CMD_MC_SET_CHN_WORK_BAND = 0x0025, 		//设置通道加工精度等级
	CMD_MC_SET_STEP_MODE = 0x0026,			//设置单段模式
	CMD_MC_ACTIVE_CHN_ORIGIN = 0x0027,		//更新通道所有轴的坐标系原点
	CMD_MC_ACTIVE_CHN_TOOL_OFFSET = 0x0028,	//更新通道所有轴的刀具偏置
	CMD_MC_SET_CHN_TOOL_LIFE = 0x0029,      //设置通道当前刀具寿命
	CMD_MC_SET_G84_PARAM = 0x002A,			//设置刚攻参数
	CMD_MC_SET_FLIP_COMP_PARAM = 0x002B,    //设置木工挑角补偿参数   木工机专用


	CMD_MC_SET_MULTI_AXIS_MODE = 0x0030,	//设置多轴联动模式
	CMD_MC_SET_CHN_FIVE_AXIS_PARAM = 0x0031,	//设置五轴联动加工相关参数
	CMD_MC_SET_CHN_TILT_AXIS_PARAM = 0x0032,	//设置倾斜轴加工相关参数


	//磨削指令
	CMD_MC_SET_SPECIAL_INTP_MODE = 0x0040,     //设置专用插补使能   0--G13.1    121--G12.1   122--G12.2   123--G12.3   71--G7.1
	CMD_MC_SET_SPEC_INTP_PARM_DYNAMIC = 0x0041,		//设置专用插补的动态参数
	CMD_MC_ENABLE_GRIND_SHOCK = 0x0042,       //使能G12.2的震荡磨削
	CMD_MC_SET_SPEC_INTP_PARM_STATIC = 0x0043,	//设置G12.2的静态参数

	//调试参数
	CMD_MC_SET_CHN_DEBUG_PARAM = 0x0060,     //设置通道调试参数

	//升级命令
	CMD_MC_UPDATE_START = 0x0070,			//开始升级
	CMD_MC_UPDATE_FILE_SIZE = 0x0071,		//升级文件大小，字节数
	CMD_MC_UPDATE_CRC = 0x0072,				//发送升级文件CRC

	//轴控制命令
	CMD_MC_SET_AXIS_ON = 0x0081,			//轴使能设置
	CMD_MC_SET_AXIS_BASE_INFO = 0x0082,	    //设置各轴螺距信息
	CMD_MC_SET_AXIS_SPEED_LIMIT = 0x0083,	//设置各轴速度限制
	CMD_MC_SET_AXIS_ACC = 0x0084,			//设置各轴加速度值
	CMD_MC_AXIS_MAUAL_MOVE = 0x0085, 		//轴移动指令
	CMD_MC_SET_AXIS_ORIGIN = 0x0086,		//设置单轴坐标系原点
	CMD_MC_SET_AXIS_TOOL_OFFSET = 0x0087,	//设置单轴刀具偏置
	CMD_MC_SET_AXIS_HW_INSERT = 0x0088,		//手轮插入坐标清零
	CMD_MC_SET_AXIS_SOFT_LIMIT = 0x0090,	//设置软限位值
	CMD_MC_AXIS_ENABLE_SOFT_LIMIT = 0x0091, //设置软限位检测开关
	CMD_MC_AXIS_POS_THRESHOLD = 0x0092,     //设置各轴位置指令门限值

	//状态反馈命令
	CMD_MC_GET_VERSION = 0x0110,			//读取DSP版本号
	CMD_MC_SHAKEHANDS = 0x0111,				//初始化握手
	CMD_MC_GET_LIMIT_LEVEL = 0x0112,		//读取限位的电平状态
	CMD_MC_GET_UPDATE_STATE = 0x0120,		//读取升级状态
	CMD_MC_TEST_FLASH = 0x0121,				//测试FLASH命令


	//MC

	CMD_MC_GUARD							//卫兵
};


/*#####################################################################SC<-->MI############################################################################*/
//SC-MI命令结构
struct MiCmdData{
	uint16_t cmd; 		//指令号
	uint8_t axis_index;	//轴号,物理轴索引号
	uint8_t reserved;	//保留
	uint16_t data[7];	//数据
	uint16_t crc;		//crc校验码
};

//SC-MI命令帧结构
union MiCmdFrame{
	MiCmdData data;						//指令包，用于数据访问
	int32_t data_w[kMaxCmdCount];		//用于寄存器写入与读出
	int16_t data_crc[kMaxCmdCount*2];	//用于crc计算
};

//PMC轴运动指令结构
struct PmcCmdData{
	uint16_t cmd;			//运动指令号
	uint16_t axis_index;	//对应的物理轴号
	uint16_t data[7];       //数据
	uint16_t crc;		    //crc校验码
};

//PMC轴运动命令帧结构
union PmcCmdFrame{
	PmcCmdData data;					//指令包，用于数据访问
	int32_t data_w[kMaxCmdCount];		//用于寄存器写入与读出
	int16_t data_crc[kMaxCmdCount*2];	//用于crc计算
};

//MI指令字定义
enum MiCmdCode{
	//MI->SC
	CMD_MI_READY = 0x01,					//通知SC模块MI准备好
	CMD_MI_ALARM = 0x02,					//通知SC模块MI产生告警
	CMD_MI_PMC_CODE_REQ = 0x03,				//向SC请求PLC代码
	CMD_MI_SHAKEHAND = 0x04,				//MI与SC握手命令
	CMD_MI_HEARTBEAT = 0x05,				//MI与SC之间的心跳
	CMD_MI_BUS_ALARM = 0x06,           		//MI通知SC总线通讯故障
	CMD_MI_PMC_AXIS_RUNOVER = 0x07,         //MI通知PMC轴运动指令执行完成
	CMD_MI_REFRESH_AXIS_ZERO = 0x08,        //MI通知SC更新轴零点偏移
	CMD_MI_HW_TRACE_STATE_CHANGED = 0x09,   //MI通知SC手轮跟踪状态切换
	CMD_MI_GET_ESB = 0x0A,                  //MI向SC获取ESB设备数据

	//SC->MI
	CMD_MI_RD_PMC_REG = 0x100,				//SC读取PMC寄存器值
	CMD_MI_WR_PMC_REG = 0x101,				//SC写入PMC寄存器值
	CMD_MI_SET_PARAM = 0x102,				//SC向MI设置参数
	CMD_MI_GET_PARAM = 0x103,				//SC向MI获取参数
	CMD_MI_OPERATE = 0x104,					//SC指令MI执行某个动作，例如上伺服，断伺服等等
	CMD_MI_INIT_COMPLETE = 0x0105,			//SC向MI初始化参数完成
	CMD_MI_RESET = 0x0106,					//SC通知MI进行复位
	CMD_MI_SET_REF = 0x0107,				//SC设置指定轴的参考点,用于绝对值编码器在初始化时设置轴零点
	CMD_MI_SET_REF_CUR = 0x0108,			//SC指令MI将指定轴的当前编码器值设为参考点, 绝对值编码器和无反馈的轴用此命令
	CMD_MI_ACTIVE_Z_CAPT = 0x0109,			//SC激活指定轴的Z信号捕获功能
	CMD_MI_SET_SPD_SPEED = 0x010A,			//SC设置主轴转速
	CMD_MI_CLEAR_ROT_AXIS_POS = 0x010B,		//SC对旋转轴位置清整数圈
	CMD_MI_SET_AXIS_CHN_PHY_MAP = 0x010C,   //SC设置通道轴与物理轴的映射关系
	CMD_MI_SET_AXIS_INIT_ENCODER = 0x010D,  //SC设置上次断电前的轴编码器反馈
	CMD_MI_DO_SYNC_AXIS = 0x010E,			//SC指令MI进行同步轴同步
	CMD_MI_SET_WORK_MODE = 0x010F,		    //SC设置工作模式
	CMD_MI_ENABLE_FOLLOW = 0x0110,			//使能调高器跟随功能
	CMD_MI_SET_LASER_OUTPUT = 0x0111,		//设置激光调高器输出IO
	CMD_MI_READ_LASER_INPUT = 0x0112,		//读取激光调高器输入IO
	CMD_MI_SET_POS_CAPTURE = 0x0113,		//设置轴位置捕获
	CMD_MI_SET_SDLINK_SLAVE = 0x0114,       //设置SD-LINK从站信息
	CMD_MI_SET_SDLINK_COMPLETE = 0x0115,    //设置SD-LINK从站信息完成
	CMD_MI_PAUSE_PMC_AXIS = 0x0116,			//暂停PMC轴运行
	CMD_MI_ACTIVE_TWINING_FUNC = 0x0117,	//使能缠绕功能
	CMD_MI_START_RET_REF = 0x0118,			//开始回参考点
	CMD_MI_GET_CUR_ENCODER = 0x0119,		//获取当前编码器反馈单圈绝对值
	CMD_MI_SET_REF_POINT = 0x011A,			//设置指定轴的参考点，用于有反馈的轴有基准回零设置零点
	CMD_MI_SET_HANDWHEEL_TRACE = 0x011B,    //设置手轮跟踪状态
	CMD_MI_SET_RET_REF_FLAG = 0x011C,       //设置轴回参考点结束标志
	CMD_MI_GET_ZERO_ENCODER = 0x011D,       //SC获取指定轴的机械零点对应的编码器值
	CMD_MI_ACTIVE_SKIP = 0x011E,			//SC激活MI的跳转功能，发送跳转信号及轴mask
	
	CMD_MI_SET_AXIS_SPEED  = 0x0120,       //设置轴运行速度值
	CMD_MI_SET_AXIS_TORQUE = 0x0121,       //设置轴运行力矩值
    CMD_MI_SET_AXIS_CTRL_MODE = 0x0122,       //设置轴运行控制模式
	CMD_MI_SET_AXIS_REF_MACH_POS = 0x123,     //设置轴参考点对应的机械坐标

	CMD_MI_SET_CUR_CHN = 0x0124,			//设置当前通道号
	CMD_MI_SET_TAP_AXIS = 0x0125,            //设置攻丝轴号
	CMD_MI_SET_TAP_PARAM = 0x0126,           //设置攻丝参数
	CMD_MI_SET_TAP_RATIO = 0x0127,           //设置攻丝比例
	CMD_MI_SET_TAP_STATE = 0x0128,           //设置攻丝状态开关

	CMD_MI_EN_SYNC_AXIS = 0x129,            //同步轴使能

	CMD_MI_SET_AXIS_PC_DATA = 0x130,      //设置轴螺补数据表
	CMD_MI_SET_AXIS_PC_PARAM = 0x131,     //设置轴螺补参数
	CMD_MI_SET_AXIS_PC_PARAM2 = 0x132,     //设置轴螺补参数2，即参考点对应的机械坐标
	CMD_MI_SET_AXIS_BACKLASH = 0x133,     //设置轴的反向间隙参数

	CMD_MI_SET_PMC_REG_BIT = 0x134,       //设置PMC寄存器位
	CMD_MI_SET_IO_REMAP = 0x135,          //设置IO重定向数据

	CMD_MI_SET_AXIS_MACH_POS = 0x136,     //设置指定物理轴当前位置的机械坐标
    CMD_MI_SPD_LOCATE = 0x137,            //SC通知MI主轴定位

	CMD_MI_DEBUG_M_FUN = 0x8001,            //MI调试指令
	CMD_MI_HW_TRACE_STATE_CHANGED_RSP = 0x8009,   //MI通知SC手轮跟踪状态切换响应命令

	CMD_MI_GUARD							//卫兵
};


/*#####################################################################SC<-->HMI############################################################################*/
const int kCmdRecvBufLen = 100;  //udp命令接受缓冲区大小，环形缓冲
const int kTcpBufferSize = 1024*8;  //tcp传输缓冲区大小,8K
const unsigned int kHeartbeatTimeout = 100; //30;  //30个监控周期，30*50ms
const int kTcpSockTimeout = 3;		//TCP连接接收数据（包括recv和accept）超时时间：3秒

//收到的HMI指令节点
struct HMICmdRecvNode{
	HMICmdFrame cmd;			   //命令包
	struct sockaddr_in ip_addr;   //发送源地址
};

/**
 * @brief HMICmdNode结构用于定义SC模块发送给HMI的命令的重发队列节点
 */
struct HMICmdResendNode {
	HMICmdFrame frame;
	uint8_t timeout;   //超时计数器，初始值为5,每100ms减一，减到0则重发，并将timeout计数加一
	uint8_t resend_count;  //重发计数器，记录重发次数
};

enum ThreadRunFlag{
	THREAD_CMD_PROCESS = 0x01,	//命令处理线程
	THREAD_CMD_RESEND = 0x02,	//命令重发线程
	THREAD_FILE_TRANS = 0x04,	//文件传输线程
	THREAD_MONITOR = 0x08,		//监控线程
	THREAD_FILE_SAVEAS = 0x10	//文件另存为线程
};

#endif /* INC_COMMUNICATION_COMM_DATA_DEFINITION_H_ */
