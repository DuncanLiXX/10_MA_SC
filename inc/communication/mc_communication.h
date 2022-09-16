/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file mc_communication.h
 *@author gonghao
 *@date 2020/06/15
 *@brief 本头文件为SC-MC通讯类的声明
 *@version
 */

#ifndef MC_COMMUNICATION_H_
#define MC_COMMUNICATION_H_
#include <stdint.h>
#include <pthread.h>
#include "list_buffer.h"
#include "comm_data_definition.h"
#include "hmi_shared_data.h"
#include "geometry_data.h"

class ChannelEngine;

//寄存器定义
#define MC_REGISTER_BASE		(0x43C00000)		//SC-MC数据交换寄存器基地址

//下行命令通道
#define MC_CMD_DOWN_FIFO_COUNT  (MC_REGISTER_BASE+0x00)	//FIFO计数,
#define MC_CMD_DOWN_WRITE_OVER  (MC_REGISTER_BASE+0x04)	//SC写入完成标志，最低位有效，写入前置0，完成后置1
#define MC_CMD_DOWN_RESET  	 (MC_REGISTER_BASE+0x08)	//全局复位信号，最低位有效，低电平有效即0有效
#define MC_CMD_DOWN_DATA0	 	 (MC_REGISTER_BASE+0x0C)	//命令帧数据0寄存器
#define MC_CMD_DOWN_DATA1	 	 (MC_REGISTER_BASE+0x10)	//命令帧数据1寄存器
#define MC_CMD_DOWN_DATA2	 	 (MC_REGISTER_BASE+0x14)	//命令帧数据2寄存器
#define MC_CMD_DOWN_DATA3	 	 (MC_REGISTER_BASE+0x18)	//命令帧数据3寄存器
#define MC_CMD_DOWN_DATA4	 	 (MC_REGISTER_BASE+0x1C)	//命令帧数据4寄存器

//上行命令应答通道
#define MC_CMD_UP_FIFO_COUNT    (MC_REGISTER_BASE+0x30)		//FIFO计数,
#define MC_CMD_UP_READ_REQ  	 (MC_REGISTER_BASE+0x34)		//SC命令读取请求标志，最低位有效，读取前置1，完成后置0
#define MC_CMD_UP_DATA0	 	 (MC_REGISTER_BASE+0x38)		//命令帧数据0寄存器
#define MC_CMD_UP_DATA1	 	 (MC_REGISTER_BASE+0x3C)		//命令帧数据1寄存器
#define MC_CMD_UP_DATA2	 	 (MC_REGISTER_BASE+0x40)		//命令帧数据2寄存器
#define MC_CMD_UP_DATA3	 	 (MC_REGISTER_BASE+0x44)		//命令帧数据3寄存器
#define MC_CMD_UP_DATA4	 	 (MC_REGISTER_BASE+0x48)		//命令帧数据4寄存器

//上行状态数据通道，512bytes,n=0,1,只支持两通道
#define MC_STATUS_READ_REQ		 (MC_REGISTER_BASE+0x90)		//SC状态读取请求标志，最低位有效，读取前置1，完成后置0
#define MC_STATUS_DATA_BASE	 (MC_REGISTER_BASE+0x94)        //MC状态数据基地址
#define MC_STATUS_DATA(n)       (MC_STATUS_DATA_BASE+0x04*n)   //MC状态数据第n个数据(32bits)的寄存器地址，0<=n<kMaxMcStatusCount
#define MC_CUR_LINE_NO(n)		 (MC_STATUS_DATA_BASE+0x100+0x04*n) //第n通道的当前指令行号,n=0,1,只支持两通道
#define MC_CUR_COORD(n)         (MC_STATUS_DATA_BASE+0x108+0x02*n) //第n通道的当前坐标系,n=0,1,只支持两通道
#define MC_CUR_TOOL_OFFSET(n)   (MC_STATUS_DATA_BASE+0x10C+0x02*n) //第n通道的当前刀偏号,n=0,1,只支持两通道
#define MC_CUR_FEED(n)          (MC_STATUS_DATA_BASE+0x110+0x04*n) //第n通道的当前进给速度,n=0,1,只支持两通道
#define MC_RATED_FEED(n)        (MC_STATUS_DATA_BASE+0x118+0x04*n) //第n通道的给定进给速度,n=0,1,只支持两通道
#define MC_WORK_MODE(n)		 (MC_STATUS_DATA_BASE+0x120+0x04*n)	//第n通道的运行模式及运行指令寄存器地址，低16bit为运行模式，高16bit为运行指令
#define MC_MODE_INFO(n)         (MC_STATUS_DATA_BASE+0x128+0x04*n) //第n通道的模态数据信息，n=0,1,只支持两通道
#define MC_AXIS_INTPOVER_MASK(n) (MC_STATUS_DATA_BASE+0x130+0x04*n) //第n通道的轴插补到位mask，n=0,1,只支持两通道
#define MC_CUR_INTP_MODE(n)      (MC_STATUS_DATA_BASE+0x138+0x02*n) //第n通道的当前联动插补模式（3-3轴联动，5-5轴联动，6-倾斜面，7-G12.2），n=0,1,只支持两通道
#define MC_CUR_FRAME_INDEX(n)    (MC_STATUS_DATA_BASE+0x13C+0x02*n) //第n通道的当前运行数据帧的序号，n=0,1,只支持两通道
#define MC_AUTO_BUF_DATA_COUNT(n) 	(MC_STATUS_DATA_BASE+0x160+0x02*n) //第n通道的自动数据缓冲数量,n=0,1,只支持两通道
#define MC_MDA_BUF_DATA_COUNT(n)   (MC_STATUS_DATA_BASE+0x170+0x02*n) //第n通道的MDA数据缓冲数量,n=0,1,只支持两通道
#define MC_SOFT_LIMIT_MASK(n)   (MC_STATUS_DATA_BASE+0x180+0x04*n)    //第n通道的超软限位的轴Mask,n=0,1,只支持两通道，低16bit为负向限位，高16bit为正向限位
#define MC_POS_ERR_MASK(n)      (MC_STATUS_DATA_BASE+0x188+0x02*n)    //第n通道的位置指令过大的轴mask,n=0,1,只支持两通道
#define MC_ERR_FLAG(n) 		     (MC_STATUS_DATA_BASE+0x1A0+0x04*n) 		//MC通道错误标志段, n=0,1,只支持两通道
//#define MC_BLOCK_RUN_OVER(n)   (MC_STATUS_DATA_BASE+0x150+0x04*n/2) //第n通道的分块插补到位标志
//#define MC_STEP_RUN_OVER(n)	 (MC_STATUS_DATA_BASE+0x140+0x04*n/2)	//第n通道的单段插补到位标志
#define MC_RUN_OVER_FLAG		 (MC_STATUS_DATA_BASE+0x140)	//低16bit为单段插补到位标志，高16bit为分块插补到位标志，都是一个bit表示一个通道，
																//从bit0到bit15由低到高顺序排列
#define MC_BLOCK_RUN_OVER_MDA   (MC_STATUS_DATA_BASE+0x144)	//低16bit为MDA模式的块到位标志
#define MC_CHN_CUR_TOOL_LIFE(n) (MC_STATUS_DATA_BASE+0x148+0x04*n)    //第n通道的当前刀具寿命，n=0,1,只支持两通道
//#define MC_MDA_DATA_COUNT		 (MC_STATUS_DATA_BASE+0x170)		//第n通道的MDA数据缓冲数量,n=0,1,只支持两通道
#define MC_AXIS_CUR_POS_L(n,m)	 (MC_STATUS_DATA_BASE+0x80*n+0x10*m) 	//第n通道第m轴的当前插补位置低32位地址
#define MC_AXIS_CUR_POS_H(n,m)  (MC_AXIS_CUR_POS_L(n,m)+0x04)		//第n通道第m轴的当前插补位置高32位地址
#define MC_AXIS_TAR_POS_L(n,m)	 (MC_STATUS_DATA_BASE+0x08+0x80*n+0x10*m) 	//第n通道第m轴的当前插补目标位置低32位地址
#define MC_AXIS_TAR_POS_H(n,m)  (MC_AXIS_TAR_POS_L(n,m)+0x04)		//第n通道第m轴的当前插补目标位置高32位地址

#define MC_DEBUG_DATA(n)        (MC_STATUS_DATA_BASE + 0x1C0+0x04*n)   //读取第n组debug数据，每组为一个32位数


//下行运动数据通道，支持2通道，每通道120bytes
#define MC_GCODE_FIFO_COUNT(n)    (MC_REGISTER_BASE+0x02B0+0x1A0*n)		//FIFO计数,n=0,1,只支持两通道
#define MC_GCODE_WRITE_OVER(n)  	(MC_REGISTER_BASE+0x02B4+0x1A0*n)		//SC写入完成标志，最低位有效，写入前置0，完成后置1,n=0,1,只支持两通道
#define MC_GCODE_DATA_BASE(n)		(MC_REGISTER_BASE+0x2B8+0x1A0*n)		//运动数据基地址, n=0,1,只支持两通道
#define MC_GCODE_DATA(n,m)		(MC_GCODE_DATA_BASE(n)+0x04*m)		//第n个运动数据地址， n=0,1,只支持两通道, 0<=m<kMaxGCodeDataCount


#define PL_PROG_VERSION        (MC_REGISTER_BASE+0x07F0)       //PL当前程序版本

//Spartan6加载通道
#define SP6_INIT_DATA_FIFO     (MC_REGISTER_BASE+0x07D0)       //FIFO当前数据量，最大16
#define SP6_INIT_DATA(n)		  (MC_REGISTER_BASE+0x07D4+0x04*n)	//加载数据区，0<=n<=3
#define SP6_INIT_DATA_WRITE	  (MC_REGISTER_BASE+0x07E4)     //PS写入数据完成，bit0为1标志写入数据完成，PL可以读取数据了
#define SP6_INIT_DATA_WR_TOTAL    (MC_REGISTER_BASE+0x07E8)		//所有数据写入完成
#define SP6_INIT_DATA_CHECK_RES  (MC_REGISTER_BASE+0x07EC)     //数据校验结果


//fpga版本信息
#define PL_VERSION     (MC_REGISTER_BASE+0x076C)      //PL版本
#define SP6_VERSION	(MC_REGISTER_BASE+0x0768)		//SPARTAN6版本

//欠压告警寄存器，最低位有效，高有效
#define UNDER_VOLTAGE_ALARM     (MC_REGISTER_BASE+0x07A4)    //欠压告警寄存器

typedef ListBuffer<McCmdFrame> McCmdBuffer;   //待输出的指令消息队列


/**
 * @brief SC模块与MC模块间的通讯类，采用fpga扩展EMIF接口
 */
class MCCommunication {
public:

	// @test zk
	void test(int16_t frame_index);

	virtual ~MCCommunication();	//析构函数

	static MCCommunication *GetInstance();   //单例模式，获取此类实例的唯一访问点

	void SetInterface();   //设置接口

	//下行运动控制数据通道接口
	bool WriteGCodeData(uint8_t chn, GCodeFrame &data);   	//写入G代码编译出的运动控制数据
	bool CanWriteGCode(uint8_t chn);            //是否能写入运动控制数据
	uint32_t ReadGCodeFifoCount(uint8_t chn);              //读取下行运控数据FIFO的数据个数


	//命令通道接口
	bool WriteCmd(McCmdFrame &data, bool resend = false); 	//写入指令
	bool ReadCmdRsp(McCmdFrame &data);    //读取指令
	uint32_t ReadCmdFifoCount();        //读取命令通道FIFO中当前数据个数
	uint32_t ReadCmdBufferLen();          //读取命令缓冲数据个数

	//读取轴插补位置
	void ReadAxisIntpPos(const uint8_t chn_index, DPoint &cur_pos, DPoint &tar_pos);

	//读取通道运行模式及运行指令
	void ReadWorkMode(const uint8_t chn_index, uint16_t &work_mode);

	//读取通道当前运行指令
	void ReadCurCmd(const uint8_t chn_index, uint16_t &cur_cmd);

	//读取通道当前运行行号
	void ReadCurLineNo(const uint8_t chn_index, uint32_t &cur_lineno);

	//读取通道当前实际进给速度
	void ReadCurFeed(const uint8_t chn_index, uint32_t &cur_feed);

	//读取通道当前给定进给速度
	void ReadRatedFeed(const uint8_t chn_index, uint32_t &rated_feed);

	//读取通道当前自动数据缓冲数量
	void ReadAutoBufDataCount(const uint8_t chn_index, uint16_t &count);

	//读取通道当前MDA数据缓冲数量
	void ReadMdaBufDataCount(const uint8_t chn_index, uint16_t &count);

	//读取通道模态信息
	void ReadMcModeInfo(const uint8_t chn_index, uint32_t &mode_info);

	//读取AUTO分块插补到位标志
	bool ReadAutoBlockRunOverFlag(const uint8_t chn_index);

	//读取到位标志字段
	uint32_t ReadRunOverValue();

	//读取单段插补到位标志
	bool ReadStepRunOverFlag(const uint8_t chn_index);

	//读取MDA模式块到位标志
	bool ReadMdaBlockRunOverFlag(const uint8_t chn_index);

	//更新指定通道轴到位标志
	void ReadChnAxisRunoverMask(const uint8_t chn_index, uint32_t &axis_mask);

	//读取通道当前坐标系
	void ReadChnCurCoord(uint8_t chn_index, uint16_t &coord);

	//读取通道当前刀偏号
	void ReadChnCurToolOffset(uint8_t chn_index, uint16_t &tool_offset);

	//读取当前联动插补模式
	void ReadChnCurIntpMode(uint8_t chn_index, uint16_t &intp_mode);

	//读取当前MC错误标志
	void ReadMcErrFlag(uint8_t chn_index, uint32_t &mc_err);

	//读取通道的当前软限位触发轴mask
	void ReadChnSoftLimtMask(uint8_t chn_index, uint32_t &axis_mask);

	//读取通道的当前位置指令过大告警轴mask
	void ReadChnPosErrMask(uint8_t chn_index, uint16_t &axis_mask);

	//读取通道的当前数据帧的序号
	void ReadChnCurFrameIndex(uint8_t chn_index, uint16_t &frame_index);


	//读取通道当前刀具累计寿命
	void ReadChnCurToolLife(uint8_t chn_index, int32_t &tool_life);

	//读取Zynq-pl的版本
	bool ReadPlVer();

	//读取sp6的版本
	bool ReadSp6Ver();

	//读取MC的调试数据
	void ReadDebugData(uint32_t *debug_data);

	//读取欠压告警信号
	bool ReadUnderVoltWarn();

private:
	MCCommunication();    //构造函数

	static void *ProcessCmdThread(void *args); //接收命令处理线程函数
	bool ProcessCmdFun();  	//命令处理函数

	bool Initialize();    //初始化函数
	bool Clean();          //清理函数

	void CalMcCmdCrc(McCmdFrame &cmd);     //计算MC命令包的CRC
	void CalMcGCodeFrameCrc(GCodeFrame &data);   //计算G代码数据帧的CRC

private://私有变量
	static MCCommunication *m_p_instance;    //单实例对象

	ChannelEngine *m_p_channel_engine;   //通道引擎指针

	McCmdBuffer *m_list_cmd;	//待输出的命令队列

	int m_n_mem_file;           //驱动文件打开句柄
	uint8_t *m_p_addr_base;		//基地址

	ErrorType m_error_code;    //最近一次的错误码

	pthread_mutex_t m_mutex_cmd;   // 下行命令通道互斥量,写入命令互斥

	pthread_t m_thread_process_cmd;    //命令处理线程

	uint8_t m_n_read_status_sem;    //状态读取信号量

	uint64_t m_n_crc_err_count;		//CRC校验错计数

};

#endif /* MC_COMMUNICATION_H_ */
