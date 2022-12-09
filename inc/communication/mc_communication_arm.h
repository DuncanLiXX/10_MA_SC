/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file mc_communication_arm.h
 *@author gonghao
 *@date 2021/11/22
 *@brief 本头文件为SC-MC-ARM通讯类的声明，适用于SC与运行于ARM上的MC通讯
 *@version
 */

#ifndef MC_COMMUNICATION_ARM_H_
#define MC_COMMUNICATION_ARM_H_
#include <stdint.h>
#include <pthread.h>
#include "list_buffer.h"
#include "comm_data_definition.h"
#include "hmi_shared_data.h"
#include "geometry_data.h"

class ChannelEngine;

//命令通道缓冲深度
#define MC_ARM_CMD_FIFO_DEPTH  (64)	//64个命令帧深度

//命令帧字节数
#define MC_ARM_CMD_FRAME_SIZE (32)

//G指令运动数据包大小，字节数
#define MC_ARM_GCODE_FRAME_SIZE   (128)   //每个运动数据包128字节

//每个通道的运动数据FIFO深度
#define MC_ARM_CHN_GCODE_FIFO_DEPTH   (16)   //每个通道的运动数据缓冲深度为16

//每个通道的运动数据缓冲所占字节数
#define MC_ARM_CHN_GCODE_FIFO_SIZE   (MC_ARM_GCODE_FRAME_SIZE*MC_ARM_CHN_GCODE_FIFO_DEPTH)    //2k字节

//寄存器定义
#define MC_ARM_REGISTER_BASE		(0x1FE00000)		//SC-MC数据交换共享内存基地址

//下行命令通道
#define MC_ARM_CMD_DOWN_BASE        (MC_ARM_REGISTER_BASE+0xB4000) //下行命令通道基地址
#define MC_ARM_CMD_DOWN_WF(n)       (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n)	//第n个命令帧的写标志
#define MC_ARM_CMD_DOWN_RF(n)       (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x02)	//第n个命令帧的读标志
#define MC_ARM_CMD_DOWN_DATA0(n)	 (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x04)	//第n个命令帧数据0寄存器
#define MC_ARM_CMD_DOWN_DATA1(n)	 (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x08)	//第n个命令帧数据1寄存器
#define MC_ARM_CMD_DOWN_DATA2(n)	 (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x0C)	//第n个命令帧数据2寄存器
#define MC_ARM_CMD_DOWN_DATA3(n)	 (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x10)	//第n个命令帧数据3寄存器
#define MC_ARM_CMD_DOWN_DATA4(n)	 (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x14)	//第n个命令帧数据4寄存器

//上行命令应答通道
#define MC_ARM_CMD_UP_BASE          (MC_ARM_REGISTER_BASE+0xB4800)     //上行命令通道基地址
#define MC_ARM_CMD_UP_WF(n)         (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n)		//第n个命令帧的写标志
#define MC_ARM_CMD_UP_RF(n)  	     (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x02)		//第n个命令帧的读标志
#define MC_ARM_CMD_UP_DATA0(n)	 	 (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x04)		//第n个命令帧数据0寄存器
#define MC_ARM_CMD_UP_DATA1(n)	 	 (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x08)		//第n个命令帧数据1寄存器
#define MC_ARM_CMD_UP_DATA2(n)	 	 (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x0C)		//第n个命令帧数据2寄存器
#define MC_ARM_CMD_UP_DATA3(n)	 	 (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x10)		//第n个命令帧数据3寄存器
#define MC_ARM_CMD_UP_DATA4(n)	 	 (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x14)		//第n个命令帧数据4寄存器

//上行状态数据通道，512bytes,n=0,1,只支持两通道
#define MC_ARM_STATUS_BASE(n)       (MC_ARM_REGISTER_BASE+0xB5000+2048*n)     //上行状态通道基地址，每个通道状态占2K字节
#define MC_ARM_STATUS_POS_WF(n)	 (MC_ARM_STATUS_BASE(n))		    //第n个通道位置数据的写标志
#define MC_ARM_STATUS_POS_RF(n)     (MC_ARM_STATUS_BASE(n) + 0x02)		//第n个通道位置数据的读标志
#define MC_ARM_AXIS_CUR_POS(n,m)	 (MC_ARM_STATUS_BASE(n)+0x10*m+0x08) 	//第n通道第m轴的当前插补位置，m从0开始
#define MC_ARM_AXIS_TAR_POS(n,m)	 (MC_ARM_STATUS_BASE(n)+0x10*m+0x10) 	//第n通道第m轴的当前插补目标位置，m从0开始

#define MC_ARM_STATUS_DATA_BASE(n)	 (MC_ARM_STATUS_BASE(n)+1024)        //第n通道状态数据基地址
#define MC_ARM_STATUS_WF(n)	 	 (MC_ARM_STATUS_DATA_BASE(n))		//第n个通道状态数据的写标志
#define MC_ARM_STATUS_RF(n)     	 (MC_ARM_STATUS_DATA_BASE(n)+0x02)	//第n个通道状态数据的读标志
#define MC_ARM_CUR_FRAME_INDEX(n)   (MC_ARM_STATUS_DATA_BASE(n)+0x04) //第n通道的当前运行数据帧的序号，n=0,1,只支持两通道
#define MC_ARM_CUR_LINE_NO(n)		 (MC_ARM_STATUS_DATA_BASE(n)+0x08) //第n通道的当前指令行号,n=0,1,只支持两通道
#define MC_ARM_CUR_FEED(n)          (MC_ARM_STATUS_DATA_BASE(n)+0x0C) //第n通道的当前进给速度,n=0,1,只支持两通道
#define MC_ARM_RATED_FEED(n)        (MC_ARM_STATUS_DATA_BASE(n)+0x10) //第n通道的给定进给速度,n=0,1,只支持两通道
#define MC_ARM_CUR_COORD(n)         (MC_ARM_STATUS_DATA_BASE(n)+0x14) //第n通道的当前坐标系,n=0,1,只支持两通道
#define MC_ARM_CUR_TOOL_OFFSET(n)   (MC_ARM_STATUS_DATA_BASE(n)+0x18) //第n通道的当前刀偏号,n=0,1,只支持两通道

#define MC_ARM_WORK_MODE(n)		   (MC_ARM_STATUS_DATA_BASE(n)+0x1C)	//第n通道的运行模式及运行指令寄存器地址
#define MC_ARM_CUR_CMD(n)             (MC_ARM_STATUS_DATA_BASE(n)+0x20)    //第n通道当前运行指令，G00、G01、G02、G03
#define MC_ARM_MODE_INFO(n)           (MC_ARM_STATUS_DATA_BASE(n)+0x24)    //第n通道的模态数据信息，n=0,1,只支持两通道
#define MC_ARM_AXIS_INTPOVER_MASK(n)  (MC_ARM_STATUS_DATA_BASE(n)+0x28)    //第n通道的轴插补到位mask，n=0,1,只支持两通道
#define MC_ARM_CUR_INTP_MODE(n)       (MC_ARM_STATUS_DATA_BASE(n)+0x2C)    //第n通道的当前联动插补模式（3-3轴联动，5-5轴联动，6-倾斜面，7-G12.2），n=0,1,只支持两通道

#define MC_ARM_BLOCK_RUN_OVER(n)      (MC_ARM_STATUS_DATA_BASE(n)+0x30)    //第n通道的分块插补到位标志
#define MC_ARM_STEP_RUN_OVER(n)	   (MC_ARM_STATUS_DATA_BASE(n)+0x34)    //第n通道的单段插补到位标志
#define MC_ARM_BLOCK_RUN_OVER_MDA(n)  (MC_ARM_STATUS_DATA_BASE(n)+0x38)	//第n通道MDA模式的块到位标志
#define MC_ARM_AUTO_BUF_DATA_COUNT(n) (MC_ARM_STATUS_DATA_BASE(n)+0x3C)    //第n通道的自动数据缓冲数量,n=0,1,只支持两通道
#define MC_ARM_MDA_BUF_DATA_COUNT(n)  (MC_ARM_STATUS_DATA_BASE(n)+0x40)    //第n通道的MDA数据缓冲数量,n=0,1,只支持两通道
#define MC_ARM_ERR_FLAG(n) 		   (MC_ARM_STATUS_DATA_BASE(n)+0x60)    //MC通道错误标志段, n=0,1,只支持两通道
#define MC_ARM_NEG_SOFT_LIMIT_MASK(n) (MC_ARM_STATUS_DATA_BASE(n)+0x64)    //第n通道的超负向软限位的轴Mask,n=0,1,只支持两通道
#define MC_ARM_POS_SOFT_LIMIT_MASK(n) (MC_ARM_STATUS_DATA_BASE(n)+0x68)    //第n通道的超正向软限位的轴Mask,n=0,1,只支持两通道
#define MC_ARM_POS_ERR_MASK(n)        (MC_ARM_STATUS_DATA_BASE(n)+0x6C)    //第n通道的位置指令过大的轴mask,n=0,1,只支持两通道


//#define MC_ARM_DEBUG_DATA(n)        (MC_ARM_STATUS_DATA_BASE + 0x1C0+0x04*n)   //读取第n组debug数据，每组为一个32位数


//下行运动数据通道，支持2通道，每通道2K字节
#define MC_ARM_GCODE_BASE              (MC_ARM_REGISTER_BASE+0xB0000)                          //下行运动数据通道基地址
#define MC_ARM_GCODE_DATA_BASE(n,m)	(MC_ARM_GCODE_BASE+MC_ARM_CHN_GCODE_FIFO_SIZE*n+MC_ARM_GCODE_FRAME_SIZE*m)		//运动数据基地址, n=0,1,只支持两通道, m为第几组缓冲数据
#define MC_ARM_GCODE_WF(n,m)           (MC_ARM_GCODE_DATA_BASE(n,m))		    //第n通道的第m个数据帧的写标志
#define MC_ARM_GCODE_RF(n,m)  	        (MC_ARM_GCODE_DATA_BASE(n,m)+0x02)		//第n通道的第m个数据帧的读标志
#define MC_ARM_GCODE_DATA(n,m,k)		(MC_ARM_GCODE_DATA_BASE(n,m)+0x04+0x04*k)    //第n通道的第m个运动数据帧中的第k个32位数据地址， n=0,1,只支持两通道,
                                                                                //m为第几组缓冲数据，0<=m<MC_ARM_CHN_GCODE_FIFO_DEPTH
                                                                                //k为组内的32bit数据偏移，0<=k<kMaxGCodeDataCount


typedef ListBuffer<McCmdFrame> McArmCmdBuffer;   //待输出的指令消息队列


/**
 * @brief SC模块与MC模块间的通讯类，采用fpga扩展EMIF接口
 */
class MCArmCommunication {
public:

	virtual ~MCArmCommunication();	//析构函数

	static MCArmCommunication *GetInstance();   //单例模式，获取此类实例的唯一访问点

	void SetInterface();   //设置接口

	//下行运动控制数据通道接口
	bool WriteGCodeData(uint8_t chn, GCodeFrame &data);   	//写入G代码编译出的运动控制数据
	bool CanWriteGCode(uint8_t chn);            //是否能写入运动控制数据


	//命令通道接口
	bool WriteCmd(McCmdFrame &data, bool resend = false); 	//写入指令
	bool ReadCmdRsp(McCmdFrame &data);    //读取指令
//	uint32_t ReadCmdFifoCount();        //读取命令通道FIFO中当前数据个数
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
    void ReadCurFeed(const uint8_t chn_index, int32_t &cur_feed);

	//读取通道当前给定进给速度
    void ReadRatedFeed(const uint8_t chn_index, int32_t &rated_feed);

	//读取通道当前自动数据缓冲数量
	void ReadAutoBufDataCount(const uint8_t chn_index, uint16_t &count);

	//读取通道当前MDA数据缓冲数量
	void ReadMdaBufDataCount(const uint8_t chn_index, uint16_t &count);


	//读取通道模态信息
	void ReadMcModeInfo(const uint8_t chn_index, uint32_t &mode_info);

	//读取AUTO分块插补到位标志
	bool ReadAutoBlockRunOverFlag(const uint8_t chn_index);


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

	//读取通道的当前正向软限位触发轴mask
	void ReadChnPosSoftLimtMask(uint8_t chn_index, uint32_t &axis_mask);

	//读取通道的当前负向软限位触发轴mask
	void ReadChnNegSoftLimtMask(uint8_t chn_index, uint32_t &axis_mask);

	//读取通道的当前位置指令过大告警轴mask
	void ReadChnPosErrMask(uint8_t chn_index, uint16_t &axis_mask);

	//读取通道的当前数据帧的序号
	void ReadChnCurFrameIndex(uint8_t chn_index, uint16_t &frame_index);


	//读取MC的调试数据
	void ReadDebugData(uint32_t *debug_data);

	//读取欠压告警信号
	bool ReadUnderVoltWarn();

private:
	MCArmCommunication();    //构造函数

	static void *ProcessCmdThread(void *args); //接收命令处理线程函数
	bool ProcessCmdFun();  	//命令处理函数

	bool Initialize();    //初始化函数
	bool Clean();          //清理函数

	void CalMcCmdCrc(McCmdFrame &cmd);     //计算MC命令包的CRC
	void CalMcGCodeFrameCrc(GCodeFrame &data);   //计算G代码数据帧的CRC

private://私有变量
	static MCArmCommunication *m_p_instance;    //单实例对象

	ChannelEngine *m_p_channel_engine;   //通道引擎指针

	McArmCmdBuffer *m_list_cmd;	//待输出的命令队列

	int m_n_mem_file;           //驱动文件打开句柄
	uint8_t *m_p_addr_base;		//基地址

	ErrorType m_error_code;    //最近一次的错误码

	pthread_mutex_t m_mutex_cmd;   // 下行命令通道互斥量,写入命令互斥

	pthread_t m_thread_process_cmd;    //命令处理线程

	uint8_t m_n_read_status_sem;    //状态读取信号量

	uint64_t m_n_crc_err_count;		//CRC校验错计数

	uint8_t m_n_cur_gcode_index[kMaxChnCount];    //当前下行运动命令帧序号   0~MC_ARM_CHN_GCODE_FIFO_DEPTH循环
	uint8_t m_n_cur_cmd_send_index;      //当前下行命令帧序号  0~MC_ARM_CMD_FIFO_DEPTH循环
	uint8_t m_n_cur_cmd_recv_index;      //当前响应命令帧序号  0~MC_ARM_CMD_FIFO_DEPTH循环

};

#endif /* MC_COMMUNICATION_ARM_H_ */
