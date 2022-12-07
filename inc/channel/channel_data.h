/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file channel_data.h
 *@author gonghao
 *@date 2020/04/23
 *@brief 本头文件用于通道使用的数据类型声明
 *@version
 */


#ifndef INC_CHANNEL_CHANNEL_DATA_H_
#define INC_CHANNEL_CHANNEL_DATA_H_

//#include "global_include.h"
#include "compiler_data.h"
#include "compile_message.h"

//需要发送到MC的模态组的标志
const unsigned char McModeFlag[kMaxGModeCount] = {0, 0, 1, 1, 0, 1, 0, 1, 0, 1,     		//0组~9组
										1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 		//10组~19组
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 		//20组~29组
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 		//30组~39组


/**
 * @brief 通道状态集合
 */
struct ChannelStatusCollect{
	double rated_manual_speed; 			//手动速度，单位：mm/s
	uint32_t workpiece_count;       	//工件计数，最大99999999
    uint32_t workpiece_require;         //需求件数
    uint32_t workpiece_count_total;     //总共件数
    uint32_t machinetime_total;         //累计加工时间
	Mask32 func_state_flags;        	//系统功能状态标志，包括单段、跳段、选停、空运行、手轮跟踪、机床锁，辅助锁
	int32_t rated_spindle_speed;           	//用户设定主轴转速，单位：转/分
	uint32_t rated_feed; 				//用户设定进给速度，单位：um/s
	uint16_t gmode[kMaxGModeCount];     //G指令模态通道当前模态
	uint8_t chn_work_mode;			    //通道模式, 自动、MDA、手动、手轮
	uint8_t machining_state;       	    //加工状态
	uint8_t cur_tool;               //当前刀号，从1开始编号，0为主轴刀号
	uint8_t preselect_tool_no;		//预选刀刀号
	uint8_t cur_h_code;				//当前H补偿号
	uint8_t cur_d_code;				//当前D补偿号
	uint8_t auto_ratio;            //自动进给倍率
	uint8_t spindle_ratio;         //主轴倍率
	uint8_t rapid_ratio;			//快速进给倍率
	uint8_t manual_ratio;          //手动倍率
	uint8_t manual_step;           //手动步长，0:INVALID;1:X1;2:X10;3:X100;4:X1000
	int8_t cur_manual_move_dir;		//当前手动移动方向，0--停止   1--正向   -1--负向
	uint8_t cur_axis;				//当前轴号,通道轴号
	uint8_t spindle_mode;           //主轴工作方式，0：速度控制方式；1：位置控制方式
	int8_t spindle_dir;				//主轴方向，-1：反转；0：停转；1：正转
	uint8_t returned_to_ref_point;	//是否已回参考点，bit0-bit7分别表示轴0-7是否已回参考点
	uint8_t es_button_on;			//急停按键是否处于按下状态，每一个bit表示一个急停按键
	char cur_nc_file_name[kMaxFileNameLen];	//当前加工主程序文件名

#ifdef USES_SPEED_TORQUE_CTRL
	uint8_t cur_axis_ctrl_mode[kMaxAxisChn];    //当前各通道轴控制模式, 2-速度控制模式    3-力矩控制模式
#endif
	uint8_t cur_chn_axis_phy[kMaxAxisChn];	    //当前通道内轴对应的物理轴索引
	int mcode; // @add zk 暂时没用
};


/**
 * @brief 通道实时状态,此结构的状态都需要从MC、MI实时获取
 */
struct ChannelRealtimeStatus{
	DPointChn cur_pos_machine;   		//当前各轴反馈位置，机械坐标系，单位：mm
	DPointChn cur_pos_work;			//当前各轴插补位置，工件坐标系，单位：mm
	DPointChn tar_pos_work;			//当前各轴目标位置，工件坐标系，单位：mm
	int32_t spindle_cur_speed; 				//主轴当前转速，单位：转/分
    int32_t cur_feed; 					  //当前进给速度，单位：um/s
	uint32_t machining_time; 				//加工时间，单位：秒
	uint32_t machining_time_remains; 		//剩余加工时间，单位：秒
    uint32_t machining_time_total;          //累计加工时间，单位：秒
	uint32_t line_no;						//当前行号
	
	DPointChn cur_feedbck_velocity;			//当前各轴实际速度，单位：mm/min
    DPointChn cur_feedbck_torque;			    //当前各轴实际力矩，单位：0.001额定力矩

    int32_t tap_err;                        //刚性攻丝误差(最大值) 单位：um
    int32_t tap_err_now;                    //刚性攻丝误差(当前值) 单位：um
};

//传递给MC的模态位结构
struct McModeBits{
	uint32_t mode_g17:2;		//02组
	uint32_t mode_g90:1;		//03组
	uint32_t mode_g94:1;		//05组
	uint32_t mode_g20:1;		//06组
	uint32_t mode_g40:4;		//07组
	uint32_t mode_g73:4;		//09组
	uint32_t mode_g98:1;		//10组
	uint32_t mode_g50:1;		//11组
	uint32_t mode_g60:1;		//13组
	uint32_t mode_g68:1;		//16组
	uint32_t mode_d:7;			//D值
	uint32_t reserved:8;		//保留
};
union McModeStatus{
	McModeBits bits;
	uint32_t all;
};

//传递给MC的通道错误标志结构
struct McErrorBits{
	uint32_t err_soft_limit_neg:1;		//bit0  负向软限位告警
	uint32_t err_soft_limit_pos:1;		//bit1  正向软限位告警
	uint32_t err_pos_over:1;		    //bit2  插补位置距离超限
	uint32_t err_arc_data:1;			//bit3  圆弧数据错
	uint32_t err_cmd_crc:1;				//bit4  命令传输校验错
	uint32_t err_data_crc:1;		    //bit5  数据传输校验错
	uint32_t reserved:26;		        //保留
};
union McErrorFlag{
	McErrorBits bits;
	uint32_t all;
};


/**
 * @brief MC模块通道状态数据结构
 */
struct ChannelMcStatus{
	DPoint intp_pos;   //各轴当前插补位置
	DPoint intp_tar_pos;   //各轴当前插补目标位置
	uint32_t cur_line_no;  //当前程序代码行
    int32_t cur_feed;		//当前进给速度,um/s
    int32_t rated_feed;	//当前给定进给速度，um/s
	uint32_t axis_over_mask;  	//轴插补到位mask
	McModeStatus mc_mode;		  //MC当前模态
	McErrorFlag mc_error;        //MC错误状态
	uint32_t run_over;
	uint16_t buf_data_count;	//自动加工缓冲数据量
	uint16_t mda_data_count;	//MDA缓冲数据量
	uint16_t cur_cmd;		//当前运行指令
	uint16_t cur_mode;		//当前模式， 0--手动  1--自动   2--MDA
	uint16_t axis_soft_negative_limit;   //轴负向软限位mask
	uint16_t axis_soft_postive_limit;	//轴正向软限位mask
	uint16_t pos_error_mask;   //位置指令过大告警的轴mask
	bool step_over;			//单段插补到位
	bool auto_block_over;	//AUTO分块插补到位
	bool mda_block_over;	//MDA分块插补到位
	
#ifdef USES_SPEED_TORQUE_CTRL
    DPoint intp_velocity;   //各轴当前速度指令值  单位：mm/min  ( 1°/min)
    DPoint intp_torque;     //各轴当前力矩指令值  单位：0.001额定力矩
#endif	
};




/**
 * @brief 通道自动模式状态场景，当状态由自动模式的非READY状态切出时，需要保存其状态，以方便在切回自动模式并继续运行时，将回退至之前的状态
 */
struct ChannelAutoScene{
	bool need_reload_flag;			//是否有数据需要恢复
	uint8_t machining_state;       	//加工状态
	uint8_t run_thread_state;		//保存自动模式的G代码运行线程状态备份
	uint8_t cur_tool;               //当前刀号，从1开始编号，0为主轴刀号
	uint8_t preselect_tool_no;		//预选刀刀号
	uint8_t cur_h_code;				//当前H补偿号
	uint8_t cur_d_code;				//当前D补偿号
	uint8_t spindle_mode;           //主轴工作方式，0：速度控制方式；1：位置控制方式
	int8_t spindle_dir;				//主轴方向，-1：反转；0：停转；1：正转
	uint16_t gmode[kMaxGModeCount]; //G指令模态通道当前模态
	int32_t rated_spindle_speed;           	//用户设定主轴转速，单位：转/分
	uint32_t rated_feed; 				//用户设定进给速度，单位：um/s
	McModeStatus mc_mode_exec;      //指令消息执行时修改的MC模态组信息
	DPointChn cur_pos_machine;			//当前各轴机械位置，机械坐标系，单位：mm

	uint8_t subprog_count;     	//子程序嵌套层数计数,包括宏程序调用
	uint8_t macroprog_count;    //宏程序嵌套层数计数，不包括子程序
	ListNode<RecordMsg *> *p_last_output_msg;   //最近发送的运动数据
};

/**
 * @brief 通道加工复位扫描模式状态记录，用于在加工复位中记录模式的中间状态
 */
struct ChnRestartMode{
	DPointChn pos_target;                   //目标位置
	int32_t rated_spindle_speed;           	//用户设定主轴转速，单位：转/分
	int32_t cur_scode;                      //当前S指令
	uint32_t rated_feed; 					//用户设定进给速度，单位：um/s
	uint16_t gmode[kMaxGModeCount]; 		//G指令模态通道当前模态
	uint8_t cur_tool;               		//当前刀号，从1开始编号，0为主轴刀号
	uint8_t cur_tcode;		                //当前T指令
	uint8_t cur_h_code;						//当前H补偿号
	uint8_t cur_d_code;						//当前D补偿号
	int8_t spindle_dir;						//主轴方向，-1：反转；0：停转；1：正转
	uint8_t sub_prog_call;                  //子程序调用记录，包括宏程序调用
};

//螺补数据分配表
struct AxisPcDataAlloc{
	uint8_t axis_index;      //轴序号，0开始
	uint16_t start_index;     //螺补数据起始点，0开始
	uint16_t end_index;       //螺补数据终点，0开始
	uint16_t pc_count;        //螺补数据个数，双向螺补则为：螺补点数*2
};
typedef ListBuffer<AxisPcDataAlloc> PcDataAllocList;      //螺补数据分布队列

/**
 * @brief 通道产品范围，X、Y、Z轴的切削区域
 */
struct ChnSimulateZone{
	double zone[3][2];    //维度1:0-X轴   1-Y轴   2-Z轴， 维度2:0-最小值   1-最大值
};

/**
 * @brief 五轴插补模式定义，与MC的定义同步
 */
enum FiveAxisIntpMode{
	CANCEL_MODE = 0,	//!< CANCEL_MODE
	G43_4_MODE = 5,     //!< G43_4_MODE
	G68_3_MODE = 6,     //!< G68_3_MODE
	G53_1_MODE = 7      //!< G53_1_MODE
};

/**
 * @brief 复位模式定义
 */
enum RestartMode{
	NOT_RESTART = 0, //!< NOT_RESTART     非加工复位
	NORMAL_RESTART, //!< NORMAL_RESTART   正常加工复位，每一行都进行解析
	FAST_RESTART    //!< FAST_RESTART     快速加工复位，只解析接近目的行号的1万行，前面的都只统计行号
};


#ifdef USES_ADDITIONAL_PROGRAM
//附加程序类型（前置程序，后置程序）
enum AddProgType{
	NONE_ADD = 0,              //非附加程序
	NORMAL_START_ADD = 1,      //加工启动前置程序
	CONTINUE_START_ADD = 2,    //断点继续前置程序
	RESET_START_ADD = 3,       //加工复位前置程序
	NORMAL_END_ADD = 11,       //加工结束后置程序  M30、M02
	PAUSE_ADD = 12,            //暂停后置程序
	RESET_ADD = 13             //复位后置程序
};
#endif

#endif /* INC_CHANNEL_CHANNEL_DATA_H_ */
