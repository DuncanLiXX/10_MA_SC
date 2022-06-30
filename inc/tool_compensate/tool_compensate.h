/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2021/09/07
 *@brief 本头文件包含刀具半径补偿类的声明
 *@version
 */

#ifndef TOOL_COMPENSATE_H_
#define TOOL_COMPENSATE_H_

#include "tool_comp_data.h"
#include "tools_comp.h"
#include "list_buffer.h"
#include "compile_message.h"

class ChannelControl;


const int kCompTypes = 5;


//刀补状态
enum ToolCompensateState{
	COMP_NONE = 0,  //无刀补
	COMP_INIT,      //刀补建立
	COMP_RUN,       //刀补中
	COMP_CANCEL,    //刀补撤销
	COMP_CANCEL_PRE,  //刀补撤销前一段

};

//刀补插入数据类型
enum CompInsertType{
	INSERT_ARC = 0,   //插入圆弧
	INSERT_LINE       //插入直线
};

//刀补衔接类型
enum CompJoinType{
	JOIN_EXTEND = 0,      //延长型
	JOIN_SHORT			  //缩短型
};

class ToolCompensate {
public:
	ToolCompensate();		//构造函数
	virtual ~ToolCompensate();   //析构函数

	void SetOutputMsgList(OutputMsgList *output_msg){m_p_output_msg_list = output_msg;}  //设置指令消息输出缓冲区指针

	void ProcessData(ListNode<RecordMsg *> *node);    //数据处理入口函数

	void ResetAllDatas();      // 复位或初始化内部数据

	void SetCurPlane(int plane){m_cur_plane = (GCode)plane;}   //设置当前平面

	void SetChannelIndex(uint8_t chn_index);    //设置通道号


private:   //私有接口函数
	int CompensateOut();     //刀补处理函数

	void StateMachine();   //刀补状态机

private:
	uint8_t m_n_channel_index;     //所属通道号
	ChannelControl *m_p_channel_control;    //通道控制对象
	ToolCompMsgList m_n_toolcomp_list_msg;   //从编译模块输入的编译数据缓冲

	OutputMsgList *m_p_output_msg_list;   //待输出至MC的指令消息队列

//	uint8_t m_n_select[kCompTypes][kCompTypes];
//	ToolCompensateState m_n_comp_state;   	//0 --无刀补 ；1 --刀补建立；// 2 --刀补进行中  3 --刀补撤消 4--撤消前一段  其它 --异常，无刀补
//	CompInsertType m_n_comp_type;     		//0圆弧插入,1为直线插入
//	CompJoinType m_n_type_flag;                      	//1--缩短型    0----为非缩短型
//	double m_df_angle_pre,m_df_angle_after;
//	DataLine *m_vec_line1, *m_vec_line2,*m_vec_line3, *m_vec_line4;   //用于干涉检查的方向向量
//	DPoint   m_pos_target_old, m_pos_target, m_pos_target_new;

//  int16_t  m_cur_tool_index;   // 当前刀补号  D值
//  int8_t m_cur_tool_dir; // 0--G41左刀补  1--G42右刀补 
//  int8_t m_cur_work_plane; // 
//	double m_df_tool_radius;    //当前刀补半径  单位：mm
	GCode m_cur_plane;       //当前刀补平面  G17_CMD/G18_CMD/G19_CMD
	GCode m_cur_cmd;         //当前刀补类型  G40_CMD/G41_CMD/G42_CMD
	
	ToolRec m_cur_tool;
	CompensateTroop *m_p_ToolCompBuffer;
};

#endif /* TOOL_COMPENSATE_H_ */
