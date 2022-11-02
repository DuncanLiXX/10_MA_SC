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

#include "list_buffer.h"
#include "compile_message.h"

#define None 999999.999

class ChannelControl;

class ToolCompensate {
public:
	ToolCompensate();		//构造函数
	virtual ~ToolCompensate();   //析构函数

	void SetOutputMsgList(OutputMsgList *output_msg);  //设置指令消息输出缓冲区指针

	void ProcessData(ListNode<RecordMsg *> *node);    //数据处理入口函数

	void ResetAllDatas();      // 复位或初始化内部数据
	void SetChannelIndex(uint8_t chn_index);    //设置通道号

    //void setDMode(int Dvalue);
    void setToolRadius(double radius){comp_radius = radius;}

    void Reset();

private:
	uint8_t m_n_channel_index;     //所属通道号
	ChannelControl *m_p_channel_control;    //通道控制对象
	OutputMsgList *m_p_output_msg_list;   //待输出至MC的指令消息队列
	
	ToolRec m_cur_tool;

    DPointChn cur_point;
    RecordMsg * msg;
    double comp_radius;
};

#endif /* TOOL_COMPENSATE_H_ */
