/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2021/09/07
 *@brief 本头文件包含刀具半径补偿类的实现
 *@version
 */

#include "channel_engine.h"
#include "channel_control.h"

#include "tool_compensate.h"


/**
 * @brief 构造函数
 */
ToolCompensate::ToolCompensate() {
	// TODO Auto-generated constructor stub

	this->m_p_output_msg_list = nullptr;
	this->m_p_channel_control = nullptr;
	this->m_list_msg.Clear();

	this->m_n_channel_index = 0;

	this->m_df_tool_radius = 0;
	m_cur_plane = G17_CMD;
	this->m_cur_cmd = G40_CMD;
}

/**
 * @brief 析构函数
 */
ToolCompensate::~ToolCompensate() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief 设置通道号
 * @param chn_index : 通道号
 */
void ToolCompensate::SetChannelIndex(uint8_t chn_index){
	this->m_n_channel_index = chn_index;

	this->m_p_channel_control = g_ptr_chn_engine->GetChnControl(chn_index);
}

/**
 * @brief 数据处理入口函数
 * @param node : 编译数据节点指针
 */
void ToolCompensate::ProcessData(ListNode<RecordMsg *> *node){
	this->m_list_msg.Append(node);   //插入数据

	this->StateMachine();     //状态机切换

	this->Compensate();		  //刀补运算并输出
}

/**
 * @brief 刀补状态机
 */
void ToolCompensate::StateMachine(){
	ListNode<RecordMsg *> *node = this->m_list_msg.TailNode();
	RecordMsg *msg = static_cast<RecordMsg *>(node->data);

	if(msg == nullptr)
		return;

	if(msg->GetMsgType() == COMPENSATE_MSG){
		CompensateMsg *tmp = (CompensateMsg *)msg;
		int gcode = tmp->GetGCode();
		if(gcode == G40_CMD){ //取消刀补
			this->m_cur_cmd = G40_CMD;

		}else if(gcode == G41_CMD || gcode == G42_CMD){  //建立刀补
			this->m_cur_cmd = (GCode)gcode;

			int dd = tmp->GetCompValue();    //获取刀补D值

			this->m_df_tool_radius = this->m_p_channel_control->GetToolCompRadius(dd);   //获取当前刀补半径

		}
	}

}

/**
 * @brief 刀补处理函数，并输出数据
 */
void ToolCompensate::Compensate(){
	//


	//输出将数据
	ListNode<RecordMsg *> *node = nullptr;
	if(this->m_p_output_msg_list){
		node = this->m_list_msg.HeadNode();
		while(node != nullptr){
			this->m_list_msg.RemoveHead();
			this->m_p_output_msg_list->Append(node);   //压入待发送队列
	//		printf("ToolCompensate::Compensate(), line = %llu\n", node->data->GetLineNo());

			node = this->m_list_msg.HeadNode();  //取下一个消息
		}
	}
}
