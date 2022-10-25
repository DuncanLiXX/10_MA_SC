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
#include "tool_comp_data.h"
#include "tool_compensate.h"
#include "rs274.h"


/**
 * @brief 构造函数
 */
ToolCompensate::ToolCompensate() {
	// TODO Auto-generated constructor stub

	this->m_p_output_msg_list = nullptr;
	this->m_p_channel_control = nullptr;

	this->m_n_channel_index = 0;

	this->m_cur_tool.G41G42dir  = 40;
	this->m_cur_tool.D = 0;
	this->m_cur_tool.Radius = 0;
	this->m_cur_tool.plane = PLANE_XY;
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


void ToolCompensate::Reset(){}

/**
 * @brief 数据处理入口函数
 * @param node : 编译数据节点指针
 * 在 compile.runmsg 中已经处理了 G90 G91 不用考虑增量情况 所有计算都是绝对坐标值
 */
void ToolCompensate::ProcessData(ListNode<RecordMsg *> *node){

	msg = node->data;
    this->m_p_output_msg_list->Append(node);
}

void ToolCompensate::ResetAllDatas(){

}
