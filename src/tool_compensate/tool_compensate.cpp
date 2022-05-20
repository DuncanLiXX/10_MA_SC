/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2021/09/07
 *@brief ��ͷ�ļ��������߰뾶�������ʵ��
 *@version
 */

#include "channel_engine.h"
#include "channel_control.h"

#include "tool_compensate.h"


/**
 * @brief ���캯��
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
 * @brief ��������
 */
ToolCompensate::~ToolCompensate() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief ����ͨ����
 * @param chn_index : ͨ����
 */
void ToolCompensate::SetChannelIndex(uint8_t chn_index){
	this->m_n_channel_index = chn_index;

	this->m_p_channel_control = g_ptr_chn_engine->GetChnControl(chn_index);
}

/**
 * @brief ���ݴ�����ں���
 * @param node : �������ݽڵ�ָ��
 */
void ToolCompensate::ProcessData(ListNode<RecordMsg *> *node){
	this->m_list_msg.Append(node);   //��������

	this->StateMachine();     //״̬���л�

	this->Compensate();		  //�������㲢���
}

/**
 * @brief ����״̬��
 */
void ToolCompensate::StateMachine(){
	ListNode<RecordMsg *> *node = this->m_list_msg.TailNode();
	RecordMsg *msg = static_cast<RecordMsg *>(node->data);

	if(msg == nullptr)
		return;

	if(msg->GetMsgType() == COMPENSATE_MSG){
		CompensateMsg *tmp = (CompensateMsg *)msg;
		int gcode = tmp->GetGCode();
		if(gcode == G40_CMD){ //ȡ������
			this->m_cur_cmd = G40_CMD;

		}else if(gcode == G41_CMD || gcode == G42_CMD){  //��������
			this->m_cur_cmd = (GCode)gcode;

			int dd = tmp->GetCompValue();    //��ȡ����Dֵ

			this->m_df_tool_radius = this->m_p_channel_control->GetToolCompRadius(dd);   //��ȡ��ǰ�����뾶

		}
	}

}

/**
 * @brief ���������������������
 */
void ToolCompensate::Compensate(){
	//


	//���������
	ListNode<RecordMsg *> *node = nullptr;
	if(this->m_p_output_msg_list){
		node = this->m_list_msg.HeadNode();
		while(node != nullptr){
			this->m_list_msg.RemoveHead();
			this->m_p_output_msg_list->Append(node);   //ѹ������Ͷ���
	//		printf("ToolCompensate::Compensate(), line = %llu\n", node->data->GetLineNo());

			node = this->m_list_msg.HeadNode();  //ȡ��һ����Ϣ
		}
	}
}
