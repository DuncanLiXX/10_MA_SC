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
#include "tool_comp_data.h"
#include "tool_compensate.h"
#include "rs274.h"


/**
 * @brief ���캯��
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


void ToolCompensate::Reset(){}

/**
 * @brief ���ݴ�����ں���
 * @param node : �������ݽڵ�ָ��
 * �� compile.runmsg ���Ѿ������� G90 G91 ���ÿ���������� ���м��㶼�Ǿ�������ֵ
 */
void ToolCompensate::ProcessData(ListNode<RecordMsg *> *node){

	msg = node->data;
    this->m_p_output_msg_list->Append(node);
}

void ToolCompensate::ResetAllDatas(){

}
