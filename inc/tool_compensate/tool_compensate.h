/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2021/09/07
 *@brief ��ͷ�ļ��������߰뾶�����������
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
	ToolCompensate();		//���캯��
	virtual ~ToolCompensate();   //��������

	void SetOutputMsgList(OutputMsgList *output_msg);  //����ָ����Ϣ���������ָ��

	void ProcessData(ListNode<RecordMsg *> *node);    //���ݴ�����ں���

	void ResetAllDatas();      // ��λ���ʼ���ڲ�����
	void SetChannelIndex(uint8_t chn_index);    //����ͨ����

    //void setDMode(int Dvalue);
    void setToolRadius(double radius){comp_radius = radius;}

    void Reset();

private:
	uint8_t m_n_channel_index;     //����ͨ����
	ChannelControl *m_p_channel_control;    //ͨ�����ƶ���
	OutputMsgList *m_p_output_msg_list;   //�������MC��ָ����Ϣ����
	
	ToolRec m_cur_tool;

    DPointChn cur_point;
    RecordMsg * msg;
    double comp_radius;
};

#endif /* TOOL_COMPENSATE_H_ */
