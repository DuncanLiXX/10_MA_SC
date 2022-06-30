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

#include "tool_comp_data.h"
#include "tools_comp.h"
#include "list_buffer.h"
#include "compile_message.h"

class ChannelControl;


const int kCompTypes = 5;


//����״̬
enum ToolCompensateState{
	COMP_NONE = 0,  //�޵���
	COMP_INIT,      //��������
	COMP_RUN,       //������
	COMP_CANCEL,    //��������
	COMP_CANCEL_PRE,  //��������ǰһ��

};

//����������������
enum CompInsertType{
	INSERT_ARC = 0,   //����Բ��
	INSERT_LINE       //����ֱ��
};

//�����ν�����
enum CompJoinType{
	JOIN_EXTEND = 0,      //�ӳ���
	JOIN_SHORT			  //������
};

class ToolCompensate {
public:
	ToolCompensate();		//���캯��
	virtual ~ToolCompensate();   //��������

	void SetOutputMsgList(OutputMsgList *output_msg){m_p_output_msg_list = output_msg;}  //����ָ����Ϣ���������ָ��

	void ProcessData(ListNode<RecordMsg *> *node);    //���ݴ�����ں���

	void ResetAllDatas();      // ��λ���ʼ���ڲ�����

	void SetCurPlane(int plane){m_cur_plane = (GCode)plane;}   //���õ�ǰƽ��

	void SetChannelIndex(uint8_t chn_index);    //����ͨ����


private:   //˽�нӿں���
	int CompensateOut();     //����������

	void StateMachine();   //����״̬��

private:
	uint8_t m_n_channel_index;     //����ͨ����
	ChannelControl *m_p_channel_control;    //ͨ�����ƶ���
	ToolCompMsgList m_n_toolcomp_list_msg;   //�ӱ���ģ������ı������ݻ���

	OutputMsgList *m_p_output_msg_list;   //�������MC��ָ����Ϣ����

//	uint8_t m_n_select[kCompTypes][kCompTypes];
//	ToolCompensateState m_n_comp_state;   	//0 --�޵��� ��1 --����������// 2 --����������  3 --�������� 4--����ǰһ��  ���� --�쳣���޵���
//	CompInsertType m_n_comp_type;     		//0Բ������,1Ϊֱ�߲���
//	CompJoinType m_n_type_flag;                      	//1--������    0----Ϊ��������
//	double m_df_angle_pre,m_df_angle_after;
//	DataLine *m_vec_line1, *m_vec_line2,*m_vec_line3, *m_vec_line4;   //���ڸ�����ķ�������
//	DPoint   m_pos_target_old, m_pos_target, m_pos_target_new;

//  int16_t  m_cur_tool_index;   // ��ǰ������  Dֵ
//  int8_t m_cur_tool_dir; // 0--G41�󵶲�  1--G42�ҵ��� 
//  int8_t m_cur_work_plane; // 
//	double m_df_tool_radius;    //��ǰ�����뾶  ��λ��mm
	GCode m_cur_plane;       //��ǰ����ƽ��  G17_CMD/G18_CMD/G19_CMD
	GCode m_cur_cmd;         //��ǰ��������  G40_CMD/G41_CMD/G42_CMD
	
	ToolRec m_cur_tool;
	CompensateTroop *m_p_ToolCompBuffer;
};

#endif /* TOOL_COMPENSATE_H_ */
