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
#include "tools_comp.h"
#include "tool_compensate.h"


/**
 * @brief ���캯��
 */
ToolCompensate::ToolCompensate() {
	// TODO Auto-generated constructor stub

	this->m_p_output_msg_list = nullptr;
	this->m_p_channel_control = nullptr;
	this->m_n_toolcomp_list_msg.Clear();

	this->m_n_channel_index = 0;

	this->m_cur_tool.G41G42dir  = 40;
	this->m_cur_tool.D = 0;
	this->m_cur_tool.Radius = 0;
	this->m_cur_tool.plane = PLANE_XY;

	this->m_cur_plane = G17_CMD;
	this->m_cur_cmd = G40_CMD;

	this->m_p_ToolCompBuffer = new CompensateTroop();  //new CompensateTroop(MAX_OFFSET_NUMBER);
	this->m_p_ToolCompBuffer->SetToollistbuffer(&m_n_toolcomp_list_msg);
}

/**
 * @brief ��������
 */
ToolCompensate::~ToolCompensate() {
	// TODO Auto-generated destructor stub
	
	this->m_n_toolcomp_list_msg.Clear();
	delete (this->m_p_ToolCompBuffer);
}

/**
 * @brief ��ʼ����λ�ڲ�����
 * @param chn_index : ͨ����
 */
void ToolCompensate::ResetAllDatas(){
	this->m_cur_tool.G41G42dir = 40;
	this->m_cur_tool.D = 0;
	this->m_cur_tool.Radius = 0;
	this->m_cur_tool.plane = PLANE_XY;
	
	this->m_cur_plane = G17_CMD;
	this->m_cur_cmd = G40_CMD;

	this->m_n_toolcomp_list_msg.Clear();

	this->m_p_ToolCompBuffer->ResetAllDatas();
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
	this->m_n_toolcomp_list_msg.Append(node);   //��������

	this->StateMachine();     //״̬���л�

    int outCounts = this->CompensateOut();		  //�������㲢���
    int list_num = this->m_n_toolcomp_list_msg.GetLength();
//	   printf("0-------->outCounts=%d  list_num=%d \n",outCounts,list_num);

	while(outCounts>0 && list_num>0)  // 
	{
       outCounts = this->CompensateOut();	//�˴���Ҫ�жϵ��������ﻺ��������Ƿ���Լ������
       list_num = this->m_n_toolcomp_list_msg.GetLength();
//	   printf("1-------->outCounts=%d  list_num=%d \n",outCounts,list_num);
	}
}

/**
 * @brief ����״̬��
 */
void ToolCompensate::StateMachine(){
	ListNode<RecordMsg *> *node = this->m_n_toolcomp_list_msg.TailNode();
	RecordMsg *msg = static_cast<RecordMsg *>(node->data);

	if(msg == nullptr)
		return;

	CodeMsgType MsgType = msg->GetMsgType();
	if(MsgType == COMPENSATE_MSG){
		CompensateMsg *tmp = (CompensateMsg *)msg;
		int gcode = tmp->GetGCode();
		if(gcode == G40_CMD){ //ȡ������
			this->m_cur_cmd = G40_CMD;
			this->m_cur_tool.G41G42dir = 40;
		    this->m_cur_tool.Radius = 0;
		    this->m_cur_tool.D = 0;

		}else if(gcode == G41_CMD || gcode == G42_CMD){  //��������
			this->m_cur_cmd = (GCode)gcode;

			int dd = tmp->GetCompValue();    //��ȡ����Dֵ
			this->m_cur_tool.D = dd;
			this->m_cur_tool.Radius = this->m_p_channel_control->GetToolCompRadius(dd);   //��ȡ��ǰ�����뾶
			
//			printf("$$$$$$$$$this->m_n_channel_index = %d\n",this->m_n_channel_index);
			
//			printf("$$$$$$$$$this->m_cur_tool.D = %d Radius=%lf \n",this->m_cur_tool.D,this->m_cur_tool.Radius);

			if(gcode == G41_CMD)
			{
                this->m_cur_tool.G41G42dir = 41;
			}
			else
			{
                this->m_cur_tool.G41G42dir = 42;
			}

			if(this->m_p_ToolCompBuffer != nullptr) this->m_p_ToolCompBuffer->setStartCompState();
		}
	}
	else if(MsgType == MODE_MSG){
		ModeMsg *tmp = (ModeMsg *)msg;
		int gcode = tmp->GetGCode();
		if(gcode == G17_CMD){ 
			this->m_cur_plane = G17_CMD;
			this->m_cur_tool.plane = PLANE_XY;			
		}else if(gcode == G18_CMD){  //��������
			this->m_cur_plane = G18_CMD;
			this->m_cur_tool.plane = PLANE_ZX;   //��ȡ��ǰ�����뾶
		}else if(gcode == G19_CMD){  //��������
			this->m_cur_plane  = G19_CMD;
			this->m_cur_tool.plane = PLANE_YZ;   //��ȡ��ǰ�����뾶
		}
	}	
	else if(MsgType == LINE_MSG || MsgType == RAPID_MSG || MsgType == ARC_MSG  )
	{
		ModeMsg *tmp = (ModeMsg *)msg;
		tmp->SetAToolRec(this->m_cur_tool);
//		printf("$$$$$$$$$  add toolradious %f\n",this->m_cur_tool.Radius);
	}
}

/**
 * @brief ���������������������
 */
int ToolCompensate::CompensateOut(){
	//
   int nums = m_p_ToolCompBuffer->Compensate();
   int retcnt = nums;
   
//   printf("------------->>>>>>> out nums = %d  ToolCompensateState=%d \n",nums,(int)(m_p_ToolCompBuffer->GetToolCompensateState()));
   
   if(nums>0)
   {	//���������
		ListNode<RecordMsg *> *node = nullptr;
		if(this->m_p_output_msg_list){
			while(nums--){
			    node = this->m_n_toolcomp_list_msg.HeadNode();  //ȡ��һ����Ϣ
				if(node != nullptr)
				{
					this->m_n_toolcomp_list_msg.RemoveHead();
					this->m_p_output_msg_list->Append(node);   //ѹ������Ͷ���													
				}
			}
		}
   }

   return retcnt;
}
