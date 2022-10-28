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


Interp interp;

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

	/*switch(msg->GetMsgType()){
		case LINE_MSG:{  // G01
			LineMsg * tmsg = (LineMsg *)msg;

			ADRSREG_DEF block;
			memset(&block, 0, sizeof(block));

			block.GA.All = TGCDROM[1]; // Gģ̬��
			// λ�� x y z
			block.A1 = tmsg->GetTargetPos().GetAxisValue(0)*1000; // ת��δ um ����
			block.A2 = tmsg->GetTargetPos().GetAxisValue(1)*1000;
			block.A3 = tmsg->GetTargetPos().GetAxisValue(2)*1000;
			// x y z �����־
			block.FA1.inp = 1;
			block.FA2.inp = 1;
			block.FA3.inp = 1;

			block.F = tmsg->GetFeed();
			block.FF.inp = 1;
			block.line_number = tmsg->GetLineNo();
			pushComp(block);

			break;
		}
		case RAPID_MSG:{ // G00
			RapidMsg * tmsg = (RapidMsg *)msg;

			ADRSREG_DEF block;
			memset(&block, 0, sizeof(block));

			block.GA.All = TGCDROM[0]; // Gģ̬��
			// λ�� x y z
			block.A1 = tmsg->GetTargetPos().GetAxisValue(0)*1000; // ת��δ um ����
			block.A2 = tmsg->GetTargetPos().GetAxisValue(1)*1000;
			block.A3 = tmsg->GetTargetPos().GetAxisValue(2)*1000;
			// x y z �����־
			block.FA1.inp = 1;
			block.FA2.inp = 1;
			block.FA3.inp = 1;
			block.line_number = tmsg->GetLineNo();
			pushComp(block);
			break;
		}
		case ARC_MSG:{   // G02 G03
			ArcMsg * tmsg = (ArcMsg *)msg;

			ADRSREG_DEF block;
			memset(&block, 0, sizeof(block));

			if(tmsg->GetGCode() == 20){
				block.GA.All = TGCDROM[2]; // Gģ̬��
			}else{
				block.GA.All = TGCDROM[3]; // Gģ̬��
			}

			// λ�� x y z
			block.A1 = tmsg->GetTargetPos().GetAxisValue(0)*1000; // ת��δ um ����
			block.A2 = tmsg->GetTargetPos().GetAxisValue(1)*1000;
			block.A3 = tmsg->GetTargetPos().GetAxisValue(2)*1000;
			// x y z �����־
			block.FA1.inp = 1;
			block.FA2.inp = 1;
			block.FA3.inp = 1;

			// IJR
			if(tmsg->getR() != 0){
				block.R = tmsg->getR();
				block.FR.inp = 1;
			}else{
				block.I = tmsg->getI();
				block.J = tmsg->getJ();
				block.FI.inp = 1;
				block.FJ.inp = 1;
			}

			block.F = tmsg->GetFeed();
			block.FF.inp = 1;
			block.line_number = tmsg->GetLineNo();
			pushComp(block);
			break;
		}
		case COMPENSATE_MSG: // G40 G41 G42
		{
			CompensateMsg *tmsg = (CompensateMsg *) msg;

			ADRSREG_DEF block;
			memset(&block, 0, sizeof(block));
			block.GG.All = TGCDROM[tmsg->GetGCode()/10];
			block.line_number = tmsg->GetLineNo();
			pushComp(block);
			break;
		}
	}*/



	switch(msg->GetMsgType()){
		case LINE_MSG:
		case RAPID_MSG:
		case ARC_MSG:
			interp.init_block(&interp._setup._block);
			break;
		case COMPENSATE_MSG:
		{
			break;
		}
	}

	switch(msg->GetMsgType()){
		case LINE_MSG:{
			LineMsg * tmsg = (LineMsg *) msg;
			block * pblock = interp.interp_block();
			pblock->x_flag = true;
			pblock->y_flag = true;
			pblock->z_flag = true;
		}
		case RAPID_MSG:{
			RapidMsg * tmsg = (RapidMsg *)msg;
		}
		case ARC_MSG:{
			ArcMsg * tmsg = (ArcMsg *) msg;
		}
		case COMPENSATE_MSG:{
			CompensateMsg * tmsg = (CompensateMsg *)msg;

			printf("compensate gcode %d\n", tmsg->GetGCode());

			if(tmsg->GetGCode() == G40_CMD){

				interp.convert_cutter_compensation_off(&interp._setup);

			}else if(tmsg->GetGCode() == G41_CMD){

				interp.convert_cutter_compensation_on(LEFT,this->comp_radius,&interp._setup);

			}else if(tmsg->GetGCode() == G42_CMD){

				interp.convert_cutter_compensation_on(RIGHT,this->comp_radius,&interp._setup);
			}
		}
		default:{

		}
	}


    this->m_p_output_msg_list->Append(node);

}

void ToolCompensate::ResetAllDatas(){

}
