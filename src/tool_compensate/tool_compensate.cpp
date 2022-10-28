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


Interp interp;

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

	/*switch(msg->GetMsgType()){
		case LINE_MSG:{  // G01
			LineMsg * tmsg = (LineMsg *)msg;

			ADRSREG_DEF block;
			memset(&block, 0, sizeof(block));

			block.GA.All = TGCDROM[1]; // G模态组
			// 位置 x y z
			block.A1 = tmsg->GetTargetPos().GetAxisValue(0)*1000; // 转换未 um 整数
			block.A2 = tmsg->GetTargetPos().GetAxisValue(1)*1000;
			block.A3 = tmsg->GetTargetPos().GetAxisValue(2)*1000;
			// x y z 输入标志
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

			block.GA.All = TGCDROM[0]; // G模态组
			// 位置 x y z
			block.A1 = tmsg->GetTargetPos().GetAxisValue(0)*1000; // 转换未 um 整数
			block.A2 = tmsg->GetTargetPos().GetAxisValue(1)*1000;
			block.A3 = tmsg->GetTargetPos().GetAxisValue(2)*1000;
			// x y z 输入标志
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
				block.GA.All = TGCDROM[2]; // G模态组
			}else{
				block.GA.All = TGCDROM[3]; // G模态组
			}

			// 位置 x y z
			block.A1 = tmsg->GetTargetPos().GetAxisValue(0)*1000; // 转换未 um 整数
			block.A2 = tmsg->GetTargetPos().GetAxisValue(1)*1000;
			block.A3 = tmsg->GetTargetPos().GetAxisValue(2)*1000;
			// x y z 输入标志
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
