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

void ToolCompensate::SetOutputMsgList(OutputMsgList *output_msg){
	m_p_output_msg_list = output_msg;
	setOutputMsgList(output_msg);
}

/**
 * @brief 设置通道号
 * @param chn_index : 通道号
 */
void ToolCompensate::SetChannelIndex(uint8_t chn_index){
	this->m_n_channel_index = chn_index;
	this->m_p_channel_control = g_ptr_chn_engine->GetChnControl(chn_index);
}

void ToolCompensate::clearError(){
	err_code = ERR_NONE;
	interp.err_code = ERR_NONE;
}


void ToolCompensate::Reset(){
	interp.reset();
}

/**
 * @brief 数据处理入口函数
 * @param node : 编译数据节点指针
 * 在 compile.runmsg 中已经处理了 G90 G91 不用考虑增量情况 所有计算都是绝对坐标值
 */
void ToolCompensate::ProcessData(ListNode<RecordMsg *> *node){

	msg = node->data;


	switch(msg->GetMsgType()){
		case RAPID_MSG:{
			RapidMsg * tmsg = (RapidMsg *)msg;
			block * pblock = interp.interp_block();
			interp.init_block(pblock);
			pblock->x_flag = true;
			pblock->y_flag = true;
			pblock->z_flag = true;
			pblock->x_number = tmsg->GetTargetPos().GetAxisValue(0);
			pblock->y_number = tmsg->GetTargetPos().GetAxisValue(1);
			pblock->z_number = tmsg->GetTargetPos().GetAxisValue(2);
			pblock->line_number = tmsg->GetLineNo();
			pblock->flags = tmsg->GetFlags().all;

			pblock->a_number = tmsg->GetTargetPos().GetAxisValue(3);
			pblock->b_number = tmsg->GetTargetPos().GetAxisValue(4);
			pblock->c_number = tmsg->GetTargetPos().GetAxisValue(5);
			pblock->u_number = tmsg->GetTargetPos().GetAxisValue(6);
			pblock->v_number = tmsg->GetTargetPos().GetAxisValue(7);

			printf("convert rapid lino:%llu ... \n", tmsg->GetLineNo());
			interp.convert_straight(0, &interp._setup._block, &interp._setup);
			break;
		}
		case LINE_MSG:{
			LineMsg * tmsg = (LineMsg *) msg;
			block * pblock = interp.interp_block();
			interp.init_block(pblock);
			pblock->x_flag = true;
			pblock->y_flag = true;
			pblock->z_flag = true;
			pblock->x_number = tmsg->GetTargetPos().GetAxisValue(0);
			pblock->y_number = tmsg->GetTargetPos().GetAxisValue(1);
			pblock->z_number = tmsg->GetTargetPos().GetAxisValue(2);

			pblock->a_number = tmsg->GetTargetPos().GetAxisValue(3);
			pblock->b_number = tmsg->GetTargetPos().GetAxisValue(4);
			pblock->c_number = tmsg->GetTargetPos().GetAxisValue(5);
			pblock->u_number = tmsg->GetTargetPos().GetAxisValue(6);
			pblock->v_number = tmsg->GetTargetPos().GetAxisValue(7);

			pblock->flags = tmsg->GetFlags().all;
			pblock->f_number = tmsg->GetFeed();
			pblock->line_number = tmsg->GetLineNo();
			printf("convert line lino:%llu... \n", tmsg->GetLineNo());
			interp.convert_straight(10, &interp._setup._block, &interp._setup);
			break;
		}
		case ARC_MSG:{
			ArcMsg * tmsg = (ArcMsg *) msg;
			block * pblock = interp.interp_block();
			interp.init_block(pblock);
			pblock->x_flag = true;
			pblock->y_flag = true;
			pblock->z_flag = true;
			pblock->x_number = tmsg->GetTargetPos().GetAxisValue(0);
			pblock->y_number = tmsg->GetTargetPos().GetAxisValue(1);
			pblock->z_number = tmsg->GetTargetPos().GetAxisValue(2);

			pblock->a_number = tmsg->GetTargetPos().GetAxisValue(3);
			pblock->b_number = tmsg->GetTargetPos().GetAxisValue(4);
			pblock->c_number = tmsg->GetTargetPos().GetAxisValue(5);
			pblock->u_number = tmsg->GetTargetPos().GetAxisValue(6);
			pblock->v_number = tmsg->GetTargetPos().GetAxisValue(7);

			pblock->flags = tmsg->GetFlags().all;
			pblock->f_number = tmsg->GetFeed();
			pblock->line_number = tmsg->GetLineNo();

			if(tmsg->getR() != 0){
				pblock->r_flag = true;
				pblock->r_number = tmsg->getR();
			}else{
				pblock->i_flag = true;
				pblock->j_flag = true;
				pblock->i_number = tmsg->getI();
				pblock->j_number = tmsg->getJ();
			}

			printf("convert arc lino: %llu... \n", tmsg->GetLineNo());
			if(tmsg->getDirect() == -1){
				interp.convert_arc(20, &interp._setup._block, &interp._setup);
			}else{
				interp.convert_arc(30, &interp._setup._block, &interp._setup);
			}

			break;
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

			break;
		}
		default:{

			if(interp.isCompOn){
				uint16_t flags = msg->GetFlags().all;

				// 遇到 M30 或 等待移动到位标志  压空刀补队列 完成运动
				if((flags & FLAG_BLOCK_OVER) or (flags & FLAG_WAIT_MOVE_OVER) or (flags & FLAG_EOF))
				{
					interp.flush_comp();
					// 程序结束 关闭刀补
					if(flags & FLAG_EOF) interp.convert_cutter_compensation_off(&interp._setup);
				}
			}

			this->m_p_output_msg_list->Append(node);
		}
	}

	if(interp.err_code != 0){
		err_code = interp.err_code;
	}

	//this->m_p_output_msg_list->Append(node);

}

void ToolCompensate::ResetAllDatas(){

}
