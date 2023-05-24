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
	clearError();
}

void ToolCompensate::setCurAxisPos(int axis, double pos){
	interp.setCurAxisPos(axis, pos);
}

/**
 * @brief 数据处理入口函数
 * @param node : 编译数据节点指针
 * 在 compile.runmsg 中已经处理了 G90 G91 不用考虑增量情况 所有计算都是绝对坐标值
 */
void ToolCompensate::ProcessData(ListNode<RecordMsg *> *node){

	msg = node->data;

	// 限制刀补平面
	if(this->plane != 170){

		if(interp.isCompOn) interp.reset();

		if(msg->GetMsgType() == COMPENSATE_MSG){
			CompensateMsg * tmsg = (CompensateMsg *)msg;
			if(tmsg->GetGCode() == G41_CMD || tmsg->GetGCode() == G42_CMD){
				this->err_code = INVALID_COMPENSATE_PLANE;
				return;
			}
		}
		// 非 G17 平面不进行刀补处理
		this->m_p_output_msg_list->Append(node);
		return;
	}

	CodeMsgType msg_type = msg->GetMsgType();
	uint32_t line_number = msg->GetLineNo();

	switch(msg_type){
		case RAPID_MSG:{

			RapidMsg * tmsg = (RapidMsg *)msg;
            if(tmsg->GetPmcAxisCount() > 0){
                this->m_p_output_msg_list->Append(node);
                break;
            }
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
            delete msg;
            delete node;
			break;
		}
		case LINE_MSG:{

			LineMsg * tmsg = (LineMsg *) msg;
            if(tmsg->GetPmcAxisCount() > 0){
                this->m_p_output_msg_list->Append(node);
                break;
            }
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
            delete msg;
            delete node;
			break;
		}
		case ARC_MSG:{
			ArcMsg * tmsg = (ArcMsg *) msg;
            if(tmsg->GetPmcAxisCount() > 0){
            	this->m_p_output_msg_list->Append(node);
                break;
            }
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

            delete msg;
            delete node;
			break;
		}
		case COMPENSATE_MSG:{
			CompensateMsg * tmsg = (CompensateMsg *)msg;

			interp.flush_comp();

			if(tmsg->GetGCode() == G40_CMD){
				comp_cancel_flag = false;
				comp_side = NOCOMP;
				interp.convert_cutter_compensation_off(&interp._setup);
			}else if(tmsg->GetGCode() == G41_CMD){

				if(comp_radius < 0.0){
					comp_side = RIGHT;
					interp.convert_cutter_compensation_on(comp_side,fabs(this->comp_radius),&interp._setup);
				}else{
					comp_side = LEFT;
					interp.convert_cutter_compensation_on(comp_side,this->comp_radius,&interp._setup);
				}



			}else if(tmsg->GetGCode() == G42_CMD){

				if(comp_radius < 0.0){
					comp_side = LEFT;
					interp.convert_cutter_compensation_on(comp_side,fabs(this->comp_radius),&interp._setup);
				}else{
					comp_side = RIGHT;
					interp.convert_cutter_compensation_on(comp_side,this->comp_radius,&interp._setup);
				}
			}
			//RecordMsgFlag flags;
			//flags.all = 9;
			//tmsg->SetFlags(flags);

			this->m_p_output_msg_list->Append(node);

			break;
		}
		case COORD_MSG:{
			CoordMsg *coord_msg = (CoordMsg *)msg;
			int gcode = coord_msg->GetGCode();
			printf("gcode %d\n", gcode);
			// @add zk 更新刀补起始位置
			if(gcode == G53_CMD){
				DPointChn point = coord_msg->GetTargetPos();
				for(int i=0; i<8; i++){
					interp.setCurAxisPos(i, point.m_df_point[i]);
				}
			}

			if(interp.isCompOn){
				interp.convert_close_compensation(&interp._setup);
				comp_cancel_flag = true;
			}

			this->m_p_output_msg_list->Append(node);
			break;
		}
		default:{

			uint16_t flags = msg->GetFlags().all;

			if(flags & FLAG_EOF){
				interp.reset();
				comp_cancel_flag = false;
			}

			if(interp.isCompOn){

				if(msg_type == SKIP_MSG){
					err_code = G31_NOT_ALLOWED;
					return;
				}

				// 遇到 M30 或 等待移动到位标志  压空刀补队列 完成运动
				if((flags & FLAG_BLOCK_OVER) or (flags & FLAG_WAIT_MOVE_OVER) or (flags & FLAG_EOF))
				{
					// 程序结束 关闭刀补
					interp.flush_comp();
					comp_cancel_flag = false;
				}

				if(msg_type == REF_RETURN_MSG){
					interp.convert_close_compensation(&interp._setup);
				}

				if(msg_type == LOOP_MSG){
					LoopMsg *tmsg = (LoopMsg *)msg;
					if(tmsg->GetGCode() != G80_CMD){
						interp.convert_close_compensation(&interp._setup);
						comp_cancel_flag = true;
					}
				}

				if(msg_type == RESTART_OVER_MSG||msg_type == AUTO_TOOL_MEASURE_MSG){
					interp.convert_close_compensation(&interp._setup);
				}
			}

			if(msg_type == LOOP_MSG){
				LoopMsg *tmsg = (LoopMsg *)msg;

				if(tmsg->GetGCode() == G80_CMD){
					if(comp_cancel_flag && (comp_side != NOCOMP)){
						interp.convert_cutter_compensation_on(comp_side, fabs(this->comp_radius), &interp._setup);
						comp_cancel_flag = false;
					}
				}
			}

			this->m_p_output_msg_list->Append(node);
		}
	}

	if(interp.err_code != 0){
		err_code = interp.err_code;
		err_lino = line_number;
	}
}

void ToolCompensate::ResetAllDatas(){

}
