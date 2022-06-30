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
#include "tools_comp.h"
#include "tool_compensate.h"


/**
 * @brief 构造函数
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
 * @brief 析构函数
 */
ToolCompensate::~ToolCompensate() {
	// TODO Auto-generated destructor stub
	
	this->m_n_toolcomp_list_msg.Clear();
	delete (this->m_p_ToolCompBuffer);
}

/**
 * @brief 初始化或复位内部数据
 * @param chn_index : 通道号
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
 * @brief 设置通道号
 * @param chn_index : 通道号
 */
void ToolCompensate::SetChannelIndex(uint8_t chn_index){
	this->m_n_channel_index = chn_index;

	this->m_p_channel_control = g_ptr_chn_engine->GetChnControl(chn_index);
}

/**
 * @brief 数据处理入口函数
 * @param node : 编译数据节点指针
 */
void ToolCompensate::ProcessData(ListNode<RecordMsg *> *node){
	this->m_n_toolcomp_list_msg.Append(node);   //插入数据

	this->StateMachine();     //状态机切换

    int outCounts = this->CompensateOut();		  //刀补运算并输出
    int list_num = this->m_n_toolcomp_list_msg.GetLength();
//	   printf("0-------->outCounts=%d  list_num=%d \n",outCounts,list_num);

	while(outCounts>0 && list_num>0)  // 
	{
       outCounts = this->CompensateOut();	//此处主要判断刀补队列里缓冲的数据是否可以继续输出
       list_num = this->m_n_toolcomp_list_msg.GetLength();
//	   printf("1-------->outCounts=%d  list_num=%d \n",outCounts,list_num);
	}
}

/**
 * @brief 刀补状态机
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
		if(gcode == G40_CMD){ //取消刀补
			this->m_cur_cmd = G40_CMD;
			this->m_cur_tool.G41G42dir = 40;
		    this->m_cur_tool.Radius = 0;
		    this->m_cur_tool.D = 0;

		}else if(gcode == G41_CMD || gcode == G42_CMD){  //建立刀补
			this->m_cur_cmd = (GCode)gcode;

			int dd = tmp->GetCompValue();    //获取刀补D值
			this->m_cur_tool.D = dd;
			this->m_cur_tool.Radius = this->m_p_channel_control->GetToolCompRadius(dd);   //获取当前刀补半径
			
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
		}else if(gcode == G18_CMD){  //建立刀补
			this->m_cur_plane = G18_CMD;
			this->m_cur_tool.plane = PLANE_ZX;   //获取当前刀补半径
		}else if(gcode == G19_CMD){  //建立刀补
			this->m_cur_plane  = G19_CMD;
			this->m_cur_tool.plane = PLANE_YZ;   //获取当前刀补半径
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
 * @brief 刀补处理函数，并输出数据
 */
int ToolCompensate::CompensateOut(){
	//
   int nums = m_p_ToolCompBuffer->Compensate();
   int retcnt = nums;
   
//   printf("------------->>>>>>> out nums = %d  ToolCompensateState=%d \n",nums,(int)(m_p_ToolCompBuffer->GetToolCompensateState()));
   
   if(nums>0)
   {	//输出将数据
		ListNode<RecordMsg *> *node = nullptr;
		if(this->m_p_output_msg_list){
			while(nums--){
			    node = this->m_n_toolcomp_list_msg.HeadNode();  //取下一个消息
				if(node != nullptr)
				{
					this->m_n_toolcomp_list_msg.RemoveHead();
					this->m_p_output_msg_list->Append(node);   //压入待发送队列													
				}
			}
		}
   }

   return retcnt;
}
