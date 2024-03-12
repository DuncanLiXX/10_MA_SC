/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file Compiler_data.h
 *@author gonghao
 *@date 2020/04/13
 *@brief 本头文件定义G代码编译器生成的数据消息类InfoMessage
 *@version
 */

#include "compile_message.h"

//template<> int ListNode<RecordMsg *>::new_count = 0;

/*****************************************************************RecordMsg类***************************************************************/

/**
 * @brief 构造函数
 */
RecordMsg::RecordMsg()/*:m_p_next(nullptr), m_p_prev(nullptr)*/{
	this->m_n_type = NORMAL_MSG;
	this->m_n_line_no = 0;
	this->m_n_flags.all = 0;
	this->m_n_frame_index = 0;
    Singleton<ShowSc>::instance().NewMsg();
}

/**
 * @brief 析构函数
 */
RecordMsg::~RecordMsg(){
    Singleton<ShowSc>::instance().DeleteMsg();
}

/**
 * @brief 设置对应标志的状态
 * @param flag: 指定设置的标志类型
 * @param value：设定的值
 */
void RecordMsg::SetFlag(RecordFlag flag, bool value){
	if(value)
		m_n_flags.all |= flag;
	else
		m_n_flags.all &= (~flag);
}

/**
 * @brief 在当前消息后插入新的消息
 * @param msg : 待插入的新消息对象指针
 */
//void RecordMsg::AppendMsg(RecordMsg *msg){
//	if(msg == nullptr)
//		return;
//	msg->m_p_prev = this;
//	msg->m_p_next = m_p_next;
//	if(m_p_next != nullptr){
//		m_p_next->m_p_prev = msg;
//	}
//
//	m_p_next = msg;
//
//}

/**
 * @brief 将自身移出队列
 */
//void RecordMsg::RemoveSelf(){
//	if(m_p_prev != nullptr){
//		m_p_prev->m_p_next = m_p_next;
//	}
//	if(m_p_next != nullptr){
//		m_p_next->m_p_prev = m_p_prev;
//	}
//	m_p_next = nullptr;
//	m_p_prev = nullptr;
//}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
RecordMsg& RecordMsg::operator=( const RecordMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const RecordMsg &one, RecordMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all)
		return true;
	return false;
}

/*****************************************************************ErrorMsg类***************************************************************/

ErrorMsg::ErrorMsg(int err_code){
	m_error_code = err_code;
//	this->m_b_clear_info = false;
	this->m_n_info_type = 1;
	SetMsgType(ERROR_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
}

ErrorMsg::ErrorMsg(){
	m_error_code = ERR_NONE;
//	this->m_b_clear_info = false;
	this->m_n_info_type = 1;
	SetMsgType(ERROR_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
}


void ErrorMsg::Execute(){

}

void ErrorMsg::GetData(void *rec){

}


void ErrorMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int ErrorMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void ErrorMsg::PrintString(){
	printf("[%lld]ErrorMsg[%d]\n", m_n_line_no, m_error_code);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
ErrorMsg& ErrorMsg::operator=( const ErrorMsg& msg){
	if(&msg == this)
		return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_error_code = msg.m_error_code;
//	this->m_b_clear_info = msg.m_b_clear_info;
	this->m_n_info_type = msg.m_n_info_type;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const ErrorMsg &one, ErrorMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_error_code == two.m_error_code &&
//			one.m_b_clear_info == two.m_b_clear_info &&
			one.m_n_info_type == two.m_n_info_type)
		return true;
	return false;
}

/*****************************************************************RestartOverMsg类***************************************************************/

RestartOverMsg::RestartOverMsg(){

	SetMsgType(RESTART_OVER_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
	this->SetFlag(FLAG_STEP, false);    //单段不停
}



void RestartOverMsg::Execute(){

}

void RestartOverMsg::GetData(void *rec){

}


void RestartOverMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int RestartOverMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void RestartOverMsg::PrintString(){
	printf("[%lld]RestartOverMsg\n", m_n_line_no);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
RestartOverMsg& RestartOverMsg::operator=( const RestartOverMsg& msg){
	if(&msg == this)
		return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;

	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const RestartOverMsg &one, RestartOverMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all)
		return true;
	return false;
}

/*****************************************************************ModeMsg类***************************************************************/
ModeMsg::ModeMsg(int gcode){
	this->m_n_g_code = gcode;
	this->m_n_last_g_code = gcode;
	SetMsgType(MODE_MSG);
}

void ModeMsg::Execute(){

}

void ModeMsg::GetData(void *rec){

}

void ModeMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int ModeMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void ModeMsg::PrintString(){
	printf("[%lld]ModeMsg[%d]\n", m_n_line_no, m_n_g_code);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
ModeMsg& ModeMsg::operator=( const ModeMsg& msg){
	if(&msg == this)
		return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_g_code = msg.m_n_g_code;
	this->m_n_last_g_code = msg.m_n_last_g_code;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const ModeMsg &one, ModeMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_g_code == two.m_n_g_code &&
			one.m_n_last_g_code == two.m_n_last_g_code)
		return true;
	return false;
}


/*****************************************************************MoveMsg类***************************************************************/
MoveMsg::MoveMsg(int gcode):ModeMsg(gcode){
	this->m_b_mach_coord = false;   //默认为工件坐标系
	SetMsgType(MOVE_MSG);
}

void MoveMsg::Execute(){

}

void MoveMsg::GetData(void *rec){

}

void MoveMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int MoveMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void MoveMsg::PrintString(){
	printf("[%lld]MoveMsg[%d], mach_coord=%hhu\n", m_n_line_no, m_n_g_code, m_b_mach_coord);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
MoveMsg& MoveMsg::operator=( const MoveMsg& msg){
	if(&msg == this)
		return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_g_code = msg.m_n_g_code;
	this->m_n_last_g_code = msg.m_n_last_g_code;
	this->m_b_mach_coord = msg.m_b_mach_coord;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const MoveMsg &one, MoveMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_g_code == two.m_n_g_code &&
			one.m_n_last_g_code == two.m_n_last_g_code &&
			one.m_b_mach_coord == two.m_b_mach_coord)
		return true;
	return false;
}

/*****************************************************************CoordMsg类***************************************************************/
CoordMsg::CoordMsg(const DPointChn &pos, const DPointChn &src, int gcode, uint32_t axis_mask) : ModeMsg(gcode){
	this->m_pos = pos;
	this->m_pos_src = src;
	this->m_n_axis_mask = axis_mask;
	this->m_n_exec_step = 0;
	SetMsgType(COORD_MSG);

	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
	if(gcode == G53_CMD && axis_mask != 0){
		SetFlag(FLAG_AXIS_MOVE, true);   //轴移动指令标志
	}
}

void CoordMsg::Execute(){

}

void CoordMsg::GetData(void *rec){

}

void CoordMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data[out] : 输出数据
 * @param mask : 通道插补轴掩码
 * @param flag : 是否反向引导输出
 * @return 0--成功返回   1--无需返回
 */
int CoordMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;
	if(this->m_n_g_code != G53_CMD)
		return 1;

	//移动指令类型
	data->data.cmd = MOVE_G00;

	data->data.feed = 0;  //G00不需要发送进给速度


	data->data.line_no = this->m_n_line_no; 	//行号

	//目标位置
	DPoint pos;
	int count = 0;
	uint32_t ms = 0x01;
	double *pp = &pos.x;
	double *ps = m_pos.m_df_point;
	if(flag)
		ps = this->m_pos_src.m_df_point;
	for(int i = 0; i < kMaxAxisChn && count < 8; i++){
		if(mask & ms){
			*pp = ps[i];
		}
        pp++;
        count++;
		ms = ms<<1;
	}
	data->data.pos0 = MM2NM0_1(pos.x); //单位由mm转换为0.1nm
	data->data.pos1 = MM2NM0_1(pos.y);
	data->data.pos2 = MM2NM0_1(pos.z);
	data->data.pos3 = MM2NM0_1(pos.a4);
	data->data.pos4 = MM2NM0_1(pos.a5);
	data->data.pos5 = MM2NM0_1(pos.a6);
	data->data.pos6 = MM2NM0_1(pos.a7);
	data->data.pos7 = MM2NM0_1(pos.a8);

	//Ext_type
	if(this->m_n_flags.bits.step_flag)  //单段标志
		data->data.ext_type = 0x20;
	else
		data->data.ext_type = 0;

	if(this->m_n_flags.bits.jump_flag)
		data->data.ext_type |=0x100;   //bit8：跳段标志

	return res;
}

/**
 * @brief 生成仿真数据包
 * @param data[out] : 返回仿真数据，返回的坐标均为工件坐标系
 * @return 0--成功返回   1--无需返回
 */
int CoordMsg::GetSimulateData(CompilerSimData &data){
	data.type = 0;

	data.target[0] = m_pos.m_df_point[0];
	data.target[1] = m_pos.m_df_point[1];
	data.target[2] = m_pos.m_df_point[2];

	return 0;
}

/**
 * @brief 获取多维点形式的坐标值
 * @param pos[out] : 输出坐标值
 */
//void CoordMsg::GetTargetPosEx(DPointChn &pos){
//	int idx = 0;
//
//	for(int i = 0; i < kMaxAxisChn && idx < 8; i++){
//		if(this->m_n_axis_mask & (0x01<<i)){
//			pos.SetAxisValue(i, this->m_pos.GetAxisValue(idx));
//			idx++;
//		}
//	}
//}

void CoordMsg::PrintString(){
	printf("[%lld]CoordMsg[%d]\n", m_n_line_no, m_n_g_code);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
CoordMsg& CoordMsg::operator=( const CoordMsg& msg){
	if(&msg == this)
		return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_g_code = msg.m_n_g_code;
	this->m_pos = msg.m_pos;
	this->m_n_axis_mask = msg.m_n_axis_mask;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const CoordMsg &one, CoordMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_g_code == two.m_n_g_code &&
			one.m_pos == two.m_pos &&
			one.m_n_axis_mask == two.m_n_axis_mask)
		return true;
	return false;
}


/*****************************************************************LoopMsg类***************************************************************/
LoopMsg::LoopMsg(int gcode, double *param, uint8_t count, uint32_t mask):ModeMsg(gcode){

	SetMsgType(LOOP_MSG);

	this->m_mask_param = mask;
	this->m_p_df_param = param;
	this->m_n_param_count = count;

	SetFlag(FLAG_WAIT_MOVE_OVER, true);
}

void LoopMsg::Execute(){

}

void LoopMsg::GetData(void *rec){

}

void LoopMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int LoopMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void LoopMsg::PrintString(){
	printf("[%lld]LoopMsg[%d]\n", m_n_line_no, m_n_g_code);
}

/**
 * @brief 返回子程序号
 * @return
 */
int LoopMsg::GetMacroProgIndex(){
	int macro_index = 0;

	switch (this->GetGCode()) {
	case G73_CMD:
		macro_index = 9073;
		break;
	case G74_CMD:
		macro_index = 9074;
		break;
	case G76_CMD:
		macro_index = 9076;
		break;
	case G80_CMD:
		macro_index = 9080;
		break;
	case G81_CMD:
		macro_index = 9081;
		break;
	case G82_CMD:
		macro_index = 9082;
		break;
	case G83_CMD:
		macro_index = 9083;
		break;
	case G84_CMD:
		macro_index = 9084;
		break;
	case G85_CMD:
		macro_index = 9085;
		break;
	case G86_CMD:
		macro_index = 9086;
		break;
	case G87_CMD:
		macro_index = 9087;
		break;
	case G88_CMD:
		macro_index = 9088;
		break;
	case G89_CMD:
		macro_index = 9089;
		break;
	default:
		break;
	}

	return macro_index;
}

/**
 * @brief 返回子程序名
 * @param name[out] : 输出子程序名称字符串
 * @param abs_path[in] : true--绝对路径   false--相对路径
 */
//void LoopMsg::GetMacroProgName(char *name, bool abs_path){
//	int macro_index = this->GetMacroProgIndex();
//	if(abs_path){
//		if(m_n_macro_prog_type == 2){//同目录下用户宏程序
//			if(macro_index <= 9999)
//				sprintf(name, "%sO%04d.nc", PATH_NC_FILE, macro_index);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.nc", PATH_NC_FILE, macro_index);
//		}
//		else if(m_n_macro_prog_type == 3){//系统宏程序
//			if(macro_index <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.nc", PATH_NC_FILE, macro_index);
//			else
//				sprintf(name, "%ssys_sub/O%d.nc", PATH_NC_FILE, macro_index);
//		}else if(m_n_macro_prog_type == 4){
//			if(macro_index <= 9999)
//				sprintf(name, "%sO%04d.iso", PATH_NC_FILE, macro_index);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.iso", PATH_NC_FILE, macro_index);
//		}else if(m_n_macro_prog_type == 5){
//			if(macro_index <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.iso", PATH_NC_FILE, macro_index);
//			else
//				sprintf(name, "%ssys_sub/O%d.iso", PATH_NC_FILE, macro_index);
//		}else if(m_n_macro_prog_type == 6){//同目录下用户宏程序
//			if(macro_index <= 9999)
//				sprintf(name, "%sO%04d.NC", PATH_NC_FILE, macro_index);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.NC", PATH_NC_FILE, macro_index);
//		}else if(m_n_macro_prog_type == 7){//系统宏程序
//			if(macro_index <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.NC", PATH_NC_FILE, macro_index);
//			else
//				sprintf(name, "%ssys_sub/O%d.NC", PATH_NC_FILE, macro_index);
//		}else if(m_n_macro_prog_type == 8){
//			if(macro_index <= 9999)
//				sprintf(name, "%sO%04d.ISO", PATH_NC_FILE, macro_index);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.ISO", PATH_NC_FILE, macro_index);
//		}else if(m_n_macro_prog_type == 9){
//			if(macro_index <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.ISO", PATH_NC_FILE, macro_index);
//			else
//				sprintf(name, "%ssys_sub/O%d.ISO", PATH_NC_FILE, macro_index);
//		}
//	}else{
//		if(m_n_macro_prog_type == 2){//同目录下用户宏程序
//			if(macro_index <= 9999)
//				sprintf(name, "O%04d.nc", macro_index);   //拼接文件名称
//			else
//				sprintf(name, "O%d.nc", macro_index);
//		}
//		else if(m_n_macro_prog_type == 3){//系统宏程序
//			if(macro_index <= 9999)
//				sprintf(name, "sys_sub/O%04d.nc", macro_index);
//			else
//				sprintf(name, "sys_sub/O%d.nc", macro_index);
//		}else if(m_n_macro_prog_type == 4){
//			if(macro_index <= 9999)
//				sprintf(name, "O%04d.iso", macro_index);   //拼接文件名称
//			else
//				sprintf(name, "O%d.iso", macro_index);
//		}else if(m_n_macro_prog_type == 5){
//			if(macro_index <= 9999)
//				sprintf(name, "sys_sub/O%04d.iso", macro_index);
//			else
//				sprintf(name, "sys_sub/O%d.iso", macro_index);
//		}else if(m_n_macro_prog_type == 6){//同目录下用户宏程序
//			if(macro_index <= 9999)
//				sprintf(name, "O%04d.NC", macro_index);   //拼接文件名称
//			else
//				sprintf(name, "O%d.NC", macro_index);
//		}else if(m_n_macro_prog_type == 7){//系统宏程序
//			if(macro_index <= 9999)
//				sprintf(name, "sys_sub/O%04d.NC", macro_index);
//			else
//				sprintf(name, "sys_sub/O%d.NC", macro_index);
//		}else if(m_n_macro_prog_type == 8){
//			if(macro_index <= 9999)
//				sprintf(name, "O%04d.ISO", macro_index);   //拼接文件名称
//			else
//				sprintf(name, "O%d.ISO", macro_index);
//		}else if(m_n_macro_prog_type == 9){
//			if(macro_index <= 9999)
//				sprintf(name, "sys_sub/O%04d.ISO", macro_index);
//			else
//				sprintf(name, "sys_sub/O%d.ISO", macro_index);
//		}
//	}
//}

/**
 * @brief 返回参数数据
 * @param mask[out] : 返回参数mask
 * @param count[out] ： 返回参数个数
 * @return 返回参数数据区指针
 */
double *LoopMsg::GetParameter(uint32_t &mask, uint8_t &count){
	mask = m_mask_param;
	count = m_n_param_count;
	return m_p_df_param;
}

/**
 * @brief 设置上一个文件的绝对路径，用于手轮反向引导
 * @param file[in] : 文件绝对路径
 */
void LoopMsg::SetLastProgFile(char *file){
	if(file != nullptr){
		memset(m_str_last_prog_file, 0x00, kMaxPathLen);
		strcpy(m_str_last_prog_file, file);
	}
}

/**
 * @brief 获取上一个文件的绝对路径，用于手轮方向引导
 * @param file
 */
void LoopMsg::GetLastProgFile(char *file){
	if(file != nullptr){
		strcpy(file, m_str_last_prog_file);
	}
}


/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
LoopMsg& LoopMsg::operator=( const LoopMsg& msg){
	if(&msg == this)
		return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_g_code = msg.m_n_g_code;
	this->m_n_macro_prog_type = msg.m_n_macro_prog_type;
	this->m_mask_param = msg.m_mask_param;
	this->m_n_param_count = msg.m_n_param_count;
	if(this->m_p_df_param != nullptr){
		delete []m_p_df_param;
		m_p_df_param = nullptr;
	}
	if(this->m_n_param_count > 0){
		m_p_df_param = new double[this->m_n_param_count];
		memcpy(m_p_df_param, msg.m_p_df_param, sizeof(double)*m_n_param_count);
	}

	strcpy(this->m_str_last_prog_file, msg.m_str_last_prog_file);
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const LoopMsg &one, LoopMsg &two){
	if(one.m_n_param_count == two.m_n_param_count && one.m_mask_param == two.m_mask_param){
		for(uint8_t i = 0; i < one.m_n_param_count; i++){
			if(one.m_p_df_param[i] != two.m_p_df_param[i])
				return false;
		}
	}else
		return false;

	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_g_code == two.m_n_g_code &&
			one.m_n_macro_prog_type == two.m_n_macro_prog_type)
		return true;
	return false;
}


/*****************************************************************FeedMsg类***************************************************************/
FeedMsg::FeedMsg(double feed, double last_feed){
	this->m_df_feed = feed;
	this->m_df_last_feed = last_feed;
	SetMsgType(FEED_MSG);
}

void FeedMsg::Execute(){

}

void FeedMsg::GetData(void *rec){

}

void FeedMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int FeedMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void FeedMsg::PrintString(){
	printf("[%lld]FeedMsg[%lf, %lf]\n", m_n_line_no, m_df_feed, m_df_last_feed);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
FeedMsg& FeedMsg::operator=( const FeedMsg& msg){
	if(&msg == this)
		return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_df_feed = msg.m_df_feed;
	this->m_df_last_feed = msg.m_df_last_feed;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const FeedMsg &one, FeedMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_df_feed == two.m_df_feed &&
			one.m_df_last_feed == two.m_df_last_feed)
		return true;
	return false;
}

/*****************************************************************SpeedMsg类***************************************************************/
SpeedMsg::SpeedMsg(double speed){
	this->m_df_speed = speed;
	SetMsgType(SPEED_MSG);

	SetFlag(FLAG_WAIT_MOVE_OVER, true);
}

void SpeedMsg::Execute(){

}

void SpeedMsg::GetData(void *rec){

}

void SpeedMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int SpeedMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void SpeedMsg::PrintString(){
	printf("[%lld]SpeedMsg[%lf]\n", m_n_line_no, m_df_speed);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
SpeedMsg& SpeedMsg::operator=( const SpeedMsg& msg){
	if(&msg == this)
		return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_df_speed = msg.m_df_speed;

	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const SpeedMsg &one, SpeedMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_df_speed == two.m_df_speed)
		return true;
	return false;
}

/*****************************************************************ToolMsg类***************************************************************/
/**
 * @brief 构造函数
 * @param tool ： T代码数组指针
 * @param total ：T代码数量
 */
ToolMsg::ToolMsg(int *tool, uint8_t total){
	memset(m_n_tool, 0x00, sizeof(uint16_t)*kMaxTCodeInLine);
	this->m_n_tool_count = total;
	for(uint8_t i = 0; i < total; i++)
		this->m_n_tool[i] = tool[i];

	memset(m_n_tool_exec_segment, 0, kMaxTCodeInLine);

	this->m_n_sub_prog_name = kTCodeSubProg;
	m_n_sub_prog_type = 0;
	SetMsgType(TOOL_MSG);

	SetFlag(FLAG_WAIT_MOVE_OVER, true);
}

void ToolMsg::Execute(){

}

void ToolMsg::GetData(void *rec){

}

void ToolMsg::SetData(void *rec){

}

/**
 * @brief 是否首次执行此命令，即如果所有T指令的step都是0则为首次执行
 * @return true--是     false--不是
 */
bool ToolMsg::IsFirstExec(){
	for(uint8_t i = 0; i < this->m_n_tool_count; i++){
		if(this->m_n_tool_exec_segment[i] > 0)
			return false;
	}

	return true;
}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int ToolMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void ToolMsg::PrintString(){
	printf("[%lld]ToolMsg[%d]\n", m_n_line_no, m_n_tool_count);
}

/**
 * @brief 返回子程序名
 * @param name[out] : 输出子程序名
 * @param abs_path[in] : true--绝对路径   false--相对路径
 */
//void ToolMsg::GetSubProgName(char *name, bool abs_path){
//	if(abs_path){
//        //string dirName = m_p_file_map_info->GetDirName();
//		if(m_n_sub_prog_type == 2){//同目录下用户子程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.nc", PATH_NC_FILE, m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//		}
//		else if(m_n_sub_prog_type == 3){//系统子程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 4){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.iso", PATH_NC_FILE, m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 5){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 6){//同目录下用户宏程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.NC", PATH_NC_FILE, m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 7){//系统宏程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 8){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.ISO", PATH_NC_FILE, m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.ISO", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 9){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.ISO", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.ISO", PATH_NC_FILE, m_n_sub_prog_name);
//        }else if (m_n_sub_prog_type == 10){
//            if(m_n_sub_prog_name <= 9999)
//                sprintf(name, "%ssys_mac/O%04d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//            else
//                sprintf(name, "%ssys_mac/O%d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//        }else if (m_n_sub_prog_type == 11){
//            if(m_n_sub_prog_name <= 9999)
//                sprintf(name, "%ssys_mac/O%04d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//            else
//                sprintf(name, "%ssys_mac/O%d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//        }else if (m_n_sub_prog_name == 12){
//            if(m_n_sub_prog_name <= 9999)
//                sprintf(name, "%ssys_mac/O%04d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//            else
//                sprintf(name, "%ssys_mac/O%d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//        }else if (m_n_sub_prog_name == 13){
//            if(m_n_sub_prog_name <= 9999)
//                sprintf(name, "%ssys_mac/O%04d.ISO", PATH_NC_FILE, m_n_sub_prog_name);
//            else
//                sprintf(name, "%ssys_mac/O%d.ISO", PATH_NC_FILE, m_n_sub_prog_name);
//        }

//	}else{
//		if(m_n_sub_prog_type == 2){//同目录下用户子程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.nc", m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "O%d.nc", m_n_sub_prog_name);
//		}
//		else if(m_n_sub_prog_type == 3){//系统子程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.nc", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.nc", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 4){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.iso", m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "O%d.iso", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 5){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.iso", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.iso", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 6){//同目录下用户宏程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.NC", m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "O%d.NC", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 7){//系统宏程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.NC", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.NC", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 8){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.ISO", m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "O%d.ISO", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 9){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.ISO", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.ISO", m_n_sub_prog_name);
//		}
//	}

//}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
ToolMsg& ToolMsg::operator=( const ToolMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_tool_count = msg.m_n_tool_count;
	this->m_n_last_tool = msg.m_n_last_tool;
	memcpy(this->m_n_tool, msg.m_n_tool, sizeof(uint16_t)*kMaxTCodeInLine);
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const ToolMsg &one, ToolMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_tool_count == two.m_n_tool_count &&
			one.m_n_last_tool == two.m_n_last_tool){
		for(uint8_t i = 0; i < one.m_n_tool_count; i++){
			if(one.m_n_tool[i] != two.m_n_tool[i])
				return false;
		}
		return true;
	}

	return false;
}


/*****************************************************************TimeWaitMsg类***************************************************************/
TimeWaitMsg::TimeWaitMsg(uint32_t time):ModeMsg(G04_CMD){
	this->m_n_time_delay = time;
	this->m_n_start_time = 0;
	SetMsgType(TIME_WAIT_MSG);

	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
}

void TimeWaitMsg::Execute(){

}

void TimeWaitMsg::GetData(void *rec){

}

void TimeWaitMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int TimeWaitMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void TimeWaitMsg::PrintString(){
	printf("[%lld]TimeWaitMsg[%d]\n", m_n_line_no, m_n_time_delay);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
TimeWaitMsg& TimeWaitMsg::operator=( const TimeWaitMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_time_delay = msg.m_n_time_delay;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const TimeWaitMsg &one, TimeWaitMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_time_delay == two.m_n_time_delay)
		return true;
	return false;
}


/*****************************************************************RefReturnMsg类***************************************************************/
/**
 * @brief 构造函数
 * @param gcode
 * @param axis_mask
 * @param mid
 */
RefReturnMsg::RefReturnMsg(int gcode, uint32_t axis_mask, DPointChn &mid):ModeMsg(gcode){
	this->m_n_axis_mask = axis_mask;
	this->m_pos_middle = mid;
	this->m_n_exec_step = 0;
	SetMsgType(REF_RETURN_MSG);

//	SetFlag(FLAG_AXIS_MOVE, true);   //轴移动指令
	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
	SetFlag(FLAG_BLOCK_OVER, true);		//块结束
}

void RefReturnMsg::Execute(){

}

void RefReturnMsg::GetData(void *rec){

}

void RefReturnMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int RefReturnMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

/**
 * @brief 生成仿真数据包
 * @param data[out] : 返回仿真数据
 * @return 0--成功返回   1--无需返回
 */
int RefReturnMsg::GetSimulateData(CompilerSimData &data){
	data.type = 0;

	//返回中间点坐标
	data.target[0] = this->m_pos_middle.m_df_point[0];
	data.target[1] = this->m_pos_middle.m_df_point[1];
	data.target[2] = this->m_pos_middle.m_df_point[2];

	return 0;
}

void RefReturnMsg::PrintString(){
	printf("[%lld]RefReturnMsg\n", m_n_line_no);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
RefReturnMsg& RefReturnMsg::operator=( const RefReturnMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_axis_mask = msg.m_n_axis_mask;
	this->m_pos_middle = msg.m_pos_middle;
	this->ref_id = msg.ref_id;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const RefReturnMsg &one, RefReturnMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_axis_mask == two.m_n_axis_mask &&
			one.m_pos_middle == two.m_pos_middle)
		return true;
	return false;
}


/*****************************************************************SkipMsg类***************************************************************/
/**
 * @brief 构造函数
 * @param source
 * @param target
 * @param feed
 * @param axis_mask
 */
SkipMsg::SkipMsg(const DPointChn &source, const DPointChn &target, const double feed, const uint32_t axis_mask):LineMsg(source, target, feed, axis_mask){
	this->m_n_exec_step = 0;
	SetMsgType(SKIP_MSG);

	SetFlag(FLAG_AXIS_MOVE, true);   //轴移动指令
	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
	SetFlag(FLAG_BLOCK_OVER, true);		//块结束
}

/**
 * @brief
 */
void SkipMsg::Execute(){

}

void SkipMsg::GetData(void *rec){

}

void SkipMsg::SetData(void *rec){

}

void SkipMsg::PrintString(){
	printf("[%lld]SkipMsg\n", m_n_line_no);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
SkipMsg& SkipMsg::operator=( const SkipMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_point_source = msg.m_point_source;
	this->m_point_target = msg.m_point_target;
	this->m_axis_move_mask = msg.m_axis_move_mask;
	this->m_df_feed = msg.m_df_feed;
	this->m_io_data = msg.m_io_data;

	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
 
bool operator ==( const SkipMsg &one, SkipMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_point_source == two.m_point_source &&
			one.m_point_target == two.m_point_target &&
			one.m_axis_move_mask == two.m_axis_move_mask &&
			one.m_df_feed == two.m_df_feed &&
			one.m_io_data == two.m_io_data)
		return true;
	return false;
}


/*****************************************************************SkipMeasureMsg类***************************************************************/
/**
 * @brief 构造函数
 * @param source
 * @param target
 * @param feed
 * @param axis_mask
 */
SkipMeasureMsg::SkipMeasureMsg(const DPointChn &source, const DPointChn &target, const double feed, const uint32_t axis_mask):LineMsg(source, target, feed, axis_mask){
    this->m_n_exec_step = 0;
    SetMsgType(SKIP_MEASURE_MSG);

    SetFlag(FLAG_AXIS_MOVE, true);   //轴移动指令
    SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
    SetFlag(FLAG_BLOCK_OVER, true);		//块结束
}

/**
 * @brief
 */
void SkipMeasureMsg::Execute(){

}

void SkipMeasureMsg::GetData(void *rec){

}

void SkipMeasureMsg::SetData(void *rec){

}


/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
SkipMeasureMsg& SkipMeasureMsg::operator=( const SkipMeasureMsg& msg){
    if(&msg == this)
        return *this;
    this->m_n_type = msg.m_n_type;
    this->m_n_line_no = msg.m_n_line_no;
    this->m_n_flags.all = msg.m_n_flags.all;
    this->m_point_source = msg.m_point_source;
    this->m_point_target = msg.m_point_target;
    this->m_axis_move_mask = msg.m_axis_move_mask;
    this->m_df_feed = msg.m_df_feed;
    this->m_io_data = msg.m_io_data;

    return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */

bool operator ==( const SkipMeasureMsg &one, SkipMeasureMsg &two){
    if(one.m_n_type == two.m_n_type &&
            one.m_n_line_no == two.m_n_line_no &&
            one.m_n_flags.all == two.m_n_flags.all &&
            one.m_point_source == two.m_point_source &&
            one.m_point_target == two.m_point_target &&
            one.m_axis_move_mask == two.m_axis_move_mask &&
            one.m_df_feed == two.m_df_feed &&
            one.m_io_data == two.m_io_data)
        return true;
    return false;
}


/*****************************************************************AutoToolMeasureMsg类***************************************************************/

/**
 * @brief 构造函数
 * @param target ： 目标位置
 * @param axis_mask ：轴运动mask
 * @param times : 对刀次数
 */
AutoToolMeasureMsg::AutoToolMeasureMsg(const DPointChn &target, const uint32_t axis_mask, const int h_code, const int times):ModeMsg(G37_CMD){

	this->m_pos_target = target;
	this->m_mask_axis_move = axis_mask;
	this->m_n_times = times;
	this->m_n_h_index = h_code;

	m_n_macro_prog_name = 9037;

	this->SetMsgType(AUTO_TOOL_MEASURE_MSG);

    //SetFlag(FLAG_AXIS_MOVE, true);   //轴移动指令
	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
	SetFlag(FLAG_BLOCK_OVER, true);		//块结束
}

/**
 * @brief
 */
void AutoToolMeasureMsg::Execute(){

}

void AutoToolMeasureMsg::GetData(void *rec){

}

void AutoToolMeasureMsg::SetData(void *rec){

}

void AutoToolMeasureMsg::PrintString(){
	printf("[%lld]AutoToolMeasureMsg\n", m_n_line_no);
}

/**
 * @brief 返回宏程序名
 * @param name[out] : 宏程序文件绝对路径
 * @param abs_path[in] : true--绝对路径   false--相对路径
 */
//void AutoToolMeasureMsg::GetMacroProgName(char *name, bool abs_path){
//	if(abs_path){
//		if(m_n_macro_prog_type == 2){//同目录下用户宏程序
//			sprintf(name, "%sO%04d.nc", PATH_NC_FILE, m_n_macro_prog_name);   //拼接文件名称
//		}else if(m_n_macro_prog_type == 3){//系统宏程序
//			sprintf(name, "%ssys_sub/O%04d.nc", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 4){
//			sprintf(name, "%sO%04d.iso", PATH_NC_FILE, m_n_macro_prog_name);   //拼接文件名称
//		}else if(m_n_macro_prog_type == 5){
//			sprintf(name, "%ssys_sub/O%04d.iso", PATH_NC_FILE, m_n_macro_prog_name);
//		}else 	if(m_n_macro_prog_type == 6){//同目录下用户宏程序
//			sprintf(name, "%sO%04d.NC", PATH_NC_FILE, m_n_macro_prog_name);   //拼接文件名称
//		}else if(m_n_macro_prog_type == 7){//系统宏程序
//			sprintf(name, "%ssys_sub/O%04d.NC", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 8){
//			sprintf(name, "%sO%04d.ISO", PATH_NC_FILE, m_n_macro_prog_name);   //拼接文件名称
//		}else if(m_n_macro_prog_type == 9){
//			sprintf(name, "%ssys_sub/O%04d.ISO", PATH_NC_FILE, m_n_macro_prog_name);
//		}
//	}else{
//		if(m_n_macro_prog_type == 2){//同目录下用户宏程序
//			sprintf(name, "O%04d.nc", m_n_macro_prog_name);   //拼接文件名称
//		}else if(m_n_macro_prog_type == 3){//系统宏程序
//			sprintf(name, "sys_sub/O%04d.nc", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 4){
//			sprintf(name, "O%04d.iso", m_n_macro_prog_name);   //拼接文件名称
//		}else if(m_n_macro_prog_type == 5){
//			sprintf(name, "sys_sub/O%04d.iso", m_n_macro_prog_name);
//		}else 	if(m_n_macro_prog_type == 6){//同目录下用户宏程序
//			sprintf(name, "O%04d.NC", m_n_macro_prog_name);   //拼接文件名称
//		}else if(m_n_macro_prog_type == 7){//系统宏程序
//			sprintf(name, "sys_sub/O%04d.NC", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 8){
//			sprintf(name, "O%04d.ISO", m_n_macro_prog_name);   //拼接文件名称
//		}else if(m_n_macro_prog_type == 9){
//			sprintf(name, "sys_sub/O%04d.ISO", m_n_macro_prog_name);
//		}
//	}

//}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
AutoToolMeasureMsg& AutoToolMeasureMsg::operator=( const AutoToolMeasureMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_pos_target = msg.m_pos_target;
	this->m_mask_axis_move = msg.m_mask_axis_move;
	this->m_n_times = msg.m_n_times;
	this->m_n_h_index = msg.m_n_h_index;

	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */

bool operator ==( const AutoToolMeasureMsg &one, AutoToolMeasureMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_pos_target == two.m_pos_target &&
			one.m_mask_axis_move == two.m_mask_axis_move &&
			one.m_n_times == two.m_n_times &&
			one.m_n_h_index == two.m_n_h_index)
		return true;
	return false;
}


/*****************************************************************AuxMsg类***************************************************************/
//默认构造函数
AuxMsg::AuxMsg(int mcode){
	memset(m_n_m_code, 0, sizeof(int)*kMaxMCodeInLine);
	m_n_m_code[0] = mcode;
	m_n_m_count = 1;
	memset(m_n_aux_exec_segment, 0, kMaxMCodeInLine);
	SetMsgType(AUX_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位

    //lidianqiang:MDA自动加上M30临时改为M300
    if(mcode == 2 || mcode == 30 || mcode == 300)
		this->SetFlag(FLAG_EOF, true);		//M02/M30程序结束指令
}

AuxMsg::AuxMsg(int *mcode, uint8_t total){
	if(mcode != nullptr)
		memcpy(this->m_n_m_code, mcode, sizeof(int)*total);

	m_n_m_count = total;
	memset(m_n_aux_exec_segment, 0, kMaxMCodeInLine);

	SetMsgType(AUX_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位

	for(uint8_t i = 0; i < total; i++){
        if(mcode[i] == 2 || mcode[i] == 30 || mcode[i] == 300)
			this->SetFlag(FLAG_EOF, true);		//M02/M30程序结束指令
	}
}

void AuxMsg::Execute(){

}

void AuxMsg::GetData(void *rec){

}

void AuxMsg::SetData(void *rec){

}

/**
 * @brief 是否首次执行此命令，即如果所有M指令的step都是0则为首次执行
 * @return true--是     false--不是
 */
bool AuxMsg::IsFirstExec(){
	for(uint8_t i = 0; i < this->m_n_m_count; i++){
		if(this->m_n_aux_exec_segment[i] > 0)
			return false;
	}

	return true;
}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int AuxMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void AuxMsg::PrintString(){
	printf("[%lld]AuxMsg[%d]\n", m_n_line_no, m_n_m_count);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
AuxMsg& AuxMsg::operator=( const AuxMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	memcpy(m_n_m_code, msg.m_n_m_code, sizeof(int)*kMaxMCodeInLine);
	this->m_n_m_count = msg.m_n_m_count;
	memcpy(m_n_aux_exec_segment, msg.m_n_aux_exec_segment, sizeof(uint8_t)*kMaxMCodeInLine);
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const AuxMsg &one, AuxMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_m_count == two.m_n_m_count){
		for(uint8_t i = 0; i < one.m_n_m_count; i++){
			if(one.m_n_m_code[i] != two.m_n_m_code[i])
				return false;
		}
		return true;
	}

	return false;
}

/*****************************************************************SubProgCallMsg类***************************************************************/
SubProgCallMsg::SubProgCallMsg(int pcode, int lcode, uint8_t scan):AuxMsg(98){
	this->m_n_sub_prog_name = pcode;
	this->m_n_call_times = lcode;
	memset(this->m_str_last_prog_file, 0x00, kMaxPathLen);
	SetMsgType(SUBPROG_CALL_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位

    m_n_scan_mode = scan;//子程序查找规则
}

void SubProgCallMsg::Execute(){

}

void SubProgCallMsg::GetData(void *rec){

}

void SubProgCallMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int SubProgCallMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

/**
 * @brief 打印信息
 */
void SubProgCallMsg::PrintString(){
	printf("[%lld]SubProgCallMsg[%d, %d, %d]\n", m_n_line_no, m_n_m_code[0], m_n_sub_prog_name, m_n_call_times);
}

/**
 * @brief 设置上一个文件的绝对路径，用于手轮反向引导
 * @param file[in] : 文件绝对路径
 */
void SubProgCallMsg::SetLastProgFile(char *file){
	if(file != nullptr){
		memset(m_str_last_prog_file, 0x00, kMaxPathLen);
		strcpy(m_str_last_prog_file, file);
	}
}

/**
 * @brief 获取上一个文件的绝对路径，用于手轮方向引导
 * @param file
 */
void SubProgCallMsg::GetLastProgFile(char *file){
	if(file != nullptr){
		strcpy(file, m_str_last_prog_file);
    }
}

/**
 * @brief 返回子程序查找规则
 * @return
 */
uint8_t SubProgCallMsg::GetScanMode() const
{
    return m_n_scan_mode;
}

/**
 * @brief 返回子程序名
 * @param name[out] : 输出子程序名
 * @param abs_path[in] : true--绝对路径   false--相对路径
 */
//void SubProgCallMsg::GetSubProgName(char *name, bool abs_path){
//	if(abs_path){
//		if(m_n_sub_prog_type == 2){//同目录下用户子程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.nc", PATH_NC_FILE, m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//		}
//		else if(m_n_sub_prog_type == 3){//系统子程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 4){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.iso", PATH_NC_FILE, m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 5){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 6){//同目录下用户宏程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.NC", PATH_NC_FILE, m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 7){//系统宏程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 8){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.ISO", PATH_NC_FILE, m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.ISO", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 9){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.ISO", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.ISO", PATH_NC_FILE, m_n_sub_prog_name);
//		}
//	}else{
//		if(m_n_sub_prog_type == 2){//同目录下用户子程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.nc", m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "O%d.nc", m_n_sub_prog_name);
//		}
//		else if(m_n_sub_prog_type == 3){//系统子程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.nc", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.nc", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 4){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.iso", m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "O%d.iso", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 5){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.iso", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.iso", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 6){//同目录下用户宏程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.NC", m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "O%d.NC", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 7){//系统宏程序
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.NC", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.NC", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 8){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.ISO", m_n_sub_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "O%d.ISO", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 9){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.ISO", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.ISO", m_n_sub_prog_name);
//		}
//	}

//}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
SubProgCallMsg& SubProgCallMsg::operator=( const SubProgCallMsg& msg){

	if(&msg == this)
			return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_m_code[0] = msg.m_n_m_code[0];
	this->m_n_m_count = msg.m_n_m_count;
	this->m_n_sub_prog_name = msg.m_n_sub_prog_name;
	this->m_n_call_times = msg.m_n_call_times;
	this->m_n_sub_prog_type = msg.m_n_sub_prog_type;
	strcpy(this->m_str_last_prog_file, msg.m_str_last_prog_file);
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const SubProgCallMsg &one, SubProgCallMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_m_code[0] == two.m_n_m_code[0] &&
			one.m_n_sub_prog_name == two.m_n_sub_prog_name &&
			one.m_n_call_times == two.m_n_call_times &&
			one.m_n_sub_prog_type == two.m_n_sub_prog_type)
		return true;
	return false;
}

/*****************************************************************ProgCallMsg类***************************************************************/
/**
 * @brief 构造函数
 * @param pcode : 宏程序号
 * @param lcode ：调用次数
 * @param param ：参数指针
 * @param count ：参数个数
 * @param mask ：参数mask
 */
MacroProgCallMsg::MacroProgCallMsg(int pcode, int lcode, double *param, uint8_t count, uint32_t mask, uint8_t scan):ModeMsg(G65_CMD){

	this->m_n_macro_prog_name = pcode;
	this->m_n_call_times = lcode;
	this->m_mask_param = mask;
	this->m_p_df_param = param;
	this->m_n_param_count = count;

    m_n_scan_mode = scan;//子程序查找模式

	SetMsgType(MACRO_PROG_CALL_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位

}

/**
 * @brief 析构函数
 */
MacroProgCallMsg::~MacroProgCallMsg(){
	if(this->m_p_df_param != nullptr){
		delete []this->m_p_df_param;
		this->m_p_df_param = nullptr;
	}
}

void MacroProgCallMsg::Execute(){

}

void MacroProgCallMsg::GetData(void *rec){

}

void MacroProgCallMsg::SetData(void *rec){

}

/**
 * @brief 返回参数数据
 * @param mask[out] : 返回参数mask
 * @param count[out] ： 返回参数个数
 * @return 返回参数数据区指针
 */
double *MacroProgCallMsg::GetParameter(uint32_t &mask, uint8_t &count){
	mask = m_mask_param;
	count = m_n_param_count;
	return m_p_df_param;
}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int MacroProgCallMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void MacroProgCallMsg::PrintString(){
	printf("[%lld]MacroProgCallMsg[%d, %d]\n", m_n_line_no, m_n_macro_prog_name, m_n_call_times);
}

/**
 * @brief 设置上一个文件的绝对路径，用于手轮反向引导
 * @param file[in] : 文件绝对路径
 */
void MacroProgCallMsg::SetLastProgFile(char *file){
	if(file != nullptr){
		memset(m_str_last_prog_file, 0x00, kMaxPathLen);
		strcpy(m_str_last_prog_file, file);
	}
}

/**
 * @brief 获取上一个文件的绝对路径，用于手轮方向引导
 * @param file
 */
void MacroProgCallMsg::GetLastProgFile(char *file){
	if(file != nullptr){
		strcpy(file, m_str_last_prog_file);
    }
}

/**
 * @brief 返回宏程序查找规则
 * @return
 */
uint8_t MacroProgCallMsg::GetScanMode() const
{
    return m_n_scan_mode;
}


/**
 * @brief 返回子程序名
 * @param name[out] : 输出子程序名称字符串
 * @param abs_path[in] : true--绝对路径   false--相对路径
 */
//void MacroProgCallMsg::GetMacroProgName(char *name, bool abs_path){
//	if(abs_path){
//		if(m_n_macro_prog_type == 2){//同目录下用户宏程序
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%sO%04d.nc", PATH_NC_FILE, m_n_macro_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.nc", PATH_NC_FILE, m_n_macro_prog_name);
//		}
//		else if(m_n_macro_prog_type == 3){//系统宏程序
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.nc", PATH_NC_FILE, m_n_macro_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.nc", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 4){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%sO%04d.iso", PATH_NC_FILE, m_n_macro_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.iso", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 5){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.iso", PATH_NC_FILE, m_n_macro_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.iso", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 6){//同目录下用户宏程序
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%sO%04d.NC", PATH_NC_FILE, m_n_macro_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.NC", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 7){//系统宏程序
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.NC", PATH_NC_FILE, m_n_macro_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.NC", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 8){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%sO%04d.ISO", PATH_NC_FILE, m_n_macro_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "%sO%d.ISO", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 9){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.ISO", PATH_NC_FILE, m_n_macro_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.ISO", PATH_NC_FILE, m_n_macro_prog_name);
//		}
//	}else{
//		if(m_n_macro_prog_type == 2){//同目录下用户宏程序
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "O%04d.nc", m_n_macro_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "O%d.nc", m_n_macro_prog_name);
//		}
//		else if(m_n_macro_prog_type == 3){//系统宏程序
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.nc", m_n_macro_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.nc", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 4){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "O%04d.iso", m_n_macro_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "O%d.iso", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 5){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.iso", m_n_macro_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.iso", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 6){//同目录下用户宏程序
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "O%04d.NC", m_n_macro_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "O%d.NC", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 7){//系统宏程序
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.NC", m_n_macro_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.NC", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 8){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "O%04d.ISO", m_n_macro_prog_name);   //拼接文件名称
//			else
//				sprintf(name, "O%d.ISO", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 9){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.ISO", m_n_macro_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.ISO", m_n_macro_prog_name);
//		}
//	}

//}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
MacroProgCallMsg& MacroProgCallMsg::operator=( const MacroProgCallMsg& msg){

	if(&msg == this)
			return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_g_code = msg.m_n_g_code;
	this->m_n_macro_prog_name = msg.m_n_macro_prog_name;
	this->m_n_call_times = msg.m_n_call_times;
	this->m_n_macro_prog_type = msg.m_n_macro_prog_type;
	this->m_mask_param = msg.m_mask_param;
	this->m_n_param_count = msg.m_n_param_count;
	if(this->m_p_df_param != nullptr){
		delete []m_p_df_param;
		m_p_df_param = nullptr;
	}
	if(this->m_n_param_count > 0){
		m_p_df_param = new double[this->m_n_param_count];
		memcpy(m_p_df_param, msg.m_p_df_param, sizeof(double)*m_n_param_count);
	}

	strcpy(this->m_str_last_prog_file, msg.m_str_last_prog_file);
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const MacroProgCallMsg &one, MacroProgCallMsg &two){
	if(one.m_n_param_count == two.m_n_param_count && one.m_mask_param == two.m_mask_param){
		for(uint8_t i = 0; i < one.m_n_param_count; i++){
			if(one.m_p_df_param[i] != two.m_p_df_param[i])
				return false;
		}
	}else
		return false;

	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_g_code == two.m_n_g_code &&
			one.m_n_macro_prog_name == two.m_n_macro_prog_name &&
			one.m_n_call_times == two.m_n_call_times &&
			one.m_n_macro_prog_type == two.m_n_macro_prog_type)
		return true;
	return false;
}

/*****************************************************************SubProgReturnMsg类******************************************************************/
SubProgReturnMsg::SubProgReturnMsg(char *file, bool ret_macro):AuxMsg(99){
	strcpy(this->m_str_file_name, file);
	this->m_b_ret_from_macroprog = ret_macro;
	SetMsgType(SUBPROG_RETURN_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
}

void SubProgReturnMsg::Execute(){

}

void SubProgReturnMsg::GetData(void *rec){

}

void SubProgReturnMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int SubProgReturnMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void SubProgReturnMsg::PrintString(){
    printf("SubProgReturnMsg[file:%s]\n", this->m_str_file_name);
}


/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
SubProgReturnMsg& SubProgReturnMsg::operator=( const SubProgReturnMsg& msg){

	if(&msg == this)
			return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_m_code[0] = msg.m_n_m_code[0];
	this->m_n_m_count = msg.m_n_m_count;
	strcpy(m_str_file_name, msg.m_str_file_name);
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const SubProgReturnMsg &one, SubProgReturnMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_m_code == two.m_n_m_code &&
			strcmp(one.m_str_file_name, two.m_str_file_name)==0)
		return true;
	return false;
}



/*****************************************************************CompensateMsg类***************************************************************/
/**
 * @brief CompensateMsg构造函数
 * @param gcode
 * @param data
 * @param source
 * @param target
 * @param axis_mask
 * @param feed
 * @param move_type
 */
CompensateMsg::CompensateMsg(int gcode, uint16_t data, const DPointChn &source, const DPointChn &target, const uint32_t axis_mask,
		const double feed, uint8_t move_type):LineMsg(source, target, feed, axis_mask){
	this->m_n_g_code = gcode;
	this->m_n_data = data;
	this->m_n_data_last = data;
	this->m_n_move_type = move_type;
	SetMsgType(COMPENSATE_MSG);
	this->m_n_exec_step = 0;


	if(m_axis_move_mask == 0){
		this->SetFlag(FLAG_AXIS_MOVE, false);
	}

//	if(gcode == G43_CMD || gcode == G44_CMD || gcode == G49_CMD || gcode == G43_4_CMD){	//如果是刀具长度补偿则有轴移动
////		SetFlag(FLAG_BLOCK_OVER, true);  //设置分块结束标志   //注释原因：此处不设置块结束标志，由后面是否跟随移动指令来决定是否设置
//
//		SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
//	}

	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
}

void CompensateMsg::Execute(){

}

void CompensateMsg::GetData(void *rec){

}

void CompensateMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @param flag : true--反向    false--正向
 * @return 0--成功返回   1--无需返回
 */
int CompensateMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;

	if(this->m_n_flags.bits.axis_move == 0)
		return 1;

	//移动指令类型
	data->data.cmd = m_n_move_type;


	//进给速度
	if(m_n_move_type == MOVE_G01)
		data->data.feed = FEED_TRANS(m_df_feed);   //单位由mm/min转换为um/s
    else{
		data->data.feed = 0;
    }
//	printf("linemsg feed : %d\n", data->data.feed);


	data->data.line_no = this->m_n_line_no; 	//行号

	//目标位置
	DPoint pos;
	int count = 0;
	uint32_t ms = 0x01;
	double *pp = &pos.x;
	double *ps = m_point_target.m_df_point;
	if(flag)
		ps = this->m_point_source.m_df_point;
	for(int i = 0; i < kMaxAxisChn && count < 8; i++){
		if(mask & ms){
			*pp = ps[i];
		}
        pp++;
        count++;
		ms = ms<<1;
	}

	data->data.pos0 = MM2NM0_1(pos.x); //单位由mm转换为0.1nm
	data->data.pos1 = MM2NM0_1(pos.y);
	data->data.pos2 = MM2NM0_1(pos.z);
	data->data.pos3 = MM2NM0_1(pos.a4);
	data->data.pos4 = MM2NM0_1(pos.a5);
	data->data.pos5 = MM2NM0_1(pos.a6);
	data->data.pos6 = MM2NM0_1(pos.a7);
	data->data.pos7 = MM2NM0_1(pos.a8);

	//Ext_type
	if(this->m_n_flags.bits.step_flag)  //单段标志
		data->data.ext_type = 0x20;
	else
		data->data.ext_type = 0;

	if(this->m_n_flags.bits.jump_flag)
		data->data.ext_type |=0x100;   //bit8：跳段标志


	return res;
}

/**
 * @brief 生成仿真数据包
 * @param data[out] : 返回仿真数据
 * @return 0--成功返回   1--无需返回
 */
int CompensateMsg::GetSimulateData(CompilerSimData &data){

	if(m_n_move_type == MOVE_G01)
		data.type = 1;
	else
		data.type = 0;

	data.target[0] = this->m_point_target.m_df_point[0];
	data.target[1] = this->m_point_target.m_df_point[1];
	data.target[2] = this->m_point_target.m_df_point[2];

	return 0;
}

void CompensateMsg::PrintString(){
	printf("[%lld]CompensateMsg[%d], dest[%lf, %lf, %lf]\n", m_n_line_no, m_n_g_code, m_point_target.m_df_point[0], m_point_target.m_df_point[1],
			m_point_target.m_df_point[2]);
}


/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
CompensateMsg& CompensateMsg::operator=( const CompensateMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_g_code = msg.m_n_g_code;
	this->m_n_data = msg.m_n_data;
	this->m_n_data_last = msg.m_n_data_last;
	this->m_point_target = msg.m_point_target;
	this->m_point_source = msg.m_point_source;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const CompensateMsg &one, CompensateMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_g_code == two.m_n_g_code &&
			one.m_n_data == two.m_n_data &&
			one.m_n_data == two.m_n_data_last &&
			one.m_point_target == two.m_point_target &&
			one.m_point_source == two.m_point_source)
		return true;
	return false;
}

/*****************************************************************RapidMsg类***************************************************************/
RapidMsg::RapidMsg(const DPointChn &source, const DPointChn &target, const uint32_t axis_mask):MoveMsg(G00_CMD){
	m_point_target = target;
	m_point_source = source;
	m_axis_move_mask = axis_mask;
	m_io_data = 0;

//	this->m_p_pmc_target = nullptr;
//	this->m_pmc_move_mask = 0;
    this->m_n_pmc_count = 0;
//	this->m_b_inc_pos = false;
	m_n_exec_step = 0;

	SetMsgType(RAPID_MSG);

	this->SetFlag(FLAG_AXIS_MOVE, true);  //有轴移动指令

	if(target == source)
		this->SetFlag(FLAG_POINT_MOVE, true);   //点动移动
}

/**
 * @brief 析构函数
 */
RapidMsg::~RapidMsg(){
//	if(this->m_p_pmc_target)
//		delete []this->m_p_pmc_target;
}

void RapidMsg::Execute(){

}

void RapidMsg::GetData(void *rec){

}

void RapidMsg::SetData(void *rec){

}

/**
 * @brief 设置PMC轴的运动数据
 * @param pmc_count ： PMC轴数
 * @param inc : 是否增量模式位置
 */
void RapidMsg::SetPmcAxisCount(uint8_t pmc_mask){

    this->m_n_pmc_count = pmc_mask;


//    if(m_n_pmc_count > 0){
//		SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
//		SetFlag(FLAG_BLOCK_OVER, true);		  //块结束
//	}
}

/**
 * @brief 获取PMC轴的运动目标数据
 * @param count[out] ： PMC轴数
 * @param mask[out] ： 轴的mask
 * @param inc[out] : 增量模式位置
 * @return 轴目标位置数组的头指针
 */
//double *RapidMsg::GetPmcTarget(uint8_t &count, uint32_t &mask, bool &inc){
//	count =  this->m_n_pmc_count;
//	mask = this->m_pmc_move_mask;
//	inc = this->m_b_inc_pos;
//	return this->m_p_pmc_target;
//}

/**
 * @brief 更新刀偏后同步目标位置
 * @param pos : 更新刀偏后的当前工件坐标系位置
 */
void RapidMsg::RefreshTargetPos(DPointChn &pos){
	if(this->m_axis_move_mask == 0)
		return;

	this->m_point_source = pos;  //同步起始位置

	for(int i = 0; i < 8; i++){
		if(m_axis_move_mask & (0x01<<i)){
			continue;
		}
		m_point_target.m_df_point[i] = pos.m_df_point[i];	//同步目标位置
	}

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int RapidMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;
	//移动指令类型
	data->data.cmd = MOVE_G00;

	data->data.feed = 0;  //G00不需要发送进给速度


	data->data.line_no = this->m_n_line_no; 	//行号

	//目标位置
	DPoint pos;
	int count = 0;
	uint32_t ms = 0x01;
	double *pp = &pos.x;
	double *ps = m_point_target.m_df_point;
	if(flag)
		ps = this->m_point_source.m_df_point;
	for(int i = 0; i < kMaxAxisChn && count < 8; i++){
		if(mask & ms){
			*pp = ps[i];
		}
        pp++;
        count++;
		ms = ms<<1;
	}

	data->data.pos0 = MM2NM0_1(pos.x); //单位由mm转换为0.1nm
	data->data.pos1 = MM2NM0_1(pos.y);
	data->data.pos2 = MM2NM0_1(pos.z);
	data->data.pos3 = MM2NM0_1(pos.a4);
	data->data.pos4 = MM2NM0_1(pos.a5);
	data->data.pos5 = MM2NM0_1(pos.a6);
	data->data.pos6 = MM2NM0_1(pos.a7);
	data->data.pos7 = MM2NM0_1(pos.a8);

	data->data.reserved = this->m_io_data;   //IO数据

	//Ext_type
	if(this->m_n_flags.bits.step_flag)  //单段标志
		data->data.ext_type = 0x20;
	else
		data->data.ext_type = 0;

	if(this->m_n_flags.bits.jump_flag)
		data->data.ext_type |=0x100;   //bit8：跳段标志


//	printf("output rapid msg: (%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf)\n", pos.x, pos.y, pos.z, pos.a4, pos.a5, pos.a6, pos.a7, pos.a8);

	return res;
}

/**
 * @brief 生成仿真数据包
 * @param data[out] : 返回仿真数据
 * @return 0--成功返回   1--无需返回
 */
int RapidMsg::GetSimulateData(CompilerSimData &data){

	data.type = 0;

	data.target[0] = this->m_point_target.m_df_point[0];
	data.target[1] = this->m_point_target.m_df_point[1];
	data.target[2] = this->m_point_target.m_df_point[2];

	return 0;
}

/**
 * @brief 打印输出
 */
void RapidMsg::PrintString(){
	printf("[%lld]RapidMsg, dest[%lf, %lf, %lf]\n", m_n_line_no, m_point_target.m_df_point[0], m_point_target.m_df_point[1], m_point_target.m_df_point[2]);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
RapidMsg& RapidMsg::operator=( const RapidMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_point_target = msg.m_point_target;
	this->m_point_source = msg.m_point_source;
	this->m_io_data = msg.m_io_data;

    this->m_n_pmc_count = msg.m_n_pmc_count;
//	this->m_pmc_move_mask = msg.m_pmc_move_mask;
//	this->m_b_inc_pos = msg.m_b_inc_pos;
	this->m_n_exec_step = msg.m_n_exec_step;
//	if(this->m_p_pmc_target){
//		delete []m_p_pmc_target;
//		m_p_pmc_target = nullptr;
//	}
//	if(m_n_pmc_count > 0){
//		m_p_pmc_target = new double[m_n_pmc_count];
//		if(m_p_pmc_target){
//			memcpy(m_p_pmc_target, msg.m_p_pmc_target, sizeof(double)*m_n_pmc_count);
//		}
//	}
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const RapidMsg &one, RapidMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_point_target == two.m_point_target &&
			one.m_point_source == two.m_point_source &&
			one.m_io_data == two.m_io_data &&
            one.m_n_pmc_count == two.m_n_pmc_count)
		return true;
	return false;
}

/*****************************************************************LineMsg类***************************************************************/
/**
 * @brief 构造函数
 */
LineMsg::LineMsg(const DPointChn &source, const DPointChn &target, const double feed, const uint32_t axis_mask):MoveMsg(G01_CMD){
	m_point_target = target;
	m_point_source = source;
	m_axis_move_mask = axis_mask;
	m_df_feed = feed;
	m_io_data = 0;

//	m_p_pmc_target = nullptr;
//	m_pmc_move_mask = 0;
    m_n_pmc_count = 0;
//	m_b_inc_pos = false;
	m_n_exec_step = 0;

	SetMsgType(LINE_MSG);


	this->SetFlag(FLAG_AXIS_MOVE, true);  //有轴移动指令

	if(target == source)
		this->SetFlag(FLAG_POINT_MOVE, true);   //点动移动

}

/**
 * @brief 析构函数
 */
LineMsg::~LineMsg(){
//	if(this->m_p_pmc_target)
//		delete []this->m_p_pmc_target;
}

void LineMsg::Execute(){

}

void LineMsg::GetData(void *rec){

}

void LineMsg::SetData(void *rec){

}

/**
 * @brief 设置PMC轴的运动数据
 * @param pmc_count ： PMC轴数
 */
void LineMsg::SetPmcAxisCount(uint8_t pmc_mask){
    this->m_n_pmc_count = pmc_mask;

//    if(m_n_pmc_count > 0){
//		SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
//		SetFlag(FLAG_BLOCK_OVER, true);		//块结束
//	}
}

/**
 * @brief 获取PMC轴的运动目标数据
 * @param count[out] ： PMC轴数
 * @param mask[out] ： 轴的mask
 * @param inc[out] : 增量模式位置
 * @return 轴目标位置数组的头指针
 */
//double *LineMsg::GetPmcTarget(uint8_t &count, uint32_t &mask, bool &inc){
//	count =  this->m_n_pmc_count;
//	mask = this->m_pmc_move_mask;
//	inc = this->m_b_inc_pos;
//	return this->m_p_pmc_target;
//}


/**
 * @brief 更新刀偏后同步目标位置
 * @param pos : 更新刀偏后的当前工件坐标系位置
 */
void LineMsg::RefreshTargetPos(DPointChn &pos){
	if(this->m_axis_move_mask == 0)
		return;

	printf("LineMsg::RefreshTargetPos, line=%llu, tar(%lf, %lf, %lf), newpos(%lf, %lf, %lf)\n", this->GetLineNo(), m_point_target.m_df_point[0],
			m_point_target.m_df_point[1], m_point_target.m_df_point[2], pos.m_df_point[0], pos.m_df_point[1], pos.m_df_point[2]);
	this->m_point_source = pos;  //同步起始位置

	for(int i = 0; i < 8; i++){
		if(m_axis_move_mask & (0x01<<i)){
			continue;
		}
		m_point_target.m_df_point[i] = pos.m_df_point[i];	//同步目标位置
	}

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @param flag : true--反向    false--正向
 * @return 0--成功返回   1--无需返回
 */
int LineMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;


	//移动指令类型
	data->data.cmd = MOVE_G01;

	//进给速度
	data->data.feed = FEED_TRANS(m_df_feed);   //单位由mm/min转换为um/s


	data->data.line_no = this->m_n_line_no; 	//行号

//	printf("line data: x[%lf], y[%lf]\n", m_point_target.x, m_point_target.y);

	//目标位置
	DPoint pos;
	int count = 0;
	uint32_t ms = 0x01;
	double *pp = &pos.x;
	double *ps = m_point_target.m_df_point;
	if(flag)
		ps = this->m_point_source.m_df_point;
	for(int i = 0; i < kMaxAxisChn && count < 8; i++){
        if(mask & ms){
        	*pp = ps[i];
		}
        pp++;
        count++;
		ms = ms<<1;
	}

	data->data.pos0 = MM2NM0_1(pos.x); //单位由mm转换为0.1nm
	data->data.pos1 = MM2NM0_1(pos.y);
	data->data.pos2 = MM2NM0_1(pos.z);
	data->data.pos3 = MM2NM0_1(pos.a4);
	data->data.pos4 = MM2NM0_1(pos.a5);
	data->data.pos5 = MM2NM0_1(pos.a6);
	data->data.pos6 = MM2NM0_1(pos.a7);
	data->data.pos7 = MM2NM0_1(pos.a8);

    ///printf("-----------------------> LineMsg pos %lf -- %lf -- %lf mask: %d\n", pos.x, pos.y, pos.z, mask);
	//printf("posnm %lld -- %lld -- %lld \n", data->data.pos0, data->data.pos1, data->data.pos2);

	data->data.reserved = this->m_io_data;   //IO数据

	//Ext_type
	if(this->m_n_flags.bits.step_flag)  //单段标志
		data->data.ext_type = 0x20;
	else
		data->data.ext_type = 0;

	if(this->m_n_flags.bits.jump_flag)
		data->data.ext_type |=0x100;   //bit8：跳段标志


	return res;
}

/**
 * @brief 生成仿真数据包
 * @param data[out] : 返回仿真数据
 * @return 0--成功返回   1--无需返回
 */
int LineMsg::GetSimulateData(CompilerSimData &data){

	data.type = 1;

	data.target[0] = this->m_point_target.m_df_point[0];
	data.target[1] = this->m_point_target.m_df_point[1];
	data.target[2] = this->m_point_target.m_df_point[2];

	return 0;
}

void LineMsg::PrintString(){
	printf("[%lld]LineMsg, dest[%lf, %lf, %lf]\n", m_n_line_no, m_point_target.m_df_point[0], m_point_target.m_df_point[1], m_point_target.m_df_point[2]);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
LineMsg& LineMsg::operator=( const LineMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_point_target = msg.m_point_target;
	this->m_point_source = msg.m_point_source;
	this->m_io_data = msg.m_io_data;

    this->m_n_pmc_count = msg.m_n_pmc_count;
//	this->m_pmc_move_mask = msg.m_pmc_move_mask;
//	this->m_b_inc_pos = msg.m_b_inc_pos;
	this->m_n_exec_step = msg.m_n_exec_step;
//	if(this->m_p_pmc_target){
//		delete []m_p_pmc_target;
//		m_p_pmc_target = nullptr;
//	}
//	if(m_n_pmc_count > 0){
//		m_p_pmc_target = new double[m_n_pmc_count];
//		if(m_p_pmc_target){
//			memcpy(m_p_pmc_target, msg.m_p_pmc_target, sizeof(double)*m_n_pmc_count);
//		}
//	}

	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const LineMsg &one, LineMsg &two){

	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_point_target == two.m_point_target &&
			one.m_point_source == two.m_point_source &&
			one.m_io_data == two.m_io_data &&
            one.m_n_pmc_count == two.m_n_pmc_count)
		return true;
	return false;
}
/*****************************************************************ArcMsg类***************************************************************/
/**
 * @brief 圆弧消息构造函数
 * @param code
 * @param source
 * @param target
 * @param center
 * @param radius
 * @param dir_flag
 * @param major_flag
 * @param circle_flag
 */
ArcMsg::ArcMsg(int code,
		const DPointChn& source,
        const DPointChn& target,
        const DPointChn& center,
        const double& radius,
		const double feed,
	    const uint32_t axis_mask,
        const int8_t dir_flag,
        const int8_t major_flag,
		const int8_t circle_flag):MoveMsg(code){
	this->m_point_source = source;
	this->m_point_target = target;
	this->m_point_center = center;
	this->m_df_radius = radius;
	this->m_flag_direct = dir_flag;
	this->m_flag_major = major_flag;
	this->m_flag_circle = circle_flag;
	this->m_df_feed = feed;
	this->m_axis_move_mask = axis_mask;
	m_io_data = 0;
	this->m_gmode_2 = G17_CMD;   //默认G17平面
	SetMsgType(ARC_MSG);

//	if(m_flag_circle || target != source)
	this->SetFlag(FLAG_AXIS_MOVE, true);  //有轴移动指令
}

void ArcMsg::Execute(){

}

void ArcMsg::GetData(void *rec){

}

void ArcMsg::SetData(void *rec){

}

/**
 * @brief 更新刀偏后同步目标位置
 * @param pos : 更新刀偏后的当前工件坐标系位置
 */
void ArcMsg::RefreshTargetPos(DPointChn &pos){
	if(this->m_axis_move_mask == 0)
		return;

	this->m_point_source = pos;  //同步起始位置

	for(int i = 0; i < 8; i++){
		if(m_axis_move_mask & (0x01<<i)){
			continue;
		}
		m_point_target.m_df_point[i] = pos.m_df_point[i];	//同步目标位置
	}

	//重新计算圆心
	this->CalArcCenter(m_point_source, m_point_target, m_df_radius, m_flag_direct*m_flag_major, m_gmode_2, this->m_point_center);

	printf("arcmsg::refreshtargetpos, src(%lf, %lf, %lf), tar(%lf, %lf, %lf), center(%lf, %lf, %lf)\n",
			m_point_source.m_df_point[0], m_point_source.m_df_point[1], m_point_source.m_df_point[2],
			m_point_target.m_df_point[0], m_point_target.m_df_point[1], m_point_target.m_df_point[2],
			m_point_center.m_df_point[0], m_point_center.m_df_point[1], m_point_center.m_df_point[2]);

}

/**
 * @brief 计算圆弧中心坐标
 * @param src : 圆弧起点坐标
 * @param tar ：圆弧终点坐标
 * @param radius ：圆弧半径
 * @param flag ： -1表示顺时针，1表示逆时针
 * @param gmode_2 : 第二组G模态，G17、G18、G19
 * @param [out]center : 输出计算出的圆弧中心坐标
 * @return true--成功   false--失败
 */
bool ArcMsg::CalArcCenter(const DPointChn &src, const DPointChn &tar, const double &radius, const int8_t flag, const uint16_t gmode_2, DPointChn &cen){
	DPlane source = Point2Plane(src, gmode_2);
	DPlane target = Point2Plane(tar, gmode_2);


	DPlane mid = source + target;
	mid /= 2;   //弦中心点
	DPlane dd = mid - source;
	double dxy = dd.x * dd.x + dd.y * dd.y;
	double dr2 = radius * radius - dxy;
	if(dr2 <= MZERO)
		dr2 = 0.0;
	double dr = sqrt(dxy); //一半的弦长
	if((dr-radius) > 2E-3){  //弦长大于直径，无法自洽，告警
		printf("cal raius failed : %lf, %lf\n", dr, radius);
		return false;
	}
//	printf("src(%lf, %lf), tar(%lf, %lf), L/2= %lf, radius=%lf\n", source.x, source.y, target.x, target.y, dr, radius);
	double  angle = GetVectAngle(target, source) + flag * M_PI_2;
//	printf("angle = %lf\n", angle);
	dr = sqrt(dr2);
//	printf("dr = %lf, dr2=%lf\n", dr, dr2);
	DPlane  move(dr * cos(angle), dr * sin(angle) );
	DPlane center = mid + move;
//	printf("mid(%lf, %lf), move(%lf, %lf)\n", mid.x, mid.y, move.x, move.y);
	cen = src;
	Plane2Point(center, cen, gmode_2);
	printf("cal arc center: (%lf, %lf, %lf)\n", cen.m_df_point[0], cen.m_df_point[1], cen.m_df_point[2]);

	return true;
}

void ArcMsg::SetPmcAxisCount(uint8_t pmc_count){
    this->m_n_pmc_count = pmc_count;

//    if(m_n_pmc_count > 0){
//        SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
//        SetFlag(FLAG_BLOCK_OVER, true);		//块结束
//    }
}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @param flag : 反向执行标志，true--反向，false--正向，主要用于手轮正反向跟踪
 * @return 0--成功返回   1--无需返回
 */
int ArcMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;

	//移动指令类型
	if(m_n_g_code == G02_CMD){
		if(flag)
			data->data.cmd = MOVE_G03;
		else
			data->data.cmd = MOVE_G02;
	}
	else if(m_n_g_code == G03_CMD){
		if(flag)
			data->data.cmd = MOVE_G02;
		else
			data->data.cmd = MOVE_G03;
	}

	//进给速度
	data->data.feed = FEED_TRANS(m_df_feed);   //单位由mm/min转换为um/s

	data->data.line_no = this->m_n_line_no; 	//行号

	//目标位置
	DPoint pos;
	int count = 0;
	uint32_t ms = 0x01;
	double *pp = &pos.x;
	double *ps = m_point_target.m_df_point;
	if(flag)
		ps = this->m_point_source.m_df_point;
	for(int i = 0; i < kMaxAxisChn && count < 8; i++){
		if(mask & ms){
			*pp = ps[i];
		}
        pp++;
        count++;
		ms = ms<<1;
	}

	data->data.pos0 = MM2NM0_1(pos.x); //单位由mm转换为0.1nm
	data->data.pos1 = MM2NM0_1(pos.y);
	data->data.pos2 = MM2NM0_1(pos.z);
	data->data.pos3 = MM2NM0_1(pos.a4);
	data->data.pos4 = MM2NM0_1(pos.a5);
	data->data.pos5 = MM2NM0_1(pos.a6);
	data->data.pos6 = MM2NM0_1(pos.a7);
	data->data.pos7 = MM2NM0_1(pos.a8);

	//圆心坐标
	data->data.arc_center0 = MM2NM0_1(this->m_point_center.m_df_point[0]);
	data->data.arc_center1 = MM2NM0_1(this->m_point_center.m_df_point[1]);
	data->data.arc_center2 = MM2NM0_1(this->m_point_center.m_df_point[2]);

	//半径
	data->data.arc_radius = MM2NM0_1(this->m_df_radius);

	data->data.reserved = this->m_io_data;  //IO数据

	//Ext_type
	data->data.ext_type = 0x00;
	//设置优劣弧标志， bit3: 0-表示劣弧  1-表示优弧
	if(this->m_flag_major == -1)		//优弧
		data->data.ext_type |= 0x08;
	//设置整圆标志，bit4: 0-表示圆弧   1-表示整圆
	if(this->m_flag_circle)
		data->data.ext_type |= 0x10;	//整圆

	if(this->m_n_flags.bits.step_flag)
		data->data.ext_type |= 0x20;	//bit5:单段标志

	if(this->m_n_flags.bits.jump_flag)
		data->data.ext_type |=0x100;   //bit8：跳段标志

	return res;
}

/**
 * @brief 生成仿真数据包
 * @param data[out] : 返回仿真数据
 * @return 0--成功返回   1--无需返回
 */
int ArcMsg::GetSimulateData(CompilerSimData &data){
	if(m_n_g_code == G02_CMD)
		data.type = 2;
	else if(m_n_g_code == G03_CMD)
		data.type = 3;

	//终点
	data.target[0] = this->m_point_target.m_df_point[0];
	data.target[1] = this->m_point_target.m_df_point[1];
	data.target[2] = this->m_point_target.m_df_point[2];

	//圆心
	data.center[0] = this->m_point_center.m_df_point[0];
	data.center[1] = this->m_point_center.m_df_point[1];
	data.center[2] = this->m_point_center.m_df_point[2];

	//半径
	data.radius = this->m_df_radius;

	if(this->m_flag_major == -1){//优弧， radius返回负值
		if(data.radius > 0)
			data.radius *= -1;
	}
	else if(this->m_flag_major == 1){  //劣弧， radius返回正值
		if(data.radius < 0)
			data.radius *= -1;
	}

	if(this->m_point_target == this->m_point_source &&
			this->m_flag_circle == 0)   //非整圆
		data.radius = 0;

	data.plane = this->m_gmode_2;  //当前平面
	
	return 0;
}

void ArcMsg::PrintString(){
	printf("[%lld]ArcMsg, radius = %lf, dir = %d, major_flag = %d, circle_flag = %d\n",
			m_n_line_no, m_df_radius, m_flag_direct, m_flag_major, m_flag_circle);

	double *tar = m_point_target.m_df_point;
	double *src = m_point_source.m_df_point;
	double *cen = m_point_center.m_df_point;

	printf("target: %lf %lf src %lf %lf center: %lf %lf\n",
			*tar, *(tar+1), *src, *(src+1), *cen, *(cen+1));
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
ArcMsg& ArcMsg::operator=( const ArcMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_flag_direct = msg.m_flag_direct;
	this->m_flag_major = msg.m_flag_major;
	this->m_flag_circle = msg.m_flag_circle;
	this->m_df_radius = msg.m_df_radius;
	this->m_point_center = msg.m_point_center;
	this->m_point_source = msg.m_point_source;
	this->m_point_target = msg.m_point_target;
	this->m_io_data = msg.m_io_data;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const ArcMsg &one, ArcMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_flag_direct == two.m_flag_direct &&
			one.m_flag_major == two.m_flag_major &&
			one.m_flag_circle == two.m_flag_circle &&
			one.m_df_radius == two.m_df_radius &&
			one.m_point_center == two.m_point_center &&
			one.m_point_source == two.m_point_source &&
			one.m_point_target == two.m_point_target &&
			one.m_io_data == two.m_io_data)
		return true;
	return false;
}

/*****************************************************************SpindleCheckMsg类***************************************************************/
SpindleCheckMsg::SpindleCheckMsg(int gcode, int p, int q, int r, int i):ModeMsg(gcode){
	this->m_delay_time = p;
	this->m_allowed_tolerance = q;
	this->m_warn_ratio = r;
	this->m_warn_speed = i;
	SetMsgType(SPD_CHK_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
}

void SpindleCheckMsg::Execute(){

}

void SpindleCheckMsg::GetData(void *rec){

}

void SpindleCheckMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int SpindleCheckMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 1;

	return res;
}

void SpindleCheckMsg::PrintString(){
	printf("[%lld]SpindleCheckMsg\n", m_n_line_no);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
SpindleCheckMsg& SpindleCheckMsg::operator=( const SpindleCheckMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_g_code = msg.m_n_g_code;
	this->m_delay_time = msg.m_delay_time;
	this->m_allowed_tolerance = msg.m_allowed_tolerance;
	this->m_warn_ratio = msg.m_warn_ratio;
	this->m_warn_speed = msg.m_warn_speed;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const SpindleCheckMsg &one, SpindleCheckMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_g_code ==  two.m_n_g_code &&
			one.m_delay_time == two.m_delay_time &&
			one.m_allowed_tolerance == two.m_allowed_tolerance &&
			one.m_warn_ratio == two.m_warn_ratio &&
			one.m_warn_speed == two.m_warn_speed)
		return true;
	return false;
}

/*****************************************************************MacroCmdMsg类***************************************************************/
/**
 * @brief 构造函数
 * @param macro
 */
MacroCmdMsg::MacroCmdMsg(LexerMacroCmd *macro){
	SetMsgType(MACRO_MSG);
	this->m_n_run_step = 0;
	this->m_ln_offset = 0;
	//this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
	if(macro == nullptr){
		this->m_macro_cmd = MACRO_CMD_INVALID;
		while(!m_macro_expression[0].empty())
			m_macro_expression[0].pop();
		while(!m_macro_expression[1].empty())
			m_macro_expression[1].pop();
		return;
	}
	m_macro_cmd = macro->cmd;
	m_macro_expression[0].swap(macro->macro_expression[0]);
	m_macro_expression[1].swap(macro->macro_expression[1]);
	this->m_macro_exp_res[0].init = false;
	this->m_macro_exp_res[1].init = false;
	this->m_b_exp_cal[0] = false;
	this->m_b_exp_cal[1] = false;

}

void MacroCmdMsg::Execute(){

}

void MacroCmdMsg::GetData(void *rec){

}

void MacroCmdMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int MacroCmdMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 1;

	return res;
}

void MacroCmdMsg::PrintString(){
	printf("[%lld]MacroCmdMsg\n", m_n_line_no);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
MacroCmdMsg& MacroCmdMsg::operator=(MacroCmdMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_macro_cmd = msg.m_macro_cmd;
//	this->m_macro_expression[0] = msg.m_macro_expression[0];
//	this->m_macro_expression[1] = msg.m_macro_expression[1];
	CopyMacroExp(this->m_macro_expression[0], msg.m_macro_expression[0]);
	CopyMacroExp(this->m_macro_expression[1], msg.m_macro_expression[1]);
	this->m_macro_exp_res[0] = msg.m_macro_exp_res[0];
	this->m_macro_exp_res[1] = msg.m_macro_exp_res[1];

	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==(MacroCmdMsg &one, MacroCmdMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_macro_cmd ==  two.m_macro_cmd &&
			IsEqualMacroExp(one.m_macro_expression[0], two.m_macro_expression[0]) &&
			IsEqualMacroExp(one.m_macro_expression[1], two.m_macro_expression[1]))
		return true;
	return false;
}

/*****************************************************************PolarIntpMsg类***************************************************************/
PolarIntpMsg::PolarIntpMsg(int gcode, int p, int r, int l, int q, int x, int y, int i, int j):ModeMsg(gcode){
	this->m_n_p = p;
	this->m_n_r = r;
	this->m_n_l = l;
	this->m_n_q = q;
	this->m_n_start_x = x;
	this->m_n_start_y = y;
	this->m_n_vect_i = i;
	this->m_n_vect_j = j;

	this->m_n_exec_step = 0;

	SetMsgType(POLAR_INTP_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
}

void PolarIntpMsg::Execute(){

}

void PolarIntpMsg::GetData(void *rec){

}

void PolarIntpMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int PolarIntpMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 1;

	return res;
}

void PolarIntpMsg::PrintString(){
	printf("[%lld]PolarIntpMsg\n", m_n_line_no);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
PolarIntpMsg& PolarIntpMsg::operator=( const PolarIntpMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_g_code = msg.m_n_g_code;
	this->m_n_p = msg.m_n_p;
	this->m_n_r = msg.m_n_r;
	this->m_n_l = msg.m_n_l;
	this->m_n_q = msg.m_n_q;
	this->m_n_start_x = msg.m_n_start_x;
	this->m_n_start_y = msg.m_n_start_y;
	this->m_n_vect_i = msg.m_n_vect_i;
	this->m_n_vect_j = msg.m_n_vect_j;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const PolarIntpMsg &one, PolarIntpMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_g_code ==  two.m_n_g_code &&
			one.m_n_p == two.m_n_p &&
			one.m_n_r == two.m_n_r &&
			one.m_n_l == two.m_n_l &&
			one.m_n_q == two.m_n_q &&
			one.m_n_start_x == two.m_n_start_x &&
			one.m_n_start_y == two.m_n_start_y &&
			one.m_n_vect_i == two.m_n_vect_i &&
			one.m_n_vect_j == two.m_n_vect_j)
		return true;
	return false;
}


/*****************************************************************ClearCirclePosMsg类***************************************************************/
ClearCirclePosMsg::ClearCirclePosMsg(int gcode, uint32_t axis_mask, int loop):ModeMsg(gcode){
	this->m_axis_mask = axis_mask;
	this->m_n_circle_loop = loop;

	m_n_exec_step = 0;
	this->m_n_g_code = 2000;  //G200

	SetMsgType(CLEAR_CIRCLE_POS_MSG);

	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
}

void ClearCirclePosMsg::Execute(){

}

void ClearCirclePosMsg::GetData(void *rec){

}

void ClearCirclePosMsg::SetData(void *rec){

}

/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int ClearCirclePosMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void ClearCirclePosMsg::PrintString(){
	printf("[%lld]ClearCirclePosMsg[%d]\n", m_n_line_no, m_n_g_code);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
ClearCirclePosMsg& ClearCirclePosMsg::operator=( const ClearCirclePosMsg& msg){
	if(&msg == this)
		return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_g_code = msg.m_n_g_code;
	this->m_axis_mask = msg.m_axis_mask;
	this->m_n_circle_loop = msg.m_n_circle_loop;
	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const ClearCirclePosMsg &one, ClearCirclePosMsg &two){
	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_g_code == two.m_n_g_code &&
			one.m_axis_mask == two.m_axis_mask &&
			one.m_n_circle_loop == two.m_n_circle_loop)
		return true;
	return false;
}

#ifdef USES_SPEED_TORQUE_CTRL
/*****************************************************************SpeedCtrlMsg类***************************************************************/
/**
 * @brief 构造函数
 * @param target : 目标位置
 * @param count ： PMC轴数
 * @param axis_mask ： 轴的mask
 */
SpeedCtrlMsg::SpeedCtrlMsg(int gcode, double *target, const uint8_t count, const uint32_t axis_mask):ModeMsg(gcode){
	m_p_target_speed = nullptr;
	
	m_axis_move_mask = axis_mask;
	m_n_axis_count = count;
	if(count > 0 && target != nullptr){
		m_p_target_speed = new double[count];
		if(m_p_target_speed){
			uint8_t tc = 0;
			for(uint8_t i = 0; i < kMaxAxisChn && tc < count; i++){
				if(axis_mask & (0x01<<i)){
					m_p_target_speed[tc++] = target[i];
				}
			}
		//	memcpy(m_p_target_speed, target, sizeof(double)*count);
		}
	}

	m_n_exec_step = 0;

	SetMsgType(SPEED_CTRL_MSG);

	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
}

/**
 * @brief 析构函数
 */
SpeedCtrlMsg::~SpeedCtrlMsg(){
	if(this->m_p_target_speed)
		delete []m_p_target_speed;
}

void SpeedCtrlMsg::Execute(){

}

void SpeedCtrlMsg::GetData(void *rec){

}

void SpeedCtrlMsg::SetData(void *rec){

}


/**
 * @brief 获取PMC轴的运动目标数据
 * @param count[out] ： PMC轴数
 * @param mask[out] ： 轴的mask
 * @return 轴目标位置数组的头指针
 */
double *SpeedCtrlMsg::GetTargetValue(uint8_t &count, uint32_t &mask){
	count =  this->m_n_axis_count;
	mask = this->m_axis_move_mask;

	return this->m_p_target_speed;
}


/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int SpeedCtrlMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;

	return res;
}

/**
 * @brief 统一指令单位
 * @param data
 * @return 0--成功返回   1--无需返回
 */
void  SpeedCtrlMsg::ChangeUint(){
  for(uint8_t i=0; i<8; i++){
  if(this->m_axis_move_mask &(0x01<<i)){

  	}
  }
}
	

void SpeedCtrlMsg::PrintString(){
	printf("[%lld]SpeedCtrlMsg[%d], speed[%lf, %lf, %lf]\n", m_n_line_no, this->m_n_g_code/10, m_p_target_speed[0], m_p_target_speed[1], m_p_target_speed[2]);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
SpeedCtrlMsg& SpeedCtrlMsg::operator=( const SpeedCtrlMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_p_target_speed = msg.m_p_target_speed;

	this->m_n_axis_count = msg.m_n_axis_count;
	this->m_axis_move_mask = msg.m_axis_move_mask;

	if(m_p_target_speed){
		delete []m_p_target_speed;
		m_p_target_speed = nullptr;
	}
	if(m_n_axis_count > 0){
		m_p_target_speed = new double[m_n_axis_count];
		if(m_p_target_speed){
			memcpy(m_p_target_speed, msg.m_p_target_speed, sizeof(double)*m_n_axis_count);
		}
	}

	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const SpeedCtrlMsg &one, SpeedCtrlMsg &two){
	if(one.m_n_axis_count == two.m_n_axis_count &&
			one.m_axis_move_mask == two.m_axis_move_mask){
		for(uint8_t i = 0; i < one.m_n_axis_count; i++){
			if(one.m_p_target_speed[i] != two.m_p_target_speed[i])
				return false;
		}
	}else
		return false;

	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all)
		return true;
	return false;
}



/*****************************************************************SpeedCtrlMsg类***************************************************************/
/**
 * @brief 构造函数
 * @param target : 目标位置
 * @param count ： PMC轴数
 * @param axis_mask ： 轴的mask
 */
TorqueCtrlMsg::TorqueCtrlMsg(int gcode, double *target, const uint8_t count, const uint32_t axis_mask, uint32_t time):ModeMsg(gcode){
	m_p_target_torque = nullptr;

	m_axis_move_mask = axis_mask;
	m_n_axis_count = count;

	m_n_exec_step = 0;

	m_n_check_count = 0;

	m_n_timeover = time;

	if(count > 0 && target != nullptr){
		this->m_p_target_torque = new double[count];
		if(this->m_p_target_torque){
			memcpy(m_p_target_torque, target, sizeof(double)*count);
		}
	}

	SetMsgType(TORQUE_CTRL_MSG);

    if(gcode == G2001_CMD || gcode == G2002_CMD || gcode == G2003_CMD){
	   SetFlag(FLAG_WAIT_MOVE_OVER, true);   //需要等待运动到位
    }else{
	   SetFlag(FLAG_WAIT_MOVE_OVER, false);   //不用等待运动到位
    }
}

/**
 * @brief 析构函数
 */
TorqueCtrlMsg::~TorqueCtrlMsg(){
	if(m_p_target_torque)
		delete []m_p_target_torque;
}

void TorqueCtrlMsg::Execute(){

}

void TorqueCtrlMsg::GetData(void *rec){

}

void TorqueCtrlMsg::SetData(void *rec){

}


/**
 * @brief 获取PMC轴的运动目标数据
 * @param count[out] ： PMC轴数
 * @param mask[out] ： 轴的mask
 * @return 轴目标位置数组的头指针
 */
double *TorqueCtrlMsg::GetTargetValue(uint8_t &count, uint32_t &mask){
	count =  this->m_n_axis_count;
	mask = this->m_axis_move_mask;

	return this->m_p_target_torque;
}


/**
 * @brief 生成待输出给MC的运动控制数据包
 * @param data
 * @param mask : 通道插补轴掩码
 * @return 0--成功返回   1--无需返回
 */
int TorqueCtrlMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;




	return res;
}

void TorqueCtrlMsg::PrintString(){
	printf("[%lld]TorqueCtrlMsg[%d], speed[%lf, %lf, %lf]\n", m_n_line_no, this->m_n_g_code/10, m_p_target_torque[0], m_p_target_torque[1], m_p_target_torque[2]);
}

/**
 * @brief 赋值运算符
 * @param msg
 * @return
 */
TorqueCtrlMsg& TorqueCtrlMsg::operator=( const TorqueCtrlMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->m_n_timeover = msg.m_n_timeover;

	this->m_n_axis_count = msg.m_n_axis_count;
	this->m_axis_move_mask = msg.m_axis_move_mask;

	if(this->m_p_target_torque){
		delete []m_p_target_torque;
		m_p_target_torque = nullptr;
	}
	if(m_n_axis_count > 0){
		m_p_target_torque = new double[m_n_axis_count];
		if(m_p_target_torque){
			memcpy(m_p_target_torque, msg.m_p_target_torque, sizeof(double)*m_n_axis_count);
		}
	}

	return *this;
}

/**
 * @brief 判断运算符
 * @param one
 * @param two
 * @return
 */
bool operator ==( const TorqueCtrlMsg &one, TorqueCtrlMsg &two){
	if(one.m_n_axis_count == two.m_n_axis_count &&
			one.m_axis_move_mask == two.m_axis_move_mask){
		for(uint8_t i = 0; i < one.m_n_axis_count; i++){
			if(one.m_p_target_torque[i] != two.m_p_target_torque[i])
				return false;
		}
	}else
		return false;

	if(one.m_n_type == two.m_n_type &&
			one.m_n_line_no == two.m_n_line_no &&
			one.m_n_flags.all == two.m_n_flags.all &&
			one.m_n_timeover == two.m_n_timeover)
		return true;
	return false;
}

#endif

// @add zk
InputMsg::InputMsg():RecordMsg(){
	SetMsgType(INPUT_MSG);
}

InputMsg::~InputMsg(){

}

void InputMsg::Execute(){}
void InputMsg::GetData(void* rec ){}
void InputMsg::SetData(void* rec){}
//生成待输出给MC的运动控制数据包
int InputMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag)
{
	return 0;
}
void InputMsg::PrintString(){}

InputMsg& InputMsg::operator=( const InputMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;

	return *this;
}


ExactStopMsg::ExactStopMsg(const DPointChn &source, const DPointChn &target, const uint32_t axis_mask):RecordMsg(){
	SetMsgType(EXACT_STOP_MSG);
	m_point_source = source;
	m_point_target = target;
	m_axis_mask = axis_mask;
	m_n_flags.all = 71;
}

ExactStopMsg::~ExactStopMsg(){

}

void ExactStopMsg::Execute(){}
void ExactStopMsg::GetData(void* rec ){}
void ExactStopMsg::SetData(void* rec){}

int ExactStopMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag)
{
	return 0;
}
void ExactStopMsg::PrintString(){}

ExactStopMsg& ExactStopMsg::operator=( const ExactStopMsg& msg){
	if(&msg == this)
		return *this;

	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;

	m_point_source = msg.m_point_source;
	m_point_target = msg.m_point_target;
	m_axis_mask = msg.m_axis_mask;

	return *this;
}

OpenFileMsg::OpenFileMsg(bool end):RecordMsg(){
	SetMsgType(OPEN_FILE_MSG);
	order_end = end;
}

OpenFileMsg::~OpenFileMsg(){

}

void OpenFileMsg::Execute(){}
void OpenFileMsg::GetData(void* rec ){}
void OpenFileMsg::SetData(void* rec){}
//生成待输出给MC的运动控制数据包
int OpenFileMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag)
{
	return 0;
}
void OpenFileMsg::PrintString(){}

OpenFileMsg& OpenFileMsg::operator=( const OpenFileMsg& msg){
	if(&msg == this)
		return *this;
	this->m_n_type = msg.m_n_type;
	this->m_n_line_no = msg.m_n_line_no;
	this->m_n_flags.all = msg.m_n_flags.all;
	this->OData = msg.OData;

	return *this;
}

// @add zk


