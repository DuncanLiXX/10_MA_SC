/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file Compiler_data.h
 *@author gonghao
 *@date 2020/04/13
 *@brief ��ͷ�ļ�����G������������ɵ�������Ϣ��InfoMessage
 *@version
 */

#include "compile_message.h"

//template<> int ListNode<RecordMsg *>::new_count = 0;

/*****************************************************************RecordMsg��***************************************************************/

/**
 * @brief ���캯��
 */
RecordMsg::RecordMsg()/*:m_p_next(nullptr), m_p_prev(nullptr)*/{
	this->m_n_type = NORMAL_MSG;
	this->m_n_line_no = 0;
	this->m_n_flags.all = 0;
	this->m_n_frame_index = 0;
    Singleton<ShowSc>::instance().NewMsg();
}

/**
 * @brief ��������
 */
RecordMsg::~RecordMsg(){
    Singleton<ShowSc>::instance().DeleteMsg();
}

/**
 * @brief ���ö�Ӧ��־��״̬
 * @param flag: ָ�����õı�־����
 * @param value���趨��ֵ
 */
void RecordMsg::SetFlag(RecordFlag flag, bool value){
	if(value)
		m_n_flags.all |= flag;
	else
		m_n_flags.all &= (~flag);
}

/**
 * @brief �ڵ�ǰ��Ϣ������µ���Ϣ
 * @param msg : �����������Ϣ����ָ��
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
 * @brief �������Ƴ�����
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
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************ErrorMsg��***************************************************************/

ErrorMsg::ErrorMsg(int err_code){
	m_error_code = err_code;
//	this->m_b_clear_info = false;
	this->m_n_info_type = 1;
	SetMsgType(ERROR_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
}

ErrorMsg::ErrorMsg(){
	m_error_code = ERR_NONE;
//	this->m_b_clear_info = false;
	this->m_n_info_type = 1;
	SetMsgType(ERROR_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
}


void ErrorMsg::Execute(){

}

void ErrorMsg::GetData(void *rec){

}


void ErrorMsg::SetData(void *rec){

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int ErrorMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void ErrorMsg::PrintString(){
	printf("[%lld]ErrorMsg[%d]\n", m_n_line_no, m_error_code);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************RestartOverMsg��***************************************************************/

RestartOverMsg::RestartOverMsg(){

	SetMsgType(RESTART_OVER_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
	this->SetFlag(FLAG_STEP, false);    //���β�ͣ
}



void RestartOverMsg::Execute(){

}

void RestartOverMsg::GetData(void *rec){

}


void RestartOverMsg::SetData(void *rec){

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int RestartOverMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void RestartOverMsg::PrintString(){
	printf("[%lld]RestartOverMsg\n", m_n_line_no);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************ModeMsg��***************************************************************/
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
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int ModeMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void ModeMsg::PrintString(){
	printf("[%lld]ModeMsg[%d]\n", m_n_line_no, m_n_g_code);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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


/*****************************************************************MoveMsg��***************************************************************/
MoveMsg::MoveMsg(int gcode):ModeMsg(gcode){
	this->m_b_mach_coord = false;   //Ĭ��Ϊ��������ϵ
	SetMsgType(MOVE_MSG);
}

void MoveMsg::Execute(){

}

void MoveMsg::GetData(void *rec){

}

void MoveMsg::SetData(void *rec){

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int MoveMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void MoveMsg::PrintString(){
	printf("[%lld]MoveMsg[%d], mach_coord=%hhu\n", m_n_line_no, m_n_g_code, m_b_mach_coord);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************CoordMsg��***************************************************************/
CoordMsg::CoordMsg(const DPointChn &pos, const DPointChn &src, int gcode, uint32_t axis_mask) : ModeMsg(gcode){
	this->m_pos = pos;
	this->m_pos_src = src;
	this->m_n_axis_mask = axis_mask;
	this->m_n_exec_step = 0;
	SetMsgType(COORD_MSG);

	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
	if(gcode == G53_CMD && axis_mask != 0){
		SetFlag(FLAG_AXIS_MOVE, true);   //���ƶ�ָ���־
	}
}

void CoordMsg::Execute(){

}

void CoordMsg::GetData(void *rec){

}

void CoordMsg::SetData(void *rec){

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data[out] : �������
 * @param mask : ͨ���岹������
 * @param flag : �Ƿ����������
 * @return 0--�ɹ�����   1--���践��
 */
int CoordMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;
	if(this->m_n_g_code != G53_CMD)
		return 1;

	//�ƶ�ָ������
	data->data.cmd = MOVE_G00;

	data->data.feed = 0;  //G00����Ҫ���ͽ����ٶ�


	data->data.line_no = this->m_n_line_no; 	//�к�

	//Ŀ��λ��
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
	data->data.pos0 = MM2NM0_1(pos.x); //��λ��mmת��Ϊ0.1nm
	data->data.pos1 = MM2NM0_1(pos.y);
	data->data.pos2 = MM2NM0_1(pos.z);
	data->data.pos3 = MM2NM0_1(pos.a4);
	data->data.pos4 = MM2NM0_1(pos.a5);
	data->data.pos5 = MM2NM0_1(pos.a6);
	data->data.pos6 = MM2NM0_1(pos.a7);
	data->data.pos7 = MM2NM0_1(pos.a8);

	//Ext_type
	if(this->m_n_flags.bits.step_flag)  //���α�־
		data->data.ext_type = 0x20;
	else
		data->data.ext_type = 0;

	if(this->m_n_flags.bits.jump_flag)
		data->data.ext_type |=0x100;   //bit8�����α�־

	return res;
}

/**
 * @brief ���ɷ������ݰ�
 * @param data[out] : ���ط������ݣ����ص������Ϊ��������ϵ
 * @return 0--�ɹ�����   1--���践��
 */
int CoordMsg::GetSimulateData(CompilerSimData &data){
	data.type = 0;

	data.target[0] = m_pos.m_df_point[0];
	data.target[1] = m_pos.m_df_point[1];
	data.target[2] = m_pos.m_df_point[2];

	return 0;
}

/**
 * @brief ��ȡ��ά����ʽ������ֵ
 * @param pos[out] : �������ֵ
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
 * @brief ��ֵ�����
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
 * @brief �ж������
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


/*****************************************************************LoopMsg��***************************************************************/
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
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int LoopMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void LoopMsg::PrintString(){
	printf("[%lld]LoopMsg[%d]\n", m_n_line_no, m_n_g_code);
}

/**
 * @brief �����ӳ����
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
 * @brief �����ӳ�����
 * @param name[out] : ����ӳ��������ַ���
 * @param abs_path[in] : true--����·��   false--���·��
 */
//void LoopMsg::GetMacroProgName(char *name, bool abs_path){
//	int macro_index = this->GetMacroProgIndex();
//	if(abs_path){
//		if(m_n_macro_prog_type == 2){//ͬĿ¼���û������
//			if(macro_index <= 9999)
//				sprintf(name, "%sO%04d.nc", PATH_NC_FILE, macro_index);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.nc", PATH_NC_FILE, macro_index);
//		}
//		else if(m_n_macro_prog_type == 3){//ϵͳ�����
//			if(macro_index <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.nc", PATH_NC_FILE, macro_index);
//			else
//				sprintf(name, "%ssys_sub/O%d.nc", PATH_NC_FILE, macro_index);
//		}else if(m_n_macro_prog_type == 4){
//			if(macro_index <= 9999)
//				sprintf(name, "%sO%04d.iso", PATH_NC_FILE, macro_index);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.iso", PATH_NC_FILE, macro_index);
//		}else if(m_n_macro_prog_type == 5){
//			if(macro_index <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.iso", PATH_NC_FILE, macro_index);
//			else
//				sprintf(name, "%ssys_sub/O%d.iso", PATH_NC_FILE, macro_index);
//		}else if(m_n_macro_prog_type == 6){//ͬĿ¼���û������
//			if(macro_index <= 9999)
//				sprintf(name, "%sO%04d.NC", PATH_NC_FILE, macro_index);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.NC", PATH_NC_FILE, macro_index);
//		}else if(m_n_macro_prog_type == 7){//ϵͳ�����
//			if(macro_index <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.NC", PATH_NC_FILE, macro_index);
//			else
//				sprintf(name, "%ssys_sub/O%d.NC", PATH_NC_FILE, macro_index);
//		}else if(m_n_macro_prog_type == 8){
//			if(macro_index <= 9999)
//				sprintf(name, "%sO%04d.ISO", PATH_NC_FILE, macro_index);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.ISO", PATH_NC_FILE, macro_index);
//		}else if(m_n_macro_prog_type == 9){
//			if(macro_index <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.ISO", PATH_NC_FILE, macro_index);
//			else
//				sprintf(name, "%ssys_sub/O%d.ISO", PATH_NC_FILE, macro_index);
//		}
//	}else{
//		if(m_n_macro_prog_type == 2){//ͬĿ¼���û������
//			if(macro_index <= 9999)
//				sprintf(name, "O%04d.nc", macro_index);   //ƴ���ļ�����
//			else
//				sprintf(name, "O%d.nc", macro_index);
//		}
//		else if(m_n_macro_prog_type == 3){//ϵͳ�����
//			if(macro_index <= 9999)
//				sprintf(name, "sys_sub/O%04d.nc", macro_index);
//			else
//				sprintf(name, "sys_sub/O%d.nc", macro_index);
//		}else if(m_n_macro_prog_type == 4){
//			if(macro_index <= 9999)
//				sprintf(name, "O%04d.iso", macro_index);   //ƴ���ļ�����
//			else
//				sprintf(name, "O%d.iso", macro_index);
//		}else if(m_n_macro_prog_type == 5){
//			if(macro_index <= 9999)
//				sprintf(name, "sys_sub/O%04d.iso", macro_index);
//			else
//				sprintf(name, "sys_sub/O%d.iso", macro_index);
//		}else if(m_n_macro_prog_type == 6){//ͬĿ¼���û������
//			if(macro_index <= 9999)
//				sprintf(name, "O%04d.NC", macro_index);   //ƴ���ļ�����
//			else
//				sprintf(name, "O%d.NC", macro_index);
//		}else if(m_n_macro_prog_type == 7){//ϵͳ�����
//			if(macro_index <= 9999)
//				sprintf(name, "sys_sub/O%04d.NC", macro_index);
//			else
//				sprintf(name, "sys_sub/O%d.NC", macro_index);
//		}else if(m_n_macro_prog_type == 8){
//			if(macro_index <= 9999)
//				sprintf(name, "O%04d.ISO", macro_index);   //ƴ���ļ�����
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
 * @brief ���ز�������
 * @param mask[out] : ���ز���mask
 * @param count[out] �� ���ز�������
 * @return ���ز���������ָ��
 */
double *LoopMsg::GetParameter(uint32_t &mask, uint8_t &count){
	mask = m_mask_param;
	count = m_n_param_count;
	return m_p_df_param;
}

/**
 * @brief ������һ���ļ��ľ���·�����������ַ�������
 * @param file[in] : �ļ�����·��
 */
void LoopMsg::SetLastProgFile(char *file){
	if(file != nullptr){
		memset(m_str_last_prog_file, 0x00, kMaxPathLen);
		strcpy(m_str_last_prog_file, file);
	}
}

/**
 * @brief ��ȡ��һ���ļ��ľ���·�����������ַ�������
 * @param file
 */
void LoopMsg::GetLastProgFile(char *file){
	if(file != nullptr){
		strcpy(file, m_str_last_prog_file);
	}
}


/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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


/*****************************************************************FeedMsg��***************************************************************/
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
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int FeedMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void FeedMsg::PrintString(){
	printf("[%lld]FeedMsg[%lf, %lf]\n", m_n_line_no, m_df_feed, m_df_last_feed);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************SpeedMsg��***************************************************************/
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
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int SpeedMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void SpeedMsg::PrintString(){
	printf("[%lld]SpeedMsg[%lf]\n", m_n_line_no, m_df_speed);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************ToolMsg��***************************************************************/
/**
 * @brief ���캯��
 * @param tool �� T��������ָ��
 * @param total ��T��������
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
 * @brief �Ƿ��״�ִ�д�������������Tָ���step����0��Ϊ�״�ִ��
 * @return true--��     false--����
 */
bool ToolMsg::IsFirstExec(){
	for(uint8_t i = 0; i < this->m_n_tool_count; i++){
		if(this->m_n_tool_exec_segment[i] > 0)
			return false;
	}

	return true;
}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int ToolMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void ToolMsg::PrintString(){
	printf("[%lld]ToolMsg[%d]\n", m_n_line_no, m_n_tool_count);
}

/**
 * @brief �����ӳ�����
 * @param name[out] : ����ӳ�����
 * @param abs_path[in] : true--����·��   false--���·��
 */
//void ToolMsg::GetSubProgName(char *name, bool abs_path){
//	if(abs_path){
//        //string dirName = m_p_file_map_info->GetDirName();
//		if(m_n_sub_prog_type == 2){//ͬĿ¼���û��ӳ���
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.nc", PATH_NC_FILE, m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//		}
//		else if(m_n_sub_prog_type == 3){//ϵͳ�ӳ���
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 4){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.iso", PATH_NC_FILE, m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 5){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 6){//ͬĿ¼���û������
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.NC", PATH_NC_FILE, m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 7){//ϵͳ�����
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 8){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.ISO", PATH_NC_FILE, m_n_sub_prog_name);   //ƴ���ļ�����
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
//		if(m_n_sub_prog_type == 2){//ͬĿ¼���û��ӳ���
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.nc", m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "O%d.nc", m_n_sub_prog_name);
//		}
//		else if(m_n_sub_prog_type == 3){//ϵͳ�ӳ���
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.nc", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.nc", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 4){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.iso", m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "O%d.iso", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 5){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.iso", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.iso", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 6){//ͬĿ¼���û������
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.NC", m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "O%d.NC", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 7){//ϵͳ�����
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.NC", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.NC", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 8){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.ISO", m_n_sub_prog_name);   //ƴ���ļ�����
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
 * @brief ��ֵ�����
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
 * @brief �ж������
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


/*****************************************************************TimeWaitMsg��***************************************************************/
TimeWaitMsg::TimeWaitMsg(uint32_t time):ModeMsg(G04_CMD){
	this->m_n_time_delay = time;
	this->m_n_start_time = 0;
	SetMsgType(TIME_WAIT_MSG);

	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
}

void TimeWaitMsg::Execute(){

}

void TimeWaitMsg::GetData(void *rec){

}

void TimeWaitMsg::SetData(void *rec){

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int TimeWaitMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void TimeWaitMsg::PrintString(){
	printf("[%lld]TimeWaitMsg[%d]\n", m_n_line_no, m_n_time_delay);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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


/*****************************************************************RefReturnMsg��***************************************************************/
/**
 * @brief ���캯��
 * @param gcode
 * @param axis_mask
 * @param mid
 */
RefReturnMsg::RefReturnMsg(int gcode, uint32_t axis_mask, DPointChn &mid):ModeMsg(gcode){
	this->m_n_axis_mask = axis_mask;
	this->m_pos_middle = mid;
	this->m_n_exec_step = 0;
	SetMsgType(REF_RETURN_MSG);

//	SetFlag(FLAG_AXIS_MOVE, true);   //���ƶ�ָ��
	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
	SetFlag(FLAG_BLOCK_OVER, true);		//�����
}

void RefReturnMsg::Execute(){

}

void RefReturnMsg::GetData(void *rec){

}

void RefReturnMsg::SetData(void *rec){

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int RefReturnMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

/**
 * @brief ���ɷ������ݰ�
 * @param data[out] : ���ط�������
 * @return 0--�ɹ�����   1--���践��
 */
int RefReturnMsg::GetSimulateData(CompilerSimData &data){
	data.type = 0;

	//�����м������
	data.target[0] = this->m_pos_middle.m_df_point[0];
	data.target[1] = this->m_pos_middle.m_df_point[1];
	data.target[2] = this->m_pos_middle.m_df_point[2];

	return 0;
}

void RefReturnMsg::PrintString(){
	printf("[%lld]RefReturnMsg\n", m_n_line_no);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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


/*****************************************************************SkipMsg��***************************************************************/
/**
 * @brief ���캯��
 * @param source
 * @param target
 * @param feed
 * @param axis_mask
 */
SkipMsg::SkipMsg(const DPointChn &source, const DPointChn &target, const double feed, const uint32_t axis_mask):LineMsg(source, target, feed, axis_mask){
	this->m_n_exec_step = 0;
	SetMsgType(SKIP_MSG);

	SetFlag(FLAG_AXIS_MOVE, true);   //���ƶ�ָ��
	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
	SetFlag(FLAG_BLOCK_OVER, true);		//�����
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
 * @brief ��ֵ�����
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
 * @brief �ж������
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


/*****************************************************************SkipMeasureMsg��***************************************************************/
/**
 * @brief ���캯��
 * @param source
 * @param target
 * @param feed
 * @param axis_mask
 */
SkipMeasureMsg::SkipMeasureMsg(const DPointChn &source, const DPointChn &target, const double feed, const uint32_t axis_mask):LineMsg(source, target, feed, axis_mask){
    this->m_n_exec_step = 0;
    SetMsgType(SKIP_MEASURE_MSG);

    SetFlag(FLAG_AXIS_MOVE, true);   //���ƶ�ָ��
    SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
    SetFlag(FLAG_BLOCK_OVER, true);		//�����
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
 * @brief ��ֵ�����
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
 * @brief �ж������
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


/*****************************************************************AutoToolMeasureMsg��***************************************************************/

/**
 * @brief ���캯��
 * @param target �� Ŀ��λ��
 * @param axis_mask �����˶�mask
 * @param times : �Ե�����
 */
AutoToolMeasureMsg::AutoToolMeasureMsg(const DPointChn &target, const uint32_t axis_mask, const int h_code, const int times):ModeMsg(G37_CMD){

	this->m_pos_target = target;
	this->m_mask_axis_move = axis_mask;
	this->m_n_times = times;
	this->m_n_h_index = h_code;

	m_n_macro_prog_name = 9037;

	this->SetMsgType(AUTO_TOOL_MEASURE_MSG);

    //SetFlag(FLAG_AXIS_MOVE, true);   //���ƶ�ָ��
	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
	SetFlag(FLAG_BLOCK_OVER, true);		//�����
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
 * @brief ���غ������
 * @param name[out] : ������ļ�����·��
 * @param abs_path[in] : true--����·��   false--���·��
 */
//void AutoToolMeasureMsg::GetMacroProgName(char *name, bool abs_path){
//	if(abs_path){
//		if(m_n_macro_prog_type == 2){//ͬĿ¼���û������
//			sprintf(name, "%sO%04d.nc", PATH_NC_FILE, m_n_macro_prog_name);   //ƴ���ļ�����
//		}else if(m_n_macro_prog_type == 3){//ϵͳ�����
//			sprintf(name, "%ssys_sub/O%04d.nc", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 4){
//			sprintf(name, "%sO%04d.iso", PATH_NC_FILE, m_n_macro_prog_name);   //ƴ���ļ�����
//		}else if(m_n_macro_prog_type == 5){
//			sprintf(name, "%ssys_sub/O%04d.iso", PATH_NC_FILE, m_n_macro_prog_name);
//		}else 	if(m_n_macro_prog_type == 6){//ͬĿ¼���û������
//			sprintf(name, "%sO%04d.NC", PATH_NC_FILE, m_n_macro_prog_name);   //ƴ���ļ�����
//		}else if(m_n_macro_prog_type == 7){//ϵͳ�����
//			sprintf(name, "%ssys_sub/O%04d.NC", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 8){
//			sprintf(name, "%sO%04d.ISO", PATH_NC_FILE, m_n_macro_prog_name);   //ƴ���ļ�����
//		}else if(m_n_macro_prog_type == 9){
//			sprintf(name, "%ssys_sub/O%04d.ISO", PATH_NC_FILE, m_n_macro_prog_name);
//		}
//	}else{
//		if(m_n_macro_prog_type == 2){//ͬĿ¼���û������
//			sprintf(name, "O%04d.nc", m_n_macro_prog_name);   //ƴ���ļ�����
//		}else if(m_n_macro_prog_type == 3){//ϵͳ�����
//			sprintf(name, "sys_sub/O%04d.nc", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 4){
//			sprintf(name, "O%04d.iso", m_n_macro_prog_name);   //ƴ���ļ�����
//		}else if(m_n_macro_prog_type == 5){
//			sprintf(name, "sys_sub/O%04d.iso", m_n_macro_prog_name);
//		}else 	if(m_n_macro_prog_type == 6){//ͬĿ¼���û������
//			sprintf(name, "O%04d.NC", m_n_macro_prog_name);   //ƴ���ļ�����
//		}else if(m_n_macro_prog_type == 7){//ϵͳ�����
//			sprintf(name, "sys_sub/O%04d.NC", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 8){
//			sprintf(name, "O%04d.ISO", m_n_macro_prog_name);   //ƴ���ļ�����
//		}else if(m_n_macro_prog_type == 9){
//			sprintf(name, "sys_sub/O%04d.ISO", m_n_macro_prog_name);
//		}
//	}

//}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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


/*****************************************************************AuxMsg��***************************************************************/
//Ĭ�Ϲ��캯��
AuxMsg::AuxMsg(int mcode){
	memset(m_n_m_code, 0, sizeof(int)*kMaxMCodeInLine);
	m_n_m_code[0] = mcode;
	m_n_m_count = 1;
	memset(m_n_aux_exec_segment, 0, kMaxMCodeInLine);
	SetMsgType(AUX_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ

    //lidianqiang:MDA�Զ�����M30��ʱ��ΪM300
    if(mcode == 2 || mcode == 30 || mcode == 300)
		this->SetFlag(FLAG_EOF, true);		//M02/M30�������ָ��
}

AuxMsg::AuxMsg(int *mcode, uint8_t total){
	if(mcode != nullptr)
		memcpy(this->m_n_m_code, mcode, sizeof(int)*total);

	m_n_m_count = total;
	memset(m_n_aux_exec_segment, 0, kMaxMCodeInLine);

	SetMsgType(AUX_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ

	for(uint8_t i = 0; i < total; i++){
        if(mcode[i] == 2 || mcode[i] == 30 || mcode[i] == 300)
			this->SetFlag(FLAG_EOF, true);		//M02/M30�������ָ��
	}
}

void AuxMsg::Execute(){

}

void AuxMsg::GetData(void *rec){

}

void AuxMsg::SetData(void *rec){

}

/**
 * @brief �Ƿ��״�ִ�д�������������Mָ���step����0��Ϊ�״�ִ��
 * @return true--��     false--����
 */
bool AuxMsg::IsFirstExec(){
	for(uint8_t i = 0; i < this->m_n_m_count; i++){
		if(this->m_n_aux_exec_segment[i] > 0)
			return false;
	}

	return true;
}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int AuxMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void AuxMsg::PrintString(){
	printf("[%lld]AuxMsg[%d]\n", m_n_line_no, m_n_m_count);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************SubProgCallMsg��***************************************************************/
SubProgCallMsg::SubProgCallMsg(int pcode, int lcode, uint8_t scan):AuxMsg(98){
	this->m_n_sub_prog_name = pcode;
	this->m_n_call_times = lcode;
	memset(this->m_str_last_prog_file, 0x00, kMaxPathLen);
	SetMsgType(SUBPROG_CALL_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ

    m_n_scan_mode = scan;//�ӳ�����ҹ���
}

void SubProgCallMsg::Execute(){

}

void SubProgCallMsg::GetData(void *rec){

}

void SubProgCallMsg::SetData(void *rec){

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int SubProgCallMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

/**
 * @brief ��ӡ��Ϣ
 */
void SubProgCallMsg::PrintString(){
	printf("[%lld]SubProgCallMsg[%d, %d, %d]\n", m_n_line_no, m_n_m_code[0], m_n_sub_prog_name, m_n_call_times);
}

/**
 * @brief ������һ���ļ��ľ���·�����������ַ�������
 * @param file[in] : �ļ�����·��
 */
void SubProgCallMsg::SetLastProgFile(char *file){
	if(file != nullptr){
		memset(m_str_last_prog_file, 0x00, kMaxPathLen);
		strcpy(m_str_last_prog_file, file);
	}
}

/**
 * @brief ��ȡ��һ���ļ��ľ���·�����������ַ�������
 * @param file
 */
void SubProgCallMsg::GetLastProgFile(char *file){
	if(file != nullptr){
		strcpy(file, m_str_last_prog_file);
    }
}

/**
 * @brief �����ӳ�����ҹ���
 * @return
 */
uint8_t SubProgCallMsg::GetScanMode() const
{
    return m_n_scan_mode;
}

/**
 * @brief �����ӳ�����
 * @param name[out] : ����ӳ�����
 * @param abs_path[in] : true--����·��   false--���·��
 */
//void SubProgCallMsg::GetSubProgName(char *name, bool abs_path){
//	if(abs_path){
//		if(m_n_sub_prog_type == 2){//ͬĿ¼���û��ӳ���
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.nc", PATH_NC_FILE, m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//		}
//		else if(m_n_sub_prog_type == 3){//ϵͳ�ӳ���
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.nc", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 4){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.iso", PATH_NC_FILE, m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 5){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.iso", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 6){//ͬĿ¼���û������
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.NC", PATH_NC_FILE, m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 7){//ϵͳ�����
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.NC", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 8){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%sO%04d.ISO", PATH_NC_FILE, m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.ISO", PATH_NC_FILE, m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 9){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.ISO", PATH_NC_FILE, m_n_sub_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.ISO", PATH_NC_FILE, m_n_sub_prog_name);
//		}
//	}else{
//		if(m_n_sub_prog_type == 2){//ͬĿ¼���û��ӳ���
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.nc", m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "O%d.nc", m_n_sub_prog_name);
//		}
//		else if(m_n_sub_prog_type == 3){//ϵͳ�ӳ���
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.nc", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.nc", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 4){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.iso", m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "O%d.iso", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 5){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.iso", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.iso", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 6){//ͬĿ¼���û������
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.NC", m_n_sub_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "O%d.NC", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 7){//ϵͳ�����
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.NC", m_n_sub_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.NC", m_n_sub_prog_name);
//		}else if(m_n_sub_prog_type == 8){
//			if(m_n_sub_prog_name <= 9999)
//				sprintf(name, "O%04d.ISO", m_n_sub_prog_name);   //ƴ���ļ�����
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
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************ProgCallMsg��***************************************************************/
/**
 * @brief ���캯��
 * @param pcode : ������
 * @param lcode �����ô���
 * @param param ������ָ��
 * @param count ����������
 * @param mask ������mask
 */
MacroProgCallMsg::MacroProgCallMsg(int pcode, int lcode, double *param, uint8_t count, uint32_t mask, uint8_t scan):ModeMsg(G65_CMD){

	this->m_n_macro_prog_name = pcode;
	this->m_n_call_times = lcode;
	this->m_mask_param = mask;
	this->m_p_df_param = param;
	this->m_n_param_count = count;

    m_n_scan_mode = scan;//�ӳ������ģʽ

	SetMsgType(MACRO_PROG_CALL_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ

}

/**
 * @brief ��������
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
 * @brief ���ز�������
 * @param mask[out] : ���ز���mask
 * @param count[out] �� ���ز�������
 * @return ���ز���������ָ��
 */
double *MacroProgCallMsg::GetParameter(uint32_t &mask, uint8_t &count){
	mask = m_mask_param;
	count = m_n_param_count;
	return m_p_df_param;
}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int MacroProgCallMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void MacroProgCallMsg::PrintString(){
	printf("[%lld]MacroProgCallMsg[%d, %d]\n", m_n_line_no, m_n_macro_prog_name, m_n_call_times);
}

/**
 * @brief ������һ���ļ��ľ���·�����������ַ�������
 * @param file[in] : �ļ�����·��
 */
void MacroProgCallMsg::SetLastProgFile(char *file){
	if(file != nullptr){
		memset(m_str_last_prog_file, 0x00, kMaxPathLen);
		strcpy(m_str_last_prog_file, file);
	}
}

/**
 * @brief ��ȡ��һ���ļ��ľ���·�����������ַ�������
 * @param file
 */
void MacroProgCallMsg::GetLastProgFile(char *file){
	if(file != nullptr){
		strcpy(file, m_str_last_prog_file);
    }
}

/**
 * @brief ���غ������ҹ���
 * @return
 */
uint8_t MacroProgCallMsg::GetScanMode() const
{
    return m_n_scan_mode;
}


/**
 * @brief �����ӳ�����
 * @param name[out] : ����ӳ��������ַ���
 * @param abs_path[in] : true--����·��   false--���·��
 */
//void MacroProgCallMsg::GetMacroProgName(char *name, bool abs_path){
//	if(abs_path){
//		if(m_n_macro_prog_type == 2){//ͬĿ¼���û������
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%sO%04d.nc", PATH_NC_FILE, m_n_macro_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.nc", PATH_NC_FILE, m_n_macro_prog_name);
//		}
//		else if(m_n_macro_prog_type == 3){//ϵͳ�����
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.nc", PATH_NC_FILE, m_n_macro_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.nc", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 4){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%sO%04d.iso", PATH_NC_FILE, m_n_macro_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.iso", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 5){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.iso", PATH_NC_FILE, m_n_macro_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.iso", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 6){//ͬĿ¼���û������
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%sO%04d.NC", PATH_NC_FILE, m_n_macro_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.NC", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 7){//ϵͳ�����
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.NC", PATH_NC_FILE, m_n_macro_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.NC", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 8){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%sO%04d.ISO", PATH_NC_FILE, m_n_macro_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "%sO%d.ISO", PATH_NC_FILE, m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 9){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "%ssys_sub/O%04d.ISO", PATH_NC_FILE, m_n_macro_prog_name);
//			else
//				sprintf(name, "%ssys_sub/O%d.ISO", PATH_NC_FILE, m_n_macro_prog_name);
//		}
//	}else{
//		if(m_n_macro_prog_type == 2){//ͬĿ¼���û������
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "O%04d.nc", m_n_macro_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "O%d.nc", m_n_macro_prog_name);
//		}
//		else if(m_n_macro_prog_type == 3){//ϵͳ�����
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.nc", m_n_macro_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.nc", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 4){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "O%04d.iso", m_n_macro_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "O%d.iso", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 5){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.iso", m_n_macro_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.iso", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 6){//ͬĿ¼���û������
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "O%04d.NC", m_n_macro_prog_name);   //ƴ���ļ�����
//			else
//				sprintf(name, "O%d.NC", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 7){//ϵͳ�����
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "sys_sub/O%04d.NC", m_n_macro_prog_name);
//			else
//				sprintf(name, "sys_sub/O%d.NC", m_n_macro_prog_name);
//		}else if(m_n_macro_prog_type == 8){
//			if(m_n_macro_prog_name <= 9999)
//				sprintf(name, "O%04d.ISO", m_n_macro_prog_name);   //ƴ���ļ�����
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
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************SubProgReturnMsg��******************************************************************/
SubProgReturnMsg::SubProgReturnMsg(char *file, bool ret_macro):AuxMsg(99){
	strcpy(this->m_str_file_name, file);
	this->m_b_ret_from_macroprog = ret_macro;
	SetMsgType(SUBPROG_RETURN_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
}

void SubProgReturnMsg::Execute(){

}

void SubProgReturnMsg::GetData(void *rec){

}

void SubProgReturnMsg::SetData(void *rec){

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int SubProgReturnMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void SubProgReturnMsg::PrintString(){
    printf("SubProgReturnMsg[file:%s]\n", this->m_str_file_name);
}


/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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



/*****************************************************************CompensateMsg��***************************************************************/
/**
 * @brief CompensateMsg���캯��
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

//	if(gcode == G43_CMD || gcode == G44_CMD || gcode == G49_CMD || gcode == G43_4_CMD){	//����ǵ��߳��Ȳ����������ƶ�
////		SetFlag(FLAG_BLOCK_OVER, true);  //���÷ֿ������־   //ע��ԭ�򣺴˴������ÿ������־���ɺ����Ƿ�����ƶ�ָ���������Ƿ�����
//
//		SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
//	}

	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
}

void CompensateMsg::Execute(){

}

void CompensateMsg::GetData(void *rec){

}

void CompensateMsg::SetData(void *rec){

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @param flag : true--����    false--����
 * @return 0--�ɹ�����   1--���践��
 */
int CompensateMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;

	if(this->m_n_flags.bits.axis_move == 0)
		return 1;

	//�ƶ�ָ������
	data->data.cmd = m_n_move_type;


	//�����ٶ�
	if(m_n_move_type == MOVE_G01)
		data->data.feed = FEED_TRANS(m_df_feed);   //��λ��mm/minת��Ϊum/s
    else{
		data->data.feed = 0;
    }
//	printf("linemsg feed : %d\n", data->data.feed);


	data->data.line_no = this->m_n_line_no; 	//�к�

	//Ŀ��λ��
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

	data->data.pos0 = MM2NM0_1(pos.x); //��λ��mmת��Ϊ0.1nm
	data->data.pos1 = MM2NM0_1(pos.y);
	data->data.pos2 = MM2NM0_1(pos.z);
	data->data.pos3 = MM2NM0_1(pos.a4);
	data->data.pos4 = MM2NM0_1(pos.a5);
	data->data.pos5 = MM2NM0_1(pos.a6);
	data->data.pos6 = MM2NM0_1(pos.a7);
	data->data.pos7 = MM2NM0_1(pos.a8);

	//Ext_type
	if(this->m_n_flags.bits.step_flag)  //���α�־
		data->data.ext_type = 0x20;
	else
		data->data.ext_type = 0;

	if(this->m_n_flags.bits.jump_flag)
		data->data.ext_type |=0x100;   //bit8�����α�־


	return res;
}

/**
 * @brief ���ɷ������ݰ�
 * @param data[out] : ���ط�������
 * @return 0--�ɹ�����   1--���践��
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
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************RapidMsg��***************************************************************/
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

	this->SetFlag(FLAG_AXIS_MOVE, true);  //�����ƶ�ָ��

	if(target == source)
		this->SetFlag(FLAG_POINT_MOVE, true);   //�㶯�ƶ�
}

/**
 * @brief ��������
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
 * @brief ����PMC����˶�����
 * @param pmc_count �� PMC����
 * @param inc : �Ƿ�����ģʽλ��
 */
void RapidMsg::SetPmcAxisCount(uint8_t pmc_mask){

    this->m_n_pmc_count = pmc_mask;


//    if(m_n_pmc_count > 0){
//		SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
//		SetFlag(FLAG_BLOCK_OVER, true);		  //�����
//	}
}

/**
 * @brief ��ȡPMC����˶�Ŀ������
 * @param count[out] �� PMC����
 * @param mask[out] �� ���mask
 * @param inc[out] : ����ģʽλ��
 * @return ��Ŀ��λ�������ͷָ��
 */
//double *RapidMsg::GetPmcTarget(uint8_t &count, uint32_t &mask, bool &inc){
//	count =  this->m_n_pmc_count;
//	mask = this->m_pmc_move_mask;
//	inc = this->m_b_inc_pos;
//	return this->m_p_pmc_target;
//}

/**
 * @brief ���µ�ƫ��ͬ��Ŀ��λ��
 * @param pos : ���µ�ƫ��ĵ�ǰ��������ϵλ��
 */
void RapidMsg::RefreshTargetPos(DPointChn &pos){
	if(this->m_axis_move_mask == 0)
		return;

	this->m_point_source = pos;  //ͬ����ʼλ��

	for(int i = 0; i < 8; i++){
		if(m_axis_move_mask & (0x01<<i)){
			continue;
		}
		m_point_target.m_df_point[i] = pos.m_df_point[i];	//ͬ��Ŀ��λ��
	}

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int RapidMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;
	//�ƶ�ָ������
	data->data.cmd = MOVE_G00;

	data->data.feed = 0;  //G00����Ҫ���ͽ����ٶ�


	data->data.line_no = this->m_n_line_no; 	//�к�

	//Ŀ��λ��
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

	data->data.pos0 = MM2NM0_1(pos.x); //��λ��mmת��Ϊ0.1nm
	data->data.pos1 = MM2NM0_1(pos.y);
	data->data.pos2 = MM2NM0_1(pos.z);
	data->data.pos3 = MM2NM0_1(pos.a4);
	data->data.pos4 = MM2NM0_1(pos.a5);
	data->data.pos5 = MM2NM0_1(pos.a6);
	data->data.pos6 = MM2NM0_1(pos.a7);
	data->data.pos7 = MM2NM0_1(pos.a8);

	data->data.reserved = this->m_io_data;   //IO����

	//Ext_type
	if(this->m_n_flags.bits.step_flag)  //���α�־
		data->data.ext_type = 0x20;
	else
		data->data.ext_type = 0;

	if(this->m_n_flags.bits.jump_flag)
		data->data.ext_type |=0x100;   //bit8�����α�־


//	printf("output rapid msg: (%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf)\n", pos.x, pos.y, pos.z, pos.a4, pos.a5, pos.a6, pos.a7, pos.a8);

	return res;
}

/**
 * @brief ���ɷ������ݰ�
 * @param data[out] : ���ط�������
 * @return 0--�ɹ�����   1--���践��
 */
int RapidMsg::GetSimulateData(CompilerSimData &data){

	data.type = 0;

	data.target[0] = this->m_point_target.m_df_point[0];
	data.target[1] = this->m_point_target.m_df_point[1];
	data.target[2] = this->m_point_target.m_df_point[2];

	return 0;
}

/**
 * @brief ��ӡ���
 */
void RapidMsg::PrintString(){
	printf("[%lld]RapidMsg, dest[%lf, %lf, %lf]\n", m_n_line_no, m_point_target.m_df_point[0], m_point_target.m_df_point[1], m_point_target.m_df_point[2]);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************LineMsg��***************************************************************/
/**
 * @brief ���캯��
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


	this->SetFlag(FLAG_AXIS_MOVE, true);  //�����ƶ�ָ��

	if(target == source)
		this->SetFlag(FLAG_POINT_MOVE, true);   //�㶯�ƶ�

}

/**
 * @brief ��������
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
 * @brief ����PMC����˶�����
 * @param pmc_count �� PMC����
 */
void LineMsg::SetPmcAxisCount(uint8_t pmc_mask){
    this->m_n_pmc_count = pmc_mask;

//    if(m_n_pmc_count > 0){
//		SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
//		SetFlag(FLAG_BLOCK_OVER, true);		//�����
//	}
}

/**
 * @brief ��ȡPMC����˶�Ŀ������
 * @param count[out] �� PMC����
 * @param mask[out] �� ���mask
 * @param inc[out] : ����ģʽλ��
 * @return ��Ŀ��λ�������ͷָ��
 */
//double *LineMsg::GetPmcTarget(uint8_t &count, uint32_t &mask, bool &inc){
//	count =  this->m_n_pmc_count;
//	mask = this->m_pmc_move_mask;
//	inc = this->m_b_inc_pos;
//	return this->m_p_pmc_target;
//}


/**
 * @brief ���µ�ƫ��ͬ��Ŀ��λ��
 * @param pos : ���µ�ƫ��ĵ�ǰ��������ϵλ��
 */
void LineMsg::RefreshTargetPos(DPointChn &pos){
	if(this->m_axis_move_mask == 0)
		return;

	printf("LineMsg::RefreshTargetPos, line=%llu, tar(%lf, %lf, %lf), newpos(%lf, %lf, %lf)\n", this->GetLineNo(), m_point_target.m_df_point[0],
			m_point_target.m_df_point[1], m_point_target.m_df_point[2], pos.m_df_point[0], pos.m_df_point[1], pos.m_df_point[2]);
	this->m_point_source = pos;  //ͬ����ʼλ��

	for(int i = 0; i < 8; i++){
		if(m_axis_move_mask & (0x01<<i)){
			continue;
		}
		m_point_target.m_df_point[i] = pos.m_df_point[i];	//ͬ��Ŀ��λ��
	}

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @param flag : true--����    false--����
 * @return 0--�ɹ�����   1--���践��
 */
int LineMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;


	//�ƶ�ָ������
	data->data.cmd = MOVE_G01;

	//�����ٶ�
	data->data.feed = FEED_TRANS(m_df_feed);   //��λ��mm/minת��Ϊum/s


	data->data.line_no = this->m_n_line_no; 	//�к�

//	printf("line data: x[%lf], y[%lf]\n", m_point_target.x, m_point_target.y);

	//Ŀ��λ��
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

	data->data.pos0 = MM2NM0_1(pos.x); //��λ��mmת��Ϊ0.1nm
	data->data.pos1 = MM2NM0_1(pos.y);
	data->data.pos2 = MM2NM0_1(pos.z);
	data->data.pos3 = MM2NM0_1(pos.a4);
	data->data.pos4 = MM2NM0_1(pos.a5);
	data->data.pos5 = MM2NM0_1(pos.a6);
	data->data.pos6 = MM2NM0_1(pos.a7);
	data->data.pos7 = MM2NM0_1(pos.a8);

    ///printf("-----------------------> LineMsg pos %lf -- %lf -- %lf mask: %d\n", pos.x, pos.y, pos.z, mask);
	//printf("posnm %lld -- %lld -- %lld \n", data->data.pos0, data->data.pos1, data->data.pos2);

	data->data.reserved = this->m_io_data;   //IO����

	//Ext_type
	if(this->m_n_flags.bits.step_flag)  //���α�־
		data->data.ext_type = 0x20;
	else
		data->data.ext_type = 0;

	if(this->m_n_flags.bits.jump_flag)
		data->data.ext_type |=0x100;   //bit8�����α�־


	return res;
}

/**
 * @brief ���ɷ������ݰ�
 * @param data[out] : ���ط�������
 * @return 0--�ɹ�����   1--���践��
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
 * @brief ��ֵ�����
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
 * @brief �ж������
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
/*****************************************************************ArcMsg��***************************************************************/
/**
 * @brief Բ����Ϣ���캯��
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
	this->m_gmode_2 = G17_CMD;   //Ĭ��G17ƽ��
	SetMsgType(ARC_MSG);

//	if(m_flag_circle || target != source)
	this->SetFlag(FLAG_AXIS_MOVE, true);  //�����ƶ�ָ��
}

void ArcMsg::Execute(){

}

void ArcMsg::GetData(void *rec){

}

void ArcMsg::SetData(void *rec){

}

/**
 * @brief ���µ�ƫ��ͬ��Ŀ��λ��
 * @param pos : ���µ�ƫ��ĵ�ǰ��������ϵλ��
 */
void ArcMsg::RefreshTargetPos(DPointChn &pos){
	if(this->m_axis_move_mask == 0)
		return;

	this->m_point_source = pos;  //ͬ����ʼλ��

	for(int i = 0; i < 8; i++){
		if(m_axis_move_mask & (0x01<<i)){
			continue;
		}
		m_point_target.m_df_point[i] = pos.m_df_point[i];	//ͬ��Ŀ��λ��
	}

	//���¼���Բ��
	this->CalArcCenter(m_point_source, m_point_target, m_df_radius, m_flag_direct*m_flag_major, m_gmode_2, this->m_point_center);

	printf("arcmsg::refreshtargetpos, src(%lf, %lf, %lf), tar(%lf, %lf, %lf), center(%lf, %lf, %lf)\n",
			m_point_source.m_df_point[0], m_point_source.m_df_point[1], m_point_source.m_df_point[2],
			m_point_target.m_df_point[0], m_point_target.m_df_point[1], m_point_target.m_df_point[2],
			m_point_center.m_df_point[0], m_point_center.m_df_point[1], m_point_center.m_df_point[2]);

}

/**
 * @brief ����Բ����������
 * @param src : Բ���������
 * @param tar ��Բ���յ�����
 * @param radius ��Բ���뾶
 * @param flag �� -1��ʾ˳ʱ�룬1��ʾ��ʱ��
 * @param gmode_2 : �ڶ���Gģ̬��G17��G18��G19
 * @param [out]center : ����������Բ����������
 * @return true--�ɹ�   false--ʧ��
 */
bool ArcMsg::CalArcCenter(const DPointChn &src, const DPointChn &tar, const double &radius, const int8_t flag, const uint16_t gmode_2, DPointChn &cen){
	DPlane source = Point2Plane(src, gmode_2);
	DPlane target = Point2Plane(tar, gmode_2);


	DPlane mid = source + target;
	mid /= 2;   //�����ĵ�
	DPlane dd = mid - source;
	double dxy = dd.x * dd.x + dd.y * dd.y;
	double dr2 = radius * radius - dxy;
	if(dr2 <= MZERO)
		dr2 = 0.0;
	double dr = sqrt(dxy); //һ����ҳ�
	if((dr-radius) > 2E-3){  //�ҳ�����ֱ�����޷���Ǣ���澯
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
//        SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
//        SetFlag(FLAG_BLOCK_OVER, true);		//�����
//    }
}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @param flag : ����ִ�б�־��true--����false--������Ҫ�����������������
 * @return 0--�ɹ�����   1--���践��
 */
int ArcMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;

	//�ƶ�ָ������
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

	//�����ٶ�
	data->data.feed = FEED_TRANS(m_df_feed);   //��λ��mm/minת��Ϊum/s

	data->data.line_no = this->m_n_line_no; 	//�к�

	//Ŀ��λ��
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

	data->data.pos0 = MM2NM0_1(pos.x); //��λ��mmת��Ϊ0.1nm
	data->data.pos1 = MM2NM0_1(pos.y);
	data->data.pos2 = MM2NM0_1(pos.z);
	data->data.pos3 = MM2NM0_1(pos.a4);
	data->data.pos4 = MM2NM0_1(pos.a5);
	data->data.pos5 = MM2NM0_1(pos.a6);
	data->data.pos6 = MM2NM0_1(pos.a7);
	data->data.pos7 = MM2NM0_1(pos.a8);

	//Բ������
	data->data.arc_center0 = MM2NM0_1(this->m_point_center.m_df_point[0]);
	data->data.arc_center1 = MM2NM0_1(this->m_point_center.m_df_point[1]);
	data->data.arc_center2 = MM2NM0_1(this->m_point_center.m_df_point[2]);

	//�뾶
	data->data.arc_radius = MM2NM0_1(this->m_df_radius);

	data->data.reserved = this->m_io_data;  //IO����

	//Ext_type
	data->data.ext_type = 0x00;
	//�������ӻ���־�� bit3: 0-��ʾ�ӻ�  1-��ʾ�Ż�
	if(this->m_flag_major == -1)		//�Ż�
		data->data.ext_type |= 0x08;
	//������Բ��־��bit4: 0-��ʾԲ��   1-��ʾ��Բ
	if(this->m_flag_circle)
		data->data.ext_type |= 0x10;	//��Բ

	if(this->m_n_flags.bits.step_flag)
		data->data.ext_type |= 0x20;	//bit5:���α�־

	if(this->m_n_flags.bits.jump_flag)
		data->data.ext_type |=0x100;   //bit8�����α�־

	return res;
}

/**
 * @brief ���ɷ������ݰ�
 * @param data[out] : ���ط�������
 * @return 0--�ɹ�����   1--���践��
 */
int ArcMsg::GetSimulateData(CompilerSimData &data){
	if(m_n_g_code == G02_CMD)
		data.type = 2;
	else if(m_n_g_code == G03_CMD)
		data.type = 3;

	//�յ�
	data.target[0] = this->m_point_target.m_df_point[0];
	data.target[1] = this->m_point_target.m_df_point[1];
	data.target[2] = this->m_point_target.m_df_point[2];

	//Բ��
	data.center[0] = this->m_point_center.m_df_point[0];
	data.center[1] = this->m_point_center.m_df_point[1];
	data.center[2] = this->m_point_center.m_df_point[2];

	//�뾶
	data.radius = this->m_df_radius;

	if(this->m_flag_major == -1){//�Ż��� radius���ظ�ֵ
		if(data.radius > 0)
			data.radius *= -1;
	}
	else if(this->m_flag_major == 1){  //�ӻ��� radius������ֵ
		if(data.radius < 0)
			data.radius *= -1;
	}

	if(this->m_point_target == this->m_point_source &&
			this->m_flag_circle == 0)   //����Բ
		data.radius = 0;

	data.plane = this->m_gmode_2;  //��ǰƽ��
	
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
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************SpindleCheckMsg��***************************************************************/
SpindleCheckMsg::SpindleCheckMsg(int gcode, int p, int q, int r, int i):ModeMsg(gcode){
	this->m_delay_time = p;
	this->m_allowed_tolerance = q;
	this->m_warn_ratio = r;
	this->m_warn_speed = i;
	SetMsgType(SPD_CHK_MSG);
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
}

void SpindleCheckMsg::Execute(){

}

void SpindleCheckMsg::GetData(void *rec){

}

void SpindleCheckMsg::SetData(void *rec){

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int SpindleCheckMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 1;

	return res;
}

void SpindleCheckMsg::PrintString(){
	printf("[%lld]SpindleCheckMsg\n", m_n_line_no);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************MacroCmdMsg��***************************************************************/
/**
 * @brief ���캯��
 * @param macro
 */
MacroCmdMsg::MacroCmdMsg(LexerMacroCmd *macro){
	SetMsgType(MACRO_MSG);
	this->m_n_run_step = 0;
	this->m_ln_offset = 0;
	//this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
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
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int MacroCmdMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 1;

	return res;
}

void MacroCmdMsg::PrintString(){
	printf("[%lld]MacroCmdMsg\n", m_n_line_no);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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

/*****************************************************************PolarIntpMsg��***************************************************************/
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
	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
}

void PolarIntpMsg::Execute(){

}

void PolarIntpMsg::GetData(void *rec){

}

void PolarIntpMsg::SetData(void *rec){

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int PolarIntpMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 1;

	return res;
}

void PolarIntpMsg::PrintString(){
	printf("[%lld]PolarIntpMsg\n", m_n_line_no);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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


/*****************************************************************ClearCirclePosMsg��***************************************************************/
ClearCirclePosMsg::ClearCirclePosMsg(int gcode, uint32_t axis_mask, int loop):ModeMsg(gcode){
	this->m_axis_mask = axis_mask;
	this->m_n_circle_loop = loop;

	m_n_exec_step = 0;
	this->m_n_g_code = 2000;  //G200

	SetMsgType(CLEAR_CIRCLE_POS_MSG);

	this->SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
}

void ClearCirclePosMsg::Execute(){

}

void ClearCirclePosMsg::GetData(void *rec){

}

void ClearCirclePosMsg::SetData(void *rec){

}

/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int ClearCirclePosMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;



	return res;
}

void ClearCirclePosMsg::PrintString(){
	printf("[%lld]ClearCirclePosMsg[%d]\n", m_n_line_no, m_n_g_code);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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
/*****************************************************************SpeedCtrlMsg��***************************************************************/
/**
 * @brief ���캯��
 * @param target : Ŀ��λ��
 * @param count �� PMC����
 * @param axis_mask �� ���mask
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

	SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
}

/**
 * @brief ��������
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
 * @brief ��ȡPMC����˶�Ŀ������
 * @param count[out] �� PMC����
 * @param mask[out] �� ���mask
 * @return ��Ŀ��λ�������ͷָ��
 */
double *SpeedCtrlMsg::GetTargetValue(uint8_t &count, uint32_t &mask){
	count =  this->m_n_axis_count;
	mask = this->m_axis_move_mask;

	return this->m_p_target_speed;
}


/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int SpeedCtrlMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;

	return res;
}

/**
 * @brief ͳһָ�λ
 * @param data
 * @return 0--�ɹ�����   1--���践��
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
 * @brief ��ֵ�����
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
 * @brief �ж������
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



/*****************************************************************SpeedCtrlMsg��***************************************************************/
/**
 * @brief ���캯��
 * @param target : Ŀ��λ��
 * @param count �� PMC����
 * @param axis_mask �� ���mask
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
	   SetFlag(FLAG_WAIT_MOVE_OVER, true);   //��Ҫ�ȴ��˶���λ
    }else{
	   SetFlag(FLAG_WAIT_MOVE_OVER, false);   //���õȴ��˶���λ
    }
}

/**
 * @brief ��������
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
 * @brief ��ȡPMC����˶�Ŀ������
 * @param count[out] �� PMC����
 * @param mask[out] �� ���mask
 * @return ��Ŀ��λ�������ͷָ��
 */
double *TorqueCtrlMsg::GetTargetValue(uint8_t &count, uint32_t &mask){
	count =  this->m_n_axis_count;
	mask = this->m_axis_move_mask;

	return this->m_p_target_torque;
}


/**
 * @brief ���ɴ������MC���˶��������ݰ�
 * @param data
 * @param mask : ͨ���岹������
 * @return 0--�ɹ�����   1--���践��
 */
int TorqueCtrlMsg::GetOutputData(GCodeFrame *data, uint32_t mask, bool flag){
	int res = 0;




	return res;
}

void TorqueCtrlMsg::PrintString(){
	printf("[%lld]TorqueCtrlMsg[%d], speed[%lf, %lf, %lf]\n", m_n_line_no, this->m_n_g_code/10, m_p_target_torque[0], m_p_target_torque[1], m_p_target_torque[2]);
}

/**
 * @brief ��ֵ�����
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
 * @brief �ж������
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
//���ɴ������MC���˶��������ݰ�
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
//���ɴ������MC���˶��������ݰ�
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


