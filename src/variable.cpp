/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file variable.cpp
 *@author gonghao
 *@date 2020/10/19
 *@brief ��ͷ�ļ����������������
 *@version
 */

#include <variable.h>
#include "channel_engine.h"
#include "channel_control.h"
#include "alarm_processor.h"
/**
 * @brief ���캯��
 */
Variable::Variable() {
	// TODO Auto-generated constructor stub

	memset(this->m_b_init_local, 0x00, sizeof(this->m_b_init_local));   //��ʼ����־��Ϊfalse������ֵ
	memset(this->m_b_init_common, 0x00, sizeof(this->m_b_init_common));
	memset(this->m_b_init_common_keep, 0x00, sizeof(this->m_b_init_common_keep));
	memset(this->m_b_init_user_macro, 0x00, sizeof(this->m_b_init_user_macro));

	memset(this->m_df_local, 0x00, sizeof(this->m_df_local));
	memset(this->m_df_common, 0x00, sizeof(this->m_df_common));
	memset(this->m_df_common_keep, 0x00, sizeof(this->m_df_common_keep));
	memset(this->m_df_user_macro, 0x00, sizeof(this->m_df_user_macro));

	this->m_stack_local.empty();   //��վֲ�������ջ

	m_fp_keep_var = nullptr;
	m_fp_macro_var = nullptr;
    this->m_b_save_keep = false;  //Ĭ���޸���

}

/**
 * @brief ��������
 */
Variable::~Variable() {
	// TODO Auto-generated destructor stub
	this->m_stack_local.empty();   //��վֲ�������ջ

	if(m_fp_keep_var != nullptr){
		fsync(fileno(m_fp_keep_var));
		fclose(m_fp_keep_var);
	}

	if(m_fp_macro_var != nullptr){
		fsync(fileno(m_fp_macro_var));
		fclose(m_fp_macro_var);
	}
}

/**
 * @brief ��������ͨ��
 * @param chn
 */
void Variable::SetChnIndex(uint8_t chn){
	this->m_n_channel_index = chn;

	//�򿪶�Ӧ�ļ�
	char path[kMaxPathLen];
	memset(path, 0x00, kMaxPathLen);
	sprintf(path, PATH_MACRO_VAR_KEEP, chn);
	if(access(path, F_OK) == -1) 	//�ļ�������
		this->m_fp_keep_var = fopen(path, "wb+");//���ļ�
	else
		this->m_fp_keep_var = fopen(path, "rb+");//���ļ�


	memset(path, 0x00, kMaxPathLen);
	sprintf(path, PATH_USER_MACRO_VAR, chn);

	if(access(path, F_OK) == -1)
		this->m_fp_macro_var = fopen(path, "wb+");
	else
		this->m_fp_macro_var = fopen(path, "rb+");


	if(m_fp_keep_var == nullptr){
		g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "�򿪷���ʧ�Ժ���������ļ�ʧ�ܣ�");
		return;//�ļ���ʧ��
	}

	if(m_fp_macro_var == nullptr){
			g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "���û������ʧ�ܣ�");
			return;//�ļ���ʧ��
	}

	this->InitKeepVar();  //��ʼ������ʧ�Ա���
	this->InitMacroVar();  //��ʼ������ʧ�Ա���
}

/**
 * @brief ��ʼ������ʧ�Ժ����
 */
void Variable::InitKeepVar(){

	uint64_t total_size = 0;	//�����ļ��ܳ���
	uint64_t offset = kMaxCommKeepVarCount*(sizeof(double)+1);   //ƫ��
	fseek(m_fp_keep_var, 0L, SEEK_END);
	total_size = ftell(m_fp_keep_var);   //��ȡ�ļ�����

	if(total_size < offset){
		g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "�ļ����Ȳ���[%lld, %lld]��", total_size, offset);

		//��ʼ��д������
		fseek(m_fp_keep_var, 0L, SEEK_SET);
		size_t count  = fwrite(m_b_init_common_keep, 1, kMaxCommKeepVarCount, m_fp_keep_var);

		if(count != kMaxCommKeepVarCount){//д�����
			g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "��ʼ��д������ʧ�ܣ�");
			return;
		}

		count = fwrite(m_df_common_keep, 8, kMaxCommKeepVarCount, m_fp_keep_var);
		if(count != kMaxCommKeepVarCount){//д�����
			g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "��ʼ��д������ʧ�ܣ�");
			return;
		}
		fflush(m_fp_keep_var);
		fsync(fileno(m_fp_keep_var));
		return;
	}

	fseek(m_fp_keep_var, 0L, SEEK_SET);

	size_t count  = fread(this->m_b_init_common_keep, sizeof(bool), kMaxCommKeepVarCount, m_fp_keep_var);

	if(count != kMaxCommKeepVarCount){//��ȡ����
		g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "��ȡ�����ʧ��1��");
		return;
	}

	count = fread(this->m_df_common_keep, sizeof(double), kMaxCommKeepVarCount, m_fp_keep_var);
	if(count != kMaxCommKeepVarCount){//д�����
		g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "��ȡ�����ʧ��2��");
		return;
	}
	printf("succeed to initialize macro variable\n");
}

/**
 * @brief ��ʼ���û������
 */
void Variable::InitMacroVar()
{
	uint64_t total_size = 0;	//�����ļ��ܳ���
	uint64_t offset = kMaxUserMacroVarCount*(sizeof(double)+1);   //�ܴ�С  ����+bool flag
	fseek(m_fp_macro_var, 0L, SEEK_END);
	total_size = ftell(m_fp_macro_var);   //��ȡ�ļ�����

	if(total_size < offset)
	{
		g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "�ļ����Ȳ���[%lld, %lld]��", total_size, offset);

		//��ʼ��д������
		fseek(m_fp_keep_var, 0L, SEEK_SET);
		size_t count  = fwrite(m_b_init_user_macro, 1, kMaxUserMacroVarCount, m_fp_macro_var);

		if(count != kMaxCommKeepVarCount){//д�����
			g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "��ʼ��д���û������ʧ�ܣ�");
			return;
		}

		count = fwrite(m_df_user_macro, 8, kMaxUserMacroVarCount, m_fp_macro_var);
		if(count != kMaxUserMacroVarCount){//д�����
			g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "��ʼ��д���û������ʧ�ܣ�");
			return;
		}
		fflush(m_fp_macro_var);
		fsync(fileno(m_fp_macro_var));
		return;
	}

	fseek(m_fp_macro_var, 0L, SEEK_SET);

	size_t count  = fread(this->m_b_init_user_macro, sizeof(bool), kMaxUserMacroVarCount, m_fp_macro_var);

	if(count != kMaxUserMacroVarCount){//��ȡ����
		g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "��ȡ�û������ʧ��1��");
		return;
	}

	count = fread(this->m_df_user_macro, sizeof(double), kMaxUserMacroVarCount, m_fp_macro_var);
	if(count != kMaxUserMacroVarCount){//д�����
		g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "��ȡ�û������ʧ��2��");
		return;
	}
	printf("succeed to initialize user macro variable\n");
}

/**
 * @brief ��ȡ����ֵ������double
 * @param index[in] : �������
 * @param value[out] �����ر���ֵ
 * @param init[out] : �����Ƿ��ֵ�����Ƿ��ʼ��
 * @return true--�ɹ�   false--ʧ��
 */
bool Variable::GetVarValue(int index, double &value, bool &init){

	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	ChannelControl *chn_ctrl = chn_engine->GetChnControl(this->m_n_channel_index);
	if(chn_ctrl == nullptr)
		return false;

	if(index == 0 || index == 3100){	//�̶����ؿ�ֵ
		init = false;
	}else if(index == 3101){  //PI��Բ����
		init = true;
		value = M_PI;
	}else if(index == 3102){  //E����Ȼ����
		init = true;
		value = M_E;
	}else if(index > 0 && index <= kMaxLocalVarCount){  //�ֲ�����
		init = this->m_b_init_local[index-1];
		value = this->m_df_local[index-1];
	}else if(index >= 100 && index < 200){	//�Ǳ����͹�������
		init = this->m_b_init_common[index-100];
		value = this->m_df_common[index-100];
	}else if(index >= 500 && index <1000){	//�����͹�������
		init = this->m_b_init_common_keep[index-500];
		value = this->m_df_common_keep[index-500];
	}else if(index >=50000 && index < 55000){    //��չ�����͹�������
		init = this->m_b_init_user_macro[index-50000];
		value = this->m_df_user_macro[index-50000];
		//printf("===== GetVarValue value: %lf init: %d index: %d\n", value, init, index);
	}else if(index >= 1000){	//ϵͳ����
		if((index >= 5061 && index <= 5080) ||
				(index >= 5421 && index <= 5440)){
			if(!chn_ctrl->IsSkipCaptured()){   //G31����ʧ��
				init = false;
				return true;
			}
		}
		if(this->GetSysVar(index, value)){
			init = true;
		}else{
			init = false;
			return false;
		}
	}else{
		g_ptr_trace->PrintLog(LOG_ALARM, "����������[%d]�Ƿ�����ȡ����ֵʧ�ܣ�", index);
		return false;
	}

	return true;
}

/**
 * @brief ��ȡ����ֵ������int
 * @param index[in] : �������
 * @param value[out] �����ر���ֵ
 * @param init[out] : �����Ƿ��ֵ�����Ƿ��ʼ��
 * @return true--�ɹ�   false--ʧ��
 */
bool Variable::GetVarValue(int index, int &value, bool &init){
	//
	bool res = true;
	double dv = 0.0;
	res = this->GetVarValue(index, dv, init);
	if(res){
		value = static_cast<int>(dv);
	}

	return res;
}

/**
 * @brief ���ñ���ֵ
 * @param index : �������
 * @param value ������ֵ
 * @return true--�ɹ�   false--ʧ��
 */
bool Variable::SetVarValue(int index, double value){
	//
//	printf("set macro var[%d] value = %lf\n", index, value);
	if(index == 0){	//�̶����ؿ�ֵ
		return false;	//#0��������
	}else if(index > 0 && index <= kMaxLocalVarCount){  //�ֲ�����
		this->m_b_init_local[index-1] = true;
		this->m_df_local[index-1] = value;
	}else if(index >= 100 && index < 200){	//�Ǳ����͹�������
		this->m_b_init_common[index-100] = true;
		this->m_df_common[index-100] = value;
	}else if(index >= 500 && index <1000){	//�����͹�������
		this->m_b_init_common_keep[index-500] = true;
		this->m_df_common_keep[index-500] = value;
		this->SaveKeepComm(index);   //���浽�ļ�
	}else if(index >= 50000 && index < 55000){    //��չ�����͹�������
		this->m_b_init_user_macro[index-50000] = true;
		this->m_df_user_macro[index-50000] = value;
		this->SaveMacroComm(index);   //���浽�ļ�
		printf("===== SetVarValue value: %lf index: %d\n", value, index);
	}else if(index >= 1000){	//ϵͳ����
		return this->SetSysVar(index, value);
	}else{
		g_ptr_trace->PrintLog(LOG_ALARM, "����������[%d]�Ƿ������ñ���ֵʧ�ܣ�", index);
		return false;
	}

	return true;
}

/**
 * @brief ���ñ���ֵ
 * @param index : �������
 * @param value ������ֵ
 * @return true--�ɹ�   false--ʧ��
 */
bool Variable::SetVarValue(int index, int value){
	return this->SetVarValue(index, static_cast<double>(value));
}

/**
 * @brief ������λ����Ϊ��ֵ
 * @param index  : �������
 * @return true--�ɹ�   false--ʧ��
 */
bool Variable::ResetVariable(int index){
//	printf("reset macro var[%d]\n", index);
	if(index == 0){	//�̶����ؿ�ֵ
		return true;
	}else if(index > 0 && index <= kMaxLocalVarCount){  //�ֲ�����
		this->m_b_init_local[index-1] = false;
		this->m_df_local[index-1] = 0.0;
	}else if(index >= 100 && index < 200){	//�Ǳ����͹�������
		this->m_b_init_common[index-100] = false;
		this->m_df_common[index-100] = 0.0;
	}else if(index >= 500 && index <1000){	//�����͹�������
		this->m_b_init_common_keep[index-500] = false;
		this->m_df_common_keep[index-500] = 0.0;
		this->SaveKeepComm(index);
	}else if(index >= 50000 && index < 55000){
		this->m_b_init_user_macro[index-50000] = false;
		this->m_df_user_macro[index-50000] = 0.0;
		this->SaveMacroComm(index);
	}
	else{
		g_ptr_trace->PrintLog(LOG_ALARM, "����������[%d]�Ƿ�����λ����ֵʧ�ܣ�", index);
		return false;
	}
	return true;
}

/**
 * @brief �������帴λ
 */
void Variable::Reset(){
	this->m_stack_local.empty();   //��վֲ�������ջ
	memset(this->m_b_init_local, 0x00, sizeof(this->m_b_init_local));   //��ʼ����־��Ϊfalse������ֵ
//	memset(this->m_b_init_common, 0x00, sizeof(this->m_b_init_common));
//	memset(this->m_b_init_common_keep, 0x00, sizeof(this->m_b_init_common_keep));

	memset(this->m_df_local, 0x00, sizeof(this->m_df_local));
//	memset(this->m_df_common, 0x00, sizeof(this->m_df_common));
//	memset(this->m_df_common_keep, 0x00, sizeof(this->m_df_common_keep));
}

/**
 * @brief �ֲ�������λ
 */
void Variable::ResetLocalVar(){
	this->m_stack_local.empty();   //��վֲ�������ջ
	memset(this->m_b_init_local, 0x00, sizeof(this->m_b_init_local));   //��ʼ����־��Ϊfalse������ֵ
	memset(this->m_df_local, 0x00, sizeof(this->m_df_local));
}

/**
 * @brief ������������
 * @param buf
 * @param max_size
 * @param start_index : ������ʼ��ַ
 * @param count ����������
 * @return ʵ�ʿ��������ֽ���
 */
int Variable::CopyVar(char *buf, uint32_t max_size, uint32_t start_index, uint32_t count){
	int res = 0;

	if(start_index <= 33){//�ֲ�����
		if(start_index+count-1>33)
			count = 33-start_index+1;

		if(max_size < (sizeof(double)+1)*count){
			g_ptr_trace->PrintLog(LOG_ALARM, "Variable::CopyVar()���岻�㣡");
			return res;
		}
		//������ʼ����־
		memcpy(buf, &m_b_init_local[start_index-1], count);
		//��������ֵ
		memcpy(buf+count, &m_df_local[start_index-1], count*sizeof(double));

	}else if(start_index >= 100 && start_index < 200){   //�Ǳ��ֹ�������
		if(start_index+count > 200)
			count = 200-start_index;
		if(max_size < (sizeof(double)+1)*count){
			g_ptr_trace->PrintLog(LOG_ALARM, "Variable::CopyVar()���岻�㣡");
			return res;
		}
		//������ʼ����־
		memcpy(buf, &m_b_init_common[start_index-100], count);
		//��������ֵ
		memcpy(buf+count, &m_df_common[start_index-100], count*sizeof(double));

	}else if(start_index >= 500 && start_index < 1000){   //�����͹�������
		if(start_index+count > 1000)
			count = 1000-start_index;
		if(max_size < (sizeof(double)+1)*count){
			g_ptr_trace->PrintLog(LOG_ALARM, "Variable::CopyVar()���岻�㣡");
			return res;
		}
		//������ʼ����־
		memcpy(buf, &m_b_init_common_keep[start_index-500], count);
		//��������ֵ
		memcpy(buf+count, &m_df_common_keep[start_index-500], count*sizeof(double));														   
	}else if(start_index >= 50000 && start_index < 55000){
		if(start_index+count > 55000)
			count = 55000-start_index;
		if(max_size < (sizeof(double)+1)*count){
			g_ptr_trace->PrintLog(LOG_ALARM, "Variable::CopyVar()���岻�㣡");
			return res;
		}
		if(count > 100){
			// Ϊ��ȡ1000 �������   �������Ƿ��ʼ��
			memcpy(buf, &m_df_user_macro[start_index-50000], count*sizeof(double));

		}else{
			//������ʼ����־
			memcpy(buf, &m_b_init_user_macro[start_index-50000], count);
			//��������ֵ
			memcpy(buf+count, &m_df_user_macro[start_index-50000], count*sizeof(double));
		}
	}else if(start_index >=1000){//ϵͳ����
		return res;
	}else{//�Ƿ���ַ
		g_ptr_trace->PrintLog(LOG_ALARM, "Variable::CopyVar()�Ƿ���ַ����ʼ��ַ[%u]", start_index);
		return res;
	}

	res = (sizeof(double)+1)*count;
	return res;
}

/**
 * @brief �ֲ�������ջ
 * @return true--�ɹ�   false--ʧ��
 */
bool Variable::PushLocalVar(){
	LocalVarScene scene;

	//���ݾֲ�����
	memcpy(scene.init, this->m_b_init_local, sizeof(scene.init));
	memcpy(scene.value, this->m_df_local, sizeof(scene.value));

	//��λ�ֲ�����
	memset(this->m_b_init_local, 0x00, sizeof(this->m_b_init_local));
	memset(this->m_df_local, 0x00, sizeof(this->m_df_local));

	if(!this->m_stack_local.push(scene)){
		g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "Failed to push local variable!");
		return false;
	}

	return true;
}

/**
 * @brief �ֲ�������ջ
 * @return true--�ɹ�   false--ʧ��
 */
bool Variable::PopLocalVar(){
	LocalVarScene scene;


	if(!this->m_stack_local.pop(scene)){
		g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "Failed to pop local variable!");
		return false;
	}

	memcpy(this->m_b_init_local, scene.init, sizeof(scene.init));
	memcpy(this->m_df_local, scene.value, sizeof(scene.value));

	return true;
}

/**
 * @brief ��ȡϵͳ������double��
 * @param index
 * @param value
 * @return
 */
bool Variable::GetSysVar(int index, double&value){

	value = 0.0;

	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	ChannelControl *chn_ctrl = chn_engine->GetChnControl(this->m_n_channel_index);
	if(chn_ctrl == nullptr)
		return false;


	if(index == 1900){//����״̬
		value = static_cast<double>(chn_ctrl->GetChnStatus().machining_state);
	}else if(index == 1901){//�澯״̬
		AlarmProcessor *alarm_proc = AlarmProcessor::GetInstance();
		value = alarm_proc->GetLatestErrorCode();
	}else if(index >= 1000 && index <=1031){//��ӦG54-G57
		uint8_t ii = (index-1000)/8;
		uint8_t bit = (index-1000)%8;
		if(chn_ctrl->CheckGRegState(54+ii, bit))
			value = 1;
		else
			value = 0;
	}else if(index == 1032){ //��ȡG54~G57 32bits������
		value = chn_ctrl->GetMacroVar1032();
	}else if(index >= 1100 && index <= 1115){ //��ȡF54~F55
		uint8_t ii = (index-1100)/8;
		uint8_t bit = (index-1100)%8;
		if(chn_ctrl->CheckFRegState(54+ii, bit))
			value = 1;
		else
			value = 0;
	}else if(index == 1132){//��ȡF54~F55 16bits������
		value = chn_ctrl->GetMacroVar1132();
	}else if(index == 1133){//��ȡF56~F59 32bits������
		value = chn_ctrl->GetMacroVar1133();
	}
#ifdef USES_WUXI_BLOOD_CHECK
	else if(index >= 2100 && index < 2116){
		value = chn_ctrl->GetAxisCurMachPos(index-2100);
	}else if(index >= 2200 && index < 2216){
		value = chn_ctrl->GetAxisCurWorkPos(index-2200);
	}
#endif
	else if(index == 3011){  //ϵͳ���ڣ�2021.09.15�洢Ϊ��20210915
		time_t cur_time;
		struct tm* time_info;
		cur_time = time(nullptr);
		if (cur_time != -1) {
			time_info = localtime(&cur_time);
			value = time_info->tm_year + 1900;
			value *= 100;
			value += time_info->tm_mon + 1;
			value *= 100;
			value += time_info->tm_mday;
		}
	}else if(index == 3012){  //ϵͳʱ�䣬09:20:38�洢Ϊ��092038
		time_t cur_time;
		struct tm* time_info;
		cur_time = time(nullptr);
		if (cur_time != -1) {
			time_info = localtime(&cur_time);
			value = time_info->tm_hour;
			value *= 100;
			value += time_info->tm_min;
			value *= 100;
			value += time_info->tm_sec;
		}
	}
	else{
		return chn_ctrl->GetSysVarValue(index, value);
	}


	return true;
}

/**
 * @brief ��ȡϵͳ������int��
 * @param index
 * @param value
 * @return
 */
bool Variable::GetSysVar(int index, int &value){
	//
	double dv = 0.0;

	if(this->GetSysVar(index, dv))
		value = static_cast<int>(dv);
	else
		return false;

	return true;
}

/**
 * @brief ����ϵͳ������double��
 * @param index
 * @param value
 * @return  true -- �ɹ�  false -- ʧ��
 */
bool Variable::SetSysVar(int index, double value){
	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	ChannelControl *chn_ctrl = chn_engine->GetChnControl(this->m_n_channel_index);

	if(chn_ctrl == nullptr)
		return false;

	if(index >= 1100 && index <= 1115){ //����F54~F55
		uint8_t ii = (index-1100)/8;
		uint8_t bit = (index-1100)%8;
		bool v = value==0?false:true;
		if(!chn_ctrl->SetFRegValue(54+ii, bit, v))
			return false;
	}else if(index == 1132){//����F54~F55 16bits������
		uint16_t v = value;
		chn_ctrl->SetMacroVar1132(v);
	}else if(index == 1133){//����F56~F59 32bits������
		uint32_t v = value;
		chn_ctrl->SetMacroVar1133(v);
	}
	else
		return chn_ctrl->SetSysVarValue(index, value);

	return true;
}

/**
 * @brief ����ϵͳ������int��
 * @param index
 * @param value
 * @return
 */
bool Variable::SetSysVar(int index, int value){

	double dv = static_cast<double>(value);
	return this->SetSysVar(index, dv);
}

/**
 * @brief ͬ���ļ��޸���flash
 */
void Variable::Sync(bool close){
	if(m_fp_keep_var != nullptr && this->m_b_save_keep){
		fflush(m_fp_keep_var);
		fsync(fileno(m_fp_keep_var));

		this->m_b_save_keep = false;
		if(close){
			fclose(m_fp_keep_var);
//			sync();
			m_fp_keep_var = nullptr;
		}
	}
}

/**
 * @brief ���汣���ͱ���#500-#999
 */
bool Variable::SaveKeepComm(int index){

	if(index < 500 || index > 999)
		return false;

	int idx = index - 500;

	uint64_t offset = idx*sizeof(bool);   //ƫ��
	fseek(this->m_fp_keep_var, offset, SEEK_SET);

	size_t count  = fwrite(&m_b_init_common_keep[idx], 1, 1, m_fp_keep_var);

	if(count != 1){//д�����
		g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "д������ʧ�ܣ�");
		return false;
	}

	offset = kMaxCommKeepVarCount*sizeof(bool)+idx*sizeof(double);   //ƫ��
	fseek(this->m_fp_keep_var, offset, SEEK_SET);
	count = fwrite(&m_df_common_keep[idx], 8, 1, m_fp_keep_var);
	if(count != 1){//д�����
		g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "д������ʧ�ܣ�");
		return false;
	}

	this->m_b_save_keep = true;

//	fflush(m_fp_keep_var);
//	fsync(fileno(m_fp_keep_var));
	return true;
}

bool Variable::SaveMacroComm(int index){

	if(index < 50000 || index >= 55000)
		return false;

	int idx = index - 50000;

	uint64_t offset = idx*sizeof(bool);   //ƫ��
	fseek(this->m_fp_macro_var, offset, SEEK_SET);

	size_t count  = fwrite(&m_b_init_user_macro[idx], 1, 1, m_fp_macro_var);

	if(count != 1){//д�����
		g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "д���û������ʧ�ܣ�");
		return false;
	}

	offset = kMaxUserMacroVarCount*sizeof(bool)+idx*sizeof(double);   //ƫ��
	fseek(this->m_fp_macro_var, offset, SEEK_SET);
	count = fwrite(&m_df_user_macro[idx], 8, 1, m_fp_macro_var);

	if(count != 1){//д�����
		g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "д���û������ʧ�ܣ�");
		return false;
	}
	fflush(m_fp_macro_var);
	fsync(fileno(m_fp_macro_var));
	this->m_b_save_keep = true;
	return true;
}

/**
 * @brief ��ֵ�����
 * @param one
 * @return
 */
LocalVarScene &LocalVarScene::operator =(LocalVarScene &one){
	if(this == &one)
		return *this;
	memcpy(this->init, one.init, sizeof(this->init));
	memcpy(this->value, one.value, sizeof(this->value));

	return *this;
}


void Variable::MemsetMacroVar(int start, int count, double value){

	if(start >= 50000 && start <55000 && start+count <55000){
		int idx = start - 50000;
		for(int i=idx; i<idx+count; i++){
			m_b_init_user_macro[i] = 1;
			m_df_user_macro[i] = value;
		}
		SyncUserMacroVar();
	}
	return;
}

void Variable::InsertMacroVar(int index, int end, double value){

	if(index >= 50000){
		int idx = index - 50000;
		int tail = end - 50000;
		for(int i = tail; i>idx; i--){
			m_df_user_macro[i] = m_df_user_macro[i-1];
		}
		m_df_user_macro[idx] = value;
	}

	SyncUserMacroVar();
	return ;
}

void Variable::PopMacroVar(int index, int end){

	if(index >= 50000){
		int idx = index - 50000;
		int tail = end - 50000;

		for(int i=idx; i<tail; i++){
			m_df_user_macro[i] = m_df_user_macro[i+1];
		}
		m_df_user_macro[tail] = 0;
	}

	SyncUserMacroVar();
	return;
}

void Variable::SetMacroArray(int index, int count, char * buf){

	int start = index - 50000;

	memcpy(&m_df_user_macro[start], buf, sizeof(double)*count);

	SyncUserMacroVar();
}



void Variable::SyncUserMacroVar(){
	fseek(this->m_fp_macro_var, 0, SEEK_SET);

	fwrite(m_b_init_user_macro, sizeof(m_b_init_user_macro), 1, m_fp_macro_var);
	fwrite(m_df_user_macro, sizeof(m_df_user_macro), 1, m_fp_macro_var);


	if(m_fp_macro_var != nullptr){
		fflush(m_fp_macro_var);
		fsync(fileno(m_fp_macro_var));
	}
	return ;
}
