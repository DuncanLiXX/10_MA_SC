/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file variable.cpp
 *@author gonghao
 *@date 2020/10/19
 *@brief 本头文件包含宏变量类声明
 *@version
 */

#include <variable.h>
#include "channel_engine.h"
#include "channel_control.h"
#include "alarm_processor.h"
/**
 * @brief 构造函数
 */
Variable::Variable() {
	// TODO Auto-generated constructor stub

	memset(this->m_b_init_local, 0x00, sizeof(this->m_b_init_local));   //初始化标志置为false，即空值
	memset(this->m_b_init_common, 0x00, sizeof(this->m_b_init_common));
	memset(this->m_b_init_common_keep, 0x00, sizeof(this->m_b_init_common_keep));
	memset(this->m_b_init_user_macro, 0x00, sizeof(this->m_b_init_user_macro));

	memset(this->m_df_local, 0x00, sizeof(this->m_df_local));
	memset(this->m_df_common, 0x00, sizeof(this->m_df_common));
	memset(this->m_df_common_keep, 0x00, sizeof(this->m_df_common_keep));
	memset(this->m_df_user_macro, 0x00, sizeof(this->m_df_user_macro));

	this->m_stack_local.empty();   //清空局部变量堆栈

	m_fp_keep_var = nullptr;
	m_fp_macro_var = nullptr;
    this->m_b_save_keep = false;  //默认无更改

}

/**
 * @brief 析构函数
 */
Variable::~Variable() {
	// TODO Auto-generated destructor stub
	this->m_stack_local.empty();   //清空局部变量堆栈

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
 * @brief 设置所属通道
 * @param chn
 */
void Variable::SetChnIndex(uint8_t chn){
	this->m_n_channel_index = chn;

	//打开对应文件
	char path[kMaxPathLen];
	memset(path, 0x00, kMaxPathLen);
	sprintf(path, PATH_MACRO_VAR_KEEP, chn);
	if(access(path, F_OK) == -1) 	//文件不存在
		this->m_fp_keep_var = fopen(path, "wb+");//打开文件
	else
		this->m_fp_keep_var = fopen(path, "rb+");//打开文件


	memset(path, 0x00, kMaxPathLen);
	sprintf(path, PATH_USER_MACRO_VAR, chn);

	if(access(path, F_OK) == -1)
		this->m_fp_macro_var = fopen(path, "wb+");
	else
		this->m_fp_macro_var = fopen(path, "rb+");


	if(m_fp_keep_var == nullptr){
		g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "打开非易失性宏变量保存文件失败！");
		return;//文件打开失败
	}

	if(m_fp_macro_var == nullptr){
			g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "打开用户宏变量失败！");
			return;//文件打开失败
	}

	this->InitKeepVar();  //初始化非易失性变量
	this->InitMacroVar();  //初始化非易失性变量
}

/**
 * @brief 初始化非易失性宏变量
 */
void Variable::InitKeepVar(){

	uint64_t total_size = 0;	//升级文件总长度
	uint64_t offset = kMaxCommKeepVarCount*(sizeof(double)+1);   //偏移
	fseek(m_fp_keep_var, 0L, SEEK_END);
	total_size = ftell(m_fp_keep_var);   //获取文件长度

	if(total_size < offset){
		g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "文件长度不够[%lld, %lld]！", total_size, offset);

		//初始化写入数据
		fseek(m_fp_keep_var, 0L, SEEK_SET);
		size_t count  = fwrite(m_b_init_common_keep, 1, kMaxCommKeepVarCount, m_fp_keep_var);

		if(count != kMaxCommKeepVarCount){//写入错误
			g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "初始化写入宏变量失败！");
			return;
		}

		count = fwrite(m_df_common_keep, 8, kMaxCommKeepVarCount, m_fp_keep_var);
		if(count != kMaxCommKeepVarCount){//写入错误
			g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "初始化写入宏变量失败！");
			return;
		}
		fflush(m_fp_keep_var);
		fsync(fileno(m_fp_keep_var));
		return;
	}

	fseek(m_fp_keep_var, 0L, SEEK_SET);

	size_t count  = fread(this->m_b_init_common_keep, sizeof(bool), kMaxCommKeepVarCount, m_fp_keep_var);

	if(count != kMaxCommKeepVarCount){//读取错误
		g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "读取宏变量失败1！");
		return;
	}

	count = fread(this->m_df_common_keep, sizeof(double), kMaxCommKeepVarCount, m_fp_keep_var);
	if(count != kMaxCommKeepVarCount){//写入错误
		g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "读取宏变量失败2！");
		return;
	}
	printf("succeed to initialize macro variable\n");
}

/**
 * @brief 初始化用户宏变量
 */
void Variable::InitMacroVar()
{
	uint64_t total_size = 0;	//升级文件总长度
	uint64_t offset = kMaxUserMacroVarCount*(sizeof(double)+1);   //总大小  数据+bool flag
	fseek(m_fp_macro_var, 0L, SEEK_END);
	total_size = ftell(m_fp_macro_var);   //获取文件长度

	if(total_size < offset)
	{
		g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "文件长度不够[%lld, %lld]！", total_size, offset);

		//初始化写入数据
		fseek(m_fp_keep_var, 0L, SEEK_SET);
		size_t count  = fwrite(m_b_init_user_macro, 1, kMaxUserMacroVarCount, m_fp_macro_var);

		if(count != kMaxCommKeepVarCount){//写入错误
			g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "初始化写入用户宏变量失败！");
			return;
		}

		count = fwrite(m_df_user_macro, 8, kMaxUserMacroVarCount, m_fp_macro_var);
		if(count != kMaxUserMacroVarCount){//写入错误
			g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "初始化写入用户宏变量失败！");
			return;
		}
		fflush(m_fp_macro_var);
		fsync(fileno(m_fp_macro_var));
		return;
	}

	fseek(m_fp_macro_var, 0L, SEEK_SET);

	size_t count  = fread(this->m_b_init_user_macro, sizeof(bool), kMaxUserMacroVarCount, m_fp_macro_var);

	if(count != kMaxUserMacroVarCount){//读取错误
		g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "读取用户宏变量失败1！");
		return;
	}

	count = fread(this->m_df_user_macro, sizeof(double), kMaxUserMacroVarCount, m_fp_macro_var);
	if(count != kMaxUserMacroVarCount){//写入错误
		g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "读取用户宏变量失败2！");
		return;
	}
	printf("succeed to initialize user macro variable\n");
}

/**
 * @brief 获取变量值，返回double
 * @param index[in] : 变量序号
 * @param value[out] ：返回变量值
 * @param init[out] : 返回是否空值，即是否初始化
 * @return true--成功   false--失败
 */
bool Variable::GetVarValue(int index, double &value, bool &init){

	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	ChannelControl *chn_ctrl = chn_engine->GetChnControl(this->m_n_channel_index);
	if(chn_ctrl == nullptr)
		return false;

	if(index == 0 || index == 3100){	//固定返回空值
		init = false;
	}else if(index == 3101){  //PI，圆周率
		init = true;
		value = M_PI;
	}else if(index == 3102){  //E，自然对数
		init = true;
		value = M_E;
	}else if(index > 0 && index <= kMaxLocalVarCount){  //局部变量
		init = this->m_b_init_local[index-1];
		value = this->m_df_local[index-1];
	}else if(index >= 100 && index < 200){	//非保持型公共变量
		init = this->m_b_init_common[index-100];
		value = this->m_df_common[index-100];
	}else if(index >= 500 && index <1000){	//保持型公共变量
		init = this->m_b_init_common_keep[index-500];
		value = this->m_df_common_keep[index-500];
	}else if(index >=50000 && index < 55000){    //扩展保持型公共变量
		init = this->m_b_init_user_macro[index-50000];
		value = this->m_df_user_macro[index-50000];
		//printf("===== GetVarValue value: %lf init: %d index: %d\n", value, init, index);
	}else if(index >= 1000){	//系统变量
		if((index >= 5061 && index <= 5080) ||
				(index >= 5421 && index <= 5440)){
			if(!chn_ctrl->IsSkipCaptured()){   //G31捕获失败
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
		g_ptr_trace->PrintLog(LOG_ALARM, "变量索引号[%d]非法，获取变量值失败！", index);
		return false;
	}

	return true;
}

/**
 * @brief 获取变量值，返回int
 * @param index[in] : 变量序号
 * @param value[out] ：返回变量值
 * @param init[out] : 返回是否空值，即是否初始化
 * @return true--成功   false--失败
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
 * @brief 设置变量值
 * @param index : 变量序号
 * @param value ：变量值
 * @return true--成功   false--失败
 */
bool Variable::SetVarValue(int index, double value){
	//
//	printf("set macro var[%d] value = %lf\n", index, value);
	if(index == 0){	//固定返回空值
		return false;	//#0不可设置
	}else if(index > 0 && index <= kMaxLocalVarCount){  //局部变量
		this->m_b_init_local[index-1] = true;
		this->m_df_local[index-1] = value;
	}else if(index >= 100 && index < 200){	//非保持型公共变量
		this->m_b_init_common[index-100] = true;
		this->m_df_common[index-100] = value;
	}else if(index >= 500 && index <1000){	//保持型公共变量
		this->m_b_init_common_keep[index-500] = true;
		this->m_df_common_keep[index-500] = value;
		this->SaveKeepComm(index);   //保存到文件
	}else if(index >= 50000 && index < 55000){    //扩展保持型公共变量
		this->m_b_init_user_macro[index-50000] = true;
		this->m_df_user_macro[index-50000] = value;
		this->SaveMacroComm(index);   //保存到文件
		printf("===== SetVarValue value: %lf index: %d\n", value, index);
	}else if(index >= 1000){	//系统变量
		return this->SetSysVar(index, value);
	}else{
		g_ptr_trace->PrintLog(LOG_ALARM, "变量索引号[%d]非法，设置变量值失败！", index);
		return false;
	}

	return true;
}

/**
 * @brief 设置变量值
 * @param index : 变量序号
 * @param value ：变量值
 * @return true--成功   false--失败
 */
bool Variable::SetVarValue(int index, int value){
	return this->SetVarValue(index, static_cast<double>(value));
}

/**
 * @brief 变量复位，置为空值
 * @param index  : 变量序号
 * @return true--成功   false--失败
 */
bool Variable::ResetVariable(int index){
//	printf("reset macro var[%d]\n", index);
	if(index == 0){	//固定返回空值
		return true;
	}else if(index > 0 && index <= kMaxLocalVarCount){  //局部变量
		this->m_b_init_local[index-1] = false;
		this->m_df_local[index-1] = 0.0;
	}else if(index >= 100 && index < 200){	//非保持型公共变量
		this->m_b_init_common[index-100] = false;
		this->m_df_common[index-100] = 0.0;
	}else if(index >= 500 && index <1000){	//保持型公共变量
		this->m_b_init_common_keep[index-500] = false;
		this->m_df_common_keep[index-500] = 0.0;
		this->SaveKeepComm(index);
	}else if(index >= 50000 && index < 55000){
		this->m_b_init_user_macro[index-50000] = false;
		this->m_df_user_macro[index-50000] = 0.0;
		this->SaveMacroComm(index);
	}
	else{
		g_ptr_trace->PrintLog(LOG_ALARM, "变量索引号[%d]非法，复位变量值失败！", index);
		return false;
	}
	return true;
}

/**
 * @brief 变量整体复位
 */
void Variable::Reset(){
	this->m_stack_local.empty();   //清空局部变量堆栈
	memset(this->m_b_init_local, 0x00, sizeof(this->m_b_init_local));   //初始化标志置为false，即空值
//	memset(this->m_b_init_common, 0x00, sizeof(this->m_b_init_common));
//	memset(this->m_b_init_common_keep, 0x00, sizeof(this->m_b_init_common_keep));

	memset(this->m_df_local, 0x00, sizeof(this->m_df_local));
//	memset(this->m_df_common, 0x00, sizeof(this->m_df_common));
//	memset(this->m_df_common_keep, 0x00, sizeof(this->m_df_common_keep));
}

/**
 * @brief 局部变量复位
 */
void Variable::ResetLocalVar(){
	this->m_stack_local.empty();   //清空局部变量堆栈
	memset(this->m_b_init_local, 0x00, sizeof(this->m_b_init_local));   //初始化标志置为false，即空值
	memset(this->m_df_local, 0x00, sizeof(this->m_df_local));
}

/**
 * @brief 拷贝变量数据
 * @param buf
 * @param max_size
 * @param start_index : 变量起始地址
 * @param count ：变量个数
 * @return 实际拷贝数据字节数
 */
int Variable::CopyVar(char *buf, uint32_t max_size, uint32_t start_index, uint32_t count){
	int res = 0;

	if(start_index <= 33){//局部变量
		if(start_index+count-1>33)
			count = 33-start_index+1;

		if(max_size < (sizeof(double)+1)*count){
			g_ptr_trace->PrintLog(LOG_ALARM, "Variable::CopyVar()缓冲不足！");
			return res;
		}
		//拷贝初始化标志
		memcpy(buf, &m_b_init_local[start_index-1], count);
		//拷贝变量值
		memcpy(buf+count, &m_df_local[start_index-1], count*sizeof(double));

	}else if(start_index >= 100 && start_index < 200){   //非保持公共变量
		if(start_index+count > 200)
			count = 200-start_index;
		if(max_size < (sizeof(double)+1)*count){
			g_ptr_trace->PrintLog(LOG_ALARM, "Variable::CopyVar()缓冲不足！");
			return res;
		}
		//拷贝初始化标志
		memcpy(buf, &m_b_init_common[start_index-100], count);
		//拷贝变量值
		memcpy(buf+count, &m_df_common[start_index-100], count*sizeof(double));

	}else if(start_index >= 500 && start_index < 1000){   //保持型公共变量
		if(start_index+count > 1000)
			count = 1000-start_index;
		if(max_size < (sizeof(double)+1)*count){
			g_ptr_trace->PrintLog(LOG_ALARM, "Variable::CopyVar()缓冲不足！");
			return res;
		}
		//拷贝初始化标志
		memcpy(buf, &m_b_init_common_keep[start_index-500], count);
		//拷贝变量值
		memcpy(buf+count, &m_df_common_keep[start_index-500], count*sizeof(double));														   
	}else if(start_index >= 50000 && start_index < 55000){
		if(start_index+count > 55000)
			count = 55000-start_index;
		if(max_size < (sizeof(double)+1)*count){
			g_ptr_trace->PrintLog(LOG_ALARM, "Variable::CopyVar()缓冲不足！");
			return res;
		}
		if(count > 100){
			// 为获取1000 个宏变量   不考虑是否初始化
			memcpy(buf, &m_df_user_macro[start_index-50000], count*sizeof(double));

		}else{
			//拷贝初始化标志
			memcpy(buf, &m_b_init_user_macro[start_index-50000], count);
			//拷贝变量值
			memcpy(buf+count, &m_df_user_macro[start_index-50000], count*sizeof(double));
		}
	}else if(start_index >=1000){//系统变量
		return res;
	}else{//非法地址
		g_ptr_trace->PrintLog(LOG_ALARM, "Variable::CopyVar()非法地址！起始地址[%u]", start_index);
		return res;
	}

	res = (sizeof(double)+1)*count;
	return res;
}

/**
 * @brief 局部变量入栈
 * @return true--成功   false--失败
 */
bool Variable::PushLocalVar(){
	LocalVarScene scene;

	//备份局部变量
	memcpy(scene.init, this->m_b_init_local, sizeof(scene.init));
	memcpy(scene.value, this->m_df_local, sizeof(scene.value));

	//复位局部变量
	memset(this->m_b_init_local, 0x00, sizeof(this->m_b_init_local));
	memset(this->m_df_local, 0x00, sizeof(this->m_df_local));

	if(!this->m_stack_local.push(scene)){
		g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "Failed to push local variable!");
		return false;
	}

	return true;
}

/**
 * @brief 局部变量出栈
 * @return true--成功   false--失败
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
 * @brief 获取系统变量，double型
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


	if(index == 1900){//待机状态
		value = static_cast<double>(chn_ctrl->GetChnStatus().machining_state);
	}else if(index == 1901){//告警状态
		AlarmProcessor *alarm_proc = AlarmProcessor::GetInstance();
		value = alarm_proc->GetLatestErrorCode();
	}else if(index >= 1000 && index <=1031){//对应G54-G57
		uint8_t ii = (index-1000)/8;
		uint8_t bit = (index-1000)%8;
		if(chn_ctrl->CheckGRegState(54+ii, bit))
			value = 1;
		else
			value = 0;
	}else if(index == 1032){ //获取G54~G57 32bits的数据
		value = chn_ctrl->GetMacroVar1032();
	}else if(index >= 1100 && index <= 1115){ //获取F54~F55
		uint8_t ii = (index-1100)/8;
		uint8_t bit = (index-1100)%8;
		if(chn_ctrl->CheckFRegState(54+ii, bit))
			value = 1;
		else
			value = 0;
	}else if(index == 1132){//获取F54~F55 16bits的数据
		value = chn_ctrl->GetMacroVar1132();
	}else if(index == 1133){//获取F56~F59 32bits的数据
		value = chn_ctrl->GetMacroVar1133();
	}
#ifdef USES_WUXI_BLOOD_CHECK
	else if(index >= 2100 && index < 2116){
		value = chn_ctrl->GetAxisCurMachPos(index-2100);
	}else if(index >= 2200 && index < 2216){
		value = chn_ctrl->GetAxisCurWorkPos(index-2200);
	}
#endif
	else if(index == 3011){  //系统日期，2021.09.15存储为：20210915
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
	}else if(index == 3012){  //系统时间，09:20:38存储为：092038
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
 * @brief 获取系统变量，int型
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
 * @brief 设置系统变量，double型
 * @param index
 * @param value
 * @return  true -- 成功  false -- 失败
 */
bool Variable::SetSysVar(int index, double value){
	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	ChannelControl *chn_ctrl = chn_engine->GetChnControl(this->m_n_channel_index);

	if(chn_ctrl == nullptr)
		return false;

	if(index >= 1100 && index <= 1115){ //设置F54~F55
		uint8_t ii = (index-1100)/8;
		uint8_t bit = (index-1100)%8;
		bool v = value==0?false:true;
		if(!chn_ctrl->SetFRegValue(54+ii, bit, v))
			return false;
	}else if(index == 1132){//设置F54~F55 16bits的数据
		uint16_t v = value;
		chn_ctrl->SetMacroVar1132(v);
	}else if(index == 1133){//设置F56~F59 32bits的数据
		uint32_t v = value;
		chn_ctrl->SetMacroVar1133(v);
	}
	else
		return chn_ctrl->SetSysVarValue(index, value);

	return true;
}

/**
 * @brief 设置系统变量，int型
 * @param index
 * @param value
 * @return
 */
bool Variable::SetSysVar(int index, int value){

	double dv = static_cast<double>(value);
	return this->SetSysVar(index, dv);
}

/**
 * @brief 同步文件修改至flash
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
 * @brief 保存保持型变量#500-#999
 */
bool Variable::SaveKeepComm(int index){

	if(index < 500 || index > 999)
		return false;

	int idx = index - 500;

	uint64_t offset = idx*sizeof(bool);   //偏移
	fseek(this->m_fp_keep_var, offset, SEEK_SET);

	size_t count  = fwrite(&m_b_init_common_keep[idx], 1, 1, m_fp_keep_var);

	if(count != 1){//写入错误
		g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "写入宏变量失败！");
		return false;
	}

	offset = kMaxCommKeepVarCount*sizeof(bool)+idx*sizeof(double);   //偏移
	fseek(this->m_fp_keep_var, offset, SEEK_SET);
	count = fwrite(&m_df_common_keep[idx], 8, 1, m_fp_keep_var);
	if(count != 1){//写入错误
		g_ptr_trace->PrintTrace(TRACE_ERROR, MACRO_VARIABLE, "写入宏变量失败！");
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

	uint64_t offset = idx*sizeof(bool);   //偏移
	fseek(this->m_fp_macro_var, offset, SEEK_SET);

	size_t count  = fwrite(&m_b_init_user_macro[idx], 1, 1, m_fp_macro_var);

	if(count != 1){//写入错误
		g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "写入用户宏变量失败！");
		return false;
	}

	offset = kMaxUserMacroVarCount*sizeof(bool)+idx*sizeof(double);   //偏移
	fseek(this->m_fp_macro_var, offset, SEEK_SET);
	count = fwrite(&m_df_user_macro[idx], 8, 1, m_fp_macro_var);

	if(count != 1){//写入错误
		g_ptr_trace->PrintTrace(TRACE_ERROR, USER_MACRO_VARIABLE, "写入用户宏变量失败！");
		return false;
	}
	fflush(m_fp_macro_var);
	fsync(fileno(m_fp_macro_var));
	this->m_b_save_keep = true;
	return true;
}

/**
 * @brief 赋值运算符
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
