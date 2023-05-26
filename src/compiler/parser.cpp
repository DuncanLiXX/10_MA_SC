/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file parser.cpp
 *@author gonghao
 *@date 2020/04/10
 *@brief 本头文件为G代码语法解析器类的实现
 *@version
 */

#include <parser.h>
#include "channel_engine.h"
#include "channel_control.h"
#include "variable.h"
#include "parm_manager.h"
#include "spindle_control.h"

const int kSeparatedMCodeArray[] = {0,1,2,30,98,99,198};   //不能与其它M代码同行的M代码
const int kSeparatedMCodeCount = sizeof(kSeparatedMCodeArray)/sizeof(int);   //不能同行的M代码个数

/**
 * @brief 语法解析器构造函数
 * @param lexer_result：存放词法分析结果的数据结构指针
 */
Parser::Parser(uint8_t chn_index, LexerResult *lexer_result, CompileRecList *parser_result, CompileStatusCollect *compiler_status) {
	// TODO Auto-generated constructor stub
	this->m_n_channel_index = chn_index;
	this->m_p_lexer_result = lexer_result;
	this->m_p_parser_result = parser_result;
	this->m_p_compiler_status = compiler_status;

	this->m_p_channel_control = ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index);
//#ifdef USES_WOOD_MACHINE
	this->m_b_multi_mcode = true;   //对于木工机，多M代码功能默认打开
//#else
//	this->m_b_multi_mcode = false;  //多M代码默认为关闭状态
//#endif
	this->m_b_call_macro_prog = false;
	m_mask_pmc_axis = 0;
	m_n_pmc_axis_count = 0;
	memset(m_axis_name, 0x00, kAxisNameBufSize);
	memset(m_axis_name_ex, 0, kMaxAxisChn);

	this->SetAxisNameEx(g_ptr_parm_manager->GetSystemConfig()->axis_name_ex);

	this->m_p_simulate_mode = this->m_p_channel_control->GetSimulateModePtr();


	this->m_p_variable = nullptr;

	this->m_p_compiler_status->mode.Initialize();

	this->Reset();

}

/**
 * @brief 析构函数
 */
Parser::~Parser() {
	// TODO Auto-generated destructor stub

}


/**
 * @brief 复位语法解析器的状态
 */
void Parser::Reset(){
	m_error_code = ERR_NONE;
	m_b_f_code = false;   //默认F值未指定
	this->m_b_call_macro_prog = false;
//	this->m_p_compiler_status->mode.move_mode = 1;   //默认是01组模态
	this->m_p_compiler_status->mode.Reset();
	this->m_mode_mask = GMODE_NONE;
	this->m_b_has_g53 = false;
}

/**
 * @brief 刷新通道轴名称缓冲
 * @param name ：传入的轴名称数组
 */
void Parser::RefreshAxisName(){
	memset(m_axis_name, 0x00, kAxisNameBufSize);
	m_n_pmc_axis_count = 0;
	uint8_t phy_axis = 0;
//	char names[10] = "XYZABCUVW";
	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
	SCAxisConfig *axis_config = g_ptr_parm_manager->GetAxisConfig();    //物理轴配置
    ChannelControl *channel_control = g_ptr_chn_engine->GetChnControl(m_n_channel_index);
//	printf("Parser axis name:");
	for(uint8_t i = 0; i < chn_config->chn_axis_count; i++){
		phy_axis = chn_config->chn_axis_phy[i];
		if(phy_axis){
	//		if(axis_config[phy_axis-1].axis_type != AXIS_SPINDLE){
				m_axis_name[i] = strAxisNames[chn_config->chn_axis_name[i]];
	//		}
			if(this->m_b_axis_name_ex){
				m_axis_name_ex[i] = chn_config->chn_axis_name_ex[i];

//				printf("%c%hhu, ", m_axis_name[i], m_axis_name_ex[i]);
			}

            if(g_ptr_chn_engine->GetPmcActive(phy_axis-1)) {
				m_mask_pmc_axis |= (0x01<<i);   //标记pmc轴
				m_n_pmc_axis_count++;
			}
		}
	}
//	printf("\n");
}

/**
 * @brief 设置轴名称扩展下标使能
 * @param flag : false--关闭    true--打开
 */
void Parser::SetAxisNameEx(bool flag){

	this->m_b_axis_name_ex = flag;
    this->RefreshAxisName();
}

uint32_t Parser::GetPmcAxisMask()
{
    return m_mask_pmc_axis;
}

/**
 * @brief 对词法分析结果进行语法解析
 * @return true--成功  false--失败
 */
bool Parser::Compile(){
	bool res = true;
	if(m_p_lexer_result == nullptr){
		CreateError(ERR_COMPILER_INTER, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	if(this->m_p_lexer_result->error_code != ERR_NONE){ //在此生成词法分析的错误消息
		this->CreateErrorMsg(m_p_lexer_result->error_code);
		return true;
	}
	else if(m_p_lexer_result->macro_flag){//编译宏指令
		res = CompileMacro();
	}
	else{//编译G/M/S/T指令
		res = CompileGCode();
	}

	if(m_error_code != ERR_NONE){//发生编译错误
		//TODO 生成ERRORMSG插入消息队列
		printf("parser: create error message!\n");
		this->CreateErrorMsg(m_error_code);
		return true;
	}

	return res;
}

/**
 * @brief 编译宏指令
 * @return true--成功  false--失败
 */
bool Parser::CompileMacro(){
	bool res = true;
	LexerMacroCmd *macro_cmd = &(m_p_lexer_result->nc_code.macro_cmd);

	res = this->CreateMacroMsg(macro_cmd);

	//给本行最后一条指令置FLAG_LAST_REC标志
	if(res){
		ListNode<RecordMsg *> *tail = this->m_p_parser_result->TailNode();
		if(tail != nullptr){
			RecordMsg *msg = tail->data;
			if(msg != nullptr){
				msg->SetFlag(FLAG_LAST_REC, true);   //同行最后一条指令
				if(msg->GetMsgType() != LOOP_MSG){
					msg->SetFlag(FLAG_STEP, true);    //单步暂停标志
				}
			}
		}
	}

	return res;
}


/**
 * @brief 编译G指令
 * @return true--成功  false--失败
 */
bool Parser::CompileGCode(){
	bool res = true;
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

//	if(g_code->mask_value & (0x01<<O_DATA))
//	{
//		printf("get O value: %lf", g_code->value[O_DATA]);
//		return true;
//	}

	if(!CheckGCode(g_code)){//检查代码合法性
		printf("failed checkGcode fun\n");
		return false;
	}


	if(!AnalyzeGCode(g_code)){//将G代码转换为对应的Message
		printf("Failed to analyzed G Code:%lld\n", this->m_p_lexer_result->line_no);
		return false;
	}


	if(!ProcessMCode(g_code)){//处理M代码，将其转换为对应的Message
		printf("Failed to process M code\n");
		return false;
	}

	//给本行最后一条指令置FLAG_LAST_REC标志
	ListNode<RecordMsg *> *tail = this->m_p_parser_result->TailNode();
	if(tail != nullptr){
		RecordMsg *msg = tail->data;
		if(msg != nullptr){
			msg->SetFlag(FLAG_LAST_REC, true);   //同行最后一条指令
			if(msg->GetMsgType() != LOOP_MSG){
				msg->SetFlag(FLAG_STEP, true);    //单步暂停标志
			}
		}
	}

	return res;
}

/**
 * @brief 检查G指令合法性，包括M代码
 * @param gcode : 词法分析结果指针
 * @return true--成功  false--失败,产生非语法性错误
 */
bool Parser::CheckGCode(LexerGCode *gcode){

//	printf("Enter CheckGCode: %d\n", gcode->gcode_count);
//	if(gcode == nullptr){
//		m_error_code = ERR_COMPILER_INTER;
//		return false;
//	}

	if((!this->m_b_multi_mcode && gcode->mcode_count > 1) ||   //禁止同行多M指令时，M代码不能超过1个
			(m_b_multi_mcode && gcode->mcode_count > kMaxMCodeInLine)){ //允许同行多M指令时，M代码不能超过kMaxMCodeInLine定义的最大值
		m_error_code = ERR_TOO_MANY_M_CODE;  //M指令过多
		return false;
	}

	if(gcode->tcode_count > kMaxTCodeInLine){
		m_error_code = ERR_TOO_MANY_T_CODE;   //T指令过多
		g_ptr_trace->PrintTrace(TRACE_ERROR, COMPILER_PARSER, "Parser::CheckGCode, too many t code! tcode_count = %hhu\n", gcode->tcode_count);
		return false;
	}
/*int exp_index = static_cast<int>(g_code->value[addr]);
			while(!this->GetExpressionResult(g_code->macro_expression[exp_index], res))
 * */
	//进行模式检查
	uint64_t mode_mask = GMODE_NONE;
	int code = 0, count = 0, exp_index = 0;
	MacroVarValue res;
	//int code_limit = kMaxGCodeCount;
	//int mode_code[kMaxGModeCount];
	memset(m_mode_code, 0x00, sizeof(int)*kMaxGModeCount);
	this->m_b_has_g53 = false;
	for(int i = 0; i < gcode->gcode_count; i++){
		if(gcode->g_value[i] < 0){//为负值，说明此G代码含有宏表达式
			exp_index = abs(gcode->g_value[i] + 1);
			while(!this->GetExpressionResult(gcode->macro_expression[exp_index], res)){
				if(this->m_error_code != ERR_NONE)
					return false;
				usleep(10000);  //休眠10ms，等待MC运行到位
			}
			if(res.init){
				gcode->g_value[i] = res.value*10;  //放大十倍
			}else{
				m_error_code = ERR_G_EXP_NULL;   //G指令表达式为空
				return false;
			}

		}
		code = gcode->g_value[i]/10;

		// @add zk  新增 对G54.1 P XX 支持
		if(gcode->g_value[i] == 541){
			double value = 0.0;
			if(!GetCodeData(P_DATA, value)){
				CreateError(ERR_NO_P_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, m_p_lexer_result->line_no);
				return false;
			}

			if(value < 1 or value > 99){
				CreateError(ERR_NO_P_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, m_p_lexer_result->line_no);
				return false;
			}
			code = 5400 + value;
			// 将 G54.1 P XX 转换为 G5401 ~ G5499 处理
			gcode->g_value[i] = code * 10;
		}

		// @add zk

		if((code >=5401 && code <=5499) &&
				(0x00 == (mode_mask & (0x01<<14)))){   //特殊处理G5401~G5499
			mode_mask |= (0x01<<14);
			code /= 100;   //缩小到有效范围内
			count++;
		}else if(code == 53){   //G53指令特殊处理,不计入0组模态，因为G53可以与G31等指令同行同行
			this->m_b_has_g53 = true;
			count++;
		}else if(gcode->g_value[i] == G84_3_CMD){  //G84.3计入39组非模态自定义指令
			code = kMaxGCodeCount-1;
			if(0x00 == (mode_mask & (0x01<<GCode2Mode[code]))){
				mode_mask |= (((uint64_t)0x01)<<GCode2Mode[code]);
				count++;
			}
		}else if((code < kMaxGCodeCount) &&
				(0x00 == (mode_mask & (0x01<<GCode2Mode[code])))){//不存在同组G代码
			mode_mask |= (((uint64_t)0x01)<<GCode2Mode[code]);
			count++;
		}else if(code >= kMaxGCodeCount){
			code = kMaxGCodeCount-1;
			if(0x00 == (mode_mask & (0x01<<GCode2Mode[code]))){
				mode_mask |= (((uint64_t)0x01)<<GCode2Mode[code]);
				count++;
			}
		}

		m_mode_code[GCode2Mode[code]] = gcode->g_value[i];
	}
	this->m_mode_mask = mode_mask;

	//删除无效的同组G代码，G代码顺序会按照模态组由小到大重新排序
//	if(count < gcode->gcode_count){
//		gcode->gcode_count = count;
//		memset(gcode->g_value, 0x00, kMaxGCodeInLine*sizeof(int));
//		int index = 0;
//		for(int i = 0; i < kMaxGModeCount; i++){
//			if(mode_mask &0x01){
//				gcode->g_value[index++] = m_mode_code[i];
//				if(index == count)
//					break;
//			}
//			mode_mask = mode_mask>>1;
//		}
//	}

	//TODO 检查是否有不能同行的指令
	//00/01/09组模态指令不能同行
	if(count > 1){  //多个G指令才做检查
		int cc = 0;
		if(m_mode_mask & GMODE_00){  //排除G53指令，G53指令可以去01组或者09组移动指令同行，G53只在当前行起效
			cc++;
		}
		if(m_mode_mask & GMODE_01)
			cc++;
		if((m_mode_mask & GMODE_09) && (m_mode_code[9] != G80_CMD))
			cc++;
		if(cc > 1){//告警
			m_error_code = ERR_NOT_IN_ONE_LINE;  //存在不可同行的指令
			return false;
		}
	}

	// 若存在 M#XXX 先计算出宏表达式的值
	for(int i=0; i<gcode->mcode_count; i++){
		if(gcode->m_value[i] < 0){//为负值，说明此M代码含有宏表达式
			exp_index = abs(gcode->m_value[i] + 1);
			while(!this->GetExpressionResult(gcode->macro_expression[exp_index], res)){
				if(this->m_error_code != ERR_NONE)
					return false;
				usleep(10000);  //休眠10ms，等待MC运行到位
			}
			if(res.init){
				gcode->m_value[i] = res.value;
			}else{
				m_error_code = ERR_M_EXP_NULL;   //
				return false;
			}

		}
	}

	//是否存在不能同行的M代码
	if(gcode->mcode_count > 1){
		for(int i = 0; i < gcode->mcode_count; i++){
			for(int j = 0; j < kSeparatedMCodeCount; j++){
				if(gcode->m_value[i] == kSeparatedMCodeArray[j]){
					m_error_code = ERR_MCODE_SEPARATED;  //存在不能同行的M指令
					return false;
				}
			}
		}
	}

	/***************/
	// 若存在 T#XXX 先计算宏表达式
	for(int i=0; i<gcode->tcode_count; i++){
		if(gcode->t_value[i] < 0){//为负值，说明此T代码含有宏表达式
			exp_index = abs(gcode->t_value[i] + 1);
			while(!this->GetExpressionResult(gcode->macro_expression[exp_index], res)){
				if(this->m_error_code != ERR_NONE)
					return false;
				usleep(10000);  //休眠10ms，等待MC运行到位
			}
			if(res.init){
				gcode->t_value[i] = res.value;
			}else{
				m_error_code = ERR_T_EXP_NULL;   //
				return false;
			}
		}
	}
	/*********************/

	//检查是否存在自定义指令及系统定义的宏程序指令
	this->m_b_call_macro_prog = this->HasMacroProgCall();

//	printf("Exit CheckGCode\n");
	return true;
}


/**
 * @brief 是否存在调用宏程序的指令
 * @return true--存在    false--不存在
 */
bool Parser::HasMacroProgCall(){

	//存在G65指令
	if((m_mode_mask & GMODE_00) && (m_mode_code[0] == G65_CMD)){
		return true;
	}

	//存在09组固定循环指令
	if(m_mode_mask & GMODE_09){
		return true;
	}


	LexerGCode *gcode = &(m_p_lexer_result->nc_code.gcode);
	for(int i = 0; i < gcode->mcode_count; i++){
//		if(gcode->m_value[i] == 6){//存在M06换刀指令
//			return true;
//		}

		//TODO 存在自定义M指令
	}

	//TODO 存在自定义G指令


	return false;
}

bool Parser::IsSysVar(int index){

	if(index < 1000 || (index >= 50000 && index <55000))
		return false;
	else if(index >= 1000)
		return true;
	return false;
}


/**
 * @brief 分析GCode并生成对应的Message
 * @param gcode : 词法分析结果指针
 * @return true--成功  false--失败
 */
bool Parser::AnalyzeGCode(LexerGCode *gcode){
//	if(gcode == nullptr){
//		m_error_code = ERR_COMPILER_INTER;
//		return false;
//	}

	//printf("mask mode : 0x%llx\n", m_mode_mask);
	bool has_move_code = false;  //是否有轴移动G指令
	bool user_defined_code = false;  //是否有用户自定义指令
	bool macro_call_flag = false;   //当前是否处于宏程序调用状态



	if(((m_mode_mask & GMODE_00) && (m_mode_code[0] == G65_CMD)) ||  //G65宏程序调用
			(m_mode_mask & GMODE_09) ||                              //固定循环指令
			(((m_mode_mask & GMODE_01) == 0) && (m_p_compiler_status->mode.gmode[9] != G80_CMD))){
		macro_call_flag = true;
	}

	if(!macro_call_flag){  //没有宏程序调用指令时才处理FeedMsg和ToolMsg
		//处理F指令
		if(!m_b_call_macro_prog && (gcode->mask_value & (0x01<<F_DATA))){
			if(!this->CreateFeedMsg())
				return false;
		}

		//处理T指令,预选刀
		if(!m_b_call_macro_prog && (gcode->mask_value & (0x01<<T_DATA))){
			if(!this->CreateToolMsg(gcode->t_value, gcode->tcode_count))
				return false;
		}

		//@test zk 截取 D H的值  原规则 D H 并未单独作为指令被处理
		if(gcode->mask_value&(0x01<<D_DATA)){

		}

		if(gcode->mask_value&(0x01 << H_DATA)){

		}
	}

	//处理自定义指令,第39组模态
	if(m_mode_mask & GMODE_39){
		user_defined_code = true;
		if(m_mode_code[kMaxGModeCount-1] == G200_CMD){
			if(!this->CreateClearCirclePosMsg())
				return false;
		}else if(m_mode_code[kMaxGModeCount-1] == G120_CMD){
			if(!CreateInfoMsg()){
				return false;
			}
		}else if(m_mode_code[kMaxGModeCount-1] == G84_3_CMD){
			if(!CreateModeMsg(m_mode_code[kMaxGModeCount-1]))
				return false;
		}
#ifdef USES_SPEED_TORQUE_CTRL
		else if(m_mode_code[kMaxGModeCount-1] == G1000_CMD || 
			m_mode_code[kMaxGModeCount-1] == G1001_CMD|| 
			m_mode_code[kMaxGModeCount-1] == G1002_CMD|| 
			m_mode_code[kMaxGModeCount-1] == G1003_CMD){  //创建速度控制指令消息
			if(!CreateSpeedCtrlMsg(m_mode_code[kMaxGModeCount-1]))
				return false;
		}else if(m_mode_code[kMaxGModeCount-1] == G2000_CMD ||
		m_mode_code[kMaxGModeCount-1] == G2001_CMD ||
		m_mode_code[kMaxGModeCount-1] == G2002_CMD ||
		m_mode_code[kMaxGModeCount-1] == G2003_CMD){  //创建力矩控制指令消息
			if(!CreateTorqueCtrlMsg(m_mode_code[kMaxGModeCount-1]))
				return false;
		}
#endif
	}


	//处理21组模态指令：G12.1/G12.2/G12.3/G13.1  极坐标插补及磨床相关指令
	if(m_mode_mask & GMODE_21){
		if(!this->CreatePolarIntpMsg(m_mode_code[21])){
			return false;
		}
	}

	//处理02组模态指令：G17/G18/G19 平面指定指令
	if(m_mode_mask & GMODE_02){
		if(!CreateModeMsg(m_mode_code[2]))
			return false;
	}

	//处理06组模态指令：G20/G21 公制英制转换
	if(m_mode_mask & GMODE_06){
		if(!CreateModeMsg(m_mode_code[6]))
			return false;
	}


	//处理19组模态指令：G25/G26 主轴速度变动检测ON/OFF
	if(m_mode_mask & GMODE_19){
		if(!this->CreateSpindleCheckMsg()){
			return false;
		}
	}

	//处理03组模态指令：G90/G91 绝对、增量指令
	if(m_mode_mask & GMODE_03){
		if(!CreateModeMsg(m_mode_code[3]))
			return false;
	}



	//处理05组模态指令：G94/G95 每分钟进给，每转进给
	if(m_mode_mask & GMODE_05){
		if(!CreateModeMsg(m_mode_code[5]))
			return false;
	}


	//处理00组非模态指令
	if(m_mode_mask & GMODE_00){//TODO 处理00组非模态指令：G04/G10/G28/G29/G71/G72/G92/G52等等, G53指令不在此处理
		if(m_mode_code[0] == G52_CMD ||
//				m_mode_code[0] == G53_CMD ||
				m_mode_code[0] == G92_CMD){  //坐标系指令
			if(!CreateCoordMsg(m_mode_code[0])){
				return false;
			}
//			else if(m_mode_code[0] == G53_CMD)
//				has_move_code = true;
		}else if(m_mode_code[0] == G04_CMD){
			if(!CreateTimeWaitMsg()){
				return false;
			}
		}else if(m_mode_code[0] == G27_CMD or m_mode_code[0] == G28_CMD or m_mode_code[0] == G30_CMD){
			if(!CreateRefReturnMsg(m_mode_code[0]))
				return false;
			else
				has_move_code = true;
		}else if(m_mode_code[0] == G31_CMD){
			if(!CreateSkipRunMsg(m_mode_code[0]))
				return false;
			else
				has_move_code = true;
		}else if(m_mode_code[0] == G65_CMD){ //宏程序调用
			if(!CreateMacroProgCallMsg())
				return false;
		}else if(m_mode_code[0] == G37_CMD){
			if(!CreateAutoToolMeasureMsg())
				return false;
			else
				has_move_code = true;
		}else if(m_mode_code[0] == G10_CMD){
			if(!CreateInputMsg()){
				return false;
			}
		}else if(m_mode_code[0] == G09_CMD){
			if(!CreateExactStopMsg()){
				return false;
			}
		}
		else{
			if(!CreateModeMsg(m_mode_code[0]))
				return false;
		}
	}


	//TODO 处理07组模态指令：G40/G41/G42 半径补偿指令
	if(m_mode_mask & GMODE_07){
		//has_move_code = true;
		if(!this->CreateCompensateMsg(m_mode_code[7])){
			return false;
		}
	}

	//TODO 处理08组模态指令：G43/G44/G49 长度补偿指令
	if(m_mode_mask & GMODE_08){
		has_move_code = true;
		if(!this->CreateCompensateMsg(m_mode_code[8])){
			return false;
		}
	}

	//TODO 处理14组模态指令：G54~G59 工件坐标系选择指令，包括G5401~G5499扩展工件坐标系
	if(m_mode_mask & GMODE_14){

		printf("create gcode : %d\n", m_mode_code[14]);
		if(!CreateCoordMsg(m_mode_code[14]))
			return false;
	}

	if(m_mode_mask & GMODE_15){
		if(!CreateModeMsg(m_mode_code[15]))
			return false;
	}

	//处理10组模态指令：G98/G99 固定循环（初始平面/R点平面）返回
	if(m_mode_mask & GMODE_10){
		if(!CreateModeMsg(m_mode_code[10]))
			return false;
	}


	//处理09组模态指令：G73/G74/G76/G80/G81~G89 固定循环指令
	if(m_mode_mask & GMODE_09){
		has_move_code = true;
		if(!CreateLoopMsg(m_mode_code[9]))
			return false;
	}

	//处理01组模态指令：G00/G01/G02/G03/G6.2/G33~G36 切削类指令
	if(m_mode_mask & GMODE_01 && !has_move_code){

		has_move_code = true;
		switch(m_mode_code[1]){
		case G00_CMD:
			if(!CreateRapidMsg())
				return false;
			break;
		case G01_CMD:
			if(!CreateLineMsg())
				return false;
			break;
		case G02_CMD:
		case G03_CMD:
			if(!CreateArcMsg(m_mode_code[1]))
				return false;
			break;
		default:
			//生成不支持指令错误消息
			printf("unsupported G code[%d]!\n", m_mode_code[1]);
			m_error_code = ERR_INVALID_CODE;
			return false;
		}

		MoveMsg * node = (MoveMsg *)m_p_parser_result->TailNode();
		node->setCancelG80(true);

	}



	// 保存之前的模态处理
	if(!has_move_code && !user_defined_code){//有轴位置指令，但是无轴移动指令和自定义G指令
	//	printf("no move cmd!\n");
		if(HasAxisPos()){
	//		printf("no move cmd, has axis pos!\n");

			//存在G53则生成G53指令
			if(this->m_b_has_g53){
				if(!CreateCoordMsg(m_mode_code[0])){
					return false;
				}
				else
					has_move_code = true;
				this->m_b_has_g53 = false;
			}else if(m_p_compiler_status->mode.gmode[9] == G80_CMD){//01组模态
				switch(m_p_compiler_status->mode.gmode[1]){
				case G00_CMD:
					if(!CreateRapidMsg())
						return false;
					break;
				case G01_CMD:
					if(!CreateLineMsg())
						return false;
					break;
				case G02_CMD:
				case G03_CMD:
					if(!CreateArcMsg(m_p_compiler_status->mode.gmode[1]))
						return false;
					break;
				default:
					//生成不支持指令错误消息
					printf("no move code, unsupported G code[%d]!\n", m_p_compiler_status->mode.gmode[1]);
					m_error_code = ERR_INVALID_CODE;
					return false;
				}

			}else{//09组模态
				if(!CreateLoopMsg(m_p_compiler_status->mode.gmode[9]))
					return false;
			}
		}
	}

	return true;
}

/**
 * @brief 是否存在轴位置指令
 * @return true--存在轴位置   false--不存在
 */
bool Parser::HasAxisPos(){
	int addr = 0;
	int i = 0;
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);
	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);


	while(m_axis_name[i] != '\0'){
		if(this->m_b_axis_name_ex){  //允许通道轴名称扩展
			addr = m_axis_name[i]-'A';
			if(m_axis_name_ex[i] == 0 && (g_code->mask_value & (0x01<<addr))){ //是否存在addr参数
				return true;
			}else if(m_axis_name_ex[i] > 0 &&
					(g_code->mask_pos[chn_config->chn_axis_name[i]] & (0x01<<(m_axis_name_ex[i]-1)))){
				return true;
			}
		}else{//不允许通道轴名称扩展
			addr = m_axis_name[i]-'A';
			if(g_code->mask_value & (0x01<<addr)){ //是否存在addr参数
				return true;
			}
		}

		i++;
	}

	//考虑圆弧指令的IJKR指令
	if(m_p_compiler_status->mode.gmode[9] == G80_CMD &&
			(m_p_compiler_status->mode.gmode[1] == G02_CMD || m_p_compiler_status->mode.gmode[1] == G03_CMD)){
		char tmp[4] = "IJK";
		i = 0;
		while(tmp[i] != '\0'){
			addr = tmp[i]-'A';
			if(g_code->mask_value & (0x01<<addr)){ //是否存在addr参数
				return true;
			}
			i++;
		}
	}

	return false;
}

/**
 * @brief 处理M指令
 * @param gcode : 词法分析结果指针
 * @return true--成功  false--失败
 */
bool Parser::ProcessMCode(LexerGCode *gcode){

	//处理S指令,放在M指令之前处理
	if(!m_b_call_macro_prog && (gcode->mask_value & (0x01<<S_DATA))){
		if(!this->CreateSpeedMsg())
			return false;
	}

	if(gcode->mcode_count == 1){
		if(gcode->m_value[0] == 98){  //子程序调用M指令
			if(!this->CreateSubProgCallMsg()){
				return false;
			}
		}else if(!this->CreateAuxMsg(gcode->m_value, 1))
			return false;

	}else if(gcode->mcode_count > 1){

		if(!this->CreateAuxMsg(gcode->m_value, gcode->mcode_count))
			return false;
	}

	return true;
}

/**
 * @brief 计算表达式的结果
 * @param express : 表达式
 * @param res ：输出的计算结果
 * @return true--结果计算成功   false--结果计算失败
 */
bool Parser::GetExpressionResult(MacroExpression &express, MacroVarValue &res){
	stack<MacroVarValue> stack_value;  //中间值栈
	MacroExpression macro_bak;   //备份
	res.init = false;

	if(express.empty()){//表达式为空，错误的宏指令格式，告警
		m_error_code = ERR_INVALID_MACRO_EXP;
		printf("@@@@@@ERR_INVALID_MACRO_EXP1\n");
		return false;
	}

	//清空中间值栈
	while(!stack_value.empty()){
		stack_value.pop();
	}

	int value_count = 0;
	MacroRec rec;
	MacroVarValue value1, value2;  //操作数1，操作数2
	MacroVarValue res_tmp;
	while(!express.empty()){
		rec = express.top();
		macro_bak.push(rec);   //暂存备份
		express.pop();
//		printf("get express res, rec,opt = %d, value = %lf\n", rec.opt, rec.value);
		if(rec.opt == MACRO_OPT_VALUE){
			value1.value = rec.value;
			value1.init = true;
			stack_value.push(value1);
//			printf("macro value : %lf, size = %d\n", rec.value, stack_value.size());
		}
		else if(rec.opt == MACRO_OPT_FIX){//下取整，舍去小数部分
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				printf("@@@@@@ERR_INVALID_MACRO_EXP2\n");
				return false;
			}

			value1 = stack_value.top();
			stack_value.pop();
			if(value1.init){
				res_tmp.value = floor(value1.value);
				res_tmp.init = true;
			}
			else
				res_tmp.init = false;	//空值取整仍然为空值
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_FUP){ //上取整，将小数部分进位到整数部分
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}

			value1 = stack_value.top();
			stack_value.pop();
			if(value1.init){
				res_tmp.value = ceil(value1.value);
				res_tmp.init = true;
			}
			else
				res_tmp.init = false;	//空值取整仍然为空值
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_ROUND){//四舍五入，在算数或逻辑运算指令中使用时，在第1个小数位置四舍五入，在NC语句地址中使用时，根据地址最小设定单位四舍五入
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.init){
				res_tmp.value = floor(value1.value+0.5);
				res_tmp.init = true;
			}
			else
				res_tmp.init = false;	//空值取整仍然为空值
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_ABS){//绝对值
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.init){
				res_tmp.value = fabs(value1.value);
				res_tmp.init = true;
			}
			else
				res_tmp.init = false;	//空值取整仍然为空值
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_SIN){ //正弦
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.init){
				res_tmp.value = sin(value1.value*M_PI/180.);
				res_tmp.init = true;
			}
			else{
				res_tmp.init = false;	//空值三角运算仍然为空值
				res_tmp.value = 0.0;
			}
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_COS){//余弦
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.init){
				res_tmp.value = cos(value1.value*M_PI/180.);
				res_tmp.init = true;
			}
			else{
				res_tmp.init = false;	//空值三角运算仍然为空值
				res_tmp.value = 0.0;
			}

			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_TAN){//正切
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.init){
				res_tmp.value = tan(value1.value*M_PI/180.);
				res_tmp.init = true;
			}
			else{
				res_tmp.init = false;	//空值三角运算仍然为空值
				res_tmp.value = 0.0;
			}

			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_ASIN){ //反正弦
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.init){
				if(value1.value > 1 || value1.value < -1){  //操作数不在有效范围
					m_error_code = ERR_MACRO_OPT_VALUE;
					return false;
				}
				res_tmp.value = asin(value1.value);
				res_tmp.value = res_tmp.value*180./M_PI;  //由弧度转换为度
				res_tmp.init = true;
			}else{
				res_tmp.init = false;   //空值三角运算仍然为空值
				res_tmp.value = 0.0;
			}
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_ACOS){//反余弦
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.init){
				if(value1.value > 1 || value1.value < -1){  //操作数不在有效范围
					m_error_code = ERR_MACRO_OPT_VALUE;
					return false;
				}
				res_tmp.value = acos(value1.value);
				res_tmp.value = res_tmp.value*180./M_PI;  //由弧度转换为度
				res_tmp.init = true;
			}
			else{
				res_tmp.init = false;	//空值三角运算仍然为空值
				res_tmp.value = 0.0;
			}
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_ATAN){//反正切
			if(rec.value == 2){//双操作数，返回值范围：-PI~PI
				if(stack_value.size() < 2){
					m_error_code = ERR_INVALID_MACRO_EXP;
					return false;
				}
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
				if(value1.init && value2.init){
					res_tmp.value = atan2(value2.value, value1.value);
					res_tmp.init = true;
				}else{
					res_tmp.init = false;
					res_tmp.value = 0.0;
				}

			}
			else{//单操作数， 返回值范围：-PI/2~PI/2
				if(stack_value.empty()){//找不到操作数，告警
					m_error_code = ERR_INVALID_MACRO_EXP;
					return false;
				}
				value1 = stack_value.top();
				stack_value.pop();
				if(value1.init){
					res_tmp.value = atan(value1.value);
					res_tmp.init = true;
				}else{
					res_tmp.init = false;
					res_tmp.value = 0.0;
				}
			}

			res_tmp.value = res_tmp.value*180./M_PI;//由弧度转换为度
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_SQRT){ //平方根
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
//			if(!value1.init)
//				value1.value = 0.0;

			if(value1.value < 0){  //操作数不在有效范围
				m_error_code = ERR_MACRO_OPT_VALUE;
				return false;
			}
			res_tmp.value = sqrt(value1.value);

			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_LN){//自然对数
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
//			if(!value1.init)
//				value1.value = 0.0;

			if(value1.value < 0){  //操作数不在有效范围
				m_error_code = ERR_MACRO_OPT_VALUE;
				return false;
			}
			res_tmp.value = log(value1.value);
			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_EXP){//指数函数
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
//			if(!value1.init)
//				value1.value = 0.0;
			res_tmp.value = exp(value1.value);
			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_EQ){ //等于
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				value2.init = true;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}

			if(value1.init && value2.init){
				if(fabs(value1.value - value2.value) <= MZERO)
					res_tmp.value = 1;
				else
					res_tmp.value = 0;
			}else if(!value1.init && !value2.init){
				res_tmp.value = 1;  //都为空值时，判断为真
			}
			else  //其中一个为空时，判断为假
				res_tmp.value = 0;
			res_tmp.init = true;

			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_NE){//不等于
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				value2.init = true;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}

			if(value1.init && value2.init){
				if(fabs(value1.value-value2.value) > MZERO)
					res_tmp.value = 1;
				else
					res_tmp.value = 0;
			}else if(!value1.init && !value2.init)
				res_tmp.value = 0;	//都为空值时，判定为假
			else
				res_tmp.value = 1;	//只有一个为空值时，判定为真
			res_tmp.init = true;

			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_GT){//大于
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				value2.init = true;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			//空值当0处理
			if(value2.value > value1.value)
				res_tmp.value = 1;
			else
				res_tmp.value = 0;

			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_LT){ //小于
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				value2.init = true;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}

			//空值当0处理
			if(value2.value < value1.value)
				res_tmp.value = 1;
			else
				res_tmp.value = 0;


			res_tmp.init = true;
		//	printf("LT resulst: %lf, %hhu\n", res_tmp.value, res_tmp.init);
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_GE){//大于等于
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				value2.init = true;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}

			//空值当0处理
			if(value2.value >= value1.value)
				res_tmp.value = 1;
			else
				res_tmp.value = 0;


			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_LE){//小于等于
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				value2.init = true;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}

			//空值当0处理
			if(value2.value <= value1.value)
				res_tmp.value = 1;
			else
				res_tmp.value = 0;


			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_OR){ //或
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				value2.init = true;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
//			if(!value1.init)
//				value1.value = 0.0;
//			if(!value2.init)
//				value2.value = 0.0;

			res_tmp.value = Double2Long(value2.value) | Double2Long(value1.value);
			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_XOR){//异或
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				value2.init = true;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}

//			if(!value1.init)
//				value1.value = 0.0;
//			if(!value2.init)
//				value2.value = 0.0;

			res_tmp.value = Double2Long(value2.value) ^ Double2Long(value1.value);
			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_AND){//与
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
//			if(!value1.init)
//				value1.value = 0.0;
//			if(!value2.init)
//				value2.value = 0.0;

			res_tmp.value = Double2Long(value2.value) & Double2Long(value1.value);
			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_RD){ //读取，#
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				printf("@@@@@@ERR_INVALID_MACRO_EXP3\n");
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
//			if(!value1.init)
//				value1.value = 0.0;

			if(this->IsSysVar(static_cast<int>(value1.value))){
				//判断MC是否运行到位
				if(!ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index)->IsBlockRunOver() ||
						!ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index)->IsOutputMsgRunover()){
//					printf("wait run over : blockflag=%hhu, outputempty=%hhu\n", ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index)->IsBlockRunOver(),
//							ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index)->IsOutputMsgRunover());
					goto REC;  //终止计算，恢复数据
				}
			}
		//	res_tmp = GetMacroVar(static_cast<int>(value1));
			if(!GetMacroVar(static_cast<int>(value1.value), res_tmp.value, res_tmp.init)){
				m_error_code = ERR_INVALID_MACRO_EXP;
				printf("@@@@@@ERR_INVALID_MACRO_EXP4\n");
				return false;
			}

//			printf("read var: %d , %lf\n", static_cast<int>(value1.value), res_tmp.value);

			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_WR){//赋值，=
			if(stack_value.size() < 2){
				m_error_code = ERR_INVALID_MACRO_EXP;
				printf("@@@@@@ERR_INVALID_MACRO_EXP5\n");
//				printf("MACRO_OPT_WR size = %d, return \n", stack_value.size());
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			value2 = stack_value.top();
			stack_value.pop();
//			if(!value2.init)
//				value2.value = 0.0;

			if(this->IsSysVar(static_cast<int>(value2.value))){
				//判断MC是否运行到位
				if(!ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index)->IsBlockRunOver())
					goto REC;  //终止计算，恢复数据
			}

			if(!SetMacroVar(static_cast<int>(value2.value), value1.value, value1.init)){
				printf("MACRO_OPT_WR error\n");
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}else{
				printf("MACRO_OPT_WR succeed\n");
			}
		}
		else if(rec.opt == MACRO_OPT_ADD){//加法
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				value2.init = true;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}

//			if(!value1.init)
//				value1.value = 0.0;
//			if(!value2.init)
//				value2.value = 0.0;

			res_tmp.value = value2.value + value1.value;
			res_tmp.init = true;

			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_SUB){ //减法
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 && value_count >=1 && !express.empty()){
				if(express.top().opt == MACRO_OPT_VALUE){  //双操作数
					value1 = stack_value.top();
					stack_value.pop();
					value2.value = express.top().value;
					value2.init = true;
					express.pop();
				}else if(express.top().opt == MACRO_OPT_WR){  //处理形如：#1=-10的赋值语句
					value1 = stack_value.top();
					stack_value.pop();
					value2.value = 0;
					value2.init = true;
				}else{
					m_error_code = ERR_INVALID_MACRO_EXP;
					printf("@@@@@@ERR_INVALID_MACRO_EXP6, rec.value=%lf, value_count=%d, express=%hhu\n", rec.value, value_count, express.empty());
					return false;
				}
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				printf("@@@@@@ERR_INVALID_MACRO_EXP7, rec.value=%lf, value_count=%d, express=%hhu\n", rec.value, value_count, express.empty());
				return false;
			}

//			if(!value1.init)
//				value1.value = 0.0;
//			if(!value2.init)
//				value2.value = 0.0;

			res_tmp.value = value2.value - value1.value;
			res_tmp.init = true;

			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_MULTIPLY){//乘法
			value_count = stack_value.size();
//			printf("stack_size = %d, exp_size =%d,top_opt = %d\n", value_count, express.size(), express.top().opt);
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				value2.init = true;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}

			res_tmp.value = value2.value * value1.value;
			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_DIVIDE){//除法
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				value2.init = true;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}

			if(value1.value == 0.0){
				m_error_code = ERR_MACRO_DIV_ZERO;
				return false;
			}

			res_tmp.value = value2.value/value1.value;
			res_tmp.init = true;

			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_BIN){//从BCD码转为二进制码   #1=BIN[#2]
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.value < 0){  //操作数不在有效范围
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			res_tmp.value = BCD2BIN(Double2Long(value1.value));
			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_BCD){//从二进制码转为BCD码   #1=BCD[#2]
			if(stack_value.empty()){//找不到操作数，告警
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.value < 0){  //操作数不在有效范围
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			res_tmp.value = BIN2BCD(Double2Long(value1.value));
			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_POW){//乘方  #1=POW[#2,#3]
			if(stack_value.size() < 2){
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			value2 = stack_value.top();
			stack_value.pop();
			res_tmp.value = pow(value2.value, value1.value);
			res_tmp.init = 0;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_MOD){//求余数  #i=#j MOD #k (#j、#k取整后求取余数，#j为负时，#i也为负)
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 &&
					value_count >=1 &&
					!express.empty() &&
					express.top().opt == MACRO_OPT_VALUE){
				value1 = stack_value.top();
				stack_value.pop();
				value2.value = express.top().value;
				value2.init = true;
				express.pop();
			}
			else{
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}

			res_tmp.value = Double2Long(value2.value) % Double2Long(value1.value);
			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else{
			m_error_code = ERR_INVALID_MACRO_EXP;
			return false;
		}
	}

	value_count = stack_value.size();
	if(value_count > 1){
		m_error_code = ERR_MACRO_EXP_CAL;
		return false;
	}else if(value_count == 1){
		res = stack_value.top();
		stack_value.pop();
	}
//	printf("get exp result: value = %lf, init = %hhu\n", res.value, res.init);
	return true;

	REC:  //恢复数据
	while(!macro_bak.empty()){
		rec = macro_bak.top();
		express.push(rec);
		macro_bak.pop();
	}
	usleep(1000);  //休眠1ms
	return false;
}

/**
 * @brief BCD转BIN
 * @param val
 */
long Parser::BCD2BIN(long val){
  	long temp;
   	long bcd=0l;
   	long dec=1l;
   	int size = 2*sizeof(long);
   	for(int i=0;i<size;i++){
       	temp=(val&0x0f)*dec;
       	dec*=10;
       	val>>=4;
       	bcd+=temp;
	}
   	return bcd;
}

/**
 * @brief BIN转BCD
 * @param val
 */
long Parser::BIN2BCD(long val){
	long temp;
	long bin=0;
	int size = 2*sizeof(long);
	for(int i=0;i<size;i++){
		temp=val%10;
		val/=10;
		bin|=(temp<<(i*4));
	}
	return bin;
}

/**
 * @brief 将double型数据转换为long型
 * @param data : 待转换数据
 */
long Parser::Double2Long(const double &data){
	if (data >= 0)
		return data+0.5;
	else
		return data-0.5;
}

/**
 * @brief 读取宏变量的值
 * @param index[in] ：变量下标
 * @param value[out] : 变量值
 * @param init[out] : 是否空值
 * @return
 */
bool Parser::GetMacroVar(int index, double &value, bool &init){

	return this->m_p_variable->GetVarValue(index, value, init);

}

/**
 * @brief 设置宏变量的值
 * @param index[in] ：变量下标
 * @param value[in] : 变量值
 * @param init[in] : 是否空值
 * @return true--成功  false--失败
 */
bool Parser::SetMacroVar(int index, double value, bool init){

	if(init)
		return this->m_p_variable->SetVarValue(index, value);
	else
		return this->m_p_variable->ResetVariable(index);
}

/**
 * @brief 构造编译错误消息，并插入消息队列
 * @param err : 错误码
 * @return true--成功  false--失败
 */
bool Parser::CreateErrorMsg(const ErrorType err){

	RecordMsg *new_msg = new ErrorMsg(err);
	if(new_msg == nullptr){
		//内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号

	if(this->m_p_compiler_status->jump_flag){
		new_msg->SetFlag(FLAG_JUMP, true);
		this->m_error_code = ERR_NONE;
	}

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief 构造信息提示消息，并插入消息队列
 * @return true--成功  false--失败
 */
bool Parser::CreateInfoMsg(){
	int info_code = 0;
	int info_type = 0;
	double value = 0.0;

	//读取信息编号
	if(!GetCodeData(P_DATA, value)){
		if(m_error_code == ERR_NONE)
			m_error_code = ERR_NO_P_DATA;
		return false;
	}

	info_code = static_cast<int>(value);

	//读取信息类型，可省略，默认为0-提示信息
	if(GetCodeData(L_DATA, value)){
		info_type = static_cast<int>(value);
	}

	if(info_type != 0 && info_type != 1){
		m_error_code = ERR_INVALID_L_DATA;
		return false;
	}

	printf("create info msg, info_code = %d, info_type=%d\n", info_code, info_type);

	RecordMsg *new_msg = new ErrorMsg(info_code);
	if(new_msg == nullptr){
		//内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号

	((ErrorMsg *)new_msg)->SetInfoType(info_type);


	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief 构造模态消息，并插入消息队列，此消息适用于无参数的模态G指令
 * @param gcode : 无参数的模态G指令
 * @return true--成功  false--失败
 */
bool Parser::CreateModeMsg(const int gcode){
	RecordMsg *new_msg = new ModeMsg(gcode);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}


	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	//if(ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index)->IsStepMode())
	//	new_msg->SetFlag(FLAG_WAIT_MOVE_OVER, true);
	//@ 解决 G90  G01 xxx G91  中间的运动指令执行不了 G91指令等待运动结束 导致程序卡死问题
	new_msg->SetFlag(FLAG_WAIT_MOVE_OVER, true);
//	if(gcode == G90_CMD || gcode == G91_CMD || gcode == G20_CMD || gcode == G21_CMD){
//		m_p_compiler_status->mode.gmode[GetModeGroup(gcode)] = gcode;   //G91/G90可能与轴移动指令同行，必须提前动作
//	}

	if(gcode == G17_CMD || gcode == G18_CMD || gcode == G19_CMD){
		m_p_compiler_status->mode.gmode[GetModeGroup(gcode)] = gcode;   //G17/G18/G19可能与轴移动指令同行，必须提前动作
	}

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief 构造循环指令消息，并插入消息队列
 * @param gcode : 循环G指令
 * @return true--成功  false--失败
 */
bool Parser::CreateLoopMsg(const int gcode){
//	DPointChn source = this->m_p_compiler_status->cur_pos;   //起点
//	DPointChn target = source;	//终点
//	uint32_t mask = 0;   //轴mask

//	if(gcode != G80_CMD){
//		if(!GetTargetPos(target, mask))
//			return false;
//	}

	//获取参数
	double *param = nullptr;
	uint8_t pc = 0;  //参数个数
	uint32_t pm = 0;  //参数mask
	this->GetParaData(&param, pc, pm);

	RecordMsg *new_msg = new LoopMsg(gcode, param, pc, pm);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);
	//@test zk run message时会去修改模态  在这里修改太早了
	//this->m_p_compiler_status->mode.gmode[GetModeGroup(gcode)] = gcode;
//	this->m_p_compiler_status->mode.move_mode = 9;

//	printf("create loop msg : %d\n", gcode);
	ProcessLastBlockRec(new_msg);

//	this->m_p_compiler_status->cur_pos = target; //更新编译当前位置
	return true;
}

/**
 * @brief 构造坐标系指令消息，并插入消息队列
 * @param gcode : 坐标系G指令，G52/G53/G92/
 * @return true--成功  false--失败
 */
bool Parser::CreateCoordMsg(const int gcode){

	DPointChn pos;	//终点
	DPointChn src = this->m_p_compiler_status->cur_pos;   //起点
	uint32_t axis_mask = 0;   //轴掩码
	double data = 0;

	if(gcode == G52_CMD || gcode == G53_CMD || gcode == G92_CMD){
		if(!GetTargetPos(pos, axis_mask))
			return false;
	}

	if(gcode >= G5401_CMD and gcode <= G5499_CMD){
		// @test
		int coord = gcode/10 - 5400;
		SCChannelConfig * pChnConfig = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
		if(coord > pChnConfig->ex_coord_count){
			CreateError(ERR_NC_FORMAT, ERROR_LEVEL, CLEAR_BY_MCP_RESET,this->m_p_lexer_result->line_no);
			return false;
		}

	}

	RecordMsg *new_msg = new CoordMsg(pos, src, gcode, axis_mask);

	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);
//	printf("create coord msg:%d\n", gcode);

	return true;
}

/**
 * @brief 构造进给速度指令消息，并插入消息队列
 * @return true--成功  false--失败
 */
bool Parser::CreateFeedMsg(){
	double df_feed = 0;  //进给速度，单位：mm/min

	if(!GetCodeData(F_DATA, df_feed)){
		return false;
	}

//	int feed = static_cast<int>(df_feed);

	RecordMsg *new_msg = new FeedMsg(df_feed, m_p_compiler_status->mode.f_mode);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_b_f_code = true;
	m_p_parser_result->Append(new_msg);
	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 *
 * @param mcode ： M指令值数组指针
 * @param total : 同行M指令总数
 * @return
 */
bool Parser::CreateAuxMsg(int *mcode, uint8_t total){
	AuxMsg *new_msg = new AuxMsg(mcode, total);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);
	m_p_parser_result->Append(new_msg);
	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief 构造子程序调用消息，并插入消息队列
 * @return
 */
bool Parser::CreateSubProgCallMsg(){
	int pcode = 0;		//子程序名称
	int lcode = 1;  	//重复调用次数，默认一次

	double value = 0.0;   //临时变量
	if(!GetCodeData(P_DATA, value)){
		if(m_error_code == ERR_NONE)
			m_error_code = ERR_NO_SUB_PROG_P;  //没有指定子程序号
		return false;
	}else{
		pcode = static_cast<int>(value);
	}

	if(GetCodeData(L_DATA, value)){
		lcode = static_cast<int>(value);
	}

	RecordMsg *new_msg = new SubProgCallMsg(pcode, lcode);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);
	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief 构造主轴转速指令消息，并插入消息队列
 * @return true--成功  false--失败
 */
bool Parser::CreateSpeedMsg(){
	double df_speed = 0.0;

	GetCodeData(S_DATA, df_speed);

	int speed = static_cast<int>(df_speed);

	RecordMsg *new_msg = new SpeedMsg(speed);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);
	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief 构造刀具指令消息，并插入消息队列
 * @param tcode ： T指令值数组指针
 * @param total : 同行T指令总数
 * @return true--成功  false--失败
 */
bool Parser::CreateToolMsg(int *tcode, uint8_t total){

	if(*tcode < 0 or *tcode > 60){
		CreateError(ERR_T_EXP_NULL, ERROR_LEVEL, CLEAR_BY_MCP_RESET, m_p_lexer_result->line_no);
		return false;
	}

	RecordMsg *new_msg = new ToolMsg(tcode, total);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);
	m_p_parser_result->Append(new_msg);

//	this->m_p_compiler_status->mode.t_mode = tcode[0];   //修改编译器状态

	printf("CreateToolMsg, total count = %hhu\n", total);
	ProcessLastBlockRec(new_msg);
	return true;
}

/**
 * @brief 构造刀补指令消息，并插入消息队列
 * @param gcode : 刀补G指令
 * @return true--成功  false--失败
 */
bool Parser::CreateCompensateMsg(int gcode){

	DPointChn source = this->m_p_compiler_status->cur_pos;   //起点
	DPointChn target = source;	//终点

	double df_data = 0.0;
	uint16_t data = 0;
	uint32_t mask = 0;  //轴

	uint8_t move_type = MOVE_G00;

	if(gcode == G41_CMD || gcode == G42_CMD || gcode == G40_CMD){  //建立刀具半径补偿需要轴移动

//		if(!GetTargetPos(target, mask)){
//			return false;
//		}

		if(!GetCodeData(D_DATA, df_data))
		{
			if(gcode == G40_CMD){
				df_data = 0;
			}else if(m_error_code == ERR_NONE){
				m_error_code = ERR_NO_D_DATA;
				return false;
			}
		}
		data = static_cast<uint16_t>(df_data);

//		if(m_p_compiler_status->mode.gmode[1] == G01_CMD)
//			move_type = MOVE_G01;

	}else if(gcode == G43_CMD || gcode == G44_CMD || gcode == G49_CMD){  //刀具长度补偿
		if(!GetTargetPos(target, mask))
			return false;

		if(!GetCodeData(H_DATA, df_data))
		{
			if(gcode == G49_CMD){
				df_data = 0;
			}else if(m_error_code == ERR_NONE){//缺少H值指定
				m_error_code = ERR_NO_H_DATA;
				return false;
			}
		}
		data = static_cast<uint16_t>(df_data);

		if(m_p_compiler_status->mode.gmode[1] == G01_CMD)
			move_type = MOVE_G01;
	}else if(gcode == G43_4_CMD){   //激活五轴RTCP

		if(!GetTargetPos(target, mask))
			return false;

		if(!GetCodeData(H_DATA, df_data))
		{
			if(m_error_code == ERR_NONE)
				m_error_code = ERR_NO_H_DATA;
			return false;
		}
		data = static_cast<uint16_t>(df_data);
		if(m_p_compiler_status->mode.gmode[1] == G01_CMD)
			move_type = MOVE_G01;
		else if(m_p_compiler_status->mode.gmode[1] == G02_CMD)
			move_type = MOVE_G02;
		else if(m_p_compiler_status->mode.gmode[1] == G03_CMD)
			move_type = MOVE_G03;
	}



	RecordMsg *new_msg = new CompensateMsg(gcode, data, source, target, mask, m_p_compiler_status->mode.f_mode, move_type);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	//同行是否存在G53，即指定机械坐标
	if(this->m_b_has_g53){
		((CompensateMsg *)new_msg)->SetMachCoord(true);
	}

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

//	this->m_p_compiler_status->cur_pos = target; //更新编译当前位置

	return true;
}

/**
 * @brief 构造G00快速定位消息，并插入消息队列
 * @return true--成功  false--失败
 */
bool Parser::CreateRapidMsg(){
	DPointChn source = this->m_p_compiler_status->cur_pos;   //起点
	DPointChn target = source;	//终点
	uint32_t axis_mask = 0;

	uint8_t io = 0;
	double data = 0;
//	double pmc_data[this->m_n_pmc_axis_count];   //PMC轴位置数据
//	uint32_t pmc_mask = 0;
	uint8_t pmc_count = 0;
//	bool inc_flag = true;

	if(!GetTargetPos(target, axis_mask, &pmc_count))   //获取插补轴目标位置
		return false;

//	uint8_t count = this->GetTargetPosEx(pmc_data, pmc_mask, inc_flag, m_n_pmc_axis_count);   //获取PMC轴运动数据
//	if(count == 0xFF){
//		return false;
//	}

	RecordMsg *new_msg = new RapidMsg(source, target, axis_mask);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号

	if(GetCodeData(P_DATA, data)){   //IO数据
		io = static_cast<uint8_t>(data);
		((RapidMsg *)new_msg)->SetIoData(io);   //设置IO数据
	}

	if(pmc_count > 0){
        ((RapidMsg *)new_msg)->SetPmcAxisCount(pmc_count);
	}

	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	//同行是否存在G53，即指定机械坐标
	if(this->m_b_has_g53){
		((RapidMsg *)new_msg)->SetMachCoord(true);
	}

	m_p_parser_result->Append(new_msg);
    ProcessLastBlockRec(new_msg);

    // 如果对旋转轴移动，需要插入清整数圈的msg
//    SCAxisConfig *axis_config = g_ptr_parm_manager->GetAxisConfig();
//    SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig();
//    for(int i = 0; i < chn_config->chn_axis_count; i++){
//        uint8_t mask = axis_mask & (0x01 << i);
//        if(!mask || (axis_config[i].axis_type != AXIS_ROTATE))
//            continue;
//        double pos = target.GetAxisValue(i);
//        if(pos >= 0 && pos < 360)   // 不超范围，不需要清
//            continue;
//        // 添加一条G200消息,对旋转轴清整数圈
//        RecordMsg *clr_msg = new ClearCirclePosMsg(2000, mask, 360*1000);
//        new_msg->SetLineNo(this->m_p_lexer_result->line_no);
//        m_p_parser_result->Append(clr_msg);
//        ProcessLastBlockRec(clr_msg);
//        ScPrintf("append ClearCirclePosMsg, msk = %u",mask);
//    }

//	this->m_p_compiler_status->cur_pos = target; //更新编译当前位置

	return true;
}

/**
 * @brief 构造G01直线切削消息，并插入消息队列
 * @return true--成功  false--失败
 */
bool Parser::CreateLineMsg(){
//    ScPrintf("Parser::CreateLineMsg\n");

	DPointChn source = this->m_p_compiler_status->cur_pos;   //起点
	DPointChn target = source;	//终点
	RecordMsg *new_msg = nullptr;
	uint32_t axis_mask = 0;
	uint8_t io = 0;
	double data = 0;
//	printf("line msg target init: (%lf, %lf, %lf, %lf, %lf, %lf)\n",target.x, target.y, target.z, target.a4,
//			target.a5, target.a6);

//	double pmc_data[this->m_n_pmc_axis_count];   //PMC轴位置数据
//	uint32_t pmc_mask = 0;
//	bool inc_flag = true;
	uint8_t pmc_count = 0;

	if(!GetTargetPos(target, axis_mask, &pmc_count))
		return false;
   ScPrintf("Parser::CreateLineMsg pmc_count = %u", pmc_count);


//	uint8_t count = this->GetTargetPosEx(pmc_data, pmc_mask, inc_flag, m_n_pmc_axis_count);   //获取PMC轴运动数据
//	if(count == 0xFF){
//		return false;
//	}


//	printf("line msg get target : (%lf, %lf, %lf, %lf, %lf, %lf), mask = 0x%x\n",target.x, target.y, target.z, target.a4,
//			target.a5, target.a6, axis_mask);

	if(!m_b_f_code){//未指定F值
		printf("ERR_NO_F_DATA\n");
		m_error_code = ERR_NO_F_DATA;
		return false;
	}

//    // 如果在攻丝状态移动z轴，同时也指定主轴
//    uint8_t axis_z = m_p_channel_control->GetChnAxisFromName(AXIS_NAME_Z);
//    SpindleControl *spindle = m_p_channel_control->GetSpdCtrl();
//    if(spindle->isTapEnable() && (axis_mask & (0x01 << axis_z))){
//        axis_mask |= (0x01 << spindle->GetPhyAxis());

//        bool ret = spindle->CalTapPosition(target.m_df_point[axis_z],
//                                           target.m_df_point[spindle->GetPhyAxis()]);
//        if(!ret){
//            m_error_code = ERR_CAL_SPD_TAP_POS;
//            return false;
//        }
//    }

	new_msg = new LineMsg(source, target, m_p_compiler_status->mode.f_mode, axis_mask);

	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号

	if(GetCodeData(P_DATA, data)){   //IO数据
		io = static_cast<uint8_t>(data);
		((LineMsg *)new_msg)->SetIoData(io);   //设置IO数据
	}

	if(pmc_count > 0){
        ((LineMsg *)new_msg)->SetPmcAxisCount(pmc_count);
        ScPrintf("SetPmcAxisData count = %u", pmc_count);
	}
//	printf("line msg create, wait=%hhu\n", new_msg->IsNeedWaitMsg());

//	printf("createlinemsg, lineno = %lld\n", this->m_p_lexer_result->line_no);
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	//同行是否存在G53，即指定机械坐标
	if(this->m_b_has_g53){
		((LineMsg *)new_msg)->SetMachCoord(true);
	}

	m_p_parser_result->Append(new_msg);
    ProcessLastBlockRec(new_msg);

//    // 如果对旋转轴移动，需要插入清整数圈的msg
//    SCAxisConfig *axis_config = g_ptr_parm_manager->GetAxisConfig();
//    SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig();
//    for(int i = 0; i < chn_config->chn_axis_count; i++){
//        uint8_t mask = axis_mask & (0x01 << i);
//        if(!mask || (axis_config[i].axis_type != AXIS_ROTATE))
//            continue;
//        double pos = target.GetAxisValue(i);
//        if(pos >= 0 && pos < 360)   // 不超范围，不需要清
//            continue;
//        // 添加一条G200消息,对旋转轴清整数圈
//        RecordMsg *clr_msg = new ClearCirclePosMsg(2000, mask, 360*1000);
//        new_msg->SetLineNo(this->m_p_lexer_result->line_no);
//        m_p_parser_result->Append(clr_msg);
//        ProcessLastBlockRec(clr_msg);
//        ScPrintf("append ClearCirclePosMsg, msk = %u",mask);
//    }

//	printf("line msg : (%lf, %lf, %lf, %lf, %lf, %lf), %lld\n",target.x, target.y, target.z, target.a4,
//			target.a5, target.a6, new_msg->GetLineNo());
	return true;
}


/**
 * @brief 构造圆弧切削消息，并插入消息队列
 * @param gcode : 圆弧G代码，G02/G03
 * @return true--成功  false--失败
 */
bool Parser::CreateArcMsg(const int gcode){
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);
//	if(g_code == nullptr){
//		m_error_code = ERR_COMPILER_INTER;
//		return false;
//	}
    double i_number = 0, j_number = 0, r_number = 0;

	if(!m_b_f_code){//未指定F值
		printf("ERR_NO_F_DATA\n");
		m_error_code = ERR_NO_F_DATA;
		return false;
	}

    //TODO 检查参数
	DPointChn source = this->m_p_compiler_status->cur_pos;   //起点坐标
	DPointChn target, center;  //终点坐标，圆心坐标
	double radius = 0.0;  //半径
	int8_t major_flag = 1;  //优弧标志， 1--劣弧    -1--优弧
	int8_t circle_flag = 0;	//整圆标志， 0--圆弧    1--整圆
	int8_t dir_flag = -1;  //方向标志，-1:clockwise,1:anticlockwise
	uint32_t axis_mask = 0;

	uint8_t io = 0;
	double data = 0;

	if(gcode == G03_CMD)
		dir_flag = 1;

    uint8_t pmc_count = 0;
	//读取终点坐标
    if(!GetTargetPos(target, axis_mask, &pmc_count)){
		return false;   //产生错误，返回
	}

	//读取其它参数
	if(GetCodeData(R_DATA, radius)){//存在R参数，IJK和R同时存在时R优先，忽略IJK
		if((g_code->mask_dot & (0x01<<R_DATA)) == 0){
			radius /= 1000.;   //省略小数点则以um为单位
		}
		if(radius < 0 ) major_flag = -1; 	//优弧

		r_number = radius;

        radius = fabs(radius);

		//计算圆心坐标
        //printf("===== ARC SOURCE %lf %lf %lf\n",
        //        source.GetAxisValue(0), source.GetAxisValue(1), source.GetAxisValue(2));


        DPointChn target_pos = target;
        if (this->m_p_compiler_status->mode.gmode[3] == G91_CMD)
        {// llx add 解决增量模式跑圆弧报警无法自洽问题
            target_pos += source;
        }
        if(!CalArcCenter(source, target_pos, radius, major_flag*dir_flag, center)){
			return false; //产生错误，返回
		}

	}
	else{
		double i = 0.0, j = 0.0, k = 0.0;  //I/J/K的值,省略则默认为0
		bool has_data = false;  //标志是否存在IJK参数
		if(GetCodeData(I_DATA, i)){//读取I参数
			if((g_code->mask_dot & (0x01<<I_DATA)) == 0){
				i /= 1000.;   //省略小数点则以um为单位
			}
			has_data = true;
			i_number = i;
		}
		if(GetCodeData(J_DATA, j)){//读取I参数
			if((g_code->mask_dot & (0x01<<J_DATA)) == 0){
				j /= 1000.;   //省略小数点则以um为单位
			}
			has_data = true;
            j_number = j;
		}
		if(GetCodeData(K_DATA, k)){//读取I参数
			if((g_code->mask_dot & (0x01<<K_DATA)) == 0){
				k /= 1000.;   //省略小数点则以um为单位
			}
			has_data = true;
		}

		if(!has_data){//TODO 圆弧数据缺失，告警
			m_error_code = ERR_ARC_NO_DATA;
			return false;
		}


		//计算圆心坐标以及半径
		if(m_p_compiler_status->mode.gmode[2] == PLANE_XY){ //XY平面
			radius = sqrt(i * i + j * j);
		//	printf("i = %lf, j = %lf, radius = %lf\n", i, j, radius);

		}else if(m_p_compiler_status->mode.gmode[2] == PLANE_ZX){ //ZX平面
			radius = sqrt(i * i + k * k);

		}else if(m_p_compiler_status->mode.gmode[2] == PLANE_YZ){ //YZ平面
			radius = sqrt(j * j + k * k);
		}
		DPointChn vec(i,j,k);
		center = source + vec;  //圆心坐标

//		printf("cen[%lf, %lf, %lf], src[%lf, %lf, %lf], vec[%lf, %lf, %lf]\n", center.x, center.y, center.z, source.x,source.y, source.z,
//				vec.x, vec.y, vec.z);
//
//		printf("tar[%lf, %lf, %lf]\n", target.x, target.y, target.z);

        DPointChn target_pos = target;

        if(source != target_pos){
			DPlane cen = Point2Plane(center, m_p_compiler_status->mode.gmode[2]);
            DPlane tar = Point2Plane(target_pos, m_p_compiler_status->mode.gmode[2]);
            double dr2 = GetVectLength(cen, tar);

			if(fabs(dr2-radius) > 2e-3){  //起点和终点到圆心距离差超过2um则告警
				printf("cal arc error, %lf, %lf, %d\n", dr2, radius, m_p_compiler_status->mode.gmode[2]);
				m_error_code = ERR_ARC_INVALID_DATA;
				return false;
			}

			//判断优劣弧
			DPlane src = Point2Plane(source, m_p_compiler_status->mode.gmode[2]);
			double angle_start = GetVectAngle(src, cen);
			double angle_end = GetVectAngle(tar, cen);
			double angle = (angle_end - angle_start) * dir_flag;
			if(angle < 0)
				angle += 2*M_PI;
			if(angle > M_PI)
				major_flag = -1;   //大于180度，优弧
		}
		else{
			//判断是否整圆
			circle_flag = 1;    //IJK编程，起点和终点重合则为整圆
			major_flag = -1;    //优弧
		}

	}

	RecordMsg *new_msg = new ArcMsg(gcode, source, target, center, radius, m_p_compiler_status->mode.f_mode, axis_mask,
			dir_flag, major_flag, circle_flag);

	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

    ArcMsg * msg = (ArcMsg *) new_msg;
    if(pmc_count > 0){
        msg->SetPmcAxisCount(pmc_count);
    }

    msg->setI(i_number);
    msg->setJ(j_number);
    msg->setR(r_number);

	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	((ArcMsg *)new_msg)->SetPlaneMode(m_p_compiler_status->mode.gmode[2]);   //设置平面模态

	//同行是否存在G53，即指定机械坐标
	if(this->m_b_has_g53){
		((ArcMsg *)new_msg)->SetMachCoord(true);
	}

	if(GetCodeData(P_DATA, data)){   //IO数据
		io = static_cast<uint8_t>(data);
		((ArcMsg *)new_msg)->SetIoData(io);   //设置IO数据
	}

	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	this->m_p_compiler_status->mode.gmode[1] = gcode;
	this->m_p_compiler_status->mode.gmode[9] = G80_CMD;  //自动取消循环指令
 //   this->m_p_compiler_status->cur_pos = target; //更新编译当前位置
	return true;
}

/**
 * @brief 构造极坐标插补相关消息
 * @param gcode : 极坐标插补及磨削相关G代码，G12.1/G12.2/G13.1
 * @return true--成功  false--失败
 */
bool Parser::CreatePolarIntpMsg(const int gcode){
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	int p_data = -1;
	int r_data = -1;
	int l_data = -1;
	int q_data = -1;
	int x_data = 0;
	int y_data = 0;
	int i_data = 0;
	int j_data = 0;


#ifdef USES_GRIND_MACHINE
	if(gcode == G12_2_CMD || gcode == G12_3_CMD){
		double data;  //临时数据


		//读取参数
		if(GetCodeData(P_DATA, data)){
			p_data = data;			//平滑时间  单位：ms
		}
		if(GetCodeData(R_DATA, data)){
			r_data = data;    //砂轮半径索引
		}
		if(GetCodeData(L_DATA, data)){
			l_data = data;    //暂未使用
		}
		if(GetCodeData(Q_DATA, data)){
			q_data = data;    //暂未使用
		}

		if(gcode == G12_3_CMD){  //G12.3指令还有X、Y、I、J参数
			if(GetCodeData(X_DATA, data)){
				if((g_code->mask_dot & (0x01<<X_DATA)) == 0){
					x_data = data;   //省略小数点则以um为单位
				}
				x_data = data*1000;			//磨削起点X轴坐标，  单位：um
			}
			if(GetCodeData(Y_DATA, data)){
				y_data = data*1000;    //磨削起点Y轴坐标，  单位：um
			}
			if(GetCodeData(I_DATA, data)){
				i_data = data*1000000;    //磨削方向单位矢量，放大1000000倍
			}
			if(GetCodeData(J_DATA, data)){
				j_data = data*1000000;    //磨削方向单位矢量，放大1000000倍
			}

		}
	}
#endif
	RecordMsg *new_msg = new PolarIntpMsg(gcode, p_data, r_data, l_data, q_data, x_data, y_data, i_data, j_data);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	this->m_p_compiler_status->mode.gmode[21] = gcode;
	return true;
}

/**
 * @brief 构造旋转轴清整数圈位置消息
 * @return true--成功  false--失败
 */
bool Parser::CreateClearCirclePosMsg(){
	int p_data = -1;
	uint32_t mask = 0;
	double data;  //临时数据

	int gcode = 2000;  //G200

	//读取参数
	if(GetCodeData(P_DATA, data)){
		data *= 1000;    //模，单位mm，乘以1000，转换为um单位
		p_data = data;
	}else{
		m_error_code = ERR_LACK_OF_PARAM;   //缺少必要参数
		return false;
	}

	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
	SCAxisConfig *axis_config = g_ptr_parm_manager->GetAxisConfig();
	
	uint8_t phy_axis = 0;
	for(int i = 0; i < chn_config->chn_axis_count; i++){
		phy_axis = g_ptr_chn_engine->GetChnAxistoPhyAixs(m_n_channel_index,i);
		if(phy_axis == 0xFF)    //未配置
			continue;

		if(axis_config[phy_axis].axis_type != AXIS_ROTATE)   //非旋转轴
			continue;

//		printf("i=%d, phy_axis=%hhu, name_idx=%hhu\n", i, phy_axis, chn_config->chn_axis_name[i]);

		switch(chn_config->chn_axis_name[i]){
		case 0:  //X
			if(GetCodeData(X_DATA, data)){
				mask |= (0x01<<phy_axis);
			}
			break;

		case 1:	 //Y
			if(GetCodeData(Y_DATA, data)){
				mask |= (0x01<<phy_axis);
			}
			break;

		case 2:  //Z
			if(GetCodeData(Z_DATA, data)){
				mask |= (0x01<<phy_axis);
			}
			break;

		case 3:  //A
			if(GetCodeData(A_DATA, data)){
				mask |= (0x01<<phy_axis);
			}
			break;

		case 4:  //B
			if(GetCodeData(B_DATA, data)){
				mask |= (0x01<<phy_axis);
			}
			break;

		case 5:		//C
			if(GetCodeData(C_DATA, data)){
				mask |= (0x01<<phy_axis);
			}
			break;

		case 6:		//U
			if(GetCodeData(U_DATA, data)){
				mask |= (0x01<<phy_axis);
			}
			break;

		case 7:		//V
			if(GetCodeData(V_DATA, data)){
				mask |= (0x01<<phy_axis);
			}
			break;

		case 8:		//W
			if(GetCodeData(W_DATA, data)){
				mask |= (0x01<<phy_axis);
			}
			break;

		default:
			break;
		}

	}

	printf("create clear pos message:mask=0x%x, mode=%d um\n", mask, p_data);

	RecordMsg *new_msg = new ClearCirclePosMsg(gcode, mask, p_data);   //G200
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	this->m_p_compiler_status->mode.gmode[39] = gcode;
	return true;
}

/**
 * @brief 构造延时等待消息
 * @return true--成功  false--失败
 */
bool Parser::CreateTimeWaitMsg(){
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	uint32_t time = 0;
	double data = 0;

	// 更新需求:  P指定ms  与小数点无关  X 支持小数点编程
	if(GetCodeData(X_DATA, data)){
		//time = data * 1000;
		if((g_code->mask_dot & (0x01<<X_DATA)) == 0){  //省略小数点则输入值单位为ms
			time = data;
		}else
			time = data*1000;   //带小数点，输入值单位为s
	}else if(GetCodeData(P_DATA, data)){
		time = data;
		/*if((g_code->mask_dot & (0x01<<P_DATA)) == 0){  //省略小数点则输入值单位为ms
			time = data;
		}else
			time = data*1000;   //带小数点，输入值单位为s*/
	}

	RecordMsg *new_msg = new TimeWaitMsg(time);   //G04
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	this->m_p_compiler_status->mode.gmode[0] = G04_CMD;

//	printf("create time wait msg, time = %u\n", time);

	return true;
}

/**
 * @brief 构造参考点返回消息
 * @param gcode : G代码
 * @return true--成功  false--失败
 */
bool Parser::CreateRefReturnMsg(const int gcode){
//	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	uint32_t mask = 0;
	DPointChn middle;

	this->GetTargetPos(middle, mask);

	RefReturnMsg *new_msg = new RefReturnMsg(gcode, mask, middle);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号

	double data;

	if(GetCodeData(P_DATA, data)){
		new_msg->ref_id = (int)data;
	}

	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append((RecordMsg *)new_msg);

	ProcessLastBlockRec((RecordMsg *)new_msg);

	this->m_p_compiler_status->mode.gmode[0] = gcode;

//	printf("create Ref return msg, axis_mask = 0x%x\n", mask);

	return true;
}


/**
 * @brief 构造运行中跳转检测  G31
 * @param gcode : G代码  
 * @return true--成功  false--失败
 */
bool Parser::CreateSkipRunMsg(const int gcode){
//	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	DPointChn source = this->m_p_compiler_status->cur_pos;   //起点
	DPointChn target = source;	//终点
	uint32_t axis_mask = 0;



	if(!GetTargetPos(target, axis_mask))
		return false;

	if(!m_b_f_code){//未指定F值
		printf("ERR_NO_F_DATA\n");
		m_error_code = ERR_NO_F_DATA;
		return false;
	}
	

	RecordMsg *new_msg = new SkipMsg(source, target, m_p_compiler_status->mode.f_mode, axis_mask);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	//同行是否存在G53，即指定机械坐标
	if(this->m_b_has_g53){
		((SkipMsg *)new_msg)->SetMachCoord(true);
	}

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	this->m_p_compiler_status->mode.gmode[0] = gcode;

//	printf("create Ref return msg, axis_mask = 0x%x\n", mask);

	return true;
}

/**
 * @brief 构造宏程序调用指令消息，并插入消息队列
 * @return true--成功  false--失败
 */
bool Parser::CreateMacroProgCallMsg(){

	int pcode = 0;		//宏程序名称
	int lcode = 1;  	//重复调用次数，默认一次

	double value = 0.0;   //临时变量
	if(!GetCodeData(P_DATA, value)){
		if(m_error_code == ERR_NONE)
			m_error_code = ERR_NO_SUB_PROG_P;  //没有指定子程序号
		return false;
	}else{
		pcode = static_cast<int>(value);
	}

	if(GetCodeData(L_DATA, value)){
		lcode = static_cast<int>(value);
	}

	//获取参数
	double *param = nullptr;
	uint8_t pc = 0;  //参数个数
	uint32_t pm = 0;  //参数mask
	this->GetParaData(&param, pc, pm);


	RecordMsg *new_msg = new MacroProgCallMsg(pcode, lcode, param, pc, pm);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);
	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief 构造自动对刀指令消息，并插入消息队列
 * @return true--成功  false--失败
 */
bool Parser::CreateAutoToolMeasureMsg(){
	printf("Parser::CreateAutoToolMeasureMsg\n");
	int hcode = 0;		//写入的H值
	int lcode = 1;  	//重复对刀次数，默认一次

	DPointChn target = this->m_p_compiler_status->cur_pos;   //终点
	uint32_t axis_mask = 0;  //轴mask


	double value = 0.0;   //临时变量
	if(!GetCodeData(H_DATA, value)){
		if(m_error_code == ERR_NONE){
			//默认使用当前H值
			hcode = static_cast<int>(m_p_compiler_status->mode.h_mode);
		}
	}else{
		hcode = static_cast<int>(value);
	}

	if(GetCodeData(L_DATA, value)){
		lcode = static_cast<int>(value);
	}

	if(!GetTargetPos(target, axis_mask)){
		printf("failed to get target pos\n");
		return false;
	}


	RecordMsg *new_msg = new AutoToolMeasureMsg(target, axis_mask, hcode, lcode);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);
	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	printf("exit Parser::CreateAutoToolMeasureMsg\n");
	return true;
}



#ifdef USES_SPEED_TORQUE_CTRL	
/**
 * @brief 构造速度控制消息
 * @param gcode : G指令值
 * @return
 */
bool Parser::CreateSpeedCtrlMsg(const int gcode){
	uint32_t mask = 0;

	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
	double speed[chn_config->chn_axis_count];   //目标速度
	

	printf("--IN--Parser:CreateSpeedCtrlMsg:gcode=%d ",gcode);
	uint8_t count = this->GetTargetSpeed(speed, mask, chn_config->chn_axis_count);
	if(count == 0xFF)
		return false;

	printf("CreateSpeedCtrlMsg:count=%hhu, mask=0x%x, speed=[%lf, %lf, %lf]\n", count, mask, speed[0], speed[1], speed[2]);
	RecordMsg *new_msg = new SpeedCtrlMsg(gcode, speed, count, mask);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	this->m_p_compiler_status->mode.gmode[0] = gcode;

	return true;
}

/**
 * @brief 构造力矩控制消息
 * @param gcode : G指令值
 * @return
 */
bool Parser::CreateTorqueCtrlMsg(const int gcode){
	uint32_t mask = 0;
    double data = 0;
	int16_t lmt;
	uint32_t time = 0;  //超时时间

	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
	double torque[chn_config->chn_axis_count];   //目标力矩

    printf("--IN--Parser:CreateTorqueCtrlMsg:gcode=%d ",gcode);

    LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);
	
	uint8_t count = this->GetTargetTorque(torque, mask, chn_config->chn_axis_count);
	if(count == 0xFF)
		return false;

	if(GetCodeData(P_DATA, data)){
		if((g_code->mask_dot & (0x01<<P_DATA)) == 0){  //省略小数点则输入值单位为ms
			time = data;
		}else
			time = data*1000;   //带小数点，输入值单位为s

	//	printf("torque msg: q=%u\n", time);
	}

	RecordMsg *new_msg = new TorqueCtrlMsg(gcode, torque, count, mask, time);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	data = 0;
	if(GetCodeData(S_DATA, data)){   //IO数据
		lmt = static_cast<int16_t>(data);
		((TorqueCtrlMsg*)new_msg)->SetSpeedLmtValue(lmt);   //设置速度限制值
	}
	
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	this->m_p_compiler_status->mode.gmode[0] = gcode;

    printf("--OUT--CreateTorqueCtrlMsg success:mask=0x%x val=%f  %f  %f  %f ", mask, torque[0],torque[1], torque[2], torque[3]);

	return true;
}
#endif

/**
 * @brief 构造主轴转速检测消息(G25/G26)，并插入消息队列
 * @return true--成功  false--失败
 */
bool Parser::CreateSpindleCheckMsg(){
//	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	int p = 0, q = 0, r = 0, i = 0;
	//TODO 检查参数


	RecordMsg *new_msg = new SpindleCheckMsg(m_mode_code[19], p, q, r, i);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

bool Parser::CreateInputMsg(){
	InputMsg * new_msg = new InputMsg();

	GetCodeData(L_DATA, new_msg->LData);
	GetCodeData(P_DATA, new_msg->PData);
	GetCodeData(H_DATA, new_msg->HData);
	GetCodeData(D_DATA, new_msg->DData);
	GetCodeData(R_DATA, new_msg->RData);
	GetCodeData(X_DATA, new_msg->XData);
	GetCodeData(Y_DATA, new_msg->YData);
	GetCodeData(Z_DATA, new_msg->ZData);
	GetCodeData(Q_DATA, new_msg->QData);

	printf("create input msg: l: %d -- p: %d -- r: %d\n", new_msg->LData,new_msg->PData,new_msg->RData);

	new_msg->SetLineNo(this->m_p_lexer_result->line_no);
	m_p_parser_result->Append(new_msg);
	ProcessLastBlockRec(new_msg);
	return true;
}

bool Parser::CreateExactStopMsg(){
	DPointChn source = this->m_p_compiler_status->cur_pos;   //起点
	DPointChn target = source;	//终点

	RecordMsg *new_msg = nullptr;
	uint32_t axis_mask = 0;
	uint8_t io = 0;
	double data = 0;

	uint8_t pmc_count = 0;

	if(!GetTargetPos(target, axis_mask, &pmc_count))
		return false;

	new_msg = new ExactStopMsg(source, target, axis_mask);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号

	m_p_parser_result->Append(new_msg);
	ProcessLastBlockRec(new_msg);
	return true;
}

/**
 * @brief 是否有指定地址字数据
 * @param addr : 地址字
 * @return true--有   false--没有
 */
bool Parser::HasCodeData(DataAddr addr){
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	if(g_code->mask_value & (0x01<<addr)) //是否存在addr参数
		return true;

	return false;
}

/**
 * @brief 获取指定地址字的数据
 * @param addr : 地址字
 * @param [out]data : 返回数据
 * @return true--找到数据    false--没有数据
 */
bool Parser::GetCodeData(DataAddr addr, double &data){

	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	uint32_t mask = (0x01<<addr);
	MacroVarValue res;

    if(g_code->mask_value & mask){ //是否存在addr参数
		if(g_code->mask_macro & mask){ //表达式
			int exp_index = static_cast<int>(g_code->value[addr]);
			while(!this->GetExpressionResult(g_code->macro_expression[exp_index], res)){
				if(this->m_error_code != ERR_NONE)
					return false;
				usleep(10000);  //休眠10ms，等待MC运行到位
			}

			if(!res.init){ //空值，则忽略此地址字
				return false;
			}else
				data = res.value;
		}
		else{ //常数值
			data = g_code->value[addr];
		}
		g_code->mask_value &= (~mask);   //使用过后就复位此参数mask
	}else{//无此地址字数据
        //printf("Get %c data: no data\n", 'A'+addr);
		//data = g_code->value[addr];
		return false;
	}

	return true;
}

/**
 * @brief 获取自变量参数,除了G/L/N/O/P外其余字母均可做自变量，最多21个
 * @modify zk 解除P变量限制
 * @param param[out] : 返回自变量值，动态分配内存，使用后需自行释放
 * @param pc[out] ： 返回自变量个数
 * @param mask[out] ： 返回自变量mask
 */
void Parser::GetParaData(double **param, uint8_t &pc, uint32_t &mask){
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);
	pc = 0;
	mask = 0;
	if(param == nullptr)
		return;

	int dd = A_DATA;
	uint32_t tm = g_code->mask_value;
	while(tm != 0){
		if(tm & 0x01){
			//先统计自变量数量  @modify zk 解除P变量限制
			if(dd != G_DATA && dd!= L_DATA && dd != N_DATA && dd != O_DATA){
				pc++;
				mask |= (0x01<<dd);
			}

		}
		tm = tm>>1;
		dd++;
	}

	if(pc == 0)  //没有自变量
		return;

	printf("get macro sub param : count = %hhu, mask = 0x%x\n", pc, mask);

	*param = new double[pc];
	tm = mask;
	dd = A_DATA;
	double *pp = *param;
	while(tm != 0){
		if(tm & 0x01){
			this->GetCodeData((DataAddr)dd, *pp);
			pp++;
		}
		tm = tm>>1;
		dd++;
	}
}

/**
 * @brief 用轴扩展名称获取指令数据
 * @param name : 轴名称字符
 * @param name_ex : 轴扩展名序号
 * @param data[out] : 返回的数据
 * @return true--找到数据    false--没有数据
 */
bool Parser::GetAxisExData(uint8_t name, uint8_t name_ex, double &data){
	if(name > 8 || (name_ex > 16))
		return false;

	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);
	uint16_t mask = (0x01<<(name_ex-1));
	MacroVarValue res;

	if(g_code->mask_pos[name] & mask){
//		printf("Parser::GetAxisExData, mask_pos=0x%x, mask=0x%x\n", g_code->mask_pos[name], mask);
		if(g_code->mask_pos_macro[name] & mask){ //表达式
			int exp_index = static_cast<int>(g_code->pos_value[name][name_ex-1]);
			while(!this->GetExpressionResult(g_code->macro_expression[exp_index], res)){
				if(this->m_error_code != ERR_NONE)
					return false;
				usleep(10000);  //休眠10ms，等待MC运行到位
			}

			if(!res.init){ //空值，则忽略此地址字
				return false;
			}else
				data = res.value;
		}
		else{ //常数值
			data = g_code->pos_value[name][name_ex-1];
		}
		g_code->mask_pos[name] &= (~mask);   //使用过后就复位此参数mask
	}else
		return false;

	return true;
}

/**
 * @brief 获取目标点坐标,只对插补NC轴
 * @param [out]target : 返回目标点坐标
 * @param [out]axis_mask: 返回轴移动的mask，标志哪些轴会移动
 * @param [out]pmc_count: PMC轴个数
 * @return  true--成功    false--失败
 */
bool Parser::GetTargetPos(DPointChn &target, uint32_t &axis_mask, uint8_t *count){
	DataAddr addr = A_DATA;
    uint8_t i = 0;
	double data = 0.0;
	double *p_target_pos = target.m_df_point;
	double *p_source_pos = m_p_compiler_status->cur_pos.m_df_point;
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
    uint8_t pmc_axis_count = 0;
	uint8_t axis_name_idx = 0;
	uint32_t tm = 0x01;
	bool has_valid_world = false;

	for(i = 0; i < chn_config->chn_axis_count; i++){
//		addr = static_cast<DataAddr>(m_axis_name[i]-'A');

		axis_name_idx = chn_config->chn_axis_name[i];
		if(m_b_axis_name_ex && chn_config->chn_axis_name_ex[i] > 0){  //有扩展下标
			has_valid_world = true;
			if(this->GetAxisExData(axis_name_idx, this->m_axis_name_ex[i], data)){  //有数据
				if(((g_code->mask_dot_ex[axis_name_idx] & (0x01<<(chn_config->chn_axis_name_ex[i]-1))) == 0) &&
						((g_code->mask_pos_macro[axis_name_idx] & (0x01<<(chn_config->chn_axis_name_ex[i]-1))) == 0)){  //没有小数点并且非宏表达式
					data /= 1000.;   //省略小数点则以um为单位
				}
				*p_target_pos = data;
//				if((this->m_mask_pmc_axis & tm) == 0 && this->m_p_compiler_status->mode.gmode[3] == G91_CMD){  //插补轴且增量编程模式
//					*p_target_pos += *p_source_pos;
//				}
				axis_mask |= tm;  //设置轴mask
                if(this->m_mask_pmc_axis & tm)
                {
                    pmc_axis_count++;   //计数PMC轴
                }

			}else if(m_error_code == ERR_NONE){ //无此参数,使用起点相应轴位置
				*p_target_pos = *p_source_pos;
			}else{//异常
				printf("get target pos failed\n");
				return false;
			}
		}else{//无扩展下标
			 addr = static_cast<DataAddr>(m_axis_name[i]-'A');
			 if(GetCodeData(addr, data)){//有此参数，读取成功
				 has_valid_world = true;
				 if(((g_code->mask_dot & (0x01<<addr)) == 0) &&
						((g_code->mask_macro & (0x01<<addr)) == 0)){  //没有小数点并且非宏表达式
					data /= 1000.;   //省略小数点则以um为单位
				}
				*p_target_pos = data;
//				if((this->m_mask_pmc_axis & tm) == 0 && this->m_p_compiler_status->mode.gmode[3] == G91_CMD){  //插补轴且增量编程模式
//					*p_target_pos += *p_source_pos;
//				}

				axis_mask |= tm;  //设置轴mask
                if(this->m_mask_pmc_axis & tm) {
                    pmc_axis_count++;   //计数PMC轴
                }

			 }else if(m_error_code == ERR_NONE){ //无此参数,使用起点相应轴位置
				*p_target_pos = *p_source_pos;
			 }else{//异常
				printf("get target pos failed\n");
				return false;
			 }
		}
		tm = tm<<1;
		p_target_pos++;
		p_source_pos++;
	}

	//@ add zk G00 G01没有任何有效参数
	//--与其他系统比较 决定支持 G00 G01 不带任何 xyz参数
	/*
	if(!has_valid_world){
		m_error_code = ERR_LACK_OF_PARAM;   //缺少必要参数
		return false;
	}*/

    if(count)
        *count = pmc_axis_count;

	return true;
}

/**
 * @brief 获取PMC轴目标点坐标
 * @param target[out] : 返回PMC轴的目标数据
 * @param axis_mask[out] ： 返回轴移动的mask，标志哪些轴会移动
 * @param inc_mode[out] : 返回的轴位置是否增量模式
 * @param max[in] : target数组的个数
 * @return 返回有目标数据的PMC轴的数量, 返回0xFF表示出错
 */
//uint8_t Parser::GetTargetPosEx(double *target, uint32_t &axis_mask, bool &inc_mode,  uint8_t max){
//	uint8_t axis_count = 0;
//    uint8_t i = 0;
//	double data = 0.0;
//	DataAddr addr = A_DATA;
//
//	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);
//
//	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
//	uint8_t axis_name_idx = 0;
//	for(i = 0; i < chn_config->chn_axis_count && axis_count < max; i++){
//		if((this->m_mask_pmc_axis & (0x01<<i)) == 0)
//			continue;   //跳过插补轴
//		addr = static_cast<DataAddr>(m_axis_name[i]-'A');
//		axis_name_idx = chn_config->chn_axis_name[i];
//		if(m_b_axis_name_ex && chn_config->chn_axis_name_ex[i] > 0){  //有扩展下标
//			if(this->GetAxisExData(axis_name_idx, chn_config->chn_axis_name_ex[i], data)){  //有数据
//				if(((g_code->mask_dot_ex[axis_name_idx] & (0x01<<(chn_config->chn_axis_name_ex[i]-1))) == 0) &&
//						((g_code->mask_pos_macro[axis_name_idx] & (0x01<<(chn_config->chn_axis_name_ex[i]-1))) == 0)){  //没有小数点并且非宏表达式
//					data /= 1000.;   //省略小数点则以um为单位
//				}
//				*target = data;
//
//				axis_mask |= (0x01<<i);  //设置轴mask
//
//				axis_count++;
//				target++;
//				printf("GetTargetPosEx 111, %lf, 0x%x\n", data, axis_mask);
//			}else if(m_error_code == ERR_NONE){ //无此参数
//	//			printf("GetTargetPosEx 222, %lf, 0x%x\n", data, axis_mask);
//			}else{//异常
//				printf("get pmc target pos failed\n");
//				return 0xFF;
//			}
//		}else{//无扩展下标
//			 addr = static_cast<DataAddr>(m_axis_name[i]-'A');
//			 if(GetCodeData(addr, data)){//有此参数，读取成功
//				if(((g_code->mask_dot & (0x01<<addr)) == 0) &&
//						((g_code->mask_macro & (0x01<<addr)) == 0)){  //没有小数点并且非宏表达式
//					data /= 1000.;   //省略小数点则以um为单位
//				}
//				*target = data;
//
//				axis_mask |= (0x01<<i);  //设置轴mask
//				axis_count++;
//				target++;
//			 }else if(m_error_code == ERR_NONE){ //无此参数
//
//			 }else{//异常
//				printf("get pmc target pos failed\n");
//				return 0xFF;
//			 }
//		}
//
//	}
//
//
//	if(this->m_p_compiler_status->mode.gmode[3] == G91_CMD)
//		inc_mode = true;
//	else
//		inc_mode = false;
//
//	printf("GetTargetPosEx, count = %hhu, mask = 0x%x, inc = %hhu\n", axis_count, axis_mask, inc_mode);
//	return axis_count;
//}


#ifdef USES_SPEED_TORQUE_CTRL	
/**
 * @brief 获取目标速度
 * @param [out]target : 返回目标点速度值
 * @param [out]axis_mask: 返回轴移动的mask，标志哪些轴会移动
 * @return  返回有目标数据的PMC轴的数量, 返回0xFF表示出错
 */
uint8_t Parser::GetTargetSpeed(double *target, uint32_t &axis_mask, uint8_t max){
	DataAddr addr = A_DATA;
	int i = 0;
	double data = 0.0;
	uint8_t axis_count = 0;


	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
	for(i = 0; i < chn_config->chn_axis_count && axis_count < max; i++){

	//	addr = static_cast<DataAddr>(m_axis_name[i]-'A');
		if(m_b_axis_name_ex && chn_config->chn_axis_name_ex[i] > 0){  //有扩展下标
			if(this->GetAxisExData(chn_config->chn_axis_name[i], chn_config->chn_axis_name_ex[i], data)){  //有数据

				*target = data;

				axis_mask |= (0x01<<i);  //设置轴mask
			}else if(m_error_code == ERR_NONE){ //无此参数

			}else{//异常
				printf("get target speed failed\n");
				return 0xFF;
			}
		}else{//无扩展下标
			 addr = static_cast<DataAddr>(m_axis_name[i]-'A');
			 if(GetCodeData(addr, data)){//有此参数，读取成功

				*target = data;

				axis_mask |= (0x01<<i);  //设置轴mask
			 }else if(m_error_code == ERR_NONE){ //无此参数

			 }else{//异常
				printf("get target speed failed\n");
				return 0xFF;
			 }
		}
		axis_count++;
		target++;
	}

	return axis_count;
}


/**
 * @brief 获取目标力矩
 * @param [out]target : 返回目标力矩值
 * @param [out]axis_mask: 返回轴移动的mask，标志哪些轴会移动
 * @param max : target数组最大个数
 * @return  返回有目标数据的PMC轴的数量, 返回0xFF表示出错
 */
uint8_t Parser::GetTargetTorque(double *target, uint32_t &axis_mask, uint8_t max){
	DataAddr addr = A_DATA;
	int i = 0;
	double data = 0.0;
	uint8_t axis_count = 0;

	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
	for(i = 0; i < chn_config->chn_axis_count && axis_count < max; i++){

	//	addr = static_cast<DataAddr>(m_axis_name[i]-'A');
		if(m_b_axis_name_ex && chn_config->chn_axis_name_ex[i] > 0){  //有扩展下标
			if(this->GetAxisExData(chn_config->chn_axis_name[i], chn_config->chn_axis_name_ex[i], data)){  //有数据

				*target = data;

				axis_mask |= (0x01<<i);  //设置轴mask
			}else if(m_error_code == ERR_NONE){ //无此参数

			}else{//异常
				printf("get target torque failed\n");
				return 0xFF;
			}
		}else{//无扩展下标
			 addr = static_cast<DataAddr>(m_axis_name[i]-'A');
			 if(GetCodeData(addr, data)){//有此参数，读取成功

				*target = data;

				axis_mask |= (0x01<<i);  //设置轴mask
			 }else if(m_error_code == ERR_NONE){ //无此参数

			 }else{//异常
				printf("get target torque failed\n");
				return 0xFF;
			 }
		}
		axis_count++;
		target++;
	}

	return axis_count;
}

#endif

/**
 * @brief 计算圆弧中心坐标
 * @param src : 圆弧起点坐标
 * @param tar ：圆弧终点坐标
 * @param radius ：圆弧半径
 * @param flag ： -1表示顺时针，1表示逆时针
 * @param [out]center : 输出计算出的圆弧中心坐标
 * @return true--成功   false--失败
 */
bool Parser::CalArcCenter(const DPointChn &src, const DPointChn &tar, const double &radius, const int8_t flag, DPointChn &cen){
	DPlane source = Point2Plane(src, this->m_p_compiler_status->mode.gmode[2]);
	DPlane target = Point2Plane(tar, this->m_p_compiler_status->mode.gmode[2]);


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
		m_error_code = ERR_ARC_INVALID_DATA;
		return false;
	}
	// @test zk
	//printf("src(%lf, %lf), tar(%lf, %lf), L/2= %lf, radius=%lf\n", source.x, source.y, target.x, target.y, dr, radius);
	double  angle = GetVectAngle(target, source) + flag * M_PI_2;
	printf("angle = %lf\n", angle);
	dr = sqrt(dr2);
	printf("dr = %lf, dr2=%lf\n", dr, dr2);
	DPlane  move(dr * cos(angle), dr * sin(angle) );
	DPlane center = mid + move;
	//printf("mid(%lf, %lf), move(%lf, %lf)\n", mid.x, mid.y, move.x, move.y);
	cen = src;
	Plane2Point(center, cen, this->m_p_compiler_status->mode.gmode[2]);
	//printf("cal arc center: (%lf, %lf, %lf)\n", cen.m_df_point[0], cen.m_df_point[1], cen.m_df_point[2]);

	return true;
}

/**
 * @brief 找到分块结束移动指令，并设置分块结束标志
 * @param new_msg : 新的生成的指令消息
 */
void Parser::ProcessLastBlockRec(RecordMsg *new_msg){
	if(new_msg->IsNeedWaitMsg()){
		if(*m_pp_last_move_rec != nullptr){
			(*this->m_pp_last_move_rec)->SetFlag(FLAG_BLOCK_OVER, true);
			*m_pp_last_move_rec = nullptr;
		}
	}
	if(new_msg->IsMoveMsg() && !new_msg->CheckFlag(FLAG_BLOCK_OVER)){
		if(!new_msg->CheckFlag(FLAG_JUMP) || !m_p_channel_control->CheckFuncState(FS_BLOCK_SKIP))
			*m_pp_last_move_rec = new_msg;
	}
}

/**
 * @brief 构造宏指令消息，并插入消息队列
 * @param macro : 词法分析器输出的宏指令
 * @return
 */
bool Parser::CreateMacroMsg(LexerMacroCmd *macro){
	//
	RecordMsg *new_msg = new MacroCmdMsg(macro);
	if(new_msg == nullptr){
		//TODO 内存分配失败，告警
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

//	printf("create macro message   %llu\n", this->m_p_lexer_result->line_no);
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //设置当前行号
	(static_cast<MacroCmdMsg *>(new_msg))->SetOffset(this->m_p_lexer_result->offset);

	if(macro->cmd == MACRO_CMD_IF){
		new_msg->SetFlag(FLAG_WAIT_MOVE_OVER, true);
	}


	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}
