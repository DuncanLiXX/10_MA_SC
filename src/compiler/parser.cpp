/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file parser.cpp
 *@author gonghao
 *@date 2020/04/10
 *@brief ��ͷ�ļ�ΪG�����﷨���������ʵ��
 *@version
 */

#include <parser.h>
#include "channel_engine.h"
#include "channel_control.h"
#include "variable.h"
#include "parm_manager.h"
#include "spindle_control.h"

const int kSeparatedMCodeArray[] = {0,1,2,30,98,99,198};   //����������M����ͬ�е�M����
const int kSeparatedMCodeCount = sizeof(kSeparatedMCodeArray)/sizeof(int);   //����ͬ�е�M�������

/**
 * @brief �﷨���������캯��
 * @param lexer_result����Ŵʷ�������������ݽṹָ��
 */
Parser::Parser(uint8_t chn_index, LexerResult *lexer_result, CompileRecList *parser_result, CompileStatusCollect *compiler_status) {
	// TODO Auto-generated constructor stub
	this->m_n_channel_index = chn_index;
	this->m_p_lexer_result = lexer_result;
	this->m_p_parser_result = parser_result;
	this->m_p_compiler_status = compiler_status;

	this->m_p_channel_control = ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index);
//#ifdef USES_WOOD_MACHINE
	this->m_b_multi_mcode = true;   //����ľ��������M���빦��Ĭ�ϴ�
//#else
//	this->m_b_multi_mcode = false;  //��M����Ĭ��Ϊ�ر�״̬
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
 * @brief ��������
 */
Parser::~Parser() {
	// TODO Auto-generated destructor stub

}


/**
 * @brief ��λ�﷨��������״̬
 */
void Parser::Reset(){
	m_error_code = ERR_NONE;
	m_b_f_code = false;   //Ĭ��Fֵδָ��
	this->m_b_call_macro_prog = false;
//	this->m_p_compiler_status->mode.move_mode = 1;   //Ĭ����01��ģ̬
	this->m_p_compiler_status->mode.Reset();
	this->m_mode_mask = GMODE_NONE;
	this->m_b_has_g53 = false;
}

/**
 * @brief ˢ��ͨ�������ƻ���
 * @param name �����������������
 */
void Parser::RefreshAxisName(){
	memset(m_axis_name, 0x00, kAxisNameBufSize);
	m_n_pmc_axis_count = 0;
	uint8_t phy_axis = 0;
//	char names[10] = "XYZABCUVW";
	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
	SCAxisConfig *axis_config = g_ptr_parm_manager->GetAxisConfig();    //����������
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
				m_mask_pmc_axis |= (0x01<<i);   //���pmc��
				m_n_pmc_axis_count++;
			}
		}
	}
//	printf("\n");
}

/**
 * @brief ������������չ�±�ʹ��
 * @param flag : false--�ر�    true--��
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
 * @brief �Դʷ�������������﷨����
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::Compile(){
	bool res = true;
	if(m_p_lexer_result == nullptr){
		CreateError(ERR_COMPILER_INTER, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	if(this->m_p_lexer_result->error_code != ERR_NONE){ //�ڴ����ɴʷ������Ĵ�����Ϣ
		this->CreateErrorMsg(m_p_lexer_result->error_code);
		return true;
	}
	else if(m_p_lexer_result->macro_flag){//�����ָ��
		res = CompileMacro();
	}
	else{//����G/M/S/Tָ��
		res = CompileGCode();
	}

	if(m_error_code != ERR_NONE){//�����������
		//TODO ����ERRORMSG������Ϣ����
		printf("parser: create error message!\n");
		this->CreateErrorMsg(m_error_code);
		return true;
	}

	return res;
}

/**
 * @brief �����ָ��
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CompileMacro(){
	bool res = true;
	LexerMacroCmd *macro_cmd = &(m_p_lexer_result->nc_code.macro_cmd);

	res = this->CreateMacroMsg(macro_cmd);

	//���������һ��ָ����FLAG_LAST_REC��־
	if(res){
		ListNode<RecordMsg *> *tail = this->m_p_parser_result->TailNode();
		if(tail != nullptr){
			RecordMsg *msg = tail->data;
			if(msg != nullptr){
				msg->SetFlag(FLAG_LAST_REC, true);   //ͬ�����һ��ָ��
				if(msg->GetMsgType() != LOOP_MSG){
					msg->SetFlag(FLAG_STEP, true);    //������ͣ��־
				}
			}
		}
	}

	return res;
}


/**
 * @brief ����Gָ��
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CompileGCode(){
	bool res = true;
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

//	if(g_code->mask_value & (0x01<<O_DATA))
//	{
//		printf("get O value: %lf", g_code->value[O_DATA]);
//		return true;
//	}

	if(!CheckGCode(g_code)){//������Ϸ���
		printf("failed checkGcode fun\n");
		return false;
	}


	if(!AnalyzeGCode(g_code)){//��G����ת��Ϊ��Ӧ��Message
		printf("Failed to analyzed G Code:%lld\n", this->m_p_lexer_result->line_no);
		return false;
	}


	if(!ProcessMCode(g_code)){//����M���룬����ת��Ϊ��Ӧ��Message
		printf("Failed to process M code\n");
		return false;
	}

	//���������һ��ָ����FLAG_LAST_REC��־
	ListNode<RecordMsg *> *tail = this->m_p_parser_result->TailNode();
	if(tail != nullptr){
		RecordMsg *msg = tail->data;
		if(msg != nullptr){
			msg->SetFlag(FLAG_LAST_REC, true);   //ͬ�����һ��ָ��
			if(msg->GetMsgType() != LOOP_MSG){
				msg->SetFlag(FLAG_STEP, true);    //������ͣ��־
			}
		}
	}

	return res;
}

/**
 * @brief ���Gָ��Ϸ��ԣ�����M����
 * @param gcode : �ʷ��������ָ��
 * @return true--�ɹ�  false--ʧ��,�������﷨�Դ���
 */
bool Parser::CheckGCode(LexerGCode *gcode){

//	printf("Enter CheckGCode: %d\n", gcode->gcode_count);
//	if(gcode == nullptr){
//		m_error_code = ERR_COMPILER_INTER;
//		return false;
//	}

	if((!this->m_b_multi_mcode && gcode->mcode_count > 1) ||   //��ֹͬ�ж�Mָ��ʱ��M���벻�ܳ���1��
			(m_b_multi_mcode && gcode->mcode_count > kMaxMCodeInLine)){ //����ͬ�ж�Mָ��ʱ��M���벻�ܳ���kMaxMCodeInLine��������ֵ
		m_error_code = ERR_TOO_MANY_M_CODE;  //Mָ�����
		return false;
	}

	if(gcode->tcode_count > kMaxTCodeInLine){
		m_error_code = ERR_TOO_MANY_T_CODE;   //Tָ�����
		g_ptr_trace->PrintTrace(TRACE_ERROR, COMPILER_PARSER, "Parser::CheckGCode, too many t code! tcode_count = %hhu\n", gcode->tcode_count);
		return false;
	}
/*int exp_index = static_cast<int>(g_code->value[addr]);
			while(!this->GetExpressionResult(g_code->macro_expression[exp_index], res))
 * */
	//����ģʽ���
	uint64_t mode_mask = GMODE_NONE;
	int code = 0, count = 0, exp_index = 0;
	MacroVarValue res;
	//int code_limit = kMaxGCodeCount;
	//int mode_code[kMaxGModeCount];
	memset(m_mode_code, 0x00, sizeof(int)*kMaxGModeCount);
	this->m_b_has_g53 = false;
	for(int i = 0; i < gcode->gcode_count; i++){
		if(gcode->g_value[i] < 0){//Ϊ��ֵ��˵����G���뺬�к���ʽ
			exp_index = abs(gcode->g_value[i] + 1);
			while(!this->GetExpressionResult(gcode->macro_expression[exp_index], res)){
				if(this->m_error_code != ERR_NONE)
					return false;
				usleep(10000);  //����10ms���ȴ�MC���е�λ
			}
			if(res.init){
				gcode->g_value[i] = res.value*10;  //�Ŵ�ʮ��
			}else{
				m_error_code = ERR_G_EXP_NULL;   //Gָ����ʽΪ��
				return false;
			}

		}
		code = gcode->g_value[i]/10;

		// @add zk  ���� ��G54.1 P XX ֧��
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
			// �� G54.1 P XX ת��Ϊ G5401 ~ G5499 ����
			gcode->g_value[i] = code * 10;
		}

		// @add zk

		if((code >=5401 && code <=5499) &&
				(0x00 == (mode_mask & (0x01<<14)))){   //���⴦��G5401~G5499
			mode_mask |= (0x01<<14);
			code /= 100;   //��С����Ч��Χ��
			count++;
		}else if(code == 53){   //G53ָ�����⴦��,������0��ģ̬����ΪG53������G31��ָ��ͬ��ͬ��
			this->m_b_has_g53 = true;
			count++;
		}else if(gcode->g_value[i] == G84_3_CMD){  //G84.3����39���ģ̬�Զ���ָ��
			code = kMaxGCodeCount-1;
			if(0x00 == (mode_mask & (0x01<<GCode2Mode[code]))){
				mode_mask |= (((uint64_t)0x01)<<GCode2Mode[code]);
				count++;
			}
		}else if((code < kMaxGCodeCount) &&
				(0x00 == (mode_mask & (0x01<<GCode2Mode[code])))){//������ͬ��G����
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

	//ɾ����Ч��ͬ��G���룬G����˳��ᰴ��ģ̬����С������������
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

	//TODO ����Ƿ��в���ͬ�е�ָ��
	//00/01/09��ģָ̬���ͬ��
	if(count > 1){  //���Gָ��������
		int cc = 0;
		if(m_mode_mask & GMODE_00){  //�ų�G53ָ�G53ָ�����ȥ01�����09���ƶ�ָ��ͬ�У�G53ֻ�ڵ�ǰ����Ч
			cc++;
		}
		if(m_mode_mask & GMODE_01)
			cc++;
		if((m_mode_mask & GMODE_09) && (m_mode_code[9] != G80_CMD))
			cc++;
		if(cc > 1){//�澯
			m_error_code = ERR_NOT_IN_ONE_LINE;  //���ڲ���ͬ�е�ָ��
			return false;
		}
	}

	// ������ M#XXX �ȼ��������ʽ��ֵ
	for(int i=0; i<gcode->mcode_count; i++){
		if(gcode->m_value[i] < 0){//Ϊ��ֵ��˵����M���뺬�к���ʽ
			exp_index = abs(gcode->m_value[i] + 1);
			while(!this->GetExpressionResult(gcode->macro_expression[exp_index], res)){
				if(this->m_error_code != ERR_NONE)
					return false;
				usleep(10000);  //����10ms���ȴ�MC���е�λ
			}
			if(res.init){
				gcode->m_value[i] = res.value;
			}else{
				m_error_code = ERR_M_EXP_NULL;   //
				return false;
			}

		}
	}

	//�Ƿ���ڲ���ͬ�е�M����
	if(gcode->mcode_count > 1){
		for(int i = 0; i < gcode->mcode_count; i++){
			for(int j = 0; j < kSeparatedMCodeCount; j++){
				if(gcode->m_value[i] == kSeparatedMCodeArray[j]){
					m_error_code = ERR_MCODE_SEPARATED;  //���ڲ���ͬ�е�Mָ��
					return false;
				}
			}
		}
	}

	/***************/
	// ������ T#XXX �ȼ������ʽ
	for(int i=0; i<gcode->tcode_count; i++){
		if(gcode->t_value[i] < 0){//Ϊ��ֵ��˵����T���뺬�к���ʽ
			exp_index = abs(gcode->t_value[i] + 1);
			while(!this->GetExpressionResult(gcode->macro_expression[exp_index], res)){
				if(this->m_error_code != ERR_NONE)
					return false;
				usleep(10000);  //����10ms���ȴ�MC���е�λ
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

	//����Ƿ�����Զ���ָ�ϵͳ����ĺ����ָ��
	this->m_b_call_macro_prog = this->HasMacroProgCall();

//	printf("Exit CheckGCode\n");
	return true;
}


/**
 * @brief �Ƿ���ڵ��ú�����ָ��
 * @return true--����    false--������
 */
bool Parser::HasMacroProgCall(){

	//����G65ָ��
	if((m_mode_mask & GMODE_00) && (m_mode_code[0] == G65_CMD)){
		return true;
	}

	//����09��̶�ѭ��ָ��
	if(m_mode_mask & GMODE_09){
		return true;
	}


	LexerGCode *gcode = &(m_p_lexer_result->nc_code.gcode);
	for(int i = 0; i < gcode->mcode_count; i++){
//		if(gcode->m_value[i] == 6){//����M06����ָ��
//			return true;
//		}

		//TODO �����Զ���Mָ��
	}

	//TODO �����Զ���Gָ��


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
 * @brief ����GCode�����ɶ�Ӧ��Message
 * @param gcode : �ʷ��������ָ��
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::AnalyzeGCode(LexerGCode *gcode){
//	if(gcode == nullptr){
//		m_error_code = ERR_COMPILER_INTER;
//		return false;
//	}

	//printf("mask mode : 0x%llx\n", m_mode_mask);
	bool has_move_code = false;  //�Ƿ������ƶ�Gָ��
	bool user_defined_code = false;  //�Ƿ����û��Զ���ָ��
	bool macro_call_flag = false;   //��ǰ�Ƿ��ں�������״̬



	if(((m_mode_mask & GMODE_00) && (m_mode_code[0] == G65_CMD)) ||  //G65��������
			(m_mode_mask & GMODE_09) ||                              //�̶�ѭ��ָ��
			(((m_mode_mask & GMODE_01) == 0) && (m_p_compiler_status->mode.gmode[9] != G80_CMD))){
		macro_call_flag = true;
	}

	if(!macro_call_flag){  //û�к�������ָ��ʱ�Ŵ���FeedMsg��ToolMsg
		//����Fָ��
		if(!m_b_call_macro_prog && (gcode->mask_value & (0x01<<F_DATA))){
			if(!this->CreateFeedMsg())
				return false;
		}

		//����Tָ��,Ԥѡ��
		if(!m_b_call_macro_prog && (gcode->mask_value & (0x01<<T_DATA))){
			if(!this->CreateToolMsg(gcode->t_value, gcode->tcode_count))
				return false;
		}

		//@test zk ��ȡ D H��ֵ  ԭ���� D H ��δ������Ϊָ�����
		if(gcode->mask_value&(0x01<<D_DATA)){

		}

		if(gcode->mask_value&(0x01 << H_DATA)){

		}
	}

	//�����Զ���ָ��,��39��ģ̬
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
			m_mode_code[kMaxGModeCount-1] == G1003_CMD){  //�����ٶȿ���ָ����Ϣ
			if(!CreateSpeedCtrlMsg(m_mode_code[kMaxGModeCount-1]))
				return false;
		}else if(m_mode_code[kMaxGModeCount-1] == G2000_CMD ||
		m_mode_code[kMaxGModeCount-1] == G2001_CMD ||
		m_mode_code[kMaxGModeCount-1] == G2002_CMD ||
		m_mode_code[kMaxGModeCount-1] == G2003_CMD){  //�������ؿ���ָ����Ϣ
			if(!CreateTorqueCtrlMsg(m_mode_code[kMaxGModeCount-1]))
				return false;
		}
#endif
	}


	//����21��ģָ̬�G12.1/G12.2/G12.3/G13.1  ������岹��ĥ�����ָ��
	if(m_mode_mask & GMODE_21){
		if(!this->CreatePolarIntpMsg(m_mode_code[21])){
			return false;
		}
	}

	//����02��ģָ̬�G17/G18/G19 ƽ��ָ��ָ��
	if(m_mode_mask & GMODE_02){
		if(!CreateModeMsg(m_mode_code[2]))
			return false;
	}

	//����06��ģָ̬�G20/G21 ����Ӣ��ת��
	if(m_mode_mask & GMODE_06){
		if(!CreateModeMsg(m_mode_code[6]))
			return false;
	}


	//����19��ģָ̬�G25/G26 �����ٶȱ䶯���ON/OFF
	if(m_mode_mask & GMODE_19){
		if(!this->CreateSpindleCheckMsg()){
			return false;
		}
	}

	//����03��ģָ̬�G90/G91 ���ԡ�����ָ��
	if(m_mode_mask & GMODE_03){
		if(!CreateModeMsg(m_mode_code[3]))
			return false;
	}



	//����05��ģָ̬�G94/G95 ÿ���ӽ�����ÿת����
	if(m_mode_mask & GMODE_05){
		if(!CreateModeMsg(m_mode_code[5]))
			return false;
	}


	//����00���ģָ̬��
	if(m_mode_mask & GMODE_00){//TODO ����00���ģָ̬�G04/G10/G28/G29/G71/G72/G92/G52�ȵ�, G53ָ��ڴ˴���
		if(m_mode_code[0] == G52_CMD ||
//				m_mode_code[0] == G53_CMD ||
				m_mode_code[0] == G92_CMD){  //����ϵָ��
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
		}else if(m_mode_code[0] == G65_CMD){ //��������
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


	//TODO ����07��ģָ̬�G40/G41/G42 �뾶����ָ��
	if(m_mode_mask & GMODE_07){
		//has_move_code = true;
		if(!this->CreateCompensateMsg(m_mode_code[7])){
			return false;
		}
	}

	//TODO ����08��ģָ̬�G43/G44/G49 ���Ȳ���ָ��
	if(m_mode_mask & GMODE_08){
		has_move_code = true;
		if(!this->CreateCompensateMsg(m_mode_code[8])){
			return false;
		}
	}

	//TODO ����14��ģָ̬�G54~G59 ��������ϵѡ��ָ�����G5401~G5499��չ��������ϵ
	if(m_mode_mask & GMODE_14){

		printf("create gcode : %d\n", m_mode_code[14]);
		if(!CreateCoordMsg(m_mode_code[14]))
			return false;
	}

	if(m_mode_mask & GMODE_15){
		if(!CreateModeMsg(m_mode_code[15]))
			return false;
	}

	//����10��ģָ̬�G98/G99 �̶�ѭ������ʼƽ��/R��ƽ�棩����
	if(m_mode_mask & GMODE_10){
		if(!CreateModeMsg(m_mode_code[10]))
			return false;
	}


	//����09��ģָ̬�G73/G74/G76/G80/G81~G89 �̶�ѭ��ָ��
	if(m_mode_mask & GMODE_09){
		has_move_code = true;
		if(!CreateLoopMsg(m_mode_code[9]))
			return false;
	}

	//����01��ģָ̬�G00/G01/G02/G03/G6.2/G33~G36 ������ָ��
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
			//���ɲ�֧��ָ�������Ϣ
			printf("unsupported G code[%d]!\n", m_mode_code[1]);
			m_error_code = ERR_INVALID_CODE;
			return false;
		}

		MoveMsg * node = (MoveMsg *)m_p_parser_result->TailNode();
		node->setCancelG80(true);

	}



	// ����֮ǰ��ģ̬����
	if(!has_move_code && !user_defined_code){//����λ��ָ����������ƶ�ָ����Զ���Gָ��
	//	printf("no move cmd!\n");
		if(HasAxisPos()){
	//		printf("no move cmd, has axis pos!\n");

			//����G53������G53ָ��
			if(this->m_b_has_g53){
				if(!CreateCoordMsg(m_mode_code[0])){
					return false;
				}
				else
					has_move_code = true;
				this->m_b_has_g53 = false;
			}else if(m_p_compiler_status->mode.gmode[9] == G80_CMD){//01��ģ̬
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
					//���ɲ�֧��ָ�������Ϣ
					printf("no move code, unsupported G code[%d]!\n", m_p_compiler_status->mode.gmode[1]);
					m_error_code = ERR_INVALID_CODE;
					return false;
				}

			}else{//09��ģ̬
				if(!CreateLoopMsg(m_p_compiler_status->mode.gmode[9]))
					return false;
			}
		}
	}

	return true;
}

/**
 * @brief �Ƿ������λ��ָ��
 * @return true--������λ��   false--������
 */
bool Parser::HasAxisPos(){
	int addr = 0;
	int i = 0;
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);
	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);


	while(m_axis_name[i] != '\0'){
		if(this->m_b_axis_name_ex){  //����ͨ����������չ
			addr = m_axis_name[i]-'A';
			if(m_axis_name_ex[i] == 0 && (g_code->mask_value & (0x01<<addr))){ //�Ƿ����addr����
				return true;
			}else if(m_axis_name_ex[i] > 0 &&
					(g_code->mask_pos[chn_config->chn_axis_name[i]] & (0x01<<(m_axis_name_ex[i]-1)))){
				return true;
			}
		}else{//������ͨ����������չ
			addr = m_axis_name[i]-'A';
			if(g_code->mask_value & (0x01<<addr)){ //�Ƿ����addr����
				return true;
			}
		}

		i++;
	}

	//����Բ��ָ���IJKRָ��
	if(m_p_compiler_status->mode.gmode[9] == G80_CMD &&
			(m_p_compiler_status->mode.gmode[1] == G02_CMD || m_p_compiler_status->mode.gmode[1] == G03_CMD)){
		char tmp[4] = "IJK";
		i = 0;
		while(tmp[i] != '\0'){
			addr = tmp[i]-'A';
			if(g_code->mask_value & (0x01<<addr)){ //�Ƿ����addr����
				return true;
			}
			i++;
		}
	}

	return false;
}

/**
 * @brief ����Mָ��
 * @param gcode : �ʷ��������ָ��
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::ProcessMCode(LexerGCode *gcode){

	//����Sָ��,����Mָ��֮ǰ����
	if(!m_b_call_macro_prog && (gcode->mask_value & (0x01<<S_DATA))){
		if(!this->CreateSpeedMsg())
			return false;
	}

	if(gcode->mcode_count == 1){
		if(gcode->m_value[0] == 98){  //�ӳ������Mָ��
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
 * @brief ������ʽ�Ľ��
 * @param express : ���ʽ
 * @param res ������ļ�����
 * @return true--�������ɹ�   false--�������ʧ��
 */
bool Parser::GetExpressionResult(MacroExpression &express, MacroVarValue &res){
	stack<MacroVarValue> stack_value;  //�м�ֵջ
	MacroExpression macro_bak;   //����
	res.init = false;

	if(express.empty()){//���ʽΪ�գ�����ĺ�ָ���ʽ���澯
		m_error_code = ERR_INVALID_MACRO_EXP;
		printf("@@@@@@ERR_INVALID_MACRO_EXP1\n");
		return false;
	}

	//����м�ֵջ
	while(!stack_value.empty()){
		stack_value.pop();
	}

	int value_count = 0;
	MacroRec rec;
	MacroVarValue value1, value2;  //������1��������2
	MacroVarValue res_tmp;
	while(!express.empty()){
		rec = express.top();
		macro_bak.push(rec);   //�ݴ汸��
		express.pop();
//		printf("get express res, rec,opt = %d, value = %lf\n", rec.opt, rec.value);
		if(rec.opt == MACRO_OPT_VALUE){
			value1.value = rec.value;
			value1.init = true;
			stack_value.push(value1);
//			printf("macro value : %lf, size = %d\n", rec.value, stack_value.size());
		}
		else if(rec.opt == MACRO_OPT_FIX){//��ȡ������ȥС������
			if(stack_value.empty()){//�Ҳ������������澯
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
				res_tmp.init = false;	//��ֵȡ����ȻΪ��ֵ
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_FUP){ //��ȡ������С�����ֽ�λ����������
			if(stack_value.empty()){//�Ҳ������������澯
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
				res_tmp.init = false;	//��ֵȡ����ȻΪ��ֵ
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_ROUND){//�������룬���������߼�����ָ����ʹ��ʱ���ڵ�1��С��λ���������룬��NC����ַ��ʹ��ʱ�����ݵ�ַ��С�趨��λ��������
			if(stack_value.empty()){//�Ҳ������������澯
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
				res_tmp.init = false;	//��ֵȡ����ȻΪ��ֵ
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_ABS){//����ֵ
			if(stack_value.empty()){//�Ҳ������������澯
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
				res_tmp.init = false;	//��ֵȡ����ȻΪ��ֵ
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_SIN){ //����
			if(stack_value.empty()){//�Ҳ������������澯
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
				res_tmp.init = false;	//��ֵ����������ȻΪ��ֵ
				res_tmp.value = 0.0;
			}
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_COS){//����
			if(stack_value.empty()){//�Ҳ������������澯
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
				res_tmp.init = false;	//��ֵ����������ȻΪ��ֵ
				res_tmp.value = 0.0;
			}

			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_TAN){//����
			if(stack_value.empty()){//�Ҳ������������澯
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
				res_tmp.init = false;	//��ֵ����������ȻΪ��ֵ
				res_tmp.value = 0.0;
			}

			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_ASIN){ //������
			if(stack_value.empty()){//�Ҳ������������澯
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.init){
				if(value1.value > 1 || value1.value < -1){  //������������Ч��Χ
					m_error_code = ERR_MACRO_OPT_VALUE;
					return false;
				}
				res_tmp.value = asin(value1.value);
				res_tmp.value = res_tmp.value*180./M_PI;  //�ɻ���ת��Ϊ��
				res_tmp.init = true;
			}else{
				res_tmp.init = false;   //��ֵ����������ȻΪ��ֵ
				res_tmp.value = 0.0;
			}
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_ACOS){//������
			if(stack_value.empty()){//�Ҳ������������澯
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.init){
				if(value1.value > 1 || value1.value < -1){  //������������Ч��Χ
					m_error_code = ERR_MACRO_OPT_VALUE;
					return false;
				}
				res_tmp.value = acos(value1.value);
				res_tmp.value = res_tmp.value*180./M_PI;  //�ɻ���ת��Ϊ��
				res_tmp.init = true;
			}
			else{
				res_tmp.init = false;	//��ֵ����������ȻΪ��ֵ
				res_tmp.value = 0.0;
			}
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_ATAN){//������
			if(rec.value == 2){//˫������������ֵ��Χ��-PI~PI
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
			else{//���������� ����ֵ��Χ��-PI/2~PI/2
				if(stack_value.empty()){//�Ҳ������������澯
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

			res_tmp.value = res_tmp.value*180./M_PI;//�ɻ���ת��Ϊ��
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_SQRT){ //ƽ����
			if(stack_value.empty()){//�Ҳ������������澯
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
//			if(!value1.init)
//				value1.value = 0.0;

			if(value1.value < 0){  //������������Ч��Χ
				m_error_code = ERR_MACRO_OPT_VALUE;
				return false;
			}
			res_tmp.value = sqrt(value1.value);

			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_LN){//��Ȼ����
			if(stack_value.empty()){//�Ҳ������������澯
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
//			if(!value1.init)
//				value1.value = 0.0;

			if(value1.value < 0){  //������������Ч��Χ
				m_error_code = ERR_MACRO_OPT_VALUE;
				return false;
			}
			res_tmp.value = log(value1.value);
			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_EXP){//ָ������
			if(stack_value.empty()){//�Ҳ������������澯
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
		else if(rec.opt == MACRO_OPT_EQ){ //����
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
				res_tmp.value = 1;  //��Ϊ��ֵʱ���ж�Ϊ��
			}
			else  //����һ��Ϊ��ʱ���ж�Ϊ��
				res_tmp.value = 0;
			res_tmp.init = true;

			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_NE){//������
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
				res_tmp.value = 0;	//��Ϊ��ֵʱ���ж�Ϊ��
			else
				res_tmp.value = 1;	//ֻ��һ��Ϊ��ֵʱ���ж�Ϊ��
			res_tmp.init = true;

			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_GT){//����
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
			//��ֵ��0����
			if(value2.value > value1.value)
				res_tmp.value = 1;
			else
				res_tmp.value = 0;

			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_LT){ //С��
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

			//��ֵ��0����
			if(value2.value < value1.value)
				res_tmp.value = 1;
			else
				res_tmp.value = 0;


			res_tmp.init = true;
		//	printf("LT resulst: %lf, %hhu\n", res_tmp.value, res_tmp.init);
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_GE){//���ڵ���
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

			//��ֵ��0����
			if(value2.value >= value1.value)
				res_tmp.value = 1;
			else
				res_tmp.value = 0;


			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_LE){//С�ڵ���
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

			//��ֵ��0����
			if(value2.value <= value1.value)
				res_tmp.value = 1;
			else
				res_tmp.value = 0;


			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_OR){ //��
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
		else if(rec.opt == MACRO_OPT_XOR){//���
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
		else if(rec.opt == MACRO_OPT_AND){//��
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
		else if(rec.opt == MACRO_OPT_RD){ //��ȡ��#
			if(stack_value.empty()){//�Ҳ������������澯
				m_error_code = ERR_INVALID_MACRO_EXP;
				printf("@@@@@@ERR_INVALID_MACRO_EXP3\n");
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
//			if(!value1.init)
//				value1.value = 0.0;

			if(this->IsSysVar(static_cast<int>(value1.value))){
				//�ж�MC�Ƿ����е�λ
				if(!ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index)->IsBlockRunOver() ||
						!ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index)->IsOutputMsgRunover()){
//					printf("wait run over : blockflag=%hhu, outputempty=%hhu\n", ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index)->IsBlockRunOver(),
//							ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index)->IsOutputMsgRunover());
					goto REC;  //��ֹ���㣬�ָ�����
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
		else if(rec.opt == MACRO_OPT_WR){//��ֵ��=
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
				//�ж�MC�Ƿ����е�λ
				if(!ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index)->IsBlockRunOver())
					goto REC;  //��ֹ���㣬�ָ�����
			}

			if(!SetMacroVar(static_cast<int>(value2.value), value1.value, value1.init)){
				printf("MACRO_OPT_WR error\n");
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}else{
				printf("MACRO_OPT_WR succeed\n");
			}
		}
		else if(rec.opt == MACRO_OPT_ADD){//�ӷ�
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
		else if(rec.opt == MACRO_OPT_SUB){ //����
			value_count = stack_value.size();
			if(rec.value == 1 && value_count >= 2){
				value1 = stack_value.top();
				stack_value.pop();
				value2 = stack_value.top();
				stack_value.pop();
			}
			else if(rec.value == 0 && value_count >=1 && !express.empty()){
				if(express.top().opt == MACRO_OPT_VALUE){  //˫������
					value1 = stack_value.top();
					stack_value.pop();
					value2.value = express.top().value;
					value2.init = true;
					express.pop();
				}else if(express.top().opt == MACRO_OPT_WR){  //�������磺#1=-10�ĸ�ֵ���
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
		else if(rec.opt == MACRO_OPT_MULTIPLY){//�˷�
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
		else if(rec.opt == MACRO_OPT_DIVIDE){//����
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
		else if(rec.opt == MACRO_OPT_BIN){//��BCD��תΪ��������   #1=BIN[#2]
			if(stack_value.empty()){//�Ҳ������������澯
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.value < 0){  //������������Ч��Χ
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			res_tmp.value = BCD2BIN(Double2Long(value1.value));
			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_BCD){//�Ӷ�������תΪBCD��   #1=BCD[#2]
			if(stack_value.empty()){//�Ҳ������������澯
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			value1 = stack_value.top();
			stack_value.pop();
			if(value1.value < 0){  //������������Ч��Χ
				m_error_code = ERR_INVALID_MACRO_EXP;
				return false;
			}
			res_tmp.value = BIN2BCD(Double2Long(value1.value));
			res_tmp.init = true;
			stack_value.push(res_tmp);
		}
		else if(rec.opt == MACRO_OPT_POW){//�˷�  #1=POW[#2,#3]
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
		else if(rec.opt == MACRO_OPT_MOD){//������  #i=#j MOD #k (#j��#kȡ������ȡ������#jΪ��ʱ��#iҲΪ��)
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

	REC:  //�ָ�����
	while(!macro_bak.empty()){
		rec = macro_bak.top();
		express.push(rec);
		macro_bak.pop();
	}
	usleep(1000);  //����1ms
	return false;
}

/**
 * @brief BCDתBIN
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
 * @brief BINתBCD
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
 * @brief ��double������ת��Ϊlong��
 * @param data : ��ת������
 */
long Parser::Double2Long(const double &data){
	if (data >= 0)
		return data+0.5;
	else
		return data-0.5;
}

/**
 * @brief ��ȡ�������ֵ
 * @param index[in] �������±�
 * @param value[out] : ����ֵ
 * @param init[out] : �Ƿ��ֵ
 * @return
 */
bool Parser::GetMacroVar(int index, double &value, bool &init){

	return this->m_p_variable->GetVarValue(index, value, init);

}

/**
 * @brief ���ú������ֵ
 * @param index[in] �������±�
 * @param value[in] : ����ֵ
 * @param init[in] : �Ƿ��ֵ
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::SetMacroVar(int index, double value, bool init){

	if(init)
		return this->m_p_variable->SetVarValue(index, value);
	else
		return this->m_p_variable->ResetVariable(index);
}

/**
 * @brief ������������Ϣ����������Ϣ����
 * @param err : ������
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateErrorMsg(const ErrorType err){

	RecordMsg *new_msg = new ErrorMsg(err);
	if(new_msg == nullptr){
		//�ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�

	if(this->m_p_compiler_status->jump_flag){
		new_msg->SetFlag(FLAG_JUMP, true);
		this->m_error_code = ERR_NONE;
	}

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief ������Ϣ��ʾ��Ϣ����������Ϣ����
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateInfoMsg(){
	int info_code = 0;
	int info_type = 0;
	double value = 0.0;

	//��ȡ��Ϣ���
	if(!GetCodeData(P_DATA, value)){
		if(m_error_code == ERR_NONE)
			m_error_code = ERR_NO_P_DATA;
		return false;
	}

	info_code = static_cast<int>(value);

	//��ȡ��Ϣ���ͣ���ʡ�ԣ�Ĭ��Ϊ0-��ʾ��Ϣ
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
		//�ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�

	((ErrorMsg *)new_msg)->SetInfoType(info_type);


	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief ����ģ̬��Ϣ����������Ϣ���У�����Ϣ�������޲�����ģ̬Gָ��
 * @param gcode : �޲�����ģ̬Gָ��
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateModeMsg(const int gcode){
	RecordMsg *new_msg = new ModeMsg(gcode);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}


	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	//if(ChannelEngine::GetInstance()->GetChnControl(m_n_channel_index)->IsStepMode())
	//	new_msg->SetFlag(FLAG_WAIT_MOVE_OVER, true);
	//@ ��� G90  G01 xxx G91  �м���˶�ָ��ִ�в��� G91ָ��ȴ��˶����� ���³���������
	new_msg->SetFlag(FLAG_WAIT_MOVE_OVER, true);
//	if(gcode == G90_CMD || gcode == G91_CMD || gcode == G20_CMD || gcode == G21_CMD){
//		m_p_compiler_status->mode.gmode[GetModeGroup(gcode)] = gcode;   //G91/G90���������ƶ�ָ��ͬ�У�������ǰ����
//	}

	if(gcode == G17_CMD || gcode == G18_CMD || gcode == G19_CMD){
		m_p_compiler_status->mode.gmode[GetModeGroup(gcode)] = gcode;   //G17/G18/G19���������ƶ�ָ��ͬ�У�������ǰ����
	}

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief ����ѭ��ָ����Ϣ����������Ϣ����
 * @param gcode : ѭ��Gָ��
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateLoopMsg(const int gcode){
//	DPointChn source = this->m_p_compiler_status->cur_pos;   //���
//	DPointChn target = source;	//�յ�
//	uint32_t mask = 0;   //��mask

//	if(gcode != G80_CMD){
//		if(!GetTargetPos(target, mask))
//			return false;
//	}

	//��ȡ����
	double *param = nullptr;
	uint8_t pc = 0;  //��������
	uint32_t pm = 0;  //����mask
	this->GetParaData(&param, pc, pm);

	RecordMsg *new_msg = new LoopMsg(gcode, param, pc, pm);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);
	//@test zk run messageʱ��ȥ�޸�ģ̬  �������޸�̫����
	//this->m_p_compiler_status->mode.gmode[GetModeGroup(gcode)] = gcode;
//	this->m_p_compiler_status->mode.move_mode = 9;

//	printf("create loop msg : %d\n", gcode);
	ProcessLastBlockRec(new_msg);

//	this->m_p_compiler_status->cur_pos = target; //���±��뵱ǰλ��
	return true;
}

/**
 * @brief ��������ϵָ����Ϣ����������Ϣ����
 * @param gcode : ����ϵGָ�G52/G53/G92/
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateCoordMsg(const int gcode){

	DPointChn pos;	//�յ�
	DPointChn src = this->m_p_compiler_status->cur_pos;   //���
	uint32_t axis_mask = 0;   //������
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
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);
//	printf("create coord msg:%d\n", gcode);

	return true;
}

/**
 * @brief ��������ٶ�ָ����Ϣ����������Ϣ����
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateFeedMsg(){
	double df_feed = 0;  //�����ٶȣ���λ��mm/min

	if(!GetCodeData(F_DATA, df_feed)){
		return false;
	}

//	int feed = static_cast<int>(df_feed);

	RecordMsg *new_msg = new FeedMsg(df_feed, m_p_compiler_status->mode.f_mode);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_b_f_code = true;
	m_p_parser_result->Append(new_msg);
	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 *
 * @param mcode �� Mָ��ֵ����ָ��
 * @param total : ͬ��Mָ������
 * @return
 */
bool Parser::CreateAuxMsg(int *mcode, uint8_t total){
	AuxMsg *new_msg = new AuxMsg(mcode, total);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);
	m_p_parser_result->Append(new_msg);
	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief �����ӳ��������Ϣ����������Ϣ����
 * @return
 */
bool Parser::CreateSubProgCallMsg(){
	int pcode = 0;		//�ӳ�������
	int lcode = 1;  	//�ظ����ô�����Ĭ��һ��

	double value = 0.0;   //��ʱ����
	if(!GetCodeData(P_DATA, value)){
		if(m_error_code == ERR_NONE)
			m_error_code = ERR_NO_SUB_PROG_P;  //û��ָ���ӳ����
		return false;
	}else{
		pcode = static_cast<int>(value);
	}

	if(GetCodeData(L_DATA, value)){
		lcode = static_cast<int>(value);
	}

	RecordMsg *new_msg = new SubProgCallMsg(pcode, lcode);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);
	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief ��������ת��ָ����Ϣ����������Ϣ����
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateSpeedMsg(){
	double df_speed = 0.0;

	GetCodeData(S_DATA, df_speed);

	int speed = static_cast<int>(df_speed);

	RecordMsg *new_msg = new SpeedMsg(speed);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);
	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief ���쵶��ָ����Ϣ����������Ϣ����
 * @param tcode �� Tָ��ֵ����ָ��
 * @param total : ͬ��Tָ������
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateToolMsg(int *tcode, uint8_t total){

	if(*tcode < 0 or *tcode > 60){
		CreateError(ERR_T_EXP_NULL, ERROR_LEVEL, CLEAR_BY_MCP_RESET, m_p_lexer_result->line_no);
		return false;
	}

	RecordMsg *new_msg = new ToolMsg(tcode, total);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);
	m_p_parser_result->Append(new_msg);

//	this->m_p_compiler_status->mode.t_mode = tcode[0];   //�޸ı�����״̬

	printf("CreateToolMsg, total count = %hhu\n", total);
	ProcessLastBlockRec(new_msg);
	return true;
}

/**
 * @brief ���쵶��ָ����Ϣ����������Ϣ����
 * @param gcode : ����Gָ��
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateCompensateMsg(int gcode){

	DPointChn source = this->m_p_compiler_status->cur_pos;   //���
	DPointChn target = source;	//�յ�

	double df_data = 0.0;
	uint16_t data = 0;
	uint32_t mask = 0;  //��

	uint8_t move_type = MOVE_G00;

	if(gcode == G41_CMD || gcode == G42_CMD || gcode == G40_CMD){  //�������߰뾶������Ҫ���ƶ�

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

	}else if(gcode == G43_CMD || gcode == G44_CMD || gcode == G49_CMD){  //���߳��Ȳ���
		if(!GetTargetPos(target, mask))
			return false;

		if(!GetCodeData(H_DATA, df_data))
		{
			if(gcode == G49_CMD){
				df_data = 0;
			}else if(m_error_code == ERR_NONE){//ȱ��Hֵָ��
				m_error_code = ERR_NO_H_DATA;
				return false;
			}
		}
		data = static_cast<uint16_t>(df_data);

		if(m_p_compiler_status->mode.gmode[1] == G01_CMD)
			move_type = MOVE_G01;
	}else if(gcode == G43_4_CMD){   //��������RTCP

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
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	//ͬ���Ƿ����G53����ָ����е����
	if(this->m_b_has_g53){
		((CompensateMsg *)new_msg)->SetMachCoord(true);
	}

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

//	this->m_p_compiler_status->cur_pos = target; //���±��뵱ǰλ��

	return true;
}

/**
 * @brief ����G00���ٶ�λ��Ϣ����������Ϣ����
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateRapidMsg(){
	DPointChn source = this->m_p_compiler_status->cur_pos;   //���
	DPointChn target = source;	//�յ�
	uint32_t axis_mask = 0;

	uint8_t io = 0;
	double data = 0;
//	double pmc_data[this->m_n_pmc_axis_count];   //PMC��λ������
//	uint32_t pmc_mask = 0;
	uint8_t pmc_count = 0;
//	bool inc_flag = true;

	if(!GetTargetPos(target, axis_mask, &pmc_count))   //��ȡ�岹��Ŀ��λ��
		return false;

//	uint8_t count = this->GetTargetPosEx(pmc_data, pmc_mask, inc_flag, m_n_pmc_axis_count);   //��ȡPMC���˶�����
//	if(count == 0xFF){
//		return false;
//	}

	RecordMsg *new_msg = new RapidMsg(source, target, axis_mask);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�

	if(GetCodeData(P_DATA, data)){   //IO����
		io = static_cast<uint8_t>(data);
		((RapidMsg *)new_msg)->SetIoData(io);   //����IO����
	}

	if(pmc_count > 0){
        ((RapidMsg *)new_msg)->SetPmcAxisCount(pmc_count);
	}

	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	//ͬ���Ƿ����G53����ָ����е����
	if(this->m_b_has_g53){
		((RapidMsg *)new_msg)->SetMachCoord(true);
	}

	m_p_parser_result->Append(new_msg);
    ProcessLastBlockRec(new_msg);

    // �������ת���ƶ�����Ҫ����������Ȧ��msg
//    SCAxisConfig *axis_config = g_ptr_parm_manager->GetAxisConfig();
//    SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig();
//    for(int i = 0; i < chn_config->chn_axis_count; i++){
//        uint8_t mask = axis_mask & (0x01 << i);
//        if(!mask || (axis_config[i].axis_type != AXIS_ROTATE))
//            continue;
//        double pos = target.GetAxisValue(i);
//        if(pos >= 0 && pos < 360)   // ������Χ������Ҫ��
//            continue;
//        // ���һ��G200��Ϣ,����ת��������Ȧ
//        RecordMsg *clr_msg = new ClearCirclePosMsg(2000, mask, 360*1000);
//        new_msg->SetLineNo(this->m_p_lexer_result->line_no);
//        m_p_parser_result->Append(clr_msg);
//        ProcessLastBlockRec(clr_msg);
//        ScPrintf("append ClearCirclePosMsg, msk = %u",mask);
//    }

//	this->m_p_compiler_status->cur_pos = target; //���±��뵱ǰλ��

	return true;
}

/**
 * @brief ����G01ֱ��������Ϣ����������Ϣ����
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateLineMsg(){
//    ScPrintf("Parser::CreateLineMsg\n");

	DPointChn source = this->m_p_compiler_status->cur_pos;   //���
	DPointChn target = source;	//�յ�
	RecordMsg *new_msg = nullptr;
	uint32_t axis_mask = 0;
	uint8_t io = 0;
	double data = 0;
//	printf("line msg target init: (%lf, %lf, %lf, %lf, %lf, %lf)\n",target.x, target.y, target.z, target.a4,
//			target.a5, target.a6);

//	double pmc_data[this->m_n_pmc_axis_count];   //PMC��λ������
//	uint32_t pmc_mask = 0;
//	bool inc_flag = true;
	uint8_t pmc_count = 0;

	if(!GetTargetPos(target, axis_mask, &pmc_count))
		return false;
   ScPrintf("Parser::CreateLineMsg pmc_count = %u", pmc_count);


//	uint8_t count = this->GetTargetPosEx(pmc_data, pmc_mask, inc_flag, m_n_pmc_axis_count);   //��ȡPMC���˶�����
//	if(count == 0xFF){
//		return false;
//	}


//	printf("line msg get target : (%lf, %lf, %lf, %lf, %lf, %lf), mask = 0x%x\n",target.x, target.y, target.z, target.a4,
//			target.a5, target.a6, axis_mask);

	if(!m_b_f_code){//δָ��Fֵ
		printf("ERR_NO_F_DATA\n");
		m_error_code = ERR_NO_F_DATA;
		return false;
	}

//    // ����ڹ�˿״̬�ƶ�z�ᣬͬʱҲָ������
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
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�

	if(GetCodeData(P_DATA, data)){   //IO����
		io = static_cast<uint8_t>(data);
		((LineMsg *)new_msg)->SetIoData(io);   //����IO����
	}

	if(pmc_count > 0){
        ((LineMsg *)new_msg)->SetPmcAxisCount(pmc_count);
        ScPrintf("SetPmcAxisData count = %u", pmc_count);
	}
//	printf("line msg create, wait=%hhu\n", new_msg->IsNeedWaitMsg());

//	printf("createlinemsg, lineno = %lld\n", this->m_p_lexer_result->line_no);
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	//ͬ���Ƿ����G53����ָ����е����
	if(this->m_b_has_g53){
		((LineMsg *)new_msg)->SetMachCoord(true);
	}

	m_p_parser_result->Append(new_msg);
    ProcessLastBlockRec(new_msg);

//    // �������ת���ƶ�����Ҫ����������Ȧ��msg
//    SCAxisConfig *axis_config = g_ptr_parm_manager->GetAxisConfig();
//    SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig();
//    for(int i = 0; i < chn_config->chn_axis_count; i++){
//        uint8_t mask = axis_mask & (0x01 << i);
//        if(!mask || (axis_config[i].axis_type != AXIS_ROTATE))
//            continue;
//        double pos = target.GetAxisValue(i);
//        if(pos >= 0 && pos < 360)   // ������Χ������Ҫ��
//            continue;
//        // ���һ��G200��Ϣ,����ת��������Ȧ
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
 * @brief ����Բ��������Ϣ����������Ϣ����
 * @param gcode : Բ��G���룬G02/G03
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateArcMsg(const int gcode){
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);
//	if(g_code == nullptr){
//		m_error_code = ERR_COMPILER_INTER;
//		return false;
//	}
    double i_number = 0, j_number = 0, r_number = 0;

	if(!m_b_f_code){//δָ��Fֵ
		printf("ERR_NO_F_DATA\n");
		m_error_code = ERR_NO_F_DATA;
		return false;
	}

    //TODO ������
	DPointChn source = this->m_p_compiler_status->cur_pos;   //�������
	DPointChn target, center;  //�յ����꣬Բ������
	double radius = 0.0;  //�뾶
	int8_t major_flag = 1;  //�Ż���־�� 1--�ӻ�    -1--�Ż�
	int8_t circle_flag = 0;	//��Բ��־�� 0--Բ��    1--��Բ
	int8_t dir_flag = -1;  //�����־��-1:clockwise,1:anticlockwise
	uint32_t axis_mask = 0;

	uint8_t io = 0;
	double data = 0;

	if(gcode == G03_CMD)
		dir_flag = 1;

    uint8_t pmc_count = 0;
	//��ȡ�յ�����
    if(!GetTargetPos(target, axis_mask, &pmc_count)){
		return false;   //�������󣬷���
	}

	//��ȡ��������
	if(GetCodeData(R_DATA, radius)){//����R������IJK��Rͬʱ����ʱR���ȣ�����IJK
		if((g_code->mask_dot & (0x01<<R_DATA)) == 0){
			radius /= 1000.;   //ʡ��С��������umΪ��λ
		}
		if(radius < 0 ) major_flag = -1; 	//�Ż�

		r_number = radius;

        radius = fabs(radius);

		//����Բ������
        //printf("===== ARC SOURCE %lf %lf %lf\n",
        //        source.GetAxisValue(0), source.GetAxisValue(1), source.GetAxisValue(2));


        DPointChn target_pos = target;
        if (this->m_p_compiler_status->mode.gmode[3] == G91_CMD)
        {// llx add �������ģʽ��Բ�������޷���Ǣ����
            target_pos += source;
        }
        if(!CalArcCenter(source, target_pos, radius, major_flag*dir_flag, center)){
			return false; //�������󣬷���
		}

	}
	else{
		double i = 0.0, j = 0.0, k = 0.0;  //I/J/K��ֵ,ʡ����Ĭ��Ϊ0
		bool has_data = false;  //��־�Ƿ����IJK����
		if(GetCodeData(I_DATA, i)){//��ȡI����
			if((g_code->mask_dot & (0x01<<I_DATA)) == 0){
				i /= 1000.;   //ʡ��С��������umΪ��λ
			}
			has_data = true;
			i_number = i;
		}
		if(GetCodeData(J_DATA, j)){//��ȡI����
			if((g_code->mask_dot & (0x01<<J_DATA)) == 0){
				j /= 1000.;   //ʡ��С��������umΪ��λ
			}
			has_data = true;
            j_number = j;
		}
		if(GetCodeData(K_DATA, k)){//��ȡI����
			if((g_code->mask_dot & (0x01<<K_DATA)) == 0){
				k /= 1000.;   //ʡ��С��������umΪ��λ
			}
			has_data = true;
		}

		if(!has_data){//TODO Բ������ȱʧ���澯
			m_error_code = ERR_ARC_NO_DATA;
			return false;
		}


		//����Բ�������Լ��뾶
		if(m_p_compiler_status->mode.gmode[2] == PLANE_XY){ //XYƽ��
			radius = sqrt(i * i + j * j);
		//	printf("i = %lf, j = %lf, radius = %lf\n", i, j, radius);

		}else if(m_p_compiler_status->mode.gmode[2] == PLANE_ZX){ //ZXƽ��
			radius = sqrt(i * i + k * k);

		}else if(m_p_compiler_status->mode.gmode[2] == PLANE_YZ){ //YZƽ��
			radius = sqrt(j * j + k * k);
		}
		DPointChn vec(i,j,k);
		center = source + vec;  //Բ������

//		printf("cen[%lf, %lf, %lf], src[%lf, %lf, %lf], vec[%lf, %lf, %lf]\n", center.x, center.y, center.z, source.x,source.y, source.z,
//				vec.x, vec.y, vec.z);
//
//		printf("tar[%lf, %lf, %lf]\n", target.x, target.y, target.z);

        DPointChn target_pos = target;

        if(source != target_pos){
			DPlane cen = Point2Plane(center, m_p_compiler_status->mode.gmode[2]);
            DPlane tar = Point2Plane(target_pos, m_p_compiler_status->mode.gmode[2]);
            double dr2 = GetVectLength(cen, tar);

			if(fabs(dr2-radius) > 2e-3){  //�����յ㵽Բ�ľ�����2um��澯
				printf("cal arc error, %lf, %lf, %d\n", dr2, radius, m_p_compiler_status->mode.gmode[2]);
				m_error_code = ERR_ARC_INVALID_DATA;
				return false;
			}

			//�ж����ӻ�
			DPlane src = Point2Plane(source, m_p_compiler_status->mode.gmode[2]);
			double angle_start = GetVectAngle(src, cen);
			double angle_end = GetVectAngle(tar, cen);
			double angle = (angle_end - angle_start) * dir_flag;
			if(angle < 0)
				angle += 2*M_PI;
			if(angle > M_PI)
				major_flag = -1;   //����180�ȣ��Ż�
		}
		else{
			//�ж��Ƿ���Բ
			circle_flag = 1;    //IJK��̣������յ��غ���Ϊ��Բ
			major_flag = -1;    //�Ż�
		}

	}

	RecordMsg *new_msg = new ArcMsg(gcode, source, target, center, radius, m_p_compiler_status->mode.f_mode, axis_mask,
			dir_flag, major_flag, circle_flag);

	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
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

	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	((ArcMsg *)new_msg)->SetPlaneMode(m_p_compiler_status->mode.gmode[2]);   //����ƽ��ģ̬

	//ͬ���Ƿ����G53����ָ����е����
	if(this->m_b_has_g53){
		((ArcMsg *)new_msg)->SetMachCoord(true);
	}

	if(GetCodeData(P_DATA, data)){   //IO����
		io = static_cast<uint8_t>(data);
		((ArcMsg *)new_msg)->SetIoData(io);   //����IO����
	}

	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	this->m_p_compiler_status->mode.gmode[1] = gcode;
	this->m_p_compiler_status->mode.gmode[9] = G80_CMD;  //�Զ�ȡ��ѭ��ָ��
 //   this->m_p_compiler_status->cur_pos = target; //���±��뵱ǰλ��
	return true;
}

/**
 * @brief ���켫����岹�����Ϣ
 * @param gcode : ������岹��ĥ�����G���룬G12.1/G12.2/G13.1
 * @return true--�ɹ�  false--ʧ��
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
		double data;  //��ʱ����


		//��ȡ����
		if(GetCodeData(P_DATA, data)){
			p_data = data;			//ƽ��ʱ��  ��λ��ms
		}
		if(GetCodeData(R_DATA, data)){
			r_data = data;    //ɰ�ְ뾶����
		}
		if(GetCodeData(L_DATA, data)){
			l_data = data;    //��δʹ��
		}
		if(GetCodeData(Q_DATA, data)){
			q_data = data;    //��δʹ��
		}

		if(gcode == G12_3_CMD){  //G12.3ָ���X��Y��I��J����
			if(GetCodeData(X_DATA, data)){
				if((g_code->mask_dot & (0x01<<X_DATA)) == 0){
					x_data = data;   //ʡ��С��������umΪ��λ
				}
				x_data = data*1000;			//ĥ�����X�����꣬  ��λ��um
			}
			if(GetCodeData(Y_DATA, data)){
				y_data = data*1000;    //ĥ�����Y�����꣬  ��λ��um
			}
			if(GetCodeData(I_DATA, data)){
				i_data = data*1000000;    //ĥ������λʸ�����Ŵ�1000000��
			}
			if(GetCodeData(J_DATA, data)){
				j_data = data*1000000;    //ĥ������λʸ�����Ŵ�1000000��
			}

		}
	}
#endif
	RecordMsg *new_msg = new PolarIntpMsg(gcode, p_data, r_data, l_data, q_data, x_data, y_data, i_data, j_data);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	this->m_p_compiler_status->mode.gmode[21] = gcode;
	return true;
}

/**
 * @brief ������ת��������Ȧλ����Ϣ
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateClearCirclePosMsg(){
	int p_data = -1;
	uint32_t mask = 0;
	double data;  //��ʱ����

	int gcode = 2000;  //G200

	//��ȡ����
	if(GetCodeData(P_DATA, data)){
		data *= 1000;    //ģ����λmm������1000��ת��Ϊum��λ
		p_data = data;
	}else{
		m_error_code = ERR_LACK_OF_PARAM;   //ȱ�ٱ�Ҫ����
		return false;
	}

	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
	SCAxisConfig *axis_config = g_ptr_parm_manager->GetAxisConfig();
	
	uint8_t phy_axis = 0;
	for(int i = 0; i < chn_config->chn_axis_count; i++){
		phy_axis = g_ptr_chn_engine->GetChnAxistoPhyAixs(m_n_channel_index,i);
		if(phy_axis == 0xFF)    //δ����
			continue;

		if(axis_config[phy_axis].axis_type != AXIS_ROTATE)   //����ת��
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
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	this->m_p_compiler_status->mode.gmode[39] = gcode;
	return true;
}

/**
 * @brief ������ʱ�ȴ���Ϣ
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateTimeWaitMsg(){
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	uint32_t time = 0;
	double data = 0;

	// ��������:  Pָ��ms  ��С�����޹�  X ֧��С������
	if(GetCodeData(X_DATA, data)){
		//time = data * 1000;
		if((g_code->mask_dot & (0x01<<X_DATA)) == 0){  //ʡ��С����������ֵ��λΪms
			time = data;
		}else
			time = data*1000;   //��С���㣬����ֵ��λΪs
	}else if(GetCodeData(P_DATA, data)){
		time = data;
		/*if((g_code->mask_dot & (0x01<<P_DATA)) == 0){  //ʡ��С����������ֵ��λΪms
			time = data;
		}else
			time = data*1000;   //��С���㣬����ֵ��λΪs*/
	}

	RecordMsg *new_msg = new TimeWaitMsg(time);   //G04
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	this->m_p_compiler_status->mode.gmode[0] = G04_CMD;

//	printf("create time wait msg, time = %u\n", time);

	return true;
}

/**
 * @brief ����ο��㷵����Ϣ
 * @param gcode : G����
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateRefReturnMsg(const int gcode){
//	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	uint32_t mask = 0;
	DPointChn middle;

	this->GetTargetPos(middle, mask);

	RefReturnMsg *new_msg = new RefReturnMsg(gcode, mask, middle);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�

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
 * @brief ������������ת���  G31
 * @param gcode : G����  
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateSkipRunMsg(const int gcode){
//	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	DPointChn source = this->m_p_compiler_status->cur_pos;   //���
	DPointChn target = source;	//�յ�
	uint32_t axis_mask = 0;



	if(!GetTargetPos(target, axis_mask))
		return false;

	if(!m_b_f_code){//δָ��Fֵ
		printf("ERR_NO_F_DATA\n");
		m_error_code = ERR_NO_F_DATA;
		return false;
	}
	

	RecordMsg *new_msg = new SkipMsg(source, target, m_p_compiler_status->mode.f_mode, axis_mask);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	//ͬ���Ƿ����G53����ָ����е����
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
 * @brief �����������ָ����Ϣ����������Ϣ����
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateMacroProgCallMsg(){

	int pcode = 0;		//���������
	int lcode = 1;  	//�ظ����ô�����Ĭ��һ��

	double value = 0.0;   //��ʱ����
	if(!GetCodeData(P_DATA, value)){
		if(m_error_code == ERR_NONE)
			m_error_code = ERR_NO_SUB_PROG_P;  //û��ָ���ӳ����
		return false;
	}else{
		pcode = static_cast<int>(value);
	}

	if(GetCodeData(L_DATA, value)){
		lcode = static_cast<int>(value);
	}

	//��ȡ����
	double *param = nullptr;
	uint8_t pc = 0;  //��������
	uint32_t pm = 0;  //����mask
	this->GetParaData(&param, pc, pm);


	RecordMsg *new_msg = new MacroProgCallMsg(pcode, lcode, param, pc, pm);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);
	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	return true;
}

/**
 * @brief �����Զ��Ե�ָ����Ϣ����������Ϣ����
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateAutoToolMeasureMsg(){
	printf("Parser::CreateAutoToolMeasureMsg\n");
	int hcode = 0;		//д���Hֵ
	int lcode = 1;  	//�ظ��Ե�������Ĭ��һ��

	DPointChn target = this->m_p_compiler_status->cur_pos;   //�յ�
	uint32_t axis_mask = 0;  //��mask


	double value = 0.0;   //��ʱ����
	if(!GetCodeData(H_DATA, value)){
		if(m_error_code == ERR_NONE){
			//Ĭ��ʹ�õ�ǰHֵ
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
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);
	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	printf("exit Parser::CreateAutoToolMeasureMsg\n");
	return true;
}



#ifdef USES_SPEED_TORQUE_CTRL	
/**
 * @brief �����ٶȿ�����Ϣ
 * @param gcode : Gָ��ֵ
 * @return
 */
bool Parser::CreateSpeedCtrlMsg(const int gcode){
	uint32_t mask = 0;

	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
	double speed[chn_config->chn_axis_count];   //Ŀ���ٶ�
	

	printf("--IN--Parser:CreateSpeedCtrlMsg:gcode=%d ",gcode);
	uint8_t count = this->GetTargetSpeed(speed, mask, chn_config->chn_axis_count);
	if(count == 0xFF)
		return false;

	printf("CreateSpeedCtrlMsg:count=%hhu, mask=0x%x, speed=[%lf, %lf, %lf]\n", count, mask, speed[0], speed[1], speed[2]);
	RecordMsg *new_msg = new SpeedCtrlMsg(gcode, speed, count, mask);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
	if(this->m_p_compiler_status->jump_flag)
		new_msg->SetFlag(FLAG_JUMP, true);

	m_p_parser_result->Append(new_msg);

	ProcessLastBlockRec(new_msg);

	this->m_p_compiler_status->mode.gmode[0] = gcode;

	return true;
}

/**
 * @brief �������ؿ�����Ϣ
 * @param gcode : Gָ��ֵ
 * @return
 */
bool Parser::CreateTorqueCtrlMsg(const int gcode){
	uint32_t mask = 0;
    double data = 0;
	int16_t lmt;
	uint32_t time = 0;  //��ʱʱ��

	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
	double torque[chn_config->chn_axis_count];   //Ŀ������

    printf("--IN--Parser:CreateTorqueCtrlMsg:gcode=%d ",gcode);

    LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);
	
	uint8_t count = this->GetTargetTorque(torque, mask, chn_config->chn_axis_count);
	if(count == 0xFF)
		return false;

	if(GetCodeData(P_DATA, data)){
		if((g_code->mask_dot & (0x01<<P_DATA)) == 0){  //ʡ��С����������ֵ��λΪms
			time = data;
		}else
			time = data*1000;   //��С���㣬����ֵ��λΪs

	//	printf("torque msg: q=%u\n", time);
	}

	RecordMsg *new_msg = new TorqueCtrlMsg(gcode, torque, count, mask, time);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	data = 0;
	if(GetCodeData(S_DATA, data)){   //IO����
		lmt = static_cast<int16_t>(data);
		((TorqueCtrlMsg*)new_msg)->SetSpeedLmtValue(lmt);   //�����ٶ�����ֵ
	}
	
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
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
 * @brief ��������ת�ټ����Ϣ(G25/G26)����������Ϣ����
 * @return true--�ɹ�  false--ʧ��
 */
bool Parser::CreateSpindleCheckMsg(){
//	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	int p = 0, q = 0, r = 0, i = 0;
	//TODO ������


	RecordMsg *new_msg = new SpindleCheckMsg(m_mode_code[19], p, q, r, i);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
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
	DPointChn source = this->m_p_compiler_status->cur_pos;   //���
	DPointChn target = source;	//�յ�

	RecordMsg *new_msg = nullptr;
	uint32_t axis_mask = 0;
	uint8_t io = 0;
	double data = 0;

	uint8_t pmc_count = 0;

	if(!GetTargetPos(target, axis_mask, &pmc_count))
		return false;

	new_msg = new ExactStopMsg(source, target, axis_mask);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�

	m_p_parser_result->Append(new_msg);
	ProcessLastBlockRec(new_msg);
	return true;
}

/**
 * @brief �Ƿ���ָ����ַ������
 * @param addr : ��ַ��
 * @return true--��   false--û��
 */
bool Parser::HasCodeData(DataAddr addr){
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	if(g_code->mask_value & (0x01<<addr)) //�Ƿ����addr����
		return true;

	return false;
}

/**
 * @brief ��ȡָ����ַ�ֵ�����
 * @param addr : ��ַ��
 * @param [out]data : ��������
 * @return true--�ҵ�����    false--û������
 */
bool Parser::GetCodeData(DataAddr addr, double &data){

	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	uint32_t mask = (0x01<<addr);
	MacroVarValue res;

    if(g_code->mask_value & mask){ //�Ƿ����addr����
		if(g_code->mask_macro & mask){ //���ʽ
			int exp_index = static_cast<int>(g_code->value[addr]);
			while(!this->GetExpressionResult(g_code->macro_expression[exp_index], res)){
				if(this->m_error_code != ERR_NONE)
					return false;
				usleep(10000);  //����10ms���ȴ�MC���е�λ
			}

			if(!res.init){ //��ֵ������Դ˵�ַ��
				return false;
			}else
				data = res.value;
		}
		else{ //����ֵ
			data = g_code->value[addr];
		}
		g_code->mask_value &= (~mask);   //ʹ�ù���͸�λ�˲���mask
	}else{//�޴˵�ַ������
        //printf("Get %c data: no data\n", 'A'+addr);
		//data = g_code->value[addr];
		return false;
	}

	return true;
}

/**
 * @brief ��ȡ�Ա�������,����G/L/N/O/P��������ĸ�������Ա��������21��
 * @modify zk ���P��������
 * @param param[out] : �����Ա���ֵ����̬�����ڴ棬ʹ�ú��������ͷ�
 * @param pc[out] �� �����Ա�������
 * @param mask[out] �� �����Ա���mask
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
			//��ͳ���Ա�������  @modify zk ���P��������
			if(dd != G_DATA && dd!= L_DATA && dd != N_DATA && dd != O_DATA){
				pc++;
				mask |= (0x01<<dd);
			}

		}
		tm = tm>>1;
		dd++;
	}

	if(pc == 0)  //û���Ա���
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
 * @brief ������չ���ƻ�ȡָ������
 * @param name : �������ַ�
 * @param name_ex : ����չ�����
 * @param data[out] : ���ص�����
 * @return true--�ҵ�����    false--û������
 */
bool Parser::GetAxisExData(uint8_t name, uint8_t name_ex, double &data){
	if(name > 8 || (name_ex > 16))
		return false;

	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);
	uint16_t mask = (0x01<<(name_ex-1));
	MacroVarValue res;

	if(g_code->mask_pos[name] & mask){
//		printf("Parser::GetAxisExData, mask_pos=0x%x, mask=0x%x\n", g_code->mask_pos[name], mask);
		if(g_code->mask_pos_macro[name] & mask){ //���ʽ
			int exp_index = static_cast<int>(g_code->pos_value[name][name_ex-1]);
			while(!this->GetExpressionResult(g_code->macro_expression[exp_index], res)){
				if(this->m_error_code != ERR_NONE)
					return false;
				usleep(10000);  //����10ms���ȴ�MC���е�λ
			}

			if(!res.init){ //��ֵ������Դ˵�ַ��
				return false;
			}else
				data = res.value;
		}
		else{ //����ֵ
			data = g_code->pos_value[name][name_ex-1];
		}
		g_code->mask_pos[name] &= (~mask);   //ʹ�ù���͸�λ�˲���mask
	}else
		return false;

	return true;
}

/**
 * @brief ��ȡĿ�������,ֻ�Բ岹NC��
 * @param [out]target : ����Ŀ�������
 * @param [out]axis_mask: �������ƶ���mask����־��Щ����ƶ�
 * @param [out]pmc_count: PMC�����
 * @return  true--�ɹ�    false--ʧ��
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
		if(m_b_axis_name_ex && chn_config->chn_axis_name_ex[i] > 0){  //����չ�±�
			has_valid_world = true;
			if(this->GetAxisExData(axis_name_idx, this->m_axis_name_ex[i], data)){  //������
				if(((g_code->mask_dot_ex[axis_name_idx] & (0x01<<(chn_config->chn_axis_name_ex[i]-1))) == 0) &&
						((g_code->mask_pos_macro[axis_name_idx] & (0x01<<(chn_config->chn_axis_name_ex[i]-1))) == 0)){  //û��С���㲢�ҷǺ���ʽ
					data /= 1000.;   //ʡ��С��������umΪ��λ
				}
				*p_target_pos = data;
//				if((this->m_mask_pmc_axis & tm) == 0 && this->m_p_compiler_status->mode.gmode[3] == G91_CMD){  //�岹�����������ģʽ
//					*p_target_pos += *p_source_pos;
//				}
				axis_mask |= tm;  //������mask
                if(this->m_mask_pmc_axis & tm)
                {
                    pmc_axis_count++;   //����PMC��
                }

			}else if(m_error_code == ERR_NONE){ //�޴˲���,ʹ�������Ӧ��λ��
				*p_target_pos = *p_source_pos;
			}else{//�쳣
				printf("get target pos failed\n");
				return false;
			}
		}else{//����չ�±�
			 addr = static_cast<DataAddr>(m_axis_name[i]-'A');
			 if(GetCodeData(addr, data)){//�д˲�������ȡ�ɹ�
				 has_valid_world = true;
				 if(((g_code->mask_dot & (0x01<<addr)) == 0) &&
						((g_code->mask_macro & (0x01<<addr)) == 0)){  //û��С���㲢�ҷǺ���ʽ
					data /= 1000.;   //ʡ��С��������umΪ��λ
				}
				*p_target_pos = data;
//				if((this->m_mask_pmc_axis & tm) == 0 && this->m_p_compiler_status->mode.gmode[3] == G91_CMD){  //�岹�����������ģʽ
//					*p_target_pos += *p_source_pos;
//				}

				axis_mask |= tm;  //������mask
                if(this->m_mask_pmc_axis & tm) {
                    pmc_axis_count++;   //����PMC��
                }

			 }else if(m_error_code == ERR_NONE){ //�޴˲���,ʹ�������Ӧ��λ��
				*p_target_pos = *p_source_pos;
			 }else{//�쳣
				printf("get target pos failed\n");
				return false;
			 }
		}
		tm = tm<<1;
		p_target_pos++;
		p_source_pos++;
	}

	//@ add zk G00 G01û���κ���Ч����
	//--������ϵͳ�Ƚ� ����֧�� G00 G01 �����κ� xyz����
	/*
	if(!has_valid_world){
		m_error_code = ERR_LACK_OF_PARAM;   //ȱ�ٱ�Ҫ����
		return false;
	}*/

    if(count)
        *count = pmc_axis_count;

	return true;
}

/**
 * @brief ��ȡPMC��Ŀ�������
 * @param target[out] : ����PMC���Ŀ������
 * @param axis_mask[out] �� �������ƶ���mask����־��Щ����ƶ�
 * @param inc_mode[out] : ���ص���λ���Ƿ�����ģʽ
 * @param max[in] : target����ĸ���
 * @return ������Ŀ�����ݵ�PMC�������, ����0xFF��ʾ����
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
//			continue;   //�����岹��
//		addr = static_cast<DataAddr>(m_axis_name[i]-'A');
//		axis_name_idx = chn_config->chn_axis_name[i];
//		if(m_b_axis_name_ex && chn_config->chn_axis_name_ex[i] > 0){  //����չ�±�
//			if(this->GetAxisExData(axis_name_idx, chn_config->chn_axis_name_ex[i], data)){  //������
//				if(((g_code->mask_dot_ex[axis_name_idx] & (0x01<<(chn_config->chn_axis_name_ex[i]-1))) == 0) &&
//						((g_code->mask_pos_macro[axis_name_idx] & (0x01<<(chn_config->chn_axis_name_ex[i]-1))) == 0)){  //û��С���㲢�ҷǺ���ʽ
//					data /= 1000.;   //ʡ��С��������umΪ��λ
//				}
//				*target = data;
//
//				axis_mask |= (0x01<<i);  //������mask
//
//				axis_count++;
//				target++;
//				printf("GetTargetPosEx 111, %lf, 0x%x\n", data, axis_mask);
//			}else if(m_error_code == ERR_NONE){ //�޴˲���
//	//			printf("GetTargetPosEx 222, %lf, 0x%x\n", data, axis_mask);
//			}else{//�쳣
//				printf("get pmc target pos failed\n");
//				return 0xFF;
//			}
//		}else{//����չ�±�
//			 addr = static_cast<DataAddr>(m_axis_name[i]-'A');
//			 if(GetCodeData(addr, data)){//�д˲�������ȡ�ɹ�
//				if(((g_code->mask_dot & (0x01<<addr)) == 0) &&
//						((g_code->mask_macro & (0x01<<addr)) == 0)){  //û��С���㲢�ҷǺ���ʽ
//					data /= 1000.;   //ʡ��С��������umΪ��λ
//				}
//				*target = data;
//
//				axis_mask |= (0x01<<i);  //������mask
//				axis_count++;
//				target++;
//			 }else if(m_error_code == ERR_NONE){ //�޴˲���
//
//			 }else{//�쳣
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
 * @brief ��ȡĿ���ٶ�
 * @param [out]target : ����Ŀ����ٶ�ֵ
 * @param [out]axis_mask: �������ƶ���mask����־��Щ����ƶ�
 * @return  ������Ŀ�����ݵ�PMC�������, ����0xFF��ʾ����
 */
uint8_t Parser::GetTargetSpeed(double *target, uint32_t &axis_mask, uint8_t max){
	DataAddr addr = A_DATA;
	int i = 0;
	double data = 0.0;
	uint8_t axis_count = 0;


	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
	for(i = 0; i < chn_config->chn_axis_count && axis_count < max; i++){

	//	addr = static_cast<DataAddr>(m_axis_name[i]-'A');
		if(m_b_axis_name_ex && chn_config->chn_axis_name_ex[i] > 0){  //����չ�±�
			if(this->GetAxisExData(chn_config->chn_axis_name[i], chn_config->chn_axis_name_ex[i], data)){  //������

				*target = data;

				axis_mask |= (0x01<<i);  //������mask
			}else if(m_error_code == ERR_NONE){ //�޴˲���

			}else{//�쳣
				printf("get target speed failed\n");
				return 0xFF;
			}
		}else{//����չ�±�
			 addr = static_cast<DataAddr>(m_axis_name[i]-'A');
			 if(GetCodeData(addr, data)){//�д˲�������ȡ�ɹ�

				*target = data;

				axis_mask |= (0x01<<i);  //������mask
			 }else if(m_error_code == ERR_NONE){ //�޴˲���

			 }else{//�쳣
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
 * @brief ��ȡĿ������
 * @param [out]target : ����Ŀ������ֵ
 * @param [out]axis_mask: �������ƶ���mask����־��Щ����ƶ�
 * @param max : target����������
 * @return  ������Ŀ�����ݵ�PMC�������, ����0xFF��ʾ����
 */
uint8_t Parser::GetTargetTorque(double *target, uint32_t &axis_mask, uint8_t max){
	DataAddr addr = A_DATA;
	int i = 0;
	double data = 0.0;
	uint8_t axis_count = 0;

	SCChannelConfig *chn_config = g_ptr_parm_manager->GetChannelConfig(m_n_channel_index);
	for(i = 0; i < chn_config->chn_axis_count && axis_count < max; i++){

	//	addr = static_cast<DataAddr>(m_axis_name[i]-'A');
		if(m_b_axis_name_ex && chn_config->chn_axis_name_ex[i] > 0){  //����չ�±�
			if(this->GetAxisExData(chn_config->chn_axis_name[i], chn_config->chn_axis_name_ex[i], data)){  //������

				*target = data;

				axis_mask |= (0x01<<i);  //������mask
			}else if(m_error_code == ERR_NONE){ //�޴˲���

			}else{//�쳣
				printf("get target torque failed\n");
				return 0xFF;
			}
		}else{//����չ�±�
			 addr = static_cast<DataAddr>(m_axis_name[i]-'A');
			 if(GetCodeData(addr, data)){//�д˲�������ȡ�ɹ�

				*target = data;

				axis_mask |= (0x01<<i);  //������mask
			 }else if(m_error_code == ERR_NONE){ //�޴˲���

			 }else{//�쳣
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
 * @brief ����Բ����������
 * @param src : Բ���������
 * @param tar ��Բ���յ�����
 * @param radius ��Բ���뾶
 * @param flag �� -1��ʾ˳ʱ�룬1��ʾ��ʱ��
 * @param [out]center : ����������Բ����������
 * @return true--�ɹ�   false--ʧ��
 */
bool Parser::CalArcCenter(const DPointChn &src, const DPointChn &tar, const double &radius, const int8_t flag, DPointChn &cen){
	DPlane source = Point2Plane(src, this->m_p_compiler_status->mode.gmode[2]);
	DPlane target = Point2Plane(tar, this->m_p_compiler_status->mode.gmode[2]);


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
 * @brief �ҵ��ֿ�����ƶ�ָ������÷ֿ������־
 * @param new_msg : �µ����ɵ�ָ����Ϣ
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
 * @brief �����ָ����Ϣ����������Ϣ����
 * @param macro : �ʷ�����������ĺ�ָ��
 * @return
 */
bool Parser::CreateMacroMsg(LexerMacroCmd *macro){
	//
	RecordMsg *new_msg = new MacroCmdMsg(macro);
	if(new_msg == nullptr){
		//TODO �ڴ����ʧ�ܣ��澯
		CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

//	printf("create macro message   %llu\n", this->m_p_lexer_result->line_no);
	new_msg->SetLineNo(this->m_p_lexer_result->line_no);  //���õ�ǰ�к�
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
