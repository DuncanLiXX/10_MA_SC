/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file lexer.cpp
 *@author gonghao
 *@date 2020/04/10
 *@brief ��ͷ�ļ�ΪG����ʷ����������ʵ��
 *@version
 */
#include "global_include.h"
#include <lexer.h>

const char *MacroKeys[] = {"IF","THEN","GOTO","WHILE","DO","ELSE","ELSEIF","ENDIF","END",nullptr};  //��ָ�����˳����MacroCmd��Ӧ
const char *MacroFuncs[] = {"FIX","FUP","ROUND","ABS","SIN","COS","TAN","ASIN","ACOS","ATAN","SQRT","POW","BIN","BCD","LN",
							"EXP","EQ","NE","GT","LT","GE","LE","OR","XOR","AND","MOD",nullptr};  //�������������˳����MacroOpt��Ӧ

const uint8_t MacroOptPriority[] = {3,3,3,3,3,3,3,3,3,3,
									 3,3,3,3,3,3,0,0,0,0,
									 0,0,1,1,2,2,4,0,1,1,
									 2, 2};//����ʽ����������ȼ�������˳����MacroOpt��Ӧ

/*******************************************************MacroExpProcessor���ʵ��*******************************************************************/
/**
 * @brief ���캯��
 */
MacroExpProcessor::MacroExpProcessor(){
	this->Clear();
}

/**
 * @brief ��������
 */
MacroExpProcessor::~MacroExpProcessor(){
	this->Clear();
}

/**
 * @brief ѹ��������ַ���
 */
//bool MacroExpProcessor::PushOpt(char *c){
//	bool res = true;
//
//	return res;
//}

/**
 * @brief ѹ�������
 * @return true--�ɹ�  false--ʧ��
 */
bool MacroExpProcessor::PushOpt(MacroOpt opt){
	bool res = true;
	MacroRec rec, tmp;
	rec.opt = opt;
	rec.value = 0;
	int top_prio;
	bool bfind = false;

	if(opt == MACRO_OPT_LEFT_BRACKET){//'[',ֱ����ջ
		m_stack_opt.push(rec);
		//m_stack_prio.push(MacroOptPriority[opt]);//'['û�����ȼ�
		m_top_opt = MACRO_OPT_LEFT_BRACKET;
	}
	else if(opt == MACRO_OPT_RIGHT_BRACKET){//']',��m_stack_opt�е�ֵ���������ֱ������'['

		while(!m_stack_opt.empty()){

			tmp = m_stack_opt.top();
			if(tmp.opt == MACRO_OPT_VALUE){
				m_stack_exp.push(tmp);   //����������ʽջ
				m_stack_opt.pop();
			}
			else if(tmp.opt == MACRO_OPT_LEFT_BRACKET){//����������
				m_stack_opt.pop();
				bfind = true;
				m_b_bracket_pair = true;
				break;
			}
			else{
				m_stack_exp.push(tmp);
				m_stack_opt.pop();
				m_stack_prio.pop();
			}
		}

		if(bfind){
			//�޸�ջ�����������
			if(m_stack_opt.empty()){//������
				m_top_opt = MACRO_OPT_INVALID;
			}else{
				tmp = m_stack_opt.top();
				if(tmp.opt == MACRO_OPT_VALUE)
					res = false;  //'['���治Ӧ�ó�����ֵ
				else
					m_top_opt = tmp.opt;
			}
		}
		else
			res = false;  //���Ų�ƥ��
	}
	else if(opt == MACRO_OPT_WR){//'='
		//�������ջ������
		while(!m_stack_opt.empty()){
			tmp = m_stack_opt.top();
			if(tmp.opt == MACRO_OPT_VALUE){
				m_stack_exp.push(tmp);   //����������ʽջ
				m_stack_opt.pop();
			}
			else if(tmp.opt == MACRO_OPT_LEFT_BRACKET){//����������,�澯
				res = false;
				return res;
			}
			else{
				m_stack_exp.push(tmp);
				m_stack_opt.pop();
				m_stack_prio.pop();
			}
		}

		//��ʱm_stack_exp��ջ��Ӧ���ǡ�#������������������澯
		if(m_stack_exp.top().opt != MACRO_OPT_RD)
			res = false;
		else{
			m_stack_exp.pop();   //�滻��#��
			m_stack_opt.push(rec);
			m_stack_prio.push(MacroOptPriority[opt]);
		}
		m_top_opt = MACRO_OPT_INVALID;
		m_b_bracket_pair = false;

	}
	else if(opt == MACRO_OPT_COMMA){ //','  ����m_stack_opt�е�����ѹ��m_stack_exp�У�ֱ������'['

		while(!m_stack_opt.empty()){

			tmp = m_stack_opt.top();
			if(tmp.opt == MACRO_OPT_VALUE){
				m_stack_exp.push(tmp);   //����������ʽջ
				m_stack_opt.pop();
			}
			else if(tmp.opt == MACRO_OPT_LEFT_BRACKET){//����������
				bfind = true;
				break;
			}
			else{
				m_stack_exp.push(tmp);
				m_stack_opt.pop();
				m_stack_prio.pop();
			}
		}
		if(bfind){
			m_top_opt = MACRO_OPT_LEFT_BRACKET;
			tmp = m_stack_opt.top();
			m_stack_opt.pop();
			rec = m_stack_opt.top();
			m_stack_opt.pop();
			if(rec.opt == MACRO_OPT_ATAN || rec.opt == MACRO_OPT_POW){
				rec.value = 2;   //�޸�valueֵΪ2����ʾ˫������
			}else{
				return false;  //��ATAN��POW���������
			}
			m_stack_opt.push(rec);
			m_stack_opt.push(tmp);
		}else  //û��'['���澯
			return false;
		m_b_bracket_pair = false;
	}
	else{
		if(m_top_opt == MACRO_OPT_INVALID ||
				m_top_opt == MACRO_OPT_LEFT_BRACKET){ //ջ�������������Ϊ�����ţ�ֱ����ջ
			if(m_b_bracket_pair && IsDualOpt(opt)){
				rec.value = 1;   //��־ǰ��Ĳ����������������ջ
			}
			m_stack_opt.push(rec);
			m_stack_prio.push(MacroOptPriority[opt]);
			m_b_bracket_pair = false;
		}
		else if(m_top_opt == MACRO_OPT_ATAN && opt == MACRO_OPT_DIVIDE){ //���ݴ���ATAN���ʽ��#i=ATAN[#j]/[#k]
			rec = m_stack_opt.top();
			m_stack_opt.pop();
			if(rec.opt == MACRO_OPT_ATAN){
				rec.value = 2;   //�޸�valueֵΪ2����ʾ˫������
			}else{
				return false;
			}
			m_stack_opt.push(rec);
			m_b_bracket_pair = false;
		}
		else{
			if(m_b_bracket_pair){ //ǰһ���������ųɶ�
				if(IsDualOpt(opt))
					rec.value = 1;   //��־ǰ��Ĳ����������������ջ

			}
			if(MacroOptPriority[opt] > MacroOptPriority[m_top_opt]){  //���ȼ�����ջ��,��ֱ����ջ
				m_stack_opt.push(rec);
				m_stack_prio.push(MacroOptPriority[opt]);
				m_b_bracket_pair = false;
			}
			else if(MacroOptPriority[opt] <= MacroOptPriority[m_top_opt]){ //���ȼ�С�ڵ���ջ�������Ƚ�ջ�����ȼ����ڵ����������������
				if(IsDualOpt(opt))
					rec.value = 1;   //��־ǰ��Ĳ����������������ջ
				while(!m_stack_opt.empty()){

					tmp = m_stack_opt.top();
					if(tmp.opt == MACRO_OPT_VALUE){
						m_stack_exp.push(tmp);   //����������ʽջ
						m_stack_opt.pop();
					}
					else if(tmp.opt == MACRO_OPT_LEFT_BRACKET){//���������ţ���ջ���˳�
						break;
					}
					else{//���������
						if(!m_stack_prio.empty()){
							top_prio = m_stack_prio.top();
							if(top_prio < MacroOptPriority[opt]){//ջ�����ȼ�С���˳�
								break;
							}
						}
						m_stack_exp.push(tmp);
						m_stack_opt.pop();
						m_stack_prio.pop();
					}
				}
				m_stack_opt.push(rec);
				m_stack_prio.push(MacroOptPriority[opt]);
				m_b_bracket_pair = false;
			}
		}
		m_top_opt = opt;
	}
	return res;
}

/**
 * @brief ѹ�������
 * @return true--�ɹ�  false--ʧ��
 */
bool MacroExpProcessor::PushValue(double value){
	bool res = true;

	MacroRec rec;
	rec.opt = MACRO_OPT_VALUE;
	rec.value = value;

//	if(m_top_opt == MACRO_OPT_INVALID ||
//			m_top_opt == MACRO_OPT_LEFT_BRACKET){//���m_stack_optջ��û�������������ջ��Ϊ'['����ֱ��ѹ��m_stack_expջ
//		m_stack_exp.push(rec);
//		return res;
//	}

	m_stack_opt.push(rec);  //������ֱ����ջ
	return res;
}

/**
 * @brief �Ƿ�˫�����������
 * @param opt : �����
 * @return true--��˫������   false--����˫������
 */
bool MacroExpProcessor::IsDualOpt(MacroOpt opt){
	if((opt >= MACRO_OPT_ADD && opt <= MACRO_OPT_DIVIDE) ||
			(opt >= MACRO_OPT_EQ && opt <=MACRO_OPT_MOD)){
		return true;
	}
	return false;
}

/**
 * @brief �������ı��ʽ
 * @return true--�ɹ�  false--ʧ��
 */
bool MacroExpProcessor::GetExpression(MacroExpression &exp){
	bool res = true;
	MacroRec rec;
	while(!exp.empty())
		exp.pop(); // ��ճ�ʼ��

//	printf("get expression, size = %d\n", m_stack_opt.size());

	while(!m_stack_opt.empty()){
		rec = m_stack_opt.top();
		m_stack_opt.pop();
//		printf("get rec, opt = %d, value = %lf\n", rec.opt, rec.value);
		if(rec.opt == MACRO_OPT_LEFT_BRACKET){//����������,�澯
			res = false;
			return res;
		}
		else
		{
			m_stack_exp.push(rec);
		}
	}


	while(!m_stack_exp.empty()){
		rec = m_stack_exp.top();

	//	printf("add rec, opt = %d, value = %lf\n", rec.opt, rec.value);
		exp.push(rec);
		m_stack_exp.pop();
	}

//	printf("get macro expression over \n");

	return res;
}

/**
 * @brief �������
 */
void MacroExpProcessor::Clear(){
	while(!m_stack_opt.empty())
		m_stack_opt.pop();
	while(!m_stack_prio.empty())
		m_stack_prio.pop();
	while(!m_stack_exp.empty())
		m_stack_exp.pop();
	m_top_opt = MACRO_OPT_INVALID;
	m_b_bracket_pair = false;
}

/******************************************************************Lexer���ʵ��*********************************************************************/

/**
 * @brief �ʷ����������캯��
 * @param line_buf����������뻺��ָ��
 * @param result����ű����������ݽṹָ��
 */
Lexer::Lexer(char *line_buf, LexerResult *result) {
	// TODO Auto-generated constructor stub
	m_comp_buf = line_buf;
	m_p_lexer_result = result;
	Reset();
}

/**
 * @brief ��������
 */
Lexer::~Lexer() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief ��λ�ʷ���������״̬
 */
void Lexer::Reset(){
	m_has_domain = false;
	m_has_macro_cmd = false;
	m_has_macro_opt = false;
//	m_has_separator = false;
	m_has_dot = false;
	m_has_axis_name_ex = false;

	m_in_alph = false;
	m_in_digit = false;
	m_in_macro_exp = false;
	m_minus_flag = false;

	m_macro_cmd = MACRO_CMD_INVALID;
	m_macro_opt = MACRO_OPT_INVALID;
	m_cur_domain = '\0';
	memset(m_alph_buf, 0x00, kMaxAlphaBufLen+1);
	memset(m_digit_buf, 0x00, kMaxDigitBufLen+1);
	m_alph_index = 0;
	m_digit_index = 0;
	m_macro_exp.Clear();
//	m_error_code = ERR_NONE;
	m_macro_count = 0;
	m_n_axis_name_ex = 0;

	m_p_lexer_result->Reset();//��λ�ʷ��������

}

/**
 * @brief ��������
 * @return true--�ɹ�  false--ʧ��
 */
bool Lexer::Compile(){
	bool res = true;
	char *comp_buf = m_comp_buf;
	if(comp_buf == nullptr || this->m_p_lexer_result == nullptr){
		CreateError(ERR_COMPILER_INTER, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

    //printf("compile line:%s\n", m_comp_buf);
	while(*comp_buf != '\0'){
//		printf("lexer::compile char [%c]\n", *comp_buf);
		if(isalpha(*comp_buf)){//��ĸ
			if(m_in_digit){//������ʶ��ĵ�ֵַ��
				if(!this->ProcessValue()){
					res = false;  //�����˳�
					break;
				}
			}
			if(!m_in_alph){
				m_in_alph = true;
				m_alph_index = 0;
				memset(m_alph_buf, 0x00, kMaxAlphaBufLen+1);
			}
			if(m_alph_index < kMaxAlphaBufLen)
				m_alph_buf[m_alph_index++] = *comp_buf;
			else{//��������޷�����������
				m_p_lexer_result->error_code = ERR_TOO_LONG_WORD;
				break;
			}
		}else if(isdigit(*comp_buf) || *comp_buf == '.'){//����
			if(m_in_alph){//������
				if(!ProcessWord()){
					res = false;
					break;
				}
			}
			if(*comp_buf == '.'){
				if(m_has_dot){
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				m_has_dot = true;
			}

			if(!m_in_digit){
				m_in_digit = true;
				memset(m_digit_buf, 0x00, kMaxDigitBufLen+1);
				m_digit_index = 0;
				if(m_minus_flag){
					m_digit_buf[m_digit_index++] = '-';   //��Ӹ���
					m_minus_flag = false;  //��λ
				}
			}
			if(m_digit_index < kMaxDigitBufLen)
				m_digit_buf[m_digit_index++] = *comp_buf;
			else{//������ֵ���޷�����������
				m_p_lexer_result->error_code = ERR_TOO_LONG_DIGIT;
				break;
			}
		}else if(isspace(*comp_buf)){//�ո�

				if(m_in_digit){//����ֵ
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//������
					if(!this->ProcessWord()){
						res = false;
						break;
					}
				}
		}else{
			switch(*comp_buf){
			case '#':
				if(m_in_digit){//'#'ǰ�治������ֳ���N**��֮�������
					if(this->m_has_domain && this->m_cur_domain == 'N'){
						if(!this->ProcessValue()){
							res = false;
							break;
						}
					}else{
						printf("@@@@@@ERR_NC_FORMAT_1\n");
						m_p_lexer_result->error_code = ERR_NC_FORMAT;
						break;
					}
				}
				else if(m_in_alph){//������
					if(!this->ProcessWord()){
						res = false;
						break;
					}
				}
				if(!m_in_macro_exp){
					m_in_macro_exp = true;  //��ʼ�������ʽ
					m_macro_exp.Clear();
				}

				m_macro_exp.PushOpt(MACRO_OPT_RD);
				break;
			case '[':
				if(m_in_digit){//'['ǰ�治�����������
					printf("@@@@@@ERR_NC_FORMAT_2*\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				else if(m_in_alph){//������
					if(!this->ProcessWord()){
						res = false;
						break;
					}
				}
				if(!m_in_macro_exp){
					m_in_macro_exp = true;  //��ʼ�������ʽ
					m_macro_exp.Clear();
				}
				m_macro_exp.PushOpt(MACRO_OPT_LEFT_BRACKET);
				break;
			case '+':
				if(m_in_digit){//������ֵ
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//������
					if(!this->ProcessWord()){
						res = false;
						break;
					}
				}
				if(!m_in_macro_exp){  //�Ǳ��ʽ�У���Ϊ�����Ŵ���
					m_minus_flag = false;
					break;
				}
				m_macro_exp.PushOpt(MACRO_OPT_ADD);
				break;
			case '-':
				if(m_in_digit){//������ֵ
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//������
					if(!this->ProcessWord()){
						res = false;
						break;
					}
				}
				if(!m_in_macro_exp){  //��Ϊ�����Ŵ���
					m_minus_flag = true;
					break;
				}
				else if(!m_macro_exp.IsOptStackEmpty()  && m_macro_exp.CurTopOpt()==MACRO_OPT_LEFT_BRACKET)
				{
					m_minus_flag = true;
					break;
				}
				m_macro_exp.PushOpt(MACRO_OPT_SUB);
				break;
			case '=':
				//printf("=====%hhu, name_ex=%hhu\n", m_in_macro_exp, m_b_axis_name_ex);
				if(!m_in_macro_exp){//'='���ܳ����ڷǱ��ʽ��
					//�������±꣬������Ŀ��λ�ã� ����X1=206.567
					if(m_b_axis_name_ex){
						if(m_has_domain && m_in_digit && m_digit_index <= 2){
							this->ProcessAxisEx();
							break;
						}
					}

					printf("@@@@@@ERR_NC_FORMAT_2\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(m_in_digit){//������ֵ
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//������
					printf("@@@@@@ERR_NC_FORMAT_8\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				m_macro_exp.PushOpt(MACRO_OPT_WR);
				break;
			case ']':
				if(!m_in_macro_exp){//']'���ܳ����ڷǱ��ʽ��
					printf("@@@@@@ERR_NC_FORMAT_9\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(m_in_digit){//������ֵ
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//������
					printf("@@@@@@ERR_NC_FORMAT_10\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(!m_macro_exp.PushOpt(MACRO_OPT_RIGHT_BRACKET)){
					m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
				}else if(this->m_macro_cmd != MACRO_CMD_INVALID && m_macro_exp.IsOptStackEmpty()){ //��ָ���У����ʽ����
					res = GetOneRecord();
				}
				break;
			case '*':
				if(!m_in_macro_exp){//'*'���ܳ����ڷǱ��ʽ��
					printf("@@@@@@ERR_NC_FORMAT_3*\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(m_in_digit){//������ֵ
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//������
					m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
					res = false;
					break;
				}
				if(!m_macro_exp.PushOpt(MACRO_OPT_MULTIPLY)){
					m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
				}

				break;
			case '/':
				if(!m_in_macro_exp){  //�Ǵ������ʽ״̬���澯
					printf("@@@@@@ERR_NC_FORMAT_4*\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(m_in_digit){//������ֵ
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//������
					printf("@@@@@@ERR_NC_FORMAT_5*\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(!m_macro_exp.PushOpt(MACRO_OPT_DIVIDE))
					m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
				break;
			case ',':  //����ATAN/POW���������ֶ���
				if(!this->m_in_macro_exp){
					printf("@@@@@@ERR_NC_FORMAT_6*\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(m_in_digit){//������ֵ
					if(!this->ProcessValue()){
						break;
					}
				}
				else if(m_in_alph){//������
					printf("@@@@@@ERR_NC_FORMAT_7*\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;

				}
				if(!m_macro_exp.PushOpt(MACRO_OPT_COMMA)){
					printf("@@@@@@ERR_NC_FORMAT_8*\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
				}
				break;
			default:
				printf("lexer: unsupported code: %c\n", *comp_buf);
				m_p_lexer_result->error_code = ERR_INVALID_CODE;
				break;
			}

		}
		if(m_p_lexer_result->error_code != ERR_NONE)
			break;  //��������ѭ��

		comp_buf++;
	}

	if(m_p_lexer_result->error_code == ERR_NONE){//�޴���
		if(m_in_digit){//����ֵ
			if(!this->ProcessValue()){
				res = false;
			}
		}
		else if(m_in_alph){//������
			if(!this->ProcessWord()){
				res = false;
			}
			// ELSE   ENDIF  ����ռһ��  Ҫ���⴦��
			if(strcasecmp(comp_buf, "ELSE") == 0 or strcasecmp(comp_buf, "ENDIF")){
				m_in_macro_exp = true;
			}
		}

		if(m_p_lexer_result->error_code == ERR_NONE && m_in_macro_exp){
//			printf("process last record , m_in_macro_exp = %d, errcode = %d\n", m_in_macro_exp, m_p_lexer_result->error_code);
			res = GetOneRecord();  //�����һ������
		}
	}
	return res;
}

/**
 * @brief ����һ������
 * @return true--�ɹ�  false--ʧ��
 */
bool Lexer::GetOneRecord(){
	bool res = true;

	if(m_has_domain)
		res = GetOneGCode();
	else if(m_has_macro_cmd)
		res = GetOneMacroCmd();
	else if(m_in_macro_exp)//�����ĸ�ֵ���ʽ��
		res = GetOneMacroExp();
	else
		printf("error in getonerecord\n");

	//ͳһ��λ��־
	m_has_domain = false;
	m_has_macro_cmd = false;
	m_has_macro_opt = false;
	m_has_dot = false;
	m_has_axis_name_ex = false;
	m_in_digit = false;
	m_in_macro_exp = false;
	m_in_alph = false;
	m_n_axis_name_ex = 0;
	return res;
}

/**
 * @brief ����һ���ֵַ��
 * @return  true--�ɹ�  false--ʧ��
 */
bool Lexer::GetOneGCode(){
	bool res = true;  //�����Ƿ�ɹ�
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	int domain_index = m_cur_domain - 'A';

	m_p_lexer_result->macro_flag = false;   //�Ǻ�ָ��

//	printf("GetOneGCode, macro = %d, digit = %d, m_has_axis_name_ex=%hhu\n", m_in_macro_exp, m_in_digit, m_has_axis_name_ex);

	if(m_in_macro_exp){//��ĸ-���ʽ��
		if(m_macro_count == kMaxMacroInLine){ //����������ʽ�������澯
			m_p_lexer_result->error_code = ERR_TOO_MANY_MACRO_EXP;
		}else{
			m_macro_count++; //��Ϊ0�޷����ָ�ֵ�����Դ洢�ı��ʽ������1��ʼ
			if(this->m_has_axis_name_ex){   //�������±�
//				char names[]="XYZABCUVW";
				const char *pf = strchr(strAxisNames, this->m_cur_domain);
				int index_name = pf-strAxisNames;

				g_code->mask_pos[index_name] |= (0x01<<(m_n_axis_name_ex-1));          //��λ��Ŀ������mask
				g_code->mask_pos_macro[index_name] |= (0x01<<(m_n_axis_name_ex-1));    //��λ��Ŀ������mask
				g_code->pos_value[index_name][m_n_axis_name_ex-1] = m_macro_count-1;   //������Ŀ��λ�ñ��ʽ����

	//			printf("GetOneGCode->macro_exp, index_name=%d, mask=0x%x\n", index_name, g_code->mask_pos_macro[index_name]);

			}else{
				g_code->mask_value |= (0x01<<domain_index);
				g_code->mask_macro |= (0x01<<domain_index);
				g_code->value[domain_index] = m_macro_count-1;

	//			printf("cur domain = %c\n", m_cur_domain);

				if(m_cur_domain == 'G'){
					if(g_code->gcode_count < kMaxGCodeInLine)
						g_code->g_value[g_code->gcode_count++] = m_macro_count*-1;  //��������ĸ�ֵ
					else{//Gָ���������
						m_p_lexer_result->error_code = ERR_TOO_MANY_G_CODE;
					}
				}
				else if(m_cur_domain == 'M'){
					if(g_code->mcode_count < kMaxMCodeInLine)
						g_code->m_value[g_code->mcode_count++] = m_macro_count * -1;   //����Mָ��ֵ
					else{//Mָ���������
						m_p_lexer_result->error_code = ERR_TOO_MANY_M_CODE;
					}
				}else if(m_cur_domain == 'T'){
					if(g_code->tcode_count < kMaxTCodeInLine)
						g_code->t_value[g_code->tcode_count++] = m_macro_count * -1;   //����Tָ��ֵ
					else{//Tָ���������
						m_p_lexer_result->error_code = ERR_TOO_MANY_T_CODE;    //�澯��T��̫��
						printf("Lexer::GetOneGCode---1, too many t code! tcode_count = %hhu\n", g_code->tcode_count);
					}
				}
			}


			if(!m_macro_exp.GetExpression(g_code->macro_expression[m_macro_count-1])){//���ʽ�Ƿ�
				m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
			}

		}
		m_in_macro_exp = false;
	}
	else if(m_in_digit){//��ĸ-ֵ��
		if(this->m_has_axis_name_ex){  //�������±�
//			char names[]="XYZABCUVW";
			const char *pf = strchr(strAxisNames, this->m_cur_domain);
			int index_name = pf-strAxisNames;
			g_code->pos_value[index_name][m_n_axis_name_ex-1] = atof(m_digit_buf);   //������Ŀ��λ��
			g_code->mask_pos[index_name] |= (0x01<<(m_n_axis_name_ex-1));   //��λ��Ŀ������mask

			if(m_has_dot)
				g_code->mask_dot_ex[index_name] |= (0x01<<(m_n_axis_name_ex-1));   //��λС�����־

		}else{
			g_code->value[domain_index] = atof(m_digit_buf);
			g_code->mask_value |= (0x01<<domain_index);
			if(m_has_dot)
				g_code->mask_dot |= (0x01<<domain_index);
			if(m_cur_domain == 'G'){
				if(g_code->gcode_count < kMaxGCodeInLine)
					g_code->g_value[g_code->gcode_count++] = g_code->value[domain_index]*10;  //�Ŵ�ʮ����֧��G43.4��ָ��
				else{//Gָ���������
					m_p_lexer_result->error_code = ERR_TOO_MANY_G_CODE;
				}
			}
			else if(m_cur_domain == 'M'){
				if(g_code->mcode_count < kMaxMCodeInLine)
					g_code->m_value[g_code->mcode_count++] = g_code->value[domain_index];   //����Mָ��ֵ
				else{//Mָ���������
					m_p_lexer_result->error_code = ERR_TOO_MANY_M_CODE;
				}
			}else if(m_cur_domain == 'T'){
				if(g_code->tcode_count < kMaxTCodeInLine)
                {
					g_code->t_value[g_code->tcode_count++] = g_code->value[domain_index];   //����Tָ��ֵ
                    //std::cout << "-------> tcode_count: " << (int)g_code->tcode_count << "value: " << (int)g_code->value[domain_index] << std::endl;
                }
				else{//Tָ���������
					m_p_lexer_result->error_code = ERR_TOO_MANY_T_CODE;    //�澯��T��̫��
					printf("Lexer::GetOneGCode---2, too many t code! tcode_count = %hhu\n", g_code->tcode_count);
				}
			}
		}

		m_in_digit = false;
	}
	else{//ֻ��һ����ĸ���澯
		printf("@@@@@@ERR_NC_FORMAT_11\n");
		m_p_lexer_result->error_code = ERR_NC_FORMAT;
	}

//	printf("GetOneGCode over\n");
	m_has_domain = false;
	m_has_axis_name_ex = false;
	return res;
}

/**
 * @brief ����һ���ָ��
 * @return  true--�ɹ�  false--ʧ��
 */
bool Lexer::GetOneMacroCmd(){
	bool res = true;
	LexerMacroCmd *macro_cmd = &(m_p_lexer_result->nc_code.macro_cmd);
	MacroRec rec;

	if(!m_in_macro_exp && !m_in_digit &&
			m_macro_cmd != MACRO_CMD_DO && m_macro_cmd != MACRO_CMD_END){//û�б��ʽҲû����ֵ���澯
		printf("@@@@@@ERR_NC_FORMAT_12\n");
		m_p_lexer_result->error_code = ERR_NC_FORMAT;
		res = false;
		goto END;
	}

	if(!m_p_lexer_result->macro_flag){
		m_p_lexer_result->macro_flag = true;   //��ָ��
		macro_cmd->cmd = m_macro_cmd;

		if(m_in_macro_exp){
			if(!m_macro_exp.GetExpression(macro_cmd->macro_expression[0])){//��ʾʽ�Ƿ�
				m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
		//		res = false;
			}else
				m_macro_count++;
		}else if(m_in_digit){  //����DOm, ENDm��GOTO m

			rec.opt = MACRO_OPT_VALUE;
			rec.value = atof(m_digit_buf);
			macro_cmd->macro_expression[0].push(rec);
			m_in_digit = false;
			m_macro_count++;
		}else if(macro_cmd->cmd == MACRO_CMD_DO || macro_cmd->cmd == MACRO_CMD_END){//DO����û�и�ʶ��ţ���Ĭ��ʶ���Ϊ0
			rec.opt = MACRO_OPT_VALUE;
			rec.value = 0;
			macro_cmd->macro_expression[0].push(rec);
			m_in_digit = false;
			m_macro_count++;
		}
		else{  //��ʽ����
			printf("@@@@@@ERR_NC_FORMAT_3\n");
			m_p_lexer_result->error_code = ERR_NC_FORMAT;
		//	res = false;
		}

	}
	else{//��Ҫ�ж�IF-GOTO,IF-THEN,WHILE-DO
		if(macro_cmd->cmd == MACRO_CMD_IF && m_macro_cmd == MACRO_CMD_GOTO)
			macro_cmd->cmd = MACRO_CMD_IF_GOTO;
		else if(macro_cmd->cmd == MACRO_CMD_IF && m_macro_cmd == MACRO_CMD_THEN)
			macro_cmd->cmd = MACRO_CMD_IF_THEN;
		else if(macro_cmd->cmd == MACRO_CMD_WHILE && m_macro_cmd == MACRO_CMD_DO)
			macro_cmd->cmd = MACRO_CMD_WHILE_DO;
		else{
			printf("@@@@@@ERR_NC_FORMAT_4\n");
			m_p_lexer_result->error_code = ERR_NC_FORMAT;
		//	res = false;
		}

		if(m_in_macro_exp &&
				(macro_cmd->cmd == MACRO_CMD_IF_GOTO || macro_cmd->cmd == MACRO_CMD_IF_THEN)){
			if(!m_macro_exp.GetExpression(macro_cmd->macro_expression[1])){//��ʾʽ�Ƿ�
				m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
			}
			else
				m_macro_count++;
		}
		else if(m_in_digit &&   //IF-THEN���治��ֱ�Ӹ�����
				(macro_cmd->cmd == MACRO_CMD_IF_GOTO || macro_cmd->cmd == MACRO_CMD_WHILE_DO)){  //����DOm��GOTO m

			rec.opt = MACRO_OPT_VALUE;
			rec.value = atof(m_digit_buf);
			macro_cmd->macro_expression[1].push(rec);
			m_in_digit = false;
			m_macro_count++;
		}
		else if(macro_cmd->cmd == MACRO_CMD_WHILE_DO){//DO����û�и�ʶ��ţ���Ĭ��ʶ���Ϊ0
			rec.opt = MACRO_OPT_VALUE;
			rec.value = 0;
			macro_cmd->macro_expression[1].push(rec);
			m_in_digit = false;
			m_macro_count++;
		}
		else{  //��ʽ����
			printf("@@@@@@ERR_NC_FORMAT_13\n");
			m_p_lexer_result->error_code = ERR_NC_FORMAT;
		//	res = false;
		}

	}
	END:
	m_has_macro_cmd = false;
	m_in_macro_exp = false;
	return res;
}

/**
 * @brief ����һ�е����ĸ�ֵ���ʽ
 * @return true--�ɹ�  false--ʧ��
 */
bool Lexer::GetOneMacroExp(){
	bool res = true;
	LexerMacroCmd *macro_code = &(m_p_lexer_result->nc_code.macro_cmd);

	if(m_p_lexer_result->macro_flag){
		printf("@@@@@@ERR_NC_FORMAT_5\n");
		m_p_lexer_result->error_code = ERR_NC_FORMAT;
		goto END;
	}

	if((m_p_lexer_result->nc_code.gcode.mask_value & (~(0x01<<N_DATA))) != 0 ){  //���ˡ�N�������б�Ĵ���
		printf("@@@@@@ERR_NC_FORMAT_14\n");
		m_p_lexer_result->error_code = ERR_NC_FORMAT;
		goto END;
	}

	if(!m_macro_exp.GetExpression(macro_code->macro_expression[0])){//��ʾʽ�Ƿ�
		m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
		goto END;
	}


	m_p_lexer_result->macro_flag = true;
	macro_code->cmd = MACRO_CMD_EXP; //�����ĺ���ʽ

//	printf("get one macro expression, cmd = %d\n", macro_code->cmd);

	END:
	m_in_macro_exp = false;
	return res;
}

/**
 * @brief ����һ������
 * @return true--�ɹ�   false--ʧ��
 */
bool Lexer::ProcessWord(){
	bool res = true;

    //printf("lexer::process word, [%s]\n", m_alph_buf);

	if(!m_in_macro_exp){//�Ǳ��ʽ״̬
		if(m_has_domain){//�Ѵ�����ĸ�򣬸澯
			printf("@@@@@@ERR_NC_FORMAT_6\n");
			m_p_lexer_result->error_code = ERR_NC_FORMAT;  //�޷��������ļ�
		}
		else if(m_alph_index == 1){//��¼��ĸ��
			m_has_domain = true;
			m_cur_domain = m_alph_buf[0];
			m_in_alph = false;
		}
		else{//�ַ������ȴ���1
			m_macro_cmd = this->IsMacroKeys(m_alph_buf);  //�ж��Ƿ�Ϊ��ָ��
			if(m_macro_cmd == MACRO_CMD_INVALID){
				//�澯
				m_p_lexer_result->error_code = ERR_MACRO_FORMAT;  //�ж��Ƿ�Ϊ��ָ��
				g_ptr_trace->PrintTrace(TRACE_ERROR, COMPILER_LEXER, "�����޷������ĺ�ָ��[%s]��", m_alph_buf);
			}
			else{//�Ϸ���ָ��
				m_has_macro_cmd = true;
				m_in_alph = false;
//				printf("GET Macro cmd = %d  %s\n", m_macro_cmd, m_alph_buf);
			}
		}
	}else{//���ʽ״̬������Ϊ������
		if(m_has_domain && m_in_alph && m_alph_index == 1){//���ʽ������������ʽ
			res = GetOneRecord();

			//������ĸ��
			m_has_domain = true;
			m_cur_domain = m_alph_buf[0];
			m_in_alph = false;

		}
		else{
			m_macro_opt = this->IsMacroOpt(m_alph_buf);
			if(m_macro_opt == MACRO_OPT_INVALID){
				//�澯
				m_p_lexer_result->error_code = ERR_MACRO_OPT;  //�޷������ĺ������
				g_ptr_trace->PrintTrace(TRACE_ERROR, COMPILER_LEXER, "�����޷������ĺ������[%s]��", m_alph_buf);
			}else{ //�Ϸ��������
				m_has_macro_opt = true;
				m_macro_exp.PushOpt(m_macro_opt); //ѹ����ʽջ
				m_in_alph = false;
			}
		}

	}

	if(m_p_lexer_result->error_code != ERR_NONE){
		printf("lexer::process word, err_code = %d\n", m_p_lexer_result->error_code);
	}

	return res;
}

/**
 * @brief ����һ����ֵ
 * @return true--�ɹ�   false--ʧ��
 */
bool Lexer::ProcessValue(){
	bool res = true;

//    printf("lexer::process value\n");

	if(!m_in_macro_exp){//�Ǳ��ʽ״̬
		if(m_has_domain){//����ĸ��
			res = GetOneRecord();  //����һ������
		}
		else if(m_has_macro_cmd){ //����DOm/ENDm
			res = GetOneRecord();
		}
		else{//û����ĸ���Ҳ��ں���ʽ�У�ֱ�ӳ�����ֵ������
			printf("@@@@@@ERR_NC_FORMAT_15\n");
			m_p_lexer_result->error_code = ERR_NC_FORMAT;
		}

	}else{//���ʽ״̬������Ϊ������
		m_macro_exp.PushValue(atof(m_digit_buf)); //������ѹ����ʽջ
	}

//	printf("lexer::process value, err_code = %d\n", m_p_lexer_result->error_code);

	m_in_digit = false;
	m_has_dot = false;
//	m_has_axis_name_ex = false;
//	m_n_axis_name_ex = 0;
	return res;
}


/**
 * @brief ���������չ�±�
 * @return true--�ɹ�   false--ʧ��
 */
bool Lexer::ProcessAxisEx(){
	bool res = true;
//	char names[10] = "XYZABCUVW";

//	printf("Lexer::ProcessAxisEx(), domain = %c\n", m_cur_domain);

	if(m_has_dot){  //���ֺ���С���㣬�����ʽ
		m_p_lexer_result->error_code = ERR_NC_FORMAT;  //�޷��������ļ�
	}else{
		const char *pf = strchr(strAxisNames, this->m_cur_domain);
		if(pf == nullptr){  //���������ַ�
			m_p_lexer_result->error_code = ERR_NC_FORMAT;  //�޷��������ļ�
		}else{
			int index_ex = atoi(this->m_digit_buf);   //��չ�±�
			if(index_ex <= 0 || index_ex >16){  //�������±겻��Ϊ��ֵ
				m_p_lexer_result->error_code = ERR_NC_FORMAT;  //�޷��������ļ�
			}else{
				m_has_axis_name_ex = true;
				m_n_axis_name_ex = index_ex;
	//			printf("axis name ex : %d\n", m_n_axis_name_ex);
			}
		}
	}

	m_in_digit = false;
	m_has_dot = false;
	return res;
}

/**
 * @brief �ж��ַ����Ƿ�Ϊ��ָ��
 * @param buf:[in]�������ַ���
 * @return �ɹ��򷵻ض�Ӧ��ָ��ţ����򷵻�MACRO_CMD_INVALID
 */
MacroCmd Lexer::IsMacroKeys(char *buf){
	MacroCmd key = MACRO_CMD_INVALID;
	if(buf == nullptr)
		return key;
	int i = 0;
	while(MacroKeys[i] != nullptr){
		if(strcasecmp(buf, MacroKeys[i]) == 0){
			key = static_cast<MacroCmd>(i);
			break;
		}
		i++;
	}
	return key;
}

/**
 * @brief �ж��ַ����Ƿ�Ϊ�������
 * @param buf:[in]�������ַ���
 * @return �ɹ��򷵻ض�Ӧ�Ĳ������ţ����򷵻�MACRO_OPT_INVALID
 */
MacroOpt Lexer::IsMacroOpt(char *buf){
	MacroOpt key = MACRO_OPT_INVALID;
	if(buf == nullptr)
		return key;
	int i = 0;
	while(MacroFuncs[i] != nullptr){
		if(strcasecmp(buf, MacroFuncs[i]) == 0){
			key = static_cast<MacroOpt>(i);
			break;
		}
		i++;
	}
	return key;
}

/**
 * @brief �ж������opt�Ƿ�Ϊ���������
 * @param opt
 * @return true--��   false--����
 */
bool Lexer::IsMacroFunOpt(const MacroOpt &opt){

	return (opt>=MACRO_OPT_FIX && opt<=MACRO_OPT_EXP)?true:false;
}


