/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file lexer.cpp
 *@author gonghao
 *@date 2020/04/10
 *@brief 本头文件为G代码词法解析器类的实现
 *@version
 */
#include "global_include.h"
#include <lexer.h>

const char *MacroKeys[] = {"IF","THEN","GOTO","WHILE","DO","ELSE","ELSEIF","ENDIF","END",nullptr};  //宏指令，定义顺序于MacroCmd对应
const char *MacroFuncs[] = {"FIX","FUP","ROUND","ABS","SIN","COS","TAN","ASIN","ACOS","ATAN","SQRT","POW","BIN","BCD","LN",
							"EXP","EQ","NE","GT","LT","GE","LE","OR","XOR","AND","MOD",nullptr};  //宏运算命令，定义顺序与MacroOpt对应

const uint8_t MacroOptPriority[] = {3,3,3,3,3,3,3,3,3,3,
									 3,3,3,3,3,3,0,0,0,0,
									 0,0,1,1,2,2,4,0,1,1,
									 2, 2};//宏表达式中运算符优先级，定义顺序与MacroOpt对应

/*******************************************************MacroExpProcessor类的实现*******************************************************************/
/**
 * @brief 构造函数
 */
MacroExpProcessor::MacroExpProcessor(){
	this->Clear();
}

/**
 * @brief 析构函数
 */
MacroExpProcessor::~MacroExpProcessor(){
	this->Clear();
}

/**
 * @brief 压入操作符字符串
 */
//bool MacroExpProcessor::PushOpt(char *c){
//	bool res = true;
//
//	return res;
//}

/**
 * @brief 压入操作符
 * @return true--成功  false--失败
 */
bool MacroExpProcessor::PushOpt(MacroOpt opt){
	bool res = true;
	MacroRec rec, tmp;
	rec.opt = opt;
	rec.value = 0;
	int top_prio;
	bool bfind = false;

	if(opt == MACRO_OPT_LEFT_BRACKET){//'[',直接入栈
		m_stack_opt.push(rec);
		//m_stack_prio.push(MacroOptPriority[opt]);//'['没有优先级
		m_top_opt = MACRO_OPT_LEFT_BRACKET;
	}
	else if(opt == MACRO_OPT_RIGHT_BRACKET){//']',将m_stack_opt中的值逐个弹出，直到遇到'['

		while(!m_stack_opt.empty()){

			tmp = m_stack_opt.top();
			if(tmp.opt == MACRO_OPT_VALUE){
				m_stack_exp.push(tmp);   //操作数入表达式栈
				m_stack_opt.pop();
			}
			else if(tmp.opt == MACRO_OPT_LEFT_BRACKET){//遇到左括号
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
			//修改栈顶运算符变量
			if(m_stack_opt.empty()){//弹空了
				m_top_opt = MACRO_OPT_INVALID;
			}else{
				tmp = m_stack_opt.top();
				if(tmp.opt == MACRO_OPT_VALUE)
					res = false;  //'['后面不应该出现数值
				else
					m_top_opt = tmp.opt;
			}
		}
		else
			res = false;  //括号不匹配
	}
	else if(opt == MACRO_OPT_WR){//'='
		//逐个弹出栈内数据
		while(!m_stack_opt.empty()){
			tmp = m_stack_opt.top();
			if(tmp.opt == MACRO_OPT_VALUE){
				m_stack_exp.push(tmp);   //操作数入表达式栈
				m_stack_opt.pop();
			}
			else if(tmp.opt == MACRO_OPT_LEFT_BRACKET){//遇到左括号,告警
				res = false;
				return res;
			}
			else{
				m_stack_exp.push(tmp);
				m_stack_opt.pop();
				m_stack_prio.pop();
			}
		}

		//此时m_stack_exp的栈顶应该是‘#’运算符，如果不是则告警
		if(m_stack_exp.top().opt != MACRO_OPT_RD)
			res = false;
		else{
			m_stack_exp.pop();   //替换‘#’
			m_stack_opt.push(rec);
			m_stack_prio.push(MacroOptPriority[opt]);
		}
		m_top_opt = MACRO_OPT_INVALID;
		m_b_bracket_pair = false;

	}
	else if(opt == MACRO_OPT_COMMA){ //','  弹出m_stack_opt中的数据压入m_stack_exp中，直到遇到'['

		while(!m_stack_opt.empty()){

			tmp = m_stack_opt.top();
			if(tmp.opt == MACRO_OPT_VALUE){
				m_stack_exp.push(tmp);   //操作数入表达式栈
				m_stack_opt.pop();
			}
			else if(tmp.opt == MACRO_OPT_LEFT_BRACKET){//遇到左括号
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
				rec.value = 2;   //修改value值为2，表示双操作数
			}else{
				return false;  //非ATAN、POW运算符则告错
			}
			m_stack_opt.push(rec);
			m_stack_opt.push(tmp);
		}else  //没有'['，告警
			return false;
		m_b_bracket_pair = false;
	}
	else{
		if(m_top_opt == MACRO_OPT_INVALID ||
				m_top_opt == MACRO_OPT_LEFT_BRACKET){ //栈中无运算符或者为左括号，直接入栈
			if(m_b_bracket_pair && IsDualOpt(opt)){
				rec.value = 1;   //标志前面的操作数先于运算符入栈
			}
			m_stack_opt.push(rec);
			m_stack_prio.push(MacroOptPriority[opt]);
			m_b_bracket_pair = false;
		}
		else if(m_top_opt == MACRO_OPT_ATAN && opt == MACRO_OPT_DIVIDE){ //兼容处理ATAN表达式：#i=ATAN[#j]/[#k]
			rec = m_stack_opt.top();
			m_stack_opt.pop();
			if(rec.opt == MACRO_OPT_ATAN){
				rec.value = 2;   //修改value值为2，表示双操作数
			}else{
				return false;
			}
			m_stack_opt.push(rec);
			m_b_bracket_pair = false;
		}
		else{
			if(m_b_bracket_pair){ //前一操作符括号成对
				if(IsDualOpt(opt))
					rec.value = 1;   //标志前面的操作数先于运算符入栈

			}
			if(MacroOptPriority[opt] > MacroOptPriority[m_top_opt]){  //优先级大于栈顶,则直接入栈
				m_stack_opt.push(rec);
				m_stack_prio.push(MacroOptPriority[opt]);
				m_b_bracket_pair = false;
			}
			else if(MacroOptPriority[opt] <= MacroOptPriority[m_top_opt]){ //优先级小于等于栈顶，则先将栈中优先级大于等于它的运算符弹出
				if(IsDualOpt(opt))
					rec.value = 1;   //标志前面的操作数先于运算符入栈
				while(!m_stack_opt.empty()){

					tmp = m_stack_opt.top();
					if(tmp.opt == MACRO_OPT_VALUE){
						m_stack_exp.push(tmp);   //操作数入表达式栈
						m_stack_opt.pop();
					}
					else if(tmp.opt == MACRO_OPT_LEFT_BRACKET){//遇到左括号，入栈，退出
						break;
					}
					else{//其它运算符
						if(!m_stack_prio.empty()){
							top_prio = m_stack_prio.top();
							if(top_prio < MacroOptPriority[opt]){//栈顶优先级小则退出
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
 * @brief 压入操作数
 * @return true--成功  false--失败
 */
bool MacroExpProcessor::PushValue(double value){
	bool res = true;

	MacroRec rec;
	rec.opt = MACRO_OPT_VALUE;
	rec.value = value;

//	if(m_top_opt == MACRO_OPT_INVALID ||
//			m_top_opt == MACRO_OPT_LEFT_BRACKET){//如果m_stack_opt栈中没有运算符，或者栈顶为'['，则直接压入m_stack_exp栈
//		m_stack_exp.push(rec);
//		return res;
//	}

	m_stack_opt.push(rec);  //操作数直接入栈
	return res;
}

/**
 * @brief 是否双操作数运算符
 * @param opt : 运算符
 * @return true--是双操作数   false--不是双操作数
 */
bool MacroExpProcessor::IsDualOpt(MacroOpt opt){
	if((opt >= MACRO_OPT_ADD && opt <= MACRO_OPT_DIVIDE) ||
			(opt >= MACRO_OPT_EQ && opt <=MACRO_OPT_MOD)){
		return true;
	}
	return false;
}

/**
 * @brief 生成最后的表达式
 * @return true--成功  false--失败
 */
bool MacroExpProcessor::GetExpression(MacroExpression &exp){
	bool res = true;
	MacroRec rec;
	while(!exp.empty())
		exp.pop(); // 清空初始化

//	printf("get expression, size = %d\n", m_stack_opt.size());

	while(!m_stack_opt.empty()){
		rec = m_stack_opt.top();
		m_stack_opt.pop();
//		printf("get rec, opt = %d, value = %lf\n", rec.opt, rec.value);
		if(rec.opt == MACRO_OPT_LEFT_BRACKET){//遇到左括号,告警
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
 * @brief 清空数据
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

/******************************************************************Lexer类的实现*********************************************************************/

/**
 * @brief 词法解析器构造函数
 * @param line_buf：带编译代码缓冲指针
 * @param result：存放编译结果的数据结构指针
 */
Lexer::Lexer(char *line_buf, LexerResult *result) {
	// TODO Auto-generated constructor stub
	m_comp_buf = line_buf;
	m_p_lexer_result = result;
	Reset();
}

/**
 * @brief 析构函数
 */
Lexer::~Lexer() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief 复位词法解析器的状态
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

	m_p_lexer_result->Reset();//复位词法分析结果

}

/**
 * @brief 解析数据
 * @return true--成功  false--失败
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
		if(isalpha(*comp_buf)){//字母
			if(m_in_digit){//处理已识别的地址值对
				if(!this->ProcessValue()){
					res = false;  //出错退出
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
			else{//超长词语，无法解析，报错
				m_p_lexer_result->error_code = ERR_TOO_LONG_WORD;
				break;
			}
		}else if(isdigit(*comp_buf) || *comp_buf == '.'){//数字
			if(m_in_alph){//处理单词
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
					m_digit_buf[m_digit_index++] = '-';   //添加负号
					m_minus_flag = false;  //复位
				}
			}
			if(m_digit_index < kMaxDigitBufLen)
				m_digit_buf[m_digit_index++] = *comp_buf;
			else{//超长数值，无法解析，报错
				m_p_lexer_result->error_code = ERR_TOO_LONG_DIGIT;
				break;
			}
		}else if(isspace(*comp_buf)){//空格

				if(m_in_digit){//处理值
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//处理单词
					if(!this->ProcessWord()){
						res = false;
						break;
					}
				}
		}else{
			switch(*comp_buf){
			case '#':
				if(m_in_digit){//'#'前面不允许出现除“N**”之外的数字
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
				else if(m_in_alph){//处理单词
					if(!this->ProcessWord()){
						res = false;
						break;
					}
				}
				if(!m_in_macro_exp){
					m_in_macro_exp = true;  //开始处理宏表达式
					m_macro_exp.Clear();
				}

				m_macro_exp.PushOpt(MACRO_OPT_RD);
				break;
			case '[':
				if(m_in_digit){//'['前面不允许出现数字
					printf("@@@@@@ERR_NC_FORMAT_2*\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				else if(m_in_alph){//处理单词
					if(!this->ProcessWord()){
						res = false;
						break;
					}
				}
				if(!m_in_macro_exp){
					m_in_macro_exp = true;  //开始处理宏表达式
					m_macro_exp.Clear();
				}
				m_macro_exp.PushOpt(MACRO_OPT_LEFT_BRACKET);
				break;
			case '+':
				if(m_in_digit){//处理数值
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//处理单词
					if(!this->ProcessWord()){
						res = false;
						break;
					}
				}
				if(!m_in_macro_exp){  //非表达式中，作为正负号处理
					m_minus_flag = false;
					break;
				}
				m_macro_exp.PushOpt(MACRO_OPT_ADD);
				break;
			case '-':
				if(m_in_digit){//处理数值
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//处理单词
					if(!this->ProcessWord()){
						res = false;
						break;
					}
				}
				if(!m_in_macro_exp){  //作为正负号处理
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
				if(!m_in_macro_exp){//'='不能出现在非表达式中
					//允许轴下标，解析轴目标位置， 例如X1=206.567
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
				if(m_in_digit){//处理数值
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//处理单词
					printf("@@@@@@ERR_NC_FORMAT_8\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				m_macro_exp.PushOpt(MACRO_OPT_WR);
				break;
			case ']':
				if(!m_in_macro_exp){//']'不能出现在非表达式中
					printf("@@@@@@ERR_NC_FORMAT_9\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(m_in_digit){//处理数值
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//处理单词
					printf("@@@@@@ERR_NC_FORMAT_10\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(!m_macro_exp.PushOpt(MACRO_OPT_RIGHT_BRACKET)){
					m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
				}else if(this->m_macro_cmd != MACRO_CMD_INVALID && m_macro_exp.IsOptStackEmpty()){ //宏指令行，表达式结束
					res = GetOneRecord();
				}
				break;
			case '*':
				if(!m_in_macro_exp){//'*'不能出现在非表达式中
					printf("@@@@@@ERR_NC_FORMAT_3*\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(m_in_digit){//处理数值
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//处理单词
					m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
					res = false;
					break;
				}
				if(!m_macro_exp.PushOpt(MACRO_OPT_MULTIPLY)){
					m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
				}

				break;
			case '/':
				if(!m_in_macro_exp){  //非处理宏表达式状态，告警
					printf("@@@@@@ERR_NC_FORMAT_4*\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(m_in_digit){//处理数值
					if(!this->ProcessValue()){
						res = false;
						break;
					}
				}
				else if(m_in_alph){//处理单词
					printf("@@@@@@ERR_NC_FORMAT_5*\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(!m_macro_exp.PushOpt(MACRO_OPT_DIVIDE))
					m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
				break;
			case ',':  //对于ATAN/POW运算符会出现逗号
				if(!this->m_in_macro_exp){
					printf("@@@@@@ERR_NC_FORMAT_6*\n");
					m_p_lexer_result->error_code = ERR_NC_FORMAT;
					break;
				}
				if(m_in_digit){//处理数值
					if(!this->ProcessValue()){
						break;
					}
				}
				else if(m_in_alph){//处理单词
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
			break;  //出错跳出循环

		comp_buf++;
	}

	if(m_p_lexer_result->error_code == ERR_NONE){//无错误
		if(m_in_digit){//处理值
			if(!this->ProcessValue()){
				res = false;
			}
		}
		else if(m_in_alph){//处理单词
			if(!this->ProcessWord()){
				res = false;
			}
			// ELSE   ENDIF  独立占一行  要特殊处理
			if(strcasecmp(comp_buf, "ELSE") == 0 or strcasecmp(comp_buf, "ENDIF")){
				m_in_macro_exp = true;
			}
		}

		if(m_p_lexer_result->error_code == ERR_NONE && m_in_macro_exp){
//			printf("process last record , m_in_macro_exp = %d, errcode = %d\n", m_in_macro_exp, m_p_lexer_result->error_code);
			res = GetOneRecord();  //最后处理一组数据
		}
	}
	return res;
}

/**
 * @brief 处理一组数据
 * @return true--成功  false--失败
 */
bool Lexer::GetOneRecord(){
	bool res = true;

	if(m_has_domain)
		res = GetOneGCode();
	else if(m_has_macro_cmd)
		res = GetOneMacroCmd();
	else if(m_in_macro_exp)//单纯的赋值表达式行
		res = GetOneMacroExp();
	else
		printf("error in getonerecord\n");

	//统一复位标志
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
 * @brief 处理一组地址值对
 * @return  true--成功  false--失败
 */
bool Lexer::GetOneGCode(){
	bool res = true;  //处理是否成功
	LexerGCode *g_code = &(m_p_lexer_result->nc_code.gcode);

	int domain_index = m_cur_domain - 'A';

	m_p_lexer_result->macro_flag = false;   //非宏指令

//	printf("GetOneGCode, macro = %d, digit = %d, m_has_axis_name_ex=%hhu\n", m_in_macro_exp, m_in_digit, m_has_axis_name_ex);

	if(m_in_macro_exp){//字母-表达式对
		if(m_macro_count == kMaxMacroInLine){ //超过最大宏表达式个数，告警
			m_p_lexer_result->error_code = ERR_TOO_MANY_MACRO_EXP;
		}else{
			m_macro_count++; //因为0无法区分负值，所以存储的表达式索引从1开始
			if(this->m_has_axis_name_ex){   //轴名称下标
//				char names[]="XYZABCUVW";
				const char *pf = strchr(strAxisNames, this->m_cur_domain);
				int index_name = pf-strAxisNames;

				g_code->mask_pos[index_name] |= (0x01<<(m_n_axis_name_ex-1));          //置位轴目标数据mask
				g_code->mask_pos_macro[index_name] |= (0x01<<(m_n_axis_name_ex-1));    //置位轴目标数据mask
				g_code->pos_value[index_name][m_n_axis_name_ex-1] = m_macro_count-1;   //保存轴目标位置表达式索引

	//			printf("GetOneGCode->macro_exp, index_name=%d, mask=0x%x\n", index_name, g_code->mask_pos_macro[index_name]);

			}else{
				g_code->mask_value |= (0x01<<domain_index);
				g_code->mask_macro |= (0x01<<domain_index);
				g_code->value[domain_index] = m_macro_count-1;

	//			printf("cur domain = %c\n", m_cur_domain);

				if(m_cur_domain == 'G'){
					if(g_code->gcode_count < kMaxGCodeInLine)
						g_code->g_value[g_code->gcode_count++] = m_macro_count*-1;  //存放索引的负值
					else{//G指令个数超限
						m_p_lexer_result->error_code = ERR_TOO_MANY_G_CODE;
					}
				}
				else if(m_cur_domain == 'M'){
					if(g_code->mcode_count < kMaxMCodeInLine)
						g_code->m_value[g_code->mcode_count++] = m_macro_count * -1;   //保存M指令值
					else{//M指令个数超限
						m_p_lexer_result->error_code = ERR_TOO_MANY_M_CODE;
					}
				}else if(m_cur_domain == 'T'){
					if(g_code->tcode_count < kMaxTCodeInLine)
						g_code->t_value[g_code->tcode_count++] = m_macro_count * -1;   //保存T指令值
					else{//T指令个数超限
						m_p_lexer_result->error_code = ERR_TOO_MANY_T_CODE;    //告警，T码太多
						printf("Lexer::GetOneGCode---1, too many t code! tcode_count = %hhu\n", g_code->tcode_count);
					}
				}
			}


			if(!m_macro_exp.GetExpression(g_code->macro_expression[m_macro_count-1])){//表达式非法
				m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
			}

		}
		m_in_macro_exp = false;
	}
	else if(m_in_digit){//字母-值对
		if(this->m_has_axis_name_ex){  //轴名称下标
//			char names[]="XYZABCUVW";
			const char *pf = strchr(strAxisNames, this->m_cur_domain);
			int index_name = pf-strAxisNames;
			g_code->pos_value[index_name][m_n_axis_name_ex-1] = atof(m_digit_buf);   //保存轴目标位置
			g_code->mask_pos[index_name] |= (0x01<<(m_n_axis_name_ex-1));   //置位轴目标数据mask

			if(m_has_dot)
				g_code->mask_dot_ex[index_name] |= (0x01<<(m_n_axis_name_ex-1));   //置位小数点标志

		}else{
			g_code->value[domain_index] = atof(m_digit_buf);
			g_code->mask_value |= (0x01<<domain_index);
			if(m_has_dot)
				g_code->mask_dot |= (0x01<<domain_index);
			if(m_cur_domain == 'G'){
				if(g_code->gcode_count < kMaxGCodeInLine)
					g_code->g_value[g_code->gcode_count++] = g_code->value[domain_index]*10;  //放大十倍，支持G43.4类指令
				else{//G指令个数超限
					m_p_lexer_result->error_code = ERR_TOO_MANY_G_CODE;
				}
			}
			else if(m_cur_domain == 'M'){
				if(g_code->mcode_count < kMaxMCodeInLine)
					g_code->m_value[g_code->mcode_count++] = g_code->value[domain_index];   //保存M指令值
				else{//M指令个数超限
					m_p_lexer_result->error_code = ERR_TOO_MANY_M_CODE;
				}
			}else if(m_cur_domain == 'T'){
				if(g_code->tcode_count < kMaxTCodeInLine)
                {
					g_code->t_value[g_code->tcode_count++] = g_code->value[domain_index];   //保存T指令值
                    //std::cout << "-------> tcode_count: " << (int)g_code->tcode_count << "value: " << (int)g_code->value[domain_index] << std::endl;
                }
				else{//T指令个数超限
					m_p_lexer_result->error_code = ERR_TOO_MANY_T_CODE;    //告警，T码太多
					printf("Lexer::GetOneGCode---2, too many t code! tcode_count = %hhu\n", g_code->tcode_count);
				}
			}
		}

		m_in_digit = false;
	}
	else{//只有一个字母，告警
		printf("@@@@@@ERR_NC_FORMAT_11\n");
		m_p_lexer_result->error_code = ERR_NC_FORMAT;
	}

//	printf("GetOneGCode over\n");
	m_has_domain = false;
	m_has_axis_name_ex = false;
	return res;
}

/**
 * @brief 处理一组宏指令
 * @return  true--成功  false--失败
 */
bool Lexer::GetOneMacroCmd(){
	bool res = true;
	LexerMacroCmd *macro_cmd = &(m_p_lexer_result->nc_code.macro_cmd);
	MacroRec rec;

	if(!m_in_macro_exp && !m_in_digit &&
			m_macro_cmd != MACRO_CMD_DO && m_macro_cmd != MACRO_CMD_END){//没有表达式也没有数值，告警
		printf("@@@@@@ERR_NC_FORMAT_12\n");
		m_p_lexer_result->error_code = ERR_NC_FORMAT;
		res = false;
		goto END;
	}

	if(!m_p_lexer_result->macro_flag){
		m_p_lexer_result->macro_flag = true;   //宏指令
		macro_cmd->cmd = m_macro_cmd;

		if(m_in_macro_exp){
			if(!m_macro_exp.GetExpression(macro_cmd->macro_expression[0])){//表示式非法
				m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
		//		res = false;
			}else
				m_macro_count++;
		}else if(m_in_digit){  //处理DOm, ENDm，GOTO m

			rec.opt = MACRO_OPT_VALUE;
			rec.value = atof(m_digit_buf);
			macro_cmd->macro_expression[0].push(rec);
			m_in_digit = false;
			m_macro_count++;
		}else if(macro_cmd->cmd == MACRO_CMD_DO || macro_cmd->cmd == MACRO_CMD_END){//DO后面没有跟识别号，则默认识别号为0
			rec.opt = MACRO_OPT_VALUE;
			rec.value = 0;
			macro_cmd->macro_expression[0].push(rec);
			m_in_digit = false;
			m_macro_count++;
		}
		else{  //格式错误
			printf("@@@@@@ERR_NC_FORMAT_3\n");
			m_p_lexer_result->error_code = ERR_NC_FORMAT;
		//	res = false;
		}

	}
	else{//需要判断IF-GOTO,IF-THEN,WHILE-DO
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
			if(!m_macro_exp.GetExpression(macro_cmd->macro_expression[1])){//表示式非法
				m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
			}
			else
				m_macro_count++;
		}
		else if(m_in_digit &&   //IF-THEN后面不能直接跟常量
				(macro_cmd->cmd == MACRO_CMD_IF_GOTO || macro_cmd->cmd == MACRO_CMD_WHILE_DO)){  //处理DOm，GOTO m

			rec.opt = MACRO_OPT_VALUE;
			rec.value = atof(m_digit_buf);
			macro_cmd->macro_expression[1].push(rec);
			m_in_digit = false;
			m_macro_count++;
		}
		else if(macro_cmd->cmd == MACRO_CMD_WHILE_DO){//DO后面没有跟识别号，则默认识别号为0
			rec.opt = MACRO_OPT_VALUE;
			rec.value = 0;
			macro_cmd->macro_expression[1].push(rec);
			m_in_digit = false;
			m_macro_count++;
		}
		else{  //格式错误
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
 * @brief 处理一行单纯的赋值表达式
 * @return true--成功  false--失败
 */
bool Lexer::GetOneMacroExp(){
	bool res = true;
	LexerMacroCmd *macro_code = &(m_p_lexer_result->nc_code.macro_cmd);

	if(m_p_lexer_result->macro_flag){
		printf("@@@@@@ERR_NC_FORMAT_5\n");
		m_p_lexer_result->error_code = ERR_NC_FORMAT;
		goto END;
	}

	if((m_p_lexer_result->nc_code.gcode.mask_value & (~(0x01<<N_DATA))) != 0 ){  //除了‘N’不能有别的代码
		printf("@@@@@@ERR_NC_FORMAT_14\n");
		m_p_lexer_result->error_code = ERR_NC_FORMAT;
		goto END;
	}

	if(!m_macro_exp.GetExpression(macro_code->macro_expression[0])){//表示式非法
		m_p_lexer_result->error_code = ERR_INVALID_MACRO_EXP;
		goto END;
	}


	m_p_lexer_result->macro_flag = true;
	macro_code->cmd = MACRO_CMD_EXP; //单纯的宏表达式

//	printf("get one macro expression, cmd = %d\n", macro_code->cmd);

	END:
	m_in_macro_exp = false;
	return res;
}

/**
 * @brief 处理一个单词
 * @return true--成功   false--失败
 */
bool Lexer::ProcessWord(){
	bool res = true;

    //printf("lexer::process word, [%s]\n", m_alph_buf);

	if(!m_in_macro_exp){//非表达式状态
		if(m_has_domain){//已存在字母域，告警
			printf("@@@@@@ERR_NC_FORMAT_6\n");
			m_p_lexer_result->error_code = ERR_NC_FORMAT;  //无法解析的文件
		}
		else if(m_alph_index == 1){//记录字母域
			m_has_domain = true;
			m_cur_domain = m_alph_buf[0];
			m_in_alph = false;
		}
		else{//字符串长度大于1
			m_macro_cmd = this->IsMacroKeys(m_alph_buf);  //判断是否为宏指令
			if(m_macro_cmd == MACRO_CMD_INVALID){
				//告警
				m_p_lexer_result->error_code = ERR_MACRO_FORMAT;  //判断是否为宏指令
				g_ptr_trace->PrintTrace(TRACE_ERROR, COMPILER_LEXER, "错误：无法解析的宏指令[%s]！", m_alph_buf);
			}
			else{//合法宏指令
				m_has_macro_cmd = true;
				m_in_alph = false;
//				printf("GET Macro cmd = %d  %s\n", m_macro_cmd, m_alph_buf);
			}
		}
	}else{//表达式状态，解析为操作符
		if(m_has_domain && m_in_alph && m_alph_index == 1){//表达式结束，处理表达式
			res = GetOneRecord();

			//解析字母域
			m_has_domain = true;
			m_cur_domain = m_alph_buf[0];
			m_in_alph = false;

		}
		else{
			m_macro_opt = this->IsMacroOpt(m_alph_buf);
			if(m_macro_opt == MACRO_OPT_INVALID){
				//告警
				m_p_lexer_result->error_code = ERR_MACRO_OPT;  //无法解析的宏运算符
				g_ptr_trace->PrintTrace(TRACE_ERROR, COMPILER_LEXER, "错误：无法解析的宏运算符[%s]！", m_alph_buf);
			}else{ //合法宏操作符
				m_has_macro_opt = true;
				m_macro_exp.PushOpt(m_macro_opt); //压入表达式栈
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
 * @brief 处理一个数值
 * @return true--成功   false--失败
 */
bool Lexer::ProcessValue(){
	bool res = true;

//    printf("lexer::process value\n");

	if(!m_in_macro_exp){//非表达式状态
		if(m_has_domain){//有字母域
			res = GetOneRecord();  //处理一组数据
		}
		else if(m_has_macro_cmd){ //处理DOm/ENDm
			res = GetOneRecord();
		}
		else{//没有字母域并且不在宏表达式中，直接出现数值，报错
			printf("@@@@@@ERR_NC_FORMAT_15\n");
			m_p_lexer_result->error_code = ERR_NC_FORMAT;
		}

	}else{//表达式状态，解析为操作数
		m_macro_exp.PushValue(atof(m_digit_buf)); //将数据压入表达式栈
	}

//	printf("lexer::process value, err_code = %d\n", m_p_lexer_result->error_code);

	m_in_digit = false;
	m_has_dot = false;
//	m_has_axis_name_ex = false;
//	m_n_axis_name_ex = 0;
	return res;
}


/**
 * @brief 处理轴的扩展下标
 * @return true--成功   false--失败
 */
bool Lexer::ProcessAxisEx(){
	bool res = true;
//	char names[10] = "XYZABCUVW";

//	printf("Lexer::ProcessAxisEx(), domain = %c\n", m_cur_domain);

	if(m_has_dot){  //数字后有小数点，错误格式
		m_p_lexer_result->error_code = ERR_NC_FORMAT;  //无法解析的文件
	}else{
		const char *pf = strchr(strAxisNames, this->m_cur_domain);
		if(pf == nullptr){  //非轴名称字符
			m_p_lexer_result->error_code = ERR_NC_FORMAT;  //无法解析的文件
		}else{
			int index_ex = atoi(this->m_digit_buf);   //扩展下标
			if(index_ex <= 0 || index_ex >16){  //轴名称下标不能为负值
				m_p_lexer_result->error_code = ERR_NC_FORMAT;  //无法解析的文件
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
 * @brief 判断字符串是否为宏指令
 * @param buf:[in]待处理字符串
 * @return 成功则返回对应的指令号，否则返回MACRO_CMD_INVALID
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
 * @brief 判断字符串是否为宏运算符
 * @param buf:[in]待处理字符串
 * @return 成功则返回对应的操作符号，否则返回MACRO_OPT_INVALID
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
 * @brief 判断运算符opt是否为函数运算符
 * @param opt
 * @return true--是   false--不是
 */
bool Lexer::IsMacroFunOpt(const MacroOpt &opt){

	return (opt>=MACRO_OPT_FIX && opt<=MACRO_OPT_EXP)?true:false;
}


