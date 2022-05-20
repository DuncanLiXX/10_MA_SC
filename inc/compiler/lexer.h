/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file lexer.h
 *@author gonghao
 *@date 2020/04/10
 *@brief 本头文件为G代码词法解析器类的声明
 *@version
 */

#ifndef INC_COMPILER_LEXER_H_
#define INC_COMPILER_LEXER_H_

#include "compiler_data.h"

/**
 * @brief 宏表达式处理类，负责将字符串形式的宏表达式按照运算顺序转换成栈型数据，后续执行模块可以直接运算
 */
class MacroExpProcessor{
public:
	MacroExpProcessor();
	virtual ~MacroExpProcessor();

//	bool PushOpt(char *c);  //压入操作符字符串
	bool PushOpt(MacroOpt opt);   //压入操作符
	bool PushValue(double value);  //压入操作数
	bool GetExpression(MacroExpression &exp);   //生成最后的表达式
	void Clear();            //清空数据

	bool IsOptStackEmpty(){return m_stack_opt.empty();}   //运算符栈是否为空
	bool IsMacroFunOpt(const MacroOpt &opt){return (opt>=MACRO_OPT_FIX && opt<=MACRO_OPT_EXP)?true:false;}   //判断运算符opt是否为函数运算符
	MacroOpt CurTopOpt(){return m_top_opt;}   //返回当前运算符栈的栈顶运算符

private:
	bool IsDualOpt(MacroOpt opt);  //是否双操作数运算符
private:
	stack<MacroRec> m_stack_opt;   //运算符和操作数栈
	stack<uint8_t> m_stack_prio; //运算符优先级栈
	stack<MacroRec> m_stack_exp;    //表达式栈

	MacroOpt m_top_opt;      //栈顶运算符
	bool m_b_bracket_pair;  //上一个操作符，左右括号成对了
};

/**
 * @brief NC代码词法解析器
 */
class Lexer {
public:
	Lexer(char *line_buf, LexerResult *result);  //构造函数
	virtual ~Lexer();      //析构函数

	bool Compile();     //解析数据
	ErrorType GetErrorCode(){return this->m_p_lexer_result->error_code;}   //获取错误码

	void Reset();     //复位状态

	void SetAxisNameEx(bool flag){this->m_b_axis_name_ex = flag;}   //设置轴名称扩展下标使能

private:
	bool GetOneRecord();   //处理一组数据

	bool GetOneGCode();    //处理一组地址值对
	bool GetOneMacroCmd();   //处理一组宏指令
	bool GetOneMacroExp();     //处理一行赋值表达式

	bool ProcessWord();       //处理单词
	bool ProcessValue();      //处理数值

	bool ProcessAxisEx();     //处理轴的扩展下标

	MacroCmd IsMacroKeys(char *buf);  //判断字符串是否为宏指令
	MacroOpt IsMacroOpt(char *buf);   //判断字符串是否为宏操作符
	bool IsMacroFunOpt(const MacroOpt &opt);   //判断运算符opt是否为函数运算符

private:
	char* m_comp_buf;               //一行代码的编译缓冲
	bool m_has_domain;              //是否有数据域
//	bool m_has_data;                //是否有数值
	bool m_has_macro_cmd;           //是否宏指令
	bool m_has_macro_opt;           //是否有宏操作符,主要用于识别宏表达式结束，遇到']'时复位
//	bool m_has_separator;             //是否遇到分隔符
	bool m_has_dot;                  //是否遇到小数点
	bool m_has_axis_name_ex;         //是否遇到轴名称扩展

	bool m_in_alph;                   //是否在处理字母中
	bool m_in_digit;                  //是否在处理数字中
	bool m_in_macro_exp;              //是否在处理宏表达式

	bool m_minus_flag;                 //数值正负号,正号为1，负号为-1

	MacroCmd m_macro_cmd;               //当前解析出来的宏指令
	MacroOpt m_macro_opt;               //当前解析出来的宏操作符
	int m_macro_count;                  //宏表达式个数
	char m_cur_domain;                 //当前解析出来的字母值域
	char m_alph_buf[kMaxAlphaBufLen+1];    //单词缓冲
	char m_digit_buf[kMaxDigitBufLen+1];   //数值缓冲
	uint32_t m_alph_index;       //单词位置索引
	uint32_t m_digit_index;      //数值位置索引

	int m_n_axis_name_ex;   //轴名称扩展下标

	MacroExpProcessor m_macro_exp;  //表达式处理器

	LexerResult *m_p_lexer_result;    //词法分析结果

	bool m_b_axis_name_ex;   //允许轴名称扩展下标

//	ErrorType m_error_code;    //错误码
};

#endif /* INC_COMPILER_LEXER_H_ */
