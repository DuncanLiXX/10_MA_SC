/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file lexer.h
 *@author gonghao
 *@date 2020/04/10
 *@brief ��ͷ�ļ�ΪG����ʷ��������������
 *@version
 */

#ifndef INC_COMPILER_LEXER_H_
#define INC_COMPILER_LEXER_H_

#include "compiler_data.h"

/**
 * @brief ����ʽ�����࣬�����ַ�����ʽ�ĺ���ʽ��������˳��ת����ջ�����ݣ�����ִ��ģ�����ֱ������
 */
class MacroExpProcessor{
public:
	MacroExpProcessor();
	virtual ~MacroExpProcessor();

//	bool PushOpt(char *c);  //ѹ��������ַ���
	bool PushOpt(MacroOpt opt);   //ѹ�������
	bool PushValue(double value);  //ѹ�������
	bool GetExpression(MacroExpression &exp);   //�������ı��ʽ
	void Clear();            //�������

	bool IsOptStackEmpty(){return m_stack_opt.empty();}   //�����ջ�Ƿ�Ϊ��
	bool IsMacroFunOpt(const MacroOpt &opt){return (opt>=MACRO_OPT_FIX && opt<=MACRO_OPT_EXP)?true:false;}   //�ж������opt�Ƿ�Ϊ���������
	MacroOpt CurTopOpt(){return m_top_opt;}   //���ص�ǰ�����ջ��ջ�������

private:
	bool IsDualOpt(MacroOpt opt);  //�Ƿ�˫�����������
private:
	stack<MacroRec> m_stack_opt;   //������Ͳ�����ջ
	stack<uint8_t> m_stack_prio; //��������ȼ�ջ
	stack<MacroRec> m_stack_exp;    //���ʽջ

	MacroOpt m_top_opt;      //ջ�������
	bool m_b_bracket_pair;  //��һ�����������������ųɶ���
};

/**
 * @brief NC����ʷ�������
 */
class Lexer {
public:
	Lexer(char *line_buf, LexerResult *result);  //���캯��
	virtual ~Lexer();      //��������

	bool Compile();     //��������
	ErrorType GetErrorCode(){return this->m_p_lexer_result->error_code;}   //��ȡ������

	void Reset();     //��λ״̬

	void SetAxisNameEx(bool flag){this->m_b_axis_name_ex = flag;}   //������������չ�±�ʹ��

private:
	bool GetOneRecord();   //����һ������

	bool GetOneGCode();    //����һ���ֵַ��
	bool GetOneMacroCmd();   //����һ���ָ��
	bool GetOneMacroExp();     //����һ�и�ֵ���ʽ

	bool ProcessWord();       //������
	bool ProcessValue();      //������ֵ

	bool ProcessAxisEx();     //���������չ�±�

	MacroCmd IsMacroKeys(char *buf);  //�ж��ַ����Ƿ�Ϊ��ָ��
	MacroOpt IsMacroOpt(char *buf);   //�ж��ַ����Ƿ�Ϊ�������
	bool IsMacroFunOpt(const MacroOpt &opt);   //�ж������opt�Ƿ�Ϊ���������

private:
	char* m_comp_buf;               //һ�д���ı��뻺��
	bool m_has_domain;              //�Ƿ���������
//	bool m_has_data;                //�Ƿ�����ֵ
	bool m_has_macro_cmd;           //�Ƿ��ָ��
	bool m_has_macro_opt;           //�Ƿ��к������,��Ҫ����ʶ�����ʽ����������']'ʱ��λ
//	bool m_has_separator;             //�Ƿ������ָ���
	bool m_has_dot;                  //�Ƿ�����С����
	bool m_has_axis_name_ex;         //�Ƿ�������������չ

	bool m_in_alph;                   //�Ƿ��ڴ�����ĸ��
	bool m_in_digit;                  //�Ƿ��ڴ���������
	bool m_in_macro_exp;              //�Ƿ��ڴ������ʽ

	bool m_minus_flag;                 //��ֵ������,����Ϊ1������Ϊ-1

	MacroCmd m_macro_cmd;               //��ǰ���������ĺ�ָ��
	MacroOpt m_macro_opt;               //��ǰ���������ĺ������
	int m_macro_count;                  //����ʽ����
	char m_cur_domain;                 //��ǰ������������ĸֵ��
	char m_alph_buf[kMaxAlphaBufLen+1];    //���ʻ���
	char m_digit_buf[kMaxDigitBufLen+1];   //��ֵ����
	uint32_t m_alph_index;       //����λ������
	uint32_t m_digit_index;      //��ֵλ������

	int m_n_axis_name_ex;   //��������չ�±�

	MacroExpProcessor m_macro_exp;  //���ʽ������

	LexerResult *m_p_lexer_result;    //�ʷ��������

	bool m_b_axis_name_ex;   //������������չ�±�

//	ErrorType m_error_code;    //������
};

#endif /* INC_COMPILER_LEXER_H_ */
