/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file parser.h
 *@author gonghao
 *@date 2020/04/10
 *@brief ��ͷ�ļ�ΪG�����﷨�������������
 *@version
 */
#ifndef INC_COMPILER_PARSER_H_
#define INC_COMPILER_PARSER_H_

#include <stack>
#include "compiler_data.h"
#include "compile_message.h"

class Variable;   //�������
class ChannelControl;   //ͨ��������

const int kAxisNameBufSize = kMaxAxisChn+1;   //ͨ�������ƻ����С

class Parser {
public:
	Parser(uint8_t chn_index, LexerResult *lexer_result, CompileRecList *parser_result, CompileStatusCollect *compiler_status);  //���캯��
	virtual ~Parser();  //��������

	void SetMacroVar(Variable *var){this->m_p_variable = var;}   //���ú��������ָ��

	bool Compile();     //��������
	ErrorType GetErrorCode(){return m_error_code;}   //��ȡ������

	void Reset();     //��λ״̬

	void RefreshAxisName();   //ˢ��ͨ�������Ƶ���������

	void SetCompilerStatus(CompileStatusCollect *status){m_p_compiler_status = status;}   //���ñ�����״ָ̬��
	void SetLastMoveRecPointer(RecordMsg **pp){this->m_pp_last_move_rec = pp;}		//�������һ�����ƶ�ָ���ָ���ָ��

	void SetParserResList(CompileRecList *rec_list){m_p_parser_result = rec_list;}   //�����﷨����������ָ��

	bool GetExpressionResult(MacroExpression &express, MacroVarValue &res);   //������ʽ�Ľ��

	void SetAxisNameEx(bool flag);   //������������չ�±�ʹ��

private:
	bool CompileMacro();   //�����ָ��
	bool CompileGCode();   //����G/M/S/Tָ��

	bool CheckGCode(LexerGCode *gcode);     //���GCode�Ϸ���
	bool AnalyzeGCode(LexerGCode *gcode);   //����GCode�����ɶ�Ӧ��Message
	bool ProcessMCode(LexerGCode *gcode);   //����Mָ��

	bool GetMacroVar(int index, double &value, bool &init);   //��ȡ�������ֵ
	bool SetMacroVar(int index, double value, bool init = true);  //���ú������ֵ

	bool CreateErrorMsg(const ErrorType err);   //������������Ϣ����������Ϣ����
	bool CreateInfoMsg();                    //������Ϣ��ʾ��Ϣ����������Ϣ����
	bool CreateModeMsg(const int gcode);      //����ģ̬��Ϣ����������Ϣ���У�����Ϣ�������޲�����ģ̬Gָ��
	bool CreateRapidMsg();                //����G00���ٶ�λ��Ϣ����������Ϣ����
	bool CreateLineMsg();                 //����G01ֱ��������Ϣ����������Ϣ����
	bool CreateArcMsg(const int gcode);       //����Բ��������Ϣ����������Ϣ����
	bool CreateAuxMsg(int *mcode, uint8_t total);         //���츨��ָ����Ϣ����������Ϣ����
	bool CreateSubProgCallMsg();     //�����ӳ��������Ϣ����������Ϣ����
	bool CreateFeedMsg();			//��������ٶ�ָ����Ϣ����������Ϣ����
	bool CreateSpeedMsg();		//��������ת��ָ����Ϣ����������Ϣ����
	bool CreateToolMsg(int *tcode, uint8_t total);		//���쵶��ָ����Ϣ����������Ϣ����
	bool CreateCompensateMsg(int gcode);	//���쵶��ָ����Ϣ����������Ϣ����
	bool CreateSpindleCheckMsg();        //��������ת�ټ����Ϣ����������Ϣ����
	bool CreateLoopMsg(const int gcode);  	//����ѭ��ָ����Ϣ����������Ϣ����
	bool CreateCoordMsg(const int gcode);   //��������ϵָ����Ϣ����������Ϣ����
	bool CreateMacroMsg(LexerMacroCmd *macro);   //�����ָ����Ϣ����������Ϣ����
	bool CreatePolarIntpMsg(const int gcode);	//���켫����岹�����Ϣ
    bool CreateClearCirclePosMsg();			//������ת��������Ȧλ����Ϣ
    bool CreateTimeWaitMsg();				//������ʱ�ȴ���Ϣ
    bool CreateRefReturnMsg(const int gcode);					//����ο��㷵����Ϣ
    bool CreateSkipRunMsg(const int gcode);	  //�����������ж���ת��Ϣ
    bool CreateMacroProgCallMsg();   //�����������ָ����Ϣ����������Ϣ����
    bool CreateAutoToolMeasureMsg();   //�����Զ��Ե�ָ����Ϣ����������Ϣ����
	bool CreateInputMsg();			   // @add zk G10 L_ P_ R_
	bool CreateExactStopMsg();         // @add zk G09 X_ Y_ Z_

#ifdef USES_SPEED_TORQUE_CTRL	
    bool CreateSpeedCtrlMsg(const int gcode);     //�����ٶȿ�����Ϣ
    bool CreateTorqueCtrlMsg(const int gcode);     //�������ؿ�����Ϣ
#endif	

	long Double2Long(const double &data); //��double������ת��Ϊlong��
	long BIN2BCD(long val);  //BINתBCD
	long BCD2BIN(long val);  //BCDתBIN

	bool GetCodeData(DataAddr addr, double &data);   //��ȡָ����ַ�ֵ�����
	bool HasCodeData(DataAddr addr);    //�Ƿ���ָ����ַ������
    bool GetTargetPos(DPointChn &target, uint32_t &axis_mask, uint8_t *count = nullptr);         //��ȡĿ�������
	void GetParaData(double **param, uint8_t &pc, uint32_t &mask);    //��ȡ�Ա�������
	
	bool GetAxisExData(uint8_t name, uint8_t name_ex, double &data);    //������չ���ƻ�ȡָ������
//	uint8_t GetTargetPosEx(double *target, uint32_t &axis_mask, bool &inc_mode, uint8_t max);         //��ȡPMC��Ŀ�������

#ifdef USES_SPEED_TORQUE_CTRL	
	uint8_t GetTargetSpeed(double *target, uint32_t &axis_mask, uint8_t max);         //��ȡĿ���ٶ�ֵ
	uint8_t GetTargetTorque(double *target, uint32_t &axis_mask, uint8_t max);         //��ȡĿ������ֵ

//	uint8_t GetTargetSpeedEx(double *target, uint32_t &axis_mask, uint8_t max);    //��ȡPMC���Ŀ���ٶ�
//	uint8_t GetTargetTorqueEx(double *target, uint32_t &axis_mask, uint8_t max);         //��ȡPMC���Ŀ������ֵ
#endif	

	bool CalArcCenter(const DPointChn &src, const DPointChn &tar, const double &radius, const int8_t flag, DPointChn &center);  //����Բ����������

	bool HasAxisPos();  //�Ƿ������λ��ָ��

	void ProcessLastBlockRec(RecordMsg *new_msg);   //�ҵ��ֿ�����ƶ�ָ������÷ֿ������־

	bool IsSysVar(int index);    //�Ƿ�ϵͳ����

	bool HasMacroProgCall();    //�Ƿ���ڵ��ú�����ָ��

private:
	uint8_t m_n_channel_index;         //����ͨ��
	LexerResult *m_p_lexer_result;    //�ʷ��������
	CompileRecList *m_p_parser_result;   //�﷨�����������
	RecordMsg **m_pp_last_move_rec;		//��ǰ���һ����������ƶ�ָ��ָ���ָ��
	ChannelControl *m_p_channel_control;   //����ͨ��ָ��

	ErrorType m_error_code;    //�﷨�����룬���﷨����ֱ�Ӹ澯��������λ�˴�����
	bool m_b_multi_mcode;     //�Ƿ�����һ�ж��M���룬���3��

	SimulateMode *m_p_simulate_mode;  //����ģʽ

	char m_axis_name[kAxisNameBufSize];     //ͨ�������ƣ���ʱֻ֧�ֵ���ĸ������
	uint8_t m_axis_name_ex[kMaxAxisChn];     //ͨ����������չ�±�
	uint64_t m_mode_mask;             //Gָ��ģ̬�����룬���ڱ�־��ǰ�����г�������Щģ̬���ָ����ֵĶ�Ӧbit��һ
	int m_mode_code[kMaxGModeCount];  //��ǰ�����а�������Чģָ̬��
	bool m_b_has_g53;               //��ǰ�����а���G53ָ��

	CompileStatusCollect *m_p_compiler_status;   //������״̬����ָ��

	Variable *m_p_variable;  //��������ָ��

	bool m_b_f_code;		//�Ƿ�ָ����Fֵ

	bool m_b_axis_name_ex;   //������������չ�±�

	bool m_b_call_macro_prog;   //�Ƿ���Ҫ���ú����

	uint32_t m_mask_pmc_axis;    //ͨ����PMC��mask
	uint8_t m_n_pmc_axis_count;  //PMC������
};

#endif /* INC_COMPILER_PARSER_H_ */
