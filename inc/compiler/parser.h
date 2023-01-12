/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file parser.h
 *@author gonghao
 *@date 2020/04/10
 *@brief 本头文件为G代码语法解析器类的声明
 *@version
 */
#ifndef INC_COMPILER_PARSER_H_
#define INC_COMPILER_PARSER_H_

#include <stack>
#include "compiler_data.h"
#include "compile_message.h"

class Variable;   //宏变量类
class ChannelControl;   //通道控制类

const int kAxisNameBufSize = kMaxAxisChn+1;   //通道轴名称缓冲大小

class Parser {
public:
	Parser(uint8_t chn_index, LexerResult *lexer_result, CompileRecList *parser_result, CompileStatusCollect *compiler_status);  //构造函数
	virtual ~Parser();  //析构函数

	void SetMacroVar(Variable *var){this->m_p_variable = var;}   //设置宏变量对象指针

	bool Compile();     //解析数据
	ErrorType GetErrorCode(){return m_error_code;}   //获取错误码

	void Reset();     //复位状态

	void RefreshAxisName();   //刷新通道轴名称等配置数据

	void SetCompilerStatus(CompileStatusCollect *status){m_p_compiler_status = status;}   //设置编译器状态指针
	void SetLastMoveRecPointer(RecordMsg **pp){this->m_pp_last_move_rec = pp;}		//设置最后一条轴移动指令的指针的指针

	void SetParserResList(CompileRecList *rec_list){m_p_parser_result = rec_list;}   //设置语法编译结果队列指针

	bool GetExpressionResult(MacroExpression &express, MacroVarValue &res);   //计算表达式的结果

	void SetAxisNameEx(bool flag);   //设置轴名称扩展下标使能

private:
	bool CompileMacro();   //编译宏指令
	bool CompileGCode();   //编译G/M/S/T指令

	bool CheckGCode(LexerGCode *gcode);     //检查GCode合法性
	bool AnalyzeGCode(LexerGCode *gcode);   //分析GCode并生成对应的Message
	bool ProcessMCode(LexerGCode *gcode);   //处理M指令

	bool GetMacroVar(int index, double &value, bool &init);   //读取宏变量的值
	bool SetMacroVar(int index, double value, bool init = true);  //设置宏变量的值

	bool CreateErrorMsg(const ErrorType err);   //构造编译错误消息，并插入消息队列
	bool CreateInfoMsg();                    //构造信息提示消息，并插入消息队列
	bool CreateModeMsg(const int gcode);      //构造模态消息，并插入消息队列，此消息适用于无参数的模态G指令
	bool CreateRapidMsg();                //构造G00快速定位消息，并插入消息队列
	bool CreateLineMsg();                 //构造G01直线切削消息，并插入消息队列
	bool CreateArcMsg(const int gcode);       //构造圆弧切削消息，并插入消息队列
	bool CreateAuxMsg(int *mcode, uint8_t total);         //构造辅助指令消息，并插入消息队列
	bool CreateSubProgCallMsg();     //构造子程序调用消息，并插入消息队列
	bool CreateFeedMsg();			//构造进给速度指令消息，并插入消息队列
	bool CreateSpeedMsg();		//构造主轴转速指令消息，并插入消息队列
	bool CreateToolMsg(int *tcode, uint8_t total);		//构造刀具指令消息，并插入消息队列
	bool CreateCompensateMsg(int gcode);	//构造刀补指令消息，并插入消息队列
	bool CreateSpindleCheckMsg();        //构造主轴转速检测消息，并插入消息队列
	bool CreateLoopMsg(const int gcode);  	//构造循环指令消息，并插入消息队列
	bool CreateCoordMsg(const int gcode);   //构造坐标系指令消息，并插入消息队列
	bool CreateMacroMsg(LexerMacroCmd *macro);   //构造宏指令消息，并插入消息队列
	bool CreatePolarIntpMsg(const int gcode);	//构造极坐标插补相关消息
    bool CreateClearCirclePosMsg();			//构造旋转轴清整数圈位置消息
    bool CreateTimeWaitMsg();				//构造延时等待消息
    bool CreateRefReturnMsg(const int gcode);					//构造参考点返回消息
    bool CreateSkipRunMsg(const int gcode);	  //构造运行中中断跳转消息
    bool CreateMacroProgCallMsg();   //构造宏程序调用指令消息，并插入消息队列
    bool CreateAutoToolMeasureMsg();   //构造自动对刀指令消息，并插入消息队列
	bool CreateInputMsg();			   // @add zk G10 L_ P_ R_
	bool CreateExactStopMsg();         // @add zk G09 X_ Y_ Z_

#ifdef USES_SPEED_TORQUE_CTRL	
    bool CreateSpeedCtrlMsg(const int gcode);     //构造速度控制消息
    bool CreateTorqueCtrlMsg(const int gcode);     //构造力矩控制消息
#endif	

	long Double2Long(const double &data); //将double型数据转换为long型
	long BIN2BCD(long val);  //BIN转BCD
	long BCD2BIN(long val);  //BCD转BIN

	bool GetCodeData(DataAddr addr, double &data);   //获取指定地址字的数据
	bool HasCodeData(DataAddr addr);    //是否有指定地址字数据
    bool GetTargetPos(DPointChn &target, uint32_t &axis_mask, uint8_t *count = nullptr);         //获取目标点坐标
	void GetParaData(double **param, uint8_t &pc, uint32_t &mask);    //获取自变量参数
	
	bool GetAxisExData(uint8_t name, uint8_t name_ex, double &data);    //用轴扩展名称获取指令数据
//	uint8_t GetTargetPosEx(double *target, uint32_t &axis_mask, bool &inc_mode, uint8_t max);         //获取PMC轴目标点坐标

#ifdef USES_SPEED_TORQUE_CTRL	
	uint8_t GetTargetSpeed(double *target, uint32_t &axis_mask, uint8_t max);         //获取目标速度值
	uint8_t GetTargetTorque(double *target, uint32_t &axis_mask, uint8_t max);         //获取目标力矩值

//	uint8_t GetTargetSpeedEx(double *target, uint32_t &axis_mask, uint8_t max);    //获取PMC轴的目标速度
//	uint8_t GetTargetTorqueEx(double *target, uint32_t &axis_mask, uint8_t max);         //获取PMC轴的目标力矩值
#endif	

	bool CalArcCenter(const DPointChn &src, const DPointChn &tar, const double &radius, const int8_t flag, DPointChn &center);  //计算圆弧中心坐标

	bool HasAxisPos();  //是否存在轴位置指令

	void ProcessLastBlockRec(RecordMsg *new_msg);   //找到分块结束移动指令，并设置分块结束标志

	bool IsSysVar(int index);    //是否系统变量

	bool HasMacroProgCall();    //是否存在调用宏程序的指令

private:
	uint8_t m_n_channel_index;         //所属通道
	LexerResult *m_p_lexer_result;    //词法分析结果
	CompileRecList *m_p_parser_result;   //语法分析结果队列
	RecordMsg **m_pp_last_move_rec;		//当前最后一条编译的轴移动指令指针的指针
	ChannelControl *m_p_channel_control;   //所属通道指针

	ErrorType m_error_code;    //语法错误码，非语法错误直接告警，不用置位此错误码
	bool m_b_multi_mcode;     //是否允许一行多个M代码，最多3个

	SimulateMode *m_p_simulate_mode;  //仿真模式

	char m_axis_name[kAxisNameBufSize];     //通道轴名称，暂时只支持单字母轴名称
	uint8_t m_axis_name_ex[kMaxAxisChn];     //通道轴名称扩展下标
	uint64_t m_mode_mask;             //G指令模态组掩码，用于标志当前代码行出现了哪些模态组的指令，出现的对应bit置一
	int m_mode_code[kMaxGModeCount];  //当前编译行包含的有效模态指令
	bool m_b_has_g53;               //当前编译行包含G53指令

	CompileStatusCollect *m_p_compiler_status;   //编译器状态集合指针

	Variable *m_p_variable;  //变量数据指针

	bool m_b_f_code;		//是否指定了F值

	bool m_b_axis_name_ex;   //允许轴名称扩展下标

	bool m_b_call_macro_prog;   //是否需要调用宏程序

	uint32_t m_mask_pmc_axis;    //通道内PMC轴mask
	uint8_t m_n_pmc_axis_count;  //PMC轴数量
};

#endif /* INC_COMPILER_PARSER_H_ */
