/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file Compiler.h
 *@author gonghao
 *@date 2020/03/19
 *@brief 本头文件为G代码编译器类的声明
 *@version
 */

#ifndef INC_COMPILER_COMPILER_H_
#define INC_COMPILER_COMPILER_H_

//#include "global_include.h"
#include "compiler_data.h"
#include "lexer.h"
#include "parser.h"
#include "compile_message.h"
#include "parm_definition.h"
#include "tool_compensate.h"


class ChannelControl;  //通道控制
class Variable;        //宏变量
/**
 * @brief NC代码编译器
 */
class Compiler{
public://公共接口
	Compiler(ChannelControl *chn_ctrl);
	virtual ~Compiler();

	void InitCompiler();       //编译器初始化
//	void SetFileMap(AsFileMapInfo *pFile);  //设置文件映射对象
	void Reset();               //编译器整体复位
	void ResetState();    //复位编译器状态

	void DoIdle();     //空闲处理函数


	void SetMode(CompilerWorkMode mode);   	//设置编译工作模式
	CompilerWorkMode GetMode(){return m_work_mode;}     //返回编译模式

    ErrorType GetErrorCode(){return m_error_code;}   //返回错误码

	bool IsCompileOver(){return m_b_compile_over;}  //返回是否编译结束


	bool SetCurPos(const DPoint &cur_pos);   //设置当前点位置
	bool SetCurPos(const DPointChn &cur_pos);  //设置当前点位置

	bool SetCurGMode();   //重新初始化G模态

	bool OpenFile(const char *file, bool sub_flag=false);		//打开待编译文件
	bool OpenFileInScene(const char *file);   //在保存的场景中打开文件

	bool GetLineData();       //获取一行数据

	bool CompileLine();   //编译一行代码
	bool RunMessage();    //执行编译产生的Msg

	bool CompileOver();   //编译结束处理

	void RecycleCompile();		//循环编译，处理M99指令
	void GetCurNcFile(char *file);   //获取当前加工的NC文件名称

	bool SaveScene();   //缓存编译器状态
	bool ReloadScene(bool bRecPos = true);   //恢复编译器状态

	bool IsExactStop(){return m_compiler_status.exact_stop_flag;}   //是否处于准停状态
	bool IsEndOfFile(){return m_b_eof;}    //是否到达文件尾
	bool IsPreScaning(){return compiler_lock;}  // 是否正在预扫描
	bool IsSubProgram(){return (bool)m_n_sub_program;}   	//是否在子程序中

	void SetOutputMsgList(OutputMsgList *output_msg){m_p_tool_compensate->SetOutputMsgList(output_msg);}  //设置指令消息输出缓冲区指针
	uint16_t GetCurPlane(){return m_compiler_status.mode.gmode[2];}    //获取当前平面模式

	void UnmapCurNcFile();	//关闭当前文件映射
	bool RemapCurNcFile();   //重新映射nc文件

	bool IsBlockListEmpty(){return m_p_block_msg_list->GetLength()==0?true:false;}   //分块队列是否为空
	bool RefreshBlockMovePos(DPointChn &pos);   //同步起点位置


	void SetAxisNameEx(bool flag);   //设置轴名称扩展下标使能

    void RefreshPmcAxis();

	void SetRestartPara(const uint64_t line, const uint8_t mode);    //设置加工复位参数

	void SetSimulateMode(SimulateMode *mode){this->m_p_simulate_mode = mode;}   //设置仿真模式

	int CallMarcoProgWithNoPara(int macro_index, bool flag = true);    //无参数调用宏程序

	// @add zk
	double GetFValue(){
		return m_compiler_status.mode.f_mode;
	}

    // sub_name:子程序号
    // file_only:只搜索实际文件
    int FindSubProgram(int sub_name, bool file_only = false, uint8_t scanMode = 0);   //查找并打开子程序


    int getSubCallTimes(){ return m_n_sub_call_times;}
    bool needJumpUpper(){return isJumpUpper;}

    void setCompensationPos(const DPointChn &pos);

    void GetMacroSubProgPath(int macro_group, int macro_index, bool abs_path, char *name);   //获取宏程序的路径数据

    bool m_n_cur_dir_sub_prog = false;    // 是否为当前目录下的子程序

#ifdef USES_WOOD_MACHINE
	bool FindPreStartSpdCmd(uint64_t line_min , uint64_t line_max, SpindleStartOffset &spd_cmd);   //查找是否存在可预启动的主轴指令
#endif

private://私有接口
    static void *PreScanThread(void *args);  //G代码与扫描运行线程函数
	void PreScan();     //代码预扫描，找到子程序和标签行
	void PreScanLine1(char *buf, uint64_t offset, uint64_t line_no, bool &flag, LoopOffsetStack &loop_stack, CompilerScene *scene);  //预扫描第一遍，分析行数据
	void PreScanLine2(char *buf, uint64_t offset, uint64_t line_no, bool &flag, CompilerScene *scene);  //预扫描第二遍，分析行数据
	int GetPreScanLine(char *line, uint64_t &read_total, AsFileMapInfo &map);   //获取预扫描的一行数据


//	void SwapDown();          //向下翻页

	bool RunAuxMsg(RecordMsg *msg);    	//编译器中运行辅助指令消息
	bool RunSubProgCallMsg(RecordMsg *msg);  //编译器中运行子程序调用指令消息
	bool RunLineMsg(RecordMsg *msg);   	//编译器中运行直线切削指令消息
	bool RunRapidMsg(RecordMsg *msg);  	//编译器中运行快速定位指令消息
	bool RunArcMsg(RecordMsg *msg);  		//编译器中运行圆弧切削指令消息
	bool RunCoordMsg(RecordMsg *msg);  	//编译器中运行坐标系指定指令消息
	bool RunToolMsg(RecordMsg *msg);  	//编译器中运行刀具指令消息
	bool RunModeMsg(RecordMsg *msg);  	//编译器中运行一般模态指令消息，不带参数的模态指令
	bool RunFeedMsg(RecordMsg *msg);  	//编译器中运行进给速度指定指令消息
	bool RunSpeedMsg(RecordMsg *msg);  	//编译器中运行主轴转速指令消息
	bool RunLoopMsg(RecordMsg *msg);  	//编译器中运行循环指令消息
	bool RunCompensateMsg(RecordMsg *msg);  //编译器中运行刀补指令消息
	bool RunMacroMsg(RecordMsg *msg);     //编译器中运行宏指令消息
	bool RunErrorMsg(RecordMsg *msg);  	//编译器中运行错误指令消息
	bool RunPolarIntpMsg(RecordMsg *msg);		//编译器中运行极坐标插补消息
	bool RunClearCirclePosMsg(RecordMsg *msg); 	//编译器中运行清整数圈位置消息
	bool RunTimeWaitMsg(RecordMsg *msg); 	//编译器中运行延时消息
	bool RunRefReturnMsg(RecordMsg *msg);		//编译器中运行回参考点消息
	bool RunSkipMsg(RecordMsg *msg);		   //编译器中运行跳转消息	
	bool RunMacroProgCallMsg(RecordMsg *msg);  //编译器中运行宏程序调用指令消息
	bool RunAutoToolMeasureMsg(RecordMsg *msg);   //编译器中运行自动对刀指令消息
	bool RunM06Msg(RecordMsg *msg);       //编译器中运行换刀指令消息


	bool JumpToMacroProg(int macro_index);   //跳转到指定宏程序
	
#ifdef USES_SPEED_TORQUE_CTRL	
	bool RunSpeedCtrlMsg(RecordMsg *msg);    //编译器中运行速度控制消息
	bool RunTorqueCtrlMsg(RecordMsg *msg);    //编译器中运行力矩控制消息
#endif	


	bool CreateErrorMsg(const ErrorType err, uint64_t line_no);   //构造编译错误消息，并推入下一步


	bool ProcessHead();      //处理代码头部信息
	bool ProcessMain(); //处理代码主体
	bool CompileHead();     //编译头部信息
//	bool CompileMain();      //编译主体代码
	bool ProcessJumpLine();  //处理跳段

	bool GetHeadInfo();   //解析获取头部的附加信息，包括刀路合并信息


	bool DoLexer();    //进行词法解析
	bool DoParser();   //进行语法解析
	bool GetNumber(char *ps);   //从给定字符串得到数值

	bool CheckHead();     //检查程序头部合法性
	char *GetConditionJumpMask(char *ps);  //获取跳段号
	char *GetSerialNo(char *ps);   //获取顺序号

//	char *GetGMessage(char *ps);   //生成G命令消息
//	char *GetMMessage(char *ps);   //生成M命令消息
//	char *GetSMessage(char *ps);   //生成S命令消息
//	char *GetFMessage(char *ps);   //生成F命令消息
//	char *GetTMessage(char *ps);   //生成T命令消息

	bool IsNewLineChar(char *pc);  //判断是否换行符

//	bool CallSubProgram(int sub_name);   //调用子程序
	bool ReturnFromSubProg();				//子程序返回处理

	int FindJumpLabel(int label, uint64_t &offset, uint64_t &line_no);    //查找跳转点位置

	bool JumpToLoopHead(LoopOffset &loop);    //编译跳转到循环头部位置

	bool JumpToLoopEnd(LoopOffset &loop);    //编译跳转到对应的END指令

	bool CheckJumpGoto(uint64_t line_src, uint64_t line_des);     //检查跳转的目的行的合法性

	uint8_t MapAxisNameToParamIdx(uint8_t axis_name);   //将轴名称映射为变量ID

	void SaveLoopParam(LoopMsg *loop);    //保存循环指令的参数到宏变量 根据global决定是全局变量还是局部变量
	void ResetLoopParam();                //复位循环指令参数对应的宏变量#171~#195

	bool JumpLine(int line_no, uint64_t offset, MacroCmdMsg *tmp);

#ifdef USES_FIVE_AXIS_FUNC
	void ProcessFiveAxisRotPos(DPoint &tar, DPoint &src, uint32_t mask);    //处理五轴无限旋转轴就近路径功能
	void ProcessFiveAxisRotPos(DPointChn &tar, DPointChn &src, uint32_t mask);    //处理五轴无限旋转轴就近路径功能
#endif

    void ProcessRotateAxisPos(DPointChn &tar, DPointChn &src, uint32_t mask);

private://私有成员
	uint8_t m_n_channel_index;              //所属通道号
	AsFileMapInfo *m_p_file_map_info;       //当前编译文件
//	AsFileMapInfo *m_p_file_map_info_auto;   //AUTO待编译文件
//	AsFileMapInfo *m_p_file_map_info_mda;	//MDA待编译文件

	SCChannelConfig *m_p_channel_config;   //所属通道配置
//#ifdef USES_FIVE_AXIS_FUNC
//	FiveAxisConfig *m_p_five_axis_config;  //所属通道的五轴配置
//#endif

	MCCommunication *m_p_mc_communication;   //MC通讯接口
	ChannelControl *m_p_channel_control;	//所属的通道控制

//	ToolCompensate m_tool_compensate;  //刀补处理对象
	ToolCompensate *m_p_tool_compensate;  //刀补处理对象
	ToolCompensate *m_p_tool_compensate_auto;  //刀补处理对象
	ToolCompensate *m_p_tool_compensate_mda;  //刀补处理对象

	char m_line_buf[kMaxLineSize];     //当前编译行缓冲

	char m_c_end_line;  //G代码行结束标志字符

//	OutputMsgList *m_p_output_msg_list;   //待输出至MC的指令消息队列
	OutputMsgList *m_p_block_msg_list;	//处理分段标志的缓冲队列
	RecordMsg *m_p_last_move_msg;    //当前已编译数据中最后一条轴移动指令

	OutputMsgList *m_p_block_msg_list_auto;	//AUTO模式处理分段标志的缓冲队列
	OutputMsgList *m_p_block_msg_list_mda;	//MDA模式处理分段标志的缓冲队列

	bool m_b_check;    //是否进行语法检查

	bool m_b_has_over_code;   //是否有M30/M02/M99程序结束指令，没有需要告警

	uint64_t m_ln_cur_line_no;  //当前行号，从1开始
//	uint64_t m_ln_cur_line_offset;   //当前编译行相对于文件头的偏移
	ProgramType m_n_sub_program;       //当前所处程序类型，0--主程序   1--子程序   2--宏程序
	bool m_b_comment;           //当前是否注释状态,允许跨行注释
	bool m_b_left;              //是否（）注释形式,即是否找到‘（’
	int m_n_sub_call_times{0};		//当前子程序调用次数
	ErrorType m_error_code;    //错误码

//	CompilerMainState m_n_main_state;   //编译主循环状态
	CompileFileState m_n_compile_state;   //编译状态
	CompileHeadState m_n_head_state;     //头部编译状态
//	unsigned char m_n_line_state;      //

	SimulateMode *m_p_simulate_mode;  //仿真模式

	bool isJumpUpper{false};
	bool isSubInSameFile{false};      // 子程序在同一个程序内
	uint64_t ln_read_size;
	char * p_cur_file_pos;
	uint64_t ln_cur_line_no;

	//文件操作
	char *m_p_cur_file_pos;     //当前文件读取位置指针
	uint64_t m_ln_read_size;  //当前已读取字节数
	bool m_b_eof;         //到达文件尾
	bool m_b_compile_over;     //编译结束标志，遇到了M30/M02

	LexerResult m_lexer_result;   //词法分析结果
	CompileRecList *m_p_parser_result;    //当前语法分析结果队列指针，根据模式指向auto的队列或者mda的队列

	CompileRecList *m_p_parser_res_auto;	//AUTO模式语法分析结果缓冲
	CompileRecList *m_p_parser_res_mda;		//MDA模式语法分析结果缓冲

	Lexer *m_p_lexer;         //词法解析器
	Parser *m_p_parser;       //语法解析器

	CompilerWorkMode m_work_mode;   //编译工作模式模式

	CompileStatusCollect m_compiler_status;  //编译器状态集合

	CompilerSceneStack m_stack_scene;   //场景状态保存栈

	pthread_t m_thread_prescan;   			//G代码预扫描线程
	LabelOffsetList *m_p_list_label;	//行标签号偏移位置存储队列
	SubProgOffsetList *m_p_list_subprog;	//本文件内部子程序偏移位置存储队列
	LoopRecList *m_p_list_loop;			//循环体起末点位置记录队列，预扫描时生成
	LoopOffsetStack m_stack_loop;        //循环位置存储栈，运行时使用

	/************预扫描**********************/
	// 记录当前处理的IF在主容器中的索引
	int m_node_vector_index = 0;
	// 记录 vector<IfElseOffset> 总数
	int m_node_vector_len = 0;
	// 记录ELSE出现次数
	int m_else_count_prescan = 0;
	// 主容器 每个vector<IfElseOffset> 都是一个IF.......ENDIF记录
	vector<vector<IfElseOffset>> m_node_vectors_vector;
	// 存在嵌套关系时 记录外层vector<IfElseOffset>在主容器中索引的栈
	vector<int> m_stack_vector_index_prescan;
	// 记录每个IF中 ELSE 个数的栈
	vector<int> m_stack_else_count_prescan;
	/***************运行时********************/
	// 运行时 记录当前处理的节点栈
	vector<IfElseOffset> m_node_stack_run;
	// 运行时 记录是否遇到else跳转栈
	vector<int> m_else_jump_stack_run;
	// 遇 else 跳转标志位
	bool m_b_else_jump = false;

#ifdef USES_WOOD_MACHINE
	SpindleStartList *m_p_list_spd_start;         //主轴启动命令队列

	int m_n_s_code_in_prescan;      //预扫描中记录当前S指令值
#endif
	bool m_b_prescan_over;         //预扫描结束
	bool m_b_breakout_prescan;     //中断预扫描线程标志
	bool m_b_prescan_in_stack;     //预扫描栈中的文件
	Variable *m_p_variable;        //宏变量指针
	bool m_b_axis_name_ex;         //允许轴名称扩展下标

    uint8_t m_n_restart_mode;     //加工复位模式，0--非加工复位   1--正常加工复位    2--快速加工复位
    uint64_t m_n_restart_line;    //加工复位目的行号
    char m_last_main_file[kMaxPathLen];        //最后加载的程序

    // @add zk
    bool compiler_lock = false;  // 防止子程序调用 编译先于与扫描执行
};


#endif /* INC_COMPILER_COMPILER_H_ */
