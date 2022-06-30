/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file Compiler_data.h
 *@author gonghao
 *@date 2020/03/31
 *@brief 本头文件为G代码编译器使用的数据结构的声明
 *@version
 */

#ifndef INC_COMPILER_COMPILER_DATA_H_
#define INC_COMPILER_COMPILER_DATA_H_

#include "global_include.h"

class RecordMsg;

const int kMaxTagCount = 26;     //最大地址标签数，26个英文字母
const int kMaxGCodeInLine = 16;  //一行最大G代码个数
#ifdef USES_WOOD_MACHINE
const int kMaxMCodeInLine = 16;   //一行最大M代码个数
const int kMaxTCodeInLine = 16;   //32;   //一行最大T代码个数
const int kMaxMacroInLine = 16;   //32;   //一行最大Macro表达式个数
#else
const int kMaxMCodeInLine = 3;   //一行最大M代码个数
const int kMaxTCodeInLine = 1;   //一行最大T代码个数
const int kMaxMacroInLine = 8;   //一行最大Macro表达式个数
#endif

const int kMaxLineSize = 256;   //一行G代码最大字节数

const int kMaxAxisNameCount = 9;   //通道轴名称个数，最多9个“X/Y/Z/A/B/C/U/V/W”

const int kMaxAlphaBufLen = 16;  //单词解析缓冲长度
const int kMaxDigitBufLen = 16;  //数值解析缓冲长度

extern const int kMaxFileMapSize;   //文件内存映射区最大值
extern const int kMaxGCodeCount;     //系统定义G指令最大值
extern const char *MacroKeys[];  //宏指令
extern const char *MacroFunc[];  //宏运算命令
extern const unsigned char GCode2Mode[];   //G代码模态映射，下标为G代码索引，取整数部分，比如G02和G02.3的索引都为2
extern const int kMacroParamToLocalVarIndex[]; //宏程序参数对应的局部变量索引, 顺序对应A/B/C/D/E...X/Y/Z，0表示非法
extern const int kLoopParamToLocalVarIndex[];  //循环指令参数对应的局部变量索引, 顺序对应A/B/C/D/E...X/Y/Z，0表示非法， 相对宏程序自变量增加了P参数
extern const int kLoopParamToGlobalVarIndex[]; //循环指令参数对应全局变量索引，顺序对应A/B/C/D/E...X/Y/Z，0表示非法


//编译器线程状态
enum CompilerState{
    IDLE = 0,   		//空闲
    RUN,    			//运行
	WAIT_EXECUTE,		//等待指令execute执行结束
	WAIT_RUN,			//等待指令RUN运行结束
    PAUSE,    			//暂停
    STOP,     			//停止
    ERROR     			//错误
};


//编译主循环状态
//enum CompilerMainState{
//    doEnd        =   0,
//    doRead       =   1,
//    doCompile    =   2,
//    doHead       =   3,
//    doHeadInfo   =   4,
//    doSubprogram =   5,
//    doMain       =   6,
//    doLineReady  =   7,
//    doLineStart  =   8,
//    doMacro      =   9,
//    doFind       =   10,
//    doReadSub    =   11,
//    doCallSub    =   12,
//    doJumpReady  =   13,
//    doJumpStart  =   14,
//    doCompiPause =   15,
//    doCompiWaitResume = 16
//};

//编译文件状态
enum CompileFileState{
    FILE_HEAD = 0,    //编译文件头部信息
    FILE_MAIN,        //编译文件代码主体
    FILE_FIND         //定位文件子函数位置
};

//文件头部编译状态
enum CompileHeadState{
    HEAD_INFO = 0,   //头部信息
    SUBPROGRAM       //子程序
};

//编译器工作模式
enum CompilerWorkMode{
	AUTO_COMPILER = 1,  //自动编译器模式
	MDA_COMPILER		//MDA编译器模式
};

enum SimulateMode{
	SIM_NONE = 0,		//非仿真模式
	SIM_OUTLINE,		//轮廓仿真
	SIM_TOOLPATH,		//刀路仿真
	SIM_MACHINING		//加工仿真
};

//指令消息类型
enum CodeMsgType{
	NORMAL_MSG = 0,		//0	普通消息
	ERROR_MSG = 2,  		// 2	错误消息
	LINE_MSG = 4,      		// 4 G01消息
	ARC_MSG,       		//5	G02/G03消息
	REF_RETURN_MSG = 8,     //8 返回参考点消息
	RAPID_MSG = 10,    		//10 G00消息
	FEED_MSG,      		//11 进给速度消息
	SPEED_MSG,          //12 主轴转速消息

	TOOL_MSG = 14,      		//14 刀具消息
	CBS_MSG,        	//15 样条消息
	MODE_MSG,			//16 模式消息
	AUX_MSG,			//17 M指令消息
	SPD_CHK_MSG,        //18 主轴转速检测消息
	COMPENSATE_MSG,		//19 刀补消息
	LOOP_MSG,			//20 循环指令消息
	COORD_MSG,          //21 坐标系消息
	SUBPROG_CALL_MSG,   //22 子程序调用消息
	MACRO_MSG,          //23 宏指令消息
	SUBPROG_RETURN_MSG, //24 子程序返回消息
	POLAR_INTP_MSG,		//25 极坐标插补类消息
	CLEAR_CIRCLE_POS_MSG, //26 清整数圈位置消息
	TIME_WAIT_MSG,		//27 延时消息
	SPEED_CTRL_MSG,     //28 速度指令消息
	TORQUE_CTRL_MSG,    //29 力矩指令消息
	SKIP_MSG,            //30 跳转指令信息
	MACRO_PROG_CALL_MSG,  //31 宏程序调用消息
	AUTO_TOOL_MEASURE_MSG, //32  自动对刀指令消息
	RESTART_OVER_MSG,     //33   加工复位完成消息
	MOVE_MSG,             //34   移动指令基类消息
	GUARD_MSG           //消息类型卫兵
};

//定义宏指令操作
enum MacroOpt{
	MACRO_OPT_INVALID = -1, //非法操作符

	//函数
	MACRO_OPT_FIX = 0,  //下取整，舍去小数部分
	MACRO_OPT_FUP,		//上取整，将小数部分进位到整数部分
	MACRO_OPT_ROUND,	//四舍五入，在算数或逻辑运算指令中使用时，在第1个小数位置四舍五入，在NC语句地址中使用时，根据地址最小设定单位四舍五入
	MACRO_OPT_ABS,		//绝对值
	MACRO_OPT_SIN,		//正弦
	MACRO_OPT_COS,		//余弦
	MACRO_OPT_TAN,		//正切
	MACRO_OPT_ASIN,		//反正弦
	MACRO_OPT_ACOS,		//反余弦
	MACRO_OPT_ATAN,		//反正切，支持三种格式：#i=ATAN[#j]   #i=ATAN[#j]/[#k]    #i=ATAN[#j,#k]
	MACRO_OPT_SQRT,		//平方根
	MACRO_OPT_POW,      //乘方  #1=POW[#2,#3]
	MACRO_OPT_BIN,      //从BCD码转为二进制码   #1=BIN[#2]
	MACRO_OPT_BCD,      //从二进制码转为BCD码   #1=BCD[#2]
	MACRO_OPT_LN,		//自然对数
	MACRO_OPT_EXP,		//指数函数
	MACRO_OPT_EQ,		//等于
	MACRO_OPT_NE,		//不等于
	MACRO_OPT_GT,		//大于
	MACRO_OPT_LT,		//小于
	MACRO_OPT_GE,		//大于等于
	MACRO_OPT_LE,		//小于等于
	MACRO_OPT_OR,		//或
	MACRO_OPT_XOR,		//异或
	MACRO_OPT_AND,		//与
	MACRO_OPT_MOD,   	//求余数  #i=#j MOD #k (#j、#k取整后求取余数，#j为负时，#i也为负)

	//符号
	MACRO_OPT_RD,       //读取，#
	MACRO_OPT_WR,       //赋值
	MACRO_OPT_ADD,      //加
	MACRO_OPT_SUB,      //减
	MACRO_OPT_MULTIPLY, //乘
	MACRO_OPT_DIVIDE,   //除


	//非运算符
	MACRO_OPT_LEFT_BRACKET,  //左括号
	MACRO_OPT_RIGHT_BRACKET, //右括号
	MACRO_OPT_VALUE,    //操作数

	MACRO_OPT_COMMA,    //逗号，适用于ATAN/POW运算符
	MACRO_OPT_GUARD		//卫兵
};

//定义宏命令
enum MacroCmd{
	MACRO_CMD_INVALID = -1,  //非法宏命令
	MACRO_CMD_IF = 0,
	MACRO_CMD_THEN,
	MACRO_CMD_GOTO,		    //强制跳转
	MACRO_CMD_WHILE,
	MACRO_CMD_DO,
	MACRO_CMD_ELSE,
	MACRO_CMD_ELSEIF,
	MACRO_CMD_ENDIF,
	MACRO_CMD_END,          //循环尾部
	MACRO_CMD_IF_GOTO,		//条件跳转
	MACRO_CMD_IF_THEN,		//条件执行
	MACRO_CMD_WHILE_DO,     //循环
	MACRO_CMD_EXP,			//纯表达式行
	MACRO_CMD_GUARD			//卫兵
};

//G代码模式组定义
enum GModeIndex{
	GMODE_NONE = 0x00,              //无
	GMODE_00 = 0X01, 				//00组
	GMODE_01 = 0X02,				//01组
	GMODE_02 = 0X04,				//02组
	GMODE_03 = 0X08,
	GMODE_04 = 0X10,
	GMODE_05 = 0X20,
	GMODE_06 = 0X40,
	GMODE_07 = 0X80,
	GMODE_08 = 0X0100,
	GMODE_09 = 0X0200,
	GMODE_10 = 0X0400,
	GMODE_11 = 0X0800,
	GMODE_12 = 0X1000,
	GMODE_13 = 0X2000,
	GMODE_14 = 0X4000,
	GMODE_15 = 0X8000,
	GMODE_16 = 0X010000,
	GMODE_17 = 0X020000,
	GMODE_18 = 0X040000,
	GMODE_19 = 0X080000,
	GMODE_20 = 0X100000,
	GMODE_21 = 0X200000,
	GMODE_22 = 0X400000,
	GMODE_23 = 0X800000,
	GMODE_24 = 0X01000000,
	GMODE_25 = 0X02000000,
	GMODE_26 = 0X04000000,
	GMODE_27 = 0X08000000,
	GMODE_28 = 0X10000000,
	GMODE_29 = 0X20000000,
	GMODE_30 = 0X40000000,
	GMODE_31 = 0X80000000,
	GMODE_32 = 0X0100000000,
	GMODE_33 = 0X0200000000,		//33组
	GMODE_34 = 0X0400000000,		//34组
	GMODE_35 = 0X0800000000,		//35组
	GMODE_36 = 0x1000000000,		//36组
	GMODE_37 = 0x2000000000,		//37组
	GMODE_38 = 0x4000000000,        //38组
	GMODE_39 = 0x8000000000         //39组
};



//G指令数据地址定义
enum DataAddr{
	A_DATA = 0,
	B_DATA,
	C_DATA,
	D_DATA,
	E_DATA,
	F_DATA,
	G_DATA,
	H_DATA,
	I_DATA,
	J_DATA,
	K_DATA,
	L_DATA,
	M_DATA,
	N_DATA,
	O_DATA,
	P_DATA,
	Q_DATA,
	R_DATA,
	S_DATA,
	T_DATA,
	U_DATA,
	V_DATA,
	W_DATA,
	X_DATA,
	Y_DATA,
	Z_DATA
};

//所处程序类型
enum ProgramType{
	MAIN_PROG = 0,    	//主程序
	SUB_PROG,			//子程序
	MACRO_PROG			//宏程序
};

//宏指令单元，宏变量表达式由其组合而成
struct MacroRec{
	MacroOpt opt;          //运算符
    double value;      //操作数，详细说明如下
    					//1. 对于操作数其保存值；
    					//2. 对于运算符，其值常规为0；
    					//3. 对于ATAN/POW运算符，value=2表示为两个操作数
    					//4. 对于+、-、*、/、AND、MOD、OR、XOR这些具有两个操作数的运算符，value=1表示其前面的操作数具有高优先级先入栈了

    MacroRec& operator=( const MacroRec& rec);  //赋值运算符
    friend bool operator ==( const MacroRec &one, const MacroRec &two);  //判断运算符
    friend bool operator !=(const MacroRec &one, const MacroRec &two);    //
};
typedef stack<MacroRec> MacroExpression;  //宏表达式类型

void CopyMacroExp(MacroExpression &des, MacroExpression &src);   //拷贝赋值宏表达式堆栈
bool IsEqualMacroExp(MacroExpression &one, MacroExpression &two);   //比较两个宏表达式是否一致

//一行G指令代码的词法分析结果，
struct LexerGCode{
	void Reset();
	uint32_t mask_value;           //数据字段掩码，标记一行中出现了哪些数据段，总计32bit，使用前26bit，根据字母顺序排列“A=0,B=1,C=2,D=3......X=23,Y=24,Z=25”
	uint32_t mask_macro;           //标记哪些数据字为宏表达式
	uint32_t mask_dot;             //标记哪些字段的数据带小数点
	double value[kMaxTagCount];  //保存数据字段的值，如果是宏表达式则存储macro_expression中的索引号
	int g_value[kMaxGCodeInLine];   //记录一行中出现的G代码值，此处存放的G代码放大十倍以兼容G43.4类指令,如果是宏表达式则存储
								   //macro_expression中的索引号的负值,索引号从1开始
	int m_value[kMaxMCodeInLine];   //记录一行中出现的M代码值，如果是宏表达式则存储macro_expression中的索引号的负值,索引号从1开始
	uint8_t gcode_count;          //一行中的G代码个数
	uint8_t mcode_count;          //一行中的M代码个数

	int t_value[kMaxTCodeInLine];    //记录一行中出现的T代码值，如果是宏表达式则存储macro_expression中的索引号的负值,索引号从1开始
	uint8_t tcode_count;          //一行中T代码的个数

	double pos_value[kMaxAxisNameCount][16];    //带下标的轴目标位置数据,[X/Y/Z/A/B/C/U/V/W][1~9]
	uint16_t mask_pos[kMaxAxisNameCount];        //带下标的轴目标数据掩码:X=0,Y=1,Z=2,A=3,B=4,C=5,U=6,V=7,W=8, BIT0~BIT15代表下标1~16
	uint16_t mask_pos_macro[kMaxAxisNameCount];  //标记哪些扩展轴名称数据字为宏表达式
	uint16_t mask_dot_ex[kMaxAxisNameCount];             //标记哪些轴名称扩展字段的数据带小数点

	MacroExpression macro_expression[kMaxMacroInLine];  //记录宏表达式

};


//一行宏指令词法分析结果,IF/WHILE/GOTO
struct LexerMacroCmd{
	void Reset();
	MacroCmd cmd;		//宏指令类型
//	bool is_const;     //是否常量数据
	MacroExpression macro_expression[2];      //宏表达式
//	int data;          //数据，用于保存循环编号或者跳转编号
};

//NC代码解析结果
struct NcCode{
	NcCode(){}
	~NcCode(){}
	void Reset();
	LexerGCode gcode;          //G指令结果
	LexerMacroCmd macro_cmd;   //宏指令结果
};

//一行代码的词法分析结果
struct LexerResult{
	LexerResult(){}
	~LexerResult(){}
	void Reset();
	uint64_t line_no;         		//行号
	uint64_t offset;   				//行起始偏移，用于循环指令的跳转
	NcCode nc_code;				    //G代码
	bool macro_flag;               //是否宏指令行,带有IF/WHILE/GOTO等宏指令
	ErrorType error_code;    		//错误码
};

//模态集合，集中管理，方便访问
struct ModeCollect{
	void Initialize();  //初始化函数
	void Reset();        //复位操作
	ModeCollect& operator=( const ModeCollect& c);


	uint16_t gmode[kMaxGModeCount];   //G代码模态
	double f_mode;		//F代码模态值   单位：mm/min
	double s_mode;		//S代码模态值   单位：rpm
//	uint8_t move_mode;  //轴移动模态，1：01组模态  9:09组模态
	uint8_t h_mode;		//H代码模态值,掉电保存
	uint8_t d_mode;		//D代码模态值,掉电保存
	uint8_t t_mode;		//T代码模态值,掉电保存


};



//编译状态集,集合管理编译过程中的各种状态，包括模式、刀具、各轴当前位置等
struct CompileStatusCollect{
	ModeCollect mode;			//当前模态
	DPointChn cur_pos;             //当前轴位置
	
	//速度控制和力矩控制不需要记录当前的目标速度和目标力矩
//#ifdef USES_SPEED_TORQUE_CTRL
//	DPoint cur_velocity;             //当前轴速度
//	DPoint cur_torque;             //当前轴力矩
//#endif

//	double cur_feed;			//当前进给速度 单位mm/min

	bool exact_stop_flag;    	//准停标志   false-非准停     true--准停
	bool jump_flag;        		//是否包含跳段


	CompileStatusCollect& operator=( const CompileStatusCollect& c);
};

/**
* @brief：AS文件信息结构，增加了对超大文件的支持
*/
struct AsFileMapInfo
{
    AsFileMapInfo()
    {
    	Clear();
    }
    ~AsFileMapInfo()
    {
//        if(ptr_map_file){
//        //	printf("unmap file:0x%x\n", ptr_map_file);
//            munmap(ptr_map_file, ln_map_blocksize);   //取消映射
//			ptr_map_file = nullptr;
//        }
//        if(fp_cur > 0)
//        	close(fp_cur);
    }
    void Clear();    //恢复默认值，不关闭文件
    bool OpenFile(const char *name, bool sub_flag=false);  //打开文件
    void CloseFile();    //关闭当前文件
    void UnmapFile();		//关闭文件映射
    bool RemapFile();		//重新映射文件
    bool Swapdown();     //向下翻页
    bool Swapup();		   //向上翻页
    bool ResetFile();         //复位到当前文件头
    bool JumpTo(uint64_t pos);   //将当前读取位置跳转到pos处
    AsFileMapInfo& operator=( const AsFileMapInfo& c);


    //文件参数
    int fp_cur;    //当前操作的文件句柄
    uint64_t ln_file_size;   //文件大小
//    uint64_t ln_line_count;  //文件总行数
    char str_file_name[kMaxPathLen];  //文件名称

    //内存映射区变量
    uint64_t ln_map_start;      //映射缓冲起始位置对应文件偏移
    uint64_t ln_map_blocksize;    //映射区块大小
    //uint64_t ln_map_start_line;  //映射区起始行号
    char *ptr_map_file;       //内存映射区指针
};

/**
 * @brief 标签位置记录
 */
struct LabelOffset{
	int label;		//标签
	uint64_t offset;   //位置偏移
	uint64_t line_no;	//行号

	LabelOffset& operator=( const LabelOffset& c);
	bool operator ==( const LabelOffset &one){  //判断运算符
		if(one.label == this->label)
			return true;
		return false;
	}
};
typedef ListBuffer<LabelOffset> LabelOffsetList;   //标签位置队列



/**
 * @brief 子程序偏移位置记录
 */
struct SubProgOffset{
	int sub_index;		//子程序号
	uint64_t offset;    //位置偏移
	uint64_t line_no;	//行号

	SubProgOffset& operator=( const SubProgOffset& c);
	bool operator ==( const SubProgOffset &one){  //判断运算符
		if(one.sub_index == this->sub_index)
			return true;
		return false;
	}
};
typedef ListBuffer<SubProgOffset> SubProgOffsetList;     //子程序位置队列

/**
 * @brief IF ELSE OFFSET
 */
struct IfElseOffset{
	uint64_t offset;     //文件指针偏移
	uint64_t line_no;	 //行号
	uint16_t vec_index;  // 此节点链表在 vector中的索引
	uint16_t node_index; //此节点在 ifelse 链表中的位置

	IfElseOffset& operator=( const IfElseOffset& c);
	bool operator ==( const IfElseOffset &one){  //判断运算符
		if(one.offset == this->offset and one.line_no == this->line_no)
			return true;
		return false;
	}
};

typedef ListBuffer<IfElseOffset> IfElseOffsetList;


/**
 * @brief 循环体起末点位置记录
 */
struct LoopRec{
	uint64_t start_offset;   //起点位置偏移
	uint64_t start_line_no;	//起点行号
	uint64_t end_offset;	//结束位置偏移
	uint64_t end_line_no;	//结束位置行号

	LoopRec& operator=( const LoopRec& c);
	bool operator ==( const LoopRec &one){  //判断运算符
		if(one.start_line_no == this->start_line_no &&
				one.end_line_no ==  this->end_line_no)
			return true;
		return false;
	}
};
typedef ListBuffer<LoopRec> LoopRecList;    //循环体位置队列

/**
 * @brief  循环体的起始位置记录
 */
struct LoopOffset{
	uint8_t loop_index;   //DO编号，如果DO后省略了数字则为0，最大为8
	uint64_t offset;	  //位置偏移
	uint64_t line_no;    //行号

	LoopOffset& operator=( const LoopOffset& c);
	bool operator ==( const LoopOffset &one){  //判断运算符
		if(one.loop_index == this->loop_index &&
				one.line_no == this->line_no)
			return true;
		return false;
	}
};
typedef DataStack<LoopOffset> LoopOffsetStack;     //循环位置堆栈

#ifdef USES_WOOD_MACHINE
/**
 * @brief 主轴启动命令（M03/M04/S）位置偏移
 */
struct SpindleStartOffset{
	int m_code;   //03:M03   4:M04
	int s_code;   //转速
	uint64_t line_no;   //行号
    bool jump_line;   //是否存在跳段标志
//	int exec_step;   //当前执行步骤

	SpindleStartOffset& operator=( const SpindleStartOffset& c);
	bool operator ==( const SpindleStartOffset &one){  //判断运算符
		if(one.line_no == this->line_no &&
				one.m_code == this->m_code &&
				one.s_code == this->s_code &&
				one.jump_line == this->jump_line)
			return true;
		return false;
	}
};
typedef ListBuffer<SpindleStartOffset> SpindleStartList;    //主轴启动位置队列
#endif

/**
 * @brief 编译器状态场景,用于缓存编译器状态，方便模式切换
 */
struct CompilerScene{
	uint64_t ln_cur_line_no;  //当前行号，从1开始
	ProgramType n_sub_program;       //当前所处程序类型
	bool b_eof;					//到达文件尾
//	bool b_comment;           //当前是否注释状态,允许跨行注释
//	bool b_left;              //是否（）注释形式,即是否找到‘（’
	int n_sub_call_times;		//当前子程序调用次数

	CompileFileState file_state;   //编译状态
	CompileHeadState head_state;     //头部编译状态

	CompilerWorkMode work_mode;   //编译工作模式模式
	RecordMsg *p_last_move_msg;    //当前已编译数据中最后一条轴移动指令


	//文件操作
	char *ptr_cur_file_pos;     //当前文件读取位置指针
	uint64_t ln_read_size;  //当前已读取字节数

	CompileStatusCollect compiler_status;  //编译器状态集合
	AsFileMapInfo file_map_info;    //文件对象

	LabelOffsetList list_label;	//行标签号偏移位置存储队列
	SubProgOffsetList list_subprog;	//本文件内部子程序偏移位置存储队列
	LoopRecList list_loop;		//循环体队列,记录本文件内的所有循环体起始位置
	LoopOffsetStack stack_loop;     //循环位置数据

	/*****if else 处理 ******/
	vector<IfElseOffsetList> ifelse_vector;  // 链表容器
	DataStack<ListNode<IfElseOffset>> stack_ifelse_node;  //运行ifelse时节点存放栈
	bool meet_else_jump;         // 若if 条件为真  则遇到else elseif 后要直接跳转到 endif行。
	DataStack<bool> stack_else_jump_record;
	/*****if else 处理 ******/

#ifdef USES_WOOD_MACHINE
	SpindleStartList list_spd_start;   //主轴启动命令队列
#endif

	CompilerScene& operator=( const CompilerScene& c);

};

typedef DataStack<CompilerScene> CompilerSceneStack;   //编译器场景保存栈

/**
 * @brief 宏变量值
 */
struct MacroVarValue{
	double value;  //变量值
	bool init;		//是否初始化，false--未初始化，即空值   true--已初始化，value值有效
	MacroVarValue &operator=(const MacroVarValue &v);  //重写赋值运算符
	MacroVarValue(){value = 0, init = false;}   //构造函数
};


//仿真数据,编译器返回
struct CompilerSimData{
	int type;   //指令类型， 00-G00目标位置 01--G01目标位置  02--G02目标位置   03--G03目标位置
	int plane;            //当前平面    170--G17    180--G18    190--G19
	double target[3];    //坐标数据，当数据类型为圆弧半径时，pos[0]用于存放半径
	double center[3];    //圆心
	double radius;       //半径, 0表示不动，>0表示劣弧，<0表示优弧
};

struct ToolRec
{
    int G41G42dir;  // 40  41  42
//    int offset;  // 长度补偿值
	int D;       // D编号
	int H;       // H编号
	double Radius;  //半径补偿值
	int plane;  // 工作平面
	double feed;    // 进给速度
};

#endif /* INC_COMPILER_COMPILER_DATA_H_ */
