/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file Compiler_data.h
 *@author gonghao
 *@date 2020/04/13
 *@brief 本头文件定义G代码编译器生成的数据消息类InfoMessage
 *@version
 */

#ifndef INC_COMPILER_COMPILE_MESSAGE_H_
#define INC_COMPILER_COMPILE_MESSAGE_H_

//#include "global_include.h"
#include "compiler_data.h"
#include "comm_data_definition.h"

//消息类标志集合
struct RecordMsgFlagBits{
	uint16_t     last_rec:1;   		//bit0 是否同行最后一条指令
	uint16_t     axis_move:1;		//bit1 是否存在轴移动指令，1表示移动，0表示不移动
	uint16_t     step_flag:1;   	//bit2 单步模式下执行完指令后是否暂停，1表示暂停，0表示不暂停
	uint16_t     wait_move_over:1;	//bit3 需要等待前面的运动指令运动到位
	uint16_t	 eof_flag:1;		//bit4 程序结束标志
	uint16_t     jump_flag:1;		//bit5 跳段标志  0--无效   1--有效
	uint16_t     block_over:1;		//bit6 分块结束标志   0--无效   1--有效
	uint16_t     rsvd:9;    		//bit7-bit15 	保留
};

//消息类标志结构
typedef union _RecordMsgFlag{
	uint16_t    all;
	struct  	RecordMsgFlagBits bits;
}RecordMsgFlag;

//标志的枚举定义
enum RecordFlag{
	FLAG_LAST_REC = 0x01,	//是否同行最后一条指令
	FLAG_AXIS_MOVE = 0x02,	//是否存在轴移动指令
	FLAG_STEP = 0x04,		//单步标志,单段模式是否需要暂停
	FLAG_WAIT_MOVE_OVER = 0x08,	//等待运动到位标志
	FLAG_EOF = 0x10,			//程序结束标志
	FLAG_JUMP = 0x20,			//跳段标志
	FLAG_BLOCK_OVER = 0x40,		//分块结束标志,即最后一条连续的轴移动指令
	FLAG_POINT_MOVE = 0x80		//是否点动移动，即起点和终点重合
};

/**
 * @brief 指令消息类的基类，其它具体指令消息都从此类派生
 */
class RecordMsg{
public:
	RecordMsg();
	virtual ~RecordMsg();

//	void AppendMsg(RecordMsg *msg);  	//插入新的消息
//	void RemoveSelf();                //将自身摘出队列
//	RecordMsg *GetPreMsg(){return m_p_prev;}   //返回上一条消息指针
//	RecordMsg *GetNextMsg(){return m_p_next;}  //返回下一条消息的指针

    void SetMsgType(CodeMsgType type){m_n_type = type;}   //设置消息类型
	CodeMsgType GetMsgType(){return m_n_type;}           //获取消息类型
	void SetLineNo(uint64_t line){m_n_line_no = line;}    //设置行号
	uint64_t GetLineNo(){return m_n_line_no;}             //获取行号

	bool CheckFlag(RecordFlag flag){return (m_n_flags.all & flag)?true:false;}  //获取对应标志状态
	void SetFlag(RecordFlag flag, bool value);   //设置对应标志的状态

	// @modified by zk
	RecordMsgFlag GetFlags(){return m_n_flags;}
	void SetFlags(RecordMsgFlag flags){m_n_flags = flags;}
    // @modified by zk

	bool IsNeedWaitMsg(){return (m_n_flags.all & FLAG_WAIT_MOVE_OVER)?true:false;}    //判断是否是需要等待轴运动到位的消息
	bool IsEndMsg(){return (m_n_flags.all & FLAG_EOF)?true:false;}	//判断是否程序结束指令
	bool IsMoveMsg(){return (m_n_flags.all & FLAG_AXIS_MOVE)?true:false;} //判断是否轴移动指令
	bool HasJumpFlag(){return (m_n_flags.all & FLAG_JUMP)?true:false;} //判断是否包含跳段属性
	bool IsPointMsg(){return (m_n_flags.all & FLAG_POINT_MOVE)?true:false;}  //是否点动移动消息
	bool IsLineLastMsg(){return (m_n_flags.all & FLAG_LAST_REC)?true:false;}  //是否同行的最后一条指令

	void SetFrameIndex(uint16_t index){this->m_n_frame_index = index;}   //设置数据顺序号
	uint16_t GetFrameIndex(){return this->m_n_frame_index;}     //获取数据顺序号

	virtual void Execute() = 0;		//执行函数
	virtual void GetData(void* rec) = 0;		//获取数据
	virtual void SetData(void* rec) = 0;		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag) = 0;   	//生成待输出给MC的运动控制数据包
	virtual int GetSimulateData(CompilerSimData &data){return 1;}    //生成仿真数据包

	virtual void PrintString() = 0;   //用于程序调试

	RecordMsg& operator=( const RecordMsg& msg);  //赋值运算符
	friend bool operator ==( const RecordMsg &one, RecordMsg &two);  //判断运算符
protected:
	//	RecordMsg* m_p_next;  //下一组数据
	//	RecordMsg* m_p_prev;  //上一组数据
		CodeMsgType m_n_type;	//消息类型
		uint64_t m_n_line_no;  //行号
		RecordMsgFlag m_n_flags; //状态标志集合，用bit形式节省空间
		uint16_t m_n_frame_index;   //数据顺序号，0~65535循环
private:


};

/**
 * @brief 错误消息
 */
class ErrorMsg : public RecordMsg{
public:
	ErrorMsg(int err_code);  //构造函数
	ErrorMsg();        //构造函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	int GetErrorCode(){return m_error_code;}   //返回错误代码
	void SetErrorCode(int err){m_error_code = err;}   //设置错误代码

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   //生成待输出给MC的运动控制数据包

//	void SetClearInfoFlag(bool flag){this->m_b_clear_info = flag;}   //设置消息清除标志
//	bool GetClearInfoFlag(){return this->m_b_clear_info;}      //返回消息清除标志

	void SetInfoType(int type){this->m_n_info_type = type;}   //设置信息类型
	int GetInfoType(){return this->m_n_info_type;}   //返回信息类型

	virtual void PrintString();   //用于程序调试

	ErrorMsg& operator=( const ErrorMsg& msg);  //赋值运算符
	friend bool operator ==( const ErrorMsg &one, ErrorMsg &two);  //判断运算符
private:
	int m_error_code;    //错误代码
	int m_n_info_type;        //消息类型， 0表示提示信息，1表示告警信息
//	bool m_b_clear_info;      //清空提示信息

};

/**
 * @brief 加工复位完成消息
 */
class RestartOverMsg : public RecordMsg{
public:
	RestartOverMsg();  //构造函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据


	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	RestartOverMsg& operator=( const RestartOverMsg& msg);  //赋值运算符
	friend bool operator ==( const RestartOverMsg &one, RestartOverMsg &two);  //判断运算符
};

//@test zk 增加D H指令解析
//class




/**
 * @brief 模式消息
 */
class ModeMsg : public RecordMsg{
public:
	ModeMsg(int gcode);		//构造函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	int GetGCode(){return m_n_g_code;}   //返回G指令代码
	void SetLastGCode(int gcode_last){this->m_n_last_g_code = gcode_last;}   //设置历史模态值
	int GetLastGCode(){return this->m_n_last_g_code;}    //获取历史模态值

	ToolRec GetAToolRec(){return this->m_tool_info;}
	void SetAToolRec(ToolRec tool){this->m_tool_info = tool;}

	ModeMsg& operator=( const ModeMsg& msg);  //赋值运算符
	friend bool operator ==( const ModeMsg &one, ModeMsg &two);  //判断运算符
protected:
	int m_n_g_code;   //G指令代码，放大十倍存储，例如G43.4存储值为434
	int m_n_last_g_code;    //前一个模态，为了支持手轮反向引导功能

	ToolRec m_tool_info;
};

/**
 * @brief 运动指令消息，所有运动指令均继承自此Msg
 */
class MoveMsg : public ModeMsg{
public:
	MoveMsg(int gcode);		//构造函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	void SetMachCoord(bool flag){this->m_b_mach_coord = flag;}   //设置机械坐标系标志
	bool IsMachCoord(){return this->m_b_mach_coord;}     //是否机械坐标系

	MoveMsg& operator=( const MoveMsg& msg);  //赋值运算符
	friend bool operator ==( const MoveMsg &one, MoveMsg &two);  //判断运算符

	DPointChn &GetTargetPos(){return m_point_target;}  //返回终点坐标
	DPointChn &GetSourcePos(){return m_point_source;}  //返回起点坐标
	void SetTargetPos(DPointChn pos){ m_point_target = pos;}  //
	void SetSourcePos(DPointChn pos){ m_point_source = pos;}  //
	void setCancelG80(bool flag){cancel_g80 = flag;}
	bool NeedCancelG80(){return cancel_g80;};

protected:
	bool m_b_mach_coord;    //当前是否机械坐标系
	DPointChn m_point_target;   //目标点
	DPointChn m_point_source;   //起始点
	bool cancel_g80 = false;
};


/**
 * @brief 坐标系建立消息，包括G92/G52/G53/G54~G59/G5401~G5499
 */
class CoordMsg : public ModeMsg{
public:
	CoordMsg(const DPointChn &pos, const DPointChn &src, int gcode, uint32_t axis_mask);    //构造函数
	virtual void Execute();   	//执行函数
	virtual void GetData(void *rec);   //获取数据
	virtual void SetData(void *rec);   //设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包
	virtual int GetSimulateData(CompilerSimData &data);    //生成仿真数据包

	virtual void PrintString();    //用于程序调试

	uint8_t GetExecStep(){return m_n_exec_step;}		//获取当前步骤
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //设置执行步骤
	DPointChn &GetTargetPos(){return m_pos;}    //返回目标位置
//	void GetTargetPosEx(DPointChn &pos);     //获取多维点形式的坐标值
	uint32_t GetAxisMask(){return m_n_axis_mask;}    //返回轴掩码

	CoordMsg &operator=(const CoordMsg &msg);   //赋值运算符
	friend bool operator ==( const CoordMsg &one, CoordMsg &two);  //判断运算符
protected:
	DPointChn m_pos;   //当前位置在新定义的坐标系下的坐标，只针对插补轴
	DPointChn m_pos_src;   //起点坐标
	uint32_t m_n_axis_mask;  //指定轴掩码
	uint8_t m_n_exec_step;  //执行阶段标志

};



/**
 * @brief 进给速度消息
 */
class FeedMsg : public RecordMsg{
public:
	FeedMsg(double feed, double last_feed);		//构造函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	double GetFeed(){return m_df_feed;}   //返回进给速度

	FeedMsg& operator=( const FeedMsg& msg);  //赋值运算符
	friend bool operator ==( const FeedMsg &one, FeedMsg &two);  //判断运算符
protected:
	double m_df_feed;   //进给速度，单位:mm/min
	double m_df_last_feed;   //上一个指定进给速度， 为了支持手轮反向引导功能
};

/**
 * @brief 主轴转速消息
 */
class SpeedMsg : public RecordMsg{
public:
	SpeedMsg(double speed);		//构造函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	double GetSpeed(){return m_df_speed;}   //返回主轴转速

	SpeedMsg& operator=( const SpeedMsg& msg);  //赋值运算符
	friend bool operator ==( const SpeedMsg &one, SpeedMsg &two);  //判断运算符
protected:
	double m_df_speed;   //主轴转速，单位:rpm
    double m_df_last_speed;    //上一个指定的主轴转速，为了支持手轮反向引导功能
};

/**
 * @brief 刀具消息
 */
class ToolMsg : public RecordMsg{
public:
	ToolMsg(int *tool, uint8_t total);		//构造函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	uint16_t GetTool(uint8_t index){return m_n_tool[index];}  //返回刀号
	uint8_t GetCount(){return m_n_tool_count;}   //获取T指令个数

	bool IsFirstExec();    //是否首次执行此命令

	uint8_t GetExecStep(uint8_t index){return index<m_n_tool_count?m_n_tool_exec_segment[index]:-1;}		//获取当前步骤，失败返回-1
	void SetExecStep(uint8_t index, uint8_t step){if(index < m_n_tool_count) m_n_tool_exec_segment[index] = step;} //设置执行步骤
	void IncreaseExecStep(uint8_t index){if(index<m_n_tool_count) m_n_tool_exec_segment[index]++;}   //步骤自加

	int GetSubProgIndex(){return m_n_sub_prog_name;}  //返回子程序号
    //void GetSubProgName(char *name, bool abs_path);	 //返回子程序名

	void SetSubProgType(uint8_t type){m_n_sub_prog_type = type;} //设置子程序类型
	uint8_t GetSubProgType(){return m_n_sub_prog_type;}   	//返回子程序类型

	uint16_t GetLastTool(){return this->m_n_last_tool;}   //返回上一把刀号
	void SetLastTool(uint16_t last_tool){this->m_n_last_tool = last_tool;}   //设置上一把刀号

	ToolMsg& operator=( const ToolMsg& msg);  //赋值运算符
	friend bool operator ==( const ToolMsg &one, ToolMsg &two);  //判断运算符
protected:
	uint16_t m_n_tool[kMaxTCodeInLine];   //刀号

	uint16_t m_n_last_tool;   //上一把刀号

	int m_n_sub_prog_name;   //子程序名
	uint8_t m_n_sub_prog_type;  //子程序类型  1--在本程序内   2--同目录下nc文件    3--系统子程序目录nc文件     4--同目录下iso文件    5--系统子程序目录iso文件

	uint8_t m_n_tool_count;   //T指令总数
	uint8_t m_n_tool_exec_segment[kMaxTCodeInLine];   //T指令执行阶段   0--修改代码编译执行线程状态   1--将T代码发送给PMC    2--等待PMC返回执行完成标志   0xFF--表示执行结束
};

/**
 * @brief 延时消息，包括G04
 */
class TimeWaitMsg : public ModeMsg{
public:
	TimeWaitMsg(uint32_t time);    //构造函数
	virtual void Execute();   	//执行函数
	virtual void GetData(void *rec);   //获取数据
	virtual void SetData(void *rec);   //设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();    //用于程序调试

	uint32_t GetDelayTime(){return m_n_time_delay;}		//获取延时时间
	void SetStartTime(uint64_t start){ m_n_start_time = start;} 			//设置起始时间
	uint64_t GetStartTime(){return m_n_start_time;}      //返回当前起始时间

	TimeWaitMsg &operator=(const TimeWaitMsg &msg);   //赋值运算符
	friend bool operator ==( const TimeWaitMsg &one, TimeWaitMsg &two);  //判断运算符
private:
	uint32_t m_n_time_delay;   //延时时间，单位ms
	uint64_t m_n_start_time;  //计时起始时间,单位ms，

};

/**
 * @brief 参考点返回消息，包括G27 G28、G29、G30
 */
class RefReturnMsg : public ModeMsg{
public:
	RefReturnMsg(int gcode, uint32_t axis_mask, DPointChn &mid);    //构造函数
	virtual void Execute();   	//执行函数
	virtual void GetData(void *rec);   //获取数据
	virtual void SetData(void *rec);   //设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包
	virtual int GetSimulateData(CompilerSimData &data);    //生成仿真数据包

	virtual void PrintString();    //用于程序调试

	uint8_t GetExecStep(){return m_n_exec_step;}		//获取当前步骤
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //设置执行步骤
	uint32_t GetAxisMask(){return m_n_axis_mask;} 	//返回轴mask
	DPointChn &GetMiddlePos(){return m_pos_middle;}     //返回中间点坐标
	RefReturnMsg &operator=(const RefReturnMsg &msg);   //赋值运算符
	friend bool operator ==( const RefReturnMsg &one, RefReturnMsg &two);  //判断运算符

	double ref_id;      // 回第几个参考点  给 G30 PX 用

private:
	uint32_t m_n_axis_mask;   //标记回参考点的轴，通道轴
	DPointChn m_pos_middle;  //中间点位置
	uint8_t m_n_exec_step;  //执行阶段标志

};


/**
 * @brief M辅助指令消息
 */
class AuxMsg : public RecordMsg{
public:
	AuxMsg(int mcode);    //单M指令构造函数
	AuxMsg(int *mcode, uint8_t total);		//构造函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	int GetMCode(uint8_t index=0){return index<m_n_m_count?m_n_m_code[index]:-1;}   //返回M代码值,失败返回-1
	uint8_t GetMCount(){return m_n_m_count;}   //返回M代码个数

	uint8_t GetExecStep(uint8_t index){return index<m_n_m_count?m_n_aux_exec_segment[index]:-1;}		//获取当前步骤，失败返回-1
	void SetExecStep(uint8_t index, uint8_t step){if(index < m_n_m_count) m_n_aux_exec_segment[index] = step;} //设置执行步骤
	void IncreaseExecStep(uint8_t index){if(index<m_n_m_count) m_n_aux_exec_segment[index]++;}   //步骤自加

	bool IsFirstExec();    //是否首次执行此命令

	AuxMsg& operator=( const AuxMsg& msg);  //赋值运算符
	friend bool operator ==( const AuxMsg &one, AuxMsg &two);  //判断运算符
protected:
	int m_n_m_code[kMaxMCodeInLine];   //M指令代码
//	uint8_t m_n_m_index;		//M指令顺序号，一行最多3个M指令，从1-3
	uint8_t m_n_m_count;      //M指令个数
	uint8_t m_n_aux_exec_segment[kMaxMCodeInLine];   //辅助指令执行阶段   0--修改代码编译执行线程状态   1--将M代码发送给PMC    2--等待PMC返回执行完成标志
};

/**
 * @brief M98子程序调用指令消息
 */
class SubProgCallMsg : public AuxMsg{
public:
    SubProgCallMsg(int pcode, int lcode, uint8_t scan = 0);    //构造函数

	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	int GetSubProgIndex(){return m_n_sub_prog_name;}  //返回子程序号
    //void GetSubProgName(char *name, bool abs_path);					//返回子程序名
	int GetCallTimes(){return m_n_call_times;}		//返回调用次数

	void SetSubProgType(uint8_t type){m_n_sub_prog_type = type;} //设置子程序类型
	uint8_t GetSubProgType(){return m_n_sub_prog_type;}   	//返回子程序类型

	void SetLastProgFile(char *file);   //设置上一个文件的绝对路径，用于手轮反向引导
	void GetLastProgFile(char *file);   //获取上一个文件的绝对路径，用于手轮方向引导

    uint8_t GetScanMode() const;

	SubProgCallMsg& operator=( const SubProgCallMsg& msg);  //赋值运算符
	friend bool operator ==( const SubProgCallMsg &one, SubProgCallMsg &two);  //判断运算符
private:
	int m_n_sub_prog_name;   //子程序名
	int m_n_call_times;     //调用次数
	uint8_t m_n_sub_prog_type;  //子程序类型  1--在本程序内   2--同目录下nc文件    3--系统子程序目录nc文件     4--同目录下iso文件    5--系统子程序目录iso文件
    uint8_t m_n_scan_mode = 0;//子程序查找规则 0--> (1.sys_sub查找, 2.mac_sub查找)  1--> (1.本目录查找, 2.sys_sub查找, 3.mac_sub查找)

	char m_str_last_prog_file[kMaxPathLen];   //前一个文件绝对路径，用于反向引导
};

/**
 * @brief G65宏程序调用指令消息
 */
class MacroProgCallMsg : public ModeMsg{
public:
    MacroProgCallMsg(int pcode, int lcode, double *param, uint8_t count, uint32_t mask, uint8_t scan = 0);    //构造函数
	~MacroProgCallMsg();  //析构函数

	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	int GetMacroProgIndex(){return m_n_macro_prog_name;}  //返回子程序号
    //void GetMacroProgName(char *name, bool abs_path);					//返回子程序名
	int GetCallTimes(){return m_n_call_times;}		//返回调用次数
	double *GetParameter(uint32_t &mask, uint8_t &count);     //返回参数数据


	void SetMacroProgType(uint8_t type){m_n_macro_prog_type = type;} //设置宏程序类型
	uint8_t GetMacroProgType(){return m_n_macro_prog_type;}   	//返回宏程序类型

	void SetLastProgFile(char *file);   //设置上一个文件的绝对路径，用于手轮反向引导
	void GetLastProgFile(char *file);   //获取上一个文件的绝对路径，用于手轮方向引导

    uint8_t GetScanMode() const;

	MacroProgCallMsg& operator=( const MacroProgCallMsg& msg);  //赋值运算符
	friend bool operator ==( const MacroProgCallMsg &one, MacroProgCallMsg &two);  //判断运算符
private:
	int m_n_macro_prog_name;   //宏程序名
	int m_n_call_times;     //调用次数
	double *m_p_df_param;   //调用参数
	uint32_t m_mask_param;   //有效参数MASK
	uint8_t m_n_param_count;  //参数个数
	uint8_t m_n_macro_prog_type;  //子程序类型  1--在本程序内   2--同目录下nc文件    3--系统子程序目录nc文件     4--同目录下iso文件    5--系统子程序目录iso文件
    uint8_t m_n_scan_mode = 0;//宏程序查找规则 0--> (1.sys_sub查找, 2.mac_sub查找)  1--> (1.本目录查找, 2.sys_sub查找, 3.mac_sub查找)

	char m_str_last_prog_file[kMaxPathLen];   //前一个文件绝对路径，用于反向引导
};

/**
 * @brief 子程序返回消息
 */
class SubProgReturnMsg : public AuxMsg{
public:
	SubProgReturnMsg(char *file_name, bool ret_macro);    //构造函数

	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	void GetReturnFileName(char *file){strcpy(file, m_str_file_name);}  //返回上层程序名称
	bool IsRetFromMacroProg(){return this->m_b_ret_from_macroprog;}    //是否从宏程序调用中返回

	SubProgReturnMsg& operator=( const SubProgReturnMsg& msg);  //赋值运算符
	friend bool operator ==( const SubProgReturnMsg &one, SubProgReturnMsg &two);  //判断运算符
private:
	char m_str_file_name[kMaxFileNameLen];   //子程序名
	bool m_b_ret_from_macroprog;        //是否从宏程序返回

};


/**
 * @brief 循环指令消息
 */
class LoopMsg : public ModeMsg{
public:
	LoopMsg(const int gcode, double *param, uint8_t count, uint32_t mask);   //构造函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	int GetMacroProgIndex();  //返回子程序号
    //void GetMacroProgName(char *name, bool abs_path);					//返回子程序名
	double *GetParameter(uint32_t &mask, uint8_t &count);     //返回参数数据


	void SetMacroProgType(uint8_t type){m_n_macro_prog_type = type;} //设置宏程序类型
	uint8_t GetMacroProgType(){return m_n_macro_prog_type;}   	//返回宏程序类型

	void SetLastProgFile(char *file);   //设置上一个文件的绝对路径，用于手轮反向引导
	void GetLastProgFile(char *file);   //获取上一个文件的绝对路径，用于手轮方向引导

	LoopMsg& operator=( const LoopMsg& msg);  //赋值运算符
	friend bool operator ==( const LoopMsg &one, LoopMsg &two);  //判断运算符

private:
	double *m_p_df_param;   //调用参数
	uint32_t m_mask_param;   //有效参数MASK
	uint8_t m_n_param_count;  //参数个数
	uint8_t m_n_macro_prog_type;  //子程序类型  1--在本程序内   2--同目录下nc文件    3--系统子程序目录nc文件     4--同目录下iso文件    5--系统子程序目录iso文件

	char m_str_last_prog_file[kMaxPathLen];   //前一个文件绝对路径，用于反向引导
};

/**
 * @brief G00快速定位消息
 */
class RapidMsg : public MoveMsg{
public:
	RapidMsg(const DPointChn &source, const DPointChn &target, const uint32_t axis_mask);		//构造函数
	virtual ~RapidMsg();       //析构函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包
	virtual int GetSimulateData(CompilerSimData &data);    //生成仿真数据包

	virtual void PrintString();   //用于程序调试
	
	uint32_t GetAxisMoveMask(){return m_axis_move_mask;}   //返回移动轴mask

	void RefreshTargetPos(DPointChn &pos);	//同步目标位置

	void SetIoData(uint8_t data){this->m_io_data = data;}   //设置IO数据

    void SetPmcAxisCount(uint8_t count);   //设置PMC轴的运动数据
    uint8_t GetPmcAxisCount(){return this->m_n_pmc_count;}   //获取运动PMC轴数
//	double *GetPmcTarget(uint8_t &count, uint32_t &mask, bool &inc);   //获取PMC轴的运动目标数据
	uint8_t GetExecStep(){return m_n_exec_step;}		//获取当前步骤
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //设置执行步骤

	RapidMsg& operator=( const RapidMsg& msg);  //赋值运算符
	friend bool operator ==( const RapidMsg &one, RapidMsg &two);  //判断运算符
private:
	uint32_t m_axis_move_mask;  //移动轴mask，bit0-bit15分别标志通道第1-16轴是否有移动指令
	uint8_t m_io_data;       //io输出数据，0x00表示无IO输出   从1开始，有效范围1~127

//	double *m_p_pmc_target;  //PMC轴的目标位置
//	uint32_t m_pmc_move_mask;   //PMC移动轴mask
    uint8_t m_n_pmc_count{0};     //PMC移动轴数量
//	bool m_b_inc_pos;          //PMC轴目标地址是否增量模式
	uint8_t m_n_exec_step;     //执行阶段标志
};

/**
 * @brief G01直线切削消息
 */
class LineMsg : public MoveMsg{
public:
	LineMsg(const DPointChn &source, const DPointChn &target, const double feed, const uint32_t axis_mask);    //构造函数
	virtual ~LineMsg();
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包
	virtual int GetSimulateData(CompilerSimData &data);    //生成仿真数据包

	virtual void PrintString();   //用于程序调试

	void SetFeed(const double feed){m_df_feed = feed;}  //设置进给速度
	double GetFeed(){return m_df_feed;};    //获取进给速度
	
	uint32_t GetAxisMoveMask(){return m_axis_move_mask;}   //返回移动轴mask

	void RefreshTargetPos(DPointChn &pos);	//同步目标位置

	void SetIoData(uint8_t data){this->m_io_data = data;}   //设置IO数据

    void SetPmcAxisCount(uint8_t pmc_mask);   //设置PMC轴的运动数据
    uint8_t GetPmcAxisCount(){return this->m_n_pmc_count;}   //获取运动PMC轴数
//	double *GetPmcTarget(uint8_t &count, uint32_t &mask, bool &inc);   //获取PMC轴的运动目标数据
	uint8_t GetExecStep(){return m_n_exec_step;}		//获取当前步骤
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //设置执行步骤

	LineMsg& operator=( const LineMsg& msg);  //赋值运算符
	friend bool operator ==( const LineMsg &one, LineMsg &two);  //判断运算符

protected:
	double m_df_feed;		//进给速度   单位：mm/min
	uint32_t m_axis_move_mask;  //移动轴mask，bit0-bit15分别标志通道第1-16轴是否有移动指令
	uint8_t m_io_data;       //io输出数据，0x00表示无IO输出   从1开始，有效范围1~127

    uint8_t m_n_pmc_count{0};     //PMC移动轴数量
	uint8_t m_n_exec_step;     //执行阶段标志
    //	double *m_p_pmc_target;  //PMC轴的目标位置
    //	uint32_t m_pmc_move_mask;   //PMC移动轴mask
    //	bool m_b_inc_pos;          //PMC轴目标地址是否增量模式
};

/**
 * @brief 跳转消息，包括G31
 */
class SkipMsg : public LineMsg{
public:
	SkipMsg(const DPointChn &source, const DPointChn &target, const double feed, const uint32_t axis_mask);    //构造函数
	virtual ~SkipMsg(){}    //析构函数
	virtual void Execute();   	//执行函数
	virtual void GetData(void *rec);   //获取数据
	virtual void SetData(void *rec);   //设置数据

//	virtual int GetOutputData(GCodeFrame *data, uint32_t mask);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();    //用于程序调试

	uint8_t GetExecStep(){return m_n_exec_step;}		//获取当前步骤
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //设置执行步骤


	SkipMsg &operator=(const SkipMsg &msg);   //赋值运算符
	friend bool operator ==( const SkipMsg &one, SkipMsg &two);  //判断运算符
private:
	uint8_t m_n_exec_step;  //执行阶段标志

};

/**
 * @brief 自动对刀
 * G37  ...
 */
class AutoToolMeasureMsg : public ModeMsg{
public:
	AutoToolMeasureMsg(const DPointChn &target, const uint32_t axis_mask, const int h_code, const int times);    //构造函数
	virtual ~AutoToolMeasureMsg(){}       //析构函数
	virtual void Execute();   	//执行函数
	virtual void GetData(void *rec);   //获取数据
	virtual void SetData(void *rec);   //设置数据

	virtual void PrintString();    //用于程序调试

	DPointChn &GetTarget(){return this->m_pos_target;}   //返回目标位置
	uint32_t GetAxisMoveMask(){return this->m_mask_axis_move;}   //返回轴移动mask
	int GetTimes(){return this->m_n_times;}    //返回对刀次数
	int GetHIndex(){return this->m_n_h_index;}   //返回写入的H值号

	int GetMacroProgIndex(){return m_n_macro_prog_name;}  //返回对应宏程序号
    //void GetMacroProgName(char *name, bool abs_path);	   //返回宏程序名

	void SetMacroProgType(uint8_t type){m_n_macro_prog_type = type;} //设置宏程序类型
	uint8_t GetMacroProgType(){return m_n_macro_prog_type;}   	//返回宏程序类型

	AutoToolMeasureMsg &operator=(const AutoToolMeasureMsg &msg);   //赋值运算符
	friend bool operator ==( const AutoToolMeasureMsg &one, AutoToolMeasureMsg &two);  //判断运算符
private:
	DPointChn m_pos_target;    //目标位置
	uint32_t m_mask_axis_move;   //轴移动mask
	int m_n_times;            //执行次数
	int m_n_h_index;          //写入的H值

	int m_n_macro_prog_name;   //宏程序号
	uint8_t m_n_macro_prog_type;  //子程序类型  1--在本程序内   2--同目录下nc文件    3--系统子程序目录nc文件     4--同目录下iso文件    5--系统子程序目录iso文件

};

/**
 * @brief 刀补消息（G40/G41/G42/G43/G44/G49）
 */
class CompensateMsg : public LineMsg{
public:
	CompensateMsg(int gcode, uint16_t data, const DPointChn &source, const DPointChn &target, const uint32_t axis_mask, const double feed, uint8_t move_type = 0);		//构造函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包
	virtual int GetSimulateData(CompilerSimData &data);    //生成仿真数据包

	virtual void PrintString();   //用于程序调试

	int GetCompValue(){return m_n_data;}  //获取补偿值
	void SetCompLastValue(int value){this->m_n_data_last = value;}   //设置历史补偿值
	int GetLastCompValue(){return m_n_data_last;}     //获取历史补偿值

	uint8_t GetExecStep(){return m_n_exec_step;}		//获取当前步骤
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //设置执行步骤
    uint8_t GetMoveType(){return m_n_move_type;};       //移动类型


	CompensateMsg& operator=( const CompensateMsg& msg);  //赋值运算符
	friend bool operator ==( const CompensateMsg &one, CompensateMsg &two);  //判断运算符
	int GetGcode(){return m_n_g_code;};

private:
	uint16_t m_n_data;   //数据，对于G41/G42为D值，对于G43/G44为H值
	uint16_t m_n_data_last;   //之前的数据，对于G41/G42为D值，对于G43/G44为H值，用于手轮反向引导
	uint8_t m_n_move_type;   //0--表示G00， 1--表示G01
	uint8_t m_n_exec_step;  //执行阶段标志

};

/**
 * @brief G02/G03圆弧消息
 */
class ArcMsg : public MoveMsg{
public:
	int arc_id;  // @test zk
	ArcMsg(int code,
			const DPointChn& source,
	        const DPointChn& target,
		    const DPointChn& center,
		    const double& radius,
			const double feed,
			const uint32_t axis_mask,
		    const int8_t dir_flag = -1 ,
			const int8_t major_flag = 1,
			const int8_t circle_flag = 0);

	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包
	virtual int GetSimulateData(CompilerSimData &data);    //生成仿真数据包

	virtual void PrintString();   //用于程序调试

	void SetFeed(const double feed){m_df_feed = feed;}  //设置进给速度
	double GetFeed(){return m_df_feed;};    //获取进给速度
	void SetPlaneMode(const uint16_t gmode2){m_gmode_2 = gmode2;}    //设置加工平面模态

	DPointChn &GetCenterPos(){return m_point_center;}  //返回起点坐标
	
	void SetCenterPos(DPointChn pos){m_point_center = pos;}  
	
	void getArcFlags(int8_t &dir, int8_t &major, int8_t &circle){dir=m_flag_direct,major=m_flag_major,circle=m_flag_circle;}    // 返回圆心角 
//	double getAngle(void);    // 返回圆心角 
	uint32_t GetAxisMoveMask(){return m_axis_move_mask;}   //返回移动轴mask

	void RefreshTargetPos(DPointChn &pos);	//同步目标位置

	bool CalArcCenter(const DPointChn &src, const DPointChn &tar, const double &radius, const int8_t flag, const uint16_t gmode_2, DPointChn &center);  //计算圆弧中心坐标

	void SetIoData(uint8_t data){this->m_io_data = data;}   //设置IO数据

	int getDirect(){return m_flag_direct;}

	void setI(double i){ i_number = i;}
	void setJ(double j){ j_number = j;}
	void setR(double r){ r_number = r;}
    double getI(){ return i_number;}
    double getJ(){ return j_number;}
    double getR(){ return r_number;}

	ArcMsg& operator=( const ArcMsg& msg);  //赋值运算符
	friend bool operator ==( const ArcMsg &one, ArcMsg &two);  //判断运算符
    void SetPmcAxisCount(uint8_t count);   //设置PMC轴的运动数据
    uint8_t GetPmcAxisCount(){return this->m_n_pmc_count;}   //获取运动PMC轴数

private:
	int8_t m_flag_direct;  	//轨迹方向标志，-1:clockwise,1:anticlockwise  //顺时针(-1)，逆时针(1)
	int8_t m_flag_major;   	//优弧标志， 1:劣弧   -1:优弧
	int8_t m_flag_circle;	//整圆标志， 0：圆弧	1：整圆
	DPointChn  m_point_center;	//圆心坐标
	double  m_df_radius;	//半径，单位mm
	double m_df_feed;		//进给速度   单位：mm/min
	uint32_t m_axis_move_mask;  //移动轴mask，bit0-bit15分别标志通道第1-16轴是否有移动指令
	uint8_t m_io_data;       //io输出数据，0x00表示无IO输出   从1开始，有效范围1~127
    uint16_t m_gmode_2;     //加工平面模态
	double i_number, j_number, r_number;
    uint8_t m_n_pmc_count{0};     //PMC移动轴数量
};

/**
 * @brief G25/G26主轴转速检测消息
 */
class SpindleCheckMsg : public ModeMsg{
public:
	SpindleCheckMsg(int gcode, int p, int q, int r, int i);

	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	SpindleCheckMsg& operator=( const SpindleCheckMsg& msg);  //赋值运算符
	friend bool operator ==( const SpindleCheckMsg &one, SpindleCheckMsg &two);  //判断运算符

private:
	int m_delay_time;   //P参数，发出S指令后延时检测转速，单位：ms
	int m_allowed_tolerance;   //Q参数，指定的主轴转速容限，单位：%
	int m_warn_ratio;      //R参数，产生告警的主轴转速变动率，单位：%
	int m_warn_speed;      //I参数，产生告警的主轴转速差，单位：转每分

};

/**
 * @brief 宏指令消息
 */
class MacroCmdMsg : public RecordMsg{
public:
	MacroCmdMsg(LexerMacroCmd *macro);		//构造函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	int GetMacroCmd(){return m_macro_cmd;}   //返回G指令代码
	MacroExpression &GetMacroExp(int index){return m_macro_expression[index];}   //获取宏表达式
	MacroVarValue &GetMacroExpResult(int index){return m_macro_exp_res[index];}   //获取宏表达式结果
	bool GetMacroExpCalFlag(int index){return this->m_b_exp_cal[index];}    //获取宏表达式计算标志
	void SetMacroExpCalFlag(int index, bool flag){this->m_b_exp_cal[index] = flag;}  //设置宏表达式计算标志
	void SetRunStep(uint8_t step){m_n_run_step = step;}   //设置执行进度标志
	uint8_t GetRunStep(){return m_n_run_step;}   //获取执行进度标志
	void SetOffset(uint64_t offset){m_ln_offset = offset;}    //设置行地址偏移
	uint64_t GetOffset(){return m_ln_offset;}    //获取行地址开关

	MacroCmdMsg& operator=(MacroCmdMsg& msg);  //赋值运算符
	friend bool operator ==(MacroCmdMsg &one, MacroCmdMsg &two);  //判断运算符
protected:
	MacroCmd m_macro_cmd;   //宏指令类型
	MacroExpression m_macro_expression[2];  //宏表达式
	MacroVarValue m_macro_exp_res[2];     //宏表达式运算结果
	bool m_b_exp_cal[2];         //宏表达式是否已经计算
	uint8_t m_n_run_step;     //宏指令执行的进度， 0--表示计算表达式0    1--表示计算表达式1
	uint64_t m_ln_offset;     //指令所在行偏移地址
};

/**
 * @brief 极坐标插补相关消息
 */
class PolarIntpMsg : public ModeMsg{
public:
	PolarIntpMsg(int gcode, int p = 0, int r = 0, int l = 0, int q = 0, int x = 0, int y = 0, int i = 0, int j = 0);

	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	uint8_t GetExecStep(){return m_n_exec_step;}		//获取当前步骤
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //设置执行步骤
	int GetParamP(){return m_n_p;}
	int GetParamR(){return m_n_r;}
	int GetParamL(){return m_n_l;}
	int GetParamQ(){return m_n_q;}
	int GetParamX(){return m_n_start_x;}
	int GetParamY(){return m_n_start_y;}
	int GetParamI(){return m_n_vect_i;}
	int GetParamJ(){return m_n_vect_j;}

	PolarIntpMsg& operator=( const PolarIntpMsg& msg);  //赋值运算符
	friend bool operator ==( const PolarIntpMsg &one, PolarIntpMsg &two);  //判断运算符

private:
	int m_n_p;   	//P参数，G12.2指令用于指定磨削指令平滑时间，单位：ms
	int m_n_r;   	//R参数，G12.2指令用于指定砂轮半径索引
	int m_n_l;      //L参数，暂未使用
	int m_n_q;      //Q参数，暂未使用

	int m_n_start_x;   //磨削起点X坐标, 单位：um
	int m_n_start_y;	//磨削起点Y坐标，单位：um
	int m_n_vect_i;		//磨削方向矢量I，放大1000000倍
	int m_n_vect_j;		//磨削方向矢量J，放大1000000倍


	uint8_t m_n_exec_step;  //执行阶段标志
};

/**
 * @brief 清整数圈位置消息
 */
class ClearCirclePosMsg : public ModeMsg{
public:
	ClearCirclePosMsg(int gcode, uint32_t axis_mask, int loop);		//构造函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	uint8_t GetExecStep(){return m_n_exec_step;}		//获取当前步骤
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //设置执行步骤
	int GetCircleMode(){return m_n_circle_loop;};
	uint32_t GetAxisMask(){return m_axis_mask;};


	ClearCirclePosMsg& operator=( const ClearCirclePosMsg& msg);  //赋值运算符
	friend bool operator ==( const ClearCirclePosMsg &one, ClearCirclePosMsg &two);  //判断运算符
private:
	uint32_t m_axis_mask;  //轴掩码
	int m_n_circle_loop;  //模

	uint8_t m_n_exec_step;  //执行阶段标志
};

#ifdef USES_SPEED_TORQUE_CTRL
/**
 * @brief 速度指令消息（G1000/G1001）
 */
class SpeedCtrlMsg : public ModeMsg{
public:
	SpeedCtrlMsg(int gcode, double *target, const uint8_t count, const uint32_t axis_mask);    //构造函数
	virtual ~SpeedCtrlMsg();    //析构函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	double *GetTargetSpeed(){return m_p_target_speed;}  //返回终点坐标

	void  ChangeUint();

	uint32_t GetAxisMoveMask(){return m_axis_move_mask;}   //返回轴mask
	
	uint8_t GetExecStep(){return m_n_exec_step;}		//获取当前步骤
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //设置执行步骤

//	void SetPmcAxisData(double *target, uint8_t count, uint32_t axis_mask);   //设置PMC轴的运动数据
	double *GetTargetValue(uint8_t &count, uint32_t &mask);   //获取PMC轴的运动目标数据

	SpeedCtrlMsg& operator=( const SpeedCtrlMsg& msg);  //赋值运算符
	friend bool operator ==( const SpeedCtrlMsg &one, SpeedCtrlMsg &two);  //判断运算符

protected:
	double *m_p_target_speed;   //目标速度

	uint32_t m_axis_move_mask;  //运动轴mask，bit0-bit31分别标志通道第1-32轴是否有移动指令

	uint8_t m_n_axis_count;     //移动轴数量

	uint8_t m_n_exec_step;  //执行阶段标志
};

/**
 * @brief 力矩指令消息（G2000/G2001）
 */
class TorqueCtrlMsg : public ModeMsg{
public:
	TorqueCtrlMsg(int gcode, double *target, const uint8_t count, const uint32_t axis_mask, uint32_t time = 0);    //构造函数
	virtual ~TorqueCtrlMsg();  //析构函数
	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//生成待输出给MC的运动控制数据包

	virtual void PrintString();   //用于程序调试

	double *GetTargetTorque(){return m_p_target_torque;}  //返回目标力矩

	uint32_t GetAxisMoveMask(){return m_axis_move_mask;}   //返回轴mask

	uint8_t GetMoveAxisCount(){return m_n_axis_count;}   //返回运动轴数量

	uint32_t GetTimeoverValue(){return m_n_timeover;}   //返回超时时间
	struct timeval *GetStartTimePtr(){return &m_start_time;}  //返回起始时间指针

	uint8_t GetCheckCount(){return m_n_check_count;}     //返回力矩到达确认计数
	void ResetCheckCount(){m_n_check_count = 0;}         //将力矩到达确认计数重置为0
	void IncCheckCount(){m_n_check_count++;}        //力矩确认计数加一

//	void SetPmcAxisData(double *target, uint8_t count, uint32_t axis_mask);   //设置PMC轴的运动数据
	double *GetTargetValue(uint8_t &count, uint32_t &mask);   //获取目标数据

	void SetSpeedLmtValue(int16_t val){ m_axis_speed_lmt = val;}
	
	int16_t GetSpeedLmtValue(){return m_axis_speed_lmt;}
	
	uint8_t GetExecStep(){return m_n_exec_step;}		//获取当前步骤
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //设置执行步骤

	TorqueCtrlMsg& operator=( const TorqueCtrlMsg& msg);  //赋值运算符
	friend bool operator ==( const TorqueCtrlMsg &one, TorqueCtrlMsg &two);  //判断运算符

protected:
	double *m_p_target_torque;   //目标力矩
	uint32_t m_axis_move_mask;  //运动轴mask，bit0-bit31分别标志通道第1-32轴是否有速度指令
	uint32_t m_n_timeover;     //超时时间，单位：毫秒   0-表示无超时
	int16_t m_axis_speed_lmt;  // 力矩控制时的轴速度限制值
	uint8_t m_n_exec_step;     //执行阶段标志
	uint8_t m_n_axis_count;     //移动轴数量

	uint8_t m_n_check_count;  //力矩到达确认计数

	struct timeval m_start_time;    //命令执行起始时间，用于超时判断


};
#endif

/**
 * @brief 输入指令消息（G10）
 */
class InputMsg : public RecordMsg{
public:
	InputMsg();    //构造函数
	virtual ~InputMsg();  //析构函数

	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据
	//生成待输出给MC的运动控制数据包
	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);
	virtual void PrintString();   //用于程序调试
	InputMsg& operator=( const InputMsg& msg);  //赋值运算符

	double LData;
	double PData;
	double RData;
	double HData;
	double DData;
	double XData;
	double YData;
	double ZData;
	double QData;

};

// 准停消息  G09
class ExactStopMsg: public RecordMsg{
public:
	ExactStopMsg(const DPointChn &source, const DPointChn &target, const uint32_t axis_mask);    //构造函数
	virtual ~ExactStopMsg();  //析构函数

	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据
	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);
	virtual void PrintString();   //用于程序调试
	ExactStopMsg& operator=( const ExactStopMsg& msg);  //赋值运算符

	DPointChn &GetTargetPos(){return m_point_target;}  //返回终点坐标
	DPointChn &GetSourcePos(){return m_point_source;}  //返回起点坐标
	uint32_t GetAxisMask(){return m_axis_mask;}

	DPointChn m_point_target;
	DPointChn m_point_source;
	uint32_t  m_axis_mask;
};

class OpenFileMsg :public RecordMsg{
public:
	OpenFileMsg();    //构造函数
	virtual ~OpenFileMsg();  //析构函数

	virtual void Execute();		//执行函数
	virtual void GetData(void* rec );	//获取数据
	virtual void SetData(void* rec);		//设置数据
	//生成待输出给MC的运动控制数据包
	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);
	virtual void PrintString();   //用于程序调试
	OpenFileMsg& operator=( const OpenFileMsg& msg);  //赋值运算符
	double OData;
};



//class FiveAxisMsg:public ModeMsg{
//public:
//	FiveAxisMsg(int gcode); 		//构造函数
//
//private:
//
//};

typedef ListBuffer<RecordMsg *> CompileRecList;  //语法分析结果队列类型
typedef ListBuffer<RecordMsg *> OutputMsgList;   //待输出的指令消息队列
typedef ListBuffer<RecordMsg *> ToolCompMsgList;   //刀补的指令消息队列



#endif /* INC_COMPILER_COMPILE_MESSAGE_H_ */
