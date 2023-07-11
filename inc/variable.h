/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file variable.h
 *@author gonghao
 *@date 2020/10/19
 *@brief 本头文件包含宏变量定义
 *@version
 */
#include "global_include.h"
#include "data_stack.h"

#ifndef VARIABLE_H_
#define VARIABLE_H_

const int kMaxLocalVarCount = 33;   //局部变量个数
const int kMaxCommVarCount = 100;   //非保持型公共变量个数
const int kMaxCommKeepVarCount = 500;   	//保持型公共变量个数
const int kMaxUserMacroVarCount = 5000;
const int kMaxVarTotalCount = kMaxLocalVarCount+kMaxCommVarCount+kMaxCommKeepVarCount;   	//除系统变量外的变量总数

struct LocalVarScene{
	bool init[kMaxLocalVarCount];
	double value[kMaxLocalVarCount];

	LocalVarScene &operator =(LocalVarScene &one); 	//赋值运算符
};

class Variable {
public:
	Variable();
	virtual ~Variable();

	void SetChnIndex(uint8_t chn);   //设置所属通道

	bool GetVarValue(int index, double &value, bool &init);  	//获取变量值，返回double
	bool GetVarValue(int index, int &value, bool &init);		//获取变量值，返回int

	bool SetVarValue(int index, double value);    	//设置变量值
	bool SetVarValue(int index, int value);			//设置变量值
	bool ResetVariable(int index);		//变量复位，置为空值

	bool PushLocalVar();		//局部变量入栈
	bool PopLocalVar();		//局部变量出栈

	void Reset();		//整体复位

	void ResetLocalVar();  //局部变量复位

	bool SaveKeepComm(int index);		//保存非易失性公共变量
	bool SaveMacroComm(int index);		//保存用户宏变量

//	void Save();   //关闭非易失性公共变量的文件句柄

	void Sync(bool close = false);  //同步文件修改

	bool IsKeepChanged(){return this->m_b_save_keep;}   //是否需要同步保存文件

	int CopyVar(char *buf, uint32_t max_size, uint32_t start_index, uint8_t count);		//拷贝变量数据

	void MemsetMacroVar(int start, int count, double value);
	void InsertMacroVar(int index, int end, double value);
	void PopMacroVar(int index, int end);
	void SetMacroArray(int index, int count, char * buf);

private:
	void InitKeepVar();					//初始化非易失性宏变量
	void InitMacroVar();
	bool GetSysVar(int index, double &value);		//获取系统变量，double型
	bool GetSysVar(int index, int &value);		    //获取系统变量，int型
	bool SetSysVar(int index, double value);		//设置系统变量，double型
	bool SetSysVar(int index, int value);			//设置系统变量，int型
	void SyncUserMacroVar();

private:
	uint8_t m_n_channel_index;                  //所属通道
	bool m_b_init_local[kMaxLocalVarCount];		//局部变量初始化标志
	bool m_b_init_common[kMaxCommVarCount]; 	//非保持型公共变量初始化标志
	bool m_b_init_common_keep[kMaxCommKeepVarCount];  //保持型公共变量
	bool m_b_init_user_macro[kMaxUserMacroVarCount];  //用户宏变量
	double m_df_local[kMaxLocalVarCount];  		//局部变量
	double m_df_common[kMaxCommVarCount];		//非保持型公共变量
	double m_df_common_keep[kMaxCommKeepVarCount]; 	//保持型公共变量
	double m_df_user_macro[kMaxUserMacroVarCount];  //用户宏变量
	DataStack<LocalVarScene> m_stack_local;         //局部变量保存堆栈

	FILE *m_fp_keep_var;  //保持型公共变量的文件对象指针
	FILE *m_fp_macro_var; //宏变量文件对象指针
	bool m_b_save_keep;  //是否需要保存非易失宏变量

};

#endif /* VARIABLE_H_ */
