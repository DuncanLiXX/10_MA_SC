/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file variable.h
 *@author gonghao
 *@date 2020/10/19
 *@brief ��ͷ�ļ��������������
 *@version
 */
#include "global_include.h"
#include "data_stack.h"

#ifndef VARIABLE_H_
#define VARIABLE_H_

const int kMaxLocalVarCount = 33;   //�ֲ���������
const int kMaxCommVarCount = 100;   //�Ǳ����͹�����������
const int kMaxCommKeepVarCount = 500;   	//�����͹�����������
const int kMaxUserMacroVarCount = 5000;
const int kMaxVarTotalCount = kMaxLocalVarCount+kMaxCommVarCount+kMaxCommKeepVarCount;   	//��ϵͳ������ı�������

struct LocalVarScene{
	bool init[kMaxLocalVarCount];
	double value[kMaxLocalVarCount];

	LocalVarScene &operator =(LocalVarScene &one); 	//��ֵ�����
};

class Variable {
public:
	Variable();
	virtual ~Variable();

	void SetChnIndex(uint8_t chn);   //��������ͨ��

	bool GetVarValue(int index, double &value, bool &init);  	//��ȡ����ֵ������double
	bool GetVarValue(int index, int &value, bool &init);		//��ȡ����ֵ������int

	bool SetVarValue(int index, double value);    	//���ñ���ֵ
	bool SetVarValue(int index, int value);			//���ñ���ֵ
	bool ResetVariable(int index);		//������λ����Ϊ��ֵ

	bool PushLocalVar();		//�ֲ�������ջ
	bool PopLocalVar();		//�ֲ�������ջ

	void Reset();		//���帴λ

	void ResetLocalVar();  //�ֲ�������λ

	bool SaveKeepComm(int index);		//�������ʧ�Թ�������
	bool SaveMacroComm(int index);		//�����û������

//	void Save();   //�رշ���ʧ�Թ����������ļ����

	void Sync(bool close = false);  //ͬ���ļ��޸�

	bool IsKeepChanged(){return this->m_b_save_keep;}   //�Ƿ���Ҫͬ�������ļ�

	int CopyVar(char *buf, uint32_t max_size, uint32_t start_index, uint8_t count);		//������������

	void MemsetMacroVar(int start, int count, double value);
	void InsertMacroVar(int index, int end, double value);
	void PopMacroVar(int index, int end);
	void SetMacroArray(int index, int count, char * buf);

private:
	void InitKeepVar();					//��ʼ������ʧ�Ժ����
	void InitMacroVar();
	bool GetSysVar(int index, double &value);		//��ȡϵͳ������double��
	bool GetSysVar(int index, int &value);		    //��ȡϵͳ������int��
	bool SetSysVar(int index, double value);		//����ϵͳ������double��
	bool SetSysVar(int index, int value);			//����ϵͳ������int��
	void SyncUserMacroVar();

private:
	uint8_t m_n_channel_index;                  //����ͨ��
	bool m_b_init_local[kMaxLocalVarCount];		//�ֲ�������ʼ����־
	bool m_b_init_common[kMaxCommVarCount]; 	//�Ǳ����͹���������ʼ����־
	bool m_b_init_common_keep[kMaxCommKeepVarCount];  //�����͹�������
	bool m_b_init_user_macro[kMaxUserMacroVarCount];  //�û������
	double m_df_local[kMaxLocalVarCount];  		//�ֲ�����
	double m_df_common[kMaxCommVarCount];		//�Ǳ����͹�������
	double m_df_common_keep[kMaxCommKeepVarCount]; 	//�����͹�������
	double m_df_user_macro[kMaxUserMacroVarCount];  //�û������
	DataStack<LocalVarScene> m_stack_local;         //�ֲ����������ջ

	FILE *m_fp_keep_var;  //�����͹����������ļ�����ָ��
	FILE *m_fp_macro_var; //������ļ�����ָ��
	bool m_b_save_keep;  //�Ƿ���Ҫ�������ʧ�����

};

#endif /* VARIABLE_H_ */
