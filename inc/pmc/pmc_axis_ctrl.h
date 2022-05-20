
/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_axis_ctrl.h
 *@author gonghao
 *@date 2021/03/31
 *@brief ��ͷ�ļ�����PMC������������
 *@version
 */

#ifndef INC_PMC_PMC_AXIS_CTRL_H_
#define INC_PMC_PMC_AXIS_CTRL_H_

#include "global_include.h"
#include "pmc_register.h"

class ChannelEngine;
class MICommunication;

const int kMaxPmcAxisCtrlGroup = 4*kMaxChnCount;    //ÿ��ͨ��4�飬�ܹ�16��


//PMC�����ָ��ṹ
struct PmcAxisCtrlCmd{
uint8_t cmd;			//�����
uint16_t speed;			//�ٶ�   ��λ��mm/min
int32_t distance;           //����   ��λ��um
};

class PmcAxisCtrl {
public:
	PmcAxisCtrl();
	virtual ~PmcAxisCtrl();

	bool Active(bool flag);           //�����PMC����ͨ��
	bool IsActive(){return m_b_active;}     //�Ƿ��Ѿ�����
	bool SetGroupIndex(uint8_t index);      //��������ƼĴ������
	bool WriteCmd(PmcAxisCtrlCmd &cmd);     //д��PMCָ��
	void ExecCmdOver(bool res);        //ָ��ִ�����

	uint8_t GetCmdCount(){return this->m_n_cmd_count;}    //���ص�ǰ����������
	bool IsPaused(){return m_b_pause;}    //�Ƿ���ͣ״̬

	void SetBuffState(bool flag);    //���û���״̬   flag��true--������Ч   false--������Ч
	void SetStepStop(bool flag);     //���ó����ֹͣ

	void Reset();          //ִ�и�λ����
	void Pause(bool flag);			//����ͣ

private:
	uint8_t GetRecvBufIndex();   //��õ�ǰ���ջ����������
	void ExecuteCmd();           //ִ�е�ǰִ�л����е�ָ��

private:
	uint8_t m_n_group_index;    //PMC�������ţ�0-15
	uint8_t m_n_phy_axis;       //��Ӧ����������������0��ʼ
	bool m_b_active;           //���������Ƿ񼤻�
	bool m_b_buffer;           //�����Ƿ���Ч    true--��Ч   false--��Ч
	bool m_b_step_stop;        //�����ֹͣ��Ч   true--ֹͣ��Ч    false--ֹͣ��Ч
	bool m_b_pause;				//��ͣ״̬   true--��ͣ��     false--������ͣ״̬
	uint8_t m_n_cmd_count;      //������ָ������
//	uint8_t m_n_buf_recv;       //���뻺����������
//	uint8_t m_n_buf_wait;       //�ȴ�������������
	uint8_t m_n_buf_exec;       //ִ�л�����������
	PmcAxisCtrlCmd m_pmc_cmd_buffer[3];    //PMC��ָ��壬3�����壬���ա��ȴ���ִ��

	ChannelEngine *m_p_channel_engine;    //ͨ������ָ��
	MICommunication *m_p_mi_comm;         //MIͨѶ����ָ��

	FRegBits *m_p_f_reg;
	const GRegBits *m_p_g_reg;

};

#endif /* INC_PMC_PMC_AXIS_CTRL_H_ */
