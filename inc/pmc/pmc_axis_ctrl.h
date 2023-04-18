
/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_axis_ctrl.h
 *@author gonghao
 *@date 2021/03/31
 *@brief 本头文件包含PMC轴控制类的声明
 *@version
 */

#ifndef INC_PMC_PMC_AXIS_CTRL_H_
#define INC_PMC_PMC_AXIS_CTRL_H_

#include "global_include.h"
#include "pmc_register.h"
#include <vector>

class ChannelEngine;
class MICommunication;

const int kMaxPmcAxisCtrlGroup = (4*kMaxChnCount);    //每个通道4组，总共16组


//PMC轴控制指令结构
struct PmcAxisCtrlCmd{
uint8_t cmd;			//命令号
uint16_t speed;			//速度   单位：mm/min
int32_t distance;           //距离   单位：um
};

class PmcAxisCtrl {
public:
	PmcAxisCtrl();
	virtual ~PmcAxisCtrl();

	bool Active(bool flag);           //激活此PMC控制通道
	bool IsActive(){return m_b_active;}     //是否已经激活
    bool CanActive();                       //是否能够激活
	bool SetGroupIndex(uint8_t index);      //设置轴控制寄存器组号
	bool WriteCmd(PmcAxisCtrlCmd &cmd);     //写入PMC指令
    void ExecCmdOver(bool res);        //指令执行完毕
    void SetErrState(int id);                //设置为错误状态

	uint8_t GetCmdCount(){return this->m_n_cmd_count;}    //返回当前缓冲命令数
	bool IsPaused(){return m_b_pause;}    //是否暂停状态

	void SetBuffState(bool flag);    //设置缓冲状态   flag：true--缓冲有效   false--缓冲无效
	void SetStepStop(bool flag);     //设置程序段停止

    void Reset();                   //执行复位动作
	void Pause(bool flag);			//轴暂停
    std::vector<uint64_t> GetActivePmcAxis();
    void InputSBK(uint8_t esbk, uint8_t mesbk);                //更新程序段停止信号

    void SetFeedValue(uint8_t value);
    uint8_t GetFeedValue() const;
    void SetSpeedValue(int value);
    int GetSpeedValue() const;
    void SetRapidValue(bool value);
    bool GetRapid() const;
private:
	uint8_t GetRecvBufIndex();   //获得当前接收缓冲的索引号
	void ExecuteCmd();           //执行当前执行缓冲中的指令
    void Process04Cmd(uint32_t ms);   //异步执行延时流程

    size_t axis_over_cnt = 0;

    uint8_t feed_rate;//进给倍率
    int speed_rate;//快进倍率
    bool rapid_enable;//快速移动

public:
	uint8_t m_n_group_index;    //PMC轴控制组号，0-15
	bool m_b_active;           //本控制组是否激活
	bool m_b_buffer;           //缓冲是否有效    true--有效   false--无效
	bool m_b_step_stop;        //程序段停止有效   true--停止有效    false--停止无效
    bool m_b_pause;				//暂停状态   true--暂停中     false--不在暂停状态
	uint8_t m_n_cmd_count;      //缓冲中指令数量
//	uint8_t m_n_buf_recv;       //输入缓冲区索引号
//	uint8_t m_n_buf_wait;       //等待缓冲区索引号
	uint8_t m_n_buf_exec;       //执行缓冲区索引号
	PmcAxisCtrlCmd m_pmc_cmd_buffer[3];    //PMC轴指令缓冲，3级缓冲，接收、等待、执行

	ChannelEngine *m_p_channel_engine;    //通道引擎指针
    MICommunication *m_p_mi_comm;         //MI通讯对象指针
    std::vector<SCAxisConfig *> axis_list;   //控制的轴的轴配置（数组，代表该控制组下的轴配置）

	FRegBits *m_p_f_reg;
	const GRegBits *m_p_g_reg;

};

#endif /* INC_PMC_PMC_AXIS_CTRL_H_ */
