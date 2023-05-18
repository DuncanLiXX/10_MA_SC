/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_axis_ctrl.cpp
 *@author gonghao
 *@date 2021/03/31
 *@brief ��ͷ�ļ�����PMC������������
 *@version
 */

#include <pmc_axis_ctrl.h>
#include "channel_engine.h"
#include "mi_communication.h"
#include "parameter/parm_manager.h"
#include "parm_definition.h"
#include <functional>
#include <future>
#include "channel_control.h"
#include "spindle_control.h"
#include "sync_axis_ctrl.h"

/**
 * @brief ���캯��
 */
PmcAxisCtrl::PmcAxisCtrl() {
    // TODO Auto-generated constructor stub
    this->m_n_group_index = 0xFF;    //δ��ʼ��
    this->m_b_active = false;
    this->m_p_f_reg = nullptr;
    this->m_p_g_reg = nullptr;
    this->m_b_buffer = true;  //Ĭ�ϻ�����Ч
    m_b_step_stop = false;   //Ĭ����Ч
    m_b_pause = false;      //Ĭ�ϲ�����ͣ״̬

    this->m_p_channel_engine = nullptr;
    this->m_p_mi_comm = nullptr;

    feed_rate = 100;//��ʼ����������Ϊ100%
    speed_rate = 100;//��ʼ�����ٱ���Ϊ100%
    rapid_enable = false;//Ĭ�ϲ����������ƶ�
}

/**
 * @brief ��������
 */
PmcAxisCtrl::~PmcAxisCtrl() {
    // TODO Auto-generated destructor stub
}

/**
 * @brief ��������ƼĴ������
 * @param index : ���, 0-15
 * return true--�ɹ�    false--ʧ��
 */
bool PmcAxisCtrl::SetGroupIndex(uint8_t index){

    if(index >= kMaxPmcAxisCtrlGroup)
        return false;

    this->m_p_channel_engine = ChannelEngine::GetInstance();
    this->m_p_mi_comm = MICommunication::GetInstance();

    this->m_n_group_index = index;
    this->m_p_f_reg = m_p_channel_engine->GetChnFRegBits(index/4);
    this->m_p_g_reg = m_p_channel_engine->GetChnGRegBits(index/4);

    //����ͨ���������
    SCAxisConfig *pAxis = ParmManager::GetInstance()->GetAxisConfig();
    SCSystemConfig *sys = ParmManager::GetInstance()->GetSystemConfig();

    for(uint8_t i = 0; i < sys->axis_count; i++){
        if(pAxis[i].axis_pmc == 0)
            continue;
        if(pAxis[i].axis_pmc == index+1){
            axis_list.push_back(&pAxis[i]);
        }
    }

    switch(this->m_n_group_index%4){
    case 0:
        this->m_p_f_reg->EADEN1 = 1;
        break;
    case 1:
        this->m_p_f_reg->EADEN2 = 1;
        break;
    case 2:
        this->m_p_f_reg->EADEN3 = 1;
        break;
    case 3:
        this->m_p_f_reg->EADEN4 = 1;
        break;
    }

    return true;
}

/**
 * @brief �����PMC����ͨ��
 * @param flag : true--����    false--��ֹ
 * @return true--�ɹ�   false--ʧ��
 */
bool PmcAxisCtrl::Active(bool flag){
    if(this->m_b_active == flag)
        return true;

    if (!CanActive())
        return false;

    printf("PmcAxisCtrl::Active[%hhu]:%d\n", this->m_n_group_index, flag);

    for (auto itr = axis_list.begin(); itr != axis_list.end(); ++itr)
    {
        if ((*itr)->axis_pmc)
        {
            m_b_active = flag;

            if (flag)
            {//����
                m_p_channel_engine->SetPmcActive((*itr)->axis_index);
            }
            else
            {
                m_p_channel_engine->RstPmcActive((*itr)->axis_index);
            }
            //ˢ��Phaser,��֪��ΪʲôҪ������������PMC��״̬
            SCSystemConfig *sys = ParmManager::GetInstance()->GetSystemConfig();
            for (int i = 0; i < sys->chn_count; ++i)
            {
                m_p_channel_engine->GetChnControl(i)->RefreshPmcAxis();
            }

            //֪ͨMI�������л�
            MiCmdFrame cmd;
            memset(&cmd, 0x00, sizeof(cmd));
            cmd.data.cmd = CMD_MI_SET_AXIS_CHN_PHY_MAP;
            cmd.data.reserved = flag ? 0x10 : 0;
            cmd.data.axis_index = (*itr)->axis_index+1;
            cmd.data.data[0] = (*itr)->axis_index+1;
            cmd.data.data[1] = 1;//1��̬�л�,0��ʼ����
            this->m_p_mi_comm->WriteCmd(cmd);
        }
    }

    if(flag){
        // ״̬
        switch(this->m_n_group_index%4){
        case 0:
            this->m_p_f_reg->EADEN1 = 1;
            this->m_p_f_reg->EBSYA = this->m_p_g_reg->EBUFA;
            break;
        case 1:
            this->m_p_f_reg->EADEN2 = 1;
            this->m_p_f_reg->EBSYB = this->m_p_g_reg->EBUFB;
            break;
        case 2:
            this->m_p_f_reg->EADEN3 = 1;
            this->m_p_f_reg->EBSYC = this->m_p_g_reg->EBUFC;
            break;
        case 3:
            this->m_p_f_reg->EADEN4 = 1;
            this->m_p_f_reg->EBSYD = this->m_p_g_reg->EBUFD;
            break;
        }
    }
    return true;
}

bool PmcAxisCtrl::CanActive()
{
    //���_EAXSL�ź�
    FRegBits *freg0 = m_p_channel_engine->GetChnFRegBits(0);
    if(freg0->_EAXSL == 1)
        return false;

    for (int i = 0; i < m_p_channel_engine->GetChnCount(); ++i)
    {
        if (m_p_channel_engine->GetChnControl()[i].IsMachinRunning())//����״̬���ܼ���
        {
            return false;
        }

        for (auto itr = axis_list.begin(); itr != axis_list.end(); ++itr)
        {
            if (m_p_channel_engine->GetChnControl()[i].GetSpdCtrl()->GetPhyAxis()//���᲻�ܼ���
                    == (*itr)->axis_index)
            {
                return false;
            }

            if (m_p_channel_engine->GetSyncAxisCtrl()->CheckSyncState((*itr)->axis_index) == 2)
            {//�Ӷ��᲻��ΪPMC��
                return false;
            }
        }
    }

    return true;
}

/**
 * @brief ���û���״̬
 * @param flag : true--������Ч   false--������Ч
 */
void PmcAxisCtrl::SetBuffState(bool flag){
    if(flag == m_b_buffer)
        return;
    printf("PmcAxisCtrl::SetBuffState[%hhu]:%d\n", m_n_group_index, flag);
    this->m_b_buffer = flag;
}

/**
 * @brief ���ó����ֹͣ
 * @param flag : true--�����ֹͣ��Ч     false--�����ֹͣ��Ч
 */
void PmcAxisCtrl::SetStepStop(bool flag){
    if(flag == this->m_b_step_stop)
        return;

    std::cout << "PmcAxisCtrl::SetStepStop " << (int)flag << std::endl;
    this->m_b_step_stop = flag;
}

/**
 * @brief ִ�и�λ����
 */
void PmcAxisCtrl::Reset(){
    for(unsigned int i = 0; i < axis_list.size(); i++){
        this->m_p_channel_engine->ManualMoveStop(axis_list.at(i)->axis_index);
    }

    while(m_pmc_cmd_buffer.size())
    {
        m_pmc_cmd_buffer.pop();
    }

    this->m_b_pause = false;

    switch(this->m_n_group_index%4){
    case 0:
        this->m_p_f_reg->EACNT1 = 0;
        this->m_p_f_reg->EBSYA = this->m_p_g_reg->EBUFA;
        break;
    case 1:
        this->m_p_f_reg->EACNT2 = 0;
        this->m_p_f_reg->EBSYB = this->m_p_g_reg->EBUFB;
        break;
    case 2:
        this->m_p_f_reg->EACNT3 = 0;
        this->m_p_f_reg->EBSYC = this->m_p_g_reg->EBUFC;
        break;
    case 3:
        this->m_p_f_reg->EACNT4 = 0;
        this->m_p_f_reg->EBSYD = this->m_p_g_reg->EBUFD;
        break;
    }

    //���_EAXSL�ź�
    FRegBits *freg0 = m_p_channel_engine->GetChnFRegBits(0);
    FRegBits *freg = nullptr;
    bool flag = false;
    for(uint8_t i = 0; i < kMaxChnCount; i++){
        freg = m_p_channel_engine->GetChnFRegBits(i);
        if(freg->EACNT1 || freg->EACNT2 || freg->EACNT3 || freg->EACNT4){
            flag = true;
            break;
        }
    }
    if(!flag)
        freg0->_EAXSL = 0;

    switch(this->m_n_group_index%4){
    case 0:
        this->m_p_f_reg->EABUFA = 0;
        this->m_p_f_reg->EIALA = 0;
        break;
    case 1:
        this->m_p_f_reg->EABUFB = 0;
        this->m_p_f_reg->EIALB = 0;
        break;
    case 2:
        this->m_p_f_reg->EABUFC = 0;
        this->m_p_f_reg->EIALC = 0;
        break;
    case 3:
        this->m_p_f_reg->EABUFD = 0;
        this->m_p_f_reg->EIALD = 0;
        break;
    }
}

/**
 * @brief ����ͣ
 * @flag : true--������ͣ      false--ȡ����ͣ
 */
void PmcAxisCtrl::Pause(bool flag){
    //����ͣ
    if(this->m_b_pause == flag)
        return;

    this->m_b_pause = flag;
    for(unsigned int i = 0; i < axis_list.size(); i++){
        this->m_p_channel_engine->PausePmcAxis(axis_list.at(i)->axis_index, flag);
    }

    switch(this->m_n_group_index%4){
    case 0:
        this->m_p_f_reg->EGENA = flag?0:1;
        break;
    case 1:
        this->m_p_f_reg->EGENB = flag?0:1;
        break;
    case 2:
        this->m_p_f_reg->EGENC = flag?0:1;
        break;
    case 3:
        this->m_p_f_reg->EGEND = flag?0:1;
        break;
    }
}

/**
 * @brief ��ȡ�Ѽ����PMC��
 * @return �������
 */
std::vector<uint64_t> PmcAxisCtrl::GetActivePmcAxis()
{
    std::vector<uint64_t> vecAxis;
    if (m_b_active) {
        for (auto itr = axis_list.begin(); itr != axis_list.end(); ++itr)
        {
            if ((*itr)->axis_pmc) {
                vecAxis.push_back((*itr)->axis_index);
            }
        }
    }
    return vecAxis;
}

/**
 * @brief ���³����ֹͣ�ź�
 * @esbk ����ֹͣ�ź�
 * @mesbk ����ֹͣ�����ź�
 */
void PmcAxisCtrl::InputSBK(uint8_t esbk, uint8_t mesbk)
{
    if (mesbk || !esbk)
        SetStepStop(false);
    if (esbk)
        SetStepStop(true);
}

/**
 * @brief ����PMC���������
 * @param value ���ʣ��ٷֱ�
 */
void PmcAxisCtrl::SetFeedValue(uint8_t value)
{
    if (feed_rate == value)
        return;
    feed_rate = value;
}

/**
 * @brief ��ȡPMC���������
 * @return ���ʣ��ٷֱ�
 */
uint8_t PmcAxisCtrl::GetFeedValue() const
{
    return feed_rate;
}

/**
 * @brief ����PMC������ƶ�����
 * @param value ���ʣ��ٷֱ�
 */
void PmcAxisCtrl::SetSpeedValue(int value)
{
    if (speed_rate == value)
        return;
    speed_rate = value;
}

/**
 * @brief ��ȡPMC������ƶ�����
 * @return ���ʣ��ٷֱ�
 */
int PmcAxisCtrl::GetSpeedValue() const
{
    return speed_rate;
}

/**
 * @brief �����Ƿ��������ƶ�
 * @param true:������false:�ر�
 */
void PmcAxisCtrl::SetRapidValue(bool value)
{
    if (rapid_enable == value)
        return;
    rapid_enable = value;
}

/**
 * @brief ��ȡ�Ƿ��������ƶ�
 * @return true:������false:�ر�
 */
bool PmcAxisCtrl::GetRapid() const
{
    return rapid_enable;
}


/**
 * @brief ��ȡPMCָ���ָ�����
 * @param cmd
 * @return
 */
bool PmcAxisCtrl::ReadCmd(PmcAxisCtrlCmd &cmd)
{
    if(!this->m_b_buffer && m_pmc_cmd_buffer.size() > 0)
    {//û�л�����
        return false;
    }
    if(m_pmc_cmd_buffer.size() >= 3 /*|| !this->IsActive()*/)  //��������,����δ����
    {
        return false;
    }

    if (m_pmc_cmd_buffer.size() != 0 && cmd.cmd == 5)
    {
        return false;
    }

    m_pmc_cmd_buffer.push(cmd);
    std::cout << "PmcAxisCtrl::ReadCmd: " << (int)cmd.cmd << " cur_size: " << m_pmc_cmd_buffer.size() << std::endl;

    bool eabuf = (m_pmc_cmd_buffer.size() >= 3);
    switch(this->m_n_group_index%4){
    case 0:
        //��ǰ���㣬��ֹ���̴���
        //this->m_p_f_reg->EINPA = 0;
        this->m_p_f_reg->EBSYA = m_p_f_reg->EBSYA?0:1;
        this->m_p_f_reg->EABUFA = eabuf;
        break;
    case 1:
        //this->m_p_f_reg->EINPB = 0;
        this->m_p_f_reg->EBSYB = m_p_f_reg->EBSYB?0:1;
        this->m_p_f_reg->EABUFB = eabuf;
        break;
    case 2:
        //this->m_p_f_reg->EINPC = 0;
        this->m_p_f_reg->EBSYC = m_p_f_reg->EBSYC?0:1;
        this->m_p_f_reg->EABUFC = eabuf;
        break;
    case 3:
        //this->m_p_f_reg->EINPD = 0;
        this->m_p_f_reg->EBSYD = m_p_f_reg->EBSYD?0:1;
        this->m_p_f_reg->EABUFD = eabuf;
        break;
    }

    return true;
}


/**
 * @brief д��PMCָ��
 * @param cmd
 * @return   true--�ɹ�    false--ʧ��
 */
bool PmcAxisCtrl::WriteCmd(){

    if (m_pmc_cmd_buffer.empty() || m_b_step_stop)//û�п�д�������
    {
        return false;
    }

    //��ǰ���Ƿ񼤻�״̬
    if (!IsActive())
    {
        SetErrState(0);
        return false;
    }

    bool cmd_executing = true;
    switch(this->m_n_group_index%4){
    case 0:
        cmd_executing = this->m_p_f_reg->EACNT1;
        break;
    case 1:
        cmd_executing = this->m_p_f_reg->EACNT2;
        break;
    case 2:
        cmd_executing = this->m_p_f_reg->EACNT3;
        break;
    case 3:
        cmd_executing = this->m_p_f_reg->EACNT4;
        break;
    }

    //��һ��ָ�û��ִ�����
    if (cmd_executing)
    {
        return false;
    }

    FRegBits *chn0_freg = m_p_channel_engine->GetChnFRegBits(0);
    switch(this->m_n_group_index%4){
    case 0:
        this->m_p_f_reg->EACNT1 = 1;
        chn0_freg->_EAXSL = 1;
        break;
    case 1:
        this->m_p_f_reg->EACNT2 = 1;
        chn0_freg->_EAXSL = 1;
        break;
    case 2:
        this->m_p_f_reg->EACNT3 = 1;
        chn0_freg->_EAXSL = 1;
        break;
    case 3:
        this->m_p_f_reg->EACNT4 = 1;
        chn0_freg->_EAXSL = 1;
        break;
    }

    this->ExecuteCmd();
    return true;
}

/**
 * @brief ָ��ִ�����
 * @param res : ִ�н��  true--�ɹ�   false--ʧ��
 */
void PmcAxisCtrl::ExecCmdOver(bool res){
    if (++axis_over_cnt < axis_list.size())
    {//�ȴ�ͬ��������ᶼִ����ɣ�����ָ�����������ִ�����
        std::cout << "group over_cnt: " << (int)axis_over_cnt << " size: " << axis_list.size() << std::endl;
        return;
    }
    printf("PmcAxisCtrl::ExecCmdOver()\n");

    switch(this->m_n_group_index%4){
    case 0:
        this->m_p_f_reg->EACNT1 = 0;
        this->m_p_f_reg->EINPA = 1;
        this->m_p_f_reg->EIALA = res?0:1;
        break;
    case 1:
        this->m_p_f_reg->EACNT2 = 0;
        this->m_p_f_reg->EINPB = 1;
        this->m_p_f_reg->EIALB = res?0:1;
        break;
    case 2:
        this->m_p_f_reg->EACNT3 = 0;
        this->m_p_f_reg->EINPC = 1;
        this->m_p_f_reg->EIALC = res?0:1;
        break;
    case 3:
        this->m_p_f_reg->EACNT4 = 0;
        this->m_p_f_reg->EINPD = 1;
        this->m_p_f_reg->EIALD = res?0:1;
        break;
    }

    //���_EAXSL�ź�
    FRegBits *freg0 = m_p_channel_engine->GetChnFRegBits(0);
    FRegBits *freg = nullptr;
    bool flag = false;
    for(uint8_t i = 0; i < kMaxChnCount; i++){
        freg = m_p_channel_engine->GetChnFRegBits(i);
        if(freg->EACNT1 || freg->EACNT2 || freg->EACNT3 || freg->EACNT4){
            flag = true;
            break;
        }
    }
    if(!flag)
        freg0->_EAXSL = 0;
}

void PmcAxisCtrl::SetErrState(int id)
{
    switch(this->m_n_group_index%4){
    case 0:
        this->m_p_f_reg->EBSYA = this->m_p_f_reg->EBSYA?0:1;
        break;
    case 1:
        this->m_p_f_reg->EBSYB = this->m_p_f_reg->EBSYB?0:1;
        break;
    case 2:
        this->m_p_f_reg->EBSYC = this->m_p_f_reg->EBSYC?0:1;
        break;
    case 3:
        this->m_p_f_reg->EBSYD = this->m_p_f_reg->EBSYD?0:1;
        break;
    }
    CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, id, CHANNEL_ENGINE_INDEX);
}

/**
 * @brief ִ�е�ǰִ�л����е�ָ��
 */
void PmcAxisCtrl::ExecuteCmd(){

    axis_over_cnt = 0;
    PmcAxisCtrlCmd pmcAxiscmd = m_pmc_cmd_buffer.front();
    m_pmc_cmd_buffer.pop();

    for(unsigned int i = 0; i < axis_list.size(); i++){
        SCAxisConfig *axis = axis_list.at(i);
        uint8_t cmd = pmcAxiscmd.cmd;   //ָ��

        std::cout << "PmcAxisCtrl::ExecuteCmd " << (int)cmd << std::endl;
        if (!g_ptr_chn_engine->GetAxisRetRefFlag(axis->axis_index) && cmd != 0x05)
        {//�ж����Ƿ����ο���
            CreateError(ERR_AXIS_REF_NONE,
                ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, axis->axis_index);
            break;
        }

        if(cmd == 0x00 || cmd == 0x01 || cmd == 0x10 || cmd == 0x11){  //���ٶ�λ����������
            uint32_t speed = pmcAxiscmd.speed;              //�ٶȣ���λת��
            int64_t dis = pmcAxiscmd.distance*1e4;          //�ƶ����룬��λת����um-->0.1nm

            if (cmd == 0x00 || cmd == 0x10)
            {
                rapid_enable ? speed = axis->rapid_speed : speed = axis->manual_speed;
                speed = speed * double(feed_rate / 100.) ;
            }
            else
            {
                speed = speed * double(feed_rate / 100.) ;
            }

            if(speed < axis->pmc_min_speed || speed > axis->pmc_max_speed){
                CreateError(ERR_PMC_SPEED_ERROR,
                            ERROR_LEVEL,
                            CLEAR_BY_MCP_RESET,
                            0,CHANNEL_ENGINE_INDEX,axis->axis_index);
                break;
            }

            speed = speed * 1000/60; // ��λת����mm/min-->um/s

            PmcCmdFrame pmc_cmd;
            pmc_cmd.data.cmd = 0x0100;   //��������ģʽ
            pmc_cmd.data.axis_index = axis->axis_index+1;
            pmc_cmd.data.axis_index |= 0xFF00;      //��־ͨ������
            pmc_cmd.data.data[0] = 0;
            memcpy(&pmc_cmd.data.data[1], &dis, sizeof(dis));
            memcpy(&pmc_cmd.data.data[5], &speed, sizeof(speed));

            if(cmd == 0x10 || cmd == 0x11){
                pmc_cmd.data.cmd = 0x00;    //��������ģʽ
            }

            printf("pmc axis execute cmd: speed = %u, dis = %lld\n", speed, dis);
            this->m_p_channel_engine->SendPmcAxisCmd(pmc_cmd);

            //����״̬
            switch(this->m_n_group_index%4){
            case 0:
                this->m_p_f_reg->EADEN1 = 0;   //��������ź�
                this->m_p_f_reg->EGENA = 1;    //���ƶ��ź�
                this->m_p_f_reg->EINPA = 0;    //�ᵽλ�ź�
                break;
            case 1:
                this->m_p_f_reg->EADEN2 = 0;
                this->m_p_f_reg->EGENB = 1;
                this->m_p_f_reg->EINPB = 0;
                break;
            case 2:
                this->m_p_f_reg->EADEN3 = 0;
                this->m_p_f_reg->EGENC = 1;
                this->m_p_f_reg->EINPC = 0;
                break;
            case 3:
                this->m_p_f_reg->EADEN4 = 0;
                this->m_p_f_reg->EGEND = 1;
                this->m_p_f_reg->EINPD = 0;
                break;
            }
        }else if(cmd == 0x04){   //��ͣ  G04
            //����״̬
            switch(this->m_n_group_index%4){
            case 0:
                this->m_p_f_reg->EADEN1 = 0;   //��������ź�
                this->m_p_f_reg->EINPA = 0;    //�ᵽλ�ź�
                break;
            case 1:
                this->m_p_f_reg->EADEN2 = 0;
                this->m_p_f_reg->EINPB = 0;
                break;
            case 2:
                this->m_p_f_reg->EADEN3 = 0;
                this->m_p_f_reg->EINPC = 0;
                break;
            case 3:
                this->m_p_f_reg->EADEN4 = 0;
                this->m_p_f_reg->EINPD = 0;
                break;
            }
            uint32_t ms = pmcAxiscmd.distance;
            std::thread wait(&PmcAxisCtrl::Process04Cmd, this, ms);
            wait.detach();
        }else if(cmd == 0x05){   //�زο��㶯��
            if (!g_ptr_chn_engine->SetPmcRetRef(axis->axis_index, feed_rate))
            {
                uint32_t speed = axis->rapid_speed;        //�ٶȣ���λת��
                speed = speed * double(speed_rate) / 100.;

                int64_t dis = axis->axis_home_pos[0]*1e7;       //�ƶ����룬��λת����mm-->0.1nm

                if(speed < axis->pmc_min_speed || speed > axis->pmc_max_speed){
                    CreateError(ERR_PMC_SPEED_ERROR,
                                ERROR_LEVEL,
                                CLEAR_BY_MCP_RESET,
                                0,CHANNEL_ENGINE_INDEX,axis->axis_index);
                    break;
                }

                speed = speed * 1000/60; // ��λת����mm/min-->um/s
                ScPrintf("Pmc cmd5, go home: speed = %u, dis=%lld",speed, dis);

                PmcCmdFrame pmc_cmd;
                pmc_cmd.data.cmd = 0x00;   //��������ģʽ
                pmc_cmd.data.axis_index = axis->axis_index+1;
                pmc_cmd.data.axis_index |= 0xFF00;      //��־ͨ������
                pmc_cmd.data.data[0] = 0;
                memcpy(&pmc_cmd.data.data[1], &dis, sizeof(dis));
                memcpy(&pmc_cmd.data.data[5], &speed, sizeof(speed));
                this->m_p_channel_engine->SendPmcAxisCmd(pmc_cmd);
            }

            //����״̬
            switch(this->m_n_group_index%4){
            case 0:
                this->m_p_f_reg->EADEN1 = 0;   //��������ź�
                this->m_p_f_reg->EGENA = 1;    //���ƶ��ź�
                this->m_p_f_reg->EINPA = 0;    //�ᵽλ�ź�
                break;
            case 1:
                this->m_p_f_reg->EADEN2 = 0;
                this->m_p_f_reg->EGENB = 1;
                this->m_p_f_reg->EINPB = 0;
                break;
            case 2:
                this->m_p_f_reg->EADEN3 = 0;
                this->m_p_f_reg->EGENC = 1;
                this->m_p_f_reg->EINPC = 0;
                break;
            case 3:
                this->m_p_f_reg->EADEN4 = 0;
                this->m_p_f_reg->EGEND = 1;
                this->m_p_f_reg->EINPD = 0;
                break;
            }
        }else if(cmd == 0x07){
            //ʹ�þ���ʽ
            uint32_t speed = axis->rapid_speed;        //�ٶȣ���λת��
            speed = speed * double(speed_rate) / 100.;
            //double cur_pos = m_p_channel_engine->GetPhyAxisMachPosFeedback(axis->axis_index);
            int64_t dis = axis->axis_home_pos[0]*1e7;       //�ƶ����룬��λת����mm-->0.1nm

            ScPrintf("Pmc cmd7, go home: speed = %u, dis=%lld",speed, dis);
            if(speed < axis->pmc_min_speed || speed > axis->pmc_max_speed)
            {
                CreateError(ERR_PMC_SPEED_ERROR,
                            ERROR_LEVEL,
                            CLEAR_BY_MCP_RESET,
                            0,CHANNEL_ENGINE_INDEX,axis->axis_index);
                break;
            }

            speed = speed * 1000/60; // ��λת����mm/min-->um/s

            PmcCmdFrame pmc_cmd;
            pmc_cmd.data.cmd = 0x00;   //��������ģʽ
            pmc_cmd.data.axis_index = axis->axis_index+1;
            pmc_cmd.data.axis_index |= 0xFF00;      //��־ͨ������
            pmc_cmd.data.data[0] = 0;
            memcpy(&pmc_cmd.data.data[1], &dis, sizeof(dis));
            memcpy(&pmc_cmd.data.data[5], &speed, sizeof(speed));
            this->m_p_channel_engine->SendPmcAxisCmd(pmc_cmd);


            switch(this->m_n_group_index%4){
            case 0:
                this->m_p_f_reg->EADEN1 = 0;   //��������ź�
                this->m_p_f_reg->EGENA = 1;    //���ƶ��ź�
                this->m_p_f_reg->EINPA = 0;    //�ᵽλ�ź�
                break;
            case 1:
                this->m_p_f_reg->EADEN2 = 0;
                this->m_p_f_reg->EGENB = 1;
                this->m_p_f_reg->EINPB = 0;
                break;
            case 2:
                this->m_p_f_reg->EADEN3 = 0;
                this->m_p_f_reg->EGENC = 1;
                this->m_p_f_reg->EINPC = 0;
                break;
            case 3:
                this->m_p_f_reg->EADEN4 = 0;
                this->m_p_f_reg->EGEND = 1;
                this->m_p_f_reg->EINPD = 0;
                break;
            }
        }else if(cmd == 0x06){    //JOG��������������JOG
            uint32_t speed = pmcAxiscmd.speed*1000/60;   //�ٶȣ���λת����mm/min-->um/s
            speed = speed * double(feed_rate / 100.);
            int64_t dir = (pmcAxiscmd.distance == 0) ? -1 : 1;
            int64_t dis = dir * 9999 * 1e7;

            PmcCmdFrame pmc_cmd;
            pmc_cmd.data.cmd = 0x0100;   //��������ģʽ
            pmc_cmd.data.axis_index = axis->axis_index+1;
            pmc_cmd.data.axis_index |= 0xFF00;      //��־ͨ������
            pmc_cmd.data.data[0] = 0;
            memcpy(&pmc_cmd.data.data[1], &dis, sizeof(dis));
            memcpy(&pmc_cmd.data.data[5], &speed, sizeof(speed));

            //		printf("pmc axis execute cmd: speed = %u, dis = %lld\n", speed, dis);

            this->m_p_channel_engine->SendPmcAxisCmd(pmc_cmd);

            //JOGָ�����
//            this->m_n_cmd_count--;
//            this->m_n_buf_exec++;
//            if(this->m_n_buf_exec == 3)
//                this->m_n_buf_exec = 0;

            //����״̬
            switch(this->m_n_group_index%4){
            case 0:
                this->m_p_f_reg->EADEN1 = 0;   //��������ź�
                this->m_p_f_reg->EGENA = 1;    //���ƶ��ź�
                this->m_p_f_reg->EINPA = 0;    //�ᵽλ�ź�
                break;
            case 1:
                this->m_p_f_reg->EADEN2 = 0;
                this->m_p_f_reg->EGENB = 1;
                this->m_p_f_reg->EINPB = 0;
                break;
            case 2:
                this->m_p_f_reg->EADEN3 = 0;
                this->m_p_f_reg->EGENC = 1;
                this->m_p_f_reg->EINPC = 0;
                break;
            case 3:
                this->m_p_f_reg->EADEN4 = 0;
                this->m_p_f_reg->EGEND = 1;
                this->m_p_f_reg->EINPD = 0;
                break;
            }
        }
    }

    switch(this->m_n_group_index%4){
    case 0:
        this->m_p_f_reg->EABUFA = 0;   //�������źŸ�λ
        break;
    case 1:
        this->m_p_f_reg->EABUFB = 0;
        break;
    case 2:
        this->m_p_f_reg->EABUFC = 0;
        break;
    case 3:
        this->m_p_f_reg->EABUFD = 0;
        break;
    }
}

void PmcAxisCtrl::Process04Cmd(uint32_t ms){
    std::this_thread::sleep_for(std::chrono::microseconds(ms * 1000));

    bool bOver = false;
    switch(this->m_n_group_index%4){
    case 0:
        if (this->m_p_f_reg->EACNT1)
            bOver = true;
        break;
    case 1:
        if (this->m_p_f_reg->EACNT2)
            bOver = true;
        break;
    case 2:
        if (this->m_p_f_reg->EACNT3)
            bOver = true;
        break;
    case 3:
        if (this->m_p_f_reg->EACNT4)
            bOver = true;
        break;
    }
    if (bOver)
        this->ExecCmdOver(true);
}
