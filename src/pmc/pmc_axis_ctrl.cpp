/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_axis_ctrl.cpp
 *@author gonghao
 *@date 2021/03/31
 *@brief 本头文件包含PMC轴控制类的声明
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
 * @brief 构造函数
 */
PmcAxisCtrl::PmcAxisCtrl() {
    // TODO Auto-generated constructor stub
    this->m_n_group_index = 0xFF;    //未初始化
    this->m_b_active = false;
    this->m_p_f_reg = nullptr;
    this->m_p_g_reg = nullptr;
    this->m_b_buffer = true;  //默认缓冲有效
    m_b_step_stop = false;   //默认无效
    m_b_pause = false;      //默认不在暂停状态

    this->m_p_channel_engine = nullptr;
    this->m_p_mi_comm = nullptr;

    feed_rate = 100;//初始化进给倍率为100%
    speed_rate = 100;//初始化快速倍率为100%
    rapid_enable = false;//默认不开启快速移动
}

/**
 * @brief 析构函数
 */
PmcAxisCtrl::~PmcAxisCtrl() {
    // TODO Auto-generated destructor stub
}

/**
 * @brief 设置轴控制寄存器组号
 * @param index : 组号, 0-15
 * return true--成功    false--失败
 */
bool PmcAxisCtrl::SetGroupIndex(uint8_t index){

    if(index >= kMaxPmcAxisCtrlGroup)
        return false;

    this->m_p_channel_engine = ChannelEngine::GetInstance();
    this->m_p_mi_comm = MICommunication::GetInstance();

    this->m_n_group_index = index;
    this->m_p_f_reg = m_p_channel_engine->GetChnFRegBits(index/4);
    this->m_p_g_reg = m_p_channel_engine->GetChnGRegBits(index/4);

    //设置通道物理轴号
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
 * @brief 激活此PMC控制通道
 * @param flag : true--激活    false--禁止
 * @return true--成功   false--失败
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
            {//激活
                m_p_channel_engine->SetPmcActive((*itr)->axis_index);
            }
            else
            {
                m_p_channel_engine->RstPmcActive((*itr)->axis_index);
            }
            //刷新Phaser,不知道为什么要给解析器发送PMC轴状态
            SCSystemConfig *sys = ParmManager::GetInstance()->GetSystemConfig();
            for (int i = 0; i < sys->chn_count; ++i)
            {
                m_p_channel_engine->GetChnControl(i)->RefreshPmcAxis();
            }

            //通知MI进行轴切换
            MiCmdFrame cmd;
            memset(&cmd, 0x00, sizeof(cmd));
            cmd.data.cmd = CMD_MI_SET_AXIS_CHN_PHY_MAP;
            cmd.data.reserved = flag ? 0x10 : 0;
            cmd.data.axis_index = (*itr)->axis_index+1;
            cmd.data.data[0] = (*itr)->axis_index+1;
            cmd.data.data[1] = 1;//1动态切换,0初始配置
            this->m_p_mi_comm->WriteCmd(cmd);
        }
    }

    if(flag){
        // 状态
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
    //检查_EAXSL信号
    FRegBits *freg0 = m_p_channel_engine->GetChnFRegBits(0);
    if(freg0->_EAXSL == 1)
        return false;

    for (int i = 0; i < m_p_channel_engine->GetChnCount(); ++i)
    {
        if (m_p_channel_engine->GetChnControl()[i].IsMachinRunning())//运行状态不能激活
        {
            return false;
        }

        for (auto itr = axis_list.begin(); itr != axis_list.end(); ++itr)
        {
            if (m_p_channel_engine->GetChnControl()[i].GetSpdCtrl()->GetPhyAxis()//主轴不能激活
                    == (*itr)->axis_index)
            {
                return false;
            }

            if (m_p_channel_engine->GetSyncAxisCtrl()->CheckSyncState((*itr)->axis_index) == 2)
            {//从动轴不能为PMC轴
                return false;
            }
        }
    }

    return true;
}

/**
 * @brief 设置缓冲状态
 * @param flag : true--缓冲有效   false--缓冲无效
 */
void PmcAxisCtrl::SetBuffState(bool flag){
    if(flag == m_b_buffer)
        return;
    printf("PmcAxisCtrl::SetBuffState[%hhu]:%d\n", m_n_group_index, flag);
    this->m_b_buffer = flag;
}

/**
 * @brief 设置程序段停止
 * @param flag : true--程序段停止有效     false--程序段停止无效
 */
void PmcAxisCtrl::SetStepStop(bool flag){
    if(flag == this->m_b_step_stop)
        return;

    std::cout << "PmcAxisCtrl::SetStepStop " << (int)flag << std::endl;
    this->m_b_step_stop = flag;
}

/**
 * @brief 执行复位动作
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

    //检查_EAXSL信号
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
 * @brief 轴暂停
 * @flag : true--激活暂停      false--取消暂停
 */
void PmcAxisCtrl::Pause(bool flag){
    //轴暂停
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
 * @brief 获取已激活的PMC轴
 * @return 物理轴号
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
 * @brief 更新程序段停止信号
 * @esbk 程序停止信号
 * @mesbk 程序停止屏蔽信号
 */
void PmcAxisCtrl::InputSBK(uint8_t esbk, uint8_t mesbk)
{
    if (mesbk || !esbk)
        SetStepStop(false);
    if (esbk)
        SetStepStop(true);
}

/**
 * @brief 设置PMC轴进给倍率
 * @param value 倍率，百分比
 */
void PmcAxisCtrl::SetFeedValue(uint8_t value)
{
    if (feed_rate == value)
        return;
    feed_rate = value;
}

/**
 * @brief 获取PMC轴进给倍率
 * @return 倍率，百分比
 */
uint8_t PmcAxisCtrl::GetFeedValue() const
{
    return feed_rate;
}

/**
 * @brief 设置PMC轴快速移动倍率
 * @param value 倍率，百分比
 */
void PmcAxisCtrl::SetSpeedValue(int value)
{
    if (speed_rate == value)
        return;
    speed_rate = value;
}

/**
 * @brief 获取PMC轴快速移动倍率
 * @return 倍率，百分比
 */
int PmcAxisCtrl::GetSpeedValue() const
{
    return speed_rate;
}

/**
 * @brief 设置是否开启快速移动
 * @param true:开启，false:关闭
 */
void PmcAxisCtrl::SetRapidValue(bool value)
{
    if (rapid_enable == value)
        return;
    rapid_enable = value;
}

/**
 * @brief 获取是否开启快速移动
 * @return true:开启，false:关闭
 */
bool PmcAxisCtrl::GetRapid() const
{
    return rapid_enable;
}


/**
 * @brief 读取PMC指令，到指令缓存中
 * @param cmd
 * @return
 */
bool PmcAxisCtrl::ReadCmd(PmcAxisCtrlCmd &cmd)
{
    if(!this->m_b_buffer && m_pmc_cmd_buffer.size() > 0)
    {//没有缓冲区
        return false;
    }
    if(m_pmc_cmd_buffer.size() >= 3 /*|| !this->IsActive()*/)  //缓冲已满,或者未激活
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
        //提前至零，防止流程错乱
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
 * @brief 写入PMC指令
 * @param cmd
 * @return   true--成功    false--失败
 */
bool PmcAxisCtrl::WriteCmd(){

    if (m_pmc_cmd_buffer.empty() || m_b_step_stop)//没有可写入的命令
    {
        return false;
    }

    //当前轴是否激活状态
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

    //上一条指令还没有执行完成
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
 * @brief 指令执行完毕
 * @param res : 执行结果  true--成功   false--失败
 */
void PmcAxisCtrl::ExecCmdOver(bool res){
    if (++axis_over_cnt < axis_list.size())
    {//等待同组的所有轴都执行完成，这条指令才算真正的执行完毕
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

    //检查_EAXSL信号
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
 * @brief 执行当前执行缓冲中的指令
 */
void PmcAxisCtrl::ExecuteCmd(){

    axis_over_cnt = 0;
    PmcAxisCtrlCmd pmcAxiscmd = m_pmc_cmd_buffer.front();
    m_pmc_cmd_buffer.pop();

    for(unsigned int i = 0; i < axis_list.size(); i++){
        SCAxisConfig *axis = axis_list.at(i);
        uint8_t cmd = pmcAxiscmd.cmd;   //指令

        std::cout << "PmcAxisCtrl::ExecuteCmd " << (int)cmd << std::endl;
        if (!g_ptr_chn_engine->GetAxisRetRefFlag(axis->axis_index) && cmd != 0x05)
        {//判断轴是否建立参考点
            CreateError(ERR_AXIS_REF_NONE,
                ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, axis->axis_index);
            break;
        }

        if(cmd == 0x00 || cmd == 0x01 || cmd == 0x10 || cmd == 0x11){  //快速定位、切削进给
            uint32_t speed = pmcAxiscmd.speed;              //速度，单位转换
            int64_t dis = pmcAxiscmd.distance*1e4;          //移动距离，单位转换：um-->0.1nm

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

            speed = speed * 1000/60; // 单位转换：mm/min-->um/s

            PmcCmdFrame pmc_cmd;
            pmc_cmd.data.cmd = 0x0100;   //增量坐标模式
            pmc_cmd.data.axis_index = axis->axis_index+1;
            pmc_cmd.data.axis_index |= 0xFF00;      //标志通道引擎
            pmc_cmd.data.data[0] = 0;
            memcpy(&pmc_cmd.data.data[1], &dis, sizeof(dis));
            memcpy(&pmc_cmd.data.data[5], &speed, sizeof(speed));

            if(cmd == 0x10 || cmd == 0x11){
                pmc_cmd.data.cmd = 0x00;    //绝对坐标模式
            }

            printf("pmc axis execute cmd: speed = %u, dis = %lld\n", speed, dis);
            this->m_p_channel_engine->SendPmcAxisCmd(pmc_cmd);

            //设置状态
            switch(this->m_n_group_index%4){
            case 0:
                this->m_p_f_reg->EADEN1 = 0;   //分配完成信号
                this->m_p_f_reg->EGENA = 1;    //轴移动信号
                this->m_p_f_reg->EINPA = 0;    //轴到位信号
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
        }else if(cmd == 0x04){   //暂停  G04
            //设置状态
            switch(this->m_n_group_index%4){
            case 0:
                this->m_p_f_reg->EADEN1 = 0;   //分配完成信号
                this->m_p_f_reg->EINPA = 0;    //轴到位信号
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
        }else if(cmd == 0x05){   //回参考点动作
            if (!g_ptr_chn_engine->SetPmcRetRef(axis->axis_index, feed_rate))
            {
                uint32_t speed = axis->rapid_speed;        //速度，单位转换
                speed = speed * double(speed_rate) / 100.;

                int64_t dis = axis->axis_home_pos[0]*1e7;       //移动距离，单位转换：mm-->0.1nm

                if(speed < axis->pmc_min_speed || speed > axis->pmc_max_speed){
                    CreateError(ERR_PMC_SPEED_ERROR,
                                ERROR_LEVEL,
                                CLEAR_BY_MCP_RESET,
                                0,CHANNEL_ENGINE_INDEX,axis->axis_index);
                    break;
                }

                speed = speed * 1000/60; // 单位转换：mm/min-->um/s
                ScPrintf("Pmc cmd5, go home: speed = %u, dis=%lld",speed, dis);

                PmcCmdFrame pmc_cmd;
                pmc_cmd.data.cmd = 0x00;   //绝对坐标模式
                pmc_cmd.data.axis_index = axis->axis_index+1;
                pmc_cmd.data.axis_index |= 0xFF00;      //标志通道引擎
                pmc_cmd.data.data[0] = 0;
                memcpy(&pmc_cmd.data.data[1], &dis, sizeof(dis));
                memcpy(&pmc_cmd.data.data[5], &speed, sizeof(speed));
                this->m_p_channel_engine->SendPmcAxisCmd(pmc_cmd);
            }

            //设置状态
            switch(this->m_n_group_index%4){
            case 0:
                this->m_p_f_reg->EADEN1 = 0;   //分配完成信号
                this->m_p_f_reg->EGENA = 1;    //轴移动信号
                this->m_p_f_reg->EINPA = 0;    //轴到位信号
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
            //使用绝对式
            uint32_t speed = axis->rapid_speed;        //速度，单位转换
            speed = speed * double(speed_rate) / 100.;
            //double cur_pos = m_p_channel_engine->GetPhyAxisMachPosFeedback(axis->axis_index);
            int64_t dis = axis->axis_home_pos[0]*1e7;       //移动距离，单位转换：mm-->0.1nm

            ScPrintf("Pmc cmd7, go home: speed = %u, dis=%lld",speed, dis);
            if(speed < axis->pmc_min_speed || speed > axis->pmc_max_speed)
            {
                CreateError(ERR_PMC_SPEED_ERROR,
                            ERROR_LEVEL,
                            CLEAR_BY_MCP_RESET,
                            0,CHANNEL_ENGINE_INDEX,axis->axis_index);
                break;
            }

            speed = speed * 1000/60; // 单位转换：mm/min-->um/s

            PmcCmdFrame pmc_cmd;
            pmc_cmd.data.cmd = 0x00;   //绝对坐标模式
            pmc_cmd.data.axis_index = axis->axis_index+1;
            pmc_cmd.data.axis_index |= 0xFF00;      //标志通道引擎
            pmc_cmd.data.data[0] = 0;
            memcpy(&pmc_cmd.data.data[1], &dis, sizeof(dis));
            memcpy(&pmc_cmd.data.data[5], &speed, sizeof(speed));
            this->m_p_channel_engine->SendPmcAxisCmd(pmc_cmd);


            switch(this->m_n_group_index%4){
            case 0:
                this->m_p_f_reg->EADEN1 = 0;   //分配完成信号
                this->m_p_f_reg->EGENA = 1;    //轴移动信号
                this->m_p_f_reg->EINPA = 0;    //轴到位信号
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
        }else if(cmd == 0x06){    //JOG进给，连续进给JOG
            uint32_t speed = pmcAxiscmd.speed*1000/60;   //速度，单位转换：mm/min-->um/s
            speed = speed * double(feed_rate / 100.);
            int64_t dir = (pmcAxiscmd.distance == 0) ? -1 : 1;
            int64_t dis = dir * 9999 * 1e7;

            PmcCmdFrame pmc_cmd;
            pmc_cmd.data.cmd = 0x0100;   //增量坐标模式
            pmc_cmd.data.axis_index = axis->axis_index+1;
            pmc_cmd.data.axis_index |= 0xFF00;      //标志通道引擎
            pmc_cmd.data.data[0] = 0;
            memcpy(&pmc_cmd.data.data[1], &dis, sizeof(dis));
            memcpy(&pmc_cmd.data.data[5], &speed, sizeof(speed));

            //		printf("pmc axis execute cmd: speed = %u, dis = %lld\n", speed, dis);

            this->m_p_channel_engine->SendPmcAxisCmd(pmc_cmd);

            //JOG指令不缓冲
//            this->m_n_cmd_count--;
//            this->m_n_buf_exec++;
//            if(this->m_n_buf_exec == 3)
//                this->m_n_buf_exec = 0;

            //设置状态
            switch(this->m_n_group_index%4){
            case 0:
                this->m_p_f_reg->EADEN1 = 0;   //分配完成信号
                this->m_p_f_reg->EGENA = 1;    //轴移动信号
                this->m_p_f_reg->EINPA = 0;    //轴到位信号
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
        this->m_p_f_reg->EABUFA = 0;   //缓冲满信号复位
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
