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
    this->m_n_buf_exec = 0;
    this->m_b_active = false;
    this->m_n_cmd_count = 0;
    this->m_p_f_reg = nullptr;
    this->m_p_g_reg = nullptr;
    this->m_b_buffer = true;  //默认缓冲有效
    m_b_step_stop = false;   //默认无效
    m_b_pause = false;      //默认不在暂停状态

    this->m_p_channel_engine = nullptr;
    this->m_p_mi_comm = nullptr;
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

    //printf("PmcAxisCtrl::Active[%hhu]:%hhu\n", this->m_n_group_index, flag);

    //检查_EAXSL信号
    FRegBits *freg0 = m_p_channel_engine->GetChnFRegBits(0);
    if(freg0->_EAXSL == 1)
        return false;

    if (!CanActive())
        return false;

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
            //刷新Phaser
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
            //cmd.data.data[1] = flag ? 1 : 0;  // 0--初始配置  1--动态切换
            cmd.data.data[1] = 1;
            this->m_p_mi_comm->WriteCmd(cmd);


        }
    }

    if(flag){
        // 状态
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
    }
    return true;
}

bool PmcAxisCtrl::CanActive()
{
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
 * @brief 获得当前接收缓冲的索引号
 * @return 返回当前可用的接收缓冲的索引号，从0开始，0xFF表示无可用缓冲
 */
uint8_t PmcAxisCtrl::GetRecvBufIndex(){
    uint8_t index = 0xFF;
    if(this->m_n_cmd_count < 3){
        index = this->m_n_buf_exec + m_n_cmd_count;
        if(index >= 3)
            index -= 3;
    }
    return index;
}

/**
 * @brief 设置缓冲状态
 * @param flag : true--缓冲有效   false--缓冲无效
 */
void PmcAxisCtrl::SetBuffState(bool flag){
    if(flag == m_b_buffer)
        return;
    if(this->m_n_cmd_count > 1){//TODO 告警，缓冲区关闭或打开时，必须保证缓冲区为空
        return;
    }
    //printf("PmcAxisCtrl::SetBuffState[%hhu]:%hhu\n", m_n_group_index, flag);

    this->m_b_buffer = flag;
    if(flag && m_n_cmd_count==1){//当前有数据，缓冲切换至有效，则翻转EBSYg信号
        switch(this->m_n_group_index%4){
        case 0:
            this->m_p_f_reg->EBSYA = m_p_f_reg->EBSYA?0:1;
            break;
        case 1:
            this->m_p_f_reg->EBSYB = m_p_f_reg->EBSYB?0:1;
            break;
        case 2:
            this->m_p_f_reg->EBSYC = m_p_f_reg->EBSYC?0:1;
            break;
        case 3:
            this->m_p_f_reg->EBSYD = m_p_f_reg->EBSYD?0:1;
            break;
        }
    }
}

/**
 * @brief 设置程序段停止
 * @param flag : true--程序段停止有效     false--程序段停止无效
 */
void PmcAxisCtrl::SetStepStop(bool flag){
    if(flag == this->m_b_step_stop)
        return;

    this->m_b_step_stop = flag;
    if(!flag && this->m_n_cmd_count > 0){  //执行当前缓存指令
        this->ExecuteCmd();
    }
}

/**
 * @brief 执行复位动作
 */
void PmcAxisCtrl::Reset(){
    for(unsigned int i = 0; i < axis_list.size(); i++){
        this->m_p_channel_engine->ManualMoveStop(axis_list.at(i)->axis_index);
    }
    if(this->m_n_cmd_count > 0){  //停止当前指令执行，清空缓冲
        this->m_n_cmd_count = 0;
        this->m_b_pause = false;

        switch(this->m_n_group_index%4){
        case 0:
            printf("EBSYA = %hhu, EBUFA = %hhu\n", this->m_p_f_reg->EBSYA, this->m_p_g_reg->EBUFA);
            this->m_p_f_reg->EACNT1 = 0;
            if(!m_b_buffer)//检查EMBUFg缓存无效信号
            {
                this->m_p_f_reg->EBSYA = m_p_f_reg->EBSYA?0:1;
            }

            printf("PmcAxisCtrl::Reset over, EBSYA = %hhu, EBUFA = %hhu\n", this->m_p_f_reg->EBSYA, this->m_p_g_reg->EBUFA);
            break;
        case 1:
            printf("EBSYB = %hhu, EBUFB = %hhu\n", this->m_p_f_reg->EBSYB, this->m_p_g_reg->EBUFB);
            this->m_p_f_reg->EACNT2 = 0;
            if(!m_b_buffer)//检查EMBUFg缓存无效信号
                this->m_p_f_reg->EBSYB = m_p_f_reg->EBSYB?0:1;

            printf("PmcAxisCtrl::Reset over, EBSYB = %hhu, EBUFB = %hhu\n", this->m_p_f_reg->EBSYB, this->m_p_g_reg->EBUFB);
            break;
        case 2:
            printf("EBSYC = %hhu, EBUFC = %hhu\n", this->m_p_f_reg->EBSYC, this->m_p_g_reg->EBUFC);
            this->m_p_f_reg->EACNT3 = 0;
            if(!m_b_buffer)//检查EMBUFg缓存无效信号
                this->m_p_f_reg->EBSYC = m_p_f_reg->EBSYC?0:1;

            printf("PmcAxisCtrl::Reset over, EBSYC = %hhu, EBUFC = %hhu\n", this->m_p_f_reg->EBSYC, this->m_p_g_reg->EBUFC);
            break;
        case 3:
            printf("EBSYD = %hhu, EBUFD = %hhu\n", this->m_p_f_reg->EBSYD, this->m_p_g_reg->EBUFD);
            this->m_p_f_reg->EACNT4 = 0;
            if(!m_b_buffer)//检查EMBUFg缓存无效信号
                this->m_p_f_reg->EBSYD = m_p_f_reg->EBSYD?0:1;

            printf("PmcAxisCtrl::Reset over, EBSYD = %hhu, EBUFD = %hhu\n", this->m_p_f_reg->EBSYD, this->m_p_g_reg->EBUFD);
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
}

/**
 * @brief 轴暂停
 * @flag : true--激活暂停      false--取消暂停
 */
void PmcAxisCtrl::Pause(bool flag){
    //轴暂停
    if(this->m_n_cmd_count == 0 || this->m_b_pause == flag)
        return;

    this->m_b_pause = flag;

    //	printf("PmcAxisCtrl::Pause: %hhu\n", flag);
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
 * @brief 写入PMC指令
 * @param cmd
 * @return   true--成功    false--失败
 */
bool PmcAxisCtrl::WriteCmd(PmcAxisCtrlCmd &cmd){
    //写入PMC指令
    if(!this->m_b_buffer && this->m_n_cmd_count > 0)  //缓冲无效
    {
        return false;
    }
    if(this->m_n_cmd_count == 3 || !this->IsActive())  //缓冲已满,或者未激活
    {
        return false;
    }

    //	switch(this->m_n_group_index%4){
    //	case 0:
    //		printf("PmcAxisCtrl::WriteCmd:axis = %hhu, cmd=%hhu, spd=%hu, dis=%d, [%hhu, %hhu]\n", this->m_n_phy_axis, cmd.cmd, cmd.speed, cmd.distance,
    //					this->m_p_f_reg->EBSYA, this->m_p_g_reg->EBUFA);
    //		break;
    //	case 1:
    //		printf("PmcAxisCtrl::WriteCmd:axis = %hhu, cmd=%hhu, spd=%hu, dis=%d, [%hhu, %hhu]\n", this->m_n_phy_axis, cmd.cmd, cmd.speed, cmd.distance,
    //					this->m_p_f_reg->EBSYB, this->m_p_g_reg->EBUFB);
    //		break;
    //	case 2:
    //		printf("PmcAxisCtrl::WriteCmd:axis = %hhu, cmd=%hhu, spd=%hu, dis=%d, [%hhu, %hhu]\n", this->m_n_phy_axis, cmd.cmd, cmd.speed, cmd.distance,
    //					this->m_p_f_reg->EBSYC, this->m_p_g_reg->EBUFC);
    //		break;
    //	case 3:
    //		printf("PmcAxisCtrl::WriteCmd:axis = %hhu, cmd=%hhu, spd=%hu, dis=%d, [%hhu, %hhu]\n", this->m_n_phy_axis, cmd.cmd, cmd.speed, cmd.distance,
    //					this->m_p_f_reg->EBSYD, this->m_p_g_reg->EBUFD);
    //		break;
    //	}

    uint8_t index = this->GetRecvBufIndex();
    this->m_pmc_cmd_buffer[index] = cmd;

    this->m_n_cmd_count++;  //命令计数加一

    if(this->m_n_cmd_count == 1 && !this->m_b_step_stop){  //将当前命令发送至MI执行
        this->ExecuteCmd();
    }else if(this->m_n_cmd_count == 3){//缓冲满
        switch(this->m_n_group_index%4){
        case 0:
            this->m_p_f_reg->EABUFA = 1;
            break;
        case 1:
            this->m_p_f_reg->EABUFB = 1;
            break;
        case 2:
            this->m_p_f_reg->EABUFC = 1;
            break;
        case 3:
            this->m_p_f_reg->EABUFD = 1;
            break;
        }
    }

    //翻转EBSYg信号,置位EACNTg信号
    FRegBits *chn0_freg = m_p_channel_engine->GetChnFRegBits(0);
    switch(this->m_n_group_index%4){
    case 0:
        if(this->m_b_buffer)//检查EMBUFg缓存无效信号
            this->m_p_f_reg->EBSYA = m_p_f_reg->EBSYA?0:1;
        this->m_p_f_reg->EACNT1 = 1;
        chn0_freg->_EAXSL = 1;
        break;
    case 1:
        if(m_p_g_reg->EMBUFB == 0)//检查EMBUFg缓存无效信号
            this->m_p_f_reg->EBSYB = m_p_f_reg->EBSYB?0:1;
        this->m_p_f_reg->EACNT2 = 1;
        chn0_freg->_EAXSL = 1;
        break;
    case 2:
        if(m_p_g_reg->EMBUFC == 0)//检查EMBUFg缓存无效信号
            this->m_p_f_reg->EBSYC = m_p_f_reg->EBSYC?0:1;
        this->m_p_f_reg->EACNT3 = 1;
        chn0_freg->_EAXSL = 1;
        break;
    case 3:
        if(m_p_g_reg->EMBUFD == 0)//检查EMBUFg缓存无效信号
            this->m_p_f_reg->EBSYD = m_p_f_reg->EBSYD?0:1;
        this->m_p_f_reg->EACNT4 = 1;
        chn0_freg->_EAXSL = 1;
        break;
    }

    return true;
}

/**
 * @brief 指令执行完毕
 * @param res : 执行结果  true--成功   false--失败
 */
void PmcAxisCtrl::ExecCmdOver(bool res){
    if(this->m_n_cmd_count == 0)
        return;
    this->m_n_cmd_count--;
    printf("PmcAxisCtrl::ExecCmdOver(), cmd_count = %hhu\n", this->m_n_cmd_count);
    if(this->m_n_cmd_count > 0){ //继续执行下一条指令
        this->m_n_buf_exec++;
        if(this->m_n_buf_exec == 3)
            this->m_n_buf_exec = 0;

        if(!this->m_b_step_stop)
            this->ExecuteCmd();
    }else{//缓冲无数据
        switch(this->m_n_group_index%4){
        case 0:
            this->m_p_f_reg->EACNT1 = 0;
            if(!m_b_buffer)//检查EMBUFg缓存无效信号
                this->m_p_f_reg->EBSYA = m_p_f_reg->EBSYA?0:1;
            break;
        case 1:
            this->m_p_f_reg->EACNT2 = 0;
            if(!m_b_buffer)//检查EMBUFg缓存无效信号
                this->m_p_f_reg->EBSYB = m_p_f_reg->EBSYB?0:1;
            break;
        case 2:
            this->m_p_f_reg->EACNT3 = 0;
            if(!m_b_buffer)//检查EMBUFg缓存无效信号
                this->m_p_f_reg->EBSYC = m_p_f_reg->EBSYC?0:1;
            break;
        case 3:
            this->m_p_f_reg->EACNT4 = 0;
            if(!m_b_buffer)//检查EMBUFg缓存无效信号
                this->m_p_f_reg->EBSYD = m_p_f_reg->EBSYD?0:1;
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

    switch(this->m_n_group_index%4){
    case 0:
        this->m_p_f_reg->EABUFA = 0;
        this->m_p_f_reg->EINPA = 1;
        this->m_p_f_reg->EIALA = res?0:1;
        break;
    case 1:
        this->m_p_f_reg->EABUFB = 0;
        this->m_p_f_reg->EINPB = 1;
        this->m_p_f_reg->EIALB = res?0:1;
        break;
    case 2:
        this->m_p_f_reg->EABUFC = 0;
        this->m_p_f_reg->EINPC = 1;
        this->m_p_f_reg->EIALC = res?0:1;
        break;
    case 3:
        this->m_p_f_reg->EABUFD = 0;
        this->m_p_f_reg->EINPD = 1;
        this->m_p_f_reg->EIALD = res?0:1;
        break;
    }

}

void PmcAxisCtrl::SetErrState(int id)
{
    switch(this->m_n_group_index%4){
    case 0:
        this->m_p_f_reg->EBSYA = this->m_p_f_reg->EBSYA?0:1;
        break;
    case 1:
        this->m_p_f_reg->EBSYA = this->m_p_f_reg->EBSYA?0:1;
        break;
    case 2:
        this->m_p_f_reg->EBSYA = this->m_p_f_reg->EBSYA?0:1;
        break;
    case 3:
        this->m_p_f_reg->EBSYA = this->m_p_f_reg->EBSYA?0:1;
        break;
    }
    CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, id, CHANNEL_ENGINE_INDEX);
}

/**
 * @brief 执行当前执行缓冲中的指令
 */
void PmcAxisCtrl::ExecuteCmd(){
    if(this->m_n_cmd_count == 0)
        return;
    for(unsigned int i = 0; i < axis_list.size(); i++){
        SCAxisConfig *axis = axis_list.at(i);
        uint8_t cmd = m_pmc_cmd_buffer[this->m_n_buf_exec].cmd;   //指令

        std::cout << "PmcAxisCtrl::ExecuteCmd " << (int)cmd << std::endl;
        if (!g_ptr_chn_engine->GetAxisRetRefFlag(axis->axis_index) && cmd != 0x05)
        {//判断轴是否建立参考点
            CreateError(ERR_AXIS_REF_NONE,
                ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, axis->axis_index);
            break;
        }

        if(cmd == 0x00 || cmd == 0x01 || cmd == 0x10 || cmd == 0x11){  //快速定位、切削进给
            uint32_t speed = m_pmc_cmd_buffer[this->m_n_buf_exec].speed;        //速度，单位转换
            int64_t dis = m_pmc_cmd_buffer[this->m_n_buf_exec].distance*1e4;       //移动距离，单位转换：um-->0.1nm
            if(cmd == 0x00 && !axis->pmc_g00_by_EIFg){ // 使用定位速度作为快移速度
                speed = axis->rapid_speed;
            }
            if(cmd == 0x00 || cmd == 0x01){
                if(speed < axis->pmc_min_speed || speed > axis->pmc_max_speed){
                    CreateError(ERR_PMC_SPEED_ERROR,
                                ERROR_LEVEL,
                                CLEAR_BY_MCP_RESET,
                                0,CHANNEL_ENGINE_INDEX,axis->axis_index);
                    break;
                }
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
            uint32_t ms = m_pmc_cmd_buffer[this->m_n_buf_exec].distance;
            std::thread wait(&PmcAxisCtrl::Process04Cmd, this, ms);
            wait.detach();
            break; // 暂停指令只需执行一次即可
        }else if(cmd == 0x05){   //回参考点动作
            if (!g_ptr_chn_engine->SetPmcRetRef(axis->axis_index))
            {
                uint32_t speed = axis->rapid_speed;        //速度，单位转换
                double cur_pos = m_p_channel_engine->GetPhyAxisMachPosFeedback(axis->axis_index);
                int64_t dis = (axis->axis_home_pos[0] - cur_pos)*1e7;       //移动距离，单位转换：mm-->0.1nm

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
                pmc_cmd.data.cmd = 0x0100;   //增量坐标模式
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
            uint32_t speed = axis->rapid_speed;        //速度，单位转换
            double cur_pos = m_p_channel_engine->GetPhyAxisMachPosFeedback(axis->axis_index);
            int64_t dis = (axis->axis_home_pos[0] - cur_pos)*1e7;       //移动距离，单位转换：mm-->0.1nm

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
            pmc_cmd.data.cmd = 0x0100;   //增量坐标模式
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
            uint32_t speed = m_pmc_cmd_buffer[this->m_n_buf_exec].speed*1000/60;   //速度，单位转换：mm/min-->um/s
            int64_t dir = (m_pmc_cmd_buffer[this->m_n_buf_exec].distance == 0) ? -1 : 1;
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
            this->m_n_cmd_count--;
            this->m_n_buf_exec++;
            if(this->m_n_buf_exec == 3)
                this->m_n_buf_exec = 0;

            //设置状态
            switch(this->m_n_group_index%4){
            case 0:
                this->m_p_f_reg->EADEN1 = 0;   //分配完成信号
                this->m_p_f_reg->EGENA = 1;    //轴移动信号
                this->m_p_f_reg->EINPA = 0;    //轴到位信号
                this->m_p_f_reg->EABUFA = 0;   //缓冲满信号复位
                if(!m_b_buffer)//检查EMBUFg缓存无效信号
                    this->m_p_f_reg->EBSYA = m_p_f_reg->EBSYA?0:1;
                break;
            case 1:
                this->m_p_f_reg->EADEN2 = 0;
                this->m_p_f_reg->EGENB = 1;
                this->m_p_f_reg->EINPB = 0;
                this->m_p_f_reg->EABUFB = 0;
                if(!m_b_buffer)//检查EMBUFg缓存无效信号
                    this->m_p_f_reg->EBSYB = m_p_f_reg->EBSYB?0:1;
                break;
            case 2:
                this->m_p_f_reg->EADEN3 = 0;
                this->m_p_f_reg->EGENC = 1;
                this->m_p_f_reg->EINPC = 0;
                this->m_p_f_reg->EABUFC = 0;
                if(!m_b_buffer)//检查EMBUFg缓存无效信号
                    this->m_p_f_reg->EBSYC = m_p_f_reg->EBSYC?0:1;
                break;
            case 3:
                this->m_p_f_reg->EADEN4 = 0;
                this->m_p_f_reg->EGEND = 1;
                this->m_p_f_reg->EINPD = 0;
                this->m_p_f_reg->EABUFD = 0;
                if(!m_b_buffer)//检查EMBUFg缓存无效信号
                    this->m_p_f_reg->EBSYD = m_p_f_reg->EBSYD?0:1;
                break;
            }
        }
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
