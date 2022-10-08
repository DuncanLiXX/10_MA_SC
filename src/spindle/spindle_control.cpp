#include <future>
#include <functional>
#include "spindle_control.h"
#include "trace.h"
#include "mc_communication.h"
#include "mi_communication.h"
#include "parm_definition.h"

using namespace Spindle;

SpindleControl::SpindleControl()
{
}

void SpindleControl::SetComponent(MICommunication *mi,
                                  MCCommunication *mc,
                                  FRegBits *f_reg)
{
    this->mi = mi;
    this->mc = mc;
    this->F = f_reg;
}

void SpindleControl::SetSpindleParams(SCAxisConfig *spindle,
                                      uint32_t da_prec,
                                      uint8_t phy_axis,
                                      uint8_t z_axis)
{
    this->spindle = spindle;
    this->da_prec = da_prec;
    this->phy_axis = phy_axis;
    this->z_axis = z_axis;
    UpdateParams();
}

void SpindleControl::Reset()
{
    if(!spindle)
        return;
    wait_sar = false;
    wait_off = false;
    if(isTapEnable())
        CancelRigidTap();

    UpdateParams();
}

void SpindleControl::InputSCode(uint32_t s_code)
{
    if(!spindle)
        return;
    F->scode_0 = (s_code&0xFF);
    F->scode_1 = ((s_code>>8)&0xFF);
    F->scode_2 = ((s_code>>16)&0xFF);
    F->scode_3 = ((s_code>>24)&0xFF);

    cnc_speed = s_code;
    if(cnc_speed > spindle->spd_max_speed)
        cnc_speed = spindle->spd_max_speed;
    if(cnc_speed < spindle->spd_min_speed)
        cnc_speed = spindle->spd_min_speed;
    UpdateSpindleState();
}

uint32_t SpindleControl::GetSCode()
{
    return cnc_speed;
}

void SpindleControl::InputPolar(Spindle::CncPolar polar)
{
    if(!spindle)
        return;
    cnc_polar = polar;
    if(polar == Positive || polar == Negative){
        // 收到正/反转信号
        if(!motor_enable) //主轴不在使能状态，需要先使能
        {
            mi->SendAxisEnableCmd(phy_axis+1, true);
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
        wait_sar = true;
        UpdateSpindleState();
    }else{
        // 收到主轴停信号
        if(!motor_enable) //主轴已经不在使能状态
            return;
        wait_off = true;
        mi->SendAxisEnableCmd(phy_axis+1,false);
    }
}

void SpindleControl::SetMode(Mode mode)
{
    if(!spindle)
        return;
    if(mode == Speed)
        mi->SendAxisCtrlModeSwitchCmd(phy_axis+1, 2);
    else if(mode == Position)
        mi->SendAxisCtrlModeSwitchCmd(phy_axis+1, 1);

    // 模式的修改等mi命令回复
}

Mode SpindleControl::GetMode()
{
    return mode;
}

bool SpindleControl::isTapEnable()
{
    return tap_enable;
}

void SpindleControl::StartRigidTap(double feed)
{
    if(!spindle)
        return;
    // todo:如果不在位置模式，报警：速度模式下不能刚性攻丝

    // todo:如果进给为0，报警：攻丝比例异常
    if(feed == 0){
        return;
    }
    // ratio:攻丝比例，10000*S/F，S单位为rpm，F单位为mm/min
    int32_t ratio = -10000.0*cnc_speed*spindle->move_pr/feed;
    if(TSO)
        ratio *= SOV;

    // 发送攻丝轴号
    mi->SendTapAxisCmd(chn, phy_axis+1, z_axis+1);
    // 发送攻丝参数
    mi->SendTapParams(chn,spindle->spd_sync_error_gain,
                      spindle->spd_speed_feed_gain,
                      spindle->spd_pos_ratio_gain);
    // 发送攻丝比例
    mi->SendTapRatioCmd(chn, ratio);
    // 打开攻丝状态
    SendMcRigidTapFlag(true);
    mi->SendTapStateCmd(chn,true);
    tap_enable = true;
}

void SpindleControl::CancelRigidTap()
{
    if(!spindle)
        return;
    SendMcRigidTapFlag(false);
    mi->SendTapStateCmd(chn, false);
    mi->SendTapRatioCmd(chn, 0);

    tap_enable = false;

    // 给信号梯图来取消攻丝
    if(RGTAP)
        F->RGSPP = 0;

    // todo:在一段时间内等待切回速度模式，否则报警：取消刚性攻丝后没有进入速度模式
}

// _SSTP信号输入 低有效
void SpindleControl::InputSSTP(bool _SSTP)
{
    if(!spindle)
        return;
    this->_SSTP = _SSTP;
    // 修改主轴使能状态
    if(!_SSTP && SOR)
        F->ENB = 1;
    else
        F->ENB = _SSTP;
    ///待确认：_SSTP信号是否需要清除速度
    if(!_SSTP) // _SSTP信号需要清除之前S代码设置的速度
        cnc_speed = 0;

    UpdateSpindleState();
}

void SpindleControl::InputSOR(bool SOR)
{
    if(!spindle)
        return;
    this->SOR = SOR;
    // 修改主轴使能状态
    if(!_SSTP && SOR)
        F->ENB = 1;
    UpdateSpindleState();
}

void SpindleControl::InputSOV(uint8_t SOV)
{
    if(!spindle)
        return;
    this->SOV = SOV;
    UpdateSpindleState();
}

void SpindleControl::InputRI(uint16_t RI)
{
    if(!spindle)
        return;
    this->RI = RI;
    if(SIND == 1) // 主轴速度由PMC确定时，才需要更新速度
    {
        UpdateSpindleState();
    }
}

void SpindleControl::InputSGN(bool SGN)
{
    if(!spindle)
        return;
    this->SGN = SGN;
    if(SSIN == 1) // 主轴极性由PMC确定时，才需要更新速度
    {
        UpdateSpindleState();
    }
}

void SpindleControl::InputSSIN(bool SSIN)
{
    if(!spindle)
        return;
    this->SSIN = SSIN;
    UpdateSpindleState();
}

void SpindleControl::InputSIND(bool SIND)
{
    if(!spindle)
        return;
    this->SIND = SIND;
    UpdateSpindleState();
}

void SpindleControl::InputORCMA(bool ORCMA)
{
    if(!spindle)
        return;
    // 收到定位信号，向MI发送主轴定位指令
    this->ORCMA = ORCMA;
    if(ORCMA)
    {
        mi->SendSpdLocateCmd(chn, phy_axis+1);
    }
}

void SpindleControl::InputRGTAP(bool RGTAP)
{
    if(!spindle)
        return;
    this->RGTAP = RGTAP;
    Mode toMode;
    if(RGTAP)
        toMode = Position;
    else
        toMode = Speed;

    // 模式不一样才切换
    if(mode != toMode){
        SetMode(toMode);
    }
}

void SpindleControl::RspORCMA(bool success)
{
    if(!spindle)
        return;
    // 定位成功，将ORAR置为1，通知PMC定位动作完成
    if(success && ORCMA){
        F->ORAR = 1;
    }
}

void SpindleControl::RspCtrlMode(uint8_t axis, Spindle::Mode mode)
{
    if(!spindle)
        return;
    if(axis == phy_axis+1){
        // 刚性攻丝信号为1的状态下，切换到了位置模式，那么准备攻丝
        if(mode == Position && RGTAP){
            F->RGSPP = 1;
        }

        this->mode = mode;
    }
}

void SpindleControl::RspAxisEnable(uint8_t axis, bool enable)
{
    if(!spindle)
        return;
    // 正在等待下使能，并且收到了回复
    if(axis == phy_axis+1){
        motor_enable = enable;
        if(wait_off && !enable){
            wait_off = false;
            F->SST = 1;
        }
    }
}

void SpindleControl::RspSpindleSpeed(uint8_t axis, bool success)
{
    if(!spindle)
        return;
    // 正在等待速度到达，并且收到了回复
    if(axis == phy_axis+1 && wait_sar && success)
    {
        wait_sar = false;
        F->SAR = 1;
    }
}

int32_t SpindleControl::GetSpindleSpeed()
{
    if(!spindle)
        return 0;

    int32_t speed;
    if(!mi->ReadPhyAxisSpeed(&speed, phy_axis)){
        return 0;
    }

    CncPolar polar = CalPolar();
    if(polar == Stop){
        return 0;
    }
    speed = fabs(speed);
    if(polar == Negative){
        speed *= -1;
    }
    speed = speed*60/(spindle->move_pr*1000);
    return speed;
}

void SpindleControl::UpdateParams()
{
    if(!spindle)
        return;
    GST = spindle->spd_ctrl_GST;
    SGB = spindle->spd_ctrl_SGB;
    SFA = spindle->spd_ctrl_SFA;
    ORM = spindle->spd_ctrl_ORM;
    TCW = spindle->spd_ctrl_TCW;
    CWM = spindle->spd_ctrl_CWM;
    TSO = spindle->spd_ctrl_TSO;
}

// 根据当前状态更新转速
void SpindleControl::UpdateSpindleState()
{
    if(!spindle)
        return;
    // 这里要判断是否需要换挡
    // 0：不用换挡，继续往下执行，直接输出转速
    // 1：需要换挡，转速的输出由SendGearLevel内部处理，先返回
    if(UpdateSpindleLevel(cnc_speed))
    {
        return;
    }

    SendSpdSpeedToMi();
}

uint16_t SpindleControl::GetMaxSpeed()
{
    if(!spindle)
        return 0;
    if(level == Low)
        return  spindle->spd_gear_speed_low;
    else if(level == Middle)
        return spindle->spd_gear_speed_middle;
    else
        return spindle->spd_gear_speed_high;
}

// 根据当前状态获取主轴转向
CncPolar SpindleControl::CalPolar()
{
    if(!spindle)
        return Stop;
    int polar = Stop; // 速度极性 -1:未初始化 0:正 1:负 2:停
    if(SSIN == 0) // 极性由cnc来确定
    {
        // 主轴定向功能，极性由参数ORM设定
        if(GST == 0 && SOR == 1)
        {
            return (CncPolar)ORM;
        }
        /*
         * TCW    CWM    极性
         *  0      0     正
         *  0      1     负
         *  1      0     M03为正  M04为负
         *  1      1     M04为负  M03为正
         */
        if(TCW == 0)
        {
            polar = CWM;
        }
        else
        {
            polar = cnc_polar;
            if(CWM == 1 && (polar == Positive || polar == Negative))
                polar = !polar;
        }
    }
    else // 极性由pmc信号SGN来确定
    {
        polar = SGN;
    }
    return (CncPolar)polar;
}

// 根据当前状态获取DA电平值
int32_t SpindleControl::CalDaOutput()
{
    if(!spindle)
        return 0;
    int32_t output; // 转速
    uint16_t max_spd = GetMaxSpeed();

    if(SIND == 0) // 速度由cnc来确定
    {
        int32_t rpm = 0;
        if(SOR == 0)
        {
            rpm = cnc_speed;  // 单位:rpm
        }
        else
        {
            rpm = spindle->spd_sor_speed;
        }
        rpm *= SOV/100.0; // 乘以倍率
        output = da_prec * (1.0 * rpm/max_spd);   // 转化为电平值
    }
    else // 速度由pmc来确定
    {
        output = RI;
    }

    if(output >= da_prec)
    {
        output = da_prec - 1;
    }
    return output;
}

bool SpindleControl::UpdateSpindleLevel(uint16_t speed)
{
    if(!spindle)
        return false;
    // 获取方向
    CncPolar polar = CalPolar();
    if(polar == Stop)
    {
        return false;
    }

    uint8_t GR1O = 0, GR2O = 0, GR3O = 0;
    CalLevel(GR1O, GR2O, GR3O);

    // 档位选择信号需要修改
    if(GR1O != F->GR1O || GR2O != F->GR2O || GR3O != F->GR3O)
    {
        F->GR1O = GR1O;
        F->GR2O = GR2O;
        F->GR3O = GR3O;

        // SFA使能时，需要发送SF信号
        if(SFA)
        {
            //延时TM ms后发送SF信号
            std::this_thread::sleep_for(std::chrono::microseconds(TM));
            F->SF = 1;

            // 记录档位
            if(GR1O){
                level = Low;
            }else if(GR2O){
                level = Middle;
            }else if(GR3O){
                level = High;
            }else{
                printf("Spindle error: unknowed level!");
            }

            //延时TMF ms后发送速度(异步)
            SpindleControl s;
            auto func = std::bind(&SpindleControl::DelaySendSpdSpeedToMi,
                                  this, std::placeholders::_1);
            std::async(std::launch::async, func, TMF);

            //            //等待SF信号关闭(收到FIN信号后，ProcessPmcSignal中会把SF设置为0)
            //            int delay = 0;
            //            while(F->SF == 1 || delay > 5000)
            //            {
            //                std::this_thread::sleep_for(std::chrono::microseconds(10));
            //                delay += 10;
            //                if(F->SF == 0)  // PMC回应了SF信号，换挡成功
            //                {
            //                    SwLevelSuccess();
            //                }
            //            }
            //            if(F->SF == 1) // PMC处理超时
            //            {
            //                printf("FIN response for SF timeout!\n");
            //            }

            return true;
        }
    }
    return false;
}

void SpindleControl::CalLevel(uint8_t &GR1O, uint8_t &GR2O, uint8_t &GR3O)
{
    if(!spindle)
        return;
    GR1O = 0;
    GR2O = 0;
    GR3O = 0;
    if(SGB == 0) // A换挡方式
    {
        //档位1：0~档位1最大转速
        //档位2：档位1最大转速~档位2最大转速
        //档位3：档位2最大转速~档位3最大转速
        if(cnc_speed <= spindle->spd_gear_speed_low) //档位1
        {
            GR1O = 1;
        }
        else if(cnc_speed <= spindle->spd_gear_speed_middle) // 档位2
        {
            GR2O = 1;
        }
        else    // 档位3
        {
            GR3O = 1;
        }

    }
    else if(SGB == 1) // B换挡方式
    {
        //档位1：0~中低档切换速度
        //档位2：中低档切换速度~中高档切换速度
        //档位3：中高档切换速度~主轴最高转速
        if(cnc_speed <= spindle->spd_gear_switch_speed1) // 档位1
        {
            GR1O = 1;
        }
        else if(cnc_speed <= spindle->spd_gear_switch_speed2) // 档位2
        {
            GR2O = 1;
        }
        else    // 档位3
        {
            GR3O = 1;
        }
    }
}

void SpindleControl::SendSpdSpeedToMi()
{
    if(!spindle)
        return;
    // 攻丝状态下禁止修改转速
    if(RGTAP)
        return;

    // 获取方向
    CncPolar polar = CalPolar();
    if(polar == Stop)
    {
        return;
    }

    // 获取速度
    int32_t output = CalDaOutput();

    // 速度输出到PMC（不考虑方向）
    F->RO = output;

    output *= spindle->spd_analog_gain/1000.0;     // 乘以增益

    if(polar == Negative)
    {
        output *= -1;
    }

    output += spindle->zero_compensation; // 加上零漂

    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_SPD_SPEED;
    cmd.data.axis_index = phy_axis;
    cmd.data.data[0] = (int16_t)output;
    mi->WriteCmd(cmd);
}

void SpindleControl::DelaySendSpdSpeedToMi(uint16_t ms)
{
    if(!spindle)
        return;
    std::this_thread::sleep_for(std::chrono::microseconds(ms));
    SendSpdSpeedToMi();
}

void SpindleControl::SendMcRigidTapFlag(bool enable)
{
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = chn;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_SET_G84_INTP_MODE;
    cmd.data.data[0] = enable?1:0;
    mc->WriteCmd(cmd);
}
