#include <future>
#include <functional>
#include "spindle_control.h"
#include "trace.h"
#include "mc_communication.h"
#include "mi_communication.h"
#include "parm_definition.h"
#include "global_include.h"
#include "channel_control.h"
#include "channel_engine.h"

using namespace Spindle;

SpindleControl::SpindleControl()
{
}

void SpindleControl::SetComponent(MICommunication *mi,
                                  MCCommunication *mc,
                                  FRegBits *f_reg,
                                  const GRegBits *g_reg)
{
    this->mi = mi;
    this->mc = mc;
    this->F = f_reg;
    this->G = g_reg;
    InputSSTP(G->_SSTP);
    InputSOR(G->SOR);
    InputSOV(G->SOV);
    InputRI(G->RI);
    InputSGN(G->SGN);
    InputSSIN(G->SSIN);
    InputSIND(G->SIND);
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

    UpdateParams();
}

void SpindleControl::InputSCode(uint32_t s_code)
{
    if(!spindle)
        return;
    printf("SpindleControl::InputSCode s_code = %d\n");
    F->scode_0 = (s_code&0xFF);
    F->scode_1 = ((s_code>>8)&0xFF);
    F->scode_2 = ((s_code>>16)&0xFF);
    F->scode_3 = ((s_code>>24)&0xFF);

    cnc_speed = s_code;
    if(cnc_speed > spindle->spd_max_speed)
        cnc_speed = spindle->spd_max_speed;
    if(cnc_speed < spindle->spd_min_speed)
        cnc_speed = spindle->spd_min_speed;
    if(CalPolar() == Stop)
        return;
    UpdateSpindleState();
}

uint32_t SpindleControl::GetSCode()
{
    return cnc_speed*SOV/100.0;
}

void SpindleControl::InputPolar(Spindle::CncPolar polar)
{
    if(!spindle)
        return;
    printf("SpindleControl::InputPolar %d\n",polar);
    if(polar == Positive || polar == Negative){
        // 收到正/反转信号
        wait_on = true;
        mi->SendAxisEnableCmd(phy_axis+1, true);
        printf("SendAxisEnable enable = %d\n",true);
    }else{
        // 收到主轴停信号
        wait_off = true;
        mi->SendAxisEnableCmd(phy_axis+1,false);
        printf("SendAxisEnable enable = %d\n",false);
    }
    cnc_polar = polar;
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

bool SpindleControl::IsValid()
{
    if(!spindle || spindle->axis_interface == 0){
        return false;
    }else{
        return true;
    }
}

bool SpindleControl::isTapEnable()
{
    return tap_enable;
}

void SpindleControl::SetTapFeed(double feed)
{
    tap_feed = feed;
}

void SpindleControl::StartRigidTap(double feed)
{
    if(!spindle || spindle->axis_interface == 0)
        return;
    // 如果不在位置模式，报警：速度模式下不能刚性攻丝
    if(mode != Position){
        CreateError(ERR_SPD_TAP_START_FAIL,
                    ERROR_LEVEL,
                    CLEAR_BY_MCP_RESET);
        return;
    }

    // 如果进给为0，报警：攻丝比例异常
    if(feed == 0){
        CreateError(ERR_SPD_TAP_RATIO_FAULT,
                    ERROR_LEVEL,
                    CLEAR_BY_MCP_RESET);
        return;
    }
    // ratio:攻丝比例，10000*S/F，S单位为rpm，F单位为mm/min
    int32_t ratio = -10000.0*cnc_speed*spindle->move_pr/feed;
    if(TSO)
        ratio *= SOV/100.0;

    // 发送攻丝轴号
    mi->SendTapAxisCmd(chn, phy_axis+1, z_axis+1);
    printf("SendTapAxisCmd spd=%d,z=%d\n",phy_axis+1,z_axis+1);
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
    std::this_thread::sleep_for(std::chrono::microseconds(100*1000));
    F->RGSPP = 1;
}

void SpindleControl::CancelRigidTap()
{
    if(!spindle || spindle->axis_interface == 0)
        return;
    SendMcRigidTapFlag(false);
    mi->SendTapStateCmd(chn, false);
    mi->SendTapRatioCmd(chn, 0);

    tap_enable = false;


    // 给信号梯图来取消攻丝
    if(F->RGSPP)
        F->RGSPP = 0;
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
    printf("InputSOR:SOR = %d\n");
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
        // 定位时主轴转速为0，报警
        if(abs(GetSpindleSpeed()) == 0){
            CreateError(ERR_SPD_LOCATE_SPEED,
                        ERROR_LEVEL,
                        CLEAR_BY_MCP_RESET);
            return;
        }
        mi->SendSpdLocateCmd(chn, phy_axis+1,true);
        printf("SendSpdLocateCmd enable = %d\n",true);
    }else{
        mi->SendSpdLocateCmd(chn, phy_axis+1,false);
        printf("SendSpdLocateCmd enable = %d\n",false);
    }
}

void SpindleControl::InputRGTAP(bool RGTAP)
{
    if(!spindle)
        return;
    this->RGTAP = RGTAP;
    if(RGTAP)
        StartRigidTap(tap_feed);
    else
        CancelRigidTap();
}

void SpindleControl::InputRGMD(bool RGMD)
{
    if(!spindle)
        return;
    this->RGMD = RGMD;

    if(RGMD)
        SetMode(Position);
    else
        SetMode(Speed);
}

void SpindleControl::RspORCMA(bool success)
{
    if(!spindle)
        return;
    printf("SpindleControl::RspORCMA : success = %d\n",success);
    // 定位成功，将ORAR置为1，通知PMC定位动作完成
    if(success && ORCMA){
        F->ORAR = 1;
    }
}

void SpindleControl::RspCtrlMode(uint8_t axis, Spindle::Mode mode)
{
    if(!spindle)
        return;
    printf("RspCtrlMode:axis = %d,mode = %d\n",axis,mode);
    if(axis == phy_axis + 1){
        auto func = std::bind(&SpindleControl::ProcessModeChanged,
                              this, std::placeholders::_1);
        std::async(std::launch::async, func, mode);
    }
}

void SpindleControl::RspAxisEnable(uint8_t axis, bool enable)
{
    if(!spindle)
        return;
    printf("RspAxisEnable:axis = %d,enable = %d\n",axis,enable);

    if(axis == phy_axis+1){
        // 正在等待下使能，并且收到了回复
        motor_enable = enable;
        if(wait_off && !enable){
            wait_off = false;
            F->SST = 1;
        }
        // 正在等待上使能，上使能后根据当前状态发生转速
        if(wait_on){
            wait_on = false;
            wait_sar = true;
            UpdateSpindleState();
        }
    }
}

void SpindleControl::RspSpindleSpeed(uint8_t axis, bool success)
{
    if(!spindle)
        return;
    printf("SpindleControl::RspSpindleSpeed: axis=%d,success=%d\n",axis,success);
    if(axis == phy_axis + 1){
        // 正在等待速度到达，并且收到了回复
        if(wait_sar && success)
        {
            wait_sar = false;
            F->SAR = 1;
        }
    }
}

int32_t SpindleControl::GetSpindleSpeed()
{
    if(!spindle)
        return 0;

    if(spindle->axis_interface == 0){
        int speed = cnc_speed * SOV/100.0;
        return speed;
    }

    int32_t speed;
    if(!mi->ReadPhyAxisSpeed(&speed, phy_axis)){
        return 0;
    }

    //    CncPolar polar = CalPolar();
    //    if(polar == Stop){
    //        return 0;
    //    }
    //    speed = fabs(speed);
    //    if(polar == Negative){
    //        speed *= -1;
    //    }
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
    Level level;

    if(SFA) // 如果开启了换挡功能，最大转速根据
        level = to_level;
    else
        level = this->level;
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
        if(SOR == 1)
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
        else if(TCW == 0)
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
    printf("SIND=%d SOR=%d cnc_speed=%d SOV=%d max_spd=%d da_prec=%d\n",SIND,SOR,cnc_speed,SOV,max_spd,da_prec);

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

    printf("GRO: %d %d %d %d %d %d\n",F->GR1O,F->GR2O,F->GR3O,
           GR1O,GR2O,GR3O);

    // 档位选择信号需要修改
    if(GR1O != F->GR1O || GR2O != F->GR2O || GR3O != F->GR3O)
    {
        F->GR1O = GR1O;
        F->GR2O = GR2O;
        F->GR3O = GR3O;

        // SFA使能时，才执行换挡动作
        if(SFA && SIND == 0)
        {
            //异步处理换挡逻辑
            auto func = std::bind(&SpindleControl::ProcessSwitchLevel,
                                  this);
            std::async(std::launch::async, func, TMF);

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

    if(SSIN)    // 转速由PMC来控制，保持当前档位
    {
        if(level == Low)
            GR1O = 1;
        else if(level == Middle)
            GR2O = 1;
        else
            GR3O = 1;
    }
    else if(SGB == 0) // A换挡方式
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
    CncPolar polar;
    int32_t output;
    // 攻丝状态下禁止修改转速
    if(RGTAP){
        CreateError(ERR_SPD_RUN_IN_TAP,
                    ERROR_LEVEL,
                    CLEAR_BY_MCP_RESET);
        return;
    }

    // 获取方向
    polar = CalPolar();
    if(polar == Stop)
    {
        output = 0;
    }
    else
    {
        // 获取速度
        output = CalDaOutput();
        // 速度输出到PMC（不考虑方向）
        F->RO = output;
    }

    output *= spindle->spd_analog_gain/1000.0;     // 乘以增益

    if(polar == Negative)
    {
        output *= -1;
    }

    output += spindle->zero_compensation; // 加上零漂

    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_SPD_SPEED;
    cmd.data.axis_index = phy_axis+1;
    cmd.data.data[0] = (int16_t)output;
    printf("send spindle DA output: %d\n",output);
    mi->WriteCmd(cmd);
}

void SpindleControl::ProcessModeChanged(Spindle::Mode mode)
{
    if(mode == Position){
        ChannelEngine *engine = ChannelEngine::GetInstance();
        ChannelControl *control = engine->GetChnControl(0);
        ChannelRealtimeStatus status = control->GetRealtimeStatus();
        double pos = status.cur_pos_machine.GetAxisValue(phy_axis);
        mi->SetAxisRefCur(phy_axis+1,pos);
        printf("SetAxisRefCur: axis=%d, pos = %lf\n",phy_axis+1,pos);
        printf("RspCtrlMode:Position\n");
        std::this_thread::sleep_for(std::chrono::microseconds(100*1000));
        F->RGMP = 1;
    }else{
        F->RGMP = 0;
        printf("RspCtrlMode:Speed\n");
    }

    this->mode = mode;
}

void SpindleControl::ProcessSwitchLevel()
{
    if(!spindle)
        return;

    //延时TM us后发送SF信号
    std::this_thread::sleep_for(std::chrono::microseconds(TM));
    F->SF = 1;
    printf("set SF 1\n");

    // 记录档位
    if(F->GR1O){
        to_level = Low;
    }else if(F->GR2O){
        to_level = Middle;
    }else if(F->GR3O){
        to_level = High;
    }else{
        printf("Spindle error: unknowed level!");
    }

    // 延时TMF后发送速度
    std::this_thread::sleep_for(std::chrono::microseconds(TMF));
    SendSpdSpeedToMi();

    //等待SF信号关闭(收到FIN信号后，ProcessPmcSignal中会把SF设置为0)
    int delay = 0;
    while(F->SF == 1 && delay < 5000)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
        delay += 10;
        if(F->SF == 0)  // PMC回应了SF信号，换挡成功
        {
            // 目标档位赋值给当前档位
            level = to_level;
            printf("change level: %d\n",level);
            break;
        }
    }
    if(F->SF == 1) // PMC处理超时
    {
        // 清除掉目标档位
        printf("change level fail\n");
        to_level = level;
        CreateError(ERR_SPD_SW_LEVEL_FAIL,
                    ERROR_LEVEL,
                    CLEAR_BY_MCP_RESET);
    }
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
