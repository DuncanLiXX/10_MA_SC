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
        // �յ���/��ת�ź�
        wait_on = true;
        mi->SendAxisEnableCmd(phy_axis+1, true);
        printf("SendAxisEnable enable = %d\n",true);
    }else{
        // �յ�����ͣ�ź�
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

    // ģʽ���޸ĵ�mi����ظ�
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
    // �������λ��ģʽ���������ٶ�ģʽ�²��ܸ��Թ�˿
    if(mode != Position){
        CreateError(ERR_SPD_TAP_START_FAIL,
                    ERROR_LEVEL,
                    CLEAR_BY_MCP_RESET);
        return;
    }

    // �������Ϊ0����������˿�����쳣
    if(feed == 0){
        CreateError(ERR_SPD_TAP_RATIO_FAULT,
                    ERROR_LEVEL,
                    CLEAR_BY_MCP_RESET);
        return;
    }
    // ratio:��˿������10000*S/F��S��λΪrpm��F��λΪmm/min
    int32_t ratio = -10000.0*cnc_speed*spindle->move_pr/feed;
    if(TSO)
        ratio *= SOV/100.0;

    // ���͹�˿���
    mi->SendTapAxisCmd(chn, phy_axis+1, z_axis+1);
    printf("SendTapAxisCmd spd=%d,z=%d\n",phy_axis+1,z_axis+1);
    // ���͹�˿����
    mi->SendTapParams(chn,spindle->spd_sync_error_gain,
                      spindle->spd_speed_feed_gain,
                      spindle->spd_pos_ratio_gain);
    // ���͹�˿����
    mi->SendTapRatioCmd(chn, ratio);
    // �򿪹�˿״̬
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


    // ���ź���ͼ��ȡ����˿
    if(F->RGSPP)
        F->RGSPP = 0;
}

// _SSTP�ź����� ����Ч
void SpindleControl::InputSSTP(bool _SSTP)
{
    if(!spindle)
        return;
    this->_SSTP = _SSTP;
    // �޸�����ʹ��״̬
    if(!_SSTP && SOR)
        F->ENB = 1;
    else
        F->ENB = _SSTP;
    ///��ȷ�ϣ�_SSTP�ź��Ƿ���Ҫ����ٶ�
    if(!_SSTP) // _SSTP�ź���Ҫ���֮ǰS�������õ��ٶ�
        cnc_speed = 0;

    UpdateSpindleState();
}

void SpindleControl::InputSOR(bool SOR)
{
    if(!spindle)
        return;
    printf("InputSOR:SOR = %d\n");
    this->SOR = SOR;
    // �޸�����ʹ��״̬
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
    if(SIND == 1) // �����ٶ���PMCȷ��ʱ������Ҫ�����ٶ�
    {
        UpdateSpindleState();
    }
}

void SpindleControl::InputSGN(bool SGN)
{
    if(!spindle)
        return;
    this->SGN = SGN;
    if(SSIN == 1) // ���Ἣ����PMCȷ��ʱ������Ҫ�����ٶ�
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

    // �յ���λ�źţ���MI�������ᶨλָ��
    this->ORCMA = ORCMA;
    if(ORCMA)
    {
        // ��λʱ����ת��Ϊ0������
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
    // ��λ�ɹ�����ORAR��Ϊ1��֪ͨPMC��λ�������
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
        // ���ڵȴ���ʹ�ܣ������յ��˻ظ�
        motor_enable = enable;
        if(wait_off && !enable){
            wait_off = false;
            F->SST = 1;
        }
        // ���ڵȴ���ʹ�ܣ���ʹ�ܺ���ݵ�ǰ״̬����ת��
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
        // ���ڵȴ��ٶȵ�������յ��˻ظ�
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

// ���ݵ�ǰ״̬����ת��
void SpindleControl::UpdateSpindleState()
{
    if(!spindle)
        return;
    // ����Ҫ�ж��Ƿ���Ҫ����
    // 0�����û�������������ִ�У�ֱ�����ת��
    // 1����Ҫ������ת�ٵ������SendGearLevel�ڲ������ȷ���
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

    if(SFA) // ��������˻������ܣ����ת�ٸ���
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

// ���ݵ�ǰ״̬��ȡ����ת��
CncPolar SpindleControl::CalPolar()
{
    if(!spindle)
        return Stop;
    int polar = Stop; // �ٶȼ��� -1:δ��ʼ�� 0:�� 1:�� 2:ͣ
    if(SSIN == 0) // ������cnc��ȷ��
    {
        // ���ᶨ���ܣ������ɲ���ORM�趨
        if(SOR == 1)
        {
            return (CncPolar)ORM;
        }
        /*
         * TCW    CWM    ����
         *  0      0     ��
         *  0      1     ��
         *  1      0     M03Ϊ��  M04Ϊ��
         *  1      1     M04Ϊ��  M03Ϊ��
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
    else // ������pmc�ź�SGN��ȷ��
    {
        polar = SGN;
    }
    return (CncPolar)polar;
}

// ���ݵ�ǰ״̬��ȡDA��ƽֵ
int32_t SpindleControl::CalDaOutput()
{
    if(!spindle)
        return 0;
    int32_t output; // ת��
    uint16_t max_spd = GetMaxSpeed();

    if(SIND == 0) // �ٶ���cnc��ȷ��
    {
        int32_t rpm = 0;
        if(SOR == 0)
        {
            rpm = cnc_speed;  // ��λ:rpm
        }
        else
        {
            rpm = spindle->spd_sor_speed;
        }
        rpm *= SOV/100.0; // ���Ա���
        output = da_prec * (1.0 * rpm/max_spd);   // ת��Ϊ��ƽֵ
    }
    else // �ٶ���pmc��ȷ��
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
    // ��ȡ����
    CncPolar polar = CalPolar();
    if(polar == Stop)
    {
        return false;
    }

    uint8_t GR1O = 0, GR2O = 0, GR3O = 0;
    CalLevel(GR1O, GR2O, GR3O);

    printf("GRO: %d %d %d %d %d %d\n",F->GR1O,F->GR2O,F->GR3O,
           GR1O,GR2O,GR3O);

    // ��λѡ���ź���Ҫ�޸�
    if(GR1O != F->GR1O || GR2O != F->GR2O || GR3O != F->GR3O)
    {
        F->GR1O = GR1O;
        F->GR2O = GR2O;
        F->GR3O = GR3O;

        // SFAʹ��ʱ����ִ�л�������
        if(SFA && SIND == 0)
        {
            //�첽�������߼�
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

    if(SSIN)    // ת����PMC�����ƣ����ֵ�ǰ��λ
    {
        if(level == Low)
            GR1O = 1;
        else if(level == Middle)
            GR2O = 1;
        else
            GR3O = 1;
    }
    else if(SGB == 0) // A������ʽ
    {
        //��λ1��0~��λ1���ת��
        //��λ2����λ1���ת��~��λ2���ת��
        //��λ3����λ2���ת��~��λ3���ת��
        if(cnc_speed <= spindle->spd_gear_speed_low) //��λ1
        {
            GR1O = 1;
        }
        else if(cnc_speed <= spindle->spd_gear_speed_middle) // ��λ2
        {
            GR2O = 1;
        }
        else    // ��λ3
        {
            GR3O = 1;
        }

    }
    else if(SGB == 1) // B������ʽ
    {
        //��λ1��0~�е͵��л��ٶ�
        //��λ2���е͵��л��ٶ�~�иߵ��л��ٶ�
        //��λ3���иߵ��л��ٶ�~�������ת��
        if(cnc_speed <= spindle->spd_gear_switch_speed1) // ��λ1
        {
            GR1O = 1;
        }
        else if(cnc_speed <= spindle->spd_gear_switch_speed2) // ��λ2
        {
            GR2O = 1;
        }
        else    // ��λ3
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
    // ��˿״̬�½�ֹ�޸�ת��
    if(RGTAP){
        CreateError(ERR_SPD_RUN_IN_TAP,
                    ERROR_LEVEL,
                    CLEAR_BY_MCP_RESET);
        return;
    }

    // ��ȡ����
    polar = CalPolar();
    if(polar == Stop)
    {
        output = 0;
    }
    else
    {
        // ��ȡ�ٶ�
        output = CalDaOutput();
        // �ٶ������PMC�������Ƿ���
        F->RO = output;
    }

    output *= spindle->spd_analog_gain/1000.0;     // ��������

    if(polar == Negative)
    {
        output *= -1;
    }

    output += spindle->zero_compensation; // ������Ư

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

    //��ʱTM us����SF�ź�
    std::this_thread::sleep_for(std::chrono::microseconds(TM));
    F->SF = 1;
    printf("set SF 1\n");

    // ��¼��λ
    if(F->GR1O){
        to_level = Low;
    }else if(F->GR2O){
        to_level = Middle;
    }else if(F->GR3O){
        to_level = High;
    }else{
        printf("Spindle error: unknowed level!");
    }

    // ��ʱTMF�����ٶ�
    std::this_thread::sleep_for(std::chrono::microseconds(TMF));
    SendSpdSpeedToMi();

    //�ȴ�SF�źŹر�(�յ�FIN�źź�ProcessPmcSignal�л��SF����Ϊ0)
    int delay = 0;
    while(F->SF == 1 && delay < 5000)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
        delay += 10;
        if(F->SF == 0)  // PMC��Ӧ��SF�źţ������ɹ�
        {
            // Ŀ�굵λ��ֵ����ǰ��λ
            level = to_level;
            printf("change level: %d\n",level);
            break;
        }
    }
    if(F->SF == 1) // PMC����ʱ
    {
        // �����Ŀ�굵λ
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
