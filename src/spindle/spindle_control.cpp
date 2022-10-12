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
        // �յ���/��ת�ź�
        if(!motor_enable) //���᲻��ʹ��״̬����Ҫ��ʹ��
        {
            mi->SendAxisEnableCmd(phy_axis+1, true);
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
        wait_sar = true;
        UpdateSpindleState();
    }else{
        // �յ�����ͣ�ź�
        if(!motor_enable) //�����Ѿ�����ʹ��״̬
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

    // ģʽ���޸ĵ�mi����ظ�
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
    // todo:�������λ��ģʽ���������ٶ�ģʽ�²��ܸ��Թ�˿

    // todo:�������Ϊ0����������˿�����쳣
    if(feed == 0){
        return;
    }
    // ratio:��˿������10000*S/F��S��λΪrpm��F��λΪmm/min
    int32_t ratio = -10000.0*cnc_speed*spindle->move_pr/feed;
    if(TSO)
        ratio *= SOV;

    // ���͹�˿���
    mi->SendTapAxisCmd(chn, phy_axis+1, z_axis+1);
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
}

void SpindleControl::CancelRigidTap()
{
    if(!spindle)
        return;
    SendMcRigidTapFlag(false);
    mi->SendTapStateCmd(chn, false);
    mi->SendTapRatioCmd(chn, 0);

    tap_enable = false;

    // ���ź���ͼ��ȡ����˿
    if(RGTAP)
        F->RGSPP = 0;

    // todo:��һ��ʱ���ڵȴ��л��ٶ�ģʽ�����򱨾���ȡ�����Թ�˿��û�н����ٶ�ģʽ
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

    // ģʽ��һ�����л�
    if(mode != toMode){
        SetMode(toMode);
    }
}

void SpindleControl::RspORCMA(bool success)
{
    if(!spindle)
        return;
    // ��λ�ɹ�����ORAR��Ϊ1��֪ͨPMC��λ�������
    if(success && ORCMA){
        F->ORAR = 1;
    }
}

void SpindleControl::RspCtrlMode(uint8_t axis, Spindle::Mode mode)
{
    if(!spindle)
        return;
    if(axis == phy_axis+1){
        // ���Թ�˿�ź�Ϊ1��״̬�£��л�����λ��ģʽ����ô׼����˿
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
    // ���ڵȴ���ʹ�ܣ������յ��˻ظ�
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
    // ���ڵȴ��ٶȵ�������յ��˻ظ�
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
        if(GST == 0 && SOR == 1)
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

    // ��λѡ���ź���Ҫ�޸�
    if(GR1O != F->GR1O || GR2O != F->GR2O || GR3O != F->GR3O)
    {
        F->GR1O = GR1O;
        F->GR2O = GR2O;
        F->GR3O = GR3O;

        // SFAʹ��ʱ����Ҫ����SF�ź�
        if(SFA)
        {
            //��ʱTM ms����SF�ź�
            std::this_thread::sleep_for(std::chrono::microseconds(TM));
            F->SF = 1;

            // ��¼��λ
            if(GR1O){
                level = Low;
            }else if(GR2O){
                level = Middle;
            }else if(GR3O){
                level = High;
            }else{
                printf("Spindle error: unknowed level!");
            }

            //��ʱTMF ms�����ٶ�(�첽)
            SpindleControl s;
            auto func = std::bind(&SpindleControl::DelaySendSpdSpeedToMi,
                                  this, std::placeholders::_1);
            std::async(std::launch::async, func, TMF);

            //            //�ȴ�SF�źŹر�(�յ�FIN�źź�ProcessPmcSignal�л��SF����Ϊ0)
            //            int delay = 0;
            //            while(F->SF == 1 || delay > 5000)
            //            {
            //                std::this_thread::sleep_for(std::chrono::microseconds(10));
            //                delay += 10;
            //                if(F->SF == 0)  // PMC��Ӧ��SF�źţ������ɹ�
            //                {
            //                    SwLevelSuccess();
            //                }
            //            }
            //            if(F->SF == 1) // PMC����ʱ
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
    if(SGB == 0) // A������ʽ
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
    // ��˿״̬�½�ֹ�޸�ת��
    if(RGTAP)
        return;

    // ��ȡ����
    CncPolar polar = CalPolar();
    if(polar == Stop)
    {
        return;
    }

    // ��ȡ�ٶ�
    int32_t output = CalDaOutput();

    // �ٶ������PMC�������Ƿ���
    F->RO = output;

    output *= spindle->spd_analog_gain/1000.0;     // ��������

    if(polar == Negative)
    {
        output *= -1;
    }

    output += spindle->zero_compensation; // ������Ư

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
