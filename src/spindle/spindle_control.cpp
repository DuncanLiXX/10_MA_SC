#include <future>
#include <functional>
#include <unistd.h>
#include "spindle_control.h"
#include "trace.h"
#include "mc_communication.h"
#include "mi_communication.h"
#include "parm_definition.h"
#include "global_include.h"
#include "channel_control.h"
#include "channel_engine.h"
#include "global_definition.h"
#include "pmc_register.h"
#include "variable.h"
#include "channel_data.h"

using namespace Spindle;

static struct timeval time_start;
static struct timeval time_end;


SpindleControl::SpindleControl()
{
}

void SpindleControl::SetComponent(MICommunication *mi,
                                  MCCommunication *mc,
                                  FRegBits *f_reg,
                                  const GRegBits *g_reg,
                                  Variable *variable)
{
    this->mi = mi;
    this->mc = mc;
    this->F = f_reg;
    this->G = g_reg;
    this->variable = variable;

    mi->SendMiParam<uint16_t>(phy_axis+1, 1726, uint16_t(spindle->spd_locate_ang*100));

    InputSSTP(G->_SSTP);
    InputSOR(G->SOR);
    InputSOV(G->SOV);
    InputRI(G->RI);
    InputSGN(G->SGN);
    InputSSIN(G->SSIN);
    InputSIND(G->SIND);
    LoadTapState(tap_state);
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
    running_rtnt = false;

    UpdateParams();
}

void SpindleControl::InputSCode(uint32_t s_code)
{
    if(!spindle)
        return;

    ScPrintf("SpindleControl::InputSCode s_code = %d\n", s_code);
    cnc_speed = s_code;
    if(cnc_speed > spindle->spd_max_speed)
        cnc_speed = spindle->spd_max_speed;
    if(cnc_speed < spindle->spd_min_speed)
        cnc_speed = spindle->spd_min_speed;

    F->scode_0 = (cnc_speed&0xFF);
    F->scode_1 = ((cnc_speed>>8)&0xFF);
    F->scode_2 = ((cnc_speed>>16)&0xFF);
    F->scode_3 = ((cnc_speed>>24)&0xFF);

    if(CalPolar() == Stop)
        return;
    UpdateSpindleState();
}

uint32_t SpindleControl::GetSCode()
{
    return cnc_speed;
}

// ���᷽������
void SpindleControl::InputPolar(Spindle::Polar polar)
{
    if(!spindle){

    	printf("inputPolar return no spindle\n");
    	return;
    }

    if(this->ORCMA){
    	printf("inputPolar return ORCMA\n");
    	return;
    }

    ScPrintf("SpindleControl::InputPolar %d\n",polar);

    cnc_polar = polar;

    if(polar == Positive || polar == Negative){
        // �յ���/��ת�ź�
        if(motor_enable){
            wait_sar = true;
            UpdateSpindleState();
        }else{
            wait_on = true;
            mi->SendAxisEnableCmd(phy_axis+1, true);
            printf("SendAxisEnable enable = %d\n",true);
        }
    }else{

    	SendSpdSpeedToMi(0);

    	int count = 0;

		while(spindle->axis_interface != 0 && fabs(GetSpindleSpeed()) > 1)
		{
			count ++;
			usleep(10000);
			if(count > 200)
				break;
		}

    	// �յ�����ͣ�ź�
        if(!motor_enable){
            F->SST = 1;
        }else{
            wait_off = true;
            mi->SendAxisEnableCmd(phy_axis+1,false);
            printf("SendAxisEnable enable = %d\n",false);
        }
    }
}

// M26 M27

void SpindleControl::SetMode(Mode mode)
{
    if(!spindle)
        return;

    if(mode == Speed){
    	mi->SendAxisEnableCmd(phy_axis+1, false);
    	mi->SendAxisCtrlModeSwitchCmd(phy_axis+1, 2);
    }else if(mode == Position){
        // �ٶȽ�Ϊ0
    	SendSpdSpeedToMi(0);
    	// ����ٶȽ�Ϊ0
    	int count = 0;

    	while(spindle->axis_interface != 0 && fabs(GetSpindleSpeed()) > 1)
    	{
    		count ++;
    		usleep(10000);
    		if(count > 200) break;
    	}

    	// ������᲻��ʹ��״̬������ʹ��
    	if(!motor_enable){
    		mi->SendAxisEnableCmd(phy_axis+1, true);
    		// �ȴ�������ʹ�ܻظ�  2s��ʱ
    		int count = 0;
    		while(!motor_enable){
    			count ++;
    			if(count > 200){
    				CreateError(ERR_SPD_TAP_START_FAIL,
    							ERROR_LEVEL,
    							CLEAR_BY_MCP_RESET);
    				count = 0;
    				return;
    			}
    			usleep(10000);
    		}
    		count = 0;
    	}

    	mi->SendAxisCtrlModeSwitchCmd(phy_axis+1, 1);
    }
    // ģʽ���޸ĵ�mi����ظ�
}

Mode SpindleControl::GetMode()
{
    return mode;
}

uint8_t SpindleControl::Type()
{
    if(!spindle)
        return 0;
    else if(spindle->axis_interface == 0)
        return 1;
    else
        return 2;
}

bool SpindleControl::isTapEnable()
{
    return tap_enable;
}

void SpindleControl::SetTapFeed(double feed)
{
    tap_feed = feed;
}

// ��ʼ���Թ�˿
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

    tap_ratio = -10000.0*cnc_speed*spindle->move_pr/(feed*TapDir);

    // ��˿����Ҫ���⴦��
    if(running_rtnt && tap_state.tap_flag){
        tap_ratio = -10000.0*tap_state.S*spindle->move_pr/(tap_state.F*tap_state.dir);
    }


    ScPrintf("=====================================\n");
    ScPrintf("running_rtnt=%u S=%u  F=%lf ratio=%lf\n",
    		running_rtnt,tap_state.S,tap_state.F, tap_ratio);
    //if(TSO)
    //    ratio *= SOV/100.0;

    //���͹�˿���
    mi->SendTapAxisCmd(chn, phy_axis+1, z_axis+1);
    printf("SendTapAxisCmd spd=%d,z=%d\n",phy_axis+1,z_axis+1);
    //���͹�˿����
    mi->SendTapParams(chn,(uint32_t)(spindle->spd_sync_error_gain*1000),
    		(uint32_t)(spindle->spd_speed_feed_gain*1000),
			(uint32_t)(spindle->spd_pos_ratio_gain*1000));
    //���͹�˿����
    mi->SendTapRatioCmd(chn, tap_ratio);
    //�򿪹�˿״̬
    SendMcRigidTapFlag(true);
    mi->SendTapStateCmd(chn,true, running_rtnt);
    tap_enable = true;
    std::this_thread::sleep_for(std::chrono::microseconds(100*1000));
    printf("********************** RGSPP = 1\n");
    F->RGSPP = 1;

    tap_state.tap_flag = true;
    tap_state.F = feed;
    if(cnc_speed > 0) tap_state.S = cnc_speed;
    tap_state.polar = cnc_polar;
    tap_state.phy_axis = phy_axis;
    tap_state.z_axis = z_axis;
    tap_state.dir = TapDir;
    double R = 0.0;
    bool inited = false;
    if(variable->GetVarValue(188,R,inited)){
        tap_state.R = R;
    }else{
        return;
    }
    SaveTapState();
}

// ȡ�����Թ�˿
void SpindleControl::CancelRigidTap()
{
    if(!spindle || spindle->axis_interface == 0)
        return;

    ChannelEngine *engine = ChannelEngine::GetInstance();
    ChannelControl *control = engine->GetChnControl(0);
	ChannelRealtimeStatus status = control->GetRealtimeStatus();

	SendMcRigidTapFlag(false);

	mi->SendTapStateCmd(chn, false);

    mi->SendTapRatioCmd(chn, 0);

    std::this_thread::sleep_for(std::chrono::microseconds(100*1000));

    double pos = status.cur_pos_machine.GetAxisValue(phy_axis);
    mi->SetAxisRefCur(phy_axis+1,pos);

    tap_enable = false;

    // ���ź���ͼ��ȡ����˿
    if(F->RGSPP){

    	F->RGSPP = 0;
    }

}

void SpindleControl::ResetTapFlag()
{
    tap_state.tap_flag = false;
    SaveTapState();
}

// _SSTP�ź����� ����Ч   ����ֹͣ�ź�
void SpindleControl::InputSSTP(bool _SSTP)
{
    if(!spindle)
        return;

    ScPrintf("SpindleControl::InputSSTP _SSTP = %d\n", _SSTP);

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

// ����׼ͣ
void SpindleControl::InputSOR(bool SOR)
{
    if(!spindle)
        return;
    printf("InputSOR:SOR = %d\n",SOR);
    ScPrintf("InputSOR:SOR = %d\n",SOR);
    this->SOR = SOR;
    // �޸�����ʹ��״̬
    if(!_SSTP && SOR)
        F->ENB = 1;
    UpdateSpindleState();
}

// �������ᱶ��
void SpindleControl::InputSOV(uint8_t SOV)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputSOV SOV = %d\n", SOV);

    // λ��ģʽ�²����޸����ᱶ��
    if(mode == Position)
        return;
    this->SOV = SOV;
    UpdateSpindleState();
}

// ����ת���ⲿ����
void SpindleControl::InputRI(uint16_t RI)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputRI RI = %d\n", RI);

    this->RI = RI;
    if(SIND == 1) // �����ٶ���PMCȷ��ʱ������Ҫ�����ٶ�
    {
        UpdateSpindleState();
    }
}

// ���᷽��PMC����
void SpindleControl::InputSGN(bool SGN)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputSGN SGN = %d\n", SGN);

    this->SGN = SGN;
    if(SSIN == 1) // ���Ἣ����PMCȷ��ʱ������Ҫ�����ٶ�
    {
        UpdateSpindleState();
    }
}

// ���᷽�����ѡ��  0 CNC����  1 PMC����
void SpindleControl::InputSSIN(bool SSIN)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputSSIN SSIN = %d\n", SSIN);

    this->SSIN = SSIN;
    UpdateSpindleState();
}

// �����ٶȿ���ѡ��    0 CNC����  1 PMC����
void SpindleControl::InputSIND(bool SIND)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputSIND SIND = %d\n", SIND);

    this->SIND = SIND;
    UpdateSpindleState();
}

// ��λ�ź�
void SpindleControl::InputORCMA(bool ORCMA)
{
    static std::future<void> ans;
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputORCMA ORCMA = %d\n", ORCMA);
    auto func = std::bind(&SpindleControl::ProcessORCMA,
                          this, std::placeholders::_1);
    ans = std::async(std::launch::async, func, ORCMA);
}

// ���Թ�˿ģʽ�л� ͬ��
void SpindleControl::InputRGTAP(bool RGTAP)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputRGTAP RGTAP = %d\n", RGTAP);

    this->RGTAP = RGTAP;
    if(RGTAP){
        // ������᲻��ʹ��״̬������ʹ��
    	if(!motor_enable){
    		mi->SendAxisEnableCmd(phy_axis+1, true);
    		// �ȴ�������ʹ�ܻظ�  2s��ʱ
    		int count = 0;
    		while(!motor_enable){
    			count ++;
    			if(count > 200){
    				CreateError(ERR_SPD_TAP_START_FAIL,
    							ERROR_LEVEL,
    							CLEAR_BY_MCP_RESET);
    				count = 0;
    				return;
    			}
    			usleep(10000);
    		}
    		count = 0;
    	}
    	StartRigidTap(tap_feed);
    }
    else{
    	CancelRigidTap();
    }
}

// �������ģʽ�л�   λ��ģʽ
void SpindleControl::InputRGMD(bool RGMD)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputRGMD RGMD = %d\n", RGMD);

    this->RGMD = RGMD;

    if(RGMD)
        SetMode(Position);
    else
        SetMode(Speed);
}

// ��˿�����ź�
void SpindleControl::InputRTNT(bool RTNT)
{
    static std::future<void> ans;
    this->RTNT = RTNT;
    if(!RTNT || !spindle)
        return;
    ScPrintf("SpindleControl::InputRTNT RTNT = %d\n", RTNT);

    ChannelEngine *engine = ChannelEngine::GetInstance();
    ChannelControl *control = engine->GetChnControl(0);

    if(control->GetChnStatus().chn_work_mode == AUTO_MODE){
        CreateError(ERR_SPD_RTNT_IN_AUTO,
                    INFO_LEVEL,
                    CLEAR_BY_MCP_RESET);
        return;
    }

    // ���ع�˿״̬
    printf("===== SSIN %d SIND %d _SSTP %d\n", SSIN, SIND, _SSTP);

    //if(SSIN == 1 || SIND == 1 || _SSTP == 0 ||
    if(SSIN == 1 || SIND == 1 || !LoadTapState(tap_state) || !tap_state.tap_flag)
    {
        CreateError(ERR_SPD_RTNT_INVALID,
                    ERROR_LEVEL,
                    CLEAR_BY_MCP_RESET);
        return;
    }

    auto func = std::bind(&SpindleControl::ProcessRTNT,
                          this);
    ans = std::async(std::launch::async, func);
}

// ���ᶨλ
void SpindleControl::RspORCMA(bool success)
{
    if(!spindle)
        return;

	// @test zk
	gettimeofday(&time_end, NULL);

	//printf("=============== time elapse %d sec\n", time_end.tv_sec - time_start.tv_sec);
	//printf("=============== time elapse %d usec\n", time_end.tv_usec - time_start.tv_usec);

    ScPrintf("SpindleControl::RspORCMA : success = %d\n",success);
    // ��λ�ɹ�����ORAR��Ϊ1��֪ͨPMC��λ�������
    if(success && ORCMA){
    	F->ORAR = 1;
    }
}

void SpindleControl::RspCtrlMode(uint8_t axis, Spindle::Mode mode)
{
    static std::future<void> ans;
    if(!spindle)
        return;
    ScPrintf("RspCtrlMode:axis = %d,mode = %d\n",axis,mode);
    if(axis == phy_axis + 1){
        auto func = std::bind(&SpindleControl::ProcessModeChanged,
                              this, std::placeholders::_1);
        ans = std::async(std::launch::async, func, mode);
    }
}

void SpindleControl::RspAxisEnable(uint8_t axis, bool enable)
{
    if(!spindle)
        return;
    ScPrintf("RspAxisEnable:axis = %d,enable = %d\n",axis,enable);

    if(axis == phy_axis+1){
        // ���ڵȴ���ʹ�ܣ������յ��˻ظ�
        motor_enable = enable;
        if(wait_off && !enable){
            wait_off = false;
            F->SST = 1;
            F->SAR = 0;   //��ʹ�� �ٶȵ����źŸ�λ
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
    ScPrintf("SpindleControl::RspSpindleSpeed: axis=%d,success=%d\n",axis,success);
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
        if(cnc_polar == Polar::Positive){
        	return cnc_speed_virtual;
        }else{
        	return -cnc_speed_virtual;
        }
    }

    int32_t umps;
    if(!mi->ReadPhyAxisSpeed(&umps, phy_axis)){
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
    double speed = (double)umps*60.0/(spindle->move_pr*1000.0);
    return (int32_t)speed;
}

double SpindleControl::GetSpdAngle()
{
    if(!spindle)
        return 0;

    ChannelEngine *engine = ChannelEngine::GetInstance();
    return engine->GetSpdAngleFeedback(phy_axis);
//    double pos = engine->GetPhyAxisMachPosFeedback(phy_axis);
//    double model = fmod(pos, spindle->move_pr);
//    if(model < 0)
//        model += spindle->move_pr;
//    return (model/spindle->move_pr)*360.0;
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
    if(!spindle){
    	return;
    }
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
Polar SpindleControl::CalPolar()
{
    if(!spindle)
        return Stop;
    int polar = Stop; // �ٶȼ��� -1:δ��ʼ�� 0:�� 1:�� 2:ͣ
    if(SSIN == 0) // ������cnc��ȷ��
    {
    	// ���ᶨ���ܣ������ɲ���ORM�趨
        if(SOR == 1)
        {
        	return (Polar)ORM;
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

    return (Polar)polar;
}

// ���ݵ�ǰ״̬��ȡDA��ƽֵ
int32_t SpindleControl::CalDaOutput()
{
    if(!spindle)
        return 0;
    int32_t output = 0; // ת��
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

        // @add zk ģ��������ת��
		if(rpm > spindle->spd_max_speed)
			rpm = spindle->spd_max_speed;
		if(rpm < spindle->spd_min_speed)
			rpm = spindle->spd_min_speed;

		cnc_speed_virtual = rpm;

		if(spindle->spd_vctrl_mode == 1){
			output = da_prec * (1.0 * rpm/max_spd);   // ת��Ϊ��ƽֵ
		}else if(spindle->spd_vctrl_mode == 2){
			int zero_offset = da_prec/2;
			if(cnc_polar == Polar::Positive){
				output = zero_offset * (1.0*rpm/max_spd);
				output = zero_offset + output;
			}else{
				output = zero_offset * (1.0*rpm/max_spd);
				output = zero_offset - output;
			}
		}else{
			output = 0;
		}


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
    static std::future<void> ans;
    if(!spindle)
        return false;
    // ��ȡ����
    Polar polar = CalPolar();
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
            ans = std::async(std::launch::async, func, TMF);

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
    Polar polar;
    int32_t output;
    // λ��ģʽ
    if(mode == Position){
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
        printf("===== F->RO: %d\n", output);
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
    ScPrintf("send spindle DA output: %d\n",output);
    mi->WriteCmd(cmd);
}

void SpindleControl::SendSpdSpeedToMi(int16_t speed){
	int16_t output = 0;
	output += spindle->zero_compensation;
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_SET_SPD_SPEED;
	cmd.data.axis_index = phy_axis+1;
	cmd.data.data[0] = (int16_t)output;
	mi->WriteCmd(cmd);
}

void SpindleControl::ProcessORCMA(bool ORCMA)
{

	//std::this_thread::sleep_for(std::chrono::microseconds(100 * 1000));
	printf("===== ProcessORCMA %d\n", ORCMA);

	this->ORCMA = ORCMA;

	if(ORCMA){
        /*// ������᲻��ʹ��״̬������ʹ��
        if(!motor_enable)
            mi->SendAxisEnableCmd(phy_axis+1, true);
        //std::this_thread::sleep_for(std::chrono::microseconds(1000 * 1000));

        // �ȴ�������ʹ�ܻظ�  2s��ʱ
        count = 0;
        while(!motor_enable){
        	count ++;
        	if(count > 200){
        		printf("1111111111111111111\n");

        		count = 0;
        		return;
        	}
        	usleep(10000);
        }
        count = 0;*/
		// ����ǰ����ض���ʹ��  ����ʹ������ִ����ʹ��
		//while(motor_enable){usleep(10000);};

		// ������᲻��ʹ��״̬������ʹ��
    	if(!motor_enable){
    		mi->SendAxisEnableCmd(phy_axis+1, true);
    		//std::this_thread::sleep_for(std::chrono::microseconds(1000 * 1000));

    		// �ȴ�������ʹ�ܻظ�  2s��ʱ
    		int count = 0;
    		while(!motor_enable){
    			count ++;
    			if(count > 200){
    				printf("333333333333333333333333\n");
    				CreateError(ERR_SPD_LOCATE_FAIL,
    							ERROR_LEVEL,
    							CLEAR_BY_MCP_RESET);
    				count = 0;
    				return;
    			}
    			usleep(10000);
    		}
    		count = 0;
    	}
    	// @test zk
    	gettimeofday(&time_start, NULL);
        mi->SendSpdLocateCmd(chn, phy_axis+1,true);
    }else{
        //Polar polar = CalPolar();
        //bool need_enable = (polar == Positive || polar == Negative);
        // ȡ������ʱ������֮ǰ״̬���ָ����ʹ��
        /*if(motor_enable != need_enable)
            mi->SendAxisEnableCmd(phy_axis+1, need_enable);


        // �ȴ�������ʹ�ܻظ�  2s��ʱ
        count = 0;
        while(!motor_enable){
        	count ++;
        	if(count > 200){
        		printf("222222222222222222222\n");

        		count = 0;
        		return;
        	}
        	usleep(10000);
        }
        count = 0;*/
    	// @add zk ȡ������ʱ �����������
    	F->ORAR = 0;
    	mi->SendAxisEnableCmd(phy_axis+1, false);
    	mi->SendSpdLocateCmd(chn, phy_axis+1,false);
    }

    return;
}

void SpindleControl::ProcessModeChanged(Spindle::Mode mode)
{
    printf("========== ProcessModeChanged  %d\n", mode);

    if(mode == Position){
    	ChannelEngine *engine = ChannelEngine::GetInstance();
        ChannelControl *control = engine->GetChnControl(0);
        ChannelRealtimeStatus status = control->GetRealtimeStatus();
        double pos = status.cur_pos_machine.GetAxisValue(phy_axis);
        mi->SetAxisRefCur(phy_axis+1,pos);
        //printf("SetAxisRefCur: axis=%d, pos = %lf\n",phy_axis+1,pos);
        //control->SyncMcPosition();
        std::this_thread::sleep_for(std::chrono::microseconds(100*1000));
        F->RGMP = 1;
    }else{
    	// �˳�λ��ģʽ��ʹ��
    	F->RGMP = 0;
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

void SpindleControl::SaveTapState()
{
    ScPrintf("SaveTapState\n");
    int fp = open(PATH_TAP_STATE, O_CREAT | O_RDWR);
    if(fp < 0){         //�ļ���ʧ��
        printf("Open tap state fail!");
        return;
    }

    ssize_t res = write(fp, &tap_state, sizeof(TapState));
    if(res == -1){      //д��ʧ��
        close(fp);
        printf("Write tap state fail!");
        return;
    }
    fsync(fp);
    close(fp);
}

bool SpindleControl::LoadTapState(TapState &state)
{
    ScPrintf("LoadTapState\n");
    TapState tmp;
    int fp = open(PATH_TAP_STATE, O_RDWR);
    if(fp < 0){         //�ļ���ʧ��
        printf("Read tap state fail!");
        return false;
    }

    uint16_t read_size = read(fp, &tmp, sizeof (tmp));
    if(read_size != sizeof(tmp)){
        close(fp);
        printf("Tap state size error!");
        return false;
    }

    state = tmp;
    close(fp);
    return true;
}

// ��˿���˴���
void SpindleControl::ProcessRTNT()
{
	// ������᲻��ʹ��״̬������ʹ��
    // �ָ���˿״̬
    double R = tap_state.R + spindle->spd_rtnt_distance;

    variable->SetVarValue(188, R);
    variable->SetVarValue(179, tap_state.F);
    SetTapFeed(tap_state.F);
    TapDir = tap_state.dir;

    ChannelEngine *engine = ChannelEngine::GetInstance();
    ChannelControl *control = engine->GetChnControl(0);
    running_rtnt = true;

    if(!control->CallMacroProgram(6100))
    {
    	CreateError(ERR_SPD_RTNT_FAIL,
                    ERROR_LEVEL,
                    CLEAR_BY_MCP_RESET);
        running_rtnt = false;
        return;
    }

    std::this_thread::sleep_for(std::chrono::microseconds(500 * 1000));

    // �ȴ����˵�λ
    DPointChn pos_work = control->GetRealtimeStatus().cur_pos_work;
    running_rtnt = true;

    while(fabs(pos_work.GetAxisValue(z_axis) - R) > 0.005){
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
        pos_work = control->GetRealtimeStatus().cur_pos_work;
        if(!running_rtnt)
            break;
    }

    running_rtnt = false;
    // û���˵�λ
    if(fabs(pos_work.GetAxisValue(z_axis) - R) > 0.005){
    	return;
    }

    F->RTPT = 1;
    ResetTapFlag();
}

void SpindleControl::EStop(){

	if(!spindle) return;

	if(RGTAP){CancelRigidTap(); RGTAP = 0;}

	if(RGMD){SetMode(Speed); RGMD = 0;}

	//��ܶ��ι�˿�����߳̿���
	running_rtnt = false;

	InputPolar(Stop);
}
