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

// 主轴方向输入
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
        // 收到正/反转信号
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

    	// 收到主轴停信号
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
        // 速度降为0
    	SendSpdSpeedToMi(0);
    	// 检测速度将为0
    	int count = 0;

    	while(spindle->axis_interface != 0 && fabs(GetSpindleSpeed()) > 1)
    	{
    		count ++;
    		usleep(10000);
    		if(count > 200) break;
    	}

    	// 如果主轴不在使能状态，先上使能
    	if(!motor_enable){
    		mi->SendAxisEnableCmd(phy_axis+1, true);
    		// 等待主轴上使能回复  2s超时
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
    // 模式的修改等mi命令回复
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

// 开始刚性攻丝
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

    tap_ratio = -10000.0*cnc_speed*spindle->move_pr/(feed*TapDir);

    // 攻丝回退要特殊处理
    if(running_rtnt && tap_state.tap_flag){
        tap_ratio = -10000.0*tap_state.S*spindle->move_pr/(tap_state.F*tap_state.dir);
    }


    ScPrintf("=====================================\n");
    ScPrintf("running_rtnt=%u S=%u  F=%lf ratio=%lf\n",
    		running_rtnt,tap_state.S,tap_state.F, tap_ratio);
    //if(TSO)
    //    ratio *= SOV/100.0;

    //发送攻丝轴号
    mi->SendTapAxisCmd(chn, phy_axis+1, z_axis+1);
    printf("SendTapAxisCmd spd=%d,z=%d\n",phy_axis+1,z_axis+1);
    //发送攻丝参数
    mi->SendTapParams(chn,(uint32_t)(spindle->spd_sync_error_gain*1000),
    		(uint32_t)(spindle->spd_speed_feed_gain*1000),
			(uint32_t)(spindle->spd_pos_ratio_gain*1000));
    //发送攻丝比例
    mi->SendTapRatioCmd(chn, tap_ratio);
    //打开攻丝状态
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

// 取消刚性攻丝
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

    // 给信号梯图来取消攻丝
    if(F->RGSPP){

    	F->RGSPP = 0;
    }

}

void SpindleControl::ResetTapFlag()
{
    tap_state.tap_flag = false;
    SaveTapState();
}

// _SSTP信号输入 低有效   主轴停止信号
void SpindleControl::InputSSTP(bool _SSTP)
{
    if(!spindle)
        return;

    ScPrintf("SpindleControl::InputSSTP _SSTP = %d\n", _SSTP);

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

// 主轴准停
void SpindleControl::InputSOR(bool SOR)
{
    if(!spindle)
        return;
    printf("InputSOR:SOR = %d\n",SOR);
    ScPrintf("InputSOR:SOR = %d\n",SOR);
    this->SOR = SOR;
    // 修改主轴使能状态
    if(!_SSTP && SOR)
        F->ENB = 1;
    UpdateSpindleState();
}

// 设置主轴倍率
void SpindleControl::InputSOV(uint8_t SOV)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputSOV SOV = %d\n", SOV);

    // 位置模式下不能修改主轴倍率
    if(mode == Position)
        return;
    this->SOV = SOV;
    UpdateSpindleState();
}

// 主轴转速外部输入
void SpindleControl::InputRI(uint16_t RI)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputRI RI = %d\n", RI);

    this->RI = RI;
    if(SIND == 1) // 主轴速度由PMC确定时，才需要更新速度
    {
        UpdateSpindleState();
    }
}

// 主轴方向PMC输入
void SpindleControl::InputSGN(bool SGN)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputSGN SGN = %d\n", SGN);

    this->SGN = SGN;
    if(SSIN == 1) // 主轴极性由PMC确定时，才需要更新速度
    {
        UpdateSpindleState();
    }
}

// 主轴方向控制选择  0 CNC控制  1 PMC控制
void SpindleControl::InputSSIN(bool SSIN)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputSSIN SSIN = %d\n", SSIN);

    this->SSIN = SSIN;
    UpdateSpindleState();
}

// 主轴速度控制选择    0 CNC控制  1 PMC控制
void SpindleControl::InputSIND(bool SIND)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputSIND SIND = %d\n", SIND);

    this->SIND = SIND;
    UpdateSpindleState();
}

// 定位信号
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

// 刚性攻丝模式切换 同步
void SpindleControl::InputRGTAP(bool RGTAP)
{
    if(!spindle)
        return;
    ScPrintf("SpindleControl::InputRGTAP RGTAP = %d\n", RGTAP);

    this->RGTAP = RGTAP;
    if(RGTAP){
        // 如果主轴不在使能状态，先上使能
    	if(!motor_enable){
    		mi->SendAxisEnableCmd(phy_axis+1, true);
    		// 等待主轴上使能回复  2s超时
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

// 主轴控制模式切换   位置模式
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

// 攻丝回退信号
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

    // 加载攻丝状态
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

// 主轴定位
void SpindleControl::RspORCMA(bool success)
{
    if(!spindle)
        return;

	// @test zk
	gettimeofday(&time_end, NULL);

	//printf("=============== time elapse %d sec\n", time_end.tv_sec - time_start.tv_sec);
	//printf("=============== time elapse %d usec\n", time_end.tv_usec - time_start.tv_usec);

    ScPrintf("SpindleControl::RspORCMA : success = %d\n",success);
    // 定位成功，将ORAR置为1，通知PMC定位动作完成
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
        // 正在等待下使能，并且收到了回复
        motor_enable = enable;
        if(wait_off && !enable){
            wait_off = false;
            F->SST = 1;
            F->SAR = 0;   //掉使能 速度到达信号复位
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
    ScPrintf("SpindleControl::RspSpindleSpeed: axis=%d,success=%d\n",axis,success);
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

// 根据当前状态更新转速
void SpindleControl::UpdateSpindleState()
{
    if(!spindle){
    	return;
    }
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
Polar SpindleControl::CalPolar()
{
    if(!spindle)
        return Stop;
    int polar = Stop; // 速度极性 -1:未初始化 0:正 1:负 2:停
    if(SSIN == 0) // 极性由cnc来确定
    {
    	// 主轴定向功能，极性由参数ORM设定
        if(SOR == 1)
        {
        	return (Polar)ORM;
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

    return (Polar)polar;
}

// 根据当前状态获取DA电平值
int32_t SpindleControl::CalDaOutput()
{
    if(!spindle)
        return 0;
    int32_t output = 0; // 转速
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

        // @add zk 模拟量主轴转速
		if(rpm > spindle->spd_max_speed)
			rpm = spindle->spd_max_speed;
		if(rpm < spindle->spd_min_speed)
			rpm = spindle->spd_min_speed;

		cnc_speed_virtual = rpm;

		if(spindle->spd_vctrl_mode == 1){
			output = da_prec * (1.0 * rpm/max_spd);   // 转化为电平值
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
    static std::future<void> ans;
    if(!spindle)
        return false;
    // 获取方向
    Polar polar = CalPolar();
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
    Polar polar;
    int32_t output;
    // 位置模式
    if(mode == Position){
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
        printf("===== F->RO: %d\n", output);
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
        /*// 如果主轴不在使能状态，先上使能
        if(!motor_enable)
            mi->SendAxisEnableCmd(phy_axis+1, true);
        //std::this_thread::sleep_for(std::chrono::microseconds(1000 * 1000));

        // 等待主轴上使能回复  2s超时
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
		// 定向前主轴必定下使能  等下使能了再执行上使能
		//while(motor_enable){usleep(10000);};

		// 如果主轴不在使能状态，先上使能
    	if(!motor_enable){
    		mi->SendAxisEnableCmd(phy_axis+1, true);
    		//std::this_thread::sleep_for(std::chrono::microseconds(1000 * 1000));

    		// 等待主轴上使能回复  2s超时
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
        // 取消定向时，根据之前状态来恢复电机使能
        /*if(motor_enable != need_enable)
            mi->SendAxisEnableCmd(phy_axis+1, need_enable);


        // 等待主轴上使能回复  2s超时
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
    	// @add zk 取消定向时 定向完成置零
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
    	// 退出位置模式下使能
    	F->RGMP = 0;
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

void SpindleControl::SaveTapState()
{
    ScPrintf("SaveTapState\n");
    int fp = open(PATH_TAP_STATE, O_CREAT | O_RDWR);
    if(fp < 0){         //文件打开失败
        printf("Open tap state fail!");
        return;
    }

    ssize_t res = write(fp, &tap_state, sizeof(TapState));
    if(res == -1){      //写入失败
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
    if(fp < 0){         //文件打开失败
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

// 攻丝回退处理
void SpindleControl::ProcessRTNT()
{
	// 如果主轴不在使能状态，先上使能
    // 恢复攻丝状态
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

    // 等待回退到位
    DPointChn pos_work = control->GetRealtimeStatus().cur_pos_work;
    running_rtnt = true;

    while(fabs(pos_work.GetAxisValue(z_axis) - R) > 0.005){
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
        pos_work = control->GetRealtimeStatus().cur_pos_work;
        if(!running_rtnt)
            break;
    }

    running_rtnt = false;
    // 没回退到位
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

	//规避二次攻丝回退线程卡死
	running_rtnt = false;

	InputPolar(Stop);
}
