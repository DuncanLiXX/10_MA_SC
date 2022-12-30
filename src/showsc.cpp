#include "showsc.h"
#include "parm_definition.h"
#include "alarm_processor.h"
#include "trace.h"
#include "channel_data.h"
#include <functional>
#include <future>
#include <string>
#include "spindle_control.h"
#include "pmc_axis_ctrl.h"
#include "sync_axis_ctrl.h"

ShowSc::ShowSc()
{
    trace = TraceInfo::GetInstance();
    std::future<void> * res = new std::future<void>();
    auto process = std::bind(&ShowSc::ProcessPrintThread,
                             this);
    *res = std::async(std::launch::async, process);
}

ShowSc::~ShowSc()
{

}

void ScPrintf(const char * fmt,...)
{
    char printf_buf[1024];
    va_list args;
    va_start(args,fmt);
    vsprintf(printf_buf, fmt, args);
    va_end(args);
    puts(printf_buf);
    ShowSc &showSc = Singleton<ShowSc>::instance();
    if(showSc.GetPrintType() != TypePrintOutput)
        return;
    std::string s = printf_buf;
    showSc.SendMsg(s);
}

void ShowSc::ProcessPrintThread()
{
    while(1){
        std::this_thread::sleep_for(std::chrono::microseconds(interval * 1000));
        if(print_type == TypeNone)
            continue;

        switch (print_type) {
        case TypeChnStatus:
            PrintChnStatus();break;
        case TypeMcStatus:
            PrintMcStatus();break;
        case TypeRealtimeData:
            PrintRealtimeData();break;
        case TypeSpindle:
            PrintSpindle();break;
        case TypeSysConfig:
            PrintSysConfig();break;
        case TypeChnConfig:
            PrintChnConfig();break;
        case TypeAxisConfig:{
            for(int axis = 0; axis < kMaxAxisChn; axis++){
                PrintAxisConfig(axis);
            }
        }break;
        case TypeCoordConfig:
            PrintCoordConfig();break;
        case TypeExCoordConfig:
            PrintExCoordConfig();break;
        case TypeGrbCoordConfig:
            PrintGrbCoordConfig();break;
        case TypeTooOffsetlConfig:
            PrintToolOffsetConfig();break;
        case TypeToolPotConfig:
            PrintToolPotConfig();break;
        case TypeFiveAxisConfig:
            PrintFiveAxisCoinfig();break;
        case TypeMode:
            PrintMode();break;
        case TypeWarning:
            PrintWarning();break;
        case TypeFRegState:
            PrintFRegState();break;
        case TypeGRegState:
            PrintGRegState();break;
        case TypePmcAxis:
            PrintPmcAxisCtrl();break;
        case TypeSyncAxis:
            PrintSyncAxisCtrl();break;
        default:break;
        }

        // 间隔设置为0，只打印一次
        if(interval == 0){
            print_type = TypeNone;
        }
    }
}

void ShowSc::KeyFormat(string &key)
{
    if(key.length() > 25)
        key = key.substr(0,25);
    else if(key.length() < 25)
    {
        while(key.length() < 25)
            key.insert(0," ");
    }
}

void ShowSc::AddPair(string &s, string key, uint64_t value)
{
    KeyFormat(key);
    s = s + key + "    " + to_string(value) + "\n";
}

void ShowSc::AddPair(string &s, string key, int64_t value)
{
    KeyFormat(key);
    s = s + key + "    " + to_string(value) + "\n";
}

void ShowSc::AddPair(string &s, string key, uint32_t value)
{
    KeyFormat(key);
    s = s + key + "    " + to_string(value) + "\n";
}

void ShowSc::AddPair(string &s, string key, int32_t value)
{
    KeyFormat(key);
    s = s + key + "    " + to_string(value) + "\n";
}

void ShowSc::AddPair(string &s, string key, uint16_t value)
{
    KeyFormat(key);
    s = s + key + "    " + to_string(value) + "\n";
}

void ShowSc::AddPair(string &s, string key, int16_t value)
{
    KeyFormat(key);
    s = s + key + "    " + to_string(value) + "\n";
}

void ShowSc::AddPair(string &s, string key, uint8_t value)
{
    KeyFormat(key);
    s = s + key + "    " + to_string(value) + "\n";
}

void ShowSc::AddPair(string &s, string key, int8_t value)
{
    KeyFormat(key);
    s = s + key + "    " + to_string(value) + "\n";
}

void ShowSc::AddPair(string &s, string key, double value)
{
    KeyFormat(key);
    s = s + key + "    " + to_string(value) + "\n";
}

void ShowSc::AddPair(string &s, string key, string value)
{
    KeyFormat(key);
    s = s + key + "    " + value + "\n";
}

void ShowSc::PrintChnStatus()
{
    if(!chn_status)
        return;
    string s = "";
    AddPair(s,"rated_manual_speed",chn_status->rated_manual_speed);
    AddPair(s,"workpiece_count",chn_status->workpiece_count);
    AddPair(s,"workpiece_count_total",chn_status->workpiece_count_total);
    AddPair(s,"machinetime_total",chn_status->machinetime_total);
    Mask32 func_mask = chn_status->func_state_flags;
    AddPair(s,"FS_SINGLE_LINE Mask",func_mask.CheckMask(FS_SINGLE_LINE));
    AddPair(s,"FS_DRY_RUN Mask",func_mask.CheckMask(FS_DRY_RUN));
    AddPair(s,"FS_OPTIONAL_STOP Mask",func_mask.CheckMask(FS_OPTIONAL_STOP));
    AddPair(s,"FS_HANDWHEEL_CONTOUR Mask",func_mask.CheckMask(FS_HANDWHEEL_CONTOUR));
    AddPair(s,"FS_BLOCK_SKIP Mask",func_mask.CheckMask(FS_BLOCK_SKIP));
    AddPair(s,"FS_EDIT_LOCKED Mask",func_mask.CheckMask(FS_EDIT_LOCKED));
    AddPair(s,"FS_MACHINE_LOCKED Mask",func_mask.CheckMask(FS_MACHINE_LOCKED));
    AddPair(s,"FS_AUXILIARY_LOCKED Mask",func_mask.CheckMask(FS_AUXILIARY_LOCKED));
    AddPair(s,"FS_MANUAL_RAPID Mask",func_mask.CheckMask(FS_MANUAL_RAPID));
    AddPair(s,"rated_spindle_speed",chn_status->rated_spindle_speed);
    AddPair(s,"rated_feed",chn_status->rated_feed);
    if(chn_status->chn_work_mode == AUTO_MODE)
        AddPair(s,"work mode","AUTO_MODE");
    else if(chn_status->chn_work_mode == MDA_MODE)
        AddPair(s,"work mode","MDA_MODE");
    else if(chn_status->chn_work_mode == MANUAL_STEP_MODE)
        AddPair(s,"work mode","MANUAL_STEP_MODE");
    else if(chn_status->chn_work_mode == MANUAL_MODE)
        AddPair(s,"work mode","MANUAL_MODE");
    else if(chn_status->chn_work_mode == MPG_MODE)
        AddPair(s,"work mode","MPG_MODE");
    else if(chn_status->chn_work_mode == EDIT_MODE)
        AddPair(s,"work mode","EDIT_MODE");
    else if(chn_status->chn_work_mode == REF_MODE)
        AddPair(s,"work mode","REF_MODE");
    else
        AddPair(s,"work mode","INVALID_MODE");
    if(chn_status->machining_state == MS_READY)
        AddPair(s,"machining_state","MS_READY");
    else if(chn_status->machining_state == MS_RUNNING)
        AddPair(s,"machining_state","MS_RUNNING");
    else if(chn_status->machining_state == MS_OUTLINE_SIMULATING)
        AddPair(s,"machining_state","MS_OUTLINE_SIMULATING");
    else if(chn_status->machining_state == MS_TOOL_PATH_SIMULATING)
        AddPair(s,"machining_state","MS_TOOL_PATH_SIMULATING");
    else if(chn_status->machining_state == MS_MACHIN_SIMULATING)
        AddPair(s,"machining_state","MS_MACHIN_SIMULATING");
    else if(chn_status->machining_state == MS_PAUSING)
        AddPair(s,"machining_state","MS_PAUSING");
    else if(chn_status->machining_state == MS_PAUSED)
        AddPair(s,"machining_state","MS_PAUSED");
    else if(chn_status->machining_state == MS_STOPPING)
        AddPair(s,"machining_state","MS_STOPPING");
    else if(chn_status->machining_state == MS_REF_POINT_RETURNING)
        AddPair(s,"machining_state","MS_REF_POINT_RETURNING");
    else if(chn_status->machining_state == MS_WARNING)
        AddPair(s,"machining_state","MS_WARNING");

    AddPair(s,"cur_tool",chn_status->cur_tool);
    AddPair(s,"preselect_tool_no",chn_status->preselect_tool_no);
    AddPair(s,"cur_h_code",chn_status->cur_h_code);
    AddPair(s,"cur_d_code",chn_status->cur_d_code);
    AddPair(s,"auto_ratio",chn_status->auto_ratio);
    AddPair(s,"rapid_ratio",chn_status->rapid_ratio);
    AddPair(s,"manual_ratio",chn_status->manual_ratio);

    AddPair(s,"manual_step",chn_status->manual_step);
    AddPair(s,"cur_manual_move_dir",chn_status->cur_manual_move_dir);
    AddPair(s,"cur_axis",chn_status->cur_axis);

    string ref_s = "";
    for(int axis = 0; axis < kMaxAxisChn; axis++){
        if(chn_status->returned_to_ref_point & (0x01 << axis)){
            ref_s += "1";
        }else{
            ref_s += "0";
        }
    }
    AddPair(s,"returned_to_ref_point",ref_s);

    AddPair(s,"cur_nc_file_name",chn_status->cur_nc_file_name);
    for(int axis = 0; axis < kMaxAxisChn; axis++){
        string key = "";
        key = key + "axis_phy"+"[" + to_string(axis) +"]";
        AddPair(s,"axis_phy",chn_status->cur_chn_axis_phy[axis]);
    }

    SendMsg(s);
}

void ShowSc::PrintMcStatus()
{
    if(!mc_status)
        return;
    string s = "";
    string intp_pos_s = "[ ";
    for(int axis = 0; axis < kMaxAxisChn; axis++){
        intp_pos_s = intp_pos_s + to_string(mc_status->intp_pos.GetAxisValue(axis)) + " ";
    }
    intp_pos_s = intp_pos_s + "]";
    AddPair(s,"intp_pos",intp_pos_s);

    string intp_tar_pos_s = "[ ";
    for(int axis = 0; axis < kMaxAxisChn; axis++){
        intp_tar_pos_s = intp_tar_pos_s + to_string(mc_status->intp_tar_pos.GetAxisValue(axis)) + " ";
    }
    intp_tar_pos_s = intp_tar_pos_s + "]";
    AddPair(s,"intp_tar_pos_s",intp_tar_pos_s);

    AddPair(s,"cur_line_no",mc_status->cur_line_no);
    AddPair(s,"cur_feed",mc_status->cur_feed);
    AddPair(s,"rated_feed",mc_status->rated_feed);
    AddPair(s,"axis_over_mask",mc_status->axis_over_mask);

    AddPair(s,"mode_g17",mc_status->mc_mode.bits.mode_g17);
    AddPair(s,"mode_g90",mc_status->mc_mode.bits.mode_g90);
    AddPair(s,"mode_g94",mc_status->mc_mode.bits.mode_g94);
    AddPair(s,"mode_g20",mc_status->mc_mode.bits.mode_g20);
    AddPair(s,"mode_g40",mc_status->mc_mode.bits.mode_g40);
    AddPair(s,"mode_g73",mc_status->mc_mode.bits.mode_g73);
    AddPair(s,"mode_g98",mc_status->mc_mode.bits.mode_g98);
    AddPair(s,"mode_g50",mc_status->mc_mode.bits.mode_g50);
    AddPair(s,"mode_g60",mc_status->mc_mode.bits.mode_g60);
    AddPair(s,"mode_g68",mc_status->mc_mode.bits.mode_g68);
    AddPair(s,"mode_d",mc_status->mc_mode.bits.mode_d);

    AddPair(s,"err_soft_limit_neg",mc_status->mc_error.bits.err_soft_limit_neg);
    AddPair(s,"err_soft_limit_pos",mc_status->mc_error.bits.err_soft_limit_pos);
    AddPair(s,"err_pos_over",mc_status->mc_error.bits.err_pos_over);
    AddPair(s,"err_arc_data",mc_status->mc_error.bits.err_arc_data);
    AddPair(s,"err_cmd_crc",mc_status->mc_error.bits.err_cmd_crc);
    AddPair(s,"err_data_crc",mc_status->mc_error.bits.err_data_crc);

    AddPair(s,"run_over",mc_status->run_over);
    AddPair(s,"buf_data_count",mc_status->buf_data_count);
    AddPair(s,"mda_data_count",mc_status->mda_data_count);
    if(mc_status->cur_cmd == 0)
        AddPair(s,"cur_cmd","G00");
    else if(mc_status->cur_cmd == 1)
        AddPair(s,"cur_cmd","G01");
    else if(mc_status->cur_cmd == 2)
        AddPair(s,"cur_cmd","G02");
    else if(mc_status->cur_cmd == 3)
        AddPair(s,"cur_cmd","G03");

    if(mc_status->cur_mode == 0)
        AddPair(s,"cur_mode","JOG");
    else if(mc_status->cur_mode == 1)
        AddPair(s,"cur_mode","AUTO");
    else if(mc_status->cur_mode == 2)
        AddPair(s,"cur_mode","MDA");

    AddPair(s,"axis_soft_negative_limit",mc_status->axis_soft_negative_limit);
    AddPair(s,"axis_soft_postive_limit",mc_status->axis_soft_postive_limit);
    AddPair(s,"pos_error_mask",mc_status->pos_error_mask);
    AddPair(s,"step_over",mc_status->step_over);
    AddPair(s,"auto_block_over",mc_status->auto_block_over);
    AddPair(s,"mda_block_over",mc_status->mda_block_over);

    SendMsg(s);
}


void ShowSc::PrintRealtimeData()
{
    if(!chn_rt_status)
        return;

    string s = "";
    string cur_pos_machine_s = "[ ";
    for(int axis = 0; axis < kMaxAxisChn; axis++){
        cur_pos_machine_s = cur_pos_machine_s +
                to_string(chn_rt_status->cur_pos_machine.GetAxisValue(axis)) + " ";
    }
    cur_pos_machine_s = cur_pos_machine_s + "]";
    AddPair(s,"cur_pos_machine",cur_pos_machine_s);

    string cur_pos_work_s = "[ ";
    for(int axis = 0; axis < kMaxAxisChn; axis++){
        cur_pos_work_s = cur_pos_work_s +
                to_string(chn_rt_status->cur_pos_work.GetAxisValue(axis)) + " ";
    }
    cur_pos_work_s = cur_pos_work_s + "]";
    AddPair(s,"cur_pos_work",cur_pos_work_s);

    string tar_pos_work_s = "[ ";
    for(int axis = 0; axis < kMaxAxisChn; axis++){
        tar_pos_work_s = tar_pos_work_s +
                to_string(chn_rt_status->tar_pos_work.GetAxisValue(axis)) + " ";
    }

    string cur_feedbck_velocity_s = "[ ";
    for(int axis = 0; axis < kMaxAxisChn; axis++){
        cur_feedbck_velocity_s = cur_feedbck_velocity_s +
                to_string(chn_rt_status->cur_feedbck_velocity.GetAxisValue(axis)) + " ";
    }
    cur_feedbck_velocity_s = cur_feedbck_velocity_s + "]";
    AddPair(s,"cur_feedbck_velocity",cur_feedbck_velocity_s);

    string cur_feedbck_torque_s = "[ ";
    for(int axis = 0; axis < kMaxAxisChn; axis++){
        cur_feedbck_torque_s = cur_feedbck_torque_s +
                to_string(chn_rt_status->cur_feedbck_torque.GetAxisValue(axis)) + " ";
    }
    cur_feedbck_torque_s = cur_feedbck_torque_s + "]";
    AddPair(s,"cur_feedbck_torque",cur_feedbck_torque_s);

    AddPair(s,"spindle_cur_speed",chn_rt_status->spindle_cur_speed);
    AddPair(s,"cur_feed",chn_rt_status->cur_feed);
    AddPair(s,"machining_time",chn_rt_status->machining_time);
    AddPair(s,"machining_time_remains",chn_rt_status->machining_time_remains);
    AddPair(s,"machining_time_total",chn_rt_status->machining_time_total);
    AddPair(s,"line_no",chn_rt_status->line_no);
    AddPair(s,"tap_err",chn_rt_status->tap_err);
    AddPair(s,"tap_err_now",chn_rt_status->tap_err_now);
    AddPair(s,"spd_angle",chn_rt_status->spd_angle);

    AddPair(s,"msg_new_count",msg_new_count);
    AddPair(s,"msg_delete_count",msg_delete_count);

    SendMsg(s);
}

void ShowSc::PrintSpindle()
{
    if(!spindle)
        return;

    string s = "";

    AddPair(s,"spindle_axis",spindle->phy_axis);
    AddPair(s,"z_axis",spindle->z_axis);
    AddPair(s,"da_prec",spindle->da_prec);
    AddPair(s,"to_level",spindle->to_level);
    AddPair(s,"level",spindle->level);
    AddPair(s,"cnc_polar",spindle->cnc_polar);
    AddPair(s,"cnc_speed",spindle->cnc_speed);
    if(spindle->mode == Spindle::Speed)
        AddPair(s,"ctrl mode","Speed");
    else
        AddPair(s,"ctrl mode","Position");

    AddPair(s,"tap_enable",spindle->tap_enable);
    AddPair(s,"motor_enable",spindle->motor_enable);
    AddPair(s,"_SSTP",spindle->_SSTP);
    AddPair(s,"SOR",spindle->SOR);
    AddPair(s,"SAR",spindle->SAR);
    AddPair(s,"SOV",spindle->SOV);
    AddPair(s,"RI",spindle->RI);
    AddPair(s,"SGN",spindle->SGN);
    AddPair(s,"SSIN",spindle->SSIN);
    AddPair(s,"SIND",spindle->SIND);
    AddPair(s,"ORCMA",spindle->ORCMA);
    AddPair(s,"RGTAP",spindle->RGTAP);
    AddPair(s,"tap_feed",spindle->tap_feed);

    SCAxisConfig *cfg = &axis_config[spindle->phy_axis];
    AddPair(s,"move_pr",cfg->move_pr);
    AddPair(s,"zero_compensation",cfg->zero_compensation);
    AddPair(s,"spd_gear_ratio",cfg->spd_gear_ratio);
    AddPair(s,"spd_max_speed",cfg->spd_max_speed);
    AddPair(s,"spd_min_speed",cfg->spd_min_speed);
    AddPair(s,"spd_start_time",cfg->spd_start_time);
    AddPair(s,"spd_stop_time",cfg->spd_stop_time);
    AddPair(s,"spd_vctrl_mode",cfg->spd_vctrl_mode);
    AddPair(s,"spd_speed_check",cfg->spd_speed_check);
    AddPair(s,"spd_speed_match",cfg->spd_speed_match);
    AddPair(s,"spd_speed_diff_limit",cfg->spd_speed_diff_limit);
    AddPair(s,"spd_encoder_location",cfg->spd_encoder_location);
    AddPair(s,"spd_ctrl_GST",cfg->spd_ctrl_GST);
    AddPair(s,"spd_ctrl_SGB",cfg->spd_ctrl_SGB);
    AddPair(s,"spd_ctrl_SFA",cfg->spd_ctrl_SFA);
    AddPair(s,"spd_ctrl_ORM",cfg->spd_ctrl_ORM);
    AddPair(s,"spd_ctrl_TCW",cfg->spd_ctrl_TCW);
    AddPair(s,"spd_ctrl_CWM",cfg->spd_ctrl_CWM);
    AddPair(s,"spd_ctrl_TSO",cfg->spd_ctrl_TSO);

    AddPair(s,"spd_analog_gain",cfg->spd_analog_gain);
    AddPair(s,"spd_sor_speed",cfg->spd_sor_speed);
    AddPair(s,"spd_motor_min_speed",cfg->spd_motor_min_speed);
    AddPair(s,"spd_motor_max_speed",cfg->spd_motor_max_speed);
    AddPair(s,"spd_gear_speed_low",cfg->spd_gear_speed_low);
    AddPair(s,"spd_gear_speed_middle",cfg->spd_gear_speed_middle);
    AddPair(s,"spd_gear_speed_high",cfg->spd_gear_speed_high);
    AddPair(s,"spd_gear_switch_speed1",cfg->spd_gear_switch_speed1);
    AddPair(s,"spd_gear_switch_speed2",cfg->spd_gear_switch_speed2);
    AddPair(s,"spd_sync_error_gain",cfg->spd_sync_error_gain);
    AddPair(s,"spd_speed_feed_gain",cfg->spd_speed_feed_gain);
    AddPair(s,"spd_pos_ratio_gain",cfg->spd_pos_ratio_gain);

    AddPair(s,"spd_rtnt_rate_on",cfg->spd_rtnt_rate_on);
    AddPair(s,"spd_rtnt_rate",cfg->spd_rtnt_rate);
    AddPair(s,"spd_rtnt_distance",cfg->spd_rtnt_distance);
    AddPair(s,"spd_locate_ang",cfg->spd_locate_ang);

    AddPair(s,"tap_flag",spindle->tap_state.tap_flag);
    AddPair(s,"tap_f",spindle->tap_state.F);
    AddPair(s,"tap_R",spindle->tap_state.R);
    AddPair(s,"tap_S",spindle->tap_state.S);
    SendMsg(s);
}

void ShowSc::PrintSysConfig()
{
    if(!sys_config)
        return;

    string s = "";

    SCSystemConfig *cfg = sys_config;
    AddPair(s,"cnc_mode",cfg->cnc_mode);
    AddPair(s,"max_chn_count",cfg->max_chn_count);
    AddPair(s,"max_axis_count",cfg->max_axis_count);
    AddPair(s,"chn_count",cfg->chn_count);
    AddPair(s,"axis_count",cfg->axis_count);
    AddPair(s,"axis_name_ex",cfg->axis_name_ex);
    AddPair(s,"fix_ratio_find_ref",cfg->fix_ratio_find_ref);
    AddPair(s,"hw_code_type",cfg->hw_code_type);
    AddPair(s,"io_filter_time",cfg->io_filter_time);
    AddPair(s,"fast_io_filter_time",cfg->fast_io_filter_time);
    AddPair(s,"free_space_limit",cfg->free_space_limit);
    AddPair(s,"backlight_delay_time",cfg->backlight_delay_time);
    AddPair(s,"bus_cycle",cfg->bus_cycle);
    AddPair(s,"save_lineno_poweroff",cfg->save_lineno_poweroff);
    AddPair(s,"manual_ret_ref_mode",cfg->manual_ret_ref_mode);
    AddPair(s,"beep_time",cfg->beep_time);
    AddPair(s,"da_ocp",cfg->da_ocp);
    AddPair(s,"da_prec",cfg->da_prec);
    AddPair(s,"alarm_temperature",cfg->alarm_temperature);
    AddPair(s,"trace_level",cfg->trace_level);
    AddPair(s,"debug_mode",cfg->debug_mode);
    AddPair(s,"hw_rev_trace",cfg->hw_rev_trace);

    SendMsg(s);
}

void ShowSc::PrintChnConfig()
{
    if(!chn_config)
        return;

    string s = "";

    SCChannelConfig *cfg = chn_config;
    AddPair(s,"chn_index",cfg->chn_index);
    AddPair(s,"chn_axis_count",cfg->chn_axis_count);
    AddPair(s,"chn_group_index",cfg->chn_group_index);

    string chn_axis_phy_s = "[ ";
    for(int axis = 0; axis < kMaxAxisChn; axis++)
        chn_axis_phy_s = chn_axis_phy_s + to_string(cfg->chn_axis_phy[axis]) + " ";
    chn_axis_phy_s = chn_axis_phy_s + "]";
    AddPair(s,"chn_axis_phy",chn_axis_phy_s);

    string chn_axis_name_s = "[ ";
    for(int axis = 0; axis < kMaxAxisChn; axis++)
        chn_axis_name_s = chn_axis_name_s + to_string(cfg->chn_axis_name[axis]) + " ";
    chn_axis_name_s = chn_axis_name_s + "]";
    AddPair(s,"chn_axis_name",chn_axis_name_s);

    string chn_axis_name_ex_s = "[ ";
    for(int axis = 0; axis < kMaxAxisChn; axis++)
        chn_axis_name_ex_s = chn_axis_name_ex_s + to_string(cfg->chn_axis_name_ex[axis]) + " ";
    chn_axis_name_ex_s = chn_axis_name_ex_s + "]";
    AddPair(s,"chn_axis_name_ex",chn_axis_name_ex_s);

    AddPair(s,"intep_mode",cfg->intep_mode);
    AddPair(s,"intep_cycle",cfg->intep_cycle);
    AddPair(s,"chn_precision",cfg->chn_precision);
    AddPair(s,"chn_look_ahead",cfg->chn_look_ahead);
    AddPair(s,"chn_feed_limit_level",cfg->chn_feed_limit_level);
    AddPair(s,"zmove_stop",cfg->zmove_stop);
    AddPair(s,"corner_stop_enable",cfg->corner_stop_enable);
    AddPair(s,"corner_stop",cfg->corner_stop);
    AddPair(s,"corner_stop_angle_min",cfg->corner_stop_angle_min);
    AddPair(s,"corner_acc_limit",cfg->corner_acc_limit);
    AddPair(s,"long_line_acc",cfg->long_line_acc);

    AddPair(s,"arc_err_limit",cfg->arc_err_limit);

    AddPair(s,"chn_spd_limit_on_axis",cfg->chn_spd_limit_on_axis);
    AddPair(s,"chn_spd_limit_on_acc",cfg->chn_spd_limit_on_acc);
    AddPair(s,"chn_spd_limit_on_curvity",cfg->chn_spd_limit_on_curvity);
    AddPair(s,"chn_rapid_overlap_level",cfg->chn_rapid_overlap_level);

    AddPair(s,"chn_max_vel",cfg->chn_max_vel);
    AddPair(s,"chn_max_acc",cfg->chn_max_acc);
    AddPair(s,"chn_max_dec",cfg->chn_max_dec);
    AddPair(s,"chn_max_corner_acc",cfg->chn_max_corner_acc);
    AddPair(s,"chn_max_arc_acc",cfg->chn_max_arc_acc);
    AddPair(s,"chn_s_cut_filter_time",cfg->chn_s_cut_filter_time);

    AddPair(s,"default_plane",cfg->default_plane);
    AddPair(s,"default_cmd_mode",cfg->default_cmd_mode);
    AddPair(s,"default_feed_mode",cfg->default_feed_mode);

    AddPair(s,"rapid_mode",cfg->rapid_mode);
    AddPair(s,"cut_plan_mode",cfg->cut_plan_mode);
    AddPair(s,"rapid_plan_mode",cfg->rapid_plan_mode);
    AddPair(s,"ex_coord_count",cfg->ex_coord_count);

    AddPair(s,"change_tool_mode",cfg->change_tool_mode);
    AddPair(s,"tool_live_check",cfg->tool_live_check);
    AddPair(s,"auto_tool_measure",cfg->auto_tool_measure);

    AddPair(s,"gcode_trace",cfg->gcode_trace);
    AddPair(s,"gcode_unit",cfg->gcode_unit);

    AddPair(s,"timing_mode",cfg->timing_mode);

    AddPair(s,"chn_small_line_time",cfg->chn_small_line_time);

    AddPair(s,"g31_skip_signal",cfg->g31_skip_signal);
    AddPair(s,"g31_sig_level",cfg->g31_sig_level);
    AddPair(s,"rst_hold_time",cfg->rst_hold_time);
    AddPair(s,"rst_mode",cfg->rst_mode);
    AddPair(s,"g01_max_speed",cfg->g01_max_speed);
    AddPair(s,"mpg_level3_step",cfg->mpg_level3_step);
    AddPair(s,"mpg_level4_step",cfg->mpg_level4_step);

    SendMsg(s);
}

void ShowSc::PrintAxisConfig(int axis)
{
    if(!axis_config)
        return;

    string s = "";

    SCAxisConfig *cfg = &axis_config[axis];
    AddPair(s,"axis_index",cfg->axis_index);
    AddPair(s,"axis_type",cfg->axis_type);
    AddPair(s,"axis_interface",cfg->axis_interface);
    AddPair(s,"axis_port",cfg->axis_port);
    AddPair(s,"axis_linear_type",cfg->axis_linear_type);
    AddPair(s,"axis_pmc",cfg->axis_pmc);
    AddPair(s,"pmc_g00_by_EIFg",cfg->pmc_g00_by_EIFg);
    AddPair(s,"pmc_min_speed",cfg->pmc_min_speed);
    AddPair(s,"pmc_max_speed",cfg->pmc_max_speed);

    AddPair(s,"kp1",cfg->kp1);
    AddPair(s,"kp2",cfg->kp2);
    AddPair(s,"ki",cfg->ki);
    AddPair(s,"kd",cfg->kd);
    AddPair(s,"kil",cfg->kil);
    AddPair(s,"kvff",cfg->kvff);
    AddPair(s,"kaff",cfg->kaff);

    AddPair(s,"track_err_limit",cfg->track_err_limit);
    AddPair(s,"location_err_limit",cfg->location_err_limit);

    AddPair(s,"save_pos_poweroff",cfg->save_pos_poweroff);

    AddPair(s,"motor_count_pr",cfg->motor_count_pr);
    AddPair(s,"motor_speed_max",cfg->motor_speed_max);
    AddPair(s,"move_pr",cfg->move_pr);
    AddPair(s,"motor_dir",cfg->motor_dir);
    AddPair(s,"feedback_mode",cfg->feedback_mode);

    AddPair(s,"ret_ref_mode",cfg->ret_ref_mode);
    AddPair(s,"ret_ref_dir",cfg->ret_ref_dir);
    AddPair(s,"ret_ref_change_dir",cfg->ret_ref_change_dir);
    AddPair(s,"ref_signal",cfg->ref_signal);
    AddPair(s,"ret_ref_index",cfg->ret_ref_index);
    AddPair(s,"ref_base_diff_check",cfg->ref_base_diff_check);
    AddPair(s,"ref_base_diff",cfg->ref_base_diff);
    AddPair(s,"ref_encoder",cfg->ref_encoder);

    AddPair(s,"manual_speed",cfg->manual_speed);
    AddPair(s,"rapid_speed",cfg->rapid_speed);
    AddPair(s,"reset_speed",cfg->reset_speed);
    AddPair(s,"ret_ref_speed",cfg->ret_ref_speed);
    AddPair(s,"ret_ref_speed_second", cfg->ret_ref_speed_second);
    AddPair(s,"ref_offset_pos", cfg->ref_offset_pos);
    AddPair(s,"ref_z_distance_max", cfg->ref_z_distance_max);

    AddPair(s,"rapid_acc",cfg->rapid_acc);
    AddPair(s,"manual_acc",cfg->manual_acc);
    AddPair(s,"start_acc",cfg->start_acc);

    AddPair(s,"corner_acc_limit",cfg->corner_acc_limit);

    AddPair(s,"rapid_s_plan_filter_time",cfg->rapid_s_plan_filter_time);

    AddPair(s,"post_filter_type",cfg->post_filter_type);
    AddPair(s,"post_filter_time_1",cfg->post_filter_time_1);
    AddPair(s,"post_filter_time_2",cfg->post_filter_time_2);

    AddPair(s,"backlash_enable",cfg->backlash_enable);
    AddPair(s,"backlash_forward",cfg->backlash_forward);
    AddPair(s,"backlash_negative",cfg->backlash_negative);

    AddPair(s,"pc_type",cfg->pc_type);
    AddPair(s,"pc_enable",cfg->pc_enable);
    AddPair(s,"pc_offset",cfg->pc_offset);
    AddPair(s,"pc_count",cfg->pc_count);
    AddPair(s,"pc_ref_index",cfg->pc_ref_index);
    AddPair(s,"pc_inter_dist",cfg->pc_inter_dist);

    AddPair(s,"soft_limit_max_1",cfg->soft_limit_max_1);
    AddPair(s,"soft_limit_min_1",cfg->soft_limit_min_1);
    AddPair(s,"soft_limit_check_1",cfg->soft_limit_check_1);
    AddPair(s,"soft_limit_max_2",cfg->soft_limit_max_2);
    AddPair(s,"soft_limit_min_2",cfg->soft_limit_min_2);
    AddPair(s,"soft_limit_check_2",cfg->soft_limit_check_2);
    AddPair(s,"soft_limit_max_3",cfg->soft_limit_max_3);
    AddPair(s,"soft_limit_min_3",cfg->soft_limit_min_3);
    AddPair(s,"soft_limit_check_3",cfg->soft_limit_check_3);

    AddPair(s,"off_line_check",cfg->off_line_check);

    AddPair(s,"ctrl_mode",cfg->ctrl_mode);
    AddPair(s,"pulse_count_pr",cfg->pulse_count_pr);
    AddPair(s,"encoder_lines",cfg->encoder_lines);
    AddPair(s,"encoder_max_cycle",cfg->encoder_max_cycle);

    AddPair(s,"fast_locate",cfg->fast_locate);
    AddPair(s,"pos_disp_mode",cfg->pos_disp_mode);

    AddPair(s,"sync_axis",cfg->sync_axis);
    AddPair(s,"series_ctrl_axis",cfg->series_ctrl_axis);
    AddPair(s,"master_axis_no",cfg->master_axis_no);
    AddPair(s,"disp_coord",cfg->disp_coord);
    AddPair(s,"auto_sync",cfg->auto_sync);
    AddPair(s,"sync_err_max_pos",cfg->sync_err_max_pos);
    AddPair(s,"benchmark_offset",cfg->benchmark_offset);
    AddPair(s,"sync_pre_load_torque",cfg->sync_pre_load_torque);
    AddPair(s,"sync_err_max_torque",cfg->sync_err_max_torque);
    AddPair(s,"sync_err_max_mach",cfg->sync_err_max_mach);
    AddPair(s,"sync_pos_detect",cfg->sync_pos_detect);
    AddPair(s,"sync_mach_detect",cfg->sync_mach_detect);
    AddPair(s,"sync_torque_detect",cfg->sync_torque_detect);
    AddPair(s,"serial_torque_ratio",cfg->serial_torque_ratio);
    AddPair(s,"serial_pre_speed",cfg->serial_pre_speed);

    for(int i = 0; i < 10; i++){
        string axis_home_pos_s = "axis_home_pos[" + to_string(i) + "]";
        AddPair(s,axis_home_pos_s,cfg->axis_home_pos[i]);
    }

    AddPair(s,"ref_mark_err",cfg->ref_mark_err);

    SendMsg(s);
}

void ShowSc::PrintCoordConfig()
{
    if(!coord)
        return;

    string s = "";
    for(int i = 53; i <= 59; i++){ // G53 - G59
        string coord_name_s = "G" + to_string(i);
        string value_s = "[";
        for(int axis = 0; axis < kMaxAxisChn; axis++){
            value_s = value_s + to_string(coord[i-53].offset[axis])+" ";
        }
        value_s = value_s + "]";

        AddPair(s,coord_name_s,value_s);
    }

    if(global_coord){
        string g93_value_s = "[";
        for(int axis = 0; axis < kMaxAxisChn; axis++){
            g93_value_s = g93_value_s + to_string(global_coord->offset[axis])+" ";
        }
        g93_value_s = g93_value_s + "]";
        AddPair(s,"G92",g93_value_s);
    }

    SendMsg(s);
}

void ShowSc::PrintExCoordConfig()
{
    if(!ex_coord)
        return;

    string s = "";
    for(int i = 5401; i <= 5410; i++){ // G53 - G59
        string coord_name_s = "G" + to_string(i);
        string value_s = "[";
        for(int axis = 0; axis < kMaxAxisChn; axis++){
            value_s = value_s + to_string(ex_coord[i-5401].offset[axis])+" ";
        }
        value_s = value_s + "]";

        AddPair(s,coord_name_s,value_s);
    }

    SendMsg(s);
}

void ShowSc::PrintGrbCoordConfig()
{

}

void ShowSc::PrintToolOffsetConfig()
{

}
void ShowSc::PrintToolPotConfig()
{

}

void ShowSc::PrintFiveAxisCoinfig()
{

}

void ShowSc::PrintMode()
{
    if(!chn_status)
        return;
    uint16_t *mode = chn_status->gmode;

    string s = "";

    for(int i = 0; i < 40; i++){
        string key = "gmode[" + to_string(i) + "]";
        string value = "G"+to_string((int)mode[i]/10);
        AddPair(s,key,value);
    }

    SendMsg(s);
}

void ShowSc::PrintWarning()
{
    AlarmProcessor *alarms = AlarmProcessor::GetInstance();
    if(!alarms)
        return;

    string s= "";

    vector<ErrorInfo> list = alarms->GetWarningList();
    //int count = list.BufLen();
    for(size_t i = 0; i < list.size(); i++){
        ErrorInfo info = list.at(i);
        string key = "error info[" + to_string(i) = "]";
        char value[128];
        sprintf(value,"chn=%hhu, axis=%hhu, err_code=%hu, err_info=0x%x\n",
                info.channel_index, info.axis_index, info.error_code, info.error_info);
        AddPair(s,key,value);
    }

    SendMsg(s);
}

void ShowSc::PrintFRegState()
{
    if(!F)
        return;

    uint8_t *F = (uint8_t*)this->F;
    string s = "";
    AddPair(s,"\t","#7\t#6\t#5\t#4\t#3\t#2\t#1\t#0\t");
    for(int i = 0; i < 256; i++){
        string reg_name_s = "F" + to_string(i) + "\t";
        string reg_value_s = "";
        for(int bit = 7; bit >= 0; bit--){
            reg_value_s = reg_value_s + to_string(bool(F[i] & (0x01 << bit)))+"\t";
        }
        AddPair(s,reg_name_s,reg_value_s);
    }

    SendMsg(s);
}

void ShowSc::PrintGRegState()
{
    if(!G)
        return;

    uint8_t *G = (uint8_t*)this->G;
    string s = "";
    AddPair(s,"\t","#7\t#6\t#5\t#4\t#3\t#2\t#1\t#0\t");
    for(int i = 0; i < 256; i++){
        string reg_name_s = "G" + to_string(i) + "\t";
        string reg_value_s = "";
        for(int bit = 7; bit >= 0; bit--){
            reg_value_s = reg_value_s + to_string(bool(G[i] & (0x01 << bit)))+"\t";
        }
        AddPair(s,reg_name_s,reg_value_s);
    }

    SendMsg(s);
}

void ShowSc::PrintPmcAxisCtrl()
{
    if(!pmc_axis_ctrl)
        return;

    string s = "";
    for(int i = 0; i < 4; i++){
        PmcAxisCtrl *cfg = &pmc_axis_ctrl[i];
        char title[20];
        sprintf(title,"[Group-%d]",i);
        AddPair(s,title,"");
        AddPair(s,"m_n_group_index",cfg->m_n_group_index);
        string axis_indexs = "[ ";
        for(unsigned int j = 0; j<cfg->axis_list.size(); j++){
            axis_indexs = axis_indexs + to_string(cfg->axis_list.at(j)->axis_index) + " ";
        }
        axis_indexs = axis_indexs + "]";
        AddPair(s,"axis_list",axis_indexs);
        AddPair(s,"m_b_active",cfg->m_b_active);
        AddPair(s,"m_b_buffer",cfg->m_b_buffer);
        AddPair(s,"m_b_step_stop",cfg->m_b_step_stop);
        AddPair(s,"m_b_pause",cfg->m_b_pause);
        AddPair(s,"m_n_cmd_count",cfg->m_n_cmd_count);
        AddPair(s,"m_n_buf_exec",cfg->m_n_buf_exec);
        AddPair(s,"cmd[0].cmd",cfg->m_pmc_cmd_buffer[0].cmd);
        AddPair(s,"cmd[0].speed",cfg->m_pmc_cmd_buffer[0].speed);
        AddPair(s,"cmd[0].distance",cfg->m_pmc_cmd_buffer[0].distance);
        AddPair(s,"cmd[1].cmd",cfg->m_pmc_cmd_buffer[1].cmd);
        AddPair(s,"cmd[1].speed",cfg->m_pmc_cmd_buffer[1].speed);
        AddPair(s,"cmd[1].distance",cfg->m_pmc_cmd_buffer[1].distance);
        AddPair(s,"cmd[2].cmd",cfg->m_pmc_cmd_buffer[2].cmd);
        AddPair(s,"cmd[2].speed",cfg->m_pmc_cmd_buffer[2].speed);
        AddPair(s,"cmd[2].distance",cfg->m_pmc_cmd_buffer[2].distance);
        s.append("\n");
    }
    SendMsg(s);
}

void ShowSc::PrintSyncAxisCtrl()
{
    if(!sync_axis_ctrl)
        return;
    string s = "";
    string sync_mask = "[ ";
    for(int i = 0; i < chn_config->chn_axis_count; i++){
        sync_mask = sync_mask + to_string(bool(sync_axis_ctrl->sync_mask & (0x01 << i))) + " ";
    }
    sync_mask = sync_mask + "]";
    AddPair(s,"sync_mask",sync_mask);
    string sync_en = "[ ";
    for(int i = 0; i < chn_config->chn_axis_count; i++){
        sync_en = sync_en + to_string(bool(sync_axis_ctrl->sync_en & (0x01 << i))) + " ";
    }
    sync_en = sync_en + "]";
    AddPair(s,"sync_en",sync_en);

    string SYNC = "[ ";
    for(int i = 0; i < chn_config->chn_axis_count; i++){
        SYNC = SYNC + to_string(bool(sync_axis_ctrl->SYNC & (0x01 << i))) + " ";
    }
    SYNC = SYNC + "]";
    AddPair(s,"SYNC",SYNC);

    string SYNCJ = "[ ";
    for(int i = 0; i < chn_config->chn_axis_count; i++){
        SYNCJ = SYNCJ + to_string(bool(sync_axis_ctrl->SYNCJ & (0x01 << i))) + " ";
    }
    SYNCJ = SYNCJ + "]";
    AddPair(s,"SYNCJ",SYNCJ);
    if(sync_axis_ctrl->mode == AUTO_MODE){
        AddPair(s,"mode",sync_axis_ctrl->mode);
    }
    if(sync_axis_ctrl->mode == AUTO_MODE)
        AddPair(s,"work mode","AUTO_MODE");
    else if(sync_axis_ctrl->mode == MDA_MODE)
        AddPair(s,"work mode","MDA_MODE");
    else if(sync_axis_ctrl->mode == MANUAL_STEP_MODE)
        AddPair(s,"work mode","MANUAL_STEP_MODE");
    else if(sync_axis_ctrl->mode == MANUAL_MODE)
        AddPair(s,"work mode","MANUAL_MODE");
    else if(sync_axis_ctrl->mode == MPG_MODE)
        AddPair(s,"work mode","MPG_MODE");
    else if(sync_axis_ctrl->mode == EDIT_MODE)
        AddPair(s,"work mode","EDIT_MODE");
    else if(sync_axis_ctrl->mode == REF_MODE)
        AddPair(s,"work mode","REF_MODE");
    else
        AddPair(s,"work mode","INVALID_MODE");
    AddPair(s,"wait_en_index",sync_axis_ctrl->wait_en_index);

    SendMsg(s);
}

void ShowSc::SendMsg(string &s)
{
    trace->SendMsg(PrintTopic,s);
}
