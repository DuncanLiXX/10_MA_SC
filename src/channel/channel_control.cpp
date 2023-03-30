/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file ChannelControl.cpp
 *@author gonghao
 *@date 2020/03/19
 *@brief 本文件为通道控制类的实现
 *通道控制类的主要作用是管理通道状态，控制通道处理流程
 *@version
 */

#include <channel_control.h>
#include "hmi_communication.h"
#include "mi_communication.h"
#include "mc_communication.h"
#include "mc_communication_arm.h"								 
#include "parm_manager.h"
#include "channel_engine.h"
#include "alarm_processor.h"
#include "spindle_control.h"
#include "showsc.h"
#include "axis_status_ctrl.h"

using namespace Spindle;

int ctrlmode_switch_wait = 0;   //用于轴模式切换等待延时

/**
 * @brief 构造函数
 */
ChannelControl::ChannelControl() {
    // TODO Auto-generated constructor stub
    m_p_compiler = nullptr;
    m_n_channel_index = 0;   //默认通道索引号为0

    m_b_lineno_from_mc = false;   //默认从MC模块获取当前运行代码的行号
    m_b_mc_need_start = true;
    //	m_b_step_exec = false;
    m_n_step_change_mode_count = 0;
    m_error_code = ERR_NONE;

    m_thread_breakcontinue = 0;
    m_thread_refresh_status = 0;
    m_thread_compiler = 0;

    m_n_subprog_count = 0;
    m_n_macroprog_count = 0;
    m_b_ret_from_macroprog = false;

    m_n_mask_clear_pos = 0;

    m_mask_run_pmc = 0;
    m_mask_runover_pmc = 0;

    m_n_mc_auto_buf_max = 200;

    this->m_n_cur_proc_group = 0;

    m_b_cancel_breakcontinue_thread = false;

    m_p_pmc_reg = nullptr;
    m_p_f_reg = nullptr;
    m_p_f_reg = nullptr;

    this->m_p_last_output_msg = nullptr;

    m_b_manual_tool_measure = false;
    m_b_cancel_manual_tool_measure = false;

    this->m_b_manual_call_macro = false;
    this->m_b_cancel_manual_call_macro = false;

    this->m_b_delay_to_reset = false;

    m_b_pos_captured = false;

    m_b_hmi_graph = false;     //默认非图形模式

    memset(&m_channel_status, 0x00, sizeof(ChannelStatusCollect));
    //	memset(m_str_cur_nc_file, 0x00, kMaxPathLen);
    //
    //	strcpy(m_str_cur_nc_file, "3.nc");  //for test

    this->InitMcModeStatus();
    m_channel_rt_status.cur_pos_machine = 0.0;

    m_channel_rt_status.cur_feedbck_velocity = 0.0;
    m_channel_rt_status.cur_feedbck_torque= 0.0;
#ifdef USES_SPEED_TORQUE_CTRL	
    m_n_need_reset_axis_ctrlmode = 0;
#endif

#ifdef USES_ADDITIONAL_PROGRAM
    m_n_add_prog_type = NONE_ADD;        //当前附加程序执行状态
#endif

    m_channel_rt_status.cur_pos_work = 0.0;
    m_channel_rt_status.tar_pos_work = 0.0;
    //	memset(m_channel_rt_status.path_remains, 0x00, sizeof(m_channel_rt_status.path_remains));
    m_channel_rt_status.spindle_cur_speed = 0;
    m_channel_rt_status.cur_feed = 0;
    m_channel_rt_status.machining_time = 0;
    m_channel_rt_status.machining_time_remains = 0;
    m_channel_rt_status.machining_time_total = 0;
    m_channel_rt_status.line_no = 1;

    m_time_remain = 0;

    m_n_hw_trace_state = NONE_TRACE;
    this->m_n_hw_trace_state_change_to = NONE_TRACE;
    this->m_b_change_hw_trace_state = false;

    this->m_n_restart_mode = NOT_RESTART;
    this->m_n_restart_line = 0;
    this->m_n_restart_step = 0;

    //this->m_n_M29_flag = 0;
    //this->m_n_G843_flag = 0;
    //this->m_n_G843_ratio = 0;

    gettimeofday(&m_time_start_mc, nullptr);  //初始化MC启动时间

#ifdef USES_WUXI_BLOOD_CHECK
    this->m_b_returnning_home = false;
    this->m_n_ret_home_step = 0;
#endif

#ifdef USES_GRIND_MACHINE
    this->m_b_ret_safe_state = false;
    this->m_n_ret_safe_step = 0;
    this->m_p_mech_arm_param = nullptr;
    this->m_p_mech_arm_state = nullptr;
#endif

#ifdef USES_LASER_MACHINE
    this->m_b_laser_calibration = false;
    this->m_b_cancel_calib = false;
    this->m_n_calib_step = 0;
#endif

#ifdef USES_WOOD_MACHINE
    this->m_n_ref_tool_life_delay = 0;
    m_b_save_tool_info = false;

    m_b_prestart_spd = false;
    this->m_n_spd_prestart_step = 0;
#endif

#ifdef USES_SIMULATION_TEST
    m_file_sim_data = -1;      //仿真数据保存文件句柄
#endif
}

/**
 * @brief 析构函数
 */
ChannelControl::~ChannelControl() {
    // TODO Auto-generated destructor stub

    void* thread_result;
    int res = ERR_NONE;

    //退出编译运行线程
    res = pthread_cancel(m_thread_compiler);
    if (res != ERR_NONE) {
        printf("compile thread cancel failed\n");
    }

    //	usleep(1000);
    res = pthread_join(m_thread_compiler, &thread_result);//等待编译器线程退出完成
    if (res != ERR_NONE) {
        printf("compile thread join failed\n");
    }
    m_thread_compiler = 0;

    //退出状态刷新线程
    res = pthread_cancel(m_thread_refresh_status);
    if (res != ERR_NONE) {
        printf("status refresh thread cancel failed\n");
    }

    //	usleep(1000);
    res = pthread_join(m_thread_refresh_status, &thread_result);//等待编译器线程退出完成
    if (res != ERR_NONE) {
        printf("status refresh thread join failed\n");
    }
    m_thread_refresh_status = 0;

    if(m_p_output_msg_list_auto != nullptr){//销毁AUTO指令消息输出缓冲
        delete m_p_output_msg_list_auto;
        m_p_output_msg_list_auto = nullptr;
    }

    if(m_p_output_msg_list_mda != nullptr){//销毁MDA指令消息输出缓冲
        delete m_p_output_msg_list_mda;
        m_p_output_msg_list_mda = nullptr;
    }

    //释放编译器对象
    if(m_p_compiler != nullptr){
        delete m_p_compiler;
        m_p_compiler = nullptr;
    }

    //释放打印线程
    delete  &Singleton<ShowSc>::instance();

    pthread_mutex_destroy(&m_mutex_change_state);

}

/**
 * @brief 将G代码模态初始化到默认状态
 */
void ChannelControl::InitGCodeMode(){
    m_channel_status.gmode[1] = G00_CMD;  //01组默认G00
    m_channel_status.gmode[2] = G17_CMD;  //02组默认G17
    m_channel_status.gmode[3] = G90_CMD;  //03组默认G90
    //	m_channel_status.gmode[4] = G23_CMD;
    m_channel_status.gmode[5] = G94_CMD;  //05组默认G94  每分钟进给
    m_channel_status.gmode[6] = G21_CMD;  //06组默认G21  公制单位

    m_channel_status.gmode[7] = G40_CMD;  //07组默认G40
    m_channel_status.gmode[8] = G49_CMD;  //08组默认G49
    m_channel_status.gmode[9] = G80_CMD;  //09组默认G80
    m_channel_status.gmode[10] = G98_CMD;  //10组默认G98  返回初始平面
    m_channel_status.gmode[11] = G50_CMD;  //11组默认G50  比例缩放取消
    m_channel_status.gmode[14] = G54_CMD;  //14组默认G54  G54工件坐标系
    m_channel_status.gmode[15] = G64_CMD;  //15组默认G64  切削方式
    m_channel_status.gmode[16] = G69_CMD;  //16组默认G69  坐标旋转或者三维坐标变换方式OFF

    m_channel_status.gmode[17] = G15_CMD;  //17组默认15   极坐标指令取消
    m_channel_status.gmode[19] = G26_CMD;  //19组默认G26  主轴速度变动检测ON
}


/**
 * @brief 初始化函数
 * @param chn_index : 通道索引号
 * @param engine : 通道引擎指针
 * @param hmi_comm ：HMI通讯接口指针
 * @param parm ：参数访问接口指针
 */
bool ChannelControl::Initialize(uint8_t chn_index, ChannelEngine *engine, HMICommunication *hmi_comm,
                                MICommunication *mi_comm, MCCommunication *mc_comm, ParmManager *parm, PmcRegister *reg){
    printf("Enter ChannelControl::Initialize\n");

    pthread_attr_t attr;
    struct sched_param param;
    int res = 0;

    char path[kMaxPathLen] = {0};

    m_n_channel_index = chn_index;
    m_p_channel_engine = engine;
    m_p_hmi_comm = hmi_comm;
    m_p_mi_comm = mi_comm;
    m_p_mc_comm = mc_comm;
    m_p_spindle = new SpindleControl;
    m_p_pmc_reg = reg;
    m_p_f_reg = &reg->FReg().bits[m_n_channel_index];
    m_p_g_reg = &reg->GReg().bits[m_n_channel_index];
    m_p_general_config = parm->GetSystemConfig();
    m_p_channel_config = parm->GetChannelConfig(chn_index);
    m_p_axis_config = parm->GetAxisConfig();
    this->m_p_chn_coord_config = parm->GetCoordConfig(chn_index);
    this->m_p_chn_ex_coord_config = parm->GetExCoordConfig(chn_index);
    this->m_p_chn_g92_offset = new SCCoordConfig();
    memset(m_p_chn_g92_offset, 0, sizeof(SCCoordConfig));
    this->m_p_chn_tool_config = parm->GetToolConfig(chn_index);
    this->m_p_chn_tool_info = parm->GetToolPotConfig(chn_index);

    this->m_p_chn_proc_param = parm->GetChnProcessParam(chn_index);
    this->m_p_axis_proc_param = parm->GetAxisProcParam();

#ifdef USES_FIVE_AXIS_FUNC
    this->m_p_chn_5axis_config = parm->GetFiveAxisConfig(chn_index);
    this->UpdateFiveAxisRotParam();
#endif

    m_n_cur_tcode = -1;
    //	m_n_cur_scode = -1;
    //	m_n_cur_dcode = -1;
    //	m_n_cur_hcode = -1;

    m_n_real_phy_axis = 0;

    this->m_mask_intp_axis = 0;
    this->m_n_intp_axis_count = 0;

    m_b_need_change_to_pause = false;

    this->m_n_spindle_count = 0;
    memset(m_spd_axis_phy, 0x00, kMaxChnSpdCount);

    //初始化回参考点标志
    m_channel_status.returned_to_ref_point = 0x00;
    uint8_t phy_axis = 0;
    for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        phy_axis = m_p_channel_config->chn_axis_phy[i];
        if(phy_axis == 0){//未配置
            m_channel_status.returned_to_ref_point |= (0x01<<i);
            continue;
        }
        if(m_p_axis_config[phy_axis-1].axis_interface == VIRTUAL_AXIS   //虚拟轴不用建立参考点
            || m_p_axis_config[phy_axis-1].axis_type == AXIS_SPINDLE	//主轴不用建立参数点
            //|| (m_p_axis_config[phy_axis-1].feedback_mode == ABSOLUTE_ENCODER && m_p_axis_config[phy_axis-1].ref_encoder != kAxisRefNoDef))  //已经建立参考点的绝对式编码器，不用再次建立参考点
            || (m_p_axis_config[phy_axis-1].feedback_mode == ABSOLUTE_ENCODER && m_p_axis_config[phy_axis-1].ref_complete != 0))  //已经建立参考点的绝对式编码器，不用再次建立参考点
        {
            m_channel_status.returned_to_ref_point |= (0x01<<i);
        }
        if(m_p_axis_config[phy_axis-1].axis_interface != VIRTUAL_AXIS)
        {
            m_n_real_phy_axis = m_n_real_phy_axis | 0x01<<(phy_axis-1);    //初始化实际物理轴mask
        }

        uint8_t z_axis =  this->GetPhyAxisFromName(AXIS_NAME_Z);
        uint32_t da_prec = m_p_channel_engine->GetDaPrecision();

        //初始化主轴信息
        if(m_p_axis_config[phy_axis-1].axis_type == AXIS_SPINDLE && m_n_spindle_count < kMaxChnSpdCount){
            this->m_spd_axis_phy[m_n_spindle_count] = phy_axis;
            m_n_spindle_count++;
            m_p_spindle->SetSpindleParams(&m_p_axis_config[phy_axis-1],
                    da_prec,phy_axis-1,z_axis);
            m_p_spindle->SetComponent(m_p_mi_comm,
                                      m_p_mc_comm,
                                      m_p_f_reg,
                                      m_p_g_reg,
                                      &m_macro_variable);
            if(m_n_spindle_count > 1){
                CreateError(ERR_SPD_MULTI_NUM, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
            }
        }

        //PMC轴不在这里激活，通过梯图来激活
        this->m_mask_intp_axis |= (0x01<<i);
        this->m_n_intp_axis_count++;

#ifdef USES_SPEED_TORQUE_CTRL
        // 轴控制模式，初始化配置到当前状态
        m_channel_status.cur_axis_ctrl_mode[i] = m_p_axis_config[phy_axis-1].axis_type;
#endif
        m_channel_status.cur_chn_axis_phy[i]   = m_p_channel_config->chn_axis_phy[i];
    }

    ShowSc &showSc = Singleton<ShowSc>::instance();
    showSc.AddComponent(&m_channel_status);
    showSc.AddComponent(&m_channel_rt_status);
    showSc.AddComponent(&m_channel_mc_status);
    showSc.AddComponent(m_p_spindle);
    showSc.AddComponent(m_p_general_config);
    showSc.AddComponent(m_p_channel_config);
    showSc.AddComponent(m_p_axis_config);
    showSc.AddComponent(m_p_chn_coord_config,
                        m_p_chn_ex_coord_config,
                        m_p_chn_g92_offset);
    showSc.AddComponent(m_p_chn_tool_config);
    showSc.AddComponent(m_p_chn_tool_info);
    showSc.AddComponent(m_p_chn_5axis_config);
    showSc.AddComponent(m_p_f_reg);
    showSc.AddComponent(m_p_g_reg);
    showSc.AddComponent(m_p_channel_engine->GetPmcAxisCtrl());
    showSc.AddComponent(m_p_channel_engine->GetSyncAxisCtrl());

    this->m_macro_variable.SetChnIndex(m_n_channel_index);


    memset(this->m_str_mda_path, 0x00, kMaxPathLen);
    this->GetMdaFilePath(m_str_mda_path);   //初始化mda文件路径

    m_channel_rt_status.machining_time_total = g_ptr_parm_manager->GetCurTotalMachingTime(chn_index);

#ifdef USES_GRIND_MACHINE
    m_p_grind_config = parm->GetGrindConfig();       //磨床参数
#endif

    m_n_run_thread_state = IDLE;
    m_thread_compiler = 0;
    //	m_p_output_buffer = nullptr;
    m_p_output_msg_list = nullptr;
    m_p_output_msg_list_auto = nullptr;
    m_p_output_msg_list_mda = nullptr;

    m_b_init_compiler_pos = false;

    m_change_work_mode_to = INVALID_MODE;

    m_n_send_mc_data_err = 0;  //for test

    //	m_b_quit = false;
    m_n_frame_index = 1;    //当前运动数据帧号默认为0
#ifdef USES_SIMULATION_TEST
    m_simulate_mode = SIM_OUTLINE;    //默认在轮廓仿真模式
#else
    m_simulate_mode = SIM_NONE;    //默认在非仿真模式
#endif

    this->m_b_mc_on_arm = this->m_p_channel_engine->IsMcArmChn(this->m_n_channel_index);

    m_scene_auto.need_reload_flag = false;
    m_scene_auto.machining_state = MS_READY;   //初始化为默认为就绪

    this->InitialChannelStatus();

    //初始化XYZ轴对应的物理轴
    m_n_xyz_axis_phy[0] = this->GetPhyAxisFromName(AXIS_NAME_X);
    m_n_xyz_axis_phy[1] = this->GetPhyAxisFromName(AXIS_NAME_Y);
    m_n_xyz_axis_phy[2] = this->GetPhyAxisFromName(AXIS_NAME_Z);

    m_n_graph_pos_count = 0;
    m_n_graph_pos_write_idx = 0;
    m_n_graph_pos_read_idx = 0;


    //创建自动模式输出指令消息列表
    this->m_p_output_msg_list_auto = new OutputMsgList(kMaxOutputBufCount);
    if(this->m_p_output_msg_list_auto == nullptr){
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建自动模式输出指令消息队列失败!", m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //初始化失败
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER, 0, m_n_channel_index);
        m_n_run_thread_state = ERROR;
        return false;
    }

    //创建MDA模式输出指令消息列表
    this->m_p_output_msg_list_mda = new OutputMsgList();
    if(this->m_p_output_msg_list_mda == nullptr){
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建MDA模式输出指令消息队列失败!", m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //初始化失败
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER, 0, m_n_channel_index);
        m_n_run_thread_state = ERROR;
        return false;
    }

    m_p_output_msg_list = m_p_output_msg_list_auto;   //默认指向自动输出队列

    pthread_mutex_init(&m_mutex_change_state, nullptr);

    //创建编译器对象
    m_p_compiler = new Compiler(this);
    if(m_p_compiler == nullptr){
        m_error_code = ERR_MEMORY_NEW;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER, 0, m_n_channel_index);
        return false;
    }
    m_p_compiler->SetOutputMsgList(m_p_output_msg_list);
    m_p_compiler->SetAxisNameEx(g_ptr_parm_manager->GetSystemConfig()->axis_name_ex);

    //创建编译线程
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//
    param.__sched_priority = 35; //95;
    pthread_attr_setschedparam(&attr, &param);
    //	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
    //	if (res) {
    //		printf("pthread setinheritsched failed\n");
    //	}

    res = pthread_create(&m_thread_compiler, &attr,
                         ChannelControl::CompileThread, this);    //开启G代码编译运行线程
    if (res != 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译运行线程创建失败!", m_n_channel_index);
        m_error_code = ERR_SC_INIT;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER, 0, m_n_channel_index);
        goto END;
    }

    //开启状态刷新线程
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//
    param.__sched_priority = 36; //96;
    pthread_attr_setschedparam(&attr, &param);
    res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
    if (res) {
        printf("pthread setinheritsched failed\n");
    }

    res = pthread_create(&m_thread_refresh_status, &attr,
                         ChannelControl::RefreshStatusThread, this);    //开启通道状态刷新线程
    if (res != 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "通道[%d]状态刷新线程启动失败！", m_n_channel_index);
        res = ERR_SC_INIT;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER, 0, m_n_channel_index);
        goto END;
    }

    /*printf("------------- %d\n", strlen(m_channel_status.cur_nc_file_name));

    if(strlen(m_channel_status.cur_nc_file_name) > kMaxFileNameLen-2){
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]文件名过长: [%s] 打开失败!",
                m_n_channel_index, m_channel_status.cur_nc_file_name);
        CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                m_n_channel_index);
        goto END;
    }*/


    //默认打开文件
    if(strlen(m_channel_status.cur_nc_file_name) > 0 and
            strlen(m_channel_status.cur_nc_file_name) < kMaxFileNameLen - 2){  // @modify zk 加载过长文件名导致崩溃
        strcpy(path, PATH_NC_FILE);
        strcat(path, m_channel_status.cur_nc_file_name);   //拼接文件绝对路径
        //验证文件是否存在
        if(access(path, F_OK) == -1){//文件不存在，将文件名复位
            g_ptr_trace->PrintLog(LOG_ALARM, "通道[%d]默认打开NC文件[%s]不存在！",
                                  m_n_channel_index, m_channel_status.cur_nc_file_name);
            memset(m_channel_status.cur_nc_file_name, 0x00, kMaxFileNameLen);
        }else{ //文件存在
            this->m_p_compiler->OpenFile(path);
            printf("set nc file cmd : %s\n", path);
        }
    }


END:
    pthread_attr_destroy(&attr);


    //	this->m_p_general_config->hw_rev_trace = 1;   //测试用，强制打开手轮反向跟踪功能  FOR TEST  测试完请删除

    printf("Exit ChannelControl::Initialize\n");
    return true;
}

/**
 * @brief 初始化通道状态
 */
void ChannelControl::InitialChannelStatus(){
    memset(&m_channel_status, 0x00, sizeof(m_channel_status));

    InitGCodeMode();  //初始化G模态

    m_channel_status.rated_manual_speed = 100;	//6000mm/min
    //	m_channel_status.chn_work_mode = INVALID_MODE;
#if defined(USES_WUXI_BLOOD_CHECK) || defined(USES_GENERAL_AUTOMATIC)
    m_channel_status.chn_work_mode = AUTO_MODE;	//默认自动模式
    this->m_p_f_reg->OP = 0;
    this->m_p_f_reg->MMEM = 1;
    this->SendWorkModeToMc(MC_MODE_AUTO);
    //	this->SetWorkMode(AUTO_MODE);
#else
    m_channel_status.chn_work_mode = MANUAL_MODE;	//默认手动连续模式
    this->m_p_f_reg->OP = 0;
    this->m_p_f_reg->MJ = 1;
    //	this->SetWorkMode(MANUAL_MODE);
#endif
    m_channel_status.machining_state = MS_READY;
    m_channel_status.manual_step = 1;	//默认为1um步长
    m_channel_status.spindle_mode = 0;  	//默认速度控制
    m_channel_status.cur_axis = 0;    //默认轴号为0
    m_channel_status.func_state_flags.Init();	//默认均为0
    m_channel_status.auto_ratio = 100;
    m_channel_status.rapid_ratio = 100;
    m_channel_status.spindle_ratio = 100;
    m_channel_status.manual_ratio = 100;
    m_channel_status.workpiece_count = g_ptr_parm_manager->GetCurWorkPiece(m_n_channel_index);
    m_channel_status.workpiece_require = g_ptr_parm_manager->GetCurRequirePiece(m_n_channel_index);
    m_channel_status.workpiece_count_total = g_ptr_parm_manager->GetTotalWorkPiece(m_n_channel_index);
    m_channel_status.machinetime_total = g_ptr_parm_manager->GetCurTotalMachingTime(m_n_channel_index);
    m_channel_status.cur_tool = g_ptr_parm_manager->GetCurTool(m_n_channel_index);

    this->m_n_cur_proc_group = g_ptr_parm_manager->GetCurProcParamIndex(m_n_channel_index);
    this->m_p_f_reg->PPI = m_n_cur_proc_group;

    //	printf("ini ref flag = 0x%hhx, spindle count = %hhu\n", m_channel_status.returned_to_ref_point, m_n_spindle_count);

    if(this->m_n_spindle_count > kMaxSpindleChn){  //主轴个数超限
        CreateError(ERR_CHN_SPINDLE_OVERRUN, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        this->m_error_code = ERR_CHN_SPINDLE_OVERRUN;
    }

    //	strcpy(m_channel_status.cur_nc_file_name, "20.nc");	//for test
    g_ptr_parm_manager->GetCurNcFile(this->m_n_channel_index, m_channel_status.cur_nc_file_name);
}


/**
 * @brief 通道复位函数
 */
void ChannelControl::Reset(){

    this->m_error_code = ERR_NONE;

    if(this->m_thread_breakcontinue > 0){//处于断点继续线程执行过程中，则退出断点继续线程
        this->CancelBreakContinueThread();
    }

    if(this->m_channel_status.machining_state == MS_RUNNING){   //运行中进行复位操作
        this->StopRunGCode(false);
        this->m_b_delay_to_reset = true;
    }else if(this->m_n_run_thread_state != IDLE){  //编译线程运行状态，先停止
        this->StopCompilerRun();
    }

    if(this->m_b_manual_tool_measure){  //终止手动对刀
        m_b_cancel_manual_tool_measure = true;
        this->StopRunGCode(false);
        this->m_b_delay_to_reset = true;
    }

    if(this->m_b_manual_call_macro){   //终止手动调用宏程序
        m_b_cancel_manual_call_macro = true;
        this->m_b_delay_to_reset = true;
        this->StopRunGCode(false);
    }

    if(m_b_delay_to_reset){
        //	printf("ChannelControl::Reset(), delay to return\n");
        return;   //延迟处理复位
    }

    //复位加工复位状态
    this->m_n_restart_mode = NOT_RESTART;
    this->m_n_restart_line = 0;
    this->m_n_restart_step = 0;
    this->m_p_compiler->SetRestartPara(0, NOT_RESTART);

    this->ClearHmiInfoMsg();

    //复位辅助指令输出
    if(this->m_p_f_reg->MF){
        this->m_p_f_reg->MF = 0;
        this->SendMCodeToPmc(0, 0);
    }
    if(this->m_p_f_reg->MF2){
        this->m_p_f_reg->MF2 = 0;
        this->SendMCodeToPmc(0, 1);
    }
    if(this->m_p_f_reg->MF3){
        this->m_p_f_reg->MF3 = 0;
        this->SendMCodeToPmc(0, 2);
    }
    this->m_p_f_reg->DM00 = 0;
    this->m_p_f_reg->DM01 = 0;
    this->m_p_f_reg->DM02 = 0;
    this->m_p_f_reg->DM30 = 0;
    this->m_p_f_reg->DM99 = 0;
    this->m_p_f_reg->DM98 = 0;

#ifdef USES_SPEED_TORQUE_CTRL	
    this->ResetAllAxisOutZero();  // 如果有轴切换了控制模式，首先在此处复位速度值 和 力矩值 为0
#endif

    // 主轴复位
    m_p_spindle->Reset();

#ifdef USES_WOOD_MACHINE
    //	this->m_b_delay_servo_off = false;

    m_b_prestart_spd = false;
    m_n_spd_prestart_step = 0;
    //	printf("spindle prestart over####reset\n");
#endif

#ifdef USES_WUXI_BLOOD_CHECK
    if(this->m_b_returnning_home){
        this->m_b_returnning_home = false;
        this->m_n_ret_home_step = 0;
        this->ManualMoveStop(0x0F);
    }
#endif

#ifdef USES_LASER_MACHINE
    if(this->m_b_laser_calibration){
        //		this->m_b_laser_calibration = false;
        //		this->m_n_calib_step = 0;
        this->m_b_cancel_calib = true;
    }
#endif

    this->ManualMoveStop();
    this->m_mask_run_pmc = 0;
    this->m_mask_runover_pmc = 0;

//    //同步PMC轴的目标位置和当前位置
//    for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
//        if(this->m_mask_pmc_axis & (0x01<<i)){
//            m_channel_rt_status.tar_pos_work.m_df_point[i] = m_channel_rt_status.cur_pos_work.m_df_point[i];
//        }
//    }

    m_n_subprog_count = 0;
    m_n_macroprog_count = 0;
    m_b_ret_from_macroprog = false;
    m_b_init_compiler_pos = false;  //编译器初始位置需要重新初始化
    m_b_need_change_to_pause = false;

    this->m_p_compiler->Reset();
    if(strlen(m_channel_status.cur_nc_file_name) > 0){
        char file_name[128];
        memset(file_name, 0x0, 128);
        strcpy(file_name, PATH_NC_FILE);
        strcat(file_name, m_channel_status.cur_nc_file_name);   //拼接文件绝对路径
        this->SendOpenFileCmdToHmi(m_channel_status.cur_nc_file_name);
    }

    this->m_macro_variable.Reset();   //宏变量复位
    this->m_scene_auto.need_reload_flag = false;   //取消断点继续标志

    this->SendMcResetCmd();   //复位MC
    this->ResetMcLineNo();
    if(this->m_channel_status.chn_work_mode == AUTO_MODE){
        this->InitMcIntpAutoBuf();
    }else if(m_channel_status.chn_work_mode == MDA_MODE){
        this->InitMcIntpMdaBuf();
    }

    if(this->IsStepMode()){
        this->SetMcStepMode(true);
    }

    m_p_output_msg_list_auto->Clear();
    m_p_output_msg_list_mda->Clear();

    this->m_p_last_output_msg = nullptr;
    m_b_change_hw_trace_state = false;
    //	if(this->m_n_hw_trace_state != NONE_TRACE)
    //		this->m_n_hw_trace_state = NORMAL_TRACE;

    SetCurLineNo(1);

#ifdef USES_SPEED_TORQUE_CTRL	
    if(this->ResetAllAxisCtrlMode(0)){
        printf("---the first reset axis_ctrlmode failed! \n");
        m_n_need_reset_axis_ctrlmode = 20;  //  模式复位切换没有全部成功，在idle 函数中再去尝试复位，最多20次
    }else{
        printf("---the first reset axis_ctrlmode succeed! \n");
        m_n_need_reset_axis_ctrlmode = 0;  //   控制模式复位全部成功
    }
#endif

#ifdef USES_ADDITIONAL_PROGRAM
    this->m_n_add_prog_type = NONE_ADD;
    this->m_n_sub_count_bak = 0;
#endif

    //激活待生效坐标系参数
    if(g_ptr_parm_manager->ActiveCoordParam(m_n_channel_index))
        this->SetMcCoord(true);

    //激活待生效的刀具偏置数据
    if(g_ptr_parm_manager->ActiveToolOffsetParam(m_n_channel_index)){
        if(this->m_channel_status.cur_h_code > 0)
            this->SetMcToolOffset(true);
    }

    m_simulate_mode = SIM_NONE;    //默认在非仿真模式
    this->SetMiSimMode(false);  //复位MI仿真状态

    //复位通道状态为READY
    uint8_t state = MS_READY;   //告警状态

    this->SetMachineState(state);  //更新通道状态

    this->ResetMode();   //复位通道模态数据

    this->m_p_f_reg->SPL = 0;
    this->m_p_f_reg->STL = 0;
    this->m_p_f_reg->AL = 0;
    this->m_p_f_reg->OP = 0;

#ifdef USES_GRIND_MACHINE
    this->SetMcSpecialIntpMode(0);  //取消磨削状态

    if(!this->m_b_ret_safe_state){ //执行各轴回安全位置动作
        this->m_b_ret_safe_state = true;
        this->m_n_ret_safe_step = 0;

        gettimeofday(&m_time_ret_safe_delay, nullptr);  //初始化启动回安全位置时间

    }
#endif


#ifdef USES_STOP_SPD_IN_RESET
    if(this->m_n_spindle_count == 0){
        this->SpindleOut(SPD_DIR_STOP);
    }
#endif
    printf("channelcontrol[%hhu] send reset cmd!\n", this->m_n_channel_index);
}

/**
 * @brief 获取对应D值得刀具半径补偿值   半径+磨损
 * @param d_index : D值, 从1开始
 * @return
 */
double ChannelControl::GetToolCompRadius(int d_index){
    double radius = 0.0;

    if(d_index <= 0 && d_index >= kMaxToolCount)
        return radius;

    radius = m_p_chn_tool_config->radius_compensation[d_index-1]+m_p_chn_tool_config->radius_wear[d_index-1];

    return radius;
}

/**
 * @brief PMC轴定位移动
 * @param chn_axis : 通道轴号，从0开始
 * @param tar_pos ： 目标位置
 * @param inc ： 增量模式标志
 * @return true--成功   false--失败
 */
bool ChannelControl::PmcRapidMove(uint8_t chn_axis, double tar_pos, bool inc){
    uint8_t phy_axis = this->GetPhyAxis(chn_axis);
    if(phy_axis == 0xFF)
        return false;


    PmcCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(PmcCmdFrame));

    //命令号  低字节:0--单轴指令   1--多轴G01指令  高字节：0--绝对坐标   1--增量坐标
    cmd.data.cmd = inc?0x100:0;

    //轴号    低字节：轴号[1-64]    高字节：所属通道号[1-4]
    cmd.data.axis_index = ((phy_axis+1)|((m_n_channel_index+1)<<8));

    cmd.data.data[0] = 0;   //定位移动，赋0


    //设置速度
    uint32_t feed = this->m_p_axis_config[phy_axis].rapid_speed * 1000 / 60;   //转换单位为um/s

    //设置目标位置
    int64_t target = tar_pos*1e7;    //转换单位为0.1nm


    memcpy(&cmd.data.data[1], &target, sizeof(target));  //设置目标位置

    memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //设置速度

    this->m_mask_run_pmc |= 0x01L<<phy_axis;  //设置当前运行轴

    this->m_p_mi_comm->SendPmcCmd(cmd);

    printf("PmcRapidMove: axis = %d, tar_pos = %lf\n", phy_axis, tar_pos);

    return true;
}

/**
 * @brief PMC轴切削移动
 * @param total ： G01总帧数
 * @param idx ：当前帧序号，从1开始
 * @param chn_axis : 通道轴号，从0开始
 * @param tar_pos ： 目标位置
 * @param vel ： 目标速度
 * @param inc ： 增量模式标志
 * @return true--成功   false--失败
 */
bool ChannelControl::PmcLineMove(uint8_t total, uint8_t idx, uint8_t chn_axis, double tar_pos, double vel, bool inc){
    //	printf("ChannelControl::PmcLineMove, total=%hhu, idx=%hhu, chn_axis=%hhu, pos=%lf, vel=%lf, inc=%hhu\n", total, idx, chn_axis, tar_pos, vel, inc);
    uint8_t phy_axis = this->GetPhyAxis(chn_axis);
    if(phy_axis == 0xFF)
        return false;


    PmcCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(PmcCmdFrame));

    //命令号  低字节:0--单轴指令   1--多轴G01指令  高字节：0--绝对坐标   1--增量坐标
    cmd.data.cmd = inc?0x101:1;

    //轴号    低字节：轴号[1-64]    高字节：所属通道号[1-4]
    cmd.data.axis_index = ((phy_axis+1)|((m_n_channel_index+1)<<8));

    uint16_t frame = total;
    cmd.data.data[0] = frame<<8|idx;   //总帧数以及当前帧编号

    //设置速度
    uint32_t feed = vel * 1000 / 60;   //转换单位为um/s

    //设置目标位置
    int64_t target = tar_pos*1e7;    //转换单位为0.1nm


    memcpy(&cmd.data.data[1], &target, sizeof(target));  //设置目标位置

    memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //设置速度

    this->m_mask_run_pmc |= 0x01L<<phy_axis;  //设置当前运行轴

    this->m_p_mi_comm->SendPmcCmd(cmd);

    printf("PmcLineMove: axis = %d, tar_pos = %lf, vel = %lf\n", phy_axis, tar_pos, vel);

    return true;
}

/**
 * @brief PMC轴运行到位
 * @param cmd : MI发送的命令帧
 * @return true--成功   false--失败
 */
bool ChannelControl::PmcAxisRunOver(MiCmdFrame cmd){
    printf("ChannelControl::PmcAxisRunOver: phyaxis = %hhu\n", cmd.data.axis_index);

    if(this->m_mask_run_pmc == 0)
        return false;

    uint64_t mask = 0;

    if((cmd.data.data[0]&0xFF) == 0x01){//G01指令
        memcpy(&mask, &cmd.data.data[1], 4);
    }else{
        mask = 0x01L<<(cmd.data.axis_index-1);
    }

    this->m_mask_runover_pmc |= mask;

    if(this->m_mask_run_pmc == this->m_mask_runover_pmc){  //运行结束
        this->m_mask_run_pmc = 0;
        this->m_mask_runover_pmc = 0;

        printf("ChannelControl pmc axis run over\n");
    }

    return true;
}

/**
 * @brief 打开待编译文件
 * @param file_name : 待编译文件的绝对路径
 * @return true--成功  false--失败
 */
//bool ChannelControl::OpenFile(const char *file_name){
//	if(this->m_p_compiler == nullptr){
//		m_error_code = ERR_SC_INIT;
//		return false;
//	}
//
//	if(!m_p_compiler->OpenFile(file_name)){
//		m_error_code = m_p_compiler->GetErrorCode();
//		return false;
//	}
//	return true;
//}

/**
 * @brief 设置仿真模式
 * @param mode : 仿真模式
 * @return true--设置成功   false--设置失败
 */
bool ChannelControl::SetSimulateMode(const SimulateMode mode){
    if(mode == m_simulate_mode)
        return true;

    if(this->m_n_run_thread_state != IDLE && mode != SIM_NONE)
        return false;   //非空闲状态不能切换到仿真模式

    m_simulate_mode = mode;

    if(mode == SIMULATE_MACHINING){//加工仿真，需要通知MI切换至仿真状态
        this->SetMiSimMode(true);
    }else{
        this->SetMiSimMode(false);
    }

    return true;
}

/**
 * @brief 设置MI仿真模式
 * @param flag : true--进入仿真   false--退出仿真
 */
void ChannelControl::SetMiSimMode(bool flag){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_WORK_MODE;

    cmd.data.data[0] = this->m_channel_status.chn_work_mode;
    cmd.data.data[1] = flag?0x10:0x00;


    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = m_n_channel_index;  //通道号，从0开始

    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 获取G地址寄存器状态
 * @param sec ： 地址号
 * @param bit : bit位序号，0-7
 * @return true--有信号  false--无信号
 */
bool ChannelControl::CheckGRegState(int sec, int bit){
    if(sec > 255 || bit > 7)
        return false;

    uint8_t value = this->m_p_pmc_reg->GReg().all[256*this->m_n_channel_index+sec];
    return (value&(0x01<<bit))?true:false;
}

/**
 * @brief 获取F地址寄存器状态
 * @param sec ： 地址号
 * @param bit : bit位序号，0-7
 * @return true--有信号  false--无信号
 */
bool ChannelControl::CheckFRegState(int sec, int bit){
    if(sec > 255 || bit > 7)
        return false;

    uint8_t value = this->m_p_pmc_reg->FReg().all[256*this->m_n_channel_index+sec];
    return (value&(0x01<<bit))?true:false;
}

/**
 * @brief 设置F地址寄存器的值
 * @param sec ： 地址号
 * @param bit : bit位序号，0-7
 * @param value : true--有信号  false--无信号
 * @return  true--成功   false--失败
 */
bool ChannelControl::SetFRegValue(int sec, int bit, bool value){
    if(sec > 255 || bit > 7)
        return false;

    FRegister &freg = this->m_p_pmc_reg->FReg();
    uint8_t tt = freg.all[256*this->m_n_channel_index+sec];
    if(value)
        tt |= (0x01<<bit);
    else
        tt &= ~(0x01<<bit);
    freg.all[256*this->m_n_channel_index+sec] = tt;
    return true;
}

/**
 * @brief 获取#1032的值
 * @return G54~G57的值
 */
uint32_t ChannelControl::GetMacroVar1032(){
    uint32_t value = 0;

    memcpy(&value, &this->m_p_pmc_reg->GReg().all[256*this->m_n_channel_index+54], 4);

    return value;
}

/**
 * @brief 获取#1132的值
 * @return F54~F55 16bits的值
 */
uint16_t ChannelControl::GetMacroVar1132(){
    uint16_t value = 0;

    memcpy(&value, &this->m_p_pmc_reg->FReg().all[256*this->m_n_channel_index+54], 2);

    return value;
}

/**
 * @brief 获取#1133的值
 * @return F56~F59 32bits的值
 */
uint32_t ChannelControl::GetMacroVar1133(){
    uint32_t value = 0;

    memcpy(&value, &this->m_p_pmc_reg->FReg().all[256*this->m_n_channel_index+56], 4);

    return value;
}

/**
 * @brief 设置#1132的值
 * @param value
 */
void ChannelControl::SetMacroVar1132(uint16_t value){
    memcpy(&this->m_p_pmc_reg->FReg().all[256*this->m_n_channel_index+54], &value, 2);
}

/**
 * @brief 设置#1133的值
 * @param value
 */
void ChannelControl::SetMacroVar1133(uint32_t value){
    memcpy(&this->m_p_pmc_reg->FReg().all[256*this->m_n_channel_index+56], &value, 4);
}

/**
 * @brief 返回指定宏变量的值
 * @param index[in] : 系统变量索引
 * @param value[out] : 返回系统变量值
 * @return 0--成功，已初始化   1--非法索引    2--未初始化
 */
int ChannelControl::GetMacroVar(const int index, double &value){
    bool init = false;
    bool res = this->m_macro_variable.GetVarValue(index, value, init);

    if(!res)
        return 1;
    if(!init)
        return 2;

    return 0;
}

/**
 * @brief 获取系统变量值
 * @param index[in] : 系统变量索引
 * @param value[out] : 返回系统变量值
 * @return
 */
bool ChannelControl::GetSysVarValue(const int index, double&value){

    if(index == 1220){    //主轴状态
        Spindle::Polar polar = m_p_spindle->CalPolar();
        if(polar == Polar::Stop)
            value = 5;
        else if(polar == Polar::Positive)
            value = 3;
        else if(polar == Polar::Negative)
            value = 4;
    }
#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
    else if(index == 2000){  //基准刀的几何刀偏
        value = this->m_p_chn_tool_config->geometry_comp_basic[2];
    }
#endif
    else if(index >= 2001 && index <= 2200){   //刀长磨损补偿
        int id = index - 2001;
        if(id < kMaxToolCount)
            value = this->m_p_chn_tool_config->geometry_wear[id];
        else
            value = 0.0;

    }else if(index >= 2201 && index <= 2400){  //刀长几何补偿
        int id = index - 2201;
        if(id < kMaxToolCount)
            value = this->m_p_chn_tool_config->geometry_compensation[id][2];
        else
            value = 0.0;
    }else if(index == 3019){  //正在执行宏的路径号
        value = this->m_n_channel_index;

    }else if(index == 3901){  //已加工件数
        //value = this->m_channel_status.workpiece_count;
        value = this->m_channel_status.workpiece_count_total;
    }else if(index >= 4001 && index <= 4030){   //上

    }else if(index >= 4201 && index <= 4230){    //当前G指令模态
    	value = this->m_channel_status.gmode[index-4201];  //模态为G指令乘以10，G43.4保存为434
        value /= 10;
    }else if(index == 4307){   //当前D值
        value = this->m_channel_status.cur_d_code;
    }else if(index == 4309){   //当前F值
        value =  this->m_channel_status.rated_feed;
    }else if(index == 4311){   //当前H值
        value = this->m_channel_status.cur_h_code;
    }else if(index == 4313){   //TODO 当前M值
        uint32_t mcode = this->m_p_f_reg->mcode_3;
        mcode = mcode<<8;
        mcode |= this->m_p_f_reg->mcode_2;
        mcode = mcode<<8;
        mcode |= this->m_p_f_reg->mcode_1;
        mcode = mcode<<8;
        mcode |= this->m_p_f_reg->mcode_0;
        value = mcode;
    }else if(index == 4319){   //当前S值
        value = this->m_channel_status.rated_spindle_speed;
    }else if(index == 4320){   //当前T值
        value = this->m_channel_status.cur_tool;
    }else if(index == 4321){   //当前预选刀号
        value = this->m_channel_status.preselect_tool_no;
    }else if(index >= 5001 && index < 5001+kMaxAxisChn){   //当前目标位置， 工件坐标系
        value = this->m_channel_rt_status.tar_pos_work.m_df_point[index-5001];
    }else if(index >= 5021 && index < 5021+kMaxAxisChn){  //当前机械坐标
        value = this->m_channel_rt_status.cur_pos_machine.m_df_point[index-5021];
    }else if(index >= 5041 && index < 5041+kMaxAxisChn){   //当前工件坐标
        value = this->m_channel_rt_status.cur_pos_work.m_df_point[index-5041];
    }else if(index >= 5061 && index < 5061+kMaxAxisChn){  //G31捕获的轴位置，工件坐标系
        value = m_point_capture.m_df_point[index - 5061];
        this->TransMachCoordToWorkCoord(value, this->m_channel_status.gmode[14], index-5061);   //机械坐标系转化为工件坐标系
    }else if(index >= 5081 && index < 5084){  //刀具XYZ轴向长度补偿
        int id = index - 5081;
        if(this->m_channel_status.cur_h_code > 0)
            value = this->m_p_chn_tool_config->geometry_compensation[this->m_channel_status.cur_h_code-1][id];
        else
            value = 0;

    }else if(index >= 5201 && index <= 5220){   //基本坐标系偏移
        int id = index-5201;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[0].offset[id];
        else
            value = 0.0;
    }else if(index >= 5221 && index <= 5240){   //G54坐标系偏移
        int id = index-5221;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[1].offset[id];
        else
            value = 0.0;
    }else if(index >= 5241 && index <= 5260){   //G55坐标系偏移
        int id = index-5241;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[2].offset[id];
        else
            value = 0.0;
    }else if(index >= 5261 && index <= 5280){   //G56坐标系偏移
        int id = index-5261;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[3].offset[id];
        else
            value = 0.0;
    }else if(index >= 5281 && index <= 5300){   //G57坐标系偏移
        int id = index-5281;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[4].offset[id];
        else
            value = 0.0;
    }else if(index >= 5301 && index <= 5320){   //G58坐标系偏移
        int id = index-5301;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[5].offset[id];
        else
            value = 0.0;
    }else if(index >= 5321 && index <= 5340){   //G59坐标系偏移
        int id = index-5321;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[6].offset[id];
        else
            value = 0.0;
    }else if(index >= 5421 && index <= 5440){//G31捕获的轴位置，机械坐标系
        if(index - 5421 < kMaxAxisChn)
            value = m_point_capture.m_df_point[index-5421];
    }else if(index >= 7001 && index <= 8980){
        printf("alarm index: %d\n", index);
        int coord = (index - 7001)/20;  //  G540X  求X值

        if(coord < this->m_p_channel_config->ex_coord_count){
            int id = (index - 7001)%20;
            if(id < this->m_p_channel_config->chn_axis_count)
                value = m_p_chn_ex_coord_config[coord].offset[id];
            else
                value = 0.0;
        }else{
            return false;
        }
    }else if(index == 9000){
    	value = this->m_p_channel_config->G73back;
    }else if(index == 9001){
    	value = this->m_p_channel_config->G83back;
    }
    else if(index >= 12001 && index <= 12999){   //刀具半径磨损补偿
        int id = index - 12001;
        if(id < kMaxToolCount)
            value = this->m_p_chn_tool_config->radius_wear[id];
        else
            value = 0.0;
    }else if(index >= 13001 && index <= 13999){   //刀具半径几何补偿
        int id = index - 13001;
        if(id < kMaxToolCount)
            value = this->m_p_chn_tool_config->radius_compensation[id];
        else
            value = 0.0;
    }else if(index >= 30001 && index <= 30060) {    //刀具寿命管理方式
        int id = index - 30001;
        value = this->m_p_chn_tool_info->tool_life_type[id];
    }else if(index >= 30101 && index <= 30160) {    //刀具最大寿命
        int id = index - 30101;
        value = this->m_p_chn_tool_info->tool_life_max[id];
    }else if(index >= 30201 && index <= 30260) {    //刀具已用寿命
        int id = index - 30201;
        value = this->m_p_chn_tool_info->tool_life_cur[id];
    }else if(index >= 30301 && index <= 30360) {    //刀具预警寿命
        int id = index - 30301;
        value = this->m_p_chn_tool_info->tool_threshold[id];
    }else{
        return false;
    }

    return true;
}

/**
 * @brief 设置系统变量
 * @param index[in] : 系统变量索引
 * @param value[in] : 系统变量值
 * @return
 */
bool ChannelControl::SetSysVarValue(const int index, const double &value){
#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
    if(index == 2000){  //基准刀的几何刀偏
        this->m_p_chn_tool_config->geometry_comp_basic[2] = value;
        g_ptr_parm_manager->UpdateToolMeasure(this->m_n_channel_index, 0, value);

    }else if(index >= 2001 && index <= 2200){   //刀长磨损补偿
#else
    if(index >= 2001 && index <= 2200){   //刀长磨损补偿
#endif
        int id = index - 2001;
        if(id < kMaxToolCount){
            this->m_p_chn_tool_config->geometry_wear[id] = value;
            g_ptr_parm_manager->UpdateToolWear(m_n_channel_index, id, value);
            this->NotifyHmiToolOffsetChanged(id+1);   //通知HMI刀偏值更改
        }else
            return false;

    }else if(index >= 2201 && index <= 2400){  //刀长几何补偿
        int id = index - 2201;
        if(id < kMaxToolCount){
            this->m_p_chn_tool_config->geometry_compensation[id][2] = value;
            // @modify 为什么就这个 id + 1  其他的Update 没有 id + 1
            g_ptr_parm_manager->UpdateToolMeasure(this->m_n_channel_index, id, value);
            //g_ptr_parm_manager->UpdateToolMeasure(this->m_n_channel_index, id+1, value);
            this->NotifyHmiToolOffsetChanged(id+1);   //通知HMI刀偏值更改
        }else
            return false;
    }else if(index == 3000){  //只写变量，显示提示信息
        pthread_mutex_unlock(&m_mutex_change_state);
        pthread_mutex_unlock(&m_mutex_change_state);
    	uint16_t msg_id = value;
        printf("set 3000 value = %u\n", msg_id);
        if(msg_id >= 0){
			CreateError(msg_id, INFO_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
			printf("3000 msg_id: %d\n", msg_id);
		}
    }else if(index == 3006){  //只写变量，显示告警信息
    	pthread_mutex_unlock(&m_mutex_change_state);
    	uint16_t err_id = value;
        if(err_id > 0){
            CreateError(err_id, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            printf("3006 err_id: %d\n", err_id);
        }
    }else if(index == 3901){  //已加工件数
        SetChnCurWorkPiece(value);
    }else if(index == 4320){  //设置当前刀号
        this->m_channel_status.cur_tool = value;
        g_ptr_parm_manager->SetCurTool(m_n_channel_index, m_channel_status.cur_tool);
        this->SendModeChangToHmi(T_MODE);

#ifdef USES_WOOD_MACHINE
        this->SetMcToolLife();
#endif
    }
    else if(index >= 5201 && index <= 5220){   //基本坐标系偏移
        int id = index-5201;
        if(id < this->m_p_channel_config->chn_axis_count){
            this->m_p_chn_coord_config[0].offset[id] = value;
            HmiCoordConfig cfg;
            memcpy(&cfg, &this->m_p_chn_coord_config[0], sizeof(HmiCoordConfig));
            g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, 0, cfg, true);
            this->SetMcCoord(true);
            this->NotifyHmiWorkcoordChanged(0);
        }
        else
            return false;
    }else if(index >= 5221 && index <= 5240){   //G54坐标系偏移
        int id = index-5221;
        if(id < this->m_p_channel_config->chn_axis_count){
            this->m_p_chn_coord_config[1].offset[id] = value;
            HmiCoordConfig cfg;
            memcpy(&cfg, &this->m_p_chn_coord_config[1], sizeof(HmiCoordConfig));
            g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, 1, cfg, true);
            this->NotifyHmiWorkcoordChanged(1);
            if(this->m_channel_status.gmode[14] == (G54_CMD))
                this->SetMcCoord(true);
        }
        else
            return false;
    }else if(index >= 5241 && index <= 5260){   //G55坐标系偏移
        int id = index-5241;
        if(id < this->m_p_channel_config->chn_axis_count){
            this->m_p_chn_coord_config[2].offset[id] = value;

            HmiCoordConfig cfg;
            memcpy(&cfg, &this->m_p_chn_coord_config[2], sizeof(HmiCoordConfig));
            g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, 2, cfg, true);
            this->NotifyHmiWorkcoordChanged(2);
            if(this->m_channel_status.gmode[14] == (G55_CMD))
                this->SetMcCoord(true);
        }
        else
            return false;
    }else if(index >= 5261 && index <= 5280){   //G56坐标系偏移
        int id = index-5261;
        if(id < this->m_p_channel_config->chn_axis_count){
            this->m_p_chn_coord_config[3].offset[id] = value;
            HmiCoordConfig cfg;
            memcpy(&cfg, &this->m_p_chn_coord_config[3], sizeof(HmiCoordConfig));
            g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, 3, cfg, true);
            this->NotifyHmiWorkcoordChanged(3);
            if(this->m_channel_status.gmode[14] == G56_CMD)
                this->SetMcCoord(true);
        }
        else
            return false;
    }else if(index >= 5281 && index <= 5300){   //G57坐标系偏移
        int id = index-5281;
        if(id < this->m_p_channel_config->chn_axis_count){
            this->m_p_chn_coord_config[4].offset[id] = value;
            HmiCoordConfig cfg;
            memcpy(&cfg, &this->m_p_chn_coord_config[4], sizeof(HmiCoordConfig));
            g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, 4, cfg, true);
            this->NotifyHmiWorkcoordChanged(4);
            if(this->m_channel_status.gmode[14] == (G57_CMD))
                this->SetMcCoord(true);
        }
        else
            return false;
    }else if(index >= 5301 && index <= 5320){   //G58坐标系偏移
        int id = index-5301;
        if(id < this->m_p_channel_config->chn_axis_count){
            this->m_p_chn_coord_config[5].offset[id] = value;

            HmiCoordConfig cfg;
            memcpy(&cfg, &this->m_p_chn_coord_config[5], sizeof(HmiCoordConfig));
            g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, 5, cfg, true);
            this->NotifyHmiWorkcoordChanged(5);
            if(this->m_channel_status.gmode[14] == (G58_CMD))
                this->SetMcCoord(true);
        }
        else
            return false;
    }else if(index >= 5321 && index <= 5340){   //G59坐标系偏移
        int id = index-5321;
        if(id < this->m_p_channel_config->chn_axis_count){
            this->m_p_chn_coord_config[6].offset[id] = value;
            HmiCoordConfig cfg;
            memcpy(&cfg, &this->m_p_chn_coord_config[6], sizeof(HmiCoordConfig));
            g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, 6, cfg, true);
            this->NotifyHmiWorkcoordChanged(6);
            if(this->m_channel_status.gmode[14] == (G59_CMD))
                this->SetMcCoord(true);
        }
        else
            return false;
    }else if(index >= 7001 && index <= 8980){
        int coord = (index - 7001)/20;  //  G540X  求X值
        // 超过了最大扩展坐标数量
        if(coord < this->m_p_channel_config->ex_coord_count){
            int id = (index - 7001)%20;
            if(id < this->m_p_channel_config->chn_axis_count){
                this->m_p_chn_ex_coord_config[coord].offset[id] = value;
                HmiCoordConfig cfg;
                memcpy(&cfg, &this->m_p_chn_ex_coord_config[coord], sizeof(HmiCoordConfig));
                g_ptr_parm_manager->UpdateExCoordConfig(m_n_channel_index, coord, cfg, true);
                this->NotifyHmiWorkcoordExChanged(coord);
                if(this->m_channel_status.gmode[14] == (G5401_CMD+10*coord))
                    this->SetMcCoord(true);
            }else{
                return false;
            }
        }else{
            return false;
        }
    }
    else if(index >= 12001 && index <= 12999){   //刀具半径磨损补偿
        int id = index - 12001;
        if(id < kMaxToolCount){
            this->m_p_chn_tool_config->radius_wear[id] = value;
            g_ptr_parm_manager->UpdateToolRadiusWear(m_n_channel_index, id, value);
            this->NotifyHmiToolOffsetChanged(id+1);   //通知HMI刀偏值更改
        }
        else
            return false;;
    }
    else if(index >= 13001 && index <= 13999){   //刀具半径几何补偿
        int id = index - 13001;
        if(id < kMaxToolCount){
            this->m_p_chn_tool_config->radius_compensation[id] = value;

            g_ptr_parm_manager->UpdateToolRadiusGeo(m_n_channel_index, id, value);
            this->NotifyHmiToolOffsetChanged(id+1);   //通知HMI刀偏值更改
        }
        else
            return false;
    }
    else if(index >= 30001 && index <= 30060) {    //刀具寿命管理方式
        int id = index - 30001;
        this->m_p_chn_tool_info->tool_life_type[id] = value;
        NotifyHmiToolPotChanged();
    }else if(index >= 30101 && index <= 30160) {    //刀具最大寿命
        int id = index - 30101;
        this->m_p_chn_tool_info->tool_life_max[id] = value;
        NotifyHmiToolPotChanged();
    }else if(index >= 30201 && index <= 30260) {    //刀具已用寿命
        int id = index - 30201;
        this->m_p_chn_tool_info->tool_life_cur[id] = value;
        NotifyHmiToolPotChanged();
    }else if(index >= 30301 && index <= 30360) {    //刀具预警寿命
        int id = index - 30301;
        this->m_p_chn_tool_info->tool_threshold[id] = value;
        NotifyHmiToolPotChanged();
    }
    else
        return false;

    return true;
}

/**
 * @brief 初始化传递给MC的模态信息
 */
void ChannelControl::InitMcModeStatus(){
    this->m_mc_mode_exec.all = 0;
    this->m_mc_mode_cur.all = 0;

    //02组
    m_mc_mode_exec.bits.mode_g17 = (m_channel_status.gmode[2]-G17_CMD)/10;

    //03组
    m_mc_mode_exec.bits.mode_g90 = (m_channel_status.gmode[3]-G90_CMD)/10;

    //05组
    m_mc_mode_exec.bits.mode_g94 = (m_channel_status.gmode[5]-G94_CMD)/10;

    //06组
    m_mc_mode_exec.bits.mode_g20 = (m_channel_status.gmode[6]-G20_CMD)/10;

    //07组
    m_mc_mode_exec.bits.mode_g40 = (m_channel_status.gmode[7]-G40_CMD)/10;

    //09组
    if(m_channel_status.gmode[9] == G89_CMD)
        m_mc_mode_exec.bits.mode_g73 = 15;
    else
        m_mc_mode_exec.bits.mode_g73 = (m_channel_status.gmode[9]-G73_CMD)/10;

    //10组
    m_mc_mode_exec.bits.mode_g98 = (m_channel_status.gmode[10]-G98_CMD)/10;

    //11组
    m_mc_mode_exec.bits.mode_g50 = (m_channel_status.gmode[11]-G50_CMD)/10;

    //13组
    m_mc_mode_exec.bits.mode_g60 = (m_channel_status.gmode[13]-G60_CMD)/10;

    //16组
    m_mc_mode_exec.bits.mode_g68 = (m_channel_status.gmode[16]-G68_CMD)/10;

    //D值
    m_mc_mode_exec.bits.mode_d = m_channel_status.cur_d_code;
}

/**
 * @brief 初始化加工复位中间模态
 */
void ChannelControl::InitRestartMode(){
    m_mode_restart.gmode[1] = G00_CMD;  //01组默认G00
    m_mode_restart.gmode[2] = G17_CMD;  //02组默认G17
    m_mode_restart.gmode[3] = G90_CMD;  //03组默认G90
    //	m_mode_restart.gmode[4] = G23_CMD;
    m_mode_restart.gmode[5] = G94_CMD;  //05组默认G94  每分钟进给
    m_mode_restart.gmode[6] = G21_CMD;  //06组默认G21  公制单位

    m_mode_restart.gmode[7] = G40_CMD;  //07组默认G40
    m_mode_restart.gmode[8] = this->m_channel_status.gmode[8];  //08组默认G49
    m_mode_restart.gmode[9] = G80_CMD;  //09组默认G80
    m_mode_restart.gmode[10] = G98_CMD;  //10组默认G98  返回初始平面
    m_mode_restart.gmode[11] = G50_CMD;  //11组默认G50  比例缩放取消
    m_mode_restart.gmode[14] = this->m_channel_status.gmode[14];  //14组默认G54  G54工件坐标系
    m_mode_restart.gmode[15] = G64_CMD;  //15组默认G64  切削方式
    m_mode_restart.gmode[16] = G69_CMD;  //16组默认G69  坐标旋转或者三维坐标变换方式OFF

    m_mode_restart.gmode[17] = G15_CMD;  //17组默认15   极坐标指令取消
    m_mode_restart.gmode[19] = G26_CMD;  //19组默认G26  主轴速度变动检测ON

    m_mode_restart.cur_d_code = this->m_channel_status.cur_d_code;
    m_mode_restart.cur_h_code = this->m_channel_status.cur_h_code;
    m_mode_restart.cur_tool = this->m_channel_status.cur_tool;
    m_mode_restart.rated_feed = 0;
    m_mode_restart.rated_spindle_speed = 0;
    m_mode_restart.spindle_dir = 0;

    m_mode_restart.sub_prog_call = 0;

    memset(m_mode_restart.pos_target.m_df_point, 0x00, sizeof(double)*kMaxAxisChn);
}

/**
 * @brief 返回通道MDA文件名称
 * @param path[out] : 返回通道MDA文件的绝对路径
 */
void ChannelControl::GetMdaFilePath(char *path){
    if(path == nullptr)
        return;

    sprintf(path, PATH_MDA_FILE, this->m_n_channel_index);
}

PmcAxisCtrl *ChannelControl::GetPmcAxisCtrl(){
    return m_p_channel_engine->GetChnPmcAxisCtrl(this->m_n_channel_index);
}

void ChannelControl::RefreshPmcAxis()
{
    m_p_compiler->RefreshPmcAxis();
}

/**
 * @brief 清除加工累计时间
 */
void ChannelControl::ClearMachineTimeTotal()
{
    m_channel_rt_status.machining_time_total = 0;
}

/**
 * @brief 复位通道模态为默认值
 */
void ChannelControl::ResetMode(){
    printf("reset mode start!!!\n");

    //01组根据通道参数：默认进给方式 来复位
    if(m_p_channel_config->default_feed_mode == 0){
        m_channel_status.gmode[1] = G00_CMD;
    }else if(m_p_channel_config->default_feed_mode == 1){
        m_channel_status.gmode[1] = G01_CMD;
    }else if(m_p_channel_config->default_feed_mode == 2){
        m_channel_status.gmode[1] = G02_CMD;
    }else{
        m_channel_status.gmode[1] = G03_CMD;
    }

    //02组根据通道参数：默认加工平面 来复位
    if(m_p_channel_config->default_plane == 0){
        m_channel_status.gmode[2] = G17_CMD;
    }else if(m_p_channel_config->default_plane == 1){
        m_channel_status.gmode[2] = G18_CMD;
    }else{
        m_channel_status.gmode[2] = G19_CMD;
    }

    // 03组根据通道参数：默认指令模式 来复位
    if(m_p_channel_config->default_cmd_mode == 0){
        m_channel_status.gmode[3] = G90_CMD;
    }else{
        m_channel_status.gmode[3] = G91_CMD;
    }

    m_channel_status.gmode[5] = G94_CMD;  //05组默认G94  每分钟进给
    m_channel_status.gmode[6] = G21_CMD;  //06组默认G21  公制单位

    m_channel_status.gmode[7] = G40_CMD;  //07组默认G40
    //	m_channel_status.gmode[8] = G49_CMD;  //08组默认G49
    m_channel_status.gmode[9] = G80_CMD;  //09组默认G80
    m_channel_status.gmode[10] = G98_CMD;  //10组默认G98  返回初始平面
    m_channel_status.gmode[11] = G50_CMD;  //11组默认G50  比例缩放取消
    //	m_channel_status.gmode[14] = G54_CMD;  //14组默认G54  G54工件坐标系
    m_channel_status.gmode[15] = G64_CMD;  //15组默认G64  切削方式
    m_channel_status.gmode[16] = G69_CMD;  //16组默认G69  坐标旋转或者三维坐标变换方式OFF

    m_channel_status.gmode[17] = G15_CMD;  //17组默认15   极坐标指令取消
    m_channel_status.gmode[19] = G26_CMD;  //19组默认G26  主轴速度变动检测ON

    m_channel_status.cur_d_code = 0;
    //	m_channel_status.cur_h_code = 0;

    //	this->SetMcToolOffset(false);   //取消刀偏
    this->SendChnStatusChangeCmdToHmi(G_MODE);
    this->SendModeChangToHmi(H_MODE);
    this->SendModeChangToHmi(D_MODE);
    printf("reset mode finished!!!\n");
}

/**
 * @brief 开始G代码运行
 */
void ChannelControl::StartRunGCode(){
	printf("start run g code \n");

	g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC,"@#@#Enter ChannelControl::StartRunGCode(), m_n_run_thread_state = %d, chn_work_mode = %hhu, machine_mode = %hhu, mc_mode=%hu\n", m_n_run_thread_state,
			m_channel_status.chn_work_mode, m_channel_status.machining_state, m_channel_mc_status.cur_mode);


	if(m_channel_status.chn_work_mode != AUTO_MODE &&
			m_channel_status.chn_work_mode != MDA_MODE)
		return;


#ifdef USES_GRIND_MACHINE
    if(this->m_b_ret_safe_state){  //回安全位置动作未完成
        printf("回安全位置动作未完成，不执行运行动作\n");
        return;
    }
#endif

    //检查轴是否回参考点
    uint32_t axis_mask = 0;
    for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if((m_channel_status.returned_to_ref_point & (0x01<<i)) == 0x00){
            //有轴未回参考点，通知HMI
            axis_mask |= (0x01<<i);
            g_ptr_trace->PrintLog(LOG_ALARM, "通道[%hhu]轴%hhu未回参考点，禁止自动运行！\n", m_n_channel_index, this->m_p_channel_config->chn_axis_name[i]);
        }
    }
    if(axis_mask != 0){
        this->m_error_code = ERR_AXIS_REF_NONE;
        CreateError(m_error_code, WARNING_LEVEL, CLEAR_BY_MCP_RESET, axis_mask, m_n_channel_index);
        //	this->SendMessageToHmi(MSG_TIPS, MSG_ID_AXIS_REF);
        return;
    }

    //检查当前是否处于错误状态
    if(g_ptr_alarm_processor->HasErrorInfo(m_n_channel_index)){
        ErrorInfo err_info;
        g_ptr_alarm_processor->GetLatestErrorInfo(&err_info);
        g_ptr_trace->PrintLog(LOG_ALARM, "系统处于告警状态，禁止自动运行！last error = %d", err_info.error_code);
        g_ptr_alarm_processor->PrintError();
        return;
    }

    //当前的加工文件是否存在
    if(!this->IsNcFileExist(this->m_channel_status.cur_nc_file_name)){
        this->m_error_code = ERR_NC_FILE_NOT_EXIST;
        CreateError(m_error_code, WARNING_LEVEL, CLEAR_BY_MCP_RESET, axis_mask, m_n_channel_index);
        return;
    }


    //当前正处于停止或暂停过程中，则返回
    if(m_channel_status.machining_state == MS_PAUSING || m_channel_status.machining_state == MS_STOPPING){
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "machine state is %hhu, return\n", m_channel_status.machining_state);
        return;
    }

#ifdef USES_ADDITIONAL_PROGRAM
    AddProgType add_type = NONE_ADD;   //运行附加程序类型
#endif

    uint8_t state = MS_RUNNING;

    if(m_simulate_mode == SIM_OUTLINE)
        state = MS_OUTLINE_SIMULATING;
    else if(m_simulate_mode == SIM_TOOLPATH)
        state = MS_TOOL_PATH_SIMULATING;
    else if(m_simulate_mode == SIM_MACHINING)
        state = MS_MACHIN_SIMULATING;

    if(this->m_n_restart_mode != NOT_RESTART){  //加工复位
        this->m_p_compiler->SetRestartPara(m_n_restart_line, m_n_restart_mode);  //设置编译器加工复位参数
        this->InitRestartMode();
        this->SendMessageToHmi(MSG_TIPS, MSG_ID_RESTARTING);   //发送HMI的提示信息
    }

    if(m_channel_status.machining_state == MS_READY &&
            (state == MS_RUNNING || state == MS_MACHIN_SIMULATING)){ //开始一次新的运行，复位MC
        this->ResetMcLineNo();

    }

    if(!this->m_b_init_compiler_pos){
        this->m_p_compiler->SetCurPos(this->m_channel_rt_status.cur_pos_work);
        this->m_p_compiler->SetCurGMode();   //初始化编译器模态
        this->m_b_init_compiler_pos = true;
    }

    if(this->m_channel_status.chn_work_mode == AUTO_MODE){

        string msg = "开始加工程序(" + string(this->m_channel_status.cur_nc_file_name) + ")";
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug, msg);

        if(this->m_channel_status.machining_state == MS_READY){  //就绪状态则直接启动编译
            this->SendWorkModeToMc(MC_MODE_AUTO);

            //初始化仿真绘图位置数据缓冲相关变量
            m_n_graph_pos_count = 0;
            m_n_graph_pos_write_idx = 0;
            m_n_graph_pos_read_idx = 0;

            //激活参数
            g_ptr_parm_manager->ActiveNewStartParam();

            this->m_macro_variable.Reset();

            //初始化传递给MC模块的模态组信息
            this->InitMcModeStatus();

            this->m_p_compiler->SetCurPos(this->m_channel_rt_status.cur_pos_work);  //设置编译器起始位置
            this->m_p_compiler->SetCurGMode();   //初始化编译器模态

            if(this->m_simulate_mode != SIM_NONE){
                this->m_pos_simulate_cur_work = this->m_channel_rt_status.cur_pos_work;   //初始化仿真模式当前工件坐标
#ifdef USES_SIMULATION_TEST
                if(m_file_sim_data > 0){
                    close(m_file_sim_data);
                    m_file_sim_data = -1;  //先关闭文件
                }
                m_file_sim_data = open(PATH_SIMULATE_DATA, O_CREAT|O_TRUNC|O_WRONLY); //打开文件
                if(m_file_sim_data < 0){
                    CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
                    this->m_error_code = ERR_OPEN_FILE;
                    g_ptr_trace->PrintLog(LOG_ALARM, "打开仿真数据临时文件失败！");
                    return;//文件打开失败
                }
#endif	
                SimulateData data;
                MonitorDataType type = MDT_CHN_SIM_GRAPH;  //数据类型
                if(this->m_simulate_mode == SIM_TOOLPATH)
                    type = MDT_CHN_SIM_TOOL_PATH;
                else if(this->m_simulate_mode == SIM_MACHINING)
                    type = MDT_CHN_SIM_MACH;
                //写入仿真范围
                data.type = ZONE_LIMIT_MIN;   //首先写入负向限制
                data.pos[0] = this->m_p_axis_config[GetPhyAxisFromName(AXIS_NAME_X)].soft_limit_min_1;
                data.pos[1] = this->m_p_axis_config[GetPhyAxisFromName(AXIS_NAME_Y)].soft_limit_min_1;
                data.pos[2] = this->m_p_axis_config[GetPhyAxisFromName(AXIS_NAME_Z)].soft_limit_min_1;
                this->SendSimulateDataToHmi(type, data);   //发送负向限制

                data.type = ZONE_LIMIT_MAX;   //再写入正向限制
                data.pos[0] = this->m_p_axis_config[GetPhyAxisFromName(AXIS_NAME_X)].soft_limit_max_1;
                data.pos[1] = this->m_p_axis_config[GetPhyAxisFromName(AXIS_NAME_Y)].soft_limit_max_1;
                data.pos[2] = this->m_p_axis_config[GetPhyAxisFromName(AXIS_NAME_Z)].soft_limit_max_1;
                this->SendSimulateDataToHmi(type, data);   //发送正向限制

                //写入当前位置
                data.type = ORIGIN_POINT;
                data.pos[0] = this->GetAxisCurMachPos(this->GetChnAxisFromName(AXIS_NAME_X));
                data.pos[1] = this->GetAxisCurMachPos(this->GetChnAxisFromName(AXIS_NAME_Y));
                data.pos[2] = this->GetAxisCurMachPos(this->GetChnAxisFromName(AXIS_NAME_Z));
                this->SendSimulateDataToHmi(type, data);   //发送初始位置
            }
#ifdef USES_ADDITIONAL_PROGRAM
            else if(this->m_n_restart_mode != NOT_RESTART){  //加工复位
                add_type = RESET_START_ADD;     //
            }else{
                add_type = NORMAL_START_ADD;
            }
#endif

#ifdef USES_WOOD_MACHINE
            m_b_prestart_spd = false;
            this->m_n_spd_prestart_step = 0;
#endif

            m_channel_rt_status.machining_time = 0;    //重置加工时间
        }
        else if(m_channel_status.machining_state == MS_PAUSED){
#ifndef USES_GRIND_MACHINE  //玻璃机不需要断点返回功能
#ifndef USES_ADDITIONAL_PROGRAM
            if(this->m_scene_auto.need_reload_flag){
#endif
                if(StartBreakpointContinue()){
                    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "start break point continue thread\n");
                    gettimeofday(&m_time_start_maching, nullptr);  //初始化启动时间
                    goto END;
                }
#ifndef USES_ADDITIONAL_PROGRAM
            }
#endif
#else

#endif
            if(this->m_mask_run_pmc){
                this->PausePmcAxis(NO_AXIS, false);  //继续运行PMC轴
            }else if(IsMcNeedStart()){
                this->StartMcIntepolate();
                printf("---------1111111--------------\n");
            }
            else{
#ifdef USES_GRIND_MACHINE
                RecordMsg *msg = nullptr;
                ListNode<RecordMsg *> *node = m_p_output_msg_list->HeadNode();
                if(node != nullptr){
                    msg = static_cast<RecordMsg *>(node->data);

                    if(msg->GetMsgType() == POLAR_INTP_MSG){
                        PolarIntpMsg *polarmsg = (PolarIntpMsg *)msg;
                        if(polarmsg->GetExecStep() > 0){
                            polarmsg->SetExecStep(0);    //重新启动G12.3指令
                        }
                    }
                }
#endif
            }

        }else{
            printf("StartRunGCode WARNING:m_channel_status.machining_state: %hhu\n", m_channel_status.machining_state);
        }

        gettimeofday(&m_time_start_maching, nullptr);  //初始化启动时间

        //printf("StartRunGCode:m_n_run_thread_state, %d\n", m_n_run_thread_state);
        if(m_n_run_thread_state == IDLE || m_n_run_thread_state == PAUSE){
            m_n_run_thread_state = RUN;  //置为运行状态
        }

        this->m_p_f_reg->OP = 1;   //自动运行状态

    }
    else if(m_channel_status.chn_work_mode == MDA_MODE){

        string msg = "开始执行MDA程序";
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug, msg);

        this->m_p_output_msg_list->Clear();	//清空缓冲数据
        this->InitMcIntpMdaBuf();  //清空MC中的MDA数据缓冲

        //初始化传递给MC模块的模态组信息
        this->InitMcModeStatus();

        this->m_p_compiler->SetCurPos(this->m_channel_rt_status.cur_pos_work);
        this->m_p_compiler->SetCurGMode();   //初始化编译器模态

        m_channel_rt_status.machining_time = 0;        //重置加工时间
        gettimeofday(&m_time_start_maching, nullptr);  //初始化启动时间

        this->SendMdaDataReqToHmi();  //发送MDA数据请求
        printf("send mda data req cmd to hmi, chn=%hhu\n", this->m_n_channel_index);
    }

END:
    m_n_step_change_mode_count = 0;

    this->SetMachineState(state);  //更新通道状态

    this->m_p_f_reg->SPL = 0;
    this->m_p_f_reg->STL = 1;

#ifdef USES_ADDITIONAL_PROGRAM
    if(add_type != NONE_ADD){
        this->CallAdditionalProgram(add_type);
    }
#endif

    printf("exit ChannelControl::StartRunGCode()\n");

}

/**
 * @brief 设置当前行号从MC模块获取
 */
void ChannelControl::SetCurLineNoFromMc(){

	if(this->m_n_macroprog_count == 0 || this->m_p_general_config->debug_mode > 0)  //调试模式下或者没有宏程序调用
        m_b_lineno_from_mc = true;
}

/**
 * @brief 启动MC开始插补
 */
void ChannelControl::StartMcIntepolate(){
    uint16_t mc_work_mode = MC_MODE_AUTO;    //默认自动模式

#ifdef USES_ADDITIONAL_PROGRAM
    if(this->m_channel_status.chn_work_mode == MDA_MODE ||
            this->m_b_manual_tool_measure ||
            this->m_b_manual_call_macro ||
            this->m_n_add_prog_type == CONTINUE_START_ADD)
#else
    if(this->m_channel_status.chn_work_mode == MDA_MODE ||
            this->m_b_manual_tool_measure ||
            this->m_b_manual_call_macro)
#endif
        mc_work_mode = MC_MODE_MDA;
    else if(this->m_channel_status.chn_work_mode != AUTO_MODE)
        return;

    printf("----------> mc_work_mode %d\n", mc_work_mode);
    this->SendIntpModeCmdToMc(mc_work_mode);

    SetCurLineNoFromMc();	//当前行号从MC获取

    m_b_mc_need_start = false; //防止重复发送start

    gettimeofday(&m_time_start_mc, nullptr);  //更新MC启动时间
    printf("@@@@@@@StartMcIntepolate\n");
}

/**
 * @brief 暂停G代码运行
 */
void ChannelControl::PauseRunGCode(){
    uint8_t state = MS_PAUSING;

    if(this->m_channel_status.chn_work_mode == AUTO_MODE){
        string msg = "暂停加工程序(" + string(this->m_channel_status.cur_nc_file_name) + ")";
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug, msg);
        if(this->m_simulate_mode != SIM_NONE &&
                (m_channel_status.machining_state == MS_OUTLINE_SIMULATING ||
                 m_channel_status.machining_state == MS_TOOL_PATH_SIMULATING ||
                 m_channel_status.machining_state == MS_MACHIN_SIMULATING)){  //仿真直接结束
            m_n_run_thread_state = STOP;
            if(m_channel_status.machining_state == MS_MACHIN_SIMULATING){  //加工仿真
                //向MC模块发送停止命令
            	this->PauseMc();
            }
            m_n_subprog_count = 0;
            m_n_macroprog_count = 0;
            m_b_ret_from_macroprog = false;
            m_b_init_compiler_pos = false;  //编译器初始位置需要重新初始化

            this->m_p_compiler->Reset();
            this->m_p_output_msg_list->Clear();
#ifdef 	USES_SIMULATION_TEST
            if(this->m_file_sim_data > 0){
                close(this->m_file_sim_data);
                m_file_sim_data = -1;
            }
#endif

            this->SetCurLineNo(1);

            this->m_p_f_reg->STL = 0;
            this->m_p_f_reg->SPL = 0;

            this->SetMachineState(MS_READY);  //更新通道状态

            return;
        }

        if(this->m_channel_status.machining_state != MS_RUNNING
        #ifdef USES_ADDITIONAL_PROGRAM
                || this->m_n_add_prog_type != NONE_ADD
        #endif
                ){ //非运行状态,或者附加程序运行期间
            return;
        }

        if(m_n_run_thread_state == RUN ){	//只有RUN状态，运行线程才需要PAUSE
            m_n_run_thread_state = PAUSE;
        }

        if(this->m_mask_run_pmc){   //暂停PMC轴
            this->PausePmcAxis(NO_AXIS, true);
            state = MS_PAUSED;
        }else{
            //向MC模块发送暂停指令
        	this->PauseMc();
        }

        this->m_p_f_reg->STL = 0;
        this->m_p_f_reg->SPL = 1;

    }
    else if(m_channel_status.chn_work_mode == MDA_MODE){

        m_n_run_thread_state = STOP;
        this->m_p_compiler->Reset();

        state = MS_STOPPING;	//停止过程中，在判断MC模块各轴停下来后置为MS_STOP状态

        if(this->m_mask_run_pmc){   //停止PMC轴
            this->StopPmcAxis(NO_AXIS);
            state = MS_PAUSED;
        }else{
            //向MC模块发送停止命令
        	this->PauseMc();
        }

        this->m_p_output_msg_list->Clear();

        this->m_p_f_reg->STL = 0;
        this->m_p_f_reg->SPL = 0;

    }else
        return;

    if(m_simulate_mode != SIM_NONE)
        state = MS_READY;   //仿真模式直接置为READY

    this->SetMachineState(state);  //更新通道状态
}

/**
 * @brief 停止编译,将编译线程置为IDLE状态
 */
void ChannelControl::StopCompilerRun(){
    printf("stop compiler run\n");

    // @test zk
    pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止
    //printf("locked 1\n");
    m_n_run_thread_state = STOP;
    pthread_mutex_unlock(&m_mutex_change_state);
    //printf("unlocked 1\n");
}

/**
 * @brief 停止G代码运行
 * @param reset : 是否复位数据和行号， true--复位   false--不复位
 */
void ChannelControl::StopRunGCode(bool reset){
    string msg = "停止加工程序(" + string(this->m_channel_status.cur_nc_file_name) + ")";
    g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug, msg);
    printf("ChannelControl::StopRunGCode()\n");

    if(this->m_channel_status.machining_state == MS_READY && !m_b_cancel_manual_call_macro)  //空闲状态直接返回
        return;

    uint8_t state = MS_STOPPING;

    if(this->m_thread_breakcontinue > 0){//处于断点继续线程执行过程中，则退出断点继续线程
        this->CancelBreakContinueThread();
    }

    this->StopCompilerRun(); //停止编译

    //TODO 向MC模块发送停止命令
    this->PauseMc();

    if(m_simulate_mode != SIM_NONE || this->m_channel_status.machining_state == MS_PAUSED || m_b_cancel_manual_call_macro)
        state = MS_READY;

    int count = 0;
    while(this->m_channel_status.machining_state != MS_READY &&
          this->m_channel_status.machining_state != MS_WARNING){//等待停止到位
        if(++count > 200)
            break;
        usleep(100);
    }

    m_n_subprog_count = 0;
    m_n_macroprog_count = 0;
    m_b_ret_from_macroprog = false;
    m_b_init_compiler_pos = false;  //编译器初始位置需要重新初始化
    this->m_p_compiler->Reset();

    if(this->m_channel_status.chn_work_mode == AUTO_MODE
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type != CONTINUE_START_ADD
        #endif
            )
        this->m_scene_auto.need_reload_flag = false;   //取消断点继续标志

    if(reset){
        this->m_macro_variable.Reset();   //宏变量复位
        this->ResetMcLineNo();
        if(this->m_channel_status.chn_work_mode == AUTO_MODE
        #ifdef USES_ADDITIONAL_PROGRAM
                && this->m_n_add_prog_type != CONTINUE_START_ADD
        #endif
                ){
            this->InitMcIntpAutoBuf();
        }else if(m_channel_status.chn_work_mode >= MDA_MODE){   //包含手动模式
            this->InitMcIntpMdaBuf();
        }

        m_p_output_msg_list->Clear();
        //m_p_output_msg_list_auto->Clear();
        //m_p_output_msg_list_mda->Clear();

        SetCurLineNo(1);
    }

    if(this->IsStepMode()){
        this->SetMcStepMode(true);
    }

    this->SetMachineState(state);  //更新通道状态

    //this->m_p_f_reg->SPL = 0;
    this->m_p_f_reg->STL = 0;
}


/**
 * @brief 暂停MC模块的G代码插补，即将工作模式切换到手动
 */
void ChannelControl::PauseMc(){
    //更新当前MC的运行模式
    if(!this->m_b_mc_on_arm)
        this->m_p_mc_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);
    else
        this->m_p_mc_arm_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);

	// @test
	printf("pausemc m_channel_mc_status.cur_mode: %d\n", m_channel_mc_status.cur_mode);

//	printf("pausemc, mc_mode = %hu\n", m_channel_mc_status.cur_mode);
	if(m_channel_mc_status.cur_mode == MC_MODE_MANUAL ){//停止所有轴
		uint16_t mask = 0;
		for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
			mask |= (0x01<<i);
		}
		this->ManualMoveStop(mask);
	}else
		// @bug  部分SC这个指令发送无效 导致 SC假死  PMC能亮 但无法动作
		this->SendIntpModeCmdToMc(MC_MODE_MANUAL);
}

/**
 * @brief 向MC发送G31停止命令,即设置通道停止运行并置运行到位
 */
void ChannelControl::SendMcG31Stop(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = this->m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_STOP_G31_CMD;
    if(this->m_channel_status.chn_work_mode == AUTO_MODE)
        cmd.data.data[0] = 1;   //停止
    else
        cmd.data.data[0] = 2;  //非自动模式

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}


///**
// * @brief 向MC发送刚性攻丝状态命令，MC会切换到刚性攻丝的速度规划参数
// */
//void ChannelControl::SendMcRigidTapFlag(bool tap_flag){
//	McCmdFrame cmd;
//	memset(&cmd, 0x00, sizeof(McCmdFrame));

//	cmd.data.channel_index = this->m_n_channel_index;
//	cmd.data.axis_index = NO_AXIS;
//	cmd.data.cmd = CMD_MC_SET_G84_INTP_MODE;
//	cmd.data.data[0] = tap_flag?1:0;

//	if(!this->m_b_mc_on_arm)
//		m_p_mc_comm->WriteCmd(cmd);
//	else
//		m_p_mc_arm_comm->WriteCmd(cmd);
//}


/**
 * @brief 发送MC复位指令
 */
void ChannelControl::SendMcResetCmd(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));
    cmd.data.axis_index = NO_AXIS;
    cmd.data.channel_index = this->m_n_channel_index;

    cmd.data.cmd = CMD_MC_SYS_RESET;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 通知MC进行通道MDA插补缓冲初始化
 */
void ChannelControl::InitMcIntpMdaBuf(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = this->m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_INIT_CHN_MDA_BUF;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 通知MC进行通道自动数据缓冲初始化
 */
void ChannelControl::InitMcIntpAutoBuf(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = this->m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_INIT_CHN_AUTO_BUF;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 空闲处理函数
 * @param count : 空闲周期计数
 */
void ChannelControl::DoIdle(uint32_t count){

    if(count%150 == 0 && (!this->IsMachinRunning())
            && this->m_macro_variable.IsKeepChanged()){//同步PMC寄存器
        this->m_macro_variable.Sync();
    }

#ifdef USES_SPEED_TORQUE_CTRL
    //轴控制模式切换
    if(this->m_n_need_reset_axis_ctrlmode == 1){
        printf("---m_n_need_reset_axis_ctrlmode=%hhu \n",m_n_need_reset_axis_ctrlmode);
        this->m_n_need_reset_axis_ctrlmode = 0;
        if(this->ResetAllAxisCtrlMode(0)){  //切换失败
            this->m_error_code = ERR_AXIS_CTRL_MODE_SWITCH;
            CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            printf("---reset axis_ctrlmode succeed! \n");
        }else{
            printf("---reset axis_ctrlmode failed! \n");
        }
    }else if(this->m_n_need_reset_axis_ctrlmode > 1){
        printf("---m_n_need_reset_axis_ctrlmode=%hhu \n",m_n_need_reset_axis_ctrlmode);
        if(false == this->ResetAllAxisCtrlMode(0)){
            this->m_n_need_reset_axis_ctrlmode = 0;
            printf("---reset axis_ctrlmode succeed! \n");
        }else{
            this->m_n_need_reset_axis_ctrlmode --;
            printf("---reset axis_ctrlmode failed! \n");
        }
    }
#endif

    this->m_p_compiler->DoIdle();

}

/**
 * @brief 将MC的各轴插补位置刷新到通道实时状态中
 */
void ChannelControl::RefreshAxisIntpPos(){
    int count = 0;
    for(int i = 0; i < m_p_channel_config->chn_axis_count && count < 8; i++){
        if(g_ptr_chn_engine->GetPmcActive(this->GetPhyAxis(i))) {
            m_channel_rt_status.cur_pos_work.m_df_point[i] = m_channel_rt_status.cur_pos_machine.m_df_point[i];   //pmc轴的工件坐标同机械坐标
            m_channel_rt_status.tar_pos_work.m_df_point[i] = m_channel_rt_status.cur_pos_machine.m_df_point[i] + m_p_channel_engine->GetPmcAxisRemain(i);   //pmc轴余移动量从mi读取
            count++;
            continue;
        }
        m_channel_rt_status.cur_pos_work.m_df_point[i] = m_channel_mc_status.intp_pos.GetAxisValue(count);
        // m_channel_rt_status.tar_pos_work.m_df_point[i] = m_channel_mc_status.intp_tar_pos.GetAxisValue(count);

        // 暂停或准备状态，将目标位置的值改为当前位置
        // 备注：这么改是为了解决复位/暂停后还保留余移动量的问题
        // 后面如果MC解决该问题，再改回去
        if(m_channel_status.machining_state == MS_READY)
        {
            m_channel_rt_status.tar_pos_work.m_df_point[i] = m_channel_rt_status.cur_pos_work.m_df_point[i];
        }else{
            m_channel_rt_status.tar_pos_work.m_df_point[i] = m_channel_mc_status.intp_tar_pos.GetAxisValue(count);
        }
        count++;
    }
}

/**
 * @brief 同步保存宏变量
 */
void ChannelControl::SyncMacroVar(){
    if(this->m_macro_variable.IsKeepChanged()){//同步PMC寄存器
        this->m_macro_variable.Sync();
    }
}


/**
 * @brief 发送监控数据
 * @param bAxis : 是否发送坐标轴实时位置
 * @param btime : 是否更新加工时间
 */
void ChannelControl::SendMonitorData(bool bAxis, bool btime){
    HmiChannelRealtimeStatus hmi_rt_status;
    //hmi_rt_status.cur_pos = this->m_channel_rt_status.cur_pos;
    //srand((unsigned int)time(nullptr));
    //int inc = rand()%20;
    //double inc_d = ((double)inc)/1000.;


    //	if(m_channel_rt_status.cur_feed != m_channel_mc_status.cur_feed){
    //		printf("current feed : %d um/s\n", m_channel_mc_status.cur_feed);
    //	}

    //更新通道轴位置
    double *pos = m_channel_rt_status.cur_pos_machine.m_df_point;
    double *speed = m_channel_rt_status.cur_feedbck_velocity.m_df_point;
    double *torque = m_channel_rt_status.cur_feedbck_torque.m_df_point;
    double *angle = m_channel_rt_status.spd_angle.m_df_point;
    uint8_t phy_axis = 0; //通道轴所对应的物理轴
    for(uint8_t i =0; i < this->m_p_channel_config->chn_axis_count; i++){
        phy_axis = this->m_p_channel_config->chn_axis_phy[i]; // phy_axis = this->m_channel_status.cur_chn_axis_phy[i];
        if(phy_axis > 0){
        	pos[i] = this->m_p_channel_engine->GetPhyAxisMachPosFeedback(phy_axis-1);
            speed[i] = this->m_p_channel_engine->GetPhyAxisMachSpeedFeedback(phy_axis-1);
            torque[i] = this->m_p_channel_engine->GetPhyAxisMachTorqueFeedback(phy_axis-1);
            angle[i] = this->m_p_channel_engine->GetSpdAngleFeedback(phy_axis-1);
        }
    }

    if(btime)
    {
        if (this->m_channel_status.machining_state == MS_RUNNING){  //更新加工计时
            struct timeval cur_time;
            gettimeofday(&cur_time, nullptr);
            unsigned int time = cur_time.tv_sec-m_time_start_maching.tv_sec;
            m_channel_rt_status.machining_time += time;
            m_channel_rt_status.machining_time_total += time;

            m_time_remain -= (int32_t)time;    //剩余加工时间
            m_time_remain > 0 ? m_channel_rt_status.machining_time_remains = m_time_remain
                    : m_channel_rt_status.machining_time_remains = 0;

            m_time_start_maching = cur_time;
        }
        if (this->m_channel_status.machining_state == MS_READY) { //停止加工
            if (m_channel_rt_status.machining_time != 0) {
                m_time_remain = m_channel_rt_status.machining_time;
                m_channel_rt_status.machining_time = 0;
                g_ptr_parm_manager->SetCurTotalMachiningTime(m_n_channel_index, m_channel_rt_status.machining_time_total);
            }
            if (m_channel_rt_status.machining_time_remains != 0)
                m_channel_rt_status.machining_time_remains = 0;
        }
    }

    m_channel_rt_status.cur_feed = m_channel_mc_status.cur_feed;
    //	m_channel_rt_status.machining_time = m_channel_mc_status.auto_block_over;
    //	m_channel_rt_status.machining_time = (uint32_t)this->m_n_run_thread_state;  //for test  ,线程状态
    //	m_channel_rt_status.machining_time_remains = m_channel_mc_status.buf_data_count;
    //	m_channel_rt_status.machining_time_remains = this->m_n_send_mc_data_err;    //for test ,数据发送失败次数

    //@test zk  MDA 模式下显示行号
    if(m_channel_mc_status.cur_mode == MC_MODE_MDA &&
            m_channel_mc_status.cur_line_no > 0 &&
            !m_b_manual_call_macro/*llx add 宏程序调用不能显示行号*/){ //没有调用宏程序
    	m_channel_rt_status.line_no = m_channel_mc_status.cur_line_no;
    }

    if(m_channel_mc_status.cur_mode == MC_MODE_AUTO &&
        #ifdef USES_ADDITIONAL_PROGRAM
            m_n_add_prog_type == NONE_ADD &&
        #endif
            m_b_lineno_from_mc && m_channel_mc_status.cur_line_no > 0){
        m_channel_rt_status.line_no = m_channel_mc_status.cur_line_no;
    }


    RefreshAxisIntpPos();   //刷新工件坐标系当前位置及目标位置

    //	// 调试螺补用  FOR TEST
    //	for(uint8_t i =0; i < this->m_p_channel_config->chn_axis_count; i++){
    //		phy_axis = this->m_p_channel_config->chn_axis_phy[i]; // phy_axis = this->m_channel_status.cur_chn_axis_phy[i];
    //		if(phy_axis > 0){
    //			m_channel_rt_status.cur_pos_work.m_df_point[i] = this->m_p_channel_engine->GetPhyAxisMachPosIntp(phy_axis-1);
    //		}
    //	}
    //	//end for test

    if(this->m_n_spindle_count > 0){
        m_channel_rt_status.spindle_cur_speed = m_p_spindle->GetSpindleSpeed();
    }

    //处于刚攻状态需要读取刚攻误差
    if(m_p_g_reg->RGTAP == 1){
        m_p_mi_comm->ReadTapErr(&m_channel_rt_status.tap_err,
                                &m_channel_rt_status.tap_err_now,
                                1);
    }
    m_channel_rt_status.spd_angle = m_p_spindle->GetSpdAngle();

    //	printf("axis pos %lf, %lf, %lf\n", pos[0], pos[1], pos[2]);

    uint8_t data_type = MDT_CHN_RT_STATUS;
    uint8_t chn_index = this->m_n_channel_index;
    uint16_t data_len = sizeof(HmiChannelRealtimeStatus);
    memcpy(&hmi_rt_status, &m_channel_rt_status, data_len);
    if(this->m_channel_status.chn_work_mode != AUTO_MODE && this->m_channel_status.chn_work_mode != MDA_MODE){//手动模式下，余移动流量显示0
        memcpy(hmi_rt_status.tar_pos_work, hmi_rt_status.cur_pos_work, sizeof(double)*kMaxAxisChn);
    }

    //	memcpy(hmi_rt_status.cur_pos_machine, m_channel_rt_status.cur_pos_machine.m_df_point, sizeof(double)*kMaxAxisChn);
    //	memcpy(hmi_rt_status.cur_pos_work, m_channel_rt_status.cur_pos_work.m_df_point, sizeof(double)*kMaxAxisChn);
    //	memcpy(hmi_rt_status.tar_pos_work, m_channel_rt_status.tar_pos_work.m_df_point, sizeof(double)*kMaxAxisChn);
    //	hmi_rt_status.cur_feed = m_channel_rt_status.cur_feed;
    //	hmi_rt_status.line_no = m_channel_rt_status.line_no;
    //	hmi_rt_status.machining_time = m_channel_rt_status.machining_time;
    //	hmi_rt_status.machining_time_remains = m_channel_rt_status.machining_time_remains;
    //	hmi_rt_status.spindle_cur_speed = m_channel_rt_status.spindle_cur_speed;

    int buf_len = 4+data_len;
    char buffer[512];
    memset(buffer, 0x00, 512);
    memcpy(buffer, &data_type, 1);
    memcpy(buffer+1, &chn_index, 1);
    memcpy(buffer+2, &data_len, 2);
    memcpy(buffer+4, &hmi_rt_status, data_len);

    this->m_p_hmi_comm->SendMonitorData(buffer, buf_len);//send(socket, buffer, buf_len, MSG_NOSIGNAL);

    if(bAxis && this->m_channel_status.chn_work_mode == AUTO_MODE && this->m_b_hmi_graph){
        this->SendHmiGraphPosData();  //发送绘图位置数据
    }

    //	double inc_d = 0.001;
    //	m_channel_rt_status.cur_pos += DPoint(inc_d, inc_d, inc_d,inc_d,inc_d, inc_d,inc_d, inc_d);

}

/**
 * @brief 发送仿真数据给HMI
 * @param type : 数据类型，轮廓仿真、刀路仿真
 * @param data
 */
void ChannelControl::SendSimulateDataToHmi(MonitorDataType type, SimulateData &data){
    if(type != MDT_CHN_SIM_GRAPH && type != MDT_CHN_SIM_TOOL_PATH){
        g_ptr_trace->PrintLog(LOG_ALARM, "仿真数据类型非法[%d]", type);
        return;
    }
#ifdef USES_SIMULATION_TEST
    if(this->m_file_sim_data > 0){
        ssize_t res = write(m_file_sim_data, &data.type, sizeof(int));
        if(res != 4){
            g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "写入仿真数据类型错误！res=%d", res);
        }
        res = write(m_file_sim_data, data.pos, sizeof(double)*3);
        if(res != 24){
            g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "写入仿真数据位置错误！res=%d", res);
        }
    }else{
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "仿真数据文件句柄为空！");
    }
#else

    uint8_t data_type = type;
    uint8_t chn_index = this->m_n_channel_index;
    uint16_t data_len = sizeof(SimulateData);

    int buf_len = 4+data_len;
    char buffer[64];
    memset(buffer, 0x00, 64);
    memcpy(buffer, &data_type, 1);
    memcpy(buffer+1, &chn_index, 1);
    memcpy(buffer+2, &data_len, 2);
    memcpy(buffer+4, &data, data_len);

    this->m_p_hmi_comm->SendMonitorData(buffer, buf_len);
#endif
}

/**
 * @brief 处理HMI模块发送的命令
 * @param cmd ：待处理的HMI命令包
 */
void ChannelControl::ProcessHmiCmd(HMICmdFrame &cmd){

    switch(cmd.cmd){
    case CMD_HMI_GET_CHN_STATE:   //HMI获取通道当前状态
        ProcessHmiGetChnStateCmd(cmd);
        break;
    case CMD_HMI_RESTART:               //加工复位 11
        this->ProcessHmiRestartCmd(cmd);
        break;
    case CMD_HMI_SIMULATE:				//仿真 12
        this->ProcessHmiSimulateCmd(cmd);
        break;
    case CMD_HMI_SET_NC_FILE:	    	//设置当前加工文件 13
        this->ProcessHmiSetNcFileCmd(cmd);
        break;
    case CMD_HMI_FIND_REF_POINT:		//确定参考点 14
        break;
    case CMD_HMI_SET_REF_POINT:			//设置参考点
        this->ProcessHmiSetRefCmd(cmd);
        break;
    case CMD_SC_MDA_DATA_REQ:		//MDA代码请求 115
        this->ProcessMdaData(cmd);
        break;
    case CMD_HMI_GET_MACRO_VAR:			//HMI向SC请求宏变量的值   30
        this->ProcessHmiGetMacroVarCmd(cmd);
        break;
    case CMD_HMI_SET_MACRO_VAR:        //HMI向SC设置宏变量寄存器的值   31
        this->ProcessHmiSetMacroVarCmd(cmd);
        break;

    case CMD_HMI_CLEAR_WORKPIECE:      //HMI请求SC将加工计数清零,临时计数(区分白夜班)
        this->ProcessHmiClearWorkPieceCmd(cmd);
        this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);   //通知HMI加工计数变更
        break;
#ifdef USES_LASER_MACHINE
    case CMD_HMI_SET_CALIBRATION:     //HMI向SC发出调高器标定指令 32
        cocopark
                this->ProcessHmiCalibCmd(cmd.data[0]);
        break;
#endif
    case CMD_HMI_CLEAR_TOTAL_PIECE:    //总共计数清零
        this->ProcessHmiClearTotalPieceCmd(cmd);
        this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);   //通知HMI加工计数变更
        break;
    case CMD_HMI_SET_REQUIRE_PIECE:     //设置需求件数
        this->ProcessHmiSetRequirePieceCmd(cmd);
        break;
    case CMD_HMI_AXIS_MANUAL_MOVE:     //HMI指令轴移动
        break;
    case CMD_HMI_MANUAL_TOOL_MEASURE:     //HMI向SC发起手动对刀操作  0x29
        this->ProcessHmiManualToolMeasureCmd(cmd);
        break;
    case CMD_HMI_SET_CUR_MACH_POS:        //HMI向SC重设指定轴的机械坐标  0x32
        this->ProcessHmiSetCurMachPosCmd(cmd);
        break;
    case CMD_HMI_CLEAR_MSG:               //HMI通知SC清除消息  0x33
        //		printf("channelcontrol process CMD_HMI_CLEAR_MSG, cmd_ex=%hhu\n", cmd.cmd_extension);
        this->ProcessHmiClearMsgCmd(cmd);
        break;
    default:

        break;

    }
}

/**
 * @brief 处理回参考点命令
 * @param cmd : HMI命令包
 */
void ChannelControl::ProcessHmiFindRefCmd(HMICmdFrame &cmd){


}

/**
 * @brief 处理获取宏变量命令
 * @param cmd
 */
void ChannelControl::ProcessHmiGetMacroVarCmd(HMICmdFrame &cmd){
    //std::cout << "ChannelControl::ProcessHmiGetMacroVarCmd" << std::endl;
    cmd.frame_number |= 0x8000;   //设置回复标志

    uint32_t start_index = 0;   //起始编号
    uint8_t count = 0;			//变量个数

    if(cmd.data_len != 5){	//数据长度不合法
        cmd.cmd_extension = FAILED;
        this->m_p_hmi_comm->SendCmd(cmd);
        g_ptr_trace->PrintLog(LOG_ALARM, "ChannelControl::ProcessHmiSetMacroVarCmd()数据长度不合法，data_len = %hu！", cmd.data_len);
        return;
    }

    memcpy(&start_index, &cmd.data[0], 4);
    memcpy(&count, &cmd.data[4], 1);

    //拷贝数据
    int len = this->m_macro_variable.CopyVar(&cmd.data[cmd.data_len], 1000, start_index, count);
    //std::cout << "Get len: " << len << std::endl;
    if(0 == len){
        cmd.cmd_extension = FAILED;
        std::cout << "Send Failed" << std::endl;
        this->m_p_hmi_comm->SendCmd(cmd);
        return;
    }

    cmd.data_len += len;
    cmd.cmd_extension = SUCCEED;

    this->m_p_hmi_comm->SendCmd(cmd);
}


/**
 * @brief 处理设置宏变量命令
 * @param cmd : HMI命令包
 */
void ChannelControl::ProcessHmiSetMacroVarCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;   //设置回复标志

    uint32_t start_index = 0;   //起始编号
    uint8_t count = 0;			//变量个数

    if(this->m_p_channel_engine->IsParamImported(CONFIG_MACRO_VAR)){
        cmd.cmd_extension = FAILED;
        this->m_p_hmi_comm->SendCmd(cmd);
        g_ptr_trace->PrintLog(LOG_ALARM, "已导入外部宏变量，请先重新启动！");
        return;
    }
    if(cmd.data_len < 5){	//数据长度不合法
        cmd.cmd_extension = FAILED;
        this->m_p_hmi_comm->SendCmd(cmd);
        g_ptr_trace->PrintLog(LOG_ALARM, "ChannelControl::ProcessHmiSetMacroVarCmd()数据长度不合法，data_len = %hu！", cmd.data_len);
        return;
    }
    memcpy(&start_index, &cmd.data[0], 4);
    memcpy(&count, &cmd.data[4], 1);

    if(count > 100 || cmd.data_len != 5+(sizeof(double)+1)*count){ //数据长度不合法
        cmd.cmd_extension = FAILED;
        this->m_p_hmi_comm->SendCmd(cmd);
        g_ptr_trace->PrintLog(LOG_ALARM, "ChannelControl::ProcessHmiSetMacroVarCmd()数据长度不合法，data_len = %hu, count= %hhu！",
                              cmd.data_len, count);
        return;
    }

    //拷贝数据
    bool init[count];
    double value[count];
    memcpy(init, &cmd.data[5], count);
    memcpy(value, &cmd.data[5+count], sizeof(double)*count);

    for(int i = 0; i < count; i++){
        if(init[i]){
            this->m_macro_variable.SetVarValue(start_index+i, value[i]);
            //printf("set macro var: %u, %lf\n", start_index+i, value[i]);
        }else{
            this->m_macro_variable.ResetVariable(start_index+i);
            //printf("reset macro var: %u\n", start_index+i);
        }
    }

    cmd.data_len = 5;

    cmd.cmd_extension = SUCCEED;

    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 处理加工计数清零命令
 * @param cmd : HMI命令包
 */
void ChannelControl::ProcessHmiClearWorkPieceCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;   //设置回复标志

    this->m_channel_status.workpiece_count = 0;
    cmd.cmd_extension = SUCCEED;

    this->m_p_hmi_comm->SendCmd(cmd);

    g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
    this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);   //通知HMI加工计数变更
}

/**
 * @brief 处理总共件数清零命令
 * @param cmd : HMI命令包
 */
void ChannelControl::ProcessHmiClearTotalPieceCmd(HMICmdFrame &cmd)
{
    cmd.frame_number |= 0x8000;   //设置回复标志

    this->m_channel_status.workpiece_count = 0;
    this->m_channel_status.workpiece_count_total = 0;
    cmd.cmd_extension = SUCCEED;

    this->m_p_hmi_comm->SendCmd(cmd);

    g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
    g_ptr_parm_manager->SetTotalWorkPiece(m_n_channel_index, m_channel_status.workpiece_count_total);
    this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);   //通知HMI加工计数变更
}

/**
 * @brief 处理加工复位命令
 * @param cmd : HMI命令包
 */
void ChannelControl::ProcessHmiRestartCmd(HMICmdFrame &cmd){
    //读取复位行号
    uint64_t line = 0;
    if(this->m_channel_status.chn_work_mode != AUTO_MODE ||
            this->m_channel_status.machining_state == MS_WARNING){  //非自动模式，或者告警状态，不可以加工复位
        cmd.cmd_extension = FAILED;
    }else if(cmd.data_len == 8){
        memcpy(&line, cmd.data, sizeof(line));
        printf("ProcessHmiRestartCmd, restart line = %llu\n", line);
        cmd.cmd_extension = SUCCEED;
    }else{
        printf("ProcessHmiRestartCmd, error data in data_len:%hu\n", cmd.data_len);
        cmd.cmd_extension = FAILED;
    }


    //首先发送响应消息
    cmd.frame_number |= 0x8000;
    cmd.channel_index = this->m_n_channel_index;
    //	cmd.cmd_extension = SUCCEED;
    cmd.data_len = 0;
    this->m_p_hmi_comm->SendCmd(cmd);


    if(cmd.cmd_extension == SUCCEED){ //可以执行加工复位
        if(this->m_channel_status.machining_state != MS_READY){
            this->StopRunGCode();  //停止当前运行
        }

        //设置加工复位的状态量
        this->m_n_restart_line = line;
        this->m_n_restart_step = 1;
        this->m_n_restart_mode = NORMAL_RESTART;

        m_channel_rt_status.line_no = m_n_restart_line;
    }

}

/**
 * @brief 设置需求件数个数
 * @param cmd : HMI命令包
 */
void ChannelControl::ProcessHmiSetRequirePieceCmd(HMICmdFrame &cmd)
{
    cmd.frame_number |= 0x8000;

    memcpy(&m_channel_status.workpiece_require, cmd.data, cmd.data_len);
    std::cout << "setCurRequired piece " << m_channel_status.workpiece_require << std::endl;
    g_ptr_parm_manager->SetCurRequirePiece(m_n_channel_index, m_channel_status.workpiece_require);

    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 处理手动对刀指令
 * @param cmd : HMI命令包
 */
void ChannelControl::ProcessHmiManualToolMeasureCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;   //设置回复标志

    if(this->m_channel_status.chn_work_mode >= MANUAL_STEP_MODE ){
        if(cmd.cmd_extension == 0x00){  //请求执行手动对刀
            if(cmd.data_len != 2*sizeof(int) ||
                    this->m_b_manual_tool_measure ||
                    this->m_b_cancel_manual_tool_measure){
                cmd.cmd_extension = REFUSE;
                this->m_p_hmi_comm->SendCmd(cmd);  //回复响应命令
            }else{
                cmd.cmd_extension = APPROVE;

                this->m_p_hmi_comm->SendCmd(cmd);  //回复响应命令

                int h_code = 0;
                int times = 0;
                memcpy(&h_code, cmd.data, sizeof(int));
                memcpy(&times, cmd.data+sizeof(int), sizeof(int));
                this->ManualToolMeasure(h_code, times);
            }

        }else if(cmd.cmd_extension == 0x01){   //请求取消手动对刀
            cmd.cmd_extension = APPROVE;

            if(m_b_manual_tool_measure && !m_b_cancel_manual_tool_measure){
                //取消对刀
                m_b_cancel_manual_tool_measure = true;
                this->StopRunGCode(true);
            }

            this->m_p_hmi_comm->SendCmd(cmd);  //回复响应命令
        }
    }else{
        cmd.cmd_extension = REFUSE;
        this->m_p_hmi_comm->SendCmd(cmd);  //回复响应命令
    }

}

/**
 * @brief 处理HMI设置轴当前位置的机械坐标命令
 * @param cmd : HMI命令包
 */
void ChannelControl::ProcessHmiSetCurMachPosCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;   //设置回复标志

#ifdef USES_WOOD_MACHINE
    if(cmd.data_len != 9 || this->m_channel_status.chn_work_mode != REF_MODE){  //重设机械零点只能在原点模式下进行
#else
    if(cmd.data_len != 9){
#endif	
        cmd.data[9] = FAILED;
        cmd.data_len = 10;
    }else{
        uint8_t chn_axis = cmd.data[0];
        double mach_pos = 0;
        memcpy(&mach_pos, &cmd.data[1], sizeof(double));

        uint8_t phy_axis = this->GetPhyAxis(chn_axis);
        if(phy_axis == 0xff){
            cmd.data[9] = FAILED;
            cmd.data_len++;
        }else{
            //向MI发送设置命令
            int32_t pos = mach_pos*1000;   //单位转换，mm->um

            MiCmdFrame mi_cmd;
            memset(&mi_cmd, 0x00, sizeof(mi_cmd));
            mi_cmd.data.cmd = CMD_MI_SET_AXIS_MACH_POS;
            mi_cmd.data.axis_index = phy_axis+1;


            mi_cmd.data.data[0] = pos&0xFFFF;
            mi_cmd.data.data[1] = (pos>>16)&0xFFFF;

            this->m_p_mi_comm->WriteCmd(mi_cmd);

            cmd.data[9] = SUCCEED;
            cmd.data_len++;
        }
    }

    this->m_p_hmi_comm->SendCmd(cmd);  //回复响应命令
}

/**
 * @brief 处理HMI清除消息命令
 * @param cmd : HMI指令包
 */
void ChannelControl::ProcessHmiClearMsgCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;
    cmd.data_len = 0;



    if(cmd.cmd_extension == 0xFF){  //清空告警队列
        g_ptr_alarm_processor->Clear();
        cmd.cmd_extension = SUCCEED;
    }else if(cmd.cmd_extension == 0xEE){  //清除警告及提示信息
        g_ptr_alarm_processor->ClearWarning(this->m_n_channel_index);
        cmd.cmd_extension = SUCCEED;
    }else{
        cmd.cmd_extension = FAILED;
    }

    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 处理设置参考点命令
 * @param cmd
 */
void ChannelControl::ProcessHmiSetRefCmd(HMICmdFrame &cmd){

    uint8_t cur_axis_mac = this->GetCurPhyAxis();		//当前通道轴对应的机械轴号

    if(cur_axis_mac != 0){
        this->SetAxisRefCur(cur_axis_mac);	//发送获取当前编码器值得命令

        //置回参考点标志
        m_channel_status.returned_to_ref_point |= (0x01<<m_channel_status.cur_axis);
        this->m_p_channel_engine->SetRetRefFlag(cur_axis_mac-1, true);
        this->SendChnStatusChangeCmdToHmi(REF_POINT_FLAG);
    }
    else
        printf("当前物理轴未配置！\n");

    printf("process set ref cmd, %d\n", cur_axis_mac);

    cmd.data[0] = SUCCEED;
    cmd.data_len = 0x01;
    cmd.frame_number |= 0x8000;
    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 返回当前通道轴对应的物理轴号
 * @return 返回当前的物理轴索引号，从1开始，0表示未配置
 */
uint8_t ChannelControl::GetCurPhyAxis(){
    return m_p_channel_config->chn_axis_phy[m_channel_status.cur_axis];  //return this->m_channel_status.cur_chn_axis_phy[m_channel_status.cur_axis];
}


/**
 * @brief 获取指定轴的当前编码器值，此处只发送获取命令
 * @param axis ： 物理轴编号, 从1开始
 *
 */
void ChannelControl::SetAxisRefCur(uint8_t axis){
    m_p_mi_comm->SetAxisRefCur(axis,m_p_axis_config[axis-1].axis_home_pos[0]);
}

/**
 * @brief 设置当前工艺参数组号
 * @param index : 工艺参数组号,并切换工艺参数
 * @return true--成功    false--失败，设定值超过合理范围
 */
bool ChannelControl::SetCurProcParamIndex(uint8_t index){
    if(index < kMaxProcParamCount){
        if(index == m_n_cur_proc_group)  //相同则直接退出
            return true;

        this->m_n_cur_proc_group = index;
        g_ptr_parm_manager->SetCurProcParamIndex(m_n_channel_index, index);
        this->m_p_f_reg->PPI = m_n_cur_proc_group;


        //切换通道参数
        this->m_p_channel_config->rapid_mode = this->m_p_chn_proc_param->chn_param[index].rapid_mode;
        this->m_p_channel_config->cut_plan_mode = this->m_p_chn_proc_param->chn_param[index].cut_plan_mode;
        this->m_p_channel_config->rapid_plan_mode = this->m_p_chn_proc_param->chn_param[index].rapid_plan_mode;
        this->m_p_channel_config->corner_acc_limit = this->m_p_chn_proc_param->chn_param[index].corner_acc_limit;
        this->m_p_channel_config->corner_stop = this->m_p_chn_proc_param->chn_param[index].corner_stop;
        this->m_p_channel_config->corner_stop_angle_min = this->m_p_chn_proc_param->chn_param[index].corner_stop_angle_min;
        this->m_p_channel_config->corner_stop_enable = this->m_p_chn_proc_param->chn_param[index].corner_stop_enable;
        this->m_p_channel_config->chn_spd_limit_on_acc = this->m_p_chn_proc_param->chn_param[index].chn_spd_limit_on_acc;
        this->m_p_channel_config->chn_spd_limit_on_axis = this->m_p_chn_proc_param->chn_param[index].chn_spd_limit_on_axis;
        this->m_p_channel_config->chn_spd_limit_on_curvity = this->m_p_chn_proc_param->chn_param[index].chn_spd_limit_on_curvity;
        this->m_p_channel_config->chn_rapid_overlap_level = this->m_p_chn_proc_param->chn_param[index].chn_rapid_overlap_level;
        this->m_p_channel_config->chn_s_cut_filter_time = this->m_p_chn_proc_param->chn_param[index].chn_s_cut_filter_time;
        this->m_p_channel_config->chn_small_line_time = this->m_p_chn_proc_param->chn_param[index].chn_small_line_time;
        this->m_p_channel_config->chn_max_vel = this->m_p_chn_proc_param->chn_param[index].chn_max_vel;
        this->m_p_channel_config->chn_max_acc = this->m_p_chn_proc_param->chn_param[index].chn_max_acc;
        this->m_p_channel_config->chn_max_dec = this->m_p_chn_proc_param->chn_param[index].chn_max_dec;
        this->m_p_channel_config->chn_max_corner_acc = this->m_p_chn_proc_param->chn_param[index].chn_max_corner_acc;
        this->m_p_channel_config->chn_max_arc_acc = this->m_p_chn_proc_param->chn_param[index].chn_max_arc_acc;
#ifdef USES_WOOD_MACHINE
        this->m_p_channel_config->flip_comp_value = this->m_p_chn_proc_param->chn_param[index].flip_comp_value;
#endif


        //将新参数更新到MC
        this->SetMcChnPlanMode();
        this->SetMcChnPlanParam();
        this->SetMcTapPlanParam();
        this->SetMcChnCornerStopParam();
        this->SetMcChnPlanFun();
#ifdef USES_WOOD_MACHINE
        this->SetMcFlipCompParam();
#endif

        //切换轴参数
        uint8_t phy_axis = 0;
        for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            phy_axis = this->GetPhyAxis(i);
            if(phy_axis == 0xFF)
                continue;

            this->m_p_axis_config[phy_axis].rapid_acc = this->m_p_axis_proc_param[phy_axis].axis_param[index].rapid_acc;
            this->m_p_axis_config[phy_axis].manual_acc = this->m_p_axis_proc_param[phy_axis].axis_param[index].manual_acc;
            this->m_p_axis_config[phy_axis].start_acc = this->m_p_axis_proc_param[phy_axis].axis_param[index].start_acc;
            this->m_p_axis_config[phy_axis].corner_acc_limit = this->m_p_axis_proc_param[phy_axis].axis_param[index].corner_acc_limit;
            this->m_p_axis_config[phy_axis].rapid_s_plan_filter_time = this->m_p_axis_proc_param[phy_axis].axis_param[index].rapid_s_plan_filter_time;
            this->m_p_axis_config[phy_axis].post_filter_type = this->m_p_axis_proc_param[phy_axis].axis_param[index].post_filter_type;
            this->m_p_axis_config[phy_axis].post_filter_time_1 = this->m_p_axis_proc_param[phy_axis].axis_param[index].post_filter_time_1;
            this->m_p_axis_config[phy_axis].post_filter_time_2 = this->m_p_axis_proc_param[phy_axis].axis_param[index].post_filter_time_2;


            this->SetChnAxisAccParam(i);
            this->SetChnAxisSpeedParam(i);
            this->m_p_channel_engine->SendMiAxisFilterParam(i);
        }

        g_ptr_parm_manager->ChangeChnProcParamIndex(this->m_n_channel_index, index);  //包存参数更改


    }else
        return false;
    return true;
}

/**
 * @brief 加工复位执行函数，通过插入Msg的方式，重建执行时的模态
 * @param line_no : 加工复位的行号
 */
void ChannelControl::DoRestart(uint64_t line_no){

    printf("ChannelControl::DoRestart: line=%llu\n", line_no);

    RecordMsg *msg = nullptr;
    DPointChn tar = this->m_channel_rt_status.cur_pos_work;	//终点
    DPointChn src = this->m_channel_rt_status.cur_pos_work;   //起点
    uint32_t axis_mask = 0;   //轴掩码


    ListNode<RecordMsg *> *node = this->m_p_output_msg_list_auto->HeadNode();

    //设置模态
    if(this->m_channel_status.gmode[14] != this->m_mode_restart.gmode[14]){//工件坐标系不匹配
        printf("设置坐标系，%hu to %hu\n", m_channel_status.gmode[14], m_mode_restart.gmode[14]);
        msg = new CoordMsg(tar, src, m_mode_restart.gmode[14], axis_mask);
        if(msg != nullptr){
            msg->SetLineNo(line_no);
            this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //插入工件坐标系设定指令
        }
    }
    if(m_channel_status.cur_d_code != m_mode_restart.cur_d_code ||
            m_channel_status.gmode[7] != m_mode_restart.gmode[7]){  //G40模态不匹配或D值不同
        printf("设置坐G40模态，%hu to %hu， D %hhu to %hhu\n", m_channel_status.gmode[7], m_mode_restart.gmode[7],
                m_channel_status.cur_d_code, m_mode_restart.cur_d_code);

        m_channel_status.cur_d_code = m_mode_restart.cur_d_code;
    }
    if(m_channel_status.cur_h_code != m_mode_restart.cur_h_code ||
            m_channel_status.gmode[8] != m_mode_restart.gmode[8]){  //G49模态不匹配或H值不同
        printf("设置G49模态，%hu to %hu, cur_h = %hhu to %hhu\n", m_channel_status.gmode[8], m_mode_restart.gmode[8],
                m_channel_status.cur_h_code, m_mode_restart.cur_h_code);

        msg = new CompensateMsg(m_mode_restart.gmode[8], m_mode_restart.cur_h_code, src, tar, axis_mask, m_mode_restart.rated_feed, MOVE_G00);
        if(msg != nullptr){
            msg->SetLineNo(line_no);
            this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //插入刀具长度补偿坐标系设定指令
        }
    }
    memcpy(this->m_channel_status.gmode, this->m_mode_restart.gmode, sizeof(uint16_t)*kMaxGModeCount);  //将模态整体赋值到通道当前模态中
    this->InitMcModeStatus();


    //刀号是否一致，不一致则换刀
    if(this->m_channel_status.cur_tool != this->m_mode_restart.cur_tool){  //不一致，换刀

        //		msg = new AuxMsg(6, 1);    //M06换刀指令
        //		if(msg != nullptr){
        //			msg->SetLineNo(line_no);
        //			this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //插入换刀指令
        //		}

        m_channel_status.cur_tool = m_mode_restart.cur_tool;
        SendModeChangToHmi(T_MODE);
    }

    //切换主轴状态
    if(this->m_channel_status.rated_spindle_speed != this->m_mode_restart.rated_spindle_speed){  //指定速度不同
        msg = new SpeedMsg(m_mode_restart.rated_spindle_speed);
        if(msg != nullptr){
            msg->SetLineNo(line_no);
            this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //插入S值指定指令
        }
    }
    if(this->m_channel_status.spindle_dir != this->m_mode_restart.spindle_dir){  //主轴工作状态不同
        int mcode = 5;  //默认M05
        if(m_mode_restart.spindle_dir == SPD_DIR_POSITIVE)
            mcode = 3;
        else if(m_mode_restart.spindle_dir == SPD_DIR_NEGATIVE)
            mcode = 4;
        msg = new AuxMsg(&mcode, 1);
        if(msg != nullptr){
            msg->SetLineNo(line_no);
            this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //插入主轴旋转指令
        }
    }

    if(this->m_channel_rt_status.cur_pos_work != m_mode_restart.pos_target){   //当前位置不是起点位置
        printf("坐标不一致，[%lf, %lf, %lf] to [%lf, %lf, %lf]\n", m_channel_rt_status.cur_pos_work.m_df_point[0], m_channel_rt_status.cur_pos_work.m_df_point[1],
                m_channel_rt_status.cur_pos_work.m_df_point[2], m_mode_restart.pos_target.m_df_point[0], m_mode_restart.pos_target.m_df_point[1],
                m_mode_restart.pos_target.m_df_point[2]);

        src = this->m_channel_rt_status.cur_pos_work;
        tar = src;
        uint8_t chn_axis_z = this->GetChnAxisFromName(AXIS_NAME_Z);
        uint8_t phy_axis_z = this->GetPhyAxis(chn_axis_z);

        //Z轴抬到机械原点
        if(phy_axis_z != NO_AXIS){
            tar.SetAxisValue(chn_axis_z, this->m_p_axis_config[phy_axis_z].axis_home_pos[0]);
            axis_mask = 0x01<<chn_axis_z;
            msg = new RapidMsg(src, tar, axis_mask);
            if(msg != nullptr){
                msg->SetLineNo(line_no);
                msg->SetFlag(FLAG_BLOCK_OVER, true);
                this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //插入抬Z轴指令
            }
        }

        //非XYZ轴复位
        axis_mask = 0;
        uint8_t phy_axis = 0;
        src = tar;
        for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            phy_axis = this->GetPhyAxis(i);
            if(phy_axis == NO_AXIS)
                continue;
            if(this->m_p_axis_config[phy_axis].axis_type != AXIS_SPINDLE && this->m_p_axis_config[phy_axis].sync_axis == 0 &&
                    m_p_channel_config->chn_axis_name[i] > AXIS_NAME_Z){   //非主轴，非从动轴，且非XYZ轴
                axis_mask |= 0x01<<i;
                tar.m_df_point[i] = m_mode_restart.pos_target.m_df_point[i];
            }
        }
        if(axis_mask != 0){
            msg = new RapidMsg(src, tar, axis_mask);
            if(msg != nullptr){
                msg->SetLineNo(line_no);
                msg->SetFlag(FLAG_BLOCK_OVER, true);
                this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //插入非XYZ轴复位指令
            }
        }


        //XY轴运动到起点位置
        axis_mask = 0;
        phy_axis = 0;
        src = tar;
        for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            phy_axis = this->GetPhyAxis(i);
            if(phy_axis == NO_AXIS)
                continue;
            if(this->m_p_axis_config[phy_axis].axis_type != AXIS_SPINDLE &&
                    m_p_channel_config->chn_axis_name[i] < AXIS_NAME_Z){   //非主轴，XY轴
                axis_mask |= 0x01<<i;
                tar.m_df_point[i] = m_mode_restart.pos_target.m_df_point[i];
            }
        }
        if(axis_mask != 0){
            msg = new RapidMsg(src, tar, axis_mask);
            if(msg != nullptr){
                msg->SetLineNo(line_no);
                msg->SetFlag(FLAG_BLOCK_OVER, true);
                this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //插入非XYZ轴复位指令
            }
        }

        //Z轴下降到起点位置
        src = tar;
        if(phy_axis_z != NO_AXIS){
            tar.m_df_point[chn_axis_z] = m_mode_restart.pos_target.m_df_point[chn_axis_z];
            axis_mask = 0x01<<chn_axis_z;
            msg = new RapidMsg(src, tar, axis_mask);
            if(msg != nullptr){
                msg->SetLineNo(line_no);
                msg->SetFlag(FLAG_BLOCK_OVER, true);
                this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //插入Z轴复位指令
            }
        }
    }

    //插入复位完成消息
    msg = new RestartOverMsg();
    if(msg != nullptr){
        msg->SetLineNo(line_no);
        this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //插入加工复位完成指令
    }


    //设置RESTART标志
    this->m_n_restart_step = 2;

}

/**
 * @brief 发送主轴转速对应的DA值到MI
 * @param da_value
 */
void ChannelControl::SendSpdSpeedToMi(uint8_t axis, int16_t da_value){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_SPD_SPEED;
    cmd.data.axis_index = axis;
    cmd.data.data[0] = da_value;
    //	cmd.data.reserved = type;

    this->m_p_mi_comm->WriteCmd(cmd);
}


#ifdef USES_SPEED_TORQUE_CTRL
/**
 * @brief 发送轴控制模式切换命令到MI
 * @param value
 */
void ChannelControl::SendAxisCtrlModeSwitchCmdToMi(uint8_t axis, uint8_t type){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_CTRL_MODE;
    cmd.data.axis_index = axis+1;
    cmd.data.data[0] = type;

    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 发送速度轴运行速度值到MI  value:um/s   0.001°/s
 * @param value
 */
void ChannelControl::SendAxisSpeedCmdToMi(uint8_t axis, int32_t value){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_SPEED;
    cmd.data.axis_index = axis+1;
    memcpy(cmd.data.data, &value, sizeof(int32_t));
    //	cmd.data.reserved = type;

    this->m_p_mi_comm->WriteCmd(cmd);
    printf("execute SendAxisSpeedCmdToMi: axis =%d  value=%d \n",(int)axis,(int)value);
}


/**
 * @brief 发送力矩轴运行力矩值到MI
 * @param value
 */
void ChannelControl::SendAxisTorqueCmdToMi(uint8_t axis, int16_t value, int16_t speed_lmt,int16_t flag){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_TORQUE;
    cmd.data.axis_index = axis+1;
    cmd.data.data[0] = value;
    cmd.data.data[1] = speed_lmt;
    cmd.data.data[2] = flag;

    this->m_p_mi_comm->WriteCmd(cmd);


    printf("execute SendAxisTorqueCmdToMi: axis =%d  value=%d  speed_lmt= %d \n",(int)axis,(int)value,(int)speed_lmt);
}
#endif



/**
 * @brief 设置指定轴的零点位置对应的编码器值
 * @param axis	： 物理轴编号
 * @param encoder ：编码器值
 */
void ChannelControl::SetAxisRefEncoder(uint8_t axis, int64_t encoder){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_REF;
    cmd.data.axis_index = axis;
    memcpy(cmd.data.data, &encoder, sizeof(int64_t));

    this->m_p_mi_comm->WriteCmd(cmd);
}


/**
 * @brief 激活指定轴的Z信号捕获功能
 * @param axis : 物理轴编号
 */
void ChannelControl::ActiveAxisZCapture(uint8_t axis){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_ACTIVE_Z_CAPT;
    cmd.data.axis_index = axis;
    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 发送轴位置清整数圈指令
 * @param axis : 物理轴号, 从1开始
 * @param mode : 整圈的模，单位um
 */
void ChannelControl::SendMiClearPosCmd(uint8_t axis, int mode){
    //	printf("SendMiClearPosCmd:%hhu, %d\n", axis, mode);
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_CLEAR_ROT_AXIS_POS;
    cmd.data.axis_index = axis;
    cmd.data.data[0] = mode & 0xFFFF;
    cmd.data.data[1] = (mode >> 16)& 0xFFFF;

    this->m_p_mi_comm->WriteCmd(cmd);
}

///**
// * @brief 发送攻丝轴号给MI
// * @param spd : 主轴对应的物理轴号，从1开始
// * @param zAxis ： Z轴对应的物理轴号，从1开始
// */
//void ChannelControl::SendMiTapAxisCmd(uint16_t spd, uint16_t zAxis){
//	MiCmdFrame cmd;
//	memset(&cmd, 0x00, sizeof(cmd));
//	cmd.data.cmd = CMD_MI_SET_TAP_AXIS;
//	cmd.data.axis_index = 0xFF;
//	cmd.data.reserved = this->m_n_channel_index;
//	cmd.data.data[0] = spd;
//	cmd.data.data[1] = zAxis;

//	this->m_p_mi_comm->WriteCmd(cmd);
//}

// * @brief 发送攻丝参数给MI
// */
//void ChannelControl::SendMiTapParamCmd(){
//	//当前使用主轴参考点7、8、9代替攻丝前馈增益、补偿系数等

//	MiCmdFrame cmd;
//	memset(&cmd, 0x00, sizeof(cmd));
//	cmd.data.cmd = CMD_MI_SET_TAP_PARAM;
//	cmd.data.axis_index = 0xFF;
//	cmd.data.reserved = this->m_n_channel_index;

//	uint32_t param = this->m_p_axis_config[this->m_spd_axis_phy[0]-1].axis_home_pos[6]*1000;
//	memcpy(&cmd.data.data[0], &param, sizeof(uint32_t));

//	param = this->m_p_axis_config[this->m_spd_axis_phy[0]-1].axis_home_pos[7]*1000;
//	memcpy(&cmd.data.data[2], &param, sizeof(uint32_t));

//	param = this->m_p_axis_config[this->m_spd_axis_phy[0]-1].axis_home_pos[8]*1000;
//	memcpy(&cmd.data.data[4], &param, sizeof(uint32_t));

//	this->m_p_mi_comm->WriteCmd(cmd);
//}

///**
// * @brief 发送攻丝比例给MI
// * @param state :
// * @param ratio ： 攻丝比例，10000*S/F，S单位为rpm，F单位为mm/min
// */
//void ChannelControl::SendMiTapRatioCmd(int32_t ratio){
//	MiCmdFrame cmd;
//	memset(&cmd, 0x00, sizeof(cmd));
//	cmd.data.cmd = CMD_MI_SET_TAP_RATIO;
//	cmd.data.axis_index = 0xFF;
//	cmd.data.reserved = this->m_n_channel_index;
//	memcpy(&cmd.data.data[0], &ratio, sizeof(int32_t));

//	this->m_p_mi_comm->WriteCmd(cmd);
//}

///**
// * @brief 发送攻丝状态给MI
// * @param state : 攻丝状态，true表示开，false表示关
// *
// */
//void ChannelControl::SendMiTapStateCmd(bool state){
//	MiCmdFrame cmd;
//	memset(&cmd, 0x00, sizeof(cmd));
//	cmd.data.cmd = CMD_MI_SET_TAP_STATE;
//	cmd.data.axis_index = 0xFF;
//	cmd.data.reserved = this->m_n_channel_index;
//	cmd.data.data[0] = state?1:0;;

//	this->m_p_mi_comm->WriteCmd(cmd);
//}

/**
 * @brief 发送通道轴-物理轴映射给MI，发送所有轴
 */
void ChannelControl::SendMiChnAxisMap(){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_CHN_PHY_MAP;
    cmd.data.reserved = this->m_n_channel_index;

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        cmd.data.axis_index = i+1;

        cmd.data.data[0] = this->m_p_channel_config->chn_axis_phy[i];   //cmd.data.data[0] = this->m_channel_status.cur_chn_axis_phy[i];   //对应物理轴号

        this->m_p_mi_comm->WriteCmd(cmd);

        //		printf("chan axis = %hhu  chan= %hhu  phyaxis = %hhu",i , this->m_n_channel_index,this->m_p_channel_config->chn_axis_phy[i] );
    }
}

/**
 * @brief 发送通道轴-物理轴映射给MI，单轴，主要用于轴映射关系的动态切换
 */
void ChannelControl::SendAxisMapCmdToMi(uint8_t phy_axis,uint8_t chan,uint8_t axis){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_CHN_PHY_MAP;
    cmd.data.reserved = chan;
    cmd.data.axis_index = axis+1;
    cmd.data.data[0] = phy_axis+1; // this->m_p_channel_config->chn_axis_phy[i];   //对应物理轴号
    cmd.data.data[1] = 1;  // 0--初始配置  1--动态切换
    this->m_p_mi_comm->WriteCmd(cmd);
}


/**
 * @brief 获取通道状态
 * @param status
 */
void ChannelControl::GetChnStatus(HmiChannelStatus &status){
    memcpy((char *)&status, (char *)&m_channel_status, sizeof(HmiChannelStatus));
}

/**
 * @brief 处理HMI获取通道状态指令
 * @param cmd : HMI指令
 */
void ChannelControl::ProcessHmiGetChnStateCmd(HMICmdFrame &cmd){
    //发送响应包
    cmd.frame_number |= 0x8000;
    cmd.cmd_extension = SUCCEED;
    cmd.data_len = (uint16_t)sizeof(HmiChannelStatus);
    memcpy(cmd.data, (char *)&m_channel_status, cmd.data_len);
    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 处理HMI仿真指令
 * @param cmd : HMI指令
 */
void ChannelControl::ProcessHmiSimulateCmd(HMICmdFrame &cmd){
    //	if(cmd.cmd_extension == SIMULATE_OUTLINE){//轮廓仿真
    //
    //
    //	}
    //	else if(cmd.cmd_extension == SIMULATE_TOOLPATH){//刀路仿真
    //
    //	}
    //	else if(cmd.cmd_extension == SIMULATE_MACHINING){//加工仿真
    //
    //	}

    //设置编译器模式
    if(this->m_channel_status.machining_state == MS_PAUSED){

        this->StopRunGCode();    //处于暂停状态需要先复位G代码运行

        m_channel_status.machining_state = MS_READY;
    }

    if(m_channel_status.machining_state == MS_READY){

        this->SetSimulateMode((SimulateMode)cmd.cmd_extension);
        //启动编译器
        this->StartRunGCode();
        cmd.data[0] = SUCCEED;
    }else{
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]切换仿真模式失败！当前[%d]，目标[%d]。", m_n_channel_index,
                              this->m_simulate_mode, cmd.cmd_extension);  	//设置失败

        //结果
        cmd.data[0] = FAILED;
    }

    //发送响应包
    cmd.frame_number |= 0x8000;
    cmd.data_len = 1;
    m_p_hmi_comm->SendCmd(cmd);

}

/**
 * @brief 设置当前加工文件
 * @param cmd
 */
void ChannelControl::ProcessHmiSetNcFileCmd(HMICmdFrame &cmd){

    cmd.frame_number |= 0x8000;   //设置回复标志
    // @add zk 防止打开过长文件名文件
    if(strlen(cmd.data) > 127) return;
    if (strcmp(m_channel_status.cur_nc_file_name, cmd.data))
        m_time_remain = 0;
    if(this->m_channel_status.chn_work_mode != AUTO_MODE){   //非自动模式
        strcpy(m_channel_status.cur_nc_file_name, cmd.data);
        cmd.cmd_extension = SUCCEED;

        this->m_p_hmi_comm->SendCmd(cmd);

        if(m_channel_status.machining_state == MS_PAUSED){//通道复位
            this->Reset();

        }
        g_ptr_parm_manager->SetCurNcFile(m_n_channel_index, m_channel_status.cur_nc_file_name);    //修改当前NC文件

        char path[kMaxPathLen] = {0};
        strcpy(path, PATH_NC_FILE);
        strcat(path, m_channel_status.cur_nc_file_name);   //拼接文件绝对路径
        this->m_p_compiler->OpenFileInScene(path);
    }else if(m_channel_status.machining_state == MS_RUNNING ||
             m_channel_status.machining_state == MS_OUTLINE_SIMULATING ||
             m_channel_status.machining_state == MS_TOOL_PATH_SIMULATING ||
             m_channel_status.machining_state == MS_MACHIN_SIMULATING ||
             m_channel_status.machining_state == MS_PAUSING){  //编译过程中禁止设置

        cmd.cmd_extension = FAILED;
        this->m_p_hmi_comm->SendCmd(cmd);
    }
    else{
        strcpy(m_channel_status.cur_nc_file_name, cmd.data);
        cmd.cmd_extension = SUCCEED;

        this->m_p_hmi_comm->SendCmd(cmd);

        if(m_channel_status.machining_state == MS_PAUSED){//通道复位
            this->Reset();

        }
        g_ptr_parm_manager->SetCurNcFile(m_n_channel_index, m_channel_status.cur_nc_file_name);    //修改当前NC文件

        char path[kMaxPathLen] = {0};
        strcpy(path, PATH_NC_FILE);
        strcat(path, m_channel_status.cur_nc_file_name);   //拼接文件绝对路径
        this->m_p_compiler->OpenFile(path);
        printf("set nc file cmd2 : %s\n", path);
    }
}

/**
 * @brief 处理MDA数据
 * @param cmd : 指令
 */
void ChannelControl::ProcessMdaData(HMICmdFrame &cmd){

    printf("ChannelControl::ProcessMdaData, chn=%hhu\n", this->m_n_channel_index);
    if(this->m_channel_status.chn_work_mode != MDA_MODE) //非MDA模式则退出
        return;
    int fp = open(m_str_mda_path, O_CREAT|O_TRUNC|O_WRONLY); //打开文件
    if(fp < 0){
        CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
        this->m_error_code = ERR_OPEN_FILE;
        g_ptr_trace->PrintLog(LOG_ALARM, "打开MDA临时文件[%s]失败！", m_str_mda_path);
        return;//文件打开失败
    }
    else{
        printf("open file %s\n", m_str_mda_path);
    }

    ssize_t res = write(fp, cmd.data, cmd.data_len);
    if(res == -1){//写入失败
        close(fp);
        CreateError(ERR_LOAD_MDA_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
        this->m_error_code = ERR_LOAD_MDA_DATA;
        return;
    }else if(res != cmd.data_len){//数据缺失
        close(fp);
        CreateError(ERR_LOAD_MDA_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
        this->m_error_code = ERR_LOAD_MDA_DATA;
        printf("write mda temp file error, plan = %d, actual = %d\n", cmd.data_len, res);
        return;
    }

    close(fp);

    if(cmd.data_len == 0){
        printf("MDA data length is 0!\n");
        this->SetMachineState(MS_READY);  //更新通道状态
        return;
    }

    // @test zk

    //m_channel_status.chn_work_mode = AUTO_MODE;
    //this->SendWorkModeToMc(MC_MODE_AUTO);
    //this->m_p_compiler->SetMode((CompilerWorkMode)MC_MODE_AUTO);

    if(!m_p_compiler->OpenFile(m_str_mda_path)){		//编译器打开文件失败
        return;
    }


    printf("ProcessMdaData() m_n_run_thread_state: %d\n", m_n_run_thread_state);
    if(m_n_run_thread_state == IDLE || m_n_run_thread_state == PAUSE){
        pthread_mutex_lock(&m_mutex_change_state);
        //printf("locked 2\n");
        m_n_run_thread_state = RUN;  //置为运行状态
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 2\n");
    }

    this->m_p_f_reg->OP = 1;   //自动运行状态

}

/**
 * @brief 向HMI发送工作状态改变命令
 * @param mach_state ：通道加工状态
 * @return
 */
bool ChannelControl::SendMachineStateCmdToHmi(uint8_t mach_state){
    HMICmdFrame cmd;
    cmd.frame_number = 0;
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_WORK_STATE;
    cmd.cmd_extension = mach_state;
    cmd.data_len = 0x00;

    printf("send machine state[%hhu] to HMI\n", mach_state);


    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 向HMI发送工作模式命令
 * @param chn_work_mode : 通道工作模式
 * @return
 */
bool ChannelControl::SendWorkModeCmdToHmi(uint8_t chn_work_mode){
    HMICmdFrame cmd;
    cmd.frame_number = 0;
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_WORK_MODE;
    cmd.cmd_extension = chn_work_mode;
    cmd.data_len = 0x00;

    const vector<string> table =
        {"非法模式", "自动模式", "MDI模式", "增量模式", "手动模式", "手轮模式", "编辑模式", "回零模式"};
    if (chn_work_mode > 0 && chn_work_mode < table.size())
    {
        g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug,
                                            string("[模式选择] " + table[chn_work_mode]));
    }


    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 向HMI发送通道状态改变命令
 * @param status_type : 状态类型
 * @return
 */
bool ChannelControl::SendChnStatusChangeCmdToHmi(uint16_t status_type){
    HMICmdFrame cmd;
    cmd.frame_number = 0;
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_STATUS_CHANGE;
    cmd.cmd_extension = status_type;

    switch(status_type){
    case FUNC_STATE:
        cmd.data_len = 0x04;
        memcpy(cmd.data, &m_channel_status.func_state_flags, sizeof(Mask32));
        break;
    case AUTO_RATIO:
        cmd.data_len = 0x01;
        cmd.data[0] = m_channel_status.auto_ratio;
        break;
    case SPINDLE_RATIO:
        cmd.data_len = 0x01;
        cmd.data[0] = m_channel_status.spindle_ratio;
        break;
    case RAPID_RATIO:
        cmd.data_len = 0x01;
        cmd.data[0] = m_channel_status.rapid_ratio;
        break;
    case MANUAL_RATIO:
        cmd.data_len = 0x01;
        cmd.data[0] = m_channel_status.manual_ratio;
        break;
    case G_MODE:
        cmd.data_len = kMaxGModeCount*sizeof(uint16_t);
        memcpy(cmd.data, m_channel_status.gmode, cmd.data_len);
        break;
    case MANUAL_STEP:
        cmd.data_len = 0x01;
        cmd.data[0] = m_channel_status.manual_step;
        break;
    case REF_POINT_FLAG:
        cmd.data_len = 0x01;
        cmd.data[0] = m_channel_status.returned_to_ref_point;
        break;
    case CUR_AXIS:
        cmd.data_len = 0x01;
        cmd.data[0] = m_channel_status.cur_axis;
        break;
    default:
        printf("@@@@Invalid value in ChannelControl::SendChnStatusChangeCmdToHmi: %d\n", status_type);
        break;
    }


    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 向HMI发送非G模态状态改变命令，包含T/D/H/F/S
 * @param mode_type
 * @return
 */
bool ChannelControl::SendModeChangToHmi(uint16_t mode_type){
    HMICmdFrame cmd;
    cmd.frame_number = 0;
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_MODE_CHANGE;
    cmd.cmd_extension = mode_type;

    switch(mode_type){
    case T_MODE:		//T
        cmd.data_len = 0x01;
        cmd.data[0] = m_channel_status.cur_tool;
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug,
                                            string("[刀号T]切换为 " + to_string(m_channel_status.cur_tool)));
        //		printf("SendModeChangToHmi, T = %hhu\n", m_channel_status.cur_tool);
        break;
    case D_MODE:		//D
        cmd.data_len = 0x01;
        cmd.data[0] = m_channel_status.cur_d_code;
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug,
                                            string("[D]切换为 " + to_string(m_channel_status.cur_d_code)));
        break;
    case H_MODE:		//H
        cmd.data_len = 0x01;
        cmd.data[0] = m_channel_status.cur_h_code;
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug,
                                            string("[H]切换为 " + to_string(m_channel_status.cur_h_code)));
        break;
    case F_MODE:		//F
        cmd.data_len = sizeof(m_channel_status.rated_feed);
        memcpy(cmd.data, &m_channel_status.rated_feed, cmd.data_len);
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug,
                                            string("[进给速度F]切换为 " + to_string(m_channel_status.rated_feed*60/1000)));
        break;
    case S_MODE:		//S

        cmd.data_len = sizeof(m_channel_status.rated_spindle_speed);
        memcpy(cmd.data, &m_channel_status.rated_spindle_speed, cmd.data_len);
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug,
                                            string("[主轴转速S]切换为 " + to_string(m_channel_status.rated_spindle_speed*60/1000)));
        break;
    default:
        printf("@@@@Invalid value in ChannelControl::SendModeChangToHmi\n");
        break;
    }


    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 向HMI发送MDA数据请求
 * @return true-成功    false-失败
 */
bool ChannelControl::SendMdaDataReqToHmi(){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_MDA_DATA_REQ;
    cmd.cmd_extension = 0;
    cmd.data_len = 0;

    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 向HMI发送提示信息
 * @param msg_type : 消息类型
 * @param msg_id ： 消息ID
 * @param msg_param : 消息参数
 * @return true--发送成功  false--发送失败
 */
bool ChannelControl::SendMessageToHmi(uint16_t msg_type, uint32_t msg_id, uint32_t msg_param){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_MESSAGE;
    cmd.cmd_extension = msg_type;
    cmd.data_len = sizeof(uint32_t)*2;
    memcpy(cmd.data, &msg_id, sizeof(uint32_t));
    memcpy(cmd.data+sizeof(uint32_t), &msg_param, sizeof(uint32_t));

    return this->m_p_hmi_comm->SendCmd(cmd);
}


/**
 * @brief 发送手动对刀结果给HMI
 * @param res : 对刀结果
 * @return true--成功   false--失败
 */
bool ChannelControl::SendManualToolMeasureResToHmi(bool res){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_TOOL_MEASURE_OVER;
    cmd.cmd_extension = res?SUCCEED:FAILED;
    cmd.data_len = 0;


    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 清空当前提示信息
 * @return true--成功   false--失败
 */
bool ChannelControl::ClearHmiInfoMsg(){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_CLEAR_ALARM;
    cmd.cmd_extension = 0xEE;   //清除所有提示信息
    cmd.data_len = 0;


    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 向HMI更新加工计数
 * @param count : 工件计数
 * @return true--成功   false--失败
 */
bool ChannelControl::SendWorkCountToHmi(uint32_t count, uint32_t totalCount){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_WORKPIECE_COUNT;
    cmd.data_len = sizeof(uint32_t)*2;
    memcpy(cmd.data, &count, sizeof(uint32_t));
    memcpy(cmd.data+sizeof(uint32_t), &totalCount, sizeof(uint32_t));

    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 通知HMI加工完成
 * @return true--成功   false--失败
 */
bool ChannelControl::SendMachOverToHmi(){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_NOTIFY_MACH_OVER;

    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 向HMI发送切换NC文件显示命令
 * @param filename
 * @return true--成功   false--失败
 */
bool ChannelControl::SendOpenFileCmdToHmi(char *filename){
    HMICmdFrame cmd;
    cmd.frame_number = 0;
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_DISPLAY_FILE;
    cmd.cmd_extension = 0;
    strcpy(cmd.data, filename);
    cmd.data_len = strlen(cmd.data);

    return this->m_p_hmi_comm->SendCmd(cmd);
}


/**
 * @brief 设置加工状态
 * @param mach_state
 */
void ChannelControl::SetMachineState(uint8_t mach_state){
    // @modify zk 记录之前状态
	uint8_t old_stat = m_channel_status.machining_state;

	g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "Enter SetMachineState, old = %d, new = %d, m_n_run_thread_state = %d\n", m_channel_status.machining_state, mach_state, m_n_run_thread_state);
    if(m_channel_status.machining_state == mach_state)
        return;

    if(mach_state == MS_WARNING && m_channel_status.machining_state == MS_STOPPING
            && m_channel_mc_status.cur_mode != MC_MODE_MANUAL)  //等待停止到位后再切换为告警状态
        return;

    m_channel_status.machining_state = mach_state;       //更新通道状态

    if(mach_state == MS_PAUSED || mach_state == MS_READY){
        m_b_mc_need_start = true;
        //		printf("set mc need start true~~~~~\n");
    }

    //注释说明：不应在此处保存现场，因为插补完成和运动到位存在时间差，此处保存现场会造成位置误差
    //	if(mach_state == MS_PAUSED){
    //		this->SaveAutoScene();
    //	}

    SendMachineStateCmdToHmi(mach_state);   //通知HMI
    SendMachineStateToMc(mach_state);       //通知mc

    //printf("========== old_stat %d, cur_stat %d\n", old_stat, mach_state);

    // @modify zk 修改之后处理
    if(mach_state == MS_PAUSED){
    	this->m_p_f_reg->STL = 0;
		this->m_p_f_reg->SPL = 1;
    }else if(mach_state == MS_WARNING && old_stat == MS_RUNNING){
    	this->m_p_f_reg->STL = 0;
		this->m_p_f_reg->SPL = 1;
    }else if(mach_state == MS_READY){
    	this->m_p_f_reg->STL = 0;
		this->m_p_f_reg->SPL = 0;
    }else if(mach_state == MS_RUNNING){
    	this->m_p_f_reg->STL = 1;
    	this->m_p_f_reg->SPL = 0;
    }

    // @modify zk 之前的处理
    /* if(mach_state == MS_PAUSED || mach_state == MS_WARNING){
    	this->m_p_f_reg->STL = 0;
        this->m_p_f_reg->SPL = 1;
        printf("111111111111111\n");
        //this->m_b_need_change_to_pause = true;
        //@TODO 之后加个报警信号 提出去触发暂停信号
        //这里无法直接改G信号  这里 pause 会造成死锁
        //this->m_p_channel_engine->Pause();

    //}else if(mach_state == MS_READY || mach_state == MS_STOPPING){
    }else if(mach_state == MS_READY || mach_state == MS_STOPPING){

        this->m_p_f_reg->STL = 0;
        this->m_p_f_reg->SPL = 0;
        printf("22222222222222\n");
    }else if(mach_state == MS_RUNNING){
        this->m_p_f_reg->STL = 1;
        this->m_p_f_reg->SPL = 0;
        printf("33333333333333\n");
    }*/

    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "exit SetMachineState, old = %d, new = %d\n", m_channel_status.machining_state, mach_state);
}

/**
 * @brief 切换工作模式
 * @param work_mode : 工作模式
 * @return  无
 */
void ChannelControl::SetWorkMode(uint8_t work_mode){

    if(this->m_channel_status.chn_work_mode == work_mode)
        return;

    //	if(work_mode == AUTO_MODE)
    //		mc_work_mode = 1;
    //	else if(work_mode == MDA_MODE)
    //		mc_work_mode = 2;

    static bool bSaveAutoScene = true;   //是否保存自动模式状态

    // 解决运行中直接切换模式，导致的从新恢复自动时，导致的轴运动不正常的问题
    if(m_channel_status.chn_work_mode == AUTO_MODE &&
            m_channel_status.machining_state == MS_RUNNING)
    {
        this->Pause();
        usleep(200000);
    }

    //处理当前模式的退出动作
    switch(m_channel_status.chn_work_mode){
    case AUTO_MODE:
        //		this->m_n_restart_line = 0;
        //		this->m_n_restart_mode = NOT_RESTART;
        this->m_n_restart_step = 0;
        if(this->m_thread_breakcontinue > 0){//处于断点继续线程执行过程中，则退出断点继续线程
            this->m_b_cancel_breakcontinue_thread = true;
            bSaveAutoScene = false;   //此时不需要保存自动模式状态
        }
    case MDA_MODE:
        pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止
        //printf("locked 3\n");
        if(this->m_channel_status.machining_state == MS_RUNNING){//运行时需要先暂停
            //printf("paus1111\n");
            this->PauseRunGCode();
            this->m_change_work_mode_to = work_mode;	//保存目标模式，在系统停下来后切换
            pthread_mutex_unlock(&m_mutex_change_state);
            return;
        }
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 3\n");
        break;
    case MANUAL_STEP_MODE:
    case MANUAL_MODE:
        //停止对刀
        if(this->m_b_manual_tool_measure){  //终止手动对刀
            m_b_cancel_manual_tool_measure = true;
            this->StopRunGCode(true);
        }

        //停止手动宏程序调用
        if(this->m_b_manual_call_macro){
            this->m_b_cancel_manual_call_macro = true;
            this->StopRunGCode(true);
        }


        //手动停止
        this->ManualMoveStop();
        this->m_p_f_reg->MINC = 0;
        this->m_p_f_reg->MJ = 0;
        break;
    case MPG_MODE:
        //停止对刀
        if(this->m_b_manual_tool_measure){  //终止手动对刀
            m_b_cancel_manual_tool_measure = true;
            this->StopRunGCode(true);
        }

        //停止手动宏程序调用
        if(this->m_b_manual_call_macro){
            this->m_b_cancel_manual_call_macro = true;
            this->StopRunGCode(true);
        }

        this->m_p_f_reg->MH = 0;
        break;
    case REF_MODE:
        this->m_p_f_reg->MREF = 0;
        break;
    default:
        break;
    }

    if(m_channel_status.chn_work_mode == AUTO_MODE && bSaveAutoScene)
        this->SaveAutoScene();

    bSaveAutoScene = true;

    this->m_p_f_reg->MMEM = 0;
    this->m_p_f_reg->MMDI = 0;
    this->m_p_f_reg->MINC = 0;
    this->m_p_f_reg->MJ = 0;
    this->m_p_f_reg->MH = 0;
    //切换模式
    switch(work_mode){
    case AUTO_MODE:
    case MDA_MODE:
        pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止
        //printf("locked 4\n");
        //首先切换输出消息缓冲队列
        if(work_mode == AUTO_MODE){
            m_p_output_msg_list = m_p_output_msg_list_auto;
            //	m_n_run_thread_state = m_n_run_state_auto_bak;
            this->ReloadAutoScene();
            this->m_p_f_reg->MMEM = 1;
            this->SendWorkModeToMc(MC_MODE_AUTO);
        }
        else{
            m_p_output_msg_list = m_p_output_msg_list_mda;
            m_n_run_thread_state = IDLE;
            this->m_p_f_reg->MMDI = 1;
            this->SendWorkModeToMc(MC_MODE_MDA);
            if(m_channel_status.machining_state == MS_PAUSED){
                this->SetMachineState(MS_READY);
            }

            this->m_n_subprog_count = 0;
            this->m_n_macroprog_count = 0;
        }
        this->m_p_compiler->SetMode((CompilerWorkMode)work_mode);	//编译器切换模式
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //切换输出队列
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 4\n");

        //		//设置OP信号
        //		if(!this->m_p_channel_engine->IsEmergency() && !g_ptr_alarm_processor->HasErrorInfo(m_n_channel_index)){
        //			this->m_p_f_reg->OP = 1;
        //		}
        break;
    case MANUAL_STEP_MODE:
        //		this->m_p_f_reg->OP = 0;
        this->m_p_f_reg->MINC = 1;

        //手动模式下，编译器设置为MDA模式
        //pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止
        printf("locked 5\n");
        m_p_output_msg_list = m_p_output_msg_list_mda;
        m_n_run_thread_state = IDLE;
        this->m_p_compiler->SetMode(MDA_COMPILER);	//编译器切换模式
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //切换输出队列
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 5\n");
        this->SendWorkModeToMc(MC_MODE_MANUAL);
        break;
    case MANUAL_MODE:
        //		this->m_p_f_reg->OP = 0;
        this->m_p_f_reg->MJ = 1;

        //手动模式下，编译器设置为MDA模式
        pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止
        //printf("locked 6\n");
        m_p_output_msg_list = m_p_output_msg_list_mda;
        m_n_run_thread_state = IDLE;
        this->m_p_compiler->SetMode(MDA_COMPILER);	//编译器切换模式
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //切换输出队列
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 6\n");
        this->SendWorkModeToMc(MC_MODE_MANUAL);
        break;
    case MPG_MODE:
        //		this->m_p_f_reg->OP = 0;
        this->m_p_f_reg->MH = 1;

        //手动模式下，编译器设置为MDA模式
        pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止
        //printf("locked 7\n");
        m_p_output_msg_list = m_p_output_msg_list_mda;
        m_n_run_thread_state = IDLE;
        this->m_p_compiler->SetMode(MDA_COMPILER);	//编译器切换模式
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //切换输出队列
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 7\n");
        this->SendWorkModeToMc(MC_MODE_MANUAL);
        break;
    case REF_MODE:
        this->m_p_f_reg->MREF = 1;

        //原定模式下，编译器设置为MDA模式
        pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止
        //printf("locked 8\n");
        m_p_output_msg_list = m_p_output_msg_list_mda;
        m_n_run_thread_state = IDLE;
        this->m_p_compiler->SetMode(MDA_COMPILER);	//编译器切换模式
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //切换输出队列
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 8\n");
        this->SendWorkModeToMc(MC_MODE_MANUAL);
        break;
    default:
        break;
    }

    m_channel_status.chn_work_mode = work_mode;

    this->SendWorkModeCmdToHmi(work_mode); 	//通知HMI工作模式切换

}

/**
 * @brief 向MC模块发送DSP插补工作模式命令
 * @param intp_mode : 0--手动插补    1--自动插补    2--MDA插补
 * @return
 */
void ChannelControl::SendIntpModeCmdToMc(uint16_t intp_mode){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = this->m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_SET_CHN_INTP_MODE;

    cmd.data.data[0] = intp_mode;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);

}

/**
 * @brief 向MC模块发送加工模式命令，主要用于区分插补目标位置，方便计算不同模式的余移动量
 * @param work_mode : 0--手动    1--自动    2--MDA
 */
void ChannelControl::SendWorkModeToMc(uint16_t work_mode){

    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = this->m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_SET_CHN_WORK_MODE;

    cmd.data.data[0] = work_mode;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

void ChannelControl::SendMachineStateToMc(uint8_t state){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = this->m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_MACHINE_STATE;

    cmd.data.data[0] = state;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 结束编译后的处理
 */
void ChannelControl::CompileOver(){
    if(m_p_compiler->CompileOver()){
#ifdef USES_ADDITIONAL_PROGRAM
        if(this->m_n_add_prog_type != CONTINUE_START_ADD)
#endif
            SetMachineState(MS_READY);
    }

#ifndef USES_SIMULATION_TEST
    m_simulate_mode = SIM_NONE;
#endif
}

/**
 * @brief file_name是否为当前加工文件
 * @param file_name : 文件名称
 */
bool ChannelControl::IsCurNcFile(char *file_name){
    char nc_file[kMaxFileNameLen];
    memset(nc_file, 0x00, kMaxFileNameLen);
    m_p_compiler->GetCurNcFile(nc_file);
    if(strcmp(file_name, nc_file) == 0)
        return true;
    return false;
}

/**
 * @brief 刷新nc文件file,  如果file已经加载，则unmap
 * @param file
 */
void ChannelControl::RefreshFile(char *file){
    if(!this->IsCurNcFile(file))
        return;

    if(this->m_channel_status.machining_state != MS_READY){
        this->StopRunGCode();
    }

    this->m_p_compiler->UnmapCurNcFile();
}

/**
 * @brief 重新映射当前加载的NC文件
 */
bool ChannelControl::RemapFile(char *file){
    if(!this->IsCurNcFile(file))
        return true;
    return this->m_p_compiler->RemapCurNcFile();
}

/**
 * @brief 复位MC模块的当前加工行号，置零
 */
void ChannelControl::ResetMcLineNo(){
    McCmdFrame cmd;

    cmd.data.cmd = CMD_MC_CLEAR_CHN_LINE_NO;
    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief G代码运行线程函数
 * @param void *args: ChannelControl对象指针
 */
void *ChannelControl::CompileThread(void *args){
    printf("Start ChannelControl::CompileThread! id = %ld\n", syscall(SYS_gettid));
    ChannelControl *p_channel_control = static_cast<ChannelControl *>(args);

    int res = ERR_NONE;
    res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
    if (res != ERR_NONE) {
        printf("Quit ChannelControl::CompileThread with error 1!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }
    res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
    if (res != ERR_NONE) {
        printf("Quit ChannelControl::CompileThread with error 2!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }

    while(!g_sys_state.system_ready) //等待系统就绪
        usleep(50000);  //等待50ms

    res = p_channel_control->Run();


    printf("Exit ChannelControl::CompileThread!\n");
    pthread_exit(NULL);
}

/**
 * @brief G代码运行行函数
 * @return 执行结果
 */
int ChannelControl::Run(){
    int res = ERR_NONE;
    //  初始化
    //	struct timeval tvStart;
    //	struct timeval tvNow;
    //	unsigned int nTimeDelay = 0;

    bool bf = true;
    int compile_count = 0;  //连续编译代码行计数

    //执行循环
    while(!g_sys_state.system_quit)
    {
        if(m_n_run_thread_state == RUN)
        {
            //printf("m_n_run_thread_state = RUN\n");
            pthread_mutex_lock(&m_mutex_change_state);
            //printf("locked 9\n");
            //			if(this->m_channel_status.chn_work_mode == AUTO_MODE &&
            //#ifdef USES_ADDITIONAL_PROGRAM
            //					this->m_n_add_prog_type != CONTINUE_START_ADD &&
            //#endif
            //					(m_n_hw_trace_state == REVERSE_TRACE || m_p_last_output_msg != this->m_p_output_msg_list->TailNode()))
            if(m_n_hw_trace_state == REVERSE_TRACE || m_p_last_output_msg != this->m_p_output_msg_list->TailNode())
            {
            	//自动模式下，反向引导或者正向引导缓冲数据未发送完，则不进行编译
                //	printf("@@@@@@, last= %d, tail=%d\n", m_p_last_output_msg, m_p_output_msg_list->TailNode());
                //printf("11111\n");
            	bf = ExecuteMessage();

				if(!bf){
					if(m_error_code != ERR_NONE){
						g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "execute message error3, %d\n", m_error_code);
						m_n_run_thread_state = ERROR; //编译出错
					}
					else
						usleep(10000);
				}
			}else if(m_p_compiler->GetErrorCode() != ERR_NONE)
			{
				//编译器出错，但需要继续执行已编译指令
				if(m_p_compiler->RunMessage()){
					if(!ExecuteMessage()){
						if(m_error_code != ERR_NONE){
							g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "execute message error1, %d\n", m_error_code);
							m_n_run_thread_state = ERROR; //编译出错
						}
					}
				}
				else{
					if(m_error_code != ERR_NONE){
						m_n_run_thread_state = ERROR; //编译出错
					}else
						m_n_run_thread_state = WAIT_RUN;//执行失败，状态切换到WAIT_RUN
				}
			}
			else if(m_p_compiler->GetLineData())
			{
				//获取一行源码
				if(!m_p_compiler->CompileLine())  //编译一行代码
				{
					m_n_run_thread_state = ERROR; //编译出错
					g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "#####Compile Error#####\n");
				}
                else{

                    //printf("----------------------------> CompileLine\n");
					if(m_p_compiler->RunMessage()){
                        //printf("----------------------------> RunMessage\n");
						if(!ExecuteMessage()){
                            //printf("----------------------------> ExecuteMessage\n");
							if(m_error_code != ERR_NONE){

								g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "execute message error2, %d\n", m_error_code);
								m_n_run_thread_state = ERROR; //编译出错
							}else{  //执行未成功，转换为WAIT_EXECUTE状态

								usleep(10000);   //休眠10ms
                                //printf("----------------------------> WaitExcute\n");
							}
						}
					}
					else{
						if(m_p_compiler->GetErrorCode() != ERR_NONE){
                            //printf("ccccc\n");
							CreateError(m_p_compiler->GetErrorCode(), ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
							m_n_run_thread_state = ERROR;
						}else{
							//printf("dcba\n");
							m_n_run_thread_state = WAIT_RUN;//执行失败，状态切换到WAIT_RUN
						}
					}
				}

                if(++compile_count >= 20){
                    compile_count = 0;
                    usleep(2000);     //连续编译50条，休眠2ms
                }
            }
            else if(m_p_compiler->IsCompileOver()){  //已经编译完成

            	ExecuteMessage();
            }
            else
            {//出错
            	if(m_p_compiler->GetErrorCode() != ERR_NONE){
                    g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "Get line data error!\n");
                    m_n_run_thread_state = ERROR;
                }
            }


#ifndef USES_WOOD_MACHINE   //木工专机不检查M30结束指令
            if(m_p_compiler->IsEndOfFile() && !m_p_compiler->IsCompileOver() &&
                    this->m_channel_status.chn_work_mode == AUTO_MODE && !m_p_compiler->IsPreScaning() && !m_p_compiler->IsSubProgram()
        #ifdef USES_ADDITIONAL_PROGRAM
                    && this->m_n_add_prog_type != CONTINUE_START_ADD
        #endif
                    )
            {

            	m_n_run_thread_state = ERROR;
                g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]语法错误，未找到结束指令！", m_n_channel_index);
                CreateError(ERR_NO_END, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            }
#endif
            pthread_mutex_unlock(&m_mutex_change_state);
            //printf("unlocked 9\n");
            //			if(this->m_lexer_result.line_no%10000 == 0){
            //				gettimeofday(&tvNow, NULL);
            //				nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;
            //				printf("compile 10000 line cost %dus\n", nTimeDelay);
            //			}

        }
        else if(m_n_run_thread_state == ERROR)
        {
            //TODO 处理错误
            g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "Compile Error:%d, %d\n", m_error_code, m_p_compiler->GetErrorCode());
            //CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            // @test zk  解决编译报警但运行并未停止问题
            PauseMc();

            m_n_run_thread_state = STOP;
        }
        else if(m_n_run_thread_state == STOP)
        {
            //TODO 编译结束，复位编译器状态及变量

            m_n_run_thread_state = IDLE;

            g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "Compiler run STOP!!!!!\n");
        }
        else if(m_n_run_thread_state == PAUSE)
        {
            usleep(2000);
        }
        else if(m_n_run_thread_state == WAIT_EXECUTE)
        {
            pthread_mutex_lock(&m_mutex_change_state);

            bf = ExecuteMessage();
            pthread_mutex_unlock(&m_mutex_change_state);

            if(!bf){
                if(m_error_code != ERR_NONE){
                    g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "execute message error4, %d\n", m_error_code);
                    m_n_run_thread_state = ERROR; //编译出错
                }
                else{
                    usleep(10000);
                }
            }
        }
        else if(m_n_run_thread_state == WAIT_RUN)
        {
            pthread_mutex_lock(&m_mutex_change_state);

            bf = m_p_compiler->RunMessage();

            if(!ExecuteMessage()){
                if(m_error_code != ERR_NONE){
                    g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "execute message error5, %d\n", m_error_code);
                    m_n_run_thread_state = ERROR; //编译出错
                }
            }

            pthread_mutex_unlock(&m_mutex_change_state);
            //printf("unlocked 11\n");
            if(!bf){
            	usleep(10000);
                m_n_run_thread_state = WAIT_RUN;    //防止在ExecuteMessage（）函数中修改m_n_run_thread_state
            }
            else if(m_n_run_thread_state != ERROR){
                m_n_run_thread_state = RUN; //等待执行完成， 状态切回RUN
            }

        }
        else
        {
            usleep(10000);   //非运行状态，线程挂起10ms
        }
    }

    return res;
}


/**
 * @brief  状态更新线程函数
 * @param args
 */
void *ChannelControl::RefreshStatusThread(void *args){
    ChannelControl *chn_ctrl = static_cast<ChannelControl *>(args);
    printf("Start ChannelControl[%d]::RefreshStatusThread!thread id = %ld\n", chn_ctrl->GetChnIndex(), syscall(SYS_gettid));

    int res = ERR_NONE;
    res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
    if (res != ERR_NONE) {
        printf("Quit ChannelControl::RefreshStatusThread!thread with error 1!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }
    res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
    if (res != ERR_NONE) {
        printf("Quit ChannelControl::RefreshStatusThread!thread with error 2!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }

    while(!g_sys_state.system_ready){  //等待系统启动
        usleep(10000);
    }

    chn_ctrl->RefreshStatusFun();

    printf("Quit ChannelControl::RefreshStatusThread!!\n");
    pthread_exit(NULL);
}

bool ChannelControl::RefreshStatusFun(){

	bool step_mode_flag = false;   //是否单步模式
	int check_count = 1;           //状态确认次数
	uint32_t data = 0;
	uint64_t count = 0;           //循环计数
	while(!g_sys_state.system_quit){

		if(!this->m_b_mc_on_arm){

			//更新当前MC的运行模式
			this->m_p_mc_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);

			//更新当前行号
			this->m_p_mc_comm->ReadCurLineNo(m_n_channel_index, m_channel_mc_status.cur_line_no);

			//更新当前进给速度
			this->m_p_mc_comm->ReadCurFeed(m_n_channel_index, m_channel_mc_status.cur_feed);

			//更新当前给定进给速度
			this->m_p_mc_comm->ReadRatedFeed(m_n_channel_index, m_channel_mc_status.rated_feed);

			//更新当前运行指令`
			this->m_p_mc_comm->ReadCurCmd(m_n_channel_index, m_channel_mc_status.cur_cmd);

			//更新MC当前缓冲数据量
			this->m_p_mc_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);

			//更新MDA缓冲数据量
			this->m_p_mc_comm->ReadMdaBufDataCount(m_n_channel_index, m_channel_mc_status.mda_data_count);

			//更新当前轴插补位置
			this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

			//更新AUTO分块到位标志
			m_channel_mc_status.auto_block_over = m_p_mc_comm->ReadAutoBlockRunOverFlag(m_n_channel_index);

			//更新MDA分块到位标志
			m_channel_mc_status.mda_block_over = m_p_mc_comm->ReadMdaBlockRunOverFlag(m_n_channel_index);

			//更新单段到位标志
			m_channel_mc_status.step_over = m_p_mc_comm->ReadStepRunOverFlag(m_n_channel_index);

			//更新当前轴到位标志
			this->m_p_mc_comm->ReadChnAxisRunoverMask(m_n_channel_index, m_channel_mc_status.axis_over_mask);
            //更新当前轴到位(手动)标志
            this->m_p_mc_comm->ReadChnManuAxisRunoverMask(m_n_channel_index, m_channel_mc_status.manu_axis_over_mask);

			//更新当前MC告警标志
			this->m_p_mc_comm->ReadMcErrFlag(m_n_channel_index, m_channel_mc_status.mc_error.all);

			if(m_channel_mc_status.cur_mode == MC_MODE_AUTO &&
#ifdef USES_ADDITIONAL_PROGRAM
				m_n_add_prog_type == NONE_ADD &&
#endif
				m_b_lineno_from_mc && m_channel_mc_status.cur_line_no > 0){  //即刻更新实时状态行号
			m_channel_rt_status.line_no = m_channel_mc_status.cur_line_no;
		    }

		}else{
			//更新当前MC的运行模式
			this->m_p_mc_arm_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);

			//更新当前行号
			this->m_p_mc_arm_comm->ReadCurLineNo(m_n_channel_index, m_channel_mc_status.cur_line_no);

			//更新当前进给速度
			this->m_p_mc_arm_comm->ReadCurFeed(m_n_channel_index, m_channel_mc_status.cur_feed);

			//更新当前给定进给速度
			this->m_p_mc_arm_comm->ReadRatedFeed(m_n_channel_index, m_channel_mc_status.rated_feed);

			//更新当前运行指令`
			this->m_p_mc_arm_comm->ReadCurCmd(m_n_channel_index, m_channel_mc_status.cur_cmd);

			//更新MC当前缓冲数据量
			this->m_p_mc_arm_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);

			//更新MDA缓冲数据量
			this->m_p_mc_arm_comm->ReadMdaBufDataCount(m_n_channel_index, m_channel_mc_status.mda_data_count);

			//更新当前轴插补位置
			this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

			//更新AUTO分块到位标志
			m_channel_mc_status.auto_block_over = m_p_mc_arm_comm->ReadAutoBlockRunOverFlag(m_n_channel_index);

			//更新MDA分块到位标志
			m_channel_mc_status.mda_block_over = m_p_mc_arm_comm->ReadMdaBlockRunOverFlag(m_n_channel_index);

			//更新单段到位标志
			m_channel_mc_status.step_over = m_p_mc_arm_comm->ReadStepRunOverFlag(m_n_channel_index);

			//更新当前轴到位标志
			this->m_p_mc_arm_comm->ReadChnAxisRunoverMask(m_n_channel_index, m_channel_mc_status.axis_over_mask);

			//更新当前MC告警标志
			this->m_p_mc_arm_comm->ReadMcErrFlag(m_n_channel_index, m_channel_mc_status.mc_error.all);


			if(m_channel_mc_status.cur_mode == MC_MODE_AUTO &&
#ifdef USES_ADDITIONAL_PROGRAM
				m_n_add_prog_type == NONE_ADD &&
#endif
				m_b_lineno_from_mc && m_channel_mc_status.cur_line_no > 0){  //即刻更新实时状态行号
			m_channel_rt_status.line_no = m_channel_mc_status.cur_line_no;
		    }

		}

		//printf("m_channel_mc_status.cur_mode : %d\n", m_channel_mc_status.cur_mode);
		//更新系统状态
		step_mode_flag = IsStepMode();
		if(step_mode_flag){
			check_count = 10;    //2;
		}
		else{
			check_count = 10;    //1;   //增加延时，等待轴运行到位
		}
		if(m_channel_mc_status.cur_mode ==MC_MODE_MANUAL){

			if(m_channel_status.machining_state == MS_PAUSING ||		//暂停中并且MC已切换至手动模式
					m_channel_status.machining_state == MS_STOPPING ||
					(m_channel_status.machining_state == MS_RUNNING &&
							step_mode_flag && m_channel_mc_status.step_over && !m_b_mc_need_start)){//单段模式下，单段到位

				if(++m_n_step_change_mode_count > check_count){
					if(m_channel_status.machining_state == MS_STOPPING){
						if(g_ptr_alarm_processor->HasErrorInfo()){
							this->SetMachineState(MS_WARNING);
						}else{
							SetMachineState(MS_READY);
						}
						if(this->m_channel_status.chn_work_mode == AUTO_MODE
#ifdef USES_ADDITIONAL_PROGRAM
								&& this->m_n_add_prog_type != CONTINUE_START_ADD
#endif
						){
							this->SetCurLineNo(1);
							this->InitMcIntpAutoBuf();
						}else if(m_channel_status.chn_work_mode == MDA_MODE){
							this->InitMcIntpMdaBuf();
						}

						this->m_p_output_msg_list->Clear();
						this->ResetMcLineNo();
					}
					else{
					//	printf("change to pause in RefreshStatusFun();change_to_pause=%hhu\n", this->m_b_need_change_to_pause);
						if(IsStepMode()){
							if(this->m_channel_mc_status.buf_data_count > 0)
								this->m_b_need_change_to_pause = true;
						}
						SetMachineState(MS_PAUSED);
					}

					if(m_change_work_mode_to != INVALID_MODE && this->m_thread_breakcontinue == 0){//需要切换模式，断点继续也已经结束
						SetWorkMode(m_change_work_mode_to);
						m_change_work_mode_to = INVALID_MODE;
					}
					printf("ChannelControl::RefreshStatusFun() switch to pause\n");
					m_n_step_change_mode_count = 0;
				}

			}else if(this->m_b_delay_to_reset && (m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING)){
				this->m_b_delay_to_reset = false;
				this->Reset();
			}
		}else{
			m_n_step_change_mode_count = 0;


			//更新模态, 只有MC处于AUTO模式时才从MC中读取模态
			//更新MC模态信息
			if(!this->m_b_mc_on_arm)
				this->m_p_mc_comm->ReadMcModeInfo(m_n_channel_index, m_channel_mc_status.mc_mode.all);
			else
				this->m_p_mc_arm_comm->ReadMcModeInfo(m_n_channel_index, m_channel_mc_status.mc_mode.all);

			this->RefreshModeInfo(m_channel_mc_status.mc_mode);
		}

		if(this->m_b_change_hw_trace_state){//切换手轮跟踪模式
			if(m_channel_mc_status.cur_mode ==MC_MODE_MANUAL && m_channel_mc_status.cur_feed < 3){
				bool res = this->ChangeHwTraceState(this->m_n_hw_trace_state_change_to);
//				printf("change to mpg trace mode:%hhu\n", m_n_hw_trace_state_change_to);

				//发送MI手轮状态切换响应命令
				MiCmdFrame cmd;
				memset(&cmd, 0x00, sizeof(cmd));
				cmd.data.cmd = CMD_MI_HW_TRACE_STATE_CHANGED_RSP;
				cmd.data.reserved = m_n_channel_index;
				cmd.data.axis_index = 0xFF;
				cmd.data.data[0] = m_n_hw_trace_state_change_to;
				cmd.data.data[1] = res?0x01:0x00;
				this->m_p_mi_comm->WriteCmd(cmd);

				m_b_change_hw_trace_state = false;
			}

		}


		//更新用户给定进给速度
		if((m_channel_status.chn_work_mode == AUTO_MODE || m_channel_status.chn_work_mode == MDA_MODE) &&   //手动模式下F模态固定为0，不更新
				m_channel_status.rated_feed != m_channel_mc_status.rated_feed){
			m_channel_status.rated_feed = m_channel_mc_status.rated_feed;
			this->SendModeChangToHmi(F_MODE);
		}

//		if((m_channel_status.chn_work_mode == AUTO_MODE || m_channel_status.chn_work_mode == MDA_MODE) &&
//				m_channel_status.machining_state == MS_RUNNING ){
//			uint16_t mc_work_mode;
//			this->m_p_mc_comm->ReadWorkMode(m_n_channel_index, mc_work_mode);
//
//			if(mc_work_mode == MC_MODE_MANUAL)
//				this->SetMachineState(MS_READY);
//		}

		if(m_channel_mc_status.mc_error.bits.err_soft_limit_neg || m_channel_mc_status.mc_error.bits.err_soft_limit_pos){//软限位告警
			if(!this->m_b_mc_on_arm){
				this->m_p_mc_comm->ReadChnSoftLimtMask(m_n_channel_index, data);
				this->m_channel_mc_status.axis_soft_negative_limit = data&0xFFFF;
				this->m_channel_mc_status.axis_soft_postive_limit = (data>>16)&0xFFFF;
			}else{
				this->m_p_mc_arm_comm->ReadChnPosSoftLimtMask(m_n_channel_index, data);
				this->m_channel_mc_status.axis_soft_postive_limit = data;
				this->m_p_mc_arm_comm->ReadChnNegSoftLimtMask(m_n_channel_index, data);
				this->m_channel_mc_status.axis_soft_negative_limit = data;
			}
			if(m_channel_mc_status.mc_error.bits.err_soft_limit_neg){
				CreateError(ERR_SOFT_LIMIT_NEG, ERROR_LEVEL, CLEAR_BY_MCP_RESET, data, m_n_channel_index);
			}
			if(m_channel_mc_status.mc_error.bits.err_soft_limit_pos){
				CreateError(ERR_SOFT_LIMIT_POS, ERROR_LEVEL, CLEAR_BY_MCP_RESET, data, m_n_channel_index);
			}
		}

		if(m_channel_mc_status.mc_error.bits.err_pos_over){//位置指令过大
			if(!this->m_b_mc_on_arm)
				this->m_p_mc_comm->ReadChnPosErrMask(m_n_channel_index, m_channel_mc_status.pos_error_mask);
			else
				this->m_p_mc_arm_comm->ReadChnPosErrMask(m_n_channel_index, m_channel_mc_status.pos_error_mask);

			printf("============ ERR_POS_ERR ===============\n");
			CreateError(ERR_POS_ERR, ERROR_LEVEL, CLEAR_BY_MCP_RESET, m_channel_mc_status.pos_error_mask, m_n_channel_index);
		}

		if(m_channel_mc_status.mc_error.bits.err_arc_data){//圆弧数据错误
			CreateError(ERR_ARC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
		}

		if(m_channel_mc_status.mc_error.bits.err_cmd_crc){//指令校验错误
			CreateError(ERR_CMD_CRC, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
		}

		if(m_channel_mc_status.mc_error.bits.err_data_crc){//数据包校验错误
			CreateError(ERR_DATA_CRC, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
		}


		if(this->m_b_manual_tool_measure){ //当前正在执行手动对刀动作
			if(this->GetCompileState() == IDLE ){
				this->m_b_manual_tool_measure = false;

				//TODO 给HMI发送对刀完成消息
				this->SendManualToolMeasureResToHmi(!this->m_b_cancel_manual_tool_measure);
				m_b_cancel_manual_tool_measure = false;   //复位信号
			}
		}

		if(this->m_b_manual_call_macro){
            if(this->GetCompileState() == IDLE || this->GetCompileState() == STOP/*中途停止时，State为Stop*/){
				this->m_b_manual_call_macro = false;

				this->m_b_cancel_manual_call_macro = false;
			}
		}

		if(this->m_channel_status.chn_work_mode == AUTO_MODE && this->m_b_hmi_graph &&
				(this->m_channel_status.machining_state == MS_RUNNING || this->m_channel_status.machining_state == MS_MACHIN_SIMULATING)){
			this->ReadGraphAxisPos();
		}

#ifdef USES_WUXI_BLOOD_CHECK
		if(this->m_b_returnning_home){  //回零处理
			this->ReturnHome();
		}
#endif

#ifdef USES_GRIND_MACHINE   //玻璃磨床
		if(this->m_b_ret_safe_state && m_p_f_reg->SA == 1){	//需要等待上伺服完成
			if(g_ptr_alarm_processor->HasErrorInfo()){//出错取消回安全位置
				m_b_ret_safe_state = false;
			}else{
				//等待延时执行
				struct timeval time_now;
				unsigned int time_elpase = 0;
				gettimeofday(&time_now, NULL);
				time_elpase = (time_now.tv_sec-m_time_ret_safe_delay.tv_sec)*1000000+time_now.tv_usec-m_time_ret_safe_delay.tv_usec;
				if(time_elpase > 50000){
					this->ReturnSafeState();    //回安全点
				}
			}
		}
#endif

#ifdef USES_LASER_MACHINE
		if(m_b_laser_calibration){
			this->ProcessLaserCalibration();   //执行调高器标定
			this->ReadHeightRegulatorInput();   //读取输入信号
		}

#endif

#ifdef USES_WOOD_MACHINE
		//读取当前刀具寿命
		if(m_n_ref_tool_life_delay > 0){
			m_n_ref_tool_life_delay--;
		}else if(count%500 == 0){  //每500个周期更新一次
			if(m_channel_status.cur_tool > 0){
				int life = 0;
				this->m_p_mc_comm->ReadChnCurToolLife(this->m_n_channel_index, life);
				if(this->m_channel_status.machining_state != MS_RUNNING &&
						this->m_p_chn_tool_info->tool_life_cur[this->m_channel_status.cur_tool-1] != life){
					m_p_chn_tool_info->tool_life_cur[this->m_channel_status.cur_tool-1] = life;

					//保存至文件
					g_ptr_parm_manager->UpdateToolPotConfig(this->m_n_channel_index, *this->m_p_chn_tool_info);
					m_b_save_tool_info = false;
				}else if(this->m_p_chn_tool_info->tool_life_cur[this->m_channel_status.cur_tool-1] != life){
					m_p_chn_tool_info->tool_life_cur[this->m_channel_status.cur_tool-1] = life;
					m_b_save_tool_info = true;
				}


				if(m_p_chn_tool_info->tool_life_max[this->m_channel_status.cur_tool-1] > 0 && count%5000 == 0){  //告警周期20s，检查一次
					if(life >= m_p_chn_tool_info->tool_life_max[this->m_channel_status.cur_tool-1]){ //刀具到寿
						CreateError(ERR_TOOL_LIFE_OVER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, this->m_channel_status.cur_tool, m_n_channel_index);
					}else if(life >= m_p_chn_tool_info->tool_life_max[this->m_channel_status.cur_tool-1]*0.95){  //刀具接近到寿
						CreateError(ERR_TOOL_LIFE_COMING, WARNING_LEVEL, CLEAR_BY_MCP_RESET, this->m_channel_status.cur_tool, m_n_channel_index);
					}
				}
			}
		}
#endif

        if(m_channel_status.cur_tool > 0 && m_channel_status.cur_tool <= kMaxToolCount){
            int cur_tool = m_channel_status.cur_tool - 1;
            if(m_p_chn_tool_info->tool_life_type[cur_tool] == ToolPot_Cnt)
            {// 按次计数
                if (m_p_chn_tool_info->tool_life_cur[cur_tool] >= m_p_chn_tool_info->tool_threshold[cur_tool])
                {
                    if (m_p_chn_tool_info->tool_life_cur[cur_tool] >= m_p_chn_tool_info->tool_life_max[cur_tool])
                    {
                        CreateError(ERR_TOOL_LIFE_OVER, ERROR_LEVEL, CLEAR_BY_MCP_RESET, m_channel_status.cur_tool, m_n_channel_index);
                    }
                    else
                    {
                        CreateError(ERR_TOOL_LIFE_COMING, WARNING_LEVEL, CLEAR_BY_MCP_RESET, m_channel_status.cur_tool, m_n_channel_index);
                    }
                }
            }
        }

		count++;   //计数加一
		usleep(4000);
	}

	return true;
}

/**
 * @brief 是否MC块运行到位
 * @return true--是  false--否
 */
bool ChannelControl::IsBlockRunOver(){
    struct timeval cur_time;
    gettimeofday(&cur_time, nullptr);
    unsigned int delay = (cur_time.tv_sec-m_time_start_mc.tv_sec)*1000000+cur_time.tv_usec-m_time_start_mc.tv_usec;  //单位微秒
    if(delay < MC_STATE_REFRESH_CYCLE) //发送MC启动命令5ms后才能确认到位标志
        return false;

    if(this->m_channel_status.chn_work_mode == AUTO_MODE){
        if(!this->m_b_mc_on_arm)
            m_channel_mc_status.auto_block_over = m_p_mc_comm->ReadAutoBlockRunOverFlag(m_n_channel_index);
        else
            m_channel_mc_status.auto_block_over = m_p_mc_arm_comm->ReadAutoBlockRunOverFlag(m_n_channel_index);
        return m_channel_mc_status.auto_block_over;
    }
    else{
        if(!this->m_b_mc_on_arm)
            m_channel_mc_status.mda_block_over = m_p_mc_comm->ReadMdaBlockRunOverFlag(m_n_channel_index);
        else
            m_channel_mc_status.mda_block_over = m_p_mc_arm_comm->ReadMdaBlockRunOverFlag(m_n_channel_index);
        return m_channel_mc_status.mda_block_over;
    }
}

/**
 * @brief 设置当前行号，并且设置当前行号不从MC获取
 * @param line_no
 */
void ChannelControl::SetCurLineNo(uint32_t line_no){
    this->m_b_lineno_from_mc = false;
#ifdef USES_ADDITIONAL_PROGRAM
    if(this->m_n_add_prog_type != NONE_ADD)
        return;
#endif
    if(this->m_n_macroprog_count == 0 || this->m_p_general_config->debug_mode > 0)
    {
        this->m_channel_rt_status.line_no = line_no;
    }
    ResetMcLineNo();//复位MC模块当前行号
}


/**
 * @brief 向HMI发送仿真数据
 * @param msg : 待处理消息
 * @return
 */
bool ChannelControl::OutputSimulateData(RecordMsg *msg){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "ChannelControl::OutputSimulateData, line = %llu", msg->GetLineNo());
    SimulateData data;
    CompilerSimData comp_data;
    MonitorDataType type = MDT_CHN_SIM_GRAPH;  //数据类型
    uint8_t chn_axis_x = 0, chn_axis_y = 0, chn_axis_z = 0;  //通道轴号

    if(msg->IsMoveMsg()){
        if(1 == msg->GetSimulateData(comp_data))
            return true;

        if(m_simulate_mode == SIM_TOOLPATH)
            type = MDT_CHN_SIM_TOOL_PATH;

        chn_axis_x = this->GetChnAxisFromName(AXIS_NAME_X);
        chn_axis_y = this->GetChnAxisFromName(AXIS_NAME_Y);
        chn_axis_z = this->GetChnAxisFromName(AXIS_NAME_Z);
        //数据转换及发送
        if(comp_data.type == RAPID_TARGET || comp_data.type == LINE_TARGET){  //G00/G01
            data.type = comp_data.type;
            data.pos[0] = comp_data.target[0];
            data.pos[1] = comp_data.target[1];
            data.pos[2] = comp_data.target[2];

            //转换为机械坐标系
            if(chn_axis_x != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[0], this->m_channel_status.gmode[14], chn_axis_x);
            if(chn_axis_y != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[1], this->m_channel_status.gmode[14], chn_axis_y);
            if(chn_axis_z != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[2], this->m_channel_status.gmode[14], chn_axis_z);

            //发送数据给HMI
            this->SendSimulateDataToHmi(type, data);
        }else if(comp_data.type == ARC2_TARGET || comp_data.type == ARC3_TARGET){  //G02/G03
            data.type = comp_data.type;
            data.pos[0] = comp_data.target[0];
            data.pos[1] = comp_data.target[1];
            data.pos[2] = comp_data.target[2];   //目标位置

            //转换为机械坐标系
            if(chn_axis_x != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[0], this->m_channel_status.gmode[14], chn_axis_x);
            if(chn_axis_y != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[1], this->m_channel_status.gmode[14], chn_axis_y);
            if(chn_axis_z != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[2], this->m_channel_status.gmode[14], chn_axis_z);
            this->SendSimulateDataToHmi(type, data);  //发送目标位置

            //圆心
            data.type = ARC_CENTER;
            data.pos[0] = comp_data.center[0];
            data.pos[1] = comp_data.center[1];
            data.pos[2] = comp_data.center[2];
            //转换为机械坐标系
            if(chn_axis_x != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[0], this->m_channel_status.gmode[14], chn_axis_x);
            if(chn_axis_y != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[1], this->m_channel_status.gmode[14], chn_axis_y);
            if(chn_axis_z != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[2], this->m_channel_status.gmode[14], chn_axis_z);
            this->SendSimulateDataToHmi(type, data);  //发送圆心坐标

            //半径
            data.type = ARC_RADIUS;
            data.pos[0] = comp_data.radius;
            data.pos[1] = comp_data.plane;
            data.pos[2] = 0;
            this->SendSimulateDataToHmi(type, data);  //发送圆半径，包含优劣弧及点信息
        }
    }

    return true;
}


/**
 * @brief 向MC发送编译G代码数据
 * @param msg : 待处理消息
 * @param flag_block : 是否需要强制附上快结束标志
 * @return true--成功   false--MC模块缓冲满
 */
bool ChannelControl::OutputData(RecordMsg *msg, bool flag_block){
    //	if(!this->m_p_mc_comm->CanWriteGCode(m_n_channel_index)){
    //		m_n_send_mc_data_err++;
    //		return false;
    //	}


    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){   //轮廓仿真和刀路仿真时直接发送数据给HMI
        return this->OutputSimulateData(msg);
    }

    //MC缓冲的运动数据+PL中缓冲的数据不能大于MC的运动数据缓冲容量，防止MDA数据被阻塞
    if(this->m_channel_status.chn_work_mode == AUTO_MODE){
        
        uint32_t gcode_fifo_count = 30;
        if(!this->m_b_mc_on_arm)
            gcode_fifo_count = m_p_mc_comm->ReadGCodeFifoCount(m_n_channel_index);

        if((m_channel_mc_status.buf_data_count + gcode_fifo_count) >= m_n_mc_auto_buf_max){
            m_n_send_mc_data_err++;
            //			printf("ChannelControl::OutputData, return false, buf_count=%hu, fifo=%u, max=%hu\n", m_channel_mc_status.buf_data_count,
            //					m_p_mc_comm->ReadGCodeFifoCount(m_n_channel_index), m_n_mc_auto_buf_max);
            return false;
        }else{
            //			printf("ChannelControl::OutputData, return true, buf_count=%hu, fifo=%u, max=%hu\n", m_channel_mc_status.buf_data_count,
            //								m_p_mc_comm->ReadGCodeFifoCount(m_n_channel_index), m_n_mc_auto_buf_max);
        }
    }

    GCodeFrame data_frame;   	//G代码数据帧

    //	bool is_last = false;

    if(msg->GetFrameIndex() == 0){
        msg->SetFrameIndex(m_n_frame_index);
        data_frame.data.frame_index = this->m_n_frame_index++;
        if(m_n_frame_index == 0)
            m_n_frame_index = 1;
    }else{
        data_frame.data.frame_index = msg->GetFrameIndex();
    }

    //处理插补轴的运动指令
    bool flag = this->m_n_hw_trace_state==REVERSE_TRACE?true:false;    //是否反向引导
    if(msg->GetOutputData(&data_frame, m_mask_intp_axis, flag) == 1)  //无要发送的运动数据
        return true;

    //设置模态信息
    data_frame.data.mode = this->m_mc_mode_exec.all;
    //	if(data_frame.data.mode & 0x03){
    //		printf("Gmode 02 is G18~~~~~~outputdata: 0x%x, 0x%x\n", data_frame.data.mode, m_mc_mode_exec.all);
    //	}

    //设置EXT_TYPE字段
    //最后一个数据，修改Ext_Type中的bit0，表示block已结束
    if(msg->CheckFlag(FLAG_BLOCK_OVER) || flag_block){
        data_frame.data.ext_type |= 0x0001;
        //printf("line %llu send block over@@@@@@@\n", msg->GetLineNo());
        //	is_last = true;
    }
    //bit2bit1
    switch(msg->GetMsgType()){
    case ARC_MSG:{
        //设置当前平面
        uint16_t plane = this->m_mc_mode_exec.bits.mode_g17;   //
        if(plane == 2) //YZ平面，G19
            data_frame.data.ext_type |= 0x02;
        else if(plane == 1)  //XZ平面，G18
            data_frame.data.ext_type |= 0x04;
        break;
    }
    case LINE_MSG:{
#ifdef USES_WOOD_MACHINE
        if(this->m_p_g_reg->QDE == 1){  //快钻功能激活
            LineMsg *linemsg = static_cast<LineMsg *>(msg);
            uint8_t z_chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
            uint32_t msk = 0x01;
            if(linemsg->GetAxisMoveMask() == (msk<<z_chn_axis)){   //只有Z轴移动时
                data_frame.data.cmd = MOVE_G00;   //强制将G01指令替换为G00指令
                data_frame.data.feed = 0;

                if(m_p_channel_config->rapid_mode == 1){
                    data_frame.data.ext_type |= 0x02;	//直线定位
                }
            }else{
                //bit2bit1设置插补模式：00-XYZ插补   01-所有轴插补
                if(m_p_channel_config->intep_mode == 1){
                    data_frame.data.ext_type |= 0x02;   //所有轴插补
                }
            }
        }else{
            //bit2bit1设置插补模式：00-XYZ插补   01-所有轴插补
            if(m_p_channel_config->intep_mode == 1){
                data_frame.data.ext_type |= 0x02;   //所有轴插补
            }
        }
#else
        //bit2bit1设置插补模式：00-XYZ插补   01-所有轴插补
        if(m_p_channel_config->intep_mode == 1){
            data_frame.data.ext_type |= 0x02;   //所有轴插补
        }
#endif
        break;
    }
    case COMPENSATE_MSG:
    case SKIP_MSG:
        //bit2bit1设置插补模式：00-XYZ插补   01-所有轴插补
        if(m_p_channel_config->intep_mode == 1){
            data_frame.data.ext_type |= 0x02;   //所有轴插补
        }
        // @add zk  不指定类型会使 MC 空运行
        if(msg->GetMsgType() == COMPENSATE_MSG)
            data_frame.data.cmd = ((CompensateMsg*)msg)->GetMoveType();   // G43根据之前模态确定移动类型
        else
            data_frame.data.cmd = 1;   // G31需要强制设为G01
        break;
    case RAPID_MSG:
    case COORD_MSG:
        if(m_p_channel_config->rapid_mode == 1){
            data_frame.data.ext_type |= 0x02;	//直线定位
        }
        break;
    default:
        break;
    }

    //bit6  准停标志：0-非准停   1--准停
    if(this->m_p_compiler->IsExactStop()){
        data_frame.data.ext_type |= 0x40; 	//准停
    }

    //bit7 0--自动数据   1--MDA数据
    if(m_channel_status.chn_work_mode == MDA_MODE ||
            this->m_b_manual_tool_measure || //自动对刀
            this->m_b_manual_call_macro   //手动调用宏程序
        #ifdef USES_ADDITIONAL_PROGRAM
            || (m_n_add_prog_type == CONTINUE_START_ADD)
        #endif
            )   //断点继续前置程序调用
        data_frame.data.ext_type |= 0x80; 	//MDA数据


    //	if(msg->GetMsgType() == ARC_MSG){
    //		printf("output arc msg: cmd = %hu, ext_type=0x%hx, mode=0x%x, line=%u\n", data_frame.data.cmd, data_frame.data.ext_type, data_frame.data.mode,
    //				data_frame.data.line_no);
    //	}


    //发送数据至MC
    bool res = false;
    if(!this->m_b_mc_on_arm){
        // 查看 msg的 ext type mc会因ext的值对不上而空执行
    	printf("ext_type: %d\n", data_frame.data.ext_type);
    	res = m_p_mc_comm->WriteGCodeData(m_n_channel_index, data_frame);
    }else{

    	res = this->m_p_mc_arm_comm->WriteGCodeData(m_n_channel_index, data_frame);
    }
    if(!res){
        //发送失败
        m_n_send_mc_data_err++;
        //		printf("failed to send out data frame: cmd = %d, line = %d\n", data_frame.data.cmd, data_frame.data.line_no);
        return false;
    }
    else{

        //	if(is_last)
        //		m_b_mc_need_start = true;
        //		printf("send data out : cmd = %d, feed = %d, ext = 0x%04x, line = %d, tar(%lld, %lld, %lld)\n", data_frame.data.cmd,data_frame.data.feed,
        //				data_frame.data.ext_type, data_frame.data.line_no, data_frame.data.pos0, data_frame.data.pos1, data_frame.data.pos2);


    }

    //将数据压入发送缓冲区
    //	if(!AddOutputData(data_frame)){
    //		printf("OutputData false\n");
    //		return false;
    //	}

    //	printf("send out data frame: cmd = %d, line = %d\n", data_frame.data.cmd, data_frame.data.line_no);

    return true;
}

/**
 * @brief 将运动控制数据帧压入缓冲
 * @param data : 待发送给MC模块的数据帧
 * @return  true--成功    false--失败，缓冲已满
 */
//bool ChannelControl::AddOutputData(const GCodeFrame &data){
//	int cur_count = this->m_p_output_buffer->BufLen();
//
//	//先把之前的数据发送出去
//	GCodeFrame *send_data = nullptr;
//	for(int i = 0; i < cur_count; i++){
//		send_data = this->m_p_output_buffer->ReadDataPtr(0);
//		if(!m_p_mc_comm->WriteGCodeData(*send_data)){
//			//发送失败
//			return false;
//		}
//		else{
//			printf("send data out : cmd = %d, feed = %d, ext = 0x%04x, line = %d\n", send_data->data.cmd,
//					send_data->data.feed, send_data->data.ext_type, send_data->data.line_no);
//		}
//		m_p_output_buffer->RemoveHead();
//	}
//
//
//	m_p_output_buffer->WriteData(&data, 1);
//
//	return true;
//}

/**
 * @brief 将运动数据缓冲中的数据都输出，置位块结束标志
 * @return true--成功    false--失败，缓冲已满
 */
//bool ChannelControl::OutputLastBlockItem(){
//	int cur_count = this->m_p_output_buffer->BufLen();
//
//	GCodeFrame *send_data = nullptr;
//	while(cur_count > 0){
//		send_data = this->m_p_output_buffer->ReadDataPtr(0);
//		if(cur_count == 1){
//			//最后一个数据，修改Ext_Type中的bit0，表示block已结束
//			send_data->data.ext_type |= 0x0001;
//
//			m_b_mc_need_start = true;
//		}
//		if(!m_p_mc_comm->WriteGCodeData(*send_data)){
//			//发送失败
//			return false;
//		}
//		else{
//			printf("send data out last : cmd = %d, feed = %d, ext = 0x%04x, line = %d\n", send_data->data.cmd,
//								send_data->data.feed, send_data->data.ext_type, send_data->data.line_no);
//		}
//		m_p_output_buffer->RemoveHead();
//
//		cur_count--;
//	}
//	return true;
//}

/**
 * @brief 将运动数据缓冲中的数据都输出,不置位块结束标志
 * @return true--成功    false--失败，缓冲已满
 */
//bool ChannelControl::OutputAllData(){
//	int cur_count = this->m_p_output_buffer->BufLen();
//
//	GCodeFrame *send_data = nullptr;
//	while(cur_count > 0){
//		send_data = this->m_p_output_buffer->ReadDataPtr(0);
//
//		if(!m_p_mc_comm->WriteGCodeData(*send_data)){
//			//发送失败
//			return false;
//		}
//		else{
//			printf("send data out all : cmd = %d, feed = %d, ext = 0x%04x, line = %d\n", send_data->data.cmd,
//								send_data->data.feed, send_data->data.ext_type, send_data->data.line_no);
//		}
//		m_p_output_buffer->RemoveHead();
//
//		cur_count--;
//	}
//	return true;
//}

/**
 * @brief 将DPoint对象变换为DPointChn对象
 * @param src
 * @param tar
 */
void ChannelControl::DPoint2DPointChn(const DPoint &src, DPointChn &tar){
    int count = 0;
    uint32_t mask = 0x01;

    for(int i = 0; i < this->m_p_channel_config->chn_axis_count && count < 8; i++){
        if(this->m_mask_intp_axis & mask){
            tar.m_df_point[i] = src.GetAxisValue(count);
        }
        //count ++;  // @modify zk 20220905
    }
}

/**
 * @brief 将DPointChn对象变换为DPoint对象
 * @param src
 * @param tar
 */
void ChannelControl::DPointChn2DPoint(const DPointChn &src, DPoint &tar){

}

/**
 * @brief 同步已编译的轴移动指令的位置
 * @param pos : 新位置
 * @return : true--表示有轴移动指令   false--表示没有轴移动指令
 */
bool ChannelControl::RefreshOuputMovePos(DPointChn &pos){
    RecordMsg *msg = nullptr;
    ListNode<RecordMsg *> *node = m_p_output_msg_list->HeadNode();

    if(this->m_channel_status.chn_work_mode == AUTO_MODE && this->m_p_last_output_msg != nullptr){//支持手轮反向引导后，运行完的轴移动指令不会立刻出队列
        node = m_p_last_output_msg->next;
    }
    CodeMsgType msg_type = NORMAL_MSG;
    bool res = false;
    DPointChn pp = pos;

    //	printf("ChannelControl::RefreshOuputMovePos, count=%d\n", m_p_output_msg_list->GetLength());
    while(node != nullptr){
        msg = static_cast<RecordMsg *>(node->data);

        msg_type = msg->GetMsgType();

        //		printf("refreshoutputmovepos, msg_type=%hhu, line=%llu\n", msg_type, msg->GetLineNo());
        switch(msg_type){
        case LINE_MSG:
            ((LineMsg *)msg)->RefreshTargetPos(pp);
            pp = ((LineMsg *)msg)->GetTargetPos();
            res = true;
            break;
        case RAPID_MSG:
            ((RapidMsg *)msg)->RefreshTargetPos(pp);
            pp = ((RapidMsg *)msg)->GetTargetPos();
            res = true;
            break;
        case ARC_MSG:
            ((ArcMsg *)msg)->RefreshTargetPos(pp);
            pp = ((ArcMsg *)msg)->GetTargetPos();
            break;
        case COMPENSATE_MSG:
            if(msg->IsMoveMsg()){
                ((CompensateMsg *)msg)->RefreshTargetPos(pp);
                pp = ((CompensateMsg *)msg)->GetTargetPos();
                res = true;
                //			printf("comp msg, line=%llu\n", msg->GetLineNo());
            }
            break;
        default:
            break;
        }
        node = node->next;  //取下一个消息
    }

    //同步编译器分块队列中的移动指令
    res |= this->m_p_compiler->RefreshBlockMovePos(pp);

    return res;
}

/**
 * @brief 同行是否有轴移动指令
 * @return
 */
bool ChannelControl::IsMoveMsgLine(){
    RecordMsg *msg = nullptr;
    ListNode<RecordMsg *> *node = m_p_output_msg_list->HeadNode();
    if(this->m_channel_status.chn_work_mode == AUTO_MODE && this->m_p_last_output_msg != nullptr){
        node = m_p_last_output_msg->next;
    }
    bool res = false;
    while(node != nullptr){
        msg = static_cast<RecordMsg *>(node->data);

        if(msg->IsMoveMsg()){
            res = true;
            break;
        }else if(msg->IsLineLastMsg())
            break;

        node = node->next;  //取下一个消息
    }
    return res;
}

/**
 * @brief NC文件是否存在
 * @param file_name : NC加工文件名
 * @return  true--存在   false--不存在
 */
bool ChannelControl::IsNcFileExist(char *file_name){
    char path[kMaxPathLen] = {0};
    strcpy(path, PATH_NC_FILE);

    strcat(path, file_name);

    if(access(path, F_OK) == -1){	//不存在，返回失败
        return false;
    }
    return true;
}

/**
 * @brief 是否在运行中，包括自动运行，加工仿真，刀路仿真
 * @return true--运行中   false--空闲中
 */
bool ChannelControl::IsMachinRunning(){
    if(m_channel_status.machining_state == MS_RUNNING ||
            m_channel_status.machining_state == MS_OUTLINE_SIMULATING ||
            m_channel_status.machining_state == MS_TOOL_PATH_SIMULATING ||
            m_channel_status.machining_state == MS_MACHIN_SIMULATING ||
            m_b_manual_tool_measure ||
            m_b_manual_call_macro
        #ifdef USES_ADDITIONAL_PROGRAM
            || (m_n_add_prog_type == CONTINUE_START_ADD)
        #endif
            )
        return true;
    return false;
}

/**
 * @brief 是否单段模式,只有在自动模式下，单段才有效
 * @return  true--是   false--不是
 */
bool ChannelControl::IsStepMode(){
    if(m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE) &&     //单段有效
            (m_channel_status.chn_work_mode == AUTO_MODE) &&               //自动模式
            (m_simulate_mode == SIM_NONE)    //非仿真状态
        #ifdef USES_ADDITIONAL_PROGRAM
            && (m_n_add_prog_type == NONE_ADD)    //非附加程序运行状态
        #endif
            ){
        if(this->m_n_macroprog_count == 0 || this->m_p_general_config->debug_mode > 0) //非宏程序调用或者调试模式打开
            return true;
    }

    return false;
}

/**
 * @brief 命令消息执行模块
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteMessage(){
    int count = m_p_output_msg_list->GetLength();
    if(count == 0){
    	return true;
    }

    bool res = false;
    RecordMsg *msg = nullptr;
    ListNode<RecordMsg *> *node = nullptr;   //
    ListNode<RecordMsg *> *node_next = nullptr;
    CodeMsgType msg_type = NORMAL_MSG;
    bool flag = false;
    int end_cmd = 0;    //0--表示非结束指令   30--表示M30    99--表示M99
    bool pause_flag = false;


    if(this->m_channel_status.chn_work_mode == AUTO_MODE &&
            this->m_simulate_mode == SIM_NONE     //非仿真状态
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type != CONTINUE_START_ADD
        #endif
            ){

        if(this->m_n_restart_mode != NOT_RESTART){//加工复位流程，前面扫描的数据不保留
            node = m_p_output_msg_list->HeadNode();

        }else{
            if(this->m_p_last_output_msg == nullptr){
                if(this->m_n_hw_trace_state == REVERSE_TRACE){
                    node = m_p_output_msg_list->TailNode();    //之前为空队列则从尾部开始处理
                    //	printf("reverse from tail#######\n");
                }else{
                    node = m_p_output_msg_list->HeadNode();    //之前为空队列则从头部开始处理
                    //	printf("normal from head@@@@@\n");
                }

            }else{//否则从下一个msg开始处理
                if(this->m_n_hw_trace_state == REVERSE_TRACE){
                    node = m_p_last_output_msg->pre;
                }else{
                    node = m_p_last_output_msg->next;
                }
            }

            //如果手轮反向跟踪到头，则给出提示
            if(this->m_n_hw_trace_state == REVERSE_TRACE && node == nullptr){
                int count = 0;
                if(!this->m_b_mc_on_arm){
                    //更新当前MC的运行模式
                    this->m_p_mc_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);
                    //更新MC当前缓冲数据量
                    this->m_p_mc_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);
                    while(m_channel_mc_status.cur_mode == MC_MODE_MANUAL && this->m_channel_mc_status.buf_data_count == 0 && count < 3){

                        usleep(2000);  //休眠2ms

                        //更新当前MC的运行模式
                        this->m_p_mc_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);
                        //更新MC当前缓冲数据量
                        this->m_p_mc_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);

                        count++;
                    }
                }else{
                    //更新当前MC的运行模式
                    this->m_p_mc_arm_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);
                    //更新MC当前缓冲数据量
                    this->m_p_mc_arm_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);
                    while(m_channel_mc_status.cur_mode == MC_MODE_MANUAL && this->m_channel_mc_status.buf_data_count == 0 && count < 3){

                        usleep(2000);  //休眠2ms

                        //更新当前MC的运行模式
                        this->m_p_mc_arm_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);
                        //更新MC当前缓冲数据量
                        this->m_p_mc_arm_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);

                        count++;
                    }
                }

                if(m_channel_mc_status.cur_mode == MC_MODE_MANUAL && this->m_channel_mc_status.buf_data_count == 0 && count >= 3){
                    CreateError(ERR_HW_REV_OVER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                    return res;
                }
            }
        }

#ifdef USES_WOOD_MACHINE
        if(node != nullptr)
            this->PreExecSpindleCmd(this->m_channel_rt_status.line_no);   //执行主轴预启动
#endif

    }else{
        node = m_p_output_msg_list->HeadNode();
    }

    while(node != nullptr){
        msg = static_cast<RecordMsg *>(node->data);
        end_cmd = 0;
        if(m_channel_status.chn_work_mode == AUTO_MODE &&
                m_n_restart_mode != NOT_RESTART && this->m_n_restart_step == 1 &&
                this->m_mode_restart.sub_prog_call == 0 &&     //在主程序中
                msg->GetLineNo() >= this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
                && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
                ){  //运行到复位指定行，需要先重建模态
            this->DoRestart(msg->GetLineNo());
            this->m_n_run_thread_state = WAIT_EXECUTE;
            return res;
        }

        flag = false;
        msg_type = msg->GetMsgType();


        // @test zk
        static uint64_t line_no = 0;
        static int type = 0;
        if(line_no != msg->GetLineNo() || type != msg->GetMsgType()){
            line_no = msg->GetLineNo();
            type = msg->GetMsgType();
            //printf("---------->excute message line no %llu  msg type: %d flag: %d\n", line_no, msg_type, msg->GetFlags().all);
        }
        // @test zk

        switch(msg_type){
        case AUX_MSG:
#ifdef USES_WOOD_MACHINE
            res = ExecuteAuxMsg_wood(msg);
#else
            res = ExecuteAuxMsg(msg);
#endif
            end_cmd = ((AuxMsg *)msg)->GetMCode(0);
            break;
        case COORD_MSG:
            res = ExecuteCoordMsg(msg);
            break;
        case RAPID_MSG:
            if(m_channel_status.chn_work_mode == AUTO_MODE && m_n_hw_trace_state == REVERSE_TRACE){
                node_next = node->pre;
                if(node_next != nullptr){
                    if(!this->IsRunmoveMsg(node_next->data))
                        flag = true;
                }else
                    flag = true;
            }
            res = ExecuteRapidMsg(msg, flag);
            break;
        case LINE_MSG:
            if(m_channel_status.chn_work_mode == AUTO_MODE && m_n_hw_trace_state == REVERSE_TRACE){
                node_next = node->pre;
                if(node_next != nullptr){
                    if(!this->IsRunmoveMsg(node_next->data))
                        flag = true;
                }else
                    flag = true;
            }
            res = ExecuteLineMsg(msg, flag);
            break;
        case COMPENSATE_MSG:
            res = ExecuteCompensateMsg(msg);
            break;
        case MODE_MSG:
            res = ExecuteModeMsg(msg);
            break;
        case FEED_MSG:
            res = ExecuteFeedMsg(msg);
            break;
        case SPEED_MSG:
            res = ExecuteSpeedMsg(msg);
            break;
        case TOOL_MSG:
            res = ExecuteToolMsg(msg);
            break;
        case LOOP_MSG:
            res = ExecuteLoopMsg(msg);
            break;
        case ARC_MSG:
            if(m_channel_status.chn_work_mode == AUTO_MODE && m_n_hw_trace_state == REVERSE_TRACE){
                node_next = node->pre;
                if(node_next != nullptr){
                    if(!this->IsRunmoveMsg(node_next->data))
                        flag = true;
                }else
                    flag = true;
            }
            res = this->ExecuteArcMsg(msg, flag);
            break;
        case ERROR_MSG:
            res = this->ExecuteErrorMsg(msg);
            break;
        case SUBPROG_CALL_MSG:
            res = this->ExecuteSubProgCallMsg(msg);
            break;
        case MACRO_MSG:
            res = this->ExecuteMacroCmdMsg(msg);
            break;
        case SUBPROG_RETURN_MSG:
            res = this->ExecuteSubProgReturnMsg(msg);
            break;
        case POLAR_INTP_MSG:
            res = this->ExecutePolarIntpMsg(msg);
            break;
        case CLEAR_CIRCLE_POS_MSG:
            res = this->ExecuteClearCirclePosMsg(msg);
            break;
        case TIME_WAIT_MSG:
            res = this->ExecuteTimeWaitMsg(msg);
            break;
        case REF_RETURN_MSG:
            res = this->ExecuteRefReturnMsg(msg);
            break;
#ifdef USES_SPEED_TORQUE_CTRL			
        case SPEED_CTRL_MSG:
            res = this->ExecuteSpeedCtrlMsg(msg);
            break;
        case TORQUE_CTRL_MSG:
            res = this->ExecuteTorqueCtrlMsg(msg);
            break;
#endif		
        case SKIP_MSG:
            res = this->ExecuteSkipMsg(msg);
            break;
        case MACRO_PROG_CALL_MSG:
            res = this->ExecuteMacroProgCallMsg(msg);
            break;
        case AUTO_TOOL_MEASURE_MSG:
            res = this->ExecuteAutoToolMeasureMsg(msg);
            break;
        case RESTART_OVER_MSG:
            res = this->ExecuteRestartOverMsg(msg);
            break;
        case INPUT_MSG:
            res = this->ExecuteInputMsg(msg);
            break;
        case EXACT_STOP_MSG:
            res = this->ExecuteExactStopMsg(msg);
            break;
        default:
            break;
        }

        //		if(this->m_n_add_prog_type == CONTINUE_START_ADD)
        //			printf("execute msg %d, line = %llu,  res = %d, fifo=%u\n", msg->GetMsgType(), msg->GetLineNo(), res, m_p_mc_comm->ReadGCodeFifoCount(m_n_channel_index));

        if(!res){
            if(m_n_run_thread_state != WAIT_RUN  && m_n_run_thread_state != WAIT_EXECUTE){ //当前非WAIT_RUN状态，将线程置为WAIT_EXECUTE状态
                this->m_n_run_thread_state = WAIT_EXECUTE;
                //		printf("set WAIT_EXECUTE, line = %llu\n", msg->GetLineNo());
            }

            break;
        }else{//执行结束
            if(this->m_n_restart_mode != NOT_RESTART        //加工复位
        #ifdef USES_ADDITIONAL_PROGRAM
                    && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
                    ){

                if(this->m_n_restart_step == 1){  //扫描阶段的数据执行完后直接删除
                    this->m_p_output_msg_list->Delete(node);   //删除此节点命令
                    node = m_p_output_msg_list->HeadNode();  //取下一个消息
                    return res;
                }
                //				else if(this->m_n_restart_step == 2){//FOR TEST
                //					g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "Restart to exec msg %d, line = %llu,  res = %d\n", msg->GetMsgType(), msg->GetLineNo(), res);
                //				}
            }

            if(msg->IsNeedWaitMsg() && (this->m_simulate_mode == SIM_NONE || this->m_simulate_mode == SIM_MACHINING)){  //非仿真、加工仿真才需要给MC发送启动指令
                m_b_mc_need_start = true;
                //				printf("set mc need start true in exec~~~~~\n");
            }


            if((this->m_channel_status.machining_state == MS_RUNNING || this->m_b_manual_tool_measure
                || this->m_b_manual_call_macro
    #ifdef USES_ADDITIONAL_PROGRAM
                || (m_n_add_prog_type == CONTINUE_START_ADD)
    #endif
                ) &&    //运行状态或者手动对刀状态
                    msg->IsMoveMsg() && this->m_b_mc_need_start){
                //				printf("move data send start, msg type = %d\n", msg->GetMsgType());
                this->StartMcIntepolate();
            }
            //			else {
            //				printf("ismovemsg:%hhu, m_b_mc_need_start=%hhu\n", msg->IsMoveMsg(), m_b_mc_need_start);
            //			}


            if(IsStepMode()){//单段模式, 非宏程序调用或者打开了调试模式
                if(msg->CheckFlag(FLAG_LAST_REC)){
                    if((!msg->IsMoveMsg() || msg_type == AUX_MSG || msg_type == REF_RETURN_MSG) &&
                            !msg->IsEndMsg()){	//单步模式下，运行完一行的最后一条代码后暂停
                        printf("PAUSED in execute\n");
                        m_n_run_thread_state = PAUSE;
                        pause_flag = true;
                    }else if(msg->IsMoveMsg()){
                        this->m_b_need_change_to_pause = true;   //mc运行完成后切换暂停状态
                    }
                }
                //				if(msg->GetFlag(FLAG_LAST_REC))
                //					m_b_step_exec = false;
            }

            if(msg->IsEndMsg()){	//处理M02/M30消息
                this->m_n_run_thread_state = STOP;
            }else if(m_n_run_thread_state != WAIT_RUN && this->m_error_code == ERR_NONE && this->IsMachinRunning() && !pause_flag){
                this->m_n_run_thread_state = RUN;
                //	printf("execute message change state to RUN!!!!!\n");
            }else if(m_n_run_thread_state == WAIT_EXECUTE &&
                     (m_channel_status.machining_state == MS_PAUSED || m_channel_status.machining_state == MS_PAUSING || pause_flag)){
                m_n_run_thread_state = PAUSE;
            }else{
                //	printf("execute msg over : m_n_run_thread_state=%hhu, machining_state=%hhu\n", m_n_run_thread_state, m_channel_status.machining_state);
            }

#ifdef USES_ADDITIONAL_PROGRAM
            //复位附加程序执行状态
            if((m_n_add_prog_type == NORMAL_START_ADD || m_n_add_prog_type == RESET_START_ADD) &&
                    (m_n_sub_count_bak == m_n_subprog_count) &&
                    end_cmd == 99){
                this->m_n_add_prog_type = NONE_ADD;
                this->m_n_sub_count_bak = 0;
                if(this->IsStepMode())
                    this->SetMcStepMode(true);

                //				printf("########add execute over11111\n");
            }else if(m_n_add_prog_type == CONTINUE_START_ADD &&
                     end_cmd == 30){
                this->m_n_add_prog_type = NONE_ADD;  //结束前置程序调用

                if(this->IsStepMode())
                    this->SetMcStepMode(true);
                //				printf("########add exectue over2222\n");
            }
#endif

            if(m_channel_status.chn_work_mode == AUTO_MODE &&
                    m_simulate_mode == SIM_NONE      //非仿真状态
        #ifdef USES_ADDITIONAL_PROGRAM
                    && m_n_add_prog_type == NONE_ADD
        #endif
                    ){
                //				if(this->m_n_restart_mode != NOT_RESTART){//复位状态过程中的数据都删除
                //					this->m_p_output_msg_list->Delete(node);   //删除此节点命令
                //					node = m_p_output_msg_list->HeadNode();  //取下一个消息
                //				}else{
                //根据msg类型判断是否需要清空之前的缓冲
                if(this->IsBufClearMsg(msg)){
                    //	g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "@@@@@@@@#####clear buf message,line = %llu\n", msg->GetLineNo());
                    this->m_p_output_msg_list->ClearBefore(node);
                    this->m_p_last_output_msg = nullptr;
                    node = this->m_p_output_msg_list->HeadNode();
                }else{
                    this->m_p_last_output_msg = node;
                    //				printf("set last output:%d\n", node);
                    if(this->m_n_hw_trace_state == REVERSE_TRACE){
                        node = node->pre;
                        //		printf("reverse output msg, type = %hhu, line=%llu\n", msg->GetMsgType(), msg->GetLineNo());
                    }else{
                        ListNode<RecordMsg *> *node_tmp = node;
                        node = node->next;
                        if(msg->GetMsgType() == FEED_MSG){
                            this->m_p_last_output_msg = node_tmp->pre;
                            this->m_p_output_msg_list->Delete(node_tmp);   //删除FEED命令包
                            //			printf("delete feed msg##@@@\n");
                        }

                    }
                }
                //				}

            }else{
                //		printf("delete node msg, type = %d, line=%llu\n", msg->GetMsgType(), msg->GetLineNo());
                this->m_p_output_msg_list->Delete(node);   //删除此节点命令
                node = m_p_output_msg_list->HeadNode();  //取下一个消息
            }

        }

    }
    if(pause_flag){
        SetMachineState(MS_PAUSED);
    }
    //printf("exit execute: %d  res: %d\n", m_n_run_thread_state, res);
    return res;
}

/**
 * @brief 实际执行辅助指令消息
 * @param msg : 指令消息
 * @return true--成功  false--失败，PL中缓冲已满或者等待运行到位
 */
bool ChannelControl::ExecuteAuxMsg(RecordMsg *msg){

	AuxMsg *tmp = (AuxMsg *)msg;
    uint8_t m_count = tmp->GetMCount();   //一行中M代码总数
    uint8_t m_index = 0;
    int mcode = 0;

    if(this->m_n_restart_mode != NOT_RESTART &&
            tmp->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        for(m_index = 0; m_index < m_count; m_index++){
            mcode = tmp->GetMCode(m_index);

            if(mcode == 6){//换刀
                this->m_mode_restart.cur_tool = this->m_mode_restart.cur_tcode;
            }else if(mcode == 3){
                this->m_mode_restart.rated_spindle_speed = this->m_mode_restart.cur_scode;
                this->m_mode_restart.spindle_dir = SPD_DIR_POSITIVE;
            }else if(mcode == 4){
                this->m_mode_restart.rated_spindle_speed = this->m_mode_restart.cur_scode;
                this->m_mode_restart.spindle_dir = SPD_DIR_NEGATIVE;
            }else if(mcode == 5){
                this->m_mode_restart.rated_spindle_speed = 0;
                this->m_mode_restart.spindle_dir = SPD_DIR_STOP;
            }else if(mcode == 2 || mcode == 30 || mcode == 99){  //直接结束，切换到READY状态
                if(mcode == 99 && m_mode_restart.sub_prog_call > 0)
                    this->m_mode_restart.sub_prog_call--;
                else{
                    ResetMcLineNo();//复位MC模块当前行号
                    this->SetCurLineNo(1);

                    CompileOver();
                    this->m_n_restart_mode = NOT_RESTART;
                    this->m_n_restart_line = 0;
                    this->m_n_restart_step = 0;
                    this->m_p_compiler->SetRestartPara(0, m_n_restart_mode);   //复位编译器的加工复位标志

                    this->ClearHmiInfoMsg();   //清除HMI的提示信息
                }
            }
        }

        return true;
    }

    if(this->m_simulate_mode != SIM_NONE){  //仿真模式
        //设置当前行号
        SetCurLineNo(msg->GetLineNo());

        mcode = tmp->GetMCode(m_index);

        printf("mcode : %d\n", mcode);

        if(mcode == 2 || mcode == 30){
            ResetMcLineNo();//复位MC模块当前行号
            this->SetCurLineNo(1);

            this->m_p_f_reg->STL = 0;
            this->m_p_f_reg->SPL = 0;
            this->m_p_f_reg->OP = 0;

            CompileOver();
#ifdef 	USES_SIMULATION_TEST
            if(this->m_file_sim_data > 0){
                close(this->m_file_sim_data);
                m_file_sim_data = -1;
            }
#endif
        }else if(mcode == 99){
            if(m_n_subprog_count > 0){
                m_n_subprog_count--;
                m_b_ret_from_macroprog = false;

                this->m_n_run_thread_state = RUN;
            }
            else{
                ResetMcLineNo();//复位MC模块当前行号
                this->SetCurLineNo(1);

                this->m_p_f_reg->STL = 0;
                this->m_p_f_reg->SPL = 0;
                this->m_p_f_reg->OP = 0;

                this->m_n_run_thread_state = STOP;
                CompileOver();
#ifdef 	USES_SIMULATION_TEST
                if(this->m_file_sim_data > 0){
                    close(this->m_file_sim_data);
                    m_file_sim_data = -1;
                }
#endif
            }
        }
        return true;
    }


    if(tmp->IsFirstExec()){
        //首先将缓冲中的所有待发送指令发送给MC
        //		if(!OutputLastBlockItem()){
        //			//PL中的FIFO已满，发送失败
        //			return false;
        //		}

        int limit = 3;
        if(this->IsStepMode())
            limit = 5;	//单步模式需要多次验证，因为状态切换有延时

        //等待MC分块的插补到位信号，以及MI的运行到位信号
        int count = 0;
        bool block_over = false;
        while(1){
            block_over = CheckBlockOverFlag();
            if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
                //printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
                return false;    //还未运行到位
            }
            else if(++count < limit){
                usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
                //printf("execute aus msg: blockflag=%d, count = %d\n",  block_over, count);

            }else
                break;
        }

        if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }


        //设置当前行号
        if(tmp->GetMCode(0) != 99 && !m_b_ret_from_macroprog)
            SetCurLineNo(msg->GetLineNo());

    }

    bool bRet = true;
    struct timeval time_now;

    //llx test
    struct timeval time_now_test;
    unsigned int time_elpase_test = 0;

    unsigned int time_elpase = 0;
    //	uint64_t mask = 0;
    for(m_index = 0; m_index < m_count; m_index++){
        if(tmp->GetExecStep(m_index) == 0xFF)
            continue;       //已执行完成则直接跳过

        mcode = tmp->GetMCode(m_index);

        if(mcode != 300) // M300不需要显示
            NotifyHmiMCode(mcode);

        switch(mcode){
        case 30:  	//M30
        case 2:		//M02
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M30\n");

                //TODO 将代码发送给PMC
                this->SendMCodeToPmc(mcode, m_index);
                if(mcode == 30)
                    this->m_p_f_reg->DM30 = 1;  //置位DM30
                else
                    this->m_p_f_reg->DM02 = 1;  //置位DM02

                gettimeofday(&m_time_m_start[m_index], NULL);   //


                //五轴机床需要对无限旋转轴清整数圈
                //#ifdef USES_FIVE_AXIS_FUNC
                //			if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS){
                //				//向MI发送清整数圈位置指令
                //				m_n_mask_clear_pos = 0;
                //
                //				for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
                //					if(((m_mask_5axis_rot_nolimit>>i) & 0x01) == 0){
                //
                //						continue;
                //					}
                //					this->SendMiClearPosCmd(GetPhyAxis(i)+1, 360*1000);
                //				}
                //			}
                //#endif

                tmp->IncreaseExecStep(m_index);

            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index, true);    //置位选通信号

                gettimeofday(&m_time_m_start[m_index], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    gettimeofday(&m_time_m_start[m_index], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                //#ifdef USES_FIVE_AXIS_FUNC
                //			//等待MI设置完成
                //			if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS &&
                //					m_mask_5axis_rot_nolimit != 0){
                //				if(m_mask_5axis_rot_nolimit != m_n_mask_clear_pos){
                //					break;
                //				}
                //			}
                //#endif

                this->SetMFSig(m_index, false);    //复位选通信号

                if(mcode == 30)
                    this->m_p_f_reg->DM30 = 0;  //复位DM30
                else
                    this->m_p_f_reg->DM02 = 0;  //复位DM02

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index);

#ifdef USES_ADDITIONAL_PROGRAM
                if(this->m_n_add_prog_type == NONE_ADD){
#endif
                    if(mcode == 30){
                        ResetMcLineNo();//复位MC模块当前行号
                        this->SetCurLineNo(1);
                    }

                    this->m_p_f_reg->STL = 0;
                    this->m_p_f_reg->SPL = 0;
                    this->m_p_f_reg->OP = 0;

                    //工件计数加一
                    if(this->m_channel_status.chn_work_mode == AUTO_MODE){
//                        this->m_channel_status.workpiece_count++;
//                        g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
//                        this->m_channel_status.workpiece_count_total++;
//                        g_ptr_parm_manager->SetTotalWorkPiece(m_n_channel_index, m_channel_status.workpiece_count_total);
//                        this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);  //通知HMI更新加工计数

//                        if (m_channel_status.workpiece_require != 0 && m_channel_status.workpiece_count >= m_channel_status.workpiece_require)
//                        {//已到达需求件数
//                            CreateError(ERR_REACH_WORK_PIECE, INFO_LEVEL, CLEAR_BY_MCP_RESET);
//                        }
                        this->ResetMode();   //模态恢复默认值

                        //激活工件坐标系参数
                        if(g_ptr_parm_manager->ActiveCoordParam(m_n_channel_index))
                            this->SetMcCoord(true);

                        //激活待生效的刀具偏置数据
                        if(g_ptr_parm_manager->ActiveToolOffsetParam(m_n_channel_index)){
                            if(this->m_channel_status.cur_h_code > 0)
                                this->SetMcToolOffset(true);
                        }

                        g_ptr_parm_manager->ActiveNewStartParam();

                    }
#ifdef USES_ADDITIONAL_PROGRAM
                }
#endif

                CompileOver();
                this->SetMiSimMode(false);  //复位MI仿真状态

                //#ifdef USES_FIVE_AXIS_FUNC
                //
                //			if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS &&
                //					m_mask_5axis_rot_nolimit != 0){
                //				//从MC同步工件坐标
                //				this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
                //
                //				m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
                //				m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
                //
                //				this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
                //
                //		//		printf("intp pos (%lf, %lf, %lf, %lf)\n", m_channel_mc_status.intp_pos.x, m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z,
                //		//				m_channel_mc_status.intp_pos.a6);
                //
                //				//从MI同步机械坐标
                //				this->m_p_channel_engine->SendMonitorData(false, false);
                //			}
                //#endif
                if(m_channel_status.chn_work_mode == AUTO_MODE){
#ifdef USES_ADDITIONAL_PROGRAM
                    if(this->m_n_add_prog_type == NONE_ADD){
                        this->SendMachOverToHmi();  //发送加工结束消息给HMI
                    }
#else
                    string msg = "结束加工程序(" + string(this->m_channel_status.cur_nc_file_name) + ")";
                    g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug, msg);

                    this->SendMachOverToHmi();  //发送加工结束消息给HMI
#endif
                }
                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态

                printf("execute M30 over\n");
            }

            break;
        case 0:		//M00无条件暂停
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M00\n");
                //TODO 将代码发送给PMC
                this->SendMCodeToPmc(mcode, m_index);
                this->m_p_f_reg->DM00 = 1;  //置位DM00

                gettimeofday(&m_time_m_start[m_index], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                gettimeofday(&m_time_m_start[m_index], NULL);
                this->SetMFSig(m_index, true);    //置位选通信号
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    gettimeofday(&m_time_m_start[m_index], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index, false);    //复位选通信号

                this->m_p_f_reg->DM00 = 0;  //复位DM00

                this->m_p_f_reg->STL = 0;
                this->m_p_f_reg->SPL = 1;

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index);


                this->PauseRunGCode();
                m_n_run_thread_state = PAUSE;

                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            }
            break;
        case 1:		//M01选择性暂停
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M01\n");
                //TODO 将代码发送给PMC
                this->SendMCodeToPmc(mcode, m_index);
                this->m_p_f_reg->DM01 = 1;  //置位DM01

                gettimeofday(&m_time_m_start[m_index], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index, true);    //置位选通信号

                gettimeofday(&m_time_m_start[m_index], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    gettimeofday(&m_time_m_start[m_index], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index, false);    //复位选通信号

                this->m_p_f_reg->DM01 = 0;  //复位DM01

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index);


                if(this->m_channel_status.func_state_flags.CheckMask(FS_OPTIONAL_STOP)){//选停状态
                    PauseRunGCode();

                    m_n_run_thread_state = PAUSE;
                    this->m_p_f_reg->STL = 0;
                    this->m_p_f_reg->SPL = 1;
                }

                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            }
            break;
        case 6:		//M06换刀
            if(tmp->GetExecStep(m_index) == 0){
                //TODO 将代码发送给PMC
                printf("start to execute M06\n");
                this->SendMCodeToPmc(mcode, m_index);

                gettimeofday(&m_time_m_start[m_index], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                gettimeofday(&m_time_m_start[m_index], NULL);
                this->SetMFSig(m_index, true);    //置位选通信号
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    gettimeofday(&m_time_m_start[m_index], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index, false);    //复位选通信号
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index);

#ifdef USES_WOOD_MACHINE
                this->m_n_ref_tool_life_delay = 3;
#endif

                //更新当前刀号
                //				this->m_channel_status.cur_tool = this->m_n_cur_tcode;
                //				g_ptr_parm_manager->SetCurTool(m_n_channel_index, m_channel_status.cur_tool);

#ifdef USES_WOOD_MACHINE
                //				this->SetMcToolLife();
#endif

                //				this->SendModeChangToHmi(T_MODE);


                printf("execute M06 over: cur_T = %hhu\n", m_channel_status.cur_tool);

                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            }

            break;
        case 36:    //换刀完成
        {
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M36, test\n");
                this->SendMCodeToPmc(mcode, m_index);
                gettimeofday(&m_time_m_start[m_index], NULL);

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index, true);    //置位选通信号
                gettimeofday(&m_time_m_start[m_index], NULL);

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    gettimeofday(&m_time_m_start[m_index], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index)
                            && this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index)/*再次判断FIN，此时的内存数据可能已经刷新*/){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间
                this->SetMFSig(m_index, false);    //复位选通信号

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index);
                int cur_tool = this->m_channel_status.cur_tool - 1;
                if (cur_tool >= 0 && cur_tool < kMaxToolCount)   //cur_tool=0是主轴
                {
                    std::cout << m_p_chn_tool_info->tool_life_max[cur_tool] << " " << m_p_chn_tool_info->tool_life_type[cur_tool] << std::endl;
                    if (m_p_chn_tool_info->tool_life_type[cur_tool] == ToolPot_Cnt)
                    {//刀具寿命：计次方式
                        m_p_chn_tool_info->tool_life_cur[cur_tool]++;
                        NotifyHmiToolPotChanged();
                        g_ptr_parm_manager->UpdateToolPotConfig(m_n_channel_index, *m_p_chn_tool_info);
                        std::cout << "cur_tool: " << cur_tool << " cur_life: " << m_p_chn_tool_info->tool_life_cur[cur_tool]
                                     << "max_life: " << m_p_chn_tool_info->tool_life_max[cur_tool] << " cur threshold: " << m_p_chn_tool_info->tool_threshold[cur_tool] << std::endl;
                    }
                }
                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            }
        }
            break;
#ifdef USES_GRIND_MACHINE
        case 10: //开启震荡磨
            this->EnableGrindShock(true);
            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            break;
        case 11://关闭震荡磨
            this->EnableGrindShock(false);
            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            break;
        case 60:  //工件夹紧
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M60\n");
                this->m_p_f_reg->work_hold = 1;

                gettimeofday(&m_time_m_start[m_index], nullptr);  //启动计时

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) ==1){
                if(this->m_p_g_reg->work_down_check == 0){
                    struct timeval time_now;
                    int time_elpase = 0;
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > this->m_p_mech_arm_param->wait_overtime*1000){  //超时，告警
                        this->m_error_code = ERR_WORK_CYLINDER_DOWN;
                        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                    }
                    break;
                }
                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
                printf("execute M60 over\n");
            }
            break;
        case 61:  //工件松开
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M61\n");
                this->m_p_f_reg->work_hold = 0;

                gettimeofday(&m_time_m_start[m_index], nullptr);  //启动计时

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) ==1){
                if(this->m_p_g_reg->work_up_check == 0){
                    struct timeval time_now;
                    int time_elpase = 0;
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > this->m_p_mech_arm_param->wait_overtime*1000){  //超时，告警
                        this->m_error_code = ERR_WORK_CYLINDER_UP;
                        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                    }
                    break;
                }
                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
                printf("execute M61 over\n");
            }
            break;
        case 62:  //工位吸真空开
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M62\n");
                this->m_p_f_reg->work_vac = 1;

                gettimeofday(&m_timm_time_m_start[m_index], nullptr);  //启动计时

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) ==1){
                if(this->m_p_g_reg->work_vac_check == 0){
                    struct timeval time_now;
                    int time_elpase = 0;
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > this->m_p_mech_arm_param->wait_overtime*1000){  //超时，告警
                        this->m_error_code = ERR_WORK_VAC_OPEN;
                        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                    }
                    break;
                }
                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
                printf("execute M62 over \n");
            }
            break;
        case 63:  //工位吸真空关
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M63\n");
                this->m_p_f_reg->work_vac = 0;

                gettimeofday(&m_time_m_start[m_index], nullptr);  //启动计时

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) ==1){
                if(this->m_p_g_reg->work_vac_check == 1){
                    struct timeval time_now;
                    int time_elpase = 0;
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > this->m_p_mech_arm_param->wait_overtime*1000){  //超时，告警
                        this->m_error_code = ERR_WORK_VAC_CLOSE;
                        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                    }
                    break;
                }
                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
                printf("execute M63 over \n");
            }
            break;
        case 66:  //主机端请求上料
            if(this->m_p_g_reg->main_chn == GRIND_MAIN_CHN){
                this->ProcessGrindM66(tmp);
            }
            else{
                this->ProcessGrindM66_slave(tmp);
            }
            break;
            //	case 68:  //从机端请求上料
            //		this->ProcessGrindM68(tmp);
            //		break;
#endif
        case 98:	//M98子程序调用
            printf("execute M98\n");
            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            break;
        case 99:   //M99
        	if(tmp->GetExecStep(m_index) == 0){
                printf("execute M99, m_n_subprog_count = %hhu call times=%d\n", m_n_subprog_count, m_p_compiler->getSubCallTimes());
                if(m_n_subprog_count > 0){
                	if(m_p_compiler->needJumpUpper()) m_n_subprog_count--;
                    m_b_ret_from_macroprog = false;
                }
                else{
                	m_p_compiler->RecycleCompile();   //主程序则循环调用
//                    this->m_channel_status.workpiece_count++;  //工件计数加一
//                    g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
//                    this->m_channel_status.workpiece_count_total++;
//                    g_ptr_parm_manager->SetTotalWorkPiece(m_n_channel_index, m_channel_status.workpiece_count_total);
//                    this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);  //通知HMI更新加工计数

//                    if (m_channel_status.workpiece_require != 0 && m_channel_status.workpiece_count >= m_channel_status.workpiece_require)
//                    {//已到达需求件数
//                        CreateError(ERR_REACH_WORK_PIECE, INFO_LEVEL, CLEAR_BY_MCP_RESET);
//                    }

#ifdef USES_GRIND_MACHINE
                    if(this->m_channel_status.workpiece_count >= this->m_p_mech_arm_param->grind_wheel_life){
                        //砂轮到寿
                        this->m_error_code = ERR_GRIND_WHEEL_CHANGE;
                        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                    }
#endif
                }


                //TODO 将代码发送给PMC
                this->m_p_f_reg->DM99 = 1;  //置位DM99, 维持20ms

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                tmp->IncreaseExecStep(m_index);

            }else if(tmp->GetExecStep(m_index) == 2){
                this->m_n_run_thread_state = RUN;

                this->m_p_f_reg->DM99 = 0;
                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态

                printf("execute M99 over\n");
            }

            break;
        case 300:   //lidianqiang:MDA自动加上M30临时改为M300
            //NotifyHmiMCode(0);
            //this->SendMCodeToPmc(0, m_index);
            ResetMcLineNo();//复位MC模块当前行号
            this->SetCurLineNo(1);

            this->m_p_f_reg->STL = 0;
            this->m_p_f_reg->SPL = 0;
            this->m_p_f_reg->OP = 0;

            CompileOver();
            this->SetMiSimMode(false);  //复位MI仿真状态
            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
        break;
        case 998:	//M998 用于暂停编译
            printf("execute M998\n");
            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            break;
        case 999:  //M999 暂停编译，并且同步位置
            printf("execute M999\n");
            //同步位置
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
                //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //同步已编译的轴移动指令的位置

            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            break;
#ifdef USES_TWINING_FUNC
        case 560:  //激活缠绕功能
            ActiveTwiningFunc(mcode);

            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            printf("active twining function\n");
            break;
        case 561:  //关闭缠绕功能
            ActiveTwiningFunc(mcode);

            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            printf("close twining function\n");
            break;
        case 562:
        case 563:
        case 564:
        case 565:
        case 566:
        case 567:
        case 568:
        case 569:
            ActiveTwiningFunc(mcode);

            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            break;
#endif

        default:   //其它M代码
            if(mcode >= 4000 && mcode <= 4099){   // MI临时调试  M代码
                this->MiDebugFunc(mcode);
                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
                break;
            }
            else if(mcode >= 40010 && mcode <= 40083){  // 轴速度控制  力矩控制  M代码
#ifdef USES_SPEED_TORQUE_CTRL
                ProcessCtrlModeSwitch(tmp, m_index);
                break;
#endif
            }
            else if(mcode >= 41011 && mcode <= 41648){  // 轴重新映射  M代码
                ProcessAxisMapSwitch(tmp, m_index);
                break;
            }


            if(tmp->GetExecStep(m_index) == 0){
                //TODO 将代码发送给PMC
                g_ptr_trace->PrintLog(LOG_ALARM, "default:执行的M代码：M%02d", mcode);
                this->SendMCodeToPmc(mcode, m_index);

                gettimeofday(&m_time_m_start[m_index], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){

            	// 主轴不存在或者主轴为虚拟轴，不执行主轴辅助功能
                if((mcode == 3 || mcode == 4 || mcode == 5
                    || mcode == 19 || mcode == 20 || mcode == 26
                    || mcode == 27 || mcode == 28 || mcode == 29)
                        && m_p_spindle->Type() != 2){
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase < 100000){
                        this->SetMFSig(m_index, true);    //置位选通信号
                    }else{
                        this->SetMFSig(m_index, false);    //复位选通信号
                        tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
                    }
                    break;
                }

                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                gettimeofday(&m_time_m_start[m_index], NULL);
                this->SetMFSig(m_index, true);    //置位选通信号

                //llx test
                gettimeofday(&m_time_test, NULL);

                tmp->IncreaseExecStep(m_index);

                // 如果当前在正转状态下再给M03，那么ProcessPMCSignal就扫描不到变化了
                // 这里需要做特殊处理
                if(mcode == 3 && m_p_g_reg->SFR && m_p_spindle->Type() == 2){
                	printf("======================= spindle M03\n");
                	m_p_spindle->InputPolar(Polar::Positive);
                }else if(mcode == 4 && m_p_g_reg->SRV && m_p_spindle->Type() == 2){
                	printf("======================= spindle M04\n");
                	m_p_spindle->InputPolar(Polar::Negative);
                }else if(mcode == 5 && m_p_g_reg->SFR == 0 && m_p_g_reg->SRV == 0
                         && m_p_spindle->Type() == 2){
                	printf("======================= spindle M05\n");
                	m_p_spindle->InputPolar(Polar::Stop);
                }
            }else if(tmp->GetExecStep(m_index) == 2){
            	//等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index)) {
                    //gettimeofday(&m_time_m_start[m_index], NULL);   //开始计时

                    //llx test
                    //gettimeofday(&time_now_test, NULL);
                    //time_elpase_test = (time_now_test.tv_sec-m_time_test.tv_sec)*1000000+time_now_test.tv_usec-m_time_test.tv_usec;
                    //std::cout << "+++++++++++++++++++++++++++++timeval: " << (int)time_elpase_test << std::endl;
                    //std::cout << "mcode: " << (int)mcode << " ME: " << (int)this->GetMExcSig(m_index) << " FIN: " << (int)this->m_p_g_reg->FIN << std::endl;
                    //std::cout << "mfin: " << (int)this->GetMFINSig(m_index) << std::endl;
                }
                else
                {
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index)
                        && this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index)/*再次判断FIN，此时的内存数据可能已经刷新*/){//超过200ms任未进入执行状态，则告警“不支持的M代码”

                        //llx test
                        gettimeofday(&time_now_test, NULL);
                        time_elpase_test = (time_now_test.tv_sec-m_time_test.tv_sec)*1000000+time_now_test.tv_usec-m_time_test.tv_usec;
                        //std::cout << "+++++++++++++++++++++++++++++timeval: " << (int)time_elpase_test << std::endl;
                        //std::cout << "mcode: " << (int)mcode << " ME: " << (int)this->GetMExcSig(m_index) << " FIN: " << (int)this->m_p_g_reg->FIN << std::endl;
                        //std::cout << "mfin: " << (int)this->GetMFINSig(m_index) << std::endl;

                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }
                tmp->IncreaseExecStep(m_index);

            }else if(tmp->GetExecStep(m_index) == 3){
            	if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index, false);    //复位选通信号
                tmp->IncreaseExecStep(m_index);

                // 收到FIN后再清除通知信号，确保梯图已经读取
                if((mcode == 3 || mcode == 4) && m_p_f_reg->SAR == 1){ // 速度到达
                    m_p_f_reg->SAR = 0;
                }else if(mcode == 5 && m_p_f_reg->SST == 1){ // 零速
                    m_p_f_reg->SST = 0;
                }else if(mcode == 19 && m_p_f_reg->ORAR == 1){ // 定位结束
                    //printf("=========================ORAR = 0\n");
                	m_p_f_reg->ORAR = 0;
                }
            }else if(tmp->GetExecStep(m_index) == 4){
            	//等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index);

                tmp->IncreaseExecStep(m_index);
            }else{
            	//this->ExecMCode(tmp, m_index);  //执行某些M代码需要系统执行的动作
                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            }
            break;
        }
        if(tmp->GetExecStep(m_index) != 0xFF)//还未执行结束，返回false
            bRet = false;
        else{
            g_ptr_trace->PrintLog(LOG_ALARM, "执行的M代码结束：M%02d", mcode);
        }
    }

    if(bRet){
        if(this->m_p_f_reg->DEN == 1)
            this->m_p_f_reg->DEN = 0;  //复位DEN信号
    }

    return bRet;
}

/**
 * @brief 执行某些M代码需要系统执行的动作
 * @param msg : M指令
 * @param index :
 */
void ChannelControl::ExecMCode(AuxMsg *msg, uint8_t index){
    //    if(msg == nullptr)
    //        return;
    //    printf("M29 ...................\n");
    //    int mcode = msg->GetMCode(index);

    //    if(mcode == 28){    // M28切换到速度模式
    //        m_p_spindle->SetMode(Speed);
    //        msg->SetExecStep(index, 0xFF);
    //    }else if(mcode == 29){  // M29切换到位置模式
    //        m_p_spindle->SetMode(Position);
    //        msg->SetExecStep(index, 0xFF);
    //    }else{
    //        msg->SetExecStep(index, 0xFF);  //M指令执行结束
    //    }
    //    if(mcode == 28 || mcode == 29){  //主轴CS模式切换

    //        if(this->m_n_spindle_count == 0){  //没有主轴
    //            msg->SetExecStep(index, 0xFF);  //M指令执行结束
    //            return;
    //        }
    //        this->ProcessSpdModeSwitch(msg, index);
    //        this->m_n_M29_flag = mcode-28;

    //    }else{
    //        msg->SetExecStep(index, 0xFF);  //M指令执行结束
    //    }
}

/**
 * @brief 实际执行直线切削指令消息
 * @param msg : 指令消息
 * @param flag_block : 是否需要强制附上快结束标志
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteLineMsg(RecordMsg *msg, bool flag_block){
    LineMsg *linemsg = (LineMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            linemsg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        this->m_mode_restart.pos_target = linemsg->GetTargetPos();  //更新目标位置
        this->m_mode_restart.gmode[1] = linemsg->GetGCode();   //更新模态

        return true;
    }


    if(m_p_spindle->isTapEnable()){
        uint8_t z_axis =  this->GetPhyAxisFromName(AXIS_NAME_Z);

        if(fabs(this->m_channel_rt_status.cur_pos_work.GetAxisValue(z_axis)
                - linemsg->GetTargetPos().GetAxisValue(z_axis)) < 0.005){
            m_error_code = ERR_SPD_TAP_POS_ERROR;
            CreateError(ERR_SPD_TAP_POS_ERROR, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            printf("攻丝状态下， Z轴起点与终点相同 \n");
            printf("source pos:%lf,to pos:%lf\n",linemsg->GetSourcePos().GetAxisValue(2),
                   linemsg->GetTargetPos().GetAxisValue(2));
            return false;
        }
    }

    uint32_t mask = linemsg->GetAxisMoveMask();
    for(int i=0; i<m_p_channel_config->chn_axis_count; i++){
        if((mask & (0x01<<i)) == 0)
            continue;
        if (g_ptr_chn_engine->GetPmcActive(GetPhyAxis(i))) {
            m_error_code = ERR_PMC_IVALID_USED;
            CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            return false;
        }
    }

    //std::cout << "++++++++++++++mask: " << (int)mask << " feed: " << linemsg->GetFeed() << " pmcCount:" << (int)linemsg->GetPmcAxisCount() << std::endl;

    if(msg->IsNeedWaitMsg() && (m_simulate_mode == SIM_NONE || m_simulate_mode == SIM_MACHINING)){//需要等待的命令

        if(linemsg->GetExecStep() == 0){ //只有第一步开始执行时需要等待
            int limit = 2;
            if(this->IsStepMode())
                limit = 4;	//单步模式需要多次验证，因为状态切换有延时

            //等待MC分块的插补到位信号，以及MI的运行到位信号
            int count = 0;
            while(1){
                bool block_over = CheckBlockOverFlag();
                if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                        m_channel_status.machining_state == MS_PAUSED ||
                        m_channel_status.machining_state == MS_WARNING ||
                        this->m_mask_run_pmc != 0){ //未达到执行条件
                    //			printf("compensate exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
                    return false;    //还未运行到位
                }
                else if(++count < limit){
                    usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
                    //		printf("execute compensate msg[%d]: blockflag=%d, count = %d, step = %hhu\n", compmsg->GetGCode(),block_over, count, compmsg->GetExecStep());

                }else
                    break;
            }

            if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
                this->m_b_need_change_to_pause = false;
                m_n_run_thread_state = PAUSE;
                SetMachineState(MS_PAUSED);
                return false;
            }
        }

    }else if(!OutputData(msg, flag_block))
        return false;

    m_n_run_thread_state = RUN;

    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){
        this->m_channel_status.gmode[1] = G01_CMD;    //仿真模式下，立即修改当前模式,不通知HMI
        this->m_pos_simulate_cur_work = linemsg->GetTargetPos();
        this->SetCurLineNo(msg->GetLineNo());
    }

    //	printf("execute line msg out, block_flag=%hhu, feed=%lf\n", msg->CheckFlag(FLAG_BLOCK_OVER), linemsg->GetFeed());

    /*if(m_channel_status.gmode[9] != G80_CMD){
		m_channel_status.gmode[9] = G80_CMD;
		this->SendChnStatusChangeCmdToHmi(G_MODE);
	}*/

    return true;
}

/**
 * @brief 实际执行快速定位指令消息
 * @param msg : 指令消息
 * @param flag_block : 是否需要强制附上快结束标志
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteRapidMsg(RecordMsg *msg, bool flag_block){
    RapidMsg *rapidmsg = (RapidMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            rapidmsg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        this->m_mode_restart.pos_target = rapidmsg->GetTargetPos();  //更新目标位置
        this->m_mode_restart.gmode[1] = rapidmsg->GetGCode();   //更新模态
        return true;
    }

    uint32_t mask = rapidmsg->GetAxisMoveMask();
    for(int i=0; i<m_p_channel_config->chn_axis_count; i++){
        if((mask & (0x01<<i)) == 0)
            continue;
        if (g_ptr_chn_engine->GetPmcActive(GetPhyAxis(i))) {
            m_error_code = ERR_PMC_IVALID_USED;
            CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            return false;
        }
    }

    if(msg->IsNeedWaitMsg() && (m_simulate_mode == SIM_NONE || m_simulate_mode == SIM_MACHINING)){//需要等待的命令
        if(rapidmsg->GetExecStep() == 0){ //只有第一步开始执行时需要等待
            int limit = 2;
            if(this->IsStepMode())
                limit = 4;	//单步模式需要多次验证，因为状态切换有延时

            //等待MC分块的插补到位信号，以及MI的运行到位信号
            int count = 0;
            while(1){
                bool block_over = CheckBlockOverFlag();
                if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                        m_channel_status.machining_state == MS_PAUSED ||
                        m_channel_status.machining_state == MS_WARNING ||
                        this->m_mask_run_pmc != 0){ //未达到执行条件

                    //			printf("compensate exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
                    return false;    //还未运行到位
                }
                else if(++count < limit){
                    usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
                    //		printf("execute compensate msg[%d]: blockflag=%d, count = %d, step = %hhu\n", compmsg->GetGCode(),block_over, count, compmsg->GetExecStep());

                }else
                    break;
            }

            if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
                this->m_b_need_change_to_pause = false;
                m_n_run_thread_state = PAUSE;
                SetMachineState(MS_PAUSED);

                return false;
            }
        }

    }else if(!OutputData(msg, flag_block))
        return false;

    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){
        this->m_channel_status.gmode[1] = G00_CMD;    //仿真模式下，立即修改当前模式,不通知HMI

        this->m_pos_simulate_cur_work = rapidmsg->GetTargetPos();

        this->SetCurLineNo(msg->GetLineNo());
    }

    /*
    if(m_channel_status.gmode[9] != G80_CMD){
		m_channel_status.gmode[9] = G80_CMD;
		this->SendChnStatusChangeCmdToHmi(G_MODE);
	}*/

    //	printf("*****execute rapid msg out, block_flag=%hhu\n", msg->CheckFlag(FLAG_BLOCK_OVER));
    m_n_run_thread_state = RUN;

    return true;
}

/**
 * @brief 实际执行圆弧切削指令消息
 * @param msg : 指令消息
 * @param flag_block : 是否需要强制附上快结束标志
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteArcMsg(RecordMsg *msg, bool flag_block){
    ArcMsg *arc_msg = (ArcMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            arc_msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        this->m_mode_restart.pos_target = arc_msg->GetTargetPos();  //更新目标位置
        this->m_mode_restart.gmode[1] = arc_msg->GetGCode();   //更新模态

        return true;
    }

    //printf("execute arc msg  arc_id: %d\n", arc_msg->arc_id);
    uint32_t mask = arc_msg->GetAxisMoveMask();
    for(int i=0; i<m_p_channel_config->chn_axis_count; i++){
        if((mask & (0x01<<i)) == 0)
            continue;

        if (g_ptr_chn_engine->GetPmcActive(GetPhyAxis(i))) {
            m_error_code = ERR_PMC_IVALID_USED;
            CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            return false;
        }
    }

    if(!OutputData(msg, flag_block))
        return false;

    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){
        this->m_pos_simulate_cur_work = arc_msg->GetTargetPos();
        this->m_channel_status.gmode[1] = arc_msg->GetGCode();    //仿真模式下，立即修改当前模式,不通知HMI
    }

    /*if(m_channel_status.gmode[9] != G80_CMD){
		m_channel_status.gmode[9] = G80_CMD;
		this->SendChnStatusChangeCmdToHmi(G_MODE);
	}*/

    this->m_n_run_thread_state = RUN;

    return true;
}

/**
 * @brief 实际执行坐标系指定指令消息
 * @param msg : 指令消息
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteCoordMsg(RecordMsg *msg){
    //首先将缓冲中的所有待发送指令发送给MC
	CoordMsg *coordmsg = (CoordMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位

		int gcode = coordmsg->GetGCode();
		if ((gcode >= G54_CMD && gcode <= G59_CMD)
				|| (gcode >= G5401_CMD && gcode <= G5499_CMD) || gcode == 541){
			this->m_mode_restart.gmode[14] = gcode;//更新模态
		}
		return true;
	}

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){
        if(m_simulate_mode == SIM_OUTLINE || m_simulate_mode == SIM_TOOLPATH)
            break;

        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            //		printf("coord exec return: 0x%x, count=%hu, block_over=%hhu\n", m_p_mc_comm->ReadRunOverValue(), this->ReadMcMoveDataCount(), block_over);
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            //		printf("execute coord msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

	if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
		this->m_b_need_change_to_pause = false;
		m_n_run_thread_state = PAUSE;
		SetMachineState(MS_PAUSED);
	//	printf("coord msg paused, line_no = %u\n", m_channel_rt_status.line_no);
		return false;
	}


    //设置当前行号
    SetCurLineNo(msg->GetLineNo());


    bool flag = this->m_n_hw_trace_state==REVERSE_TRACE?true:false;    //是否反向引导
    //更新模态
    int gcode = flag?coordmsg->GetLastGCode():coordmsg->GetGCode();

	// @add zk
	if(gcode == G92_CMD){ //设定工件坐标系

		DPointChn point = coordmsg->GetTargetPos();
		for(int i=0; i<kMaxAxisChn; i++){
            if(!(coordmsg->GetAxisMask() & (0x01 << i)))
                continue;
			double offset = GetAxisCurMachPos(i) - point.GetAxisValue(i);
			m_p_chn_g92_offset->offset[i] = offset;
		}
		G52Active = false;
		G92Active = true;
	}

	// @add zk
	if(gcode == G52_CMD){//局部坐标系

        // @add  zk
        DPointChn point = coordmsg->GetTargetPos();
        uint32_t axis_mask = coordmsg->GetAxisMask();

        //当局部坐标偏置都为0时  取消局部坐标系
        switch(coordmsg->GetExecStep()){
        case 0:
            //第一步：更新坐标系的值 激活坐标系
        	flag_cancel_g52 = true;

        	for(int i = 0; i < kMaxAxisChn; i++){
        		//this->SetMcAxisOrigin(i);
				if(axis_mask & (0x01<<i)){

					G52offset[i] = point.GetAxisValue(i);
					// 有值  不需要取消 G52
					if(G52offset[i] > 0.0001) flag_cancel_g52 = false;

					int64_t origin_pos = 0;
					if(G92Active){

						// g92 offset
						origin_pos += (int64_t)(m_p_chn_g92_offset->offset[i] * 1e7);
						// g52 offset
						origin_pos += (int64_t)(G52offset[i]* 1e7);
					}else{

						// ext offset
						origin_pos += m_p_chn_coord_config[0].offset[i] * 1e7;  //基本工件坐标系

						// G54XX offset
						int coord_index = m_channel_status.gmode[14];
						if(coord_index <= G59_CMD ){
							origin_pos += (int64_t)(m_p_chn_coord_config[coord_index/10-53].offset[i] * 1e7);    //单位由mm转换为0.1nm
						}else if(coord_index <= G5499_CMD){
							origin_pos += (int64_t)(m_p_chn_ex_coord_config[coord_index/10-5401].offset[i] * 1e7);    //单位由mm转换为0.1nm
						}
						// g92 offset
						origin_pos += (int64_t)(m_p_chn_g92_offset->offset[i] * 1e7);
						// G52 offset
						origin_pos += (int64_t)(G52offset[i]* 1e7);
					}

					this->SetMcAxisOrigin(i, origin_pos);
        		}
            }

            this->ActiveMcOrigin(true);
            coordmsg->SetExecStep(1); //跳转下一步
            return false;

        case 1:
            //第二步：等待MC设置新工件坐标系完成
            usleep(8000);
            coordmsg->SetExecStep(2);  //跳转下一步
            return false;

        case 2:
            //第三步：同步位置
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){

                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //同步已编译的轴移动指令的位置
            break;

        default:
            printf("execute coord msg[%hu] error, step = %hhu\n", gcode, coordmsg->GetExecStep());
            break;
        }

        // @add  zk
        if(flag_cancel_g52){
        	G52Active = false;
        }
        else{
        	G52Active = true;
        }

    }else if(gcode == G53_CMD){ //机械坐标系

        if(!OutputData(msg, true))
            return false;

        if(this->m_simulate_mode != SIM_NONE)  //保存仿真当前位置
            this->m_pos_simulate_cur_work = coordmsg->GetTargetPos();

    }else if(m_simulate_mode == SIM_NONE || m_simulate_mode == SIM_MACHINING){  //非仿真模式或者加工仿真模式

    	if(G52Active) return true;

    	uint16_t coord_mc = 0;
        switch(coordmsg->GetExecStep()){
        case 0:
            //第一步：更新模态，将新工件坐标系发送到MC
            this->m_channel_status.gmode[14] = gcode;

            //更新MC的坐标系数据
            this->SetMcCoord(true);

            coordmsg->SetExecStep(1); //跳转下一步
            return false;
        case 1:
            //第二步：等待MC设置新工件坐标系完成
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadChnCurCoord(m_n_channel_index, coord_mc);  //读取MC当前工件坐标系
            else
                this->m_p_mc_arm_comm->ReadChnCurCoord(m_n_channel_index, coord_mc);
            if(coord_mc*10 == gcode){
                coordmsg->SetExecStep(2);  //跳转下一步
            }
            return false;
        case 2:
            //第三步：同步位置
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
                //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //同步已编译的轴移动指令的位置
            break;
        default:
            printf("execute coord msg[%hu] error, step = %hhu\n", gcode, coordmsg->GetExecStep());
            break;
        }

    }else{//轮廓仿真，刀路仿真
        //先把当前位置的工件坐标转换为机械坐标
        this->TransWorkCoordToMachCoord(this->m_pos_simulate_cur_work, m_channel_status.gmode[14], m_mask_intp_axis);

        this->m_channel_status.gmode[14] = gcode;  //修改模态

        //转换当前位置的工件坐标
        this->TransMachCoordToWorkCoord(m_pos_simulate_cur_work, m_channel_status.gmode[14], m_mask_intp_axis);

        if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
            this->m_p_compiler->SetCurPos(m_pos_simulate_cur_work);   //同步编译器位置
        }else
            this->RefreshOuputMovePos(m_pos_simulate_cur_work);    //同步已编译的轴移动指令的位置
    }

    printf("execute coord message : %d\n", m_channel_status.gmode[14]);

    this->SendChnStatusChangeCmdToHmi(G_MODE);

    return true;
}

/**
 * @brief 实际执行刀具指令消息
 * @param msg : 指令消息
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteToolMsg(RecordMsg *msg){
    //单段模式下首先将缓冲中的所有待发送指令发送给MC
    //	if(this->m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE)){
    //		if(!this->OutputAllData()){
    //			//PL中的FIFO已满，发送失败
    //			return false;
    //		}
    //	}
    ToolMsg *toolmsg = (ToolMsg *)msg;
    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        this->m_mode_restart.cur_tcode = toolmsg->GetTool(0);
#ifdef USES_T_CODE_MACRO
        this->m_mode_restart.sub_prog_call++;
#endif
        return true;
    }

    if(this->m_simulate_mode != SIM_NONE){  //仿真模式
        //设置当前行号
        SetCurLineNo(toolmsg->GetLineNo());
        m_n_run_thread_state = RUN;
        return true;
    }


#ifdef USES_T_CODE_MACRO
    //单段模式下，等待MC运行完所有数据
    if(this->IsStepMode()){
        int count = 0;
        while(count < 4 ){
            if(this->ReadMcMoveDataCount() > 0 ||
                    (!CheckStepOverFlag() && !CheckBlockOverFlag()) ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){
                return false;    //还未运行到位
            }
            else{
                count++;
                printf("tool: step=%d, %d, c = %d\n", m_channel_mc_status.step_over,m_channel_mc_status.auto_block_over, count);
                usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            }
        }

        if(this->m_b_need_change_to_pause){//单段，切换暂停状态
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }
    }else{//非单段模式
        if(m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING)
            return false;
    }

    this->m_n_cur_tcode = toolmsg->GetTool(0);
    this->m_channel_status.preselect_tool_no = m_n_cur_tcode;



    //	bool flag = (m_n_hw_trace_state==REVERSE_TRACE)?true:false;    //是否反向引导
    //
    //	if(flag){ //反向引导
    //		if(this->m_p_general_config->debug_mode > 0){  //调式模式下，打开宏程序文件
    //			if(toolmsg->GetSubProgType() > 1){
    //				char file[kMaxFileNameLen];
    //				memset(file, 0x00, kMaxFileNameLen);
    //				toolmsg->GetLastProgFile(file);
    //				this->SendOpenFileCmdToHmi(file);
    //			}
    //		}
    //
    //		//设置当前行号
    //		SetCurLineNo(msg->GetLineNo());
    //
    //		m_n_subprog_count--;
    //		m_n_macroprog_count--;

    //		if(this->IsStepMode()){
    //			this->SetMcStepMode(true);
    //		}
    //
    //	}else{
    if(m_n_subprog_count >= kMaxSubNestedCount){
        this->m_error_code = ERR_SUB_NESTED_COUNT;   //子程序嵌套层数过多
        CreateError(ERR_SUB_NESTED_COUNT, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        return false;
    }

    //TODO 向HMI发送命令打开子文件
    if(this->m_p_general_config->debug_mode > 0){  //调式模式下，打开宏程序文件
        if(toolmsg->GetSubProgType() > 1){
            char file[kMaxFileNameLen];
            memset(file, 0x00, kMaxFileNameLen);
            toolmsg->GetSubProgName(file, false);
            this->SendOpenFileCmdToHmi(file);
        }
    }

    //设置当前行号
    SetCurLineNo(msg->GetLineNo());

    m_n_subprog_count++;
    m_n_macroprog_count++;

    if(this->m_p_general_config->debug_mode == 0){
        this->SetMcStepMode(false);
        this->m_b_need_change_to_pause = false;
    }

    //	}
#else
    //		printf("execute tool msg: line = %llu, tool = %hu\n", toolmsg->GetLineNo(), toolmsg->GetTool(0));
    if(toolmsg->IsFirstExec()){
        //首先将缓冲中的所有待发送指令发送给MC
        //		if(!OutputLastBlockItem()){
        //			//PL中的FIFO已满，发送失败
        //			return false;
        //		}

        int limit = 2;
        if(this->IsStepMode())
            limit = 4;	//单步模式需要多次验证，因为状态切换有延时

        //等待MC分块的插补到位信号，以及MI的运行到位信号
        int count = 0;
        bool block_over = false;
        while(1){
            block_over = CheckBlockOverFlag();
            if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
                //		printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
                return false;    //还未运行到位
            }
            else if(++count < limit){
                usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
                //		printf("execute aus msg: blockflag=%d, count = %d\n",  block_over, count);

            }else
                break;
        }

        if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }

        SetCurLineNo(toolmsg->GetLineNo());
    }


    bool bRet = true;
    struct timeval time_now;
    unsigned int time_elpase = 0;
    uint8_t index = 0;
    uint8_t count = toolmsg->GetCount();
    uint16_t tcode = 0;
    //	uint64_t mask = 0;
    for(index = 0; index < count; index++){
        if(toolmsg->GetExecStep(index) == 0xFF)//已经执行结束的就跳过
            continue;

        tcode = toolmsg->GetTool(index);
        printf("tool index = %hhu, tcode=%hu, step = %hhu\n", index, tcode, toolmsg->GetExecStep(index));

        switch(toolmsg->GetExecStep(index)){
        case 0:{
            this->m_n_cur_tcode = tcode;
            this->m_channel_status.preselect_tool_no = m_n_cur_tcode;

            //TODO 将代码发送给PMC
            this->SendTCodeToPmc(tcode, index);

            gettimeofday(&m_time_t_start[index], NULL);   //

            toolmsg->IncreaseExecStep(index);
            break;
        }
        case 1:{
        	//等待TMF延时，置位TF选通信号
            gettimeofday(&time_now, NULL);
            time_elpase = (time_now.tv_sec-m_time_t_start[index].tv_sec)*1000000+time_now.tv_usec-m_time_t_start[index].tv_usec;
            if(time_elpase < 16000)
                break;		//未到延时时间

            this->SetTFSig(index, true);    //置位选通信号

            toolmsg->IncreaseExecStep(index);
            break;
        }
        case 2:{
            //等待对应的TFIN信号
            if(this->GetTFINSig(index))
                gettimeofday(&m_time_t_start[index], NULL);   //开始计时
            else
                break;

            toolmsg->IncreaseExecStep(index);
            break;
        }
        case 3:{
        	if(!this->GetTFINSig(index)){  //干扰信号，重新确认
                toolmsg->SetExecStep(index, 2);
                break;
            }

            //等待TFIN延时，复位TF信号
            gettimeofday(&time_now, NULL);
            time_elpase = (time_now.tv_sec-m_time_t_start[index].tv_sec)*1000000+time_now.tv_usec-m_time_t_start[index].tv_usec;
            if(time_elpase < 16000)
                break;		//未到延时时间

            this->SetTFSig(index, false);    //复位选通信号

            toolmsg->IncreaseExecStep(index);
            break;
        }
        case 4:{
            //等待FIN信号复位
            if(this->GetTFINSig(index))
                break;

            //复位辅助指令信号
            this->SendTCodeToPmc(0, index);

            toolmsg->SetExecStep(index, 0xFF);   //0xFF表示执行结束

            printf("execute T%hu over\n", tcode);
            break;
        }
        default:
            break;
        }

        if(toolmsg->GetExecStep(index) != 0xFF)//还未执行结束，返回false
            bRet = false;

    }

#endif

    printf("execute tool message: cur_t_code=%d, return %hhu\n", m_n_cur_tcode, bRet);

    return bRet;
}

/**
 * @brief 实际执行一般模态指令消息，不带参数的模态指令
 * @param msg : 指令消息
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteModeMsg(RecordMsg *msg){
    //单段模式下首先将缓冲中的所有待发送指令发送给MC
    //	if(this->m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE)){
    //		if(!this->OutputAllData()){
    //			//PL中的FIFO已满，发送失败
    //			return false;
    //		}
    //	}
    ModeMsg *modemsg = (ModeMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        int cmd = modemsg->GetGCode();
        //	this->m_mode_restart.pos_target = loopmsg->GetTargetPos();
        this->m_mode_restart.gmode[GetModeGroup(cmd)] = cmd;
        return true;
    }

    if(m_simulate_mode == SIM_OUTLINE && m_simulate_mode == SIM_TOOLPATH){  //轮廓仿真，刀路仿真
        //设置当前行号
        SetCurLineNo(msg->GetLineNo());
        return true;
    }

    int count = 0;
    while(count < 4 ){
        if(this->ReadMcMoveDataCount() > 0 || !this->CheckBlockOverFlag() ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){
            return false;    //还未运行到位
        }
        else{
            count++;
            //			printf("ExecuteModeMsg: step=%d, %d, c = %d\n", m_channel_mc_status.step_over,m_channel_mc_status.auto_block_over, count);
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
        }
    }

    //单段模式
    if(this->IsStepMode()){
        if(this->m_b_need_change_to_pause){//单段，切换暂停状态
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            printf("mode msg[%hu] paused, line_no = %u\n", modemsg->GetGCode(), m_channel_rt_status.line_no);
            return false;
        }

        //设置当前行号
        SetCurLineNo(msg->GetLineNo());

    }else{//非单段模式
        if(m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING)
            return false;
    }



    bool flag = this->m_n_hw_trace_state==REVERSE_TRACE?true:false;    //是否反向引导
    int cmd = flag?modemsg->GetLastGCode():modemsg->GetGCode();
    //	printf("execute mode message1 : cmd = %d\n", cmd);


    //	if(cmd == G84_3_CMD){
    //		if(this->m_n_M29_flag == 1 && this->m_n_G843_flag == 0){
    //			this->SendMcRigidTapFlag(true);
    //			this->SendMiTapStateCmd(true);
    //			this->m_n_G843_flag = 1;
    ////			printf("send G843 message : %d\n", cmd);
    //		}
    //	}

    //更新模态
    int mode_index = GCode2Mode[cmd/10];
    if(McModeFlag[mode_index]){//需要发送给MC的模态指令
        switch(mode_index){
        case 2:
            this->m_mc_mode_exec.bits.mode_g17 = cmd/10 - 17;
            break;
        case 3:
            this->m_mc_mode_exec.bits.mode_g90 = cmd/10 - 90;
            break;
        case 5:
            this->m_mc_mode_exec.bits.mode_g94 = cmd/10 - 94;
            break;
        case 6:
            this->m_mc_mode_exec.bits.mode_g20 = cmd/10 -20;
            break;
        case 10:
            this->m_mc_mode_exec.bits.mode_g98 = cmd/10 - 98;
            break;
        case 11:
            this->m_mc_mode_exec.bits.mode_g50 = cmd/10 - 50;
            break;
        case 13:
            this->m_mc_mode_exec.bits.mode_g60 = cmd/10 - 60;
            break;
        case 16:
            this->m_mc_mode_exec.bits.mode_g68 = cmd/10 - 68;
            break;
        default:
            break;
        }
    }

    if(this->IsStepMode()){
        this->RefreshModeInfo(m_mc_mode_exec);	//单段模式需要立即更新模态
    }


    this->m_channel_status.gmode[GCode2Mode[cmd/10]] = cmd;


    //TODO 执行各模态指令
    printf("execute mode message : %d\n", cmd);
    if(cmd == G98_CMD){
        Variable *pv = GetMacroVar();
        pv->SetVarValue(198, 0.0);
    }

    if(cmd == G99_CMD){
        Variable *pv = GetMacroVar();
        pv->SetVarValue(198, 1.0);
    }

    if(cmd == G61_CMD){
        printf("G61\n");
    }else if(cmd == G62_CMD){
        printf("G62\n");
    }else if(cmd == G63_CMD){
        printf("G63\n");
    }else if(cmd == G64_CMD){
        printf("G64\n");
    }

    //通知HMI
    this->SendChnStatusChangeCmdToHmi(G_MODE);

    return true;
}

/**
 * @brief 实际执行进给速度指定指令消息
 * @param msg : 指令消息
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteFeedMsg(RecordMsg *msg){

    //	if(m_channel_status.machining_state == MS_PAUSED ||
    //			m_channel_status.machining_state == MS_WARNING)
    //		return false;
    //
    //	//单段模式下，等待MC运行完所有数据
    //	if(this->m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE)){
    //		int count = 0;
    //		while(count < 2 ){
    //			if(this->ReadMcMoveDataCount() > 0 || (!this->CheckStepOverFlag() && !this->CheckBlockOverFlag())){
    //				return false;    //还未运行到位
    //			}
    //			else{
    //				count++;
    //	//			printf("execute feed msg: stepflag=%d,  count = %d\n", m_channel_mc_status.step_over, count);
    //				usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
    //			}
    //		}
    //	}

    //更新状态
    //	FeedMsg *feedmsg = (FeedMsg *)msg;

    //	this->m_channel_status.rated_feed = feedmsg->GetFeed()*1000/60;   //单位转换:mm/min --> um/s
    return true;
}

/**
 * @brief 实际执行主轴转速指令消息
 * @param msg : 指令消息
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteSpeedMsg(RecordMsg *msg){
    //单段模式下首先将缓冲中的所有待发送指令发送给MC
    //	if(this->m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE)){
    //		if(!this->OutputAllData()){
    //			//PL中的FIFO已满，发送失败
    //			return false;
    //		}
    //	}

    SpeedMsg *speed = (SpeedMsg *)msg;

    //TODO 当前先用第一个主轴来判断
    uint8_t phy_spd = this->m_spd_axis_phy[0];
    SCAxisConfig *spd_config = nullptr;
    if(phy_spd > 0)
        spd_config = &m_p_axis_config[phy_spd-1];

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        this->m_mode_restart.cur_scode = speed->GetSpeed();

        if(phy_spd > 0){
            if(m_mode_restart.cur_scode > (int)spd_config->spd_max_speed)
                m_mode_restart.cur_scode = spd_config->spd_max_speed;
            if(m_mode_restart.cur_scode < (int)spd_config->spd_min_speed)
                m_mode_restart.cur_scode = spd_config->spd_min_speed;
        }

        if(m_mode_restart.spindle_dir != SPD_DIR_STOP){  //当前主轴已经处于旋转状态
            m_mode_restart.rated_spindle_speed = m_mode_restart.cur_scode;
        }

        return true;
    }

    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){  //轮廓仿真，刀路仿真

        //设置当前行号
        SetCurLineNo(speed->GetLineNo());

        m_n_run_thread_state = RUN;

        return true;
    }

    //单段模式下，等待MC运行完所有数据
    //	if(this->IsStepMode()){

#ifdef USES_WOOD_MACHINE
    double td = 0;
    int forward_line = 0;    //主轴预启动提前行数
    if(this->GetMacroVar(810, td) == 0)
        forward_line = td;

    //		printf("!!!!SPEED MSG,line=%d, curline=%u, msgline=%llu\n", forward_line, m_channel_rt_status.line_no, msg->GetLineNo());

    if(!this->IsStepMode() && ((m_channel_rt_status.line_no+forward_line) >= msg->GetLineNo())){  //非单段模式下的主轴启动指令预启动
        printf("S cmd forward execute at line:%u\n", m_channel_rt_status.line_no);
    }else{

#endif
        int count = 0;
        while(count < 4 ){
            if(this->ReadMcMoveDataCount() > 0 ||
                    (!this->CheckStepOverFlag() && !this->CheckBlockOverFlag()) ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){
                return false;    //还未运行到位
            }
            else{
                count++;
                //				printf("ExecuteSpeedMsg: step=%d, %d, c = %d\n", m_channel_mc_status.step_over,m_channel_mc_status.auto_block_over, count);
                usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            }
        }

        if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }
#ifdef USES_WOOD_MACHINE
    }
#endif
    //	}else{//非单段模式
    //		if(m_channel_status.machining_state == MS_PAUSED ||
    //				m_channel_status.machining_state == MS_WARNING)
    //			return false;
    //	}

    //设置当前行号
    SetCurLineNo(msg->GetLineNo());

    //S代码输入到主轴模块
    m_p_spindle->InputSCode(speed->GetSpeed());

    //ScPrintf("ExecuteSpeedMsg::%d rpm\n", m_p_spindle->GetSCode());

    //更新当前S值
    m_channel_status.rated_spindle_speed = m_p_spindle->GetSCode();
    this->SendModeChangToHmi(S_MODE);

    return true;
}

/**
 * @brief 实际执行循环指令消息
 * @param msg : 指令消息
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteLoopMsg(RecordMsg *msg){
    //单段模式下首先将缓冲中的所有待发送指令发送给MC
    //	if(this->m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE)){
    //		if(!this->OutputAllData()){
    //			//PL中的FIFO已满，发送失败
    //			return false;
    //		}
    //	}
    LoopMsg *loopmsg = (LoopMsg *)msg;
    int cmd = loopmsg->GetGCode();

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        //	this->m_mode_restart.pos_target = loopmsg->GetTargetPos();
        this->m_mode_restart.gmode[GetModeGroup(cmd)] = cmd;
        this->m_mode_restart.sub_prog_call++;
        return true;
    }

    if(m_simulate_mode == SIM_OUTLINE && m_simulate_mode == SIM_TOOLPATH){  //轮廓仿真，刀路仿真
        //设置当前行号
        SetCurLineNo(msg->GetLineNo());
        m_n_subprog_count++;
        m_n_macroprog_count++;
        return true;
    }
    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int limit = 1;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    int count = 0;
    while(1){

        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            //	printf("loop exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            printf("execute loop msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    if(m_n_subprog_count >= kMaxSubNestedCount){
        this->m_error_code = ERR_SUB_NESTED_COUNT;   //子程序嵌套层数过多
        CreateError(ERR_SUB_NESTED_COUNT, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        return false;
    }

    //TODO 向HMI发送命令打开子文件
    if(this->m_p_general_config->debug_mode > 0){  //调式模式下，打开宏程序文件
        if(this->m_channel_status.chn_work_mode == AUTO_MODE){
            if(loopmsg->GetMacroProgType() > 1){
                char file[kMaxFileNameLen];
                memset(file, 0x00, kMaxFileNameLen);
                loopmsg->GetMacroProgName(file, false);
                this->SendOpenFileCmdToHmi(file);
            }
        }
    }

    //设置当前行号
    SetCurLineNo(msg->GetLineNo());

    m_n_subprog_count++;
    m_n_macroprog_count++;

    if(this->m_p_general_config->debug_mode == 0){
        this->SetMcStepMode(false);
    }

    //更新模态
    if(cmd == G89_CMD)
        m_mc_mode_exec.bits.mode_g73 = 15;    //mode_g74只有4bit，值范围0~15，因此G89需要特殊处理
    else
        m_mc_mode_exec.bits.mode_g73 = cmd/10 - 73;

    //	printf("execute loop msg: gmode[9] = %hu, gcode = %d\n", m_channel_status.gmode[9], loopmsg->GetGCode());

    if(m_channel_status.gmode[9] == G80_CMD &&
            (loopmsg->GetGCode() == G84_CMD || loopmsg->GetGCode() == G74_CMD)){ //切换刚性攻丝模态，则发送数据给MI
        uint8_t pc = 0;
        uint32_t pm = 0;
        uint32_t mask = 0x01;
        double feed = 0;
        int i  = 0;
        double *pp = loopmsg->GetParameter(pm, pc);

        while(pm != 0){  //获取F参数
            if(pm & mask){
                if(i == F_DATA){
                    feed = *pp;
                    break;
                }
                pp++;
            }
            pm = pm>>1;
            i++;
        }

        if(loopmsg->GetGCode() == G74_CMD){
            m_p_spindle->SetTapFeed(-feed);
        }else{
            m_p_spindle->SetTapFeed(feed);
        }


    }
    else if((m_channel_status.gmode[9] == G84_CMD || m_channel_status.gmode[9] == G74_CMD)
            && loopmsg->GetGCode() == G80_CMD){ // 退出攻丝
        m_p_spindle->ResetTapFlag();
    }

    this->m_channel_status.gmode[9] = loopmsg->GetGCode();

    // 通知HMI模态变化
    this->SendChnStatusChangeCmdToHmi(G_MODE);

    return true;
}

/**
 * @brief 实际执行刀补指令消息
 * @param msg : 指令消息
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteCompensateMsg(RecordMsg *msg){
    CompensateMsg *compmsg = (CompensateMsg *)msg;
    int type = compmsg->GetGCode();

    if(this->m_n_restart_mode != NOT_RESTART &&
            compmsg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        this->m_mode_restart.pos_target = compmsg->GetTargetPos();
        this->m_mode_restart.gmode[GetModeGroup(type)] = type;
        if(type == G43_CMD || type == G44_CMD || type == G49_CMD || type == G43_4_CMD){
            this->m_mode_restart.cur_h_code = compmsg->GetCompValue();
        }
        else if(type == G40_CMD || type == G41_CMD || type == G42_CMD){
            this->m_mode_restart.cur_d_code = compmsg->GetCompValue();
        }
        return true;
    }

    if(this->m_simulate_mode != SIM_NONE)
        this->m_pos_simulate_cur_work = compmsg->GetTargetPos();  //保存仿真模式当前位置

    int count = 0;

    if(msg->IsNeedWaitMsg() && (m_simulate_mode == SIM_NONE || m_simulate_mode == SIM_MACHINING)){//需要等待的命令
        int limit = 2;
        if(this->IsStepMode())
            limit = 4;	//单步模式需要多次验证，因为状态切换有延时

        //等待MC分块的插补到位信号，以及MI的运行到位信号
        while(1){
            bool block_over = CheckBlockOverFlag();
            if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
                //			printf("compensate exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
                return false;    //还未运行到位
            }
            else if(++count < limit){
                usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
                //		printf("execute compensate msg[%d]: blockflag=%d, count = %d, step = %hhu\n", compmsg->GetGCode(),block_over, count, compmsg->GetExecStep());

            }else
                break;
        }

        if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }

        //设置当前行号
        SetCurLineNo(msg->GetLineNo());

    }
    else{//无需等待的命令

        //单段模式下，等待MC运行完所有数据
        if(this->IsStepMode()){
            int count = 0;
            while(count < 4 ){
                if(this->ReadMcMoveDataCount() > 0 ||
                        (!this->CheckStepOverFlag() && !this->CheckBlockOverFlag()) ||
                        m_channel_status.machining_state == MS_PAUSED ||
                        m_channel_status.machining_state == MS_WARNING){
                    return false;    //还未运行到位
                }
                else{
                    count++;
                    printf("execute compensate msg[%d]: step=%d, %d, c = %d\n", compmsg->GetGCode(), m_channel_mc_status.step_over,m_channel_mc_status.auto_block_over, count);
                    usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
                }
            }
            //设置当前行号
            SetCurLineNo(msg->GetLineNo());

        }else{//非单段模式
            if(m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING)
                return false;
        }
    }

    // @test zk
	if(type == G41_CMD || type == G42_CMD || type == G40_CMD){//半径补偿，先更新到MC模态信息
		if(m_simulate_mode == SIM_NONE || m_simulate_mode == SIM_MACHINING){  //非仿真模式或者加工仿真
		   this->m_mc_mode_exec.bits.mode_g40 = (type-G40_CMD)/10;
		   //this->m_mc_mode_exec.bits.mode_d = value;

		   if(this->IsStepMode()){
			   this->RefreshModeInfo(m_mc_mode_exec);	//单段模式需要立即更新模态
		   }
		}

		this->m_channel_status.cur_d_code = compmsg->GetCompValue();;
		this->SendModeChangToHmi(D_MODE);
		m_n_run_thread_state = RUN;
		return true;
	}
	// @test zk

    bool flag = this->m_n_hw_trace_state==REVERSE_TRACE?true:false;    //是否反向引导
        int value = flag?compmsg->GetLastCompValue():compmsg->GetCompValue();
    //切换补偿值，更新模态

    uint16_t offset_mc = 0;  //mc中的当前刀偏
    int32_t z_axis_offset = 0;
    uint16_t intp_mode = 0;

    if(type == G43_CMD || type == G44_CMD || type == G49_CMD){//G43/G44    即时更新到通道状态的模态信息中

    	if(this->m_simulate_mode != SIM_NONE){//仿真模式，跳过刀长补偿
            if(!OutputData(msg, true))
                return false;
            m_n_run_thread_state = RUN;
            return true;
        }


        //	printf("execute G49 msg:%hhu\n", compmsg->GetExecStep());
        switch(compmsg->GetExecStep()){
        case 0://第一步：将新偏置发送到MC
#ifdef USES_FIVE_AXIS_FUNC
            m_b_5axis_clear_rot = false;
#endif
            if(type == G49_CMD){
                if(this->m_channel_status.gmode[8] == G43_4_CMD){ //取消RTCP
                    this->SetMcRtcpMode(G43_4_MODE, CANCEL_MODE, 0);
#ifdef USES_FIVE_AXIS_FUNC
                    m_b_5axis_clear_rot = true;  //旋转轴需要清整数圈
#endif
                }else{
                    this->m_channel_status.gmode[8] = type;
                    this->m_channel_status.cur_h_code = value;
                    this->SetMcToolOffset(false);
                }

            }
            else{
                this->m_channel_status.gmode[8] = type;
                this->m_channel_status.cur_h_code = value;

                this->SetMcToolOffset(true);
            }

            compmsg->SetExecStep(1);	//跳转下一步
            printf("execute step 1\n");
            return false;
        case 1:
            //第二步：等待MC执行新刀偏完成
            if(type == G49_CMD && this->m_channel_status.gmode[8] == G43_4_CMD){//取消RTCP
                if(!this->m_b_mc_on_arm)
                    this->m_p_mc_comm->ReadChnCurIntpMode(this->m_n_channel_index, intp_mode);  //读取MC当前特殊插补模式
                else
                    this->m_p_mc_arm_comm->ReadChnCurIntpMode(this->m_n_channel_index, intp_mode);
                if(intp_mode == CANCEL_MODE){
                    this->m_channel_status.gmode[8] = type;
                    this->m_channel_status.cur_h_code = value;
                    compmsg->SetExecStep(2);  //跳转下一步
                }
                else
                    printf("G49, intp_mode = %hu\n", intp_mode);
            }else{
                if(!this->m_b_mc_on_arm)
                    this->m_p_mc_comm->ReadChnCurToolOffset(this->m_n_channel_index, offset_mc);  //读取MC当前刀偏
                else
                    this->m_p_mc_arm_comm->ReadChnCurToolOffset(m_n_channel_index, offset_mc);
                if(offset_mc == value){
                    compmsg->SetExecStep(2);  //跳转下一步
                }
                printf("offset_mc = %hu, value = %hu\n", offset_mc, value);
            }
            printf("execute step 2\n");
            return false;
        case 2:
            //第三步：同步位置
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            RefreshAxisIntpPos();

            //			printf("compensate: refreshpos[%lf, %lf, %lf]\n", m_channel_rt_status.cur_pos_work.m_df_point[0], m_channel_rt_status.cur_pos_work.m_df_point[1],
            //								m_channel_rt_status.cur_pos_work.m_df_point[2]);


            //刷新运动指令目标位置
            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);


            this->SendChnStatusChangeCmdToHmi(G_MODE);
            this->SendModeChangToHmi(H_MODE);
#ifdef USES_FIVE_AXIS_FUNC
            if(m_b_5axis_clear_rot){
                compmsg->SetExecStep(3);  //跳转下一步
                printf("execute step 3\n");
            }else
#endif
                compmsg->SetExecStep(6);  //跳过清整数圈步骤
            printf("execute step 6\n");
            return false;
#ifdef USES_FIVE_AXIS_FUNC
        case 3:  //第四步: 如果是G49取消五轴变换，则执行G200，旋转轴清整数圈

            m_mask_5axis_rot_nolimit_phyaxis = 0;
            //五轴机床需要对无线旋转轴清整数圈
            if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS){
                //向MI发送清整数圈位置指令
                m_n_mask_clear_pos = 0;

                for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
                    if(((m_mask_5axis_rot_nolimit>>i) & 0x01) == 0){

                        continue;
                    }
                    uint8_t PhyAxisIndex = GetPhyAxis(i);
                    m_mask_5axis_rot_nolimit_phyaxis |= (0x01<<PhyAxisIndex);
                    this->SendMiClearPosCmd(PhyAxisIndex+1, 360*1000);
                }
                compmsg->SetExecStep(4);	//跳转下一步
            }else
                compmsg->SetExecStep(6);	//跳过清整数圈步骤
            printf("execute step 4\n");
            return false;
        case 4://第五步
            //等待MI设置完成
            if((m_mask_5axis_rot_nolimit_phyaxis & this->m_n_mask_clear_pos) == m_mask_5axis_rot_nolimit_phyaxis) {    //if(m_mask_5axis_rot_nolimit == this->m_n_mask_clear_pos)
                compmsg->SetExecStep(5);	//跳转下一步
            }
            printf("execute step 5\n");
            return false;
        case 5:
            //第六步：同步位置

            //从MC同步工件坐标
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();

            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);

            //		printf("intp pos (%lf, %lf, %lf, %lf)\n", m_channel_mc_status.intp_pos.x, m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z,
            //				m_channel_mc_status.intp_pos.a6);

            //从MI同步机械坐标
            this->m_p_channel_engine->SendMonitorData(false, false);

            //		printf("cur mach pos (%lf, %lf, %lf, %lf)\n", m_channel_rt_status.cur_pos_machine.x, m_channel_rt_status.cur_pos_machine.y,
            //				m_channel_rt_status.cur_pos_machine.z, m_channel_rt_status.cur_pos_machine.a6);

            compmsg->SetExecStep(6);  //跳转下一步
            printf("execute step 6\n");
            return false;
#endif
        case 6:
            //第七步：发送轴运动指令，并启动MC
            if(!compmsg->CheckFlag(FLAG_AXIS_MOVE)){//没有轴移动，结束
                //	this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
                break;
            }

            // @zk  发送这条命令会 卡死 MC tips: 这条指令没指定cmd 指定为00 后可运行
            if(!OutputData(msg, true))
                return false;

            this->StartMcIntepolate();  //启动MC
            //			compmsg->SetExecStep(7);  //跳转下一步
            printf("execute g49 mode over\n");
            break;
            //		case 7:
            //			//第五步：等待MC运行到位
            //			//在函数头处等待
            //
            //			printf("execute step 4\n");
            //			break;
        default:
            printf("G43/G44/G49 execute error!\n");
            break;
        }

    }else if(type == G43_4_CMD){  //激活RTCP
        switch(compmsg->GetExecStep()){  //新流程：先激活RTCP再走运动指令
        case 0://第一步：将新偏置发送到MC

            z_axis_offset = m_p_chn_tool_config->geometry_compensation[value-1][2] * 1e3;  //单位由mm转换为um
            //	z_axis_offset += m_p_chn_tool_config->geometry_comp_basic[2] * 1e3;   //基准刀偏
            z_axis_offset += m_p_chn_tool_config->geometry_wear[value-1] * 1e3;   //叠加磨损补偿

            //	printf("G43.4 send :idx = %d, offset=%d\n", value, z_axis_offset);

            this->SetMcRtcpMode(G43_4_MODE, G43_4_MODE, z_axis_offset);

            compmsg->SetExecStep(1);	//跳转下一步
            return false;
        case 1:
            //第二步：等待MC执行新刀偏完成
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadChnCurIntpMode(this->m_n_channel_index, intp_mode);  //读取MC当前特殊插补模式
            else
                this->m_p_mc_arm_comm->ReadChnCurIntpMode(m_n_channel_index, intp_mode);
            if(intp_mode == G43_4_MODE){
                compmsg->SetExecStep(2);  //跳转下一步
            }
            return false;
        case 2:
            //第三步：同步位置
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();


            //			compmsg->RefreshTargetPos(m_channel_mc_status.intp_pos);    //同步当前行的目标位置
            //			this->m_p_compiler->SetCurPos(compmsg->GetTargetPos());   //同步编译器位置
            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);  //同步位置

            this->m_channel_status.gmode[8] = type;
            this->m_channel_status.cur_h_code = value;

            this->SendChnStatusChangeCmdToHmi(G_MODE);
            this->SendModeChangToHmi(H_MODE);


            compmsg->SetExecStep(3);	//跳转下一步
            return false;
        case 3:
            //第四步：发送轴运动指令，并启动MC
            if(!compmsg->CheckFlag(FLAG_AXIS_MOVE)){//没有轴移动,结束
                break;
            }

            if(!OutputData(msg, true))
                return false;

            this->StartMcIntepolate();  //启动MC

            printf("execute G43.4 over\n");
            break;

        default:
            printf("G43/G44/G49 execute error!\n");
            break;
        }


        //老的处理流程：先运行移动指令再激活RTCP
        //		switch(compmsg->GetExecStep()){
        //		case 0:
        //			//第一步：发送轴运动指令，并启动MC
        //			if(!compmsg->CheckFlag(FLAG_AXIS_MOVE)){//没有轴移动，跳转第三步
        //				compmsg->SetExecStep(2);  //跳转下一步
        //				return false;
        //			}
        //
        //			if(!OutputData(msg))
        //				return false;
        //
        //			this->StartMcIntepolate();  //启动MC
        //
        //			compmsg->SetExecStep(1);  //跳转下一步
        //			printf("execute step 0\n");
        //			return false;
        //		case 1:
        //			//第二步：等待MC运行到位
        //			//在函数头处等待
        //			compmsg->SetExecStep(2);  //跳转下一步
        //			printf("execute step 1\n");
        //			return false;
        //		case 2://第三步：将新偏置发送到MC
        //
        //			z_axis_offset = m_p_chn_tool_config->geometry_compensation[value-1][2] * 1e3;  //单位由mm转换为um
        //			z_axis_offset += m_p_chn_tool_config->geometry_wear[value-1] * 1e3;   //叠加磨损补偿
        //
        //			this->SetMcRtcpMode(G43_4_MODE, G43_4_MODE, z_axis_offset);
        //
        //			compmsg->SetExecStep(3);	//跳转下一步
        //			return false;
        //		case 3:
        //			//第二步：等待MC执行新刀偏完成
        //			this->m_p_mc_comm->ReadChnCurIntpMode(this->m_n_channel_index, intp_mode);  //读取MC当前特殊插补模式
        //			if(intp_mode == G43_4_MODE){
        //				compmsg->SetExecStep(4);  //跳转下一步
        //			}
        //			return false;
        //		case 4:
        //			//第三步：同步位置
        //			this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        //
        //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        //
        //			this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
        //
        //			this->m_channel_status.gmode[8] = type;
        //			this->m_channel_status.cur_h_code = value;
        //
        //			this->SendChnStatusChangeCmdToHmi(G_MODE);
        //			this->SendModeChangToHmi(H_MODE);
        //
        //
        //			break;
        //
        //		default:
        //			printf("G43/G44/G49 execute error!\n");
        //			break;
        //		}


    }

    m_n_run_thread_state = RUN;

    printf("execute compensate message: %d, d=%d, h=%d\n", type, m_channel_status.cur_d_code, m_channel_status.cur_h_code);

    return true;
}

/**
 * @brief 实际执行错误指令消息
 * @param msg : 指令消息
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteErrorMsg(RecordMsg *msg){
    //首先将缓冲中的所有待发送指令发送给MC
    //	if(this->m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE)){
    //		if(!this->OutputAllData()){
    //			//PL中的FIFO已满，发送失败
    //			return false;
    //		}
    //	}

    int limit = 1;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){
        if(m_simulate_mode == SIM_OUTLINE || m_simulate_mode == SIM_TOOLPATH)
            break;

        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            //	printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            printf("execute error msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    //生成告警
    ErrorMsg *err = (ErrorMsg *)msg;
    if(err->GetInfoType() == 1){  //告警
        this->m_error_code = (ErrorType)err->GetErrorCode();
        printf("execute error msg: %d, %llu\n", m_error_code, err->GetLineNo());
        if (!m_b_lineno_from_mc)    //llx add,处理加工文件名和行号对应不上问题
            SetCurLineNo(err->GetLineNo());
        this->m_n_run_thread_state = ERROR;
        CreateError(err->GetErrorCode(), ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
    }else if(err->GetInfoType() == 0){  //提示
        if(err->GetErrorCode() == 0){//清空当前提示信息
            ClearHmiInfoMsg();
        }else{ //发送提示信息
            //			CreateError(err->GetErrorCode(), INFO_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            this->SendMessageToHmi(MSG_TIPS, err->GetErrorCode());
        }
        this->m_n_run_thread_state = RUN;
    }

    return true;
}

/**
 * @brief 实际执行子程序调用消息
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteSubProgCallMsg(RecordMsg *msg){
    SubProgCallMsg *sub_msg = (SubProgCallMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        this->m_mode_restart.sub_prog_call++;
        return true;
    }

    if(m_simulate_mode == SIM_OUTLINE && m_simulate_mode == SIM_TOOLPATH){  //轮廓仿真，刀路仿真
        //设置当前行号
        SetCurLineNo(msg->GetLineNo());

        m_n_subprog_count++;

        return true;
    }

    int limit = 1;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            //	printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            printf("execute subprog call msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    bool flag = (m_n_hw_trace_state==REVERSE_TRACE)?true:false;    //是否反向引导
    int mcode = sub_msg->GetMCode(0);

    if(mcode == 98){

    	if(flag){//反向引导
            m_n_subprog_count -= 1;

            if(sub_msg->GetSubProgType() == 2 ||
                    sub_msg->GetSubProgType() == 4){
                char file[kMaxFileNameLen];
                memset(file, 0x00, kMaxFileNameLen);
                sub_msg->GetLastProgFile(file);
                this->SendOpenFileCmdToHmi(file);   //通知打开上一层程序文件
            }

            //设置当前行号
            SetCurLineNo(msg->GetLineNo());

        }else{
            if(m_n_subprog_count >= kMaxSubNestedCount){
                this->m_error_code = ERR_SUB_NESTED_COUNT;   //子程序嵌套层数过多
                CreateError(ERR_SUB_NESTED_COUNT, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                return false;
            }

            if(sub_msg->GetExecStep(0) == 0){
                m_n_subprog_count += 1;

                //TODO 向HMI发送命令打开子文件
                if(this->m_channel_status.chn_work_mode == AUTO_MODE){
                	if(sub_msg->GetSubProgType() == 2 ||
                            sub_msg->GetSubProgType() == 4 ||
                            sub_msg->GetSubProgType() == 6){
                    	char file[kMaxFileNameLen];
                        memset(file, 0x00, kMaxFileNameLen);
                        sub_msg->GetSubProgName(file, false);
                        this->SendOpenFileCmdToHmi(file);
                    }
                }

                //设置当前行号
                SetCurLineNo(msg->GetLineNo());
                this->m_p_f_reg->DM98 = 1;
                sub_msg->IncreaseExecStep(0);
            }else if(sub_msg->GetExecStep(0) == 1){
            	sub_msg->IncreaseExecStep(0);
            }else if(sub_msg->GetExecStep(0) == 2){
            	this->m_p_f_reg->DM98 = 0;
                sub_msg->SetExecStep(0, 0);
            }

            if(sub_msg->GetExecStep(0) > 0)
                return false;
        }
    }
    return true;
}

/**
 * @brief 实际执行宏程序调用消息
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteMacroProgCallMsg(RecordMsg *msg){
    MacroProgCallMsg *macro_msg = (MacroProgCallMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位

        this->m_mode_restart.sub_prog_call++;
        return true;
    }

    //	printf("ChannelControl::ExecuteMacroProgCallMsg\n");

    if(m_simulate_mode == SIM_OUTLINE && m_simulate_mode == SIM_TOOLPATH){  //轮廓仿真，刀路仿真
        //设置当前行号
        SetCurLineNo(msg->GetLineNo());

        m_n_subprog_count++;
        m_n_macroprog_count++;

        return true;
    }

    int limit = 1;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){

        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            //	printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            printf("execute subprog call msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    bool flag = (m_n_hw_trace_state==REVERSE_TRACE)?true:false;    //是否反向引导

    if(flag){ //反向引导
        if(this->m_p_general_config->debug_mode > 0){  //调式模式下，打开宏程序文件
            if(macro_msg->GetMacroProgType() > 1){
                char file[kMaxFileNameLen];
                memset(file, 0x00, kMaxFileNameLen);
                macro_msg->GetLastProgFile(file);
                this->SendOpenFileCmdToHmi(file);
            }
        }

        //设置当前行号
        SetCurLineNo(msg->GetLineNo());

        m_n_subprog_count--;
        m_n_macroprog_count--;

    }else{
        if(m_n_subprog_count >= kMaxSubNestedCount){
            this->m_error_code = ERR_SUB_NESTED_COUNT;   //子程序嵌套层数过多
            CreateError(ERR_SUB_NESTED_COUNT, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            return false;
        }

        //TODO 向HMI发送命令打开子文件
        if(this->m_p_general_config->debug_mode > 0){  //调式模式下，打开宏程序文件
            if(this->m_channel_status.chn_work_mode == AUTO_MODE){
                if(macro_msg->GetMacroProgType() > 1){
                    char file[kMaxFileNameLen];
                    memset(file, 0x00, kMaxFileNameLen);
                    macro_msg->GetMacroProgName(file, false);
                    this->SendOpenFileCmdToHmi(file);
                }
            }
        }

        //设置当前行号
        SetCurLineNo(msg->GetLineNo());

        m_n_subprog_count++;
        m_n_macroprog_count++;

        if(this->m_p_general_config->debug_mode == 0){
            this->SetMcStepMode(false);
        }

    }

    return true;
}

/**
 * @brief 实际执行自动对刀指令消息
 * @param msg : 执行消息指针
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteAutoToolMeasureMsg(RecordMsg *msg){
    AutoToolMeasureMsg *macro_msg = (AutoToolMeasureMsg *)msg;
    int limit = 1;
    bool block_over = false;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    if(this->m_n_restart_mode != NOT_RESTART &&
            macro_msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        this->m_mode_restart.sub_prog_call++;
        return true;
    }

    if(this->m_simulate_mode != SIM_NONE){  //仿真时直接返回
        //设置当前行号
        SetCurLineNo(msg->GetLineNo());
        return true;
    }


    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){
        block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            //	printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            printf("execute subprog call msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }


    if(m_n_subprog_count >= kMaxSubNestedCount){
        this->m_error_code = ERR_SUB_NESTED_COUNT;   //子程序嵌套层数过多
        CreateError(ERR_SUB_NESTED_COUNT, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        return false;
    }

    //TODO 向HMI发送命令打开子文件
    if(this->m_p_general_config->debug_mode > 0){  //调式模式下，打开宏程序文件
        if(this->m_channel_status.chn_work_mode == AUTO_MODE){
            if(macro_msg->GetMacroProgType() > 1){
                char file[kMaxFileNameLen];
                memset(file, 0x00, kMaxFileNameLen);
                macro_msg->GetMacroProgName(file, false);
                this->SendOpenFileCmdToHmi(file);
            }
        }
    }

    //设置当前行号
    SetCurLineNo(msg->GetLineNo());

    m_n_subprog_count++;
    m_n_macroprog_count++;

    if(this->m_p_general_config->debug_mode == 0){  //非调试模式则单段无效
        this->SetMcStepMode(false);
        //	this->m_b_need_change_to_pause = false;
    }
    return true;
}

/**
 * @brief 实际执行加工复位完成指令消息
 * @param msg : 消息指针
 * @return true--成功  false--失败
 */
bool ChannelControl::ExecuteRestartOverMsg(RecordMsg *msg){
    printf("ChannelControl::ExecuteRestartOverMsg\n");

    this->m_n_restart_mode = NOT_RESTART;
    this->m_n_restart_line = 0;
    this->m_n_restart_step = 0;
    this->m_p_compiler->SetRestartPara(0, m_n_restart_mode);   //复位编译器的加工复位标志

    this->ClearHmiInfoMsg();   //清除HMI的提示信息

    //	this->PauseRunGCode();    //FOR TEST  暂停方便观察

    return true;
}

/**
 * @brief
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteMacroCmdMsg(RecordMsg *msg){
    // MacroCmdMsg *macro_msg = static_cast<MacroCmdMsg *>(msg);
    // printf("enter execute macro cmd message\n");

	if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        return true;
    }

    //等待MC运行完所有数据
    int count = 0;
    int limited = 1;
    if(this->IsStepMode())  //单段模式需要多等待一个周期，状态刷新需要一个周期
        limited = 4;
    while(count < limited ){
        if(m_simulate_mode == SIM_OUTLINE || m_simulate_mode == SIM_TOOLPATH)
            break;

        if(this->ReadMcMoveDataCount() > 0 ||
                // @test zk 解决MDI执行宏程序 遇到宏变量计算卡顿问题  不确定会不会产生新问题
                //	(!this->CheckStepOverFlag() && !this->CheckBlockOverFlag()) ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){
            bool flag1 = this->CheckStepOverFlag();
            bool flag2 = this->CheckBlockOverFlag();
            bool flag3 = (!this->CheckStepOverFlag() && !this->CheckBlockOverFlag());
            return false;    //还未运行到位
        }
        else{
            count++;
            //		printf("Macro cmd: step=%d, %d, c = %d\n", m_channel_mc_status.step_over,m_channel_mc_status.auto_block_over, count);
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
        }
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    //设置当前行号
    SetCurLineNo(msg->GetLineNo());

    return true;
}

/**
 * @brief 执行子程序返回消息
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteSubProgReturnMsg(RecordMsg *msg){
    SubProgReturnMsg *ret_msg = static_cast<SubProgReturnMsg *>(msg);

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        return true;
    }

    if(m_simulate_mode == SIM_OUTLINE && m_simulate_mode == SIM_TOOLPATH){  //轮廓仿真，刀路仿真
        //设置当前行号
        SetCurLineNo(msg->GetLineNo());

        if(ret_msg->IsRetFromMacroProg() && m_n_macroprog_count > 0){  //宏程序返回
            m_n_macroprog_count--;
            m_b_ret_from_macroprog = true;
        }
        return true;
    }


    int limit = 1;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            //	printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            printf("execute subprog return msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }


    //TODO 向HMI发送命令打开上级文件文件
#ifdef USES_ADDITIONAL_PROGRAM
    if(this->m_channel_status.chn_work_mode == AUTO_MODE && m_n_add_prog_type == NONE_ADD &&
            (!ret_msg->IsRetFromMacroProg() || this->m_p_general_config->debug_mode > 0)){    //自动模式，并且是子程序返回或者调试模式打开
#else
    if(this->m_channel_status.chn_work_mode == AUTO_MODE &&
            (!ret_msg->IsRetFromMacroProg() || this->m_p_general_config->debug_mode > 0)){    //自动模式，并且是子程序返回或者调试模式打开
#endif
        char file[kMaxFileNameLen];
        memset(file, 0x00, kMaxFileNameLen);
        ret_msg->GetReturnFileName(file);
        this->SendOpenFileCmdToHmi(file);

        printf("execute sub program return msg: file = %s\n", file);
    }

    //设置当前行号
    if(ret_msg->IsRetFromMacroProg() && m_n_macroprog_count > 0){  //宏程序返回，不更新行号
        m_n_macroprog_count--;
        m_b_ret_from_macroprog = true;

        if(this->IsStepMode()){
            this->SetMcStepMode(true);
        }
    }else{
        SetCurLineNo(msg->GetLineNo());
        printf("sub prog return line : %lld\n", msg->GetLineNo());
    }

    return true;
}

/**
 * @brief 执行极坐标插补及磨削类消息
 * @param msg
 * @return
 */
bool ChannelControl::ExecutePolarIntpMsg(RecordMsg *msg){
    PolarIntpMsg *polarmsg = (PolarIntpMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        int cmd = polarmsg->GetGCode();
        this->m_mode_restart.gmode[GetModeGroup(cmd)] = cmd;
        return true;
    }

    if(m_simulate_mode == SIM_OUTLINE && m_simulate_mode == SIM_TOOLPATH){  //轮廓仿真，刀路仿真
        //设置当前行号
        SetCurLineNo(msg->GetLineNo());

        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            //		printf("PolarIntpMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            //		printf("execute PolarIntpMsg[%d]: blockflag=%d, count = %d, step = %hhu\n", polarmsg->GetGCode(),block_over, count, polarmsg->GetExecStep());

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }


    //设置当前行号
    SetCurLineNo(polarmsg->GetLineNo());


    //切换补偿值，更新模态
    int type = polarmsg->GetGCode();
    uint16_t intp_type = type;  //插补方式

    if(intp_type == G13_1_CMD)
        intp_type = 0;


    //	else if(type == G12_2_CMD){//设置磨削状态， 先更新到MC模态信息

    switch(polarmsg->GetExecStep()){
    case 0://第一步：将新插补方式发送到MC
        this->m_channel_status.gmode[21] = type;

#ifdef USES_GRIND_MACHINE
        if(type == G12_2_CMD){//专用磨削G12.2
            //向MC设置磨削动态参数
            int radius = -1;
            int r_index = polarmsg->GetParamR();
            if(r_index >= 1 && r_index <= 6 )
                radius = this->m_p_grind_config->wheel_raidus_multi[r_index -1]*1000;  //单位转换：mm->um
            this->SetMcGrindParamDynamic(intp_type, 0, polarmsg->GetParamP(), radius);  //发送P、R参数
            this->SetMcGrindParamDynamic(intp_type, 1, polarmsg->GetParamQ(), polarmsg->GetParamL());  //发送L、Q参数

            printf("G12.2 send param: type = %d, P=%d, R=%d, Q=%d, L=%d\n", intp_type, polarmsg->GetParamP(), radius,
                   polarmsg->GetParamQ(), polarmsg->GetParamL());
        }else if(type == G12_3_CMD){//专用磨削G12.3
            //向MC设置磨削动态参数
            int radius = -1;
            int r_index = polarmsg->GetParamR();
            if(r_index >= 1 && r_index <= 6 )
                radius = this->m_p_grind_config->wheel_raidus_multi[r_index -1]*1000;  //单位转换：mm->um
            this->SetMcGrindParamDynamic(intp_type, 0, polarmsg->GetParamX(), polarmsg->GetParamY());  //发送X、Y参数
            this->SetMcGrindParamDynamic(intp_type, 1, polarmsg->GetParamI(), polarmsg->GetParamJ());  //发送I、J参数
            this->SetMcGrindParamDynamic(intp_type, 2, polarmsg->GetParamP(), radius);  //发送P、R参数
            this->SetMcGrindParamDynamic(intp_type, 3, polarmsg->GetParamQ(), polarmsg->GetParamL());  //发送L、Q参数

            printf("G12.3 send param: type = %d, X=%d, Y=%d, I=%d, J=%d, P=%d, R=%d, Q=%d, L=%d\n", intp_type, polarmsg->GetParamX(),
                   polarmsg->GetParamY(), polarmsg->GetParamI(), polarmsg->GetParamJ(), polarmsg->GetParamP(), radius,
                   polarmsg->GetParamQ(), polarmsg->GetParamL());
        }
#endif
        //向MC发送设置磨削状态指令
        this->SetMcSpecialIntpMode(intp_type);

        this->SendChnStatusChangeCmdToHmi(G_MODE);

        polarmsg->SetExecStep(1);	//跳转下一步
        return false;
    case 1://第二步
        //第二步：等待MC执行磨削设置完成
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadChnCurIntpMode(this->m_n_channel_index, intp_type);  //读取MC当前特殊插补模式
        else
            this->m_p_mc_arm_comm->ReadChnCurIntpMode(m_n_channel_index, intp_type);
        if(type == G13_1_CMD){
            if(intp_type == CANCEL_MODE)   //取消特殊插补模式
                polarmsg->SetExecStep(2);  //跳转下一步
        }
#ifdef USES_GRIND_MACHINE
        else if(type == G12_2_CMD || type == G12_3_CMD){//专用磨削
            if(intp_type == G53_1_MODE){
                polarmsg->SetExecStep(2);  //跳转下一步
            }
        }
#endif
        return false;
    case 2:
        //第三步：同步位置
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        else
            this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        this->RefreshAxisIntpPos();

        this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置

        polarmsg->SetExecStep(3);  //跳转下一步
        return false;
    case 3:
        //第四步：结束

        break;
    default:
        printf("G12.2/G12.3 execute error!\n");
        break;
    }
    //	}


    m_n_run_thread_state = RUN;


    printf("execute polarintp message:%d\n", type);

    return true;
}

/**
 * @brief 执行参考点返回消息
 * @param msg
 * @return
 */
static int wait_times = 0;
static timeval m_time_ret;
bool ChannelControl::ExecuteRefReturnMsg(RecordMsg *msg){
    RefReturnMsg *refmsg = (RefReturnMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        //	this->m_mode_restart.pos_target = loopmsg->GetTargetPos();
        uint32_t axis_mask = refmsg->GetAxisMask();
        DPointChn pos = m_mode_restart.pos_target;
        for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            if(axis_mask & (0x01<<i)){
                pos.m_df_point[i] = m_p_axis_config[this->GetPhyAxis(i)].axis_home_pos[0];
            }
        }
        this->TransMachCoordToWorkCoord(pos, m_mode_restart.gmode[14], m_mode_restart.cur_h_code, axis_mask);
        m_mode_restart.pos_target = pos;

        return true;
    }

    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){

        //设置当前行号
        SetCurLineNo(refmsg->GetLineNo());

        //		int gcode = refmsg->GetGCode();
        uint32_t axis_mask = refmsg->GetAxisMask();
        DPointChn &mid_pos = refmsg->GetMiddlePos();

        SimulateData data_mid, data_tar;
        MonitorDataType type = MDT_CHN_SIM_GRAPH;  //数据类型

        if(m_simulate_mode == SIM_TOOLPATH)
            type = MDT_CHN_SIM_TOOL_PATH;

        //发送参考点数据
        uint8_t chn_axis = this->GetChnAxisFromName(AXIS_NAME_X);
        if(chn_axis != 0xFF && (axis_mask & (0x01<<chn_axis))){
            data_mid.pos[0] = 	mid_pos.m_df_point[chn_axis];   //中间点位数据
            this->TransWorkCoordToMachCoord(data_mid.pos[0], m_channel_status.gmode[14], chn_axis);  //工件坐标转换为机械坐标

            data_tar.pos[0] = m_p_axis_config[this->GetPhyAxis(chn_axis)].axis_home_pos[0];   //机械坐标
            m_pos_simulate_cur_work.SetAxisValue(chn_axis, data_tar.pos[0]);   //保存仿真当前位置
            this->TransMachCoordToWorkCoord(m_pos_simulate_cur_work.m_df_point[chn_axis], this->m_channel_status.gmode[14], chn_axis);   //转换为工件坐标系
        }
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Y);
        if(chn_axis != 0xFF && (axis_mask & (0x01<<chn_axis))){
            data_mid.pos[1] = 	mid_pos.m_df_point[chn_axis];   //中间点位数据
            this->TransWorkCoordToMachCoord(data_mid.pos[1], m_channel_status.gmode[14], chn_axis);  //工件坐标转换为机械坐标

            data_tar.pos[1] = m_p_axis_config[this->GetPhyAxis(chn_axis)].axis_home_pos[0];   //机械坐标
            m_pos_simulate_cur_work.SetAxisValue(chn_axis, data_tar.pos[1]);   //保存仿真当前位置
            this->TransMachCoordToWorkCoord(m_pos_simulate_cur_work.m_df_point[chn_axis], this->m_channel_status.gmode[14], chn_axis);   //转换为工件坐标系
        }
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
        if(chn_axis != 0xFF && (axis_mask & (0x01<<chn_axis))){
            data_mid.pos[2] = 	mid_pos.m_df_point[chn_axis];   //中间点位数据
            this->TransWorkCoordToMachCoord(data_mid.pos[2], m_channel_status.gmode[14], chn_axis);  //工件坐标转换为机械坐标

            data_tar.pos[2] = m_p_axis_config[this->GetPhyAxis(chn_axis)].axis_home_pos[0];   //机械坐标
            m_pos_simulate_cur_work.SetAxisValue(chn_axis, data_tar.pos[2]);   //保存仿真当前位置
            this->TransMachCoordToWorkCoord(m_pos_simulate_cur_work.m_df_point[chn_axis], this->m_channel_status.gmode[14], chn_axis);   //转换为工件坐标系
        }
        this->SendSimulateDataToHmi(type, data_mid);   //发送中间点
        this->SendSimulateDataToHmi(type, data_tar);   //发送终点

        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){
        if(m_simulate_mode == SIM_OUTLINE || m_simulate_mode == SIM_TOOLPATH)
            break;

        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            //			printf("RefReturnMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            //		printf("execute RefReturnMsg[%d]: blockflag=%d, count = %d\n", refmsg->GetGCode(),block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    //设置当前行号
    SetCurLineNo(refmsg->GetLineNo());

    int gcode = refmsg->GetGCode();
    uint32_t axis_mask = refmsg->GetAxisMask();
    DPointChn &mid_pos = refmsg->GetMiddlePos();

    double *pos = mid_pos.m_df_point;
    //	SCAxisConfig *axis_config = nullptr;
    uint8_t phy_axis = 0;
    uint8_t i = 0;
    bool flag = true;
    int ref_id = refmsg->ref_id;

    if(gcode == G30_CMD and (ref_id < 1 or ref_id > 4)){
        //  参考点序号超出范围
        printf("G30 specify a refpoint not exist ！\n");
        CreateError(ERR_NO_CUR_RUN_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, gcode, m_n_channel_index);
        return false;
    }

    if(gcode == G28_CMD or gcode == G30_CMD){
        switch(refmsg->GetExecStep()){
        case 0:

        	//第一步：控制对应的轴走到中间点位置
            for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                if(axis_mask & (0x01<<i)){
                    phy_axis = this->GetPhyAxis(i);
                    if(phy_axis != 0xff){
                        this->ManualMove(i, pos[i], m_p_axis_config[phy_axis].rapid_speed, true);  //工件坐标系
                        printf("manual move axis=%hhu, target=%lf, speed=%lf\n", i, pos[i], m_p_axis_config[phy_axis].rapid_speed);
                        gettimeofday(&m_time_ret, NULL);
                    }

                }
            }

            refmsg->SetExecStep(1);	//跳转下一步
            return false;
        case 1:
        {
            struct timeval time_now;
            gettimeofday(&time_now, NULL);
            unsigned int time_elpase = (time_now.tv_sec-m_time_ret.tv_sec)*1000000+time_now.tv_usec-m_time_ret.tv_usec;
            if (time_elpase >= 50000) {//等待MC刷新axis_over_mask标记
                //第二步：等待对应轴到位
                for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                    if(axis_mask & (0x01<<i)){
                        //if(fabs(this->m_channel_rt_status.cur_pos_work.GetAxisValue(i) - pos[i]) > 1e-3){ //未到位
                        if ((m_channel_mc_status.manu_axis_over_mask & (0x01<<i)) == 0) {
                            //                        printf("step 1: axis %hhu cur pos = %lf, target=%lf\n", i, m_channel_rt_status.cur_pos_work.GetAxisValue(i), pos[i]);

                            flag = false;
                            //phy_axis = this->GetPhyAxis(i);
                            //if(phy_axis != 0xff){
                            //    this->ManualMove(i, pos[i], m_p_axis_config[phy_axis].rapid_speed, true);  //工件坐标系

                            //}
                            //    printf("cur work pos = %lf, tar pos = %lf\n", m_channel_rt_status.cur_pos_work.GetAxisPos(i), pos[i]);

                        }
                    }
                }
                if(flag)
                    refmsg->SetExecStep(2);  //跳转下一步
            }
            return false;
        }
        case 2:

            printf("ret ref, step 2\n");
            //第三步：控制对应的轴走到机械零点
            if(gcode == G28_CMD){
                for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                    if(axis_mask & (0x01<<i)){
                        phy_axis = this->GetPhyAxis(i);
                        if(phy_axis != 0xff){
#ifdef USES_RET_REF_TO_MACH_ZERO
                            this->ManualMove(i, 0, m_p_axis_config[phy_axis].rapid_speed);  //机械坐标系
                            printf("manual move machpos axis=%hhu, target=0, speed=%lf\n", i,	m_p_axis_config[phy_axis].rapid_speed);
#else
                            this->ManualMove(i, m_p_axis_config[phy_axis].axis_home_pos[0], m_p_axis_config[phy_axis].rapid_speed);  //机械坐标系
                            printf("manual move machpos axis=%hhu, target=%lf, speed=%lf\n", i, m_p_axis_config[phy_axis].axis_home_pos[0],
                                    m_p_axis_config[phy_axis].rapid_speed);
#endif
                        }
                    }
                }
                refmsg->SetExecStep(3);  //跳转下一步
                gettimeofday(&m_time_ret, NULL);
                return false;

            }else{
                // G30
                for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                    if(axis_mask & (0x01<<i)){
                        phy_axis = this->GetPhyAxis(i);
                        if(phy_axis != 0xff){
                            this->ManualMove(i, m_p_axis_config[phy_axis].axis_home_pos[ref_id-1], m_p_axis_config[phy_axis].rapid_speed);  //机械坐标系
                        }
                    }
                }
                refmsg->SetExecStep(3);  //跳转下一步
                gettimeofday(&m_time_ret, NULL);
                return false;
            }

        case 3:{
            struct timeval time_now;
            gettimeofday(&time_now, NULL);
            unsigned int time_elpase = (time_now.tv_sec-m_time_ret.tv_sec)*1000000+time_now.tv_usec-m_time_ret.tv_usec;
            if (time_elpase > 50000) {
                //第四步：等待struct timeval time_now;
                for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                    phy_axis = this->GetPhyAxis(i);
                    double target_pos =  m_p_axis_config[phy_axis].axis_home_pos[0];
#ifdef USES_RET_REF_TO_MACH_ZERO
                double target_pos = 0;
#endif
                    if(gcode == G30_CMD) target_pos = m_p_axis_config[phy_axis].axis_home_pos[ref_id-1];
                    uint8_t mlk_mask = m_p_channel_engine->GetMlkMask();
                    if((axis_mask & (0x01<<i)) && !(mlk_mask & (0x01 << i))){

                        if ((m_channel_mc_status.manu_axis_over_mask & (0x01<<i)) == 0) {
                        //if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(i) - target_pos) > 5e-3){ //未到位
                            //printf("step 3: axis %hhu cur pos = %lf, target=%lf\n", i, m_channel_rt_status.cur_pos_machine.GetAxisValue(i), target_pos);
                            flag = false;
                            //phy_axis = this->GetPhyAxis(i);
                            //if(phy_axis != 0xff){
                                //printf("phy_axis: %d target_pos: %lf\n", phy_axis, target_pos);
                             //   this->ManualMove(i, target_pos, m_p_axis_config[phy_axis].rapid_speed);  //机械坐标系
                            //}
                        }
                    }
                }

                if(flag){
                    refmsg->SetExecStep(4);  //跳转下一步
                    //printf("ret ref, jump to step 4\n");
                }
            }
            return false;
        }
        case 4:
            //第五步：同步位置
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
                //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //同步已编译的轴移动指令的位置

            break;

        default:
            printf("ref return msg execute error!\n");
            break;
        }
    }else if(gcode == G27_CMD){
        switch(refmsg->GetExecStep()){
        case 0:
            printf("G27 step 0 ...\n");
            // 移动到 G27 指定点
            for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                if(axis_mask & (0x01<<i)){
                    phy_axis = this->GetPhyAxis(i);
                    if(phy_axis != 0xff){
                        this->ManualMove(i, pos[i], m_p_axis_config[phy_axis].rapid_speed, true);  //工件坐标系
                    }
                }
            }
            refmsg->SetExecStep(1);
            return false;
        case 1:
            printf("G27 step 1 ...\n");
            //第二步：等待对应轴到位
            for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                if(axis_mask & (0x01<<i)){
                    //printf("axis %hhu abs %lf\n", fabs(this->m_channel_rt_status.cur_pos_work.GetAxisValue(i) - pos[i]));
                    if(fabs(this->m_channel_rt_status.cur_pos_work.GetAxisValue(i) - pos[i]) > 0.001){ //未到位
                        //printf("step 1: axis %hhu cur pos = %lf, target=%lf\n", i, m_channel_rt_status.cur_pos_work.GetAxisValue(i), pos[i]);
                        flag = false;
                        phy_axis = this->GetPhyAxis(i);
                        if(phy_axis != 0xff){
                            this->ManualMove(i, pos[i], m_p_axis_config[phy_axis].rapid_speed, true);  //工件坐标系
                        }
                        //	printf("cur work pos = %lf, tar pos = %lf\n", m_channel_rt_status.cur_pos_work.GetAxisPos(i), pos[i]);
                    }

                }
            }
            if(flag)
                refmsg->SetExecStep(2);  //跳转下一步
            return false;
        case 2:
        {
            //第三步：判断是否为参考点
            // 这里有个局部变量  得加括号 不然编译报错 error: jump to case label
            bool in_pos = false;

            // 检测每个参考点到位信号  因为机械坐标系会延时到达 所以要多检测几次
            for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                if(axis_mask & (0x01<<i)){
                    phy_axis = this->GetPhyAxis(i);
                    if(phy_axis != 0xff){
                        if(i<8){
                            in_pos = CheckFRegState(94, i);
                        }else{
                            in_pos = CheckFRegState(95, i-8);
                        }
                        if(!in_pos) break;
                    }
                }
            }

            if(!in_pos){
                // 没有到位
                wait_times ++;

                if(wait_times > 10){
                    // 十次检测都没到位  判断为回参考点失败 发出警告
                    wait_times = 0;
                    m_error_code = ERR_RET_REF_FAILED;
                    CreateError(ERR_RET_REF_FAILED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, gcode, m_n_channel_index);
                }

                return false;
            }else{
                wait_times = 0;
            }
            refmsg->SetExecStep(3);
            return false;}
        case 3:
            //第四步：同步位置
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            this->RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //同步已编译的轴移动指令的位置
            break;
        }
    }

    m_n_run_thread_state = RUN;

    printf("execute ref return msg\n");

    return true;
}

/**
 * @brief 执行跳转消息
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteSkipMsg(RecordMsg *msg){
    SkipMsg *skipmsg = (SkipMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位

        return true;
    }

    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){  //轮廓仿真，刀路仿真

        //设置当前行号
        SetCurLineNo(skipmsg->GetLineNo());

        if(!OutputData(msg))
            return false;

        m_n_run_thread_state = RUN;

        return true;
    }

    if(skipmsg->GetExecStep() == 0){  //开始执行的阶段才需要等待MC执行完毕
        int limit = 2;
        if(this->IsStepMode())
            limit = 4;	//单步模式需要多次验证，因为状态切换有延时

        //等待MC分块的插补到位信号，以及MI的运行到位信号
        int count = 0;
        while(1){
            bool block_over = CheckBlockOverFlag();
            if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
                //	printf("RefReturnMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
                return false;    //还未运行到位
            }
            else if(++count < limit){
                usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
                //		printf("execute RefReturnMsg[%d]: blockflag=%d, count = %d\n", refmsg->GetGCode(),block_over, count);

            }else
                break;
        }
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }


    //设置当前行号
    SetCurLineNo(skipmsg->GetLineNo());

    uint32_t axis_mask = skipmsg->GetAxisMoveMask();


    switch(skipmsg->GetExecStep()){
    case 0:{
        //第一步：通知MI开始抓取SKIP信号，及对应的轴标志
        this->m_b_pos_captured = false;
        this->m_n_mask_pos_capture = axis_mask;
        this->m_n_mask_captured = 0;

        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.cmd = CMD_MI_ACTIVE_SKIP;
        cmd.data.reserved = 0;   //激活
        cmd.data.data[0] = this->m_p_channel_config->g31_skip_signal/10;  //跳转信号段号
        cmd.data.data[1] = this->m_p_channel_config->g31_skip_signal%10;  // 跳转信号位号
        memcpy(&cmd.data.data[2], &axis_mask, sizeof(uint32_t));           //轴mask
        cmd.data.data[4] = this->m_n_channel_index+1;   //MI中通道号从1开始
        cmd.data.data[5] = this->m_p_channel_config->g31_sig_level;    //跳转信号有效电平
        this->m_p_mi_comm->WriteCmd(cmd);

        printf("send skip cmd to mi, signal:%u, data[0]=%hu, data1=%hu, level=%hu\n", m_p_channel_config->g31_skip_signal, cmd.data.data[0], cmd.data.data[1], cmd.data.data[5]);

        skipmsg->SetExecStep(1);	//跳转下一步
        return false;
    }
    case 1:
        //第二步：将运动数据发送至MC，并启动
        if(!m_b_pos_captured){  //还未捕获信号，防止出现MI先捕获信号，后向MC发送运动数据情况
            if(!OutputData(msg))
                return false;

            this->StartMcIntepolate();   //启动MC执行
            printf("@@@@Send g31 move cmd to mc\n");
        }

        skipmsg->SetExecStep(2);  //跳转下一步
        return false;
    case 2:{
        //第三步：等待MI捕获结果或者MC运行到位
        if(m_b_pos_captured){//MI未捕获到信号

            //给MC发送停止命令
            if(!m_b_mc_need_start)  //已经发送了MC启动命令才发送G31停止
                this->SendMcG31Stop();
            //				printf("send G31 stop to mc\n");

            printf("skip cmd cathe signal, send G31 stop\n");

            skipmsg->SetExecStep(3);  //跳转下一步
            return false;  //继续等待
        }else if(CheckBlockOverFlag() && ReadMcMoveDataCount() == 0){//MC块到位并且缓冲无数据
            //发送关闭G31指令命令
            MiCmdFrame cmd;
            memset(&cmd, 0x00, sizeof(cmd));
            cmd.data.cmd = CMD_MI_ACTIVE_SKIP;
            cmd.data.reserved = 1;   //关闭
            cmd.data.data[0] = this->m_p_channel_config->g31_skip_signal/10;  //跳转信号段号
            cmd.data.data[1] = this->m_p_channel_config->g31_skip_signal%10;  // 跳转信号位号
            memcpy(&cmd.data.data[2], &axis_mask, sizeof(uint32_t));           //轴mask
            cmd.data.data[4] = this->m_n_channel_index+1;   //MI中通道号从1开始
            cmd.data.data[5] = this->m_p_channel_config->g31_sig_level;    //跳转信号有效电平
            this->m_p_mi_comm->WriteCmd(cmd);

            printf("skip cmd catch no signal!\n");

            skipmsg->SetExecStep(4);   //跳转第四步
            return false;
        }

        return false;
    }
    case 3:
        //第四步：等待对应的轴到位,同步位置
        if(CheckBlockOverFlag() && ReadMcMoveDataCount() == 0){//MC块到位并且缓冲无数据
            //从MC同步工件坐标
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //				m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //				m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();

            this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置

            this->m_p_channel_engine->SendMonitorData(false, false);
        }else{
            //		printf("step3: block over=%hhu, datacount=%hu\n", CheckBlockOverFlag(), ReadMcMoveDataCount());
            return false;
        }

        printf("goto step 4\n");
        skipmsg->SetExecStep(4);  //跳转下一步
        return false;
    case 4:

        skipmsg->SetFlag(FLAG_AXIS_MOVE, false);//关闭轴移动属性，防止后面再发送MC启动命令

        break;
    default:
        printf("skip msg execute error!\n");
        break;
    }
    m_n_run_thread_state = RUN;
    printf("execute skip msg over\n");

    return true;
}

#ifdef USES_SPEED_TORQUE_CTRL
/**
 * @brief 执行速度控制消息
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteSpeedCtrlMsg(RecordMsg *msg){
    SpeedCtrlMsg *speedmsg = (SpeedCtrlMsg *)msg;
    printf("--IN--ChannelControl:: ExecuteSpeedCtrlMsg \n");

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位

        //TODO  此处需要添加后续处理步骤

        return true;
    }

    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){  //轮廓仿真，刀路仿真

        //设置当前行号
        SetCurLineNo(speedmsg->GetLineNo());

        m_n_run_thread_state = RUN;

        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            printf("ExecuteSpeedCtrlMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            //		printf("execute RefReturnMsg[%d]: blockflag=%d, count = %d\n", refmsg->GetGCode(),block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    //设置当前行号
    SetCurLineNo(speedmsg->GetLineNo());

    int gcode = speedmsg->GetGCode();
    uint32_t axis_mask = 0;    //实际移动轴mask
    uint8_t axis_count = 0;    //实际移动轴数
    double *speed = speedmsg->GetTargetValue(axis_count, axis_mask);


    uint8_t unit_flag = 0;  // mm/min
    if(gcode == G1000_CMD|| gcode == G1002_CMD)
    {
        unit_flag = 1;  //  rpm--> mm/min
    }
    uint8_t check_flag = 0;  //
    if(gcode == G1002_CMD|| gcode == G1003_CMD)
    {
        check_flag = 1;  //  rpm--> mm/min  检查标志
    }

    uint8_t phy_axis = 0;
    uint8_t i = 0, acount = 0;
    bool flag = true;
    if(gcode == G1000_CMD || gcode == G1001_CMD || gcode == G1002_CMD|| gcode == G1003_CMD ){
        switch(speedmsg->GetExecStep()){
        case 0:
            //第一步：将速度控制数据发送到MI
            for(i = 0; i < m_p_channel_config->chn_axis_count && acount < axis_count; i++){
                if(axis_mask & (0x01<<i)){
                    if (this->m_channel_status.cur_axis_ctrl_mode[i] == 2){   // 判断该轴为速度控制轴
                        phy_axis = this->GetPhyAxis(i);
                        if(phy_axis != 0xff){
                            double feed = speed[acount];
                            if(unit_flag==1){
                                feed *= m_p_axis_config[phy_axis].move_pr;
                            }
                            feed = feed*1000/60;
                            int32_t feed1 = feed;
                            this->SendAxisSpeedCmdToMi(phy_axis, feed1);
                            acount++;
                        }
                    }
                }
            }
            if(check_flag==0)
                speedmsg->SetExecStep(2);	//跳转下一步
            else
                speedmsg->SetExecStep(1);	//跳转下一步
            return false;
        case 1:
            //第二步：等待对应轴到位
            this->m_p_channel_engine->SendMonitorData(false, false);
            for(i = 0; i < m_p_channel_config->chn_axis_count && acount < axis_count; i++){
                if (this->m_channel_status.cur_axis_ctrl_mode[i] == 2){   // 判断该轴为速度控制轴
                    if(axis_mask & (0x01<<i)){
                        double feed = speed[acount];
                        if(unit_flag==1){
                            feed *= m_p_axis_config[phy_axis].move_pr;
                        }
                        feed = feed*1000/60;
                        double errspeed = fabs(feed)*0.1;
                        if(errspeed<3) errspeed = 3;
                        double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(i);
                        if(fabs(curfeed - feed) > errspeed){ //速度未到达
                            flag = false;
                        }
                        printf("axis: %hhu cur feed = %lf, feed = %lf\n", i, curfeed, feed);
                    }
                }
            }
            if(flag)
                speedmsg->SetExecStep(2);  //跳转下一步
            return false;
        case 2:
            //第三步：同步位置
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
                //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //同步已编译的轴移动指令的位置
            break;

        default:
            printf("speed ctrl msg execute error!\n");
            break;
        }
    }

    m_n_run_thread_state = RUN;

    //	this->m_channel_status.gmode[GCode2Mode[gcode/10]] = gcode;
    //	this->SendChnStatusChangeCmdToHmi(G_MODE);

    printf("execute speed ctrl return msg\n");

    return true;
}

/**
 * @brief 执行力矩控制消息
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteTorqueCtrlMsg(RecordMsg *msg){
    TorqueCtrlMsg *torquemsg = (TorqueCtrlMsg *)msg;

    //     printf("--IN--ChannelControl:: ExecuteTorqueCtrlMsg \n");

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位

        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            printf("ExecuteTorqueCtrlMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            //		printf("execute ExecuteTorqueCtrlMsg[%d]: blockflag=%d, count = %d\n", refmsg->GetGCode(),block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    //设置当前行号
    SetCurLineNo(torquemsg->GetLineNo());

    int gcode = torquemsg->GetGCode();
    uint32_t axis_mask = torquemsg->GetAxisMoveMask();  //实际移动轴mask
    uint8_t axis_count = 0;   //实际移动轴数

    double *torque = torquemsg->GetTargetValue(axis_count, axis_mask);

    int16_t speed_lmt = torquemsg->GetSpeedLmtValue();
    uint32_t timeover = 0;   //超时时间，0表示无超时
    struct timeval *start_time = torquemsg->GetStartTimePtr();


    uint8_t check_flag = 1;  // mm/min
    if(gcode == G2000_CMD)
    {
        check_flag = 0;  //  rpm--> mm/min
    }else if(gcode == G2001_CMD || gcode == G2002_CMD || gcode == G2003_CMD){
        timeover = torquemsg->GetTimeoverValue()*1000;   //单位由ms转变为us
    }

    //    printf("execute ExecuteTorqueCtrlMsg: gcode=%d, axis_mask = %d  speed_lmt=%d, timeover=%u \n",(uint32_t)gcode,axis_mask, (uint32_t)speed_lmt, timeover);
    //    printf("execute ExecuteTorqueCtrlMsg: torque_value =%f  %f  %f  %f  \n",torque[0],torque[1],torque[2],torque[3]);

    uint8_t phy_axis = 0;
    uint8_t i = 0;
    bool flag = true;
    //	if(gcode == G2000_CMD || gcode == G2001_CMD ){
    switch(torquemsg->GetExecStep()){
    case 0:
        //第一步：控制对应的轴走到中间点位置
        for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
            if (this->m_channel_status.cur_axis_ctrl_mode[i] == 3){   // 判断该轴为力矩控制轴
                if(axis_mask & (0x01<<i)){
                    phy_axis = this->GetPhyAxis(i);
                    if(phy_axis != 0xff){
                        int16_t tor = torque[i];
                        this->SendAxisTorqueCmdToMi(phy_axis, tor, speed_lmt,0);
                    }
                }
            }
        }
        if(check_flag == 0){
            torquemsg->SetExecStep(2);	//跳转下一步
        }else{
            torquemsg->ResetCheckCount();
            torquemsg->SetExecStep(1);	//跳转下一步
            if(timeover > 0){
                gettimeofday(start_time, nullptr);   //记录起始时间

            }

        }
        return false;
    case 1:
        //第二步：等待对应轴到位
        this->m_p_channel_engine->SendMonitorData(false, false);
        for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
            if (this->m_channel_status.cur_axis_ctrl_mode[i] == 3){   // 判断该轴为力矩控制轴
                if(axis_mask & (0x01<<i)){

                    int16_t tor = torque[i];
                    int16_t err = abs(tor)/10;
                    if(err < 10)
                        err = 10;
                    if(gcode == G2002_CMD || gcode == G2003_CMD){  //以力矩到达为结束条件
                        int16_t cur_tor = this->m_channel_rt_status.cur_feedbck_torque.GetAxisValue(i);
                        if(abs(cur_tor) < (abs(tor)-err)){ //未到指定力矩值
                            flag = false;
                        }
                        printf("axis: %hhu cur curTor = %hd, torque = %hd\n", i, cur_tor, tor);
                    }else if(gcode == G2001_CMD){ //以转速到0为结束条件
                        double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(i);
                        if(fabs(curfeed) > 2) { //未到位
                            flag = false;
                        }
                        printf("axis: %hhu cur curfeed = %lf, torque = %lf\n", i, curfeed, torque[i]);
                    }
                }
            }
        }
        if(flag){
            torquemsg->IncCheckCount();
            if(torquemsg->GetCheckCount() >= 3)
                torquemsg->SetExecStep(2);  //跳转下一步
        }
        else{
            torquemsg->ResetCheckCount();

            if(timeover > 0){//超时判断
                struct timeval time_now;
                gettimeofday(&time_now, nullptr);
                unsigned int time_elpase = (time_now.tv_sec-start_time->tv_sec)*1000000+time_now.tv_usec-start_time->tv_usec;

                if(time_elpase >= timeover){ //超时处理
                    printf("execute torque timeover\n");
                    //轴停止运动
                    for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                        if (this->m_channel_status.cur_axis_ctrl_mode[i] == 3){   // 判断该轴为力矩控制轴
                            if(axis_mask & (0x01<<i)){
                                phy_axis = this->GetPhyAxis(i);
                                if(phy_axis != 0xff){
                                    this->SendAxisTorqueCmdToMi(phy_axis, 0, 0,0);
                                }
                            }
                        }
                    }

                    if(gcode == G2003_CMD){ //G2003超时不需要告警，正常结束
                        torquemsg->SetExecStep(2);  //跳转下一步
                    }else{
                        torquemsg->SetExecStep(10);  //跳转到超时处理步骤
                    }
                }else{
                    printf("time_elpase:%u, timeover=%u\n", time_elpase, timeover);
                    //					printf("start time : %d, %d\n", start_time->tv_sec, start_time->tv_usec);
                }
            }
        }
        return false;
    case 2:
        //第三步：同步位置
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        else
            this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        this->RefreshAxisIntpPos();

        if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
            //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
            this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
        }else
            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //同步已编译的轴移动指令的位置
        break;

    case 10:
        //超时处理, 先同步位置再生成告警
        //同步位置
        this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        this->RefreshAxisIntpPos();

        if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
            //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
            this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
        }else
            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //同步已编译的轴移动指令的位置

        //生成告警
        this->m_error_code = ERR_AXIS_TORQUE_OVERTIME;
        this->m_n_run_thread_state = ERROR;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        printf(" failed to execute  torque ctrl msg\n");
        return true;
    default:
        printf("torque ctrl execute error!\n");
        break;
    }
    //	}

    m_n_run_thread_state = RUN;

    //	this->m_channel_status.gmode[GCode2Mode[gcode/10]] = gcode;
    //	this->SendChnStatusChangeCmdToHmi(G_MODE);

    printf(" execute  torque ctrl  return msg\n");

    return true;
}
#endif

/**
 * @brief 执行延时消息
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteTimeWaitMsg(RecordMsg *msg){
    TimeWaitMsg *timemsg = (TimeWaitMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        return true;
    }

    if(this->m_simulate_mode != SIM_NONE){  //仿真模式
        //设置当前行号
        SetCurLineNo(timemsg->GetLineNo());
        m_n_run_thread_state = RUN;
        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            //		printf("TimeWaitMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            //		printf("execute TimeWaitMsg[%d]: blockflag=%d, count = %d, time = %hhu\n", timemsg->GetGCode(),block_over, count, timemsg->GetTimeCount());

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }


    //设置当前行号
    SetCurLineNo(timemsg->GetLineNo());

    uint32_t time = timemsg->GetDelayTime();

    uint64_t cur_time = 0;
    if(time > 0){

        struct timeval tvNow;
        gettimeofday(&tvNow, NULL);
        cur_time = tvNow.tv_sec*1000+tvNow.tv_usec/1000;  //ms单位
        if(timemsg->GetStartTime() == 0){
            timemsg->SetStartTime(cur_time);
            return false;
        }else if((cur_time-timemsg->GetStartTime()) < time){
            //	printf("execute time wait msg, time = %u, start=%llu, cur = %llu\n", time, timemsg->GetStartTime(), cur_time);
            return false;
        }
    }

    m_n_run_thread_state = RUN;

    //printf("execute time wait msg, time delay : %llu ms\n", cur_time-timemsg->GetStartTime());

    return true;
}

/**
 * @brief 执行清整数圈消息
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteClearCirclePosMsg(RecordMsg *msg){
    ClearCirclePosMsg *clearmsg = (ClearCirclePosMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            clearmsg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        return true;
    }

    if(this->m_simulate_mode != SIM_NONE && this->m_simulate_mode != SIM_MACHINING){  //仿真模式,且非加工仿真，直接返回
        //设置当前行号
        SetCurLineNo(msg->GetLineNo());

        //仿真清整数圈
        uint64_t mask = clearmsg->GetAxisMask();
        double mode = clearmsg->GetCircleMode();
        mode /= 1000;   //由um转换为mm
        //ScPrintf("ClearCirclePosMsg: mask = %04llx mode=%lf", mask, mode);
        for(int i = 0; i < m_p_general_config->axis_count; i++){
            if(((mask>>i) & 0x01) == 0){
                continue;
            }
            int tt = m_pos_simulate_cur_work.m_df_point[this->GetChnAxisFromPhyAxis(i)] / mode;
            m_pos_simulate_cur_work.m_df_point[this->GetChnAxisFromPhyAxis(i)] -= tt * mode;  //更新当前仿真位置
        }

        this->m_p_compiler->SetCurPos(m_pos_simulate_cur_work);   //同步编译器位置
        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时
    else if(clearmsg->GetExecStep() == 0){
        limit = 40;   //延时200ms
    }

    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            //		printf("PolarIntpMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            //		printf("execute ClearCirclePosMsg[%d]: blockflag=%d, count = %d, step = %hhu\n", clearmsg->GetGCode(),block_over, count, clearmsg->GetExecStep());

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }


    //设置当前行号
    SetCurLineNo(clearmsg->GetLineNo());


    uint8_t i = 0;
    uint64_t mask = clearmsg->GetAxisMask();
    switch(clearmsg->GetExecStep()){
    case 0://第一步：发送指令到MI
        this->m_channel_status.gmode[39] = clearmsg->GetGCode();

        //ScPrintf("clear pos message , mask = 0x%llx, mode=%d\n", mask, clearmsg->GetCircleMode());
        //向MI发送清整数圈位置指令
        m_n_mask_clear_pos = 0;
        for(i = 0; i < m_p_general_config->axis_count; i++){
            if(((mask>>i) & 0x01) == 0){
                continue;
            }
            this->SendMiClearPosCmd(i+1, clearmsg->GetCircleMode());
        }

        this->SendChnStatusChangeCmdToHmi(G_MODE);

        clearmsg->SetExecStep(1);	//跳转下一步
        return false;
    case 1://第二步
        //ScPrintf("clear pos message step1 , mask = 0x%llx, m_n_mask_clear_pos = 0x%llx \n", mask, this->m_n_mask_clear_pos);
        //第二步：等待MI设置完成
        if((mask & this->m_n_mask_clear_pos) == mask){  // if(mask == this->m_n_mask_clear_pos){
            clearmsg->SetExecStep(2);	//跳转下一步
        }

        return false;
    case 2:
        //第三步：同步位置

        //从MC同步工件坐标
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        else
            this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        //		m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //		m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        this->RefreshAxisIntpPos();

        this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置

        //		printf("intp pos (%lf, %lf, %lf, %lf)\n", m_channel_mc_status.intp_pos.x, m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z,
        //				m_channel_mc_status.intp_pos.a6);

        //从MI同步机械坐标
        this->m_p_channel_engine->SendMonitorData(false, false);

        //		printf("cur mach pos (%lf, %lf, %lf, %lf)\n", m_channel_rt_status.cur_pos_machine.x, m_channel_rt_status.cur_pos_machine.y,
        //				m_channel_rt_status.cur_pos_machine.z, m_channel_rt_status.cur_pos_machine.a6);

        clearmsg->SetExecStep(3);  //跳转下一步
        return false;
    case 3:
        //第四步：结束

        break;
    default:
        printf("G200 execute error!\n");
        break;
    }

    m_n_run_thread_state = RUN;

    printf("execute clearpos message\n");

    return true;
}

// @add zk 处理 G10 输入数据指令
bool ChannelControl::ExecuteInputMsg(RecordMsg * msg){

    InputMsg * input_msg = (InputMsg *)msg;

    bool isAbs = this->m_channel_status.gmode[3] == 900 ? true: false;

    printf("ldata: %lf -- pdata: %lf --- rdata: %lf\n", input_msg->LData, input_msg->PData, input_msg->RData);

    if(this->m_n_restart_mode != NOT_RESTART &&
            input_msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        return true;
    }

    int count = 0;
    while(count < 4 ){
        if(this->ReadMcMoveDataCount() > 0 || !this->CheckBlockOverFlag() ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){
            return false;    //还未运行到位
        }
        else{
            count++;
            //			printf("ExecuteModeMsg: step=%d, %d, c = %d\n", m_channel_mc_status.step_over,m_channel_mc_status.auto_block_over, count);
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
        }
    }

    //单段模式
    if(this->IsStepMode()){
        if(this->m_b_need_change_to_pause){//单段，切换暂停状态
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }

        //设置当前行号
        SetCurLineNo(msg->GetLineNo());

    }else{//非单段模式
        if(m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING)
            return false;
    }

    int input_type = input_msg->LData;
    int tool_number = input_msg->PData;
    int plane = this->m_mc_mode_exec.bits.mode_g17;

    if(input_type < 20){
        if(tool_number <= 0 or tool_number > kMaxToolCount){
            // @TODO 刀具号超出范围
            CreateError(ERR_NO_CUR_RUN_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            this->m_error_code = ERR_NO_CUR_RUN_DATA;
            return false;
        }
    }

    printf("input type %d\n", input_type);

    switch(input_type){
    case 10:{
        // XY平面 刀长补偿 Z   TODO 其他平面的情况下 刀长补偿到不同的轴号
    	double comp_data;
    	if(isAbs){
        	if(plane == 0){
                this->m_p_chn_tool_config->geometry_compensation[tool_number-1][2] = input_msg->RData;
                comp_data = this->m_p_chn_tool_config->geometry_compensation[tool_number-1][2];
        	}else if(plane == 1){
                this->m_p_chn_tool_config->geometry_compensation[tool_number-1][1] = input_msg->RData;
                comp_data = this->m_p_chn_tool_config->geometry_compensation[tool_number-1][1];
        	}else if(plane == 3){
                this->m_p_chn_tool_config->geometry_compensation[tool_number-1][0] = input_msg->RData;
                comp_data = this->m_p_chn_tool_config->geometry_compensation[tool_number-1][0];
        	}
    	}else{
        	if(plane == 0){
                this->m_p_chn_tool_config->geometry_compensation[tool_number-1][2] += input_msg->RData;
                comp_data = this->m_p_chn_tool_config->geometry_compensation[tool_number-1][2];
        	}else if(plane == 1){
                this->m_p_chn_tool_config->geometry_compensation[tool_number-1][1] += input_msg->RData;
                comp_data = this->m_p_chn_tool_config->geometry_compensation[tool_number-1][1];
        	}else if(plane == 3){
                this->m_p_chn_tool_config->geometry_compensation[tool_number-1][0] += input_msg->RData;
                comp_data = this->m_p_chn_tool_config->geometry_compensation[tool_number-1][0];
        	}
    	}

        g_ptr_parm_manager->UpdateToolMeasure(this->m_n_channel_index, tool_number-1, comp_data);
        this->NotifyHmiToolOffsetChanged(tool_number);   //通知HMI刀偏值更改
        break;
    }
    case 11:{

    	if(isAbs)
    		this->m_p_chn_tool_config->radius_compensation[tool_number-1] = input_msg->RData;
    	else
    		this->m_p_chn_tool_config->radius_compensation[tool_number-1] += input_msg->RData;

    	double comp_data = this->m_p_chn_tool_config->radius_compensation[tool_number-1];

        g_ptr_parm_manager->UpdateToolRadiusGeo(this->m_n_channel_index, tool_number-1, comp_data);
        this->NotifyHmiToolOffsetChanged(tool_number);   //通知HMI刀偏值更改
        break;
    }
    case 12:{
    	if(isAbs)
    		this->m_p_chn_tool_config->geometry_wear[tool_number-1] = input_msg->RData;
    	else
    		this->m_p_chn_tool_config->geometry_wear[tool_number-1] += input_msg->RData;

    	double comp_data = this->m_p_chn_tool_config->geometry_wear[tool_number-1];

        g_ptr_parm_manager->UpdateToolWear(this->m_n_channel_index, tool_number-1, comp_data);
        this->NotifyHmiToolOffsetChanged(tool_number);   //通知HMI刀偏值更改
        break;
    }
    case 13:{
        if(isAbs)
        	this->m_p_chn_tool_config->radius_wear[tool_number-1] = input_msg->RData;
        else
        	this->m_p_chn_tool_config->radius_wear[tool_number-1] += input_msg->RData;

        double comp_data = this->m_p_chn_tool_config->radius_wear[tool_number-1];
        g_ptr_parm_manager->UpdateToolRadiusWear(this->m_n_channel_index, tool_number-1, comp_data);
        this->NotifyHmiToolOffsetChanged(tool_number);   //通知HMI刀偏值更改
        break;
    }
    case 20:{
        int param = input_msg->PData;
        //printf("11111 pdata: %lf --- rdata: %lf\n", input_msg->PData, input_msg->RData);

        if(SetSysVarValue(param, input_msg->RData)){
        	//printf("22222 param: %d - value: %lf\n", param, input_msg->RData);
        	break;
        }
        else{
        	//printf("33333 param: %d - value: %lf\n", param, input_msg->RData);
        	CreateError(ERR_NO_CUR_RUN_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            this->m_error_code = ERR_NO_CUR_RUN_DATA;
            return false;
        }
    }
    case 50:{  // 修改系统参数, 立即生效的才有用(有些参数这样改会存在问题)

    }
    default:{
        // @TODO 不支持的L指定
        CreateError(ERR_NO_CUR_RUN_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        this->m_error_code = ERR_NO_CUR_RUN_DATA;
        return false;
    }
    }

    return true;
}

bool ChannelControl::ExecuteExactStopMsg(RecordMsg *msg){
    ExactStopMsg * exact_msg = (ExactStopMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        //	this->m_mode_restart.pos_target = loopmsg->GetTargetPos();
        uint32_t axis_mask = exact_msg->GetAxisMask();
        DPointChn pos = m_mode_restart.pos_target;
        for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            if(axis_mask & (0x01<<i)){
                pos.m_df_point[i] = m_p_axis_config[this->GetPhyAxis(i)].axis_home_pos[0];
            }
        }
        this->TransMachCoordToWorkCoord(pos, m_mode_restart.gmode[14], m_mode_restart.cur_h_code, axis_mask);
        m_mode_restart.pos_target = pos;

        return true;
    }

    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){
        // @TODO 模拟模式下动作
        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//单步模式需要多次验证，因为状态切换有延时

    //等待MC分块的插补到位信号，以及MI的运行到位信号
    int count = 0;
    while(1){
        if(m_simulate_mode == SIM_OUTLINE || m_simulate_mode == SIM_TOOLPATH)
            break;

        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
            //			printf("RefReturnMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //还未运行到位
        }
        else if(++count < limit){
            usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
            //		printf("execute RefReturnMsg[%d]: blockflag=%d, count = %d\n", refmsg->GetGCode(),block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    //设置当前行号
    SetCurLineNo(exact_msg->GetLineNo());

    double * pos = exact_msg->m_point_target.m_df_point;
    uint32_t axis_mask = exact_msg->GetAxisMask();
    uint8_t phy_axis = 0;
    bool flag = true;
    // 等待各轴到位
    for(int i = 0; i < m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            if(fabs(this->m_channel_rt_status.cur_pos_work.GetAxisValue(i) - pos[i]) > 1e-3){ //未到位

                flag = false;
                phy_axis = this->GetPhyAxis(i);
                printf("phy_axis %d target pos: %lf\n", phy_axis, pos[i]);
                if(phy_axis != 0xff){
                    this->ManualMove(i, pos[i], m_p_axis_config[phy_axis].rapid_speed, true);  //工件坐标系

                }
                //	printf("cur work pos = %lf, tar pos = %lf\n", m_channel_rt_status.cur_pos_work.GetAxisPos(i), pos[i]);
            }
        }
    }

    if(!flag) return false;

    //第四步：同步位置
    if(!this->m_b_mc_on_arm)
        this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
    else
        this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

    this->RefreshAxisIntpPos();

    if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
        this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
    }else
        this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //同步已编译的轴移动指令的位置

    m_n_run_thread_state = RUN;
    // @test zk 不知道为何  必须重新启动插补才能继续下面的运动指令
    StartMcIntepolate();
    return true;
}


/**
 * @brief 设置功能状态，例如：单段，选停等等
 * @param state : 设置的状态
 * @param mode : 状态的开关   0--关闭   1--打开    10--点动（即根据当前状态取反）
 */
void ChannelControl::SetFuncState(int state, uint8_t mode){
    bool cur_check = m_channel_status.func_state_flags.CheckMask(state);
    if(mode != 10){
        if(mode == 0 && !cur_check)  //无变化
            return;
        else if(mode == 1 && cur_check)
            return;
    }

    if(state == FS_SINGLE_LINE){//单段
        if(mode == 10){   //点动
            if(m_channel_status.func_state_flags.CheckMask(state)){
                this->SetMcStepMode(false);
                this->m_b_need_change_to_pause = false;
            }
            else if(this->m_n_macroprog_count == 0 || this->m_p_general_config->debug_mode > 0){  //非宏程序调用，并关闭调试模式
                this->SetMcStepMode(true);
            }
        }else if(mode == 0){  //关闭
            this->SetMcStepMode(false);
            this->m_b_need_change_to_pause = false;
        }else if(this->m_n_macroprog_count == 0 || this->m_p_general_config->debug_mode > 0){//打开
            this->SetMcStepMode(true);
        }

    }

    if(mode == 10){   //点动
        if(m_channel_status.func_state_flags.CheckMask(state)){
            m_channel_status.func_state_flags.ResetMask(state);
            if(state == FS_SINGLE_LINE){
                this->m_p_f_reg->MSBK = 0;
            }else if(state == FS_BLOCK_SKIP){
                this->m_p_f_reg->MBDT1 = 0;
            }
        }else{
            m_channel_status.func_state_flags.SetMask(state);
            if(state == FS_SINGLE_LINE){
                this->m_p_f_reg->MSBK = 1;
            }else if(state == FS_BLOCK_SKIP){
                this->m_p_f_reg->MBDT1 = 1;
            }
        }


    }else if(mode == 0){//关闭
        m_channel_status.func_state_flags.ResetMask(state);
        if(state == FS_SINGLE_LINE){
            this->m_p_f_reg->MSBK = 0;
        }else if(state == FS_BLOCK_SKIP){
            this->m_p_f_reg->MBDT1 = 0;
        }
    }else{//打开
        m_channel_status.func_state_flags.SetMask(state);
        if(state == FS_SINGLE_LINE){
            this->m_p_f_reg->MSBK = 1;
        }else if(state == FS_BLOCK_SKIP){
            this->m_p_f_reg->MBDT1 = 1;
        }
    }

    const vector<string> table = {"单段执行", "空运行", "选择停止", "手轮模拟", "单段跳跃", "编辑锁", "机床锁",
                                 "机床辅助锁", "手动快速"};
    if(state >= 0 && state < (int)table.size())
    {
        if (mode == 0)
            g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "关闭[" + table[state] + "]");
        else if (mode == 10)
            g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "开启[" + table[state] + "]点动");
        else
            g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "开启[" + table[state] + "]");
    }
    //通知HMI
    this->SendChnStatusChangeCmdToHmi(FUNC_STATE);
}

/**
 * @brief 通道是否处于func状态
 * @param func : 指定状态
 * @return
 */
bool ChannelControl::CheckFuncState(int func){
    return m_channel_status.func_state_flags.CheckMask(func);
}

/**
 * @brief 设置自动倍率
 * @param ratio
 */
void ChannelControl::SetAutoRatio(uint8_t ratio){
    //	printf("ChannelControl::SetAutoRatio, chn=%hhu, old_r = %hhu, ratio=%hhu\n", m_n_channel_index, m_channel_status.auto_ratio, ratio);
    this->m_channel_status.auto_ratio = ratio;

    g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "[进给倍率]切换为 "+to_string(ratio));
    //通知MC
    this->SetMcRatio();

    //通知HMI
    this->SendChnStatusChangeCmdToHmi(AUTO_RATIO);
}

/**
 * @brief 设置手动倍率
 * @param ratio
 */
void ChannelControl::SetManualRatio(uint8_t ratio){
    if(m_channel_status.manual_ratio == ratio)
        return;
    this->m_channel_status.manual_ratio = ratio;

    g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "[手动倍率]切换为 "+to_string(ratio));
    //通知MC
    this->SetMcRatio();

    //通知HMI
    this->SendChnStatusChangeCmdToHmi(MANUAL_RATIO);
}

/**
 * @brief 设置快速进给倍率
 * @param ratio
 */
void ChannelControl::SetRapidRatio(uint8_t ratio){

    this->m_channel_status.rapid_ratio = ratio;

    g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "[快移倍率]切换为 "+to_string(ratio));

    //通知MC
    this->SetMcRatio();

    //通知HMI
    this->SendChnStatusChangeCmdToHmi(RAPID_RATIO);
}

/**
 * @brief 设置主轴倍率
 * @param ratio
 */
void ChannelControl::SetSpindleRatio(uint8_t ratio){
    this->m_channel_status.spindle_ratio = ratio;

    m_p_spindle->InputSOV(ratio);

    g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "[主轴倍率]切换为 "+to_string(ratio));
    //通知HMI
    this->SendChnStatusChangeCmdToHmi(SPINDLE_RATIO);
}

/**
 * @brief 设置手动步长
 * @param step : 步长， 1--1um  2--10um  3--100um   4--1000um
 */
void ChannelControl::SetManualStep(uint8_t step){

    if(this->m_channel_status.manual_step == step)
        return;

    this->m_channel_status.manual_step = step;



    //	printf("set manual step : %hhu\n", step);

    //通知HMI
    this->SendChnStatusChangeCmdToHmi(MANUAL_STEP);
}

/**
 * @brief 设置手动快速移动状态
 * @param mode : 状态的开关   0--关闭   1--打开    10--点动（即根据当前状态取反）
 */
void ChannelControl::SetManualRapidMove(uint8_t mode){
    //	if(m_channel_status.chn_work_mode != MANUAL_MODE &&
    //			m_channel_status.chn_work_mode != MANUAL_STEP_MODE){  //非手动模式，不做响应，退出
    //		return;
    //	}

    if(mode == 10){
        if(m_channel_status.func_state_flags.CheckMask(FS_MANUAL_RAPID)){
            m_channel_status.func_state_flags.SetMask(FS_MANUAL_RAPID, 0);
            printf("manual rapid 0\n");
        }
        else{
            m_channel_status.func_state_flags.SetMask(FS_MANUAL_RAPID, 1);
            printf("manual rapid 1\n");
        }
    }else if(mode == 0){
        if(!m_channel_status.func_state_flags.CheckMask(FS_MANUAL_RAPID))  //不变，直接返回
            return;
        m_channel_status.func_state_flags.SetMask(FS_MANUAL_RAPID, 0);
    }else{
        if(m_channel_status.func_state_flags.CheckMask(FS_MANUAL_RAPID))   //不变，直接返回
            return;
        m_channel_status.func_state_flags.SetMask(FS_MANUAL_RAPID, 1);
    }


    //重发手动命令
//    if(m_channel_status.cur_manual_move_dir != DIR_STOP){
//        this->ManualMove(m_channel_status.cur_manual_move_dir);
//    }
}

/**
 * @brief 设置当前轴
 * @param axis : 当前轴号
 */
void ChannelControl::SetCurAxis(uint8_t axis){
    if(this->m_channel_status.cur_axis == axis)
        return;
    this->m_channel_status.cur_axis = axis;

    //通知HMI
    this->SendChnStatusChangeCmdToHmi(CUR_AXIS);
}

/**
 * @brief 手轮移动指令
 * @param hw_count : 手轮摇动的格数，正向为正数，负向为负数
 */
void ChannelControl::HandwheelMove(int32_t hw_count){
    if(m_channel_status.chn_work_mode != MPG_MODE){  //非手轮模式，不做响应，退出
        return;
    }
}

/**
 * @brief 获取通道轴对应的物理轴索引号，0开始
 * @param chn_axis :  通道轴号，0开始
 * @return 返回对应的物理轴索引，不成功返回0xff，成功返回0开始的物理轴索引号
 */
uint8_t ChannelControl::GetPhyAxis(uint8_t chn_axis){
    if(chn_axis >= m_p_channel_config->chn_axis_count)
        return 0xff;

    if(m_p_channel_config->chn_axis_phy[chn_axis] == 0)  // if(m_channel_status.cur_chn_axis_phy[chn_axis] == 0)
        return 0xff;  //未配置


    return m_p_channel_config->chn_axis_phy[chn_axis]-1;  // return m_channel_status.cur_chn_axis_phy[chn_axis]-1;
}

/**
 * @brief 获得对应物理轴的通道轴索引号，0开始
 * @param phy_axis : 物理轴号， 0开始
 * @return 返回对应的通道轴索引，不成功返回0xff，成功返回0开始的通道轴索引号
 */
uint8_t ChannelControl::GetChnAxisFromPhyAxis(uint8_t phy_axis){
    uint8_t res = 0xFF;

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(this->m_p_channel_config->chn_axis_phy[i] == phy_axis+1){  //		if(this->m_channel_status.cur_chn_axis_phy[i] == phy_axis+1){
            res = i;
            break;
        }
    }

    return res;
}

void ChannelControl::SyncMcPosition(){
    if(!this->m_b_mc_on_arm)
        this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
    else
        this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

    this->RefreshAxisIntpPos();
    this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
}

/**
 * @brief 获取对应通道轴名称的物理轴索引号，0开始
 * @param axis_name : 轴名称索引
 * @return 返回对应的物理轴索引，不成功返回0xff，成功返回0开始的物理轴索引号
 */
uint8_t ChannelControl::GetPhyAxisFromName(uint8_t axis_name){
    uint8_t index = 0xFF;

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(this->m_p_channel_config->chn_axis_name[i] == axis_name){
            index = i;
            break;
        }
    }
    if(index == 0xFF)
        return 0xff;

    if(m_p_channel_config->chn_axis_phy[index] == 0){ //	if(m_channel_status.cur_chn_axis_phy[index] == 0)
        return 0xff;  //未配置
    }

    return m_p_channel_config->chn_axis_phy[index]-1; //return m_channel_status.cur_chn_axis_phy[index]-1;
}

/**
 * @brief 获取对应通道轴名称的通道轴索引号，0开始
 * @param axis_name : 轴名称索引
 * @return 返回对应的通道轴索引，不成功返回0xff，成功返回0开始的通道轴索引号
 */
uint8_t ChannelControl::GetChnAxisFromName(uint8_t axis_name){
    uint8_t index = 0xFF;

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(this->m_p_channel_config->chn_axis_name[i] == axis_name){
            index = i;
            break;
        }
    }
    return index;
}

/**
 * @brief 获取指定序号的主轴对应的通道轴号，0开始
 * @param spd_idx : 主轴序号，0开始
 * @return 返回对应的通道轴号，0开始, 失败返回0xFF
 */
uint8_t ChannelControl::GetSpdChnAxis(uint8_t spd_idx){
    if(spd_idx >= this->m_n_spindle_count)
        return 0xFF;

    return this->GetChnAxisFromPhyAxis(this->m_spd_axis_phy[spd_idx]-1);
}


/**
 * @brief 手动移动，向dir方向移动dis距离
 * @param axis : 通道轴号，从0开始
 * @param dir : 运动方向
 * @param vel : 运动速度, 单位：mm/min
 * @param inc_dis ： 增量位置
 */
//void ChannelControl::ManualMove2(uint8_t axis, int8_t dir, double vel, double inc_dis){

//    //检查硬限位
//    uint8_t phy_axis = this->GetPhyAxis(axis);
//    if(phy_axis != 0xff){
//        if(m_p_channel_engine->CheckAxisHardLimit(phy_axis, dir)){   //硬限位告警，直接返回
//            printf("hard limit active, manual move return 1 \n");
//            return;
//        }
//    }

//    McCmdFrame cmd;
//    memset(&cmd, 0x00, sizeof(McCmdFrame));

//    cmd.data.channel_index = m_n_channel_index;
//    cmd.data.axis_index = axis+1;   //轴号从1开始
//    cmd.data.cmd = CMD_MC_AXIS_MAUAL_MOVE;

//    //设置速度
//    uint32_t feed = vel*1000/60;   //转换单位为um/s

//    //	memcpy(cmd.data.data, &feed, sizeof(feed));
//    cmd.data.data[0] = (feed & 0xFFFF);
//    cmd.data.data[1] = ((feed>>16)&0xFFFF);

//    //设置目标位置
//    int64_t tar_pos = fabs(inc_dis) * 1e7 *dir;   //单位转换：mm-->0.1nms


//    if((m_channel_status.returned_to_ref_point & (0x01<<axis))
//            && m_p_axis_config[phy_axis].soft_limit_check_1 == 1){//软限位1有效
//        int64_t cur_pos = this->GetAxisCurMachPos(axis)*1e7;  //当前位置
//        int64_t limit_pos = 0;
//        int8_t dir = (tar_pos > cur_pos)?DIR_POSITIVE:DIR_NEGATIVE;
//        if(dir == DIR_POSITIVE){//正向
//            limit_pos = m_p_axis_config[phy_axis].soft_limit_max_1*1e7;


//            if(limit_pos < cur_pos){//已经在限位外，告警
//                CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index, m_channel_status.cur_axis);
//                //	this->m_error_code = ERR_SOFTLIMIT_POS;
//                return;
//            }else if(limit_pos == cur_pos){  //已经到达正限位极限位置
//                CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index, m_channel_status.cur_axis);
//                return;
//            }else if(tar_pos > (limit_pos-cur_pos)){
//                tar_pos = limit_pos - this->GetAxisCurIntpTarPos(m_channel_status.cur_axis, true)*1e7;

//            }
//        }else if(dir == DIR_NEGATIVE){//负向
//            limit_pos = m_p_axis_config[phy_axis].soft_limit_min_1*1e7;

//            if(limit_pos > cur_pos){//已经在限位外，告警
//                CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index, m_channel_status.cur_axis);
//                //	this->m_error_code = ERR_SOFTLIMIT_NEG;
//                return;
//            }else if(limit_pos == cur_pos){  //已经到达负限位极限位置
//                CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index, m_channel_status.cur_axis);
//                return;
//            }else if(tar_pos < (limit_pos-cur_pos)){
//                tar_pos = limit_pos-this->GetAxisCurIntpTarPos(m_channel_status.cur_axis, true)*1e7;
//            }
//        }
//    }

//    cmd.data.data[2] = (tar_pos & 0xFFFF);
//    cmd.data.data[3] = ((tar_pos>>16)&0xFFFF);
//    cmd.data.data[4] = ((tar_pos>>32) & 0xFFFF);
//    cmd.data.data[5] = ((tar_pos>>48)&0xFFFF);

//    cmd.data.data[6] = 0x02;   //增量目标位置

//    if(!this->m_b_mc_on_arm)
//        m_p_mc_comm->WriteCmd(cmd);
//    else
//        m_p_mc_arm_comm->WriteCmd(cmd);

//    printf("manual move2: axis = %d, tar_pos = %lld, type = 0x%x\n", axis, tar_pos, cmd.data.data[6]);
//}

/**
 * @brief 手动移动轴
 * @param dir : 运动方向
 */
void ChannelControl::ManualMove(int8_t dir){
    if(m_channel_status.chn_work_mode != MANUAL_MODE &&
            m_channel_status.chn_work_mode != MANUAL_STEP_MODE){  //非手动模式，不做响应，退出
    	return;
    }
    // printf("send manualmove cmd to mc\n");

    //检查硬限位
    uint8_t phy_axis = this->GetPhyAxis(m_channel_status.cur_axis);
    if(phy_axis != 0xff){
        if(m_p_channel_engine->CheckAxisHardLimit(phy_axis, dir)){   //硬限位告警，直接返回
            printf("hard limit active, manual move return 2 \n");
            return;
        }
    }
    //设置目标位置

    //int64_t cur_pos = GetAxisCurMachPos(m_channel_status.cur_axis)*1e7;  //当前位置
    int64_t cur_pos = GetAxisCurIntpTarPos(m_channel_status.cur_axis, false)*1e7;
    int64_t tar_pos = 0;
    if(GetChnWorkMode() == MANUAL_STEP_MODE){ //手动单步
        tar_pos = cur_pos + GetCurManualStep()*1e4*dir;		//转换单位为0.1nm

    }else{
        tar_pos = cur_pos + 99999*1e7*dir;    //手动连续模式，将目标位置设置的很远
    }
    //ScPrintf("mode == %d tar_pos = %lld",GetChnWorkMode(), tar_pos);
    //检查软限位
    double limit = 0;
    if(CheckSoftLimit((ManualMoveDir)dir, phy_axis,
                      GetAxisCurMachPos(m_channel_status.cur_axis))){
        // ScPrintf("soft limit active, manual move abs return \n");
        return;
    }else if(GetSoftLimt((ManualMoveDir)dir, phy_axis, limit) && dir == DIR_POSITIVE && tar_pos > limit*1e7){
        tar_pos = limit * 1e7;
    }else if(GetSoftLimt((ManualMoveDir)dir, phy_axis, limit) && dir == DIR_NEGATIVE && tar_pos < limit*1e7){
        tar_pos = limit * 1e7;
    }
    //ScPrintf("GetAxisCurIntpTarPos = %llf", GetAxisCurIntpTarPos(m_channel_status.cur_axis, true)*1e7);
    int64_t n_inc_dis = tar_pos - GetAxisCurIntpTarPos(m_channel_status.cur_axis, false)*1e7;
    if((m_p_channel_engine->GetMlkMask() & (0x01<<m_channel_status.cur_axis))
            && GetChnWorkMode() == MANUAL_STEP_MODE){
        n_inc_dis = GetCurManualStep()*1e4*dir;
    }

    if(g_ptr_chn_engine->GetPmcActive(this->GetPhyAxis(m_channel_status.cur_axis))) {
        //this->ManualMovePmc(dir);
        m_error_code = ERR_PMC_IVALID_USED;
        CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        return;
    }

    m_channel_status.cur_manual_move_dir = dir;   //保存方向

    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = m_channel_status.cur_axis+1;   //轴号从1开始
    cmd.data.cmd = CMD_MC_AXIS_MAUAL_MOVE;

    //设置速度
    uint32_t feed = this->m_p_axis_config[phy_axis].manual_speed*1000/60;   //转换单位为um/s
    if(this->m_channel_status.func_state_flags.CheckMask(FS_MANUAL_RAPID)){
        feed *= 2;  //速度翻倍
        printf("double manual feed\n");
    }
    // ScPrintf("axis:%u, feed:%u, n_inc_dis=%lld",m_channel_status.cur_axis,feed, n_inc_dis);

    cmd.data.data[0] = (feed & 0xFFFF);
    cmd.data.data[1] = ((feed>>16)&0xFFFF);
    cmd.data.data[2] = (n_inc_dis & 0xFFFF);
    cmd.data.data[3] = ((n_inc_dis>>16)&0xFFFF);
    cmd.data.data[4] = ((n_inc_dis>>32) & 0xFFFF);
    cmd.data.data[5] = ((n_inc_dis>>48)&0xFFFF);

    cmd.data.data[6] = 0x02;   //增量目标位置

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);

    printf("manual move: axis = %d, tar_pos = %lld, type = 0x%x, ratio = %hhu\n", m_channel_status.cur_axis, tar_pos, cmd.data.data[6],
            this->m_channel_status.manual_ratio);
}

/**
 * @brief 手动移动PMC轴
 * @param dir : 运动方向
 */
void ChannelControl::ManualMovePmc(int8_t dir){
    uint8_t phy_axis = this->GetPhyAxis(m_channel_status.cur_axis);  //当前轴对应的物理轴号
    m_channel_status.cur_manual_move_dir = dir;   //保存方向

    PmcCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(PmcCmdFrame));

    //命令号  低字节:0--单轴指令   1--多轴G01指令  高字节：0--绝对坐标   1--增量坐标
    cmd.data.cmd = 0x100;

    //轴号    低字节：轴号[1-64]    高字节：所属通道号[1-4]
    cmd.data.axis_index = ((phy_axis+1)|((m_n_channel_index+1)<<8));

    cmd.data.data[0] = 0;   //定位移动，赋0


    //设置速度
    uint32_t feed = this->m_p_axis_config[phy_axis].manual_speed * 1000 / 60;   //转换单位为um/s
    if(this->m_channel_status.func_state_flags.CheckMask(FS_MANUAL_RAPID)){
        feed *= 2;  //速度翻倍
        printf("double manual feed\n");
    }

    //设置目标位置
    int64_t cur_pos = GetAxisCurMachPos(m_channel_status.cur_axis)*1e7;  //当前位置
    int64_t tar_pos = 0;
    if(GetChnWorkMode() == MANUAL_STEP_MODE){ //手动单步
        tar_pos = cur_pos + GetCurManualStep()*1e4*dir;		//转换单位为0.1nm

    }else{
        tar_pos = cur_pos + 99999*1e7*dir;    //手动连续模式，将目标位置设置的很远
    }

    //检查软限位
    double limit = 0;
    if(CheckSoftLimit((ManualMoveDir)dir, phy_axis,
                      GetAxisCurMachPos(m_channel_status.cur_axis))){
        printf("soft limit active, manual move abs return \n");
        return;
    }else if(GetSoftLimt((ManualMoveDir)dir, phy_axis, limit) && dir == DIR_POSITIVE && tar_pos > limit * 1e7){
        tar_pos = limit * 1e7;
    }else if(GetSoftLimt((ManualMoveDir)dir, phy_axis, limit) && dir == DIR_NEGATIVE && tar_pos < limit * 1e7){
        tar_pos = limit * 1e7;
    }
    int64_t n_inc_dis = tar_pos - GetAxisCurIntpTarPos(m_channel_status.cur_axis, true)*1e7;

    if((m_p_channel_engine->GetMlkMask() & (0x01<<m_channel_status.cur_axis))
            && GetChnWorkMode() == MANUAL_STEP_MODE){
        n_inc_dis = GetCurManualStep()*1e4*dir;
    }

    memcpy(&cmd.data.data[1], &n_inc_dis, sizeof(tar_pos));  //设置目标位置

    memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //设置速度

    this->m_mask_run_pmc = 0x01L<<phy_axis;  //设置当前运行轴

    this->m_p_mi_comm->SendPmcCmd(cmd);


    printf("chn manual move pmc: axis = %d, tar_pos = %lld\n", m_channel_status.cur_axis, tar_pos);
}


/**
 * @brief 获取当前手动单步步长设置
 * @return 当前手动步长的值，单位um
 */
int ChannelControl::GetCurManualStep(){
    int step = 0;
    switch(m_channel_status.manual_step){
    case MANUAL_STEP_1:
        step = 1;
        break;
    case MANUAL_STEP_10:
        step = 10;
        break;
    case MANUAL_STEP_100:
        step = 100;
        break;
    case MANUAL_STEP_1000:
        step = 1000;
        break;
    case MANUAL_STEP_INVALID:

        break;
    }
    return step;
}

/**
 * @brief 手动指令某个轴以指定速度运动到指定位置
 * @param axis : 通道轴号，指定需要移动的轴，从0开始计数
 * @param pos : 目标位置，单位mm
 * @param vel ： 目标速度，单位mm/min
 * @param workcoord_flag : 目标位置是否为工件坐标系下的坐标，0--机械坐标系   1--工件坐标系
 */
void ChannelControl::ManualMove(uint8_t axis, double pos, double vel, bool workcoord_flag){

    if(g_ptr_chn_engine->GetPmcActive(this->GetPhyAxis(axis))) {
        //this->ManualMovePmc(axis, pos, vel);
        m_error_code = ERR_PMC_IVALID_USED;
        CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        return;
    }

    if(workcoord_flag){//工件坐标系转换为机械坐标系
        this->TransWorkCoordToMachCoord(pos, this->m_channel_status.gmode[14], axis);
    }

    //设置目标位置
    int64_t tar_pos = pos*1e7;    //转换单位为0.1nm
    uint8_t phy_axis = this->GetPhyAxis(axis);
    int64_t cur_pos = GetAxisCurMachPos(axis)*1e7;  //当前位置
    ManualMoveDir dir = (tar_pos > cur_pos)?DIR_POSITIVE:DIR_NEGATIVE;  //移动方向

    //检查软限位
    double limit = 0;
    if(CheckSoftLimit(dir, phy_axis, GetAxisCurMachPos(axis))){
        printf("soft limit active, manual move abs return \n");
        return;
    }else if(GetSoftLimt(dir, phy_axis, limit) && dir == DIR_POSITIVE && pos > limit * 1e7){
        tar_pos = limit * 1e7;
    }else if(GetSoftLimt(dir, phy_axis, limit) && dir == DIR_NEGATIVE && pos < limit * 1e7){
        tar_pos = limit * 1e7;
    }

    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = axis+1;	//MC中轴号从1开始
    cmd.data.cmd = CMD_MC_AXIS_MAUAL_MOVE;

    //设置速度
    uint32_t feed = vel*1000/60;   //转换单位为um/s
    memcpy(cmd.data.data, &feed, sizeof(feed));
    memcpy(&cmd.data.data[2], &tar_pos, sizeof(tar_pos));

    //	if(workcoord_flag)
    //		cmd.data.data[6] = 1;   //绝对位置，工件坐标系

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
     //printf("send manual move cmd: axis=%d, pos = %lf, vel = %lf\n", axis, pos, vel);
}

/**
 * @brief 停止当前轴手动移动
 * @param 无
 */
void ChannelControl::ManualMoveStop(){
    if(m_channel_status.chn_work_mode == MANUAL_STEP_MODE){  //手动单步模式，不做响应，退出
        return;
    }

    m_channel_status.cur_manual_move_dir = DIR_STOP;

    if(g_ptr_chn_engine->GetPmcActive(this->GetPhyAxis(m_channel_status.cur_axis))) {
        //PMC轴的Stop动作没有屏蔽，如果屏蔽此处，担心某些特殊情况停不下来
        std::cout << "ChannelControl::ManualMoveStop() pmc axis " << (int)m_channel_status.cur_axis << std::endl;
        this->ManualMovePmcStop();
        return;
    }

    printf("send manualmove stop cmd to mc\n");
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_MULTI_AXIS_STOP;
    cmd.data.data[0] = (0x01<<m_channel_status.cur_axis);

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 停止轴的手动移动
 * @param axis_mask : 需要停止的轴mask
 */
void ChannelControl::ManualMoveStop(uint16_t axis_mask){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_MULTI_AXIS_STOP;
    cmd.data.data[0] = axis_mask;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 停止当前PMC轴手动移动
 */
void ChannelControl::ManualMovePmcStop(){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(MiCmdFrame));

    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = this->m_n_channel_index+1;
    cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
    cmd.data.data[0] = 0x20;   //停止当前轴运动，并抛弃当前运动指令

    m_p_mi_comm->WriteCmd(cmd);

    this->m_mask_run_pmc = 0;
    this->m_mask_runover_pmc = 0;
}

/**
 * @brief 停止PMC轴的手动移动
 * @param phy_axis : 物理轴号，从0开始
 */
void ChannelControl::ManualMovePmcStop(uint8_t phy_axis){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(MiCmdFrame));

    if(phy_axis != 0xFF)
        cmd.data.axis_index = phy_axis+1;
    else
        cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = this->m_n_channel_index+1;
    cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
    cmd.data.data[0] = 0x20;   //停止当前轴运动，并抛弃当前运动指令

    m_p_mi_comm->WriteCmd(cmd);

    if(phy_axis < 64){
        this->m_mask_run_pmc &= ~(0x01L<<phy_axis);
        this->m_mask_runover_pmc &= ~(0x01L<<phy_axis);
    }else{
        this->m_mask_run_pmc = 0;
        this->m_mask_runover_pmc = 0;
    }
}

/**
 * @brief 暂停PMC轴移动
 * @param phy_axis : 0~63 : 指定轴号     0xFF : 指定所有轴
 * @param flag  : true--暂停     false--取消暂停
 */
void ChannelControl::PausePmcAxis(uint8_t phy_axis, bool flag){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(MiCmdFrame));

    uint16_t data = 0x01;
    if(!flag)
        data = 0x10;   //取消暂停
    if(phy_axis != 0xFF)
        cmd.data.axis_index = phy_axis+1;
    else
        cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = this->m_n_channel_index+1;
    cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
    cmd.data.data[0] = data;   //暂停当前轴运动

    m_p_mi_comm->WriteCmd(cmd);

    printf("ChannelControl::PausePmcAxis, phy_axis=%hhu, flag = %hhu\n", phy_axis, flag);
}


/**
 * @brief 停止PMC轴移动
 * @param phy_axis : 0~63 : 指定轴号     0xFF : 指定所有轴
 */
void ChannelControl::StopPmcAxis(uint8_t phy_axis){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(MiCmdFrame));

    if(phy_axis != 0xFF)
        cmd.data.axis_index = phy_axis+1;
    else
        cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = this->m_n_channel_index+1;
    cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
    cmd.data.data[0] = 0x20;   //停止当前轴运动

    m_p_mi_comm->WriteCmd(cmd);

    if(phy_axis < 64){
        this->m_mask_run_pmc &= ~(0x01L<<phy_axis);
        this->m_mask_runover_pmc &= ~(0x01L<<phy_axis);
    }else{
        this->m_mask_run_pmc = 0;
        this->m_mask_runover_pmc = 0;
    }

    printf("ChannelControl::StopPmcAxis, phy_axis=%hhu\n", phy_axis);
}


/**
 * @brief 手动指令某个PMC轴以指定速度运动到指定位置
 * @param axis : 通道轴号，指定需要移动的轴，从0开始计数
 * @param pos : 目标位置，单位mm
 * @param vel ： 目标速度，单位mm/min
 */
void ChannelControl::ManualMovePmc(uint8_t axis, double pos, double vel){
    uint8_t phy_axis = this->GetPhyAxis(axis);

    PmcCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(PmcCmdFrame));

    //命令号  低字节:0--单轴指令   1--多轴G01指令  高字节：0--绝对坐标   1--增量坐标
    cmd.data.cmd = 0;

    //轴号    低字节：轴号[1-64]    高字节：所属通道号[1-4]
    cmd.data.axis_index = ((phy_axis+1)|((m_n_channel_index+1)<<8));

    cmd.data.data[0] = 0;   //定位移动，赋0


    //设置速度
    uint32_t feed = vel*1000/60;   //转换单位为um/s


    //设置目标位置
    int64_t tar_pos = pos*1e7;    //转换单位为0.1nm
    int64_t cur_pos = GetAxisCurMachPos(axis)*1e7;  //当前位置
    ManualMoveDir dir = (tar_pos > cur_pos)?DIR_POSITIVE:DIR_NEGATIVE;  //移动方向
    //检查软限位
    double limit = 0;
    if(CheckSoftLimit(dir, phy_axis, GetAxisCurMachPos(axis))){
        printf("soft limit active, manual move abs return \n");
        return;
    }else if(GetSoftLimt(dir, phy_axis, limit) && dir == DIR_POSITIVE && pos > limit * 1e7){
        tar_pos = limit * 1e7;
    }else if(GetSoftLimt(dir, phy_axis, limit) && dir == DIR_NEGATIVE && pos < limit * 1e7){
        tar_pos = limit * 1e7;
    }

    memcpy(&cmd.data.data[1], &tar_pos, sizeof(tar_pos));  //设置目标位置

    memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //设置速度

    this->m_mask_run_pmc = 0x01L<<phy_axis;  //设置当前运行轴

    this->m_p_mi_comm->SendPmcCmd(cmd);

    //	printf("send manual move cmd: axis=%d, pos = %lf, vel = %lf\n", axis, pos, vel);
}

bool ChannelControl::CheckSoftLimit(ManualMoveDir dir, uint8_t phy_axis, double pos){
    return m_p_channel_engine->CheckSoftLimit(dir,phy_axis,pos);
}

bool ChannelControl::GetSoftLimt(ManualMoveDir dir, uint8_t phy_axis, double &limit){
    return m_p_channel_engine->GetSoftLimt(dir,phy_axis,limit);
}

/**
 * @brief 设置MC单段模式
 * @param flag : true--设置单段有效   false--取消单段
 */
void ChannelControl::SetMcStepMode(bool flag){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_SET_STEP_MODE;
    cmd.data.data[0] = flag?1:0;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 向MC设置轴的工件坐标系原点
 * @param axis_index : 轴号索引，从0开始
 */
void ChannelControl::SetMcAxisOrigin(uint8_t axis_index){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = axis_index+1;
    cmd.data.cmd = CMD_MC_SET_AXIS_ORIGIN;

    int64_t origin_pos = m_p_chn_coord_config[0].offset[axis_index] * 1e7;  //基本工件坐标系
    int coord_index = m_channel_status.gmode[14];
    if(coord_index <= G59_CMD ){
        origin_pos += m_p_chn_coord_config[coord_index/10-53].offset[axis_index] * 1e7;    //单位由mm转换为0.1nm
    }else if(coord_index <= G5499_CMD){
        origin_pos += m_p_chn_ex_coord_config[coord_index/10-5401].offset[axis_index] * 1e7;    //单位由mm转换为0.1nm
    }

    printf("SetMcAxisOrigin, axis: %d value: %ld\n", axis_index, origin_pos);

    cmd.data.data[0] = origin_pos&0xFFFF;
    cmd.data.data[1] = (origin_pos>>16)&0xFFFF;
    cmd.data.data[2] = (origin_pos>>32)&0xFFFF;
    cmd.data.data[3] = (origin_pos>>48)&0xFFFF;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 向MC设置轴的工件坐标系原点
 * @param axis_index : 轴号索引，从0开始
 */
void ChannelControl::SetMcAxisOrigin(uint8_t axis_index, int64_t origin_pos){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = axis_index+1;
    cmd.data.cmd = CMD_MC_SET_AXIS_ORIGIN;

    cmd.data.data[0] = origin_pos&0xFFFF;
    cmd.data.data[1] = (origin_pos>>16)&0xFFFF;
    cmd.data.data[2] = (origin_pos>>32)&0xFFFF;
    cmd.data.data[3] = (origin_pos>>48)&0xFFFF;

    printf("SetMcAxisOrigin2, axis: %d value: %lld\n", axis_index, origin_pos);

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 激活设置的工件坐标系
 * @param active_flag : 激活标志，true--激活   false--失效
 */
void ChannelControl::ActiveMcOrigin(bool active_flag){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint16_t coord_index = m_channel_status.gmode[14]/10;   //模态存放时扩大了10倍，此处缩小10倍

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_ACTIVE_CHN_ORIGIN;
    cmd.data.data[0] = active_flag?coord_index:0;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 向MC设置各轴刀具偏置
 * @param axis_index : 轴号索引，从0开始
 */
void ChannelControl::SetMcAxisToolOffset(uint8_t axis_index){
    SCAxisConfig &axis_config = this->m_p_axis_config[this->GetPhyAxis(axis_index)];

    if(axis_config.axis_type != AXIS_LINEAR || m_channel_status.cur_h_code == 0 || m_channel_status.gmode[8] == G49_CMD)  //非直线轴或者当前刀偏为0
    {
    	return;
    }

    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = axis_index+1;
    cmd.data.cmd = CMD_MC_SET_AXIS_TOOL_OFFSET;



    int64_t offset = 0;
    if(axis_config.axis_linear_type == LINE_AXIS_Z){
        offset = m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2] * 1e7;  //单位由mm转换为0.1nm
        //	offset += m_p_chn_tool_config->geometry_comp_basic[2] * 1e7;   //基准刀偏
        offset += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1] * 1e7;   //叠加磨损补偿
    }
    else if(axis_config.axis_linear_type == LINE_AXIS_X || axis_config.axis_linear_type == LINE_AXIS_Y){
        offset = m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][axis_config.axis_linear_type-1] * 1e7;  //单位由mm转换为0.1nm
    }
    else //非XYZ轴没有刀具偏置
    {
    	printf("非XYZ轴没有刀具偏置");
    	return;
    }

    if(this->m_channel_status.gmode[8] == G44_CMD)	//负向补偿
        offset *= -1;

    cmd.data.data[0] = offset&0xFFFF;
    cmd.data.data[1] = (offset>>16)&0xFFFF;
    cmd.data.data[2] = (offset>>32)&0xFFFF;
    cmd.data.data[3] = (offset>>48)&0xFFFF;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
    printf("SetMcAxisToolOffset, axis = %hhu, offset = %lld\n", axis_index, offset);

}

/**
 * @brief 激活设置的刀具偏置
 * @param active_flag : 激活标志，true--激活   false--失效
 */
void ChannelControl::ActiveMcToolOffset(bool active_flag){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_ACTIVE_CHN_TOOL_OFFSET;
    cmd.data.data[0] = active_flag?m_channel_status.cur_h_code:0;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief MC是否需要发送启动命令
 * @return
 */
bool ChannelControl::IsMcNeedStart(){
    printf("need=%hhu, bufcount=%hu, autoblock=%hhu\n", m_b_mc_need_start, m_channel_mc_status.buf_data_count, m_channel_mc_status.auto_block_over);
    if(!m_b_mc_need_start)
        return false;

    if(m_channel_mc_status.buf_data_count == 0 && m_channel_mc_status.auto_block_over)  //MC没有可运行数据
        return false;

    return true;
}

/**
 * @brief 读取当前MC中运动数据的数量,自适应AUTO和MDA模式
 * @return mc中剩余运动数据量
 */
uint16_t ChannelControl::ReadMcMoveDataCount(){
    if(this->m_channel_status.chn_work_mode >= MDA_MODE
        #ifdef USES_ADDITIONAL_PROGRAM
            || this->m_n_add_prog_type == CONTINUE_START_ADD
        #endif
            ){
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadMdaBufDataCount(m_n_channel_index, m_channel_mc_status.mda_data_count);
        else
            this->m_p_mc_arm_comm->ReadMdaBufDataCount(m_n_channel_index, m_channel_mc_status.mda_data_count);
        return m_channel_mc_status.mda_data_count;
    }
    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);
    else
        m_p_mc_arm_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);
    return m_channel_mc_status.buf_data_count;
}

/**
 * @brief 当前分块到位标志是否有效
 * @return true--有效   false--无效
 */
bool ChannelControl::CheckBlockOverFlag(){

    struct timeval cur_time;
    gettimeofday(&cur_time, nullptr);
    unsigned int delay = (cur_time.tv_sec-m_time_start_mc.tv_sec)*1000000+cur_time.tv_usec-m_time_start_mc.tv_usec;  //单位微秒
    if(delay < MC_STATE_REFRESH_CYCLE) //发送MC启动命令5ms后才能确认到位标志
        return false;

    if(m_channel_status.chn_work_mode == AUTO_MODE
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type != CONTINUE_START_ADD
        #endif
            ){
        if(!this->m_b_mc_on_arm)
            m_channel_mc_status.auto_block_over = m_p_mc_comm->ReadAutoBlockRunOverFlag(m_n_channel_index);
        else
            m_channel_mc_status.auto_block_over = m_p_mc_arm_comm->ReadAutoBlockRunOverFlag(m_n_channel_index);
        return m_channel_mc_status.auto_block_over;
    }

    if(!this->m_b_mc_on_arm)
        m_channel_mc_status.mda_block_over = m_p_mc_comm->ReadMdaBlockRunOverFlag(m_n_channel_index);
    else
        m_channel_mc_status.mda_block_over = m_p_mc_arm_comm->ReadMdaBlockRunOverFlag(m_n_channel_index);
    return m_channel_mc_status.mda_block_over;
}

/**
 * @brief 当前单段到位标志是否有效
 * @return true--有效   false--无效
 */
bool ChannelControl::CheckStepOverFlag(){
    struct timeval cur_time;
    gettimeofday(&cur_time, nullptr);
    unsigned int delay = (cur_time.tv_sec-m_time_start_mc.tv_sec)*1000000+cur_time.tv_usec-m_time_start_mc.tv_usec;  //单位微秒
    if(delay < MC_STATE_REFRESH_CYCLE) //发送MC启动命令5ms后才能确认到位标志
        return false;

    if(!this->m_b_mc_on_arm)
        this->m_channel_mc_status.step_over = this->m_p_mc_comm->ReadStepRunOverFlag(m_n_channel_index);
    else
        this->m_channel_mc_status.step_over = this->m_p_mc_arm_comm->ReadStepRunOverFlag(m_n_channel_index);
    return m_channel_mc_status.step_over;
}

/**
 * @brief 更新当前通道模态数据
 * @param mode
 */
void ChannelControl::RefreshModeInfo(const McModeStatus &mode){
    bool b_change = false;
    if(this->m_mc_mode_cur.all != mode.all){
        this->m_channel_status.gmode[2] = mode.bits.mode_g17*10+G17_CMD;
        this->m_channel_status.gmode[3] = mode.bits.mode_g90*10+G90_CMD;
        this->m_channel_status.gmode[5] = mode.bits.mode_g94*10+G94_CMD;
        this->m_channel_status.gmode[6] = mode.bits.mode_g20*10+G20_CMD;
        this->m_channel_status.gmode[7] = mode.bits.mode_g40*10+G40_CMD;
        if(mode.bits.mode_g73 == 15)
            this->m_channel_status.gmode[9] = G89_CMD;
        else
            this->m_channel_status.gmode[9] = mode.bits.mode_g73*10+G73_CMD;
        this->m_channel_status.gmode[10] = mode.bits.mode_g98*10+G98_CMD;
        this->m_channel_status.gmode[11] = mode.bits.mode_g50*10+G50_CMD;
        this->m_channel_status.gmode[13] = mode.bits.mode_g60*10+G60_CMD;
        this->m_channel_status.gmode[16] = mode.bits.mode_g68*10+G68_CMD;
        //this->m_channel_status.cur_d_code = mode.bits.mode_d;//llx add 刀具半径D模态，不从MC读取,由SC直接设置

        this->m_mc_mode_cur.all = mode.all;

        b_change = true;
    }
    int mode_1 = m_channel_mc_status.cur_cmd*10;
    if(m_channel_status.gmode[1] != mode_1){
        //		printf("change 01group mode : old = %d, new = %d\n", m_channel_status.gmode[1], mode_1);
        //当前01组G模态
        m_channel_status.gmode[1] = mode_1;
        b_change = true;

    }

    if(b_change){
        this->SendChnStatusChangeCmdToHmi(G_MODE);
        //this->SendModeChangToHmi(D_MODE);//llx add 刀具半径D模态，不从MC读取,由SC直接设置
    }

}

/**
 * @brief 由SC直接更新模态数据
 * @param mode_type : 模态类型
 * @param value     : 更新值
 */
void ChannelControl::UpdateModeData(uint16_t mode_type, int value)
{
    switch(mode_type){
    case D_MODE:
        m_channel_status.cur_d_code = value;
        SendModeChangToHmi(D_MODE);
        break;
    default:
        break;
    }
}

/**
 * @brief 设置坐标系原点数据
 * @param flag : 激活工件坐标系标志， true - 激活   false - 失效
 */
void ChannelControl::SetMcCoord(bool flag){
    //	printf("set mc coord!!!!!\n");
    if(flag){
        for(int i = 0; i < m_p_channel_config->chn_axis_count; i++){

        	int64_t origin_pos = 0;

        	if(G92Active){
        		origin_pos += (int64_t)(m_p_chn_g92_offset->offset[i] * 1e7);
        	}else{
        		// ext offset
        		origin_pos += (int64_t)(m_p_chn_coord_config[0].offset[i] * 1e7);  //基本工件坐标系

				// g92 offset
				origin_pos += (int64_t)(m_p_chn_g92_offset->offset[i] * 1e7);

        		// g54xx offset
        		int coord_index = m_channel_status.gmode[14];
				if(coord_index <= G59_CMD ){
					origin_pos += (int64_t)(m_p_chn_coord_config[coord_index/10-53].offset[i] * 1e7);    //单位由mm转换为0.1nm
					printf("===== g54 offset  axis: %i offset %lld\n", i, (int64_t)(m_p_chn_coord_config[coord_index/10-53].offset[i] * 1e7));
				}else if(coord_index <= G5499_CMD){
					origin_pos += (int64_t)(m_p_chn_ex_coord_config[coord_index/10-5401].offset[i] * 1e7);    //单位由mm转换为0.1nm
					printf("===== g5401 offset  axis: %i offset %lld\n", i, (int64_t)(m_p_chn_ex_coord_config[coord_index/10-5401].offset[i] * 1e7));
				}
        	}

        	this->SetMcAxisOrigin(i, origin_pos);

        	/*
        	//this->SetMcAxisOrigin(i);
            printf("=============================axis %d ======================\n", i);
            int64_t origin_pos = 0;
            if(!G92Active) origin_pos = (int64_t)(m_p_chn_coord_config[0].offset[i] * 1e7);  //基本工件坐标系

        	printf("===== basic offset  axis: %i offset %lld\n", i, origin_pos);
            int coord_index = m_channel_status.gmode[14];
            if(coord_index <= G59_CMD ){
            	origin_pos += (int64_t)(m_p_chn_coord_config[coord_index/10-53].offset[i] * 1e7);    //单位由mm转换为0.1nm
            	printf("===== g54 offset  axis: %i offset %lld\n", i, (int64_t)(m_p_chn_coord_config[coord_index/10-53].offset[i] * 1e7));
            }else if(coord_index <= G5499_CMD){
                origin_pos += (int64_t)(m_p_chn_ex_coord_config[coord_index/10-5401].offset[i] * 1e7);    //单位由mm转换为0.1nm
                printf("===== g5401 offset  axis: %i offset %lld\n", i, (int64_t)(m_p_chn_ex_coord_config[coord_index/10-5401].offset[i] * 1e7));
            }

            origin_pos += (int64_t)(m_p_chn_g92_offset->offset[i] * 1e7);
            printf("===== g92 offset  axis: %i offset %lld\n", i, (int64_t)(m_p_chn_g92_offset->offset[i] * 1e7));

            printf("===== coord_index: %d SetMcCoord axis: %d origin: %lld\n", coord_index, i, origin_pos);

            this->SetMcAxisOrigin(i, origin_pos);*/
        }
    }

    this->ActiveMcOrigin(flag);
}

/**
 * @brief 设置MC的刀具偏置
 * @param flag : 激活刀具偏置标志， true - 激活   false - 失效
 */
void ChannelControl::SetMcToolOffset(bool flag){
    if(flag){
        for(int i = 0; i < m_p_channel_config->chn_axis_count; i++){
            if(m_p_axis_config[this->GetPhyAxis(i)].axis_type == AXIS_LINEAR)
                this->SetMcAxisToolOffset(i);
        }
    }

    this->ActiveMcToolOffset(flag);
    printf("setMcToolOffset, H = %hhu\n", this->m_channel_status.cur_h_code);
}

/**
 * @brief 设置通道轴名称
 */
void ChannelControl::SetChnAxisName(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;

    cmd.data.cmd = CMD_MC_SET_CHN_AXIS_NAME;

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        cmd.data.axis_index = i+1;
        cmd.data.data[0] = this->m_p_channel_config->chn_axis_name[i];

        if(this->m_p_general_config->axis_name_ex){  //允许扩展轴下标
            cmd.data.data[1] = this->m_p_channel_config->chn_axis_name_ex[i];
        }

        if(!this->m_b_mc_on_arm)
            m_p_mc_comm->WriteCmd(cmd);
        else
            m_p_mc_arm_comm->WriteCmd(cmd);
        //	printf("set chn[%hhu] axis[%hhu] name[%hhu] \n", m_n_channel_index, i+1, m_p_channel_config->chn_axis_name[i]);
    }
}

/**
 * @brief 设置轴使能，以及插补模式（1--NC轴插补   2--PMC轴插补）
 * @param index : 轴号索引，从0开始
 */
void ChannelControl::SetChnAxisOn(uint8_t chn_axis){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint8_t phy_axis = this->GetPhyAxis(chn_axis);

    cmd.data.axis_index = chn_axis+1;
    cmd.data.channel_index = m_n_channel_index;   // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SET_AXIS_ON;

    //轴使能  0--禁止该轴   1--使能该轴
    SCAxisConfig &axis_config = this->m_p_axis_config[phy_axis];
    if(axis_config.axis_type != AXIS_SPINDLE)
        cmd.data.data[0] = 1;
    else
        cmd.data.data[0] = 0;


    //轴插补类型   1--NC轴插补（自动、手动、MDI）  2--PMC轴插补   其它值无效  （注意10MC的DSP暂不支持 纯粹PMC轴）
    cmd.data.data[1] = 1;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 设置指定轴的螺距及最大速度等基本信息
 * @param index : 轴索引号, 从0开始
 */
void ChannelControl::SetChnAxisBaseParam(uint8_t chn_axis){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint8_t phy_axis = this->GetPhyAxis(chn_axis);

    cmd.data.axis_index = chn_axis+1;
    cmd.data.channel_index = m_n_channel_index;  // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SET_AXIS_BASE_INFO;

    SCAxisConfig &axis_config = this->m_p_axis_config[phy_axis];
    //轴类型   1--直线轴   2--旋转轴   3--主轴
    cmd.data.data[0] = axis_config.axis_type+1;

    //螺距，单位：1um
    uint32_t data = axis_config.move_pr * 1e3;  //mm->1um
    cmd.data.data[1] = (data&0xFFFF);
    cmd.data.data[2] = ((data>>16)&0xFFFF);

    //最大速度限制， 单位：um/s
    data = axis_config.move_pr * axis_config.motor_speed_max * 1000 /60;   //mm/min -> um/s
    cmd.data.data[3] = (data&0xFFFF);
    cmd.data.data[4] = ((data>>16)&0xFFFF);

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);

}

/**
 * @brief 设置指定轴的速度相关信息
 * @param index : 轴索引号, 从0开始
 */
void ChannelControl::SetChnAxisSpeedParam(uint8_t chn_axis){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint8_t phy_axis = this->GetPhyAxis(chn_axis);

    cmd.data.axis_index = chn_axis+1;
    cmd.data.channel_index = m_n_channel_index;  // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SET_AXIS_SPEED_LIMIT;

    SCAxisConfig &axis_config = this->m_p_axis_config[phy_axis];
    //G00速度，单位：um/s
    uint32_t data = axis_config.rapid_speed * 1000 / 60;  //mm/min -> um/s
    //	printf("set axis %hhu g00 speed: %u\n", index, data);
    cmd.data.data[0] = (data&0xFFFF);
    cmd.data.data[1] = ((data>>16)&0xFFFF);
    //	printf("set axis %hhu g00 speed: 0x%hx,  0x%hx\n", index, cmd.data.data[0], cmd.data.data[1]);

    //转角速度差限制， 单位：um/s
    data = axis_config.corner_acc_limit * 1000 /60;   //mm/min -> um/s
    cmd.data.data[2] = (data&0xFFFF);
    cmd.data.data[3] = ((data>>16)&0xFFFF);

    //手动过渡速度，单位：um/s  //TODO 此参数后续将启用，此处传输一个固定值100mm/min
    data = 100 * 1000 / 60;    //mm/min -> um/s
    cmd.data.data[4] = (data&0xFFFF);
    cmd.data.data[5] = ((data>>16)&0xFFFF);

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 设置指定轴加速度相关信息
 * @param index : 轴索引号, 从0开始
 */
void ChannelControl::SetChnAxisAccParam(uint8_t chn_axis){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint8_t phy_axis = this->GetPhyAxis(chn_axis);

    cmd.data.axis_index = chn_axis+1;
    cmd.data.channel_index = m_n_channel_index;  // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SET_AXIS_ACC;
    //	printf("set axis[%hhu] acc:rap = %lf, start = %lf\n", index, m_p_axis_config[index].rapid_acc,
    //			m_p_axis_config[index].start_acc);

    //G00加速度，单位：10mm/s^2
    uint16_t data = m_p_axis_config[phy_axis].rapid_acc / 10;
    cmd.data.data[0] = data;

    //手动加速度， 单位：10mm/s^2
    data = m_p_axis_config[phy_axis].manual_acc /10;
    cmd.data.data[1] = data;

    //手动过渡加速度，单位：10mm/s^2
    data = m_p_axis_config[phy_axis].start_acc /10;
    cmd.data.data[2] = data;

    //G00 S型时间常数
    cmd.data.data[3] = m_p_axis_config[phy_axis].rapid_s_plan_filter_time;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 设置指定轴的软限位开关
 * @param axis : 指定轴号，从0开始
 */
void ChannelControl::SetChnAxisSoftLimit(uint8_t chn_axis){
    //发送MC轴软限位数据
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint8_t phy_axis = this->GetPhyAxis(chn_axis);

    uint16_t tmp = 0;

    SCAxisConfig &axis_config = this->m_p_axis_config[phy_axis];
    if(axis_config.soft_limit_check_1)
        tmp |= 0x01;
    if(axis_config.soft_limit_check_2)
        tmp |= 0x02;
    if(axis_config.soft_limit_check_3)
        tmp |= 0x04;


    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = chn_axis+1;
    cmd.data.cmd = CMD_MC_AXIS_ENABLE_SOFT_LIMIT;


    cmd.data.data[0] = tmp;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}
/**
 * @brief 强制关闭指定轴的软限位开关
 * @param chn_axis : 指定轴号，从0开始
 */
void ChannelControl::CloseChnAxisSoftLimit(uint8_t chn_axis){
    //发送MC轴软限位数据
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = chn_axis+1;
    cmd.data.cmd = CMD_MC_AXIS_ENABLE_SOFT_LIMIT;


    cmd.data.data[0] = 0;

    m_p_mc_comm->WriteCmd(cmd);
}


/**
 * @brief 设置指定轴的位置指令门限值
 * @param chn_axis : 指定通道轴号，从0开始
 */
void ChannelControl::SetChnAxisPosThreshold(uint8_t chn_axis){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint8_t phy_axis = this->GetPhyAxis(chn_axis);

    uint16_t tmp = 0;

    SCAxisConfig &axis_config = this->m_p_axis_config[phy_axis];

    tmp = axis_config.axis_home_pos[9]*1000;


    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = chn_axis+1;
    cmd.data.cmd = CMD_MC_AXIS_POS_THRESHOLD;

    cmd.data.data[0] = tmp;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);

}

/**
 * @brief 设置指定轴的软限位值
 * @param axis : 指定轴号，从0开始
 * @param index : 软限位组号，总共3组，0-2
 */
void ChannelControl::SetChnAxisSoftLimitValue(uint8_t chn_axis, uint8_t index){
    //发送MC轴软限位数据
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint8_t phy_axis = this->GetPhyAxis(chn_axis);

    uint16_t tmp = index;

    double max = 0, min = 0;
    if(index == 0){
        max = this->m_p_axis_config[phy_axis].soft_limit_max_1;
        min = this->m_p_axis_config[phy_axis].soft_limit_min_1;
    }else if(index ==1){
        max = this->m_p_axis_config[phy_axis].soft_limit_max_2;
        min = this->m_p_axis_config[phy_axis].soft_limit_min_2;
    }else if(index ==2){
        max = this->m_p_axis_config[phy_axis].soft_limit_max_3;
        min = this->m_p_axis_config[phy_axis].soft_limit_min_3;
    }

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = chn_axis+1;
    cmd.data.cmd = CMD_MC_SET_AXIS_SOFT_LIMIT;


    cmd.data.data[0] = tmp;


    int32_t softlimit = max*1e3;   //单位由mm转换为1um
    cmd.data.data[1] = (softlimit & 0xFFFF);
    cmd.data.data[2] = ((softlimit>>16) & 0xFFFF);

    softlimit = min*1e3;   //单位由mm转换为1um
    cmd.data.data[3] = (softlimit & 0xFFFF);
    cmd.data.data[4] = ((softlimit>>16) & 0xFFFF);

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 设置本通道内所有轴的参数
 * @param axis : 指定轴号，从0开始
 * @param index :
 */
void ChannelControl::SetChnAllAxisParam(void){

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        this->SetChnAxisOn(i);
        this->SetChnAxisBaseParam(i);
        this->SetChnAxisSpeedParam(i);
        this->SetChnAxisAccParam(i);

        this->SetChnAxisSoftLimitValue(i, 0);
        this->SetChnAxisSoftLimitValue(i, 1);
        this->SetChnAxisSoftLimitValue(i, 2);
        this->SetChnAxisSoftLimit(i);
        this->SetChnAxisPosThreshold(i);
        //	printf("set chn[%hhu] axis[%hhu] name[%hhu] \n", m_n_channel_index, i+1, m_p_channel_config->chn_axis_name[i]);
    }
}


/**
 * @brief 设置加工速度规划方式
 */
void ChannelControl::SetMcChnPlanMode(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;

    cmd.data.cmd = CMD_MC_SET_CHN_PLAN_MODE;
    cmd.data.data[0] = this->m_p_channel_config->rapid_plan_mode;
    cmd.data.data[1] = this->m_p_channel_config->cut_plan_mode;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);

}

/**
 * @brief 设置通道加工速度规划参数
 */
void ChannelControl::SetMcChnPlanParam(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;

    cmd.data.cmd = CMD_MC_SET_CHN_PLAN_PARAM;

    uint32_t data = m_p_channel_config->chn_max_vel*1000/60;  //单位转换   mm/min-->um/s
    cmd.data.data[0] = data&0xFFFF;
    cmd.data.data[1] = (data>>16)&0xFFFF;

    uint16_t tmp = m_p_channel_config->chn_max_acc/10;    //单位转换  mm/s^2-->10mm/s^2
    cmd.data.data[2] = tmp;
    tmp = m_p_channel_config->chn_max_dec/10;    //单位转换  mm/s^2-->10mm/s^2
    cmd.data.data[3] = tmp;

    cmd.data.data[4] = m_p_channel_config->chn_s_cut_filter_time;

    tmp = m_p_channel_config->chn_max_arc_acc/10;    //单位转换  mm/s^2-->10mm/s^2
    cmd.data.data[5] = tmp;

    tmp = m_p_channel_config->chn_max_corner_acc/10;    //单位转换  mm/s^2-->10mm/s^2
    cmd.data.data[6] = tmp;


    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 设置刚性攻丝加工规划参数
 */
void ChannelControl::SetMcTapPlanParam(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;

    cmd.data.cmd = CMD_MC_SET_TAP_PLAN_PARAM;

    uint16_t tmp = m_p_channel_config->tap_max_acc/10;    //单位转换  mm/s^2-->10mm/s^2
    cmd.data.data[0] = tmp;
    tmp = m_p_channel_config->tap_max_dec/10;    //单位转换  mm/s^2-->10mm/s^2
    cmd.data.data[1] = tmp;

    cmd.data.data[2] = m_p_channel_config->tap_plan_mode;

    tmp = m_p_channel_config->tap_s_cut_filter_time;
    cmd.data.data[3] = tmp;


    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 设置通道加工速度功能开关
 */
void ChannelControl::SetMcChnPlanFun(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;

    cmd.data.cmd = CMD_MC_SET_CHN_PLAN_SWITCH;
    cmd.data.data[0] = this->m_p_channel_config->chn_spd_limit_on_axis;
    cmd.data.data[1] = this->m_p_channel_config->chn_spd_limit_on_acc;
    cmd.data.data[2] = this->m_p_channel_config->chn_spd_limit_on_curvity;
    cmd.data.data[3] = this->m_p_channel_config->chn_rapid_overlap_level;
    cmd.data.data[4] = this->m_p_channel_config->intep_mode;
    cmd.data.data[5] = this->m_p_channel_config->chn_small_line_time;
    cmd.data.data[6] = this->m_p_channel_config->chn_look_ahead;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 设置拐角准停参数
 */
void ChannelControl::SetMcChnCornerStopParam(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;

    cmd.data.cmd = CMD_MC_SET_CHN_CORNER_STOP_PARAM;
    cmd.data.data[0] = this->m_p_channel_config->corner_stop_enable;
    uint32_t data = m_p_channel_config->corner_stop_angle_min;
    data *= 1000;   //0.001度
    cmd.data.data[1] = data&0xFFFF;
    cmd.data.data[2] = (data>>16)&0xFFFF;
    data =m_p_channel_config->corner_stop;  //0.001度
    data *= 1000;  //0.001度
    cmd.data.data[3] = data&0xFFFF;
    cmd.data.data[4] = (data>>16)&0xFFFF;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}


/**
 * @brief 设置MC的调试参数
 * @param index : 调试参数号
 */
#ifdef USES_WOOD_MACHINE
//设置MC的挑角补偿值
void ChannelControl::SetMcFlipCompParam(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;

    cmd.data.cmd = CMD_MC_SET_FLIP_COMP_PARAM;

    int value = this->m_p_channel_config->flip_comp_value;
    cmd.data.data[0] = value&0xFFFF;
    cmd.data.data[1] = (value>>16)&0xFFFF;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

void ChannelControl::SetMcDebugParam(uint16_t index){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;

    cmd.data.cmd = CMD_MC_SET_CHN_DEBUG_PARAM;
    cmd.data.data[0] = index;

    int data = 0;
    switch(index){
    case 0:
        data = m_p_channel_config->debug_param_1;
        break;
    case 1:
        data = m_p_channel_config->debug_param_2;
        break;
    case 2:
        data = m_p_channel_config->debug_param_3;
        break;
    case 3:
        data = m_p_channel_config->debug_param_4;
        break;
    case 4:
        data = m_p_channel_config->debug_param_5;
        break;
    default:
        return;
    }

    cmd.data.data[1] = data&0xFFFF;
    cmd.data.data[2] = (data>>16)&0xFFFF;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);

    //	printf("ChannelControl::SetMcDebugParam, chn_idx=%hhu, index=%hu, data=%d\n", m_n_channel_index, index, data);
}
#endif


/**
 * @brief 给MC发送系统复位指令
 */
void ChannelControl::SendMcSysResetCmd(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = 0xFF;
    cmd.data.cmd = CMD_MC_SYS_RESET;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 获取单轴的当前工件坐标系位置
 * @param axis_index : 通道轴号，从0开始
 * @return
 */
double ChannelControl::GetAxisCurWorkPos(uint8_t axis_index){
    double pos = 0.0;
    if(axis_index >= this->m_p_channel_config->chn_axis_count)
        return pos;

    pos = this->m_channel_rt_status.cur_pos_work.GetAxisValue(axis_index);

    return pos;
}

/**
 * @brief 获取单轴的当前机械坐标系位置
 * @param axis_index : 通道轴号，从0开始
 * @return
 */
double ChannelControl::GetAxisCurMachPos(uint8_t axis_index){
    double pos = 0.0;
    if(axis_index >= this->m_p_channel_config->chn_axis_count)
        return pos;
    uint8_t phy_axis = this->GetPhyAxis(axis_index);
    if(phy_axis == 0xFF)
        return pos;

    if (g_ptr_chn_engine->GetPmcActive(phy_axis)){
        return this->m_p_channel_engine->GetPhyAxisMachPosFeedback(phy_axis);
    }

    double *pp = m_channel_rt_status.cur_pos_machine.m_df_point;
    pos = pp[axis_index];
    return pos;
}

/**
 * @brief 获取单轴的当前运行速度
 * @param axis_index : 通道轴号，从0开始
 * @return
 */
double ChannelControl::GetAxisCurFedBckAxisSpeed(uint8_t axis_index){
    double val = 0.0;
    if(axis_index >= this->m_p_channel_config->chn_axis_count)
        return val;

    return m_channel_rt_status.cur_feedbck_torque.GetAxisValue(axis_index);
}

/**
 * @brief 获取单轴的当前运行力矩
 * @param axis_index : 通道轴号，从0开始
 * @return
 */
double ChannelControl::GetAxisCurFedBckAxisTorque(uint8_t axis_index){
    double val = 0.0;
    if(axis_index >= this->m_p_channel_config->chn_axis_count)
        return val;

    return m_channel_rt_status.cur_feedbck_torque.GetAxisValue(axis_index);
}

/**
 * @brief 获取单轴的当前插补目标位置
 * @param axis_index : 通道轴号，从0开始
 * @param bMachCoord : true--机械坐标系下的目标位置    false--工件坐标系下的目标位置
 * @return 指定轴的当前插补目标位置
 */
double ChannelControl::GetAxisCurIntpTarPos(uint8_t axis_index, bool bMachCoord){
    double pos = 0.0;
    if(axis_index >= this->m_p_channel_config->chn_axis_count)
        return pos;

    DPointChn  target = m_channel_rt_status.tar_pos_work;
    if(bMachCoord)
        this->TransWorkCoordToMachCoord(target, this->m_channel_status.gmode[14],0x01<<axis_index );

    pos = target.GetAxisValue(axis_index);
    return pos;
}

/**
 * @brief 向MC更新倍率信息
 */
void ChannelControl::SetMcRatio(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

	cmd.data.channel_index = m_n_channel_index;
	cmd.data.axis_index = NO_AXIS;
	cmd.data.cmd = CMD_MC_SET_CHN_RATIO;
	cmd.data.data[0] = this->m_channel_status.rapid_ratio;
	cmd.data.data[1] = this->m_channel_status.auto_ratio;
	cmd.data.data[2] = this->m_channel_status.manual_ratio;

//    ScPrintf("rapid: %d auto: %d manual: %d\n",
//			this->m_channel_status.rapid_ratio,
//			this->m_channel_status.auto_ratio,
//			this->m_channel_status.manual_ratio);

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

#ifdef USES_WOOD_MACHINE
/**
 * @brief 向MC设置当前刀具寿命,    木工机刀具寿命以切削长度计算
 */
void ChannelControl::SetMcToolLife(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));
    int life = this->GetCurToolLife();

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_SET_CHN_TOOL_LIFE;
    cmd.data.data[0] = life&0xFFFF;
    cmd.data.data[1] = ((life>>16)&0xFFFF);

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);

}

/**
 * @brief 执行主轴预启动流程
 * @param line ： 当前指令行
 */
void ChannelControl::PreExecSpindleCmd(uint64_t line){
    static bool check_time = true;   //是否进行延时检查，防止MC的当前行号更新不及时，造成主轴预启动误触发

    if(this->m_b_prestart_spd && this->m_n_spd_prestart_step == 0xFF){
        //预启动已执行完成
        return;
    }

#ifdef USES_ADDITIONAL_PROGRAM
    if(this->m_n_add_prog_type != NONE_ADD)   //附加程序不执行主轴预启动
        return;
#endif

    if(!m_b_mc_need_start && check_time){
        struct timeval cur_time;
        gettimeofday(&cur_time, nullptr);
        unsigned int delay = (cur_time.tv_sec-m_time_start_mc.tv_sec)*1000000+cur_time.tv_usec-m_time_start_mc.tv_usec;  //单位微秒
        if(delay < MC_STATE_REFRESH_CYCLE) //发送MC启动命令5ms后才能确认到位标志
            return ;
        else
            check_time = false;
    }else if(m_b_mc_need_start){
        check_time = true;
    }

    if(!this->m_b_prestart_spd){//查找是否有预启动指令
        if(this->m_n_hw_trace_state != NONE_TRACE || this->IsStepMode() || this->m_channel_status.chn_work_mode == MDA_MODE)
            return;   //MDA模式，手轮跟踪或者单步模式下不进行主轴预启动

        double td = 0;
        int forward_line = 0;    //主轴预启动提前行数
        if(this->GetMacroVar(810, td) == 0)
            forward_line = td;

        if(!this->m_p_compiler->FindPreStartSpdCmd(line, line+forward_line, this->m_spd_prestart_offset)){
            //			printf("ChannelControl::PreExecSpindleCmd not fine: %llu, forward=%d\n", line, forward_line);

            return;  //没有找到预启动主轴命令
        }else{
            this->m_b_prestart_spd = true;
            this->m_n_spd_prestart_step = 0;
            printf("ChannelControl::PreExecSpindleCmd FIND SPD CMD[%lld: %lld]\n", line, this->m_spd_prestart_offset.line_no);
        }
    }
    //	printf("ChannelControl::PreExecSpindleCmd : %llu\n", line);

    //按步骤执行主轴启动动作
    int m_code = m_spd_prestart_offset.m_code;
    struct timeval time_now;
    unsigned int time_elpase = 0;
    if(this->m_n_spd_prestart_step == 0){
        //将代码发送给PMC
        this->SendMCodeToPmc(this->m_spd_prestart_offset.m_code, 0);

        gettimeofday(&m_time_m_start[0], NULL);   //

        this->m_channel_status.spindle_dir = (m_code==3)?SPD_DIR_POSITIVE:SPD_DIR_NEGATIVE;  //修改主轴旋转方向

        m_n_spd_prestart_step++;
    }else if(m_n_spd_prestart_step == 1){
        //等待TMF延时，置位MF选通信号
        gettimeofday(&time_now, NULL);
        time_elpase = (time_now.tv_sec-m_time_m_start[0].tv_sec)*1000000+
                time_now.tv_usec-m_time_m_start[0].tv_usec;
        if(time_elpase < 16000)
            return;		//未到延时时间

        this->SetMFSig(0, true);    //置位选通信号

        gettimeofday(&m_time_m_start[0], NULL);
        m_n_spd_prestart_step++;
    }else if(m_n_spd_prestart_step == 2){
        //等待FIN信号
        if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(0))
            gettimeofday(&m_time_m_start[0], NULL);   //开始计时
        else{
            gettimeofday(&time_now, NULL);
            time_elpase = (time_now.tv_sec-m_time_m_start[0].tv_sec)*1000000+
                    time_now.tv_usec-m_time_m_start[0].tv_usec;
            if(time_elpase > kMCodeTimeout && !this->GetMExcSig(0)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, m_code, m_n_channel_index);
                this->m_error_code = ERR_M_CODE;
                printf("NNNNOT support m code!!!!!!![%u]\n", time_elpase);
            }else
                return;
        }

        m_n_spd_prestart_step++;
    }else if(m_n_spd_prestart_step == 3){
        if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(0)){
            m_n_spd_prestart_step = 2;
            return;
        }

        //等待TFIN延时，复位MF信号
        gettimeofday(&time_now, NULL);
        time_elpase = (time_now.tv_sec-m_time_m_start[0].tv_sec)*1000000+
                time_now.tv_usec-m_time_m_start[0].tv_usec;
        if(time_elpase < 16000)
            return;		//未到延时时间

        this->SetMFSig(0, false);    //复位选通信号
        m_n_spd_prestart_step++;
    }else if(m_n_spd_prestart_step == 4){
        //等待FIN信号复位
        if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(0))
            return;

        //复位辅助指令信号和DEN信号
        this->SendMCodeToPmc(0, 0);

        if(this->m_n_spindle_count > 0){
            m_channel_status.rated_spindle_speed = m_spd_prestart_offset.s_code;
            this->SpindleSpeedDaOut(m_channel_status.rated_spindle_speed);
            this->SendModeChangToHmi(S_MODE);
        }
        this->m_p_f_reg->SPS = 1;    //标志主轴正转

        m_n_spd_prestart_step = 0xFF;    //置位结束状态
    }

}


/**
 * @brief 实际执行辅助指令消息， 木工项目定制，处理主轴预启动相关
 * @param msg : 指令消息
 * @return true--成功  false--失败，PL中缓冲已满或者等待运行到位
 */
bool ChannelControl::ExecuteAuxMsg_wood(RecordMsg *msg){
    AuxMsg *tmp = (AuxMsg *)msg;
    uint8_t m_count = tmp->GetMCount();   //一行中M代码总数
    uint8_t m_index = 0;
    uint8_t m_index_add = 0;   //如果正在执行主轴预启动，追加的序号
    int mcode = 0;
    bool bHasSpdCmd = false;   //是否有M03、M04指令


    if(this->m_n_restart_mode != NOT_RESTART &&
            tmp->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //非附加程序运行状态
        #endif
            ){//加工复位
        for(m_index = 0; m_index < m_count; m_index++){
            mcode = tmp->GetMCode(m_index);

            if(mcode == 6){//换刀
                this->m_mode_restart.cur_tool = this->m_mode_restart.cur_tcode;
            }else if(mcode == 3){
                this->m_mode_restart.rated_spindle_speed = this->m_mode_restart.cur_scode;
                this->m_mode_restart.spindle_dir = SPD_DIR_POSITIVE;
            }else if(mcode == 4){
                this->m_mode_restart.rated_spindle_speed = this->m_mode_restart.cur_scode;
                this->m_mode_restart.spindle_dir = SPD_DIR_NEGATIVE;
            }else if(mcode == 5){
                this->m_mode_restart.rated_spindle_speed = 0;
                this->m_mode_restart.spindle_dir = SPD_DIR_STOP;
            }else if(mcode == 2 || mcode == 30 || mcode == 99){  //直接结束，切换到READY状态
                if(mcode == 99 && m_mode_restart.sub_prog_call > 0){
                    this->m_mode_restart.sub_prog_call--;
                    printf("reset M30, sub_call = %hhu\n", m_mode_restart.sub_prog_call);
                }
                else{
                    this->SetCurLineNo(1);
                    this->m_n_run_thread_state = STOP;
                    CompileOver();

                    m_b_ret_from_macroprog = false;
                    m_b_init_compiler_pos = false;  //编译器初始位置需要重新初始化
                    this->m_p_compiler->Reset();

                    m_p_output_msg_list->Clear();

                    this->m_n_restart_mode = NOT_RESTART;
                    this->m_n_restart_line = 0;
                    this->m_n_restart_step = 0;
                    this->m_p_compiler->SetRestartPara(0, m_n_restart_mode);   //复位编译器的加工复位标志

                    this->ClearHmiInfoMsg();   //清除HMI的提示信息

                    this->m_p_f_reg->SPL = 0;
                    this->m_p_f_reg->STL = 0;
                }

            }
        }

        return true;
    }

    if(this->m_simulate_mode != SIM_NONE){  //仿真模式
        //设置当前行号
        SetCurLineNo(msg->GetLineNo());

        mcode = tmp->GetMCode(m_index);
        if(mcode == 2 || mcode == 30){
            ResetMcLineNo();//复位MC模块当前行号
            this->SetCurLineNo(1);

            this->m_p_f_reg->STL = 0;
            this->m_p_f_reg->SPL = 0;
            this->m_p_f_reg->OP = 0;

            CompileOver();
#ifdef 	USES_SIMULATION_TEST
            if(this->m_file_sim_data > 0){
                close(this->m_file_sim_data);
                m_file_sim_data = -1;
                g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "关闭仿真数据文件");
            }
#endif
        }else if(mcode == 99){
            if(m_n_subprog_count > 0){
                m_n_subprog_count--;
                m_b_ret_from_macroprog = false;

                this->m_n_run_thread_state = RUN;
            }
            else{
                ResetMcLineNo();//复位MC模块当前行号
                this->SetCurLineNo(1);

                this->m_p_f_reg->STL = 0;
                this->m_p_f_reg->SPL = 0;
                this->m_p_f_reg->OP = 0;

                this->m_n_run_thread_state = STOP;
                CompileOver();
#ifdef 	USES_SIMULATION_TEST
                if(this->m_file_sim_data > 0){
                    close(this->m_file_sim_data);
                    m_file_sim_data = -1;
                    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "关闭仿真数据文件");
                }
#endif
            }
        }
        return true;
    }

    if(tmp->IsFirstExec()){
        int limit = 3;
        if(this->IsStepMode())
            limit = 5;	//单步模式需要多次验证，因为状态切换有延时

        //等待MC分块的插补到位信号，以及MI的运行到位信号
        int count = 0;
        bool block_over = false;
        while(1){
            block_over = CheckBlockOverFlag();
            if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){ //未达到执行条件
                //		printf("aux exec return: 0x%x, count=%d, block_over=%hhu\n", m_p_mc_comm->ReadRunOverValue(), ReadMcMoveDataCount(), block_over);
                return false;    //还未运行到位
            }
            else if(++count < limit){
                usleep(5000);   //等待5ms，因为MC状态更新周期为5ms，需要等待状态确认
                printf("execute aus msg: blockflag=%d, count = %d\n",  block_over, count);

            }else
                break;
        }

        //		printf("isstepmode = %hhu, change_to_pause_flag = %hhu\n", this->IsStepMode(), this->m_b_need_change_to_pause);
        if(this->IsStepMode() && this->m_b_need_change_to_pause){//单段，切换暂停状态
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }


        //设置当前行号
        if(tmp->GetMCode(0) != 99 && !m_b_ret_from_macroprog)
            SetCurLineNo(msg->GetLineNo());
    }

    this->PreExecSpindleCmd(this->m_channel_rt_status.line_no);   //执行主轴预启动

    if(this->m_b_prestart_spd){ //有主轴预启动指令，设置增量序号
        if(m_count >= kMaxMCodeInLine && this->m_n_spd_prestart_step != 0xFF){
            //当前行M指令达到最数量，如果预启动未执行完则等待，执行完则结束
            return false;
        }else if(m_count < kMaxMCodeInLine){
            m_index_add = 1;
        }else{  //当前M指令达到最大数量，且主轴预启动执行结束，则复位主轴预启动标志
            m_b_prestart_spd = false;
            m_n_spd_prestart_step = 0;
            printf("spindle prestart over####1\n");
        }
    }


    bool bRet = true;
    struct timeval time_now;
    unsigned int time_elpase = 0;
    //	uint64_t mask = 0;

    for(m_index = 0; m_index < m_count; m_index++){
        if(tmp->GetExecStep(m_index) == 0xFF)
            continue;       //已执行完成则直接跳过

        mcode = tmp->GetMCode(m_index);

        switch(mcode){
        case 30:  	//M30
        case 2:		//M02
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M30\n");
                //TODO 将代码发送给PMC
                this->SendMCodeToPmc(mcode, m_index+m_index_add);
                if(mcode == 30)
                    this->m_p_f_reg->DM30 = 1;  //置位DM30
                else
                    this->m_p_f_reg->DM02 = 1;  //置位DM02

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //


                //五轴机床需要对无限旋转轴清整数圈
                //#ifdef USES_FIVE_AXIS_FUNC
                //			if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS){
                //				//向MI发送清整数圈位置指令
                //				m_n_mask_clear_pos = 0;
                //
                //				for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
                //					if(((m_mask_5axis_rot_nolimit>>i) & 0x01) == 0){
                //
                //						continue;
                //					}
                //					this->SendMiClearPosCmd(GetPhyAxis(i)+1, 360*1000);
                //				}
                //			}
                //#endif

                tmp->IncreaseExecStep(m_index);

            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index+m_index_add, true);    //置位选通信号

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }


                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index+m_index_add)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                //#ifdef USES_FIVE_AXIS_FUNC
                //			//等待MI设置完成
                //			if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS &&
                //					m_mask_5axis_rot_nolimit != 0){
                //				if(m_mask_5axis_rot_nolimit != m_n_mask_clear_pos){
                //					break;
                //				}
                //			}
                //#endif

                this->SetMFSig(m_index+m_index_add, false);    //复位选通信号

                if(mcode == 30)
                    this->m_p_f_reg->DM30 = 0;  //复位DM30
                else
                    this->m_p_f_reg->DM02 = 0;  //复位DM02

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index+m_index_add);

#ifdef USES_ADDITIONAL_PROGRAM
                if(this->m_n_add_prog_type == NONE_ADD){
#endif
                    if(mcode == 30){

                        ResetMcLineNo();//复位MC模块当前行号
                        this->SetCurLineNo(1);
                        printf("reset lineno : 1\n");
                    }

                    this->m_p_f_reg->STL = 0;
                    this->m_p_f_reg->SPL = 0;
                    this->m_p_f_reg->OP = 0;

                    //工件计数加一
                    if(this->m_channel_status.chn_work_mode == AUTO_MODE){

                        this->m_channel_status.workpiece_count++;
                        g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
                        this->m_channel_status.workpiece_count_total++;
                        g_ptr_parm_manager->SetTotalWorkPiece(m_n_channel_index, m_channel_status.workpiece_count_total);
                        this->SendWorkCountToHmi(m_channel_status.workpiece_count);  //通知HMI更新加工计数

                        g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC,"@#@#加工计数加一：%d", m_channel_status.workpiece_count);
                        this->ResetMode();   //模态恢复默认值

                        //激活工件坐标系参数
                        if(g_ptr_parm_manager->ActiveCoordParam(m_n_channel_index))
                            this->SetMcCoord(true);

                        //激活待生效的刀具偏置数据
                        if(g_ptr_parm_manager->ActiveToolOffsetParam(m_n_channel_index)){
                            if(this->m_channel_status.cur_h_code > 0)
                                this->SetMcToolOffset(true);
                        }

                        g_ptr_parm_manager->ActiveNewStartParam();


                    }
#ifdef USES_ADDITIONAL_PROGRAM
                }
#endif

                CompileOver();

                //#ifdef USES_FIVE_AXIS_FUNC
                //
                //			if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS &&
                //					m_mask_5axis_rot_nolimit != 0){
                //				//从MC同步工件坐标
                //				this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
                //
                //				m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
                //				m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
                //
                //				this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
                //
                //		//		printf("intp pos (%lf, %lf, %lf, %lf)\n", m_channel_mc_status.intp_pos.x, m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z,
                //		//				m_channel_mc_status.intp_pos.a6);
                //
                //				//从MI同步机械坐标
                //				this->m_p_channel_engine->SendMonitorData(false, false);
                //			}
                //#endif
                if(m_channel_status.chn_work_mode == AUTO_MODE){
#ifdef USES_ADDITIONAL_PROGRAM
                    if(this->m_n_add_prog_type == NONE_ADD){
                        this->SendMachOverToHmi();  //发送加工结束消息给HMI
                    }
#else
                    this->SendMachOverToHmi();  //发送加工结束消息给HMI
#endif
                }
                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态

                printf("execute M30 over\n");
            }

            break;
        case 0:		//M00无条件暂停
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M00\n");
                //TODO 将代码发送给PMC
                this->SendMCodeToPmc(mcode, m_index+m_index_add);
                this->m_p_f_reg->DM00 = 1;  //置位DM00

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                this->SetMFSig(m_index+m_index_add, true);    //置位选通信号
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index+m_index_add)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index+m_index_add, false);    //复位选通信号

                this->m_p_f_reg->DM00 = 0;  //复位DM00

                this->m_p_f_reg->STL = 0;
                this->m_p_f_reg->SPL = 1;

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index+m_index_add);


                this->PauseRunGCode();
                m_n_run_thread_state = PAUSE;

                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            }
            break;
        case 1:		//M01选择性暂停
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M01\n");
                //TODO 将代码发送给PMC
                this->SendMCodeToPmc(mcode, m_index+m_index_add);
                this->m_p_f_reg->DM01 = 1;  //置位DM01

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index+m_index_add, true);    //置位选通信号

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index+m_index_add)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index+m_index_add, false);    //复位选通信号

                this->m_p_f_reg->DM01 = 0;  //复位DM01

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index+m_index_add);


                if(this->m_channel_status.func_state_flags.CheckMask(FS_OPTIONAL_STOP)){//选停状态
                    PauseRunGCode();

                    m_n_run_thread_state = PAUSE;
                    this->m_p_f_reg->STL = 0;
                    this->m_p_f_reg->SPL = 1;
                }

                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            }
            break;
        case 3:		//M03主轴正转
            bHasSpdCmd = true;
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M03\n");
                if(this->m_b_prestart_spd){

                    if(m_n_spd_prestart_step == 0xFF)  //等待预启动结束
                        tmp->SetExecStep(m_index, 0xFF);   //有预启动，直接结束
                    break;
                }
                //将代码发送给PMC
                this->SendMCodeToPmc(mcode, m_index+m_index_add);

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                this->m_channel_status.spindle_dir = SPD_DIR_POSITIVE;  //修改主轴旋转方向

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index+m_index_add, true);    //置位选通信号

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index+m_index_add)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index+m_index_add, false);    //复位选通信号
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index+m_index_add);

                if(this->m_n_spindle_count > 0){
                    //				if(m_p_axis_config[this->m_spd_axis_phy[0]-1].spd_vctrl_mode == 2){//-10v ~ 10v
                    m_channel_status.rated_spindle_speed = m_n_cur_scode;
                    this->SendModeChangToHmi(S_MODE);
                    this->SpindleSpeedDaOut(m_channel_status.rated_spindle_speed);
                    //				}

                }
                this->m_p_f_reg->SPS = 1;    //标志主轴正转



                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            }
            break;
        case 4:		//M04主轴反转
            bHasSpdCmd = true;
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M04\n");

                if(this->m_b_prestart_spd){
                    if(m_n_spd_prestart_step == 0xFF)  //等待预启动结束
                        tmp->SetExecStep(m_index, 0xFF);   //有预启动，直接结束
                    break;
                }

                //将代码发送给PMC
                this->SendMCodeToPmc(mcode, m_index+m_index_add);

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                this->m_channel_status.spindle_dir = SPD_DIR_NEGATIVE;  //修改主轴旋转方向

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index+m_index_add, true);    //置位选通信号

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index+m_index_add)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index+m_index_add, false);    //复位选通信号
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index+m_index_add);


                if(this->m_n_spindle_count > 0){
                    //	if(m_p_axis_config[this->m_spd_axis_phy[0]-1].spd_vctrl_mode == 2){//-10v ~ 10v
                    m_channel_status.rated_spindle_speed = m_n_cur_scode;
                    this->SendModeChangToHmi(S_MODE);
                    this->SpindleSpeedDaOut(m_channel_status.rated_spindle_speed);
                    //	}
                }
                this->m_p_f_reg->SPS = 2;    //标志主轴反转


                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            }

            break;
        case 5:		//M05主轴停转
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M05\n");
                if(this->m_b_prestart_spd){
                    //TODO 报错，主轴预启动期间不能调用M05
                    CreateError(ERR_SPD_PRESTART_M05, ERROR_LEVEL, CLEAR_BY_MCP_RESET, tmp->GetLineNo(), m_n_channel_index);
                    this->m_error_code = ERR_SPD_PRESTART_M05;
                    break;
                }

                //TODO 将代码发送给PMC
                this->SendMCodeToPmc(mcode, m_index+m_index_add);

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                this->m_channel_status.spindle_dir = SPD_DIR_STOP;  //修改主轴旋转方向
                m_channel_status.rated_spindle_speed = 0;
                this->SendModeChangToHmi(S_MODE);

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index+m_index_add, true);    //置位选通信号

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index+m_index_add)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index+m_index_add, false);    //复位选通信号
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index+m_index_add);

                if(this->m_n_spindle_count > 0){
                    //	if(m_p_axis_config[this->m_spd_axis_phy[0]-1].spd_vctrl_mode == 2){//-10v ~ 10v
                    this->SpindleSpeedDaOut(m_channel_status.rated_spindle_speed);
                    //	}
                }
                this->m_p_f_reg->SPS = 0;    //标志主轴停转
                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            }
            break;
        case 6:		//M06换刀
            if(tmp->GetExecStep(m_index) == 0){
                //TODO 将代码发送给PMC
                printf("start to execute M06\n");
                this->SendMCodeToPmc(mcode, m_index+m_index_add);

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                this->SetMFSig(m_index+m_index_add, true);    //置位选通信号
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                    }else
                        break;
                }

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index+m_index_add)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index+m_index_add, false);    //复位选通信号
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index+m_index_add);

                this->m_n_ref_tool_life_delay = 3;

                //				this->SetMcToolLife();

                printf("execute M06 over: cur_T = %hhu\n", m_channel_status.cur_tool);

                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            }

            break;
        case 98:	//M98子程序调用
            printf("execute M98\n");
            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            break;
        case 99:   //M99
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M99, subprog_count = %hhu\n", m_n_subprog_count);
                if(m_n_subprog_count > 0){
                    m_n_subprog_count--;
                    m_b_ret_from_macroprog = false;
                }
                else{
                    m_p_compiler->RecycleCompile();   //主程序则循环调用

                    this->m_channel_status.workpiece_count++;  //工件计数加一
                    g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
                    this->m_channel_status.workpiece_count_total++;
                    g_ptr_parm_manager->SetTotalWorkPiece(m_n_channel_index, m_channel_status.workpiece_count_total);
                    this->SendWorkCountToHmi(m_channel_status.workpiece_count);  //通知HMI更新加工计数
                }

                //TODO 将代码发送给PMC
                this->m_p_f_reg->DM99 = 1;  //置位DM99, 维持20ms

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                tmp->IncreaseExecStep(m_index);

            }else if(tmp->GetExecStep(m_index) == 2){
                this->m_n_run_thread_state = RUN;

                this->m_p_f_reg->DM99 = 0;
                tmp->SetExecStep(m_index, 0xFF);    //置位结束状态

                printf("execute M99 over\n");
            }
            break;
        case 998:	//M998 用于暂停编译
            printf("execute M998\n");
            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            break;
        case 999:  //M999 暂停编译，并且同步位置
            printf("execute M999\n");
            //同步位置
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
                //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //同步已编译的轴移动指令的位置

            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            break;
#ifdef USES_TWINING_FUNC
        case 560:  //激活缠绕功能
            ActiveTwiningFunc(mcode);

            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            printf("active twining function\n");
            break;
        case 561:  //关闭缠绕功能
            ActiveTwiningFunc(mcode);

            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            printf("close twining function\n");
            break;
        case 562:
        case 563:
        case 564:
        case 565:
        case 566:
        case 567:
        case 568:
        case 569:
            ActiveTwiningFunc(mcode);

            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            break;
#endif
        case 4000:
        case 4001:
        case 4002:
        case 4003:
        case 4004:
        case 4005:
        case 4006:
        case 4007:
        case 4008:
        case 4009:
            this->MiDebugFunc(mcode);
            tmp->SetExecStep(m_index, 0xFF);    //置位结束状态
            break;

        default:   //其它M代码
            if(mcode >= 40010 && mcode <= 40083){//处理力矩速度模式切换
#ifdef USES_SPEED_TORQUE_CTRL
                ProcessCtrlModeSwitch(tmp, m_index);
                break;
#endif
            }
            else if(mcode >= 41011 && mcode <= 41648){   //TODO 处理轴映射切换
                ProcessAxisMapSwitch(tmp, m_index);
                break;
            }

            if(tmp->GetExecStep(m_index) == 0){
                //TODO 将代码发送给PMC
                g_ptr_trace->PrintLog(LOG_ALARM, "执行的M代码：M%02d, index=[%hhu,%hhu]", mcode, m_index, m_index_add);
                this->SendMCodeToPmc(mcode, m_index+m_index_add);

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //等待TMF延时，置位MF选通信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                this->SetMFSig(m_index+m_index_add, true);    //置位选通信号
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //等待FIN信号
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //开始计时
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//超过200ms任未进入执行状态，则告警“不支持的M代码”
                        CreateError(ERR_M_CODE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mcode, m_n_channel_index);
                        this->m_error_code = ERR_M_CODE;
                        printf("NOT support m code!!!!!!![%u, %hhu, %hhu]\n", time_elpase, m_index, m_index_add);
                    }else
                        break;
                }

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 3){
                if(this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index+m_index_add)){
                    tmp->SetExecStep(m_index, 2);
                    break;
                }

                //等待TFIN延时，复位MF信号
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//未到延时时间

                this->SetMFSig(m_index+m_index_add, false);    //复位选通信号
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //等待FIN信号复位
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //复位辅助指令信号和DEN信号
                this->SendMCodeToPmc(0, m_index+m_index_add);


                tmp->IncreaseExecStep(m_index);
            }else{
                this->ExecMCode(tmp, m_index);  //执行某些M代码需要系统执行的动作

                //				tmp->SetExecStep(m_index, 0xFF);    //置位结束状态

            }
            break;
        }
        if(tmp->GetExecStep(m_index) != 0xFF)//还未执行结束，返回false
            bRet = false;
    }

    if(bRet){

        if(this->m_p_f_reg->DEN == 1)
            this->m_p_f_reg->DEN = 0;  //复位DEN信号

        if(bHasSpdCmd && m_b_prestart_spd){  //复位主轴预启动标志
            m_b_prestart_spd = false;
            m_n_spd_prestart_step = 0;
            printf("spindle prestart over####3\n");
        }

    }

    return bRet;
}

#endif

/**
 * @brief 获取当前刀具寿命
 * @return
 */
int ChannelControl::GetCurToolLife(){
    int life = 0;
    if(this->m_channel_status.cur_tool > 0)
        life = this->m_p_chn_tool_info->tool_life_cur[this->m_channel_status.cur_tool-1];

    return life;
}

#ifdef USES_GRIND_MACHINE
/**
 * @brief 设置G12.2磨削动态参数
 * @param intp_type : 特殊插补类型，0-	关闭（G13.1）   122?G12.2 123--G12.3 121 ?G12.1   71-G7.1
 * @param data_idx : 数据序号，0-3
 * @param data1 ： 数据1
 * @param data2 ： 数据2
 */
void ChannelControl::SetMcGrindParamDynamic(uint16_t intp_type, uint16_t data_idx, int data1, int data2){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_SET_SPEC_INTP_PARM_DYNAMIC;
    cmd.data.data[0] = intp_type;
    cmd.data.data[1] = data_idx;
    cmd.data.data[2] = data1&0xFFFF;
    cmd.data.data[3] = ((data1>>16)&0xFFFF);
    cmd.data.data[4] = data2&0xFFFF;
    cmd.data.data[5] = ((data2>>16)&0xFFFF);

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 使能震荡磨
 * @param enable : true--使能   false--禁止
 */
void ChannelControl::EnableGrindShock(bool enable){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_ENABLE_GRIND_SHOCK;
    cmd.data.data[0] = enable?1:0;


    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 执行复位安全位置动作
 */
void ChannelControl::ReturnSafeState(){
    uint8_t phy_axis = 0;
    uint8_t chn_axis = 0;
    switch(m_n_ret_safe_step){
    case 0: //first step   X轴回到安全点
        printf("ReturnSafeState, step 0\n");
        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_X);
        if(phy_axis == NO_AXIS){
            this->m_n_ret_safe_step = 8;   //失败，跳出
            printf("X轴未配置！\n");
            break;
        }

        if(m_channel_mc_status.cur_mode != MC_MODE_MANUAL){  //MC还未切换到手动状态
            break;
        }

        this->SetMcSpecialIntpMode(0);  //取消磨削状态

        chn_axis = this->GetChnAxisFromName(AXIS_NAME_X);
        printf("chn=%hhu, safepos = %lf, speed = %lf\n", chn_axis, m_p_grind_config->safe_pos_x,
               m_p_axis_config[phy_axis].reset_speed);
        this->ManualMove(chn_axis, this->m_p_grind_config->safe_pos_x, this->m_p_axis_config[phy_axis].reset_speed);   //X轴以复位速度返回安全位置，机械坐标系

        this->m_n_ret_safe_step++;
        break;
    case 1://second step    等待X轴到位
        //	printf("ReturnSafeState, step 1\n");
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_X);
        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_X);

        if(fabs(m_channel_rt_status.cur_pos_machine.GetAxisValue(chn_axis) - m_p_grind_config->safe_pos_x) < 1e-3){ //到位
            m_n_ret_safe_step++;
        }else{
            //			printf("step1: intp_pos=%lf, fb_pos = %lf\n", m_p_channel_engine->GetPhyAxisMachPosIntp(phy_axis),
            //					m_p_channel_engine->GetPhyAxisMachPosFeedback(phy_axis));
        }
        this->ManualMove(chn_axis, this->m_p_grind_config->safe_pos_x, this->m_p_axis_config[phy_axis].reset_speed);   //X轴以复位速度返回安全位置，机械坐标系

        break;
    case 2:{//third step   C轴回正
        printf("ReturnSafeState, step 2\n");
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_C);   //C轴对应通道轴索引号
        if(chn_axis == NO_AXIS){
            this->m_n_ret_safe_step = 8;   //失败，跳出
            printf("C轴未配置！\n");
            break;
        }
        double pos = m_channel_rt_status.cur_pos_machine.GetAxisValue(chn_axis);  //当前C轴机械坐标

        //计算最近的360整数倍位置
        double ratio = pos/360.;
        int ratio_int = ratio;
        double pf = fmod(ratio, 1);
        if(pf > 0.5)
            ratio_int++;
        else if(pf < -0.5)
            ratio_int--;

        m_df_c_pos_tar = ratio_int *360.0;

        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_C);
        if(phy_axis == NO_AXIS){
            this->m_n_ret_safe_step = 8;   //失败，跳出
            printf("C轴未配置！\n");
            break;
        }

        this->ManualMove(chn_axis, m_df_c_pos_tar, this->m_p_axis_config[phy_axis].reset_speed);   //C轴以复位速度返回零点位置，机械坐标系

        this->m_n_ret_safe_step++;
        printf("move c axis to pos: %lf\n", m_df_c_pos_tar);
        break;
    }
    case 3://fourth step 等待C轴回正到位
        //printf("ReturnSafeState, step 3\n");
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_C);
        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_C);
        if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(chn_axis) - m_df_c_pos_tar) >= 1e-3){ //未到位
            this->ManualMove(chn_axis, m_df_c_pos_tar, this->m_p_axis_config[phy_axis].reset_speed);   //C轴以复位速度返回零点位置，机械坐标系
            break;
        }

        this->m_n_ret_safe_step++;
        break;
    case 4: //开始进行C轴坐标清整数圈
        printf("ReturnSafeState, step 4\n");
        m_n_mask_clear_pos = 0;
        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_C);

        this->SendMiClearPosCmd(phy_axis+1, 360*1000);

        this->m_n_ret_safe_step++;
        break;
    case 5:{
        printf("ReturnSafeState, step 5\n");
        //等待MI设置完成
        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_C);
        uint64_t mask = 0x01<<phy_axis;
        if(mask == this->m_n_mask_clear_pos){  //
            this->m_n_ret_safe_step++;;	//跳转下一步
        }
    }
        break;
    case 6:
        printf("ReturnSafeState, step 6\n");
        //同步位置

        //从MC同步工件坐标
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        else
            this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;

        this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置

        //		printf("intp pos (%lf, %lf, %lf, %lf)\n", m_channel_mc_status.intp_pos.x, m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z,
        //				m_channel_mc_status.intp_pos.a6);

        //从MI同步机械坐标
        this->m_p_channel_engine->SendMonitorData(false, false);

        //		printf("cur mach pos (%lf, %lf, %lf, %lf)\n", m_channel_rt_status.cur_pos_machine.x, m_channel_rt_status.cur_pos_machine.y,
        //				m_channel_rt_status.cur_pos_machine.z, m_channel_rt_status.cur_pos_machine.a6);

        this->m_n_ret_safe_step++;  //跳转下一步
        break;
    case 7: //结束
        printf("Succeed to return safe pos!!!!!\n");
        this->m_b_ret_safe_state = false;
        this->m_n_ret_safe_step = 0;
        break;
    case 8: //出错
        printf("Failed to return safe pos!!!!!\n");
        this->m_b_ret_safe_state = false;
        this->m_n_ret_safe_step = 0;
        break;
    default:
        break;
    }
    return;
}

/**
 * @brief 运行从机上下料流程
 */
void ChannelControl::RunM66_slave(){
    uint8_t cur_chn = this->m_p_mech_arm_state->cur_chn;   //当前通道
    int delay_time_vac = this->m_p_mech_arm_param->delay_time_vac*1000;
    int wait_overtime = this->m_p_mech_arm_param->wait_overtime*1000;   //等待超时
    int work_vac_delay = this->m_p_mech_arm_param->work_vac_check_delay*1000;   //工位吸真空延时

    if(this->m_p_mech_arm_state->run_flag != MECH_ARM_IDLE){
        if(this->m_p_mech_arm_state->cur_chn != GRIND_SLAVE_CHN)//主机占用，等待
            return;  //等待控制权
        else if(this->m_p_mech_arm_state->cur_step > 29)  //从通道上一次M66还没执行完，等待
            return;
    }
    else{//抢占控制权
        int err = pthread_mutex_trylock(&m_p_mech_arm_state->mutex_change_run);
        if(err == 0){//lock成功
            this->m_p_mech_arm_state->run_flag = MECH_ARM_RUN;
            this->m_p_mech_arm_state->cur_chn = GRIND_SLAVE_CHN;
            m_p_mech_arm_state->cur_step = 0;
            pthread_mutex_unlock(&m_p_mech_arm_state->mutex_change_run);

            this->m_p_mech_arm_state->is_exit_inter_area = false;
            if(this->m_p_g_reg->left_claw_vac_check == 0){
                this->m_p_mech_arm_state->has_material = false;
                this->m_p_mech_arm_state->repos = false;
            }else{
                this->m_p_mech_arm_state->has_material = true;
            }

            if(this->m_p_g_reg->right_claw_vac_check == 0)
                this->m_p_mech_arm_state->has_finished_material = false;
            else
                this->m_p_mech_arm_state->has_finished_material = true;

            //强制夹爪上升
            this->m_p_f_reg->left_claw_down = 0;
            this->m_p_f_reg->right_claw_down = 0;

            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        }else
            return;
    }

    if(this->m_p_mech_arm_state->run_flag == MECH_ARM_PAUSE && m_p_mech_arm_state->cur_step > 0)
        return;

    if(m_p_mech_arm_state->cur_step == 0){  //step-0   起始步骤
        g_ptr_trace->PrintLog(LOG_ALARM, "执行的从机M代码：M66\n");

        printf("goto step 1\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 1){ //step-1  判断状态，决定跳转动作
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 300000){ //延时300ms
            return;
        }

        if(m_p_mech_arm_state->total_count == 0){
            printf("料盘规格为0！\n");
            return;
        }

        //工位气缸上升不到位
        if(this->m_p_g_reg->right_work_up_check == 1){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_UP_RIGHT;   //右机工位气缸上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;  //工位气缸上升不到位
        }

        if(this->m_p_g_reg->right_tray_inpos == 1){//料盘不在位置
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){
                this->m_p_mech_arm_state->err_state = ERR_ARM_NO_TRAY;   //料盘不在位置
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;  //料盘不在位置
        }

        //检查上下料轴是否回零点
        if((this->m_p_f_reg->ZRF1 & (0x01<<5))==0 || (this->m_p_f_reg->ZRF1 & (0x01<<6))==0){
            //有轴未回零点
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){
                this->m_p_mech_arm_state->err_state = ERR_ARM_AXIS_NO_REF;
                this->m_p_channel_engine->SendHMIMechArmException(ERR_ARM_AXIS_NO_REF);  //向HMI告警，机械臂未回零
            }
            return;
        }

        //动作前夹爪必须上到位，保护措施
        if(this->m_p_g_reg->left_claw_up_check == 0 || this->m_p_g_reg->right_claw_up_check == 0){//夹爪位上到位
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待左爪上到位信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //机械臂上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;
        }

        //Y轴处于干涉位置，必须先退出干涉区域
        if(this->m_p_mech_arm_state->is_exit_inter_area){
            if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) -
                    m_p_mech_arm_state->y_target_pos) > 0.005){//还未离开干涉区域
                return;
            }
            this->m_p_mech_arm_state->is_exit_inter_area = false;  //复位标志
        }else if(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) > m_p_mech_arm_param->pos_tray_base_y[GRIND_SLAVE_CHN]){
            m_p_mech_arm_state->y_target_pos = m_p_mech_arm_param->pos_tray_base_y[GRIND_SLAVE_CHN];
            this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                    m_p_mech_arm_param->speed, false);  //绝对坐标
            this->m_p_mech_arm_state->is_exit_inter_area = true;
            return;
        }

        //料盘是否已经放满,提醒换料
        if(m_p_mech_arm_state->cur_index[GRIND_SLAVE_CHN] == m_p_mech_arm_state->total_count /*&&
                                                        m_p_mech_arm_state->cur_finished_count[GRIND_SLAVE_CHN] >= m_p_mech_arm_state->total_count-1*/){//成品已放满
            if(this->m_p_f_reg->change_tray_right == 0){
                this->m_p_f_reg->change_tray_right = 1;   //提醒换料盘
                //				this->m_p_channel_engine->SendHMIMechArmException(ERR_NEW_TRAY_REQ);  //向HMI请求更换料盘
            }
            return;
        }


        if(this->m_p_mech_arm_state->has_material){
            if(this->m_p_mech_arm_state->repos){
                printf("jump to step 15\n");
                this->m_p_mech_arm_state->cur_step = 15;//持料等待，则直接去取放料
                //记录时间
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            }else{//未对位，则先对位
                //计算X、Y的位置
                this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_corr_fine_x+
                        this->m_p_mech_arm_param->pos_corr_rough_x;
                this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_corr_fine_y+
                        this->m_p_mech_arm_param->pos_corr_rough_y;

                this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                        m_p_mech_arm_param->speed, false);  //绝对坐标
                this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                        m_p_mech_arm_param->speed, false);  //绝对坐标

                //对位气缸松开
                this->m_p_f_reg->correct_pos_x = 0;
                this->m_p_f_reg->correct_pos_y = 0;

                this->m_p_mech_arm_state->corr_index = 0;  //复位对位次数

                //记录时间
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

                printf("jump to step 7\n");
                this->m_p_mech_arm_state->cur_step = 7;  //去对位
            }
        }else if(this->m_p_mech_arm_state->cur_index[cur_chn] < this->m_p_mech_arm_state->total_count){
            printf("goto step 2\n");
            m_p_mech_arm_state->cur_step = 2;//先去取料
        }else{//料盘已经无料可取，直接跳转去取成品
            printf("goto step 15\n");
            this->m_p_mech_arm_state->cur_step = 15;//无毛坯可取，则直接去取放料
            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
        }
    }else if(m_p_mech_arm_state->cur_step == 2){ //step-2 X/Y轴同时开始移动到取料位置
        //计算X、Y的目标位置
        uint8_t row = m_p_mech_arm_state->cur_index[cur_chn]/m_p_mech_arm_param->col_count;   //当前行
        uint8_t col = m_p_mech_arm_state->cur_index[cur_chn]%m_p_mech_arm_param->col_count;	  //当前列
        m_p_mech_arm_state->x_target_pos =
                m_p_mech_arm_param->pos_tray_base_x[cur_chn] + m_p_mech_arm_param->dis_tray_x * col;
        m_p_mech_arm_state->y_target_pos =
                m_p_mech_arm_param->pos_tray_base_y[cur_chn] - m_p_mech_arm_param->dis_tray_y * row;

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                m_p_mech_arm_param->speed, false);  //绝对坐标
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                m_p_mech_arm_param->speed, false);  //绝对坐标

        printf("goto step 3, x:%lf, y:%lf\n", m_p_mech_arm_state->x_target_pos, m_p_mech_arm_state->y_target_pos);
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 3){  //step-3  等待轴运行到位，左夹爪下（取料）
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005
                || fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //没有到位

        //左爪下
        this->m_p_f_reg->left_claw_down = 1;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 4\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 4){ //step-4  延时300ms等待左爪下到位，开真空
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 300000){ //延时300ms
            return;
        }

        //开左爪真空
        this->m_p_f_reg->left_claw_vac = 1;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 5\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 5){ //step-5  确认左爪真空信号，左爪上
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;


        if(this->m_p_g_reg->left_claw_vac_check == 0 && time_elpase < wait_overtime)  //等待左爪真空信号
            return;

        //左爪上
        this->m_p_f_reg->left_claw_down = 0;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 6\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 6){ //step-6   确认左爪上到位，X、Y同时移动到粗对位位置
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 800000)  //延时800ms，防止误判吸真空信号
            return;

        if(this->m_p_g_reg->left_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待左爪上到位信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //机械臂上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;  //左爪上未到位
        }

        if(this->m_p_g_reg->left_claw_vac_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待左爪真空信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_FETCH_VAC;   //料盘取料失败，吸真空异常
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //向HMI发送异常
            }
            return;  //左爪真空信号异常
        }
        //		printf("run slave m66, left_vac=%hhu, left_up=%hhu, over=%d\n", m_p_g_reg->left_claw_vac_check,
        //				m_p_g_reg->left_claw_up_check, wait_overtime);


        this->m_p_mech_arm_state->cur_index[cur_chn]++;  //当前取料序号加一

        //计算X、Y的位置
        this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_corr_fine_x+
                this->m_p_mech_arm_param->pos_corr_rough_x;
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_corr_fine_y+
                this->m_p_mech_arm_param->pos_corr_rough_y;

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                m_p_mech_arm_param->speed, false);  //绝对坐标
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                m_p_mech_arm_param->speed, false);  //绝对坐标

        //对位气缸松开
        this->m_p_f_reg->correct_pos_x = 0;
        this->m_p_f_reg->correct_pos_y = 0;

        this->m_p_mech_arm_state->corr_index = 0;  //复位对位次数

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 7， x=%lf, y=%lf\n", m_p_mech_arm_state->x_target_pos, m_p_mech_arm_state->y_target_pos);
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 7){  //step-7    等待轴到位，左爪下
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005
                || fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //没有到位

        if(this->m_p_g_reg->correct_pos_x_check == 0 || this->m_p_g_reg->correct_pos_y_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待对位气缸松开信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_CORR_POS;   //对位失败，对位气缸不在位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;   //对位气缸松开未完成
        }

        this->m_p_f_reg->left_claw_down = 1;  //左爪下

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 8\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 8){  //step-8 延时等待左爪下到位，关左爪真空
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 200000){ //延时200ms
            return;
        }

        this->m_p_f_reg->left_claw_vac = 0;  //关左爪真空

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 9\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 9){	//step-9  延时300ms，左夹爪上
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        //		if(time_elpase < delay_time_vac){ //延时300ms
        //			return;
        //		}

        if(this->m_p_g_reg->left_claw_vac_check == 1 && time_elpase < wait_overtime)
            return;

        this->m_p_f_reg->left_claw_down = 0;  //左夹爪上

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 10\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 10){  //step-10 等待左夹爪上到位，对位气缸夹紧，X、Y轴移动到精对位位置
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //延时500ms
            return;
        }

        if(this->m_p_g_reg->left_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待左爪上到位信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //机械臂上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;  //左爪上未到位
        }

        //对位气缸夹紧
        this->m_p_f_reg->correct_pos_x = 1;
        this->m_p_f_reg->correct_pos_y = 1;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        //计算X、Y轴目标位置
        this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_corr_fine_x;
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_corr_fine_y;

        //使用固定低速，防止抖动
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                2000, false);  //绝对坐标
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                2000, false);  //绝对坐标

        printf("goto step 11， x=%lf, y=%lf\n", m_p_mech_arm_state->x_target_pos, m_p_mech_arm_state->y_target_pos);
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 11){ //step-11 延时1000ms，并等待X、Y轴到位，左爪下
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 1000000){ //延时1000ms
            return;
        }

        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005
                || fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //没有到位

        if(++this->m_p_mech_arm_state->corr_index < this->m_p_mech_arm_param->corr_pos_count){//对位次数不够
            //对位气缸松开
            this->m_p_f_reg->correct_pos_x = 0;
            this->m_p_f_reg->correct_pos_y = 0;

            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

            printf("repos, jump to step 10\n");
            m_p_mech_arm_state->cur_step = 10;
            return;
        }

        m_p_mech_arm_state->corr_index = 0;

        this->m_p_f_reg->left_claw_down = 1;   //左爪下

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 12\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 12){  //step-12  延时500ms，开左爪真空
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //延时500ms
            return;
        }

        this->m_p_f_reg->left_claw_vac = 1;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);

        printf("goto step 13\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 13){ //step-13  延时300ms，确认左爪真空信号，对位气缸松开
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        //		if(time_elpase < 300000){ //延时300ms
        //			return;
        //		}

        if(this->m_p_g_reg->left_claw_vac_check == 0 && time_elpase < wait_overtime){
            //记录时间2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
            return;   //左爪真空信号未确认
        }

        time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm_2.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm_2.tv_usec;
        if(time_elpase < 500000){ //延时500ms
            return;
        }


        //对位气缸松开
        this->m_p_f_reg->correct_pos_x = 0;
        this->m_p_f_reg->correct_pos_y = 0;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 14\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 14){ //step-14  确认对位气缸松开到位，左夹爪上
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->correct_pos_x_check == 0 || this->m_p_g_reg->correct_pos_y_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待对位气缸松开到位信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_CORR_POS;   //对位失败，对位气缸不在位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;   //对位气缸松开未完成
        }

        this->m_p_f_reg->left_claw_down = 0;

        this->m_p_mech_arm_state->has_material = true;   //夹爪持料
        this->m_p_mech_arm_state->repos = true;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 15\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 15){ //step-15  延时1s，确认左爪上到位，再次确认左爪真空信号，移动X轴到取料（送料）位置
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 1000000){ //延时1000ms
            return;
        }
        if(this->m_p_g_reg->left_claw_up_check == 0 ||
                this->m_p_g_reg->right_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待夹爪上到位信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //机械臂上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;
        }

        if(this->m_p_mech_arm_state->has_material && this->m_p_g_reg->left_claw_vac_check == 0 ){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待夹爪真空信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_CORR_VAC;   //对位失败，吸真空异常
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;
        }

        //更新从机工位状态，是否有料，工位吸真空检测有信号则工位有料，否则无料
        if(this->m_p_g_reg->right_work_vac_check == 1)
            this->m_p_mech_arm_state->work_material[GRIND_SLAVE_CHN] = true;  //工位有料
        else
            this->m_p_mech_arm_state->work_material[GRIND_SLAVE_CHN] = false;  //工位无料

        //计算X轴目标位置
        if(this->m_p_mech_arm_state->work_material[cur_chn]){//加工位有料，取料位置
            this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_work_x[cur_chn] -
                    this->m_p_mech_arm_param->dis_mech_arm;
        }else{//加工位没有料，直接送料
            this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_work_x[cur_chn];
        }

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                m_p_mech_arm_param->speed, false);  //绝对坐标

        printf("goto step 16, X=%lf\n", this->m_p_mech_arm_state->x_target_pos);
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 16){ //step-16  等待X轴到位，确认加工位真空信号以及压紧气缸上升到位信号，移动Y轴到取料位
        //		if(this->m_p_g_reg->work_up_check == 0){
        //			this->m_p_f_reg->work_hold = 0;  //压紧气缸上升
        //			return;
        //		}
        //		if(this->m_p_mech_arm_state->work_material[cur_chn] &&
        //				this->m_p_g_reg->right_work_vac_check == 0){  //加工位有料则确认真空信号
        //			if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){  //等待工位真空信号
        //				this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_FETCH_VAC;   //工位取料失败，吸真空异常
        //				this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
        //			}
        //			return;
        //		}

        //X轴是否到位
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005){
            return;   //没有到位
        }

        //工位气缸上升不到位
        if(this->m_p_g_reg->right_work_up_check == 1){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_UP_RIGHT;   //右机工位气缸上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;  //工位气缸上升不到位
        }

        double spd = m_p_mech_arm_param->speed;
        if(!this->m_p_mech_arm_state->work_material[cur_chn]){//工位无成品，直接放料，速度减半
            spd = spd/2;
        }
        //计算Y轴目标位置
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_work_y[cur_chn];

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                spd, false);  //绝对坐标

        printf("goto step 17\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 17){ //step-17  等待Y轴到位，取料则右爪下，放料则左爪下
        //Y轴是否到位
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005){
            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            return;   //没有到位
        }

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//取料
            this->m_p_f_reg->right_claw_down = 1;
        }else{
            struct timeval time_now;
            gettimeofday(&time_now, NULL);
            int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                    time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
            if(time_elpase < 1000000){ //延时1000ms
                return;
            }
            this->m_p_f_reg->left_claw_down = 1;  //放料
        }

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 18\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 18){ //step-18  延时300ms，取料开右爪真空，放料开加工位真空
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;


        if(this->m_p_mech_arm_state->work_material[cur_chn]){//取料，开右爪真空
            if(time_elpase < 300000){ //延时300ms
                return;
            }
            this->m_p_f_reg->right_claw_vac = 1;
        }else{
            this->m_p_f_reg->work_vac_right = 1;  //放料，通知右机开加工位真空
            //记录时间2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
        }

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 19\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 19){ //step-19  取料确认右真空信号后关加工位真空，放料确认加工位真空信号后关左爪真空
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//取料
            if(this->m_p_g_reg->right_claw_vac_check == 0 && time_elpase < wait_overtime)
                return;

            this->m_p_f_reg->work_vac_right = 0;   //通知右机关加工位真空
        }else{//放料
            if(this->m_p_g_reg->right_work_vac_check == 0 && time_elpase < wait_overtime){  //确认右机加工位真空信号
                //记录时间2
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
                return;
            }

            time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm_2.tv_sec)*1000000+
                    time_now.tv_usec-m_p_mech_arm_state->time_start_arm_2.tv_usec;
            if(time_elpase < work_vac_delay){ //工位吸真空延时后再关左爪真空
                return;
            }

            this->m_p_f_reg->left_claw_vac = 0;
        }

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 20\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 20){ //step-20  取料确认加工位真空确认信号消失后延时300ms，放料确认左爪真空信号消失后延时200ms
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//取料
            if(this->m_p_g_reg->right_work_vac_check == 1 && time_elpase < wait_overtime)
                return;
        }else{//放料
            if(this->m_p_g_reg->left_claw_vac_check == 1 && time_elpase < wait_overtime)
                return;
        }

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 21\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 21){ //step-21   延时300ms后，取料则右爪上，放料则左爪上
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < delay_time_vac){ //延时300ms
            return;
        }

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//取料
            this->m_p_f_reg->right_claw_down = 0;

            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            //记录时间2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
        }else{//放料
            this->m_p_f_reg->left_claw_down = 0;
            this->m_p_mech_arm_state->has_material = false;   //夹爪无料
            this->m_p_mech_arm_state->repos = false;

            this->m_p_mech_arm_state->work_material[cur_chn] = true;    //加工位有料

            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

            //跳转28步
            printf("21 jump to step 28\n");
            m_p_mech_arm_state->cur_step = 28;
            return;
        }

        printf("goto step 22\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 22){ //step-22  确认右爪上升到位并且确认真空信号, 如果左爪有料则移动X轴去放料，如果无料则跳转28步
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->right_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待右爪上到位信号
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //机械臂上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }

            //记录时间2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);

            return;
        }

        time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm_2.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm_2.tv_usec;
        if(time_elpase < 600000){   //右爪上升到位后，再延时600ms检测右爪吸真空信号
            return;
        }

        if(this->m_p_g_reg->right_claw_vac_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待右爪真空信号
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_FETCH_VAC;   //工位取料失败，吸真空异常
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;
        }

        this->m_p_mech_arm_state->work_material[cur_chn] = false;  //加工位无料
        this->m_p_mech_arm_state->has_finished_material = true;  //夹爪有成品

        if(this->m_p_mech_arm_state->has_material){  //左爪有料
            this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_work_x[cur_chn];

            this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                    m_p_mech_arm_param->speed/2, false);  //绝对坐标
        }else{ //左爪无料，跳转
            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

            //跳转28步
            printf("22 jump to step 28\n");
            m_p_mech_arm_state->cur_step = 28;
            return;
        }

        printf("goto step 23\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 23){ //step-23   等待X轴到位，左爪下
        //X轴是否到位
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005){
            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            return;   //没有到位
        }

        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 1000000){ //延时1000ms
            return;
        }

        this->m_p_f_reg->left_claw_down = 1;

        //记录时间
        //		gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 24\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 24){ //step-24  延时200ms，开右加工位真空
        //		struct timeval time_now;
        //		gettimeofday(&time_now, NULL);
        //		unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
        //				time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        //		if(time_elpase < 300000){ //延时300ms
        //			return;
        //		}

        this->m_p_f_reg->work_vac_right = 1;  //开工位真空

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
        //记录时间2
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);


        printf("goto step 25\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 25){ //step-25  确认右加工位真空信号，关左爪真空
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;


        if(this->m_p_g_reg->right_work_vac_check == 0 && time_elpase < wait_overtime){
            //记录时间2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);

            return;
        }

        if(time_elpase < work_vac_delay){ //延时500ms，等待工位吸真空稳定
            return;
        }

        this->m_p_f_reg->left_claw_vac = 0;

        this->m_p_mech_arm_state->work_material[cur_chn] = true;    //加工位有料

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 26\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 26){ //step-26  等待左爪真空信号消失
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->left_claw_vac_check == 1 && time_elpase < wait_overtime)
            return;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 27\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 27){ //step-27  延时200ms， 左爪上升
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < delay_time_vac){ //延时200ms
            return;
        }

        this->m_p_f_reg->left_claw_down = 0;

        this->m_p_mech_arm_state->has_material = false;   //夹爪无料
        this->m_p_mech_arm_state->repos = false;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 28\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 28){ //step-28  确认左右爪上升到位，移动Y轴到料盘基准位置
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //延时500ms
            return;
        }

        if(this->m_p_g_reg->left_claw_up_check == 0 || this->m_p_g_reg->right_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待夹爪上到位信号
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //机械臂上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;
        }

        //检查工位吸真空信号
        if(this->m_p_mech_arm_state->work_material[cur_chn] &&
                this->m_p_g_reg->right_work_vac_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_SET_VAC;   //工位放料失败，吸真空异常
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;
        }

        //计算Y轴目标位置
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_tray_base_y[cur_chn];

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                m_p_mech_arm_param->speed, false);  //绝对坐标

        printf("goto step 29\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 29){ //step-29   等待Y轴到位
        //Y轴是否到位
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //没有到位

        this->m_p_f_reg->m66_runover_right = 1; //通知从机，上下料M66指令执行完毕

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("succeed to execute M66, goto step 30\n");
        m_p_mech_arm_state->cur_step++;

    }
}

/**
 * @brief 处理M66指令，主机上下料请求
 * @param msg : 消息指针
 */
void ChannelControl::ProcessGrindM66(AuxMsg *msg){
    if(msg == nullptr)
        return;

    if(this->m_p_mech_arm_state->run_flag == MECH_ARM_PAUSE && msg->GetExecStep() > 0)
        return;

    if(msg->GetExecStep() > 0 && this->m_p_mech_arm_state->err_state != ERR_ARM_NONE)    //处于错误状态则返回，阻塞
        return;

    uint8_t cur_chn = this->m_p_mech_arm_state->cur_chn;   //当前通道
    int delay_time_vac = this->m_p_mech_arm_param->delay_time_vac*1000;
    int wait_overtime = this->m_p_mech_arm_param->wait_overtime*1000;   //等待超时
    int work_vac_delay = this->m_p_mech_arm_param->work_vac_check_delay*1000;   //工位吸真空延时
    if(msg->GetExecStep() == 0){  //step-0   起始步骤，不能等待
        g_ptr_trace->PrintLog(LOG_ALARM, "执行的M代码：M%02d", msg->GetMCode());

        printf("goto step 1\n");
        //		m_p_mech_arm_state->cur_step++;//此处没获取到控制权，不能修改cur_step
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 1){ //step-1   获取机械手控制权
        if(this->m_p_mech_arm_state->run_flag != MECH_ARM_IDLE){
            if(this->m_p_mech_arm_state->cur_chn != GRIND_MAIN_CHN)
                return;   //已经被占用，等待
            else if(this->m_p_mech_arm_state->cur_step > 29)  //上一次M66还没执行完，等待
                return;
        }else{
            int err = pthread_mutex_trylock(&m_p_mech_arm_state->mutex_change_run);
            if(err == 0){//lock成功
                this->m_p_mech_arm_state->run_flag = MECH_ARM_RUN;
                this->m_p_mech_arm_state->cur_chn = GRIND_MAIN_CHN;
                m_p_mech_arm_state->cur_step = 1;
                pthread_mutex_unlock(&m_p_mech_arm_state->mutex_change_run);

                this->m_p_mech_arm_state->is_exit_inter_area = false;
                if(this->m_p_g_reg->left_claw_vac_check == 0){
                    this->m_p_mech_arm_state->has_material = false;
                    this->m_p_mech_arm_state->repos = false;

                }else{
                    this->m_p_mech_arm_state->has_material = true;

                }

                if(this->m_p_g_reg->right_claw_vac_check == 0){
                    this->m_p_mech_arm_state->has_finished_material = false;

                }
                else{
                    this->m_p_mech_arm_state->has_finished_material = true;

                }

                //强制夹爪上升
                this->m_p_f_reg->left_claw_down = 0;
                this->m_p_f_reg->right_claw_down = 0;

                //记录时间
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

                this->m_p_mech_arm_state->msg = msg;
            }else
                return;
        }

        if(this->m_p_g_reg->left_tray_inpos == 1){//料盘不在位置
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){
                this->m_p_mech_arm_state->err_state = ERR_ARM_NO_TRAY;   //料盘不在位置
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;  //料盘不在位置
        }

        //检查上下料轴是否回零点
        if((this->m_p_f_reg->ZRF1 & (0x01<<5))==0 || (this->m_p_f_reg->ZRF1 & (0x01<<6))==0){
            //有轴未回零点
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){
                this->m_p_mech_arm_state->err_state = ERR_ARM_AXIS_NO_REF;
                this->m_p_channel_engine->SendHMIMechArmException(ERR_ARM_AXIS_NO_REF);  //向HMI告警，机械臂未回零
            }
            return;
        }

        if(m_p_mech_arm_state->total_count == 0){
            printf("料盘规格为0！\n");
            return;
        }

        //动作前夹爪必须上到位，保护措施
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 300000){ //延时300ms
            return;
        }
        if(this->m_p_g_reg->left_claw_up_check == 0 || this->m_p_g_reg->right_claw_up_check == 0){//夹爪位上到位
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待左爪上到位信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //机械臂上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;
        }

        //工位气缸上升不到位
        if(this->m_p_g_reg->work_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_UP_LEFT;   //左机工位气缸上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;  //工位气缸上升不到位
        }

        //Y轴处于干涉位置，必须先退出干涉区域
        if(this->m_p_mech_arm_state->is_exit_inter_area){
            if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) -
                    m_p_mech_arm_state->y_target_pos) > 0.005){//还未离开干涉区域
                return;
            }
            this->m_p_mech_arm_state->is_exit_inter_area = false;  //复位标志
        }else if(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) > m_p_mech_arm_param->pos_tray_base_y[GRIND_MAIN_CHN]){
            m_p_mech_arm_state->y_target_pos = m_p_mech_arm_param->pos_tray_base_y[GRIND_MAIN_CHN];
            this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                    m_p_mech_arm_param->speed, false);  //绝对坐标
            this->m_p_mech_arm_state->is_exit_inter_area = true;
            return;
        }


        //料盘是否已经放满,提醒换料
        if(m_p_mech_arm_state->cur_index[GRIND_MAIN_CHN] >= m_p_mech_arm_state->total_count /*&&
                                                        m_p_mech_arm_state->cur_finished_count[GRIND_MAIN_CHN] >= (m_p_mech_arm_state->total_count-1)*/){//成品已放满
            if(this->m_p_f_reg->change_tray_left == 0){
                this->m_p_f_reg->change_tray_left = 1;   //提醒换料盘
                this->m_p_channel_engine->SendHMIMechArmException(ERR_NEW_TRAY_REQ);  //向HMI请求更换料盘
            }
            return;
        }


        //		printf("main step-1 index=%hu, fin=%hu, total=%hu\n", m_p_mech_arm_state->cur_index[GRIND_MAIN_CHN],
        //				m_p_mech_arm_state->cur_finished_count[GRIND_MAIN_CHN], m_p_mech_arm_state->total_count);

        if(this->m_p_mech_arm_state->has_material){
            if(this->m_p_mech_arm_state->repos){ //已经对位
                printf("jump to step 15\n");
                this->m_p_mech_arm_state->cur_step = 15;

                //记录时间
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
                msg->SetExecStep(15);  //持料等待，则直接去取放料
            }else{//未对位，则先对位
                //计算X、Y的位置
                this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_corr_fine_x+
                        this->m_p_mech_arm_param->pos_corr_rough_x;
                this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_corr_fine_y+
                        this->m_p_mech_arm_param->pos_corr_rough_y;

                this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                        m_p_mech_arm_param->speed, false);  //绝对坐标
                this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                        m_p_mech_arm_param->speed, false);  //绝对坐标

                //对位气缸松开
                this->m_p_f_reg->correct_pos_x = 0;
                this->m_p_f_reg->correct_pos_y = 0;

                this->m_p_mech_arm_state->corr_index = 0;  //复位对位次数
                //记录时间
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

                printf("jump to step 7\n");
                this->m_p_mech_arm_state->cur_step = 7;
                msg->SetExecStep(7);  //去对位
            }
        }else if(this->m_p_mech_arm_state->cur_index[GRIND_MAIN_CHN] < this->m_p_mech_arm_state->total_count){
            printf("goto step 2\n");
            m_p_mech_arm_state->cur_step = 2;
            msg->IncreaseExecStep();   //先去取毛坯料
        }else if(this->m_p_mech_arm_state->work_material[GRIND_MAIN_CHN] == 1){//料盘已经无料可取，直接跳转去取成品
            //			printf("goto step 15: cur_index = %hu, fin=%hu\n", m_p_mech_arm_state->cur_index[GRIND_MAIN_CHN],
            //					m_p_mech_arm_state->cur_finished_count[GRIND_MAIN_CHN]);
            this->m_p_mech_arm_state->cur_step = 15;
            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            msg->SetExecStep(15);  //持料等待，则直接去取放料
        }else{//错误状态，即无成品可取，也无毛坯可取
            printf("error state!\n");
            return;
        }
    }else if(msg->GetExecStep() == 2){ //step-2 X/Y轴同时开始移动到取料位置
        //计算X、Y的目标位置
        uint8_t row = m_p_mech_arm_state->cur_index[cur_chn]/m_p_mech_arm_param->col_count;   //当前行
        uint8_t col = m_p_mech_arm_state->cur_index[cur_chn]%m_p_mech_arm_param->col_count;	  //当前列
        m_p_mech_arm_state->x_target_pos =
                m_p_mech_arm_param->pos_tray_base_x[cur_chn] + m_p_mech_arm_param->dis_tray_x * col;
        m_p_mech_arm_state->y_target_pos =
                m_p_mech_arm_param->pos_tray_base_y[cur_chn] - m_p_mech_arm_param->dis_tray_y * row;

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                m_p_mech_arm_param->speed, false);  //绝对坐标
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                m_p_mech_arm_param->speed, false);  //绝对坐标

        printf("goto step 3, x:%lf, y:%lf\n", m_p_mech_arm_state->x_target_pos, m_p_mech_arm_state->y_target_pos);
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 3){  //step-3  等待轴运行到位，左夹爪下（取料）
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005
                || fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //没有到位

        //左爪下
        this->m_p_f_reg->left_claw_down = 1;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 4\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 4){ //step-4  延时300ms等待左爪下到位，开真空
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 300000){ //延时300ms
            return;
        }

        //开左爪真空
        this->m_p_f_reg->left_claw_vac = 1;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 5\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 5){ //step-5  确认左爪真空信号，左爪上
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->left_claw_vac_check == 0 && time_elpase < wait_overtime)  //确认左爪真空信号，并且不超时
            return;

        //左爪上
        this->m_p_f_reg->left_claw_down = 0;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 6\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 6){ //step-6   确认左爪上到位，X、Y同时移动到粗对位位置
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 800000)  //延时800ms，防止误判吸真空信号
            return;

        if(this->m_p_g_reg->left_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待左爪上到位信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //机械臂上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //向HMI发送异常
            }
            return;
        }
        if(this->m_p_g_reg->left_claw_vac_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待左爪真空信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_FETCH_VAC;   //料盘取料失败，吸真空异常
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //向HMI发送异常
            }
            return;  //左爪真空信号异常
        }

        this->m_p_mech_arm_state->cur_index[cur_chn]++;  //当前取料序号加一

        //计算X、Y的位置
        this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_corr_fine_x+
                this->m_p_mech_arm_param->pos_corr_rough_x;
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_corr_fine_y+
                this->m_p_mech_arm_param->pos_corr_rough_y;

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                m_p_mech_arm_param->speed, false);  //绝对坐标
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                m_p_mech_arm_param->speed, false);  //绝对坐标

        //对位气缸松开
        this->m_p_f_reg->correct_pos_x = 0;
        this->m_p_f_reg->correct_pos_y = 0;

        this->m_p_mech_arm_state->corr_index = 0;  //复位对位次数

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 7， x=%lf, y=%lf\n", m_p_mech_arm_state->x_target_pos, m_p_mech_arm_state->y_target_pos);
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 7){  //step-7    等待轴到位，左爪下
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005
                || fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //没有到位


        if(this->m_p_g_reg->correct_pos_x_check == 0 || this->m_p_g_reg->correct_pos_y_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待对位气缸松开到位信号
                this->m_p_mech_arm_state->err_state = ERR_ARM_CORR_POS;   //对位失败，对位气缸不在位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //向HMI发送异常
            }
            return;   //对位气缸松开未完成
        }

        this->m_p_f_reg->left_claw_down = 1;  //左爪下

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 8\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 8){  //step-8 延时等待左爪下到位，关真空
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //延时500ms
            return;
        }

        this->m_p_f_reg->left_claw_vac = 0;  //关真空

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 9\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 9){	//step-9  延时800ms，左夹爪上
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < delay_time_vac){ //延时800ms
            return;
        }
        if(this->m_p_g_reg->left_claw_vac_check == 1 && time_elpase < wait_overtime)
            return;

        this->m_p_f_reg->left_claw_down = 0;  //左夹爪上

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 10\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 10){  //step-10 等待左夹爪上到位，对位气缸夹紧，X、Y轴移动到精对位位置
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //延时500ms
            return;
        }

        if(this->m_p_g_reg->left_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待左爪上到位信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //机械臂上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //向HMI发送异常
            }
            return;  //左爪上未到位
        }


        //对位气缸夹紧
        this->m_p_f_reg->correct_pos_x = 1;
        this->m_p_f_reg->correct_pos_y = 1;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        //计算X、Y轴目标位置
        this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_corr_fine_x;
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_corr_fine_y;

        //使用固定低速，防止抖动
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                2000, false);  //绝对坐标
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                2000, false);  //绝对坐标

        printf("goto step 11， x=%lf, y=%lf\n", m_p_mech_arm_state->x_target_pos, m_p_mech_arm_state->y_target_pos);
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 11){ //step-11 延时1000ms，并确认XY轴到位，左爪下
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 1000000){ //延时1000ms
            return;
        }

        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005
                || fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //没有到位

        if(++this->m_p_mech_arm_state->corr_index < this->m_p_mech_arm_param->corr_pos_count){//对位次数不够
            //对位气缸松开
            this->m_p_f_reg->correct_pos_x = 0;
            this->m_p_f_reg->correct_pos_y = 0;

            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

            printf("repos, jump to step 10\n");
            m_p_mech_arm_state->cur_step = 10;
            msg->SetExecStep(10);
            return;
        }

        m_p_mech_arm_state->corr_index = 0;

        this->m_p_f_reg->left_claw_down = 1;   //左爪下

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 12\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 12){  //step-12  延时500ms，开左爪真空
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //延时500ms
            return;
        }
        this->m_p_f_reg->left_claw_vac = 1;


        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);

        printf("goto step 13\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 13){ //step-13  延时200ms， 确认左爪真空信号，确认后再延时500ms，对位气缸松开
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        //		if(time_elpase < 200000){ //延时200ms
        //			return;
        //		}

        if(this->m_p_g_reg->left_claw_vac_check == 0 && time_elpase < wait_overtime){
            //记录时间2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
            return;   //左爪真空信号未确认
        }

        time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm_2.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm_2.tv_usec;
        if(time_elpase < 500000){ //延时500ms
            return;
        }

        //对位气缸松开
        this->m_p_f_reg->correct_pos_x = 0;
        this->m_p_f_reg->correct_pos_y = 0;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 14\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 14){ //step-14   确认对位气缸松开信号，左夹爪上
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->correct_pos_x_check == 0 || this->m_p_g_reg->correct_pos_y_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待对位气缸松开到位超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_CORR_POS;   //对位失败，对位气缸不在位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //向HMI发送异常
            }
            return;   //对位气缸松开未完成
        }

        this->m_p_f_reg->left_claw_down = 0;   //左爪上

        this->m_p_mech_arm_state->has_material = true;   //夹爪持料
        this->m_p_mech_arm_state->repos = true;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 15\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 15){ //step-15  延时1000ms后，确认左爪上到位，再次确认左爪真空信号，移动X轴到取料（送料）位置
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 1000000){ //延时1000ms
            return;
        }

        //更新工位状态，是否有料，工位吸真空检测有信号则工位有料，否则无料
        printf("refresh work vac: out=%hhu, in=%hhu\n", m_p_f_reg->work_vac,
               m_p_g_reg->work_vac_check);
        if(this->m_p_f_reg->work_vac == 1 && m_p_g_reg->work_vac_check == 1){
            this->m_p_mech_arm_state->work_material[GRIND_MAIN_CHN] = true;  //工位有料
        }else{
            this->m_p_mech_arm_state->work_material[GRIND_MAIN_CHN] = false;  //工位无料
        }

        if(this->m_p_g_reg->left_claw_up_check == 0 ||
                this->m_p_g_reg->right_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待左右爪上升到位超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //机械臂上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //向HMI发送异常
            }
            return;
        }


        if(this->m_p_mech_arm_state->has_material && this->m_p_g_reg->left_claw_vac_check == 0 ){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //吸真空异常
                this->m_p_mech_arm_state->err_state = ERR_ARM_CORR_VAC;   //对位失败，吸真空异常
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //向HMI发送异常
            }
            return;
        }

        //计算X轴目标位置
        if(this->m_p_mech_arm_state->work_material[cur_chn]){//加工位有料，取料位置
            this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_work_x[cur_chn] -
                    this->m_p_mech_arm_param->dis_mech_arm;
        }else{//加工位没有料，直接送料
            this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_work_x[cur_chn];
        }

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                m_p_mech_arm_param->speed, false);  //绝对坐标

        printf("goto step 16, X=%lf\n", this->m_p_mech_arm_state->x_target_pos);
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 16){ //step-16  等待X轴到位，确认加工位真空信号以及压紧气缸上升到位信号，移动Y轴到取料位
        //X轴是否到位
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005)
            return;   //没有到位

        double spd = m_p_mech_arm_param->speed;  //速度
        if(this->m_p_g_reg->work_up_check == 0){
            this->m_p_f_reg->work_hold = 0;  //压紧气缸上升
            return;
        }

        if(this->m_p_mech_arm_state->work_material[cur_chn] &&
                this->m_p_g_reg->work_vac_check == 0)  //加工位有料则确认真空信号
            return;

        if(!this->m_p_mech_arm_state->work_material[cur_chn]){
            spd = spd/2;
        }

        //计算Y轴目标位置
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_work_y[cur_chn];

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                spd, false);  //绝对坐标

        printf("goto step 17\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 17){ //step-17  等待Y轴到位，取料则右爪下，放料则左爪下
        //Y轴是否到位
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005){
            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            return;   //没有到位
        }


        if(this->m_p_mech_arm_state->work_material[cur_chn]){//取料
            this->m_p_f_reg->right_claw_down = 1;
        }else{
            struct timeval time_now;  //爪子停止有抖动，延时提高精度
            gettimeofday(&time_now, NULL);
            int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                    time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
            if(time_elpase < 1000000){ //延时1000ms
                return;
            }

            this->m_p_f_reg->left_claw_down = 1;  //放料
        }

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 18\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 18){ //step-18  延时300ms，取料开右爪真空，放料开加工位真空
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;


        if(this->m_p_mech_arm_state->work_material[cur_chn]){//取料，开右爪真空
            if(time_elpase < 300000){ //延时300ms
                return;
            }
            this->m_p_f_reg->right_claw_vac = 1;
        }else{
            this->m_p_f_reg->work_vac = 1;  //放料，开加工位真空

            //记录时间2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
        }

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 19\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 19){ //step-19  取料确认右真空信号后关加工位真空，放料确认加工位真空信号后关左爪真空
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//取料
            if(this->m_p_g_reg->right_claw_vac_check == 0 && time_elpase < wait_overtime)
                return;

            this->m_p_f_reg->work_vac = 0;   //关工位真空
        }else{//放料
            if(this->m_p_g_reg->work_vac_check == 0 && time_elpase < wait_overtime){
                //记录时间2
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
                return;
            }

            time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm_2.tv_sec)*1000000+
                    time_now.tv_usec-m_p_mech_arm_state->time_start_arm_2.tv_usec;
            if(time_elpase < work_vac_delay){ //工位吸真空延时后再关左爪真空
                return;
            }

            this->m_p_f_reg->left_claw_vac = 0; //关左爪真空
        }

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 20\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 20){ //step-20  取料确认加工位真空确认信号消失后延时300ms，放料确认左爪真空信号消失后延时200ms
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//取料
            if(this->m_p_g_reg->work_vac_check == 1 && time_elpase < wait_overtime)
                return;
        }else{//放料
            if(this->m_p_g_reg->left_claw_vac_check == 1 && time_elpase < wait_overtime){
                //				printf("等待左爪真空信号消失,left_claw_vac = %hhu\n", this->m_p_g_reg->left_claw_vac_check);
                return;
            }
        }

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 21\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 21){ //step-21   延时300ms后，取料则右爪上，放料则左爪上
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < delay_time_vac){ //延时300ms
            return;
        }

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//取料
            this->m_p_f_reg->right_claw_down = 0;

            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        }else{//放料
            this->m_p_f_reg->left_claw_down = 0;

            this->m_p_mech_arm_state->work_material[cur_chn] = true;    //加工位有料

            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


            //跳转28步
            printf("jump to step 28\n");
            m_p_mech_arm_state->cur_step = 28;
            msg->SetExecStep(28);
            return;
        }

        printf("goto step 22\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 22){ //step-22  确认右爪上升到位, 如果左爪有料则移动X轴去放料，如果无料则跳转26步
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->right_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待左右爪上升到位超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //机械臂上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //向HMI发送异常
            }
            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            return;
        }

        if(time_elpase < 600000){   //右爪上升到位后，再延时600ms检测右爪吸真空信号
            return;
        }

        if(this->m_p_g_reg->right_claw_vac_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待右爪吸真空超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_FETCH_VAC;   //工位取料失败，吸真空异常
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //向HMI发送异常
            }
            return;
        }

        this->m_p_mech_arm_state->work_material[cur_chn] = false;  //加工位无料
        this->m_p_mech_arm_state->has_finished_material = true;  //夹爪有成品

        if(this->m_p_mech_arm_state->has_material){  //左爪有料
            this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_work_x[cur_chn];

            this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                    m_p_mech_arm_param->speed/2, false);  //绝对坐标
        }else{ //左爪无料，跳转
            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


            //跳转28步
            printf("goto step 28\n");
            m_p_mech_arm_state->cur_step = 28;
            msg->SetExecStep(28);
            return;
        }

        printf("goto step 23\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 23){ //step-23   等待X轴到位，左爪下
        //X轴是否到位
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005){
            //记录时间
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            return;   //没有到位
        }

        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 1000000){ //延时1000ms
            return;
        }

        this->m_p_f_reg->left_claw_down = 1;

        //		//记录时间
        //		gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 24\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 24){ //step-24  延时300ms，开加工位真空
        //		struct timeval time_now;
        //		gettimeofday(&time_now, NULL);
        //		int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
        //				time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        //		if(time_elpase < 300000){ //延时300ms
        //			return;
        //		}

        this->m_p_f_reg->work_vac = 1;  //开工位真空

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
        //记录时间2
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);

        printf("goto step 25\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 25){ //step-25  确认加工位真空信号，关左爪真空
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->work_vac_check == 0 && time_elpase < wait_overtime){
            //记录时间2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
            return;
        }

        time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm_2.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm_2.tv_usec;
        if(time_elpase < work_vac_delay){ //工位吸真空延时，等待工位吸真空稳定
            return;
        }

        this->m_p_f_reg->left_claw_vac = 0;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        this->m_p_mech_arm_state->work_material[cur_chn] = true;    //加工位有料
        printf("goto step 26\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 26){ //step-26  等待左爪真空信号消失
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->left_claw_vac_check == 1 && time_elpase < wait_overtime)
            return;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 27\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 27){ //step-27  延时200ms， 左爪上升
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < delay_time_vac){ //破真空延时
            return;
        }

        this->m_p_f_reg->left_claw_down = 0;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 28\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 28){ //step-28  确认左右爪上升到位，移动Y轴到料盘基准位置
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //延时500ms
            return;
        }

        if(this->m_p_g_reg->left_claw_up_check == 0 || this->m_p_g_reg->right_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //等待左右爪上升到位超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //机械臂上升不到位
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //向HMI发送异常
            }
            return;
        }

        if(this->m_p_mech_arm_state->work_material[cur_chn] &&
                this->m_p_g_reg->work_vac_check == 0){ //检查工位吸真空信号
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){  //等待工位吸真空信号超时
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_SET_VAC;   //工位放料失败，吸真空异常
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //向HMI发送异常
            }
            return;
        }

        this->m_p_mech_arm_state->has_material = false;   //夹爪无料
        this->m_p_mech_arm_state->repos = false;

        //计算Y轴目标位置
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_tray_base_y[cur_chn];

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                m_p_mech_arm_param->speed, false);  //绝对坐标

        printf("goto step 29\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 29){ //step-29   等待Y轴到位
        //Y轴是否到位
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //没有到位

        this->m_p_mech_arm_state->msg = nullptr;

        //记录时间
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("succeed to execute M66, goto step 30\n");
        m_p_mech_arm_state->cur_step++;
        msg->SetExecStep(0);
    }
}

/**
 * @brief 处理从机M66指令，从机上下料请求
 */
void ChannelControl::ProcessGrindM66_slave(AuxMsg *msg){
    if(msg == nullptr)
        return;

    if(msg->GetExecStep() == 0){
        printf("start execute slave M66 cmd\n");
        printf("goto step 1\n");
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 1){  //step-1  检查加工位夹头是否已经上升到位
        if(this->m_p_g_reg->work_up_check == 0)
            return;

        this->m_p_f_reg->m66_req_right = 1;   //向左机请求上下料

        printf("goto step 2\n");
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 2){ //step-2 等待上下料结束信号
        if(this->m_p_g_reg->m66_runover_right == 0)
            return;

        printf("succeed to execute slave M66\n");
        this->m_p_f_reg->m66_req_right = 0;    //复位上下料请求
        msg->SetExecStep(0);
    }
}
#endif

/**
 * @brief 设置特殊插补模式，包括极坐标插补G12.1和磨削专用指令G12.2，以及取消指令G13.1
 */
void ChannelControl::SetMcSpecialIntpMode(uint16_t intp_type){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_SET_SPECIAL_INTP_MODE;
    cmd.data.data[0] = intp_type;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 设置五轴RTCP状态
 * @param cmd_type : 指令类型，5-5轴 6-G68.2 7-G53.1
 * @param switch_type ： 切换状态，0-取消 5-切换到5轴  6-切换到G68.2 7-切换到G53.1
 * @param h_val ：Z向刀具偏置，单位：um
 */
void ChannelControl::SetMcRtcpMode(uint16_t cmd_type, uint16_t switch_type, int32_t h_val){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_SET_MULTI_AXIS_MODE;
    cmd.data.data[0] = cmd_type;
    cmd.data.data[1] = switch_type;
    cmd.data.data[2] = h_val & 0xFFFF;
    cmd.data.data[3] = ((h_val >> 16) & 0xFFFF);

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

void ChannelControl::SetChnCurWorkPiece(int newCnt)
{
    int diff = newCnt - m_channel_status.workpiece_count_total;
    this->m_channel_status.workpiece_count = newCnt;
    this->m_channel_status.workpiece_count_total += diff;
    g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
    g_ptr_parm_manager->SetTotalWorkPiece(m_n_channel_index, m_channel_status.workpiece_count_total);
    SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);
}

/**
 * @brief 退出断点继续线程
 * @return true -- 成功退出   false--退出失败
 */
bool ChannelControl::CancelBreakContinueThread(){
    if(this->m_thread_breakcontinue > 0){//处于断点继续线程执行过程中，则退出断点继续线程
        //		void* thread_result = nullptr;
        //		int res = pthread_cancel(m_thread_breakcontinue);
        //		if (res != ERR_NONE) {
        //			printf("Failed to breakcontinue thread!\n");
        //		}
        //
        //		//	usleep(1000);
        //		res = pthread_join(m_thread_breakcontinue, &thread_result);//等待编译器线程退出完成
        //		if (res != ERR_NONE) {
        //			printf("breakcontinue thread join failed\n");
        //		}else
        //			printf("Succeed to cancel breakcontinue thread\n");
        //		m_thread_breakcontinue = 0;
        m_b_cancel_breakcontinue_thread = true;
        int count = 0;
        while(m_thread_breakcontinue > 0){
            if(count++ >= 5000)
                break;//超时退出
            usleep(100);
        }
        if(m_thread_breakcontinue > 0){
            printf("Failed to breakcontinue thread!");
            return false;
        }
    }
    return true;
}

/**
 * @brief 暂停G代码运行
 */
void ChannelControl::Pause(){

    if(m_channel_status.chn_work_mode == AUTO_MODE || m_channel_status.chn_work_mode == MDA_MODE){
        if(this->m_thread_breakcontinue > 0){//处于断点继续线程执行过程中，则退出断点继续线程
            this->CancelBreakContinueThread();
        }

        pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止
        //printf("locked 12\n");
        this->PauseRunGCode();

        pthread_mutex_unlock(&m_mutex_change_state);  //等待编译运行线程停止
        //printf("unlocked 12\n");
    }
    else{
        this->ManualMoveStop();  //手动停止
    }

}

/**
 * @brief 保存自动模式加工暂停时的状态
 * flag : 是否需要置位need_reload_flag， true需要置位，false不需要置位
 */
void ChannelControl::SaveAutoScene(bool flag){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "ChannelControl::SaveAutoScene(), machining_state=%hhu, FLAG=%hhu, cur_pos[%lf, %lf, %lf], last_output=%d\n", m_channel_status.machining_state,
                            m_scene_auto.need_reload_flag, m_channel_rt_status.cur_pos_machine.m_df_point[0], m_channel_rt_status.cur_pos_machine.m_df_point[1],
            m_channel_rt_status.cur_pos_machine.m_df_point[2], (int)this->m_p_last_output_msg);
    //	printf("ChannelControl::SaveAutoScene(), machining_state=%hhu, FLAG=%hhu, cur_pos[%lf, %lf, %lf], last_output=%d\n", m_channel_status.machining_state,
    //			m_scene_auto.need_reload_flag, m_channel_rt_status.cur_pos_machine.m_df_point[0], m_channel_rt_status.cur_pos_machine.m_df_point[1],
    //			m_channel_rt_status.cur_pos_machine.m_df_point[2], (int)this->m_p_last_output_msg);

    if(m_scene_auto.need_reload_flag)  //还未恢复状态时，不再保存
        return;

    m_scene_auto.machining_state = m_channel_status.machining_state;
    if(m_scene_auto.machining_state != MS_RUNNING &&
            m_scene_auto.machining_state != MS_PAUSING &&
            m_scene_auto.machining_state != MS_PAUSED){	//非加工状态无需保存状态
        m_scene_auto.need_reload_flag = false;
        printf("ChannelControl::SaveAutoScene(), false!machining_state=%hhu\n", m_scene_auto.machining_state);
        return;
    }

    m_scene_auto.cur_pos_machine = m_channel_rt_status.cur_pos_machine;
    memcpy(m_scene_auto.gmode, m_channel_status.gmode, kMaxGModeCount*sizeof(uint16_t));
    m_scene_auto.cur_d_code = m_channel_status.cur_d_code;
    m_scene_auto.cur_h_code = m_channel_status.cur_h_code;
    m_scene_auto.cur_tool = m_channel_status.cur_tool;
    m_scene_auto.preselect_tool_no = m_channel_status.preselect_tool_no;
    m_scene_auto.rated_feed = m_channel_status.rated_feed;
    m_scene_auto.rated_spindle_speed = m_channel_status.rated_spindle_speed;
    m_scene_auto.run_thread_state = m_n_run_thread_state;
    m_scene_auto.spindle_dir = m_channel_status.spindle_dir;
    m_scene_auto.spindle_mode = m_channel_status.spindle_mode;
    m_scene_auto.mc_mode_exec = m_mc_mode_exec;

    m_scene_auto.subprog_count = this->m_n_subprog_count;
    m_scene_auto.macroprog_count = this->m_n_macroprog_count;
    m_scene_auto.p_last_output_msg = this->m_p_last_output_msg;

    m_scene_auto.need_reload_flag = flag;
}

/**
 * @brief 恢复自动加工暂停时的状态
 */
void ChannelControl::ReloadAutoScene(){
    if(!m_scene_auto.need_reload_flag){	//就绪模式无需恢复状态
        return;
    }

    this->SetMachineState(m_scene_auto.machining_state);

    m_n_run_thread_state = (CompilerState)m_scene_auto.run_thread_state;
    m_channel_status.rated_feed = m_scene_auto.rated_feed;
    //	m_channel_status.rated_spindle_speed = m_scene_auto.rated_spindle_speed;   //改在breakcontinue流程中恢复主轴转速
    m_mc_mode_exec = m_scene_auto.mc_mode_exec;
    m_mc_mode_cur = m_scene_auto.mc_mode_exec;

    this->m_n_subprog_count = m_scene_auto.subprog_count;
    this->m_n_macroprog_count = m_scene_auto.macroprog_count;
    this->m_p_last_output_msg = m_scene_auto.p_last_output_msg;

}

/**
 * @brief 启动自动模式断点继续处理流程
 * @return true--启动成功    false--无需处理
 */
bool ChannelControl::StartBreakpointContinue(){

    //创建断点继续执行线程
    int res = 0;
    pthread_attr_t attr;
    struct sched_param param;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//
    param.__sched_priority = 35; //95;
    pthread_attr_setschedparam(&attr, &param);
    //	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
    //	if (res) {
    //		g_ptr_trace->PrintLog(LOG_ALARM, "断点继续处理线程设置线程继承模式失败！");
    //		m_error_code = ERR_INIT_BREAKCONTINUE;
    //		goto END;
    //	}

    res = pthread_create(&m_thread_breakcontinue, &attr,
                         ChannelControl::BreakContinueThread, this);    //开启G代码编译运行线程
    if (res != 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]断点继续处理线程创建失败!", m_n_channel_index);
        m_error_code = ERR_INIT_BREAKCONTINUE;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER, 0, m_n_channel_index);
        goto END;
    }

END:
    pthread_attr_destroy(&attr);
    return true;
}

/**
 * @brief 断点继续执行线程函数
 * @param void *args: ChannelControl对象指针
 */
void *ChannelControl::BreakContinueThread(void *args){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "Start Compiler::BreakContinueThread, threadid = %ld!", syscall(SYS_gettid));

    ChannelControl *p_channel_control = static_cast<ChannelControl *>(args);

    int res = ERR_NONE;
    res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
    if (res != ERR_NONE) {
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_CONTROL_SC, "Quit ChannelControl::BreakContinueThread with error 1!");
        pthread_exit((void*) EXIT_FAILURE);
    }
    res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
    if (res != ERR_NONE) {
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_CONTROL_SC, "Quit ChannelControl::BreakContinueThread with error 2!");
        pthread_exit((void*) EXIT_FAILURE);
    }


    res = p_channel_control->BreakContinueProcess();

    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "Exit ChannelControl::BreakContinueThread, threadid = %ld!", syscall(SYS_gettid));
    pthread_exit(NULL);
}

int ChannelControl::BreakContinueProcess(){
    int res = ERR_NONE;
    int i = 0;
    uint8_t phy_axis = 0;  //物理轴号
    uint8_t chn_axis_z = 0;    //Z轴通道轴号
    double safe_pos = 0;
    double *pos_cur=nullptr, *pos_scene = nullptr;

    this->m_n_breakcontinue_segment = 1;	//设置初始值


    while(!m_b_cancel_breakcontinue_thread){

        switch(m_n_breakcontinue_segment){
        case -1:	//失败
            m_scene_auto.need_reload_flag = false;		//复位标志
            m_b_cancel_breakcontinue_thread = true;
            printf("failed to exec break continue!\n");
            break;

        case 0:		//成功结束,启动加工
            if(this->m_mask_run_pmc){
                this->PausePmcAxis(NO_AXIS, false);  //继续运行PMC轴
            }else if(IsMcNeedStart()){
                this->StartMcIntepolate();
            }
            gettimeofday(&m_time_start_maching, nullptr);  //初始化启动时间
            m_time_remain = 0;                             //初始化剩余加工时间

            if(m_n_run_thread_state == IDLE || m_n_run_thread_state == PAUSE){
                m_n_run_thread_state = RUN;  //置为运行状态
            }

            this->m_p_f_reg->OP = 1;   //自动运行状态

            m_scene_auto.need_reload_flag = false;     //复位标志
            m_b_cancel_breakcontinue_thread = true;
            printf("succeed to exec break continue!, m_b_mc_need_start=%hhu\n", m_b_mc_need_start);
            break;
#ifdef USES_ADDITIONAL_PROGRAM
        case 1://开始执行前置程序
            this->CallAdditionalProgram(CONTINUE_START_ADD);

            m_n_breakcontinue_segment++;
            g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "go to step 2\n");
            break;
        case 2: //等待前置程序执行完成
            if(this->m_n_add_prog_type == NONE_ADD){
                g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "execute step 2, %d\n", m_scene_auto.p_last_output_msg);

                pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止

                m_channel_status.rated_feed = m_scene_auto.rated_feed;
                //	m_channel_status.rated_spindle_speed = m_scene_auto.rated_spindle_speed;
                m_mc_mode_exec = m_scene_auto.mc_mode_exec;
                m_mc_mode_cur = m_scene_auto.mc_mode_exec;
                this->m_n_subprog_count = m_scene_auto.subprog_count;
                this->m_n_macroprog_count = m_scene_auto.macroprog_count;
                this->m_p_last_output_msg = m_scene_auto.p_last_output_msg;

                memcpy(m_channel_status.gmode, m_scene_auto.gmode, kMaxGModeCount*sizeof(uint16_t));   //恢复模态

                //恢复自动模式
                m_p_output_msg_list = m_p_output_msg_list_auto;
                m_n_run_thread_state = (CompilerState)m_scene_auto.run_thread_state;
                this->m_p_compiler->SetMode(AUTO_COMPILER);	//编译器切换模式
                this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //切换输出队列
                pthread_mutex_unlock(&m_mutex_change_state);


                if(this->IsStepMode()){
                    this->SetMcStepMode(true);
                }

                if(m_scene_auto.need_reload_flag)
                    m_n_breakcontinue_segment++;
                else
                    m_n_breakcontinue_segment = 0;   //无需重载数据时，直接结束
                g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "go to step 3, %d\n", this->m_p_last_output_msg);
            }
            break;
        case 3: //检查当前刀号
            if(this->m_channel_status.cur_tool == m_scene_auto.cur_tool){	//刀号未变
                m_n_breakcontinue_segment = 10;
                g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "go to step 10\n");
                break;
            }

            //TODO 通知PMC进行换刀

            m_n_breakcontinue_segment++;
            g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "go to step 4\n");
            break;
        case 4:  //等待换刀结束
            //TODO  等待MC完成信号

            m_channel_status.cur_tool = m_scene_auto.cur_tool;
            this->SendModeChangToHmi(T_MODE);

#ifdef USES_WOOD_MACHINE
            this->SetMcToolLife();
#endif

            m_n_breakcontinue_segment = 10;
            g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "go to step 10\n");
            break;
#else
        case 1:		//检查当前刀号
            if(this->m_channel_status.cur_tool == m_scene_auto.cur_tool){	//刀号未变
                m_n_breakcontinue_segment = 10;
                break;
            }

            //TODO 通知PMC进行换刀

            m_n_breakcontinue_segment++;
            printf("goto step 2\n");
            break;
        case 2:		//等待换刀结束
            //TODO  等待MC完成信号

            m_channel_status.cur_tool = m_scene_auto.cur_tool;
            this->SendModeChangToHmi(T_MODE);

#ifdef USES_WOOD_MACHINE
            this->SetMcToolLife();
#endif

            m_n_breakcontinue_segment = 10;
            printf("goto step 10\n");
            break;
#endif
        case 10:   //检查刀偏设置，包括D、H
            this->m_channel_status.gmode[8] = m_scene_auto.gmode[8];	// 恢复G43/G44/G49模态
            this->m_channel_status.gmode[7] = m_scene_auto.gmode[7];	// 恢复G40/G41/G42模态
            if(m_channel_status.cur_h_code != m_scene_auto.cur_h_code){ //恢复H值
                this->m_channel_status.cur_h_code = m_scene_auto.cur_h_code;
                if(m_channel_status.gmode[8] == G49_CMD)
                    this->SetMcToolOffset(false);
                else
                    this->SetMcToolOffset(true);

                this->SendModeChangToHmi(H_MODE);
            }

            if(m_channel_status.cur_d_code != m_scene_auto.cur_d_code){
                m_channel_status.cur_d_code = m_scene_auto.cur_d_code;

                this->SendModeChangToHmi(D_MODE);
            }

            m_n_breakcontinue_segment = 20;
            printf("goto step 20\n");
            break;

        case 20:	//检查坐标系
            if(m_channel_status.gmode[14] != m_scene_auto.gmode[14]){
                this->m_channel_status.gmode[14] = m_scene_auto.gmode[14];

                //更新MC的坐标系数据
                this->SetMcCoord(true);
            }

            m_n_breakcontinue_segment = 30;
            printf("goto step 30\n");
            break;
        case 30:	//恢复其它模态
            memcpy(m_channel_status.gmode, m_scene_auto.gmode, 2*kMaxGModeCount);

            this->SendChnStatusChangeCmdToHmi(G_MODE);
            m_n_breakcontinue_segment = 60;
            // lidianqiang:取消Z轴抬刀和下刀动作
            // m_n_breakcontinue_segment = 40;
            printf("goto step 40\n");
            break;
        case 40: 	//恢复主轴状态
            //lidianqiang:主轴恢复与当前主轴功能有冲突，暂不考虑主轴恢复的问题
            //			if(m_channel_status.spindle_dir == m_scene_auto.spindle_dir &&
            //					m_channel_status.rated_spindle_speed == m_scene_auto.rated_spindle_speed &&
            //					m_channel_status.spindle_mode == m_scene_auto.spindle_mode){
            //				m_n_breakcontinue_segment = 50;
            //				printf("goto step 50\n");
            //				break;
            //			}

            //			this->SpindleOut(m_scene_auto.spindle_dir, m_scene_auto.rated_spindle_speed);
            m_n_breakcontinue_segment++;
            printf("goto step 41\n");
            break;
        case 41:	//等待主轴转速到达
            //lidianqiang:主轴恢复与当前主轴功能有冲突，暂不考虑主轴恢复的问题
            //			if(labs(m_channel_rt_status.spindle_cur_speed-(labs(m_channel_status.rated_spindle_speed)*m_channel_status.spindle_dir*m_channel_status.spindle_ratio/100)) > (labs(m_channel_status.rated_spindle_speed)*0.05)){
            //				printf("cur_speed = %d, rated=%d, dir=%hhd, ratio=%d\n", m_channel_rt_status.spindle_cur_speed, m_channel_status.rated_spindle_speed,
            //						m_channel_status.spindle_dir, m_channel_status.spindle_ratio);
            //				break;
            //			}

            m_n_breakcontinue_segment = 50;
            printf("goto step 50\n");
            break;
        case 50:	//回到断点位置，首先Z轴移动到安全高度(如果没有其他轴需要移动，则不需要到安全高度)
            if(this->m_channel_rt_status.cur_pos_machine == m_scene_auto.cur_pos_machine){
                m_n_breakcontinue_segment = 0;
                printf("axis not move !!!!\n");
                break;
            }
            else{
                printf("cur pos : %f, %f, %f, %f, scene pos: %f, %f, %f, %f\n", m_channel_rt_status.cur_pos_machine.m_df_point[0], m_channel_rt_status.cur_pos_machine.m_df_point[1],
                        m_channel_rt_status.cur_pos_machine.m_df_point[2], m_channel_rt_status.cur_pos_machine.m_df_point[3], m_scene_auto.cur_pos_machine.m_df_point[0],
                        m_scene_auto.cur_pos_machine.m_df_point[1], m_scene_auto.cur_pos_machine.m_df_point[2], m_scene_auto.cur_pos_machine.m_df_point[3]);
            }
            chn_axis_z = this->GetChnAxisFromName(AXIS_NAME_Z);
            phy_axis = m_p_channel_config->chn_axis_phy[chn_axis_z];
            if(phy_axis > 0){  //
                safe_pos = m_p_axis_config[phy_axis-1].axis_home_pos[1];
            }

            if(m_channel_rt_status.cur_pos_machine.GetAxisValue(chn_axis_z)< safe_pos)  //当Z轴低于安全高度时，先抬到安全高度
                this->ManualMove(chn_axis_z, safe_pos, 2000);		//TODO 使用机械坐标系
            else{
                m_n_breakcontinue_segment = 60;
                break;
            }

            m_n_breakcontinue_segment++;
            printf("goto step 51\n");
            break;
        case 51:	//等待Z轴到位
            chn_axis_z = this->GetChnAxisFromName(AXIS_NAME_Z);
            phy_axis = m_p_channel_config->chn_axis_phy[chn_axis_z];
            if(phy_axis > 0){  //
                safe_pos = m_p_axis_config[phy_axis-1].axis_home_pos[1];
            }
            if(fabs(this->m_channel_rt_status.cur_pos_machine.m_df_point[chn_axis_z]-safe_pos) < 5e-3){ //到位
                m_n_breakcontinue_segment = 60;
                printf("goto step 60\n");
                break;
            }else{
                //		this->ManualMove(chn_axis_z, safe_pos, 1000);		//TODO 使用机械坐标系
                usleep(5000);
            }
            break;
        case 60:{	//其它伺服轴回到断点位置
            pos_cur = m_channel_rt_status.cur_pos_machine.m_df_point;
            pos_scene = m_scene_auto.cur_pos_machine.m_df_point;
            for(i = 0; i < this->m_p_channel_config->chn_axis_count; i++, pos_cur++, pos_scene++){
                phy_axis = m_p_channel_config->chn_axis_phy[i]; // phy_axis = m_channel_status.cur_chn_axis_phy[i];
                if(phy_axis > 0){
                    if(m_p_axis_config[phy_axis-1].axis_linear_type == LINE_AXIS_Z ||
                            m_p_axis_config[phy_axis-1].axis_linear_type == LINE_AXIS_PZ ||
                            m_p_axis_config[phy_axis-1].axis_type == AXIS_SPINDLE)    //跳过Z轴或主轴
                        continue;
                }

                if(fabs(*pos_cur - *pos_scene) >= 1e-3)
                    this->ManualMove(i, *pos_scene, 2000);		//TODO 使用机械坐标系
            }
            m_n_breakcontinue_segment = 61;
            printf("goto step 61\n");
            break;
        }
        case 61:{	//等待伺服轴运动到位
            pos_cur = m_channel_rt_status.cur_pos_machine.m_df_point;
            pos_scene = m_scene_auto.cur_pos_machine.m_df_point;
            for(i = 0; i < this->m_p_channel_config->chn_axis_count; i++, pos_cur++, pos_scene++){
                phy_axis = m_p_channel_config->chn_axis_phy[i];  // phy_axis = m_channel_status.cur_chn_axis_phy[i];
                if(phy_axis > 0){
                    if(m_p_axis_config[phy_axis-1].axis_linear_type == LINE_AXIS_Z ||
                            m_p_axis_config[phy_axis-1].axis_linear_type == LINE_AXIS_PZ ||
                            m_p_axis_config[phy_axis-1].axis_type == AXIS_SPINDLE)    //跳过Z轴或主轴
                        continue;
                }

                if(fabs(*pos_cur - *pos_scene) >= 5e-3){ //未到位
                    //		printf("%d axis not arrived, %lf, %lf, %d\n", i, *pos_cur, *pos_scene, m_p_channel_config->chn_axis_count);
                    break;
                }
            }
            if(i == m_p_channel_config->chn_axis_count){ //所有轴都到位
                m_n_breakcontinue_segment++;
                printf("goto step 62\n");
            }
            else
                usleep(5000);

            break;
        }
        case 62:  	//Z轴复位到断点位置
            //			chn_axis_z = this->GetChnAxisFromName(AXIS_NAME_Z);

            //			this->ManualMove(chn_axis_z, m_scene_auto.cur_pos_machine.m_df_point[chn_axis_z], 2000);		//TODO 使用机械坐标系

            //			m_n_breakcontinue_segment++;
            //			printf("goto step 63\n");
            // lidianqiang:取消Z轴抬刀和下刀动作
            m_n_breakcontinue_segment = 0;
            break;
        case 63:	//等待Z轴到位
            if(fabs(m_channel_rt_status.cur_pos_machine.m_df_point[2] - m_scene_auto.cur_pos_machine.m_df_point[2]) < 1e-3){ //到位
                m_n_breakcontinue_segment = 0;
                printf("goto step 0\n");
            }else
                usleep(5000);

            break;
        }
        usleep(1000);
    }
#ifdef USES_ADDITIONAL_PROGRAM
    if(m_n_breakcontinue_segment == 1 || m_n_breakcontinue_segment == 2){   //取消断点继续前置程序运行

        this->StopCompilerRun();
        //向MC模块发送暂停指令
        this->PauseMc();

        this->m_p_output_msg_list->Clear();

        //恢复自动模式
        pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止
        m_p_output_msg_list = m_p_output_msg_list_auto;
        m_n_run_thread_state = PAUSE;
        this->m_p_compiler->SetMode(AUTO_COMPILER);	//编译器切换模式
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //切换输出队列
        pthread_mutex_unlock(&m_mutex_change_state);

        m_n_add_prog_type = NONE_ADD;   //复位附加程序执行状态

    }else
#endif
        if(m_n_breakcontinue_segment != 0 && m_n_breakcontinue_segment != -1){
            this->ManualMoveStop();
        }

    m_b_cancel_breakcontinue_thread = false;
    m_thread_breakcontinue = 0;
    return res;
}

/**
 * @brief 指定轴回参考点
 * @param axis_mask : 指定哪些通道轴回零
 * @return
 */
#ifdef USES_WUXI_BLOOD_CHECK
bool ChannelControl::ReturnRef(uint8_t axis_mask){
    SCAxisConfig *axis_config = nullptr;
    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i) && m_p_channel_config->chn_axis_phy[i] != 0){ // if(axis_mask & (0x01<<i) && m_channel_status.cur_chn_axis_phy[i] != 0){
            axis_config = &m_p_axis_config[m_p_channel_config->chn_axis_phy[i]-1]; //axis_config = &m_p_axis_config[m_channel_status.cur_chn_axis_phy[i]-1];
            this->ManualMove(i, axis_config->axis_home_pos[0], axis_config->ret_ref_speed, false);   //回到机械坐标系下的第一参考点
        }
    }
    return true;
}
#endif

/**
 * @brief 回参考点执行函数
 * @param flag : true--所有通道轴回零    false--当前轴回零
 */
void ChannelControl::ProcessHmiReturnRefCmd(bool flag){
    printf("ChannelControl::ProcessHmiReturnRefCmd, flag = %hhu\n", flag);
    if(flag){//所有通道轴回零
        uint8_t phy_axis = 0;
        for(int i=0; i < this->m_p_channel_config->chn_axis_count; i++){
            phy_axis = this->GetPhyAxis(i);
            if(phy_axis != 0xff){
                this->m_p_channel_engine->SetRetRefMask(phy_axis);
            }
        }

    }else{ //当前轴回零
        uint8_t cur_phy = this->GetCurPhyAxis();
        if(cur_phy != 0){
            this->m_p_channel_engine->SetRetRefMask(cur_phy-1);
        }
    }
}

/**
 * @brief 设置轴名称扩展下标使能
 * @param flag : 允许轴名称下标
 */
void ChannelControl::SetAxisNameEx(bool flag){
    this->m_p_compiler->SetAxisNameEx(flag);
}

#ifdef USES_WUXI_BLOOD_CHECK
/**
 * @brief 设置回零标志
 */
void ChannelControl::SetRetHomeFlag(){
    if(!m_b_returnning_home){
        m_b_returnning_home = true;
        m_n_ret_home_step=0;
    }

}

/**
 * @brief 回参考点   无锡项目定制
 * @return
 */
bool ChannelControl::ReturnHome(){
    uint8_t phy_axis = 0;
    struct timeval cur_time;
    uint64_t time_elpase = 0;
    double wait_time = 0;
    bool init = false;
    switch(m_n_ret_home_step){
    case 0: //first step
        this->ReturnRef(0x04);   //Z轴回零
        this->m_n_ret_home_step++;
        break;
    case 1://second step    等待Z轴到位
        phy_axis = this->m_p_channel_config->chn_axis_phy[2]; // phy_axis = this->m_channel_status.cur_chn_axis_phy[2];
        if(phy_axis > 0){
            if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(2) - m_p_axis_config[phy_axis].axis_home_pos[0]) < 1e-3){ //到位
                m_n_ret_home_step++;
            }
        }
        break;
    case 2://third step   XY轴回零
        this->ReturnRef(0x03);
        this->m_n_ret_home_step++;
        break;
    case 3://fourth step 等待XY到位
        phy_axis = this->m_p_channel_config->chn_axis_phy[0];  //X轴 // phy_axis = this->m_channel_status.cur_chn_axis_phy[0];  //X轴
        if(phy_axis > 0){
            if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(0) - m_p_axis_config[phy_axis].axis_home_pos[0]) >= 1e-3){ //未到位
                break;
            }
        }
        phy_axis = this->m_p_channel_config->chn_axis_phy[1];   //Y轴 // phy_axis = this->m_channel_status.cur_chn_axis_phy[1];   //Y轴
        if(phy_axis > 0){
            if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(1) - m_p_axis_config[phy_axis].axis_home_pos[0]) >= 1e-3){ //未到位
                break;
            }
        }

        gettimeofday(&m_time_start_wait, nullptr);  //记录起始时间
        m_n_ret_home_step++;
        break;
    case 4: //fifth step 等待延时
        this->m_macro_variable.GetVarValue(510, wait_time, init);
        gettimeofday(&cur_time, nullptr);
        time_elpase = (cur_time.tv_sec-m_time_start_wait.tv_sec)*1000 + (cur_time.tv_usec - m_time_start_wait.tv_usec)/1000;  //ms
        if(time_elpase < wait_time)
            break;		//未到延时时间
        m_n_ret_home_step++;
        break;
    case 5: //sixth step  夹头复位
        //	this->ReturnRef(0x08);   //夹头轴回零
        this->m_n_ret_home_step++;
        break;
    case 6:
        phy_axis = this->m_p_channel_config->chn_axis_phy[3]; // phy_axis = this->m_channel_status.cur_chn_axis_phy[3];
        if(phy_axis > 0){
            if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(3) - m_p_axis_config[phy_axis].axis_home_pos[0]) < 1e-3){ //到位
                m_n_ret_home_step = 0;
                this->m_b_returnning_home = false;  //回零结束
            }
        }
        break;
    default:
        break;
    }
    return true;
}
#endif

/**
 * @brief 处理急停
 * @return
 */
bool ChannelControl::EmergencyStop(){

#ifdef USES_GRIND_MACHINE
    if(this->m_b_ret_safe_state){ //取消各轴复位动作
        this->m_b_ret_safe_state = false;
        this->m_n_ret_safe_step = 0;
    }


    //复位IO输出
    this->m_p_f_reg->work_hold = 0;
    this->m_p_f_reg->work_vac = 0;
    this->m_p_f_reg->left_claw_down = 0;
    this->m_p_f_reg->left_claw_vac = 0;
    this->m_p_f_reg->right_claw_down = 0;
    this->m_p_f_reg->right_claw_vac = 0;
    this->m_p_f_reg->correct_pos_x = 0;
    this->m_p_f_reg->correct_pos_y = 0;

    if(this->m_p_pmc_reg->GReg().bits[0].main_chn == GRIND_MAIN_CHN)
        this->m_p_mech_arm_state->work_material[GRIND_MAIN_CHN] = false;
    //	else
    //		this->m_p_mech_arm_state->work_material[GRIND_SLAVE_CHN] = false;   //从机的急停没有接到主机

    this->m_p_mech_arm_state->has_finished_material = false;
    this->m_p_mech_arm_state->has_material = false;
    this->m_p_mech_arm_state->repos = false;
#endif

    this->m_p_f_reg->RST = 1;

    //停止编译
    pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止
    //printf("locked 15\n");
    m_n_run_thread_state = STOP;
    pthread_mutex_unlock(&m_mutex_change_state);
    //printf("unlocked 15\n");
    //停止MC执行

    this->PauseMc();

    if(this->m_thread_breakcontinue > 0){//处于断点继续线程执行过程中，则退出断点继续线程
        this->CancelBreakContinueThread();
    }

    m_p_spindle->EStop();

    // 取消刚性攻丝状态
    //if(m_p_spindle->isTapEnable())
        //m_p_spindle->CancelRigidTap();

    //	if(this->m_n_M29_flag){  //主轴恢复速度控制
    //		this->ProcessM29Reset();
    //	}

    //更新状态
    uint8_t state = MS_WARNING;   //告警状态
#ifdef USES_WOOD_MACHINE
    state = MS_STOPPING;   //停止中
#endif
    this->SetMachineState(state);  //更新通道状态

    //复位各轴回参考点标志，只有增量式编码器的轴需要复位
    for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(m_p_axis_config[m_p_channel_config->chn_axis_phy[i]-1].axis_interface != VIRTUAL_AXIS		//非虚拟轴
                && m_p_axis_config[m_p_channel_config->chn_axis_phy[i]-1].axis_type != AXIS_SPINDLE				//非主轴
                && m_p_axis_config[m_p_channel_config->chn_axis_phy[i]-1].feedback_mode == INCREMENTAL_ENCODER)    //增量式编码器
        {
            m_channel_status.returned_to_ref_point &= ~(0x01<<i);
        }
    }
    this->SendChnStatusChangeCmdToHmi(REF_POINT_FLAG);


    //OP信号复位
    this->m_p_f_reg->OP = 0;
    //this->m_p_f_reg->SPL = 0;

    return true;

}

/**
 * @brief 返回是否还有待运行的指令
 * @return
 */
bool ChannelControl::IsOutputMsgRunover(){
    bool res = false;
    if(this->m_channel_status.chn_work_mode == AUTO_MODE){
        if(m_p_last_output_msg == nullptr || m_p_last_output_msg == this->m_p_output_msg_list->TailNode())
            res = true;
    }else{
        res = this->m_p_output_msg_list->IsEmpty();
    }

    return res;
}

/**
 * @brief 设置通道轴回参考点标志
 * @param chn_axis ： 通道轴号，从0开始
 * @param flag  ： true--设置     false--取消
 */
void ChannelControl::SetRefPointFlag(uint8_t chn_axis, bool flag){
    if(flag){
        this->m_channel_status.returned_to_ref_point |= (0x01<<chn_axis);
    }else
        this->m_channel_status.returned_to_ref_point &= ~(0x01<<chn_axis);

    this->SendChnStatusChangeCmdToHmi(REF_POINT_FLAG);

    //设置通道轴的软限位检查标志
    if(flag){
        this->SetChnAxisSoftLimit(chn_axis);
    }else{
        this->CloseChnAxisSoftLimit(chn_axis);   //未回参考点状态，则强制关闭软限位
    }
}

/**
 * @brief 将M代码发送给PMC
 * @param m_code : M指令
 * @param m_index ： M指令顺序号，即同行的第几个M指令, 值范围：0-15
 */
void ChannelControl::SendMCodeToPmc(uint32_t m_code, uint8_t m_index){
    switch(m_index){
    case 0:
        m_p_f_reg->mcode_0 = (m_code&0xFF);
        m_p_f_reg->mcode_1 = ((m_code>>8)&0xFF);
        m_p_f_reg->mcode_2 = ((m_code>>16)&0xFF);
        m_p_f_reg->mcode_3 = ((m_code>>24)&0xFF);
        break;
    case 1:
        m_p_f_reg->mcode_20 = (m_code&0xFF);
        m_p_f_reg->mcode_21 = ((m_code>>8)&0xFF);
        break;
    case 2:
        m_p_f_reg->mcode_30 = (m_code&0xFF);
        m_p_f_reg->mcode_31 = ((m_code>>8)&0xFF);
        break;
    case 3:
        m_p_f_reg->mcode_40 = (m_code&0xFF);
        m_p_f_reg->mcode_41 = ((m_code>>8)&0xFF);
        break;
    case 4:
        m_p_f_reg->mcode_50 = (m_code&0xFF);
        m_p_f_reg->mcode_51 = ((m_code>>8)&0xFF);
        break;
    case 5:
        m_p_f_reg->mcode_60 = (m_code&0xFF);
        m_p_f_reg->mcode_61 = ((m_code>>8)&0xFF);
        break;
    case 6:
        m_p_f_reg->mcode_70 = (m_code&0xFF);
        m_p_f_reg->mcode_71 = ((m_code>>8)&0xFF);
        break;
    case 7:
        m_p_f_reg->mcode_80 = (m_code&0xFF);
        m_p_f_reg->mcode_81 = ((m_code>>8)&0xFF);
        break;
    case 8:
        m_p_f_reg->mcode_90 = (m_code&0xFF);
        m_p_f_reg->mcode_91 = ((m_code>>8)&0xFF);
        break;
    case 9:
        m_p_f_reg->mcode_100 = (m_code&0xFF);
        m_p_f_reg->mcode_101 = ((m_code>>8)&0xFF);
        break;
    case 10:
        m_p_f_reg->mcode_110 = (m_code&0xFF);
        m_p_f_reg->mcode_111 = ((m_code>>8)&0xFF);
        break;
    case 11:
        m_p_f_reg->mcode_120 = (m_code&0xFF);
        m_p_f_reg->mcode_121 = ((m_code>>8)&0xFF);
        break;
    case 12:
        m_p_f_reg->mcode_130 = (m_code&0xFF);
        m_p_f_reg->mcode_131 = ((m_code>>8)&0xFF);
        break;
    case 13:
        m_p_f_reg->mcode_140 = (m_code&0xFF);
        m_p_f_reg->mcode_141 = ((m_code>>8)&0xFF);
        break;
    case 14:
        m_p_f_reg->mcode_150 = (m_code&0xFF);
        m_p_f_reg->mcode_151 = ((m_code>>8)&0xFF);
        break;
    case 15:
        m_p_f_reg->mcode_160 = (m_code&0xFF);
        m_p_f_reg->mcode_161 = ((m_code>>8)&0xFF);
        break;
    default:
        printf("Invalid M code index[%hhu]!\n", m_index);
        break;
    }

}

/**
 * @brief 设置MFn选通信号
 * @param index : M指令顺序号，0~15
 * @param value ： true--置一    false--置零
 */
void ChannelControl::SetMFSig(uint8_t index, bool value){
    if(index == 0){
        this->m_p_f_reg->MF = value?1:0;
    }else if(index == 1){
        this->m_p_f_reg->MF2 = value?1:0;
    }else if(index == 2){
        this->m_p_f_reg->MF3 = value?1:0;
    }else if(index == 3){
        this->m_p_f_reg->MF4 = value?1:0;
    }else if(index == 4){
        this->m_p_f_reg->MF5 = value?1:0;
    }else if(index == 5){
        this->m_p_f_reg->MF6 = value?1:0;
    }else if(index == 6){
        this->m_p_f_reg->MF7 = value?1:0;
    }else if(index == 7){
        this->m_p_f_reg->MF8 = value?1:0;
    }else if(index == 8){
        this->m_p_f_reg->MF9 = value?1:0;
    }else if(index == 9){
        this->m_p_f_reg->MF10 = value?1:0;
    }else if(index == 10){
        this->m_p_f_reg->MF11 = value?1:0;
    }else if(index == 11){
        this->m_p_f_reg->MF12 = value?1:0;
    }else if(index == 12){
        this->m_p_f_reg->MF13 = value?1:0;
    }else if(index == 13){
        this->m_p_f_reg->MF14 = value?1:0;
    }else if(index == 14){
        this->m_p_f_reg->MF15 = value?1:0;
    }else if(index == 15){
        this->m_p_f_reg->MF16 = value?1:0;
    }
}

/**
 * @brief 返回指定序号的MFIN信号
 * @param index : M指令顺序号，0~15
 * @return true--有信号    false--无信号
 */
bool ChannelControl::GetMFINSig(uint8_t index){
    switch(index){
    case 0:
        return this->m_p_g_reg->MFIN?true:false;
    case 1:
        return this->m_p_g_reg->MFIN2?true:false;
    case 2:
        return this->m_p_g_reg->MFIN3?true:false;
    case 3:
        return this->m_p_g_reg->MFIN4?true:false;
    case 4:
        return this->m_p_g_reg->MFIN5?true:false;
    case 5:
        return this->m_p_g_reg->MFIN6?true:false;
    case 6:
        return this->m_p_g_reg->MFIN7?true:false;
    case 7:
        return this->m_p_g_reg->MFIN8?true:false;
    case 8:
        return this->m_p_g_reg->MFIN9?true:false;
    case 9:
        return this->m_p_g_reg->MFIN10?true:false;
    case 10:
        return this->m_p_g_reg->MFIN11?true:false;
    case 11:
        return this->m_p_g_reg->MFIN12?true:false;
    case 12:
        return this->m_p_g_reg->MFIN13?true:false;
    case 13:
        return this->m_p_g_reg->MFIN14?true:false;
    case 14:
        return this->m_p_g_reg->MFIN15?true:false;
    case 15:
        return this->m_p_g_reg->MFIN16?true:false;
    default:
        return false;
    }
    return false;
}

/**
 * @brief 返回指定序号的MExc信号
 * @param index : M指令顺序号，0~15
 * @return   true--有信号    false--无信号
 */
bool ChannelControl::GetMExcSig(uint8_t index){
    switch(index){
    case 0:
        return this->m_p_g_reg->MEXC1?true:false;
    case 1:
        return this->m_p_g_reg->MEXC2?true:false;
    case 2:
        return this->m_p_g_reg->MEXC3?true:false;
    case 3:
        return this->m_p_g_reg->MEXC4?true:false;
    case 4:
        return this->m_p_g_reg->MEXC5?true:false;
    case 5:
        return this->m_p_g_reg->MEXC6?true:false;
    case 6:
        return this->m_p_g_reg->MEXC7?true:false;
    case 7:
        return this->m_p_g_reg->MEXC8?true:false;
    case 8:
        return this->m_p_g_reg->MEXC9?true:false;
    case 9:
        return this->m_p_g_reg->MEXC10?true:false;
    case 10:
        return this->m_p_g_reg->MEXC11?true:false;
    case 11:
        return this->m_p_g_reg->MEXC12?true:false;
    case 12:
        return this->m_p_g_reg->MEXC13?true:false;
    case 13:
        return this->m_p_g_reg->MEXC14?true:false;
    case 14:
        return this->m_p_g_reg->MEXC15?true:false;
    case 15:
        return this->m_p_g_reg->MEXC16?true:false;
    default:
        return false;
    }
    return false;
}

/**
 * @brief 将T指令发送至PMC寄存器
 * @param t_code ： T指令值，刀具号
 * @param index : T指令顺序号，即同行的第几个T指令, 值范围：0-15
 */
void ChannelControl::SendTCodeToPmc(uint32_t t_code, uint8_t index){
    switch(index){
    case 0:
        m_p_f_reg->tcode_0 = (t_code&0xFF);
        m_p_f_reg->tcode_1 = ((t_code>>8)&0xFF);
        m_p_f_reg->tcode_2 = ((t_code>>16)&0xFF);
        m_p_f_reg->tcode_3 = ((t_code>>24)&0xFF);
        break;
    case 1:
        m_p_f_reg->tcode_20 = (t_code&0xFF);
        m_p_f_reg->tcode_21 = ((t_code>>8)&0xFF);
        break;
    case 2:
        m_p_f_reg->tcode_30 = (t_code&0xFF);
        m_p_f_reg->tcode_31 = ((t_code>>8)&0xFF);
        break;
    case 3:
        m_p_f_reg->tcode_40 = (t_code&0xFF);
        m_p_f_reg->tcode_41 = ((t_code>>8)&0xFF);
        break;
    case 4:
        m_p_f_reg->tcode_50 = (t_code&0xFF);
        m_p_f_reg->tcode_51 = ((t_code>>8)&0xFF);
        break;
    case 5:
        m_p_f_reg->tcode_60 = (t_code&0xFF);
        m_p_f_reg->tcode_61 = ((t_code>>8)&0xFF);
        break;
    case 6:
        m_p_f_reg->tcode_70 = (t_code&0xFF);
        m_p_f_reg->tcode_71 = ((t_code>>8)&0xFF);
        break;
    case 7:
        m_p_f_reg->tcode_80 = (t_code&0xFF);
        m_p_f_reg->tcode_81 = ((t_code>>8)&0xFF);
        break;
    case 8:
        m_p_f_reg->tcode_90 = (t_code&0xFF);
        m_p_f_reg->tcode_91 = ((t_code>>8)&0xFF);
        break;
    case 9:
        m_p_f_reg->tcode_100 = (t_code&0xFF);
        m_p_f_reg->tcode_101 = ((t_code>>8)&0xFF);
        break;
    case 10:
        m_p_f_reg->tcode_110 = (t_code&0xFF);
        m_p_f_reg->tcode_111 = ((t_code>>8)&0xFF);
        break;
    case 11:
        m_p_f_reg->tcode_120 = (t_code&0xFF);
        m_p_f_reg->tcode_121 = ((t_code>>8)&0xFF);
        break;
    case 12:
        m_p_f_reg->tcode_130 = (t_code&0xFF);
        m_p_f_reg->tcode_131 = ((t_code>>8)&0xFF);
        break;
    case 13:
        m_p_f_reg->tcode_140 = (t_code&0xFF);
        m_p_f_reg->tcode_141 = ((t_code>>8)&0xFF);
        break;
    case 14:
        m_p_f_reg->tcode_150 = (t_code&0xFF);
        m_p_f_reg->tcode_151 = ((t_code>>8)&0xFF);
        break;
    case 15:
        m_p_f_reg->tcode_160 = (t_code&0xFF);
        m_p_f_reg->tcode_161 = ((t_code>>8)&0xFF);
        break;
    default:
        printf("Invalid T code index[%hhu]!\n", index);
        break;
    }
}

/**
 * @brief 设置TF选通信号
 * @param index :  T指令顺序号， 0~15
 * @param value ： true--置一    false--置零
 */
void ChannelControl::SetTFSig(uint8_t index, bool value){
    switch(index){
    case 0:
        m_p_f_reg->TF = value?1:0;
        break;
    case 1:
        m_p_f_reg->TF2 = value?1:0;
        break;
    case 2:
        m_p_f_reg->TF3 = value?1:0;
        break;
    case 3:
        m_p_f_reg->TF4 = value?1:0;
        break;
    case 4:
        m_p_f_reg->TF5 = value?1:0;
        break;
    case 5:
        m_p_f_reg->TF6 = value?1:0;
        break;
    case 6:
        m_p_f_reg->TF7 = value?1:0;
        break;
    case 7:
        m_p_f_reg->TF8 = value?1:0;
        break;
    case 8:
        m_p_f_reg->TF9 = value?1:0;
        break;
    case 9:
        m_p_f_reg->TF10 = value?1:0;
        break;
    case 10:
        m_p_f_reg->TF11 = value?1:0;
        break;
    case 11:
        m_p_f_reg->TF12 = value?1:0;
        break;
    case 12:
        m_p_f_reg->TF13 = value?1:0;
        break;
    case 13:
        m_p_f_reg->TF14 = value?1:0;
        break;
    case 14:
        m_p_f_reg->TF15 = value?1:0;
        break;
    case 15:
        m_p_f_reg->TF16 = value?1:0;
        break;
    default:
        break;
    }

}

/**
 * @brief 返回指定序号的TFIN信号
 * @param index : T指令顺序号， 0~15
 * @return true--有信号    false--无信号
 */
bool ChannelControl::GetTFINSig(uint8_t index){
    switch(index){
    case 0:
        return this->m_p_g_reg->TFIN?true:false;
    case 1:
        return this->m_p_g_reg->TFIN2?true:false;
    case 2:
        return this->m_p_g_reg->TFIN3?true:false;
    case 3:
        return this->m_p_g_reg->TFIN4?true:false;
    case 4:
        return this->m_p_g_reg->TFIN5?true:false;
    case 5:
        return this->m_p_g_reg->TFIN6?true:false;
    case 6:
        return this->m_p_g_reg->TFIN7?true:false;
    case 7:
        return this->m_p_g_reg->TFIN8?true:false;
    case 8:
        return this->m_p_g_reg->TFIN9?true:false;
    case 9:
        return this->m_p_g_reg->TFIN10?true:false;
    case 10:
        return this->m_p_g_reg->TFIN11?true:false;
    case 11:
        return this->m_p_g_reg->TFIN12?true:false;
    case 12:
        return this->m_p_g_reg->TFIN13?true:false;
    case 13:
        return this->m_p_g_reg->TFIN14?true:false;
    case 14:
        return this->m_p_g_reg->TFIN15?true:false;
    case 15:
        return this->m_p_g_reg->TFIN16?true:false;
    default:
        return false;
    }
    return false;
}

/**
 * @brief 将主轴转速转换为电平值
 * @param speed : 主轴转速
 */
void ChannelControl::SpindleSpeedDaOut(int32_t speed){
    if(this->m_n_spindle_count == 0){
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "通道[%hhu]没有配置主轴\n", m_n_channel_index);
        return;//没有主轴
    }
    //TODO 当前先用第一个主轴来判断
    uint8_t phy_spd = this->m_spd_axis_phy[0];
    SCAxisConfig &spd_config = m_p_axis_config[phy_spd-1];

    if(spd_config.spd_vctrl_mode == 0){//控制模式为禁止
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "主轴[%hhu]电压控制模式为禁止\n", phy_spd);
        return;
    }

    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC,"speed = %drpm, spd_dir = %hhd, motor_dir = %hhd\n", speed, m_channel_status.spindle_dir, spd_config.motor_dir);

    int s = labs(speed);
    s = s * this->m_channel_status.spindle_ratio / 100;    //转速乘以主轴倍率

    if((uint32_t)s > spd_config.spd_max_speed)
        s = spd_config.spd_max_speed;

    if(spd_config.spd_vctrl_mode == 2){	//-10v ~ 10v
        s *= this->m_channel_status.spindle_dir;
    }else if(this->m_channel_status.spindle_dir == 0)  //停转
        s = 0;

    s += spd_config.zero_compensation;   //加上零偏

    //	s /= spd_config.spd_gear_ratio;

    int64_t tt = s;
    uint32_t da_prec = this->m_p_channel_engine->GetDaPrecision();  //DA精度
    tt = tt * da_prec / spd_config.spd_max_speed;  //转换为电平值

    //防止16位溢出
    if(labs(tt) >= da_prec){
        if(tt > 0)
            tt = da_prec - 1 ;
        else{
            tt = da_prec - 1;
            tt *= -1;
        }
    }

    if(spd_config.motor_dir == 1)
        tt *= -1;   //设置方向为反转

    g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC,"Output spd speed da value: %lld, spd_axis=%hhu, max_speed=%u, da_prec=%u\n",
                            tt, phy_spd, spd_config.spd_max_speed, da_prec);

    this->SendSpdSpeedToMi(phy_spd, (int16_t)tt);

}

/**
 * @brief 主轴输出
 * @param dir : 主轴旋转方向
 * @param speed : 主轴指定转速
 */
void ChannelControl::SpindleOut(int dir, int speed){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "ChannelControl::spindleout : %d, speed=%d, chn=%hhu\n", dir, speed, this->m_n_channel_index);
    if(dir == SPD_DIR_STOP)
        this->m_p_f_reg->SPS = 0;    //标志主轴停转
    else if(dir == SPD_DIR_POSITIVE)
        this->m_p_f_reg->SPS = 1;    //标志主轴正转
    else if(dir == SPD_DIR_NEGATIVE)
        this->m_p_f_reg->SPS = 2;    //标志主轴反转

    if(this->m_n_spindle_count == 0)
        return;
    this->m_channel_status.spindle_dir = dir;
    if(dir == SPD_DIR_STOP)
        this->m_channel_status.rated_spindle_speed = 0;
    else if(speed == 0){
        this->m_channel_status.rated_spindle_speed = this->m_p_axis_config[m_spd_axis_phy[0]-1].spd_set_speed;
    }else
        this->m_channel_status.rated_spindle_speed = speed;

    this->SendModeChangToHmi(S_MODE);

    this->SpindleSpeedDaOut(m_channel_status.rated_spindle_speed);
}

/**
 * @brief 更新工件坐标系
 * @param index
 * @param cfg
 */
void ChannelControl::UpdateCoord(uint8_t index, HmiCoordConfig &cfg){
    if(m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING){//非运行状态，直接生效
        g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, index, cfg, true);
        if(index == 0 || this->m_channel_status.gmode[14] == (G53_CMD+index*10))  //修改了基本偏移，或者当前坐标系
            this->SetMcCoord(true);
    }
    else
        g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, index, cfg, true);

    //	g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, index, cfg, true);
}

/**
 * @brief 更新扩展工件坐标系
 * @param index
 * @param cfg
 */
void ChannelControl::UpdateExCoord(uint8_t index, HmiCoordConfig &cfg){
    if(m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING){//非运行状态，直接生效
        g_ptr_parm_manager->UpdateExCoordConfig(m_n_channel_index, index, cfg, true);

        if(this->m_channel_status.gmode[14] == (G5401_CMD+index*10))
            this->SetMcCoord(true);
    }else if(this->m_channel_status.gmode[14] != (G5401_CMD+index*10)){
        g_ptr_parm_manager->UpdateExCoordConfig(m_n_channel_index, index, cfg, true);
    }
    else
        g_ptr_parm_manager->UpdateExCoordConfig(m_n_channel_index, index, cfg, true);
}

/**
 * @brief 更新所有工件坐标系为设定值
 * @param val
 * @return
 */
bool ChannelControl::UpdateAllCoord(double val)
{
    if(m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING){//非运行状态，直接生效
        //常规坐标系 基本坐标系+G54-G59
        g_ptr_parm_manager->UpdateAllCoordConfig(m_n_channel_index, val, true);
        return true;
    }
    return false;

}

/**
 * @brief 更新所有扩展工件坐标系为设定值
 * @param val
 * @return
 */
bool ChannelControl::UpdateAllExCoord(double val, int count)
{
    if(m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING){//非运行状态，直接生效
        //扩展坐标系
        std::cout << "count: " << count << std::endl;
        g_ptr_parm_manager->UpdateAllExCoordConfig(m_n_channel_index, val, true, count);
        return true;
    }
    return false;
}

/**
 * @brief 更新刀具偏置
 * @param index : 刀偏编号，从0开始
 * @param cfg
 */
void ChannelControl::UpdateToolOffset(uint8_t index, HmiToolOffsetConfig &cfg){
    if(m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING){//非运行状态，直接生效
        g_ptr_parm_manager->UpdateToolOffsetConfig(m_n_channel_index, index, cfg, true);
        if(this->m_channel_status.gmode[8] != G49_CMD && this->m_channel_status.cur_h_code == index+1)
            this->SetMcToolOffset(true);
    }else if(this->m_channel_status.cur_h_code != index+1){  //非当前刀偏，直接生效
        g_ptr_parm_manager->UpdateToolOffsetConfig(m_n_channel_index, index, cfg, true);
    }
    else
        g_ptr_parm_manager->UpdateToolOffsetConfig(m_n_channel_index, index, cfg, true);
}

bool ChannelControl::UpdateAllToolOffset(const HmiToolOffsetConfig &cfg)
{
    //if(m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING){//非运行状态
        g_ptr_parm_manager->UpdateAllToolOffsetConfig(m_n_channel_index, cfg);
        //if(this->m_channel_status.gmode[8] != G49_CMD && this->m_channel_status.cur_h_code == index+1)
        //    this->SetMcToolOffset(true);
    //}
    return true;
}

/**
 * @brief 修改清整数圈位置的轴mask
 * @param axis_index : 物理轴序号，从1开始
 */
void ChannelControl::SetClearPosAxisMask(uint8_t axis_index){

    if(axis_index > this->m_p_general_config->axis_count)
        return;
    this->m_n_mask_clear_pos |= (0x01<<(axis_index-1));

}

/**
 * @brief 保存非易失性宏变量
 */
void ChannelControl::SaveKeepMacroVar(){
    this->m_macro_variable.Sync(true);  //保存并关闭文件
}

/**
 * @brief 检查轴上伺服状态
 * @param flag
 */
bool ChannelControl::CheckAxisSrvOn(uint64_t &flag){
    uint64_t line_axis = m_n_real_phy_axis;
    uint64_t srvon_mask = flag;
    //printf("CheckAxisSrvOn:m_n_real_phy_axis=%lld,flag=%lld",m_n_real_phy_axis,flag);

    if(m_p_spindle->Type() != 0){
        line_axis &= ~(0x01 << m_p_spindle->GetPhyAxis());
        srvon_mask &= ~(0x01 << m_p_spindle->GetPhyAxis());
    }
    for(int i=0; i<m_p_channel_config->chn_axis_count; i++){
        if(m_p_g_reg->SVF & (0x01<<i)){
            line_axis &= ~(0x01 << i);
            srvon_mask &= ~(0x01 << i);
        }
    }
    //printf("CheckAxisSrvOn:line_axis=%lld,srvon_mask=%lld",line_axis,srvon_mask);

    if((line_axis & srvon_mask) == line_axis){
        this->m_p_f_reg->SA = 1;
        return true;
    }else{
        this->m_p_f_reg->SA = 0;
        return false;
    }
}

/**
 * @brief 复位OP信号
 */
void ChannelControl::ResetOPSignal(){
    this->m_p_f_reg->OP = 0;
    printf("ChannelControl::ResetOPSignal()\n");
}

/**
 * @brief 复位RST信号
 */
void ChannelControl::ResetRSTSignal(){
    //
    this->m_p_f_reg->RST = 0;
}

/**
 * @brief 设置告警信号
 * @param value
 */
void ChannelControl::SetALSignal(bool value){

	if(value){
		// 刚攻报警 退出刚攻状态反馈 避免刚攻中无法复位
		printf("+++++++++++++++++++++++++ RGSPP = 0\n");
		m_p_f_reg->RGSPP = 0;
		this->m_p_f_reg->AL = 1;
	}else{
		this->m_p_f_reg->AL = 0;
	}
}

/**
 * @brief 将坐标由机械坐标系转换为工件坐标系
 * @param pos : 待转换的坐标
 * @param coord_idx : 工件坐标系
 * @param axis_mask : 轴掩码
 */
void ChannelControl::TransMachCoordToWorkCoord(DPointChn &pos, uint16_t coord_idx, uint32_t axis_mask){
    double origin_pos = 0;
    double *pp = pos.m_df_point;
    uint8_t phy_axis = 0xFF;

    //axis_mask &= (~m_mask_pmc_axis);   //去掉PMC轴

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            origin_pos = m_p_chn_coord_config[0].offset[i];  //基本工件坐标系

            if(coord_idx >= G54_CMD && coord_idx <= G59_CMD ){
                origin_pos += m_p_chn_coord_config[coord_idx/10-53].offset[i];
            }else if(coord_idx >= G5401_CMD && coord_idx <= G5499_CMD){
                origin_pos += m_p_chn_ex_coord_config[coord_idx/10-5401].offset[i];
            }

            if(G52Active){
            	origin_pos += G52offset[i];
            }
            origin_pos += m_p_chn_g92_offset->offset[i];

            phy_axis = this->GetPhyAxis(i);
            if(phy_axis != 0xFF){
                if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z轴需要加上刀长偏置
                    //	origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //基准刀偏
                    origin_pos += m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2];  //
                    origin_pos += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1];   //叠加磨损补偿
                }

            }

            *pp -= origin_pos;    //机械坐标 - 工件坐标系偏移 = 工件坐标

        }
        pp++;

    }

}

/**
 * @brief 将坐标由机械坐标系转换为工件坐标系
 * @param pos : 待转换的坐标
 * @param coord_idx : 工件坐标系
 * @param h_code : 刀长补偿号
 * @param axis_mask : 轴掩码
 */
void ChannelControl::TransMachCoordToWorkCoord(DPointChn &pos, uint16_t coord_idx, uint16_t h_code, uint32_t axis_mask){
    double origin_pos = 0;
    double *pp = pos.m_df_point;
    uint8_t phy_axis = 0xFF;

    //axis_mask &= (~m_mask_pmc_axis);   //去掉PMC轴

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            origin_pos = m_p_chn_coord_config[0].offset[i];  //基本工件坐标系

            if(coord_idx >= G54_CMD && coord_idx <= G59_CMD ){
                origin_pos += m_p_chn_coord_config[coord_idx/10-53].offset[i];
            }else if(coord_idx >= G5401_CMD && coord_idx <= G5499_CMD){
                origin_pos += m_p_chn_ex_coord_config[coord_idx/10-5401].offset[i];
            }

            if(G52Active){
            	origin_pos += G52offset[i];
		    }
            origin_pos += m_p_chn_g92_offset->offset[i];

            phy_axis = this->GetPhyAxis(i);
            if(phy_axis != 0xFF){
                if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && h_code > 0){//Z轴需要加上刀长偏置
                    //	origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //基准刀偏
                    origin_pos += m_p_chn_tool_config->geometry_compensation[h_code-1][2];  //
                    origin_pos += m_p_chn_tool_config->geometry_wear[h_code-1];   //叠加磨损补偿
                }
            }
            *pp -= origin_pos;    //机械坐标 - 工件坐标系偏移 = 工件坐标
        }
        pp++;
    }
}

/**
 * @brief 将坐标由机械坐标系转换为工件坐标系
 * @param pos : 待转换的坐标
 * @param coord_idx : 工件坐标系
 * @param axis_mask : 轴掩码
 */
void ChannelControl::TransMachCoordToWorkCoord(DPoint &pos, uint16_t coord_idx, uint32_t axis_mask){
    double origin_pos = 0;
    double *pp = &pos.x;
    uint8_t phy_axis = 0xFF;

    //axis_mask &= (~m_mask_pmc_axis);   //去掉PMC轴

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            origin_pos = m_p_chn_coord_config[0].offset[i];  //基本工件坐标系

            if(coord_idx >= G54_CMD && coord_idx <= G59_CMD ){
                origin_pos += m_p_chn_coord_config[coord_idx/10-53].offset[i];
            }else if(coord_idx >= G5401_CMD && coord_idx <= G5499_CMD){
                origin_pos += m_p_chn_ex_coord_config[coord_idx/10-5401].offset[i];
            }

            if(G52Active){
            	origin_pos += G52offset[i];
		    }
            origin_pos += m_p_chn_g92_offset->offset[i];

            phy_axis = this->GetPhyAxis(i);
            if(phy_axis != 0xFF){
                if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z轴需要加上刀长偏置
                    //		origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //基准刀偏
                    origin_pos += m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2];  //
                    origin_pos += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1];   //叠加磨损补偿
                }

            }

            *pp -= origin_pos;    //机械坐标 - 工件坐标系偏移 = 工件坐标

        }
        pp++;
    }
}

/**
 * @brief 将坐标由工件坐标系转换为机械坐标系
 * @param pos : 待转换的坐标
 * @param coord_idx : 工件坐标系
 *  @param axis_mask : 轴掩码
 */
void ChannelControl::TransWorkCoordToMachCoord(DPointChn &pos, uint16_t coord_idx, uint32_t axis_mask){
    double origin_pos = 0;
    double *pp = pos.m_df_point;
    uint8_t phy_axis = 0xFF;

    //axis_mask &= (~m_mask_pmc_axis);   //去掉PMC轴

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            origin_pos = m_p_chn_coord_config[0].offset[i];  //基本工件坐标系

            if(coord_idx >= G54_CMD && coord_idx <= G59_CMD ){
                origin_pos += m_p_chn_coord_config[coord_idx/10-53].offset[i];
            }else if(coord_idx >= G5401_CMD && coord_idx <= G5499_CMD){
                origin_pos += m_p_chn_ex_coord_config[coord_idx/10-5401].offset[i];
            }

            phy_axis = this->GetPhyAxis(i);
            if(phy_axis != 0xFF){
                if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z轴需要加上刀长偏置
                    //		origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //基准刀偏
                    origin_pos += m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2];  //
                    origin_pos += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1];   //叠加磨损补偿
                }

            }

            *pp += origin_pos;    //机械坐标 - 工件坐标系偏移 = 工件坐标

        }
        pp++;

    }
}

/**
 * @brief 将坐标由工件坐标系转换为机械坐标系
 * @param pos : 待转换的坐标
 * @param coord_idx : 工件坐标系
 *  @param axis_mask : 轴掩码
 */
void ChannelControl::TransWorkCoordToMachCoord(DPoint &pos, uint16_t coord_idx, uint32_t axis_mask){
    double origin_pos = 0;
    double *pp = &pos.x;
    uint8_t phy_axis = 0xFF;

    //axis_mask &= (~m_mask_pmc_axis);   //去掉PMC轴

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            origin_pos = m_p_chn_coord_config[0].offset[i];  //基本工件坐标系

            if(coord_idx >= G54_CMD && coord_idx <= G59_CMD ){
                origin_pos += m_p_chn_coord_config[coord_idx/10-53].offset[i];
            }else if(coord_idx >= G5401_CMD && coord_idx <= G5499_CMD){
                origin_pos += m_p_chn_ex_coord_config[coord_idx/10-5401].offset[i];
            }

            phy_axis = this->GetPhyAxis(i);
            if(phy_axis != 0xFF){
                if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z轴需要加上刀长偏置
                    //		origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //基准刀偏
                    origin_pos += m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2];  //
                    origin_pos += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1];   //叠加磨损补偿
                }

            }

            *pp += origin_pos;    //机械坐标 - 工件坐标系偏移 = 工件坐标

        }
        pp++;
    }
}

/**
 * @brief 将单轴坐标由机械坐标系转换为工件坐标系
 * @param pos : 单轴机械坐标
 * @param coord_idx : 工件坐标系
 * @param axis ：通道轴号, 从0开始
 */
void ChannelControl::TransMachCoordToWorkCoord(double &pos, uint16_t coord_idx, uint8_t axis){
    double origin_pos = 0;
    origin_pos = m_p_chn_coord_config[0].offset[axis];  //基本工件坐标系
    if(coord_idx >= G54_CMD && coord_idx <= G59_CMD ){
        origin_pos += m_p_chn_coord_config[coord_idx/10-53].offset[axis];
    }else if(coord_idx >= G5401_CMD && coord_idx <= G5499_CMD){
        origin_pos += m_p_chn_ex_coord_config[coord_idx/10-5401].offset[axis];
    }

    if(G52Active){
    	origin_pos += G52offset[axis];
    }
    origin_pos += m_p_chn_g92_offset->offset[axis];

    uint8_t phy_axis = this->GetPhyAxis(axis);
    if(phy_axis != 0xFF){
        if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z轴需要加上刀长偏置
            //	origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //基准刀偏
            origin_pos += m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2];
            origin_pos += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1];   //叠加磨损补偿
        }

    }


    pos -= origin_pos;    //机械坐标 - 工件坐标系偏移 = 工件坐标
}

/**
 * @brief 将单轴坐标由工件坐标系转换为机械坐标系
 * @param pos : 单轴工件坐标系
 * @param coord_idx : 工件坐标系
 * @param axis ：通道轴号
 */
void ChannelControl::TransWorkCoordToMachCoord(double &pos, uint16_t coord_idx, uint8_t axis){
    double origin_pos = 0;
    origin_pos = m_p_chn_coord_config[0].offset[axis];  //基本工件坐标系
    if(coord_idx >= G54_CMD && coord_idx <= G59_CMD ){
        origin_pos += m_p_chn_coord_config[coord_idx/10-53].offset[axis];
    }else if(coord_idx >= G5401_CMD && coord_idx <= G5499_CMD){
        origin_pos += m_p_chn_ex_coord_config[coord_idx/10-5401].offset[axis];
    }

    uint8_t phy_axis = this->GetPhyAxis(axis);
    if(phy_axis != 0xFF){
        if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z轴需要加上刀长偏置
            //	origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //基准刀偏
            origin_pos += m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2];
            origin_pos += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1];   //叠加磨损补偿
        }

    }

    pos += origin_pos;    //机械坐标 - 工件坐标系偏移 = 工件坐标
}

/**
 * @brief 将单轴坐标由工件坐标系转换为机械坐标系
 * @param pos :  待转换的工件坐标
 * @param coord_idx ： 工件坐标系号
 * @param h_code ：刀长偏置号，从1开始
 * @param axis ： 通道轴号
 */
void ChannelControl::TransWorkCoordToMachCoord(double &pos, uint16_t coord_idx, uint16_t h_code, uint8_t axis){
    double origin_pos = 0;
    origin_pos = m_p_chn_coord_config[0].offset[axis];  //基本工件坐标系
    if(coord_idx >= G54_CMD && coord_idx <= G59_CMD ){
        origin_pos += m_p_chn_coord_config[coord_idx/10-53].offset[axis];
    }else if(coord_idx >= G5401_CMD && coord_idx <= G5499_CMD){
        origin_pos += m_p_chn_ex_coord_config[coord_idx/10-5401].offset[axis];
    }

    uint8_t phy_axis = this->GetPhyAxis(axis);
    if(phy_axis != 0xFF){
        if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z轴需要加上刀长偏置
            //	origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //基准刀偏
            origin_pos += m_p_chn_tool_config->geometry_compensation[h_code-1][2];
            origin_pos += m_p_chn_tool_config->geometry_wear[h_code-1];   //叠加磨损补偿
        }

    }

    pos += origin_pos;    //机械坐标 - 工件坐标系偏移 = 工件坐标
}


#ifdef USES_LASER_MACHINE
/**
 * @brief 处理HMI调高器标定指令
 * @param bstart : true--开始标定    false--取消标定
 */
void ChannelControl::ProcessHmiCalibCmd(bool bstart){

    if(bstart){
        if(!this->m_b_laser_calibration){
            this->m_b_laser_calibration = true;
            this->m_b_cancel_calib = false;
            this->m_n_calib_step = 0;
        }
    }else{
        if(this->m_b_laser_calibration){

            this->m_b_cancel_calib = true;
        }
    }
}

/**
 * @brief 处理激光调高器标定
 */
void ChannelControl::ProcessLaserCalibration(){

    //参考点标定过程中检测COLLISION信号（A11）是否被置位，为High则标定失败，告警并退出
    if(m_n_cur_ref_point > 0 && m_n_calib_step > 5){
        if(this->m_n_laser_input_signal & 0x02){
            printf("调高器标定异常！\n");
            this->m_n_calib_step = 100;
        }
    }

    //中断标定，退出
    if(this->m_b_cancel_calib){
        printf("退出调高器标定！\n");
        this->m_n_calib_step = 100;
        this->m_b_cancel_calib = false;
    }

    uint8_t chn_axis = 0;
    uint8_t phy_axis = 0;
    switch(this->m_n_calib_step){
    case 0: //取消调高器跟随使能
        this->EnableHeightRegulatorFollow(false);
        printf("禁止跟随使能\n");
        m_n_calib_step++;
        break;
    case 1: //确认量程及标定点数
        this->m_n_follow_range = HYD_RANGE_10;		//调高器范围10mm
        this->m_n_calib_ref_count = 16;   //16点标定
        this->m_n_cur_ref_point = 0;      //当前标定第一个参考点

        m_n_calib_step++;
        break;
    case 2: //设置CAL.REQUEST信号（PIN A3, High）
        this->SetHeightRegulatorOutput(1, true);

        printf("置位CAL.REQUEST信号,等待READY信号复位...\n");
        m_n_calib_step++;
        break;
    case 3:{ //等待READY信号置位LOW，然后指令Z轴慢速下移，设置MI捕获Z轴零点位置
        if(this->m_n_laser_input_signal & 0x08){
            break;  //等待READY信号变低
        }
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
        if(chn_axis == NO_AXIS){ //退出
            printf("Z轴未配置！\n");
            m_n_calib_step = 100;
            break;
        }
        uint64_t mask = 0x01;
        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_Z);
        if(phy_axis == NO_AXIS){ //退出
            printf("Z轴未配置！\n");
            m_n_calib_step = 100;
            break;
        }
        mask = (mask << phy_axis);
        this->SendCatchPosCmd(mask, 2, true);   //设置Z轴的位置捕获
        this->ManualMove2(chn_axis, DIR_NEGATIVE, 120, 1000);  //Z轴开始慢速向负向移动

        printf("Z轴慢速碰板，寻找零点！mask = 0x%llx\n", mask);
        m_n_calib_step++;
    }
        break;
    case 4: //等待MI捕获Z轴位置，并停下Z轴
        if(this->m_b_pos_captured){
            chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
            this->ManualMoveStop(0x01<<chn_axis);   //停止Z轴

            printf("找到零点：%lf, %lf, %lf, %lf\n", m_point_capture.x, m_point_capture.y,
                   m_point_capture.z, m_point_capture.a4);
            m_n_calib_step++;
        }

        break;
    case 5:{ //移动到当前参考点
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
        double *pp = &this->m_point_capture.x;
        pp += chn_axis;
        double pos_tar = *pp;
        pos_tar += kHYDCalibRefPos[m_n_follow_range][m_n_cur_ref_point];  //目标位置
        //	if(this->m_n_cur_ref_point == 0){//第一个参考点，向正方向移动到最大量程
        this->ManualMove(chn_axis, pos_tar, 3000);
        //	}else{
        //		this->ManualMove2(chn_axis, DIR_NEGATIVE, 3000,
        //				kHYDCalibRefPos[m_n_follow_range][m_n_cur_ref_point-1]-kHYDCalibRefPos[m_n_follow_range][m_n_cur_ref_point]);
        //	}
        printf("移动到第%hhu参考点:%lf mm\n", m_n_cur_ref_point+1, pos_tar);
        m_n_calib_step++;
    }
        break;
    case 6:{ //等待轴移动到位，设置STROBE信号（PIN A7, High）
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
        double *pp = &this->m_point_capture.x;
        pp += chn_axis;
        double pos_tar = *pp;
        pos_tar += kHYDCalibRefPos[m_n_follow_range][m_n_cur_ref_point];  //目标位置

        if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(chn_axis) - pos_tar) < 1e-3){ //到位
            this->SetHeightRegulatorOutput(5, true);  //置位STROBE信号（PIN A7, High）

            gettimeofday(&m_time_delay_start, nullptr);  //记录起始时间
            printf("到位延时...\n");
            m_n_calib_step++;
        }

    }
        break;
    case 7:{ //等待240ms延时
        struct timeval cur_time;
        uint64_t time_elpase = 0;
        gettimeofday(&cur_time, nullptr);
        time_elpase = (cur_time.tv_sec-m_time_delay_start.tv_sec)*1000 + (cur_time.tv_usec - m_time_delay_start.tv_usec)/1000;  //ms
        if(time_elpase < 240)
            break;		//未到延时时间
        m_n_calib_step++;
    }
        break;
    case 8: //等待POS_REACHED置位（PIN A15, High），复位STROBE信号（PIN A7, LOW）
        if(this->m_n_laser_input_signal & 0x20){  //POS_REACHED信号为高
            this->SetHeightRegulatorOutput(5, false);  //复位STROBE信号（PIN A7, Low）

            m_n_calib_step++;
        }

        break;
    case 9: //判断是否测量完所有参考点，如果没有则跳回step5，否则执行下一步
        if(this->m_n_laser_input_signal & 0x20){
            break;   //如果POS_REACHED信号没有复位则等待
        }
        if(++m_n_cur_ref_point < this->m_n_calib_ref_count){ //未测量完所有参考点
            m_n_calib_step = 5;
            break;
        }
        m_n_calib_step++;
        break;
    case 10:{ //Z轴移动到35mm高度，
        printf("开始标定COLLISION信号\n");
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
        double *pp = &this->m_point_capture.x;
        pp += chn_axis;
        double pos_tar = *pp;
        pos_tar += 35.0;  //目标位置

        this->ManualMove(chn_axis, pos_tar, 5000);

    }
        m_n_calib_step++;
        break;
    case 11:{ //等待Z轴到位
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
        double *pp = &this->m_point_capture.x;
        pp += chn_axis;
        double pos_tar = *pp;
        pos_tar += 35.0;  //目标位置

        if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(chn_axis) - pos_tar) < 1e-3){ //到位
            gettimeofday(&m_time_delay_start, nullptr);  //记录起始时间
            printf("Z轴到位\n");
            m_n_calib_step++;
        }
    }
        break;
    case 12:{ //等待240ms延时
        struct timeval cur_time;
        uint64_t time_elpase = 0;
        gettimeofday(&cur_time, nullptr);
        time_elpase = (cur_time.tv_sec-m_time_delay_start.tv_sec)*1000 + (cur_time.tv_usec - m_time_delay_start.tv_usec)/1000;  //ms
        if(time_elpase < 240)
            break;		//未到延时时间
        m_n_calib_step++;
    }
        break;
    case 13: //标定结束，复位CAL.REQUEST信号（PIN A3, LOW）
        this->SetHeightRegulatorOutput(1, false);
        printf("复位CAL.REQUEST信号\n");
        m_n_calib_step++;
        break;
    case 14: //等待READY信号置位High
        if(this->m_n_laser_input_signal & 0x08){//等待READY信号变高
            this->m_b_laser_calibration = false;
            m_n_calib_step = 0;
            printf("调高器标定成功！\n");
        }
        break;
    case 100: //失败，退出
        this->ManualMoveStop();   //停止所有轴移动
        this->SetHeightRegulatorOutput(1, false);
        this->SetHeightRegulatorOutput(5, false);  //复位STROBE信号（PIN A7, Low）
        this->m_b_laser_calibration = false;
        m_n_calib_step = 0;
        printf("调高器标定失败！\n");
        break;
    default:
        printf("标定流程执行异常!\n");
        break;
    }
}

/**
 * @breif 调高器跟随使能
 * @param enable : true -- 使能跟随功能     false -- 禁止跟随功能
 */
void ChannelControl::EnableHeightRegulatorFollow(bool enable){
    uint8_t phy_axis = this->GetPhyAxisFromName(AXIS_NAME_Z);  //跟随轴号
    if(phy_axis != NO_AXIS){
        phy_axis += 1;  //MI中轴号从1开始
    }
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_ENABLE_FOLLOW;
    cmd.data.axis_index = phy_axis;
    //	cmd.data.reserved = type;
    cmd.data.data[0] = enable?1:0;
    cmd.data.data[1] = 5000;   //跟随高度5mm

    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @breif 读取调高器输入信号
 */
void ChannelControl::ReadHeightRegulatorInput(){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_READ_LASER_INPUT;
    cmd.data.axis_index = NO_AXIS;


    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 设置调高器输出信号
 * @param idx : 1-8分别表示1-8号输出
 * @param value ： true--高电平    false--低电平
 */
void ChannelControl::SetHeightRegulatorOutput(uint8_t idx, bool value){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_LASER_OUTPUT;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.data[0] = idx;
    cmd.data.data[1] = value?1:0;


    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 向MI发送轴位置捕获指令
 * @param axis_mask ：捕获轴的mask
 * @param io_idx ：1-6 ： 分别表示IN1-IN6
 * @param level : 有效电平， true-高有效    false-低有效
 */
void ChannelControl::SendCatchPosCmd(uint64_t axis_mask, uint16_t io_idx, bool level){
    this->m_b_pos_captured = false;
    this->m_n_mask_pos_capture = axis_mask;
    this->m_n_mask_captured = 0;

    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_POS_CAPTURE;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.data[0] = axis_mask & 0xFFFF;
    cmd.data.data[1] = (axis_mask>>16)&0xFFFF;
    cmd.data.data[2] = (axis_mask>>32)&0xFFFF;
    cmd.data.data[3] = (axis_mask>>48)&0xFFFF;
    cmd.data.data[4] = io_idx;
    cmd.data.data[5] = level?1:0;

    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 处理MI返回的激光调高器输入信号值
 * @param cmd
 */
void ChannelControl::ProcessMiInputSignal(MiCmdFrame &cmd){
    this->m_n_laser_input_signal = cmd.data.data[0];
}

/**
 * @brief 处理Mi返回的轴位置捕获结果
 * @param cmd
 */
void ChannelControl::ProcessMiPosCapture(MiCmdFrame &cmd){
    if(!this->m_b_laser_calibration)
        return;
    printf("Get pos captured response\n");
    uint64_t tmp = 0x01;
    int64_t tpp = 0;
    if(this->m_n_mask_pos_capture & (tmp<<(cmd.data.axis_index-1))){
        this->m_n_mask_captured |= (tmp<<(cmd.data.axis_index-1));   //设置该轴捕获成功

        memcpy(&tpp, cmd.data.data, sizeof(int64_t));    //获取位置

        uint8_t chn_axis = this->GetChnAxisFromPhyAxis(cmd.data.axis_index-1);
        if(chn_axis == NO_AXIS)
            return;
        double *p = &(m_point_capture.x);
        double df = tpp;
        p += chn_axis;
        *p = df/1e7;   //转换单位  0.1nm-->mm
        printf("capture pos : %lf, %lf, %lld\n", m_point_capture.z, df, tpp);
    }

    if(m_n_mask_captured == m_n_mask_pos_capture)
        this->m_b_pos_captured = true;    //位置捕获完成
}

#endif


#ifdef USES_FIVE_AXIS_FUNC
/**
 * @brief 更新五轴旋转轴的相关参数
 */
void ChannelControl::UpdateFiveAxisRotParam(){
    if(m_p_chn_5axis_config->five_axis_type == NO_FIVEAXIS || m_p_chn_5axis_config->five_axis_type > C_A_TILT_FIVEAXIS)
    {
        this->m_n_5axis_rot1 = NO_AXIS;
        this->m_n_5axis_rot2 = NO_AXIS;
        this->m_mask_5axis_rot_nolimit = 0;
    }
    else if(m_p_chn_5axis_config->five_axis_type == C_A_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_C);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_A);
        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit = 0x01<<m_n_5axis_rot1;

    }
    else if(m_p_chn_5axis_config->five_axis_type == C_B_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_C);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_B);
        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit = 0x01<<m_n_5axis_rot1;
    }
    else if(m_p_chn_5axis_config->five_axis_type == A_B_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_A);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_B);
        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit = 0x01<<m_n_5axis_rot1;
    }
    else if(m_p_chn_5axis_config->five_axis_type == B_A_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_B);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_A);
        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit = 0x01<<m_n_5axis_rot1;
    }
    else if(m_p_chn_5axis_config->five_axis_type == A1_C1_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_A);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_C);
        m_mask_5axis_rot_nolimit = 0;  // 先清零
        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot1;
        if(m_p_chn_5axis_config->range_limit_2)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot2;

    }
    else if(m_p_chn_5axis_config->five_axis_type == B1_C1_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_B);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_C);
        m_mask_5axis_rot_nolimit = 0;  // 先清零
        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot1;
        if(m_p_chn_5axis_config->range_limit_2)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot2;
    }
    else if(m_p_chn_5axis_config->five_axis_type == A1_B1_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_A);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_B);
        m_mask_5axis_rot_nolimit = 0;  // 先清零
        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot1;
        if(m_p_chn_5axis_config->range_limit_2)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot2;
    }
    else if(m_p_chn_5axis_config->five_axis_type == B1_A1_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_B);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_A);
        m_mask_5axis_rot_nolimit = 0;  // 先清零
        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot1;
        if(m_p_chn_5axis_config->range_limit_2)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot2;
    }
    else if(m_p_chn_5axis_config->five_axis_type == C1_A_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_C);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_A);

        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit = 0x01<<m_n_5axis_rot1;

    }
    else if(m_p_chn_5axis_config->five_axis_type == C1_B_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_C);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_B);

        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit = 0x01<<m_n_5axis_rot1;
    }
    else if(m_p_chn_5axis_config->five_axis_type == A1_B_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_A);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_B);

        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit = 0x01<<m_n_5axis_rot1;
    }
    else if(m_p_chn_5axis_config->five_axis_type == B1_A_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_B);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_A);

        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit = 0x01<<m_n_5axis_rot1;
    }
    else
    {
        this->m_n_5axis_rot1 = NO_AXIS;
        this->m_n_5axis_rot2 = NO_AXIS;
        this->m_mask_5axis_rot_nolimit = 0;
    }
}

/**
 * @brief 设置通道五轴相关参数
 */
void ChannelControl::SetMcChnFiveAxisParam(){
    this->UpdateFiveAxisParam(MACHINE_TYPE);
    UpdateFiveAxisParam(X_OFFSET_1);
    UpdateFiveAxisParam(Y_OFFSET_1);
    UpdateFiveAxisParam(Z_OFFSET_1);
    UpdateFiveAxisParam(X_OFFSET_2);
    UpdateFiveAxisParam(Y_OFFSET_2);
    UpdateFiveAxisParam(Z_OFFSET_2);
    UpdateFiveAxisParam(ROT_SWING_ARM_LEN);
    UpdateFiveAxisParam(POST_DIR_1);
    UpdateFiveAxisParam(POST_DIR_2);

    UpdateFiveAxisParam(SPEED_LIMIT);
    UpdateFiveAxisParam(SPEED_LIMIT_X);
    UpdateFiveAxisParam(SPEED_LIMIT_Y);
    UpdateFiveAxisParam(SPEED_LIMIT_Z);
    UpdateFiveAxisParam(SPEED_LIMIT_A);
    UpdateFiveAxisParam(SPEED_LIMIT_B);
    UpdateFiveAxisParam(SPEED_LIMIT_C);
    UpdateFiveAxisParam(SPEED_PLAN_MODE);
    UpdateFiveAxisParam(INTP_ANGLE_STEP);
    UpdateFiveAxisParam(INTP_LEN_STEP);
    UpdateFiveAxisParam(PROGRAM_COORD);

}

/**
 * @brief 设置五轴参数
 * @param type : 参数类型
 */
void ChannelControl::UpdateFiveAxisParam(FiveAxisParamType type){
    McCmdFrame cmd;
    int32_t data = 0;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_SET_CHN_FIVE_AXIS_PARAM;
    cmd.data.data[0] = type;

    switch(type){
    case MACHINE_TYPE: 	//五轴类型
        data = this->m_p_chn_5axis_config->five_axis_type;
        break;
    case X_OFFSET_1:        //第一转轴中心相对刀尖点X坐标
        data = this->m_p_chn_5axis_config->x_offset_1*1000;   //单位转换：mm->um
        break;
    case Y_OFFSET_1:        //第一转轴中心相对刀尖点Y坐标
        data = this->m_p_chn_5axis_config->y_offset_1*1000;   //单位转换：mm->um
        break;
    case Z_OFFSET_1:        //第一转轴中心相对刀尖点Z坐标
        data = this->m_p_chn_5axis_config->z_offset_1*1000;   //单位转换：mm->um
        break;
    case X_OFFSET_2:        //第二转轴中心相对刀尖点X坐标
        data = this->m_p_chn_5axis_config->x_offset_2*1000;   //单位转换：mm->um
        break;
    case Y_OFFSET_2:        //第二转轴中心相对刀尖点Y坐标
        data = this->m_p_chn_5axis_config->y_offset_2*1000;   //单位转换：mm->um
        break;
    case Z_OFFSET_2:        //第二转轴中心相对刀尖点Z坐标
        data = this->m_p_chn_5axis_config->z_offset_2*1000;   //单位转换：mm->um
        break;
    case ROT_SWING_ARM_LEN: //刀具旋转摆臂长度
        data = this->m_p_chn_5axis_config->rot_swing_arm_len*1000;   //单位转换：mm->um
        break;
    case POST_DIR_1:      //第一转轴旋转正方向
        data = this->m_p_chn_5axis_config->post_dir_1;   //
        break;
    case POST_DIR_2:        //第二转轴旋转正方向
        data = this->m_p_chn_5axis_config->post_dir_2;
        break;

    case SPEED_LIMIT:       //五轴速度限制开关
        data = this->m_p_chn_5axis_config->five_axis_speed_plan;
        break;

    case SPEED_LIMIT_X:    //五轴联动X轴速度限制
        data = this->m_p_chn_5axis_config->speed_limit_x * 1000. / 60.;    //单位转换： mm/min --> um/s
        break;
    case SPEED_LIMIT_Y:         //五轴联动Y轴速度限制
        data = this->m_p_chn_5axis_config->speed_limit_y * 1000. / 60.;    //单位转换： mm/min --> um/s
        break;
    case SPEED_LIMIT_Z:         //五轴联动Z轴速度限制
        data = this->m_p_chn_5axis_config->speed_limit_z * 1000. / 60.;    //单位转换： mm/min --> um/s
        break;
    case SPEED_LIMIT_A:         //五轴联动A轴速度限制
        data = this->m_p_chn_5axis_config->speed_limit_a * 1000. / 60.;    //单位转换： mm/min --> um/s
        break;
    case SPEED_LIMIT_B:         //五轴联动B轴速度限制
        data = this->m_p_chn_5axis_config->speed_limit_b * 1000. / 60.;    //单位转换： mm/min --> um/s
        break;
    case SPEED_LIMIT_C:         //五轴联动C轴速度限制
        data = this->m_p_chn_5axis_config->speed_limit_c * 1000. / 60.;    //单位转换： mm/min --> um/s
        break;
    case SPEED_PLAN_MODE:   //五轴速度规划方式
        data = this->m_p_chn_5axis_config->five_axis_plan_mode;
        break;
    case INTP_ANGLE_STEP:      //五轴联动粗插补角度步距
        data = this->m_p_chn_5axis_config->intp_angle_step * 1000;   //单位转换 ： 度 -->  0.001度
        break;
    case INTP_LEN_STEP:        //五轴联动粗插补最小长度
        data = this->m_p_chn_5axis_config->intp_len_step * 1000;     //单位转换：mm->um
        break;
    case PROGRAM_COORD:       //五轴编程坐标系
        data = this->m_p_chn_5axis_config->five_axis_coord;       //五轴编程坐标系
        break;
    }

    memcpy(&cmd.data.data[1], &data, sizeof(data));

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}
#endif


#ifdef USES_TWINING_FUNC
/**
 * @brief 开关缠绕功能
 * @param active : true--激活   false--关闭
 */
void ChannelControl::ActiveTwiningFunc(int active){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_ACTIVE_TWINING_FUNC;
    cmd.data.axis_index = NO_AXIS;

    switch(active){
    case 560:
        cmd.data.data[0] = 0x01;
        break;
    case 561:
        cmd.data.data[0] = 0x10;
        break;
    case 562:
        cmd.data.data[0] = 0x20;
        break;
    case 563:
        cmd.data.data[0] = 0x30;
        break;
    case 564:
        cmd.data.data[0] = 0x40;
        break;
    case 565:
        cmd.data.data[0] = 0x50;
        break;
    case 566:
        cmd.data.data[0] = 0x60;
        break;
    case 567:
        cmd.data.data[0] = 0x70;
        break;
    case 568:
        cmd.data.data[0] = 0x80;
        break;
    case 569:
        cmd.data.data[0] = 0x90;
        break;
    default:
        break;

    }

    this->m_p_mi_comm->WriteCmd(cmd);
}
#endif

/**
 * @brief 发送MI调试指令
 * @param mcode
 */
void ChannelControl::MiDebugFunc(int mcode){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_DEBUG_M_FUN;

    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved  = m_n_channel_index+1;
    cmd.data.data[0] = mcode;

    this->m_p_mi_comm->WriteCmd(cmd);
}

void ChannelControl::AddWorkCountPiece(int addNum)
{
    this->m_channel_status.workpiece_count += addNum;
    g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
    this->m_channel_status.workpiece_count_total += addNum;
    g_ptr_parm_manager->SetTotalWorkPiece(m_n_channel_index, m_channel_status.workpiece_count_total);
    this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);  //通知HMI更新加工计数

    if (m_channel_status.workpiece_require != 0 && m_channel_status.workpiece_count >= m_channel_status.workpiece_require)
    {//已到达需求件数
        CreateError(ERR_REACH_WORK_PIECE, INFO_LEVEL, CLEAR_BY_MCP_RESET);
    }
}

#ifdef USES_SPEED_TORQUE_CTRL

/**
 * @brief 处理主轴CS模式切换流程
 * @param msg : M指令消息
 * @param index ： M代码序号
 */
void ChannelControl::ProcessSpdModeSwitch(AuxMsg *msg, uint8_t index){
    if(msg == nullptr)
        return;
    int mCode=-1;
    uint8_t SpdIndex = 0, PhySpdIndex = -1;
    int toMode = -1;


    mCode = msg->GetMCode(index);
    if(mCode == 28){  //切回速度模式
        SpdIndex = 0;
        toMode = 2;
        PhySpdIndex = this->m_spd_axis_phy[SpdIndex]-1;
        SpdIndex = this->GetSpdChnAxis(SpdIndex);
    }else if(mCode == 29){  //切换至位置模式
        SpdIndex = 0;
        toMode = 1;
        PhySpdIndex = this->m_spd_axis_phy[SpdIndex]-1;
        SpdIndex = this->GetSpdChnAxis(SpdIndex);
    }else{
        msg->SetExecStep(index, 0xFF);
        return;
    }

    this->m_p_channel_engine->SendMonitorData(false, false);

    if(msg->GetExecStep(index) == 5){

        if(PhySpdIndex == 0){    // 轴不存在 直接返回，结束
            msg->SetExecStep(index, 0xFF);
            return;
        }
        int curMode = this->m_channel_status.cur_axis_ctrl_mode[SpdIndex];

        if(toMode == curMode){     // 该轴当前模式已经是切换模式了
            msg->SetExecStep(index, 0xFF);
            return;
        }

        if(curMode==2 || curMode==3){
            double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(SpdIndex);
            int32_t feed = fabs(curfeed);
            if(feed <= 1){  // 速度接近 0 	    可以切换                                                    // 位置指令切换为速度或力矩
                this->m_channel_status.cur_axis_ctrl_mode[SpdIndex] = toMode;  //记录当前轴控制模式
                m_channel_status.rated_spindle_speed = m_n_cur_scode;
                this->SendModeChangToHmi(S_MODE);

                this->SendAxisCtrlModeSwitchCmdToMi(PhySpdIndex, toMode);

                ctrlmode_switch_wait = 30;   //等待10个周期
                //				gettimeofday(&ctrlmode_switch_start, NULL);
                //				printf("switch to pos ctrl 111\n");

                msg->SetExecStep(index, 8);
            }else{
                if(curMode==2){          // 速度控制模式设置速度为0
                    //   this->SendAxisSpeedCmdToMi(PhySpdIndex,0);
                    this->SpindleOut(SPD_DIR_STOP);
                    msg->IncreaseExecStep(index);
                }else if(curMode==3){    // 力矩控制模式设置速度限制为0
                    this->SendAxisTorqueCmdToMi(PhySpdIndex,0,0,1);
                    msg->IncreaseExecStep(index);
                }
            }
        }else{		                                                        // 位置指令切换为速度或力矩
            this->m_channel_status.cur_axis_ctrl_mode[SpdIndex] = toMode;  //记录当前轴控制模式
            this->SendAxisCtrlModeSwitchCmdToMi(PhySpdIndex, toMode);
            msg->SetExecStep(index, 11);
        }

    }else if(msg->GetExecStep(index) == 6){
        // 判断轴是否停止
        printf("#### waiting for spd stop!!! \n");
        double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(SpdIndex);
        int32_t feed = fabs(curfeed);
        if(feed <= 1){  // 速度接近 0
            msg->IncreaseExecStep(index);
        }
    }else if(msg->GetExecStep(index) == 7){		//
        m_channel_status.rated_spindle_speed = m_n_cur_scode;
        this->SendModeChangToHmi(S_MODE);

        this->m_channel_status.cur_axis_ctrl_mode[SpdIndex] = toMode;  //记录当前轴控制模式
        this->SendAxisCtrlModeSwitchCmdToMi(PhySpdIndex, toMode);

        if(toMode==0 || toMode==1 ){
            ctrlmode_switch_wait = 10;   //等待10个周期

            msg->IncreaseExecStep(index);    // 如果是切换到位置模式， 则要去同步位置坐标
        }else{
            msg->SetExecStep(index, 10);
        }

    }else if(msg->GetExecStep(index) == 8){
        //等待一个执行周期10ms，等待MC控制模式切换完成
        if(--ctrlmode_switch_wait == 0){
            msg->IncreaseExecStep(index);
        }

    }else if(msg->GetExecStep(index) == 9){
        //同步位置
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        else
            this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        //		m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //		m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        RefreshAxisIntpPos();

        if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
            this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
            //			printf("sync compiler curpos[%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf]\n", m_channel_mc_status.intp_pos.x,
            //					m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z, m_channel_mc_status.intp_pos.a4, m_channel_mc_status.intp_pos.a5,
            //					m_channel_mc_status.intp_pos.a6, m_channel_mc_status.intp_pos.a7, m_channel_mc_status.intp_pos.a8);

        }else
            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //同步已编译的轴移动指令的位置


        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 10){

        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 11){
        msg->SetExecStep(index, 0xFF);    //置位结束状态
    }
}

///**
// * @brief 处理M29状态复位流程
// */
//void ChannelControl::ProcessM29Reset(){
//	uint8_t SpdIndex = 0, PhySpdIndex = -1;
//	int toMode = 2;  //切回速度模式

//	PhySpdIndex = this->m_spd_axis_phy[SpdIndex]-1;
//	SpdIndex = this->GetSpdChnAxis(SpdIndex);



//	if(PhySpdIndex == 0){    // 轴不存在 直接结束
//		m_n_M29_flag = 0;
//		return;
//	}

//	// 位置指令切换为速度
//	this->m_channel_status.cur_axis_ctrl_mode[SpdIndex] = toMode;  //记录当前轴控制模式
//	this->SendAxisCtrlModeSwitchCmdToMi(PhySpdIndex, toMode);

//	m_channel_status.rated_spindle_speed = 0;
//	this->SendModeChangToHmi(S_MODE);

//	m_n_M29_flag = 0;

//}

//struct timeval ctrlmode_switch_start;   //等待起始时间
/**
 * @brief 处理控制模式动态切换M指令
 * @param index : M代码序号
 * @param msg
 */
void ChannelControl::ProcessCtrlModeSwitch(AuxMsg *msg, uint8_t index){
    if(msg == nullptr)
        return;
    int mCode0=-1,mCode=-1;
    uint8_t AixsIndex = 0, PhyAixsIndex = -1;
    int toMode = -1;


    mCode0 = msg->GetMCode(index);
    mCode = mCode0 - 40000;
    AixsIndex = mCode/10-1;
    toMode = mCode%10;
    PhyAixsIndex = this->GetPhyAxis(AixsIndex);

    this->m_p_channel_engine->SendMonitorData(false, false);

    if(msg->GetExecStep(index) == 0){
        printf("execute M%d\n", mCode0);

        if(AixsIndex>8 ||
                PhyAixsIndex==0xff ||    // 轴不存在 直接返回，结束
                toMode>3){
            msg->SetExecStep(index, 0xFF);
            return;
        }
        int curMode = this->m_channel_status.cur_axis_ctrl_mode[AixsIndex];

        if(toMode == curMode ||     // 该轴当前模式已经是切换模式了
                (toMode<=1 && curMode<=1)){  // 直线位置模式 和 旋转位置模式 之间就不要切换了
            msg->SetExecStep(index, 0xFF);
            return;
        }

        if(curMode==2 || curMode==3){
            double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(AixsIndex);
            int32_t feed = fabs(curfeed);
            if(feed <= 1){  // 速度接近 0 	    可以切换                                                    // 位置指令切换为速度或力矩
                this->m_channel_status.cur_axis_ctrl_mode[AixsIndex] = toMode;  //记录当前轴控制模式
                this->SendAxisCtrlModeSwitchCmdToMi(PhyAixsIndex, toMode);

                ctrlmode_switch_wait = 30;   //等待10个周期
                //				gettimeofday(&ctrlmode_switch_start, NULL);
                //				printf("switch to pos ctrl 111\n");

                msg->SetExecStep(index, 3);
            }else{
                if(curMode==2){          // 速度控制模式设置速度为0
                    this->SendAxisSpeedCmdToMi(PhyAixsIndex,0);
                    msg->IncreaseExecStep(index);
                }else if(curMode==3){    // 力矩控制模式设置速度限制为0
                    this->SendAxisTorqueCmdToMi(PhyAixsIndex,0,0,1);
                    msg->IncreaseExecStep(index);
                }
            }
        }else{		                                                        // 位置指令切换为速度或力矩
            this->m_channel_status.cur_axis_ctrl_mode[AixsIndex] = toMode;  //记录当前轴控制模式
            this->SendAxisCtrlModeSwitchCmdToMi(PhyAixsIndex, toMode);
            msg->SetExecStep(index, 6);
        }

    }else if(msg->GetExecStep(index) == 1){
        // 判断轴是否停止
        printf("#### waiting for axis stop!!! \n");
        double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(AixsIndex);
        int32_t feed = fabs(curfeed);
        if(feed <= 1){  // 速度接近 0
            msg->IncreaseExecStep(index);
        }
    }else if(msg->GetExecStep(index) == 2){		//

        this->m_channel_status.cur_axis_ctrl_mode[AixsIndex] = toMode;  //记录当前轴控制模式
        this->SendAxisCtrlModeSwitchCmdToMi(PhyAixsIndex, toMode);

        if(toMode==0 || toMode==1 ){
            ctrlmode_switch_wait = 10;   //等待10个周期
            //			gettimeofday(&ctrlmode_switch_start, NULL);
            //			printf("switch to pos ctrl 222\n");

            msg->IncreaseExecStep(index);    // 如果是切换到位置模式， 则要去同步位置坐标
        }else{
            msg->SetExecStep(index, 5);
        }

    }else if(msg->GetExecStep(index) == 3){
        //等待一个执行周期10ms，等待MC控制模式切换完成
        if(--ctrlmode_switch_wait == 0){
            msg->IncreaseExecStep(index);
        }

    }else if(msg->GetExecStep(index) == 4){
        //		struct timeval time_now;
        //		gettimeofday(&time_now, NULL);
        //		int time_elpase = (time_now.tv_sec-ctrlmode_switch_start.tv_sec)*1000000+
        //									time_now.tv_usec-ctrlmode_switch_start.tv_usec;

        //同步位置
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        else
            this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        //		m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //		m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        RefreshAxisIntpPos();

        if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
            this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //同步编译器位置
            //			printf("sync compiler curpos[%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf]\n", m_channel_mc_status.intp_pos.x,
            //					m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z, m_channel_mc_status.intp_pos.a4, m_channel_mc_status.intp_pos.a5,
            //					m_channel_mc_status.intp_pos.a6, m_channel_mc_status.intp_pos.a7, m_channel_mc_status.intp_pos.a8);

        }else
            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //同步已编译的轴移动指令的位置


        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 5){

        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 6){

        printf("#### execute M%d succeed! \n", mCode0);
        //		msg->SetExecStep(index, 0);   //M指令执行结束
        msg->SetExecStep(index, 0xFF);    //置位结束状态
    }

}


/**
 * @brief 复位处理控制模式，系统复位时，首先速度控制和力矩控制输出为0
 * @param msg
 */
bool ChannelControl::ResetAllAxisOutZero(void){
    bool flag=false;
    int phy_axis;
    for(int i=0; i < this->m_p_channel_config->chn_axis_count; i++){
        phy_axis = this->GetPhyAxis(i);
        if(phy_axis != 0xff){
            if(this->m_channel_status.cur_axis_ctrl_mode[i] != m_p_axis_config[phy_axis].axis_type){
                if(this->m_channel_status.cur_axis_ctrl_mode[i] == 2){
                    this->SendAxisSpeedCmdToMi(phy_axis,0);
                    flag = true;
                    printf("---ResetAllAxisOutZero axis %d  speed zero\n",i);
                }
                else if(this->m_channel_status.cur_axis_ctrl_mode[i] == 3){
                    this->SendAxisTorqueCmdToMi(phy_axis,0,0,1); // 只设置速度限制值为0
                    flag = true;
                    printf("---ResetAllAxisOutZero axis %d  torque zero\n",i);
                }
            }
        }
    }
    return flag;
}

/**
 * @brief 复位处理控制模式，系统复位时，恢复通道内所有轴的控制模式为默认模式, 需要轴在不运动的时候再切换
 * @param flag: 强行切换标识， 0--等速度为0再切换   1--强行切换
 */

bool ChannelControl::ResetAllAxisCtrlMode(uint8_t flag){
    bool b_need_flag = false;
    int phy_axis;
    this->m_p_channel_engine->SendMonitorData(false, false);
    for(int i=0; i < this->m_p_channel_config->chn_axis_count; i++){
        phy_axis = this->GetPhyAxis(i);
        if(phy_axis != 0xff){
            if(this->m_channel_status.cur_axis_ctrl_mode[i] != m_p_axis_config[phy_axis].axis_type){

                if(this->m_channel_status.cur_axis_ctrl_mode[i] == 2 || this->m_channel_status.cur_axis_ctrl_mode[i] == 3){
                    double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(i);
                    int32_t feed = fabs(curfeed);
                    if(flag>0 || feed<5){  // 速度接近 0  或者为强行切换状态
                        this->m_channel_status.cur_axis_ctrl_mode[i] = m_p_axis_config[phy_axis].axis_type;  //记录当前轴控制模式
                        this->SendAxisCtrlModeSwitchCmdToMi(phy_axis, m_p_axis_config[phy_axis].axis_type);
                    }
                    else{
                        b_need_flag = true;
                    }
                }
            }
        }
    }
    return b_need_flag;
}
#endif

/**
 * @brief 处理物理轴和通道轴之间映射切换
 * @param msg
 */
void ChannelControl::ProcessAxisMapSwitch(AuxMsg *msg, uint8_t index){
    if(msg == nullptr)
        return;

    return;

    int mCode=-1;

    static uint8_t other_phy_axis,other_chan,other_chan_axis;
    static uint8_t cur_phy_axis,cur_chan,cur_chan_axis;


    mCode = msg->GetMCode(index);
    mCode -= 41000;
    cur_chan_axis = mCode%10-1;
    other_phy_axis = mCode/10-1;

    if(msg->GetExecStep(index) == 0){  // 第一步，把当前轴切换到PLC轴
        printf("execute M%d\n", mCode);

        cur_phy_axis = this->GetPhyAxis(cur_chan_axis);
        cur_chan = this->m_n_channel_index;
        other_chan = this->m_p_channel_engine->GetAxisChannel(other_phy_axis, other_chan_axis);

        if(cur_phy_axis==0xff) return;  // 轴不存在 直接返回
        if(cur_phy_axis==other_phy_axis) return;  // 切换目标轴与当前轴相同，直接返回

        if(cur_chan != other_chan){
            int64_t taxis = 0x01<<(cur_phy_axis-1);
            //this->m_n_real_phy_axis &= (~(taxis));
        }
        //	this->m_map_phy_axis_chn[m_p_channel_config[i].chn_axis_phy[j]-1] = i;
        //	this->m_channel_status.cur_chn_axis_phy[] = ;
        
        //   将当前轴对应的物理轴切到PLC通道
        this->SendAxisMapCmdToMi(cur_phy_axis, 0xFF,0);

        msg->IncreaseExecStep(index);  //此处必须加一，否则M指令即结束执行
    }else if(msg->GetExecStep(index) == 1){  // 第二步，把目标物理轴切到当前通道当前轴
        
        if(cur_chan != other_chan){
            int64_t taxis = 0x01<<(other_phy_axis-1);
            //this->m_n_real_phy_axis |= taxis;
            //		   this->m_map_phy_axis_chn[other_phy_axis-1] = i;
        }
        this->m_channel_status.cur_chn_axis_phy[cur_chan_axis] = other_phy_axis;
        //  将当前通道轴映射到目标物理轴
        this->SendAxisMapCmdToMi(other_phy_axis,this->m_n_channel_index,cur_chan_axis);

        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 2){   // 第三步，把第一步的物理轴由PLC通道切换到目标通道
        //
        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 3){


        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 4){

        //		msg->SetExecStep(index, 0);   //M指令执行结束
        msg->SetExecStep(index, 0xFF);    //置位结束状态
    }
}

/**
 * @brief 处理MI返回的跳转命令响应
 * @param cmd : MI命令包
 */
void ChannelControl::ProcessSkipCmdRsp(MiCmdFrame &cmd){
    uint8_t axis = cmd.data.data[5]-1;
    int64_t pos = 0;


    uint64_t tmp = 0x01;
    if(this->m_n_mask_pos_capture & (tmp<<axis)){
        this->m_n_mask_captured |= (tmp<<axis);   //设置该轴捕获成功

        memcpy(&pos, cmd.data.data, sizeof(int64_t));    //获取位置



        double df = pos;

        m_point_capture.m_df_point[axis] = df/1e7;   //转换单位  0.1nm-->mm
        printf("capture axis[%hhu] pos : %lf\n", axis, m_point_capture.m_df_point[axis]);
    }

    if(m_n_mask_captured == m_n_mask_pos_capture){
        this->m_b_pos_captured = true;    //位置捕获完成
    }
}

/**
 * @brief 手动对刀处理，使用MDA模式执行G37指令进行手动对刀
 * @param h_code ： 写入的刀偏H值
 * @param times : 重复对刀次数
 */
void ChannelControl::ManualToolMeasure(int h_code, int times){

    printf("ChannelControl::ManualToolMeasure: hcode=%d, times=%d\n", h_code, times);
    //准备对刀指令语句
    char cmd_buf[256];
    memset (cmd_buf, 0x00, 256);
    sprintf(cmd_buf, "G37 H%dL%d", h_code, times);

    //打开MDA对应的NC文件
    int fp = open(m_str_mda_path, O_CREAT|O_TRUNC|O_WRONLY); //打开文件
    if(fp < 0){
        CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
        this->m_error_code = ERR_OPEN_FILE;
        g_ptr_trace->PrintLog(LOG_ALARM, "打开MDA临时文件[%s]失败！", m_str_mda_path);
        return;//文件打开失败
    }
    else{
        printf("open file %s\n", m_str_mda_path);
    }

    int len = strlen(cmd_buf);
    ssize_t res = write(fp, cmd_buf, len);
    if(res == -1){//写入失败
        close(fp);
        CreateError(ERR_LOAD_MDA_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
        this->m_error_code = ERR_LOAD_MDA_DATA;
        return;
    }else if(res != len){//数据缺失
        close(fp);
        CreateError(ERR_LOAD_MDA_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
        this->m_error_code = ERR_LOAD_MDA_DATA;
        printf("write mda temp file error, plan = %d, actual = %d\n", len, res);
        return;
    }

    close(fp);

    this->m_p_compiler->SetCurPos(this->m_channel_rt_status.cur_pos_work);

    if(!m_p_compiler->OpenFile(m_str_mda_path)){		//编译器打开文件失败
        return;
    }

    printf("ManualToolMeasure() m_n_run_thread_state: %d\n", m_n_run_thread_state);
    if(m_n_run_thread_state == IDLE || m_n_run_thread_state == PAUSE){
        pthread_mutex_lock(&m_mutex_change_state);
        //printf("locked 16\n");
        m_n_run_thread_state = RUN;  //置为运行状态
        this->m_b_manual_tool_measure = true;
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 16\n");
    }
}

/**
 * @brief 获取指定刀偏的数据
 * @param idx[in] : 刀偏值，从1开始, 0是基准刀偏
 * @param cfg[out] ： 返回刀偏数据
 */
void ChannelControl::GetHmiToolOffset(const uint8_t idx, HmiToolOffsetConfig &cfg){
    uint8_t index = 0;
#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
    if(idx == 0){
        cfg.geometry_compensation[0] = this->m_p_chn_tool_config->geometry_comp_basic[0];
        cfg.geometry_compensation[1] = this->m_p_chn_tool_config->geometry_comp_basic[1];
        cfg.geometry_compensation[2] = this->m_p_chn_tool_config->geometry_comp_basic[2];
        cfg.geometry_wear = 0;
        cfg.radius_compensation = 0;
        cfg.radius_wear = 0;
    }else if(idx > 0 && idx <= kMaxToolCount){
#else
    if(idx > 0 && idx <= kMaxToolCount){
#endif
        index = idx-1;
    }
    cfg.geometry_compensation[0] = this->m_p_chn_tool_config->geometry_compensation[index][0];
    cfg.geometry_compensation[1] = this->m_p_chn_tool_config->geometry_compensation[index][1];
    cfg.geometry_compensation[2] = this->m_p_chn_tool_config->geometry_compensation[index][2];
    cfg.geometry_wear = this->m_p_chn_tool_config->geometry_wear[index];
    cfg.radius_compensation = this->m_p_chn_tool_config->radius_compensation[index];
    cfg.radius_wear = this->m_p_chn_tool_config->radius_wear[index];
}

/**
 * @brief 通知HMI刀具偏置参数值发生变更
 * @param h_idx : H值索引，从1开始
 */
bool ChannelControl::NotifyHmiToolOffsetChanged(uint8_t h_idx){     //
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_PARAM_CHANGED;
    cmd.cmd_extension = TOOL_OFFSET_CONFIG;
    cmd.data_len = 1+sizeof(HmiToolOffsetConfig);
    cmd.data[0] = h_idx;
    HmiToolOffsetConfig tool_offset;
    this->GetHmiToolOffset(h_idx, tool_offset);
    memcpy(&cmd.data[1], &tool_offset, sizeof(HmiToolOffsetConfig));

    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 通知HMI刀具信息发生改变
 * @param tool_index : 值索引
 */
bool ChannelControl::NotifyHmiToolPotChanged()
{
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_PARAM_CHANGED;
    cmd.cmd_extension = TOOL_POT_CONFIG;
    cmd.data_len = sizeof(HmiToolPotConfig);
    memcpy(&cmd.data[0], g_ptr_parm_manager->GetToolPotConfig(cmd.channel_index), sizeof(HmiToolPotConfig));

    return this->m_p_hmi_comm->SendCmd(cmd);
}


/**
 * @brief 通知HMI工件坐标系设置发生变更
 * @param coord_idx : 工件坐标系索引，0--基本偏移   1~6--G54~G59
 * @return true--成功  false--失败
 */
bool ChannelControl::NotifyHmiWorkcoordChanged(uint8_t coord_idx){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_PARAM_CHANGED;
    cmd.cmd_extension = COORD_CONFIG;
    cmd.data_len = 1+sizeof(HmiCoordConfig);
    cmd.data[0] = coord_idx;
    memcpy(&cmd.data[1], &this->m_p_chn_coord_config[coord_idx], sizeof(HmiCoordConfig));

    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 通知HMI扩展坐标系设置发生变更
 * @param coord_idx : 工件坐标系索引，0--基本偏移   1~99--G5401~G5499
 * @return true--成功  false--失败
 */
bool ChannelControl::NotifyHmiWorkcoordExChanged(uint8_t coord_idx){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_PARAM_CHANGED;
    cmd.cmd_extension = EX_COORD_CONFIG;
    cmd.data_len = 1+sizeof(HmiCoordConfig);
    cmd.data[0] = coord_idx;
    memcpy(&cmd.data[1], &this->m_p_chn_ex_coord_config[coord_idx], sizeof(HmiCoordConfig));


    return this->m_p_hmi_comm->SendCmd(cmd);
}

bool ChannelControl::NotifyHmiMCode(int mcode){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_NOTIFY_MCODE;
    cmd.data_len = sizeof(int);
    memcpy(&cmd.data, &mcode, sizeof(int));

    return this->m_p_hmi_comm->SendCmd(cmd);
};



/**
 * @brief 判断是否缓冲清空消息
 * @param msg : 消息指针
 * @return true--需要清空缓冲    false--不需要清空缓冲
 */
bool ChannelControl::IsBufClearMsg(RecordMsg *msg){
    bool res = true;
    int type = msg->GetMsgType();
    if(type == LINE_MSG || type == ARC_MSG || type == RAPID_MSG || type == FEED_MSG ||
            type == SPEED_MSG || type == COMPENSATE_MSG || type == LOOP_MSG ||
            type == COORD_MSG || type == SUBPROG_CALL_MSG || type == MACRO_MSG ||
            type == SUBPROG_RETURN_MSG || type == MACRO_PROG_CALL_MSG ){
        res = false;
    }else if(type == AUX_MSG){  //TODO M03/M04/M05主轴指令允许回退

    }else if(type == MODE_MSG){   //TODO 支持其它模态消息
        //		int gcode = ((ModeMsg *)msg)->GetGCode();
        res = false;
    }


    return res;
}

/**
 * @brief 是否轴移动消息(G00、G01、G02、G03)
 * @param msg : 消息指针
 * @return shi
 */
bool ChannelControl::IsRunmoveMsg(RecordMsg *msg){
    bool res = false;
    int type = msg->GetMsgType();
    if(type == LINE_MSG || type == ARC_MSG || type == RAPID_MSG)
        res = true;
    return res;
}

/**
 * @brief 在自动数据缓冲中查找指定帧号的数据
 * @param index : 帧序号
 * @return 找到的msg指针
 */
ListNode<RecordMsg *> *ChannelControl::FindMsgNode(uint16_t index){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "ChannelControl::FindMsgNode, find=%hu, count=%d\n",
                            index, this->m_p_output_msg_list_auto->GetLength());

    ListNode<RecordMsg *> *node = this->m_p_output_msg_list_auto->HeadNode();

    if(node != nullptr)
        printf("head node : framidx=%hu, line=%llu\n", node->data->GetFrameIndex(), node->data->GetLineNo());

    int count = 0;
    while(node != nullptr){
        if(node->data->GetFrameIndex() == index){
            g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "ChannelControl::FindMsgNode, frame=%hu, line=%llu, count = %d\n",
                                    index, node->data->GetLineNo(), count);
            break;
        }
        node = node->next;
        count ++;
    }

    printf("find msg frame index count = %d\n", count);
    return node;
}

/**
 * @brief 处理MI发送的手轮跟踪状态切换指令
 * @param cmd : MI发送的指令
 */
void ChannelControl::ProcessMiHWTraceStateChanged(MiCmdFrame &cmd){
    cmd.data.cmd |= 0x8000;    //回复帧
    HWTraceState state_new = (HWTraceState)cmd.data.data[0];   //新状态

    printf("ProcessMiHWTraceStateChanged: newstate = %d\n", (int)state_new);

    if(/*m_channel_status.chn_work_mode != AUTO_MODE ||*/   //处理反向跟踪时直接切换模式，此处不能限制模式
            (cmd.data.data[0] == REVERSE_TRACE && this->m_p_general_config->hw_rev_trace == 0)){
        cmd.data.data[1] = 0x00;   //失败，反向引导功能关闭

        this->m_p_mi_comm->WriteCmd(cmd);
    }else{
        cmd.data.data[1] = 0x01;  //切换成功

        if(this->m_n_hw_trace_state == NONE_TRACE && state_new == NORMAL_TRACE){  //非手轮跟踪模式 -->  正向手轮跟踪模式
            this->m_n_hw_trace_state = NORMAL_TRACE;

            this->m_p_mi_comm->WriteCmd(cmd);

        }else if(this->m_n_hw_trace_state == NORMAL_TRACE && state_new == NONE_TRACE){  //正向手轮跟踪模式  --> 非手轮跟踪模式
            this->m_n_hw_trace_state = NONE_TRACE;

            this->m_p_mi_comm->WriteCmd(cmd);

        }else if(this->m_n_hw_trace_state == NORMAL_TRACE && state_new == REVERSE_TRACE){  //正向手轮跟踪模式  --> 反向手轮跟踪模式
            if(m_channel_status.machining_state != MS_RUNNING ||
                    (m_channel_mc_status.cur_mode ==MC_MODE_MANUAL && m_channel_mc_status.cur_feed < 3)){ //满足切换条件
                this->ChangeHwTraceState(state_new);

                this->m_p_mi_comm->WriteCmd(cmd);

            }else{ //不满足切换条件，等待MC暂停到位
                this->m_n_hw_trace_state_change_to = state_new;
                this->m_b_change_hw_trace_state = true;
            }


        }else if(this->m_n_hw_trace_state == REVERSE_TRACE && state_new == NORMAL_TRACE){  //反向手轮跟踪模式  --> 正向手轮跟踪模式
            if(m_channel_status.machining_state != MS_RUNNING ||
                    (m_channel_mc_status.cur_mode ==MC_MODE_MANUAL && m_channel_mc_status.cur_feed < 3)){ //满足切换条件
                this->ChangeHwTraceState(state_new);

                this->m_p_mi_comm->WriteCmd(cmd);

            }else{ //不满足切换条件，等待MC暂停到位
                this->m_n_hw_trace_state_change_to = state_new;
                this->m_b_change_hw_trace_state = true;
            }
        }else{  //其它类型的切换属于非法
            cmd.data.data[1] = 0x00;   //失败，反向引导功能关闭

            this->m_p_mi_comm->WriteCmd(cmd);
        }

    }
}

/**
 * @brief 切换手轮跟踪状态
 * @param state : 待切换的新状态
 * @return true--成功   false--失败
 */
bool ChannelControl::ChangeHwTraceState(HWTraceState state){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "ChangeHwTraceState: newstate = %d\n", (int)state);
    //同步当前位置
    if(!this->m_b_mc_on_arm)
        this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
    else
        this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
    this->RefreshAxisIntpPos();

    //读取当前帧号
    uint16_t cur_frame_idx = 0;
    if(!this->m_b_mc_on_arm)
        this->m_p_mc_comm->ReadChnCurFrameIndex(m_n_channel_index, cur_frame_idx);
    else
        this->m_p_mc_arm_comm->ReadChnCurFrameIndex(m_n_channel_index, cur_frame_idx);


    //找到当前数据帧
    ListNode<RecordMsg *> *node = this->FindMsgNode(cur_frame_idx);
    if(m_channel_status.machining_state == MS_RUNNING && node == nullptr && this->m_p_output_msg_list_auto->GetLength() > 0){  //运行中
        printf("Failed to find cmd frame index = %hu\n", cur_frame_idx);
        this->m_error_code = ERR_NO_CUR_RUN_DATA;
        CreateError(ERR_NO_CUR_RUN_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        return false;
    }

    pthread_mutex_lock(&m_mutex_change_state);
    //printf("locked 17\n");
    //清空当前MC中的自动缓冲数据
    this->InitMcIntpAutoBuf();

    //重置缓冲发送指针以及手轮跟踪状态
    if(state == NORMAL_TRACE){
        if(node != nullptr)
            this->m_p_last_output_msg = node->pre;
        g_ptr_alarm_processor->RemoveWarning(this->m_n_channel_index, ERR_HW_REV_OVER);    //删除反向告警
    }
    else if(state == REVERSE_TRACE){
        if(node != nullptr)
            this->m_p_last_output_msg = node->next;
    }
    this->m_n_hw_trace_state = state;

    pthread_mutex_unlock(&m_mutex_change_state);
    //printf("unlocked 17\n");


    // 发送启动消息
    //	this->StartMcIntepolate();
    m_b_mc_need_start = true;

    return true;
}

/**
 * @brief 读取图形模式下的高频轴位置数据, 读取XYZ三个轴的反馈位置
 */
void ChannelControl::ReadGraphAxisPos(){
    if(this->m_n_graph_pos_count == kGraphPosBufCount-1){
        //缓冲已满，先发送
        this->SendHmiGraphPosData();
    }

    this->m_p_mi_comm->ReadPhyAxisFbPos(&m_pos_graph_array[m_n_graph_pos_write_idx].x, this->m_n_xyz_axis_phy, 3);

    if(++m_n_graph_pos_write_idx >= kGraphPosBufCount)
        m_n_graph_pos_write_idx = 0;

    this->m_n_graph_pos_count++;
}

/**
 * @brief 给HMI发送绘图位置数据
 */
void ChannelControl::SendHmiGraphPosData(){
    if(this->m_n_graph_pos_count == 0)
        return;   //无数据

    uint8_t data_type = MDT_CHN_AXIS_POS;
    uint8_t chn_index = this->m_n_channel_index;
    uint16_t count = this->m_n_graph_pos_count;
    uint16_t data_len = sizeof(CoordAxisPos)*count;


    int buf_len = 4+data_len;
    char buffer[buf_len];
    memset(buffer, 0x00, 64);
    memcpy(buffer, &data_type, 1);
    memcpy(buffer+1, &chn_index, 1);
    memcpy(buffer+2, &data_len, 2);

    //拷贝数据
    int cc = kGraphPosBufCount-m_n_graph_pos_read_idx;
    if(cc >= count){
        memcpy(buffer+4, &m_pos_graph_array[m_n_graph_pos_read_idx], data_len);
        m_n_graph_pos_read_idx += count;
        if(m_n_graph_pos_read_idx >=  kGraphPosBufCount)
            m_n_graph_pos_read_idx = 0;

    }else{
        memcpy(buffer+4, &m_pos_graph_array[m_n_graph_pos_read_idx], cc*sizeof(CoordAxisPos));

        int c2 = count -cc;
        memcpy(buffer+4+cc*sizeof(CoordAxisPos), &m_pos_graph_array[0], c2*sizeof(CoordAxisPos));
        m_n_graph_pos_read_idx += count;
        if(m_n_graph_pos_read_idx >=  kGraphPosBufCount)
            m_n_graph_pos_read_idx -= kGraphPosBufCount;
    }
    m_n_graph_pos_count -= count;

    this->m_p_hmi_comm->SendMonitorData(buffer, buf_len);
}

/**
 * @brief 调用宏程序
 * @param macro_index : 宏程序编号
 */
bool ChannelControl::CallMacroProgram(uint16_t macro_index){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "ChannelControl[%hhu]::CallMacroProgram, macro_index = %hu\n", this->m_n_channel_index, macro_index);
    uint8_t mode = this->m_channel_status.chn_work_mode;
    uint8_t state = this->m_channel_status.machining_state;
    //自动、MDA模式(必须在MS_RUNNING状态)
    if((mode == AUTO_MODE || mode == MDA_MODE) && state == MS_RUNNING){
        this->m_p_compiler->CallMarcoProgWithNoPara(macro_index);
        m_n_subprog_count++;
        m_n_macroprog_count++;

        if(this->m_p_general_config->debug_mode == 0){
            this->SetMcStepMode(false);
        }

    }else if(mode == MANUAL_STEP_MODE || mode == MANUAL_MODE || mode == MPG_MODE ||
             mode == REF_MODE || state != MS_RUNNING){  //手动、手轮模式（非运行状态亦可）
        if(this->m_b_manual_call_macro){  //已经处于手动宏程序调用中
            g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "已处于手动宏程序调用中执行过程！[%hu]", macro_index);
            return false;
        }
        if(!m_p_compiler->FindSubProgram(macro_index, true)){
            return false;
        }

        char cmd_buf[256];
        memset (cmd_buf, 0x00, 256);
        sprintf(cmd_buf, "G65P%hu", macro_index);

        //打开MDA对应的NC文件
        int fp = open(m_str_mda_path, O_CREAT|O_TRUNC|O_WRONLY); //打开文件
        if(fp < 0){
            CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
            this->m_error_code = ERR_OPEN_FILE;
            g_ptr_trace->PrintLog(LOG_ALARM, "打开MDA临时文件[%s]失败！", m_str_mda_path);
            return false;//文件打开失败
        }
        else{
            printf("open file %s\n", m_str_mda_path);
        }

        int len = strlen(cmd_buf);
        ssize_t res = write(fp, cmd_buf, len);
        if(res == -1){//写入失败
            close(fp);
            CreateError(ERR_LOAD_MDA_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
            this->m_error_code = ERR_LOAD_MDA_DATA;
            return false;
        }else if(res != len){//数据缺失
            close(fp);
            CreateError(ERR_LOAD_MDA_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
            this->m_error_code = ERR_LOAD_MDA_DATA;
            printf("write mda temp file error, plan = %d, actual = %d\n", len, res);
            return false;
        }

        close(fp);

        this->m_p_compiler->SetCurPos(this->m_channel_rt_status.cur_pos_work);

        if(!m_p_compiler->OpenFile(m_str_mda_path)){		//编译器打开文件失败
            printf("CallMacroProgram:compiler open file failed\n");
            return false;
        }

        printf("CallMacroProgram() m_n_run_thread_state: %d\n", m_n_run_thread_state);
        if(m_n_run_thread_state == IDLE || m_n_run_thread_state == PAUSE){
            pthread_mutex_lock(&m_mutex_change_state);
            //printf("locked 18\n");
            m_n_run_thread_state = RUN;  //置为运行状态
            this->m_b_manual_call_macro = true;
            pthread_mutex_unlock(&m_mutex_change_state);
            //printf("unlocked 18\n");
        }
    }else{
        return false;
    }
    return true;
}

#ifdef USES_ADDITIONAL_PROGRAM
/**
 * @brief 调用附加程序（前置/后置）
 * @param type : 调用类型， 1--加工启动前置程序   2--断点继续前置程序   3--加工复位前置程序    11--结束后置程序    12--暂停后置程序    13--复位后置程序
 * @return  true--成功   false--失败
 */
bool ChannelControl::CallAdditionalProgram(AddProgType type){

    if(this->m_n_add_prog_type != NONE_ADD){
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "已处于附加程序执行过程！[%hhu:%hhu]", m_n_add_prog_type, type);

        return false;
    }

    if(this->m_channel_status.chn_work_mode != AUTO_MODE){
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "非自动模式，拒绝执行附件程序！[当前模式：%hhu]", m_channel_status.chn_work_mode);

        return false;
    }

    this->m_n_add_prog_type = type;
    if(type == NORMAL_START_ADD || type == RESET_START_ADD){
        //		if(this->m_channel_status.machining_state != MS_RUNNING){  //非运行状态，不响应
        //			return false;
        //		}

        this->m_n_sub_count_bak = m_n_subprog_count;   //记录子程序嵌套层数

        if(type == NORMAL_START_ADD)
            this->m_p_compiler->CallMarcoProgWithNoPara(9701, false);
        else if(type == RESET_START_ADD)
            this->m_p_compiler->CallMarcoProgWithNoPara(9703, false);
        m_n_subprog_count++;
        m_n_macroprog_count++;



        //		if(this->m_p_general_config->debug_mode == 0){
        this->SetMcStepMode(false);
        //		}

    }else if(type == CONTINUE_START_ADD){  //在MDA通道执行
        char cmd_buf[256];
        memset (cmd_buf, 0x00, 256);
        sprintf(cmd_buf, "G65P%hu", 9702);

        //打开MDA对应的NC文件
        int fp = open(m_str_mda_path, O_CREAT|O_TRUNC|O_WRONLY); //打开文件
        if(fp < 0){
            CreateError(ERR_EXEC_FORWARD_PROG, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 1, m_n_channel_index);
            this->m_error_code = ERR_EXEC_FORWARD_PROG;
            g_ptr_trace->PrintLog(LOG_ALARM, "打开MDA临时文件[%s]失败！", m_str_mda_path);
            return false;//文件打开失败
        }

        int len = strlen(cmd_buf);
        ssize_t res = write(fp, cmd_buf, len);
        if(res == -1){//写入失败
            close(fp);
            CreateError(ERR_EXEC_FORWARD_PROG, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 2, m_n_channel_index);
            this->m_error_code = ERR_EXEC_FORWARD_PROG;
            g_ptr_trace->PrintLog(LOG_ALARM, "写入MDA临时文件[%s]失败！", m_str_mda_path);
            return false;
        }else if(res != len){//数据缺失
            close(fp);
            CreateError(ERR_EXEC_FORWARD_PROG, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 3, m_n_channel_index);
            this->m_error_code = ERR_EXEC_FORWARD_PROG;
            g_ptr_trace->PrintLog(LOG_ALARM, "写入MDA临时文件[%s]失败！[%d:%d]", m_str_mda_path, len, res);
            return false;
        }

        close(fp);

        this->SaveAutoScene(false);   //保存

        //编译器切换为MDA模式
        pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止
        m_p_output_msg_list = m_p_output_msg_list_mda;
        m_n_run_thread_state = IDLE;
        this->m_p_compiler->SetMode(MDA_COMPILER);	//编译器切换模式
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //切换输出队列
        pthread_mutex_unlock(&m_mutex_change_state);

        //初始化传递给MC模块的模态组信息
        this->InitMcModeStatus();
        this->m_p_compiler->SetCurPos(this->m_channel_rt_status.cur_pos_work);

        if(!m_p_compiler->OpenFile(m_str_mda_path)){		//编译器打开文件失败

            //恢复自动模式
            pthread_mutex_lock(&m_mutex_change_state);  //等待编译运行线程停止
            m_p_output_msg_list = m_p_output_msg_list_auto;
            m_n_run_thread_state = PAUSE;
            this->m_p_compiler->SetMode(AUTO_COMPILER);	//编译器切换模式
            this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //切换输出队列
            pthread_mutex_unlock(&m_mutex_change_state);

            CreateError(ERR_EXEC_FORWARD_PROG, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 4, m_n_channel_index);
            this->m_error_code = ERR_EXEC_FORWARD_PROG;
            g_ptr_trace->PrintLog(LOG_ALARM, "编译器打开MDA文件[%s]失败！", m_str_mda_path);

            return false;
        }

        this->m_n_subprog_count = 0;
        this->m_n_macroprog_count = 0;

        this->InitMcIntpMdaBuf();   //初始化MC模块的MDA缓冲

        this->SetMcStepMode(false);

        this->SetCurLineNo(this->m_channel_rt_status.line_no);
        if(m_n_run_thread_state == IDLE || m_n_run_thread_state == PAUSE){
            pthread_mutex_lock(&m_mutex_change_state);
            m_n_run_thread_state = RUN;  //置为运行状态
            pthread_mutex_unlock(&m_mutex_change_state);
            printf("m_n_run_thread_state = %d\n", m_n_run_thread_state);
        }

    }else if(type == RESET_START_ADD){

    }else if(type == NORMAL_END_ADD){

    }else if(type == PAUSE_ADD){//在MDA通道执行

    }else if(type == RESET_ADD){

    }else{
        this->m_n_add_prog_type = NONE_ADD;
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "不支持的附加程序模式！[%hhu]", type);
        return false;
    }


    return true;
}
#endif

/**
 * @brief 输出调试数据
 */
void ChannelControl::PrintDebugInfo(){
    printf("CHN [%hhu] DEBUG INFO\n", m_n_channel_index);
    uint32_t fifo_count = this->m_p_mc_comm->ReadGCodeFifoCount(m_n_channel_index);
    uint16_t mc_buf_count = 0;
    this->m_p_mc_comm->ReadAutoBufDataCount(m_n_channel_index, mc_buf_count);
    printf("outputbuf = %d, fifo_count = %u, buf_count = %hu, compiler_run_state = %hhu, mach_state = %hhu, hw_trace=%hhu, err = %d\n", this->m_p_output_msg_list_auto->GetLength(), fifo_count, mc_buf_count, m_n_run_thread_state,
           m_channel_status.machining_state, m_channel_status.func_state_flags.CheckMask(FS_HANDWHEEL_CONTOUR), m_error_code);

    printf("mc status: cur_cmd = %hu, cur_mode=%hu, cur_feed = %u, rated_feed = %u, axis_over_mask = 0x%x\n", m_channel_mc_status.cur_cmd,
           m_channel_mc_status.cur_mode, m_channel_mc_status.cur_feed, m_channel_mc_status.rated_feed, m_channel_mc_status.axis_over_mask);

    /********************************************************************输出MC模块debug数据************************************************************/
    uint32_t debug[16];
    memset(debug, 0x00, 64);
    this->m_p_mc_comm->ReadDebugData(debug);
    printf("MC debug info: TTT1 = %d, TTT2 = %d, TTT3 = %d, TTT4 = %d, TTT5 = %d, TTT6 = %d, TTT7 = %d, TTT8 = %d\n", debug[0], debug[1],
            debug[2], debug[3], debug[4], debug[5], debug[6], debug[7]);
    printf("MC debug info: TTT9 = %d, TTT10 = %d, TTT11 = %d, TTT12 = %d, TTT13 = %d, TTT14 = %d, TTT15 = %d, TTT16 = %d\n", debug[8],
            debug[9], debug[10], debug[11], debug[12], debug[13], debug[14], debug[15]);

}

/**
 * @brief 输出调试数据
 */
void ChannelControl::PrintDebugInfo1(){
    printf("CHN [%hhu] DEBUG INFO\n", m_n_channel_index);
    uint16_t mc_buf_max_count = this->GetMcAutoBufMax();
    uint16_t mc_buf_count = 0;
    this->m_p_mc_arm_comm->ReadAutoBufDataCount(m_n_channel_index, mc_buf_count);		;
    printf("outputbuf = %d, mc_buf_max_count = %u, buf_count = %hu, compiler_run_state = %hhu, mach_state = %hhu, err = %d\n", this->m_p_output_msg_list_auto->GetLength(), mc_buf_max_count, mc_buf_count, m_n_run_thread_state,
           m_channel_status.machining_state, m_error_code);
    printf("mc status: cur_cmd = %hu, cur_mode=%hu, cur_feed = %u, rated_feed = %u, axis_over_mask = 0x%x\n", m_channel_mc_status.cur_cmd,
           m_channel_mc_status.cur_mode, m_channel_mc_status.cur_feed, m_channel_mc_status.rated_feed, m_channel_mc_status.axis_over_mask);

}


// @test zk
void ChannelControl::test(){
	CreateError(33002, INFO_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
	//StartMcIntepolate();
    //StraightTraverse(0,100,100,100);
}


// @add zk
// 直接往mc运动队列中加入 G01 数据   自动模式下会产生运动  其他模式下会放入数据并在启动时运动
void ChannelControl::StraightFeed(int chn, double x, double y, double z, int feed)
{
    GCodeFrame frame;
    memset(&frame, 0x0, sizeof(frame));
    frame.data.frame_index = 0;
    frame.data.cmd = 1;
    frame.data.ext_type = 0x21;
    frame.data.feed = FEED_TRANS(feed);
    frame.data.pos0 = MM2NM0_1(x);
    frame.data.pos1 = MM2NM0_1(y);
    frame.data.pos2 = MM2NM0_1(z);
    m_p_mc_comm->WriteGCodeData(chn, frame);
}

// 直接往mc运动队列中加入 G00 数据  作用效果同上
void ChannelControl::StraightTraverse(int chn, double x, double y, double z)
{
    GCodeFrame frame;
    memset(&frame, 0x0, sizeof(frame));
    frame.data.frame_index = 0;
    frame.data.cmd = 0;
    frame.data.ext_type = 0x21;
    frame.data.feed = FEED_TRANS(0);
    frame.data.pos0 = MM2NM0_1(x);
    frame.data.pos1 = MM2NM0_1(y);
    frame.data.pos2 = MM2NM0_1(z);
    m_p_mc_comm->WriteGCodeData(chn, frame);
}

void ChannelControl::g73_func(){
    StartMcIntepolate();

    //double data;

    bool flag = false;

    double x_value, y_value, z_value, r_value, q_value, d_value, f_value;
    double init_z_plane, z_cur;  //初始平面

    m_macro_variable.GetVarValue(179, f_value, flag);
    if(!flag or f_value <= 0){
        f_value = m_p_compiler->GetFValue();
    }

    m_macro_variable.GetVarValue(194, x_value, flag);
    m_macro_variable.GetVarValue(195, y_value, flag);
    m_macro_variable.GetVarValue(196, z_value, flag);
    m_macro_variable.GetVarValue(177, d_value, flag);
    m_macro_variable.GetVarValue(188, r_value, flag);
    m_macro_variable.GetVarValue(187, q_value, flag);

    init_z_plane = GetAxisCurWorkPos(2);

    // 1
    StraightTraverse(0, x_value, y_value, init_z_plane);

    // 2
    StraightTraverse(0, x_value, y_value, r_value);

    z_cur = r_value - q_value;

    while(z_cur > z_value){

        StraightFeed(0, x_value, y_value, z_cur, f_value);

        z_cur += d_value;

        StraightTraverse(0, x_value, y_value, z_cur);

        z_cur = z_cur - d_value - q_value;
    }

    if(z_cur != z_value){
        StraightFeed(0, x_value, y_value, z_value, f_value);
    }

    StraightTraverse(0, x_value, y_value, init_z_plane);

}
// @add zk

