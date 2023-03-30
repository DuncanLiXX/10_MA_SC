/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file ChannelControl.cpp
 *@author gonghao
 *@date 2020/03/19
 *@brief ���ļ�Ϊͨ���������ʵ��
 *ͨ�����������Ҫ�����ǹ���ͨ��״̬������ͨ����������
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

int ctrlmode_switch_wait = 0;   //������ģʽ�л��ȴ���ʱ

/**
 * @brief ���캯��
 */
ChannelControl::ChannelControl() {
    // TODO Auto-generated constructor stub
    m_p_compiler = nullptr;
    m_n_channel_index = 0;   //Ĭ��ͨ��������Ϊ0

    m_b_lineno_from_mc = false;   //Ĭ�ϴ�MCģ���ȡ��ǰ���д�����к�
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

    m_b_hmi_graph = false;     //Ĭ�Ϸ�ͼ��ģʽ

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
    m_n_add_prog_type = NONE_ADD;        //��ǰ���ӳ���ִ��״̬
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

    gettimeofday(&m_time_start_mc, nullptr);  //��ʼ��MC����ʱ��

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
    m_file_sim_data = -1;      //�������ݱ����ļ����
#endif
}

/**
 * @brief ��������
 */
ChannelControl::~ChannelControl() {
    // TODO Auto-generated destructor stub

    void* thread_result;
    int res = ERR_NONE;

    //�˳����������߳�
    res = pthread_cancel(m_thread_compiler);
    if (res != ERR_NONE) {
        printf("compile thread cancel failed\n");
    }

    //	usleep(1000);
    res = pthread_join(m_thread_compiler, &thread_result);//�ȴ��������߳��˳����
    if (res != ERR_NONE) {
        printf("compile thread join failed\n");
    }
    m_thread_compiler = 0;

    //�˳�״̬ˢ���߳�
    res = pthread_cancel(m_thread_refresh_status);
    if (res != ERR_NONE) {
        printf("status refresh thread cancel failed\n");
    }

    //	usleep(1000);
    res = pthread_join(m_thread_refresh_status, &thread_result);//�ȴ��������߳��˳����
    if (res != ERR_NONE) {
        printf("status refresh thread join failed\n");
    }
    m_thread_refresh_status = 0;

    if(m_p_output_msg_list_auto != nullptr){//����AUTOָ����Ϣ�������
        delete m_p_output_msg_list_auto;
        m_p_output_msg_list_auto = nullptr;
    }

    if(m_p_output_msg_list_mda != nullptr){//����MDAָ����Ϣ�������
        delete m_p_output_msg_list_mda;
        m_p_output_msg_list_mda = nullptr;
    }

    //�ͷű���������
    if(m_p_compiler != nullptr){
        delete m_p_compiler;
        m_p_compiler = nullptr;
    }

    //�ͷŴ�ӡ�߳�
    delete  &Singleton<ShowSc>::instance();

    pthread_mutex_destroy(&m_mutex_change_state);

}

/**
 * @brief ��G����ģ̬��ʼ����Ĭ��״̬
 */
void ChannelControl::InitGCodeMode(){
    m_channel_status.gmode[1] = G00_CMD;  //01��Ĭ��G00
    m_channel_status.gmode[2] = G17_CMD;  //02��Ĭ��G17
    m_channel_status.gmode[3] = G90_CMD;  //03��Ĭ��G90
    //	m_channel_status.gmode[4] = G23_CMD;
    m_channel_status.gmode[5] = G94_CMD;  //05��Ĭ��G94  ÿ���ӽ���
    m_channel_status.gmode[6] = G21_CMD;  //06��Ĭ��G21  ���Ƶ�λ

    m_channel_status.gmode[7] = G40_CMD;  //07��Ĭ��G40
    m_channel_status.gmode[8] = G49_CMD;  //08��Ĭ��G49
    m_channel_status.gmode[9] = G80_CMD;  //09��Ĭ��G80
    m_channel_status.gmode[10] = G98_CMD;  //10��Ĭ��G98  ���س�ʼƽ��
    m_channel_status.gmode[11] = G50_CMD;  //11��Ĭ��G50  ��������ȡ��
    m_channel_status.gmode[14] = G54_CMD;  //14��Ĭ��G54  G54��������ϵ
    m_channel_status.gmode[15] = G64_CMD;  //15��Ĭ��G64  ������ʽ
    m_channel_status.gmode[16] = G69_CMD;  //16��Ĭ��G69  ������ת������ά����任��ʽOFF

    m_channel_status.gmode[17] = G15_CMD;  //17��Ĭ��15   ������ָ��ȡ��
    m_channel_status.gmode[19] = G26_CMD;  //19��Ĭ��G26  �����ٶȱ䶯���ON
}


/**
 * @brief ��ʼ������
 * @param chn_index : ͨ��������
 * @param engine : ͨ������ָ��
 * @param hmi_comm ��HMIͨѶ�ӿ�ָ��
 * @param parm ���������ʽӿ�ָ��
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

    //��ʼ���زο����־
    m_channel_status.returned_to_ref_point = 0x00;
    uint8_t phy_axis = 0;
    for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        phy_axis = m_p_channel_config->chn_axis_phy[i];
        if(phy_axis == 0){//δ����
            m_channel_status.returned_to_ref_point |= (0x01<<i);
            continue;
        }
        if(m_p_axis_config[phy_axis-1].axis_interface == VIRTUAL_AXIS   //�����᲻�ý����ο���
            || m_p_axis_config[phy_axis-1].axis_type == AXIS_SPINDLE	//���᲻�ý���������
            //|| (m_p_axis_config[phy_axis-1].feedback_mode == ABSOLUTE_ENCODER && m_p_axis_config[phy_axis-1].ref_encoder != kAxisRefNoDef))  //�Ѿ������ο���ľ���ʽ�������������ٴν����ο���
            || (m_p_axis_config[phy_axis-1].feedback_mode == ABSOLUTE_ENCODER && m_p_axis_config[phy_axis-1].ref_complete != 0))  //�Ѿ������ο���ľ���ʽ�������������ٴν����ο���
        {
            m_channel_status.returned_to_ref_point |= (0x01<<i);
        }
        if(m_p_axis_config[phy_axis-1].axis_interface != VIRTUAL_AXIS)
        {
            m_n_real_phy_axis = m_n_real_phy_axis | 0x01<<(phy_axis-1);    //��ʼ��ʵ��������mask
        }

        uint8_t z_axis =  this->GetPhyAxisFromName(AXIS_NAME_Z);
        uint32_t da_prec = m_p_channel_engine->GetDaPrecision();

        //��ʼ��������Ϣ
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

        //PMC�᲻�����Ｄ�ͨ����ͼ������
        this->m_mask_intp_axis |= (0x01<<i);
        this->m_n_intp_axis_count++;

#ifdef USES_SPEED_TORQUE_CTRL
        // �����ģʽ����ʼ�����õ���ǰ״̬
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
    this->GetMdaFilePath(m_str_mda_path);   //��ʼ��mda�ļ�·��

    m_channel_rt_status.machining_time_total = g_ptr_parm_manager->GetCurTotalMachingTime(chn_index);

#ifdef USES_GRIND_MACHINE
    m_p_grind_config = parm->GetGrindConfig();       //ĥ������
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
    m_n_frame_index = 1;    //��ǰ�˶�����֡��Ĭ��Ϊ0
#ifdef USES_SIMULATION_TEST
    m_simulate_mode = SIM_OUTLINE;    //Ĭ������������ģʽ
#else
    m_simulate_mode = SIM_NONE;    //Ĭ���ڷǷ���ģʽ
#endif

    this->m_b_mc_on_arm = this->m_p_channel_engine->IsMcArmChn(this->m_n_channel_index);

    m_scene_auto.need_reload_flag = false;
    m_scene_auto.machining_state = MS_READY;   //��ʼ��ΪĬ��Ϊ����

    this->InitialChannelStatus();

    //��ʼ��XYZ���Ӧ��������
    m_n_xyz_axis_phy[0] = this->GetPhyAxisFromName(AXIS_NAME_X);
    m_n_xyz_axis_phy[1] = this->GetPhyAxisFromName(AXIS_NAME_Y);
    m_n_xyz_axis_phy[2] = this->GetPhyAxisFromName(AXIS_NAME_Z);

    m_n_graph_pos_count = 0;
    m_n_graph_pos_write_idx = 0;
    m_n_graph_pos_read_idx = 0;


    //�����Զ�ģʽ���ָ����Ϣ�б�
    this->m_p_output_msg_list_auto = new OutputMsgList(kMaxOutputBufCount);
    if(this->m_p_output_msg_list_auto == nullptr){
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�����������Զ�ģʽ���ָ����Ϣ����ʧ��!", m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //��ʼ��ʧ��
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER, 0, m_n_channel_index);
        m_n_run_thread_state = ERROR;
        return false;
    }

    //����MDAģʽ���ָ����Ϣ�б�
    this->m_p_output_msg_list_mda = new OutputMsgList();
    if(this->m_p_output_msg_list_mda == nullptr){
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]����������MDAģʽ���ָ����Ϣ����ʧ��!", m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //��ʼ��ʧ��
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER, 0, m_n_channel_index);
        m_n_run_thread_state = ERROR;
        return false;
    }

    m_p_output_msg_list = m_p_output_msg_list_auto;   //Ĭ��ָ���Զ��������

    pthread_mutex_init(&m_mutex_change_state, nullptr);

    //��������������
    m_p_compiler = new Compiler(this);
    if(m_p_compiler == nullptr){
        m_error_code = ERR_MEMORY_NEW;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER, 0, m_n_channel_index);
        return false;
    }
    m_p_compiler->SetOutputMsgList(m_p_output_msg_list);
    m_p_compiler->SetAxisNameEx(g_ptr_parm_manager->GetSystemConfig()->axis_name_ex);

    //���������߳�
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//
    param.__sched_priority = 35; //95;
    pthread_attr_setschedparam(&attr, &param);
    //	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
    //	if (res) {
    //		printf("pthread setinheritsched failed\n");
    //	}

    res = pthread_create(&m_thread_compiler, &attr,
                         ChannelControl::CompileThread, this);    //����G������������߳�
    if (res != 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]���������̴߳���ʧ��!", m_n_channel_index);
        m_error_code = ERR_SC_INIT;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER, 0, m_n_channel_index);
        goto END;
    }

    //����״̬ˢ���߳�
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//
    param.__sched_priority = 36; //96;
    pthread_attr_setschedparam(&attr, &param);
    res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
    if (res) {
        printf("pthread setinheritsched failed\n");
    }

    res = pthread_create(&m_thread_refresh_status, &attr,
                         ChannelControl::RefreshStatusThread, this);    //����ͨ��״̬ˢ���߳�
    if (res != 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "ͨ��[%d]״̬ˢ���߳�����ʧ�ܣ�", m_n_channel_index);
        res = ERR_SC_INIT;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER, 0, m_n_channel_index);
        goto END;
    }

    /*printf("------------- %d\n", strlen(m_channel_status.cur_nc_file_name));

    if(strlen(m_channel_status.cur_nc_file_name) > kMaxFileNameLen-2){
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�ļ�������: [%s] ��ʧ��!",
                m_n_channel_index, m_channel_status.cur_nc_file_name);
        CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                m_n_channel_index);
        goto END;
    }*/


    //Ĭ�ϴ��ļ�
    if(strlen(m_channel_status.cur_nc_file_name) > 0 and
            strlen(m_channel_status.cur_nc_file_name) < kMaxFileNameLen - 2){  // @modify zk ���ع����ļ������±���
        strcpy(path, PATH_NC_FILE);
        strcat(path, m_channel_status.cur_nc_file_name);   //ƴ���ļ�����·��
        //��֤�ļ��Ƿ����
        if(access(path, F_OK) == -1){//�ļ������ڣ����ļ�����λ
            g_ptr_trace->PrintLog(LOG_ALARM, "ͨ��[%d]Ĭ�ϴ�NC�ļ�[%s]�����ڣ�",
                                  m_n_channel_index, m_channel_status.cur_nc_file_name);
            memset(m_channel_status.cur_nc_file_name, 0x00, kMaxFileNameLen);
        }else{ //�ļ�����
            this->m_p_compiler->OpenFile(path);
            printf("set nc file cmd : %s\n", path);
        }
    }


END:
    pthread_attr_destroy(&attr);


    //	this->m_p_general_config->hw_rev_trace = 1;   //�����ã�ǿ�ƴ����ַ�����ٹ���  FOR TEST  ��������ɾ��

    printf("Exit ChannelControl::Initialize\n");
    return true;
}

/**
 * @brief ��ʼ��ͨ��״̬
 */
void ChannelControl::InitialChannelStatus(){
    memset(&m_channel_status, 0x00, sizeof(m_channel_status));

    InitGCodeMode();  //��ʼ��Gģ̬

    m_channel_status.rated_manual_speed = 100;	//6000mm/min
    //	m_channel_status.chn_work_mode = INVALID_MODE;
#if defined(USES_WUXI_BLOOD_CHECK) || defined(USES_GENERAL_AUTOMATIC)
    m_channel_status.chn_work_mode = AUTO_MODE;	//Ĭ���Զ�ģʽ
    this->m_p_f_reg->OP = 0;
    this->m_p_f_reg->MMEM = 1;
    this->SendWorkModeToMc(MC_MODE_AUTO);
    //	this->SetWorkMode(AUTO_MODE);
#else
    m_channel_status.chn_work_mode = MANUAL_MODE;	//Ĭ���ֶ�����ģʽ
    this->m_p_f_reg->OP = 0;
    this->m_p_f_reg->MJ = 1;
    //	this->SetWorkMode(MANUAL_MODE);
#endif
    m_channel_status.machining_state = MS_READY;
    m_channel_status.manual_step = 1;	//Ĭ��Ϊ1um����
    m_channel_status.spindle_mode = 0;  	//Ĭ���ٶȿ���
    m_channel_status.cur_axis = 0;    //Ĭ�����Ϊ0
    m_channel_status.func_state_flags.Init();	//Ĭ�Ͼ�Ϊ0
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

    if(this->m_n_spindle_count > kMaxSpindleChn){  //�����������
        CreateError(ERR_CHN_SPINDLE_OVERRUN, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        this->m_error_code = ERR_CHN_SPINDLE_OVERRUN;
    }

    //	strcpy(m_channel_status.cur_nc_file_name, "20.nc");	//for test
    g_ptr_parm_manager->GetCurNcFile(this->m_n_channel_index, m_channel_status.cur_nc_file_name);
}


/**
 * @brief ͨ����λ����
 */
void ChannelControl::Reset(){

    this->m_error_code = ERR_NONE;

    if(this->m_thread_breakcontinue > 0){//���ڶϵ�����߳�ִ�й����У����˳��ϵ�����߳�
        this->CancelBreakContinueThread();
    }

    if(this->m_channel_status.machining_state == MS_RUNNING){   //�����н��и�λ����
        this->StopRunGCode(false);
        this->m_b_delay_to_reset = true;
    }else if(this->m_n_run_thread_state != IDLE){  //�����߳�����״̬����ֹͣ
        this->StopCompilerRun();
    }

    if(this->m_b_manual_tool_measure){  //��ֹ�ֶ��Ե�
        m_b_cancel_manual_tool_measure = true;
        this->StopRunGCode(false);
        this->m_b_delay_to_reset = true;
    }

    if(this->m_b_manual_call_macro){   //��ֹ�ֶ����ú����
        m_b_cancel_manual_call_macro = true;
        this->m_b_delay_to_reset = true;
        this->StopRunGCode(false);
    }

    if(m_b_delay_to_reset){
        //	printf("ChannelControl::Reset(), delay to return\n");
        return;   //�ӳٴ���λ
    }

    //��λ�ӹ���λ״̬
    this->m_n_restart_mode = NOT_RESTART;
    this->m_n_restart_line = 0;
    this->m_n_restart_step = 0;
    this->m_p_compiler->SetRestartPara(0, NOT_RESTART);

    this->ClearHmiInfoMsg();

    //��λ����ָ�����
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
    this->ResetAllAxisOutZero();  // ��������л��˿���ģʽ�������ڴ˴���λ�ٶ�ֵ �� ����ֵ Ϊ0
#endif

    // ���Ḵλ
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

//    //ͬ��PMC���Ŀ��λ�ú͵�ǰλ��
//    for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
//        if(this->m_mask_pmc_axis & (0x01<<i)){
//            m_channel_rt_status.tar_pos_work.m_df_point[i] = m_channel_rt_status.cur_pos_work.m_df_point[i];
//        }
//    }

    m_n_subprog_count = 0;
    m_n_macroprog_count = 0;
    m_b_ret_from_macroprog = false;
    m_b_init_compiler_pos = false;  //��������ʼλ����Ҫ���³�ʼ��
    m_b_need_change_to_pause = false;

    this->m_p_compiler->Reset();
    if(strlen(m_channel_status.cur_nc_file_name) > 0){
        char file_name[128];
        memset(file_name, 0x0, 128);
        strcpy(file_name, PATH_NC_FILE);
        strcat(file_name, m_channel_status.cur_nc_file_name);   //ƴ���ļ�����·��
        this->SendOpenFileCmdToHmi(m_channel_status.cur_nc_file_name);
    }

    this->m_macro_variable.Reset();   //�������λ
    this->m_scene_auto.need_reload_flag = false;   //ȡ���ϵ������־

    this->SendMcResetCmd();   //��λMC
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
        m_n_need_reset_axis_ctrlmode = 20;  //  ģʽ��λ�л�û��ȫ���ɹ�����idle ��������ȥ���Ը�λ�����20��
    }else{
        printf("---the first reset axis_ctrlmode succeed! \n");
        m_n_need_reset_axis_ctrlmode = 0;  //   ����ģʽ��λȫ���ɹ�
    }
#endif

#ifdef USES_ADDITIONAL_PROGRAM
    this->m_n_add_prog_type = NONE_ADD;
    this->m_n_sub_count_bak = 0;
#endif

    //�������Ч����ϵ����
    if(g_ptr_parm_manager->ActiveCoordParam(m_n_channel_index))
        this->SetMcCoord(true);

    //�������Ч�ĵ���ƫ������
    if(g_ptr_parm_manager->ActiveToolOffsetParam(m_n_channel_index)){
        if(this->m_channel_status.cur_h_code > 0)
            this->SetMcToolOffset(true);
    }

    m_simulate_mode = SIM_NONE;    //Ĭ���ڷǷ���ģʽ
    this->SetMiSimMode(false);  //��λMI����״̬

    //��λͨ��״̬ΪREADY
    uint8_t state = MS_READY;   //�澯״̬

    this->SetMachineState(state);  //����ͨ��״̬

    this->ResetMode();   //��λͨ��ģ̬����

    this->m_p_f_reg->SPL = 0;
    this->m_p_f_reg->STL = 0;
    this->m_p_f_reg->AL = 0;
    this->m_p_f_reg->OP = 0;

#ifdef USES_GRIND_MACHINE
    this->SetMcSpecialIntpMode(0);  //ȡ��ĥ��״̬

    if(!this->m_b_ret_safe_state){ //ִ�и���ذ�ȫλ�ö���
        this->m_b_ret_safe_state = true;
        this->m_n_ret_safe_step = 0;

        gettimeofday(&m_time_ret_safe_delay, nullptr);  //��ʼ�������ذ�ȫλ��ʱ��

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
 * @brief ��ȡ��ӦDֵ�õ��߰뾶����ֵ   �뾶+ĥ��
 * @param d_index : Dֵ, ��1��ʼ
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
 * @brief PMC�ᶨλ�ƶ�
 * @param chn_axis : ͨ����ţ���0��ʼ
 * @param tar_pos �� Ŀ��λ��
 * @param inc �� ����ģʽ��־
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelControl::PmcRapidMove(uint8_t chn_axis, double tar_pos, bool inc){
    uint8_t phy_axis = this->GetPhyAxis(chn_axis);
    if(phy_axis == 0xFF)
        return false;


    PmcCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(PmcCmdFrame));

    //�����  ���ֽ�:0--����ָ��   1--����G01ָ��  ���ֽڣ�0--��������   1--��������
    cmd.data.cmd = inc?0x100:0;

    //���    ���ֽڣ����[1-64]    ���ֽڣ�����ͨ����[1-4]
    cmd.data.axis_index = ((phy_axis+1)|((m_n_channel_index+1)<<8));

    cmd.data.data[0] = 0;   //��λ�ƶ�����0


    //�����ٶ�
    uint32_t feed = this->m_p_axis_config[phy_axis].rapid_speed * 1000 / 60;   //ת����λΪum/s

    //����Ŀ��λ��
    int64_t target = tar_pos*1e7;    //ת����λΪ0.1nm


    memcpy(&cmd.data.data[1], &target, sizeof(target));  //����Ŀ��λ��

    memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //�����ٶ�

    this->m_mask_run_pmc |= 0x01L<<phy_axis;  //���õ�ǰ������

    this->m_p_mi_comm->SendPmcCmd(cmd);

    printf("PmcRapidMove: axis = %d, tar_pos = %lf\n", phy_axis, tar_pos);

    return true;
}

/**
 * @brief PMC�������ƶ�
 * @param total �� G01��֡��
 * @param idx ����ǰ֡��ţ���1��ʼ
 * @param chn_axis : ͨ����ţ���0��ʼ
 * @param tar_pos �� Ŀ��λ��
 * @param vel �� Ŀ���ٶ�
 * @param inc �� ����ģʽ��־
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelControl::PmcLineMove(uint8_t total, uint8_t idx, uint8_t chn_axis, double tar_pos, double vel, bool inc){
    //	printf("ChannelControl::PmcLineMove, total=%hhu, idx=%hhu, chn_axis=%hhu, pos=%lf, vel=%lf, inc=%hhu\n", total, idx, chn_axis, tar_pos, vel, inc);
    uint8_t phy_axis = this->GetPhyAxis(chn_axis);
    if(phy_axis == 0xFF)
        return false;


    PmcCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(PmcCmdFrame));

    //�����  ���ֽ�:0--����ָ��   1--����G01ָ��  ���ֽڣ�0--��������   1--��������
    cmd.data.cmd = inc?0x101:1;

    //���    ���ֽڣ����[1-64]    ���ֽڣ�����ͨ����[1-4]
    cmd.data.axis_index = ((phy_axis+1)|((m_n_channel_index+1)<<8));

    uint16_t frame = total;
    cmd.data.data[0] = frame<<8|idx;   //��֡���Լ���ǰ֡���

    //�����ٶ�
    uint32_t feed = vel * 1000 / 60;   //ת����λΪum/s

    //����Ŀ��λ��
    int64_t target = tar_pos*1e7;    //ת����λΪ0.1nm


    memcpy(&cmd.data.data[1], &target, sizeof(target));  //����Ŀ��λ��

    memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //�����ٶ�

    this->m_mask_run_pmc |= 0x01L<<phy_axis;  //���õ�ǰ������

    this->m_p_mi_comm->SendPmcCmd(cmd);

    printf("PmcLineMove: axis = %d, tar_pos = %lf, vel = %lf\n", phy_axis, tar_pos, vel);

    return true;
}

/**
 * @brief PMC�����е�λ
 * @param cmd : MI���͵�����֡
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelControl::PmcAxisRunOver(MiCmdFrame cmd){
    printf("ChannelControl::PmcAxisRunOver: phyaxis = %hhu\n", cmd.data.axis_index);

    if(this->m_mask_run_pmc == 0)
        return false;

    uint64_t mask = 0;

    if((cmd.data.data[0]&0xFF) == 0x01){//G01ָ��
        memcpy(&mask, &cmd.data.data[1], 4);
    }else{
        mask = 0x01L<<(cmd.data.axis_index-1);
    }

    this->m_mask_runover_pmc |= mask;

    if(this->m_mask_run_pmc == this->m_mask_runover_pmc){  //���н���
        this->m_mask_run_pmc = 0;
        this->m_mask_runover_pmc = 0;

        printf("ChannelControl pmc axis run over\n");
    }

    return true;
}

/**
 * @brief �򿪴������ļ�
 * @param file_name : �������ļ��ľ���·��
 * @return true--�ɹ�  false--ʧ��
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
 * @brief ���÷���ģʽ
 * @param mode : ����ģʽ
 * @return true--���óɹ�   false--����ʧ��
 */
bool ChannelControl::SetSimulateMode(const SimulateMode mode){
    if(mode == m_simulate_mode)
        return true;

    if(this->m_n_run_thread_state != IDLE && mode != SIM_NONE)
        return false;   //�ǿ���״̬�����л�������ģʽ

    m_simulate_mode = mode;

    if(mode == SIMULATE_MACHINING){//�ӹ����棬��Ҫ֪ͨMI�л�������״̬
        this->SetMiSimMode(true);
    }else{
        this->SetMiSimMode(false);
    }

    return true;
}

/**
 * @brief ����MI����ģʽ
 * @param flag : true--�������   false--�˳�����
 */
void ChannelControl::SetMiSimMode(bool flag){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_WORK_MODE;

    cmd.data.data[0] = this->m_channel_status.chn_work_mode;
    cmd.data.data[1] = flag?0x10:0x00;


    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = m_n_channel_index;  //ͨ���ţ���0��ʼ

    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief ��ȡG��ַ�Ĵ���״̬
 * @param sec �� ��ַ��
 * @param bit : bitλ��ţ�0-7
 * @return true--���ź�  false--���ź�
 */
bool ChannelControl::CheckGRegState(int sec, int bit){
    if(sec > 255 || bit > 7)
        return false;

    uint8_t value = this->m_p_pmc_reg->GReg().all[256*this->m_n_channel_index+sec];
    return (value&(0x01<<bit))?true:false;
}

/**
 * @brief ��ȡF��ַ�Ĵ���״̬
 * @param sec �� ��ַ��
 * @param bit : bitλ��ţ�0-7
 * @return true--���ź�  false--���ź�
 */
bool ChannelControl::CheckFRegState(int sec, int bit){
    if(sec > 255 || bit > 7)
        return false;

    uint8_t value = this->m_p_pmc_reg->FReg().all[256*this->m_n_channel_index+sec];
    return (value&(0x01<<bit))?true:false;
}

/**
 * @brief ����F��ַ�Ĵ�����ֵ
 * @param sec �� ��ַ��
 * @param bit : bitλ��ţ�0-7
 * @param value : true--���ź�  false--���ź�
 * @return  true--�ɹ�   false--ʧ��
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
 * @brief ��ȡ#1032��ֵ
 * @return G54~G57��ֵ
 */
uint32_t ChannelControl::GetMacroVar1032(){
    uint32_t value = 0;

    memcpy(&value, &this->m_p_pmc_reg->GReg().all[256*this->m_n_channel_index+54], 4);

    return value;
}

/**
 * @brief ��ȡ#1132��ֵ
 * @return F54~F55 16bits��ֵ
 */
uint16_t ChannelControl::GetMacroVar1132(){
    uint16_t value = 0;

    memcpy(&value, &this->m_p_pmc_reg->FReg().all[256*this->m_n_channel_index+54], 2);

    return value;
}

/**
 * @brief ��ȡ#1133��ֵ
 * @return F56~F59 32bits��ֵ
 */
uint32_t ChannelControl::GetMacroVar1133(){
    uint32_t value = 0;

    memcpy(&value, &this->m_p_pmc_reg->FReg().all[256*this->m_n_channel_index+56], 4);

    return value;
}

/**
 * @brief ����#1132��ֵ
 * @param value
 */
void ChannelControl::SetMacroVar1132(uint16_t value){
    memcpy(&this->m_p_pmc_reg->FReg().all[256*this->m_n_channel_index+54], &value, 2);
}

/**
 * @brief ����#1133��ֵ
 * @param value
 */
void ChannelControl::SetMacroVar1133(uint32_t value){
    memcpy(&this->m_p_pmc_reg->FReg().all[256*this->m_n_channel_index+56], &value, 4);
}

/**
 * @brief ����ָ���������ֵ
 * @param index[in] : ϵͳ��������
 * @param value[out] : ����ϵͳ����ֵ
 * @return 0--�ɹ����ѳ�ʼ��   1--�Ƿ�����    2--δ��ʼ��
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
 * @brief ��ȡϵͳ����ֵ
 * @param index[in] : ϵͳ��������
 * @param value[out] : ����ϵͳ����ֵ
 * @return
 */
bool ChannelControl::GetSysVarValue(const int index, double&value){

    if(index == 1220){    //����״̬
        Spindle::Polar polar = m_p_spindle->CalPolar();
        if(polar == Polar::Stop)
            value = 5;
        else if(polar == Polar::Positive)
            value = 3;
        else if(polar == Polar::Negative)
            value = 4;
    }
#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
    else if(index == 2000){  //��׼���ļ��ε�ƫ
        value = this->m_p_chn_tool_config->geometry_comp_basic[2];
    }
#endif
    else if(index >= 2001 && index <= 2200){   //����ĥ�𲹳�
        int id = index - 2001;
        if(id < kMaxToolCount)
            value = this->m_p_chn_tool_config->geometry_wear[id];
        else
            value = 0.0;

    }else if(index >= 2201 && index <= 2400){  //�������β���
        int id = index - 2201;
        if(id < kMaxToolCount)
            value = this->m_p_chn_tool_config->geometry_compensation[id][2];
        else
            value = 0.0;
    }else if(index == 3019){  //����ִ�к��·����
        value = this->m_n_channel_index;

    }else if(index == 3901){  //�Ѽӹ�����
        //value = this->m_channel_status.workpiece_count;
        value = this->m_channel_status.workpiece_count_total;
    }else if(index >= 4001 && index <= 4030){   //��

    }else if(index >= 4201 && index <= 4230){    //��ǰGָ��ģ̬
    	value = this->m_channel_status.gmode[index-4201];  //ģ̬ΪGָ�����10��G43.4����Ϊ434
        value /= 10;
    }else if(index == 4307){   //��ǰDֵ
        value = this->m_channel_status.cur_d_code;
    }else if(index == 4309){   //��ǰFֵ
        value =  this->m_channel_status.rated_feed;
    }else if(index == 4311){   //��ǰHֵ
        value = this->m_channel_status.cur_h_code;
    }else if(index == 4313){   //TODO ��ǰMֵ
        uint32_t mcode = this->m_p_f_reg->mcode_3;
        mcode = mcode<<8;
        mcode |= this->m_p_f_reg->mcode_2;
        mcode = mcode<<8;
        mcode |= this->m_p_f_reg->mcode_1;
        mcode = mcode<<8;
        mcode |= this->m_p_f_reg->mcode_0;
        value = mcode;
    }else if(index == 4319){   //��ǰSֵ
        value = this->m_channel_status.rated_spindle_speed;
    }else if(index == 4320){   //��ǰTֵ
        value = this->m_channel_status.cur_tool;
    }else if(index == 4321){   //��ǰԤѡ����
        value = this->m_channel_status.preselect_tool_no;
    }else if(index >= 5001 && index < 5001+kMaxAxisChn){   //��ǰĿ��λ�ã� ��������ϵ
        value = this->m_channel_rt_status.tar_pos_work.m_df_point[index-5001];
    }else if(index >= 5021 && index < 5021+kMaxAxisChn){  //��ǰ��е����
        value = this->m_channel_rt_status.cur_pos_machine.m_df_point[index-5021];
    }else if(index >= 5041 && index < 5041+kMaxAxisChn){   //��ǰ��������
        value = this->m_channel_rt_status.cur_pos_work.m_df_point[index-5041];
    }else if(index >= 5061 && index < 5061+kMaxAxisChn){  //G31�������λ�ã���������ϵ
        value = m_point_capture.m_df_point[index - 5061];
        this->TransMachCoordToWorkCoord(value, this->m_channel_status.gmode[14], index-5061);   //��е����ϵת��Ϊ��������ϵ
    }else if(index >= 5081 && index < 5084){  //����XYZ���򳤶Ȳ���
        int id = index - 5081;
        if(this->m_channel_status.cur_h_code > 0)
            value = this->m_p_chn_tool_config->geometry_compensation[this->m_channel_status.cur_h_code-1][id];
        else
            value = 0;

    }else if(index >= 5201 && index <= 5220){   //��������ϵƫ��
        int id = index-5201;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[0].offset[id];
        else
            value = 0.0;
    }else if(index >= 5221 && index <= 5240){   //G54����ϵƫ��
        int id = index-5221;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[1].offset[id];
        else
            value = 0.0;
    }else if(index >= 5241 && index <= 5260){   //G55����ϵƫ��
        int id = index-5241;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[2].offset[id];
        else
            value = 0.0;
    }else if(index >= 5261 && index <= 5280){   //G56����ϵƫ��
        int id = index-5261;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[3].offset[id];
        else
            value = 0.0;
    }else if(index >= 5281 && index <= 5300){   //G57����ϵƫ��
        int id = index-5281;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[4].offset[id];
        else
            value = 0.0;
    }else if(index >= 5301 && index <= 5320){   //G58����ϵƫ��
        int id = index-5301;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[5].offset[id];
        else
            value = 0.0;
    }else if(index >= 5321 && index <= 5340){   //G59����ϵƫ��
        int id = index-5321;
        if(id < this->m_p_channel_config->chn_axis_count)
            value = this->m_p_chn_coord_config[6].offset[id];
        else
            value = 0.0;
    }else if(index >= 5421 && index <= 5440){//G31�������λ�ã���е����ϵ
        if(index - 5421 < kMaxAxisChn)
            value = m_point_capture.m_df_point[index-5421];
    }else if(index >= 7001 && index <= 8980){
        printf("alarm index: %d\n", index);
        int coord = (index - 7001)/20;  //  G540X  ��Xֵ

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
    else if(index >= 12001 && index <= 12999){   //���߰뾶ĥ�𲹳�
        int id = index - 12001;
        if(id < kMaxToolCount)
            value = this->m_p_chn_tool_config->radius_wear[id];
        else
            value = 0.0;
    }else if(index >= 13001 && index <= 13999){   //���߰뾶���β���
        int id = index - 13001;
        if(id < kMaxToolCount)
            value = this->m_p_chn_tool_config->radius_compensation[id];
        else
            value = 0.0;
    }else if(index >= 30001 && index <= 30060) {    //������������ʽ
        int id = index - 30001;
        value = this->m_p_chn_tool_info->tool_life_type[id];
    }else if(index >= 30101 && index <= 30160) {    //�����������
        int id = index - 30101;
        value = this->m_p_chn_tool_info->tool_life_max[id];
    }else if(index >= 30201 && index <= 30260) {    //������������
        int id = index - 30201;
        value = this->m_p_chn_tool_info->tool_life_cur[id];
    }else if(index >= 30301 && index <= 30360) {    //����Ԥ������
        int id = index - 30301;
        value = this->m_p_chn_tool_info->tool_threshold[id];
    }else{
        return false;
    }

    return true;
}

/**
 * @brief ����ϵͳ����
 * @param index[in] : ϵͳ��������
 * @param value[in] : ϵͳ����ֵ
 * @return
 */
bool ChannelControl::SetSysVarValue(const int index, const double &value){
#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
    if(index == 2000){  //��׼���ļ��ε�ƫ
        this->m_p_chn_tool_config->geometry_comp_basic[2] = value;
        g_ptr_parm_manager->UpdateToolMeasure(this->m_n_channel_index, 0, value);

    }else if(index >= 2001 && index <= 2200){   //����ĥ�𲹳�
#else
    if(index >= 2001 && index <= 2200){   //����ĥ�𲹳�
#endif
        int id = index - 2001;
        if(id < kMaxToolCount){
            this->m_p_chn_tool_config->geometry_wear[id] = value;
            g_ptr_parm_manager->UpdateToolWear(m_n_channel_index, id, value);
            this->NotifyHmiToolOffsetChanged(id+1);   //֪ͨHMI��ƫֵ����
        }else
            return false;

    }else if(index >= 2201 && index <= 2400){  //�������β���
        int id = index - 2201;
        if(id < kMaxToolCount){
            this->m_p_chn_tool_config->geometry_compensation[id][2] = value;
            // @modify Ϊʲô����� id + 1  ������Update û�� id + 1
            g_ptr_parm_manager->UpdateToolMeasure(this->m_n_channel_index, id, value);
            //g_ptr_parm_manager->UpdateToolMeasure(this->m_n_channel_index, id+1, value);
            this->NotifyHmiToolOffsetChanged(id+1);   //֪ͨHMI��ƫֵ����
        }else
            return false;
    }else if(index == 3000){  //ֻд��������ʾ��ʾ��Ϣ
        pthread_mutex_unlock(&m_mutex_change_state);
        pthread_mutex_unlock(&m_mutex_change_state);
    	uint16_t msg_id = value;
        printf("set 3000 value = %u\n", msg_id);
        if(msg_id >= 0){
			CreateError(msg_id, INFO_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
			printf("3000 msg_id: %d\n", msg_id);
		}
    }else if(index == 3006){  //ֻд��������ʾ�澯��Ϣ
    	pthread_mutex_unlock(&m_mutex_change_state);
    	uint16_t err_id = value;
        if(err_id > 0){
            CreateError(err_id, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            printf("3006 err_id: %d\n", err_id);
        }
    }else if(index == 3901){  //�Ѽӹ�����
        SetChnCurWorkPiece(value);
    }else if(index == 4320){  //���õ�ǰ����
        this->m_channel_status.cur_tool = value;
        g_ptr_parm_manager->SetCurTool(m_n_channel_index, m_channel_status.cur_tool);
        this->SendModeChangToHmi(T_MODE);

#ifdef USES_WOOD_MACHINE
        this->SetMcToolLife();
#endif
    }
    else if(index >= 5201 && index <= 5220){   //��������ϵƫ��
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
    }else if(index >= 5221 && index <= 5240){   //G54����ϵƫ��
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
    }else if(index >= 5241 && index <= 5260){   //G55����ϵƫ��
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
    }else if(index >= 5261 && index <= 5280){   //G56����ϵƫ��
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
    }else if(index >= 5281 && index <= 5300){   //G57����ϵƫ��
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
    }else if(index >= 5301 && index <= 5320){   //G58����ϵƫ��
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
    }else if(index >= 5321 && index <= 5340){   //G59����ϵƫ��
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
        int coord = (index - 7001)/20;  //  G540X  ��Xֵ
        // �����������չ��������
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
    else if(index >= 12001 && index <= 12999){   //���߰뾶ĥ�𲹳�
        int id = index - 12001;
        if(id < kMaxToolCount){
            this->m_p_chn_tool_config->radius_wear[id] = value;
            g_ptr_parm_manager->UpdateToolRadiusWear(m_n_channel_index, id, value);
            this->NotifyHmiToolOffsetChanged(id+1);   //֪ͨHMI��ƫֵ����
        }
        else
            return false;;
    }
    else if(index >= 13001 && index <= 13999){   //���߰뾶���β���
        int id = index - 13001;
        if(id < kMaxToolCount){
            this->m_p_chn_tool_config->radius_compensation[id] = value;

            g_ptr_parm_manager->UpdateToolRadiusGeo(m_n_channel_index, id, value);
            this->NotifyHmiToolOffsetChanged(id+1);   //֪ͨHMI��ƫֵ����
        }
        else
            return false;
    }
    else if(index >= 30001 && index <= 30060) {    //������������ʽ
        int id = index - 30001;
        this->m_p_chn_tool_info->tool_life_type[id] = value;
        NotifyHmiToolPotChanged();
    }else if(index >= 30101 && index <= 30160) {    //�����������
        int id = index - 30101;
        this->m_p_chn_tool_info->tool_life_max[id] = value;
        NotifyHmiToolPotChanged();
    }else if(index >= 30201 && index <= 30260) {    //������������
        int id = index - 30201;
        this->m_p_chn_tool_info->tool_life_cur[id] = value;
        NotifyHmiToolPotChanged();
    }else if(index >= 30301 && index <= 30360) {    //����Ԥ������
        int id = index - 30301;
        this->m_p_chn_tool_info->tool_threshold[id] = value;
        NotifyHmiToolPotChanged();
    }
    else
        return false;

    return true;
}

/**
 * @brief ��ʼ�����ݸ�MC��ģ̬��Ϣ
 */
void ChannelControl::InitMcModeStatus(){
    this->m_mc_mode_exec.all = 0;
    this->m_mc_mode_cur.all = 0;

    //02��
    m_mc_mode_exec.bits.mode_g17 = (m_channel_status.gmode[2]-G17_CMD)/10;

    //03��
    m_mc_mode_exec.bits.mode_g90 = (m_channel_status.gmode[3]-G90_CMD)/10;

    //05��
    m_mc_mode_exec.bits.mode_g94 = (m_channel_status.gmode[5]-G94_CMD)/10;

    //06��
    m_mc_mode_exec.bits.mode_g20 = (m_channel_status.gmode[6]-G20_CMD)/10;

    //07��
    m_mc_mode_exec.bits.mode_g40 = (m_channel_status.gmode[7]-G40_CMD)/10;

    //09��
    if(m_channel_status.gmode[9] == G89_CMD)
        m_mc_mode_exec.bits.mode_g73 = 15;
    else
        m_mc_mode_exec.bits.mode_g73 = (m_channel_status.gmode[9]-G73_CMD)/10;

    //10��
    m_mc_mode_exec.bits.mode_g98 = (m_channel_status.gmode[10]-G98_CMD)/10;

    //11��
    m_mc_mode_exec.bits.mode_g50 = (m_channel_status.gmode[11]-G50_CMD)/10;

    //13��
    m_mc_mode_exec.bits.mode_g60 = (m_channel_status.gmode[13]-G60_CMD)/10;

    //16��
    m_mc_mode_exec.bits.mode_g68 = (m_channel_status.gmode[16]-G68_CMD)/10;

    //Dֵ
    m_mc_mode_exec.bits.mode_d = m_channel_status.cur_d_code;
}

/**
 * @brief ��ʼ���ӹ���λ�м�ģ̬
 */
void ChannelControl::InitRestartMode(){
    m_mode_restart.gmode[1] = G00_CMD;  //01��Ĭ��G00
    m_mode_restart.gmode[2] = G17_CMD;  //02��Ĭ��G17
    m_mode_restart.gmode[3] = G90_CMD;  //03��Ĭ��G90
    //	m_mode_restart.gmode[4] = G23_CMD;
    m_mode_restart.gmode[5] = G94_CMD;  //05��Ĭ��G94  ÿ���ӽ���
    m_mode_restart.gmode[6] = G21_CMD;  //06��Ĭ��G21  ���Ƶ�λ

    m_mode_restart.gmode[7] = G40_CMD;  //07��Ĭ��G40
    m_mode_restart.gmode[8] = this->m_channel_status.gmode[8];  //08��Ĭ��G49
    m_mode_restart.gmode[9] = G80_CMD;  //09��Ĭ��G80
    m_mode_restart.gmode[10] = G98_CMD;  //10��Ĭ��G98  ���س�ʼƽ��
    m_mode_restart.gmode[11] = G50_CMD;  //11��Ĭ��G50  ��������ȡ��
    m_mode_restart.gmode[14] = this->m_channel_status.gmode[14];  //14��Ĭ��G54  G54��������ϵ
    m_mode_restart.gmode[15] = G64_CMD;  //15��Ĭ��G64  ������ʽ
    m_mode_restart.gmode[16] = G69_CMD;  //16��Ĭ��G69  ������ת������ά����任��ʽOFF

    m_mode_restart.gmode[17] = G15_CMD;  //17��Ĭ��15   ������ָ��ȡ��
    m_mode_restart.gmode[19] = G26_CMD;  //19��Ĭ��G26  �����ٶȱ䶯���ON

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
 * @brief ����ͨ��MDA�ļ�����
 * @param path[out] : ����ͨ��MDA�ļ��ľ���·��
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
 * @brief ����ӹ��ۼ�ʱ��
 */
void ChannelControl::ClearMachineTimeTotal()
{
    m_channel_rt_status.machining_time_total = 0;
}

/**
 * @brief ��λͨ��ģ̬ΪĬ��ֵ
 */
void ChannelControl::ResetMode(){
    printf("reset mode start!!!\n");

    //01�����ͨ��������Ĭ�Ͻ�����ʽ ����λ
    if(m_p_channel_config->default_feed_mode == 0){
        m_channel_status.gmode[1] = G00_CMD;
    }else if(m_p_channel_config->default_feed_mode == 1){
        m_channel_status.gmode[1] = G01_CMD;
    }else if(m_p_channel_config->default_feed_mode == 2){
        m_channel_status.gmode[1] = G02_CMD;
    }else{
        m_channel_status.gmode[1] = G03_CMD;
    }

    //02�����ͨ��������Ĭ�ϼӹ�ƽ�� ����λ
    if(m_p_channel_config->default_plane == 0){
        m_channel_status.gmode[2] = G17_CMD;
    }else if(m_p_channel_config->default_plane == 1){
        m_channel_status.gmode[2] = G18_CMD;
    }else{
        m_channel_status.gmode[2] = G19_CMD;
    }

    // 03�����ͨ��������Ĭ��ָ��ģʽ ����λ
    if(m_p_channel_config->default_cmd_mode == 0){
        m_channel_status.gmode[3] = G90_CMD;
    }else{
        m_channel_status.gmode[3] = G91_CMD;
    }

    m_channel_status.gmode[5] = G94_CMD;  //05��Ĭ��G94  ÿ���ӽ���
    m_channel_status.gmode[6] = G21_CMD;  //06��Ĭ��G21  ���Ƶ�λ

    m_channel_status.gmode[7] = G40_CMD;  //07��Ĭ��G40
    //	m_channel_status.gmode[8] = G49_CMD;  //08��Ĭ��G49
    m_channel_status.gmode[9] = G80_CMD;  //09��Ĭ��G80
    m_channel_status.gmode[10] = G98_CMD;  //10��Ĭ��G98  ���س�ʼƽ��
    m_channel_status.gmode[11] = G50_CMD;  //11��Ĭ��G50  ��������ȡ��
    //	m_channel_status.gmode[14] = G54_CMD;  //14��Ĭ��G54  G54��������ϵ
    m_channel_status.gmode[15] = G64_CMD;  //15��Ĭ��G64  ������ʽ
    m_channel_status.gmode[16] = G69_CMD;  //16��Ĭ��G69  ������ת������ά����任��ʽOFF

    m_channel_status.gmode[17] = G15_CMD;  //17��Ĭ��15   ������ָ��ȡ��
    m_channel_status.gmode[19] = G26_CMD;  //19��Ĭ��G26  �����ٶȱ䶯���ON

    m_channel_status.cur_d_code = 0;
    //	m_channel_status.cur_h_code = 0;

    //	this->SetMcToolOffset(false);   //ȡ����ƫ
    this->SendChnStatusChangeCmdToHmi(G_MODE);
    this->SendModeChangToHmi(H_MODE);
    this->SendModeChangToHmi(D_MODE);
    printf("reset mode finished!!!\n");
}

/**
 * @brief ��ʼG��������
 */
void ChannelControl::StartRunGCode(){
	printf("start run g code \n");

	g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC,"@#@#Enter ChannelControl::StartRunGCode(), m_n_run_thread_state = %d, chn_work_mode = %hhu, machine_mode = %hhu, mc_mode=%hu\n", m_n_run_thread_state,
			m_channel_status.chn_work_mode, m_channel_status.machining_state, m_channel_mc_status.cur_mode);


	if(m_channel_status.chn_work_mode != AUTO_MODE &&
			m_channel_status.chn_work_mode != MDA_MODE)
		return;


#ifdef USES_GRIND_MACHINE
    if(this->m_b_ret_safe_state){  //�ذ�ȫλ�ö���δ���
        printf("�ذ�ȫλ�ö���δ��ɣ���ִ�����ж���\n");
        return;
    }
#endif

    //������Ƿ�زο���
    uint32_t axis_mask = 0;
    for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if((m_channel_status.returned_to_ref_point & (0x01<<i)) == 0x00){
            //����δ�زο��㣬֪ͨHMI
            axis_mask |= (0x01<<i);
            g_ptr_trace->PrintLog(LOG_ALARM, "ͨ��[%hhu]��%hhuδ�زο��㣬��ֹ�Զ����У�\n", m_n_channel_index, this->m_p_channel_config->chn_axis_name[i]);
        }
    }
    if(axis_mask != 0){
        this->m_error_code = ERR_AXIS_REF_NONE;
        CreateError(m_error_code, WARNING_LEVEL, CLEAR_BY_MCP_RESET, axis_mask, m_n_channel_index);
        //	this->SendMessageToHmi(MSG_TIPS, MSG_ID_AXIS_REF);
        return;
    }

    //��鵱ǰ�Ƿ��ڴ���״̬
    if(g_ptr_alarm_processor->HasErrorInfo(m_n_channel_index)){
        ErrorInfo err_info;
        g_ptr_alarm_processor->GetLatestErrorInfo(&err_info);
        g_ptr_trace->PrintLog(LOG_ALARM, "ϵͳ���ڸ澯״̬����ֹ�Զ����У�last error = %d", err_info.error_code);
        g_ptr_alarm_processor->PrintError();
        return;
    }

    //��ǰ�ļӹ��ļ��Ƿ����
    if(!this->IsNcFileExist(this->m_channel_status.cur_nc_file_name)){
        this->m_error_code = ERR_NC_FILE_NOT_EXIST;
        CreateError(m_error_code, WARNING_LEVEL, CLEAR_BY_MCP_RESET, axis_mask, m_n_channel_index);
        return;
    }


    //��ǰ������ֹͣ����ͣ�����У��򷵻�
    if(m_channel_status.machining_state == MS_PAUSING || m_channel_status.machining_state == MS_STOPPING){
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "machine state is %hhu, return\n", m_channel_status.machining_state);
        return;
    }

#ifdef USES_ADDITIONAL_PROGRAM
    AddProgType add_type = NONE_ADD;   //���и��ӳ�������
#endif

    uint8_t state = MS_RUNNING;

    if(m_simulate_mode == SIM_OUTLINE)
        state = MS_OUTLINE_SIMULATING;
    else if(m_simulate_mode == SIM_TOOLPATH)
        state = MS_TOOL_PATH_SIMULATING;
    else if(m_simulate_mode == SIM_MACHINING)
        state = MS_MACHIN_SIMULATING;

    if(this->m_n_restart_mode != NOT_RESTART){  //�ӹ���λ
        this->m_p_compiler->SetRestartPara(m_n_restart_line, m_n_restart_mode);  //���ñ������ӹ���λ����
        this->InitRestartMode();
        this->SendMessageToHmi(MSG_TIPS, MSG_ID_RESTARTING);   //����HMI����ʾ��Ϣ
    }

    if(m_channel_status.machining_state == MS_READY &&
            (state == MS_RUNNING || state == MS_MACHIN_SIMULATING)){ //��ʼһ���µ����У���λMC
        this->ResetMcLineNo();

    }

    if(!this->m_b_init_compiler_pos){
        this->m_p_compiler->SetCurPos(this->m_channel_rt_status.cur_pos_work);
        this->m_p_compiler->SetCurGMode();   //��ʼ��������ģ̬
        this->m_b_init_compiler_pos = true;
    }

    if(this->m_channel_status.chn_work_mode == AUTO_MODE){

        string msg = "��ʼ�ӹ�����(" + string(this->m_channel_status.cur_nc_file_name) + ")";
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug, msg);

        if(this->m_channel_status.machining_state == MS_READY){  //����״̬��ֱ����������
            this->SendWorkModeToMc(MC_MODE_AUTO);

            //��ʼ�������ͼλ�����ݻ�����ر���
            m_n_graph_pos_count = 0;
            m_n_graph_pos_write_idx = 0;
            m_n_graph_pos_read_idx = 0;

            //�������
            g_ptr_parm_manager->ActiveNewStartParam();

            this->m_macro_variable.Reset();

            //��ʼ�����ݸ�MCģ���ģ̬����Ϣ
            this->InitMcModeStatus();

            this->m_p_compiler->SetCurPos(this->m_channel_rt_status.cur_pos_work);  //���ñ�������ʼλ��
            this->m_p_compiler->SetCurGMode();   //��ʼ��������ģ̬

            if(this->m_simulate_mode != SIM_NONE){
                this->m_pos_simulate_cur_work = this->m_channel_rt_status.cur_pos_work;   //��ʼ������ģʽ��ǰ��������
#ifdef USES_SIMULATION_TEST
                if(m_file_sim_data > 0){
                    close(m_file_sim_data);
                    m_file_sim_data = -1;  //�ȹر��ļ�
                }
                m_file_sim_data = open(PATH_SIMULATE_DATA, O_CREAT|O_TRUNC|O_WRONLY); //���ļ�
                if(m_file_sim_data < 0){
                    CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
                    this->m_error_code = ERR_OPEN_FILE;
                    g_ptr_trace->PrintLog(LOG_ALARM, "�򿪷���������ʱ�ļ�ʧ�ܣ�");
                    return;//�ļ���ʧ��
                }
#endif	
                SimulateData data;
                MonitorDataType type = MDT_CHN_SIM_GRAPH;  //��������
                if(this->m_simulate_mode == SIM_TOOLPATH)
                    type = MDT_CHN_SIM_TOOL_PATH;
                else if(this->m_simulate_mode == SIM_MACHINING)
                    type = MDT_CHN_SIM_MACH;
                //д����淶Χ
                data.type = ZONE_LIMIT_MIN;   //����д�븺������
                data.pos[0] = this->m_p_axis_config[GetPhyAxisFromName(AXIS_NAME_X)].soft_limit_min_1;
                data.pos[1] = this->m_p_axis_config[GetPhyAxisFromName(AXIS_NAME_Y)].soft_limit_min_1;
                data.pos[2] = this->m_p_axis_config[GetPhyAxisFromName(AXIS_NAME_Z)].soft_limit_min_1;
                this->SendSimulateDataToHmi(type, data);   //���͸�������

                data.type = ZONE_LIMIT_MAX;   //��д����������
                data.pos[0] = this->m_p_axis_config[GetPhyAxisFromName(AXIS_NAME_X)].soft_limit_max_1;
                data.pos[1] = this->m_p_axis_config[GetPhyAxisFromName(AXIS_NAME_Y)].soft_limit_max_1;
                data.pos[2] = this->m_p_axis_config[GetPhyAxisFromName(AXIS_NAME_Z)].soft_limit_max_1;
                this->SendSimulateDataToHmi(type, data);   //������������

                //д�뵱ǰλ��
                data.type = ORIGIN_POINT;
                data.pos[0] = this->GetAxisCurMachPos(this->GetChnAxisFromName(AXIS_NAME_X));
                data.pos[1] = this->GetAxisCurMachPos(this->GetChnAxisFromName(AXIS_NAME_Y));
                data.pos[2] = this->GetAxisCurMachPos(this->GetChnAxisFromName(AXIS_NAME_Z));
                this->SendSimulateDataToHmi(type, data);   //���ͳ�ʼλ��
            }
#ifdef USES_ADDITIONAL_PROGRAM
            else if(this->m_n_restart_mode != NOT_RESTART){  //�ӹ���λ
                add_type = RESET_START_ADD;     //
            }else{
                add_type = NORMAL_START_ADD;
            }
#endif

#ifdef USES_WOOD_MACHINE
            m_b_prestart_spd = false;
            this->m_n_spd_prestart_step = 0;
#endif

            m_channel_rt_status.machining_time = 0;    //���üӹ�ʱ��
        }
        else if(m_channel_status.machining_state == MS_PAUSED){
#ifndef USES_GRIND_MACHINE  //����������Ҫ�ϵ㷵�ع���
#ifndef USES_ADDITIONAL_PROGRAM
            if(this->m_scene_auto.need_reload_flag){
#endif
                if(StartBreakpointContinue()){
                    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "start break point continue thread\n");
                    gettimeofday(&m_time_start_maching, nullptr);  //��ʼ������ʱ��
                    goto END;
                }
#ifndef USES_ADDITIONAL_PROGRAM
            }
#endif
#else

#endif
            if(this->m_mask_run_pmc){
                this->PausePmcAxis(NO_AXIS, false);  //��������PMC��
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
                            polarmsg->SetExecStep(0);    //��������G12.3ָ��
                        }
                    }
                }
#endif
            }

        }else{
            printf("StartRunGCode WARNING:m_channel_status.machining_state: %hhu\n", m_channel_status.machining_state);
        }

        gettimeofday(&m_time_start_maching, nullptr);  //��ʼ������ʱ��

        //printf("StartRunGCode:m_n_run_thread_state, %d\n", m_n_run_thread_state);
        if(m_n_run_thread_state == IDLE || m_n_run_thread_state == PAUSE){
            m_n_run_thread_state = RUN;  //��Ϊ����״̬
        }

        this->m_p_f_reg->OP = 1;   //�Զ�����״̬

    }
    else if(m_channel_status.chn_work_mode == MDA_MODE){

        string msg = "��ʼִ��MDA����";
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug, msg);

        this->m_p_output_msg_list->Clear();	//��ջ�������
        this->InitMcIntpMdaBuf();  //���MC�е�MDA���ݻ���

        //��ʼ�����ݸ�MCģ���ģ̬����Ϣ
        this->InitMcModeStatus();

        this->m_p_compiler->SetCurPos(this->m_channel_rt_status.cur_pos_work);
        this->m_p_compiler->SetCurGMode();   //��ʼ��������ģ̬

        m_channel_rt_status.machining_time = 0;        //���üӹ�ʱ��
        gettimeofday(&m_time_start_maching, nullptr);  //��ʼ������ʱ��

        this->SendMdaDataReqToHmi();  //����MDA��������
        printf("send mda data req cmd to hmi, chn=%hhu\n", this->m_n_channel_index);
    }

END:
    m_n_step_change_mode_count = 0;

    this->SetMachineState(state);  //����ͨ��״̬

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
 * @brief ���õ�ǰ�кŴ�MCģ���ȡ
 */
void ChannelControl::SetCurLineNoFromMc(){

	if(this->m_n_macroprog_count == 0 || this->m_p_general_config->debug_mode > 0)  //����ģʽ�»���û�к�������
        m_b_lineno_from_mc = true;
}

/**
 * @brief ����MC��ʼ�岹
 */
void ChannelControl::StartMcIntepolate(){
    uint16_t mc_work_mode = MC_MODE_AUTO;    //Ĭ���Զ�ģʽ

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

    SetCurLineNoFromMc();	//��ǰ�кŴ�MC��ȡ

    m_b_mc_need_start = false; //��ֹ�ظ�����start

    gettimeofday(&m_time_start_mc, nullptr);  //����MC����ʱ��
    printf("@@@@@@@StartMcIntepolate\n");
}

/**
 * @brief ��ͣG��������
 */
void ChannelControl::PauseRunGCode(){
    uint8_t state = MS_PAUSING;

    if(this->m_channel_status.chn_work_mode == AUTO_MODE){
        string msg = "��ͣ�ӹ�����(" + string(this->m_channel_status.cur_nc_file_name) + ")";
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug, msg);
        if(this->m_simulate_mode != SIM_NONE &&
                (m_channel_status.machining_state == MS_OUTLINE_SIMULATING ||
                 m_channel_status.machining_state == MS_TOOL_PATH_SIMULATING ||
                 m_channel_status.machining_state == MS_MACHIN_SIMULATING)){  //����ֱ�ӽ���
            m_n_run_thread_state = STOP;
            if(m_channel_status.machining_state == MS_MACHIN_SIMULATING){  //�ӹ�����
                //��MCģ�鷢��ֹͣ����
            	this->PauseMc();
            }
            m_n_subprog_count = 0;
            m_n_macroprog_count = 0;
            m_b_ret_from_macroprog = false;
            m_b_init_compiler_pos = false;  //��������ʼλ����Ҫ���³�ʼ��

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

            this->SetMachineState(MS_READY);  //����ͨ��״̬

            return;
        }

        if(this->m_channel_status.machining_state != MS_RUNNING
        #ifdef USES_ADDITIONAL_PROGRAM
                || this->m_n_add_prog_type != NONE_ADD
        #endif
                ){ //������״̬,���߸��ӳ��������ڼ�
            return;
        }

        if(m_n_run_thread_state == RUN ){	//ֻ��RUN״̬�������̲߳���ҪPAUSE
            m_n_run_thread_state = PAUSE;
        }

        if(this->m_mask_run_pmc){   //��ͣPMC��
            this->PausePmcAxis(NO_AXIS, true);
            state = MS_PAUSED;
        }else{
            //��MCģ�鷢����ָͣ��
        	this->PauseMc();
        }

        this->m_p_f_reg->STL = 0;
        this->m_p_f_reg->SPL = 1;

    }
    else if(m_channel_status.chn_work_mode == MDA_MODE){

        m_n_run_thread_state = STOP;
        this->m_p_compiler->Reset();

        state = MS_STOPPING;	//ֹͣ�����У����ж�MCģ�����ͣ��������ΪMS_STOP״̬

        if(this->m_mask_run_pmc){   //ֹͣPMC��
            this->StopPmcAxis(NO_AXIS);
            state = MS_PAUSED;
        }else{
            //��MCģ�鷢��ֹͣ����
        	this->PauseMc();
        }

        this->m_p_output_msg_list->Clear();

        this->m_p_f_reg->STL = 0;
        this->m_p_f_reg->SPL = 0;

    }else
        return;

    if(m_simulate_mode != SIM_NONE)
        state = MS_READY;   //����ģʽֱ����ΪREADY

    this->SetMachineState(state);  //����ͨ��״̬
}

/**
 * @brief ֹͣ����,�������߳���ΪIDLE״̬
 */
void ChannelControl::StopCompilerRun(){
    printf("stop compiler run\n");

    // @test zk
    pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
    //printf("locked 1\n");
    m_n_run_thread_state = STOP;
    pthread_mutex_unlock(&m_mutex_change_state);
    //printf("unlocked 1\n");
}

/**
 * @brief ֹͣG��������
 * @param reset : �Ƿ�λ���ݺ��кţ� true--��λ   false--����λ
 */
void ChannelControl::StopRunGCode(bool reset){
    string msg = "ֹͣ�ӹ�����(" + string(this->m_channel_status.cur_nc_file_name) + ")";
    g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug, msg);
    printf("ChannelControl::StopRunGCode()\n");

    if(this->m_channel_status.machining_state == MS_READY && !m_b_cancel_manual_call_macro)  //����״ֱ̬�ӷ���
        return;

    uint8_t state = MS_STOPPING;

    if(this->m_thread_breakcontinue > 0){//���ڶϵ�����߳�ִ�й����У����˳��ϵ�����߳�
        this->CancelBreakContinueThread();
    }

    this->StopCompilerRun(); //ֹͣ����

    //TODO ��MCģ�鷢��ֹͣ����
    this->PauseMc();

    if(m_simulate_mode != SIM_NONE || this->m_channel_status.machining_state == MS_PAUSED || m_b_cancel_manual_call_macro)
        state = MS_READY;

    int count = 0;
    while(this->m_channel_status.machining_state != MS_READY &&
          this->m_channel_status.machining_state != MS_WARNING){//�ȴ�ֹͣ��λ
        if(++count > 200)
            break;
        usleep(100);
    }

    m_n_subprog_count = 0;
    m_n_macroprog_count = 0;
    m_b_ret_from_macroprog = false;
    m_b_init_compiler_pos = false;  //��������ʼλ����Ҫ���³�ʼ��
    this->m_p_compiler->Reset();

    if(this->m_channel_status.chn_work_mode == AUTO_MODE
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type != CONTINUE_START_ADD
        #endif
            )
        this->m_scene_auto.need_reload_flag = false;   //ȡ���ϵ������־

    if(reset){
        this->m_macro_variable.Reset();   //�������λ
        this->ResetMcLineNo();
        if(this->m_channel_status.chn_work_mode == AUTO_MODE
        #ifdef USES_ADDITIONAL_PROGRAM
                && this->m_n_add_prog_type != CONTINUE_START_ADD
        #endif
                ){
            this->InitMcIntpAutoBuf();
        }else if(m_channel_status.chn_work_mode >= MDA_MODE){   //�����ֶ�ģʽ
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

    this->SetMachineState(state);  //����ͨ��״̬

    //this->m_p_f_reg->SPL = 0;
    this->m_p_f_reg->STL = 0;
}


/**
 * @brief ��ͣMCģ���G����岹����������ģʽ�л����ֶ�
 */
void ChannelControl::PauseMc(){
    //���µ�ǰMC������ģʽ
    if(!this->m_b_mc_on_arm)
        this->m_p_mc_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);
    else
        this->m_p_mc_arm_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);

	// @test
	printf("pausemc m_channel_mc_status.cur_mode: %d\n", m_channel_mc_status.cur_mode);

//	printf("pausemc, mc_mode = %hu\n", m_channel_mc_status.cur_mode);
	if(m_channel_mc_status.cur_mode == MC_MODE_MANUAL ){//ֹͣ������
		uint16_t mask = 0;
		for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
			mask |= (0x01<<i);
		}
		this->ManualMoveStop(mask);
	}else
		// @bug  ����SC���ָ�����Ч ���� SC����  PMC���� ���޷�����
		this->SendIntpModeCmdToMc(MC_MODE_MANUAL);
}

/**
 * @brief ��MC����G31ֹͣ����,������ͨ��ֹͣ���в������е�λ
 */
void ChannelControl::SendMcG31Stop(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = this->m_n_channel_index;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_STOP_G31_CMD;
    if(this->m_channel_status.chn_work_mode == AUTO_MODE)
        cmd.data.data[0] = 1;   //ֹͣ
    else
        cmd.data.data[0] = 2;  //���Զ�ģʽ

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}


///**
// * @brief ��MC���͸��Թ�˿״̬���MC���л������Թ�˿���ٶȹ滮����
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
 * @brief ����MC��λָ��
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
 * @brief ֪ͨMC����ͨ��MDA�岹�����ʼ��
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
 * @brief ֪ͨMC����ͨ���Զ����ݻ����ʼ��
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
 * @brief ���д�����
 * @param count : �������ڼ���
 */
void ChannelControl::DoIdle(uint32_t count){

    if(count%150 == 0 && (!this->IsMachinRunning())
            && this->m_macro_variable.IsKeepChanged()){//ͬ��PMC�Ĵ���
        this->m_macro_variable.Sync();
    }

#ifdef USES_SPEED_TORQUE_CTRL
    //�����ģʽ�л�
    if(this->m_n_need_reset_axis_ctrlmode == 1){
        printf("---m_n_need_reset_axis_ctrlmode=%hhu \n",m_n_need_reset_axis_ctrlmode);
        this->m_n_need_reset_axis_ctrlmode = 0;
        if(this->ResetAllAxisCtrlMode(0)){  //�л�ʧ��
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
 * @brief ��MC�ĸ���岹λ��ˢ�µ�ͨ��ʵʱ״̬��
 */
void ChannelControl::RefreshAxisIntpPos(){
    int count = 0;
    for(int i = 0; i < m_p_channel_config->chn_axis_count && count < 8; i++){
        if(g_ptr_chn_engine->GetPmcActive(this->GetPhyAxis(i))) {
            m_channel_rt_status.cur_pos_work.m_df_point[i] = m_channel_rt_status.cur_pos_machine.m_df_point[i];   //pmc��Ĺ�������ͬ��е����
            m_channel_rt_status.tar_pos_work.m_df_point[i] = m_channel_rt_status.cur_pos_machine.m_df_point[i] + m_p_channel_engine->GetPmcAxisRemain(i);   //pmc�����ƶ�����mi��ȡ
            count++;
            continue;
        }
        m_channel_rt_status.cur_pos_work.m_df_point[i] = m_channel_mc_status.intp_pos.GetAxisValue(count);
        // m_channel_rt_status.tar_pos_work.m_df_point[i] = m_channel_mc_status.intp_tar_pos.GetAxisValue(count);

        // ��ͣ��׼��״̬����Ŀ��λ�õ�ֵ��Ϊ��ǰλ��
        // ��ע����ô����Ϊ�˽����λ/��ͣ�󻹱������ƶ���������
        // �������MC��������⣬�ٸĻ�ȥ
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
 * @brief ͬ����������
 */
void ChannelControl::SyncMacroVar(){
    if(this->m_macro_variable.IsKeepChanged()){//ͬ��PMC�Ĵ���
        this->m_macro_variable.Sync();
    }
}


/**
 * @brief ���ͼ������
 * @param bAxis : �Ƿ���������ʵʱλ��
 * @param btime : �Ƿ���¼ӹ�ʱ��
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

    //����ͨ����λ��
    double *pos = m_channel_rt_status.cur_pos_machine.m_df_point;
    double *speed = m_channel_rt_status.cur_feedbck_velocity.m_df_point;
    double *torque = m_channel_rt_status.cur_feedbck_torque.m_df_point;
    double *angle = m_channel_rt_status.spd_angle.m_df_point;
    uint8_t phy_axis = 0; //ͨ��������Ӧ��������
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
        if (this->m_channel_status.machining_state == MS_RUNNING){  //���¼ӹ���ʱ
            struct timeval cur_time;
            gettimeofday(&cur_time, nullptr);
            unsigned int time = cur_time.tv_sec-m_time_start_maching.tv_sec;
            m_channel_rt_status.machining_time += time;
            m_channel_rt_status.machining_time_total += time;

            m_time_remain -= (int32_t)time;    //ʣ��ӹ�ʱ��
            m_time_remain > 0 ? m_channel_rt_status.machining_time_remains = m_time_remain
                    : m_channel_rt_status.machining_time_remains = 0;

            m_time_start_maching = cur_time;
        }
        if (this->m_channel_status.machining_state == MS_READY) { //ֹͣ�ӹ�
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
    //	m_channel_rt_status.machining_time = (uint32_t)this->m_n_run_thread_state;  //for test  ,�߳�״̬
    //	m_channel_rt_status.machining_time_remains = m_channel_mc_status.buf_data_count;
    //	m_channel_rt_status.machining_time_remains = this->m_n_send_mc_data_err;    //for test ,���ݷ���ʧ�ܴ���

    //@test zk  MDA ģʽ����ʾ�к�
    if(m_channel_mc_status.cur_mode == MC_MODE_MDA &&
            m_channel_mc_status.cur_line_no > 0 &&
            !m_b_manual_call_macro/*llx add �������ò�����ʾ�к�*/){ //û�е��ú����
    	m_channel_rt_status.line_no = m_channel_mc_status.cur_line_no;
    }

    if(m_channel_mc_status.cur_mode == MC_MODE_AUTO &&
        #ifdef USES_ADDITIONAL_PROGRAM
            m_n_add_prog_type == NONE_ADD &&
        #endif
            m_b_lineno_from_mc && m_channel_mc_status.cur_line_no > 0){
        m_channel_rt_status.line_no = m_channel_mc_status.cur_line_no;
    }


    RefreshAxisIntpPos();   //ˢ�¹�������ϵ��ǰλ�ü�Ŀ��λ��

    //	// �����ݲ���  FOR TEST
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

    //���ڸչ�״̬��Ҫ��ȡ�չ����
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
    if(this->m_channel_status.chn_work_mode != AUTO_MODE && this->m_channel_status.chn_work_mode != MDA_MODE){//�ֶ�ģʽ�£����ƶ�������ʾ0
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
        this->SendHmiGraphPosData();  //���ͻ�ͼλ������
    }

    //	double inc_d = 0.001;
    //	m_channel_rt_status.cur_pos += DPoint(inc_d, inc_d, inc_d,inc_d,inc_d, inc_d,inc_d, inc_d);

}

/**
 * @brief ���ͷ������ݸ�HMI
 * @param type : �������ͣ��������桢��·����
 * @param data
 */
void ChannelControl::SendSimulateDataToHmi(MonitorDataType type, SimulateData &data){
    if(type != MDT_CHN_SIM_GRAPH && type != MDT_CHN_SIM_TOOL_PATH){
        g_ptr_trace->PrintLog(LOG_ALARM, "�����������ͷǷ�[%d]", type);
        return;
    }
#ifdef USES_SIMULATION_TEST
    if(this->m_file_sim_data > 0){
        ssize_t res = write(m_file_sim_data, &data.type, sizeof(int));
        if(res != 4){
            g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "д������������ʹ���res=%d", res);
        }
        res = write(m_file_sim_data, data.pos, sizeof(double)*3);
        if(res != 24){
            g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "д���������λ�ô���res=%d", res);
        }
    }else{
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "���������ļ����Ϊ�գ�");
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
 * @brief ����HMIģ�鷢�͵�����
 * @param cmd ���������HMI�����
 */
void ChannelControl::ProcessHmiCmd(HMICmdFrame &cmd){

    switch(cmd.cmd){
    case CMD_HMI_GET_CHN_STATE:   //HMI��ȡͨ����ǰ״̬
        ProcessHmiGetChnStateCmd(cmd);
        break;
    case CMD_HMI_RESTART:               //�ӹ���λ 11
        this->ProcessHmiRestartCmd(cmd);
        break;
    case CMD_HMI_SIMULATE:				//���� 12
        this->ProcessHmiSimulateCmd(cmd);
        break;
    case CMD_HMI_SET_NC_FILE:	    	//���õ�ǰ�ӹ��ļ� 13
        this->ProcessHmiSetNcFileCmd(cmd);
        break;
    case CMD_HMI_FIND_REF_POINT:		//ȷ���ο��� 14
        break;
    case CMD_HMI_SET_REF_POINT:			//���òο���
        this->ProcessHmiSetRefCmd(cmd);
        break;
    case CMD_SC_MDA_DATA_REQ:		//MDA�������� 115
        this->ProcessMdaData(cmd);
        break;
    case CMD_HMI_GET_MACRO_VAR:			//HMI��SC����������ֵ   30
        this->ProcessHmiGetMacroVarCmd(cmd);
        break;
    case CMD_HMI_SET_MACRO_VAR:        //HMI��SC���ú�����Ĵ�����ֵ   31
        this->ProcessHmiSetMacroVarCmd(cmd);
        break;

    case CMD_HMI_CLEAR_WORKPIECE:      //HMI����SC���ӹ���������,��ʱ����(���ְ�ҹ��)
        this->ProcessHmiClearWorkPieceCmd(cmd);
        this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);   //֪ͨHMI�ӹ��������
        break;
#ifdef USES_LASER_MACHINE
    case CMD_HMI_SET_CALIBRATION:     //HMI��SC�����������궨ָ�� 32
        cocopark
                this->ProcessHmiCalibCmd(cmd.data[0]);
        break;
#endif
    case CMD_HMI_CLEAR_TOTAL_PIECE:    //�ܹ���������
        this->ProcessHmiClearTotalPieceCmd(cmd);
        this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);   //֪ͨHMI�ӹ��������
        break;
    case CMD_HMI_SET_REQUIRE_PIECE:     //�����������
        this->ProcessHmiSetRequirePieceCmd(cmd);
        break;
    case CMD_HMI_AXIS_MANUAL_MOVE:     //HMIָ�����ƶ�
        break;
    case CMD_HMI_MANUAL_TOOL_MEASURE:     //HMI��SC�����ֶ��Ե�����  0x29
        this->ProcessHmiManualToolMeasureCmd(cmd);
        break;
    case CMD_HMI_SET_CUR_MACH_POS:        //HMI��SC����ָ����Ļ�е����  0x32
        this->ProcessHmiSetCurMachPosCmd(cmd);
        break;
    case CMD_HMI_CLEAR_MSG:               //HMI֪ͨSC�����Ϣ  0x33
        //		printf("channelcontrol process CMD_HMI_CLEAR_MSG, cmd_ex=%hhu\n", cmd.cmd_extension);
        this->ProcessHmiClearMsgCmd(cmd);
        break;
    default:

        break;

    }
}

/**
 * @brief ����زο�������
 * @param cmd : HMI�����
 */
void ChannelControl::ProcessHmiFindRefCmd(HMICmdFrame &cmd){


}

/**
 * @brief �����ȡ���������
 * @param cmd
 */
void ChannelControl::ProcessHmiGetMacroVarCmd(HMICmdFrame &cmd){
    //std::cout << "ChannelControl::ProcessHmiGetMacroVarCmd" << std::endl;
    cmd.frame_number |= 0x8000;   //���ûظ���־

    uint32_t start_index = 0;   //��ʼ���
    uint8_t count = 0;			//��������

    if(cmd.data_len != 5){	//���ݳ��Ȳ��Ϸ�
        cmd.cmd_extension = FAILED;
        this->m_p_hmi_comm->SendCmd(cmd);
        g_ptr_trace->PrintLog(LOG_ALARM, "ChannelControl::ProcessHmiSetMacroVarCmd()���ݳ��Ȳ��Ϸ���data_len = %hu��", cmd.data_len);
        return;
    }

    memcpy(&start_index, &cmd.data[0], 4);
    memcpy(&count, &cmd.data[4], 1);

    //��������
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
 * @brief �������ú��������
 * @param cmd : HMI�����
 */
void ChannelControl::ProcessHmiSetMacroVarCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;   //���ûظ���־

    uint32_t start_index = 0;   //��ʼ���
    uint8_t count = 0;			//��������

    if(this->m_p_channel_engine->IsParamImported(CONFIG_MACRO_VAR)){
        cmd.cmd_extension = FAILED;
        this->m_p_hmi_comm->SendCmd(cmd);
        g_ptr_trace->PrintLog(LOG_ALARM, "�ѵ����ⲿ���������������������");
        return;
    }
    if(cmd.data_len < 5){	//���ݳ��Ȳ��Ϸ�
        cmd.cmd_extension = FAILED;
        this->m_p_hmi_comm->SendCmd(cmd);
        g_ptr_trace->PrintLog(LOG_ALARM, "ChannelControl::ProcessHmiSetMacroVarCmd()���ݳ��Ȳ��Ϸ���data_len = %hu��", cmd.data_len);
        return;
    }
    memcpy(&start_index, &cmd.data[0], 4);
    memcpy(&count, &cmd.data[4], 1);

    if(count > 100 || cmd.data_len != 5+(sizeof(double)+1)*count){ //���ݳ��Ȳ��Ϸ�
        cmd.cmd_extension = FAILED;
        this->m_p_hmi_comm->SendCmd(cmd);
        g_ptr_trace->PrintLog(LOG_ALARM, "ChannelControl::ProcessHmiSetMacroVarCmd()���ݳ��Ȳ��Ϸ���data_len = %hu, count= %hhu��",
                              cmd.data_len, count);
        return;
    }

    //��������
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
 * @brief ����ӹ�������������
 * @param cmd : HMI�����
 */
void ChannelControl::ProcessHmiClearWorkPieceCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;   //���ûظ���־

    this->m_channel_status.workpiece_count = 0;
    cmd.cmd_extension = SUCCEED;

    this->m_p_hmi_comm->SendCmd(cmd);

    g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
    this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);   //֪ͨHMI�ӹ��������
}

/**
 * @brief �����ܹ�������������
 * @param cmd : HMI�����
 */
void ChannelControl::ProcessHmiClearTotalPieceCmd(HMICmdFrame &cmd)
{
    cmd.frame_number |= 0x8000;   //���ûظ���־

    this->m_channel_status.workpiece_count = 0;
    this->m_channel_status.workpiece_count_total = 0;
    cmd.cmd_extension = SUCCEED;

    this->m_p_hmi_comm->SendCmd(cmd);

    g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
    g_ptr_parm_manager->SetTotalWorkPiece(m_n_channel_index, m_channel_status.workpiece_count_total);
    this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);   //֪ͨHMI�ӹ��������
}

/**
 * @brief ����ӹ���λ����
 * @param cmd : HMI�����
 */
void ChannelControl::ProcessHmiRestartCmd(HMICmdFrame &cmd){
    //��ȡ��λ�к�
    uint64_t line = 0;
    if(this->m_channel_status.chn_work_mode != AUTO_MODE ||
            this->m_channel_status.machining_state == MS_WARNING){  //���Զ�ģʽ�����߸澯״̬�������Լӹ���λ
        cmd.cmd_extension = FAILED;
    }else if(cmd.data_len == 8){
        memcpy(&line, cmd.data, sizeof(line));
        printf("ProcessHmiRestartCmd, restart line = %llu\n", line);
        cmd.cmd_extension = SUCCEED;
    }else{
        printf("ProcessHmiRestartCmd, error data in data_len:%hu\n", cmd.data_len);
        cmd.cmd_extension = FAILED;
    }


    //���ȷ�����Ӧ��Ϣ
    cmd.frame_number |= 0x8000;
    cmd.channel_index = this->m_n_channel_index;
    //	cmd.cmd_extension = SUCCEED;
    cmd.data_len = 0;
    this->m_p_hmi_comm->SendCmd(cmd);


    if(cmd.cmd_extension == SUCCEED){ //����ִ�мӹ���λ
        if(this->m_channel_status.machining_state != MS_READY){
            this->StopRunGCode();  //ֹͣ��ǰ����
        }

        //���üӹ���λ��״̬��
        this->m_n_restart_line = line;
        this->m_n_restart_step = 1;
        this->m_n_restart_mode = NORMAL_RESTART;

        m_channel_rt_status.line_no = m_n_restart_line;
    }

}

/**
 * @brief ���������������
 * @param cmd : HMI�����
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
 * @brief �����ֶ��Ե�ָ��
 * @param cmd : HMI�����
 */
void ChannelControl::ProcessHmiManualToolMeasureCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;   //���ûظ���־

    if(this->m_channel_status.chn_work_mode >= MANUAL_STEP_MODE ){
        if(cmd.cmd_extension == 0x00){  //����ִ���ֶ��Ե�
            if(cmd.data_len != 2*sizeof(int) ||
                    this->m_b_manual_tool_measure ||
                    this->m_b_cancel_manual_tool_measure){
                cmd.cmd_extension = REFUSE;
                this->m_p_hmi_comm->SendCmd(cmd);  //�ظ���Ӧ����
            }else{
                cmd.cmd_extension = APPROVE;

                this->m_p_hmi_comm->SendCmd(cmd);  //�ظ���Ӧ����

                int h_code = 0;
                int times = 0;
                memcpy(&h_code, cmd.data, sizeof(int));
                memcpy(&times, cmd.data+sizeof(int), sizeof(int));
                this->ManualToolMeasure(h_code, times);
            }

        }else if(cmd.cmd_extension == 0x01){   //����ȡ���ֶ��Ե�
            cmd.cmd_extension = APPROVE;

            if(m_b_manual_tool_measure && !m_b_cancel_manual_tool_measure){
                //ȡ���Ե�
                m_b_cancel_manual_tool_measure = true;
                this->StopRunGCode(true);
            }

            this->m_p_hmi_comm->SendCmd(cmd);  //�ظ���Ӧ����
        }
    }else{
        cmd.cmd_extension = REFUSE;
        this->m_p_hmi_comm->SendCmd(cmd);  //�ظ���Ӧ����
    }

}

/**
 * @brief ����HMI�����ᵱǰλ�õĻ�е��������
 * @param cmd : HMI�����
 */
void ChannelControl::ProcessHmiSetCurMachPosCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;   //���ûظ���־

#ifdef USES_WOOD_MACHINE
    if(cmd.data_len != 9 || this->m_channel_status.chn_work_mode != REF_MODE){  //�����е���ֻ����ԭ��ģʽ�½���
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
            //��MI������������
            int32_t pos = mach_pos*1000;   //��λת����mm->um

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

    this->m_p_hmi_comm->SendCmd(cmd);  //�ظ���Ӧ����
}

/**
 * @brief ����HMI�����Ϣ����
 * @param cmd : HMIָ���
 */
void ChannelControl::ProcessHmiClearMsgCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;
    cmd.data_len = 0;



    if(cmd.cmd_extension == 0xFF){  //��ո澯����
        g_ptr_alarm_processor->Clear();
        cmd.cmd_extension = SUCCEED;
    }else if(cmd.cmd_extension == 0xEE){  //������漰��ʾ��Ϣ
        g_ptr_alarm_processor->ClearWarning(this->m_n_channel_index);
        cmd.cmd_extension = SUCCEED;
    }else{
        cmd.cmd_extension = FAILED;
    }

    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief �������òο�������
 * @param cmd
 */
void ChannelControl::ProcessHmiSetRefCmd(HMICmdFrame &cmd){

    uint8_t cur_axis_mac = this->GetCurPhyAxis();		//��ǰͨ�����Ӧ�Ļ�е���

    if(cur_axis_mac != 0){
        this->SetAxisRefCur(cur_axis_mac);	//���ͻ�ȡ��ǰ������ֵ������

        //�ûزο����־
        m_channel_status.returned_to_ref_point |= (0x01<<m_channel_status.cur_axis);
        this->m_p_channel_engine->SetRetRefFlag(cur_axis_mac-1, true);
        this->SendChnStatusChangeCmdToHmi(REF_POINT_FLAG);
    }
    else
        printf("��ǰ������δ���ã�\n");

    printf("process set ref cmd, %d\n", cur_axis_mac);

    cmd.data[0] = SUCCEED;
    cmd.data_len = 0x01;
    cmd.frame_number |= 0x8000;
    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief ���ص�ǰͨ�����Ӧ���������
 * @return ���ص�ǰ�������������ţ���1��ʼ��0��ʾδ����
 */
uint8_t ChannelControl::GetCurPhyAxis(){
    return m_p_channel_config->chn_axis_phy[m_channel_status.cur_axis];  //return this->m_channel_status.cur_chn_axis_phy[m_channel_status.cur_axis];
}


/**
 * @brief ��ȡָ����ĵ�ǰ������ֵ���˴�ֻ���ͻ�ȡ����
 * @param axis �� ��������, ��1��ʼ
 *
 */
void ChannelControl::SetAxisRefCur(uint8_t axis){
    m_p_mi_comm->SetAxisRefCur(axis,m_p_axis_config[axis-1].axis_home_pos[0]);
}

/**
 * @brief ���õ�ǰ���ղ������
 * @param index : ���ղ������,���л����ղ���
 * @return true--�ɹ�    false--ʧ�ܣ��趨ֵ��������Χ
 */
bool ChannelControl::SetCurProcParamIndex(uint8_t index){
    if(index < kMaxProcParamCount){
        if(index == m_n_cur_proc_group)  //��ͬ��ֱ���˳�
            return true;

        this->m_n_cur_proc_group = index;
        g_ptr_parm_manager->SetCurProcParamIndex(m_n_channel_index, index);
        this->m_p_f_reg->PPI = m_n_cur_proc_group;


        //�л�ͨ������
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


        //���²������µ�MC
        this->SetMcChnPlanMode();
        this->SetMcChnPlanParam();
        this->SetMcTapPlanParam();
        this->SetMcChnCornerStopParam();
        this->SetMcChnPlanFun();
#ifdef USES_WOOD_MACHINE
        this->SetMcFlipCompParam();
#endif

        //�л������
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

        g_ptr_parm_manager->ChangeChnProcParamIndex(this->m_n_channel_index, index);  //�����������


    }else
        return false;
    return true;
}

/**
 * @brief �ӹ���λִ�к�����ͨ������Msg�ķ�ʽ���ؽ�ִ��ʱ��ģ̬
 * @param line_no : �ӹ���λ���к�
 */
void ChannelControl::DoRestart(uint64_t line_no){

    printf("ChannelControl::DoRestart: line=%llu\n", line_no);

    RecordMsg *msg = nullptr;
    DPointChn tar = this->m_channel_rt_status.cur_pos_work;	//�յ�
    DPointChn src = this->m_channel_rt_status.cur_pos_work;   //���
    uint32_t axis_mask = 0;   //������


    ListNode<RecordMsg *> *node = this->m_p_output_msg_list_auto->HeadNode();

    //����ģ̬
    if(this->m_channel_status.gmode[14] != this->m_mode_restart.gmode[14]){//��������ϵ��ƥ��
        printf("��������ϵ��%hu to %hu\n", m_channel_status.gmode[14], m_mode_restart.gmode[14]);
        msg = new CoordMsg(tar, src, m_mode_restart.gmode[14], axis_mask);
        if(msg != nullptr){
            msg->SetLineNo(line_no);
            this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //���빤������ϵ�趨ָ��
        }
    }
    if(m_channel_status.cur_d_code != m_mode_restart.cur_d_code ||
            m_channel_status.gmode[7] != m_mode_restart.gmode[7]){  //G40ģ̬��ƥ���Dֵ��ͬ
        printf("������G40ģ̬��%hu to %hu�� D %hhu to %hhu\n", m_channel_status.gmode[7], m_mode_restart.gmode[7],
                m_channel_status.cur_d_code, m_mode_restart.cur_d_code);

        m_channel_status.cur_d_code = m_mode_restart.cur_d_code;
    }
    if(m_channel_status.cur_h_code != m_mode_restart.cur_h_code ||
            m_channel_status.gmode[8] != m_mode_restart.gmode[8]){  //G49ģ̬��ƥ���Hֵ��ͬ
        printf("����G49ģ̬��%hu to %hu, cur_h = %hhu to %hhu\n", m_channel_status.gmode[8], m_mode_restart.gmode[8],
                m_channel_status.cur_h_code, m_mode_restart.cur_h_code);

        msg = new CompensateMsg(m_mode_restart.gmode[8], m_mode_restart.cur_h_code, src, tar, axis_mask, m_mode_restart.rated_feed, MOVE_G00);
        if(msg != nullptr){
            msg->SetLineNo(line_no);
            this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //���뵶�߳��Ȳ�������ϵ�趨ָ��
        }
    }
    memcpy(this->m_channel_status.gmode, this->m_mode_restart.gmode, sizeof(uint16_t)*kMaxGModeCount);  //��ģ̬���帳ֵ��ͨ����ǰģ̬��
    this->InitMcModeStatus();


    //�����Ƿ�һ�£���һ���򻻵�
    if(this->m_channel_status.cur_tool != this->m_mode_restart.cur_tool){  //��һ�£�����

        //		msg = new AuxMsg(6, 1);    //M06����ָ��
        //		if(msg != nullptr){
        //			msg->SetLineNo(line_no);
        //			this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //���뻻��ָ��
        //		}

        m_channel_status.cur_tool = m_mode_restart.cur_tool;
        SendModeChangToHmi(T_MODE);
    }

    //�л�����״̬
    if(this->m_channel_status.rated_spindle_speed != this->m_mode_restart.rated_spindle_speed){  //ָ���ٶȲ�ͬ
        msg = new SpeedMsg(m_mode_restart.rated_spindle_speed);
        if(msg != nullptr){
            msg->SetLineNo(line_no);
            this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //����Sֵָ��ָ��
        }
    }
    if(this->m_channel_status.spindle_dir != this->m_mode_restart.spindle_dir){  //���Ṥ��״̬��ͬ
        int mcode = 5;  //Ĭ��M05
        if(m_mode_restart.spindle_dir == SPD_DIR_POSITIVE)
            mcode = 3;
        else if(m_mode_restart.spindle_dir == SPD_DIR_NEGATIVE)
            mcode = 4;
        msg = new AuxMsg(&mcode, 1);
        if(msg != nullptr){
            msg->SetLineNo(line_no);
            this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //����������תָ��
        }
    }

    if(this->m_channel_rt_status.cur_pos_work != m_mode_restart.pos_target){   //��ǰλ�ò������λ��
        printf("���겻һ�£�[%lf, %lf, %lf] to [%lf, %lf, %lf]\n", m_channel_rt_status.cur_pos_work.m_df_point[0], m_channel_rt_status.cur_pos_work.m_df_point[1],
                m_channel_rt_status.cur_pos_work.m_df_point[2], m_mode_restart.pos_target.m_df_point[0], m_mode_restart.pos_target.m_df_point[1],
                m_mode_restart.pos_target.m_df_point[2]);

        src = this->m_channel_rt_status.cur_pos_work;
        tar = src;
        uint8_t chn_axis_z = this->GetChnAxisFromName(AXIS_NAME_Z);
        uint8_t phy_axis_z = this->GetPhyAxis(chn_axis_z);

        //Z��̧����еԭ��
        if(phy_axis_z != NO_AXIS){
            tar.SetAxisValue(chn_axis_z, this->m_p_axis_config[phy_axis_z].axis_home_pos[0]);
            axis_mask = 0x01<<chn_axis_z;
            msg = new RapidMsg(src, tar, axis_mask);
            if(msg != nullptr){
                msg->SetLineNo(line_no);
                msg->SetFlag(FLAG_BLOCK_OVER, true);
                this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //����̧Z��ָ��
            }
        }

        //��XYZ�Ḵλ
        axis_mask = 0;
        uint8_t phy_axis = 0;
        src = tar;
        for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            phy_axis = this->GetPhyAxis(i);
            if(phy_axis == NO_AXIS)
                continue;
            if(this->m_p_axis_config[phy_axis].axis_type != AXIS_SPINDLE && this->m_p_axis_config[phy_axis].sync_axis == 0 &&
                    m_p_channel_config->chn_axis_name[i] > AXIS_NAME_Z){   //�����ᣬ�ǴӶ��ᣬ�ҷ�XYZ��
                axis_mask |= 0x01<<i;
                tar.m_df_point[i] = m_mode_restart.pos_target.m_df_point[i];
            }
        }
        if(axis_mask != 0){
            msg = new RapidMsg(src, tar, axis_mask);
            if(msg != nullptr){
                msg->SetLineNo(line_no);
                msg->SetFlag(FLAG_BLOCK_OVER, true);
                this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //�����XYZ�Ḵλָ��
            }
        }


        //XY���˶������λ��
        axis_mask = 0;
        phy_axis = 0;
        src = tar;
        for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            phy_axis = this->GetPhyAxis(i);
            if(phy_axis == NO_AXIS)
                continue;
            if(this->m_p_axis_config[phy_axis].axis_type != AXIS_SPINDLE &&
                    m_p_channel_config->chn_axis_name[i] < AXIS_NAME_Z){   //�����ᣬXY��
                axis_mask |= 0x01<<i;
                tar.m_df_point[i] = m_mode_restart.pos_target.m_df_point[i];
            }
        }
        if(axis_mask != 0){
            msg = new RapidMsg(src, tar, axis_mask);
            if(msg != nullptr){
                msg->SetLineNo(line_no);
                msg->SetFlag(FLAG_BLOCK_OVER, true);
                this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //�����XYZ�Ḵλָ��
            }
        }

        //Z���½������λ��
        src = tar;
        if(phy_axis_z != NO_AXIS){
            tar.m_df_point[chn_axis_z] = m_mode_restart.pos_target.m_df_point[chn_axis_z];
            axis_mask = 0x01<<chn_axis_z;
            msg = new RapidMsg(src, tar, axis_mask);
            if(msg != nullptr){
                msg->SetLineNo(line_no);
                msg->SetFlag(FLAG_BLOCK_OVER, true);
                this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //����Z�Ḵλָ��
            }
        }
    }

    //���븴λ�����Ϣ
    msg = new RestartOverMsg();
    if(msg != nullptr){
        msg->SetLineNo(line_no);
        this->m_p_output_msg_list_auto->InsertBefore(msg, node);   //����ӹ���λ���ָ��
    }


    //����RESTART��־
    this->m_n_restart_step = 2;

}

/**
 * @brief ��������ת�ٶ�Ӧ��DAֵ��MI
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
 * @brief ���������ģʽ�л����MI
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
 * @brief �����ٶ��������ٶ�ֵ��MI  value:um/s   0.001��/s
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
 * @brief ������������������ֵ��MI
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
 * @brief ����ָ��������λ�ö�Ӧ�ı�����ֵ
 * @param axis	�� ��������
 * @param encoder ��������ֵ
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
 * @brief ����ָ�����Z�źŲ�����
 * @param axis : ��������
 */
void ChannelControl::ActiveAxisZCapture(uint8_t axis){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_ACTIVE_Z_CAPT;
    cmd.data.axis_index = axis;
    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief ������λ��������Ȧָ��
 * @param axis : �������, ��1��ʼ
 * @param mode : ��Ȧ��ģ����λum
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
// * @brief ���͹�˿��Ÿ�MI
// * @param spd : �����Ӧ��������ţ���1��ʼ
// * @param zAxis �� Z���Ӧ��������ţ���1��ʼ
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

// * @brief ���͹�˿������MI
// */
//void ChannelControl::SendMiTapParamCmd(){
//	//��ǰʹ������ο���7��8��9���湥˿ǰ�����桢����ϵ����

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
// * @brief ���͹�˿������MI
// * @param state :
// * @param ratio �� ��˿������10000*S/F��S��λΪrpm��F��λΪmm/min
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
// * @brief ���͹�˿״̬��MI
// * @param state : ��˿״̬��true��ʾ����false��ʾ��
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
 * @brief ����ͨ����-������ӳ���MI������������
 */
void ChannelControl::SendMiChnAxisMap(){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_CHN_PHY_MAP;
    cmd.data.reserved = this->m_n_channel_index;

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        cmd.data.axis_index = i+1;

        cmd.data.data[0] = this->m_p_channel_config->chn_axis_phy[i];   //cmd.data.data[0] = this->m_channel_status.cur_chn_axis_phy[i];   //��Ӧ�������

        this->m_p_mi_comm->WriteCmd(cmd);

        //		printf("chan axis = %hhu  chan= %hhu  phyaxis = %hhu",i , this->m_n_channel_index,this->m_p_channel_config->chn_axis_phy[i] );
    }
}

/**
 * @brief ����ͨ����-������ӳ���MI�����ᣬ��Ҫ������ӳ���ϵ�Ķ�̬�л�
 */
void ChannelControl::SendAxisMapCmdToMi(uint8_t phy_axis,uint8_t chan,uint8_t axis){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_CHN_PHY_MAP;
    cmd.data.reserved = chan;
    cmd.data.axis_index = axis+1;
    cmd.data.data[0] = phy_axis+1; // this->m_p_channel_config->chn_axis_phy[i];   //��Ӧ�������
    cmd.data.data[1] = 1;  // 0--��ʼ����  1--��̬�л�
    this->m_p_mi_comm->WriteCmd(cmd);
}


/**
 * @brief ��ȡͨ��״̬
 * @param status
 */
void ChannelControl::GetChnStatus(HmiChannelStatus &status){
    memcpy((char *)&status, (char *)&m_channel_status, sizeof(HmiChannelStatus));
}

/**
 * @brief ����HMI��ȡͨ��״ָ̬��
 * @param cmd : HMIָ��
 */
void ChannelControl::ProcessHmiGetChnStateCmd(HMICmdFrame &cmd){
    //������Ӧ��
    cmd.frame_number |= 0x8000;
    cmd.cmd_extension = SUCCEED;
    cmd.data_len = (uint16_t)sizeof(HmiChannelStatus);
    memcpy(cmd.data, (char *)&m_channel_status, cmd.data_len);
    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief ����HMI����ָ��
 * @param cmd : HMIָ��
 */
void ChannelControl::ProcessHmiSimulateCmd(HMICmdFrame &cmd){
    //	if(cmd.cmd_extension == SIMULATE_OUTLINE){//��������
    //
    //
    //	}
    //	else if(cmd.cmd_extension == SIMULATE_TOOLPATH){//��·����
    //
    //	}
    //	else if(cmd.cmd_extension == SIMULATE_MACHINING){//�ӹ�����
    //
    //	}

    //���ñ�����ģʽ
    if(this->m_channel_status.machining_state == MS_PAUSED){

        this->StopRunGCode();    //������ͣ״̬��Ҫ�ȸ�λG��������

        m_channel_status.machining_state = MS_READY;
    }

    if(m_channel_status.machining_state == MS_READY){

        this->SetSimulateMode((SimulateMode)cmd.cmd_extension);
        //����������
        this->StartRunGCode();
        cmd.data[0] = SUCCEED;
    }else{
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�л�����ģʽʧ�ܣ���ǰ[%d]��Ŀ��[%d]��", m_n_channel_index,
                              this->m_simulate_mode, cmd.cmd_extension);  	//����ʧ��

        //���
        cmd.data[0] = FAILED;
    }

    //������Ӧ��
    cmd.frame_number |= 0x8000;
    cmd.data_len = 1;
    m_p_hmi_comm->SendCmd(cmd);

}

/**
 * @brief ���õ�ǰ�ӹ��ļ�
 * @param cmd
 */
void ChannelControl::ProcessHmiSetNcFileCmd(HMICmdFrame &cmd){

    cmd.frame_number |= 0x8000;   //���ûظ���־
    // @add zk ��ֹ�򿪹����ļ����ļ�
    if(strlen(cmd.data) > 127) return;
    if (strcmp(m_channel_status.cur_nc_file_name, cmd.data))
        m_time_remain = 0;
    if(this->m_channel_status.chn_work_mode != AUTO_MODE){   //���Զ�ģʽ
        strcpy(m_channel_status.cur_nc_file_name, cmd.data);
        cmd.cmd_extension = SUCCEED;

        this->m_p_hmi_comm->SendCmd(cmd);

        if(m_channel_status.machining_state == MS_PAUSED){//ͨ����λ
            this->Reset();

        }
        g_ptr_parm_manager->SetCurNcFile(m_n_channel_index, m_channel_status.cur_nc_file_name);    //�޸ĵ�ǰNC�ļ�

        char path[kMaxPathLen] = {0};
        strcpy(path, PATH_NC_FILE);
        strcat(path, m_channel_status.cur_nc_file_name);   //ƴ���ļ�����·��
        this->m_p_compiler->OpenFileInScene(path);
    }else if(m_channel_status.machining_state == MS_RUNNING ||
             m_channel_status.machining_state == MS_OUTLINE_SIMULATING ||
             m_channel_status.machining_state == MS_TOOL_PATH_SIMULATING ||
             m_channel_status.machining_state == MS_MACHIN_SIMULATING ||
             m_channel_status.machining_state == MS_PAUSING){  //��������н�ֹ����

        cmd.cmd_extension = FAILED;
        this->m_p_hmi_comm->SendCmd(cmd);
    }
    else{
        strcpy(m_channel_status.cur_nc_file_name, cmd.data);
        cmd.cmd_extension = SUCCEED;

        this->m_p_hmi_comm->SendCmd(cmd);

        if(m_channel_status.machining_state == MS_PAUSED){//ͨ����λ
            this->Reset();

        }
        g_ptr_parm_manager->SetCurNcFile(m_n_channel_index, m_channel_status.cur_nc_file_name);    //�޸ĵ�ǰNC�ļ�

        char path[kMaxPathLen] = {0};
        strcpy(path, PATH_NC_FILE);
        strcat(path, m_channel_status.cur_nc_file_name);   //ƴ���ļ�����·��
        this->m_p_compiler->OpenFile(path);
        printf("set nc file cmd2 : %s\n", path);
    }
}

/**
 * @brief ����MDA����
 * @param cmd : ָ��
 */
void ChannelControl::ProcessMdaData(HMICmdFrame &cmd){

    printf("ChannelControl::ProcessMdaData, chn=%hhu\n", this->m_n_channel_index);
    if(this->m_channel_status.chn_work_mode != MDA_MODE) //��MDAģʽ���˳�
        return;
    int fp = open(m_str_mda_path, O_CREAT|O_TRUNC|O_WRONLY); //���ļ�
    if(fp < 0){
        CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
        this->m_error_code = ERR_OPEN_FILE;
        g_ptr_trace->PrintLog(LOG_ALARM, "��MDA��ʱ�ļ�[%s]ʧ�ܣ�", m_str_mda_path);
        return;//�ļ���ʧ��
    }
    else{
        printf("open file %s\n", m_str_mda_path);
    }

    ssize_t res = write(fp, cmd.data, cmd.data_len);
    if(res == -1){//д��ʧ��
        close(fp);
        CreateError(ERR_LOAD_MDA_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
        this->m_error_code = ERR_LOAD_MDA_DATA;
        return;
    }else if(res != cmd.data_len){//����ȱʧ
        close(fp);
        CreateError(ERR_LOAD_MDA_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
        this->m_error_code = ERR_LOAD_MDA_DATA;
        printf("write mda temp file error, plan = %d, actual = %d\n", cmd.data_len, res);
        return;
    }

    close(fp);

    if(cmd.data_len == 0){
        printf("MDA data length is 0!\n");
        this->SetMachineState(MS_READY);  //����ͨ��״̬
        return;
    }

    // @test zk

    //m_channel_status.chn_work_mode = AUTO_MODE;
    //this->SendWorkModeToMc(MC_MODE_AUTO);
    //this->m_p_compiler->SetMode((CompilerWorkMode)MC_MODE_AUTO);

    if(!m_p_compiler->OpenFile(m_str_mda_path)){		//���������ļ�ʧ��
        return;
    }


    printf("ProcessMdaData() m_n_run_thread_state: %d\n", m_n_run_thread_state);
    if(m_n_run_thread_state == IDLE || m_n_run_thread_state == PAUSE){
        pthread_mutex_lock(&m_mutex_change_state);
        //printf("locked 2\n");
        m_n_run_thread_state = RUN;  //��Ϊ����״̬
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 2\n");
    }

    this->m_p_f_reg->OP = 1;   //�Զ�����״̬

}

/**
 * @brief ��HMI���͹���״̬�ı�����
 * @param mach_state ��ͨ���ӹ�״̬
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
 * @brief ��HMI���͹���ģʽ����
 * @param chn_work_mode : ͨ������ģʽ
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
        {"�Ƿ�ģʽ", "�Զ�ģʽ", "MDIģʽ", "����ģʽ", "�ֶ�ģʽ", "����ģʽ", "�༭ģʽ", "����ģʽ"};
    if (chn_work_mode > 0 && chn_work_mode < table.size())
    {
        g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug,
                                            string("[ģʽѡ��] " + table[chn_work_mode]));
    }


    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief ��HMI����ͨ��״̬�ı�����
 * @param status_type : ״̬����
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
 * @brief ��HMI���ͷ�Gģ̬״̬�ı��������T/D/H/F/S
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
                                            string("[����T]�л�Ϊ " + to_string(m_channel_status.cur_tool)));
        //		printf("SendModeChangToHmi, T = %hhu\n", m_channel_status.cur_tool);
        break;
    case D_MODE:		//D
        cmd.data_len = 0x01;
        cmd.data[0] = m_channel_status.cur_d_code;
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug,
                                            string("[D]�л�Ϊ " + to_string(m_channel_status.cur_d_code)));
        break;
    case H_MODE:		//H
        cmd.data_len = 0x01;
        cmd.data[0] = m_channel_status.cur_h_code;
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug,
                                            string("[H]�л�Ϊ " + to_string(m_channel_status.cur_h_code)));
        break;
    case F_MODE:		//F
        cmd.data_len = sizeof(m_channel_status.rated_feed);
        memcpy(cmd.data, &m_channel_status.rated_feed, cmd.data_len);
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug,
                                            string("[�����ٶ�F]�л�Ϊ " + to_string(m_channel_status.rated_feed*60/1000)));
        break;
    case S_MODE:		//S

        cmd.data_len = sizeof(m_channel_status.rated_spindle_speed);
        memcpy(cmd.data, &m_channel_status.rated_spindle_speed, cmd.data_len);
        g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug,
                                            string("[����ת��S]�л�Ϊ " + to_string(m_channel_status.rated_spindle_speed*60/1000)));
        break;
    default:
        printf("@@@@Invalid value in ChannelControl::SendModeChangToHmi\n");
        break;
    }


    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief ��HMI����MDA��������
 * @return true-�ɹ�    false-ʧ��
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
 * @brief ��HMI������ʾ��Ϣ
 * @param msg_type : ��Ϣ����
 * @param msg_id �� ��ϢID
 * @param msg_param : ��Ϣ����
 * @return true--���ͳɹ�  false--����ʧ��
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
 * @brief �����ֶ��Ե������HMI
 * @param res : �Ե����
 * @return true--�ɹ�   false--ʧ��
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
 * @brief ��յ�ǰ��ʾ��Ϣ
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelControl::ClearHmiInfoMsg(){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_CLEAR_ALARM;
    cmd.cmd_extension = 0xEE;   //���������ʾ��Ϣ
    cmd.data_len = 0;


    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief ��HMI���¼ӹ�����
 * @param count : ��������
 * @return true--�ɹ�   false--ʧ��
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
 * @brief ֪ͨHMI�ӹ����
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelControl::SendMachOverToHmi(){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = m_n_channel_index;
    cmd.cmd = CMD_SC_NOTIFY_MACH_OVER;

    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief ��HMI�����л�NC�ļ���ʾ����
 * @param filename
 * @return true--�ɹ�   false--ʧ��
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
 * @brief ���üӹ�״̬
 * @param mach_state
 */
void ChannelControl::SetMachineState(uint8_t mach_state){
    // @modify zk ��¼֮ǰ״̬
	uint8_t old_stat = m_channel_status.machining_state;

	g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "Enter SetMachineState, old = %d, new = %d, m_n_run_thread_state = %d\n", m_channel_status.machining_state, mach_state, m_n_run_thread_state);
    if(m_channel_status.machining_state == mach_state)
        return;

    if(mach_state == MS_WARNING && m_channel_status.machining_state == MS_STOPPING
            && m_channel_mc_status.cur_mode != MC_MODE_MANUAL)  //�ȴ�ֹͣ��λ�����л�Ϊ�澯״̬
        return;

    m_channel_status.machining_state = mach_state;       //����ͨ��״̬

    if(mach_state == MS_PAUSED || mach_state == MS_READY){
        m_b_mc_need_start = true;
        //		printf("set mc need start true~~~~~\n");
    }

    //ע��˵������Ӧ�ڴ˴������ֳ�����Ϊ�岹��ɺ��˶���λ����ʱ���˴������ֳ������λ�����
    //	if(mach_state == MS_PAUSED){
    //		this->SaveAutoScene();
    //	}

    SendMachineStateCmdToHmi(mach_state);   //֪ͨHMI
    SendMachineStateToMc(mach_state);       //֪ͨmc

    //printf("========== old_stat %d, cur_stat %d\n", old_stat, mach_state);

    // @modify zk �޸�֮����
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

    // @modify zk ֮ǰ�Ĵ���
    /* if(mach_state == MS_PAUSED || mach_state == MS_WARNING){
    	this->m_p_f_reg->STL = 0;
        this->m_p_f_reg->SPL = 1;
        printf("111111111111111\n");
        //this->m_b_need_change_to_pause = true;
        //@TODO ֮��Ӹ������ź� ���ȥ������ͣ�ź�
        //�����޷�ֱ�Ӹ�G�ź�  ���� pause ���������
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
 * @brief �л�����ģʽ
 * @param work_mode : ����ģʽ
 * @return  ��
 */
void ChannelControl::SetWorkMode(uint8_t work_mode){

    if(this->m_channel_status.chn_work_mode == work_mode)
        return;

    //	if(work_mode == AUTO_MODE)
    //		mc_work_mode = 1;
    //	else if(work_mode == MDA_MODE)
    //		mc_work_mode = 2;

    static bool bSaveAutoScene = true;   //�Ƿ񱣴��Զ�ģʽ״̬

    // ���������ֱ���л�ģʽ�����µĴ��»ָ��Զ�ʱ�����µ����˶�������������
    if(m_channel_status.chn_work_mode == AUTO_MODE &&
            m_channel_status.machining_state == MS_RUNNING)
    {
        this->Pause();
        usleep(200000);
    }

    //����ǰģʽ���˳�����
    switch(m_channel_status.chn_work_mode){
    case AUTO_MODE:
        //		this->m_n_restart_line = 0;
        //		this->m_n_restart_mode = NOT_RESTART;
        this->m_n_restart_step = 0;
        if(this->m_thread_breakcontinue > 0){//���ڶϵ�����߳�ִ�й����У����˳��ϵ�����߳�
            this->m_b_cancel_breakcontinue_thread = true;
            bSaveAutoScene = false;   //��ʱ����Ҫ�����Զ�ģʽ״̬
        }
    case MDA_MODE:
        pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
        //printf("locked 3\n");
        if(this->m_channel_status.machining_state == MS_RUNNING){//����ʱ��Ҫ����ͣ
            //printf("paus1111\n");
            this->PauseRunGCode();
            this->m_change_work_mode_to = work_mode;	//����Ŀ��ģʽ����ϵͳͣ�������л�
            pthread_mutex_unlock(&m_mutex_change_state);
            return;
        }
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 3\n");
        break;
    case MANUAL_STEP_MODE:
    case MANUAL_MODE:
        //ֹͣ�Ե�
        if(this->m_b_manual_tool_measure){  //��ֹ�ֶ��Ե�
            m_b_cancel_manual_tool_measure = true;
            this->StopRunGCode(true);
        }

        //ֹͣ�ֶ���������
        if(this->m_b_manual_call_macro){
            this->m_b_cancel_manual_call_macro = true;
            this->StopRunGCode(true);
        }


        //�ֶ�ֹͣ
        this->ManualMoveStop();
        this->m_p_f_reg->MINC = 0;
        this->m_p_f_reg->MJ = 0;
        break;
    case MPG_MODE:
        //ֹͣ�Ե�
        if(this->m_b_manual_tool_measure){  //��ֹ�ֶ��Ե�
            m_b_cancel_manual_tool_measure = true;
            this->StopRunGCode(true);
        }

        //ֹͣ�ֶ���������
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
    //�л�ģʽ
    switch(work_mode){
    case AUTO_MODE:
    case MDA_MODE:
        pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
        //printf("locked 4\n");
        //�����л������Ϣ�������
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
        this->m_p_compiler->SetMode((CompilerWorkMode)work_mode);	//�������л�ģʽ
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //�л��������
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 4\n");

        //		//����OP�ź�
        //		if(!this->m_p_channel_engine->IsEmergency() && !g_ptr_alarm_processor->HasErrorInfo(m_n_channel_index)){
        //			this->m_p_f_reg->OP = 1;
        //		}
        break;
    case MANUAL_STEP_MODE:
        //		this->m_p_f_reg->OP = 0;
        this->m_p_f_reg->MINC = 1;

        //�ֶ�ģʽ�£�����������ΪMDAģʽ
        //pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
        printf("locked 5\n");
        m_p_output_msg_list = m_p_output_msg_list_mda;
        m_n_run_thread_state = IDLE;
        this->m_p_compiler->SetMode(MDA_COMPILER);	//�������л�ģʽ
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //�л��������
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 5\n");
        this->SendWorkModeToMc(MC_MODE_MANUAL);
        break;
    case MANUAL_MODE:
        //		this->m_p_f_reg->OP = 0;
        this->m_p_f_reg->MJ = 1;

        //�ֶ�ģʽ�£�����������ΪMDAģʽ
        pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
        //printf("locked 6\n");
        m_p_output_msg_list = m_p_output_msg_list_mda;
        m_n_run_thread_state = IDLE;
        this->m_p_compiler->SetMode(MDA_COMPILER);	//�������л�ģʽ
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //�л��������
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 6\n");
        this->SendWorkModeToMc(MC_MODE_MANUAL);
        break;
    case MPG_MODE:
        //		this->m_p_f_reg->OP = 0;
        this->m_p_f_reg->MH = 1;

        //�ֶ�ģʽ�£�����������ΪMDAģʽ
        pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
        //printf("locked 7\n");
        m_p_output_msg_list = m_p_output_msg_list_mda;
        m_n_run_thread_state = IDLE;
        this->m_p_compiler->SetMode(MDA_COMPILER);	//�������л�ģʽ
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //�л��������
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 7\n");
        this->SendWorkModeToMc(MC_MODE_MANUAL);
        break;
    case REF_MODE:
        this->m_p_f_reg->MREF = 1;

        //ԭ��ģʽ�£�����������ΪMDAģʽ
        pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
        //printf("locked 8\n");
        m_p_output_msg_list = m_p_output_msg_list_mda;
        m_n_run_thread_state = IDLE;
        this->m_p_compiler->SetMode(MDA_COMPILER);	//�������л�ģʽ
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //�л��������
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 8\n");
        this->SendWorkModeToMc(MC_MODE_MANUAL);
        break;
    default:
        break;
    }

    m_channel_status.chn_work_mode = work_mode;

    this->SendWorkModeCmdToHmi(work_mode); 	//֪ͨHMI����ģʽ�л�

}

/**
 * @brief ��MCģ�鷢��DSP�岹����ģʽ����
 * @param intp_mode : 0--�ֶ��岹    1--�Զ��岹    2--MDA�岹
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
 * @brief ��MCģ�鷢�ͼӹ�ģʽ�����Ҫ�������ֲ岹Ŀ��λ�ã�������㲻ͬģʽ�����ƶ���
 * @param work_mode : 0--�ֶ�    1--�Զ�    2--MDA
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
 * @brief ���������Ĵ���
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
 * @brief file_name�Ƿ�Ϊ��ǰ�ӹ��ļ�
 * @param file_name : �ļ�����
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
 * @brief ˢ��nc�ļ�file,  ���file�Ѿ����أ���unmap
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
 * @brief ����ӳ�䵱ǰ���ص�NC�ļ�
 */
bool ChannelControl::RemapFile(char *file){
    if(!this->IsCurNcFile(file))
        return true;
    return this->m_p_compiler->RemapCurNcFile();
}

/**
 * @brief ��λMCģ��ĵ�ǰ�ӹ��кţ�����
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
 * @brief G���������̺߳���
 * @param void *args: ChannelControl����ָ��
 */
void *ChannelControl::CompileThread(void *args){
    printf("Start ChannelControl::CompileThread! id = %ld\n", syscall(SYS_gettid));
    ChannelControl *p_channel_control = static_cast<ChannelControl *>(args);

    int res = ERR_NONE;
    res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
    if (res != ERR_NONE) {
        printf("Quit ChannelControl::CompileThread with error 1!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }
    res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
    if (res != ERR_NONE) {
        printf("Quit ChannelControl::CompileThread with error 2!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }

    while(!g_sys_state.system_ready) //�ȴ�ϵͳ����
        usleep(50000);  //�ȴ�50ms

    res = p_channel_control->Run();


    printf("Exit ChannelControl::CompileThread!\n");
    pthread_exit(NULL);
}

/**
 * @brief G���������к���
 * @return ִ�н��
 */
int ChannelControl::Run(){
    int res = ERR_NONE;
    //  ��ʼ��
    //	struct timeval tvStart;
    //	struct timeval tvNow;
    //	unsigned int nTimeDelay = 0;

    bool bf = true;
    int compile_count = 0;  //������������м���

    //ִ��ѭ��
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
            	//�Զ�ģʽ�£�����������������������������δ�����꣬�򲻽��б���
                //	printf("@@@@@@, last= %d, tail=%d\n", m_p_last_output_msg, m_p_output_msg_list->TailNode());
                //printf("11111\n");
            	bf = ExecuteMessage();

				if(!bf){
					if(m_error_code != ERR_NONE){
						g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "execute message error3, %d\n", m_error_code);
						m_n_run_thread_state = ERROR; //�������
					}
					else
						usleep(10000);
				}
			}else if(m_p_compiler->GetErrorCode() != ERR_NONE)
			{
				//��������������Ҫ����ִ���ѱ���ָ��
				if(m_p_compiler->RunMessage()){
					if(!ExecuteMessage()){
						if(m_error_code != ERR_NONE){
							g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "execute message error1, %d\n", m_error_code);
							m_n_run_thread_state = ERROR; //�������
						}
					}
				}
				else{
					if(m_error_code != ERR_NONE){
						m_n_run_thread_state = ERROR; //�������
					}else
						m_n_run_thread_state = WAIT_RUN;//ִ��ʧ�ܣ�״̬�л���WAIT_RUN
				}
			}
			else if(m_p_compiler->GetLineData())
			{
				//��ȡһ��Դ��
				if(!m_p_compiler->CompileLine())  //����һ�д���
				{
					m_n_run_thread_state = ERROR; //�������
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
								m_n_run_thread_state = ERROR; //�������
							}else{  //ִ��δ�ɹ���ת��ΪWAIT_EXECUTE״̬

								usleep(10000);   //����10ms
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
							m_n_run_thread_state = WAIT_RUN;//ִ��ʧ�ܣ�״̬�л���WAIT_RUN
						}
					}
				}

                if(++compile_count >= 20){
                    compile_count = 0;
                    usleep(2000);     //��������50��������2ms
                }
            }
            else if(m_p_compiler->IsCompileOver()){  //�Ѿ��������

            	ExecuteMessage();
            }
            else
            {//����
            	if(m_p_compiler->GetErrorCode() != ERR_NONE){
                    g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "Get line data error!\n");
                    m_n_run_thread_state = ERROR;
                }
            }


#ifndef USES_WOOD_MACHINE   //ľ��ר�������M30����ָ��
            if(m_p_compiler->IsEndOfFile() && !m_p_compiler->IsCompileOver() &&
                    this->m_channel_status.chn_work_mode == AUTO_MODE && !m_p_compiler->IsPreScaning() && !m_p_compiler->IsSubProgram()
        #ifdef USES_ADDITIONAL_PROGRAM
                    && this->m_n_add_prog_type != CONTINUE_START_ADD
        #endif
                    )
            {

            	m_n_run_thread_state = ERROR;
                g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�﷨����δ�ҵ�����ָ�", m_n_channel_index);
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
            //TODO �������
            g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "Compile Error:%d, %d\n", m_error_code, m_p_compiler->GetErrorCode());
            //CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            // @test zk  ������뱨�������в�δֹͣ����
            PauseMc();

            m_n_run_thread_state = STOP;
        }
        else if(m_n_run_thread_state == STOP)
        {
            //TODO �����������λ������״̬������

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
                    m_n_run_thread_state = ERROR; //�������
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
                    m_n_run_thread_state = ERROR; //�������
                }
            }

            pthread_mutex_unlock(&m_mutex_change_state);
            //printf("unlocked 11\n");
            if(!bf){
            	usleep(10000);
                m_n_run_thread_state = WAIT_RUN;    //��ֹ��ExecuteMessage�����������޸�m_n_run_thread_state
            }
            else if(m_n_run_thread_state != ERROR){
                m_n_run_thread_state = RUN; //�ȴ�ִ����ɣ� ״̬�л�RUN
            }

        }
        else
        {
            usleep(10000);   //������״̬���̹߳���10ms
        }
    }

    return res;
}


/**
 * @brief  ״̬�����̺߳���
 * @param args
 */
void *ChannelControl::RefreshStatusThread(void *args){
    ChannelControl *chn_ctrl = static_cast<ChannelControl *>(args);
    printf("Start ChannelControl[%d]::RefreshStatusThread!thread id = %ld\n", chn_ctrl->GetChnIndex(), syscall(SYS_gettid));

    int res = ERR_NONE;
    res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
    if (res != ERR_NONE) {
        printf("Quit ChannelControl::RefreshStatusThread!thread with error 1!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }
    res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
    if (res != ERR_NONE) {
        printf("Quit ChannelControl::RefreshStatusThread!thread with error 2!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }

    while(!g_sys_state.system_ready){  //�ȴ�ϵͳ����
        usleep(10000);
    }

    chn_ctrl->RefreshStatusFun();

    printf("Quit ChannelControl::RefreshStatusThread!!\n");
    pthread_exit(NULL);
}

bool ChannelControl::RefreshStatusFun(){

	bool step_mode_flag = false;   //�Ƿ񵥲�ģʽ
	int check_count = 1;           //״̬ȷ�ϴ���
	uint32_t data = 0;
	uint64_t count = 0;           //ѭ������
	while(!g_sys_state.system_quit){

		if(!this->m_b_mc_on_arm){

			//���µ�ǰMC������ģʽ
			this->m_p_mc_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);

			//���µ�ǰ�к�
			this->m_p_mc_comm->ReadCurLineNo(m_n_channel_index, m_channel_mc_status.cur_line_no);

			//���µ�ǰ�����ٶ�
			this->m_p_mc_comm->ReadCurFeed(m_n_channel_index, m_channel_mc_status.cur_feed);

			//���µ�ǰ���������ٶ�
			this->m_p_mc_comm->ReadRatedFeed(m_n_channel_index, m_channel_mc_status.rated_feed);

			//���µ�ǰ����ָ��`
			this->m_p_mc_comm->ReadCurCmd(m_n_channel_index, m_channel_mc_status.cur_cmd);

			//����MC��ǰ����������
			this->m_p_mc_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);

			//����MDA����������
			this->m_p_mc_comm->ReadMdaBufDataCount(m_n_channel_index, m_channel_mc_status.mda_data_count);

			//���µ�ǰ��岹λ��
			this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

			//����AUTO�ֿ鵽λ��־
			m_channel_mc_status.auto_block_over = m_p_mc_comm->ReadAutoBlockRunOverFlag(m_n_channel_index);

			//����MDA�ֿ鵽λ��־
			m_channel_mc_status.mda_block_over = m_p_mc_comm->ReadMdaBlockRunOverFlag(m_n_channel_index);

			//���µ��ε�λ��־
			m_channel_mc_status.step_over = m_p_mc_comm->ReadStepRunOverFlag(m_n_channel_index);

			//���µ�ǰ�ᵽλ��־
			this->m_p_mc_comm->ReadChnAxisRunoverMask(m_n_channel_index, m_channel_mc_status.axis_over_mask);
            //���µ�ǰ�ᵽλ(�ֶ�)��־
            this->m_p_mc_comm->ReadChnManuAxisRunoverMask(m_n_channel_index, m_channel_mc_status.manu_axis_over_mask);

			//���µ�ǰMC�澯��־
			this->m_p_mc_comm->ReadMcErrFlag(m_n_channel_index, m_channel_mc_status.mc_error.all);

			if(m_channel_mc_status.cur_mode == MC_MODE_AUTO &&
#ifdef USES_ADDITIONAL_PROGRAM
				m_n_add_prog_type == NONE_ADD &&
#endif
				m_b_lineno_from_mc && m_channel_mc_status.cur_line_no > 0){  //���̸���ʵʱ״̬�к�
			m_channel_rt_status.line_no = m_channel_mc_status.cur_line_no;
		    }

		}else{
			//���µ�ǰMC������ģʽ
			this->m_p_mc_arm_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);

			//���µ�ǰ�к�
			this->m_p_mc_arm_comm->ReadCurLineNo(m_n_channel_index, m_channel_mc_status.cur_line_no);

			//���µ�ǰ�����ٶ�
			this->m_p_mc_arm_comm->ReadCurFeed(m_n_channel_index, m_channel_mc_status.cur_feed);

			//���µ�ǰ���������ٶ�
			this->m_p_mc_arm_comm->ReadRatedFeed(m_n_channel_index, m_channel_mc_status.rated_feed);

			//���µ�ǰ����ָ��`
			this->m_p_mc_arm_comm->ReadCurCmd(m_n_channel_index, m_channel_mc_status.cur_cmd);

			//����MC��ǰ����������
			this->m_p_mc_arm_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);

			//����MDA����������
			this->m_p_mc_arm_comm->ReadMdaBufDataCount(m_n_channel_index, m_channel_mc_status.mda_data_count);

			//���µ�ǰ��岹λ��
			this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

			//����AUTO�ֿ鵽λ��־
			m_channel_mc_status.auto_block_over = m_p_mc_arm_comm->ReadAutoBlockRunOverFlag(m_n_channel_index);

			//����MDA�ֿ鵽λ��־
			m_channel_mc_status.mda_block_over = m_p_mc_arm_comm->ReadMdaBlockRunOverFlag(m_n_channel_index);

			//���µ��ε�λ��־
			m_channel_mc_status.step_over = m_p_mc_arm_comm->ReadStepRunOverFlag(m_n_channel_index);

			//���µ�ǰ�ᵽλ��־
			this->m_p_mc_arm_comm->ReadChnAxisRunoverMask(m_n_channel_index, m_channel_mc_status.axis_over_mask);

			//���µ�ǰMC�澯��־
			this->m_p_mc_arm_comm->ReadMcErrFlag(m_n_channel_index, m_channel_mc_status.mc_error.all);


			if(m_channel_mc_status.cur_mode == MC_MODE_AUTO &&
#ifdef USES_ADDITIONAL_PROGRAM
				m_n_add_prog_type == NONE_ADD &&
#endif
				m_b_lineno_from_mc && m_channel_mc_status.cur_line_no > 0){  //���̸���ʵʱ״̬�к�
			m_channel_rt_status.line_no = m_channel_mc_status.cur_line_no;
		    }

		}

		//printf("m_channel_mc_status.cur_mode : %d\n", m_channel_mc_status.cur_mode);
		//����ϵͳ״̬
		step_mode_flag = IsStepMode();
		if(step_mode_flag){
			check_count = 10;    //2;
		}
		else{
			check_count = 10;    //1;   //������ʱ���ȴ������е�λ
		}
		if(m_channel_mc_status.cur_mode ==MC_MODE_MANUAL){

			if(m_channel_status.machining_state == MS_PAUSING ||		//��ͣ�в���MC���л����ֶ�ģʽ
					m_channel_status.machining_state == MS_STOPPING ||
					(m_channel_status.machining_state == MS_RUNNING &&
							step_mode_flag && m_channel_mc_status.step_over && !m_b_mc_need_start)){//����ģʽ�£����ε�λ

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

					if(m_change_work_mode_to != INVALID_MODE && this->m_thread_breakcontinue == 0){//��Ҫ�л�ģʽ���ϵ����Ҳ�Ѿ�����
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


			//����ģ̬, ֻ��MC����AUTOģʽʱ�Ŵ�MC�ж�ȡģ̬
			//����MCģ̬��Ϣ
			if(!this->m_b_mc_on_arm)
				this->m_p_mc_comm->ReadMcModeInfo(m_n_channel_index, m_channel_mc_status.mc_mode.all);
			else
				this->m_p_mc_arm_comm->ReadMcModeInfo(m_n_channel_index, m_channel_mc_status.mc_mode.all);

			this->RefreshModeInfo(m_channel_mc_status.mc_mode);
		}

		if(this->m_b_change_hw_trace_state){//�л����ָ���ģʽ
			if(m_channel_mc_status.cur_mode ==MC_MODE_MANUAL && m_channel_mc_status.cur_feed < 3){
				bool res = this->ChangeHwTraceState(this->m_n_hw_trace_state_change_to);
//				printf("change to mpg trace mode:%hhu\n", m_n_hw_trace_state_change_to);

				//����MI����״̬�л���Ӧ����
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


		//�����û����������ٶ�
		if((m_channel_status.chn_work_mode == AUTO_MODE || m_channel_status.chn_work_mode == MDA_MODE) &&   //�ֶ�ģʽ��Fģ̬�̶�Ϊ0��������
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

		if(m_channel_mc_status.mc_error.bits.err_soft_limit_neg || m_channel_mc_status.mc_error.bits.err_soft_limit_pos){//����λ�澯
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

		if(m_channel_mc_status.mc_error.bits.err_pos_over){//λ��ָ�����
			if(!this->m_b_mc_on_arm)
				this->m_p_mc_comm->ReadChnPosErrMask(m_n_channel_index, m_channel_mc_status.pos_error_mask);
			else
				this->m_p_mc_arm_comm->ReadChnPosErrMask(m_n_channel_index, m_channel_mc_status.pos_error_mask);

			printf("============ ERR_POS_ERR ===============\n");
			CreateError(ERR_POS_ERR, ERROR_LEVEL, CLEAR_BY_MCP_RESET, m_channel_mc_status.pos_error_mask, m_n_channel_index);
		}

		if(m_channel_mc_status.mc_error.bits.err_arc_data){//Բ�����ݴ���
			CreateError(ERR_ARC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
		}

		if(m_channel_mc_status.mc_error.bits.err_cmd_crc){//ָ��У�����
			CreateError(ERR_CMD_CRC, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
		}

		if(m_channel_mc_status.mc_error.bits.err_data_crc){//���ݰ�У�����
			CreateError(ERR_DATA_CRC, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
		}


		if(this->m_b_manual_tool_measure){ //��ǰ����ִ���ֶ��Ե�����
			if(this->GetCompileState() == IDLE ){
				this->m_b_manual_tool_measure = false;

				//TODO ��HMI���ͶԵ������Ϣ
				this->SendManualToolMeasureResToHmi(!this->m_b_cancel_manual_tool_measure);
				m_b_cancel_manual_tool_measure = false;   //��λ�ź�
			}
		}

		if(this->m_b_manual_call_macro){
            if(this->GetCompileState() == IDLE || this->GetCompileState() == STOP/*��;ֹͣʱ��StateΪStop*/){
				this->m_b_manual_call_macro = false;

				this->m_b_cancel_manual_call_macro = false;
			}
		}

		if(this->m_channel_status.chn_work_mode == AUTO_MODE && this->m_b_hmi_graph &&
				(this->m_channel_status.machining_state == MS_RUNNING || this->m_channel_status.machining_state == MS_MACHIN_SIMULATING)){
			this->ReadGraphAxisPos();
		}

#ifdef USES_WUXI_BLOOD_CHECK
		if(this->m_b_returnning_home){  //���㴦��
			this->ReturnHome();
		}
#endif

#ifdef USES_GRIND_MACHINE   //����ĥ��
		if(this->m_b_ret_safe_state && m_p_f_reg->SA == 1){	//��Ҫ�ȴ����ŷ����
			if(g_ptr_alarm_processor->HasErrorInfo()){//����ȡ���ذ�ȫλ��
				m_b_ret_safe_state = false;
			}else{
				//�ȴ���ʱִ��
				struct timeval time_now;
				unsigned int time_elpase = 0;
				gettimeofday(&time_now, NULL);
				time_elpase = (time_now.tv_sec-m_time_ret_safe_delay.tv_sec)*1000000+time_now.tv_usec-m_time_ret_safe_delay.tv_usec;
				if(time_elpase > 50000){
					this->ReturnSafeState();    //�ذ�ȫ��
				}
			}
		}
#endif

#ifdef USES_LASER_MACHINE
		if(m_b_laser_calibration){
			this->ProcessLaserCalibration();   //ִ�е������궨
			this->ReadHeightRegulatorInput();   //��ȡ�����ź�
		}

#endif

#ifdef USES_WOOD_MACHINE
		//��ȡ��ǰ��������
		if(m_n_ref_tool_life_delay > 0){
			m_n_ref_tool_life_delay--;
		}else if(count%500 == 0){  //ÿ500�����ڸ���һ��
			if(m_channel_status.cur_tool > 0){
				int life = 0;
				this->m_p_mc_comm->ReadChnCurToolLife(this->m_n_channel_index, life);
				if(this->m_channel_status.machining_state != MS_RUNNING &&
						this->m_p_chn_tool_info->tool_life_cur[this->m_channel_status.cur_tool-1] != life){
					m_p_chn_tool_info->tool_life_cur[this->m_channel_status.cur_tool-1] = life;

					//�������ļ�
					g_ptr_parm_manager->UpdateToolPotConfig(this->m_n_channel_index, *this->m_p_chn_tool_info);
					m_b_save_tool_info = false;
				}else if(this->m_p_chn_tool_info->tool_life_cur[this->m_channel_status.cur_tool-1] != life){
					m_p_chn_tool_info->tool_life_cur[this->m_channel_status.cur_tool-1] = life;
					m_b_save_tool_info = true;
				}


				if(m_p_chn_tool_info->tool_life_max[this->m_channel_status.cur_tool-1] > 0 && count%5000 == 0){  //�澯����20s�����һ��
					if(life >= m_p_chn_tool_info->tool_life_max[this->m_channel_status.cur_tool-1]){ //���ߵ���
						CreateError(ERR_TOOL_LIFE_OVER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, this->m_channel_status.cur_tool, m_n_channel_index);
					}else if(life >= m_p_chn_tool_info->tool_life_max[this->m_channel_status.cur_tool-1]*0.95){  //���߽ӽ�����
						CreateError(ERR_TOOL_LIFE_COMING, WARNING_LEVEL, CLEAR_BY_MCP_RESET, this->m_channel_status.cur_tool, m_n_channel_index);
					}
				}
			}
		}
#endif

        if(m_channel_status.cur_tool > 0 && m_channel_status.cur_tool <= kMaxToolCount){
            int cur_tool = m_channel_status.cur_tool - 1;
            if(m_p_chn_tool_info->tool_life_type[cur_tool] == ToolPot_Cnt)
            {// ���μ���
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

		count++;   //������һ
		usleep(4000);
	}

	return true;
}

/**
 * @brief �Ƿ�MC�����е�λ
 * @return true--��  false--��
 */
bool ChannelControl::IsBlockRunOver(){
    struct timeval cur_time;
    gettimeofday(&cur_time, nullptr);
    unsigned int delay = (cur_time.tv_sec-m_time_start_mc.tv_sec)*1000000+cur_time.tv_usec-m_time_start_mc.tv_usec;  //��λ΢��
    if(delay < MC_STATE_REFRESH_CYCLE) //����MC��������5ms�����ȷ�ϵ�λ��־
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
 * @brief ���õ�ǰ�кţ��������õ�ǰ�кŲ���MC��ȡ
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
    ResetMcLineNo();//��λMCģ�鵱ǰ�к�
}


/**
 * @brief ��HMI���ͷ�������
 * @param msg : ��������Ϣ
 * @return
 */
bool ChannelControl::OutputSimulateData(RecordMsg *msg){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "ChannelControl::OutputSimulateData, line = %llu", msg->GetLineNo());
    SimulateData data;
    CompilerSimData comp_data;
    MonitorDataType type = MDT_CHN_SIM_GRAPH;  //��������
    uint8_t chn_axis_x = 0, chn_axis_y = 0, chn_axis_z = 0;  //ͨ�����

    if(msg->IsMoveMsg()){
        if(1 == msg->GetSimulateData(comp_data))
            return true;

        if(m_simulate_mode == SIM_TOOLPATH)
            type = MDT_CHN_SIM_TOOL_PATH;

        chn_axis_x = this->GetChnAxisFromName(AXIS_NAME_X);
        chn_axis_y = this->GetChnAxisFromName(AXIS_NAME_Y);
        chn_axis_z = this->GetChnAxisFromName(AXIS_NAME_Z);
        //����ת��������
        if(comp_data.type == RAPID_TARGET || comp_data.type == LINE_TARGET){  //G00/G01
            data.type = comp_data.type;
            data.pos[0] = comp_data.target[0];
            data.pos[1] = comp_data.target[1];
            data.pos[2] = comp_data.target[2];

            //ת��Ϊ��е����ϵ
            if(chn_axis_x != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[0], this->m_channel_status.gmode[14], chn_axis_x);
            if(chn_axis_y != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[1], this->m_channel_status.gmode[14], chn_axis_y);
            if(chn_axis_z != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[2], this->m_channel_status.gmode[14], chn_axis_z);

            //�������ݸ�HMI
            this->SendSimulateDataToHmi(type, data);
        }else if(comp_data.type == ARC2_TARGET || comp_data.type == ARC3_TARGET){  //G02/G03
            data.type = comp_data.type;
            data.pos[0] = comp_data.target[0];
            data.pos[1] = comp_data.target[1];
            data.pos[2] = comp_data.target[2];   //Ŀ��λ��

            //ת��Ϊ��е����ϵ
            if(chn_axis_x != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[0], this->m_channel_status.gmode[14], chn_axis_x);
            if(chn_axis_y != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[1], this->m_channel_status.gmode[14], chn_axis_y);
            if(chn_axis_z != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[2], this->m_channel_status.gmode[14], chn_axis_z);
            this->SendSimulateDataToHmi(type, data);  //����Ŀ��λ��

            //Բ��
            data.type = ARC_CENTER;
            data.pos[0] = comp_data.center[0];
            data.pos[1] = comp_data.center[1];
            data.pos[2] = comp_data.center[2];
            //ת��Ϊ��е����ϵ
            if(chn_axis_x != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[0], this->m_channel_status.gmode[14], chn_axis_x);
            if(chn_axis_y != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[1], this->m_channel_status.gmode[14], chn_axis_y);
            if(chn_axis_z != 0xFF)
                this->TransWorkCoordToMachCoord(data.pos[2], this->m_channel_status.gmode[14], chn_axis_z);
            this->SendSimulateDataToHmi(type, data);  //����Բ������

            //�뾶
            data.type = ARC_RADIUS;
            data.pos[0] = comp_data.radius;
            data.pos[1] = comp_data.plane;
            data.pos[2] = 0;
            this->SendSimulateDataToHmi(type, data);  //����Բ�뾶���������ӻ�������Ϣ
        }
    }

    return true;
}


/**
 * @brief ��MC���ͱ���G��������
 * @param msg : ��������Ϣ
 * @param flag_block : �Ƿ���Ҫǿ�Ƹ��Ͽ������־
 * @return true--�ɹ�   false--MCģ�黺����
 */
bool ChannelControl::OutputData(RecordMsg *msg, bool flag_block){
    //	if(!this->m_p_mc_comm->CanWriteGCode(m_n_channel_index)){
    //		m_n_send_mc_data_err++;
    //		return false;
    //	}


    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){   //��������͵�·����ʱֱ�ӷ������ݸ�HMI
        return this->OutputSimulateData(msg);
    }

    //MC������˶�����+PL�л�������ݲ��ܴ���MC���˶����ݻ�����������ֹMDA���ݱ�����
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

    GCodeFrame data_frame;   	//G��������֡

    //	bool is_last = false;

    if(msg->GetFrameIndex() == 0){
        msg->SetFrameIndex(m_n_frame_index);
        data_frame.data.frame_index = this->m_n_frame_index++;
        if(m_n_frame_index == 0)
            m_n_frame_index = 1;
    }else{
        data_frame.data.frame_index = msg->GetFrameIndex();
    }

    //����岹����˶�ָ��
    bool flag = this->m_n_hw_trace_state==REVERSE_TRACE?true:false;    //�Ƿ�������
    if(msg->GetOutputData(&data_frame, m_mask_intp_axis, flag) == 1)  //��Ҫ���͵��˶�����
        return true;

    //����ģ̬��Ϣ
    data_frame.data.mode = this->m_mc_mode_exec.all;
    //	if(data_frame.data.mode & 0x03){
    //		printf("Gmode 02 is G18~~~~~~outputdata: 0x%x, 0x%x\n", data_frame.data.mode, m_mc_mode_exec.all);
    //	}

    //����EXT_TYPE�ֶ�
    //���һ�����ݣ��޸�Ext_Type�е�bit0����ʾblock�ѽ���
    if(msg->CheckFlag(FLAG_BLOCK_OVER) || flag_block){
        data_frame.data.ext_type |= 0x0001;
        //printf("line %llu send block over@@@@@@@\n", msg->GetLineNo());
        //	is_last = true;
    }
    //bit2bit1
    switch(msg->GetMsgType()){
    case ARC_MSG:{
        //���õ�ǰƽ��
        uint16_t plane = this->m_mc_mode_exec.bits.mode_g17;   //
        if(plane == 2) //YZƽ�棬G19
            data_frame.data.ext_type |= 0x02;
        else if(plane == 1)  //XZƽ�棬G18
            data_frame.data.ext_type |= 0x04;
        break;
    }
    case LINE_MSG:{
#ifdef USES_WOOD_MACHINE
        if(this->m_p_g_reg->QDE == 1){  //���깦�ܼ���
            LineMsg *linemsg = static_cast<LineMsg *>(msg);
            uint8_t z_chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
            uint32_t msk = 0x01;
            if(linemsg->GetAxisMoveMask() == (msk<<z_chn_axis)){   //ֻ��Z���ƶ�ʱ
                data_frame.data.cmd = MOVE_G00;   //ǿ�ƽ�G01ָ���滻ΪG00ָ��
                data_frame.data.feed = 0;

                if(m_p_channel_config->rapid_mode == 1){
                    data_frame.data.ext_type |= 0x02;	//ֱ�߶�λ
                }
            }else{
                //bit2bit1���ò岹ģʽ��00-XYZ�岹   01-������岹
                if(m_p_channel_config->intep_mode == 1){
                    data_frame.data.ext_type |= 0x02;   //������岹
                }
            }
        }else{
            //bit2bit1���ò岹ģʽ��00-XYZ�岹   01-������岹
            if(m_p_channel_config->intep_mode == 1){
                data_frame.data.ext_type |= 0x02;   //������岹
            }
        }
#else
        //bit2bit1���ò岹ģʽ��00-XYZ�岹   01-������岹
        if(m_p_channel_config->intep_mode == 1){
            data_frame.data.ext_type |= 0x02;   //������岹
        }
#endif
        break;
    }
    case COMPENSATE_MSG:
    case SKIP_MSG:
        //bit2bit1���ò岹ģʽ��00-XYZ�岹   01-������岹
        if(m_p_channel_config->intep_mode == 1){
            data_frame.data.ext_type |= 0x02;   //������岹
        }
        // @add zk  ��ָ�����ͻ�ʹ MC ������
        if(msg->GetMsgType() == COMPENSATE_MSG)
            data_frame.data.cmd = ((CompensateMsg*)msg)->GetMoveType();   // G43����֮ǰģ̬ȷ���ƶ�����
        else
            data_frame.data.cmd = 1;   // G31��Ҫǿ����ΪG01
        break;
    case RAPID_MSG:
    case COORD_MSG:
        if(m_p_channel_config->rapid_mode == 1){
            data_frame.data.ext_type |= 0x02;	//ֱ�߶�λ
        }
        break;
    default:
        break;
    }

    //bit6  ׼ͣ��־��0-��׼ͣ   1--׼ͣ
    if(this->m_p_compiler->IsExactStop()){
        data_frame.data.ext_type |= 0x40; 	//׼ͣ
    }

    //bit7 0--�Զ�����   1--MDA����
    if(m_channel_status.chn_work_mode == MDA_MODE ||
            this->m_b_manual_tool_measure || //�Զ��Ե�
            this->m_b_manual_call_macro   //�ֶ����ú����
        #ifdef USES_ADDITIONAL_PROGRAM
            || (m_n_add_prog_type == CONTINUE_START_ADD)
        #endif
            )   //�ϵ����ǰ�ó������
        data_frame.data.ext_type |= 0x80; 	//MDA����


    //	if(msg->GetMsgType() == ARC_MSG){
    //		printf("output arc msg: cmd = %hu, ext_type=0x%hx, mode=0x%x, line=%u\n", data_frame.data.cmd, data_frame.data.ext_type, data_frame.data.mode,
    //				data_frame.data.line_no);
    //	}


    //����������MC
    bool res = false;
    if(!this->m_b_mc_on_arm){
        // �鿴 msg�� ext type mc����ext��ֵ�Բ��϶���ִ��
    	printf("ext_type: %d\n", data_frame.data.ext_type);
    	res = m_p_mc_comm->WriteGCodeData(m_n_channel_index, data_frame);
    }else{

    	res = this->m_p_mc_arm_comm->WriteGCodeData(m_n_channel_index, data_frame);
    }
    if(!res){
        //����ʧ��
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

    //������ѹ�뷢�ͻ�����
    //	if(!AddOutputData(data_frame)){
    //		printf("OutputData false\n");
    //		return false;
    //	}

    //	printf("send out data frame: cmd = %d, line = %d\n", data_frame.data.cmd, data_frame.data.line_no);

    return true;
}

/**
 * @brief ���˶���������֡ѹ�뻺��
 * @param data : �����͸�MCģ�������֡
 * @return  true--�ɹ�    false--ʧ�ܣ���������
 */
//bool ChannelControl::AddOutputData(const GCodeFrame &data){
//	int cur_count = this->m_p_output_buffer->BufLen();
//
//	//�Ȱ�֮ǰ�����ݷ��ͳ�ȥ
//	GCodeFrame *send_data = nullptr;
//	for(int i = 0; i < cur_count; i++){
//		send_data = this->m_p_output_buffer->ReadDataPtr(0);
//		if(!m_p_mc_comm->WriteGCodeData(*send_data)){
//			//����ʧ��
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
 * @brief ���˶����ݻ����е����ݶ��������λ�������־
 * @return true--�ɹ�    false--ʧ�ܣ���������
 */
//bool ChannelControl::OutputLastBlockItem(){
//	int cur_count = this->m_p_output_buffer->BufLen();
//
//	GCodeFrame *send_data = nullptr;
//	while(cur_count > 0){
//		send_data = this->m_p_output_buffer->ReadDataPtr(0);
//		if(cur_count == 1){
//			//���һ�����ݣ��޸�Ext_Type�е�bit0����ʾblock�ѽ���
//			send_data->data.ext_type |= 0x0001;
//
//			m_b_mc_need_start = true;
//		}
//		if(!m_p_mc_comm->WriteGCodeData(*send_data)){
//			//����ʧ��
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
 * @brief ���˶����ݻ����е����ݶ����,����λ�������־
 * @return true--�ɹ�    false--ʧ�ܣ���������
 */
//bool ChannelControl::OutputAllData(){
//	int cur_count = this->m_p_output_buffer->BufLen();
//
//	GCodeFrame *send_data = nullptr;
//	while(cur_count > 0){
//		send_data = this->m_p_output_buffer->ReadDataPtr(0);
//
//		if(!m_p_mc_comm->WriteGCodeData(*send_data)){
//			//����ʧ��
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
 * @brief ��DPoint����任ΪDPointChn����
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
 * @brief ��DPointChn����任ΪDPoint����
 * @param src
 * @param tar
 */
void ChannelControl::DPointChn2DPoint(const DPointChn &src, DPoint &tar){

}

/**
 * @brief ͬ���ѱ�������ƶ�ָ���λ��
 * @param pos : ��λ��
 * @return : true--��ʾ�����ƶ�ָ��   false--��ʾû�����ƶ�ָ��
 */
bool ChannelControl::RefreshOuputMovePos(DPointChn &pos){
    RecordMsg *msg = nullptr;
    ListNode<RecordMsg *> *node = m_p_output_msg_list->HeadNode();

    if(this->m_channel_status.chn_work_mode == AUTO_MODE && this->m_p_last_output_msg != nullptr){//֧�����ַ�������������������ƶ�ָ������̳�����
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
        node = node->next;  //ȡ��һ����Ϣ
    }

    //ͬ���������ֿ�����е��ƶ�ָ��
    res |= this->m_p_compiler->RefreshBlockMovePos(pp);

    return res;
}

/**
 * @brief ͬ���Ƿ������ƶ�ָ��
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

        node = node->next;  //ȡ��һ����Ϣ
    }
    return res;
}

/**
 * @brief NC�ļ��Ƿ����
 * @param file_name : NC�ӹ��ļ���
 * @return  true--����   false--������
 */
bool ChannelControl::IsNcFileExist(char *file_name){
    char path[kMaxPathLen] = {0};
    strcpy(path, PATH_NC_FILE);

    strcat(path, file_name);

    if(access(path, F_OK) == -1){	//�����ڣ�����ʧ��
        return false;
    }
    return true;
}

/**
 * @brief �Ƿ��������У������Զ����У��ӹ����棬��·����
 * @return true--������   false--������
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
 * @brief �Ƿ񵥶�ģʽ,ֻ�����Զ�ģʽ�£����β���Ч
 * @return  true--��   false--����
 */
bool ChannelControl::IsStepMode(){
    if(m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE) &&     //������Ч
            (m_channel_status.chn_work_mode == AUTO_MODE) &&               //�Զ�ģʽ
            (m_simulate_mode == SIM_NONE)    //�Ƿ���״̬
        #ifdef USES_ADDITIONAL_PROGRAM
            && (m_n_add_prog_type == NONE_ADD)    //�Ǹ��ӳ�������״̬
        #endif
            ){
        if(this->m_n_macroprog_count == 0 || this->m_p_general_config->debug_mode > 0) //�Ǻ������û��ߵ���ģʽ��
            return true;
    }

    return false;
}

/**
 * @brief ������Ϣִ��ģ��
 * @return true--�ɹ�  false--ʧ��
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
    int end_cmd = 0;    //0--��ʾ�ǽ���ָ��   30--��ʾM30    99--��ʾM99
    bool pause_flag = false;


    if(this->m_channel_status.chn_work_mode == AUTO_MODE &&
            this->m_simulate_mode == SIM_NONE     //�Ƿ���״̬
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type != CONTINUE_START_ADD
        #endif
            ){

        if(this->m_n_restart_mode != NOT_RESTART){//�ӹ���λ���̣�ǰ��ɨ������ݲ�����
            node = m_p_output_msg_list->HeadNode();

        }else{
            if(this->m_p_last_output_msg == nullptr){
                if(this->m_n_hw_trace_state == REVERSE_TRACE){
                    node = m_p_output_msg_list->TailNode();    //֮ǰΪ�ն������β����ʼ����
                    //	printf("reverse from tail#######\n");
                }else{
                    node = m_p_output_msg_list->HeadNode();    //֮ǰΪ�ն������ͷ����ʼ����
                    //	printf("normal from head@@@@@\n");
                }

            }else{//�������һ��msg��ʼ����
                if(this->m_n_hw_trace_state == REVERSE_TRACE){
                    node = m_p_last_output_msg->pre;
                }else{
                    node = m_p_last_output_msg->next;
                }
            }

            //������ַ�����ٵ�ͷ���������ʾ
            if(this->m_n_hw_trace_state == REVERSE_TRACE && node == nullptr){
                int count = 0;
                if(!this->m_b_mc_on_arm){
                    //���µ�ǰMC������ģʽ
                    this->m_p_mc_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);
                    //����MC��ǰ����������
                    this->m_p_mc_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);
                    while(m_channel_mc_status.cur_mode == MC_MODE_MANUAL && this->m_channel_mc_status.buf_data_count == 0 && count < 3){

                        usleep(2000);  //����2ms

                        //���µ�ǰMC������ģʽ
                        this->m_p_mc_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);
                        //����MC��ǰ����������
                        this->m_p_mc_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);

                        count++;
                    }
                }else{
                    //���µ�ǰMC������ģʽ
                    this->m_p_mc_arm_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);
                    //����MC��ǰ����������
                    this->m_p_mc_arm_comm->ReadAutoBufDataCount(m_n_channel_index, m_channel_mc_status.buf_data_count);
                    while(m_channel_mc_status.cur_mode == MC_MODE_MANUAL && this->m_channel_mc_status.buf_data_count == 0 && count < 3){

                        usleep(2000);  //����2ms

                        //���µ�ǰMC������ģʽ
                        this->m_p_mc_arm_comm->ReadWorkMode(m_n_channel_index, m_channel_mc_status.cur_mode);
                        //����MC��ǰ����������
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
            this->PreExecSpindleCmd(this->m_channel_rt_status.line_no);   //ִ������Ԥ����
#endif

    }else{
        node = m_p_output_msg_list->HeadNode();
    }

    while(node != nullptr){
        msg = static_cast<RecordMsg *>(node->data);
        end_cmd = 0;
        if(m_channel_status.chn_work_mode == AUTO_MODE &&
                m_n_restart_mode != NOT_RESTART && this->m_n_restart_step == 1 &&
                this->m_mode_restart.sub_prog_call == 0 &&     //����������
                msg->GetLineNo() >= this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
                && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
                ){  //���е���λָ���У���Ҫ���ؽ�ģ̬
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
            if(m_n_run_thread_state != WAIT_RUN  && m_n_run_thread_state != WAIT_EXECUTE){ //��ǰ��WAIT_RUN״̬�����߳���ΪWAIT_EXECUTE״̬
                this->m_n_run_thread_state = WAIT_EXECUTE;
                //		printf("set WAIT_EXECUTE, line = %llu\n", msg->GetLineNo());
            }

            break;
        }else{//ִ�н���
            if(this->m_n_restart_mode != NOT_RESTART        //�ӹ���λ
        #ifdef USES_ADDITIONAL_PROGRAM
                    && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
                    ){

                if(this->m_n_restart_step == 1){  //ɨ��׶ε�����ִ�����ֱ��ɾ��
                    this->m_p_output_msg_list->Delete(node);   //ɾ���˽ڵ�����
                    node = m_p_output_msg_list->HeadNode();  //ȡ��һ����Ϣ
                    return res;
                }
                //				else if(this->m_n_restart_step == 2){//FOR TEST
                //					g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "Restart to exec msg %d, line = %llu,  res = %d\n", msg->GetMsgType(), msg->GetLineNo(), res);
                //				}
            }

            if(msg->IsNeedWaitMsg() && (this->m_simulate_mode == SIM_NONE || this->m_simulate_mode == SIM_MACHINING)){  //�Ƿ��桢�ӹ��������Ҫ��MC��������ָ��
                m_b_mc_need_start = true;
                //				printf("set mc need start true in exec~~~~~\n");
            }


            if((this->m_channel_status.machining_state == MS_RUNNING || this->m_b_manual_tool_measure
                || this->m_b_manual_call_macro
    #ifdef USES_ADDITIONAL_PROGRAM
                || (m_n_add_prog_type == CONTINUE_START_ADD)
    #endif
                ) &&    //����״̬�����ֶ��Ե�״̬
                    msg->IsMoveMsg() && this->m_b_mc_need_start){
                //				printf("move data send start, msg type = %d\n", msg->GetMsgType());
                this->StartMcIntepolate();
            }
            //			else {
            //				printf("ismovemsg:%hhu, m_b_mc_need_start=%hhu\n", msg->IsMoveMsg(), m_b_mc_need_start);
            //			}


            if(IsStepMode()){//����ģʽ, �Ǻ������û��ߴ��˵���ģʽ
                if(msg->CheckFlag(FLAG_LAST_REC)){
                    if((!msg->IsMoveMsg() || msg_type == AUX_MSG || msg_type == REF_RETURN_MSG) &&
                            !msg->IsEndMsg()){	//����ģʽ�£�������һ�е����һ���������ͣ
                        printf("PAUSED in execute\n");
                        m_n_run_thread_state = PAUSE;
                        pause_flag = true;
                    }else if(msg->IsMoveMsg()){
                        this->m_b_need_change_to_pause = true;   //mc������ɺ��л���ͣ״̬
                    }
                }
                //				if(msg->GetFlag(FLAG_LAST_REC))
                //					m_b_step_exec = false;
            }

            if(msg->IsEndMsg()){	//����M02/M30��Ϣ
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
            //��λ���ӳ���ִ��״̬
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
                this->m_n_add_prog_type = NONE_ADD;  //����ǰ�ó������

                if(this->IsStepMode())
                    this->SetMcStepMode(true);
                //				printf("########add exectue over2222\n");
            }
#endif

            if(m_channel_status.chn_work_mode == AUTO_MODE &&
                    m_simulate_mode == SIM_NONE      //�Ƿ���״̬
        #ifdef USES_ADDITIONAL_PROGRAM
                    && m_n_add_prog_type == NONE_ADD
        #endif
                    ){
                //				if(this->m_n_restart_mode != NOT_RESTART){//��λ״̬�����е����ݶ�ɾ��
                //					this->m_p_output_msg_list->Delete(node);   //ɾ���˽ڵ�����
                //					node = m_p_output_msg_list->HeadNode();  //ȡ��һ����Ϣ
                //				}else{
                //����msg�����ж��Ƿ���Ҫ���֮ǰ�Ļ���
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
                            this->m_p_output_msg_list->Delete(node_tmp);   //ɾ��FEED�����
                            //			printf("delete feed msg##@@@\n");
                        }

                    }
                }
                //				}

            }else{
                //		printf("delete node msg, type = %d, line=%llu\n", msg->GetMsgType(), msg->GetLineNo());
                this->m_p_output_msg_list->Delete(node);   //ɾ���˽ڵ�����
                node = m_p_output_msg_list->HeadNode();  //ȡ��һ����Ϣ
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
 * @brief ʵ��ִ�и���ָ����Ϣ
 * @param msg : ָ����Ϣ
 * @return true--�ɹ�  false--ʧ�ܣ�PL�л����������ߵȴ����е�λ
 */
bool ChannelControl::ExecuteAuxMsg(RecordMsg *msg){

	AuxMsg *tmp = (AuxMsg *)msg;
    uint8_t m_count = tmp->GetMCount();   //һ����M��������
    uint8_t m_index = 0;
    int mcode = 0;

    if(this->m_n_restart_mode != NOT_RESTART &&
            tmp->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        for(m_index = 0; m_index < m_count; m_index++){
            mcode = tmp->GetMCode(m_index);

            if(mcode == 6){//����
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
            }else if(mcode == 2 || mcode == 30 || mcode == 99){  //ֱ�ӽ������л���READY״̬
                if(mcode == 99 && m_mode_restart.sub_prog_call > 0)
                    this->m_mode_restart.sub_prog_call--;
                else{
                    ResetMcLineNo();//��λMCģ�鵱ǰ�к�
                    this->SetCurLineNo(1);

                    CompileOver();
                    this->m_n_restart_mode = NOT_RESTART;
                    this->m_n_restart_line = 0;
                    this->m_n_restart_step = 0;
                    this->m_p_compiler->SetRestartPara(0, m_n_restart_mode);   //��λ�������ļӹ���λ��־

                    this->ClearHmiInfoMsg();   //���HMI����ʾ��Ϣ
                }
            }
        }

        return true;
    }

    if(this->m_simulate_mode != SIM_NONE){  //����ģʽ
        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());

        mcode = tmp->GetMCode(m_index);

        printf("mcode : %d\n", mcode);

        if(mcode == 2 || mcode == 30){
            ResetMcLineNo();//��λMCģ�鵱ǰ�к�
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
                ResetMcLineNo();//��λMCģ�鵱ǰ�к�
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
        //���Ƚ������е����д�����ָ��͸�MC
        //		if(!OutputLastBlockItem()){
        //			//PL�е�FIFO����������ʧ��
        //			return false;
        //		}

        int limit = 3;
        if(this->IsStepMode())
            limit = 5;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

        //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
        int count = 0;
        bool block_over = false;
        while(1){
            block_over = CheckBlockOverFlag();
            if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
                //printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
                return false;    //��δ���е�λ
            }
            else if(++count < limit){
                usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
                //printf("execute aus msg: blockflag=%d, count = %d\n",  block_over, count);

            }else
                break;
        }

        if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }


        //���õ�ǰ�к�
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
            continue;       //��ִ�������ֱ������

        mcode = tmp->GetMCode(m_index);

        if(mcode != 300) // M300����Ҫ��ʾ
            NotifyHmiMCode(mcode);

        switch(mcode){
        case 30:  	//M30
        case 2:		//M02
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M30\n");

                //TODO �����뷢�͸�PMC
                this->SendMCodeToPmc(mcode, m_index);
                if(mcode == 30)
                    this->m_p_f_reg->DM30 = 1;  //��λDM30
                else
                    this->m_p_f_reg->DM02 = 1;  //��λDM02

                gettimeofday(&m_time_m_start[m_index], NULL);   //


                //���������Ҫ��������ת��������Ȧ
                //#ifdef USES_FIVE_AXIS_FUNC
                //			if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS){
                //				//��MI����������Ȧλ��ָ��
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
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index, true);    //��λѡͨ�ź�

                gettimeofday(&m_time_m_start[m_index], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    gettimeofday(&m_time_m_start[m_index], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                //#ifdef USES_FIVE_AXIS_FUNC
                //			//�ȴ�MI�������
                //			if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS &&
                //					m_mask_5axis_rot_nolimit != 0){
                //				if(m_mask_5axis_rot_nolimit != m_n_mask_clear_pos){
                //					break;
                //				}
                //			}
                //#endif

                this->SetMFSig(m_index, false);    //��λѡͨ�ź�

                if(mcode == 30)
                    this->m_p_f_reg->DM30 = 0;  //��λDM30
                else
                    this->m_p_f_reg->DM02 = 0;  //��λDM02

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index);

#ifdef USES_ADDITIONAL_PROGRAM
                if(this->m_n_add_prog_type == NONE_ADD){
#endif
                    if(mcode == 30){
                        ResetMcLineNo();//��λMCģ�鵱ǰ�к�
                        this->SetCurLineNo(1);
                    }

                    this->m_p_f_reg->STL = 0;
                    this->m_p_f_reg->SPL = 0;
                    this->m_p_f_reg->OP = 0;

                    //����������һ
                    if(this->m_channel_status.chn_work_mode == AUTO_MODE){
//                        this->m_channel_status.workpiece_count++;
//                        g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
//                        this->m_channel_status.workpiece_count_total++;
//                        g_ptr_parm_manager->SetTotalWorkPiece(m_n_channel_index, m_channel_status.workpiece_count_total);
//                        this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);  //֪ͨHMI���¼ӹ�����

//                        if (m_channel_status.workpiece_require != 0 && m_channel_status.workpiece_count >= m_channel_status.workpiece_require)
//                        {//�ѵ����������
//                            CreateError(ERR_REACH_WORK_PIECE, INFO_LEVEL, CLEAR_BY_MCP_RESET);
//                        }
                        this->ResetMode();   //ģ̬�ָ�Ĭ��ֵ

                        //���������ϵ����
                        if(g_ptr_parm_manager->ActiveCoordParam(m_n_channel_index))
                            this->SetMcCoord(true);

                        //�������Ч�ĵ���ƫ������
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
                this->SetMiSimMode(false);  //��λMI����״̬

                //#ifdef USES_FIVE_AXIS_FUNC
                //
                //			if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS &&
                //					m_mask_5axis_rot_nolimit != 0){
                //				//��MCͬ����������
                //				this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
                //
                //				m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
                //				m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
                //
                //				this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
                //
                //		//		printf("intp pos (%lf, %lf, %lf, %lf)\n", m_channel_mc_status.intp_pos.x, m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z,
                //		//				m_channel_mc_status.intp_pos.a6);
                //
                //				//��MIͬ����е����
                //				this->m_p_channel_engine->SendMonitorData(false, false);
                //			}
                //#endif
                if(m_channel_status.chn_work_mode == AUTO_MODE){
#ifdef USES_ADDITIONAL_PROGRAM
                    if(this->m_n_add_prog_type == NONE_ADD){
                        this->SendMachOverToHmi();  //���ͼӹ�������Ϣ��HMI
                    }
#else
                    string msg = "�����ӹ�����(" + string(this->m_channel_status.cur_nc_file_name) + ")";
                    g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug, msg);

                    this->SendMachOverToHmi();  //���ͼӹ�������Ϣ��HMI
#endif
                }
                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬

                printf("execute M30 over\n");
            }

            break;
        case 0:		//M00��������ͣ
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M00\n");
                //TODO �����뷢�͸�PMC
                this->SendMCodeToPmc(mcode, m_index);
                this->m_p_f_reg->DM00 = 1;  //��λDM00

                gettimeofday(&m_time_m_start[m_index], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                gettimeofday(&m_time_m_start[m_index], NULL);
                this->SetMFSig(m_index, true);    //��λѡͨ�ź�
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    gettimeofday(&m_time_m_start[m_index], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index, false);    //��λѡͨ�ź�

                this->m_p_f_reg->DM00 = 0;  //��λDM00

                this->m_p_f_reg->STL = 0;
                this->m_p_f_reg->SPL = 1;

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index);


                this->PauseRunGCode();
                m_n_run_thread_state = PAUSE;

                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            }
            break;
        case 1:		//M01ѡ������ͣ
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M01\n");
                //TODO �����뷢�͸�PMC
                this->SendMCodeToPmc(mcode, m_index);
                this->m_p_f_reg->DM01 = 1;  //��λDM01

                gettimeofday(&m_time_m_start[m_index], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index, true);    //��λѡͨ�ź�

                gettimeofday(&m_time_m_start[m_index], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    gettimeofday(&m_time_m_start[m_index], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index, false);    //��λѡͨ�ź�

                this->m_p_f_reg->DM01 = 0;  //��λDM01

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index);


                if(this->m_channel_status.func_state_flags.CheckMask(FS_OPTIONAL_STOP)){//ѡͣ״̬
                    PauseRunGCode();

                    m_n_run_thread_state = PAUSE;
                    this->m_p_f_reg->STL = 0;
                    this->m_p_f_reg->SPL = 1;
                }

                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            }
            break;
        case 6:		//M06����
            if(tmp->GetExecStep(m_index) == 0){
                //TODO �����뷢�͸�PMC
                printf("start to execute M06\n");
                this->SendMCodeToPmc(mcode, m_index);

                gettimeofday(&m_time_m_start[m_index], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                gettimeofday(&m_time_m_start[m_index], NULL);
                this->SetMFSig(m_index, true);    //��λѡͨ�ź�
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    gettimeofday(&m_time_m_start[m_index], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index, false);    //��λѡͨ�ź�
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index);

#ifdef USES_WOOD_MACHINE
                this->m_n_ref_tool_life_delay = 3;
#endif

                //���µ�ǰ����
                //				this->m_channel_status.cur_tool = this->m_n_cur_tcode;
                //				g_ptr_parm_manager->SetCurTool(m_n_channel_index, m_channel_status.cur_tool);

#ifdef USES_WOOD_MACHINE
                //				this->SetMcToolLife();
#endif

                //				this->SendModeChangToHmi(T_MODE);


                printf("execute M06 over: cur_T = %hhu\n", m_channel_status.cur_tool);

                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            }

            break;
        case 36:    //�������
        {
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M36, test\n");
                this->SendMCodeToPmc(mcode, m_index);
                gettimeofday(&m_time_m_start[m_index], NULL);

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index, true);    //��λѡͨ�ź�
                gettimeofday(&m_time_m_start[m_index], NULL);

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    gettimeofday(&m_time_m_start[m_index], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index)
                            && this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index)/*�ٴ��ж�FIN����ʱ���ڴ����ݿ����Ѿ�ˢ��*/){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��
                this->SetMFSig(m_index, false);    //��λѡͨ�ź�

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index);
                int cur_tool = this->m_channel_status.cur_tool - 1;
                if (cur_tool >= 0 && cur_tool < kMaxToolCount)   //cur_tool=0������
                {
                    std::cout << m_p_chn_tool_info->tool_life_max[cur_tool] << " " << m_p_chn_tool_info->tool_life_type[cur_tool] << std::endl;
                    if (m_p_chn_tool_info->tool_life_type[cur_tool] == ToolPot_Cnt)
                    {//�����������ƴη�ʽ
                        m_p_chn_tool_info->tool_life_cur[cur_tool]++;
                        NotifyHmiToolPotChanged();
                        g_ptr_parm_manager->UpdateToolPotConfig(m_n_channel_index, *m_p_chn_tool_info);
                        std::cout << "cur_tool: " << cur_tool << " cur_life: " << m_p_chn_tool_info->tool_life_cur[cur_tool]
                                     << "max_life: " << m_p_chn_tool_info->tool_life_max[cur_tool] << " cur threshold: " << m_p_chn_tool_info->tool_threshold[cur_tool] << std::endl;
                    }
                }
                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            }
        }
            break;
#ifdef USES_GRIND_MACHINE
        case 10: //������ĥ
            this->EnableGrindShock(true);
            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            break;
        case 11://�ر���ĥ
            this->EnableGrindShock(false);
            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            break;
        case 60:  //�����н�
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M60\n");
                this->m_p_f_reg->work_hold = 1;

                gettimeofday(&m_time_m_start[m_index], nullptr);  //������ʱ

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) ==1){
                if(this->m_p_g_reg->work_down_check == 0){
                    struct timeval time_now;
                    int time_elpase = 0;
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > this->m_p_mech_arm_param->wait_overtime*1000){  //��ʱ���澯
                        this->m_error_code = ERR_WORK_CYLINDER_DOWN;
                        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                    }
                    break;
                }
                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
                printf("execute M60 over\n");
            }
            break;
        case 61:  //�����ɿ�
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M61\n");
                this->m_p_f_reg->work_hold = 0;

                gettimeofday(&m_time_m_start[m_index], nullptr);  //������ʱ

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) ==1){
                if(this->m_p_g_reg->work_up_check == 0){
                    struct timeval time_now;
                    int time_elpase = 0;
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > this->m_p_mech_arm_param->wait_overtime*1000){  //��ʱ���澯
                        this->m_error_code = ERR_WORK_CYLINDER_UP;
                        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                    }
                    break;
                }
                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
                printf("execute M61 over\n");
            }
            break;
        case 62:  //��λ����տ�
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M62\n");
                this->m_p_f_reg->work_vac = 1;

                gettimeofday(&m_timm_time_m_start[m_index], nullptr);  //������ʱ

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) ==1){
                if(this->m_p_g_reg->work_vac_check == 0){
                    struct timeval time_now;
                    int time_elpase = 0;
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > this->m_p_mech_arm_param->wait_overtime*1000){  //��ʱ���澯
                        this->m_error_code = ERR_WORK_VAC_OPEN;
                        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                    }
                    break;
                }
                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
                printf("execute M62 over \n");
            }
            break;
        case 63:  //��λ����չ�
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M63\n");
                this->m_p_f_reg->work_vac = 0;

                gettimeofday(&m_time_m_start[m_index], nullptr);  //������ʱ

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) ==1){
                if(this->m_p_g_reg->work_vac_check == 1){
                    struct timeval time_now;
                    int time_elpase = 0;
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase > this->m_p_mech_arm_param->wait_overtime*1000){  //��ʱ���澯
                        this->m_error_code = ERR_WORK_VAC_CLOSE;
                        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                    }
                    break;
                }
                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
                printf("execute M63 over \n");
            }
            break;
        case 66:  //��������������
            if(this->m_p_g_reg->main_chn == GRIND_MAIN_CHN){
                this->ProcessGrindM66(tmp);
            }
            else{
                this->ProcessGrindM66_slave(tmp);
            }
            break;
            //	case 68:  //�ӻ�����������
            //		this->ProcessGrindM68(tmp);
            //		break;
#endif
        case 98:	//M98�ӳ������
            printf("execute M98\n");
            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            break;
        case 99:   //M99
        	if(tmp->GetExecStep(m_index) == 0){
                printf("execute M99, m_n_subprog_count = %hhu call times=%d\n", m_n_subprog_count, m_p_compiler->getSubCallTimes());
                if(m_n_subprog_count > 0){
                	if(m_p_compiler->needJumpUpper()) m_n_subprog_count--;
                    m_b_ret_from_macroprog = false;
                }
                else{
                	m_p_compiler->RecycleCompile();   //��������ѭ������
//                    this->m_channel_status.workpiece_count++;  //����������һ
//                    g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
//                    this->m_channel_status.workpiece_count_total++;
//                    g_ptr_parm_manager->SetTotalWorkPiece(m_n_channel_index, m_channel_status.workpiece_count_total);
//                    this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);  //֪ͨHMI���¼ӹ�����

//                    if (m_channel_status.workpiece_require != 0 && m_channel_status.workpiece_count >= m_channel_status.workpiece_require)
//                    {//�ѵ����������
//                        CreateError(ERR_REACH_WORK_PIECE, INFO_LEVEL, CLEAR_BY_MCP_RESET);
//                    }

#ifdef USES_GRIND_MACHINE
                    if(this->m_channel_status.workpiece_count >= this->m_p_mech_arm_param->grind_wheel_life){
                        //ɰ�ֵ���
                        this->m_error_code = ERR_GRIND_WHEEL_CHANGE;
                        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                    }
#endif
                }


                //TODO �����뷢�͸�PMC
                this->m_p_f_reg->DM99 = 1;  //��λDM99, ά��20ms

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                tmp->IncreaseExecStep(m_index);

            }else if(tmp->GetExecStep(m_index) == 2){
                this->m_n_run_thread_state = RUN;

                this->m_p_f_reg->DM99 = 0;
                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬

                printf("execute M99 over\n");
            }

            break;
        case 300:   //lidianqiang:MDA�Զ�����M30��ʱ��ΪM300
            //NotifyHmiMCode(0);
            //this->SendMCodeToPmc(0, m_index);
            ResetMcLineNo();//��λMCģ�鵱ǰ�к�
            this->SetCurLineNo(1);

            this->m_p_f_reg->STL = 0;
            this->m_p_f_reg->SPL = 0;
            this->m_p_f_reg->OP = 0;

            CompileOver();
            this->SetMiSimMode(false);  //��λMI����״̬
            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
        break;
        case 998:	//M998 ������ͣ����
            printf("execute M998\n");
            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            break;
        case 999:  //M999 ��ͣ���룬����ͬ��λ��
            printf("execute M999\n");
            //ͬ��λ��
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
                //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //ͬ���ѱ�������ƶ�ָ���λ��

            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            break;
#ifdef USES_TWINING_FUNC
        case 560:  //������ƹ���
            ActiveTwiningFunc(mcode);

            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            printf("active twining function\n");
            break;
        case 561:  //�رղ��ƹ���
            ActiveTwiningFunc(mcode);

            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
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

            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            break;
#endif

        default:   //����M����
            if(mcode >= 4000 && mcode <= 4099){   // MI��ʱ����  M����
                this->MiDebugFunc(mcode);
                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
                break;
            }
            else if(mcode >= 40010 && mcode <= 40083){  // ���ٶȿ���  ���ؿ���  M����
#ifdef USES_SPEED_TORQUE_CTRL
                ProcessCtrlModeSwitch(tmp, m_index);
                break;
#endif
            }
            else if(mcode >= 41011 && mcode <= 41648){  // ������ӳ��  M����
                ProcessAxisMapSwitch(tmp, m_index);
                break;
            }


            if(tmp->GetExecStep(m_index) == 0){
                //TODO �����뷢�͸�PMC
                g_ptr_trace->PrintLog(LOG_ALARM, "default:ִ�е�M���룺M%02d", mcode);
                this->SendMCodeToPmc(mcode, m_index);

                gettimeofday(&m_time_m_start[m_index], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){

            	// ���᲻���ڻ�������Ϊ�����ᣬ��ִ�����Ḩ������
                if((mcode == 3 || mcode == 4 || mcode == 5
                    || mcode == 19 || mcode == 20 || mcode == 26
                    || mcode == 27 || mcode == 28 || mcode == 29)
                        && m_p_spindle->Type() != 2){
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                    if(time_elpase < 100000){
                        this->SetMFSig(m_index, true);    //��λѡͨ�ź�
                    }else{
                        this->SetMFSig(m_index, false);    //��λѡͨ�ź�
                        tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
                    }
                    break;
                }

                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                gettimeofday(&m_time_m_start[m_index], NULL);
                this->SetMFSig(m_index, true);    //��λѡͨ�ź�

                //llx test
                gettimeofday(&m_time_test, NULL);

                tmp->IncreaseExecStep(m_index);

                // �����ǰ����ת״̬���ٸ�M03����ôProcessPMCSignal��ɨ�費���仯��
                // ������Ҫ�����⴦��
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
            	//�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index)) {
                    //gettimeofday(&m_time_m_start[m_index], NULL);   //��ʼ��ʱ

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
                        && this->m_p_g_reg->FIN == 0 && !this->GetMFINSig(m_index)/*�ٴ��ж�FIN����ʱ���ڴ����ݿ����Ѿ�ˢ��*/){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱

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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index].tv_sec)*1000000+time_now.tv_usec-m_time_m_start[m_index].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index, false);    //��λѡͨ�ź�
                tmp->IncreaseExecStep(m_index);

                // �յ�FIN�������֪ͨ�źţ�ȷ����ͼ�Ѿ���ȡ
                if((mcode == 3 || mcode == 4) && m_p_f_reg->SAR == 1){ // �ٶȵ���
                    m_p_f_reg->SAR = 0;
                }else if(mcode == 5 && m_p_f_reg->SST == 1){ // ����
                    m_p_f_reg->SST = 0;
                }else if(mcode == 19 && m_p_f_reg->ORAR == 1){ // ��λ����
                    //printf("=========================ORAR = 0\n");
                	m_p_f_reg->ORAR = 0;
                }
            }else if(tmp->GetExecStep(m_index) == 4){
            	//�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index);

                tmp->IncreaseExecStep(m_index);
            }else{
            	//this->ExecMCode(tmp, m_index);  //ִ��ĳЩM������Ҫϵͳִ�еĶ���
                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            }
            break;
        }
        if(tmp->GetExecStep(m_index) != 0xFF)//��δִ�н���������false
            bRet = false;
        else{
            g_ptr_trace->PrintLog(LOG_ALARM, "ִ�е�M���������M%02d", mcode);
        }
    }

    if(bRet){
        if(this->m_p_f_reg->DEN == 1)
            this->m_p_f_reg->DEN = 0;  //��λDEN�ź�
    }

    return bRet;
}

/**
 * @brief ִ��ĳЩM������Ҫϵͳִ�еĶ���
 * @param msg : Mָ��
 * @param index :
 */
void ChannelControl::ExecMCode(AuxMsg *msg, uint8_t index){
    //    if(msg == nullptr)
    //        return;
    //    printf("M29 ...................\n");
    //    int mcode = msg->GetMCode(index);

    //    if(mcode == 28){    // M28�л����ٶ�ģʽ
    //        m_p_spindle->SetMode(Speed);
    //        msg->SetExecStep(index, 0xFF);
    //    }else if(mcode == 29){  // M29�л���λ��ģʽ
    //        m_p_spindle->SetMode(Position);
    //        msg->SetExecStep(index, 0xFF);
    //    }else{
    //        msg->SetExecStep(index, 0xFF);  //Mָ��ִ�н���
    //    }
    //    if(mcode == 28 || mcode == 29){  //����CSģʽ�л�

    //        if(this->m_n_spindle_count == 0){  //û������
    //            msg->SetExecStep(index, 0xFF);  //Mָ��ִ�н���
    //            return;
    //        }
    //        this->ProcessSpdModeSwitch(msg, index);
    //        this->m_n_M29_flag = mcode-28;

    //    }else{
    //        msg->SetExecStep(index, 0xFF);  //Mָ��ִ�н���
    //    }
}

/**
 * @brief ʵ��ִ��ֱ������ָ����Ϣ
 * @param msg : ָ����Ϣ
 * @param flag_block : �Ƿ���Ҫǿ�Ƹ��Ͽ������־
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteLineMsg(RecordMsg *msg, bool flag_block){
    LineMsg *linemsg = (LineMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            linemsg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        this->m_mode_restart.pos_target = linemsg->GetTargetPos();  //����Ŀ��λ��
        this->m_mode_restart.gmode[1] = linemsg->GetGCode();   //����ģ̬

        return true;
    }


    if(m_p_spindle->isTapEnable()){
        uint8_t z_axis =  this->GetPhyAxisFromName(AXIS_NAME_Z);

        if(fabs(this->m_channel_rt_status.cur_pos_work.GetAxisValue(z_axis)
                - linemsg->GetTargetPos().GetAxisValue(z_axis)) < 0.005){
            m_error_code = ERR_SPD_TAP_POS_ERROR;
            CreateError(ERR_SPD_TAP_POS_ERROR, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            printf("��˿״̬�£� Z��������յ���ͬ \n");
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

    if(msg->IsNeedWaitMsg() && (m_simulate_mode == SIM_NONE || m_simulate_mode == SIM_MACHINING)){//��Ҫ�ȴ�������

        if(linemsg->GetExecStep() == 0){ //ֻ�е�һ����ʼִ��ʱ��Ҫ�ȴ�
            int limit = 2;
            if(this->IsStepMode())
                limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

            //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
            int count = 0;
            while(1){
                bool block_over = CheckBlockOverFlag();
                if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                        m_channel_status.machining_state == MS_PAUSED ||
                        m_channel_status.machining_state == MS_WARNING ||
                        this->m_mask_run_pmc != 0){ //δ�ﵽִ������
                    //			printf("compensate exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
                    return false;    //��δ���е�λ
                }
                else if(++count < limit){
                    usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
                    //		printf("execute compensate msg[%d]: blockflag=%d, count = %d, step = %hhu\n", compmsg->GetGCode(),block_over, count, compmsg->GetExecStep());

                }else
                    break;
            }

            if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
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
        this->m_channel_status.gmode[1] = G01_CMD;    //����ģʽ�£������޸ĵ�ǰģʽ,��֪ͨHMI
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
 * @brief ʵ��ִ�п��ٶ�λָ����Ϣ
 * @param msg : ָ����Ϣ
 * @param flag_block : �Ƿ���Ҫǿ�Ƹ��Ͽ������־
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteRapidMsg(RecordMsg *msg, bool flag_block){
    RapidMsg *rapidmsg = (RapidMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            rapidmsg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        this->m_mode_restart.pos_target = rapidmsg->GetTargetPos();  //����Ŀ��λ��
        this->m_mode_restart.gmode[1] = rapidmsg->GetGCode();   //����ģ̬
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

    if(msg->IsNeedWaitMsg() && (m_simulate_mode == SIM_NONE || m_simulate_mode == SIM_MACHINING)){//��Ҫ�ȴ�������
        if(rapidmsg->GetExecStep() == 0){ //ֻ�е�һ����ʼִ��ʱ��Ҫ�ȴ�
            int limit = 2;
            if(this->IsStepMode())
                limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

            //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
            int count = 0;
            while(1){
                bool block_over = CheckBlockOverFlag();
                if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                        m_channel_status.machining_state == MS_PAUSED ||
                        m_channel_status.machining_state == MS_WARNING ||
                        this->m_mask_run_pmc != 0){ //δ�ﵽִ������

                    //			printf("compensate exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
                    return false;    //��δ���е�λ
                }
                else if(++count < limit){
                    usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
                    //		printf("execute compensate msg[%d]: blockflag=%d, count = %d, step = %hhu\n", compmsg->GetGCode(),block_over, count, compmsg->GetExecStep());

                }else
                    break;
            }

            if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
                this->m_b_need_change_to_pause = false;
                m_n_run_thread_state = PAUSE;
                SetMachineState(MS_PAUSED);

                return false;
            }
        }

    }else if(!OutputData(msg, flag_block))
        return false;

    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){
        this->m_channel_status.gmode[1] = G00_CMD;    //����ģʽ�£������޸ĵ�ǰģʽ,��֪ͨHMI

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
 * @brief ʵ��ִ��Բ������ָ����Ϣ
 * @param msg : ָ����Ϣ
 * @param flag_block : �Ƿ���Ҫǿ�Ƹ��Ͽ������־
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteArcMsg(RecordMsg *msg, bool flag_block){
    ArcMsg *arc_msg = (ArcMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            arc_msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        this->m_mode_restart.pos_target = arc_msg->GetTargetPos();  //����Ŀ��λ��
        this->m_mode_restart.gmode[1] = arc_msg->GetGCode();   //����ģ̬

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
        this->m_channel_status.gmode[1] = arc_msg->GetGCode();    //����ģʽ�£������޸ĵ�ǰģʽ,��֪ͨHMI
    }

    /*if(m_channel_status.gmode[9] != G80_CMD){
		m_channel_status.gmode[9] = G80_CMD;
		this->SendChnStatusChangeCmdToHmi(G_MODE);
	}*/

    this->m_n_run_thread_state = RUN;

    return true;
}

/**
 * @brief ʵ��ִ������ϵָ��ָ����Ϣ
 * @param msg : ָ����Ϣ
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteCoordMsg(RecordMsg *msg){
    //���Ƚ������е����д�����ָ��͸�MC
	CoordMsg *coordmsg = (CoordMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ

		int gcode = coordmsg->GetGCode();
		if ((gcode >= G54_CMD && gcode <= G59_CMD)
				|| (gcode >= G5401_CMD && gcode <= G5499_CMD) || gcode == 541){
			this->m_mode_restart.gmode[14] = gcode;//����ģ̬
		}
		return true;
	}

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){
        if(m_simulate_mode == SIM_OUTLINE || m_simulate_mode == SIM_TOOLPATH)
            break;

        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            //		printf("coord exec return: 0x%x, count=%hu, block_over=%hhu\n", m_p_mc_comm->ReadRunOverValue(), this->ReadMcMoveDataCount(), block_over);
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            //		printf("execute coord msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

	if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
		this->m_b_need_change_to_pause = false;
		m_n_run_thread_state = PAUSE;
		SetMachineState(MS_PAUSED);
	//	printf("coord msg paused, line_no = %u\n", m_channel_rt_status.line_no);
		return false;
	}


    //���õ�ǰ�к�
    SetCurLineNo(msg->GetLineNo());


    bool flag = this->m_n_hw_trace_state==REVERSE_TRACE?true:false;    //�Ƿ�������
    //����ģ̬
    int gcode = flag?coordmsg->GetLastGCode():coordmsg->GetGCode();

	// @add zk
	if(gcode == G92_CMD){ //�趨��������ϵ

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
	if(gcode == G52_CMD){//�ֲ�����ϵ

        // @add  zk
        DPointChn point = coordmsg->GetTargetPos();
        uint32_t axis_mask = coordmsg->GetAxisMask();

        //���ֲ�����ƫ�ö�Ϊ0ʱ  ȡ���ֲ�����ϵ
        switch(coordmsg->GetExecStep()){
        case 0:
            //��һ������������ϵ��ֵ ��������ϵ
        	flag_cancel_g52 = true;

        	for(int i = 0; i < kMaxAxisChn; i++){
        		//this->SetMcAxisOrigin(i);
				if(axis_mask & (0x01<<i)){

					G52offset[i] = point.GetAxisValue(i);
					// ��ֵ  ����Ҫȡ�� G52
					if(G52offset[i] > 0.0001) flag_cancel_g52 = false;

					int64_t origin_pos = 0;
					if(G92Active){

						// g92 offset
						origin_pos += (int64_t)(m_p_chn_g92_offset->offset[i] * 1e7);
						// g52 offset
						origin_pos += (int64_t)(G52offset[i]* 1e7);
					}else{

						// ext offset
						origin_pos += m_p_chn_coord_config[0].offset[i] * 1e7;  //������������ϵ

						// G54XX offset
						int coord_index = m_channel_status.gmode[14];
						if(coord_index <= G59_CMD ){
							origin_pos += (int64_t)(m_p_chn_coord_config[coord_index/10-53].offset[i] * 1e7);    //��λ��mmת��Ϊ0.1nm
						}else if(coord_index <= G5499_CMD){
							origin_pos += (int64_t)(m_p_chn_ex_coord_config[coord_index/10-5401].offset[i] * 1e7);    //��λ��mmת��Ϊ0.1nm
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
            coordmsg->SetExecStep(1); //��ת��һ��
            return false;

        case 1:
            //�ڶ������ȴ�MC�����¹�������ϵ���
            usleep(8000);
            coordmsg->SetExecStep(2);  //��ת��һ��
            return false;

        case 2:
            //��������ͬ��λ��
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){

                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //ͬ���ѱ�������ƶ�ָ���λ��
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

    }else if(gcode == G53_CMD){ //��е����ϵ

        if(!OutputData(msg, true))
            return false;

        if(this->m_simulate_mode != SIM_NONE)  //������浱ǰλ��
            this->m_pos_simulate_cur_work = coordmsg->GetTargetPos();

    }else if(m_simulate_mode == SIM_NONE || m_simulate_mode == SIM_MACHINING){  //�Ƿ���ģʽ���߼ӹ�����ģʽ

    	if(G52Active) return true;

    	uint16_t coord_mc = 0;
        switch(coordmsg->GetExecStep()){
        case 0:
            //��һ��������ģ̬�����¹�������ϵ���͵�MC
            this->m_channel_status.gmode[14] = gcode;

            //����MC������ϵ����
            this->SetMcCoord(true);

            coordmsg->SetExecStep(1); //��ת��һ��
            return false;
        case 1:
            //�ڶ������ȴ�MC�����¹�������ϵ���
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadChnCurCoord(m_n_channel_index, coord_mc);  //��ȡMC��ǰ��������ϵ
            else
                this->m_p_mc_arm_comm->ReadChnCurCoord(m_n_channel_index, coord_mc);
            if(coord_mc*10 == gcode){
                coordmsg->SetExecStep(2);  //��ת��һ��
            }
            return false;
        case 2:
            //��������ͬ��λ��
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
                //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //ͬ���ѱ�������ƶ�ָ���λ��
            break;
        default:
            printf("execute coord msg[%hu] error, step = %hhu\n", gcode, coordmsg->GetExecStep());
            break;
        }

    }else{//�������棬��·����
        //�Ȱѵ�ǰλ�õĹ�������ת��Ϊ��е����
        this->TransWorkCoordToMachCoord(this->m_pos_simulate_cur_work, m_channel_status.gmode[14], m_mask_intp_axis);

        this->m_channel_status.gmode[14] = gcode;  //�޸�ģ̬

        //ת����ǰλ�õĹ�������
        this->TransMachCoordToWorkCoord(m_pos_simulate_cur_work, m_channel_status.gmode[14], m_mask_intp_axis);

        if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
            this->m_p_compiler->SetCurPos(m_pos_simulate_cur_work);   //ͬ��������λ��
        }else
            this->RefreshOuputMovePos(m_pos_simulate_cur_work);    //ͬ���ѱ�������ƶ�ָ���λ��
    }

    printf("execute coord message : %d\n", m_channel_status.gmode[14]);

    this->SendChnStatusChangeCmdToHmi(G_MODE);

    return true;
}

/**
 * @brief ʵ��ִ�е���ָ����Ϣ
 * @param msg : ָ����Ϣ
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteToolMsg(RecordMsg *msg){
    //����ģʽ�����Ƚ������е����д�����ָ��͸�MC
    //	if(this->m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE)){
    //		if(!this->OutputAllData()){
    //			//PL�е�FIFO����������ʧ��
    //			return false;
    //		}
    //	}
    ToolMsg *toolmsg = (ToolMsg *)msg;
    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        this->m_mode_restart.cur_tcode = toolmsg->GetTool(0);
#ifdef USES_T_CODE_MACRO
        this->m_mode_restart.sub_prog_call++;
#endif
        return true;
    }

    if(this->m_simulate_mode != SIM_NONE){  //����ģʽ
        //���õ�ǰ�к�
        SetCurLineNo(toolmsg->GetLineNo());
        m_n_run_thread_state = RUN;
        return true;
    }


#ifdef USES_T_CODE_MACRO
    //����ģʽ�£��ȴ�MC��������������
    if(this->IsStepMode()){
        int count = 0;
        while(count < 4 ){
            if(this->ReadMcMoveDataCount() > 0 ||
                    (!CheckStepOverFlag() && !CheckBlockOverFlag()) ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){
                return false;    //��δ���е�λ
            }
            else{
                count++;
                printf("tool: step=%d, %d, c = %d\n", m_channel_mc_status.step_over,m_channel_mc_status.auto_block_over, count);
                usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            }
        }

        if(this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }
    }else{//�ǵ���ģʽ
        if(m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING)
            return false;
    }

    this->m_n_cur_tcode = toolmsg->GetTool(0);
    this->m_channel_status.preselect_tool_no = m_n_cur_tcode;



    //	bool flag = (m_n_hw_trace_state==REVERSE_TRACE)?true:false;    //�Ƿ�������
    //
    //	if(flag){ //��������
    //		if(this->m_p_general_config->debug_mode > 0){  //��ʽģʽ�£��򿪺�����ļ�
    //			if(toolmsg->GetSubProgType() > 1){
    //				char file[kMaxFileNameLen];
    //				memset(file, 0x00, kMaxFileNameLen);
    //				toolmsg->GetLastProgFile(file);
    //				this->SendOpenFileCmdToHmi(file);
    //			}
    //		}
    //
    //		//���õ�ǰ�к�
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
        this->m_error_code = ERR_SUB_NESTED_COUNT;   //�ӳ���Ƕ�ײ�������
        CreateError(ERR_SUB_NESTED_COUNT, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        return false;
    }

    //TODO ��HMI������������ļ�
    if(this->m_p_general_config->debug_mode > 0){  //��ʽģʽ�£��򿪺�����ļ�
        if(toolmsg->GetSubProgType() > 1){
            char file[kMaxFileNameLen];
            memset(file, 0x00, kMaxFileNameLen);
            toolmsg->GetSubProgName(file, false);
            this->SendOpenFileCmdToHmi(file);
        }
    }

    //���õ�ǰ�к�
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
        //���Ƚ������е����д�����ָ��͸�MC
        //		if(!OutputLastBlockItem()){
        //			//PL�е�FIFO����������ʧ��
        //			return false;
        //		}

        int limit = 2;
        if(this->IsStepMode())
            limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

        //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
        int count = 0;
        bool block_over = false;
        while(1){
            block_over = CheckBlockOverFlag();
            if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
                //		printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
                return false;    //��δ���е�λ
            }
            else if(++count < limit){
                usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
                //		printf("execute aus msg: blockflag=%d, count = %d\n",  block_over, count);

            }else
                break;
        }

        if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
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
        if(toolmsg->GetExecStep(index) == 0xFF)//�Ѿ�ִ�н����ľ�����
            continue;

        tcode = toolmsg->GetTool(index);
        printf("tool index = %hhu, tcode=%hu, step = %hhu\n", index, tcode, toolmsg->GetExecStep(index));

        switch(toolmsg->GetExecStep(index)){
        case 0:{
            this->m_n_cur_tcode = tcode;
            this->m_channel_status.preselect_tool_no = m_n_cur_tcode;

            //TODO �����뷢�͸�PMC
            this->SendTCodeToPmc(tcode, index);

            gettimeofday(&m_time_t_start[index], NULL);   //

            toolmsg->IncreaseExecStep(index);
            break;
        }
        case 1:{
        	//�ȴ�TMF��ʱ����λTFѡͨ�ź�
            gettimeofday(&time_now, NULL);
            time_elpase = (time_now.tv_sec-m_time_t_start[index].tv_sec)*1000000+time_now.tv_usec-m_time_t_start[index].tv_usec;
            if(time_elpase < 16000)
                break;		//δ����ʱʱ��

            this->SetTFSig(index, true);    //��λѡͨ�ź�

            toolmsg->IncreaseExecStep(index);
            break;
        }
        case 2:{
            //�ȴ���Ӧ��TFIN�ź�
            if(this->GetTFINSig(index))
                gettimeofday(&m_time_t_start[index], NULL);   //��ʼ��ʱ
            else
                break;

            toolmsg->IncreaseExecStep(index);
            break;
        }
        case 3:{
        	if(!this->GetTFINSig(index)){  //�����źţ�����ȷ��
                toolmsg->SetExecStep(index, 2);
                break;
            }

            //�ȴ�TFIN��ʱ����λTF�ź�
            gettimeofday(&time_now, NULL);
            time_elpase = (time_now.tv_sec-m_time_t_start[index].tv_sec)*1000000+time_now.tv_usec-m_time_t_start[index].tv_usec;
            if(time_elpase < 16000)
                break;		//δ����ʱʱ��

            this->SetTFSig(index, false);    //��λѡͨ�ź�

            toolmsg->IncreaseExecStep(index);
            break;
        }
        case 4:{
            //�ȴ�FIN�źŸ�λ
            if(this->GetTFINSig(index))
                break;

            //��λ����ָ���ź�
            this->SendTCodeToPmc(0, index);

            toolmsg->SetExecStep(index, 0xFF);   //0xFF��ʾִ�н���

            printf("execute T%hu over\n", tcode);
            break;
        }
        default:
            break;
        }

        if(toolmsg->GetExecStep(index) != 0xFF)//��δִ�н���������false
            bRet = false;

    }

#endif

    printf("execute tool message: cur_t_code=%d, return %hhu\n", m_n_cur_tcode, bRet);

    return bRet;
}

/**
 * @brief ʵ��ִ��һ��ģָ̬����Ϣ������������ģָ̬��
 * @param msg : ָ����Ϣ
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteModeMsg(RecordMsg *msg){
    //����ģʽ�����Ƚ������е����д�����ָ��͸�MC
    //	if(this->m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE)){
    //		if(!this->OutputAllData()){
    //			//PL�е�FIFO����������ʧ��
    //			return false;
    //		}
    //	}
    ModeMsg *modemsg = (ModeMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        int cmd = modemsg->GetGCode();
        //	this->m_mode_restart.pos_target = loopmsg->GetTargetPos();
        this->m_mode_restart.gmode[GetModeGroup(cmd)] = cmd;
        return true;
    }

    if(m_simulate_mode == SIM_OUTLINE && m_simulate_mode == SIM_TOOLPATH){  //�������棬��·����
        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());
        return true;
    }

    int count = 0;
    while(count < 4 ){
        if(this->ReadMcMoveDataCount() > 0 || !this->CheckBlockOverFlag() ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){
            return false;    //��δ���е�λ
        }
        else{
            count++;
            //			printf("ExecuteModeMsg: step=%d, %d, c = %d\n", m_channel_mc_status.step_over,m_channel_mc_status.auto_block_over, count);
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
        }
    }

    //����ģʽ
    if(this->IsStepMode()){
        if(this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            printf("mode msg[%hu] paused, line_no = %u\n", modemsg->GetGCode(), m_channel_rt_status.line_no);
            return false;
        }

        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());

    }else{//�ǵ���ģʽ
        if(m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING)
            return false;
    }



    bool flag = this->m_n_hw_trace_state==REVERSE_TRACE?true:false;    //�Ƿ�������
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

    //����ģ̬
    int mode_index = GCode2Mode[cmd/10];
    if(McModeFlag[mode_index]){//��Ҫ���͸�MC��ģָ̬��
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
        this->RefreshModeInfo(m_mc_mode_exec);	//����ģʽ��Ҫ��������ģ̬
    }


    this->m_channel_status.gmode[GCode2Mode[cmd/10]] = cmd;


    //TODO ִ�и�ģָ̬��
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

    //֪ͨHMI
    this->SendChnStatusChangeCmdToHmi(G_MODE);

    return true;
}

/**
 * @brief ʵ��ִ�н����ٶ�ָ��ָ����Ϣ
 * @param msg : ָ����Ϣ
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteFeedMsg(RecordMsg *msg){

    //	if(m_channel_status.machining_state == MS_PAUSED ||
    //			m_channel_status.machining_state == MS_WARNING)
    //		return false;
    //
    //	//����ģʽ�£��ȴ�MC��������������
    //	if(this->m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE)){
    //		int count = 0;
    //		while(count < 2 ){
    //			if(this->ReadMcMoveDataCount() > 0 || (!this->CheckStepOverFlag() && !this->CheckBlockOverFlag())){
    //				return false;    //��δ���е�λ
    //			}
    //			else{
    //				count++;
    //	//			printf("execute feed msg: stepflag=%d,  count = %d\n", m_channel_mc_status.step_over, count);
    //				usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
    //			}
    //		}
    //	}

    //����״̬
    //	FeedMsg *feedmsg = (FeedMsg *)msg;

    //	this->m_channel_status.rated_feed = feedmsg->GetFeed()*1000/60;   //��λת��:mm/min --> um/s
    return true;
}

/**
 * @brief ʵ��ִ������ת��ָ����Ϣ
 * @param msg : ָ����Ϣ
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteSpeedMsg(RecordMsg *msg){
    //����ģʽ�����Ƚ������е����д�����ָ��͸�MC
    //	if(this->m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE)){
    //		if(!this->OutputAllData()){
    //			//PL�е�FIFO����������ʧ��
    //			return false;
    //		}
    //	}

    SpeedMsg *speed = (SpeedMsg *)msg;

    //TODO ��ǰ���õ�һ���������ж�
    uint8_t phy_spd = this->m_spd_axis_phy[0];
    SCAxisConfig *spd_config = nullptr;
    if(phy_spd > 0)
        spd_config = &m_p_axis_config[phy_spd-1];

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        this->m_mode_restart.cur_scode = speed->GetSpeed();

        if(phy_spd > 0){
            if(m_mode_restart.cur_scode > (int)spd_config->spd_max_speed)
                m_mode_restart.cur_scode = spd_config->spd_max_speed;
            if(m_mode_restart.cur_scode < (int)spd_config->spd_min_speed)
                m_mode_restart.cur_scode = spd_config->spd_min_speed;
        }

        if(m_mode_restart.spindle_dir != SPD_DIR_STOP){  //��ǰ�����Ѿ�������ת״̬
            m_mode_restart.rated_spindle_speed = m_mode_restart.cur_scode;
        }

        return true;
    }

    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){  //�������棬��·����

        //���õ�ǰ�к�
        SetCurLineNo(speed->GetLineNo());

        m_n_run_thread_state = RUN;

        return true;
    }

    //����ģʽ�£��ȴ�MC��������������
    //	if(this->IsStepMode()){

#ifdef USES_WOOD_MACHINE
    double td = 0;
    int forward_line = 0;    //����Ԥ������ǰ����
    if(this->GetMacroVar(810, td) == 0)
        forward_line = td;

    //		printf("!!!!SPEED MSG,line=%d, curline=%u, msgline=%llu\n", forward_line, m_channel_rt_status.line_no, msg->GetLineNo());

    if(!this->IsStepMode() && ((m_channel_rt_status.line_no+forward_line) >= msg->GetLineNo())){  //�ǵ���ģʽ�µ���������ָ��Ԥ����
        printf("S cmd forward execute at line:%u\n", m_channel_rt_status.line_no);
    }else{

#endif
        int count = 0;
        while(count < 4 ){
            if(this->ReadMcMoveDataCount() > 0 ||
                    (!this->CheckStepOverFlag() && !this->CheckBlockOverFlag()) ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){
                return false;    //��δ���е�λ
            }
            else{
                count++;
                //				printf("ExecuteSpeedMsg: step=%d, %d, c = %d\n", m_channel_mc_status.step_over,m_channel_mc_status.auto_block_over, count);
                usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            }
        }

        if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }
#ifdef USES_WOOD_MACHINE
    }
#endif
    //	}else{//�ǵ���ģʽ
    //		if(m_channel_status.machining_state == MS_PAUSED ||
    //				m_channel_status.machining_state == MS_WARNING)
    //			return false;
    //	}

    //���õ�ǰ�к�
    SetCurLineNo(msg->GetLineNo());

    //S�������뵽����ģ��
    m_p_spindle->InputSCode(speed->GetSpeed());

    //ScPrintf("ExecuteSpeedMsg::%d rpm\n", m_p_spindle->GetSCode());

    //���µ�ǰSֵ
    m_channel_status.rated_spindle_speed = m_p_spindle->GetSCode();
    this->SendModeChangToHmi(S_MODE);

    return true;
}

/**
 * @brief ʵ��ִ��ѭ��ָ����Ϣ
 * @param msg : ָ����Ϣ
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteLoopMsg(RecordMsg *msg){
    //����ģʽ�����Ƚ������е����д�����ָ��͸�MC
    //	if(this->m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE)){
    //		if(!this->OutputAllData()){
    //			//PL�е�FIFO����������ʧ��
    //			return false;
    //		}
    //	}
    LoopMsg *loopmsg = (LoopMsg *)msg;
    int cmd = loopmsg->GetGCode();

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        //	this->m_mode_restart.pos_target = loopmsg->GetTargetPos();
        this->m_mode_restart.gmode[GetModeGroup(cmd)] = cmd;
        this->m_mode_restart.sub_prog_call++;
        return true;
    }

    if(m_simulate_mode == SIM_OUTLINE && m_simulate_mode == SIM_TOOLPATH){  //�������棬��·����
        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());
        m_n_subprog_count++;
        m_n_macroprog_count++;
        return true;
    }
    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int limit = 1;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    int count = 0;
    while(1){

        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            //	printf("loop exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            printf("execute loop msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    if(m_n_subprog_count >= kMaxSubNestedCount){
        this->m_error_code = ERR_SUB_NESTED_COUNT;   //�ӳ���Ƕ�ײ�������
        CreateError(ERR_SUB_NESTED_COUNT, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        return false;
    }

    //TODO ��HMI������������ļ�
    if(this->m_p_general_config->debug_mode > 0){  //��ʽģʽ�£��򿪺�����ļ�
        if(this->m_channel_status.chn_work_mode == AUTO_MODE){
            if(loopmsg->GetMacroProgType() > 1){
                char file[kMaxFileNameLen];
                memset(file, 0x00, kMaxFileNameLen);
                loopmsg->GetMacroProgName(file, false);
                this->SendOpenFileCmdToHmi(file);
            }
        }
    }

    //���õ�ǰ�к�
    SetCurLineNo(msg->GetLineNo());

    m_n_subprog_count++;
    m_n_macroprog_count++;

    if(this->m_p_general_config->debug_mode == 0){
        this->SetMcStepMode(false);
    }

    //����ģ̬
    if(cmd == G89_CMD)
        m_mc_mode_exec.bits.mode_g73 = 15;    //mode_g74ֻ��4bit��ֵ��Χ0~15�����G89��Ҫ���⴦��
    else
        m_mc_mode_exec.bits.mode_g73 = cmd/10 - 73;

    //	printf("execute loop msg: gmode[9] = %hu, gcode = %d\n", m_channel_status.gmode[9], loopmsg->GetGCode());

    if(m_channel_status.gmode[9] == G80_CMD &&
            (loopmsg->GetGCode() == G84_CMD || loopmsg->GetGCode() == G74_CMD)){ //�л����Թ�˿ģ̬���������ݸ�MI
        uint8_t pc = 0;
        uint32_t pm = 0;
        uint32_t mask = 0x01;
        double feed = 0;
        int i  = 0;
        double *pp = loopmsg->GetParameter(pm, pc);

        while(pm != 0){  //��ȡF����
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
            && loopmsg->GetGCode() == G80_CMD){ // �˳���˿
        m_p_spindle->ResetTapFlag();
    }

    this->m_channel_status.gmode[9] = loopmsg->GetGCode();

    // ֪ͨHMIģ̬�仯
    this->SendChnStatusChangeCmdToHmi(G_MODE);

    return true;
}

/**
 * @brief ʵ��ִ�е���ָ����Ϣ
 * @param msg : ָ����Ϣ
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteCompensateMsg(RecordMsg *msg){
    CompensateMsg *compmsg = (CompensateMsg *)msg;
    int type = compmsg->GetGCode();

    if(this->m_n_restart_mode != NOT_RESTART &&
            compmsg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
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
        this->m_pos_simulate_cur_work = compmsg->GetTargetPos();  //�������ģʽ��ǰλ��

    int count = 0;

    if(msg->IsNeedWaitMsg() && (m_simulate_mode == SIM_NONE || m_simulate_mode == SIM_MACHINING)){//��Ҫ�ȴ�������
        int limit = 2;
        if(this->IsStepMode())
            limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

        //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
        while(1){
            bool block_over = CheckBlockOverFlag();
            if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
                //			printf("compensate exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
                return false;    //��δ���е�λ
            }
            else if(++count < limit){
                usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
                //		printf("execute compensate msg[%d]: blockflag=%d, count = %d, step = %hhu\n", compmsg->GetGCode(),block_over, count, compmsg->GetExecStep());

            }else
                break;
        }

        if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }

        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());

    }
    else{//����ȴ�������

        //����ģʽ�£��ȴ�MC��������������
        if(this->IsStepMode()){
            int count = 0;
            while(count < 4 ){
                if(this->ReadMcMoveDataCount() > 0 ||
                        (!this->CheckStepOverFlag() && !this->CheckBlockOverFlag()) ||
                        m_channel_status.machining_state == MS_PAUSED ||
                        m_channel_status.machining_state == MS_WARNING){
                    return false;    //��δ���е�λ
                }
                else{
                    count++;
                    printf("execute compensate msg[%d]: step=%d, %d, c = %d\n", compmsg->GetGCode(), m_channel_mc_status.step_over,m_channel_mc_status.auto_block_over, count);
                    usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
                }
            }
            //���õ�ǰ�к�
            SetCurLineNo(msg->GetLineNo());

        }else{//�ǵ���ģʽ
            if(m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING)
                return false;
        }
    }

    // @test zk
	if(type == G41_CMD || type == G42_CMD || type == G40_CMD){//�뾶�������ȸ��µ�MCģ̬��Ϣ
		if(m_simulate_mode == SIM_NONE || m_simulate_mode == SIM_MACHINING){  //�Ƿ���ģʽ���߼ӹ�����
		   this->m_mc_mode_exec.bits.mode_g40 = (type-G40_CMD)/10;
		   //this->m_mc_mode_exec.bits.mode_d = value;

		   if(this->IsStepMode()){
			   this->RefreshModeInfo(m_mc_mode_exec);	//����ģʽ��Ҫ��������ģ̬
		   }
		}

		this->m_channel_status.cur_d_code = compmsg->GetCompValue();;
		this->SendModeChangToHmi(D_MODE);
		m_n_run_thread_state = RUN;
		return true;
	}
	// @test zk

    bool flag = this->m_n_hw_trace_state==REVERSE_TRACE?true:false;    //�Ƿ�������
        int value = flag?compmsg->GetLastCompValue():compmsg->GetCompValue();
    //�л�����ֵ������ģ̬

    uint16_t offset_mc = 0;  //mc�еĵ�ǰ��ƫ
    int32_t z_axis_offset = 0;
    uint16_t intp_mode = 0;

    if(type == G43_CMD || type == G44_CMD || type == G49_CMD){//G43/G44    ��ʱ���µ�ͨ��״̬��ģ̬��Ϣ��

    	if(this->m_simulate_mode != SIM_NONE){//����ģʽ��������������
            if(!OutputData(msg, true))
                return false;
            m_n_run_thread_state = RUN;
            return true;
        }


        //	printf("execute G49 msg:%hhu\n", compmsg->GetExecStep());
        switch(compmsg->GetExecStep()){
        case 0://��һ��������ƫ�÷��͵�MC
#ifdef USES_FIVE_AXIS_FUNC
            m_b_5axis_clear_rot = false;
#endif
            if(type == G49_CMD){
                if(this->m_channel_status.gmode[8] == G43_4_CMD){ //ȡ��RTCP
                    this->SetMcRtcpMode(G43_4_MODE, CANCEL_MODE, 0);
#ifdef USES_FIVE_AXIS_FUNC
                    m_b_5axis_clear_rot = true;  //��ת����Ҫ������Ȧ
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

            compmsg->SetExecStep(1);	//��ת��һ��
            printf("execute step 1\n");
            return false;
        case 1:
            //�ڶ������ȴ�MCִ���µ�ƫ���
            if(type == G49_CMD && this->m_channel_status.gmode[8] == G43_4_CMD){//ȡ��RTCP
                if(!this->m_b_mc_on_arm)
                    this->m_p_mc_comm->ReadChnCurIntpMode(this->m_n_channel_index, intp_mode);  //��ȡMC��ǰ����岹ģʽ
                else
                    this->m_p_mc_arm_comm->ReadChnCurIntpMode(this->m_n_channel_index, intp_mode);
                if(intp_mode == CANCEL_MODE){
                    this->m_channel_status.gmode[8] = type;
                    this->m_channel_status.cur_h_code = value;
                    compmsg->SetExecStep(2);  //��ת��һ��
                }
                else
                    printf("G49, intp_mode = %hu\n", intp_mode);
            }else{
                if(!this->m_b_mc_on_arm)
                    this->m_p_mc_comm->ReadChnCurToolOffset(this->m_n_channel_index, offset_mc);  //��ȡMC��ǰ��ƫ
                else
                    this->m_p_mc_arm_comm->ReadChnCurToolOffset(m_n_channel_index, offset_mc);
                if(offset_mc == value){
                    compmsg->SetExecStep(2);  //��ת��һ��
                }
                printf("offset_mc = %hu, value = %hu\n", offset_mc, value);
            }
            printf("execute step 2\n");
            return false;
        case 2:
            //��������ͬ��λ��
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            RefreshAxisIntpPos();

            //			printf("compensate: refreshpos[%lf, %lf, %lf]\n", m_channel_rt_status.cur_pos_work.m_df_point[0], m_channel_rt_status.cur_pos_work.m_df_point[1],
            //								m_channel_rt_status.cur_pos_work.m_df_point[2]);


            //ˢ���˶�ָ��Ŀ��λ��
            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);


            this->SendChnStatusChangeCmdToHmi(G_MODE);
            this->SendModeChangToHmi(H_MODE);
#ifdef USES_FIVE_AXIS_FUNC
            if(m_b_5axis_clear_rot){
                compmsg->SetExecStep(3);  //��ת��һ��
                printf("execute step 3\n");
            }else
#endif
                compmsg->SetExecStep(6);  //����������Ȧ����
            printf("execute step 6\n");
            return false;
#ifdef USES_FIVE_AXIS_FUNC
        case 3:  //���Ĳ�: �����G49ȡ������任����ִ��G200����ת��������Ȧ

            m_mask_5axis_rot_nolimit_phyaxis = 0;
            //���������Ҫ��������ת��������Ȧ
            if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS){
                //��MI����������Ȧλ��ָ��
                m_n_mask_clear_pos = 0;

                for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
                    if(((m_mask_5axis_rot_nolimit>>i) & 0x01) == 0){

                        continue;
                    }
                    uint8_t PhyAxisIndex = GetPhyAxis(i);
                    m_mask_5axis_rot_nolimit_phyaxis |= (0x01<<PhyAxisIndex);
                    this->SendMiClearPosCmd(PhyAxisIndex+1, 360*1000);
                }
                compmsg->SetExecStep(4);	//��ת��һ��
            }else
                compmsg->SetExecStep(6);	//����������Ȧ����
            printf("execute step 4\n");
            return false;
        case 4://���岽
            //�ȴ�MI�������
            if((m_mask_5axis_rot_nolimit_phyaxis & this->m_n_mask_clear_pos) == m_mask_5axis_rot_nolimit_phyaxis) {    //if(m_mask_5axis_rot_nolimit == this->m_n_mask_clear_pos)
                compmsg->SetExecStep(5);	//��ת��һ��
            }
            printf("execute step 5\n");
            return false;
        case 5:
            //��������ͬ��λ��

            //��MCͬ����������
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

            //��MIͬ����е����
            this->m_p_channel_engine->SendMonitorData(false, false);

            //		printf("cur mach pos (%lf, %lf, %lf, %lf)\n", m_channel_rt_status.cur_pos_machine.x, m_channel_rt_status.cur_pos_machine.y,
            //				m_channel_rt_status.cur_pos_machine.z, m_channel_rt_status.cur_pos_machine.a6);

            compmsg->SetExecStep(6);  //��ת��һ��
            printf("execute step 6\n");
            return false;
#endif
        case 6:
            //���߲����������˶�ָ�������MC
            if(!compmsg->CheckFlag(FLAG_AXIS_MOVE)){//û�����ƶ�������
                //	this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
                break;
            }

            // @zk  ������������� ���� MC tips: ����ָ��ûָ��cmd ָ��Ϊ00 �������
            if(!OutputData(msg, true))
                return false;

            this->StartMcIntepolate();  //����MC
            //			compmsg->SetExecStep(7);  //��ת��һ��
            printf("execute g49 mode over\n");
            break;
            //		case 7:
            //			//���岽���ȴ�MC���е�λ
            //			//�ں���ͷ���ȴ�
            //
            //			printf("execute step 4\n");
            //			break;
        default:
            printf("G43/G44/G49 execute error!\n");
            break;
        }

    }else if(type == G43_4_CMD){  //����RTCP
        switch(compmsg->GetExecStep()){  //�����̣��ȼ���RTCP�����˶�ָ��
        case 0://��һ��������ƫ�÷��͵�MC

            z_axis_offset = m_p_chn_tool_config->geometry_compensation[value-1][2] * 1e3;  //��λ��mmת��Ϊum
            //	z_axis_offset += m_p_chn_tool_config->geometry_comp_basic[2] * 1e3;   //��׼��ƫ
            z_axis_offset += m_p_chn_tool_config->geometry_wear[value-1] * 1e3;   //����ĥ�𲹳�

            //	printf("G43.4 send :idx = %d, offset=%d\n", value, z_axis_offset);

            this->SetMcRtcpMode(G43_4_MODE, G43_4_MODE, z_axis_offset);

            compmsg->SetExecStep(1);	//��ת��һ��
            return false;
        case 1:
            //�ڶ������ȴ�MCִ���µ�ƫ���
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadChnCurIntpMode(this->m_n_channel_index, intp_mode);  //��ȡMC��ǰ����岹ģʽ
            else
                this->m_p_mc_arm_comm->ReadChnCurIntpMode(m_n_channel_index, intp_mode);
            if(intp_mode == G43_4_MODE){
                compmsg->SetExecStep(2);  //��ת��һ��
            }
            return false;
        case 2:
            //��������ͬ��λ��
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();


            //			compmsg->RefreshTargetPos(m_channel_mc_status.intp_pos);    //ͬ����ǰ�е�Ŀ��λ��
            //			this->m_p_compiler->SetCurPos(compmsg->GetTargetPos());   //ͬ��������λ��
            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);  //ͬ��λ��

            this->m_channel_status.gmode[8] = type;
            this->m_channel_status.cur_h_code = value;

            this->SendChnStatusChangeCmdToHmi(G_MODE);
            this->SendModeChangToHmi(H_MODE);


            compmsg->SetExecStep(3);	//��ת��һ��
            return false;
        case 3:
            //���Ĳ����������˶�ָ�������MC
            if(!compmsg->CheckFlag(FLAG_AXIS_MOVE)){//û�����ƶ�,����
                break;
            }

            if(!OutputData(msg, true))
                return false;

            this->StartMcIntepolate();  //����MC

            printf("execute G43.4 over\n");
            break;

        default:
            printf("G43/G44/G49 execute error!\n");
            break;
        }


        //�ϵĴ������̣��������ƶ�ָ���ټ���RTCP
        //		switch(compmsg->GetExecStep()){
        //		case 0:
        //			//��һ�����������˶�ָ�������MC
        //			if(!compmsg->CheckFlag(FLAG_AXIS_MOVE)){//û�����ƶ�����ת������
        //				compmsg->SetExecStep(2);  //��ת��һ��
        //				return false;
        //			}
        //
        //			if(!OutputData(msg))
        //				return false;
        //
        //			this->StartMcIntepolate();  //����MC
        //
        //			compmsg->SetExecStep(1);  //��ת��һ��
        //			printf("execute step 0\n");
        //			return false;
        //		case 1:
        //			//�ڶ������ȴ�MC���е�λ
        //			//�ں���ͷ���ȴ�
        //			compmsg->SetExecStep(2);  //��ת��һ��
        //			printf("execute step 1\n");
        //			return false;
        //		case 2://������������ƫ�÷��͵�MC
        //
        //			z_axis_offset = m_p_chn_tool_config->geometry_compensation[value-1][2] * 1e3;  //��λ��mmת��Ϊum
        //			z_axis_offset += m_p_chn_tool_config->geometry_wear[value-1] * 1e3;   //����ĥ�𲹳�
        //
        //			this->SetMcRtcpMode(G43_4_MODE, G43_4_MODE, z_axis_offset);
        //
        //			compmsg->SetExecStep(3);	//��ת��һ��
        //			return false;
        //		case 3:
        //			//�ڶ������ȴ�MCִ���µ�ƫ���
        //			this->m_p_mc_comm->ReadChnCurIntpMode(this->m_n_channel_index, intp_mode);  //��ȡMC��ǰ����岹ģʽ
        //			if(intp_mode == G43_4_MODE){
        //				compmsg->SetExecStep(4);  //��ת��һ��
        //			}
        //			return false;
        //		case 4:
        //			//��������ͬ��λ��
        //			this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        //
        //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        //
        //			this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
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
 * @brief ʵ��ִ�д���ָ����Ϣ
 * @param msg : ָ����Ϣ
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteErrorMsg(RecordMsg *msg){
    //���Ƚ������е����д�����ָ��͸�MC
    //	if(this->m_channel_status.func_state_flags.CheckMask(FS_SINGLE_LINE)){
    //		if(!this->OutputAllData()){
    //			//PL�е�FIFO����������ʧ��
    //			return false;
    //		}
    //	}

    int limit = 1;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){
        if(m_simulate_mode == SIM_OUTLINE || m_simulate_mode == SIM_TOOLPATH)
            break;

        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            //	printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            printf("execute error msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    //���ɸ澯
    ErrorMsg *err = (ErrorMsg *)msg;
    if(err->GetInfoType() == 1){  //�澯
        this->m_error_code = (ErrorType)err->GetErrorCode();
        printf("execute error msg: %d, %llu\n", m_error_code, err->GetLineNo());
        if (!m_b_lineno_from_mc)    //llx add,����ӹ��ļ������кŶ�Ӧ��������
            SetCurLineNo(err->GetLineNo());
        this->m_n_run_thread_state = ERROR;
        CreateError(err->GetErrorCode(), ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
    }else if(err->GetInfoType() == 0){  //��ʾ
        if(err->GetErrorCode() == 0){//��յ�ǰ��ʾ��Ϣ
            ClearHmiInfoMsg();
        }else{ //������ʾ��Ϣ
            //			CreateError(err->GetErrorCode(), INFO_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            this->SendMessageToHmi(MSG_TIPS, err->GetErrorCode());
        }
        this->m_n_run_thread_state = RUN;
    }

    return true;
}

/**
 * @brief ʵ��ִ���ӳ��������Ϣ
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteSubProgCallMsg(RecordMsg *msg){
    SubProgCallMsg *sub_msg = (SubProgCallMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        this->m_mode_restart.sub_prog_call++;
        return true;
    }

    if(m_simulate_mode == SIM_OUTLINE && m_simulate_mode == SIM_TOOLPATH){  //�������棬��·����
        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());

        m_n_subprog_count++;

        return true;
    }

    int limit = 1;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            //	printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            printf("execute subprog call msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    bool flag = (m_n_hw_trace_state==REVERSE_TRACE)?true:false;    //�Ƿ�������
    int mcode = sub_msg->GetMCode(0);

    if(mcode == 98){

    	if(flag){//��������
            m_n_subprog_count -= 1;

            if(sub_msg->GetSubProgType() == 2 ||
                    sub_msg->GetSubProgType() == 4){
                char file[kMaxFileNameLen];
                memset(file, 0x00, kMaxFileNameLen);
                sub_msg->GetLastProgFile(file);
                this->SendOpenFileCmdToHmi(file);   //֪ͨ����һ������ļ�
            }

            //���õ�ǰ�к�
            SetCurLineNo(msg->GetLineNo());

        }else{
            if(m_n_subprog_count >= kMaxSubNestedCount){
                this->m_error_code = ERR_SUB_NESTED_COUNT;   //�ӳ���Ƕ�ײ�������
                CreateError(ERR_SUB_NESTED_COUNT, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
                return false;
            }

            if(sub_msg->GetExecStep(0) == 0){
                m_n_subprog_count += 1;

                //TODO ��HMI������������ļ�
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

                //���õ�ǰ�к�
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
 * @brief ʵ��ִ�к���������Ϣ
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteMacroProgCallMsg(RecordMsg *msg){
    MacroProgCallMsg *macro_msg = (MacroProgCallMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ

        this->m_mode_restart.sub_prog_call++;
        return true;
    }

    //	printf("ChannelControl::ExecuteMacroProgCallMsg\n");

    if(m_simulate_mode == SIM_OUTLINE && m_simulate_mode == SIM_TOOLPATH){  //�������棬��·����
        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());

        m_n_subprog_count++;
        m_n_macroprog_count++;

        return true;
    }

    int limit = 1;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){

        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            //	printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            printf("execute subprog call msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    bool flag = (m_n_hw_trace_state==REVERSE_TRACE)?true:false;    //�Ƿ�������

    if(flag){ //��������
        if(this->m_p_general_config->debug_mode > 0){  //��ʽģʽ�£��򿪺�����ļ�
            if(macro_msg->GetMacroProgType() > 1){
                char file[kMaxFileNameLen];
                memset(file, 0x00, kMaxFileNameLen);
                macro_msg->GetLastProgFile(file);
                this->SendOpenFileCmdToHmi(file);
            }
        }

        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());

        m_n_subprog_count--;
        m_n_macroprog_count--;

    }else{
        if(m_n_subprog_count >= kMaxSubNestedCount){
            this->m_error_code = ERR_SUB_NESTED_COUNT;   //�ӳ���Ƕ�ײ�������
            CreateError(ERR_SUB_NESTED_COUNT, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            return false;
        }

        //TODO ��HMI������������ļ�
        if(this->m_p_general_config->debug_mode > 0){  //��ʽģʽ�£��򿪺�����ļ�
            if(this->m_channel_status.chn_work_mode == AUTO_MODE){
                if(macro_msg->GetMacroProgType() > 1){
                    char file[kMaxFileNameLen];
                    memset(file, 0x00, kMaxFileNameLen);
                    macro_msg->GetMacroProgName(file, false);
                    this->SendOpenFileCmdToHmi(file);
                }
            }
        }

        //���õ�ǰ�к�
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
 * @brief ʵ��ִ���Զ��Ե�ָ����Ϣ
 * @param msg : ִ����Ϣָ��
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteAutoToolMeasureMsg(RecordMsg *msg){
    AutoToolMeasureMsg *macro_msg = (AutoToolMeasureMsg *)msg;
    int limit = 1;
    bool block_over = false;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    if(this->m_n_restart_mode != NOT_RESTART &&
            macro_msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        this->m_mode_restart.sub_prog_call++;
        return true;
    }

    if(this->m_simulate_mode != SIM_NONE){  //����ʱֱ�ӷ���
        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());
        return true;
    }


    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){
        block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            //	printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            printf("execute subprog call msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }


    if(m_n_subprog_count >= kMaxSubNestedCount){
        this->m_error_code = ERR_SUB_NESTED_COUNT;   //�ӳ���Ƕ�ײ�������
        CreateError(ERR_SUB_NESTED_COUNT, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        return false;
    }

    //TODO ��HMI������������ļ�
    if(this->m_p_general_config->debug_mode > 0){  //��ʽģʽ�£��򿪺�����ļ�
        if(this->m_channel_status.chn_work_mode == AUTO_MODE){
            if(macro_msg->GetMacroProgType() > 1){
                char file[kMaxFileNameLen];
                memset(file, 0x00, kMaxFileNameLen);
                macro_msg->GetMacroProgName(file, false);
                this->SendOpenFileCmdToHmi(file);
            }
        }
    }

    //���õ�ǰ�к�
    SetCurLineNo(msg->GetLineNo());

    m_n_subprog_count++;
    m_n_macroprog_count++;

    if(this->m_p_general_config->debug_mode == 0){  //�ǵ���ģʽ�򵥶���Ч
        this->SetMcStepMode(false);
        //	this->m_b_need_change_to_pause = false;
    }
    return true;
}

/**
 * @brief ʵ��ִ�мӹ���λ���ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelControl::ExecuteRestartOverMsg(RecordMsg *msg){
    printf("ChannelControl::ExecuteRestartOverMsg\n");

    this->m_n_restart_mode = NOT_RESTART;
    this->m_n_restart_line = 0;
    this->m_n_restart_step = 0;
    this->m_p_compiler->SetRestartPara(0, m_n_restart_mode);   //��λ�������ļӹ���λ��־

    this->ClearHmiInfoMsg();   //���HMI����ʾ��Ϣ

    //	this->PauseRunGCode();    //FOR TEST  ��ͣ����۲�

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
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        return true;
    }

    //�ȴ�MC��������������
    int count = 0;
    int limited = 1;
    if(this->IsStepMode())  //����ģʽ��Ҫ��ȴ�һ�����ڣ�״̬ˢ����Ҫһ������
        limited = 4;
    while(count < limited ){
        if(m_simulate_mode == SIM_OUTLINE || m_simulate_mode == SIM_TOOLPATH)
            break;

        if(this->ReadMcMoveDataCount() > 0 ||
                // @test zk ���MDIִ�к���� ������������㿨������  ��ȷ���᲻�����������
                //	(!this->CheckStepOverFlag() && !this->CheckBlockOverFlag()) ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){
            bool flag1 = this->CheckStepOverFlag();
            bool flag2 = this->CheckBlockOverFlag();
            bool flag3 = (!this->CheckStepOverFlag() && !this->CheckBlockOverFlag());
            return false;    //��δ���е�λ
        }
        else{
            count++;
            //		printf("Macro cmd: step=%d, %d, c = %d\n", m_channel_mc_status.step_over,m_channel_mc_status.auto_block_over, count);
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
        }
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    //���õ�ǰ�к�
    SetCurLineNo(msg->GetLineNo());

    return true;
}

/**
 * @brief ִ���ӳ��򷵻���Ϣ
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteSubProgReturnMsg(RecordMsg *msg){
    SubProgReturnMsg *ret_msg = static_cast<SubProgReturnMsg *>(msg);

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        return true;
    }

    if(m_simulate_mode == SIM_OUTLINE && m_simulate_mode == SIM_TOOLPATH){  //�������棬��·����
        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());

        if(ret_msg->IsRetFromMacroProg() && m_n_macroprog_count > 0){  //����򷵻�
            m_n_macroprog_count--;
            m_b_ret_from_macroprog = true;
        }
        return true;
    }


    int limit = 1;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            //	printf("aux exec return: 0x%x\n", m_p_mc_comm->ReadRunOverValue());
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            printf("execute subprog return msg: blockflag=%d, count = %d\n", block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }


    //TODO ��HMI����������ϼ��ļ��ļ�
#ifdef USES_ADDITIONAL_PROGRAM
    if(this->m_channel_status.chn_work_mode == AUTO_MODE && m_n_add_prog_type == NONE_ADD &&
            (!ret_msg->IsRetFromMacroProg() || this->m_p_general_config->debug_mode > 0)){    //�Զ�ģʽ���������ӳ��򷵻ػ��ߵ���ģʽ��
#else
    if(this->m_channel_status.chn_work_mode == AUTO_MODE &&
            (!ret_msg->IsRetFromMacroProg() || this->m_p_general_config->debug_mode > 0)){    //�Զ�ģʽ���������ӳ��򷵻ػ��ߵ���ģʽ��
#endif
        char file[kMaxFileNameLen];
        memset(file, 0x00, kMaxFileNameLen);
        ret_msg->GetReturnFileName(file);
        this->SendOpenFileCmdToHmi(file);

        printf("execute sub program return msg: file = %s\n", file);
    }

    //���õ�ǰ�к�
    if(ret_msg->IsRetFromMacroProg() && m_n_macroprog_count > 0){  //����򷵻أ��������к�
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
 * @brief ִ�м�����岹��ĥ������Ϣ
 * @param msg
 * @return
 */
bool ChannelControl::ExecutePolarIntpMsg(RecordMsg *msg){
    PolarIntpMsg *polarmsg = (PolarIntpMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        int cmd = polarmsg->GetGCode();
        this->m_mode_restart.gmode[GetModeGroup(cmd)] = cmd;
        return true;
    }

    if(m_simulate_mode == SIM_OUTLINE && m_simulate_mode == SIM_TOOLPATH){  //�������棬��·����
        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());

        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            //		printf("PolarIntpMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            //		printf("execute PolarIntpMsg[%d]: blockflag=%d, count = %d, step = %hhu\n", polarmsg->GetGCode(),block_over, count, polarmsg->GetExecStep());

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }


    //���õ�ǰ�к�
    SetCurLineNo(polarmsg->GetLineNo());


    //�л�����ֵ������ģ̬
    int type = polarmsg->GetGCode();
    uint16_t intp_type = type;  //�岹��ʽ

    if(intp_type == G13_1_CMD)
        intp_type = 0;


    //	else if(type == G12_2_CMD){//����ĥ��״̬�� �ȸ��µ�MCģ̬��Ϣ

    switch(polarmsg->GetExecStep()){
    case 0://��һ�������²岹��ʽ���͵�MC
        this->m_channel_status.gmode[21] = type;

#ifdef USES_GRIND_MACHINE
        if(type == G12_2_CMD){//ר��ĥ��G12.2
            //��MC����ĥ����̬����
            int radius = -1;
            int r_index = polarmsg->GetParamR();
            if(r_index >= 1 && r_index <= 6 )
                radius = this->m_p_grind_config->wheel_raidus_multi[r_index -1]*1000;  //��λת����mm->um
            this->SetMcGrindParamDynamic(intp_type, 0, polarmsg->GetParamP(), radius);  //����P��R����
            this->SetMcGrindParamDynamic(intp_type, 1, polarmsg->GetParamQ(), polarmsg->GetParamL());  //����L��Q����

            printf("G12.2 send param: type = %d, P=%d, R=%d, Q=%d, L=%d\n", intp_type, polarmsg->GetParamP(), radius,
                   polarmsg->GetParamQ(), polarmsg->GetParamL());
        }else if(type == G12_3_CMD){//ר��ĥ��G12.3
            //��MC����ĥ����̬����
            int radius = -1;
            int r_index = polarmsg->GetParamR();
            if(r_index >= 1 && r_index <= 6 )
                radius = this->m_p_grind_config->wheel_raidus_multi[r_index -1]*1000;  //��λת����mm->um
            this->SetMcGrindParamDynamic(intp_type, 0, polarmsg->GetParamX(), polarmsg->GetParamY());  //����X��Y����
            this->SetMcGrindParamDynamic(intp_type, 1, polarmsg->GetParamI(), polarmsg->GetParamJ());  //����I��J����
            this->SetMcGrindParamDynamic(intp_type, 2, polarmsg->GetParamP(), radius);  //����P��R����
            this->SetMcGrindParamDynamic(intp_type, 3, polarmsg->GetParamQ(), polarmsg->GetParamL());  //����L��Q����

            printf("G12.3 send param: type = %d, X=%d, Y=%d, I=%d, J=%d, P=%d, R=%d, Q=%d, L=%d\n", intp_type, polarmsg->GetParamX(),
                   polarmsg->GetParamY(), polarmsg->GetParamI(), polarmsg->GetParamJ(), polarmsg->GetParamP(), radius,
                   polarmsg->GetParamQ(), polarmsg->GetParamL());
        }
#endif
        //��MC��������ĥ��״ָ̬��
        this->SetMcSpecialIntpMode(intp_type);

        this->SendChnStatusChangeCmdToHmi(G_MODE);

        polarmsg->SetExecStep(1);	//��ת��һ��
        return false;
    case 1://�ڶ���
        //�ڶ������ȴ�MCִ��ĥ���������
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadChnCurIntpMode(this->m_n_channel_index, intp_type);  //��ȡMC��ǰ����岹ģʽ
        else
            this->m_p_mc_arm_comm->ReadChnCurIntpMode(m_n_channel_index, intp_type);
        if(type == G13_1_CMD){
            if(intp_type == CANCEL_MODE)   //ȡ������岹ģʽ
                polarmsg->SetExecStep(2);  //��ת��һ��
        }
#ifdef USES_GRIND_MACHINE
        else if(type == G12_2_CMD || type == G12_3_CMD){//ר��ĥ��
            if(intp_type == G53_1_MODE){
                polarmsg->SetExecStep(2);  //��ת��һ��
            }
        }
#endif
        return false;
    case 2:
        //��������ͬ��λ��
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        else
            this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        this->RefreshAxisIntpPos();

        this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��

        polarmsg->SetExecStep(3);  //��ת��һ��
        return false;
    case 3:
        //���Ĳ�������

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
 * @brief ִ�вο��㷵����Ϣ
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
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
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

        //���õ�ǰ�к�
        SetCurLineNo(refmsg->GetLineNo());

        //		int gcode = refmsg->GetGCode();
        uint32_t axis_mask = refmsg->GetAxisMask();
        DPointChn &mid_pos = refmsg->GetMiddlePos();

        SimulateData data_mid, data_tar;
        MonitorDataType type = MDT_CHN_SIM_GRAPH;  //��������

        if(m_simulate_mode == SIM_TOOLPATH)
            type = MDT_CHN_SIM_TOOL_PATH;

        //���Ͳο�������
        uint8_t chn_axis = this->GetChnAxisFromName(AXIS_NAME_X);
        if(chn_axis != 0xFF && (axis_mask & (0x01<<chn_axis))){
            data_mid.pos[0] = 	mid_pos.m_df_point[chn_axis];   //�м��λ����
            this->TransWorkCoordToMachCoord(data_mid.pos[0], m_channel_status.gmode[14], chn_axis);  //��������ת��Ϊ��е����

            data_tar.pos[0] = m_p_axis_config[this->GetPhyAxis(chn_axis)].axis_home_pos[0];   //��е����
            m_pos_simulate_cur_work.SetAxisValue(chn_axis, data_tar.pos[0]);   //������浱ǰλ��
            this->TransMachCoordToWorkCoord(m_pos_simulate_cur_work.m_df_point[chn_axis], this->m_channel_status.gmode[14], chn_axis);   //ת��Ϊ��������ϵ
        }
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Y);
        if(chn_axis != 0xFF && (axis_mask & (0x01<<chn_axis))){
            data_mid.pos[1] = 	mid_pos.m_df_point[chn_axis];   //�м��λ����
            this->TransWorkCoordToMachCoord(data_mid.pos[1], m_channel_status.gmode[14], chn_axis);  //��������ת��Ϊ��е����

            data_tar.pos[1] = m_p_axis_config[this->GetPhyAxis(chn_axis)].axis_home_pos[0];   //��е����
            m_pos_simulate_cur_work.SetAxisValue(chn_axis, data_tar.pos[1]);   //������浱ǰλ��
            this->TransMachCoordToWorkCoord(m_pos_simulate_cur_work.m_df_point[chn_axis], this->m_channel_status.gmode[14], chn_axis);   //ת��Ϊ��������ϵ
        }
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
        if(chn_axis != 0xFF && (axis_mask & (0x01<<chn_axis))){
            data_mid.pos[2] = 	mid_pos.m_df_point[chn_axis];   //�м��λ����
            this->TransWorkCoordToMachCoord(data_mid.pos[2], m_channel_status.gmode[14], chn_axis);  //��������ת��Ϊ��е����

            data_tar.pos[2] = m_p_axis_config[this->GetPhyAxis(chn_axis)].axis_home_pos[0];   //��е����
            m_pos_simulate_cur_work.SetAxisValue(chn_axis, data_tar.pos[2]);   //������浱ǰλ��
            this->TransMachCoordToWorkCoord(m_pos_simulate_cur_work.m_df_point[chn_axis], this->m_channel_status.gmode[14], chn_axis);   //ת��Ϊ��������ϵ
        }
        this->SendSimulateDataToHmi(type, data_mid);   //�����м��
        this->SendSimulateDataToHmi(type, data_tar);   //�����յ�

        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){
        if(m_simulate_mode == SIM_OUTLINE || m_simulate_mode == SIM_TOOLPATH)
            break;

        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            //			printf("RefReturnMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            //		printf("execute RefReturnMsg[%d]: blockflag=%d, count = %d\n", refmsg->GetGCode(),block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    //���õ�ǰ�к�
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
        //  �ο�����ų�����Χ
        printf("G30 specify a refpoint not exist ��\n");
        CreateError(ERR_NO_CUR_RUN_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, gcode, m_n_channel_index);
        return false;
    }

    if(gcode == G28_CMD or gcode == G30_CMD){
        switch(refmsg->GetExecStep()){
        case 0:

        	//��һ�������ƶ�Ӧ�����ߵ��м��λ��
            for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                if(axis_mask & (0x01<<i)){
                    phy_axis = this->GetPhyAxis(i);
                    if(phy_axis != 0xff){
                        this->ManualMove(i, pos[i], m_p_axis_config[phy_axis].rapid_speed, true);  //��������ϵ
                        printf("manual move axis=%hhu, target=%lf, speed=%lf\n", i, pos[i], m_p_axis_config[phy_axis].rapid_speed);
                        gettimeofday(&m_time_ret, NULL);
                    }

                }
            }

            refmsg->SetExecStep(1);	//��ת��һ��
            return false;
        case 1:
        {
            struct timeval time_now;
            gettimeofday(&time_now, NULL);
            unsigned int time_elpase = (time_now.tv_sec-m_time_ret.tv_sec)*1000000+time_now.tv_usec-m_time_ret.tv_usec;
            if (time_elpase >= 50000) {//�ȴ�MCˢ��axis_over_mask���
                //�ڶ������ȴ���Ӧ�ᵽλ
                for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                    if(axis_mask & (0x01<<i)){
                        //if(fabs(this->m_channel_rt_status.cur_pos_work.GetAxisValue(i) - pos[i]) > 1e-3){ //δ��λ
                        if ((m_channel_mc_status.manu_axis_over_mask & (0x01<<i)) == 0) {
                            //                        printf("step 1: axis %hhu cur pos = %lf, target=%lf\n", i, m_channel_rt_status.cur_pos_work.GetAxisValue(i), pos[i]);

                            flag = false;
                            //phy_axis = this->GetPhyAxis(i);
                            //if(phy_axis != 0xff){
                            //    this->ManualMove(i, pos[i], m_p_axis_config[phy_axis].rapid_speed, true);  //��������ϵ

                            //}
                            //    printf("cur work pos = %lf, tar pos = %lf\n", m_channel_rt_status.cur_pos_work.GetAxisPos(i), pos[i]);

                        }
                    }
                }
                if(flag)
                    refmsg->SetExecStep(2);  //��ת��һ��
            }
            return false;
        }
        case 2:

            printf("ret ref, step 2\n");
            //�����������ƶ�Ӧ�����ߵ���е���
            if(gcode == G28_CMD){
                for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                    if(axis_mask & (0x01<<i)){
                        phy_axis = this->GetPhyAxis(i);
                        if(phy_axis != 0xff){
#ifdef USES_RET_REF_TO_MACH_ZERO
                            this->ManualMove(i, 0, m_p_axis_config[phy_axis].rapid_speed);  //��е����ϵ
                            printf("manual move machpos axis=%hhu, target=0, speed=%lf\n", i,	m_p_axis_config[phy_axis].rapid_speed);
#else
                            this->ManualMove(i, m_p_axis_config[phy_axis].axis_home_pos[0], m_p_axis_config[phy_axis].rapid_speed);  //��е����ϵ
                            printf("manual move machpos axis=%hhu, target=%lf, speed=%lf\n", i, m_p_axis_config[phy_axis].axis_home_pos[0],
                                    m_p_axis_config[phy_axis].rapid_speed);
#endif
                        }
                    }
                }
                refmsg->SetExecStep(3);  //��ת��һ��
                gettimeofday(&m_time_ret, NULL);
                return false;

            }else{
                // G30
                for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                    if(axis_mask & (0x01<<i)){
                        phy_axis = this->GetPhyAxis(i);
                        if(phy_axis != 0xff){
                            this->ManualMove(i, m_p_axis_config[phy_axis].axis_home_pos[ref_id-1], m_p_axis_config[phy_axis].rapid_speed);  //��е����ϵ
                        }
                    }
                }
                refmsg->SetExecStep(3);  //��ת��һ��
                gettimeofday(&m_time_ret, NULL);
                return false;
            }

        case 3:{
            struct timeval time_now;
            gettimeofday(&time_now, NULL);
            unsigned int time_elpase = (time_now.tv_sec-m_time_ret.tv_sec)*1000000+time_now.tv_usec-m_time_ret.tv_usec;
            if (time_elpase > 50000) {
                //���Ĳ����ȴ�struct timeval time_now;
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
                        //if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(i) - target_pos) > 5e-3){ //δ��λ
                            //printf("step 3: axis %hhu cur pos = %lf, target=%lf\n", i, m_channel_rt_status.cur_pos_machine.GetAxisValue(i), target_pos);
                            flag = false;
                            //phy_axis = this->GetPhyAxis(i);
                            //if(phy_axis != 0xff){
                                //printf("phy_axis: %d target_pos: %lf\n", phy_axis, target_pos);
                             //   this->ManualMove(i, target_pos, m_p_axis_config[phy_axis].rapid_speed);  //��е����ϵ
                            //}
                        }
                    }
                }

                if(flag){
                    refmsg->SetExecStep(4);  //��ת��һ��
                    //printf("ret ref, jump to step 4\n");
                }
            }
            return false;
        }
        case 4:
            //���岽��ͬ��λ��
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
                //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //ͬ���ѱ�������ƶ�ָ���λ��

            break;

        default:
            printf("ref return msg execute error!\n");
            break;
        }
    }else if(gcode == G27_CMD){
        switch(refmsg->GetExecStep()){
        case 0:
            printf("G27 step 0 ...\n");
            // �ƶ��� G27 ָ����
            for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                if(axis_mask & (0x01<<i)){
                    phy_axis = this->GetPhyAxis(i);
                    if(phy_axis != 0xff){
                        this->ManualMove(i, pos[i], m_p_axis_config[phy_axis].rapid_speed, true);  //��������ϵ
                    }
                }
            }
            refmsg->SetExecStep(1);
            return false;
        case 1:
            printf("G27 step 1 ...\n");
            //�ڶ������ȴ���Ӧ�ᵽλ
            for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                if(axis_mask & (0x01<<i)){
                    //printf("axis %hhu abs %lf\n", fabs(this->m_channel_rt_status.cur_pos_work.GetAxisValue(i) - pos[i]));
                    if(fabs(this->m_channel_rt_status.cur_pos_work.GetAxisValue(i) - pos[i]) > 0.001){ //δ��λ
                        //printf("step 1: axis %hhu cur pos = %lf, target=%lf\n", i, m_channel_rt_status.cur_pos_work.GetAxisValue(i), pos[i]);
                        flag = false;
                        phy_axis = this->GetPhyAxis(i);
                        if(phy_axis != 0xff){
                            this->ManualMove(i, pos[i], m_p_axis_config[phy_axis].rapid_speed, true);  //��������ϵ
                        }
                        //	printf("cur work pos = %lf, tar pos = %lf\n", m_channel_rt_status.cur_pos_work.GetAxisPos(i), pos[i]);
                    }

                }
            }
            if(flag)
                refmsg->SetExecStep(2);  //��ת��һ��
            return false;
        case 2:
        {
            //���������ж��Ƿ�Ϊ�ο���
            // �����и��ֲ�����  �ü����� ��Ȼ���뱨�� error: jump to case label
            bool in_pos = false;

            // ���ÿ���ο��㵽λ�ź�  ��Ϊ��е����ϵ����ʱ���� ����Ҫ���⼸��
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
                // û�е�λ
                wait_times ++;

                if(wait_times > 10){
                    // ʮ�μ�ⶼû��λ  �ж�Ϊ�زο���ʧ�� ��������
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
            //���Ĳ���ͬ��λ��
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            this->RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //ͬ���ѱ�������ƶ�ָ���λ��
            break;
        }
    }

    m_n_run_thread_state = RUN;

    printf("execute ref return msg\n");

    return true;
}

/**
 * @brief ִ����ת��Ϣ
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteSkipMsg(RecordMsg *msg){
    SkipMsg *skipmsg = (SkipMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ

        return true;
    }

    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){  //�������棬��·����

        //���õ�ǰ�к�
        SetCurLineNo(skipmsg->GetLineNo());

        if(!OutputData(msg))
            return false;

        m_n_run_thread_state = RUN;

        return true;
    }

    if(skipmsg->GetExecStep() == 0){  //��ʼִ�еĽ׶β���Ҫ�ȴ�MCִ�����
        int limit = 2;
        if(this->IsStepMode())
            limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

        //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
        int count = 0;
        while(1){
            bool block_over = CheckBlockOverFlag();
            if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
                //	printf("RefReturnMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
                return false;    //��δ���е�λ
            }
            else if(++count < limit){
                usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
                //		printf("execute RefReturnMsg[%d]: blockflag=%d, count = %d\n", refmsg->GetGCode(),block_over, count);

            }else
                break;
        }
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }


    //���õ�ǰ�к�
    SetCurLineNo(skipmsg->GetLineNo());

    uint32_t axis_mask = skipmsg->GetAxisMoveMask();


    switch(skipmsg->GetExecStep()){
    case 0:{
        //��һ����֪ͨMI��ʼץȡSKIP�źţ�����Ӧ�����־
        this->m_b_pos_captured = false;
        this->m_n_mask_pos_capture = axis_mask;
        this->m_n_mask_captured = 0;

        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.cmd = CMD_MI_ACTIVE_SKIP;
        cmd.data.reserved = 0;   //����
        cmd.data.data[0] = this->m_p_channel_config->g31_skip_signal/10;  //��ת�źŶκ�
        cmd.data.data[1] = this->m_p_channel_config->g31_skip_signal%10;  // ��ת�ź�λ��
        memcpy(&cmd.data.data[2], &axis_mask, sizeof(uint32_t));           //��mask
        cmd.data.data[4] = this->m_n_channel_index+1;   //MI��ͨ���Ŵ�1��ʼ
        cmd.data.data[5] = this->m_p_channel_config->g31_sig_level;    //��ת�ź���Ч��ƽ
        this->m_p_mi_comm->WriteCmd(cmd);

        printf("send skip cmd to mi, signal:%u, data[0]=%hu, data1=%hu, level=%hu\n", m_p_channel_config->g31_skip_signal, cmd.data.data[0], cmd.data.data[1], cmd.data.data[5]);

        skipmsg->SetExecStep(1);	//��ת��һ��
        return false;
    }
    case 1:
        //�ڶ��������˶����ݷ�����MC��������
        if(!m_b_pos_captured){  //��δ�����źţ���ֹ����MI�Ȳ����źţ�����MC�����˶��������
            if(!OutputData(msg))
                return false;

            this->StartMcIntepolate();   //����MCִ��
            printf("@@@@Send g31 move cmd to mc\n");
        }

        skipmsg->SetExecStep(2);  //��ת��һ��
        return false;
    case 2:{
        //���������ȴ�MI����������MC���е�λ
        if(m_b_pos_captured){//MIδ�����ź�

            //��MC����ֹͣ����
            if(!m_b_mc_need_start)  //�Ѿ�������MC��������ŷ���G31ֹͣ
                this->SendMcG31Stop();
            //				printf("send G31 stop to mc\n");

            printf("skip cmd cathe signal, send G31 stop\n");

            skipmsg->SetExecStep(3);  //��ת��һ��
            return false;  //�����ȴ�
        }else if(CheckBlockOverFlag() && ReadMcMoveDataCount() == 0){//MC�鵽λ���һ���������
            //���͹ر�G31ָ������
            MiCmdFrame cmd;
            memset(&cmd, 0x00, sizeof(cmd));
            cmd.data.cmd = CMD_MI_ACTIVE_SKIP;
            cmd.data.reserved = 1;   //�ر�
            cmd.data.data[0] = this->m_p_channel_config->g31_skip_signal/10;  //��ת�źŶκ�
            cmd.data.data[1] = this->m_p_channel_config->g31_skip_signal%10;  // ��ת�ź�λ��
            memcpy(&cmd.data.data[2], &axis_mask, sizeof(uint32_t));           //��mask
            cmd.data.data[4] = this->m_n_channel_index+1;   //MI��ͨ���Ŵ�1��ʼ
            cmd.data.data[5] = this->m_p_channel_config->g31_sig_level;    //��ת�ź���Ч��ƽ
            this->m_p_mi_comm->WriteCmd(cmd);

            printf("skip cmd catch no signal!\n");

            skipmsg->SetExecStep(4);   //��ת���Ĳ�
            return false;
        }

        return false;
    }
    case 3:
        //���Ĳ����ȴ���Ӧ���ᵽλ,ͬ��λ��
        if(CheckBlockOverFlag() && ReadMcMoveDataCount() == 0){//MC�鵽λ���һ���������
            //��MCͬ����������
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //				m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //				m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();

            this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��

            this->m_p_channel_engine->SendMonitorData(false, false);
        }else{
            //		printf("step3: block over=%hhu, datacount=%hu\n", CheckBlockOverFlag(), ReadMcMoveDataCount());
            return false;
        }

        printf("goto step 4\n");
        skipmsg->SetExecStep(4);  //��ת��һ��
        return false;
    case 4:

        skipmsg->SetFlag(FLAG_AXIS_MOVE, false);//�ر����ƶ����ԣ���ֹ�����ٷ���MC��������

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
 * @brief ִ���ٶȿ�����Ϣ
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteSpeedCtrlMsg(RecordMsg *msg){
    SpeedCtrlMsg *speedmsg = (SpeedCtrlMsg *)msg;
    printf("--IN--ChannelControl:: ExecuteSpeedCtrlMsg \n");

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ

        //TODO  �˴���Ҫ��Ӻ���������

        return true;
    }

    if(this->m_simulate_mode == SIM_OUTLINE || this->m_simulate_mode == SIM_TOOLPATH){  //�������棬��·����

        //���õ�ǰ�к�
        SetCurLineNo(speedmsg->GetLineNo());

        m_n_run_thread_state = RUN;

        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            printf("ExecuteSpeedCtrlMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            //		printf("execute RefReturnMsg[%d]: blockflag=%d, count = %d\n", refmsg->GetGCode(),block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    //���õ�ǰ�к�
    SetCurLineNo(speedmsg->GetLineNo());

    int gcode = speedmsg->GetGCode();
    uint32_t axis_mask = 0;    //ʵ���ƶ���mask
    uint8_t axis_count = 0;    //ʵ���ƶ�����
    double *speed = speedmsg->GetTargetValue(axis_count, axis_mask);


    uint8_t unit_flag = 0;  // mm/min
    if(gcode == G1000_CMD|| gcode == G1002_CMD)
    {
        unit_flag = 1;  //  rpm--> mm/min
    }
    uint8_t check_flag = 0;  //
    if(gcode == G1002_CMD|| gcode == G1003_CMD)
    {
        check_flag = 1;  //  rpm--> mm/min  ����־
    }

    uint8_t phy_axis = 0;
    uint8_t i = 0, acount = 0;
    bool flag = true;
    if(gcode == G1000_CMD || gcode == G1001_CMD || gcode == G1002_CMD|| gcode == G1003_CMD ){
        switch(speedmsg->GetExecStep()){
        case 0:
            //��һ�������ٶȿ������ݷ��͵�MI
            for(i = 0; i < m_p_channel_config->chn_axis_count && acount < axis_count; i++){
                if(axis_mask & (0x01<<i)){
                    if (this->m_channel_status.cur_axis_ctrl_mode[i] == 2){   // �жϸ���Ϊ�ٶȿ�����
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
                speedmsg->SetExecStep(2);	//��ת��һ��
            else
                speedmsg->SetExecStep(1);	//��ת��һ��
            return false;
        case 1:
            //�ڶ������ȴ���Ӧ�ᵽλ
            this->m_p_channel_engine->SendMonitorData(false, false);
            for(i = 0; i < m_p_channel_config->chn_axis_count && acount < axis_count; i++){
                if (this->m_channel_status.cur_axis_ctrl_mode[i] == 2){   // �жϸ���Ϊ�ٶȿ�����
                    if(axis_mask & (0x01<<i)){
                        double feed = speed[acount];
                        if(unit_flag==1){
                            feed *= m_p_axis_config[phy_axis].move_pr;
                        }
                        feed = feed*1000/60;
                        double errspeed = fabs(feed)*0.1;
                        if(errspeed<3) errspeed = 3;
                        double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(i);
                        if(fabs(curfeed - feed) > errspeed){ //�ٶ�δ����
                            flag = false;
                        }
                        printf("axis: %hhu cur feed = %lf, feed = %lf\n", i, curfeed, feed);
                    }
                }
            }
            if(flag)
                speedmsg->SetExecStep(2);  //��ת��һ��
            return false;
        case 2:
            //��������ͬ��λ��
            if(!this->m_b_mc_on_arm)
                this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
            else
                this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
                //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //ͬ���ѱ�������ƶ�ָ���λ��
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
 * @brief ִ�����ؿ�����Ϣ
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteTorqueCtrlMsg(RecordMsg *msg){
    TorqueCtrlMsg *torquemsg = (TorqueCtrlMsg *)msg;

    //     printf("--IN--ChannelControl:: ExecuteTorqueCtrlMsg \n");

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ

        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            printf("ExecuteTorqueCtrlMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            //		printf("execute ExecuteTorqueCtrlMsg[%d]: blockflag=%d, count = %d\n", refmsg->GetGCode(),block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    //���õ�ǰ�к�
    SetCurLineNo(torquemsg->GetLineNo());

    int gcode = torquemsg->GetGCode();
    uint32_t axis_mask = torquemsg->GetAxisMoveMask();  //ʵ���ƶ���mask
    uint8_t axis_count = 0;   //ʵ���ƶ�����

    double *torque = torquemsg->GetTargetValue(axis_count, axis_mask);

    int16_t speed_lmt = torquemsg->GetSpeedLmtValue();
    uint32_t timeover = 0;   //��ʱʱ�䣬0��ʾ�޳�ʱ
    struct timeval *start_time = torquemsg->GetStartTimePtr();


    uint8_t check_flag = 1;  // mm/min
    if(gcode == G2000_CMD)
    {
        check_flag = 0;  //  rpm--> mm/min
    }else if(gcode == G2001_CMD || gcode == G2002_CMD || gcode == G2003_CMD){
        timeover = torquemsg->GetTimeoverValue()*1000;   //��λ��msת��Ϊus
    }

    //    printf("execute ExecuteTorqueCtrlMsg: gcode=%d, axis_mask = %d  speed_lmt=%d, timeover=%u \n",(uint32_t)gcode,axis_mask, (uint32_t)speed_lmt, timeover);
    //    printf("execute ExecuteTorqueCtrlMsg: torque_value =%f  %f  %f  %f  \n",torque[0],torque[1],torque[2],torque[3]);

    uint8_t phy_axis = 0;
    uint8_t i = 0;
    bool flag = true;
    //	if(gcode == G2000_CMD || gcode == G2001_CMD ){
    switch(torquemsg->GetExecStep()){
    case 0:
        //��һ�������ƶ�Ӧ�����ߵ��м��λ��
        for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
            if (this->m_channel_status.cur_axis_ctrl_mode[i] == 3){   // �жϸ���Ϊ���ؿ�����
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
            torquemsg->SetExecStep(2);	//��ת��һ��
        }else{
            torquemsg->ResetCheckCount();
            torquemsg->SetExecStep(1);	//��ת��һ��
            if(timeover > 0){
                gettimeofday(start_time, nullptr);   //��¼��ʼʱ��

            }

        }
        return false;
    case 1:
        //�ڶ������ȴ���Ӧ�ᵽλ
        this->m_p_channel_engine->SendMonitorData(false, false);
        for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
            if (this->m_channel_status.cur_axis_ctrl_mode[i] == 3){   // �жϸ���Ϊ���ؿ�����
                if(axis_mask & (0x01<<i)){

                    int16_t tor = torque[i];
                    int16_t err = abs(tor)/10;
                    if(err < 10)
                        err = 10;
                    if(gcode == G2002_CMD || gcode == G2003_CMD){  //�����ص���Ϊ��������
                        int16_t cur_tor = this->m_channel_rt_status.cur_feedbck_torque.GetAxisValue(i);
                        if(abs(cur_tor) < (abs(tor)-err)){ //δ��ָ������ֵ
                            flag = false;
                        }
                        printf("axis: %hhu cur curTor = %hd, torque = %hd\n", i, cur_tor, tor);
                    }else if(gcode == G2001_CMD){ //��ת�ٵ�0Ϊ��������
                        double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(i);
                        if(fabs(curfeed) > 2) { //δ��λ
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
                torquemsg->SetExecStep(2);  //��ת��һ��
        }
        else{
            torquemsg->ResetCheckCount();

            if(timeover > 0){//��ʱ�ж�
                struct timeval time_now;
                gettimeofday(&time_now, nullptr);
                unsigned int time_elpase = (time_now.tv_sec-start_time->tv_sec)*1000000+time_now.tv_usec-start_time->tv_usec;

                if(time_elpase >= timeover){ //��ʱ����
                    printf("execute torque timeover\n");
                    //��ֹͣ�˶�
                    for(i = 0; i < m_p_channel_config->chn_axis_count; i++){
                        if (this->m_channel_status.cur_axis_ctrl_mode[i] == 3){   // �жϸ���Ϊ���ؿ�����
                            if(axis_mask & (0x01<<i)){
                                phy_axis = this->GetPhyAxis(i);
                                if(phy_axis != 0xff){
                                    this->SendAxisTorqueCmdToMi(phy_axis, 0, 0,0);
                                }
                            }
                        }
                    }

                    if(gcode == G2003_CMD){ //G2003��ʱ����Ҫ�澯����������
                        torquemsg->SetExecStep(2);  //��ת��һ��
                    }else{
                        torquemsg->SetExecStep(10);  //��ת����ʱ������
                    }
                }else{
                    printf("time_elpase:%u, timeover=%u\n", time_elpase, timeover);
                    //					printf("start time : %d, %d\n", start_time->tv_sec, start_time->tv_usec);
                }
            }
        }
        return false;
    case 2:
        //��������ͬ��λ��
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        else
            this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        this->RefreshAxisIntpPos();

        if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
            //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
            this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
        }else
            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //ͬ���ѱ�������ƶ�ָ���λ��
        break;

    case 10:
        //��ʱ����, ��ͬ��λ�������ɸ澯
        //ͬ��λ��
        this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        this->RefreshAxisIntpPos();

        if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
            //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
            this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
        }else
            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //ͬ���ѱ�������ƶ�ָ���λ��

        //���ɸ澯
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
 * @brief ִ����ʱ��Ϣ
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteTimeWaitMsg(RecordMsg *msg){
    TimeWaitMsg *timemsg = (TimeWaitMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        return true;
    }

    if(this->m_simulate_mode != SIM_NONE){  //����ģʽ
        //���õ�ǰ�к�
        SetCurLineNo(timemsg->GetLineNo());
        m_n_run_thread_state = RUN;
        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            //		printf("TimeWaitMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            //		printf("execute TimeWaitMsg[%d]: blockflag=%d, count = %d, time = %hhu\n", timemsg->GetGCode(),block_over, count, timemsg->GetTimeCount());

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }


    //���õ�ǰ�к�
    SetCurLineNo(timemsg->GetLineNo());

    uint32_t time = timemsg->GetDelayTime();

    uint64_t cur_time = 0;
    if(time > 0){

        struct timeval tvNow;
        gettimeofday(&tvNow, NULL);
        cur_time = tvNow.tv_sec*1000+tvNow.tv_usec/1000;  //ms��λ
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
 * @brief ִ��������Ȧ��Ϣ
 * @param msg
 * @return
 */
bool ChannelControl::ExecuteClearCirclePosMsg(RecordMsg *msg){
    ClearCirclePosMsg *clearmsg = (ClearCirclePosMsg *)msg;

    if(this->m_n_restart_mode != NOT_RESTART &&
            clearmsg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        return true;
    }

    if(this->m_simulate_mode != SIM_NONE && this->m_simulate_mode != SIM_MACHINING){  //����ģʽ,�ҷǼӹ����棬ֱ�ӷ���
        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());

        //����������Ȧ
        uint64_t mask = clearmsg->GetAxisMask();
        double mode = clearmsg->GetCircleMode();
        mode /= 1000;   //��umת��Ϊmm
        //ScPrintf("ClearCirclePosMsg: mask = %04llx mode=%lf", mask, mode);
        for(int i = 0; i < m_p_general_config->axis_count; i++){
            if(((mask>>i) & 0x01) == 0){
                continue;
            }
            int tt = m_pos_simulate_cur_work.m_df_point[this->GetChnAxisFromPhyAxis(i)] / mode;
            m_pos_simulate_cur_work.m_df_point[this->GetChnAxisFromPhyAxis(i)] -= tt * mode;  //���µ�ǰ����λ��
        }

        this->m_p_compiler->SetCurPos(m_pos_simulate_cur_work);   //ͬ��������λ��
        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ
    else if(clearmsg->GetExecStep() == 0){
        limit = 40;   //��ʱ200ms
    }

    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){
        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            //		printf("PolarIntpMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            //		printf("execute ClearCirclePosMsg[%d]: blockflag=%d, count = %d, step = %hhu\n", clearmsg->GetGCode(),block_over, count, clearmsg->GetExecStep());

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }


    //���õ�ǰ�к�
    SetCurLineNo(clearmsg->GetLineNo());


    uint8_t i = 0;
    uint64_t mask = clearmsg->GetAxisMask();
    switch(clearmsg->GetExecStep()){
    case 0://��һ��������ָ�MI
        this->m_channel_status.gmode[39] = clearmsg->GetGCode();

        //ScPrintf("clear pos message , mask = 0x%llx, mode=%d\n", mask, clearmsg->GetCircleMode());
        //��MI����������Ȧλ��ָ��
        m_n_mask_clear_pos = 0;
        for(i = 0; i < m_p_general_config->axis_count; i++){
            if(((mask>>i) & 0x01) == 0){
                continue;
            }
            this->SendMiClearPosCmd(i+1, clearmsg->GetCircleMode());
        }

        this->SendChnStatusChangeCmdToHmi(G_MODE);

        clearmsg->SetExecStep(1);	//��ת��һ��
        return false;
    case 1://�ڶ���
        //ScPrintf("clear pos message step1 , mask = 0x%llx, m_n_mask_clear_pos = 0x%llx \n", mask, this->m_n_mask_clear_pos);
        //�ڶ������ȴ�MI�������
        if((mask & this->m_n_mask_clear_pos) == mask){  // if(mask == this->m_n_mask_clear_pos){
            clearmsg->SetExecStep(2);	//��ת��һ��
        }

        return false;
    case 2:
        //��������ͬ��λ��

        //��MCͬ����������
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        else
            this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        //		m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //		m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        this->RefreshAxisIntpPos();

        this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��

        //		printf("intp pos (%lf, %lf, %lf, %lf)\n", m_channel_mc_status.intp_pos.x, m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z,
        //				m_channel_mc_status.intp_pos.a6);

        //��MIͬ����е����
        this->m_p_channel_engine->SendMonitorData(false, false);

        //		printf("cur mach pos (%lf, %lf, %lf, %lf)\n", m_channel_rt_status.cur_pos_machine.x, m_channel_rt_status.cur_pos_machine.y,
        //				m_channel_rt_status.cur_pos_machine.z, m_channel_rt_status.cur_pos_machine.a6);

        clearmsg->SetExecStep(3);  //��ת��һ��
        return false;
    case 3:
        //���Ĳ�������

        break;
    default:
        printf("G200 execute error!\n");
        break;
    }

    m_n_run_thread_state = RUN;

    printf("execute clearpos message\n");

    return true;
}

// @add zk ���� G10 ��������ָ��
bool ChannelControl::ExecuteInputMsg(RecordMsg * msg){

    InputMsg * input_msg = (InputMsg *)msg;

    bool isAbs = this->m_channel_status.gmode[3] == 900 ? true: false;

    printf("ldata: %lf -- pdata: %lf --- rdata: %lf\n", input_msg->LData, input_msg->PData, input_msg->RData);

    if(this->m_n_restart_mode != NOT_RESTART &&
            input_msg->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        return true;
    }

    int count = 0;
    while(count < 4 ){
        if(this->ReadMcMoveDataCount() > 0 || !this->CheckBlockOverFlag() ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){
            return false;    //��δ���е�λ
        }
        else{
            count++;
            //			printf("ExecuteModeMsg: step=%d, %d, c = %d\n", m_channel_mc_status.step_over,m_channel_mc_status.auto_block_over, count);
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
        }
    }

    //����ģʽ
    if(this->IsStepMode()){
        if(this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }

        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());

    }else{//�ǵ���ģʽ
        if(m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING)
            return false;
    }

    int input_type = input_msg->LData;
    int tool_number = input_msg->PData;
    int plane = this->m_mc_mode_exec.bits.mode_g17;

    if(input_type < 20){
        if(tool_number <= 0 or tool_number > kMaxToolCount){
            // @TODO ���ߺų�����Χ
            CreateError(ERR_NO_CUR_RUN_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
            this->m_error_code = ERR_NO_CUR_RUN_DATA;
            return false;
        }
    }

    printf("input type %d\n", input_type);

    switch(input_type){
    case 10:{
        // XYƽ�� �������� Z   TODO ����ƽ�������� ������������ͬ�����
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
        this->NotifyHmiToolOffsetChanged(tool_number);   //֪ͨHMI��ƫֵ����
        break;
    }
    case 11:{

    	if(isAbs)
    		this->m_p_chn_tool_config->radius_compensation[tool_number-1] = input_msg->RData;
    	else
    		this->m_p_chn_tool_config->radius_compensation[tool_number-1] += input_msg->RData;

    	double comp_data = this->m_p_chn_tool_config->radius_compensation[tool_number-1];

        g_ptr_parm_manager->UpdateToolRadiusGeo(this->m_n_channel_index, tool_number-1, comp_data);
        this->NotifyHmiToolOffsetChanged(tool_number);   //֪ͨHMI��ƫֵ����
        break;
    }
    case 12:{
    	if(isAbs)
    		this->m_p_chn_tool_config->geometry_wear[tool_number-1] = input_msg->RData;
    	else
    		this->m_p_chn_tool_config->geometry_wear[tool_number-1] += input_msg->RData;

    	double comp_data = this->m_p_chn_tool_config->geometry_wear[tool_number-1];

        g_ptr_parm_manager->UpdateToolWear(this->m_n_channel_index, tool_number-1, comp_data);
        this->NotifyHmiToolOffsetChanged(tool_number);   //֪ͨHMI��ƫֵ����
        break;
    }
    case 13:{
        if(isAbs)
        	this->m_p_chn_tool_config->radius_wear[tool_number-1] = input_msg->RData;
        else
        	this->m_p_chn_tool_config->radius_wear[tool_number-1] += input_msg->RData;

        double comp_data = this->m_p_chn_tool_config->radius_wear[tool_number-1];
        g_ptr_parm_manager->UpdateToolRadiusWear(this->m_n_channel_index, tool_number-1, comp_data);
        this->NotifyHmiToolOffsetChanged(tool_number);   //֪ͨHMI��ƫֵ����
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
    case 50:{  // �޸�ϵͳ����, ������Ч�Ĳ�����(��Щ���������Ļ��������)

    }
    default:{
        // @TODO ��֧�ֵ�Lָ��
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
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
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
        // @TODO ģ��ģʽ�¶���
        return true;
    }

    int limit = 2;
    if(this->IsStepMode())
        limit = 4;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

    //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
    int count = 0;
    while(1){
        if(m_simulate_mode == SIM_OUTLINE || m_simulate_mode == SIM_TOOLPATH)
            break;

        bool block_over = CheckBlockOverFlag();
        if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                m_channel_status.machining_state == MS_PAUSED ||
                m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
            //			printf("RefReturnMsg exec return: %d, %d , %d\n", block_over, ReadMcMoveDataCount(), m_channel_status.machining_state);
            return false;    //��δ���е�λ
        }
        else if(++count < limit){
            usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
            //		printf("execute RefReturnMsg[%d]: blockflag=%d, count = %d\n", refmsg->GetGCode(),block_over, count);

        }else
            break;
    }

    if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
        this->m_b_need_change_to_pause = false;
        m_n_run_thread_state = PAUSE;
        SetMachineState(MS_PAUSED);
        return false;
    }

    //���õ�ǰ�к�
    SetCurLineNo(exact_msg->GetLineNo());

    double * pos = exact_msg->m_point_target.m_df_point;
    uint32_t axis_mask = exact_msg->GetAxisMask();
    uint8_t phy_axis = 0;
    bool flag = true;
    // �ȴ����ᵽλ
    for(int i = 0; i < m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            if(fabs(this->m_channel_rt_status.cur_pos_work.GetAxisValue(i) - pos[i]) > 1e-3){ //δ��λ

                flag = false;
                phy_axis = this->GetPhyAxis(i);
                printf("phy_axis %d target pos: %lf\n", phy_axis, pos[i]);
                if(phy_axis != 0xff){
                    this->ManualMove(i, pos[i], m_p_axis_config[phy_axis].rapid_speed, true);  //��������ϵ

                }
                //	printf("cur work pos = %lf, tar pos = %lf\n", m_channel_rt_status.cur_pos_work.GetAxisPos(i), pos[i]);
            }
        }
    }

    if(!flag) return false;

    //���Ĳ���ͬ��λ��
    if(!this->m_b_mc_on_arm)
        this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
    else
        this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

    this->RefreshAxisIntpPos();

    if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
        this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
    }else
        this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //ͬ���ѱ�������ƶ�ָ���λ��

    m_n_run_thread_state = RUN;
    // @test zk ��֪��Ϊ��  �������������岹���ܼ���������˶�ָ��
    StartMcIntepolate();
    return true;
}


/**
 * @brief ���ù���״̬�����磺���Σ�ѡͣ�ȵ�
 * @param state : ���õ�״̬
 * @param mode : ״̬�Ŀ���   0--�ر�   1--��    10--�㶯�������ݵ�ǰ״̬ȡ����
 */
void ChannelControl::SetFuncState(int state, uint8_t mode){
    bool cur_check = m_channel_status.func_state_flags.CheckMask(state);
    if(mode != 10){
        if(mode == 0 && !cur_check)  //�ޱ仯
            return;
        else if(mode == 1 && cur_check)
            return;
    }

    if(state == FS_SINGLE_LINE){//����
        if(mode == 10){   //�㶯
            if(m_channel_status.func_state_flags.CheckMask(state)){
                this->SetMcStepMode(false);
                this->m_b_need_change_to_pause = false;
            }
            else if(this->m_n_macroprog_count == 0 || this->m_p_general_config->debug_mode > 0){  //�Ǻ������ã����رյ���ģʽ
                this->SetMcStepMode(true);
            }
        }else if(mode == 0){  //�ر�
            this->SetMcStepMode(false);
            this->m_b_need_change_to_pause = false;
        }else if(this->m_n_macroprog_count == 0 || this->m_p_general_config->debug_mode > 0){//��
            this->SetMcStepMode(true);
        }

    }

    if(mode == 10){   //�㶯
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


    }else if(mode == 0){//�ر�
        m_channel_status.func_state_flags.ResetMask(state);
        if(state == FS_SINGLE_LINE){
            this->m_p_f_reg->MSBK = 0;
        }else if(state == FS_BLOCK_SKIP){
            this->m_p_f_reg->MBDT1 = 0;
        }
    }else{//��
        m_channel_status.func_state_flags.SetMask(state);
        if(state == FS_SINGLE_LINE){
            this->m_p_f_reg->MSBK = 1;
        }else if(state == FS_BLOCK_SKIP){
            this->m_p_f_reg->MBDT1 = 1;
        }
    }

    const vector<string> table = {"����ִ��", "������", "ѡ��ֹͣ", "����ģ��", "������Ծ", "�༭��", "������",
                                 "����������", "�ֶ�����"};
    if(state >= 0 && state < (int)table.size())
    {
        if (mode == 0)
            g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "�ر�[" + table[state] + "]");
        else if (mode == 10)
            g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "����[" + table[state] + "]�㶯");
        else
            g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "����[" + table[state] + "]");
    }
    //֪ͨHMI
    this->SendChnStatusChangeCmdToHmi(FUNC_STATE);
}

/**
 * @brief ͨ���Ƿ���func״̬
 * @param func : ָ��״̬
 * @return
 */
bool ChannelControl::CheckFuncState(int func){
    return m_channel_status.func_state_flags.CheckMask(func);
}

/**
 * @brief �����Զ�����
 * @param ratio
 */
void ChannelControl::SetAutoRatio(uint8_t ratio){
    //	printf("ChannelControl::SetAutoRatio, chn=%hhu, old_r = %hhu, ratio=%hhu\n", m_n_channel_index, m_channel_status.auto_ratio, ratio);
    this->m_channel_status.auto_ratio = ratio;

    g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "[��������]�л�Ϊ "+to_string(ratio));
    //֪ͨMC
    this->SetMcRatio();

    //֪ͨHMI
    this->SendChnStatusChangeCmdToHmi(AUTO_RATIO);
}

/**
 * @brief �����ֶ�����
 * @param ratio
 */
void ChannelControl::SetManualRatio(uint8_t ratio){
    if(m_channel_status.manual_ratio == ratio)
        return;
    this->m_channel_status.manual_ratio = ratio;

    g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "[�ֶ�����]�л�Ϊ "+to_string(ratio));
    //֪ͨMC
    this->SetMcRatio();

    //֪ͨHMI
    this->SendChnStatusChangeCmdToHmi(MANUAL_RATIO);
}

/**
 * @brief ���ÿ��ٽ�������
 * @param ratio
 */
void ChannelControl::SetRapidRatio(uint8_t ratio){

    this->m_channel_status.rapid_ratio = ratio;

    g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "[���Ʊ���]�л�Ϊ "+to_string(ratio));

    //֪ͨMC
    this->SetMcRatio();

    //֪ͨHMI
    this->SendChnStatusChangeCmdToHmi(RAPID_RATIO);
}

/**
 * @brief �������ᱶ��
 * @param ratio
 */
void ChannelControl::SetSpindleRatio(uint8_t ratio){
    this->m_channel_status.spindle_ratio = ratio;

    m_p_spindle->InputSOV(ratio);

    g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "[���ᱶ��]�л�Ϊ "+to_string(ratio));
    //֪ͨHMI
    this->SendChnStatusChangeCmdToHmi(SPINDLE_RATIO);
}

/**
 * @brief �����ֶ�����
 * @param step : ������ 1--1um  2--10um  3--100um   4--1000um
 */
void ChannelControl::SetManualStep(uint8_t step){

    if(this->m_channel_status.manual_step == step)
        return;

    this->m_channel_status.manual_step = step;



    //	printf("set manual step : %hhu\n", step);

    //֪ͨHMI
    this->SendChnStatusChangeCmdToHmi(MANUAL_STEP);
}

/**
 * @brief �����ֶ������ƶ�״̬
 * @param mode : ״̬�Ŀ���   0--�ر�   1--��    10--�㶯�������ݵ�ǰ״̬ȡ����
 */
void ChannelControl::SetManualRapidMove(uint8_t mode){
    //	if(m_channel_status.chn_work_mode != MANUAL_MODE &&
    //			m_channel_status.chn_work_mode != MANUAL_STEP_MODE){  //���ֶ�ģʽ��������Ӧ���˳�
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
        if(!m_channel_status.func_state_flags.CheckMask(FS_MANUAL_RAPID))  //���䣬ֱ�ӷ���
            return;
        m_channel_status.func_state_flags.SetMask(FS_MANUAL_RAPID, 0);
    }else{
        if(m_channel_status.func_state_flags.CheckMask(FS_MANUAL_RAPID))   //���䣬ֱ�ӷ���
            return;
        m_channel_status.func_state_flags.SetMask(FS_MANUAL_RAPID, 1);
    }


    //�ط��ֶ�����
//    if(m_channel_status.cur_manual_move_dir != DIR_STOP){
//        this->ManualMove(m_channel_status.cur_manual_move_dir);
//    }
}

/**
 * @brief ���õ�ǰ��
 * @param axis : ��ǰ���
 */
void ChannelControl::SetCurAxis(uint8_t axis){
    if(this->m_channel_status.cur_axis == axis)
        return;
    this->m_channel_status.cur_axis = axis;

    //֪ͨHMI
    this->SendChnStatusChangeCmdToHmi(CUR_AXIS);
}

/**
 * @brief �����ƶ�ָ��
 * @param hw_count : ����ҡ���ĸ���������Ϊ����������Ϊ����
 */
void ChannelControl::HandwheelMove(int32_t hw_count){
    if(m_channel_status.chn_work_mode != MPG_MODE){  //������ģʽ��������Ӧ���˳�
        return;
    }
}

/**
 * @brief ��ȡͨ�����Ӧ�������������ţ�0��ʼ
 * @param chn_axis :  ͨ����ţ�0��ʼ
 * @return ���ض�Ӧ�����������������ɹ�����0xff���ɹ�����0��ʼ��������������
 */
uint8_t ChannelControl::GetPhyAxis(uint8_t chn_axis){
    if(chn_axis >= m_p_channel_config->chn_axis_count)
        return 0xff;

    if(m_p_channel_config->chn_axis_phy[chn_axis] == 0)  // if(m_channel_status.cur_chn_axis_phy[chn_axis] == 0)
        return 0xff;  //δ����


    return m_p_channel_config->chn_axis_phy[chn_axis]-1;  // return m_channel_status.cur_chn_axis_phy[chn_axis]-1;
}

/**
 * @brief ��ö�Ӧ�������ͨ���������ţ�0��ʼ
 * @param phy_axis : ������ţ� 0��ʼ
 * @return ���ض�Ӧ��ͨ�������������ɹ�����0xff���ɹ�����0��ʼ��ͨ����������
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
    this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
}

/**
 * @brief ��ȡ��Ӧͨ�������Ƶ������������ţ�0��ʼ
 * @param axis_name : ����������
 * @return ���ض�Ӧ�����������������ɹ�����0xff���ɹ�����0��ʼ��������������
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
        return 0xff;  //δ����
    }

    return m_p_channel_config->chn_axis_phy[index]-1; //return m_channel_status.cur_chn_axis_phy[index]-1;
}

/**
 * @brief ��ȡ��Ӧͨ�������Ƶ�ͨ���������ţ�0��ʼ
 * @param axis_name : ����������
 * @return ���ض�Ӧ��ͨ�������������ɹ�����0xff���ɹ�����0��ʼ��ͨ����������
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
 * @brief ��ȡָ����ŵ������Ӧ��ͨ����ţ�0��ʼ
 * @param spd_idx : ������ţ�0��ʼ
 * @return ���ض�Ӧ��ͨ����ţ�0��ʼ, ʧ�ܷ���0xFF
 */
uint8_t ChannelControl::GetSpdChnAxis(uint8_t spd_idx){
    if(spd_idx >= this->m_n_spindle_count)
        return 0xFF;

    return this->GetChnAxisFromPhyAxis(this->m_spd_axis_phy[spd_idx]-1);
}


/**
 * @brief �ֶ��ƶ�����dir�����ƶ�dis����
 * @param axis : ͨ����ţ���0��ʼ
 * @param dir : �˶�����
 * @param vel : �˶��ٶ�, ��λ��mm/min
 * @param inc_dis �� ����λ��
 */
//void ChannelControl::ManualMove2(uint8_t axis, int8_t dir, double vel, double inc_dis){

//    //���Ӳ��λ
//    uint8_t phy_axis = this->GetPhyAxis(axis);
//    if(phy_axis != 0xff){
//        if(m_p_channel_engine->CheckAxisHardLimit(phy_axis, dir)){   //Ӳ��λ�澯��ֱ�ӷ���
//            printf("hard limit active, manual move return 1 \n");
//            return;
//        }
//    }

//    McCmdFrame cmd;
//    memset(&cmd, 0x00, sizeof(McCmdFrame));

//    cmd.data.channel_index = m_n_channel_index;
//    cmd.data.axis_index = axis+1;   //��Ŵ�1��ʼ
//    cmd.data.cmd = CMD_MC_AXIS_MAUAL_MOVE;

//    //�����ٶ�
//    uint32_t feed = vel*1000/60;   //ת����λΪum/s

//    //	memcpy(cmd.data.data, &feed, sizeof(feed));
//    cmd.data.data[0] = (feed & 0xFFFF);
//    cmd.data.data[1] = ((feed>>16)&0xFFFF);

//    //����Ŀ��λ��
//    int64_t tar_pos = fabs(inc_dis) * 1e7 *dir;   //��λת����mm-->0.1nms


//    if((m_channel_status.returned_to_ref_point & (0x01<<axis))
//            && m_p_axis_config[phy_axis].soft_limit_check_1 == 1){//����λ1��Ч
//        int64_t cur_pos = this->GetAxisCurMachPos(axis)*1e7;  //��ǰλ��
//        int64_t limit_pos = 0;
//        int8_t dir = (tar_pos > cur_pos)?DIR_POSITIVE:DIR_NEGATIVE;
//        if(dir == DIR_POSITIVE){//����
//            limit_pos = m_p_axis_config[phy_axis].soft_limit_max_1*1e7;


//            if(limit_pos < cur_pos){//�Ѿ�����λ�⣬�澯
//                CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index, m_channel_status.cur_axis);
//                //	this->m_error_code = ERR_SOFTLIMIT_POS;
//                return;
//            }else if(limit_pos == cur_pos){  //�Ѿ���������λ����λ��
//                CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index, m_channel_status.cur_axis);
//                return;
//            }else if(tar_pos > (limit_pos-cur_pos)){
//                tar_pos = limit_pos - this->GetAxisCurIntpTarPos(m_channel_status.cur_axis, true)*1e7;

//            }
//        }else if(dir == DIR_NEGATIVE){//����
//            limit_pos = m_p_axis_config[phy_axis].soft_limit_min_1*1e7;

//            if(limit_pos > cur_pos){//�Ѿ�����λ�⣬�澯
//                CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index, m_channel_status.cur_axis);
//                //	this->m_error_code = ERR_SOFTLIMIT_NEG;
//                return;
//            }else if(limit_pos == cur_pos){  //�Ѿ����︺��λ����λ��
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

//    cmd.data.data[6] = 0x02;   //����Ŀ��λ��

//    if(!this->m_b_mc_on_arm)
//        m_p_mc_comm->WriteCmd(cmd);
//    else
//        m_p_mc_arm_comm->WriteCmd(cmd);

//    printf("manual move2: axis = %d, tar_pos = %lld, type = 0x%x\n", axis, tar_pos, cmd.data.data[6]);
//}

/**
 * @brief �ֶ��ƶ���
 * @param dir : �˶�����
 */
void ChannelControl::ManualMove(int8_t dir){
    if(m_channel_status.chn_work_mode != MANUAL_MODE &&
            m_channel_status.chn_work_mode != MANUAL_STEP_MODE){  //���ֶ�ģʽ��������Ӧ���˳�
    	return;
    }
    // printf("send manualmove cmd to mc\n");

    //���Ӳ��λ
    uint8_t phy_axis = this->GetPhyAxis(m_channel_status.cur_axis);
    if(phy_axis != 0xff){
        if(m_p_channel_engine->CheckAxisHardLimit(phy_axis, dir)){   //Ӳ��λ�澯��ֱ�ӷ���
            printf("hard limit active, manual move return 2 \n");
            return;
        }
    }
    //����Ŀ��λ��

    //int64_t cur_pos = GetAxisCurMachPos(m_channel_status.cur_axis)*1e7;  //��ǰλ��
    int64_t cur_pos = GetAxisCurIntpTarPos(m_channel_status.cur_axis, false)*1e7;
    int64_t tar_pos = 0;
    if(GetChnWorkMode() == MANUAL_STEP_MODE){ //�ֶ�����
        tar_pos = cur_pos + GetCurManualStep()*1e4*dir;		//ת����λΪ0.1nm

    }else{
        tar_pos = cur_pos + 99999*1e7*dir;    //�ֶ�����ģʽ����Ŀ��λ�����õĺ�Զ
    }
    //ScPrintf("mode == %d tar_pos = %lld",GetChnWorkMode(), tar_pos);
    //�������λ
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

    m_channel_status.cur_manual_move_dir = dir;   //���淽��

    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = m_channel_status.cur_axis+1;   //��Ŵ�1��ʼ
    cmd.data.cmd = CMD_MC_AXIS_MAUAL_MOVE;

    //�����ٶ�
    uint32_t feed = this->m_p_axis_config[phy_axis].manual_speed*1000/60;   //ת����λΪum/s
    if(this->m_channel_status.func_state_flags.CheckMask(FS_MANUAL_RAPID)){
        feed *= 2;  //�ٶȷ���
        printf("double manual feed\n");
    }
    // ScPrintf("axis:%u, feed:%u, n_inc_dis=%lld",m_channel_status.cur_axis,feed, n_inc_dis);

    cmd.data.data[0] = (feed & 0xFFFF);
    cmd.data.data[1] = ((feed>>16)&0xFFFF);
    cmd.data.data[2] = (n_inc_dis & 0xFFFF);
    cmd.data.data[3] = ((n_inc_dis>>16)&0xFFFF);
    cmd.data.data[4] = ((n_inc_dis>>32) & 0xFFFF);
    cmd.data.data[5] = ((n_inc_dis>>48)&0xFFFF);

    cmd.data.data[6] = 0x02;   //����Ŀ��λ��

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);

    printf("manual move: axis = %d, tar_pos = %lld, type = 0x%x, ratio = %hhu\n", m_channel_status.cur_axis, tar_pos, cmd.data.data[6],
            this->m_channel_status.manual_ratio);
}

/**
 * @brief �ֶ��ƶ�PMC��
 * @param dir : �˶�����
 */
void ChannelControl::ManualMovePmc(int8_t dir){
    uint8_t phy_axis = this->GetPhyAxis(m_channel_status.cur_axis);  //��ǰ���Ӧ���������
    m_channel_status.cur_manual_move_dir = dir;   //���淽��

    PmcCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(PmcCmdFrame));

    //�����  ���ֽ�:0--����ָ��   1--����G01ָ��  ���ֽڣ�0--��������   1--��������
    cmd.data.cmd = 0x100;

    //���    ���ֽڣ����[1-64]    ���ֽڣ�����ͨ����[1-4]
    cmd.data.axis_index = ((phy_axis+1)|((m_n_channel_index+1)<<8));

    cmd.data.data[0] = 0;   //��λ�ƶ�����0


    //�����ٶ�
    uint32_t feed = this->m_p_axis_config[phy_axis].manual_speed * 1000 / 60;   //ת����λΪum/s
    if(this->m_channel_status.func_state_flags.CheckMask(FS_MANUAL_RAPID)){
        feed *= 2;  //�ٶȷ���
        printf("double manual feed\n");
    }

    //����Ŀ��λ��
    int64_t cur_pos = GetAxisCurMachPos(m_channel_status.cur_axis)*1e7;  //��ǰλ��
    int64_t tar_pos = 0;
    if(GetChnWorkMode() == MANUAL_STEP_MODE){ //�ֶ�����
        tar_pos = cur_pos + GetCurManualStep()*1e4*dir;		//ת����λΪ0.1nm

    }else{
        tar_pos = cur_pos + 99999*1e7*dir;    //�ֶ�����ģʽ����Ŀ��λ�����õĺ�Զ
    }

    //�������λ
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

    memcpy(&cmd.data.data[1], &n_inc_dis, sizeof(tar_pos));  //����Ŀ��λ��

    memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //�����ٶ�

    this->m_mask_run_pmc = 0x01L<<phy_axis;  //���õ�ǰ������

    this->m_p_mi_comm->SendPmcCmd(cmd);


    printf("chn manual move pmc: axis = %d, tar_pos = %lld\n", m_channel_status.cur_axis, tar_pos);
}


/**
 * @brief ��ȡ��ǰ�ֶ�������������
 * @return ��ǰ�ֶ�������ֵ����λum
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
 * @brief �ֶ�ָ��ĳ������ָ���ٶ��˶���ָ��λ��
 * @param axis : ͨ����ţ�ָ����Ҫ�ƶ����ᣬ��0��ʼ����
 * @param pos : Ŀ��λ�ã���λmm
 * @param vel �� Ŀ���ٶȣ���λmm/min
 * @param workcoord_flag : Ŀ��λ���Ƿ�Ϊ��������ϵ�µ����꣬0--��е����ϵ   1--��������ϵ
 */
void ChannelControl::ManualMove(uint8_t axis, double pos, double vel, bool workcoord_flag){

    if(g_ptr_chn_engine->GetPmcActive(this->GetPhyAxis(axis))) {
        //this->ManualMovePmc(axis, pos, vel);
        m_error_code = ERR_PMC_IVALID_USED;
        CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        return;
    }

    if(workcoord_flag){//��������ϵת��Ϊ��е����ϵ
        this->TransWorkCoordToMachCoord(pos, this->m_channel_status.gmode[14], axis);
    }

    //����Ŀ��λ��
    int64_t tar_pos = pos*1e7;    //ת����λΪ0.1nm
    uint8_t phy_axis = this->GetPhyAxis(axis);
    int64_t cur_pos = GetAxisCurMachPos(axis)*1e7;  //��ǰλ��
    ManualMoveDir dir = (tar_pos > cur_pos)?DIR_POSITIVE:DIR_NEGATIVE;  //�ƶ�����

    //�������λ
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
    cmd.data.axis_index = axis+1;	//MC����Ŵ�1��ʼ
    cmd.data.cmd = CMD_MC_AXIS_MAUAL_MOVE;

    //�����ٶ�
    uint32_t feed = vel*1000/60;   //ת����λΪum/s
    memcpy(cmd.data.data, &feed, sizeof(feed));
    memcpy(&cmd.data.data[2], &tar_pos, sizeof(tar_pos));

    //	if(workcoord_flag)
    //		cmd.data.data[6] = 1;   //����λ�ã���������ϵ

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
     //printf("send manual move cmd: axis=%d, pos = %lf, vel = %lf\n", axis, pos, vel);
}

/**
 * @brief ֹͣ��ǰ���ֶ��ƶ�
 * @param ��
 */
void ChannelControl::ManualMoveStop(){
    if(m_channel_status.chn_work_mode == MANUAL_STEP_MODE){  //�ֶ�����ģʽ��������Ӧ���˳�
        return;
    }

    m_channel_status.cur_manual_move_dir = DIR_STOP;

    if(g_ptr_chn_engine->GetPmcActive(this->GetPhyAxis(m_channel_status.cur_axis))) {
        //PMC���Stop����û�����Σ�������δ˴�������ĳЩ�������ͣ������
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
 * @brief ֹͣ����ֶ��ƶ�
 * @param axis_mask : ��Ҫֹͣ����mask
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
 * @brief ֹͣ��ǰPMC���ֶ��ƶ�
 */
void ChannelControl::ManualMovePmcStop(){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(MiCmdFrame));

    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = this->m_n_channel_index+1;
    cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
    cmd.data.data[0] = 0x20;   //ֹͣ��ǰ���˶�����������ǰ�˶�ָ��

    m_p_mi_comm->WriteCmd(cmd);

    this->m_mask_run_pmc = 0;
    this->m_mask_runover_pmc = 0;
}

/**
 * @brief ֹͣPMC����ֶ��ƶ�
 * @param phy_axis : ������ţ���0��ʼ
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
    cmd.data.data[0] = 0x20;   //ֹͣ��ǰ���˶�����������ǰ�˶�ָ��

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
 * @brief ��ͣPMC���ƶ�
 * @param phy_axis : 0~63 : ָ�����     0xFF : ָ��������
 * @param flag  : true--��ͣ     false--ȡ����ͣ
 */
void ChannelControl::PausePmcAxis(uint8_t phy_axis, bool flag){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(MiCmdFrame));

    uint16_t data = 0x01;
    if(!flag)
        data = 0x10;   //ȡ����ͣ
    if(phy_axis != 0xFF)
        cmd.data.axis_index = phy_axis+1;
    else
        cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = this->m_n_channel_index+1;
    cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
    cmd.data.data[0] = data;   //��ͣ��ǰ���˶�

    m_p_mi_comm->WriteCmd(cmd);

    printf("ChannelControl::PausePmcAxis, phy_axis=%hhu, flag = %hhu\n", phy_axis, flag);
}


/**
 * @brief ֹͣPMC���ƶ�
 * @param phy_axis : 0~63 : ָ�����     0xFF : ָ��������
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
    cmd.data.data[0] = 0x20;   //ֹͣ��ǰ���˶�

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
 * @brief �ֶ�ָ��ĳ��PMC����ָ���ٶ��˶���ָ��λ��
 * @param axis : ͨ����ţ�ָ����Ҫ�ƶ����ᣬ��0��ʼ����
 * @param pos : Ŀ��λ�ã���λmm
 * @param vel �� Ŀ���ٶȣ���λmm/min
 */
void ChannelControl::ManualMovePmc(uint8_t axis, double pos, double vel){
    uint8_t phy_axis = this->GetPhyAxis(axis);

    PmcCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(PmcCmdFrame));

    //�����  ���ֽ�:0--����ָ��   1--����G01ָ��  ���ֽڣ�0--��������   1--��������
    cmd.data.cmd = 0;

    //���    ���ֽڣ����[1-64]    ���ֽڣ�����ͨ����[1-4]
    cmd.data.axis_index = ((phy_axis+1)|((m_n_channel_index+1)<<8));

    cmd.data.data[0] = 0;   //��λ�ƶ�����0


    //�����ٶ�
    uint32_t feed = vel*1000/60;   //ת����λΪum/s


    //����Ŀ��λ��
    int64_t tar_pos = pos*1e7;    //ת����λΪ0.1nm
    int64_t cur_pos = GetAxisCurMachPos(axis)*1e7;  //��ǰλ��
    ManualMoveDir dir = (tar_pos > cur_pos)?DIR_POSITIVE:DIR_NEGATIVE;  //�ƶ�����
    //�������λ
    double limit = 0;
    if(CheckSoftLimit(dir, phy_axis, GetAxisCurMachPos(axis))){
        printf("soft limit active, manual move abs return \n");
        return;
    }else if(GetSoftLimt(dir, phy_axis, limit) && dir == DIR_POSITIVE && pos > limit * 1e7){
        tar_pos = limit * 1e7;
    }else if(GetSoftLimt(dir, phy_axis, limit) && dir == DIR_NEGATIVE && pos < limit * 1e7){
        tar_pos = limit * 1e7;
    }

    memcpy(&cmd.data.data[1], &tar_pos, sizeof(tar_pos));  //����Ŀ��λ��

    memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //�����ٶ�

    this->m_mask_run_pmc = 0x01L<<phy_axis;  //���õ�ǰ������

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
 * @brief ����MC����ģʽ
 * @param flag : true--���õ�����Ч   false--ȡ������
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
 * @brief ��MC������Ĺ�������ϵԭ��
 * @param axis_index : �����������0��ʼ
 */
void ChannelControl::SetMcAxisOrigin(uint8_t axis_index){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = axis_index+1;
    cmd.data.cmd = CMD_MC_SET_AXIS_ORIGIN;

    int64_t origin_pos = m_p_chn_coord_config[0].offset[axis_index] * 1e7;  //������������ϵ
    int coord_index = m_channel_status.gmode[14];
    if(coord_index <= G59_CMD ){
        origin_pos += m_p_chn_coord_config[coord_index/10-53].offset[axis_index] * 1e7;    //��λ��mmת��Ϊ0.1nm
    }else if(coord_index <= G5499_CMD){
        origin_pos += m_p_chn_ex_coord_config[coord_index/10-5401].offset[axis_index] * 1e7;    //��λ��mmת��Ϊ0.1nm
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
 * @brief ��MC������Ĺ�������ϵԭ��
 * @param axis_index : �����������0��ʼ
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
 * @brief �������õĹ�������ϵ
 * @param active_flag : �����־��true--����   false--ʧЧ
 */
void ChannelControl::ActiveMcOrigin(bool active_flag){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint16_t coord_index = m_channel_status.gmode[14]/10;   //ģ̬���ʱ������10�����˴���С10��

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
 * @brief ��MC���ø��ᵶ��ƫ��
 * @param axis_index : �����������0��ʼ
 */
void ChannelControl::SetMcAxisToolOffset(uint8_t axis_index){
    SCAxisConfig &axis_config = this->m_p_axis_config[this->GetPhyAxis(axis_index)];

    if(axis_config.axis_type != AXIS_LINEAR || m_channel_status.cur_h_code == 0 || m_channel_status.gmode[8] == G49_CMD)  //��ֱ������ߵ�ǰ��ƫΪ0
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
        offset = m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2] * 1e7;  //��λ��mmת��Ϊ0.1nm
        //	offset += m_p_chn_tool_config->geometry_comp_basic[2] * 1e7;   //��׼��ƫ
        offset += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1] * 1e7;   //����ĥ�𲹳�
    }
    else if(axis_config.axis_linear_type == LINE_AXIS_X || axis_config.axis_linear_type == LINE_AXIS_Y){
        offset = m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][axis_config.axis_linear_type-1] * 1e7;  //��λ��mmת��Ϊ0.1nm
    }
    else //��XYZ��û�е���ƫ��
    {
    	printf("��XYZ��û�е���ƫ��");
    	return;
    }

    if(this->m_channel_status.gmode[8] == G44_CMD)	//���򲹳�
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
 * @brief �������õĵ���ƫ��
 * @param active_flag : �����־��true--����   false--ʧЧ
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
 * @brief MC�Ƿ���Ҫ������������
 * @return
 */
bool ChannelControl::IsMcNeedStart(){
    printf("need=%hhu, bufcount=%hu, autoblock=%hhu\n", m_b_mc_need_start, m_channel_mc_status.buf_data_count, m_channel_mc_status.auto_block_over);
    if(!m_b_mc_need_start)
        return false;

    if(m_channel_mc_status.buf_data_count == 0 && m_channel_mc_status.auto_block_over)  //MCû�п���������
        return false;

    return true;
}

/**
 * @brief ��ȡ��ǰMC���˶����ݵ�����,����ӦAUTO��MDAģʽ
 * @return mc��ʣ���˶�������
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
 * @brief ��ǰ�ֿ鵽λ��־�Ƿ���Ч
 * @return true--��Ч   false--��Ч
 */
bool ChannelControl::CheckBlockOverFlag(){

    struct timeval cur_time;
    gettimeofday(&cur_time, nullptr);
    unsigned int delay = (cur_time.tv_sec-m_time_start_mc.tv_sec)*1000000+cur_time.tv_usec-m_time_start_mc.tv_usec;  //��λ΢��
    if(delay < MC_STATE_REFRESH_CYCLE) //����MC��������5ms�����ȷ�ϵ�λ��־
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
 * @brief ��ǰ���ε�λ��־�Ƿ���Ч
 * @return true--��Ч   false--��Ч
 */
bool ChannelControl::CheckStepOverFlag(){
    struct timeval cur_time;
    gettimeofday(&cur_time, nullptr);
    unsigned int delay = (cur_time.tv_sec-m_time_start_mc.tv_sec)*1000000+cur_time.tv_usec-m_time_start_mc.tv_usec;  //��λ΢��
    if(delay < MC_STATE_REFRESH_CYCLE) //����MC��������5ms�����ȷ�ϵ�λ��־
        return false;

    if(!this->m_b_mc_on_arm)
        this->m_channel_mc_status.step_over = this->m_p_mc_comm->ReadStepRunOverFlag(m_n_channel_index);
    else
        this->m_channel_mc_status.step_over = this->m_p_mc_arm_comm->ReadStepRunOverFlag(m_n_channel_index);
    return m_channel_mc_status.step_over;
}

/**
 * @brief ���µ�ǰͨ��ģ̬����
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
        //this->m_channel_status.cur_d_code = mode.bits.mode_d;//llx add ���߰뾶Dģ̬������MC��ȡ,��SCֱ������

        this->m_mc_mode_cur.all = mode.all;

        b_change = true;
    }
    int mode_1 = m_channel_mc_status.cur_cmd*10;
    if(m_channel_status.gmode[1] != mode_1){
        //		printf("change 01group mode : old = %d, new = %d\n", m_channel_status.gmode[1], mode_1);
        //��ǰ01��Gģ̬
        m_channel_status.gmode[1] = mode_1;
        b_change = true;

    }

    if(b_change){
        this->SendChnStatusChangeCmdToHmi(G_MODE);
        //this->SendModeChangToHmi(D_MODE);//llx add ���߰뾶Dģ̬������MC��ȡ,��SCֱ������
    }

}

/**
 * @brief ��SCֱ�Ӹ���ģ̬����
 * @param mode_type : ģ̬����
 * @param value     : ����ֵ
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
 * @brief ��������ϵԭ������
 * @param flag : ���������ϵ��־�� true - ����   false - ʧЧ
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
        		origin_pos += (int64_t)(m_p_chn_coord_config[0].offset[i] * 1e7);  //������������ϵ

				// g92 offset
				origin_pos += (int64_t)(m_p_chn_g92_offset->offset[i] * 1e7);

        		// g54xx offset
        		int coord_index = m_channel_status.gmode[14];
				if(coord_index <= G59_CMD ){
					origin_pos += (int64_t)(m_p_chn_coord_config[coord_index/10-53].offset[i] * 1e7);    //��λ��mmת��Ϊ0.1nm
					printf("===== g54 offset  axis: %i offset %lld\n", i, (int64_t)(m_p_chn_coord_config[coord_index/10-53].offset[i] * 1e7));
				}else if(coord_index <= G5499_CMD){
					origin_pos += (int64_t)(m_p_chn_ex_coord_config[coord_index/10-5401].offset[i] * 1e7);    //��λ��mmת��Ϊ0.1nm
					printf("===== g5401 offset  axis: %i offset %lld\n", i, (int64_t)(m_p_chn_ex_coord_config[coord_index/10-5401].offset[i] * 1e7));
				}
        	}

        	this->SetMcAxisOrigin(i, origin_pos);

        	/*
        	//this->SetMcAxisOrigin(i);
            printf("=============================axis %d ======================\n", i);
            int64_t origin_pos = 0;
            if(!G92Active) origin_pos = (int64_t)(m_p_chn_coord_config[0].offset[i] * 1e7);  //������������ϵ

        	printf("===== basic offset  axis: %i offset %lld\n", i, origin_pos);
            int coord_index = m_channel_status.gmode[14];
            if(coord_index <= G59_CMD ){
            	origin_pos += (int64_t)(m_p_chn_coord_config[coord_index/10-53].offset[i] * 1e7);    //��λ��mmת��Ϊ0.1nm
            	printf("===== g54 offset  axis: %i offset %lld\n", i, (int64_t)(m_p_chn_coord_config[coord_index/10-53].offset[i] * 1e7));
            }else if(coord_index <= G5499_CMD){
                origin_pos += (int64_t)(m_p_chn_ex_coord_config[coord_index/10-5401].offset[i] * 1e7);    //��λ��mmת��Ϊ0.1nm
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
 * @brief ����MC�ĵ���ƫ��
 * @param flag : �����ƫ�ñ�־�� true - ����   false - ʧЧ
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
 * @brief ����ͨ��������
 */
void ChannelControl::SetChnAxisName(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;

    cmd.data.cmd = CMD_MC_SET_CHN_AXIS_NAME;

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        cmd.data.axis_index = i+1;
        cmd.data.data[0] = this->m_p_channel_config->chn_axis_name[i];

        if(this->m_p_general_config->axis_name_ex){  //������չ���±�
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
 * @brief ������ʹ�ܣ��Լ��岹ģʽ��1--NC��岹   2--PMC��岹��
 * @param index : �����������0��ʼ
 */
void ChannelControl::SetChnAxisOn(uint8_t chn_axis){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint8_t phy_axis = this->GetPhyAxis(chn_axis);

    cmd.data.axis_index = chn_axis+1;
    cmd.data.channel_index = m_n_channel_index;   // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SET_AXIS_ON;

    //��ʹ��  0--��ֹ����   1--ʹ�ܸ���
    SCAxisConfig &axis_config = this->m_p_axis_config[phy_axis];
    if(axis_config.axis_type != AXIS_SPINDLE)
        cmd.data.data[0] = 1;
    else
        cmd.data.data[0] = 0;


    //��岹����   1--NC��岹���Զ����ֶ���MDI��  2--PMC��岹   ����ֵ��Ч  ��ע��10MC��DSP�ݲ�֧�� ����PMC�ᣩ
    cmd.data.data[1] = 1;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief ����ָ������ݾ༰����ٶȵȻ�����Ϣ
 * @param index : ��������, ��0��ʼ
 */
void ChannelControl::SetChnAxisBaseParam(uint8_t chn_axis){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint8_t phy_axis = this->GetPhyAxis(chn_axis);

    cmd.data.axis_index = chn_axis+1;
    cmd.data.channel_index = m_n_channel_index;  // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SET_AXIS_BASE_INFO;

    SCAxisConfig &axis_config = this->m_p_axis_config[phy_axis];
    //������   1--ֱ����   2--��ת��   3--����
    cmd.data.data[0] = axis_config.axis_type+1;

    //�ݾ࣬��λ��1um
    uint32_t data = axis_config.move_pr * 1e3;  //mm->1um
    cmd.data.data[1] = (data&0xFFFF);
    cmd.data.data[2] = ((data>>16)&0xFFFF);

    //����ٶ����ƣ� ��λ��um/s
    data = axis_config.move_pr * axis_config.motor_speed_max * 1000 /60;   //mm/min -> um/s
    cmd.data.data[3] = (data&0xFFFF);
    cmd.data.data[4] = ((data>>16)&0xFFFF);

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);

}

/**
 * @brief ����ָ������ٶ������Ϣ
 * @param index : ��������, ��0��ʼ
 */
void ChannelControl::SetChnAxisSpeedParam(uint8_t chn_axis){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint8_t phy_axis = this->GetPhyAxis(chn_axis);

    cmd.data.axis_index = chn_axis+1;
    cmd.data.channel_index = m_n_channel_index;  // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SET_AXIS_SPEED_LIMIT;

    SCAxisConfig &axis_config = this->m_p_axis_config[phy_axis];
    //G00�ٶȣ���λ��um/s
    uint32_t data = axis_config.rapid_speed * 1000 / 60;  //mm/min -> um/s
    //	printf("set axis %hhu g00 speed: %u\n", index, data);
    cmd.data.data[0] = (data&0xFFFF);
    cmd.data.data[1] = ((data>>16)&0xFFFF);
    //	printf("set axis %hhu g00 speed: 0x%hx,  0x%hx\n", index, cmd.data.data[0], cmd.data.data[1]);

    //ת���ٶȲ����ƣ� ��λ��um/s
    data = axis_config.corner_acc_limit * 1000 /60;   //mm/min -> um/s
    cmd.data.data[2] = (data&0xFFFF);
    cmd.data.data[3] = ((data>>16)&0xFFFF);

    //�ֶ������ٶȣ���λ��um/s  //TODO �˲������������ã��˴�����һ���̶�ֵ100mm/min
    data = 100 * 1000 / 60;    //mm/min -> um/s
    cmd.data.data[4] = (data&0xFFFF);
    cmd.data.data[5] = ((data>>16)&0xFFFF);

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief ����ָ������ٶ������Ϣ
 * @param index : ��������, ��0��ʼ
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

    //G00���ٶȣ���λ��10mm/s^2
    uint16_t data = m_p_axis_config[phy_axis].rapid_acc / 10;
    cmd.data.data[0] = data;

    //�ֶ����ٶȣ� ��λ��10mm/s^2
    data = m_p_axis_config[phy_axis].manual_acc /10;
    cmd.data.data[1] = data;

    //�ֶ����ɼ��ٶȣ���λ��10mm/s^2
    data = m_p_axis_config[phy_axis].start_acc /10;
    cmd.data.data[2] = data;

    //G00 S��ʱ�䳣��
    cmd.data.data[3] = m_p_axis_config[phy_axis].rapid_s_plan_filter_time;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief ����ָ���������λ����
 * @param axis : ָ����ţ���0��ʼ
 */
void ChannelControl::SetChnAxisSoftLimit(uint8_t chn_axis){
    //����MC������λ����
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
 * @brief ǿ�ƹر�ָ���������λ����
 * @param chn_axis : ָ����ţ���0��ʼ
 */
void ChannelControl::CloseChnAxisSoftLimit(uint8_t chn_axis){
    //����MC������λ����
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;
    cmd.data.axis_index = chn_axis+1;
    cmd.data.cmd = CMD_MC_AXIS_ENABLE_SOFT_LIMIT;


    cmd.data.data[0] = 0;

    m_p_mc_comm->WriteCmd(cmd);
}


/**
 * @brief ����ָ�����λ��ָ������ֵ
 * @param chn_axis : ָ��ͨ����ţ���0��ʼ
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
 * @brief ����ָ���������λֵ
 * @param axis : ָ����ţ���0��ʼ
 * @param index : ����λ��ţ��ܹ�3�飬0-2
 */
void ChannelControl::SetChnAxisSoftLimitValue(uint8_t chn_axis, uint8_t index){
    //����MC������λ����
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


    int32_t softlimit = max*1e3;   //��λ��mmת��Ϊ1um
    cmd.data.data[1] = (softlimit & 0xFFFF);
    cmd.data.data[2] = ((softlimit>>16) & 0xFFFF);

    softlimit = min*1e3;   //��λ��mmת��Ϊ1um
    cmd.data.data[3] = (softlimit & 0xFFFF);
    cmd.data.data[4] = ((softlimit>>16) & 0xFFFF);

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief ���ñ�ͨ����������Ĳ���
 * @param axis : ָ����ţ���0��ʼ
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
 * @brief ���üӹ��ٶȹ滮��ʽ
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
 * @brief ����ͨ���ӹ��ٶȹ滮����
 */
void ChannelControl::SetMcChnPlanParam(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;

    cmd.data.cmd = CMD_MC_SET_CHN_PLAN_PARAM;

    uint32_t data = m_p_channel_config->chn_max_vel*1000/60;  //��λת��   mm/min-->um/s
    cmd.data.data[0] = data&0xFFFF;
    cmd.data.data[1] = (data>>16)&0xFFFF;

    uint16_t tmp = m_p_channel_config->chn_max_acc/10;    //��λת��  mm/s^2-->10mm/s^2
    cmd.data.data[2] = tmp;
    tmp = m_p_channel_config->chn_max_dec/10;    //��λת��  mm/s^2-->10mm/s^2
    cmd.data.data[3] = tmp;

    cmd.data.data[4] = m_p_channel_config->chn_s_cut_filter_time;

    tmp = m_p_channel_config->chn_max_arc_acc/10;    //��λת��  mm/s^2-->10mm/s^2
    cmd.data.data[5] = tmp;

    tmp = m_p_channel_config->chn_max_corner_acc/10;    //��λת��  mm/s^2-->10mm/s^2
    cmd.data.data[6] = tmp;


    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief ���ø��Թ�˿�ӹ��滮����
 */
void ChannelControl::SetMcTapPlanParam(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;

    cmd.data.cmd = CMD_MC_SET_TAP_PLAN_PARAM;

    uint16_t tmp = m_p_channel_config->tap_max_acc/10;    //��λת��  mm/s^2-->10mm/s^2
    cmd.data.data[0] = tmp;
    tmp = m_p_channel_config->tap_max_dec/10;    //��λת��  mm/s^2-->10mm/s^2
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
 * @brief ����ͨ���ӹ��ٶȹ��ܿ���
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
 * @brief ���ùս�׼ͣ����
 */
void ChannelControl::SetMcChnCornerStopParam(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = m_n_channel_index;

    cmd.data.cmd = CMD_MC_SET_CHN_CORNER_STOP_PARAM;
    cmd.data.data[0] = this->m_p_channel_config->corner_stop_enable;
    uint32_t data = m_p_channel_config->corner_stop_angle_min;
    data *= 1000;   //0.001��
    cmd.data.data[1] = data&0xFFFF;
    cmd.data.data[2] = (data>>16)&0xFFFF;
    data =m_p_channel_config->corner_stop;  //0.001��
    data *= 1000;  //0.001��
    cmd.data.data[3] = data&0xFFFF;
    cmd.data.data[4] = (data>>16)&0xFFFF;

    if(!this->m_b_mc_on_arm)
        m_p_mc_comm->WriteCmd(cmd);
    else
        m_p_mc_arm_comm->WriteCmd(cmd);
}


/**
 * @brief ����MC�ĵ��Բ���
 * @param index : ���Բ�����
 */
#ifdef USES_WOOD_MACHINE
//����MC�����ǲ���ֵ
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
 * @brief ��MC����ϵͳ��λָ��
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
 * @brief ��ȡ����ĵ�ǰ��������ϵλ��
 * @param axis_index : ͨ����ţ���0��ʼ
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
 * @brief ��ȡ����ĵ�ǰ��е����ϵλ��
 * @param axis_index : ͨ����ţ���0��ʼ
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
 * @brief ��ȡ����ĵ�ǰ�����ٶ�
 * @param axis_index : ͨ����ţ���0��ʼ
 * @return
 */
double ChannelControl::GetAxisCurFedBckAxisSpeed(uint8_t axis_index){
    double val = 0.0;
    if(axis_index >= this->m_p_channel_config->chn_axis_count)
        return val;

    return m_channel_rt_status.cur_feedbck_torque.GetAxisValue(axis_index);
}

/**
 * @brief ��ȡ����ĵ�ǰ��������
 * @param axis_index : ͨ����ţ���0��ʼ
 * @return
 */
double ChannelControl::GetAxisCurFedBckAxisTorque(uint8_t axis_index){
    double val = 0.0;
    if(axis_index >= this->m_p_channel_config->chn_axis_count)
        return val;

    return m_channel_rt_status.cur_feedbck_torque.GetAxisValue(axis_index);
}

/**
 * @brief ��ȡ����ĵ�ǰ�岹Ŀ��λ��
 * @param axis_index : ͨ����ţ���0��ʼ
 * @param bMachCoord : true--��е����ϵ�µ�Ŀ��λ��    false--��������ϵ�µ�Ŀ��λ��
 * @return ָ����ĵ�ǰ�岹Ŀ��λ��
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
 * @brief ��MC���±�����Ϣ
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
 * @brief ��MC���õ�ǰ��������,    ľ���������������������ȼ���
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
 * @brief ִ������Ԥ��������
 * @param line �� ��ǰָ����
 */
void ChannelControl::PreExecSpindleCmd(uint64_t line){
    static bool check_time = true;   //�Ƿ������ʱ��飬��ֹMC�ĵ�ǰ�кŸ��²���ʱ���������Ԥ�����󴥷�

    if(this->m_b_prestart_spd && this->m_n_spd_prestart_step == 0xFF){
        //Ԥ������ִ�����
        return;
    }

#ifdef USES_ADDITIONAL_PROGRAM
    if(this->m_n_add_prog_type != NONE_ADD)   //���ӳ���ִ������Ԥ����
        return;
#endif

    if(!m_b_mc_need_start && check_time){
        struct timeval cur_time;
        gettimeofday(&cur_time, nullptr);
        unsigned int delay = (cur_time.tv_sec-m_time_start_mc.tv_sec)*1000000+cur_time.tv_usec-m_time_start_mc.tv_usec;  //��λ΢��
        if(delay < MC_STATE_REFRESH_CYCLE) //����MC��������5ms�����ȷ�ϵ�λ��־
            return ;
        else
            check_time = false;
    }else if(m_b_mc_need_start){
        check_time = true;
    }

    if(!this->m_b_prestart_spd){//�����Ƿ���Ԥ����ָ��
        if(this->m_n_hw_trace_state != NONE_TRACE || this->IsStepMode() || this->m_channel_status.chn_work_mode == MDA_MODE)
            return;   //MDAģʽ�����ָ��ٻ��ߵ���ģʽ�²���������Ԥ����

        double td = 0;
        int forward_line = 0;    //����Ԥ������ǰ����
        if(this->GetMacroVar(810, td) == 0)
            forward_line = td;

        if(!this->m_p_compiler->FindPreStartSpdCmd(line, line+forward_line, this->m_spd_prestart_offset)){
            //			printf("ChannelControl::PreExecSpindleCmd not fine: %llu, forward=%d\n", line, forward_line);

            return;  //û���ҵ�Ԥ������������
        }else{
            this->m_b_prestart_spd = true;
            this->m_n_spd_prestart_step = 0;
            printf("ChannelControl::PreExecSpindleCmd FIND SPD CMD[%lld: %lld]\n", line, this->m_spd_prestart_offset.line_no);
        }
    }
    //	printf("ChannelControl::PreExecSpindleCmd : %llu\n", line);

    //������ִ��������������
    int m_code = m_spd_prestart_offset.m_code;
    struct timeval time_now;
    unsigned int time_elpase = 0;
    if(this->m_n_spd_prestart_step == 0){
        //�����뷢�͸�PMC
        this->SendMCodeToPmc(this->m_spd_prestart_offset.m_code, 0);

        gettimeofday(&m_time_m_start[0], NULL);   //

        this->m_channel_status.spindle_dir = (m_code==3)?SPD_DIR_POSITIVE:SPD_DIR_NEGATIVE;  //�޸�������ת����

        m_n_spd_prestart_step++;
    }else if(m_n_spd_prestart_step == 1){
        //�ȴ�TMF��ʱ����λMFѡͨ�ź�
        gettimeofday(&time_now, NULL);
        time_elpase = (time_now.tv_sec-m_time_m_start[0].tv_sec)*1000000+
                time_now.tv_usec-m_time_m_start[0].tv_usec;
        if(time_elpase < 16000)
            return;		//δ����ʱʱ��

        this->SetMFSig(0, true);    //��λѡͨ�ź�

        gettimeofday(&m_time_m_start[0], NULL);
        m_n_spd_prestart_step++;
    }else if(m_n_spd_prestart_step == 2){
        //�ȴ�FIN�ź�
        if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(0))
            gettimeofday(&m_time_m_start[0], NULL);   //��ʼ��ʱ
        else{
            gettimeofday(&time_now, NULL);
            time_elpase = (time_now.tv_sec-m_time_m_start[0].tv_sec)*1000000+
                    time_now.tv_usec-m_time_m_start[0].tv_usec;
            if(time_elpase > kMCodeTimeout && !this->GetMExcSig(0)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

        //�ȴ�TFIN��ʱ����λMF�ź�
        gettimeofday(&time_now, NULL);
        time_elpase = (time_now.tv_sec-m_time_m_start[0].tv_sec)*1000000+
                time_now.tv_usec-m_time_m_start[0].tv_usec;
        if(time_elpase < 16000)
            return;		//δ����ʱʱ��

        this->SetMFSig(0, false);    //��λѡͨ�ź�
        m_n_spd_prestart_step++;
    }else if(m_n_spd_prestart_step == 4){
        //�ȴ�FIN�źŸ�λ
        if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(0))
            return;

        //��λ����ָ���źź�DEN�ź�
        this->SendMCodeToPmc(0, 0);

        if(this->m_n_spindle_count > 0){
            m_channel_status.rated_spindle_speed = m_spd_prestart_offset.s_code;
            this->SpindleSpeedDaOut(m_channel_status.rated_spindle_speed);
            this->SendModeChangToHmi(S_MODE);
        }
        this->m_p_f_reg->SPS = 1;    //��־������ת

        m_n_spd_prestart_step = 0xFF;    //��λ����״̬
    }

}


/**
 * @brief ʵ��ִ�и���ָ����Ϣ�� ľ����Ŀ���ƣ���������Ԥ�������
 * @param msg : ָ����Ϣ
 * @return true--�ɹ�  false--ʧ�ܣ�PL�л����������ߵȴ����е�λ
 */
bool ChannelControl::ExecuteAuxMsg_wood(RecordMsg *msg){
    AuxMsg *tmp = (AuxMsg *)msg;
    uint8_t m_count = tmp->GetMCount();   //һ����M��������
    uint8_t m_index = 0;
    uint8_t m_index_add = 0;   //�������ִ������Ԥ������׷�ӵ����
    int mcode = 0;
    bool bHasSpdCmd = false;   //�Ƿ���M03��M04ָ��


    if(this->m_n_restart_mode != NOT_RESTART &&
            tmp->GetLineNo() < this->m_n_restart_line
        #ifdef USES_ADDITIONAL_PROGRAM
            && this->m_n_add_prog_type == NONE_ADD      //�Ǹ��ӳ�������״̬
        #endif
            ){//�ӹ���λ
        for(m_index = 0; m_index < m_count; m_index++){
            mcode = tmp->GetMCode(m_index);

            if(mcode == 6){//����
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
            }else if(mcode == 2 || mcode == 30 || mcode == 99){  //ֱ�ӽ������л���READY״̬
                if(mcode == 99 && m_mode_restart.sub_prog_call > 0){
                    this->m_mode_restart.sub_prog_call--;
                    printf("reset M30, sub_call = %hhu\n", m_mode_restart.sub_prog_call);
                }
                else{
                    this->SetCurLineNo(1);
                    this->m_n_run_thread_state = STOP;
                    CompileOver();

                    m_b_ret_from_macroprog = false;
                    m_b_init_compiler_pos = false;  //��������ʼλ����Ҫ���³�ʼ��
                    this->m_p_compiler->Reset();

                    m_p_output_msg_list->Clear();

                    this->m_n_restart_mode = NOT_RESTART;
                    this->m_n_restart_line = 0;
                    this->m_n_restart_step = 0;
                    this->m_p_compiler->SetRestartPara(0, m_n_restart_mode);   //��λ�������ļӹ���λ��־

                    this->ClearHmiInfoMsg();   //���HMI����ʾ��Ϣ

                    this->m_p_f_reg->SPL = 0;
                    this->m_p_f_reg->STL = 0;
                }

            }
        }

        return true;
    }

    if(this->m_simulate_mode != SIM_NONE){  //����ģʽ
        //���õ�ǰ�к�
        SetCurLineNo(msg->GetLineNo());

        mcode = tmp->GetMCode(m_index);
        if(mcode == 2 || mcode == 30){
            ResetMcLineNo();//��λMCģ�鵱ǰ�к�
            this->SetCurLineNo(1);

            this->m_p_f_reg->STL = 0;
            this->m_p_f_reg->SPL = 0;
            this->m_p_f_reg->OP = 0;

            CompileOver();
#ifdef 	USES_SIMULATION_TEST
            if(this->m_file_sim_data > 0){
                close(this->m_file_sim_data);
                m_file_sim_data = -1;
                g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "�رշ��������ļ�");
            }
#endif
        }else if(mcode == 99){
            if(m_n_subprog_count > 0){
                m_n_subprog_count--;
                m_b_ret_from_macroprog = false;

                this->m_n_run_thread_state = RUN;
            }
            else{
                ResetMcLineNo();//��λMCģ�鵱ǰ�к�
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
                    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "�رշ��������ļ�");
                }
#endif
            }
        }
        return true;
    }

    if(tmp->IsFirstExec()){
        int limit = 3;
        if(this->IsStepMode())
            limit = 5;	//����ģʽ��Ҫ�����֤����Ϊ״̬�л�����ʱ

        //�ȴ�MC�ֿ�Ĳ岹��λ�źţ��Լ�MI�����е�λ�ź�
        int count = 0;
        bool block_over = false;
        while(1){
            block_over = CheckBlockOverFlag();
            if(this->ReadMcMoveDataCount() > 0 || !block_over ||
                    m_channel_status.machining_state == MS_PAUSED ||
                    m_channel_status.machining_state == MS_WARNING){ //δ�ﵽִ������
                //		printf("aux exec return: 0x%x, count=%d, block_over=%hhu\n", m_p_mc_comm->ReadRunOverValue(), ReadMcMoveDataCount(), block_over);
                return false;    //��δ���е�λ
            }
            else if(++count < limit){
                usleep(5000);   //�ȴ�5ms����ΪMC״̬��������Ϊ5ms����Ҫ�ȴ�״̬ȷ��
                printf("execute aus msg: blockflag=%d, count = %d\n",  block_over, count);

            }else
                break;
        }

        //		printf("isstepmode = %hhu, change_to_pause_flag = %hhu\n", this->IsStepMode(), this->m_b_need_change_to_pause);
        if(this->IsStepMode() && this->m_b_need_change_to_pause){//���Σ��л���ͣ״̬
            this->m_b_need_change_to_pause = false;
            m_n_run_thread_state = PAUSE;
            SetMachineState(MS_PAUSED);
            return false;
        }


        //���õ�ǰ�к�
        if(tmp->GetMCode(0) != 99 && !m_b_ret_from_macroprog)
            SetCurLineNo(msg->GetLineNo());
    }

    this->PreExecSpindleCmd(this->m_channel_rt_status.line_no);   //ִ������Ԥ����

    if(this->m_b_prestart_spd){ //������Ԥ����ָ������������
        if(m_count >= kMaxMCodeInLine && this->m_n_spd_prestart_step != 0xFF){
            //��ǰ��Mָ��ﵽ�����������Ԥ����δִ������ȴ���ִ���������
            return false;
        }else if(m_count < kMaxMCodeInLine){
            m_index_add = 1;
        }else{  //��ǰMָ��ﵽ���������������Ԥ����ִ�н�������λ����Ԥ������־
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
            continue;       //��ִ�������ֱ������

        mcode = tmp->GetMCode(m_index);

        switch(mcode){
        case 30:  	//M30
        case 2:		//M02
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M30\n");
                //TODO �����뷢�͸�PMC
                this->SendMCodeToPmc(mcode, m_index+m_index_add);
                if(mcode == 30)
                    this->m_p_f_reg->DM30 = 1;  //��λDM30
                else
                    this->m_p_f_reg->DM02 = 1;  //��λDM02

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //


                //���������Ҫ��������ת��������Ȧ
                //#ifdef USES_FIVE_AXIS_FUNC
                //			if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS){
                //				//��MI����������Ȧλ��ָ��
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
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index+m_index_add, true);    //��λѡͨ�ź�

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                //#ifdef USES_FIVE_AXIS_FUNC
                //			//�ȴ�MI�������
                //			if(this->m_p_chn_5axis_config->five_axis_type != NO_FIVEAXIS &&
                //					m_mask_5axis_rot_nolimit != 0){
                //				if(m_mask_5axis_rot_nolimit != m_n_mask_clear_pos){
                //					break;
                //				}
                //			}
                //#endif

                this->SetMFSig(m_index+m_index_add, false);    //��λѡͨ�ź�

                if(mcode == 30)
                    this->m_p_f_reg->DM30 = 0;  //��λDM30
                else
                    this->m_p_f_reg->DM02 = 0;  //��λDM02

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index+m_index_add);

#ifdef USES_ADDITIONAL_PROGRAM
                if(this->m_n_add_prog_type == NONE_ADD){
#endif
                    if(mcode == 30){

                        ResetMcLineNo();//��λMCģ�鵱ǰ�к�
                        this->SetCurLineNo(1);
                        printf("reset lineno : 1\n");
                    }

                    this->m_p_f_reg->STL = 0;
                    this->m_p_f_reg->SPL = 0;
                    this->m_p_f_reg->OP = 0;

                    //����������һ
                    if(this->m_channel_status.chn_work_mode == AUTO_MODE){

                        this->m_channel_status.workpiece_count++;
                        g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
                        this->m_channel_status.workpiece_count_total++;
                        g_ptr_parm_manager->SetTotalWorkPiece(m_n_channel_index, m_channel_status.workpiece_count_total);
                        this->SendWorkCountToHmi(m_channel_status.workpiece_count);  //֪ͨHMI���¼ӹ�����

                        g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC,"@#@#�ӹ�������һ��%d", m_channel_status.workpiece_count);
                        this->ResetMode();   //ģ̬�ָ�Ĭ��ֵ

                        //���������ϵ����
                        if(g_ptr_parm_manager->ActiveCoordParam(m_n_channel_index))
                            this->SetMcCoord(true);

                        //�������Ч�ĵ���ƫ������
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
                //				//��MCͬ����������
                //				this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
                //
                //				m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
                //				m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
                //
                //				this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
                //
                //		//		printf("intp pos (%lf, %lf, %lf, %lf)\n", m_channel_mc_status.intp_pos.x, m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z,
                //		//				m_channel_mc_status.intp_pos.a6);
                //
                //				//��MIͬ����е����
                //				this->m_p_channel_engine->SendMonitorData(false, false);
                //			}
                //#endif
                if(m_channel_status.chn_work_mode == AUTO_MODE){
#ifdef USES_ADDITIONAL_PROGRAM
                    if(this->m_n_add_prog_type == NONE_ADD){
                        this->SendMachOverToHmi();  //���ͼӹ�������Ϣ��HMI
                    }
#else
                    this->SendMachOverToHmi();  //���ͼӹ�������Ϣ��HMI
#endif
                }
                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬

                printf("execute M30 over\n");
            }

            break;
        case 0:		//M00��������ͣ
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M00\n");
                //TODO �����뷢�͸�PMC
                this->SendMCodeToPmc(mcode, m_index+m_index_add);
                this->m_p_f_reg->DM00 = 1;  //��λDM00

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                this->SetMFSig(m_index+m_index_add, true);    //��λѡͨ�ź�
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index+m_index_add, false);    //��λѡͨ�ź�

                this->m_p_f_reg->DM00 = 0;  //��λDM00

                this->m_p_f_reg->STL = 0;
                this->m_p_f_reg->SPL = 1;

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index+m_index_add);


                this->PauseRunGCode();
                m_n_run_thread_state = PAUSE;

                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            }
            break;
        case 1:		//M01ѡ������ͣ
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M01\n");
                //TODO �����뷢�͸�PMC
                this->SendMCodeToPmc(mcode, m_index+m_index_add);
                this->m_p_f_reg->DM01 = 1;  //��λDM01

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index+m_index_add, true);    //��λѡͨ�ź�

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index+m_index_add, false);    //��λѡͨ�ź�

                this->m_p_f_reg->DM01 = 0;  //��λDM01

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index+m_index_add);


                if(this->m_channel_status.func_state_flags.CheckMask(FS_OPTIONAL_STOP)){//ѡͣ״̬
                    PauseRunGCode();

                    m_n_run_thread_state = PAUSE;
                    this->m_p_f_reg->STL = 0;
                    this->m_p_f_reg->SPL = 1;
                }

                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            }
            break;
        case 3:		//M03������ת
            bHasSpdCmd = true;
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M03\n");
                if(this->m_b_prestart_spd){

                    if(m_n_spd_prestart_step == 0xFF)  //�ȴ�Ԥ��������
                        tmp->SetExecStep(m_index, 0xFF);   //��Ԥ������ֱ�ӽ���
                    break;
                }
                //�����뷢�͸�PMC
                this->SendMCodeToPmc(mcode, m_index+m_index_add);

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                this->m_channel_status.spindle_dir = SPD_DIR_POSITIVE;  //�޸�������ת����

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index+m_index_add, true);    //��λѡͨ�ź�

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index+m_index_add, false);    //��λѡͨ�ź�
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index+m_index_add);

                if(this->m_n_spindle_count > 0){
                    //				if(m_p_axis_config[this->m_spd_axis_phy[0]-1].spd_vctrl_mode == 2){//-10v ~ 10v
                    m_channel_status.rated_spindle_speed = m_n_cur_scode;
                    this->SendModeChangToHmi(S_MODE);
                    this->SpindleSpeedDaOut(m_channel_status.rated_spindle_speed);
                    //				}

                }
                this->m_p_f_reg->SPS = 1;    //��־������ת



                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            }
            break;
        case 4:		//M04���ᷴת
            bHasSpdCmd = true;
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M04\n");

                if(this->m_b_prestart_spd){
                    if(m_n_spd_prestart_step == 0xFF)  //�ȴ�Ԥ��������
                        tmp->SetExecStep(m_index, 0xFF);   //��Ԥ������ֱ�ӽ���
                    break;
                }

                //�����뷢�͸�PMC
                this->SendMCodeToPmc(mcode, m_index+m_index_add);

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                this->m_channel_status.spindle_dir = SPD_DIR_NEGATIVE;  //�޸�������ת����

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index+m_index_add, true);    //��λѡͨ�ź�

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index+m_index_add, false);    //��λѡͨ�ź�
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index+m_index_add);


                if(this->m_n_spindle_count > 0){
                    //	if(m_p_axis_config[this->m_spd_axis_phy[0]-1].spd_vctrl_mode == 2){//-10v ~ 10v
                    m_channel_status.rated_spindle_speed = m_n_cur_scode;
                    this->SendModeChangToHmi(S_MODE);
                    this->SpindleSpeedDaOut(m_channel_status.rated_spindle_speed);
                    //	}
                }
                this->m_p_f_reg->SPS = 2;    //��־���ᷴת


                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            }

            break;
        case 5:		//M05����ͣת
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M05\n");
                if(this->m_b_prestart_spd){
                    //TODO ��������Ԥ�����ڼ䲻�ܵ���M05
                    CreateError(ERR_SPD_PRESTART_M05, ERROR_LEVEL, CLEAR_BY_MCP_RESET, tmp->GetLineNo(), m_n_channel_index);
                    this->m_error_code = ERR_SPD_PRESTART_M05;
                    break;
                }

                //TODO �����뷢�͸�PMC
                this->SendMCodeToPmc(mcode, m_index+m_index_add);

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                this->m_channel_status.spindle_dir = SPD_DIR_STOP;  //�޸�������ת����
                m_channel_status.rated_spindle_speed = 0;
                this->SendModeChangToHmi(S_MODE);

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index+m_index_add, true);    //��λѡͨ�ź�

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index+m_index_add, false);    //��λѡͨ�ź�
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index+m_index_add);

                if(this->m_n_spindle_count > 0){
                    //	if(m_p_axis_config[this->m_spd_axis_phy[0]-1].spd_vctrl_mode == 2){//-10v ~ 10v
                    this->SpindleSpeedDaOut(m_channel_status.rated_spindle_speed);
                    //	}
                }
                this->m_p_f_reg->SPS = 0;    //��־����ͣת
                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            }
            break;
        case 6:		//M06����
            if(tmp->GetExecStep(m_index) == 0){
                //TODO �����뷢�͸�PMC
                printf("start to execute M06\n");
                this->SendMCodeToPmc(mcode, m_index+m_index_add);

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                this->SetMFSig(m_index+m_index_add, true);    //��λѡͨ�ź�
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index+m_index_add, false);    //��λѡͨ�ź�
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index+m_index_add);

                this->m_n_ref_tool_life_delay = 3;

                //				this->SetMcToolLife();

                printf("execute M06 over: cur_T = %hhu\n", m_channel_status.cur_tool);

                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            }

            break;
        case 98:	//M98�ӳ������
            printf("execute M98\n");
            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            break;
        case 99:   //M99
            if(tmp->GetExecStep(m_index) == 0){
                printf("execute M99, subprog_count = %hhu\n", m_n_subprog_count);
                if(m_n_subprog_count > 0){
                    m_n_subprog_count--;
                    m_b_ret_from_macroprog = false;
                }
                else{
                    m_p_compiler->RecycleCompile();   //��������ѭ������

                    this->m_channel_status.workpiece_count++;  //����������һ
                    g_ptr_parm_manager->SetCurWorkPiece(m_n_channel_index, m_channel_status.workpiece_count);
                    this->m_channel_status.workpiece_count_total++;
                    g_ptr_parm_manager->SetTotalWorkPiece(m_n_channel_index, m_channel_status.workpiece_count_total);
                    this->SendWorkCountToHmi(m_channel_status.workpiece_count);  //֪ͨHMI���¼ӹ�����
                }

                //TODO �����뷢�͸�PMC
                this->m_p_f_reg->DM99 = 1;  //��λDM99, ά��20ms

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                tmp->IncreaseExecStep(m_index);

            }else if(tmp->GetExecStep(m_index) == 2){
                this->m_n_run_thread_state = RUN;

                this->m_p_f_reg->DM99 = 0;
                tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬

                printf("execute M99 over\n");
            }
            break;
        case 998:	//M998 ������ͣ����
            printf("execute M998\n");
            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            break;
        case 999:  //M999 ��ͣ���룬����ͬ��λ��
            printf("execute M999\n");
            //ͬ��λ��
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

            //			m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
            //			m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
            this->RefreshAxisIntpPos();

            if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
                //			printf("coordmsg setpos, %d\n", this->m_p_output_msg_list->GetLength());
                this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
            }else
                this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //ͬ���ѱ�������ƶ�ָ���λ��

            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            break;
#ifdef USES_TWINING_FUNC
        case 560:  //������ƹ���
            ActiveTwiningFunc(mcode);

            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            printf("active twining function\n");
            break;
        case 561:  //�رղ��ƹ���
            ActiveTwiningFunc(mcode);

            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
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

            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
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
            tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬
            break;

        default:   //����M����
            if(mcode >= 40010 && mcode <= 40083){//���������ٶ�ģʽ�л�
#ifdef USES_SPEED_TORQUE_CTRL
                ProcessCtrlModeSwitch(tmp, m_index);
                break;
#endif
            }
            else if(mcode >= 41011 && mcode <= 41648){   //TODO ������ӳ���л�
                ProcessAxisMapSwitch(tmp, m_index);
                break;
            }

            if(tmp->GetExecStep(m_index) == 0){
                //TODO �����뷢�͸�PMC
                g_ptr_trace->PrintLog(LOG_ALARM, "ִ�е�M���룺M%02d, index=[%hhu,%hhu]", mcode, m_index, m_index_add);
                this->SendMCodeToPmc(mcode, m_index+m_index_add);

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //

                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 1){
                //�ȴ�TMF��ʱ����λMFѡͨ�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);
                this->SetMFSig(m_index+m_index_add, true);    //��λѡͨ�ź�
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 2){
                //�ȴ�FIN�ź�
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    gettimeofday(&m_time_m_start[m_index+m_index_add], NULL);   //��ʼ��ʱ
                else{
                    gettimeofday(&time_now, NULL);
                    time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                            time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                    if(time_elpase > kMCodeTimeout && !this->GetMExcSig(m_index+m_index_add)){//����200ms��δ����ִ��״̬����澯����֧�ֵ�M���롱
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

                //�ȴ�TFIN��ʱ����λMF�ź�
                gettimeofday(&time_now, NULL);
                time_elpase = (time_now.tv_sec-m_time_m_start[m_index+m_index_add].tv_sec)*1000000+
                        time_now.tv_usec-m_time_m_start[m_index+m_index_add].tv_usec;
                if(time_elpase < 16000)
                    break;		//δ����ʱʱ��

                this->SetMFSig(m_index+m_index_add, false);    //��λѡͨ�ź�
                tmp->IncreaseExecStep(m_index);
            }else if(tmp->GetExecStep(m_index) == 4){
                //�ȴ�FIN�źŸ�λ
                if(this->m_p_g_reg->FIN == 1 || this->GetMFINSig(m_index+m_index_add))
                    break;

                //��λ����ָ���źź�DEN�ź�
                this->SendMCodeToPmc(0, m_index+m_index_add);


                tmp->IncreaseExecStep(m_index);
            }else{
                this->ExecMCode(tmp, m_index);  //ִ��ĳЩM������Ҫϵͳִ�еĶ���

                //				tmp->SetExecStep(m_index, 0xFF);    //��λ����״̬

            }
            break;
        }
        if(tmp->GetExecStep(m_index) != 0xFF)//��δִ�н���������false
            bRet = false;
    }

    if(bRet){

        if(this->m_p_f_reg->DEN == 1)
            this->m_p_f_reg->DEN = 0;  //��λDEN�ź�

        if(bHasSpdCmd && m_b_prestart_spd){  //��λ����Ԥ������־
            m_b_prestart_spd = false;
            m_n_spd_prestart_step = 0;
            printf("spindle prestart over####3\n");
        }

    }

    return bRet;
}

#endif

/**
 * @brief ��ȡ��ǰ��������
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
 * @brief ����G12.2ĥ����̬����
 * @param intp_type : ����岹���ͣ�0-	�رգ�G13.1��   122?G12.2 123--G12.3 121 ?G12.1   71-G7.1
 * @param data_idx : ������ţ�0-3
 * @param data1 �� ����1
 * @param data2 �� ����2
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
 * @brief ʹ����ĥ
 * @param enable : true--ʹ��   false--��ֹ
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
 * @brief ִ�и�λ��ȫλ�ö���
 */
void ChannelControl::ReturnSafeState(){
    uint8_t phy_axis = 0;
    uint8_t chn_axis = 0;
    switch(m_n_ret_safe_step){
    case 0: //first step   X��ص���ȫ��
        printf("ReturnSafeState, step 0\n");
        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_X);
        if(phy_axis == NO_AXIS){
            this->m_n_ret_safe_step = 8;   //ʧ�ܣ�����
            printf("X��δ���ã�\n");
            break;
        }

        if(m_channel_mc_status.cur_mode != MC_MODE_MANUAL){  //MC��δ�л����ֶ�״̬
            break;
        }

        this->SetMcSpecialIntpMode(0);  //ȡ��ĥ��״̬

        chn_axis = this->GetChnAxisFromName(AXIS_NAME_X);
        printf("chn=%hhu, safepos = %lf, speed = %lf\n", chn_axis, m_p_grind_config->safe_pos_x,
               m_p_axis_config[phy_axis].reset_speed);
        this->ManualMove(chn_axis, this->m_p_grind_config->safe_pos_x, this->m_p_axis_config[phy_axis].reset_speed);   //X���Ը�λ�ٶȷ��ذ�ȫλ�ã���е����ϵ

        this->m_n_ret_safe_step++;
        break;
    case 1://second step    �ȴ�X�ᵽλ
        //	printf("ReturnSafeState, step 1\n");
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_X);
        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_X);

        if(fabs(m_channel_rt_status.cur_pos_machine.GetAxisValue(chn_axis) - m_p_grind_config->safe_pos_x) < 1e-3){ //��λ
            m_n_ret_safe_step++;
        }else{
            //			printf("step1: intp_pos=%lf, fb_pos = %lf\n", m_p_channel_engine->GetPhyAxisMachPosIntp(phy_axis),
            //					m_p_channel_engine->GetPhyAxisMachPosFeedback(phy_axis));
        }
        this->ManualMove(chn_axis, this->m_p_grind_config->safe_pos_x, this->m_p_axis_config[phy_axis].reset_speed);   //X���Ը�λ�ٶȷ��ذ�ȫλ�ã���е����ϵ

        break;
    case 2:{//third step   C�����
        printf("ReturnSafeState, step 2\n");
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_C);   //C���Ӧͨ����������
        if(chn_axis == NO_AXIS){
            this->m_n_ret_safe_step = 8;   //ʧ�ܣ�����
            printf("C��δ���ã�\n");
            break;
        }
        double pos = m_channel_rt_status.cur_pos_machine.GetAxisValue(chn_axis);  //��ǰC���е����

        //���������360������λ��
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
            this->m_n_ret_safe_step = 8;   //ʧ�ܣ�����
            printf("C��δ���ã�\n");
            break;
        }

        this->ManualMove(chn_axis, m_df_c_pos_tar, this->m_p_axis_config[phy_axis].reset_speed);   //C���Ը�λ�ٶȷ������λ�ã���е����ϵ

        this->m_n_ret_safe_step++;
        printf("move c axis to pos: %lf\n", m_df_c_pos_tar);
        break;
    }
    case 3://fourth step �ȴ�C�������λ
        //printf("ReturnSafeState, step 3\n");
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_C);
        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_C);
        if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(chn_axis) - m_df_c_pos_tar) >= 1e-3){ //δ��λ
            this->ManualMove(chn_axis, m_df_c_pos_tar, this->m_p_axis_config[phy_axis].reset_speed);   //C���Ը�λ�ٶȷ������λ�ã���е����ϵ
            break;
        }

        this->m_n_ret_safe_step++;
        break;
    case 4: //��ʼ����C������������Ȧ
        printf("ReturnSafeState, step 4\n");
        m_n_mask_clear_pos = 0;
        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_C);

        this->SendMiClearPosCmd(phy_axis+1, 360*1000);

        this->m_n_ret_safe_step++;
        break;
    case 5:{
        printf("ReturnSafeState, step 5\n");
        //�ȴ�MI�������
        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_C);
        uint64_t mask = 0x01<<phy_axis;
        if(mask == this->m_n_mask_clear_pos){  //
            this->m_n_ret_safe_step++;;	//��ת��һ��
        }
    }
        break;
    case 6:
        printf("ReturnSafeState, step 6\n");
        //ͬ��λ��

        //��MCͬ����������
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        else
            this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;

        this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��

        //		printf("intp pos (%lf, %lf, %lf, %lf)\n", m_channel_mc_status.intp_pos.x, m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z,
        //				m_channel_mc_status.intp_pos.a6);

        //��MIͬ����е����
        this->m_p_channel_engine->SendMonitorData(false, false);

        //		printf("cur mach pos (%lf, %lf, %lf, %lf)\n", m_channel_rt_status.cur_pos_machine.x, m_channel_rt_status.cur_pos_machine.y,
        //				m_channel_rt_status.cur_pos_machine.z, m_channel_rt_status.cur_pos_machine.a6);

        this->m_n_ret_safe_step++;  //��ת��һ��
        break;
    case 7: //����
        printf("Succeed to return safe pos!!!!!\n");
        this->m_b_ret_safe_state = false;
        this->m_n_ret_safe_step = 0;
        break;
    case 8: //����
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
 * @brief ���дӻ�����������
 */
void ChannelControl::RunM66_slave(){
    uint8_t cur_chn = this->m_p_mech_arm_state->cur_chn;   //��ǰͨ��
    int delay_time_vac = this->m_p_mech_arm_param->delay_time_vac*1000;
    int wait_overtime = this->m_p_mech_arm_param->wait_overtime*1000;   //�ȴ���ʱ
    int work_vac_delay = this->m_p_mech_arm_param->work_vac_check_delay*1000;   //��λ�������ʱ

    if(this->m_p_mech_arm_state->run_flag != MECH_ARM_IDLE){
        if(this->m_p_mech_arm_state->cur_chn != GRIND_SLAVE_CHN)//����ռ�ã��ȴ�
            return;  //�ȴ�����Ȩ
        else if(this->m_p_mech_arm_state->cur_step > 29)  //��ͨ����һ��M66��ûִ���꣬�ȴ�
            return;
    }
    else{//��ռ����Ȩ
        int err = pthread_mutex_trylock(&m_p_mech_arm_state->mutex_change_run);
        if(err == 0){//lock�ɹ�
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

            //ǿ�Ƽ�צ����
            this->m_p_f_reg->left_claw_down = 0;
            this->m_p_f_reg->right_claw_down = 0;

            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        }else
            return;
    }

    if(this->m_p_mech_arm_state->run_flag == MECH_ARM_PAUSE && m_p_mech_arm_state->cur_step > 0)
        return;

    if(m_p_mech_arm_state->cur_step == 0){  //step-0   ��ʼ����
        g_ptr_trace->PrintLog(LOG_ALARM, "ִ�еĴӻ�M���룺M66\n");

        printf("goto step 1\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 1){ //step-1  �ж�״̬��������ת����
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 300000){ //��ʱ300ms
            return;
        }

        if(m_p_mech_arm_state->total_count == 0){
            printf("���̹��Ϊ0��\n");
            return;
        }

        //��λ������������λ
        if(this->m_p_g_reg->right_work_up_check == 1){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_UP_RIGHT;   //�һ���λ������������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;  //��λ������������λ
        }

        if(this->m_p_g_reg->right_tray_inpos == 1){//���̲���λ��
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){
                this->m_p_mech_arm_state->err_state = ERR_ARM_NO_TRAY;   //���̲���λ��
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;  //���̲���λ��
        }

        //������������Ƿ�����
        if((this->m_p_f_reg->ZRF1 & (0x01<<5))==0 || (this->m_p_f_reg->ZRF1 & (0x01<<6))==0){
            //����δ�����
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){
                this->m_p_mech_arm_state->err_state = ERR_ARM_AXIS_NO_REF;
                this->m_p_channel_engine->SendHMIMechArmException(ERR_ARM_AXIS_NO_REF);  //��HMI�澯����е��δ����
            }
            return;
        }

        //����ǰ��צ�����ϵ�λ��������ʩ
        if(this->m_p_g_reg->left_claw_up_check == 0 || this->m_p_g_reg->right_claw_up_check == 0){//��צλ�ϵ�λ
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ�ϵ�λ�źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //��е����������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;
        }

        //Y�ᴦ�ڸ���λ�ã��������˳���������
        if(this->m_p_mech_arm_state->is_exit_inter_area){
            if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) -
                    m_p_mech_arm_state->y_target_pos) > 0.005){//��δ�뿪��������
                return;
            }
            this->m_p_mech_arm_state->is_exit_inter_area = false;  //��λ��־
        }else if(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) > m_p_mech_arm_param->pos_tray_base_y[GRIND_SLAVE_CHN]){
            m_p_mech_arm_state->y_target_pos = m_p_mech_arm_param->pos_tray_base_y[GRIND_SLAVE_CHN];
            this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                    m_p_mech_arm_param->speed, false);  //��������
            this->m_p_mech_arm_state->is_exit_inter_area = true;
            return;
        }

        //�����Ƿ��Ѿ�����,���ѻ���
        if(m_p_mech_arm_state->cur_index[GRIND_SLAVE_CHN] == m_p_mech_arm_state->total_count /*&&
                                                        m_p_mech_arm_state->cur_finished_count[GRIND_SLAVE_CHN] >= m_p_mech_arm_state->total_count-1*/){//��Ʒ�ѷ���
            if(this->m_p_f_reg->change_tray_right == 0){
                this->m_p_f_reg->change_tray_right = 1;   //���ѻ�����
                //				this->m_p_channel_engine->SendHMIMechArmException(ERR_NEW_TRAY_REQ);  //��HMI�����������
            }
            return;
        }


        if(this->m_p_mech_arm_state->has_material){
            if(this->m_p_mech_arm_state->repos){
                printf("jump to step 15\n");
                this->m_p_mech_arm_state->cur_step = 15;//���ϵȴ�����ֱ��ȥȡ����
                //��¼ʱ��
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            }else{//δ��λ�����ȶ�λ
                //����X��Y��λ��
                this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_corr_fine_x+
                        this->m_p_mech_arm_param->pos_corr_rough_x;
                this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_corr_fine_y+
                        this->m_p_mech_arm_param->pos_corr_rough_y;

                this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                        m_p_mech_arm_param->speed, false);  //��������
                this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                        m_p_mech_arm_param->speed, false);  //��������

                //��λ�����ɿ�
                this->m_p_f_reg->correct_pos_x = 0;
                this->m_p_f_reg->correct_pos_y = 0;

                this->m_p_mech_arm_state->corr_index = 0;  //��λ��λ����

                //��¼ʱ��
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

                printf("jump to step 7\n");
                this->m_p_mech_arm_state->cur_step = 7;  //ȥ��λ
            }
        }else if(this->m_p_mech_arm_state->cur_index[cur_chn] < this->m_p_mech_arm_state->total_count){
            printf("goto step 2\n");
            m_p_mech_arm_state->cur_step = 2;//��ȥȡ��
        }else{//�����Ѿ����Ͽ�ȡ��ֱ����תȥȡ��Ʒ
            printf("goto step 15\n");
            this->m_p_mech_arm_state->cur_step = 15;//��ë����ȡ����ֱ��ȥȡ����
            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
        }
    }else if(m_p_mech_arm_state->cur_step == 2){ //step-2 X/Y��ͬʱ��ʼ�ƶ���ȡ��λ��
        //����X��Y��Ŀ��λ��
        uint8_t row = m_p_mech_arm_state->cur_index[cur_chn]/m_p_mech_arm_param->col_count;   //��ǰ��
        uint8_t col = m_p_mech_arm_state->cur_index[cur_chn]%m_p_mech_arm_param->col_count;	  //��ǰ��
        m_p_mech_arm_state->x_target_pos =
                m_p_mech_arm_param->pos_tray_base_x[cur_chn] + m_p_mech_arm_param->dis_tray_x * col;
        m_p_mech_arm_state->y_target_pos =
                m_p_mech_arm_param->pos_tray_base_y[cur_chn] - m_p_mech_arm_param->dis_tray_y * row;

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                m_p_mech_arm_param->speed, false);  //��������
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                m_p_mech_arm_param->speed, false);  //��������

        printf("goto step 3, x:%lf, y:%lf\n", m_p_mech_arm_state->x_target_pos, m_p_mech_arm_state->y_target_pos);
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 3){  //step-3  �ȴ������е�λ�����צ�£�ȡ�ϣ�
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005
                || fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //û�е�λ

        //��צ��
        this->m_p_f_reg->left_claw_down = 1;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 4\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 4){ //step-4  ��ʱ300ms�ȴ���צ�µ�λ�������
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 300000){ //��ʱ300ms
            return;
        }

        //����צ���
        this->m_p_f_reg->left_claw_vac = 1;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 5\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 5){ //step-5  ȷ����צ����źţ���צ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;


        if(this->m_p_g_reg->left_claw_vac_check == 0 && time_elpase < wait_overtime)  //�ȴ���צ����ź�
            return;

        //��צ��
        this->m_p_f_reg->left_claw_down = 0;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 6\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 6){ //step-6   ȷ����צ�ϵ�λ��X��Yͬʱ�ƶ����ֶ�λλ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 800000)  //��ʱ800ms����ֹ����������ź�
            return;

        if(this->m_p_g_reg->left_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ�ϵ�λ�źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //��е����������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;  //��צ��δ��λ
        }

        if(this->m_p_g_reg->left_claw_vac_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ����źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_FETCH_VAC;   //����ȡ��ʧ�ܣ�������쳣
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //��HMI�����쳣
            }
            return;  //��צ����ź��쳣
        }
        //		printf("run slave m66, left_vac=%hhu, left_up=%hhu, over=%d\n", m_p_g_reg->left_claw_vac_check,
        //				m_p_g_reg->left_claw_up_check, wait_overtime);


        this->m_p_mech_arm_state->cur_index[cur_chn]++;  //��ǰȡ����ż�һ

        //����X��Y��λ��
        this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_corr_fine_x+
                this->m_p_mech_arm_param->pos_corr_rough_x;
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_corr_fine_y+
                this->m_p_mech_arm_param->pos_corr_rough_y;

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                m_p_mech_arm_param->speed, false);  //��������
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                m_p_mech_arm_param->speed, false);  //��������

        //��λ�����ɿ�
        this->m_p_f_reg->correct_pos_x = 0;
        this->m_p_f_reg->correct_pos_y = 0;

        this->m_p_mech_arm_state->corr_index = 0;  //��λ��λ����

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 7�� x=%lf, y=%lf\n", m_p_mech_arm_state->x_target_pos, m_p_mech_arm_state->y_target_pos);
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 7){  //step-7    �ȴ��ᵽλ����צ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005
                || fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //û�е�λ

        if(this->m_p_g_reg->correct_pos_x_check == 0 || this->m_p_g_reg->correct_pos_y_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���λ�����ɿ��źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_CORR_POS;   //��λʧ�ܣ���λ���ײ���λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;   //��λ�����ɿ�δ���
        }

        this->m_p_f_reg->left_claw_down = 1;  //��צ��

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 8\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 8){  //step-8 ��ʱ�ȴ���צ�µ�λ������צ���
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 200000){ //��ʱ200ms
            return;
        }

        this->m_p_f_reg->left_claw_vac = 0;  //����צ���

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 9\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 9){	//step-9  ��ʱ300ms�����צ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        //		if(time_elpase < delay_time_vac){ //��ʱ300ms
        //			return;
        //		}

        if(this->m_p_g_reg->left_claw_vac_check == 1 && time_elpase < wait_overtime)
            return;

        this->m_p_f_reg->left_claw_down = 0;  //���צ��

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 10\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 10){  //step-10 �ȴ����צ�ϵ�λ����λ���׼н���X��Y���ƶ�������λλ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //��ʱ500ms
            return;
        }

        if(this->m_p_g_reg->left_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ�ϵ�λ�źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //��е����������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;  //��צ��δ��λ
        }

        //��λ���׼н�
        this->m_p_f_reg->correct_pos_x = 1;
        this->m_p_f_reg->correct_pos_y = 1;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        //����X��Y��Ŀ��λ��
        this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_corr_fine_x;
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_corr_fine_y;

        //ʹ�ù̶����٣���ֹ����
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                2000, false);  //��������
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                2000, false);  //��������

        printf("goto step 11�� x=%lf, y=%lf\n", m_p_mech_arm_state->x_target_pos, m_p_mech_arm_state->y_target_pos);
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 11){ //step-11 ��ʱ1000ms�����ȴ�X��Y�ᵽλ����צ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 1000000){ //��ʱ1000ms
            return;
        }

        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005
                || fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //û�е�λ

        if(++this->m_p_mech_arm_state->corr_index < this->m_p_mech_arm_param->corr_pos_count){//��λ��������
            //��λ�����ɿ�
            this->m_p_f_reg->correct_pos_x = 0;
            this->m_p_f_reg->correct_pos_y = 0;

            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

            printf("repos, jump to step 10\n");
            m_p_mech_arm_state->cur_step = 10;
            return;
        }

        m_p_mech_arm_state->corr_index = 0;

        this->m_p_f_reg->left_claw_down = 1;   //��צ��

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 12\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 12){  //step-12  ��ʱ500ms������צ���
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //��ʱ500ms
            return;
        }

        this->m_p_f_reg->left_claw_vac = 1;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);

        printf("goto step 13\n");
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 13){ //step-13  ��ʱ300ms��ȷ����צ����źţ���λ�����ɿ�
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        //		if(time_elpase < 300000){ //��ʱ300ms
        //			return;
        //		}

        if(this->m_p_g_reg->left_claw_vac_check == 0 && time_elpase < wait_overtime){
            //��¼ʱ��2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
            return;   //��צ����ź�δȷ��
        }

        time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm_2.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm_2.tv_usec;
        if(time_elpase < 500000){ //��ʱ500ms
            return;
        }


        //��λ�����ɿ�
        this->m_p_f_reg->correct_pos_x = 0;
        this->m_p_f_reg->correct_pos_y = 0;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 14\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 14){ //step-14  ȷ�϶�λ�����ɿ���λ�����צ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->correct_pos_x_check == 0 || this->m_p_g_reg->correct_pos_y_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���λ�����ɿ���λ�źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_CORR_POS;   //��λʧ�ܣ���λ���ײ���λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;   //��λ�����ɿ�δ���
        }

        this->m_p_f_reg->left_claw_down = 0;

        this->m_p_mech_arm_state->has_material = true;   //��צ����
        this->m_p_mech_arm_state->repos = true;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 15\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 15){ //step-15  ��ʱ1s��ȷ����צ�ϵ�λ���ٴ�ȷ����צ����źţ��ƶ�X�ᵽȡ�ϣ����ϣ�λ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 1000000){ //��ʱ1000ms
            return;
        }
        if(this->m_p_g_reg->left_claw_up_check == 0 ||
                this->m_p_g_reg->right_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ�ϵ�λ�źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //��е����������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;
        }

        if(this->m_p_mech_arm_state->has_material && this->m_p_g_reg->left_claw_vac_check == 0 ){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ����źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_CORR_VAC;   //��λʧ�ܣ�������쳣
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;
        }

        //���´ӻ���λ״̬���Ƿ����ϣ���λ����ռ�����ź���λ���ϣ���������
        if(this->m_p_g_reg->right_work_vac_check == 1)
            this->m_p_mech_arm_state->work_material[GRIND_SLAVE_CHN] = true;  //��λ����
        else
            this->m_p_mech_arm_state->work_material[GRIND_SLAVE_CHN] = false;  //��λ����

        //����X��Ŀ��λ��
        if(this->m_p_mech_arm_state->work_material[cur_chn]){//�ӹ�λ���ϣ�ȡ��λ��
            this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_work_x[cur_chn] -
                    this->m_p_mech_arm_param->dis_mech_arm;
        }else{//�ӹ�λû���ϣ�ֱ������
            this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_work_x[cur_chn];
        }

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                m_p_mech_arm_param->speed, false);  //��������

        printf("goto step 16, X=%lf\n", this->m_p_mech_arm_state->x_target_pos);
        m_p_mech_arm_state->cur_step++;
    }else if(m_p_mech_arm_state->cur_step == 16){ //step-16  �ȴ�X�ᵽλ��ȷ�ϼӹ�λ����ź��Լ�ѹ������������λ�źţ��ƶ�Y�ᵽȡ��λ
        //		if(this->m_p_g_reg->work_up_check == 0){
        //			this->m_p_f_reg->work_hold = 0;  //ѹ����������
        //			return;
        //		}
        //		if(this->m_p_mech_arm_state->work_material[cur_chn] &&
        //				this->m_p_g_reg->right_work_vac_check == 0){  //�ӹ�λ������ȷ������ź�
        //			if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){  //�ȴ���λ����ź�
        //				this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_FETCH_VAC;   //��λȡ��ʧ�ܣ�������쳣
        //				this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
        //			}
        //			return;
        //		}

        //X���Ƿ�λ
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005){
            return;   //û�е�λ
        }

        //��λ������������λ
        if(this->m_p_g_reg->right_work_up_check == 1){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_UP_RIGHT;   //�һ���λ������������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;  //��λ������������λ
        }

        double spd = m_p_mech_arm_param->speed;
        if(!this->m_p_mech_arm_state->work_material[cur_chn]){//��λ�޳�Ʒ��ֱ�ӷ��ϣ��ٶȼ���
            spd = spd/2;
        }
        //����Y��Ŀ��λ��
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_work_y[cur_chn];

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                spd, false);  //��������

        printf("goto step 17\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 17){ //step-17  �ȴ�Y�ᵽλ��ȡ������צ�£���������צ��
        //Y���Ƿ�λ
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005){
            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            return;   //û�е�λ
        }

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//ȡ��
            this->m_p_f_reg->right_claw_down = 1;
        }else{
            struct timeval time_now;
            gettimeofday(&time_now, NULL);
            int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                    time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
            if(time_elpase < 1000000){ //��ʱ1000ms
                return;
            }
            this->m_p_f_reg->left_claw_down = 1;  //����
        }

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 18\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 18){ //step-18  ��ʱ300ms��ȡ�Ͽ���צ��գ����Ͽ��ӹ�λ���
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;


        if(this->m_p_mech_arm_state->work_material[cur_chn]){//ȡ�ϣ�����צ���
            if(time_elpase < 300000){ //��ʱ300ms
                return;
            }
            this->m_p_f_reg->right_claw_vac = 1;
        }else{
            this->m_p_f_reg->work_vac_right = 1;  //���ϣ�֪ͨ�һ����ӹ�λ���
            //��¼ʱ��2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
        }

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 19\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 19){ //step-19  ȡ��ȷ��������źź�ؼӹ�λ��գ�����ȷ�ϼӹ�λ����źź����צ���
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//ȡ��
            if(this->m_p_g_reg->right_claw_vac_check == 0 && time_elpase < wait_overtime)
                return;

            this->m_p_f_reg->work_vac_right = 0;   //֪ͨ�һ��ؼӹ�λ���
        }else{//����
            if(this->m_p_g_reg->right_work_vac_check == 0 && time_elpase < wait_overtime){  //ȷ���һ��ӹ�λ����ź�
                //��¼ʱ��2
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
                return;
            }

            time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm_2.tv_sec)*1000000+
                    time_now.tv_usec-m_p_mech_arm_state->time_start_arm_2.tv_usec;
            if(time_elpase < work_vac_delay){ //��λ�������ʱ���ٹ���צ���
                return;
            }

            this->m_p_f_reg->left_claw_vac = 0;
        }

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 20\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 20){ //step-20  ȡ��ȷ�ϼӹ�λ���ȷ���ź���ʧ����ʱ300ms������ȷ����צ����ź���ʧ����ʱ200ms
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//ȡ��
            if(this->m_p_g_reg->right_work_vac_check == 1 && time_elpase < wait_overtime)
                return;
        }else{//����
            if(this->m_p_g_reg->left_claw_vac_check == 1 && time_elpase < wait_overtime)
                return;
        }

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 21\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 21){ //step-21   ��ʱ300ms��ȡ������צ�ϣ���������צ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < delay_time_vac){ //��ʱ300ms
            return;
        }

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//ȡ��
            this->m_p_f_reg->right_claw_down = 0;

            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            //��¼ʱ��2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
        }else{//����
            this->m_p_f_reg->left_claw_down = 0;
            this->m_p_mech_arm_state->has_material = false;   //��צ����
            this->m_p_mech_arm_state->repos = false;

            this->m_p_mech_arm_state->work_material[cur_chn] = true;    //�ӹ�λ����

            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

            //��ת28��
            printf("21 jump to step 28\n");
            m_p_mech_arm_state->cur_step = 28;
            return;
        }

        printf("goto step 22\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 22){ //step-22  ȷ����צ������λ����ȷ������ź�, �����צ�������ƶ�X��ȥ���ϣ������������ת28��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->right_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ�ϵ�λ�ź�
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //��е����������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }

            //��¼ʱ��2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);

            return;
        }

        time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm_2.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm_2.tv_usec;
        if(time_elpase < 600000){   //��צ������λ������ʱ600ms�����צ������ź�
            return;
        }

        if(this->m_p_g_reg->right_claw_vac_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ����ź�
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_FETCH_VAC;   //��λȡ��ʧ�ܣ�������쳣
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;
        }

        this->m_p_mech_arm_state->work_material[cur_chn] = false;  //�ӹ�λ����
        this->m_p_mech_arm_state->has_finished_material = true;  //��צ�г�Ʒ

        if(this->m_p_mech_arm_state->has_material){  //��צ����
            this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_work_x[cur_chn];

            this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                    m_p_mech_arm_param->speed/2, false);  //��������
        }else{ //��צ���ϣ���ת
            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

            //��ת28��
            printf("22 jump to step 28\n");
            m_p_mech_arm_state->cur_step = 28;
            return;
        }

        printf("goto step 23\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 23){ //step-23   �ȴ�X�ᵽλ����צ��
        //X���Ƿ�λ
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005){
            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            return;   //û�е�λ
        }

        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 1000000){ //��ʱ1000ms
            return;
        }

        this->m_p_f_reg->left_claw_down = 1;

        //��¼ʱ��
        //		gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 24\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 24){ //step-24  ��ʱ200ms�����Ҽӹ�λ���
        //		struct timeval time_now;
        //		gettimeofday(&time_now, NULL);
        //		unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
        //				time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        //		if(time_elpase < 300000){ //��ʱ300ms
        //			return;
        //		}

        this->m_p_f_reg->work_vac_right = 1;  //����λ���

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
        //��¼ʱ��2
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);


        printf("goto step 25\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 25){ //step-25  ȷ���Ҽӹ�λ����źţ�����צ���
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;


        if(this->m_p_g_reg->right_work_vac_check == 0 && time_elpase < wait_overtime){
            //��¼ʱ��2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);

            return;
        }

        if(time_elpase < work_vac_delay){ //��ʱ500ms���ȴ���λ������ȶ�
            return;
        }

        this->m_p_f_reg->left_claw_vac = 0;

        this->m_p_mech_arm_state->work_material[cur_chn] = true;    //�ӹ�λ����

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 26\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 26){ //step-26  �ȴ���צ����ź���ʧ
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->left_claw_vac_check == 1 && time_elpase < wait_overtime)
            return;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 27\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 27){ //step-27  ��ʱ200ms�� ��צ����
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < delay_time_vac){ //��ʱ200ms
            return;
        }

        this->m_p_f_reg->left_claw_down = 0;

        this->m_p_mech_arm_state->has_material = false;   //��צ����
        this->m_p_mech_arm_state->repos = false;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 28\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 28){ //step-28  ȷ������צ������λ���ƶ�Y�ᵽ���̻�׼λ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //��ʱ500ms
            return;
        }

        if(this->m_p_g_reg->left_claw_up_check == 0 || this->m_p_g_reg->right_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ�ϵ�λ�ź�
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //��е����������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;
        }

        //��鹤λ������ź�
        if(this->m_p_mech_arm_state->work_material[cur_chn] &&
                this->m_p_g_reg->right_work_vac_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_SET_VAC;   //��λ����ʧ�ܣ�������쳣
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;
        }

        //����Y��Ŀ��λ��
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_tray_base_y[cur_chn];

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                m_p_mech_arm_param->speed, false);  //��������

        printf("goto step 29\n");
        m_p_mech_arm_state->cur_step++;

    }else if(m_p_mech_arm_state->cur_step == 29){ //step-29   �ȴ�Y�ᵽλ
        //Y���Ƿ�λ
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //û�е�λ

        this->m_p_f_reg->m66_runover_right = 1; //֪ͨ�ӻ���������M66ָ��ִ�����

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("succeed to execute M66, goto step 30\n");
        m_p_mech_arm_state->cur_step++;

    }
}

/**
 * @brief ����M66ָ���������������
 * @param msg : ��Ϣָ��
 */
void ChannelControl::ProcessGrindM66(AuxMsg *msg){
    if(msg == nullptr)
        return;

    if(this->m_p_mech_arm_state->run_flag == MECH_ARM_PAUSE && msg->GetExecStep() > 0)
        return;

    if(msg->GetExecStep() > 0 && this->m_p_mech_arm_state->err_state != ERR_ARM_NONE)    //���ڴ���״̬�򷵻أ�����
        return;

    uint8_t cur_chn = this->m_p_mech_arm_state->cur_chn;   //��ǰͨ��
    int delay_time_vac = this->m_p_mech_arm_param->delay_time_vac*1000;
    int wait_overtime = this->m_p_mech_arm_param->wait_overtime*1000;   //�ȴ���ʱ
    int work_vac_delay = this->m_p_mech_arm_param->work_vac_check_delay*1000;   //��λ�������ʱ
    if(msg->GetExecStep() == 0){  //step-0   ��ʼ���裬���ܵȴ�
        g_ptr_trace->PrintLog(LOG_ALARM, "ִ�е�M���룺M%02d", msg->GetMCode());

        printf("goto step 1\n");
        //		m_p_mech_arm_state->cur_step++;//�˴�û��ȡ������Ȩ�������޸�cur_step
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 1){ //step-1   ��ȡ��е�ֿ���Ȩ
        if(this->m_p_mech_arm_state->run_flag != MECH_ARM_IDLE){
            if(this->m_p_mech_arm_state->cur_chn != GRIND_MAIN_CHN)
                return;   //�Ѿ���ռ�ã��ȴ�
            else if(this->m_p_mech_arm_state->cur_step > 29)  //��һ��M66��ûִ���꣬�ȴ�
                return;
        }else{
            int err = pthread_mutex_trylock(&m_p_mech_arm_state->mutex_change_run);
            if(err == 0){//lock�ɹ�
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

                //ǿ�Ƽ�צ����
                this->m_p_f_reg->left_claw_down = 0;
                this->m_p_f_reg->right_claw_down = 0;

                //��¼ʱ��
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

                this->m_p_mech_arm_state->msg = msg;
            }else
                return;
        }

        if(this->m_p_g_reg->left_tray_inpos == 1){//���̲���λ��
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){
                this->m_p_mech_arm_state->err_state = ERR_ARM_NO_TRAY;   //���̲���λ��
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;  //���̲���λ��
        }

        //������������Ƿ�����
        if((this->m_p_f_reg->ZRF1 & (0x01<<5))==0 || (this->m_p_f_reg->ZRF1 & (0x01<<6))==0){
            //����δ�����
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){
                this->m_p_mech_arm_state->err_state = ERR_ARM_AXIS_NO_REF;
                this->m_p_channel_engine->SendHMIMechArmException(ERR_ARM_AXIS_NO_REF);  //��HMI�澯����е��δ����
            }
            return;
        }

        if(m_p_mech_arm_state->total_count == 0){
            printf("���̹��Ϊ0��\n");
            return;
        }

        //����ǰ��צ�����ϵ�λ��������ʩ
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 300000){ //��ʱ300ms
            return;
        }
        if(this->m_p_g_reg->left_claw_up_check == 0 || this->m_p_g_reg->right_claw_up_check == 0){//��צλ�ϵ�λ
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ�ϵ�λ�źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //��е����������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;
        }

        //��λ������������λ
        if(this->m_p_g_reg->work_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_UP_LEFT;   //�����λ������������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;  //��λ������������λ
        }

        //Y�ᴦ�ڸ���λ�ã��������˳���������
        if(this->m_p_mech_arm_state->is_exit_inter_area){
            if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) -
                    m_p_mech_arm_state->y_target_pos) > 0.005){//��δ�뿪��������
                return;
            }
            this->m_p_mech_arm_state->is_exit_inter_area = false;  //��λ��־
        }else if(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) > m_p_mech_arm_param->pos_tray_base_y[GRIND_MAIN_CHN]){
            m_p_mech_arm_state->y_target_pos = m_p_mech_arm_param->pos_tray_base_y[GRIND_MAIN_CHN];
            this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                    m_p_mech_arm_param->speed, false);  //��������
            this->m_p_mech_arm_state->is_exit_inter_area = true;
            return;
        }


        //�����Ƿ��Ѿ�����,���ѻ���
        if(m_p_mech_arm_state->cur_index[GRIND_MAIN_CHN] >= m_p_mech_arm_state->total_count /*&&
                                                        m_p_mech_arm_state->cur_finished_count[GRIND_MAIN_CHN] >= (m_p_mech_arm_state->total_count-1)*/){//��Ʒ�ѷ���
            if(this->m_p_f_reg->change_tray_left == 0){
                this->m_p_f_reg->change_tray_left = 1;   //���ѻ�����
                this->m_p_channel_engine->SendHMIMechArmException(ERR_NEW_TRAY_REQ);  //��HMI�����������
            }
            return;
        }


        //		printf("main step-1 index=%hu, fin=%hu, total=%hu\n", m_p_mech_arm_state->cur_index[GRIND_MAIN_CHN],
        //				m_p_mech_arm_state->cur_finished_count[GRIND_MAIN_CHN], m_p_mech_arm_state->total_count);

        if(this->m_p_mech_arm_state->has_material){
            if(this->m_p_mech_arm_state->repos){ //�Ѿ���λ
                printf("jump to step 15\n");
                this->m_p_mech_arm_state->cur_step = 15;

                //��¼ʱ��
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
                msg->SetExecStep(15);  //���ϵȴ�����ֱ��ȥȡ����
            }else{//δ��λ�����ȶ�λ
                //����X��Y��λ��
                this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_corr_fine_x+
                        this->m_p_mech_arm_param->pos_corr_rough_x;
                this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_corr_fine_y+
                        this->m_p_mech_arm_param->pos_corr_rough_y;

                this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                        m_p_mech_arm_param->speed, false);  //��������
                this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                        m_p_mech_arm_param->speed, false);  //��������

                //��λ�����ɿ�
                this->m_p_f_reg->correct_pos_x = 0;
                this->m_p_f_reg->correct_pos_y = 0;

                this->m_p_mech_arm_state->corr_index = 0;  //��λ��λ����
                //��¼ʱ��
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

                printf("jump to step 7\n");
                this->m_p_mech_arm_state->cur_step = 7;
                msg->SetExecStep(7);  //ȥ��λ
            }
        }else if(this->m_p_mech_arm_state->cur_index[GRIND_MAIN_CHN] < this->m_p_mech_arm_state->total_count){
            printf("goto step 2\n");
            m_p_mech_arm_state->cur_step = 2;
            msg->IncreaseExecStep();   //��ȥȡë����
        }else if(this->m_p_mech_arm_state->work_material[GRIND_MAIN_CHN] == 1){//�����Ѿ����Ͽ�ȡ��ֱ����תȥȡ��Ʒ
            //			printf("goto step 15: cur_index = %hu, fin=%hu\n", m_p_mech_arm_state->cur_index[GRIND_MAIN_CHN],
            //					m_p_mech_arm_state->cur_finished_count[GRIND_MAIN_CHN]);
            this->m_p_mech_arm_state->cur_step = 15;
            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            msg->SetExecStep(15);  //���ϵȴ�����ֱ��ȥȡ����
        }else{//����״̬�����޳�Ʒ��ȡ��Ҳ��ë����ȡ
            printf("error state!\n");
            return;
        }
    }else if(msg->GetExecStep() == 2){ //step-2 X/Y��ͬʱ��ʼ�ƶ���ȡ��λ��
        //����X��Y��Ŀ��λ��
        uint8_t row = m_p_mech_arm_state->cur_index[cur_chn]/m_p_mech_arm_param->col_count;   //��ǰ��
        uint8_t col = m_p_mech_arm_state->cur_index[cur_chn]%m_p_mech_arm_param->col_count;	  //��ǰ��
        m_p_mech_arm_state->x_target_pos =
                m_p_mech_arm_param->pos_tray_base_x[cur_chn] + m_p_mech_arm_param->dis_tray_x * col;
        m_p_mech_arm_state->y_target_pos =
                m_p_mech_arm_param->pos_tray_base_y[cur_chn] - m_p_mech_arm_param->dis_tray_y * row;

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                m_p_mech_arm_param->speed, false);  //��������
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                m_p_mech_arm_param->speed, false);  //��������

        printf("goto step 3, x:%lf, y:%lf\n", m_p_mech_arm_state->x_target_pos, m_p_mech_arm_state->y_target_pos);
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 3){  //step-3  �ȴ������е�λ�����צ�£�ȡ�ϣ�
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005
                || fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //û�е�λ

        //��צ��
        this->m_p_f_reg->left_claw_down = 1;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 4\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 4){ //step-4  ��ʱ300ms�ȴ���צ�µ�λ�������
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 300000){ //��ʱ300ms
            return;
        }

        //����צ���
        this->m_p_f_reg->left_claw_vac = 1;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 5\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 5){ //step-5  ȷ����צ����źţ���צ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->left_claw_vac_check == 0 && time_elpase < wait_overtime)  //ȷ����צ����źţ����Ҳ���ʱ
            return;

        //��צ��
        this->m_p_f_reg->left_claw_down = 0;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 6\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 6){ //step-6   ȷ����צ�ϵ�λ��X��Yͬʱ�ƶ����ֶ�λλ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 800000)  //��ʱ800ms����ֹ����������ź�
            return;

        if(this->m_p_g_reg->left_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ�ϵ�λ�źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //��е����������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state);   //��HMI�����쳣
            }
            return;
        }
        if(this->m_p_g_reg->left_claw_vac_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ����źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_FETCH_VAC;   //����ȡ��ʧ�ܣ�������쳣
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //��HMI�����쳣
            }
            return;  //��צ����ź��쳣
        }

        this->m_p_mech_arm_state->cur_index[cur_chn]++;  //��ǰȡ����ż�һ

        //����X��Y��λ��
        this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_corr_fine_x+
                this->m_p_mech_arm_param->pos_corr_rough_x;
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_corr_fine_y+
                this->m_p_mech_arm_param->pos_corr_rough_y;

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                m_p_mech_arm_param->speed, false);  //��������
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                m_p_mech_arm_param->speed, false);  //��������

        //��λ�����ɿ�
        this->m_p_f_reg->correct_pos_x = 0;
        this->m_p_f_reg->correct_pos_y = 0;

        this->m_p_mech_arm_state->corr_index = 0;  //��λ��λ����

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 7�� x=%lf, y=%lf\n", m_p_mech_arm_state->x_target_pos, m_p_mech_arm_state->y_target_pos);
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 7){  //step-7    �ȴ��ᵽλ����צ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005
                || fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //û�е�λ


        if(this->m_p_g_reg->correct_pos_x_check == 0 || this->m_p_g_reg->correct_pos_y_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���λ�����ɿ���λ�ź�
                this->m_p_mech_arm_state->err_state = ERR_ARM_CORR_POS;   //��λʧ�ܣ���λ���ײ���λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //��HMI�����쳣
            }
            return;   //��λ�����ɿ�δ���
        }

        this->m_p_f_reg->left_claw_down = 1;  //��צ��

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 8\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 8){  //step-8 ��ʱ�ȴ���צ�µ�λ�������
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //��ʱ500ms
            return;
        }

        this->m_p_f_reg->left_claw_vac = 0;  //�����

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 9\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 9){	//step-9  ��ʱ800ms�����צ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < delay_time_vac){ //��ʱ800ms
            return;
        }
        if(this->m_p_g_reg->left_claw_vac_check == 1 && time_elpase < wait_overtime)
            return;

        this->m_p_f_reg->left_claw_down = 0;  //���צ��

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 10\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 10){  //step-10 �ȴ����צ�ϵ�λ����λ���׼н���X��Y���ƶ�������λλ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //��ʱ500ms
            return;
        }

        if(this->m_p_g_reg->left_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ�ϵ�λ�źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //��е����������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //��HMI�����쳣
            }
            return;  //��צ��δ��λ
        }


        //��λ���׼н�
        this->m_p_f_reg->correct_pos_x = 1;
        this->m_p_f_reg->correct_pos_y = 1;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        //����X��Y��Ŀ��λ��
        this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_corr_fine_x;
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_corr_fine_y;

        //ʹ�ù̶����٣���ֹ����
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                2000, false);  //��������
        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                2000, false);  //��������

        printf("goto step 11�� x=%lf, y=%lf\n", m_p_mech_arm_state->x_target_pos, m_p_mech_arm_state->y_target_pos);
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 11){ //step-11 ��ʱ1000ms����ȷ��XY�ᵽλ����צ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 1000000){ //��ʱ1000ms
            return;
        }

        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005
                || fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //û�е�λ

        if(++this->m_p_mech_arm_state->corr_index < this->m_p_mech_arm_param->corr_pos_count){//��λ��������
            //��λ�����ɿ�
            this->m_p_f_reg->correct_pos_x = 0;
            this->m_p_f_reg->correct_pos_y = 0;

            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

            printf("repos, jump to step 10\n");
            m_p_mech_arm_state->cur_step = 10;
            msg->SetExecStep(10);
            return;
        }

        m_p_mech_arm_state->corr_index = 0;

        this->m_p_f_reg->left_claw_down = 1;   //��צ��

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 12\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 12){  //step-12  ��ʱ500ms������צ���
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //��ʱ500ms
            return;
        }
        this->m_p_f_reg->left_claw_vac = 1;


        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);

        printf("goto step 13\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 13){ //step-13  ��ʱ200ms�� ȷ����צ����źţ�ȷ�Ϻ�����ʱ500ms����λ�����ɿ�
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        //		if(time_elpase < 200000){ //��ʱ200ms
        //			return;
        //		}

        if(this->m_p_g_reg->left_claw_vac_check == 0 && time_elpase < wait_overtime){
            //��¼ʱ��2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
            return;   //��צ����ź�δȷ��
        }

        time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm_2.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm_2.tv_usec;
        if(time_elpase < 500000){ //��ʱ500ms
            return;
        }

        //��λ�����ɿ�
        this->m_p_f_reg->correct_pos_x = 0;
        this->m_p_f_reg->correct_pos_y = 0;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 14\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 14){ //step-14   ȷ�϶�λ�����ɿ��źţ����צ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->correct_pos_x_check == 0 || this->m_p_g_reg->correct_pos_y_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���λ�����ɿ���λ��ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_CORR_POS;   //��λʧ�ܣ���λ���ײ���λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //��HMI�����쳣
            }
            return;   //��λ�����ɿ�δ���
        }

        this->m_p_f_reg->left_claw_down = 0;   //��צ��

        this->m_p_mech_arm_state->has_material = true;   //��צ����
        this->m_p_mech_arm_state->repos = true;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 15\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 15){ //step-15  ��ʱ1000ms��ȷ����צ�ϵ�λ���ٴ�ȷ����צ����źţ��ƶ�X�ᵽȡ�ϣ����ϣ�λ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 1000000){ //��ʱ1000ms
            return;
        }

        //���¹�λ״̬���Ƿ����ϣ���λ����ռ�����ź���λ���ϣ���������
        printf("refresh work vac: out=%hhu, in=%hhu\n", m_p_f_reg->work_vac,
               m_p_g_reg->work_vac_check);
        if(this->m_p_f_reg->work_vac == 1 && m_p_g_reg->work_vac_check == 1){
            this->m_p_mech_arm_state->work_material[GRIND_MAIN_CHN] = true;  //��λ����
        }else{
            this->m_p_mech_arm_state->work_material[GRIND_MAIN_CHN] = false;  //��λ����
        }

        if(this->m_p_g_reg->left_claw_up_check == 0 ||
                this->m_p_g_reg->right_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ�����צ������λ��ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //��е����������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //��HMI�����쳣
            }
            return;
        }


        if(this->m_p_mech_arm_state->has_material && this->m_p_g_reg->left_claw_vac_check == 0 ){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //������쳣
                this->m_p_mech_arm_state->err_state = ERR_ARM_CORR_VAC;   //��λʧ�ܣ�������쳣
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //��HMI�����쳣
            }
            return;
        }

        //����X��Ŀ��λ��
        if(this->m_p_mech_arm_state->work_material[cur_chn]){//�ӹ�λ���ϣ�ȡ��λ��
            this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_work_x[cur_chn] -
                    this->m_p_mech_arm_param->dis_mech_arm;
        }else{//�ӹ�λû���ϣ�ֱ������
            this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_work_x[cur_chn];
        }

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                m_p_mech_arm_param->speed, false);  //��������

        printf("goto step 16, X=%lf\n", this->m_p_mech_arm_state->x_target_pos);
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 16){ //step-16  �ȴ�X�ᵽλ��ȷ�ϼӹ�λ����ź��Լ�ѹ������������λ�źţ��ƶ�Y�ᵽȡ��λ
        //X���Ƿ�λ
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005)
            return;   //û�е�λ

        double spd = m_p_mech_arm_param->speed;  //�ٶ�
        if(this->m_p_g_reg->work_up_check == 0){
            this->m_p_f_reg->work_hold = 0;  //ѹ����������
            return;
        }

        if(this->m_p_mech_arm_state->work_material[cur_chn] &&
                this->m_p_g_reg->work_vac_check == 0)  //�ӹ�λ������ȷ������ź�
            return;

        if(!this->m_p_mech_arm_state->work_material[cur_chn]){
            spd = spd/2;
        }

        //����Y��Ŀ��λ��
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_work_y[cur_chn];

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                spd, false);  //��������

        printf("goto step 17\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 17){ //step-17  �ȴ�Y�ᵽλ��ȡ������צ�£���������צ��
        //Y���Ƿ�λ
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005){
            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            return;   //û�е�λ
        }


        if(this->m_p_mech_arm_state->work_material[cur_chn]){//ȡ��
            this->m_p_f_reg->right_claw_down = 1;
        }else{
            struct timeval time_now;  //צ��ֹͣ�ж�������ʱ��߾���
            gettimeofday(&time_now, NULL);
            int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                    time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
            if(time_elpase < 1000000){ //��ʱ1000ms
                return;
            }

            this->m_p_f_reg->left_claw_down = 1;  //����
        }

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 18\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 18){ //step-18  ��ʱ300ms��ȡ�Ͽ���צ��գ����Ͽ��ӹ�λ���
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;


        if(this->m_p_mech_arm_state->work_material[cur_chn]){//ȡ�ϣ�����צ���
            if(time_elpase < 300000){ //��ʱ300ms
                return;
            }
            this->m_p_f_reg->right_claw_vac = 1;
        }else{
            this->m_p_f_reg->work_vac = 1;  //���ϣ����ӹ�λ���

            //��¼ʱ��2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
        }

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


        printf("goto step 19\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 19){ //step-19  ȡ��ȷ��������źź�ؼӹ�λ��գ�����ȷ�ϼӹ�λ����źź����צ���
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//ȡ��
            if(this->m_p_g_reg->right_claw_vac_check == 0 && time_elpase < wait_overtime)
                return;

            this->m_p_f_reg->work_vac = 0;   //�ع�λ���
        }else{//����
            if(this->m_p_g_reg->work_vac_check == 0 && time_elpase < wait_overtime){
                //��¼ʱ��2
                gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
                return;
            }

            time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm_2.tv_sec)*1000000+
                    time_now.tv_usec-m_p_mech_arm_state->time_start_arm_2.tv_usec;
            if(time_elpase < work_vac_delay){ //��λ�������ʱ���ٹ���צ���
                return;
            }

            this->m_p_f_reg->left_claw_vac = 0; //����צ���
        }

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 20\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 20){ //step-20  ȡ��ȷ�ϼӹ�λ���ȷ���ź���ʧ����ʱ300ms������ȷ����צ����ź���ʧ����ʱ200ms
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//ȡ��
            if(this->m_p_g_reg->work_vac_check == 1 && time_elpase < wait_overtime)
                return;
        }else{//����
            if(this->m_p_g_reg->left_claw_vac_check == 1 && time_elpase < wait_overtime){
                //				printf("�ȴ���צ����ź���ʧ,left_claw_vac = %hhu\n", this->m_p_g_reg->left_claw_vac_check);
                return;
            }
        }

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 21\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 21){ //step-21   ��ʱ300ms��ȡ������צ�ϣ���������צ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < delay_time_vac){ //��ʱ300ms
            return;
        }

        if(this->m_p_mech_arm_state->work_material[cur_chn]){//ȡ��
            this->m_p_f_reg->right_claw_down = 0;

            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        }else{//����
            this->m_p_f_reg->left_claw_down = 0;

            this->m_p_mech_arm_state->work_material[cur_chn] = true;    //�ӹ�λ����

            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


            //��ת28��
            printf("jump to step 28\n");
            m_p_mech_arm_state->cur_step = 28;
            msg->SetExecStep(28);
            return;
        }

        printf("goto step 22\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 22){ //step-22  ȷ����צ������λ, �����צ�������ƶ�X��ȥ���ϣ������������ת26��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->right_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ�����צ������λ��ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //��е����������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //��HMI�����쳣
            }
            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            return;
        }

        if(time_elpase < 600000){   //��צ������λ������ʱ600ms�����צ������ź�
            return;
        }

        if(this->m_p_g_reg->right_claw_vac_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ���צ����ճ�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_FETCH_VAC;   //��λȡ��ʧ�ܣ�������쳣
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //��HMI�����쳣
            }
            return;
        }

        this->m_p_mech_arm_state->work_material[cur_chn] = false;  //�ӹ�λ����
        this->m_p_mech_arm_state->has_finished_material = true;  //��צ�г�Ʒ

        if(this->m_p_mech_arm_state->has_material){  //��צ����
            this->m_p_mech_arm_state->x_target_pos = this->m_p_mech_arm_param->pos_work_x[cur_chn];

            this->m_p_channel_engine->ManualMovePmc(ARM_PMC_X, m_p_mech_arm_state->x_target_pos,
                                                    m_p_mech_arm_param->speed/2, false);  //��������
        }else{ //��צ���ϣ���ת
            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);


            //��ת28��
            printf("goto step 28\n");
            m_p_mech_arm_state->cur_step = 28;
            msg->SetExecStep(28);
            return;
        }

        printf("goto step 23\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 23){ //step-23   �ȴ�X�ᵽλ����צ��
        //X���Ƿ�λ
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_X) - m_p_mech_arm_state->x_target_pos) > 0.005){
            //��¼ʱ��
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
            return;   //û�е�λ
        }

        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 1000000){ //��ʱ1000ms
            return;
        }

        this->m_p_f_reg->left_claw_down = 1;

        //		//��¼ʱ��
        //		gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 24\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 24){ //step-24  ��ʱ300ms�����ӹ�λ���
        //		struct timeval time_now;
        //		gettimeofday(&time_now, NULL);
        //		int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
        //				time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        //		if(time_elpase < 300000){ //��ʱ300ms
        //			return;
        //		}

        this->m_p_f_reg->work_vac = 1;  //����λ���

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);
        //��¼ʱ��2
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);

        printf("goto step 25\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 25){ //step-25  ȷ�ϼӹ�λ����źţ�����צ���
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->work_vac_check == 0 && time_elpase < wait_overtime){
            //��¼ʱ��2
            gettimeofday(&this->m_p_mech_arm_state->time_start_arm_2, NULL);
            return;
        }

        time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm_2.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm_2.tv_usec;
        if(time_elpase < work_vac_delay){ //��λ�������ʱ���ȴ���λ������ȶ�
            return;
        }

        this->m_p_f_reg->left_claw_vac = 0;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        this->m_p_mech_arm_state->work_material[cur_chn] = true;    //�ӹ�λ����
        printf("goto step 26\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 26){ //step-26  �ȴ���צ����ź���ʧ
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;

        if(this->m_p_g_reg->left_claw_vac_check == 1 && time_elpase < wait_overtime)
            return;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 27\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 27){ //step-27  ��ʱ200ms�� ��צ����
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < delay_time_vac){ //�������ʱ
            return;
        }

        this->m_p_f_reg->left_claw_down = 0;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("goto step 28\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 28){ //step-28  ȷ������צ������λ���ƶ�Y�ᵽ���̻�׼λ��
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        int time_elpase = (time_now.tv_sec-m_p_mech_arm_state->time_start_arm.tv_sec)*1000000+
                time_now.tv_usec-m_p_mech_arm_state->time_start_arm.tv_usec;
        if(time_elpase < 500000){ //��ʱ500ms
            return;
        }

        if(this->m_p_g_reg->left_claw_up_check == 0 || this->m_p_g_reg->right_claw_up_check == 0){
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE &&
                    time_elpase > wait_overtime){  //�ȴ�����צ������λ��ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_UP;   //��е����������λ
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //��HMI�����쳣
            }
            return;
        }

        if(this->m_p_mech_arm_state->work_material[cur_chn] &&
                this->m_p_g_reg->work_vac_check == 0){ //��鹤λ������ź�
            if(this->m_p_mech_arm_state->err_state == ERR_ARM_NONE){  //�ȴ���λ������źų�ʱ
                this->m_p_mech_arm_state->err_state = ERR_ARM_WORK_SET_VAC;   //��λ����ʧ�ܣ�������쳣
                this->m_p_channel_engine->SendHMIMechArmException(m_p_mech_arm_state->err_state); //��HMI�����쳣
            }
            return;
        }

        this->m_p_mech_arm_state->has_material = false;   //��צ����
        this->m_p_mech_arm_state->repos = false;

        //����Y��Ŀ��λ��
        this->m_p_mech_arm_state->y_target_pos = this->m_p_mech_arm_param->pos_tray_base_y[cur_chn];

        this->m_p_channel_engine->ManualMovePmc(ARM_PMC_Y, m_p_mech_arm_state->y_target_pos,
                                                m_p_mech_arm_param->speed, false);  //��������

        printf("goto step 29\n");
        m_p_mech_arm_state->cur_step++;
        msg->IncreaseExecStep();

    }else if(msg->GetExecStep() == 29){ //step-29   �ȴ�Y�ᵽλ
        //Y���Ƿ�λ
        if(fabs(m_p_channel_engine->GetPhyAxisMachPosFeedback(ARM_PMC_Y) - m_p_mech_arm_state->y_target_pos) > 0.005)
            return;   //û�е�λ

        this->m_p_mech_arm_state->msg = nullptr;

        //��¼ʱ��
        gettimeofday(&this->m_p_mech_arm_state->time_start_arm, NULL);

        printf("succeed to execute M66, goto step 30\n");
        m_p_mech_arm_state->cur_step++;
        msg->SetExecStep(0);
    }
}

/**
 * @brief ����ӻ�M66ָ��ӻ�����������
 */
void ChannelControl::ProcessGrindM66_slave(AuxMsg *msg){
    if(msg == nullptr)
        return;

    if(msg->GetExecStep() == 0){
        printf("start execute slave M66 cmd\n");
        printf("goto step 1\n");
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 1){  //step-1  ���ӹ�λ��ͷ�Ƿ��Ѿ�������λ
        if(this->m_p_g_reg->work_up_check == 0)
            return;

        this->m_p_f_reg->m66_req_right = 1;   //���������������

        printf("goto step 2\n");
        msg->IncreaseExecStep();
    }else if(msg->GetExecStep() == 2){ //step-2 �ȴ������Ͻ����ź�
        if(this->m_p_g_reg->m66_runover_right == 0)
            return;

        printf("succeed to execute slave M66\n");
        this->m_p_f_reg->m66_req_right = 0;    //��λ����������
        msg->SetExecStep(0);
    }
}
#endif

/**
 * @brief ��������岹ģʽ������������岹G12.1��ĥ��ר��ָ��G12.2���Լ�ȡ��ָ��G13.1
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
 * @brief ��������RTCP״̬
 * @param cmd_type : ָ�����ͣ�5-5�� 6-G68.2 7-G53.1
 * @param switch_type �� �л�״̬��0-ȡ�� 5-�л���5��  6-�л���G68.2 7-�л���G53.1
 * @param h_val ��Z�򵶾�ƫ�ã���λ��um
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
 * @brief �˳��ϵ�����߳�
 * @return true -- �ɹ��˳�   false--�˳�ʧ��
 */
bool ChannelControl::CancelBreakContinueThread(){
    if(this->m_thread_breakcontinue > 0){//���ڶϵ�����߳�ִ�й����У����˳��ϵ�����߳�
        //		void* thread_result = nullptr;
        //		int res = pthread_cancel(m_thread_breakcontinue);
        //		if (res != ERR_NONE) {
        //			printf("Failed to breakcontinue thread!\n");
        //		}
        //
        //		//	usleep(1000);
        //		res = pthread_join(m_thread_breakcontinue, &thread_result);//�ȴ��������߳��˳����
        //		if (res != ERR_NONE) {
        //			printf("breakcontinue thread join failed\n");
        //		}else
        //			printf("Succeed to cancel breakcontinue thread\n");
        //		m_thread_breakcontinue = 0;
        m_b_cancel_breakcontinue_thread = true;
        int count = 0;
        while(m_thread_breakcontinue > 0){
            if(count++ >= 5000)
                break;//��ʱ�˳�
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
 * @brief ��ͣG��������
 */
void ChannelControl::Pause(){

    if(m_channel_status.chn_work_mode == AUTO_MODE || m_channel_status.chn_work_mode == MDA_MODE){
        if(this->m_thread_breakcontinue > 0){//���ڶϵ�����߳�ִ�й����У����˳��ϵ�����߳�
            this->CancelBreakContinueThread();
        }

        pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
        //printf("locked 12\n");
        this->PauseRunGCode();

        pthread_mutex_unlock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
        //printf("unlocked 12\n");
    }
    else{
        this->ManualMoveStop();  //�ֶ�ֹͣ
    }

}

/**
 * @brief �����Զ�ģʽ�ӹ���ͣʱ��״̬
 * flag : �Ƿ���Ҫ��λneed_reload_flag�� true��Ҫ��λ��false����Ҫ��λ
 */
void ChannelControl::SaveAutoScene(bool flag){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "ChannelControl::SaveAutoScene(), machining_state=%hhu, FLAG=%hhu, cur_pos[%lf, %lf, %lf], last_output=%d\n", m_channel_status.machining_state,
                            m_scene_auto.need_reload_flag, m_channel_rt_status.cur_pos_machine.m_df_point[0], m_channel_rt_status.cur_pos_machine.m_df_point[1],
            m_channel_rt_status.cur_pos_machine.m_df_point[2], (int)this->m_p_last_output_msg);
    //	printf("ChannelControl::SaveAutoScene(), machining_state=%hhu, FLAG=%hhu, cur_pos[%lf, %lf, %lf], last_output=%d\n", m_channel_status.machining_state,
    //			m_scene_auto.need_reload_flag, m_channel_rt_status.cur_pos_machine.m_df_point[0], m_channel_rt_status.cur_pos_machine.m_df_point[1],
    //			m_channel_rt_status.cur_pos_machine.m_df_point[2], (int)this->m_p_last_output_msg);

    if(m_scene_auto.need_reload_flag)  //��δ�ָ�״̬ʱ�����ٱ���
        return;

    m_scene_auto.machining_state = m_channel_status.machining_state;
    if(m_scene_auto.machining_state != MS_RUNNING &&
            m_scene_auto.machining_state != MS_PAUSING &&
            m_scene_auto.machining_state != MS_PAUSED){	//�Ǽӹ�״̬���豣��״̬
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
 * @brief �ָ��Զ��ӹ���ͣʱ��״̬
 */
void ChannelControl::ReloadAutoScene(){
    if(!m_scene_auto.need_reload_flag){	//����ģʽ����ָ�״̬
        return;
    }

    this->SetMachineState(m_scene_auto.machining_state);

    m_n_run_thread_state = (CompilerState)m_scene_auto.run_thread_state;
    m_channel_status.rated_feed = m_scene_auto.rated_feed;
    //	m_channel_status.rated_spindle_speed = m_scene_auto.rated_spindle_speed;   //����breakcontinue�����лָ�����ת��
    m_mc_mode_exec = m_scene_auto.mc_mode_exec;
    m_mc_mode_cur = m_scene_auto.mc_mode_exec;

    this->m_n_subprog_count = m_scene_auto.subprog_count;
    this->m_n_macroprog_count = m_scene_auto.macroprog_count;
    this->m_p_last_output_msg = m_scene_auto.p_last_output_msg;

}

/**
 * @brief �����Զ�ģʽ�ϵ������������
 * @return true--�����ɹ�    false--���账��
 */
bool ChannelControl::StartBreakpointContinue(){

    //�����ϵ����ִ���߳�
    int res = 0;
    pthread_attr_t attr;
    struct sched_param param;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//
    param.__sched_priority = 35; //95;
    pthread_attr_setschedparam(&attr, &param);
    //	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
    //	if (res) {
    //		g_ptr_trace->PrintLog(LOG_ALARM, "�ϵ���������߳������̼̳߳�ģʽʧ�ܣ�");
    //		m_error_code = ERR_INIT_BREAKCONTINUE;
    //		goto END;
    //	}

    res = pthread_create(&m_thread_breakcontinue, &attr,
                         ChannelControl::BreakContinueThread, this);    //����G������������߳�
    if (res != 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�ϵ���������̴߳���ʧ��!", m_n_channel_index);
        m_error_code = ERR_INIT_BREAKCONTINUE;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER, 0, m_n_channel_index);
        goto END;
    }

END:
    pthread_attr_destroy(&attr);
    return true;
}

/**
 * @brief �ϵ����ִ���̺߳���
 * @param void *args: ChannelControl����ָ��
 */
void *ChannelControl::BreakContinueThread(void *args){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "Start Compiler::BreakContinueThread, threadid = %ld!", syscall(SYS_gettid));

    ChannelControl *p_channel_control = static_cast<ChannelControl *>(args);

    int res = ERR_NONE;
    res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
    if (res != ERR_NONE) {
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_CONTROL_SC, "Quit ChannelControl::BreakContinueThread with error 1!");
        pthread_exit((void*) EXIT_FAILURE);
    }
    res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
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
    uint8_t phy_axis = 0;  //�������
    uint8_t chn_axis_z = 0;    //Z��ͨ�����
    double safe_pos = 0;
    double *pos_cur=nullptr, *pos_scene = nullptr;

    this->m_n_breakcontinue_segment = 1;	//���ó�ʼֵ


    while(!m_b_cancel_breakcontinue_thread){

        switch(m_n_breakcontinue_segment){
        case -1:	//ʧ��
            m_scene_auto.need_reload_flag = false;		//��λ��־
            m_b_cancel_breakcontinue_thread = true;
            printf("failed to exec break continue!\n");
            break;

        case 0:		//�ɹ�����,�����ӹ�
            if(this->m_mask_run_pmc){
                this->PausePmcAxis(NO_AXIS, false);  //��������PMC��
            }else if(IsMcNeedStart()){
                this->StartMcIntepolate();
            }
            gettimeofday(&m_time_start_maching, nullptr);  //��ʼ������ʱ��
            m_time_remain = 0;                             //��ʼ��ʣ��ӹ�ʱ��

            if(m_n_run_thread_state == IDLE || m_n_run_thread_state == PAUSE){
                m_n_run_thread_state = RUN;  //��Ϊ����״̬
            }

            this->m_p_f_reg->OP = 1;   //�Զ�����״̬

            m_scene_auto.need_reload_flag = false;     //��λ��־
            m_b_cancel_breakcontinue_thread = true;
            printf("succeed to exec break continue!, m_b_mc_need_start=%hhu\n", m_b_mc_need_start);
            break;
#ifdef USES_ADDITIONAL_PROGRAM
        case 1://��ʼִ��ǰ�ó���
            this->CallAdditionalProgram(CONTINUE_START_ADD);

            m_n_breakcontinue_segment++;
            g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "go to step 2\n");
            break;
        case 2: //�ȴ�ǰ�ó���ִ�����
            if(this->m_n_add_prog_type == NONE_ADD){
                g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "execute step 2, %d\n", m_scene_auto.p_last_output_msg);

                pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ

                m_channel_status.rated_feed = m_scene_auto.rated_feed;
                //	m_channel_status.rated_spindle_speed = m_scene_auto.rated_spindle_speed;
                m_mc_mode_exec = m_scene_auto.mc_mode_exec;
                m_mc_mode_cur = m_scene_auto.mc_mode_exec;
                this->m_n_subprog_count = m_scene_auto.subprog_count;
                this->m_n_macroprog_count = m_scene_auto.macroprog_count;
                this->m_p_last_output_msg = m_scene_auto.p_last_output_msg;

                memcpy(m_channel_status.gmode, m_scene_auto.gmode, kMaxGModeCount*sizeof(uint16_t));   //�ָ�ģ̬

                //�ָ��Զ�ģʽ
                m_p_output_msg_list = m_p_output_msg_list_auto;
                m_n_run_thread_state = (CompilerState)m_scene_auto.run_thread_state;
                this->m_p_compiler->SetMode(AUTO_COMPILER);	//�������л�ģʽ
                this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //�л��������
                pthread_mutex_unlock(&m_mutex_change_state);


                if(this->IsStepMode()){
                    this->SetMcStepMode(true);
                }

                if(m_scene_auto.need_reload_flag)
                    m_n_breakcontinue_segment++;
                else
                    m_n_breakcontinue_segment = 0;   //������������ʱ��ֱ�ӽ���
                g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "go to step 3, %d\n", this->m_p_last_output_msg);
            }
            break;
        case 3: //��鵱ǰ����
            if(this->m_channel_status.cur_tool == m_scene_auto.cur_tool){	//����δ��
                m_n_breakcontinue_segment = 10;
                g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "go to step 10\n");
                break;
            }

            //TODO ֪ͨPMC���л���

            m_n_breakcontinue_segment++;
            g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "go to step 4\n");
            break;
        case 4:  //�ȴ���������
            //TODO  �ȴ�MC����ź�

            m_channel_status.cur_tool = m_scene_auto.cur_tool;
            this->SendModeChangToHmi(T_MODE);

#ifdef USES_WOOD_MACHINE
            this->SetMcToolLife();
#endif

            m_n_breakcontinue_segment = 10;
            g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "go to step 10\n");
            break;
#else
        case 1:		//��鵱ǰ����
            if(this->m_channel_status.cur_tool == m_scene_auto.cur_tool){	//����δ��
                m_n_breakcontinue_segment = 10;
                break;
            }

            //TODO ֪ͨPMC���л���

            m_n_breakcontinue_segment++;
            printf("goto step 2\n");
            break;
        case 2:		//�ȴ���������
            //TODO  �ȴ�MC����ź�

            m_channel_status.cur_tool = m_scene_auto.cur_tool;
            this->SendModeChangToHmi(T_MODE);

#ifdef USES_WOOD_MACHINE
            this->SetMcToolLife();
#endif

            m_n_breakcontinue_segment = 10;
            printf("goto step 10\n");
            break;
#endif
        case 10:   //��鵶ƫ���ã�����D��H
            this->m_channel_status.gmode[8] = m_scene_auto.gmode[8];	// �ָ�G43/G44/G49ģ̬
            this->m_channel_status.gmode[7] = m_scene_auto.gmode[7];	// �ָ�G40/G41/G42ģ̬
            if(m_channel_status.cur_h_code != m_scene_auto.cur_h_code){ //�ָ�Hֵ
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

        case 20:	//�������ϵ
            if(m_channel_status.gmode[14] != m_scene_auto.gmode[14]){
                this->m_channel_status.gmode[14] = m_scene_auto.gmode[14];

                //����MC������ϵ����
                this->SetMcCoord(true);
            }

            m_n_breakcontinue_segment = 30;
            printf("goto step 30\n");
            break;
        case 30:	//�ָ�����ģ̬
            memcpy(m_channel_status.gmode, m_scene_auto.gmode, 2*kMaxGModeCount);

            this->SendChnStatusChangeCmdToHmi(G_MODE);
            m_n_breakcontinue_segment = 60;
            // lidianqiang:ȡ��Z��̧�����µ�����
            // m_n_breakcontinue_segment = 40;
            printf("goto step 40\n");
            break;
        case 40: 	//�ָ�����״̬
            //lidianqiang:����ָ��뵱ǰ���Ṧ���г�ͻ���ݲ���������ָ�������
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
        case 41:	//�ȴ�����ת�ٵ���
            //lidianqiang:����ָ��뵱ǰ���Ṧ���г�ͻ���ݲ���������ָ�������
            //			if(labs(m_channel_rt_status.spindle_cur_speed-(labs(m_channel_status.rated_spindle_speed)*m_channel_status.spindle_dir*m_channel_status.spindle_ratio/100)) > (labs(m_channel_status.rated_spindle_speed)*0.05)){
            //				printf("cur_speed = %d, rated=%d, dir=%hhd, ratio=%d\n", m_channel_rt_status.spindle_cur_speed, m_channel_status.rated_spindle_speed,
            //						m_channel_status.spindle_dir, m_channel_status.spindle_ratio);
            //				break;
            //			}

            m_n_breakcontinue_segment = 50;
            printf("goto step 50\n");
            break;
        case 50:	//�ص��ϵ�λ�ã�����Z���ƶ�����ȫ�߶�(���û����������Ҫ�ƶ�������Ҫ����ȫ�߶�)
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

            if(m_channel_rt_status.cur_pos_machine.GetAxisValue(chn_axis_z)< safe_pos)  //��Z����ڰ�ȫ�߶�ʱ����̧����ȫ�߶�
                this->ManualMove(chn_axis_z, safe_pos, 2000);		//TODO ʹ�û�е����ϵ
            else{
                m_n_breakcontinue_segment = 60;
                break;
            }

            m_n_breakcontinue_segment++;
            printf("goto step 51\n");
            break;
        case 51:	//�ȴ�Z�ᵽλ
            chn_axis_z = this->GetChnAxisFromName(AXIS_NAME_Z);
            phy_axis = m_p_channel_config->chn_axis_phy[chn_axis_z];
            if(phy_axis > 0){  //
                safe_pos = m_p_axis_config[phy_axis-1].axis_home_pos[1];
            }
            if(fabs(this->m_channel_rt_status.cur_pos_machine.m_df_point[chn_axis_z]-safe_pos) < 5e-3){ //��λ
                m_n_breakcontinue_segment = 60;
                printf("goto step 60\n");
                break;
            }else{
                //		this->ManualMove(chn_axis_z, safe_pos, 1000);		//TODO ʹ�û�е����ϵ
                usleep(5000);
            }
            break;
        case 60:{	//�����ŷ���ص��ϵ�λ��
            pos_cur = m_channel_rt_status.cur_pos_machine.m_df_point;
            pos_scene = m_scene_auto.cur_pos_machine.m_df_point;
            for(i = 0; i < this->m_p_channel_config->chn_axis_count; i++, pos_cur++, pos_scene++){
                phy_axis = m_p_channel_config->chn_axis_phy[i]; // phy_axis = m_channel_status.cur_chn_axis_phy[i];
                if(phy_axis > 0){
                    if(m_p_axis_config[phy_axis-1].axis_linear_type == LINE_AXIS_Z ||
                            m_p_axis_config[phy_axis-1].axis_linear_type == LINE_AXIS_PZ ||
                            m_p_axis_config[phy_axis-1].axis_type == AXIS_SPINDLE)    //����Z�������
                        continue;
                }

                if(fabs(*pos_cur - *pos_scene) >= 1e-3)
                    this->ManualMove(i, *pos_scene, 2000);		//TODO ʹ�û�е����ϵ
            }
            m_n_breakcontinue_segment = 61;
            printf("goto step 61\n");
            break;
        }
        case 61:{	//�ȴ��ŷ����˶���λ
            pos_cur = m_channel_rt_status.cur_pos_machine.m_df_point;
            pos_scene = m_scene_auto.cur_pos_machine.m_df_point;
            for(i = 0; i < this->m_p_channel_config->chn_axis_count; i++, pos_cur++, pos_scene++){
                phy_axis = m_p_channel_config->chn_axis_phy[i];  // phy_axis = m_channel_status.cur_chn_axis_phy[i];
                if(phy_axis > 0){
                    if(m_p_axis_config[phy_axis-1].axis_linear_type == LINE_AXIS_Z ||
                            m_p_axis_config[phy_axis-1].axis_linear_type == LINE_AXIS_PZ ||
                            m_p_axis_config[phy_axis-1].axis_type == AXIS_SPINDLE)    //����Z�������
                        continue;
                }

                if(fabs(*pos_cur - *pos_scene) >= 5e-3){ //δ��λ
                    //		printf("%d axis not arrived, %lf, %lf, %d\n", i, *pos_cur, *pos_scene, m_p_channel_config->chn_axis_count);
                    break;
                }
            }
            if(i == m_p_channel_config->chn_axis_count){ //�����ᶼ��λ
                m_n_breakcontinue_segment++;
                printf("goto step 62\n");
            }
            else
                usleep(5000);

            break;
        }
        case 62:  	//Z�Ḵλ���ϵ�λ��
            //			chn_axis_z = this->GetChnAxisFromName(AXIS_NAME_Z);

            //			this->ManualMove(chn_axis_z, m_scene_auto.cur_pos_machine.m_df_point[chn_axis_z], 2000);		//TODO ʹ�û�е����ϵ

            //			m_n_breakcontinue_segment++;
            //			printf("goto step 63\n");
            // lidianqiang:ȡ��Z��̧�����µ�����
            m_n_breakcontinue_segment = 0;
            break;
        case 63:	//�ȴ�Z�ᵽλ
            if(fabs(m_channel_rt_status.cur_pos_machine.m_df_point[2] - m_scene_auto.cur_pos_machine.m_df_point[2]) < 1e-3){ //��λ
                m_n_breakcontinue_segment = 0;
                printf("goto step 0\n");
            }else
                usleep(5000);

            break;
        }
        usleep(1000);
    }
#ifdef USES_ADDITIONAL_PROGRAM
    if(m_n_breakcontinue_segment == 1 || m_n_breakcontinue_segment == 2){   //ȡ���ϵ����ǰ�ó�������

        this->StopCompilerRun();
        //��MCģ�鷢����ָͣ��
        this->PauseMc();

        this->m_p_output_msg_list->Clear();

        //�ָ��Զ�ģʽ
        pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
        m_p_output_msg_list = m_p_output_msg_list_auto;
        m_n_run_thread_state = PAUSE;
        this->m_p_compiler->SetMode(AUTO_COMPILER);	//�������л�ģʽ
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //�л��������
        pthread_mutex_unlock(&m_mutex_change_state);

        m_n_add_prog_type = NONE_ADD;   //��λ���ӳ���ִ��״̬

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
 * @brief ָ����زο���
 * @param axis_mask : ָ����Щͨ�������
 * @return
 */
#ifdef USES_WUXI_BLOOD_CHECK
bool ChannelControl::ReturnRef(uint8_t axis_mask){
    SCAxisConfig *axis_config = nullptr;
    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i) && m_p_channel_config->chn_axis_phy[i] != 0){ // if(axis_mask & (0x01<<i) && m_channel_status.cur_chn_axis_phy[i] != 0){
            axis_config = &m_p_axis_config[m_p_channel_config->chn_axis_phy[i]-1]; //axis_config = &m_p_axis_config[m_channel_status.cur_chn_axis_phy[i]-1];
            this->ManualMove(i, axis_config->axis_home_pos[0], axis_config->ret_ref_speed, false);   //�ص���е����ϵ�µĵ�һ�ο���
        }
    }
    return true;
}
#endif

/**
 * @brief �زο���ִ�к���
 * @param flag : true--����ͨ�������    false--��ǰ�����
 */
void ChannelControl::ProcessHmiReturnRefCmd(bool flag){
    printf("ChannelControl::ProcessHmiReturnRefCmd, flag = %hhu\n", flag);
    if(flag){//����ͨ�������
        uint8_t phy_axis = 0;
        for(int i=0; i < this->m_p_channel_config->chn_axis_count; i++){
            phy_axis = this->GetPhyAxis(i);
            if(phy_axis != 0xff){
                this->m_p_channel_engine->SetRetRefMask(phy_axis);
            }
        }

    }else{ //��ǰ�����
        uint8_t cur_phy = this->GetCurPhyAxis();
        if(cur_phy != 0){
            this->m_p_channel_engine->SetRetRefMask(cur_phy-1);
        }
    }
}

/**
 * @brief ������������չ�±�ʹ��
 * @param flag : �����������±�
 */
void ChannelControl::SetAxisNameEx(bool flag){
    this->m_p_compiler->SetAxisNameEx(flag);
}

#ifdef USES_WUXI_BLOOD_CHECK
/**
 * @brief ���û����־
 */
void ChannelControl::SetRetHomeFlag(){
    if(!m_b_returnning_home){
        m_b_returnning_home = true;
        m_n_ret_home_step=0;
    }

}

/**
 * @brief �زο���   ������Ŀ����
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
        this->ReturnRef(0x04);   //Z�����
        this->m_n_ret_home_step++;
        break;
    case 1://second step    �ȴ�Z�ᵽλ
        phy_axis = this->m_p_channel_config->chn_axis_phy[2]; // phy_axis = this->m_channel_status.cur_chn_axis_phy[2];
        if(phy_axis > 0){
            if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(2) - m_p_axis_config[phy_axis].axis_home_pos[0]) < 1e-3){ //��λ
                m_n_ret_home_step++;
            }
        }
        break;
    case 2://third step   XY�����
        this->ReturnRef(0x03);
        this->m_n_ret_home_step++;
        break;
    case 3://fourth step �ȴ�XY��λ
        phy_axis = this->m_p_channel_config->chn_axis_phy[0];  //X�� // phy_axis = this->m_channel_status.cur_chn_axis_phy[0];  //X��
        if(phy_axis > 0){
            if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(0) - m_p_axis_config[phy_axis].axis_home_pos[0]) >= 1e-3){ //δ��λ
                break;
            }
        }
        phy_axis = this->m_p_channel_config->chn_axis_phy[1];   //Y�� // phy_axis = this->m_channel_status.cur_chn_axis_phy[1];   //Y��
        if(phy_axis > 0){
            if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(1) - m_p_axis_config[phy_axis].axis_home_pos[0]) >= 1e-3){ //δ��λ
                break;
            }
        }

        gettimeofday(&m_time_start_wait, nullptr);  //��¼��ʼʱ��
        m_n_ret_home_step++;
        break;
    case 4: //fifth step �ȴ���ʱ
        this->m_macro_variable.GetVarValue(510, wait_time, init);
        gettimeofday(&cur_time, nullptr);
        time_elpase = (cur_time.tv_sec-m_time_start_wait.tv_sec)*1000 + (cur_time.tv_usec - m_time_start_wait.tv_usec)/1000;  //ms
        if(time_elpase < wait_time)
            break;		//δ����ʱʱ��
        m_n_ret_home_step++;
        break;
    case 5: //sixth step  ��ͷ��λ
        //	this->ReturnRef(0x08);   //��ͷ�����
        this->m_n_ret_home_step++;
        break;
    case 6:
        phy_axis = this->m_p_channel_config->chn_axis_phy[3]; // phy_axis = this->m_channel_status.cur_chn_axis_phy[3];
        if(phy_axis > 0){
            if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(3) - m_p_axis_config[phy_axis].axis_home_pos[0]) < 1e-3){ //��λ
                m_n_ret_home_step = 0;
                this->m_b_returnning_home = false;  //�������
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
 * @brief ����ͣ
 * @return
 */
bool ChannelControl::EmergencyStop(){

#ifdef USES_GRIND_MACHINE
    if(this->m_b_ret_safe_state){ //ȡ�����Ḵλ����
        this->m_b_ret_safe_state = false;
        this->m_n_ret_safe_step = 0;
    }


    //��λIO���
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
    //		this->m_p_mech_arm_state->work_material[GRIND_SLAVE_CHN] = false;   //�ӻ��ļ�ͣû�нӵ�����

    this->m_p_mech_arm_state->has_finished_material = false;
    this->m_p_mech_arm_state->has_material = false;
    this->m_p_mech_arm_state->repos = false;
#endif

    this->m_p_f_reg->RST = 1;

    //ֹͣ����
    pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
    //printf("locked 15\n");
    m_n_run_thread_state = STOP;
    pthread_mutex_unlock(&m_mutex_change_state);
    //printf("unlocked 15\n");
    //ֹͣMCִ��

    this->PauseMc();

    if(this->m_thread_breakcontinue > 0){//���ڶϵ�����߳�ִ�й����У����˳��ϵ�����߳�
        this->CancelBreakContinueThread();
    }

    m_p_spindle->EStop();

    // ȡ�����Թ�˿״̬
    //if(m_p_spindle->isTapEnable())
        //m_p_spindle->CancelRigidTap();

    //	if(this->m_n_M29_flag){  //����ָ��ٶȿ���
    //		this->ProcessM29Reset();
    //	}

    //����״̬
    uint8_t state = MS_WARNING;   //�澯״̬
#ifdef USES_WOOD_MACHINE
    state = MS_STOPPING;   //ֹͣ��
#endif
    this->SetMachineState(state);  //����ͨ��״̬

    //��λ����زο����־��ֻ������ʽ������������Ҫ��λ
    for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(m_p_axis_config[m_p_channel_config->chn_axis_phy[i]-1].axis_interface != VIRTUAL_AXIS		//��������
                && m_p_axis_config[m_p_channel_config->chn_axis_phy[i]-1].axis_type != AXIS_SPINDLE				//������
                && m_p_axis_config[m_p_channel_config->chn_axis_phy[i]-1].feedback_mode == INCREMENTAL_ENCODER)    //����ʽ������
        {
            m_channel_status.returned_to_ref_point &= ~(0x01<<i);
        }
    }
    this->SendChnStatusChangeCmdToHmi(REF_POINT_FLAG);


    //OP�źŸ�λ
    this->m_p_f_reg->OP = 0;
    //this->m_p_f_reg->SPL = 0;

    return true;

}

/**
 * @brief �����Ƿ��д����е�ָ��
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
 * @brief ����ͨ����زο����־
 * @param chn_axis �� ͨ����ţ���0��ʼ
 * @param flag  �� true--����     false--ȡ��
 */
void ChannelControl::SetRefPointFlag(uint8_t chn_axis, bool flag){
    if(flag){
        this->m_channel_status.returned_to_ref_point |= (0x01<<chn_axis);
    }else
        this->m_channel_status.returned_to_ref_point &= ~(0x01<<chn_axis);

    this->SendChnStatusChangeCmdToHmi(REF_POINT_FLAG);

    //����ͨ���������λ����־
    if(flag){
        this->SetChnAxisSoftLimit(chn_axis);
    }else{
        this->CloseChnAxisSoftLimit(chn_axis);   //δ�زο���״̬����ǿ�ƹر�����λ
    }
}

/**
 * @brief ��M���뷢�͸�PMC
 * @param m_code : Mָ��
 * @param m_index �� Mָ��˳��ţ���ͬ�еĵڼ���Mָ��, ֵ��Χ��0-15
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
 * @brief ����MFnѡͨ�ź�
 * @param index : Mָ��˳��ţ�0~15
 * @param value �� true--��һ    false--����
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
 * @brief ����ָ����ŵ�MFIN�ź�
 * @param index : Mָ��˳��ţ�0~15
 * @return true--���ź�    false--���ź�
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
 * @brief ����ָ����ŵ�MExc�ź�
 * @param index : Mָ��˳��ţ�0~15
 * @return   true--���ź�    false--���ź�
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
 * @brief ��Tָ�����PMC�Ĵ���
 * @param t_code �� Tָ��ֵ�����ߺ�
 * @param index : Tָ��˳��ţ���ͬ�еĵڼ���Tָ��, ֵ��Χ��0-15
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
 * @brief ����TFѡͨ�ź�
 * @param index :  Tָ��˳��ţ� 0~15
 * @param value �� true--��һ    false--����
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
 * @brief ����ָ����ŵ�TFIN�ź�
 * @param index : Tָ��˳��ţ� 0~15
 * @return true--���ź�    false--���ź�
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
 * @brief ������ת��ת��Ϊ��ƽֵ
 * @param speed : ����ת��
 */
void ChannelControl::SpindleSpeedDaOut(int32_t speed){
    if(this->m_n_spindle_count == 0){
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "ͨ��[%hhu]û����������\n", m_n_channel_index);
        return;//û������
    }
    //TODO ��ǰ���õ�һ���������ж�
    uint8_t phy_spd = this->m_spd_axis_phy[0];
    SCAxisConfig &spd_config = m_p_axis_config[phy_spd-1];

    if(spd_config.spd_vctrl_mode == 0){//����ģʽΪ��ֹ
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "����[%hhu]��ѹ����ģʽΪ��ֹ\n", phy_spd);
        return;
    }

    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC,"speed = %drpm, spd_dir = %hhd, motor_dir = %hhd\n", speed, m_channel_status.spindle_dir, spd_config.motor_dir);

    int s = labs(speed);
    s = s * this->m_channel_status.spindle_ratio / 100;    //ת�ٳ������ᱶ��

    if((uint32_t)s > spd_config.spd_max_speed)
        s = spd_config.spd_max_speed;

    if(spd_config.spd_vctrl_mode == 2){	//-10v ~ 10v
        s *= this->m_channel_status.spindle_dir;
    }else if(this->m_channel_status.spindle_dir == 0)  //ͣת
        s = 0;

    s += spd_config.zero_compensation;   //������ƫ

    //	s /= spd_config.spd_gear_ratio;

    int64_t tt = s;
    uint32_t da_prec = this->m_p_channel_engine->GetDaPrecision();  //DA����
    tt = tt * da_prec / spd_config.spd_max_speed;  //ת��Ϊ��ƽֵ

    //��ֹ16λ���
    if(labs(tt) >= da_prec){
        if(tt > 0)
            tt = da_prec - 1 ;
        else{
            tt = da_prec - 1;
            tt *= -1;
        }
    }

    if(spd_config.motor_dir == 1)
        tt *= -1;   //���÷���Ϊ��ת

    g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC,"Output spd speed da value: %lld, spd_axis=%hhu, max_speed=%u, da_prec=%u\n",
                            tt, phy_spd, spd_config.spd_max_speed, da_prec);

    this->SendSpdSpeedToMi(phy_spd, (int16_t)tt);

}

/**
 * @brief �������
 * @param dir : ������ת����
 * @param speed : ����ָ��ת��
 */
void ChannelControl::SpindleOut(int dir, int speed){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "ChannelControl::spindleout : %d, speed=%d, chn=%hhu\n", dir, speed, this->m_n_channel_index);
    if(dir == SPD_DIR_STOP)
        this->m_p_f_reg->SPS = 0;    //��־����ͣת
    else if(dir == SPD_DIR_POSITIVE)
        this->m_p_f_reg->SPS = 1;    //��־������ת
    else if(dir == SPD_DIR_NEGATIVE)
        this->m_p_f_reg->SPS = 2;    //��־���ᷴת

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
 * @brief ���¹�������ϵ
 * @param index
 * @param cfg
 */
void ChannelControl::UpdateCoord(uint8_t index, HmiCoordConfig &cfg){
    if(m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING){//������״̬��ֱ����Ч
        g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, index, cfg, true);
        if(index == 0 || this->m_channel_status.gmode[14] == (G53_CMD+index*10))  //�޸��˻���ƫ�ƣ����ߵ�ǰ����ϵ
            this->SetMcCoord(true);
    }
    else
        g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, index, cfg, true);

    //	g_ptr_parm_manager->UpdateCoordConfig(m_n_channel_index, index, cfg, true);
}

/**
 * @brief ������չ��������ϵ
 * @param index
 * @param cfg
 */
void ChannelControl::UpdateExCoord(uint8_t index, HmiCoordConfig &cfg){
    if(m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING){//������״̬��ֱ����Ч
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
 * @brief �������й�������ϵΪ�趨ֵ
 * @param val
 * @return
 */
bool ChannelControl::UpdateAllCoord(double val)
{
    if(m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING){//������״̬��ֱ����Ч
        //��������ϵ ��������ϵ+G54-G59
        g_ptr_parm_manager->UpdateAllCoordConfig(m_n_channel_index, val, true);
        return true;
    }
    return false;

}

/**
 * @brief ����������չ��������ϵΪ�趨ֵ
 * @param val
 * @return
 */
bool ChannelControl::UpdateAllExCoord(double val, int count)
{
    if(m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING){//������״̬��ֱ����Ч
        //��չ����ϵ
        std::cout << "count: " << count << std::endl;
        g_ptr_parm_manager->UpdateAllExCoordConfig(m_n_channel_index, val, true, count);
        return true;
    }
    return false;
}

/**
 * @brief ���µ���ƫ��
 * @param index : ��ƫ��ţ���0��ʼ
 * @param cfg
 */
void ChannelControl::UpdateToolOffset(uint8_t index, HmiToolOffsetConfig &cfg){
    if(m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING){//������״̬��ֱ����Ч
        g_ptr_parm_manager->UpdateToolOffsetConfig(m_n_channel_index, index, cfg, true);
        if(this->m_channel_status.gmode[8] != G49_CMD && this->m_channel_status.cur_h_code == index+1)
            this->SetMcToolOffset(true);
    }else if(this->m_channel_status.cur_h_code != index+1){  //�ǵ�ǰ��ƫ��ֱ����Ч
        g_ptr_parm_manager->UpdateToolOffsetConfig(m_n_channel_index, index, cfg, true);
    }
    else
        g_ptr_parm_manager->UpdateToolOffsetConfig(m_n_channel_index, index, cfg, true);
}

bool ChannelControl::UpdateAllToolOffset(const HmiToolOffsetConfig &cfg)
{
    //if(m_channel_status.machining_state == MS_READY || m_channel_status.machining_state == MS_WARNING){//������״̬
        g_ptr_parm_manager->UpdateAllToolOffsetConfig(m_n_channel_index, cfg);
        //if(this->m_channel_status.gmode[8] != G49_CMD && this->m_channel_status.cur_h_code == index+1)
        //    this->SetMcToolOffset(true);
    //}
    return true;
}

/**
 * @brief �޸�������Ȧλ�õ���mask
 * @param axis_index : ��������ţ���1��ʼ
 */
void ChannelControl::SetClearPosAxisMask(uint8_t axis_index){

    if(axis_index > this->m_p_general_config->axis_count)
        return;
    this->m_n_mask_clear_pos |= (0x01<<(axis_index-1));

}

/**
 * @brief �������ʧ�Ժ����
 */
void ChannelControl::SaveKeepMacroVar(){
    this->m_macro_variable.Sync(true);  //���沢�ر��ļ�
}

/**
 * @brief ��������ŷ�״̬
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
 * @brief ��λOP�ź�
 */
void ChannelControl::ResetOPSignal(){
    this->m_p_f_reg->OP = 0;
    printf("ChannelControl::ResetOPSignal()\n");
}

/**
 * @brief ��λRST�ź�
 */
void ChannelControl::ResetRSTSignal(){
    //
    this->m_p_f_reg->RST = 0;
}

/**
 * @brief ���ø澯�ź�
 * @param value
 */
void ChannelControl::SetALSignal(bool value){

	if(value){
		// �չ����� �˳��չ�״̬���� ����չ����޷���λ
		printf("+++++++++++++++++++++++++ RGSPP = 0\n");
		m_p_f_reg->RGSPP = 0;
		this->m_p_f_reg->AL = 1;
	}else{
		this->m_p_f_reg->AL = 0;
	}
}

/**
 * @brief �������ɻ�е����ϵת��Ϊ��������ϵ
 * @param pos : ��ת��������
 * @param coord_idx : ��������ϵ
 * @param axis_mask : ������
 */
void ChannelControl::TransMachCoordToWorkCoord(DPointChn &pos, uint16_t coord_idx, uint32_t axis_mask){
    double origin_pos = 0;
    double *pp = pos.m_df_point;
    uint8_t phy_axis = 0xFF;

    //axis_mask &= (~m_mask_pmc_axis);   //ȥ��PMC��

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            origin_pos = m_p_chn_coord_config[0].offset[i];  //������������ϵ

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
                if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z����Ҫ���ϵ���ƫ��
                    //	origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //��׼��ƫ
                    origin_pos += m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2];  //
                    origin_pos += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1];   //����ĥ�𲹳�
                }

            }

            *pp -= origin_pos;    //��е���� - ��������ϵƫ�� = ��������

        }
        pp++;

    }

}

/**
 * @brief �������ɻ�е����ϵת��Ϊ��������ϵ
 * @param pos : ��ת��������
 * @param coord_idx : ��������ϵ
 * @param h_code : ����������
 * @param axis_mask : ������
 */
void ChannelControl::TransMachCoordToWorkCoord(DPointChn &pos, uint16_t coord_idx, uint16_t h_code, uint32_t axis_mask){
    double origin_pos = 0;
    double *pp = pos.m_df_point;
    uint8_t phy_axis = 0xFF;

    //axis_mask &= (~m_mask_pmc_axis);   //ȥ��PMC��

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            origin_pos = m_p_chn_coord_config[0].offset[i];  //������������ϵ

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
                if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && h_code > 0){//Z����Ҫ���ϵ���ƫ��
                    //	origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //��׼��ƫ
                    origin_pos += m_p_chn_tool_config->geometry_compensation[h_code-1][2];  //
                    origin_pos += m_p_chn_tool_config->geometry_wear[h_code-1];   //����ĥ�𲹳�
                }
            }
            *pp -= origin_pos;    //��е���� - ��������ϵƫ�� = ��������
        }
        pp++;
    }
}

/**
 * @brief �������ɻ�е����ϵת��Ϊ��������ϵ
 * @param pos : ��ת��������
 * @param coord_idx : ��������ϵ
 * @param axis_mask : ������
 */
void ChannelControl::TransMachCoordToWorkCoord(DPoint &pos, uint16_t coord_idx, uint32_t axis_mask){
    double origin_pos = 0;
    double *pp = &pos.x;
    uint8_t phy_axis = 0xFF;

    //axis_mask &= (~m_mask_pmc_axis);   //ȥ��PMC��

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            origin_pos = m_p_chn_coord_config[0].offset[i];  //������������ϵ

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
                if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z����Ҫ���ϵ���ƫ��
                    //		origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //��׼��ƫ
                    origin_pos += m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2];  //
                    origin_pos += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1];   //����ĥ�𲹳�
                }

            }

            *pp -= origin_pos;    //��е���� - ��������ϵƫ�� = ��������

        }
        pp++;
    }
}

/**
 * @brief �������ɹ�������ϵת��Ϊ��е����ϵ
 * @param pos : ��ת��������
 * @param coord_idx : ��������ϵ
 *  @param axis_mask : ������
 */
void ChannelControl::TransWorkCoordToMachCoord(DPointChn &pos, uint16_t coord_idx, uint32_t axis_mask){
    double origin_pos = 0;
    double *pp = pos.m_df_point;
    uint8_t phy_axis = 0xFF;

    //axis_mask &= (~m_mask_pmc_axis);   //ȥ��PMC��

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            origin_pos = m_p_chn_coord_config[0].offset[i];  //������������ϵ

            if(coord_idx >= G54_CMD && coord_idx <= G59_CMD ){
                origin_pos += m_p_chn_coord_config[coord_idx/10-53].offset[i];
            }else if(coord_idx >= G5401_CMD && coord_idx <= G5499_CMD){
                origin_pos += m_p_chn_ex_coord_config[coord_idx/10-5401].offset[i];
            }

            phy_axis = this->GetPhyAxis(i);
            if(phy_axis != 0xFF){
                if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z����Ҫ���ϵ���ƫ��
                    //		origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //��׼��ƫ
                    origin_pos += m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2];  //
                    origin_pos += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1];   //����ĥ�𲹳�
                }

            }

            *pp += origin_pos;    //��е���� - ��������ϵƫ�� = ��������

        }
        pp++;

    }
}

/**
 * @brief �������ɹ�������ϵת��Ϊ��е����ϵ
 * @param pos : ��ת��������
 * @param coord_idx : ��������ϵ
 *  @param axis_mask : ������
 */
void ChannelControl::TransWorkCoordToMachCoord(DPoint &pos, uint16_t coord_idx, uint32_t axis_mask){
    double origin_pos = 0;
    double *pp = &pos.x;
    uint8_t phy_axis = 0xFF;

    //axis_mask &= (~m_mask_pmc_axis);   //ȥ��PMC��

    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            origin_pos = m_p_chn_coord_config[0].offset[i];  //������������ϵ

            if(coord_idx >= G54_CMD && coord_idx <= G59_CMD ){
                origin_pos += m_p_chn_coord_config[coord_idx/10-53].offset[i];
            }else if(coord_idx >= G5401_CMD && coord_idx <= G5499_CMD){
                origin_pos += m_p_chn_ex_coord_config[coord_idx/10-5401].offset[i];
            }

            phy_axis = this->GetPhyAxis(i);
            if(phy_axis != 0xFF){
                if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z����Ҫ���ϵ���ƫ��
                    //		origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //��׼��ƫ
                    origin_pos += m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2];  //
                    origin_pos += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1];   //����ĥ�𲹳�
                }

            }

            *pp += origin_pos;    //��е���� - ��������ϵƫ�� = ��������

        }
        pp++;
    }
}

/**
 * @brief �����������ɻ�е����ϵת��Ϊ��������ϵ
 * @param pos : �����е����
 * @param coord_idx : ��������ϵ
 * @param axis ��ͨ�����, ��0��ʼ
 */
void ChannelControl::TransMachCoordToWorkCoord(double &pos, uint16_t coord_idx, uint8_t axis){
    double origin_pos = 0;
    origin_pos = m_p_chn_coord_config[0].offset[axis];  //������������ϵ
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
        if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z����Ҫ���ϵ���ƫ��
            //	origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //��׼��ƫ
            origin_pos += m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2];
            origin_pos += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1];   //����ĥ�𲹳�
        }

    }


    pos -= origin_pos;    //��е���� - ��������ϵƫ�� = ��������
}

/**
 * @brief �����������ɹ�������ϵת��Ϊ��е����ϵ
 * @param pos : ���Ṥ������ϵ
 * @param coord_idx : ��������ϵ
 * @param axis ��ͨ�����
 */
void ChannelControl::TransWorkCoordToMachCoord(double &pos, uint16_t coord_idx, uint8_t axis){
    double origin_pos = 0;
    origin_pos = m_p_chn_coord_config[0].offset[axis];  //������������ϵ
    if(coord_idx >= G54_CMD && coord_idx <= G59_CMD ){
        origin_pos += m_p_chn_coord_config[coord_idx/10-53].offset[axis];
    }else if(coord_idx >= G5401_CMD && coord_idx <= G5499_CMD){
        origin_pos += m_p_chn_ex_coord_config[coord_idx/10-5401].offset[axis];
    }

    uint8_t phy_axis = this->GetPhyAxis(axis);
    if(phy_axis != 0xFF){
        if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z����Ҫ���ϵ���ƫ��
            //	origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //��׼��ƫ
            origin_pos += m_p_chn_tool_config->geometry_compensation[m_channel_status.cur_h_code-1][2];
            origin_pos += m_p_chn_tool_config->geometry_wear[m_channel_status.cur_h_code-1];   //����ĥ�𲹳�
        }

    }

    pos += origin_pos;    //��е���� - ��������ϵƫ�� = ��������
}

/**
 * @brief �����������ɹ�������ϵת��Ϊ��е����ϵ
 * @param pos :  ��ת���Ĺ�������
 * @param coord_idx �� ��������ϵ��
 * @param h_code ������ƫ�úţ���1��ʼ
 * @param axis �� ͨ�����
 */
void ChannelControl::TransWorkCoordToMachCoord(double &pos, uint16_t coord_idx, uint16_t h_code, uint8_t axis){
    double origin_pos = 0;
    origin_pos = m_p_chn_coord_config[0].offset[axis];  //������������ϵ
    if(coord_idx >= G54_CMD && coord_idx <= G59_CMD ){
        origin_pos += m_p_chn_coord_config[coord_idx/10-53].offset[axis];
    }else if(coord_idx >= G5401_CMD && coord_idx <= G5499_CMD){
        origin_pos += m_p_chn_ex_coord_config[coord_idx/10-5401].offset[axis];
    }

    uint8_t phy_axis = this->GetPhyAxis(axis);
    if(phy_axis != 0xFF){
        if(this->m_p_axis_config[phy_axis].axis_linear_type == LINE_AXIS_Z && m_channel_status.cur_h_code > 0){//Z����Ҫ���ϵ���ƫ��
            //	origin_pos += m_p_chn_tool_config->geometry_comp_basic[2];   //��׼��ƫ
            origin_pos += m_p_chn_tool_config->geometry_compensation[h_code-1][2];
            origin_pos += m_p_chn_tool_config->geometry_wear[h_code-1];   //����ĥ�𲹳�
        }

    }

    pos += origin_pos;    //��е���� - ��������ϵƫ�� = ��������
}


#ifdef USES_LASER_MACHINE
/**
 * @brief ����HMI�������궨ָ��
 * @param bstart : true--��ʼ�궨    false--ȡ���궨
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
 * @brief ������������궨
 */
void ChannelControl::ProcessLaserCalibration(){

    //�ο���궨�����м��COLLISION�źţ�A11���Ƿ���λ��ΪHigh��궨ʧ�ܣ��澯���˳�
    if(m_n_cur_ref_point > 0 && m_n_calib_step > 5){
        if(this->m_n_laser_input_signal & 0x02){
            printf("�������궨�쳣��\n");
            this->m_n_calib_step = 100;
        }
    }

    //�жϱ궨���˳�
    if(this->m_b_cancel_calib){
        printf("�˳��������궨��\n");
        this->m_n_calib_step = 100;
        this->m_b_cancel_calib = false;
    }

    uint8_t chn_axis = 0;
    uint8_t phy_axis = 0;
    switch(this->m_n_calib_step){
    case 0: //ȡ������������ʹ��
        this->EnableHeightRegulatorFollow(false);
        printf("��ֹ����ʹ��\n");
        m_n_calib_step++;
        break;
    case 1: //ȷ�����̼��궨����
        this->m_n_follow_range = HYD_RANGE_10;		//��������Χ10mm
        this->m_n_calib_ref_count = 16;   //16��궨
        this->m_n_cur_ref_point = 0;      //��ǰ�궨��һ���ο���

        m_n_calib_step++;
        break;
    case 2: //����CAL.REQUEST�źţ�PIN A3, High��
        this->SetHeightRegulatorOutput(1, true);

        printf("��λCAL.REQUEST�ź�,�ȴ�READY�źŸ�λ...\n");
        m_n_calib_step++;
        break;
    case 3:{ //�ȴ�READY�ź���λLOW��Ȼ��ָ��Z���������ƣ�����MI����Z�����λ��
        if(this->m_n_laser_input_signal & 0x08){
            break;  //�ȴ�READY�źű��
        }
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
        if(chn_axis == NO_AXIS){ //�˳�
            printf("Z��δ���ã�\n");
            m_n_calib_step = 100;
            break;
        }
        uint64_t mask = 0x01;
        phy_axis = this->GetPhyAxisFromName(AXIS_NAME_Z);
        if(phy_axis == NO_AXIS){ //�˳�
            printf("Z��δ���ã�\n");
            m_n_calib_step = 100;
            break;
        }
        mask = (mask << phy_axis);
        this->SendCatchPosCmd(mask, 2, true);   //����Z���λ�ò���
        this->ManualMove2(chn_axis, DIR_NEGATIVE, 120, 1000);  //Z�Ὺʼ���������ƶ�

        printf("Z���������壬Ѱ����㣡mask = 0x%llx\n", mask);
        m_n_calib_step++;
    }
        break;
    case 4: //�ȴ�MI����Z��λ�ã���ͣ��Z��
        if(this->m_b_pos_captured){
            chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
            this->ManualMoveStop(0x01<<chn_axis);   //ֹͣZ��

            printf("�ҵ���㣺%lf, %lf, %lf, %lf\n", m_point_capture.x, m_point_capture.y,
                   m_point_capture.z, m_point_capture.a4);
            m_n_calib_step++;
        }

        break;
    case 5:{ //�ƶ�����ǰ�ο���
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
        double *pp = &this->m_point_capture.x;
        pp += chn_axis;
        double pos_tar = *pp;
        pos_tar += kHYDCalibRefPos[m_n_follow_range][m_n_cur_ref_point];  //Ŀ��λ��
        //	if(this->m_n_cur_ref_point == 0){//��һ���ο��㣬���������ƶ����������
        this->ManualMove(chn_axis, pos_tar, 3000);
        //	}else{
        //		this->ManualMove2(chn_axis, DIR_NEGATIVE, 3000,
        //				kHYDCalibRefPos[m_n_follow_range][m_n_cur_ref_point-1]-kHYDCalibRefPos[m_n_follow_range][m_n_cur_ref_point]);
        //	}
        printf("�ƶ�����%hhu�ο���:%lf mm\n", m_n_cur_ref_point+1, pos_tar);
        m_n_calib_step++;
    }
        break;
    case 6:{ //�ȴ����ƶ���λ������STROBE�źţ�PIN A7, High��
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
        double *pp = &this->m_point_capture.x;
        pp += chn_axis;
        double pos_tar = *pp;
        pos_tar += kHYDCalibRefPos[m_n_follow_range][m_n_cur_ref_point];  //Ŀ��λ��

        if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(chn_axis) - pos_tar) < 1e-3){ //��λ
            this->SetHeightRegulatorOutput(5, true);  //��λSTROBE�źţ�PIN A7, High��

            gettimeofday(&m_time_delay_start, nullptr);  //��¼��ʼʱ��
            printf("��λ��ʱ...\n");
            m_n_calib_step++;
        }

    }
        break;
    case 7:{ //�ȴ�240ms��ʱ
        struct timeval cur_time;
        uint64_t time_elpase = 0;
        gettimeofday(&cur_time, nullptr);
        time_elpase = (cur_time.tv_sec-m_time_delay_start.tv_sec)*1000 + (cur_time.tv_usec - m_time_delay_start.tv_usec)/1000;  //ms
        if(time_elpase < 240)
            break;		//δ����ʱʱ��
        m_n_calib_step++;
    }
        break;
    case 8: //�ȴ�POS_REACHED��λ��PIN A15, High������λSTROBE�źţ�PIN A7, LOW��
        if(this->m_n_laser_input_signal & 0x20){  //POS_REACHED�ź�Ϊ��
            this->SetHeightRegulatorOutput(5, false);  //��λSTROBE�źţ�PIN A7, Low��

            m_n_calib_step++;
        }

        break;
    case 9: //�ж��Ƿ���������вο��㣬���û��������step5������ִ����һ��
        if(this->m_n_laser_input_signal & 0x20){
            break;   //���POS_REACHED�ź�û�и�λ��ȴ�
        }
        if(++m_n_cur_ref_point < this->m_n_calib_ref_count){ //δ���������вο���
            m_n_calib_step = 5;
            break;
        }
        m_n_calib_step++;
        break;
    case 10:{ //Z���ƶ���35mm�߶ȣ�
        printf("��ʼ�궨COLLISION�ź�\n");
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
        double *pp = &this->m_point_capture.x;
        pp += chn_axis;
        double pos_tar = *pp;
        pos_tar += 35.0;  //Ŀ��λ��

        this->ManualMove(chn_axis, pos_tar, 5000);

    }
        m_n_calib_step++;
        break;
    case 11:{ //�ȴ�Z�ᵽλ
        chn_axis = this->GetChnAxisFromName(AXIS_NAME_Z);
        double *pp = &this->m_point_capture.x;
        pp += chn_axis;
        double pos_tar = *pp;
        pos_tar += 35.0;  //Ŀ��λ��

        if(fabs(this->m_channel_rt_status.cur_pos_machine.GetAxisValue(chn_axis) - pos_tar) < 1e-3){ //��λ
            gettimeofday(&m_time_delay_start, nullptr);  //��¼��ʼʱ��
            printf("Z�ᵽλ\n");
            m_n_calib_step++;
        }
    }
        break;
    case 12:{ //�ȴ�240ms��ʱ
        struct timeval cur_time;
        uint64_t time_elpase = 0;
        gettimeofday(&cur_time, nullptr);
        time_elpase = (cur_time.tv_sec-m_time_delay_start.tv_sec)*1000 + (cur_time.tv_usec - m_time_delay_start.tv_usec)/1000;  //ms
        if(time_elpase < 240)
            break;		//δ����ʱʱ��
        m_n_calib_step++;
    }
        break;
    case 13: //�궨��������λCAL.REQUEST�źţ�PIN A3, LOW��
        this->SetHeightRegulatorOutput(1, false);
        printf("��λCAL.REQUEST�ź�\n");
        m_n_calib_step++;
        break;
    case 14: //�ȴ�READY�ź���λHigh
        if(this->m_n_laser_input_signal & 0x08){//�ȴ�READY�źű��
            this->m_b_laser_calibration = false;
            m_n_calib_step = 0;
            printf("�������궨�ɹ���\n");
        }
        break;
    case 100: //ʧ�ܣ��˳�
        this->ManualMoveStop();   //ֹͣ�������ƶ�
        this->SetHeightRegulatorOutput(1, false);
        this->SetHeightRegulatorOutput(5, false);  //��λSTROBE�źţ�PIN A7, Low��
        this->m_b_laser_calibration = false;
        m_n_calib_step = 0;
        printf("�������궨ʧ�ܣ�\n");
        break;
    default:
        printf("�궨����ִ���쳣!\n");
        break;
    }
}

/**
 * @breif ����������ʹ��
 * @param enable : true -- ʹ�ܸ��湦��     false -- ��ֹ���湦��
 */
void ChannelControl::EnableHeightRegulatorFollow(bool enable){
    uint8_t phy_axis = this->GetPhyAxisFromName(AXIS_NAME_Z);  //�������
    if(phy_axis != NO_AXIS){
        phy_axis += 1;  //MI����Ŵ�1��ʼ
    }
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_ENABLE_FOLLOW;
    cmd.data.axis_index = phy_axis;
    //	cmd.data.reserved = type;
    cmd.data.data[0] = enable?1:0;
    cmd.data.data[1] = 5000;   //����߶�5mm

    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @breif ��ȡ�����������ź�
 */
void ChannelControl::ReadHeightRegulatorInput(){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_READ_LASER_INPUT;
    cmd.data.axis_index = NO_AXIS;


    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief ���õ���������ź�
 * @param idx : 1-8�ֱ��ʾ1-8�����
 * @param value �� true--�ߵ�ƽ    false--�͵�ƽ
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
 * @brief ��MI������λ�ò���ָ��
 * @param axis_mask ���������mask
 * @param io_idx ��1-6 �� �ֱ��ʾIN1-IN6
 * @param level : ��Ч��ƽ�� true-����Ч    false-����Ч
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
 * @brief ����MI���صļ�������������ź�ֵ
 * @param cmd
 */
void ChannelControl::ProcessMiInputSignal(MiCmdFrame &cmd){
    this->m_n_laser_input_signal = cmd.data.data[0];
}

/**
 * @brief ����Mi���ص���λ�ò�����
 * @param cmd
 */
void ChannelControl::ProcessMiPosCapture(MiCmdFrame &cmd){
    if(!this->m_b_laser_calibration)
        return;
    printf("Get pos captured response\n");
    uint64_t tmp = 0x01;
    int64_t tpp = 0;
    if(this->m_n_mask_pos_capture & (tmp<<(cmd.data.axis_index-1))){
        this->m_n_mask_captured |= (tmp<<(cmd.data.axis_index-1));   //���ø��Ჶ��ɹ�

        memcpy(&tpp, cmd.data.data, sizeof(int64_t));    //��ȡλ��

        uint8_t chn_axis = this->GetChnAxisFromPhyAxis(cmd.data.axis_index-1);
        if(chn_axis == NO_AXIS)
            return;
        double *p = &(m_point_capture.x);
        double df = tpp;
        p += chn_axis;
        *p = df/1e7;   //ת����λ  0.1nm-->mm
        printf("capture pos : %lf, %lf, %lld\n", m_point_capture.z, df, tpp);
    }

    if(m_n_mask_captured == m_n_mask_pos_capture)
        this->m_b_pos_captured = true;    //λ�ò������
}

#endif


#ifdef USES_FIVE_AXIS_FUNC
/**
 * @brief ����������ת�����ز���
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
        m_mask_5axis_rot_nolimit = 0;  // ������
        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot1;
        if(m_p_chn_5axis_config->range_limit_2)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot2;

    }
    else if(m_p_chn_5axis_config->five_axis_type == B1_C1_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_B);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_C);
        m_mask_5axis_rot_nolimit = 0;  // ������
        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot1;
        if(m_p_chn_5axis_config->range_limit_2)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot2;
    }
    else if(m_p_chn_5axis_config->five_axis_type == A1_B1_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_A);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_B);
        m_mask_5axis_rot_nolimit = 0;  // ������
        if(m_p_chn_5axis_config->range_limit_1)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot1;
        if(m_p_chn_5axis_config->range_limit_2)
            m_mask_5axis_rot_nolimit |= 0x01<<m_n_5axis_rot2;
    }
    else if(m_p_chn_5axis_config->five_axis_type == B1_A1_FIVEAXIS)
    {
        m_n_5axis_rot1 = this->GetChnAxisFromName(AXIS_NAME_B);
        m_n_5axis_rot2 = this->GetChnAxisFromName(AXIS_NAME_A);
        m_mask_5axis_rot_nolimit = 0;  // ������
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
 * @brief ����ͨ��������ز���
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
 * @brief �����������
 * @param type : ��������
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
    case MACHINE_TYPE: 	//��������
        data = this->m_p_chn_5axis_config->five_axis_type;
        break;
    case X_OFFSET_1:        //��һת��������Ե����X����
        data = this->m_p_chn_5axis_config->x_offset_1*1000;   //��λת����mm->um
        break;
    case Y_OFFSET_1:        //��һת��������Ե����Y����
        data = this->m_p_chn_5axis_config->y_offset_1*1000;   //��λת����mm->um
        break;
    case Z_OFFSET_1:        //��һת��������Ե����Z����
        data = this->m_p_chn_5axis_config->z_offset_1*1000;   //��λת����mm->um
        break;
    case X_OFFSET_2:        //�ڶ�ת��������Ե����X����
        data = this->m_p_chn_5axis_config->x_offset_2*1000;   //��λת����mm->um
        break;
    case Y_OFFSET_2:        //�ڶ�ת��������Ե����Y����
        data = this->m_p_chn_5axis_config->y_offset_2*1000;   //��λת����mm->um
        break;
    case Z_OFFSET_2:        //�ڶ�ת��������Ե����Z����
        data = this->m_p_chn_5axis_config->z_offset_2*1000;   //��λת����mm->um
        break;
    case ROT_SWING_ARM_LEN: //������ת�ڱ۳���
        data = this->m_p_chn_5axis_config->rot_swing_arm_len*1000;   //��λת����mm->um
        break;
    case POST_DIR_1:      //��һת����ת������
        data = this->m_p_chn_5axis_config->post_dir_1;   //
        break;
    case POST_DIR_2:        //�ڶ�ת����ת������
        data = this->m_p_chn_5axis_config->post_dir_2;
        break;

    case SPEED_LIMIT:       //�����ٶ����ƿ���
        data = this->m_p_chn_5axis_config->five_axis_speed_plan;
        break;

    case SPEED_LIMIT_X:    //��������X���ٶ�����
        data = this->m_p_chn_5axis_config->speed_limit_x * 1000. / 60.;    //��λת���� mm/min --> um/s
        break;
    case SPEED_LIMIT_Y:         //��������Y���ٶ�����
        data = this->m_p_chn_5axis_config->speed_limit_y * 1000. / 60.;    //��λת���� mm/min --> um/s
        break;
    case SPEED_LIMIT_Z:         //��������Z���ٶ�����
        data = this->m_p_chn_5axis_config->speed_limit_z * 1000. / 60.;    //��λת���� mm/min --> um/s
        break;
    case SPEED_LIMIT_A:         //��������A���ٶ�����
        data = this->m_p_chn_5axis_config->speed_limit_a * 1000. / 60.;    //��λת���� mm/min --> um/s
        break;
    case SPEED_LIMIT_B:         //��������B���ٶ�����
        data = this->m_p_chn_5axis_config->speed_limit_b * 1000. / 60.;    //��λת���� mm/min --> um/s
        break;
    case SPEED_LIMIT_C:         //��������C���ٶ�����
        data = this->m_p_chn_5axis_config->speed_limit_c * 1000. / 60.;    //��λת���� mm/min --> um/s
        break;
    case SPEED_PLAN_MODE:   //�����ٶȹ滮��ʽ
        data = this->m_p_chn_5axis_config->five_axis_plan_mode;
        break;
    case INTP_ANGLE_STEP:      //���������ֲ岹�ǶȲ���
        data = this->m_p_chn_5axis_config->intp_angle_step * 1000;   //��λת�� �� �� -->  0.001��
        break;
    case INTP_LEN_STEP:        //���������ֲ岹��С����
        data = this->m_p_chn_5axis_config->intp_len_step * 1000;     //��λת����mm->um
        break;
    case PROGRAM_COORD:       //����������ϵ
        data = this->m_p_chn_5axis_config->five_axis_coord;       //����������ϵ
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
 * @brief ���ز��ƹ���
 * @param active : true--����   false--�ر�
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
 * @brief ����MI����ָ��
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
    this->SendWorkCountToHmi(m_channel_status.workpiece_count, m_channel_status.workpiece_count_total);  //֪ͨHMI���¼ӹ�����

    if (m_channel_status.workpiece_require != 0 && m_channel_status.workpiece_count >= m_channel_status.workpiece_require)
    {//�ѵ����������
        CreateError(ERR_REACH_WORK_PIECE, INFO_LEVEL, CLEAR_BY_MCP_RESET);
    }
}

#ifdef USES_SPEED_TORQUE_CTRL

/**
 * @brief ��������CSģʽ�л�����
 * @param msg : Mָ����Ϣ
 * @param index �� M�������
 */
void ChannelControl::ProcessSpdModeSwitch(AuxMsg *msg, uint8_t index){
    if(msg == nullptr)
        return;
    int mCode=-1;
    uint8_t SpdIndex = 0, PhySpdIndex = -1;
    int toMode = -1;


    mCode = msg->GetMCode(index);
    if(mCode == 28){  //�л��ٶ�ģʽ
        SpdIndex = 0;
        toMode = 2;
        PhySpdIndex = this->m_spd_axis_phy[SpdIndex]-1;
        SpdIndex = this->GetSpdChnAxis(SpdIndex);
    }else if(mCode == 29){  //�л���λ��ģʽ
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

        if(PhySpdIndex == 0){    // �᲻���� ֱ�ӷ��أ�����
            msg->SetExecStep(index, 0xFF);
            return;
        }
        int curMode = this->m_channel_status.cur_axis_ctrl_mode[SpdIndex];

        if(toMode == curMode){     // ���ᵱǰģʽ�Ѿ����л�ģʽ��
            msg->SetExecStep(index, 0xFF);
            return;
        }

        if(curMode==2 || curMode==3){
            double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(SpdIndex);
            int32_t feed = fabs(curfeed);
            if(feed <= 1){  // �ٶȽӽ� 0 	    �����л�                                                    // λ��ָ���л�Ϊ�ٶȻ�����
                this->m_channel_status.cur_axis_ctrl_mode[SpdIndex] = toMode;  //��¼��ǰ�����ģʽ
                m_channel_status.rated_spindle_speed = m_n_cur_scode;
                this->SendModeChangToHmi(S_MODE);

                this->SendAxisCtrlModeSwitchCmdToMi(PhySpdIndex, toMode);

                ctrlmode_switch_wait = 30;   //�ȴ�10������
                //				gettimeofday(&ctrlmode_switch_start, NULL);
                //				printf("switch to pos ctrl 111\n");

                msg->SetExecStep(index, 8);
            }else{
                if(curMode==2){          // �ٶȿ���ģʽ�����ٶ�Ϊ0
                    //   this->SendAxisSpeedCmdToMi(PhySpdIndex,0);
                    this->SpindleOut(SPD_DIR_STOP);
                    msg->IncreaseExecStep(index);
                }else if(curMode==3){    // ���ؿ���ģʽ�����ٶ�����Ϊ0
                    this->SendAxisTorqueCmdToMi(PhySpdIndex,0,0,1);
                    msg->IncreaseExecStep(index);
                }
            }
        }else{		                                                        // λ��ָ���л�Ϊ�ٶȻ�����
            this->m_channel_status.cur_axis_ctrl_mode[SpdIndex] = toMode;  //��¼��ǰ�����ģʽ
            this->SendAxisCtrlModeSwitchCmdToMi(PhySpdIndex, toMode);
            msg->SetExecStep(index, 11);
        }

    }else if(msg->GetExecStep(index) == 6){
        // �ж����Ƿ�ֹͣ
        printf("#### waiting for spd stop!!! \n");
        double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(SpdIndex);
        int32_t feed = fabs(curfeed);
        if(feed <= 1){  // �ٶȽӽ� 0
            msg->IncreaseExecStep(index);
        }
    }else if(msg->GetExecStep(index) == 7){		//
        m_channel_status.rated_spindle_speed = m_n_cur_scode;
        this->SendModeChangToHmi(S_MODE);

        this->m_channel_status.cur_axis_ctrl_mode[SpdIndex] = toMode;  //��¼��ǰ�����ģʽ
        this->SendAxisCtrlModeSwitchCmdToMi(PhySpdIndex, toMode);

        if(toMode==0 || toMode==1 ){
            ctrlmode_switch_wait = 10;   //�ȴ�10������

            msg->IncreaseExecStep(index);    // ������л���λ��ģʽ�� ��Ҫȥͬ��λ������
        }else{
            msg->SetExecStep(index, 10);
        }

    }else if(msg->GetExecStep(index) == 8){
        //�ȴ�һ��ִ������10ms���ȴ�MC����ģʽ�л����
        if(--ctrlmode_switch_wait == 0){
            msg->IncreaseExecStep(index);
        }

    }else if(msg->GetExecStep(index) == 9){
        //ͬ��λ��
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        else
            this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        //		m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //		m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        RefreshAxisIntpPos();

        if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
            this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
            //			printf("sync compiler curpos[%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf]\n", m_channel_mc_status.intp_pos.x,
            //					m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z, m_channel_mc_status.intp_pos.a4, m_channel_mc_status.intp_pos.a5,
            //					m_channel_mc_status.intp_pos.a6, m_channel_mc_status.intp_pos.a7, m_channel_mc_status.intp_pos.a8);

        }else
            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //ͬ���ѱ�������ƶ�ָ���λ��


        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 10){

        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 11){
        msg->SetExecStep(index, 0xFF);    //��λ����״̬
    }
}

///**
// * @brief ����M29״̬��λ����
// */
//void ChannelControl::ProcessM29Reset(){
//	uint8_t SpdIndex = 0, PhySpdIndex = -1;
//	int toMode = 2;  //�л��ٶ�ģʽ

//	PhySpdIndex = this->m_spd_axis_phy[SpdIndex]-1;
//	SpdIndex = this->GetSpdChnAxis(SpdIndex);



//	if(PhySpdIndex == 0){    // �᲻���� ֱ�ӽ���
//		m_n_M29_flag = 0;
//		return;
//	}

//	// λ��ָ���л�Ϊ�ٶ�
//	this->m_channel_status.cur_axis_ctrl_mode[SpdIndex] = toMode;  //��¼��ǰ�����ģʽ
//	this->SendAxisCtrlModeSwitchCmdToMi(PhySpdIndex, toMode);

//	m_channel_status.rated_spindle_speed = 0;
//	this->SendModeChangToHmi(S_MODE);

//	m_n_M29_flag = 0;

//}

//struct timeval ctrlmode_switch_start;   //�ȴ���ʼʱ��
/**
 * @brief �������ģʽ��̬�л�Mָ��
 * @param index : M�������
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
                PhyAixsIndex==0xff ||    // �᲻���� ֱ�ӷ��أ�����
                toMode>3){
            msg->SetExecStep(index, 0xFF);
            return;
        }
        int curMode = this->m_channel_status.cur_axis_ctrl_mode[AixsIndex];

        if(toMode == curMode ||     // ���ᵱǰģʽ�Ѿ����л�ģʽ��
                (toMode<=1 && curMode<=1)){  // ֱ��λ��ģʽ �� ��תλ��ģʽ ֮��Ͳ�Ҫ�л���
            msg->SetExecStep(index, 0xFF);
            return;
        }

        if(curMode==2 || curMode==3){
            double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(AixsIndex);
            int32_t feed = fabs(curfeed);
            if(feed <= 1){  // �ٶȽӽ� 0 	    �����л�                                                    // λ��ָ���л�Ϊ�ٶȻ�����
                this->m_channel_status.cur_axis_ctrl_mode[AixsIndex] = toMode;  //��¼��ǰ�����ģʽ
                this->SendAxisCtrlModeSwitchCmdToMi(PhyAixsIndex, toMode);

                ctrlmode_switch_wait = 30;   //�ȴ�10������
                //				gettimeofday(&ctrlmode_switch_start, NULL);
                //				printf("switch to pos ctrl 111\n");

                msg->SetExecStep(index, 3);
            }else{
                if(curMode==2){          // �ٶȿ���ģʽ�����ٶ�Ϊ0
                    this->SendAxisSpeedCmdToMi(PhyAixsIndex,0);
                    msg->IncreaseExecStep(index);
                }else if(curMode==3){    // ���ؿ���ģʽ�����ٶ�����Ϊ0
                    this->SendAxisTorqueCmdToMi(PhyAixsIndex,0,0,1);
                    msg->IncreaseExecStep(index);
                }
            }
        }else{		                                                        // λ��ָ���л�Ϊ�ٶȻ�����
            this->m_channel_status.cur_axis_ctrl_mode[AixsIndex] = toMode;  //��¼��ǰ�����ģʽ
            this->SendAxisCtrlModeSwitchCmdToMi(PhyAixsIndex, toMode);
            msg->SetExecStep(index, 6);
        }

    }else if(msg->GetExecStep(index) == 1){
        // �ж����Ƿ�ֹͣ
        printf("#### waiting for axis stop!!! \n");
        double curfeed = this->m_channel_rt_status.cur_feedbck_velocity.GetAxisValue(AixsIndex);
        int32_t feed = fabs(curfeed);
        if(feed <= 1){  // �ٶȽӽ� 0
            msg->IncreaseExecStep(index);
        }
    }else if(msg->GetExecStep(index) == 2){		//

        this->m_channel_status.cur_axis_ctrl_mode[AixsIndex] = toMode;  //��¼��ǰ�����ģʽ
        this->SendAxisCtrlModeSwitchCmdToMi(PhyAixsIndex, toMode);

        if(toMode==0 || toMode==1 ){
            ctrlmode_switch_wait = 10;   //�ȴ�10������
            //			gettimeofday(&ctrlmode_switch_start, NULL);
            //			printf("switch to pos ctrl 222\n");

            msg->IncreaseExecStep(index);    // ������л���λ��ģʽ�� ��Ҫȥͬ��λ������
        }else{
            msg->SetExecStep(index, 5);
        }

    }else if(msg->GetExecStep(index) == 3){
        //�ȴ�һ��ִ������10ms���ȴ�MC����ģʽ�л����
        if(--ctrlmode_switch_wait == 0){
            msg->IncreaseExecStep(index);
        }

    }else if(msg->GetExecStep(index) == 4){
        //		struct timeval time_now;
        //		gettimeofday(&time_now, NULL);
        //		int time_elpase = (time_now.tv_sec-ctrlmode_switch_start.tv_sec)*1000000+
        //									time_now.tv_usec-ctrlmode_switch_start.tv_usec;

        //ͬ��λ��
        if(!this->m_b_mc_on_arm)
            this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
        else
            this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);

        //		m_channel_rt_status.cur_pos_work = m_channel_mc_status.intp_pos;
        //		m_channel_rt_status.tar_pos_work = m_channel_mc_status.intp_tar_pos;
        RefreshAxisIntpPos();

        if(!this->IsMoveMsgLine() && this->m_p_compiler->IsBlockListEmpty()){
            this->m_p_compiler->SetCurPos(this->m_channel_mc_status.intp_pos);   //ͬ��������λ��
            //			printf("sync compiler curpos[%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf]\n", m_channel_mc_status.intp_pos.x,
            //					m_channel_mc_status.intp_pos.y, m_channel_mc_status.intp_pos.z, m_channel_mc_status.intp_pos.a4, m_channel_mc_status.intp_pos.a5,
            //					m_channel_mc_status.intp_pos.a6, m_channel_mc_status.intp_pos.a7, m_channel_mc_status.intp_pos.a8);

        }else
            this->RefreshOuputMovePos(m_channel_rt_status.cur_pos_work);    //ͬ���ѱ�������ƶ�ָ���λ��


        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 5){

        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 6){

        printf("#### execute M%d succeed! \n", mCode0);
        //		msg->SetExecStep(index, 0);   //Mָ��ִ�н���
        msg->SetExecStep(index, 0xFF);    //��λ����״̬
    }

}


/**
 * @brief ��λ�������ģʽ��ϵͳ��λʱ�������ٶȿ��ƺ����ؿ������Ϊ0
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
                    this->SendAxisTorqueCmdToMi(phy_axis,0,0,1); // ֻ�����ٶ�����ֵΪ0
                    flag = true;
                    printf("---ResetAllAxisOutZero axis %d  torque zero\n",i);
                }
            }
        }
    }
    return flag;
}

/**
 * @brief ��λ�������ģʽ��ϵͳ��λʱ���ָ�ͨ����������Ŀ���ģʽΪĬ��ģʽ, ��Ҫ���ڲ��˶���ʱ�����л�
 * @param flag: ǿ���л���ʶ�� 0--���ٶ�Ϊ0���л�   1--ǿ���л�
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
                    if(flag>0 || feed<5){  // �ٶȽӽ� 0  ����Ϊǿ���л�״̬
                        this->m_channel_status.cur_axis_ctrl_mode[i] = m_p_axis_config[phy_axis].axis_type;  //��¼��ǰ�����ģʽ
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
 * @brief �����������ͨ����֮��ӳ���л�
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

    if(msg->GetExecStep(index) == 0){  // ��һ�����ѵ�ǰ���л���PLC��
        printf("execute M%d\n", mCode);

        cur_phy_axis = this->GetPhyAxis(cur_chan_axis);
        cur_chan = this->m_n_channel_index;
        other_chan = this->m_p_channel_engine->GetAxisChannel(other_phy_axis, other_chan_axis);

        if(cur_phy_axis==0xff) return;  // �᲻���� ֱ�ӷ���
        if(cur_phy_axis==other_phy_axis) return;  // �л�Ŀ�����뵱ǰ����ͬ��ֱ�ӷ���

        if(cur_chan != other_chan){
            int64_t taxis = 0x01<<(cur_phy_axis-1);
            //this->m_n_real_phy_axis &= (~(taxis));
        }
        //	this->m_map_phy_axis_chn[m_p_channel_config[i].chn_axis_phy[j]-1] = i;
        //	this->m_channel_status.cur_chn_axis_phy[] = ;
        
        //   ����ǰ���Ӧ���������е�PLCͨ��
        this->SendAxisMapCmdToMi(cur_phy_axis, 0xFF,0);

        msg->IncreaseExecStep(index);  //�˴������һ������Mָ�����ִ��
    }else if(msg->GetExecStep(index) == 1){  // �ڶ�������Ŀ���������е���ǰͨ����ǰ��
        
        if(cur_chan != other_chan){
            int64_t taxis = 0x01<<(other_phy_axis-1);
            //this->m_n_real_phy_axis |= taxis;
            //		   this->m_map_phy_axis_chn[other_phy_axis-1] = i;
        }
        this->m_channel_status.cur_chn_axis_phy[cur_chan_axis] = other_phy_axis;
        //  ����ǰͨ����ӳ�䵽Ŀ��������
        this->SendAxisMapCmdToMi(other_phy_axis,this->m_n_channel_index,cur_chan_axis);

        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 2){   // ���������ѵ�һ������������PLCͨ���л���Ŀ��ͨ��
        //
        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 3){


        msg->IncreaseExecStep(index);
    }else if(msg->GetExecStep(index) == 4){

        //		msg->SetExecStep(index, 0);   //Mָ��ִ�н���
        msg->SetExecStep(index, 0xFF);    //��λ����״̬
    }
}

/**
 * @brief ����MI���ص���ת������Ӧ
 * @param cmd : MI�����
 */
void ChannelControl::ProcessSkipCmdRsp(MiCmdFrame &cmd){
    uint8_t axis = cmd.data.data[5]-1;
    int64_t pos = 0;


    uint64_t tmp = 0x01;
    if(this->m_n_mask_pos_capture & (tmp<<axis)){
        this->m_n_mask_captured |= (tmp<<axis);   //���ø��Ჶ��ɹ�

        memcpy(&pos, cmd.data.data, sizeof(int64_t));    //��ȡλ��



        double df = pos;

        m_point_capture.m_df_point[axis] = df/1e7;   //ת����λ  0.1nm-->mm
        printf("capture axis[%hhu] pos : %lf\n", axis, m_point_capture.m_df_point[axis]);
    }

    if(m_n_mask_captured == m_n_mask_pos_capture){
        this->m_b_pos_captured = true;    //λ�ò������
    }
}

/**
 * @brief �ֶ��Ե�����ʹ��MDAģʽִ��G37ָ������ֶ��Ե�
 * @param h_code �� д��ĵ�ƫHֵ
 * @param times : �ظ��Ե�����
 */
void ChannelControl::ManualToolMeasure(int h_code, int times){

    printf("ChannelControl::ManualToolMeasure: hcode=%d, times=%d\n", h_code, times);
    //׼���Ե�ָ�����
    char cmd_buf[256];
    memset (cmd_buf, 0x00, 256);
    sprintf(cmd_buf, "G37 H%dL%d", h_code, times);

    //��MDA��Ӧ��NC�ļ�
    int fp = open(m_str_mda_path, O_CREAT|O_TRUNC|O_WRONLY); //���ļ�
    if(fp < 0){
        CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
        this->m_error_code = ERR_OPEN_FILE;
        g_ptr_trace->PrintLog(LOG_ALARM, "��MDA��ʱ�ļ�[%s]ʧ�ܣ�", m_str_mda_path);
        return;//�ļ���ʧ��
    }
    else{
        printf("open file %s\n", m_str_mda_path);
    }

    int len = strlen(cmd_buf);
    ssize_t res = write(fp, cmd_buf, len);
    if(res == -1){//д��ʧ��
        close(fp);
        CreateError(ERR_LOAD_MDA_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
        this->m_error_code = ERR_LOAD_MDA_DATA;
        return;
    }else if(res != len){//����ȱʧ
        close(fp);
        CreateError(ERR_LOAD_MDA_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
        this->m_error_code = ERR_LOAD_MDA_DATA;
        printf("write mda temp file error, plan = %d, actual = %d\n", len, res);
        return;
    }

    close(fp);

    this->m_p_compiler->SetCurPos(this->m_channel_rt_status.cur_pos_work);

    if(!m_p_compiler->OpenFile(m_str_mda_path)){		//���������ļ�ʧ��
        return;
    }

    printf("ManualToolMeasure() m_n_run_thread_state: %d\n", m_n_run_thread_state);
    if(m_n_run_thread_state == IDLE || m_n_run_thread_state == PAUSE){
        pthread_mutex_lock(&m_mutex_change_state);
        //printf("locked 16\n");
        m_n_run_thread_state = RUN;  //��Ϊ����״̬
        this->m_b_manual_tool_measure = true;
        pthread_mutex_unlock(&m_mutex_change_state);
        //printf("unlocked 16\n");
    }
}

/**
 * @brief ��ȡָ����ƫ������
 * @param idx[in] : ��ƫֵ����1��ʼ, 0�ǻ�׼��ƫ
 * @param cfg[out] �� ���ص�ƫ����
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
 * @brief ֪ͨHMI����ƫ�ò���ֵ�������
 * @param h_idx : Hֵ��������1��ʼ
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
 * @brief ֪ͨHMI������Ϣ�����ı�
 * @param tool_index : ֵ����
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
 * @brief ֪ͨHMI��������ϵ���÷������
 * @param coord_idx : ��������ϵ������0--����ƫ��   1~6--G54~G59
 * @return true--�ɹ�  false--ʧ��
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
 * @brief ֪ͨHMI��չ����ϵ���÷������
 * @param coord_idx : ��������ϵ������0--����ƫ��   1~99--G5401~G5499
 * @return true--�ɹ�  false--ʧ��
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
 * @brief �ж��Ƿ񻺳������Ϣ
 * @param msg : ��Ϣָ��
 * @return true--��Ҫ��ջ���    false--����Ҫ��ջ���
 */
bool ChannelControl::IsBufClearMsg(RecordMsg *msg){
    bool res = true;
    int type = msg->GetMsgType();
    if(type == LINE_MSG || type == ARC_MSG || type == RAPID_MSG || type == FEED_MSG ||
            type == SPEED_MSG || type == COMPENSATE_MSG || type == LOOP_MSG ||
            type == COORD_MSG || type == SUBPROG_CALL_MSG || type == MACRO_MSG ||
            type == SUBPROG_RETURN_MSG || type == MACRO_PROG_CALL_MSG ){
        res = false;
    }else if(type == AUX_MSG){  //TODO M03/M04/M05����ָ���������

    }else if(type == MODE_MSG){   //TODO ֧������ģ̬��Ϣ
        //		int gcode = ((ModeMsg *)msg)->GetGCode();
        res = false;
    }


    return res;
}

/**
 * @brief �Ƿ����ƶ���Ϣ(G00��G01��G02��G03)
 * @param msg : ��Ϣָ��
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
 * @brief ���Զ����ݻ����в���ָ��֡�ŵ�����
 * @param index : ֡���
 * @return �ҵ���msgָ��
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
 * @brief ����MI���͵����ָ���״̬�л�ָ��
 * @param cmd : MI���͵�ָ��
 */
void ChannelControl::ProcessMiHWTraceStateChanged(MiCmdFrame &cmd){
    cmd.data.cmd |= 0x8000;    //�ظ�֡
    HWTraceState state_new = (HWTraceState)cmd.data.data[0];   //��״̬

    printf("ProcessMiHWTraceStateChanged: newstate = %d\n", (int)state_new);

    if(/*m_channel_status.chn_work_mode != AUTO_MODE ||*/   //���������ʱֱ���л�ģʽ���˴���������ģʽ
            (cmd.data.data[0] == REVERSE_TRACE && this->m_p_general_config->hw_rev_trace == 0)){
        cmd.data.data[1] = 0x00;   //ʧ�ܣ������������ܹر�

        this->m_p_mi_comm->WriteCmd(cmd);
    }else{
        cmd.data.data[1] = 0x01;  //�л��ɹ�

        if(this->m_n_hw_trace_state == NONE_TRACE && state_new == NORMAL_TRACE){  //�����ָ���ģʽ -->  �������ָ���ģʽ
            this->m_n_hw_trace_state = NORMAL_TRACE;

            this->m_p_mi_comm->WriteCmd(cmd);

        }else if(this->m_n_hw_trace_state == NORMAL_TRACE && state_new == NONE_TRACE){  //�������ָ���ģʽ  --> �����ָ���ģʽ
            this->m_n_hw_trace_state = NONE_TRACE;

            this->m_p_mi_comm->WriteCmd(cmd);

        }else if(this->m_n_hw_trace_state == NORMAL_TRACE && state_new == REVERSE_TRACE){  //�������ָ���ģʽ  --> �������ָ���ģʽ
            if(m_channel_status.machining_state != MS_RUNNING ||
                    (m_channel_mc_status.cur_mode ==MC_MODE_MANUAL && m_channel_mc_status.cur_feed < 3)){ //�����л�����
                this->ChangeHwTraceState(state_new);

                this->m_p_mi_comm->WriteCmd(cmd);

            }else{ //�������л��������ȴ�MC��ͣ��λ
                this->m_n_hw_trace_state_change_to = state_new;
                this->m_b_change_hw_trace_state = true;
            }


        }else if(this->m_n_hw_trace_state == REVERSE_TRACE && state_new == NORMAL_TRACE){  //�������ָ���ģʽ  --> �������ָ���ģʽ
            if(m_channel_status.machining_state != MS_RUNNING ||
                    (m_channel_mc_status.cur_mode ==MC_MODE_MANUAL && m_channel_mc_status.cur_feed < 3)){ //�����л�����
                this->ChangeHwTraceState(state_new);

                this->m_p_mi_comm->WriteCmd(cmd);

            }else{ //�������л��������ȴ�MC��ͣ��λ
                this->m_n_hw_trace_state_change_to = state_new;
                this->m_b_change_hw_trace_state = true;
            }
        }else{  //�������͵��л����ڷǷ�
            cmd.data.data[1] = 0x00;   //ʧ�ܣ������������ܹر�

            this->m_p_mi_comm->WriteCmd(cmd);
        }

    }
}

/**
 * @brief �л����ָ���״̬
 * @param state : ���л�����״̬
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelControl::ChangeHwTraceState(HWTraceState state){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "ChangeHwTraceState: newstate = %d\n", (int)state);
    //ͬ����ǰλ��
    if(!this->m_b_mc_on_arm)
        this->m_p_mc_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
    else
        this->m_p_mc_arm_comm->ReadAxisIntpPos(m_n_channel_index, m_channel_mc_status.intp_pos, m_channel_mc_status.intp_tar_pos);
    this->RefreshAxisIntpPos();

    //��ȡ��ǰ֡��
    uint16_t cur_frame_idx = 0;
    if(!this->m_b_mc_on_arm)
        this->m_p_mc_comm->ReadChnCurFrameIndex(m_n_channel_index, cur_frame_idx);
    else
        this->m_p_mc_arm_comm->ReadChnCurFrameIndex(m_n_channel_index, cur_frame_idx);


    //�ҵ���ǰ����֡
    ListNode<RecordMsg *> *node = this->FindMsgNode(cur_frame_idx);
    if(m_channel_status.machining_state == MS_RUNNING && node == nullptr && this->m_p_output_msg_list_auto->GetLength() > 0){  //������
        printf("Failed to find cmd frame index = %hu\n", cur_frame_idx);
        this->m_error_code = ERR_NO_CUR_RUN_DATA;
        CreateError(ERR_NO_CUR_RUN_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        return false;
    }

    pthread_mutex_lock(&m_mutex_change_state);
    //printf("locked 17\n");
    //��յ�ǰMC�е��Զ���������
    this->InitMcIntpAutoBuf();

    //���û��巢��ָ���Լ����ָ���״̬
    if(state == NORMAL_TRACE){
        if(node != nullptr)
            this->m_p_last_output_msg = node->pre;
        g_ptr_alarm_processor->RemoveWarning(this->m_n_channel_index, ERR_HW_REV_OVER);    //ɾ������澯
    }
    else if(state == REVERSE_TRACE){
        if(node != nullptr)
            this->m_p_last_output_msg = node->next;
    }
    this->m_n_hw_trace_state = state;

    pthread_mutex_unlock(&m_mutex_change_state);
    //printf("unlocked 17\n");


    // ����������Ϣ
    //	this->StartMcIntepolate();
    m_b_mc_need_start = true;

    return true;
}

/**
 * @brief ��ȡͼ��ģʽ�µĸ�Ƶ��λ������, ��ȡXYZ������ķ���λ��
 */
void ChannelControl::ReadGraphAxisPos(){
    if(this->m_n_graph_pos_count == kGraphPosBufCount-1){
        //�����������ȷ���
        this->SendHmiGraphPosData();
    }

    this->m_p_mi_comm->ReadPhyAxisFbPos(&m_pos_graph_array[m_n_graph_pos_write_idx].x, this->m_n_xyz_axis_phy, 3);

    if(++m_n_graph_pos_write_idx >= kGraphPosBufCount)
        m_n_graph_pos_write_idx = 0;

    this->m_n_graph_pos_count++;
}

/**
 * @brief ��HMI���ͻ�ͼλ������
 */
void ChannelControl::SendHmiGraphPosData(){
    if(this->m_n_graph_pos_count == 0)
        return;   //������

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

    //��������
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
 * @brief ���ú����
 * @param macro_index : �������
 */
bool ChannelControl::CallMacroProgram(uint16_t macro_index){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_CONTROL_SC, "ChannelControl[%hhu]::CallMacroProgram, macro_index = %hu\n", this->m_n_channel_index, macro_index);
    uint8_t mode = this->m_channel_status.chn_work_mode;
    uint8_t state = this->m_channel_status.machining_state;
    //�Զ���MDAģʽ(������MS_RUNNING״̬)
    if((mode == AUTO_MODE || mode == MDA_MODE) && state == MS_RUNNING){
        this->m_p_compiler->CallMarcoProgWithNoPara(macro_index);
        m_n_subprog_count++;
        m_n_macroprog_count++;

        if(this->m_p_general_config->debug_mode == 0){
            this->SetMcStepMode(false);
        }

    }else if(mode == MANUAL_STEP_MODE || mode == MANUAL_MODE || mode == MPG_MODE ||
             mode == REF_MODE || state != MS_RUNNING){  //�ֶ�������ģʽ��������״̬��ɣ�
        if(this->m_b_manual_call_macro){  //�Ѿ������ֶ�����������
            g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "�Ѵ����ֶ�����������ִ�й��̣�[%hu]", macro_index);
            return false;
        }
        if(!m_p_compiler->FindSubProgram(macro_index, true)){
            return false;
        }

        char cmd_buf[256];
        memset (cmd_buf, 0x00, 256);
        sprintf(cmd_buf, "G65P%hu", macro_index);

        //��MDA��Ӧ��NC�ļ�
        int fp = open(m_str_mda_path, O_CREAT|O_TRUNC|O_WRONLY); //���ļ�
        if(fp < 0){
            CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
            this->m_error_code = ERR_OPEN_FILE;
            g_ptr_trace->PrintLog(LOG_ALARM, "��MDA��ʱ�ļ�[%s]ʧ�ܣ�", m_str_mda_path);
            return false;//�ļ���ʧ��
        }
        else{
            printf("open file %s\n", m_str_mda_path);
        }

        int len = strlen(cmd_buf);
        ssize_t res = write(fp, cmd_buf, len);
        if(res == -1){//д��ʧ��
            close(fp);
            CreateError(ERR_LOAD_MDA_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
            this->m_error_code = ERR_LOAD_MDA_DATA;
            return false;
        }else if(res != len){//����ȱʧ
            close(fp);
            CreateError(ERR_LOAD_MDA_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, errno, m_n_channel_index);
            this->m_error_code = ERR_LOAD_MDA_DATA;
            printf("write mda temp file error, plan = %d, actual = %d\n", len, res);
            return false;
        }

        close(fp);

        this->m_p_compiler->SetCurPos(this->m_channel_rt_status.cur_pos_work);

        if(!m_p_compiler->OpenFile(m_str_mda_path)){		//���������ļ�ʧ��
            printf("CallMacroProgram:compiler open file failed\n");
            return false;
        }

        printf("CallMacroProgram() m_n_run_thread_state: %d\n", m_n_run_thread_state);
        if(m_n_run_thread_state == IDLE || m_n_run_thread_state == PAUSE){
            pthread_mutex_lock(&m_mutex_change_state);
            //printf("locked 18\n");
            m_n_run_thread_state = RUN;  //��Ϊ����״̬
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
 * @brief ���ø��ӳ���ǰ��/���ã�
 * @param type : �������ͣ� 1--�ӹ�����ǰ�ó���   2--�ϵ����ǰ�ó���   3--�ӹ���λǰ�ó���    11--�������ó���    12--��ͣ���ó���    13--��λ���ó���
 * @return  true--�ɹ�   false--ʧ��
 */
bool ChannelControl::CallAdditionalProgram(AddProgType type){

    if(this->m_n_add_prog_type != NONE_ADD){
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "�Ѵ��ڸ��ӳ���ִ�й��̣�[%hhu:%hhu]", m_n_add_prog_type, type);

        return false;
    }

    if(this->m_channel_status.chn_work_mode != AUTO_MODE){
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "���Զ�ģʽ���ܾ�ִ�и�������[��ǰģʽ��%hhu]", m_channel_status.chn_work_mode);

        return false;
    }

    this->m_n_add_prog_type = type;
    if(type == NORMAL_START_ADD || type == RESET_START_ADD){
        //		if(this->m_channel_status.machining_state != MS_RUNNING){  //������״̬������Ӧ
        //			return false;
        //		}

        this->m_n_sub_count_bak = m_n_subprog_count;   //��¼�ӳ���Ƕ�ײ���

        if(type == NORMAL_START_ADD)
            this->m_p_compiler->CallMarcoProgWithNoPara(9701, false);
        else if(type == RESET_START_ADD)
            this->m_p_compiler->CallMarcoProgWithNoPara(9703, false);
        m_n_subprog_count++;
        m_n_macroprog_count++;



        //		if(this->m_p_general_config->debug_mode == 0){
        this->SetMcStepMode(false);
        //		}

    }else if(type == CONTINUE_START_ADD){  //��MDAͨ��ִ��
        char cmd_buf[256];
        memset (cmd_buf, 0x00, 256);
        sprintf(cmd_buf, "G65P%hu", 9702);

        //��MDA��Ӧ��NC�ļ�
        int fp = open(m_str_mda_path, O_CREAT|O_TRUNC|O_WRONLY); //���ļ�
        if(fp < 0){
            CreateError(ERR_EXEC_FORWARD_PROG, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 1, m_n_channel_index);
            this->m_error_code = ERR_EXEC_FORWARD_PROG;
            g_ptr_trace->PrintLog(LOG_ALARM, "��MDA��ʱ�ļ�[%s]ʧ�ܣ�", m_str_mda_path);
            return false;//�ļ���ʧ��
        }

        int len = strlen(cmd_buf);
        ssize_t res = write(fp, cmd_buf, len);
        if(res == -1){//д��ʧ��
            close(fp);
            CreateError(ERR_EXEC_FORWARD_PROG, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 2, m_n_channel_index);
            this->m_error_code = ERR_EXEC_FORWARD_PROG;
            g_ptr_trace->PrintLog(LOG_ALARM, "д��MDA��ʱ�ļ�[%s]ʧ�ܣ�", m_str_mda_path);
            return false;
        }else if(res != len){//����ȱʧ
            close(fp);
            CreateError(ERR_EXEC_FORWARD_PROG, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 3, m_n_channel_index);
            this->m_error_code = ERR_EXEC_FORWARD_PROG;
            g_ptr_trace->PrintLog(LOG_ALARM, "д��MDA��ʱ�ļ�[%s]ʧ�ܣ�[%d:%d]", m_str_mda_path, len, res);
            return false;
        }

        close(fp);

        this->SaveAutoScene(false);   //����

        //�������л�ΪMDAģʽ
        pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
        m_p_output_msg_list = m_p_output_msg_list_mda;
        m_n_run_thread_state = IDLE;
        this->m_p_compiler->SetMode(MDA_COMPILER);	//�������л�ģʽ
        this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //�л��������
        pthread_mutex_unlock(&m_mutex_change_state);

        //��ʼ�����ݸ�MCģ���ģ̬����Ϣ
        this->InitMcModeStatus();
        this->m_p_compiler->SetCurPos(this->m_channel_rt_status.cur_pos_work);

        if(!m_p_compiler->OpenFile(m_str_mda_path)){		//���������ļ�ʧ��

            //�ָ��Զ�ģʽ
            pthread_mutex_lock(&m_mutex_change_state);  //�ȴ����������߳�ֹͣ
            m_p_output_msg_list = m_p_output_msg_list_auto;
            m_n_run_thread_state = PAUSE;
            this->m_p_compiler->SetMode(AUTO_COMPILER);	//�������л�ģʽ
            this->m_p_compiler->SetOutputMsgList(m_p_output_msg_list);  //�л��������
            pthread_mutex_unlock(&m_mutex_change_state);

            CreateError(ERR_EXEC_FORWARD_PROG, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 4, m_n_channel_index);
            this->m_error_code = ERR_EXEC_FORWARD_PROG;
            g_ptr_trace->PrintLog(LOG_ALARM, "��������MDA�ļ�[%s]ʧ�ܣ�", m_str_mda_path);

            return false;
        }

        this->m_n_subprog_count = 0;
        this->m_n_macroprog_count = 0;

        this->InitMcIntpMdaBuf();   //��ʼ��MCģ���MDA����

        this->SetMcStepMode(false);

        this->SetCurLineNo(this->m_channel_rt_status.line_no);
        if(m_n_run_thread_state == IDLE || m_n_run_thread_state == PAUSE){
            pthread_mutex_lock(&m_mutex_change_state);
            m_n_run_thread_state = RUN;  //��Ϊ����״̬
            pthread_mutex_unlock(&m_mutex_change_state);
            printf("m_n_run_thread_state = %d\n", m_n_run_thread_state);
        }

    }else if(type == RESET_START_ADD){

    }else if(type == NORMAL_END_ADD){

    }else if(type == PAUSE_ADD){//��MDAͨ��ִ��

    }else if(type == RESET_ADD){

    }else{
        this->m_n_add_prog_type = NONE_ADD;
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_CONTROL_SC, "��֧�ֵĸ��ӳ���ģʽ��[%hhu]", type);
        return false;
    }


    return true;
}
#endif

/**
 * @brief �����������
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

    /********************************************************************���MCģ��debug����************************************************************/
    uint32_t debug[16];
    memset(debug, 0x00, 64);
    this->m_p_mc_comm->ReadDebugData(debug);
    printf("MC debug info: TTT1 = %d, TTT2 = %d, TTT3 = %d, TTT4 = %d, TTT5 = %d, TTT6 = %d, TTT7 = %d, TTT8 = %d\n", debug[0], debug[1],
            debug[2], debug[3], debug[4], debug[5], debug[6], debug[7]);
    printf("MC debug info: TTT9 = %d, TTT10 = %d, TTT11 = %d, TTT12 = %d, TTT13 = %d, TTT14 = %d, TTT15 = %d, TTT16 = %d\n", debug[8],
            debug[9], debug[10], debug[11], debug[12], debug[13], debug[14], debug[15]);

}

/**
 * @brief �����������
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
// ֱ����mc�˶������м��� G01 ����   �Զ�ģʽ�»�����˶�  ����ģʽ�»�������ݲ�������ʱ�˶�
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

// ֱ����mc�˶������м��� G00 ����  ����Ч��ͬ��
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
    double init_z_plane, z_cur;  //��ʼƽ��

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

