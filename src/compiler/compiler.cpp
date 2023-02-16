/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file Compiler.cpp
 *@author gonghao
 *@date 2020/03/19
 *@brief ���ļ�ΪG������������ʵ��
 *@version
 */

#include "compiler.h"
#include "global_include.h"
#include "lexer.h"
#include "comm_data_definition.h"
#include "mc_communication.h"
#include "channel_control.h"
#include "variable.h"
#include "parm_manager.h"

//template<> int ListNode<LabelOffset>::new_count = 0;
//template<> int ListNode<SubProgOffset>::new_count = 0;
//template<> int ListNode<LoopRec>::new_count = 0;

/**
 * @brief �������๹�캯��
 */
Compiler::Compiler(ChannelControl *chn_ctrl) {
    // TODO Auto-generated constructor stub

    m_p_channel_control = chn_ctrl;
    m_n_channel_index = chn_ctrl->GetChnIndex();
    m_p_variable = chn_ctrl->GetMacroVar();
    m_p_channel_config = nullptr;

    m_p_block_msg_list = nullptr;
    m_p_parser_result = nullptr;
    m_p_parser_res_auto = nullptr;
    m_p_parser_res_mda = nullptr;

    m_p_last_move_msg = nullptr;

    m_p_simulate_mode = chn_ctrl->GetSimulateModePtr();

    //m_tool_compensate.SetChannelIndex(m_n_channel_index);

    m_thread_prescan = 0;

    //#ifdef USES_FIVE_AXIS_FUNC
    //	this->m_p_five_axis_config = ParmManager::GetInstance()->GetFiveAxisConfig(m_n_channel_index);   //��ʼ����������ָ��
    //#endif

    m_p_channel_config = ParmManager::GetInstance()->GetChannelConfig(
                m_n_channel_index);  //��ʼ��ͨ������
    m_p_mc_communication = MCCommunication::GetInstance();   //��ʼ��MCͨѶ�ӿ�

    InitCompiler();
    if (m_error_code != ERR_NONE) {
        printf("Failed to init compiler!\n");
    }

    if(this->m_p_tool_compensate_auto != nullptr)
    {
        this->m_p_tool_compensate_auto->SetChannelIndex(m_n_channel_index);
    }
    if(this->m_p_tool_compensate_mda != nullptr)
    {
        this->m_p_tool_compensate_mda->SetChannelIndex(m_n_channel_index);
    }
}

/**
 * @brief ����������������
 */
Compiler::~Compiler() {
    // TODO Auto-generated destructor stub

    if (m_p_lexer != nullptr) {
        delete m_p_lexer;
        m_p_lexer = nullptr;
    }
    if (m_p_parser != nullptr) {
        delete m_p_parser;
        m_p_parser = nullptr;
    }
    if (m_p_parser_res_auto != nullptr) {
        delete m_p_parser_res_auto;
        m_p_parser_res_auto = nullptr;
    }
    if (m_p_parser_res_mda != nullptr) {
        delete m_p_parser_res_mda;
        m_p_parser_res_mda = nullptr;
    }

    if (m_p_block_msg_list_auto != nullptr) {
        delete m_p_block_msg_list_auto;
        m_p_block_msg_list_auto = nullptr;
    }
    if (m_p_block_msg_list_mda != nullptr) {
        delete m_p_block_msg_list_mda;
        m_p_block_msg_list_mda = nullptr;
    }

    if (m_p_tool_compensate_auto != nullptr) {
        delete m_p_tool_compensate_auto;
        m_p_tool_compensate_auto = nullptr;
    }
    if (m_p_tool_compensate_mda != nullptr) {
        delete m_p_tool_compensate_mda;
        m_p_tool_compensate_mda = nullptr;
    }

    if (m_p_file_map_info != nullptr) {
        delete m_p_file_map_info;
        m_p_file_map_info = nullptr;
    }

    if(this->m_p_list_label){
        delete m_p_list_label;
        m_p_list_label=nullptr;
    }

    if(this->m_p_list_subprog){
        delete m_p_list_subprog;
        m_p_list_subprog=nullptr;
    }

    if(this->m_p_list_loop){
        delete m_p_list_loop;
        m_p_list_loop=nullptr;
    }

    // ����if else ���ݼ�¼����
    //if(this->m_p_list_ifelse){
    //delete m_p_list_ifelse;
    //m_p_list_ifelse = nullptr;
    //}

#ifdef USES_WOOD_MACHINE
    if(this->m_p_list_spd_start == nullptr){
        delete m_p_list_spd_start;
        m_p_list_spd_start=nullptr;
    }
#endif

    //	if(m_p_file_map_info_mda != nullptr){
    //		delete m_p_file_map_info_mda;
    //		m_p_file_map_info_mda = nullptr;
    //	}

}

/**
 * @brief ��������ʼ��
 */
void Compiler::InitCompiler() {
    printf("Enter Compiler::InitCompiler\n");

    m_b_check = true;   //Ĭ�Ͻ����﷨���
    m_b_has_over_code = false;  //Ĭ��û�н���ָ��

    m_n_sub_program = MAIN_PROG;  //Ĭ����������״̬
    m_c_end_line = ';';   //�����н�����
    m_ln_cur_line_no = 1;   //��ǰ�����к�

    this->m_b_breakout_prescan = false;
    this->m_b_prescan_over = false;
    m_b_prescan_in_stack = false;

    memset(m_line_buf, 0x00, kMaxLineSize); //��ʼ�������еĻ�����

    m_p_cur_file_pos = nullptr;
    m_ln_read_size = 0;

    this->m_lexer_result.Reset();

    m_stack_scene.empty();
    m_stack_loop.empty();

    this->m_compiler_status.exact_stop_flag = false;
    this->m_compiler_status.jump_flag = false;

    //��ʼ�������ļ���������
    m_p_file_map_info = new AsFileMapInfo();
    if (m_p_file_map_info == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�����������ļ�����ʧ��!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;
        return;
    }

    //��ʼ����ǩ����
    this->m_p_list_label = new LabelOffsetList();
    if (m_p_list_label == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]������������ǩ����ʧ��!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;
        return;
    }

    //��ʼ���ӳ������
    this->m_p_list_subprog = new SubProgOffsetList();
    if (m_p_list_subprog == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]������������ǩ����ʧ��!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;
        return;
    }

    //��ʼ��ѭ������ĩ��λ�ü�¼����
    this->m_p_list_loop = new LoopRecList();
    if (m_p_list_loop == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]����������ѭ�������ʧ��!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;
        return;
    }

#ifdef USES_WOOD_MACHINE
    //��ʼ����������ָ���¼����
    this->m_p_list_spd_start = new SpindleStartList();
    if (m_p_list_spd_start == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]������������������ָ�����ʧ��!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;
        return;
    }
#endif
    //��ʼ������MDA�ļ���������
    //	m_p_file_map_info_mda = new AsFileMapInfo();
    //	if(m_p_file_map_info_mda == nullptr){
    //		g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]����������MDA�ļ�����ʧ��!", m_n_channel_index);
    //		m_error_code = ERR_SC_INIT;
    //		m_n_thread_state = ERROR;
    //		return;
    //	}
    //
    //	m_p_file_map_info = m_p_file_map_info_auto;  //��ǰ�ļ����������ʼ��ΪAUTO�ļ�����

    //��ʼ��AUTOģʽ�﷨�����������
    this->m_p_parser_res_auto = new CompileRecList();
    if (this->m_p_parser_res_auto == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�����������Զ�ģʽ�﷨�����������ʧ��!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //��ʼ��ʧ��
        //	m_n_thread_state = ERROR;
        return;
    }
    //��ʼ��MDAģʽ�﷨�����������
    this->m_p_parser_res_mda = new CompileRecList();
    if (this->m_p_parser_res_mda == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]����������MDAģʽ�﷨�����������ʧ��!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //��ʼ��ʧ��
        //	m_n_thread_state = ERROR;
        return;
    }

    this->m_p_parser_result = m_p_parser_res_auto;  //Ĭ��ָ���Զ�ģʽ����

    this->m_p_lexer = new Lexer(m_line_buf, &m_lexer_result);

    this->m_p_parser = new Parser(m_n_channel_index, &m_lexer_result,
                                  m_p_parser_result, &m_compiler_status);
    if (m_p_lexer == nullptr || m_p_parser == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�����������ʷ�/�﷨������ʧ��!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //��ʼ��ʧ��
        //	m_n_thread_state = ERROR;
        return;
    }
    this->m_p_parser->SetMacroVar(this->m_p_variable);
    this->m_p_parser->SetLastMoveRecPointer(&this->m_p_last_move_msg);

    //����AUTOģʽ�ֶδ�����
    this->m_p_block_msg_list_auto = new OutputMsgList();
    if (this->m_p_block_msg_list_auto == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�����������Զ�ģʽ�ֶ�ָ����Ϣ����ʧ��!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //��ʼ��ʧ��
        return;
    }
    //����MDAģʽ�ֶδ�����
    this->m_p_block_msg_list_mda = new OutputMsgList();
    if (this->m_p_block_msg_list_mda == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]����������MDAģʽ�ֶ�ָ����Ϣ����ʧ��!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //��ʼ��ʧ��
        return;
    }

    this->m_p_block_msg_list = this->m_p_block_msg_list_auto;

    //����AUTOģʽ����������
    this->m_p_tool_compensate_auto = new ToolCompensate();
    if (this->m_p_tool_compensate_auto == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�����������Զ�ģʽ����ָ����Ϣ����ʧ��!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //��ʼ��ʧ��
        return;
    }
    //����MDAģʽ����������
    this->m_p_tool_compensate_mda = new ToolCompensate();
    if (this->m_p_tool_compensate_mda == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]����������MDAģʽ����ָ����Ϣ����ʧ��!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //��ʼ��ʧ��
        return;
    }
    this->m_p_tool_compensate = this->m_p_tool_compensate_auto;

    //״̬��ʼ��
    m_b_eof = false;
    m_b_comment = false;
    m_b_left = false;
    m_b_compile_over = false;
    m_error_code = ERR_NONE;

    m_work_mode = AUTO_COMPILER;  //Ĭ�����Զ�ģʽ

    m_n_compile_state = FILE_HEAD;
    m_n_head_state = HEAD_INFO;

    printf("Exit Compiler::InitCompiler\n");
}

/**
 * @brief ���ñ�������ǰ��λ�ã�ֻ���NC��
 * @param cur_pos : ��ǰλ��
 * @return true--�ɹ�   false--ʧ��
 */
bool Compiler::SetCurPos(const DPoint &cur_pos) {
    //	if(this->m_n_thread_state != IDLE)
    //		return false;
    //	printf("Compiler::SetCurPos: old(%lf, %lf, %lf, %lf, %lf, %lf)\nnew(%lf, %lf, %lf, %lf, %lf, %lf)",
    //			m_compiler_status.cur_pos.x, m_compiler_status.cur_pos.y, m_compiler_status.cur_pos.z, m_compiler_status.cur_pos.a4,
    //			m_compiler_status.cur_pos.a5, m_compiler_status.cur_pos.a6, cur_pos.x, cur_pos.y, cur_pos.z, cur_pos.a4, cur_pos.a5, cur_pos.a6);
    //	this->m_compiler_status.cur_pos = cur_pos;

    int count = 0; 
    uint32_t mask = m_p_parser->GetPmcAxisMask();//PMC�������
    for(int i = 0; i < this->m_p_channel_config->chn_axis_count && count < 8; i++){
        if(mask & (0x01<<i))
            continue;   //����PMC��

        this->m_compiler_status.cur_pos.m_df_point[i] = cur_pos.GetAxisValue(count);
        count++;
    }
    return true;
}


/**
 * @brief ���ñ�������ǰ��λ�ã�ֻ���NC��
 * @param cur_pos : ��ǰλ��
 * @return true--�ɹ�   false--ʧ��
 */
bool Compiler::SetCurPos(const DPointChn &cur_pos) {
    //	if(this->m_n_thread_state != IDLE)
    //		return false;
    //	printf("Compiler::SetCurPos: old(%lf, %lf, %lf, %lf, %lf, %lf)\nnew(%lf, %lf, %lf, %lf, %lf, %lf)\n",
    //			m_compiler_status.cur_pos.m_df_point[0], m_compiler_status.cur_pos.m_df_point[1], m_compiler_status.cur_pos.m_df_point[2], m_compiler_status.cur_pos.m_df_point[3],
    //			m_compiler_status.cur_pos.m_df_point[4], m_compiler_status.cur_pos.m_df_point[5], cur_pos.m_df_point[0], cur_pos.m_df_point[1], cur_pos.m_df_point[2],
    //			cur_pos.m_df_point[3], cur_pos.m_df_point[4], cur_pos.m_df_point[5]);
    this->m_compiler_status.cur_pos = cur_pos;


    return true;
}

/**
 * @brief ���³�ʼ��Gģ̬
 * @return true--�ɹ�   false--ʧ��
 */
bool Compiler::SetCurGMode(){

    for(int i = 0; i < kMaxGModeCount; i++)
        this->m_compiler_status.mode.gmode[i] = this->m_p_channel_control->GetChnStatus().gmode[i];

    this->m_compiler_status.mode.h_mode = this->m_p_channel_control->GetChnStatus().cur_h_code;
    this->m_compiler_status.mode.d_mode = this->m_p_channel_control->GetChnStatus().cur_d_code;
    this->m_compiler_status.mode.t_mode = this->m_p_channel_control->GetChnStatus().cur_tool;

    return true;
}

/**
 * @brief ���������״̬
 * @return true--�ɹ�   false--ʧ��
 */
bool Compiler::SaveScene() {

    // @test zk
    printf("save scene !!!\n");
    CompilerScene scene;
    //	if(scene == nullptr){
    //		g_ptr_trace->PrintLog(LOG_ALARM, "���������״̬�������ڴ�ʧ�ܣ�");
    //		return false;
    //	}

    scene.file_map_info = *m_p_file_map_info;

    //	memcpy(&scene->file_map_info, m_p_file_map_info, sizeof(AsFileMapInfo));
    scene.n_sub_program = m_n_sub_program;
    scene.b_eof = m_b_eof;
    //	scene.thread_state = this->m_n_thread_state;
    scene.compiler_status = this->m_compiler_status;
    scene.file_state = this->m_n_compile_state;
    scene.head_state = this->m_n_head_state;
    scene.ln_cur_line_no = this->m_ln_cur_line_no;
    scene.ln_read_size = this->m_ln_read_size;
    scene.ptr_cur_file_pos = this->m_p_cur_file_pos;
    scene.work_mode = this->m_work_mode;
    scene.p_last_move_msg = this->m_p_last_move_msg;
    scene.n_sub_call_times = this->m_n_sub_call_times;
    //	scene.simulate_mode = this->m_simulate_mode;

    scene.list_label = *m_p_list_label;
    scene.list_subprog = *m_p_list_subprog;
    scene.list_loop = *m_p_list_loop;
    scene.stack_loop = m_stack_loop;

    /***************************************/
    scene.node_vectors_vector = m_node_vectors_vector;
    scene.node_stack_run = m_node_stack_run;
    scene.else_jump_stack_run = m_else_jump_stack_run;
    scene.else_jump = m_b_else_jump;
    /***************************************/

#ifdef USES_WOOD_MACHINE
    scene.list_spd_start = *m_p_list_spd_start;
    //	printf("Compiler::SaveScene(), spd_count=%d, scene=%d\n", m_p_list_spd_start->GetLength(), scene.list_spd_start.GetLength());
#endif

    if (!this->m_stack_scene.push(scene)) {
        g_ptr_trace->PrintLog(LOG_ALARM, "���������״̬ʧ�ܣ�");
        return false;
    }


    printf("save scene2, m_work_mode:%d, scene.work_mode = %d, scene.cur_file_pos=%u\n",
           m_work_mode, scene.work_mode, (uint32_t)scene.ptr_cur_file_pos);
    for(IfElseOffset node: scene.node_stack_run){
        printf("node line no: %llu\n", node.line_no);
    }
    return true;
}

/**
 * @brief �ָ�������״̬
 * @param bRecPos : �Ƿ�ָ���ǰλ�ã� true--�ָ�  false--���ָ�
 * @return true--�ɹ�   false--ʧ��
 */
bool Compiler::ReloadScene(bool bRecPos){
    CompilerScene scene;
    DPointChn pos_tmp;
    ModeCollect mode_tmp;

    // @test zk
    printf("reload scene !!!\n");

    if (this->m_stack_scene.size() == 0)
        return false;

    m_stack_scene.pop(scene);


    bool same_file = false;  //�Ƿ���ͬ�ļ�

    if (strcmp(m_p_file_map_info->str_file_name,
               scene.file_map_info.str_file_name) == 0) {
        //		printf("the same file\n");
        same_file = true;

    } else {
        m_p_file_map_info->CloseFile();
        *m_p_file_map_info = scene.file_map_info;
    }

    if(!bRecPos){
        pos_tmp = m_compiler_status.cur_pos;
    }

    //���������в���ģ̬���ûָ�
    // @modify zk  ���Եõ� M98 ����ΪSUB_PROG G65 P���̶�ѭ�� ����Ϊ MACRO_PROG
    if(this->m_n_sub_program == MACRO_PROG){
    	mode_tmp = this->m_compiler_status.mode;
    }


    scene.file_map_info.Clear();  //��ֹ����delete sceneʱ���ļ�ӳ��ر�

    //	memcpy(m_p_file_map_info, &scene.file_map_info, sizeof(AsFileMapInfo));
    m_n_sub_program = scene.n_sub_program;
    m_b_eof = scene.b_eof;
    //	this->m_n_thread_state = scene.thread_state;

    // Ҫʵ���ӳ������ �̶�ѭ������ģ̬ ע���������  ����֪���᲻��������������
    //if(this->m_n_sub_program != SUB_PROG) //�ӳ�����ò��ûָ�ģ̬
    this->m_compiler_status = scene.compiler_status;

    this->m_n_compile_state = scene.file_state;
    this->m_n_head_state = scene.head_state;
    this->m_ln_cur_line_no = scene.ln_cur_line_no;
    this->m_ln_read_size = scene.ln_read_size;
    this->m_p_cur_file_pos = scene.ptr_cur_file_pos;
    //	this->m_simulate_mode = scene.simulate_mode;
    this->m_work_mode = scene.work_mode;
    this->m_p_last_move_msg = scene.p_last_move_msg;
    this->m_n_sub_call_times = scene.n_sub_call_times;


    *m_p_list_label = scene.list_label;
    *m_p_list_subprog = scene.list_subprog;
    *m_p_list_loop = scene.list_loop;
    m_stack_loop = scene.stack_loop;

    /***********************************/
    m_node_vectors_vector = scene.node_vectors_vector;
    m_node_stack_run = scene.node_stack_run;
    m_else_jump_stack_run = scene.else_jump_stack_run;
    m_b_else_jump = scene.else_jump;
    /***********************************/


#ifdef USES_WOOD_MACHINE
    *m_p_list_spd_start = scene.list_spd_start;
    //	printf("reload compiler scene, spd_count=%d, scene=%d\n", m_p_list_spd_start->GetLength(), scene.list_spd_start.GetLength());
#endif

    if(!bRecPos){
        this->m_compiler_status.cur_pos = pos_tmp;
    }

    //���������в���ģ̬���ûָ�
    if(this->m_n_sub_program == MACRO_PROG){
        this->m_compiler_status.mode.d_mode = mode_tmp.d_mode;
        this->m_compiler_status.mode.h_mode = mode_tmp.h_mode;
        this->m_compiler_status.mode.t_mode = mode_tmp.t_mode;
    }

    if (same_file) {
        m_p_file_map_info->JumpTo(this->m_ln_read_size);
    }

    printf("reload scene, m_work_mode:%d, scene.m_work_mode = %d�� cur_line = %llu, m_p_cur_file_pos=%u, file=%s\n",
           m_work_mode, scene.work_mode, this->m_ln_cur_line_no, (uint32_t)this->m_p_cur_file_pos, m_p_file_map_info->str_file_name);

    for(IfElseOffset node: m_node_stack_run){
        printf("---------------------------------->node line no: %llu\n", node.line_no);
    }

    return true;
}

/**
 * @brief G������ɨ�������̺߳���
 * @param args
 */
void *Compiler::PreScanThread(void *args) {
    printf("Start Compiler::PreScanThread! id = %ld\n", syscall(SYS_gettid));
    Compiler *p_compiler = static_cast<Compiler *>(args);

    int res = ERR_NONE;
    res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
    if (res != ERR_NONE) {
        printf("Quit Compiler::PreScanThread with error 1!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }
    res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL); //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
    if (res != ERR_NONE) {
        printf("Quit Compiler::PreScanThread with error 2!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }

    p_compiler->PreScan();

    printf("Exit Compiler::PreScanThread!\n");
    pthread_exit(NULL);
}

/**
 * @brief ��ȡԤɨ���һ������
 * @param line[out] : ���ص�һ������
 * @param read_total ���Ѷ�ȡ�����ֽ���
 * @param map : �ļ�ӳ�����
 * @return
 */
int Compiler::GetPreScanLine(char *line, uint64_t &read_total,
                             AsFileMapInfo &map) {
    if (line == nullptr)
        return 0;
    memset(line, 0x00, kMaxLineSize);

    if (read_total == map.ln_file_size || map.ptr_map_file == (char *)MAP_FAILED)
        return 0;

    char *pcur = map.ptr_map_file + read_total - map.ln_map_start;		//��ǰָ��
    char c_cur = *pcur;     //��ǰ�ַ�
    char c_next = '\0';   //��һ���ַ�
    bool beof = false;     //�ĵ�����
    bool bSwapdown = false;   //�Ƿ����·�ҳ
    int index = 0;         //�����ֽ���
    uint64_t block_limit = map.ln_map_start + map.ln_map_blocksize;

REDO:
    //��ȡ��һ���ַ�
    if (read_total == block_limit - 1) {         //����ӳ���β��
        if (read_total == map.ln_file_size - 1) {  //�����ļ�β
            c_next = '\0';
        } else {
            if (map.Swapdown()) {
                bSwapdown = true;
                c_next = *map.ptr_map_file;
            } else {
                m_error_code = ERR_READ_FILE;
                return 0;
            }
        }
    } else
        c_next = *(pcur + 1);   //��һ���ַ�

    while (!beof && (c_cur != '\r' || c_next != '\n') &&	//�������ֻ��и�ʽ��\r\n��\n��
           (c_cur != '\n'))	//�ǻ��з�
    {

        if (index < kMaxLineSize - 1)	//���ܳ���һ������ַ���
        {
            line[index++] = toupper(c_cur);  //��ĸͳһ�ô�д

        } else {
            break;
        }
        read_total++;

        if (read_total == block_limit) { //���ﵱǰӳ����ĩβ
            if (read_total == map.ln_file_size) {  //�����ļ�β
                //	printf("file end ####\n");
                beof = true;
                break;
            } else if (bSwapdown) {
                pcur = map.ptr_map_file;
                block_limit = map.ln_map_start + map.ln_map_blocksize;
                bSwapdown = false;
            }

        } else
            pcur++;

        c_cur = *pcur;     //��ǰ�ַ�
        if (read_total == block_limit - 1) {     //����ӳ���β��
            if (read_total == map.ln_file_size - 1) {  //�����ļ�β
                c_next = '\0';
            } else {
                if (map.Swapdown()) {
                    bSwapdown = true;
                    c_next = *map.ptr_map_file;
                } else {
                    m_error_code = ERR_READ_FILE;
                    return 0;
                }
            }
        } else
            c_next = *(pcur + 1);   //��һ���ַ�
    }

    if (!beof) {
        if (c_cur == '\r') {
            read_total += 2;
            pcur += 2;
        } else if (c_cur == '\n') {
            read_total++;
            pcur++;
        } else if (index >= kMaxLineSize) {
            printf("too many characters in one line!!!!!\n");
        }

        if (index == 0) {
            c_cur = *pcur;     //��ǰ�ַ�
            goto REDO;
        }
    }

    return index;
}

/**
 * @brief ����Ԥɨ�裬�ҵ��ӳ���ͱ�ǩ��
 */
void Compiler::PreScan() {
    //	printf("start compiler PreScan, thread id = %ld\n", syscall(SYS_gettid));

    uint64_t total_size = 0;		//�ļ��ܴ�С
    uint64_t read_size = 0;    //�Ѷ�ȡ���ܴ�С
    //	uint64_t read_size_bak = 0;
    uint64_t line_no = 0;    //�к�
    char *line = nullptr;   //���д��뻺��
    //	char line[kMaxLineSize];   //���д��뻺��
    size_t len = 0;
    ssize_t read_block = 0;  	//���ζ�ȡ�Ĵ�С
    bool comment_flag = false;   //ע��״̬
    int count = 0;

    LoopOffsetStack loop_stack;  //ѭ�����ջ


    //���Ժ�ʱ
    struct timeval tvStart;
    struct timeval tvNow;
    unsigned int nTimeDelay = 0;
    unsigned int nTime1 = 0;
    gettimeofday(&tvStart, NULL);

#ifdef USES_WOOD_MACHINE
    this->m_n_s_code_in_prescan = 0;
#endif

    //��Ԥɨ���ļ�
    FILE *file = nullptr;
    CompilerScene *ptr_scene = nullptr;
    StackRec<CompilerScene> *scene_node = nullptr;
    if(this->m_b_prescan_in_stack){
    	scene_node = this->m_stack_scene.bottom();
        if(scene_node != nullptr){
            ptr_scene = &scene_node->rec;
            file = fopen(ptr_scene->file_map_info.str_file_name, "r+");
        }else{
            //TODO ����
            printf("error in Compiler::PreScan()\n");
            return;
        }

    }else{
        file = fopen(m_p_file_map_info->str_file_name, "r+");
    }
    if (nullptr == file) {
        g_ptr_trace->PrintLog(LOG_ALARM, "Ԥɨ���̴߳��ļ�ʧ��[%s]��errno = %d",
                              m_p_file_map_info->str_file_name, errno);
        CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
        goto END;
    }

    fseek(file, 0L, SEEK_END);
    total_size = ftell(file);   //��ȡ�ļ�����

    fseek(file, 0L, SEEK_SET);   //�ص��ļ�ͷ

    //ʹ���ڴ�ӳ�䷽ʽ
    //	AsFileMapInfo map;
    //	if(!map.OpenFile(m_p_file_map_info->str_file_name)){
    //		g_ptr_trace->PrintLog(LOG_ALARM, "Ԥɨ���̴߳��ļ�ʧ��[%s]��errno = %d", m_p_file_map_info->str_file_name, errno);
    //		CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
    //		goto END;
    //
    //	}
    //	total_size = map.ln_file_size;

    //��һ��ɨ�裬ʶ����ӳ���ţ�O****��,�Լ�GOTOָ��
    while ((read_block = getline(&line, &len, file)) != -1) {
        //	read_size_bak = read_size;
        //	while(this->GetPreScanLine(line, read_size, map) > 0){
        if (m_b_breakout_prescan) //�ж��˳�
            goto END;
        line_no++;
        this->PreScanLine1(line, read_size, line_no, comment_flag, loop_stack, ptr_scene);

        //	read_size_bak = read_size;
        read_size += read_block;

        if(count++ >= 10000){
            usleep(1000);
            count = 0;
        }
    }


    /*** test if else */

    printf("*********************************************\n");
    for(unsigned int i=0; i<m_node_vectors_vector.size(); i++){
        vector<IfElseOffset> node_vector = m_node_vectors_vector.at(i);
        printf("node number: %d\n", i+1);

        for(unsigned int j=0; j<node_vector.size(); j++){
            printf("lino: %lld --", node_vector.at(j).line_no);
        }
        printf("\n");
    }

    /*** test if else */

    if (total_size != read_size) { //û��������ȡ�ļ����澯
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]Ԥɨ���һ��ִ��ʧ�ܣ��ļ�δ����������%llu, %llu",
                              m_n_channel_index, total_size, read_size);
        CreateError(ERR_PRE_SCAN, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
        goto END;
    }

    gettimeofday(&tvNow, NULL);
    nTime1 = (tvNow.tv_sec - tvStart.tv_sec) * 1000000 + tvNow.tv_usec
            - tvStart.tv_usec;

    //�ڶ���ɨ�裬�ҵ�����GOTOָ����ת�кŵ�ƫ��
    if (this->m_p_list_label->GetLength() > 0) {
        fseek(file, 0L, SEEK_SET);   //�ص��ļ�ͷ
        read_size = 0;
        comment_flag = false;
        line_no = 0;
        count = 0;
        //	read_size_bak = read_size;
        //	map.ResetFile();
        while ((read_block = getline(&line, &len, file)) != -1) {
            //	while(this->GetPreScanLine(line, read_size, map) > 0){

            if (m_b_breakout_prescan)	//�ж��˳�
                goto END;

            line_no++;
            this->PreScanLine2(line, read_size, line_no, comment_flag, ptr_scene);

            //	read_size_bak = read_size;
            read_size += read_block;

            if(count++ >= 10000){
                usleep(1000);
                count = 0;
            }
        }

        if (total_size != read_size) {	//û��������ȡ�ļ����澯
            g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]Ԥɨ��ڶ���ִ��ʧ�ܣ��ļ�δ����������",
                                  m_n_channel_index);
            CreateError(ERR_PRE_SCAN, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                        m_n_channel_index);
            goto END;
        }
    }

    m_b_prescan_over = true;  //�ɹ�����

    gettimeofday(&tvNow, NULL);
    nTimeDelay = (tvNow.tv_sec - tvStart.tv_sec) * 1000000 + tvNow.tv_usec
            - tvStart.tv_usec;
    printf("prescan time : first = %u us | second = %u us\n", nTime1,
           nTimeDelay);

END: if (line != nullptr)
        free(line);   //�ͷ�getline��������Ļ���
    fclose(file);
    //	m_thread_prescan = 0;
    m_b_breakout_prescan = false;
    this->m_b_prescan_in_stack = false;
    this->compiler_lock = false;
    printf("exit compiler PreScan, thread id = %ld\n", syscall(SYS_gettid));
}

/**
 * @brief Ԥɨ���һ�飬���������ݣ�ʶ����ӳ���ţ�O****����GOTOָ��Լ����е�ѭ����
 *        ����ľ��ר������Ҫ��������������ָ��M03/M04��ת��ָ��S��
 * @param buf : �����ݻ���
 * @param line_no : �к�
 * @param flag : ����ע��״̬
 */
void Compiler::PreScanLine1(char *buf, uint64_t offset, uint64_t line_no,
                            bool &flag, LoopOffsetStack &loop_stack, CompilerScene *scene) {

    if (buf == nullptr)
        return;
    char *pc = buf;
    char digit_buf[kMaxDigitBufLen+1];
    bool jump_flag = false;    //�Ƿ�������η�
    bool first_alpha = true;   //�Ƿ��׸��ַ�
    bool sub_prog = false;   //�ӳ���ͷ
    bool goto_cmd = false;   //GOTOָ��
    bool loop_do = false;   //ѭ��ͷ
    bool loop_end = false;  //ѭ��β

    bool if_cmd = false;    //���� IF ָ��
    bool endif_cmd = false;
    bool elseif_cmd = false;
    bool else_cmd = false;

    int digit_count = 0;  //���ּ���
    ListNode < LabelOffset > *node = nullptr;

#ifdef USES_WOOD_MACHINE

    bool m_code = false;   //Mָ��
    bool s_code = false;   //Sָ��
    bool m_msg = false;    //�Ƿ����Mָ��ֵ��
    int m_digit = 0;       //Mָ��ֵ
    bool s_msg = false;
    int s_digit = 0;
#endif

    memset(digit_buf, 0, kMaxDigitBufLen+1);

    StrUpper(buf);

    //printf("lino: %llu -- %s", line_no, buf);

    while (*pc != '\0') {
        if (flag) {  //����ע��״̬
            if (*pc == ')') {  //����ע��״̬
                flag = false;
            }
            pc++;
            continue;
        }

        if (isspace(*pc)) { //�հ��ַ�
            if ((sub_prog || goto_cmd) && digit_count > 0) { //�ҵ��ӳ���ͷ����gotoָ��
                break;
            }else if(loop_do || loop_end){
                break;
            }
#ifdef USES_WOOD_MACHINE
            else if(m_code && digit_count > 0){   //����Mָ��
                m_digit = atoi(digit_buf);
                if(m_digit == 3 || m_digit == 4 || m_digit == 5){  //ֻ����M04��M03
                    m_msg = true;
                }
                m_code = false;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
            }else if(s_code && digit_count > 0){   //����Sָ��
                s_digit = atoi(digit_buf);
                s_msg = true;
                s_code = false;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
            }
#endif
            pc++;
            continue;
        }
        else if (*pc == '(') { //�������ע��
            flag = true;
            pc++;
            continue;
        } else if (*pc == m_c_end_line || (*pc == '/' && *(pc + 1) == '/')) { //����ע��
            break;
        } else if(*pc == '/' && first_alpha){  //���η�
            jump_flag = true;
        }else if (*pc == 'O' && first_alpha) { //�ӳ���ͷ
            if (!isalpha(*(pc + 1))){
                sub_prog = true;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
            }
        }else if(*pc == 'I' && *(pc + 1) == 'F' && first_alpha){
        	if_cmd = true;
            pc += 2;
            continue;
        }else if(*pc == 'E' && *(pc + 1) == 'N'&& *(pc + 2) == 'D'&& *(pc + 3) == 'I' && *(pc + 4) == 'F' && first_alpha){
            endif_cmd = true;
            pc += 5;
            continue;
        }else if(*pc == 'E' && *(pc + 1) == 'L'&& *(pc + 2) == 'S'&& *(pc + 3) == 'E' && *(pc + 4) == 'I'  && *(pc + 5) == 'F' && first_alpha){
            elseif_cmd = true;
            pc += 6;
            continue;
        }else if(*pc == 'E' && *(pc + 1) == 'L'&& *(pc + 2) == 'S'&& *(pc + 3) == 'E'  && first_alpha){
            else_cmd = true;
            pc += 4;
            continue;
        }
        else if (*pc == 'G' && *(pc + 1) == 'O' && *(pc + 2) == 'T' && *(pc + 3) == 'O' && !isalpha(*(pc+4))) { //gotoָ��

            if (first_alpha || pc == buf || !isalpha(*(pc - 1))) {
                goto_cmd = true;
                if_cmd = false;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
                pc += 4;
                continue;
            }

        } else if(*pc == 'D' && *(pc+1) == 'O' && !isalpha(*(pc+2))){//DOָ��
            //	printf("FIND DO CMD!!!!!%c\n", *(pc-1));
            if(first_alpha || pc == buf || !isalpha(*(pc - 1))){
                loop_do = true;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
                pc += 2;
                continue;
            }

        }else if(*pc == 'E' && *(pc+1) == 'N' && *(pc+2) == 'D' && !isalpha(*(pc+3))){//ENDָ��
            //	printf("FIND END CMD!!!!!%c\n", *(pc-1));
            if(first_alpha || pc == buf || !isalpha(*(pc - 1))){
                loop_end = true;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
                pc += 3;
                continue;
            }

        }
#ifdef USES_WOOD_MACHINE
        else if(*pc == 'M' && !isalpha(*(pc+1))){  //M03/M04ָ��
            if(s_code && digit_count > 0){//����Sָ��
                s_digit = atoi(digit_buf);
                s_msg = true;
                s_code = false;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
            }

            if(first_alpha || pc == buf || !isalpha(*(pc - 1))){
                m_code = true;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
                pc++;
                continue;
            }
        }else if(*pc == 'S' && !isalpha(*(pc+1))){  //Sָ��
            if(m_code && digit_count > 0){//����Mָ��
                m_digit = atoi(digit_buf);
                if(m_digit == 3 || m_digit == 4 || m_digit == 5){  //ֻ����M04��M03
                    m_msg = true;
                }
                m_code = false;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
            }

            if(first_alpha || pc == buf || !isalpha(*(pc - 1))){
                s_code = true;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
                pc++;
                continue;
            }
        }
#endif
        else if (isdigit(*pc)) { //����
            if (sub_prog) {
                digit_buf[digit_count++] = *pc;
                if (digit_count >= kMaxSubNameLen){ //�ӳ���ų���Χ
                    //TODO �澯���ӳ���ų���Χ�����ܳ���5λ

                    return;
                }
            } else if (goto_cmd) {
                digit_buf[digit_count++] = *pc;
                if (digit_count >= kMaxLineNoLen){ //˳��ų���Χ
                    //TODO �澯��˳��ų���Χ�����ܳ���8λ

                    return;
                }
            }else if(loop_do || loop_end){
                digit_buf[digit_count++] = *pc;
                if(digit_count > 1){//DOn��n�����Ǹ�λ�������ܳ���8
                    //TODO �澯��DO��ų���Χ

                    return;
                }
            }
#ifdef USES_WOOD_MACHINE
            else if(m_code){  //����Mָ��
                if(digit_count < kMaxMCodeLen)
                    digit_buf[digit_count++] = *pc;
            }else if(s_code){  //����Sָ��
                if(digit_count < kMaxSCodeLen)
                    digit_buf[digit_count++] = *pc;
            }
#endif
        } else if ((sub_prog || goto_cmd) && digit_count > 0) { //�ҵ��ӳ���ͷ����gotoָ��
            break;
        }else if(loop_do || loop_end){
            break;
        }
#ifdef USES_WOOD_MACHINE
        else if(m_code && digit_count > 0){
            m_digit = atoi(digit_buf);
            if(m_digit == 3 || m_digit == 4 || m_digit == 5){  //ֻ����M04��M03
                m_msg = true;
            }
            m_code = false;
            digit_count = 0;
            memset(digit_buf, 0, 10);
        }else if(s_code && digit_count > 0){
            s_digit = atoi(digit_buf);
            s_msg = true;
            s_code = false;
            digit_count = 0;
            memset(digit_buf, 0, 10);
        }
#endif

        if (first_alpha)
            first_alpha = false;
        pc++;
    }

    if (sub_prog && digit_count > 0) { //�ҵ��ӳ���ͷ
        SubProgOffset sub_offset;
        sub_offset.offset = offset;
        sub_offset.line_no = line_no;
        sub_offset.sub_index = atoi(digit_buf);
        //		printf("find sub program %d, line=%llu\n", sub_offset.sub_index, line_no);

        if(this->m_b_prescan_in_stack){   //��ջ�ļ�Ԥɨ��
            if (scene->list_subprog.HasData(sub_offset)) {
                //����ͬ���ӳ��򣬸澯
                g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]Ԥɨ���﷨������ͬ���ӳ����[%d]��",
                                      m_n_channel_index, sub_offset.sub_index);
                CreateError(ERR_SAME_SUB_INDEX, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                            sub_offset.sub_index, m_n_channel_index);
            } else
                scene->list_subprog.Append(sub_offset);
        }else{
            if (this->m_p_list_subprog->HasData(sub_offset)) {
                //����ͬ���ӳ��򣬸澯
                g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]Ԥɨ���﷨������ͬ���ӳ����[%d]��",
                                      m_n_channel_index, sub_offset.sub_index);
                CreateError(ERR_SAME_SUB_INDEX, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                            sub_offset.sub_index, m_n_channel_index);
            } else
                this->m_p_list_subprog->Append(sub_offset);
        }

    } else if (goto_cmd && digit_count > 0) { //�ҵ�gotoָ��
        LabelOffset label_offset;
        label_offset.offset = 0;   //offset�ڵڶ���ɨ��ʱȷ��
        label_offset.line_no = 0;
        label_offset.label = atoi(digit_buf);

        //	printf("find goto label %d\n", label_offset.label);

        if(this->m_b_prescan_in_stack){   //��ջ�ļ�Ԥɨ��
            if (!scene->list_label.HasData(label_offset)){
                //����������
                node = scene->list_label.HeadNode();
                while (node != nullptr) {
                    if (node->data.label > label_offset.label) {
                        scene->list_label.InsertBefore(label_offset, node);
                        break;
                    }
                    node = node->next;
                }
                if (node == nullptr) {
                    scene->list_label.Append(label_offset);
                }
            }
        }else{
            if (!m_p_list_label->HasData(label_offset)){
                //����������
                node = m_p_list_label->HeadNode();
                while (node != nullptr) {
                    if (node->data.label > label_offset.label) {
                        m_p_list_label->InsertBefore(label_offset, node);
                        break;
                    }
                    node = node->next;
                }
                if (node == nullptr) {
                    m_p_list_label->Append(label_offset);
                }
            }
        }

    }else if(loop_do){	//�ҵ�DOָ��
        LoopOffset loop_offset;
        loop_offset.line_no = line_no;
        loop_offset.offset = offset;

        if(digit_count > 0)
            loop_offset.loop_index = atoi(digit_buf);
        else
            loop_offset.loop_index = 0;

        loop_stack.push(loop_offset);
        //	printf("push do cmd: %lld\n", line_no);
    }else if(loop_end){		//�ҵ�ENDָ��
        LoopRec loop_rec;
        LoopOffset loop_offset;
        //	printf("pop end: %lld, loop_stack: %d\n", line_no, loop_stack.size());
        if(loop_stack.size()>0){  //���ж�ӦDOָ��
            loop_stack.pop(loop_offset);
            loop_rec.start_line_no = loop_offset.line_no;
            loop_rec.start_offset = loop_offset.offset;
            loop_rec.end_line_no = line_no;
            loop_rec.end_offset = offset;
            if(this->m_b_prescan_in_stack){   //��ջ�ļ�Ԥɨ��
                scene->list_loop.Append(loop_rec);
            }else{
                this->m_p_list_loop->Append(loop_rec);
            }

            //		printf("find loop rec: s[%lld, %lld] e[%lld, %lld]\n", loop_rec.start_line_no, loop_rec.start_offset, line_no, offset);
        }
    }
#ifdef USES_WOOD_MACHINE
    else if(m_code && digit_count > 0){
        m_digit = atoi(digit_buf);
        if(m_digit == 3 || m_digit == 4){  //ֻ����M04��M03
            m_msg = true;
        }
        m_code = false;
        digit_count = 0;
        memset(digit_buf, 0, 10);
    }else if(s_code && digit_count > 0){
        s_digit = atoi(digit_buf);
        s_msg = true;
        s_code = false;
        digit_count = 0;
        memset(digit_buf, 0, 10);
    }
    //����ת��ָ��
    if(s_msg){
        this->m_n_s_code_in_prescan = s_digit;
    }

    //��������ָ��
    if(m_msg){
        if(m_digit == 5)   //����ͣ
            this->m_n_s_code_in_prescan = 0;
        else if(m_digit == 3 || m_digit ==4){
            SpindleStartOffset spd_cmd;
            spd_cmd.line_no = line_no;
            spd_cmd.m_code = m_digit;
            spd_cmd.s_code = m_n_s_code_in_prescan;
            spd_cmd.jump_line = jump_flag;
            //	spd_cmd.exec_step = 0;
            printf("find spindle cmd:%d, spd:%d, line:%llu, jump=%hhu\n", m_digit, m_n_s_code_in_prescan, line_no, spd_cmd.jump_line);

            if(this->m_b_prescan_in_stack){   //��ջ�ļ�Ԥɨ��
                scene->list_spd_start.Append(spd_cmd);
                printf("append in scene\n");
            }else{
                this->m_p_list_spd_start->Append(spd_cmd);
                printf("append in body\n");
            }
        }
    }
#endif

    // �ҵ� if ָ��
    if(if_cmd){
    	// ��ʼ����������
        m_else_count_prescan = 0;
        m_node_vector_index = m_node_vector_len;
        m_node_vector_len ++;
        m_stack_vector_index_prescan.push_back(m_node_vector_index);
        m_stack_else_count_prescan.push_back(m_else_count_prescan);

        // �½��ڵ�
        IfElseOffset node;
        node.offset = offset;
        node.line_no = line_no;
        node.vec_index = m_node_vector_index;
        node.node_index = 0;
        // �½��ڵ�����
        vector<IfElseOffset> node_vector;
        node_vector.push_back(node);
        //�ڵ�����װ��������
        if(this->m_b_prescan_in_stack){
            scene->node_vectors_vector.push_back(node_vector);
        }
        else{
            m_node_vectors_vector.push_back(node_vector);
        }

        if_cmd = false;

    }else if(endif_cmd){

        if(m_stack_vector_index_prescan.size() == 0){
            g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]Ԥɨ���﷨����  ���� IF ELSEIF ELSE ENDIF �﷨���1������\n",
                                  m_n_channel_index);

            CreateError(IF_ELSE_MATCH_FAILED, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                        line_no, m_n_channel_index);
            return;
        }

        // �½��ڵ�
        IfElseOffset node;
        node.offset = offset;
        node.line_no = line_no;
        node.vec_index = m_node_vector_index;

        //�ڵ�װ������
        if(this->m_b_prescan_in_stack){
            node.node_index = scene->node_vectors_vector.at(m_node_vector_index).size();
            scene->node_vectors_vector.at(m_node_vector_index).push_back(node);
        }
        else{
            node.node_index = m_node_vectors_vector.at(m_node_vector_index).size();
            m_node_vectors_vector.at(m_node_vector_index).push_back(node);
        }

        // ����IF �������  ��ջ
        m_stack_vector_index_prescan.pop_back();
        m_stack_else_count_prescan.pop_back();
        // ���� IF Ƕ��
        if(m_stack_vector_index_prescan.size() != 0){
            m_node_vector_index = m_stack_vector_index_prescan.back();
            m_else_count_prescan = m_stack_else_count_prescan.back();
        }

        endif_cmd = false;

    }else if(else_cmd or elseif_cmd){

        if(m_stack_vector_index_prescan.size() == 0){
            g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]Ԥɨ���﷨����  ���� IF ELSEIF ELSE ENDIF �﷨���2������\n",
                                  m_n_channel_index);

            CreateError(IF_ELSE_MATCH_FAILED, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                        line_no, m_n_channel_index);
            return;
        }

        if(else_cmd){
            ++ m_else_count_prescan;
            if(m_else_count_prescan > 1){
                g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]Ԥɨ���﷨����  ���� IF ELSEIF ELSE ENDIF �﷨���3������\n",
                                      m_n_channel_index);
                CreateError(IF_ELSE_MATCH_FAILED, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                            line_no, m_n_channel_index);
            }
        }

        IfElseOffset node;
        node.offset = offset;
        node.line_no = line_no;
        node.vec_index = m_node_vector_index;

        if(this->m_b_prescan_in_stack){
            node.node_index = scene->node_vectors_vector.at(m_node_vector_index).size();
            scene->node_vectors_vector.at(m_node_vector_index).push_back(node);
        }else{
            node.node_index = m_node_vectors_vector.at(m_node_vector_index).size();
            m_node_vectors_vector.at(m_node_vector_index).push_back(node);
        }

        else_cmd = false;
        elseif_cmd = false;
    }


}

/**
 * @brief Ԥɨ��ڶ��飬���������ݣ��ҵ�����GOTOָ����ת�кŵ�ƫ��
 * @param buf : �����ݻ���
 * @param offset : ��ǰ�е�ƫ��λ��
 * @param line_no : �к�
 * @param flag : ����ע��״̬
 */
void Compiler::PreScanLine2(char *buf, uint64_t offset, uint64_t line_no,
                            bool &flag, CompilerScene *scene) {
    //Ԥɨ��ڶ��飬����������

    if (buf == nullptr)
        return;
    char *pc = buf;
    char digit_buf[10];
    bool line_serial = false;   //˳���
    int digit_count = 0;  //���ּ���
    memset(digit_buf, 0, 10);

    StrUpper(buf);
    while (*pc != '\0') {
        if (flag) {  //����ע��״̬
            if (*pc == ')') {  //����ע��״̬
                flag = false;
            }
            pc++;
            continue;
        }

        if (isspace(*pc)) { //�հ��ַ�
            if (line_serial && digit_count > 0) { //�ҵ�˳���
                break;
            }
            pc++;
            continue;
        }

        if (strchr("XYZMABCDHIJKTUVW", *pc) != nullptr)   //�ӿ�ɨ���ٶ�
            break;
        else if (*pc == '(') { //�������ע��
            flag = true;
            pc++;
            continue;
        } else if (*pc == m_c_end_line || (*pc == '/' && *(pc + 1) == '/')) { //����ע��
            break; //ֱ�ӷ���
        } else if (*pc == 'N') { //˳���
            if (!isalpha(*(pc + 1)))
                line_serial = true;
        } else if (*pc == m_c_end_line) { //�н�����
            if (line_serial && digit_count > 0) { //�ҵ�˳���
                break;
            }
        } else if (isdigit(*pc)) { //����
            if (line_serial) {
                if (digit_count >= kMaxLineNoLen) //˳��ų���Χ
                    return;
                digit_buf[digit_count++] = *pc;
            }
        } else if (line_serial && digit_count > 0) { //�ҵ�˳���
            break;
        }

        if (!line_serial)
            return;

        pc++;
    }

    if (line_serial) {
        int label = atoi(digit_buf);
        ListNode < LabelOffset > *node = nullptr;
        if(this->m_b_prescan_in_stack){   //��ջ�ļ�Ԥɨ��
            node = scene->list_label.HeadNode();
        }else{
            node = this->m_p_list_label->HeadNode();
        }

        while (node != nullptr) {
            if (node->data.label == label) {
                if (node->data.offset > 0) { //TODO �����ظ���˳���  �澯
                    g_ptr_trace->PrintLog(LOG_ALARM,
                                          "CHN[%d]Ԥɨ���﷨������ͬ��˳���[%d]��", m_n_channel_index,
                                          label);
                    CreateError(ERR_SAME_SEQ_NO, ERROR_LEVEL,
                                CLEAR_BY_MCP_RESET, label, m_n_channel_index);
                }
                node->data.offset = offset;
                node->data.line_no = line_no;
                //printf("pre scan 2, find label %d , offset %llu\n", label, offset);
                break;
            } else if (label < node->data.label)
                break;

            node = node->next;
        }
    }
}

/**
 * @brief �򿪴������ļ�
 * @param file : �������ļ�
 * @param sub_flag : �Ƿ��ӳ����ļ�
 */
bool Compiler::OpenFile(const char *file, bool sub_flag) {
    //	bool res = false;
    //	if(this->m_work_mode == MDA_COMPILER){  //MDAģʽ������������ʱ�ٴ��ļ�
    //		res = m_p_file_map_info->OpenFile(m_str_mda_path);
    //	}
    //	else if(m_work_mode == AUTO_COMPILER){
    //		res = m_p_file_map_info->OpenFile(file);
    //	}
    // printf("compiler::openfile, file[%s], sub_flag= %hhu\n", file, sub_flag);
    int res = 0;

    // @add zk ���ع����ļ������±���
    if(strlen(file) > kMaxFileNameLen-1){
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�ļ�������[%s]���ļ�ʧ��!",
                              m_n_channel_index, file);
        CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
        return false;
    }
    // @add zk

    if(this->m_work_mode == MDA_COMPILER && !sub_flag){  //
        char tmp_file[kMaxPathLen] = {0};	//�ļ�·��
        this->m_p_channel_control->GetMdaFilePath(tmp_file);
        if(strcmp(file, tmp_file) != 0){
            printf("error in Compiler::OpenFile, can't open file in MDA mode!file[%s]\n", file);
            return false;
        }
    }

    if (!m_p_file_map_info->OpenFile(file, sub_flag)) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]���������ļ�[%s]ʧ��!",
                              m_n_channel_index, file);
        CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
        return false;
    }

    m_p_cur_file_pos = nullptr;
    this->m_ln_read_size = 0;
    this->m_ln_cur_line_no = 1;
    this->m_b_eof = false;

    void* thread_result;
    //����Ԥɨ���߳�
    if (!m_b_prescan_over && m_thread_prescan != 0){
        //���˳�֮ǰ���߳�
        m_b_breakout_prescan = true;
        int wait_count = 0;

        while (m_b_breakout_prescan && wait_count++ < 200)
            usleep(1000);  //�ȴ��߳��˳�
        if (m_b_breakout_prescan) {//�ȴ��˳�ʧ�ܣ�������cancel�߳�

            //�˳�Ԥɨ�������߳�
            res = pthread_cancel(m_thread_prescan);
            if (res != 0) {
                g_ptr_trace->PrintLog(LOG_ALARM, "Ԥɨ���߳��˳�ʧ��1��errno = %d\n",
                                      res);
                printf("Ԥɨ���߳��˳�ʧ��1111\n");
                CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                            0, m_n_channel_index);
                return false;
            }

            usleep(1000);
        }
        res = pthread_join(m_thread_prescan, &thread_result);  //�ȴ��߳��˳����
        if (res != 0) {
            g_ptr_trace->PrintLog(LOG_ALARM, "Ԥɨ���߳��˳�ʧ��2��errno = %d\n",
                                  res);
            printf("Ԥɨ���߳��˳�ʧ��2222\n");
            CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                        0, m_n_channel_index);
            return false;
        }
        m_thread_prescan = 0;

    }else if(m_b_prescan_over && m_thread_prescan != 0){  //�ͷ�Ԥɨ���߳���Դ
        res = pthread_join(m_thread_prescan, &thread_result);  //�ȴ��߳��˳����
        if (res != 0) {
            g_ptr_trace->PrintLog(LOG_ALARM, "Ԥɨ���߳���Դ�ͷ�ʧ�ܣ�errno = %d\n",
                                  res);
            printf("Ԥɨ���߳��˳�ʧ��3333\n");
            CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                        0, m_n_channel_index);
        }
        m_thread_prescan = 0;
    }
    m_b_breakout_prescan = false;
    m_b_prescan_over = false;
    m_b_prescan_in_stack = false;
    this->m_p_list_label->Clear();
    this->m_p_list_subprog->Clear();
    this->m_p_list_loop->Clear();
    m_stack_loop.empty();  //���ѭ��λ������

    m_node_vector_index = 0;
    m_node_vector_len = 0;
    m_else_count_prescan = 0;
    m_node_vectors_vector.clear();
    m_stack_vector_index_prescan.clear();
    m_stack_else_count_prescan.clear();


#ifdef USES_WOOD_MACHINE
    m_p_list_spd_start->Clear();
#endif

    pthread_attr_t attr;
    struct sched_param param;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//
    if (sub_flag)
        param.__sched_priority = 36;	//96; //�ӳ���Ԥɨ���̱߳ȱ����߳����ȼ���һ��
    else
        param.__sched_priority = 35; //95; //������Ԥɨ���̱߳ȱ����߳����ȼ�һ����Ϊ��������ܱȽϴ�
    pthread_attr_setschedparam(&attr, &param);

    /* Use scheduling parameters of attr */
    //	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
    //	if (res) {
    //		printf("pthread setinheritsched failed\n");
    //	}
    res = pthread_create(&m_thread_prescan, &attr, Compiler::PreScanThread, this);    //����G����Ԥ�����߳�
    if (res != 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]������Ԥɨ���̴߳���ʧ��! res = %d, errno = %d, errstr=%s",
                              m_n_channel_index, res, errno, strerror(errno));
        m_error_code = ERR_PRE_SCAN;
        CreateError(ERR_PRE_SCAN, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
    }

    pthread_attr_destroy(&attr);

    printf("exit openfile priority %d\n", param.__sched_priority);

    return true;
}

/**
 * @brief �ڱ���ĳ����д��ļ��������ڷ��Զ�ģʽ�£����ؼӹ��ļ�
 * @param file �� ���ص����ļ�·��
 * @return
 */
bool Compiler::OpenFileInScene(const char *file){
    printf("compiler::OpenFileInScene, file[%s]\n", file);
    int res = 0;

    CompilerScene scene;

    if (this->m_stack_scene.size() == 0)
        return false;
    printf("stack size = %d\n", this->m_stack_scene.size());
    m_stack_scene.pop(scene);

    scene.file_map_info.CloseFile();

    if (!scene.file_map_info.OpenFile(file)) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�������򿪳���ջ�ļ�[%s]ʧ��!",
                              m_n_channel_index, file);
        CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
        return false;
    }

    scene.ptr_cur_file_pos = nullptr;
    scene.ln_cur_line_no = 1;
    scene.ln_read_size = 0;
    scene.b_eof = false;
    scene.list_label.Clear();
    scene.list_subprog.Clear();
    scene.list_loop.Clear();

#ifdef USES_WOOD_MACHINE
    scene.list_spd_start.Clear();
#endif

    //	printf("OpenFileInScene: map_info.ptr=%u, , ")

    this->m_stack_scene.push(scene);   //������ջ

    scene.file_map_info.Clear();//��ֹ����delete sceneʱ���ļ�ӳ��ر�

    void* thread_result;
    //����Ԥɨ���߳�
    if (!m_b_prescan_over && m_thread_prescan != 0) {
        //���˳�֮ǰ���߳�
        m_b_breakout_prescan = true;
        int wait_count = 0;

        while (m_b_breakout_prescan && wait_count++ < 200)
            usleep(1000);  //�ȴ��߳��˳�
        if (m_b_breakout_prescan) {//�ȴ��˳�ʧ�ܣ�������cancel�߳�

            //�˳�Ԥɨ�������߳�
            res = pthread_cancel(m_thread_prescan);
            if (res != 0) {
                g_ptr_trace->PrintLog(LOG_ALARM, "Ԥɨ���߳��˳�ʧ��1��errno = %d\n",
                                      res);
                printf("Ԥɨ���߳��˳�ʧ��4444\n");
                CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                            0, m_n_channel_index);
                return false;
            }

            usleep(1000);
        }
        res = pthread_join(m_thread_prescan, &thread_result);  //�ȴ��߳��˳����
        if (res != 0) {
            g_ptr_trace->PrintLog(LOG_ALARM, "Ԥɨ���߳��˳�ʧ��2��errno = %d\n",
                                  res);
            printf("Ԥɨ���߳��˳�ʧ��5555\n");
            CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                        0, m_n_channel_index);
            return false;
        }
        m_thread_prescan = 0;

    }else if(m_b_prescan_over && m_thread_prescan != 0){  //�ͷ�Ԥɨ���߳���Դ
        res = pthread_join(m_thread_prescan, &thread_result);  //�ȴ��߳��˳����
        if (res != 0) {
            g_ptr_trace->PrintLog(LOG_ALARM, "Ԥɨ���߳���Դ�ͷ�ʧ�ܣ�errno = %d\n",
                                  res);
            printf("Ԥɨ���߳��˳�ʧ��6666\n");
            CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                        0, m_n_channel_index);
        }
        m_thread_prescan = 0;
    }
    m_b_breakout_prescan = false;
    m_b_prescan_over = false;
    m_b_prescan_in_stack = true;
    //	this->m_p_list_label->Clear();
    //	this->m_p_list_subprog->Clear();
    //	this->m_p_list_loop->Clear();

    m_stack_loop.empty();  //���ѭ��λ������

    pthread_attr_t attr;
    struct sched_param param;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//

    param.__sched_priority = 35; //95; //������Ԥɨ���̱߳ȱ����߳����ȼ�һ����Ϊ��������ܱȽϴ�
    pthread_attr_setschedparam(&attr, &param);

    /* Use scheduling parameters of attr */
    //	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
    //	if (res) {
    //		printf("pthread setinheritsched failed\n");
    //	}
    res = pthread_create(&m_thread_prescan, &attr, Compiler::PreScanThread, this);    //����G����Ԥ�����߳�
    if (res != 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]������Ԥɨ���̴߳���ʧ��! res = %d, errno = %d, errstr=%s",
                              m_n_channel_index, res, errno, strerror(errno));
        m_error_code = ERR_PRE_SCAN;
        CreateError(ERR_PRE_SCAN, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
    }

    pthread_attr_destroy(&attr);

    printf("exit OpenFileInScene priority %d\n", param.__sched_priority);

    return true;
}

/**
 * @brief ������������ش�������M30/M02/M99��Щ����ָ��ʱ����
 */
bool Compiler::CompileOver() {
    printf("enter compiler::compileover, m_work_mode = %d\n", m_work_mode);
    if (this->m_n_sub_program != MAIN_PROG) {
        //�ӳ���
        g_ptr_trace->PrintTrace(TRACE_ERROR, COMPILER_CHN,
                                "���ô����ӳ������Compiler::CompileOver()������");
        //TODO ����
        return false;
    }
    //���������������
    if (m_work_mode == AUTO_COMPILER && m_stack_scene.size() > 0) { //�Զ�ģʽ�£���������������ö�ջӦ�ÿ�
        g_ptr_trace->PrintLog(LOG_ALARM, "������������ӳ�����ö�ջ�ǿգ�");
        CreateError(ERR_MAIN_OVER, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
        return false;
    }else if (m_work_mode == MDA_COMPILER && m_stack_scene.size() == 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "MDAģʽ�£�������ö�ջΪ�գ�");
        CreateError(ERR_MDA_OVER, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
        return false;
    }

    m_p_file_map_info->ResetFile();
    this->m_compiler_status.mode.Reset();
    if (m_work_mode == AUTO_COMPILER)
        this->m_n_compile_state = FILE_HEAD;
    else
        m_n_compile_state = FILE_MAIN;
    this->m_n_head_state = HEAD_INFO;
    this->m_ln_cur_line_no = 1;
    this->m_ln_read_size = 0;
    this->m_p_cur_file_pos = nullptr;
    this->m_p_last_move_msg = nullptr;

    m_b_eof = false;
    m_b_comment = false;
    m_b_left = false;
    m_b_compile_over = false;

    m_b_else_jump = false;
    m_node_stack_run.clear();
    m_else_jump_stack_run.clear();

    printf("exit compiler::compileover\n");
    return true;
}

/**
 * @brief ѭ�����룬����M99ָ��
 */
void Compiler::RecycleCompile() {
    printf("compiler::RecycleCompile\n");
    //	if (m_n_sub_program != MAIN_PROG) {  //�ӳ������ѭ������
    //		printf("sub prog return\n");
    //		return;
    //	}
    this->m_p_file_map_info->ResetFile();  //�ļ���λ��ͷ��
    this->m_compiler_status.mode.Reset();
    if (m_work_mode == AUTO_COMPILER && m_n_sub_program == MAIN_PROG)
        this->m_n_compile_state = FILE_HEAD;
    else
        m_n_compile_state = FILE_MAIN;
    this->m_n_head_state = HEAD_INFO;
    this->m_ln_cur_line_no = 1;
    this->m_ln_read_size = 0;
    this->m_p_cur_file_pos = nullptr;
    this->m_p_last_move_msg = nullptr;
    m_b_eof = false;
    m_b_comment = false;
    m_b_left = false;
    m_b_compile_over = false;

}

/**
 * @brief ���ñ��빤��ģʽ
 * @param mode ������ģʽ
 */
void Compiler::SetMode(CompilerWorkMode mode) {

    printf("compiler::SetMode, old mode = %d, new mode = %d\n", m_work_mode, mode);
    if (m_work_mode == mode)
        return;

    //TODO ��Ӻ���������,AUTO��MDA֮����Ҫ��buffer���л����Լ�״̬�л�
    if (m_work_mode == AUTO_COMPILER) {  //AUTO�л���MDA

        //		if(m_n_thread_state == RUN){//�����������RUN״̬��������ͣ
        //			this->PauseCompile();
        //		}
        //		pthread_mutex_lock(&m_mutex_change_state);

        //����״̬
        this->SaveScene();

        //��λ������״̬
        this->ResetState();

        this->m_p_file_map_info->Clear();  //����Զ�ģʽ�ļ������ݣ���ֹ��mda�ļ�ʱ�ر��Զ�ģʽ�µ��ļ�

        this->m_n_compile_state = FILE_MAIN;

        this->m_p_block_msg_list = this->m_p_block_msg_list_mda;
        this->m_p_tool_compensate = this->m_p_tool_compensate_mda;
        this->m_p_parser_result = this->m_p_parser_res_mda;

        //	pthread_mutex_unlock(&m_mutex_change_state);
    } else if (m_work_mode == MDA_COMPILER) {  //MDA�л���AUTO
        //		if(m_n_thread_state == RUN){//�����������RUN״̬������ֹͣ
        //			this->StopCompile();
        //		}
        char tmp_file[kMaxPathLen] = {0};	//�ļ�·��
        this->m_p_channel_control->GetMdaFilePath(tmp_file);
        if(strcmp(this->m_p_file_map_info->str_file_name, tmp_file) != 0){  //
            //			printf("Compiler::SetMode, clear file:%s\n", this->m_p_file_map_info->str_file_name);
            this->m_p_file_map_info->Clear();
        }
        this->ReloadScene();  //�ָ��ֳ�

        this->m_p_block_msg_list = this->m_p_block_msg_list_auto;
        this->m_p_tool_compensate = this->m_p_tool_compensate_auto;
        this->m_p_parser_result = this->m_p_parser_res_auto;

    }

    this->m_p_parser->SetParserResList(m_p_parser_result);

    //�����µ�ģʽ
    m_work_mode = mode;

}

/**
 * @brief ���������帴λ��ֹͣ���룬�ָ���ʼ״̬
 */
void Compiler::Reset(){

    printf("compiler reset !!!\n");

    CompilerScene scene;

    this->m_b_breakout_prescan = true;

    if (m_work_mode == MDA_COMPILER) {
        printf("compiler reset: MDA_COMPILER\n");
        while (m_stack_scene.size() > 0) {
            this->m_stack_scene.cur(scene);
            if (scene.work_mode == MDA_COMPILER && scene.n_sub_program != MAIN_PROG) { //��ջ���ӳ���״ֱ̬��ɾ��
                m_stack_scene.pop(scene);
                scene.file_map_info.CloseFile();
            }else if (scene.work_mode == MDA_COMPILER) {
                this->ReloadScene();  //�ָ����������ֳ�
            }else if(scene.work_mode == AUTO_COMPILER && scene.n_sub_program != MAIN_PROG){ //�����Զ�ģʽ���ݣ�˵��MDA��û�е����ӳ���
                m_stack_scene.pop(scene);
                scene.file_map_info.CloseFile();
            }else if(scene.work_mode == AUTO_COMPILER){  //�Զ�ģʽ������״̬����λ
                m_stack_scene.pop(scene);
                scene.b_eof = false;
                scene.file_map_info.ResetFile();   //��λ���ļ�ͷ
                scene.compiler_status.exact_stop_flag = false;
                scene.compiler_status.jump_flag = false;
                scene.compiler_status.mode.Reset();
                scene.file_state = FILE_HEAD;
                scene.head_state = HEAD_INFO;

                scene.ln_cur_line_no = 1;
                scene.ln_read_size = 0;
                scene.n_sub_call_times = 0;
                scene.p_last_move_msg = nullptr;
                scene.ptr_cur_file_pos = nullptr;
                scene.stack_loop.empty();

                m_stack_scene.push(scene);
                printf("reset compiler stack:%d\n", m_stack_scene.size());
                break;
            }

        }
        this->m_p_tool_compensate_mda->Reset();
        this->m_n_compile_state = FILE_MAIN;

    } else if (m_work_mode == AUTO_COMPILER) {
        printf("compiler reset: AUTO_COMPILER\n");
        while (m_stack_scene.size() > 0) {
            this->m_stack_scene.cur(scene);
            if (scene.n_sub_program != MAIN_PROG) { //�ӳ��򳡾�ֱ��ɾ��
                m_stack_scene.pop();

            } else {

                this->ReloadScene();   //�ָ����������ֳ�
            }
        }

        printf("scene reset finished!!!\n");
        this->m_p_tool_compensate_auto->Reset();
        this->m_n_compile_state = FILE_HEAD;
    }

    this->m_p_lexer->Reset();
    this->m_p_parser->Reset();

    m_p_file_map_info->ResetFile();
    this->m_compiler_status.exact_stop_flag = false;
    this->m_compiler_status.jump_flag = false;
    this->m_compiler_status.mode.Reset();
    this->m_n_head_state = HEAD_INFO;
    this->m_ln_cur_line_no = 1;
    this->m_ln_read_size = 0;
    this->m_p_cur_file_pos = nullptr;
    this->m_p_last_move_msg = nullptr;
    m_b_eof = false;
    m_b_comment = false;
    m_b_left = false;
    m_b_has_over_code = false;
    m_b_compile_over = false;
    this->m_n_restart_line = 0;
    this->m_n_restart_mode = NOT_RESTART;

    m_error_code = ERR_NONE;

    this->m_p_parser_res_auto->Clear();
    this->m_p_parser_res_mda->Clear();
    this->m_lexer_result.Reset();
    m_p_block_msg_list_auto->Clear();
    m_p_block_msg_list_mda->Clear();
    this->m_p_tool_compensate_auto->ResetAllDatas();
    this->m_p_tool_compensate_mda->ResetAllDatas();

    m_stack_loop.empty();  //���ѭ��λ������

    /********************************/

    // ��λ����ʱ��¼
    m_node_stack_run.clear();
    m_else_jump_stack_run.clear();
    m_b_else_jump = false;
    /********************************/
}

/**
 * @brief ������״̬��λ
 */
void Compiler::ResetState() {
    //m_p_file_map_info->Clear();
    m_p_file_map_info->ResetFile();
    m_n_sub_program = MAIN_PROG;
    //	this->m_n_thread_state = IDLE;
    this->m_compiler_status.mode.Reset();
    this->m_compiler_status.exact_stop_flag = false;
    this->m_compiler_status.jump_flag = false;

    if (m_work_mode == AUTO_COMPILER)
        this->m_n_compile_state = FILE_HEAD;
    else
        this->m_n_compile_state = FILE_MAIN;
    this->m_n_head_state = HEAD_INFO;
    this->m_ln_cur_line_no = 1;
    this->m_ln_read_size = 0;
    this->m_p_cur_file_pos = nullptr;
    //	this->m_simulate_mode = SIM_NONE;
    m_b_eof = false;
    m_b_comment = false;
    m_b_left = false;
    m_b_has_over_code = false;
    m_b_compile_over = false;
    m_error_code = ERR_NONE;
}

/**
 * @brief ���д�����
 */
void Compiler::DoIdle(){

}

/**
 *@brief �����ļ�ӳ�����
 *@param AsFileMapInfo *pFile: �ļ�ӳ�����ָ��
 */
/*void Compiler::SetFileMap(AsFileMapInfo *pFile){
 m_p_file_map_info = pFile;
 //m_p_cur_pos = reinterpret_cast<char *>(m_p_file_map_info->ptr_map_file);
 }*/

/**
 * @brief �ж��Ƿ��з�
 * @param char *pc:���ж��ַ�
 * @return �ǻ��з��򷵻�true�����򷵻�false
 */
bool Compiler::IsNewLineChar(char *pc) {
    if ((*pc == '\r' && *(pc + 1) == '\n') || *pc == '\n')  //�������ֻ��и�ʽ"\r\n"��\n
        return true;
    return false;
}

/**
 * @brief ��ȡ��ǰ�ӹ���NC�ļ�����
 * @param file[out] : �����ǰnc�ļ�����
 */
void Compiler::GetCurNcFile(char *file){
    if(file == nullptr)
        return;

    strcpy(file, m_p_file_map_info->str_file_name);
}

//int total_time = 0;

/**
 * @brief ��ȡһ��NC���룬�ŵ���ǰ�����л�������,�˺������ص��ַ����Ѿ�ȥ���˶���Ŀհ��ַ��Լ�ע��
 * @return ִ�н��,�ɹ�����true��ʧ�ܷ���false
 */
bool Compiler::GetLineData() {
    //	struct timeval tvStart;
    //	struct timeval tvNow;
    //	unsigned int nTimeDelay = 0;
    //
    //	gettimeofday(&tvStart, NULL);

    //	printf("enter getlinedata\n");

    bool res = true;
    if(m_p_file_map_info == nullptr || m_b_compile_over) {  //�������
        //	printf("getline return, 111, %d, %d\n", (int)m_p_file_map_info, m_b_compile_over);
        return false;
    }


    this->m_p_lexer->Reset(); //�ʷ���������λ
    memset(m_line_buf, 0x00, static_cast<size_t>(kMaxLineSize));

    if (m_b_eof || m_p_file_map_info->ln_file_size == 0) {
        m_b_eof = true;
        if(this->m_work_mode == MDA_COMPILER){  //MDAģʽ�£����ļ�β��������
            //lidianqiang:MDA�Զ�����M30��ʱ��ΪM300
            strcpy(m_line_buf, "M300");
            this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //�����к�
            g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "MDA insert M30-2\n");
            return true;
        }
#ifdef USES_WOOD_MACHINE
        if(m_n_sub_program == MAIN_PROG){//������
            strcpy(m_line_buf, "M30");   //ľ��ר������û��M30ָ�ϵͳ�Զ����
            this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //�����к�
            g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "Insert M30-2\n");
        }else{//�ӳ���/�����
            strcpy(m_line_buf, "M99");   //ľ��ר������û��M99ָ�ϵͳ�Զ����
            this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //�����к�
            g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "Insert M99-2\n");
        }
        return true;
#else
        return false;   //ֱ�ӷ���
#endif
    }

    if(m_p_cur_file_pos == nullptr){
        m_p_cur_file_pos = m_p_file_map_info->ptr_map_file;
    }


    int index = 0;
    int last_char_type = 0;   //ǰһ���ַ����ͣ�0��ʾ���ַ�Ҳ�����֣�1��ʾ�ַ���2��ʾ����
    bool bSpace = false;   //ǰһ���ַ�Ϊ�հ��ַ�
    bool bFirst = false;    //������һ���ǿհ��ַ�
    bool bSwapdown = false;  //���·�ҳ

    char c_cur = *m_p_cur_file_pos;   //��ǰ�ַ�

    char c_next = '\0';   //��һ���ַ�
    uint64_t block_limit = m_p_file_map_info->ln_map_start
            + m_p_file_map_info->ln_map_blocksize;

    this->m_compiler_status.jump_flag = false;

REDO:
    //��ȡ��һ���ַ�
    if (m_ln_read_size >= block_limit - 1) {   //����ӳ���β��
        if (m_ln_read_size >= m_p_file_map_info->ln_file_size - 1) {  //�����ļ�β
            c_next = '\0';
        } else {
            printf("enter getlinedata111113333\n");
            if (m_p_file_map_info->Swapdown()) {
                bSwapdown = true;
                c_next = *m_p_file_map_info->ptr_map_file;
            } else {
                m_error_code = ERR_READ_FILE;

                return false;
            }
        }
    } else
        c_next = *(m_p_cur_file_pos + 1);   //��һ���ַ�

    this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //�����к�
    this->m_lexer_result.offset = this->m_ln_read_size;      //��������ƫ����
    //	printf("getline lineno = %lld\n", this->m_lexer_result.line_no);
    while (!m_b_eof && (c_cur != '\r' || c_next != '\n') &&//�������ֻ��и�ʽ��\r\n��\n��
           (c_cur != '\n')) {
        if (!m_b_comment) {
            if (c_cur == '(') {
                m_b_comment = true;
                m_b_left = true;
            } else if (c_cur == m_c_end_line
                       || (c_cur == '/' && c_next == '/')) {	//����ע��
                m_b_comment = true;
            }
        }

        if (!m_b_comment) {	//��ע��
            if (index < kMaxLineSize - 1)	//���ܳ���һ������ַ���
            {
                if (isspace(c_cur)) {
                    if (!bSpace && bFirst) {
                        bSpace = true;
                        m_line_buf[index++] = c_cur;
                    }
                } else {
                    bSpace = false;
                    bFirst = true;
                    //������ĸ����ĸ��������������ּ�Ŀհ��ַ��������հ��ַ���ɾ��
                    if(isdigit(c_cur)){
                        if(last_char_type != 2 && index > 0 && isspace(m_line_buf[index-1])){
                            index--;
                        }
                        last_char_type = 2;
                    }
                    else if(isalpha(c_cur)){
                        if(last_char_type != 1 && index > 0 && isspace(m_line_buf[index-1])){
                            index--;
                        }
                        last_char_type = 1;
                    }else
                        last_char_type = 0;
                    m_line_buf[index++] = toupper(c_cur);  //��ĸͳһ�ô�д
                }
            }
        } else {  //����ע�Ͳ���
            if (m_b_left && c_cur == ')') {  //����ע��
                m_b_comment = false;
                m_b_left = false;
            }
        }

        m_ln_read_size++;  //�Ѷ�ȡ�ֽ�����һ

        if (m_ln_read_size == block_limit) { //���ﵱǰӳ����ĩβ
            if (m_ln_read_size == m_p_file_map_info->ln_file_size) {  //�����ļ�β
                printf("eof 222, read_size = %lld, block_limit = %lld, file_size = %lld\n",
                       m_ln_read_size, block_limit,
                       m_p_file_map_info->ln_file_size);
                m_b_eof = true;
                break;
            } else if (bSwapdown) {
                m_p_cur_file_pos = m_p_file_map_info->ptr_map_file;
                block_limit = m_p_file_map_info->ln_map_start
                        + m_p_file_map_info->ln_map_blocksize;
                bSwapdown = false;
            }
        } else {
            m_p_cur_file_pos++;
        }

        c_cur = *m_p_cur_file_pos;     //��ǰ�ַ�
        if (m_ln_read_size == block_limit - 1) {     //����ӳ���β��
            if (m_ln_read_size == m_p_file_map_info->ln_file_size - 1) { //�����ļ�β
                c_next = '\0';
            } else {
                if (m_p_file_map_info->Swapdown()) {
                    bSwapdown = true;
                    c_next = *m_p_file_map_info->ptr_map_file;
                } else {
                    m_error_code = ERR_READ_FILE;
                    return false;
                }
            }
        } else
            c_next = *(m_p_cur_file_pos + 1);   //��һ���ַ�
    }

    if (m_b_comment && !m_b_left)
        m_b_comment = false;  //����"//"��ʼ�ĵ���ע�ͣ����б�־��λ

    //����һ�д�����β�Ŀհ��ַ�
    while (index > 0 && isspace(m_line_buf[index - 1])) {
        m_line_buf[index - 1] = '\0';
        index--;
    }

    //������β�ַ���ָ����һ������
    if (!m_b_eof) {

        if (c_cur == '\n') {
            m_p_cur_file_pos += 1;
            m_ln_read_size += 1;
        } else {
            m_p_cur_file_pos += 2;
            m_ln_read_size += 2;
        }

        m_ln_cur_line_no++;  //��ǰ�����кż�һ

        if (m_ln_read_size >= m_p_file_map_info->ln_file_size) {
            printf("eof 3333, read_size = %llu, file_size = %llu\n",
                   m_ln_read_size, m_p_file_map_info->ln_file_size);
            m_b_eof = true;
        }

    } else {
        //		this->StopCompile();
        g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "end of the file\n");
    }

    if (index == 0) {  //����հ������ļ�δ�������������ȡ��һ��
        if (!m_b_eof) {
            index = 0;
            bSpace = false;   //ǰһ���ַ�Ϊ�հ��ַ�
            bFirst = false;    //������һ���ǿհ��ַ�
            c_cur = *m_p_cur_file_pos;     //��ǰ�ַ�
            //	c_next = *(m_p_cur_file_pos+1);   //��һ���ַ�
            goto REDO;
            //�ض���һ��
        } else if (this->m_work_mode == MDA_COMPILER) {
            //MDAģʽ������һ��M30��������
            //lidianqiang:MDA�Զ�����M30��ʱ��ΪM300
            strcpy(m_line_buf, "M300");
            this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //�����к�
            g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "MDA insert M30-1\n");
        } else {
#ifdef USES_WOOD_MACHINE
            if(m_n_sub_program == MAIN_PROG){//������
                strcpy(m_line_buf, "M30");   //ľ��ר������û��M30ָ�ϵͳ�Զ����
                this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //�����к�
                g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "Insert M30-1\n");
            }else{//�ӳ���/�����
                strcpy(m_line_buf, "M99");   //ľ��ר������û��M99ָ�ϵͳ�Զ����
                this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //�����к�
                g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "Insert M99-1\n");
            }
            return true;
#else
            res = false;
#endif
        }

    }

    return res;
}

//int total2 = 0;
/**
 * @brief ����һ�д���
 * @return ִ�н��
 */
bool Compiler::CompileLine() {
    //	struct timeval tvStart;
    //	struct timeval tvNow;
    //	unsigned int nTimeDelay = 0;
    //
    //	gettimeofday(&tvStart, NULL);

    //	printf("------> compile line ...\n");

    bool res = true;

    switch (m_n_compile_state) {
    case FILE_HEAD:
        res = ProcessHead();
        break;

        //  case FILE_FIND:
        //      dowithFind();
        //      break;

    case FILE_MAIN:
        res = ProcessMain();
        break;
    default:
        //������
        break;
    }

    //	gettimeofday(&tvNow, NULL);
    //	nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;
    //	total2 +=nTimeDelay;
    //	if(m_lexer_result.line_no % 10000 == 0){
    //		printf("***compile 10000 line cost %ldus***\n", total2);
    //		total2 = 0;
    //	}

    return res;
}


/**
 * @brief ִ�б��������Msg,�޸ı�����״̬
 * @return true--�ɹ�   false--ʧ��
 */
bool Compiler::RunMessage() {
    bool res = true;
    RecordMsg *msg = nullptr;

    if(compiler_lock){
    	return false;
    }

    //	int count = m_p_parser_result->GetLength();

    ListNode<RecordMsg *> *node = m_p_parser_result->HeadNode();
    CodeMsgType msg_type = NORMAL_MSG;

    while (node != nullptr) {
        msg = static_cast<RecordMsg *>(node->data);

        if (msg != nullptr) {
            if(msg->CheckFlag(FLAG_JUMP) && this->m_p_channel_control->CheckFuncState(FS_BLOCK_SKIP)){  //���μ���,��ǰָ��ȡ��
                this->m_p_parser_result->RemoveHead();
                delete node;
                node = this->m_p_parser_result->HeadNode();  //ȡ��һ����Ϣ
                continue;
            }
            msg_type = msg->GetMsgType();

            // @test zk
            static uint64_t cur_line = 0;
            static int type = 0;

            if(cur_line != msg->GetLineNo() || type != msg->GetMsgType()){
                cur_line = msg->GetLineNo();
                type = msg->GetMsgType();
                printf("compiler run message  line no: %llu,  type: %d\n", cur_line, msg_type);
            }
            // @test zk

            switch (msg_type) {
            case AUX_MSG:
                res = RunAuxMsg(msg);
                break;
            case SUBPROG_CALL_MSG:
                res = this->RunSubProgCallMsg(msg);
                compiler_lock = true;
                break;
            case MACRO_PROG_CALL_MSG:
                res = this->RunMacroProgCallMsg(msg);
                compiler_lock = true;
                break;
            case COORD_MSG:
                res = this->RunCoordMsg(msg);
                break;
            case RAPID_MSG:
                res = this->RunRapidMsg(msg);
                break;
            case LINE_MSG:
                res = this->RunLineMsg(msg);
                break;
            case COMPENSATE_MSG:
                res = this->RunCompensateMsg(msg);
                break;
            case MODE_MSG:
                res = this->RunModeMsg(msg);
                break;
            case FEED_MSG:
                res = this->RunFeedMsg(msg);
                break;
            case SPEED_MSG:
                res = this->RunSpeedMsg(msg);
                break;
            case TOOL_MSG:
                res = this->RunToolMsg(msg);
                break;
            case LOOP_MSG:
                res = this->RunLoopMsg(msg);
                compiler_lock = true;
                break;
            case ARC_MSG:
                res = this->RunArcMsg(msg);
                break;
            case MACRO_MSG:
                res = this->RunMacroMsg(msg);
                break;
            case ERROR_MSG:
                res = this->RunErrorMsg(msg);
                break;
            case POLAR_INTP_MSG:
                res = this->RunPolarIntpMsg(msg);
                break;
            case CLEAR_CIRCLE_POS_MSG:
                res = this->RunClearCirclePosMsg(msg);
                break;
            case TIME_WAIT_MSG:
                res = this->RunTimeWaitMsg(msg);
                break;
            case REF_RETURN_MSG:
                res = this->RunRefReturnMsg(msg);
                break;
#ifdef USES_SPEED_TORQUE_CTRL	
            case SPEED_CTRL_MSG:
                res = this->RunSpeedCtrlMsg(msg);
                break;
            case TORQUE_CTRL_MSG:
                res = this->RunTorqueCtrlMsg(msg);
                break;
#endif			
            case SKIP_MSG:
                res = this->RunSkipMsg(msg);
                break;
            case AUTO_TOOL_MEASURE_MSG:
                res = this->RunAutoToolMeasureMsg(msg);
                break;
            default:
                break;
            }

            if (!res) {
                if (this->m_error_code != ERR_NONE){
                    printf("clear msg\n");
                    this->m_p_parser_result->Clear();  //������ն���

                }
                break;  //ִ�в��ɹ�
            }

            //msg->PrintString();

            //TODO ���ӵ�������

            this->m_p_parser_result->RemoveHead();
            this->m_p_block_msg_list->Append(node);   //ѹ��ֶζ���

        }

        node = this->m_p_parser_result->HeadNode();  //ȡ��һ����Ϣ
    }

    //����ֶ�
    node = this->m_p_block_msg_list->HeadNode();
    while (node != nullptr) {
        msg = static_cast<RecordMsg *>(node->data);
        if (msg != nullptr) { //��������£�ȫ�����
            if (msg == this->m_p_last_move_msg
                    && this->m_error_code == ERR_NONE &&
                    this->m_work_mode == AUTO_COMPILER) {  //MDAģʽ�£�ȫ�����
                break;  //δȷ���Ƿ�Ϊ�ֶ����һ���ƶ�ָ��ݲ�ִ��
            }

            this->m_p_block_msg_list->RemoveHead();
            //			this->m_p_output_msg_list->Append(node);   //ѹ������Ͷ���
            this->m_p_tool_compensate->ProcessData(node);  //ѹ�뵶�����ݻ���

            if(m_p_tool_compensate->err_code != ERR_NONE){
                m_error_code = m_p_tool_compensate->err_code;
                CreateError(m_p_tool_compensate->err_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                            m_n_channel_index);
                //m_p_tool_compensate->clearError();

                res = false;
            }

        }

        node = this->m_p_block_msg_list->HeadNode();  //ȡ��һ����Ϣ
    }

    /*if(res){
        printf("----compiler run message %llu success\n", lineNo);
    }else{
        printf("----compiler run message %llu failed\n", lineNo);
    }*/

    return res;
}

/**
 * @brief ִ�и���ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunAuxMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    AuxMsg *tmp = (AuxMsg *) msg;
    uint8_t count = tmp->GetMCount();

    for(uint8_t i = 0; i < count; i++){
        switch (tmp->GetMCode(i)) {
        case 30:  	//M30
        case 2:		//M02
            if (m_n_sub_program != MAIN_PROG) {  //�ӳ���������֣��澯
                CreateErrorMsg(ERR_SUB_END, msg->GetLineNo());
                return false;
            } else {
                this->m_b_compile_over = true;
                printf("compiler run M30\n");

            }

            break;
        case 06:   //����
            printf("run m06 msg\n");

            break;

            //	case 98:   //M98   �ӳ������  ʹ�ö�����SubProgCallMsg����
            //		break;
        case 99:   //M99
            if (m_n_sub_program != MAIN_PROG) {
                this->ReturnFromSubProg();   //�ӳ����򷵻ص��ó���
            } else {
                this->m_b_compile_over = true;
            }

            printf("compiler run M99\n");
            break;
        case 300: //lidianqiang:MDA�Զ�����M30��ʱ��ΪM300
            this->m_b_compile_over = true;
            break;
        default:
            break;
        }
    }
    return true;
}

/**
 * @brief �������������ӳ������ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunSubProgCallMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;

    if (!this->m_b_prescan_over && m_thread_prescan != 0) //Ԥɨ��δ�������ݲ�ִ��
        return false;

    SubProgCallMsg *sub_msg = (SubProgCallMsg *) msg;

    int sub_index = sub_msg->GetSubProgIndex();

    //�����ӳ���
    int sub_loc = this->FindSubProgram(sub_index);
    sub_msg->SetSubProgType(sub_loc);
    if (sub_loc == 0) {
        CreateErrorMsg(ERR_NO_SUB_PROG, msg->GetLineNo());  //�Ҳ�����Ӧ�ӳ���
        return false;
    }

    //���浱ǰ������״̬
    this->SaveScene();

    //�ֲ�������ջ
    m_p_channel_control->GetMacroVar()->PushLocalVar();

    this->m_n_sub_call_times = sub_msg->GetCallTimes();//���ô���


    //���ӳ����ļ�
    this->m_n_sub_program = SUB_PROG;
    char filepath[kMaxPathLen] = { 0 };   //�ļ�·��
    if (sub_loc == 1) {   //ͬ�ļ��ӳ���
        ListNode < SubProgOffset > *node = m_p_list_subprog->HeadNode();
        uint64_t offset_sub = 0;
        while (node != nullptr) {
            if (node->data.sub_index == sub_index) {
                offset_sub = node->data.offset;
                break;
            }
            node = node->next;
        }

        if (!this->m_p_file_map_info->JumpTo(offset_sub)) {   //ӳ��ʧ��
            CreateErrorMsg(ERR_JUMP_SUB_PROG, msg->GetLineNo());  //�ӳ�����תʧ��
            return false;
        }

        this->m_ln_read_size = offset_sub;
        this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file
                + m_ln_read_size - m_p_file_map_info->ln_map_start;

        this->m_ln_cur_line_no = node->data.line_no;
    } else {
        sub_msg->GetSubProgName(filepath, true);
        sub_msg->SetLastProgFile(this->m_p_file_map_info->str_file_name+strlen(PATH_NC_FILE));  // ���浱ǰ�ļ�·�������·��
        //�������ļ���
        if (!this->OpenFile(filepath, (bool)m_n_sub_program)){ //���Դ�nc�ļ�ʧ��
            return false;
        }
    }

    if (m_work_mode == AUTO_COMPILER)
        this->m_n_compile_state = FILE_HEAD;
    else
        this->m_n_compile_state = FILE_MAIN;
    this->m_n_head_state = HEAD_INFO;

    return true;
}

/**
 * @brief �����������к�������ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunMacroProgCallMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;

    if (!this->m_b_prescan_over && m_thread_prescan != 0){ //Ԥɨ��δ�������ݲ�ִ��

        return false;
    }

    MacroProgCallMsg *sub_msg = (MacroProgCallMsg *) msg;

    int macro_index = sub_msg->GetMacroProgIndex();

    //�����ӳ���
    int macro_loc = this->FindSubProgram(macro_index);
    sub_msg->SetMacroProgType(macro_loc);
    if (macro_loc == 0) {
        CreateErrorMsg(ERR_NO_SUB_PROG, msg->GetLineNo());  //�Ҳ�����Ӧ�ӳ���
        return false;
    }

    //���浱ǰ������״̬
    this->SaveScene();

    Variable *pv = m_p_channel_control->GetMacroVar();
    //�ֲ�������ջ
    pv->PushLocalVar();

    //��ֵ�Ա����������ֲ�����
    uint8_t pc = 0;
    uint32_t pm = 0;
    int i  = 0;
    double *pp = sub_msg->GetParameter(pm, pc);
    while(pm != 0){
        if(pm & 0x01){
            pv->SetVarValue(kMacroParamToLocalVarIndex[i], *pp);
            pp++;
        }
        pm = pm>>1;
        i++;
    }

    this->m_n_sub_call_times = sub_msg->GetCallTimes();//���ô���

    //���ӳ����ļ�
    this->m_n_sub_program = MACRO_PROG;
    char filepath[kMaxPathLen] = { 0 };   //�ļ�·��
    if (macro_loc == 1) {   //ͬ�ļ��ӳ���
        ListNode < SubProgOffset > *node = m_p_list_subprog->HeadNode();
        uint64_t offset_sub = 0;
        while (node != nullptr) {
            if (node->data.sub_index == macro_index) {
                offset_sub = node->data.offset;
                break;
            }
            node = node->next;
        }

        if (!this->m_p_file_map_info->JumpTo(offset_sub)) {   //ӳ��ʧ��
            CreateErrorMsg(ERR_JUMP_SUB_PROG, msg->GetLineNo());  //�ӳ�����תʧ��
            return false;
        }

        this->m_ln_read_size = offset_sub;
        this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file
                + m_ln_read_size - m_p_file_map_info->ln_map_start;

        this->m_ln_cur_line_no = node->data.line_no;
    } else {
        sub_msg->GetMacroProgName(filepath, true);

        sub_msg->SetLastProgFile(this->m_p_file_map_info->str_file_name+strlen(PATH_NC_FILE));  //���浱ǰ�ļ�·��, ���·�������͸�HMI
        //�������ļ���
        if (!this->OpenFile(filepath, (bool)m_n_sub_program)){ //���Դ�nc�ļ�ʧ��
            return false;
        }

    }

    if (m_work_mode == AUTO_COMPILER)
        this->m_n_compile_state = FILE_HEAD;
    else
        this->m_n_compile_state = FILE_MAIN;
    this->m_n_head_state = HEAD_INFO;

    return true;
}

/**
 * @brief �������������Զ��Ե�ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunAutoToolMeasureMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;

    if(*this->m_p_simulate_mode != SIM_NONE)  //����ֱ�ӷ���
        return true;

    if (!this->m_b_prescan_over && m_thread_prescan != 0){ //Ԥɨ��δ�������ݲ�ִ��

        return false;
    }

    AutoToolMeasureMsg *sub_msg = (AutoToolMeasureMsg *) msg;

    int macro_index = sub_msg->GetMacroProgIndex();      //�Զ��Ե������

    //�����ӳ���
    int macro_loc = this->FindSubProgram(macro_index);
    sub_msg->SetMacroProgType(macro_loc);
    if (macro_loc == 0 || macro_loc == 1) {
        printf("create error in RunAutoToolMeasureMsg\n");
        CreateErrorMsg(ERR_NO_SUB_PROG, msg->GetLineNo());  //�Ҳ�����Ӧ�ӳ���
        return false;
    }

    //���浱ǰ������״̬
    this->SaveScene();

    Variable *pv = m_p_channel_control->GetMacroVar();
    //�ֲ�������ջ
    pv->PushLocalVar();

    //��ֵĿ��λ�ò������ֲ�����
    uint32_t axis_mask = sub_msg->GetAxisMoveMask();
    DPointChn &target = sub_msg->GetTarget();
    int h_data = sub_msg->GetHIndex();
    int l_data = sub_msg->GetTimes();
    if(axis_mask & 0x01){
        pv->SetVarValue(kMacroParamToLocalVarIndex[X_DATA], target.m_df_point[0]);
    }
    if(axis_mask & 0x02){
        pv->SetVarValue(kMacroParamToLocalVarIndex[Y_DATA], target.m_df_point[1]);
    }
    if(axis_mask & 0x04){
        pv->SetVarValue(kMacroParamToLocalVarIndex[Z_DATA], target.m_df_point[2]);
    }
    pv->SetVarValue(kMacroParamToLocalVarIndex[H_DATA], h_data);
    pv->SetVarValue(kMacroParamToLocalVarIndex[Q_DATA], l_data);  //L�������ں���򴫲Σ���Q���

    this->m_n_sub_call_times = 1;//���ô���

    //���ӳ����ļ�
    this->m_n_sub_program = MACRO_PROG;
    char filepath[kMaxPathLen] = { 0 };   //�ļ�·��
    sub_msg->GetMacroProgName(filepath, true);


    //�������ļ���
    if (!this->OpenFile(filepath, (bool)m_n_sub_program)){ //���Դ�nc�ļ�ʧ��
        return false;
    }

    if (m_work_mode == AUTO_COMPILER)
        this->m_n_compile_state = FILE_HEAD;
    else
        this->m_n_compile_state = FILE_MAIN;
    this->m_n_head_state = HEAD_INFO;

    return true;
}

/**
 * @brief ִ������ϵָ��ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunCoordMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    CoordMsg *tmp = (CoordMsg *) msg;

    int gcode = tmp->GetGCode();
    switch (gcode) {
    case G52_CMD:  		//G52  �ֲ�����ϵ
        m_compiler_status.mode.gmode[0] = gcode;  //����ģ̬
        break;
    case G53_CMD:		//G53  ��������ϵ
        //�����������ָ��
        if(m_compiler_status.mode.gmode[3] == G91_CMD){  //�������ģʽ
            double *p_target_pos = tmp->GetTargetPos().m_df_point;
            double *p_source_pos = m_compiler_status.cur_pos.m_df_point;
            uint32_t mask = tmp->GetAxisMask();
            uint32_t tm = 0x01;

            for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
                if(mask & tm){
                    *p_target_pos += *p_source_pos;
                }
                tm = tm<<1;
                p_target_pos++;
                p_source_pos++;
            }

        }else{
            //��Ŀ��λ�û���Ϊ��������ϵ
            this->m_p_channel_control->TransMachCoordToWorkCoord(tmp->GetTargetPos(), m_compiler_status.mode.gmode[14], m_compiler_status.mode.h_mode, tmp->GetAxisMask());
        }
#ifdef USES_FIVE_AXIS_FUNC
        this->ProcessFiveAxisRotPos(tmp->GetTargetPos(), m_compiler_status.cur_pos, tmp->GetAxisMask());
#endif
        //		m_compiler_status.cur_pos = tmp->GetTargetPos(); //���±��뵱ǰλ��
        this->SetCurPos(tmp->GetTargetPos());

        m_compiler_status.mode.gmode[0] = gcode;  //����ģ̬
        break;
    case G92_CMD:  	 	//G92  ��������ϵ
        m_compiler_status.mode.gmode[0] = gcode;  //����ģ̬
        break;
    default:
        if ((gcode >= G54_CMD && gcode <= G59_CMD)
                || (gcode >= G5401_CMD && gcode <= G5499_CMD)){
            tmp->SetLastGCode(m_compiler_status.mode.gmode[14]);   //������ʷֵ�����ڷ�����������
            m_compiler_status.mode.gmode[14] = gcode;  //����ģ̬
        }
        break;
    }

    //	printf("run coord msg: %d\n", gcode);
    return true;
}

/**
 * @brief ִ��һ��ģָ̬����Ϣ������������ģָ̬��
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunModeMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    ModeMsg *tmp = (ModeMsg *) msg;
    int gcode = tmp->GetGCode();
    switch (gcode) {
    case G90_CMD:   //���Ա��ָ��
    case G91_CMD:	//��Ա��ָ��
    case G21_CMD:   //���Ƶ�λ
    case G20_CMD:   //Ӣ�Ƶ�λ
        tmp->SetLastGCode(m_compiler_status.mode.gmode[GetModeGroup(gcode)]);   //������ʷ���ݣ��������ַ�������
        m_compiler_status.mode.gmode[GetModeGroup(gcode)] = gcode;
        break;
    case G17_CMD:
    case G18_CMD:
    case G19_CMD:
        tmp->SetLastGCode(m_compiler_status.mode.gmode[GetModeGroup(gcode)]);   //������ʷ���ݣ��������ַ�������
        //		this->m_tool_compensate.SetCurPlane(gcode);   //���õ���ģ�鵱ǰƽ��
        break;
    case G84_3_CMD:
        tmp->SetLastGCode(m_compiler_status.mode.gmode[39]);   //������ʷ���ݣ��������ַ�������
        break;
    default:
        tmp->SetLastGCode(m_compiler_status.mode.gmode[GetModeGroup(gcode)]);   //������ʷ���ݣ��������ַ�������
        m_compiler_status.mode.gmode[GetModeGroup(gcode)] = gcode;   //�޸ı�����Gģ̬
        break;
    }


    return true;
}

/**
 * @brief ִ�п��ٶ�λָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunRapidMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    RapidMsg *tmp = (RapidMsg *) msg;

    //�����������ָ��
    if(m_compiler_status.mode.gmode[3] == G91_CMD){  //�������ģʽ
        double *p_target_pos = tmp->GetTargetPos().m_df_point;
        double *p_source_pos = m_compiler_status.cur_pos.m_df_point;
        uint32_t mask = tmp->GetAxisMoveMask();
        uint32_t tm = 0x01;

        for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            if(mask & tm){
                *p_target_pos += *p_source_pos;
            }
            tm = tm<<1;
            p_target_pos++;
            p_source_pos++;
        }

    }else{
        //��Ŀ��λ�û���Ϊ��������ϵ
        if(tmp->IsMachCoord())
            this->m_p_channel_control->TransMachCoordToWorkCoord(tmp->GetTargetPos(), m_compiler_status.mode.gmode[14],
                m_compiler_status.mode.h_mode, tmp->GetAxisMoveMask());
    }

#ifdef USES_FIVE_AXIS_FUNC
    this->ProcessFiveAxisRotPos(tmp->GetTargetPos(), tmp->GetSourcePos(), tmp->GetAxisMoveMask());
#endif
    m_compiler_status.mode.gmode[1] = G00_CMD;
    m_compiler_status.mode.gmode[9] = G80_CMD;  //�Զ�ȡ��ѭ��ָ��
    //	m_compiler_status.cur_pos = tmp->GetTargetPos(); //���±��뵱ǰλ��
    this->SetCurPos(tmp->GetTargetPos());

    return true;
}

/**
 * @brief ִ��ֱ������ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunLineMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    LineMsg *tmp = (LineMsg *) msg;

    double feed = m_compiler_status.mode.f_mode;
    if(feed > m_p_channel_config->g01_max_speed)
        feed = m_p_channel_config->g01_max_speed;
    tmp->SetFeed(feed);  //���ý����ٶ�

    //�����������ָ��
    if(m_compiler_status.mode.gmode[3] == G91_CMD){  //�������ģʽ
        double *p_target_pos = tmp->GetTargetPos().m_df_point;
        double *p_source_pos = m_compiler_status.cur_pos.m_df_point;
        uint32_t mask = tmp->GetAxisMoveMask();
        uint32_t tm = 0x01;

        for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            if(mask & tm){
                *p_target_pos += *p_source_pos;
            }
            tm = tm<<1;
            p_target_pos++;
            p_source_pos++;
        }

    }else{
        //��Ŀ��λ�û���Ϊ��������ϵ
        if(tmp->IsMachCoord()){
            printf("G54=%hu, h_code=%hhu\n", m_compiler_status.mode.gmode[14], m_compiler_status.mode.h_mode);
            printf("line g53: %lf, %lf, %lf\n", tmp->GetTargetPos().m_df_point[0], tmp->GetTargetPos().m_df_point[1], tmp->GetTargetPos().m_df_point[2]);
            this->m_p_channel_control->TransMachCoordToWorkCoord(tmp->GetTargetPos(), m_compiler_status.mode.gmode[14],
                    m_compiler_status.mode.h_mode, tmp->GetAxisMoveMask());

            printf("line g53 trans workcoord: %lf, %lf, %lf\n", tmp->GetTargetPos().m_df_point[0], tmp->GetTargetPos().m_df_point[1], tmp->GetTargetPos().m_df_point[2]);
        }
    }

#ifdef USES_FIVE_AXIS_FUNC
    this->ProcessFiveAxisRotPos(tmp->GetTargetPos(), tmp->GetSourcePos(), tmp->GetAxisMoveMask());
#endif

    m_compiler_status.mode.gmode[1] = G01_CMD;
    m_compiler_status.mode.gmode[9] = G80_CMD;  //�Զ�ȡ��ѭ��ָ��
    //	m_compiler_status.cur_pos = tmp->GetTargetPos(); //���±��뵱ǰλ��
    this->SetCurPos(tmp->GetTargetPos());
    //	printf("run line msg : %lf, %lf, %lf, F=%lf\n", tmp->GetTargetPos().GetAxisValue(0), tmp->GetTargetPos().GetAxisValue(1),
    //			tmp->GetTargetPos().GetAxisValue(2), tmp->GetFeed());
    return true;
}

/**
 * @brief ִ��Բ������ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunArcMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    ArcMsg *tmp = (ArcMsg *) msg;
    int gcode = tmp->GetGCode();

    //�����������ָ��
    if(m_compiler_status.mode.gmode[3] == G91_CMD){  //�������ģʽ
        double *p_target_pos = tmp->GetTargetPos().m_df_point;
        double *p_source_pos = m_compiler_status.cur_pos.m_df_point;
        uint32_t mask = tmp->GetAxisMoveMask();
        uint32_t tm = 0x01;

        for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            if(mask & tm){
                *p_target_pos += *p_source_pos;
            }
            tm = tm<<1;
            p_target_pos++;
            p_source_pos++;
        }

    }else{
        //��Ŀ��λ�û���Ϊ��������ϵ
        if(tmp->IsMachCoord())
            this->m_p_channel_control->TransMachCoordToWorkCoord(tmp->GetTargetPos(), m_compiler_status.mode.gmode[14],
                m_compiler_status.mode.h_mode, tmp->GetAxisMoveMask());
    }

#ifdef USES_FIVE_AXIS_FUNC
    this->ProcessFiveAxisRotPos(tmp->GetTargetPos(), tmp->GetSourcePos(), tmp->GetAxisMoveMask());
#endif

    tmp->SetFeed(m_compiler_status.mode.f_mode);

    m_compiler_status.mode.gmode[1] = gcode;
    m_compiler_status.mode.gmode[9] = G80_CMD;  //�Զ�ȡ��ѭ��ָ��
    //	m_compiler_status.cur_pos = tmp->GetTargetPos(); //���±��뵱ǰλ��
    this->SetCurPos(tmp->GetTargetPos());
    return true;
}

/**
 * @brief ִ�н����ٶ�ָ��ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunFeedMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    FeedMsg *tmp = (FeedMsg *) msg;

    m_compiler_status.mode.f_mode = tmp->GetFeed();

    //printf("run feed message: %lf~~~~~\n", m_compiler_status.mode.f_mode);

    return true;
}

/**
 * @brief ִ�н����ٶ�ָ��ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunSpeedMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    SpeedMsg *tmp = (SpeedMsg *) msg;

    m_compiler_status.mode.s_mode = tmp->GetSpeed();

    return true;
}

/**
 * @brief ִ�е���ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunCompensateMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    CompensateMsg *tmp = (CompensateMsg *) msg;
    int gcode = tmp->GetGCode();

    //�����������ָ��
    if(m_compiler_status.mode.gmode[3] == G91_CMD){  //�������ģʽ
        double *p_target_pos = tmp->GetTargetPos().m_df_point;
        double *p_source_pos = m_compiler_status.cur_pos.m_df_point;
        uint32_t mask = tmp->GetAxisMoveMask();
        uint32_t tm = 0x01;

        for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            if(mask & tm){
                *p_target_pos += *p_source_pos;
            }
            tm = tm<<1;
            p_target_pos++;
            p_source_pos++;
        }

    }else{
        //��Ŀ��λ�û���Ϊ��������ϵ
        if(tmp->IsMachCoord())
            this->m_p_channel_control->TransMachCoordToWorkCoord(tmp->GetTargetPos(), m_compiler_status.mode.gmode[14],
                m_compiler_status.mode.h_mode, tmp->GetAxisMoveMask());
    }

#ifdef USES_FIVE_AXIS_FUNC
    this->ProcessFiveAxisRotPos(tmp->GetTargetPos(), tmp->GetSourcePos(), tmp->GetAxisMoveMask());
#endif

    if (gcode == G41_CMD || gcode == G42_CMD) {

        tmp->SetCompLastValue(m_compiler_status.mode.d_mode);  //��¼��ʷֵ
        m_compiler_status.mode.d_mode = tmp->GetCompValue();   //�޸ı�����״̬

        SCToolOffsetConfig * offset_config = g_ptr_parm_manager->GetToolConfig(m_n_channel_index);
        int d_value = tmp->GetCompValue();
        if(d_value > kMaxToolCount  or d_value < 0) return false;
        printf("===== d value: %d\n", d_value);
        if(d_value == 0){
            this->m_p_tool_compensate->setToolRadius(0);
        }else {
            double radius = offset_config->radius_compensation[d_value - 1] + offset_config->radius_wear[d_value-1];
            this->m_p_tool_compensate->setToolRadius(radius);
        }
        //m_p_channel_control->UpdateModeData(D_MODE, d_value);//llx add,��SCֱ�Ӹ���Dģ̬��������MC

    } else if (gcode == G43_CMD || gcode == G44_CMD || gcode == G43_4_CMD) {  //���߳��Ȳ���
        tmp->SetCompLastValue(m_compiler_status.mode.h_mode);  //��¼��ʷֵ
        if(*m_p_simulate_mode != SIM_NONE) //�Ƿ���ģʽ��������������Ч
            m_compiler_status.mode.h_mode = tmp->GetCompValue();   //�޸ı�����״̬
    }else if(gcode == G49_CMD){
        tmp->SetCompLastValue(m_compiler_status.mode.h_mode);  //��¼��ʷֵ
        m_compiler_status.mode.h_mode = 0;  //ȡ��������������RTCP
    }else if(gcode == G40_CMD){
    	tmp->SetCompLastValue(m_compiler_status.mode.d_mode);  //��¼��ʷֵ
        m_compiler_status.mode.d_mode = 0;   //ȡ�����߰뾶����
    }

    tmp->SetFeed(m_compiler_status.mode.f_mode);

    m_compiler_status.mode.gmode[GetModeGroup(gcode)] = gcode;   //�޸ı�����״̬
    //	m_compiler_status.cur_pos = tmp->GetTargetPos(); //���±��뵱ǰλ��
    this->SetCurPos(tmp->GetTargetPos());
    return true;
}

/**
 * @brief �����������м�����岹��Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunPolarIntpMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    PolarIntpMsg *tmp = (PolarIntpMsg *) msg;
    int gcode = tmp->GetGCode();

#ifdef USES_GRIND_MACHINE
    if (gcode == G12_2_CMD || gcode == G12_3_CMD) {  //ĥ��ָ��

        printf("RunPolarIntpMsg~\n");

    }


#endif

    m_compiler_status.mode.gmode[GetModeGroup(gcode)] = gcode;   //�޸ı�����״̬
    return true;
}

/**
 * @brief ��������������ʱ��Ϣ
 * @param msg
 * @return
 */
bool Compiler::RunTimeWaitMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    TimeWaitMsg *tmp = (TimeWaitMsg *) msg;
    int gcode = tmp->GetGCode();

    m_compiler_status.mode.gmode[0] = gcode;   //�޸ı�����״̬
    return true;
}

/**
 * @brief �����������лزο�����Ϣ
 * @param msg
 * @return
 */
bool Compiler::RunRefReturnMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    RefReturnMsg *tmp = (RefReturnMsg *) msg;
    int gcode = tmp->GetGCode();

    //�����������ָ��
    if(m_compiler_status.mode.gmode[3] == G91_CMD){  //�������ģʽ
        DPointChn &mid_pos = tmp->GetMiddlePos();
        double *p_target_pos = mid_pos.m_df_point;
        double *p_source_pos = m_compiler_status.cur_pos.m_df_point;
        uint32_t mask = tmp->GetAxisMask();
        uint32_t tm = 0x01;

        for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            if(mask & tm){
                //				printf("RunRefReturnMsg1: i = %d, target=%lf, src=%lf\n", i, *p_target_pos, *p_source_pos);
                *p_target_pos += *p_source_pos;
                //				printf("RunRefReturnMsg2: i = %d, target=%lf, src=%lf\n", i, *p_target_pos, *p_source_pos);

            }
            tm = tm<<1;
            p_target_pos++;
            p_source_pos++;
        }

    }

    m_compiler_status.mode.gmode[0] = gcode;   //�޸ı�����״̬
    return true;
}


/**
 * @brief ��������������ת��Ϣ
 * @param msg
 * @return
 */
bool Compiler::RunSkipMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    SkipMsg *tmp = (SkipMsg *) msg;
    int gcode = tmp->GetGCode();

    //�����������ָ��
    if(m_compiler_status.mode.gmode[3] == G91_CMD){  //�������ģʽ
        double *p_target_pos = tmp->GetTargetPos().m_df_point;
        double *p_source_pos = m_compiler_status.cur_pos.m_df_point;
        uint32_t mask = tmp->GetAxisMoveMask();
        uint32_t tm = 0x01;

        for(int i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
            if(mask & tm){
                *p_target_pos += *p_source_pos;
            }
            tm = tm<<1;
            p_target_pos++;
            p_source_pos++;
        }

    }else{
        //��Ŀ��λ�û���Ϊ��������ϵ
        if(tmp->IsMachCoord())
            this->m_p_channel_control->TransMachCoordToWorkCoord(tmp->GetTargetPos(), m_compiler_status.mode.gmode[14],
                m_compiler_status.mode.h_mode, tmp->GetAxisMoveMask());
    }

    tmp->SetFeed(m_compiler_status.mode.f_mode);

    m_compiler_status.mode.gmode[0] = gcode;   //�޸ı�����״̬

    //	m_compiler_status.cur_pos = tmp->GetTargetPos(); //���±��뵱ǰλ��
    this->SetCurPos(tmp->GetTargetPos());
    return true;
}

#ifdef USES_SPEED_TORQUE_CTRL	
/**
 * @brief �������������ٶȿ�����Ϣ
 * @param msg : ָ����Ϣ
 * @return
 */
bool Compiler::RunSpeedCtrlMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    SpeedCtrlMsg *tmp = (SpeedCtrlMsg *) msg;
    int gcode = tmp->GetGCode();

    m_compiler_status.mode.gmode[kMaxGModeCount-1] = gcode;   //�޸ı�����״̬
    return true;
}


/**
 * @brief ���������������ؿ�����Ϣ
 * @param msg : ָ����Ϣ
 * @return
 */
bool Compiler::RunTorqueCtrlMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    TorqueCtrlMsg *tmp = (TorqueCtrlMsg *) msg;
    int gcode = tmp->GetGCode();

    m_compiler_status.mode.gmode[kMaxGModeCount-1] = gcode;   //�޸ı�����״̬


    printf("execute Compiler:: RunTorqueCtrlMsg \n");

    return true;
}
#endif

/**
 * @brief ������������������Ȧλ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunClearCirclePosMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    ClearCirclePosMsg *tmp = (ClearCirclePosMsg *) msg;
    int gcode = tmp->GetGCode();

    m_compiler_status.mode.gmode[39] = gcode;   //�޸ı�����״̬
    return true;
}

/**
 * @brief �����������к�ָ��
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunMacroMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;

    MacroCmdMsg *tmp = (MacroCmdMsg *) msg;
    int macro_cmd = tmp->GetMacroCmd();
    MacroVarValue exp_res;   //�ݴ����ʽ���
    uint64_t offset = 0, line_no = 0;
    LoopOffset loop;
    int index = 0;
    bool flag = true;
    int res = 0;

    switch (macro_cmd) {
    case MACRO_CMD_GOTO:	//��תָ������ת����Ӧ���봦
        //printf("---------------- goto cmd -------> %llu\n", tmp->GetLineNo());
        if(!tmp->GetMacroExpCalFlag(0)){
            if(!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0), tmp->GetMacroExpResult(0))) {//���ʽ����ʧ��
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //���ʽ�����쳣
                    return false;
                }

                //û�д�����˵�����ʽ����Ҫ����ϵͳ����������ȴ�MC���е�λ
                return false;
            }
            tmp->SetMacroExpCalFlag(0, true);
        }


        //���ʽ����ɹ�,ִ����ת����
        res = FindJumpLabel(static_cast<int>(tmp->GetMacroExpResult(0).value), offset, line_no);
        if (0 == res) {
            //û���ҵ���ת�㣬�����澯
            CreateErrorMsg(ERR_NO_JUMP_LABEL, tmp->GetLineNo());  //���ʽ�����쳣
            return false;
        }else if(2 == res)
            return false;   //�ȴ�Ԥɨ�����

        //�����ת�Ϸ���
        if(!this->CheckJumpGoto(tmp->GetLineNo(), line_no)){
            CreateErrorMsg(ERR_JUMP_GOTO_ILLIGAL, tmp->GetLineNo());
            return false;
        }

        if (!this->m_p_file_map_info->JumpTo(offset)) {  //ӳ��ʧ��
            CreateErrorMsg(ERR_JUMP_GOTO, tmp->GetLineNo());  //�ӳ�����תʧ��
            return false;
        }

        this->m_ln_read_size = offset;
        this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file
                + m_ln_read_size - m_p_file_map_info->ln_map_start;

        this->m_ln_cur_line_no = line_no;

        break;
    case MACRO_CMD_IF_GOTO: //������תָ����Ҫ�ڴ˴��ȴ�
        if (tmp->GetRunStep() == 0) {
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0),
                                                 tmp->GetMacroExpResult(0))) { //���ʽ����ʧ��
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //���ʽ�����쳣
                    return false;
                }

                //û�д�����˵�����ʽ����Ҫ����ϵͳ����������ȴ�MC���е�λ
                return false;
            } else if (static_cast<int>(tmp->GetMacroExpResult(0).value) == 1) { //��������
                tmp->SetRunStep(1);  //���ý���
            } else {  //���������㣬��ִ����ת
                break;
            }
        }
        if (tmp->GetRunStep() == 1) {
            if(!tmp->GetMacroExpCalFlag(1)){
                if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(1),
                                                     tmp->GetMacroExpResult(1))) {  //���ʽ����ʧ��
                    m_error_code = m_p_parser->GetErrorCode();
                    if (m_error_code != ERR_NONE) {
                        CreateErrorMsg(m_error_code, tmp->GetLineNo());  //���ʽ�����쳣
                        return false;
                    }

                    //û�д�����˵�����ʽ����Ҫ����ϵͳ����������ȴ�MC���е�λ
                    return false;
                }
                tmp->SetMacroExpCalFlag(1, true);
            }
            //ִ����ת
            //���ʽ����ɹ�,ִ����ת����
            res = FindJumpLabel(static_cast<int>(tmp->GetMacroExpResult(1).value), offset, line_no);
            printf("jump res = %d, result = %lf, offset= %llu, line=%llu\n", res, tmp->GetMacroExpResult(1).value, offset, line_no);
            if (0 == res) {
                //û���ҵ���ת�㣬�����澯
                CreateErrorMsg(ERR_NO_JUMP_LABEL, tmp->GetLineNo()); //���ʽ�����쳣
                return false;
            }else if(2 == res){
                return false;   //�ȴ�Ԥɨ�����
            }

            //�����ת�Ϸ���
            if(!this->CheckJumpGoto(tmp->GetLineNo(), line_no)){
                CreateErrorMsg(ERR_JUMP_GOTO_ILLIGAL, tmp->GetLineNo());
                return false;
            }

            if (!this->m_p_file_map_info->JumpTo(offset)) {  //ӳ��ʧ��
                CreateErrorMsg(ERR_JUMP_GOTO, tmp->GetLineNo());  //�ӳ�����תʧ��
                return false;
            }

            this->m_ln_read_size = offset;
            this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file
                    + m_ln_read_size - m_p_file_map_info->ln_map_start;

            this->m_ln_cur_line_no = line_no;

        }
        break;
    case MACRO_CMD_IF_THEN:
        if (tmp->GetRunStep() == 0) {
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0),
                                                 exp_res)) {  //���ʽ����ʧ��
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //���ʽ�����쳣
                    return false;
                }

                //û�д�����˵�����ʽ����Ҫ����ϵͳ����������ȴ�MC���е�λ
                return false;
            } else if (static_cast<int>(exp_res.value) == 1) { //��������

                tmp->SetRunStep(1);  //���ý���
            } else {  //���������㣬��ִ��
                break;
            }
        }
        if (tmp->GetRunStep() == 1) {  //���ʽ����ɹ�,ִ��
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(1),
                                                 exp_res)) {  //���ʽ����ʧ��
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //���ʽ�����쳣
                    return false;
                }

                //û�д�����˵�����ʽ����Ҫ����ϵͳ����������ȴ�MC���е�λ
                return false;
            }
        }
        break;
    case MACRO_CMD_WHILE_DO: {
        if (tmp->GetRunStep() == 0) { //�ж�while����
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0),
                                                 exp_res)) { //���ʽ����ʧ��
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //���ʽ�����쳣
                    return false;
                }

                //û�д�����˵�����ʽ����Ҫ����ϵͳ����������ȴ�MC���е�λ
                return false;
            } else if (static_cast<int>(exp_res.value) == 1) { //��������
                //	printf("while ok\n");
                flag = true;
            } else { //���������㣬����ѭ����
                //	printf("while failed, %lf, %hhu", exp_res.value, exp_res.init);
                flag = false;
            }
            tmp->SetRunStep(1);  //���ý���
        }
        if (tmp->GetRunStep() == 1) {
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(1),
                                                 exp_res)) {  //���ʽ����ʧ��
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //���ʽ�����쳣
                    return false;
                }

                //û�д�����˵�����ʽ����Ҫ����ϵͳ����������ȴ�MC���е�λ
                return false;
            } else {
                index = static_cast<int>(exp_res.value);
                if (index >= 0 && index <= kMaxDoIndex) { //��������
                    loop.loop_index = index;
                    loop.line_no = tmp->GetLineNo();
                    loop.offset = tmp->GetOffset();
                    if (flag) {
                        //ѭ��λ��������ջ
                        //						printf("push loop\n");
                        this->m_stack_loop.push(loop);
                        //						printf("do push loop, index = %hhu, line=%lld, offset= %lld\n", loop.loop_index, loop.line_no, loop.offset);
                    } else {
                        printf("jump to end cmd\n");
                        //��ת����Ӧ��ENDָ��
                        if (!this->JumpToLoopEnd(loop)) {
                            CreateErrorMsg(ERR_MACRO_DO_END_MATCH,
                                           tmp->GetLineNo());  //���ʽ�����쳣
                            return false;
                        }
                    }
                } else {  //DOʶ��ų���Χ���澯
                    CreateErrorMsg(ERR_MACRO_DO_OUT, tmp->GetLineNo()); //DOʶ��ų���Χ
                    return false;
                }

            }

        }
    }
        break;
    case MACRO_CMD_END:
        if (tmp->GetRunStep() == 0) {
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0),
                                                 exp_res)) {  //���ʽ����ʧ��
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //���ʽ�����쳣
                    return false;
                }

                //û�д�����˵�����ʽ����Ҫ����ϵͳ����������ȴ�MC���е�λ
                return false;
            } else {
                index = static_cast<int>(exp_res.value);
                if (index >= 0 && index <= kMaxDoIndex) { //��������
                    //����ѭ����ʼ��
                    if (this->m_stack_loop.size() == 0) {
                        CreateErrorMsg(ERR_MACRO_DO_END_MATCH,
                                       tmp->GetLineNo());  //DO-ENDָ�ƥ��
                        return false;
                    }

                    //��ת
                    if (m_stack_loop.pop(loop)) {
                        if (!this->JumpToLoopHead(loop)) {
                            CreateErrorMsg(ERR_JUMP_END, tmp->GetLineNo()); //��תʧ��
                            return false;
                        }
                    }
                } else {  //DO-ENDʶ��ų���Χ���澯
                    CreateErrorMsg(ERR_MACRO_DO_OUT, tmp->GetLineNo()); //DO-ENDʶ��ų���Χ
                    return false;
                }
            }
        }
        break;
    case MACRO_CMD_DO:  //ʡ��WHILE������ѭ��
        if (tmp->GetRunStep() == 0) {
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0),
                                                 exp_res)) {  //���ʽ����ʧ��
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //���ʽ�����쳣
                    return false;
                }

                //û�д�����˵�����ʽ����Ҫ����ϵͳ����������ȴ�MC���е�λ
                return false;
            } else {
                index = static_cast<int>(exp_res.value);
                if (index >= 0 && index <= kMaxDoIndex) { //��������
                    //ѭ��λ��������ջ
                    loop.loop_index = index;
                    loop.line_no = tmp->GetLineNo();
                    loop.offset = tmp->GetOffset();
                    this->m_stack_loop.push(loop);
                } else {					//DOʶ��ų���Χ���澯
                    CreateErrorMsg(ERR_MACRO_DO_OUT, tmp->GetLineNo()); //DOʶ��ų���Χ
                    return false;
                }
            }
        }
        break;
    case MACRO_CMD_EXP:		//������ʽ
        if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0), exp_res)) {//���ʽ����ʧ��
            m_error_code = m_p_parser->GetErrorCode();
            if (m_error_code != ERR_NONE) {
                CreateErrorMsg(m_error_code, tmp->GetLineNo());  //���ʽ�����쳣
                return false;
            }

            //û�д�����˵�����ʽ����Ҫ����ϵͳ����������ȴ�MC���е�λ
            return false;
        }
        break;
    case MACRO_CMD_IF:{
    	//@test
    	printf("===== IF CMD %llu\n", tmp->GetLineNo());
        if(!tmp->GetMacroExpCalFlag(0)){
            if(!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0), tmp->GetMacroExpResult(0))) {//���ʽ����ʧ��
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //���ʽ�����쳣
                    return false;
                }
                //û�д�����˵�����ʽ����Ҫ����ϵͳ����������ȴ�MC���е�λ
                return false;
            }

            tmp->SetMacroExpCalFlag(0, true);


            IfElseOffset node;
            //printf("===== node vectors vector size: %d\n", m_node_vectors_vector.size());

            for(vector<IfElseOffset> node_vector : m_node_vectors_vector){
            	// @test
                //printf("===== node lino: %llu, tmp lino: %llu\n", node_vector.at(0).line_no, tmp->GetLineNo());

            	if(node_vector.at(0).line_no == tmp->GetLineNo()){
            		printf("===== node lino: %llu, tmp lino: %llu\n", node_vector.at(0).line_no, tmp->GetLineNo());
            		node = node_vector.at(0);
                }
            }

            /*************************************************/
            if(node.line_no == 0){
                /*** test if else */
                printf("-------------------------------------\n");

                int count = 1;
                for(vector<IfElseOffset> node_vector : m_node_vectors_vector){
                    printf("node num: %d\n", count);
                    for(IfElseOffset node : node_vector){
                        printf("lino: %lld --", node.line_no);
                    }
                    printf("\n");
                    count ++;
                }
                return false;
            }
            /****************************************************/

            if(static_cast<int>(tmp->GetMacroExpResult(0).value) == 1){
                // �������� ��ת�� node vector �� back��endif�ڵ㣩
                node = m_node_vectors_vector.at(node.vec_index).back();
                m_b_else_jump = true;                     // ���� else elseif ��ת����Ӧ��endif��

            }else{
                //����������     ����һ���ڵ���ת
                node = m_node_vectors_vector.at(node.vec_index).at(node.node_index + 1);
                m_b_else_jump = false;

                if(!JumpLine(node.line_no, node.offset, tmp))
                    printf("------ jump line failed!!!\n");
            }

            m_node_stack_run.push_back(node);
            printf("===== push back size: %d\n", m_node_stack_run.size());
            m_else_jump_stack_run.push_back(m_b_else_jump);
        }
        break;
    }
    case MACRO_CMD_ELSEIF:{

    	//@test
		printf("===== ELSEIF CMD %llu\n", tmp->GetLineNo());

    	if(m_node_stack_run.size() == 0){
            printf("IF ELSE ��ƥ��, û��if��ջ������  else/ elseif\n");
            CreateErrorMsg(IF_ELSE_MATCH_FAILED, tmp->GetLineNo());
            return false;
        }

        IfElseOffset node;
        node = m_node_stack_run.back();

        if(node.line_no == 0){
            printf("m_stack_ifelse_node ջ��������\n");
            break;
        }

        // ֮ǰ�� if �Ѿ���������  ֱ����ת�� endif
        if(m_b_else_jump){

            if(!JumpLine(node.line_no, node.offset, tmp))
                printf("------ jump line failed!!!\n");

            break;
        }

        if(!tmp->GetMacroExpCalFlag(0)){
            if(!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0), tmp->GetMacroExpResult(0))) {//���ʽ����ʧ��
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //���ʽ�����쳣
                    return false;
                }

                //û�д�����˵�����ʽ����Ҫ����ϵͳ����������ȴ�MC���е�λ
                return false;
            }
            tmp->SetMacroExpCalFlag(0, true);

            // ��������
            if(static_cast<int>(tmp->GetMacroExpResult(0).value) == 1){
                //printf("cmd  elseif: 1111\n");
                m_b_else_jump = true;
                m_else_jump_stack_run.pop_back();
                m_else_jump_stack_run.push_back(m_b_else_jump);
                // �ҵ� node_vector endif �ڵ�
                node = m_node_vectors_vector.at(node.vec_index).back();
                m_node_stack_run.pop_back();
                m_node_stack_run.push_back(node);

                // ����������
            }else{
                node = m_node_vectors_vector.at(node.vec_index).at(node.node_index + 1);
                m_node_stack_run.pop_back();
                m_node_stack_run.push_back(node);

                if(!JumpLine(node.line_no, node.offset, tmp))
                    printf("------ jump line failed!!!\n");
            }
        }

        break;
    }
    case MACRO_CMD_ELSE:{

    	//@test
		printf("===== ELSE CMD %llu\n", tmp->GetLineNo());

    	if(m_node_stack_run.size() == 0){
            printf("IF ELSE ��ƥ��, û��if��ջ������  else/ elseif 222\n");
            CreateErrorMsg(IF_ELSE_MATCH_FAILED, tmp->GetLineNo());
            return false;
        }

        if(m_b_else_jump){
            IfElseOffset node;
            node = m_node_stack_run.back();
            if(node.line_no == 0){
                printf("m_stack_ifelse_node ջ��������\n");
                break;
            }

            if(!JumpLine(node.line_no, node.offset, tmp))
                printf("------ jump line failed!!!\n");

            break;
        }
        break;
    }
    case MACRO_CMD_ENDIF:{

    	//@test
		printf("===== ENDIF CMD %llu\n", tmp->GetLineNo());

    	if(m_node_stack_run.size() == 0){
            printf("IF ELSE ��ƥ��, û��if��ջ������  endif \n");
            CreateErrorMsg(IF_ELSE_MATCH_FAILED, tmp->GetLineNo());
            return false;
        }

    	m_node_stack_run.pop_back();
        m_else_jump_stack_run.pop_back();
        m_b_else_jump = m_else_jump_stack_run.back();
        break;
    }
    default:
        printf("@@@@@@ERR_NC_FORMAT_7\n");
        CreateErrorMsg(ERR_NC_FORMAT, tmp->GetLineNo());  //�޷������ĺ�ָ��
        return false;
    }

    return true;
}

bool Compiler::JumpLine(int line_no, uint64_t offset, MacroCmdMsg *tmp){
    //�����ת�Ϸ���
    if(!this->CheckJumpGoto(tmp->GetLineNo(), line_no)){
        CreateErrorMsg(ERR_JUMP_GOTO_ILLIGAL, tmp->GetLineNo());
        return false;
    }

    if (!this->m_p_file_map_info->JumpTo(offset)) {  //ӳ��ʧ��
        CreateErrorMsg(ERR_JUMP_GOTO, tmp->GetLineNo());  //�ӳ�����תʧ��
        return false;
    }

    this->m_ln_read_size = offset;
    this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file
            + m_ln_read_size - m_p_file_map_info->ln_map_start;

    this->m_ln_cur_line_no = line_no;
    return true;
}

/**
 * @brief ִ�е���ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunToolMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    ToolMsg *tmp = (ToolMsg *) msg;


#ifdef USES_T_CODE_MACRO
    if (!this->m_b_prescan_over && m_thread_prescan != 0) //Ԥɨ��δ�������ݲ�ִ��
        return false;

    int sub_index = tmp->GetSubProgIndex();   //Tָ���Ӧ����O9000.nc�ӳ���

    //�����ӳ���
    int sub_loc = this->FindSubProgram(sub_index);
    tmp->SetSubProgType(sub_loc);
    if (sub_loc == 0) {
        CreateErrorMsg(ERR_NO_SUB_PROG, msg->GetLineNo());  //�Ҳ�����Ӧ�ӳ���
        return false;
    }

    //���浱ǰ������״̬
    this->SaveScene();

    //�ֲ�������ջ
    m_p_channel_control->GetMacroVar()->PushLocalVar();

    this->m_n_sub_call_times = 1;//���ô���

    //��ֵT�������ֲ�����#20��ȫ�ֱ���#146
    int t_data = tmp->GetTool();
    m_p_channel_control->GetMacroVar()->SetVarValue(kMacroParamToLocalVarIndex[T_DATA], t_data);
    m_p_channel_control->GetMacroVar()->SetVarValue(146, t_data);



    //���ӳ����ļ�
    this->m_n_sub_program = MACRO_PROG;
    char filepath[kMaxPathLen] = { 0 };   //�ļ�·��
    if (sub_loc == 1) {   //ͬ�ļ��ӳ���
        ListNode < SubProgOffset > *node = m_p_list_subprog->HeadNode();
        uint64_t offset_sub = 0;
        while (node != nullptr) {
            if (node->data.sub_index == sub_index) {
                offset_sub = node->data.offset;
                break;
            }
            node = node->next;
        }

        if (!this->m_p_file_map_info->JumpTo(offset_sub)) {   //ӳ��ʧ��
            CreateErrorMsg(ERR_JUMP_SUB_PROG, msg->GetLineNo());  //�ӳ�����תʧ��
            return false;
        }

        this->m_ln_read_size = offset_sub;
        this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file
                + m_ln_read_size - m_p_file_map_info->ln_map_start;

        this->m_ln_cur_line_no = node->data.line_no;
    } else {
        tmp->GetSubProgName(filepath, true);


        //		tmp->SetLastProgFile(this->m_p_file_map_info->str_file_name+strlen(PATH_NC_FILE));  // ���浱ǰ�ļ�·�������·��
        //�������ļ���
        if (!this->OpenFile(filepath, (bool)m_n_sub_program)){ //���Դ�nc�ļ�ʧ��
            return false;
        }

    }

    if (m_work_mode == AUTO_COMPILER)
        this->m_n_compile_state = FILE_HEAD;
    else
        this->m_n_compile_state = FILE_MAIN;
    this->m_n_head_state = HEAD_INFO;
#endif

    m_compiler_status.mode.t_mode = tmp->GetTool(0);

    return true;
}

/**
 * @brief ִ��ѭ��ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunLoopMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;

    LoopMsg *sub_msg = (LoopMsg *) msg;
    int gcode = sub_msg->GetGCode();

    //TODO  ����ѭ��ָ���ӳ���
    if (!this->m_b_prescan_over && m_thread_prescan != 0){ //Ԥɨ��δ�������ݲ�ִ��
        return false;
    }

    int macro_index = sub_msg->GetMacroProgIndex();

    //�����ӳ���
    int macro_loc = this->FindSubProgram(macro_index);
    sub_msg->SetMacroProgType(macro_loc);
    if (macro_loc == 0) {
        CreateErrorMsg(ERR_INVALID_CODE, msg->GetLineNo());  //�Ҳ�����Ӧ�ӳ���
        return false;
    }

    if(gcode != G80_CMD)
        this->SaveLoopParam(sub_msg);  //���������ȫ�ֱ���
    else
        this->ResetLoopParam();   //G80��λ����

    m_compiler_status.mode.gmode[GetModeGroup(gcode)] = gcode;   //�޸ı�����Gģ̬

    //���浱ǰ������״̬
    this->SaveScene();

    Variable *pv = m_p_channel_control->GetMacroVar();
    //�ֲ�������ջ

    // @test zk  ����ǰ�ֲ������浽scene��  ���㵱ǰ�ֲ�������
    pv->PushLocalVar();


    //��ֵ�Ա����������ֲ�����
    uint8_t pc = 0;
    uint32_t pm = 0;
    int i  = 0;
    double *pp = sub_msg->GetParameter(pm, pc);
    //����Ƿ���F������û�еĻ�������ǰ��Feedֵ����F����
    if((pm & (0x01<<F_DATA)) == 0){  //û��F��������δָ��Feed
        pv->SetVarValue(kLoopParamToLocalVarIndex[F_DATA], m_compiler_status.mode.f_mode);
    }

    //д����Ե����������
    uint8_t chn_axis_count = this->m_p_channel_control->GetChnAxisCount();
    uint8_t chn_axis_data = 0;

    // @zk ��һ���Ὣ��ǰ����ֵ���浽��Ӧ�ľֲ����� (ָ�� X_ Y_ Z_ ...)
    for(uint8_t i = 0; i < chn_axis_count; i++){
        chn_axis_data = this->MapAxisNameToParamIdx(m_p_channel_control->GetChnAxisName(i));

        if((pm & (0x01<<chn_axis_data)) == 0){
            pv->SetVarValue(kLoopParamToLocalVarIndex[chn_axis_data], m_compiler_status.cur_pos.GetAxisValue(i));
        }
    }
    // @zk ���̶�ѭ��ָ��ָ���Ĳ��� ���浽�ֲ����� (���� X_ Y_ Z_ ...)
    while(pm != 0){
        if(pm & 0x01){
            pv->SetVarValue(kLoopParamToLocalVarIndex[i], *pp);
            pp++;
        }
        pm = pm>>1;
        i++;
    }

    this->m_n_sub_call_times = 1;//���ô���

    //���ӳ����ļ�
    this->m_n_sub_program = MACRO_PROG;
    char filepath[kMaxPathLen] = { 0 };   //�ļ�·��

    sub_msg->GetMacroProgName(filepath, true);

    sub_msg->SetLastProgFile(this->m_p_file_map_info->str_file_name+strlen(PATH_NC_FILE));  //���浱ǰ�ļ�·��, ���·�������͸�HMI
    //�������ļ���
    if (!this->OpenFile(filepath, (bool)m_n_sub_program)){ //���Դ�nc�ļ�ʧ��
        return false;
    }


    if (m_work_mode == AUTO_COMPILER)
        this->m_n_compile_state = FILE_HEAD;
    else
        this->m_n_compile_state = FILE_MAIN;
    this->m_n_head_state = HEAD_INFO;

    return true;
}

/**
 * @brief ִ�д���ָ����Ϣ
 * @param msg : ��Ϣָ��
 * @return
 */
bool Compiler::RunErrorMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    ErrorMsg *tmp = static_cast<ErrorMsg *>(msg);

    //
    if(tmp->GetInfoType() == 1){  //������Ϣ
        this->m_error_code = (ErrorType)tmp->GetErrorCode();
    }
    printf("run error message! errcode = %d, infotype = %d, line = %lld\n", m_error_code, tmp->GetInfoType(), tmp->GetLineNo());

    return true;
}

/**
 * @brief ������������Ϣ����������Ϣ����
 * @param err : ������
 * @param line_no : �����к�
 * @return true--�ɹ�  false--ʧ��
 */
bool Compiler::CreateErrorMsg(const ErrorType err, uint64_t line_no) {
    RecordMsg *new_msg = new ErrorMsg(err);
    if (new_msg == nullptr) {
        //TODO �ڴ����ʧ�ܣ��澯
        m_error_code = ERR_MEMORY_NEW;
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]�������澯�����ڴ�ʧ�ܣ�",
                              m_n_channel_index);
        CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER, 0,
                    m_n_channel_index);
        return false;
    }
    new_msg->SetLineNo(line_no);  //���õ�ǰ�к�
    m_error_code = err;

    //TODO  ֱ��������һ����
    return this->m_p_block_msg_list->Append(new_msg);
}

/**
 * @brief �������ͷ����Ϣ
 * @return ִ�н��
 */
bool Compiler::ProcessHead() {
    bool res = true;

    if (this->m_work_mode == MDA_COMPILER) {
        this->m_n_compile_state = FILE_MAIN;
        return res;
    }

    switch (m_n_head_state) {
    case HEAD_INFO:
        res = CompileHead();
        break;

        //  case SUBPROGRAM:
        //      compSubprogram();
        //      break;

    default:
        break;
    }

    if (m_error_code != ERR_NONE) {
        //CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
        CreateErrorMsg(m_error_code, this->m_ln_cur_line_no);
        return true;
    }

    return res;
}

/**
 * @brief ����NC��������
 * @return ִ�н��
 */
bool Compiler::ProcessMain() {
    bool res = true;

    //TODO ��������
    if (this->m_line_buf[0] == '/') {
        //���δ���
        this->m_compiler_status.jump_flag = true;
        int len = strlen(m_line_buf);
        int index = 1;
        for (; index < len; index++) {
            if (!isspace(m_line_buf[index]))  //�ҵ���һ���ǿո��ַ�
                break;
        }
        if (index < len)
            strcpy(m_line_buf, m_line_buf + index);
    }

    //���Ƚ��дʷ�����
    if (!DoLexer()) {
        res = false;
    } else {
        //�����﷨����
        if (!DoParser()) {
            res = false;
        }
    }

    return res;
}

/**
 * @brief ����ͷ����Ϣ
 * @return ִ�н��
 */
bool Compiler::CompileHead() {
    bool res = true;
    //  const char* str = "\x20:,";

    res = GetHeadInfo();  //���Ȼ�ȡͷ��������Ϣ
    if (!res) {
        return res;
    }

    res = CheckHead();
    if (res) {
        m_n_compile_state = FILE_MAIN;  //�л�������������״̬
    }

    return res;
}

/**
 * @brief �����������
 * @return ִ�н��
 */
//bool Compiler::CompileMain(){
//	bool res = true;
//
//
//
//	return res;
//}


/**
 * @brief ������ͷ���Ϸ���
 * @return ִ�н��
 */
bool Compiler::CheckHead() {
    bool res = false;
    char *compBuf = m_line_buf;

    if (strcmp(compBuf, "%") == 0) {
        res = true; //������
    }else if(m_n_sub_program != MAIN_PROG){  //���ӳ����У�������%
        res = this->ProcessMain(); //
    }else {
#ifdef USES_WOOD_MACHINE
        res = this->ProcessMain();
#else
        if (m_b_check) {
            m_error_code = ERR_NO_START;   //����ʼ%
        }
#endif

    }

    return res;
}

/**
 * @brief ������ȡͷ���ĸ�����Ϣ��������·�ϲ���Ϣ
 * @return ִ�н��
 */
bool Compiler::GetHeadInfo() {
    bool res = true;
    //	char* compBuf = m_line_buf;

    //���ӻ��������л�ͷ����Ϣ����
    //	char *pp = compBuf;

    //	if(*pp == '@')
    //	{
    //		char *p1 = pp;
    //		toolLine[nToolCount++] = strtol(++p1, &pp, 10);
    //		if(toolLine[nToolCount-1] <= 0 || pp == p1)
    //		{
    //			if (m_b_check)
    //			{
    //				m_error_code = ERR_NC_FORMAT;
    //				return false;
    //			}
    //		}
    //		return true;
    //	}
    //	else if(*pp == '$')
    //	{
    //		char *p1 = pp;
    //		coordLine[nCoordCount++] = strtol(++p1, &pp, 10);
    //		if(coordLine[nCoordCount-1]<= 0 || pp == p1)
    //		{
    //			if (m_b_check)
    //			{
    //				m_error_code = ERR_NC_FORMAT;
    //				return false;
    //			}
    //		}
    //		return true;
    //	}
    //	else if(*pp == '*')
    //	{
    //		char *p1 = pp;
    //		long ln = strtol(++p1, &pp, 10);
    //		if(ln<= 0 || pp == p1)
    //		{
    //			if (m_b_check)
    //			{
    //				m_error_code = ERR_NC_FORMAT;
    //				return false;
    //			}
    //		}
    //		else
    //			nFileEndLine = ln;
    //		return true;
    //	}

    return res;
}

/**
 * @brief ���дʷ�����
 * @return ִ�н��
 */
bool Compiler::DoLexer() {
    bool res = true;

    res = m_p_lexer->Compile();
    if (!res) {
        printf("lexer error\n");
        m_error_code = ERR_COMPILER_INTER;  //�������ڲ�����
    }
    return res;
}

/**
 * @brief �����﷨����
 * @return ִ�н��
 */
bool Compiler::DoParser() {
    bool res = true;

    res = m_p_parser->Compile();
    if (!res) { //�������﷨����
        m_error_code = ERR_COMPILER_INTER;  //�������ڲ�����
    }
    return res;
}

/**
 * @brief �����ӳ���
 * @param sub_name : �ӳ����
 * @return  true--�ɹ�   false--ʧ��
 */
//bool Compiler::CallSubProgram(int sub_name){
//
//
//	return true;
//}

/**
 * @brief �޲������ú����
 * @param macro_index : �������
 * @param flag : ֪ͨHMI�ļ������ true--֪ͨ   false--��֪ͨ
 * @return 0--����ʧ��   ����--���óɹ�
 */
int Compiler::CallMarcoProgWithNoPara(int macro_index, bool flag){
    //�����ӳ���
    int macro_loc = this->FindSubProgram(macro_index);

    if (macro_loc == 0 || macro_loc == 1) {
        printf("create error in CallMarcoProgWithNoPara\n");
        CreateErrorMsg(ERR_NO_SUB_PROG, 0);  //�Ҳ�����Ӧ�ӳ���
        return 0;
    }

    //���浱ǰ������״̬
    this->SaveScene();

    Variable *pv = m_p_channel_control->GetMacroVar();
    //�ֲ�������ջ
    pv->PushLocalVar();

    this->m_n_sub_call_times = 1;//���ô���

    //���ӳ����ļ�
    this->m_n_sub_program = MACRO_PROG;
    char filepath[kMaxPathLen] = { 0 };   //�ļ�·��
    GetMacroSubProgPath(macro_loc, macro_index, true, filepath);

    //�������ļ���
    if (!this->OpenFile(filepath, (bool)m_n_sub_program)){ //���Դ�nc�ļ�ʧ��
        return 0;
    }

    if (m_work_mode == AUTO_COMPILER)
        this->m_n_compile_state = FILE_HEAD;
    else
        this->m_n_compile_state = FILE_MAIN;
    this->m_n_head_state = HEAD_INFO;

    //TODO ��HMI������������ļ�
    if(flag && g_ptr_parm_manager->GetSystemConfig()->debug_mode > 0){ //��ʽģʽ�£��򿪺�����ļ�
        this->m_p_channel_control->SendOpenFileCmdToHmi(filepath);
    }

    return macro_loc;
}

/**
 * @brief ��ȡ������·������
 * @param macro_group : ��������ͣ�1--�ڱ�������   2/6--ͬĿ¼��nc�ļ�    3/7--ϵͳ�ӳ���Ŀ¼nc�ļ�    4/8--ͬĿ¼��iso�ļ�    5/9--ϵͳ�ӳ���Ŀ¼iso�ļ�
 * @param macro_index : ������
 * @param abs_path : �Ƿ����·��
 * @param name[out] : ���������ļ�·��
 */
void Compiler::GetMacroSubProgPath(int macro_group, int macro_index, bool abs_path, char *name){
    if(abs_path){
        if(macro_group == 2){//ͬĿ¼���û������
            sprintf(name, "%sO%04d.nc", PATH_NC_FILE, macro_index);   //ƴ���ļ�����
        }else if(macro_group == 3){//ϵͳ�����
            sprintf(name, "%ssys_sub/O%04d.nc", PATH_NC_FILE, macro_index);
        }else if(macro_group == 4){
            sprintf(name, "%sO%04d.iso", PATH_NC_FILE, macro_index);   //ƴ���ļ�����
        }else if(macro_group == 5){
            sprintf(name, "%ssys_sub/O%04d.iso", PATH_NC_FILE, macro_index);
        }else if(macro_group == 6){//ͬĿ¼���û������
            sprintf(name, "%sO%04d.NC", PATH_NC_FILE, macro_index);   //ƴ���ļ�����
        }else if(macro_group == 7){//ϵͳ�����
            sprintf(name, "%ssys_sub/O%04d.NC", PATH_NC_FILE, macro_index);
        }else if(macro_group == 8){
            sprintf(name, "%sO%04d.ISO", PATH_NC_FILE, macro_index);   //ƴ���ļ�����
        }else if(macro_group == 9){
            sprintf(name, "%ssys_sub/O%04d.ISO", PATH_NC_FILE, macro_index);
        }
    }else{
        if(macro_group == 2){//ͬĿ¼���û������
            sprintf(name, "O%04d.nc", macro_index);   //ƴ���ļ�����
        }else if(macro_group == 3){//ϵͳ�����
            sprintf(name, "sys_sub/O%04d.nc", macro_index);
        }else if(macro_group == 4){
            sprintf(name, "O%04d.iso", macro_index);   //ƴ���ļ�����
        }else if(macro_group == 5){
            sprintf(name, "sys_sub/O%04d.iso", macro_index);
        }else 	if(macro_group == 6){//ͬĿ¼���û������
            sprintf(name, "O%04d.NC", macro_index);   //ƴ���ļ�����
        }else if(macro_group == 7){//ϵͳ�����
            sprintf(name, "sys_sub/O%04d.NC", macro_index);
        }else if(macro_group == 8){
            sprintf(name, "O%04d.ISO", macro_index);   //ƴ���ļ�����
        }else if(macro_group == 9){
            sprintf(name, "sys_sub/O%04d.ISO", macro_index);
        }
    }
}


/**
 * @brief �ӳ��򷵻ش���
 * @return  true--�ɹ�   false--ʧ��
 */
bool Compiler::ReturnFromSubProg() {
    printf("Return from sub program, sub_prog=%d, call_time=%d\n", m_n_sub_program, m_n_sub_call_times);
    if (this->m_n_sub_program != MAIN_PROG) {

        if(--m_n_sub_call_times > 0){  //��ε���δ����
            this->RecycleCompile();
            return true;
        }

        //Ԥɨ���߳��Ƿ������δ�������˳�
        void* thread_result;
        int res = 0;
        if (!m_b_prescan_over && m_thread_prescan != 0) {//Ԥɨ���߳�δ�������˳����ȴ������
            //���˳�֮ǰ���߳�
            m_b_breakout_prescan = true;
            int wait_count = 0;

            while (m_b_breakout_prescan && wait_count++ < 200)
                usleep(1000);  //�ȴ��߳��˳�
            if (m_b_breakout_prescan) {//�ȴ��˳�ʧ�ܣ�������cancel�߳�

                //�˳�Ԥɨ�������߳�
                res = pthread_cancel(m_thread_prescan);
                if (res != 0) {
                    g_ptr_trace->PrintLog(LOG_ALARM, "Ԥɨ���߳��˳�ʧ��3��errno = %d\n",
                                          res);
                    printf("Ԥɨ���߳��˳�ʧ��7777\n");
                    CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                                0, m_n_channel_index);
                    return false;
                }

                usleep(1000);
            }
            res = pthread_join(m_thread_prescan, &thread_result);  //�ȴ��߳��˳����
            if (res != 0) {
                g_ptr_trace->PrintLog(LOG_ALARM, "Ԥɨ���߳��˳�ʧ��4��errno = %d\n",
                                      res);
                printf("Ԥɨ���߳��˳�ʧ��8888\n");
                CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                            0, m_n_channel_index);
                return false;
            }
            m_thread_prescan = 0;

        }else if(m_b_prescan_over && m_thread_prescan != 0){  //�ͷ�Ԥɨ���߳���Դ
            res = pthread_join(m_thread_prescan, &thread_result);  //�ȴ��߳��˳����
            if (res != 0) {
                g_ptr_trace->PrintLog(LOG_ALARM, "Ԥɨ���߳���Դ�ͷ�ʧ��2��errno = %d\n",
                                      res);
                printf("Ԥɨ���߳��˳�ʧ��9999\n");
                CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                            0, m_n_channel_index);
            }
            m_thread_prescan = 0;
        }
        m_b_breakout_prescan = false;
        m_b_prescan_over = false;

        printf("exit pre scan thread\n");

        bool ret_macro_prog = (m_n_sub_program==MACRO_PROG)?true:false;    //���صĵ�������

        //�ֲ�������ջ
        // @test �ָ�ջ���ֲ�����
        m_p_channel_control->GetMacroVar()->PopLocalVar();

        //�ӳ���������ָ��ֳ�
        if (!this->ReloadScene(false)) {
            g_ptr_trace->PrintLog(LOG_ALARM, "�ӳ���������ָ��ֳ�ʧ�ܣ�");
            //TODO ����
            CreateError(ERR_SUB_BACK, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                        m_n_channel_index);
            return false;
        }

        this->m_p_cur_file_pos = this->m_p_file_map_info->ptr_map_file
                + this->m_ln_read_size - m_p_file_map_info->ln_map_start;


        //���ӳ��򷵻أ������ӳ��򷵻���Ϣ����Ҫ������֪ͨHMI����һ�������ļ�
        char file[kMaxFileNameLen];
        memset(file, 0x00, kMaxFileNameLen);
        strcpy(file, m_p_file_map_info->str_file_name);
        char *p = strrchr(file, '/');
        if(p != nullptr){
            strcpy(file, p+1);
        }
        SubProgReturnMsg *msg = new SubProgReturnMsg(file, ret_macro_prog);
        msg->SetLineNo(this->m_ln_cur_line_no-1);
        m_p_block_msg_list->Append(msg);
        printf("@@@@@@insert block subprogret msg : %s, line=%lld\n", file, msg->GetLineNo());
    }

    return true;
}

/**
 * @brief �����ӳ���,�ӳ���Ĳ���˳��1.��������  2. NC����Ŀ¼  3. ϵͳ�ӳ���Ŀ¼
 * @param sub_name : �ӳ����
 * @return 0--û�ҵ���Ӧ�ӳ���   1--�ڱ�������   2--ͬĿ¼��nc�ļ�    3--ϵͳ�ӳ���Ŀ¼nc�ļ�    4--ͬĿ¼��iso�ļ�    5--ϵͳ�ӳ���Ŀ¼iso�ļ�
 *                                               6--ͬĿ¼��NC�ļ�    7--ϵͳ�ӳ���Ŀ¼NC�ļ�    8--ͬĿ¼��ISO�ļ�    9--ϵͳ�ӳ���Ŀ¼ISO�ļ�
 */
int Compiler::FindSubProgram(int sub_name, bool file_only) {   //���Ҳ����ӳ���

    //�ڳ����ڲ�����
    if(!file_only){
        ListNode < SubProgOffset > *node = m_p_list_subprog->HeadNode();
        while (node != nullptr) {
            if (node->data.sub_index == sub_name) {
                return 1;
            }
            node = node->next;
        }
    }

    //ͬĿ¼������
    //nc�ļ�
    char filepath[kMaxPathLen] = { 0 };   //�ļ�·��
    if (sub_name <= 9999)
        sprintf(filepath, "%sO%04d.nc", PATH_NC_FILE, sub_name);   //ƴ���ļ�����·��
    else
        sprintf(filepath, "%sO%d.nc", PATH_NC_FILE, sub_name);   //ƴ���ļ�����·��

    printf("sub program file 1: %s\n", filepath);
    if (access(filepath, F_OK) == 0) {	//���ڶ�Ӧ�ļ�
        return 2;
    }else{//��׺��д
        memset(filepath, 0x00, kMaxPathLen);
        if (sub_name <= 9999)
            sprintf(filepath, "%sO%04d.NC", PATH_NC_FILE, sub_name);   //ƴ���ļ�����·��
        else
            sprintf(filepath, "%sO%d.NC", PATH_NC_FILE, sub_name);   //ƴ���ļ�����·��

        if (access(filepath, F_OK) == 0) {	//���ڶ�Ӧ�ļ�
            return 6;
        }
    }

    //iso�ļ�
    memset(filepath, 0x00, kMaxPathLen);
    if (sub_name <= 9999)
        sprintf(filepath, "%sO%04d.iso", PATH_NC_FILE, sub_name);   //ƴ���ļ�����·��
    else
        sprintf(filepath, "%sO%d.iso", PATH_NC_FILE, sub_name);   //ƴ���ļ�����·��

    printf("sub program file 2: %s\n", filepath);
    if (access(filepath, F_OK) == 0) {	//���ڶ�Ӧ�ļ�
        return 4;
    }else{
        memset(filepath, 0x00, kMaxPathLen);
        if (sub_name <= 9999)
            sprintf(filepath, "%sO%04d.ISO", PATH_NC_FILE, sub_name);   //ƴ���ļ�����·��
        else
            sprintf(filepath, "%sO%d.ISO", PATH_NC_FILE, sub_name);   //ƴ���ļ�����·��

        if (access(filepath, F_OK) == 0) {	//���ڶ�Ӧ�ļ�
            return 8;
        }
    }

    //ϵͳ�ӳ���Ŀ¼������
    memset(filepath, 0x00, kMaxPathLen);
    if (sub_name <= 9999)
        sprintf(filepath, "%sO%04d.nc", PATH_NC_SUB_FILE, sub_name);  //ƴ���ļ�����·��
    else
        sprintf(filepath, "%sO%d.nc", PATH_NC_SUB_FILE, sub_name);   //ƴ���ļ�����·��
    printf("sys sub program file 3: %s\n", filepath);
    if (access(filepath, F_OK) == 0) {	//���ڶ�Ӧ�ļ�
        return 3;
    }else{
        memset(filepath, 0x00, kMaxPathLen);
        if (sub_name <= 9999)
            sprintf(filepath, "%sO%04d.NC", PATH_NC_SUB_FILE, sub_name);  //ƴ���ļ�����·��
        else
            sprintf(filepath, "%sO%d.NC", PATH_NC_SUB_FILE, sub_name);   //ƴ���ļ�����·��

        if (access(filepath, F_OK) == 0) {	//���ڶ�Ӧ�ļ�
            return 7;
        }
    }

    //iso�ļ�
    memset(filepath, 0x00, kMaxPathLen);
    if (sub_name <= 9999)
        sprintf(filepath, "%sO%04d.iso", PATH_NC_SUB_FILE, sub_name);  //ƴ���ļ�����·��
    else
        sprintf(filepath, "%sO%d.iso", PATH_NC_SUB_FILE, sub_name);   //ƴ���ļ�����·��
    printf("sys sub program file 4: %s\n", filepath);
    if (access(filepath, F_OK) == 0) {	//���ڶ�Ӧ�ļ�
        return 5;
    }else{
        memset(filepath, 0x00, kMaxPathLen);
        if (sub_name <= 9999)
            sprintf(filepath, "%sO%04d.ISO", PATH_NC_SUB_FILE, sub_name);  //ƴ���ļ�����·��
        else
            sprintf(filepath, "%sO%d.ISO", PATH_NC_SUB_FILE, sub_name);   //ƴ���ļ�����·��

        if (access(filepath, F_OK) == 0) {	//���ڶ�Ӧ�ļ�
            return 9;
        }
    }

    return 0;
}

/**
 * @brief ������ת��λ��
 * @param label[in] : ��ת��˳���
 * @param offset[out] : ��ת��ƫ��
 * @param line_no[out] : ��ת���к�
 * @return 0--û�ҵ�    1--�ҵ�   2--�ȴ�Ԥɨ�����
 */
int Compiler::FindJumpLabel(int label, uint64_t &offset, uint64_t &line_no) {
    //������ת��λ��
    int res = 2;

    ListNode < LabelOffset > *node = m_p_list_label->HeadNode();
    while (node != nullptr) {
        if (node->data.label == label) {
            offset = node->data.offset;
            line_no = node->data.line_no;
            res = 1;
            break;
        }
        node = node->next;
    }

    if(res != 1 && this->m_b_prescan_over)  //Ԥɨ���ѽ���������δ�ҵ���ת��
        res = 0;

    return res;
}

/**
 * @brief ������ת��ѭ��ͷ��λ��
 * @param loop : ѭ��ƫ��λ��
 * @return
 */
bool Compiler::JumpToLoopHead(LoopOffset &loop) {
    if (!this->m_p_file_map_info->JumpTo(loop.offset)) {	//ӳ��ʧ��
        return false;
    }

    this->m_ln_read_size = loop.offset;
    this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file + m_ln_read_size
            - m_p_file_map_info->ln_map_start;

    this->m_ln_cur_line_no = loop.line_no;

    return true;
}

/**
 * @brief ������ת��ѭ��ĩβ
 * @param loop : ѭ��ƫ��λ��
 * @return
 */
bool Compiler::JumpToLoopEnd(LoopOffset &loop) {
    bool flag = false;
    int idx = 0;
    MacroVarValue exp_res;
    LoopOffsetStack stack_loop;   //����ѭ��Ƕ�ף��ݴ�Ƕ�׵�WHILE_DO ����DO
    LoopOffset lt;

    stack_loop.empty();
    while (this->GetLineData()) {
        if (strcasestr(this->m_line_buf, "DO") != nullptr) {	//��DOָ��
            if(DoLexer()){//�ʷ������ɹ�
                if (m_lexer_result.macro_flag){//����ʽ
                    if(m_lexer_result.nc_code.macro_cmd.cmd == MACRO_CMD_WHILE_DO){
                        idx = 1;
                    }else if(m_lexer_result.nc_code.macro_cmd.cmd == MACRO_CMD_DO) {
                        idx = 0;
                    }
                    if (m_p_parser->GetExpressionResult(
                                m_lexer_result.nc_code.macro_cmd.macro_expression[idx],
                                exp_res)) {   //���ʽ����ʧ��
                        lt.loop_index = static_cast<int>(exp_res.value);
                        stack_loop.push(lt);  //��ջ
                    }else
                        break;
                }
            }
            else {
                break;
            }
        }else if (strcasestr(this->m_line_buf, "END") != nullptr) {	//��ENDָ��
            if (DoLexer()) {   //�ʷ������ɹ�
                if (m_lexer_result.macro_flag
                        && m_lexer_result.nc_code.macro_cmd.cmd
                        == MACRO_CMD_END) {
                    if (m_p_parser->GetExpressionResult(
                                m_lexer_result.nc_code.macro_cmd.macro_expression[0],
                                exp_res)) {
                        if(stack_loop.size() > 0){//��Ƕ�׵�ѭ��ָ��
                            if(stack_loop.pop(lt)){
                                if(lt.loop_index != static_cast<int>(exp_res.value)){  //DO-END��ƥ��
                                    g_ptr_trace->PrintLog(LOG_ALARM, "ENDָ��˳��Ų�ƥ�䣡");
                                    break;
                                }
                            }
                        }else if (static_cast<int>(exp_res.value) == loop.loop_index) {
                            flag = true;
                            break;
                        } else {
                            g_ptr_trace->PrintLog(LOG_ALARM, "ENDָ��˳��Ų�ƥ�䣡");
                            break;
                        }
                    }else//���ʽ����ʧ��
                        break;
                }

            } else {
                break;
            }
        }
    }

    return flag;
}

/**
 * @brief �����ת��Ŀ���еĺϷ���.1. ���Դ�ѭ�����ڵ�ת��ѭ������Ҳ������ѭ��������ת�� 2.�����Դ�ѭ��������ת��ѭ������
 * @param line_src
 * @param line_des
 * @return
 */
bool Compiler::CheckJumpGoto(uint64_t line_src, uint64_t line_des){
    printf("##########CheckJumpGoto:%lld, %lld\n", line_src, line_des);
    bool res = true;
    ListNode <LoopRec> *node = m_p_list_loop->HeadNode();
    LoopOffset loop;
    while (node != nullptr) {
        if (line_des > node->data.start_line_no && line_des < node->data.end_line_no) {  //Ŀ������ѭ������
            if(line_src < node->data.start_line_no || line_src > node->data.end_line_no ){ //��ת�����ѭ������
                res = false;
                break;
            }
        }else if(line_src > node->data.start_line_no && line_src < node->data.end_line_no){
            //������ѭ���壬����Ҫ������ջ��ѭ�������DO����
            printf("CheckJumpGoto jump:%lld, %lld\n", node->data.start_line_no, node->data.end_line_no);
            while(this->m_stack_loop.pop(loop)){
                if(loop.line_no == node->data.start_line_no){
                    printf("pop loop: %lld\n", loop.line_no);
                    break;
                }
                else{
                    printf("WARNNING:CheckJumpGoto\n");
                }
            }
        }

        node = node->next;
    }

    //  ���� if �� goto
    for(vector<IfElseOffset> node_vector: m_node_vectors_vector){
        // ��ת�յ��� һ�� if (elseif\else\endif) ֮��
        if(line_des > node_vector.at(0).line_no and line_des < node_vector.at(1).line_no){
            // ��ת��㲻�� ���� if (elseif\else\endif) ֮��
            if(line_src < node_vector.at(0).line_no or line_src > node_vector.at(1).line_no){
                res = false;
                break;
            }
        }
    }

    while(m_node_stack_run.size() != 0){
        // Ŀ���к� ���� ջ���ڵ��¼�к�  ����

		IfElseOffset node = m_node_stack_run.back();

		if(line_des > m_node_vectors_vector.at(node.vec_index).back().line_no){
        	m_node_stack_run.pop_back();
            m_else_jump_stack_run.pop_back();
        }else{
            break;
        }
    }
    return res;
}

/**
 * @brief �رյ�ǰ�ļ�ӳ��
 */
void Compiler::UnmapCurNcFile(){
    m_p_file_map_info->UnmapFile();
}

/**
 * @brie ����ӳ��nc�ļ�
 * @return
 */
bool Compiler::RemapCurNcFile(){
    return m_p_file_map_info->RemapFile();
}

/**
 * @brief ������������չ�±�ʹ��
 * @param flag : �Ƿ�������������չ�±�
 */
void Compiler::SetAxisNameEx(bool flag){
    this->m_b_axis_name_ex = flag;
    this->m_p_lexer->SetAxisNameEx(flag);
    this->m_p_parser->SetAxisNameEx(flag);
}

void Compiler::RefreshPmcAxis()
{
    m_p_parser->RefreshAxisName();
}

/**
 * @brief ���üӹ���λ����
 * @param line : ��λ�к�
 * @param mode ����λģʽ
 */
void Compiler::SetRestartPara(const uint64_t line, const uint8_t mode){
    this->m_n_restart_line = line;
    this->m_n_restart_mode = mode;
}

/**
 * @brief ͬ�����λ��
 * @param pos
 * @return
 */
bool Compiler::RefreshBlockMovePos(DPointChn &pos){
    RecordMsg *msg = nullptr;
    ListNode<RecordMsg *> *node = this->m_p_block_msg_list->HeadNode();
    CodeMsgType msg_type = NORMAL_MSG;
    DPointChn pp = pos;
    bool res = false;
    while(node != nullptr){
        msg = static_cast<RecordMsg *>(node->data);

        msg_type = msg->GetMsgType();

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
            }
            break;
        default:
            break;
        }
        node = node->next;  //ȡ��һ����Ϣ
    }

    this->SetCurPos(pp);   //���±������������
    //	printf("refresh block pos:%lf, %lf, %lf\n", pp.m_df_point[0], pp.m_df_point[1], pp.m_df_point[2]);
    return res;
}


/**
 * @brief ��������ӳ��Ϊ����ID
 * @param axis_name : ������
 * @return ������ID, ����0xFF�����ת��ʧ��
 */
uint8_t Compiler::MapAxisNameToParamIdx(uint8_t axis_name){
    uint8_t para = 0xFF;
    switch(axis_name){
    case AXIS_NAME_X:
        para = X_DATA;
        break;
    case AXIS_NAME_Y:
        para = Y_DATA;
        break;
    case AXIS_NAME_Z:			//Z��
        para = Z_DATA;
        break;
    case AXIS_NAME_A:			//A��
        para = A_DATA;
        break;
    case AXIS_NAME_B:			//B��
        para = B_DATA;
        break;
    case AXIS_NAME_C:			//C��
        para = C_DATA;
        break;
    case AXIS_NAME_U:			//U��
        para = U_DATA;
        break;
    case AXIS_NAME_V:			//V��
        para = V_DATA;
        break;
    case AXIS_NAME_W:				//W��
        para = W_DATA;
        break;
    default:
        break;
    }

    return para;
}

/**
 * @brief ����ѭ��ָ��Ĳ����������#171~#195
 * @param loop : ѭ��ָ����Ϣ
 */
void Compiler::SaveLoopParam(LoopMsg *loop){
    if(loop == nullptr)
        return;
    uint8_t pc = 0;   //��������
    uint32_t pm = 0;  //��������
    int i  = 0;
    double *pp = loop->GetParameter(pm, pc);
    Variable *pv = m_p_channel_control->GetMacroVar();

    // @question ��run message֮ǰģ̬�ͱ�����  �������о���Զ��������
    if(m_compiler_status.mode.gmode[9] == G80_CMD){  //�ɷ�ѭ��ģ̬ת��ѭ��ָ��ģ̬
        this->ResetLoopParam();   //�ȸ�λ����
        //д����Ե����������
        uint8_t chn_axis_count = this->m_p_channel_control->GetChnAxisCount();
        uint8_t chn_axis_data = 0;
        for(uint8_t i = 0; i < chn_axis_count; i++){
            chn_axis_data = this->MapAxisNameToParamIdx(m_p_channel_control->GetChnAxisName(i));

            //@add zk ��¼��ʼƽ��
            if(chn_axis_data == 25){
                pv->SetVarValue(199, m_compiler_status.cur_pos.GetAxisValue(i));
            }

            if((pm & (0x01<<chn_axis_data)) == 0){
                pv->SetVarValue(kLoopParamToGlobalVarIndex[chn_axis_data], m_compiler_status.cur_pos.GetAxisValue(i));
            }
        }
    }

    //����ǰ����д��ȫ�ֱ���
    while(pm != 0){
        if(pm & 0x01){
            pv->SetVarValue(kLoopParamToGlobalVarIndex[i], *pp);
            pp++;
        }
        pm = pm>>1;
        i++;
    }
}

/**
 * @brief ��λѭ��ָ�������Ӧ�ĺ����#171~#195
 */
void Compiler::ResetLoopParam(){
    Variable *pv = m_p_channel_control->GetMacroVar();

    for(int i = 0; i < 26; i++){
        pv->ResetVariable(kLoopParamToGlobalVarIndex[i]);
    }
}

#ifdef USES_WOOD_MACHINE
/**
 * @brief �����Ƿ���ڿ�Ԥ����������ָ��
 * @param line_min[in] : �����к�
 * @param line_max[in] : �����к�
 * @param spd_cmd[out] : ������������ڷ���ƥ��������������
 * @return true--�ҵ�ƥ�������    false--��ƥ�������
 */
bool Compiler::FindPreStartSpdCmd(uint64_t line_min , uint64_t line_max, SpindleStartOffset &spd_cmd){
    bool res = false;

    //	printf("Compiler::FindPreStartSpdCmd, count= %d\n", m_p_list_spd_start->GetLength());
    ListNode<SpindleStartOffset> *node = this->m_p_list_spd_start->HeadNode();


    while(node != nullptr){
        //		printf("Compiler::FindPreStartSpdCmd, node=%llu, min=%llu, max=%llu\n", node->data.line_no, line_min, line_max);


        if(node->data.line_no > line_min && node->data.line_no <= line_max){  //��Ԥ������Χ���ҵ�ƥ����
            if(!m_p_channel_control->CheckFuncState(FS_BLOCK_SKIP) || !node->data.jump_line){
                spd_cmd = node->data;
                res = true;
                break;
            }
        }else if(node->data.line_no > line_max)
            break;
        node = node->next;
    }

    return res;
}
#endif

#ifdef USES_FIVE_AXIS_FUNC
/**
 * @brief ��������������ת��ͽ�·������
 * @param tar : �յ�λ��
 * @param src : ���λ��
 * @param mask : �˶����mask
 */
void Compiler::ProcessFiveAxisRotPos(DPoint &tar, DPoint &src, uint32_t mask){
    if (!m_p_channel_control->IsFiveAxisMach() || m_p_channel_control->GetFiveAxisRotMaskNoLimt() == 0)  //�������������û��������ת��
        return;

    if(this->m_compiler_status.mode.gmode[8] != G43_4_CMD)
        return;   //������任״̬�����˳�

    if(this->m_compiler_status.mode.gmode[3] == G91_CMD)
        return;   //����ʽ��̲����ͽ�·��

    uint8_t axis_mask = mask & m_p_channel_control->GetFiveAxisRotMaskNoLimt();
    if(axis_mask == 0)
        return;
    double *psrc = &src.x;
    double *pdes = &tar.x;
    double dd = 0, dd_d = 0;
    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            dd = (*pdes - *psrc)/360.;

            if(dd > 0.5){
                dd_d = floor(dd);   //����ȡ��
                if(dd-dd_d > 0.5)
                    dd_d += 1;
                *pdes -= 360.0*dd_d;
            }
            if(dd < -0.5){
                dd_d = ceil(dd);   //����ȡ��
                if(dd-dd_d < -0.5)
                    dd_d -= 1;
                *pdes += 360.0*fabs(dd_d);
            }
            //	printf("ProcessFiveAxisRotPos: axis=%hhu, des=%lf, src=%lf\n", i, *pdes, *psrc);
        }
        psrc++;
        pdes++;
    }
}

/**
 * @brief ��������������ת��ͽ�·������
 * @param tar : �յ�λ��
 * @param src : ���λ��
 * @param mask : �˶����mask
 */
void Compiler::ProcessFiveAxisRotPos(DPointChn &tar, DPointChn &src, uint32_t mask){
    if (!m_p_channel_control->IsFiveAxisMach() || m_p_channel_control->GetFiveAxisRotMaskNoLimt() == 0)  //�������������û��������ת��
        return;

    if(this->m_compiler_status.mode.gmode[8] != G43_4_CMD)
        return;   //������任״̬�����˳�

    if(this->m_compiler_status.mode.gmode[3] == G91_CMD)
        return;   //����ʽ��̲����ͽ�·��

    uint8_t axis_mask = mask & m_p_channel_control->GetFiveAxisRotMaskNoLimt();
    if(axis_mask == 0)
        return;
    double *psrc = src.m_df_point;
    double *pdes = tar.m_df_point;
    double dd = 0, dd_d = 0;

    uint32_t pmc_mask = m_p_parser->GetPmcAxisMask();//PMC�������
    mask &= (~pmc_mask);    //ȥ��PMC��
    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            dd = (*pdes - *psrc)/360.;

            if(dd > 0.5){
                dd_d = floor(dd);   //����ȡ��
                if(dd-dd_d > 0.5)
                    dd_d += 1;
                *pdes -= 360.0*dd_d;
            }
            if(dd < -0.5){
                dd_d = ceil(dd);   //����ȡ��
                if(dd-dd_d < -0.5)
                    dd_d -= 1;
                *pdes += 360.0*fabs(dd_d);
            }

            //	printf("ProcessFiveAxisRotPos: axis=%hhu, des=%lf, src=%lf\n", i, *pdes, *psrc);
        }
        psrc++;
        pdes++;

    }

}
#endif

