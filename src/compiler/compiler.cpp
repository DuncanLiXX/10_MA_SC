/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file Compiler.cpp
 *@author gonghao
 *@date 2020/03/19
 *@brief 本文件为G代码编译器类的实现
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
 * @brief 编译器类构造函数
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
    //	this->m_p_five_axis_config = ParmManager::GetInstance()->GetFiveAxisConfig(m_n_channel_index);   //初始化五轴配置指针
    //#endif

    m_p_channel_config = ParmManager::GetInstance()->GetChannelConfig(
                m_n_channel_index);  //初始化通道参数
    m_p_mc_communication = MCCommunication::GetInstance();   //初始化MC通讯接口

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
 * @brief 编译器类析构函数
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

    // 单个if else 数据记录链表
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
 * @brief 编译器初始化
 */
void Compiler::InitCompiler() {
    printf("Enter Compiler::InitCompiler\n");

    m_b_check = true;   //默认进行语法检查
    m_b_has_over_code = false;  //默认没有结束指令

    m_n_sub_program = MAIN_PROG;  //默认在主程序状态
    m_c_end_line = ';';   //程序行结束符
    m_ln_cur_line_no = 1;   //当前编译行号

    this->m_b_breakout_prescan = false;
    this->m_b_prescan_over = false;
    m_b_prescan_in_stack = false;

    memset(m_line_buf, 0x00, kMaxLineSize); //初始化编译行的缓冲区

    m_p_cur_file_pos = nullptr;
    m_ln_read_size = 0;

    this->m_lexer_result.Reset();

    m_stack_scene.empty();
    m_stack_loop.empty();

    this->m_compiler_status.exact_stop_flag = false;
    this->m_compiler_status.jump_flag = false;

    //初始化创建文件操作对象
    m_p_file_map_info = new AsFileMapInfo();
    if (m_p_file_map_info == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建文件对象失败!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;
        return;
    }

    //初始化标签队列
    this->m_p_list_label = new LabelOffsetList();
    if (m_p_list_label == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建标签队列失败!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;
        return;
    }

    //初始化子程序队列
    this->m_p_list_subprog = new SubProgOffsetList();
    if (m_p_list_subprog == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建标签队列失败!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;
        return;
    }

    //初始化循环体起末点位置记录队列
    this->m_p_list_loop = new LoopRecList();
    if (m_p_list_loop == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建循环体队列失败!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;
        return;
    }

#ifdef USES_WOOD_MACHINE
    //初始化主轴启动指令记录队列
    this->m_p_list_spd_start = new SpindleStartList();
    if (m_p_list_spd_start == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建主轴启动指令队列失败!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;
        return;
    }
#endif
    //初始化创建MDA文件操作对象
    //	m_p_file_map_info_mda = new AsFileMapInfo();
    //	if(m_p_file_map_info_mda == nullptr){
    //		g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建MDA文件对象失败!", m_n_channel_index);
    //		m_error_code = ERR_SC_INIT;
    //		m_n_thread_state = ERROR;
    //		return;
    //	}
    //
    //	m_p_file_map_info = m_p_file_map_info_auto;  //当前文件操作对象初始化为AUTO文件对象

    //初始化AUTO模式语法解析结果队列
    this->m_p_parser_res_auto = new CompileRecList();
    if (this->m_p_parser_res_auto == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建自动模式语法解析结果队列失败!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //初始化失败
        //	m_n_thread_state = ERROR;
        return;
    }
    //初始化MDA模式语法解析结果队列
    this->m_p_parser_res_mda = new CompileRecList();
    if (this->m_p_parser_res_mda == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建MDA模式语法解析结果队列失败!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //初始化失败
        //	m_n_thread_state = ERROR;
        return;
    }

    this->m_p_parser_result = m_p_parser_res_auto;  //默认指向自动模式缓冲

    this->m_p_lexer = new Lexer(m_line_buf, &m_lexer_result);

    this->m_p_parser = new Parser(m_n_channel_index, &m_lexer_result,
                                  m_p_parser_result, &m_compiler_status);
    if (m_p_lexer == nullptr || m_p_parser == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建词法/语法解析器失败!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //初始化失败
        //	m_n_thread_state = ERROR;
        return;
    }
    this->m_p_parser->SetMacroVar(this->m_p_variable);
    this->m_p_parser->SetLastMoveRecPointer(&this->m_p_last_move_msg);

    //创建AUTO模式分段处理缓冲
    this->m_p_block_msg_list_auto = new OutputMsgList();
    if (this->m_p_block_msg_list_auto == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建自动模式分段指令消息队列失败!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //初始化失败
        return;
    }
    //创建MDA模式分段处理缓冲
    this->m_p_block_msg_list_mda = new OutputMsgList();
    if (this->m_p_block_msg_list_mda == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建MDA模式分段指令消息队列失败!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //初始化失败
        return;
    }

    this->m_p_block_msg_list = this->m_p_block_msg_list_auto;

    //创建AUTO模式刀补处理缓冲
    this->m_p_tool_compensate_auto = new ToolCompensate();
    if (this->m_p_tool_compensate_auto == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建自动模式刀补指令消息队列失败!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //初始化失败
        return;
    }
    //创建MDA模式刀补处理缓冲
    this->m_p_tool_compensate_mda = new ToolCompensate();
    if (this->m_p_tool_compensate_mda == nullptr) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器创建MDA模式刀补指令消息队列失败!",
                              m_n_channel_index);
        m_error_code = ERR_SC_INIT;  //初始化失败
        return;
    }
    this->m_p_tool_compensate = this->m_p_tool_compensate_auto;

    //状态初始化
    m_b_eof = false;
    m_b_comment = false;
    m_b_left = false;
    m_b_compile_over = false;
    m_error_code = ERR_NONE;

    m_work_mode = AUTO_COMPILER;  //默认在自动模式

    m_n_compile_state = FILE_HEAD;
    m_n_head_state = HEAD_INFO;

    printf("Exit Compiler::InitCompiler\n");
}

/**
 * @brief 设置编译器当前点位置，只针对NC轴
 * @param cur_pos : 当前位置
 * @return true--成功   false--失败
 */
bool Compiler::SetCurPos(const DPoint &cur_pos) {
    //	if(this->m_n_thread_state != IDLE)
    //		return false;
    //	printf("Compiler::SetCurPos: old(%lf, %lf, %lf, %lf, %lf, %lf)\nnew(%lf, %lf, %lf, %lf, %lf, %lf)",
    //			m_compiler_status.cur_pos.x, m_compiler_status.cur_pos.y, m_compiler_status.cur_pos.z, m_compiler_status.cur_pos.a4,
    //			m_compiler_status.cur_pos.a5, m_compiler_status.cur_pos.a6, cur_pos.x, cur_pos.y, cur_pos.z, cur_pos.a4, cur_pos.a5, cur_pos.a6);
    //	this->m_compiler_status.cur_pos = cur_pos;

    int count = 0; 
    uint32_t mask = m_p_parser->GetPmcAxisMask();//PMC轴的掩码
    for(int i = 0; i < this->m_p_channel_config->chn_axis_count && count < 8; i++){
        if(mask & (0x01<<i))
            continue;   //跳过PMC轴

        this->m_compiler_status.cur_pos.m_df_point[i] = cur_pos.GetAxisValue(count);
        count++;
    }
    return true;
}


/**
 * @brief 设置编译器当前点位置，只针对NC轴
 * @param cur_pos : 当前位置
 * @return true--成功   false--失败
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
 * @brief 重新初始化G模态
 * @return true--成功   false--失败
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
 * @brief 缓存编译器状态
 * @return true--成功   false--失败
 */
bool Compiler::SaveScene() {

    // @test zk
    printf("save scene !!!\n");
    CompilerScene scene;
    //	if(scene == nullptr){
    //		g_ptr_trace->PrintLog(LOG_ALARM, "保存编译器状态，分配内存失败！");
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
        g_ptr_trace->PrintLog(LOG_ALARM, "保存编译器状态失败！");
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
 * @brief 恢复编译器状态
 * @param bRecPos : 是否恢复当前位置， true--恢复  false--不恢复
 * @return true--成功   false--失败
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


    bool same_file = false;  //是否相同文件

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

    //宏程序调用有部分模态不用恢复
    // @modify zk  测试得到 M98 类型为SUB_PROG G65 P、固定循环 类型为 MACRO_PROG
    if(this->m_n_sub_program == MACRO_PROG){
    	mode_tmp = this->m_compiler_status.mode;
    }


    scene.file_map_info.Clear();  //防止后面delete scene时将文件映射关闭

    //	memcpy(m_p_file_map_info, &scene.file_map_info, sizeof(AsFileMapInfo));
    m_n_sub_program = scene.n_sub_program;
    m_b_eof = scene.b_eof;
    //	this->m_n_thread_state = scene.thread_state;

    // 要实现子程序调用 固定循环保持模态 注释这个条件  但不知道会不会引发其他问题
    //if(this->m_n_sub_program != SUB_PROG) //子程序调用不用恢复模态
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

    //宏程序调用有部分模态不用恢复
    if(this->m_n_sub_program == MACRO_PROG){
        this->m_compiler_status.mode.d_mode = mode_tmp.d_mode;
        this->m_compiler_status.mode.h_mode = mode_tmp.h_mode;
        this->m_compiler_status.mode.t_mode = mode_tmp.t_mode;
    }

    if (same_file) {
        m_p_file_map_info->JumpTo(this->m_ln_read_size);
    }

    printf("reload scene, m_work_mode:%d, scene.m_work_mode = %d， cur_line = %llu, m_p_cur_file_pos=%u, file=%s\n",
           m_work_mode, scene.work_mode, this->m_ln_cur_line_no, (uint32_t)this->m_p_cur_file_pos, m_p_file_map_info->str_file_name);

    for(IfElseOffset node: m_node_stack_run){
        printf("---------------------------------->node line no: %llu\n", node.line_no);
    }

    return true;
}

/**
 * @brief G代码与扫描运行线程函数
 * @param args
 */
void *Compiler::PreScanThread(void *args) {
    printf("Start Compiler::PreScanThread! id = %ld\n", syscall(SYS_gettid));
    Compiler *p_compiler = static_cast<Compiler *>(args);

    int res = ERR_NONE;
    res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
    if (res != ERR_NONE) {
        printf("Quit Compiler::PreScanThread with error 1!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }
    res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL); //线程退出方式为异步退出，不用等到cancellation point
    if (res != ERR_NONE) {
        printf("Quit Compiler::PreScanThread with error 2!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }

    p_compiler->PreScan();

    printf("Exit Compiler::PreScanThread!\n");
    pthread_exit(NULL);
}

/**
 * @brief 获取预扫描的一行数据
 * @param line[out] : 返回的一行数据
 * @param read_total ：已读取的总字节数
 * @param map : 文件映射对象
 * @return
 */
int Compiler::GetPreScanLine(char *line, uint64_t &read_total,
                             AsFileMapInfo &map) {
    if (line == nullptr)
        return 0;
    memset(line, 0x00, kMaxLineSize);

    if (read_total == map.ln_file_size || map.ptr_map_file == (char *)MAP_FAILED)
        return 0;

    char *pcur = map.ptr_map_file + read_total - map.ln_map_start;		//当前指针
    char c_cur = *pcur;     //当前字符
    char c_next = '\0';   //下一个字符
    bool beof = false;     //文档结束
    bool bSwapdown = false;   //是否向下翻页
    int index = 0;         //本行字节数
    uint64_t block_limit = map.ln_map_start + map.ln_map_blocksize;

REDO:
    //读取下一个字符
    if (read_total == block_limit - 1) {         //到达映射块尾部
        if (read_total == map.ln_file_size - 1) {  //到达文件尾
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
        c_next = *(pcur + 1);   //下一个字符

    while (!beof && (c_cur != '\r' || c_next != '\n') &&	//兼容两种换行格式（\r\n和\n）
           (c_cur != '\n'))	//非换行符
    {

        if (index < kMaxLineSize - 1)	//不能超过一行最大字符数
        {
            line[index++] = toupper(c_cur);  //字母统一用大写

        } else {
            break;
        }
        read_total++;

        if (read_total == block_limit) { //到达当前映射区末尾
            if (read_total == map.ln_file_size) {  //到达文件尾
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

        c_cur = *pcur;     //当前字符
        if (read_total == block_limit - 1) {     //到达映射块尾部
            if (read_total == map.ln_file_size - 1) {  //到达文件尾
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
            c_next = *(pcur + 1);   //下一个字符
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
            c_cur = *pcur;     //当前字符
            goto REDO;
        }
    }

    return index;
}

/**
 * @brief 代码预扫描，找到子程序和标签行
 */
void Compiler::PreScan() {
    //	printf("start compiler PreScan, thread id = %ld\n", syscall(SYS_gettid));

    uint64_t total_size = 0;		//文件总大小
    uint64_t read_size = 0;    //已读取的总大小
    //	uint64_t read_size_bak = 0;
    uint64_t line_no = 0;    //行号
    char *line = nullptr;   //单行代码缓冲
    //	char line[kMaxLineSize];   //单行代码缓冲
    size_t len = 0;
    ssize_t read_block = 0;  	//单次读取的大小
    bool comment_flag = false;   //注释状态
    int count = 0;

    LoopOffsetStack loop_stack;  //循环体堆栈


    //测试耗时
    struct timeval tvStart;
    struct timeval tvNow;
    unsigned int nTimeDelay = 0;
    unsigned int nTime1 = 0;
    gettimeofday(&tvStart, NULL);

#ifdef USES_WOOD_MACHINE
    this->m_n_s_code_in_prescan = 0;
#endif

    //打开预扫描文件
    FILE *file = nullptr;
    CompilerScene *ptr_scene = nullptr;
    StackRec<CompilerScene> *scene_node = nullptr;
    if(this->m_b_prescan_in_stack){
    	scene_node = this->m_stack_scene.bottom();
        if(scene_node != nullptr){
            ptr_scene = &scene_node->rec;
            file = fopen(ptr_scene->file_map_info.str_file_name, "r+");
        }else{
            //TODO 报错
            printf("error in Compiler::PreScan()\n");
            return;
        }

    }else{
        file = fopen(m_p_file_map_info->str_file_name, "r+");
    }
    if (nullptr == file) {
        g_ptr_trace->PrintLog(LOG_ALARM, "预扫描线程打开文件失败[%s]！errno = %d",
                              m_p_file_map_info->str_file_name, errno);
        CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
        goto END;
    }

    fseek(file, 0L, SEEK_END);
    total_size = ftell(file);   //获取文件长度

    fseek(file, 0L, SEEK_SET);   //回到文件头

    //使用内存映射方式
    //	AsFileMapInfo map;
    //	if(!map.OpenFile(m_p_file_map_info->str_file_name)){
    //		g_ptr_trace->PrintLog(LOG_ALARM, "预扫描线程打开文件失败[%s]！errno = %d", m_p_file_map_info->str_file_name, errno);
    //		CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, m_n_channel_index);
    //		goto END;
    //
    //	}
    //	total_size = map.ln_file_size;

    //第一遍扫描，识别出子程序号（O****）,以及GOTO指令
    while ((read_block = getline(&line, &len, file)) != -1) {
        //	read_size_bak = read_size;
        //	while(this->GetPreScanLine(line, read_size, map) > 0){
        if (m_b_breakout_prescan) //中断退出
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

    if (total_size != read_size) { //没有完整读取文件，告警
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]预扫描第一遍执行失败！文件未完整分析！%llu, %llu",
                              m_n_channel_index, total_size, read_size);
        CreateError(ERR_PRE_SCAN, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
        goto END;
    }

    gettimeofday(&tvNow, NULL);
    nTime1 = (tvNow.tv_sec - tvStart.tv_sec) * 1000000 + tvNow.tv_usec
            - tvStart.tv_usec;

    //第二遍扫描，找到所有GOTO指令跳转行号的偏移
    if (this->m_p_list_label->GetLength() > 0) {
        fseek(file, 0L, SEEK_SET);   //回到文件头
        read_size = 0;
        comment_flag = false;
        line_no = 0;
        count = 0;
        //	read_size_bak = read_size;
        //	map.ResetFile();
        while ((read_block = getline(&line, &len, file)) != -1) {
            //	while(this->GetPreScanLine(line, read_size, map) > 0){

            if (m_b_breakout_prescan)	//中断退出
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

        if (total_size != read_size) {	//没有完整读取文件，告警
            g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]预扫描第二遍执行失败！文件未完整分析！",
                                  m_n_channel_index);
            CreateError(ERR_PRE_SCAN, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                        m_n_channel_index);
            goto END;
        }
    }

    m_b_prescan_over = true;  //成功结束

    gettimeofday(&tvNow, NULL);
    nTimeDelay = (tvNow.tv_sec - tvStart.tv_sec) * 1000000 + tvNow.tv_usec
            - tvStart.tv_usec;
    printf("prescan time : first = %u us | second = %u us\n", nTime1,
           nTimeDelay);

END: if (line != nullptr)
        free(line);   //释放getline函数分配的缓冲
    fclose(file);
    //	m_thread_prescan = 0;
    m_b_breakout_prescan = false;
    this->m_b_prescan_in_stack = false;
    this->compiler_lock = false;
    printf("exit compiler PreScan, thread id = %ld\n", syscall(SYS_gettid));
}

/**
 * @brief 预扫描第一遍，分析行数据，识别出子程序号（O****），GOTO指令，以及所有的循环体
 *        对于木工专机，还要分析：主轴启动指令M03/M04和转速指令S，
 * @param buf : 行数据缓冲
 * @param line_no : 行号
 * @param flag : 跨行注释状态
 */
void Compiler::PreScanLine1(char *buf, uint64_t offset, uint64_t line_no,
                            bool &flag, LoopOffsetStack &loop_stack, CompilerScene *scene) {

    if (buf == nullptr)
        return;
    char *pc = buf;
    char digit_buf[kMaxDigitBufLen+1];
    bool jump_flag = false;    //是否包含跳段符
    bool first_alpha = true;   //是否首个字符
    bool sub_prog = false;   //子程序头
    bool goto_cmd = false;   //GOTO指令
    bool loop_do = false;   //循环头
    bool loop_end = false;  //循环尾

    bool if_cmd = false;    //存在 IF 指令
    bool endif_cmd = false;
    bool elseif_cmd = false;
    bool else_cmd = false;

    int digit_count = 0;  //数字计数
    ListNode < LabelOffset > *node = nullptr;

#ifdef USES_WOOD_MACHINE

    bool m_code = false;   //M指令
    bool s_code = false;   //S指令
    bool m_msg = false;    //是否存在M指令值对
    int m_digit = 0;       //M指令值
    bool s_msg = false;
    int s_digit = 0;
#endif

    memset(digit_buf, 0, kMaxDigitBufLen+1);

    StrUpper(buf);

    //printf("lino: %llu -- %s", line_no, buf);

    while (*pc != '\0') {
        if (flag) {  //处于注释状态
            if (*pc == ')') {  //结束注释状态
                flag = false;
            }
            pc++;
            continue;
        }

        if (isspace(*pc)) { //空白字符
            if ((sub_prog || goto_cmd) && digit_count > 0) { //找到子程序头或者goto指令
                break;
            }else if(loop_do || loop_end){
                break;
            }
#ifdef USES_WOOD_MACHINE
            else if(m_code && digit_count > 0){   //处理M指令
                m_digit = atoi(digit_buf);
                if(m_digit == 3 || m_digit == 4 || m_digit == 5){  //只处理M04和M03
                    m_msg = true;
                }
                m_code = false;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
            }else if(s_code && digit_count > 0){   //处理S指令
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
        else if (*pc == '(') { //进入跨行注释
            flag = true;
            pc++;
            continue;
        } else if (*pc == m_c_end_line || (*pc == '/' && *(pc + 1) == '/')) { //单行注释
            break;
        } else if(*pc == '/' && first_alpha){  //跳段符
            jump_flag = true;
        }else if (*pc == 'O' && first_alpha) { //子程序头
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
        else if (*pc == 'G' && *(pc + 1) == 'O' && *(pc + 2) == 'T' && *(pc + 3) == 'O' && !isalpha(*(pc+4))) { //goto指令

            if (first_alpha || pc == buf || !isalpha(*(pc - 1))) {
                goto_cmd = true;
                if_cmd = false;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
                pc += 4;
                continue;
            }

        } else if(*pc == 'D' && *(pc+1) == 'O' && !isalpha(*(pc+2))){//DO指令
            //	printf("FIND DO CMD!!!!!%c\n", *(pc-1));
            if(first_alpha || pc == buf || !isalpha(*(pc - 1))){
                loop_do = true;
                digit_count = 0;
                memset(digit_buf, 0, kMaxDigitBufLen+1);
                pc += 2;
                continue;
            }

        }else if(*pc == 'E' && *(pc+1) == 'N' && *(pc+2) == 'D' && !isalpha(*(pc+3))){//END指令
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
        else if(*pc == 'M' && !isalpha(*(pc+1))){  //M03/M04指令
            if(s_code && digit_count > 0){//处理S指令
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
        }else if(*pc == 'S' && !isalpha(*(pc+1))){  //S指令
            if(m_code && digit_count > 0){//处理M指令
                m_digit = atoi(digit_buf);
                if(m_digit == 3 || m_digit == 4 || m_digit == 5){  //只处理M04和M03
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
        else if (isdigit(*pc)) { //数字
            if (sub_prog) {
                digit_buf[digit_count++] = *pc;
                if (digit_count >= kMaxSubNameLen){ //子程序号超范围
                    //TODO 告警：子程序号超范围，不能超过5位

                    return;
                }
            } else if (goto_cmd) {
                digit_buf[digit_count++] = *pc;
                if (digit_count >= kMaxLineNoLen){ //顺序号超范围
                    //TODO 告警：顺序号超范围，不能超过8位

                    return;
                }
            }else if(loop_do || loop_end){
                digit_buf[digit_count++] = *pc;
                if(digit_count > 1){//DOn，n必须是个位数，不能超过8
                    //TODO 告警：DO序号超范围

                    return;
                }
            }
#ifdef USES_WOOD_MACHINE
            else if(m_code){  //处理M指令
                if(digit_count < kMaxMCodeLen)
                    digit_buf[digit_count++] = *pc;
            }else if(s_code){  //处理S指令
                if(digit_count < kMaxSCodeLen)
                    digit_buf[digit_count++] = *pc;
            }
#endif
        } else if ((sub_prog || goto_cmd) && digit_count > 0) { //找到子程序头或者goto指令
            break;
        }else if(loop_do || loop_end){
            break;
        }
#ifdef USES_WOOD_MACHINE
        else if(m_code && digit_count > 0){
            m_digit = atoi(digit_buf);
            if(m_digit == 3 || m_digit == 4 || m_digit == 5){  //只处理M04和M03
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

    if (sub_prog && digit_count > 0) { //找到子程序头
        SubProgOffset sub_offset;
        sub_offset.offset = offset;
        sub_offset.line_no = line_no;
        sub_offset.sub_index = atoi(digit_buf);
        //		printf("find sub program %d, line=%llu\n", sub_offset.sub_index, line_no);

        if(this->m_b_prescan_in_stack){   //堆栈文件预扫描
            if (scene->list_subprog.HasData(sub_offset)) {
                //有相同的子程序，告警
                g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]预扫描语法错误！相同的子程序号[%d]！",
                                      m_n_channel_index, sub_offset.sub_index);
                CreateError(ERR_SAME_SUB_INDEX, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                            sub_offset.sub_index, m_n_channel_index);
            } else
                scene->list_subprog.Append(sub_offset);
        }else{
            if (this->m_p_list_subprog->HasData(sub_offset)) {
                //有相同的子程序，告警
                g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]预扫描语法错误！相同的子程序号[%d]！",
                                      m_n_channel_index, sub_offset.sub_index);
                CreateError(ERR_SAME_SUB_INDEX, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                            sub_offset.sub_index, m_n_channel_index);
            } else
                this->m_p_list_subprog->Append(sub_offset);
        }

    } else if (goto_cmd && digit_count > 0) { //找到goto指令
        LabelOffset label_offset;
        label_offset.offset = 0;   //offset在第二次扫描时确定
        label_offset.line_no = 0;
        label_offset.label = atoi(digit_buf);

        //	printf("find goto label %d\n", label_offset.label);

        if(this->m_b_prescan_in_stack){   //堆栈文件预扫描
            if (!scene->list_label.HasData(label_offset)){
                //按升序排列
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
                //按升序排列
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

    }else if(loop_do){	//找到DO指令
        LoopOffset loop_offset;
        loop_offset.line_no = line_no;
        loop_offset.offset = offset;

        if(digit_count > 0)
            loop_offset.loop_index = atoi(digit_buf);
        else
            loop_offset.loop_index = 0;

        loop_stack.push(loop_offset);
        //	printf("push do cmd: %lld\n", line_no);
    }else if(loop_end){		//找到END指令
        LoopRec loop_rec;
        LoopOffset loop_offset;
        //	printf("pop end: %lld, loop_stack: %d\n", line_no, loop_stack.size());
        if(loop_stack.size()>0){  //已有对应DO指令
            loop_stack.pop(loop_offset);
            loop_rec.start_line_no = loop_offset.line_no;
            loop_rec.start_offset = loop_offset.offset;
            loop_rec.end_line_no = line_no;
            loop_rec.end_offset = offset;
            if(this->m_b_prescan_in_stack){   //堆栈文件预扫描
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
        if(m_digit == 3 || m_digit == 4){  //只处理M04和M03
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
    //处理转速指令
    if(s_msg){
        this->m_n_s_code_in_prescan = s_digit;
    }

    //处理主轴指令
    if(m_msg){
        if(m_digit == 5)   //主轴停
            this->m_n_s_code_in_prescan = 0;
        else if(m_digit == 3 || m_digit ==4){
            SpindleStartOffset spd_cmd;
            spd_cmd.line_no = line_no;
            spd_cmd.m_code = m_digit;
            spd_cmd.s_code = m_n_s_code_in_prescan;
            spd_cmd.jump_line = jump_flag;
            //	spd_cmd.exec_step = 0;
            printf("find spindle cmd:%d, spd:%d, line:%llu, jump=%hhu\n", m_digit, m_n_s_code_in_prescan, line_no, spd_cmd.jump_line);

            if(this->m_b_prescan_in_stack){   //堆栈文件预扫描
                scene->list_spd_start.Append(spd_cmd);
                printf("append in scene\n");
            }else{
                this->m_p_list_spd_start->Append(spd_cmd);
                printf("append in body\n");
            }
        }
    }
#endif

    // 找到 if 指令
    if(if_cmd){
    	// 初始化基础变量
        m_else_count_prescan = 0;
        m_node_vector_index = m_node_vector_len;
        m_node_vector_len ++;
        m_stack_vector_index_prescan.push_back(m_node_vector_index);
        m_stack_else_count_prescan.push_back(m_else_count_prescan);

        // 新建节点
        IfElseOffset node;
        node.offset = offset;
        node.line_no = line_no;
        node.vec_index = m_node_vector_index;
        node.node_index = 0;
        // 新建节点容器
        vector<IfElseOffset> node_vector;
        node_vector.push_back(node);
        //节点容器装入主容器
        if(this->m_b_prescan_in_stack){
            scene->node_vectors_vector.push_back(node_vector);
        }
        else{
            m_node_vectors_vector.push_back(node_vector);
        }

        if_cmd = false;

    }else if(endif_cmd){

        if(m_stack_vector_index_prescan.size() == 0){
            g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]预扫描语法错误  请检查 IF ELSEIF ELSE ENDIF 语法配对1！！！\n",
                                  m_n_channel_index);

            CreateError(IF_ELSE_MATCH_FAILED, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                        line_no, m_n_channel_index);
            return;
        }

        // 新建节点
        IfElseOffset node;
        node.offset = offset;
        node.line_no = line_no;
        node.vec_index = m_node_vector_index;

        //节点装入容器
        if(this->m_b_prescan_in_stack){
            node.node_index = scene->node_vectors_vector.at(m_node_vector_index).size();
            scene->node_vectors_vector.at(m_node_vector_index).push_back(node);
        }
        else{
            node.node_index = m_node_vectors_vector.at(m_node_vector_index).size();
            m_node_vectors_vector.at(m_node_vector_index).push_back(node);
        }

        // 这条IF 处理结束  弹栈
        m_stack_vector_index_prescan.pop_back();
        m_stack_else_count_prescan.pop_back();
        // 存在 IF 嵌套
        if(m_stack_vector_index_prescan.size() != 0){
            m_node_vector_index = m_stack_vector_index_prescan.back();
            m_else_count_prescan = m_stack_else_count_prescan.back();
        }

        endif_cmd = false;

    }else if(else_cmd or elseif_cmd){

        if(m_stack_vector_index_prescan.size() == 0){
            g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]预扫描语法错误  请检查 IF ELSEIF ELSE ENDIF 语法配对2！！！\n",
                                  m_n_channel_index);

            CreateError(IF_ELSE_MATCH_FAILED, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                        line_no, m_n_channel_index);
            return;
        }

        if(else_cmd){
            ++ m_else_count_prescan;
            if(m_else_count_prescan > 1){
                g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]预扫描语法错误  请检查 IF ELSEIF ELSE ENDIF 语法配对3！！！\n",
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
 * @brief 预扫描第二遍，分析行数据，找到所有GOTO指令跳转行号的偏移
 * @param buf : 行数据缓冲
 * @param offset : 当前行的偏移位置
 * @param line_no : 行号
 * @param flag : 跨行注释状态
 */
void Compiler::PreScanLine2(char *buf, uint64_t offset, uint64_t line_no,
                            bool &flag, CompilerScene *scene) {
    //预扫描第二遍，分析行数据

    if (buf == nullptr)
        return;
    char *pc = buf;
    char digit_buf[10];
    bool line_serial = false;   //顺序号
    int digit_count = 0;  //数字计数
    memset(digit_buf, 0, 10);

    StrUpper(buf);
    while (*pc != '\0') {
        if (flag) {  //处于注释状态
            if (*pc == ')') {  //结束注释状态
                flag = false;
            }
            pc++;
            continue;
        }

        if (isspace(*pc)) { //空白字符
            if (line_serial && digit_count > 0) { //找到顺序号
                break;
            }
            pc++;
            continue;
        }

        if (strchr("XYZMABCDHIJKTUVW", *pc) != nullptr)   //加快扫描速度
            break;
        else if (*pc == '(') { //进入跨行注释
            flag = true;
            pc++;
            continue;
        } else if (*pc == m_c_end_line || (*pc == '/' && *(pc + 1) == '/')) { //单行注释
            break; //直接返回
        } else if (*pc == 'N') { //顺序号
            if (!isalpha(*(pc + 1)))
                line_serial = true;
        } else if (*pc == m_c_end_line) { //行结束符
            if (line_serial && digit_count > 0) { //找到顺序号
                break;
            }
        } else if (isdigit(*pc)) { //数字
            if (line_serial) {
                if (digit_count >= kMaxLineNoLen) //顺序号超范围
                    return;
                digit_buf[digit_count++] = *pc;
            }
        } else if (line_serial && digit_count > 0) { //找到顺序号
            break;
        }

        if (!line_serial)
            return;

        pc++;
    }

    if (line_serial) {
        int label = atoi(digit_buf);
        ListNode < LabelOffset > *node = nullptr;
        if(this->m_b_prescan_in_stack){   //堆栈文件预扫描
            node = scene->list_label.HeadNode();
        }else{
            node = this->m_p_list_label->HeadNode();
        }

        while (node != nullptr) {
            if (node->data.label == label) {
                if (node->data.offset > 0) { //TODO 存在重复的顺序号  告警
                    g_ptr_trace->PrintLog(LOG_ALARM,
                                          "CHN[%d]预扫描语法错误！相同的顺序号[%d]！", m_n_channel_index,
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
 * @brief 打开待编译文件
 * @param file : 待编译文件
 * @param sub_flag : 是否子程序文件
 */
bool Compiler::OpenFile(const char *file, bool sub_flag) {
    //	bool res = false;
    //	if(this->m_work_mode == MDA_COMPILER){  //MDA模式下在启动编译时再打开文件
    //		res = m_p_file_map_info->OpenFile(m_str_mda_path);
    //	}
    //	else if(m_work_mode == AUTO_COMPILER){
    //		res = m_p_file_map_info->OpenFile(file);
    //	}
    // printf("compiler::openfile, file[%s], sub_flag= %hhu\n", file, sub_flag);
    int res = 0;

    // @add zk 加载过长文件名导致崩溃
    if(strlen(file) > kMaxFileNameLen-1){
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]文件名过长[%s]打开文件失败!",
                              m_n_channel_index, file);
        CreateError(ERR_OPEN_FILE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
        return false;
    }
    // @add zk

    if(this->m_work_mode == MDA_COMPILER && !sub_flag){  //
        char tmp_file[kMaxPathLen] = {0};	//文件路径
        this->m_p_channel_control->GetMdaFilePath(tmp_file);
        if(strcmp(file, tmp_file) != 0){
            printf("error in Compiler::OpenFile, can't open file in MDA mode!file[%s]\n", file);
            return false;
        }
    }

    if (!m_p_file_map_info->OpenFile(file, sub_flag)) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器打开文件[%s]失败!",
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
    //创建预扫描线程
    if (!m_b_prescan_over && m_thread_prescan != 0){
        //先退出之前的线程
        m_b_breakout_prescan = true;
        int wait_count = 0;

        while (m_b_breakout_prescan && wait_count++ < 200)
            usleep(1000);  //等待线程退出
        if (m_b_breakout_prescan) {//等待退出失败，则主动cancel线程

            //退出预扫描运行线程
            res = pthread_cancel(m_thread_prescan);
            if (res != 0) {
                g_ptr_trace->PrintLog(LOG_ALARM, "预扫描线程退出失败1！errno = %d\n",
                                      res);
                printf("预扫描线程退出失败1111\n");
                CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                            0, m_n_channel_index);
                return false;
            }

            usleep(1000);
        }
        res = pthread_join(m_thread_prescan, &thread_result);  //等待线程退出完成
        if (res != 0) {
            g_ptr_trace->PrintLog(LOG_ALARM, "预扫描线程退出失败2！errno = %d\n",
                                  res);
            printf("预扫描线程退出失败2222\n");
            CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                        0, m_n_channel_index);
            return false;
        }
        m_thread_prescan = 0;

    }else if(m_b_prescan_over && m_thread_prescan != 0){  //释放预扫描线程资源
        res = pthread_join(m_thread_prescan, &thread_result);  //等待线程退出完成
        if (res != 0) {
            g_ptr_trace->PrintLog(LOG_ALARM, "预扫描线程资源释放失败！errno = %d\n",
                                  res);
            printf("预扫描线程退出失败3333\n");
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
    m_stack_loop.empty();  //清空循环位置数据

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
        param.__sched_priority = 36;	//96; //子程序，预扫描线程比编译线程优先级高一级
    else
        param.__sched_priority = 35; //95; //主程序，预扫描线程比编译线程优先级一，因为主程序可能比较大
    pthread_attr_setschedparam(&attr, &param);

    /* Use scheduling parameters of attr */
    //	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
    //	if (res) {
    //		printf("pthread setinheritsched failed\n");
    //	}
    res = pthread_create(&m_thread_prescan, &attr, Compiler::PreScanThread, this);    //开启G代码预编译线程
    if (res != 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器预扫描线程创建失败! res = %d, errno = %d, errstr=%s",
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
 * @brief 在保存的场景中打开文件，比如在非自动模式下，加载加工文件
 * @param file ： 加载的主文件路径
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
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器打开场景栈文件[%s]失败!",
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

    this->m_stack_scene.push(scene);   //重新入栈

    scene.file_map_info.Clear();//防止后面delete scene时将文件映射关闭

    void* thread_result;
    //创建预扫描线程
    if (!m_b_prescan_over && m_thread_prescan != 0) {
        //先退出之前的线程
        m_b_breakout_prescan = true;
        int wait_count = 0;

        while (m_b_breakout_prescan && wait_count++ < 200)
            usleep(1000);  //等待线程退出
        if (m_b_breakout_prescan) {//等待退出失败，则主动cancel线程

            //退出预扫描运行线程
            res = pthread_cancel(m_thread_prescan);
            if (res != 0) {
                g_ptr_trace->PrintLog(LOG_ALARM, "预扫描线程退出失败1！errno = %d\n",
                                      res);
                printf("预扫描线程退出失败4444\n");
                CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                            0, m_n_channel_index);
                return false;
            }

            usleep(1000);
        }
        res = pthread_join(m_thread_prescan, &thread_result);  //等待线程退出完成
        if (res != 0) {
            g_ptr_trace->PrintLog(LOG_ALARM, "预扫描线程退出失败2！errno = %d\n",
                                  res);
            printf("预扫描线程退出失败5555\n");
            CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                        0, m_n_channel_index);
            return false;
        }
        m_thread_prescan = 0;

    }else if(m_b_prescan_over && m_thread_prescan != 0){  //释放预扫描线程资源
        res = pthread_join(m_thread_prescan, &thread_result);  //等待线程退出完成
        if (res != 0) {
            g_ptr_trace->PrintLog(LOG_ALARM, "预扫描线程资源释放失败！errno = %d\n",
                                  res);
            printf("预扫描线程退出失败6666\n");
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

    m_stack_loop.empty();  //清空循环位置数据

    pthread_attr_t attr;
    struct sched_param param;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//

    param.__sched_priority = 35; //95; //主程序，预扫描线程比编译线程优先级一，因为主程序可能比较大
    pthread_attr_setschedparam(&attr, &param);

    /* Use scheduling parameters of attr */
    //	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
    //	if (res) {
    //		printf("pthread setinheritsched failed\n");
    //	}
    res = pthread_create(&m_thread_prescan, &attr, Compiler::PreScanThread, this);    //开启G代码预编译线程
    if (res != 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器预扫描线程创建失败! res = %d, errno = %d, errstr=%s",
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
 * @brief 编译结束后的相关处理，遇到M30/M02/M99这些结束指令时调用
 */
bool Compiler::CompileOver() {
    printf("enter compiler::compileover, m_work_mode = %d\n", m_work_mode);
    if (this->m_n_sub_program != MAIN_PROG) {
        //子程序
        g_ptr_trace->PrintTrace(TRACE_ERROR, COMPILER_CHN,
                                "调用错误，子程序进入Compiler::CompileOver()函数！");
        //TODO 报错
        return false;
    }
    //主程序结束，正常
    if (m_work_mode == AUTO_COMPILER && m_stack_scene.size() > 0) { //自动模式下，主程序结束，调用堆栈应该空
        g_ptr_trace->PrintLog(LOG_ALARM, "主程序结束，子程序调用堆栈非空！");
        CreateError(ERR_MAIN_OVER, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                    m_n_channel_index);
        return false;
    }else if (m_work_mode == MDA_COMPILER && m_stack_scene.size() == 0) {
        g_ptr_trace->PrintLog(LOG_ALARM, "MDA模式下，程序调用堆栈为空！");
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
 * @brief 循环编译，处理M99指令
 */
void Compiler::RecycleCompile() {
    printf("compiler::RecycleCompile\n");
    //	if (m_n_sub_program != MAIN_PROG) {  //子程序可以循环调用
    //		printf("sub prog return\n");
    //		return;
    //	}
    this->m_p_file_map_info->ResetFile();  //文件复位到头部
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
 * @brief 设置编译工作模式
 * @param mode ：工作模式
 */
void Compiler::SetMode(CompilerWorkMode mode) {

    printf("compiler::SetMode, old mode = %d, new mode = %d\n", m_work_mode, mode);
    if (m_work_mode == mode)
        return;

    //TODO 添加后续处理动作,AUTO和MDA之间需要做buffer的切换，以及状态切换
    if (m_work_mode == AUTO_COMPILER) {  //AUTO切换到MDA

        //		if(m_n_thread_state == RUN){//如果编译器在RUN状态，则先暂停
        //			this->PauseCompile();
        //		}
        //		pthread_mutex_lock(&m_mutex_change_state);

        //缓存状态
        this->SaveScene();

        //复位编译器状态
        this->ResetState();

        this->m_p_file_map_info->Clear();  //清除自动模式文件打开数据，防止打开mda文件时关闭自动模式下的文件

        this->m_n_compile_state = FILE_MAIN;

        this->m_p_block_msg_list = this->m_p_block_msg_list_mda;
        this->m_p_tool_compensate = this->m_p_tool_compensate_mda;
        this->m_p_parser_result = this->m_p_parser_res_mda;

        //	pthread_mutex_unlock(&m_mutex_change_state);
    } else if (m_work_mode == MDA_COMPILER) {  //MDA切换到AUTO
        //		if(m_n_thread_state == RUN){//如果编译器在RUN状态，则先停止
        //			this->StopCompile();
        //		}
        char tmp_file[kMaxPathLen] = {0};	//文件路径
        this->m_p_channel_control->GetMdaFilePath(tmp_file);
        if(strcmp(this->m_p_file_map_info->str_file_name, tmp_file) != 0){  //
            //			printf("Compiler::SetMode, clear file:%s\n", this->m_p_file_map_info->str_file_name);
            this->m_p_file_map_info->Clear();
        }
        this->ReloadScene();  //恢复现场

        this->m_p_block_msg_list = this->m_p_block_msg_list_auto;
        this->m_p_tool_compensate = this->m_p_tool_compensate_auto;
        this->m_p_parser_result = this->m_p_parser_res_auto;

    }

    this->m_p_parser->SetParserResList(m_p_parser_result);

    //设置新的模式
    m_work_mode = mode;

}

/**
 * @brief 编译器整体复位，停止编译，恢复初始状态
 */
void Compiler::Reset(){

    printf("compiler reset !!!\n");

    CompilerScene scene;

    this->m_b_breakout_prescan = true;

    if (m_work_mode == MDA_COMPILER) {
        printf("compiler reset: MDA_COMPILER\n");
        while (m_stack_scene.size() > 0) {
            this->m_stack_scene.cur(scene);
            if (scene.work_mode == MDA_COMPILER && scene.n_sub_program != MAIN_PROG) { //入栈的子程序状态直接删除
                m_stack_scene.pop(scene);
                scene.file_map_info.CloseFile();
            }else if (scene.work_mode == MDA_COMPILER) {
                this->ReloadScene();  //恢复到主程序现场
            }else if(scene.work_mode == AUTO_COMPILER && scene.n_sub_program != MAIN_PROG){ //遇到自动模式数据，说明MDA下没有调用子程序
                m_stack_scene.pop(scene);
                scene.file_map_info.CloseFile();
            }else if(scene.work_mode == AUTO_COMPILER){  //自动模式主程序状态，复位
                m_stack_scene.pop(scene);
                scene.b_eof = false;
                scene.file_map_info.ResetFile();   //复位到文件头
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
            if (scene.n_sub_program != MAIN_PROG) { //子程序场景直接删除
                m_stack_scene.pop();

            } else {

                this->ReloadScene();   //恢复到主程序现场
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

    m_stack_loop.empty();  //清空循环位置数据

    /********************************/

    // 复位运行时记录
    m_node_stack_run.clear();
    m_else_jump_stack_run.clear();
    m_b_else_jump = false;
    /********************************/
}

/**
 * @brief 编译器状态复位
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
 * @brief 空闲处理函数
 */
void Compiler::DoIdle(){

}

/**
 *@brief 设置文件映射对象
 *@param AsFileMapInfo *pFile: 文件映射对象指针
 */
/*void Compiler::SetFileMap(AsFileMapInfo *pFile){
 m_p_file_map_info = pFile;
 //m_p_cur_pos = reinterpret_cast<char *>(m_p_file_map_info->ptr_map_file);
 }*/

/**
 * @brief 判断是否换行符
 * @param char *pc:待判断字符
 * @return 是换行符则返回true，否则返回false
 */
bool Compiler::IsNewLineChar(char *pc) {
    if ((*pc == '\r' && *(pc + 1) == '\n') || *pc == '\n')  //兼容两种换行格式"\r\n"和\n
        return true;
    return false;
}

/**
 * @brief 获取当前加工的NC文件名称
 * @param file[out] : 输出当前nc文件名称
 */
void Compiler::GetCurNcFile(char *file){
    if(file == nullptr)
        return;

    strcpy(file, m_p_file_map_info->str_file_name);
}

//int total_time = 0;

/**
 * @brief 获取一行NC代码，放到当前编译行缓冲里面,此函数返回的字符串已经去除了多余的空白字符以及注释
 * @return 执行结果,成功返回true，失败返回false
 */
bool Compiler::GetLineData() {
    //	struct timeval tvStart;
    //	struct timeval tvNow;
    //	unsigned int nTimeDelay = 0;
    //
    //	gettimeofday(&tvStart, NULL);

    //	printf("enter getlinedata\n");

    bool res = true;
    if(m_p_file_map_info == nullptr || m_b_compile_over) {  //编译结束
        //	printf("getline return, 111, %d, %d\n", (int)m_p_file_map_info, m_b_compile_over);
        return false;
    }


    this->m_p_lexer->Reset(); //词法分析器复位
    memset(m_line_buf, 0x00, static_cast<size_t>(kMaxLineSize));

    if (m_b_eof || m_p_file_map_info->ln_file_size == 0) {
        m_b_eof = true;
        if(this->m_work_mode == MDA_COMPILER){  //MDA模式下，到文件尾结束编译
            //lidianqiang:MDA自动加上M30临时改为M300
            strcpy(m_line_buf, "M300");
            this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //更新行号
            g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "MDA insert M30-2\n");
            return true;
        }
#ifdef USES_WOOD_MACHINE
        if(m_n_sub_program == MAIN_PROG){//主程序
            strcpy(m_line_buf, "M30");   //木工专机可以没有M30指令，系统自动添加
            this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //更新行号
            g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "Insert M30-2\n");
        }else{//子程序/宏程序
            strcpy(m_line_buf, "M99");   //木工专机可以没有M99指令，系统自动添加
            this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //更新行号
            g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "Insert M99-2\n");
        }
        return true;
#else
        return false;   //直接返回
#endif
    }

    if(m_p_cur_file_pos == nullptr){
        m_p_cur_file_pos = m_p_file_map_info->ptr_map_file;
    }


    int index = 0;
    int last_char_type = 0;   //前一个字符类型，0表示非字符也非数字，1表示字符，2表示数字
    bool bSpace = false;   //前一个字符为空白字符
    bool bFirst = false;    //遇到第一个非空白字符
    bool bSwapdown = false;  //向下翻页

    char c_cur = *m_p_cur_file_pos;   //当前字符

    char c_next = '\0';   //下一个字符
    uint64_t block_limit = m_p_file_map_info->ln_map_start
            + m_p_file_map_info->ln_map_blocksize;

    this->m_compiler_status.jump_flag = false;

REDO:
    //读取下一个字符
    if (m_ln_read_size >= block_limit - 1) {   //到达映射块尾部
        if (m_ln_read_size >= m_p_file_map_info->ln_file_size - 1) {  //到达文件尾
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
        c_next = *(m_p_cur_file_pos + 1);   //下一个字符

    this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //更新行号
    this->m_lexer_result.offset = this->m_ln_read_size;      //保存行首偏移量
    //	printf("getline lineno = %lld\n", this->m_lexer_result.line_no);
    while (!m_b_eof && (c_cur != '\r' || c_next != '\n') &&//兼容两种换行格式（\r\n和\n）
           (c_cur != '\n')) {
        if (!m_b_comment) {
            if (c_cur == '(') {
                m_b_comment = true;
                m_b_left = true;
            } else if (c_cur == m_c_end_line
                       || (c_cur == '/' && c_next == '/')) {	//开启注释
                m_b_comment = true;
            }
        }

        if (!m_b_comment) {	//非注释
            if (index < kMaxLineSize - 1)	//不能超过一行最大字符数
            {
                if (isspace(c_cur)) {
                    if (!bSpace && bFirst) {
                        bSpace = true;
                        m_line_buf[index++] = c_cur;
                    }
                } else {
                    bSpace = false;
                    bFirst = true;
                    //保留字母与字母间或者数字与数字间的空白字符，其它空白字符均删除
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
                    m_line_buf[index++] = toupper(c_cur);  //字母统一用大写
                }
            }
        } else {  //跨行注释部分
            if (m_b_left && c_cur == ')') {  //结束注释
                m_b_comment = false;
                m_b_left = false;
            }
        }

        m_ln_read_size++;  //已读取字节数加一

        if (m_ln_read_size == block_limit) { //到达当前映射区末尾
            if (m_ln_read_size == m_p_file_map_info->ln_file_size) {  //到达文件尾
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

        c_cur = *m_p_cur_file_pos;     //当前字符
        if (m_ln_read_size == block_limit - 1) {     //到达映射块尾部
            if (m_ln_read_size == m_p_file_map_info->ln_file_size - 1) { //到达文件尾
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
            c_next = *(m_p_cur_file_pos + 1);   //下一个字符
    }

    if (m_b_comment && !m_b_left)
        m_b_comment = false;  //对于"//"开始的单行注释，进行标志复位

    //处理一行代码首尾的空白字符
    while (index > 0 && isspace(m_line_buf[index - 1])) {
        m_line_buf[index - 1] = '\0';
        index--;
    }

    //跳过行尾字符，指向下一行行首
    if (!m_b_eof) {

        if (c_cur == '\n') {
            m_p_cur_file_pos += 1;
            m_ln_read_size += 1;
        } else {
            m_p_cur_file_pos += 2;
            m_ln_read_size += 2;
        }

        m_ln_cur_line_no++;  //当前编译行号加一

        if (m_ln_read_size >= m_p_file_map_info->ln_file_size) {
            printf("eof 3333, read_size = %llu, file_size = %llu\n",
                   m_ln_read_size, m_p_file_map_info->ln_file_size);
            m_b_eof = true;
        }

    } else {
        //		this->StopCompile();
        g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "end of the file\n");
    }

    if (index == 0) {  //如果空白行且文件未结束，则继续读取下一行
        if (!m_b_eof) {
            index = 0;
            bSpace = false;   //前一个字符为空白字符
            bFirst = false;    //遇到第一个非空白字符
            c_cur = *m_p_cur_file_pos;     //当前字符
            //	c_next = *(m_p_cur_file_pos+1);   //下一个字符
            goto REDO;
            //重读下一行
        } else if (this->m_work_mode == MDA_COMPILER) {
            //MDA模式，插入一个M30结束命令
            //lidianqiang:MDA自动加上M30临时改为M300
            strcpy(m_line_buf, "M300");
            this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //更新行号
            g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "MDA insert M30-1\n");
        } else {
#ifdef USES_WOOD_MACHINE
            if(m_n_sub_program == MAIN_PROG){//主程序
                strcpy(m_line_buf, "M30");   //木工专机可以没有M30指令，系统自动添加
                this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //更新行号
                g_ptr_trace->PrintTrace(TRACE_DETAIL, COMPILER_CHN, "Insert M30-1\n");
            }else{//子程序/宏程序
                strcpy(m_line_buf, "M99");   //木工专机可以没有M99指令，系统自动添加
                this->m_lexer_result.line_no = this->m_ln_cur_line_no;   //更新行号
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
 * @brief 编译一行代码
 * @return 执行结果
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
        //出错处理
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
 * @brief 执行编译产生的Msg,修改编译器状态
 * @return true--成功   false--失败
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
            if(msg->CheckFlag(FLAG_JUMP) && this->m_p_channel_control->CheckFuncState(FS_BLOCK_SKIP)){  //跳段激活,则当前指令取消
                this->m_p_parser_result->RemoveHead();
                delete node;
                node = this->m_p_parser_result->HeadNode();  //取下一个消息
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
                    this->m_p_parser_result->Clear();  //出错，清空队列

                }
                break;  //执行不成功
            }

            //msg->PrintString();

            //TODO 增加刀补处理

            this->m_p_parser_result->RemoveHead();
            this->m_p_block_msg_list->Append(node);   //压入分段队列

        }

        node = this->m_p_parser_result->HeadNode();  //取下一个消息
    }

    //处理分段
    node = this->m_p_block_msg_list->HeadNode();
    while (node != nullptr) {
        msg = static_cast<RecordMsg *>(node->data);
        if (msg != nullptr) { //出错情况下，全部输出
            if (msg == this->m_p_last_move_msg
                    && this->m_error_code == ERR_NONE &&
                    this->m_work_mode == AUTO_COMPILER) {  //MDA模式下，全部输出
                break;  //未确定是否为分段最后一条移动指令，暂不执行
            }

            this->m_p_block_msg_list->RemoveHead();
            //			this->m_p_output_msg_list->Append(node);   //压入待发送队列
            this->m_p_tool_compensate->ProcessData(node);  //压入刀补数据缓冲

            if(m_p_tool_compensate->err_code != ERR_NONE){
                m_error_code = m_p_tool_compensate->err_code;
                CreateError(m_p_tool_compensate->err_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                            m_n_channel_index);
                //m_p_tool_compensate->clearError();

                res = false;
            }

        }

        node = this->m_p_block_msg_list->HeadNode();  //取下一个消息
    }

    /*if(res){
        printf("----compiler run message %llu success\n", lineNo);
    }else{
        printf("----compiler run message %llu failed\n", lineNo);
    }*/

    return res;
}

/**
 * @brief 执行辅助指令消息
 * @param msg : 消息指针
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
            if (m_n_sub_program != MAIN_PROG) {  //子程序不允许出现，告警
                CreateErrorMsg(ERR_SUB_END, msg->GetLineNo());
                return false;
            } else {
                this->m_b_compile_over = true;
                printf("compiler run M30\n");

            }

            break;
        case 06:   //换刀
            printf("run m06 msg\n");

            break;

            //	case 98:   //M98   子程序调用  使用独立的SubProgCallMsg处理
            //		break;
        case 99:   //M99
            if (m_n_sub_program != MAIN_PROG) {
                this->ReturnFromSubProg();   //子程序则返回调用程序
            } else {
                this->m_b_compile_over = true;
            }

            printf("compiler run M99\n");
            break;
        case 300: //lidianqiang:MDA自动加上M30临时改为M300
            this->m_b_compile_over = true;
            break;
        default:
            break;
        }
    }
    return true;
}

/**
 * @brief 编译器中运行子程序调用指令消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunSubProgCallMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;

    if (!this->m_b_prescan_over && m_thread_prescan != 0) //预扫描未结束，暂不执行
        return false;

    SubProgCallMsg *sub_msg = (SubProgCallMsg *) msg;

    int sub_index = sub_msg->GetSubProgIndex();

    //查找子程序
    int sub_loc = this->FindSubProgram(sub_index);
    sub_msg->SetSubProgType(sub_loc);
    if (sub_loc == 0) {
        CreateErrorMsg(ERR_NO_SUB_PROG, msg->GetLineNo());  //找不到对应子程序
        return false;
    }

    //保存当前编译器状态
    this->SaveScene();

    //局部变量入栈
    m_p_channel_control->GetMacroVar()->PushLocalVar();

    this->m_n_sub_call_times = sub_msg->GetCallTimes();//调用次数


    //打开子程序文件
    this->m_n_sub_program = SUB_PROG;
    char filepath[kMaxPathLen] = { 0 };   //文件路径
    if (sub_loc == 1) {   //同文件子程序
        ListNode < SubProgOffset > *node = m_p_list_subprog->HeadNode();
        uint64_t offset_sub = 0;
        while (node != nullptr) {
            if (node->data.sub_index == sub_index) {
                offset_sub = node->data.offset;
                break;
            }
            node = node->next;
        }

        if (!this->m_p_file_map_info->JumpTo(offset_sub)) {   //映射失败
            CreateErrorMsg(ERR_JUMP_SUB_PROG, msg->GetLineNo());  //子程序跳转失败
            return false;
        }

        this->m_ln_read_size = offset_sub;
        this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file
                + m_ln_read_size - m_p_file_map_info->ln_map_start;

        this->m_ln_cur_line_no = node->data.line_no;
    } else {
        sub_msg->GetSubProgName(filepath, true);
        sub_msg->SetLastProgFile(this->m_p_file_map_info->str_file_name+strlen(PATH_NC_FILE));  // 保存当前文件路径，相对路径
        //独立子文件打开
        if (!this->OpenFile(filepath, (bool)m_n_sub_program)){ //尝试打开nc文件失败
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
 * @brief 编译器中运行宏程序调用指令消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunMacroProgCallMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;

    if (!this->m_b_prescan_over && m_thread_prescan != 0){ //预扫描未结束，暂不执行

        return false;
    }

    MacroProgCallMsg *sub_msg = (MacroProgCallMsg *) msg;

    int macro_index = sub_msg->GetMacroProgIndex();

    //查找子程序
    int macro_loc = this->FindSubProgram(macro_index);
    sub_msg->SetMacroProgType(macro_loc);
    if (macro_loc == 0) {
        CreateErrorMsg(ERR_NO_SUB_PROG, msg->GetLineNo());  //找不到对应子程序
        return false;
    }

    //保存当前编译器状态
    this->SaveScene();

    Variable *pv = m_p_channel_control->GetMacroVar();
    //局部变量入栈
    pv->PushLocalVar();

    //赋值自变量参数到局部变量
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

    this->m_n_sub_call_times = sub_msg->GetCallTimes();//调用次数

    //打开子程序文件
    this->m_n_sub_program = MACRO_PROG;
    char filepath[kMaxPathLen] = { 0 };   //文件路径
    if (macro_loc == 1) {   //同文件子程序
        ListNode < SubProgOffset > *node = m_p_list_subprog->HeadNode();
        uint64_t offset_sub = 0;
        while (node != nullptr) {
            if (node->data.sub_index == macro_index) {
                offset_sub = node->data.offset;
                break;
            }
            node = node->next;
        }

        if (!this->m_p_file_map_info->JumpTo(offset_sub)) {   //映射失败
            CreateErrorMsg(ERR_JUMP_SUB_PROG, msg->GetLineNo());  //子程序跳转失败
            return false;
        }

        this->m_ln_read_size = offset_sub;
        this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file
                + m_ln_read_size - m_p_file_map_info->ln_map_start;

        this->m_ln_cur_line_no = node->data.line_no;
    } else {
        sub_msg->GetMacroProgName(filepath, true);

        sub_msg->SetLastProgFile(this->m_p_file_map_info->str_file_name+strlen(PATH_NC_FILE));  //保存当前文件路径, 相对路径，发送给HMI
        //独立子文件打开
        if (!this->OpenFile(filepath, (bool)m_n_sub_program)){ //尝试打开nc文件失败
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
 * @brief 编译器中运行自动对刀指令消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunAutoToolMeasureMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;

    if(*this->m_p_simulate_mode != SIM_NONE)  //仿真直接返回
        return true;

    if (!this->m_b_prescan_over && m_thread_prescan != 0){ //预扫描未结束，暂不执行

        return false;
    }

    AutoToolMeasureMsg *sub_msg = (AutoToolMeasureMsg *) msg;

    int macro_index = sub_msg->GetMacroProgIndex();      //自动对刀宏程序

    //查找子程序
    int macro_loc = this->FindSubProgram(macro_index);
    sub_msg->SetMacroProgType(macro_loc);
    if (macro_loc == 0 || macro_loc == 1) {
        printf("create error in RunAutoToolMeasureMsg\n");
        CreateErrorMsg(ERR_NO_SUB_PROG, msg->GetLineNo());  //找不到对应子程序
        return false;
    }

    //保存当前编译器状态
    this->SaveScene();

    Variable *pv = m_p_channel_control->GetMacroVar();
    //局部变量入栈
    pv->PushLocalVar();

    //赋值目标位置参数到局部变量
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
    pv->SetVarValue(kMacroParamToLocalVarIndex[Q_DATA], l_data);  //L不可用于宏程序传参，用Q替代

    this->m_n_sub_call_times = 1;//调用次数

    //打开子程序文件
    this->m_n_sub_program = MACRO_PROG;
    char filepath[kMaxPathLen] = { 0 };   //文件路径
    sub_msg->GetMacroProgName(filepath, true);


    //独立子文件打开
    if (!this->OpenFile(filepath, (bool)m_n_sub_program)){ //尝试打开nc文件失败
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
 * @brief 执行坐标系指定指令消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunCoordMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    CoordMsg *tmp = (CoordMsg *) msg;

    int gcode = tmp->GetGCode();
    switch (gcode) {
    case G52_CMD:  		//G52  局部坐标系
        m_compiler_status.mode.gmode[0] = gcode;  //更新模态
        break;
    case G53_CMD:		//G53  机床坐标系
        //处理增量编程指令
        if(m_compiler_status.mode.gmode[3] == G91_CMD){  //增量编程模式
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
            //将目标位置换算为工件坐标系
            this->m_p_channel_control->TransMachCoordToWorkCoord(tmp->GetTargetPos(), m_compiler_status.mode.gmode[14], m_compiler_status.mode.h_mode, tmp->GetAxisMask());
        }
#ifdef USES_FIVE_AXIS_FUNC
        this->ProcessFiveAxisRotPos(tmp->GetTargetPos(), m_compiler_status.cur_pos, tmp->GetAxisMask());
#endif
        //		m_compiler_status.cur_pos = tmp->GetTargetPos(); //更新编译当前位置
        this->SetCurPos(tmp->GetTargetPos());

        m_compiler_status.mode.gmode[0] = gcode;  //更新模态
        break;
    case G92_CMD:  	 	//G92  工件坐标系
        m_compiler_status.mode.gmode[0] = gcode;  //更新模态
        break;
    default:
        if ((gcode >= G54_CMD && gcode <= G59_CMD)
                || (gcode >= G5401_CMD && gcode <= G5499_CMD)){
            tmp->SetLastGCode(m_compiler_status.mode.gmode[14]);   //设置历史值，用于反向手轮引导
            m_compiler_status.mode.gmode[14] = gcode;  //更新模态
        }
        break;
    }

    //	printf("run coord msg: %d\n", gcode);
    return true;
}

/**
 * @brief 执行一般模态指令消息，不带参数的模态指令
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunModeMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    ModeMsg *tmp = (ModeMsg *) msg;
    int gcode = tmp->GetGCode();
    switch (gcode) {
    case G90_CMD:   //绝对编程指令
    case G91_CMD:	//相对编程指令
    case G21_CMD:   //公制单位
    case G20_CMD:   //英制单位
        tmp->SetLastGCode(m_compiler_status.mode.gmode[GetModeGroup(gcode)]);   //设置历史数据，用于手轮反向引导
        m_compiler_status.mode.gmode[GetModeGroup(gcode)] = gcode;
        break;
    case G17_CMD:
    case G18_CMD:
    case G19_CMD:
        tmp->SetLastGCode(m_compiler_status.mode.gmode[GetModeGroup(gcode)]);   //设置历史数据，用于手轮反向引导
        //		this->m_tool_compensate.SetCurPlane(gcode);   //设置刀补模块当前平面
        break;
    case G84_3_CMD:
        tmp->SetLastGCode(m_compiler_status.mode.gmode[39]);   //设置历史数据，用于手轮反向引导
        break;
    default:
        tmp->SetLastGCode(m_compiler_status.mode.gmode[GetModeGroup(gcode)]);   //设置历史数据，用于手轮反向引导
        m_compiler_status.mode.gmode[GetModeGroup(gcode)] = gcode;   //修改编译器G模态
        break;
    }


    return true;
}

/**
 * @brief 执行快速定位指令消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunRapidMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    RapidMsg *tmp = (RapidMsg *) msg;

    //处理增量编程指令
    if(m_compiler_status.mode.gmode[3] == G91_CMD){  //增量编程模式
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
        //将目标位置换算为工件坐标系
        if(tmp->IsMachCoord())
            this->m_p_channel_control->TransMachCoordToWorkCoord(tmp->GetTargetPos(), m_compiler_status.mode.gmode[14],
                m_compiler_status.mode.h_mode, tmp->GetAxisMoveMask());
    }

#ifdef USES_FIVE_AXIS_FUNC
    this->ProcessFiveAxisRotPos(tmp->GetTargetPos(), tmp->GetSourcePos(), tmp->GetAxisMoveMask());
#endif
    m_compiler_status.mode.gmode[1] = G00_CMD;
    m_compiler_status.mode.gmode[9] = G80_CMD;  //自动取消循环指令
    //	m_compiler_status.cur_pos = tmp->GetTargetPos(); //更新编译当前位置
    this->SetCurPos(tmp->GetTargetPos());

    return true;
}

/**
 * @brief 执行直线切削指令消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunLineMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    LineMsg *tmp = (LineMsg *) msg;

    double feed = m_compiler_status.mode.f_mode;
    if(feed > m_p_channel_config->g01_max_speed)
        feed = m_p_channel_config->g01_max_speed;
    tmp->SetFeed(feed);  //设置进给速度

    //处理增量编程指令
    if(m_compiler_status.mode.gmode[3] == G91_CMD){  //增量编程模式
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
        //将目标位置换算为工件坐标系
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
    m_compiler_status.mode.gmode[9] = G80_CMD;  //自动取消循环指令
    //	m_compiler_status.cur_pos = tmp->GetTargetPos(); //更新编译当前位置
    this->SetCurPos(tmp->GetTargetPos());
    //	printf("run line msg : %lf, %lf, %lf, F=%lf\n", tmp->GetTargetPos().GetAxisValue(0), tmp->GetTargetPos().GetAxisValue(1),
    //			tmp->GetTargetPos().GetAxisValue(2), tmp->GetFeed());
    return true;
}

/**
 * @brief 执行圆弧切削指令消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunArcMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    ArcMsg *tmp = (ArcMsg *) msg;
    int gcode = tmp->GetGCode();

    //处理增量编程指令
    if(m_compiler_status.mode.gmode[3] == G91_CMD){  //增量编程模式
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
        //将目标位置换算为工件坐标系
        if(tmp->IsMachCoord())
            this->m_p_channel_control->TransMachCoordToWorkCoord(tmp->GetTargetPos(), m_compiler_status.mode.gmode[14],
                m_compiler_status.mode.h_mode, tmp->GetAxisMoveMask());
    }

#ifdef USES_FIVE_AXIS_FUNC
    this->ProcessFiveAxisRotPos(tmp->GetTargetPos(), tmp->GetSourcePos(), tmp->GetAxisMoveMask());
#endif

    tmp->SetFeed(m_compiler_status.mode.f_mode);

    m_compiler_status.mode.gmode[1] = gcode;
    m_compiler_status.mode.gmode[9] = G80_CMD;  //自动取消循环指令
    //	m_compiler_status.cur_pos = tmp->GetTargetPos(); //更新编译当前位置
    this->SetCurPos(tmp->GetTargetPos());
    return true;
}

/**
 * @brief 执行进给速度指定指令消息
 * @param msg : 消息指针
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
 * @brief 执行进给速度指定指令消息
 * @param msg : 消息指针
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
 * @brief 执行刀补指令消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunCompensateMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    CompensateMsg *tmp = (CompensateMsg *) msg;
    int gcode = tmp->GetGCode();

    //处理增量编程指令
    if(m_compiler_status.mode.gmode[3] == G91_CMD){  //增量编程模式
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
        //将目标位置换算为工件坐标系
        if(tmp->IsMachCoord())
            this->m_p_channel_control->TransMachCoordToWorkCoord(tmp->GetTargetPos(), m_compiler_status.mode.gmode[14],
                m_compiler_status.mode.h_mode, tmp->GetAxisMoveMask());
    }

#ifdef USES_FIVE_AXIS_FUNC
    this->ProcessFiveAxisRotPos(tmp->GetTargetPos(), tmp->GetSourcePos(), tmp->GetAxisMoveMask());
#endif

    if (gcode == G41_CMD || gcode == G42_CMD) {

        tmp->SetCompLastValue(m_compiler_status.mode.d_mode);  //记录历史值
        m_compiler_status.mode.d_mode = tmp->GetCompValue();   //修改编译器状态

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
        //m_p_channel_control->UpdateModeData(D_MODE, d_value);//llx add,由SC直接更新D模态，不经过MC

    } else if (gcode == G43_CMD || gcode == G44_CMD || gcode == G43_4_CMD) {  //刀具长度补偿
        tmp->SetCompLastValue(m_compiler_status.mode.h_mode);  //记录历史值
        if(*m_p_simulate_mode != SIM_NONE) //非仿真模式，刀长补偿才有效
            m_compiler_status.mode.h_mode = tmp->GetCompValue();   //修改编译器状态
    }else if(gcode == G49_CMD){
        tmp->SetCompLastValue(m_compiler_status.mode.h_mode);  //记录历史值
        m_compiler_status.mode.h_mode = 0;  //取消刀长补偿或者RTCP
    }else if(gcode == G40_CMD){
    	tmp->SetCompLastValue(m_compiler_status.mode.d_mode);  //记录历史值
        m_compiler_status.mode.d_mode = 0;   //取消刀具半径补偿
    }

    tmp->SetFeed(m_compiler_status.mode.f_mode);

    m_compiler_status.mode.gmode[GetModeGroup(gcode)] = gcode;   //修改编译器状态
    //	m_compiler_status.cur_pos = tmp->GetTargetPos(); //更新编译当前位置
    this->SetCurPos(tmp->GetTargetPos());
    return true;
}

/**
 * @brief 编译器中运行极坐标插补消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunPolarIntpMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    PolarIntpMsg *tmp = (PolarIntpMsg *) msg;
    int gcode = tmp->GetGCode();

#ifdef USES_GRIND_MACHINE
    if (gcode == G12_2_CMD || gcode == G12_3_CMD) {  //磨削指令

        printf("RunPolarIntpMsg~\n");

    }


#endif

    m_compiler_status.mode.gmode[GetModeGroup(gcode)] = gcode;   //修改编译器状态
    return true;
}

/**
 * @brief 编译器中运行延时消息
 * @param msg
 * @return
 */
bool Compiler::RunTimeWaitMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    TimeWaitMsg *tmp = (TimeWaitMsg *) msg;
    int gcode = tmp->GetGCode();

    m_compiler_status.mode.gmode[0] = gcode;   //修改编译器状态
    return true;
}

/**
 * @brief 编译器中运行回参考点消息
 * @param msg
 * @return
 */
bool Compiler::RunRefReturnMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    RefReturnMsg *tmp = (RefReturnMsg *) msg;
    int gcode = tmp->GetGCode();

    //处理增量编程指令
    if(m_compiler_status.mode.gmode[3] == G91_CMD){  //增量编程模式
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

    m_compiler_status.mode.gmode[0] = gcode;   //修改编译器状态
    return true;
}


/**
 * @brief 编译器中运行跳转消息
 * @param msg
 * @return
 */
bool Compiler::RunSkipMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    SkipMsg *tmp = (SkipMsg *) msg;
    int gcode = tmp->GetGCode();

    //处理增量编程指令
    if(m_compiler_status.mode.gmode[3] == G91_CMD){  //增量编程模式
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
        //将目标位置换算为工件坐标系
        if(tmp->IsMachCoord())
            this->m_p_channel_control->TransMachCoordToWorkCoord(tmp->GetTargetPos(), m_compiler_status.mode.gmode[14],
                m_compiler_status.mode.h_mode, tmp->GetAxisMoveMask());
    }

    tmp->SetFeed(m_compiler_status.mode.f_mode);

    m_compiler_status.mode.gmode[0] = gcode;   //修改编译器状态

    //	m_compiler_status.cur_pos = tmp->GetTargetPos(); //更新编译当前位置
    this->SetCurPos(tmp->GetTargetPos());
    return true;
}

#ifdef USES_SPEED_TORQUE_CTRL	
/**
 * @brief 编译器中运行速度控制消息
 * @param msg : 指令消息
 * @return
 */
bool Compiler::RunSpeedCtrlMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    SpeedCtrlMsg *tmp = (SpeedCtrlMsg *) msg;
    int gcode = tmp->GetGCode();

    m_compiler_status.mode.gmode[kMaxGModeCount-1] = gcode;   //修改编译器状态
    return true;
}


/**
 * @brief 编译器中运行力矩控制消息
 * @param msg : 指令消息
 * @return
 */
bool Compiler::RunTorqueCtrlMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    TorqueCtrlMsg *tmp = (TorqueCtrlMsg *) msg;
    int gcode = tmp->GetGCode();

    m_compiler_status.mode.gmode[kMaxGModeCount-1] = gcode;   //修改编译器状态


    printf("execute Compiler:: RunTorqueCtrlMsg \n");

    return true;
}
#endif

/**
 * @brief 编译器中运行清整数圈位置消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunClearCirclePosMsg(RecordMsg *msg){
    if (msg == nullptr)
        return true;
    ClearCirclePosMsg *tmp = (ClearCirclePosMsg *) msg;
    int gcode = tmp->GetGCode();

    m_compiler_status.mode.gmode[39] = gcode;   //修改编译器状态
    return true;
}

/**
 * @brief 编译器中运行宏指令
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunMacroMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;

    MacroCmdMsg *tmp = (MacroCmdMsg *) msg;
    int macro_cmd = tmp->GetMacroCmd();
    MacroVarValue exp_res;   //暂存宏表达式结果
    uint64_t offset = 0, line_no = 0;
    LoopOffset loop;
    int index = 0;
    bool flag = true;
    int res = 0;

    switch (macro_cmd) {
    case MACRO_CMD_GOTO:	//跳转指令则跳转到相应代码处
        //printf("---------------- goto cmd -------> %llu\n", tmp->GetLineNo());
        if(!tmp->GetMacroExpCalFlag(0)){
            if(!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0), tmp->GetMacroExpResult(0))) {//表达式运算失败
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //表达式计算异常
                    return false;
                }

                //没有错误，则说明表达式中需要访问系统变量，必须等待MC运行到位
                return false;
            }
            tmp->SetMacroExpCalFlag(0, true);
        }


        //表达式运算成功,执行跳转动作
        res = FindJumpLabel(static_cast<int>(tmp->GetMacroExpResult(0).value), offset, line_no);
        if (0 == res) {
            //没有找到跳转点，产生告警
            CreateErrorMsg(ERR_NO_JUMP_LABEL, tmp->GetLineNo());  //表达式计算异常
            return false;
        }else if(2 == res)
            return false;   //等待预扫描结束

        //检查跳转合法性
        if(!this->CheckJumpGoto(tmp->GetLineNo(), line_no)){
            CreateErrorMsg(ERR_JUMP_GOTO_ILLIGAL, tmp->GetLineNo());
            return false;
        }

        if (!this->m_p_file_map_info->JumpTo(offset)) {  //映射失败
            CreateErrorMsg(ERR_JUMP_GOTO, tmp->GetLineNo());  //子程序跳转失败
            return false;
        }

        this->m_ln_read_size = offset;
        this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file
                + m_ln_read_size - m_p_file_map_info->ln_map_start;

        this->m_ln_cur_line_no = line_no;

        break;
    case MACRO_CMD_IF_GOTO: //条件跳转指令需要在此处等待
        if (tmp->GetRunStep() == 0) {
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0),
                                                 tmp->GetMacroExpResult(0))) { //表达式运算失败
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //表达式计算异常
                    return false;
                }

                //没有错误，则说明表达式中需要访问系统变量，必须等待MC运行到位
                return false;
            } else if (static_cast<int>(tmp->GetMacroExpResult(0).value) == 1) { //条件满足
                tmp->SetRunStep(1);  //设置进度
            } else {  //条件不满足，不执行跳转
                break;
            }
        }
        if (tmp->GetRunStep() == 1) {
            if(!tmp->GetMacroExpCalFlag(1)){
                if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(1),
                                                     tmp->GetMacroExpResult(1))) {  //表达式运算失败
                    m_error_code = m_p_parser->GetErrorCode();
                    if (m_error_code != ERR_NONE) {
                        CreateErrorMsg(m_error_code, tmp->GetLineNo());  //表达式计算异常
                        return false;
                    }

                    //没有错误，则说明表达式中需要访问系统变量，必须等待MC运行到位
                    return false;
                }
                tmp->SetMacroExpCalFlag(1, true);
            }
            //执行跳转
            //表达式运算成功,执行跳转动作
            res = FindJumpLabel(static_cast<int>(tmp->GetMacroExpResult(1).value), offset, line_no);
            printf("jump res = %d, result = %lf, offset= %llu, line=%llu\n", res, tmp->GetMacroExpResult(1).value, offset, line_no);
            if (0 == res) {
                //没有找到跳转点，产生告警
                CreateErrorMsg(ERR_NO_JUMP_LABEL, tmp->GetLineNo()); //表达式计算异常
                return false;
            }else if(2 == res){
                return false;   //等待预扫描结束
            }

            //检查跳转合法性
            if(!this->CheckJumpGoto(tmp->GetLineNo(), line_no)){
                CreateErrorMsg(ERR_JUMP_GOTO_ILLIGAL, tmp->GetLineNo());
                return false;
            }

            if (!this->m_p_file_map_info->JumpTo(offset)) {  //映射失败
                CreateErrorMsg(ERR_JUMP_GOTO, tmp->GetLineNo());  //子程序跳转失败
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
                                                 exp_res)) {  //表达式运算失败
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //表达式计算异常
                    return false;
                }

                //没有错误，则说明表达式中需要访问系统变量，必须等待MC运行到位
                return false;
            } else if (static_cast<int>(exp_res.value) == 1) { //条件满足

                tmp->SetRunStep(1);  //设置进度
            } else {  //条件不满足，不执行
                break;
            }
        }
        if (tmp->GetRunStep() == 1) {  //表达式运算成功,执行
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(1),
                                                 exp_res)) {  //表达式运算失败
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //表达式计算异常
                    return false;
                }

                //没有错误，则说明表达式中需要访问系统变量，必须等待MC运行到位
                return false;
            }
        }
        break;
    case MACRO_CMD_WHILE_DO: {
        if (tmp->GetRunStep() == 0) { //判断while条件
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0),
                                                 exp_res)) { //表达式运算失败
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //表达式计算异常
                    return false;
                }

                //没有错误，则说明表达式中需要访问系统变量，必须等待MC运行到位
                return false;
            } else if (static_cast<int>(exp_res.value) == 1) { //条件满足
                //	printf("while ok\n");
                flag = true;
            } else { //条件不满足，跳过循环体
                //	printf("while failed, %lf, %hhu", exp_res.value, exp_res.init);
                flag = false;
            }
            tmp->SetRunStep(1);  //设置进度
        }
        if (tmp->GetRunStep() == 1) {
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(1),
                                                 exp_res)) {  //表达式运算失败
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //表达式计算异常
                    return false;
                }

                //没有错误，则说明表达式中需要访问系统变量，必须等待MC运行到位
                return false;
            } else {
                index = static_cast<int>(exp_res.value);
                if (index >= 0 && index <= kMaxDoIndex) { //条件满足
                    loop.loop_index = index;
                    loop.line_no = tmp->GetLineNo();
                    loop.offset = tmp->GetOffset();
                    if (flag) {
                        //循环位置数据入栈
                        //						printf("push loop\n");
                        this->m_stack_loop.push(loop);
                        //						printf("do push loop, index = %hhu, line=%lld, offset= %lld\n", loop.loop_index, loop.line_no, loop.offset);
                    } else {
                        printf("jump to end cmd\n");
                        //跳转到对应的END指令
                        if (!this->JumpToLoopEnd(loop)) {
                            CreateErrorMsg(ERR_MACRO_DO_END_MATCH,
                                           tmp->GetLineNo());  //表达式计算异常
                            return false;
                        }
                    }
                } else {  //DO识别号超范围，告警
                    CreateErrorMsg(ERR_MACRO_DO_OUT, tmp->GetLineNo()); //DO识别号超范围
                    return false;
                }

            }

        }
    }
        break;
    case MACRO_CMD_END:
        if (tmp->GetRunStep() == 0) {
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0),
                                                 exp_res)) {  //表达式运算失败
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //表达式计算异常
                    return false;
                }

                //没有错误，则说明表达式中需要访问系统变量，必须等待MC运行到位
                return false;
            } else {
                index = static_cast<int>(exp_res.value);
                if (index >= 0 && index <= kMaxDoIndex) { //条件满足
                    //返回循环开始处
                    if (this->m_stack_loop.size() == 0) {
                        CreateErrorMsg(ERR_MACRO_DO_END_MATCH,
                                       tmp->GetLineNo());  //DO-END指令不匹配
                        return false;
                    }

                    //跳转
                    if (m_stack_loop.pop(loop)) {
                        if (!this->JumpToLoopHead(loop)) {
                            CreateErrorMsg(ERR_JUMP_END, tmp->GetLineNo()); //跳转失败
                            return false;
                        }
                    }
                } else {  //DO-END识别号超范围，告警
                    CreateErrorMsg(ERR_MACRO_DO_OUT, tmp->GetLineNo()); //DO-END识别号超范围
                    return false;
                }
            }
        }
        break;
    case MACRO_CMD_DO:  //省略WHILE则无限循环
        if (tmp->GetRunStep() == 0) {
            if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0),
                                                 exp_res)) {  //表达式运算失败
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //表达式计算异常
                    return false;
                }

                //没有错误，则说明表达式中需要访问系统变量，必须等待MC运行到位
                return false;
            } else {
                index = static_cast<int>(exp_res.value);
                if (index >= 0 && index <= kMaxDoIndex) { //条件满足
                    //循环位置数据入栈
                    loop.loop_index = index;
                    loop.line_no = tmp->GetLineNo();
                    loop.offset = tmp->GetOffset();
                    this->m_stack_loop.push(loop);
                } else {					//DO识别号超范围，告警
                    CreateErrorMsg(ERR_MACRO_DO_OUT, tmp->GetLineNo()); //DO识别号超范围
                    return false;
                }
            }
        }
        break;
    case MACRO_CMD_EXP:		//纯宏表达式
        if (!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0), exp_res)) {//表达式运算失败
            m_error_code = m_p_parser->GetErrorCode();
            if (m_error_code != ERR_NONE) {
                CreateErrorMsg(m_error_code, tmp->GetLineNo());  //表达式计算异常
                return false;
            }

            //没有错误，则说明表达式中需要访问系统变量，必须等待MC运行到位
            return false;
        }
        break;
    case MACRO_CMD_IF:{
    	//@test
    	printf("===== IF CMD %llu\n", tmp->GetLineNo());
        if(!tmp->GetMacroExpCalFlag(0)){
            if(!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0), tmp->GetMacroExpResult(0))) {//表达式运算失败
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //表达式计算异常
                    return false;
                }
                //没有错误，则说明表达式中需要访问系统变量，必须等待MC运行到位
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
                // 条件成立 跳转到 node vector 的 back（endif节点）
                node = m_node_vectors_vector.at(node.vec_index).back();
                m_b_else_jump = true;                     // 遇到 else elseif 跳转到对应的endif上

            }else{
                //条件不成立     往下一个节点跳转
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
            printf("IF ELSE 不匹配, 没有if入栈但遇到  else/ elseif\n");
            CreateErrorMsg(IF_ELSE_MATCH_FAILED, tmp->GetLineNo());
            return false;
        }

        IfElseOffset node;
        node = m_node_stack_run.back();

        if(node.line_no == 0){
            printf("m_stack_ifelse_node 栈出错！！！\n");
            break;
        }

        // 之前的 if 已经满足条件  直接跳转到 endif
        if(m_b_else_jump){

            if(!JumpLine(node.line_no, node.offset, tmp))
                printf("------ jump line failed!!!\n");

            break;
        }

        if(!tmp->GetMacroExpCalFlag(0)){
            if(!m_p_parser->GetExpressionResult(tmp->GetMacroExp(0), tmp->GetMacroExpResult(0))) {//表达式运算失败
                m_error_code = m_p_parser->GetErrorCode();
                if (m_error_code != ERR_NONE) {
                    CreateErrorMsg(m_error_code, tmp->GetLineNo());  //表达式计算异常
                    return false;
                }

                //没有错误，则说明表达式中需要访问系统变量，必须等待MC运行到位
                return false;
            }
            tmp->SetMacroExpCalFlag(0, true);

            // 条件成立
            if(static_cast<int>(tmp->GetMacroExpResult(0).value) == 1){
                //printf("cmd  elseif: 1111\n");
                m_b_else_jump = true;
                m_else_jump_stack_run.pop_back();
                m_else_jump_stack_run.push_back(m_b_else_jump);
                // 找到 node_vector endif 节点
                node = m_node_vectors_vector.at(node.vec_index).back();
                m_node_stack_run.pop_back();
                m_node_stack_run.push_back(node);

                // 条件不成立
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
            printf("IF ELSE 不匹配, 没有if入栈但遇到  else/ elseif 222\n");
            CreateErrorMsg(IF_ELSE_MATCH_FAILED, tmp->GetLineNo());
            return false;
        }

        if(m_b_else_jump){
            IfElseOffset node;
            node = m_node_stack_run.back();
            if(node.line_no == 0){
                printf("m_stack_ifelse_node 栈出错！！！\n");
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
            printf("IF ELSE 不匹配, 没有if入栈但遇到  endif \n");
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
        CreateErrorMsg(ERR_NC_FORMAT, tmp->GetLineNo());  //无法解析的宏指令
        return false;
    }

    return true;
}

bool Compiler::JumpLine(int line_no, uint64_t offset, MacroCmdMsg *tmp){
    //检查跳转合法性
    if(!this->CheckJumpGoto(tmp->GetLineNo(), line_no)){
        CreateErrorMsg(ERR_JUMP_GOTO_ILLIGAL, tmp->GetLineNo());
        return false;
    }

    if (!this->m_p_file_map_info->JumpTo(offset)) {  //映射失败
        CreateErrorMsg(ERR_JUMP_GOTO, tmp->GetLineNo());  //子程序跳转失败
        return false;
    }

    this->m_ln_read_size = offset;
    this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file
            + m_ln_read_size - m_p_file_map_info->ln_map_start;

    this->m_ln_cur_line_no = line_no;
    return true;
}

/**
 * @brief 执行刀具指令消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunToolMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    ToolMsg *tmp = (ToolMsg *) msg;


#ifdef USES_T_CODE_MACRO
    if (!this->m_b_prescan_over && m_thread_prescan != 0) //预扫描未结束，暂不执行
        return false;

    int sub_index = tmp->GetSubProgIndex();   //T指令对应调用O9000.nc子程序

    //查找子程序
    int sub_loc = this->FindSubProgram(sub_index);
    tmp->SetSubProgType(sub_loc);
    if (sub_loc == 0) {
        CreateErrorMsg(ERR_NO_SUB_PROG, msg->GetLineNo());  //找不到对应子程序
        return false;
    }

    //保存当前编译器状态
    this->SaveScene();

    //局部变量入栈
    m_p_channel_control->GetMacroVar()->PushLocalVar();

    this->m_n_sub_call_times = 1;//调用次数

    //赋值T参数到局部变量#20，全局变量#146
    int t_data = tmp->GetTool();
    m_p_channel_control->GetMacroVar()->SetVarValue(kMacroParamToLocalVarIndex[T_DATA], t_data);
    m_p_channel_control->GetMacroVar()->SetVarValue(146, t_data);



    //打开子程序文件
    this->m_n_sub_program = MACRO_PROG;
    char filepath[kMaxPathLen] = { 0 };   //文件路径
    if (sub_loc == 1) {   //同文件子程序
        ListNode < SubProgOffset > *node = m_p_list_subprog->HeadNode();
        uint64_t offset_sub = 0;
        while (node != nullptr) {
            if (node->data.sub_index == sub_index) {
                offset_sub = node->data.offset;
                break;
            }
            node = node->next;
        }

        if (!this->m_p_file_map_info->JumpTo(offset_sub)) {   //映射失败
            CreateErrorMsg(ERR_JUMP_SUB_PROG, msg->GetLineNo());  //子程序跳转失败
            return false;
        }

        this->m_ln_read_size = offset_sub;
        this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file
                + m_ln_read_size - m_p_file_map_info->ln_map_start;

        this->m_ln_cur_line_no = node->data.line_no;
    } else {
        tmp->GetSubProgName(filepath, true);


        //		tmp->SetLastProgFile(this->m_p_file_map_info->str_file_name+strlen(PATH_NC_FILE));  // 保存当前文件路径，相对路径
        //独立子文件打开
        if (!this->OpenFile(filepath, (bool)m_n_sub_program)){ //尝试打开nc文件失败
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
 * @brief 执行循环指令消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunLoopMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;

    LoopMsg *sub_msg = (LoopMsg *) msg;
    int gcode = sub_msg->GetGCode();

    //TODO  调用循环指令子程序
    if (!this->m_b_prescan_over && m_thread_prescan != 0){ //预扫描未结束，暂不执行
        return false;
    }

    int macro_index = sub_msg->GetMacroProgIndex();

    //查找子程序
    int macro_loc = this->FindSubProgram(macro_index);
    sub_msg->SetMacroProgType(macro_loc);
    if (macro_loc == 0) {
        CreateErrorMsg(ERR_INVALID_CODE, msg->GetLineNo());  //找不到对应子程序
        return false;
    }

    if(gcode != G80_CMD)
        this->SaveLoopParam(sub_msg);  //保存参数到全局变量
    else
        this->ResetLoopParam();   //G80复位参数

    m_compiler_status.mode.gmode[GetModeGroup(gcode)] = gcode;   //修改编译器G模态

    //保存当前编译器状态
    this->SaveScene();

    Variable *pv = m_p_channel_control->GetMacroVar();
    //局部变量入栈

    // @test zk  将当前局部变量存到scene中  清零当前局部变量。
    pv->PushLocalVar();


    //赋值自变量参数到局部变量
    uint8_t pc = 0;
    uint32_t pm = 0;
    int i  = 0;
    double *pp = sub_msg->GetParameter(pm, pc);
    //检查是否有F变量，没有的话，将当前的Feed值赋给F变量
    if((pm & (0x01<<F_DATA)) == 0){  //没有F变量，即未指定Feed
        pv->SetVarValue(kLoopParamToLocalVarIndex[F_DATA], m_compiler_status.mode.f_mode);
    }

    //写入忽略的轴坐标参数
    uint8_t chn_axis_count = this->m_p_channel_control->GetChnAxisCount();
    uint8_t chn_axis_data = 0;

    // @zk 这一步会将当前坐标值保存到对应的局部变量 (指定 X_ Y_ Z_ ...)
    for(uint8_t i = 0; i < chn_axis_count; i++){
        chn_axis_data = this->MapAxisNameToParamIdx(m_p_channel_control->GetChnAxisName(i));

        if((pm & (0x01<<chn_axis_data)) == 0){
            pv->SetVarValue(kLoopParamToLocalVarIndex[chn_axis_data], m_compiler_status.cur_pos.GetAxisValue(i));
        }
    }
    // @zk 将固定循环指令指定的参数 保存到局部变量 (覆盖 X_ Y_ Z_ ...)
    while(pm != 0){
        if(pm & 0x01){
            pv->SetVarValue(kLoopParamToLocalVarIndex[i], *pp);
            pp++;
        }
        pm = pm>>1;
        i++;
    }

    this->m_n_sub_call_times = 1;//调用次数

    //打开子程序文件
    this->m_n_sub_program = MACRO_PROG;
    char filepath[kMaxPathLen] = { 0 };   //文件路径

    sub_msg->GetMacroProgName(filepath, true);

    sub_msg->SetLastProgFile(this->m_p_file_map_info->str_file_name+strlen(PATH_NC_FILE));  //保存当前文件路径, 相对路径，发送给HMI
    //独立子文件打开
    if (!this->OpenFile(filepath, (bool)m_n_sub_program)){ //尝试打开nc文件失败
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
 * @brief 执行错误指令消息
 * @param msg : 消息指针
 * @return
 */
bool Compiler::RunErrorMsg(RecordMsg *msg) {
    if (msg == nullptr)
        return true;
    ErrorMsg *tmp = static_cast<ErrorMsg *>(msg);

    //
    if(tmp->GetInfoType() == 1){  //错误信息
        this->m_error_code = (ErrorType)tmp->GetErrorCode();
    }
    printf("run error message! errcode = %d, infotype = %d, line = %lld\n", m_error_code, tmp->GetInfoType(), tmp->GetLineNo());

    return true;
}

/**
 * @brief 构造编译错误消息，并插入消息队列
 * @param err : 错误码
 * @param line_no : 错误行号
 * @return true--成功  false--失败
 */
bool Compiler::CreateErrorMsg(const ErrorType err, uint64_t line_no) {
    RecordMsg *new_msg = new ErrorMsg(err);
    if (new_msg == nullptr) {
        //TODO 内存分配失败，告警
        m_error_code = ERR_MEMORY_NEW;
        g_ptr_trace->PrintLog(LOG_ALARM, "CHN[%d]编译器告警分配内存失败！",
                              m_n_channel_index);
        CreateError(ERR_MEMORY_NEW, FATAL_LEVEL, CLEAR_BY_RESET_POWER, 0,
                    m_n_channel_index);
        return false;
    }
    new_msg->SetLineNo(line_no);  //设置当前行号
    m_error_code = err;

    //TODO  直接推入下一步骤
    return this->m_p_block_msg_list->Append(new_msg);
}

/**
 * @brief 处理代码头部信息
 * @return 执行结果
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
 * @brief 处理NC代码主体
 * @return 执行结果
 */
bool Compiler::ProcessMain() {
    bool res = true;

    //TODO 处理跳段
    if (this->m_line_buf[0] == '/') {
        //跳段处理
        this->m_compiler_status.jump_flag = true;
        int len = strlen(m_line_buf);
        int index = 1;
        for (; index < len; index++) {
            if (!isspace(m_line_buf[index]))  //找到第一个非空格字符
                break;
        }
        if (index < len)
            strcpy(m_line_buf, m_line_buf + index);
    }

    //首先进行词法解析
    if (!DoLexer()) {
        res = false;
    } else {
        //进行语法解析
        if (!DoParser()) {
            res = false;
        }
    }

    return res;
}

/**
 * @brief 编译头部信息
 * @return 执行结果
 */
bool Compiler::CompileHead() {
    bool res = true;
    //  const char* str = "\x20:,";

    res = GetHeadInfo();  //首先获取头部附加信息
    if (!res) {
        return res;
    }

    res = CheckHead();
    if (res) {
        m_n_compile_state = FILE_MAIN;  //切换到编译主程序状态
    }

    return res;
}

/**
 * @brief 编译主体代码
 * @return 执行结果
 */
//bool Compiler::CompileMain(){
//	bool res = true;
//
//
//
//	return res;
//}


/**
 * @brief 检查程序头部合法性
 * @return 执行结果
 */
bool Compiler::CheckHead() {
    bool res = false;
    char *compBuf = m_line_buf;

    if (strcmp(compBuf, "%") == 0) {
        res = true; //主程序
    }else if(m_n_sub_program != MAIN_PROG){  //在子程序中，可以无%
        res = this->ProcessMain(); //
    }else {
#ifdef USES_WOOD_MACHINE
        res = this->ProcessMain();
#else
        if (m_b_check) {
            m_error_code = ERR_NO_START;   //无起始%
        }
#endif

    }

    return res;
}

/**
 * @brief 解析获取头部的附加信息，包括刀路合并信息
 * @return 执行结果
 */
bool Compiler::GetHeadInfo() {
    bool res = true;
    //	char* compBuf = m_line_buf;

    //增加换刀坐标切换头部信息解析
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
 * @brief 进行词法解析
 * @return 执行结果
 */
bool Compiler::DoLexer() {
    bool res = true;

    res = m_p_lexer->Compile();
    if (!res) {
        printf("lexer error\n");
        m_error_code = ERR_COMPILER_INTER;  //编译器内部错误
    }
    return res;
}

/**
 * @brief 进行语法解析
 * @return 执行结果
 */
bool Compiler::DoParser() {
    bool res = true;

    res = m_p_parser->Compile();
    if (!res) { //产生非语法错误
        m_error_code = ERR_COMPILER_INTER;  //编译器内部错误
    }
    return res;
}

/**
 * @brief 调用子程序
 * @param sub_name : 子程序号
 * @return  true--成功   false--失败
 */
//bool Compiler::CallSubProgram(int sub_name){
//
//
//	return true;
//}

/**
 * @brief 无参数调用宏程序
 * @param macro_index : 宏程序编号
 * @param flag : 通知HMI文件变更， true--通知   false--不通知
 * @return 0--调用失败   其它--调用成功
 */
int Compiler::CallMarcoProgWithNoPara(int macro_index, bool flag){
    //查找子程序
    int macro_loc = this->FindSubProgram(macro_index);

    if (macro_loc == 0 || macro_loc == 1) {
        printf("create error in CallMarcoProgWithNoPara\n");
        CreateErrorMsg(ERR_NO_SUB_PROG, 0);  //找不到对应子程序
        return 0;
    }

    //保存当前编译器状态
    this->SaveScene();

    Variable *pv = m_p_channel_control->GetMacroVar();
    //局部变量入栈
    pv->PushLocalVar();

    this->m_n_sub_call_times = 1;//调用次数

    //打开子程序文件
    this->m_n_sub_program = MACRO_PROG;
    char filepath[kMaxPathLen] = { 0 };   //文件路径
    GetMacroSubProgPath(macro_loc, macro_index, true, filepath);

    //独立子文件打开
    if (!this->OpenFile(filepath, (bool)m_n_sub_program)){ //尝试打开nc文件失败
        return 0;
    }

    if (m_work_mode == AUTO_COMPILER)
        this->m_n_compile_state = FILE_HEAD;
    else
        this->m_n_compile_state = FILE_MAIN;
    this->m_n_head_state = HEAD_INFO;

    //TODO 向HMI发送命令打开子文件
    if(flag && g_ptr_parm_manager->GetSystemConfig()->debug_mode > 0){ //调式模式下，打开宏程序文件
        this->m_p_channel_control->SendOpenFileCmdToHmi(filepath);
    }

    return macro_loc;
}

/**
 * @brief 获取宏程序的路径数据
 * @param macro_group : 宏程序类型，1--在本程序内   2/6--同目录下nc文件    3/7--系统子程序目录nc文件    4/8--同目录下iso文件    5/9--系统子程序目录iso文件
 * @param macro_index : 宏程序号
 * @param abs_path : 是否绝对路径
 * @param name[out] : 输出宏程序文件路径
 */
void Compiler::GetMacroSubProgPath(int macro_group, int macro_index, bool abs_path, char *name){
    if(abs_path){
        if(macro_group == 2){//同目录下用户宏程序
            sprintf(name, "%sO%04d.nc", PATH_NC_FILE, macro_index);   //拼接文件名称
        }else if(macro_group == 3){//系统宏程序
            sprintf(name, "%ssys_sub/O%04d.nc", PATH_NC_FILE, macro_index);
        }else if(macro_group == 4){
            sprintf(name, "%sO%04d.iso", PATH_NC_FILE, macro_index);   //拼接文件名称
        }else if(macro_group == 5){
            sprintf(name, "%ssys_sub/O%04d.iso", PATH_NC_FILE, macro_index);
        }else if(macro_group == 6){//同目录下用户宏程序
            sprintf(name, "%sO%04d.NC", PATH_NC_FILE, macro_index);   //拼接文件名称
        }else if(macro_group == 7){//系统宏程序
            sprintf(name, "%ssys_sub/O%04d.NC", PATH_NC_FILE, macro_index);
        }else if(macro_group == 8){
            sprintf(name, "%sO%04d.ISO", PATH_NC_FILE, macro_index);   //拼接文件名称
        }else if(macro_group == 9){
            sprintf(name, "%ssys_sub/O%04d.ISO", PATH_NC_FILE, macro_index);
        }
    }else{
        if(macro_group == 2){//同目录下用户宏程序
            sprintf(name, "O%04d.nc", macro_index);   //拼接文件名称
        }else if(macro_group == 3){//系统宏程序
            sprintf(name, "sys_sub/O%04d.nc", macro_index);
        }else if(macro_group == 4){
            sprintf(name, "O%04d.iso", macro_index);   //拼接文件名称
        }else if(macro_group == 5){
            sprintf(name, "sys_sub/O%04d.iso", macro_index);
        }else 	if(macro_group == 6){//同目录下用户宏程序
            sprintf(name, "O%04d.NC", macro_index);   //拼接文件名称
        }else if(macro_group == 7){//系统宏程序
            sprintf(name, "sys_sub/O%04d.NC", macro_index);
        }else if(macro_group == 8){
            sprintf(name, "O%04d.ISO", macro_index);   //拼接文件名称
        }else if(macro_group == 9){
            sprintf(name, "sys_sub/O%04d.ISO", macro_index);
        }
    }
}


/**
 * @brief 子程序返回处理
 * @return  true--成功   false--失败
 */
bool Compiler::ReturnFromSubProg() {
    printf("Return from sub program, sub_prog=%d, call_time=%d\n", m_n_sub_program, m_n_sub_call_times);
    if (this->m_n_sub_program != MAIN_PROG) {

        if(--m_n_sub_call_times > 0){  //多次调用未结束
            this->RecycleCompile();
            return true;
        }

        //预扫描线程是否结束，未结束则退出
        void* thread_result;
        int res = 0;
        if (!m_b_prescan_over && m_thread_prescan != 0) {//预扫描线程未结束，退出并等待其结束
            //先退出之前的线程
            m_b_breakout_prescan = true;
            int wait_count = 0;

            while (m_b_breakout_prescan && wait_count++ < 200)
                usleep(1000);  //等待线程退出
            if (m_b_breakout_prescan) {//等待退出失败，则主动cancel线程

                //退出预扫描运行线程
                res = pthread_cancel(m_thread_prescan);
                if (res != 0) {
                    g_ptr_trace->PrintLog(LOG_ALARM, "预扫描线程退出失败3！errno = %d\n",
                                          res);
                    printf("预扫描线程退出失败7777\n");
                    CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                                0, m_n_channel_index);
                    return false;
                }

                usleep(1000);
            }
            res = pthread_join(m_thread_prescan, &thread_result);  //等待线程退出完成
            if (res != 0) {
                g_ptr_trace->PrintLog(LOG_ALARM, "预扫描线程退出失败4！errno = %d\n",
                                      res);
                printf("预扫描线程退出失败8888\n");
                CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                            0, m_n_channel_index);
                return false;
            }
            m_thread_prescan = 0;

        }else if(m_b_prescan_over && m_thread_prescan != 0){  //释放预扫描线程资源
            res = pthread_join(m_thread_prescan, &thread_result);  //等待线程退出完成
            if (res != 0) {
                g_ptr_trace->PrintLog(LOG_ALARM, "预扫描线程资源释放失败2！errno = %d\n",
                                      res);
                printf("预扫描线程退出失败9999\n");
                CreateError(ERR_QUIT_PRESCAN, ERROR_LEVEL, CLEAR_BY_RESET_POWER,
                            0, m_n_channel_index);
            }
            m_thread_prescan = 0;
        }
        m_b_breakout_prescan = false;
        m_b_prescan_over = false;

        printf("exit pre scan thread\n");

        bool ret_macro_prog = (m_n_sub_program==MACRO_PROG)?true:false;    //返回的调用类型

        //局部变量出栈
        // @test 恢复栈顶局部变量
        m_p_channel_control->GetMacroVar()->PopLocalVar();

        //子程序结束，恢复现场
        if (!this->ReloadScene(false)) {
            g_ptr_trace->PrintLog(LOG_ALARM, "子程序结束，恢复现场失败！");
            //TODO 报错
            CreateError(ERR_SUB_BACK, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0,
                        m_n_channel_index);
            return false;
        }

        this->m_p_cur_file_pos = this->m_p_file_map_info->ptr_map_file
                + this->m_ln_read_size - m_p_file_map_info->ln_map_start;


        //从子程序返回，插入子程序返回消息，主要功能是通知HMI打开上一级调用文件
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
 * @brief 查找子程序,子程序的查找顺序：1.本程序内  2. NC程序目录  3. 系统子程序目录
 * @param sub_name : 子程序号
 * @return 0--没找到对应子程序   1--在本程序内   2--同目录下nc文件    3--系统子程序目录nc文件    4--同目录下iso文件    5--系统子程序目录iso文件
 *                                               6--同目录下NC文件    7--系统子程序目录NC文件    8--同目录下ISO文件    9--系统子程序目录ISO文件
 */
int Compiler::FindSubProgram(int sub_name, bool file_only) {   //查找并打开子程序

    //在程序内部搜索
    if(!file_only){
        ListNode < SubProgOffset > *node = m_p_list_subprog->HeadNode();
        while (node != nullptr) {
            if (node->data.sub_index == sub_name) {
                return 1;
            }
            node = node->next;
        }
    }

    //同目录下搜索
    //nc文件
    char filepath[kMaxPathLen] = { 0 };   //文件路径
    if (sub_name <= 9999)
        sprintf(filepath, "%sO%04d.nc", PATH_NC_FILE, sub_name);   //拼接文件绝对路径
    else
        sprintf(filepath, "%sO%d.nc", PATH_NC_FILE, sub_name);   //拼接文件绝对路径

    printf("sub program file 1: %s\n", filepath);
    if (access(filepath, F_OK) == 0) {	//存在对应文件
        return 2;
    }else{//后缀大写
        memset(filepath, 0x00, kMaxPathLen);
        if (sub_name <= 9999)
            sprintf(filepath, "%sO%04d.NC", PATH_NC_FILE, sub_name);   //拼接文件绝对路径
        else
            sprintf(filepath, "%sO%d.NC", PATH_NC_FILE, sub_name);   //拼接文件绝对路径

        if (access(filepath, F_OK) == 0) {	//存在对应文件
            return 6;
        }
    }

    //iso文件
    memset(filepath, 0x00, kMaxPathLen);
    if (sub_name <= 9999)
        sprintf(filepath, "%sO%04d.iso", PATH_NC_FILE, sub_name);   //拼接文件绝对路径
    else
        sprintf(filepath, "%sO%d.iso", PATH_NC_FILE, sub_name);   //拼接文件绝对路径

    printf("sub program file 2: %s\n", filepath);
    if (access(filepath, F_OK) == 0) {	//存在对应文件
        return 4;
    }else{
        memset(filepath, 0x00, kMaxPathLen);
        if (sub_name <= 9999)
            sprintf(filepath, "%sO%04d.ISO", PATH_NC_FILE, sub_name);   //拼接文件绝对路径
        else
            sprintf(filepath, "%sO%d.ISO", PATH_NC_FILE, sub_name);   //拼接文件绝对路径

        if (access(filepath, F_OK) == 0) {	//存在对应文件
            return 8;
        }
    }

    //系统子程序目录下搜索
    memset(filepath, 0x00, kMaxPathLen);
    if (sub_name <= 9999)
        sprintf(filepath, "%sO%04d.nc", PATH_NC_SUB_FILE, sub_name);  //拼接文件绝对路径
    else
        sprintf(filepath, "%sO%d.nc", PATH_NC_SUB_FILE, sub_name);   //拼接文件绝对路径
    printf("sys sub program file 3: %s\n", filepath);
    if (access(filepath, F_OK) == 0) {	//存在对应文件
        return 3;
    }else{
        memset(filepath, 0x00, kMaxPathLen);
        if (sub_name <= 9999)
            sprintf(filepath, "%sO%04d.NC", PATH_NC_SUB_FILE, sub_name);  //拼接文件绝对路径
        else
            sprintf(filepath, "%sO%d.NC", PATH_NC_SUB_FILE, sub_name);   //拼接文件绝对路径

        if (access(filepath, F_OK) == 0) {	//存在对应文件
            return 7;
        }
    }

    //iso文件
    memset(filepath, 0x00, kMaxPathLen);
    if (sub_name <= 9999)
        sprintf(filepath, "%sO%04d.iso", PATH_NC_SUB_FILE, sub_name);  //拼接文件绝对路径
    else
        sprintf(filepath, "%sO%d.iso", PATH_NC_SUB_FILE, sub_name);   //拼接文件绝对路径
    printf("sys sub program file 4: %s\n", filepath);
    if (access(filepath, F_OK) == 0) {	//存在对应文件
        return 5;
    }else{
        memset(filepath, 0x00, kMaxPathLen);
        if (sub_name <= 9999)
            sprintf(filepath, "%sO%04d.ISO", PATH_NC_SUB_FILE, sub_name);  //拼接文件绝对路径
        else
            sprintf(filepath, "%sO%d.ISO", PATH_NC_SUB_FILE, sub_name);   //拼接文件绝对路径

        if (access(filepath, F_OK) == 0) {	//存在对应文件
            return 9;
        }
    }

    return 0;
}

/**
 * @brief 查找跳转点位置
 * @param label[in] : 跳转点顺序号
 * @param offset[out] : 跳转点偏移
 * @param line_no[out] : 跳转点行号
 * @return 0--没找到    1--找到   2--等待预扫描结束
 */
int Compiler::FindJumpLabel(int label, uint64_t &offset, uint64_t &line_no) {
    //查找跳转点位置
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

    if(res != 1 && this->m_b_prescan_over)  //预扫描已结束，但是未找到跳转点
        res = 0;

    return res;
}

/**
 * @brief 编译跳转到循环头部位置
 * @param loop : 循环偏移位置
 * @return
 */
bool Compiler::JumpToLoopHead(LoopOffset &loop) {
    if (!this->m_p_file_map_info->JumpTo(loop.offset)) {	//映射失败
        return false;
    }

    this->m_ln_read_size = loop.offset;
    this->m_p_cur_file_pos = m_p_file_map_info->ptr_map_file + m_ln_read_size
            - m_p_file_map_info->ln_map_start;

    this->m_ln_cur_line_no = loop.line_no;

    return true;
}

/**
 * @brief 编译跳转到循环末尾
 * @param loop : 循环偏移位置
 * @return
 */
bool Compiler::JumpToLoopEnd(LoopOffset &loop) {
    bool flag = false;
    int idx = 0;
    MacroVarValue exp_res;
    LoopOffsetStack stack_loop;   //处理循环嵌套，暂存嵌套的WHILE_DO 或者DO
    LoopOffset lt;

    stack_loop.empty();
    while (this->GetLineData()) {
        if (strcasestr(this->m_line_buf, "DO") != nullptr) {	//有DO指令
            if(DoLexer()){//词法分析成功
                if (m_lexer_result.macro_flag){//宏表达式
                    if(m_lexer_result.nc_code.macro_cmd.cmd == MACRO_CMD_WHILE_DO){
                        idx = 1;
                    }else if(m_lexer_result.nc_code.macro_cmd.cmd == MACRO_CMD_DO) {
                        idx = 0;
                    }
                    if (m_p_parser->GetExpressionResult(
                                m_lexer_result.nc_code.macro_cmd.macro_expression[idx],
                                exp_res)) {   //表达式运算失败
                        lt.loop_index = static_cast<int>(exp_res.value);
                        stack_loop.push(lt);  //入栈
                    }else
                        break;
                }
            }
            else {
                break;
            }
        }else if (strcasestr(this->m_line_buf, "END") != nullptr) {	//有END指令
            if (DoLexer()) {   //词法分析成功
                if (m_lexer_result.macro_flag
                        && m_lexer_result.nc_code.macro_cmd.cmd
                        == MACRO_CMD_END) {
                    if (m_p_parser->GetExpressionResult(
                                m_lexer_result.nc_code.macro_cmd.macro_expression[0],
                                exp_res)) {
                        if(stack_loop.size() > 0){//有嵌套的循环指令
                            if(stack_loop.pop(lt)){
                                if(lt.loop_index != static_cast<int>(exp_res.value)){  //DO-END不匹配
                                    g_ptr_trace->PrintLog(LOG_ALARM, "END指令顺序号不匹配！");
                                    break;
                                }
                            }
                        }else if (static_cast<int>(exp_res.value) == loop.loop_index) {
                            flag = true;
                            break;
                        } else {
                            g_ptr_trace->PrintLog(LOG_ALARM, "END指令顺序号不匹配！");
                            break;
                        }
                    }else//表达式运算失败
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
 * @brief 检查跳转的目的行的合法性.1. 可以从循环体内调转到循环体外也可以在循环体内跳转； 2.不可以从循环体外跳转到循环体内
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
        if (line_des > node->data.start_line_no && line_des < node->data.end_line_no) {  //目的行在循环体内
            if(line_src < node->data.start_line_no || line_src > node->data.end_line_no ){ //跳转起点在循环体外
                res = false;
                break;
            }
        }else if(line_src > node->data.start_line_no && line_src < node->data.end_line_no){
            //跳出了循环体，则需要将已入栈的循环体起点DO弹出
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

    //  处理 if 中 goto
    for(vector<IfElseOffset> node_vector: m_node_vectors_vector){
        // 跳转终点在 一组 if (elseif\else\endif) 之间
        if(line_des > node_vector.at(0).line_no and line_des < node_vector.at(1).line_no){
            // 跳转起点不在 这组 if (elseif\else\endif) 之间
            if(line_src < node_vector.at(0).line_no or line_src > node_vector.at(1).line_no){
                res = false;
                break;
            }
        }
    }

    while(m_node_stack_run.size() != 0){
        // 目标行号 大于 栈顶节点记录行号  弹出

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
 * @brief 关闭当前文件映射
 */
void Compiler::UnmapCurNcFile(){
    m_p_file_map_info->UnmapFile();
}

/**
 * @brie 重新映射nc文件
 * @return
 */
bool Compiler::RemapCurNcFile(){
    return m_p_file_map_info->RemapFile();
}

/**
 * @brief 设置轴名称扩展下标使能
 * @param flag : 是否允许轴名称扩展下标
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
 * @brief 设置加工复位参数
 * @param line : 复位行号
 * @param mode ：复位模式
 */
void Compiler::SetRestartPara(const uint64_t line, const uint8_t mode){
    this->m_n_restart_line = line;
    this->m_n_restart_mode = mode;
}

/**
 * @brief 同步起点位置
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
        node = node->next;  //取下一个消息
    }

    this->SetCurPos(pp);   //更新编译器起点坐标
    //	printf("refresh block pos:%lf, %lf, %lf\n", pp.m_df_point[0], pp.m_df_point[1], pp.m_df_point[2]);
    return res;
}


/**
 * @brief 将轴名称映射为变量ID
 * @param axis_name : 轴名称
 * @return ：参数ID, 返回0xFF则表明转换失败
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
    case AXIS_NAME_Z:			//Z轴
        para = Z_DATA;
        break;
    case AXIS_NAME_A:			//A轴
        para = A_DATA;
        break;
    case AXIS_NAME_B:			//B轴
        para = B_DATA;
        break;
    case AXIS_NAME_C:			//C轴
        para = C_DATA;
        break;
    case AXIS_NAME_U:			//U轴
        para = U_DATA;
        break;
    case AXIS_NAME_V:			//V轴
        para = V_DATA;
        break;
    case AXIS_NAME_W:				//W轴
        para = W_DATA;
        break;
    default:
        break;
    }

    return para;
}

/**
 * @brief 保存循环指令的参数到宏变量#171~#195
 * @param loop : 循环指令消息
 */
void Compiler::SaveLoopParam(LoopMsg *loop){
    if(loop == nullptr)
        return;
    uint8_t pc = 0;   //参数个数
    uint32_t pm = 0;  //参数掩码
    int i  = 0;
    double *pp = loop->GetParameter(pm, pc);
    Variable *pv = m_p_channel_control->GetMacroVar();

    // @question 在run message之前模态就被改了  此条件感觉永远成立不了
    if(m_compiler_status.mode.gmode[9] == G80_CMD){  //由非循环模态转入循环指令模态
        this->ResetLoopParam();   //先复位参数
        //写入忽略的轴坐标参数
        uint8_t chn_axis_count = this->m_p_channel_control->GetChnAxisCount();
        uint8_t chn_axis_data = 0;
        for(uint8_t i = 0; i < chn_axis_count; i++){
            chn_axis_data = this->MapAxisNameToParamIdx(m_p_channel_control->GetChnAxisName(i));

            //@add zk 记录初始平面
            if(chn_axis_data == 25){
                pv->SetVarValue(199, m_compiler_status.cur_pos.GetAxisValue(i));
            }

            if((pm & (0x01<<chn_axis_data)) == 0){
                pv->SetVarValue(kLoopParamToGlobalVarIndex[chn_axis_data], m_compiler_status.cur_pos.GetAxisValue(i));
            }
        }
    }

    //将当前参数写入全局变量
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
 * @brief 复位循环指令参数对应的宏变量#171~#195
 */
void Compiler::ResetLoopParam(){
    Variable *pv = m_p_channel_control->GetMacroVar();

    for(int i = 0; i < 26; i++){
        pv->ResetVariable(kLoopParamToGlobalVarIndex[i]);
    }
}

#ifdef USES_WOOD_MACHINE
/**
 * @brief 查找是否存在可预启动的主轴指令
 * @param line_min[in] : 下限行号
 * @param line_max[in] : 上限行号
 * @param spd_cmd[out] : 输出参数，用于返回匹配的主轴操作命令
 * @return true--找到匹配的命令    false--无匹配的命令
 */
bool Compiler::FindPreStartSpdCmd(uint64_t line_min , uint64_t line_max, SpindleStartOffset &spd_cmd){
    bool res = false;

    //	printf("Compiler::FindPreStartSpdCmd, count= %d\n", m_p_list_spd_start->GetLength());
    ListNode<SpindleStartOffset> *node = this->m_p_list_spd_start->HeadNode();


    while(node != nullptr){
        //		printf("Compiler::FindPreStartSpdCmd, node=%llu, min=%llu, max=%llu\n", node->data.line_no, line_min, line_max);


        if(node->data.line_no > line_min && node->data.line_no <= line_max){  //在预启动范围，找到匹配项
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
 * @brief 处理五轴无限旋转轴就近路径功能
 * @param tar : 终点位置
 * @param src : 起点位置
 * @param mask : 运动轴的mask
 */
void Compiler::ProcessFiveAxisRotPos(DPoint &tar, DPoint &src, uint32_t mask){
    if (!m_p_channel_control->IsFiveAxisMach() || m_p_channel_control->GetFiveAxisRotMaskNoLimt() == 0)  //非五轴机床或者没有无限旋转轴
        return;

    if(this->m_compiler_status.mode.gmode[8] != G43_4_CMD)
        return;   //非五轴变换状态，则退出

    if(this->m_compiler_status.mode.gmode[3] == G91_CMD)
        return;   //增量式编程不做就近路径

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
                dd_d = floor(dd);   //向下取整
                if(dd-dd_d > 0.5)
                    dd_d += 1;
                *pdes -= 360.0*dd_d;
            }
            if(dd < -0.5){
                dd_d = ceil(dd);   //向上取整
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
 * @brief 处理五轴无限旋转轴就近路径功能
 * @param tar : 终点位置
 * @param src : 起点位置
 * @param mask : 运动轴的mask
 */
void Compiler::ProcessFiveAxisRotPos(DPointChn &tar, DPointChn &src, uint32_t mask){
    if (!m_p_channel_control->IsFiveAxisMach() || m_p_channel_control->GetFiveAxisRotMaskNoLimt() == 0)  //非五轴机床或者没有无限旋转轴
        return;

    if(this->m_compiler_status.mode.gmode[8] != G43_4_CMD)
        return;   //非五轴变换状态，则退出

    if(this->m_compiler_status.mode.gmode[3] == G91_CMD)
        return;   //增量式编程不做就近路径

    uint8_t axis_mask = mask & m_p_channel_control->GetFiveAxisRotMaskNoLimt();
    if(axis_mask == 0)
        return;
    double *psrc = src.m_df_point;
    double *pdes = tar.m_df_point;
    double dd = 0, dd_d = 0;

    uint32_t pmc_mask = m_p_parser->GetPmcAxisMask();//PMC轴的掩码
    mask &= (~pmc_mask);    //去掉PMC轴
    for(uint8_t i = 0; i < this->m_p_channel_config->chn_axis_count; i++){
        if(axis_mask & (0x01<<i)){
            dd = (*pdes - *psrc)/360.;

            if(dd > 0.5){
                dd_d = floor(dd);   //向下取整
                if(dd-dd_d > 0.5)
                    dd_d += 1;
                *pdes -= 360.0*dd_d;
            }
            if(dd < -0.5){
                dd_d = ceil(dd);   //向上取整
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

