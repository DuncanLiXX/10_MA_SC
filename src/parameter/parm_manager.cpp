/*
 * parm_manager.cpp
 *
 *  Created on: 2020��5��6��
 *      Author: M
 */
/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file parm_manager.cpp
 *@author gonghao
 *@date 2020/05/06
 *@brief ��Դ�ļ�Ϊ�����������ʵ��
 *@version
 */

#include "channel_engine.h"
#include "channel_control.h"
#include "parm_manager.h"
#include "global_definition.h"
#include "mi_communication.h"


//template<> int ListNode<ParamUpdate>::new_count = 0;
//template<> int ListNode<CoordUpdate>::new_count = 0;



ParmManager* ParmManager::m_p_instance = nullptr;  //��ʼ����������ָ��Ϊ��

/**
 * @brief  ���캯��
 */
ParmManager::ParmManager(){
	m_sc_system_config = nullptr;
	m_sc_channel_config = nullptr;
	m_sc_axis_config = nullptr;
	m_sc_tool_config = nullptr;
	m_sc_tool_pot_config = nullptr;
	m_sc_coord_config = nullptr;
	m_b_poweroff_param = false;
	m_sc_pc_table = nullptr;

	this->m_p_chn_process_param = nullptr;
	this->m_p_axis_process_param = nullptr;

	this->m_sc_io_remap_list.Clear();


	m_ini_system = new IniFile();		//ϵͳ�����ļ�
	m_ini_chn = new IniFile();			//ͨ�������ļ�
	m_ini_axis = new IniFile();			//�������ļ�
	m_ini_tool = new IniFile();			//���������ļ�  ��������ƫ�ú͵�λ��Ϣ
	m_ini_coord = new IniFile();		//��������ϵ�����ļ�
	m_ini_ex_coord = new IniFile();		//��չ��������ϵ�����ļ�
	m_ini_chn_scene = new IniFile();	//ͨ��״̬�ļ�
	m_ini_pc_table = new IniFile();     //�ݲ������ļ�
	m_ini_io_remap = new IniFile();     //IO�ض����ļ�

	m_ini_proc_chn = new IniFile();     //�������ͨ�������ļ�
	m_ini_proc_axis = new IniFile();    //�������������ļ�
    m_ini_handwheel_map = new IniFile;  //����ͨ��ӳ���ϵ

	this->m_list_new_start = new UpdateParamList();
	this->m_list_reset = new UpdateParamList();
	this->m_list_coord = new UpdateCoordList();
	this->m_list_tool_offset = new UpdateToolOffsetList();

	this->m_list_proc_new_start.Clear();
	this->m_list_proc_reset.Clear();

	for(int i = 0; i < kMaxChnCount; i++)
		m_sc_ex_coord_config[i] = nullptr;

#ifdef USES_FIVE_AXIS_FUNC
	m_five_axis_config = nullptr;
	m_ini_five_axis = new IniFile();   //���������ļ�
#endif

#ifdef USES_GRIND_MACHINE
	this->m_grind_config = nullptr;
	this->m_ini_grind = new IniFile();	//ĥ�������ļ�
	this->m_b_update_grind = false;
#endif
}

/**
 * @brief  ��������
 */
ParmManager::~ParmManager(){
	//�ͷŲ������¶���
	if(m_list_new_start != nullptr){
		delete m_list_new_start;
		m_list_new_start = nullptr;
	}
	if(this->m_list_reset != nullptr){
		delete m_list_reset;
		m_list_reset = nullptr;
	}
	if(this->m_list_coord != nullptr){
		delete m_list_coord;
		m_list_coord = nullptr;
	}
	if(this->m_list_tool_offset != nullptr){
		delete m_list_tool_offset;
		m_list_tool_offset = nullptr;
	}

	//�ͷŹ�������ϵ����
	if(m_sc_coord_config != nullptr){
		delete []m_sc_coord_config;
		m_sc_coord_config = nullptr;
	}

	//�ͷŹ�������ϵ����
	for(int i = 0; i < kMaxChnCount; i++)
		if(m_sc_ex_coord_config[i] != nullptr){
			delete []m_sc_ex_coord_config[i];
			m_sc_ex_coord_config[i] = nullptr;
		}

	//�ͷ�������
	if(m_sc_axis_config != nullptr){
		delete []m_sc_axis_config;
		m_sc_axis_config = nullptr;
	}

	//�ͷŵ�������
	if(m_sc_tool_config != nullptr){
		delete []m_sc_tool_config;
		m_sc_tool_config = nullptr;
	}
	if(m_sc_tool_pot_config != nullptr){
		delete []m_sc_tool_pot_config;
		m_sc_tool_pot_config = nullptr;
	}


	//�ͷ�ͨ������
	if(m_sc_channel_config != nullptr){
		delete []m_sc_channel_config;
		m_sc_channel_config = nullptr;
	}

	//�ͷ�ϵͳ����
	if(m_sc_system_config != nullptr){
		delete m_sc_system_config;
		m_sc_system_config = nullptr;
	}

	//�ͷ��ݲ�����
	if(this->m_sc_pc_table != nullptr){
		for(int i = 0; i < kMaxAxisNum; i++){
			if(m_sc_pc_table->pc_table[i] != nullptr){
				delete []m_sc_pc_table->pc_table[i];
				m_sc_pc_table->pc_table[i] = nullptr;
			}
		}
		delete m_sc_pc_table;
		this->m_sc_pc_table = nullptr;
	}

	if(this->m_p_chn_process_param != nullptr){
		delete []this->m_p_chn_process_param;
		m_p_chn_process_param = nullptr;
	}

	if(this->m_p_axis_process_param != nullptr){
		delete []this->m_p_axis_process_param;
		m_p_axis_process_param = nullptr;
	}


	//�ͷ�INI�ļ�����
	if(m_ini_system != nullptr){
		delete m_ini_system;
		m_ini_system = nullptr;
	}
	if(m_ini_chn != nullptr){
		delete m_ini_chn;
		m_ini_chn = nullptr;
	}
	if(m_ini_axis != nullptr){
		delete m_ini_axis;
		m_ini_axis = nullptr;
	}
	if(m_ini_tool != nullptr){
		delete m_ini_tool;
		m_ini_tool = nullptr;
	}
	if(m_ini_coord != nullptr){
		delete m_ini_coord;
		m_ini_coord = nullptr;
	}
	if(m_ini_ex_coord != nullptr){
		delete m_ini_ex_coord;
		m_ini_ex_coord = nullptr;
	}
	if(m_ini_chn_scene != nullptr){
		delete m_ini_chn_scene;
		m_ini_chn_scene = nullptr;
	}
	if(m_ini_pc_table != nullptr){
		delete m_ini_pc_table;
		m_ini_pc_table = nullptr;
	}
	if(m_ini_io_remap != nullptr){
		delete m_ini_io_remap;
		m_ini_io_remap = nullptr;
	}

	if(m_ini_proc_chn != nullptr){
		delete m_ini_proc_chn;
		m_ini_proc_chn = nullptr;
	}

	if(m_ini_proc_axis != nullptr){
		delete m_ini_proc_axis;
		m_ini_proc_axis = nullptr;
	}

#ifdef USES_FIVE_AXIS_FUNC
	if(m_five_axis_config != nullptr){
		delete []m_five_axis_config;
		m_five_axis_config = nullptr;
	}
	if(m_ini_five_axis != nullptr){
		delete m_ini_five_axis;
		m_ini_five_axis = nullptr;
	}
#endif

#ifdef USES_GRIND_MACHINE
	//�ͷ�ĥ������
	if(m_grind_config != nullptr){
		delete m_grind_config;
		m_grind_config = nullptr;
	}

	if(m_ini_grind != nullptr){
		delete m_ini_grind;
		m_ini_grind = nullptr;
	}
#endif
    if (m_ini_handwheel_map != nullptr) {
        delete m_ini_handwheel_map;
        m_ini_handwheel_map = nullptr;
    }
}

/**
 * @brief ��ȡ��ʵ����Ψһ���ʽӿں���������ģʽ
 */
ParmManager* ParmManager::GetInstance(){
	if(nullptr == m_p_instance){
		m_p_instance = new ParmManager();
	}
	return m_p_instance;
}

/**
 * @brief  ��ʼ��������
 */
void ParmManager::InitParm(){

	//��ʼ��ϵͳ����
	ReadParm(SYS_CONFIG);

	//��ʼ��������
	ReadParm(AXIS_CONFIG);

	//��ʼ��ͨ������
	ReadParm(CHN_CONFIG);

	//��ʼ����������
	ReadParm(TOOL_OFFSET_CONFIG);

	//��ʼ����������ϵ����
	ReadParm(COORD_CONFIG);

	//��ʼ����չ��������ϵ����
	ReadParm(EX_COORD_CONFIG);

	//��ʼ��ͨ��״̬
	ReadParm(CHN_STATE_SCENE);

	//��ʼ���ݲ�����
	ReadParm(PITCH_COMP_DATA);

	//��ʼ�����ղ���
	ReadParm(PROCESS_PARAM);

#ifdef USES_FIVE_AXIS_FUNC
	//��ʼ�������������
	ReadParm(FIVE_AXIS_CONFIG);
#endif

#ifdef USES_GRIND_MACHINE
	//��ʼ��ĥ������
	ReadParm(GRIND_CONFIG);
#endif

	//��ʼ��IO��ӳ������
	this->ReadParm(IO_REMAP_DATA);

    //��ʼ������ͨ��ӳ��
    this->ReadParm(HANDWHEEL_MAP);
}


/**
 * @brief  �������ļ��ж�ȡ����
 * @param type : ��������
 * @return
 */
bool ParmManager::ReadParm(ParmType type){
	bool res = true;
	switch(type){
	case SYS_CONFIG:
		res = ReadSysConfig();
		break;
	case CHN_CONFIG:
		res = ReadChnConfig();
		break;
	case AXIS_CONFIG:
		res = ReadAxisConfig();
		break;
	case TOOL_OFFSET_CONFIG:
		res = ReadToolConfig();
		break;
	case COORD_CONFIG:
		res = ReadCoordConfig();
		break;
	case EX_COORD_CONFIG:
		res = ReadExCoordConfig();
		break;
	case CHN_STATE_SCENE:
		res = this->ReadChnStateScene();
		break;
	case PITCH_COMP_DATA:
		res = this->ReadPcData();
		break;
	case IO_REMAP_DATA:
		res = this->ReadIoRemapConfig();
		break;
	case PROCESS_PARAM:
		res = this->ReadChnProcParam();
		res = this->ReadAxisProcParam();
		break;
#ifdef USES_FIVE_AXIS_FUNC
	case FIVE_AXIS_CONFIG:
		res = this->ReadFiveAxisConfig();
		break;
#endif
#ifdef USES_GRIND_MACHINE
	case GRIND_CONFIG:
		res = this->ReadGrindConfig();
		break;
#endif
    case HANDWHEEL_MAP:
        res = this->ReadHandWheelParam();
        break;
	default:
		res = false;
		break;
	}

	return res;
}

/**
 * @brief ��ȡϵͳ����
 * @return  true--�ɹ�   false--ʧ��
 */
bool ParmManager::ReadSysConfig(){
	int err_code = ERR_NONE;
	if(m_sc_system_config == nullptr){
		m_sc_system_config = new SCSystemConfig();
	}

	if(m_sc_system_config == nullptr || m_ini_system == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}


	if(m_ini_system->Load(SYS_CONFIG_FILE) == 0){//��ȡ���óɹ�
		m_sc_system_config->cnc_mode = 1;
		m_sc_system_config->max_chn_count = kMaxChnCount;
		m_sc_system_config->max_axis_count = kMaxAxisNum;     //12;
        //m_sc_system_config->chn_count = m_ini_system->GetIntValueOrDefault("system", "chn_count", 1);
        m_sc_system_config->chn_count = 1; //ĿǰSCֻ֧�ֵ�ͨ��
		m_sc_system_config->axis_count = m_ini_system->GetIntValueOrDefault("system", "axis_count", 4);
		m_sc_system_config->axis_name_ex = m_ini_system->GetIntValueOrDefault("system", "axis_name_ex", 0);
		m_sc_system_config->bus_cycle = m_ini_system->GetIntValueOrDefault("system", "bus_cycle", 3);
		m_sc_system_config->fix_ratio_find_ref = m_ini_system->GetIntValueOrDefault("system", "fix_ratio_find_ref", 1);
//		m_sc_system_config->pc_type = m_ini_system->GetIntValueOrDefault("system", "pc_type", 1);
		
		m_sc_system_config->hw_code_type = m_ini_system->GetIntValueOrDefault("system", "hw_code_type", 0);    //���ֱ��뷽ʽ     0--�����Ʊ���    1--������

		m_sc_system_config->io_filter_time = m_ini_system->GetIntValueOrDefault("system", "io_filter_time", 1000);	//us
		m_sc_system_config->fast_io_filter_time = m_ini_system->GetIntValueOrDefault("system", "fast_io_filter_time", 500); 	//us
		m_sc_system_config->free_space_limit = m_ini_system->GetIntValueOrDefault("system", "free_space_limit", 10240);	//KB
		m_sc_system_config->backlight_delay_time = m_ini_system->GetIntValueOrDefault("system", "backlight_delay_time", 0);	//��
		m_sc_system_config->save_lineno_poweroff = m_ini_system->GetIntValueOrDefault("system", "save_lineno_poweroff", 0);
        m_sc_system_config->manual_ret_ref_mode = m_ini_system->GetIntValueOrDefault("system", "manual_ret_ref_mode", 0);
		m_sc_system_config->beep_time = m_ini_system->GetIntValueOrDefault("system", "beep_time", 15);		//s
		m_sc_system_config->da_ocp = m_ini_system->GetIntValueOrDefault("system", "da_ocp", 0);
		m_sc_system_config->da_prec = m_ini_system->GetIntValueOrDefault("system", "da_prec", 16);


		m_sc_system_config->alarm_temperature = m_ini_system->GetIntValueOrDefault("system", "alarm_temperature", 70);
		m_sc_system_config->trace_level = m_ini_system->GetIntValueOrDefault("system", "trace_level", 0);
		m_sc_system_config->debug_mode = m_ini_system->GetIntValueOrDefault("system", "debug_mode", 0);

        m_sc_system_config->hw_rev_trace = m_ini_system->GetIntValueOrDefault("system", "hw_rev_trace", 0);

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡϵͳ�����ļ��ɹ���\n");

	}else{
		if(m_ini_system->CreateFile(SYS_CONFIG_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ��ϵͳ�����ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

		//����Ĭ��ֵ
		m_sc_system_config->cnc_mode = 1;
		m_sc_system_config->max_chn_count = kMaxChnCount;
		m_sc_system_config->max_axis_count = kMaxAxisNum;    //12;
		m_sc_system_config->chn_count = 1;
//		m_sc_system_config->mcp_count = 1;
		m_sc_system_config->axis_count = 4;
		m_sc_system_config->axis_name_ex = 0;
		m_sc_system_config->bus_cycle = 3;
		m_sc_system_config->fix_ratio_find_ref = 1;
//		m_sc_system_config->pc_type = 1;

		m_sc_system_config->hw_code_type = 0;    //���ֱ��뷽ʽ     0--�����Ʊ���    1--������

		m_sc_system_config->io_filter_time = 1000;	//us
		m_sc_system_config->fast_io_filter_time = 500; 	//us
		m_sc_system_config->free_space_limit = 10240;	//KB
		m_sc_system_config->backlight_delay_time = 0;	//��
		m_sc_system_config->save_lineno_poweroff = 0;
		m_sc_system_config->manual_ret_ref_mode = 0;
		m_sc_system_config->beep_time = 15;		//s
		m_sc_system_config->da_ocp = 0;
		m_sc_system_config->da_prec = 16;


		m_sc_system_config->alarm_temperature = 70;
		m_sc_system_config->trace_level = 0;
		m_sc_system_config->debug_mode = 0;
		m_sc_system_config->hw_rev_trace = 0;

		//����Ĭ��ini����
		IniSection *ns = m_ini_system->AddSecttion("system");
		if(ns == nullptr){
			err_code = ERR_SC_INIT;
			goto END;
		}
		m_ini_system->AddKeyValuePair(string("chn_count"), string("1"), ns);
		m_ini_system->AddKeyValuePair(string("axis_count"), string("4"), ns);
		m_ini_system->AddKeyValuePair(string("axis_name_ex"), string("0"), ns);
		m_ini_system->AddKeyValuePair(string("bus_cycle"), string("1"), ns);
		m_ini_system->AddKeyValuePair(string("fix_ratio_find_ref"), string("1"), ns);
		m_ini_system->AddKeyValuePair(string("pc_type"), string("1"), ns);

		m_ini_system->AddKeyValuePair(string("hw_code_type"), string("0"), ns);    //���ֱ��뷽ʽ     0--�����Ʊ���    1--������

		m_ini_system->AddKeyValuePair(string("io_filter_time"), string("1000"), ns);
		m_ini_system->AddKeyValuePair(string("fast_io_filter_time"), string("500"), ns);
		m_ini_system->AddKeyValuePair(string("free_space_limit"), string("10240"), ns);
		m_ini_system->AddKeyValuePair(string("backlight_delay_time"), string("0"), ns);
		m_ini_system->AddKeyValuePair(string("save_lineno_poweroff"), string("0"), ns);
		m_ini_system->AddKeyValuePair(string("manual_ret_ref_mode"), string("0"), ns);
		m_ini_system->AddKeyValuePair(string("beep_time"), string("15"), ns);
		m_ini_system->AddKeyValuePair(string("da_ocp"), string("0"), ns);
		m_ini_system->AddKeyValuePair(string("da_prec"), string("16"), ns);
		m_ini_system->AddKeyValuePair(string("alarm_temperature"), string("70"), ns);
		m_ini_system->AddKeyValuePair(string("trace_level"), string("0"), ns);
		m_ini_system->AddKeyValuePair(string("debug_mode"), string("0"), ns);
		m_ini_system->AddKeyValuePair(string("hw_rev_trace"), string("0"), ns);

		m_ini_system->Save();

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ��ϵͳ�����ļ��ɹ���\n");
	}
END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	return true;
}

/**
 * @brief ��ȡͨ������
 * @return  true--�ɹ�   false--ʧ��
 */
bool ParmManager::ReadChnConfig(){
	int err_code = ERR_NONE;
	char sname[20];
	char kname[30];
	char value[10];

	int i = 0, j = 0;
	IniSection *ns = nullptr;

	if(m_sc_channel_config == nullptr){
//		m_sc_channel_config = new SCChannelConfig[m_sc_system_config->chn_count];//����ͨ��������
		m_sc_channel_config = new SCChannelConfig[m_sc_system_config->max_chn_count];//�������ͨ����������Ϊ��֧�ֲ�����һ������
	}

	if(m_sc_channel_config == nullptr || m_ini_chn == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_chn->Load(CHN_CONFIG_FILE) == 0){//��ȡ���óɹ�

	//	for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){	  //֧�ֲ����޸�һ������
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			m_sc_channel_config[i].chn_index = i;
			m_sc_channel_config[i].chn_axis_count = m_ini_chn->GetIntValueOrDefault(sname, "chn_axis_count", 4);
			m_sc_channel_config[i].chn_group_index = m_ini_chn->GetIntValueOrDefault(sname, "chn_group_index", 0);
//			m_sc_channel_config[i].chn_axis_x = m_ini_chn->GetIntValueOrDefault(sname, "chn_axis_x", 0);    //Ĭ��ֵΪ0����δ����
//			m_sc_channel_config[i].chn_axis_y = m_ini_chn->GetIntValueOrDefault(sname, "chn_axis_y", 0);    //Ĭ��ֵΪ0����δ����
//			m_sc_channel_config[i].chn_axis_z = m_ini_chn->GetIntValueOrDefault(sname, "chn_axis_z", 0);    //Ĭ��ֵΪ0����δ����
			for(j = 0; j < kMaxAxisChn; j++){
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_axis_%d", j+1);
				m_sc_channel_config[i].chn_axis_phy[j] = m_ini_chn->GetIntValueOrDefault(sname, kname, 0);    //Ĭ��ֵΪ0����δ����
			}
			for(j = 0; j < kMaxAxisChn; j++){
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_axis_name_%d", j+1);
				m_sc_channel_config[i].chn_axis_name[j] = m_ini_chn->GetIntValueOrDefault(sname, kname, j);
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_axis_name_ex_%d", j+1);
				m_sc_channel_config[i].chn_axis_name_ex[j] = m_ini_chn->GetIntValueOrDefault(sname, kname, 0);
			}
			m_sc_channel_config[i].intep_mode = m_ini_chn->GetIntValueOrDefault(sname, "intep_mode", 0);
			m_sc_channel_config[i].intep_cycle = m_ini_chn->GetIntValueOrDefault(sname, "intep_cycle", 1);

			m_sc_channel_config[i].chn_max_vel = m_ini_chn->GetDoubleValueOrDefault(sname, "chn_max_vel", 40000);	//mm/min
			m_sc_channel_config[i].chn_max_acc = m_ini_chn->GetDoubleValueOrDefault(sname, "chn_max_acc", 2000);	//mm/s^2
			m_sc_channel_config[i].chn_max_dec = m_ini_chn->GetDoubleValueOrDefault(sname, "chn_max_dec", 2000);	//mm/s^2
			m_sc_channel_config[i].chn_max_corner_acc = m_ini_chn->GetDoubleValueOrDefault(sname, "chn_max_corner_acc", 800);	//mm/s^2
			m_sc_channel_config[i].chn_max_arc_acc = m_ini_chn->GetDoubleValueOrDefault(sname, "chn_max_arc_acc", 1000);	//mm/s^2
			m_sc_channel_config[i].chn_s_cut_filter_time = m_ini_chn->GetIntValueOrDefault(sname, "chn_s_cut_filter_time", 5);	//ms

			m_sc_channel_config[i].chn_spd_limit_on_axis = m_ini_chn->GetIntValueOrDefault(sname, "chn_spd_limit_on_axis", 1);
			m_sc_channel_config[i].chn_spd_limit_on_acc = m_ini_chn->GetIntValueOrDefault(sname, "chn_spd_limit_on_acc", 1);
			m_sc_channel_config[i].chn_spd_limit_on_curvity = m_ini_chn->GetIntValueOrDefault(sname, "chn_spd_limit_on_curvity", 1);
			m_sc_channel_config[i].chn_rapid_overlap_level = m_ini_chn->GetIntValueOrDefault(sname, "chn_rapid_overlap_level", 1);
//			printf("on_axis[%hhu]  on_acc[%hhu]  oncur[%hhu]  level[%hhu]\n", m_sc_channel_config[i].chn_spd_limit_on_acc,
//					m_sc_channel_config[i].chn_spd_limit_on_curvity, m_sc_channel_config[i].chn_rapid_overlap_level);


			m_sc_channel_config[i].chn_precision = m_ini_chn->GetIntValueOrDefault(sname, "chn_precision", 10);
			m_sc_channel_config[i].chn_look_ahead = m_ini_chn->GetIntValueOrDefault(sname, "chn_look_ahead", 1);
			m_sc_channel_config[i].chn_feed_limit_level = m_ini_chn->GetIntValueOrDefault(sname, "chn_feed_limit_level", 0);
			m_sc_channel_config[i].zmove_stop = m_ini_chn->GetIntValueOrDefault(sname, "zmove_stop", 0);
			m_sc_channel_config[i].corner_stop_enable = m_ini_chn->GetIntValueOrDefault(sname, "corner_stop_enable", 1);
			m_sc_channel_config[i].corner_stop = m_ini_chn->GetIntValueOrDefault(sname, "corner_stop", 45);
			m_sc_channel_config[i].corner_stop_angle_min = m_ini_chn->GetIntValueOrDefault(sname, "corner_stop_angle_min", 1);
			m_sc_channel_config[i].corner_acc_limit = m_ini_chn->GetIntValueOrDefault(sname, "corner_acc_limit", 1);
			m_sc_channel_config[i].long_line_acc = m_ini_chn->GetIntValueOrDefault(sname, "long_line_acc", 1);

			m_sc_channel_config[i].arc_err_limit = m_ini_chn->GetIntValueOrDefault(sname, "arc_err_limit", 3);
			m_sc_channel_config[i].default_plane = m_ini_chn->GetIntValueOrDefault(sname, "default_plane", 0);
            m_sc_channel_config[i].default_cmd_mode = m_ini_chn->GetIntValueOrDefault(sname, "default_cmd_mode", 0);
            m_sc_channel_config[i].default_feed_mode = m_ini_chn->GetIntValueOrDefault(sname, "default_feed_mode", 1);
            m_sc_channel_config[i].rapid_mode = m_ini_chn->GetIntValueOrDefault(sname, "rapid_mode", 0);

			m_sc_channel_config[i].cut_plan_mode = m_ini_chn->GetIntValueOrDefault(sname, "cut_plan_mode", 1);	//Ĭ��S��
			m_sc_channel_config[i].rapid_plan_mode = m_ini_chn->GetIntValueOrDefault(sname, "rapid_plan_mode", 1);	//Ĭ��S��

			m_sc_channel_config[i].ex_coord_count = m_ini_chn->GetIntValueOrDefault(sname, "ex_coord_count", 0);
			m_sc_channel_config[i].change_tool_mode = m_ini_chn->GetIntValueOrDefault(sname, "change_tool_mode", 1);
			m_sc_channel_config[i].tool_live_check = m_ini_chn->GetIntValueOrDefault(sname, "tool_live_check", 0);
			m_sc_channel_config[i].auto_tool_measure = m_ini_chn->GetIntValueOrDefault(sname, "auto_tool_measure", 0);
			m_sc_channel_config[i].gcode_trace = m_ini_chn->GetIntValueOrDefault(sname, "gcode_trace", 1);
			m_sc_channel_config[i].gcode_unit = m_ini_chn->GetIntValueOrDefault(sname, "gcode_unit", 0);
			m_sc_channel_config[i].timing_mode = m_ini_chn->GetIntValueOrDefault(sname, "timing_mode", 0);

			m_sc_channel_config[i].chn_small_line_time = m_ini_chn->GetIntValueOrDefault(sname, "chn_small_line_time", 30);

			m_sc_channel_config[i].g31_skip_signal = m_ini_chn->GetIntValueOrDefault(sname, "g31_skip_signal", 0);
            m_sc_channel_config[i].g31_sig_level = m_ini_chn->GetIntValueOrDefault(sname, "g31_sig_level", 0);
            m_sc_channel_config[i].rst_hold_time = m_ini_chn->GetIntValueOrDefault(sname, "rst_hold_time", 16);
            m_sc_channel_config[i].rst_mode = m_ini_chn->GetIntValueOrDefault(sname, "rst_mode", 0);
            m_sc_channel_config[i].g00_max_speed = m_ini_chn->GetIntValueOrDefault(sname, "g00_max_speed", 30000);
            m_sc_channel_config[i].g01_max_speed = m_ini_chn->GetIntValueOrDefault(sname, "g01_max_speed", 30000);

#ifdef USES_WOOD_MACHINE
			m_sc_channel_config[i].debug_param_1 = m_ini_chn->GetIntValueOrDefault(sname, "debug_param_1", 0);  //���Բ���1
			m_sc_channel_config[i].debug_param_2 = m_ini_chn->GetIntValueOrDefault(sname, "debug_param_2", 0);  //���Բ���2
			m_sc_channel_config[i].debug_param_3 = m_ini_chn->GetIntValueOrDefault(sname, "debug_param_3", 0);  //���Բ���3
			m_sc_channel_config[i].debug_param_4 = m_ini_chn->GetIntValueOrDefault(sname, "debug_param_4", 0);  //���Բ���4
			m_sc_channel_config[i].debug_param_5 = m_ini_chn->GetIntValueOrDefault(sname, "debug_param_5", 0);  //���Բ���5
			
			m_sc_channel_config[i].flip_comp_value = m_ini_chn->GetIntValueOrDefault(sname, "flip_comp_value", 0);  //���ǲ���ֵ
#endif

		}
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡͨ�������ļ��ɹ���\n");

	}
	else{
		if(m_ini_chn->CreateFile(CHN_CONFIG_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ��ͨ�������ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}


//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){	  //֧�ֲ����޸�һ������
			//����Ĭ��ֵ
			m_sc_channel_config[i].chn_index = i;
			m_sc_channel_config[i].chn_axis_count = 4;
			m_sc_channel_config[i].chn_group_index = 0;  //Ĭ��Ϊ��һ��ʽ��
//			m_sc_channel_config[i].chn_axis_x = 0;    //Ĭ��ֵΪ0����δ����
//			m_sc_channel_config[i].chn_axis_y = 0;    //Ĭ��ֵΪ0����δ����
//			m_sc_channel_config[i].chn_axis_z = 0;    //Ĭ��ֵΪ0����δ����
			for(j = 0; j < kMaxAxisChn; j++){
				m_sc_channel_config[i].chn_axis_phy[j] = 0;    //Ĭ��ֵΪ0����δ����
			}
			for(j = 0; j < kMaxAxisChn; j++){
				m_sc_channel_config[i].chn_axis_name[j] = j;
				m_sc_channel_config[i].chn_axis_name_ex[j] = 0;
			}
			m_sc_channel_config[i].intep_mode = 0;
			m_sc_channel_config[i].intep_cycle = 1;

			m_sc_channel_config[i].chn_max_vel = 40000;	//mm/min
			m_sc_channel_config[i].chn_max_acc = 2000;	//mm/s^2
			m_sc_channel_config[i].chn_max_dec = 2000;	//mm/s^2
			m_sc_channel_config[i].chn_max_corner_acc = 800;	//mm/s^2
			m_sc_channel_config[i].chn_max_arc_acc = 1000;	//mm/s^2
			m_sc_channel_config[i].chn_s_cut_filter_time = 5;	//ms

			m_sc_channel_config[i].chn_spd_limit_on_axis = 1;
			m_sc_channel_config[i].chn_spd_limit_on_acc = 1;
			m_sc_channel_config[i].chn_spd_limit_on_curvity = 1;
			m_sc_channel_config[i].chn_rapid_overlap_level = 1;

			m_sc_channel_config[i].chn_precision = 10;
			m_sc_channel_config[i].chn_look_ahead = 1;
			m_sc_channel_config[i].chn_feed_limit_level = 0;
			m_sc_channel_config[i].zmove_stop = 0;
			m_sc_channel_config[i].corner_stop_enable = 1;
			m_sc_channel_config[i].corner_stop = 45;
			m_sc_channel_config[i].corner_stop_angle_min = 1;
			m_sc_channel_config[i].corner_acc_limit = 1;
			m_sc_channel_config[i].long_line_acc = 1;

			m_sc_channel_config[i].arc_err_limit = 3;
			m_sc_channel_config[i].default_plane = 0;
            m_sc_channel_config[i].default_cmd_mode = 0;
            m_sc_channel_config[i].default_feed_mode = 1;
            m_sc_channel_config[i].rapid_mode = 0;

			m_sc_channel_config[i].cut_plan_mode = 1;	//Ĭ��S��
			m_sc_channel_config[i].rapid_plan_mode = 1;	//Ĭ��S��

			m_sc_channel_config[i].ex_coord_count = 0;
			m_sc_channel_config[i].change_tool_mode = 1;
			m_sc_channel_config[i].tool_live_check = 0;
			m_sc_channel_config[i].auto_tool_measure = 0;
			m_sc_channel_config[i].gcode_trace = 1;
			m_sc_channel_config[i].gcode_unit = 0;
			m_sc_channel_config[i].timing_mode = 0;

			m_sc_channel_config[i].chn_small_line_time = 30;

			m_sc_channel_config[i].g31_skip_signal = 0;
            m_sc_channel_config[i].g31_sig_level = 0;
            m_sc_channel_config[i].rst_hold_time = 16;
            m_sc_channel_config[i].rst_mode = 0;
            m_sc_channel_config[i].g00_max_speed = 30000;
            m_sc_channel_config[i].g01_max_speed = 30000;
#ifdef USES_WOOD_MACHINE
			m_sc_channel_config[i].debug_param_1 = 0;  //���Բ���1
			m_sc_channel_config[i].debug_param_2 = 0;  //���Բ���2
			m_sc_channel_config[i].debug_param_3 = 0;  //���Բ���3
			m_sc_channel_config[i].debug_param_4 = 0;  //���Բ���4
			m_sc_channel_config[i].debug_param_5 = 0;  //���Բ���5
			
			m_sc_channel_config[i].flip_comp_value = 0;   //���ǲ���ֵ
#endif

			//����Ĭ��ini����
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			ns = m_ini_chn->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			m_ini_chn->AddKeyValuePair(string("chn_axis_count"), string("4"), ns);
			m_ini_chn->AddKeyValuePair(string("chn_group_index"), string("0"), ns);    //Ĭ�ϵ�һ��ʽ��   ���ӷ�ʽ�����
//			m_ini_chn->AddKeyValuePair(string("chn_axis_x"), string("0"), ns);
//			m_ini_chn->AddKeyValuePair(string("chn_axis_y"), string("0"), ns);
//			m_ini_chn->AddKeyValuePair(string("chn_axis_z"), string("0"), ns);
			for(j = 0; j < kMaxAxisChn; j++){
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_axis_%d", j+1);
				m_ini_chn->AddKeyValuePair(kname, string("0"), ns);
			}
			for(j = 0; j < kMaxAxisChn; j++){
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_axis_name_%d", j+1);
				memset(value, 0x00, sizeof(value));
				sprintf(value, "%d", j);
				m_ini_chn->AddKeyValuePair(kname, value, ns);
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_axis_name_ex_%d", j+1);
				m_ini_chn->AddKeyValuePair(kname, string("0"), ns);
			}
			m_ini_chn->AddKeyValuePair(string("intep_mode"), string("0"), ns);
			m_ini_chn->AddKeyValuePair(string("intep_cycle"), string("1"), ns);

			m_ini_chn->AddKeyValuePair(string("chn_max_vel"), string("40000"), ns);
			m_ini_chn->AddKeyValuePair(string("chn_max_acc"), string("2000"), ns);
			m_ini_chn->AddKeyValuePair(string("chn_max_dec"), string("2000"), ns);
			m_ini_chn->AddKeyValuePair(string("chn_max_corner_acc"), string("800"), ns);
			m_ini_chn->AddKeyValuePair(string("chn_max_arc_acc"), string("1000"), ns);
			m_ini_chn->AddKeyValuePair(string("chn_s_cut_filter_time"), string("5"), ns);

			m_ini_chn->AddKeyValuePair(string("chn_spd_limit_on_axis"), string("1"), ns);
			m_ini_chn->AddKeyValuePair(string("chn_spd_limit_on_acc"), string("1"), ns);
			m_ini_chn->AddKeyValuePair(string("chn_spd_limit_on_curvity"), string("1"), ns);
			m_ini_chn->AddKeyValuePair(string("chn_rapid_overlap_level"), string("1"), ns);

			m_ini_chn->AddKeyValuePair(string("chn_precision"), string("10"), ns);
			m_ini_chn->AddKeyValuePair(string("chn_look_ahead"), string("1"), ns);
			m_ini_chn->AddKeyValuePair(string("chn_feed_limit_level"), string("0"), ns);
			m_ini_chn->AddKeyValuePair(string("zmove_stop"), string("0"), ns);
			m_ini_chn->AddKeyValuePair(string("corner_stop_enable"), string("1"), ns);
			m_ini_chn->AddKeyValuePair(string("corner_stop"), string("45"), ns);
			m_ini_chn->AddKeyValuePair(string("corner_stop_angle_min"), string("1"), ns);
			m_ini_chn->AddKeyValuePair(string("corner_acc_limit"), string("1"), ns);
			m_ini_chn->AddKeyValuePair(string("long_line_acc"), string("1"), ns);

			m_ini_chn->AddKeyValuePair(string("arc_err_limit"), string("3"), ns);
			m_ini_chn->AddKeyValuePair(string("default_plane"), string("0"), ns);
            m_ini_chn->AddKeyValuePair(string("default_cmd_mode"), string("0"), ns);
            m_ini_chn->AddKeyValuePair(string("default_feed_mode"), string("1"), ns);
            m_ini_chn->AddKeyValuePair(string("rapid_mode"), string("0"), ns);

			m_ini_chn->AddKeyValuePair(string("cut_plan_mode"), string("1"), ns);
			m_ini_chn->AddKeyValuePair(string("rapid_plan_mode"), string("1"), ns);

			m_ini_chn->AddKeyValuePair(string("ex_coord_count"), string("0"), ns);
			m_ini_chn->AddKeyValuePair(string("change_tool_mode"), string("1"), ns);
			m_ini_chn->AddKeyValuePair(string("tool_live_check"), string("0"), ns);
			m_ini_chn->AddKeyValuePair(string("auto_tool_measure"), string("0"), ns);
			m_ini_chn->AddKeyValuePair(string("gcode_trace"), string("1"), ns);
			m_ini_chn->AddKeyValuePair(string("gcode_unit"), string("0"), ns);
			m_ini_chn->AddKeyValuePair(string("timing_mode"), string("0"), ns);

			m_ini_chn->AddKeyValuePair(string("chn_small_line_time"), string("30"), ns);

			m_ini_chn->AddKeyValuePair(string("g31_skip_signal"), string("0"), ns);
            m_ini_chn->AddKeyValuePair(string("g31_sig_level"), string("0"), ns);
            m_ini_chn->AddKeyValuePair(string("rst_hold_time"), string("16"), ns);
            m_ini_chn->AddKeyValuePair(string("rst_mode"), string("0"), ns);
            m_ini_chn->AddKeyValuePair(string("g00_max_speed"), string("30000"), ns);
            m_ini_chn->AddKeyValuePair(string("g01_max_speed"), string("30000"), ns);

#ifdef USES_WOOD_MACHINE
			m_ini_chn->AddKeyValuePair(string("debug_param_1"), string("0"), ns);
			m_ini_chn->AddKeyValuePair(string("debug_param_2"), string("0"), ns);
			m_ini_chn->AddKeyValuePair(string("debug_param_3"), string("0"), ns);
			m_ini_chn->AddKeyValuePair(string("debug_param_4"), string("0"), ns);
			m_ini_chn->AddKeyValuePair(string("debug_param_5"), string("0"), ns);
			
			m_ini_chn->AddKeyValuePair(string("flip_comp_value"), string("0"), ns);
#endif
		}

		m_ini_chn->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ��ͨ�������ļ��ɹ���\n");

	}


	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief ��ȡ�������ͨ������
 * @return true--�ɹ�  false--ʧ��
 */
bool ParmManager::ReadChnProcParam(){
	int err_code = ERR_NONE;
	char sname[20];
	char kname[30];

	int i = 0, j = 0;
	IniSection *ns = nullptr;

	if(this->m_p_chn_process_param == nullptr){
//		m_p_chn_process_param = new ChnProcParamGroup[m_sc_system_config->chn_count];//����ͨ��������
		m_p_chn_process_param = new ChnProcParamGroup[m_sc_system_config->max_chn_count];//�������ͨ����������֧�ֲ����޸�һ������
	}

	if(m_p_chn_process_param == nullptr || m_ini_proc_chn == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_proc_chn->Load(CHN_PROC_PARAM_FILE) == 0){//��ȡ���óɹ�

//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){	  //֧�ֲ����޸�һ������
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);

			for(j = 0; j < kMaxProcParamCount; j++){  //�������й��ղ�����
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_max_acc_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_max_acc = m_ini_proc_chn->GetDoubleValueOrDefault(sname, kname, 2000);	//mm/s^2

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_max_arc_acc_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_max_arc_acc = m_ini_proc_chn->GetDoubleValueOrDefault(sname, kname, 1000);	//mm/s^2

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_max_corner_acc_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_max_corner_acc = m_ini_proc_chn->GetDoubleValueOrDefault(sname, kname, 800);	//mm/s^2

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_max_dec_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_max_dec = m_ini_proc_chn->GetDoubleValueOrDefault(sname, kname, 2000);	//mm/s^2

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_max_vel_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_max_vel = m_ini_proc_chn->GetDoubleValueOrDefault(sname, kname, 40000);	//mm/min

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_rapid_overlap_level_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_rapid_overlap_level = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_s_cut_filter_time_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_s_cut_filter_time = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 5);	//ms

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_small_line_time_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_small_line_time = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 30);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_spd_limit_on_acc_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_spd_limit_on_acc = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);	//Ĭ�ϴ�

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_spd_limit_on_axis_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_spd_limit_on_axis = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);	//Ĭ�ϴ�

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_spd_limit_on_curvity_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_spd_limit_on_curvity = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);  //Ĭ�ϴ�

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_acc_limit_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_acc_limit = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);   //Ĭ�ϴ�

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_stop_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_stop = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 45);	//Ĭ��45��

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_stop_angle_min_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_stop_angle_min = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_stop_enable_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_stop_enable = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "cut_plan_mode_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].cut_plan_mode = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);	//Ĭ��S��

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "rapid_mode_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].rapid_mode = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 0);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "rapid_plan_mode_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].rapid_plan_mode = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);	//Ĭ��S��
				
#ifdef USES_WOOD_MACHINE
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "flip_comp_value_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].flip_comp_value = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 0);	//Ĭ��0
#endif
			}

		}
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡͨ��������ز����ļ��ɹ���\n");

	}else{
		if(m_ini_proc_chn->CreateFile(CHN_PROC_PARAM_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ��ͨ��������ز��������ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}


//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){	  //֧�ֲ����޸�һ������
			//����Ĭ��ini����
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			ns = m_ini_proc_chn->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			for(j = 0; j < kMaxProcParamCount; j++){  //�������й��ղ�����
				//����Ĭ��ֵ
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_max_acc_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_max_acc = 2000;	//mm/s^2
				m_ini_proc_chn->AddKeyValuePair(kname, string("2000"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_max_arc_acc_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_max_arc_acc = 1000;	//mm/s^2
				m_ini_proc_chn->AddKeyValuePair(kname, string("1000"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_max_corner_acc_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_max_corner_acc = 800;	//mm/s^2
				m_ini_proc_chn->AddKeyValuePair(kname, string("800"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_max_dec_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_max_dec = 2000;	//mm/s^2
				m_ini_proc_chn->AddKeyValuePair(kname, string("2000"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_max_vel_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_max_vel = 40000;	//mm/min
				m_ini_proc_chn->AddKeyValuePair(kname, string("40000"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_rapid_overlap_level_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_rapid_overlap_level = 1;
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_s_cut_filter_time_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_s_cut_filter_time = 5;	//ms
				m_ini_proc_chn->AddKeyValuePair(kname, string("5"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_small_line_time_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_small_line_time = 30;
				m_ini_proc_chn->AddKeyValuePair(kname, string("30"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_spd_limit_on_acc_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_spd_limit_on_acc = 1;	//Ĭ�ϴ�
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_spd_limit_on_axis_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_spd_limit_on_axis = 1;	//Ĭ�ϴ�
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_spd_limit_on_curvity_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_spd_limit_on_curvity = 1;  //Ĭ�ϴ�
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_acc_limit_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_acc_limit = 1;   //Ĭ�ϴ�
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_stop_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_stop = 45;	//Ĭ��45��
				m_ini_proc_chn->AddKeyValuePair(kname, string("45"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_stop_angle_min_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_stop_angle_min = 1;
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_stop_enable_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_stop_enable = 1;
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "cut_plan_mode_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].cut_plan_mode = 1;	//Ĭ��S��
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "rapid_mode_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].rapid_mode = 0;
				m_ini_proc_chn->AddKeyValuePair(kname, string("0"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "rapid_plan_mode_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].rapid_plan_mode = 1;	//Ĭ��S��
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

#ifdef USES_WOOD_MACHINE
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "flip_comp_value_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].flip_comp_value = 0;	//Ĭ��0
				m_ini_proc_chn->AddKeyValuePair(kname, string("0"), ns);
#endif
			}

		}

		m_ini_proc_chn->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ��ͨ��������ز����ļ��ɹ���\n");

	}


	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}
/**
 * @brief ��ȡ������������
 * @return   true--�ɹ�   false--ʧ��
 */
bool ParmManager::ReadAxisProcParam(){
	//��ȡ�������ͨ������
	int err_code = ERR_NONE;
	char sname[20];
	char kname[30];

	int i = 0, j = 0;
	IniSection *ns = nullptr;

	if(this->m_p_axis_process_param == nullptr){
//		m_p_axis_process_param = new AxisProcParamGroup[m_sc_system_config->axis_count];//����������������
		m_p_axis_process_param = new AxisProcParamGroup[m_sc_system_config->max_axis_count];//���������������������֧�ֲ����޸ĺ�һ������
	}

	if(m_p_axis_process_param == nullptr || m_ini_proc_axis == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_proc_axis->Load(AXIS_PROC_PARAM_FILE) == 0){//��ȡ���óɹ�

		for(i = 0; i < m_sc_system_config->max_axis_count; i++){  //�������������ᣬ ֧�ֲ����޸ĺ�һ������
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "axis_%d", i+1);

			for(j = 0; j < kMaxProcParamCount; j++){  //�������й��ղ�����
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_acc_limit_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].corner_acc_limit = m_ini_proc_axis->GetDoubleValueOrDefault(sname, kname, 200);	//mm/s^2

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "manual_acc_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].manual_acc = m_ini_proc_axis->GetDoubleValueOrDefault(sname, kname, 2000);	//mm/s^2

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "post_filter_time_1_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].post_filter_time_1 = m_ini_proc_axis->GetIntValueOrDefault(sname, kname, 0);	//ms

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "post_filter_time_2_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].post_filter_time_2 = m_ini_proc_axis->GetIntValueOrDefault(sname, kname, 0);	//ms

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "post_filter_type_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].post_filter_type = m_ini_proc_axis->GetIntValueOrDefault(sname, kname, 0);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "rapid_acc_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].rapid_acc = m_ini_proc_axis->GetDoubleValueOrDefault(sname, kname, 1500);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "rapid_s_plan_filter_time_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].rapid_s_plan_filter_time = m_ini_proc_axis->GetIntValueOrDefault(sname, kname, 5);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "start_acc_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].start_acc = m_ini_proc_axis->GetDoubleValueOrDefault(sname, kname, 500);
			}

		}
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡ�Ṥ����ز����ļ��ɹ���\n");

	}
	else{
		if(m_ini_proc_axis->CreateFile(AXIS_PROC_PARAM_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ���Ṥ����ز��������ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}


		for(i = 0; i < m_sc_system_config->max_axis_count; i++){  //�������������ᣬ ֧�ֲ����޸ĺ�һ������
			//����Ĭ��ini����
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "axis_%d", i+1);
			ns = m_ini_proc_axis->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			for(j = 0; j < kMaxProcParamCount; j++){  //�������й��ղ�����
				//����Ĭ��ֵ
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_acc_limit_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].corner_acc_limit = 200;	//mm/s^2
				m_ini_proc_axis->AddKeyValuePair(kname, string("200"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "manual_acc_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].manual_acc = 2000;	//mm/s^2
				m_ini_proc_axis->AddKeyValuePair(kname, string("2000"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "post_filter_time_1_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].post_filter_time_1 = 0;	//ms
				m_ini_proc_axis->AddKeyValuePair(kname, string("0"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "post_filter_time_2_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].post_filter_time_2 = 0;	//ms
				m_ini_proc_axis->AddKeyValuePair(kname, string("0"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "post_filter_type_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].post_filter_type = 0;
				m_ini_proc_axis->AddKeyValuePair(kname, string("0"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "rapid_acc_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].rapid_acc = 1500;
				m_ini_proc_axis->AddKeyValuePair(kname, string("1500"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "rapid_s_plan_filter_time_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].rapid_s_plan_filter_time = 5;
				m_ini_proc_axis->AddKeyValuePair(kname, string("5"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "start_acc_%d", j+1);
				m_p_axis_process_param[i].axis_param[j].start_acc = 500;
				m_ini_proc_axis->AddKeyValuePair(kname, string("500"), ns);
			}

		}

		m_ini_proc_axis->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ���Ṥ����ز����ļ��ɹ���\n");

	}


	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

    return true;
}

/**
 * @brief ��ȡ����ͨ��ӳ�����
 * @return  true--�ɹ�   false--ʧ��
 */
bool ParmManager::ReadHandWheelParam()
{
    if (m_ini_handwheel_map == nullptr){
        return false;
    }
    if(m_ini_handwheel_map->Load(HANDWHEEL_CONFIG_FILE) == 0){//��ȡ���óɹ�
        vector<string> sections;
        m_ini_handwheel_map->GetSections(&sections);
        for(auto itr = sections.begin() + 1; itr != sections.end(); ++itr)
        {
            HandWheelMapInfo info;
            info.devNum = m_ini_handwheel_map->GetIntValueOrDefault(*itr, "devNum", -1);
            info.wheelID = m_ini_handwheel_map->GetIntValueOrDefault(*itr, "wheelId", -1);
            info.channelMap = m_ini_handwheel_map->GetIntValueOrDefault(*itr, "channelMap", -1);
            std::string strValue;
            m_ini_handwheel_map->GetStringValueOrDefault(*itr, "devName", &strValue, "null");

            size_t len = strValue.length();
            if(len >= sizeof(info.devName))
                len = sizeof(info.devName)-1;
            strValue.copy(info.devName, len, 0);
            //info.devName = strValue;

            m_p_handwheel_param.push_back(info);
        }
    }
    else
    {
        if(m_ini_handwheel_map->CreateFile(HANDWHEEL_CONFIG_FILE)){
            return false;
        }

    }
    return true;
}

/**
 * @brief ��ȡ������
 * @return  true--�ɹ�   false--ʧ��
 */
bool ParmManager::ReadAxisConfig(){
	int err_code = ERR_NONE;
	char sname[32];
	char kname[64];
	char value[16];

	int i = 0, j = 0;

	IniSection *ns = nullptr;

	if(m_sc_axis_config == nullptr){
//		m_sc_axis_config = new SCAxisConfig[m_sc_system_config->axis_count];//����ͨ��������
		m_sc_axis_config = new SCAxisConfig[m_sc_system_config->max_axis_count];//����ͨ����������֧�ֲ����޸ĺ�һ������
	}

	if(m_sc_axis_config == nullptr || m_ini_axis == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_axis->Load(AXIS_CONFIG_FILE) == 0){//��ȡ���óɹ�
		for(i = 0; i < m_sc_system_config->max_axis_count; i++){
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "axis_%d", i+1);   //��1��ʼ
			m_sc_axis_config[i].axis_index = i;
			m_sc_axis_config[i].axis_type = m_ini_axis->GetIntValueOrDefault(sname, "axis_type", 0);
			m_sc_axis_config[i].axis_interface = m_ini_axis->GetIntValueOrDefault(sname, "axis_interface", 0);
			m_sc_axis_config[i].axis_port = m_ini_axis->GetIntValueOrDefault(sname, "axis_port", 0);
			m_sc_axis_config[i].axis_linear_type = m_ini_axis->GetIntValueOrDefault(sname, "axis_linear_type", 0);
			m_sc_axis_config[i].axis_pmc = m_ini_axis->GetIntValueOrDefault(sname, "axis_pmc", 0);
			m_sc_axis_config[i].kp1 = m_ini_axis->GetDoubleValueOrDefault(sname, "kp1", 50.0);
			m_sc_axis_config[i].kp2 = m_ini_axis->GetDoubleValueOrDefault(sname, "kp2", 50.0);
			m_sc_axis_config[i].ki = m_ini_axis->GetDoubleValueOrDefault(sname, "ki", 0.3);
			m_sc_axis_config[i].kil = m_ini_axis->GetDoubleValueOrDefault(sname, "kil", 50000.0);
			m_sc_axis_config[i].kd = m_ini_axis->GetDoubleValueOrDefault(sname, "kd", 0.0);
			m_sc_axis_config[i].kvff = m_ini_axis->GetIntValueOrDefault(sname, "kvff", 0);
			m_sc_axis_config[i].kaff = m_ini_axis->GetIntValueOrDefault(sname, "kaff", 0);
			m_sc_axis_config[i].track_err_limit = m_ini_axis->GetIntValueOrDefault(sname, "track_err_limit", 30000);
			m_sc_axis_config[i].location_err_limit = m_ini_axis->GetIntValueOrDefault(sname, "location_err_limit", 5);
//			m_sc_axis_config[i].soft_limit_check = m_ini_axis->GetIntValueOrDefault(sname, "soft_limit_check", 1);
			m_sc_axis_config[i].motor_count_pr = m_ini_axis->GetIntValueOrDefault(sname, "motor_count_pr", 10000);
			m_sc_axis_config[i].motor_speed_max = m_ini_axis->GetIntValueOrDefault(sname, "motor_speed_max", 3000);
			m_sc_axis_config[i].move_pr = m_ini_axis->GetDoubleValueOrDefault(sname, "move_pr", 10.0);

			m_sc_axis_config[i].motor_dir = m_ini_axis->GetIntValueOrDefault(sname, "motor_dir", 1);
			m_sc_axis_config[i].feedback_mode = m_ini_axis->GetIntValueOrDefault(sname, "feedback_mode", 0);
            //m_sc_axis_config[i].ret_ref_mode = m_ini_axis->GetIntValueOrDefault(sname, "ret_ref_mode", 0);
            m_sc_axis_config[i].ret_ref_mode = 1;
            m_sc_axis_config[i].absolute_ref_mode = m_ini_axis->GetIntValueOrDefault(sname, "absolute_ref_mode", 0);
			m_sc_axis_config[i].ret_ref_dir = m_ini_axis->GetIntValueOrDefault(sname, "ret_ref_dir", 0);
            //m_sc_axis_config[i].ret_ref_change_dir = m_ini_axis->GetIntValueOrDefault(sname, "ret_ref_change_dir", 0);
            m_sc_axis_config[i].ret_ref_change_dir = 0;
			m_sc_axis_config[i].ref_mark_err = m_ini_axis->GetDoubleValueOrDefault(sname, "ref_mark_err", 0);

			m_sc_axis_config[i].ref_signal = m_ini_axis->GetIntValueOrDefault(sname, "ref_signal", 0);
			m_sc_axis_config[i].ret_ref_index = m_ini_axis->GetIntValueOrDefault(sname, "ret_ref_index", 0);
			m_sc_axis_config[i].ref_base_diff_check = m_ini_axis->GetIntValueOrDefault(sname, "ref_base_diff_check", 0);
			m_sc_axis_config[i].ref_base_diff = m_ini_axis->GetDoubleValueOrDefault(sname, "ref_base_diff", 0);
			m_sc_axis_config[i].ref_encoder = m_ini_axis->GetInt64ValueOrDefault(sname, "ref_encoder", kAxisRefNoDef);
			m_sc_axis_config[i].manual_speed = m_ini_axis->GetDoubleValueOrDefault(sname, "manual_speed", 3000);
			m_sc_axis_config[i].rapid_speed = m_ini_axis->GetDoubleValueOrDefault(sname, "rapid_speed", 8000);
			m_sc_axis_config[i].reset_speed = m_ini_axis->GetDoubleValueOrDefault(sname, "reset_speed", 2000);
			m_sc_axis_config[i].ret_ref_speed = m_ini_axis->GetDoubleValueOrDefault(sname, "ret_ref_speed", 500);
            m_sc_axis_config[i].ret_ref_speed_second = m_ini_axis->GetDoubleValueOrDefault(sname, "ret_ref_speed_second", 100);
            m_sc_axis_config[i].ref_offset_pos = m_ini_axis->GetDoubleValueOrDefault(sname, "ref_offset_pos", 0);
            m_sc_axis_config[i].ref_z_distance_max = m_ini_axis->GetDoubleValueOrDefault(sname, "ref_z_distance_max", 0);

			m_sc_axis_config[i].rapid_acc = m_ini_axis->GetDoubleValueOrDefault(sname, "rapid_acc", 1500);
			m_sc_axis_config[i].manual_acc = m_ini_axis->GetDoubleValueOrDefault(sname, "manual_acc", 2000);
			m_sc_axis_config[i].start_acc = m_ini_axis->GetDoubleValueOrDefault(sname, "start_acc", 500);
			m_sc_axis_config[i].corner_acc_limit = m_ini_axis->GetDoubleValueOrDefault(sname, "corner_acc_limit", 200);
			m_sc_axis_config[i].rapid_s_plan_filter_time = m_ini_axis->GetDoubleValueOrDefault(sname, "rapid_s_plan_filter_time", 5);

			m_sc_axis_config[i].post_filter_type = m_ini_axis->GetIntValueOrDefault(sname, "post_filter_type", 0);
			m_sc_axis_config[i].post_filter_time_1 = m_ini_axis->GetIntValueOrDefault(sname, "post_filter_time_1", 0);
			m_sc_axis_config[i].post_filter_time_2 = m_ini_axis->GetIntValueOrDefault(sname, "post_filter_time_2", 0);
//			printf("axis %hhu, type = %hhu, time1=%hhu, time2=%hhu\n", i, m_sc_axis_config[i].post_filter_type,
//					m_sc_axis_config[i].post_filter_time_1, m_sc_axis_config[i].post_filter_time_2);

			m_sc_axis_config[i].backlash_forward = m_ini_axis->GetIntValueOrDefault(sname, "backlash_forward", 0);
			m_sc_axis_config[i].backlash_negative = m_ini_axis->GetIntValueOrDefault(sname, "backlash_negative", 0);
			m_sc_axis_config[i].backlash_enable = m_ini_axis->GetIntValueOrDefault(sname, "backlash_enable", 1);
			m_sc_axis_config[i].pc_offset = m_ini_axis->GetIntValueOrDefault(sname, "pc_offset", 1+400*i);
//			printf("init axis %hhu pc_offset = %hu\n", i, m_sc_axis_config[i].pc_offset);
			m_sc_axis_config[i].pc_type = m_ini_axis->GetIntValueOrDefault(sname, "pc_type", 0);
			m_sc_axis_config[i].pc_enable = m_ini_axis->GetIntValueOrDefault(sname, "pc_enable", 1);
			m_sc_axis_config[i].pc_count = m_ini_axis->GetIntValueOrDefault(sname, "pc_count", 0);
			m_sc_axis_config[i].pc_ref_index = m_ini_axis->GetIntValueOrDefault(sname, "pc_ref_index", 1);
			m_sc_axis_config[i].pc_inter_dist = m_ini_axis->GetDoubleValueOrDefault(sname, "pc_inter_dist", 1.0);

			m_sc_axis_config[i].soft_limit_check_1 = m_ini_axis->GetIntValueOrDefault(sname, "soft_limit_check_1", 0);
			m_sc_axis_config[i].soft_limit_max_1 = m_ini_axis->GetDoubleValueOrDefault(sname, "soft_limit_max_1", 100.0);
			m_sc_axis_config[i].soft_limit_min_1 = m_ini_axis->GetDoubleValueOrDefault(sname, "soft_limit_min_1", -100.0);
			m_sc_axis_config[i].soft_limit_check_2 = m_ini_axis->GetIntValueOrDefault(sname, "soft_limit_check_2", 0);
			m_sc_axis_config[i].soft_limit_max_2 = m_ini_axis->GetDoubleValueOrDefault(sname, "soft_limit_max_2", 100.0);
			m_sc_axis_config[i].soft_limit_min_2 = m_ini_axis->GetDoubleValueOrDefault(sname, "soft_limit_min_2", -100.0);
			m_sc_axis_config[i].soft_limit_check_3 = m_ini_axis->GetIntValueOrDefault(sname, "soft_limit_check_3", 0);
			m_sc_axis_config[i].soft_limit_max_3 = m_ini_axis->GetDoubleValueOrDefault(sname, "soft_limit_max_3", 100.0);
			m_sc_axis_config[i].soft_limit_min_3 = m_ini_axis->GetDoubleValueOrDefault(sname, "soft_limit_min_3", -100.0);


			m_sc_axis_config[i].save_pos_poweroff = m_ini_axis->GetIntValueOrDefault(sname, "save_pos_poweroff", 0);
//			m_sc_axis_config[i].axis_alarm_level = m_ini_axis->GetIntValueOrDefault(sname, "axis_alarm_level", 0);
			m_sc_axis_config[i].off_line_check = m_ini_axis->GetIntValueOrDefault(sname, "off_line_check", 0);
			m_sc_axis_config[i].ctrl_mode = m_ini_axis->GetIntValueOrDefault(sname, "ctrl_mode", 1);
			m_sc_axis_config[i].pulse_count_pr = m_ini_axis->GetIntValueOrDefault(sname, "pulse_count_pr", 2048);

			m_sc_axis_config[i].encoder_lines = m_ini_axis->GetIntValueOrDefault(sname, "encoder_lines", 17);
			m_sc_axis_config[i].zero_compensation = m_ini_axis->GetIntValueOrDefault(sname, "zero_compensation", 0);
			m_sc_axis_config[i].spd_gear_ratio = m_ini_axis->GetIntValueOrDefault(sname, "spd_gear_ratio", 1);
			m_sc_axis_config[i].spd_max_speed = m_ini_axis->GetIntValueOrDefault(sname, "spd_max_speed", 10000);
			m_sc_axis_config[i].spd_min_speed = m_ini_axis->GetIntValueOrDefault(sname, "spd_min_speed", 100);

			m_sc_axis_config[i].spd_start_time = m_ini_axis->GetIntValueOrDefault(sname, "spd_start_time", 10);
			m_sc_axis_config[i].spd_stop_time = m_ini_axis->GetIntValueOrDefault(sname, "spd_stop_time", 5);
			m_sc_axis_config[i].spd_vctrl_mode = m_ini_axis->GetIntValueOrDefault(sname, "spd_vctrl_mode", 2);
//			m_sc_axis_config[i].spd_set_dir = m_ini_axis->GetIntValueOrDefault(sname, "spd_set_dir", 0);

			m_sc_axis_config[i].spd_set_speed = m_ini_axis->GetIntValueOrDefault(sname, "spd_set_speed", 3000);
			m_sc_axis_config[i].spd_speed_check = m_ini_axis->GetIntValueOrDefault(sname, "spd_speed_check", 1);
			m_sc_axis_config[i].spd_speed_match = m_ini_axis->GetIntValueOrDefault(sname, "spd_speed_match", 1);
			m_sc_axis_config[i].spd_speed_diff_limit = m_ini_axis->GetIntValueOrDefault(sname, "spd_speed_diff_limit", 10);
			m_sc_axis_config[i].spd_encoder_location = m_ini_axis->GetIntValueOrDefault(sname, "spd_encoder_location", 0);

            m_sc_axis_config[i].spd_ctrl_GST = m_ini_axis->GetIntValueOrDefault(sname, "spd_ctrl_GST", 0);
            m_sc_axis_config[i].spd_ctrl_SGB = m_ini_axis->GetIntValueOrDefault(sname, "spd_ctrl_SGB", 0);
            m_sc_axis_config[i].spd_ctrl_SFA = m_ini_axis->GetIntValueOrDefault(sname, "spd_ctrl_SFA", 1);
            m_sc_axis_config[i].spd_ctrl_ORM = m_ini_axis->GetIntValueOrDefault(sname, "spd_ctrl_ORM", 0);
            m_sc_axis_config[i].spd_ctrl_TCW = m_ini_axis->GetIntValueOrDefault(sname, "spd_ctrl_TCW", 1);
            m_sc_axis_config[i].spd_ctrl_CWM = m_ini_axis->GetIntValueOrDefault(sname, "spd_ctrl_CWM", 0);
            m_sc_axis_config[i].spd_ctrl_TSO = m_ini_axis->GetIntValueOrDefault(sname, "spd_ctrl_TSO", 0);

            m_sc_axis_config[i].spd_analog_gain = m_ini_axis->GetIntValueOrDefault(sname, "spd_analog_gain", 1000);
            m_sc_axis_config[i].spd_sor_speed = m_ini_axis->GetIntValueOrDefault(sname, "spd_sor_speed", 20);
            m_sc_axis_config[i].spd_motor_min_speed = m_ini_axis->GetIntValueOrDefault(sname, "spd_motor_min_speed", 0);
            m_sc_axis_config[i].spd_motor_max_speed = m_ini_axis->GetIntValueOrDefault(sname, "spd_motor_max_speed", 32767);
            m_sc_axis_config[i].spd_gear_speed_low = m_ini_axis->GetIntValueOrDefault(sname, "spd_gear_speed_low", 1500);
            m_sc_axis_config[i].spd_gear_speed_middle = m_ini_axis->GetIntValueOrDefault(sname, "spd_gear_speed_middle", 5000);
            m_sc_axis_config[i].spd_gear_speed_high = m_ini_axis->GetIntValueOrDefault(sname, "spd_gear_speed_high", 10000);
            m_sc_axis_config[i].spd_gear_switch_speed1 = m_ini_axis->GetIntValueOrDefault(sname, "spd_gear_switch_speed1", 1500);
            m_sc_axis_config[i].spd_gear_switch_speed2 = m_ini_axis->GetIntValueOrDefault(sname, "spd_gear_switch_speed2", 5000);
            m_sc_axis_config[i].spd_sync_error_gain = m_ini_axis->GetIntValueOrDefault(sname, "spd_sync_error_gain", 200);
            m_sc_axis_config[i].spd_speed_feed_gain = m_ini_axis->GetIntValueOrDefault(sname, "spd_speed_feed_gain", 60000);
            m_sc_axis_config[i].spd_pos_ratio_gain = m_ini_axis->GetIntValueOrDefault(sname, "spd_pos_ratio_gain", 100000);

			m_sc_axis_config[i].fast_locate = m_ini_axis->GetIntValueOrDefault(sname, "fast_locat", 1);
			m_sc_axis_config[i].pos_disp_mode = m_ini_axis->GetIntValueOrDefault(sname, "pos_disp_mode", 0);
			m_sc_axis_config[i].encoder_max_cycle = m_ini_axis->GetIntValueOrDefault(sname, "encoder_max_cycle", 0);  //���Ȧ��Ĭ��ֵ��256��Ϊ0

			m_sc_axis_config[i].sync_axis = m_ini_axis->GetIntValueOrDefault(sname, "sync_axis", 0);
			m_sc_axis_config[i].master_axis_no = m_ini_axis->GetIntValueOrDefault(sname, "master_axis_no", 0);
			m_sc_axis_config[i].disp_coord = m_ini_axis->GetIntValueOrDefault(sname, "disp_coord", 0);
			m_sc_axis_config[i].benchmark_offset = m_ini_axis->GetDoubleValueOrDefault(sname, "benchmark_offset", 0);
			m_sc_axis_config[i].sync_err_max = m_ini_axis->GetIntValueOrDefault(sname, "sync_err_max", 0);
			m_sc_axis_config[i].auto_sync = m_ini_axis->GetIntValueOrDefault(sname, "auto_sync", 0);


			for(j = 0; j < 10; j++){
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "axis_home_pos_%d", j+1);
				m_sc_axis_config[i].axis_home_pos[j] = m_ini_axis->GetDoubleValueOrDefault(sname, kname, 0.0);
			}

		}
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡ�������ļ��ɹ���\n");
	}
	else {
		if(m_ini_axis->CreateFile(AXIS_CONFIG_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ���������ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

		for(i = 0; i < m_sc_system_config->max_axis_count; i++){
			//����Ĭ��ֵ
			m_sc_axis_config[i].axis_index = i;
			m_sc_axis_config[i].axis_type = 0;	//Ĭ��ֱ����
			m_sc_axis_config[i].axis_interface = 0;	//Ĭ��������
			m_sc_axis_config[i].axis_port = 0;
			m_sc_axis_config[i].axis_linear_type = 0;
			m_sc_axis_config[i].axis_pmc = 0;
			m_sc_axis_config[i].kp1 = 50.0;
			m_sc_axis_config[i].kp2 = 50.0;
			m_sc_axis_config[i].ki = 0.3;
			m_sc_axis_config[i].kil = 50000.0;
			m_sc_axis_config[i].kd = 0.0;
			m_sc_axis_config[i].kvff = 0;
			m_sc_axis_config[i].kaff = 0;
			m_sc_axis_config[i].track_err_limit = 300; //30000;
			m_sc_axis_config[i].location_err_limit = 100; //5;
//			m_sc_axis_config[i].soft_limit_check = 1;
			m_sc_axis_config[i].motor_count_pr = 10000;
			m_sc_axis_config[i].motor_speed_max = 3000;
			m_sc_axis_config[i].move_pr = 10.0; //5.0;

			m_sc_axis_config[i].motor_dir = 1;
			m_sc_axis_config[i].feedback_mode = 0;
            m_sc_axis_config[i].ret_ref_mode = 1;
			m_sc_axis_config[i].ret_ref_dir = 0;
            m_sc_axis_config[i].absolute_ref_mode = 0;
			m_sc_axis_config[i].ret_ref_change_dir = 0;
			m_sc_axis_config[i].ref_mark_err = 0;

			m_sc_axis_config[i].ref_signal = 0;
			m_sc_axis_config[i].ret_ref_index = 0;
			m_sc_axis_config[i].ref_base_diff_check = 0;
			m_sc_axis_config[i].ref_base_diff = 0;
			m_sc_axis_config[i].ref_encoder = kAxisRefNoDef;
			m_sc_axis_config[i].manual_speed = 3000;
			m_sc_axis_config[i].rapid_speed = 8000;
			m_sc_axis_config[i].reset_speed = 2000;
			m_sc_axis_config[i].ret_ref_speed = 500;
            m_sc_axis_config[i].ret_ref_speed_second = 100;
            m_sc_axis_config[i].ref_offset_pos = 0;
            m_sc_axis_config[i].ref_z_distance_max = 0;

			m_sc_axis_config[i].rapid_acc = 1500;
			m_sc_axis_config[i].manual_acc = 2000;
			m_sc_axis_config[i].start_acc = 500;
			m_sc_axis_config[i].corner_acc_limit = 200;
			m_sc_axis_config[i].rapid_s_plan_filter_time = 5;

			m_sc_axis_config[i].post_filter_type = 0;
			m_sc_axis_config[i].post_filter_time_1 = 0;
			m_sc_axis_config[i].post_filter_time_2 = 0;

			m_sc_axis_config[i].backlash_forward = 0;
			m_sc_axis_config[i].backlash_negative = 0;
			m_sc_axis_config[i].pc_offset = 1+400*i;
			m_sc_axis_config[i].pc_count = 0;
			m_sc_axis_config[i].pc_ref_index = 1;
			m_sc_axis_config[i].pc_inter_dist = 1.0;

			m_sc_axis_config[i].soft_limit_check_1 = 0;
			m_sc_axis_config[i].soft_limit_max_1 = 100.0;
			m_sc_axis_config[i].soft_limit_min_1 = -100.0;
			m_sc_axis_config[i].soft_limit_check_2 = 0;
			m_sc_axis_config[i].soft_limit_max_2 = 100.0;
			m_sc_axis_config[i].soft_limit_min_2 = -100.0;
			m_sc_axis_config[i].soft_limit_check_3 = 0;
			m_sc_axis_config[i].soft_limit_max_3 = 100.0;
			m_sc_axis_config[i].soft_limit_min_3 = -100.0;


			m_sc_axis_config[i].save_pos_poweroff = 0;
//			m_sc_axis_config[i].axis_alarm_level = 0;
			m_sc_axis_config[i].off_line_check = 0;
			m_sc_axis_config[i].ctrl_mode = 1;
			m_sc_axis_config[i].pulse_count_pr = 10000; //2048;

			m_sc_axis_config[i].encoder_lines = 17;
			m_sc_axis_config[i].zero_compensation = 0;
			m_sc_axis_config[i].spd_gear_ratio = 1;
			m_sc_axis_config[i].spd_max_speed = 10000;
			m_sc_axis_config[i].spd_min_speed = 100;

			m_sc_axis_config[i].spd_start_time = 10;
			m_sc_axis_config[i].spd_stop_time = 5;
			m_sc_axis_config[i].spd_vctrl_mode = 2;
//			m_sc_axis_config[i].spd_set_dir = 0;

			m_sc_axis_config[i].spd_set_speed = 3000;
			m_sc_axis_config[i].spd_speed_check = 1;
			m_sc_axis_config[i].spd_speed_match = 1;
			m_sc_axis_config[i].spd_speed_diff_limit = 10;
			m_sc_axis_config[i].spd_encoder_location = 0;

            m_sc_axis_config[i].spd_ctrl_GST = 0;
            m_sc_axis_config[i].spd_ctrl_SGB = 0;
            m_sc_axis_config[i].spd_ctrl_SFA = 1;
            m_sc_axis_config[i].spd_ctrl_ORM = 0;
            m_sc_axis_config[i].spd_ctrl_TCW = 1;
            m_sc_axis_config[i].spd_ctrl_CWM = 0;
            m_sc_axis_config[i].spd_ctrl_TSO = 0;

            m_sc_axis_config[i].spd_analog_gain = 1000;
            m_sc_axis_config[i].spd_sor_speed = 20;
            m_sc_axis_config[i].spd_motor_min_speed = 0;
            m_sc_axis_config[i].spd_motor_max_speed = 32767;
            m_sc_axis_config[i].spd_gear_speed_low = 1500;
            m_sc_axis_config[i].spd_gear_speed_middle = 5000;
            m_sc_axis_config[i].spd_gear_speed_high = 10000;
            m_sc_axis_config[i].spd_gear_switch_speed1 = 1500;
            m_sc_axis_config[i].spd_gear_switch_speed2 = 5000;
            m_sc_axis_config[i].spd_sync_error_gain = 200;
            m_sc_axis_config[i].spd_speed_feed_gain = 60000;
            m_sc_axis_config[i].spd_pos_ratio_gain = 100000;

			m_sc_axis_config[i].fast_locate = 1;
			m_sc_axis_config[i].pos_disp_mode = 0;
			m_sc_axis_config[i].encoder_max_cycle = 0;   //���Ȧ��Ĭ��ֵ��256��Ϊ0

			m_sc_axis_config[i].sync_axis = 0;
			m_sc_axis_config[i].master_axis_no = 0;
			m_sc_axis_config[i].disp_coord = 0;
			m_sc_axis_config[i].benchmark_offset = 0;
			m_sc_axis_config[i].sync_err_max = 0;
			m_sc_axis_config[i].auto_sync = 0;

			for(j = 0; j < 10; j++){
				m_sc_axis_config[i].axis_home_pos[j] = 0.0;
			}

			//����Ĭ��ini����
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "axis_%d", i+1);    //��1��ʼ
			ns = m_ini_axis->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}
			m_ini_axis->AddKeyValuePair(string("axis_type"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("axis_interface"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("axis_port"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("axis_linear_type"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("axis_pmc"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("kp1"), string("50.0"), ns);
			m_ini_axis->AddKeyValuePair(string("kp2"), string("50.0"), ns);
			m_ini_axis->AddKeyValuePair(string("ki"), string("0.3"), ns);
			m_ini_axis->AddKeyValuePair(string("kil"), string("50000.0"), ns);
			m_ini_axis->AddKeyValuePair(string("kd"), string("0.0"), ns);
			m_ini_axis->AddKeyValuePair(string("kvff"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("kaff"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("track_err_limit"), string("300"), ns);
			m_ini_axis->AddKeyValuePair(string("location_err_limit"), string("100"), ns);
//			m_ini_axis->AddKeyValuePair(string("soft_limit_check"), string("1"), ns);
			m_ini_axis->AddKeyValuePair(string("motor_count_pr"), string("10000"), ns);
			m_ini_axis->AddKeyValuePair(string("motor_max_rpm"), string("3000"), ns);

			m_ini_axis->AddKeyValuePair(string("move_pr"), string("10.0"), ns);
			m_ini_axis->AddKeyValuePair(string("motor_dir"), string("1"), ns);
			m_ini_axis->AddKeyValuePair(string("feedback_mode"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("ret_ref_mode"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("ret_ref_dir"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("ret_ref_change_dir"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("ref_mark_err"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("ref_signal"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("ret_ref_index"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("ref_base_diff_check"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("ref_base_diff"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("manual_speed"), string("3000"), ns);
			m_ini_axis->AddKeyValuePair(string("rapid_speed"), string("8000"), ns);
			m_ini_axis->AddKeyValuePair(string("reset_speed"), string("2000"), ns);
			m_ini_axis->AddKeyValuePair(string("ret_ref_speed"), string("500"), ns);

            m_ini_axis->AddKeyValuePair(string("ret_ref_speed_second"), string("100"), ns);
            m_ini_axis->AddKeyValuePair(string("ref_offset_pos"), string("0"), ns);
            m_ini_axis->AddKeyValuePair(string("ref_z_distance_max"), string("0"), ns);

			m_ini_axis->AddKeyValuePair(string("rapid_acc"), string("1500"), ns);
			m_ini_axis->AddKeyValuePair(string("manual_acc"), string("2000"), ns);
			m_ini_axis->AddKeyValuePair(string("start_acc"), string("500"), ns);
			m_ini_axis->AddKeyValuePair(string("corner_acc_limit"), string("200"), ns);
			m_ini_axis->AddKeyValuePair(string("rapid_s_plan_filter_time"), string("5"), ns);

			m_ini_axis->AddKeyValuePair(string("post_filter_type"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("post_filter_time_1"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("post_filter_time_2"), string("0"), ns);

			m_ini_axis->AddKeyValuePair(string("backlash_forward"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("backlash_negative"), string("0"), ns);

			memset(value, 0x00, sizeof(value));
			sprintf(value, "%hu", m_sc_axis_config[i].pc_offset);
			m_ini_axis->AddKeyValuePair(string("pc_offset"), value, ns);
			m_ini_axis->AddKeyValuePair(string("pc_count"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("pc_ref_index"), string("1"), ns);
			m_ini_axis->AddKeyValuePair(string("pc_inter_dist"), string("1.0"), ns);

			m_ini_axis->AddKeyValuePair(string("soft_limit_check_1"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("soft_limit_max_1"), string("100.0"), ns);
			m_ini_axis->AddKeyValuePair(string("soft_limit_min_1"), string("-100.0"), ns);
			m_ini_axis->AddKeyValuePair(string("soft_limit_check_2"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("soft_limit_max_2"), string("100.0"), ns);
			m_ini_axis->AddKeyValuePair(string("soft_limit_min_2"), string("-100.0"), ns);
			m_ini_axis->AddKeyValuePair(string("soft_limit_check_3"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("soft_limit_max_3"), string("100.0"), ns);
			m_ini_axis->AddKeyValuePair(string("soft_limit_min_3"), string("-100.0"), ns);

			m_ini_axis->AddKeyValuePair(string("save_pos_poweroff"), string("0"), ns);
//			m_ini_axis->AddKeyValuePair("axis_alarm_level", string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("off_line_check"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("ctrl_mode"), string("1"), ns);
			m_ini_axis->AddKeyValuePair(string("pulse_count_pr"), string("10000"), ns);
			m_ini_axis->AddKeyValuePair(string("encoder_lines"), string("17"), ns);
			m_ini_axis->AddKeyValuePair(string("zero_compensation"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("spd_gear_ratio"), string("1"), ns);
			m_ini_axis->AddKeyValuePair(string("spd_max_speed"), string("10000"), ns);
			m_ini_axis->AddKeyValuePair(string("spd_min_speed"), string("100"), ns);
			m_ini_axis->AddKeyValuePair(string("spd_start_time"), string("10"), ns);
			m_ini_axis->AddKeyValuePair(string("spd_stop_time"), string("5"), ns);
			m_ini_axis->AddKeyValuePair(string("spd_vctrl_mode"), string("2"), ns);
			m_ini_axis->AddKeyValuePair(string("spd_set_dir"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("spd_set_speed"), string("3000"), ns);

			m_ini_axis->AddKeyValuePair(string("spd_speed_check"), string("1"), ns);
			m_ini_axis->AddKeyValuePair(string("spd_speed_match"), string("1"), ns);
			m_ini_axis->AddKeyValuePair(string("spd_speed_diff_limit"), string("10"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_encoder_location"), string("0"), ns);

            m_ini_axis->AddKeyValuePair(string("spd_ctrl_GST"), string("0"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_ctrl_SGB"), string("0"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_ctrl_SFA"), string("1"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_ctrl_ORM"), string("0"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_ctrl_TCW"), string("1"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_ctrl_CWM"), string("0"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_ctrl_TSO"), string("0"), ns);

            m_ini_axis->AddKeyValuePair(string("spd_analog_gain"), string("1000"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_sor_speed"), string("20"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_motor_min_speed"), string("0"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_motor_max_speed"), string("32767"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_gear_speed_low"), string("1500"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_gear_speed_middle"), string("5000"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_gear_speed_high"), string("10000"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_gear_switch_speed1"), string("1500"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_gear_switch_speed2"), string("5000"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_sync_error_gain"), string("200"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_speed_feed_gain"), string("60000"), ns);
            m_ini_axis->AddKeyValuePair(string("spd_pos_ratio_gain"), string("100000"), ns);

			m_ini_axis->AddKeyValuePair(string("fast_locate"), string("1"), ns);
			m_ini_axis->AddKeyValuePair(string("pos_disp_mode"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("encoder_max_cycle"), string("0"), ns);  //���Ȧ��Ĭ��ֵ��256��Ϊ0

			m_ini_axis->AddKeyValuePair(string("sync_axis"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("master_axis_no"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("disp_coord"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("benchmark_offset"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("sync_err_max"), string("0"), ns);
			m_ini_axis->AddKeyValuePair(string("auto_sync"), string("0"), ns);



			for(j = 0; j < 10; j++){
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "axis_home_pos_%d", j+1);
				m_ini_axis->AddKeyValuePair(kname, string("0.0"), ns);
			}

		}
		m_ini_axis->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ���������ļ��ɹ���\n");

	}

	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief ��ȡ��������
 * @return  true--�ɹ�   false--ʧ��
 */
bool ParmManager::ReadToolConfig(){
	int err_code = ERR_NONE;
	char sname[32];
	char kname[64];
	char value[32];

	int i = 0, j = 0;

	IniSection *ns=nullptr;

	if(m_sc_tool_config == nullptr){
//		m_sc_tool_config = new SCToolOffsetConfig[m_sc_system_config->chn_count];  //����ͨ��������
//		m_sc_tool_pot_config = new SCToolPotConfig[m_sc_system_config->chn_count];

		m_sc_tool_config = new SCToolOffsetConfig[m_sc_system_config->max_chn_count];  //�������ͨ����������֧�ֲ����޸�һ������
		m_sc_tool_pot_config = new SCToolPotConfig[m_sc_system_config->max_chn_count];
	}

	if(m_sc_tool_config == nullptr || m_sc_tool_pot_config == nullptr || m_ini_tool == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_tool->Load(TOOL_CONFIG_FILE) == 0){//��ȡ���óɹ�
//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //֧�ֲ����޸�һ������
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);

#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
			//��ȡ��׼������
			memset(kname, 0x00, sizeof(kname));
			sprintf(kname, "geo_comp_basic_x");
			m_sc_tool_config[i].geometry_comp_basic[0] = m_ini_tool->GetDoubleValueOrDefault(sname, kname, 0.0);
			sprintf(kname, "geo_comp_basic_y");
			m_sc_tool_config[i].geometry_comp_basic[1] = m_ini_tool->GetDoubleValueOrDefault(sname, kname, 0.0);
			sprintf(kname, "geo_comp_basic_z");
			m_sc_tool_config[i].geometry_comp_basic[2] = m_ini_tool->GetDoubleValueOrDefault(sname, kname, 0.0);
//			printf("read tool base z: %lf\n", m_sc_tool_config[i].geometry_comp_basic[2]);
#endif

			for(j = 0; j < kMaxToolCount; j++){
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "geo_comp_x_%d", j);
				m_sc_tool_config[i].geometry_compensation[j][0] = 	m_ini_tool->GetDoubleValueOrDefault(sname, kname, 0.0);
				sprintf(kname, "geo_comp_y_%d", j);
				m_sc_tool_config[i].geometry_compensation[j][1] = 	m_ini_tool->GetDoubleValueOrDefault(sname, kname, 0.0);
				sprintf(kname, "geo_comp_z_%d", j);
				m_sc_tool_config[i].geometry_compensation[j][2] = 	m_ini_tool->GetDoubleValueOrDefault(sname, kname, 0.0);

				sprintf(kname, "geo_wear_%d", j);
				m_sc_tool_config[i].geometry_wear[j] = 	m_ini_tool->GetDoubleValueOrDefault(sname, kname, 0.0);

				sprintf(kname, "radius_comp_%d", j);
				m_sc_tool_config[i].radius_compensation[j] = m_ini_tool->GetDoubleValueOrDefault(sname, kname, 0.0);

				sprintf(kname, "radius_wear_%d", j);
				m_sc_tool_config[i].radius_wear[j] = m_ini_tool->GetDoubleValueOrDefault(sname, kname, 0.0);

				//��λ��Ϣ
			//	m_sc_tool_pot_config[i].tool_index = j+1;

				sprintf(kname, "tool_type_%d", j);
				m_sc_tool_pot_config[i].tool_type[j] = m_ini_tool->GetIntValueOrDefault(sname, kname, 0);

				sprintf(kname, "huge_tool_flag_%d", j);
				m_sc_tool_pot_config[i].huge_tool_flag[j] = m_ini_tool->GetIntValueOrDefault(sname, kname, 0);

				sprintf(kname, "tool_pot_index_%d", j);
                m_sc_tool_pot_config[i].tool_pot_index[j] = m_ini_tool->GetIntValueOrDefault(sname, kname, j);

				sprintf(kname, "tool_life_max_%d", j);
				m_sc_tool_pot_config[i].tool_life_max[j] = m_ini_tool->GetIntValueOrDefault(sname, kname, 0);

				sprintf(kname, "tool_life_cur_%d", j);
				m_sc_tool_pot_config[i].tool_life_cur[j] = m_ini_tool->GetIntValueOrDefault(sname, kname, 0);

                sprintf(kname, "tool_threshold_%d", j);
                m_sc_tool_pot_config[i].tool_threshold[j] = m_ini_tool->GetIntValueOrDefault(sname, kname, 0);

                sprintf(kname, "tool_life_type_%d", j);
                m_sc_tool_pot_config[i].tool_life_type[j] = m_ini_tool->GetIntValueOrDefault(sname, kname, 0);
			}

		}

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡ���������ļ��ɹ���\n");
	}
	else {
		if(m_ini_tool->CreateFile(TOOL_CONFIG_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ�ϵ��������ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

        for(i = 0; i < m_sc_system_config->chn_count; i++){
        //for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //֧�ֲ����޸�һ������
			//����Ĭ��ini����
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			ns = m_ini_tool->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			memset(kname, 0x00, sizeof(kname));
#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
			sprintf(kname, "geo_comp_basic_x");
			m_sc_tool_config[i].geometry_comp_basic[0] = 0.0;
			m_ini_tool->AddKeyValuePair(kname, string("0.0"), ns);
			sprintf(kname, "geo_comp_basic_y");
			m_sc_tool_config[i].geometry_comp_basic[1] = 0.0;
			m_ini_tool->AddKeyValuePair(kname, string("0.0"), ns);
			sprintf(kname, "geo_comp_basic_z");
			m_sc_tool_config[i].geometry_comp_basic[2] = 0.0;
			m_ini_tool->AddKeyValuePair(kname, string("0.0"), ns);
#endif

			for(j = 0; j < kMaxToolCount; j++){
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "geo_comp_x_%d", j);
				m_sc_tool_config[i].geometry_compensation[j][0] = 0.0;
				m_ini_tool->AddKeyValuePair(kname, string("0.0"), ns);

				sprintf(kname, "geo_comp_y_%d", j);
				m_sc_tool_config[i].geometry_compensation[j][1] = 0.0;
				m_ini_tool->AddKeyValuePair(kname, string("0.0"), ns);
				sprintf(kname, "geo_comp_z_%d", j);
//				m_sc_tool_config[i].geometry_compensation[j][2] = 5.0;   //for test
//				m_ini_tool->AddKeyValuePair(kname, string("5.0"), ns);
                m_sc_tool_config[i].geometry_compensation[j][2] = 0.0;   //for test
                m_ini_tool->AddKeyValuePair(kname, string("0.0"), ns);

				sprintf(kname, "geo_wear_%d", j);
				m_sc_tool_config[i].geometry_wear[j] = 	0.0;
				m_ini_tool->AddKeyValuePair(kname, string("0.0"), ns);

				sprintf(kname, "radius_comp_%d", j);
				m_sc_tool_config[i].radius_compensation[j] = 0.0;
				m_ini_tool->AddKeyValuePair(kname, string("0.0"), ns);

				sprintf(kname, "radius_wear_%d", j);
				m_sc_tool_config[i].radius_wear[j] = 0.0;
				m_ini_tool->AddKeyValuePair(kname, string("0.0"), ns);

				//��λ��Ϣ
                sprintf(kname, "tool_type_%d", j);
                m_sc_tool_pot_config[i].tool_type[j] = 0;
                m_ini_tool->AddKeyValuePair(kname, string("0"), ns);

		//		m_sc_tool_pot_config[i].tool_index = j+1;
				sprintf(kname, "huge_tool_flag_%d", j);
				m_sc_tool_pot_config[i].huge_tool_flag[j] = 0;
				m_ini_tool->AddKeyValuePair(kname, string("0"), ns);

				sprintf(kname, "tool_pot_index_%d", j);
                sprintf(value, "%d", j);
                m_sc_tool_pot_config[i].tool_pot_index[j] = j;
				m_ini_tool->AddKeyValuePair(kname, value, ns);

                sprintf(kname, "tool_life_cur_%d", j);
                m_sc_tool_pot_config[i].tool_life_cur[j] = 0;
                m_ini_tool->AddKeyValuePair(kname, string("0"), ns);

                sprintf(kname, "tool_life_max_%d", j);
                m_sc_tool_pot_config[i].tool_life_max[j] = 0;
                m_ini_tool->AddKeyValuePair(kname, string("0"), ns);

                sprintf(kname, "tool_threshold_%d", j);
                m_sc_tool_pot_config[i].tool_threshold[j] = 0;
                m_ini_tool->AddKeyValuePair(kname, string("0"), ns);

                sprintf(kname, "tool_life_type_%d", j);
                m_sc_tool_pot_config[i].tool_life_type[j] = 0;
                m_ini_tool->AddKeyValuePair(kname, string("0"), ns);
			}
		}

		m_ini_tool->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ�ϵ��������ļ��ɹ���\n");

	}

	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief ��ȡ��������ϵ����
 * @return  true--�ɹ�   false--ʧ��
 */
bool ParmManager::ReadCoordConfig(){
	int err_code = ERR_NONE;
	char sname[32];
	char kname[64];


	int i = 0, j = 0, k = 0;

	IniSection *ns=nullptr;

	if(m_sc_coord_config == nullptr){
//		m_sc_coord_config = new SCCoordConfig[m_sc_system_config->chn_count * kWorkCoordCount];  //����ͨ��������
		m_sc_coord_config = new SCCoordConfig[m_sc_system_config->max_chn_count * kWorkCoordCount];  //�������ͨ����������֧�ֲ����޸�һ������
	}

	if(m_sc_coord_config == nullptr || m_ini_coord == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_coord->Load(WORK_COORD_FILE) == 0){//��ȡ���óɹ�
//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //֧�ֲ����޸�һ������
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			for(j = 0; j < kWorkCoordCount; j++){
				for(k = 0; k < kMaxAxisChn; k++){
					memset(kname, 0x00, sizeof(kname));
					sprintf(kname, "offset_%d_%d", j, k);
					m_sc_coord_config[i*kWorkCoordCount+j].offset[k] = 	m_ini_coord->GetDoubleValueOrDefault(sname, kname, 0.0);
				}
			}

		}
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡ��������ϵ�����ļ��ɹ���\n");
	}
	else {
		if(m_ini_coord->CreateFile(WORK_COORD_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ�Ϲ�������ϵ�����ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //֧�ֲ����޸�һ������
			//����Ĭ��ini����
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			ns = m_ini_coord->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}
			for(j = 0; j < kWorkCoordCount; j++){
				for(k = 0; k < kMaxAxisChn; k++){
					memset(kname, 0x00, sizeof(kname));
					sprintf(kname, "offset_%d_%d", j, k);
					m_sc_coord_config[i*kWorkCoordCount+j].offset[k] = 10.0;   //for test
					m_ini_coord->AddKeyValuePair(kname, string("10.0"), ns);
				}
			}
		}

		m_ini_coord->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ�Ϲ�������ϵ�����ļ��ɹ���\n");
	}

	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief ��ȡ��չ��������ϵ����
 * @return true--�ɹ�   false--ʧ��
 */
bool ParmManager::ReadExCoordConfig(){
	int err_code = ERR_NONE;
	char sname[32];
	char kname[64];

	int i = 0, j = 0, k = 0;
	IniSection *ns=nullptr;

	if(m_ini_ex_coord == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

//	for(int i = 0; i < m_sc_system_config->chn_count; i++){
	for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //֧�ֲ����޸�һ������
		if(m_sc_channel_config[i].ex_coord_count > 0 && m_sc_ex_coord_config[i] == nullptr){
			m_sc_ex_coord_config[i] = new SCCoordConfig[m_sc_channel_config[i].ex_coord_count];	//����ͨ��������

			if(m_sc_ex_coord_config[i] == nullptr){
				err_code = ERR_SC_INIT;

				goto END;
			}
		}
	}


	if(m_ini_ex_coord->Load(EX_WORK_COORD_FILE) == 0){//��ȡ���óɹ�

//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //֧�ֲ����޸�һ������
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			for(j = 0; j < m_sc_channel_config[i].ex_coord_count; j++){
				for(k = 0; k < kMaxAxisChn; k++){
					memset(kname, 0x00, sizeof(kname));
					sprintf(kname, "offset_%d_%d", j, k);
					m_sc_ex_coord_config[i][j].offset[k] = m_ini_ex_coord->GetDoubleValueOrDefault(sname, kname, 0.0);
				}
			}

		}

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡ��չ��������ϵ�����ļ��ɹ���\n");
	}
	else {
		if(m_ini_ex_coord->CreateFile(EX_WORK_COORD_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ����չ��������ϵ�����ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //֧�ֲ����޸�һ������
			//����Ĭ��ini����
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			ns = m_ini_ex_coord->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			for(j = 0; j < m_sc_channel_config[i].ex_coord_count; j++){
				for(k = 0; k < kMaxAxisChn; k++){
					memset(kname, 0x00, sizeof(kname));
					sprintf(kname, "ex_offset_%d_%d", j, k);
					m_sc_ex_coord_config[i][j].offset[k] = 100.0;    //for test
					m_ini_ex_coord->AddKeyValuePair(kname, string("100.0"), ns);

				}
			}

		}

		m_ini_ex_coord->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ����չ��������ϵ�����ļ��ɹ���\n");

	}

	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief ��ȡIO�ض�������
 * @return true--�ɹ�   false--ʧ��
 */
bool ParmManager::ReadIoRemapConfig(){
	int err_code = ERR_NONE;
	char sname[32];
	char kname[64];
	IoRemapInfo io_info;

	int total_count = 0;
	int i = 0;
//	IniSection *ns=nullptr;

	vector<string> sec_array;

	this->m_sc_io_remap_list.Clear();

	if(m_ini_io_remap == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}


	if(m_ini_io_remap->Load(IO_REMAP_FILE) == 0){//��ȡ���óɹ�

		sec_array.clear();
		total_count = m_ini_io_remap->GetSections(&sec_array);   //��ȡ����

//		printf("read io file:%d\n", total_count);

		for(i = 1; i < total_count; i++){
			memset(sname, 0x00, sizeof(sname));
			sec_array[i].copy(sname, sec_array[i].length(), 0);

	//		printf("read io remap:sec[%s]\n", sname);

			memset(kname, 0x00, sizeof(kname));
			sprintf(kname, "iotype");
			io_info.iotype = m_ini_io_remap->GetIntValueOrDefault(sname, kname, 0);

			sprintf(kname, "addr");
			io_info.addr = m_ini_io_remap->GetIntValueOrDefault(sname, kname, 0);

			sprintf(kname, "bit");
			io_info.bit = m_ini_io_remap->GetIntValueOrDefault(sname, kname, 0);

			sprintf(kname, "addrmap");
			io_info.addrmap = m_ini_io_remap->GetIntValueOrDefault(sname, kname, 0);

			sprintf(kname, "newaddr");
			io_info.newaddr = m_ini_io_remap->GetIntValueOrDefault(sname, kname, 0);

			sprintf(kname, "newbit");
			io_info.newbit = m_ini_io_remap->GetIntValueOrDefault(sname, kname, 0);

			sprintf(kname, "valtype");
			io_info.valtype = m_ini_io_remap->GetIntValueOrDefault(sname, kname, 0);

			this->m_sc_io_remap_list.Append(io_info);
		}

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡIO�ض��������ļ��ɹ���\n");
	}else {
		if(m_ini_io_remap->CreateFile(IO_REMAP_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ��IO�ض��������ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

		m_ini_ex_coord->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ��IO�ض��������ļ��ɹ���\n");

	}

	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief ��ȡͨ��״̬����
 * @return
 */
bool ParmManager::ReadChnStateScene(){
	int err_code = ERR_NONE;
	char sname[32];
	char kname[64];

	if(m_ini_chn_scene->Load(CHN_SCENE_FILE) == 0){//��ȡ���óɹ�

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡͨ��״̬�ļ��ɹ���\n");
	}
	else {
		if(m_ini_chn_scene->CreateFile(CHN_SCENE_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ��ͨ��״̬�ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

		IniSection *ns=nullptr;
//		for(int i = 0; i < m_sc_system_config->chn_count; i++){
		for(int i = 0; i < m_sc_system_config->max_chn_count; i++){  //֧�ֲ����޸�һ������
			//����Ĭ��ini����
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			ns = m_ini_chn_scene->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			memset(kname, 0x00, sizeof(kname));
			sprintf(kname, "cur_nc_file");
			m_ini_chn_scene->AddKeyValuePair(kname, string(""), ns);

			memset(kname, 0x00, sizeof(kname));
			sprintf(kname, "cur_work_piece");
			m_ini_chn_scene->AddKeyValuePair(kname, string("0"), ns);

			memset(kname, 0x00, sizeof(kname));
			sprintf(kname, "cur_tool");
			m_ini_chn_scene->AddKeyValuePair(kname, string("0"), ns);

			//��ǰ���ղ������
			memset(kname, 0x00, sizeof(kname));
			sprintf(kname, "cur_proc_index");
			m_ini_chn_scene->AddKeyValuePair(kname, string("0"), ns);

		}

		m_ini_chn_scene->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ��ͨ��״̬�ļ��ɹ���\n");

	}
	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief ��ȡ�ݲ�����
 * @return
 */
bool ParmManager::ReadPcData(){
	int err_code = ERR_NONE;
	char sname[32];
	char kname[64];

	int i = 0, j = 0;

	IniSection *ns = nullptr;

	if(m_sc_pc_table == nullptr){
		m_sc_pc_table = new AxisPitchCompTable();
	}

	if(m_sc_pc_table == nullptr || m_ini_pc_table == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_pc_table->Load(PITCH_COMP_FILE) == 0){//��ȡ���óɹ�

		for(i = 0; i < m_sc_system_config->axis_count; i++){
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "axis_%d", i+1);   //��1��ʼ

			if(m_sc_pc_table->pc_table[i] == nullptr){

				if(m_sc_axis_config[i].pc_type == 0){

					m_sc_pc_table->pc_table[i] = new double[m_sc_axis_config[i].pc_count];      //�����ݲ�
				}else if(m_sc_axis_config[i].pc_type == 1){
					m_sc_pc_table->pc_table[i] = new double[m_sc_axis_config[i].pc_count*2];    //˫���ݲ�
				}

				if(m_sc_pc_table->pc_table[i] == nullptr){

					err_code = ERR_SC_INIT;

					goto END;
				}
			}



			if(m_sc_axis_config[i].pc_type == 0){
				for(j = 0; j < this->m_sc_axis_config[i].pc_count; j++){
					memset(kname, 0x00, sizeof(kname));
					sprintf(kname, "pc_pos_%d", j+1);    //�����ݲ�
					m_sc_pc_table->pc_table[i][j] = m_ini_pc_table->GetDoubleValueOrDefault(sname, kname, 0.0);
				}
			}else if(m_sc_axis_config[i].pc_type == 1){
				for(j = 0; j < this->m_sc_axis_config[i].pc_count; j++){
					memset(kname, 0x00, sizeof(kname));
					sprintf(kname, "pc_pos_%d", j+1);    //�����ݲ�
					m_sc_pc_table->pc_table[i][j] = m_ini_pc_table->GetDoubleValueOrDefault(sname, kname, 0.0);
					sprintf(kname, "pc_neg_%d", j+1);    //�����ݲ�
					m_sc_pc_table->pc_table[i][j+m_sc_axis_config[i].pc_count] = m_ini_pc_table->GetDoubleValueOrDefault(sname, kname, 0.0);

//					if(i== 0 && j == this->m_sc_axis_config[i].pc_count-1){
//						printf("axis0, pos=%lf, neg=%lf\n", m_sc_pc_table->pc_table[i][j], m_sc_pc_table->pc_table[i][j+m_sc_axis_config[i].pc_count]);
//					}
				}
			}
		}
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡ���ݲ������ļ��ɹ���\n");
	}
	else {
		if(m_ini_pc_table->CreateFile(PITCH_COMP_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ���ݲ������ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

		for(i = 0; i < m_sc_system_config->axis_count; i++){

			if(m_sc_pc_table->pc_table[i] == nullptr){
				if(m_sc_axis_config[i].pc_type == 0)
					m_sc_pc_table->pc_table[i] = new double[m_sc_axis_config[i].pc_count];      //�����ݲ�
				else
					m_sc_pc_table->pc_table[i] = new double[m_sc_axis_config[i].pc_count*2];    //˫���ݲ�

				if(m_sc_pc_table->pc_table[i] == nullptr){
					err_code = ERR_SC_INIT;

					goto END;
				}
			}

			//����Ĭ��ֵ����Ϊ0
			if(m_sc_axis_config[i].pc_type == 0){//�����ݲ�
				memset(m_sc_pc_table->pc_table[i], 0x00, sizeof(double)*m_sc_axis_config[i].pc_count);
			}else{//˫���ݲ�
				memset(m_sc_pc_table->pc_table[i], 0x00, sizeof(double)*m_sc_axis_config[i].pc_count*2);
			}


			//����Ĭ��ini����
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "axis_%d", i+1);    //��1��ʼ
			ns = m_ini_pc_table->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			if(m_sc_axis_config[i].pc_type == 0){//�����ݲ�
				for(j = 0; j < this->m_sc_axis_config[i].pc_count; j++){
					memset(kname, 0x00, sizeof(kname));
					sprintf(kname, "pc_pos_%d", j+1);
					m_ini_pc_table->AddKeyValuePair(kname, string("0.0"), ns);
				}
			}else{//˫���ݲ�
				for(j = 0; j < this->m_sc_axis_config[i].pc_count; j++){
					memset(kname, 0x00, sizeof(kname));
					sprintf(kname, "pc_pos_%d", j+1);
					m_ini_pc_table->AddKeyValuePair(kname, string("0.0"), ns);
					sprintf(kname, "pc_neg_%d", j+1);
					m_ini_pc_table->AddKeyValuePair(kname, string("0.0"), ns);
				}
			}


		}
		m_ini_pc_table->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ���ݲ������ļ��ɹ���\n");

	}

	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

#ifdef USES_FIVE_AXIS_FUNC
/**
 * @brief ��ȡ��������
 * @return
 */
bool ParmManager::ReadFiveAxisConfig(){
	int err_code = ERR_NONE;

	char sname[20];

	int i = 0;
	IniSection *ns = nullptr;

	if(m_five_axis_config == nullptr){
//		m_five_axis_config = new FiveAxisConfig[m_sc_system_config->chn_count]();//����ͨ��������;
		m_five_axis_config = new FiveAxisConfig[m_sc_system_config->max_chn_count]();//�������ͨ����������֧�ֲ����޸�һ������
	}

	if(m_five_axis_config == nullptr || this->m_ini_five_axis == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}


	if(m_ini_five_axis->Load(FIVE_AXIS_CONFIG_FILE) == 0){//��ȡ���óɹ�

//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){    //֧�ֲ����޸�һ������
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			m_five_axis_config[i].five_axis_type = m_ini_five_axis->GetIntValueOrDefault(sname, "five_axis_type", 0);   //�����������
			m_five_axis_config[i].x_offset_1 = m_ini_five_axis->GetDoubleValueOrDefault(sname, "x_offset_1", 10.0);    //��һת��������Ե����X����
			m_five_axis_config[i].y_offset_1 = m_ini_five_axis->GetDoubleValueOrDefault(sname, "y_offset_1", 10.0);    //��һת��������Ե����Y����
			m_five_axis_config[i].z_offset_1 = m_ini_five_axis->GetDoubleValueOrDefault(sname, "z_offset_1", 10.0);    //��һת��������Ե����Y����
			m_five_axis_config[i].post_dir_1 = m_ini_five_axis->GetIntValueOrDefault(sname, "post_dir_1", 0);   	//��һת����ת������
			m_five_axis_config[i].range_limit_1 = m_ini_five_axis->GetIntValueOrDefault(sname, "range_limit_1", 0);   //��һת����ת����

			m_five_axis_config[i].x_offset_2 = m_ini_five_axis->GetDoubleValueOrDefault(sname, "x_offset_2", 10.0);    //�ڶ�ת��������Ե����X����
			m_five_axis_config[i].y_offset_2 = m_ini_five_axis->GetDoubleValueOrDefault(sname, "y_offset_2", 10.0);    //�ڶ�ת��������Ե����Y����
			m_five_axis_config[i].z_offset_2 = m_ini_five_axis->GetDoubleValueOrDefault(sname, "z_offset_2", 10.0);    //�ڶ�ת��������Ե����Y����
			m_five_axis_config[i].post_dir_2 = m_ini_five_axis->GetIntValueOrDefault(sname, "post_dir_2", 0);   	//�ڶ�ת����ת������
			m_five_axis_config[i].range_limit_2 = m_ini_five_axis->GetIntValueOrDefault(sname, "range_limit_2", 0);   //�ڶ�ת����ת����

			m_five_axis_config[i].rot_swing_arm_len = m_ini_five_axis->GetDoubleValueOrDefault(sname, "rot_swing_arm_len", 10.0);   //������ת�ڱ۳���
			m_five_axis_config[i].five_axis_coord = m_ini_five_axis->GetIntValueOrDefault(sname, "five_axis_coord", 0);   //����������ϵѡ��

			m_five_axis_config[i].speed_limit_x = m_ini_five_axis->GetDoubleValueOrDefault(sname, "speed_limit_x", 10000.0);   //��������X���ٶ�����
			m_five_axis_config[i].speed_limit_y = m_ini_five_axis->GetDoubleValueOrDefault(sname, "speed_limit_y", 10000.0);   //��������X���ٶ�����
			m_five_axis_config[i].speed_limit_z = m_ini_five_axis->GetDoubleValueOrDefault(sname, "speed_limit_z", 10000.0);   //��������X���ٶ�����
			m_five_axis_config[i].speed_limit_a = m_ini_five_axis->GetDoubleValueOrDefault(sname, "speed_limit_a", 10000.0);   //��������X���ٶ�����
			m_five_axis_config[i].speed_limit_b = m_ini_five_axis->GetDoubleValueOrDefault(sname, "speed_limit_b", 10000.0);   //��������X���ٶ�����
			m_five_axis_config[i].speed_limit_c = m_ini_five_axis->GetDoubleValueOrDefault(sname, "speed_limit_c", 10000.0);   //��������X���ٶ�����

			m_five_axis_config[i].intp_angle_step = m_ini_five_axis->GetDoubleValueOrDefault(sname, "intp_angle_step", 1.0);   //���������ֲ岹�ǶȲ���
			m_five_axis_config[i].intp_len_step = m_ini_five_axis->GetDoubleValueOrDefault(sname, "intp_len_step", 1.0);   //���������ֲ岹��С����
			m_five_axis_config[i].five_axis_speed_plan = m_ini_five_axis->GetIntValueOrDefault(sname, "five_axis_speed_plan", 0);   //���������ٶȹ滮
			m_five_axis_config[i].five_axis_plan_mode = m_ini_five_axis->GetIntValueOrDefault(sname, "five_axis_plan_mode", 0);   //�����ٶȹ滮��ʽ
		}
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡ���������ļ��ɹ���\n");

	}
	else {
		if(m_ini_five_axis->CreateFile(FIVE_AXIS_CONFIG_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ�����������ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}
//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){    //֧�ֲ����޸�һ������
			//����Ĭ��ֵ
			m_five_axis_config[i].five_axis_type = 0;  //�����������
			m_five_axis_config[i].x_offset_1 = 10.0;    //��һת��������Ե����X����
			m_five_axis_config[i].y_offset_1 = 10.0;    //��һת��������Ե����Y����
			m_five_axis_config[i].z_offset_1 = 10.0;    //��һת��������Ե����Y����
			m_five_axis_config[i].post_dir_1 = 0;   //��һת����ת������
			m_five_axis_config[i].range_limit_1 = 0;   //��һת����ת����

			m_five_axis_config[i].x_offset_2 = 10.0;    //�ڶ�ת��������Ե����X����
			m_five_axis_config[i].y_offset_2 = 10.0;    //�ڶ�ת��������Ե����Y����
			m_five_axis_config[i].z_offset_2 = 10.0;    //�ڶ�ת��������Ե����Y����
			m_five_axis_config[i].post_dir_2 = 0;   //�ڶ�ת����ת������
			m_five_axis_config[i].range_limit_2 = 0;   //�ڶ�ת����ת����

			m_five_axis_config[i].rot_swing_arm_len = 10.0;   //������ת�ڱ۳���
			m_five_axis_config[i].five_axis_coord = 0;   //����������ϵѡ��

			m_five_axis_config[i].speed_limit_x = 10000.0;   //��������X���ٶ�����
			m_five_axis_config[i].speed_limit_y = 10000.0;   //��������Y���ٶ�����
			m_five_axis_config[i].speed_limit_z = 10000.0;   //��������Z���ٶ�����
			m_five_axis_config[i].speed_limit_a = 10000.0;   //��������A���ٶ�����
			m_five_axis_config[i].speed_limit_b = 10000.0;   //��������B���ٶ�����
			m_five_axis_config[i].speed_limit_c = 10000.0;   //��������C���ٶ�����

			m_five_axis_config[i].intp_angle_step = 1.0;   //���������ֲ岹�ǶȲ���
			m_five_axis_config[i].intp_len_step = 1.0;   //���������ֲ岹��С����
			m_five_axis_config[i].five_axis_speed_plan = 0;   //���������ٶȹ滮
			m_five_axis_config[i].five_axis_plan_mode = 0;   //�����ٶȹ滮��ʽ

			//����Ĭ��ini����
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			ns = m_ini_five_axis->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			m_ini_five_axis->AddKeyValuePair(string("five_axis_type"), string("0"), ns);   //�����������
			m_ini_five_axis->AddKeyValuePair(string("x_offset_1"), string("10.0"), ns);   //��һת��������Ե����X����
			m_ini_five_axis->AddKeyValuePair(string("y_offset_1"), string("10.0"), ns);   //��һת��������Ե����Y����
			m_ini_five_axis->AddKeyValuePair(string("z_offset_1"), string("10.0"), ns);   //��һת��������Ե����Z����
			m_ini_five_axis->AddKeyValuePair(string("post_dir_1"), string("0"), ns);     //��һת����ת������
			m_ini_five_axis->AddKeyValuePair(string("range_limit_1"), string("0"), ns);   //��һת����ת����

			m_ini_five_axis->AddKeyValuePair(string("x_offset_2"), string("10.0"), ns);   //�ڶ�ת��������Ե����X����
			m_ini_five_axis->AddKeyValuePair(string("y_offset_2"), string("10.0"), ns);   //�ڶ�ת��������Ե����Y����
			m_ini_five_axis->AddKeyValuePair(string("z_offset_2"), string("10.0"), ns);   //�ڶ�ת��������Ե����Z����
			m_ini_five_axis->AddKeyValuePair(string("post_dir_2"), string("0"), ns);     //�ڶ�ת����ת������
			m_ini_five_axis->AddKeyValuePair(string("range_limit_2"), string("0"), ns);   //�ڶ�ת����ת����

			m_ini_five_axis->AddKeyValuePair(string("rot_swing_arm_len"), string("10.0"), ns);   //������ת�ڱ۳���
			m_ini_five_axis->AddKeyValuePair(string("five_axis_coord"), string("0"), ns);   //����������ϵѡ��
			m_ini_five_axis->AddKeyValuePair(string("speed_limit_x"), string("10000.0"), ns);   //��������X���ٶ�����
			m_ini_five_axis->AddKeyValuePair(string("speed_limit_y"), string("10000.0"), ns);   //��������Y���ٶ�����
			m_ini_five_axis->AddKeyValuePair(string("speed_limit_z"), string("10000.0"), ns);   //��������Z���ٶ�����
			m_ini_five_axis->AddKeyValuePair(string("speed_limit_a"), string("10000.0"), ns);   //��������A���ٶ�����
			m_ini_five_axis->AddKeyValuePair(string("speed_limit_b"), string("10000.0"), ns);   //��������B���ٶ�����
			m_ini_five_axis->AddKeyValuePair(string("speed_limit_c"), string("10000.0"), ns);   //��������C���ٶ�����

			m_ini_five_axis->AddKeyValuePair(string("intp_angle_step"), string("1.0"), ns);   //���������ֲ岹�ǶȲ���
			m_ini_five_axis->AddKeyValuePair(string("intp_len_step"), string("1.0"), ns);   //���������ֲ岹��С����
			m_ini_five_axis->AddKeyValuePair(string("five_axis_speed_plan"), string("0"), ns);   //���������ٶȹ滮
			m_ini_five_axis->AddKeyValuePair(string("five_axis_plan_mode"), string("0"), ns);   //�����ٶȹ滮��ʽ
		}

		m_ini_five_axis->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ�����������ļ��ɹ���\n");


	}
	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief �����������
 * @param chn_index �� ͨ����
 * @param param_no : ������
 * @param value �� ����ֵ
 * @return
 */
bool ParmManager::UpdateFiveAxisParam(uint16_t chn_index, uint32_t param_no, ParamValue &value){
	bool res = true;
//	if(chn_index >= this->m_sc_system_config->chn_count){
	if(chn_index >= this->m_sc_system_config->max_chn_count){   //֧�ֲ����޸ģ�һ������
		g_ptr_trace->PrintLog(LOG_ALARM, "����������£�ͨ���ŷǷ���%hhu", chn_index);
		return false;
	}
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	sprintf(sname, "channel_%hhu", chn_index);
	switch(param_no){
	case 3000:	//�����������
		sprintf(kname, "five_axis_type");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 3010:	//��һת��������Ե����X����
		sprintf(kname, "x_offset_1");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3011:  //��һת��������Ե����Y����
		sprintf(kname, "y_offset_1");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3012:  //��һת��������Ե����Z����
		sprintf(kname, "z_offset_1");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3013:  //��һת����ת������
		sprintf(kname, "post_dir_1");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 3014:  //��һת����ת����
		sprintf(kname, "range_limit_1");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 3020:	//�ڶ�ת��������Ե����X����
		sprintf(kname, "x_offset_2");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3021:  //�ڶ�ת��������Ե����Y����
		sprintf(kname, "y_offset_2");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3022:  //�ڶ�ת��������Ե����Z����
		sprintf(kname, "z_offset_2");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3023:  //�ڶ�ת����ת������
		sprintf(kname, "post_dir_2");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 3024:  //�ڶ�ת����ת����
		sprintf(kname, "range_limit_2");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 3030:	//������ת�ڱ۳���
		sprintf(kname, "rot_swing_arm_len");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3031:	//����������ϵѡ��
		sprintf(kname, "five_axis_coord");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 3040:	//��������X���ٶ�����
		sprintf(kname, "speed_limit_x");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3041:	//��������Y���ٶ�����
		sprintf(kname, "speed_limit_y");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3042:	//��������Z���ٶ�����
		sprintf(kname, "speed_limit_z");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3043:	//��������A���ٶ�����
		sprintf(kname, "speed_limit_a");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3044:	//��������B���ٶ�����
		sprintf(kname, "speed_limit_b");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3045:	//��������C���ٶ�����
		sprintf(kname, "speed_limit_c");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3050:	//���������ֲ岹�ǶȲ���
		sprintf(kname, "intp_angle_step");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3051:	//���������ֲ岹��С����
		sprintf(kname, "intp_len_step");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3060:	//���������ٶȹ滮
		sprintf(kname, "five_axis_speed_plan");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 3061:	//�����ٶȹ滮��ʽ
		sprintf(kname, "five_axis_plan_mode");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;

	default:	//
		g_ptr_trace->PrintLog(LOG_ALARM, "����������£������ŷǷ���%d", param_no);
		res = false;
		return res;
	}
	m_ini_five_axis->Save();

	return res;
}

/**
 * @brief
 * @param chn_index �� ͨ����
 * @param param_no : ������
 * @param value �� ����ֵ
 */
void ParmManager::ActiveFiveAxisParam(uint16_t chn_index, uint32_t param_no, ParamValue &value){

	ChannelControl *p_chn = ChannelEngine::GetInstance()->GetChnControl(chn_index);
	if(p_chn == nullptr)
		return;
	switch(param_no){
	case 3000:	//�����������
		m_five_axis_config[chn_index].five_axis_type = value.value_uint8;
		break;
	case 3010:	//��һת��������Ե����X����
		m_five_axis_config[chn_index].x_offset_1 = value.value_double;
		p_chn->UpdateFiveAxisParam(X_OFFSET_1);
		break;
	case 3011:  //��һת��������Ե����Y����
		m_five_axis_config[chn_index].y_offset_1 = value.value_double;
		p_chn->UpdateFiveAxisParam(Y_OFFSET_1);
		break;
	case 3012:  //��һת��������Ե����Z����
		m_five_axis_config[chn_index].z_offset_1 = value.value_double;
		p_chn->UpdateFiveAxisParam(Z_OFFSET_1);
		break;
	case 3013:  //��һת����ת������
		m_five_axis_config[chn_index].post_dir_1 = value.value_uint8;
		p_chn->UpdateFiveAxisParam(POST_DIR_1);
		break;
	case 3014:  //��һת����ת����
		m_five_axis_config[chn_index].range_limit_1 = value.value_uint8;
		p_chn->UpdateFiveAxisRotParam();
		break;
	case 3020:	//�ڶ�ת��������Ե����X����
		m_five_axis_config[chn_index].x_offset_2 = value.value_double;
		p_chn->UpdateFiveAxisParam(X_OFFSET_2);
		break;
	case 3021:  //�ڶ�ת��������Ե����Y����
		m_five_axis_config[chn_index].y_offset_2 = value.value_double;
		p_chn->UpdateFiveAxisParam(Y_OFFSET_2);
		break;
	case 3022:  //�ڶ�ת��������Ե����Z����
		m_five_axis_config[chn_index].z_offset_2 = value.value_double;
		p_chn->UpdateFiveAxisParam(Z_OFFSET_2);
		break;
	case 3023:  //�ڶ�ת����ת������
		m_five_axis_config[chn_index].post_dir_2 = value.value_uint8;
		p_chn->UpdateFiveAxisParam(POST_DIR_2);
		break;
	case 3024:  //�ڶ�ת����ת����
		m_five_axis_config[chn_index].range_limit_2 = value.value_uint8;
		p_chn->UpdateFiveAxisRotParam();
		break;
	case 3030:	//������ת�ڱ۳���
		m_five_axis_config[chn_index].rot_swing_arm_len = value.value_double;
		p_chn->UpdateFiveAxisParam(ROT_SWING_ARM_LEN);
		break;
	case 3031:	//����������ϵѡ��
		m_five_axis_config[chn_index].five_axis_coord = value.value_uint8;
		p_chn->UpdateFiveAxisParam(PROGRAM_COORD);
		break;
	case 3040:	//��������X���ٶ�����
		m_five_axis_config[chn_index].speed_limit_x = value.value_double;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT_X);
		break;
	case 3041:	//��������Y���ٶ�����
		m_five_axis_config[chn_index].speed_limit_y = value.value_double;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT_Y);
		break;
	case 3042:	//��������Z���ٶ�����
		m_five_axis_config[chn_index].speed_limit_z = value.value_double;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT_Z);
		break;
	case 3043:	//��������A���ٶ�����
		m_five_axis_config[chn_index].speed_limit_a = value.value_double;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT_A);
		break;
	case 3044:	//��������B���ٶ�����
		m_five_axis_config[chn_index].speed_limit_b = value.value_double;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT_B);
		break;
	case 3045:	//��������C���ٶ�����
		m_five_axis_config[chn_index].speed_limit_c = value.value_double;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT_C);
		break;
	case 3050:	//���������ֲ岹�ǶȲ���
		m_five_axis_config[chn_index].intp_angle_step = value.value_double;
		p_chn->UpdateFiveAxisParam(INTP_ANGLE_STEP);
		break;
	case 3051:	//���������ֲ岹��С����
		m_five_axis_config[chn_index].intp_len_step = value.value_double;
		p_chn->UpdateFiveAxisParam(INTP_LEN_STEP);
		break;
	case 3060:	//���������ٶȹ滮
		m_five_axis_config[chn_index].five_axis_speed_plan = value.value_uint8;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT);
		break;
	case 3061:	//�����ٶȹ滮��ʽ
		m_five_axis_config[chn_index].five_axis_plan_mode = value.value_uint8;
		p_chn->UpdateFiveAxisParam(SPEED_PLAN_MODE);
		break;
	default:	//
		g_ptr_trace->PrintLog(LOG_ALARM, "����������£������ŷǷ���%d", param_no);
		break;
	}
}

#endif

#ifdef USES_GRIND_MACHINE
/**
 * @brief ��ȡĥ��ר�ò���
 * @return
 */
bool ParmManager::ReadGrindConfig(){
	int err_code = ERR_NONE;

	int i = 0;

	if(m_grind_config == nullptr){
		m_grind_config = new GrindConfig();
	}

	if(m_grind_config == nullptr || m_ini_grind == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}


	if(m_ini_grind->Load(GRIND_CONFIG_FILE) == 0){//��ȡ���óɹ�

		m_grind_config->wheel_radius = m_ini_grind->GetDoubleValueOrDefault("grind", "wheel_radius", 80);
		m_grind_config->safe_pos_x = m_ini_grind->GetDoubleValueOrDefault("grind", "safe_pos_x", 30);
		m_grind_config->safe_pos_c = m_ini_grind->GetDoubleValueOrDefault("grind", "safe_pos_c", 0);
		m_grind_config->shock_z_max_dis = m_ini_grind->GetDoubleValueOrDefault("grind", "shock_z_max_dis", 4);
		m_grind_config->shock_x_compen_ratio = m_ini_grind->GetDoubleValueOrDefault("grind", "shock_x_compen_ratio", -2.5);
		m_grind_config->shock_speed = m_ini_grind->GetDoubleValueOrDefault("grind", "shock_speed", 120);
		m_grind_config->shock_manual_ratio_ctrl = m_ini_grind->GetIntValueOrDefault("grind", "shock_manual_ratio_ctrl", 0);
		m_grind_config->shock_init_dir = m_ini_grind->GetIntValueOrDefault("grind", "shock_init_dir", 1);
		m_grind_config->grind_dir = m_ini_grind->GetIntValueOrDefault("grind", "grind_dir", 0);
		m_grind_config->corner_vec_speed = m_ini_grind->GetDoubleValueOrDefault("grind", "corner_vec_speed", 1800);
		m_grind_config->grind_smooth_fun = m_ini_grind->GetIntValueOrDefault("grind", "grind_smooth_fun", 0);
		m_grind_config->grind_smooth_time = m_ini_grind->GetIntValueOrDefault("grind", "grind_smooth_time", 2);
		m_grind_config->wheel_raidus_multi[0] = m_ini_grind->GetDoubleValueOrDefault("grind", "wheel_raidus_1", 30);
		m_grind_config->wheel_raidus_multi[1] = m_ini_grind->GetDoubleValueOrDefault("grind", "wheel_raidus_2", 30);
		m_grind_config->wheel_raidus_multi[2] = m_ini_grind->GetDoubleValueOrDefault("grind", "wheel_raidus_3", 30);
		m_grind_config->wheel_raidus_multi[3] = m_ini_grind->GetDoubleValueOrDefault("grind", "wheel_raidus_4", 30);
		m_grind_config->wheel_raidus_multi[4] = m_ini_grind->GetDoubleValueOrDefault("grind", "wheel_raidus_5", 30);
		m_grind_config->wheel_raidus_multi[5] = m_ini_grind->GetDoubleValueOrDefault("grind", "wheel_raidus_6", 30);
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "��ȡĥ�������ļ��ɹ���\n");
	}
	else {
		if(m_ini_grind->CreateFile(GRIND_CONFIG_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "����Ĭ��ĥ�������ļ�ʧ�ܣ�\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

		//����Ĭ��ֵ
		m_grind_config->wheel_radius = 80;
		m_grind_config->safe_pos_x = 30;
		m_grind_config->safe_pos_c = 0;
		m_grind_config->shock_z_max_dis = 4;
		m_grind_config->shock_x_compen_ratio = -2.5;
		m_grind_config->shock_speed = 120;
		m_grind_config->shock_manual_ratio_ctrl = 0;
		m_grind_config->shock_init_dir = 1;
		m_grind_config->grind_dir = 0;
		m_grind_config->corner_vec_speed = 1800;
		m_grind_config->grind_smooth_fun = 0;
		m_grind_config->grind_smooth_time = 2;
		for(i = 0; i < 6; i++)
			m_grind_config->wheel_raidus_multi[i] = 30;

		//����Ĭ��ini����
		IniSection *ns = m_ini_grind->AddSecttion("grind");
		if(ns == nullptr){
			err_code = ERR_SC_INIT;
			goto END;
		}
		m_ini_grind->AddKeyValuePair(string("wheel_radius"), string("80"), ns);
		m_ini_grind->AddKeyValuePair(string("safe_pos_x"), string("30"), ns);
		m_ini_grind->AddKeyValuePair(string("safe_pos_c"), string("0"), ns);
		m_ini_grind->AddKeyValuePair(string("shock_z_max_dis"), string("4"), ns);
		m_ini_grind->AddKeyValuePair(string("shock_x_compen_ratio"), string("-2.5"), ns);
		m_ini_grind->AddKeyValuePair(string("shock_speed"), string("120"), ns);
		m_ini_grind->AddKeyValuePair(string("shock_manual_ratio_ctrl"), string("0"), ns);
		m_ini_grind->AddKeyValuePair(string("shock_init_dir"), string("1"), ns);
		m_ini_grind->AddKeyValuePair(string("grind_dir"), string("0"), ns);
		m_ini_grind->AddKeyValuePair(string("corner_vec_speed"), string("1800"), ns);
		m_ini_grind->AddKeyValuePair(string("grind_smooth_fun"), string("0"), ns);
		m_ini_grind->AddKeyValuePair(string("grind_smooth_time"), string("2"), ns);
		m_ini_grind->AddKeyValuePair(string("wheel_raidus_1"), string("30"), ns);
		m_ini_grind->AddKeyValuePair(string("wheel_raidus_2"), string("30"), ns);
		m_ini_grind->AddKeyValuePair(string("wheel_raidus_3"), string("30"), ns);
		m_ini_grind->AddKeyValuePair(string("wheel_raidus_4"), string("30"), ns);
		m_ini_grind->AddKeyValuePair(string("wheel_raidus_5"), string("30"), ns);
		m_ini_grind->AddKeyValuePair(string("wheel_raidus_6"), string("30"), ns);

		m_ini_grind->Save();

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "����Ĭ��ĥ�������ļ��ɹ���\n");


	}
	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}
#endif

/**
 * @brief ����������ļ�
 * @param type : ��������
 * @return true--�ɹ�   false--ʧ��
 */
bool ParmManager::SaveParm(ParmType type){

	switch(type){
	case SYS_CONFIG:
		this->m_ini_system->Save();
		break;
	case CHN_CONFIG:
		this->m_ini_chn->Save();
		break;
	case AXIS_CONFIG:
		this->m_ini_axis->Save();
		break;
	case TOOL_OFFSET_CONFIG:
		this->m_ini_tool->Save();
		break;
	case TOOL_POT_CONFIG:
		this->m_ini_tool->Save();
		break;
	case COORD_CONFIG:
		this->m_ini_coord->Save();
		break;
	case EX_COORD_CONFIG:
		this->m_ini_ex_coord->Save();
		break;
	case CHN_STATE_SCENE:
		this->m_ini_chn_scene->Save();
		break;
	case PITCH_COMP_DATA:
		this->m_ini_pc_table->Save();
		break;
#ifdef USES_FIVE_AXIS_FUNC
	case FIVE_AXIS_CONFIG:
		this->m_ini_five_axis->Save();
		break;
#endif
#ifdef  USES_GRIND_MACHINE
	case GRIND_CONFIG:
		this->m_ini_grind->Save();
		break;
#endif
	default:
		break;
	}

	return true;
}

/**
 * @brief  �������в������ļ�
 * @return true--�ɹ�   false--ʧ��
 */
bool ParmManager::SaveAllParm(){
	bool res = true;
	res &= SaveParm(SYS_CONFIG);

	res &= SaveParm(CHN_CONFIG);

	res &= SaveParm(AXIS_CONFIG);

	res &= SaveParm(TOOL_OFFSET_CONFIG);

	res &= SaveParm(COORD_CONFIG);

	res &= SaveParm(EX_COORD_CONFIG);

	res &= SaveParm(PITCH_COMP_DATA);

#ifdef USES_FIVE_AXIS_FUNC
	res &= SaveParm(FIVE_AXIS_CONFIG);
#endif

#ifdef USES_GRIND_MACHINE
	res &= SaveParm(GRIND_CONFIG);
#endif
	return res;
}

/**
 * @brief  ��ȡָ��ͨ������
 * @param index �� ͨ��������
 * @return �ɹ����ض�Ӧ����ָ�룬ʧ�ܷ���nullptr
 */
SCChannelConfig *ParmManager::GetChannelConfig(int index){
//	if(m_sc_channel_config == nullptr || index >= m_sc_system_config->chn_count)
	if(m_sc_channel_config == nullptr ||
		index >= m_sc_system_config->max_chn_count)   //�����޸ģ�һ������
		return nullptr;
	return &m_sc_channel_config[index];
}

/**
 * @brief ��ȡ�������ͨ������
 * @param index �� ͨ��������
 * @return �ɹ����ض�Ӧ����ָ�룬ʧ�ܷ���nullptr
 */
ChnProcParamGroup *ParmManager::GetChnProcessParam(int index){
//	if(this->m_p_chn_process_param == nullptr || index >= m_sc_system_config->chn_count)
	if(this->m_p_chn_process_param == nullptr ||
		index >= m_sc_system_config->max_chn_count)   //�����޸ģ�һ������
		return nullptr;
	return &m_p_chn_process_param[index];
}

/**
 * @brief ���µ��߰뾶ĥ��ֵ
 * @param chn_index �� ͨ���ţ���0��ʼ
 * @param index : ��0��ʼ
 * @param value �����߰뾶ĥ��
 */
void ParmManager::UpdateToolRadiusWear(uint16_t chn_index, uint8_t index, const double &value){
	char sname[32];	//section name
	char kname[64];	//key name


	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	this->m_sc_tool_config[chn_index].radius_wear[index] = value;

	sprintf(sname, "channel_%hu", chn_index);
	sprintf(kname, "radius_wear_%hhu", index);

	m_ini_tool->SetDoubleValue(sname, kname, value);

	this->m_ini_tool->Save();
}

/**
 * @brief ���µ��߰뾶����ƫִ
 * @param chn_index �� ͨ���ţ���0��ʼ
 * @param index : ��0��ʼ
 * @param value : ���߼��ΰ뾶
 */
void ParmManager::UpdateToolRadiusGeo(uint16_t chn_index, uint8_t index, const double &value){
	char sname[32];	//section name
	char kname[64];	//key name


	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	this->m_sc_tool_config[chn_index].radius_compensation[index] = value;

	sprintf(sname, "channel_%hu", chn_index);
	sprintf(kname, "radius_comp_%hhu", index);

	m_ini_tool->SetDoubleValue(sname, kname, value);

	this->m_ini_tool->Save();
}

/**
 * @brief ���µ���ĥ��ֵ
 * @param chn_index �� ͨ���ţ���0��ʼ
 * @param index : ��0��ʼ
 * @param value �� ����ĥ��ֵ����λ��mm
 */
void ParmManager::UpdateToolWear(uint16_t chn_index, uint8_t index, const double &value){
	char sname[32];	//section name
	char kname[64];	//key name


	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	this->m_sc_tool_config[chn_index].geometry_wear[index] = value;

	sprintf(sname, "channel_%hu", chn_index);
	sprintf(kname, "geo_wear_%hhu", index);

	m_ini_tool->SetDoubleValue(sname, kname, value);

	this->m_ini_tool->Save();
}

/**
 * @brief ���µ��߳��Ȳ���ֵ
 * @param chn_index �� ͨ���ţ���0��ʼ
 * @param index �� 0��ʾ���»�׼���� ����ֵΪ�ǻ�׼��
 * @param value ����������λ��mm
 */
void ParmManager::UpdateToolMeasure(uint16_t chn_index, uint8_t index, const double &value){
	char sname[32];	//section name
	char kname[64];	//key name


	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));


	sprintf(sname, "channel_%hu", chn_index);

#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
	if(index == 0){
		this->m_sc_tool_config[chn_index].geometry_comp_basic[2] = value;
		sprintf(kname, "geo_comp_basic_z");
	}else{
#else
	if(index > 0){
#endif
		this->m_sc_tool_config[chn_index].geometry_compensation[index-1][2] = value;
		sprintf(kname, "geo_comp_z_%hhu", index-1);
	}

	m_ini_tool->SetDoubleValue(sname, kname, value);

	this->m_ini_tool->Save();
}

/**
 * @brief ���µ���ƫ����Ϣ
 * @param chn_index �� ͨ���ţ���0��ʼ
 * @param index �� ���߲����ţ���0��ʼ�� H1ʱindexΪ0
 * @param active : true--����    false--��������¶���
 * @param cfg
 */
void ParmManager::UpdateToolOffsetConfig(uint16_t chn_index, uint8_t index, HmiToolOffsetConfig &cfg, bool active){
	char sname[32];	//section name
	char kname[64];	//key name
//	char value[32]; //value

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));
//	memset(value, 0x00, sizeof(value));

	if(active){
		this->m_sc_tool_config[chn_index].geometry_compensation[index][0] = cfg.geometry_compensation[0];
		this->m_sc_tool_config[chn_index].geometry_compensation[index][1] = cfg.geometry_compensation[1];
		this->m_sc_tool_config[chn_index].geometry_compensation[index][2] = cfg.geometry_compensation[2];
		this->m_sc_tool_config[chn_index].geometry_wear[index] = cfg.geometry_wear;
		this->m_sc_tool_config[chn_index].radius_compensation[index] = cfg.radius_compensation;
		this->m_sc_tool_config[chn_index].radius_wear[index] = cfg.radius_wear;
	}else{//�������Ч����
		ToolOffsetUpdate data;
		data.chn_index = chn_index;
		data.tool_offset_index = index;
		memcpy(&data.value, &cfg, sizeof(HmiToolOffsetConfig));
		this->m_list_tool_offset->Append(data);

	}


	sprintf(sname, "channel_%hu", chn_index);
	sprintf(kname, "geo_comp_x_%hhu", index);
	m_ini_tool->SetDoubleValue(sname, kname, cfg.geometry_compensation[0]);
	sprintf(kname, "geo_comp_y_%hhu", index);
	m_ini_tool->SetDoubleValue(sname, kname, cfg.geometry_compensation[1]);
	sprintf(kname, "geo_comp_z_%hhu", index);
	m_ini_tool->SetDoubleValue(sname, kname, cfg.geometry_compensation[2]);

	sprintf(kname, "geo_wear_%hhu", index);
	m_ini_tool->SetDoubleValue(sname, kname, cfg.geometry_wear);
	sprintf(kname, "radius_comp_%hhu", index);
	m_ini_tool->SetDoubleValue(sname, kname, cfg.radius_compensation);

	sprintf(kname, "radius_wear_%hhu", index);
	m_ini_tool->SetDoubleValue(sname, kname, cfg.radius_wear);

	this->m_ini_tool->Save();
}

/**
 * @brief ���µ�λ��Ϣ
 * @param chn_index �� ͨ��������
 * @param cfg : ������
 */
void ParmManager::UpdateToolPotConfig(uint16_t chn_index, SCToolPotConfig &cfg){
	char sname[32];	//section name
	char kname[64];	//key name
	char value[32]; //value

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));
	memset(value, 0x00, sizeof(value));

	memcpy(&m_sc_tool_pot_config[chn_index], &cfg, sizeof(SCToolPotConfig));		//���µ�ǰ����

	sprintf(sname, "channel_%hu", chn_index);

	for(int i = 0; i < kMaxToolCount; i++){
		sprintf(kname, "tool_type_%d", i);
		m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].tool_type[i]);

		sprintf(kname, "huge_tool_flag_%d", i);
		m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].huge_tool_flag[i]);

		sprintf(kname, "tool_pot_index_%d", i);
		m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].tool_pot_index[i]);

		sprintf(kname, "tool_life_max_%d", i);
		m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].tool_life_max[i]);

		sprintf(kname, "tool_life_cur_%d", i);
		m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].tool_life_cur[i]);

        sprintf(kname, "tool_threshold_%d", i);
        m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].tool_threshold[i]);

        sprintf(kname, "tool_life_type_%d", i);
        m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].tool_life_type[i]);

	}

	this->m_ini_tool->Save();
}

/**
 * @brief ���µ�����Ϣ
 * @param chn_index �� ͨ��������
 */
void ParmManager::UpdateToolPotConfig(uint16_t chn_index, bool save){
	char sname[32];	//section name
	char kname[64];	//key name
	char value[32]; //value

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));
	memset(value, 0x00, sizeof(value));

	sprintf(sname, "channel_%hu", chn_index);

	for(int i = 0; i < kMaxToolCount; i++){
		sprintf(kname, "tool_type_%d", i);
		m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].tool_type[i]);

		sprintf(kname, "huge_tool_flag_%d", i);
		m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].huge_tool_flag[i]);

		sprintf(kname, "tool_pot_index_%d", i);
		m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].tool_pot_index[i]);

		sprintf(kname, "tool_life_max_%d", i);
		m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].tool_life_max[i]);

		sprintf(kname, "tool_life_cur_%d", i);
		m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].tool_life_cur[i]);

        sprintf(kname, "tool_threshold_%d", i);
        m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].tool_threshold[i]);

        sprintf(kname, "tool_life_type_%d", i);
        m_ini_tool->SetIntValue(sname, kname, m_sc_tool_pot_config[chn_index].tool_life_type[i]);
	}

	if(save)
		this->m_ini_tool->Save();
}

/**
 * @brief ���¹�������ϵ
 * @param chn_index �� ͨ��������
 * @param index : ����ϵ����
 * @param cfg �� ������ϵ
 * @param active : true--����    false--��������¶���
 */
void ParmManager::UpdateCoordConfig(uint16_t chn_index, uint8_t index, HmiCoordConfig &cfg, bool active){
	char sname[32];	//section name
	char kname[64];	//key name
	char value[32]; //value

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));
	memset(value, 0x00, sizeof(value));
	printf("update coord config chn=%hhu, coord=%hhu, active=%hhu\n", chn_index, index, active);

	if(active){
		memcpy(&m_sc_coord_config[chn_index*kWorkCoordCount+index], &cfg, sizeof(SCCoordConfig));		//���µ�ǰ����

	}
	else{//�������Ч����
		CoordUpdate data;
		data.chn_index = chn_index;
		data.coord_index = index;
		memcpy(&data.value, &cfg, sizeof(SCCoordConfig));
		this->m_list_coord->Append(data);

	}

	sprintf(sname, "channel_%hu", chn_index);

	for(int i = 0; i < kMaxAxisChn; i++){
		sprintf(kname, "offset_%hhu_%d", index, i);
		m_ini_coord->SetDoubleValue(sname, kname, cfg.offset[i]);
	}

	this->m_ini_coord->Save();
}

/**
 * @brief ������չ��������ϵ
 * @param chn_index �� ͨ��������
 * @param index : ��չ����ϵ����
 * @param cfg �� ������ϵ
 */
void ParmManager::UpdateExCoordConfig(uint16_t chn_index, uint8_t index, HmiCoordConfig &cfg, bool active){
	char sname[32];	//section name
	char kname[64];	//key name
	char value[32]; //value

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));
	memset(value, 0x00, sizeof(value));

	if(active){
		memcpy(&m_sc_ex_coord_config[chn_index][index], &cfg, sizeof(SCCoordConfig));			//���µ�ǰ����
	}
	else{//�������Ч����
		CoordUpdate data;
		data.chn_index = chn_index;
		data.coord_index = index+7;  //����������������ϵ
		memcpy(&data.value, &cfg, sizeof(SCCoordConfig));
		this->m_list_coord->Append(data);
	}

	sprintf(sname, "channel_%hu", chn_index);

	for(int i = 0; i < kMaxAxisChn; i++){
		sprintf(kname, "offset_%hhu_%d", index, i);
		m_ini_ex_coord->SetDoubleValue(sname, kname, cfg.offset[i]);
	}

	this->m_ini_ex_coord->Save();
}

/**
 * @brief ����ָ��ͨ������Ч�Ĺ�������ϵ����
 * @param chn_index : ָ��ͨ�������� 0xFF��ʾ����ͨ��
 * @return true--����������   false -- û�м�������
 */
bool ParmManager::ActiveCoordParam(uint8_t chn_index){
	int count = 0;
	CoordUpdate *coord = nullptr;
	ListNode<CoordUpdate> *node = m_list_coord->HeadNode();
	ListNode<CoordUpdate> *node_next = nullptr;

	while(node != nullptr){
		coord = &node->data;

		if(coord->chn_index == chn_index || chn_index == 0xFF){//���µ�ǰ����
			if(coord->coord_index < kWorkCoordCount){  //��������ϵ
				memcpy(&m_sc_coord_config[coord->chn_index*kWorkCoordCount+coord->coord_index], &coord->value, sizeof(SCCoordConfig));

			}else{//��չ��������ϵ
				memcpy(&m_sc_ex_coord_config[coord->chn_index][coord->coord_index-kWorkCoordCount], &coord->value, sizeof(SCCoordConfig));
			}
			node_next = node->next;
			m_list_coord->Delete(node);
			node = node_next;
			count++;
		}else
			node = node->next;
	}


	if(count > 0)
		return true;
	return false;
}

/**
 * @brief �������Ч�ĵ���ƫ��
 * @param chn_index : ָ��ͨ�������� 0xFF��ʾ����ͨ��
 * @return true--����������   false -- û�м�������
 */
bool ParmManager::ActiveToolOffsetParam(uint8_t chn_index){
	int count = 0;
	ToolOffsetUpdate *offset = nullptr;
	ListNode<ToolOffsetUpdate> *node = m_list_tool_offset->HeadNode();
	ListNode<ToolOffsetUpdate> *node_next = nullptr;

	while(node != nullptr){
		offset = &node->data;

		if(offset->chn_index == chn_index || chn_index == 0xFF){//���µ�ǰ����
			if(offset->tool_offset_index < kMaxToolCount){
				this->m_sc_tool_config[chn_index].geometry_compensation[offset->tool_offset_index][0] = offset->value.geometry_compensation[0];
				this->m_sc_tool_config[chn_index].geometry_compensation[offset->tool_offset_index][1] = offset->value.geometry_compensation[1];
				this->m_sc_tool_config[chn_index].geometry_compensation[offset->tool_offset_index][2] = offset->value.geometry_compensation[2];
				this->m_sc_tool_config[chn_index].geometry_wear[offset->tool_offset_index] = offset->value.geometry_wear;
				this->m_sc_tool_config[chn_index].radius_compensation[offset->tool_offset_index] = offset->value.radius_compensation;
				this->m_sc_tool_config[chn_index].radius_wear[offset->tool_offset_index] = offset->value.radius_wear;

			}
			node_next = node->next;
			m_list_tool_offset->Delete(node);
			node = node_next;
			count++;
		}else
			node = node->next;
	}


	if(count > 0)
		return true;
	return false;
}

/**
 * @brief �����ݲ�����
 * @param axis_index : ��ţ�������ţ���0��ʼ
 * @param dir_flag : �Ƿ������ݲ��� true--����    false--����
 * @param offset ������ƫ�ƣ���1��ʼ
 * @param count : ���ݸ���
 * @param data : �ݲ�����
 * @return
 */
bool ParmManager::UpdatePcData(uint8_t axis_index, bool dir_flag, uint16_t offset, uint16_t count, double *data){

	if(data == nullptr || axis_index >= m_sc_system_config->axis_count){
		printf("ParmManager::UpdatePcData return false, axis_index = %hhu\n", axis_index);
		return false;
	}

	uint16_t total_count = m_sc_axis_config[axis_index].pc_count;
	if(m_sc_axis_config[axis_index].pc_type == 1){//˫���ݲ�
		total_count *= 2;    //�ܼƵ���*2
	}

	if(offset+count-1 > total_count){
		printf("ParmManager::UpdatePcData return false, offset = %hu, count=%hu\n", offset, count);
		return false;
	}

	char sname[32];	//section name
	char kname[64];	//key name
	char dir_str[20];

	uint16_t offset_index = offset;
	memset(sname, 0x00, 32);
	sprintf(sname, "axis_%d", axis_index+1);    //��1��ʼ
	memset(dir_str, 0x00, 20);

	int pc_count = m_sc_axis_config[axis_index].pc_count;

	for(int i = 0; i < count; i++ ){

		m_sc_pc_table->pc_table[axis_index][offset_index+i-1] = data[i];
		memset(kname, 0x00, sizeof(kname));

		if((offset_index+i-1) < pc_count){
			sprintf(dir_str, "pc_pos_");//�����ݲ�
			sprintf(kname, "%s%d", dir_str, offset+i);

		}else{
			sprintf(dir_str, "pc_neg_");//�����ݲ�
			sprintf(kname, "%s%d", dir_str, offset+i-pc_count);
		}

		m_ini_pc_table->SetDoubleValue(sname, kname, data[i]);
	}

	m_ini_pc_table->Save();

	return true;
}

/**
 * @brief ����IO��ӳ������
 * @param info : �ض�������
 * @return  true--�ɹ�   false--ʧ��
 */
bool ParmManager::UpdateIoRemapInfo(IoRemapInfo &info){
	char sname[32];	//section name
	char kname[32];	//key name
	char value[32];  //key value

	IniSection *ns=nullptr;

	char reg_sec[2] = {'X','Y'};

	memset(sname, 0x00, 32);
	sprintf(sname, "%c%hu%hhu", reg_sec[info.iotype], info.addr, info.bit);


	bool has_flag = this->m_ini_io_remap->HasSection(sname);
	if(has_flag){//�Ѿ�����
		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "addrmap");
		m_ini_io_remap->SetIntValue(sname, kname, info.addrmap);

		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "newaddr");
		m_ini_io_remap->SetIntValue(sname, kname, info.newaddr);

		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "newbit");
		m_ini_io_remap->SetIntValue(sname, kname, info.newbit);

		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "valtype");
		m_ini_io_remap->SetIntValue(sname, kname, info.valtype);

	}else{//�����ڣ���Ҫ���
		ns = m_ini_io_remap->AddSecttion(sname);
		if(ns == nullptr){
			return false;
		}

		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "iotype");
		memset(value, 0x00, sizeof(value));
		sprintf(value, "%hhu", info.iotype);
		m_ini_io_remap->AddKeyValuePair(kname, value, ns);

		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "addr");
		memset(value, 0x00, sizeof(value));
		sprintf(value, "%hu", info.addr);
		m_ini_io_remap->AddKeyValuePair(kname, value, ns);

		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "bit");
		memset(value, 0x00, sizeof(value));
		sprintf(value, "%hhu", info.bit);
		m_ini_io_remap->AddKeyValuePair(kname, value, ns);

		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "addrmap");
		memset(value, 0x00, sizeof(value));
		sprintf(value, "%hhu", info.addrmap);
		m_ini_io_remap->AddKeyValuePair(kname, value, ns);

		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "newaddr");
		memset(value, 0x00, sizeof(value));
		sprintf(value, "%hu", info.newaddr);
		m_ini_io_remap->AddKeyValuePair(kname, value, ns);

		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "newbit");
		memset(value, 0x00, sizeof(value));
		sprintf(value, "%hhu", info.newbit);
		m_ini_io_remap->AddKeyValuePair(kname, value, ns);

		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "valtype");
		memset(value, 0x00, sizeof(value));
		sprintf(value, "%hhu", info.valtype);
		m_ini_io_remap->AddKeyValuePair(kname, value, ns);
	}

	this->m_ini_io_remap->Save();
    return true;
}

/**
 * @brief ��������ͨ��ӳ��
 * @param HandWheelMapInfoVec : ����ͨ��ӳ���ϵ
 * @param bRestart : �Ƿ�������Ч
 * @return true--�ɹ�    false--ʧ��
 */
bool ParmManager::SyncHandWheelInfo(const HandWheelMapInfoVec &infoVec, bool bRestart)
{
    char sectionName[30];
    if (m_p_handwheel_param == infoVec)
    {//��ͼ���ýṹû�иı䣬����ֱ�Ӹ��²���
        for(unsigned int i = 0; i < infoVec.size(); ++i)
        {
            memset(sectionName, 0x00, sizeof(sectionName));
            sprintf(sectionName, "handwheel%d", infoVec[i].devNum);
            if (!bRestart)
                m_p_handwheel_param[i].channelMap = infoVec[i].channelMap;
            m_ini_handwheel_map->SetIntValue(sectionName, "channelMap", infoVec[i].channelMap);
        }
        m_ini_handwheel_map->Save();
    }
    else
    {//��ͼ���ýṹ�ı䣬��Ҫ����д��ini
        remove(HANDWHEEL_CONFIG_FILE);
        sync();
        m_ini_handwheel_map->Load(HANDWHEEL_CONFIG_FILE);
        for(unsigned int i = 0; i < infoVec.size(); ++i)
        {
            memset(sectionName, 0x00, sizeof(sectionName));
            sprintf(sectionName, "handwheel%d", i+1);

            m_ini_handwheel_map->SetIntValue(sectionName, "devNum", infoVec[i].devNum);
            m_ini_handwheel_map->SetIntValue(sectionName, "wheelId", infoVec[i].wheelID);
            m_ini_handwheel_map->SetIntValue(sectionName, "channelMap", infoVec[i].channelMap);
            m_ini_handwheel_map->SetStringValue(sectionName, "devName", infoVec[i].devName);
        }
        if (!bRestart)
            m_p_handwheel_param = infoVec;
        m_ini_handwheel_map->Save();
    }
    return true;
}

/**
 * @brief ���²���
 * @param data : ����������
 * @param active_type : ��������
 */
bool ParmManager::UpdateParameter(ParamUpdate *data, uint8_t active_type){
	bool res = true;
	if(data == nullptr)
		return true;
	printf("update paramter, chn = %hhu, para_type:%hhu, type:%hhu, id:%d\n", data->chn_index, data->param_type, active_type, data->param_no);

	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	uint8_t group_index = 0;
	if(data->chn_index < this->m_sc_system_config->chn_count)
		group_index = chn_engine->GetChnControl(data->chn_index)->GetCurProcParamIndex();

//	printf("chn %hhu cur group index : %hhu\n", data->chn_index, group_index);
	//����������д��INI�ļ�
	switch(data->param_type){
	case SYS_CONFIG:
		res = this->UpdateSystemParam(data->param_no, data->value);
		break;
	case CHN_CONFIG:
		res = this->UpdateChnParam(data->chn_index, data->param_no, data->value);
		this->UpdateChnProcParam(data->chn_index, group_index, data->param_no, data->value);
		break;
	case AXIS_CONFIG:
		res = this->UpdateAxisParam(data->axis_index, data->param_no, data->value);
		this->UpdateAxisProcParam(data->axis_index, group_index, data->param_no, data->value);
		break;

#ifdef USES_FIVE_AXIS_FUNC
	case FIVE_AXIS_CONFIG:
		res = this->UpdateFiveAxisParam(data->chn_index, data->param_no, data->value);
		break;
#endif

#ifdef USES_GRIND_MACHINE
	case GRIND_CONFIG:
		this->UpdateGrindParam(data->param_no, data->value);
		break;
#endif
	default:
		res = false;
		break;
	}
	if(res)
		this->ActiveParam(data, active_type);

//	printf("exit UpdateParameter\n");
	return res;
}

/**
 * @brief ���¹�����ز���
 * @param data : ����������
 * @param active_type : ��������
 * @return true--�ɹ�   false--ʧ��
 */
bool ParmManager::UpdateProcParam(ProcParamUpdate *data, uint8_t active_type){
	bool res = true;
	if(data == nullptr)
		return true;
	printf("update process paramter, active_type:%hhu, id:%d, group_idx:%hhu\n", active_type, data->param_data.param_no, data->group_index);

	//����������д��INI�ļ�
	switch(data->param_data.param_type){
	case CHN_CONFIG:
		res = this->UpdateChnProcParam(data->param_data.chn_index, data->group_index, data->param_data.param_no, data->param_data.value);
		break;
	case AXIS_CONFIG:
		res = this->UpdateAxisProcParam(data->param_data.axis_index, data->group_index, data->param_data.param_no, data->param_data.value);
		break;
	default:
		res = false;
		break;
	}

	//����ǵ�ǰ���ղ������򼤻�
	if(res)
		this->ActiveProcParam(data, active_type);

	return res;
}

/**
 * @brief ����ϵͳ������ֻ��д��INI�ļ�����������ϵͳ����
 * @param param_no �� ������
 * @param value	�� ����ֵ
 */
bool ParmManager::UpdateSystemParam(uint32_t param_no, ParamValue &value){
	bool res = true;
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	sprintf(sname, "system");
	switch(param_no){
	case 3: 	//ͨ����
		if(value.value_uint8 > kMaxChnCount){
			return false;
		}
		sprintf(kname, "chn_count");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 5:		//��������
		if(value.value_uint8 > kMaxAxisNum){
			return false;
		}
		sprintf(kname, "axis_count");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 6:		//����ͨѶ����
		sprintf(kname, "bus_cycle");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 10:  //���ֱ����ʽ
		sprintf(kname, "hw_code_type");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 11:  //���ַ�������
		sprintf(kname, "hw_rev_trace");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 20:	//ϵͳ����ٶ�
		sprintf(kname, "max_sys_vel");
		m_ini_system->SetDoubleValue(sname, kname, value.value_double);
		printf("update max_sys_vel, %f\n", value.value_double);
		break;
	case 24:	//�ս������ٶ�
		sprintf(kname, "max_corner_acc");
		m_ini_system->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 25:	//���������ٶ�
		sprintf(kname, "max_cent_acc");
		m_ini_system->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 30:	//�������±�
		sprintf(kname, "axis_name_ex");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 31:	//�زο���ʱ���ʹ̶�
		sprintf(kname, "fix_ratio_find_ref");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 32:	//�ݾಹ����ʽ
		//sprintf(kname, "pc_type");
		//m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		//this->m_sc_system_config->pc_type = value.value_uint8;
		break;
	case 33:	//DA��������
		sprintf(kname, "da_ocp");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 34:	//�ϵ㱣�����ǰ�к�
		sprintf(kname, "save_lineno_poweroff");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 35:	//�ֶ��زο���ģʽ
		sprintf(kname, "manual_ret_ref_mode");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 60:	//��ͨIO�˲�ʱ��
		sprintf(kname, "io_filter_time");
		m_ini_system->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 61:	//����IO�˲�ʱ��
		sprintf(kname, "fast_io_filter_time");
		m_ini_system->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 90:	//������ʱʱ��
		sprintf(kname, "backlight_delay_time");
		m_ini_system->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 91:	//����ʱ������
		sprintf(kname, "beep_time");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 92:	//�洢�ռ�澯����
		sprintf(kname, "free_space_limit");
		m_ini_system->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 93:	//����澯�¶�
		sprintf(kname, "alarm_temperature");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 99:	//������Ϣ����
		sprintf(kname, "trace_level");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 100:	//����ģʽ
		sprintf(kname, "debug_mode");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	default:	//Ĭ��
		g_ptr_trace->PrintLog(LOG_ALARM, "ϵͳ�������£������ŷǷ���%d", param_no);
		res = false;
		return res;
	}

	m_ini_system->Save();
	return res;
}

#ifdef USES_GRIND_MACHINE
/**
 * @brief ����ĥ������
 * @param param_no �� ������
 * @param value �� ����ֵ
 * @return
 */
bool ParmManager::UpdateGrindParam(uint32_t param_no, ParamValue &value){
	bool res = true;
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	sprintf(sname, "grind");
	switch(param_no){
	case 10100: 	//ɰ�ְ뾶
		sprintf(kname, "wheel_radius");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_radius = value.value_double;
		break;
	case 10101:		//X�ᰲȫλ������
		sprintf(kname, "safe_pos_x");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->safe_pos_x = value.value_double;
		break;
	case 10102:		//C�ᰲȫλ������
		sprintf(kname, "safe_pos_c");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->safe_pos_c = value.value_double;
		break;
	case 10103:	//ɰ����ĥ��Z����󳤶�
		sprintf(kname, "shock_z_max_dis");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->shock_z_max_dis = value.value_double;
		break;
	case 10104:	//ɰ����ĥ��X�Ჹ������
		sprintf(kname, "shock_x_compen_ratio");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->shock_x_compen_ratio = value.value_double;
		break;
	case 10105:	//ɰ����ĥ���ٶ�
		sprintf(kname, "shock_speed");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->shock_speed = value.value_double;
		break;
	case 10106:	//���ٶ����ֶ����ʿ���
		sprintf(kname, "shock_manual_ratio_ctrl");
		m_ini_grind->SetIntValue(sname, kname, value.value_uint8);
		m_grind_config->shock_manual_ratio_ctrl = value.value_uint8;
		break;
	case 10107:	//ɰ����ĥ����ʼ����
		sprintf(kname, "shock_init_dir");
		m_ini_grind->SetIntValue(sname, kname, value.value_uint8);
		m_grind_config->shock_init_dir = value.value_uint8;
		break;
	case 10108:	//�����ӹ�����
		sprintf(kname, "grind_dir");
		m_ini_grind->SetIntValue(sname, kname, value.value_uint8);
		m_grind_config->grind_dir = value.value_uint8;
		break;
	case 10109:	//ת���������ٶ�
		sprintf(kname, "corner_vec_speed");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->corner_vec_speed = value.value_double;
		break;
	case 10110:	//ĥ��ƽ������
		sprintf(kname, "grind_smooth_fun");
		m_ini_grind->SetIntValue(sname, kname, value.value_uint8);
		m_grind_config->grind_smooth_fun = value.value_uint8;
		break;
	case 10111:	//ĥ��ƽ��ʱ�䳣��
		sprintf(kname, "grind_smooth_time");
		m_ini_grind->SetIntValue(sname, kname, value.value_uint16);
		m_grind_config->grind_smooth_time = value.value_uint16;
		break;
	case 10112:	//ɰ�ְ뾶1
		sprintf(kname, "wheel_raidus_1");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_raidus_multi[0] = value.value_double;
		break;
	case 10113:	//ɰ�ְ뾶2
		sprintf(kname, "wheel_raidus_2");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_raidus_multi[1] = value.value_double;
		break;
	case 10114:	//ɰ�ְ뾶3
		sprintf(kname, "wheel_raidus_3");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_raidus_multi[2] = value.value_double;
		break;
	case 10115:	//ɰ�ְ뾶4
		sprintf(kname, "wheel_raidus_4");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_raidus_multi[3] = value.value_double;
		break;
	case 10116:	//ɰ�ְ뾶5
		sprintf(kname, "wheel_raidus_5");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_raidus_multi[4] = value.value_double;
		break;
	case 10117:	//ɰ�ְ뾶6
		sprintf(kname, "wheel_raidus_6");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_raidus_multi[5] = value.value_double;
		break;
	default:	//Ĭ��
		g_ptr_trace->PrintLog(LOG_ALARM, "ĥ���������£������ŷǷ���%d", param_no);
		res = false;
		return res;
	}

	m_ini_grind->Save();
	this->m_b_update_grind = true;
	return res;
}

#endif

/**
 * @brief ����ͨ��������ֻ��д��INI�ļ�����������ͨ������
 * @param chn_index �� ͨ����
 * @param param_no �� ������
 * @param value �� ����ֵ
 */
bool ParmManager::UpdateChnParam(uint8_t chn_index, uint32_t param_no, ParamValue &value){
	bool res = true;
//	if(chn_index >= this->m_sc_system_config->chn_count){
	if(chn_index >= this->m_sc_system_config->max_chn_count){  //֧�ֲ����޸ģ�һ������
		g_ptr_trace->PrintLog(LOG_ALARM, "ͨ���������£�ͨ���ŷǷ���%hhu", chn_index);
		return false;
	}
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	sprintf(sname, "channel_%hhu", chn_index);
	switch(param_no){
	case 102:	//������
		sprintf(kname, "chn_axis_count");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 103:   //ͨ��������ʽ��
		sprintf(kname, "chn_group_index");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 104:	//������X
//		sprintf(kname, "chn_axis_x");
//		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
//		break;
	case 105:	//������Y
//		sprintf(kname, "chn_axis_y");
//		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
//		break;
	case 106:	//������Z
//		sprintf(kname, "chn_axis_z");
//		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
//		break;
	case 107:	//��4
	case 108:	//��5
	case 109:	//��6
	case 110:	//��7
	case 111:	//��8
		sprintf(kname, "chn_axis_%d", param_no-103);
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 136:  //������
	case 137:
	case 138:
	case 139:
	case 140:
	case 141:
	case 142:
	case 143:
		sprintf(kname, "chn_axis_name_%d", param_no-135);
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 168:  //��������չ�±�
	case 169:
	case 170:
	case 171:
	case 172:
	case 173:
	case 174:
	case 175:
		sprintf(kname, "chn_axis_name_ex_%d", param_no-167);
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 200:	//�岹ģʽ
		sprintf(kname, "intep_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 201:	//�岹����
		sprintf(kname, "intep_cycle");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 202:	//G00����ģʽ
		sprintf(kname, "rapid_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 203:	//�����ٶȹ滮��ʽ
		sprintf(kname, "cut_plan_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 204:	//��λ�ٶȹ滮��ʽ
		sprintf(kname, "rapid_plan_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 210:	//ͨ����������ٶ�
		sprintf(kname, "chn_max_vel");
		m_ini_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 211:	//ͨ�������ٶ�
		sprintf(kname, "chn_max_acc");
		m_ini_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 212:	//ͨ�������ٶ�
		sprintf(kname, "chn_max_dec");
		m_ini_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 213:	//
		break;
	case 214:	//�սǼ��ٶ�
		sprintf(kname, "chn_max_corner_acc");
		m_ini_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 215:	//������ļ��ٶ�
		sprintf(kname, "chn_max_arc_acc");
		m_ini_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 216:	//��������S�͹滮ʱ�䳣��
		sprintf(kname, "chn_s_cut_filter_time");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint16);
		break;

	case 220:	//�ӹ�����
		sprintf(kname, "chn_precision");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 221:	//ǰհ����
		sprintf(kname, "chn_look_ahead");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 222:	//�ӹ��ٶȵ����ȼ�
		sprintf(kname, "chn_feed_limit_level");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 223:	//�ս�׼ͣ����
		sprintf(kname, "corner_stop");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 229:  //�ս�׼ͣ���޽Ƕ�
		sprintf(kname, "corner_stop_angle_min");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 224:	//�µ�׼ͣ
		sprintf(kname, "zmove_stop");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 225:	//ת�Ǽ��ٶ�����
		sprintf(kname, "corner_acc_limit");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 226:	//��ֱ�߶����м���
		sprintf(kname, "long_line_acc");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 228:  //�ս�׼ͣʹ��
		sprintf(kname, "corner_stop_enable");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 230:	//Բ���������
		sprintf(kname, "arc_err_limit");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 231:	//�������ٶȲ��ת���ٶ�ǯ��
		sprintf(kname, "chn_spd_limit_on_axis");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 232:	//���ڼ��ٶȵ�С�߶ν����ٶ�ǯ��
		sprintf(kname, "chn_spd_limit_on_acc");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 233:	//�������ʼ����ļ��ٵ�С�߶ν����ٶ�ǯ��
		sprintf(kname, "chn_spd_limit_on_curvity");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 234:	//G00�ص��ȼ�
		sprintf(kname, "chn_rapid_overlap_level");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 235:
		sprintf(kname, "chn_small_line_time");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint16);
		break;
#ifdef USES_WOOD_MACHINE
	case 240:
		sprintf(kname, "flip_comp_value");
		m_ini_chn->SetIntValue(sname, kname, value.value_int32);
		break;
#endif
	case 350:	//������ʽ
		sprintf(kname, "change_tool_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 351:	//�����������
		sprintf(kname, "tool_live_check");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 352:	//�Ե����Զ��Ե�����
		sprintf(kname, "auto_tool_measure");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 400:	//�ӹ��������
		sprintf(kname, "gcode_trace");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 401:	//������굥λ
		sprintf(kname, "gcode_unit");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 402:	//Ĭ��ƽ��
		sprintf(kname, "default_plane");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 403:	//Ĭ�ϱ��ģʽ
		sprintf(kname, "default_cmd_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 404:	//��չ��������ϵ����
		sprintf(kname, "ex_coord_count");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
    case 405:   //Ĭ�Ͻ�����ʽ
        sprintf(kname, "default_feed_mode");
        m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
        break;
	case 506:	//�ӹ���ʱ��ʽ
		sprintf(kname, "timing_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 510:   //G31��ת�ź�
		sprintf(kname, "g31_skip_signal");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 511:   //G31��ת�ź���Ч��ƽ
		sprintf(kname, "g31_sig_level");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
    case 512:   //��λʱ��
        sprintf(kname, "rst_hold_time");
        m_ini_chn->SetIntValue(sname, kname, value.value_uint16);
        break;
    case 513:   //��λ�Ƿ�����������
        sprintf(kname, "rst_mode");
        m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
        break;
    case 514:   //G00��߽����ٶ�
        sprintf(kname, "g00_max_speed");
        m_ini_chn->SetIntValue(sname, kname, value.value_uint32);
        break;
    case 515:   //G01��߽����ٶ�
        sprintf(kname, "g01_max_speed");
        m_ini_chn->SetIntValue(sname, kname, value.value_uint32);
        break;
#ifdef USES_WOOD_MACHINE
	case 600:  //DSP���Բ���1
		sprintf(kname, "debug_param_1");
		m_ini_chn->SetIntValue(sname, kname, value.value_int32);
		break;
	case 601:  //DSP���Բ���2
		sprintf(kname, "debug_param_2");
		m_ini_chn->SetIntValue(sname, kname, value.value_int32);
		break;
	case 602:  //DSP���Բ���3
		sprintf(kname, "debug_param_3");
		m_ini_chn->SetIntValue(sname, kname, value.value_int32);
		break;
	case 603:  //DSP���Բ���4
		sprintf(kname, "debug_param_4");
		m_ini_chn->SetIntValue(sname, kname, value.value_int32);
		break;
	case 604:  //DSP���Բ���5
		sprintf(kname, "debug_param_5");
		m_ini_chn->SetIntValue(sname, kname, value.value_int32);
		break;
#endif
	default:	//
		g_ptr_trace->PrintLog(LOG_ALARM, "ͨ���������£������ŷǷ���%d", param_no);
		res = false;
		return res;
	}
	m_ini_chn->Save();
	return res;
}

/**
 * @brief �����������ֻ��д��INI�ļ����������������
 * @param axis_index �� ���, ��0��ʼ���������
 * @param param_no �� ������
 * @param value �� ����ֵ
 */
bool ParmManager::UpdateAxisParam(uint8_t axis_index, uint32_t param_no, ParamValue &value){
	bool res = true;
//	if(axis_index >= this->m_sc_system_config->axis_count){
	if(axis_index >= this->m_sc_system_config->max_axis_count){   //֧�ֲ����޸ĺ�һ������
		g_ptr_trace->PrintLog(LOG_ALARM, "��������£���ŷǷ���%hhu", axis_index);
		return false;
	}
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));
//	printf("update axis param, axis[%hhu], param_no[%u], value[%hhu]\n", axis_index, param_no, value.value_uint8);

	sprintf(sname, "axis_%hhu", axis_index+1);
	switch(param_no){
	case 1001:	//������
		sprintf(kname, "axis_type");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1002:	//��ӿ�����
		sprintf(kname, "axis_interface");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1003:	//��վ�ţ������ᣩ/��Ӧ��ںţ��������ᣩ
		sprintf(kname, "axis_port");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1004:	//ֱ��������
		sprintf(kname, "axis_linear_type");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1006:	//�Ƿ�PMC��
		sprintf(kname, "axis_pmc");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1100:	//�Զ���������
		sprintf(kname, "kp1");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1101:	//�ֶ���������
		sprintf(kname, "kp2");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1102:	//���ֲ���
		sprintf(kname, "ki");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1103:	//���ֱ�����
		sprintf(kname, "kil");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1104:	//΢�ֲ���
		sprintf(kname, "kd");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1105:	//�ٶ�ǰ��ϵ��
		sprintf(kname, "kvff");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1106:	//���ٶ�ǰ��ϵ��
		sprintf(kname, "kaff");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1107:	//�����������
		sprintf(kname, "track_err_limit");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1108:	//��λ�������
		sprintf(kname, "location_err_limit");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1120:	//��λ���ٶ�
		sprintf(kname, "rapid_acc");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1121:	//�ֶ����ٶ�
		sprintf(kname, "manual_acc");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1122:	//�������ٶ�
		sprintf(kname, "start_acc");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1130:	//��λS�͹滮ʱ�䳣��
		sprintf(kname, "rapid_s_plan_filter_time");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1135:
		sprintf(kname, "corner_acc_limit");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1140:	//�岹��Ӽ����˲�������
		sprintf(kname, "post_filter_type");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1141:	//�岹��Ӽ����˲���ʱ�䳣��1
		sprintf(kname, "post_filter_time_1");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1142:	//�岹��Ӽ����˲���ʱ�䳣��2
		sprintf(kname, "post_filter_time_2");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1200:	//���ÿת����
		sprintf(kname, "motor_count_pr");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1201:	//������ת��
		sprintf(kname, "motor_speed_max");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1202:	//ÿת�ƶ�������˿���ݾ�
		sprintf(kname, "move_pr");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1203:	//�����ת����
		sprintf(kname, "motor_dir");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1204:	//��������
		sprintf(kname, "feedback_mode");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1205:	//����Ʒ�ʽ
		sprintf(kname, "ctrl_mode");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1206:	//ÿת���������
		sprintf(kname, "pulse_count_pr");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1207:	//��������Ȧ����
		sprintf(kname, "encoder_lines");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1208:	//��������Ȧ���ֵ
		sprintf(kname, "encoder_max_cycle");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1209:  //��澯��ƽ
		sprintf(kname, "axis_alarm_level");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1302:  //�ο����׼���
		sprintf(kname, "ref_mark_err");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1303:  //�زο��㷽ʽ
		sprintf(kname, "ret_ref_mode");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1304:  //�زο��㷽��
		sprintf(kname, "ret_ref_dir");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1305:  //�زο��㻻��
		sprintf(kname, "ret_ref_change_dir");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1306:  //�زο����ź�����
		sprintf(kname, "ref_signal");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1308:  //�زο����ٶ�
		sprintf(kname, "ret_ref_speed");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1309:  //�زο���˳��
		sprintf(kname, "ret_ref_index");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1310:  //�־���׼λ��ƫ����
		sprintf(kname, "ref_base_diff_check");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1312:   //�־���׼λ��ƫ��
		sprintf(kname, "ref_base_diff");
		m_ini_axis->SetIntValue(sname, kname, value.value_double);
		break;
	case 1314: 	//�ο��������ֵ
		sprintf(kname, "ref_encoder");
		m_ini_axis->SetInt64Value(sname, kname, value.value_int64);  //64λ����
		break;
    case 1315:  //�زο������
        sprintf(kname, "ret_ref_speed_second");
        m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
        break;
    case 1316:  //�زο�����ƫ����
        sprintf(kname, "ref_offset_pos");
        m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
        break;
    case 1317:  //����Z��������ƶ�����
        sprintf(kname, "ref_z_distance_max");
        m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
        break;
    case 1318:
        sprintf(kname, "absolute_ref_mode");
        m_ini_axis->SetIntValue(sname, kname, value.value_int8);
        break;

	case 1350:	//�ֶ��ٶ�
		sprintf(kname, "manual_speed");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1351:	//��λ�ٶ�
		sprintf(kname, "rapid_speed");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1352:	//��λ�ٶ�
		sprintf(kname, "reset_speed");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1400:  //�������϶
		printf("update axis 1400 val: %d\n", value.value_int16);
		sprintf(kname, "backlash_forward");
		m_ini_axis->SetIntValue(sname, kname, value.value_int16);
		break;
	case 1401:  //�������϶
		printf("update axis 1401 val: %d\n", value.value_int16);
		sprintf(kname, "backlash_negative");
		m_ini_axis->SetIntValue(sname, kname, value.value_int16);
		break;
	case 1402:   //�����϶�Ƿ���Ч  **************************
		printf("update axis 1402 \n");
		sprintf(kname, "backlash_enable");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		this->m_sc_axis_config[axis_index].backlash_enable = value.value_uint8;
		break;
	case 1403: //�ݾಹ������
		sprintf(kname, "pc_count");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		this->m_sc_axis_config[axis_index].pc_count = value.value_uint16;
		break;
	case 1404:  //�������
		sprintf(kname, "pc_inter_dist");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		this->m_sc_axis_config[axis_index].pc_inter_dist = value.value_double;
		break;
	case 1405:  //�ο��㲹��λ��
		sprintf(kname, "pc_ref_index");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		this->m_sc_axis_config[axis_index].pc_ref_index = value.value_uint16;
		break;
	case 1406:  //�ݾಹ����ʼ��
		sprintf(kname, "pc_offset");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		this->m_sc_axis_config[axis_index].pc_offset = value.value_uint16;
		break;
	case 1407:	//�ݾಹ������  ****************************
		printf("update axis 1407 \n");
		sprintf(kname, "pc_type");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		this->m_sc_axis_config[axis_index].pc_type = value.value_uint8;
		break;
	case 1408:  // �ݾಹ���Ƿ���Ч
		printf("update axis 1408 \n");
		sprintf(kname, "pc_enable");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		this->m_sc_axis_config[axis_index].pc_enable = value.value_uint8;
		break;
	case 1500: 	//����λ1
		sprintf(kname, "soft_limit_max_1");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1501:
		sprintf(kname, "soft_limit_min_1");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1502:
		sprintf(kname, "soft_limit_check_1");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1503: 	//����λ2
		sprintf(kname, "soft_limit_max_2");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1504:
		sprintf(kname, "soft_limit_min_2");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1505:
		sprintf(kname, "soft_limit_check_2");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1506: 	//����λ3
		sprintf(kname, "soft_limit_max_3");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1507:
		sprintf(kname, "soft_limit_min_3");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1508:
		sprintf(kname, "soft_limit_check_3");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1520:
	case 1521:
	case 1522:
	case 1523:
	case 1524:
	case 1525:
	case 1526:
	case 1527:
	case 1528:
	case 1529:  //�ο���
		sprintf(kname, "axis_home_pos_%d", param_no-1519);
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1550:  //���ٶ�λ    0--�ر�   1--��
		sprintf(kname, "fast_locate");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1551:  //λ����ʾģʽ   0--ѭ��ģʽ��0~360��    1--��ѭ��ģʽ
		sprintf(kname, "pos_disp_mode");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1602:	//������ٱ�
		sprintf(kname, "spd_gear_ratio");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1603:	//��Ư����
		sprintf(kname, "zero_compensation");
		m_ini_axis->SetIntValue(sname, kname, value.value_int16);
		break;
	case 1604:	//�������ת��
		sprintf(kname, "spd_max_speed");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1605: //��������ʱ��
		sprintf(kname, "spd_start_time");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1606: //�����ƶ�ʱ��
		sprintf(kname, "spd_stop_time");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1607:	//�������ת��
		sprintf(kname, "spd_min_speed");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1610:	//�����ѹ���Ʒ�ʽ
		sprintf(kname, "spd_vctrl_mode");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1612:	//��������ת��(rpm)
		sprintf(kname, "spd_set_speed");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1650:  //�Ƿ�ͬ����
		sprintf(kname, "sync_axis");
		m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
		break;
	case 1651:	//�������
		sprintf(kname, "master_axis_no");
		m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
		break;
	case 1652:	//��ʾ����
		sprintf(kname, "disp_coord");
		m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
		break;
	case 1653:	//�������׼λ��ƫ��
		sprintf(kname, "benchmark_offset");
		m_ini_axis->SetDoubleValue(sname, kname,value.value_double);
		break;
	case 1655:	//λ��ͬ��������
		sprintf(kname, "sync_err_max");
		m_ini_axis->SetIntValue(sname, kname,value.value_uint32);
		break;
	case 1656:	//�Ӷ���زο�����Զ�ͬ��У׼
		sprintf(kname, "auto_sync");
		m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
		break;
    case 1657:	//SOR�ź���;
        sprintf(kname, "spd_ctrl_GST");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1658:	//���ỻ����ʽ
        sprintf(kname, "spd_ctrl_SGB");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1659:	//���ֻ���ʱ�Ƿ����SF�ź�
        sprintf(kname, "spd_ctrl_SFA");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1660:	//���ᶨ��ʱ��ת��
        sprintf(kname, "spd_ctrl_ORM");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1661:	//����ת���Ƿ���M03/M04Ӱ��
        sprintf(kname, "spd_ctrl_TCW");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1662:	//����ת��ȡ��
        sprintf(kname, "spd_ctrl_CWM");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1663:	//���������͸��Թ�˿ʱ�����ᱶ������
        sprintf(kname, "spd_ctrl_TSO");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1664:	//ģ���������
        sprintf(kname, "spd_analog_gain");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1665:	//������ֻ���/����ʱ������ת��
        sprintf(kname, "spd_sor_speed");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1666:	//��������Сǯ���ٶ�
        sprintf(kname, "spd_motor_min_speed");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1667:	//���������ǯ���ٶ�
        sprintf(kname, "spd_motor_max_speed");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1668:	//���ֵ͵�λ���ת��
        sprintf(kname, "spd_gear_speed_low");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1669:	//�����е�λ���ת��
        sprintf(kname, "spd_gear_speed_middle");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1670:	//���ָߵ�λ���ת��
        sprintf(kname, "spd_gear_speed_high");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1671:	//B��ʽ��1->��2���ת��
        sprintf(kname, "spd_gear_switch_speed1");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1672:	//B��ʽ��2->��3���ת��
        sprintf(kname, "spd_gear_switch_speed2");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1673:	//��˿ͬ���������
        sprintf(kname, "spd_sync_error_gain");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint32);
        break;
    case 1674:	//��˿���ٶ�ǰ������
        sprintf(kname, "spd_speed_feed_gain");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint32);
        break;
    case 1675:	//��˿��λ�ñ�������
        sprintf(kname, "spd_pos_ratio_gain");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint32);
        break;

	default:
		g_ptr_trace->PrintLog(LOG_ALARM, "��������£������ŷǷ���%d", param_no);
		res = false;
		return res;
	}

	m_ini_axis->Save();
	return res;
}

/**
 * @brief ���¹������ͨ������
 * @param chn_index �� ͨ����
 * @param group_index : ���ղ�����ţ���0��ʼ
 * @param param_no �� ������
 * @param value �� ����ֵ
 * @return true--�ɹ�   false--ʧ��
 */
bool ParmManager::UpdateChnProcParam(uint8_t chn_index, uint8_t group_index, uint32_t param_no, ParamValue &value){
	bool res = true;
//	if(chn_index >= this->m_sc_system_config->chn_count){
	if(chn_index >= this->m_sc_system_config->max_chn_count){   //֧�ֲ����޸ģ�һ������
		g_ptr_trace->PrintLog(LOG_ALARM, "ͨ���������£�ͨ���ŷǷ���%hhu", chn_index);
		return false;
	}
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	sprintf(sname, "channel_%hhu", chn_index);
	group_index++;   //��ż�һ����1��ʼ
	switch(param_no){
	case 202:	//G00����ģʽ
		sprintf(kname, "rapid_mode_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
	//	printf("update para 202 , value = %hhu\n", value.value_uint8);
		break;

	case 203:	//�����ٶȹ滮��ʽ
		sprintf(kname, "cut_plan_mode_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 204:	//��λ�ٶȹ滮��ʽ
		sprintf(kname, "rapid_plan_mode_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 210:	//ͨ����������ٶ�
		sprintf(kname, "chn_max_vel_%hhu", group_index);
		m_ini_proc_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 211:	//ͨ�������ٶ�
		sprintf(kname, "chn_max_acc_%hhu", group_index);
		m_ini_proc_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 212:	//ͨ�������ٶ�
		sprintf(kname, "chn_max_dec_%hhu", group_index);
		m_ini_proc_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 214:	//�սǼ��ٶ�
		sprintf(kname, "chn_max_corner_acc_%hhu", group_index);
		m_ini_proc_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 215:	//������ļ��ٶ�
		sprintf(kname, "chn_max_arc_acc_%hhu", group_index);
		m_ini_proc_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 216:	//��������S�͹滮ʱ�䳣��
		sprintf(kname, "chn_s_cut_filter_time_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint16);
		break;

	case 223:	//�ս�׼ͣ����
		sprintf(kname, "corner_stop_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 229:  //�ս�׼ͣ���޽Ƕ�
		sprintf(kname, "corner_stop_angle_min_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 225:	//ת�Ǽ��ٶ�����
		sprintf(kname, "corner_acc_limit_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 228:  //�ս�׼ͣʹ��
		sprintf(kname, "corner_stop_enable_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 231:	//�������ٶȲ��ת���ٶ�ǯ��
		sprintf(kname, "chn_spd_limit_on_axis_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 232:	//���ڼ��ٶȵ�С�߶ν����ٶ�ǯ��
		sprintf(kname, "chn_spd_limit_on_acc_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 233:	//�������ʼ����ļ��ٵ�С�߶ν����ٶ�ǯ��
		sprintf(kname, "chn_spd_limit_on_curvity_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 234:	//G00�ص��ȼ�
		sprintf(kname, "chn_rapid_overlap_level_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 235:  //С�߶�ִ��ʱ�䳣��
		sprintf(kname, "chn_small_line_time_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint16);
		break;
#ifdef USES_WOOD_MACHINE
	case 240:  //���ǲ���ֵ
		sprintf(kname, "flip_comp_value_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_int32);
		break;
#endif
	default:	//
		g_ptr_trace->PrintLog(LOG_ALARM, "ͨ��������ز������£������ŷǷ���%d", param_no);
		res = false;
		return res;
	}
	m_ini_proc_chn->Save();
	return res;
}

/**
 * @brief ���¹�����������
 * @param axis_index �����, ��0��ʼ
 * @param group_index  : ���ղ������
 * @param param_no �� ������
 * @param value �� ����ֵ
 * @return true--�ɹ�  false--ʧ��
 */
bool ParmManager::UpdateAxisProcParam(uint8_t axis_index, uint8_t group_index, uint32_t param_no, ParamValue &value){
	bool res = true;
	if(axis_index >= this->m_sc_system_config->max_axis_count){   //֧�ֲ����޸ĺ�һ������
		g_ptr_trace->PrintLog(LOG_ALARM, "�Ṥ����ز������£���ŷǷ���%hhu", axis_index);
		return false;
	}
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));
	printf("update axis process param, axis[%hhu], group_index[%hhu], param_no[%u]\n", axis_index, group_index, param_no);

	sprintf(sname, "axis_%hhu", axis_index+1);
	group_index++;   //��ż�һ����1��ʼ
	switch(param_no){

	case 1120:	//��λ���ٶ�
		sprintf(kname, "rapid_acc_%hhu", group_index);
		m_ini_proc_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1121:	//�ֶ����ٶ�
		sprintf(kname, "manual_acc_%hhu", group_index);
		m_ini_proc_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1122:	//�������ٶ�
		sprintf(kname, "start_acc_%hhu", group_index);
		m_ini_proc_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1130:	//��λS�͹滮ʱ�䳣��
		sprintf(kname, "rapid_s_plan_filter_time_%hhu", group_index);
		m_ini_proc_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1135:  //�ս��ٶȲ�����
		sprintf(kname, "corner_acc_limit_%hhu", group_index);
		m_ini_proc_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1140:	//�岹��Ӽ����˲�������
		sprintf(kname, "post_filter_type_%hhu", group_index);
		m_ini_proc_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1141:	//�岹��Ӽ����˲���ʱ�䳣��1
		sprintf(kname, "post_filter_time_1_%hhu", group_index);
		m_ini_proc_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1142:	//�岹��Ӽ����˲���ʱ�䳣��2
		sprintf(kname, "post_filter_time_2_%hhu", group_index);
		m_ini_proc_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	default:
		g_ptr_trace->PrintLog(LOG_ALARM, "�Ṥ����ز������£������ŷǷ���%d", param_no);
		res = false;
		return res;
	}

	m_ini_proc_axis->Save();
	return res;
}

/**
 * @brief ����ָ����Ĳο��������ֵ
 * @param axis : ��ţ� ��0��ʼ
 * @param value
 * @return
 */
bool ParmManager::UpdateAxisRef(uint8_t axis, int64_t value){
	if(axis >= this->m_sc_system_config->max_axis_count){   //֧�ֲ����޸ĺ�һ������
		g_ptr_trace->PrintLog(LOG_ALARM, "��ο��������ֵ���£���ŷǷ���%hhu", axis);
		return false;
	}
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	sprintf(sname, "axis_%hhu", axis+1);

	sprintf(kname, "ref_encoder");
	m_ini_axis->SetInt64Value(sname, kname, value);  //64λ����

	m_ini_axis->Save();

//	printf("save axis %hhu ref origin point\n", axis);
	return true;
}

/**
 * @brief �������
 * @param data  : ����������
 * @param active_type : ��������
 */
void ParmManager::ActiveParam(ParamUpdate *data, uint8_t active_type){
	if(active_type == 0){	//������Ч
		this->m_b_poweroff_param = true;
	}else if(active_type == 1){		//��λ��Ч
		printf("add reset valid param:%hhu, %u \n", data->param_type, data->param_no);
		this->m_list_reset->Append(*data);
	}else if(active_type == 2){		//��һ�μӹ���Ч
		this->m_list_new_start->Append(*data);

	}else if(active_type == 3){		//������Ч
		this->ActiveParam(data);
	}else{
		g_ptr_trace->PrintLog(LOG_ALARM, "�����������ͷǷ���%hhu", active_type);
	}
}

/**
 * @brief �޸ĵ�ǰ����
 * @param data
 */
void ParmManager::ActiveParam(ParamUpdate *data){
	printf("active param : type = %hhu, param_no = %u\n", data->param_type, data->param_no);
	switch(data->param_type){
	case SYS_CONFIG:
		this->ActiveSystemParam(data->param_no, data->value);
		break;
	case CHN_CONFIG:
		this->ActiveChnParam(data->chn_index, data->param_no, data->value);
		break;
	case AXIS_CONFIG:
		this->ActiveAxisParam(data->axis_index, data->param_no, data->value);
		break;
#ifdef USES_FIVE_AXIS_FUNC
	case FIVE_AXIS_CONFIG:
		this->ActiveFiveAxisParam(data->chn_index, data->param_no, data->value);
		break;
#endif
#ifdef USES_GRIND_MACHINE
	case GRIND_CONFIG:
		this->ActiveGrindParam(data->param_no, data->value);
		break;
#endif
	default:
		break;
	}
}

/**
 * @brief �������ز�������
 * @param data  : ����������
 * @param active_type  : ��������
 */
void ParmManager::ActiveProcParam(ProcParamUpdate *data, uint8_t active_type){
	if(active_type == 0){	//������Ч
		this->m_b_poweroff_param = true;
	}else if(active_type == 1){		//��λ��Ч
	//	printf("add reset valid proc param:%hhu, %hhu, %u \n", data->group_index, data->param_data.param_type, data->param_data.param_no);
		this->m_list_proc_reset.Append(*data);
	}else if(active_type == 2){		//��һ�μӹ���Ч
		this->m_list_proc_new_start.Append(*data);

	}else if(active_type == 3){		//������Ч
		this->ActiveProcParam(data);
	}else{
		g_ptr_trace->PrintLog(LOG_ALARM, "������ز����������ͷǷ���%hhu", active_type);
	}
}

/**
 * @brief �޸ĵ�ǰ������ز�������
 * @param data  : ����������
 */
void ParmManager::ActiveProcParam(ProcParamUpdate *data){
	printf("active proc param :group = %hhu, type = %hhu, param_no = %u\n", data->group_index, data->param_data.param_type, data->param_data.param_no);

	ChannelEngine *chn_engine = ChannelEngine::GetInstance();

	switch(data->param_data.param_type){
	case CHN_CONFIG:
		if(chn_engine->GetChnControl(data->param_data.chn_index)->GetCurProcParamIndex() == data->group_index) //���ǵ�ǰ���ղ�����
			this->ActiveChnParam(data->param_data.chn_index, data->param_data.param_no, data->param_data.value, data->group_index);
		else
			this->ActiveChnProcParam(data->param_data.chn_index, data->param_data.param_no, data->param_data.value, data->group_index);
		break;
	case AXIS_CONFIG:{
		uint8_t chn_axis = 0;
		uint8_t chn_index = chn_engine->GetAxisChannel(data->param_data.axis_index, chn_axis);
		if(chn_index != 0xFF && chn_engine->GetChnControl(chn_index)->GetCurProcParamIndex() == data->group_index){  //������ͨ���ĵ�ǰ���ղ�����
			this->ActiveAxisParam(data->param_data.axis_index, data->param_data.param_no, data->param_data.value, data->group_index);
		}else{
			this->ActiveAxisProcParam(data->param_data.axis_index, data->param_data.param_no, data->param_data.value, data->group_index);
		}
		break;
	}
	default:
		break;
	}
}

/**
 * @brief ��������ͨ�����ղ���
 * @param chn_index ������ͨ���ţ���0��ʼ
 * @param param_no ���������
 * @param value �� ����ֵ
 * @param proc_index : �������ղ����飬0xFF��ʾ�ǹ��ղ���
 */
void ParmManager::ActiveChnProcParam(uint8_t chn_index, uint32_t param_no, ParamValue &value, uint8_t proc_index){

	switch(param_no){
	case 202:	//G00����ģʽ
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].rapid_mode = value.value_uint8;
		}
		break;
	case 203:	//�����ٶȹ滮��ʽ
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].cut_plan_mode = value.value_uint8;
		}
		break;
	case 204:	//��λ�ٶȹ滮��ʽ
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].rapid_plan_mode = value.value_uint8;
		}
		break;
	case 210:	//ͨ����������ٶ�
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_vel = value.value_double;
		}
		break;
	case 211:	//ͨ�������ٶ�
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_acc = value.value_double;
		}
		break;
	case 212:	//ͨ�������ٶ�
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_dec = value.value_double;
		}
		break;
	case 214:	//�սǼ��ٶ�
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_corner_acc = value.value_double;
		}
		break;
	case 215:	//������ļ��ٶ�
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_arc_acc = value.value_double;
		}
		break;
	case 216:	//��������S�͹滮ʱ�䳣��
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_s_cut_filter_time = value.value_uint16;
		}
		break;
	case 223:	//�ս�׼ͣ����
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_stop = value.value_uint8;
		}
		break;
	case 225:	//ת�Ǽ��ٶ�����
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_acc_limit = value.value_uint8;
		}
		break;
	case 228:	//�ս�׼ͣʹ��
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_stop_enable = value.value_uint8;
		}
		break;
	case 229:  //�ս�׼ͣ���޽Ƕ�
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_stop_angle_min = value.value_uint8;
		}
		break;

	case 231:	//�������ٶȲ��ת���ٶ�ǯ��
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_spd_limit_on_axis = value.value_uint8;
		}
		break;
	case 232:	//���ڼ��ٶȵ�С�߶ν����ٶ�ǯ��
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_spd_limit_on_acc = value.value_uint8;
		}
		break;
	case 233:	//�������ʼ����ļ��ٵ�С�߶ν����ٶ�ǯ��
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_spd_limit_on_curvity = value.value_uint8;
		}
		break;
	case 234:	//G00�ص��ȼ�
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_rapid_overlap_level = value.value_uint8;
		}
		break;
	case 235:	//С�߶�ִ��ʱ�䳣��
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_small_line_time = value.value_uint16;
		}
		break;
#ifdef USES_WOOD_MACHINE
	case 240:  //���ǲ���ֵ
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].flip_comp_value = value.value_int32;
		}
		break;
#endif
	default:	//
		g_ptr_trace->PrintLog(LOG_ALARM, "ͨ��������������ŷǷ���%d", param_no);
		break;
	}
}

/**
 * @brief ��������ͨ�������
 * @param axis_index ��������ţ���0��ʼ
 * @param param_no �� �������
 * @param value ������ֵ
 * @param proc_index : �������ղ�����ţ�0xFF��ʾ�ǹ��ղ���
 */
void ParmManager::ActiveAxisProcParam(uint8_t axis_index, uint32_t param_no, ParamValue &value, uint8_t proc_index){

	switch(param_no){
	case 1120:	//��λ���ٶ�
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].rapid_acc = value.value_double;
		}
		break;
	case 1121:	//�ֶ����ٶ�
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].manual_acc = value.value_double;
		}
		break;
	case 1122:	//�𲽼��ٶ�
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].start_acc = value.value_double;
		}
		break;
	case 1130:	//��λS���ٶȹ滮ʱ�䳣��
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].rapid_s_plan_filter_time = value.value_uint16;
		}
		break;
	case 1135:  //�ս��ٶȲ�����
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].corner_acc_limit = value.value_double;
		}
		break;
	case 1140:	//�岹���˲�������
		if(proc_index < kMaxProcParamCount){ //�˲�������
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].post_filter_type = value.value_uint8;
		}
		break;
	case 1141:	//�岹���˲���ʱ�䳣��1
		if(proc_index < kMaxProcParamCount){ //һ���˲���ʱ�䳣��
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].post_filter_time_1 = value.value_uint16;
		}
		break;
	case 1142:	//�岹���˲���ʱ�䳣��2
		if(proc_index < kMaxProcParamCount){ //�����˲���ʱ�䳣��
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].post_filter_time_2 = value.value_uint16;
		}
		break;

	default:
		g_ptr_trace->PrintLog(LOG_ALARM, "��������������������ŷǷ���%d", param_no);
		break;
	}
}


/**
 * @brief ���λ��Ч�Ĳ���
 */
void ParmManager::ActiveResetParam(){
	printf("active reset param: %d\n", this->m_list_reset->GetLength());
	ParamUpdate *para = nullptr;
	ListNode<ParamUpdate> *node = nullptr;

	while(!this->m_list_reset->IsEmpty()){

		node = this->m_list_reset->RemoveHead();
		if(node == nullptr)
			continue;
		para = &node->data;
		this->ActiveParam(para);
		delete node;
		node = nullptr;
	}

	ProcParamUpdate *proc_para = nullptr;
	ListNode<ProcParamUpdate> *proc_node = nullptr;
	while(!this->m_list_proc_reset.IsEmpty()){
		proc_node = this->m_list_proc_reset.RemoveHead();
		if(proc_node == nullptr)
			continue;
		proc_para = &proc_node->data;
		this->ActiveProcParam(proc_para);
		delete proc_node;
		proc_node = nullptr;
	}
}

/**
 * @brief ������һ�μӹ���Ч�Ĳ���
 */
void ParmManager::ActiveNewStartParam(){
	ParamUpdate *para = nullptr;
	ListNode<ParamUpdate> *node = nullptr;
	while(!this->m_list_new_start->IsEmpty()){
		node = this->m_list_new_start->RemoveHead();
		if(node == nullptr)
			continue;
		para = &node->data;
		this->ActiveParam(para);

		delete node;
		node = nullptr;
	}

	ProcParamUpdate *proc_para = nullptr;
	ListNode<ProcParamUpdate> *proc_node = nullptr;
	while(!this->m_list_proc_new_start.IsEmpty()){
		proc_node = this->m_list_proc_new_start.RemoveHead();
		if(proc_node == nullptr)
			continue;
		proc_para = &proc_node->data;
		this->ActiveProcParam(proc_para);
		delete proc_node;
		proc_node = nullptr;
	}
}

#ifdef USES_GRIND_MACHINE
/**
 * @brief ����ĥ������
 * @param param_no
 * @param value
 */
void ParmManager::ActiveGrindParam(uint32_t param_no, ParamValue &value){
	//
}

#endif

/**
 * @brief ����ϵͳ����
 * @param param_no
 * @param value
 */
void ParmManager::ActiveSystemParam(uint32_t param_no, ParamValue &value){
	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	switch(param_no){
	case 3: 	//ͨ����
		this->m_sc_system_config->chn_count = value.value_uint8;
		break;
	case 5:		//��������
		this->m_sc_system_config->axis_count = value.value_uint8;
		break;
	case 6:		//����ͨѶ����
		this->m_sc_system_config->bus_cycle = value.value_uint8;
		break;
	case 10:  //���ֱ����ʽ
		this->m_sc_system_config->hw_code_type = value.value_uint8;
		this->UpdateMiParam<uint8_t>(0xFF, 10, value.value_uint8);   //���ֱ�������
		break;
	case 11:   //���ַ�������
		this->m_sc_system_config->hw_rev_trace = value.value_uint8;
		chn_engine->EnableHWTraceToMi();
		break;
//	case 20:	//ϵͳ����ٶ�
//		this->m_sc_system_config->max_sys_vel = value.value_double;
//		break;
//	case 24:	//�ս������ٶ�
//		this->m_sc_system_config->max_corner_acc = value.value_double;
//		break;
//	case 25:	//���������ٶ�
//		this->m_sc_system_config->max_cent_acc = value.value_double;
//		break;
	case 30:	//�������±�
		this->m_sc_system_config->axis_name_ex = value.value_uint8;
		chn_engine->SetAxisNameEx(value.value_uint8);
		break;
	case 31:	//�زο���ʱ���ʹ̶�
		this->m_sc_system_config->fix_ratio_find_ref = value.value_uint8;
		break;
	case 32:	//�ݾಹ����ʽ
		//this->m_sc_system_config->pc_type = value.value_uint8;
		break;
	case 33:	//DA��������
		this->m_sc_system_config->da_ocp = value.value_uint8;
		break;
	case 34:	//�ϵ㱣�����ǰ�к�
		this->m_sc_system_config->save_lineno_poweroff = value.value_uint8;
		break;
	case 35:	//�ֶ��زο���ģʽ
		this->m_sc_system_config->manual_ret_ref_mode = value.value_uint8;
		break;
	case 60:	//��ͨIO�˲�ʱ��
		this->m_sc_system_config->io_filter_time = value.value_uint32;
		break;
	case 61:	//����IO�˲�ʱ��
		this->m_sc_system_config->fast_io_filter_time = value.value_uint32;
		break;
	case 90:	//������ʱʱ��
		this->m_sc_system_config->backlight_delay_time = value.value_uint16;
		break;
	case 91:	//����ʱ������
		this->m_sc_system_config->beep_time = value.value_uint8;
		break;
	case 92:	//�洢�ռ�澯����
		this->m_sc_system_config->free_space_limit = value.value_uint32;
		break;
	case 93:	//����澯�¶�
		this->m_sc_system_config->alarm_temperature = value.value_uint8;
		break;
	case 99:	//������Ϣ����
		this->m_sc_system_config->trace_level = value.value_uint8;
		g_ptr_trace->set_trace_level(m_sc_system_config->trace_level);
		break;
	case 100:	//����ģʽ
		this->m_sc_system_config->debug_mode = value.value_uint8;
		break;
	default:	//Ĭ��
		break;
	}
}

/**
 * @brief ����ͨ������
 * @param chn_index ������ͨ���ţ���0��ʼ
 * @param param_no ���������
 * @param value �� ����ֵ
 * @param proc_index : �������ղ����飬0xFF��ʾ�ǹ��ղ���
 */
void ParmManager::ActiveChnParam(uint8_t chn_index, uint32_t param_no, ParamValue &value, uint8_t proc_index){
	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	switch(param_no){
	case 102:	//������
		this->m_sc_channel_config[chn_index].chn_axis_count = value.value_uint8;
		break;
	case 103:  //ͨ��������ʽ��
		chn_engine->ChangeChnGroupIndex(chn_index, m_sc_channel_config[chn_index].chn_group_index, value.value_uint8);
		this->m_sc_channel_config[chn_index].chn_group_index = value.value_uint8;
		break;
	case 104:	//������X
//		this->m_sc_channel_config[chn_index].chn_axis_x = value.value_uint8;
//		break;
	case 105:	//������Y
//		this->m_sc_channel_config[chn_index].chn_axis_y = value.value_uint8;
//		break;
	case 106:	//������Z
//		this->m_sc_channel_config[chn_index].chn_axis_z = value.value_uint8;
//		break;
	case 107:	//��4
	case 108:	//��5
	case 109:	//��6
	case 110:	//��7
	case 111:	//��8
		this->m_sc_channel_config[chn_index].chn_axis_phy[param_no-104] = value.value_uint8;
		break;
	case 136:
	case 137:
	case 138:
	case 139:
	case 140:
	case 141:
	case 142:
	case 143:
		this->m_sc_channel_config[chn_index].chn_axis_name[param_no-136] = value.value_uint8;
		break;
	case 168:
	case 169:
	case 170:
	case 171:
	case 172:
	case 173:
	case 174:
	case 175:
		this->m_sc_channel_config[chn_index].chn_axis_name_ex[param_no-168] = value.value_uint8;
		break;
	case 200:	//�岹ģʽ
		this->m_sc_channel_config[chn_index].intep_mode = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		break;
	case 201:	//�岹����
		this->m_sc_channel_config[chn_index].intep_cycle = value.value_uint8;
		break;
	case 202:	//G00����ģʽ
		this->m_sc_channel_config[chn_index].rapid_mode = value.value_uint8;
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].rapid_mode = value.value_uint8;
		}
		break;
	case 203:	//�����ٶȹ滮��ʽ
		this->m_sc_channel_config[chn_index].cut_plan_mode = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanMode();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].cut_plan_mode = value.value_uint8;
		}
		break;
	case 204:	//��λ�ٶȹ滮��ʽ
		this->m_sc_channel_config[chn_index].rapid_plan_mode = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanMode();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].rapid_plan_mode = value.value_uint8;
		}
		break;
	case 210:	//ͨ����������ٶ�
		this->m_sc_channel_config[chn_index].chn_max_vel = value.value_double;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_vel = value.value_double;
		}
		break;
	case 211:	//ͨ�������ٶ�
		this->m_sc_channel_config[chn_index].chn_max_acc = value.value_double;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_acc = value.value_double;
		}
		break;
	case 212:	//ͨ�������ٶ�
		this->m_sc_channel_config[chn_index].chn_max_dec = value.value_double;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_dec = value.value_double;
		}
		break;
	case 213:	//
		break;
	case 214:	//�սǼ��ٶ�
		this->m_sc_channel_config[chn_index].chn_max_corner_acc = value.value_double;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_corner_acc = value.value_double;
		}
		break;
	case 215:	//������ļ��ٶ�
		this->m_sc_channel_config[chn_index].chn_max_arc_acc = value.value_double;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_arc_acc = value.value_double;
		}
		break;
	case 216:	//��������S�͹滮ʱ�䳣��
		this->m_sc_channel_config[chn_index].chn_s_cut_filter_time = value.value_uint16;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_s_cut_filter_time = value.value_uint16;
		}
		break;
	case 220:	//�ӹ�����
		this->m_sc_channel_config[chn_index].chn_precision = value.value_uint16;
		break;
	case 221:	//ǰհ����
		this->m_sc_channel_config[chn_index].chn_look_ahead = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		break;
	case 222:	//�ӹ��ٶȵ����ȼ�
		this->m_sc_channel_config[chn_index].chn_feed_limit_level = value.value_uint8;
		break;
	case 223:	//�ս�׼ͣ����
		this->m_sc_channel_config[chn_index].corner_stop = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnCornerStopParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_stop = value.value_uint8;
		}
		break;
	case 224:	//�µ�׼ͣ
		this->m_sc_channel_config[chn_index].zmove_stop = value.value_uint8;
		break;
	case 225:	//ת�Ǽ��ٶ�����
		this->m_sc_channel_config[chn_index].corner_acc_limit = value.value_uint8;
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_acc_limit = value.value_uint8;
		}
		break;
	case 226:	//��ֱ�߶����м���
		this->m_sc_channel_config[chn_index].long_line_acc = value.value_uint8;
		break;
	case 228:	//�ս�׼ͣʹ��
		this->m_sc_channel_config[chn_index].corner_stop_enable = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnCornerStopParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_stop_enable = value.value_uint8;
		}
		break;
	case 229:  //�ս�׼ͣ���޽Ƕ�
		this->m_sc_channel_config[chn_index].corner_stop_angle_min = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnCornerStopParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_stop_angle_min = value.value_uint8;
		}
		break;
	case 230:	//Բ���������
		this->m_sc_channel_config[chn_index].arc_err_limit = value.value_uint8;
		break;
	case 231:	//�������ٶȲ��ת���ٶ�ǯ��
		this->m_sc_channel_config[chn_index].chn_spd_limit_on_axis = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_spd_limit_on_axis = value.value_uint8;
		}
		break;
	case 232:	//���ڼ��ٶȵ�С�߶ν����ٶ�ǯ��
		this->m_sc_channel_config[chn_index].chn_spd_limit_on_acc = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_spd_limit_on_acc = value.value_uint8;
		}
		break;
	case 233:	//�������ʼ����ļ��ٵ�С�߶ν����ٶ�ǯ��
		this->m_sc_channel_config[chn_index].chn_spd_limit_on_curvity = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_spd_limit_on_curvity = value.value_uint8;
		}
		break;
	case 234:	//G00�ص��ȼ�
		this->m_sc_channel_config[chn_index].chn_rapid_overlap_level = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_rapid_overlap_level = value.value_uint8;
		}
		break;
	case 235:	//С�߶�ִ��ʱ�䳣��
		this->m_sc_channel_config[chn_index].chn_small_line_time = value.value_uint16;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_small_line_time = value.value_uint16;
		}
		break;
#ifdef USES_WOOD_MACHINE
	case 240:  //���ǲ���ֵ
		this->m_sc_channel_config[chn_index].flip_comp_value = value.value_int32;
		chn_engine->GetChnControl(chn_index)->SetMcFlipCompParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].flip_comp_value = value.value_int32;
		}
		break;
#endif
	case 350:	//������ʽ
		this->m_sc_channel_config[chn_index].change_tool_mode = value.value_uint8;
		break;
	case 351:	//�����������
		this->m_sc_channel_config[chn_index].tool_live_check = value.value_uint8;
		break;
	case 352:	//�Ե����Զ��Ե�����
		this->m_sc_channel_config[chn_index].auto_tool_measure = value.value_uint8;
		break;
	case 400:	//�ӹ��������
		this->m_sc_channel_config[chn_index].gcode_trace = value.value_uint8;
		break;
	case 401:	//������굥λ
		this->m_sc_channel_config[chn_index].gcode_unit = value.value_uint8;
		break;
	case 402:	//Ĭ��ƽ��
		this->m_sc_channel_config[chn_index].default_plane = value.value_uint8;
		break;
	case 403:	//Ĭ�ϱ��ģʽ
		this->m_sc_channel_config[chn_index].default_cmd_mode = value.value_uint8;
		break;
	case 404:	//��չ��������ϵ����
		this->m_sc_channel_config[chn_index].ex_coord_count = value.value_uint8;
		break;
    case 405:	//Ĭ�Ͻ�����ʽ
        this->m_sc_channel_config[chn_index].default_feed_mode = value.value_uint8;
        break;
	case 506:	//�ӹ���ʱ��ʽ
		this->m_sc_channel_config[chn_index].timing_mode = value.value_uint8;
		break;
	case 510:   //G31��ת�ź�
		this->m_sc_channel_config[chn_index].g31_skip_signal = value.value_uint32;
		break;
	case 511:   //G31��ת�ź���Ч��ƽ
		this->m_sc_channel_config[chn_index].g31_sig_level = value.value_uint8;
		break;
    case 512:   //��λʱ��
        this->m_sc_channel_config[chn_index].rst_hold_time = value.value_uint16;
        break;
    case 513:   //��λ�Ƿ�������ʱ��
        this->m_sc_channel_config[chn_index].rst_mode = value.value_uint8;
        break;
    case 514:   //G00��߽����ٶ�
        this->m_sc_channel_config[chn_index].g00_max_speed = value.value_uint32;
        break;
    case 515:   //G01��߽����ٶ�
        this->m_sc_channel_config[chn_index].g01_max_speed = value.value_uint32;
        break;
#ifdef USES_WOOD_MACHINE
	case 600:  //DSP���Բ���1
		this->m_sc_channel_config[chn_index].debug_param_1 = value.value_int32;
		chn_engine->GetChnControl(chn_index)->SetMcDebugParam(0);
		break;
	case 601:  //DSP���Բ���2
		this->m_sc_channel_config[chn_index].debug_param_2 = value.value_int32;
		chn_engine->GetChnControl(chn_index)->SetMcDebugParam(1);
		break;
	case 602:  //DSP���Բ���3
		this->m_sc_channel_config[chn_index].debug_param_3 = value.value_int32;
		chn_engine->GetChnControl(chn_index)->SetMcDebugParam(2);
		break;
	case 603:  //DSP���Բ���4
		this->m_sc_channel_config[chn_index].debug_param_4 = value.value_int32;
		chn_engine->GetChnControl(chn_index)->SetMcDebugParam(3);
		break;
	case 604:  //DSP���Բ���5
		this->m_sc_channel_config[chn_index].debug_param_5 = value.value_int32;
		chn_engine->GetChnControl(chn_index)->SetMcDebugParam(4);
		break;
#endif
	default:	//
		g_ptr_trace->PrintLog(LOG_ALARM, "ͨ��������������ŷǷ���%d", param_no);
		break;
	}
}

/**
 * @brief ���·���MI����
 * @param axis : ��ţ� MI����Ŵ�1��ʼ, 0xFF��ʾϵͳ����
 * @param para_no : ������
 * @param data ������ָ��
 * @param size ������������ռ���ֽ���
 */
template<typename T>
void ParmManager::UpdateMiParam(uint8_t axis, uint32_t para_no, T data){
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_SET_PARAM;

	cmd.data.axis_index = axis;

	memcpy(cmd.data.data, &para_no, 4);
	memcpy(&cmd.data.data[2], &data, sizeof(T));

	MICommunication::GetInstance()->WriteCmd(cmd);
}

/**
 * @brief ���������
 * @param axis_index ��������ţ���0��ʼ
 * @param param_no �� �������
 * @param value ������ֵ
 * @param proc_index : �������ղ�����ţ�0xFF��ʾ�ǹ��ղ���
 */
void ParmManager::ActiveAxisParam(uint8_t axis_index, uint32_t param_no, ParamValue &value, uint8_t proc_index){
	uint8_t chan = 0;
	uint8_t chan_axis = 0;
	uint64_t tmp_64 = 0;   //��ʱ����
	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	switch(param_no){
	case 1001:	//������
		this->m_sc_axis_config[axis_index].axis_type = value.value_uint8;
		break;
	case 1002:	//��ӿ�����
		this->m_sc_axis_config[axis_index].axis_interface = value.value_uint8;
		break;
	case 1003:	//��վ�ţ������ᣩ/��Ӧ��ںţ��������ᣩ
		this->m_sc_axis_config[axis_index].axis_port = value.value_uint8;
		UpdateMiParam<uint8_t>(axis_index+1, param_no, value.value_uint8);
		break;
	case 1004:	//ֱ��������
		this->m_sc_axis_config[axis_index].axis_linear_type = value.value_uint8;
		break;
	case 1006:	//�Ƿ�PMC��
		this->m_sc_axis_config[axis_index].axis_pmc = value.value_uint8;
		break;
	case 1100:	//�Զ���������
		printf("update axis[%hhu] kp1: %lf\n", axis_index, value.value_double);
		this->m_sc_axis_config[axis_index].kp1 = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
		break;
	case 1101:	//�ֶ���������
		this->m_sc_axis_config[axis_index].kp2 = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
		break;
	case 1102:	//���ֲ���
		this->m_sc_axis_config[axis_index].ki = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
		break;
	case 1103:	//���ֱ�����
		printf("update axis[%hhu] kil: %lf\n", axis_index, value.value_double);
		this->m_sc_axis_config[axis_index].kil = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
		break;
	case 1104:	//΢�ֲ���
		this->m_sc_axis_config[axis_index].kd = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
		break;
	case 1105:	//�ٶ�ǰ��ϵ��
		this->m_sc_axis_config[axis_index].kvff = value.value_uint16;
		UpdateMiParam<double>(axis_index+1, param_no, static_cast<double>(value.value_uint16));
		break;
	case 1106:	//���ٶ�ǰ��ϵ��
		this->m_sc_axis_config[axis_index].kaff = value.value_uint16;
		UpdateMiParam<double>(axis_index+1, param_no, static_cast<double>(value.value_uint16));
		break;
	case 1107:	//�����������
		this->m_sc_axis_config[axis_index].track_err_limit = value.value_uint32;
		tmp_64 = static_cast<uint64_t>(value.value_uint32) * 1e4;   //ת����λ��um-->0.1nm
		this->UpdateMiParam<uint64_t>(axis_index+1, param_no, tmp_64);	//���������
		break;
	case 1108:	//��λ�������
		this->m_sc_axis_config[axis_index].location_err_limit = value.value_uint16;
		tmp_64 = static_cast<uint64_t>(value.value_uint16) * 1e4;   //ת����λ��um-->0.1nm
		this->UpdateMiParam<uint64_t>(axis_index+1, param_no, tmp_64);			//��λ�����
		break;
	case 1120:	//��λ���ٶ�
		this->m_sc_axis_config[axis_index].rapid_acc = value.value_double;
//		chn_engine->SetAxisAccParam(axis_index);
		chan = chn_engine->GetAxisChannel(axis_index, chan_axis);
		chn_engine->GetChnControl(chan)->SetChnAxisAccParam(chan_axis);
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].rapid_acc = value.value_double;
		}
		break;
	case 1121:	//�ֶ����ٶ�
		this->m_sc_axis_config[axis_index].manual_acc = value.value_double;
//		chn_engine->SetAxisAccParam(axis_index);
		chan = chn_engine->GetAxisChannel(axis_index, chan_axis);
		chn_engine->GetChnControl(chan)->SetChnAxisAccParam(chan_axis);
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].manual_acc = value.value_double;
		}
		break;
	case 1122:	//�𲽼��ٶ�
		this->m_sc_axis_config[axis_index].start_acc = value.value_double;
//		chn_engine->SetAxisAccParam(axis_index);
		chan = chn_engine->GetAxisChannel(axis_index, chan_axis);
		chn_engine->GetChnControl(chan)->SetChnAxisAccParam(chan_axis);
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].start_acc = value.value_double;
		}
		break;
	case 1130:	//��λS���ٶȹ滮ʱ�䳣��
		this->m_sc_axis_config[axis_index].rapid_s_plan_filter_time = value.value_uint16;
//		chn_engine->SetAxisAccParam(axis_index);
		chan = chn_engine->GetAxisChannel(axis_index, chan_axis);
		chn_engine->GetChnControl(chan)->SetChnAxisAccParam(chan_axis);
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].rapid_s_plan_filter_time = value.value_uint16;
		}
		break;
	case 1135:  //�ս��ٶȲ�����
		this->m_sc_axis_config[axis_index].corner_acc_limit = value.value_double;
//		chn_engine->SetAxisSpeedParam(axis_index);
		chan = chn_engine->GetAxisChannel(axis_index, chan_axis);
		chn_engine->GetChnControl(chan)->SetChnAxisSpeedParam(chan_axis);
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].corner_acc_limit = value.value_double;
		}
		break;
	case 1140:	//�岹���˲�������
		this->m_sc_axis_config[axis_index].post_filter_type = value.value_uint8;
		this->UpdateMiParam<uint8_t>(axis_index+1, 1140, value.value_uint8);   //�˲�������
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].post_filter_type = value.value_uint8;
		}
		break;
	case 1141:	//�岹���˲���ʱ�䳣��1
		this->m_sc_axis_config[axis_index].post_filter_time_1 = value.value_uint16;
		this->UpdateMiParam<uint16_t>(axis_index+1, 1141, value.value_uint16);   //һ���˲���ʱ�䳣��
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].post_filter_time_1 = value.value_uint16;
		}
		break;
	case 1142:	//�岹���˲���ʱ�䳣��2
		this->m_sc_axis_config[axis_index].post_filter_time_2 = value.value_uint16;
		this->UpdateMiParam<uint16_t>(axis_index+1, 1142, value.value_uint16);   //�����˲���ʱ�䳣��
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].post_filter_time_2 = value.value_uint16;
		}
		break;
	case 1200:	//���ÿת����
		this->m_sc_axis_config[axis_index].motor_count_pr = value.value_uint32;
		break;
	case 1201:	//������ת��
		this->m_sc_axis_config[axis_index].motor_speed_max = value.value_uint32;
		break;
	case 1202:	//ÿת�ƶ�������˿���ݾ�
		this->m_sc_axis_config[axis_index].move_pr = value.value_double;
		break;
	case 1203:	//�����ת����
		this->m_sc_axis_config[axis_index].motor_dir = value.value_uint8;
		break;
	case 1204:	//��������
		this->m_sc_axis_config[axis_index].feedback_mode = value.value_uint8;
		break;
	case 1205:	//����Ʒ�ʽ
		this->m_sc_axis_config[axis_index].ctrl_mode = value.value_uint8;
		break;
	case 1206:	//ÿת���������
		this->m_sc_axis_config[axis_index].pulse_count_pr = value.value_uint32;
		break;
	case 1207:	//��������Ȧ����
		this->m_sc_axis_config[axis_index].encoder_lines = value.value_uint8;
		break;
	case 1208:	//��������Ȧ���ֵ
		this->m_sc_axis_config[axis_index].encoder_max_cycle = value.value_uint16;
		UpdateMiParam<uint16_t>(axis_index+1, param_no, value.value_uint16);
		break;
	case 1209:  //��澯��ƽ
//		this->m_sc_axis_config[axis_index].axis_alarm_level = value.value_uint8;
		UpdateMiParam<uint8_t>(axis_index+1, param_no, value.value_uint8);
		break;
	case 1302:  //�ο����׼���
		this->m_sc_axis_config[axis_index].ref_mark_err = value.value_double;
		break;
	case 1303:  //�زο��㷽ʽ
		this->m_sc_axis_config[axis_index].ret_ref_mode = value.value_uint8;

		if(value.value_uint8 == 0){
			chn_engine->SetRetRefFlag(axis_index, true);
		}else{
			chn_engine->SetRetRefFlag(axis_index, false);
		}
		break;
	case 1304:  //�زο��㷽��
		this->m_sc_axis_config[axis_index].ret_ref_dir = value.value_uint8;
		break;
	case 1305:  //�زο��㻻��
		this->m_sc_axis_config[axis_index].ret_ref_change_dir = value.value_uint8;
		break;
	case 1306:  //�زο����ź�����
		this->m_sc_axis_config[axis_index].ref_signal = value.value_uint8;
		break;
	case 1308:  //�زο����ٶ�
		this->m_sc_axis_config[axis_index].ret_ref_speed = value.value_double;
		break;
	case 1309:  //�زο���˳��
		this->m_sc_axis_config[axis_index].ret_ref_index = value.value_uint8;
		break;
	case 1310:  //�־���׼λ��ƫ����
		this->m_sc_axis_config[axis_index].ref_base_diff_check = value.value_uint8;
		break;
	case 1312:   //�־���׼λ��ƫ��
		this->m_sc_axis_config[axis_index].ref_base_diff = value.value_double;
		break;
    case 1315:   //�زο������
        this->m_sc_axis_config[axis_index].ret_ref_speed_second = value.value_double;
        break;
    case 1316:  //�زο�����ƫ����
        this->m_sc_axis_config[axis_index].ref_offset_pos = value.value_double;
        break;
    case 1317:  //����Z��������ƶ�����
        this->m_sc_axis_config[axis_index].ref_z_distance_max = value.value_double;
        break;
    case 1318:  //����ʽ������㷽ʽ
        this->m_sc_axis_config[axis_index].absolute_ref_mode = value.value_int8;
        break;
	case 1350:	//�ֶ��ٶ�
		this->m_sc_axis_config[axis_index].manual_speed = value.value_double;
		break;
	case 1351:	//��λ�ٶ�
		this->m_sc_axis_config[axis_index].rapid_speed = value.value_double;
//		chn_engine->SetAxisSpeedParam(axis_index);
		chan = chn_engine->GetAxisChannel(axis_index, chan_axis);
		chn_engine->GetChnControl(chan)->SetChnAxisSpeedParam(chan_axis);
		break;
	case 1352:	//��λ�ٶ�
		this->m_sc_axis_config[axis_index].reset_speed = value.value_double;
		break;
	case 1400:  //�������϶
		this->m_sc_axis_config[axis_index].backlash_forward = value.value_int16;
		chn_engine->SendMiBacklash(axis_index);
		break;
	case 1401:  //�������϶
		this->m_sc_axis_config[axis_index].backlash_negative = value.value_int16;
		chn_engine->SendMiBacklash(axis_index);
		break;
	case 1402:  //�����϶������Ч
		this->m_sc_axis_config[axis_index].backlash_enable = value.value_uint8;
		chn_engine->SendMiBacklash(axis_index);
		break;
	case 1405:  //�ο��㲹��λ��
		break;
	case 1500: 	//����λ1
		this->m_sc_axis_config[axis_index].soft_limit_max_1 = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
//		g_ptr_chn_engine->SetAxisSoftLimitValue(axis_index, 0);
		chan = g_ptr_chn_engine->GetAxisChannel(axis_index, chan_axis);
		g_ptr_chn_engine->GetChnControl(chan)->SetChnAxisSoftLimitValue(chan_axis, 0);
		break;
	case 1501:
		{
		this->m_sc_axis_config[axis_index].soft_limit_min_1 = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
//		g_ptr_chn_engine->SetAxisSoftLimitValue(axis_index, 0);
		chan = g_ptr_chn_engine->GetAxisChannel(axis_index, chan_axis);
		g_ptr_chn_engine->GetChnControl(chan)->SetChnAxisSoftLimitValue(chan_axis, 0);
		}
		break;
	case 1502:
		this->m_sc_axis_config[axis_index].soft_limit_check_1 = value.value_uint8;
		UpdateMiParam<uint8_t>(axis_index+1, param_no, value.value_uint8);
//		g_ptr_chn_engine->SetAxisSoftLimit(axis_index);
		chan = g_ptr_chn_engine->GetAxisChannel(axis_index, chan_axis);
		g_ptr_chn_engine->GetChnControl(chan)->SetChnAxisSoftLimit(chan_axis);
		break;
	case 1503: 	//����λ2
		this->m_sc_axis_config[axis_index].soft_limit_max_2 = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
//		g_ptr_chn_engine->SetAxisSoftLimitValue(axis_index, 1);
		chan = g_ptr_chn_engine->GetAxisChannel(axis_index, chan_axis);
		g_ptr_chn_engine->GetChnControl(chan)->SetChnAxisSoftLimitValue(chan_axis, 1);
		break;
	case 1504:
		this->m_sc_axis_config[axis_index].soft_limit_min_2 = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
//		g_ptr_chn_engine->SetAxisSoftLimitValue(axis_index, 1);
		chan = g_ptr_chn_engine->GetAxisChannel(axis_index, chan_axis);
		g_ptr_chn_engine->GetChnControl(chan)->SetChnAxisSoftLimitValue(chan_axis, 1);
		break;
	case 1505:
		this->m_sc_axis_config[axis_index].soft_limit_check_2 = value.value_uint8;
		UpdateMiParam<uint8_t>(axis_index+1, param_no, value.value_uint8);
//		g_ptr_chn_engine->SetAxisSoftLimit(axis_index);
		chan = g_ptr_chn_engine->GetAxisChannel(axis_index, chan_axis);
		g_ptr_chn_engine->GetChnControl(chan)->SetChnAxisSoftLimit(chan_axis);
		break;
	case 1506: 	//����λ3
		this->m_sc_axis_config[axis_index].soft_limit_max_3 = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
//		g_ptr_chn_engine->SetAxisSoftLimitValue(axis_index, 2);
		chan = g_ptr_chn_engine->GetAxisChannel(axis_index, chan_axis);
		g_ptr_chn_engine->GetChnControl(chan)->SetChnAxisSoftLimitValue(chan_axis, 2);
		break;
	case 1507:
		this->m_sc_axis_config[axis_index].soft_limit_min_3 = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
//		g_ptr_chn_engine->SetAxisSoftLimitValue(axis_index, 2);
		chan = g_ptr_chn_engine->GetAxisChannel(axis_index, chan_axis);
		g_ptr_chn_engine->GetChnControl(chan)->SetChnAxisSoftLimitValue(chan_axis, 2);
		break;
	case 1508:
		this->m_sc_axis_config[axis_index].soft_limit_check_3 = value.value_uint8;
		UpdateMiParam<uint8_t>(axis_index+1, param_no, value.value_uint8);
//		g_ptr_chn_engine->SetAxisSoftLimit(axis_index);
		chan = g_ptr_chn_engine->GetAxisChannel(axis_index, chan_axis);
		g_ptr_chn_engine->GetChnControl(chan)->SetChnAxisSoftLimit(chan_axis);
		break;
	case 1520:
	case 1521:
	case 1522:
	case 1523:
	case 1524:
	case 1525:
	case 1526:
	case 1527:
	case 1528:
		this->m_sc_axis_config[axis_index].axis_home_pos[param_no-1520] = value.value_double;
		break;
	case 1529:  //�ο���
		this->m_sc_axis_config[axis_index].axis_home_pos[param_no-1520] = value.value_double;
		chan = g_ptr_chn_engine->GetAxisChannel(axis_index, chan_axis);
		g_ptr_chn_engine->GetChnControl(chan)->SetChnAxisPosThreshold(chan_axis);  //���͸�MC
		break;
	case 1550:  //���ٶ�λ    0--�ر�   1--��
		this->m_sc_axis_config[axis_index].fast_locate = value.value_uint8;
		break;
		
	case 1551:  //λ����ʾģʽ   0--ѭ��ģʽ��0~360��    1--��ѭ��ģʽ
		this->m_sc_axis_config[axis_index].pos_disp_mode = value.value_uint8;
		break;

	case 1602:	//������ٱ�
		this->m_sc_axis_config[axis_index].spd_gear_ratio = value.value_uint8;

		break;
	case 1603:	//��Ư����
		this->m_sc_axis_config[axis_index].zero_compensation = value.value_uint16;

		break;
	case 1604:	//�������ת��
		this->m_sc_axis_config[axis_index].spd_max_speed = value.value_uint32;

		break;
	case 1605: //��������ʱ��
		this->m_sc_axis_config[axis_index].spd_start_time = value.value_uint16;
		UpdateMiParam<uint16_t>(axis_index+1, param_no, value.value_uint16);
		break;
	case 1606: //�����ƶ�ʱ��
		this->m_sc_axis_config[axis_index].spd_stop_time = value.value_uint16;
		UpdateMiParam<uint16_t>(axis_index+1, param_no, value.value_uint16);
		break;
	case 1607:	//�������ת��
		this->m_sc_axis_config[axis_index].spd_min_speed = value.value_uint32;

		break;
	case 1610:	//�����ѹ���Ʒ�ʽ
		this->m_sc_axis_config[axis_index].spd_vctrl_mode = value.value_uint8;

		break;
	case 1612:	//��������ת��(rpm)
		this->m_sc_axis_config[axis_index].spd_set_speed = value.value_uint32;
		break;
	case 1650:  //�Ƿ�ͬ����
		this->m_sc_axis_config[axis_index].sync_axis = value.value_uint8;
		UpdateMiParam<uint8_t>(axis_index+1, param_no, value.value_uint8);
		break;
	case 1651:	//�������
		this->m_sc_axis_config[axis_index].master_axis_no = value.value_uint8;
		UpdateMiParam<uint8_t>(axis_index+1, param_no, value.value_uint8);
		break;
	case 1652:	//��ʾ����
		this->m_sc_axis_config[axis_index].disp_coord = value.value_uint8;
		break;
	case 1653:	//�������׼λ��ƫ��
		this->m_sc_axis_config[axis_index].benchmark_offset = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double); 	//��׼ƫ��

		break;
	case 1655:	//λ��ͬ��������
		this->m_sc_axis_config[axis_index].sync_err_max = value.value_uint32;
		UpdateMiParam<uint32_t>(axis_index+1, param_no, value.value_uint32); 	//�����ͬ��������
		break;
	case 1656:	//�Ӷ���زο�����Զ�ͬ��У׼
		this->m_sc_axis_config[axis_index].auto_sync = value.value_uint8;
		break;
    case 1657:	//SOR�ź���;
        this->m_sc_axis_config[axis_index].spd_ctrl_GST = value.value_uint8;
        break;
    case 1658:	//���ỻ����ʽ
        this->m_sc_axis_config[axis_index].spd_ctrl_SGB = value.value_uint8;
        break;
    case 1659:	//���ֻ���ʱ�Ƿ����SF�ź�
        this->m_sc_axis_config[axis_index].spd_ctrl_SFA = value.value_uint8;
        break;
    case 1660:	//���ᶨ��ʱ��ת��
        this->m_sc_axis_config[axis_index].spd_ctrl_ORM = value.value_uint8;
        break;
    case 1661:	//����ת���Ƿ���M03/M04Ӱ��
        this->m_sc_axis_config[axis_index].spd_ctrl_TCW = value.value_uint8;
        break;
    case 1662:	//����ת��ȡ��
        this->m_sc_axis_config[axis_index].spd_ctrl_CWM = value.value_uint8;
        break;
    case 1663:	//���������͸��Թ�˿ʱ�����ᱶ������
        this->m_sc_axis_config[axis_index].spd_ctrl_TSO = value.value_uint8;
        break;
    case 1664:	//ģ���������
        this->m_sc_axis_config[axis_index].spd_analog_gain = value.value_uint16;
        break;
    case 1665:	//������ֻ���/����ʱ������ת��
        this->m_sc_axis_config[axis_index].spd_sor_speed = value.value_uint16;
        break;
    case 1666:	//��������Сǯ���ٶ�
        this->m_sc_axis_config[axis_index].spd_motor_min_speed = value.value_uint16;
        break;
    case 1667:	//���������ǯ���ٶ�
        this->m_sc_axis_config[axis_index].spd_motor_max_speed = value.value_uint16;
        break;
    case 1668:	//���ֵ͵�λ���ת��
        this->m_sc_axis_config[axis_index].spd_gear_speed_low = value.value_uint16;
        break;
    case 1669:	//�����е�λ���ת��
        this->m_sc_axis_config[axis_index].spd_gear_speed_middle = value.value_uint16;
        break;
    case 1670:	//���ָߵ�λ���ת��
        this->m_sc_axis_config[axis_index].spd_gear_speed_high = value.value_uint16;
        break;
    case 1671:	//B��ʽ��1->��2���ת��
        this->m_sc_axis_config[axis_index].spd_gear_switch_speed1 = value.value_uint16;
        break;
    case 1672:	//B��ʽ��2->��3���ת��
        this->m_sc_axis_config[axis_index].spd_gear_switch_speed2 = value.value_uint16;
        break;
    case 1673:	//��˿ͬ���������
        this->m_sc_axis_config[axis_index].spd_sync_error_gain = value.value_uint32;
        break;
    case 1674:	//��˿���ٶ�ǰ������
        this->m_sc_axis_config[axis_index].spd_speed_feed_gain = value.value_uint32;
        break;
    case 1675:	//��˿��λ�ñ�������
        this->m_sc_axis_config[axis_index].spd_pos_ratio_gain = value.value_uint32;
        break;
	default:
		g_ptr_trace->PrintLog(LOG_ALARM, "�������������ŷǷ���%d", param_no);
		break;
	}
}

/**
 * @brief ��ȡָ��ͨ����ǰ�򿪵�NC�ļ�����
 * @param chn_index : ͨ����
 * @param file[out] : �����ļ������ַ�����������128�ֽ�
 */
void ParmManager::GetCurNcFile(uint8_t chn_index, char *file){
	char sname[32];
	char kname[64];

	memset(sname, 0x00, sizeof(sname));
	sprintf(sname, "channel_%hhu", chn_index);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "cur_nc_file");
	string value, def_value("");
	this->m_ini_chn_scene->GetStringValueOrDefault(sname, kname, &value, def_value);

	int len = value.length();
	if(len >= kMaxFileNameLen)
		len = kMaxFileNameLen-1;
	memset(file, 0x00, kMaxFileNameLen);
	value.copy(file, len, 0);

}

/**
 * @brief ����ָ��ͨ����ǰ�򿪵�NC�ļ�����
 * @param chn_index : ͨ����
 * @param file : �ļ������ַ���
 */
void ParmManager::SetCurNcFile(uint8_t chn_index, char *file){
	char sname[32];
	char kname[64];

	memset(sname, 0x00, sizeof(sname));
	sprintf(sname, "channel_%hhu", chn_index);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "cur_nc_file");
	string value = file;
	this->m_ini_chn_scene->SetStringValue(sname, kname, value);

	this->m_ini_chn_scene->Save();

}

/**
 * @brief ����ָ��ͨ���ĵ�ǰ��������
 * @param chn_index : ͨ����
 * @param piece : ��������
 */
void ParmManager::SetCurWorkPiece(uint8_t chn_index, uint32_t piece){
	char sname[32];
	char kname[64];

	memset(sname, 0x00, sizeof(sname));
	sprintf(sname, "channel_%hhu", chn_index);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "cur_work_piece");
	this->m_ini_chn_scene->SetIntValue(sname, kname, piece);

	this->m_ini_chn_scene->Save();
}

/**
 * @brief ��ȡָ��ͨ���ĵ�ǰ��������
 * @param chn_index �� ͨ����
 * @return ���ع�������
 */
uint32_t ParmManager::GetCurWorkPiece(uint8_t chn_index){
	char sname[32];
	char kname[64];

	memset(sname, 0x00, sizeof(sname));
	sprintf(sname, "channel_%hhu", chn_index);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "cur_work_piece");
	uint32_t value, def_value = 0;
	value = m_ini_chn_scene->GetIntValueOrDefault(sname, kname, def_value);

    return value;
}

/**
 * @brief ��ȡָ��ͨ�����ܹ�����
 * @param chn_index : ͨ����
 * @return piece : ��������
 */
uint32_t ParmManager::GetTotalWorkPiece(uint8_t chn_index)
{
    char sname[32];
    char kname[64];

    memset(sname, 0x00, sizeof(sname));
    sprintf(sname, "channel_%hhu", chn_index);
    memset(kname, 0x00, sizeof(kname));
    sprintf(kname, "cur_total_piece");
    uint32_t value, def_value = 0;
    value = m_ini_chn_scene->GetIntValueOrDefault(sname, kname, def_value);

    return value;
}

/**
 * @brief ����ָ��ͨ�����ܹ�����
 * @param chn_index : ͨ����
 * @param piece : �ܹ�����
 */
void ParmManager::SetTotalWorkPiece(uint8_t chn_index, uint32_t piece)
{
    char sname[32];
    char kname[64];

    memset(sname, 0x00, sizeof(sname));
    sprintf(sname, "channel_%hhu", chn_index);
    memset(kname, 0x00, sizeof(kname));
    sprintf(kname, "cur_total_piece");
    this->m_ini_chn_scene->SetIntValue(sname, kname, piece);

    this->m_ini_chn_scene->Save();
}

/**
 * @brief ����ָ��ͨ�����ۼƼӹ�ʱ��
 * @param chn_index : ͨ����
 * @param piece : �ۼƼӹ�ʱ��
 */
void ParmManager::SetCurTotalMachiningTime(uint8_t chn_index, uint32_t totalTime)
{
    char sname[32];
    char kname[64];

    memset(sname, 0x00, sizeof(sname));
    sprintf(sname, "channel_%hhu", chn_index);
    memset(kname, 0x00, sizeof(kname));
    sprintf(kname, "cur_total_machinetime");
    this->m_ini_chn_scene->SetIntValue(sname, kname, totalTime);

    this->m_ini_chn_scene->Save();
}

/**
 * @brief ��ȡָ��ͨ�����ۼƼӹ�ʱ��
 * @param chn_index �� ͨ����
 * @return �����ۼƼӹ�ʱ��
 */
uint32_t ParmManager::GetCurTotalMachingTime(uint8_t chn_index)
{
    char sname[32];
    char kname[64];

    memset(sname, 0x00, sizeof(sname));
    sprintf(sname, "channel_%hhu", chn_index);
    memset(kname, 0x00, sizeof(kname));
    sprintf(kname, "cur_total_machinetime");
    uint32_t value, def_value = 0;
    value = m_ini_chn_scene->GetIntValueOrDefault(sname, kname, def_value);

    return value;
}

/**
 * @brief ��ȡָ��ͨ���ĵ�ǰ����
 * @param chn_index
 * @return
 */
uint8_t ParmManager::GetCurTool(uint8_t chn_index){
	char sname[32];
	char kname[64];

	memset(sname, 0x00, sizeof(sname));
	sprintf(sname, "channel_%hhu", chn_index);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "cur_tool");
	uint8_t value, def_value = 0;
	value = m_ini_chn_scene->GetIntValueOrDefault(sname, kname, def_value);

	return value;
}

/**
 * @brief ����ָ��ͨ���ĵ�ǰ����
 * @param chn_index
 * @param tool
 */
void ParmManager::SetCurTool(uint8_t chn_index, uint8_t tool){
	char sname[32];
	char kname[64];

	memset(sname, 0x00, sizeof(sname));
	sprintf(sname, "channel_%hhu", chn_index);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "cur_tool");
	this->m_ini_chn_scene->SetIntValue(sname, kname, tool);

	this->m_ini_chn_scene->Save();
}

/**
 * @brief ��ȡָ��ͨ���ĵ�ǰ���ղ������
 * @param chn_index �� ͨ���ţ���0��ʼ
 * @return ���ص�ǰ���ղ������
 */
uint8_t ParmManager::GetCurProcParamIndex(uint8_t chn_index){
	char sname[32];
	char kname[64];

	memset(sname, 0x00, sizeof(sname));
	sprintf(sname, "channel_%hhu", chn_index);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "cur_proc_index");
	uint8_t value, def_value = 0;
	value = m_ini_chn_scene->GetIntValueOrDefault(sname, kname, def_value);

	return value;
}

/**
 * @brief ����ָ��ͨ���ĵ�ǰ���ղ������
 * @param chn_index �� ͨ���ţ���0��ʼ
 * @param proc_index �� ���ղ������
 */
void ParmManager::SetCurProcParamIndex(uint8_t chn_index, uint8_t proc_index){
	char sname[32];
	char kname[64];

	memset(sname, 0x00, sizeof(sname));
	sprintf(sname, "channel_%hhu", chn_index);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "cur_proc_index");
	this->m_ini_chn_scene->SetIntValue(sname, kname, proc_index);

	this->m_ini_chn_scene->Save();
}

/**
 * @brief ���ĵ�ǰ������ز���������Ӧ���ղ���������ø��Ƶ���ǰͨ���������������
 * @param chn_index �� ͨ���ţ���0��ʼ
 * @param proc_index �� ���ղ�����ţ���0��ʼ
 */
void ParmManager::ChangeChnProcParamIndex(uint8_t chn_index, uint8_t proc_index){
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	//����ͨ������
	sprintf(sname, "channel_%hhu", chn_index);
	sprintf(kname, "rapid_mode");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].rapid_mode);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "cut_plan_mode");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].cut_plan_mode);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "rapid_plan_mode");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].rapid_plan_mode);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "corner_acc_limit");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].corner_acc_limit);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "corner_stop");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].corner_stop);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "corner_stop_angle_min");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].corner_stop_angle_min);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "corner_stop_enable");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].corner_stop_enable);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "chn_spd_limit_on_acc");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].chn_spd_limit_on_acc);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "chn_spd_limit_on_axis");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].chn_spd_limit_on_axis);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "chn_spd_limit_on_curvity");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].chn_spd_limit_on_curvity);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "chn_rapid_overlap_level");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].chn_rapid_overlap_level);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "chn_s_cut_filter_time");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].chn_s_cut_filter_time);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "chn_small_line_time");
	m_ini_chn->SetIntValue(sname, kname, m_sc_channel_config[chn_index].chn_small_line_time);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "chn_max_vel");
	m_ini_chn->SetDoubleValue(sname, kname, this->m_sc_channel_config[chn_index].chn_max_vel);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "chn_max_acc");
	m_ini_chn->SetDoubleValue(sname, kname, this->m_sc_channel_config[chn_index].chn_max_acc);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "chn_max_dec");
	m_ini_chn->SetDoubleValue(sname, kname, this->m_sc_channel_config[chn_index].chn_max_dec);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "chn_max_corner_acc");
	m_ini_chn->SetDoubleValue(sname, kname, this->m_sc_channel_config[chn_index].chn_max_corner_acc);
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "chn_max_arc_acc");
	m_ini_chn->SetDoubleValue(sname, kname, this->m_sc_channel_config[chn_index].chn_max_arc_acc);

#ifdef USES_WOOD_MACHINE
	//���ǲ���ֵ
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "flip_comp_value");
	m_ini_chn->SetIntValue(sname, kname, this->m_sc_channel_config[chn_index].flip_comp_value);
#endif

	m_ini_chn->Save();

	//�л������
	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	uint8_t phy_axis = 0;
	for(uint8_t i = 0; i < this->m_sc_channel_config[chn_index].chn_axis_count; i++){
		phy_axis = chn_engine->GetChnControl(chn_index)->GetPhyAxis(i);
		if(phy_axis == 0xFF)
			continue;

		memset(sname, 0x00, sizeof(sname));
		sprintf(sname, "axis_%hhu", phy_axis+1);
		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "rapid_acc");
		m_ini_axis->SetDoubleValue(sname, kname, this->m_sc_axis_config[phy_axis].rapid_acc);
		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "manual_acc");
		m_ini_axis->SetDoubleValue(sname, kname, this->m_sc_axis_config[phy_axis].manual_acc);
		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "start_acc");
		m_ini_axis->SetDoubleValue(sname, kname, this->m_sc_axis_config[phy_axis].start_acc);
		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "corner_acc_limit");
		m_ini_axis->SetDoubleValue(sname, kname, this->m_sc_axis_config[phy_axis].corner_acc_limit);
		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "rapid_s_plan_filter_time");
		m_ini_axis->SetIntValue(sname, kname, this->m_sc_axis_config[phy_axis].rapid_s_plan_filter_time);
		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "post_filter_type");
		m_ini_axis->SetIntValue(sname, kname, this->m_sc_axis_config[phy_axis].post_filter_type);
		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "post_filter_time_1");
		m_ini_axis->SetIntValue(sname, kname, this->m_sc_axis_config[phy_axis].post_filter_time_1);
		memset(kname, 0x00, sizeof(kname));
		sprintf(kname, "post_filter_time_2");
		m_ini_axis->SetIntValue(sname, kname, this->m_sc_axis_config[phy_axis].post_filter_time_2);

	}
	m_ini_axis->Save();

}

/**
 * @brief ��ȡPMC�����
 * @return pmc��ĸ���
 */
uint8_t ParmManager::GetPmcAxisCount(){
	uint8_t count = 0;
	for(uint8_t i = 0; i < this->m_sc_system_config->axis_count; i++){
		if(this->m_sc_axis_config[i].axis_pmc != 0)
			count++;
	}
	return count;
}
