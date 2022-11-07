/*
 * parm_manager.cpp
 *
 *  Created on: 2020年5月6日
 *      Author: M
 */
/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file parm_manager.cpp
 *@author gonghao
 *@date 2020/05/06
 *@brief 本源文件为参数管理类的实现
 *@version
 */

#include "channel_engine.h"
#include "channel_control.h"
#include "parm_manager.h"
#include "global_definition.h"
#include "mi_communication.h"


//template<> int ListNode<ParamUpdate>::new_count = 0;
//template<> int ListNode<CoordUpdate>::new_count = 0;



ParmManager* ParmManager::m_p_instance = nullptr;  //初始化单例对象指正为空

/**
 * @brief  构造函数
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


	m_ini_system = new IniFile();		//系统配置文件
	m_ini_chn = new IniFile();			//通道配置文件
	m_ini_axis = new IniFile();			//轴配置文件
	m_ini_tool = new IniFile();			//刀具配置文件  包括刀具偏置和刀位信息
	m_ini_coord = new IniFile();		//工件坐标系配置文件
	m_ini_ex_coord = new IniFile();		//扩展工件坐标系配置文件
	m_ini_chn_scene = new IniFile();	//通达状态文件
	m_ini_pc_table = new IniFile();     //螺补数据文件
	m_ini_io_remap = new IniFile();     //IO重定向文件

	m_ini_proc_chn = new IniFile();     //工艺相关通道参数文件
	m_ini_proc_axis = new IniFile();    //工艺相关轴参数文件
    m_ini_handwheel_map = new IniFile;  //手轮通道映射关系

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
	m_ini_five_axis = new IniFile();   //五轴配置文件
#endif

#ifdef USES_GRIND_MACHINE
	this->m_grind_config = nullptr;
	this->m_ini_grind = new IniFile();	//磨削配置文件
	this->m_b_update_grind = false;
#endif
}

/**
 * @brief  析构函数
 */
ParmManager::~ParmManager(){
	//释放参数更新队列
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

	//释放工件坐标系配置
	if(m_sc_coord_config != nullptr){
		delete []m_sc_coord_config;
		m_sc_coord_config = nullptr;
	}

	//释放工件坐标系配置
	for(int i = 0; i < kMaxChnCount; i++)
		if(m_sc_ex_coord_config[i] != nullptr){
			delete []m_sc_ex_coord_config[i];
			m_sc_ex_coord_config[i] = nullptr;
		}

	//释放轴配置
	if(m_sc_axis_config != nullptr){
		delete []m_sc_axis_config;
		m_sc_axis_config = nullptr;
	}

	//释放刀具配置
	if(m_sc_tool_config != nullptr){
		delete []m_sc_tool_config;
		m_sc_tool_config = nullptr;
	}
	if(m_sc_tool_pot_config != nullptr){
		delete []m_sc_tool_pot_config;
		m_sc_tool_pot_config = nullptr;
	}


	//释放通道配置
	if(m_sc_channel_config != nullptr){
		delete []m_sc_channel_config;
		m_sc_channel_config = nullptr;
	}

	//释放系统配置
	if(m_sc_system_config != nullptr){
		delete m_sc_system_config;
		m_sc_system_config = nullptr;
	}

	//释放螺补数据
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


	//释放INI文件对象
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
	//释放磨削配置
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
 * @brief 获取类实例的唯一访问接口函数，单例模式
 */
ParmManager* ParmManager::GetInstance(){
	if(nullptr == m_p_instance){
		m_p_instance = new ParmManager();
	}
	return m_p_instance;
}

/**
 * @brief  初始化各变量
 */
void ParmManager::InitParm(){

	//初始化系统参数
	ReadParm(SYS_CONFIG);

	//初始化轴配置
	ReadParm(AXIS_CONFIG);

	//初始化通道配置
	ReadParm(CHN_CONFIG);

	//初始化刀具配置
	ReadParm(TOOL_OFFSET_CONFIG);

	//初始化工件坐标系配置
	ReadParm(COORD_CONFIG);

	//初始化扩展工件坐标系配置
	ReadParm(EX_COORD_CONFIG);

	//初始化通道状态
	ReadParm(CHN_STATE_SCENE);

	//初始化螺补参数
	ReadParm(PITCH_COMP_DATA);

	//初始化工艺参数
	ReadParm(PROCESS_PARAM);

#ifdef USES_FIVE_AXIS_FUNC
	//初始化五轴参数配置
	ReadParm(FIVE_AXIS_CONFIG);
#endif

#ifdef USES_GRIND_MACHINE
	//初始化磨床配置
	ReadParm(GRIND_CONFIG);
#endif

	//初始化IO重映射数据
	this->ReadParm(IO_REMAP_DATA);

    //初始化手轮通道映射
    this->ReadParm(HANDWHEEL_MAP);
}


/**
 * @brief  从配置文件中读取参数
 * @param type : 参数类型
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
 * @brief 读取系统配置
 * @return  true--成功   false--失败
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


	if(m_ini_system->Load(SYS_CONFIG_FILE) == 0){//读取配置成功
		m_sc_system_config->cnc_mode = 1;
		m_sc_system_config->max_chn_count = kMaxChnCount;
		m_sc_system_config->max_axis_count = kMaxAxisNum;     //12;
        //m_sc_system_config->chn_count = m_ini_system->GetIntValueOrDefault("system", "chn_count", 1);
        m_sc_system_config->chn_count = 1; //目前SC只支持单通道
		m_sc_system_config->axis_count = m_ini_system->GetIntValueOrDefault("system", "axis_count", 4);
		m_sc_system_config->axis_name_ex = m_ini_system->GetIntValueOrDefault("system", "axis_name_ex", 0);
		m_sc_system_config->bus_cycle = m_ini_system->GetIntValueOrDefault("system", "bus_cycle", 3);
		m_sc_system_config->fix_ratio_find_ref = m_ini_system->GetIntValueOrDefault("system", "fix_ratio_find_ref", 1);
//		m_sc_system_config->pc_type = m_ini_system->GetIntValueOrDefault("system", "pc_type", 1);
		
		m_sc_system_config->hw_code_type = m_ini_system->GetIntValueOrDefault("system", "hw_code_type", 0);    //手轮编码方式     0--二进制编码    1--格雷码

		m_sc_system_config->io_filter_time = m_ini_system->GetIntValueOrDefault("system", "io_filter_time", 1000);	//us
		m_sc_system_config->fast_io_filter_time = m_ini_system->GetIntValueOrDefault("system", "fast_io_filter_time", 500); 	//us
		m_sc_system_config->free_space_limit = m_ini_system->GetIntValueOrDefault("system", "free_space_limit", 10240);	//KB
		m_sc_system_config->backlight_delay_time = m_ini_system->GetIntValueOrDefault("system", "backlight_delay_time", 0);	//秒
		m_sc_system_config->save_lineno_poweroff = m_ini_system->GetIntValueOrDefault("system", "save_lineno_poweroff", 0);
        m_sc_system_config->manual_ret_ref_mode = m_ini_system->GetIntValueOrDefault("system", "manual_ret_ref_mode", 0);
		m_sc_system_config->beep_time = m_ini_system->GetIntValueOrDefault("system", "beep_time", 15);		//s
		m_sc_system_config->da_ocp = m_ini_system->GetIntValueOrDefault("system", "da_ocp", 0);
		m_sc_system_config->da_prec = m_ini_system->GetIntValueOrDefault("system", "da_prec", 16);


		m_sc_system_config->alarm_temperature = m_ini_system->GetIntValueOrDefault("system", "alarm_temperature", 70);
		m_sc_system_config->trace_level = m_ini_system->GetIntValueOrDefault("system", "trace_level", 0);
		m_sc_system_config->debug_mode = m_ini_system->GetIntValueOrDefault("system", "debug_mode", 0);

        m_sc_system_config->hw_rev_trace = m_ini_system->GetIntValueOrDefault("system", "hw_rev_trace", 0);

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取系统配置文件成功！\n");

	}else{
		if(m_ini_system->CreateFile(SYS_CONFIG_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认系统配置文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

		//加载默认值
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

		m_sc_system_config->hw_code_type = 0;    //手轮编码方式     0--二进制编码    1--格雷码

		m_sc_system_config->io_filter_time = 1000;	//us
		m_sc_system_config->fast_io_filter_time = 500; 	//us
		m_sc_system_config->free_space_limit = 10240;	//KB
		m_sc_system_config->backlight_delay_time = 0;	//秒
		m_sc_system_config->save_lineno_poweroff = 0;
		m_sc_system_config->manual_ret_ref_mode = 0;
		m_sc_system_config->beep_time = 15;		//s
		m_sc_system_config->da_ocp = 0;
		m_sc_system_config->da_prec = 16;


		m_sc_system_config->alarm_temperature = 70;
		m_sc_system_config->trace_level = 0;
		m_sc_system_config->debug_mode = 0;
		m_sc_system_config->hw_rev_trace = 0;

		//生成默认ini配置
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

		m_ini_system->AddKeyValuePair(string("hw_code_type"), string("0"), ns);    //手轮编码方式     0--二进制编码    1--格雷码

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

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认系统配置文件成功！\n");
	}
END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}
	return true;
}

/**
 * @brief 读取通道配置
 * @return  true--成功   false--失败
 */
bool ParmManager::ReadChnConfig(){
	int err_code = ERR_NONE;
	char sname[20];
	char kname[30];
	char value[10];

	int i = 0, j = 0;
	IniSection *ns = nullptr;

	if(m_sc_channel_config == nullptr){
//		m_sc_channel_config = new SCChannelConfig[m_sc_system_config->chn_count];//根据通道数创建
		m_sc_channel_config = new SCChannelConfig[m_sc_system_config->max_chn_count];//根据最大通道数创建，为了支持参数修一次重启
	}

	if(m_sc_channel_config == nullptr || m_ini_chn == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_chn->Load(CHN_CONFIG_FILE) == 0){//读取配置成功

	//	for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){	  //支持参数修改一次重启
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			m_sc_channel_config[i].chn_index = i;
			m_sc_channel_config[i].chn_axis_count = m_ini_chn->GetIntValueOrDefault(sname, "chn_axis_count", 4);
			m_sc_channel_config[i].chn_group_index = m_ini_chn->GetIntValueOrDefault(sname, "chn_group_index", 0);
//			m_sc_channel_config[i].chn_axis_x = m_ini_chn->GetIntValueOrDefault(sname, "chn_axis_x", 0);    //默认值为0，即未配置
//			m_sc_channel_config[i].chn_axis_y = m_ini_chn->GetIntValueOrDefault(sname, "chn_axis_y", 0);    //默认值为0，即未配置
//			m_sc_channel_config[i].chn_axis_z = m_ini_chn->GetIntValueOrDefault(sname, "chn_axis_z", 0);    //默认值为0，即未配置
			for(j = 0; j < kMaxAxisChn; j++){
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_axis_%d", j+1);
				m_sc_channel_config[i].chn_axis_phy[j] = m_ini_chn->GetIntValueOrDefault(sname, kname, 0);    //默认值为0，即未配置
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

			m_sc_channel_config[i].cut_plan_mode = m_ini_chn->GetIntValueOrDefault(sname, "cut_plan_mode", 1);	//默认S型
			m_sc_channel_config[i].rapid_plan_mode = m_ini_chn->GetIntValueOrDefault(sname, "rapid_plan_mode", 1);	//默认S型

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
			m_sc_channel_config[i].debug_param_1 = m_ini_chn->GetIntValueOrDefault(sname, "debug_param_1", 0);  //调试参数1
			m_sc_channel_config[i].debug_param_2 = m_ini_chn->GetIntValueOrDefault(sname, "debug_param_2", 0);  //调试参数2
			m_sc_channel_config[i].debug_param_3 = m_ini_chn->GetIntValueOrDefault(sname, "debug_param_3", 0);  //调试参数3
			m_sc_channel_config[i].debug_param_4 = m_ini_chn->GetIntValueOrDefault(sname, "debug_param_4", 0);  //调试参数4
			m_sc_channel_config[i].debug_param_5 = m_ini_chn->GetIntValueOrDefault(sname, "debug_param_5", 0);  //调试参数5
			
			m_sc_channel_config[i].flip_comp_value = m_ini_chn->GetIntValueOrDefault(sname, "flip_comp_value", 0);  //挑角补偿值
#endif

		}
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取通道配置文件成功！\n");

	}
	else{
		if(m_ini_chn->CreateFile(CHN_CONFIG_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认通道配置文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}


//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){	  //支持参数修改一次重启
			//加载默认值
			m_sc_channel_config[i].chn_index = i;
			m_sc_channel_config[i].chn_axis_count = 4;
			m_sc_channel_config[i].chn_group_index = 0;  //默认为第一方式组
//			m_sc_channel_config[i].chn_axis_x = 0;    //默认值为0，即未配置
//			m_sc_channel_config[i].chn_axis_y = 0;    //默认值为0，即未配置
//			m_sc_channel_config[i].chn_axis_z = 0;    //默认值为0，即未配置
			for(j = 0; j < kMaxAxisChn; j++){
				m_sc_channel_config[i].chn_axis_phy[j] = 0;    //默认值为0，即未配置
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

			m_sc_channel_config[i].cut_plan_mode = 1;	//默认S型
			m_sc_channel_config[i].rapid_plan_mode = 1;	//默认S型

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
			m_sc_channel_config[i].debug_param_1 = 0;  //调试参数1
			m_sc_channel_config[i].debug_param_2 = 0;  //调试参数2
			m_sc_channel_config[i].debug_param_3 = 0;  //调试参数3
			m_sc_channel_config[i].debug_param_4 = 0;  //调试参数4
			m_sc_channel_config[i].debug_param_5 = 0;  //调试参数5
			
			m_sc_channel_config[i].flip_comp_value = 0;   //挑角补偿值
#endif

			//生成默认ini配置
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			ns = m_ini_chn->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			m_ini_chn->AddKeyValuePair(string("chn_axis_count"), string("4"), ns);
			m_ini_chn->AddKeyValuePair(string("chn_group_index"), string("0"), ns);    //默认第一方式组   增加方式组参数
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
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认通道配置文件成功！\n");

	}


	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief 读取工艺相关通道参数
 * @return true--成功  false--失败
 */
bool ParmManager::ReadChnProcParam(){
	int err_code = ERR_NONE;
	char sname[20];
	char kname[30];

	int i = 0, j = 0;
	IniSection *ns = nullptr;

	if(this->m_p_chn_process_param == nullptr){
//		m_p_chn_process_param = new ChnProcParamGroup[m_sc_system_config->chn_count];//根据通道数创建
		m_p_chn_process_param = new ChnProcParamGroup[m_sc_system_config->max_chn_count];//根据最大通道数创建，支持参数修改一次重启
	}

	if(m_p_chn_process_param == nullptr || m_ini_proc_chn == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_proc_chn->Load(CHN_PROC_PARAM_FILE) == 0){//读取配置成功

//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){	  //支持参数修改一次重启
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);

			for(j = 0; j < kMaxProcParamCount; j++){  //遍历所有工艺参数组
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
				m_p_chn_process_param[i].chn_param[j].chn_spd_limit_on_acc = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);	//默认打开

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_spd_limit_on_axis_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_spd_limit_on_axis = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);	//默认打开

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_spd_limit_on_curvity_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_spd_limit_on_curvity = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);  //默认打开

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_acc_limit_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_acc_limit = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);   //默认打开

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_stop_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_stop = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 45);	//默认45°

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_stop_angle_min_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_stop_angle_min = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_stop_enable_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_stop_enable = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "cut_plan_mode_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].cut_plan_mode = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);	//默认S型

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "rapid_mode_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].rapid_mode = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 0);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "rapid_plan_mode_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].rapid_plan_mode = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 1);	//默认S型
				
#ifdef USES_WOOD_MACHINE
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "flip_comp_value_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].flip_comp_value = m_ini_proc_chn->GetIntValueOrDefault(sname, kname, 0);	//默认0
#endif
			}

		}
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取通道工艺相关参数文件成功！\n");

	}else{
		if(m_ini_proc_chn->CreateFile(CHN_PROC_PARAM_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认通道工艺相关参数参数文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}


//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){	  //支持参数修改一次重启
			//生成默认ini配置
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			ns = m_ini_proc_chn->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			for(j = 0; j < kMaxProcParamCount; j++){  //遍历所有工艺参数组
				//加载默认值
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
				m_p_chn_process_param[i].chn_param[j].chn_spd_limit_on_acc = 1;	//默认打开
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_spd_limit_on_axis_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_spd_limit_on_axis = 1;	//默认打开
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "chn_spd_limit_on_curvity_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].chn_spd_limit_on_curvity = 1;  //默认打开
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_acc_limit_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_acc_limit = 1;   //默认打开
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "corner_stop_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].corner_stop = 45;	//默认45°
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
				m_p_chn_process_param[i].chn_param[j].cut_plan_mode = 1;	//默认S型
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "rapid_mode_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].rapid_mode = 0;
				m_ini_proc_chn->AddKeyValuePair(kname, string("0"), ns);

				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "rapid_plan_mode_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].rapid_plan_mode = 1;	//默认S型
				m_ini_proc_chn->AddKeyValuePair(kname, string("1"), ns);

#ifdef USES_WOOD_MACHINE
				memset(kname, 0x00, sizeof(kname));
				sprintf(kname, "flip_comp_value_%d", j+1);
				m_p_chn_process_param[i].chn_param[j].flip_comp_value = 0;	//默认0
				m_ini_proc_chn->AddKeyValuePair(kname, string("0"), ns);
#endif
			}

		}

		m_ini_proc_chn->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认通道工艺相关参数文件成功！\n");

	}


	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}
/**
 * @brief 读取工艺相关轴参数
 * @return   true--成功   false--失败
 */
bool ParmManager::ReadAxisProcParam(){
	//读取工艺相关通道参数
	int err_code = ERR_NONE;
	char sname[20];
	char kname[30];

	int i = 0, j = 0;
	IniSection *ns = nullptr;

	if(this->m_p_axis_process_param == nullptr){
//		m_p_axis_process_param = new AxisProcParamGroup[m_sc_system_config->axis_count];//根据物理轴数创建
		m_p_axis_process_param = new AxisProcParamGroup[m_sc_system_config->max_axis_count];//根据最大物理轴数创建，支持参数修改后一次重启
	}

	if(m_p_axis_process_param == nullptr || m_ini_proc_axis == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_proc_axis->Load(AXIS_PROC_PARAM_FILE) == 0){//读取配置成功

		for(i = 0; i < m_sc_system_config->max_axis_count; i++){  //遍历所有物理轴， 支持参数修改后一次重启
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "axis_%d", i+1);

			for(j = 0; j < kMaxProcParamCount; j++){  //遍历所有工艺参数组
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
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取轴工艺相关参数文件成功！\n");

	}
	else{
		if(m_ini_proc_axis->CreateFile(AXIS_PROC_PARAM_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认轴工艺相关参数参数文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}


		for(i = 0; i < m_sc_system_config->max_axis_count; i++){  //遍历所有物理轴， 支持参数修改后一次重启
			//生成默认ini配置
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "axis_%d", i+1);
			ns = m_ini_proc_axis->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			for(j = 0; j < kMaxProcParamCount; j++){  //遍历所有工艺参数组
				//加载默认值
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
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认轴工艺相关参数文件成功！\n");

	}


	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

    return true;
}

/**
 * @brief 读取手轮通道映射参数
 * @return  true--成功   false--失败
 */
bool ParmManager::ReadHandWheelParam()
{
    if (m_ini_handwheel_map == nullptr){
        return false;
    }
    if(m_ini_handwheel_map->Load(HANDWHEEL_CONFIG_FILE) == 0){//读取配置成功
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
 * @brief 读取轴配置
 * @return  true--成功   false--失败
 */
bool ParmManager::ReadAxisConfig(){
	int err_code = ERR_NONE;
	char sname[32];
	char kname[64];
	char value[16];

	int i = 0, j = 0;

	IniSection *ns = nullptr;

	if(m_sc_axis_config == nullptr){
//		m_sc_axis_config = new SCAxisConfig[m_sc_system_config->axis_count];//根据通道数创建
		m_sc_axis_config = new SCAxisConfig[m_sc_system_config->max_axis_count];//根据通道数创建，支持参数修改后一次重启
	}

	if(m_sc_axis_config == nullptr || m_ini_axis == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_axis->Load(AXIS_CONFIG_FILE) == 0){//读取配置成功
		for(i = 0; i < m_sc_system_config->max_axis_count; i++){
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "axis_%d", i+1);   //从1开始
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
			m_sc_axis_config[i].encoder_max_cycle = m_ini_axis->GetIntValueOrDefault(sname, "encoder_max_cycle", 0);  //最大圈数默认值由256改为0

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
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取轴配置文件成功！\n");
	}
	else {
		if(m_ini_axis->CreateFile(AXIS_CONFIG_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认轴配置文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

		for(i = 0; i < m_sc_system_config->max_axis_count; i++){
			//加载默认值
			m_sc_axis_config[i].axis_index = i;
			m_sc_axis_config[i].axis_type = 0;	//默认直线轴
			m_sc_axis_config[i].axis_interface = 0;	//默认虚拟轴
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
			m_sc_axis_config[i].encoder_max_cycle = 0;   //最大圈数默认值由256改为0

			m_sc_axis_config[i].sync_axis = 0;
			m_sc_axis_config[i].master_axis_no = 0;
			m_sc_axis_config[i].disp_coord = 0;
			m_sc_axis_config[i].benchmark_offset = 0;
			m_sc_axis_config[i].sync_err_max = 0;
			m_sc_axis_config[i].auto_sync = 0;

			for(j = 0; j < 10; j++){
				m_sc_axis_config[i].axis_home_pos[j] = 0.0;
			}

			//生成默认ini配置
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "axis_%d", i+1);    //从1开始
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
			m_ini_axis->AddKeyValuePair(string("encoder_max_cycle"), string("0"), ns);  //最大圈数默认值由256改为0

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
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认轴配置文件成功！\n");

	}

	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief 读取刀具配置
 * @return  true--成功   false--失败
 */
bool ParmManager::ReadToolConfig(){
	int err_code = ERR_NONE;
	char sname[32];
	char kname[64];
	char value[32];

	int i = 0, j = 0;

	IniSection *ns=nullptr;

	if(m_sc_tool_config == nullptr){
//		m_sc_tool_config = new SCToolOffsetConfig[m_sc_system_config->chn_count];  //根据通道数创建
//		m_sc_tool_pot_config = new SCToolPotConfig[m_sc_system_config->chn_count];

		m_sc_tool_config = new SCToolOffsetConfig[m_sc_system_config->max_chn_count];  //根据最大通道数创建，支持参数修改一次重启
		m_sc_tool_pot_config = new SCToolPotConfig[m_sc_system_config->max_chn_count];
	}

	if(m_sc_tool_config == nullptr || m_sc_tool_pot_config == nullptr || m_ini_tool == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_tool->Load(TOOL_CONFIG_FILE) == 0){//读取配置成功
//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //支持参数修改一次重启
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);

#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
			//读取基准刀数据
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

				//刀位信息
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

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取刀具配置文件成功！\n");
	}
	else {
		if(m_ini_tool->CreateFile(TOOL_CONFIG_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认刀具配置文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

        for(i = 0; i < m_sc_system_config->chn_count; i++){
        //for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //支持参数修改一次重启
			//生成默认ini配置
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

				//刀位信息
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
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认刀具配置文件成功！\n");

	}

	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief 读取工件坐标系配置
 * @return  true--成功   false--失败
 */
bool ParmManager::ReadCoordConfig(){
	int err_code = ERR_NONE;
	char sname[32];
	char kname[64];


	int i = 0, j = 0, k = 0;

	IniSection *ns=nullptr;

	if(m_sc_coord_config == nullptr){
//		m_sc_coord_config = new SCCoordConfig[m_sc_system_config->chn_count * kWorkCoordCount];  //根据通道数创建
		m_sc_coord_config = new SCCoordConfig[m_sc_system_config->max_chn_count * kWorkCoordCount];  //根据最大通道数创建，支持参数修改一次重启
	}

	if(m_sc_coord_config == nullptr || m_ini_coord == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}

	if(m_ini_coord->Load(WORK_COORD_FILE) == 0){//读取配置成功
//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //支持参数修改一次重启
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
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取工件坐标系配置文件成功！\n");
	}
	else {
		if(m_ini_coord->CreateFile(WORK_COORD_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认工件坐标系配置文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //支持参数修改一次重启
			//生成默认ini配置
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
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认工件坐标系配置文件成功！\n");
	}

	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief 读取扩展工件坐标系配置
 * @return true--成功   false--失败
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
	for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //支持参数修改一次重启
		if(m_sc_channel_config[i].ex_coord_count > 0 && m_sc_ex_coord_config[i] == nullptr){
			m_sc_ex_coord_config[i] = new SCCoordConfig[m_sc_channel_config[i].ex_coord_count];	//根据通道数创建

			if(m_sc_ex_coord_config[i] == nullptr){
				err_code = ERR_SC_INIT;

				goto END;
			}
		}
	}


	if(m_ini_ex_coord->Load(EX_WORK_COORD_FILE) == 0){//读取配置成功

//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //支持参数修改一次重启
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

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取扩展工件坐标系配置文件成功！\n");
	}
	else {
		if(m_ini_ex_coord->CreateFile(EX_WORK_COORD_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认扩展工件坐标系配置文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){  //支持参数修改一次重启
			//生成默认ini配置
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
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认扩展工件坐标系配置文件成功！\n");

	}

	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief 读取IO重定向数据
 * @return true--成功   false--失败
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


	if(m_ini_io_remap->Load(IO_REMAP_FILE) == 0){//读取配置成功

		sec_array.clear();
		total_count = m_ini_io_remap->GetSections(&sec_array);   //读取总数

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

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取IO重定向配置文件成功！\n");
	}else {
		if(m_ini_io_remap->CreateFile(IO_REMAP_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认IO重定向配置文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

		m_ini_ex_coord->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认IO重定向配置文件成功！\n");

	}

	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief 读取通道状态配置
 * @return
 */
bool ParmManager::ReadChnStateScene(){
	int err_code = ERR_NONE;
	char sname[32];
	char kname[64];

	if(m_ini_chn_scene->Load(CHN_SCENE_FILE) == 0){//读取配置成功

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取通道状态文件成功！\n");
	}
	else {
		if(m_ini_chn_scene->CreateFile(CHN_SCENE_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认通道状态文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

		IniSection *ns=nullptr;
//		for(int i = 0; i < m_sc_system_config->chn_count; i++){
		for(int i = 0; i < m_sc_system_config->max_chn_count; i++){  //支持参数修改一次重启
			//生成默认ini配置
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

			//当前工艺参数组号
			memset(kname, 0x00, sizeof(kname));
			sprintf(kname, "cur_proc_index");
			m_ini_chn_scene->AddKeyValuePair(kname, string("0"), ns);

		}

		m_ini_chn_scene->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认通道状态文件成功！\n");

	}
	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief 读取螺补数据
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

	if(m_ini_pc_table->Load(PITCH_COMP_FILE) == 0){//读取配置成功

		for(i = 0; i < m_sc_system_config->axis_count; i++){
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "axis_%d", i+1);   //从1开始

			if(m_sc_pc_table->pc_table[i] == nullptr){

				if(m_sc_axis_config[i].pc_type == 0){

					m_sc_pc_table->pc_table[i] = new double[m_sc_axis_config[i].pc_count];      //单向螺补
				}else if(m_sc_axis_config[i].pc_type == 1){
					m_sc_pc_table->pc_table[i] = new double[m_sc_axis_config[i].pc_count*2];    //双向螺补
				}

				if(m_sc_pc_table->pc_table[i] == nullptr){

					err_code = ERR_SC_INIT;

					goto END;
				}
			}



			if(m_sc_axis_config[i].pc_type == 0){
				for(j = 0; j < this->m_sc_axis_config[i].pc_count; j++){
					memset(kname, 0x00, sizeof(kname));
					sprintf(kname, "pc_pos_%d", j+1);    //正向螺补
					m_sc_pc_table->pc_table[i][j] = m_ini_pc_table->GetDoubleValueOrDefault(sname, kname, 0.0);
				}
			}else if(m_sc_axis_config[i].pc_type == 1){
				for(j = 0; j < this->m_sc_axis_config[i].pc_count; j++){
					memset(kname, 0x00, sizeof(kname));
					sprintf(kname, "pc_pos_%d", j+1);    //正向螺补
					m_sc_pc_table->pc_table[i][j] = m_ini_pc_table->GetDoubleValueOrDefault(sname, kname, 0.0);
					sprintf(kname, "pc_neg_%d", j+1);    //负向螺补
					m_sc_pc_table->pc_table[i][j+m_sc_axis_config[i].pc_count] = m_ini_pc_table->GetDoubleValueOrDefault(sname, kname, 0.0);

//					if(i== 0 && j == this->m_sc_axis_config[i].pc_count-1){
//						printf("axis0, pos=%lf, neg=%lf\n", m_sc_pc_table->pc_table[i][j], m_sc_pc_table->pc_table[i][j+m_sc_axis_config[i].pc_count]);
//					}
				}
			}
		}
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取轴螺补数据文件成功！\n");
	}
	else {
		if(m_ini_pc_table->CreateFile(PITCH_COMP_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认螺补数据文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

		for(i = 0; i < m_sc_system_config->axis_count; i++){

			if(m_sc_pc_table->pc_table[i] == nullptr){
				if(m_sc_axis_config[i].pc_type == 0)
					m_sc_pc_table->pc_table[i] = new double[m_sc_axis_config[i].pc_count];      //单向螺补
				else
					m_sc_pc_table->pc_table[i] = new double[m_sc_axis_config[i].pc_count*2];    //双向螺补

				if(m_sc_pc_table->pc_table[i] == nullptr){
					err_code = ERR_SC_INIT;

					goto END;
				}
			}

			//加载默认值，均为0
			if(m_sc_axis_config[i].pc_type == 0){//单向螺补
				memset(m_sc_pc_table->pc_table[i], 0x00, sizeof(double)*m_sc_axis_config[i].pc_count);
			}else{//双向螺补
				memset(m_sc_pc_table->pc_table[i], 0x00, sizeof(double)*m_sc_axis_config[i].pc_count*2);
			}


			//生成默认ini配置
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "axis_%d", i+1);    //从1开始
			ns = m_ini_pc_table->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			if(m_sc_axis_config[i].pc_type == 0){//单向螺补
				for(j = 0; j < this->m_sc_axis_config[i].pc_count; j++){
					memset(kname, 0x00, sizeof(kname));
					sprintf(kname, "pc_pos_%d", j+1);
					m_ini_pc_table->AddKeyValuePair(kname, string("0.0"), ns);
				}
			}else{//双向螺补
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
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认螺补数据文件成功！\n");

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
 * @brief 读取五轴配置
 * @return
 */
bool ParmManager::ReadFiveAxisConfig(){
	int err_code = ERR_NONE;

	char sname[20];

	int i = 0;
	IniSection *ns = nullptr;

	if(m_five_axis_config == nullptr){
//		m_five_axis_config = new FiveAxisConfig[m_sc_system_config->chn_count]();//根据通道数创建;
		m_five_axis_config = new FiveAxisConfig[m_sc_system_config->max_chn_count]();//根据最大通道数创建，支持参数修改一次重启
	}

	if(m_five_axis_config == nullptr || this->m_ini_five_axis == nullptr){
		err_code = ERR_SC_INIT;

		goto END;
	}


	if(m_ini_five_axis->Load(FIVE_AXIS_CONFIG_FILE) == 0){//读取配置成功

//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){    //支持参数修改一次重启
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			m_five_axis_config[i].five_axis_type = m_ini_five_axis->GetIntValueOrDefault(sname, "five_axis_type", 0);   //五轴机床类型
			m_five_axis_config[i].x_offset_1 = m_ini_five_axis->GetDoubleValueOrDefault(sname, "x_offset_1", 10.0);    //第一转轴中心相对刀尖点X坐标
			m_five_axis_config[i].y_offset_1 = m_ini_five_axis->GetDoubleValueOrDefault(sname, "y_offset_1", 10.0);    //第一转轴中心相对刀尖点Y坐标
			m_five_axis_config[i].z_offset_1 = m_ini_five_axis->GetDoubleValueOrDefault(sname, "z_offset_1", 10.0);    //第一转轴中心相对刀尖点Y坐标
			m_five_axis_config[i].post_dir_1 = m_ini_five_axis->GetIntValueOrDefault(sname, "post_dir_1", 0);   	//第一转轴旋转正方向
			m_five_axis_config[i].range_limit_1 = m_ini_five_axis->GetIntValueOrDefault(sname, "range_limit_1", 0);   //第一转轴旋转受限

			m_five_axis_config[i].x_offset_2 = m_ini_five_axis->GetDoubleValueOrDefault(sname, "x_offset_2", 10.0);    //第二转轴中心相对刀尖点X坐标
			m_five_axis_config[i].y_offset_2 = m_ini_five_axis->GetDoubleValueOrDefault(sname, "y_offset_2", 10.0);    //第二转轴中心相对刀尖点Y坐标
			m_five_axis_config[i].z_offset_2 = m_ini_five_axis->GetDoubleValueOrDefault(sname, "z_offset_2", 10.0);    //第二转轴中心相对刀尖点Y坐标
			m_five_axis_config[i].post_dir_2 = m_ini_five_axis->GetIntValueOrDefault(sname, "post_dir_2", 0);   	//第二转轴旋转正方向
			m_five_axis_config[i].range_limit_2 = m_ini_five_axis->GetIntValueOrDefault(sname, "range_limit_2", 0);   //第二转轴旋转受限

			m_five_axis_config[i].rot_swing_arm_len = m_ini_five_axis->GetDoubleValueOrDefault(sname, "rot_swing_arm_len", 10.0);   //刀具旋转摆臂长度
			m_five_axis_config[i].five_axis_coord = m_ini_five_axis->GetIntValueOrDefault(sname, "five_axis_coord", 0);   //五轴编程坐标系选择

			m_five_axis_config[i].speed_limit_x = m_ini_five_axis->GetDoubleValueOrDefault(sname, "speed_limit_x", 10000.0);   //五轴联动X轴速度限制
			m_five_axis_config[i].speed_limit_y = m_ini_five_axis->GetDoubleValueOrDefault(sname, "speed_limit_y", 10000.0);   //五轴联动X轴速度限制
			m_five_axis_config[i].speed_limit_z = m_ini_five_axis->GetDoubleValueOrDefault(sname, "speed_limit_z", 10000.0);   //五轴联动X轴速度限制
			m_five_axis_config[i].speed_limit_a = m_ini_five_axis->GetDoubleValueOrDefault(sname, "speed_limit_a", 10000.0);   //五轴联动X轴速度限制
			m_five_axis_config[i].speed_limit_b = m_ini_five_axis->GetDoubleValueOrDefault(sname, "speed_limit_b", 10000.0);   //五轴联动X轴速度限制
			m_five_axis_config[i].speed_limit_c = m_ini_five_axis->GetDoubleValueOrDefault(sname, "speed_limit_c", 10000.0);   //五轴联动X轴速度限制

			m_five_axis_config[i].intp_angle_step = m_ini_five_axis->GetDoubleValueOrDefault(sname, "intp_angle_step", 1.0);   //五轴联动粗插补角度步距
			m_five_axis_config[i].intp_len_step = m_ini_five_axis->GetDoubleValueOrDefault(sname, "intp_len_step", 1.0);   //五轴联动粗插补最小长度
			m_five_axis_config[i].five_axis_speed_plan = m_ini_five_axis->GetIntValueOrDefault(sname, "five_axis_speed_plan", 0);   //五轴联动速度规划
			m_five_axis_config[i].five_axis_plan_mode = m_ini_five_axis->GetIntValueOrDefault(sname, "five_axis_plan_mode", 0);   //五轴速度规划方式
		}
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取五轴配置文件成功！\n");

	}
	else {
		if(m_ini_five_axis->CreateFile(FIVE_AXIS_CONFIG_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认五轴配置文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}
//		for(i = 0; i < m_sc_system_config->chn_count; i++){
		for(i = 0; i < m_sc_system_config->max_chn_count; i++){    //支持参数修改一次重启
			//加载默认值
			m_five_axis_config[i].five_axis_type = 0;  //五轴机床类型
			m_five_axis_config[i].x_offset_1 = 10.0;    //第一转轴中心相对刀尖点X坐标
			m_five_axis_config[i].y_offset_1 = 10.0;    //第一转轴中心相对刀尖点Y坐标
			m_five_axis_config[i].z_offset_1 = 10.0;    //第一转轴中心相对刀尖点Y坐标
			m_five_axis_config[i].post_dir_1 = 0;   //第一转轴旋转正方向
			m_five_axis_config[i].range_limit_1 = 0;   //第一转轴旋转受限

			m_five_axis_config[i].x_offset_2 = 10.0;    //第二转轴中心相对刀尖点X坐标
			m_five_axis_config[i].y_offset_2 = 10.0;    //第二转轴中心相对刀尖点Y坐标
			m_five_axis_config[i].z_offset_2 = 10.0;    //第二转轴中心相对刀尖点Y坐标
			m_five_axis_config[i].post_dir_2 = 0;   //第二转轴旋转正方向
			m_five_axis_config[i].range_limit_2 = 0;   //第二转轴旋转受限

			m_five_axis_config[i].rot_swing_arm_len = 10.0;   //刀具旋转摆臂长度
			m_five_axis_config[i].five_axis_coord = 0;   //五轴编程坐标系选择

			m_five_axis_config[i].speed_limit_x = 10000.0;   //五轴联动X轴速度限制
			m_five_axis_config[i].speed_limit_y = 10000.0;   //五轴联动Y轴速度限制
			m_five_axis_config[i].speed_limit_z = 10000.0;   //五轴联动Z轴速度限制
			m_five_axis_config[i].speed_limit_a = 10000.0;   //五轴联动A轴速度限制
			m_five_axis_config[i].speed_limit_b = 10000.0;   //五轴联动B轴速度限制
			m_five_axis_config[i].speed_limit_c = 10000.0;   //五轴联动C轴速度限制

			m_five_axis_config[i].intp_angle_step = 1.0;   //五轴联动粗插补角度步距
			m_five_axis_config[i].intp_len_step = 1.0;   //五轴联动粗插补最小长度
			m_five_axis_config[i].five_axis_speed_plan = 0;   //五轴联动速度规划
			m_five_axis_config[i].five_axis_plan_mode = 0;   //五轴速度规划方式

			//生成默认ini配置
			memset(sname, 0x00, sizeof(sname));
			sprintf(sname, "channel_%d", i);
			ns = m_ini_five_axis->AddSecttion(sname);
			if(ns == nullptr){
				err_code = ERR_SC_INIT;
				goto END;
			}

			m_ini_five_axis->AddKeyValuePair(string("five_axis_type"), string("0"), ns);   //五轴机床类型
			m_ini_five_axis->AddKeyValuePair(string("x_offset_1"), string("10.0"), ns);   //第一转轴中心相对刀尖点X坐标
			m_ini_five_axis->AddKeyValuePair(string("y_offset_1"), string("10.0"), ns);   //第一转轴中心相对刀尖点Y坐标
			m_ini_five_axis->AddKeyValuePair(string("z_offset_1"), string("10.0"), ns);   //第一转轴中心相对刀尖点Z坐标
			m_ini_five_axis->AddKeyValuePair(string("post_dir_1"), string("0"), ns);     //第一转轴旋转正方向
			m_ini_five_axis->AddKeyValuePair(string("range_limit_1"), string("0"), ns);   //第一转轴旋转受限

			m_ini_five_axis->AddKeyValuePair(string("x_offset_2"), string("10.0"), ns);   //第二转轴中心相对刀尖点X坐标
			m_ini_five_axis->AddKeyValuePair(string("y_offset_2"), string("10.0"), ns);   //第二转轴中心相对刀尖点Y坐标
			m_ini_five_axis->AddKeyValuePair(string("z_offset_2"), string("10.0"), ns);   //第二转轴中心相对刀尖点Z坐标
			m_ini_five_axis->AddKeyValuePair(string("post_dir_2"), string("0"), ns);     //第二转轴旋转正方向
			m_ini_five_axis->AddKeyValuePair(string("range_limit_2"), string("0"), ns);   //第二转轴旋转受限

			m_ini_five_axis->AddKeyValuePair(string("rot_swing_arm_len"), string("10.0"), ns);   //刀具旋转摆臂长度
			m_ini_five_axis->AddKeyValuePair(string("five_axis_coord"), string("0"), ns);   //五轴编程坐标系选择
			m_ini_five_axis->AddKeyValuePair(string("speed_limit_x"), string("10000.0"), ns);   //五轴联动X轴速度限制
			m_ini_five_axis->AddKeyValuePair(string("speed_limit_y"), string("10000.0"), ns);   //五轴联动Y轴速度限制
			m_ini_five_axis->AddKeyValuePair(string("speed_limit_z"), string("10000.0"), ns);   //五轴联动Z轴速度限制
			m_ini_five_axis->AddKeyValuePair(string("speed_limit_a"), string("10000.0"), ns);   //五轴联动A轴速度限制
			m_ini_five_axis->AddKeyValuePair(string("speed_limit_b"), string("10000.0"), ns);   //五轴联动B轴速度限制
			m_ini_five_axis->AddKeyValuePair(string("speed_limit_c"), string("10000.0"), ns);   //五轴联动C轴速度限制

			m_ini_five_axis->AddKeyValuePair(string("intp_angle_step"), string("1.0"), ns);   //五轴联动粗插补角度步距
			m_ini_five_axis->AddKeyValuePair(string("intp_len_step"), string("1.0"), ns);   //五轴联动粗插补最小长度
			m_ini_five_axis->AddKeyValuePair(string("five_axis_speed_plan"), string("0"), ns);   //五轴联动速度规划
			m_ini_five_axis->AddKeyValuePair(string("five_axis_plan_mode"), string("0"), ns);   //五轴速度规划方式
		}

		m_ini_five_axis->Save();
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认五轴配置文件成功！\n");


	}
	END:
	if(err_code != ERR_NONE){
		CreateError(err_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		return false;
	}

	return true;
}

/**
 * @brief 更新五轴参数
 * @param chn_index ： 通道号
 * @param param_no : 参数号
 * @param value ： 参数值
 * @return
 */
bool ParmManager::UpdateFiveAxisParam(uint16_t chn_index, uint32_t param_no, ParamValue &value){
	bool res = true;
//	if(chn_index >= this->m_sc_system_config->chn_count){
	if(chn_index >= this->m_sc_system_config->max_chn_count){   //支持参数修改，一次重启
		g_ptr_trace->PrintLog(LOG_ALARM, "五轴参数更新，通道号非法：%hhu", chn_index);
		return false;
	}
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	sprintf(sname, "channel_%hhu", chn_index);
	switch(param_no){
	case 3000:	//五轴机床类型
		sprintf(kname, "five_axis_type");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 3010:	//第一转轴中心相对刀尖点X坐标
		sprintf(kname, "x_offset_1");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3011:  //第一转轴中心相对刀尖点Y坐标
		sprintf(kname, "y_offset_1");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3012:  //第一转轴中心相对刀尖点Z坐标
		sprintf(kname, "z_offset_1");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3013:  //第一转轴旋转正方向
		sprintf(kname, "post_dir_1");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 3014:  //第一转轴旋转受限
		sprintf(kname, "range_limit_1");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 3020:	//第二转轴中心相对刀尖点X坐标
		sprintf(kname, "x_offset_2");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3021:  //第二转轴中心相对刀尖点Y坐标
		sprintf(kname, "y_offset_2");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3022:  //第二转轴中心相对刀尖点Z坐标
		sprintf(kname, "z_offset_2");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3023:  //第二转轴旋转正方向
		sprintf(kname, "post_dir_2");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 3024:  //第二转轴旋转受限
		sprintf(kname, "range_limit_2");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 3030:	//刀具旋转摆臂长度
		sprintf(kname, "rot_swing_arm_len");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3031:	//五轴编程坐标系选择
		sprintf(kname, "five_axis_coord");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 3040:	//五轴联动X轴速度限制
		sprintf(kname, "speed_limit_x");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3041:	//五轴联动Y轴速度限制
		sprintf(kname, "speed_limit_y");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3042:	//五轴联动Z轴速度限制
		sprintf(kname, "speed_limit_z");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3043:	//五轴联动A轴速度限制
		sprintf(kname, "speed_limit_a");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3044:	//五轴联动B轴速度限制
		sprintf(kname, "speed_limit_b");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3045:	//五轴联动C轴速度限制
		sprintf(kname, "speed_limit_c");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3050:	//五轴联动粗插补角度步距
		sprintf(kname, "intp_angle_step");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3051:	//五轴联动粗插补最小长度
		sprintf(kname, "intp_len_step");
		m_ini_five_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 3060:	//五轴联动速度规划
		sprintf(kname, "five_axis_speed_plan");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 3061:	//五轴速度规划方式
		sprintf(kname, "five_axis_plan_mode");
		m_ini_five_axis->SetIntValue(sname, kname, value.value_uint8);
		break;

	default:	//
		g_ptr_trace->PrintLog(LOG_ALARM, "五轴参数更新，参数号非法：%d", param_no);
		res = false;
		return res;
	}
	m_ini_five_axis->Save();

	return res;
}

/**
 * @brief
 * @param chn_index ： 通道号
 * @param param_no : 参数号
 * @param value ： 参数值
 */
void ParmManager::ActiveFiveAxisParam(uint16_t chn_index, uint32_t param_no, ParamValue &value){

	ChannelControl *p_chn = ChannelEngine::GetInstance()->GetChnControl(chn_index);
	if(p_chn == nullptr)
		return;
	switch(param_no){
	case 3000:	//五轴机床类型
		m_five_axis_config[chn_index].five_axis_type = value.value_uint8;
		break;
	case 3010:	//第一转轴中心相对刀尖点X坐标
		m_five_axis_config[chn_index].x_offset_1 = value.value_double;
		p_chn->UpdateFiveAxisParam(X_OFFSET_1);
		break;
	case 3011:  //第一转轴中心相对刀尖点Y坐标
		m_five_axis_config[chn_index].y_offset_1 = value.value_double;
		p_chn->UpdateFiveAxisParam(Y_OFFSET_1);
		break;
	case 3012:  //第一转轴中心相对刀尖点Z坐标
		m_five_axis_config[chn_index].z_offset_1 = value.value_double;
		p_chn->UpdateFiveAxisParam(Z_OFFSET_1);
		break;
	case 3013:  //第一转轴旋转正方向
		m_five_axis_config[chn_index].post_dir_1 = value.value_uint8;
		p_chn->UpdateFiveAxisParam(POST_DIR_1);
		break;
	case 3014:  //第一转轴旋转受限
		m_five_axis_config[chn_index].range_limit_1 = value.value_uint8;
		p_chn->UpdateFiveAxisRotParam();
		break;
	case 3020:	//第二转轴中心相对刀尖点X坐标
		m_five_axis_config[chn_index].x_offset_2 = value.value_double;
		p_chn->UpdateFiveAxisParam(X_OFFSET_2);
		break;
	case 3021:  //第二转轴中心相对刀尖点Y坐标
		m_five_axis_config[chn_index].y_offset_2 = value.value_double;
		p_chn->UpdateFiveAxisParam(Y_OFFSET_2);
		break;
	case 3022:  //第二转轴中心相对刀尖点Z坐标
		m_five_axis_config[chn_index].z_offset_2 = value.value_double;
		p_chn->UpdateFiveAxisParam(Z_OFFSET_2);
		break;
	case 3023:  //第二转轴旋转正方向
		m_five_axis_config[chn_index].post_dir_2 = value.value_uint8;
		p_chn->UpdateFiveAxisParam(POST_DIR_2);
		break;
	case 3024:  //第二转轴旋转受限
		m_five_axis_config[chn_index].range_limit_2 = value.value_uint8;
		p_chn->UpdateFiveAxisRotParam();
		break;
	case 3030:	//刀具旋转摆臂长度
		m_five_axis_config[chn_index].rot_swing_arm_len = value.value_double;
		p_chn->UpdateFiveAxisParam(ROT_SWING_ARM_LEN);
		break;
	case 3031:	//五轴编程坐标系选择
		m_five_axis_config[chn_index].five_axis_coord = value.value_uint8;
		p_chn->UpdateFiveAxisParam(PROGRAM_COORD);
		break;
	case 3040:	//五轴联动X轴速度限制
		m_five_axis_config[chn_index].speed_limit_x = value.value_double;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT_X);
		break;
	case 3041:	//五轴联动Y轴速度限制
		m_five_axis_config[chn_index].speed_limit_y = value.value_double;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT_Y);
		break;
	case 3042:	//五轴联动Z轴速度限制
		m_five_axis_config[chn_index].speed_limit_z = value.value_double;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT_Z);
		break;
	case 3043:	//五轴联动A轴速度限制
		m_five_axis_config[chn_index].speed_limit_a = value.value_double;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT_A);
		break;
	case 3044:	//五轴联动B轴速度限制
		m_five_axis_config[chn_index].speed_limit_b = value.value_double;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT_B);
		break;
	case 3045:	//五轴联动C轴速度限制
		m_five_axis_config[chn_index].speed_limit_c = value.value_double;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT_C);
		break;
	case 3050:	//五轴联动粗插补角度步距
		m_five_axis_config[chn_index].intp_angle_step = value.value_double;
		p_chn->UpdateFiveAxisParam(INTP_ANGLE_STEP);
		break;
	case 3051:	//五轴联动粗插补最小长度
		m_five_axis_config[chn_index].intp_len_step = value.value_double;
		p_chn->UpdateFiveAxisParam(INTP_LEN_STEP);
		break;
	case 3060:	//五轴联动速度规划
		m_five_axis_config[chn_index].five_axis_speed_plan = value.value_uint8;
		p_chn->UpdateFiveAxisParam(SPEED_LIMIT);
		break;
	case 3061:	//五轴速度规划方式
		m_five_axis_config[chn_index].five_axis_plan_mode = value.value_uint8;
		p_chn->UpdateFiveAxisParam(SPEED_PLAN_MODE);
		break;
	default:	//
		g_ptr_trace->PrintLog(LOG_ALARM, "五轴参数更新，参数号非法：%d", param_no);
		break;
	}
}

#endif

#ifdef USES_GRIND_MACHINE
/**
 * @brief 读取磨削专用参数
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


	if(m_ini_grind->Load(GRIND_CONFIG_FILE) == 0){//读取配置成功

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
		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "读取磨削配置文件成功！\n");
	}
	else {
		if(m_ini_grind->CreateFile(GRIND_CONFIG_FILE)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PARAM_MANAGER, "创建默认磨削配置文件失败！\n");
			err_code = ERR_SC_INIT;
			goto END;
		}

		//加载默认值
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

		//生成默认ini配置
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

		g_ptr_trace->PrintTrace(TRACE_INFO, PARAM_MANAGER, "创建默认磨削配置文件成功！\n");


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
 * @brief 保存参数到文件
 * @param type : 参数类型
 * @return true--成功   false--失败
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
 * @brief  保存所有参数到文件
 * @return true--成功   false--失败
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
 * @brief  获取指定通道配置
 * @param index ： 通道索引号
 * @return 成功返回对应配置指针，失败返回nullptr
 */
SCChannelConfig *ParmManager::GetChannelConfig(int index){
//	if(m_sc_channel_config == nullptr || index >= m_sc_system_config->chn_count)
	if(m_sc_channel_config == nullptr ||
		index >= m_sc_system_config->max_chn_count)   //参数修改，一次重启
		return nullptr;
	return &m_sc_channel_config[index];
}

/**
 * @brief 获取工艺相关通道参数
 * @param index ： 通道索引号
 * @return 成功返回对应配置指针，失败返回nullptr
 */
ChnProcParamGroup *ParmManager::GetChnProcessParam(int index){
//	if(this->m_p_chn_process_param == nullptr || index >= m_sc_system_config->chn_count)
	if(this->m_p_chn_process_param == nullptr ||
		index >= m_sc_system_config->max_chn_count)   //参数修改，一次重启
		return nullptr;
	return &m_p_chn_process_param[index];
}

/**
 * @brief 更新刀具半径磨损值
 * @param chn_index ： 通道号，从0开始
 * @param index : 从0开始
 * @param value ：刀具半径磨损
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
 * @brief 更新刀具半径几何偏执
 * @param chn_index ： 通道号，从0开始
 * @param index : 从0开始
 * @param value : 刀具几何半径
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
 * @brief 更新刀长磨损值
 * @param chn_index ： 通道号，从0开始
 * @param index : 从0开始
 * @param value ： 刀长磨损值，单位：mm
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
 * @brief 更新刀具长度测量值
 * @param chn_index ： 通道号，从0开始
 * @param index ： 0表示更新基准刀， 其它值为非基准刀
 * @param value ：刀长，单位：mm
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
 * @brief 更新刀具偏置信息
 * @param chn_index ： 通道号，从0开始
 * @param index ： 刀具补偿号，从0开始， H1时index为0
 * @param active : true--更新    false--放入待更新队列
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
	}else{//加入待生效队列
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
 * @brief 更新刀位信息
 * @param chn_index ： 通道索引号
 * @param cfg : 新配置
 */
void ParmManager::UpdateToolPotConfig(uint16_t chn_index, SCToolPotConfig &cfg){
	char sname[32];	//section name
	char kname[64];	//key name
	char value[32]; //value

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));
	memset(value, 0x00, sizeof(value));

	memcpy(&m_sc_tool_pot_config[chn_index], &cfg, sizeof(SCToolPotConfig));		//更新当前参数

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
 * @brief 更新刀具信息
 * @param chn_index ： 通道索引号
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
 * @brief 更新工件坐标系
 * @param chn_index ： 通道索引号
 * @param index : 坐标系索引
 * @param cfg ： 新坐标系
 * @param active : true--更新    false--放入待更新队列
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
		memcpy(&m_sc_coord_config[chn_index*kWorkCoordCount+index], &cfg, sizeof(SCCoordConfig));		//更新当前参数

	}
	else{//加入待生效队列
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
 * @brief 更新扩展工件坐标系
 * @param chn_index ： 通道索引号
 * @param index : 扩展坐标系索引
 * @param cfg ： 新坐标系
 */
void ParmManager::UpdateExCoordConfig(uint16_t chn_index, uint8_t index, HmiCoordConfig &cfg, bool active){
	char sname[32];	//section name
	char kname[64];	//key name
	char value[32]; //value

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));
	memset(value, 0x00, sizeof(value));

	if(active){
		memcpy(&m_sc_ex_coord_config[chn_index][index], &cfg, sizeof(SCCoordConfig));			//更新当前参数
	}
	else{//加入待生效队列
		CoordUpdate data;
		data.chn_index = chn_index;
		data.coord_index = index+7;  //跳过工基本件坐标系
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
 * @brief 激活指定通道待生效的工件坐标系更改
 * @param chn_index : 指定通道索引， 0xFF表示所有通道
 * @return true--激活了数据   false -- 没有激活数据
 */
bool ParmManager::ActiveCoordParam(uint8_t chn_index){
	int count = 0;
	CoordUpdate *coord = nullptr;
	ListNode<CoordUpdate> *node = m_list_coord->HeadNode();
	ListNode<CoordUpdate> *node_next = nullptr;

	while(node != nullptr){
		coord = &node->data;

		if(coord->chn_index == chn_index || chn_index == 0xFF){//更新当前参数
			if(coord->coord_index < kWorkCoordCount){  //工件坐标系
				memcpy(&m_sc_coord_config[coord->chn_index*kWorkCoordCount+coord->coord_index], &coord->value, sizeof(SCCoordConfig));

			}else{//扩展工件坐标系
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
 * @brief 激活待生效的刀具偏置
 * @param chn_index : 指定通道索引， 0xFF表示所有通道
 * @return true--激活了数据   false -- 没有激活数据
 */
bool ParmManager::ActiveToolOffsetParam(uint8_t chn_index){
	int count = 0;
	ToolOffsetUpdate *offset = nullptr;
	ListNode<ToolOffsetUpdate> *node = m_list_tool_offset->HeadNode();
	ListNode<ToolOffsetUpdate> *node_next = nullptr;

	while(node != nullptr){
		offset = &node->data;

		if(offset->chn_index == chn_index || chn_index == 0xFF){//更新当前参数
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
 * @brief 更新螺补数据
 * @param axis_index : 轴号，物理轴号，从0开始
 * @param dir_flag : 是否正向螺补， true--正向    false--负向
 * @param offset ：轴内偏移，从1开始
 * @param count : 数据个数
 * @param data : 螺补数据
 * @return
 */
bool ParmManager::UpdatePcData(uint8_t axis_index, bool dir_flag, uint16_t offset, uint16_t count, double *data){

	if(data == nullptr || axis_index >= m_sc_system_config->axis_count){
		printf("ParmManager::UpdatePcData return false, axis_index = %hhu\n", axis_index);
		return false;
	}

	uint16_t total_count = m_sc_axis_config[axis_index].pc_count;
	if(m_sc_axis_config[axis_index].pc_type == 1){//双向螺补
		total_count *= 2;    //总计点数*2
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
	sprintf(sname, "axis_%d", axis_index+1);    //从1开始
	memset(dir_str, 0x00, 20);

	int pc_count = m_sc_axis_config[axis_index].pc_count;

	for(int i = 0; i < count; i++ ){

		m_sc_pc_table->pc_table[axis_index][offset_index+i-1] = data[i];
		memset(kname, 0x00, sizeof(kname));

		if((offset_index+i-1) < pc_count){
			sprintf(dir_str, "pc_pos_");//正向螺补
			sprintf(kname, "%s%d", dir_str, offset+i);

		}else{
			sprintf(dir_str, "pc_neg_");//负向螺补
			sprintf(kname, "%s%d", dir_str, offset+i-pc_count);
		}

		m_ini_pc_table->SetDoubleValue(sname, kname, data[i]);
	}

	m_ini_pc_table->Save();

	return true;
}

/**
 * @brief 更新IO重映射数据
 * @param info : 重定向数据
 * @return  true--成功   false--失败
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
	if(has_flag){//已经存在
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

	}else{//不存在，需要添加
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
 * @brief 更新手轮通道映射
 * @param HandWheelMapInfoVec : 手轮通道映射关系
 * @param bRestart : 是否重启生效
 * @return true--成功    false--失败
 */
bool ParmManager::SyncHandWheelInfo(const HandWheelMapInfoVec &infoVec, bool bRestart)
{
    char sectionName[30];
    if (m_p_handwheel_param == infoVec)
    {//梯图配置结构没有改变，可以直接更新参数
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
    {//梯图配置结构改变，需要重新写入ini
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
 * @brief 更新参数
 * @param data : 待更新数据
 * @param active_type : 激活类型
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
	//将参数更改写入INI文件
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
 * @brief 更新工艺相关参数
 * @param data : 待更新数据
 * @param active_type : 激活类型
 * @return true--成功   false--失败
 */
bool ParmManager::UpdateProcParam(ProcParamUpdate *data, uint8_t active_type){
	bool res = true;
	if(data == nullptr)
		return true;
	printf("update process paramter, active_type:%hhu, id:%d, group_idx:%hhu\n", active_type, data->param_data.param_no, data->group_index);

	//将参数更改写入INI文件
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

	//如果是当前工艺参数组则激活
	if(res)
		this->ActiveProcParam(data, active_type);

	return res;
}

/**
 * @brief 更新系统参数，只是写入INI文件，并不更新系统参数
 * @param param_no ： 参数号
 * @param value	： 参数值
 */
bool ParmManager::UpdateSystemParam(uint32_t param_no, ParamValue &value){
	bool res = true;
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	sprintf(sname, "system");
	switch(param_no){
	case 3: 	//通道数
		if(value.value_uint8 > kMaxChnCount){
			return false;
		}
		sprintf(kname, "chn_count");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 5:		//物理轴数
		if(value.value_uint8 > kMaxAxisNum){
			return false;
		}
		sprintf(kname, "axis_count");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 6:		//总线通讯周期
		sprintf(kname, "bus_cycle");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 10:  //手轮编码格式
		sprintf(kname, "hw_code_type");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 11:  //手轮反向引导
		sprintf(kname, "hw_rev_trace");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 20:	//系统最大速度
		sprintf(kname, "max_sys_vel");
		m_ini_system->SetDoubleValue(sname, kname, value.value_double);
		printf("update max_sys_vel, %f\n", value.value_double);
		break;
	case 24:	//拐角最大加速度
		sprintf(kname, "max_corner_acc");
		m_ini_system->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 25:	//向心最大加速度
		sprintf(kname, "max_cent_acc");
		m_ini_system->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 30:	//轴名称下标
		sprintf(kname, "axis_name_ex");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 31:	//回参考点时倍率固定
		sprintf(kname, "fix_ratio_find_ref");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 32:	//螺距补偿方式
		//sprintf(kname, "pc_type");
		//m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		//this->m_sc_system_config->pc_type = value.value_uint8;
		break;
	case 33:	//DA过流保护
		sprintf(kname, "da_ocp");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 34:	//断点保存程序当前行号
		sprintf(kname, "save_lineno_poweroff");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 35:	//手动回参考点模式
		sprintf(kname, "manual_ret_ref_mode");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 60:	//普通IO滤波时间
		sprintf(kname, "io_filter_time");
		m_ini_system->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 61:	//快速IO滤波时间
		sprintf(kname, "fast_io_filter_time");
		m_ini_system->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 90:	//背光延时时间
		sprintf(kname, "backlight_delay_time");
		m_ini_system->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 91:	//蜂鸣时间限制
		sprintf(kname, "beep_time");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 92:	//存储空间告警长度
		sprintf(kname, "free_space_limit");
		m_ini_system->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 93:	//主板告警温度
		sprintf(kname, "alarm_temperature");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 99:	//调试信息级别
		sprintf(kname, "trace_level");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 100:	//调试模式
		sprintf(kname, "debug_mode");
		m_ini_system->SetIntValue(sname, kname, value.value_uint8);
		break;
	default:	//默认
		g_ptr_trace->PrintLog(LOG_ALARM, "系统参数更新，参数号非法：%d", param_no);
		res = false;
		return res;
	}

	m_ini_system->Save();
	return res;
}

#ifdef USES_GRIND_MACHINE
/**
 * @brief 更新磨削参数
 * @param param_no ： 参数号
 * @param value ： 参数值
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
	case 10100: 	//砂轮半径
		sprintf(kname, "wheel_radius");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_radius = value.value_double;
		break;
	case 10101:		//X轴安全位置坐标
		sprintf(kname, "safe_pos_x");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->safe_pos_x = value.value_double;
		break;
	case 10102:		//C轴安全位置坐标
		sprintf(kname, "safe_pos_c");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->safe_pos_c = value.value_double;
		break;
	case 10103:	//砂轮震荡磨削Z轴最大长度
		sprintf(kname, "shock_z_max_dis");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->shock_z_max_dis = value.value_double;
		break;
	case 10104:	//砂轮震荡磨削X轴补偿比例
		sprintf(kname, "shock_x_compen_ratio");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->shock_x_compen_ratio = value.value_double;
		break;
	case 10105:	//砂轮震荡磨削速度
		sprintf(kname, "shock_speed");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->shock_speed = value.value_double;
		break;
	case 10106:	//震荡速度受手动倍率控制
		sprintf(kname, "shock_manual_ratio_ctrl");
		m_ini_grind->SetIntValue(sname, kname, value.value_uint8);
		m_grind_config->shock_manual_ratio_ctrl = value.value_uint8;
		break;
	case 10107:	//砂轮震荡磨削初始方向
		sprintf(kname, "shock_init_dir");
		m_ini_grind->SetIntValue(sname, kname, value.value_uint8);
		m_grind_config->shock_init_dir = value.value_uint8;
		break;
	case 10108:	//切削加工方向
		sprintf(kname, "grind_dir");
		m_ini_grind->SetIntValue(sname, kname, value.value_uint8);
		m_grind_config->grind_dir = value.value_uint8;
		break;
	case 10109:	//转角向量角速度
		sprintf(kname, "corner_vec_speed");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->corner_vec_speed = value.value_double;
		break;
	case 10110:	//磨削平滑功能
		sprintf(kname, "grind_smooth_fun");
		m_ini_grind->SetIntValue(sname, kname, value.value_uint8);
		m_grind_config->grind_smooth_fun = value.value_uint8;
		break;
	case 10111:	//磨削平滑时间常数
		sprintf(kname, "grind_smooth_time");
		m_ini_grind->SetIntValue(sname, kname, value.value_uint16);
		m_grind_config->grind_smooth_time = value.value_uint16;
		break;
	case 10112:	//砂轮半径1
		sprintf(kname, "wheel_raidus_1");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_raidus_multi[0] = value.value_double;
		break;
	case 10113:	//砂轮半径2
		sprintf(kname, "wheel_raidus_2");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_raidus_multi[1] = value.value_double;
		break;
	case 10114:	//砂轮半径3
		sprintf(kname, "wheel_raidus_3");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_raidus_multi[2] = value.value_double;
		break;
	case 10115:	//砂轮半径4
		sprintf(kname, "wheel_raidus_4");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_raidus_multi[3] = value.value_double;
		break;
	case 10116:	//砂轮半径5
		sprintf(kname, "wheel_raidus_5");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_raidus_multi[4] = value.value_double;
		break;
	case 10117:	//砂轮半径6
		sprintf(kname, "wheel_raidus_6");
		m_ini_grind->SetDoubleValue(sname, kname, value.value_double);
		m_grind_config->wheel_raidus_multi[5] = value.value_double;
		break;
	default:	//默认
		g_ptr_trace->PrintLog(LOG_ALARM, "磨削参数更新，参数号非法：%d", param_no);
		res = false;
		return res;
	}

	m_ini_grind->Save();
	this->m_b_update_grind = true;
	return res;
}

#endif

/**
 * @brief 更新通道参数，只是写入INI文件，并不更新通道参数
 * @param chn_index ： 通道号
 * @param param_no ： 参数号
 * @param value ： 参数值
 */
bool ParmManager::UpdateChnParam(uint8_t chn_index, uint32_t param_no, ParamValue &value){
	bool res = true;
//	if(chn_index >= this->m_sc_system_config->chn_count){
	if(chn_index >= this->m_sc_system_config->max_chn_count){  //支持参数修改，一次重启
		g_ptr_trace->PrintLog(LOG_ALARM, "通道参数更新，通道号非法：%hhu", chn_index);
		return false;
	}
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	sprintf(sname, "channel_%hhu", chn_index);
	switch(param_no){
	case 102:	//轴数量
		sprintf(kname, "chn_axis_count");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 103:   //通道所属方式组
		sprintf(kname, "chn_group_index");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 104:	//基本轴X
//		sprintf(kname, "chn_axis_x");
//		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
//		break;
	case 105:	//基本轴Y
//		sprintf(kname, "chn_axis_y");
//		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
//		break;
	case 106:	//基本轴Z
//		sprintf(kname, "chn_axis_z");
//		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
//		break;
	case 107:	//轴4
	case 108:	//轴5
	case 109:	//轴6
	case 110:	//轴7
	case 111:	//轴8
		sprintf(kname, "chn_axis_%d", param_no-103);
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 136:  //轴名称
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
	case 168:  //轴名称扩展下标
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
	case 200:	//插补模式
		sprintf(kname, "intep_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 201:	//插补周期
		sprintf(kname, "intep_cycle");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 202:	//G00运行模式
		sprintf(kname, "rapid_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 203:	//切削速度规划方式
		sprintf(kname, "cut_plan_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 204:	//定位速度规划方式
		sprintf(kname, "rapid_plan_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 210:	//通道最大限制速度
		sprintf(kname, "chn_max_vel");
		m_ini_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 211:	//通道最大加速度
		sprintf(kname, "chn_max_acc");
		m_ini_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 212:	//通道最大减速度
		sprintf(kname, "chn_max_dec");
		m_ini_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 213:	//
		break;
	case 214:	//拐角加速度
		sprintf(kname, "chn_max_corner_acc");
		m_ini_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 215:	//最大向心加速度
		sprintf(kname, "chn_max_arc_acc");
		m_ini_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 216:	//切削进给S型规划时间常数
		sprintf(kname, "chn_s_cut_filter_time");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint16);
		break;

	case 220:	//加工精度
		sprintf(kname, "chn_precision");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 221:	//前瞻功能
		sprintf(kname, "chn_look_ahead");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 222:	//加工速度调整等级
		sprintf(kname, "chn_feed_limit_level");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 223:	//拐角准停控制
		sprintf(kname, "corner_stop");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 229:  //拐角准停下限角度
		sprintf(kname, "corner_stop_angle_min");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 224:	//下刀准停
		sprintf(kname, "zmove_stop");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 225:	//转角加速度限制
		sprintf(kname, "corner_acc_limit");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 226:	//长直线段运行加速
		sprintf(kname, "long_line_acc");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 228:  //拐角准停使能
		sprintf(kname, "corner_stop_enable");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 230:	//圆弧误差限制
		sprintf(kname, "arc_err_limit");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 231:	//基于轴速度差的转角速度钳制
		sprintf(kname, "chn_spd_limit_on_axis");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 232:	//基于加速度的小线段进给速度钳制
		sprintf(kname, "chn_spd_limit_on_acc");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 233:	//基于曲率及向心加速的小线段进给速度钳制
		sprintf(kname, "chn_spd_limit_on_curvity");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 234:	//G00重叠等级
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
	case 350:	//换刀方式
		sprintf(kname, "change_tool_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 351:	//刀具寿命检测
		sprintf(kname, "tool_live_check");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 352:	//对刀仪自动对刀功能
		sprintf(kname, "auto_tool_measure");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 400:	//加工代码跟踪
		sprintf(kname, "gcode_trace");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 401:	//编程坐标单位
		sprintf(kname, "gcode_unit");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 402:	//默认平面
		sprintf(kname, "default_plane");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 403:	//默认编程模式
		sprintf(kname, "default_cmd_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 404:	//扩展工件坐标系数量
		sprintf(kname, "ex_coord_count");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
    case 405:   //默认进给方式
        sprintf(kname, "default_feed_mode");
        m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
        break;
	case 506:	//加工计时方式
		sprintf(kname, "timing_mode");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 510:   //G31跳转信号
		sprintf(kname, "g31_skip_signal");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 511:   //G31跳转信号有效电平
		sprintf(kname, "g31_sig_level");
		m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
    case 512:   //复位时间
        sprintf(kname, "rst_hold_time");
        m_ini_chn->SetIntValue(sname, kname, value.value_uint16);
        break;
    case 513:   //复位是否保留运行数据
        sprintf(kname, "rst_mode");
        m_ini_chn->SetIntValue(sname, kname, value.value_uint8);
        break;
    case 514:   //G00最高进给速度
        sprintf(kname, "g00_max_speed");
        m_ini_chn->SetIntValue(sname, kname, value.value_uint32);
        break;
    case 515:   //G01最高进给速度
        sprintf(kname, "g01_max_speed");
        m_ini_chn->SetIntValue(sname, kname, value.value_uint32);
        break;
#ifdef USES_WOOD_MACHINE
	case 600:  //DSP调试参数1
		sprintf(kname, "debug_param_1");
		m_ini_chn->SetIntValue(sname, kname, value.value_int32);
		break;
	case 601:  //DSP调试参数2
		sprintf(kname, "debug_param_2");
		m_ini_chn->SetIntValue(sname, kname, value.value_int32);
		break;
	case 602:  //DSP调试参数3
		sprintf(kname, "debug_param_3");
		m_ini_chn->SetIntValue(sname, kname, value.value_int32);
		break;
	case 603:  //DSP调试参数4
		sprintf(kname, "debug_param_4");
		m_ini_chn->SetIntValue(sname, kname, value.value_int32);
		break;
	case 604:  //DSP调试参数5
		sprintf(kname, "debug_param_5");
		m_ini_chn->SetIntValue(sname, kname, value.value_int32);
		break;
#endif
	default:	//
		g_ptr_trace->PrintLog(LOG_ALARM, "通道参数更新，参数号非法：%d", param_no);
		res = false;
		return res;
	}
	m_ini_chn->Save();
	return res;
}

/**
 * @brief 更新轴参数，只是写入INI文件，并不更新轴参数
 * @param axis_index ： 轴号, 从0开始的物理轴号
 * @param param_no ： 参数号
 * @param value ： 参数值
 */
bool ParmManager::UpdateAxisParam(uint8_t axis_index, uint32_t param_no, ParamValue &value){
	bool res = true;
//	if(axis_index >= this->m_sc_system_config->axis_count){
	if(axis_index >= this->m_sc_system_config->max_axis_count){   //支持参数修改后一次重启
		g_ptr_trace->PrintLog(LOG_ALARM, "轴参数更新，轴号非法：%hhu", axis_index);
		return false;
	}
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));
//	printf("update axis param, axis[%hhu], param_no[%u], value[%hhu]\n", axis_index, param_no, value.value_uint8);

	sprintf(sname, "axis_%hhu", axis_index+1);
	switch(param_no){
	case 1001:	//轴类型
		sprintf(kname, "axis_type");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1002:	//轴接口类型
		sprintf(kname, "axis_interface");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1003:	//从站号（总线轴）/对应轴口号（非总线轴）
		sprintf(kname, "axis_port");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1004:	//直线轴类型
		sprintf(kname, "axis_linear_type");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1006:	//是否PMC轴
		sprintf(kname, "axis_pmc");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1100:	//自动比例参数
		sprintf(kname, "kp1");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1101:	//手动比例参数
		sprintf(kname, "kp2");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1102:	//积分参数
		sprintf(kname, "ki");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1103:	//积分饱和限
		sprintf(kname, "kil");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1104:	//微分参数
		sprintf(kname, "kd");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1105:	//速度前馈系数
		sprintf(kname, "kvff");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1106:	//加速度前馈系数
		sprintf(kname, "kaff");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1107:	//跟随误差限制
		sprintf(kname, "track_err_limit");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1108:	//定位误差限制
		sprintf(kname, "location_err_limit");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1120:	//定位加速度
		sprintf(kname, "rapid_acc");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1121:	//手动加速度
		sprintf(kname, "manual_acc");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1122:	//启动加速度
		sprintf(kname, "start_acc");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1130:	//定位S型规划时间常数
		sprintf(kname, "rapid_s_plan_filter_time");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1135:
		sprintf(kname, "corner_acc_limit");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1140:	//插补后加减速滤波器类型
		sprintf(kname, "post_filter_type");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1141:	//插补后加减速滤波器时间常数1
		sprintf(kname, "post_filter_time_1");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1142:	//插补后加减速滤波器时间常数2
		sprintf(kname, "post_filter_time_2");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1200:	//电机每转计数
		sprintf(kname, "motor_count_pr");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1201:	//电机最大转速
		sprintf(kname, "motor_speed_max");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1202:	//每转移动量，即丝杆螺距
		sprintf(kname, "move_pr");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1203:	//电机旋转方向
		sprintf(kname, "motor_dir");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1204:	//反馈类型
		sprintf(kname, "feedback_mode");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1205:	//轴控制方式
		sprintf(kname, "ctrl_mode");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1206:	//每转输出脉冲数
		sprintf(kname, "pulse_count_pr");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1207:	//编码器单圈线数
		sprintf(kname, "encoder_lines");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1208:	//编码器整圈最大值
		sprintf(kname, "encoder_max_cycle");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1209:  //轴告警电平
		sprintf(kname, "axis_alarm_level");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1302:  //参考点基准误差
		sprintf(kname, "ref_mark_err");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1303:  //回参考点方式
		sprintf(kname, "ret_ref_mode");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1304:  //回参考点方向
		sprintf(kname, "ret_ref_dir");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1305:  //回参考点换向
		sprintf(kname, "ret_ref_change_dir");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1306:  //回参考点信号类型
		sprintf(kname, "ref_signal");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1308:  //回参考点速度
		sprintf(kname, "ret_ref_speed");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1309:  //回参考点顺序
		sprintf(kname, "ret_ref_index");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1310:  //粗精基准位置偏差检测
		sprintf(kname, "ref_base_diff_check");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1312:   //粗精基准位置偏差
		sprintf(kname, "ref_base_diff");
		m_ini_axis->SetIntValue(sname, kname, value.value_double);
		break;
	case 1314: 	//参考点编码器值
		sprintf(kname, "ref_encoder");
		m_ini_axis->SetInt64Value(sname, kname, value.value_int64);  //64位整型
		break;
    case 1315:  //回参考点低速
        sprintf(kname, "ret_ref_speed_second");
        m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
        break;
    case 1316:  //回参考点后的偏移量
        sprintf(kname, "ref_offset_pos");
        m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
        break;
    case 1317:  //搜索Z脉冲最大移动距离
        sprintf(kname, "ref_z_distance_max");
        m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
        break;
    case 1318:
        sprintf(kname, "absolute_ref_mode");
        m_ini_axis->SetIntValue(sname, kname, value.value_int8);
        break;

	case 1350:	//手动速度
		sprintf(kname, "manual_speed");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1351:	//定位速度
		sprintf(kname, "rapid_speed");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1352:	//复位速度
		sprintf(kname, "reset_speed");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1400:  //正向反向间隙
		printf("update axis 1400 val: %d\n", value.value_int16);
		sprintf(kname, "backlash_forward");
		m_ini_axis->SetIntValue(sname, kname, value.value_int16);
		break;
	case 1401:  //负向反向间隙
		printf("update axis 1401 val: %d\n", value.value_int16);
		sprintf(kname, "backlash_negative");
		m_ini_axis->SetIntValue(sname, kname, value.value_int16);
		break;
	case 1402:   //反向间隙是否生效  **************************
		printf("update axis 1402 \n");
		sprintf(kname, "backlash_enable");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		this->m_sc_axis_config[axis_index].backlash_enable = value.value_uint8;
		break;
	case 1403: //螺距补偿点数
		sprintf(kname, "pc_count");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		this->m_sc_axis_config[axis_index].pc_count = value.value_uint16;
		break;
	case 1404:  //补偿间隔
		sprintf(kname, "pc_inter_dist");
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		this->m_sc_axis_config[axis_index].pc_inter_dist = value.value_double;
		break;
	case 1405:  //参考点补偿位置
		sprintf(kname, "pc_ref_index");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		this->m_sc_axis_config[axis_index].pc_ref_index = value.value_uint16;
		break;
	case 1406:  //螺距补偿起始点
		sprintf(kname, "pc_offset");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		this->m_sc_axis_config[axis_index].pc_offset = value.value_uint16;
		break;
	case 1407:	//螺距补偿类型  ****************************
		printf("update axis 1407 \n");
		sprintf(kname, "pc_type");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		this->m_sc_axis_config[axis_index].pc_type = value.value_uint8;
		break;
	case 1408:  // 螺距补偿是否生效
		printf("update axis 1408 \n");
		sprintf(kname, "pc_enable");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		this->m_sc_axis_config[axis_index].pc_enable = value.value_uint8;
		break;
	case 1500: 	//软限位1
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
	case 1503: 	//软限位2
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
	case 1506: 	//软限位3
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
	case 1529:  //参考点
		sprintf(kname, "axis_home_pos_%d", param_no-1519);
		m_ini_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1550:  //快速定位    0--关闭   1--打开
		sprintf(kname, "fast_locate");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1551:  //位置显示模式   0--循环模式（0~360）    1--非循环模式
		sprintf(kname, "pos_disp_mode");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1602:	//主轴变速比
		sprintf(kname, "spd_gear_ratio");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1603:	//零漂补偿
		sprintf(kname, "zero_compensation");
		m_ini_axis->SetIntValue(sname, kname, value.value_int16);
		break;
	case 1604:	//主轴最高转速
		sprintf(kname, "spd_max_speed");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1605: //主轴启动时间
		sprintf(kname, "spd_start_time");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1606: //主轴制动时间
		sprintf(kname, "spd_stop_time");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1607:	//主轴最低转速
		sprintf(kname, "spd_min_speed");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1610:	//主轴电压控制方式
		sprintf(kname, "spd_vctrl_mode");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1612:	//主轴设置转速(rpm)
		sprintf(kname, "spd_set_speed");
		m_ini_axis->SetIntValue(sname, kname, value.value_uint32);
		break;
	case 1650:  //是否同步轴
		sprintf(kname, "sync_axis");
		m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
		break;
	case 1651:	//主动轴号
		sprintf(kname, "master_axis_no");
		m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
		break;
	case 1652:	//显示坐标
		sprintf(kname, "disp_coord");
		m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
		break;
	case 1653:	//主从轴基准位置偏差
		sprintf(kname, "benchmark_offset");
		m_ini_axis->SetDoubleValue(sname, kname,value.value_double);
		break;
	case 1655:	//位置同步最大误差
		sprintf(kname, "sync_err_max");
		m_ini_axis->SetIntValue(sname, kname,value.value_uint32);
		break;
	case 1656:	//从动轴回参考点后自动同步校准
		sprintf(kname, "auto_sync");
		m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
		break;
    case 1657:	//SOR信号用途
        sprintf(kname, "spd_ctrl_GST");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1658:	//主轴换挡方式
        sprintf(kname, "spd_ctrl_SGB");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1659:	//齿轮换挡时是否输出SF信号
        sprintf(kname, "spd_ctrl_SFA");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1660:	//主轴定向时的转向
        sprintf(kname, "spd_ctrl_ORM");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1661:	//主轴转向是否受M03/M04影响
        sprintf(kname, "spd_ctrl_TCW");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1662:	//主轴转向取反
        sprintf(kname, "spd_ctrl_CWM");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1663:	//螺纹切削和刚性攻丝时，主轴倍率设置
        sprintf(kname, "spd_ctrl_TSO");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint8);
        break;
    case 1664:	//模拟输出增益
        sprintf(kname, "spd_analog_gain");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1665:	//主轴齿轮换档/定向时的主轴转速
        sprintf(kname, "spd_sor_speed");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1666:	//主轴电机最小钳制速度
        sprintf(kname, "spd_motor_min_speed");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1667:	//主轴电机最大钳制速度
        sprintf(kname, "spd_motor_max_speed");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1668:	//齿轮低档位最高转速
        sprintf(kname, "spd_gear_speed_low");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1669:	//齿轮中档位最高转速
        sprintf(kname, "spd_gear_speed_middle");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1670:	//齿轮高档位最高转速
        sprintf(kname, "spd_gear_speed_high");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1671:	//B方式档1->档2电机转速
        sprintf(kname, "spd_gear_switch_speed1");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1672:	//B方式档2->档3电机转速
        sprintf(kname, "spd_gear_switch_speed2");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint16);
        break;
    case 1673:	//攻丝同步误差增益
        sprintf(kname, "spd_sync_error_gain");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint32);
        break;
    case 1674:	//攻丝轴速度前馈增益
        sprintf(kname, "spd_speed_feed_gain");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint32);
        break;
    case 1675:	//攻丝轴位置比例增益
        sprintf(kname, "spd_pos_ratio_gain");
        m_ini_axis->SetIntValue(sname, kname,value.value_uint32);
        break;

	default:
		g_ptr_trace->PrintLog(LOG_ALARM, "轴参数更新，参数号非法：%d", param_no);
		res = false;
		return res;
	}

	m_ini_axis->Save();
	return res;
}

/**
 * @brief 更新工艺相关通道参数
 * @param chn_index ： 通道号
 * @param group_index : 工艺参数组号，从0开始
 * @param param_no ： 参数号
 * @param value ： 参数值
 * @return true--成功   false--失败
 */
bool ParmManager::UpdateChnProcParam(uint8_t chn_index, uint8_t group_index, uint32_t param_no, ParamValue &value){
	bool res = true;
//	if(chn_index >= this->m_sc_system_config->chn_count){
	if(chn_index >= this->m_sc_system_config->max_chn_count){   //支持参数修改，一次重启
		g_ptr_trace->PrintLog(LOG_ALARM, "通道参数更新，通道号非法：%hhu", chn_index);
		return false;
	}
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	sprintf(sname, "channel_%hhu", chn_index);
	group_index++;   //组号加一，从1开始
	switch(param_no){
	case 202:	//G00运行模式
		sprintf(kname, "rapid_mode_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
	//	printf("update para 202 , value = %hhu\n", value.value_uint8);
		break;

	case 203:	//切削速度规划方式
		sprintf(kname, "cut_plan_mode_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 204:	//定位速度规划方式
		sprintf(kname, "rapid_plan_mode_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 210:	//通道最大限制速度
		sprintf(kname, "chn_max_vel_%hhu", group_index);
		m_ini_proc_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 211:	//通道最大加速度
		sprintf(kname, "chn_max_acc_%hhu", group_index);
		m_ini_proc_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 212:	//通道最大减速度
		sprintf(kname, "chn_max_dec_%hhu", group_index);
		m_ini_proc_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 214:	//拐角加速度
		sprintf(kname, "chn_max_corner_acc_%hhu", group_index);
		m_ini_proc_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 215:	//最大向心加速度
		sprintf(kname, "chn_max_arc_acc_%hhu", group_index);
		m_ini_proc_chn->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 216:	//切削进给S型规划时间常数
		sprintf(kname, "chn_s_cut_filter_time_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint16);
		break;

	case 223:	//拐角准停控制
		sprintf(kname, "corner_stop_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 229:  //拐角准停下限角度
		sprintf(kname, "corner_stop_angle_min_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 225:	//转角加速度限制
		sprintf(kname, "corner_acc_limit_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 228:  //拐角准停使能
		sprintf(kname, "corner_stop_enable_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;

	case 231:	//基于轴速度差的转角速度钳制
		sprintf(kname, "chn_spd_limit_on_axis_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 232:	//基于加速度的小线段进给速度钳制
		sprintf(kname, "chn_spd_limit_on_acc_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 233:	//基于曲率及向心加速的小线段进给速度钳制
		sprintf(kname, "chn_spd_limit_on_curvity_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 234:	//G00重叠等级
		sprintf(kname, "chn_rapid_overlap_level_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 235:  //小线段执行时间常数
		sprintf(kname, "chn_small_line_time_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_uint16);
		break;
#ifdef USES_WOOD_MACHINE
	case 240:  //挑角补偿值
		sprintf(kname, "flip_comp_value_%hhu", group_index);
		m_ini_proc_chn->SetIntValue(sname, kname, value.value_int32);
		break;
#endif
	default:	//
		g_ptr_trace->PrintLog(LOG_ALARM, "通道工艺相关参数更新，参数号非法：%d", param_no);
		res = false;
		return res;
	}
	m_ini_proc_chn->Save();
	return res;
}

/**
 * @brief 更新工艺相关轴参数
 * @param axis_index ：轴号, 从0开始
 * @param group_index  : 工艺参数组号
 * @param param_no ： 参数号
 * @param value ： 参数值
 * @return true--成功  false--失败
 */
bool ParmManager::UpdateAxisProcParam(uint8_t axis_index, uint8_t group_index, uint32_t param_no, ParamValue &value){
	bool res = true;
	if(axis_index >= this->m_sc_system_config->max_axis_count){   //支持参数修改后一次重启
		g_ptr_trace->PrintLog(LOG_ALARM, "轴工艺相关参数更新，轴号非法：%hhu", axis_index);
		return false;
	}
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));
	printf("update axis process param, axis[%hhu], group_index[%hhu], param_no[%u]\n", axis_index, group_index, param_no);

	sprintf(sname, "axis_%hhu", axis_index+1);
	group_index++;   //组号加一，从1开始
	switch(param_no){

	case 1120:	//定位加速度
		sprintf(kname, "rapid_acc_%hhu", group_index);
		m_ini_proc_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1121:	//手动加速度
		sprintf(kname, "manual_acc_%hhu", group_index);
		m_ini_proc_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1122:	//启动加速度
		sprintf(kname, "start_acc_%hhu", group_index);
		m_ini_proc_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1130:	//定位S型规划时间常数
		sprintf(kname, "rapid_s_plan_filter_time_%hhu", group_index);
		m_ini_proc_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1135:  //拐角速度差限制
		sprintf(kname, "corner_acc_limit_%hhu", group_index);
		m_ini_proc_axis->SetDoubleValue(sname, kname, value.value_double);
		break;
	case 1140:	//插补后加减速滤波器类型
		sprintf(kname, "post_filter_type_%hhu", group_index);
		m_ini_proc_axis->SetIntValue(sname, kname, value.value_uint8);
		break;
	case 1141:	//插补后加减速滤波器时间常数1
		sprintf(kname, "post_filter_time_1_%hhu", group_index);
		m_ini_proc_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	case 1142:	//插补后加减速滤波器时间常数2
		sprintf(kname, "post_filter_time_2_%hhu", group_index);
		m_ini_proc_axis->SetIntValue(sname, kname, value.value_uint16);
		break;
	default:
		g_ptr_trace->PrintLog(LOG_ALARM, "轴工艺相关参数更新，参数号非法：%d", param_no);
		res = false;
		return res;
	}

	m_ini_proc_axis->Save();
	return res;
}

/**
 * @brief 更新指定轴的参考点编码器值
 * @param axis : 轴号， 从0开始
 * @param value
 * @return
 */
bool ParmManager::UpdateAxisRef(uint8_t axis, int64_t value){
	if(axis >= this->m_sc_system_config->max_axis_count){   //支持参数修改后一次重启
		g_ptr_trace->PrintLog(LOG_ALARM, "轴参考点编码器值更新，轴号非法：%hhu", axis);
		return false;
	}
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	sprintf(sname, "axis_%hhu", axis+1);

	sprintf(kname, "ref_encoder");
	m_ini_axis->SetInt64Value(sname, kname, value);  //64位整型

	m_ini_axis->Save();

//	printf("save axis %hhu ref origin point\n", axis);
	return true;
}

/**
 * @brief 激活参数
 * @param data  : 待更新数据
 * @param active_type : 激活类型
 */
void ParmManager::ActiveParam(ParamUpdate *data, uint8_t active_type){
	if(active_type == 0){	//重启生效
		this->m_b_poweroff_param = true;
	}else if(active_type == 1){		//复位生效
		printf("add reset valid param:%hhu, %u \n", data->param_type, data->param_no);
		this->m_list_reset->Append(*data);
	}else if(active_type == 2){		//新一次加工生效
		this->m_list_new_start->Append(*data);

	}else if(active_type == 3){		//立即生效
		this->ActiveParam(data);
	}else{
		g_ptr_trace->PrintLog(LOG_ALARM, "参数激活类型非法：%hhu", active_type);
	}
}

/**
 * @brief 修改当前参数
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
 * @brief 激活工艺相关参数参数
 * @param data  : 待更新数据
 * @param active_type  : 激活类型
 */
void ParmManager::ActiveProcParam(ProcParamUpdate *data, uint8_t active_type){
	if(active_type == 0){	//重启生效
		this->m_b_poweroff_param = true;
	}else if(active_type == 1){		//复位生效
	//	printf("add reset valid proc param:%hhu, %hhu, %u \n", data->group_index, data->param_data.param_type, data->param_data.param_no);
		this->m_list_proc_reset.Append(*data);
	}else if(active_type == 2){		//新一次加工生效
		this->m_list_proc_new_start.Append(*data);

	}else if(active_type == 3){		//立即生效
		this->ActiveProcParam(data);
	}else{
		g_ptr_trace->PrintLog(LOG_ALARM, "工艺相关参数激活类型非法：%hhu", active_type);
	}
}

/**
 * @brief 修改当前工艺相关参数参数
 * @param data  : 待更新数据
 */
void ParmManager::ActiveProcParam(ProcParamUpdate *data){
	printf("active proc param :group = %hhu, type = %hhu, param_no = %u\n", data->group_index, data->param_data.param_type, data->param_data.param_no);

	ChannelEngine *chn_engine = ChannelEngine::GetInstance();

	switch(data->param_data.param_type){
	case CHN_CONFIG:
		if(chn_engine->GetChnControl(data->param_data.chn_index)->GetCurProcParamIndex() == data->group_index) //就是当前工艺参数组
			this->ActiveChnParam(data->param_data.chn_index, data->param_data.param_no, data->param_data.value, data->group_index);
		else
			this->ActiveChnProcParam(data->param_data.chn_index, data->param_data.param_no, data->param_data.value, data->group_index);
		break;
	case AXIS_CONFIG:{
		uint8_t chn_axis = 0;
		uint8_t chn_index = chn_engine->GetAxisChannel(data->param_data.axis_index, chn_axis);
		if(chn_index != 0xFF && chn_engine->GetChnControl(chn_index)->GetCurProcParamIndex() == data->group_index){  //轴所属通道的当前工艺参数组
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
 * @brief 单独更新通道工艺参数
 * @param chn_index ：所属通道号，从0开始
 * @param param_no ：参数编号
 * @param value ： 参数值
 * @param proc_index : 所属工艺参数组，0xFF表示非工艺参数
 */
void ParmManager::ActiveChnProcParam(uint8_t chn_index, uint32_t param_no, ParamValue &value, uint8_t proc_index){

	switch(param_no){
	case 202:	//G00运行模式
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].rapid_mode = value.value_uint8;
		}
		break;
	case 203:	//切削速度规划方式
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].cut_plan_mode = value.value_uint8;
		}
		break;
	case 204:	//定位速度规划方式
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].rapid_plan_mode = value.value_uint8;
		}
		break;
	case 210:	//通道最大限制速度
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_vel = value.value_double;
		}
		break;
	case 211:	//通道最大加速度
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_acc = value.value_double;
		}
		break;
	case 212:	//通道最大减速度
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_dec = value.value_double;
		}
		break;
	case 214:	//拐角加速度
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_corner_acc = value.value_double;
		}
		break;
	case 215:	//最大向心加速度
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_arc_acc = value.value_double;
		}
		break;
	case 216:	//切削进给S型规划时间常数
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_s_cut_filter_time = value.value_uint16;
		}
		break;
	case 223:	//拐角准停控制
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_stop = value.value_uint8;
		}
		break;
	case 225:	//转角加速度限制
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_acc_limit = value.value_uint8;
		}
		break;
	case 228:	//拐角准停使能
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_stop_enable = value.value_uint8;
		}
		break;
	case 229:  //拐角准停下限角度
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_stop_angle_min = value.value_uint8;
		}
		break;

	case 231:	//基于轴速度差的转角速度钳制
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_spd_limit_on_axis = value.value_uint8;
		}
		break;
	case 232:	//基于加速度的小线段进给速度钳制
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_spd_limit_on_acc = value.value_uint8;
		}
		break;
	case 233:	//基于曲率及向心加速的小线段进给速度钳制
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_spd_limit_on_curvity = value.value_uint8;
		}
		break;
	case 234:	//G00重叠等级
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_rapid_overlap_level = value.value_uint8;
		}
		break;
	case 235:	//小线段执行时间常数
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_small_line_time = value.value_uint16;
		}
		break;
#ifdef USES_WOOD_MACHINE
	case 240:  //挑角补偿值
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].flip_comp_value = value.value_int32;
		}
		break;
#endif
	default:	//
		g_ptr_trace->PrintLog(LOG_ALARM, "通道参数激活，参数号非法：%d", param_no);
		break;
	}
}

/**
 * @brief 单独更新通道轴参数
 * @param axis_index ：所属轴号，从0开始
 * @param param_no ： 参数编号
 * @param value ：参数值
 * @param proc_index : 所属工艺参数组号，0xFF表示非工艺参数
 */
void ParmManager::ActiveAxisProcParam(uint8_t axis_index, uint32_t param_no, ParamValue &value, uint8_t proc_index){

	switch(param_no){
	case 1120:	//定位加速度
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].rapid_acc = value.value_double;
		}
		break;
	case 1121:	//手动加速度
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].manual_acc = value.value_double;
		}
		break;
	case 1122:	//起步加速度
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].start_acc = value.value_double;
		}
		break;
	case 1130:	//定位S型速度规划时间常数
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].rapid_s_plan_filter_time = value.value_uint16;
		}
		break;
	case 1135:  //拐角速度差限制
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].corner_acc_limit = value.value_double;
		}
		break;
	case 1140:	//插补后滤波器类型
		if(proc_index < kMaxProcParamCount){ //滤波器类型
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].post_filter_type = value.value_uint8;
		}
		break;
	case 1141:	//插补后滤波器时间常数1
		if(proc_index < kMaxProcParamCount){ //一级滤波器时间常数
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].post_filter_time_1 = value.value_uint16;
		}
		break;
	case 1142:	//插补后滤波器时间常数2
		if(proc_index < kMaxProcParamCount){ //二级滤波器时间常数
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].post_filter_time_2 = value.value_uint16;
		}
		break;

	default:
		g_ptr_trace->PrintLog(LOG_ALARM, "工艺相关轴参数激活，参数号非法：%d", param_no);
		break;
	}
}


/**
 * @brief 激活复位有效的参数
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
 * @brief 激活新一次加工有效的参数
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
 * @brief 激活磨削参数
 * @param param_no
 * @param value
 */
void ParmManager::ActiveGrindParam(uint32_t param_no, ParamValue &value){
	//
}

#endif

/**
 * @brief 激活系统参数
 * @param param_no
 * @param value
 */
void ParmManager::ActiveSystemParam(uint32_t param_no, ParamValue &value){
	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	switch(param_no){
	case 3: 	//通道数
		this->m_sc_system_config->chn_count = value.value_uint8;
		break;
	case 5:		//物理轴数
		this->m_sc_system_config->axis_count = value.value_uint8;
		break;
	case 6:		//总线通讯周期
		this->m_sc_system_config->bus_cycle = value.value_uint8;
		break;
	case 10:  //手轮编码格式
		this->m_sc_system_config->hw_code_type = value.value_uint8;
		this->UpdateMiParam<uint8_t>(0xFF, 10, value.value_uint8);   //手轮编码类型
		break;
	case 11:   //手轮反向引导
		this->m_sc_system_config->hw_rev_trace = value.value_uint8;
		chn_engine->EnableHWTraceToMi();
		break;
//	case 20:	//系统最大速度
//		this->m_sc_system_config->max_sys_vel = value.value_double;
//		break;
//	case 24:	//拐角最大加速度
//		this->m_sc_system_config->max_corner_acc = value.value_double;
//		break;
//	case 25:	//向心最大加速度
//		this->m_sc_system_config->max_cent_acc = value.value_double;
//		break;
	case 30:	//轴名称下标
		this->m_sc_system_config->axis_name_ex = value.value_uint8;
		chn_engine->SetAxisNameEx(value.value_uint8);
		break;
	case 31:	//回参考点时倍率固定
		this->m_sc_system_config->fix_ratio_find_ref = value.value_uint8;
		break;
	case 32:	//螺距补偿方式
		//this->m_sc_system_config->pc_type = value.value_uint8;
		break;
	case 33:	//DA过流保护
		this->m_sc_system_config->da_ocp = value.value_uint8;
		break;
	case 34:	//断点保存程序当前行号
		this->m_sc_system_config->save_lineno_poweroff = value.value_uint8;
		break;
	case 35:	//手动回参考点模式
		this->m_sc_system_config->manual_ret_ref_mode = value.value_uint8;
		break;
	case 60:	//普通IO滤波时间
		this->m_sc_system_config->io_filter_time = value.value_uint32;
		break;
	case 61:	//快速IO滤波时间
		this->m_sc_system_config->fast_io_filter_time = value.value_uint32;
		break;
	case 90:	//背光延时时间
		this->m_sc_system_config->backlight_delay_time = value.value_uint16;
		break;
	case 91:	//蜂鸣时间限制
		this->m_sc_system_config->beep_time = value.value_uint8;
		break;
	case 92:	//存储空间告警长度
		this->m_sc_system_config->free_space_limit = value.value_uint32;
		break;
	case 93:	//主板告警温度
		this->m_sc_system_config->alarm_temperature = value.value_uint8;
		break;
	case 99:	//调试信息级别
		this->m_sc_system_config->trace_level = value.value_uint8;
		g_ptr_trace->set_trace_level(m_sc_system_config->trace_level);
		break;
	case 100:	//调试模式
		this->m_sc_system_config->debug_mode = value.value_uint8;
		break;
	default:	//默认
		break;
	}
}

/**
 * @brief 激活通道参数
 * @param chn_index ：所属通道号，从0开始
 * @param param_no ：参数编号
 * @param value ： 参数值
 * @param proc_index : 所属工艺参数组，0xFF表示非工艺参数
 */
void ParmManager::ActiveChnParam(uint8_t chn_index, uint32_t param_no, ParamValue &value, uint8_t proc_index){
	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	switch(param_no){
	case 102:	//轴数量
		this->m_sc_channel_config[chn_index].chn_axis_count = value.value_uint8;
		break;
	case 103:  //通道所属方式组
		chn_engine->ChangeChnGroupIndex(chn_index, m_sc_channel_config[chn_index].chn_group_index, value.value_uint8);
		this->m_sc_channel_config[chn_index].chn_group_index = value.value_uint8;
		break;
	case 104:	//基本轴X
//		this->m_sc_channel_config[chn_index].chn_axis_x = value.value_uint8;
//		break;
	case 105:	//基本轴Y
//		this->m_sc_channel_config[chn_index].chn_axis_y = value.value_uint8;
//		break;
	case 106:	//基本轴Z
//		this->m_sc_channel_config[chn_index].chn_axis_z = value.value_uint8;
//		break;
	case 107:	//轴4
	case 108:	//轴5
	case 109:	//轴6
	case 110:	//轴7
	case 111:	//轴8
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
	case 200:	//插补模式
		this->m_sc_channel_config[chn_index].intep_mode = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		break;
	case 201:	//插补周期
		this->m_sc_channel_config[chn_index].intep_cycle = value.value_uint8;
		break;
	case 202:	//G00运行模式
		this->m_sc_channel_config[chn_index].rapid_mode = value.value_uint8;
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].rapid_mode = value.value_uint8;
		}
		break;
	case 203:	//切削速度规划方式
		this->m_sc_channel_config[chn_index].cut_plan_mode = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanMode();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].cut_plan_mode = value.value_uint8;
		}
		break;
	case 204:	//定位速度规划方式
		this->m_sc_channel_config[chn_index].rapid_plan_mode = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanMode();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].rapid_plan_mode = value.value_uint8;
		}
		break;
	case 210:	//通道最大限制速度
		this->m_sc_channel_config[chn_index].chn_max_vel = value.value_double;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_vel = value.value_double;
		}
		break;
	case 211:	//通道最大加速度
		this->m_sc_channel_config[chn_index].chn_max_acc = value.value_double;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_acc = value.value_double;
		}
		break;
	case 212:	//通道最大减速度
		this->m_sc_channel_config[chn_index].chn_max_dec = value.value_double;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_dec = value.value_double;
		}
		break;
	case 213:	//
		break;
	case 214:	//拐角加速度
		this->m_sc_channel_config[chn_index].chn_max_corner_acc = value.value_double;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_corner_acc = value.value_double;
		}
		break;
	case 215:	//最大向心加速度
		this->m_sc_channel_config[chn_index].chn_max_arc_acc = value.value_double;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_max_arc_acc = value.value_double;
		}
		break;
	case 216:	//切削进给S型规划时间常数
		this->m_sc_channel_config[chn_index].chn_s_cut_filter_time = value.value_uint16;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_s_cut_filter_time = value.value_uint16;
		}
		break;
	case 220:	//加工精度
		this->m_sc_channel_config[chn_index].chn_precision = value.value_uint16;
		break;
	case 221:	//前瞻功能
		this->m_sc_channel_config[chn_index].chn_look_ahead = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		break;
	case 222:	//加工速度调整等级
		this->m_sc_channel_config[chn_index].chn_feed_limit_level = value.value_uint8;
		break;
	case 223:	//拐角准停控制
		this->m_sc_channel_config[chn_index].corner_stop = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnCornerStopParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_stop = value.value_uint8;
		}
		break;
	case 224:	//下刀准停
		this->m_sc_channel_config[chn_index].zmove_stop = value.value_uint8;
		break;
	case 225:	//转角加速度限制
		this->m_sc_channel_config[chn_index].corner_acc_limit = value.value_uint8;
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_acc_limit = value.value_uint8;
		}
		break;
	case 226:	//长直线段运行加速
		this->m_sc_channel_config[chn_index].long_line_acc = value.value_uint8;
		break;
	case 228:	//拐角准停使能
		this->m_sc_channel_config[chn_index].corner_stop_enable = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnCornerStopParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_stop_enable = value.value_uint8;
		}
		break;
	case 229:  //拐角准停下限角度
		this->m_sc_channel_config[chn_index].corner_stop_angle_min = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnCornerStopParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].corner_stop_angle_min = value.value_uint8;
		}
		break;
	case 230:	//圆弧误差限制
		this->m_sc_channel_config[chn_index].arc_err_limit = value.value_uint8;
		break;
	case 231:	//基于轴速度差的转角速度钳制
		this->m_sc_channel_config[chn_index].chn_spd_limit_on_axis = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_spd_limit_on_axis = value.value_uint8;
		}
		break;
	case 232:	//基于加速度的小线段进给速度钳制
		this->m_sc_channel_config[chn_index].chn_spd_limit_on_acc = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_spd_limit_on_acc = value.value_uint8;
		}
		break;
	case 233:	//基于曲率及向心加速的小线段进给速度钳制
		this->m_sc_channel_config[chn_index].chn_spd_limit_on_curvity = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_spd_limit_on_curvity = value.value_uint8;
		}
		break;
	case 234:	//G00重叠等级
		this->m_sc_channel_config[chn_index].chn_rapid_overlap_level = value.value_uint8;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_rapid_overlap_level = value.value_uint8;
		}
		break;
	case 235:	//小线段执行时间常数
		this->m_sc_channel_config[chn_index].chn_small_line_time = value.value_uint16;
		chn_engine->GetChnControl(chn_index)->SetMcChnPlanFun();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].chn_small_line_time = value.value_uint16;
		}
		break;
#ifdef USES_WOOD_MACHINE
	case 240:  //挑角补偿值
		this->m_sc_channel_config[chn_index].flip_comp_value = value.value_int32;
		chn_engine->GetChnControl(chn_index)->SetMcFlipCompParam();
		if(proc_index < kMaxProcParamCount){
			this->m_p_chn_process_param[chn_index].chn_param[proc_index].flip_comp_value = value.value_int32;
		}
		break;
#endif
	case 350:	//换刀方式
		this->m_sc_channel_config[chn_index].change_tool_mode = value.value_uint8;
		break;
	case 351:	//刀具寿命检测
		this->m_sc_channel_config[chn_index].tool_live_check = value.value_uint8;
		break;
	case 352:	//对刀仪自动对刀功能
		this->m_sc_channel_config[chn_index].auto_tool_measure = value.value_uint8;
		break;
	case 400:	//加工代码跟踪
		this->m_sc_channel_config[chn_index].gcode_trace = value.value_uint8;
		break;
	case 401:	//编程坐标单位
		this->m_sc_channel_config[chn_index].gcode_unit = value.value_uint8;
		break;
	case 402:	//默认平面
		this->m_sc_channel_config[chn_index].default_plane = value.value_uint8;
		break;
	case 403:	//默认编程模式
		this->m_sc_channel_config[chn_index].default_cmd_mode = value.value_uint8;
		break;
	case 404:	//扩展工件坐标系数量
		this->m_sc_channel_config[chn_index].ex_coord_count = value.value_uint8;
		break;
    case 405:	//默认进给方式
        this->m_sc_channel_config[chn_index].default_feed_mode = value.value_uint8;
        break;
	case 506:	//加工计时方式
		this->m_sc_channel_config[chn_index].timing_mode = value.value_uint8;
		break;
	case 510:   //G31跳转信号
		this->m_sc_channel_config[chn_index].g31_skip_signal = value.value_uint32;
		break;
	case 511:   //G31跳转信号有效电平
		this->m_sc_channel_config[chn_index].g31_sig_level = value.value_uint8;
		break;
    case 512:   //复位时间
        this->m_sc_channel_config[chn_index].rst_hold_time = value.value_uint16;
        break;
    case 513:   //复位是否保留运行时间
        this->m_sc_channel_config[chn_index].rst_mode = value.value_uint8;
        break;
    case 514:   //G00最高进给速度
        this->m_sc_channel_config[chn_index].g00_max_speed = value.value_uint32;
        break;
    case 515:   //G01最高进给速度
        this->m_sc_channel_config[chn_index].g01_max_speed = value.value_uint32;
        break;
#ifdef USES_WOOD_MACHINE
	case 600:  //DSP调试参数1
		this->m_sc_channel_config[chn_index].debug_param_1 = value.value_int32;
		chn_engine->GetChnControl(chn_index)->SetMcDebugParam(0);
		break;
	case 601:  //DSP调试参数2
		this->m_sc_channel_config[chn_index].debug_param_2 = value.value_int32;
		chn_engine->GetChnControl(chn_index)->SetMcDebugParam(1);
		break;
	case 602:  //DSP调试参数3
		this->m_sc_channel_config[chn_index].debug_param_3 = value.value_int32;
		chn_engine->GetChnControl(chn_index)->SetMcDebugParam(2);
		break;
	case 603:  //DSP调试参数4
		this->m_sc_channel_config[chn_index].debug_param_4 = value.value_int32;
		chn_engine->GetChnControl(chn_index)->SetMcDebugParam(3);
		break;
	case 604:  //DSP调试参数5
		this->m_sc_channel_config[chn_index].debug_param_5 = value.value_int32;
		chn_engine->GetChnControl(chn_index)->SetMcDebugParam(4);
		break;
#endif
	default:	//
		g_ptr_trace->PrintLog(LOG_ALARM, "通道参数激活，参数号非法：%d", param_no);
		break;
	}
}

/**
 * @brief 更新发送MI参数
 * @param axis : 轴号， MI的轴号从1开始, 0xFF表示系统参数
 * @param para_no : 参数号
 * @param data ：数据指针
 * @param size ：参数数据所占的字节数
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
 * @brief 激活轴参数
 * @param axis_index ：所属轴号，从0开始
 * @param param_no ： 参数编号
 * @param value ：参数值
 * @param proc_index : 所属工艺参数组号，0xFF表示非工艺参数
 */
void ParmManager::ActiveAxisParam(uint8_t axis_index, uint32_t param_no, ParamValue &value, uint8_t proc_index){
	uint8_t chan = 0;
	uint8_t chan_axis = 0;
	uint64_t tmp_64 = 0;   //临时变量
	ChannelEngine *chn_engine = ChannelEngine::GetInstance();
	switch(param_no){
	case 1001:	//轴类型
		this->m_sc_axis_config[axis_index].axis_type = value.value_uint8;
		break;
	case 1002:	//轴接口类型
		this->m_sc_axis_config[axis_index].axis_interface = value.value_uint8;
		break;
	case 1003:	//从站号（总线轴）/对应轴口号（非总线轴）
		this->m_sc_axis_config[axis_index].axis_port = value.value_uint8;
		UpdateMiParam<uint8_t>(axis_index+1, param_no, value.value_uint8);
		break;
	case 1004:	//直线轴类型
		this->m_sc_axis_config[axis_index].axis_linear_type = value.value_uint8;
		break;
	case 1006:	//是否PMC轴
		this->m_sc_axis_config[axis_index].axis_pmc = value.value_uint8;
		break;
	case 1100:	//自动比例参数
		printf("update axis[%hhu] kp1: %lf\n", axis_index, value.value_double);
		this->m_sc_axis_config[axis_index].kp1 = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
		break;
	case 1101:	//手动比例参数
		this->m_sc_axis_config[axis_index].kp2 = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
		break;
	case 1102:	//积分参数
		this->m_sc_axis_config[axis_index].ki = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
		break;
	case 1103:	//积分饱和限
		printf("update axis[%hhu] kil: %lf\n", axis_index, value.value_double);
		this->m_sc_axis_config[axis_index].kil = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
		break;
	case 1104:	//微分参数
		this->m_sc_axis_config[axis_index].kd = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double);
		break;
	case 1105:	//速度前馈系数
		this->m_sc_axis_config[axis_index].kvff = value.value_uint16;
		UpdateMiParam<double>(axis_index+1, param_no, static_cast<double>(value.value_uint16));
		break;
	case 1106:	//加速度前馈系数
		this->m_sc_axis_config[axis_index].kaff = value.value_uint16;
		UpdateMiParam<double>(axis_index+1, param_no, static_cast<double>(value.value_uint16));
		break;
	case 1107:	//跟随误差限制
		this->m_sc_axis_config[axis_index].track_err_limit = value.value_uint32;
		tmp_64 = static_cast<uint64_t>(value.value_uint32) * 1e4;   //转换单位：um-->0.1nm
		this->UpdateMiParam<uint64_t>(axis_index+1, param_no, tmp_64);	//跟踪误差限
		break;
	case 1108:	//定位误差限制
		this->m_sc_axis_config[axis_index].location_err_limit = value.value_uint16;
		tmp_64 = static_cast<uint64_t>(value.value_uint16) * 1e4;   //转换单位：um-->0.1nm
		this->UpdateMiParam<uint64_t>(axis_index+1, param_no, tmp_64);			//定位误差限
		break;
	case 1120:	//定位加速度
		this->m_sc_axis_config[axis_index].rapid_acc = value.value_double;
//		chn_engine->SetAxisAccParam(axis_index);
		chan = chn_engine->GetAxisChannel(axis_index, chan_axis);
		chn_engine->GetChnControl(chan)->SetChnAxisAccParam(chan_axis);
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].rapid_acc = value.value_double;
		}
		break;
	case 1121:	//手动加速度
		this->m_sc_axis_config[axis_index].manual_acc = value.value_double;
//		chn_engine->SetAxisAccParam(axis_index);
		chan = chn_engine->GetAxisChannel(axis_index, chan_axis);
		chn_engine->GetChnControl(chan)->SetChnAxisAccParam(chan_axis);
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].manual_acc = value.value_double;
		}
		break;
	case 1122:	//起步加速度
		this->m_sc_axis_config[axis_index].start_acc = value.value_double;
//		chn_engine->SetAxisAccParam(axis_index);
		chan = chn_engine->GetAxisChannel(axis_index, chan_axis);
		chn_engine->GetChnControl(chan)->SetChnAxisAccParam(chan_axis);
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].start_acc = value.value_double;
		}
		break;
	case 1130:	//定位S型速度规划时间常数
		this->m_sc_axis_config[axis_index].rapid_s_plan_filter_time = value.value_uint16;
//		chn_engine->SetAxisAccParam(axis_index);
		chan = chn_engine->GetAxisChannel(axis_index, chan_axis);
		chn_engine->GetChnControl(chan)->SetChnAxisAccParam(chan_axis);
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].rapid_s_plan_filter_time = value.value_uint16;
		}
		break;
	case 1135:  //拐角速度差限制
		this->m_sc_axis_config[axis_index].corner_acc_limit = value.value_double;
//		chn_engine->SetAxisSpeedParam(axis_index);
		chan = chn_engine->GetAxisChannel(axis_index, chan_axis);
		chn_engine->GetChnControl(chan)->SetChnAxisSpeedParam(chan_axis);
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].corner_acc_limit = value.value_double;
		}
		break;
	case 1140:	//插补后滤波器类型
		this->m_sc_axis_config[axis_index].post_filter_type = value.value_uint8;
		this->UpdateMiParam<uint8_t>(axis_index+1, 1140, value.value_uint8);   //滤波器类型
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].post_filter_type = value.value_uint8;
		}
		break;
	case 1141:	//插补后滤波器时间常数1
		this->m_sc_axis_config[axis_index].post_filter_time_1 = value.value_uint16;
		this->UpdateMiParam<uint16_t>(axis_index+1, 1141, value.value_uint16);   //一级滤波器时间常数
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].post_filter_time_1 = value.value_uint16;
		}
		break;
	case 1142:	//插补后滤波器时间常数2
		this->m_sc_axis_config[axis_index].post_filter_time_2 = value.value_uint16;
		this->UpdateMiParam<uint16_t>(axis_index+1, 1142, value.value_uint16);   //二级滤波器时间常数
		if(proc_index < kMaxProcParamCount){
			this->m_p_axis_process_param[axis_index].axis_param[proc_index].post_filter_time_2 = value.value_uint16;
		}
		break;
	case 1200:	//电机每转计数
		this->m_sc_axis_config[axis_index].motor_count_pr = value.value_uint32;
		break;
	case 1201:	//电机最大转速
		this->m_sc_axis_config[axis_index].motor_speed_max = value.value_uint32;
		break;
	case 1202:	//每转移动量，即丝杆螺距
		this->m_sc_axis_config[axis_index].move_pr = value.value_double;
		break;
	case 1203:	//电机旋转方向
		this->m_sc_axis_config[axis_index].motor_dir = value.value_uint8;
		break;
	case 1204:	//反馈类型
		this->m_sc_axis_config[axis_index].feedback_mode = value.value_uint8;
		break;
	case 1205:	//轴控制方式
		this->m_sc_axis_config[axis_index].ctrl_mode = value.value_uint8;
		break;
	case 1206:	//每转输出脉冲数
		this->m_sc_axis_config[axis_index].pulse_count_pr = value.value_uint32;
		break;
	case 1207:	//编码器单圈线数
		this->m_sc_axis_config[axis_index].encoder_lines = value.value_uint8;
		break;
	case 1208:	//编码器整圈最大值
		this->m_sc_axis_config[axis_index].encoder_max_cycle = value.value_uint16;
		UpdateMiParam<uint16_t>(axis_index+1, param_no, value.value_uint16);
		break;
	case 1209:  //轴告警电平
//		this->m_sc_axis_config[axis_index].axis_alarm_level = value.value_uint8;
		UpdateMiParam<uint8_t>(axis_index+1, param_no, value.value_uint8);
		break;
	case 1302:  //参考点基准误差
		this->m_sc_axis_config[axis_index].ref_mark_err = value.value_double;
		break;
	case 1303:  //回参考点方式
		this->m_sc_axis_config[axis_index].ret_ref_mode = value.value_uint8;

		if(value.value_uint8 == 0){
			chn_engine->SetRetRefFlag(axis_index, true);
		}else{
			chn_engine->SetRetRefFlag(axis_index, false);
		}
		break;
	case 1304:  //回参考点方向
		this->m_sc_axis_config[axis_index].ret_ref_dir = value.value_uint8;
		break;
	case 1305:  //回参考点换向
		this->m_sc_axis_config[axis_index].ret_ref_change_dir = value.value_uint8;
		break;
	case 1306:  //回参考点信号类型
		this->m_sc_axis_config[axis_index].ref_signal = value.value_uint8;
		break;
	case 1308:  //回参考点速度
		this->m_sc_axis_config[axis_index].ret_ref_speed = value.value_double;
		break;
	case 1309:  //回参考点顺序
		this->m_sc_axis_config[axis_index].ret_ref_index = value.value_uint8;
		break;
	case 1310:  //粗精基准位置偏差检测
		this->m_sc_axis_config[axis_index].ref_base_diff_check = value.value_uint8;
		break;
	case 1312:   //粗精基准位置偏差
		this->m_sc_axis_config[axis_index].ref_base_diff = value.value_double;
		break;
    case 1315:   //回参考点低速
        this->m_sc_axis_config[axis_index].ret_ref_speed_second = value.value_double;
        break;
    case 1316:  //回参考点后的偏移量
        this->m_sc_axis_config[axis_index].ref_offset_pos = value.value_double;
        break;
    case 1317:  //搜索Z脉冲最大移动距离
        this->m_sc_axis_config[axis_index].ref_z_distance_max = value.value_double;
        break;
    case 1318:  //绝对式电机回零方式
        this->m_sc_axis_config[axis_index].absolute_ref_mode = value.value_int8;
        break;
	case 1350:	//手动速度
		this->m_sc_axis_config[axis_index].manual_speed = value.value_double;
		break;
	case 1351:	//定位速度
		this->m_sc_axis_config[axis_index].rapid_speed = value.value_double;
//		chn_engine->SetAxisSpeedParam(axis_index);
		chan = chn_engine->GetAxisChannel(axis_index, chan_axis);
		chn_engine->GetChnControl(chan)->SetChnAxisSpeedParam(chan_axis);
		break;
	case 1352:	//复位速度
		this->m_sc_axis_config[axis_index].reset_speed = value.value_double;
		break;
	case 1400:  //正向反向间隙
		this->m_sc_axis_config[axis_index].backlash_forward = value.value_int16;
		chn_engine->SendMiBacklash(axis_index);
		break;
	case 1401:  //负向反向间隙
		this->m_sc_axis_config[axis_index].backlash_negative = value.value_int16;
		chn_engine->SendMiBacklash(axis_index);
		break;
	case 1402:  //反向间隙补偿生效
		this->m_sc_axis_config[axis_index].backlash_enable = value.value_uint8;
		chn_engine->SendMiBacklash(axis_index);
		break;
	case 1405:  //参考点补偿位置
		break;
	case 1500: 	//软限位1
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
	case 1503: 	//软限位2
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
	case 1506: 	//软限位3
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
	case 1529:  //参考点
		this->m_sc_axis_config[axis_index].axis_home_pos[param_no-1520] = value.value_double;
		chan = g_ptr_chn_engine->GetAxisChannel(axis_index, chan_axis);
		g_ptr_chn_engine->GetChnControl(chan)->SetChnAxisPosThreshold(chan_axis);  //发送给MC
		break;
	case 1550:  //快速定位    0--关闭   1--打开
		this->m_sc_axis_config[axis_index].fast_locate = value.value_uint8;
		break;
		
	case 1551:  //位置显示模式   0--循环模式（0~360）    1--非循环模式
		this->m_sc_axis_config[axis_index].pos_disp_mode = value.value_uint8;
		break;

	case 1602:	//主轴变速比
		this->m_sc_axis_config[axis_index].spd_gear_ratio = value.value_uint8;

		break;
	case 1603:	//零漂补偿
		this->m_sc_axis_config[axis_index].zero_compensation = value.value_uint16;

		break;
	case 1604:	//主轴最高转速
		this->m_sc_axis_config[axis_index].spd_max_speed = value.value_uint32;

		break;
	case 1605: //主轴启动时间
		this->m_sc_axis_config[axis_index].spd_start_time = value.value_uint16;
		UpdateMiParam<uint16_t>(axis_index+1, param_no, value.value_uint16);
		break;
	case 1606: //主轴制动时间
		this->m_sc_axis_config[axis_index].spd_stop_time = value.value_uint16;
		UpdateMiParam<uint16_t>(axis_index+1, param_no, value.value_uint16);
		break;
	case 1607:	//主轴最低转速
		this->m_sc_axis_config[axis_index].spd_min_speed = value.value_uint32;

		break;
	case 1610:	//主轴电压控制方式
		this->m_sc_axis_config[axis_index].spd_vctrl_mode = value.value_uint8;

		break;
	case 1612:	//主轴设置转速(rpm)
		this->m_sc_axis_config[axis_index].spd_set_speed = value.value_uint32;
		break;
	case 1650:  //是否同步轴
		this->m_sc_axis_config[axis_index].sync_axis = value.value_uint8;
		UpdateMiParam<uint8_t>(axis_index+1, param_no, value.value_uint8);
		break;
	case 1651:	//主动轴号
		this->m_sc_axis_config[axis_index].master_axis_no = value.value_uint8;
		UpdateMiParam<uint8_t>(axis_index+1, param_no, value.value_uint8);
		break;
	case 1652:	//显示坐标
		this->m_sc_axis_config[axis_index].disp_coord = value.value_uint8;
		break;
	case 1653:	//主从轴基准位置偏差
		this->m_sc_axis_config[axis_index].benchmark_offset = value.value_double;
		UpdateMiParam<double>(axis_index+1, param_no, value.value_double); 	//基准偏差

		break;
	case 1655:	//位置同步最大误差
		this->m_sc_axis_config[axis_index].sync_err_max = value.value_uint32;
		UpdateMiParam<uint32_t>(axis_index+1, param_no, value.value_uint32); 	//允许的同步最大误差
		break;
	case 1656:	//从动轴回参考点后自动同步校准
		this->m_sc_axis_config[axis_index].auto_sync = value.value_uint8;
		break;
    case 1657:	//SOR信号用途
        this->m_sc_axis_config[axis_index].spd_ctrl_GST = value.value_uint8;
        break;
    case 1658:	//主轴换挡方式
        this->m_sc_axis_config[axis_index].spd_ctrl_SGB = value.value_uint8;
        break;
    case 1659:	//齿轮换挡时是否输出SF信号
        this->m_sc_axis_config[axis_index].spd_ctrl_SFA = value.value_uint8;
        break;
    case 1660:	//主轴定向时的转向
        this->m_sc_axis_config[axis_index].spd_ctrl_ORM = value.value_uint8;
        break;
    case 1661:	//主轴转向是否受M03/M04影响
        this->m_sc_axis_config[axis_index].spd_ctrl_TCW = value.value_uint8;
        break;
    case 1662:	//主轴转向取反
        this->m_sc_axis_config[axis_index].spd_ctrl_CWM = value.value_uint8;
        break;
    case 1663:	//螺纹切削和刚性攻丝时，主轴倍率设置
        this->m_sc_axis_config[axis_index].spd_ctrl_TSO = value.value_uint8;
        break;
    case 1664:	//模拟输出增益
        this->m_sc_axis_config[axis_index].spd_analog_gain = value.value_uint16;
        break;
    case 1665:	//主轴齿轮换档/定向时的主轴转速
        this->m_sc_axis_config[axis_index].spd_sor_speed = value.value_uint16;
        break;
    case 1666:	//主轴电机最小钳制速度
        this->m_sc_axis_config[axis_index].spd_motor_min_speed = value.value_uint16;
        break;
    case 1667:	//主轴电机最大钳制速度
        this->m_sc_axis_config[axis_index].spd_motor_max_speed = value.value_uint16;
        break;
    case 1668:	//齿轮低档位最高转速
        this->m_sc_axis_config[axis_index].spd_gear_speed_low = value.value_uint16;
        break;
    case 1669:	//齿轮中档位最高转速
        this->m_sc_axis_config[axis_index].spd_gear_speed_middle = value.value_uint16;
        break;
    case 1670:	//齿轮高档位最高转速
        this->m_sc_axis_config[axis_index].spd_gear_speed_high = value.value_uint16;
        break;
    case 1671:	//B方式档1->档2电机转速
        this->m_sc_axis_config[axis_index].spd_gear_switch_speed1 = value.value_uint16;
        break;
    case 1672:	//B方式档2->档3电机转速
        this->m_sc_axis_config[axis_index].spd_gear_switch_speed2 = value.value_uint16;
        break;
    case 1673:	//攻丝同步误差增益
        this->m_sc_axis_config[axis_index].spd_sync_error_gain = value.value_uint32;
        break;
    case 1674:	//攻丝轴速度前馈增益
        this->m_sc_axis_config[axis_index].spd_speed_feed_gain = value.value_uint32;
        break;
    case 1675:	//攻丝轴位置比例增益
        this->m_sc_axis_config[axis_index].spd_pos_ratio_gain = value.value_uint32;
        break;
	default:
		g_ptr_trace->PrintLog(LOG_ALARM, "轴参数激活，参数号非法：%d", param_no);
		break;
	}
}

/**
 * @brief 获取指定通道当前打开的NC文件名称
 * @param chn_index : 通道号
 * @param file[out] : 返回文件名称字符串，不超过128字节
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
 * @brief 设置指定通道当前打开的NC文件名称
 * @param chn_index : 通道号
 * @param file : 文件名称字符串
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
 * @brief 设置指定通道的当前工件计数
 * @param chn_index : 通道号
 * @param piece : 工件计数
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
 * @brief 获取指定通道的当前工件计数
 * @param chn_index ： 通道号
 * @return 返回工件计数
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
 * @brief 获取指定通道的总共件数
 * @param chn_index : 通道号
 * @return piece : 工件计数
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
 * @brief 设置指定通道的总共工件
 * @param chn_index : 通道号
 * @param piece : 总共件数
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
 * @brief 设置指定通道的累计加工时间
 * @param chn_index : 通道号
 * @param piece : 累计加工时间
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
 * @brief 获取指定通道的累计加工时间
 * @param chn_index ： 通道号
 * @return 返回累计加工时间
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
 * @brief 获取指定通道的当前刀号
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
 * @brief 设置指定通道的当前刀号
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
 * @brief 获取指定通道的当前工艺参数组号
 * @param chn_index ： 通道号，从0开始
 * @return 返回当前工艺参数组号
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
 * @brief 设置指定通道的当前工艺参数组号
 * @param chn_index ： 通道号，从0开始
 * @param proc_index ： 工艺参数组号
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
 * @brief 更改当前工艺相关参数，将对应工艺参数组的配置复制到当前通道参数和轴参数中
 * @param chn_index ： 通道号，从0开始
 * @param proc_index ： 工艺参数组号，从0开始
 */
void ParmManager::ChangeChnProcParamIndex(uint8_t chn_index, uint8_t proc_index){
	char sname[32];	//section name
	char kname[64];	//key name

	memset(sname, 0x00, sizeof(sname));
	memset(kname, 0x00, sizeof(kname));

	//保存通道参数
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
	//挑角补偿值
	memset(kname, 0x00, sizeof(kname));
	sprintf(kname, "flip_comp_value");
	m_ini_chn->SetIntValue(sname, kname, this->m_sc_channel_config[chn_index].flip_comp_value);
#endif

	m_ini_chn->Save();

	//切换轴参数
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
 * @brief 获取PMC轴个数
 * @return pmc轴的个数
 */
uint8_t ParmManager::GetPmcAxisCount(){
	uint8_t count = 0;
	for(uint8_t i = 0; i < this->m_sc_system_config->axis_count; i++){
		if(this->m_sc_axis_config[i].axis_pmc != 0)
			count++;
	}
	return count;
}
