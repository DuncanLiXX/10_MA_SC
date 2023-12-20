/*
 * parm_manager.h
 *
 *  Created on: 2020年5月6日
 *      Author: M
 */
/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file parm_manager.h
 *@author gonghao
 *@date 2020/05/06
 *@brief 本头文件为参数管理类的声明
 *@version
 */

#ifndef INC_PARAMETER_PARM_MANAGER_H_
#define INC_PARAMETER_PARM_MANAGER_H_

#include "parm_definition.h"
#include "inifile.h"

using namespace inifile;


/**
 * @brief 参数管理类，主要负责所有参数的读取及更新
 */
class ParmManager{
public:
	static ParmManager *GetInstance();    //单例模式，获取此类实例的唯一访问点
	~ParmManager();

	void InitParm();   //初始化参数
	bool ReadParm(ParmType type);   //从文件读取参数

	bool SaveParm(ParmType type);    //保存参数到文件
	bool SaveAllParm();				//保存所有参数到文件

	SCSystemConfig *GetSystemConfig(){return m_sc_system_config;}
	SCChannelConfig *GetChannelConfig(){return m_sc_channel_config;}
	SCChannelConfig *GetChannelConfig(int index);
	ChnProcParamGroup *GetChnProcParam(){return this->m_p_chn_process_param;}   //返回工艺相关通道参数指针
	ChnProcParamGroup *GetChnProcessParam(int index);    //获取工艺相关通道参数，指定通道
	SCAxisConfig *GetAxisConfig(){return m_sc_axis_config;}
	AxisProcParamGroup *GetAxisProcParam(){return this->m_p_axis_process_param;}    //获取工艺相关轴参数
	SCToolOffsetConfig *GetToolConfig(int chn_index = 0){return &m_sc_tool_config[chn_index];}
	SCToolPotConfig *GetToolPotConfig(int chn_index = 0){return &m_sc_tool_pot_config[chn_index];}
	SCCoordConfig *GetCoordConfig(int chn_index = 0){return &m_sc_coord_config[chn_index * kWorkCoordCount];}
	SCCoordConfig *GetExCoordConfig(int chn_index = 0){return m_sc_ex_coord_config[chn_index];}
	AxisPitchCompTable *GetPitchCompData(){return this->m_sc_pc_table;}   //返回螺补数据表
	IoRemapList *GetIoRemapList(){return &this->m_sc_io_remap_list;}      //返回IO重定向数据表
    HandWheelMapInfoVec GetHandWheelVec() {return m_p_handwheel_param; }  //返回手轮通道映射参数





#ifdef USES_FIVE_AXIS_FUNC
	FiveAxisConfig *GetFiveAxisConfig(int chn_index){return &m_five_axis_config[chn_index];}    //返回指定通道的五轴配置
#endif
    SCFiveAxisV2Config *GetFiveAxisV2Config(int chn_index=0){return &m_sc_5axisV2_config[chn_index];} //返回指定通道的新五轴配置

#ifdef USES_GRIND_MACHINE
	GrindConfig *GetGrindConfig(){return m_grind_config;}
	bool IsGrindParamChange(){return m_b_update_grind;}   //是否更改磨床专用参数
	void ResetGrindParamFlag(){m_b_update_grind = false;}  //复位更改标志
#endif


    void ClearToolComp(uint16_t chn_index, int index, int count);
    void ClearToolOffset(uint16_t chn_index, int index, int count);
	void UpdateToolRadiusWear(uint16_t chn_index, uint8_t index, const double &value);  //更新刀具半径磨损值
	void UpdateToolRadiusGeo(uint16_t chn_index, uint8_t index, const double &value);   //更新刀具半径几何偏执
	void UpdateToolWear(uint16_t chn_index, uint8_t index, const double &value);   //更新刀长磨损值
	void UpdateToolMeasure(uint16_t chn_index, uint8_t index, const double &value);   //更新刀具长度测量值
    void UpdateGeoComp(uint16_t chn_index, uint8_t index, uint8_t axis, const double &value);
    void UpdateToolOffsetConfig(uint16_t chn_index, uint8_t index, HmiToolOffsetConfig &cfg, bool active);	//更新刀具偏置信息
    void UpdateAllToolOffsetConfig(uint16_t chn_index, HmiToolOffsetConfig cfg);   //设置所有刀具偏置信息
	void UpdateToolPotConfig(uint16_t chn_index, SCToolPotConfig &cfg);			//更新刀位信息
	void UpdateToolPotConfig(uint16_t chn_index, bool save = true);    //更新刀具信息
	void UpdateCoordConfig(uint16_t chn_index, uint8_t index, HmiCoordConfig &cfg, bool active);    //更新工件坐标系
	void UpdateExCoordConfig(uint16_t chn_index, uint8_t index, HmiCoordConfig &cfg, bool active);	//更新扩展工件坐标系

    void UpdateAllCoordConfig(uint16_t chn_index, double val, bool active); //更新所有工件坐标系
    void UpdateAllExCoordConfig(uint16_t chn_index, double val, bool active, int count);    //更新所有扩展工件坐标系

    bool ActiveCoordParam(uint8_t chn_index);   //激活待生效的工件坐标系更改
	bool ActiveToolOffsetParam(uint8_t chn_index);   //激活待生效的刀具偏置

    bool UpdateAxisParam(uint8_t axis_index, uint32_t param_no, ParamValue &value);	//更新轴参数

	bool UpdateParameter(ParamUpdate *data, uint8_t active_type);			//更新参数

	bool UpdateProcParam(ProcParamUpdate *data, uint8_t active_type);    //更新工艺相关参数

	bool UpdateAxisRef(uint8_t axis, int64_t value);	//更新指定轴的参考点编码器值

    bool UpdateAxisComplete(uint8_t axis, int complete);

	bool UpdatePcData(uint8_t axis_index, bool dir_flag, uint16_t offset, uint16_t count, double *data);    //更新螺补数据

	bool UpdateIoRemapInfo(IoRemapInfo &info);    //更新IO重映射数据
    bool ClearIoRemapInfo();                      //清除IO重映射数据

    bool SyncHandWheelInfo(const HandWheelMapInfoVec &infoVec, bool bRestart = false);       //更新手轮通道映射

	void ActiveResetParam();     //激活复位有效的参数
	void ActiveNewStartParam();		//激活新一次加工有效的参数

	void GetCurNcFile(uint8_t chn_index, char *file);   	//获取指定通道当前打开的NC文件名称
	void SetCurNcFile(uint8_t chn_index, char *file);		//设置指定通道当前打开的NC文件名称
	void SetCurWorkPiece(uint8_t chn_index, uint32_t piece);   //设置指定通道的当前工件计数
	uint32_t GetCurWorkPiece(uint8_t chn_index);          //获取指定通道的当前工件计数
    uint32_t GetTotalWorkPiece(uint8_t chn_index); //获取指定通道的总共件数
    void SetTotalWorkPiece(uint8_t chn_index, uint32_t piece); //设置指定通道的总共件数
    uint32_t GetCurRequirePiece(uint8_t chn_index);         //获取需求件数
    void SetCurRequirePiece(uint8_t chn_index, uint32_t requriePiece);//设置当前需求件数
    void SetCurTotalMachiningTime(uint8_t chn_index, uint32_t totalTime);//设置指定通道的累计加工时间
    uint32_t GetCurTotalMachingTime(uint8_t chn_index);    //获取指定通道的累计加工时间
    void SetCurFileMachiningTime(uint8_t chn_index, uint32_t totalTime);//文件加工时间
    void SetRemainTime(uint8_t chn_index, uint32_t remainTime);
    uint32_t GetRemainTime(uint8_t chn_index);
    uint32_t GetCurFileWorkPieceCnt(uint8_t chn_index);
    void SetCurFileWorkPieceCnt(uint8_t chn_index, uint32_t cnt);
    uint32_t GetCurFileMachiningTime(uint8_t chn_index);
	uint8_t GetCurTool(uint8_t chn_index);                 //获取指定通道的当前刀号
	void SetCurTool(uint8_t chn_index, uint8_t tool);     //设置指定通道的当前刀号
	uint8_t GetCurProcParamIndex(uint8_t chn_index);    //获取指定通道的当前工艺参数号
	void SetCurProcParamIndex(uint8_t chn_index, uint8_t proc_index);    //设置指定通道的当前工艺参数组号
	bool NeedRestart(){return this->m_b_poweroff_param;} 	 //是否需要重启
	uint8_t GetPmcAxisCount();  //获取PMC轴个数
	void ChangeChnProcParamIndex(uint8_t chn_index, uint8_t proc_index);     //更改当前工艺相关参数
    void UpdateMiLimit(uint8_t axis, uint8_t EXLM, uint8_t RLSOT = 0);


private:
	ParmManager();             //构造函数
	bool ReadSysConfig();    	//读取系统配置
	bool ReadChnConfig();		//读取通道配置
	bool ReadAxisConfig();	//读取轴配置
    bool Read5AxisV2Config();  //读取新五轴配置
	bool ReadToolConfig();	//读取刀具配置
	bool ReadCoordConfig();	//读取工件坐标系配置
	bool ReadExCoordConfig();	//读取扩展工件坐标系配置
	bool ReadChnStateScene();	//读取通道状态配置
	bool ReadPcData();          //读取螺补数据
	bool ReadIoRemapConfig();    //读取IO重定向数据

	bool ReadChnProcParam();    //读取工艺相关通道参数
	bool ReadAxisProcParam();   //读取工艺相关轴参数
    bool ReadHandWheelParam();  //读取手轮通道映射参数


#ifdef USES_FIVE_AXIS_FUNC
	bool ReadFiveAxisConfig();	//读取五轴配置
	bool UpdateFiveAxisParam(uint16_t chn_index, uint32_t param_no, ParamValue &value);		//更新五轴参数
	void ActiveFiveAxisParam(uint16_t chn_index, uint32_t param_no, ParamValue &value);		//激活五轴参数
#endif

#ifdef USES_GRIND_MACHINE
	bool ReadGrindConfig();		//读取磨削专用参数
	bool UpdateGrindParam(uint32_t param_no, ParamValue &value);	//更新磨削参数
	void ActiveGrindParam(uint32_t param_no, ParamValue &value);		//激活磨削参数
#endif

//	void CreateDefaultSysConfig();	//生成默认的系统配置文件
//	void CreateDefaultChnConfig();	//生成默认的通道配置文件
//	void CreateDefaultAxisConfig();	//生成默认的轴配置文件
//	void CreateDefaultToolConfig();	//生成默认的刀具配置文件
//	void CreateDefaultCoordConfig();		//生成默认的坐标系配置文件
//	void CreateDefaultExCoordConfig();		//生成默认的扩展坐标系配置文件

	bool UpdateSystemParam(uint32_t param_no, ParamValue &value);	//更新系统参数
	bool UpdateChnParam(uint8_t chn_index, uint32_t param_no, ParamValue &value);		//更新通道参数

    bool Update5AxisV2Param(uint8_t chn_index, uint32_t param_no, ParamValue &value);     //更新新五轴参数

	bool UpdateChnProcParam(uint8_t chn_index, uint8_t group_index, uint32_t param_no, ParamValue &value);   //更新工艺相关通道参数
	bool UpdateAxisProcParam(uint8_t axis_index, uint8_t group_index, uint32_t param_no, ParamValue &value);	//更新工艺相关轴参数

	void ActiveParam(ParamUpdate *data, uint8_t active_type);	//激活参数
	void ActiveParam(ParamUpdate *data); //修改当前参数
	void ActiveSystemParam(uint32_t param_no, ParamValue &value);	//激活系统参数
	void ActiveChnParam(uint8_t chn_index, uint32_t param_no, ParamValue &value, uint8_t proc_index = 0xff);		//激活通道参数
    void Active5AxisV2Param(uint8_t chn_index, uint32_t param_no, ParamValue &value); //激活新五轴参数
    void ActiveAxisParam(uint8_t axis_index, uint32_t param_no, ParamValue &value, uint8_t proc_index = 0xff);	//激活轴参数


	void ActiveProcParam(ProcParamUpdate *data, uint8_t active_type);	//激活工艺相关参数参数
	void ActiveProcParam(ProcParamUpdate *data); //修改当前工艺相关参数参数
	void ActiveChnProcParam(uint8_t chn_index, uint32_t param_no, ParamValue &value, uint8_t proc_index);    //单独更新通道工艺参数
	void ActiveAxisProcParam(uint8_t chn_index, uint32_t param_no, ParamValue &value, uint8_t proc_index);   //单独更新通道轴参数



    template<typename T>
    void UpdateMiParam(uint8_t axis, uint32_t para_no, T data);  //更新发送MI参数

private:
	static ParmManager *m_p_instance;      //单例对象

	SCSystemConfig *m_sc_system_config;		//sc模块系统配置
	SCChannelConfig *m_sc_channel_config;	//sc模块通道配置
	SCAxisConfig *m_sc_axis_config;			//sc模块轴配置
	SCToolOffsetConfig *m_sc_tool_config;	//sc模块刀具偏置配置
	SCToolPotConfig *m_sc_tool_pot_config;	//sc模块刀位信息配置
	SCCoordConfig *m_sc_coord_config;       //sc模块工件坐标系配置	G54~G59
	SCCoordConfig *m_sc_ex_coord_config[kMaxChnCount];    //sc模块扩展工件坐标系配置 G5401~G5499
	AxisPitchCompTable *m_sc_pc_table; //sc模块螺补数据表

	IoRemapList m_sc_io_remap_list;     //IO重定向数据队列

	ChnProcParamGroup *m_p_chn_process_param;    //工艺相关通道参数
	AxisProcParamGroup *m_p_axis_process_param;   //工艺相关轴参数
    HandWheelMapInfoVec m_p_handwheel_param;      //手轮通道映射参数

#ifdef USES_FIVE_AXIS_FUNC
	IniFile *m_ini_five_axis;    //五轴参数配置文件
	FiveAxisConfig *m_five_axis_config;    //五轴配置
#endif
    SCFiveAxisV2Config *m_sc_5axisV2_config; //新五轴配置

	IniFile *m_ini_system;			//系统配置文件
	IniFile *m_ini_chn;				//通道配置文件
	IniFile *m_ini_axis;			//轴配置文件
    IniFile *m_ini_5axisV2;         //新五轴配置文件
	IniFile *m_ini_tool;			//刀具配置文件  包括刀具偏置和刀位信息
	IniFile *m_ini_coord;			//工件坐标系配置文件
	IniFile *m_ini_ex_coord;		//扩展工件坐标系配置文件
	IniFile *m_ini_pc_table;        //螺补数据文件
	IniFile *m_ini_io_remap;        //IO重定向数据文件

	IniFile *m_ini_proc_chn;       //通道工艺相关参数文件
	IniFile *m_ini_proc_axis;      //轴工艺相关参数文件

	IniFile *m_ini_chn_scene;		//通道状态参数文件，保存如当前打开文件等状态

    IniFile *m_ini_handwheel_map;   //手轮通道映射关系

	UpdateParamList *m_list_new_start;	//新一次加工生效的参数
	UpdateParamList *m_list_reset;		//复位生效的参数
	UpdateCoordList *m_list_coord;      //工件坐标系
	UpdateToolOffsetList *m_list_tool_offset;    //刀具偏置

	UpdateProcParamList m_list_proc_new_start;    //新一次加工生效的工艺相关参数
	UpdateProcParamList m_list_proc_reset;    //复位生效的工艺相关参数


	bool m_b_poweroff_param;		//有参数需要重启生效

#ifdef USES_GRIND_MACHINE
	IniFile *m_ini_grind;		//磨削专用参数配置文件
	GrindConfig *m_grind_config;	//磨削参数配置
	bool m_b_update_grind;		//磨削参数更新标志  true--更新   false-未更新
#endif

};



#endif /* INC_PARAMETER_PARM_MANAGER_H_ */
