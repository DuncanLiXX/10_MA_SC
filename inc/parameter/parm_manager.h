/*
 * parm_manager.h
 *
 *  Created on: 2020��5��6��
 *      Author: M
 */
/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file parm_manager.h
 *@author gonghao
 *@date 2020/05/06
 *@brief ��ͷ�ļ�Ϊ���������������
 *@version
 */

#ifndef INC_PARAMETER_PARM_MANAGER_H_
#define INC_PARAMETER_PARM_MANAGER_H_

#include "parm_definition.h"
#include "inifile.h"

using namespace inifile;


/**
 * @brief ���������࣬��Ҫ�������в����Ķ�ȡ������
 */
class ParmManager{
public:
	static ParmManager *GetInstance();    //����ģʽ����ȡ����ʵ����Ψһ���ʵ�
	~ParmManager();

	void InitParm();   //��ʼ������
	bool ReadParm(ParmType type);   //���ļ���ȡ����

	bool SaveParm(ParmType type);    //����������ļ�
	bool SaveAllParm();				//�������в������ļ�

	SCSystemConfig *GetSystemConfig(){return m_sc_system_config;}
	SCChannelConfig *GetChannelConfig(){return m_sc_channel_config;}
	SCChannelConfig *GetChannelConfig(int index);
	ChnProcParamGroup *GetChnProcParam(){return this->m_p_chn_process_param;}   //���ع������ͨ������ָ��
	ChnProcParamGroup *GetChnProcessParam(int index);    //��ȡ�������ͨ��������ָ��ͨ��
	SCAxisConfig *GetAxisConfig(){return m_sc_axis_config;}
	AxisProcParamGroup *GetAxisProcParam(){return this->m_p_axis_process_param;}    //��ȡ������������
	SCToolOffsetConfig *GetToolConfig(int chn_index = 0){return &m_sc_tool_config[chn_index];}
	SCToolPotConfig *GetToolPotConfig(int chn_index = 0){return &m_sc_tool_pot_config[chn_index];}
	SCCoordConfig *GetCoordConfig(int chn_index = 0){return &m_sc_coord_config[chn_index * kWorkCoordCount];}
	SCCoordConfig *GetExCoordConfig(int chn_index = 0){return m_sc_ex_coord_config[chn_index];}
	AxisPitchCompTable *GetPitchCompData(){return this->m_sc_pc_table;}   //�����ݲ����ݱ�
	IoRemapList *GetIoRemapList(){return &this->m_sc_io_remap_list;}      //����IO�ض������ݱ�
    HandWheelMapInfoVec GetHandWheelVec() {return m_p_handwheel_param; }  //��������ͨ��ӳ�����





#ifdef USES_FIVE_AXIS_FUNC
	FiveAxisConfig *GetFiveAxisConfig(int chn_index){return &m_five_axis_config[chn_index];}    //����ָ��ͨ������������
#endif
    SCFiveAxisV2Config *GetFiveAxisV2Config(int chn_index=0){return &m_sc_5axisV2_config[chn_index];} //����ָ��ͨ��������������

#ifdef USES_GRIND_MACHINE
	GrindConfig *GetGrindConfig(){return m_grind_config;}
	bool IsGrindParamChange(){return m_b_update_grind;}   //�Ƿ����ĥ��ר�ò���
	void ResetGrindParamFlag(){m_b_update_grind = false;}  //��λ���ı�־
#endif


    void ClearToolComp(uint16_t chn_index, int index, int count);
    void ClearToolOffset(uint16_t chn_index, int index, int count);
	void UpdateToolRadiusWear(uint16_t chn_index, uint8_t index, const double &value);  //���µ��߰뾶ĥ��ֵ
	void UpdateToolRadiusGeo(uint16_t chn_index, uint8_t index, const double &value);   //���µ��߰뾶����ƫִ
	void UpdateToolWear(uint16_t chn_index, uint8_t index, const double &value);   //���µ���ĥ��ֵ
	void UpdateToolMeasure(uint16_t chn_index, uint8_t index, const double &value);   //���µ��߳��Ȳ���ֵ
    void UpdateGeoComp(uint16_t chn_index, uint8_t index, uint8_t axis, const double &value);
    void UpdateToolOffsetConfig(uint16_t chn_index, uint8_t index, HmiToolOffsetConfig &cfg, bool active);	//���µ���ƫ����Ϣ
    void UpdateAllToolOffsetConfig(uint16_t chn_index, HmiToolOffsetConfig cfg);   //�������е���ƫ����Ϣ
	void UpdateToolPotConfig(uint16_t chn_index, SCToolPotConfig &cfg);			//���µ�λ��Ϣ
	void UpdateToolPotConfig(uint16_t chn_index, bool save = true);    //���µ�����Ϣ
	void UpdateCoordConfig(uint16_t chn_index, uint8_t index, HmiCoordConfig &cfg, bool active);    //���¹�������ϵ
	void UpdateExCoordConfig(uint16_t chn_index, uint8_t index, HmiCoordConfig &cfg, bool active);	//������չ��������ϵ

    void UpdateAllCoordConfig(uint16_t chn_index, double val, bool active); //�������й�������ϵ
    void UpdateAllExCoordConfig(uint16_t chn_index, double val, bool active, int count);    //����������չ��������ϵ

    bool ActiveCoordParam(uint8_t chn_index);   //�������Ч�Ĺ�������ϵ����
	bool ActiveToolOffsetParam(uint8_t chn_index);   //�������Ч�ĵ���ƫ��

    bool UpdateAxisParam(uint8_t axis_index, uint32_t param_no, ParamValue &value);	//���������

	bool UpdateParameter(ParamUpdate *data, uint8_t active_type);			//���²���

	bool UpdateProcParam(ProcParamUpdate *data, uint8_t active_type);    //���¹�����ز���

	bool UpdateAxisRef(uint8_t axis, int64_t value);	//����ָ����Ĳο��������ֵ

    bool UpdateAxisComplete(uint8_t axis, int complete);

	bool UpdatePcData(uint8_t axis_index, bool dir_flag, uint16_t offset, uint16_t count, double *data);    //�����ݲ�����

	bool UpdateIoRemapInfo(IoRemapInfo &info);    //����IO��ӳ������
    bool ClearIoRemapInfo();                      //���IO��ӳ������

    bool SyncHandWheelInfo(const HandWheelMapInfoVec &infoVec, bool bRestart = false);       //��������ͨ��ӳ��

	void ActiveResetParam();     //���λ��Ч�Ĳ���
	void ActiveNewStartParam();		//������һ�μӹ���Ч�Ĳ���

	void GetCurNcFile(uint8_t chn_index, char *file);   	//��ȡָ��ͨ����ǰ�򿪵�NC�ļ�����
	void SetCurNcFile(uint8_t chn_index, char *file);		//����ָ��ͨ����ǰ�򿪵�NC�ļ�����
	void SetCurWorkPiece(uint8_t chn_index, uint32_t piece);   //����ָ��ͨ���ĵ�ǰ��������
	uint32_t GetCurWorkPiece(uint8_t chn_index);          //��ȡָ��ͨ���ĵ�ǰ��������
    uint32_t GetTotalWorkPiece(uint8_t chn_index); //��ȡָ��ͨ�����ܹ�����
    void SetTotalWorkPiece(uint8_t chn_index, uint32_t piece); //����ָ��ͨ�����ܹ�����
    uint32_t GetCurRequirePiece(uint8_t chn_index);         //��ȡ�������
    void SetCurRequirePiece(uint8_t chn_index, uint32_t requriePiece);//���õ�ǰ�������
    void SetCurTotalMachiningTime(uint8_t chn_index, uint32_t totalTime);//����ָ��ͨ�����ۼƼӹ�ʱ��
    uint32_t GetCurTotalMachingTime(uint8_t chn_index);    //��ȡָ��ͨ�����ۼƼӹ�ʱ��
    void SetCurFileMachiningTime(uint8_t chn_index, uint32_t totalTime);//�ļ��ӹ�ʱ��
    void SetRemainTime(uint8_t chn_index, uint32_t remainTime);
    uint32_t GetRemainTime(uint8_t chn_index);
    uint32_t GetCurFileWorkPieceCnt(uint8_t chn_index);
    void SetCurFileWorkPieceCnt(uint8_t chn_index, uint32_t cnt);
    uint32_t GetCurFileMachiningTime(uint8_t chn_index);
	uint8_t GetCurTool(uint8_t chn_index);                 //��ȡָ��ͨ���ĵ�ǰ����
	void SetCurTool(uint8_t chn_index, uint8_t tool);     //����ָ��ͨ���ĵ�ǰ����
	uint8_t GetCurProcParamIndex(uint8_t chn_index);    //��ȡָ��ͨ���ĵ�ǰ���ղ�����
	void SetCurProcParamIndex(uint8_t chn_index, uint8_t proc_index);    //����ָ��ͨ���ĵ�ǰ���ղ������
	bool NeedRestart(){return this->m_b_poweroff_param;} 	 //�Ƿ���Ҫ����
	uint8_t GetPmcAxisCount();  //��ȡPMC�����
	void ChangeChnProcParamIndex(uint8_t chn_index, uint8_t proc_index);     //���ĵ�ǰ������ز���
    void UpdateMiLimit(uint8_t axis, uint8_t EXLM, uint8_t RLSOT = 0);


private:
	ParmManager();             //���캯��
	bool ReadSysConfig();    	//��ȡϵͳ����
	bool ReadChnConfig();		//��ȡͨ������
	bool ReadAxisConfig();	//��ȡ������
    bool Read5AxisV2Config();  //��ȡ����������
	bool ReadToolConfig();	//��ȡ��������
	bool ReadCoordConfig();	//��ȡ��������ϵ����
	bool ReadExCoordConfig();	//��ȡ��չ��������ϵ����
	bool ReadChnStateScene();	//��ȡͨ��״̬����
	bool ReadPcData();          //��ȡ�ݲ�����
	bool ReadIoRemapConfig();    //��ȡIO�ض�������

	bool ReadChnProcParam();    //��ȡ�������ͨ������
	bool ReadAxisProcParam();   //��ȡ������������
    bool ReadHandWheelParam();  //��ȡ����ͨ��ӳ�����


#ifdef USES_FIVE_AXIS_FUNC
	bool ReadFiveAxisConfig();	//��ȡ��������
	bool UpdateFiveAxisParam(uint16_t chn_index, uint32_t param_no, ParamValue &value);		//�����������
	void ActiveFiveAxisParam(uint16_t chn_index, uint32_t param_no, ParamValue &value);		//�����������
#endif

#ifdef USES_GRIND_MACHINE
	bool ReadGrindConfig();		//��ȡĥ��ר�ò���
	bool UpdateGrindParam(uint32_t param_no, ParamValue &value);	//����ĥ������
	void ActiveGrindParam(uint32_t param_no, ParamValue &value);		//����ĥ������
#endif

//	void CreateDefaultSysConfig();	//����Ĭ�ϵ�ϵͳ�����ļ�
//	void CreateDefaultChnConfig();	//����Ĭ�ϵ�ͨ�������ļ�
//	void CreateDefaultAxisConfig();	//����Ĭ�ϵ��������ļ�
//	void CreateDefaultToolConfig();	//����Ĭ�ϵĵ��������ļ�
//	void CreateDefaultCoordConfig();		//����Ĭ�ϵ�����ϵ�����ļ�
//	void CreateDefaultExCoordConfig();		//����Ĭ�ϵ���չ����ϵ�����ļ�

	bool UpdateSystemParam(uint32_t param_no, ParamValue &value);	//����ϵͳ����
	bool UpdateChnParam(uint8_t chn_index, uint32_t param_no, ParamValue &value);		//����ͨ������

    bool Update5AxisV2Param(uint8_t chn_index, uint32_t param_no, ParamValue &value);     //�������������

	bool UpdateChnProcParam(uint8_t chn_index, uint8_t group_index, uint32_t param_no, ParamValue &value);   //���¹������ͨ������
	bool UpdateAxisProcParam(uint8_t axis_index, uint8_t group_index, uint32_t param_no, ParamValue &value);	//���¹�����������

	void ActiveParam(ParamUpdate *data, uint8_t active_type);	//�������
	void ActiveParam(ParamUpdate *data); //�޸ĵ�ǰ����
	void ActiveSystemParam(uint32_t param_no, ParamValue &value);	//����ϵͳ����
	void ActiveChnParam(uint8_t chn_index, uint32_t param_no, ParamValue &value, uint8_t proc_index = 0xff);		//����ͨ������
    void Active5AxisV2Param(uint8_t chn_index, uint32_t param_no, ParamValue &value); //�������������
    void ActiveAxisParam(uint8_t axis_index, uint32_t param_no, ParamValue &value, uint8_t proc_index = 0xff);	//���������


	void ActiveProcParam(ProcParamUpdate *data, uint8_t active_type);	//�������ز�������
	void ActiveProcParam(ProcParamUpdate *data); //�޸ĵ�ǰ������ز�������
	void ActiveChnProcParam(uint8_t chn_index, uint32_t param_no, ParamValue &value, uint8_t proc_index);    //��������ͨ�����ղ���
	void ActiveAxisProcParam(uint8_t chn_index, uint32_t param_no, ParamValue &value, uint8_t proc_index);   //��������ͨ�������



    template<typename T>
    void UpdateMiParam(uint8_t axis, uint32_t para_no, T data);  //���·���MI����

private:
	static ParmManager *m_p_instance;      //��������

	SCSystemConfig *m_sc_system_config;		//scģ��ϵͳ����
	SCChannelConfig *m_sc_channel_config;	//scģ��ͨ������
	SCAxisConfig *m_sc_axis_config;			//scģ��������
	SCToolOffsetConfig *m_sc_tool_config;	//scģ�鵶��ƫ������
	SCToolPotConfig *m_sc_tool_pot_config;	//scģ�鵶λ��Ϣ����
	SCCoordConfig *m_sc_coord_config;       //scģ�鹤������ϵ����	G54~G59
	SCCoordConfig *m_sc_ex_coord_config[kMaxChnCount];    //scģ����չ��������ϵ���� G5401~G5499
	AxisPitchCompTable *m_sc_pc_table; //scģ���ݲ����ݱ�

	IoRemapList m_sc_io_remap_list;     //IO�ض������ݶ���

	ChnProcParamGroup *m_p_chn_process_param;    //�������ͨ������
	AxisProcParamGroup *m_p_axis_process_param;   //������������
    HandWheelMapInfoVec m_p_handwheel_param;      //����ͨ��ӳ�����

#ifdef USES_FIVE_AXIS_FUNC
	IniFile *m_ini_five_axis;    //������������ļ�
	FiveAxisConfig *m_five_axis_config;    //��������
#endif
    SCFiveAxisV2Config *m_sc_5axisV2_config; //����������

	IniFile *m_ini_system;			//ϵͳ�����ļ�
	IniFile *m_ini_chn;				//ͨ�������ļ�
	IniFile *m_ini_axis;			//�������ļ�
    IniFile *m_ini_5axisV2;         //�����������ļ�
	IniFile *m_ini_tool;			//���������ļ�  ��������ƫ�ú͵�λ��Ϣ
	IniFile *m_ini_coord;			//��������ϵ�����ļ�
	IniFile *m_ini_ex_coord;		//��չ��������ϵ�����ļ�
	IniFile *m_ini_pc_table;        //�ݲ������ļ�
	IniFile *m_ini_io_remap;        //IO�ض��������ļ�

	IniFile *m_ini_proc_chn;       //ͨ��������ز����ļ�
	IniFile *m_ini_proc_axis;      //�Ṥ����ز����ļ�

	IniFile *m_ini_chn_scene;		//ͨ��״̬�����ļ��������統ǰ���ļ���״̬

    IniFile *m_ini_handwheel_map;   //����ͨ��ӳ���ϵ

	UpdateParamList *m_list_new_start;	//��һ�μӹ���Ч�Ĳ���
	UpdateParamList *m_list_reset;		//��λ��Ч�Ĳ���
	UpdateCoordList *m_list_coord;      //��������ϵ
	UpdateToolOffsetList *m_list_tool_offset;    //����ƫ��

	UpdateProcParamList m_list_proc_new_start;    //��һ�μӹ���Ч�Ĺ�����ز���
	UpdateProcParamList m_list_proc_reset;    //��λ��Ч�Ĺ�����ز���


	bool m_b_poweroff_param;		//�в�����Ҫ������Ч

#ifdef USES_GRIND_MACHINE
	IniFile *m_ini_grind;		//ĥ��ר�ò��������ļ�
	GrindConfig *m_grind_config;	//ĥ����������
	bool m_b_update_grind;		//ĥ���������±�־  true--����   false-δ����
#endif

};



#endif /* INC_PARAMETER_PARM_MANAGER_H_ */
