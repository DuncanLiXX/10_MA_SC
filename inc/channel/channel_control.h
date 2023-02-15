/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file channel_control.h
 *@author gonghao
 *@date 2020/03/19
 *@brief ��ͷ�ļ�Ϊͨ�������������
 *ͨ�����������Ҫ�����ǹ���ͨ��״̬������ͨ����������
 *@version
 */

#ifndef INC_CHANNEL_CHANNEL_CONTROL_H_
#define INC_CHANNEL_CHANNEL_CONTROL_H_

#include "compiler.h"
#include "channel_data.h"
#include "variable.h"
#include <pthread.h>

//ǰ������
class ChannelEngine;   //ͨ��������
class HMICommunication; //HMIͨѶ��
class ParmManager;      //���ù�����
class MICommunication;  //MIͨѶ��
class MCCommunication;  //MCͨѶ��
class MCArmCommunication;  //MC-ARMͨѶ��
class PmcRegister;		//PMC�Ĵ�����
class SpindleControl;   //���������
struct SCSystemConfig; //SCͨ������
struct SCChannelConfig;  //SCͨ������
struct SCAxisConfig;     //SC������
struct FRegBits;		//F�Ĵ�����λ����
struct GRegBits;		//G�Ĵ�����λ����

/**
 * @brief ͨ��������
 */
class ChannelControl {
public:
	ChannelControl();	//���캯��
	virtual ~ChannelControl(); //��������

	// @test zk
    //void test();
	// @test zk

	bool Initialize(uint8_t chn_index, ChannelEngine *engine, HMICommunication *hmi_comm,
			MICommunication *mi_comm, MCCommunication *mc_comm, ParmManager *parm, PmcRegister *reg);  //��ʼ������
	
	void SetMcArmComm(MCArmCommunication *comm){this->m_p_mc_arm_comm = comm;}   //����MC-ARMͨѶ�ӿ�

    SpindleControl *GetSpdCtrl(){return m_p_spindle;}
    const GRegBits *GetGReg(){return m_p_g_reg;}
    PmcAxisCtrl *GetPmcAxisCtrl();
    void RefreshPmcAxis();


//	bool OpenFile(const char *file_name);   //�򿪴������ļ�

	ErrorType GetErrorCode(){return m_error_code;}   //���ص�ǰ������

	const ChannelStatusCollect &GetChnStatus(){return this->m_channel_status;}  //��ȡ��ǰͨ��״̬�ṹ��
    const ChannelRealtimeStatus &GetRealtimeStatus(){return m_channel_rt_status;} //��ȡʵʱ״̬�ṹ��

	uint8_t GetChnAxisCount(){return this->m_p_channel_config->chn_axis_count;}   //��ȡͨ��������
	uint8_t GetChnAxisName(uint8_t idx){return this->m_p_channel_config->chn_axis_name[idx];}   //��ȡͨ��������

	void Pause(); 			//ֹͣG����

	void StartRunGCode();  //��ʼG��������
	void StopRunGCode(bool reset = true);	//ֹͣG��������
	void StopCompilerRun();		//ֹͣ����


	void SendMonitorData(bool bAxis, bool btime);   //���ͼ������

	void SendSimulateDataToHmi(MonitorDataType type, SimulateData &data);   //���ͷ������ݸ�HMI

	void ProcessHmiCmd(HMICmdFrame &cmd);  //����HMIָ��

	void GetChnStatus(HmiChannelStatus &status);  //��ȡͨ��״̬

	uint8_t GetChnWorkMode(){return m_channel_status.chn_work_mode;}   //��ȡͨ����ǰ����ģʽ

	void SetWorkMode(uint8_t work_mode);  //�л�����ģʽ

	bool IsRapidManualMove(){return m_channel_status.func_state_flags.CheckMask(FS_MANUAL_RAPID);}       //�Ƿ�����ֶ��ƶ�

	uint8_t GetChnIndex(){return  m_n_channel_index;}   //����ͨ��������

    void CompileOver();		//�����������

    void ResetMcLineNo();    //��λMCģ��ĵ�ǰ�ӹ��кţ�����

    bool IsBlockRunOver();   //�Ƿ�����е�λ

    bool IsMachinRunning();    //�Ƿ��������У������Զ����У��ӹ����棬��·����

	bool IsStepMode(); 		//�Ƿ񵥶�ģʽ,ֻ�����Զ�ģʽ�£����β���Ч

    void SetCurLineNo(uint32_t line_no);   	//���õ�ǰ�к�
    void SetCurLineNoFromMc();   //���õ�ǰ�кŴ�MCģ���ȡ


    CompilerState GetCompileState(){return m_n_run_thread_state;}    //��ȡG�����������״̬������/��ͣ/ֹͣ

//	bool StartCompile(const char *file);		//��ʼ����
//	void ContinueCompile(){m_n_thread_state = RUN;}	//��������

	bool SetSimulateMode(const SimulateMode mode);   //���÷���ģʽ
	SimulateMode GetSimulateMode(){return m_simulate_mode;}  	//���ط���ģʽ
	SimulateMode *GetSimulateModePtr(){return &m_simulate_mode;}  //���ط���ģʽ����ָ��


	bool OutputData(RecordMsg *msg, bool flag_block = false);     //��MC���ͱ���G��������
	bool OutputSimulateData(RecordMsg *msg);   //��HMI���ͷ�������

	uint8_t GetPhyAxis(uint8_t chn_axis);  //��ȡͨ�����Ӧ�������������ţ�0��ʼ
	uint8_t GetSpdChnAxis(uint8_t spd_idx);  //��ȡָ����ŵ������Ӧ��ͨ����ţ�0��ʼ

	double GetToolCompRadius(int d_index);    //��ȡ��ӦDֵ�õ��߰뾶����ֵ   �뾶+ĥ��

	void SetFuncState(int state, uint8_t mode = 10);		//���ù���״̬�����磺���Σ�ѡͣ�ȵ�
	void SetAutoRatio(uint8_t ratio);    //�����Զ�����
	void SetManualRatio(uint8_t ratio);	//�����ֶ�����
	void SetRapidRatio(uint8_t ratio);	//���ÿ��ٽ�������
    void SetSpindleRatio(uint8_t ratio);	//�������ᱶ��
    void SetManualStep(uint8_t step);	//�����ֶ�����
	void SetManualRapidMove(uint8_t mode = 10);			//�����ֶ������ƶ�״̬
	void SetCurAxis(uint8_t axis);		//���õ�ǰ��

	uint8_t GetAutoRatio(){return this->m_channel_status.auto_ratio;}   //���ص�ǰ�Զ�����
	uint8_t GetManualRatio(){return this->m_channel_status.manual_ratio;}     //���ص�ǰ�ֶ�����
	uint8_t GetRapidRatio(){return this->m_channel_status.rapid_ratio;}       //���ص�ǰ���ٽ�������
	uint8_t GetSpindleRatio(){return this->m_channel_status.spindle_ratio;}   //���ص�ǰ���ᱶ��
	uint8_t GetManualStep(){return this->m_channel_status.manual_step;}       //���ص�ǰ�ֶ�����

	void SetMcAutoBufMax(uint16_t count){this->m_n_mc_auto_buf_max = count;}   //����MC�Զ�ģʽ�˶����ݻ�������
	uint16_t GetMcAutoBufMax(){return this->m_n_mc_auto_buf_max;}   //��ȡMC��ͨ���Զ����ݻ�������

	void ManualMove(int8_t dir);		//�ֶ��ƶ�
    //void ManualMove2(uint8_t axis, int8_t dir, double vel, double inc_dis);  //�ֶ��ƶ�����dir�����ƶ�dis����
	void ManualMoveStop();			//ֹͣ��ǰ���ֶ��ƶ�
	void ManualMoveStop(uint16_t axis_mask);   //ֹͣ����ֶ��ƶ�

	void ManualMove(uint8_t axis, double pos, double vel, bool workcoord_flag = false);   //�ֶ�ָ��ĳ������ָ���ٶ��˶���ָ��λ��

	void ManualMovePmc(int8_t dir);		//�ֶ��ƶ�PMC��
	void ManualMovePmc2(uint8_t axis, int8_t dir, double vel, double inc_dis);   //ָ��PMC���ֶ��ƶ�
	void ManualMovePmc(uint8_t axis, double pos, double vel);   //�ֶ�ָ��ĳ��PMC����ָ���ٶ��˶���ָ��λ��
	void ManualMovePmcStop();			//ֹͣ��ǰPMC���ֶ��ƶ�
	void ManualMovePmcStop(uint8_t phy_axis);   //ֹͣPMC����ֶ��ƶ�
	void PausePmcAxis(uint8_t phy_axis, bool flag);          //��ͣPMC���ƶ�
	void StopPmcAxis(uint8_t phy_axis);       //ֹͣPMC����ƶ�

    // ����Ƿ�����λ����
    // dir: ����
    // phy_axis: ������� ��0��ʼ
    // pos: Ŀ��λ�� mm
    // ����ֵ�� 0��û�г���   1������
    bool CheckSoftLimit(ManualMoveDir dir, uint8_t phy_axis, double pos);
    // ��ȡ��λλ��
    bool GetSoftLimt(ManualMoveDir dir, uint8_t phy_axis, double &limit);

	void HandwheelMove(int32_t hw_count);   //�����ƶ�ָ��

	void ManualToolMeasure(int h_code, int times);      //�ֶ��Ե�����

	void InitMcIntpMdaBuf();		//֪ͨMC����ͨ��MDA�岹�����ʼ��
	void InitMcIntpAutoBuf();	//֪ͨMC����ͨ���Զ����ݻ����ʼ��

	void SetMcStepMode(bool flag);   //����MC����ģʽ

	Variable *GetMacroVar(){return &m_macro_variable;}   //���غ��������ָ��
	int GetMacroVar(const int index, double &value);   //����ָ���������ֵ
	bool GetSysVarValue(const int index, double &value);      //��ȡϵͳ����ֵ
	bool SetSysVarValue(const int index, const double &value);      //����ϵͳ����

	bool IsOutputMsgRunover();   //�����Ƿ��д����е�ָ��


	bool EmergencyStop();		//��ͣ����

	void Reset();               //��λͨ��״̬

	void ProcessHmiSetRefCmd(HMICmdFrame &cmd);			//�������òο�������

	void SetMcCoord(bool flag);  	//����MC������ϵԭ��
	void SetChnAxisName();		//����ͨ��������

	void SetChnAxisOn(uint8_t chn_axis);			//������ʹ�ܣ��Լ��岹ģʽ��1--NC��岹   2--PMC��岹��
	void SetChnAxisBaseParam(uint8_t chn_axis);		//����ָ������ݾ༰����ٶȵȻ�����Ϣ
	void SetChnAxisSpeedParam(uint8_t chn_axis);		//����ָ������ٶ������Ϣ
	void SetChnAxisAccParam(uint8_t chn_axis);			//����ָ������ٶ������Ϣ
	void SetChnAxisSoftLimit(uint8_t chn_axis);   //����ָ���������λ����
	void CloseChnAxisSoftLimit(uint8_t chn_axis);  //ǿ�ƹر�ָ���������λ����
	void SetChnAxisSoftLimitValue(uint8_t chn_axis, uint8_t index);   //����ָ���������λֵ
	void SetChnAllAxisParam(void);
	void SetChnAxisPosThreshold(uint8_t chn_axis);   //����ָ�����λ��ָ��
	
	void SetMachineState(uint8_t mach_state);     //���üӹ�״̬

	void SendMcSysResetCmd();	//��MC����ϵͳ��λָ��

	void SetMcChnPlanMode();			//���üӹ��ٶȹ滮��ʽ
	void SetMcChnPlanParam();			//����ͨ���ӹ��ٶȹ滮����
	void SetMcChnPlanFun();			//����ͨ���ӹ��ٶȹ��ܿ���
	void SetMcChnCornerStopParam();		//���ùս�׼ͣ����

#ifdef USES_WOOD_MACHINE
	void SetMcFlipCompParam();     //����MC�����ǲ���ֵ
	void SetMcDebugParam(uint16_t index);   //����MC�ĵ��Բ���

#endif


	double GetAxisCurIntpTarPos(uint8_t axis_index, bool bMachCoord);  //��ȡ����ĵ�ǰ�岹Ŀ��λ��
	double GetAxisCurWorkPos(uint8_t axis_index);   //��ȡ����ĵ�ǰ��������ϵλ��
	double GetAxisCurMachPos(uint8_t axis_index);	//��ȡ����ĵ�ǰ��е����ϵλ��
	double GetAxisCurFedBckAxisSpeed(uint8_t axis_index);	//��ȡ����ĵ�ǰ�����ٶ�
	double GetAxisCurFedBckAxisTorque(uint8_t axis_index);	//��ȡ����ĵ�ǰ��������
	int GetCurManualStep();	//��ȡ��ǰ�ֶ�������������

	bool SendOpenFileCmdToHmi(char *filename); 	//��HMI�����л�NC�ļ���ʾ����

	void SpindleOut(int dir, int speed=0);			//�������

	void UpdateCoord(uint8_t index, HmiCoordConfig &cfg);  //���¹�������ϵ
	void UpdateExCoord(uint8_t index, HmiCoordConfig &cfg);  //������չ��������ϵ
    bool UpdateAllCoord(double val);//�������й�������ϵΪ�趨ֵ
    bool UpdateAllExCoord(double val, int count);//�������е���չ��������ϵ�趨ֵ

	void UpdateToolOffset(uint8_t index, HmiToolOffsetConfig &cfg);   //���µ���ƫ��
    bool UpdateAllToolOffset(const HmiToolOffsetConfig &cfg);

	bool IsCurNcFile(char *file_name);    //file_name�Ƿ�Ϊ��ǰ�ӹ��ļ�

	void RefreshFile(char *file);   //ˢ��nc�ļ�file
	bool RemapFile(char *file);			//����ӳ�䵱ǰ���ص�NC�ļ�

	void SetClearPosAxisMask(uint8_t axis_index);   //�޸�������Ȧλ�õ���mask

	void SendMiChnAxisMap();   //����ͨ����-������ӳ���MI
#ifdef USES_WUXI_BLOOD_CHECK
	bool ReturnRef(uint8_t axis_mask);		//ָ����زο���
#endif

	void SaveKeepMacroVar();      //�������ʧ�Ժ����

    bool CheckAxisSrvOn(uint64_t &flag);   //��������ŷ�״̬

	void ResetOPSignal();   //��λOP�ź�
	void ResetRSTSignal();	//��λRST�ź�
	void SetALSignal(bool value);  //���ø澯�ź�

	void TransMachCoordToWorkCoord(DPointChn &pos, uint16_t coord_idx, uint16_t h_code, uint32_t axis_mask);   //�������ɻ�е����ϵת��Ϊ��������ϵ
	void TransMachCoordToWorkCoord(DPointChn &pos, uint16_t coord_idx, uint32_t axis_mask);   //�������ɻ�е����ϵת��Ϊ��������ϵ
	void TransMachCoordToWorkCoord(DPoint &pos, uint16_t coord_idx, uint32_t axis_mask);   //�������ɻ�е����ϵת��Ϊ��������ϵ
	void TransWorkCoordToMachCoord(DPointChn &pos, uint16_t coord_idx, uint32_t axis_mask);   //�������ɹ�������ϵת��Ϊ��е����ϵ
	void TransWorkCoordToMachCoord(DPoint &pos, uint16_t coord_idx, uint32_t axis_mask);   //�������ɹ�������ϵת��Ϊ��е����ϵ
    void TransMachCoordToWorkCoord(double &pos, uint16_t coord_idx, uint8_t axis);    //�����������ɻ�е����ϵת��Ϊ��������ϵ
    void TransWorkCoordToMachCoord(double &pos, uint16_t coord_idx, uint8_t axis);    //�����������ɹ�������ϵת��Ϊ��е����ϵ
	void TransWorkCoordToMachCoord(double &pos, uint16_t coord_idx, uint16_t h_code, uint8_t axis);  //�����������ɹ�������ϵת��Ϊ��е����ϵ

	void ProcessHmiReturnRefCmd(bool flag);                   //�زο���ִ�к���
	void SetRefPointFlag(uint8_t chn_axis, bool flag);    //����ͨ����زο����־

	bool CheckFuncState(int func);      //ͨ���Ƿ���func״̬

	bool CheckGRegState(int sec, int bit);  //��ȡG��ַ�Ĵ���״̬
	bool CheckFRegState(int sec, int bit);  //��ȡF��ַ�Ĵ���״̬
	bool SetFRegValue(int sec, int bit, bool value);  //����F��ַ�Ĵ�����ֵ
	uint32_t GetMacroVar1032();    //��ȡ#1032��ֵ
	uint16_t GetMacroVar1132();   //��ȡ#1132��ֵ
	uint32_t GetMacroVar1133();     //��ȡ#1133��ֵ

	void SetMacroVar1132(uint16_t value);  //����#1132��ֵ
	void SetMacroVar1133(uint32_t value);  //����#1133��ֵ

	bool IsSkipCaptured(){return m_b_pos_captured;}    //SKIP�ź��Ƿ񲶻�

#ifdef USES_GRIND_MACHINE
	void SetMechArmParam(ParamMechArm *para){this->m_p_mech_arm_param = para;}      //���û�е�ֲ���
	void SetMechArmState(StateMechArm *state){this->m_p_mech_arm_state = state;}    //���û�е��״̬

	void RunM66_slave();      //���дӻ�������
#endif

#ifdef USES_WUXI_BLOOD_CHECK
	bool ReturnHome();		//�زο���   ������Ŀ����
	void SetRetHomeFlag();    //���û����־
	bool IsReturnHome(){return m_b_returnning_home;}   	//���ػ����־
#endif

#ifdef USES_LASER_MACHINE
	void ProcessHmiCalibCmd(bool bstart);			//����HMI�������궨ָ��
	void ProcessMiInputSignal(MiCmdFrame &cmd);   //����MI���صļ�������������ź�ֵ
	void ProcessMiPosCapture(MiCmdFrame &cmd);		//����Mi���ص���λ�ò�����

	void EnableHeightRegulatorFollow(bool enable);   //����������ʹ��
#endif

#ifdef USES_FIVE_AXIS_FUNC
	void UpdateFiveAxisRotParam();      //����������ת�����ز���
	bool IsFiveAxisMach(){return m_p_chn_5axis_config->five_axis_type==NO_FIVEAXIS?false:true;}   //�Ƿ�������
	uint8_t GetFiveAxisRot1(){return this->m_n_5axis_rot1;}    //���ص�һ��ת���ͨ����������
	uint8_t GetFiveAxisRot2(){return this->m_n_5axis_rot2;}    //���صڶ���ת���ͨ����������
	uint8_t GetFiveAxisRotMaskNoLimt(){return this->m_mask_5axis_rot_nolimit;}  //��������������ת���mask

	void SetMcChnFiveAxisParam();			//��ʼ��ͨ��������ز���
	void UpdateFiveAxisParam(FiveAxisParamType type);   //�����������
#endif


#ifdef USES_WOOD_MACHINE
	bool IsToolLifeChanged(){return this->m_b_save_tool_info;}   //�Ƿ񵶾������ı�
#endif

	void DoIdle(uint32_t count);     //���д�����

	void SyncMacroVar();       //ͬ����������

	void ProcessSkipCmdRsp(MiCmdFrame &cmd);    //����MI���ص���ת������Ӧ

	void SetAxisNameEx(bool flag);   //������������չ�±�ʹ��

	bool PmcAxisRunOver(MiCmdFrame cmd);    //PMC�����е�λ

	void DPoint2DPointChn(const DPoint &src, DPointChn &tar);  //��DPoint����任ΪDPointChn����
	void DPointChn2DPoint(const DPointChn &src, DPoint &tar);  //��DPointChn����任ΪDPoint����

	void PrintDebugInfo();		//���DSP-MC��������

	void PrintDebugInfo1();		//���MI-MC��������

	void ProcessHmiManualToolMeasureCmd(HMICmdFrame &cmd);   //�����ֶ��Ե�ָ��

	void ProcessMiHWTraceStateChanged(MiCmdFrame &cmd);   //����MI���͵����ָ���״̬�л�ָ��
	bool ChangeHwTraceState(HWTraceState state);    //�л����ָ���״̬

    bool CallMacroProgram(uint16_t macro_index);      //���ú����

	uint8_t GetCurProcParamIndex(){return this->m_n_cur_proc_group;}   //���ص�ǰ���ղ������
	bool SetCurProcParamIndex(uint8_t index);  //���õ�ǰ���ղ������

	void GetMdaFilePath(char *path);   //����ͨ��MDA�ļ�����
	
	void SetHmiGraphMode(bool flag){this->m_b_hmi_graph = flag;}   //����HMIͼ����ʾģʽ

    void ClearMachineTimeTotal();

    void UpdateModeData(uint16_t mode_type, int value);//��SCֱ�Ӹ���Dģ̬����

#ifdef USES_ADDITIONAL_PROGRAM
	bool CallAdditionalProgram(AddProgType type);  //���ø��ӳ���ǰ��/���ã�
#endif

	void test(); // @test zk
	// @ add zk
	void StraightFeed(int chn, double x, double y, double z, int feed);
	void StraightTraverse(int chn, double x, double y, double z);
	void g73_func();
	// @ add zk

    uint8_t GetPhyAxisFromName(uint8_t axis_name);   //��ȡ��Ӧͨ�������Ƶ������������ţ�0��ʼ
    uint8_t GetChnAxisFromName(uint8_t axis_name);   //��ȡ��Ӧͨ�������Ƶ�ͨ���������ţ�0��ʼ
    uint8_t GetChnAxisFromPhyAxis(uint8_t phy_axis); //��ö�Ӧ�������ͨ���������ţ�0��ʼ

    void SyncMcPosition();  // ͬ��λ��

private:
	void InitialChannelStatus();		//��ʼ��ͨ��״̬
    static void *CompileThread(void *args);  //G���������̺߳���
    int Run();       //G���������к���


    static void *BreakContinueThread(void *args);  //�ϵ����ִ���̺߳���
    int BreakContinueProcess();		//�ϵ����ִ�к���

    void ResetMode();       //��λͨ��ģ̬

//  bool AddOutputData(const GCodeFrame &data);   //���˶���������֡ѹ�뻺��
//	bool OutputLastBlockItem();   	//���˶����ݻ����е����ݶ��������λ�������־
//	bool OutputAllData();    			//���˶����ݻ����е����ݶ����,����λ�������־

    void SaveAutoScene(bool flag = true);		//�����Զ�ģʽ�ӹ���ͣʱ��״̬
    void ReloadAutoScene();		//�ָ��Զ��ӹ���ͣʱ��״̬
    bool StartBreakpointContinue();	//�����Զ�ģʽ�ϵ������������

	bool ExecuteMessage();       //������Ϣִ��ģ��
	bool ExecuteAuxMsg(RecordMsg *msg);    	//ʵ��ִ�и���ָ����Ϣ
	bool ExecuteLineMsg(RecordMsg *msg, bool flag_block);   	//ʵ��ִ��ֱ������ָ����Ϣ
	bool ExecuteRapidMsg(RecordMsg *msg, bool flag_block);  	//ʵ��ִ�п��ٶ�λָ����Ϣ
	bool ExecuteArcMsg(RecordMsg *msg, bool flag_block);  		//ʵ��ִ��Բ������ָ����Ϣ
	bool ExecuteCoordMsg(RecordMsg *msg);  	//ʵ��ִ������ϵָ��ָ����Ϣ
	bool ExecuteToolMsg(RecordMsg *msg);  	//ʵ��ִ�е���ָ����Ϣ
	bool ExecuteModeMsg(RecordMsg *msg);  	//ʵ��ִ��һ��ģָ̬����Ϣ������������ģָ̬��
	bool ExecuteFeedMsg(RecordMsg *msg);  	//ʵ��ִ�н����ٶ�ָ��ָ����Ϣ
	bool ExecuteSpeedMsg(RecordMsg *msg);  	//ʵ��ִ������ת��ָ����Ϣ
	bool ExecuteLoopMsg(RecordMsg *msg);  	//ʵ��ִ��ѭ��ָ����Ϣ
	bool ExecuteCompensateMsg(RecordMsg *msg);  //ʵ��ִ�е���ָ����Ϣ
	bool ExecuteErrorMsg(RecordMsg *msg);  	//ʵ��ִ�д���ָ����Ϣ
	bool ExecuteSubProgCallMsg(RecordMsg *msg);  //ʵ��ִ���ӳ��������Ϣ
	bool ExecuteMacroCmdMsg(RecordMsg *msg);     //ִ�к�ָ����Ϣ
	bool ExecuteSubProgReturnMsg(RecordMsg *msg);	//ִ���ӳ����˳���Ϣ
	bool ExecutePolarIntpMsg(RecordMsg *msg);	//ִ�м�����岹��ĥ������Ϣ
	bool ExecuteClearCirclePosMsg(RecordMsg *msg);    //ִ��������Ȧ��Ϣ
	bool ExecuteTimeWaitMsg(RecordMsg *msg);		//ִ����ʱ��Ϣ
	bool ExecuteRefReturnMsg(RecordMsg *msg);	//ִ�вο��㷵����Ϣ
	bool ExecuteSkipMsg(RecordMsg *msg);	//ִ�вο��㷵����Ϣ
	bool ExecuteMacroProgCallMsg(RecordMsg *msg); //ʵ��ִ�к���������Ϣ
	bool ExecuteAutoToolMeasureMsg(RecordMsg *msg);   //ʵ��ִ���Զ��Ե�ָ����Ϣ
	bool ExecuteRestartOverMsg(RecordMsg *msg);   //ʵ��ִ�мӹ���λ���ָ����Ϣ
	bool ExecuteInputMsg(RecordMsg *msg);		//ִ��G10 ����ָ����Ϣ
	bool ExecuteExactStopMsg(RecordMsg *msg);   // G09

#ifdef USES_SPEED_TORQUE_CTRL	
	bool ExecuteSpeedCtrlMsg(RecordMsg *msg);   //ִ���ٶȿ�����Ϣ
	bool ExecuteTorqueCtrlMsg(RecordMsg *msg);   //ִ�����ؿ�����Ϣ
#endif	

	void PauseRunGCode();  //��ͣG��������

	void InitGCodeMode();    //��G����ģ̬��ʼ����Ĭ��״̬

	bool RefreshOuputMovePos(DPointChn &pos);   //ͬ���ѱ�������ƶ�ָ���λ��
//	bool RefreshOutputMovePos(DPoint &pos);  //ͬ���ѱ�������ƶ�ָ���λ��
	bool IsMoveMsgLine();   //ͬ���Ƿ������ƶ�ָ��

	bool IsNcFileExist(char *file_name);    //NC�ļ��Ƿ����


	void ProcessHmiGetChnStateCmd(HMICmdFrame &cmd);   //����HMI��ȡͨ��״ָ̬��
	void ProcessHmiSimulateCmd(HMICmdFrame &cmd);      //����HMI����ָ��
	void ProcessHmiSetNcFileCmd(HMICmdFrame &cmd);     //����HMI���üӹ��ļ�����
	void ProcessMdaData(HMICmdFrame &cmd);				//����MDA����
	void ProcessHmiFindRefCmd(HMICmdFrame &cmd);			//����زο�������
	void ProcessHmiGetMacroVarCmd(HMICmdFrame &cmd);	//�����ȡ���������
	void ProcessHmiSetMacroVarCmd(HMICmdFrame &cmd);	//�������ú��������
	void ProcessHmiClearWorkPieceCmd(HMICmdFrame &cmd);  //����ӹ�������������
    void ProcessHmiClearTotalPieceCmd(HMICmdFrame &cmd); //�����ܹ�������������
	void ProcessHmiRestartCmd(HMICmdFrame &cmd);    //����ӹ���λ����
    void ProcessHmiSetRequirePieceCmd(HMICmdFrame &cmd);    //����

	void ProcessHmiSetCurMachPosCmd(HMICmdFrame &cmd);      //����HMI�����ᵱǰλ�õĻ�е��������

	void ProcessHmiClearMsgCmd(HMICmdFrame &cmd);     //����HMI�����Ϣ����


#ifdef USES_LASER_MACHINE
	void ProcessLaserCalibration();        //������������궨


    void ReadHeightRegulatorInput();			//��ȡ�����������ź�
	void SetHeightRegulatorOutput(uint8_t idx, bool value);  //���õ���������ź�
	void SendCatchPosCmd(uint64_t axis_mask, uint16_t io_idx, bool level);         //��MI������λ�ò���ָ��
#endif


    bool SendMachineStateCmdToHmi(uint8_t mach_state);     //��HMI���ͼӹ�״̬����
	bool SendWorkModeCmdToHmi(uint8_t chn_work_mode); 		//��HMI���͹���ģʽ����
	bool SendChnStatusChangeCmdToHmi(uint16_t status_type);	//��HMI����ͨ��״̬�ı�����
	bool SendModeChangToHmi(uint16_t mode_type);			//��HMI���ͷ�Gģ̬״̬�ı��������T/D/H/F/S
	bool SendMdaDataReqToHmi();			//��HMI����MDA��������
	bool SendMessageToHmi(uint16_t msg_type, uint32_t msg_id, uint32_t msg_param = 0);		//��HMI������ʾ��Ϣ
    bool SendWorkCountToHmi(uint32_t count, uint32_t totalCount);   //��HMI���¼ӹ�����
	bool SendMachOverToHmi();    //֪ͨHMI�ӹ����

	bool SendManualToolMeasureResToHmi(bool res);  //�����ֶ��Ե������HMI

	bool ClearHmiInfoMsg();    //��յ�ǰ��ʾ��Ϣ


	//����MC״̬�ӿ�
	void SendIntpModeCmdToMc(uint16_t intp_mode);      //��MCģ�鷢�Ͳ岹����ģʽ����
	void SendWorkModeToMc(uint16_t work_mode);    //��MCģ�鷢�ͼӹ�ģʽ�����Ҫ�������ֲ岹Ŀ��λ�ã�������㲻ͬģʽ�����ƶ���
    void SendMachineStateToMc(uint8_t state); //��MCģ�鷢��SC��ǰ�ӹ�״̬
    void PauseMc();		//��ͣMCģ�������
	void SetMcAxisOrigin(uint8_t axis_index);		//��MC������Ĺ�������ϵԭ��
	void SetMcAxisOrigin(uint8_t axis_index, int64_t origin_pos); //��MC������Ĺ�������ϵԭ��
	void ActiveMcOrigin(bool active_flag);		//�������õĹ�������ϵ
	void SetMcAxisToolOffset(uint8_t axis_index); 	//��MC���ø��ᵶ��ƫ��
	void ActiveMcToolOffset(bool active_flag);		//�������õĵ���ƫ��
	void SendMcG31Stop();    //��MC����G31ֹͣ����
    //void SendMcRigidTapFlag(bool tap_flag);    //��MC���͸��Թ�˿����

	void SendMcResetCmd();		//����MC��λָ��

	uint16_t ReadMcMoveDataCount();   //��ȡ��ǰMC���˶����ݵ�����
	bool CheckBlockOverFlag();	//��ǰ�ֿ鵽λ��־�Ƿ���Ч
	bool CheckStepOverFlag();	//��ǰ���ε�λ��־�Ƿ���Ч

	bool IsMcNeedStart();   //MC�Ƿ���Ҫ������������

	void StartMcIntepolate();    			//����MC��ʼ�岹

	static void *RefreshStatusThread(void *args); //״̬�����̺߳���
	bool RefreshStatusFun();  	//״̬���º���

	void InitMcModeStatus();		//��ʼ�����ݸ�MC��ģ̬��Ϣ
	bool IsMcModeCmd(int cmd);	//�Ƿ���Ҫ���͵�MC��ģ̬��Ϣ����

	void RefreshModeInfo(const McModeStatus &mode);   //���µ�ǰͨ��ģ̬����
	
	void ReadGraphAxisPos();      //��ȡͼ��ģʽ�µĸ�Ƶ��λ������
	void SendHmiGraphPosData();     //��HMI���ͻ�ͼλ������

	void SetMiSimMode(bool flag);   //����MI����ģʽ


	void SetMcToolOffset(bool flag);      //����MC�ĵ���ƫ��
	void SetMcRatio();		//��MC���±�����
	void SetMcSpecialIntpMode(uint16_t intp_type);   //��������岹ģʽ������������岹G12.1��ĥ��ר��ָ��G12.2���Լ�ȡ��ָ��G13.1
	void SetMcRtcpMode(uint16_t cmd_type, uint16_t switch_type, int32_t h_val);   //��������RTCP״̬
#ifdef USES_GRIND_MACHINE
	void SetMcGrindParamDynamic(uint16_t intp_type, uint16_t data_idx, int data1, int data2);		//����G12.2/G12.3ĥ����̬����
	void EnableGrindShock(bool enable);    //ʹ����ĥ
	void ReturnSafeState();    //ִ�и�λ��ȫλ�ö���

	void ProcessGrindM66(AuxMsg *msg);   //����M66ָ���������������
	void ProcessGrindM66_slave(AuxMsg *msg);   //����ӻ�M66ָ��ӻ�����������
#endif

#ifdef USES_WOOD_MACHINE
	void SetMcToolLife();   //��MC���õ�ǰ��������    ľ���������������������ȼ���

	void PreExecSpindleCmd(uint64_t line);    //ִ������Ԥ��������
	bool ExecuteAuxMsg_wood(RecordMsg *msg);    	//ʵ��ִ�и���ָ����Ϣ, ľ�������ư汾
#endif

    void SetChnCurWorkPiece(int newCnt);   //���õ�ǰ�ӹ�����
	int GetCurToolLife();    //��ȡ��ǰ��������

	uint8_t GetCurPhyAxis();		//��ȡ��ǰ�������

	void SetAxisRefCur(uint8_t axis);	//�趨ָ����ĵ�ǰ������ֵΪ���
	void SetAxisRefEncoder(uint8_t axis, int64_t encoder);		//����ָ��������λ�ö�Ӧ�ı�����ֵ
	void ActiveAxisZCapture(uint8_t axis);		//����ָ�����Z�źŲ�����

	void SendMiClearPosCmd(uint8_t axis, int mode); //������λ��������Ȧָ��

	void SendSpdSpeedToMi(uint8_t axis, int16_t da_value);   //��������ת�ٶ�Ӧ��DAֵ��MI

	bool PmcRapidMove(uint8_t chn_axis, double tar_pos, bool inc);    //PMC�ᶨλ�ƶ�
	bool PmcLineMove(uint8_t total, uint8_t idx, uint8_t chn_axis, double tar_pos, double vel, bool inc);   //PMC�������ƶ�

#ifdef USES_SPEED_TORQUE_CTRL
	void SendAxisCtrlModeSwitchCmdToMi(uint8_t axis, uint8_t value);   //���������ģʽ�л����MI
	
	void SendAxisSpeedCmdToMi(uint8_t axis, int32_t value);   //�����ٶ��������ٶ�ֵ��MI
	
	void SendAxisTorqueCmdToMi(uint8_t axis, int16_t value, int16_t speed_lmt,int16_t flag);   //������������������ֵ��MI

	void ProcessCtrlModeSwitch(AuxMsg *msg, uint8_t index);   //�������ģʽ��̬�л�Mָ��
	bool ResetAllAxisOutZero(void);
	bool ResetAllAxisCtrlMode(uint8_t flag);

	void ProcessSpdModeSwitch(AuxMsg *msg, uint8_t index);  //��������CSģʽ�л�����
#endif

    void SendAxisMapCmdToMi(uint8_t phy_axis,uint8_t chan,uint8_t axis);   //������������ͨ����ӳ���ϵ��������

	void SendMCodeToPmc(uint32_t m_code, uint8_t m_index);  //��M���뷢�͸�PMC
	void SetMFSig(uint8_t index, bool value);      //����MFnѡͨ�ź�
	bool GetMFINSig(uint8_t index);    //����ָ����ŵ�MFIN�ź�
	bool GetMExcSig(uint8_t index);    //����ָ����ŵ�MExc�ź�

	void SendTCodeToPmc(uint32_t t_code, uint8_t index);			//��Tָ�����PMC�Ĵ���
	void SetTFSig(uint8_t index, bool value);		//����TFѡͨ�ź�
	bool GetTFINSig(uint8_t index);     //����ָ����ŵ�TFIN�ź�

	void SpindleSpeedDaOut(int32_t speed);		//������ת��ת��Ϊ��ƽֵ


	void RefreshAxisIntpPos();   //��MC�ĸ���岹λ��ˢ�µ�ͨ��ʵʱ״̬��

	bool CancelBreakContinueThread();	//�˳��ϵ�����߳�
	
	bool IsBufClearMsg(RecordMsg *msg);   //�ж��Ƿ񻺳������Ϣ
	bool IsRunmoveMsg(RecordMsg *msg);    //�Ƿ����ƶ���Ϣ
	ListNode<RecordMsg *> *FindMsgNode(uint16_t index);   //���Զ����ݻ����в���ָ��֡�ŵ�����

	void ProcessAxisMapSwitch(AuxMsg *msg, uint8_t index);   //������ӳ�䶯̬�л�Mָ��

	void GetHmiToolOffset(const uint8_t idx, HmiToolOffsetConfig &cfg);     //��ȡָ����ƫ������
	bool NotifyHmiToolOffsetChanged(uint8_t h_idx);     //֪ͨHMI����ƫ�ò���ֵ�������
	bool NotifyHmiWorkcoordChanged(uint8_t coord_idx);   //֪ͨHMI��������ϵ���÷������
	bool NotifyHmiWorkcoordExChanged(uint8_t coor_idx);  //֪ͨHMI ��չ����ϵ���÷������
    bool NotifyHmiToolPotChanged();    //֪ͨHMI������Ϣ�����ı�
    bool NotifyHmiMCode(int mcode);

	void DoRestart(uint64_t line_no);     //�ӹ���λִ�к���
	void InitRestartMode();         //��ʼ���ӹ���λ�м�ģ̬

	void ExecMCode(AuxMsg *msg, uint8_t index);     //����ִ��Mָ��ϵͳ����

//	void SendMiTapAxisCmd(uint16_t spd, uint16_t zAxis);   //���͹�˿��Ÿ�MI
//	void SendMiTapParamCmd();      //���͹�˿������MI
//	void SendMiTapRatioCmd(int32_t ratio);   //���͹�˿������MI
//	void SendMiTapStateCmd(bool state);   //���͹�˿״̬��MI

    //void ProcessM29Reset();     //����M29״̬��λ����

	void MiDebugFunc(int mcode);      //����MI����ָ��



#ifdef USES_TWINING_FUNC
	void ActiveTwiningFunc(int active);   //���ز��ƹ���
#endif


private://˽�г�Ա����
	uint8_t m_n_channel_index;   //ͨ��������

	Compiler *m_p_compiler;     //nc���������
	ChannelStatusCollect m_channel_status;   //ͨ��״̬
	ChannelRealtimeStatus m_channel_rt_status;  //ͨ��ʵʱ״̬

	ChannelMcStatus m_channel_mc_status;   //ͨ��MC״̬

	ChannelEngine *m_p_channel_engine;   //ͨ������ָ��
	HMICommunication *m_p_hmi_comm;    //HMIͨѶ�ӿ�
	MICommunication *m_p_mi_comm;		//MIͨѶ�ӿ�
	MCCommunication *m_p_mc_comm;		//MCͨѶ�ӿ�
    MCArmCommunication *m_p_mc_arm_comm;   //MC-ARMͨѶ�ӿ�
    SpindleControl *m_p_spindle;        //����ģ��
	SCSystemConfig *m_p_general_config;   //ϵͳ����
	SCChannelConfig *m_p_channel_config;  //ͨ������
	SCAxisConfig *m_p_axis_config;        //������
    SCCoordConfig *m_p_chn_coord_config;  //��������ϵ����
	SCCoordConfig *m_p_chn_ex_coord_config; //��չ��������ϵ����
	SCCoordConfig *m_p_chn_g92_offset;    //@add zk G92 ȫ������ϵƫ��

	SCToolOffsetConfig *m_p_chn_tool_config;		//����ƫ������
	SCToolPotConfig *m_p_chn_tool_info;     //������Ϣ�������������͡����׺š�������

	ChnProcParamGroup *m_p_chn_proc_param;    //�������ͨ������
	AxisProcParamGroup *m_p_axis_proc_param;   //������������

#ifdef USES_FIVE_AXIS_FUNC
	FiveAxisConfig *m_p_chn_5axis_config;    //��������
	uint8_t m_n_5axis_rot1;      //��һ��ת��ͨ���������ţ�0��ʼ
	uint8_t m_n_5axis_rot2;      //�ڶ���ת��ͨ���������ţ�0��ʼ
	uint8_t m_mask_5axis_rot_nolimit;   //����������ת��ͨ�����mask
	uint64_t m_mask_5axis_rot_nolimit_phyaxis;  //����������ת��ͨ�����������mask,
	bool m_b_5axis_clear_rot;    //������ת��������Ȧ��־������Ҫ��ǰ��ʼ����ʹ��ʱ����ǰ��ʼ��   true--��λ��   false--����λ��
#endif

#ifdef USES_GRIND_MACHINE
	GrindConfig *m_p_grind_config;       //ĥ������
	bool m_b_ret_safe_state;         //����ص���ȫ״̬
	uint8_t m_n_ret_safe_step;     //���Ḵλ����
	double m_df_c_pos_tar;        //C�����Ŀ��λ��
	struct timeval m_time_ret_safe_delay;   //���ڻذ�ȫλ����ʱִ��
	struct timeval m_time_mcode_start;   //Mָ��ִ����ʼʱ�䣬����Mָ��ִ�г�ʱ�澯

	StateMechArm *m_p_mech_arm_state;   //�����ϻ�е��״̬
	ParamMechArm *m_p_mech_arm_param;   //�����ϻ�е�ֲ���
#endif

	uint8_t m_n_spindle_count;				//�������
	uint8_t m_spd_axis_phy[kMaxChnSpdCount];	//�����Ӧ���������,��1��ʼ

	uint16_t m_n_mc_auto_buf_max;     //MC���Զ�ģʽ�˶����ݻ����С

    uint64_t m_n_real_phy_axis;          //ͨ����ʵ��������Mask

	uint32_t m_mask_intp_axis;      //ͨ���岹��mask����ͨ����˳��
	uint8_t m_n_intp_axis_count;    //ͨ���岹�����

	ErrorType m_error_code;      	//������

	uint8_t m_n_cur_proc_group;     //��ǰ���ղ������

//	char m_str_cur_nc_file[kMaxPathLen];  //��ǰNC�ӹ��ļ�,���·�������Ŀ¼"/cnc/nc_files/"

	pthread_t m_thread_refresh_status;    //״̬�����߳�
	pthread_t m_thread_compiler;   			//G������������߳�
	pthread_t m_thread_breakcontinue;		//�ϵ�����߳�

	bool m_b_init_compiler_pos;    	//�Ƿ��ʼ����������ǰλ�ã���������ϵ

	bool m_b_lineno_from_mc;    //�Ƿ��MCģ����µ�ǰ���д����к�,true��ʾ��MC��ȡ��false��ʾ����MC��ȡ

	CompilerState m_n_run_thread_state;   //G������������߳�����״̬
//	CompilerState m_n_run_state_auto_bak;	//�����Զ�ģʽ��G���������߳�״̬����

	OutputMsgList *m_p_output_msg_list;   		//�������MC��ָ����Ϣ���У�����ģʽָ��AUTO��MDA�Ķ���
	OutputMsgList *m_p_output_msg_list_auto;   //AUTOģʽ�������MC��ָ����Ϣ����
	OutputMsgList *m_p_output_msg_list_mda;   //MDAģʽ�������MC��ָ����Ϣ����
//	GCodeFrameBuffer *m_p_output_buffer;    //�˶��������ݣ��˻�����������ֻ������˶�ָ�ֻ�����˺���ָ��ʱ�Ű�ǰһ��ָ��ͳ�ȥ

	ListNode<RecordMsg *> *m_p_last_output_msg;   //������͵��˶�����
	HWTraceState m_n_hw_trace_state;        //���ָ���״̬
	HWTraceState m_n_hw_trace_state_change_to;    //��Ҫ�л������ָ���״̬
	bool m_b_change_hw_trace_state;       //�л�����״̬��־   true--���ڵȴ�MC�˶�ֹͣ��λ

	uint16_t m_n_frame_index;   //�˶�����֡�� 1~65535ѭ����0�������˶�����

//	bool m_b_quit;   //G������������߳��˳���־
	pthread_mutex_t m_mutex_change_state;    //��������״̬�л�������

	SimulateMode m_simulate_mode;  //����ģʽ
	ChnSimulateZone m_simulate_zone;   //������Χ

	uint8_t m_change_work_mode_to;	//�����л���Ŀ�깤��ģʽ���ȴ�ϵͳֹͣG��������

	int16_t m_n_cur_tcode;		//��ǰT���룬M06����,-1��ʾδ��ʼ��
    int32_t m_n_cur_scode;		//��ǰS���룬M03/M04����,-1��ʾδ��ʼ��   ��λ��rpm
//	int16_t m_n_cur_dcode;		//��ǰD���룬G41/G42����,-1��ʾδ��ʼ��
//	int16_t m_n_cur_hcode;		//��ǰH���룬G43/G44����,-1��ʾδ��ʼ��

	char m_str_mda_path[kMaxPathLen];        //ͨ������MDA�ļ�����·��

	bool m_b_mc_need_start;			//MC������Ҫ���������岹���true--��Ҫ   false--����Ҫ

	bool m_b_need_change_to_pause;   //������ͣ��־��true--�ȴ�mc�����꼴�л���ͣ״̬      false--����Ҫ�л�

	uint8_t m_n_step_change_mode_count ;    //���ڵ���ģʽ״̬�л�ȷ�ϼ���������mc����Ӧ��ʱһ��Ҫ����2�δﵽ�������л�״̬
//	bool m_b_step_exec;    //����ִ�б�־

	McModeStatus m_mc_mode_exec;   //ָ����Ϣִ��ʱ�޸ĵ�MCģ̬����Ϣ
	McModeStatus m_mc_mode_cur;	   //���һ�δ�MC��ȡ��MCģ̬��Ϣ

	ChannelAutoScene m_scene_auto;		//�Զ�ģʽ������״̬
	int m_n_breakcontinue_segment;    //�ϵ������ִ�н׶�
	bool m_b_cancel_breakcontinue_thread;   //�˳��ϵ�����̱߳�־

	uint64_t m_n_send_mc_data_err;	//��MC��������ʧ�ܼ���

	uint8_t m_n_subprog_count;     	//�ӳ���Ƕ�ײ�������,������������
	uint8_t m_n_macroprog_count;    //�����Ƕ�ײ����������������ӳ���
	bool m_b_ret_from_macroprog;    //����򷵻ر�־

	//���ڱ���ѭ��ָ�����ֵ
	double m_df_loop_param[kMaxTagCount];  //���������ֶε�ֵ

	bool m_b_manual_tool_measure;   //�Ƿ����ֶ��Ե���
	bool m_b_cancel_manual_tool_measure;   //ȡ���ֶ��Ե�

	bool m_b_manual_call_macro;           //�Ƿ����ֶ���������
	bool m_b_cancel_manual_call_macro;    //ȡ���ֶ���������

#ifdef USES_ADDITIONAL_PROGRAM
	AddProgType m_n_add_prog_type;        //��ǰ���ӳ���ִ��״̬
	uint8_t m_n_sub_count_bak;            //ִ�и��ӳ���ʱ���ӳ���Ƕ�׼������ݣ����ڷ���ʱ�����Ƿ���Ҫ��λ���ӳ���״̬��־
#endif

	struct timeval m_time_start_mc;    //���һ�η���MC���������ʱ�䣬���ڶ�ȡMC���е�λ��־ʱ����ʱ�ж�

	Variable m_macro_variable;          //�����

	PmcRegister *m_p_pmc_reg;			//PMC�Ĵ���
	FRegBits *m_p_f_reg;			    //��ͨ��F�Ĵ���ָ��
	const GRegBits *m_p_g_reg;			//��ͨ��G�Ĵ���ָ��

	struct timeval m_time_m_start[kMaxMCodeInLine];     //Mָ��ִ�еĿ�ʼʱ��
	struct timeval m_time_t_start[kMaxTCodeInLine];     //Tָ��ִ�еĿ�ʼʱ��
	struct timeval m_time_start_maching;     //��ʼ�Զ��ӹ���ʱ�䣬���ڼӹ���ʱ
    int32_t        m_time_remain;            //ʣ��ӹ�ʱ��

	uint64_t m_n_mask_clear_pos;		//λ��������Ȧ���־

	//IO����λ�ò��������
	uint64_t m_n_mask_pos_capture;      //��λ�ò���Mask
	uint64_t m_n_mask_captured;         //��λ�ò���ɹ�Mask
	DPointChn m_point_capture;		    //�������λ�ã���е����ϵ
	bool m_b_pos_captured;             //��λ�ò�����ɱ�־

	//PMC������mask
	uint64_t m_mask_run_pmc;  //��ǰ���е����mask
	uint64_t m_mask_runover_pmc;   //��ǰ������ɵ����mask

	//�ӹ���λ��ر���
    uint8_t m_n_restart_mode;     //�ӹ���λģʽ��0--�Ǽӹ���λ   1--�����ӹ���λ    2--���ټӹ���λ
    uint8_t m_n_restart_step;     //�ӹ���λ����
    uint64_t m_n_restart_line;    //�ӹ���λĿ���к�
    ChnRestartMode m_mode_restart;    //�ӹ���λ�м�ģ̬
	
	bool m_b_mc_on_arm;    //��ͨ������mcͨ���Ƿ�������ARM��
    bool m_b_delay_to_reset;   //�ӳٸ�λ�������ȴ�����ͣ��λ����ִ�и�λ����

    DPointChn m_pos_simulate_cur_work;    //����ģʽ�µĵ�ǰ��������

//	uint8_t m_n_M29_flag;   //  M29 M28 flag;   0--M28  1--M29
//	uint8_t m_n_G843_flag;  //  G843 flag;   0--�Ǹչ�����״̬         1--�չ�����״̬
//	int32_t m_n_G843_ratio; // G843 ratio; ������Թ�˿�ı���ֵ�� ��ֵ*10000����

	bool m_b_hmi_graph;    //HMI�Ƿ���ͼ��ģʽ����ģʽ��Ҫ����ʵʱ��Ƶλ������
	uint8_t m_n_xyz_axis_phy[3];    //XYZ���Ӧ��������ţ�0��ʼ
	CoordAxisPos m_pos_graph_array[kGraphPosBufCount];   //����ͼ��ģʽλ������
	uint16_t m_n_graph_pos_write_idx;     //��ǰ���е�λ�û���д������
	uint16_t m_n_graph_pos_read_idx;      //��ǰλ�����ݶ�ȡ����
    uint16_t m_n_graph_pos_count;   //��ǰλ�û�������
	

#ifdef USES_SPEED_TORQUE_CTRL
    uint8_t m_n_need_reset_axis_ctrlmode;				// ���Ḵλ����ֵ����Ϊ0ʱ����ʶ����Ҫ��λ����ĳ�����ݼ�����Ϊ0����ʶ��λ�ɹ�
#endif

#ifdef USES_WOOD_MACHINE
	//��������ˢ����ʱ��־
	uint8_t m_n_ref_tool_life_delay;    //�����л���ǰ����ʱ����ʱˢ�µ�������
	bool m_b_save_tool_info;         //����ʱ�Ƿ񱣴浶����Ϣ
	
	bool m_b_prestart_spd;    //����Ԥ����ִ�б�־    true--��ǰ����ִ��
	int m_n_spd_prestart_step;   //��ǰ����Ԥ����ִ�в��裬 ��0��ʼ��������Ϊ0xFF
	SpindleStartOffset m_spd_prestart_offset;   //��ǰ��ִ�е�����Ԥ��������
#endif


#ifdef USES_LASER_MACHINE
	bool m_b_laser_calibration;			//���м���������궨
	bool m_b_cancel_calib;				//�˳��궨����
	uint8_t m_n_calib_step;				//�궨����
	uint8_t m_n_calib_ref_count;		//�궨����
	uint8_t m_n_cur_ref_point;		    //��ǰ�궨λ�õ�����
	uint8_t m_n_laser_input_signal;          //�����������ź�
	uint8_t m_n_follow_range;           //����������߶ȷ�Χ      ��λ:mm
	struct timeval m_time_delay_start;   //�ȴ���ʼʱ��
#endif

#ifdef USES_WUXI_BLOOD_CHECK
	bool m_b_returnning_home;      //�زο�����
	uint8_t m_n_ret_home_step;     //���㲽��
	struct timeval m_time_start_wait;   //�ȴ���ʼʱ��
	uint32_t m_n_warn_status;       //�澯״̬
#endif

#ifdef USES_SIMULATION_TEST
	int m_file_sim_data;      //�������ݱ����ļ����
#endif

	double G52offset[kMaxAxisChn];
	bool G52Active = false;

};

#endif /* INC_CHANNEL_CHANNEL_CONTROL_H_ */
