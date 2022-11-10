/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file channel_engine.h
 *@author gonghao
 *@date 2020/03/19
 *@brief ��ͷ�ļ�Ϊͨ�������������
 *@version
 */

#ifndef INC_CHANNEL_CHANNEL_ENGINE_H_
#define INC_CHANNEL_CHANNEL_ENGINE_H_


#include "global_definition.h"
#include "global_structure.h"
#include "hmi_shared_data.h"
#include "comm_data_definition.h"
#include "pmc_axis_ctrl.h"
#include "channel_data.h"
#include "parm_manager.h"

#include "license_interface.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <signal.h>

#include "pmc_register.h"


//ǰ������
class ChannelControl;   //ͨ��������
class HMICommunication; //HMIͨѶ��
class MICommunication;	//MIͨѶ��
class MCCommunication;	//MCͨѶ��
class MCArmCommunication;  //MC-ARMͨѶ��
class ParmManager;      //���ù�����
class PmcRegister;		//PMC�Ĵ�����
class ChannelModeGroup;   //ͨ����ʽ����
class SyncAxisCtrl;     //ͬ���������
struct SCSystemConfig; //SCͨ������
struct SCChannelConfig;  //SCͨ������
struct SCAxisConfig;	//SC������
struct ParamUpdate;		//���������ṹ



/**
 * @brief ͨ�������࣬����ͳ�ﴦ��ͨ����Ľ����Լ��ⲿ����
 */
class ChannelEngine {
public:

	static ChannelEngine *GetInstance();   //����ģʽ����ȡ����ʵ����Ψһ���ʵ�
	virtual ~ChannelEngine();  //��������

	void Initialize(HMICommunication *hmi_comm, MICommunication *mi_comm,
			MCCommunication *mc_comm, ParmManager *parm);   //���ýӿ�

void SetMcArmComm(MCArmCommunication *comm){this->m_p_mc_arm_comm = comm;}   //����MC-ARMͨѶ�ӿ�
	bool IsMcArmChn(uint8_t chn){return this->m_mc_run_on_arm[chn];}   //�Ƿ�ͨ��MC������ARM��
	void InitBdioDev();   //��ʼ��SD-LINK��վ�豸����

#ifdef USES_PMC_2_0
    void ReadIoDev_pmc2();                               //PMC2.0�汾����ȡSD_LINK��վ�豸����
    bool CheckIoDev_pmc2(const BdioDevInfo &info);       //���SD_LINK�����Ƿ����
    bool SelectHandleWheel(int indexId, int channelId);  //��������ͨ��ӳ��
#else
	void ReadIoDev_pmc1();     //PMC1.0�汾����ȡSD_LINK��վ�豸����
#endif

	void SendMonitorData(bool bAxis, bool btime);   //���ͼ������
	void SendPmcAxisToHmi();    //����PMC��λ�ø�HMI

	void ProcessHmiCmd(HMICmdFrame &cmd);  //����HMIָ��
	void ProcessMcCmdRsp(McCmdFrame &rsp);	//����MCģ���ָ��ظ�
	void ProcessMiCmd(MiCmdFrame &cmd);		//����MIģ���ָ��


	int GetChnCount(){return this->m_p_general_config->chn_count;}     //���ص�ǰͨ������
	ChannelControl *GetChnControl(){return this->m_p_channel_control;}   //��ȡ����ͨ�����ƶ�������ĵ�ָ��
	ChannelControl *GetChnControl(uint8_t index);		//��ȡָ����ͨ�����ƶ���ָ��
	bool GetChnStatus(uint8_t chn_index, HmiChannelStatus &status);  //��ȡͨ��״̬
	uint8_t GetChnAxistoPhyAixs(uint8_t chn_index, uint8_t chn_axis);  //��ȡͨ�����Ӧ���������

//	uint16_t GetMcAutoBufMax(){return m_n_mc_auto_buf_max;}   //��ȡMC��ͨ���Զ����ݻ�������


	bool Start();   //��������������Ӧѭ������
	bool Pause();   //��ͣ����������Ӧ��������
	bool Stop(bool reset);	 //ֹͣ����ִ�У����ڴ���ϵͳ�澯�µĳ���ֹͣ
	bool Stop(uint8_t chn, bool reset);  //ֹͣ����ִ�У����ڴ���ϵͳ�澯�µĳ���ֹͣ
	bool SetCurWorkChanl(uint8_t work_chan);   //���õ�ǰͨ����
    void ServoOn();     //�Գ����������������ʹ��
    void ServoOff();	//�Գ����������������ʹ��

#ifdef USES_EMERGENCY_DEC_STOP
    void DelayToServoOff(uint8_t chn_index);  //�ӳ����ŷ�

    void SetChnStoppedMask(uint8_t chn_index);   //����ͨ��ֹͣ��λ��־
#endif

	bool SetWorkMode(uint8_t work_mode);   //���ù���ģʽ

	void ShakeHandWithMc();    	//��MCģ�鷢����������
	void ShakeHandWithMi();		//��MIģ�鷢����������

	void SetFuncState(uint8_t chn, int state, uint8_t mode = 10);		//���ù���״̬�����磺���Σ�ѡͣ�ȵ�
	void SetAutoRatio(uint8_t ratio);    //�����Զ�����
	void SetAutoRatio(uint8_t chn, uint8_t ratio);  //�����Զ�����
	void SetManualRatio(uint8_t ratio);	//�����ֶ�����
	void SetManualRatio(uint8_t chn, uint8_t ratio);	//�����ֶ�����
	void SetRapidRatio(uint8_t ratio);	//���ÿ��ٽ�������
	void SetRapidRatio(uint8_t chn, uint8_t ratio);	//���ÿ��ٽ�������
	void SetManualStep(uint16_t step);	//�����ֶ�����
	void SetManualStep(uint8_t chn, uint8_t step);  //�����ֶ�����
	void SetManualRapidMove(uint8_t mode = 10);			//�����ֶ������ƶ�״̬
	void SetCurAxis(uint8_t axis);		//���õ�ǰ��
	void SetCurAxis(uint8_t chn, uint8_t axis);  //���õ�ǰ��
	void ChangeChnGroupIndex(uint8_t chn, uint8_t group_old, uint8_t group_new);   //�ı�ָ��ͨ����������ʽ��

	void SetChnMachineState(uint8_t chn, uint8_t mach_state);   //����ͨ���ӹ�״̬

	void EnableHWTraceToMi();     //��MI�������ַ������ʹ��

//	void SetAxisOn(uint8_t index);			//������ʹ�ܣ��Լ��岹ģʽ��1--NC��岹   2--PMC��岹��
//	void SetAxisBaseParam(uint8_t index);		//����ָ������ݾ༰����ٶȵȻ�����Ϣ
//	void SetAxisSpeedParam(uint8_t index);		//����ָ������ٶ������Ϣ
//	void SetAxisAccParam(uint8_t index);			//����ָ������ٶ������Ϣ


	void ManualMove(int8_t dir);		//�ֶ��ƶ�
	void ManualMoveStop();			//�ֶ�ֹͣ
	void ManualMove(uint8_t phy_axis, int8_t dir, double vel, double inc_dis);  //�ֶ��ƶ�����dir�����ƶ�dis����
	void ManualMoveAbs(uint8_t phy_axis, double vel, double pos);   //�ֶ���Ŀ���ٶ�vel�ƶ�������Ŀ��λ��

	void ManualMoveStop(uint8_t phy_axis);			//�ֶ�ֹͣ

	void ManualMovePmc(int8_t dir);		//�ֶ��ƶ�
	void ManualMovePmc(uint8_t phy_axis, int8_t dir);   //ָ�����ֶ��ƶ�
	void ManualMovePmc(uint8_t phy_axis, double tar_pos, double vel, bool inc);  //ָ������ָ���ٶ��ƶ���ָ��λ��
	void ManualMovePmcStop();			//�ֶ�ֹͣ


	void PausePmcAxis(uint8_t phy_axis, bool flag);          //��ͣPMC���ƶ�

	double GetPhyAxisMachPosFeedback(uint8_t index){return m_df_phy_axis_pos_feedback[index];}  //��ȡָ��������ĵ�ǰ������е����

#ifdef USES_SPEED_TORQUE_CTRL
	double GetPhyAxisMachSpeedFeedback(uint8_t index){return m_df_phy_axis_speed_feedback[index];}  //��ȡָ��������ĵ�ǰ�����ٶ�
	double GetPhyAxisMachTorqueFeedback(uint8_t index){return m_df_phy_axis_torque_feedback[index];}  //��ȡָ��������ĵ�ǰ��������
#endif
	
	double GetPhyAxisMachPosIntp(uint8_t index){return m_df_phy_axis_pos_intp[index];}   //����ָ��������ĵ�ǰ�岹��е����
	void StartUpdateProcess();		//��ʼģ����������

	void ClearAlarm();				//����澯
	void SystemReset();				//ϵͳ��λ
	void Emergency(uint8_t chn = CHANNEL_ENGINE_INDEX);					//��ͣ����


	bool GetMacroVarValue(uint8_t chn, uint32_t index, bool &init, double &value);   	//��ȡ�������ֵ
	bool SetMacroVarValue(uint8_t chn, uint32_t index, bool init, double value);		//���ú������ֵ

	bool GetPmcRegValue_8(uint16_t reg_sec, uint16_t reg_index, uint8_t &value);			//��ȡPMC�Ĵ�����ֵ
	bool GetPmcRegValue_16(uint16_t reg_sec, uint16_t reg_index, uint16_t &value);			//��ȡPMC�Ĵ�����ֵ
	bool SetPmcRegValue_8(uint16_t reg_sec, uint16_t reg_index, uint8_t value);           //����PMC�Ĵ�����ֵ
	bool SetPmcRegBitValue_8(uint16_t reg_sec, uint16_t reg_index, uint8_t bit_index, bool value);    //��λ���üĴ���ֵ
	bool SetPmcRegValue_16(uint16_t reg_sec, uint16_t reg_index, uint16_t value);           //����PMC�Ĵ�����ֵ


	FRegBits *GetChnFRegBits(uint8_t chn_index);     //��ȡָ��ͨ����F�Ĵ���ָ��
	const GRegBits *GetChnGRegBits(uint8_t chn_index);      //��ȡָ��ͨ����G�Ĵ���ָ��

	static void InitPoweroffHandler();			//��ʼ���������źź͵��紦����


//	void SetAxisSoftLimit(uint8_t axis);   //����ָ���������λ����
//	void SetAxisSoftLimitValue(uint8_t axis, uint8_t index);   //����ָ���������λֵ

	uint32_t GetDaPrecision(){return this->m_n_da_prec;}  //����DA����ֵ

	bool CheckAxisHardLimit(uint8_t phy_axis, int8_t dir);   //�����������λ�澯���

	void SetPmcSignal(uint8_t chn_index, int signal, bool flag);	//����ͨ����PMC�ź�

	void SetPoweoffFlag(bool flag){m_b_power_off = flag;}  //���õ����־

	void RefreshFile(char *file);   //ˢ��nc�ļ�file
	void RemapFile(char *file);		//���¼����ļ�file

	void ProcessHmiIOCmd(HMICmdFrame &cmd);   	//����HMI��IO����

	bool IsEmergency(){return m_b_emergency;}    //�Ƿ��ڼ�ͣ״̬

	void ResetOPSignal(uint8_t chn_index);   //��λOP�ź�
	void SetALSignal(uint8_t chn_index, bool value);  //���ø澯�ź�


    void SetMiWorkMode(uint8_t value);   //����MIģ�鹤��ģʽ
    void SetMiHandwheelTrace(bool flag, uint8_t chn);   //����MIģ�����ָ���ģʽ
    void SetMiCurChannel();   //����MI��ǰͨ����

    bool CheckAxisRefBaseSignal(uint8_t phy_axis, int8_t dir);   //�����ԭ��ֻ�׼�ź�
    bool CheckAxisRefSignal(uint8_t phy_axis);      //�����ԭ�㾫��׼�źţ�IO�źţ�
    
    uint8_t GetAxisChannel(uint8_t phy_axis, uint8_t &chn_axis);     //��ȡ����������ͨ����ͨ������

    void SetRetRefMask(uint8_t phy_axis);    //���ûزο�����mask
    bool IsRefReturnning(uint8_t phy_axis);    //ָ�����Ƿ����ڻ�����

    void SetRetRefFlag(uint8_t phy_axis, bool flag);   //������زο�����ɱ�־
	void SetInRetRefFlag(uint8_t phy_axis, bool flag);  //������������ź�
	
    void SendPmcAxisCmd(PmcCmdFrame &cmd);     //����PMC���˶�ָ��
    uint8_t GetPmcAxis(uint8_t pmc_chn);        //��ȡָ��PMCͨ����PMC�����
    uint32_t GetPmcAxisRapidSpeed(uint8_t axis);   //��ȡpmc����趨��λ�ٶ�
    void SetCurPmcAxis(uint8_t axis);     //���õ�ǰPMC��

    void SetPmcOutSignal(uint8_t index, uint8_t value);    //����PMC��F�Ĵ�����OUT�����λ

    void ProcessHmiFindRefCmd(HMICmdFrame &cmd); 	//����HMI�زο���ָ��
    void ProcessHmiSetRefCmd(HMICmdFrame &cmd);		//����HMI���òο���ָ��
    void ProcessPmcRefRet(uint8_t phy_axis);         //����PMCָ������������

    void ProcessPmcAxisFindRef(uint8_t phy_axis);   //����PMC��ȷ�ϲο���ָ��

    void ClearPmcAxisMoveData();   //���MI�е�PMC���ƶ�����

    int CheckLicense(bool force = false);     //��ȨУ��

    void SetAxisNameEx(bool flag);   //������������չ�±�ʹ��

    void SendMiPcData(uint8_t axis);    //��MI�����ݲ����ݱ�
    void SendMiPcParam(uint8_t axis);   //��MI����ָ�����ݲ���������
    void SendMiPcParam2(uint8_t axis);  //��MI����ָ�����ݲ���������2
    void SendMiRefMachPos(uint8_t axis_index);   //��MI����ָ����Ĳο���λ�û�е����
    void SendMiBacklash(uint8_t axis);  //��MI����ָ����ķ����϶����

    void SendMiIoRemapInfo(IoRemapInfo &info);   //��MI��������IO�ض��������

    void SendMiAxisFilterParam(uint8_t phy_axis);   //��MI����ָ������˲�����ز���


#ifdef USES_TIANJIN_PROJ
    double TransMachToWorkCoord(double &pos, uint8_t pmc_axis_index);   //PMC���е����ϵ����������ϵ��ת��
    double TransWorkToMachCoord(double &pos, uint8_t pmc_axis_index);   //PMC�Ṥ������ϵ����е����ϵ��ת��
#endif

#ifdef USES_LASER_MACHINE
    void StartLaserCalib(bool start);				//��ʼ����������궨
    void EnableLaserHeightTrace();       //�����ͷ�߶ȸ��湦��
#endif

    void DoIdle();     //���д�����

    void SyncKeepVar();   //ͬ�������ļ�

    void SetParamImportMask(int param_type);    //���ò��������־
    bool IsParamImported(int param_type){return (this->m_mask_import_param & (0x01<<param_type)) == 0?false:true;}       //�����Ƿ����

    bool NotifyHmiAxisRefBaseDiffChanged(uint8_t axis, double diff);     //֪ͨHMI��ο���־���׼λ��ƫ������

    bool NotifyHmiAxisRefChanged(uint8_t phy_axis);     //֪ͨHMI��ο����Ӧ�ı�����ֵ���

    bool NotifyHmiPitchCompDataChanged();        //֪ͨHMI�ݲ����ݸ���

    bool ProcessPcDataImport();      //�����ݲ���������

#ifdef USES_WOOD_MACHINE
    void SaveToolInfo();    //���浶����Ϣ����
#endif

#ifdef USES_GRIND_MACHINE
	void RefreshMechArmParam();    //ˢ�»�е�ֲ���
	void RefreshMechArmSpeedDelay();  //ˢ�»�е���ٶȺ��ӳ�ʱ��
	void InitMechArmState();       //��ʼ����е��״̬
	void StopMechArmRun(bool flag);      //ֹͣ��е�ֶ�������
	void PauseMechArmRun(bool flag);     //��ͣ��е�ֶ���
	void ProcessNewTray(uint8_t chn);      //���������̶���
	void SendHMIMechArmException(uint8_t err_code);   //��HMI���ͻ�е���쳣����
	void ProcessHmiMechArmExcepRsp(HMICmdFrame &cmd);   //����HMI���صĻ�е���쳣���ݰ�

	StateMechArm *GetMechArmState(){return &this->m_mech_arm_state;}   //��ȡ��е��״̬
	ParamMechArm *GetMechArmParam(){return &this->m_mech_arm_param;}   //��ȡ��е�ֲ���
#endif

    void PrintDebugInfo();		//�����������

    PmcAxisCtrl *GetPmcAxisCtrl(){return m_pmc_axis_ctrl;}
    SyncAxisCtrl *GetSyncAxisCtrl(){return  m_sync_axis_ctrl;}

private:	//˽�г�Ա����
	ChannelEngine();   //���캯��

	static void *RefreshMiStatusThread(void *args); //MI״̬�����̺߳���
	bool RefreshMiStatusFun();  //����MI��״̬

	void SendIntepolateCycle();			//��MC���Ͳ岹���ڲ���
	void InitMcDataBuffer();				//��ʼ��MC�����ݻ�����
	void InitMcCoord();					//��ʼ������MC��������ϵ
	void SendGetMcVersionCmd();			//��ȡMCģ��İ汾��Ϣ
	void InitMcParam();					//����MC����ز���

	void InitChnModeGroup();                //��ʼ����ʽ������

	void InitPcAllocList();           //��ʼ���ݲ����ݷֲ���

#ifdef USES_GRIND_MACHINE
	void SendMcGrindParam();				//����ĥ������
	void RunMechArm();                //ִ�л�е�������϶���

	void ManualMoveMechArm();      //��е���ֶ���λ
#endif

	void ProcessMcVersionCmd(McCmdFrame &cmd);	//����MCģ�鷵�صİ汾��Ϣ
	void ProcessMiVersionCmd(MiCmdFrame &cmd);	//����MIģ�鷵�صİ汾��Ϣ


	static void PoweroffHandler(int signo, siginfo_t *info, void *context); 	//���紦����

	void SaveDataPoweroff();			//����ʱ��������

	void ProcessMiShakehand(MiCmdFrame &cmd); 	//����MI����ָ��
	void ProcessMiPmcLadderReq(MiCmdFrame &cmd);   //����PMC����ͼ��������
	void ProcessMiGetESBCmd(MiCmdFrame &cmd);    //����MI��ȡ�ŷ������ļ�����
	void ProcessMiAlarm(MiCmdFrame &cmd);			//����MI�澯��Ϣ
	int32_t LoadPmcLadderData(uint16_t index, uint16_t &flag);		//����PMC����ͼ��������
	int32_t LoadEsbData(uint16_t index, uint16_t &flag);    //����ESB�ļ����ݣ��ŷ������ļ�
	void ProcessMiSetRefCurRsp(MiCmdFrame &cmd);		//����MI���صı�����ֵ
	void ProcessMiBusError(MiCmdFrame &cmd);			//����MI���ص����ߴ���

	void ProcessMiClearPosRsp(MiCmdFrame &cmd);		//����MI���ص�������Ȧλ��ָ��ظ�

	void ProcessMiSyncAxis(MiCmdFrame &cmd);		//����MI���ص�ͬ����ͬ�����
	void ProcessMiHWTraceStateChanged(MiCmdFrame &cmd);   //����MI���͵����ָ���״̬�л�ָ��

	void ProcessHmiUpdateReq(HMICmdFrame &cmd);		//����HMI��������
	void ProcessHmiGetParam(HMICmdFrame &cmd);   //����HMI��ȡ����ָ��
	void ProcessHmiSetParam(HMICmdFrame &cmd);	//����HMI���ò���ָ��

	bool UpdateHmiPitchCompData(HMICmdFrame &cmd);   //����HMI�����ݲ�����

	void ProcessHmiGetPmcReg(HMICmdFrame &cmd);	//����HMI��ȡPMC�Ĵ���ֵ��ָ��
	void ProcessHmiSetPmcReg(HMICmdFrame &cmd);	//����HMI����PMC�Ĵ���ֵ��ָ��
	void ProcessHmiGetPmcUuid(HMICmdFrame &cmd); //����HMI����PMC��UUIDָ��
	void ProcessHmiAxisMoveCmd(HMICmdFrame &cmd);        //����HMI���ƶ�ָ��

	void ProcessHmiGetLicInfoCmd(HMICmdFrame &cmd);     //����HMI��ȡ��Ȩ��Ϣָ��
	void ProcessHmiRegLicCmd(HMICmdFrame &cmd);          //����HMIע����Ȩָ��

	void ProcessHmiGetIoRemapInfoCmd(HMICmdFrame &cmd);  //����HMI��ȡIO��ӳ����Ϣ����
	void ProcessHmiSetIoRemapInfoCmd(HMICmdFrame &cmd);  //����HMI����IO��ӳ����Ϣ����

	void ProcessHmiSetProcParamCmd(HMICmdFrame &cmd);    //����HMI���ù�����ز���������
	void ProcessHmiGetProcParamCmd(HMICmdFrame &cmd);    //����HMI��ȡ������ز���������
	void ProcessHmiSetCurProcIndex(HMICmdFrame &cmd);    //����HMI����ͨ����ǰ������ŵ�����
	void ProcessHmiGetCurProcIndex(HMICmdFrame &cmd);    //����HMI��ȡ���ղ�����ŵ�����

	void ProcessHmiClearMsgCmd(HMICmdFrame &cmd);     //����HMI�����Ϣ����
    void ProcessHmiGetErrorCmd(HMICmdFrame &cmd);       //����HMI��ȡSC��������
	
	void ProcessHmiNotifyGraphCmd(HMICmdFrame &cmd);    //����HMI֪ͨͼ��ģʽ����
    void ProcessHmiHandWheelCmd(HMICmdFrame &cmd);

	void SendHmiUpdateStatus(uint8_t total_step, uint8_t cur_step);  //��HMI��������״̬



	void ProcessPmcSignal();		//����PMC��G����
	void ProcessPmcAxisCtrl();		//����PMC������ź�
	void ProcessPmcDataWnd();       //����PMC���ݴ���



	void ProcessPmcAlarm();    //����PMC�澯

	void ProcessAxisHardLimit(uint8_t dir);   //������Ӳ��λ�澯

	void GetParamValueFromCmd(ParamUpdate *data, char *src);  //������ػ�ȡ��������

    static void *UpdateThread(void *args);  //ģ������ִ���̺߳���
    int UpdateProcess();		//ģ������ִ�к���
    int UpdateMC();				//����MCģ��
    int UpdateMI();				//����MIģ��
    int UpdateMI_2();				//����MIģ�飬��boot�ֿ�����
    int UpdateSC();				//����SCģ��
    int UpdatePL();				//����PLģ��
    int UpdatePMC();				//����PMCģ��
    int UpdateSpartan();			//����Spartanģ��
    int UpdateModbus();            //����Modbusģ��
	int UpdateDisk();			//һ������

    void SendMcResetCmd();		//����MC��λָ��
    void ClearMcAlarm();			//����MC����澯ָ��
    bool SendMcUpdateStartCmd();		//����MC������ʼָ��
    bool SendMcUpdateFileSize(uint32_t size);		//����MC�����ļ���С
    bool SendMcUpdateFileCrc(uint16_t frame_count, uint16_t crc);		//����MC�����ļ�CRC
    bool QueryMcUpdateStatus();	//��ѯMC����״̬

    bool SendMessageToHmi(uint16_t msg_type, uint32_t msg_id, uint32_t msg_param = 0);		//��HMI������ʾ��Ϣ
    uint16_t GetBusAxisCount();        //��ȡʵ������������
    void InitMiParam();     			//��ʼ��MI����
    void InitPmcReg();				//��ʼ��PMC�ķ���ʧ�ԼĴ���

    void SendPmcRegValue(PmcRegSection sec, uint16_t index, uint16_t data);  //��MI����PMC�Ĵ���
    void SendMiReset();		//����MI��λָ��
 //   void SendMiGetVerCmd();		//���ͻ�ȡMI�汾ָ��

//    template<typename T>
//    void SendMiParam(uint8_t axis, uint32_t para_no, T data);  //����MI����

    void CheckBattery();		//��ѹ��ظ澯���

    void InitPhyAxisChn();		//��ʼ����������ͨ����ӳ��
    void SendMiPhyAxisEncoder();     //��MI����������ķ���
    void SetAxisRetRefFlag();    //��MI���͸���زο��������־

    void SaveCurPhyAxisEncoder();  //���籣�浱ǰ����������ı���������
    void SaveKeepMacroVar();		//���籣�����ʧ�Ժ����

    void CheckAxisSrvOn();     //��������ŷ��ź�

    void SetSlaveInfo();		//����SD-LINK��վ����

    void SetMcAutoBufMax(uint16_t count);     //����MC��ͨ���Զ����ݻ�������

    void SendMiAxisZCaptureCmd(uint8_t phy_axis, bool active);  // ��MI���ͼ����Z�ź�����
	void ReturnRefPoint();      //�زο���ִ�к���
	void PulseAxisFindRefWithZeroSignal(uint8_t phy_axis);      // ��������л�׼��زο��㣬���ݴֻ�׼ ����׼�زο���
	void PulseAxisFindRefNoZeroSignal(uint8_t phy_axis);         // ��������޻�׼��زο��㣬�����ݾ���׼�زο���
	void EcatAxisFindRefWithZeroSignal(uint8_t phy_axis);     // ethcat���л�׼��زο��㣬���ݴֻ�׼ ����׼�زο���
	void EcatAxisFindRefNoZeroSignal(uint8_t phy_axis);        // ethcat���޻�׼��زο��㣬�����ݾ���׼�زο���
	void EcatAxisFindRefWithZeroSignal2(uint8_t phy_axis);     //ethcat���л�׼�زο��㣬�ֻ�׼�뾫��׼��ΪIO�źţ�ֱ�ߵ��
	void AxisFindRefWithZeroSignal(uint8_t phy_axis);            // ������ԭ���ź����òο��� ���������������ԭ���ź����òο���
	void AxisFindRefNoZeroSignal(uint8_t phy_axis);              // ��ǰλ������Ϊ�ο���   ������زο���  ����������òο���
    void EcatIncAxisFindRefWithZeroSignal(uint8_t phy_axis);     // ��������ʽ���л�׼�زο��㣬����Z�ź���Ϊ����׼
    void EcatIncAxisFindRefNoZeroSignal(uint8_t phy_axis);       // ��������ʽ�޻�׼�زο��㣬������Z�ź���Ϊ����׼����


	void PmcAxisRunOver(MiCmdFrame &cmd);    //PMC�����е�λ

	void ProcessGetCurAxisEncodeRsp(MiCmdFrame &cmd);    //����MI���صĵ�ǰ��������Ȧ����ֵ���زο���ʱʹ��
	void ProcessSetAxisRefRsp(MiCmdFrame &cmd);      //����MI���ص�������ο���ظ�ָ��زο���ʱʹ��
	void ProcessGetAxisZeroEncoderRsp(MiCmdFrame &cmd);    //����MI���صĻ�ȡ��е��������ֵָ��زο���ʱʹ��
	void ProcessSkipCmdRsp(MiCmdFrame &cmd);    //����MI���ص���ת������Ӧ
	void ProcessRefreshAxisZeroEncoder(MiCmdFrame &cmd);   //����MIˢ������������ֵ����
	void ProcessMiEnSyncAxisRsp(MiCmdFrame &cmd);      //����MIʹ��ͬ����ָ�����Ӧ
    void ProcessMiSpdLocateRsp(MiCmdFrame &cmd);    //����MI���ᶨλ����Ӧ
    void ProcessMiOperateCmdRsp(MiCmdFrame &cmd);  //����MI����ָ��
    void ProcessMiAxisCtrlModeRsp(MiCmdFrame &cmd);     //����MI��ģʽ�л���Ӧ
    void ProcessMiSpindleSpeedRsp(MiCmdFrame &cmd);     //����MI DA�������Ӧ
    void ProcessMiActiveAxisZCaptureRsp(MiCmdFrame &cmd);   //����MI����Z�ź�ָ����Ӧ
	
	void ProcessSetAxisCurMachPosRsp(MiCmdFrame &cmd);   //����MI�������ᵱǰλ�û�е�����������Ӧ

	void ProcessHmiGetPcDataCmd(HMICmdFrame &cmd);    //����HMI��ȡ�ݲ�����ָ��

	void CheckTmpDir();    //����tmpĿ¼�Ƿ���ڣ��������򴴽�



private:  //˽�г�Ա����
	static ChannelEngine *m_p_instance;    //Ψһʵ��ָ��
	ChannelModeGroup *m_p_channel_mode_group;  //ͨ����ʽ��
	ChannelControl *m_p_channel_control;    //ͨ������
	HMICommunication *m_p_hmi_comm;        //HMIͨѶ�ӿ�
	MICommunication *m_p_mi_comm;		   //MIͨѶ�ӿ�
	MCCommunication *m_p_mc_comm;		   //MCͨѶ�ӿ�
	MCArmCommunication *m_p_mc_arm_comm;   //MC-ARMͨѶ�ӿ�
	SCSystemConfig *m_p_general_config;    //ϵͳ����
	SCChannelConfig *m_p_channel_config;   //ͨ������
	SCAxisConfig *m_p_axis_config;			//����������
	//�ݲ����ݱ�
	AxisPitchCompTable *m_p_pc_table;   //����洢����̬���䣬�������ݲ��������ݲ�
	IoRemapList *m_p_io_remap;     //IO�ض�������

	//���ղ���
	ChnProcParamGroup *m_p_chn_proc_param;    //�������ͨ������
	AxisProcParamGroup *m_p_axis_proc_param;   //������������

	LitronucLicInfo m_lic_info;       //��Ȩ����
	long m_ln_local_time;            //���ؼ�ʱ  -1����ʾ�ļ���ʧ   -2����ʾ�ļ���    -3����ʾ�Ǳ����ļ�   >0:��ʾ������ʱ
	char m_device_sn[SN_COUNT+1];    //�豸���к�

	PcDataAllocList m_list_pc_alloc;     //�ݲ����ݷֲ���

#ifdef USES_FIVE_AXIS_FUNC
	FiveAxisConfig *m_p_chn_5axis_config;    //��������
#endif

#ifdef USES_GRIND_MACHINE
	GrindConfig *m_p_grind_config;       //ĥ������
	ParamMechArm m_mech_arm_param;       //�����ϻ�е�ֲ���
	StateMechArm m_mech_arm_state;       //��е��״̬
	MechArmManualState m_mech_arm_manual_state;    //��е���ֶ�״̬
	uint8_t m_mech_arm_manual_step;    //��е���ֶ���λ����
	double m_mech_arm_manual_target_x;  //�ֶ�X��Ŀ��λ��
	double m_mech_arm_manual_target_y;  //�ֶ�Y��Ŀ��λ��
#endif

	ErrorType m_error_code;  		//������

	uint8_t m_n_cur_channle_index;   //��ǰͨ��������
	uint8_t m_n_cur_chn_group_index;   //��ǰ��ʽ���

//	uint16_t m_n_mc_auto_buf_max;    //MC�е�ͨ���Զ�ģʽ���ݻ����������

	double *m_df_phy_axis_pos_feedback;		//�����ᵱǰ������е����
	double *m_df_phy_axis_pos_intp;        //�����ᵱǰ�岹��е����
	double *m_df_phy_axis_pos_intp_after;   //�����ᵱǰ�岹��Ӽ�������Ļ�е����,���������ݲ��������϶�Ȳ���ֵ
	double *m_df_pmc_axis_remain;    //PMC�����ƶ���

//#ifdef USES_SPEED_TORQUE_CTRL	
	double *m_df_phy_axis_speed_feedback;        //�����ᵱǰ�����ٶ�
	double *m_df_phy_axis_torque_feedback;        //�����ᵱǰ��������
//#endif	

	int64_t m_n_phy_axis_svo_on;		//������ʹ�ܱ�־

	uint8_t m_n_pmc_axis_count;     //PMC������

	pthread_t m_thread_refresh_mi_status;   //ˢ��MI״̬�����߳�
	pthread_t m_thread_update;		//ģ�������߳�
	uint8_t m_n_update_state;     //����״̬   0--������״̬    1--����SC   2--����MC   3--����PMC����ͼ   4--����MI    5--����PL     6--����FPGA

	uint16_t m_n_mc_update_status;   //MCģ������״̬
	bool m_b_get_mc_update_status;  //���յ�MC����״̬

	bool m_b_recv_mi_heartbeat;     //��־�״��յ�MI������������ʱ���Կ�ʼ���ͣɷ��ͳ�ʼ������
	bool m_b_init_mi_over;			//��־��MI���ͳ�ʼ���������

	PmcRegister *m_p_pmc_reg;			//PMC�Ĵ���
	GRegister m_g_reg_last;             //��һ���ڵ�G�Ĵ���
    FRegister m_f_reg_last;             //��һ���ڵ�F�Ĵ���

	bool m_b_emergency;				//��ͣ״̬��־  true--��ͣ״̬    false--�Ǽ�ͣ״̬
	uint64_t m_hard_limit_postive;	//Ӳ��λ������־������   64bit����64��
	uint64_t m_hard_limit_negative; //Ӳ��λ������־������

	uint32_t m_n_da_prec;			//DA���ȣ�ת�����˲�־��ȣ��������DAΪ16λ����Ϊ32768

	bool m_b_power_off;				//�����־

	uint8_t m_map_phy_axis_chn[kMaxAxisNum];      //����������ͨ��

	bool m_b_reset_rst_signal;    //��λRST�źű�־
	struct timeval m_time_rst_over;   //��λ��������ʱ��

	uint32_t m_n_idle_count;      //�������ڼ���

	//ͬ�������
    SyncAxisCtrl *m_sync_axis_ctrl;

	BdioDevList m_list_bdio_dev;    //SD-LINK�豸����

	//�زο�����ز���
	bool m_b_ret_ref;                 //�زο���״̬
	bool m_b_ret_ref_auto;            //�Զ��زο����־
	uint64_t m_n_mask_ret_ref_over;   //�ѻزο����־
	uint64_t m_n_mask_ret_ref;         			//�زο������MASK
	int m_n_ret_ref_step[kMaxAxisNum];    	//�زο��㵱ǰ����
	uint8_t m_n_ret_ref_auto_cur; 				//�Զ��زο���ʱ�ĵ�ǰ˳���
	struct timeval m_time_ret_ref[kMaxAxisNum];       //�زο���ʱ��ʱ��
	double m_df_ret_ref_tar_pos[kMaxAxisNum];  //�زο����м��ƶ�Ŀ��λ��
//	uint8_t m_n_get_cur_encoder_count;    //��ȡMI��ǰ������������������ʩ�������֤
//	uint64_t m_n_ret_ref_encoder;   //�ֻ�׼������ֵ


	//PMC���˶����
	uint64_t m_n_run_axis_mask;  //��ǰ���е����mask
	uint64_t m_n_runover_axis_mask;   //��ǰ������ɵ����mask
	PmcAxisCtrl m_pmc_axis_ctrl[kMaxPmcAxisCtrlGroup];    //PMC�����
	uint8_t m_n_cur_pmc_axis;       //��ǰPMC��  0xFF��ʾ��ǰû��ѡ��PMC��

	uint16_t m_mask_import_param;    //���������־
	
	bool m_mc_run_on_arm[kMaxChnCount];   //MCͨ���Ƿ�������ARM��            0 -- dsp    1--mi
	bool m_mc_run_dsp_flag;    // ��ͨ����MC������DSP�ı�ʶ   
	bool m_mc_run_mi_flag;     // ��ͨ����MC������MI�ı�ʶ  

#ifdef USES_EMERGENCY_DEC_STOP
	bool m_b_delay_servo_off;      //�ӳٶ��ŷ���־���ȴ�����ֹͣ���ٶ��ŷ���ľ�����ٶ�̫�첻��ֱ�Ӷ��ŷ�
	uint8_t m_mask_delay_svo_off;   //���ӳ����ŷ���ͨ��mask
	uint8_t m_mask_delay_svo_off_over;   //�Ѿ�ֹͣ��λ��ͨ����mask
#endif

    const int HANDWHEEL_BYTES = 3;
    const static map<int, SDLINK_SPEC> m_SDLINK_MAP;

};

#endif /* INC_CHANNEL_CHANNEL_ENGINE_H_ */
