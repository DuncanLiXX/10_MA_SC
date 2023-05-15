/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file mi_communication.h
 *@author gonghao
 *@date 2020/06/15
 *@brief ��ͷ�ļ�ΪSC-MIͨѶ�������
 *@version
 */

#ifndef SRC_COMMUNICATION_MI_COMMUNICATION_H_
#define SRC_COMMUNICATION_MI_COMMUNICATION_H_

#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include "hmi_shared_data.h"
#include "comm_data_definition.h"
#include "list_buffer.h"

class ChannelEngine;   //ͨ������

//����ͨ��FIFO���
#define MI_CMD_FIFO_DEPTH  (32)	//32������֡���

//����֡�ֽ���
#define MI_CMD_FRAME_SIZE (24)

//PMC�˶�ָ��ͨ��FIFO���
#define PMC_CMD_FIFO_DEPTH (32)   //32������֡���

//PMC�˶�ָ��֡�ֽ���
#define PMC_CMD_FRAME_SIZE (24)

//PMC�ļ���������С
#define MI_PMC_DATA_SIZE (131072)	//128KB

//�����ڴ��ַ����
#define SHARED_MEM_BASE		(0x1FE00000)		//�����ڴ��׵�ַ��512M�ڴ�
//#define SHARED_MEM_BASE		(0x0FE00000)		//�����ڴ��׵�ַ��256M�ڴ�

#define SHARE_MEM_MUTEX        (SHARED_MEM_BASE + 0x0)             //���ʻ�����
#define SHARED_MEM_AXIS_POS    (SHARED_MEM_BASE + 0x100)           //ͨ����λ��

//�����ݶλ���ַ
#define SHARED_MEM_CMD_SEND_BASE	(SHARED_MEM_BASE + 0x0)			//��������ͨ������ַ	1K
#define SHARED_MEM_CMD_RECV_BASE   (SHARED_MEM_BASE + 0x400)		//��������ͨ������ַ	1K
#define SHARED_MEM_MI_STATUS_BASE  (SHARED_MEM_BASE + 0x75000)		//MIϵͳ״̬������ַ  	4K
#define SHARED_MEM_AXIS_STATUS_BASE	(SHARED_MEM_BASE + 0x76000)	//ͨ����״̬������ַ  	4K
#define SHARED_MEM_SC_STATUS_BASE   (SHARED_MEM_BASE + 0x78000)    //SCϵͳ״̬������ַ    4K
#define SHARED_MEM_PMC_AXIS_POS_BASE (SHARED_MEM_BASE + 0x7FC00)   //PMC�˶�ָ��ͨ������ַ 1K
#define SHARED_MEM_PMC_FREG_BASE	(SHARED_MEM_BASE + 0x80000)		//PMC�Ĵ�������ַ		16K
#define SHARED_MEM_PMC_GREG_BASE   (SHARED_MEM_BASE + 0x84000)		//PMC�Ĵ�������ַ		16K
#define SHARED_MEM_PMC_LADDER_BASE (SHARED_MEM_BASE + 0x90000)		//PMC����ͼ����ַ		128K

//������λ�üĴ���
#define SHARED_MEM_AXIS_MAC_POS_INTP(n) (SHARED_MEM_AXIS_STATUS_BASE+0x40*(n)) 			//��n��Ĳ岹��е����
#define SHARED_MEM_AXIS_MAC_POS_FB(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x08)   			//��n��ķ�����е����
#define SHARED_MEM_AXIS_SPEED(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x10)   					//��n����ٶ�
#define SHARED_MEM_AXIS_TRK_ERR(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x14)   				//��n��ĸ������
#define SHARED_MEM_AXIS_ENCODER(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x18)                  //��n��ĵ�ǰ����������ֵ
#define SHARED_MEM_PMC_AXIS_REMAIN_DIS(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x20)           //��n������ƶ�����PMC����Ч��
#define SHARED_MEM_AXIS_MAC_POS_INTP_AFTER(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x28)       //��n��Ĳ岹��Ӽ���֮������Ļ�е����
#define SHARED_MEM_AXIS_TORQUE(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x30)   			    //��n���ʵʱ���ط���
#define SHARED_MEM_AXIS_SPD_ANGLE(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x32)   			    //��n���ʵʱ����Ƕȷ���
#define SHARED_MEM_AXIS_READ_OVER (SHARED_MEM_AXIS_STATUS_BASE+0x1000)   					//�����ݵĶ�ȡ��ɱ�־
#define SHARED_MEM_AXIS_WRITE_OVER (SHARED_MEM_AXIS_STATUS_BASE+0x1004)   				    //�����ݵ�д����ɱ�־
#define SHARED_MEM_TAP_ERR(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x1040)                  //��n����˿��ĸ��Թ�˿���
#define SHARED_MEM_TAP_ERR_NOW(n) (SHARED_MEM_AXIS_MAC_POS_INTP(n)+0x1044)                  //��n����˿��ĸ��Թ�˿���
#define SHARED_MEM_TAP_READ_OVER (SHARED_MEM_AXIS_STATUS_BASE+0x1240)                  //��n����˿��ĸ��Թ�˿���
#define SHARED_MEM_TAP_WRITE_OVER (SHARED_MEM_AXIS_STATUS_BASE+0x1244)                  //��n����˿��ĸ��Թ�˿���
//ͨ��״̬�Ĵ���


//ϵͳ״̬�Ĵ���
#define SHARED_MEM_MI_STATUS_WARN      (SHARED_MEM_MI_STATUS_BASE)				//�澯�����־��uint8_t����
#define SHARED_MEM_MI_STATUS_HLIMIT_POS  (SHARED_MEM_MI_STATUS_BASE + 0x08)     //������λ�澯
#define SHARED_MEM_MI_STATUS_HLIMIT_NEG  (SHARED_MEM_MI_STATUS_BASE + 0x10)     //�Ḻ��λ�澯
#define SHARED_MEM_MI_STATUS_ENCODER_WARN (SHARED_MEM_MI_STATUS_BASE + 0x18)    //�������澯��־,uint64_t����
#define SHARED_MEM_MI_STATUS_SVO_WARN  (SHARED_MEM_MI_STATUS_BASE + 0x20)       //�ŷ��澯��־,uint64_t����


#define SHARED_MEM_MI_STATUS_SVO_ON     (SHARED_MEM_MI_STATUS_BASE + 0x28)              //��ʹ�ܱ�־��uint64_t���ͣ�һ��bit����һ����
#define SHARED_MEM_MI_STATUS_SVO_TRACK_ERR (SHARED_MEM_MI_STATUS_BASE + 0x30)      //��������޸澯
#define SHARED_MEM_MI_STATUS_SVO_SYNC_POS_ERR   (SHARED_MEM_MI_STATUS_BASE + 0x38)      //ͬ����λ��ƫ�����澯
#define SHARED_MEM_MI_STATUS_SVO_INTP_POS_ERR  (SHARED_MEM_MI_STATUS_BASE + 0x40)      //λ��ָ�����澯
#define SHARED_MEM_MI_STATUS_SVO_SYNC_TORQUE_ERR  (SHARED_MEM_MI_STATUS_BASE + 0x48)    //ͬ��������ƫ�����澯
#define SHARED_MEM_MI_STATUS_SVO_SYNC_MACH_ERR  (SHARED_MEM_MI_STATUS_BASE + 0x50)      //ͬ�����������ƫ�����澯

#define SHARED_MEM_MI_STATUS_SVO_WARN_CODE(n)  (SHARED_MEM_MI_STATUS_BASE + 0x100 + 0x04*n)       //��n����ŷ��澯��


//SC״̬�Ĵ���
#define SHARED_MEM_SC_STATUS_HLIMIT_POS   (SHARED_MEM_SC_STATUS_BASE)      //������������λ��־
#define SHARED_MEM_SC_STATUS_HLIMIT_NEG   (SHARED_MEM_SC_STATUS_BASE + 0x08)     //�����Ḻ����λ��־

//��������ͨ���Ĵ���  MI->SC
#define SHARED_MEM_CMD_RECV_WF(n)	(SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n)		//��n������֡��д��־
#define SHARED_MEM_CMD_RECV_RF(n)	(SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n + 0x02) //��n������֡�Ķ���־
#define SHARED_MEM_CMD_RECV_DATA0(n)  (SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n + 0x04) 	//��n������֡�ĵ�0��32λ����
#define SHARED_MEM_CMD_RECV_DATA1(n)  (SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n + 0x08) 	//��n������֡�ĵ�1��32λ����
#define SHARED_MEM_CMD_RECV_DATA2(n)  (SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n + 0x0C) 	//��n������֡�ĵ�2��32λ����
#define SHARED_MEM_CMD_RECV_DATA3(n)  (SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n + 0x10) 	//��n������֡�ĵ�3��32λ����
#define SHARED_MEM_CMD_RECV_DATA4(n)  (SHARED_MEM_CMD_RECV_BASE + MI_CMD_FRAME_SIZE*n + 0x14) 	//��n������֡�ĵ�4��32λ����


//��������ͨ���Ĵ���  SC->MI
#define SHARED_MEM_CMD_SEND_WF(n)	(SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n)			//��n������֡��д��־
#define SHARED_MEM_CMD_SEND_RF(n)	(SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n + 0x02) 	//��n������֡�Ķ���־
#define SHARED_MEM_CMD_SEND_DATA0(n)  (SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n + 0x04) 	//��n������֡�ĵ�0��32λ����
#define SHARED_MEM_CMD_SEND_DATA1(n)  (SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n + 0x08) 	//��n������֡�ĵ�1��32λ����
#define SHARED_MEM_CMD_SEND_DATA2(n)  (SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n + 0x0C) 	//��n������֡�ĵ�2��32λ����
#define SHARED_MEM_CMD_SEND_DATA3(n)  (SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n + 0x10) 	//��n������֡�ĵ�3��32λ����
#define SHARED_MEM_CMD_SEND_DATA4(n)  (SHARED_MEM_CMD_SEND_BASE + MI_CMD_FRAME_SIZE*n + 0x14) 	//��n������֡�ĵ�4��32λ����

//PMC���˶�ָ��ͨ��
#define SHARED_MEM_CMD_PMC_WF(n)	(SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n)		//��n������֡��д��־
#define SHARED_MEM_CMD_PMC_RF(n)	(SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n + 0x02) //��n������֡�Ķ���־
#define SHARED_MEM_CMD_PMC_DATA0(n)  (SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n + 0x04) 	//��n������֡�ĵ�0��32λ����
#define SHARED_MEM_CMD_PMC_DATA1(n)  (SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n + 0x08) 	//��n������֡�ĵ�1��32λ����
#define SHARED_MEM_CMD_PMC_DATA2(n)  (SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n + 0x0C) 	//��n������֡�ĵ�2��32λ����
#define SHARED_MEM_CMD_PMC_DATA3(n)  (SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n + 0x10) 	//��n������֡�ĵ�3��32λ����
#define SHARED_MEM_CMD_PMC_DATA4(n)  (SHARED_MEM_PMC_AXIS_POS_BASE + PMC_CMD_FRAME_SIZE*n + 0x14) 	//��n������֡�ĵ�4��32λ����

//PMC�Ĵ������ݵ�ַ����
//NC->PMC
#define SHARED_MEM_REG_TO_PMC_BASE 	(SHARED_MEM_BASE + 0x80000)				//NC->PMC�Ĵ����׵�ַ
#define SHARED_MEM_REG_TO_PMC_F		(SHARED_MEM_REG_TO_PMC_BASE + 0x0)		//F�Ĵ�����ַ

//PMC->NC
#define SHARED_MEM_REG_TO_NC_BASE  	(SHARED_MEM_BASE + 0x84000)				//PMC->NC�Ĵ����׵�ַ
#ifndef USES_PMC_2_0

#define SHARED_MEM_REG_TO_NC_G			(SHARED_MEM_REG_TO_NC_BASE + 0x0)		//G�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_X			(SHARED_MEM_REG_TO_NC_BASE + 0x0800)	//X�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_Y			(SHARED_MEM_REG_TO_NC_BASE + 0x0900)	//Y�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_R			(SHARED_MEM_REG_TO_NC_BASE + 0x0A00)	//R�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_K			(SHARED_MEM_REG_TO_NC_BASE + 0x0E00)	//K�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_A			(SHARED_MEM_REG_TO_NC_BASE + 0x0E80)	//A�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_D			(SHARED_MEM_REG_TO_NC_BASE + 0x1000)	//D�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_C			(SHARED_MEM_REG_TO_NC_BASE + 0x2000)	//C�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_T			(SHARED_MEM_REG_TO_NC_BASE + 0x2200)	//T�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_DC		(SHARED_MEM_REG_TO_NC_BASE + 0x2400)	//DC�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_DT		(SHARED_MEM_REG_TO_NC_BASE + 0x2600)	//DT�Ĵ�����ַ

#else

#define SHARED_MEM_REG_TO_NC_G			(SHARED_MEM_REG_TO_NC_BASE + 0x0)		//G�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_X			(SHARED_MEM_REG_TO_NC_BASE + 0x0800)	//X�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_Y			(SHARED_MEM_REG_TO_NC_BASE + 0x0A00)	//Y�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_R			(SHARED_MEM_REG_TO_NC_BASE + 0x0D00)	//R�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_K			(SHARED_MEM_REG_TO_NC_BASE + 0x01500)	//K�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_A			(SHARED_MEM_REG_TO_NC_BASE + 0x01600)	//A�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_D			(SHARED_MEM_REG_TO_NC_BASE + 0x1700)	//D�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_C			(SHARED_MEM_REG_TO_NC_BASE + 0x2700)	//C�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_T			(SHARED_MEM_REG_TO_NC_BASE + 0x2800)	//T�Ĵ�����ַ
#define SHARED_MEM_REG_TO_NC_E		    (SHARED_MEM_REG_TO_NC_BASE + 0x2900)	//E�Ĵ�����ַ
//#define SHARED_MEM_REG_TO_NC_TC         (SHARED_MEM_REG_TO_NC_BASE + 0x2900)    //T�Ĵ�����ǰֵ
//#define SHARED_MEM_REG_TO_NC_TM         (SHARED_MEM_REG_TO_NC_BASE + 0x2A00)    //T�Ĵ���Mask
//#define SHARED_MEM_REG_TO_NC_E		    (SHARED_MEM_REG_TO_NC_BASE + 0x2B00)	//E�Ĵ�����ַ

#endif



typedef ListBuffer<MiCmdFrame> MiCmdBuffer;   //�������ָ����Ϣ����
typedef ListBuffer<PmcCmdFrame> PmcCmdBuffer;  //�������PMC�˶�ָ�����


/**
 * @brief SC-MIͨѶ�࣬��Ҫ����SCģ����MIģ�齨��ͨѶ�����ù����ڴ淽ʽ
 */
class MICommunication {
public:

	virtual ~MICommunication();	//��������

	static MICommunication *GetInstance();   //����ģʽ����ȡ����ʵ����Ψһ���ʵ�

	void SetInterface();   //���ýӿ�

	ErrorType GetErrorCode(){return m_error_code;}	//��������Ĵ�����

    void ReadPhyAxisCurFedBckPos(double *pos_fb, double *pos_intp,double *speed,double *torque,double *angle, uint8_t count);    //��ȡ������λ�á��ٶȡ�����
	void ReadPhyAxisFbPos(double *pos_fb, uint8_t *phy_axis, uint8_t count); //��ȡָ��������ķ���λ��
#ifdef USES_SPEED_TORQUE_CTRL
	void ReadPhyAxisCurFedBckSpeedTorque(double *speed, double *torque, uint8_t count);    //��ȡ�������ٶȺ�����
#endif
    void ReadSpindleAndle(uint16_t *angle,uint8_t count);   //��ȡ����ʵʱ�Ƕȷ���

//	void ReadPhyAxisIntpPos(double *pos, uint8_t count);   //��ȡ�������е�岹λ��
	bool ReadPhyAxisSpeed(int32_t *speed, uint8_t index);  	//��ȡָ���������ٶ�
    void ReadPhyAxisEncoder(int64_t *encoder, uint8_t count);   //��ȡ�����ᵱǰ����������
	void ReadPmcAxisRemainDis(double *pos, uint8_t count);    //��ȡPMC�����ƶ���
    void ReadTapErr(int32_t *err, int32_t *err_now,uint8_t cnt = 8);  // ��ȡ�ض���˿��Ĺ�˿���(���ֵ)
	bool ReadCmd(MiCmdFrame &data);    //��ȡָ��
	bool WriteCmd(MiCmdFrame &data, bool resend = false);   //����ָ��
	bool SendPmcCmd(PmcCmdFrame &data, bool resend = false);     //����PMC���˶�ָ��
	bool GetPmcRegByte(const int reg_type, uint8_t &value);  	//��ȡPMC���ֽڼĴ�����ֵ
	bool SetPmcRegByte(const int reg_type, const uint8_t value);	//����PMC���ֽڼĴ�����ֵ

	bool GetPmcRegWord(const int reg_type, uint16_t &value);  	//��ȡPMC˫�ֽ��ֽڼĴ�����ֵ
	bool SetPmcRegWord(const int reg_type, const uint16_t value);	//����PMC˫�ֽڼĴ�����ֵ

	bool ReadPmcReg(int sec, uint8_t *reg);		//��ȡPMC�Ĵ���������ַ�ζ�ȡ
    bool WritePmcReg(int sec, uint8_t *reg);    //д��PMC�Ĵ���������ַ��д��
	bool ReadPmcPeriod();
    //bool ReadESPSignal();                       //��ȡX��ͣ�ź�

    bool SetAxisRef(uint8_t axis, int64_t encoder);		//������ο���
    void SetAxisRefCur(uint8_t axis, double mach_pos);		//����ָ����Ĳο���

	bool StartMI();   //����MI����

	char *GetPmcDataAddr();		//��ȡPMC����ͼ�������ĵ�ַ

    // opt:�������ͣ���ϸ�ɲ鿴[SC-MI�������]�ĵ�
    // axis:��ţ���1��ʼ
    // enable: 0:���ܹ� 1:���ܿ�
    void SendOperateCmd(uint16_t opt, uint8_t axis, uint16_t enable);   //���������ָ��
    void SendAxisEnableCmd(uint8_t axis, bool enable, uint8_t pos_req = 0);   // ����ʹ��
    void SendAxisMLK(uint8_t axis, bool MLK);    // ���е��ס

    // spd_aixs: ������ţ���1��ʼ
    // z_axis: z����ţ���1��ʼ
    void SendTapAxisCmd(uint8_t chn,uint8_t spd_axis,uint8_t z_axis); // ���͹�˿���

    void SendTapRatioCmd(uint8_t chn,int32_t ratio); // ���͹�˿����
    // ����ͬ��������棬���ٶ�ǰ�����棬��λ�ñ�������
    void SendTapParams(uint8_t chn,uint32_t error_gain,
                       uint32_t feed_gain,uint32_t ratio_gain);
    void SendTapStateCmd(uint8_t chn,bool enable, bool rtnt = false); // ���͹�˿״̬��MI
    // axis: ������ţ���1��ʼ
    void SendSpdLocateCmd(uint8_t chn, uint8_t axis, bool enable); // �������ᶨλ����
    // axis: ��ţ���1��ʼ
    // type: 0��ֱ����λ�ÿ��������1����ת��λ�ÿ��������2���ٶ�ָ�������3������ָ�����
    void SendAxisCtrlModeSwitchCmd(uint8_t axis,uint8_t type);
    // enable: 0:MIʹ��Ĭ�ϵĲ����߼�  1:�Զ������ֲ���
    // step: ���ֲ��� ��λ:um
    void SendMpgStep(uint8_t chn,bool enable,uint16_t step);
    // mask: 64bit��Ӧ���64����,��Ҫͬ�������Ӧλ��1
    void SendSyncAxis(int64_t mask);
    // master: ������ţ���1��ʼ
    // slave: �Ӷ���ţ���1��ʼ
    // enable: 0:ͬ����Ч  1:ͬ����Ч
    void SendEnSyncAxis(uint8_t master, uint8_t slave, bool enable);
    // chn: ͨ����
    // enable: �Ƿ������ָ���
    // reverse: �Ƿ������
    void SendHandwheelState(uint8_t chn, bool trace_enable, bool trace_reverse,
                            bool insert_enable);
    // chn: ͨ����
    // enable: �Ƿ������ֲ���
    void SendHandwheelInsert(uint8_t chn, bool enable);
    // chn: ͨ����
    // axis: ���ֲ�����ѡ
    void SendHandwheelInsertAxis(uint8_t chn, uint8_t axis);
    // is_positive: �Ƿ�Ϊ����λ
    // mask: �澯���� ÿһbit����һ�����Ƿ��ڸ澯״̬
    void SendHardLimitState(bool is_positive,uint64_t mask);

	bool ReadEncoderWarn(uint64_t &value);		//��ȡ��������澯��־
	bool ReadServoHLimitFlag(bool pos_flag, uint64_t &value);   //��ȡ�ŷ���λ�澯��Ϣ
    bool ReadServoWarn(uint64_t &value);			//��ȡ���ŷ��澯��־
    bool ReadServoWarnCode(uint8_t axis, uint32_t &value); 	//��ȡָ������ŷ��澯��
    bool ReadAxisWarnFlag(uint64_t &warn);        //��ȡ��澯��־
    bool ReadServoOnState(uint64_t &value);		//��ȡ��ʹ��״̬
    bool ReadTrackErr(uint64_t &value);        //��ȡ�����������澯
    bool ReadSyncPosErr(uint64_t &value);     //��ȡͬ����ָ��ƫ�����澯
    bool ReadIntpPosErr(uint64_t &value);      //��ȡ��λ��ָ�����澯
    bool ReadSyncTorqueErr(uint64_t &value);   //��ȡͬ��������ƫ���
    bool ReadSyncMachErr(uint64_t &value);     //��ȡͬ�����������ƫ���

    bool WriteAxisHLimitFlag(bool pos_flag, const uint64_t value);   //д��������Ӳ��λ��־

    template<typename T>
    void SendMiParam(uint8_t axis, uint32_t para_no, T data)  //����MI����
    {
        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.cmd = CMD_MI_SET_PARAM;
        cmd.data.axis_index = axis;
        memcpy(cmd.data.data, &para_no, 4);
        memcpy(&cmd.data.data[2], &data, sizeof(T));
        WriteCmd(cmd);
    }


private://˽�нӿ�
	MICommunication();	//���캯��

	int InitThread();    //��ʼ���߳�
	int QuitThread();		//�˳��߳�

	void InitCmdChannel();   //��ʼ������ͨ��

	static void *ProcessCmdThread(void *args); //����������̺߳���


	bool ReadRegister64(const uint32_t addr, int64_t& value);	 	//�Ĵ���������
	bool ReadRegister32(const uint32_t addr, int32_t& value);		//�Ĵ�����ȡ����
	bool WriteRegister32(const uint32_t addr, const int32_t value);	//�Ĵ���д����
	bool WriteRegister64(const uint32_t addr, const int64_t value);	//�Ĵ���д����


	bool InitSharedMemory();   	//��ʼ�������ڴ�
	bool CloseSharedMemory();      //�رչ����ڴ�

	bool ProcessCmdFun();  	//�������

	void CalMiCmdCrc(MiCmdFrame &cmd);     //����MI�������CRC
	void CalPmcCmdCrc(PmcCmdFrame &cmd);   //����PMC�������CRC

private://˽�г�Ա����
	static MICommunication *m_p_instance;    //��ʵ������

	ChannelEngine *m_p_channel_engine;   //ͨ������ָ��

	int m_n_threshold;      //����Ԥ����ֵ

	uint8_t *m_p_shared_base;    //����������ַ
	int m_n_mem_file;         //�ļ��򿪾��

	ErrorType m_error_code;    //���һ�εĴ�����

	pthread_t m_thread_process_cmd;    //������߳�

	uint8_t m_n_cur_send_cmd_index;    //��ǰд����֡���   0~MI_CMD_FIFO_DEPTHѭ��
	uint8_t m_n_cur_recv_cmd_index;		//��ǰ������֡���  0~MI_CMD_FIFO_DEPTHѭ��

	pthread_mutex_t m_mutex_cmd_down;   //��������ͨ��������,д�������
	pthread_mutex_t m_mutex_pmc_cmd;    //pmc�˶�ָ��ͨ����������д��ָ���

	MiCmdBuffer *m_list_cmd;		//�����ָ���
	PmcCmdBuffer *m_list_pmc;       //�������PMC�˶�ָ���

	uint8_t m_n_cur_send_pmc_index;  //��ǰдPMC�˶�ָ��֡���   0~PMC_CMD_FIFO_DEPTHѭ��
};

#endif /* SRC_COMMUNICATION_MI_COMMUNICATION_H_ */
