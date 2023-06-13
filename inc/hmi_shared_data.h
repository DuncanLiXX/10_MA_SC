/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file hmi_shared_data.h
 *@author gonghao
 *@date 2020/03/19
 *@brief ��ͷ�ļ�������HMI���õ����ݵ����������ļ�������HMI����ͬ������
 *@version
 */

#ifndef INC_HMI_SHARED_DATA_H_
#define INC_HMI_SHARED_DATA_H_

#include <stdint.h>
#include <string>
#include <list>
#include "global_definition.h"

//HMI�������س���
const int kMaxHmiDataLen = 1024;   //HMI������й̶��������ĳ���
const int kHmiCmdHeadLen = 10;     //HMI�����ͷ�ĳ���
const int kMaxHmiCmdFrameLen = kMaxHmiDataLen + kHmiCmdHeadLen;   //HMI������ܳ���

//�汾�ַ�����󳤶�
const int kMaxVersionLen = 32;

//ϵͳ�������
const int kMaxAxisNum = 32;    //12;


const uint8_t kHmiCmdTimeout = 10;    //HMI���ʱʱ�䣬��λ100ms
const uint8_t kHmiCmdResend = 2;     //HMI������ط�����

const uint16_t kMaxHmiCmdFrameIndex = 0X7FFF;    //HMI��������֡��

//ͨ�����
const int kMaxChnCount = 4;   //���ͨ������
const int kMaxAxisChn = 8;   //ÿ��ͨ�������������
const int kMaxSpindleChn = 6;   //ÿ��ͨ��֧�ֵ������������
const int kMaxGModeCount = 40;   //G����ģ̬����


//ע��: �˴�������windows�Ķ��岻ͬ��windowsΪ256�ֽ�
const int kMaxPathLen = 128;          //·����󳤶�
const int kMaxFileNameLen = 128;      //�ļ�����󳤶�

//NC�ļ�ǩ��������
const int kNcFileSignLen = 16;   //ǩ��������

//�������
const int kMaxToolCount = 60;    //ϵͳ֧����󵶾�����

//���ղ����������
const int kMaxProcParamCount = 10;    //ϵͳ֧�ֵĹ��ղ����������

//HMIͨѶ
#define PORT_UDP_CMD   (9100)   //UDP����ͨѶ�˿�
#define PORT_UDP_CMD_SEND (9101)   //UDP�����Ŀ�Ķ˿�
#define PORT_TCP_MONITOR     (9210)  //TCP������ݶ˿ڣ����ڷ���
#define PORT_TCP_FILE        (9220)   //TCP�ļ�����˿�
#define PORT_TCP_DATA        (9230)   //TCP���ݴ���

#define STR_HMI_SHAKE  "HELLO ARADEX CNC"  //�����ַ���
#define STR_HMI_SHAKE_RESP "HELLO ARADEX HMI"  //���ֻظ��ַ���


//�ظ��������
#define APPROVE   (0x00)    //��������
#define REFUSE    (0x01)    //�ܾ�����
#define NOTEXIST  (0x02)    //�ļ������ڣ��ܾ�
#define SUCCEED   (0x00)    //ִ�гɹ�
#define FAILED    (0x01)    //ִ��ʧ��

//�ļ�����
#define FILE_UNKOWN (-1)
#define FILE_REG  (0x00)   //��ͨ�ļ�
#define FILE_DIR  (0x01)   //�ļ�Ŀ¼

//�汾�����
#define SOFTWARE_VER (0x00)    //����汾
#define HARDWARE_VER (0x01)    //Ӳ���汾

//�������ģʽ����
#define SPD_CTRL_SPEED   (0x00) 		//�����ٶȿ���
#define SPD_CTRL_POS     (0x01)		//����λ�ÿ���


//��������
#define SIMULATE_OUTLINE (0x01)    	//�������棬���ƹ�������
#define SIMULATE_TOOLPATH  (0x02)		//��·��飬�ӹ�ʱ��ʵ�ʵ�·�켣
#define SIMULATE_MACHINING (0x03)	    //�ӹ����棬��������ӹ��������᲻ʵ���˶����ڲ���������һ��

//������Ĵ����εĳ���,�ܳ���6528bytes
#ifndef USES_PMC_2_0
#define X_REG_COUNT (128)			//X�Ĵ�������
#define Y_REG_COUNT (128)			//Y�Ĵ�������
#define F_REG_COUNT (kMaxChnCount*256)			//F�Ĵ�������
#define G_REG_COUNT (kMaxChnCount*256)			//G�Ĵ�������
#define K_REG_COUNT (64)			//K�Ĵ�������
#define A_REG_COUNT (64)			//A�Ĵ�������
#define R_REG_COUNT (512)			//R�Ĵ�������
#define D_REG_COUNT (1024)			//D�Ĵ�������
#define T_REG_COUNT (128)			//T/DT�Ĵ�������
#define C_REG_COUNT (128)			//C/DC�Ĵ�������
#else
#define X_REG_COUNT (265)			//X�Ĵ�������   X0~X127(ͨ��1��+ X200~X327��ͨ��2��+ Ӳ��ַX1000~X1007 + ����DI X2000  HD1~8
#define Y_REG_COUNT (260)			//Y�Ĵ�������   Y0~Y127(ͨ��1��+ Y200~Y327��ͨ��2��+ Ӳ��ַY1000~Y1003
#define F_REG_COUNT (kMaxChnCount*256)			//F�Ĵ�������   ��ͨ��F0~F255,4ͨ��
#define G_REG_COUNT (kMaxChnCount*256)			//G�Ĵ�������   ��ͨ��G0~G255,4ͨ��
#define K_REG_COUNT (20)			//K�Ĵ�������
#define A_REG_COUNT (25)			//A�Ĵ�������
#define R_REG_COUNT (2048)			//R�Ĵ�������
#define D_REG_COUNT (2048)			//D�Ĵ�������
#define T_REG_COUNT (128)			//T�Ĵ�������   �����Ĵ���ռ2�ֽڣ����ֽ���Ϊ256�ֽ�
#define C_REG_COUNT (80)			//C�Ĵ�������   �����Ĵ���ռ4�ֽڣ�ǰ���ֽ�ΪԤ��ֵ�������ֽ�Ϊ��ǰֵ�����ֽ���Ϊ320�ֽ�
//#define TC_REG_COUNT (256)          //TC�Ĵ�������
//#define TM_REG_COUNT (16)           //TM�Ĵ�����16���ֽ�
#define E_REG_COUNT (8000)         //E�Ĵ�������
#endif


/**
 * @brief HMICmdFrame�ṹ���ڶ���HMI��SC֮�������֡
 */
struct HMICmdFrame {
	HMICmdFrame(){frame_number = 0;}
	~HMICmdFrame(){}
	uint16_t frame_number;   //֡�ţ�0-32767ѭ����bit15���ڱ�־�ظ����ݰ�
	uint16_t channel_index;  //ͨ����
	uint16_t cmd;            //�����
	uint16_t cmd_extension;  //��չ�����
	uint16_t data_len;       //�������ֽ���,0~255
	char data[kMaxHmiDataLen];    //��������
//	char *data_huge;            //��������ָ��
};


/**
 * @brief ����HMI��SC֮���������
 */
enum HMICmdCode {
	//HMI-->SC
	CMD_HMI_HEART_BEAT = 0,		    //�������� 0
	CMD_HMI_DEVICE_SCAN,			//�豸ɨ������ 1
	CMD_HMI_SHAKEHAND,				//HMI������������� 2
	CMD_HMI_GET_FILE,             	//HMI��SC�����ļ�,��ͬ���ļ�����չ��������� 3
	CMD_HMI_SEND_FILE,              //HMI��SC�����ļ�����ͬ���ļ�����չ��������� 4
	CMD_HMI_DISCONNECT,             //HMI�ر����� 5
	CMD_HMI_SET_PARA,               //HMI������������ 6
	CMD_HMI_GET_PARA,               //HMI��ȡ�������� 7
    CMD_HMI_CLEAR_WORKPIECE,        //HMI������ռӹ�����,��ʱ����(���ְ�ҹ��) 8
	CMD_HMI_SET_CUR_CHANNEL,	    //HMI���õ�ǰͨ���� 9
	CMD_HMI_NEW_ALARM,              //HMI�����µĸ澯 10
	CMD_HMI_RESTART,                //�ӹ���λ 11
	CMD_HMI_SIMULATE,				//�ӹ����� 12
	CMD_HMI_SET_NC_FILE,		    //���õ�ǰ�ӹ��ļ� 13
	CMD_HMI_FIND_REF_POINT = 14,	//ȷ���ο��� 14
//	CMD_HMI_MDA_CODE_TRANSFER,		//MDA���봫�� 15
	CMD_HMI_GET_VERSION = 16,       //HMI��ȡ�汾��Ϣ  16
	CMD_HMI_GET_FILE_SIGNATURE,     //HMI��ȡָ���ļ�������ǩ�� 17
	CMD_HMI_NC_FILE_COUNT,           //HMI��ȡSCģ��洢��NC�ӹ��ļ����� 18
	CMD_HMI_NC_FILE_INFO,            //HMI��ȡNC�ļ���Ϣ 19
	CMD_HMI_GET_CHN_COUNT,           //HMI��ȡͨ������   20
	CMD_HMI_GET_CHN_STATE,           //HMI��ȡͨ����ǰ״̬ 21
	CMD_HMI_READY,                   //HMI׼������
	CMD_HMI_UPDATE,					 //HMI֪ͨSC������������ļ�����
	CMD_HMI_FILE_OPERATE,            //HMI֪ͨ��NC�ļ����в��������������������չ����Ÿ���
	CMD_HMI_MOP_KEY,                 //HMI����MOP������Ϣ
	CMD_HMI_GET_PMC_REG,			 //HMI��SC����PMC�Ĵ�����ֵ
	CMD_HMI_SET_PMC_REG,			 //HMI��SC����PMC�Ĵ�����ֵ
	CMD_HMI_GET_PMC_UUID,			 //HMI��SC����PMC��UUID
	CMD_HMI_SET_REF_POINT,			 //HMI֪ͨSC����ǰλ������Ϊ��ǰ���ԭ��
	CMD_HMI_GET_MACRO_VAR,			 //HMI��SC����������ֵ
	CMD_HMI_SET_MACRO_VAR,			 //HMI��SC���ú�����Ĵ�����ֵ
    CMD_HMI_CLEAR_TOTAL_PIECE,       //HMI��������ܹ�����
    CMD_HMI_SET_REQUIRE_PIECE,       //HMI���������������

//	CMD_HMI_SET_CUR_PMC_AXIS,        //HMI���õ�ǰPMC��  0x20
	CMD_HMI_SET_CALIBRATION = 35,	 //HMI��SC��������������궨ָ��  0x23
	CMD_HMI_AXIS_MANUAL_MOVE = 37,   //HMIָ��SC��ĳ������ֶ��ƶ�   0x25
	CMD_HMI_GET_LIC_INFO = 39,       //HMI��SC������Ȩ��Ϣ   0x27
	CMD_HMI_SEND_LICENSE,            //HMI��SC������Ȩ��     0x28
	CMD_HMI_MANUAL_TOOL_MEASURE,     //HMI��SC�����ֶ��Ե�����  0x29
	CMD_HMI_GET_ESB_INFO,            //HMI��SC��ȡESB�ļ�����   0x2A
	CMD_HMI_ESB_OPERATE,             //HMI֪ͨSC��ָ��ESB�ļ����в���  0x2B
	CMD_HMI_GET_IO_REMAP,            //HMI��SC��ȡIO�ض�������  0x2C
	CMD_HMI_SET_IO_REMAP,            //HMI��SC����IO�ض�������  0x2D
	CMD_HMI_SET_PROC_PARAM,          //HMI��SC���ù�����ز���  0x2E
	CMD_HMI_GET_PROC_PARAM,          //HMI��SC��ȡ������ز���  0x2F
	CMD_HMI_SET_PROC_GROUP,          //HMI��SC���õ�ǰ���ղ������  0x30
	CMD_HMI_GET_PROC_GROUP,          //HMI��SC��ȡ��ǰ���ղ������  0x31
	CMD_HMI_SET_CUR_MACH_POS,        //HMI��SC����ָ����Ļ�е����   0x32
	CMD_HMI_CLEAR_MSG,               //HMI֪ͨSC�����Ϣ���������󡢾������ʾ)0x33
    CMD_HMI_NOTIFY_GRAPH = 0x35,      //HMI֪ͨSC����ͼ��ģʽ    0x35
	CMD_HMI_SYNC_TIME,               //HMI��SC��ѯ��ǰϵͳʱ��  0x36
	CMD_HMI_CHECK_SYNC_EN,           //HMI��SC��ѯͬ����״̬ 0x37
	CMD_HMI_GET_SYS_INFO,
    CMD_HMI_CLEAR_MACHINETIME_TOTAL, //HMI��SC��������ۼ�ʱ�� 0x39
    CMD_HMI_GET_HANDWHEEL_INFO,      //HMI��SC��ȡ������Ϣ 0x3A
    CMD_HMI_SET_HANDWHEEL_INFO,      //HMI��SC����������Ϣ 0x3B
    CMD_HMI_GET_ERROR_INFO,          //HMI��SC��ȡ������Ϣ 0x3C
    CMD_HMI_SET_ALL_COORD,           //HMI��SC���õ�ǰͨ�������й�������ϵ 0x3D
    CMD_HMI_BACKUP_REQUEST,          //HMI��SC���󱸷� 0x3E
    CMD_HMI_RECOVER_REQUEST,         //HMI��SC����ָ� 0x3F
    CMD_HMI_CLEAR_ALARMFILE,         //HMI��SC������ձ����ļ� 0x40
    CMD_HMI_ABSOLUTE_REF_SET,        //HMI��SC�������ʽ���������� 0x41
    CMD_HMI_SET_ALL_TOOL_OFFSET,     //HMI��SC�����������е�ƫֵ 0x42
    CMD_HMI_GET_CPU_INFO,            //HMI��SC��ȡ CPU �ڴ� ռ����Ϣ
    CMD_HMI_SERVE_DATA_REQUEST,      //HMI��SC����׼���ŷ�����
    CMD_HMI_SERVE_DATA_RESET,        //HMI��SC�����ŷ�������λ
    CMD_HMI_CLEAR_IO_MAP,            //HMI��SC�������IO��ӳ������
    CMD_HMI_GET_FILE_SYSTEM,         //HMI��SC�����ļ�ϵͳ
    CMD_HMI_GET_MKDIR,               //HMI��SC���󴴽�Ŀ¼
	// ľ��ר��
	CMD_HMI_APPEND_ORDER_LIST,	     //HMI����ų̼ӹ��ļ�
	CMD_HMI_CLEAR_ORDER_LIST,		 //HMI����ų��ļ��б�

	//SC-->HMI
	CMD_SC_NEW_ALARM = 100,			//SC��HMI�����µĸ澯��Ϣ 100
	CMD_SC_CLEAR_ALARM,			    //����澯
	CMD_SC_RESET,                   //��λ
	CMD_SC_WORK_STATE,				//֪ͨHMI���¼ӹ�״̬���ӹ��С���ͣ�����
    CMD_SC_WORK_MODE,               //����ģʽ�л���AUTO/MDA/MANUAL/HANDWHEEL
	CMD_SC_DISPLAY_MODE,     		//��ʾģʽ�л����ӹ����������á���Ϣ������
	CMD_SC_STATUS_CHANGE,			//״̬�仯�����絥�Ρ�ѡͣ�����Ρ������С����ָ��ٵȵ�
	CMD_SC_EMERGENCY,               //��ͣ
	CMD_SC_RESTART_LINE,            //֪ͨHMI��ǰ�ӹ���λ����ɨ�������
	CMD_SC_MODE_CHANGE,             //֪ͨHMI��Gָ��ģֵ̬�����仯������T��D��H��F��S
	CMD_SC_DISPLAY_FILE,            //֪ͨHMI�л���ʾ���ļ�
	CMD_SC_WORKPIECE_COUNT,			//֪ͨHMI���¹�������ֵ  0x6F
//	CMD_SC_REF_POINT_STATUS,        //֪ͨHMI��ο���״̬
	CMD_SC_READY = 0x71,			//֪ͨHMI��λ����ģ���Ѿ�׼������
	CMD_SC_MESSAGE,                 //֪ͨHMI��Ϣ
	CMD_SC_MDA_DATA_REQ,			//��HMI����MDA����
	CMD_SC_UPDATE_MODULE_STATUS,	//��HMI����ģ������״̬ 0x74
	CMD_SC_REQ_START,               //SC��HMI����ʼ�ӹ�
	CMD_SC_TOOL_CHANGE,             //SC֪ͨHMI����
	CMD_SC_MECH_ARM_ERR,            //SC֪ͨHMI��е���������쳣 0x77  ³������
	CMD_SC_TRANS_FILE_OVER,         //SC֪ͨHMI�ļ��������   0x78
	CMD_SC_TOOL_MEASURE_OVER,       //SC֪ͨHMI�ֶ��Ե����  0x79
	CMD_SC_PARAM_CHANGED,           //SC֪ͨHMI������������   0x7A
	CMD_SC_NOTIFY_MACH_OVER,        //SC֪ͨHMI�ӹ����       0x7B
    CMD_SC_NOTIFY_MCODE,			//SC֪ͨHMI M����ִ��
    CMD_SC_NOTIFY_ALARM_CHANGE,     //SC֪ͨHMI������Ϣ�ı�
    CMD_SC_BACKUP_STATUS,           //SC֪ͨHMI��ǰ����״̬     0x7E
    CMD_SC_NOTIFY_TRACELOG,         //SC֪ͨHMI������¼        0x7F
    CMD_SC_NOTIFY_PROTECT_STATUS,   //SC֪ͨHMI����״̬        0x80
    CMD_SC_NOTIFY_GATHER_FINISH,    //SC֪ͨHMI�ɼ����        0x81
	CMD_HMI_GUARD = 255       //HMI���������� 0xFF
};

//����MOP��ֵ����
enum VirtualMOPKeyValue{
	MOP_KEY_AUTO = 1,					//�Զ�
	MOP_KEY_MDA = 2,					//MDA
	MOP_KEY_JOG = 3,					//�ֶ�����
	MOP_KEY_JOG_STEP = 4,				//�ֶ�����
	MOP_KEY_HANDWHEEL = 5,				//����

	MOP_KEY_SINGLE_BLOCK = 10,			//����
	MOP_KEY_JUMP = 11,					//����
	MOP_KEY_M01 = 12,					//ѡͣ
	MOP_KEY_HW_TRACE = 13,				//���ָ���


	MOP_KEY_G00_RATIO = 20,				//G00����
	MOP_KEY_FEED_RATIO = 21,			//�ֶ�/��������
	MOP_KEY_SPD_RATIO = 22,				//���ᱶ��

	MOP_KEY_STEP_LEN = 30,				//����
	MOP_KEY_DIR_FOREWARD = 31,			//+
	MOP_KEY_DIR_REVERSE = 32,			//-
	MOP_KEY_RAPID = 33,					//����
	MOP_KEY_KEY_X_AXIS = 34,			//X��
	MOP_KEY_KEY_Y_AXIS = 35,			//Y��
	MOP_KEY_KEY_Z_AXIS = 36,			//Z��
	MOP_KEY_KEY_4_AXIS = 37,			//4��
	MOP_KEY_KEY_5_AXIS = 38,			//5��
	MOP_KEY_KEY_6_AXIS = 39,			//6��

	MOP_KEY_LIGHT = 40,					//����
	MOP_KEY_CHIP = 41,					//��м
	MOP_KEY_GAS = 42,					//����
	MOP_KEY_COOL = 43,					//��ȴ

	MOP_KEY_TOOL_D = 50,				//����ƫ��
	MOP_KEY_SPD_TOOL = 51,				//�����趨
	MOP_KEY_TOOL_MEASURE = 52,			//�Զ��Ե�
	MOP_KEY_SPD_SPEED = 55,				//����ת��

	MOP_KEY_MAGAZINE_FOREWARD = 60,		//��ѡ��
	MOP_KEY_MAGAZINE_REVERSE = 61,		//��ѡ��
	MOP_KEY_MAGAZINE_RESET = 62,		//�������
	MOP_KEY_SPD_CLAMP = 63,				//�����ͷ
	MOP_KEY_SPD_FOREWARD = 65,			//������ת
	MOP_KEY_SPD_STOP = 66,				//����ͣת
	MOP_KEY_SPD_REVERSE = 67,			//���ᷴת

	MOP_KEY_EMERGENCY = 70,				//��ͣ
	MOP_KEY_CLEAR_ALARM = 80,			//����澯
	MOP_KEY_SYS_RESET = 81,				//ϵͳ��λ

	MOP_KEY_START = 100,				//ѭ������
	MOP_KEY_PAUSE = 101,				//��������

	MOP_KEY_AXIS = 102,                 //PMC��ѡ��
	MOP_KEY_CUR_AXIS = 103,             //��ǰ��ѡ��

#ifdef USES_GRIND_MACHINE
	MOP_KEY_Z_AXIS_POS = 500,			//Z�������ֶ�
	MOP_KEY_C_AXIS_POS = 501,			//C�������ֶ�
	MOP_KEY_C_AXIS_NEG = 502,			//C�Ḻ���ֶ�
	MOP_KEY_Z_AXIS_NEG = 503,			//Z�Ḻ���ֶ�
	MOP_KEY_X_AXIS_NEG = 504,           //X�Ḻ���ֶ�
	MOP_KEY_X_AXIS_POS = 505,			//X�������ֶ�

	MOP_KEY_PMC_X_AXIS_POS = 510,       //PMC��X����
	MOP_KEY_PMC_X_AXIS_NEG = 511,       //PMC��X����
	MOP_KEY_PMC_Y_AXIS_POS = 512,       //PMC��Y����
	MOP_KEY_PMC_Y_AXIS_NEG = 513,       //PMC��Y����
	MOP_KEY_PMC_LEFT_AXIS_POS = 514,    //�������������
	MOP_KEY_PMC_LEFT_AXIS_NEG = 515,    //��������Ḻ��
	MOP_KEY_PMC_RIGHT_AXIS_POS = 516,   //�Ҳ�����������
	MOP_KEY_PMC_RIGHT_AXIS_NEG = 517,   //�Ҳ������Ḻ��


	MOP_KEY_WORKPIECE_CLAMP = 600,		//�����н�
	MOP_KEY_VACUUM = 601,				//���
	MOP_KEY_SPINDLE_START = 602,		//��������
	MOP_KEY_SPINDLE_STOP = 603,			//����ֹͣ
	MOP_KEY_NEW_TRAY = 604,				//������
	MOP_KEY_PAUSE_CARRY = 605,          //��ͣ������


	MOP_KEY_IO_0 = 610,			//
	MOP_KEY_IO_1 = 611,
	MOP_KEY_IO_2 = 612,
	MOP_KEY_IO_3 = 613,
	MOP_KEY_IO_4 = 614,
	MOP_KEY_IO_5 = 615,
	MOP_KEY_IO_6 = 616,
	MOP_KEY_IO_7 = 617,
	MOP_KEY_IO_8 = 618,
	MOP_KEY_IO_9 = 619,
	MOP_KEY_IO_10 = 620,
	MOP_KEY_IO_11 = 621,
	MOP_KEY_IO_12 = 622,
	MOP_KEY_IO_13 = 623,
	MOP_KEY_IO_14 = 624,
	MOP_KEY_IO_15 = 625,
	MOP_KEY_IO_16 = 626,			//
	MOP_KEY_IO_17 = 627,
	MOP_KEY_IO_18 = 628,
	MOP_KEY_IO_19 = 629,
	MOP_KEY_IO_20 = 630,
	MOP_KEY_IO_21 = 631,
	MOP_KEY_IO_22 = 632,
	MOP_KEY_IO_23 = 633,
	MOP_KEY_IO_24 = 634,
	MOP_KEY_IO_25 = 635,
	MOP_KEY_IO_26 = 636,
	MOP_KEY_IO_27 = 637,
	MOP_KEY_IO_28 = 638,
	MOP_KEY_IO_29 = 639,
	MOP_KEY_IO_30 = 640,
	MOP_KEY_IO_31 = 641,

	MOP_KEY_FETCHING_PIECE = 700,    //ȡ����
	MOP_KEY_START_WORKING = 701,     //ȡ�����


#endif

	MOP_KEY_GUARD						//����
};

//�������Ͷ���
enum ErrorType {
	ERR_NONE = 0,        //�޴���
	//ϵͳSC�ڲ�����  1~999
    ERR_SC_INIT         = 1,    //SCģ���ʼ������
    ERR_MI_COM_INIT     = 2,    //MIͨѶģ���ʼ������
    ERR_MC_COM_INIT     = 3,	//MCͨѶģ���ʼ������
    ERR_MEMORY_NEW      = 4,    //�ڴ����ʧ��
    ERR_OPEN_DIR        = 5,	//���ļ�Ŀ¼ʧ��
    ERR_OPEN_FILE       = 6,	//���ļ�ʧ��
    ERR_READ_FILE       = 7,    //��ȡNC�ļ�ʧ��
    ERR_SAVEAS_FILE     = 8,    //�ļ����Ϊʧ��
    ERR_LOAD_MDA_DATA   = 9,    //����MDA����ʧ��
    ERR_INIT_BREAKCONTINUE = 10,    //��ʼ���ϵ�����߳�ʧ��
    ERR_INIT_UPDATE     = 11,   //��ʼ��ģ�������߳�ʧ��
    ERR_LOAD_SP6        = 12,   //��ʼ������SPARTAN6����ʧ��
    ERR_LOAD_MI         = 13,   //��ʼ������MIģ��ʧ��
    ERR_MC_INIT         = 14,   //MC����ʧ��
    ERR_MI_INIT         = 15,   //MI����ʧ��
    ERR_POWER_OFF       = 16,   //����澯
    ERR_IMPORT_CONFIG   = 17,   //���õ������������
    ERR_CHN_SPINDLE_OVERRUN = 18,   //ͨ��������������
    ERR_PMC_SDLINK_CONFIG   = 19,   //SDLINK������Ϣ����


    ERR_SYSTEM_TIME         = 80,   //ϵͳʱ���쳣
    ERR_SYSTEM_FILE         = 81,   //ϵͳ�ļ��쳣
    ERR_NO_LIC              = 82,   //����Ȩ
    ERR_LICENSE_INVALID     = 83,   //�Ƿ���Ȩ
    ERR_LIC_OVERTIME        = 84,   //ϵͳ��Ȩ�ѹ���
    ERR_LIC_DUE_SOON        = 85,   //ϵͳ��Ȩ��������

    ERR_UPDATE_MC           = 100,  //MCģ������ʧ��
    ERR_UPDATE_MI           = 101,  //MIģ������ʧ��
    ERR_UPDATE_PL           = 102,  //PLģ������ʧ��
    ERR_UPDATE_SC           = 103,  //SCģ������ʧ��
    ERR_UPDATE_PMC          = 104,  //PMC����ʧ��
    ERR_UPDATE_SPARTAN      = 105,  //SPARTAN����ʧ��
    ERR_UPDATE_SCMODBUS     = 106,  //SCModbusģ������ʧ��

    ERR_IMPORT_PC_DATA      = 107,  //�ݲ����ݵ���ʧ��
    ERR_UPDATE_DISK         = 108,  //һ������ʧ��

    ERR_EN_SYNC_AXIS        = 150,  //ͬ���Ὠ��ͬ��ʧ��
    ERR_DIS_SYNC_AXIS       = 151,  //ͬ������ͬ��ʧ��
    ERR_INIT_SYNC_AXIS      = 152,  //ͬ�����ʼ��ͬ��ʧ��

    ERR_VOLTAGE_OVER        = 500,  //��ѹ�澯
    ERR_VOLTAGE_UNDER       = 501,  //Ƿѹ�澯
    ERR_BATTERY_VOL_UNDER   = 502,  //��ص�ѹ��
    ERR_AXIS_CURRENT_OVER   = 503,  //������澯
    ERR_USB_CURRENT_OVER    = 504,  //USB�����澯

    ERR_RET_REF_FAILED      = 600,  //�زο���ʧ��
    ERR_AXIS_REF_NONE       = 601,  //��δ�زο���
    ERR_RET_REF_NOT_RUN     = 602,  //ϵͳ���ڴ���״̬����ֹ����
    ERR_NC_FILE_NOT_EXIST   = 603,  //�ļ�������
    ERR_SWITCH_MODE_IN_RET_REF = 604,   //�����н�ֹ�л�����ģʽ
    ERR_RET_REF_Z_ERR       = 605,  //����Z����λ�ó��ޣ�ԭ�㽨��ʧ��
    ERR_RET_SYNC_ERR        = 606,  //�Ӷ����ֹ����
    ERR_RET_NOT_SUPPORT     = 607,  //��֧�ֵĻ��㷽ʽ

	//�ӹ��澯	1000~1999
    ERR_EMERGENCY = 1000,               //����ֹͣ
    ERR_HARDLIMIT_POS = 1001,           //������λ�澯
    ERR_HARDLIMIT_NEG = 1002,			//�Ḻ��λ�澯
    ERR_ENCODER = 1003,                 //����������
    ERR_SERVO = 1004,					//�ŷ��澯
    ERR_SYNC_AXIS = 1005,				//ͬ������δͬ��
    ERR_SOFTLIMIT_POS = 1006,			//����������λ�澯
    ERR_SOFTLIMIT_NEG = 1007,			//�Ḻ������λ�澯
    ERR_TRACK_LIMIT = 1008,             //�����������澯
    ERR_SYNC_POS = 1009,                //ͬ����λ��ָ��ƫ�����澯
    ERR_INTP_POS = 1010,                //��λ��ָ�����澯

    ERR_AXIS_CTRL_MODE_SWITCH = 1011,   //�����ģʽ�л���ʱ
    ERR_AXIS_TORQUE_OVERTIME = 1012,    //�����ؿ��Ƴ�ʱ

    ERR_M_CODE = 1013,                  //��֧�ֵ�Mָ��
    ERR_SPD_PRESTART_M05 = 1014,        //����Ԥ�����ڼ䲻�ܵ���M05
    ERR_NO_SPD_Z_AXIS = 1015,		    //ϵͳδָ�������Z��

    ERR_SYNC_TORQUE = 1016,             //ͬ��������ƫ�����澯
    ERR_SYNC_MACH = 1017,               //ͬ�����������ƫ�����澯

    ERR_SYNC_INVALID_OPT = 1018,        //ͬ����Ƿ�����

    ERR_EXEC_FORWARD_PROG = 1100,       //ִ��ǰ�ó���ʧ��
    ERR_EXEC_BACKWARD_PROG = 1101,      //ִ�к��ó���ʧ��

    ERR_NO_CUR_RUN_DATA = 1500,         //�Ҳ�����ǰ��������

    ERR_TOOL_MEAS_POS = 1600,           //�Ե�λ��δ���ã�δʹ��
    ERR_AUTO_TOOL_MEASURE = 1601,       //�Զ��Ե�ʧ�ܣ�δʹ��

    ERR_HW_REV_OVER = 1650,             //���ַ�������޸�������   ���𣺾���
    ERR_HW_INSERT_INVALID = 1651,       //���ֲ�����Ч
    ERR_HW_INSERT_INFO = 1652,          //��ʾ��Ϣ��ʹ�����ֲ������������ϵָ�����

    ERR_TOOL_LIFE_OVER = 1660,          //������������
    ERR_TOOL_LIFE_COMING = 1661,        //����������������     ���𣺾���

    ERR_REACH_WORK_PIECE = 1662,        //�ѵ���ӹ�����   ���𣺾���

    //����澯
    ERR_SPD_TAP_START_FAIL  = 1700, //���Թ�˿ʧ��(û�н���λ��ģʽ)
    ERR_SPD_RUN_IN_TAP      = 1701, //�չ�״̬�����ٶ�ָ��
    ERR_SPD_TAP_RATIO_FAULT = 1702, //���Թ�˿�����쳣
    ERR_SPD_LOCATE_SPEED    = 1703, //��λʱ����ת���쳣
    ERR_SPD_SW_LEVEL_FAIL   = 1704, //���ỻ����ʱ
    ERR_SPD_TAP_POS_ERROR   = 1705, //�չ�λ�ô���
    ERR_SPD_RESET_IN_TAP    = 1706, //���Թ�˿�в��ܸ�λ
    ERR_SPD_RTNT_INVALID    = 1707, //���Թ�˿������Ч
    ERR_SPD_RTNT_FAIL       = 1708, //���Թ�˿����ʧ��
    ERR_SPD_RTNT_IN_AUTO    = 1709, //�Զ�ģʽ��ֹ��˿����
    ERR_SPD_MULTI_NUM       = 1710, //������澯
    ERR_SPD_LOCATE_FAIL     = 1711, //���ᶨ��ʧ��

	//PMC��澯
    ERR_PMC_AXIS_CTRL_CHANGE    = 1900, //PMC����ƷǷ��л�
    ERR_PMC_SPEED_ERROR         = 1901, //PMC���ٶ��쳣
    ERR_PMC_IVALID_USED         = 1902, //PMC��Ƿ�ʹ��

#ifdef USES_GRIND_MACHINE
	//����ĥ�߻�����Mָ��澯
	ERR_WORK_VAC_OPEN = 1950,  //��λ����ճ�ʱ
	ERR_WORK_VAC_CLOSE,      //��λ����ճ�ʱ
	ERR_WORK_CYLINDER_UP,    //��λ����������ʱ
	ERR_WORK_CYLINDER_DOWN,   //��λ�����½���ʱ
#endif


	//����澯	2000~3999
    ERR_COMPILER_INTER      = 2000, //�������ڲ�����
    ERR_NO_START            = 2001, //��%��ͷ
    ERR_NO_END              = 2002, //�޽�������
    ERR_NC_FORMAT           = 2003, //�޷�����
    ERR_MACRO_FORMAT        = 2004, //�޷������ĺ�ָ���ʽ
    ERR_MACRO_OPT           = 2005, //�޷������ĺ������
    ERR_INVALID_CODE        = 2006, //�Ƿ����룬��֧�ֵĴ���
    ERR_TOO_MANY_G_CODE     = 2007, //����NC������Gָ���������
    ERR_TOO_MANY_M_CODE     = 2008, //����NC������Mָ���������
    ERR_TOO_MANY_MACRO_EXP  = 2009, //����NC�����к���ʽ��������
    ERR_BRACKET_NOT_MATCH   = 2010, //���Ų�ƥ��
    ERR_INVALID_MACRO_EXP   = 2011, //�Ƿ��ĺ���ʽ
    ERR_MACRO_OPT_VALUE     = 2012, //�Ƿ��ĺ꺯��������
    ERR_MACRO_EXP_CAL       = 2013, //����ʽ�������
    ERR_MACRO_DIV_ZERO      = 2014, //��0����
    ERR_MACRO_DO_OUT        = 2015, //DOָ��ʶ��ų���Χ
    ERR_MACRO_DO_END_MATCH  = 2016, //DO-ENDָ�ƥ��
    ERR_NOT_IN_ONE_LINE     = 2017, //���ڲ���ͬ�е�ָ��
    ERR_ARC_NO_DATA         = 2018, //Բ��ָ��ȱ�ٲ���
    ERR_ARC_INVALID_DATA    = 2019, //Բ�����ݴ����޷���Ǣ
    ERR_NO_D_DATA           = 2020, //ȱ��Dֵָ��
    ERR_NO_H_DATA           = 2021, //ȱ��Hֵָ��
    ERR_NO_F_DATA           = 2022, //G01��G02��G03ȱ��Fֵָ��
    ERR_SUB_BACK            = 2023, //�ӳ��򷵻أ��ָ��ֳ��쳣
    ERR_MAIN_OVER           = 2024, //����������쳣���ӳ�����ö�ջ�ǿ�
    ERR_SUB_END             = 2025, //������ӳ������ָ��ӳ��������M99����
    ERR_MDA_OVER            = 2026, //MDA��������󣬵��ö�ջΪ��
    ERR_NO_SUB_PROG_P       = 2027, //��P�ֶ�ָ���ӳ����
    ERR_MCODE_SEPARATED     = 2028, //���ڲ���ͬ�е�Mָ��     ע������Mָ���ҪNC�ڲ������Mָ�����������Mָ��ͬ��
    ERR_PRE_SCAN            = 2029, //Ԥɨ��ִ��ʧ��
    ERR_QUIT_PRESCAN        = 2030, //ȡ��Ԥɨ��ʧ��
    ERR_SAME_SUB_INDEX      = 2031, //�ӳ���ͬ��
    ERR_SAME_SEQ_NO         = 2032, //�ظ���˳���
    ERR_NO_SUB_PROG         = 2033, //�Ҳ���ָ�����ӳ���
    ERR_JUMP_SUB_PROG       = 2034, //��ת�ӳ���ʧ��
    ERR_SUB_NESTED_COUNT    = 2035, //�ӳ���Ƕ�ײ�������
    ERR_NO_JUMP_LABEL       = 2036, //û���ҵ���ת��λ��
    ERR_JUMP_GOTO           = 2037, //ִ��GOTOָ���תʧ��
    ERR_JUMP_END            = 2038, //ENDָ��ִ��ʧ��
    ERR_LACK_OF_PARAM       = 2039, //ָ��ȱ�ٲ���
    ERR_JUMP_GOTO_ILLIGAL   = 2040,	//�Ƿ���ת
    ERR_NO_P_DATA           = 2041, //ȱ��Pֵָ��
    ERR_INVALID_L_DATA      = 2042, //�Ƿ�Lֵָ��
    ERR_TOO_MANY_T_CODE     = 2043, //����NC������Tָ���������
    ERR_TOO_LONG_WORD       = 2044, //��������޷�����
    ERR_TOO_LONG_DIGIT      = 2045, //������ֵ���޷�����
    ERR_G_EXP_NULL          = 2046, //Gָ����ʽΪ��ֵ
    ERR_M_EXP_NULL          = 2047, //Mָ����ʽΪ��ֵ
    ERR_T_EXP_NULL          = 2048, //Tָ����ʽΪ��ֵ
    IF_ELSE_MATCH_FAILED    = 2049, // if else ƥ��ʧ�� ջ��û��if ȴ������ else
    COMP_LIST_NOT_INIT      = 2050, // ����ģ�� �����Ϣ����ָ��δ��ʼ��
    ARC_RADIUS_TOO_SMALL    = 2051, // Բ���뾶��С �޷����㵶��
    ARC_NOT_VALID           = 2052, // Բ�����ݴ���
    TOOL_RADIUS_BIGGER_THAN_ARC     = 2053, // �����뾶����Բ���뾶
    ARC_NOT_ALLOWED         = 2054, // �رյ������һ���ƶ�������Բ��
    TOOL_RADIUS_COMP_BUG    = 2055,
    CONCAVE_CORNER_ERROR    = 2056,
    ARC_TO_ARC_SAME_CENTER  = 2057, //Բ����Բ���������ܼ���ͬ��Բ��
    MOVE_SMALLER_THAN_CMPRADIUS     = 2058, //�ƶ�����С�ڵ����뾶 �޷����㵶��
    ERR_CAL_SPD_TAP_POS     = 2059,   // �޷�������Թ�˿ʱ������λ��
    F_VALUE_ERROR           = 2060,   // F����ָ��Ϊ����


	//�����澯	5000~6999
    ERR_CANCEL_R_COMP_WITHOUT_MOVE      = 5000, //ȡ�����߰뾶����ʱû�����ƶ�
    ERR_NO_MOVE_DATA_EXCEED             = 5001, //���߰뾶�����Ǽӹ����ݳ���
    ERR_C_L_COMP_AXIS_EXCEED            = 5002, //C�൶�߳��Ȳ������������,�������ָ��������г��Ȳ���
    ERR_TOOL_RADIUS_EXCEED              = 5003, //���߰뾶����
    ERR_CHANGE_R_COMP_SURFACE           = 5004, //���߰뾶���������в��ɸı�ӹ�ƽ��
    ERR_CHANGE_R_COMP_STAT_CMD          = 5005, //������ȡ�����߰뾶����ָ�����
    ERR_END_WITHOUT_CANCEL_L_COMP       = 5006, //�������ʱû��ȡ�����߳��Ȳ���
    ERR_R_COMP_UNDER_MACHINE_COORD      = 5007, //�ڻ�������ϵ�²���ʹ�õ��߰뾶����
    ERR_WRONG_MOV_CMD_FOR_L_COMP_CHANGE = 5008, //���߳��Ȳ����仯ʱָ���岹ָ�����
    ERR_R_COMP_NOT_SUPPORTED_FOR_CURVE_TYPE = 5009,	//��֧��ָ���������͵ĵ��߰뾶����
    ERR_R_COMP_CHANGE_AFTER_ESTABLISH   = 5010, //���߰뾶���������ڽ����������ı�
    ERR_R_COMP_UNDER_FIXED_CYCLE        = 5011, //�̶�ѭ���в���ʹ�õ��߰뾶����
    ERR_R_COMP_GENERATE_FAIL            = 5012, //�����켣����ʧ��
	INVALID_COMPENSATE_PLANE 			= 5013,  // G18/G19ƽ���ݲ�֧�ֵ���
	ARC_NOT_ALLOWED2		 			= 5014,  // ����������һ�β�֧��Բ���켣
	G31_NOT_ALLOWED			 			= 5015,  // ���������в�֧��G31����
	DWORD_INVALID			 			= 5016,  // �������޷�ִ��Dָ��
	G41_G42_CHANGE			 			= 5017,  // ��֧�� G41_G42 ��;�л�


	//ͨѶ�澯	7000~7999
    ERR_HEARTBEAT_HMI_LOST  = 7000, //HMI������ʧ
    ERR_HEARTBEAT_MC_LOST   = 7001, //MCģ��������ʧ
    ERR_HEARTBEAT_MI_LOST   = 7002, //MIģ��������ʧ
    ERR_HMI_MONITOR_DISCON  = 7003, //HMI���tcp�����ж�
    ERR_FILE_TRANS = 7004,  //�ļ�����ʧ��
    ERR_MC_CMD_CRC = 7005,  //MC�����CRCУ���
    ERR_MI_CMD_CRC = 7006,  //MI�����CRCУ���


	//MCģ��澯 10000~19999
    ERR_SOFT_LIMIT_NEG = 10000, //��������λ�澯
    ERR_SOFT_LIMIT_POS = 10001, //��������λ�澯
    ERR_POS_ERR     = 10002,    //λ��ָ�����澯
    ERR_ARC_DATA    = 10003,    //Բ�����ݴ���澯
    ERR_CMD_CRC     = 10004,    //ָ��У���
    ERR_DATA_CRC    = 10005,    //����У���


	//PMC�澯	20000~29999



	//���߸澯	30000~39999



	//�û��Զ���澯   40000~49999

};


//Gָ��ö�ٶ���
enum GCode{
	G00_CMD = 0,
	G01_CMD = 10,
	G02_CMD = 20,
	G03_CMD = 30,
	G04_CMD = 40,
	G05_1_CMD = 51,
	G06_2_CMD = 62,
	G09_CMD = 90,
	G10_CMD = 100,
	G11_CMD = 110,
	G12_1_CMD = 121,
	G12_2_CMD = 122,
	G12_3_CMD = 123,
	G12_4_CMD = 124,
	G13_1_CMD = 131,
	G15_CMD = 150,
	G16_CMD = 160,
	G17_CMD = 170,
	G18_CMD = 180,
	G19_CMD = 190,
	G20_CMD = 200,
	G21_CMD = 210,
	G25_CMD = 250,
	G26_CMD = 260,
	G27_CMD = 270,
	G28_CMD = 280,
	G29_CMD = 290,
	G30_CMD = 300,
	G31_CMD = 310,
	G37_CMD = 370,         //�Զ��Ե�ָ��
	G40_CMD = 400,
	G41_CMD = 410,
	G42_CMD = 420,
	G43_CMD = 430,
	G43_4_CMD = 434,
	G44_CMD = 440,
	G49_CMD = 490,
	G50_CMD = 500,
	G51_CMD = 510,
	G50_1_CMD = 501,
	G51_1_CMD = 511,
	G52_CMD = 520,
	G53_CMD = 530,
	G54_CMD = 540,
	G55_CMD = 550,
	G56_CMD = 560,
	G57_CMD = 570,
	G58_CMD = 580,
	G59_CMD = 590,
	G60_CMD = 600,
	G61_CMD = 610,
	G62_CMD = 620,
	G63_CMD = 630,
	G64_CMD = 640,
	G65_CMD = 650,
	G68_CMD = 680,
	G69_CMD = 690,
	G73_CMD = 730,
	G74_CMD = 740,
	G76_CMD = 760,
	G80_CMD = 800,
	G81_CMD = 810,
	G82_CMD = 820,
	G83_CMD = 830,
	G84_CMD = 840,
	G84_3_CMD = 843,       //���͸չ�����ֵ
	G85_CMD = 850,
	G86_CMD = 860,
	G87_CMD = 870,
	G88_CMD = 880,
	G89_CMD = 890,
	G90_CMD = 900,
	G91_CMD = 910,
	G92_CMD = 920,
	G94_CMD = 940,
	G95_CMD = 950,
	G98_CMD = 980,
	G99_CMD = 990,
	G112_CMD = 1120,
	G113_CMD = 1130,
	G120_CMD = 1200,     //G120  �澯����ʾ��Ϣ����
	G200_CMD = 2000,     //G200������Ȧ����

	G1000_CMD = 10000,   //�ٶȿ���ָ����ȴ��ٶȵ���ٶȵ�λ��mm/min
	G1001_CMD = 10010,   //�ٶȿ���ָ����ȴ��ٶȵ���ٶȵ�λ��rpm
	G1002_CMD = 10020,   //�ٶȿ���ָ��ȴ��ٶȵ���ٶȵ�λ��mm/min
	G1003_CMD = 10030,   //�ٶȿ���ָ��ȴ��ٶȵ���ٶȵ�λ��rpm

	G2000_CMD = 20000,    // ����ָ����õȴ���������
	G2001_CMD = 20010,    // ����ָ���������Ϊת�ٵ���0
	G2002_CMD = 20020,    // ����ָ���������Ϊ���ص���
	G2003_CMD = 20030,    // ����ָ���������Ϊ���ص����ʱҲ�������������澯

	G5401_CMD = 54010,  //��չ��������ϵ
	G5402_CMD = 54020,
	G5403_CMD = 54030,
	G5404_CMD = 54040,
	G5405_CMD = 54050,
	G5406_CMD = 54060,
	G5407_CMD = 54070,
	G5408_CMD = 54080,
	G5409_CMD = 54090,
	G5410_CMD = 54100,
	G5411_CMD = 54110,
	G5412_CMD = 54120,
	G5413_CMD = 54130,
	G5414_CMD = 54140,
	G5415_CMD = 54150,
	G5416_CMD = 54160,
	G5417_CMD = 54170,
	G5418_CMD = 54180,
	G5419_CMD = 54190,
	G5420_CMD = 54200,
	G5421_CMD = 54210,
	G5422_CMD = 54220,
	G5423_CMD = 54230,
	G5424_CMD = 54240,
	G5425_CMD = 54250,
	G5426_CMD = 54260,
	G5427_CMD = 54270,
	G5428_CMD = 54280,
	G5429_CMD = 54290,
	G5430_CMD = 54300,
	G5431_CMD = 54310,
	G5432_CMD = 54320,
	G5433_CMD = 54330,
	G5434_CMD = 54340,
	G5435_CMD = 54350,
	G5436_CMD = 54360,
	G5437_CMD = 54370,
	G5438_CMD = 54380,
	G5439_CMD = 54390,
	G5440_CMD = 54400,
	G5441_CMD = 54410,
	G5442_CMD = 54420,
	G5443_CMD = 54430,
	G5444_CMD = 54440,
	G5445_CMD = 54450,
	G5446_CMD = 54460,
	G5447_CMD = 54470,
	G5448_CMD = 54480,
	G5449_CMD = 54490,
	G5450_CMD = 54500,
	G5451_CMD = 54510,
	G5452_CMD = 54520,
	G5453_CMD = 54530,
	G5454_CMD = 54540,
	G5455_CMD = 54550,
	G5456_CMD = 54560,
	G5457_CMD = 54570,
	G5458_CMD = 54580,
	G5459_CMD = 54590,
	G5460_CMD = 54600,
	G5461_CMD = 54610,
	G5462_CMD = 54620,
	G5463_CMD = 54630,
	G5464_CMD = 54640,
	G5465_CMD = 54650,
	G5466_CMD = 54660,
	G5467_CMD = 54670,
	G5468_CMD = 54680,
	G5469_CMD = 54690,
	G5470_CMD = 54700,
	G5471_CMD = 54710,
	G5472_CMD = 54720,
	G5473_CMD = 54730,
	G5474_CMD = 54740,
	G5475_CMD = 54750,
	G5476_CMD = 54760,
	G5477_CMD = 54770,
	G5478_CMD = 54780,
	G5479_CMD = 54790,
	G5480_CMD = 54800,
	G5481_CMD = 54810,
	G5482_CMD = 54820,
	G5483_CMD = 54830,
	G5484_CMD = 54840,
	G5485_CMD = 54850,
	G5486_CMD = 54860,
	G5487_CMD = 54870,
	G5488_CMD = 54880,
	G5489_CMD = 54890,
	G5490_CMD = 54900,
	G5491_CMD = 54910,
	G5492_CMD = 54920,
	G5493_CMD = 54930,
	G5494_CMD = 54940,
	G5495_CMD = 54950,
	G5496_CMD = 54960,
	G5497_CMD = 54970,
	G5498_CMD = 54980,
	G5499_CMD = 54990
};

/**
 * @brief �����ļ�����ʱ���ļ�����
 */
enum FileTransType{
	FILE_NO_TYPE = 0x00,        //�Ƿ�����
	FILE_G_CODE = 0x01,         //�ӹ��ļ�
	FILE_CONFIG_PACK = 0x02,	//���ô���ļ�
//	FILE_SYSTEM_CONFIG,         //ϵͳ�����ļ�
//	FILE_CHANNEL_CONFIG,        //ͨ�������ļ�
//	FILE_AXIS_CONFIG,           //�������ļ�
//	FILE_TOOL_CONFIG,          //���������ļ�
	FILE_UPDATE = 0x06,		    //�����ļ�
	FILE_PMC_LADDER,			//PMC����ͼ�ļ�
	FILE_WARN_HISTORY,          //�澯��ʷ�ļ�
	FILE_ESB_DEV = 0x10,         //ESB�豸�����ļ�
	FILE_PC_DATA,                //�ݲ������ļ�
	FILE_BACK_DISK,				//һ�������ļ�
    FILE_BACKUP_CNC,            //ȫ�̱���
};

/**
 * @brief ģ����������
 */
enum ModuleUpdateType{
	MODULE_UPDATE_NONE = 0,			//������״̬
	MODULE_UPDATE_SC,				//SCģ������
	MODULE_UPDATE_MC,				//MCģ������
	MODULE_UPDATE_PMC,				//PMC����ͼ����
	MODULE_UPDATE_MI,				//MIģ������
	MODULE_UPDATE_PL,				//PLģ������
	MODULE_UPDATE_FPGA,				//FPGAģ������
	MODULE_UPDATE_MODBUS,            //SCModbusģ������
	MODULE_UPDATE_DISK,             //һ������
};

/**
 * @brief �ļ���������
 */
enum FileOptType{
	FILE_OPT_INVALID = 0X00,   //�Ƿ�����
	FILE_OPT_DELETE,           //ɾ������
	FILE_OPT_RENAME,           //����������
	FILE_OPT_SAVEAS            //���Ϊ����
};

/**
 * @brief ��������ļ�Mask����
 */
enum ConfigMaskBits{
	CONFIG_SYSTEM = 0,	    //ϵͳ����
	CONFIG_CHANNEL = 1,	    //ͨ������
	CONFIG_AXIS = 2,		//�����
	CONFIG_COORD = 3,	    //��������ϵ
	CONFIG_EX_COORD = 4,	//��չ��������ϵ
	CONFIG_TOOL_INFO = 5,	//������Ϣ����������ƫ�ú͵�λ��Ϣ
	CONFIG_FIVE_AXIS = 6,	//��������
	CONFIG_GRIND = 7,		//ĥ�߻�����
	CONFIG_MACRO_VAR = 8,	//�����
	CONFIG_PMC_REG = 9,		//PMC�Ĵ���
	CONFIG_PMC_LDR = 10,	//PMC��ͼ
	CONFIG_IO_REMAP = 11,   //IO��ӳ������
	CONFIG_ESB_FILE = 12,   //�ŷ�����esb�ļ�

	CONFIG_TYPE_COUNT      //����ļ���������
};


/**
 * @brief ErrorInfo���ݽṹΪ������ģ�������/������ݸ�ʽ
 */
//#pragma pack(1)  //���ֽڶ���
struct ErrorInfo {
    uint64_t time; 	    //�������ʱ�䣬�ɸߵ��ͷֱ�Ϊ���꣨2�ֽڣ��£�1�ֽڣ��գ�1�ֽڣ�ʱ��1�ֽڣ��֣�1�ֽڣ��루1�ֽڣ�������ֽڱ���
    int32_t error_info;     //������Ϣ
    uint16_t error_code;    //������
    uint8_t channel_index;  //ͨ����
    uint8_t axis_index;     //ͨ���������
    uint8_t error_level;    //���󼶱�
    uint8_t clear_type;	    //����澯���

    bool operator==(const ErrorInfo &info) const {
        if(info.axis_index == axis_index && info.channel_index == channel_index &&
            info.clear_type == clear_type && info.error_code == error_code &&
            info.error_level == error_level && info.error_info == error_info){
            return true;
        }
        else
            return false;
    }
};
//#pragma pack()

//���󼶱�
enum ErrorLevel {
    FATAL_LEVEL = 0, 	//���ش���
    ERROR_LEVEL,        //����
    WARNING_LEVEL,      //����
    INFO_LEVEL,	        //��ʾ��Ϣ
    MAX_LEVEL,
};

/*
 * @brief �澯������
 */
enum ErrorClearType {
    CLEAR_BY_RESET_POWER = 0, 	//������Դ�����
    CLEAR_BY_MCP_RESET,         //��λ���
    CLEAR_BY_AUTO,              //�Զ����
};

//32bit��mask
struct Mask32 {
	uint32_t mask;
	void Init() {
		mask = 0;
	}
	void InitF() {
		mask = (uint32_t) -1;
	}
	bool CheckMask(int i) const {
		if(mask & ((uint32_t)0x01 << i))
			return true;

		return false;
	}
	void SetMask(int i) {	//��־λ��λ
		mask |= (uint32_t)0x01 << i;
	}
	void ResetMask(int i){  //��־λ����
		mask &= ~((uint32_t)0x01 << i);
	}
	void SetMask(int i, int value) {
		if (value == 0) {
			mask &= ~((uint32_t)0x01 << i);

		} else {
			SetMask(i);
		}
	}
	bool IsZero() {
		return (mask) ? false : true;
	}
	bool IsAllF() {
		return (mask == (uint32_t) -1 ) ?
				true : false;
	}
};

/*
 * @brief HMIʹ�õ�ͨ��ʵʱ״̬
 */
struct HmiChannelRealtimeStatus {
	double cur_pos_machine[kMaxAxisChn];	//��ǰʵ��λ�ã���е����ϵ����λ��mm
	double cur_pos_work[kMaxAxisChn];		//��ǰʵ��λ�ã���������ϵ����λ��mm
	double tar_pos_work[kMaxAxisChn];		//��ǰĿ��λ�ã���������ϵ����λ��mm
	int32_t spindle_cur_speed; 		        //���ᵱǰת�٣���λ��ת/����
    int32_t cur_feed; 						//��ǰ�����ٶȣ���λ��um/s
	uint32_t machining_time; 				//�ӹ�ʱ�䣬��λ����
	uint32_t machining_time_remains; 		//ʣ��ӹ�ʱ�䣬��λ����
    uint32_t machining_time_total;          //�ۼƼӹ�ʱ�䣬��λ����
	uint32_t line_no;						//��ǰ�к�

    double cur_feedbck_velocity[kMaxAxisChn];			//��ǰ����ʵ���ٶȣ���λ��mm/min
    double cur_feedbck_torque[kMaxAxisChn];			//��ǰ����ʵ�����أ���λ��0.001�����

    int32_t tap_err;                        //���Թ�˿���(���ֵ) ��λ��um
    int32_t tap_err_now;                    //���Թ�˿���(��ǰֵ) ��λ��um
    double spd_angle;                      //���ᵱǰ�Ƕ� ��λ����
};


/*
 * @brief HMIʹ�õ�ͨ��״̬
 */
struct HmiChannelStatus{
	double rated_manual_speed; 		//�ֶ��ٶȣ���λ��mm/s
	uint32_t workpiece_count;       //�������������99999999
    uint32_t workpiece_require;     //�������
    uint32_t workpiece_count_total; //�ܹ�����
    uint32_t machinetime_total;         //�ۼƼӹ�ʱ��
    Mask32 func_state_flags;        //ϵͳ����״̬��־���������Ρ����Ρ�ѡͣ�������С����ָ��١���������������
	int32_t rated_spindle_speed;    //�û��趨����ת�٣���λ��ת/����
	uint32_t rated_feed; 					//�û��趨�����ٶȣ���λ��um/s
	uint16_t g_code_mode[kMaxGModeCount];	//Gָ��ģ̬��Ϣ
	uint8_t chn_work_mode;			//ͨ������ģʽ, �Զ���MDA���ֶ�������
	uint8_t machining_state;       	//�ӹ�״̬
	uint8_t cur_tool;               //��ǰ���ţ���1��ʼ��ţ�0�ŵ�Ϊ���ᵶ��
	uint8_t preselect_tool_no;		//Ԥѡ������
	uint8_t cur_h_code;				//��ǰH������
	uint8_t cur_d_code;				//��ǰD������
	uint8_t auto_ratio;            	//�Զ���������
	uint8_t spindle_ratio;         	//���ᱶ��
	uint8_t rapid_ratio;			//���ٽ�������
	uint8_t manual_ratio;          	//�ֶ�����
	uint8_t manual_step;           	//�ֶ�������0:INVALID;1:X1;2:X10;3:X100;4:X1000
	int8_t cur_manual_move_dir;		//��ǰ�ֶ��ƶ�����0--ֹͣ   1--����   -1--����
	uint8_t cur_axis;				//��ǰ���,ͨ�����
	uint8_t spindle_mode;           //���Ṥ����ʽ��0���ٶȿ��Ʒ�ʽ��1��λ�ÿ��Ʒ�ʽ
	int8_t spindle_dir;				//���᷽��-1����ת��0��ͣת��1����ת
	uint8_t returned_to_ref_point;	//�Ƿ��ѻزο��㣬bit0-bit7�ֱ��ʾ��0-7�Ƿ��ѻزο���
	uint8_t es_button_on;			//��ͣ�����Ƿ��ڰ���״̬��ÿһ��bit��ʾһ����ͣ����
	char cur_nc_file_name[kMaxFileNameLen];	//��ǰ�ӹ��������ļ���
	uint8_t cur_chn_axis_phy[kMaxAxisChn];	    //��ǰͨ�������Ӧ������������
	int mcode; // @add zk ��ʱû��
};

/**
 * @brief ����ͨ��״̬����
 */
enum ChannelStatusType{
	FUNC_STATE = 0,			//ϵͳ����״̬��־���������Ρ����Ρ�ѡͣ�������С����ָ��١���������������
	AUTO_RATIO,				//�Զ�����
	SPINDLE_RATIO,			//���ᱶ��
	RAPID_RATIO,			//���ٽ�������
	MANUAL_RATIO,			//�ֶ�����
	G_MODE,					//G����ģʽ
	MANUAL_STEP,			//�ֶ�����
	REF_POINT_FLAG,			//�زο����־
	CUR_AXIS,				//ͨ����ǰ��

	CHN_STATUS_GUARD		//����

};

/*
 * @brief PLC����
 */
struct PmcData {
	uint8_t address;	//�ε�ַ
	uint16_t index;		//���ڵ�ַ��
	uint16_t data;		//ֵ
};

/**
 * @brief CPU���������Ϣ
 */
struct CPURunningInfo{
	double board_temperature;	//�����¶�
	uint8_t arm1_cpu_load;		//ARM1 CPUռ����
	uint8_t arm1_max_cpu_load;	//ARM1 CPUռ���ʣ�ϵͳ���������ֵ��
	uint8_t arm1_mem_load;		//ARM1 �ڴ�ռ����
	uint8_t arm1_max_mem_load;	//ARM1 �ڴ�ռ���ʣ�ϵͳ���������ֵ��
	uint8_t dsp_cpu_load;		//DSP CPUռ����
	uint8_t dsp_max_cpu_load;	//DSP CPUռ���ʣ�ϵͳ���������ֵ��

	uint8_t arm2_cpu_load;		//ARM2 CPUռ����
	uint8_t arm2_max_cpu_load;	//ARM2 CPUռ���ʣ�ϵͳ���������ֵ��
	uint8_t arm2_mem_load;		//ARM2 �ڴ�ռ����
	uint8_t arm2_max_mem_load;	//ARM2 �ڴ�ռ���ʣ�ϵͳ���������ֵ��

	uint16_t plc_fst_stage_run_time;		//PLCһ������ʱ��
	uint16_t plc_max_fst_stage_run_time;	//PLCһ������ʱ�䣨ϵͳ���������ֵ��
	uint16_t plc_snd_stage_run_time;		//PLC��������ʱ��
	uint16_t plc_max_snd_stage_run_time;	//PLC��������ʱ�䣨ϵͳ���������ֵ��
	uint16_t plc_scan_interval;	//PLCɨ������
};

//�汾��Ϣ����
struct SwVerInfoCollect{
	char platform[kMaxVersionLen];   	//ƽ̨�汾
	char sc[kMaxVersionLen];			//SCģ��
	char mc[kMaxVersionLen];			//MCģ��
	char mi[kMaxVersionLen];			//MIģ��
	char mop[kMaxVersionLen];			//�������
	char keyboard[kMaxVersionLen];		//����
	char fpga_pl[kMaxVersionLen];		//ZYNQ_PL
	char fpga_spartan[kMaxVersionLen];	//SPARTAN
};


//ϵͳ��Ϣ�ṹ��
struct SystemInfo{
	SwVerInfoCollect sw_version_info;    //�汾��Ϣ
	CPURunningInfo cpu_info;		//CPU������Ϣ
};

/**
 * @brief ������λ�ýṹ
 */
struct CoordAxisPos{
	double x;		//X��
	double y;		//Y��
	double z;		//Z��
};

/**
 * @brief PMC��λ�ýṹ
 */
struct PmcAxisPos{
	int axis_no;		//�������
	double mach_pos;	//��е���꣬��λ��mm
	double work_pos;   //�������꣬��λ: mm
	double remain_dis;	//ʣ���ƶ�������λ��mm
};


/**
 * @brief HMIģ��ϵͳ��������
 */
struct HmiSystemConfig{
	uint8_t cnc_mode;			        //�豸����		//����Ȩ������������޸�
	uint8_t max_chn_count;				//���ͨ����	//����Ȩ������������޸�
	uint8_t max_axis_count;				//�������		//����Ȩ������������޸�
	uint8_t chn_count;					//ͨ������
	uint8_t axis_count;			     	//ϵͳ����������
	uint8_t axis_name_ex;				//�Ƿ�������������չ�±�    0--�ر�		1--��
	uint8_t fix_ratio_find_ref;			//�زο���ʱ���ʹ̶�	0--�ر�		1--��
//	uint8_t pc_type;					//�ݾಹ����ʽ		0--�����ݲ�  1--˫���ݲ�
//	uint8_t mcp_count;                  //MCP����

	uint8_t hw_code_type;               //���ֱ��뷽ʽ     0--�����Ʊ���    1--������

	uint32_t io_filter_time;			//��ͨIO�˲�ʱ��	//��λ��΢��
	uint32_t fast_io_filter_time;		//����IO�˲�ʱ��	//��λ��΢��
	uint32_t free_space_limit;			//ʣ��ռ�����      //��λ��KB
	uint16_t backlight_delay_time;		//������ʱʱ��		//��λ����

	uint8_t bus_cycle;					//����ͨѶ����		0-8:125us/250us/500us/1ms/2ms/4ms/5ms/8ms/10ms

	uint8_t save_lineno_poweroff;		//���籣�浱ǰ�����к�		0--�ر�		1--��
	uint8_t manual_ret_ref_mode;		//�ֶ��زο���ģʽ		0-�״� 		1--ÿ��

	uint8_t beep_time;					//����ʱ��			//��λ����

	uint8_t da_ocp;						//DA��������	0--�ر�		1--��
	uint8_t da_prec;				    //DA�������ȣ�λ����  10-64

	uint8_t alarm_temperature;			//����澯�¶�   ��λ�����϶�
	uint8_t trace_level;				//���Ը�����Ϣ��¼����
	uint8_t debug_mode;					//����ģʽ		0--�ر�    1--ģʽ1    2--ģʽ2    3--ģʽ3

	uint8_t hw_rev_trace;               //���ַ�����������   0--�ر�   1--��

    /**********************λ�ÿ���***************************/
    uint8_t pos_check_id_1;             //λ�ÿ���1
    uint8_t pos_check_id_2;             //λ�ÿ���2
    uint8_t pos_check_id_3;             //λ�ÿ���3
    uint8_t pos_check_id_4;             //λ�ÿ���4
    uint8_t pos_check_id_5;             //λ�ÿ���5
    uint8_t pos_check_id_6;             //λ�ÿ���6
    uint8_t pos_check_id_7;             //λ�ÿ���7
    uint8_t pos_check_id_8;             //λ�ÿ���8

    double pos_check_min_1;             //λ�ÿ�����Сֵ1
    double pos_check_min_2;             //λ�ÿ�����Сֵ2
    double pos_check_min_3;             //λ�ÿ�����Сֵ3
    double pos_check_min_4;             //λ�ÿ�����Сֵ4
    double pos_check_min_5;             //λ�ÿ�����Сֵ5
    double pos_check_min_6;             //λ�ÿ�����Сֵ6
    double pos_check_min_7;             //λ�ÿ�����Сֵ7
    double pos_check_min_8;             //λ�ÿ�����Сֵ8

    double pos_check_max_1;             //λ�ÿ������ֵ1
    double pos_check_max_2;             //λ�ÿ������ֵ2
    double pos_check_max_3;             //λ�ÿ������ֵ3
    double pos_check_max_4;             //λ�ÿ������ֵ4
    double pos_check_max_5;             //λ�ÿ������ֵ5
    double pos_check_max_6;             //λ�ÿ������ֵ6
    double pos_check_max_7;             //λ�ÿ������ֵ7
    double pos_check_max_8;             //λ�ÿ������ֵ8
    /**********************λ�ÿ���***************************/
};

/**
 * @brief HMIģ��ͨ����������
 */
struct HmiChnConfig{
	uint8_t chn_index;  		//ͨ��������
	uint8_t chn_axis_count;     //ͨ��������
	uint8_t chn_group_index;    //ͨ��������ʽ����
//	uint8_t chn_axis_x;			//������X��Ӧ������������
//	uint8_t chn_axis_y;			//������Y��Ӧ������������
//	uint8_t chn_axis_z;			//������Z��Ӧ������������
	uint8_t chn_axis_phy[kMaxAxisChn];	//�������Ӧ������������

	uint8_t chn_axis_name[kMaxAxisChn];	//������
	uint8_t chn_axis_name_ex[kMaxAxisChn];	//�������±�

	uint8_t intep_mode;				//�岹ģʽ   0��XYZ����岹  1��������岹
	uint8_t intep_cycle;			//�岹����	0-4:125us/250us/400us/500us/1000us
	uint16_t chn_precision;			//�ӹ����ȣ���λ��΢��
	uint8_t chn_look_ahead;			//ǰհ���� 	0--�ر�   1--��
	uint8_t chn_feed_limit_level;	//�ӹ��ٶȵ����ȼ�  0--�ر�   1~10��ʾ�ȼ�1~�ȼ�10
	uint8_t zmove_stop;				//�µ�׼ͣ  0--�ر�    1--��
	uint8_t corner_stop_enable;		//�ս�׼ͣ���ܿ���
	uint8_t corner_stop;			//�ս�׼ͣ�Ƕ�  0-180��
	uint8_t corner_stop_angle_min;  //�ս�׼ͣ���޽Ƕ� 0-180��
	uint8_t corner_acc_limit;		//�սǼ��ٶ�����	0--�ر�   1--��
	uint8_t long_line_acc;			//��ֱ�����м���    0--�ر�   1--��

	uint8_t arc_err_limit;			//Բ���������      0-255  ��λ��um

	uint8_t chn_spd_limit_on_axis;  	//�������ٶȲ��ת���ٶ�ǯ��		0--�ر�   1--��
	uint8_t chn_spd_limit_on_acc;		//���ڼ��ٶȵ�С�߶ν����ٶ�ǯ��		0--�ر�   1--��
	uint8_t chn_spd_limit_on_curvity;	//�������ʼ����ļ��ٵ�С�߶ν����ٶ�ǯ��		0--�ر�   1--��
	uint8_t chn_rapid_overlap_level;	//G00�ص��ȼ�	0--�ر�  1~3--�ȼ�1~�ȼ�3

	double chn_max_vel;					//ͨ����������ٶ�	//��λ��mm/min
	double chn_max_acc;					//ͨ�������ٶ�	//��λ��mm/s^2
	double chn_max_dec;					//ͨ�������ٶ�	//��λ��mm/s^2
	double chn_max_corner_acc;          //���սǼ��ٶ�		//��λ��mm/s^2
	double chn_max_arc_acc;				//������ļ��ٶ�	//��λ��mm/s^2
	uint16_t chn_s_cut_filter_time;		//��������S�͹滮ʱ�䳣��  //��λ��ms

    double tap_max_acc;                 //���Թ�˿�����ٶ�	//��λ��mm/s^2
    double tap_max_dec;                 //���Թ�˿�����ٶ�	//��λ��mm/s^2
    int8_t tap_plan_mode;              //���Թ�˿�滮ģʽ  0��T��  1��S��
    uint16_t tap_s_cut_filter_time;     //���Թ�˿S�͹滮ʱ�䳣��  //��λ��ms

    uint8_t default_plane;			//Ĭ��ƽ��	   0��G17  1��G18  2��G19
    uint8_t default_cmd_mode;		//Ĭ��ָ��ģʽ   0��G90  1��G91
    uint8_t default_feed_mode;      //Ĭ�Ͻ�����ʽ   0��G00  1��G01  2��G02  3��G03

	uint8_t rapid_mode;				//G00ģʽ    0����λ     1��ֱ�߶�λ
	uint8_t cut_plan_mode;			//�����滮ģʽ	0��T��   1��S��
	uint8_t rapid_plan_mode;		//��λ�滮ģʽ  0��T��   1��S��
	uint8_t ex_coord_count;			//��չ��������ϵ����  0-99

	uint8_t change_tool_mode;		//������ʽ     0--�ֶ�    1--�Զ�
	uint8_t tool_live_check;		//�����������  0--�ر�   1--��
	uint8_t auto_tool_measure;		//�Զ��Ե�		0--�ر�   1--��

	uint8_t gcode_trace;			//�ӹ��������   0--�ر�   1--��
	uint8_t gcode_unit;				//G�����̵�λ		0--����    1--Ӣ��

	uint8_t timing_mode;			//�ӹ���ʱ��ʽ    0--ȫ����ʱ    1--������ʱ

	uint16_t chn_small_line_time;   //С�߶�ִ��ʱ�䳣��   ��λ��0.1ms   ��Χ[0, 10000]

	uint32_t g31_skip_signal;       //G31��ת�źţ�����X1004.7����Ϊ10047������10��
	uint8_t g31_sig_level;          //G31��ת�ź���Ч��ƽ    0--�͵�ƽ    1--�ߵ�ƽ
    uint16_t rst_hold_time;         //��λʱ�� ��λ:ms
    uint8_t rst_mode;               //��λʱ���������Ƿ���  0:������ 1������
    uint32_t g01_max_speed;         //G01��߽����ٶ� ��λ��mm/min
    uint16_t mpg_level3_step;       //����3�����Զ��岽�� ��λ��um
    uint16_t mpg_level4_step;       //����4�����Զ��岽�� ��λ��um

    double G73back;		// G73���˾���
    double G83back;		// G83���˾���

#ifdef USES_WOOD_MACHINE
	int debug_param_1;             //���Բ���1
	int debug_param_2;             //���Բ���2
	int debug_param_3;             //���Բ���3
	int debug_param_4;             //���Բ���4
	int debug_param_5;             //���Բ���5
	
	int flip_comp_value;           //���ǲ���    -10000~10000    ��λ��um
#endif
};


/**
 * @brief HMIģ������������
 */
struct HmiAxisConfig{
    uint8_t axis_index;						//��������	0-11��ϵͳ�Զ�����
    uint8_t axis_type;						//������,    0--ֱ����     1--��ת��     2--����/�ٶ���    3--������
    uint8_t axis_interface;					//��ӿ�����	0--������    1-������    2--��������
    uint8_t axis_port;						//��վ�ţ������ᣩ/��Ӧ��ںţ��������ᣩ1~12
    uint8_t axis_linear_type;				//ֱ��������
    uint8_t axis_pmc;						//�Ƿ�PMC��


    double kp1;								//�Զ���������
    double kp2;								//�ֶ���������
    double ki;								//���ֲ���
    double kd;								//΢�ֲ���
    double kil;								//���ֱ�����
    uint16_t kvff;							//�ٶ�ǰ��ϵ��
    uint16_t kaff;							//���ٶ�ǰ��ϵ��

    uint32_t track_err_limit;				//����������� ��λ��um
    uint16_t location_err_limit;			//��λ������� ��λ��um

//	uint8_t soft_limit_check;				//����λ���	0--�ر�   1--��
    uint8_t save_pos_poweroff;				//�����������		0--�ر�   1--��
//	uint8_t axis_alarm_level;               //��澯�źŵ�ƽ    0--�͵�ƽ  1--�ߵ�ƽ

	uint32_t motor_count_pr;				//���ÿת����
	uint32_t motor_speed_max;				//������ת��   ��λ��rpm
	double move_pr;						    //ÿת�ƶ�������˿���ݾ�    ��λ��mm(deg)
	uint8_t motor_dir;						//�����ת����    0--��ת    1--��ת
	uint8_t feedback_mode;					//�����������    0--����ʽ    1--����ʽ(����)     2--����ʽ(����)    3--��դ��        4--�޷���

    uint8_t ret_ref_mode;					//0--�޵������  1--�е������
    uint8_t absolute_ref_mode;              //����ʽ������㷽ʽ 0--�����ǵ��趨��ʽ    1--�޵�����㷽ʽ
    uint8_t ret_ref_dir;					//�زο��㷽��		0--����    1--����
	uint8_t ret_ref_change_dir;				//�زο��㻻��      0--����    1--ͬ��
	uint8_t ref_signal;						//�ο����ź�����    0--���ź�    1--Z�ź�
	uint8_t ret_ref_index;					//�زο���˳��		0-11:��һ~��ʮ��
	uint8_t ref_base_diff_check;			//�־���׼λ��ƫ����		0--�ر�   1--��
	double ref_base_diff;					//�־���׼λ��ƫ��		��λ��mm��deg����ֵΪ���ֵ�����ɸ���
	int64_t ref_encoder;					//��������ֵ������ʽ��������Ч
    double ref_offset_pos;                  //�زο�����ƫ���� ��λ:mm
    double ref_z_distance_max;              //����Z��������ƶ�����

	double manual_speed;					//�ֶ������ٶȣ���λ:mm/min
	double rapid_speed;						//��λ�ٶȣ���λ:mm/min
	double reset_speed;						//��λ�ٶȣ���λ��mm/min
	double ret_ref_speed;					//�زο����ٶȣ���λ��mm/min
    double ret_ref_speed_second;            //�زο�����ٶȣ���λ��mm/min

    double rapid_acc;   					//��λ���ٶȣ���λ:mm/s^2
    double manual_acc;						//�ֶ����ٶȣ���λ:mm/s^2
    double start_acc;						//�𲽼��ٶȣ���λ:mm/s^2

    double corner_acc_limit;				//�ս��ٶȲ����ƣ���λ��mm/min

    uint16_t rapid_s_plan_filter_time;			//G00 S���ٶȹ滮ʱ�䳣��

    uint8_t post_filter_type;		    //�岹���˲�������		0--�ر�    1--ƽ���˲�     2--��ֵ�˲�     3--S�;�ֵ�˲���
    uint16_t post_filter_time_1;		//�岹���˲���һ��ʱ�䳣��  ��λ��ms
    uint16_t post_filter_time_2;        //�岹���˲�������ʱ�䳣��  ��λ��ms

    uint8_t backlash_enable;					//�����϶�Ƿ���Ч
    //int16_t backlash_forward;					//�������϶����λ��um(deg)
    //int16_t backlash_negative;					//�������϶����λ��um(deg)
    double backlash_forward;					//�������϶����λ��mm(deg)
    double backlash_negative;                   //�������϶����λ��mm(deg)
    int16_t backlash_step;                      //�����϶��������λ��um(20-300)


    uint8_t  pc_type;							//�ݲ�����  0 �����ݲ�  1 ˫���ݲ�
    uint8_t  pc_enable;							//�ݲ��Ƿ���Ч
    uint16_t pc_offset;                         //�ݲ���ʼ��   1-10000
    uint16_t pc_count;							//�ݾ���������  0-10000
    uint16_t pc_ref_index;						//�ο��㲹��λ�� 1-10000
    double pc_inter_dist;						//�ݾಹ���������λ��mm(deg)

    double soft_limit_max_1;					//��������λ1
    double soft_limit_min_1;					//��������λ1
    uint8_t soft_limit_check_1;					//����λ1��Ч
    double soft_limit_max_2;					//��������λ2
    double soft_limit_min_2;					//��������λ2
    uint8_t soft_limit_check_2;					//����λ2��Ч
    double soft_limit_max_3;					//��������λ3
    double soft_limit_min_3;					//��������λ3
    uint8_t soft_limit_check_3;					//����λ3��Ч



    uint8_t off_line_check;						//����߼��       0--�ر�   1--��

    uint8_t ctrl_mode;							//����Ʒ�ʽ	   0--���巽��    1--��������    2--CW/CCW    3--ģ����
    uint32_t pulse_count_pr;					//ÿת���������
    uint8_t encoder_lines;						//��������Ȧ����   10~31
    uint16_t encoder_max_cycle;					//��������Ȧ���ֵ


    //������ز���
    int16_t zero_compensation;					//��Ư����(DA��ƽֵ) 0~4095
    uint8_t spd_gear_ratio;						//������ٱ�
    uint32_t spd_max_speed;						//�������ת��	��λ��rpm
    uint16_t spd_start_time;					//��������ʱ��  ��λ��0.1s
    uint16_t spd_stop_time;						//�����ƶ�ʱ��  ��λ��0.1s
    uint8_t spd_vctrl_mode;						//����ģ���ѹ���Ʒ�ʽ    0--��ֹ    1 -- 0V~10V     2-- -10V~10V
//	uint8_t spd_set_dir;						//�������÷���    0--��ת    1--��ת  //ʹ�õ����ת�������
    uint32_t spd_set_speed;						//��������ת��  ��λ��rpm
    uint8_t spd_speed_check;					//����ת�ټ��   0--�ر�   1--��
    uint8_t spd_speed_match;					//ת��ƥ����   0--�ر�   1--��
    uint8_t spd_speed_diff_limit;				//ת������������   0~100
    uint8_t spd_encoder_location;				//������λ��     0--�����     1--�����

    uint8_t spd_ctrl_GST;                       //SOR�ź���;  0�����ᶨ��  1�����ֻ���
    uint8_t spd_ctrl_SGB;                       //���ỻ����ʽ 0��A��ʽ  1��B��ʽ
    uint8_t spd_ctrl_SFA;                       //���ֻ���ʱ�Ƿ����SF�ź� 0�����  1�������
    uint8_t spd_ctrl_ORM;                       //���ᶨ��ʱ��ת�� 0����  1����
    uint8_t spd_ctrl_TCW;                       //����ת���Ƿ���M03/M04Ӱ�� 0������Ӱ�� 1����Ӱ��
    uint8_t spd_ctrl_CWM;                       //����ת��ȡ�� 0����  1����
    uint8_t spd_ctrl_TSO;                       //���������͸��Թ�˿ʱ�����ᱶ������  0��ǿ��100%  1����Ч
    uint16_t spd_analog_gain;                   //ģ��������� 700-1250 ��λ��0.1%
    uint16_t spd_sor_speed;                     //������ֻ���/����ʱ������ת�� rpm
    uint16_t spd_motor_min_speed;               //��������Сǯ���ٶ�
    uint16_t spd_motor_max_speed;               //���������ǯ���ٶ�
    uint16_t spd_gear_speed_low;                //���ֵ͵�λ���ת�� rpm
    uint16_t spd_gear_speed_middle;             //�����е�λ���ת�� rpm
    uint16_t spd_gear_speed_high;               //���ָߵ�λ���ת�� rpm
    uint16_t spd_gear_switch_speed1;            //B��ʽ��1->��2���ת��
    uint16_t spd_gear_switch_speed2;            //B��ʽ��2->��3���ת��
    double spd_sync_error_gain;               //ͬ���������  ��ΧΪ0~1000��Ĭ��Ϊ200
    double spd_speed_feed_gain;               //���ٶ�ǰ������ ��ΧΪ0~100000��Ĭ��Ϊ60000
    double spd_pos_ratio_gain;                //��λ�ñ������� ��Χ0~200000��Ĭ��Ϊ100000
    uint8_t spd_rtnt_rate_on;                   //��˿�����ڼ䣬�����Ƿ���Ч 0��ǿ��100%  1����Ч
    uint8_t spd_rtnt_rate;                      //��˿���˱��� ��λ��1%
    int32_t spd_rtnt_distance;                 //��˿���˵Ķ������ֵ ��λ��um

    double spd_locate_ang;                     //���ᶨ��Ƕ� ��λ����

    //��ת����ز���
    uint8_t fast_locate;						//���ٶ�λ    0--�ر�   1--��
    uint8_t pos_disp_mode;						//λ����ʾģʽ   0--ѭ��ģʽ��0~360��    1--��ѭ��ģʽ
    uint8_t pos_work_disp_mode;                 //���������Ƿ�ѭ����ʾ  0--��  1--��
    uint8_t pos_rel_disp_mode;                  //��������Ƿ�ѭ����ʾ  0--��  1--��
    uint8_t rot_abs_dir;                        //��ת�����ָ�����ת����  0--��ݷ���  1--ȡ����ָ�����

    //ͬ������ز���
    uint8_t sync_axis;							//�Ƿ�ͬ����  0--��   1--��
    uint8_t series_ctrl_axis;                   //�Ƿ������� 0--��   1--ͬ��   2--����
    uint8_t master_axis_no;						//�������
    uint8_t disp_coord;							//�Ƿ���ʾ����  0--��   1--��
    uint8_t auto_sync;							//�Ӷ���زο�����Զ�ͬ��У׼   0--��   1--��
    uint32_t sync_err_max_pos;					//λ��ͬ��������ֵ	 ��λ��um
    double benchmark_offset;					//�������׼λ��ƫ��  ��λ��mm
    uint8_t sync_pre_load_torque;               //Ԥ�ص���ƫ�� ��λ��1%
    uint8_t sync_err_max_torque;                //Ť��ͬ��������ֵ ��λ��1%
    uint32_t sync_err_max_mach;                 //����ͬ��������ֵ ��λ��um
    uint8_t sync_pos_detect;                    //�Ƿ����λ��ͬ������� 0--��   1����
    uint8_t sync_mach_detect;                   //�Ƿ��������ͬ������� 0--��   1����
    uint8_t sync_torque_detect;                 //�Ƿ����Ť��ͬ������� 0--��   1����
    uint16_t serial_torque_ratio;               //��������ϵ�� ��λ: 1%
    uint16_t serial_pre_speed;                  //Ԥ�ش����ٶ� ��λ��rpm

    double axis_home_pos[10];				//�ο���λ��  ��λ��mm
    double ref_mark_err;                   //�ο����׼���   ��λ��mm    ��Ч��Χ��0~10.0
    uint32_t spd_min_speed;                 //�������ת��   ��λ��rpm   0~100000

    uint8_t pmc_g00_by_EIFg;                //PMC���ʿ���
    uint16_t pmc_min_speed;                 //��СPMC�ƶ��ٶ�
    uint16_t pmc_max_speed;                 //���PMC�ƶ��ٶ�

    int    ref_complete;                    //�ο����Ƿ���
    bool init_backlash_dir;                 //�ݲ���ʼ������

    uint8_t decelerate_numerator;           //���ٱ�������
    uint8_t decelerate_denominator;         //���ٱ�����ĸ
};

/**
 * @brief HMIģ�鵶��ƫ����������
 */
struct HmiToolOffsetConfig{
	double geometry_compensation[3];    //���߼���ƫ�ã��������Ȳ���
	double geometry_wear;				//���߳���ĥ�𲹳�
	double radius_compensation;			//���߰뾶����
	double radius_wear;					//���߰뾶ĥ�𲹳�
};

/**
 * @brief HMIģ�鵶λ�������ݣ������뵶��ӳ���
 */
struct HmiToolPotConfig{
//	uint8_t tool_index;		//����
	uint8_t tool_type[kMaxToolCount];		//��������
	uint8_t huge_tool_flag[kMaxToolCount];           //�Ƿ�󵶱����󵶱���ζ��һ�ѵ���Ҫռ������λ
	uint8_t tool_pot_index[kMaxToolCount];			//���׺�
	int tool_life_max[kMaxToolCount];								//������������λ��min
	int tool_life_cur[kMaxToolCount];								//��ʹ����������λ��min
	int tool_threshold[kMaxToolCount];                             //��������Ԥ����ֵ
	uint8_t tool_life_type[kMaxToolCount];                          //���߼������ͣ�0--����������1--����ʱ�䣬2--��������
};

/**
 * @brief ĥ��ר�ò���
 */
struct GrindConfig{
	double wheel_radius;				//ɰ�ְ뾶
	double safe_pos_x;					//X�ᰲȫλ������
	double safe_pos_c;					//C�ᰲȫλ������
	double shock_z_max_dis;				//ɰ����ĥ��Z����󳤶�
	double shock_x_compen_ratio;		//ɰ����ĥ��X�Ჹ������
	double shock_speed;					//ɰ����ĥ���ٶ�
	uint8_t shock_manual_ratio_ctrl;	//���ٶ����ֶ����ʿ���
	uint8_t shock_init_dir;				//ɰ����ĥ����ʼ����
	uint8_t grind_dir;					//�����ӹ�����
	double corner_vec_speed;			//ת���������ٶ�
	uint8_t grind_smooth_fun;			//ĥ��ƽ������
	uint16_t grind_smooth_time;			//ĥ��ƽ��ʱ�䳣��
	double wheel_raidus_multi[6];		//����ɰ�ְ뾶
};

/**
 * @brief �������ò���
 */
struct FiveAxisConfig{
	uint8_t five_axis_type;				//����������������
	double x_offset_1;					//��һת��������Ե����X����
	double y_offset_1;					//��һת��������Ե����Y����
	double z_offset_1;					//��һת��������Ե����Z����
	uint8_t post_dir_1;					//��һת����ת������  0--��ʱ��    1--˳ʱ��
	uint8_t range_limit_1;				//��һת����ת����  0--����    1--������
	double x_offset_2;					//�ڶ�ת��������Ե����X����
	double y_offset_2;					//�ڶ�ת��������Ե����Y����
	double z_offset_2;					//�ڶ�ת��������Ե����Z����
	uint8_t post_dir_2;					//�ڶ�ת����ת������
	uint8_t range_limit_2;				//�ڶ�ת����ת����
	double rot_swing_arm_len;			//������ת�ڱ۳���
	uint8_t five_axis_coord;			//����������ϵ
	double speed_limit_x;				//��������X���ٶ�����
	double speed_limit_y;				//��������Y���ٶ�����
	double speed_limit_z;				//��������Z���ٶ�����
	double speed_limit_a;				//��������A���ٶ�����
	double speed_limit_b;				//��������B���ٶ�����
	double speed_limit_c;				//��������C���ٶ�����
	double intp_angle_step;				//���������ֲ岹�ǶȲ���
	double intp_len_step;				//���������ֲ岹��С����
	uint8_t five_axis_speed_plan;		//���������ٶȹ滮
	uint8_t five_axis_plan_mode;		//�����ٶȹ滮��ʽ

};

/**
 * @brief ���������ò���
 */
struct HmiFiveAxisV2Config
{
    uint8_t machine_type;               //����ṹ���� 0:������  1:˫��ͷ  2:ҡ��  3:���
    uint8_t pivot_master_axis;          //��һ��ת�� 0:X  1:Y  2:Z
    uint8_t pivot_slave_axis;           //�ڶ���ת�� 0:X  1:Y  2:Z
    double table_x_position;            //��ת̨x���� ��λ:mm
    double table_y_position;            //��ת̨y���� ��λ:mm
    double table_z_position;            //��ת̨z���� ��λ:mm
    double table_x_offset;              //�ڶ���ת��xƫ�� ��λ:mm
    double table_y_offset;              //�ڶ���ת��yƫ�� ��λ:mm
    double table_z_offset;              //�ڶ���ת��zƫ�� ��λ:mm
    uint8_t tool_dir;                   //�������� 0:X  1:Y  2:Z
    double tool_holder_offset_x;        //����xƫ�� ��λ:mm
    double tool_holder_offset_y;        //����yƫ�� ��λ:mm
    double tool_holder_offset_z;        //����zƫ�� ��λ:mm
    uint8_t table_master_dir;           //��һ��ת�᷽��  0:��  1:��
    uint8_t table_slave_dir;            //�ڶ���ת��  0:��  1:��
    double master_ref_angle_crc;        //��һ��ת���ʼ�Ƕ� ��λ:��
    double slave_ref_angle_crc;         //�ڶ���ת���ʼ�Ƕ� ��λ:��
    double tool_holder_length;          //�������� ��λ:mm
};

/**
 * @brief �������ͨ������
 */
struct ProcessParamChn{
	uint8_t rapid_mode;				//G00ģʽ    0����λ     1��ֱ�߶�λ
	uint8_t cut_plan_mode;			//�����滮ģʽ	0��T��   1��S��
	uint8_t rapid_plan_mode;		//��λ�滮ģʽ  0��T��   1��S��
	uint8_t corner_stop_enable;		//�ս�׼ͣ���ܿ���
	uint8_t corner_stop;			//�ս�׼ͣ�Ƕ�  0-180��
	uint8_t corner_stop_angle_min;  //�ս�׼ͣ���޽Ƕ� 0-180��
	uint8_t corner_acc_limit;		//�սǼ��ٶ�����	0--�ر�   1--��
	uint8_t chn_spd_limit_on_axis;  	//�������ٶȲ��ת���ٶ�ǯ��		0--�ر�   1--��
	uint8_t chn_spd_limit_on_acc;		//���ڼ��ٶȵ�С�߶ν����ٶ�ǯ��		0--�ر�   1--��
	uint8_t chn_spd_limit_on_curvity;	//�������ʼ����ļ��ٵ�С�߶ν����ٶ�ǯ��		0--�ر�   1--��
	uint8_t chn_rapid_overlap_level;	//G00�ص��ȼ�	0--�ر�  1~3--�ȼ�1~�ȼ�3
	uint16_t chn_s_cut_filter_time;		//��������S�͹滮ʱ�䳣��  //��λ��ms
	uint16_t chn_small_line_time;   //С�߶�ִ��ʱ�䳣��   ��λ��0.1ms   ��Χ[0, 10000]
	double chn_max_vel;					//ͨ����������ٶ�	//��λ��mm/min
	double chn_max_acc;					//ͨ�������ٶ�	//��λ��mm/s^2
	double chn_max_dec;					//ͨ�������ٶ�	//��λ��mm/s^2
	double chn_max_corner_acc;          //���սǼ��ٶ�		//��λ��mm/s^2
	double chn_max_arc_acc;				//������ļ��ٶ�	//��λ��mm/s^2
#ifdef USES_WOOD_MACHINE
	int flip_comp_value;            //���ǲ���    -10000~10000    ��λ��um
#endif
};

/**
 * @brief ������������
 */
struct ProcessParamAxis{
	double rapid_acc;   					//��λ���ٶȣ���λ:mm/s^2
	double manual_acc;						//�ֶ����ٶȣ���λ:mm/s^2
	double start_acc;						//�𲽼��ٶȣ���λ:mm/s^2

	double corner_acc_limit;				//�ս��ٶȲ����ƣ���λ��mm/min

	uint16_t rapid_s_plan_filter_time;			//G00 S���ٶȹ滮ʱ�䳣��

	uint8_t post_filter_type;		    //�岹���˲�������		0--�ر�    1--ƽ���˲�     2--��ֵ�˲�     3--S�;�ֵ�˲���
	uint16_t post_filter_time_1;		//�岹���˲���һ��ʱ�䳣��  ��λ��ms
	uint16_t post_filter_time_2;        //�岹���˲�������ʱ�䳣��  ��λ��ms
};

/**
 * @brief �����и����
 */
struct LaserConfig{
	double follow_height;       //����߶�   ��λ��mm
};


/**
 * @brief HMI��������ϵ����
 */
struct HmiCoordConfig{
	double offset[kMaxAxisChn];    //�Ṥ������ϵƫ��
};

//��������
struct SimulateData{
	int type;   //ָ������: -3-���淶Χ����   -2-���淶Χ����    -1--��ʼλ��   00-G00Ŀ��λ��    01--G01Ŀ��λ��  02--G02Ŀ��λ��
	                       //03--G03Ŀ��λ��   05--Բ������λ��   06--Բ���뾶   10--����ָ��
	double pos[3];    //�������ݣ�����������ΪԲ���뾶ʱ��pos[0]���ڴ�Ű뾶��pos[1]���ڴ�ŵ�ǰƽ�棨170--G17  180--G18   190--G19��
};

//������������
enum SimDataType{
	ZONE_LIMIT_MIN = -3,   //���淶Χ��������
	ZONE_LIMIT_MAX = -2,   //���淶Χ��������
	ORIGIN_POINT = -1,     //��ʼλ��
	RAPID_TARGET = 0,      //G00Ŀ��λ��
	LINE_TARGET = 1,       //G01Ŀ��λ��
	ARC2_TARGET = 2,       //G02Բ��Ŀ��λ��
	ARC3_TARGET = 3,	   //G03Բ��Ŀ��λ��
	ARC_CENTER = 5,        //Բ��Բ������
	ARC_RADIUS = 6         //Բ���뾶
};


/**
 * @brief ͨ������ģʽ
 */
enum ChnWorkMode{
	INVALID_MODE = 0,	//�Ƿ�ģʽ
	AUTO_MODE,			//�Զ�ģʽ
	MDA_MODE,			//MDAģʽ
	MANUAL_STEP_MODE,	//�ֶ�����ģʽ
	MANUAL_MODE,		//�ֶ�����ģʽ
	MPG_MODE,		    //����ģʽ
	EDIT_MODE,          //�༭ģʽ
	REF_MODE            //ԭ��ģʽ
};

/*
 * @brief �ӹ�״̬
 */
enum MachiningState {
	MS_READY = 0, 				//׼���ã����У�״̬
	MS_RUNNING, 				//����״̬
	MS_OUTLINE_SIMULATING, 		//����������
	MS_TOOL_PATH_SIMULATING, 	//��·������
	MS_MACHIN_SIMULATING,       //�ӹ�������
	MS_PAUSING, 				//��ͣ������
	MS_PAUSED, 					//��ͣ״̬
	MS_STOPPING,				//ֹͣ������
	MS_REF_POINT_RETURNING, 	//�ο��㷵����
	MS_WARNING					//ϵͳ�澯
};


/**
 * @brief ϵͳ����״̬mask����
 */
enum FuncStateMask{
	FS_SINGLE_LINE = 0,			//����
	FS_DRY_RUN,					//������
	FS_OPTIONAL_STOP,			//ѡͣ
	FS_HANDWHEEL_CONTOUR,		//���ָ���
	FS_BLOCK_SKIP,				//����
	FS_EDIT_LOCKED,				//�༭��
	FS_MACHINE_LOCKED,			//������
	FS_AUXILIARY_LOCKED, 		//����������
	FS_MANUAL_RAPID				//�ֶ�����
};




/*
 * @brief PLC�Ĵ�����ַ����
 */
enum PmcAddress {
	NULL_ADDR = 0,
	X_ADDR,
	Y_ADDR,
	F_ADDR,
	G_ADDR,
	R_ADDR,
	K_ADDR,
	D_ADDR,
	A_ADDR,
	T_ADDR,
	C_ADDR,
	DC_ADDR,
	DT_ADDR
};


/**
 * @brief �����������
 */
enum MonitorDataType{
	MDT_INVALID = 0,  		//δ��������       0
	MDT_CHN_RT_STATUS = 1,  //ͨ��ʵʱ����     1
	MDT_CHN_AXIS_POS,       //ͨ��ʵʱ��λ�ã�ˢ��Ƶ�ʸ���ʵʱ���ݣ�ͼ��ģʽʹ��    2
	MDT_CHN_PMC_AXIS_POS,   //PMC��λ������    3
	MDT_CHN_SIM_GRAPH,      //ͼ�η�������     4
	MDT_CHN_SIM_TOOL_PATH,  //��·��������     5
	MDT_CHN_SIM_MACH,       //�ӹ���������     6
	MDT_CHN_DEBUG_INFO		//ͨ��������Ϣ     7
};

//ģ�����bit����
enum ModuleReadyFlag{
	NONE_READY = 0x00,	//�޾���ģ��
	SC_READY = 0x01,	//SCģ��
	MC_READY = 0x02,	//MCģ��
    MI_READY = 0x04,    //MI�������
	PL_READY = 0x08,	//PLģ��
	SPANTAN_READY = 0x10,	//SPANTANģ��
	HMI_READY = 0x20,	//HMIģ��
    MI_START = 0x40,	//MIģ�飨MI����ǰ���ɿ�ʼˢ��G�źţ������޷�����λ���͵�����



#ifdef	USES_MC_B_BOARD
	NC_READY = 0x0F,    //NCģ�鶼����������HMIģ���ⶼ����,MC-Bû��SPANTAN
#else
	NC_READY = 0x1F,	//NCģ�鶼����������HMIģ���ⶼ����
#endif
	ALL_READY = 0x3F    //����ģ�����
};

//�ֶ���������
enum ManualStep{
	MANUAL_STEP_INVALID = 0,	//�Ƿ�ֵ
    MANUAL_STEP_1,				//1um       //0.001mm
    MANUAL_STEP_10,				//10um      //0.01mm
    MANUAL_STEP_100,			//100um     //0.1mm
    MANUAL_STEP_500,            //500um     //0.5mm
    MANUAL_STEP_1000,			//1000um    //1mm
    MANUAL_STEP_5000,           //5000um    //5mm
    MANUAL_STEP_10000,          //10000um   //10mm
    MANUAL_STEP_50000,          //50000um   //50mm
};

//�����������Ͷ���
enum ParamActiveType{
	ACTIVE_BY_POWEROFF = 0,	//������Ч
	ACTIVE_BY_RESET,        //��λ��Ч
	ACTIVE_BY_RESTART,		//��һ�μӹ���Ч
	ACTIVE_IMMEDIATE		//������Ч
};

//����ֵ���Ͷ���
enum ParamValueType{
	VALUE_UINT8 = 0,	//uint8_t
	VALUE_INT8,			//int8_t
	VALUE_UINT16,		//uint16_t
	VALUE_INT16,		//int16_t
	VALUE_UINT32,		//uint32_t
	VALUE_INT32,		//int32_t
	VALUE_UINT64,		//uint64_t
	VALUE_INT64,		//int64_t
	VALUE_DOUBLE		//double
};

//HMI��ʾ��Ϣ�����
enum HmiMsgType{
	MSG_INIT = 0,		//��ʼ����Ϣ
	MSG_TIPS 			//��ʾ��Ϣ
};

//HMI��ʾ��ϢID����
enum HmiMsgId{
	MSG_ID_AXIS_REF = 100,	//��δȷ���ο���
	MSG_ID_LIC_OVER = 160,  //ϵͳ��Ȩ��������

	MSG_ID_RESTARTING = 200,   //����������������

	MSG_ID_TOOL_CHANGING =210, //����������

	MSG_ID_GUARD

};

enum ToolPotLifeType {
    ToolPot_Close,      //�رռƴ�
    ToolPot_Cnt,        //���������ƴ�
};

enum SysUpdateType {
    BackUp_System_Param  = 0x01,        //ϵͳ����
    Backup_Tool_Param    = 0x02,        //������Ϣ
    Backup_Coord_Param   = 0x04,        //��������ϵ
    Backup_Pmc_Data      = 0x08,        //����ͼ
    Backup_Macro_Param   = 0x10,        //�����
    Backup_Esb_Data      = 0x20,        //Esb�ļ�
    Backup_Gcode_Data    = 0x40,        //G����
    Backup_IO_Remap      = 0x80,        //IO��ӳ��
    Backup_All           = 0xFFFF,      //ȫ�̱���
};

// ����/�ָ���ǰ״̬
struct SysUpdateStatus {
    enum Status {
        Idle            = 0,    //δ��ʼ
        Ready,                  //׼����ʼ
        Backupping,             //��ʼ����
        Recovering,             //��ʼ�ָ�
        FileTransing,           //�ļ�����
    };
    enum Type {
        Unkown,                 //δָ��
        Backup,                 //����
        Recover,                //�ָ�
    };
    SysUpdateStatus() = default;
    SysUpdateStatus(int cur_step, int total_step, Status status, Type type)
        : m_cur_step(cur_step), m_total_step(total_step), m_status(status), m_type(type)
    { }

    int m_cur_step = 0;
    int m_total_step = 0;
    int m_err_code = 0;
    Status m_status = Idle;
    Type m_type = Unkown;
};

struct TraceLogInfo {
    int             opt;    //��־����
    int             type;   //��־��Ϣ����
    std::string     msg;    //��־����
};

enum ScStatusType {
    Type_ProtectKey = 0,    //���򱣻�
};


/*****************************************************
** �ŷ����� *******************************************/

/**
 * @brief �ŷ���������
 */
enum class SG_Config_Type
{
    Rect = 1,
    Circle,
    RecCir,
    Tapping
};

/**
 * @brief ���͹켣
 */
struct SG_Rect_Config
{
    uint8_t axis_one = -1;
    uint8_t axis_two = -1;
    uint8_t interval = 8;       //��������
};

/**
 * @brief Բ�͹켣
 */
struct SG_Circle_Config
{
    uint8_t axis_one = -1;
    uint8_t axis_two = -1;
    uint8_t interval = 8;       //��������
    double radius = 5;          //�뾶
};

/**
 * @brief ��Բ�켣
 */
struct SG_RecCir_Config
{
    uint8_t axis_one = -1;
    uint8_t axis_two = -1;
    uint8_t interval = 8;       //��������
    double width = 10;          //���(���������ǰ뾶)
    double height = 20;         //�߶�(���������ǰ뾶)
    double radius = 5;          //���ǰ뾶
};

/**
 * @brief ���Թ�˿�켣
 */
struct SG_Tapping_Config
{
    //uint8_t axis_one = -1;      //�������
    //uint8_t axis_two = -1;      //Z�����
    uint8_t interval = 8;
};

/* �ŷ����� *******************************************
******************************************************/



struct FS_Entity
{
    int type_ = FILE_UNKOWN;
    //std::string name_;
    int size_ = 0;
    time_t time_;
    char name_[128] ;
    //std::string current_dir;
};

class FS_File : public FS_Entity
{
public:
    std::string GetFileName();      //��ȡ�ļ�����
private:
};

class FS_Dir : public FS_Entity
{
public:
    std::string GetDirName();       //��ȡĿ¼����
private:
};

class FS_FileSystem {
private:
    std::list<FS_File> file_entities_;  // �ļ��б�
    std::list<FS_Dir>  dir_entities_;   // Ŀ¼�б�
    std::string root_path_;
};


#endif /* INC_HMI_SHARED_DATA_H_ */

