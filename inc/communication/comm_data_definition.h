/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file mc_communication.h
 *@author gonghao
 *@date 2020/06/29
 *@brief ��ͷ�ļ�ΪͨѶ������ݽṹ������
 *@version
 */

#ifndef INC_COMMUNICATION_COMM_DATA_DEFINITION_H_
#define INC_COMMUNICATION_COMM_DATA_DEFINITION_H_

#include <stdint.h>
#include <netinet/in.h>
#include "circular_buffer.h"
#include "hmi_shared_data.h"

/*#####################################################################SC<-->MC############################################################################*/
//��������
const int kMaxCmdFifoCount = 15;  //  30;        //DSP��˫ͨ����FIFO��ȵ���Ϊ16��20210415��  //������ͨ��FIFO�����ȣ�ʵ�����Ϊ32�����������д��������ͨ�����Ϊ16
const int kMaxMcStatusCount = 128;      //MC״̬����32bits����������
const int kMaxGCodeDataCount = 30;      //28;  //GCode�˶�״̬������ռ��32bits����������
const int kMaxGCodeFifoCount = 15;  // 30;      //DSP��˫ͨ����FIFO��ȵ���Ϊ16��20210415��		//�����˶�ָ������ͨ��FIFO��ȣ�ʵ�����Ϊ32�����������д��
const int kMaxCmdCount = 5;			//����ͨ������֡��ռ32bits����������
const int kMaxDebugFifoCount = 254;	//��������ͨ��FIFO�����ȣ�ʵ�����Ϊ256�����������д��

//���˶���������
enum MoveCmdType{
	MOVE_G00 = 0,
	MOVE_G01 = 1,
	MOVE_G02 = 2,
	MOVE_G03 = 3,

};

//SC��MC���͵��˶��������ݰ�
struct GCodeData{
	uint16_t crc;			//CRCУ��
	uint16_t frame_index;	//֡�ţ�0-65535ѭ��
	uint16_t cmd;			//bit0-bit7:���˶��������ͣ�G00=0/G01=1/G02=2/G03=3, bit8-bit15:Reserved
	uint16_t ext_type;		//��չ����
	uint32_t line_no;		//�к�
	uint32_t feed;			//�����ٶ�
	uint32_t mode;			//ģʽmask
	uint32_t reserved;		//Ԥ��
	int64_t pos0;			//��λ������0����λ��0.1nm��
	int64_t pos1;			//��λ������1����λ��0.1nm��
	int64_t pos2;			//��λ������2����λ��0.1nm��
	int64_t pos3;			//��λ������3����λ��0.1nm��
	int64_t pos4;			//��λ������4����λ��0.1nm��
	int64_t pos5;			//��λ������5����λ��0.1nm��
	int64_t pos6;			//��λ������6����λ��0.1nm��
	int64_t pos7;			//��λ������7����λ��0.1nm��
	int64_t arc_center0;	//Բ������X����λ��0.1nm��
	int64_t arc_center1;	//Բ������Y����λ��0.1nm��
	int64_t arc_center2;	//Բ������Z����λ��0.1nm��
	int64_t arc_radius;		//Բ���뾶����λ��0.1nm��
};

//�˶���������֡
union GCodeFrame{
	GCodeData data;								//�������ݷ���
	int32_t data_w[kMaxGCodeDataCount];		//���ڼĴ���д��
	uint16_t data_crc[kMaxGCodeDataCount*2];	//����crc����
};

typedef CircularBuffer<GCodeFrame> GCodeFrameBuffer;  //�˶���������֡���壬ѭ������


//SC-MC����ṹ
struct McCmdData{
	uint16_t cmd; 		//ָ���
	uint8_t axis_index;	//���
	uint8_t channel_index;	//ͨ����
	uint16_t data[7];	//����
	uint16_t crc;		//crcУ����
};

//SC-MC����֡�ṹ
union McCmdFrame{
	McCmdData data;						//ָ������������ݷ���
	int32_t data_w[kMaxCmdCount];		//���ڼĴ���д�������
	int16_t data_crc[kMaxCmdCount*2];	//����crc����
};

enum McCmdCode{
	//ϵͳ��������
	CMD_MC_SET_HARDWARE_INFO = 0x0001,		//����Ӳ����Ϣ
	CMD_MC_SET_CNC_TYPE	= 0x0002,			//����CNC�ӹ�ģʽ
	CMD_MC_SET_CHN_AXIS_NUM = 0x0003,		//���ø�ͨ��������
	CMD_MC_RESET_ALARM = 0x0004,			//�澯��λ���
//	CMD_MC_CLEAR_DATA_BUF = 0x0005, 		//��ռӹ����ݻ�����
	CMD_MC_SET_INIT_PARAM = 0x0005,			//���ò岹���ڵȳ�ʼ������
	CMD_MC_SET_CHN_AXIS_NAME = 0x0006,		//����ͨ��������
	CMD_MC_SYS_RESET = 0x0007,				//ϵͳ��λ

	CMD_MC_SET_CHN_INTP_MODE = 0x0010,		//���ø�ͨ���岹ģʽ���Զ���MDA���ֶ���
	CMD_MC_INIT_CHN_AUTO_BUF = 0x0011,		//��ʼ����ͨ���Զ��ӹ����ݻ���
	CMD_MC_INIT_CHN_MDA_BUF = 0x0012,		//��ʼ����ͨ��MDA���ݻ���
//	CMD_MC_CLEAR_CHN_STEP_ARRIVED = 0x0013,	//
	CMD_MC_CLEAR_CHN_LINE_NO = 0x0014,		//������мӹ��к�
	CMD_MC_MULTI_AXIS_STOP = 0x0015,		//�ֶ�����ֹͣ
	CMD_MC_SET_CHN_WORK_MODE = 0x0016,      //����ͨ���ļӹ�ģʽ���Զ���MDA���ֶ���
	CMD_MC_SET_G84_INTP_MODE = 0x0017,      //����MC������岹ģʽG84�����Թ�˿��
	CMD_MC_STOP_G31_CMD = 0x0018,           //֪ͨMCֹͣG31�ƶ�����
	CMD_MC_SET_CHN_PLAN_MODE = 0x0020,		//����ͨ���ӹ��ٶȹ滮ģʽ
	CMD_MC_SET_CHN_PLAN_PARAM = 0x0021,		//����ͨ���ӹ��ٶȹ滮����
	CMD_MC_SET_CHN_PLAN_SWITCH = 0x0022,	//����ͨ���ӹ��ٶȹ��ܿ���
	CMD_MC_SET_CHN_CORNER_STOP_PARAM = 0x0023,	//���ùս�׼ͣ����
	CMD_MC_SET_CHN_RATIO = 0x0024,			//����ͨ���ӹ�����
	CMD_MC_SET_CHN_WORK_BAND = 0x0025, 		//����ͨ���ӹ����ȵȼ�
	CMD_MC_SET_STEP_MODE = 0x0026,			//���õ���ģʽ
	CMD_MC_ACTIVE_CHN_ORIGIN = 0x0027,		//����ͨ�������������ϵԭ��
	CMD_MC_ACTIVE_CHN_TOOL_OFFSET = 0x0028,	//����ͨ��������ĵ���ƫ��
	CMD_MC_SET_CHN_TOOL_LIFE = 0x0029,      //����ͨ����ǰ��������
	CMD_MC_SET_G84_PARAM = 0x002A,			//���øչ�����
	CMD_MC_SET_FLIP_COMP_PARAM = 0x002B,    //����ľ�����ǲ�������   ľ����ר��


	CMD_MC_SET_MULTI_AXIS_MODE = 0x0030,	//���ö�������ģʽ
	CMD_MC_SET_CHN_FIVE_AXIS_PARAM = 0x0031,	//�������������ӹ���ز���
	CMD_MC_SET_CHN_TILT_AXIS_PARAM = 0x0032,	//������б��ӹ���ز���


	//ĥ��ָ��
	CMD_MC_SET_SPECIAL_INTP_MODE = 0x0040,     //����ר�ò岹ʹ��   0--G13.1    121--G12.1   122--G12.2   123--G12.3   71--G7.1
	CMD_MC_SET_SPEC_INTP_PARM_DYNAMIC = 0x0041,		//����ר�ò岹�Ķ�̬����
	CMD_MC_ENABLE_GRIND_SHOCK = 0x0042,       //ʹ��G12.2����ĥ��
	CMD_MC_SET_SPEC_INTP_PARM_STATIC = 0x0043,	//����G12.2�ľ�̬����

	//���Բ���
	CMD_MC_SET_CHN_DEBUG_PARAM = 0x0060,     //����ͨ�����Բ���

	//��������
	CMD_MC_UPDATE_START = 0x0070,			//��ʼ����
	CMD_MC_UPDATE_FILE_SIZE = 0x0071,		//�����ļ���С���ֽ���
	CMD_MC_UPDATE_CRC = 0x0072,				//���������ļ�CRC

	//���������
	CMD_MC_SET_AXIS_ON = 0x0081,			//��ʹ������
	CMD_MC_SET_AXIS_BASE_INFO = 0x0082,	    //���ø����ݾ���Ϣ
	CMD_MC_SET_AXIS_SPEED_LIMIT = 0x0083,	//���ø����ٶ�����
	CMD_MC_SET_AXIS_ACC = 0x0084,			//���ø�����ٶ�ֵ
	CMD_MC_AXIS_MAUAL_MOVE = 0x0085, 		//���ƶ�ָ��
	CMD_MC_SET_AXIS_ORIGIN = 0x0086,		//���õ�������ϵԭ��
	CMD_MC_SET_AXIS_TOOL_OFFSET = 0x0087,	//���õ��ᵶ��ƫ��
	CMD_MC_SET_AXIS_HW_INSERT = 0x0088,		//���ֲ�����������
	CMD_MC_SET_AXIS_SOFT_LIMIT = 0x0090,	//��������λֵ
	CMD_MC_AXIS_ENABLE_SOFT_LIMIT = 0x0091, //��������λ��⿪��
	CMD_MC_AXIS_POS_THRESHOLD = 0x0092,     //���ø���λ��ָ������ֵ

	//״̬��������
	CMD_MC_GET_VERSION = 0x0110,			//��ȡDSP�汾��
	CMD_MC_SHAKEHANDS = 0x0111,				//��ʼ������
	CMD_MC_GET_LIMIT_LEVEL = 0x0112,		//��ȡ��λ�ĵ�ƽ״̬
	CMD_MC_GET_UPDATE_STATE = 0x0120,		//��ȡ����״̬
	CMD_MC_TEST_FLASH = 0x0121,				//����FLASH����


	//MC

	CMD_MC_GUARD							//����
};


/*#####################################################################SC<-->MI############################################################################*/
//SC-MI����ṹ
struct MiCmdData{
	uint16_t cmd; 		//ָ���
	uint8_t axis_index;	//���,������������
	uint8_t reserved;	//����
	uint16_t data[7];	//����
	uint16_t crc;		//crcУ����
};

//SC-MI����֡�ṹ
union MiCmdFrame{
	MiCmdData data;						//ָ������������ݷ���
	int32_t data_w[kMaxCmdCount];		//���ڼĴ���д�������
	int16_t data_crc[kMaxCmdCount*2];	//����crc����
};

//PMC���˶�ָ��ṹ
struct PmcCmdData{
	uint16_t cmd;			//�˶�ָ���
	uint16_t axis_index;	//��Ӧ���������
	uint16_t data[7];       //����
	uint16_t crc;		    //crcУ����
};

//PMC���˶�����֡�ṹ
union PmcCmdFrame{
	PmcCmdData data;					//ָ������������ݷ���
	int32_t data_w[kMaxCmdCount];		//���ڼĴ���д�������
	int16_t data_crc[kMaxCmdCount*2];	//����crc����
};

//MIָ���ֶ���
enum MiCmdCode{
	//MI->SC
	CMD_MI_READY = 0x01,					//֪ͨSCģ��MI׼����
	CMD_MI_ALARM = 0x02,					//֪ͨSCģ��MI�����澯
	CMD_MI_PMC_CODE_REQ = 0x03,				//��SC����PLC����
	CMD_MI_SHAKEHAND = 0x04,				//MI��SC��������
	CMD_MI_HEARTBEAT = 0x05,				//MI��SC֮�������
	CMD_MI_BUS_ALARM = 0x06,           		//MI֪ͨSC����ͨѶ����
	CMD_MI_PMC_AXIS_RUNOVER = 0x07,         //MI֪ͨPMC���˶�ָ��ִ�����
	CMD_MI_REFRESH_AXIS_ZERO = 0x08,        //MI֪ͨSC���������ƫ��
	CMD_MI_HW_TRACE_STATE_CHANGED = 0x09,   //MI֪ͨSC���ָ���״̬�л�
	CMD_MI_GET_ESB = 0x0A,                  //MI��SC��ȡESB�豸����

	//SC->MI
	CMD_MI_RD_PMC_REG = 0x100,				//SC��ȡPMC�Ĵ���ֵ
	CMD_MI_WR_PMC_REG = 0x101,				//SCд��PMC�Ĵ���ֵ
	CMD_MI_SET_PARAM = 0x102,				//SC��MI���ò���
	CMD_MI_GET_PARAM = 0x103,				//SC��MI��ȡ����
	CMD_MI_OPERATE = 0x104,					//SCָ��MIִ��ĳ���������������ŷ������ŷ��ȵ�
	CMD_MI_INIT_COMPLETE = 0x0105,			//SC��MI��ʼ���������
	CMD_MI_RESET = 0x0106,					//SC֪ͨMI���и�λ
	CMD_MI_SET_REF = 0x0107,				//SC����ָ����Ĳο���,���ھ���ֵ�������ڳ�ʼ��ʱ���������
	CMD_MI_SET_REF_CUR = 0x0108,			//SCָ��MI��ָ����ĵ�ǰ������ֵ��Ϊ�ο���, ����ֵ���������޷��������ô�����
	CMD_MI_ACTIVE_Z_CAPT = 0x0109,			//SC����ָ�����Z�źŲ�����
	CMD_MI_SET_SPD_SPEED = 0x010A,			//SC��������ת��
	CMD_MI_CLEAR_ROT_AXIS_POS = 0x010B,		//SC����ת��λ��������Ȧ
	CMD_MI_SET_AXIS_CHN_PHY_MAP = 0x010C,   //SC����ͨ�������������ӳ���ϵ
	CMD_MI_SET_AXIS_INIT_ENCODER = 0x010D,  //SC�����ϴζϵ�ǰ�������������
	CMD_MI_DO_SYNC_AXIS = 0x010E,			//SCָ��MI����ͬ����ͬ��
	CMD_MI_SET_WORK_MODE = 0x010F,		    //SC���ù���ģʽ
	CMD_MI_ENABLE_FOLLOW = 0x0110,			//ʹ�ܵ��������湦��
	CMD_MI_SET_LASER_OUTPUT = 0x0111,		//���ü�����������IO
	CMD_MI_READ_LASER_INPUT = 0x0112,		//��ȡ�������������IO
	CMD_MI_SET_POS_CAPTURE = 0x0113,		//������λ�ò���
	CMD_MI_SET_SDLINK_SLAVE = 0x0114,       //����SD-LINK��վ��Ϣ
	CMD_MI_SET_SDLINK_COMPLETE = 0x0115,    //����SD-LINK��վ��Ϣ���
	CMD_MI_PAUSE_PMC_AXIS = 0x0116,			//��ͣPMC������
	CMD_MI_ACTIVE_TWINING_FUNC = 0x0117,	//ʹ�ܲ��ƹ���
	CMD_MI_START_RET_REF = 0x0118,			//��ʼ�زο���
	CMD_MI_GET_CUR_ENCODER = 0x0119,		//��ȡ��ǰ������������Ȧ����ֵ
	CMD_MI_SET_REF_POINT = 0x011A,			//����ָ����Ĳο��㣬�����з��������л�׼�����������
	CMD_MI_SET_HANDWHEEL_TRACE = 0x011B,    //�������ָ���״̬
	CMD_MI_SET_RET_REF_FLAG = 0x011C,       //������زο��������־
	CMD_MI_GET_ZERO_ENCODER = 0x011D,       //SC��ȡָ����Ļ�е����Ӧ�ı�����ֵ
	CMD_MI_ACTIVE_SKIP = 0x011E,			//SC����MI����ת���ܣ�������ת�źż���mask
	
	CMD_MI_SET_AXIS_SPEED  = 0x0120,       //�����������ٶ�ֵ
	CMD_MI_SET_AXIS_TORQUE = 0x0121,       //��������������ֵ
    CMD_MI_SET_AXIS_CTRL_MODE = 0x0122,       //���������п���ģʽ
	CMD_MI_SET_AXIS_REF_MACH_POS = 0x123,     //������ο����Ӧ�Ļ�е����

	CMD_MI_SET_CUR_CHN = 0x0124,			//���õ�ǰͨ����
	CMD_MI_SET_TAP_AXIS = 0x0125,            //���ù�˿���
	CMD_MI_SET_TAP_PARAM = 0x0126,           //���ù�˿����
	CMD_MI_SET_TAP_RATIO = 0x0127,           //���ù�˿����
	CMD_MI_SET_TAP_STATE = 0x0128,           //���ù�˿״̬����

	CMD_MI_EN_SYNC_AXIS = 0x129,            //ͬ����ʹ��

	CMD_MI_SET_AXIS_PC_DATA = 0x130,      //�������ݲ����ݱ�
	CMD_MI_SET_AXIS_PC_PARAM = 0x131,     //�������ݲ�����
	CMD_MI_SET_AXIS_PC_PARAM2 = 0x132,     //�������ݲ�����2�����ο����Ӧ�Ļ�е����
	CMD_MI_SET_AXIS_BACKLASH = 0x133,     //������ķ����϶����

	CMD_MI_SET_PMC_REG_BIT = 0x134,       //����PMC�Ĵ���λ
	CMD_MI_SET_IO_REMAP = 0x135,          //����IO�ض�������

	CMD_MI_SET_AXIS_MACH_POS = 0x136,     //����ָ�������ᵱǰλ�õĻ�е����
    CMD_MI_SPD_LOCATE = 0x137,            //SC֪ͨMI���ᶨλ

	CMD_MI_DEBUG_M_FUN = 0x8001,            //MI����ָ��
	CMD_MI_HW_TRACE_STATE_CHANGED_RSP = 0x8009,   //MI֪ͨSC���ָ���״̬�л���Ӧ����

	CMD_MI_GUARD							//����
};


/*#####################################################################SC<-->HMI############################################################################*/
const int kCmdRecvBufLen = 100;  //udp������ܻ�������С�����λ���
const int kTcpBufferSize = 1024*8;  //tcp���仺������С,8K
const unsigned int kHeartbeatTimeout = 100; //30;  //30��������ڣ�30*50ms
const int kTcpSockTimeout = 3;		//TCP���ӽ������ݣ�����recv��accept����ʱʱ�䣺3��

//�յ���HMIָ��ڵ�
struct HMICmdRecvNode{
	HMICmdFrame cmd;			   //�����
	struct sockaddr_in ip_addr;   //����Դ��ַ
};

/**
 * @brief HMICmdNode�ṹ���ڶ���SCģ�鷢�͸�HMI��������ط����нڵ�
 */
struct HMICmdResendNode {
	HMICmdFrame frame;
	uint8_t timeout;   //��ʱ����������ʼֵΪ5,ÿ100ms��һ������0���ط�������timeout������һ
	uint8_t resend_count;  //�ط�����������¼�ط�����
};

enum ThreadRunFlag{
	THREAD_CMD_PROCESS = 0x01,	//������߳�
	THREAD_CMD_RESEND = 0x02,	//�����ط��߳�
	THREAD_FILE_TRANS = 0x04,	//�ļ������߳�
	THREAD_MONITOR = 0x08,		//����߳�
	THREAD_FILE_SAVEAS = 0x10	//�ļ����Ϊ�߳�
};

#endif /* INC_COMMUNICATION_COMM_DATA_DEFINITION_H_ */
