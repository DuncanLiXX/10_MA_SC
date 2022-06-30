/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file global_definition.h
 *@author gonghao
 *@date 2020/03/18
 *@brief ��ͷ�ļ���������ȫ����ֵ����
 *@version
 */

#ifndef INC_GLOBAL_DEFINITION_H_
#define INC_GLOBAL_DEFINITION_H_
#include <stdint.h>
//#include "hmi_shared_data.h"

#define USES_FIVE_AXIS_FUNC     //���Ṧ��

//#define USES_LICENSE_FUNC     //��Ȩ����

//#define USES_TCP_FILE_TRANS_KEEP     //tcp�ļ��������ӱ��֣�����ÿ�����½�������

#define USES_GENERAL_AUTOMATIC  //ͨ���Զ���
//#define USES_GRIND_MACHINE      //ĥ����ع�������
//#define USES_LASER_MACHINE      //�����豸��������
//#define USES_WUXI_BLOOD_CHECK   //����ѪҺ���
//#define USES_TIANJIN_PROJ            //���6ͨ����������

//#define USES_TWINING_FUNC      //���ƹ���

//#define USES_MAIN_BOARD_10MA_OLD          //�ϵ�10MA���ư�

//#define USES_PMC_PROCESS     //��PMC����PMC������в���  ³������

#define MOVE_FILTER_PARAM_FROM_CHN_TO_AXIS     //���岹��Ӽ��ٲ�����ͨ������Ų�������

#define USES_SPEED_TORQUE_CTRL      // �ٶȿ��������ؿ��ƿ���

#define USES_PMC_2_0         //ʹ��PMC2.0�汾

#define USES_TOOL_COMPENSATE    //֧�ֵ��߰뾶����

#define USES_MODBUS_PROTOCAL   //֧����̬ͨѶЭ��

#define USES_PHYSICAL_MOP       //ʹ��ʵ��MOP

//#define USES_INDEPEND_BASE_TOOL_OFFSET    //ʹ�ö����Ļ�׼��ƫ��

//#define USES_T_CODE_MACRO        //Tָ��ʹ�ú����ִ��

#define USES_PEITIAN_SMALL_FIVE   //֧������С����
//#define USES_RET_REF_TO_MACH_ZERO   //֧�ֻزο��㶯������ߵ���е��λ��


//#define USES_STOP_SPD_IN_RESET   //֧�ָ�λʱͣ����

#define USES_MC_B_BOARD       //֧��MC-B����

#define USES_WOOD_MACHINE     //֧��ľ��ר��

#define USES_ADDITIONAL_PROGRAM     //֧��ǰ�á����ó������

#define USES_EMERGENCY_DEC_STOP   //֧�ּ�ͣ����ͣ

// #define USES_SIMULATION_TEST   //������ԣ��������ݱ���Ϊ�ļ�


//�汾��Ϣ
#define ADX_SC_VERSION "P0.3.01" //SCģ������汾��

//�ļ�Ŀ¼����
#define PATH_NC_FILE "/cnc/nc_files/"    //NC�ӹ��ļ�Ŀ¼0
#define PATH_NC_SUB_FILE "/cnc/nc_files/sys_sub/"   //ϵͳ�ӳ���Ŀ¼
#define PATH_NC_SIGN_FILE "/cnc/nc_files/md5/"   //NC�ļ�ǩ���ļ�Ŀ¼
#define PATH_LOG_FILE "/cnc/trace/log/"	//LOG�ļ�Ŀ¼
#define PATH_ALARM_FILE "/cnc/trace/alarm/"	//�澯�ļ�Ŀ¼
#define PATH_CONFIG_FILE "/cnc/config/"	//�����ļ�Ŀ¼
#define PATH_PMC_FILE "/cnc/pmc/"          //PMC�ļ�Ŀ¼
#define PATH_PMC_DATA "/cnc/pmc/pmc_run.dat"   //PMC�����ļ�·��
#define PATH_PMC_LDR "/cnc/pmc/pmc_ladder.ldr"	//PMC����ͼ�ļ�·��
#define PATH_MDA_FILE "/cnc/mda/mda_%hhu.nc"    //���MDA������ļ�
#define PATH_UPDATE_PATH "/cnc/update/"    //��������ļ�·��
#define PATH_BOOT_CONFIG_FILE "/cnc/config/bootinfo.cfg"   //��������ļ�
#define PATH_SPARTAN6_PROGRAM "/cnc/bin/top.bit"     //spartan6�����ļ�
#define PATH_SPARTAN6_PROGRAM_BAK "/cnc/config/sys_top.cfg"	//spartan6�����ļ�����
#define PATH_MI_PROGRAM "/cnc/bin/MI.bin"            //MIģ������ļ�
#define PATH_MI_PROGRAM_BAK "/cnc/config/MI_BAK.bin"     //MIģ����򱸷��ļ�
#define PATH_MODBUS_PROGRAM "/cnc/bin/SCModbus_upd.elf"     //Modbusģ����������ļ�
#define PATH_PMC_REG "/cnc/pmc/pmc_register.dat"		//PMC����ʧ�ԼĴ��������ļ�
#define PATH_MACRO_VAR_KEEP "/cnc/config/macro_var/macro_variable_%hhu.dat"   //����ʧ�Ժ���������ļ�
#define PATH_USER_MACRO_VAR "/cnc/config/macro_var/user_macro_variable_%hhu.dat"
#define PATH_PHY_AXIS_ENCODER "/cnc/config/phy_encoder.dat"   //�����ᵱǰ����������
#define PATH_CONFIG_PACK_TMP "/cnc/config/config_pack_tmp.cpu"   //��������ļ���ʱ���·��
#define PATH_PC_DATA_TMP "/cnc/config/pc_data_input_tmp.dat"    //�ݲ������ļ���ʱ���·��
#define PATH_ESB_FILE "/cnc/svo_esb/"    //�ŷ������ļ����·��
#define PATH_UPDATE_DISK_CMD "/cnc/diskup/scripts/scup.sh" //һ������SC�ű�
#define PATH_BACK_DISK_CMD "/cnc/disk/scripts/scback.sh" //һ������SC�ű�

#define PATH_SIMULATE_DATA "/cnc/tmp/simulate_data.dat"   //���������ļ�


//�����ļ�·��
#define SYS_CONFIG_FILE "/cnc/config/sys_config.ini"   //ϵͳ�����ļ�
#define CHN_CONFIG_FILE "/cnc/config/chn_config.ini"   //ͨ�������ļ�
#define AXIS_CONFIG_FILE "/cnc/config/axis_config.ini"  //�������ļ�
#define PITCH_COMP_FILE "/cnc/config/pc_data.ini"       //�ݲ������ļ�
#define TOOL_CONFIG_FILE "/cnc/config/tool_config.ini"  //���������ļ�
#define WORK_COORD_FILE  "/cnc/config/work_coord.ini"   //��������ϵ�����ļ�
#define EX_WORK_COORD_FILE "/cnc/config/ex_work_coord.ini" //��չ��������ϵ�����ļ�
#define CHN_SCENE_FILE "/cnc/config/chn_scene.ini"		//����ͨ����״̬����
#define IO_REMAP_FILE "/cnc/config/ioremap.ini"       //IO�ض��������ļ�

#define CHN_PROC_PARAM_FILE "/cnc/config/chn_proc_param.ini"   //�������ͨ�������ļ�
#define AXIS_PROC_PARAM_FILE "/cnc/config/axis_proc_param.ini"   //�������ͨ�������ļ�

#ifdef USES_FIVE_AXIS_FUNC
#define FIVE_AXIS_CONFIG_FILE "/cnc/config/five_axis_config.ini"    //���������ļ�
#endif

#ifdef USES_GRIND_MACHINE
#define GRIND_CONFIG_FILE "/cnc/config/grind_config.ini"   //ĥ��ר�ò��������ļ�
#endif


#define BOARD_TEMPATURE "/sys/bus/i2c/devices/1-0048/hwmon/hwmon0/temp1_input"   //�����¶ȱ����ļ�
//

#define MI_START_REG            (0xFFFFFFF0)			//MI�����Ĵ�����ע�ⲻ�ڹ����ڴ淶Χ�ڣ���Ҫ����ӳ�䴦��

#define uchar unsigned char

//ƽ�涨��
#define PLANE_XY (170)
#define PLANE_ZX (180)
#define PLANE_YZ (190)

//NC���ģʽ
#define ABS_MODE (0)  //����������
#define RELATIVE_MODE (1)    //���������

#define MZERO (1E-6)  //����0ֵ����
#define GetModeGroup(x) GCode2Mode[x/10]


//��λת��
#define MM2NM0_1(x) (int64_t)(x * 1e7)		//��mmת��Ϊ0.1nm
#define FEED_TRANS(x) (int32_t)(x * 1000 / 60)   //��mm/minת��Ϊum/s
#define FEED_TRANS_REV (double)((x * 60)/1000.)   //��um/sת����mm/min


#define SET_SECTION8_LED _IO('x', 1) //����8�ι�
#define GET_SECTION8_LED _IO('x', 2) //��ȡ8�ι�
#define GET_GPIO_VALUE _IO('x', 3) //��ȡGPIO ��ƽ Bit0:Ƿѹ1:��ѹ2:��ص�ѹ��3:�����4:USB����5:485����ʹ��6:ϵͳ����7:Nandд����
#define SET_NOTIFY_PID _IO('x', 4) //����֪ͨPID
#define GET_NOTIFY_PID _IO('x', 5) //��ȡ֪ͨPID
#define SET_NOTIFY_INTERVAL _IO('x', 6) //����֪ͨ���(ns)
#define GET_NOTIFY_INTERVAL _IO('x', 7) //����֪ͨ���(ns)
#define SET_RS485_EN _IO('x', 8) //����485����ʹ��
#define SET_SYS_WARN _IO('x', 9) //����ϵͳ����
#define SET_NAND_PROTECTED _IO('x', 10) //����NANDд����
#define SIMULATE_ALARM_INTERRUPT _IO('x', 11) //������������ж�



//MC״̬ˢ����ʱ
#define MC_STATE_REFRESH_CYCLE (10000)    //ˢ������Ϊ5ms���������ʹ��10ms

const int64_t kAxisRefNoDef =  0xffffffffffffffff;    //����ʽ�����������δ����


const int kThreadStackSize = 1024 * 1024;   //�߳�ջ��С�� 1M

const int kMonitorInteval = 50000;    //HMI������ݷ��ͼ������λus

//ͨ�����
//const int kOutputBufCount = 20;	//�˿�����֡�����С
//const int kMaxOutputMsgCount = 100;  //���ָ����Ϣ�б���������ڵ���
const uint8_t kMaxRatio = 150;		//�������ֵ150%


//��չ��������ϵ
const int kMaxExWorkCoordCount = 99;		//��չ����ϵ������

const double EPSINON = 1e-6;   //���㾫��

//ͨ�����������ţ����ڸ澯����Ϣ�������ͨ���޹�
const uint8_t CHANNEL_ENGINE_INDEX = 0XFF;    //ͨ������������

//���޹أ����ڸ澯����Ϣ
const uint8_t NO_AXIS = 0XFF;

const uint32_t kSp6FifoDepth = 16;    //spartan6����ͨ��FIFO���

//�ӳ���ŵ���󳤶�
const int kMaxSubNameLen = 6;    //�ӳ���ŷ�Χ��0~999999

//˳��ŵ���󳤶�
const int kMaxLineNoLen = 9;    //˳��ŷ�Χ��0~999999999

//Mָ��������󳤶�
const int kMaxMCodeLen = 9;    //��Χ��0~999999999

//Sָ��������󳤶�
const int kMaxSCodeLen = 9;    //��Χ��0~999999999

//�ӳ������Ƕ�ײ���
const int kMaxSubNestedCount = 16;    //�ӳ������Ƕ��16��

//WHILE-DOʶ������ֵ
const int kMaxDoIndex = 8;

const uint8_t kMaxChnSpdCount = 6;	 //ͨ���������������

const int kMaxOutputBufCount = 1000;    //������е������������Ҫ�������ַ�������

const int kTCodeSubProg = 9000;     //Tָ���Ӧ���ӳ����

const int kMCodeTimeout = 500000;    //Mָ���жϺϷ��Եĳ�ʱʱ��  ��λ��us

//ESB�ļ����͸�MI�������ֽ���
const int kEsbDataSize = 998;

const int kGraphPosBufCount = 300;    //ͼ��λ�����ݻ��壬���ڷ����ͼ�ĸ�Ƶλ������

//�����������ַ�
const char strAxisNames[] = "XYZABCUVW";

//spartan6������عܽŶ���
#define SP6_PROGRAM "963"   //906+57
#define SP6_INIT "962"      //906+56;
#define SP6_CLK "960"          //906+54;
#define SP6_DATA  "964"         //906+58;
#define SP6_DONE  "961"         //906+55;

#define SP6_PROG_DIR "/sys/class/gpio/gpio963/direction"
#define SP6_INIT_DIR "/sys/class/gpio/gpio962/direction"
#define SP6_CLK_DIR "/sys/class/gpio/gpio960/direction"
#define SP6_DATA_DIR "/sys/class/gpio/gpio964/direction"
#define SP6_DONE_DIR "/sys/class/gpio/gpio961/direction"

#define SP6_PROG_VALUE "/sys/class/gpio/gpio963/value"
#define SP6_INIT_VALUE "/sys/class/gpio/gpio962/value"
#define SP6_CLK_VALUE "/sys/class/gpio/gpio960/value"
#define SP6_DATA_VALUE "/sys/class/gpio/gpio964/value"
#define SP6_DONE_VALUE "/sys/class/gpio/gpio961/value"


//SCģ����������
enum ModuleStartStep{
	STEP_POWER_ON = 0,		//ϵͳ�ϵ�
	STEP_INIT_TRACE,		//��ʼ����־ģ��
	STEP_LOAD_SP6_DATA,		//����SPARTAN6�Ĵ���
	STEP_LOAD_MI,			//����MIģ��Ĵ���
	STEP_INIT_PARM_MANAGER,	//��ʼ�����ù���ģ��
	STEP_INIT_HMI_COMM,		//��ʼ��HMIͨѶ�ӿ�
	STEP_INIT_MI_COMM,		//��ʼ��MIͨѶ�ӿ�
	STEP_INIT_MC_COMM,		//��ʼ��MCͨѶ�ӿ�
	STEP_INIT_ALARM_PROC,	//��ʼ���澯����ģ��
	STEP_INIT_CHN_ENGINER,	//��ʼ��ͨ������

	STEP_ALL_DONE			//��ʼ�����
};



/*
 * @brief ����ϵ����
 */
enum CoordId {
	MACHINE_COORD,
	G54_COORD,
	G55_COORD,
	G56_COORD,
	G57_COORD,
	G58_COORD,
	G59_COORD,
	G5401_COORD,
	G5402_COORD,
	G5403_COORD,
	G5404_COORD,
	G5405_COORD,
	G5406_COORD,
	G5407_COORD,
	G5408_COORD,
	G5409_COORD,
	G5410_COORD,
	G5411_COORD,
	G5412_COORD,
	G5413_COORD,
	G5414_COORD,
	G5415_COORD,
	G5416_COORD,
	G5417_COORD,
	G5418_COORD,
	G5419_COORD,
	G5420_COORD,
	G5421_COORD,
	G5422_COORD,
	G5423_COORD,
	G5424_COORD,
	G5425_COORD,
	G5426_COORD,
	G5427_COORD,
	G5428_COORD,
	G5429_COORD,
	G5430_COORD,
	G5431_COORD,
	G5432_COORD,
	G5433_COORD,
	G5434_COORD,
	G5435_COORD,
	G5436_COORD,
	G5437_COORD,
	G5438_COORD,
	G5439_COORD,
	G5440_COORD,
	G5441_COORD,
	G5442_COORD,
	G5443_COORD,
	G5444_COORD,
	G5445_COORD,
	G5446_COORD,
	G5447_COORD,
	G5448_COORD,
	G5449_COORD,
	G5450_COORD,
	G5451_COORD,
	G5452_COORD,
	G5453_COORD,
	G5454_COORD,
	G5455_COORD,
	G5456_COORD,
	G5457_COORD,
	G5458_COORD,
	G5459_COORD,
	G5460_COORD,
	G5461_COORD,
	G5462_COORD,
	G5463_COORD,
	G5464_COORD,
	G5465_COORD,
	G5466_COORD,
	G5467_COORD,
	G5468_COORD,
	G5469_COORD,
	G5470_COORD,
	G5471_COORD,
	G5472_COORD,
	G5473_COORD,
	G5474_COORD,
	G5475_COORD,
	G5476_COORD,
	G5477_COORD,
	G5478_COORD,
	G5479_COORD,
	G5480_COORD,
	G5481_COORD,
	G5482_COORD,
	G5483_COORD,
	G5484_COORD,
	G5485_COORD,
	G5486_COORD,
	G5487_COORD,
	G5488_COORD,
	G5489_COORD,
	G5490_COORD,
	G5491_COORD,
	G5492_COORD,
	G5493_COORD,
	G5494_COORD,
	G5495_COORD,
	G5496_COORD,
	G5497_COORD,
	G5498_COORD,
	G5499_COORD,
	GUARD_COORD			//����
};

//MCģ�鹤��ģʽ
enum MCWorkMode{
	MC_MODE_MANUAL = 0, 	//�ֶ�ģʽ
	MC_MODE_AUTO,			//�Զ�ģʽ
	MC_MODE_MDA				//MDAģʽ
};

enum ManualMoveDir{
	DIR_NEGATIVE = -1,		//����
	DIR_STOP = 0,			//ֹͣ
	DIR_POSITIVE = 1,		//����
};

enum NonGModeType{
	T_MODE = 0,			//T
	D_MODE,				//D
	H_MODE,				//H
	F_MODE,				//F
	S_MODE				//S
};


enum AxisInterfaceType{
	VIRTUAL_AXIS = 0,	//������
	BUS_AXIS,			//������
	ANALOG_AXIS			//ģ����
};

enum MotorFeedBackMode{
	INCREMENTAL_ENCODER = 0,   //����ʽ������
	ABSOLUTE_ENCODER_YASAKAWA,	//����ʽ��������������ABZ����
	ABSOLUTE_ENCODER_PANASONIC, //����ʽ������������,U485����
	LINEAR_ENCODER,				//��դ��
	NO_ENCODER                  //�޷���
};


/**
 * @brief MI�Ŀ����ֵ�ַ����
 */
enum MiCtrlOperate{
	AXIS_ON_FLAG = 0,			//��ʹ��
	MOTOR_ON_FLAG,
	MOTION_LEFT_LMT_FLAG,
	HW_LMT_EN_FLAG,
	HW_LMT_SENSE_FLAG,
	SW_LMT_EN_FLAG,
	HW_LMT_EN_ONCE_FLAG,
	RESERVED_1,
	RUN_DIR_FLAG,
	SRC_DEV_FLAGS,
	CAPT_COM_FLAG = 12,
	CAPT_EN_FLAG,
	CAPT_SIGNAL_FLAGS = 15,
	MOTION_RIGHT_LMT_FLAG = 17,
	PLC_INTPLA_PIMIT,
	MANU_POS_UPT_FLAG,
	CTRL_MODE_FLAG,
	PLAN_MODE_FLAGS,
	AUTO_MDI_INTPLA_PIMIT = 23,
	MANU_INTPLA_PIMIT,
	PULSE_DIR_FLAG,
	START_LMT_FLAG,
	AUTO_SMOOTH_STOP_FLAG,
	MDI_INTP_DIR_FLAG,
	AUTO_INTP_DIR_FLAG,
	RESERVED_2,
	AXIS_OFF_CHECK_FLAG

};

/**
 * @brief ����������ת����
 */
enum SpdDirect{
	SPD_DIR_NEGATIVE = -1,		//��ת
	SPD_DIR_STOP = 0,			//ͣת
	SPD_DIR_POSITIVE = 1		//��ת
};

/**
 * @brief �����Ͷ���
 */
enum AxisType{
	AXIS_LINEAR = 0,		//ֱ����
	AXIS_ROTATE,			//��ת��
	AXIS_SPINDLE,			//����
	AXIS_TORQUE             //Ť�ؿ�����
};

/**
 * @brief ֱ�������Ͷ���
 */
enum LinearAxisType{
	LINE_AXIS_NONE = 0,		//δ����
	LINE_AXIS_X,			//������X
	LINE_AXIS_Y,			//������Y
	LINE_AXIS_Z,			//������Z
	LINE_AXIS_PX,			//ƽ����X
	LINE_AXIS_PY,			//ƽ����Y
	LINE_AXIS_PZ			//ƽ����Z
};

/**
 * @brief �����ƶ���
 */
enum AxisName{
	AXIS_NAME_X = 0,        //X��
	AXIS_NAME_Y,			//Y��
	AXIS_NAME_Z,			//Z��
	AXIS_NAME_A,			//A��
	AXIS_NAME_B,			//B��
	AXIS_NAME_C,			//C��
	AXIS_NAME_U,			//U��
	AXIS_NAME_V,			//V��
	AXIS_NAME_W				//W��
};
/**
 * @brief ���ָ���״̬����
 */
enum HWTraceState{
	NONE_TRACE = 0,		//�����ָ���״̬
	NORMAL_TRACE,		//�������ָ���
	REVERSE_TRACE       //�������ָ���
};


#ifdef 	USES_LASER_MACHINE
const int kHYDRangeCount = 4;    //HYD(�����)����������������
enum HYDRangeIdx{
	HYD_RANGE_10 = 0,     //10mm����
	HYD_RANGE_15,			//15mm����
	HYD_RANGE_20,			//20mm����
	HYD_RANGE_25			//25mm����
};
const double kHYDCalibRefPos[kHYDRangeCount][16] = {10.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.5, 2.0, 1.8, 1.5, 1.2, 1.0, 0.7, 0.5, 0.2,
		                                               15.0, 12.0, 10.5, 9.0, 7.5, 6.0, 4.5, 3.75, 3.0, 2.7, 2.25, 1.8, 1.5, 1.05, 0.75, 0.3,
													   20.0, 16.0, 14.0, 12.0, 10.0, 8.0, 6.0, 5.0, 4.0, 3.6, 3.0, 2.4, 2.0, 1.4, 1.0, 0.4,
													   25.0, 20.0, 17.5, 15.0, 12.5, 10.0, 7.5, 6.25, 5.0, 4.5, 3.75, 3.0, 2.5, 1.75, 1.25, 0.5};
#endif

#ifdef USES_FIVE_AXIS_FUNC
enum FiveAxisMachType{
    NO_FIVEAXIS = 0,    	// ����ر�
    C_A_FIVEAXIS = 1,   	// C-A�����������(˫��ͷ��)
    C_B_FIVEAXIS = 2,  		// C-B�����������(˫��ͷ��)
    A_B_FIVEAXIS= 3,   		//  A-B�����������(˫��ͷ��)
    B_A_FIVEAXIS = 4,   	// B-A�����������(˫��ͷ��)
    A1_C1_FIVEAXIS = 5,   	//C'-A'�����������(˫ת̨��
    B1_C1_FIVEAXIS = 6,    	//C'-B'�����������(˫ת̨��
    A1_B1_FIVEAXIS = 7,   	// A'-B'�����������(˫ת̨��
    B1_A1_FIVEAXIS = 8,  	// B'-A'�����������(˫ת̨��)
    C1_A_FIVEAXIS= 9,   	//  C'-A�����������(�����)
    C1_B_FIVEAXIS = 10,   	// C'-B�����������(�����)
    A1_B_FIVEAXIS = 11,  	//A'-B�����������(�����)
    B1_A_FIVEAXIS = 12,    	//B'-A�����������(�����)
	C_A_TILT_FIVEAXIS = 13, //C-A��б���������
};

enum FiveAxisParamType{
	MACHINE_TYPE = 1,       //��������
	X_OFFSET_1 = 11,        //��һת��������Ե����X����
	Y_OFFSET_1 = 12,        //��һת��������Ե����Y����
	Z_OFFSET_1 = 13,        //��һת��������Ե����Z����
	X_OFFSET_2 = 14,        //�ڶ�ת��������Ե����X����
	Y_OFFSET_2 = 15,        //�ڶ�ת��������Ե����Y����
	Z_OFFSET_2 = 16,        //�ڶ�ת��������Ե����Z����
	ROT_SWING_ARM_LEN = 17, //������ת�ڱ۳���
	POST_DIR_1 = 18,        //��һת����ת������
	POST_DIR_2 = 19,        //�ڶ�ת����ת������

	SPEED_LIMIT = 21,		//�����ٶ����ƿ���

	SPEED_LIMIT_X = 22,    //��������X���ٶ�����
	SPEED_LIMIT_Y,         //��������Y���ٶ�����
	SPEED_LIMIT_Z,         //��������Z���ٶ�����
	SPEED_LIMIT_A,         //��������A���ٶ�����
	SPEED_LIMIT_B,         //��������B���ٶ�����
	SPEED_LIMIT_C,         //��������C���ٶ�����

	SPEED_PLAN_MODE = 30,   //�����ٶȹ滮��ʽ
	INTP_ANGLE_STEP = 31,   //���������ֲ岹�ǶȲ���
    INTP_LEN_STEP = 32,      //���������ֲ岹��С����
	PROGRAM_COORD = 33      //����������ϵ��ʽ
};
#endif

#ifdef USES_GRIND_MACHINE
//ĥ��ͨ������
enum GrindChannel{
	GRIND_MAIN_CHN = 0,     //��ͨ��
	GRIND_SLAVE_CHN,         //��ͨ��
	GRIND_NONE_CHN = 0xFF    //�Ƿ�ͨ��

};

//������ͨ������״̬����
enum MechArmRunState{
	MECH_ARM_IDLE = 0,		//����
	MECH_ARM_RUN = 10,      //����
	MECH_ARM_PAUSE = 20		//��ͣ
};

//�����ϻ�е���쳣״̬
enum MechArmErrState{
	ERR_ARM_NONE = 0,    //����
	ERR_ARM_NO_TRAY,     //δ��⵽����
	ERR_ARM_FETCH_VAC,   //����ȡ��ʧ�ܣ�������쳣
	ERR_ARM_SET_VAC,     //���̷���ʧ�ܣ�������쳣
	ERR_ARM_CORR_VAC,    //��λʧ�ܣ�������쳣
	ERR_ARM_CORR_POS,    //��λʧ�ܣ���λ���ײ���λ
	ERR_ARM_WORK_FETCH_VAC,  //��λȡ��ʧ�ܣ�������쳣
	ERR_ARM_WORK_SET_VAC,    //��λ����ʧ�ܣ�������쳣
	ERR_ARM_CONFIRM_WORK,    //ȷ�Ϲ�λ�Ƿ�����
	ERR_ARM_UP,			 //��е����������λ
	ERR_NEW_TRAY_REQ,    //�����������
	ERR_ARM_AXIS_NO_REF,  //��е��δ����
	ERR_ARM_WORK_UP_LEFT,  //�����λ������������λ
	ERR_ARM_WORK_UP_RIGHT  //�һ���λ������������λ

};

//��е���ֶ�״̬
enum MechArmManualState{
	ARM_MANUAL_IDLE = 0,		//����
	ARM_MANUAL_LEFT_WORK,       //��λ
	ARM_MANUAL_RIGHT_WORK,      //�ҹ�λ
	ARM_MANUAL_REPOS,           //��λƽ̨
	ARM_MANUAL_LEFT_TRAY,       //������
	ARM_MANUAL_RIGHT_TRAY       //������
};

#define ARM_PMC_X  (5)   //��е��X���Ӧ���������
#define ARM_PMC_Y  (6)   //��е��Y���Ӧ���������
#endif

#endif /* INC_GLOBAL_DEFINITION_H_ */
