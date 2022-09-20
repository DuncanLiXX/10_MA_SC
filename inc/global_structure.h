/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file global_structure.h
 *@author gonghao
 *@date 2020/03/18
 *@brief ��ͷ�ļ���������ȫ�����ݽṹ����
 *@version
 */

#ifndef INC_GLOBAL_STRUCTURE_H_
#define INC_GLOBAL_STRUCTURE_H_

#include <stdint.h>
#include <cstddef>
#include <sys/mman.h>
#include <string>
#include "hmi_shared_data.h"
#include "list_buffer.h"
#include "global_definition.h"
//#include "global_include.h"


#define PAGE_SIZE ((size_t)getpagesize())
#define PAGE_MASK ((uint64_t)(long)(PAGE_SIZE - 1))

//���ģʽС��ģʽת������
#define BigLittleSwitch16(A)   ((( (ushort)(A) & 0xff00) >> 8)   | \
                                       (( (ushort)(A) & 0x00ff) << 8))

#define BigLittleSwitch32(A)   ((( (unsigned int)(A) & 0xff000000) >> 24) | \
                                       (( (unsigned int)(A) & 0x00ff0000) >> 8)   | \
                                       (( (unsigned int)(A) & 0x0000ff00) << 8)   | \
                                       (( (unsigned int)(A) & 0x000000ff) << 24))

#define BigLittleSwitch64(A)   ((( (unsigned long long)(A) & 0xff00000000000000) >> 56) | \
					      	  (( (unsigned long long)(A) & 0x00ff000000000000) >> 40) | \
							  (( (unsigned long long)(A) & 0x0000ff0000000000) >> 24) | \
                                       (( (unsigned long long)(A) & 0x000000ff00000000) >> 8)   | \
                                       (( (unsigned long long)(A) & 0x00000000ff000000) << 8)   | \
                                       (( (unsigned long long)(A) & 0x0000000000ff0000) << 24)  | \
                                       (( (unsigned long long)(A) & 0x000000000000ff00) << 40)  | \
                                       (( (unsigned long long)(A) & 0x00000000000000ff) << 56))



/**
 * @brief ���ڱ���ϵͳȫ��״̬
 */
struct SystemState {
	bool system_ready;             	//ϵͳ��������ʼ����ɱ�־����ʼ��Ϊfalse��������ģ����س�ʼ����ɸ�Ϊtrue
	uint8_t module_ready_mask;      //��ģ�����mask��ÿ��ģ��ռһ��bit
	bool system_quit;              	//ϵͳ�˳���־���ӵ��˳��ź��Ǵ˱�־��true
	bool hmi_comm_ready;           	//��HMI��ͨѶ׼����
//	bool hmi_ready;                //HMI����
	uint32_t hmi_alarm_code;        //HMI�澯��
	bool eth0_running;             //�����Ƿ����
	uint8_t system_boot_stage;     	//ϵͳ��������,��־ϵͳ�����ĵ�ǰ�׶�
	char hmi_host_addr[16];        //��ǰ���ӵ�HMI����IP��ַ
	char local_host_addr[16];      //����IP��ַ

};

/**
 * @brief CPUռ���ʽṹ��
 */
struct CPULoad {
	char name[20];   //cpu����
	uint32_t user;    //�û�̬��ʱ
	uint32_t nice;    //niceֵΪ���̺߳�ʱ
	uint32_t system;  //ϵͳ��ʱ
	uint32_t idle;    //���к�ʱ
};

/**
 * @brief �ڴ�ռ���ʽṹ��
 */
struct MemLoad {
	char name1[20];
	char name2[20];
	uint32_t total;
	uint32_t free;
	uint32_t buffer;
	uint32_t cached;
};

/**
 * @brief CoordInfo��������ϵ��Ϣ
 */
struct CoordInfo { //����ϵ��Ϣ
	CoordInfo();
	void Init();
	void Reset();
	bool CheckMask(uint8_t index);
	void SetMask(uint8_t index, uint8_t value = 1); //��������mask �ĵ�indexλΪ valueֵ
//	double surface_vector[3];	//��ѡƽ��ķ�����
	double build_coord_dest[kMaxAxisChn]; //���������������������ϵʱ��������ݣ�G92/G52/G10����
//	double wcs_offset_differ[kMaxAxisChn]; //��ǰ��������ϵ�����G54��G59��ƫ�ƣ���λ��mm
	uint8_t surface_flag; 	//02�飬���ڱ�ʶѡ���ƽ�棬170:XY* 180:ZX* 190:YZ*
	uint8_t coord_id; 	//14�飬��������ϵѡ��0:G53��������ϵ 1-6:G54*-G59 ��������ϵ G5401~G5499
	uint8_t build_coord_flag;   //��������ϵ���͡�0:����������ϵ��1��G92��2��G52 3:G10 4��G92.1��λ
	uint8_t build_wcs_mask; 	//���G92/G52/G10/G92.1ʹ�õ���
};

/**
 * @brief spartan6 ���ݽṹ�����ڶ�ȡ�ļ�д��pl
 */
union Sp6Data{
	uint32_t data_32[4];
	char buffer[16];
};

/**
 * @brief MI�Ŀ����ֶ���
 */
struct MiCtrlBits{
	uint64_t AXIS_ON_FLAG:1;			//��ʹ��
	uint64_t MOTOR_ON_FLAG:1;
	uint64_t MOTION_LEFT_LMT_FLAG:1;
	uint64_t HW_LMT_EN_FLAG:1;
	uint64_t HW_LMT_SENSE_FLAG:1;
	uint64_t SW_LMT_EN_FLAG:1;
	uint64_t HW_LMT_EN_ONCE_FLAG:1;
	uint64_t RESERVED_1:1;
	uint64_t RUN_DIR_FLAG:1;
	uint64_t SRC_DEV_FLAGS:3;
	uint64_t CAPT_COM_FLAG:1;
	uint64_t CAPT_EN_FLAG:2;
	uint64_t CAPT_SIGNAL_FLAGS:2;
	uint64_t MOTION_RIGHT_LMT_FLAG:1;
	uint64_t PLC_INTPLA_PIMIT:1;
	uint64_t MANU_POS_UPT_FLAG:1;
	uint64_t CTRL_MODE_FLAG:1;
	uint64_t PLAN_MODE_FLAGS:2;
	uint64_t AUTO_MDI_INTPLA_PIMIT:1;
	uint64_t MANU_INTPLA_PIMIT:1;
	uint64_t PULSE_DIR_FLAG:1;
	uint64_t START_LMT_FLAG:1;
	uint64_t AUTO_SMOOTH_STOP_FLAG:1;
	uint64_t MDI_INTP_DIR_FLAG:1;
	uint64_t AUTO_INTP_DIR_FLAG:1;
	uint64_t RESERVED_2:1;
	uint64_t AXIS_OFF_CHECK_FLAG:1;
	uint64_t RESERVED_3:32;

};

/**
 * @brief MI�����ּĴ�������
 */
union MiCtrlReg{
	uint64_t all;
	MiCtrlBits bits;
};

#ifdef USES_PMC_2_0
/**
 * @brief SD-LINK��վ�豸��Ϣ�ṹ
 */
struct BdioDevInfo{
    int8_t group_index;   //�豸��
    int8_t device_type;   //�豸���ͺ� 1.SA1(DI/DO 9/4), 3.SC1(DI/DO 16/16), 4.SD1(DI/DO 12/8), 5.SE1(DI/DO 7/8)
    int8_t base_index;    //�����(Ԥ��)
    int8_t slot_index;    //��λ��(Ԥ��)
    int16_t in_bytes;     //�����ֽ���
    int16_t out_bytes;    //����ֽ���
    int8_t input_start;   //������ʼ��
    int8_t output_start;  //�����ʼ��
    int8_t handwheel_map; //����ӳ��
    std::string info_name;//�豸����
};

/**
 * @brief SD_LINK��չ�忨���
 */
struct SDLINK_SPEC {
    int8_t  inBytes;
    int8_t  outBytes;
    std::string info_name;
    bool    withHandWheel;
    SDLINK_SPEC(std::string name, int in, int out, bool handWheel) :
        inBytes(in), outBytes(out), info_name(name), withHandWheel(handWheel) {}
};

#else
/**
 * @brief SD-LINK��վ�豸��Ϣ�ṹ
 */
struct BdioDevInfo{
	int slave_index;    //��վ�ţ���1��ʼ
	int device_type;      //�豸���ͺ�
	int in_bytes;		//�����ֽ�
	int out_bytes;
};
#endif
typedef ListBuffer<BdioDevInfo> BdioDevList;   //SD-LINK��վ����

#ifdef USES_GRIND_MACHINE
//��е�۲���
struct ParamMechArm{
	int row_count;    //����
	int col_count;    //����
	double pos_tray_base_x[2];    //���̻�׼λ��X��    0-����   1-�ӻ�
	double pos_tray_base_y[2];    //���̻�׼λ��Y��   0-����   1-�ӻ�
	double dis_tray_y;     //�����м��
	double dis_tray_x;     //�����м��
	double pos_corr_rough_x;   //�ֶ�λλ��X��
	double pos_corr_rough_y;   //�ֶ�λλ��Y��
	double pos_corr_fine_x;	   //����λλ��X��
	double pos_corr_fine_y;    //����λλ��Y��
	double pos_work_x[2];		//�ӹ�λ��X    0-����   1-�ӻ�
	double pos_work_y[2];       //�ӹ�λ��Y    0-����   1-�ӻ�
	double dis_mech_arm;    //���Ҽ�צ���
	double speed;           //��е���ƶ��ٶ�
	int delay_time_vac;       //�������ʱ,��λ��ms   D408
	int wait_overtime;        //�ȴ���ʱʱ�䣬��λ��ms   D416
	int work_vac_check_delay;    //��λ�����ȷ����ʱ����λ��ms    D424
	int work_mode;          //����ģʽ��1--����ģʽ    0--˫��ģʽ    D412
	int corr_pos_count;      //��λ����      D420
};

//��е��״̬
struct StateMechArm{
	uint8_t run_flag;     //���б�־   0--����   10--����   20--��ͣ
	uint8_t cur_chn;      //��ǰ������ͨ��     0-��ͨ��       1-��ͨ��
	bool has_material;    //��ǰ�Ƿ����    true--����    false--��
	bool repos;           //��ǰ��ë���Ƿ��Ѿ���ɶ�λ
	uint8_t cur_step;     //��ǰ����
	uint16_t total_count;   //�����ܸ���
	uint16_t cur_index[2];     //��ǰʰȡ����
	uint16_t cur_finished_count[2];  //��ǰ���̳�Ʒ����
	bool work_material[2];     //��λ�Ƿ�����
	bool has_finished_material;  //�Ƿ���Ҫ�ų�Ʒ

	bool is_exit_inter_area;     //�˳����������־


	pthread_mutex_t mutex_change_run;    //����״̬�л�������
	double x_target_pos;    //��ǰX��Ŀ��λ��
	double y_target_pos;    //��ǰY��Ŀ��λ��
	int corr_index;         //�Ѷ�λ����
	struct timeval time_start_arm;   //�ȴ���ʼʱ��
	struct timeval time_start_arm_2;  //�ȴ���ʼʱ��2
	void *msg;      //Mָ����Ϣָ��
	int err_state;    //��е���쳣״̬
};
#endif

#endif /* INC_GLOBAL_STRUCTURE_H_ */
