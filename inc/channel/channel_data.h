/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file channel_data.h
 *@author gonghao
 *@date 2020/04/23
 *@brief ��ͷ�ļ�����ͨ��ʹ�õ�������������
 *@version
 */


#ifndef INC_CHANNEL_CHANNEL_DATA_H_
#define INC_CHANNEL_CHANNEL_DATA_H_

//#include "global_include.h"
#include "compiler_data.h"
#include "compile_message.h"

//��Ҫ���͵�MC��ģ̬��ı�־
const unsigned char McModeFlag[kMaxGModeCount] = {0, 0, 1, 1, 0, 1, 0, 1, 0, 1,     		//0��~9��
										1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 		//10��~19��
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 		//20��~29��
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 		//30��~39��


/**
 * @brief ͨ��״̬����
 */
struct ChannelStatusCollect{
	double rated_manual_speed; 			//�ֶ��ٶȣ���λ��mm/s
	uint32_t workpiece_count;       	//�������������99999999
    uint32_t workpiece_require;         //�������
    uint32_t workpiece_count_total;     //�ܹ�����
    uint32_t machinetime_total;         //�ۼƼӹ�ʱ��
	Mask32 func_state_flags;        	//ϵͳ����״̬��־���������Ρ����Ρ�ѡͣ�������С����ָ��١���������������
	int32_t rated_spindle_speed;           	//�û��趨����ת�٣���λ��ת/��
	uint32_t rated_feed; 				//�û��趨�����ٶȣ���λ��um/s
	uint16_t gmode[kMaxGModeCount];     //Gָ��ģ̬ͨ����ǰģ̬
	uint8_t chn_work_mode;			    //ͨ��ģʽ, �Զ���MDA���ֶ�������
	uint8_t machining_state;       	    //�ӹ�״̬
	uint8_t cur_tool;               //��ǰ���ţ���1��ʼ��ţ�0Ϊ���ᵶ��
	uint8_t preselect_tool_no;		//Ԥѡ������
	uint8_t cur_h_code;				//��ǰH������
	uint8_t cur_d_code;				//��ǰD������
	uint8_t auto_ratio;            //�Զ���������
	uint8_t spindle_ratio;         //���ᱶ��
	uint8_t rapid_ratio;			//���ٽ�������
	uint8_t manual_ratio;          //�ֶ�����
	uint8_t manual_step;           //�ֶ�������0:INVALID;1:X1;2:X10;3:X100;4:X1000
	int8_t cur_manual_move_dir;		//��ǰ�ֶ��ƶ�����0--ֹͣ   1--����   -1--����
	uint8_t cur_axis;				//��ǰ���,ͨ�����
	uint8_t spindle_mode;           //���Ṥ����ʽ��0���ٶȿ��Ʒ�ʽ��1��λ�ÿ��Ʒ�ʽ
	int8_t spindle_dir;				//���᷽��-1����ת��0��ͣת��1����ת
	uint8_t returned_to_ref_point;	//�Ƿ��ѻزο��㣬bit0-bit7�ֱ��ʾ��0-7�Ƿ��ѻزο���
	uint8_t es_button_on;			//��ͣ�����Ƿ��ڰ���״̬��ÿһ��bit��ʾһ����ͣ����
	char cur_nc_file_name[kMaxFileNameLen];	//��ǰ�ӹ��������ļ���

#ifdef USES_SPEED_TORQUE_CTRL
	uint8_t cur_axis_ctrl_mode[kMaxAxisChn];    //��ǰ��ͨ�������ģʽ, 2-�ٶȿ���ģʽ    3-���ؿ���ģʽ
#endif
	uint8_t cur_chn_axis_phy[kMaxAxisChn];	    //��ǰͨ�������Ӧ������������
	int mcode; // @add zk ��ʱû��
};


/**
 * @brief ͨ��ʵʱ״̬,�˽ṹ��״̬����Ҫ��MC��MIʵʱ��ȡ
 */
struct ChannelRealtimeStatus{
	DPointChn cur_pos_machine;   		//��ǰ���ᷴ��λ�ã���е����ϵ����λ��mm
	DPointChn cur_pos_work;			//��ǰ����岹λ�ã���������ϵ����λ��mm
	DPointChn tar_pos_work;			//��ǰ����Ŀ��λ�ã���������ϵ����λ��mm
	int32_t spindle_cur_speed; 				//���ᵱǰת�٣���λ��ת/��
    int32_t cur_feed; 					  //��ǰ�����ٶȣ���λ��um/s
	uint32_t machining_time; 				//�ӹ�ʱ�䣬��λ����
	uint32_t machining_time_remains; 		//ʣ��ӹ�ʱ�䣬��λ����
    uint32_t machining_time_total;          //�ۼƼӹ�ʱ�䣬��λ����
	uint32_t line_no;						//��ǰ�к�
	
	DPointChn cur_feedbck_velocity;			//��ǰ����ʵ���ٶȣ���λ��mm/min
    DPointChn cur_feedbck_torque;			    //��ǰ����ʵ�����أ���λ��0.001�����

    int32_t tap_err;                        //���Թ�˿���(���ֵ) ��λ��um
    int32_t tap_err_now;                    //���Թ�˿���(��ǰֵ) ��λ��um
};

//���ݸ�MC��ģ̬λ�ṹ
struct McModeBits{
	uint32_t mode_g17:2;		//02��
	uint32_t mode_g90:1;		//03��
	uint32_t mode_g94:1;		//05��
	uint32_t mode_g20:1;		//06��
	uint32_t mode_g40:4;		//07��
	uint32_t mode_g73:4;		//09��
	uint32_t mode_g98:1;		//10��
	uint32_t mode_g50:1;		//11��
	uint32_t mode_g60:1;		//13��
	uint32_t mode_g68:1;		//16��
	uint32_t mode_d:7;			//Dֵ
	uint32_t reserved:8;		//����
};
union McModeStatus{
	McModeBits bits;
	uint32_t all;
};

//���ݸ�MC��ͨ�������־�ṹ
struct McErrorBits{
	uint32_t err_soft_limit_neg:1;		//bit0  ��������λ�澯
	uint32_t err_soft_limit_pos:1;		//bit1  ��������λ�澯
	uint32_t err_pos_over:1;		    //bit2  �岹λ�þ��볬��
	uint32_t err_arc_data:1;			//bit3  Բ�����ݴ�
	uint32_t err_cmd_crc:1;				//bit4  �����У���
	uint32_t err_data_crc:1;		    //bit5  ���ݴ���У���
	uint32_t reserved:26;		        //����
};
union McErrorFlag{
	McErrorBits bits;
	uint32_t all;
};


/**
 * @brief MCģ��ͨ��״̬���ݽṹ
 */
struct ChannelMcStatus{
	DPoint intp_pos;   //���ᵱǰ�岹λ��
	DPoint intp_tar_pos;   //���ᵱǰ�岹Ŀ��λ��
	uint32_t cur_line_no;  //��ǰ���������
    int32_t cur_feed;		//��ǰ�����ٶ�,um/s
    int32_t rated_feed;	//��ǰ���������ٶȣ�um/s
	uint32_t axis_over_mask;  	//��岹��λmask
	McModeStatus mc_mode;		  //MC��ǰģ̬
	McErrorFlag mc_error;        //MC����״̬
	uint32_t run_over;
	uint16_t buf_data_count;	//�Զ��ӹ�����������
	uint16_t mda_data_count;	//MDA����������
	uint16_t cur_cmd;		//��ǰ����ָ��
	uint16_t cur_mode;		//��ǰģʽ�� 0--�ֶ�  1--�Զ�   2--MDA
	uint16_t axis_soft_negative_limit;   //�Ḻ������λmask
	uint16_t axis_soft_postive_limit;	//����������λmask
	uint16_t pos_error_mask;   //λ��ָ�����澯����mask
	bool step_over;			//���β岹��λ
	bool auto_block_over;	//AUTO�ֿ�岹��λ
	bool mda_block_over;	//MDA�ֿ�岹��λ
	
#ifdef USES_SPEED_TORQUE_CTRL
    DPoint intp_velocity;   //���ᵱǰ�ٶ�ָ��ֵ  ��λ��mm/min  ( 1��/min)
    DPoint intp_torque;     //���ᵱǰ����ָ��ֵ  ��λ��0.001�����
#endif	
};




/**
 * @brief ͨ���Զ�ģʽ״̬��������״̬���Զ�ģʽ�ķ�READY״̬�г�ʱ����Ҫ������״̬���Է������л��Զ�ģʽ����������ʱ����������֮ǰ��״̬
 */
struct ChannelAutoScene{
	bool need_reload_flag;			//�Ƿ���������Ҫ�ָ�
	uint8_t machining_state;       	//�ӹ�״̬
	uint8_t run_thread_state;		//�����Զ�ģʽ��G���������߳�״̬����
	uint8_t cur_tool;               //��ǰ���ţ���1��ʼ��ţ�0Ϊ���ᵶ��
	uint8_t preselect_tool_no;		//Ԥѡ������
	uint8_t cur_h_code;				//��ǰH������
	uint8_t cur_d_code;				//��ǰD������
	uint8_t spindle_mode;           //���Ṥ����ʽ��0���ٶȿ��Ʒ�ʽ��1��λ�ÿ��Ʒ�ʽ
	int8_t spindle_dir;				//���᷽��-1����ת��0��ͣת��1����ת
	uint16_t gmode[kMaxGModeCount]; //Gָ��ģ̬ͨ����ǰģ̬
	int32_t rated_spindle_speed;           	//�û��趨����ת�٣���λ��ת/��
	uint32_t rated_feed; 				//�û��趨�����ٶȣ���λ��um/s
	McModeStatus mc_mode_exec;      //ָ����Ϣִ��ʱ�޸ĵ�MCģ̬����Ϣ
	DPointChn cur_pos_machine;			//��ǰ�����еλ�ã���е����ϵ����λ��mm

	uint8_t subprog_count;     	//�ӳ���Ƕ�ײ�������,������������
	uint8_t macroprog_count;    //�����Ƕ�ײ����������������ӳ���
	ListNode<RecordMsg *> *p_last_output_msg;   //������͵��˶�����
};

/**
 * @brief ͨ���ӹ���λɨ��ģʽ״̬��¼�������ڼӹ���λ�м�¼ģʽ���м�״̬
 */
struct ChnRestartMode{
	DPointChn pos_target;                   //Ŀ��λ��
	int32_t rated_spindle_speed;           	//�û��趨����ת�٣���λ��ת/��
	int32_t cur_scode;                      //��ǰSָ��
	uint32_t rated_feed; 					//�û��趨�����ٶȣ���λ��um/s
	uint16_t gmode[kMaxGModeCount]; 		//Gָ��ģ̬ͨ����ǰģ̬
	uint8_t cur_tool;               		//��ǰ���ţ���1��ʼ��ţ�0Ϊ���ᵶ��
	uint8_t cur_tcode;		                //��ǰTָ��
	uint8_t cur_h_code;						//��ǰH������
	uint8_t cur_d_code;						//��ǰD������
	int8_t spindle_dir;						//���᷽��-1����ת��0��ͣת��1����ת
	uint8_t sub_prog_call;                  //�ӳ�����ü�¼��������������
};

//�ݲ����ݷ����
struct AxisPcDataAlloc{
	uint8_t axis_index;      //����ţ�0��ʼ
	uint16_t start_index;     //�ݲ�������ʼ�㣬0��ʼ
	uint16_t end_index;       //�ݲ������յ㣬0��ʼ
	uint16_t pc_count;        //�ݲ����ݸ�����˫���ݲ���Ϊ���ݲ�����*2
};
typedef ListBuffer<AxisPcDataAlloc> PcDataAllocList;      //�ݲ����ݷֲ�����

/**
 * @brief ͨ����Ʒ��Χ��X��Y��Z�����������
 */
struct ChnSimulateZone{
	double zone[3][2];    //ά��1:0-X��   1-Y��   2-Z�ᣬ ά��2:0-��Сֵ   1-���ֵ
};

/**
 * @brief ����岹ģʽ���壬��MC�Ķ���ͬ��
 */
enum FiveAxisIntpMode{
	CANCEL_MODE = 0,	//!< CANCEL_MODE
	G43_4_MODE = 5,     //!< G43_4_MODE
	G68_3_MODE = 6,     //!< G68_3_MODE
	G53_1_MODE = 7      //!< G53_1_MODE
};

/**
 * @brief ��λģʽ����
 */
enum RestartMode{
	NOT_RESTART = 0, //!< NOT_RESTART     �Ǽӹ���λ
	NORMAL_RESTART, //!< NORMAL_RESTART   �����ӹ���λ��ÿһ�ж����н���
	FAST_RESTART    //!< FAST_RESTART     ���ټӹ���λ��ֻ�����ӽ�Ŀ���кŵ�1���У�ǰ��Ķ�ֻͳ���к�
};


#ifdef USES_ADDITIONAL_PROGRAM
//���ӳ������ͣ�ǰ�ó��򣬺��ó���
enum AddProgType{
	NONE_ADD = 0,              //�Ǹ��ӳ���
	NORMAL_START_ADD = 1,      //�ӹ�����ǰ�ó���
	CONTINUE_START_ADD = 2,    //�ϵ����ǰ�ó���
	RESET_START_ADD = 3,       //�ӹ���λǰ�ó���
	NORMAL_END_ADD = 11,       //�ӹ��������ó���  M30��M02
	PAUSE_ADD = 12,            //��ͣ���ó���
	RESET_ADD = 13             //��λ���ó���
};
#endif

#endif /* INC_CHANNEL_CHANNEL_DATA_H_ */
