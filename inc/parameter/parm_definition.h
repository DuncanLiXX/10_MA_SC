/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file parm_definition.h
 *@author gonghao
 *@date 2020/05/06
 *@brief ��ͷ�ļ�Ϊ��������ṹ������
 *@version
 */

#ifndef INC_PARAMETER_PARM_DEFINITION_H_
#define INC_PARAMETER_PARM_DEFINITION_H_
#include "global_include.h"

const int kWorkCoordCount = 7;		//��������ϵ������G54~G59������һ����������ϵ

//��������ö�ٶ���
enum ParmType{
	SYS_CONFIG = 1,    //ϵͳ����  1
	CHN_CONFIG,			//ͨ������  2
	AXIS_CONFIG,		//������   3
	TOOL_OFFSET_CONFIG,		//����ƫ������  4
	TOOL_POT_CONFIG,		//����λ������  5
	COORD_CONFIG,		//����ϵ���� 6
	EX_COORD_CONFIG,	//��չ����ϵ���� 7
	FIVE_AXIS_CONFIG,	//�������� 8

	PROCESS_PARAM,         //���ղ��� 9

	GRIND_CONFIG = 10,		//ĥ��ר������

	PITCH_COMP_DATA = 13,    //�ݲ�����
	IO_REMAP_DATA = 14,      //IO��ӳ������
    HANDWHEEL_MAP = 15,      //����ͨ��ӳ��
    FIVEAXIS_V2_CONFIG = 16, // ����������


	CHN_STATE_SCENE = 100		//ͨ����ǰ״̬

};

/**
 * @brief SCģ��ϵͳ��������
 */
struct SCSystemConfig{
	uint8_t cnc_mode;			        //�豸����		//����Ȩ������������޸�
	uint8_t max_chn_count;				//���ͨ����	//����Ȩ������������޸�
	uint8_t max_axis_count;				//�������		//����Ȩ������������޸�
	uint8_t chn_count;					//ͨ������
	uint8_t axis_count;			     	//ϵͳ����������
	uint8_t axis_name_ex;				//��������չ�±�    0--�ر�		1--��
	uint8_t fix_ratio_find_ref;			//�زο���ʱ���ʹ̶�	0--�ر�		1--��
	//uint8_t pc_type;					//�ݾಹ����ʽ		0--�����ݲ�  1--˫���ݲ�
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

	uint8_t alarm_temperature;			//����澯�¶�
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
 * @brief SCģ��ͨ����������
 */
struct SCChannelConfig{
	uint8_t chn_index;  		//ͨ��������
	uint8_t chn_axis_count;     //ͨ��������
	uint8_t chn_group_index;    //ͨ��������ʽ����
//	uint8_t chn_axis_x;			//������X��Ӧ������������
//	uint8_t chn_axis_y;			//������Y��Ӧ������������
//	uint8_t chn_axis_z;			//������Z��Ӧ������������
	uint8_t chn_axis_phy[kMaxAxisChn];	//���Ӧ������������

	uint8_t chn_axis_name[kMaxAxisChn];		//������
	uint8_t chn_axis_name_ex[kMaxAxisChn];	//�������±�

	uint8_t intep_mode;				//�岹ģʽ   0��XYZ����岹  1��������岹
	uint8_t intep_cycle;			//�岹����	0-8:125us/250us/500us/1ms/2ms/4ms/5ms/8ms/10ms
	uint16_t chn_precision;			//�ӹ����ȣ���λ��΢��
	uint8_t chn_look_ahead;			//ǰհ���� 	0--�ر�   1--��
	uint8_t chn_feed_limit_level;	//�ӹ��ٶȵ����ȼ�  0--�ر�   1~10��ʾ�ȼ�1~�ȼ�10
	uint8_t zmove_stop;				//�µ�׼ͣ  0--�ر�    1--��
	uint8_t corner_stop_enable;		//�ս�׼ͣ���ܿ���
	uint8_t corner_stop;			//�ս�׼ͣ  0-180��
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
    uint8_t rst_mode;             //��λʱ�Ƿ�����������  0:������ 1:����
    uint32_t g01_max_speed;         //G01��߽����ٶ� ��λ��mm/min
    uint16_t mpg_level3_step;       //����3�����Զ��岽�� ��λ��um
    uint16_t mpg_level4_step;       //����4�����Զ��岽�� ��λ��um

    double G73back;		// G73���˾���
    double G83back;		// G83���˾���

    uint8_t tool_number;    // ������
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
 * @brief SCģ������������
 */
struct SCAxisConfig{
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
	double move_pr;							//ÿת�ƶ�������˿���ݾ�    ��λ��mm(deg)
	uint8_t motor_dir;						//�����ת����    0--��ת    1--��ת
    uint8_t feedback_mode;					//�����������    0--����ʽ(��Ȧ����ʽ)    1--����ʽ(��Ȧ����ʽ)


    uint8_t ret_ref_mode;					//0--�޵������  1--�е������
    uint8_t absolute_ref_mode;              //����ʽ������㷽ʽ 0--�����ǵ��趨��ʽ    1--�޵�����㷽ʽ
	uint8_t ret_ref_dir;					//�زο��㷽��		0--����    1--����
	uint8_t ret_ref_change_dir;				//�زο��㻻��      0--����    1--ͬ��
	uint8_t ref_signal;						//�ο����ź�����     0--���ź�    1--Z�ź�
	uint8_t ret_ref_index;					//�زο���˳��		0-11:��һ~��ʮ��
	uint8_t ref_base_diff_check;			//�־���׼λ��ƫ����		0--�ر�   1--��
	double ref_base_diff;					//�־���׼λ��ƫ��		��λ��mm��deg��
	int64_t ref_encoder;					//��������ֵ������ʽ��������Ч
    double ref_offset_pos;                //�زο�����ƫ���� ��λ:mm
    double ref_z_distance_max;            //����Z��������ƶ�����

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
    //int16_t backlash_forward;					//�������϶����λ��um(deg)->mm(deg)
    //int16_t backlash_negative;					//�������϶����λ��um(deg)->mm(deg)
    double backlash_forward;					//�������϶����λ��mm(deg)
    double backlash_negative;					//�������϶����λ��mm(deg)
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
	uint8_t fast_locate;							//���ٶ�λ    0--�ر�   1--��
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
    uint32_t sync_err_max_pos;					//λ��ͬ��������ֵ	��λ��um
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

    uint8_t pmc_g00_by_EIFg;                //PMC�ᱶ�ʿ���
    uint16_t pmc_min_speed;                 //��СPMC�ƶ��ٶ�
    uint16_t pmc_max_speed;                 //���PMC�ƶ��ٶ�

    int ref_complete;                       //�ο����Ƿ���
    bool init_backlash_dir;                 //�����϶��ʼ������

    uint8_t decelerate_numerator;           //���ٱ�������
    uint8_t decelerate_denominator;         //���ٱ�����ĸ
};

/**
 * @brief SCģ�鵶��ƫ����������
 */
struct SCToolOffsetConfig{
#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
	double geometry_comp_basic[3];                     //��׼��ƫ��
#endif
	double geometry_compensation[kMaxToolCount][3];    //���߼���ƫ�ã��������Ȳ���
	double geometry_wear[kMaxToolCount];				//���߳���ĥ�𲹳�
	double radius_compensation[kMaxToolCount];			//���߰뾶����
	double radius_wear[kMaxToolCount];					//���߰뾶ĥ�𲹳�
};

/**
 * @brief SCģ�鵶λ�������ݣ������뵼��ӳ���
 */
struct SCToolPotConfig{
//	uint8_t tool_index;		//����
	uint8_t tool_type[kMaxToolCount];		//��������
	uint8_t huge_tool_flag[kMaxToolCount];           //�Ƿ�󵶱����󵶱���ζ��һ�ѵ���Ҫռ������λ
	uint8_t tool_pot_index[kMaxToolCount];			//���׺�
	int tool_life_max[kMaxToolCount];								//������������λ��min
	int tool_life_cur[kMaxToolCount];								//��ʹ����������λ��min
	int tool_threshold[kMaxToolCount];                             //��������Ԥ����ֵ
    uint8_t tool_life_type[kMaxToolCount];                          //���߼������ͣ�0--�ر�,1--����������2--����ʱ�䣬3--��������
};


/**
 * @brief ���������ò���
 */
struct SCFiveAxisV2Config
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
 * @brief ��������ϵ����
 */
struct SCCoordConfig{
	double offset[kMaxAxisChn];    //�Ṥ������ϵƫ��
};

/**
 * @brief ��������ϵ�������ݽṹ
 */
struct CoordUpdate{
	uint8_t chn_index;  //ͨ����
	uint8_t coord_index;	//����ϵ����  0--ͨ��ƫ��  1-6:G54~G59
	SCCoordConfig value;
};

/**
 * @brief HMIģ�鵶��ƫ����������
 */
//struct HmiToolOffsetConfig{
//	double geometry_compensation[3];    //���߼���ƫ�ã��������Ȳ���
//	double geometry_wear;				//���߳���ĥ�𲹳�
//	double radius_compensation;			//���߰뾶����
//	double radius_wear;					//���߰뾶ĥ�𲹳�
//};

/**
 * @brief ����ƫ�ø������ݽṹ
 */
struct ToolOffsetUpdate{
	uint8_t chn_index;  //ͨ����
	uint8_t tool_offset_index;	//��ƫ����  0--H1
	HmiToolOffsetConfig value;
};

/**
 * @brief ���ݲ����ݱ�
 */
struct AxisPitchCompTable{
	double *pc_table[kMaxAxisNum];   //����洢����̬���䣬�������ݲ��������ݲ�
};

/**
 * @brief ����ֵ�����壬���ڱ����������ʱ�Ĳ���ֵ
 */
union ParamValue{
	uint8_t value_uint8;
	int8_t value_int8;
	uint16_t value_uint16;
	int16_t value_int16;
	uint32_t value_uint32;
	int32_t value_int32;
	uint64_t value_uint64;
	int64_t value_int64;
	double value_double;
};

/**
 * @brief ���ڱ�������޸�����
 */
struct ParamUpdate{
	uint32_t param_no;
	uint8_t param_type;
	uint8_t chn_index;
	uint8_t axis_index;
	uint8_t value_type;
	ParamValue value;
};

/**
 * @brief ���ڱ��湤����ز����޸�����
 */
struct ProcParamUpdate{
	uint8_t group_index;    //���ղ�����
	ParamUpdate param_data; //�����²�������
};

//IO��ӳ������ݽṹ����
struct IoRemapInfo{
	uint8_t  iotype;	//0������X 1�����Y
	uint16_t addr;	    //�ֽڵ�ַ������X10.2���˴���10
	uint8_t  bit;		//λ��ַ��0-7
	uint8_t  addrmap;	//�Ƿ���ӳ��Ϊ�µĵ�ַ 0����ӳ�䣬1��ӳ��
	uint16_t newaddr;	//���ֽڵ�ַ
	uint8_t  newbit;	//��λ��ַ��0-7
	uint8_t  valtype;	//ֵ���ͣ� 0��ԭֵ��1������ 2������ 3������
};

//����ͨ��ӳ��
struct HandWheelMapInfo {
    uint8_t devNum = 0;
    uint8_t wheelID = 0;
    uint8_t channelMap = 0;
    uint8_t reserve = 0;
    //std::string devName = "";
    char devName[16] = { };
    HandWheelMapInfo() = default;
    HandWheelMapInfo(int dNum, int wId, int cMap, std::string dName):
        devNum(dNum), wheelID(wId), channelMap(cMap), reserve(0)
    {
        size_t len = sizeof(devName);
        memset(devName, 0, len);
        int sizeCpy = dName.size();
        if (dName.length() >= len)
            sizeCpy = len - 1;
        memcpy(devName, dName.c_str(), sizeCpy);
    }

    bool operator ==(const HandWheelMapInfo &lhs) const{
        return (devNum == lhs.devNum) && (wheelID == lhs.wheelID) && (std::string(devName) == std::string(lhs.devName));
    }
};
typedef std::vector<HandWheelMapInfo> HandWheelMapInfoVec;

//�������ͨ��������
struct ChnProcParamGroup{
	ProcessParamChn chn_param[kMaxProcParamCount];
};

//��������������
struct AxisProcParamGroup{
	ProcessParamAxis axis_param[kMaxProcParamCount];
};

typedef ListBuffer<ParamUpdate> UpdateParamList;   //����Ч��������
typedef ListBuffer<CoordUpdate> UpdateCoordList;		//����Ч�Ĺ�������ϵ����
typedef ListBuffer<ToolOffsetUpdate> UpdateToolOffsetList;     //����Ч�ĵ���ƫ�ö���

typedef ListBuffer<ProcParamUpdate> UpdateProcParamList;   //����Ч������ز�������


typedef ListBuffer<IoRemapInfo> IoRemapList;      //IO�ض������ݶ���


#endif /* INC_PARAMETER_PARM_DEFINITION_H_ */
