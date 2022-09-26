/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2021/09/07
 *@brief ��ͷ�ļ��������߰뾶�����������
 *@version
 */

#ifndef TOOL_COMPENSATE_H_
#define TOOL_COMPENSATE_H_

#include "list_buffer.h"
#include "compile_message.h"

#define None 12345.123

class ChannelControl;

class Qc{
public:
    int type = 0;
    uint64_t lino = 0;
};

class StraightTraverseQc : public Qc{
public:
    double dx = 0;
    double dy = 0;
    double x = 0;
    double y = 0;
};

class StraightFeedQc : public Qc{
public:
    double dx = 0;
    double dy = 0;
    double x = 0;
    double y = 0;
};

class ArcFeedQc : public Qc{
public:
    int original_turns = 0;
    double end1 = 0;
    double end2 = 0;
    double center1 = 0;
    double center2 = 0;
    int turn = 0;
};



class ToolCompensate {
public:
	ToolCompensate();		//���캯��
	virtual ~ToolCompensate();   //��������

	void SetOutputMsgList(OutputMsgList *output_msg){m_p_output_msg_list = output_msg;}  //����ָ����Ϣ���������ָ��

	void ProcessData(ListNode<RecordMsg *> *node);    //���ݴ�����ں���

	void ResetAllDatas();      // ��λ���ʼ���ڲ�����
	void SetChannelIndex(uint8_t chn_index);    //����ͨ����

    //void setDMode(int Dvalue);
    void setToolRadius(double radius){__cutter_comp_radius = radius;}

    void Reset();


private:   //˽�нӿں���
    void __convert_cutter_compensation(RecordMsg *msg);
    void __convert_cutter_compensation_on(int side);
    void __convert_cutter_compensation_off();
    void __move_endpoint_and_flush(double x, double y);
    void __dequeue_canons();
    void __set_endpoint(double x, double y);

    double __find_turn(double x1, double y1, double center_x, double center_y, int turn, double x2, double y2);

    void __convert_straight(int move);
    void __convert_straight_comp1(int move, double px, double py);
    void __convert_straight_comp2(int move, double px, double py);

    void __convert_arc(int move);
    void __convert_arc2(int move, double end1, double end2, double offset1, double offset2);
    void __convert_arc_comp1(int move, double end_x, double end_y, double offset_x, double offset_y);
    void __convert_arc_comp2(int move, double end_x, double end_y, double offset_x, double offset_y);

    void __find_ends(double *end_x, double *end_y);
    void __checkRadius(int clockWise, double xStart, double yStart, double xEnd, double yEnd, double radius, double *i, double *j);

    void __ARC_FEED(double first_end, double second_end, double first_axis, double second_axis, int rotation, uint64_t lino = 0);
    void __STRAIGHT_FEED(double end_x, double end_y, uint64_t lino = 0);
    void __STRAIGHT_TRAVERSE(double end_x, double end_y, uint64_t lino = 0);
    void __enqueue_STRAIGHT_TRAVERSE(double dx, double dy, double x, double y, uint64_t lino = 0);
    void __enqueue_STRAIGHT_FEED(double dx, double dy, double x, double y, uint64_t lino = 0);
    void __enqueue_ARC_FEED(int original_turns, double end1, double end2, double center1, double center2, int turn, uint64_t lino = 0);
    bool __TOOL_INSIDE_ARC(int side, int turn){
        return (side == 1 and turn > 0) or (side == 2 and turn < 0);
    }

    void __arc_data_ijk(int move, double current_x, double current_y, double end_x, double end_y,
                        double i_number, double j_number, double radius_tolerance, double spiral_abs_tolerance,
                        double spiral_rel_tolerance);

    void __arc_data_comp_ijk(int move, int side, double tool_radius,double current_x,double current_y,
                             double end_x,double end_y,double i_number,double j_number,
                             double radius_tolerance,double spiral_abs_tolerance,double spiral_rel_tolerance);



private:
	uint8_t m_n_channel_index;     //����ͨ����
	ChannelControl *m_p_channel_control;    //ͨ�����ƶ���
	//ToolCompMsgList m_n_toolcomp_list_msg;   //�ӱ���ģ������ı������ݻ���

	OutputMsgList *m_p_output_msg_list;   //�������MC��ָ����Ϣ����
	
	ToolRec m_cur_tool;

    double __current_x = 0;       // ��ǰXʵ��λ��
    double __current_y = 0;     // ��ǰYʵ��λ��
    double __program_x = 0;      // ��ǰX���λ��
    double __program_y = 0;       // ��ǰY���λ��

    double __x_number = None;                  // action�е�x,yλ����Ϣ
    double __y_number = None;
    double __i_number = 0;                 // action�е�i,jλ����Ϣ
    double __j_number = 0;
    double __lo_x = 0;                    // �켣λ��
    double __lo_y = 0;
    double __axis_offset_x = 0.0;              // G92ƫ��λ��
    double __axis_offset_y = 0.0;
    double __g92_offset_x = 0.0;
    double __g92_offset_y = 0.0;

    double __cutter_comp_radius = 0;     // ���߰뾶
    vector<Qc *> __qc;                  // ���ڵ��߲�����queue����
    bool __distance_mode = 0;        // 0-����λ��, 1-���λ��
    bool __cutter_comp_firstmove = true;    // �Ƿ�ΪG41/G42����׸��˶�
    int __cutter_comp_side = 0;             // 0-�޵��߲���, 1-G41����, 2-G42����
    bool __arc_not_allowed = false;          // �Ƿ�������һ����Բ��
    string __errInfo;                       // ������Ϣ

    bool __endpoint_valid = false;              // ���߲��������µĽ���λ���Ƿ���Ч
    double __endpoint_x = 0;
    double __endpoint_y = 0;
    double __center1 = 0;
    double __center2 = 0;
    int __turn;

    DPointChn cur_point;
    bool first_move = true;
    RecordMsg * msg;
};

#endif /* TOOL_COMPENSATE_H_ */
