/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2021/09/07
 *@brief 本头文件包含刀具半径补偿类的声明
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
	ToolCompensate();		//构造函数
	virtual ~ToolCompensate();   //析构函数

	void SetOutputMsgList(OutputMsgList *output_msg){m_p_output_msg_list = output_msg;}  //设置指令消息输出缓冲区指针

	void ProcessData(ListNode<RecordMsg *> *node);    //数据处理入口函数

	void ResetAllDatas();      // 复位或初始化内部数据
	void SetChannelIndex(uint8_t chn_index);    //设置通道号

    //void setDMode(int Dvalue);
    void setToolRadius(double radius){__cutter_comp_radius = radius;}

    void Reset();


private:   //私有接口函数
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
	uint8_t m_n_channel_index;     //所属通道号
	ChannelControl *m_p_channel_control;    //通道控制对象
	//ToolCompMsgList m_n_toolcomp_list_msg;   //从编译模块输入的编译数据缓冲

	OutputMsgList *m_p_output_msg_list;   //待输出至MC的指令消息队列
	
	ToolRec m_cur_tool;

    double __current_x = 0;       // 当前X实际位置
    double __current_y = 0;     // 当前Y实际位置
    double __program_x = 0;      // 当前X编程位置
    double __program_y = 0;       // 当前Y编程位置

    double __x_number = None;                  // action中的x,y位置信息
    double __y_number = None;
    double __i_number = 0;                 // action中的i,j位置信息
    double __j_number = 0;
    double __lo_x = 0;                    // 轨迹位置
    double __lo_y = 0;
    double __axis_offset_x = 0.0;              // G92偏移位置
    double __axis_offset_y = 0.0;
    double __g92_offset_x = 0.0;
    double __g92_offset_y = 0.0;

    double __cutter_comp_radius = 0;     // 刀具半径
    vector<Qc *> __qc;                  // 用于刀具补偿的queue序列
    bool __distance_mode = 0;        // 0-绝对位置, 1-相对位置
    bool __cutter_comp_firstmove = true;    // 是否为G41/G42后的首个运动
    int __cutter_comp_side = 0;             // 0-无刀具补偿, 1-G41补偿, 2-G42补偿
    bool __arc_not_allowed = false;          // 是否允许下一段是圆弧
    string __errInfo;                       // 错误信息

    bool __endpoint_valid = false;              // 刀具补偿计算新的结束位置是否有效
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
