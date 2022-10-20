/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.h
 *@author gonghao
 *@date 2021/09/07
 *@brief 本头文件包含刀具半径补偿类的实现
 *@version
 */

#include "channel_engine.h"
#include "channel_control.h"
#include "tool_comp_data.h"
#include "tool_compensate.h"

void ToolCompensate::__ARC_FEED(double first_end, double second_end,
                                double first_axis, double second_axis,
                                int rotation, uint64_t lino)
{
    DPointChn target;
    double feed;
    uint32_t mask;
    switch(msg->GetMsgType()){
        case LINE_MSG:{
            LineMsg * old_msg = (LineMsg *)msg;
            target = old_msg->GetTargetPos();
            feed = old_msg->GetFeed();
            mask = old_msg->GetAxisMoveMask();
            break;
        }
        case RAPID_MSG: {
            RapidMsg *old_msg = (RapidMsg *) msg;
            target = old_msg->GetTargetPos();
            mask = old_msg->GetAxisMoveMask();
            break;
        }
        case ARC_MSG:{
            ArcMsg * old_msg = (ArcMsg *)msg;
            target = old_msg->GetTargetPos();
            feed = old_msg->GetFeed();
            mask = old_msg->GetAxisMoveMask();
            break;
        }
    }

    //m_p_output_msg_list->Append(msg);
    //return ;

    target.SetAxisValue(0, first_end);
    target.SetAxisValue(1, second_end);
    target.SetAxisValue(2, cur_point.GetAxisValue(2));

    DPointChn center = target;
    center.SetAxisValue(0, cur_point.GetAxisValue(0) + first_axis);
    center.SetAxisValue(1, cur_point.GetAxisValue(1) + second_axis);
    double radius = sqrt(first_axis*first_axis + second_axis+second_axis);

    /**************************/
	int8_t major_flag = 1;  //优弧标志， 1--劣弧    -1--优弧
	int8_t circle_flag = 0;	//整圆标志， 0--圆弧    1--整圆
	int8_t dir_flag = -1;  //方向标志，-1:clockwise,1:anticlockwise

    int code;
	// 圆弧方向
	if(rotation < 0){
    	code = G02_CMD;
    	dir_flag = -1;
    }else{
    	code = G03_CMD;
    	dir_flag = 1;
    }

	if(cur_point == target){
		//判断是否整圆
		circle_flag = 1;    //IJK编程，起点和终点重合则为整圆

		major_flag = -1;    //优弧
	}
	else{

		DPlane cen = Point2Plane(center, PLANE_XY);
		DPlane tar = Point2Plane(target, PLANE_XY);

		//判断优劣弧
		DPlane src = Point2Plane(cur_point, PLANE_XY);
		double angle_start = GetVectAngle(src, cen);
		double angle_end = GetVectAngle(tar, cen);
		double angle = (angle_end - angle_start) * dir_flag;
		if(angle < 0)
			angle += 2*M_PI;
		if(angle > M_PI)
			major_flag = -1;   //大于180度，优弧
	}
	/*************************************/
    ArcMsg * new_msg = new ArcMsg(code, cur_point, target, center, radius,
    		feed, mask, dir_flag, major_flag, circle_flag);
    new_msg->SetLineNo(msg->GetLineNo());
    new_msg->SetPlaneMode(PLANE_XY);   //设置平面模态

    RecordMsgFlag flags;
    flags.all = 0;
    new_msg->SetFlags(flags);
    //((ArcMsg *)msg)->arc_id = 100;
    //m_p_output_msg_list->Append(msg);
    new_msg->arc_id = 200;
    m_p_output_msg_list->Append(new_msg);
    cur_point = target;
}

void ToolCompensate::__STRAIGHT_FEED(double end_x, double end_y, uint64_t lino) {

	LineMsg * old_msg = (LineMsg *)msg;
    DPointChn target = old_msg->GetTargetPos();
    target.SetAxisValue(0, end_x);
    target.SetAxisValue(1, end_y);
    LineMsg * new_msg = new LineMsg(cur_point, target, old_msg->GetFeed(), old_msg->GetAxisMoveMask());


    RecordMsgFlag flags;
    flags.all = 71;
    new_msg->SetFlags(flags);

    lino==0?new_msg->SetLineNo(old_msg->GetLineNo())
    		:new_msg->SetLineNo(lino);

    printf("straight feed end point : %lf  %lf %d\n", end_x, end_y, old_msg->GetFlags());
    m_p_output_msg_list->Append(new_msg);
    cur_point = target;
}

void ToolCompensate::__STRAIGHT_TRAVERSE(double end_x, double end_y, uint64_t lino) {
    RapidMsg * old_msg = (RapidMsg *)msg;
    DPointChn target = old_msg->GetTargetPos();
    target.SetAxisValue(0, end_x);
    target.SetAxisValue(1, end_y);
    RapidMsg * new_msg = new RapidMsg(cur_point, target, old_msg->GetAxisMoveMask());


    RecordMsgFlag flags;
    flags.all = 71;
    new_msg->SetFlags(flags);

    lino==0?new_msg->SetLineNo(old_msg->GetLineNo())
    		:new_msg->SetLineNo(lino);

	printf("straight feed end point : %lf  %lf %d\n", end_x, end_y, old_msg->GetFlags());
    m_p_output_msg_list->Append(new_msg);
    cur_point = target;
}

void ToolCompensate::__enqueue_STRAIGHT_TRAVERSE(double dx, double dy, double x, double y, uint64_t lino) {
    StraightTraverseQc * qc = new StraightTraverseQc();
    qc->dx = dx;
    qc->dy = dy;
    qc->x = x;
    qc->y = y;
    qc->type = 0;
    __qc.push_back(qc);
}

void ToolCompensate::__enqueue_STRAIGHT_FEED(double dx, double dy, double x, double y, uint64_t lino) {
    StraightFeedQc *qc = new StraightFeedQc();
    qc->dx = dx;
    qc->dy = dy;
    qc->x = x;
    qc->y = y;
    qc->type = 1;
    __qc.push_back(qc);
}

void ToolCompensate::__enqueue_ARC_FEED(int original_turns, double end1, double end2,
                                        double center1, double center2,int turn, uint64_t lino)
{
    ArcFeedQc * qc = new ArcFeedQc();
    qc->original_turns = original_turns;
    qc->end1 = end1;
    qc->end2 = end2;
    qc->center1 = center1;
    qc->center2 = center2;
    qc->turn = turn;
    qc->type = 2;
    __qc.push_back(qc);
}

/**
 * @brief 构造函数
 */
ToolCompensate::ToolCompensate() {
	// TODO Auto-generated constructor stub

	this->m_p_output_msg_list = nullptr;
	this->m_p_channel_control = nullptr;

	this->m_n_channel_index = 0;

	this->m_cur_tool.G41G42dir  = 40;
	this->m_cur_tool.D = 0;
	this->m_cur_tool.Radius = 0;
	this->m_cur_tool.plane = PLANE_XY;
}

/**
 * @brief 析构函数
 */
ToolCompensate::~ToolCompensate() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief 设置通道号
 * @param chn_index : 通道号
 */
void ToolCompensate::SetChannelIndex(uint8_t chn_index){
	this->m_n_channel_index = chn_index;
	this->m_p_channel_control = g_ptr_chn_engine->GetChnControl(chn_index);
}


void ToolCompensate::Reset(){
	__current_x = 0;       // 当前X实际位置
	__current_y = 0;     // 当前Y实际位置
	__program_x = 0;      // 当前X编程位置
	__program_y = 0;       // 当前Y编程位置

	__x_number = None;                  // action中的x,y位置信息
	__y_number = None;
	__i_number = 0;                 // action中的i,j位置信息
	__j_number = 0;
	__lo_x = 0;                    // 轨迹位置
	__lo_y = 0;

	__cutter_comp_radius = 0;     // 刀具半径
	for(Qc * qc : __qc){
		delete qc;
	}
	__qc.clear();
	__cutter_comp_firstmove = true;    // 是否为G41/G42后的首个运动
	__cutter_comp_side = 0;             // 0-无刀具补偿, 1-G41补偿, 2-G42补偿
	__arc_not_allowed = false;          // 是否允许下一段是圆弧
	__errInfo.clear();                  // 错误信息

	__endpoint_valid = false;              // 刀具补偿计算新的结束位置是否有效
	__endpoint_x = 0;
	__endpoint_y = 0;
	__center1 = 0;
	__center2 = 0;

	//DPointChn cur_point;
	first_move = true;
}

/**
 * @brief 数据处理入口函数
 * @param node : 编译数据节点指针
 * 在 compile.runmsg 中已经处理了 G90 G91 不用考虑增量情况 所有计算都是绝对坐标值
 */
void ToolCompensate::ProcessData(ListNode<RecordMsg *> *node){

	msg = node->data;
	// 记录行号
    /*switch(msg->GetMsgType()){
        case LINE_MSG:{
            LineMsg * line_msg = (LineMsg *) msg;

            if(first_move){
                cur_point = line_msg->GetSourcePos();
                first_move = false;
            }

            __x_number = line_msg->GetTargetPos().GetAxisValue(0);
            __y_number = line_msg->GetTargetPos().GetAxisValue(1);

            __convert_straight(1);   // G01  move : 1   G00  move : 0

            if(!__errInfo.empty()){
            	std::cout << "---------------------------> compensate error: " <<  __errInfo << std::endl;
            }

            return;
        }
        case RAPID_MSG:{
            RapidMsg * rapid_msg = (RapidMsg *) msg;
            if(first_move){
                cur_point = rapid_msg->GetSourcePos();
                first_move = false;
            }

            __x_number = rapid_msg->GetTargetPos().GetAxisValue(0);
            __y_number = rapid_msg->GetTargetPos().GetAxisValue(1);
            __convert_straight(0);

            if(!__errInfo.empty()){
				std::cout << "---------------------------> compensate error: " <<  __errInfo << std::endl;
			}

            return;
        }
        case ARC_MSG:{

        	ArcMsg * arc_msg = (ArcMsg *) msg;

            if(first_move){
                cur_point = arc_msg->GetSourcePos();
                first_move = false;
            }

            if(arc_msg->getR() > 0.0001){
                double cx = __x_number;
                double cy = __y_number;
                __x_number = arc_msg->GetTargetPos().GetAxisValue(0);
                __y_number = arc_msg->GetTargetPos().GetAxisValue(1);
                double i, j;
                __checkRadius(arc_msg->getDirect(), cx, cy, __x_number, __y_number, arc_msg->getR(), &i, &j);
                __i_number = i;
                __j_number = j;
            }else{
                __x_number = arc_msg->GetTargetPos().GetAxisValue(0);
                __y_number = arc_msg->GetTargetPos().GetAxisValue(1);
                __i_number = arc_msg->getI();
                __j_number = arc_msg->getJ();
            }

            if(arc_msg->getDirect() == -1)
                __convert_arc(2);
            if(arc_msg->getDirect() == 1)
                __convert_arc(3);

            if(!__errInfo.empty()){
            	std::cout << "compensate error: " <<  __errInfo << std::endl;
			}

            return;
        }
        case COMPENSATE_MSG: {
            __convert_cutter_compensation(msg);
            CompensateMsg * comp_msg = (CompensateMsg *) msg;

            break;
        }

        default:{
        	__dequeue_canons();
        	break;
        }
    }*/

    this->m_p_output_msg_list->Append(node);
}

void ToolCompensate::ResetAllDatas(){

}

void ToolCompensate::__convert_cutter_compensation(RecordMsg *msg) {
    CompensateMsg * comp_msg = (CompensateMsg *) msg;

    if(comp_msg->GetGCode() == G40_CMD){
        __convert_cutter_compensation_off();
    }else if(comp_msg->GetGCode() == G41_CMD){
        __convert_cutter_compensation_on(1);
    }else{
        __convert_cutter_compensation_on(2);
    }
}

void ToolCompensate::__convert_cutter_compensation_on(int side) {
    this->__cutter_comp_side = side;
}

void ToolCompensate::__convert_cutter_compensation_off() {
    if(__cutter_comp_side != 0 and __cutter_comp_radius > 0.0){
        __move_endpoint_and_flush(__current_x, __current_y);
    }

    if(!__errInfo.empty()) return;

    __dequeue_canons();

    if(!__errInfo.empty()) return;

    __program_x = __current_x;
    __program_y = __current_y;
    __arc_not_allowed = true;
    __cutter_comp_side = 0;
    __cutter_comp_firstmove = true;
}

void ToolCompensate::__move_endpoint_and_flush(double x, double y) {
    if(__qc.empty()) return;

    //# there may be several moves in the queue, and we need to
    //# change all of them.  consider moving into a concave corner,
    //# then up and back down, then continuing on.  there will be
    //# three moves to change.
    for(Qc * q : __qc){
        if(q->type == 2){
            ArcFeedQc * qc = (ArcFeedQc *) q;
            double r1 = hypot(qc->end1 - qc->center1, qc->end2 - qc->center2);
            int l1 = qc->original_turns;
            qc->end1 = x;
            qc->end2 = y;
            double r2 = hypot(x - qc->center1, y - qc->center2);
            int l2 = __find_turn(__endpoint_x, __endpoint_y, qc->center1, qc->center2, qc->turn, x, y);

            if(fabs(r1 - r2) > 0.01){
                __errInfo = "BUG: cutter compensation has generated an invalid arc with mismatched radii";
                return;
            }

            if (l1 and __endpoint_valid and (fabs(l2) > fabs(l1) + 0.0254)){
                __errInfo = "Arc move in concave corner cannot be reached by the tool without gouging";
                return;
            }
        }else if(q->type == 0 or q->type == 1){
            StraightFeedQc *qc = (StraightFeedQc *) q;
            double x1 = qc->dx;
            double y1 = qc->dy;
            double x2 = x - __endpoint_x;  //# new direction after clipping
            double y2 = y - __endpoint_y;
            double dot = x1*x2 + y1*y2; //# not normalized; we only care about the angle

            if(__endpoint_valid and dot <0){
                //# oops, the move is the wrong way.  this means the
                //# path has crossed because we backed up further
                //# than the line is long.  this will gouge.
                __errInfo = "Straight traverse/feed in concave corner " \
                                     "cannot be reached by the tool without gouging";
                return;
            }

            qc->x = x;
            qc->y = y;
        }
    }

    __dequeue_canons();
    __set_endpoint(x, y);

}

double ToolCompensate::__find_turn( double x1,
                                    double y1,
                                    double center_x,
                                    double center_y,
                                    int turn,
                                    double x2,
                                    double y2) {
    if(turn == 0) return 0.0;

    double alpha = atan2((y1 - center_y), (x1 - center_x));
    double beta = atan2((y2 - center_y), (x2 - center_x));


    double theta = 0.0;
    if(turn > 0){
        if(beta < alpha) beta += 2*M_PI;

        theta = (beta - alpha) + (turn - 1) * 2 * M_PI;
    }else{
        if(alpha <= beta) alpha += 2*M_PI;
        theta = (beta - alpha) + (turn + 1) * 2 * M_PI;
    }

    return theta;
}

void ToolCompensate::__dequeue_canons() {
    __endpoint_valid = false;
    if(__qc.empty()) return;

    for(Qc *q : __qc){
    	if(q->type == 2){
            ArcFeedQc *qc = (ArcFeedQc *) q;
            __ARC_FEED(qc->end1, qc->end2, qc->center1, qc->center2, qc->turn);

        }else if(q->type == 1){
            StraightFeedQc *qc = (StraightFeedQc *) q;
            __STRAIGHT_FEED(qc->x, qc->y, qc->lino);
        }else if(q->type == 0){
            StraightTraverseQc *qc = (StraightTraverseQc *)q;
            __STRAIGHT_TRAVERSE(qc->x, qc->y, qc->lino);
        }
    }

    for(Qc *q:__qc) {
       delete q;
    }
    // clear 并不能delete 指针
    __qc.clear();
}

void ToolCompensate::__set_endpoint(double x, double y) {
    __endpoint_x = x;
    __endpoint_y = y;
    __endpoint_valid = true;
}

void ToolCompensate::__convert_straight(int move) {
    //# 解析直线运动(G00 or G01)
    //# 检测到直线运动,不允许圆弧插补标志可取消(在G40时被置位)
    __arc_not_allowed = false;
    double end_x, end_y;
    __find_ends(&end_x, &end_y);

    if(__cutter_comp_side != 0 and __cutter_comp_radius > 0.0){
        if(__cutter_comp_firstmove){
            // 执行G41/42后首次直线运动
            __convert_straight_comp1(move, end_x, end_y);
        }else{
            __convert_straight_comp2(move, end_x, end_y);
        }
    }else{
        if(move == 0){
            __STRAIGHT_TRAVERSE(end_x, end_y);
        }else{
            __STRAIGHT_FEED(end_x, end_y);
        }
        __current_x = end_x;
        __current_y = end_y;
    }
}

void ToolCompensate::__find_ends(double *end_x, double *end_y) {
    bool middle = (!__cutter_comp_firstmove);
    int comp = __cutter_comp_side;

    if(__x_number == None){
        if(comp != 0 and middle){
            *end_x = __program_x;
        }else{
            *end_x = __current_x;
        }
    }else{
        *end_x = __x_number;
    }

    if(__y_number == None){
        if(comp != 0 and middle){
            *end_y = __program_y;
        }else{
            *end_y = __current_y;
        }
    }else{
        *end_y = __y_number;
    }

}

void ToolCompensate::__convert_straight_comp1(int move, double px, double py) {
    printf("---------------------> straight comp1\n");
	// 执行G41/42后首次直线运动
    double radius = __cutter_comp_radius;
    int side = __cutter_comp_side;
    double cx = __current_x;
    double cy = __current_y;
    double distance = hypot(px - cx, py - cy);
    if(distance <= radius){
        __errInfo = "刀补移动距离小于刀具半径";
        return;
    }

    double alpha = atan2(py-cy, px-cx);

    if(side == 1){
    	alpha += M_PI_2;
    }else{
    	alpha -= M_PI_2;
    }


    double end_x = px + radius*cos(alpha);
    double end_y = py + radius*sin(alpha);
    __set_endpoint(cx, cy);

    if(move == 0){
    	__enqueue_STRAIGHT_TRAVERSE(cos(alpha), sin(alpha), end_x, end_y, msg->GetLineNo());
    }else{
    	__enqueue_STRAIGHT_FEED(cos(alpha), sin(alpha), end_x, end_y, msg->GetLineNo());
    }

    __cutter_comp_firstmove = false;
    __current_x = end_x;
    __current_y = end_y;
    __program_x = px;
    __program_y = py;
}

void ToolCompensate::__convert_straight_comp2(int move, double px, double py) {
	printf("---------------------> straight comp2\n");
	//# 执行G41/42后非首次直线运动
    double small = 0.05;
    double cx = __current_x;
    double cy = __current_y;
    double end_x = cx;
    double end_y = cy;
    double opx = __program_x;
    double opy = __program_y;

    if(px == opx and py == opy){
        __errInfo = "起点与终点相同 没有发生运动！！！";
    }else{
        int side = __cutter_comp_side;
        double radius = __cutter_comp_radius;
        double theta = atan2(cy - opy, cx - opx);
        double alpha = atan2(py - opy, px - opx);

        double gamma, beta;

        if(side == 1){
            if(theta < alpha) theta += 2*M_PI;
            beta = theta - alpha - M_PI_2;
            gamma = M_PI_2;
        }else{
            if(alpha < theta) alpha += 2*M_PI;
            beta = alpha - theta - M_PI_2;
            gamma = -M_PI_2;
        }

        end_x = px + (radius * cos(alpha + gamma));
        end_y = py + (radius * sin(alpha + gamma));
        double mid_x = opx + (radius * cos(alpha + gamma));
        double mid_y = opy + (radius * sin(alpha + gamma));

        bool concave = true;

        if(beta < -small or beta > (M_PI + small)){
            concave = true;
        }else if(beta > (M_PI - small) and (!__qc.empty()) and __qc[0]->type == 2){
            ArcFeedQc *qc = (ArcFeedQc *)__qc[0];

            if((side == 2 and qc->turn > 0) or (side == 1 and qc->turn < 0)) {
                concave = true;
            }
        }else{
            concave = false;
        }

        if(!concave and (beta > small)){
            __move_endpoint_and_flush(cx, cy);

            if(!__errInfo.empty()) return;

            if(move == 1){
                // original_turn=0.0, doesn't matter, since we will not move this arc's endpoint
                int turn = side == 1 ? -1 : 1;
                printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
                __enqueue_ARC_FEED(0.0, mid_x, mid_y, opx, opy, turn);
            }else{
                // 快速移动 不需要圆弧过渡
                __enqueue_STRAIGHT_TRAVERSE(0.0, 0.0, mid_x, mid_y, msg->GetLineNo());
            }
            __dequeue_canons();
            if(!__errInfo.empty()) return;

            __set_endpoint(mid_x, mid_y);
        }else if(concave){

            if(__qc[0]->type != 2){
                //# line->line
                //# half the angle of the inside corner
                double halfcorner = (beta + M_PI) / 2.0;

                if(halfcorner == 0.0){
                    __errInfo = "Zero degree inside corner is invalid for cutter compensation";
                    return;
                }

                double retreat = radius / tan(halfcorner);
                //# move back along the compensated path
                //# this should replace the endpoint of the previous move
                mid_x = cx + retreat * cos(theta + gamma);
                mid_y = cy + retreat * sin(theta + gamma);
                //# we actually want to move the previous line's endpoint here.  That's the same as
                //# discarding that line and doing this one instead.
                __move_endpoint_and_flush(mid_x, mid_y);
                if(!__errInfo.empty()) {
                    return;
                }

            }else{
                //arc->line
                //beware: the arc we saved is the compensated one.
                ArcFeedQc *prev = (ArcFeedQc *)__qc[0];

                double oldrad = hypot(prev->center2 - prev->end2, prev->center1 - prev->end1);
                //# new line's direction
                double base_dir = atan2(py - opy, px - opx);
                double theta = prev->turn > 0 ? (base_dir + M_PI_2) : (base_dir - M_PI_2);
                double phi = atan2(prev->center2 - opy, prev->center1 - opx);

                double oldrad_uncomp;
                double angle_from_center;
                if(__TOOL_INSIDE_ARC(side, prev->turn))
                    oldrad_uncomp = oldrad + radius;
                else
                    oldrad_uncomp = oldrad - radius;
                alpha = theta - phi;
                //distance to old arc center perpendicular to the new line
                double d = oldrad_uncomp * cos(alpha);
                if(__TOOL_INSIDE_ARC(side, prev->turn)){
                    double d2 = d - radius;
                    double l = d2 / oldrad;
                    if(l > 1.0 or l < -1.0){
                        __errInfo = "Arc to straight motion makes a corner " \
                                         "the compensated tool can't fit in without gouging";
                        return;
                    }

                    if(prev->turn > 0){
                        angle_from_center = acos(l) + theta + M_PI;
                    }else {
                        angle_from_center = acos(l) + theta + M_PI;
                    }
                }
                else{
                    double d2 = d + radius;
                    double l = d2 / oldrad;
                    if(l > 1.0 or l < -1.0){
                        __errInfo = "Arc to straight motion makes a corner " \
                                         "the compensated tool can't fit in without gouging";
                        return;
                    }

                    if(prev->turn > 0)
                        angle_from_center = acos(l) + theta + M_PI;
                    else
                        angle_from_center = -acos(l) + theta + M_PI;
                }

                mid_x = prev->center1 + oldrad * cos(angle_from_center);
                mid_y = prev->center2 + oldrad * sin(angle_from_center);
                __move_endpoint_and_flush(mid_x, mid_y);

                if(!__errInfo.empty()) return;
            }

        }else{
            //# no arc needed, also not concave (colinear lines or tangent arc->line)
            __dequeue_canons();
            if(!__errInfo.empty()) return;
            __set_endpoint(mid_x, mid_y);
        }

        if(move == 0){
        	__enqueue_STRAIGHT_TRAVERSE(px - opx, py - opy, end_x, end_y, msg->GetLineNo());
        }else{
            __enqueue_STRAIGHT_FEED(px - opx, py - opy, end_x, end_y, msg->GetLineNo());
        }
    }
    __current_x = end_x;
    __current_y = end_y;
    __program_x = px;
    __program_y = py;
}

void ToolCompensate::__checkRadius(int direct, double xStart, double yStart,
                                    double xEnd, double yEnd, double radius, double *i, double *j)
{
    bool clockWise = direct == -1 ? true : false;
    double radiusAbs = fabs(radius);
    double xMid = (xStart + xEnd) / 2.0;
    double yMid = (yStart + yEnd) / 2.0;
    double halfLength = hypot(xMid-xEnd, yMid-yEnd);

    if((halfLength / radiusAbs) > (1 - 1.0e-12)) halfLength = radiusAbs;

    double theta;
    if((clockWise and radius>0.0) or ((!clockWise) and radius < 0.0)){
        theta = atan2(yEnd-yStart, xEnd-xStart) - M_PI_2;
    }else{
        theta = atan2(yEnd-yStart, xEnd-xStart) + M_PI_2;
    }

    double turn = asin(halfLength/radiusAbs);
    double offset = radiusAbs * cos(turn);
    double xCenter = xMid + offset * cos(theta);
    double yCenter = yMid + offset * sin(theta);
    *i = xCenter - xStart;
    *j = yCenter - yStart;
}

void ToolCompensate::__convert_arc(int move) {
    if(__arc_not_allowed){
        __errInfo = "执行 G40之后 首个运动不能是圆弧插补";
        return ;
    }

    bool first = __cutter_comp_firstmove;
    double end_x, end_y;
    __find_ends(&end_x, &end_y);

    if(__cutter_comp_side == 0 or __cutter_comp_radius == 0.0){
        __convert_arc2(move, end_x, end_y, __i_number, __j_number);
    }else if(first){
        // G41/G42后首个运动
        __convert_arc_comp1(move, end_x, end_y, __i_number, __j_number);
    }else{
        __convert_arc_comp2(move, end_x, end_y, __i_number, __j_number);
    }
}

void ToolCompensate::__convert_arc2(int move, double end1, double end2, double offset1, double offset2){
    __center1 = 0;
    __center2 = 0;
    __turn = 0;

    double spiral_abs_tolerance = 1;
    double radius_tolerance = 5e-5;

    __arc_data_ijk(move, __current_x, __current_y, end1, end2,
                   offset1, offset2,
                   radius_tolerance, spiral_abs_tolerance, 0.75);

    if(!__errInfo.empty()) return;

    __ARC_FEED(end1, end2, __center1, __center2, __turn);

    __current_x = end1;
    __current_y = end2;
}

void ToolCompensate::__arc_data_ijk(int move, double current_x, double current_y,
                                    double end_x, double end_y,
                                    double i_number, double j_number,
                                    double radius_tolerance, double spiral_abs_tolerance, double spiral_rel_tolerance)
{
    __center1 = current_x + i_number;
    __center2 = current_y + j_number;
    double radius = hypot(__center1 - current_x, __center2 - current_y);
    double radius2 = hypot(__center1 - end_x, __center2 - end_y);

    if(radius < radius_tolerance or radius2 < radius_tolerance){
        __errInfo = "圆弧半径过小";
        return ;
    }

    double abs_err = fabs(radius - radius2);
    double rel_err = abs_err / max(radius, radius2);

    if(abs_err > spiral_abs_tolerance or rel_err > spiral_rel_tolerance){
        __errInfo = "Radius to end of arc differs from radius to start";
        return;
    }

    if(move == 2){
        __turn = -1;
    }else{
        __turn = 1;
    }
}

void ToolCompensate::__convert_arc_comp1(int move, double end_x, double end_y, double offset_x, double offset_y) {
    int side = __cutter_comp_side;
    double tool_radius = __cutter_comp_radius;
    double spiral_abs_tolerance = 1e-3;
    double radius_tolerance = 5e-5;
    double cx = __current_x;
    double cy = __current_y;

    if(hypot(end_x - cx, end_y - cy) <= tool_radius){
        __errInfo = "Radius of cutter compensation entry arc is not greater than the tool radius";
        return;
    }

    __arc_data_comp_ijk(move, side, tool_radius, cx, cy, end_x, end_y,
                        offset_x, offset_y, radius_tolerance,
                        spiral_abs_tolerance, 0.75);

    if(!__errInfo.empty()) return;

    double center_x = __center1;
    double center_y = __center2;
    int turn = __turn;

    double gamma;
    if(__TOOL_INSIDE_ARC(side, turn)){
        gamma = atan2(center_y - end_y, center_x - end_x);
    }else{
        gamma = atan2(end_y - center_y, end_x - center_x);
    }
    // 清除G41/G42后首次运动标志
    __cutter_comp_firstmove = false;
    __program_x = end_x;
    __program_y = end_y;

    // 求得实际结束位置,这将改变圆弧的半径和圆心
    end_x += tool_radius * cos(gamma);
    end_y += tool_radius * sin(gamma);

    double b_len = hypot(cy-end_y, cx-end_x) / 2.0;
    double AB_ang = atan2(center_y - end_y, center_x - end_x);
    double A_ang = atan2(cy-end_y, cx-end_x) - AB_ang;

    if(fabs(cos(A_ang)) < 1e-4){
        __errInfo = "NCE_TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP";
        return;
    }

    double c_len = b_len / cos(A_ang);

    // 算出新圆心
    center_x = end_x + c_len * cos(AB_ang);
    center_y = end_y + c_len * sin(AB_ang);

    if(fabs(hypot(center_x - end_x, center_y - end_y) - hypot(center_x - cx, center_y - cy)) > spiral_abs_tolerance){
        __errInfo = "NCE_BUG_IN_TOOL_RADIUS_COMP";
        return;
    }

    int original_turns = __find_turn(cx, cy, center_x, center_y, turn, end_x, end_y);
    __enqueue_ARC_FEED(original_turns, end_x, end_y, center_x, center_y, turn);
    __current_x = end_x;
    __current_y = end_y;
}

void ToolCompensate::__convert_arc_comp2(int move, double end_x, double end_y, double offset_x, double offset_y) {
    double small = 0.05;
    double spiral_abs_tolerance = 1e-3;
    double radius_tolerance = 5e-5;
    double opx = __program_x;
    double opy = __program_y;
    double cx = __current_x;
    double cy = __current_y;
    //# 通过下面的函数求得self.__center1, self.__center2, self.__turn,并进行参数检查
    __arc_data_ijk(move, opx, opy, end_x, end_y, offset_x, offset_y,
                        radius_tolerance, spiral_abs_tolerance, 0.75);

    if(!__errInfo.empty()) return;

    double centerx = __center1;
    double centery = __center2;
    int turn = __turn;
    int side = __cutter_comp_side;
    double tool_radius = __cutter_comp_radius;
    double arc_radius = hypot((centerx - end_x), (centery - end_y));
    double theta = atan2(cy - opy, cx - opx);
    theta = side == 1 ? (theta - M_PI_2 ) : (theta + M_PI_2);
    double delta = atan2(centery - opy, centerx - opx);
    double alpha = move == 3 ? (delta - M_PI_2) : (delta + M_PI_2);
    double beta = side == 1 ? (theta - alpha) : (alpha - theta);

    // 使beta的对应角度在-90到270之间
    if(beta > 1.5 * M_PI) beta -= 2 * M_PI;
    if(beta < -M_PI / 2.0) beta += 2 * M_PI;

    double gamma;

    if((side == 1 and move == 3) or (side == 2 and move == 2)){
        // we are cutting inside the arc
        gamma = atan2((centery - end_y), (centerx - end_x));
        if(arc_radius <= tool_radius) {
            __errInfo = "NCE_TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP";
            return;
        }
    }else{
        gamma = atan2((end_y - centery), (end_x - centerx));
        delta += M_PI;
    }

    // 计算新的结束位置
    double new_end_x = end_x + tool_radius * cos(gamma);
    double new_end_y = end_y + tool_radius * sin(gamma);

    if (beta < -small or beta > small + M_PI or (fabs(beta - M_PI) < small and not __TOOL_INSIDE_ARC(side, turn))){
        Qc *qc = __qc[0];

        if(qc->type != 2){
            // line -> arc
            cy = arc_radius * sin(beta - M_PI_2);
            double angle_from_center;
            double dist_from_center;
            double toward_nominal;

            if(__TOOL_INSIDE_ARC(side, turn)){
               //# tool is inside the arc
               dist_from_center = arc_radius - tool_radius;
               toward_nominal = cy + tool_radius;
               double l = toward_nominal / dist_from_center;

               if(l > 1.0 or l < -1.0){
                   __errInfo = "Arc move in concave corner cannot be reached by the tool without gouging";
                   return ;
               }

               if(turn > 0) angle_from_center = theta + asin(l);
               else angle_from_center = theta - asin(l);

            }else{
                dist_from_center = arc_radius + tool_radius;
                toward_nominal = cy - tool_radius;
                double l = toward_nominal / dist_from_center;
                if(l > 1.0 or l < -1.0){
                    __errInfo = "Arc move in concave corner cannot be reached by the tool without gouging";
                    return;
                }

                if(turn > 0)
                    angle_from_center = theta + M_PI - asin(l);
                else
                    angle_from_center = theta + M_PI + asin(l);

                double midx = centerx + dist_from_center * cos(angle_from_center);
                double midy = centery + dist_from_center * sin(angle_from_center);
                __move_endpoint_and_flush(midx, midy);
                if(!__errInfo.empty()) return;
            }

        }else{
            // arc -> arc
            ArcFeedQc * prev = (ArcFeedQc *) qc;

            double oldrad = hypot(prev->center2 - prev->end2, prev->center1 - prev->end1);
            double newrad;
            if(__TOOL_INSIDE_ARC(side, turn)){
                newrad = arc_radius - tool_radius;
            }else{
                newrad = arc_radius + tool_radius;
            }
            double arc_cc = hypot(prev->center2 - centery, prev->center1 - centerx);

            if(oldrad == 0 or arc_cc == 0){
                __errInfo = "Arc to arc motion is invalid because the arcs have the same center";
                return;
            }

            double a = (pow(oldrad, 2) + pow(arc_cc, 2) - pow(newrad, 2)) / (2 * oldrad * arc_cc);

            if(a > 1.0 or a < -1.0) {
                __errInfo = "Arc to arc motion makes a corner the compensated tool can't fit in without gouging";
                return;
            }

            double pullback = acos(a);
            double cc_dir = atan2(centery - prev->center2, centerx - prev->center1);
            double dir;

            if(__TOOL_INSIDE_ARC(side, turn)){
                if(turn > 0) dir = cc_dir + pullback;
                else dir = cc_dir - pullback;
            }else{
                if(turn > 0) dir = cc_dir - pullback;
                else dir = cc_dir + pullback;
            }
            double midx = prev->center1 + oldrad * cos(dir);
            double midy = prev->center2 + oldrad * sin(dir);
            __move_endpoint_and_flush(midx, midy);

            if(!__errInfo.empty()) return;

            int original_turns = __find_turn(opx, opy, centerx, centery, turn, end_x, end_y);
            __enqueue_ARC_FEED(original_turns, new_end_x, new_end_y, centerx, centery, turn);
        }

    }else if(beta > small){
        // convex, two arcs needed
        double midx = opx + tool_radius * cos(delta);
        double midy = opy + tool_radius * sin(delta);
        __dequeue_canons();

        if(!__errInfo.empty()) return;

        //# original_turn=0.0, doesn't matter since we won't move this arc's endpoint
        if(side == 1) __enqueue_ARC_FEED(0.0, midx, midy, opx, opy, -1);
        else __enqueue_ARC_FEED(0.0, midx, midy, opx, opy, 1);
        __dequeue_canons();
        if(!__errInfo.empty()) return;
        __set_endpoint(midx, midy);
        int original_turns = __find_turn(opx, opy, centerx, centery, turn, end_x, end_y);
        __enqueue_ARC_FEED(original_turns, new_end_x, new_end_y, centerx, centery, turn);

    }else{
        __dequeue_canons();
        if(!__errInfo.empty()) return;

        __set_endpoint(cx, cy);
        int original_turns = __find_turn(opx, opy, centerx, centery, turn, end_x, end_y);
        __enqueue_ARC_FEED(original_turns, new_end_x, new_end_y, centerx, centery, turn);
    }

    __program_x = end_x;
    __program_y = end_y;
    __current_x = new_end_x;
    __current_y = new_end_y;
}

void ToolCompensate::__arc_data_comp_ijk(int move, int side, double tool_radius, double current_x, double current_y,
                                         double end_x, double end_y, double i_number, double j_number,
                                         double radius_tolerance, double spiral_abs_tolerance, double spiral_rel_tolerance)
{
    __center1 = current_x + i_number;
    __center2 = current_y + j_number;

    double arc_radius = hypot(__center1 - current_x, __center2 - current_y);
    double radius2 = hypot(__center1-end_x, __center2 - end_y);

    if(arc_radius < radius_tolerance or radius2 < radius_tolerance){
        __errInfo = "Zero-radius arc";
        return;
    }

    double abs_err = fabs(arc_radius - radius2);
    double rel_err = abs_err / max(arc_radius, radius2);

    if(abs_err > spiral_abs_tolerance or rel_err > spiral_rel_tolerance){
        __errInfo = "Radius to end of arc differs from radius to start";
        return;
    }

    if(arc_radius <= tool_radius and ((side == 1 and move == 3) or (side == 2 and move == 2))){
        __errInfo = "NCE_TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP";
        return;
    }

    if(move == 2) __turn = -1;
    else __turn = 1;
}


