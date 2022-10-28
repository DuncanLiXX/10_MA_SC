/*
 * rs274.h
 *
 *  Created on: 2022Äê10ÔÂ25ÈÕ
 *      Author: SNADO
 */

#ifndef INC_TOOL_COMPENSATE_RS274_H_
#define INC_TOOL_COMPENSATE_RS274_H_

#include <stdint.h>
#include <stdio.h>
#include <vector>

enum queued_canon_type {
	QSTRAIGHT_TRAVERSE,
	QSTRAIGHT_FEED,
	QARC_FEED
};

enum CUTTER_COMP_DIRECTION {
    RIGHT = 1,
    LEFT = 2,
};


typedef struct block_struct
{
	bool     a_flag;
	double   a_number;
	bool     b_flag;
	double   b_number;
	bool     c_flag;
	double   c_number;
	bool     x_flag;
	double   x_number;
	bool     y_flag;
	double   y_number;
	bool     z_flag;
	double   z_number;
	bool     i_flag;
	double   i_number;
	bool     j_flag;
	double   j_number;
	bool     k_flag;
	double   k_number;
	bool     r_flag;
	double   r_number;
	int      d_number;
	double   f_number;
	double   s_number;
	int      t_number;

	int      l_number;
	int      line_number;
	double   p_number;
	double   q_number;

	char     comment[256];
	int      g_modes[14];
	int      motion_to_be;
	int      m_count;
	int      m_modes[10];

} block;

typedef block * block_pointer;

typedef struct setup_struct
{
	double current_x;                             // current X-axis position
	double current_y;                             // current Y-axis position
	double current_z;                             // current Z-axis position

	double program_x;                             // program x, used when cutter comp on
	double program_y;                             // program y, used when cutter comp on
	double program_z;

	double origin_offset_x;                       // origin offset x
	double origin_offset_y;                       // origin offset y
	double origin_offset_z;                       // origin offset z

	block _block;                                 // parsed next block

	bool cutter_comp_firstmove; // this is the first comp move
	double cutter_comp_radius;                    // current cutter compensation radius
	int cutter_comp_side;                         // current cutter compensation side
	bool arc_not_allowed;
	double feed_rate;                             // feed rate in current units/min
	int plane;                            // active plane, XY-, YZ-, or XZ-plane
	bool percent_flag;                          // ON means first line was percent sign
	int sequence_number;                          // sequence number of line last read
	double tool_length_offset;                    // current tool length offset

	int motion_mode;
} setup;

typedef setup * setup_pointer;
/*******************************************************/



struct straight_traverse {
    int line_number;
    double dx, dy, dz;          // direction of original motion
    double x,y,z, a,b,c, u,v,w;
};

struct straight_feed {
    int line_number;
    double dx, dy, dz;          // direction of original motion
    double x,y,z, a,b,c, u,v,w; // target
};

struct arc_feed {
    int line_number;
    double original_turns;
    double end1, end2, center1, center2;
    int turn;
    double end3, a,b,c, u,v,w;
};

struct queued_canon {
    queued_canon_type type;
    union {
        struct straight_traverse straight_traverse;
        struct straight_feed straight_feed;
        struct arc_feed arc_feed;
    } data;
};

void dequeue_canons(setup_pointer settings);

int enqueue_STRAIGHT_FEED(setup_pointer settings, int l,
                          double dx, double dy, double dz,
                          double x, double y, double z,
						  double a, double b, double c,
						  double u, double v, double w);

int enqueue_STRAIGHT_TRAVERSE(setup_pointer settings, int l,
                              double dx, double dy, double dz,
                              double x, double y, double z,
							  double a, double b, double c,
							  double u, double v, double w);

void enqueue_ARC_FEED(setup_pointer settings, int l,
                      double original_arclen,
                      double end1, double end2,
					  double center1, double center2,
                      int turn,
                      double end3,
					  double a, double b, double c,
					  double u, double v, double w);

void set_endpoint(double x, double y);

class Interp{

public:
	Interp(){}
	~Interp(){}

	int move_endpoint_and_flush(setup_pointer s, double x, double y);

	int arc_data_comp_ijk(int move,
	         int plane,
	         int side,
	         double tool_radius,
	         double current_x,
	         double current_y,
	         double end_x,
	         double end_y,
	         int ij_absolute,
	         double i_number,
	         double j_number,
	         int p_number,
	         double *center_x,
	         double *center_y,
	         int *turn,
	         double radius_tolerance,
	         double spiral_abs_tolerance,
	         double spiral_rel_tolerance);

	 int arc_data_comp_r(int move,
	         int plane,
	         int side,
	         double tool_radius,
	         double current_x,
	         double current_y,
	         double end_x,
	         double end_y,
	         double big_radius,
	         int p_number,
	         double *center_x,
	         double *center_y,
	         int *turn,
	         double radius_tolerance);

	 int arc_data_ijk(int move,
	         int plane,
	         double current_x,
	         double current_y,
	         double end_x,
	         double end_y,
	         int ij_absolute,
	         double i_number,
	         double j_number,
	         int p_number,
	         double *center_x,
	         double *center_y,
	         int *turn,
	         double radius_tolerance,
	         double spiral_abs_tolerance,
	         double spiral_rel_tolerance);

	 int arc_data_r(int move,
	         int plane,
	         double current_x,
	         double current_y,
	         double end_x,
	         double end_y,
	         double radius,
	         int p_number,
	         double *center_x,
	         double *center_y,
	         int *turn,
	         double radius_tolerance);

	 int comp_get_current(setup_pointer settings, double *x, double *y, double *z);
	 int comp_set_current(setup_pointer settings, double x, double y, double z);
	 int comp_get_programmed(setup_pointer settings, double *x, double *y, double *z);
	 int comp_set_programmed(setup_pointer settings, double x, double y, double z);
	 int convert_arc(int move, block_pointer block, setup_pointer settings);
	 int convert_arc2(int move, block_pointer block,
				      setup_pointer settings,
				      double *current1, double *current2, double *current3,
				      double end1, double end2, double end3,
				      double AA_end, double BB_end, double CC_end,
				      double u_end, double v_end, double w_end,
				      double offset1, double offset2);

	 int convert_arc_comp1(int move, block_pointer block,
	                       setup_pointer settings,
	                       double end_x, double end_y, double end_z,
	                       double offset_x, double offset_y,
	                       double AA_end, double BB_end, double CC_end,
	                       double u_end, double v_end, double w_end);

	 int convert_arc_comp2(int move, block_pointer block,
	                       setup_pointer settings,
	                       double end_x, double end_y, double end_z,
	                       double offset_x, double offset_y,
	                       double AA_end, double BB_end, double CC_end,
	                       double u_end, double v_end, double w_end);

	 int convert_cutter_compensation_off(setup_pointer settings);

	 int convert_cutter_compensation_on(int side, double radius, setup_pointer settings);

	 int convert_straight(int move, block_pointer block,
	                             setup_pointer settings);

	 int convert_straight_comp1(int move, block_pointer block,
	                             setup_pointer settings,
	                             double px, double py, double end_z,
	                             double AA_end, double BB_end, double CC_end,
	                             double u_end, double v_end, double w_end);

	 int convert_straight_comp2(int move, block_pointer block,
	                             setup_pointer settings,
	                             double px, double py, double end_z,
	                             double AA_end, double BB_end, double CC_end,
	                             double u_end, double v_end, double w_end);

	 double find_arc_length(double x1, double y1, double z1,
	                              double center_x, double center_y, int turn,
	                              double x2, double y2, double z2);

	 int find_ends(block_pointer block, setup_pointer settings,
	               double *px, double *py, double *pz);

	 int find_relative(double x1, double y1, double z1,
	                   double *x2, double *y2, double *z2,
	                   setup_pointer settings);

	 double find_straight_length(double x2, double y2, double z2,
	                             double x1, double y1, double z1);

	 double find_turn(double x1, double y1, double center_x,
	                        double center_y, int turn, double x2, double y2);

	 int init_block(block_pointer block);

	 block* interp_block(){return &_setup._block;}

	 setup _setup;
};

void ARC_FEED(int line_number,
              double first_end,
			  double second_end,
			  double first_axis,
			  double second_axis,
			  int rotation,
			  double axis_end_point,
              double a_position,
			  double b_position,
			  double c_position,
              double u_position,
			  double v_position,
			  double w_position);

void STRAIGHT_FEED(int line_number,
                   double x, double y, double z,
                   double a, double b, double c,
                   double u, double v, double w);

void STRAIGHT_TRAVERSE(int line_number,
                       double x, double y, double z,
                       double a, double b, double c,
                       double u, double v, double w);


#endif /* INC_TOOL_COMPENSATE_RS274_H_ */
