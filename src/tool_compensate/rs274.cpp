/*
 * rs274.cpp
 *
 *  Created on: 2022年10月25日
 *      Author: SNADO
 */

#include "rs274.h"
#include <cmath>

#define RADIUS_TOLERANCE_MM 0.001
#define TINY 1e-12
#define TOLERANCE_EQUAL 0.0001
#define TOLERANCE_CONCAVE_CORNER 0.05  /* angle for testing corners */
#define TOOL_INSIDE_ARC(side, turn) (((side)==LEFT&&(turn)>0)||((side)==RIGHT&&(turn)<0))

static double endpoint[2];
static int endpoint_valid = 0;

std::vector<queued_canon>& qc(void) {
    static std::vector<queued_canon> c;
    return c;
}

int Interp::arc_data_comp_ijk(int move,
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
							  double spiral_rel_tolerance)
{
	double arc_radius;
	double radius2;
	char a = 'X', b = 'Y';

	if ( ij_absolute ) {
		*center_x = (i_number);
		*center_y = (j_number);
	} else {
		*center_x = (current_x + i_number);
		*center_y = (current_y + j_number);
	}
	arc_radius = hypot((*center_x - current_x), (*center_y - current_y));
	radius2 = hypot((*center_x - end_x), (*center_y - end_y));

	if((arc_radius < radius_tolerance) || (radius2 < radius_tolerance)){
		printf("Zero-radius arc: "
		   "start=(%c%.4f,%c%.4f) center=(%c%.4f,%c%.4f) end=(%c%.4f,%c%.4f) r1=%.4f r2=%.4f",
		   a, current_x, b, current_y,
		   a, *center_x, b, *center_y,
		   a, end_x, b, end_y, arc_radius, radius2);

		return 0;
	}


	double abs_err = fabs(arc_radius - radius2);
	double rel_err = abs_err / std::max(arc_radius, radius2);

	if((abs_err > spiral_abs_tolerance * 100.0) ||
			  (rel_err > spiral_rel_tolerance && abs_err > spiral_abs_tolerance)){
		printf("Radius to end of arc differs from radius to start: "
				   "start=(%c%.4f,%c%.4f) center=(%c%.4f,%c%.4f) end=(%c%.4f,%c%.4f) "
				   "r1=%.4f r2=%.4f abs_err=%.4g rel_err=%.4f%%",
				   a, current_x, b, current_y,
				   a, *center_x, b, *center_y,
				   a, end_x, b, end_y, arc_radius, radius2,
				   abs_err, rel_err*100);
	}

	if((arc_radius <= tool_radius and side == LEFT and move == 30) or (side == RIGHT and move == 20 and arc_radius <= tool_radius))
	{
		printf("TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP");
		return 0;
	}

	/* This catches an arc too small for the tool, also */
	if (move == 20)
		*turn = -1;
	else if (move == 30)
		*turn = 1;

	return 0;
}

int Interp::arc_data_comp_r(int move,
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
							double tolerance)
{
	double abs_radius;            // absolute value of big_radius

	abs_radius = fabs(big_radius);

	if((abs_radius <= tool_radius) and ((side == LEFT && move == 30) or (side == LEFT && move == 20))){
		printf("tool radius not less than arc radius with comp\n");
		return 0;
	}

	arc_data_r(move, plane, current_x, current_y, end_x, end_y, big_radius, p_number,
			 center_x, center_y, turn, tolerance);

	return 0;
}

int Interp::arc_data_ijk(int move,
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
						 double spiral_rel_tolerance)
{
	  double radius;                /* radius to current point */
	  double radius2;               /* radius to end point     */
	  char a = 'X', b = 'Y';

	  if ( ij_absolute ) {
	    *center_x = (i_number);
	    *center_y = (j_number);
	  } else {
	    *center_x = (current_x + i_number);
	    *center_y = (current_y + j_number);
	  }
	  radius = hypot((*center_x - current_x), (*center_y - current_y));
	  radius2 = hypot((*center_x - end_x), (*center_y - end_y));

	  if(radius < radius_tolerance or radius2 < radius_tolerance){
		  printf("arc radius too small canot move\n");
		  return 0;
	  }

	  double abs_err = fabs(radius - radius2);
	  double rel_err = abs_err / std::max(radius, radius2);

	  if((abs_err > spiral_abs_tolerance * 100.0) ||
	     (rel_err > spiral_rel_tolerance && abs_err > spiral_abs_tolerance)){

		  printf("Radius to end of arc differs from radius to start: "
		  	       "start=(%c%.4f,%c%.4f) center=(%c%.4f,%c%.4f) end=(%c%.4f,%c%.4f) "
		  	       "r1=%.4f r2=%.4f abs_err=%.4g rel_err=%.4f%%\n",
		  	       a, current_x, b, current_y,
		  	       a, *center_x, b, *center_y,
		  	       a, end_x, b, end_y, radius, radius2,
		  	       abs_err, rel_err*100);
	  }

	  if (move == 20)
	    *turn = -1;
	  else if (move == 30)
	    *turn = 1;
	  else
		  printf("arc data ijk error\n");

	  return 0;
}

int Interp::arc_data_r(int move,
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
					   double tolerance)
{
	  double abs_radius;            /* absolute value of given radius */
	  double half_length;           /* distance from M to end point   */
	  double mid_x;                 /* first coordinate of M          */
	  double mid_y;                 /* second coordinate of M         */
	  double offset;                /* distance from M to center      */
	  double theta;                 /* angle of line from M to center */
	  double turn2;                 /* absolute value of half of turn */

	  if((end_x == current_x) && (end_y == current_y)){
		  printf("arc current point same as end point\n");
		  return 0;
	  }

	  abs_radius = fabs(radius);
	  mid_x = (end_x + current_x) / 2.0;
	  mid_y = (end_y + current_y) / 2.0;
	  half_length = hypot((mid_x - end_x), (mid_y - end_y));

	  if((half_length - abs_radius) > tolerance){
		  printf("radius too small to reach end point\n");
		  return 0;
	  }

	  if ((half_length / abs_radius) > (1 - TINY))
	    half_length = abs_radius;   /* allow a small error for semicircle */
	  /* check needed before calling asin   */
	  if (((move == 20) && (radius > 0)) || ((move == 30) && (radius < 0)))
	    theta = atan2((end_y - current_y), (end_x - current_x)) - M_PI_2l;
	  else
	    theta = atan2((end_y - current_y), (end_x - current_x)) + M_PI_2l;

	  turn2 = asin(half_length / abs_radius);
	  offset = abs_radius * cos(turn2);
	  *center_x = mid_x + (offset * cos(theta));
	  *center_y = mid_y + (offset * sin(theta));
	  *turn = (move == 20) ? -1 : 1;

	  return 0;
}

int Interp::comp_get_current(setup_pointer settings,
		                     double *x,
							 double *y,
							 double *z)
{
	*x = settings->current_x;
	*y = settings->current_y;
	*z = settings->current_z;
	return 0;
}

int Interp::comp_set_current(setup_pointer settings,
		                     double x, double y, double z)
{
    settings->current_x = x;
    settings->current_y = y;
    settings->current_z = z;
	return 0;
}

int Interp::comp_get_programmed(setup_pointer settings,
		                        double *x, double *y, double *z)
{
    *x = settings->program_x;
    *y = settings->program_y;
    *z = settings->program_z;
	return 0;
}

int Interp::comp_set_programmed(setup_pointer settings,
		                        double x, double y, double z)
{
    settings->program_x = x;
    settings->program_y = y;
    settings->program_z = z;
	return 0;
}

int Interp::convert_arc(int move, block_pointer block, setup_pointer settings)
{
	int status;
	int first;                    /* flag set true if this is first move after comp true */
	int ijk_flag;                 /* flag set true if any of i,j,k present in NC code  */
	double end_x;
	double end_y;
	double end_z;
	double AA_end;
	double BB_end;
	double CC_end;
	double u_end, v_end, w_end;

	if(settings->arc_not_allowed){
		printf("arc not allowed 退出刀补第一段不能是圆弧移动 \n");
		return 0;
	}

	ijk_flag = block->i_flag || block->j_flag || block->k_flag;
	first = settings->cutter_comp_firstmove;

	if(block->r_flag) ijk_flag = false;   // R 优先级高 忽略 ij

	// @TODO 检查参数
	if(ijk_flag){

	}else{

	}

	find_ends(block, settings, &end_x, &end_y, &end_z);

	settings->motion_mode = move;

    if ((!settings->cutter_comp_side) ||
        (settings->cutter_comp_radius == 0.0)) {
      status =
        convert_arc2(move, block, settings,
                     &(settings->current_x), &(settings->current_y),
                     &(settings->current_z), end_x, end_y, end_z,
                     AA_end, BB_end, CC_end,
                     u_end, v_end, w_end,
                     block->i_number, block->j_number);
    } else if (first) {
      status = convert_arc_comp1(move, block, settings, end_x, end_y, end_z,
                                 block->i_number, block->j_number,
                                 AA_end, BB_end, CC_end,
                                 u_end, v_end, w_end);
    } else {
      status = convert_arc_comp2(move, block, settings, end_x, end_y, end_z,
                                 block->i_number, block->j_number,
                                 AA_end, BB_end, CC_end,
                                 u_end, v_end, w_end);
    }

	return 0;
}

int Interp::convert_arc2(int move,
		                 block_pointer block,
						 setup_pointer settings,
						 double *current1,
						 double *current2,
                         double *current3,
						 double end1,
						 double end2,
						 double end3,
						 double AA_end,
						 double BB_end,
                         double CC_end,
						 double u,
						 double v,
						 double w,
						 double offset1,
						 double offset2)
{
	double center1;
	double center2;
	int turn;                     /* number of full or partial turns CCW in arc */
	int plane = settings->plane;

	// Spiral tolerance is the amount of "spiral" allowed in a given arc segment, or (r2-r1)/theta
	double spiral_abs_tolerance = RADIUS_TOLERANCE_MM;

	// Radius tolerance allows a bit of leeway on the minimum radius for a radius defined arc.
	double radius_tolerance = RADIUS_TOLERANCE_MM;

	if (block->r_flag) {
	  arc_data_r(move, plane, *current1, *current2, end1, end2,
					 block->r_number, 1,
					 &center1, &center2, &turn, radius_tolerance);
	} else {
	  arc_data_ijk(move, plane, *current1, *current2, end1, end2,
					   0, offset1, offset2, 1,
					   &center1, &center2, &turn, radius_tolerance, spiral_abs_tolerance, RADIUS_TOLERANCE_MM);
	}

	ARC_FEED(block->line_number, end1, end2, center1, center2, turn, end3,
		   AA_end, BB_end, CC_end, u, v, w);
	*current1 = end1;
	*current2 = end2;
	*current3 = end3;

	return 0;
}

int Interp::convert_arc_comp1(int move,
		                      block_pointer block,
							  setup_pointer settings,
							  double end_x, double end_y,double end_z,
							  double offset_x, double offset_y,
							  double AA_end, double BB_end, double CC_end,
							  double u_end, double v_end, double w_end)
{
    double center_x, center_y;
    double gamma;                 /* direction of perpendicular to arc at end */
    int side;                     /* offset side - right or left              */
    double tool_radius;
    int turn;                     /* 1 for counterclockwise, -1 for clockwise */
    double cx, cy, cz; // current
    int plane = 0;

    side = settings->cutter_comp_side;
    tool_radius = settings->cutter_comp_radius;   /* always is positive */

    double spiral_abs_tolerance = RADIUS_TOLERANCE_MM;
    double radius_tolerance = RADIUS_TOLERANCE_MM;

    comp_get_current(settings, &cx, &cy, &cz);

    if(hypot((end_x - cx), (end_y - cy)) <= tool_radius){
    	printf("Radius of cutter compensation entry arc is not greater than the tool radius");
    	return 0;
    }

    if (block->r_flag) {
        arc_data_comp_r(move, plane, side, tool_radius, cx, cy, end_x, end_y,
                            block->r_number, 1,
                            &center_x, &center_y, &turn, radius_tolerance);
    } else {
        arc_data_comp_ijk(move, plane, side, tool_radius, cx, cy, end_x, end_y,
                              0,
                              offset_x, offset_y, 1,
                              &center_x, &center_y, &turn, radius_tolerance, spiral_abs_tolerance, RADIUS_TOLERANCE_MM);
    }

    // the tool will end up in gamma direction from the programmed arc endpoint
    if TOOL_INSIDE_ARC(side, turn) {
        // tool inside the arc: ends up toward the center
        gamma = atan2((center_y - end_y), (center_x - end_x));
    } else {
        // outside: away from the center
        gamma = atan2((end_y - center_y), (end_x - center_x));
    }

    settings->cutter_comp_firstmove = false;

    comp_set_programmed(settings, end_x, end_y, end_z);

    // move endpoint to the compensated position.  This changes the radius and center.
    end_x += tool_radius * cos(gamma);
    end_y += tool_radius * sin(gamma);

    /* To find the new center:
       imagine a right triangle ABC with A being the endpoint of the
       compensated arc, B being the center of the compensated arc, C being
       the midpoint between start and end of the compensated arc. AB_ang
       is the direction of A->B.  A_ang is the angle of the triangle
       itself.  We need to find a new center for the compensated arc
       (point B). */

    double b_len = hypot(cy - end_y, cx - end_x) / 2.0;
    double AB_ang = atan2(center_y - end_y, center_x - end_x);
    double A_ang = atan2(cy - end_y, cx - end_x) - AB_ang;

    if(fabs(cos(A_ang)) < TOLERANCE_EQUAL){
    	printf("tool radius not less than arc radius with comp\n");
    	return 0;
    }

    double c_len = b_len/cos(A_ang);

    // center of the arc is c_len from end in direction AB
    center_x = end_x + c_len * cos(AB_ang);
    center_y = end_y + c_len * sin(AB_ang);

    /* center to endpoint distances matched before - they still should. */
    if((fabs(hypot(center_x-end_x,center_y-end_y) -
            hypot(center_x-cx,center_y-cy))) > spiral_abs_tolerance){
    	printf("bug in tool radius comp\n");
    	return 0;
    }

    double theta = find_turn(cx, cy, center_x, center_y, turn, end_x, end_y);

    enqueue_ARC_FEED(settings, block->line_number,
                     theta,
                     end_x, end_y, center_x, center_y, turn, end_z,
                     AA_end, BB_end, CC_end, u_end, v_end, w_end);

    comp_set_current(settings, end_x, end_y, end_z);


    return 0;
}

int Interp::convert_arc_comp2(int move,
		                      block_pointer block,
							  setup_pointer settings,
							  double end_x, double end_y, double end_z,
							  double offset_x, double offset_y,
							  double AA_end, double BB_end, double CC_end,
							  double u, double v, double w)
{
    double alpha;                 /* direction of tangent to start of arc */
    double arc_radius;
    double beta;                  /* angle between two tangents above */
    double centerx, centery;              /* center of arc */
    double delta;                 /* direction of radius from start of arc to center of arc */
    double gamma;                 /* direction of perpendicular to arc at end */
    double midx, midy;
    int side;
    double small = TOLERANCE_CONCAVE_CORNER;      /* angle for testing corners */
    double opx = 0, opy = 0, opz = 0;
    double theta;                 /* direction of tangent to last cut */
    double tool_radius;
    int turn;                     /* number of full or partial circles CCW */
    int plane = settings->plane;
    double cx, cy, cz;
    double new_end_x, new_end_y;

    double spiral_abs_tolerance =  RADIUS_TOLERANCE_MM;
    double radius_tolerance = RADIUS_TOLERANCE_MM;

    /* find basic arc data: center_x, center_y, and turn */

    comp_get_programmed(settings, &opx, &opy, &opz);
    comp_get_current(settings, &cx, &cy, &cz);


    if (block->r_flag) {
        arc_data_r(move, plane, opx, opy, end_x, end_y,
                       block->r_number, 1,
                       &centerx, &centery, &turn, radius_tolerance);
    } else {
        arc_data_ijk(move, plane,
                         opx, opy, end_x, end_y,
                         0,
                         offset_x, offset_y, 1,
                         &centerx, &centery, &turn, radius_tolerance, spiral_abs_tolerance, RADIUS_TOLERANCE_MM);
    }

    side = settings->cutter_comp_side;
    tool_radius = settings->cutter_comp_radius;   /* always is positive */
    arc_radius = hypot((centerx - end_x), (centery - end_y));
    theta = atan2(cy - opy, cx - opx);
    theta = (side == LEFT) ? (theta - M_PI_2l) : (theta + M_PI_2l);
    delta = atan2(centery - opy, centerx - opx);
    alpha = (move == 30) ? (delta - M_PI_2l) : (delta + M_PI_2l);
    beta = (side == LEFT) ? (theta - alpha) : (alpha - theta);

    // normalize beta -90 to +270?
    beta = (beta > (1.5 * M_PIl)) ? (beta - (2 * M_PIl)) : (beta < -M_PI_2l) ? (beta + (2 * M_PIl)) : beta;

    if (((side == LEFT) && (move == 30)) || ((side == RIGHT) && (move == 20))) {
        // we are cutting inside the arc
        gamma = atan2((centery - end_y), (centerx - end_x));

        if(arc_radius <= tool_radius){
        	printf("TOOL_RADIUS_NOT_LESS_THAN_ARC_RADIUS_WITH_COMP\n");
        	return 0;
        }

    } else {
        gamma = atan2((end_y - centery), (end_x - centerx));
        delta = (delta + M_PIl);
    }

    // move arc endpoint to the compensated position
    new_end_x = end_x + tool_radius * cos(gamma);
    new_end_y = end_y + tool_radius * sin(gamma);

    if (beta < -small ||
        beta > M_PIl + small ||
        // special detection for convex corner on tangent arc->arc (like atop the middle of "m" shape)
        // or tangent line->arc (atop "h" shape)
        (fabs(beta - M_PIl) < small && !TOOL_INSIDE_ARC(side, turn))
        ) {
        // concave
        if (qc().front().type != QARC_FEED) {
            // line->arc
            double cy = arc_radius * sin(beta - M_PI_2l);
            double toward_nominal;
            double dist_from_center;
            double angle_from_center;

            if TOOL_INSIDE_ARC(side, turn) {
                // tool is inside the arc
                dist_from_center = arc_radius - tool_radius;
                toward_nominal = cy + tool_radius;
                double l = toward_nominal / dist_from_center;
                //CHKS((l > 1.0 || l < -1.0), _("Arc move in concave corner cannot be reached by the tool without gouging"));
                if(turn > 0) {
                    angle_from_center = theta + asin(l);
                } else {
                    angle_from_center = theta - asin(l);
                }
            } else {
                dist_from_center = arc_radius + tool_radius;
                toward_nominal = cy - tool_radius;
                double l = toward_nominal / dist_from_center;

                if(l > 1.0 || l < -1.0){
                	printf("Arc move in concave corner cannot be reached by the tool without gouging\n");
                	return 0;
                }

                if(turn > 0) {
                    angle_from_center = theta + M_PIl - asin(l);
                } else {
                    angle_from_center = theta + M_PIl + asin(l);
                }
            }

            midx = centerx + dist_from_center * cos(angle_from_center);
            midy = centery + dist_from_center * sin(angle_from_center);

            move_endpoint_and_flush(settings, midx, midy);
        } else {
            // arc->arc
            struct arc_feed &prev = qc().front().data.arc_feed;
            double oldrad = hypot(prev.center2 - prev.end2, prev.center1 - prev.end1);
            double newrad;
            if TOOL_INSIDE_ARC(side, turn) {
                newrad = arc_radius - tool_radius;
            } else {
                newrad = arc_radius + tool_radius;
            }

            double arc_cc, pullback, cc_dir, a;
            arc_cc = hypot(prev.center2 - centery, prev.center1 - centerx);

            if(oldrad == 0 || arc_cc == 0){
            	printf("Arc to arc motion is invalid because the arcs have the same center\n");
            	return 0;
            }

            a = (oldrad*oldrad + arc_cc*arc_cc - newrad*newrad) / (2 * oldrad * arc_cc);

            if(a > 1.0 || a < -1.0){
            	printf("Arc to arc motion makes a corner the compensated tool can't fit in without gouging\n");
            	return 0;
            }

            pullback = acos(a);
            cc_dir = atan2(centery - prev.center2, centerx - prev.center1);

            double dir;
            if TOOL_INSIDE_ARC(side, prev.turn) {
                if(turn > 0)
                    dir = cc_dir + pullback;
                else
                    dir = cc_dir - pullback;
            } else {
                if(turn > 0)
                    dir = cc_dir - pullback;
                else
                    dir = cc_dir + pullback;
            }

            midx = prev.center1 + oldrad * cos(dir);
            midy = prev.center2 + oldrad * sin(dir);

            move_endpoint_and_flush(settings, midx, midy);
        }
        enqueue_ARC_FEED(settings, block->line_number,
                         find_turn(opx, opy, centerx, centery, turn, end_x, end_y),
                         new_end_x, new_end_y, centerx, centery, turn, end_z,
                         AA_end, BB_end, CC_end, u, v, w);
    } else if (beta > small) {           /* convex, two arcs needed */
        midx = opx + tool_radius * cos(delta);
        midy = opy + tool_radius * sin(delta);
        dequeue_canons(settings);
        enqueue_ARC_FEED(settings, block->line_number,
                         0.0, // doesn't matter since we won't move this arc's endpoint
                         midx, midy, opx, opy, ((side == LEFT) ? -1 : 1),
                         cz,
                         AA_end, BB_end, CC_end, u, v, w);
        dequeue_canons(settings);
        set_endpoint(midx, midy);
        enqueue_ARC_FEED(settings, block->line_number,
                         find_turn(opx, opy, centerx, centery, turn, end_x, end_y),
                         new_end_x, new_end_y, centerx, centery, turn, end_z,
                         AA_end, BB_end, CC_end, u, v, w);
    } else {                      /* convex, one arc needed */
        dequeue_canons(settings);
        set_endpoint(cx, cy);
        enqueue_ARC_FEED(settings, block->line_number,
                         find_turn(opx, opy, centerx, centery, turn, end_x, end_y),
                         new_end_x, new_end_y, centerx, centery, turn, end_z,
                         AA_end, BB_end, CC_end, u, v, w);
    }

    comp_set_programmed(settings, end_x, end_y, end_z);
    comp_set_current(settings, new_end_x, new_end_y, end_z);

    return 0;
}

int Interp::convert_cutter_compensation_off(setup_pointer settings)
{
	if(settings->cutter_comp_side && settings->cutter_comp_radius > 0.0 &&
		!settings->cutter_comp_firstmove) {
		double cx, cy, cz;
		comp_get_current(settings, &cx, &cy, &cz);
		move_endpoint_and_flush(settings, cx, cy);
		dequeue_canons(settings);
		settings->current_x = settings->program_x;
		settings->current_y = settings->program_y;
		settings->current_z = settings->program_z;
		settings->arc_not_allowed = true;
	}
	settings->cutter_comp_side = false;
	settings->cutter_comp_firstmove = true;
	return 0;
}

int Interp::convert_cutter_compensation_on(int side, double radius,
										   setup_pointer settings)
{
	settings->cutter_comp_radius = radius;
	settings->cutter_comp_side = side;
	return 0;
}

int Interp::convert_straight(int move,
		                     block_pointer block,
		                     setup_pointer settings)
{
	double end_x;
	double end_y;
	double end_z;
	settings->arc_not_allowed = false;

	double AA_end;
	double BB_end;
	double CC_end;
	double u_end, v_end, w_end;
	int status;

	find_ends(block, settings, &end_x, &end_y, &end_z);

	if((settings->cutter_comp_side) &&
	   (settings->cutter_comp_radius > 0.0))
	{
        if (settings->cutter_comp_firstmove)
            status = convert_straight_comp1(move, block, settings, end_x, end_y, end_z,
                                            AA_end, BB_end, CC_end, u_end, v_end, w_end);
        else
            status = convert_straight_comp2(move, block, settings, end_x, end_y, end_z,
                                            AA_end, BB_end, CC_end, u_end, v_end, w_end);
	}else if (move == 0) {
		STRAIGHT_TRAVERSE(block->line_number, end_x, end_y, end_z,
						  AA_end, BB_end, CC_end,
						  u_end, v_end, w_end);
		settings->current_x = end_x;
		settings->current_y = end_y;
		settings->current_z = end_z;
	} else if (move == 10) {
		STRAIGHT_FEED(block->line_number, end_x, end_y, end_z,
				  AA_end, BB_end, CC_end,
				  u_end, v_end, w_end);
		settings->current_x = end_x;
		settings->current_y = end_y;
		settings->current_z = end_z;
	}

	return 0;
}

int Interp::convert_straight_comp1(int move,
		                           block_pointer block,
								   setup_pointer settings,
								   double px, double py, double pz,
								   double AA_end, double BB_end, double CC_end,
								   double u_end, double v_end, double w_end)
{
	double alpha;
	double distance;
	double radius = settings->cutter_comp_radius; /* always will be positive */
	double end_x, end_y;

	int side = settings->cutter_comp_side;
	double cx, cy, cz;

	comp_get_current(settings, &cx, &cy, &cz);
	distance = hypot((px - cx), (py - cy));

	if(side != LEFT and side != RIGHT){
		printf("NCE_BUG_SIDE_NOT_RIGHT_OR_LEFT\n");
	}

	if(distance <= radius){
		printf("Length of cutter compensation entry move is not greater than the tool radius");
		return 0;
	}

	alpha = atan2(py - cy, px - cx) + (side == LEFT ? M_PIl/2. : -M_PIl/2.);

	end_x = (px + (radius * cos(alpha)));
	end_y = (py + (radius * sin(alpha)));

	// with these moves we don't need to record the direction vector.
	// they cannot get reversed because they are guaranteed to be long
	// enough.

	set_endpoint(cx, cy);

	if (move == 0) {
		enqueue_STRAIGHT_TRAVERSE(settings, block->line_number,
								  cos(alpha), sin(alpha), 0,
								  end_x, end_y, pz,
								  AA_end, BB_end, CC_end, u_end, v_end, w_end);
	}
	else if (move == 1) {
		enqueue_STRAIGHT_FEED(settings, block->line_number,
							  cos(alpha), sin(alpha), 0,
							  end_x, end_y, pz,
							  AA_end, BB_end, CC_end, u_end, v_end, w_end);
	}else{
		printf("NCE_BUG_CODE_NOT_G0_OR_G1\n");
	}

	settings->cutter_comp_firstmove = false;

	comp_set_current(settings, end_x, end_y, pz);
	//settings->AA_current = AA_end;
	//settings->BB_current = BB_end;
	//settings->CC_current = CC_end;
	//settings->u_current = u_end;
	//settings->v_current = v_end;
	//settings->w_current = w_end;
	comp_set_programmed(settings, px, py, pz);
	return 0;
}

int Interp::convert_straight_comp2(int move,
		                           block_pointer block,
								   setup_pointer settings,
								   double px, double py, double pz,
								   double AA_end, double BB_end, double CC_end,
								   double u_end, double v_end, double w_end)
{
    double alpha;
    double beta;
    double end_x, end_y, end_z;                 /* x-coordinate of actual end point */
    double gamma;
    double mid_x, mid_y;                 /* x-coordinate of end of added arc, if needed */
    double radius;
    int side;
    double small = TOLERANCE_CONCAVE_CORNER;      /* radians, testing corners */
    double opx = 0, opy = 0, opz = 0;      /* old programmed beginning point */
    double theta;
    double cx, cy, cz;
    int concave;

    comp_get_current(settings, &cx, &cy, &cz);
    comp_get_current(settings, &end_x, &end_y, &end_z);
    comp_get_programmed(settings, &opx, &opy, &opz);

    if ((py == opy) && (px == opx)) {     /* no XY motion */
        if (move == 0) {
            enqueue_STRAIGHT_TRAVERSE(settings, block->line_number,
                                      px - opx, py - opy, pz - opz,
                                      cx, cy, pz,
                                      AA_end, BB_end, CC_end, u_end, v_end, w_end);
        } else if (move == 1) {
            enqueue_STRAIGHT_FEED(settings, block->line_number,
                                  px - opx, py - opy, pz - opz,
                                  cx, cy, pz, AA_end, BB_end, CC_end, u_end, v_end, w_end);
        } else
            printf("NCE_BUG_CODE_NOT_G0_OR_G1\n");
        	//ERS(NCE_BUG_CODE_NOT_G0_OR_G1);
        // end already filled out, above
    } else {
        // some XY motion
        side = settings->cutter_comp_side;
        radius = settings->cutter_comp_radius;      /* will always be positive */
        theta = atan2(cy - opy, cx - opx);
        alpha = atan2(py - opy, px - opx);

        if (side == LEFT) {
            if (theta < alpha)
                theta = (theta + (2 * M_PIl));
            beta = ((theta - alpha) - M_PI_2l);
            gamma = M_PI_2l;
        } else if (side == RIGHT) {
            if (alpha < theta)
                alpha = (alpha + (2 * M_PIl));
            beta = ((alpha - theta) - M_PI_2l);
            gamma = -M_PI_2l;
        } else{
        	printf("NCE_BUG_SIDE_NOT_RIGHT_OR_LEFT\n");
        }

        end_x = (px + (radius * cos(alpha + gamma)));
        end_y = (py + (radius * sin(alpha + gamma)));
        mid_x = (opx + (radius * cos(alpha + gamma)));
        mid_y = (opy + (radius * sin(alpha + gamma)));

        if ((beta < -small) || (beta > (M_PIl + small))) {
            concave = 1;
        } else if (beta > (M_PIl - small) &&
                   (!qc().empty() && qc().front().type == QARC_FEED &&
                    ((side == RIGHT && qc().front().data.arc_feed.turn > 0) ||
                     (side == LEFT && qc().front().data.arc_feed.turn < 0)))) {
            // this is an "h" shape, tool on right, going right to left
            // over the hemispherical round part, then up next to the
            // vertical part (or, the mirror case).  there are two ways
            // to stay to the "right", either loop down and around, or
            // stay above and right.  we're forcing above and right.
            concave = 1;
        } else {
            concave = 0;
            mid_x = (opx + (radius * cos(alpha + gamma)));
            mid_y = (opy + (radius * sin(alpha + gamma)));
        }

        if (!concave && (beta > small)) {       /* ARC NEEDED */
            //CHP(move_endpoint_and_flush(settings, cx, cy));
            if(move == 1) {
                enqueue_ARC_FEED(settings, block->line_number,
                                 0.0, // doesn't matter, since we will not move this arc's endpoint
                                 mid_x, mid_y, opx, opy,
                                 ((side == LEFT) ? -1 : 1), cz,
                                 AA_end, BB_end, CC_end, u_end, v_end, w_end);
                dequeue_canons(settings);
                set_endpoint(mid_x, mid_y);
            } else if(move == 0) {
                // we can't go around the corner because there is no
                // arc traverse.  but, if we do this anyway, at least
                // most of our rapid will be parallel to the original
                // programmed one.  if nothing else, this will look a
                // little less confusing in the preview.
                enqueue_STRAIGHT_TRAVERSE(settings, block->line_number,
                                          0.0, 0.0, 0.0,
                                          mid_x, mid_y, cz,
                                          AA_end, BB_end, CC_end,
                                          u_end, v_end, w_end);
                dequeue_canons(settings);
                set_endpoint(mid_x, mid_y);
            } //else ERS(NCE_BUG_CODE_NOT_G0_OR_G1);
        } else if (concave) {
            if (qc().front().type != QARC_FEED) {
                // line->line
                double retreat;
                // half the angle of the inside corner
                double halfcorner = (beta + M_PIl) / 2.0;
                //CHKS((halfcorner == 0.0), (_("Zero degree inside corner is invalid for cutter compensation")));
                retreat = radius / tan(halfcorner);
                // move back along the compensated path
                // this should replace the endpoint of the previous move
                mid_x = cx + retreat * cos(theta + gamma);
                mid_y = cy + retreat * sin(theta + gamma);
                // we actually want to move the previous line's endpoint here.  That's the same as
                // discarding that line and doing this one instead.
                //CHP(move_endpoint_and_flush(settings, mid_x, mid_y));
            } else {
                // arc->line
                // beware: the arc we saved is the compensated one.
                arc_feed prev = qc().front().data.arc_feed;
                double oldrad = hypot(prev.center2 - prev.end2, prev.center1 - prev.end1);
                double oldrad_uncomp;

                // new line's direction
                double base_dir = atan2(py - opy, px - opx);
                double theta;
                double phi;

                theta = (prev.turn > 0) ? base_dir + M_PI_2l : base_dir - M_PI_2l;
                phi = atan2(prev.center2 - opy, prev.center1 - opx);
                if TOOL_INSIDE_ARC(side, prev.turn) {
                    oldrad_uncomp = oldrad + radius;
                } else {
                    oldrad_uncomp = oldrad - radius;
                }

                double alpha = theta - phi;
                // distance to old arc center perpendicular to the new line
                double d = oldrad_uncomp * cos(alpha);
                double d2;
                double angle_from_center;

                if TOOL_INSIDE_ARC(side, prev.turn) {
                    d2 = d - radius;
                    double l = d2/oldrad;
                    //CHKS((l > 1.0 || l < -1.0), _("Arc to straight motion makes a corner the compensated tool can't fit in without gouging"));
                    if(prev.turn > 0)
                        angle_from_center = - acos(l) + theta + M_PIl;
                    else
                        angle_from_center = acos(l) + theta + M_PIl;
                } else {
                    d2 = d + radius;
                    double l = d2/oldrad;
                    //CHKS((l > 1.0 || l < -1.0), _("Arc to straight motion makes a corner the compensated tool can't fit in without gouging"));
                    if(prev.turn > 0)
                        angle_from_center = acos(l) + theta + M_PIl;
                    else
                        angle_from_center = - acos(l) + theta + M_PIl;
                }
                mid_x = prev.center1 + oldrad * cos(angle_from_center);
                mid_y = prev.center2 + oldrad * sin(angle_from_center);
                //CHP(move_endpoint_and_flush(settings, mid_x, mid_y));
            }
        } else {
            // no arc needed, also not concave (colinear lines or tangent arc->line)
            dequeue_canons(settings);
            set_endpoint(cx, cy);
        }
        (move == 0? enqueue_STRAIGHT_TRAVERSE: enqueue_STRAIGHT_FEED)
            (settings, block->line_number,
             px - opx, py - opy, pz - opz,
             end_x, end_y, pz,
             AA_end, BB_end, CC_end,
             u_end, v_end, w_end);
    }

    comp_set_current(settings, end_x, end_y, pz);
    //settings->AA_current = AA_end;
    //settings->BB_current = BB_end;
    //settings->CC_current = CC_end;
    //settings->u_current = u_end;
    //settings->v_current = v_end;
    //settings->w_current = w_end;
    comp_set_programmed(settings, px, py, pz);
    return 0;
}



double Interp::find_arc_length(double x1, double y1, double z1,
		                       double center_x, double center_y,
							   int turn,
							   double x2, double y2, double z2)
{
    return 0;
}

int Interp::find_ends(block_pointer block, setup_pointer s,
		              double *px, double *py, double *pz)
{
	int middle;
	int comp;

	middle = !s->cutter_comp_firstmove;
	comp = s->cutter_comp_side;

	if(block->x_flag) {
		*px = block->x_number;
	} else {
		*px = (comp && middle) ? s->program_x : s->current_x;
	}

	if(block->y_flag) {
		*py = block->y_number;
	} else {
		*py = (comp && middle) ? s->program_y : s->current_y;
	}

	if(block->z_flag) {
		*pz = block->z_number;
	} else {
		*pz = (comp && middle) ? s->program_z : s->current_z;
	}

	return 0;
}

int
Interp::find_relative(double x1, double y1, double z1,
					  double *x2, double *y2, double *z2,
					  setup_pointer settings)
{
    return 0;
}

double Interp::find_straight_length(double x2, double y2, double z2,
									double x1, double y1, double z1)
{
	return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2) + pow((z2 - z1), 2));
}

double Interp::find_turn(double x1, double y1,
		                 double center_x, double center_y,
						 int turn,
						 double x2, double y2)
{
    return 0;
}

int Interp::init_block(block_pointer block)
{
	int n;

	block->a_flag = false;
	block->b_flag = false;
	block->c_flag = false;

	block->i_flag = false;
	block->j_flag = false;
	block->k_flag = false;
	block->l_number = -1;
	block->line_number = -1;
	block->motion_to_be = -1;
	block->m_count = 0;

	block->p_number = -1.0;
	block->q_number = -1.0;
	block->r_flag = false;
	block->x_flag = false;
	block->y_flag = false;
	block->z_flag = false;

	return 0;
}


int Interp::move_endpoint_and_flush(setup_pointer s, double x, double y)
{

}

void dequeue_canons(setup_pointer settings)
{
	 if(qc().empty()) return;

	    for(unsigned int i = 0; i<qc().size(); i++) {
	        queued_canon &q = qc()[i];

	        switch(q.type) {
	        case QARC_FEED:
	            printf("issuing arc feed lineno %d\n", q.data.arc_feed.line_number);
	            ARC_FEED(q.data.arc_feed.line_number,
	                     q.data.arc_feed.end1,
	                     q.data.arc_feed.end2,
	                     q.data.arc_feed.center1,
	                     q.data.arc_feed.center2,
	                     q.data.arc_feed.turn,
	                     q.data.arc_feed.end3,
	                     q.data.arc_feed.a, q.data.arc_feed.b, q.data.arc_feed.c,
	                     q.data.arc_feed.u, q.data.arc_feed.v, q.data.arc_feed.w);
	            break;
	        case QSTRAIGHT_FEED:
	            printf("issuing straight feed lineno %d\n", q.data.straight_feed.line_number);
	            STRAIGHT_FEED(q.data.straight_feed.line_number,
	                          q.data.straight_feed.x,
	                          q.data.straight_feed.y,
	                          q.data.straight_feed.z,
	                          q.data.straight_feed.a, q.data.straight_feed.b, q.data.straight_feed.c,
	                          q.data.straight_feed.u, q.data.straight_feed.v, q.data.straight_feed.w);
	            break;
	        case QSTRAIGHT_TRAVERSE:
	            printf("issuing straight traverse lineno %d\n", q.data.straight_traverse.line_number);
	            STRAIGHT_TRAVERSE(q.data.straight_traverse.line_number,
	                              q.data.straight_traverse.x,
	                              q.data.straight_traverse.y,
	                              q.data.straight_traverse.z,
	                              q.data.straight_traverse.a, q.data.straight_traverse.b, q.data.straight_traverse.c,
	                              q.data.straight_traverse.u, q.data.straight_traverse.v, q.data.straight_traverse.w);
	            break;
	        }
	    }
	    qc().clear();
}

int enqueue_STRAIGHT_FEED(setup_pointer settings, int l,
                          double dx, double dy, double dz,
                          double x, double y, double z,
						  double a, double b, double c,
						  double u, double v, double w)
{

}

int enqueue_STRAIGHT_TRAVERSE(setup_pointer settings, int l,
                              double dx, double dy, double dz,
                              double x, double y, double z,
							  double a, double b, double c,
							  double u, double v, double w)
{

}

void enqueue_ARC_FEED(setup_pointer settings, int l,
                      double original_arclen,
                      double end1, double end2,
					  double center1, double center2,
                      int turn,
                      double end3,
					  double a, double b, double c,
					  double u, double v, double w)
{

}

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
			  double w_position)
{

}

void STRAIGHT_FEED(int line_number,
                   double x, double y, double z,
                   double a, double b, double c,
                   double u, double v, double w)
{

}

void STRAIGHT_TRAVERSE(int line_number,
                       double x, double y, double z,
                       double a, double b, double c,
                       double u, double v, double w)
{

}

void set_endpoint(double x, double y) {
    endpoint[0] = x; endpoint[1] = y;
    endpoint_valid = 1;
}




