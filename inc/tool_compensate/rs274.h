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

typedef struct block_struct
{
 bool     a_flag;
 double   a_number;
 bool     b_flag;
 double   b_number;
 bool     c_flag;
 double   c_number;

 char     comment[256];
 int      d_number;
 double   f_number;
 int      g_modes[14];

 bool     i_flag;
 double   i_number;
 bool     j_flag;
 double   j_number;
 bool     k_flag;
 double   k_number;
 int      l_number;
 int      line_number;
 int      motion_to_be;
 int      m_count;
 int      m_modes[10];
 double   p_number;
 double   q_number;
 bool     r_flag;
 double   r_number;
 double   s_number;
 int      t_number;
 bool     x_flag;
 double   x_number;
 bool     y_flag;
 double   y_number;
 bool     z_flag;
 double   z_number;
} block;

typedef block * block_pointer;

typedef struct setup_struct
{
 double AA_axis_offset;                        // A-axis g92 offset
 double AA_current;                            // current A-axis position
 double AA_origin_offset;                      // A-axis origin offset

 double BB_axis_offset;                        // B-axis g92offset
 double BB_current;                            // current B-axis position
 double BB_origin_offset;                      // B-axis origin offset


 double CC_axis_offset;                        // C-axis g92offset
 double CC_current;                            // current C-axis position
 double CC_origin_offset;                      // C-axis origin offset

 int active_g_codes [12];                // array of active G codes
 int active_m_codes [7];                // array of active M codes
 double active_settings [3];               // array of feed, speed, etc.
 double axis_offset_x;                         // X-axis g92 offset
 double axis_offset_y;                         // Y-axis g92 offset
 double axis_offset_z;                         // Z-axis g92 offset
 double current_x;                             // current X-axis position
 double current_y;                             // current Y-axis position
 double current_z;                             // current Z-axis position
 double origin_offset_x;                       // origin offset x
 double origin_offset_y;                       // origin offset y
 double origin_offset_z;                       // origin offset z

 block block1;                                 // parsed next block
 char blocktext[256];           // linetext downcased, white space gone
 int control_mode;               // exact path or cutting mode
 int current_slot;                             // carousel slot number of current tool

 double cutter_comp_radius;                    // current cutter compensation radius
 int cutter_comp_side;                         // current cutter compensation side
 double cycle_cc;                              // cc-value (normal) for canned cycles
 double cycle_i;                               // i-value for canned cycles
 double cycle_j;                               // j-value for canned cycles
 double cycle_k;                               // k-value for canned cycles
 int cycle_l;                                  // l-value for canned cycles
 double cycle_p;                               // p-value (dwell) for canned cycles
 double cycle_q;                               // q-value for canned cycles
 double cycle_r;                               // r-value for canned cycles
 int distance_mode;                  // absolute or incremental
 int feed_mode;                                // G_93 (inverse time) or G_94 units/min
 bool feed_override;                         // whether feed override is enabled
 double feed_rate;                             // feed rate in current units/min
 char filename[128];            // name of currently open NC code file
 FILE * file_pointer;                          // file pointer for open NC code file
 bool flood;                                 // whether flood coolant is on
 int length_offset_index;                      // for use with tool length offsets
 int length_units;                     // millimeters or inches
 int line_length;                              // length of line last read
 char linetext[256];            // text of most recent line read
 bool mist;                                  // whether mist coolant is on
 int motion_mode;                              // active G-code for motion
 int origin_index;                             // active origin (1=G54 to 9=G59.3)

 double parameters[5400];                // system parameters
 int parameter_occurrence;                     // parameter buffer index
 int parameter_numbers[50];                    // parameter number buffer
 double parameter_values[50];                  // parameter value buffer

 int plane;                            // active plane, XY-, YZ-, or XZ-plane
 bool percent_flag;                          // ON means first line was percent sign
 bool probe_flag;                            // flag indicating probing done
 double program_x;                             // program x, used when cutter comp on
 double program_y;                             // program y, used when cutter comp on
 int retract_mode;                    // for cycles, old_z or r_plane
 int selected_tool_slot;                       // tool slot selected but not active
 int sequence_number;                          // sequence number of line last read
 double speed;                                 // current spindle speed in rpm
 int speed_feed_mode;        // independent or synched
 bool speed_override;                        // whether speed override is enabled
 int spindle_turning;              // direction spindle is turning
 char stack[50][80];                           // stack of calls for error reporting
 int stack_index;                              // index into the stack
 double tool_length_offset;                    // current tool length offset
 int tool_max;                                 // highest number tool slot in carousel
 int tool_table_index;                         // tool index used with cutter comp
 double traverse_rate;                         // rate for traverse motions
} setup;

typedef setup * setup_pointer;




#endif /* INC_TOOL_COMPENSATE_RS274_H_ */
