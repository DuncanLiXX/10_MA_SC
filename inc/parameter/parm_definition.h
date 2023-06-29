/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file parm_definition.h
 *@author gonghao
 *@date 2020/05/06
 *@brief 本头文件为各类参数结构的声明
 *@version
 */

#ifndef INC_PARAMETER_PARM_DEFINITION_H_
#define INC_PARAMETER_PARM_DEFINITION_H_
#include "global_include.h"

const int kWorkCoordCount = 7;		//工件坐标系总数，G54~G59，包括一个基本坐标系

//参数类型枚举定义
enum ParmType{
	SYS_CONFIG = 1,    //系统配置  1
	CHN_CONFIG,			//通道配置  2
	AXIS_CONFIG,		//轴配置   3
	TOOL_OFFSET_CONFIG,		//刀具偏置配置  4
	TOOL_POT_CONFIG,		//刀具位置配置  5
	COORD_CONFIG,		//坐标系配置 6
	EX_COORD_CONFIG,	//扩展坐标系配置 7
	FIVE_AXIS_CONFIG,	//五轴配置 8

	PROCESS_PARAM,         //工艺参数 9

	GRIND_CONFIG = 10,		//磨床专用配置

	PITCH_COMP_DATA = 13,    //螺补数据
	IO_REMAP_DATA = 14,      //IO重映射数据
    HANDWHEEL_MAP = 15,      //手轮通道映射
    FIVEAXIS_V2_CONFIG = 16, // 新五轴配置


	CHN_STATE_SCENE = 100		//通道当前状态

};

/**
 * @brief SC模块系统配置数据
 */
struct SCSystemConfig{
	uint8_t cnc_mode;			        //设备类型		//由授权码给定，不可修改
	uint8_t max_chn_count;				//最大通道数	//由授权码给定，不可修改
	uint8_t max_axis_count;				//最大轴数		//由授权码给定，不可修改
	uint8_t chn_count;					//通道数量
	uint8_t axis_count;			     	//系统物理轴数量
	uint8_t axis_name_ex;				//轴名称扩展下标    0--关闭		1--打开
	uint8_t fix_ratio_find_ref;			//回参考点时倍率固定	0--关闭		1--打开
	//uint8_t pc_type;					//螺距补偿方式		0--单向螺补  1--双向螺补
//	uint8_t mcp_count;                  //MCP数量

	uint8_t hw_code_type;               //手轮编码方式     0--二进制编码    1--格雷码

	uint32_t io_filter_time;			//普通IO滤波时间	//单位：微秒
	uint32_t fast_io_filter_time;		//快速IO滤波时间	//单位：微秒
	uint32_t free_space_limit;			//剩余空间限制      //单位：KB
	uint16_t backlight_delay_time;		//背光延时时间		//单位：秒

	uint8_t bus_cycle;					//总线通讯周期		0-8:125us/250us/500us/1ms/2ms/4ms/5ms/8ms/10ms

	uint8_t save_lineno_poweroff;		//掉电保存当前运行行号		0--关闭		1--打开
	uint8_t manual_ret_ref_mode;		//手动回参考点模式		0-首次 		1--每次

	uint8_t beep_time;					//蜂鸣时间			//单位：秒

	uint8_t da_ocp;						//DA过流保护	0--关闭		1--打开
	uint8_t da_prec;				    //DA器件精度（位数）  10-64

	uint8_t alarm_temperature;			//主板告警温度
	uint8_t trace_level;				//调试跟踪信息记录级别
	uint8_t debug_mode;					//调试模式		0--关闭    1--模式1    2--模式2    3--模式3

	uint8_t hw_rev_trace;               //手轮反向引导功能   0--关闭   1--打开

    /**********************位置开关***************************/
    uint8_t pos_check_id_1;             //位置开关1
    uint8_t pos_check_id_2;             //位置开关2
    uint8_t pos_check_id_3;             //位置开关3
    uint8_t pos_check_id_4;             //位置开关4
    uint8_t pos_check_id_5;             //位置开关5
    uint8_t pos_check_id_6;             //位置开关6
    uint8_t pos_check_id_7;             //位置开关7
    uint8_t pos_check_id_8;             //位置开关8

    double pos_check_min_1;             //位置开关最小值1
    double pos_check_min_2;             //位置开关最小值2
    double pos_check_min_3;             //位置开关最小值3
    double pos_check_min_4;             //位置开关最小值4
    double pos_check_min_5;             //位置开关最小值5
    double pos_check_min_6;             //位置开关最小值6
    double pos_check_min_7;             //位置开关最小值7
    double pos_check_min_8;             //位置开关最小值8

    double pos_check_max_1;             //位置开关最大值1
    double pos_check_max_2;             //位置开关最大值2
    double pos_check_max_3;             //位置开关最大值3
    double pos_check_max_4;             //位置开关最大值4
    double pos_check_max_5;             //位置开关最大值5
    double pos_check_max_6;             //位置开关最大值6
    double pos_check_max_7;             //位置开关最大值7
    double pos_check_max_8;             //位置开关最大值8
    /**********************位置开关***************************/
};

/**
 * @brief SC模块通道配置数据
 */
struct SCChannelConfig{
	uint8_t chn_index;  		//通道索引号
	uint8_t chn_axis_count;     //通道轴数量
	uint8_t chn_group_index;    //通道所属方式组编号
//	uint8_t chn_axis_x;			//基本轴X对应的物理轴索引
//	uint8_t chn_axis_y;			//基本轴Y对应的物理轴索引
//	uint8_t chn_axis_z;			//基本轴Z对应的物理轴索引
	uint8_t chn_axis_phy[kMaxAxisChn];	//轴对应的物理轴索引

	uint8_t chn_axis_name[kMaxAxisChn];		//轴名称
	uint8_t chn_axis_name_ex[kMaxAxisChn];	//轴名称下标

	uint8_t intep_mode;				//插补模式   0：XYZ三轴插补  1：所有轴插补
	uint8_t intep_cycle;			//插补周期	0-8:125us/250us/500us/1ms/2ms/4ms/5ms/8ms/10ms
	uint16_t chn_precision;			//加工精度，单位：微米
	uint8_t chn_look_ahead;			//前瞻功能 	0--关闭   1--打开
	uint8_t chn_feed_limit_level;	//加工速度调整等级  0--关闭   1~10表示等级1~等级10
	uint8_t zmove_stop;				//下刀准停  0--关闭    1--打开
	uint8_t corner_stop_enable;		//拐角准停功能开关
	uint8_t corner_stop;			//拐角准停  0-180度
	uint8_t corner_stop_angle_min;  //拐角准停下限角度 0-180度
	uint8_t corner_acc_limit;		//拐角加速度限制	0--关闭   1--打开
	uint8_t long_line_acc;			//长直线运行加速    0--关闭   1--打开

	uint8_t arc_err_limit;			//圆弧误差限制      0-255  单位：um

	uint8_t chn_spd_limit_on_axis;  	//基于轴速度差的转角速度钳制		0--关闭   1--打开
	uint8_t chn_spd_limit_on_acc;		//基于加速度的小线段进给速度钳制		0--关闭   1--打开
	uint8_t chn_spd_limit_on_curvity;	//基于曲率及向心加速的小线段进给速度钳制		0--关闭   1--打开
	uint8_t chn_rapid_overlap_level;	//G00重叠等级	0--关闭  1~3--等级1~等级3

	double chn_max_vel;					//通道允许最大速度	//单位：mm/min
	double chn_max_acc;					//通道最大加速度	//单位：mm/s^2
	double chn_max_dec;					//通道最大减速度	//单位：mm/s^2
	double chn_max_corner_acc;          //最大拐角加速度		//单位：mm/s^2
	double chn_max_arc_acc;				//最大向心加速度	//单位：mm/s^2
	uint16_t chn_s_cut_filter_time;		//切削进给S型规划时间常数  //单位：ms

    double tap_max_acc;                 //刚性攻丝最大加速度	//单位：mm/s^2
    double tap_max_dec;                 //刚性攻丝最大减速度	//单位：mm/s^2
    int8_t tap_plan_mode;              //刚性攻丝规划模式  0：T型  1：S型
    uint16_t tap_s_cut_filter_time;     //刚性攻丝S型规划时间常数  //单位：ms

    uint8_t default_plane;			//默认平面	   0：G17  1：G18  2：G19
    uint8_t default_cmd_mode;		//默认指令模式   0：G90  1：G91
    uint8_t default_feed_mode;      //默认进给方式   0：G00  1：G01  2：G02  3：G03

	uint8_t rapid_mode;				//G00模式    0：定位     1：直线定位
	uint8_t cut_plan_mode;			//切削规划模式	0：T型   1：S型
	uint8_t rapid_plan_mode;		//定位规划模式  0：T型   1：S型
	uint8_t ex_coord_count;			//扩展工件坐标系个数  0-99

	uint8_t change_tool_mode;		//换刀方式     0--手动    1--自动
	uint8_t tool_live_check;		//刀具寿命检测  0--关闭   1--打开
	uint8_t auto_tool_measure;		//自动对刀		0--关闭   1--打开

	uint8_t gcode_trace;			//加工代码跟踪   0--关闭   1--打开
	uint8_t gcode_unit;				//G代码编程单位		0--公制    1--英制

	uint8_t timing_mode;			//加工计时方式    0--全部计时    1--切削计时

	uint16_t chn_small_line_time;   //小线段执行时间常数   单位：0.1ms   范围[0, 10000]

	uint32_t g31_skip_signal;       //G31跳转信号，例如X1004.7保存为10047，即乘10倍
	uint8_t g31_sig_level;          //G31跳转信号有效电平    0--低电平    1--高电平
    uint16_t rst_hold_time;         //复位时间 单位:ms
    uint8_t rst_mode;             //复位时是否保留运行数据  0:不保留 1:保留
    uint32_t g01_max_speed;         //G01最高进给速度 单位：mm/min
    uint16_t mpg_level3_step;       //手轮3档的自定义步长 单位：um
    uint16_t mpg_level4_step;       //手轮4档的自定义步长 单位：um

    double G73back;		// G73回退距离
    double G83back;		// G83回退距离

    uint8_t tool_number;    // 刀号数
#ifdef USES_WOOD_MACHINE
	int debug_param_1;             //调试参数1
	int debug_param_2;             //调试参数2
	int debug_param_3;             //调试参数3
	int debug_param_4;             //调试参数4
	int debug_param_5;             //调试参数5
	
	int flip_comp_value;           //挑角补偿    -10000~10000    单位：um
#endif

};

/**
 * @brief SC模块轴配置数据
 */
struct SCAxisConfig{
	uint8_t axis_index;						//轴索引号	0-11，系统自动生成
	uint8_t axis_type;						//轴类型,    0--直线轴     1--旋转轴     2--主轴/速度轴    3--力矩轴
	uint8_t axis_interface;					//轴接口类型	0--虚拟轴    1-总线轴    2--非总线轴
	uint8_t axis_port;						//从站号（总线轴）/对应轴口号（非总线轴）1~12
	uint8_t axis_linear_type;				//直线轴类型
	uint8_t axis_pmc;						//是否PMC轴

	double kp1;								//自动比例参数
	double kp2;								//手动比例参数
	double ki;								//积分参数
	double kd;								//微分参数
	double kil;								//积分饱和限
	uint16_t kvff;							//速度前馈系数
	uint16_t kaff;							//加速度前馈系数

	uint32_t track_err_limit;				//跟随误差限制 单位：um
	uint16_t location_err_limit;			//定位误差限制 单位：um

//	uint8_t soft_limit_check;				//软限位检测	0--关闭   1--打开
	uint8_t save_pos_poweroff;				//掉电坐标记忆		0--关闭   1--打开
//	uint8_t axis_alarm_level;               //轴告警信号电平    0--低电平  1--高电平


	uint32_t motor_count_pr;				//电机每转计数
	uint32_t motor_speed_max;				//电机最大转速   单位：rpm
	double move_pr;							//每转移动量，即丝杆螺距    单位：mm(deg)
	uint8_t motor_dir;						//电机旋转方向    0--正转    1--反转
    uint8_t feedback_mode;					//电机反馈类型    0--增量式(或单圈绝对式)    1--绝对式(多圈绝对式)


    uint8_t ret_ref_mode;					//0--无挡块回零  1--有挡块回零
    uint8_t absolute_ref_mode;              //绝对式电机回零方式 0--回零标记点设定方式    1--无挡块回零方式
	uint8_t ret_ref_dir;					//回参考点方向		0--负向    1--正向
	uint8_t ret_ref_change_dir;				//回参考点换向      0--反向    1--同向
	uint8_t ref_signal;						//参考点信号类型     0--零信号    1--Z信号
	uint8_t ret_ref_index;					//回参考点顺序		0-11:第一~第十二
	uint8_t ref_base_diff_check;			//粗精基准位置偏差检测		0--关闭   1--打开
	double ref_base_diff;					//粗精基准位置偏差		单位：mm（deg）
	int64_t ref_encoder;					//零点编码器值，绝对式编码器有效
    double ref_offset_pos;                //回参考点后的偏移量 单位:mm
    double ref_z_distance_max;            //搜索Z脉冲最大移动距离

	double manual_speed;					//手动进给速度，单位:mm/min
	double rapid_speed;						//定位速度，单位:mm/min
	double reset_speed;						//复位速度，单位：mm/min
	double ret_ref_speed;					//回参考点速度，单位：mm/min
    double ret_ref_speed_second;            //回参考点低速度，单位：mm/min

	double rapid_acc;   					//定位加速度，单位:mm/s^2
	double manual_acc;						//手动加速度，单位:mm/s^2
	double start_acc;						//起步加速度，单位:mm/s^2

	double corner_acc_limit;				//拐角速度差限制，单位：mm/min

	uint16_t rapid_s_plan_filter_time;			//G00 S型速度规划时间常数

	uint8_t post_filter_type;		    //插补后滤波器类型		0--关闭    1--平滑滤波     2--均值滤波     3--S型均值滤波器
	uint16_t post_filter_time_1;		//插补后滤波器一级时间常数  单位：ms
	uint16_t post_filter_time_2;        //插补后滤波器二级时间常数  单位：ms

	uint8_t backlash_enable;					//反向间隙是否生效
    //int16_t backlash_forward;					//正向反向间隙，单位：um(deg)->mm(deg)
    //int16_t backlash_negative;					//负向反向间隙，单位：um(deg)->mm(deg)
    double backlash_forward;					//正向反向间隙，单位：mm(deg)
    double backlash_negative;					//负向反向间隙，单位：mm(deg)
    int16_t backlash_step;                      //反向间隙步长，单位：um(20-300)

	uint8_t  pc_type;							//螺补类型  0 单向螺补  1 双向螺补
	uint8_t  pc_enable;							//螺补是否生效
	uint16_t pc_offset;                         //螺补起始点   1-10000
	uint16_t pc_count;							//螺距误差补偿点数  0-10000
	uint16_t pc_ref_index;						//参考点补偿位置 1-10000
	double pc_inter_dist;						//螺距补偿间隔，单位：mm(deg)

	double soft_limit_max_1;					//正向软限位1
	double soft_limit_min_1;					//负向软限位1
	uint8_t soft_limit_check_1;					//软限位1有效
	double soft_limit_max_2;					//正向软限位2
	double soft_limit_min_2;					//负向软限位2
	uint8_t soft_limit_check_2;					//软限位2有效
	double soft_limit_max_3;					//正向软限位3
	double soft_limit_min_3;					//负向软限位3
	uint8_t soft_limit_check_3;					//软限位3有效



	uint8_t off_line_check;						//轴断线检测       0--关闭   1--打开

	uint8_t ctrl_mode;							//轴控制方式	   0--脉冲方向    1--正交脉冲    2--CW/CCW    3--模拟量
	uint32_t pulse_count_pr;					//每转输出脉冲数
	uint8_t encoder_lines;						//编码器单圈线数   10~31
	uint16_t encoder_max_cycle;					//编码器整圈最大值


	//主轴相关参数
    int16_t zero_compensation;					//零漂补偿(DA电平值) 0~4095
	uint8_t spd_gear_ratio;						//主轴变速比
	uint32_t spd_max_speed;						//主轴最高转速	单位：rpm
	uint16_t spd_start_time;					//主轴启动时间  单位：0.1s
	uint16_t spd_stop_time;						//主轴制动时间  单位：0.1s
	uint8_t spd_vctrl_mode;						//主轴模拟电压控制方式    0--禁止    1 -- 0V~10V     2-- -10V~10V
//	uint8_t spd_set_dir;						//主轴设置方向    0--反转    1--正转  //使用电机旋转方向替代
	uint32_t spd_set_speed;						//主轴设置转速  单位：rpm
	uint8_t spd_speed_check;					//主轴转速检测   0--关闭   1--打开
	uint8_t spd_speed_match;					//转速匹配检测   0--关闭   1--打开
	uint8_t spd_speed_diff_limit;				//转速误差允许比例   0~100
	uint8_t spd_encoder_location;				//编码器位置     0--电机端     1--主轴端

    uint8_t spd_ctrl_GST;                       //SOR信号用途  0：主轴定向  1：齿轮换挡
    uint8_t spd_ctrl_SGB;                       //主轴换挡方式 0：A方式  1：B方式
    uint8_t spd_ctrl_SFA;                       //齿轮换挡时是否输出SF信号 0：输出  1：不输出
    uint8_t spd_ctrl_ORM;                       //主轴定向时的转向 0：正  1：负
    uint8_t spd_ctrl_TCW;                       //主轴转向是否受M03/M04影响 0：不受影响 1：受影响
    uint8_t spd_ctrl_CWM;                       //主轴转向取反 0：否  1：是
    uint8_t spd_ctrl_TSO;                       //螺纹切削和刚性攻丝时，主轴倍率设置  0：强制100%  1：有效
    uint16_t spd_analog_gain;                   //模拟输出增益 700-1250 单位：0.1%
    uint16_t spd_sor_speed;                     //主轴齿轮换档/定向时的主轴转速 rpm
    uint16_t spd_motor_min_speed;               //主轴电机最小钳制速度
    uint16_t spd_motor_max_speed;               //主轴电机最大钳制速度
    uint16_t spd_gear_speed_low;                //齿轮低档位最高转速 rpm
    uint16_t spd_gear_speed_middle;             //齿轮中档位最高转速 rpm
    uint16_t spd_gear_speed_high;               //齿轮高档位最高转速 rpm
    uint16_t spd_gear_switch_speed1;            //B方式档1->档2电机转速
    uint16_t spd_gear_switch_speed2;            //B方式档2->档3电机转速
    double spd_sync_error_gain;               //同步误差增益  范围为0~1000，默认为200
    double spd_speed_feed_gain;               //轴速度前馈增益 范围为0~100000，默认为60000
    double spd_pos_ratio_gain;                //轴位置比例增益 范围0~200000，默认为100000
    uint8_t spd_rtnt_rate_on;                   //攻丝回退期间，倍率是否有效 0：强制100%  1：有效
    uint8_t spd_rtnt_rate;                      //攻丝回退倍率 单位：1%
    int32_t spd_rtnt_distance;                 //攻丝回退的额外回退值 单位：um

    double spd_locate_ang;                     //主轴定向角度 单位：度

	//旋转轴相关参数
	uint8_t fast_locate;							//快速定位    0--关闭   1--打开
	uint8_t pos_disp_mode;						//位置显示模式   0--循环模式（0~360）    1--非循环模式
    uint8_t pos_work_disp_mode;                 //工件坐标是否循环显示  0--否  1--是
    uint8_t pos_rel_disp_mode;                  //相对坐标是否循环显示  0--否  1--是
    uint8_t rot_abs_dir;                        //旋转轴绝对指令的旋转方向  0--快捷方向  1--取决于指令符号

	//同步轴相关参数
	uint8_t sync_axis;							//是否同步轴  0--否   1--是
    uint8_t series_ctrl_axis;                   //是否串联控制 0--否   1--同向   2--反向
    uint8_t master_axis_no;						//主动轴号
	uint8_t disp_coord;							//是否显示坐标  0--否   1--是
	uint8_t auto_sync;							//从动轴回参考点后自动同步校准   0--否   1--是
    uint32_t sync_err_max_pos;					//位置同步误差报警阈值	单位：um
	double benchmark_offset;					//主从轴基准位置偏差  单位：mm    
    uint8_t sync_pre_load_torque;               //预载电流偏置 单位：1%
    uint8_t sync_err_max_torque;                //扭矩同步误差报警阈值 单位：1%
    uint32_t sync_err_max_mach;                 //坐标同步误差报警阈值 单位：um
    uint8_t sync_pos_detect;                    //是否进行位置同步误差检测 0--否   1：是
    uint8_t sync_mach_detect;                   //是否进行坐标同步误差检测 0--否   1：是
    uint8_t sync_torque_detect;                 //是否进行扭矩同步误差检测 0--否   1：是
    uint16_t serial_torque_ratio;               //串联力矩系数 单位: 1%
    uint16_t serial_pre_speed;                  //预载串联速度 单位：rpm

	double axis_home_pos[10];				//参考点位置  单位：mm
	double ref_mark_err;                   //参考点基准误差   单位：mm    有效范围：0~10.0
	uint32_t spd_min_speed;                 //主轴最低转速   单位：rpm   0~100000

    uint8_t pmc_g00_by_EIFg;                //PMC轴倍率控制
    uint16_t pmc_min_speed;                 //最小PMC移动速度
    uint16_t pmc_max_speed;                 //最大PMC移动速度

    int ref_complete;                       //参考点是否建立
    bool init_backlash_dir;                 //反向间隙初始化方向

    uint8_t decelerate_numerator;           //减速比例分子
    uint8_t decelerate_denominator;         //减速比例分母
};

/**
 * @brief SC模块刀具偏置配置数据
 */
struct SCToolOffsetConfig{
#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
	double geometry_comp_basic[3];                     //基准刀偏置
#endif
	double geometry_compensation[kMaxToolCount][3];    //刀具几何偏置，包括长度补偿
	double geometry_wear[kMaxToolCount];				//刀具长度磨损补偿
	double radius_compensation[kMaxToolCount];			//刀具半径补偿
	double radius_wear[kMaxToolCount];					//刀具半径磨损补偿
};

/**
 * @brief SC模块刀位配置数据，刀号与导套映射表
 */
struct SCToolPotConfig{
//	uint8_t tool_index;		//刀号
	uint8_t tool_type[kMaxToolCount];		//刀具类型
	uint8_t huge_tool_flag[kMaxToolCount];           //是否大刀柄，大刀柄意味着一把刀需要占三个刀位
	uint8_t tool_pot_index[kMaxToolCount];			//刀套号
	int tool_life_max[kMaxToolCount];								//刀具寿命，单位：min
	int tool_life_cur[kMaxToolCount];								//已使用寿命，单位：min
	int tool_threshold[kMaxToolCount];                             //刀具寿命预警阈值
    uint8_t tool_life_type[kMaxToolCount];                          //刀具计寿类型，0--关闭,1--换刀次数，2--切削时间，3--切削距离
};


/**
 * @brief 新五轴配置参数
 */
struct SCFiveAxisV2Config
{
    uint8_t machine_type;               //五轴结构类型 0:非五轴  1:双摆头  2:摇篮  3:混合
    uint8_t pivot_master_axis;          //第一旋转轴 0:X  1:Y  2:Z
    uint8_t pivot_slave_axis;           //第二旋转轴 0:X  1:Y  2:Z
    double table_x_position;            //旋转台x坐标 单位:mm
    double table_y_position;            //旋转台y坐标 单位:mm
    double table_z_position;            //旋转台z坐标 单位:mm
    double table_x_offset;              //第二旋转轴x偏移 单位:mm
    double table_y_offset;              //第二旋转轴y偏移 单位:mm
    double table_z_offset;              //第二旋转轴z偏移 单位:mm
    uint8_t tool_dir;                   //刀具轴向 0:X  1:Y  2:Z
    double tool_holder_offset_x;        //刀柄x偏移 单位:mm
    double tool_holder_offset_y;        //刀柄y偏移 单位:mm
    double tool_holder_offset_z;        //刀柄z偏移 单位:mm
    uint8_t table_master_dir;           //第一旋转轴方向  0:正  1:负
    uint8_t table_slave_dir;            //第二旋转轴  0:正  1:负
    double master_ref_angle_crc;        //第一旋转轴初始角度 单位:度
    double slave_ref_angle_crc;         //第二旋转轴初始角度 单位:度
    double tool_holder_length;          //刀柄长度 单位:mm
};

/**
 * @brief 工件坐标系配置
 */
struct SCCoordConfig{
	double offset[kMaxAxisChn];    //轴工件坐标系偏置
};

/**
 * @brief 工件坐标系更新数据结构
 */
struct CoordUpdate{
	uint8_t chn_index;  //通道号
	uint8_t coord_index;	//坐标系索引  0--通用偏置  1-6:G54~G59
	SCCoordConfig value;
};

/**
 * @brief HMI模块刀具偏置配置数据
 */
//struct HmiToolOffsetConfig{
//	double geometry_compensation[3];    //刀具几何偏置，包括长度补偿
//	double geometry_wear;				//刀具长度磨损补偿
//	double radius_compensation;			//刀具半径补偿
//	double radius_wear;					//刀具半径磨损补偿
//};

/**
 * @brief 刀具偏置更新数据结构
 */
struct ToolOffsetUpdate{
	uint8_t chn_index;  //通道号
	uint8_t tool_offset_index;	//刀偏索引  0--H1
	HmiToolOffsetConfig value;
};

/**
 * @brief 轴螺补数据表
 */
struct AxisPitchCompTable{
	double *pc_table[kMaxAxisNum];   //分轴存储，动态分配，先正向螺补，后负向螺补
};

/**
 * @brief 参数值联合体，用于保存参数设置时的参数值
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
 * @brief 用于保存参数修改数据
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
 * @brief 用于保存工艺相关参数修改数据
 */
struct ProcParamUpdate{
	uint8_t group_index;    //工艺参数组
	ParamUpdate param_data; //待更新参数数据
};

//IO重映射的数据结构定义
struct IoRemapInfo{
	uint8_t  iotype;	//0：输入X 1：输出Y
	uint16_t addr;	    //字节地址，例如X10.2，此处放10
	uint8_t  bit;		//位地址：0-7
	uint8_t  addrmap;	//是否重映射为新的地址 0：不映射，1：映射
	uint16_t newaddr;	//新字节地址
	uint8_t  newbit;	//新位地址：0-7
	uint8_t  valtype;	//值类型： 0：原值，1：反相 2：常高 3：常低
};

//手轮通道映射
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

//工艺相关通道参数组
struct ChnProcParamGroup{
	ProcessParamChn chn_param[kMaxProcParamCount];
};

//工艺相关轴参数组
struct AxisProcParamGroup{
	ProcessParamAxis axis_param[kMaxProcParamCount];
};

typedef ListBuffer<ParamUpdate> UpdateParamList;   //待生效参数队列
typedef ListBuffer<CoordUpdate> UpdateCoordList;		//待生效的工件坐标系队列
typedef ListBuffer<ToolOffsetUpdate> UpdateToolOffsetList;     //待生效的刀具偏置队列

typedef ListBuffer<ProcParamUpdate> UpdateProcParamList;   //待生效工艺相关参数队列


typedef ListBuffer<IoRemapInfo> IoRemapList;      //IO重定向数据队列


#endif /* INC_PARAMETER_PARM_DEFINITION_H_ */
