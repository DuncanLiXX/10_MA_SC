/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file global_definition.h
 *@author gonghao
 *@date 2020/03/18
 *@brief 本头文件包含所有全局数值定义
 *@version
 */

#ifndef INC_GLOBAL_DEFINITION_H_
#define INC_GLOBAL_DEFINITION_H_
#include <stdint.h>
//#include "hmi_shared_data.h"

#define USES_FIVE_AXIS_FUNC     //五轴功能

//#define USES_LICENSE_FUNC     //授权功能

//#define USES_TCP_FILE_TRANS_KEEP     //tcp文件传输连接保持，不用每次重新建立连接

#define USES_GENERAL_AUTOMATIC  //通用自动化
//#define USES_GRIND_MACHINE      //磨床相关功能启用
//#define USES_LASER_MACHINE      //激光设备功能启用
//#define USES_WUXI_BLOOD_CHECK   //无锡血液检测
//#define USES_TIANJIN_PROJ            //天津6通道机床定制

//#define USES_TWINING_FUNC      //缠绕功能

//#define USES_MAIN_BOARD_10MA_OLD          //老的10MA控制板

//#define USES_PMC_PROCESS     //由PMC处理PMC轴的所有操作  鲁匠定制

#define MOVE_FILTER_PARAM_FROM_CHN_TO_AXIS     //将插补后加减速参数从通道参数挪到轴参数

#define USES_SPEED_TORQUE_CTRL      // 速度控制与力矩控制开关

#define USES_PMC_2_0         //使用PMC2.0版本

#define USES_TOOL_COMPENSATE    //支持刀具半径补偿

#define USES_MODBUS_PROTOCAL   //支持组态通讯协议

#define USES_PHYSICAL_MOP       //使用实体MOP

//#define USES_INDEPEND_BASE_TOOL_OFFSET    //使用独立的基准刀偏置

//#define USES_T_CODE_MACRO        //T指令使用宏程序执行

#define USES_PEITIAN_SMALL_FIVE   //支持配天小五轴
//#define USES_RET_REF_TO_MACH_ZERO   //支持回参考点动作最后走到机械零位置


//#define USES_STOP_SPD_IN_RESET   //支持复位时停主轴

#define USES_MC_B_BOARD       //支持MC-B主板

#define USES_WOOD_MACHINE     //支持木工专机

#define USES_ADDITIONAL_PROGRAM     //支持前置、后置程序调用

#define USES_EMERGENCY_DEC_STOP   //支持急停减速停

// #define USES_SIMULATION_TEST   //仿真测试，仿真数据保存为文件


//版本信息
#define ADX_SC_VERSION "P0.3.01" //SC模块软件版本号

//文件目录定义
#define PATH_NC_FILE "/cnc/nc_files/"    //NC加工文件目录0
#define PATH_NC_SUB_FILE "/cnc/nc_files/sys_sub/"   //系统子程序目录
#define PATH_NC_SIGN_FILE "/cnc/nc_files/md5/"   //NC文件签名文件目录
#define PATH_LOG_FILE "/cnc/trace/log/"	//LOG文件目录
#define PATH_ALARM_FILE "/cnc/trace/alarm/"	//告警文件目录
#define PATH_CONFIG_FILE "/cnc/config/"	//配置文件目录
#define PATH_PMC_FILE "/cnc/pmc/"          //PMC文件目录
#define PATH_PMC_DATA "/cnc/pmc/pmc_run.dat"   //PMC运行文件路径
#define PATH_PMC_LDR "/cnc/pmc/pmc_ladder.ldr"	//PMC梯形图文件路径
#define PATH_MDA_FILE "/cnc/mda/mda_%hhu.nc"    //存放MDA代码的文件
#define PATH_UPDATE_PATH "/cnc/update/"    //存放升级文件路径
#define PATH_BOOT_CONFIG_FILE "/cnc/config/bootinfo.cfg"   //启动序号文件
#define PATH_SPARTAN6_PROGRAM "/cnc/bin/top.bit"     //spartan6代码文件
#define PATH_SPARTAN6_PROGRAM_BAK "/cnc/config/sys_top.cfg"	//spartan6代码文件备份
#define PATH_MI_PROGRAM "/cnc/bin/MI.bin"            //MI模块程序文件
#define PATH_MI_PROGRAM_BAK "/cnc/config/MI_BAK.bin"     //MI模块程序备份文件
#define PATH_MODBUS_PROGRAM "/cnc/bin/SCModbus_upd.elf"     //Modbus模块程序升级文件
#define PATH_PMC_REG "/cnc/pmc/pmc_register.dat"		//PMC非易失性寄存器保存文件
#define PATH_MACRO_VAR_KEEP "/cnc/config/macro_var/macro_variable_%hhu.dat"   //非易失性宏变量保存文件
#define PATH_USER_MACRO_VAR "/cnc/config/macro_var/user_macro_variable_%hhu.dat"
#define PATH_PHY_AXIS_ENCODER "/cnc/config/phy_encoder.dat"   //物理轴当前编码器反馈
#define PATH_CONFIG_PACK_TMP "/cnc/config/config_pack_tmp.cpu"   //参数打包文件临时存放路径
#define PATH_PC_DATA_TMP "/cnc/config/pc_data_input_tmp.dat"    //螺补导入文件临时存放路径
#define PATH_ESB_FILE "/cnc/svo_esb/"    //伺服描述文件存放路径
#define PATH_UPDATE_DISK_CMD "/cnc/diskup/scripts/scup.sh" //一键升级SC脚本
#define PATH_BACK_DISK_CMD "/cnc/disk/scripts/scback.sh" //一键备份SC脚本

#define PATH_SIMULATE_DATA "/cnc/tmp/simulate_data.dat"   //仿真数据文件


//配置文件路径
#define SYS_CONFIG_FILE "/cnc/config/sys_config.ini"   //系统配置文件
#define CHN_CONFIG_FILE "/cnc/config/chn_config.ini"   //通道配置文件
#define AXIS_CONFIG_FILE "/cnc/config/axis_config.ini"  //轴配置文件
#define PITCH_COMP_FILE "/cnc/config/pc_data.ini"       //螺补数据文件
#define TOOL_CONFIG_FILE "/cnc/config/tool_config.ini"  //刀具配置文件
#define WORK_COORD_FILE  "/cnc/config/work_coord.ini"   //工件坐标系配置文件
#define EX_WORK_COORD_FILE "/cnc/config/ex_work_coord.ini" //扩展工件坐标系配置文件
#define CHN_SCENE_FILE "/cnc/config/chn_scene.ini"		//保存通道的状态参数
#define IO_REMAP_FILE "/cnc/config/ioremap.ini"       //IO重定向数据文件

#define CHN_PROC_PARAM_FILE "/cnc/config/chn_proc_param.ini"   //工艺相关通道参数文件
#define AXIS_PROC_PARAM_FILE "/cnc/config/axis_proc_param.ini"   //工艺相关通道参数文件

#ifdef USES_FIVE_AXIS_FUNC
#define FIVE_AXIS_CONFIG_FILE "/cnc/config/five_axis_config.ini"    //五轴配置文件
#endif

#ifdef USES_GRIND_MACHINE
#define GRIND_CONFIG_FILE "/cnc/config/grind_config.ini"   //磨削专用参数配置文件
#endif


#define BOARD_TEMPATURE "/sys/bus/i2c/devices/1-0048/hwmon/hwmon0/temp1_input"   //主板温度保存文件
//

#define MI_START_REG            (0xFFFFFFF0)			//MI启动寄存器，注意不在共享内存范围内，需要单独映射处理

#define uchar unsigned char

//平面定义
#define PLANE_XY (170)
#define PLANE_ZX (180)
#define PLANE_YZ (190)

//NC编程模式
#define ABS_MODE (0)  //绝对坐标编程
#define RELATIVE_MODE (1)    //相对坐标编程

#define MZERO (1E-6)  //定义0值精度
#define GetModeGroup(x) GCode2Mode[x/10]


//单位转换
#define MM2NM0_1(x) (int64_t)(x * 1e7)		//从mm转换为0.1nm
#define FEED_TRANS(x) (int32_t)(x * 1000 / 60)   //由mm/min转换为um/s
#define FEED_TRANS_REV (double)((x * 60)/1000.)   //由um/s转换回mm/min


#define SET_SECTION8_LED _IO('x', 1) //设置8段管
#define GET_SECTION8_LED _IO('x', 2) //获取8段管
#define GET_GPIO_VALUE _IO('x', 3) //获取GPIO 电平 Bit0:欠压1:过压2:电池电压低3:轴过流4:USB过流5:485发送使能6:系统报警7:Nand写保护
#define SET_NOTIFY_PID _IO('x', 4) //设置通知PID
#define GET_NOTIFY_PID _IO('x', 5) //获取通知PID
#define SET_NOTIFY_INTERVAL _IO('x', 6) //设置通知间隔(ns)
#define GET_NOTIFY_INTERVAL _IO('x', 7) //设置通知间隔(ns)
#define SET_RS485_EN _IO('x', 8) //设置485发送使能
#define SET_SYS_WARN _IO('x', 9) //设置系统报警
#define SET_NAND_PROTECTED _IO('x', 10) //设置NAND写保护
#define SIMULATE_ALARM_INTERRUPT _IO('x', 11) //摸拟产生报警中断



//MC状态刷新延时
#define MC_STATE_REFRESH_CYCLE (10000)    //刷新周期为5ms，稳妥起见使用10ms

const int64_t kAxisRefNoDef =  0xffffffffffffffff;    //绝对式编码器轴零点未定义


const int kThreadStackSize = 1024 * 1024;   //线程栈大小， 1M

const int kMonitorInteval = 50000;    //HMI监控数据发送间隔，单位us

//通道相关
//const int kOutputBufCount = 20;	//运控数据帧缓冲大小
//const int kMaxOutputMsgCount = 100;  //输出指令消息列表的允许最大节点数
const uint8_t kMaxRatio = 150;		//倍率最大值150%


//扩展工件坐标系
const int kMaxExWorkCoordCount = 99;		//扩展坐标系最大个数

const double EPSINON = 1e-6;   //浮点精度

//通道引擎索引号，对于告警等信息，与具体通道无关
const uint8_t CHANNEL_ENGINE_INDEX = 0XFF;    //通道引擎索引号

//轴无关，对于告警等信息
const uint8_t NO_AXIS = 0XFF;

const uint32_t kSp6FifoDepth = 16;    //spartan6数据通道FIFO深度

//子程序号的最大长度
const int kMaxSubNameLen = 6;    //子程序号范围：0~999999

//顺序号的最大长度
const int kMaxLineNoLen = 9;    //顺序号范围：0~999999999

//M指令数字最大长度
const int kMaxMCodeLen = 9;    //范围：0~999999999

//S指令数字最大长度
const int kMaxSCodeLen = 9;    //范围：0~999999999

//子程序最大嵌套层数
const int kMaxSubNestedCount = 16;    //子程序最大嵌套16层

//WHILE-DO识别号最大值
const int kMaxDoIndex = 8;

const uint8_t kMaxChnSpdCount = 6;	 //通道内主轴最大数量

const int kMaxOutputBufCount = 1000;    //输出队列的最大容量，主要用于手轮反向引导

const int kTCodeSubProg = 9000;     //T指令对应的子程序号

const int kMCodeTimeout = 500000;    //M指令判断合法性的超时时间  单位：us

//ESB文件发送给MI的数据字节数
const int kEsbDataSize = 998;

const int kGraphPosBufCount = 300;    //图形位置数据缓冲，用于仿真绘图的高频位置数据

//轴名称允许字符
const char strAxisNames[] = "XYZABCUVW";

//spartan6程序加载管脚定义
#define SP6_PROGRAM "963"   //906+57
#define SP6_INIT "962"      //906+56;
#define SP6_CLK "960"          //906+54;
#define SP6_DATA  "964"         //906+58;
#define SP6_DONE  "961"         //906+55;

#define SP6_PROG_DIR "/sys/class/gpio/gpio963/direction"
#define SP6_INIT_DIR "/sys/class/gpio/gpio962/direction"
#define SP6_CLK_DIR "/sys/class/gpio/gpio960/direction"
#define SP6_DATA_DIR "/sys/class/gpio/gpio964/direction"
#define SP6_DONE_DIR "/sys/class/gpio/gpio961/direction"

#define SP6_PROG_VALUE "/sys/class/gpio/gpio963/value"
#define SP6_INIT_VALUE "/sys/class/gpio/gpio962/value"
#define SP6_CLK_VALUE "/sys/class/gpio/gpio960/value"
#define SP6_DATA_VALUE "/sys/class/gpio/gpio964/value"
#define SP6_DONE_VALUE "/sys/class/gpio/gpio961/value"


//SC模块启动步骤
enum ModuleStartStep{
	STEP_POWER_ON = 0,		//系统上电
	STEP_INIT_TRACE,		//初始化日志模块
	STEP_LOAD_SP6_DATA,		//加载SPARTAN6的代码
	STEP_LOAD_MI,			//加载MI模块的代码
	STEP_INIT_PARM_MANAGER,	//初始化配置管理模块
	STEP_INIT_HMI_COMM,		//初始化HMI通讯接口
	STEP_INIT_MI_COMM,		//初始化MI通讯接口
	STEP_INIT_MC_COMM,		//初始化MC通讯接口
	STEP_INIT_ALARM_PROC,	//初始化告警处理模块
	STEP_INIT_CHN_ENGINER,	//初始化通道引擎

	STEP_ALL_DONE			//初始化完成
};



/*
 * @brief 坐标系类型
 */
enum CoordId {
	MACHINE_COORD,
	G54_COORD,
	G55_COORD,
	G56_COORD,
	G57_COORD,
	G58_COORD,
	G59_COORD,
	G5401_COORD,
	G5402_COORD,
	G5403_COORD,
	G5404_COORD,
	G5405_COORD,
	G5406_COORD,
	G5407_COORD,
	G5408_COORD,
	G5409_COORD,
	G5410_COORD,
	G5411_COORD,
	G5412_COORD,
	G5413_COORD,
	G5414_COORD,
	G5415_COORD,
	G5416_COORD,
	G5417_COORD,
	G5418_COORD,
	G5419_COORD,
	G5420_COORD,
	G5421_COORD,
	G5422_COORD,
	G5423_COORD,
	G5424_COORD,
	G5425_COORD,
	G5426_COORD,
	G5427_COORD,
	G5428_COORD,
	G5429_COORD,
	G5430_COORD,
	G5431_COORD,
	G5432_COORD,
	G5433_COORD,
	G5434_COORD,
	G5435_COORD,
	G5436_COORD,
	G5437_COORD,
	G5438_COORD,
	G5439_COORD,
	G5440_COORD,
	G5441_COORD,
	G5442_COORD,
	G5443_COORD,
	G5444_COORD,
	G5445_COORD,
	G5446_COORD,
	G5447_COORD,
	G5448_COORD,
	G5449_COORD,
	G5450_COORD,
	G5451_COORD,
	G5452_COORD,
	G5453_COORD,
	G5454_COORD,
	G5455_COORD,
	G5456_COORD,
	G5457_COORD,
	G5458_COORD,
	G5459_COORD,
	G5460_COORD,
	G5461_COORD,
	G5462_COORD,
	G5463_COORD,
	G5464_COORD,
	G5465_COORD,
	G5466_COORD,
	G5467_COORD,
	G5468_COORD,
	G5469_COORD,
	G5470_COORD,
	G5471_COORD,
	G5472_COORD,
	G5473_COORD,
	G5474_COORD,
	G5475_COORD,
	G5476_COORD,
	G5477_COORD,
	G5478_COORD,
	G5479_COORD,
	G5480_COORD,
	G5481_COORD,
	G5482_COORD,
	G5483_COORD,
	G5484_COORD,
	G5485_COORD,
	G5486_COORD,
	G5487_COORD,
	G5488_COORD,
	G5489_COORD,
	G5490_COORD,
	G5491_COORD,
	G5492_COORD,
	G5493_COORD,
	G5494_COORD,
	G5495_COORD,
	G5496_COORD,
	G5497_COORD,
	G5498_COORD,
	G5499_COORD,
	GUARD_COORD			//卫兵
};

//MC模块工作模式
enum MCWorkMode{
	MC_MODE_MANUAL = 0, 	//手动模式
	MC_MODE_AUTO,			//自动模式
	MC_MODE_MDA				//MDA模式
};

enum ManualMoveDir{
	DIR_NEGATIVE = -1,		//负向
	DIR_STOP = 0,			//停止
	DIR_POSITIVE = 1,		//正向
};

enum NonGModeType{
	T_MODE = 0,			//T
	D_MODE,				//D
	H_MODE,				//H
	F_MODE,				//F
	S_MODE				//S
};


enum AxisInterfaceType{
	VIRTUAL_AXIS = 0,	//虚拟轴
	BUS_AXIS,			//总线轴
	ANALOG_AXIS			//模拟轴
};

enum MotorFeedBackMode{
	INCREMENTAL_ENCODER = 0,   //增量式编码器
	ABSOLUTE_ENCODER_YASAKAWA,	//绝对式编码器，安川，ABZ反馈
	ABSOLUTE_ENCODER_PANASONIC, //绝对式编码器，松下,U485反馈
	LINEAR_ENCODER,				//光栅尺
	NO_ENCODER                  //无反馈
};


/**
 * @brief MI的控制字地址定义
 */
enum MiCtrlOperate{
	AXIS_ON_FLAG = 0,			//轴使能
	MOTOR_ON_FLAG,
	MOTION_LEFT_LMT_FLAG,
	HW_LMT_EN_FLAG,
	HW_LMT_SENSE_FLAG,
	SW_LMT_EN_FLAG,
	HW_LMT_EN_ONCE_FLAG,
	RESERVED_1,
	RUN_DIR_FLAG,
	SRC_DEV_FLAGS,
	CAPT_COM_FLAG = 12,
	CAPT_EN_FLAG,
	CAPT_SIGNAL_FLAGS = 15,
	MOTION_RIGHT_LMT_FLAG = 17,
	PLC_INTPLA_PIMIT,
	MANU_POS_UPT_FLAG,
	CTRL_MODE_FLAG,
	PLAN_MODE_FLAGS,
	AUTO_MDI_INTPLA_PIMIT = 23,
	MANU_INTPLA_PIMIT,
	PULSE_DIR_FLAG,
	START_LMT_FLAG,
	AUTO_SMOOTH_STOP_FLAG,
	MDI_INTP_DIR_FLAG,
	AUTO_INTP_DIR_FLAG,
	RESERVED_2,
	AXIS_OFF_CHECK_FLAG

};

/**
 * @brief 定义主轴旋转方向
 */
enum SpdDirect{
	SPD_DIR_NEGATIVE = -1,		//反转
	SPD_DIR_STOP = 0,			//停转
	SPD_DIR_POSITIVE = 1		//正转
};

/**
 * @brief 轴类型定义
 */
enum AxisType{
	AXIS_LINEAR = 0,		//直线轴
	AXIS_ROTATE,			//旋转轴
	AXIS_SPINDLE,			//主轴
	AXIS_TORQUE             //扭矩控制轴
};

/**
 * @brief 直线轴类型定义
 */
enum LinearAxisType{
	LINE_AXIS_NONE = 0,		//未定义
	LINE_AXIS_X,			//基本轴X
	LINE_AXIS_Y,			//基本轴Y
	LINE_AXIS_Z,			//基本轴Z
	LINE_AXIS_PX,			//平行轴X
	LINE_AXIS_PY,			//平行轴Y
	LINE_AXIS_PZ			//平行轴Z
};

/**
 * @brief 轴名称定义
 */
enum AxisName{
	AXIS_NAME_X = 0,        //X轴
	AXIS_NAME_Y,			//Y轴
	AXIS_NAME_Z,			//Z轴
	AXIS_NAME_A,			//A轴
	AXIS_NAME_B,			//B轴
	AXIS_NAME_C,			//C轴
	AXIS_NAME_U,			//U轴
	AXIS_NAME_V,			//V轴
	AXIS_NAME_W				//W轴
};
/**
 * @brief 手轮跟踪状态定义
 */
enum HWTraceState{
	NONE_TRACE = 0,		//非手轮跟踪状态
	NORMAL_TRACE,		//正向手轮跟踪
	REVERSE_TRACE       //反向手轮跟踪
};


#ifdef 	USES_LASER_MACHINE
const int kHYDRangeCount = 4;    //HYD(弘宇达)调高器量程种类数
enum HYDRangeIdx{
	HYD_RANGE_10 = 0,     //10mm量程
	HYD_RANGE_15,			//15mm量程
	HYD_RANGE_20,			//20mm量程
	HYD_RANGE_25			//25mm量程
};
const double kHYDCalibRefPos[kHYDRangeCount][16] = {10.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.5, 2.0, 1.8, 1.5, 1.2, 1.0, 0.7, 0.5, 0.2,
		                                               15.0, 12.0, 10.5, 9.0, 7.5, 6.0, 4.5, 3.75, 3.0, 2.7, 2.25, 1.8, 1.5, 1.05, 0.75, 0.3,
													   20.0, 16.0, 14.0, 12.0, 10.0, 8.0, 6.0, 5.0, 4.0, 3.6, 3.0, 2.4, 2.0, 1.4, 1.0, 0.4,
													   25.0, 20.0, 17.5, 15.0, 12.5, 10.0, 7.5, 6.25, 5.0, 4.5, 3.75, 3.0, 2.5, 1.75, 1.25, 0.5};
#endif

#ifdef USES_FIVE_AXIS_FUNC
enum FiveAxisMachType{
    NO_FIVEAXIS = 0,    	// 五轴关闭
    C_A_FIVEAXIS = 1,   	// C-A类型五轴机床(双摆头型)
    C_B_FIVEAXIS = 2,  		// C-B类型五轴机床(双摆头型)
    A_B_FIVEAXIS= 3,   		//  A-B类型五轴机床(双摆头型)
    B_A_FIVEAXIS = 4,   	// B-A类型五轴机床(双摆头型)
    A1_C1_FIVEAXIS = 5,   	//C'-A'类型五轴机床(双转台型
    B1_C1_FIVEAXIS = 6,    	//C'-B'类型五轴机床(双转台型
    A1_B1_FIVEAXIS = 7,   	// A'-B'类型五轴机床(双转台型
    B1_A1_FIVEAXIS = 8,  	// B'-A'类型五轴机床(双转台型)
    C1_A_FIVEAXIS= 9,   	//  C'-A类型五轴机床(混合型)
    C1_B_FIVEAXIS = 10,   	// C'-B类型五轴机床(混合型)
    A1_B_FIVEAXIS = 11,  	//A'-B类型五轴机床(混合型)
    B1_A_FIVEAXIS = 12,    	//B'-A类型五轴机床(混合型)
	C_A_TILT_FIVEAXIS = 13, //C-A倾斜型五轴机床
};

enum FiveAxisParamType{
	MACHINE_TYPE = 1,       //五轴类型
	X_OFFSET_1 = 11,        //第一转轴中心相对刀尖点X坐标
	Y_OFFSET_1 = 12,        //第一转轴中心相对刀尖点Y坐标
	Z_OFFSET_1 = 13,        //第一转轴中心相对刀尖点Z坐标
	X_OFFSET_2 = 14,        //第二转轴中心相对刀尖点X坐标
	Y_OFFSET_2 = 15,        //第二转轴中心相对刀尖点Y坐标
	Z_OFFSET_2 = 16,        //第二转轴中心相对刀尖点Z坐标
	ROT_SWING_ARM_LEN = 17, //刀具旋转摆臂长度
	POST_DIR_1 = 18,        //第一转轴旋转正方向
	POST_DIR_2 = 19,        //第二转轴旋转正方向

	SPEED_LIMIT = 21,		//五轴速度限制开关

	SPEED_LIMIT_X = 22,    //五轴联动X轴速度限制
	SPEED_LIMIT_Y,         //五轴联动Y轴速度限制
	SPEED_LIMIT_Z,         //五轴联动Z轴速度限制
	SPEED_LIMIT_A,         //五轴联动A轴速度限制
	SPEED_LIMIT_B,         //五轴联动B轴速度限制
	SPEED_LIMIT_C,         //五轴联动C轴速度限制

	SPEED_PLAN_MODE = 30,   //五轴速度规划方式
	INTP_ANGLE_STEP = 31,   //五轴联动粗插补角度步距
    INTP_LEN_STEP = 32,      //五轴联动粗插补最小长度
	PROGRAM_COORD = 33      //五轴编程坐标系方式
};
#endif

#ifdef USES_GRIND_MACHINE
//磨削通道定义
enum GrindChannel{
	GRIND_MAIN_CHN = 0,     //主通道
	GRIND_SLAVE_CHN,         //从通道
	GRIND_NONE_CHN = 0xFF    //非法通道

};

//上下料通道运行状态定义
enum MechArmRunState{
	MECH_ARM_IDLE = 0,		//空闲
	MECH_ARM_RUN = 10,      //运行
	MECH_ARM_PAUSE = 20		//暂停
};

//上下料机械手异常状态
enum MechArmErrState{
	ERR_ARM_NONE = 0,    //正常
	ERR_ARM_NO_TRAY,     //未检测到料盘
	ERR_ARM_FETCH_VAC,   //料盘取料失败，吸真空异常
	ERR_ARM_SET_VAC,     //料盘放料失败，吸真空异常
	ERR_ARM_CORR_VAC,    //对位失败，吸真空异常
	ERR_ARM_CORR_POS,    //对位失败，对位气缸不在位
	ERR_ARM_WORK_FETCH_VAC,  //工位取料失败，吸真空异常
	ERR_ARM_WORK_SET_VAC,    //工位放料失败，吸真空异常
	ERR_ARM_CONFIRM_WORK,    //确认工位是否有料
	ERR_ARM_UP,			 //机械臂上升不到位
	ERR_NEW_TRAY_REQ,    //请求更换料盘
	ERR_ARM_AXIS_NO_REF,  //机械臂未回零
	ERR_ARM_WORK_UP_LEFT,  //左机工位气缸上升不到位
	ERR_ARM_WORK_UP_RIGHT  //右机工位气缸上升不到位

};

//机械手手动状态
enum MechArmManualState{
	ARM_MANUAL_IDLE = 0,		//空闲
	ARM_MANUAL_LEFT_WORK,       //左工位
	ARM_MANUAL_RIGHT_WORK,      //右工位
	ARM_MANUAL_REPOS,           //对位平台
	ARM_MANUAL_LEFT_TRAY,       //左料盘
	ARM_MANUAL_RIGHT_TRAY       //右料盘
};

#define ARM_PMC_X  (5)   //机械臂X轴对应的物理轴号
#define ARM_PMC_Y  (6)   //机械臂Y轴对应的物理轴号
#endif

#endif /* INC_GLOBAL_DEFINITION_H_ */
