/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file global_structure.h
 *@author gonghao
 *@date 2020/03/18
 *@brief 本头文件包含所有全局数据结构定义
 *@version
 */

#ifndef INC_GLOBAL_STRUCTURE_H_
#define INC_GLOBAL_STRUCTURE_H_

#include <stdint.h>
#include <cstddef>
#include <sys/mman.h>
#include <string>
#include "hmi_shared_data.h"
#include "list_buffer.h"
#include "global_definition.h"
//#include "global_include.h"


#define PAGE_SIZE ((size_t)getpagesize())
#define PAGE_MASK ((uint64_t)(long)(PAGE_SIZE - 1))

//大端模式小端模式转换函数
#define BigLittleSwitch16(A)   ((( (ushort)(A) & 0xff00) >> 8)   | \
                                       (( (ushort)(A) & 0x00ff) << 8))

#define BigLittleSwitch32(A)   ((( (unsigned int)(A) & 0xff000000) >> 24) | \
                                       (( (unsigned int)(A) & 0x00ff0000) >> 8)   | \
                                       (( (unsigned int)(A) & 0x0000ff00) << 8)   | \
                                       (( (unsigned int)(A) & 0x000000ff) << 24))

#define BigLittleSwitch64(A)   ((( (unsigned long long)(A) & 0xff00000000000000) >> 56) | \
					      	  (( (unsigned long long)(A) & 0x00ff000000000000) >> 40) | \
							  (( (unsigned long long)(A) & 0x0000ff0000000000) >> 24) | \
                                       (( (unsigned long long)(A) & 0x000000ff00000000) >> 8)   | \
                                       (( (unsigned long long)(A) & 0x00000000ff000000) << 8)   | \
                                       (( (unsigned long long)(A) & 0x0000000000ff0000) << 24)  | \
                                       (( (unsigned long long)(A) & 0x000000000000ff00) << 40)  | \
                                       (( (unsigned long long)(A) & 0x00000000000000ff) << 56))



/**
 * @brief 用于保存系统全局状态
 */
struct SystemState {
	bool system_ready;             	//系统启动并初始化完成标志，初始化为false，当所有模块加载初始化完成赋为true
	uint8_t module_ready_mask;      //各模块就绪mask，每个模块占一个bit
	bool system_quit;              	//系统退出标志，接到退出信号是此标志置true
	bool hmi_comm_ready;           	//与HMI的通讯准备好
//	bool hmi_ready;                //HMI就绪
	uint32_t hmi_alarm_code;        //HMI告警码
	bool eth0_running;             //网口是否可用
	uint8_t system_boot_stage;     	//系统启动步骤,标志系统启动的当前阶段
	char hmi_host_addr[16];        //当前连接的HMI主机IP地址
	char local_host_addr[16];      //本地IP地址

};

/**
 * @brief CPU占用率结构体
 */
struct CPULoad {
	char name[20];   //cpu名称
	uint32_t user;    //用户态耗时
	uint32_t nice;    //nice值为负线程耗时
	uint32_t system;  //系统耗时
	uint32_t idle;    //空闲耗时
};

/**
 * @brief 内存占用率结构体
 */
struct MemLoad {
	char name1[20];
	char name2[20];
	uint32_t total;
	uint32_t free;
	uint32_t buffer;
	uint32_t cached;
};

/**
 * @brief CoordInfo包含坐标系信息
 */
struct CoordInfo { //坐标系信息
	CoordInfo();
	void Init();
	void Reset();
	bool CheckMask(uint8_t index);
	void SetMask(uint8_t index, uint8_t value = 1); //用于设置mask 的第index位为 value值
//	double surface_vector[3];	//所选平面的法向量
	double build_coord_dest[kMaxAxisChn]; //建立工件（或机床）坐标系时输入的数据，G92/G52/G10共用
//	double wcs_offset_differ[kMaxAxisChn]; //当前工件坐标系相对于G54～G59的偏移，单位：mm
	uint8_t surface_flag; 	//02组，用于标识选择的平面，170:XY* 180:ZX* 190:YZ*
	uint8_t coord_id; 	//14组，工件坐标系选择，0:G53机床坐标系 1-6:G54*-G59 工件坐标系 G5401~G5499
	uint8_t build_coord_flag;   //建立坐标系类型。0:不建立坐标系；1：G92；2：G52 3:G10 4：G92.1复位
	uint8_t build_wcs_mask; 	//标记G92/G52/G10/G92.1使用的轴
};

/**
 * @brief spartan6 数据结构，用于读取文件写入pl
 */
union Sp6Data{
	uint32_t data_32[4];
	char buffer[16];
};

/**
 * @brief MI的控制字定义
 */
struct MiCtrlBits{
	uint64_t AXIS_ON_FLAG:1;			//轴使能
	uint64_t MOTOR_ON_FLAG:1;
	uint64_t MOTION_LEFT_LMT_FLAG:1;
	uint64_t HW_LMT_EN_FLAG:1;
	uint64_t HW_LMT_SENSE_FLAG:1;
	uint64_t SW_LMT_EN_FLAG:1;
	uint64_t HW_LMT_EN_ONCE_FLAG:1;
	uint64_t RESERVED_1:1;
	uint64_t RUN_DIR_FLAG:1;
	uint64_t SRC_DEV_FLAGS:3;
	uint64_t CAPT_COM_FLAG:1;
	uint64_t CAPT_EN_FLAG:2;
	uint64_t CAPT_SIGNAL_FLAGS:2;
	uint64_t MOTION_RIGHT_LMT_FLAG:1;
	uint64_t PLC_INTPLA_PIMIT:1;
	uint64_t MANU_POS_UPT_FLAG:1;
	uint64_t CTRL_MODE_FLAG:1;
	uint64_t PLAN_MODE_FLAGS:2;
	uint64_t AUTO_MDI_INTPLA_PIMIT:1;
	uint64_t MANU_INTPLA_PIMIT:1;
	uint64_t PULSE_DIR_FLAG:1;
	uint64_t START_LMT_FLAG:1;
	uint64_t AUTO_SMOOTH_STOP_FLAG:1;
	uint64_t MDI_INTP_DIR_FLAG:1;
	uint64_t AUTO_INTP_DIR_FLAG:1;
	uint64_t RESERVED_2:1;
	uint64_t AXIS_OFF_CHECK_FLAG:1;
	uint64_t RESERVED_3:32;

};

/**
 * @brief MI控制字寄存器定义
 */
union MiCtrlReg{
	uint64_t all;
	MiCtrlBits bits;
};

#ifdef USES_PMC_2_0
/**
 * @brief SD-LINK从站设备信息结构
 */
struct BdioDevInfo{
    int8_t group_index;   //设备号
    int8_t device_type;   //设备类型号 1.SA1(DI/DO 9/4), 3.SC1(DI/DO 16/16), 4.SD1(DI/DO 12/8), 5.SE1(DI/DO 7/8)
    int8_t base_index;    //基板号(预留)
    int8_t slot_index;    //槽位号(预留)
    int16_t in_bytes;     //输入字节数
    int16_t out_bytes;    //输出字节数
    int8_t input_start;   //输入起始点
    int8_t output_start;  //输出起始点
    int8_t handwheel_map; //手轮映射
    std::string info_name;//设备名称
};

/**
 * @brief SD_LINK扩展板卡规格
 */
struct SDLINK_SPEC {
    int8_t  inBytes;
    int8_t  outBytes;
    std::string info_name;
    bool    withHandWheel;
    SDLINK_SPEC(std::string name, int in, int out, bool handWheel) :
        inBytes(in), outBytes(out), info_name(name), withHandWheel(handWheel) {}
};

#else
/**
 * @brief SD-LINK从站设备信息结构
 */
struct BdioDevInfo{
	int slave_index;    //从站号，从1开始
	int device_type;      //设备类型号
	int in_bytes;		//输入字节
	int out_bytes;
};
#endif
typedef ListBuffer<BdioDevInfo> BdioDevList;   //SD-LINK从站队列

#ifdef USES_GRIND_MACHINE
//机械臂参数
struct ParamMechArm{
	int row_count;    //行数
	int col_count;    //列数
	double pos_tray_base_x[2];    //料盘基准位置X轴    0-主机   1-从机
	double pos_tray_base_y[2];    //料盘基准位置Y轴   0-主机   1-从机
	double dis_tray_y;     //料盘行间隔
	double dis_tray_x;     //料盘列间隔
	double pos_corr_rough_x;   //粗对位位置X轴
	double pos_corr_rough_y;   //粗对位位置Y轴
	double pos_corr_fine_x;	   //精对位位置X轴
	double pos_corr_fine_y;    //精对位位置Y轴
	double pos_work_x[2];		//加工位置X    0-主机   1-从机
	double pos_work_y[2];       //加工位置Y    0-主机   1-从机
	double dis_mech_arm;    //左右夹爪间距
	double speed;           //机械手移动速度
	int delay_time_vac;       //破真空延时,单位：ms   D408
	int wait_overtime;        //等待超时时间，单位：ms   D416
	int work_vac_check_delay;    //工位吸真空确认延时，单位：ms    D424
	int work_mode;          //工作模式，1--单机模式    0--双机模式    D412
	int corr_pos_count;      //对位次数      D420
};

//机械臂状态
struct StateMechArm{
	uint8_t run_flag;     //运行标志   0--空闲   10--运行   20--暂停
	uint8_t cur_chn;      //当前上下料通道     0-主通道       1-从通道
	bool has_material;    //当前是否持料    true--持料    false--空
	bool repos;           //当前的毛坯是否已经完成对位
	uint8_t cur_step;     //当前步骤
	uint16_t total_count;   //料盘总格数
	uint16_t cur_index[2];     //当前拾取格数
	uint16_t cur_finished_count[2];  //当前料盘成品计数
	bool work_material[2];     //工位是否有料
	bool has_finished_material;  //是否需要放成品

	bool is_exit_inter_area;     //退出干涉区域标志


	pthread_mutex_t mutex_change_run;    //运行状态切换互斥量
	double x_target_pos;    //当前X轴目标位置
	double y_target_pos;    //当前Y轴目标位置
	int corr_index;         //已对位次数
	struct timeval time_start_arm;   //等待起始时间
	struct timeval time_start_arm_2;  //等待起始时间2
	void *msg;      //M指令消息指针
	int err_state;    //机械臂异常状态
};
#endif

#endif /* INC_GLOBAL_STRUCTURE_H_ */
