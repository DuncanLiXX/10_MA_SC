/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file MCCommunication.h
 *@author gonghao
 *@date 2020/06/15
 *@brief 本头文件为SC-MC通讯类的实现
 *@version
 */

#include "mc_communication.h"
#include "global_include.h"
#include "channel_engine.h"

const uint32_t kRegMapSize = (uint32_t) 2*1024;   	//映射区大小 4K
const uint32_t kRegMemMapMask = (kRegMapSize - 1);	 //地址掩码 0x07FF

//寄存器读写宏定义函数，提高访问效率
#define WriteRegister(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( m_p_addr_base + ((int32_t)x & kRegMemMapMask));\
	*(virt_addr) = (int32_t)y;}


#define ReadRegister(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( m_p_addr_base + ((int32_t)x & kRegMemMapMask));\
	y = *(virt_addr);}

#define ReadRegister_16(x,y) {volatile int16_t *virt_addr = (volatile int16_t*) ( m_p_addr_base + ((int32_t)x & kRegMemMapMask));\
	y = *(virt_addr);}


//将两个32位整型合并为一个64位整型
#define Merge32_64(l,h) ((int64_t)(((uint64_t)l) | (((uint64_t)h)<<32)))

MCCommunication* MCCommunication::m_p_instance = nullptr;  //初始化单例对象指正为空
//template<> int ListNode<McCmdFrame>::new_count = 0;


/**
 * @brief 构造函数
 */
MCCommunication::MCCommunication() {
	// TODO Auto-generated constructor stub
	this->m_p_addr_base = nullptr;
	this->m_list_cmd = nullptr;
	this->m_n_mem_file = -1;
	m_thread_process_cmd = 0;
	m_n_crc_err_count = 0;
	this->m_error_code = ERR_NONE;

	this->Initialize();

	if(m_error_code != ERR_NONE)
		CreateError(m_error_code, FATAL_LEVEL, CLEAR_BY_RESET_POWER);  //生成MC通讯模块初始化失败告警

//	printf("sizeof GCodeData: %d\n", sizeof(GCodeData));

}

/**
 * @brief 析构函数
 */
MCCommunication::~MCCommunication() {
	// TODO Auto-generated destructor stub
	this->Clean();
}

/**
 * @brief 获取类实例的唯一访问接口函数，单例模式
 */
MCCommunication* MCCommunication::GetInstance(){
	if(nullptr == m_p_instance){
		m_p_instance = new MCCommunication();
	}
	return m_p_instance;
}

/**
 * @brief 设置接口指针
 * @param engine : 通道引擎对象指针
 */
void MCCommunication::SetInterface(){
	m_p_channel_engine = ChannelEngine::GetInstance();
}

/**
 * @brief 初始化函数，打开地址映射驱动
 * @return true--成功   false--失败
 */
bool MCCommunication::Initialize(){
	int res = ERR_NONE;
	pthread_attr_t attr;
	struct sched_param param;

	//互斥量初始化
	pthread_mutex_init(&m_mutex_cmd, nullptr);

	//分配命令缓冲
	this->m_list_cmd = new McCmdBuffer();
	if(m_list_cmd == nullptr){
		g_ptr_trace->PrintLog(LOG_ALARM, "MCCommunication::Initialize() ，初始化分配命令缓冲队列空间失败！");
		m_error_code = ERR_MC_COM_INIT;
		return false;
	}

	//状态读取信号量初始化为0
	m_n_read_status_sem = 0;

	//打开设备文件
	m_n_mem_file = open("/dev/mem", O_RDWR | O_SYNC);
	if (m_n_mem_file == -1) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCCommunication::Initialize() failed to open file[/dev/mem]! errno = %d", errno);
		m_error_code = ERR_MC_COM_INIT;
		return false;
	}

	//映射内存地址，得到基地址
	m_p_addr_base = (uint8_t *) mmap(nullptr, kRegMapSize, PROT_READ | PROT_WRITE, MAP_SHARED,
			m_n_mem_file, MC_REGISTER_BASE & (~kRegMemMapMask));
	if (m_p_addr_base == MAP_FAILED) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCCommunication::Initialize() failed to map memory! errno = %d", errno);
		m_error_code = ERR_MC_COM_INIT;

		return false;
	}

	//开启线程
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 37; //97;
	pthread_attr_setschedparam(&attr, &param);
	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
	if (res) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MC通讯接口命令处理线程设置线程继承模式失败！");
		m_error_code = ERR_MI_COM_INIT;
		goto END;
	}
	res = pthread_create(&m_thread_process_cmd, &attr,
			MCCommunication::ProcessCmdThread, this);    //开启MC命令处理线程
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MC通讯接口命令处理线程启动失败！");
		res = ERR_MI_COM_INIT;
		goto END;
	}

	END:
	pthread_attr_destroy(&attr);

	printf("MCCommunication::Initialize(), mem file : %d, base_addr : %p\n", m_n_mem_file, m_p_addr_base);

	return true;
}

/**
 * @brief 清理函数
 * @return true--成功   false--失败
 */
bool MCCommunication::Clean(){

	//退出线程
	int res = ERR_NONE;
	void* thread_result;

	//清空待发送命令队列
	if(this->m_list_cmd){
		delete m_list_cmd;
		m_list_cmd = nullptr;
	}

	//退出命令处理线程
	res = pthread_cancel(m_thread_process_cmd);
	if (res != ERR_NONE) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MC通讯接口命令处理线程退出失败！");
	}

	usleep(1000);
	res = pthread_join(m_thread_process_cmd, &thread_result);//等待命令处理线程退出完成
	if (res != ERR_NONE) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MC通讯接口等待命令处理线程退出失败！");
	}
	m_thread_process_cmd = 0;

	//取消内存映射
	if(m_p_addr_base != MAP_FAILED)
		munmap(m_p_addr_base, kRegMapSize);

	//关闭设备文件
	if(m_n_mem_file != -1) {
		close(m_n_mem_file);
	}



	pthread_mutex_destroy(&m_mutex_cmd);

	printf("MCCommunication::Clean()\n");
	return true;
}

/**
 * @brief 是否能写入运动控制数据
 * @return  true -- 可以    false -- 不可以
 */
bool MCCommunication::CanWriteGCode(uint8_t chn){
	//读取FIFO数量，判断能否写入
	uint32_t count = 0;
	ReadRegister(MC_GCODE_FIFO_COUNT(chn), count);
	if(count >= kMaxGCodeFifoCount)
		return false;   //FIFO数据满，无法写入
	return true;
}

/**
 * @brief 写入G代码编译出的运动控制数据
 * @param chn : 通道索引号，0开始
 * @param data ：待写入的数据帧
 * @return true--成功   false--失败,缓冲满
 */
bool MCCommunication::WriteGCodeData(uint8_t chn, GCodeFrame &data){
	CalMcGCodeFrameCrc(data);

	//读取FIFO数量，判断能否写入
	uint32_t count = 0;
	ReadRegister(MC_GCODE_FIFO_COUNT(chn), count);
	if(count >= kMaxGCodeFifoCount)
		return false;   //FIFO数据满，无法写入

	WriteRegister(MC_GCODE_WRITE_OVER(chn), 0);  //写入前置零

	//发送数据
	for(int i = 0; i < kMaxGCodeDataCount; i++){
		WriteRegister(MC_GCODE_DATA(chn,i), data.data_w[i]);
	}

	WriteRegister(MC_GCODE_WRITE_OVER(chn), 1);  //写入后置一

	return true;
}

/**
 * @brief 读取下行运控数据FIFO的数据个数
 * @param chn : 通道索引号，0开始
 * @return fifo当前数据个数
 */
uint32_t MCCommunication::ReadGCodeFifoCount(uint8_t chn){
	//读取下行运控数据FIFO的数据个数
	uint32_t count = 0;
	ReadRegister(MC_GCODE_FIFO_COUNT(chn), count);
	return count;
}

/**
 * @brief 读取通道运行模式及运行指令
 * @param chn_index[in] : 通道索引号
 * @param work_mode[out] : 当前的工作模式
 * @return 无
 */
void MCCommunication::ReadWorkMode(const uint8_t chn_index, uint16_t &work_mode){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

//	uint32_t data;
	ReadRegister_16(MC_WORK_MODE(chn_index), work_mode);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0

//	printf("@@@@@@mc data: 0x%08x\n", data);

//	work_mode = data&0xFFFF;
}

/**
 * @brief 读取通道当前运行指令
 * @param chn_index[in] : 通道索引号
 * @param cur_cmd[out] : 当前运行的指令
 * @return 无
 */
void MCCommunication::ReadCurCmd(const uint8_t chn_index, uint16_t &cur_cmd){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	uint32_t data;
	ReadRegister(MC_WORK_MODE(chn_index), data);

	cur_cmd = (data&0xFFFF0000)>>16;

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}

/**
 * @brief 读取轴插补位置，包括当前位置（工件坐标系），和目标位置
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param cur_pos[out] : 当前插补位置
 * @param tar_pos[out] ：当前目标位置
 * @return
 */
void MCCommunication::ReadAxisIntpPos(const uint8_t chn_index, DPoint &cur_pos, DPoint &tar_pos){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	uint32_t pos_l, pos_h;
	int64_t pos_64;
	double *pc = &cur_pos.x;
	double *pt = &tar_pos.x;
	for(int i = 0; i < kMaxAxisChn; i++){
		ReadRegister(MC_AXIS_CUR_POS_L(chn_index, i), pos_l);
		ReadRegister(MC_AXIS_CUR_POS_H(chn_index, i), pos_h);
		pos_64 = Merge32_64(pos_l, pos_h);
		*pc = ((double)pos_64)/1e7;

		ReadRegister(MC_AXIS_TAR_POS_L(chn_index, i), pos_l);
		ReadRegister(MC_AXIS_TAR_POS_H(chn_index, i), pos_h);
		pos_64 = Merge32_64(pos_l, pos_h);
		*pt = ((double)pos_64)/1e7;

		pc++;
		pt++;
	}

//	printf("read pos ; %lf, %lf, %lf\n", cur_pos.x, cur_pos.y, cur_pos.z);


	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}


/**
 * @brief 读取通道当前自动数据缓冲数量
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param count[out] : 返回数据条数
 * @return
 */
void MCCommunication::ReadAutoBufDataCount(const uint8_t chn_index, uint16_t &count){

//	uint32_t tmp_count = 0;
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister_16(MC_AUTO_BUF_DATA_COUNT(chn_index), count);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0

//	if(chn_index % 2){//奇数通道
//		count = (uint16_t)((tmp_count >> 16)&0x0000FFFF);
//	}
//	else{
//		count = (uint16_t)(tmp_count & 0x0000FFFF);
//	}
}

//
/**
 * @brief 读取通道当前MDA数据缓冲数量
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param count[out] : 返回数据条数
 */
void MCCommunication::ReadMdaBufDataCount(const uint8_t chn_index, uint16_t &count){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister_16(MC_MDA_BUF_DATA_COUNT(chn_index), count);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}

/**
 * @brief 读取通道当前进给速度
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param cur_feed[out] : 返回当前进给速度
 * @return
 */
void MCCommunication::ReadCurFeed(const uint8_t chn_index, uint32_t &cur_feed){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister(MC_CUR_FEED(chn_index), cur_feed);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}

/**
 * @brief 读取通道当前给定进给速度
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param cur_feed[out] : 返回当前给定进给速度
 * @return
 */
void MCCommunication::ReadRatedFeed(const uint8_t chn_index, uint32_t &rated_feed){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister(MC_RATED_FEED(chn_index), rated_feed);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}

/**
 * @brief 读取通道当前运行行号
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param cur_lineno[out] : 返回当前行号
 * @return
 */
void MCCommunication::ReadCurLineNo(const uint8_t chn_index, uint32_t &cur_lineno){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister(MC_CUR_LINE_NO(chn_index), cur_lineno);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}


/**
 * @brief 读取通道当前刀具累计寿命
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param tool_life[out] : 返回当前刀具寿命
 */
void MCCommunication::ReadChnCurToolLife(uint8_t chn_index, int32_t &tool_life){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister(MC_CHN_CUR_TOOL_LIFE(chn_index), tool_life);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}


/**
 * @brief 读取通道当前坐标系
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param coord[out] : 返回当前坐标系编号
 */
void MCCommunication::ReadChnCurCoord(uint8_t chn_index, uint16_t &coord){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister_16(MC_CUR_COORD(chn_index), coord);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}


/**
 * @brief 读取通道当前刀偏号
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param tool_offset[out] : 返回当前刀具偏置
 */
void MCCommunication::ReadChnCurToolOffset(uint8_t chn_index, uint16_t &tool_offset){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister_16(MC_CUR_TOOL_OFFSET(chn_index), tool_offset);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}


/**
 * @brief 读取当前联动插补模式
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道。3--3轴联动  5--5轴联动   6--倾斜面    7--G12.2磨削指令
 * @param intp_mode[out] : 返回当前联动插补模式
 */
void MCCommunication::ReadChnCurIntpMode(uint8_t chn_index, uint16_t &intp_mode){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister_16(MC_CUR_INTP_MODE(chn_index), intp_mode);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}

/**
 * @brief 读取通道的当前数据帧的序号
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param frame_index[out] : 返回当前运行的数据帧序号
 */
void MCCommunication::ReadChnCurFrameIndex(uint8_t chn_index, uint16_t &frame_index){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister_16(MC_CUR_FRAME_INDEX(chn_index), frame_index);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}

/**
 * @brief 读取通道模态信息
 * @param chn_index
 * @param mode_info
 */
void MCCommunication::ReadMcModeInfo(const uint8_t chn_index, uint32_t &mode_info){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister(MC_MODE_INFO(chn_index), mode_info);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}


//
/**
 * @brief 读取当前MC错误标志
 * @param chn_index
 * @param mc_err
 */
void MCCommunication::ReadMcErrFlag(uint8_t chn_index, uint32_t &mc_err){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister(MC_ERR_FLAG(chn_index), mc_err);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}

//
/**
 * @brief 读取通道的当前软限位触发轴mask
 * @param chn_index
 * @param axis_mask
 */
void MCCommunication::ReadChnSoftLimtMask(uint8_t chn_index, uint32_t &axis_mask){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister(MC_SOFT_LIMIT_MASK(chn_index), axis_mask);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}

//
/**
 * @brief 读取通道的当前位置指令过大告警轴mask
 * @param chn_index
 * @param axis_mask
 */
void MCCommunication::ReadChnPosErrMask(uint8_t chn_index, uint16_t &axis_mask){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister_16(MC_POS_ERR_MASK(chn_index), axis_mask);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}

//
/**
 * @brief 更新指定通道轴到位标志
 * @param chn_index[in] : 通道号
 * @param axis_mask[out] ： 返回轴插补到位mask
 */
void MCCommunication::ReadChnAxisRunoverMask(const uint8_t chn_index, uint32_t &axis_mask){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister(MC_AXIS_INTPOVER_MASK(chn_index), axis_mask);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
}

/**
 * @brief 读取分块插补到位标志
 * @param chn_index : 通道号, 范围[0,1]，只支持两通道
 * @return true--运行到位     false--未运行到位
 */
bool MCCommunication::ReadAutoBlockRunOverFlag(const uint8_t chn_index){

	uint32_t value;
	bool flag = false;

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister(MC_RUN_OVER_FLAG, value);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0
//	printf("block over: 0x%x\n", value);

	if((value & 0xFFFF0000) & (0x00010000<<chn_index))
		flag = true;

	return flag;
}

/**
 * @brief 读取MDA模式块到位标志
 * @param chn_index
 * @return true--运行到位     false--未运行到位
 */
bool MCCommunication::ReadMdaBlockRunOverFlag(const uint8_t chn_index){

	bool flag = false;
	uint32_t value = 0;

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister(MC_BLOCK_RUN_OVER_MDA, value);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0

	if((value & 0x0000FFFF) & (0x0001<<chn_index))
		flag = true;

	return flag;
}


/**
 * @brief 读取到位标志字段
 * @param over_flag
 */
uint32_t MCCommunication::ReadRunOverValue(){
	uint32_t over_flag;
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister(MC_RUN_OVER_FLAG, over_flag);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0

	return over_flag;
}

/**
 * @brief 读取单段插补到位标志
 * @param chn_index : 通道号, 范围[0,1]，只支持两通道
 * @return true--运行到位     false--未运行到位
 */
bool MCCommunication::ReadStepRunOverFlag(const uint8_t chn_index){

	uint32_t value;
	bool flag = false;

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	ReadRegister(MC_RUN_OVER_FLAG, value);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0


	if((value & 0x0000FFFF) & (0x0001<<chn_index))
		flag = true;

	return flag;
}

/**
 * @brief 计算MC命令包的CRC
 * @param cmd : 命令包
 */
void MCCommunication::CalMcCmdCrc(McCmdFrame &cmd){
	uint16_t crc = 0xffff;
	int count = kMaxCmdCount*2-1;	//不包括CRC字段本身
	for(int i = 0; i < count; i++)
		crc ^= cmd.data_crc[i];

	cmd.data.crc = crc;
}

/**
 * @brief 计算G代码数据帧的CRC
 * @param data
 */
void MCCommunication::CalMcGCodeFrameCrc(GCodeFrame &data){
	uint16_t crc = 0xFFFF;
	int count = kMaxGCodeDataCount*2;
	for(int i = 1; i < count; i++) //不包括CRC字段本身
		crc ^= data.data_crc[i];

	data.data.crc = crc;
//	printf("GCode CRC : 0x%X\n", data.data.crc);
}

/**
 * @brief 读取命令通道FIFO中当前数据个数
 * @return 命令通道FIFO中的数据个数
 */
uint32_t MCCommunication::ReadCmdFifoCount(){
	uint32_t count = 0;

	//读取FIFO数量，判断能否写入
	ReadRegister(MC_CMD_DOWN_FIFO_COUNT, count);

	return count;
}

/**
 * @brief 读取命令缓冲数据个数
 * @return
 */
uint32_t MCCommunication::ReadCmdBufferLen(){
	return m_list_cmd->GetLength();
}

/**
 * @brief 向下行命令通道写入命令帧数据
 * @param data ：待写入的数据帧
 * @return true--成功   false--失败
 */
bool MCCommunication::WriteCmd(McCmdFrame &data, bool resend){

	bool res = true;
	//计算CRC校验码
	CalMcCmdCrc(data);

	pthread_mutex_lock(&m_mutex_cmd);//上锁

	uint32_t count = 0;

	if(!resend && m_list_cmd->GetLength() > 0){//非重发并且重发队列非空，则直接进入重发队列，防止命令乱序
		if(this->m_list_cmd->Append(data)){
			goto END; //放入缓冲命令队列
		}
		else{
			g_ptr_trace->PrintLog(LOG_ALARM, "MC命令缓冲分配内存失败！");
			res = false;   //FIFO数据满，写入缓冲队列失败
			goto END;
		}
	}

	//读取FIFO数量，判断能否写入
	ReadRegister(MC_CMD_DOWN_FIFO_COUNT, count);
	while(count >= kMaxCmdFifoCount){
		if(resend){
			res = false;	//重发失败
			goto END;
		}

		if(this->m_list_cmd->Append(data)){
			g_ptr_trace->PrintTrace(TRACE_WARNING, MC_COMMUNICATION_SC, "向MC发送命令[%hu]，进入重发队列！", data.data.cmd);																													
			goto END; //放入缓冲命令队列
		}
		else{
			g_ptr_trace->PrintLog(LOG_ALARM, "MC命令缓冲分配内存失败！");
			res = false;   //FIFO数据满，写入缓冲队列失败
			goto END;
		}
	}

	WriteRegister(MC_CMD_DOWN_WRITE_OVER, 0);  //写入前置零

	//发送数据
	WriteRegister(MC_CMD_DOWN_DATA0, data.data_w[0]);
	WriteRegister(MC_CMD_DOWN_DATA1, data.data_w[1]);
	WriteRegister(MC_CMD_DOWN_DATA2, data.data_w[2]);
	WriteRegister(MC_CMD_DOWN_DATA3, data.data_w[3]);
	WriteRegister(MC_CMD_DOWN_DATA4, data.data_w[4]);


	WriteRegister(MC_CMD_DOWN_WRITE_OVER, 1);  //写入后置一

	END:
	pthread_mutex_unlock(&m_mutex_cmd);  //卸锁
	return res;
}

/**
 * @brief 读取上行命令通道的命令帧
 * @param data
 * @return true--成功   false--失败
 */
bool MCCommunication::ReadCmdRsp(McCmdFrame &data){
	//读取FIFO数量，判断能否读取
	uint32_t count = 0;
	ReadRegister(MC_CMD_UP_FIFO_COUNT, count);
	if(count == 0)
		return false;   //FIFO数据空，无法读取

	WriteRegister(MC_CMD_UP_READ_REQ, 1);  //读取前置1

	//读取数据
	ReadRegister(MC_CMD_UP_DATA0, data.data_w[0]);
	ReadRegister(MC_CMD_UP_DATA1, data.data_w[1]);
	ReadRegister(MC_CMD_UP_DATA2, data.data_w[2]);
	ReadRegister(MC_CMD_UP_DATA3, data.data_w[3]);
	ReadRegister(MC_CMD_UP_DATA4, data.data_w[4]);

	WriteRegister(MC_CMD_UP_READ_REQ, 0);  //读取后置0


	return true;
}

/**
 * @brief  SC-MC命令处理线程
 * @param args
 */
void *MCCommunication::ProcessCmdThread(void *args){
	printf("Start MCCommunication::ProcessCmdThread!thread id = %ld\n", syscall(SYS_gettid));

	MCCommunication *mc_comm = static_cast<MCCommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
	if (res != ERR_NONE) {
		printf("Quit MCCommunication::ProcessCmdThread!thread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
	if (res != ERR_NONE) {
		printf("Quit MCCommunication::ProcessCmdThread!thread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	while((g_sys_state.module_ready_mask & SC_READY) == 0){  //等待SC模块初始化完成
		usleep(10000);
	}

	res = mc_comm->ProcessCmdFun();

	printf("Quit MCCommunication::ProcessCmdThread!!\n");
	pthread_exit(NULL);
}

/**
 * @brief 命令处理函数
 * @return
 */
bool MCCommunication::ProcessCmdFun(){
	McCmdFrame cmd_frame;
//	McCmdFrame *send_cmd = nullptr;
	ListNode<McCmdFrame> *node = nullptr;


	uint64_t read_count = 0;	//for test
	while(!g_sys_state.system_quit){//程序不退出

		//处理待发送的命令
		while(!this->m_list_cmd->IsEmpty()){
			node = this->m_list_cmd->HeadNode();
			if(this->WriteCmd(static_cast<McCmdFrame &>(node->data), true)){
//				g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "重发MC命令消息成功！");
				this->m_list_cmd->Delete(node);
				node = nullptr;
			}else{
//				g_ptr_trace->PrintLog(LOG_ALARM, "重发MC命令消息失败！");
				break; //还是发送失败，跳出循环
			}
		}

		if(this->m_list_cmd->GetLength() > 100)
			g_ptr_trace->PrintLog(LOG_ALARM, "待重发MC命令消息过多[%d]！", m_list_cmd->GetLength());


		if(!this->ReadCmdRsp(cmd_frame)){
			//无数据则睡眠2ms
			usleep(2000);
//			this->m_p_channel_engine->ShakeHandWithMc();  //循环发送握手命令进行测试
			continue;
		}

		read_count++;	// 读取响应计数

		//处理数据包
		//TODO 校验CRC
		if(!CheckCrc(cmd_frame.data_crc, kMaxCmdCount*2-1, cmd_frame.data.crc)){
			//CRC校验失败
	//		CreateError(ERR_MC_CMD_CRC, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON);  //生成MC通讯CRC校验错告警
			m_n_crc_err_count++;
			printf("MC COMM CRC ERR: cmd = 0x%hx | %hhu | %hhu | 0x%hx;%hx;%hx;%hx;%hx;%hx;%hx | 0x%hx \n", cmd_frame.data.cmd,
					cmd_frame.data.axis_index, cmd_frame.data.channel_index, cmd_frame.data.data[0], cmd_frame.data.data[1], cmd_frame.data.data[2],
					cmd_frame.data.data[3], cmd_frame.data.data[4], cmd_frame.data.data[5], cmd_frame.data.data[6], cmd_frame.data.crc);

			printf("err info, recv:%lld, err:%lld percent:%lf\n", read_count, m_n_crc_err_count, (double)m_n_crc_err_count/read_count);
			if(cmd_frame.data.cmd != CMD_MC_SHAKEHANDS)//为了测试放行握手命令
				continue;
		}


		//转给ChannelEngine执行
		this->m_p_channel_engine->ProcessMcCmdRsp(cmd_frame);
	}


	return true;
}

//读取Zynq-pl的版本
bool MCCommunication::ReadPlVer(){
	uint32_t ver = 0;
	ReadRegister(PL_VERSION, ver);

	uint8_t vv[8];
	uint32_t mask = 0x0f;
	for(int i = 0; i < 8 ; i++){
		vv[7-i] = ((ver>>(4*i)) & mask);
	}

	sprintf(g_sys_info.sw_version_info.fpga_pl, "%hhu%hhu%hhu%hhu%hhu%hhu%hhu%hhu", vv[0], vv[1], vv[2], vv[3], vv[4], vv[5], vv[6], vv[7]);

	return ver?true:false;
}

//读取sp6的版本
bool MCCommunication::ReadSp6Ver(){
	uint32_t ver = 0;
	ReadRegister(SP6_VERSION, ver);

	uint8_t vv[8];
	uint32_t mask = 0x0f;
	for(int i = 0; i < 8 ; i++){
		vv[7-i] = ((ver>>(4*i)) & mask);
	}

	sprintf(g_sys_info.sw_version_info.fpga_spartan, "%hhu%hhu%hhu%hhu%hhu%hhu%hhu%hhu", vv[0], vv[1], vv[2], vv[3], vv[4], vv[5], vv[6], vv[7]);

	return ver?true:false;
}

/**
 * @brief 读取欠压告警信号
 * @return true--欠压    false--正常
 */
bool MCCommunication::ReadUnderVoltWarn(){
	uint32_t data = 0;
	ReadRegister(UNDER_VOLTAGE_ALARM, data);

	return (data&0x01)?true:false;
}


/**
 * @brief 读取MC的调试数据
 * @param debug_data[out] : 输出调试数据，16个uint32_t数据
 */
void MCCommunication::ReadDebugData(uint32_t *debug_data){
	if(debug_data == nullptr)
		return;

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //读取前置1

	for(int i = 0; i < 16; i++)
		ReadRegister(MC_DEBUG_DATA(i), debug_data[i]);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //读取后置0

}


