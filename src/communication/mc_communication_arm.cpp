/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file MCCommunication.h
 *@author gonghao
 *@date 2021/11/22
 *@brief 本头文件为SC-MC-ARM通讯类的实现，适用于SC与运行于ARM上的MC通讯
 *@version
 */

#include "mc_communication_arm.h"
#include "global_include.h"
#include "channel_engine.h"

const uint32_t kSharedRegMapSize = (uint32_t) 1024*1024;   	//映射区大小 1M
const uint32_t kSharedRegMemMapMask = (kSharedRegMapSize - 1);	 //地址掩码 0x0FFFFF



//寄存器读写宏定义函数，提高访问效率
#define WriteSharedRegister(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( m_p_addr_base + ((int32_t)x & kSharedRegMemMapMask));\
	*(virt_addr) = (int32_t)y;}


#define ReadSharedRegister(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( m_p_addr_base + ((int32_t)x & kSharedRegMemMapMask));\
	y = *(virt_addr);}

#define ReadSharedRegister_64(x,y) {volatile int64_t *virt_addr = (volatile int64_t*) ( m_p_addr_base + ((int32_t)x & kSharedRegMemMapMask));\
	y = *(virt_addr);}

#define ReadSharedRegister_16(x,y) {volatile int16_t *virt_addr = (volatile int16_t*) ( m_p_addr_base + ((int32_t)x & kSharedRegMemMapMask));\
	y = *(virt_addr);}

#define WriteSharedRegister_16(x,y) {volatile int16_t *virt_addr = (volatile int16_t*) ( m_p_addr_base + ((int32_t)x & kSharedRegMemMapMask));\
	*(virt_addr) = (int16_t)y;}


//将两个32位整型合并为一个64位整型
#define Merge32_64(l,h) ((int64_t)(((uint64_t)l) | (((uint64_t)h)<<32)))

MCArmCommunication* MCArmCommunication::m_p_instance = nullptr;  //初始化单例对象指正为空
//template<> int ListNode<McCmdFrame>::new_count = 0;


/**
 * @brief 构造函数
 */
MCArmCommunication::MCArmCommunication() {
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
MCArmCommunication::~MCArmCommunication() {
	// TODO Auto-generated destructor stub
	this->Clean();
}

/**
 * @brief 获取类实例的唯一访问接口函数，单例模式
 */
MCArmCommunication* MCArmCommunication::GetInstance(){
	if(nullptr == m_p_instance){
		m_p_instance = new MCArmCommunication();
	}
	return m_p_instance;
}

/**
 * @brief 设置接口指针
 * @param engine : 通道引擎对象指针
 */
void MCArmCommunication::SetInterface(){
	m_p_channel_engine = ChannelEngine::GetInstance();
}

/**
 * @brief 初始化函数，打开地址映射驱动
 * @return true--成功   false--失败
 */
bool MCArmCommunication::Initialize(){
	int res = ERR_NONE;
	pthread_attr_t attr;
	struct sched_param param;

	//互斥量初始化
	pthread_mutex_init(&m_mutex_cmd, nullptr);

	for(int i = 0; i < kMaxChnCount; i++)
		this->m_n_cur_gcode_index[i] = 0;
	this->m_n_cur_cmd_recv_index = 0;
	this->m_n_cur_cmd_send_index = 0;

	//分配命令缓冲
	this->m_list_cmd = new McArmCmdBuffer();
	if(m_list_cmd == nullptr){
		g_ptr_trace->PrintLog(LOG_ALARM, "MCArmCommunication::Initialize() ，初始化分配命令缓冲队列空间失败！");
		m_error_code = ERR_MC_COM_INIT;
		return false;
	}

	//状态读取信号量初始化为0
	m_n_read_status_sem = 0;

	//打开设备文件
	m_n_mem_file = open("/dev/mem", O_RDWR | O_SYNC);
	if (m_n_mem_file == -1) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCArmCommunication::Initialize() failed to open file[/dev/mem]! errno = %d", errno);
		m_error_code = ERR_MC_COM_INIT;
		return false;
	}

	//映射内存地址，得到基地址
	m_p_addr_base = (uint8_t *) mmap(nullptr, kSharedRegMapSize, PROT_READ | PROT_WRITE, MAP_SHARED,
			m_n_mem_file, MC_ARM_REGISTER_BASE & (~kSharedRegMemMapMask));
	if (m_p_addr_base == MAP_FAILED) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCArmCommunication::Initialize() failed to map memory! errno = %d", errno);
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
		g_ptr_trace->PrintLog(LOG_ALARM, "MC通讯接口2命令处理线程设置线程继承模式失败！");
		m_error_code = ERR_MI_COM_INIT;
		goto END;
	}
	res = pthread_create(&m_thread_process_cmd, &attr,
			MCArmCommunication::ProcessCmdThread, this);    //开启MC命令处理线程
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MC通讯接口2命令处理线程启动失败！");
		res = ERR_MI_COM_INIT;
		goto END;
	}

	END:
	pthread_attr_destroy(&attr);

	printf("MCArmCommunication::Initialize(), mem file : %d, base_addr : %p\n", m_n_mem_file, m_p_addr_base);

	return true;
}

/**
 * @brief 清理函数
 * @return true--成功   false--失败
 */
bool MCArmCommunication::Clean(){

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
		g_ptr_trace->PrintLog(LOG_ALARM, "MC通讯接口2命令处理线程退出失败！");
	}

	usleep(1000);
	res = pthread_join(m_thread_process_cmd, &thread_result);//等待命令处理线程退出完成
	if (res != ERR_NONE) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MC通讯接口2等待命令处理线程退出失败！");
	}
	m_thread_process_cmd = 0;

	//取消内存映射
	if(m_p_addr_base != MAP_FAILED)
		munmap(m_p_addr_base, kSharedRegMapSize);

	//关闭设备文件
	if(m_n_mem_file != -1) {
		close(m_n_mem_file);
	}

	pthread_mutex_destroy(&m_mutex_cmd);

	printf("MCArmCommunication::Clean()\n");
	return true;
}

/**
 * @brief 是否能写入运动控制数据
 * @return  true -- 可以    false -- 不可以
 */
bool MCArmCommunication::CanWriteGCode(uint8_t chn){
	//读取当前数据帧区域的读写标志，判断是否能写入
	uint16_t flag = 0;
	ReadSharedRegister_16(MC_ARM_GCODE_WF(chn, m_n_cur_gcode_index[chn]), flag);
	if(flag == 1){  //已写入过数据
		flag = 0;
		ReadSharedRegister_16(MC_ARM_GCODE_RF(chn, m_n_cur_gcode_index[chn]), flag);
		if(flag != 1)   //数据还未被读取
			return false;   //FIFO数据满，无法写入
	}
	return true;
}

/**
 * @brief 写入G代码编译出的运动控制数据
 * @param chn : 通道索引号，0开始
 * @param data ：待写入的数据帧
 * @return true--成功   false--失败,缓冲满
 */
bool MCArmCommunication::WriteGCodeData(uint8_t chn, GCodeFrame &data){
	CalMcGCodeFrameCrc(data);

//	printf("MCArmCommunication::WriteGCodeData1111\n");
	bool wf = false;	//当前数据帧是否已写入过数据
	//读取当前数据帧区域的读写标志，判断是否能写入
	uint16_t flag = 0;
	ReadSharedRegister_16(MC_ARM_GCODE_WF(chn, m_n_cur_gcode_index[chn]), flag);
	if(flag == 1){  //已写入过数据
		flag = 0;
//		printf("MCArmCommunication::WriteGCodeData2222\n");
		ReadSharedRegister_16(MC_ARM_GCODE_RF(chn, m_n_cur_gcode_index[chn]), flag);
		if(flag != 1)   //数据还未被读取
			return false;   //FIFO数据满，无法写入

		wf = true;
	}
//	printf("MCArmCommunication::WriteGCodeData3333\n");

	//发送数据
	for(int i = 0; i < kMaxGCodeDataCount; i++){
		WriteSharedRegister(MC_ARM_GCODE_DATA(chn, m_n_cur_gcode_index[chn], i), data.data_w[i]);
	}

	//置位写入标志
	if(wf){
		WriteSharedRegister_16(MC_ARM_GCODE_RF(chn, m_n_cur_gcode_index[chn]), 0);	//已写入过数据则只需要复位读取标志
	}
	else{
		WriteSharedRegister_16(MC_ARM_GCODE_RF(chn, m_n_cur_gcode_index[chn]), 0);	
		WriteSharedRegister_16(MC_ARM_GCODE_WF(chn, m_n_cur_gcode_index[chn]), 1);//未写入过数据则只需要置位写入标志
	}
//	uint16_t flag1 = 0;
//	ReadSharedRegister_16(MC_ARM_GCODE_WF(chn, m_n_cur_gcode_index[chn]), flag);
//	ReadSharedRegister_16(MC_ARM_GCODE_RF(chn, m_n_cur_gcode_index[chn]), flag1);
//	printf("read flag: %hu, %hu\n", flag, flag1);

	//写入指针下移
	if(++m_n_cur_gcode_index[chn] >= MC_ARM_CHN_GCODE_FIFO_DEPTH)
		m_n_cur_gcode_index[chn] = 0;

	return true;
}


/**
 * @brief 读取通道运行模式及运行指令
 * @param chn_index[in] : 通道索引号
 * @param work_mode[out] : 当前的工作模式
 * @return 无
 */
void MCArmCommunication::ReadWorkMode(const uint8_t chn_index, uint16_t &work_mode){
	uint32_t data;
	ReadSharedRegister(MC_ARM_WORK_MODE(chn_index), data);

//	printf("@@@@@@mc data: 0x%08x\n", data);

	work_mode = data;
}

/**
 * @brief 读取通道当前运行指令
 * @param chn_index[in] : 通道索引号
 * @param cur_cmd[out] : 当前运行的指令
 * @return 无
 */
void MCArmCommunication::ReadCurCmd(const uint8_t chn_index, uint16_t &cur_cmd){

	uint32_t data;
	ReadSharedRegister(MC_ARM_CUR_CMD(chn_index), data);

	cur_cmd = data;
}

/**
 * @brief 读取轴插补位置，包括当前位置（工件坐标系），和目标位置
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param cur_pos[out] : 当前插补位置
 * @param tar_pos[out] ：当前目标位置
 * @return
 */
void MCArmCommunication::ReadAxisIntpPos(const uint8_t chn_index, DPoint &cur_pos, DPoint &tar_pos){

	int64_t pos_64;
	double *pc = &cur_pos.x;
	double *pt = &tar_pos.x;
	for(int i = 0; i < kMaxAxisChn; i++){
		ReadSharedRegister_64(MC_ARM_AXIS_CUR_POS(chn_index, i), pos_64);
		*pc = ((double)pos_64)/1e7;

		ReadSharedRegister_64(MC_ARM_AXIS_TAR_POS(chn_index, i), pos_64);

		*pt = ((double)pos_64)/1e7;

		pc++;
		pt++;
	}

//	printf("read pos ; %lf, %lf, %lf\n", cur_pos.x, cur_pos.y, cur_pos.z);
}


/**
 * @brief 读取通道当前自动数据缓冲数量
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param count[out] : 返回数据条数
 * @return
 */
void MCArmCommunication::ReadAutoBufDataCount(const uint8_t chn_index, uint16_t &count){

	uint32_t tmp_count = 0;


	ReadSharedRegister(MC_ARM_AUTO_BUF_DATA_COUNT(chn_index), tmp_count);

	count = tmp_count;

}

//
/**
 * @brief 读取通道当前MDA数据缓冲数量
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param count[out] : 返回数据条数
 */
void MCArmCommunication::ReadMdaBufDataCount(const uint8_t chn_index, uint16_t &count){

	uint32_t tmp_count = 0;
	ReadSharedRegister(MC_ARM_MDA_BUF_DATA_COUNT(chn_index), tmp_count);
	count = tmp_count;
}

/**
 * @brief 读取通道当前进给速度
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param cur_feed[out] : 返回当前进给速度
 * @return
 */
void MCArmCommunication::ReadCurFeed(const uint8_t chn_index, int32_t &cur_feed){

	ReadSharedRegister(MC_ARM_CUR_FEED(chn_index), cur_feed);

}

/**
 * @brief 读取通道当前给定进给速度
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param cur_feed[out] : 返回当前给定进给速度
 * @return
 */
void MCArmCommunication::ReadRatedFeed(const uint8_t chn_index, int32_t &rated_feed){

	ReadSharedRegister(MC_ARM_RATED_FEED(chn_index), rated_feed);


}

/**
 * @brief 读取通道当前运行行号
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param cur_lineno[out] : 返回当前行号
 * @return
 */
void MCArmCommunication::ReadCurLineNo(const uint8_t chn_index, uint32_t &cur_lineno){

	ReadSharedRegister(MC_ARM_CUR_LINE_NO(chn_index), cur_lineno);


}


/**
 * @brief 读取通道当前坐标系
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param coord[out] : 返回当前坐标系编号
 */
void MCArmCommunication::ReadChnCurCoord(uint8_t chn_index, uint16_t &coord){

	uint32_t data = 0;
	ReadSharedRegister(MC_ARM_CUR_COORD(chn_index), data);
	coord = data;
}


/**
 * @brief 读取通道当前刀偏号
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param tool_offset[out] : 返回当前刀具偏置
 */
void MCArmCommunication::ReadChnCurToolOffset(uint8_t chn_index, uint16_t &tool_offset){

	uint32_t data = 0;
	ReadSharedRegister(MC_ARM_CUR_TOOL_OFFSET(chn_index), data);
	tool_offset = data;

}


/**
 * @brief 读取当前联动插补模式
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道。3--3轴联动  5--5轴联动   6--倾斜面    7--G12.2磨削指令
 * @param intp_mode[out] : 返回当前联动插补模式
 */
void MCArmCommunication::ReadChnCurIntpMode(uint8_t chn_index, uint16_t &intp_mode){

	uint32_t data = 0;
	ReadSharedRegister(MC_ARM_CUR_INTP_MODE(chn_index), data);
	intp_mode = data;

}

/**
 * @brief 读取通道的当前数据帧的序号
 * @param chn_index[in] : 通道索引号, 范围[0,1]，只支持两通道
 * @param frame_index[out] : 返回当前运行的数据帧序号
 */
void MCArmCommunication::ReadChnCurFrameIndex(uint8_t chn_index, uint16_t &frame_index){

	uint32_t data = 0;
	ReadSharedRegister(MC_ARM_CUR_FRAME_INDEX(chn_index), data);
	frame_index = data;

}

/**
 * @brief 读取通道模态信息
 * @param chn_index
 * @param mode_info
 */
void MCArmCommunication::ReadMcModeInfo(const uint8_t chn_index, uint32_t &mode_info){

	ReadSharedRegister(MC_ARM_MODE_INFO(chn_index), mode_info);

}


//
/**
 * @brief 读取当前MC错误标志
 * @param chn_index
 * @param mc_err
 */
void MCArmCommunication::ReadMcErrFlag(uint8_t chn_index, uint32_t &mc_err){


	ReadSharedRegister(MC_ARM_ERR_FLAG(chn_index), mc_err);


}


/**
 * @brief 读取通道的当前正向软限位触发轴mask
 * @param chn_index
 * @param axis_mask
 */
void MCArmCommunication::ReadChnPosSoftLimtMask(uint8_t chn_index, uint32_t &axis_mask){

	ReadSharedRegister(MC_ARM_POS_SOFT_LIMIT_MASK(chn_index), axis_mask);

}

/**
 * @brief 读取通道的当前负向软限位触发轴mask
 * @param chn_index
 * @param axis_mask
 */
void MCArmCommunication::ReadChnNegSoftLimtMask(uint8_t chn_index, uint32_t &axis_mask){

	ReadSharedRegister(MC_ARM_NEG_SOFT_LIMIT_MASK(chn_index), axis_mask);

}

/**
 * @brief 读取通道的当前位置指令过大告警轴mask
 * @param chn_index
 * @param axis_mask
 */
void MCArmCommunication::ReadChnPosErrMask(uint8_t chn_index, uint16_t &axis_mask){

	uint32_t data = 0;
	ReadSharedRegister(MC_ARM_POS_ERR_MASK(chn_index), data);
	axis_mask = data;

}

//
/**
 * @brief 更新指定通道轴到位标志
 * @param chn_index[in] : 通道号
 * @param axis_mask[out] ： 返回轴插补到位mask
 */
void MCArmCommunication::ReadChnAxisRunoverMask(const uint8_t chn_index, uint32_t &axis_mask){


	ReadSharedRegister(MC_ARM_AXIS_INTPOVER_MASK(chn_index), axis_mask);


}

/**
 * @brief 读取分块插补到位标志
 * @param chn_index : 通道号, 范围[0,1]，只支持两通道
 * @return true--运行到位     false--未运行到位
 */
bool MCArmCommunication::ReadAutoBlockRunOverFlag(const uint8_t chn_index){

	uint32_t value;
	bool flag = false;



	ReadSharedRegister(MC_ARM_BLOCK_RUN_OVER(chn_index), value);


//	printf("block over: 0x%x\n", value);

	if(value != 0)
		flag = true;

	return flag;
}

/**
 * @brief 读取MDA模式块到位标志
 * @param chn_index
 * @return true--运行到位     false--未运行到位
 */
bool MCArmCommunication::ReadMdaBlockRunOverFlag(const uint8_t chn_index){

	bool flag = false;
	uint32_t value = 0;



	ReadSharedRegister(MC_ARM_BLOCK_RUN_OVER_MDA(chn_index), value);



	if(value != 0)
		flag = true;

	return flag;
}


/**
 * @brief 读取单段插补到位标志
 * @param chn_index : 通道号, 范围[0,1]，只支持两通道
 * @return true--运行到位     false--未运行到位
 */
bool MCArmCommunication::ReadStepRunOverFlag(const uint8_t chn_index){

	uint32_t value;
	bool flag = false;

	ReadSharedRegister(MC_ARM_STEP_RUN_OVER(chn_index), value);

	if(value != 0)
		flag = true;

	return flag;
}

/**
 * @brief 计算MC命令包的CRC
 * @param cmd : 命令包
 */
void MCArmCommunication::CalMcCmdCrc(McCmdFrame &cmd){
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
void MCArmCommunication::CalMcGCodeFrameCrc(GCodeFrame &data){
	uint16_t crc = 0xFFFF;
	int count = kMaxGCodeDataCount*2;
	for(int i = 1; i < count; i++) //不包括CRC字段本身
		crc ^= data.data_crc[i];

	data.data.crc = crc;
//	printf("GCode CRC : 0x%X\n", data.data.crc);
}


/**
 * @brief 读取命令缓冲数据个数
 * @return
 */
uint32_t MCArmCommunication::ReadCmdBufferLen(){
	return m_list_cmd->GetLength();
}

/**
 * @brief 向下行命令通道写入命令帧数据
 * @param data ：待写入的数据帧
 * @return true--成功   false--失败
 */
bool MCArmCommunication::WriteCmd(McCmdFrame &data, bool resend){

	bool res = true;
	//计算CRC校验码
	CalMcCmdCrc(data);

	pthread_mutex_lock(&m_mutex_cmd);//上锁

	bool wf = false;	//当前数据帧是否已写入过数据
	uint16_t flag = 0;

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

	//读取当前数据帧区域的读写标志，判断是否能写入
	ReadSharedRegister_16(MC_ARM_CMD_DOWN_WF(m_n_cur_cmd_send_index), flag);
	if(flag == 1){  //已写入过数据
		flag = 0;
		ReadSharedRegister_16(MC_ARM_CMD_DOWN_RF(m_n_cur_cmd_send_index), flag);
		if(flag != 1){   //数据还未被读取
			if(resend){
				res = false;	//重发失败
				goto END;
			}

			if(this->m_list_cmd->Append(data)){
				goto END; //放入缓冲命令队列
			}
			else{
				g_ptr_trace->PrintLog(LOG_ALARM, "MC命令缓冲分配内存失败！");
				res = false;   //FIFO数据满，写入缓冲队列失败
				goto END;
			}
		}

		wf = true;
	}


	//发送数据
	WriteSharedRegister(MC_ARM_CMD_DOWN_DATA0(m_n_cur_cmd_send_index), data.data_w[0]);
	WriteSharedRegister(MC_ARM_CMD_DOWN_DATA1(m_n_cur_cmd_send_index), data.data_w[1]);
	WriteSharedRegister(MC_ARM_CMD_DOWN_DATA2(m_n_cur_cmd_send_index), data.data_w[2]);
	WriteSharedRegister(MC_ARM_CMD_DOWN_DATA3(m_n_cur_cmd_send_index), data.data_w[3]);
	WriteSharedRegister(MC_ARM_CMD_DOWN_DATA4(m_n_cur_cmd_send_index), data.data_w[4]);


	//置位写入标志
	if(wf){
		WriteSharedRegister_16(MC_ARM_CMD_DOWN_RF(m_n_cur_cmd_send_index), 0);	//已写入过数据则只需要复位读取标志
	}
	else{
		WriteSharedRegister_16(MC_ARM_CMD_DOWN_WF(m_n_cur_cmd_send_index), 1);	//未写入过数据则只需要置位写入标志
	}

	//写入指针下移
	if(++m_n_cur_cmd_send_index >= MC_ARM_CMD_FIFO_DEPTH)
		m_n_cur_cmd_send_index = 0;


	END:
	pthread_mutex_unlock(&m_mutex_cmd);  //卸锁
	return res;
}

/**
 * @brief 读取上行命令通道的命令帧
 * @param data
 * @return true--成功   false--失败
 */
bool MCArmCommunication::ReadCmdRsp(McCmdFrame &data){
	//读取当前数据帧区域的读写标志，判断是否能读取
	uint16_t flag = 0;
	ReadSharedRegister_16(MC_ARM_CMD_UP_WF(m_n_cur_cmd_recv_index), flag);
	if(flag == 0)
		return false;   //当前帧数据空，无法读取

	flag = 0;
	ReadSharedRegister_16(MC_ARM_CMD_UP_RF(m_n_cur_cmd_recv_index), flag);
	if(flag == 1){
		return false;   //当前数据已读取，不需要重复读
	}


	//读取数据
	ReadSharedRegister(MC_ARM_CMD_UP_DATA0(m_n_cur_cmd_recv_index), data.data_w[0]);
	ReadSharedRegister(MC_ARM_CMD_UP_DATA1(m_n_cur_cmd_recv_index), data.data_w[1]);
	ReadSharedRegister(MC_ARM_CMD_UP_DATA2(m_n_cur_cmd_recv_index), data.data_w[2]);
	ReadSharedRegister(MC_ARM_CMD_UP_DATA3(m_n_cur_cmd_recv_index), data.data_w[3]);
	ReadSharedRegister(MC_ARM_CMD_UP_DATA4(m_n_cur_cmd_recv_index), data.data_w[4]);


	//置位读标志
	WriteSharedRegister_16(MC_ARM_CMD_UP_RF(m_n_cur_cmd_recv_index), 1);

	//读取指针下移
	if(++m_n_cur_cmd_recv_index >= MC_ARM_CMD_FIFO_DEPTH)
		m_n_cur_cmd_recv_index = 0;

	return true;
}

/**
 * @brief  SC-MC命令处理线程
 * @param args
 */
void *MCArmCommunication::ProcessCmdThread(void *args){
	printf("Start MCArmCommunication::ProcessCmdThread!thread id = %ld\n", syscall(SYS_gettid));

	MCArmCommunication *mc_comm = static_cast<MCArmCommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
	if (res != ERR_NONE) {
		printf("Quit MCArmCommunication::ProcessCmdThread!thread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
	if (res != ERR_NONE) {
		printf("Quit MCArmCommunication::ProcessCmdThread!thread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	while((g_sys_state.module_ready_mask & SC_READY) == 0){  //等待SC模块初始化完成
		usleep(10000);
	}

	res = mc_comm->ProcessCmdFun();

	printf("Quit MCArmCommunication::ProcessCmdThread!!\n");
	pthread_exit(NULL);
}

/**
 * @brief 命令处理函数
 * @return
 */
bool MCArmCommunication::ProcessCmdFun(){
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
			g_ptr_trace->PrintLog(LOG_ALARM, "待重发MC-2命令消息过多[%d]！", m_list_cmd->GetLength());


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
			printf("MC-2 COMM CRC ERR: cmd = 0x%hx | %hhu | %hhu | 0x%hx;%hx;%hx;%hx;%hx;%hx;%hx | 0x%hx \n", cmd_frame.data.cmd,
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


/**
 * @brief 读取MC的调试数据
 * @param debug_data[out] : 输出调试数据，16个uint32_t数据
 */
void MCArmCommunication::ReadDebugData(uint32_t *debug_data){
	if(debug_data == nullptr)
		return;

//	for(int i = 0; i < 16; i++)
//		ReadSharedRegister(MC_DEBUG_DATA(i), debug_data[i]);



}


