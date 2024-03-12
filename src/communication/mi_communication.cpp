/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file MICommunication.h
 *@author gonghao
 *@date 2020/06/15
 *@brief 本头文件为SC-MI通讯类的实现
 *@version
 */

#include "global_include.h"
#include "mi_communication.h"
#include "channel_engine.h"
#include "pmc_register.h"
#include "showsc.h"

const uint32_t kSharedMemMapSize = (uint32_t) 1024*1024;   	//映射区大小 1M
const uint32_t kSharedMemMapMask = (kSharedMemMapSize - 1);	 	//地址掩码 0x0FFFFF


//寄存器读写宏定义函数，提高访问效率
#define WriteRegister64_M(x,y) {volatile int64_t *virt_addr = (volatile int64_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	*(virt_addr) = (int64_t)y;}

#define WriteRegister32_M(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	*(virt_addr) = (int32_t)y;}

#define WriteRegister16_M(x,y) {volatile int16_t *virt_addr = (volatile int16_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	*(virt_addr) = (int16_t)y;}

#define ReadRegister64U_M(x,y) {volatile uint64_t *virt_addr = (volatile uint64_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
    y = *(virt_addr);}
#define ReadRegister64_M(x,y) {volatile int64_t *virt_addr = (volatile int64_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	y = *(virt_addr);}

#define ReadRegister32_M(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	y = *(virt_addr);}

#define ReadRegister32U_M(x,y) {volatile uint32_t *virt_addr = (volatile uint32_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	y = *(virt_addr);}

#define ReadRegister16_M(x,y) {volatile int16_t *virt_addr = (volatile int16_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	y = *(virt_addr);}

#define ReadRegister16U_M(x,y) {volatile uint16_t *virt_addr = (volatile uint16_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
    y = *(virt_addr);}

#define ReadRegister8_M(x,y) {volatile int8_t *virt_addr = (volatile int8_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	y = *(virt_addr);}


MICommunication* MICommunication::m_p_instance = nullptr;  //初始化单例对象指正为空
//template<> int ListNode<MiCmdFrame>::new_count = 0;

/**
 * @brief 构造函数
 */
MICommunication::MICommunication() {
	// TODO Auto-generated constructor stub
	m_p_shared_base = (uint8_t *)MAP_FAILED;
	m_n_mem_file = 0;
	this->m_n_cur_recv_cmd_index = 0;
	this->m_n_cur_send_cmd_index = 0;
	m_error_code = ERR_NONE;

	this->m_n_cur_send_pmc_index = 0;

	if(g_ptr_parm_manager != nullptr)
		this->m_n_threshold = g_ptr_parm_manager->GetSystemConfig()->chn_count * 200;
	else
		m_n_threshold = 200;


	//初始化命令缓冲队列
	this->m_list_cmd = new MiCmdBuffer();
	if(m_list_cmd == nullptr){
		m_error_code = ERR_MI_COM_INIT;
		goto END;
	}

	//初始化PMC指令缓冲队列
	this->m_list_pmc = new PmcCmdBuffer();
	if(m_list_pmc == nullptr){
		m_error_code = ERR_MI_COM_INIT;
		goto END;
	}

	//互斥量初始化
	pthread_mutex_init(&m_mutex_cmd_down, nullptr);
	pthread_mutex_init(&m_mutex_pmc_cmd, nullptr);

	if(!this->InitSharedMemory()){
		m_error_code = ERR_MI_COM_INIT;
		goto END;
	}

	if(ERR_NONE != this->InitThread()){
		goto END;
	}

	END:
	if(m_error_code != ERR_NONE){
		CreateError(m_error_code, FATAL_LEVEL, CLEAR_BY_RESET_POWER);
	}
}

/**
 * @brief 析构函数
 */
MICommunication::~MICommunication() {
	// TODO Auto-generated destructor stub
	this->QuitThread();
	this->CloseSharedMemory();

	//释放命令缓冲队列
	if(this->m_list_cmd)
		delete m_list_cmd;

	//释放PMC轴运动命令缓冲队列
	if(this->m_list_pmc)
		delete m_list_pmc;

	pthread_mutex_destroy(&m_mutex_cmd_down);	//销毁命令写入互斥量
	pthread_mutex_destroy(&m_mutex_pmc_cmd);	//销毁PMC指令写入互斥量
}

/**
 * @brief 获取类实例的唯一访问接口函数，单例模式
 */
MICommunication* MICommunication::GetInstance(){
	if(NULL == m_p_instance){
		m_p_instance = new MICommunication();
	}
	return m_p_instance;
}

/**
 * @brief 设置接口指针
 * @param engine : 通道引擎对象指针
 */
void MICommunication::SetInterface(){
	m_p_channel_engine = ChannelEngine::GetInstance();
}

/**
 * @brief 初始化线程
 * @return
 */
int MICommunication::InitThread(){
	int res = ERR_NONE;
	pthread_attr_t attr;
	struct sched_param param;

	//开启线程
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
    param.__sched_priority = 39;//37; //97;
	pthread_attr_setschedparam(&attr, &param);
	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
	if (res) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MI通讯接口命令处理线程设置线程继承模式失败！");
		m_error_code = ERR_MI_COM_INIT;
		goto END;
	}
	res = pthread_create(&m_thread_process_cmd, &attr,
			MICommunication::ProcessCmdThread, this);    //开启MI命令处理线程
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MI通讯接口命令处理线程启动失败！");
		res = ERR_MI_COM_INIT;
		goto END;
	}

	END:
	pthread_attr_destroy(&attr);
	return res;
}

/**
 * @brief 退出线程
 * @return
 */
int MICommunication::QuitThread(){
	int res = ERR_NONE;
	void* thread_result;

	//退出命令处理线程
	res = pthread_cancel(m_thread_process_cmd);
	if (res != ERR_NONE) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MI通讯接口命令处理线程退出失败！");
	}

	usleep(1000);
	res = pthread_join(m_thread_process_cmd, &thread_result);//等待命令处理线程退出完成
	if (res != ERR_NONE) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MI通讯接口等待命令处理线程退出失败！");
	}
	m_thread_process_cmd = 0;


	return res;
}

/**
 * @brief 初始化共享内存
 * @return
 */
bool MICommunication::InitSharedMemory(){
	//打开设备文件
	m_n_mem_file = open("/dev/mem", O_RDWR | O_SYNC);

    if (m_n_mem_file == -1) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MICommunication::InitSharedMemory() failed to open file! errno = %d", errno);
		return false;
	}

	//映射内存地址，得到基地址
	m_p_shared_base = (uint8_t *) mmap(nullptr, kSharedMemMapSize, PROT_READ | PROT_WRITE, MAP_SHARED,
			m_n_mem_file, SHARED_MEM_BASE & (~kSharedMemMapMask));
	if (m_p_shared_base == MAP_FAILED) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MICommunication::InitSharedMemory() failed to map memory! errno = %d", errno);
		return false;
	}
	//初始化上下行命令通道
	InitCmdChannel();

    WriteRegister32_M(SHARED_MEM_AXIS_READ_OVER, 0);  //初始化读取完成标志为0
    WriteRegister32_M(SHARED_MEM_TAP_READ_OVER, 0);  //初始化读取完成标志为0

	printf("mem file : %d, base_addr : %p\n", m_n_mem_file, m_p_shared_base);
	return true;
}

/**
 * @brief 获取PMC梯形图数据区的地址
 * @return
 */
char *MICommunication::GetPmcDataAddr(){
	char *ptr = (char *) ( m_p_shared_base + ((int32_t)SHARED_MEM_PMC_LADDER_BASE & kSharedMemMapMask));
	return ptr;
}



void MICommunication::SendOperateCmd(uint16_t opt, uint8_t axis, uint16_t enable)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_OPERATE;
    cmd.data.axis_index = axis;
    cmd.data.data[0] = opt;
    cmd.data.data[1] = enable;
    WriteCmd(cmd);
}

void MICommunication::SendAxisEnableCmd(uint8_t axis, bool enable, uint8_t pos_req)
{
    printf("SendAxisEnable:axis = %d, enable = %d\n",axis,enable);

    // opt为1，代表是轴使能操作
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_OPERATE;
    cmd.data.axis_index = axis;
    cmd.data.data[0] = 1;
    cmd.data.data[1] = enable;
    cmd.data.data[2] = pos_req;
    WriteCmd(cmd);
}

void MICommunication::SendAxisMLK(uint8_t axis, bool MLK)
{
    printf("SendAxisMLK:axis = %d, enable = %d\n",axis,true);

    // opt为2，代表是机械锁住使能操作
    SendOperateCmd(2,axis,MLK);
}

void MICommunication::SendTapAxisCmd(uint8_t chn, uint8_t spd_axis, uint8_t z_axis)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_TAP_AXIS;
    cmd.data.axis_index = 0xFF;
    cmd.data.reserved = chn;
    // 协议中的物理轴号从1开始
    cmd.data.data[0] = spd_axis;
    cmd.data.data[1] = z_axis;
    WriteCmd(cmd);
}

void MICommunication::SendTapRatioCmd(uint8_t chn, int32_t ratio)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_TAP_RATIO;
    cmd.data.axis_index = 0xFF;
    cmd.data.reserved = chn;
    memcpy(&cmd.data.data[0], &ratio, sizeof(int32_t));
    WriteCmd(cmd);
}

void MICommunication::SendTapParams(uint8_t chn, uint32_t error_gain, uint32_t feed_gain, uint32_t ratio_gain)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_TAP_PARAM;
    cmd.data.axis_index = 0xFF;
    cmd.data.reserved = chn;

    uint32_t param = error_gain;
    memcpy(&cmd.data.data[0], &param, sizeof(uint32_t));

    param = feed_gain;
    memcpy(&cmd.data.data[2], &param, sizeof(uint32_t));

    param = ratio_gain;
    memcpy(&cmd.data.data[4], &param, sizeof(uint32_t));

    WriteCmd(cmd);
}

void MICommunication::SendTapStateCmd(uint8_t chn, bool enable, bool rtnt)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_TAP_STATE;
    cmd.data.axis_index = 0xFF;
    cmd.data.reserved = chn;
    cmd.data.data[0] = enable?1:0;
    cmd.data.data[1] = rtnt;      //是否刚性攻丝回退中启动
    WriteCmd(cmd);
}

void MICommunication::SendSpdLocateCmd(uint8_t chn, uint8_t axis,bool enable)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SPD_LOCATE;
    cmd.data.axis_index = axis;
    cmd.data.reserved = chn;
    cmd.data.data[0] = enable;
    WriteCmd(cmd);
}

void MICommunication::SendAxisCtrlModeSwitchCmd(uint8_t axis, uint8_t type)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_CTRL_MODE;
    cmd.data.axis_index = axis;
    cmd.data.data[0] = type;

    WriteCmd(cmd);
}

void MICommunication::SendMpgStep(uint8_t chn, bool enable, uint16_t step)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_MPG_STEP;
    cmd.data.axis_index = 0xff;
    cmd.data.reserved = chn;
    cmd.data.data[0] = enable;
    cmd.data.data[1] = step;

    WriteCmd(cmd);
}

void MICommunication::SendSyncAxis(int64_t mask)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_DO_SYNC_AXIS;

    cmd.data.axis_index = 0xFF;

    memcpy(cmd.data.data, &mask, sizeof(int64_t));
    printf("SendSyncAxis:mask = %lld\n",mask);

    WriteCmd(cmd);
}

void MICommunication::SendEnSyncAxis(uint8_t master, uint8_t slave, bool enable)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_EN_SYNC_AXIS;
    cmd.data.axis_index = slave;
    cmd.data.data[0] = master;
    cmd.data.data[1] = enable;
    WriteCmd(cmd);
}

void MICommunication::SendHandwheelState(uint8_t chn,
                                         bool trace_enable,
                                         bool trace_reverse,
                                         bool insert_enable)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_HANDWHEEL_TRACE;

    cmd.data.data[0] = trace_enable?0x10:0x00;
    cmd.data.data[1] = trace_reverse;   //手轮反向引导使能
    cmd.data.data[2] = insert_enable?0x10:0x00;

    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = chn;
    WriteCmd(cmd);
}

void MICommunication::SendHandwheelInsert(uint8_t chn, bool enable)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_HANDWHEEL_TRACE;

    cmd.data.data[0] = 0x00;
    cmd.data.data[1] = 0;
    cmd.data.data[2] = enable?0x10:0x00;

    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = chn;
    WriteCmd(cmd);
}

void MICommunication::SendHandwheelInsertAxis(uint8_t chn, uint8_t axis)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_MPG_INSERT_AXIS;

    cmd.data.data[0] = 0x01;
    cmd.data.data[1] = axis;

    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = chn;
    WriteCmd(cmd);
}

void MICommunication::SendHardLimitState(bool is_positive,uint64_t mask)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_HARD_LIMIT;

    uint16_t *p_mask = (uint16_t*)&mask;
    cmd.data.data[0] = is_positive;
    cmd.data.data[1] = *p_mask;
    cmd.data.data[2] = *(p_mask+1);
    cmd.data.data[3] = *(p_mask+2);
    cmd.data.data[4] = *(p_mask+3);

    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = 0xFF;
    WriteCmd(cmd);
}

void MICommunication::ReadG31_1_MeasureData(void *data, int count)
{

    uint8_t * p_addr = (uint8_t *)SHARED_MEM_PMC_LADDER_BASE;

    memcpy(data, p_addr, sizeof(int64_t)*count);
}

/**
 * @brief 初始化上下行命令通道，将所有命令帧的读写标志位置0
 */
void MICommunication::InitCmdChannel(){
	for(int i = 0; i < MI_CMD_FIFO_DEPTH; i++){
		WriteRegister32_M(SHARED_MEM_CMD_SEND_WF(i), 0);
	//	WriteRegister32_M(SHARED_MEM_CMD_RECV_WF(i), 0);
	}
}

/**
 * @brief 关闭共享内存
 * @return
 */
bool MICommunication::CloseSharedMemory(){

	//取消内存映射
	if(m_p_shared_base != MAP_FAILED)
		munmap(m_p_shared_base, kSharedMemMapSize);

	//关闭设备文件
	if(m_n_mem_file != -1) {
		close(m_n_mem_file);
	}

	printf("MICommunication::CloseSharedMemory()\n");
	return true;
}

/**
 * @brief 读取寄存器
 * @param addr : 寄存器地址偏移
 * @param value ：返回读出的数据
 * @return
 */
bool MICommunication::ReadRegister64(const uint32_t addr, int64_t& value){
	//计算寄存器虚拟地址
	volatile int64_t *virt_addr = (volatile int64_t*) (m_p_shared_base + ( addr & kSharedMemMapMask));

	//读寄存器值
	value = *(virt_addr);
	return true;
}

/**
 * @brief 读取寄存器
 * @param addr : 寄存器地址偏移
 * @param value ：返回读出的数据
 * @return
 */
bool MICommunication::ReadRegister32(const uint32_t addr, int32_t& value){
	//计算寄存器虚拟地址
	volatile int32_t *virt_addr = (volatile int32_t*) (m_p_shared_base + ( addr & kSharedMemMapMask));

	//读寄存器值
	value = *(virt_addr);
	return true;
}

/**
 * @brief 写入寄存器
 * @param addr : 寄存器地址偏移
 * @param value ：待写入的数据
 * @return
 */
bool MICommunication::WriteRegister32(const uint32_t addr, int32_t value){
	//计算寄存器虚拟地址
	volatile int32_t *virt_addr = (volatile int32_t*) ( m_p_shared_base + ( addr & kSharedMemMapMask));

	//写寄存器
	*(virt_addr) = value;

	return true;
}

/**
 * @brief 写入寄存器
 * @param addr : 寄存器地址偏移
 * @param value ：待写入的数据
 * @return
 */
bool MICommunication::WriteRegister64(const uint32_t addr, int64_t value){
	//计算寄存器虚拟地址
	volatile int64_t *virt_addr = (volatile int64_t*) ( m_p_shared_base + ( addr & kSharedMemMapMask));

	//写寄存器
	*(virt_addr) = value;

	return true;
}

/**
 * @brief 读取轴编码器告警标志
 * @param value[out] : 返回读取的值
 * @return
 */
bool MICommunication::ReadEncoderWarn(uint64_t &value){
	bool res = true;

	volatile uint64_t *virt_addr = (volatile uint64_t*) ( m_p_shared_base + ( SHARED_MEM_MI_STATUS_ENCODER_WARN & kSharedMemMapMask));

	value = *virt_addr;

	return res;
}

/**
 * @brief 读取轴伺服告警标志
 * @param value[out] : 返回读取的值
 * @return
 */
bool MICommunication::ReadServoWarn(uint64_t &value){
	bool res = true;

	volatile uint64_t *virt_addr = (volatile uint64_t*) ( m_p_shared_base + ( SHARED_MEM_MI_STATUS_SVO_WARN & kSharedMemMapMask));

	value = *virt_addr;

	return res;
}

/**
 * @brief 读取伺服限位告警信息
 * @param pos_flag : true--正限位     false--负限位
 * @param value[out] : 返回读取的数据
 * @return  true--成功   false--失败
 */
bool MICommunication::ReadServoHLimitFlag(bool pos_flag, uint64_t &value){
	bool res = true;
	volatile uint64_t *virt_addr = nullptr;
	if(pos_flag){//正限位
		virt_addr = (volatile uint64_t*) ( m_p_shared_base + ( SHARED_MEM_MI_STATUS_HLIMIT_POS & kSharedMemMapMask));
	}else{//负限位
		virt_addr = (volatile uint64_t*) ( m_p_shared_base + ( SHARED_MEM_MI_STATUS_HLIMIT_NEG & kSharedMemMapMask));
	}

	value = *virt_addr;

	return res;
}

/**
 * @brief 写入物理轴硬限位标志
 * @param pos_flag[in]  : true--正限位     false--负限位
 * @param value[in] : 待写入的硬限位数据
 * @return true--成功   false--失败
 */
bool MICommunication::WriteAxisHLimitFlag(bool pos_flag, const uint64_t value){
	bool res = true;

	if(pos_flag){//正限位
		this->WriteRegister64(SHARED_MEM_SC_STATUS_HLIMIT_POS, value);
	}else{//负限位
		this->WriteRegister64(SHARED_MEM_SC_STATUS_HLIMIT_NEG, value);
	}

	return res;
}

/**
 * @brief 发送MI参数
 * @param axis : 轴号， MI的轴号从1开始, 0xFF表示系统参数
 * @param para_no : 参数号
 * @param data ：数据指针
 * @param size ：参数数据所占的字节数
 */
//template<typename T>
//void MICommunication::SendMiParam(uint8_t axis, uint32_t para_no, T data){
//    MiCmdFrame cmd;
//    memset(&cmd, 0x00, sizeof(cmd));
//    cmd.data.cmd = CMD_MI_SET_PARAM;

//    cmd.data.axis_index = axis;

//    memcpy(cmd.data.data, &para_no, 4);
//    memcpy(&cmd.data.data[2], &data, sizeof(T));

//    WriteCmd(cmd);
//}

/**
 * @brief 读取指定轴的伺服告警码
 * @param axis : 轴号
 * @param value[out] : 返回读取的值
 * @return
 */
bool MICommunication::ReadServoWarnCode(uint8_t axis, uint32_t &value){
	bool res = true;

	volatile uint32_t *virt_addr = (volatile uint32_t*) ( m_p_shared_base + ( SHARED_MEM_MI_STATUS_SVO_WARN_CODE(axis) & kSharedMemMapMask));

	value = *virt_addr;

	return res;
}

/**
 * @brief 读取轴告警标志
 * @param warn
 * @return
 */
bool MICommunication::ReadAxisWarnFlag(uint64_t &warn){
	bool res = true;

    ReadRegister64U_M(SHARED_MEM_MI_STATUS_WARN, warn);

	return res;
}

/**
 * @brief 读取轴使能状态
 * @param value
 * @return
 */
bool MICommunication::ReadServoOnState(uint64_t &value){
	bool res = true;

	ReadRegister64_M(SHARED_MEM_MI_STATUS_SVO_ON, value);

	return res;
}

/**
 * @brief 读取轴跟踪误差过大告警
 * @param value
 * @return
 */
bool MICommunication::ReadTrackErr(uint64_t &value){

    bool res = true;
	ReadRegister64_M(SHARED_MEM_MI_STATUS_SVO_TRACK_ERR, value);

	return res;
}

/**
 * @brief 读取同步轴指令偏差过大告警
 * @param value
 * @return
 */
bool MICommunication::ReadSyncPosErr(uint64_t &value){
	bool res = true;

	ReadRegister64_M(SHARED_MEM_MI_STATUS_SVO_SYNC_POS_ERR, value);

	return res;
}

/**
 * @brief 读取轴位置指令过大告警
 * @param value
 * @return
 */
bool MICommunication::ReadIntpPosErr(uint64_t &value){
	bool res = true;

	ReadRegister64_M(SHARED_MEM_MI_STATUS_SVO_INTP_POS_ERR, value);

	return res;
}

/**
 * @brief 读取同步轴力矩偏差过大告警
 * @param value
 * @return
 */
bool MICommunication::ReadSyncTorqueErr(uint64_t &value){
    bool res = true;
    ReadRegister64_M(SHARED_MEM_MI_STATUS_SVO_SYNC_TORQUE_ERR, value);
    return res;
}

/**
 * @brief 读取同步轴机床坐标偏差过大告警
 * @param value
 * @return
 */
bool MICommunication::ReadSyncMachErr(uint64_t &value)
{
    bool res = true;
    ReadRegister64_M(SHARED_MEM_MI_STATUS_SVO_SYNC_MACH_ERR, value);
    return res;
}

/**
 * @brief 读取通道轴位置
 * @param pos_fb[out] : 数组，返回轴位置反馈
 * @param pos_intp[out] : 数组，返回轴位置插补
 * @param count ：轴个数
 */ 
bool MICommunication::ReadPhyAxisCurFedBckPos(double *pos_fb, double *pos_intp,
                                              double *speed,double *torque,
                                              double *angle,uint8_t count){
	if(pos_fb == nullptr || pos_intp == nullptr)
        return false;

	//判断写完成标志
	int32_t flag = 0;
	ReadRegister32_M(SHARED_MEM_AXIS_WRITE_OVER, flag);
	if(flag == 0){
        //printf("$$$$ReadPhyAxisPos : write flag = 0, return \n");
        return false;   //MI未更新，直接返回
	}

	ReadRegister32_M(SHARED_MEM_AXIS_READ_OVER, flag);
	if(flag == 1){
        //printf("$$$$ReadPhyAxisPos : read flag = 1, return \n");
        return false;		//已读取，直接返回
	}

	double df = 0;
	int64_t pos_tmp;
	int32_t val_tmp;
    int16_t torque_tmp;
    uint16_t angle_tmp;
	for(int i = 0; i < count; i++){
		ReadRegister64_M(SHARED_MEM_AXIS_MAC_POS_FB(i), pos_tmp);  //读反馈位置
		df = pos_tmp;
		pos_fb[i] = df/1e7;	//转换单位  0.1nm->mm

		ReadRegister64_M(SHARED_MEM_AXIS_MAC_POS_INTP(i), pos_tmp); //读插补位置
//		ReadRegister64_M(SHARED_MEM_AXIS_MAC_POS_INTP_AFTER(i), pos_tmp);  //读反馈位置 //螺补功能测试  FOR TEST
		df = pos_tmp;
		pos_intp[i] = df/1e7;   //转换单位  0.1nm->mm

		if(speed != nullptr){
			ReadRegister32_M(SHARED_MEM_AXIS_SPEED(i), val_tmp);  //读反馈速度值
            speed[i] = val_tmp;
		}

		if(torque != nullptr){
            ReadRegister16_M(SHARED_MEM_AXIS_TORQUE(i), torque_tmp); //读反馈力矩值
            torque[i] = torque_tmp;
		}

        if(angle != nullptr){
            ReadRegister16U_M(SHARED_MEM_AXIS_SPD_ANGLE(i), angle_tmp); //读主轴角度
            angle[i] = angle_tmp/100.0;
        }
	}

	WriteRegister32_M(SHARED_MEM_AXIS_READ_OVER, 1);  //置位读取完成标志

    return true;
}

/**
 * @brief 读取指定物理轴的反馈位置
 * @param pos_fb[out] : 返回轴位置
 * @param phy_axis[in] : 指定物理轴号，从0开始
 * @param count[in] : 轴数量
 */
void MICommunication::ReadPhyAxisFbPos(double *pos_fb, uint8_t *phy_axis, uint8_t count){
	if(pos_fb == nullptr || phy_axis == nullptr)
		return;

	//判断写完成标志
	int32_t flag = 0;
	ReadRegister32_M(SHARED_MEM_AXIS_WRITE_OVER, flag);
	if(flag == 0){
		return;   //MI未更新，直接返回
	}

	double df = 0;
	int64_t pos_tmp;
	for(uint8_t i = 0; i < count; i++){
		pos_tmp = 0;
		if(phy_axis[i] != 0xFF)
			ReadRegister64_M(SHARED_MEM_AXIS_MAC_POS_FB(phy_axis[i]), pos_tmp);  //读反馈位置
		df = pos_tmp;
		pos_fb[i] = df/1e7;	//转换单位  0.1nm->mm
	}
}

#ifdef USES_SPEED_TORQUE_CTRL
/**
 * @brief 读取通道轴速度和力矩值
 * @param speed[out] : 数组，返回轴速度值
 * @param torque[out] : 数组，返回轴力矩值
 * @param count ：轴个数
 */ 
void MICommunication::ReadPhyAxisCurFedBckSpeedTorque(double *speed, double *torque, uint8_t count){
	if(speed == nullptr || torque == nullptr)
		return;

	//判断写完成标志
	int32_t flag = 0;
	ReadRegister32_M(SHARED_MEM_AXIS_WRITE_OVER, flag);
	if(flag == 0){
	//	printf("$$$$ReadPhyAxisPos : write flag = 0, return \n");
		return;   //MI未更新，直接返回
	}

	ReadRegister32_M(SHARED_MEM_AXIS_READ_OVER, flag);
	if(flag == 1){
	//	printf("$$$$ReadPhyAxisPos : read flag = 1, return \n");
		return;		//已读取，直接返回
	}

	double df = 0;
	int32_t val_tmp;
	for(int i = 0; i < count; i++){		
		ReadRegister32_M(SHARED_MEM_AXIS_SPEED(i), val_tmp);  //读反馈速度值
		df = val_tmp;
		speed[i] = df;	 

		ReadRegister32_M(SHARED_MEM_AXIS_TORQUE(i), val_tmp); //读反馈力矩值
		df = val_tmp;
		torque[i] = df;   
	}


	WriteRegister32_M(SHARED_MEM_AXIS_READ_OVER, 1);  //置位读取完成标志
}

#endif

void MICommunication::ReadSpindleAndle(uint16_t *angle, uint8_t count){ // 读取主轴角度
    if(!angle)
        return;
    //判断写完成标志
    int32_t flag = 0;
    ReadRegister32_M(SHARED_MEM_AXIS_WRITE_OVER, flag);
    if(flag == 0){
        return;   //MI未更新，直接返回
    }
}

/**
 * @brief 读取物理轴机械插补位置
 * @param pos[out] : 数组，返回轴插补机械位置
 * @param count ：轴个数
 */
//void MICommunication::ReadPhyAxisIntpPos(double *pos, uint8_t count){
//	//
//}

/**
 * @brief 读取PMC轴余移动量
 * @param pos[out] : 数组，返回轴余移动量
 * @param count ：轴个数
 * @return  true--成功   false--失败
 */
void MICommunication::ReadPmcAxisRemainDis(double *pos, uint8_t count){
	if(pos == nullptr)
		return;

	//判断写完成标志
	int32_t flag = 0;
	ReadRegister32_M(SHARED_MEM_AXIS_WRITE_OVER, flag);
	if(flag == 0){
	//	printf("$$$$ReadPhyAxisPos : write flag = 0, return \n");
		return;   //MI未更新，直接返回
	}

//	ReadRegister32_M(SHARED_MEM_AXIS_READ_OVER, flag);
//	if(flag == 1){
//	//	printf("$$$$ReadPhyAxisPos : read flag = 1, return \n");
//		return false;		//已读取，直接返回
//	}


	double df = 0;
	int64_t pos_tmp;
	for(int i = 0; i < count; i++){
		ReadRegister64_M(SHARED_MEM_PMC_AXIS_REMAIN_DIS(i), pos_tmp);
		df = pos_tmp;
		pos[i] = df/1e7;	//转换单位  0.1nm->mm

	}


	WriteRegister32_M(SHARED_MEM_AXIS_READ_OVER, 1);  //置位读取完成标志
}

/**
 * @brief 读取刚性攻丝误差
 * @param err:返回的误差
 * @param group ：攻丝组 0-7
 */
void MICommunication::ReadTapErr(int32_t *err, int32_t *err_now, uint8_t cnt)
{
    //判断写完成标志
    int32_t flag = 0;
    ReadRegister32_M(SHARED_MEM_TAP_WRITE_OVER, flag);
    if(flag == 0){
        //ScPrintf("$$$$ReadTapErr : write flag = 0, return \n");
        return;   //MI未更新，直接返回
    }

    int32_t var_err = 0;
    int32_t var_err_now = 0;
    for(int i=0; i<cnt; i++){
        ReadRegister32_M(SHARED_MEM_TAP_ERR(i), var_err);
        ReadRegister32_M(SHARED_MEM_TAP_ERR_NOW(i), var_err_now);
        //ScPrintf("read addr = %x, value = %d",SHARED_MEM_TAP_ERR(i), var_err);
        //ScPrintf("read over flag addr = %x",SHARED_MEM_TAP_READ_OVER);
        err[i] = var_err;
        err_now[i] = var_err_now;
    }
    WriteRegister32_M(SHARED_MEM_TAP_READ_OVER, 1);  //置位读取完成标志
}

///**
// * @brief 读取刚性攻丝误差(当前值)
// * @param err:返回的误差
// * @param group ：攻丝组 0-7
// */
//void MICommunication::ReadTapErrNow(int32_t &err, uint8_t group)
//{
//    //判断写完成标志
//    int32_t flag = 0;
//    ReadRegister32_M(SHARED_MEM_TAP_WRITE_OVER, flag);
//    if(flag == 0){
//        return;   //MI未更新，直接返回
//    }


//    ReadRegister32_M(SHARED_MEM_TAP_ERR_NOW(group), err);
//    ScPrintf("SHARED_MEM_TAP_ERR_NEW(group) = %x, value = %d",SHARED_MEM_TAP_ERR_NOW(group),err);
//    WriteRegister32_M(SHARED_MEM_TAP_READ_OVER, 1);  //置位读取完成标志
//}

/**
 * @brief 读取指定物理轴速度
 * @param speed[out] : 返回读取的速度值
 * @param index : 指定物理轴序号，从0开始
 * @return true--成功   false--失败
 */
bool MICommunication::ReadPhyAxisSpeed(int32_t *speed, uint8_t index){
	if(index >= g_ptr_parm_manager->GetSystemConfig()->axis_count ||
			speed == nullptr)
		return false;

	//判断写完成标志
	int32_t flag = 0;
	ReadRegister32_M(SHARED_MEM_AXIS_WRITE_OVER, flag);
	if(flag == 0){
		return false;   //MI未更新，直接返回
	}
//
//	ReadRegister32_M(SHARED_MEM_AXIS_READ_OVER, flag);
//	if(flag == 1){
//		return false;		//已读取，直接返回
//	}

	ReadRegister32_M(SHARED_MEM_AXIS_SPEED(index), *speed);
	return true;
}

/**
 * @brief 读取物理轴当前编码器反馈
 * @param encoder[out] : 32位无符号整型数组，用于反馈输出
 * @param count ： 数组大小，元素个数
 */
void MICommunication::ReadPhyAxisEncoder(int64_t *encoder, uint8_t count){

	for(int i = 0; i < count; i++){
		ReadRegister64_M(SHARED_MEM_AXIS_ENCODER(i), encoder[i]);
	}
}

/**
 * @brief 读取指令
 * @param data	： 返回读取的指令
 * @return true--读到数据   false--未读取到数据
 */
bool MICommunication::ReadCmd(MiCmdFrame &data){

	int16_t flag1 = 0, flag2 = 0;
	ReadRegister16_M(SHARED_MEM_CMD_RECV_WF(m_n_cur_recv_cmd_index), flag1);
	ReadRegister16_M(SHARED_MEM_CMD_RECV_RF(m_n_cur_recv_cmd_index), flag2);

	if(flag1 == 1 && flag2 == 0){//有数据待读取
		//读取数据
		ReadRegister32_M(SHARED_MEM_CMD_RECV_DATA0(m_n_cur_recv_cmd_index), data.data_w[0]);
		ReadRegister32_M(SHARED_MEM_CMD_RECV_DATA1(m_n_cur_recv_cmd_index), data.data_w[1]);
		ReadRegister32_M(SHARED_MEM_CMD_RECV_DATA2(m_n_cur_recv_cmd_index), data.data_w[2]);
		ReadRegister32_M(SHARED_MEM_CMD_RECV_DATA3(m_n_cur_recv_cmd_index), data.data_w[3]);
		ReadRegister32_M(SHARED_MEM_CMD_RECV_DATA4(m_n_cur_recv_cmd_index), data.data_w[4]);

		//置位读标志
		WriteRegister16_M(SHARED_MEM_CMD_RECV_RF(m_n_cur_recv_cmd_index), 1);

		//读取指针下移
        if(++m_n_cur_recv_cmd_index >= MI_CMD_FIFO_DEPTH){
			m_n_cur_recv_cmd_index = 0;
        }

	}
	else
		return false;

	return true;
}

/**
 * @brief 写入指令
 * @param data : 待写入的指令
 * @param resend : 是否重发命令
 * @return true--写入成功    false--写入失败
 */
bool MICommunication::WriteCmd(MiCmdFrame &data, bool resend){

	bool res = true;
	bool wf = false;	//当前命令帧是否已写入过数据
	uint16_t flag = 0;

	CalMiCmdCrc(data);	//计算CRC
	pthread_mutex_lock(&m_mutex_cmd_down);//上锁

	if(!resend && this->m_list_cmd->GetLength() > 0){ //非重发并且重发队列非空，则直接进入重发队列，防止命令乱序
		//将命令压入缓冲队列
		if(!this->m_list_cmd->Append(data)){
			g_ptr_trace->PrintLog(LOG_ALARM, "MI命令缓冲分配内存失败！");
			res = false;
		}
		goto END;

	}

	ReadRegister16_M(SHARED_MEM_CMD_SEND_WF(m_n_cur_send_cmd_index), flag);
	if(flag == 1){//此命令帧还未被复位
		flag = 0;

		ReadRegister16_M(SHARED_MEM_CMD_SEND_RF(m_n_cur_send_cmd_index), flag);
		if(flag != 1){//命令还未读走，意味着MI命令通道满
			if(resend){ //重发失败，直接返回
				res = false;
				goto END;
			}

			//将命令压入缓冲队列
			if(!this->m_list_cmd->Append(data)){
				g_ptr_trace->PrintLog(LOG_ALARM, "MI命令缓冲分配内存失败！");
				res = false;
			}
			goto END;
		}

		wf = true;
	}

	//写入命令数据
	WriteRegister32_M(SHARED_MEM_CMD_SEND_DATA0(m_n_cur_send_cmd_index), data.data_w[0]);
	WriteRegister32_M(SHARED_MEM_CMD_SEND_DATA1(m_n_cur_send_cmd_index), data.data_w[1]);
	WriteRegister32_M(SHARED_MEM_CMD_SEND_DATA2(m_n_cur_send_cmd_index), data.data_w[2]);
	WriteRegister32_M(SHARED_MEM_CMD_SEND_DATA3(m_n_cur_send_cmd_index), data.data_w[3]);
	WriteRegister32_M(SHARED_MEM_CMD_SEND_DATA4(m_n_cur_send_cmd_index), data.data_w[4]);

	//置位写入标志
	if(wf){
		WriteRegister16_M(SHARED_MEM_CMD_SEND_RF(m_n_cur_send_cmd_index), 0);	//已写入过数据则只需要复位读取标志
	}
	else{
		WriteRegister16_M(SHARED_MEM_CMD_SEND_WF(m_n_cur_send_cmd_index), 1);	//未写入过数据则只需要置位写入标志
	}

	//写入指针下移
	if(++m_n_cur_send_cmd_index >= MI_CMD_FIFO_DEPTH)
		m_n_cur_send_cmd_index = 0;

	END:
	pthread_mutex_unlock(&m_mutex_cmd_down);  //卸锁
	return res;
}

/**
 * @brief 发送PMC轴运动指令
 * @param data : 数据帧
 * @param resend : 是否重发命令
 * @return true--写入成功    false--写入失败
 */
bool MICommunication::SendPmcCmd(PmcCmdFrame &data, bool resend){

	bool res = true;
	bool wf = false;	//当前命令帧是否已写入过数据
	uint16_t flag = 0;

	CalPmcCmdCrc(data);	//计算CRC

	pthread_mutex_lock(&m_mutex_pmc_cmd);//上锁

	if(!resend && this->m_list_pmc->GetLength() > 0){ //非重发并且重发队列非空，则直接进入重发队列，防止命令乱序
		//将命令压入缓冲队列
		if(!this->m_list_pmc->Append(data)){
			g_ptr_trace->PrintLog(LOG_ALARM, "PMC运动指令缓冲分配内存失败！");
			res = false;
		}
		goto END;

	}

	ReadRegister16_M(SHARED_MEM_CMD_PMC_WF(m_n_cur_send_pmc_index), flag);
	if(flag == 1){//此命令帧还未被复位
		flag = 0;

		ReadRegister16_M(SHARED_MEM_CMD_PMC_RF(m_n_cur_send_pmc_index), flag);
		if(flag != 1){//命令还未读走，意味着PMC运动指令通道满
			if(resend){ //重发失败，直接返回
				res = false;
				goto END;
			}

			//将命令压入缓冲队列
			if(!this->m_list_pmc->Append(data)){
				g_ptr_trace->PrintLog(LOG_ALARM, "PMC运动指令缓冲分配内存失败！");
				res = false;
			}
			goto END;
		}

		wf = true;
	}

	//写入命令数据
	WriteRegister32_M(SHARED_MEM_CMD_PMC_DATA0(m_n_cur_send_pmc_index), data.data_w[0]);
	WriteRegister32_M(SHARED_MEM_CMD_PMC_DATA1(m_n_cur_send_pmc_index), data.data_w[1]);
	WriteRegister32_M(SHARED_MEM_CMD_PMC_DATA2(m_n_cur_send_pmc_index), data.data_w[2]);
	WriteRegister32_M(SHARED_MEM_CMD_PMC_DATA3(m_n_cur_send_pmc_index), data.data_w[3]);
	WriteRegister32_M(SHARED_MEM_CMD_PMC_DATA4(m_n_cur_send_pmc_index), data.data_w[4]);

	//置位写入标志
	if(wf){
		WriteRegister16_M(SHARED_MEM_CMD_PMC_RF(m_n_cur_send_pmc_index), 0);	//已写入过数据则只需要复位读取标志
	}
	else{
		WriteRegister16_M(SHARED_MEM_CMD_PMC_WF(m_n_cur_send_pmc_index), 1);	//未写入过数据则只需要置位写入标志
	}

	//写入指针下移
	if(++m_n_cur_send_pmc_index >= PMC_CMD_FIFO_DEPTH)
		m_n_cur_send_pmc_index = 0;

	END:
	pthread_mutex_unlock(&m_mutex_pmc_cmd);  //卸锁
	return res;

}

/**
 * @brief 计算MI命令包的CRC
 * @param cmd : 命令包
 */
void MICommunication::CalMiCmdCrc(MiCmdFrame &cmd){
	uint16_t crc = 0xffff;
	int count = kMaxCmdCount*2-1;	//不包括CRC字段本身
	for(int i = 0; i < count; i++)
		crc ^= cmd.data_crc[i];

	cmd.data.crc = crc;
}

/**
 * @brief 计算PMC命令包的CRC
 * @param cmd : 命令包
 */
void MICommunication::CalPmcCmdCrc(PmcCmdFrame &cmd){
	uint16_t crc = 0xffff;
	int count = kMaxCmdCount*2-1;	//不包括CRC字段本身
	for(int i = 0; i < count; i++)
		crc ^= cmd.data_crc[i];

	cmd.data.crc = crc;
}

/**
 * @brief 设置轴参考点
 * @param axis
 * @param encoder
 * @return
 */
bool MICommunication::SetAxisRef(uint8_t axis, int64_t encoder){
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(MiCmdFrame));
	cmd.data.axis_index = axis;
	cmd.data.cmd = CMD_MI_SET_REF;
    memcpy(&cmd.data.data, &encoder, sizeof(int64_t));

	return this->WriteCmd(cmd);
}


void MICommunication::SetAxisRefCur(uint8_t axis, double mach_pos)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_REF_CUR;
    cmd.data.axis_index = axis;
//	cmd.data.reserved = type;
    int64_t pos = mach_pos*1e7;   //单位转换,0.1nm
    memcpy(cmd.data.data, &pos, sizeof(int64_t));

    WriteCmd(cmd);
}

/**
 * @brief 获取PMC单字节寄存器的值
 * @param reg_type : 寄存器类型
 * @param value ：返回寄存器值
 * @return
 */
bool MICommunication::GetPmcRegByte(const int reg_type, uint8_t &value){


	return true;
}

/**
 * @brief 设置PMC单字节寄存器的值
 * @param reg_type : 寄存器类型
 * @param value ： 待写入的值
 * @return
 */
bool MICommunication::SetPmcRegByte(const int reg_type, const uint8_t value){


	return true;
}

/**
 * @brief 获取PMC双字节字节寄存器的值
 * @param reg_type : 寄存器类型
 * @param value ：返回寄存器值
 * @return
 */
bool MICommunication::GetPmcRegWord(const int reg_type, uint16_t &value){


	return true;
}

/**
 * @brief 设置PMC双字节寄存器的值
 * @param reg_type : 寄存器类型
 * @param value ： 待写入的值
 * @return
 */
bool MICommunication::SetPmcRegWord(const int reg_type, const uint16_t value){


	return true;
}

/**
 * @brief 读取PMC寄存器，按地址段读取
 * @param sec
 * @param reg
 * @param share
 * @return
 */
bool MICommunication::ReadPmcReg(int sec, uint8_t *reg){
	//读取PMC寄存器，按地址段读取
	uint32_t addr;
	int size = 0;   //字节数
	switch(sec){
	case PMC_REG_X:
		addr = SHARED_MEM_REG_TO_NC_X;
		size = X_REG_COUNT;
		break;
	case PMC_REG_Y:
		addr = SHARED_MEM_REG_TO_NC_Y;
		size = Y_REG_COUNT;
		break;
	case PMC_REG_G:
		addr = SHARED_MEM_REG_TO_NC_G;
		size = G_REG_COUNT;
		break;
	case PMC_REG_R:
		addr = SHARED_MEM_REG_TO_NC_R;
		size = R_REG_COUNT;
		break;
	case PMC_REG_K:
		addr = SHARED_MEM_REG_TO_NC_K;
		size = K_REG_COUNT;
		break;
	case PMC_REG_A:
		addr = SHARED_MEM_REG_TO_NC_A;
		size = A_REG_COUNT;
		break;
	case PMC_REG_D:
		addr = SHARED_MEM_REG_TO_NC_D;
#ifndef USES_PMC_2_0
		size = D_REG_COUNT * 2;
#else
		size = D_REG_COUNT;
#endif
		break;
	case PMC_REG_C:
		addr = SHARED_MEM_REG_TO_NC_C;
#ifndef USES_PMC_2_0
		size = C_REG_COUNT * 2;
#else
		size = C_REG_COUNT * 4;
#endif
		break;
	case PMC_REG_T:
		addr = SHARED_MEM_REG_TO_NC_T;
		size = T_REG_COUNT * 2;
		break;
#ifndef USES_PMC_2_0
	case PMC_REG_DC:
		addr = SHARED_MEM_REG_TO_NC_DC;
		size = C_REG_COUNT * 2;
		break;
	case PMC_REG_DT:
		addr = SHARED_MEM_REG_TO_NC_DT;
		size = T_REG_COUNT * 2;
		break;
#else
//    case PMC_REG_T_C:
//        addr = SHARED_MEM_REG_TO_NC_TC;
//        size = TC_REG_COUNT;
//        break;
//    case PMC_REG_T_M:
//        addr = SHARED_MEM_REG_TO_NC_TM;
//        size = TM_REG_COUNT;
//        break;
	case PMC_REG_E:
		addr = SHARED_MEM_REG_TO_NC_E;
		size = E_REG_COUNT;
		break;
#endif
	default:
		g_ptr_trace->PrintTrace(TRACE_ERROR, MI_COMMUNICATION_SC, "无法读取的PMC寄存器类型[%d]", sec);
		return false;
	}
    int8_t *virt_addr = (int8_t*) (m_p_shared_base + ( addr & kSharedMemMapMask));

	memcpy(reg, virt_addr, size);

	return true;
}

bool MICommunication::ReadPmcPeriod(){
	// PMC 运行周期寄存器偏移 0x8E000   读写标志偏移  0x8FFF0

	int64_t * period = (int64_t *)(m_p_shared_base + 0x8E000);
	int32_t * RWFlag = (int32_t *)(m_p_shared_base + 0x8FFF0);

	if(*RWFlag != 0) return false;

	int64_t p1 = *period;
	int64_t p2 = *(period + 1);

	pmc_period1 = p1 / 1000.0;
	pmc_period2 = p2 / 1000.0;

	//printf("pmc ==========  %lf %lf\n", pmc_period1, pmc_period2);

	*RWFlag = 1;

    return true;
}

/**
 * @brief 写入PMC寄存器，按地址段写入
 * @param sec
 * @param reg
 * @return
 */
bool MICommunication::WritePmcReg(int sec, uint8_t *reg){
	//写入PMC寄存器，按地址段写入
	//读取PMC寄存器，按地址段读取
	uint32_t addr;
	int size = 0;   //字节数
	switch(sec){
//	case PMC_REG_X:
//		addr = SHARED_MEM_REG_TO_NC_X;
//		size = X_REG_COUNT;
//		break;
//	case PMC_REG_Y:
//		addr = SHARED_MEM_REG_TO_NC_Y;
//		size = Y_REG_COUNT;
//		break;
	case PMC_REG_F:
		addr = SHARED_MEM_REG_TO_PMC_F;
		size = F_REG_COUNT;
		break;
//	case PMC_REG_G:
//		addr = SHARED_MEM_REG_TO_NC_G;
//		size = G_REG_COUNT;
//		break;
//	case PMC_REG_R:
//		addr = SHARED_MEM_REG_TO_NC_R;
//		size = R_REG_COUNT;
//		break;
//	case PMC_REG_K:
//		addr = SHARED_MEM_REG_TO_NC_K;
//		size = K_REG_COUNT;
//		break;
//	case PMC_REG_A:
//		addr = SHARED_MEM_REG_TO_NC_A;
//		size = A_REG_COUNT;
//		break;
//	case PMC_REG_D:
//		addr = SHARED_MEM_REG_TO_NC_D;
//		size = D_REG_COUNT * 2;
//		break;
//	case PMC_REG_C:
//		addr = SHARED_MEM_REG_TO_NC_C;
//		size = C_REG_COUNT * 2;
//		break;
//	case PMC_REG_T:
//		addr = SHARED_MEM_REG_TO_NC_T;
//		size = T_REG_COUNT * 2;
//		break;
//	case PMC_REG_DC:
//		addr = SHARED_MEM_REG_TO_NC_DC;
//		size = C_REG_COUNT * 2;
//		break;
//	case PMC_REG_DT:
//		addr = SHARED_MEM_REG_TO_NC_DT;
//		size = T_REG_COUNT * 2;
//		break;
	default:
		g_ptr_trace->PrintTrace(TRACE_ERROR, MI_COMMUNICATION_SC, "无法写入的PMC寄存器类型[%d]", sec);
		return false;
	}
    int8_t *virt_addr = (int8_t*) (m_p_shared_base + ( addr & kSharedMemMapMask));

	memcpy(virt_addr, reg, size);

	return true;
}


/**
 * @brief  SC-MI命令处理线程
 * @param args
 */
void *MICommunication::ProcessCmdThread(void *args){
	printf("Start MICommunication::ProcessCmdThread!thread id = %ld\n", syscall(SYS_gettid));

	MICommunication *mi_comm = static_cast<MICommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
	if (res != ERR_NONE) {
		printf("Quit MICommunication::ProcessCmdThread!thread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
	if (res != ERR_NONE) {
		printf("Quit MICommunication::ProcessCmdThread!thread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	while((g_sys_state.module_ready_mask & SC_READY) == 0){  //等待SC模块初始化完成
		usleep(10000);
	}

	res = mi_comm->ProcessCmdFun();

	printf("Quit MICommunication::ProcessCmdThread!!\n");
	pthread_exit(NULL);
}

/**
 * @brief 命令处理函数
 * @return
 */
bool MICommunication::ProcessCmdFun(){
	MiCmdFrame cmd_frame;
	ListNode<MiCmdFrame> *node = nullptr;


	ListNode<PmcCmdFrame> *pmc_node = nullptr;

	while(!g_sys_state.system_quit){//程序不退出

		//处理待发送的命令
		while(!this->m_list_cmd->IsEmpty()){
			node = this->m_list_cmd->HeadNode();
			if(this->WriteCmd(static_cast<MiCmdFrame &>(node->data), true)){
//				g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "重发PMC命令消息成功！");
				this->m_list_cmd->Delete(node);
				node = nullptr;
			}else{
//				g_ptr_trace->PrintLog(LOG_ALARM, "重发PMC命令消息失败！");
				break; //还是发送失败，跳出循环
			}
		}

		if(this->m_list_cmd->GetLength() > m_n_threshold)
			g_ptr_trace->PrintLog(LOG_ALARM, "待重发MI命令消息过多[%d]！", m_list_cmd->GetLength());

		//处理待发送PMC运动指令
		while(!this->m_list_pmc->IsEmpty()){
			pmc_node = this->m_list_pmc->HeadNode();
			if(this->SendPmcCmd(static_cast<PmcCmdFrame &>(pmc_node->data), true)){
//				g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "重发PMC命令消息成功！");
				this->m_list_pmc->Delete(pmc_node);
				pmc_node = nullptr;
			}else{
//				g_ptr_trace->PrintLog(LOG_ALARM, "重发PMC命令消息失败！");
				break; //还是发送失败，跳出循环
			}
		}

		//处理上行命令通道数据
		if(!this->ReadCmd(cmd_frame)){
			//无数据则睡眠2ms
			usleep(2000);
			continue;
		}

		//处理数据包
		//TODO 校验CRC
		if(!CheckCrc(cmd_frame.data_crc, kMaxCmdCount*2-1, cmd_frame.data.crc)){
			//CRC校验失败
            CreateError(ERR_MI_CMD_CRC, WARNING_LEVEL, CLEAR_BY_MCP_RESET);  //生成MI通讯CRC校验错告警
			continue;
		}

		//转给ChannelEngine执行
		m_p_channel_engine->ProcessMiCmd(cmd_frame);
        //printf("get mi cmd : %d \n", cmd_frame.data.cmd);
	}

	return true;
}




