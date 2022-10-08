/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file MICommunication.h
 *@author gonghao
 *@date 2020/06/15
 *@brief ��ͷ�ļ�ΪSC-MIͨѶ���ʵ��
 *@version
 */


#include "global_include.h"
#include "mi_communication.h"
#include "channel_engine.h"
#include "pmc_register.h"


const uint32_t kSharedMemMapSize = (uint32_t) 1024*1024;   	//ӳ������С 1M
const uint32_t kSharedMemMapMask = (kSharedMemMapSize - 1);	 	//��ַ���� 0x0FFFFF


//�Ĵ�����д�궨�庯������߷���Ч��
#define WriteRegister64_M(x,y) {volatile int64_t *virt_addr = (volatile int64_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	*(virt_addr) = (int64_t)y;}

#define WriteRegister32_M(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	*(virt_addr) = (int32_t)y;}

#define WriteRegister16_M(x,y) {volatile int16_t *virt_addr = (volatile int16_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	*(virt_addr) = (int16_t)y;}

#define ReadRegister64_M(x,y) {volatile int64_t *virt_addr = (volatile int64_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	y = *(virt_addr);}

#define ReadRegister32_M(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	y = *(virt_addr);}

#define ReadRegister32U_M(x,y) {volatile uint32_t *virt_addr = (volatile uint32_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	y = *(virt_addr);}

#define ReadRegister16_M(x,y) {volatile int16_t *virt_addr = (volatile int16_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	y = *(virt_addr);}

#define ReadRegister8_M(x,y) {volatile int8_t *virt_addr = (volatile int8_t*) ( m_p_shared_base + ((int32_t)x & kSharedMemMapMask));\
	y = *(virt_addr);}


MICommunication* MICommunication::m_p_instance = nullptr;  //��ʼ����������ָ��Ϊ��
//template<> int ListNode<MiCmdFrame>::new_count = 0;

/**
 * @brief ���캯��
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


	//��ʼ����������
	this->m_list_cmd = new MiCmdBuffer();
	if(m_list_cmd == nullptr){
		m_error_code = ERR_MI_COM_INIT;
		goto END;
	}

	//��ʼ��PMCָ������
	this->m_list_pmc = new PmcCmdBuffer();
	if(m_list_pmc == nullptr){
		m_error_code = ERR_MI_COM_INIT;
		goto END;
	}

	//��������ʼ��
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
 * @brief ��������
 */
MICommunication::~MICommunication() {
	// TODO Auto-generated destructor stub
	this->QuitThread();
	this->CloseSharedMemory();

	//�ͷ���������
	if(this->m_list_cmd)
		delete m_list_cmd;

	//�ͷ�PMC���˶���������
	if(this->m_list_pmc)
		delete m_list_pmc;

	pthread_mutex_destroy(&m_mutex_cmd_down);	//��������д�뻥����
	pthread_mutex_destroy(&m_mutex_pmc_cmd);	//����PMCָ��д�뻥����
}

/**
 * @brief ��ȡ��ʵ����Ψһ���ʽӿں���������ģʽ
 */
MICommunication* MICommunication::GetInstance(){
	if(NULL == m_p_instance){
		m_p_instance = new MICommunication();
	}
	return m_p_instance;
}

/**
 * @brief ���ýӿ�ָ��
 * @param engine : ͨ���������ָ��
 */
void MICommunication::SetInterface(){
	m_p_channel_engine = ChannelEngine::GetInstance();
}

/**
 * @brief ��ʼ���߳�
 * @return
 */
int MICommunication::InitThread(){
	int res = ERR_NONE;
	pthread_attr_t attr;
	struct sched_param param;

	//�����߳�
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 37; //97;
	pthread_attr_setschedparam(&attr, &param);
	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
	if (res) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MIͨѶ�ӿ�������߳������̼̳߳�ģʽʧ�ܣ�");
		m_error_code = ERR_MI_COM_INIT;
		goto END;
	}
	res = pthread_create(&m_thread_process_cmd, &attr,
			MICommunication::ProcessCmdThread, this);    //����MI������߳�
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MIͨѶ�ӿ�������߳�����ʧ�ܣ�");
		res = ERR_MI_COM_INIT;
		goto END;
	}

	END:
	pthread_attr_destroy(&attr);
	return res;
}

/**
 * @brief �˳��߳�
 * @return
 */
int MICommunication::QuitThread(){
	int res = ERR_NONE;
	void* thread_result;

	//�˳�������߳�
	res = pthread_cancel(m_thread_process_cmd);
	if (res != ERR_NONE) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MIͨѶ�ӿ�������߳��˳�ʧ�ܣ�");
	}

	usleep(1000);
	res = pthread_join(m_thread_process_cmd, &thread_result);//�ȴ�������߳��˳����
	if (res != ERR_NONE) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MIͨѶ�ӿڵȴ�������߳��˳�ʧ�ܣ�");
	}
	m_thread_process_cmd = 0;


	return res;
}

/**
 * @brief ��ʼ�������ڴ�
 * @return
 */
bool MICommunication::InitSharedMemory(){

	//���豸�ļ�
	m_n_mem_file = open("/dev/mem", O_RDWR | O_SYNC);
	if (m_n_mem_file == -1) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MICommunication::InitSharedMemory() failed to open file! errno = %d", errno);
		return false;
	}

	//ӳ���ڴ��ַ���õ�����ַ
	m_p_shared_base = (uint8_t *) mmap(nullptr, kSharedMemMapSize, PROT_READ | PROT_WRITE, MAP_SHARED,
			m_n_mem_file, SHARED_MEM_BASE & (~kSharedMemMapMask));
	if (m_p_shared_base == MAP_FAILED) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MICommunication::InitSharedMemory() failed to map memory! errno = %d", errno);
		return false;
	}

	//��ʼ������������ͨ��
	InitCmdChannel();

	WriteRegister32_M(SHARED_MEM_AXIS_READ_OVER, 0);  //��ʼ����ȡ��ɱ�־Ϊ0

	printf("mem file : %d, base_addr : %p\n", m_n_mem_file, m_p_shared_base);
	return true;
}

/**
 * @brief ��ȡPMC����ͼ�������ĵ�ַ
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

void MICommunication::SendAxisEnableCmd(uint8_t axis, bool enable)
{
    // optΪ1����������ʹ�ܲ���
    SendOperateCmd(1,axis,enable);
}

void MICommunication::SendTapAxisCmd(uint8_t chn, uint8_t spd_axis, uint8_t z_axis)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_TAP_AXIS;
    cmd.data.axis_index = 0xFF;
    cmd.data.reserved = chn;
    // Э���е�������Ŵ�1��ʼ
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

void MICommunication::SendTapParams(uint8_t chn, uint16_t error_gain, uint16_t feed_gain, uint16_t ratio_gain)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_TAP_PARAM;
    cmd.data.axis_index = 0xFF;
    cmd.data.reserved = chn;

    uint32_t param = error_gain*1000;
    memcpy(&cmd.data.data[0], &param, sizeof(uint32_t));

    param = feed_gain*1000;
    memcpy(&cmd.data.data[2], &param, sizeof(uint32_t));

    param = ratio_gain*1000;
    memcpy(&cmd.data.data[4], &param, sizeof(uint32_t));

    WriteCmd(cmd);
}

void MICommunication::SendTapStateCmd(uint8_t chn, bool enable)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_TAP_STATE;
    cmd.data.axis_index = 0xFF;
    cmd.data.reserved = chn;
    cmd.data.data[0] = enable?1:0;
    WriteCmd(cmd);
}

void MICommunication::SendSpdLocateCmd(uint8_t chn, uint8_t axis)
{
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SPD_LOCATE;
    cmd.data.axis_index = axis;
    cmd.data.reserved = chn;
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

/**
 * @brief ��ʼ������������ͨ��������������֡�Ķ�д��־λ��0
 */
void MICommunication::InitCmdChannel(){
	for(int i = 0; i < MI_CMD_FIFO_DEPTH; i++){
		WriteRegister32_M(SHARED_MEM_CMD_SEND_WF(i), 0);
	//	WriteRegister32_M(SHARED_MEM_CMD_RECV_WF(i), 0);
	}
}

/**
 * @brief �رչ����ڴ�
 * @return
 */
bool MICommunication::CloseSharedMemory(){

	//ȡ���ڴ�ӳ��
	if(m_p_shared_base != MAP_FAILED)
		munmap(m_p_shared_base, kSharedMemMapSize);

	//�ر��豸�ļ�
	if(m_n_mem_file != -1) {
		close(m_n_mem_file);
	}

	printf("MICommunication::CloseSharedMemory()\n");
	return true;
}

/**
 * @brief ��ȡ�Ĵ���
 * @param addr : �Ĵ�����ַƫ��
 * @param value �����ض���������
 * @return
 */
bool MICommunication::ReadRegister64(const uint32_t addr, int64_t& value){
	//����Ĵ��������ַ
	volatile int64_t *virt_addr = (volatile int64_t*) (m_p_shared_base + ( addr & kSharedMemMapMask));

	//���Ĵ���ֵ
	value = *(virt_addr);
	return true;
}

/**
 * @brief ��ȡ�Ĵ���
 * @param addr : �Ĵ�����ַƫ��
 * @param value �����ض���������
 * @return
 */
bool MICommunication::ReadRegister32(const uint32_t addr, int32_t& value){
	//����Ĵ��������ַ
	volatile int32_t *virt_addr = (volatile int32_t*) (m_p_shared_base + ( addr & kSharedMemMapMask));

	//���Ĵ���ֵ
	value = *(virt_addr);
	return true;
}

/**
 * @brief д��Ĵ���
 * @param addr : �Ĵ�����ַƫ��
 * @param value ����д�������
 * @return
 */
bool MICommunication::WriteRegister32(const uint32_t addr, int32_t value){
	//����Ĵ��������ַ
	volatile int32_t *virt_addr = (volatile int32_t*) ( m_p_shared_base + ( addr & kSharedMemMapMask));

	//д�Ĵ���
	*(virt_addr) = value;

	return true;
}

/**
 * @brief д��Ĵ���
 * @param addr : �Ĵ�����ַƫ��
 * @param value ����д�������
 * @return
 */
bool MICommunication::WriteRegister64(const uint32_t addr, int64_t value){
	//����Ĵ��������ַ
	volatile int64_t *virt_addr = (volatile int64_t*) ( m_p_shared_base + ( addr & kSharedMemMapMask));

	//д�Ĵ���
	*(virt_addr) = value;

	return true;
}

/**
 * @brief ��ȡ��������澯��־
 * @param value[out] : ���ض�ȡ��ֵ
 * @return
 */
bool MICommunication::ReadEncoderWarn(uint64_t &value){
	bool res = true;

	volatile uint64_t *virt_addr = (volatile uint64_t*) ( m_p_shared_base + ( SHARED_MEM_MI_STATUS_ENCODER_WARN & kSharedMemMapMask));

	value = *virt_addr;

	return res;
}

/**
 * @brief ��ȡ���ŷ��澯��־
 * @param value[out] : ���ض�ȡ��ֵ
 * @return
 */
bool MICommunication::ReadServoWarn(uint64_t &value){
	bool res = true;

	volatile uint64_t *virt_addr = (volatile uint64_t*) ( m_p_shared_base + ( SHARED_MEM_MI_STATUS_SVO_WARN & kSharedMemMapMask));

	value = *virt_addr;

	return res;
}

/**
 * @brief ��ȡ�ŷ���λ�澯��Ϣ
 * @param pos_flag : true--����λ     false--����λ
 * @param value[out] : ���ض�ȡ������
 * @return  true--�ɹ�   false--ʧ��
 */
bool MICommunication::ReadServoHLimitFlag(bool pos_flag, uint64_t &value){
	bool res = true;
	volatile uint64_t *virt_addr = nullptr;
	if(pos_flag){//����λ
		virt_addr = (volatile uint64_t*) ( m_p_shared_base + ( SHARED_MEM_MI_STATUS_HLIMIT_POS & kSharedMemMapMask));
	}else{//����λ
		virt_addr = (volatile uint64_t*) ( m_p_shared_base + ( SHARED_MEM_MI_STATUS_HLIMIT_NEG & kSharedMemMapMask));
	}

	value = *virt_addr;

	return res;
}

/**
 * @brief д��������Ӳ��λ��־
 * @param pos_flag[in]  : true--����λ     false--����λ
 * @param value[in] : ��д���Ӳ��λ����
 * @return true--�ɹ�   false--ʧ��
 */
bool MICommunication::WriteAxisHLimitFlag(bool pos_flag, const uint64_t value){
	bool res = true;

	if(pos_flag){//����λ
		this->WriteRegister64(SHARED_MEM_SC_STATUS_HLIMIT_POS, value);
	}else{//����λ
		this->WriteRegister64(SHARED_MEM_SC_STATUS_HLIMIT_NEG, value);
	}

	return res;
}


/**
 * @brief ��ȡָ������ŷ��澯��
 * @param axis : ���
 * @param value[out] : ���ض�ȡ��ֵ
 * @return
 */
bool MICommunication::ReadServoWarnCode(uint8_t axis, uint32_t &value){
	bool res = true;

	volatile uint32_t *virt_addr = (volatile uint32_t*) ( m_p_shared_base + ( SHARED_MEM_MI_STATUS_SVO_WARN_CODE(axis) & kSharedMemMapMask));

	value = *virt_addr;

	return res;
}

/**
 * @brief ��ȡ��澯��־
 * @param warn
 * @return
 */
bool MICommunication::ReadAxisWarnFlag(int8_t &warn){
	bool res = true;

	ReadRegister8_M(SHARED_MEM_MI_STATUS_WARN, warn);

	return res;
}

/**
 * @brief ��ȡ��ʹ��״̬
 * @param value
 * @return
 */
bool MICommunication::ReadServoOnState(int64_t &value){
	bool res = true;

	ReadRegister64_M(SHARED_MEM_MI_STATUS_SVO_ON, value);

	return res;
}

/**
 * @brief ��ȡ�����������澯
 * @param value
 * @return
 */
bool MICommunication::ReadTrackErr(uint64_t &value){
	bool res = true;

	ReadRegister64_M(SHARED_MEM_MI_STATUS_SVO_TRACK_ERR, value);

	return res;
}

/**
 * @brief ��ȡͬ����ָ��ƫ�����澯
 * @param value
 * @return
 */
bool MICommunication::ReadSyncPosErr(uint64_t &value){
	bool res = true;

	ReadRegister64_M(SHARED_MEM_MI_STATUS_SVO_SYNC_POS_ERR, value);

	return res;
}

/**
 * @brief ��ȡ��λ��ָ�����澯
 * @param value
 * @return
 */
bool MICommunication::ReadIntpPosErr(uint64_t &value){
	bool res = true;

	ReadRegister64_M(SHARED_MEM_MI_STATUS_SVO_INTP_POS_ERR, value);

	return res;
}

/**
 * @brief ��ȡͨ����λ��
 * @param pos_fb[out] : ���飬������λ�÷���
 * @param pos_intp[out] : ���飬������λ�ò岹
 * @param count �������
 */ 
void MICommunication::ReadPhyAxisCurFedBckPos(double *pos_fb, double *pos_intp,double *speed,double *torque, uint8_t count){
	if(pos_fb == nullptr || pos_intp == nullptr)
		return;

	//�ж�д��ɱ�־
	int32_t flag = 0;
	ReadRegister32_M(SHARED_MEM_AXIS_WRITE_OVER, flag);
	if(flag == 0){
	//	printf("$$$$ReadPhyAxisPos : write flag = 0, return \n");
		return;   //MIδ���£�ֱ�ӷ���
	}

	ReadRegister32_M(SHARED_MEM_AXIS_READ_OVER, flag);
	if(flag == 1){
	//	printf("$$$$ReadPhyAxisPos : read flag = 1, return \n");
		return;		//�Ѷ�ȡ��ֱ�ӷ���
	}


	double df = 0;
	int64_t pos_tmp;
	int32_t val_tmp;
	for(int i = 0; i < count; i++){
		ReadRegister64_M(SHARED_MEM_AXIS_MAC_POS_FB(i), pos_tmp);  //������λ��
		df = pos_tmp;
		pos_fb[i] = df/1e7;	//ת����λ  0.1nm->mm

		ReadRegister64_M(SHARED_MEM_AXIS_MAC_POS_INTP(i), pos_tmp); //���岹λ��
//		ReadRegister64_M(SHARED_MEM_AXIS_MAC_POS_INTP_AFTER(i), pos_tmp);  //������λ�� //�ݲ����ܲ���  FOR TEST
		df = pos_tmp;
		pos_intp[i] = df/1e7;   //ת����λ  0.1nm->mm

		if(speed != nullptr){
			ReadRegister32_M(SHARED_MEM_AXIS_SPEED(i), val_tmp);  //�������ٶ�ֵ
			df = val_tmp;
			speed[i] = df;
		}

		if(torque != nullptr){
			ReadRegister32_M(SHARED_MEM_AXIS_TORQUE(i), val_tmp); //����������ֵ
			df = val_tmp;
			torque[i] = df;
		}
	}

	WriteRegister32_M(SHARED_MEM_AXIS_READ_OVER, 1);  //��λ��ȡ��ɱ�־
}

/**
 * @brief ��ȡָ��������ķ���λ��
 * @param pos_fb[out] : ������λ��
 * @param phy_axis[in] : ָ��������ţ���0��ʼ
 * @param count[in] : ������
 */
void MICommunication::ReadPhyAxisFbPos(double *pos_fb, uint8_t *phy_axis, uint8_t count){
	if(pos_fb == nullptr || phy_axis == nullptr)
		return;

	//�ж�д��ɱ�־
	int32_t flag = 0;
	ReadRegister32_M(SHARED_MEM_AXIS_WRITE_OVER, flag);
	if(flag == 0){
		return;   //MIδ���£�ֱ�ӷ���
	}

	double df = 0;
	int64_t pos_tmp;
	for(uint8_t i = 0; i < count; i++){
		pos_tmp = 0;
		if(phy_axis[i] != 0xFF)
			ReadRegister64_M(SHARED_MEM_AXIS_MAC_POS_FB(phy_axis[i]), pos_tmp);  //������λ��
		df = pos_tmp;
		pos_fb[i] = df/1e7;	//ת����λ  0.1nm->mm
	}
}

#ifdef USES_SPEED_TORQUE_CTRL
/**
 * @brief ��ȡͨ�����ٶȺ�����ֵ
 * @param speed[out] : ���飬�������ٶ�ֵ
 * @param torque[out] : ���飬����������ֵ
 * @param count �������
 */ 
void MICommunication::ReadPhyAxisCurFedBckSpeedTorque(double *speed, double *torque, uint8_t count){
	if(speed == nullptr || torque == nullptr)
		return;

	//�ж�д��ɱ�־
	int32_t flag = 0;
	ReadRegister32_M(SHARED_MEM_AXIS_WRITE_OVER, flag);
	if(flag == 0){
	//	printf("$$$$ReadPhyAxisPos : write flag = 0, return \n");
		return;   //MIδ���£�ֱ�ӷ���
	}

	ReadRegister32_M(SHARED_MEM_AXIS_READ_OVER, flag);
	if(flag == 1){
	//	printf("$$$$ReadPhyAxisPos : read flag = 1, return \n");
		return;		//�Ѷ�ȡ��ֱ�ӷ���
	}

	double df = 0;
	int32_t val_tmp;
	for(int i = 0; i < count; i++){		
		ReadRegister32_M(SHARED_MEM_AXIS_SPEED(i), val_tmp);  //�������ٶ�ֵ
		df = val_tmp;
		speed[i] = df;	 

		ReadRegister32_M(SHARED_MEM_AXIS_TORQUE(i), val_tmp); //����������ֵ
		df = val_tmp;
		torque[i] = df;   
	}


	WriteRegister32_M(SHARED_MEM_AXIS_READ_OVER, 1);  //��λ��ȡ��ɱ�־
}

#endif

/**
 * @brief ��ȡ�������е�岹λ��
 * @param pos[out] : ���飬������岹��еλ��
 * @param count �������
 */
//void MICommunication::ReadPhyAxisIntpPos(double *pos, uint8_t count){
//	//
//}

/**
 * @brief ��ȡPMC�����ƶ���
 * @param pos[out] : ���飬���������ƶ���
 * @param count �������
 * @return  true--�ɹ�   false--ʧ��
 */
void MICommunication::ReadPmcAxisRemainDis(double *pos, uint8_t count){
	if(pos == nullptr)
		return;

	//�ж�д��ɱ�־
	int32_t flag = 0;
	ReadRegister32_M(SHARED_MEM_AXIS_WRITE_OVER, flag);
	if(flag == 0){
	//	printf("$$$$ReadPhyAxisPos : write flag = 0, return \n");
		return;   //MIδ���£�ֱ�ӷ���
	}

//	ReadRegister32_M(SHARED_MEM_AXIS_READ_OVER, flag);
//	if(flag == 1){
//	//	printf("$$$$ReadPhyAxisPos : read flag = 1, return \n");
//		return false;		//�Ѷ�ȡ��ֱ�ӷ���
//	}


	double df = 0;
	int64_t pos_tmp;
	for(int i = 0; i < count; i++){
		ReadRegister64_M(SHARED_MEM_PMC_AXIS_REMAIN_DIS(i), pos_tmp);
		df = pos_tmp;
		pos[i] = df/1e7;	//ת����λ  0.1nm->mm

	}


	WriteRegister32_M(SHARED_MEM_AXIS_READ_OVER, 1);  //��λ��ȡ��ɱ�־
}

/**
 * @brief ��ȡָ���������ٶ�
 * @param speed[out] : ���ض�ȡ���ٶ�ֵ
 * @param index : ָ����������ţ���0��ʼ
 * @return true--�ɹ�   false--ʧ��
 */
bool MICommunication::ReadPhyAxisSpeed(int32_t *speed, uint8_t index){
	if(index >= g_ptr_parm_manager->GetSystemConfig()->axis_count ||
			speed == nullptr)
		return false;

	//�ж�д��ɱ�־
	int32_t flag = 0;
	ReadRegister32_M(SHARED_MEM_AXIS_WRITE_OVER, flag);
	if(flag == 0){
		return false;   //MIδ���£�ֱ�ӷ���
	}
//
//	ReadRegister32_M(SHARED_MEM_AXIS_READ_OVER, flag);
//	if(flag == 1){
//		return false;		//�Ѷ�ȡ��ֱ�ӷ���
//	}

	ReadRegister32_M(SHARED_MEM_AXIS_SPEED(index), *speed);
	return true;
}

/**
 * @brief ��ȡ�����ᵱǰ����������
 * @param encoder[out] : 32λ�޷����������飬���ڷ������
 * @param count �� �����С��Ԫ�ظ���
 */
void MICommunication::ReadPhyAxisEncoder(int64_t *encoder, uint8_t count){

	for(int i = 0; i < count; i++){
		ReadRegister64_M(SHARED_MEM_AXIS_ENCODER(i), encoder[i]);
	}
}

/**
 * @brief ��ȡָ��
 * @param data	�� ���ض�ȡ��ָ��
 * @return true--��������   false--δ��ȡ������
 */
bool MICommunication::ReadCmd(MiCmdFrame &data){

	int16_t flag1 = 0, flag2 = 0;
	ReadRegister16_M(SHARED_MEM_CMD_RECV_WF(m_n_cur_recv_cmd_index), flag1);
	ReadRegister16_M(SHARED_MEM_CMD_RECV_RF(m_n_cur_recv_cmd_index), flag2);



	if(flag1 == 1 && flag2 == 0){//�����ݴ���ȡ
	//	printf("read mi cmd: %d, %d, addr=0x%x\n", flag1, flag2, SHARED_MEM_CMD_RECV_WF(m_n_cur_recv_cmd_index));

		//��ȡ����
		ReadRegister32_M(SHARED_MEM_CMD_RECV_DATA0(m_n_cur_recv_cmd_index), data.data_w[0]);
		ReadRegister32_M(SHARED_MEM_CMD_RECV_DATA1(m_n_cur_recv_cmd_index), data.data_w[1]);
		ReadRegister32_M(SHARED_MEM_CMD_RECV_DATA2(m_n_cur_recv_cmd_index), data.data_w[2]);
		ReadRegister32_M(SHARED_MEM_CMD_RECV_DATA3(m_n_cur_recv_cmd_index), data.data_w[3]);
		ReadRegister32_M(SHARED_MEM_CMD_RECV_DATA4(m_n_cur_recv_cmd_index), data.data_w[4]);

		//��λ����־
		WriteRegister16_M(SHARED_MEM_CMD_RECV_RF(m_n_cur_recv_cmd_index), 1);

		//��ȡָ������
		if(++m_n_cur_recv_cmd_index >= MI_CMD_FIFO_DEPTH)
			m_n_cur_recv_cmd_index = 0;

	}
	else
		return false;

	return true;
}

/**
 * @brief д��ָ��
 * @param data : ��д���ָ��
 * @param resend : �Ƿ��ط�����
 * @return true--д��ɹ�    false--д��ʧ��
 */
bool MICommunication::WriteCmd(MiCmdFrame &data, bool resend){

	bool res = true;
	bool wf = false;	//��ǰ����֡�Ƿ���д�������
	uint16_t flag = 0;

	CalMiCmdCrc(data);	//����CRC

	pthread_mutex_lock(&m_mutex_cmd_down);//����

	if(!resend && this->m_list_cmd->GetLength() > 0){ //���ط������ط����зǿգ���ֱ�ӽ����ط����У���ֹ��������
		//������ѹ�뻺�����
		if(!this->m_list_cmd->Append(data)){
			g_ptr_trace->PrintLog(LOG_ALARM, "MI���������ڴ�ʧ�ܣ�");
			res = false;
		}
		goto END;

	}

	ReadRegister16_M(SHARED_MEM_CMD_SEND_WF(m_n_cur_send_cmd_index), flag);
	if(flag == 1){//������֡��δ����λ
		flag = 0;

		ReadRegister16_M(SHARED_MEM_CMD_SEND_RF(m_n_cur_send_cmd_index), flag);
		if(flag != 1){//���δ���ߣ���ζ��MI����ͨ����
			if(resend){ //�ط�ʧ�ܣ�ֱ�ӷ���
				res = false;
				goto END;
			}

			//������ѹ�뻺�����
			if(!this->m_list_cmd->Append(data)){
				g_ptr_trace->PrintLog(LOG_ALARM, "MI���������ڴ�ʧ�ܣ�");
				res = false;
			}
			goto END;
		}

		wf = true;
	}

	//д����������
	WriteRegister32_M(SHARED_MEM_CMD_SEND_DATA0(m_n_cur_send_cmd_index), data.data_w[0]);
	WriteRegister32_M(SHARED_MEM_CMD_SEND_DATA1(m_n_cur_send_cmd_index), data.data_w[1]);
	WriteRegister32_M(SHARED_MEM_CMD_SEND_DATA2(m_n_cur_send_cmd_index), data.data_w[2]);
	WriteRegister32_M(SHARED_MEM_CMD_SEND_DATA3(m_n_cur_send_cmd_index), data.data_w[3]);
	WriteRegister32_M(SHARED_MEM_CMD_SEND_DATA4(m_n_cur_send_cmd_index), data.data_w[4]);

	//��λд���־
	if(wf){
		WriteRegister16_M(SHARED_MEM_CMD_SEND_RF(m_n_cur_send_cmd_index), 0);	//��д���������ֻ��Ҫ��λ��ȡ��־
	}
	else{
		WriteRegister16_M(SHARED_MEM_CMD_SEND_WF(m_n_cur_send_cmd_index), 1);	//δд���������ֻ��Ҫ��λд���־
	}

	//д��ָ������
	if(++m_n_cur_send_cmd_index >= MI_CMD_FIFO_DEPTH)
		m_n_cur_send_cmd_index = 0;

	END:
	pthread_mutex_unlock(&m_mutex_cmd_down);  //ж��
	return res;
}

/**
 * @brief ����PMC���˶�ָ��
 * @param data : ����֡
 * @param resend : �Ƿ��ط�����
 * @return true--д��ɹ�    false--д��ʧ��
 */
bool MICommunication::SendPmcCmd(PmcCmdFrame &data, bool resend){

	bool res = true;
	bool wf = false;	//��ǰ����֡�Ƿ���д�������
	uint16_t flag = 0;

	CalPmcCmdCrc(data);	//����CRC

	pthread_mutex_lock(&m_mutex_pmc_cmd);//����

	if(!resend && this->m_list_pmc->GetLength() > 0){ //���ط������ط����зǿգ���ֱ�ӽ����ط����У���ֹ��������
		//������ѹ�뻺�����
		if(!this->m_list_pmc->Append(data)){
			g_ptr_trace->PrintLog(LOG_ALARM, "PMC�˶�ָ�������ڴ�ʧ�ܣ�");
			res = false;
		}
		goto END;

	}

	ReadRegister16_M(SHARED_MEM_CMD_PMC_WF(m_n_cur_send_pmc_index), flag);
	if(flag == 1){//������֡��δ����λ
		flag = 0;

		ReadRegister16_M(SHARED_MEM_CMD_PMC_RF(m_n_cur_send_pmc_index), flag);
		if(flag != 1){//���δ���ߣ���ζ��PMC�˶�ָ��ͨ����
			if(resend){ //�ط�ʧ�ܣ�ֱ�ӷ���
				res = false;
				goto END;
			}

			//������ѹ�뻺�����
			if(!this->m_list_pmc->Append(data)){
				g_ptr_trace->PrintLog(LOG_ALARM, "PMC�˶�ָ�������ڴ�ʧ�ܣ�");
				res = false;
			}
			goto END;
		}

		wf = true;
	}

	//д����������
	WriteRegister32_M(SHARED_MEM_CMD_PMC_DATA0(m_n_cur_send_pmc_index), data.data_w[0]);
	WriteRegister32_M(SHARED_MEM_CMD_PMC_DATA1(m_n_cur_send_pmc_index), data.data_w[1]);
	WriteRegister32_M(SHARED_MEM_CMD_PMC_DATA2(m_n_cur_send_pmc_index), data.data_w[2]);
	WriteRegister32_M(SHARED_MEM_CMD_PMC_DATA3(m_n_cur_send_pmc_index), data.data_w[3]);
	WriteRegister32_M(SHARED_MEM_CMD_PMC_DATA4(m_n_cur_send_pmc_index), data.data_w[4]);

	//��λд���־
	if(wf){
		WriteRegister16_M(SHARED_MEM_CMD_PMC_RF(m_n_cur_send_pmc_index), 0);	//��д���������ֻ��Ҫ��λ��ȡ��־
	}
	else{
		WriteRegister16_M(SHARED_MEM_CMD_PMC_WF(m_n_cur_send_pmc_index), 1);	//δд���������ֻ��Ҫ��λд���־
	}

	//д��ָ������
	if(++m_n_cur_send_pmc_index >= PMC_CMD_FIFO_DEPTH)
		m_n_cur_send_pmc_index = 0;

	END:
	pthread_mutex_unlock(&m_mutex_pmc_cmd);  //ж��
	return res;

}

/**
 * @brief ����MI�������CRC
 * @param cmd : �����
 */
void MICommunication::CalMiCmdCrc(MiCmdFrame &cmd){
	uint16_t crc = 0xffff;
	int count = kMaxCmdCount*2-1;	//������CRC�ֶα���
	for(int i = 0; i < count; i++)
		crc ^= cmd.data_crc[i];

	cmd.data.crc = crc;
}

/**
 * @brief ����PMC�������CRC
 * @param cmd : �����
 */
void MICommunication::CalPmcCmdCrc(PmcCmdFrame &cmd){
	uint16_t crc = 0xffff;
	int count = kMaxCmdCount*2-1;	//������CRC�ֶα���
	for(int i = 0; i < count; i++)
		crc ^= cmd.data_crc[i];

	cmd.data.crc = crc;
}

/**
 * @brief ������ο���
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

/**
 * @brief ��ȡPMC���ֽڼĴ�����ֵ
 * @param reg_type : �Ĵ�������
 * @param value �����ؼĴ���ֵ
 * @return
 */
bool MICommunication::GetPmcRegByte(const int reg_type, uint8_t &value){


	return true;
}

/**
 * @brief ����PMC���ֽڼĴ�����ֵ
 * @param reg_type : �Ĵ�������
 * @param value �� ��д���ֵ
 * @return
 */
bool MICommunication::SetPmcRegByte(const int reg_type, const uint8_t value){


	return true;
}

/**
 * @brief ��ȡPMC˫�ֽ��ֽڼĴ�����ֵ
 * @param reg_type : �Ĵ�������
 * @param value �����ؼĴ���ֵ
 * @return
 */
bool MICommunication::GetPmcRegWord(const int reg_type, uint16_t &value){


	return true;
}

/**
 * @brief ����PMC˫�ֽڼĴ�����ֵ
 * @param reg_type : �Ĵ�������
 * @param value �� ��д���ֵ
 * @return
 */
bool MICommunication::SetPmcRegWord(const int reg_type, const uint16_t value){


	return true;
}

/**
 * @brief ��ȡPMC�Ĵ���������ַ�ζ�ȡ
 * @param sec
 * @param reg
 * @param share
 * @return
 */
bool MICommunication::ReadPmcReg(int sec, uint8_t *reg){
	//��ȡPMC�Ĵ���������ַ�ζ�ȡ
	uint32_t addr;
	int size = 0;   //�ֽ���
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
	case PMC_REG_E:
		addr = SHARED_MEM_REG_TO_NC_E;
		size = E_REG_COUNT;
		break;
#endif
	default:
		g_ptr_trace->PrintTrace(TRACE_ERROR, MI_COMMUNICATION_SC, "�޷���ȡ��PMC�Ĵ�������[%d]", sec);
		return false;
	}
    int8_t *virt_addr = (int8_t*) (m_p_shared_base + ( addr & kSharedMemMapMask));

	memcpy(reg, virt_addr, size);

	return true;
}

/**
 * @brief д��PMC�Ĵ���������ַ��д��
 * @param sec
 * @param reg
 * @return
 */
bool MICommunication::WritePmcReg(int sec, uint8_t *reg){
	//д��PMC�Ĵ���������ַ��д��
	//��ȡPMC�Ĵ���������ַ�ζ�ȡ
	uint32_t addr;
	int size = 0;   //�ֽ���
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
		g_ptr_trace->PrintTrace(TRACE_ERROR, MI_COMMUNICATION_SC, "�޷�д���PMC�Ĵ�������[%d]", sec);
		return false;
	}
    int8_t *virt_addr = (int8_t*) (m_p_shared_base + ( addr & kSharedMemMapMask));

	memcpy(virt_addr, reg, size);

	return true;
}


/**
 * @brief  SC-MI������߳�
 * @param args
 */
void *MICommunication::ProcessCmdThread(void *args){
	printf("Start MICommunication::ProcessCmdThread!thread id = %ld\n", syscall(SYS_gettid));

	MICommunication *mi_comm = static_cast<MICommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
	if (res != ERR_NONE) {
		printf("Quit MICommunication::ProcessCmdThread!thread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
	if (res != ERR_NONE) {
		printf("Quit MICommunication::ProcessCmdThread!thread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	while((g_sys_state.module_ready_mask & SC_READY) == 0){  //�ȴ�SCģ���ʼ�����
		usleep(10000);
	}

	res = mi_comm->ProcessCmdFun();

	printf("Quit MICommunication::ProcessCmdThread!!\n");
	pthread_exit(NULL);
}

/**
 * @brief �������
 * @return
 */
bool MICommunication::ProcessCmdFun(){
	MiCmdFrame cmd_frame;
	ListNode<MiCmdFrame> *node = nullptr;


	ListNode<PmcCmdFrame> *pmc_node = nullptr;

	while(!g_sys_state.system_quit){//�����˳�

		//��������͵�����
		while(!this->m_list_cmd->IsEmpty()){
			node = this->m_list_cmd->HeadNode();
			if(this->WriteCmd(static_cast<MiCmdFrame &>(node->data), true)){
//				g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "�ط�PMC������Ϣ�ɹ���");
				this->m_list_cmd->Delete(node);
				node = nullptr;
			}else{
//				g_ptr_trace->PrintLog(LOG_ALARM, "�ط�PMC������Ϣʧ�ܣ�");
				break; //���Ƿ���ʧ�ܣ�����ѭ��
			}
		}

		if(this->m_list_cmd->GetLength() > m_n_threshold)
			g_ptr_trace->PrintLog(LOG_ALARM, "���ط�MI������Ϣ����[%d]��", m_list_cmd->GetLength());

		//���������PMC�˶�ָ��
		while(!this->m_list_pmc->IsEmpty()){
			pmc_node = this->m_list_pmc->HeadNode();
			if(this->SendPmcCmd(static_cast<PmcCmdFrame &>(pmc_node->data), true)){
//				g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "�ط�PMC������Ϣ�ɹ���");
				this->m_list_pmc->Delete(pmc_node);
				pmc_node = nullptr;
			}else{
//				g_ptr_trace->PrintLog(LOG_ALARM, "�ط�PMC������Ϣʧ�ܣ�");
				break; //���Ƿ���ʧ�ܣ�����ѭ��
			}
		}

		//������������ͨ������
		if(!this->ReadCmd(cmd_frame)){
			//��������˯��2ms
			usleep(2000);
			continue;
		}

		//�������ݰ�
		//TODO У��CRC
		if(!CheckCrc(cmd_frame.data_crc, kMaxCmdCount*2-1, cmd_frame.data.crc)){
			//CRCУ��ʧ��
			CreateError(ERR_MI_CMD_CRC, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON);  //����MIͨѶCRCУ���澯
			continue;
		}

//		switch(cmd_frame.data.cmd){
//		case CMD_MI_SHAKEHAND:
//			break;
//		}


		//ת��ChannelEngineִ��
		m_p_channel_engine->ProcessMiCmd(cmd_frame);
//		printf("get mi cmd : %d \n", cmd_frame.data.cmd);
	}


	return true;
}




