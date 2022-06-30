/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file MCCommunication.h
 *@author gonghao
 *@date 2020/06/15
 *@brief ��ͷ�ļ�ΪSC-MCͨѶ���ʵ��
 *@version
 */

#include "mc_communication.h"
#include "global_include.h"
#include "channel_engine.h"

const uint32_t kRegMapSize = (uint32_t) 2*1024;   	//ӳ������С 4K
const uint32_t kRegMemMapMask = (kRegMapSize - 1);	 //��ַ���� 0x07FF

//�Ĵ�����д�궨�庯������߷���Ч��
#define WriteRegister(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( m_p_addr_base + ((int32_t)x & kRegMemMapMask));\
	*(virt_addr) = (int32_t)y;}


#define ReadRegister(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( m_p_addr_base + ((int32_t)x & kRegMemMapMask));\
	y = *(virt_addr);}

#define ReadRegister_16(x,y) {volatile int16_t *virt_addr = (volatile int16_t*) ( m_p_addr_base + ((int32_t)x & kRegMemMapMask));\
	y = *(virt_addr);}


//������32λ���ͺϲ�Ϊһ��64λ����
#define Merge32_64(l,h) ((int64_t)(((uint64_t)l) | (((uint64_t)h)<<32)))

MCCommunication* MCCommunication::m_p_instance = nullptr;  //��ʼ����������ָ��Ϊ��
//template<> int ListNode<McCmdFrame>::new_count = 0;


/**
 * @brief ���캯��
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
		CreateError(m_error_code, FATAL_LEVEL, CLEAR_BY_RESET_POWER);  //����MCͨѶģ���ʼ��ʧ�ܸ澯

//	printf("sizeof GCodeData: %d\n", sizeof(GCodeData));

}

/**
 * @brief ��������
 */
MCCommunication::~MCCommunication() {
	// TODO Auto-generated destructor stub
	this->Clean();
}

/**
 * @brief ��ȡ��ʵ����Ψһ���ʽӿں���������ģʽ
 */
MCCommunication* MCCommunication::GetInstance(){
	if(nullptr == m_p_instance){
		m_p_instance = new MCCommunication();
	}
	return m_p_instance;
}

/**
 * @brief ���ýӿ�ָ��
 * @param engine : ͨ���������ָ��
 */
void MCCommunication::SetInterface(){
	m_p_channel_engine = ChannelEngine::GetInstance();
}

/**
 * @brief ��ʼ���������򿪵�ַӳ������
 * @return true--�ɹ�   false--ʧ��
 */
bool MCCommunication::Initialize(){
	int res = ERR_NONE;
	pthread_attr_t attr;
	struct sched_param param;

	//��������ʼ��
	pthread_mutex_init(&m_mutex_cmd, nullptr);

	//���������
	this->m_list_cmd = new McCmdBuffer();
	if(m_list_cmd == nullptr){
		g_ptr_trace->PrintLog(LOG_ALARM, "MCCommunication::Initialize() ����ʼ�������������пռ�ʧ�ܣ�");
		m_error_code = ERR_MC_COM_INIT;
		return false;
	}

	//״̬��ȡ�ź�����ʼ��Ϊ0
	m_n_read_status_sem = 0;

	//���豸�ļ�
	m_n_mem_file = open("/dev/mem", O_RDWR | O_SYNC);
	if (m_n_mem_file == -1) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCCommunication::Initialize() failed to open file[/dev/mem]! errno = %d", errno);
		m_error_code = ERR_MC_COM_INIT;
		return false;
	}

	//ӳ���ڴ��ַ���õ�����ַ
	m_p_addr_base = (uint8_t *) mmap(nullptr, kRegMapSize, PROT_READ | PROT_WRITE, MAP_SHARED,
			m_n_mem_file, MC_REGISTER_BASE & (~kRegMemMapMask));
	if (m_p_addr_base == MAP_FAILED) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCCommunication::Initialize() failed to map memory! errno = %d", errno);
		m_error_code = ERR_MC_COM_INIT;

		return false;
	}

	//�����߳�
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 37; //97;
	pthread_attr_setschedparam(&attr, &param);
	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
	if (res) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCͨѶ�ӿ�������߳������̼̳߳�ģʽʧ�ܣ�");
		m_error_code = ERR_MI_COM_INIT;
		goto END;
	}
	res = pthread_create(&m_thread_process_cmd, &attr,
			MCCommunication::ProcessCmdThread, this);    //����MC������߳�
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCͨѶ�ӿ�������߳�����ʧ�ܣ�");
		res = ERR_MI_COM_INIT;
		goto END;
	}

	END:
	pthread_attr_destroy(&attr);

	printf("MCCommunication::Initialize(), mem file : %d, base_addr : %p\n", m_n_mem_file, m_p_addr_base);

	return true;
}

/**
 * @brief ������
 * @return true--�ɹ�   false--ʧ��
 */
bool MCCommunication::Clean(){

	//�˳��߳�
	int res = ERR_NONE;
	void* thread_result;

	//��մ������������
	if(this->m_list_cmd){
		delete m_list_cmd;
		m_list_cmd = nullptr;
	}

	//�˳�������߳�
	res = pthread_cancel(m_thread_process_cmd);
	if (res != ERR_NONE) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCͨѶ�ӿ�������߳��˳�ʧ�ܣ�");
	}

	usleep(1000);
	res = pthread_join(m_thread_process_cmd, &thread_result);//�ȴ�������߳��˳����
	if (res != ERR_NONE) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCͨѶ�ӿڵȴ�������߳��˳�ʧ�ܣ�");
	}
	m_thread_process_cmd = 0;

	//ȡ���ڴ�ӳ��
	if(m_p_addr_base != MAP_FAILED)
		munmap(m_p_addr_base, kRegMapSize);

	//�ر��豸�ļ�
	if(m_n_mem_file != -1) {
		close(m_n_mem_file);
	}



	pthread_mutex_destroy(&m_mutex_cmd);

	printf("MCCommunication::Clean()\n");
	return true;
}

/**
 * @brief �Ƿ���д���˶���������
 * @return  true -- ����    false -- ������
 */
bool MCCommunication::CanWriteGCode(uint8_t chn){
	//��ȡFIFO�������ж��ܷ�д��
	uint32_t count = 0;
	ReadRegister(MC_GCODE_FIFO_COUNT(chn), count);
	if(count >= kMaxGCodeFifoCount)
		return false;   //FIFO���������޷�д��
	return true;
}

/**
 * @brief д��G�����������˶���������
 * @param chn : ͨ�������ţ�0��ʼ
 * @param data ����д�������֡
 * @return true--�ɹ�   false--ʧ��,������
 */
bool MCCommunication::WriteGCodeData(uint8_t chn, GCodeFrame &data){
	CalMcGCodeFrameCrc(data);

	//��ȡFIFO�������ж��ܷ�д��
	uint32_t count = 0;
	ReadRegister(MC_GCODE_FIFO_COUNT(chn), count);
	if(count >= kMaxGCodeFifoCount)
		return false;   //FIFO���������޷�д��

	WriteRegister(MC_GCODE_WRITE_OVER(chn), 0);  //д��ǰ����

	//��������
	for(int i = 0; i < kMaxGCodeDataCount; i++){
		WriteRegister(MC_GCODE_DATA(chn,i), data.data_w[i]);
	}

	WriteRegister(MC_GCODE_WRITE_OVER(chn), 1);  //д�����һ

	return true;
}

/**
 * @brief ��ȡ�����˿�����FIFO�����ݸ���
 * @param chn : ͨ�������ţ�0��ʼ
 * @return fifo��ǰ���ݸ���
 */
uint32_t MCCommunication::ReadGCodeFifoCount(uint8_t chn){
	//��ȡ�����˿�����FIFO�����ݸ���
	uint32_t count = 0;
	ReadRegister(MC_GCODE_FIFO_COUNT(chn), count);
	return count;
}

/**
 * @brief ��ȡͨ������ģʽ������ָ��
 * @param chn_index[in] : ͨ��������
 * @param work_mode[out] : ��ǰ�Ĺ���ģʽ
 * @return ��
 */
void MCCommunication::ReadWorkMode(const uint8_t chn_index, uint16_t &work_mode){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

//	uint32_t data;
	ReadRegister_16(MC_WORK_MODE(chn_index), work_mode);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0

//	printf("@@@@@@mc data: 0x%08x\n", data);

//	work_mode = data&0xFFFF;
}

/**
 * @brief ��ȡͨ����ǰ����ָ��
 * @param chn_index[in] : ͨ��������
 * @param cur_cmd[out] : ��ǰ���е�ָ��
 * @return ��
 */
void MCCommunication::ReadCurCmd(const uint8_t chn_index, uint16_t &cur_cmd){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	uint32_t data;
	ReadRegister(MC_WORK_MODE(chn_index), data);

	cur_cmd = (data&0xFFFF0000)>>16;

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}

/**
 * @brief ��ȡ��岹λ�ã�������ǰλ�ã���������ϵ������Ŀ��λ��
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param cur_pos[out] : ��ǰ�岹λ��
 * @param tar_pos[out] ����ǰĿ��λ��
 * @return
 */
void MCCommunication::ReadAxisIntpPos(const uint8_t chn_index, DPoint &cur_pos, DPoint &tar_pos){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

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
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}


/**
 * @brief ��ȡͨ����ǰ�Զ����ݻ�������
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param count[out] : ������������
 * @return
 */
void MCCommunication::ReadAutoBufDataCount(const uint8_t chn_index, uint16_t &count){

//	uint32_t tmp_count = 0;
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister_16(MC_AUTO_BUF_DATA_COUNT(chn_index), count);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0

//	if(chn_index % 2){//����ͨ��
//		count = (uint16_t)((tmp_count >> 16)&0x0000FFFF);
//	}
//	else{
//		count = (uint16_t)(tmp_count & 0x0000FFFF);
//	}
}

//
/**
 * @brief ��ȡͨ����ǰMDA���ݻ�������
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param count[out] : ������������
 */
void MCCommunication::ReadMdaBufDataCount(const uint8_t chn_index, uint16_t &count){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister_16(MC_MDA_BUF_DATA_COUNT(chn_index), count);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}

/**
 * @brief ��ȡͨ����ǰ�����ٶ�
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param cur_feed[out] : ���ص�ǰ�����ٶ�
 * @return
 */
void MCCommunication::ReadCurFeed(const uint8_t chn_index, uint32_t &cur_feed){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister(MC_CUR_FEED(chn_index), cur_feed);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}

/**
 * @brief ��ȡͨ����ǰ���������ٶ�
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param cur_feed[out] : ���ص�ǰ���������ٶ�
 * @return
 */
void MCCommunication::ReadRatedFeed(const uint8_t chn_index, uint32_t &rated_feed){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister(MC_RATED_FEED(chn_index), rated_feed);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}

/**
 * @brief ��ȡͨ����ǰ�����к�
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param cur_lineno[out] : ���ص�ǰ�к�
 * @return
 */
void MCCommunication::ReadCurLineNo(const uint8_t chn_index, uint32_t &cur_lineno){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister(MC_CUR_LINE_NO(chn_index), cur_lineno);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}


/**
 * @brief ��ȡͨ����ǰ�����ۼ�����
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param tool_life[out] : ���ص�ǰ��������
 */
void MCCommunication::ReadChnCurToolLife(uint8_t chn_index, int32_t &tool_life){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister(MC_CHN_CUR_TOOL_LIFE(chn_index), tool_life);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}


/**
 * @brief ��ȡͨ����ǰ����ϵ
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param coord[out] : ���ص�ǰ����ϵ���
 */
void MCCommunication::ReadChnCurCoord(uint8_t chn_index, uint16_t &coord){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister_16(MC_CUR_COORD(chn_index), coord);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}


/**
 * @brief ��ȡͨ����ǰ��ƫ��
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param tool_offset[out] : ���ص�ǰ����ƫ��
 */
void MCCommunication::ReadChnCurToolOffset(uint8_t chn_index, uint16_t &tool_offset){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister_16(MC_CUR_TOOL_OFFSET(chn_index), tool_offset);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}


/**
 * @brief ��ȡ��ǰ�����岹ģʽ
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ����3--3������  5--5������   6--��б��    7--G12.2ĥ��ָ��
 * @param intp_mode[out] : ���ص�ǰ�����岹ģʽ
 */
void MCCommunication::ReadChnCurIntpMode(uint8_t chn_index, uint16_t &intp_mode){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister_16(MC_CUR_INTP_MODE(chn_index), intp_mode);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}

/**
 * @brief ��ȡͨ���ĵ�ǰ����֡�����
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param frame_index[out] : ���ص�ǰ���е�����֡���
 */
void MCCommunication::ReadChnCurFrameIndex(uint8_t chn_index, uint16_t &frame_index){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister_16(MC_CUR_FRAME_INDEX(chn_index), frame_index);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}

/**
 * @brief ��ȡͨ��ģ̬��Ϣ
 * @param chn_index
 * @param mode_info
 */
void MCCommunication::ReadMcModeInfo(const uint8_t chn_index, uint32_t &mode_info){

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister(MC_MODE_INFO(chn_index), mode_info);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}


//
/**
 * @brief ��ȡ��ǰMC�����־
 * @param chn_index
 * @param mc_err
 */
void MCCommunication::ReadMcErrFlag(uint8_t chn_index, uint32_t &mc_err){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister(MC_ERR_FLAG(chn_index), mc_err);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}

//
/**
 * @brief ��ȡͨ���ĵ�ǰ����λ������mask
 * @param chn_index
 * @param axis_mask
 */
void MCCommunication::ReadChnSoftLimtMask(uint8_t chn_index, uint32_t &axis_mask){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister(MC_SOFT_LIMIT_MASK(chn_index), axis_mask);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}

//
/**
 * @brief ��ȡͨ���ĵ�ǰλ��ָ�����澯��mask
 * @param chn_index
 * @param axis_mask
 */
void MCCommunication::ReadChnPosErrMask(uint8_t chn_index, uint16_t &axis_mask){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister_16(MC_POS_ERR_MASK(chn_index), axis_mask);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}

//
/**
 * @brief ����ָ��ͨ���ᵽλ��־
 * @param chn_index[in] : ͨ����
 * @param axis_mask[out] �� ������岹��λmask
 */
void MCCommunication::ReadChnAxisRunoverMask(const uint8_t chn_index, uint32_t &axis_mask){
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister(MC_AXIS_INTPOVER_MASK(chn_index), axis_mask);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
}

/**
 * @brief ��ȡ�ֿ�岹��λ��־
 * @param chn_index : ͨ����, ��Χ[0,1]��ֻ֧����ͨ��
 * @return true--���е�λ     false--δ���е�λ
 */
bool MCCommunication::ReadAutoBlockRunOverFlag(const uint8_t chn_index){

	uint32_t value;
	bool flag = false;

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister(MC_RUN_OVER_FLAG, value);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0
//	printf("block over: 0x%x\n", value);

	if((value & 0xFFFF0000) & (0x00010000<<chn_index))
		flag = true;

	return flag;
}

/**
 * @brief ��ȡMDAģʽ�鵽λ��־
 * @param chn_index
 * @return true--���е�λ     false--δ���е�λ
 */
bool MCCommunication::ReadMdaBlockRunOverFlag(const uint8_t chn_index){

	bool flag = false;
	uint32_t value = 0;

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister(MC_BLOCK_RUN_OVER_MDA, value);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0

	if((value & 0x0000FFFF) & (0x0001<<chn_index))
		flag = true;

	return flag;
}


/**
 * @brief ��ȡ��λ��־�ֶ�
 * @param over_flag
 */
uint32_t MCCommunication::ReadRunOverValue(){
	uint32_t over_flag;
	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister(MC_RUN_OVER_FLAG, over_flag);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0

	return over_flag;
}

/**
 * @brief ��ȡ���β岹��λ��־
 * @param chn_index : ͨ����, ��Χ[0,1]��ֻ֧����ͨ��
 * @return true--���е�λ     false--δ���е�λ
 */
bool MCCommunication::ReadStepRunOverFlag(const uint8_t chn_index){

	uint32_t value;
	bool flag = false;

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	ReadRegister(MC_RUN_OVER_FLAG, value);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0


	if((value & 0x0000FFFF) & (0x0001<<chn_index))
		flag = true;

	return flag;
}

/**
 * @brief ����MC�������CRC
 * @param cmd : �����
 */
void MCCommunication::CalMcCmdCrc(McCmdFrame &cmd){
	uint16_t crc = 0xffff;
	int count = kMaxCmdCount*2-1;	//������CRC�ֶα���
	for(int i = 0; i < count; i++)
		crc ^= cmd.data_crc[i];

	cmd.data.crc = crc;
}

/**
 * @brief ����G��������֡��CRC
 * @param data
 */
void MCCommunication::CalMcGCodeFrameCrc(GCodeFrame &data){
	uint16_t crc = 0xFFFF;
	int count = kMaxGCodeDataCount*2;
	for(int i = 1; i < count; i++) //������CRC�ֶα���
		crc ^= data.data_crc[i];

	data.data.crc = crc;
//	printf("GCode CRC : 0x%X\n", data.data.crc);
}

/**
 * @brief ��ȡ����ͨ��FIFO�е�ǰ���ݸ���
 * @return ����ͨ��FIFO�е����ݸ���
 */
uint32_t MCCommunication::ReadCmdFifoCount(){
	uint32_t count = 0;

	//��ȡFIFO�������ж��ܷ�д��
	ReadRegister(MC_CMD_DOWN_FIFO_COUNT, count);

	return count;
}

/**
 * @brief ��ȡ��������ݸ���
 * @return
 */
uint32_t MCCommunication::ReadCmdBufferLen(){
	return m_list_cmd->GetLength();
}

/**
 * @brief ����������ͨ��д������֡����
 * @param data ����д�������֡
 * @return true--�ɹ�   false--ʧ��
 */
bool MCCommunication::WriteCmd(McCmdFrame &data, bool resend){

	bool res = true;
	//����CRCУ����
	CalMcCmdCrc(data);

	pthread_mutex_lock(&m_mutex_cmd);//����

	uint32_t count = 0;

	if(!resend && m_list_cmd->GetLength() > 0){//���ط������ط����зǿգ���ֱ�ӽ����ط����У���ֹ��������
		if(this->m_list_cmd->Append(data)){
			goto END; //���뻺���������
		}
		else{
			g_ptr_trace->PrintLog(LOG_ALARM, "MC���������ڴ�ʧ�ܣ�");
			res = false;   //FIFO��������д�뻺�����ʧ��
			goto END;
		}
	}

	//��ȡFIFO�������ж��ܷ�д��
	ReadRegister(MC_CMD_DOWN_FIFO_COUNT, count);
	while(count >= kMaxCmdFifoCount){
		if(resend){
			res = false;	//�ط�ʧ��
			goto END;
		}

		if(this->m_list_cmd->Append(data)){
			g_ptr_trace->PrintTrace(TRACE_WARNING, MC_COMMUNICATION_SC, "��MC��������[%hu]�������ط����У�", data.data.cmd);																													
			goto END; //���뻺���������
		}
		else{
			g_ptr_trace->PrintLog(LOG_ALARM, "MC���������ڴ�ʧ�ܣ�");
			res = false;   //FIFO��������д�뻺�����ʧ��
			goto END;
		}
	}

	WriteRegister(MC_CMD_DOWN_WRITE_OVER, 0);  //д��ǰ����

	//��������
	WriteRegister(MC_CMD_DOWN_DATA0, data.data_w[0]);
	WriteRegister(MC_CMD_DOWN_DATA1, data.data_w[1]);
	WriteRegister(MC_CMD_DOWN_DATA2, data.data_w[2]);
	WriteRegister(MC_CMD_DOWN_DATA3, data.data_w[3]);
	WriteRegister(MC_CMD_DOWN_DATA4, data.data_w[4]);


	WriteRegister(MC_CMD_DOWN_WRITE_OVER, 1);  //д�����һ

	END:
	pthread_mutex_unlock(&m_mutex_cmd);  //ж��
	return res;
}

/**
 * @brief ��ȡ��������ͨ��������֡
 * @param data
 * @return true--�ɹ�   false--ʧ��
 */
bool MCCommunication::ReadCmdRsp(McCmdFrame &data){
	//��ȡFIFO�������ж��ܷ��ȡ
	uint32_t count = 0;
	ReadRegister(MC_CMD_UP_FIFO_COUNT, count);
	if(count == 0)
		return false;   //FIFO���ݿգ��޷���ȡ

	WriteRegister(MC_CMD_UP_READ_REQ, 1);  //��ȡǰ��1

	//��ȡ����
	ReadRegister(MC_CMD_UP_DATA0, data.data_w[0]);
	ReadRegister(MC_CMD_UP_DATA1, data.data_w[1]);
	ReadRegister(MC_CMD_UP_DATA2, data.data_w[2]);
	ReadRegister(MC_CMD_UP_DATA3, data.data_w[3]);
	ReadRegister(MC_CMD_UP_DATA4, data.data_w[4]);

	WriteRegister(MC_CMD_UP_READ_REQ, 0);  //��ȡ����0


	return true;
}

/**
 * @brief  SC-MC������߳�
 * @param args
 */
void *MCCommunication::ProcessCmdThread(void *args){
	printf("Start MCCommunication::ProcessCmdThread!thread id = %ld\n", syscall(SYS_gettid));

	MCCommunication *mc_comm = static_cast<MCCommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
	if (res != ERR_NONE) {
		printf("Quit MCCommunication::ProcessCmdThread!thread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
	if (res != ERR_NONE) {
		printf("Quit MCCommunication::ProcessCmdThread!thread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	while((g_sys_state.module_ready_mask & SC_READY) == 0){  //�ȴ�SCģ���ʼ�����
		usleep(10000);
	}

	res = mc_comm->ProcessCmdFun();

	printf("Quit MCCommunication::ProcessCmdThread!!\n");
	pthread_exit(NULL);
}

/**
 * @brief �������
 * @return
 */
bool MCCommunication::ProcessCmdFun(){
	McCmdFrame cmd_frame;
//	McCmdFrame *send_cmd = nullptr;
	ListNode<McCmdFrame> *node = nullptr;


	uint64_t read_count = 0;	//for test
	while(!g_sys_state.system_quit){//�����˳�

		//��������͵�����
		while(!this->m_list_cmd->IsEmpty()){
			node = this->m_list_cmd->HeadNode();
			if(this->WriteCmd(static_cast<McCmdFrame &>(node->data), true)){
//				g_ptr_trace->PrintLog(LOG_SYSTEM_INFO, "�ط�MC������Ϣ�ɹ���");
				this->m_list_cmd->Delete(node);
				node = nullptr;
			}else{
//				g_ptr_trace->PrintLog(LOG_ALARM, "�ط�MC������Ϣʧ�ܣ�");
				break; //���Ƿ���ʧ�ܣ�����ѭ��
			}
		}

		if(this->m_list_cmd->GetLength() > 100)
			g_ptr_trace->PrintLog(LOG_ALARM, "���ط�MC������Ϣ����[%d]��", m_list_cmd->GetLength());


		if(!this->ReadCmdRsp(cmd_frame)){
			//��������˯��2ms
			usleep(2000);
//			this->m_p_channel_engine->ShakeHandWithMc();  //ѭ����������������в���
			continue;
		}

		read_count++;	// ��ȡ��Ӧ����

		//�������ݰ�
		//TODO У��CRC
		if(!CheckCrc(cmd_frame.data_crc, kMaxCmdCount*2-1, cmd_frame.data.crc)){
			//CRCУ��ʧ��
	//		CreateError(ERR_MC_CMD_CRC, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON);  //����MCͨѶCRCУ���澯
			m_n_crc_err_count++;
			printf("MC COMM CRC ERR: cmd = 0x%hx | %hhu | %hhu | 0x%hx;%hx;%hx;%hx;%hx;%hx;%hx | 0x%hx \n", cmd_frame.data.cmd,
					cmd_frame.data.axis_index, cmd_frame.data.channel_index, cmd_frame.data.data[0], cmd_frame.data.data[1], cmd_frame.data.data[2],
					cmd_frame.data.data[3], cmd_frame.data.data[4], cmd_frame.data.data[5], cmd_frame.data.data[6], cmd_frame.data.crc);

			printf("err info, recv:%lld, err:%lld percent:%lf\n", read_count, m_n_crc_err_count, (double)m_n_crc_err_count/read_count);
			if(cmd_frame.data.cmd != CMD_MC_SHAKEHANDS)//Ϊ�˲��Է�����������
				continue;
		}


		//ת��ChannelEngineִ��
		this->m_p_channel_engine->ProcessMcCmdRsp(cmd_frame);
	}


	return true;
}

//��ȡZynq-pl�İ汾
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

//��ȡsp6�İ汾
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
 * @brief ��ȡǷѹ�澯�ź�
 * @return true--Ƿѹ    false--����
 */
bool MCCommunication::ReadUnderVoltWarn(){
	uint32_t data = 0;
	ReadRegister(UNDER_VOLTAGE_ALARM, data);

	return (data&0x01)?true:false;
}


/**
 * @brief ��ȡMC�ĵ�������
 * @param debug_data[out] : ����������ݣ�16��uint32_t����
 */
void MCCommunication::ReadDebugData(uint32_t *debug_data){
	if(debug_data == nullptr)
		return;

	if(++m_n_read_status_sem == 1)
		WriteRegister(MC_STATUS_READ_REQ, 1);  //��ȡǰ��1

	for(int i = 0; i < 16; i++)
		ReadRegister(MC_DEBUG_DATA(i), debug_data[i]);

	if(--m_n_read_status_sem == 0)
		WriteRegister(MC_STATUS_READ_REQ, 0);  //��ȡ����0

}


