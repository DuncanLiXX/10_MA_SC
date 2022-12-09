/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file MCCommunication.h
 *@author gonghao
 *@date 2021/11/22
 *@brief ��ͷ�ļ�ΪSC-MC-ARMͨѶ���ʵ�֣�������SC��������ARM�ϵ�MCͨѶ
 *@version
 */

#include "mc_communication_arm.h"
#include "global_include.h"
#include "channel_engine.h"

const uint32_t kSharedRegMapSize = (uint32_t) 1024*1024;   	//ӳ������С 1M
const uint32_t kSharedRegMemMapMask = (kSharedRegMapSize - 1);	 //��ַ���� 0x0FFFFF



//�Ĵ�����д�궨�庯������߷���Ч��
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


//������32λ���ͺϲ�Ϊһ��64λ����
#define Merge32_64(l,h) ((int64_t)(((uint64_t)l) | (((uint64_t)h)<<32)))

MCArmCommunication* MCArmCommunication::m_p_instance = nullptr;  //��ʼ����������ָ��Ϊ��
//template<> int ListNode<McCmdFrame>::new_count = 0;


/**
 * @brief ���캯��
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
		CreateError(m_error_code, FATAL_LEVEL, CLEAR_BY_RESET_POWER);  //����MCͨѶģ���ʼ��ʧ�ܸ澯

//	printf("sizeof GCodeData: %d\n", sizeof(GCodeData));

}

/**
 * @brief ��������
 */
MCArmCommunication::~MCArmCommunication() {
	// TODO Auto-generated destructor stub
	this->Clean();
}

/**
 * @brief ��ȡ��ʵ����Ψһ���ʽӿں���������ģʽ
 */
MCArmCommunication* MCArmCommunication::GetInstance(){
	if(nullptr == m_p_instance){
		m_p_instance = new MCArmCommunication();
	}
	return m_p_instance;
}

/**
 * @brief ���ýӿ�ָ��
 * @param engine : ͨ���������ָ��
 */
void MCArmCommunication::SetInterface(){
	m_p_channel_engine = ChannelEngine::GetInstance();
}

/**
 * @brief ��ʼ���������򿪵�ַӳ������
 * @return true--�ɹ�   false--ʧ��
 */
bool MCArmCommunication::Initialize(){
	int res = ERR_NONE;
	pthread_attr_t attr;
	struct sched_param param;

	//��������ʼ��
	pthread_mutex_init(&m_mutex_cmd, nullptr);

	for(int i = 0; i < kMaxChnCount; i++)
		this->m_n_cur_gcode_index[i] = 0;
	this->m_n_cur_cmd_recv_index = 0;
	this->m_n_cur_cmd_send_index = 0;

	//���������
	this->m_list_cmd = new McArmCmdBuffer();
	if(m_list_cmd == nullptr){
		g_ptr_trace->PrintLog(LOG_ALARM, "MCArmCommunication::Initialize() ����ʼ�������������пռ�ʧ�ܣ�");
		m_error_code = ERR_MC_COM_INIT;
		return false;
	}

	//״̬��ȡ�ź�����ʼ��Ϊ0
	m_n_read_status_sem = 0;

	//���豸�ļ�
	m_n_mem_file = open("/dev/mem", O_RDWR | O_SYNC);
	if (m_n_mem_file == -1) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCArmCommunication::Initialize() failed to open file[/dev/mem]! errno = %d", errno);
		m_error_code = ERR_MC_COM_INIT;
		return false;
	}

	//ӳ���ڴ��ַ���õ�����ַ
	m_p_addr_base = (uint8_t *) mmap(nullptr, kSharedRegMapSize, PROT_READ | PROT_WRITE, MAP_SHARED,
			m_n_mem_file, MC_ARM_REGISTER_BASE & (~kSharedRegMemMapMask));
	if (m_p_addr_base == MAP_FAILED) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCArmCommunication::Initialize() failed to map memory! errno = %d", errno);
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
		g_ptr_trace->PrintLog(LOG_ALARM, "MCͨѶ�ӿ�2������߳������̼̳߳�ģʽʧ�ܣ�");
		m_error_code = ERR_MI_COM_INIT;
		goto END;
	}
	res = pthread_create(&m_thread_process_cmd, &attr,
			MCArmCommunication::ProcessCmdThread, this);    //����MC������߳�
	if (res != 0) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCͨѶ�ӿ�2������߳�����ʧ�ܣ�");
		res = ERR_MI_COM_INIT;
		goto END;
	}

	END:
	pthread_attr_destroy(&attr);

	printf("MCArmCommunication::Initialize(), mem file : %d, base_addr : %p\n", m_n_mem_file, m_p_addr_base);

	return true;
}

/**
 * @brief ������
 * @return true--�ɹ�   false--ʧ��
 */
bool MCArmCommunication::Clean(){

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
		g_ptr_trace->PrintLog(LOG_ALARM, "MCͨѶ�ӿ�2������߳��˳�ʧ�ܣ�");
	}

	usleep(1000);
	res = pthread_join(m_thread_process_cmd, &thread_result);//�ȴ�������߳��˳����
	if (res != ERR_NONE) {
		g_ptr_trace->PrintLog(LOG_ALARM, "MCͨѶ�ӿ�2�ȴ�������߳��˳�ʧ�ܣ�");
	}
	m_thread_process_cmd = 0;

	//ȡ���ڴ�ӳ��
	if(m_p_addr_base != MAP_FAILED)
		munmap(m_p_addr_base, kSharedRegMapSize);

	//�ر��豸�ļ�
	if(m_n_mem_file != -1) {
		close(m_n_mem_file);
	}

	pthread_mutex_destroy(&m_mutex_cmd);

	printf("MCArmCommunication::Clean()\n");
	return true;
}

/**
 * @brief �Ƿ���д���˶���������
 * @return  true -- ����    false -- ������
 */
bool MCArmCommunication::CanWriteGCode(uint8_t chn){
	//��ȡ��ǰ����֡����Ķ�д��־���ж��Ƿ���д��
	uint16_t flag = 0;
	ReadSharedRegister_16(MC_ARM_GCODE_WF(chn, m_n_cur_gcode_index[chn]), flag);
	if(flag == 1){  //��д�������
		flag = 0;
		ReadSharedRegister_16(MC_ARM_GCODE_RF(chn, m_n_cur_gcode_index[chn]), flag);
		if(flag != 1)   //���ݻ�δ����ȡ
			return false;   //FIFO���������޷�д��
	}
	return true;
}

/**
 * @brief д��G�����������˶���������
 * @param chn : ͨ�������ţ�0��ʼ
 * @param data ����д�������֡
 * @return true--�ɹ�   false--ʧ��,������
 */
bool MCArmCommunication::WriteGCodeData(uint8_t chn, GCodeFrame &data){
	CalMcGCodeFrameCrc(data);

//	printf("MCArmCommunication::WriteGCodeData1111\n");
	bool wf = false;	//��ǰ����֡�Ƿ���д�������
	//��ȡ��ǰ����֡����Ķ�д��־���ж��Ƿ���д��
	uint16_t flag = 0;
	ReadSharedRegister_16(MC_ARM_GCODE_WF(chn, m_n_cur_gcode_index[chn]), flag);
	if(flag == 1){  //��д�������
		flag = 0;
//		printf("MCArmCommunication::WriteGCodeData2222\n");
		ReadSharedRegister_16(MC_ARM_GCODE_RF(chn, m_n_cur_gcode_index[chn]), flag);
		if(flag != 1)   //���ݻ�δ����ȡ
			return false;   //FIFO���������޷�д��

		wf = true;
	}
//	printf("MCArmCommunication::WriteGCodeData3333\n");

	//��������
	for(int i = 0; i < kMaxGCodeDataCount; i++){
		WriteSharedRegister(MC_ARM_GCODE_DATA(chn, m_n_cur_gcode_index[chn], i), data.data_w[i]);
	}

	//��λд���־
	if(wf){
		WriteSharedRegister_16(MC_ARM_GCODE_RF(chn, m_n_cur_gcode_index[chn]), 0);	//��д���������ֻ��Ҫ��λ��ȡ��־
	}
	else{
		WriteSharedRegister_16(MC_ARM_GCODE_RF(chn, m_n_cur_gcode_index[chn]), 0);	
		WriteSharedRegister_16(MC_ARM_GCODE_WF(chn, m_n_cur_gcode_index[chn]), 1);//δд���������ֻ��Ҫ��λд���־
	}
//	uint16_t flag1 = 0;
//	ReadSharedRegister_16(MC_ARM_GCODE_WF(chn, m_n_cur_gcode_index[chn]), flag);
//	ReadSharedRegister_16(MC_ARM_GCODE_RF(chn, m_n_cur_gcode_index[chn]), flag1);
//	printf("read flag: %hu, %hu\n", flag, flag1);

	//д��ָ������
	if(++m_n_cur_gcode_index[chn] >= MC_ARM_CHN_GCODE_FIFO_DEPTH)
		m_n_cur_gcode_index[chn] = 0;

	return true;
}


/**
 * @brief ��ȡͨ������ģʽ������ָ��
 * @param chn_index[in] : ͨ��������
 * @param work_mode[out] : ��ǰ�Ĺ���ģʽ
 * @return ��
 */
void MCArmCommunication::ReadWorkMode(const uint8_t chn_index, uint16_t &work_mode){
	uint32_t data;
	ReadSharedRegister(MC_ARM_WORK_MODE(chn_index), data);

//	printf("@@@@@@mc data: 0x%08x\n", data);

	work_mode = data;
}

/**
 * @brief ��ȡͨ����ǰ����ָ��
 * @param chn_index[in] : ͨ��������
 * @param cur_cmd[out] : ��ǰ���е�ָ��
 * @return ��
 */
void MCArmCommunication::ReadCurCmd(const uint8_t chn_index, uint16_t &cur_cmd){

	uint32_t data;
	ReadSharedRegister(MC_ARM_CUR_CMD(chn_index), data);

	cur_cmd = data;
}

/**
 * @brief ��ȡ��岹λ�ã�������ǰλ�ã���������ϵ������Ŀ��λ��
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param cur_pos[out] : ��ǰ�岹λ��
 * @param tar_pos[out] ����ǰĿ��λ��
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
 * @brief ��ȡͨ����ǰ�Զ����ݻ�������
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param count[out] : ������������
 * @return
 */
void MCArmCommunication::ReadAutoBufDataCount(const uint8_t chn_index, uint16_t &count){

	uint32_t tmp_count = 0;


	ReadSharedRegister(MC_ARM_AUTO_BUF_DATA_COUNT(chn_index), tmp_count);

	count = tmp_count;

}

//
/**
 * @brief ��ȡͨ����ǰMDA���ݻ�������
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param count[out] : ������������
 */
void MCArmCommunication::ReadMdaBufDataCount(const uint8_t chn_index, uint16_t &count){

	uint32_t tmp_count = 0;
	ReadSharedRegister(MC_ARM_MDA_BUF_DATA_COUNT(chn_index), tmp_count);
	count = tmp_count;
}

/**
 * @brief ��ȡͨ����ǰ�����ٶ�
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param cur_feed[out] : ���ص�ǰ�����ٶ�
 * @return
 */
void MCArmCommunication::ReadCurFeed(const uint8_t chn_index, int32_t &cur_feed){

	ReadSharedRegister(MC_ARM_CUR_FEED(chn_index), cur_feed);

}

/**
 * @brief ��ȡͨ����ǰ���������ٶ�
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param cur_feed[out] : ���ص�ǰ���������ٶ�
 * @return
 */
void MCArmCommunication::ReadRatedFeed(const uint8_t chn_index, int32_t &rated_feed){

	ReadSharedRegister(MC_ARM_RATED_FEED(chn_index), rated_feed);


}

/**
 * @brief ��ȡͨ����ǰ�����к�
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param cur_lineno[out] : ���ص�ǰ�к�
 * @return
 */
void MCArmCommunication::ReadCurLineNo(const uint8_t chn_index, uint32_t &cur_lineno){

	ReadSharedRegister(MC_ARM_CUR_LINE_NO(chn_index), cur_lineno);


}


/**
 * @brief ��ȡͨ����ǰ����ϵ
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param coord[out] : ���ص�ǰ����ϵ���
 */
void MCArmCommunication::ReadChnCurCoord(uint8_t chn_index, uint16_t &coord){

	uint32_t data = 0;
	ReadSharedRegister(MC_ARM_CUR_COORD(chn_index), data);
	coord = data;
}


/**
 * @brief ��ȡͨ����ǰ��ƫ��
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param tool_offset[out] : ���ص�ǰ����ƫ��
 */
void MCArmCommunication::ReadChnCurToolOffset(uint8_t chn_index, uint16_t &tool_offset){

	uint32_t data = 0;
	ReadSharedRegister(MC_ARM_CUR_TOOL_OFFSET(chn_index), data);
	tool_offset = data;

}


/**
 * @brief ��ȡ��ǰ�����岹ģʽ
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ����3--3������  5--5������   6--��б��    7--G12.2ĥ��ָ��
 * @param intp_mode[out] : ���ص�ǰ�����岹ģʽ
 */
void MCArmCommunication::ReadChnCurIntpMode(uint8_t chn_index, uint16_t &intp_mode){

	uint32_t data = 0;
	ReadSharedRegister(MC_ARM_CUR_INTP_MODE(chn_index), data);
	intp_mode = data;

}

/**
 * @brief ��ȡͨ���ĵ�ǰ����֡�����
 * @param chn_index[in] : ͨ��������, ��Χ[0,1]��ֻ֧����ͨ��
 * @param frame_index[out] : ���ص�ǰ���е�����֡���
 */
void MCArmCommunication::ReadChnCurFrameIndex(uint8_t chn_index, uint16_t &frame_index){

	uint32_t data = 0;
	ReadSharedRegister(MC_ARM_CUR_FRAME_INDEX(chn_index), data);
	frame_index = data;

}

/**
 * @brief ��ȡͨ��ģ̬��Ϣ
 * @param chn_index
 * @param mode_info
 */
void MCArmCommunication::ReadMcModeInfo(const uint8_t chn_index, uint32_t &mode_info){

	ReadSharedRegister(MC_ARM_MODE_INFO(chn_index), mode_info);

}


//
/**
 * @brief ��ȡ��ǰMC�����־
 * @param chn_index
 * @param mc_err
 */
void MCArmCommunication::ReadMcErrFlag(uint8_t chn_index, uint32_t &mc_err){


	ReadSharedRegister(MC_ARM_ERR_FLAG(chn_index), mc_err);


}


/**
 * @brief ��ȡͨ���ĵ�ǰ��������λ������mask
 * @param chn_index
 * @param axis_mask
 */
void MCArmCommunication::ReadChnPosSoftLimtMask(uint8_t chn_index, uint32_t &axis_mask){

	ReadSharedRegister(MC_ARM_POS_SOFT_LIMIT_MASK(chn_index), axis_mask);

}

/**
 * @brief ��ȡͨ���ĵ�ǰ��������λ������mask
 * @param chn_index
 * @param axis_mask
 */
void MCArmCommunication::ReadChnNegSoftLimtMask(uint8_t chn_index, uint32_t &axis_mask){

	ReadSharedRegister(MC_ARM_NEG_SOFT_LIMIT_MASK(chn_index), axis_mask);

}

/**
 * @brief ��ȡͨ���ĵ�ǰλ��ָ�����澯��mask
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
 * @brief ����ָ��ͨ���ᵽλ��־
 * @param chn_index[in] : ͨ����
 * @param axis_mask[out] �� ������岹��λmask
 */
void MCArmCommunication::ReadChnAxisRunoverMask(const uint8_t chn_index, uint32_t &axis_mask){


	ReadSharedRegister(MC_ARM_AXIS_INTPOVER_MASK(chn_index), axis_mask);


}

/**
 * @brief ��ȡ�ֿ�岹��λ��־
 * @param chn_index : ͨ����, ��Χ[0,1]��ֻ֧����ͨ��
 * @return true--���е�λ     false--δ���е�λ
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
 * @brief ��ȡMDAģʽ�鵽λ��־
 * @param chn_index
 * @return true--���е�λ     false--δ���е�λ
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
 * @brief ��ȡ���β岹��λ��־
 * @param chn_index : ͨ����, ��Χ[0,1]��ֻ֧����ͨ��
 * @return true--���е�λ     false--δ���е�λ
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
 * @brief ����MC�������CRC
 * @param cmd : �����
 */
void MCArmCommunication::CalMcCmdCrc(McCmdFrame &cmd){
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
void MCArmCommunication::CalMcGCodeFrameCrc(GCodeFrame &data){
	uint16_t crc = 0xFFFF;
	int count = kMaxGCodeDataCount*2;
	for(int i = 1; i < count; i++) //������CRC�ֶα���
		crc ^= data.data_crc[i];

	data.data.crc = crc;
//	printf("GCode CRC : 0x%X\n", data.data.crc);
}


/**
 * @brief ��ȡ��������ݸ���
 * @return
 */
uint32_t MCArmCommunication::ReadCmdBufferLen(){
	return m_list_cmd->GetLength();
}

/**
 * @brief ����������ͨ��д������֡����
 * @param data ����д�������֡
 * @return true--�ɹ�   false--ʧ��
 */
bool MCArmCommunication::WriteCmd(McCmdFrame &data, bool resend){

	bool res = true;
	//����CRCУ����
	CalMcCmdCrc(data);

	pthread_mutex_lock(&m_mutex_cmd);//����

	bool wf = false;	//��ǰ����֡�Ƿ���д�������
	uint16_t flag = 0;

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

	//��ȡ��ǰ����֡����Ķ�д��־���ж��Ƿ���д��
	ReadSharedRegister_16(MC_ARM_CMD_DOWN_WF(m_n_cur_cmd_send_index), flag);
	if(flag == 1){  //��д�������
		flag = 0;
		ReadSharedRegister_16(MC_ARM_CMD_DOWN_RF(m_n_cur_cmd_send_index), flag);
		if(flag != 1){   //���ݻ�δ����ȡ
			if(resend){
				res = false;	//�ط�ʧ��
				goto END;
			}

			if(this->m_list_cmd->Append(data)){
				goto END; //���뻺���������
			}
			else{
				g_ptr_trace->PrintLog(LOG_ALARM, "MC���������ڴ�ʧ�ܣ�");
				res = false;   //FIFO��������д�뻺�����ʧ��
				goto END;
			}
		}

		wf = true;
	}


	//��������
	WriteSharedRegister(MC_ARM_CMD_DOWN_DATA0(m_n_cur_cmd_send_index), data.data_w[0]);
	WriteSharedRegister(MC_ARM_CMD_DOWN_DATA1(m_n_cur_cmd_send_index), data.data_w[1]);
	WriteSharedRegister(MC_ARM_CMD_DOWN_DATA2(m_n_cur_cmd_send_index), data.data_w[2]);
	WriteSharedRegister(MC_ARM_CMD_DOWN_DATA3(m_n_cur_cmd_send_index), data.data_w[3]);
	WriteSharedRegister(MC_ARM_CMD_DOWN_DATA4(m_n_cur_cmd_send_index), data.data_w[4]);


	//��λд���־
	if(wf){
		WriteSharedRegister_16(MC_ARM_CMD_DOWN_RF(m_n_cur_cmd_send_index), 0);	//��д���������ֻ��Ҫ��λ��ȡ��־
	}
	else{
		WriteSharedRegister_16(MC_ARM_CMD_DOWN_WF(m_n_cur_cmd_send_index), 1);	//δд���������ֻ��Ҫ��λд���־
	}

	//д��ָ������
	if(++m_n_cur_cmd_send_index >= MC_ARM_CMD_FIFO_DEPTH)
		m_n_cur_cmd_send_index = 0;


	END:
	pthread_mutex_unlock(&m_mutex_cmd);  //ж��
	return res;
}

/**
 * @brief ��ȡ��������ͨ��������֡
 * @param data
 * @return true--�ɹ�   false--ʧ��
 */
bool MCArmCommunication::ReadCmdRsp(McCmdFrame &data){
	//��ȡ��ǰ����֡����Ķ�д��־���ж��Ƿ��ܶ�ȡ
	uint16_t flag = 0;
	ReadSharedRegister_16(MC_ARM_CMD_UP_WF(m_n_cur_cmd_recv_index), flag);
	if(flag == 0)
		return false;   //��ǰ֡���ݿգ��޷���ȡ

	flag = 0;
	ReadSharedRegister_16(MC_ARM_CMD_UP_RF(m_n_cur_cmd_recv_index), flag);
	if(flag == 1){
		return false;   //��ǰ�����Ѷ�ȡ������Ҫ�ظ���
	}


	//��ȡ����
	ReadSharedRegister(MC_ARM_CMD_UP_DATA0(m_n_cur_cmd_recv_index), data.data_w[0]);
	ReadSharedRegister(MC_ARM_CMD_UP_DATA1(m_n_cur_cmd_recv_index), data.data_w[1]);
	ReadSharedRegister(MC_ARM_CMD_UP_DATA2(m_n_cur_cmd_recv_index), data.data_w[2]);
	ReadSharedRegister(MC_ARM_CMD_UP_DATA3(m_n_cur_cmd_recv_index), data.data_w[3]);
	ReadSharedRegister(MC_ARM_CMD_UP_DATA4(m_n_cur_cmd_recv_index), data.data_w[4]);


	//��λ����־
	WriteSharedRegister_16(MC_ARM_CMD_UP_RF(m_n_cur_cmd_recv_index), 1);

	//��ȡָ������
	if(++m_n_cur_cmd_recv_index >= MC_ARM_CMD_FIFO_DEPTH)
		m_n_cur_cmd_recv_index = 0;

	return true;
}

/**
 * @brief  SC-MC������߳�
 * @param args
 */
void *MCArmCommunication::ProcessCmdThread(void *args){
	printf("Start MCArmCommunication::ProcessCmdThread!thread id = %ld\n", syscall(SYS_gettid));

	MCArmCommunication *mc_comm = static_cast<MCArmCommunication *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
	if (res != ERR_NONE) {
		printf("Quit MCArmCommunication::ProcessCmdThread!thread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
	if (res != ERR_NONE) {
		printf("Quit MCArmCommunication::ProcessCmdThread!thread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	while((g_sys_state.module_ready_mask & SC_READY) == 0){  //�ȴ�SCģ���ʼ�����
		usleep(10000);
	}

	res = mc_comm->ProcessCmdFun();

	printf("Quit MCArmCommunication::ProcessCmdThread!!\n");
	pthread_exit(NULL);
}

/**
 * @brief �������
 * @return
 */
bool MCArmCommunication::ProcessCmdFun(){
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
			g_ptr_trace->PrintLog(LOG_ALARM, "���ط�MC-2������Ϣ����[%d]��", m_list_cmd->GetLength());


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
			printf("MC-2 COMM CRC ERR: cmd = 0x%hx | %hhu | %hhu | 0x%hx;%hx;%hx;%hx;%hx;%hx;%hx | 0x%hx \n", cmd_frame.data.cmd,
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


/**
 * @brief ��ȡMC�ĵ�������
 * @param debug_data[out] : ����������ݣ�16��uint32_t����
 */
void MCArmCommunication::ReadDebugData(uint32_t *debug_data){
	if(debug_data == nullptr)
		return;

//	for(int i = 0; i < 16; i++)
//		ReadSharedRegister(MC_DEBUG_DATA(i), debug_data[i]);



}


