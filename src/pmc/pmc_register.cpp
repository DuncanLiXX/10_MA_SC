/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file pmc_register.cpp
 *@author gonghao
 *@date 2020/10/29
 *@brief ��ͷ�ļ�����PMC�мĴ����������ʵ��
 *@version
 */

#include <pmc_register.h>
#include "global_include.h"
#include "mi_communication.h"

/**
 * @brief ���캯��
 */
PmcRegister::PmcRegister() {
	// TODO Auto-generated constructor stub
	this->Initialize();
}


/**
 * @brief ��������
 */
PmcRegister::~PmcRegister() {
	// TODO Auto-generated destructor stub

	if(m_n_fp != -1){
		fsync(m_n_fp);
		m_n_fp = -1;
	}
}

void PmcRegister::Initialize(){

	this->m_n_fp = -1;
//	m_mask_save_keep = 0x00;
	this->m_p_mi_comm = MICommunication::GetInstance();

	memset(m_x_reg, 0x00, sizeof(m_x_reg));
	memset(m_y_reg, 0x00, sizeof(m_y_reg));
	memset(m_f_reg.all, 0x00, sizeof(m_f_reg.all));
	memset(m_g_reg.all, 0x00, sizeof(m_g_reg.all));
	memset(m_r_reg, 0x00, sizeof(m_r_reg));
	memset(m_k_reg, 0x00, sizeof(m_k_reg));
	memset(m_a_reg, 0x00, sizeof(m_a_reg));

#ifndef USES_PMC_2_0
	memset(m_d_reg, 0x00, D_REG_COUNT*2);
	memset(m_c_reg, 0x00, C_REG_COUNT*2);
	memset(m_t_reg, 0x00, T_REG_COUNT*2);
	memset(m_dc_reg, 0x00, C_REG_COUNT*2);
	memset(m_dt_reg, 0x00, T_REG_COUNT*2);
#else
	memset(m_d_reg, 0x00, D_REG_COUNT);
	memset(m_c_reg, 0x00, C_REG_COUNT*4);
	memset(m_t_reg, 0x00, T_REG_COUNT*2);
	memset(m_e_reg, 0x00, E_REG_COUNT);
#endif


	if(access(PATH_PMC_REG, F_OK) == -1){  //PMC�Ĵ����ļ�������
		printf("init to create pmc register file!!!\n");

		this->InitRegFile();

//		m_n_fp = open(PATH_PMC_REG, O_RDWR);
//		if(m_n_fp < 0){
//			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��pmc�Ĵ����ļ�ʧ�ܣ�");
//			return;//�ļ���ʧ��
//		}
	}else{//�ļ�����
		bool init_reg = false;   //�Ƿ����³�ʼ���ļ�����������мĴ�����ƥ�������³�ʼ��
		m_n_fp = open(PATH_PMC_REG, O_RDWR);
		if(m_n_fp < 0){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��pmc�Ĵ����ļ�ʧ�ܣ�");
			return;//�ļ���ʧ��
		}

		uint16_t size = 0;   //�Ĵ����ֽ���
		uint16_t size_real = 0;  //ʵ�ʶ�ȡ�ֽ���

		//��ȡK�Ĵ���
		ssize_t read_size = read(m_n_fp, &size, 2);
		if(read_size != 2){ //��ȡʧ��
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡK�Ĵ����ֽ���ʧ�ܣ�");
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}

		size_real = size;
		if(size != K_REG_COUNT){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "K�Ĵ�����С����ƥ��[%hu,%hu]��", size, K_REG_COUNT);
//			close(m_n_fp);
//			m_n_fp = -1;
//			return;
			init_reg = true;
			if(size > K_REG_COUNT)
				size_real = K_REG_COUNT;
		}

		read_size = read(m_n_fp, this->m_k_reg, size_real);
		if(read_size != size_real){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡK�Ĵ�������ʧ��[%hu,%hu]��", read_size, size_real);
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}
		if(size_real < size){//�������������
			if(-1 == lseek(m_n_fp, size-size_real, SEEK_CUR)){
				g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תK�Ĵ��������ַʧ�ܣ�");
				close(m_n_fp);
				m_n_fp = -1;
				return;
			}
		}


		//��ȡD�Ĵ���
		read_size = read(m_n_fp, &size, 2);
		if(read_size != 2){ //��ȡʧ��
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡD�Ĵ����ֽ���ʧ�ܣ�");
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}
#ifndef USES_PMC_2_0
		size_real = size;
		if(size != D_REG_COUNT*2){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "D�Ĵ�����С����ƥ��[%hu,%hu]��", size, D_REG_COUNT*2);
//			close(m_n_fp);
//			m_n_fp = -1;
//			return;
			init_reg = true;
			if(size > D_REG_COUNT*2)
				size_real = D_REG_COUNT*2;
		}
		read_size = read(m_n_fp, this->m_d_reg, size_real);
		if(read_size != size_real){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡD�Ĵ�������ʧ��[%hu,%hu]��", read_size, size_real);
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}
		if(size_real < size){  //�������������
			if(-1 == lseek(m_n_fp, size-size_real, SEEK_CUR)){
				g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תD�Ĵ��������ַʧ�ܣ�");
				close(m_n_fp);
				m_n_fp = -1;
				return;
			}
		}
		if(init_reg)
			goto INIT_LABEL;

		//��ȡDC�Ĵ���
		read_size = read(m_n_fp, &size, 2);
		if(read_size != 2){ //��ȡʧ��
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡDC�Ĵ����ֽ���ʧ�ܣ�");
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}

		size_real = size;
		if(size != C_REG_COUNT*2){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "DC�Ĵ�����С����ƥ��[%hu,%hu]��", size, C_REG_COUNT*2);
//			close(m_n_fp);
//			m_n_fp = -1;
//			return;
			init_reg = true;
			if(size > C_REG_COUNT*2)
				size_real = C_REG_COUNT*2;
		}
		read_size = read(m_n_fp, this->m_dc_reg, size_real);
		if(read_size != size_real){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡDC�Ĵ�������ʧ��[%hu,%hu]��", read_size, size_real);
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}
		if(size_real < size){  //�������������
			if(-1 == lseek(m_n_fp, size-size_real, SEEK_CUR)){
				g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תDC�Ĵ��������ַʧ�ܣ�");
				close(m_n_fp);
				m_n_fp = -1;
				return;
			}
		}

		//��ȡDT�Ĵ���
		read_size = read(m_n_fp, &size, 2);
		if(read_size != 2){ //��ȡʧ��
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡDT�Ĵ����ֽ���ʧ�ܣ�");
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}

		size_real = size;
		if(size != T_REG_COUNT*2){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "DT�Ĵ�����С����ƥ��[%hu,%hu]��", size, T_REG_COUNT*2);
//			close(m_n_fp);
//			m_n_fp = -1;
//			return;
			init_reg = true;
			if(size > T_REG_COUNT*2)
				size_real = T_REG_COUNT*2;
		}
		read_size = read(m_n_fp, this->m_dt_reg, size_real);
		if(read_size != size_real){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡDT�Ĵ�������ʧ��[%hu,%hu]��", read_size, size_real);
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}
		if(size_real < size){  //�������������
			if(-1 == lseek(m_n_fp, size-size_real, SEEK_CUR)){
				g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תDT�Ĵ��������ַʧ�ܣ�");
				close(m_n_fp);
				m_n_fp = -1;
				return;
			}
		}

		//��ȡC�Ĵ���
		read_size = read(m_n_fp, &size, 2);
		if(read_size == 0){ //��ǰ���ݣ� ��ʼд��Cֵ
			size = C_REG_COUNT*2;
			lseek(m_n_fp, 0, SEEK_END);
			ssize_t res = write(m_n_fp, &size, 2);
			res = write(m_n_fp, this->m_c_reg, size);
			if(res == -1){//д��ʧ��
				close(m_n_fp);
				g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ʼ��д��C�Ĵ���ʧ�ܣ�");
				return;
			}
			fsync(m_n_fp);
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}else if(read_size != 2){//��ȡʧ��
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡC�Ĵ����ֽ���ʧ�ܣ�");
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}

		size_real = size;
		if(size != C_REG_COUNT*2){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "C�Ĵ�����С����ƥ��[%hu,%hu]��", size, C_REG_COUNT*2);
//			close(m_n_fp);
//			m_n_fp = -1;
//			return;
			init_reg = true;
			if(size > C_REG_COUNT*2)
				size_real = C_REG_COUNT*2;

		}
		read_size = read(m_n_fp, this->m_c_reg, size_real);
		if(read_size != size_real){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡC�Ĵ�������ʧ��[%hu,%hu]��", read_size, size_real);
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}
		if(size_real < size){  //�������������
			if(-1 == lseek(m_n_fp, size-size_real, SEEK_CUR)){
				g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תC�Ĵ��������ַʧ�ܣ�");
				close(m_n_fp);
				m_n_fp = -1;
				return;
			}
		}
#else
		size_real = size;
		if(size != D_REG_COUNT){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "D�Ĵ�����С����ƥ��[%hu,%hu]��", size, D_REG_COUNT);
//			close(m_n_fp);
//			m_n_fp = -1;
//			return;
			init_reg = true;
			if(size > D_REG_COUNT)
				size_real = D_REG_COUNT;
		}
		read_size = read(m_n_fp, this->m_d_reg, size_real);
		if(read_size != size_real){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡD�Ĵ�������ʧ��[%hu,%hu]��", read_size, size_real);
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}
		if(size_real < size){  //�������������
			if(-1 == lseek(m_n_fp, size-size_real, SEEK_CUR)){
				g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תD�Ĵ��������ַʧ�ܣ�");
				close(m_n_fp);
				m_n_fp = -1;
				return;
			}
		}
		if(init_reg)
			goto INIT_LABEL;


		//��ȡC�Ĵ���
		read_size = read(m_n_fp, &size, 2);
		if(read_size != 2){//��ȡʧ��
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡC�Ĵ����ֽ���ʧ�ܣ�");
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}

		size_real = size;
		if(size != C_REG_COUNT*4){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "C�Ĵ�����С����ƥ��[%hu,%hu]��", size, C_REG_COUNT*4);
//			close(m_n_fp);
//			m_n_fp = -1;
//			return;
			init_reg = true;
			if(size > C_REG_COUNT*4)
				size_real = C_REG_COUNT*4;
		}
		read_size = read(m_n_fp, this->m_c_reg, size_real);
		if(read_size != size_real){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡC�Ĵ�������ʧ��[%hu,%hu]��", read_size, size_real);
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}
		if(size_real < size){  //�������������
			if(-1 == lseek(m_n_fp, size-size_real, SEEK_CUR)){
				g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תC�Ĵ��������ַʧ�ܣ�");
				close(m_n_fp);
				m_n_fp = -1;
				return;
			}
		}

		//��ȡT�Ĵ���
		read_size = read(m_n_fp, &size, 2);
		if(read_size != 2){//��ȡʧ��
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡT�Ĵ����ֽ���ʧ�ܣ�");
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}

		size_real = size;
		if(size != T_REG_COUNT*2){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "T�Ĵ�����С����ƥ��[%hu,%hu]��", size, T_REG_COUNT*2);
//			close(m_n_fp);
//			m_n_fp = -1;
//			return;
			init_reg = true;
			if(size > T_REG_COUNT*2)
				size_real = T_REG_COUNT*2;
		}
		read_size = read(m_n_fp, this->m_t_reg, size_real);
		if(read_size != size_real){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡT�Ĵ�������ʧ��[%hu,%hu]��", read_size, size_real);
			close(m_n_fp);
			m_n_fp = -1;
			return;
		}
		if(size_real < size){  //�������������
			if(-1 == lseek(m_n_fp, size-size_real, SEEK_CUR)){
				g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תT�Ĵ��������ַʧ�ܣ�");
				close(m_n_fp);
				m_n_fp = -1;
				return;
			}
		}
#endif
		INIT_LABEL:
		if(init_reg){
			close(m_n_fp);
			m_n_fp = -1;
			this->InitRegFile();
		}
	}
	if(m_n_fp > 0){
		close(m_n_fp);
		m_n_fp = -1;
	}
}

/**
 * @brief ��ʼ���Ĵ��������ļ�
 */
void PmcRegister::InitRegFile(){
	m_n_fp = open(PATH_PMC_REG, O_CREAT|O_TRUNC|O_WRONLY); //���ļ�

	if(m_n_fp < 0){
		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��pmc�Ĵ��������ļ�ʧ�ܣ�");
		return;//�ļ���ʧ��
	}

	//����K����
	uint16_t size = K_REG_COUNT;
	ssize_t res = write(m_n_fp, &size, 2);		//д��Ĵ����ֽ���
	res = write(m_n_fp, this->m_k_reg, size);
	if(res == -1){//д��ʧ��
		close(m_n_fp);
		m_n_fp = -1;
		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ʼ��д��K�Ĵ���ʧ�ܣ�");
		return;
	}

#ifndef USES_PMC_2_0

	//����D����
	size = D_REG_COUNT*2;
	res = write(m_n_fp, &size, 2);		//д��Ĵ����ֽ���
	res = write(m_n_fp, this->m_d_reg, size);
	if(res == -1){//д��ʧ��
		close(m_n_fp);
		m_n_fp = -1;
		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ʼ��д��D�Ĵ���ʧ�ܣ�");
		return;
	}

	//����DC����
	size = C_REG_COUNT*2;
	res = write(m_n_fp, &size, 2);		//д��Ĵ����ֽ���
	res = write(m_n_fp, this->m_dc_reg, size);
	if(res == -1){//д��ʧ��
		close(m_n_fp);
		m_n_fp = -1;
		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ʼ��д��DC�Ĵ���ʧ�ܣ�");
		return;
	}

	//����DT����
	size = T_REG_COUNT*2;
	res = write(m_n_fp, &size, 2);		//д��Ĵ����ֽ���
	res = write(m_n_fp, this->m_dt_reg, size);
	if(res == -1){//д��ʧ��
		close(m_n_fp);
		m_n_fp = -1;
		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ʼ��д��DT�Ĵ���ʧ�ܣ�");
		return;
	}

	//����C����
	size = C_REG_COUNT*2;
	res = write(m_n_fp, &size, 2);		//д��Ĵ����ֽ���
	res = write(m_n_fp, this->m_c_reg, size);
	if(res == -1){//д��ʧ��
		close(m_n_fp);
		m_n_fp = -1;
		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ʼ��д��C�Ĵ���ʧ�ܣ�");
		return;
	}
#else
	//����D����
	size = D_REG_COUNT;
	res = write(m_n_fp, &size, 2);		//д��Ĵ����ֽ���
	res = write(m_n_fp, this->m_d_reg, size);
	if(res == -1){//д��ʧ��
		close(m_n_fp);
		m_n_fp = -1;
		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ʼ��д��D�Ĵ���ʧ�ܣ�");
		return;
	}

	//����C����
	size = C_REG_COUNT*4;
	res = write(m_n_fp, &size, 2);		//д��Ĵ����ֽ���
	res = write(m_n_fp, this->m_c_reg, size);
	if(res == -1){//д��ʧ��
		close(m_n_fp);
		m_n_fp = -1;
		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ʼ��д��C�Ĵ���ʧ�ܣ�");
		return;
	}

	//����T����
	size = T_REG_COUNT*2;
	res = write(m_n_fp, &size, 2);		//д��Ĵ����ֽ���
	res = write(m_n_fp, this->m_t_reg, size);
	if(res == -1){//д��ʧ��
		close(m_n_fp);
		m_n_fp = -1;
		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ʼ��д��T�Ĵ���ʧ�ܣ�");
		return;
	}
#endif
	fsync(m_n_fp);
	close(m_n_fp);
	m_n_fp = -1;
}

/**
 * @brief ���ؼĴ�����ָ��,���ֽڼĴ���
 * @param sec
 * @return
 */
uint8_t *PmcRegister::GetRegPtr8(PmcRegSection sec){
	uint8_t *p = nullptr;
	switch(sec){
	case PMC_REG_X:
		p = this->m_x_reg;
		break;
	case PMC_REG_Y:
		p = this->m_y_reg;
		break;
	case PMC_REG_F:
		p = this->m_f_reg.all;
		break;
	case PMC_REG_G:
		p = this->m_g_reg.all;
		break;
	case PMC_REG_R:
		p = this->m_r_reg;
		break;
	case PMC_REG_K:
		p = this->m_k_reg;
		break;
	case PMC_REG_A:
		p = this->m_a_reg;
		break;
#ifdef USES_PMC_2_0
	case PMC_REG_D:
		p = this->m_d_reg;
		break;
	case PMC_REG_C:
		p = this->m_c_reg;
		break;
	case PMC_REG_T:
		p = this->m_t_reg;
		break;
	case PMC_REG_E:
		p = this->m_e_reg;
		break;
#endif
	default:
		g_ptr_trace->PrintTrace(TRACE_WARNING, PMC_REGISTER, "�Ĵ�����[%d]���ǵ��ֽڼĴ�����", sec);
		break;
	}

	return p;
}

/**
 * @brief ���ؼĴ�����ָ�룬˫�ֽڼĴ���
 * @param sec
 * @return
 */
#ifndef USES_PMC_2_0
uint16_t *PmcRegister::GetRegPtr16(PmcRegSection sec){
	uint16_t *p = nullptr;
	switch(sec){
	case PMC_REG_C:
		p = this->m_c_reg;
		break;
	case PMC_REG_T:
		p = this->m_t_reg;
		break;

	case PMC_REG_D:
		p = this->m_d_reg;
		break;
	case PMC_REG_DC:
		p = this->m_dc_reg;
		break;
	case PMC_REG_DT:
		p = this->m_dt_reg;
		break;

	default:
		g_ptr_trace->PrintTrace(TRACE_WARNING, PMC_REGISTER, "�Ĵ�����[%d]����˫�ֽڼĴ�����", sec);
		break;
	}

	return p;
}
#endif

/**
 * @brief ���üĴ���ֵ
 * @param sec
 * @param index
 * @param value
 * @return
 */
bool PmcRegister::SetRegValue(PmcRegSection sec, uint16_t index, uint8_t value){
	switch(sec){
	case PMC_REG_X:
#ifdef USES_PMC_2_0
		if(index < 128)   //IO-LINK ͨ��1
			this->m_x_reg[index] = value;
		else if(index >= 200 && index < 328)    //IO-LINK ͨ��2
			this->m_x_reg[index-200+128] = value;
		else if(index >= 1000 && index < 1007)
			this->m_x_reg[index-1000+256] = value;
		else if(index == 2000)   //����DI
			this->m_x_reg[264] = value;
		else
			return false;
#else
		if(index >= X_REG_COUNT)
			return false;
		this->m_x_reg[index] = value;
#endif
		break;
	case PMC_REG_Y:
#ifdef USES_PMC_2_0
		if(index < 128)   //IO-LINK ͨ��1
			this->m_y_reg[index] = value;
		else if(index >= 200 && index < 328)    //IO-LINK ͨ��2
			this->m_y_reg[index-200+128] = value;
		else if(index >= 1000 && index < 1007)
			this->m_y_reg[index-1000+256] = value;
		else
			return false;
#else
		if(index >= Y_REG_COUNT)
			return false;
		this->m_y_reg[index] = value;
#endif
		break;
	case PMC_REG_F:
		if(index >= F_REG_COUNT)
			return false;
		this->m_f_reg.all[index] = value;
		break;
	case PMC_REG_G:
		if(index >= G_REG_COUNT)
			return false;
		// Ϊ�� cnclib �޸�G�Ĵ����������л�ͨ��  �޸�MI ��ͨ��ˢ��ͬ����SC
//		this->m_g_reg.all[index] = value;
		break;
	case PMC_REG_R:
		if(index >= R_REG_COUNT)
			return false;
		this->m_r_reg[index] = value;
		break;
	case PMC_REG_K:
		if(index >= K_REG_COUNT)
			return false;
		this->m_k_reg[index] = value;
//		this->m_mask_save_keep |= (0x01<<K_REG_BIT);
		break;
	case PMC_REG_A:
		if(index >= A_REG_COUNT)
			return false;
		this->m_a_reg[index] = value;
		break;
#ifdef USES_PMC_2_0
	case PMC_REG_D:
		if(index >= D_REG_COUNT)
			return false;
		this->m_d_reg[index] = value;
//		this->m_mask_save_keep |= (0x01<<D_REG_BIT);
		break;
	case PMC_REG_C:
		if(index >= C_REG_COUNT*4)
			return false;
		this->m_c_reg[index] = value;
		break;
	case PMC_REG_T:
		if(index >= T_REG_COUNT*2)
			return false;
		this->m_t_reg[index] = value;
		break;
	case PMC_REG_E:
		if(index >= E_REG_COUNT)
			return false;
		this->m_e_reg[index] = value;
		break;
#endif
	default:
		g_ptr_trace->PrintTrace(TRACE_WARNING, PMC_REGISTER, "�Ĵ�����[%d]���ǵ��ֽڼĴ�����", sec);
		return false;
	}

	//��MI������������
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_WR_PMC_REG;
	cmd.data.axis_index = 0xFF;
	cmd.data.data[0] = sec;
	cmd.data.data[1] = index;
	cmd.data.data[2] = value;
	this->m_p_mi_comm->WriteCmd(cmd);
	return true;
}

/**
 * @brief ���üĴ���ֵ
 * @param sec
 * @param index
 * @param value
 * @return
 */
bool PmcRegister::SetRegValue(PmcRegSection sec, uint16_t index, uint16_t value){
	switch(sec){

#ifndef USES_PMC_2_0
	case PMC_REG_D:
		if(index >= D_REG_COUNT)
			return false;
		this->m_d_reg[index] = value;
//		this->m_mask_save_keep |= (0x01<<D_REG_BIT);
		break;
	case PMC_REG_C:
		if(index >= C_REG_COUNT)
			return false;
		this->m_c_reg[index] = value;
		break;
	case PMC_REG_T:
		if(index >= T_REG_COUNT)
			return false;
		this->m_t_reg[index] = value;
		break;
	case PMC_REG_DC:
		if(index >= C_REG_COUNT)
			return false;
		this->m_dc_reg[index] = value;
//		this->m_mask_save_keep |= (0x01<<DC_REG_BIT);
		break;
	case PMC_REG_DT:
		if(index >= T_REG_COUNT)
			return false;
		this->m_dt_reg[index] = value;
//		this->m_mask_save_keep |= (0x01<<DT_REG_BIT);
		break;
#else
	case PMC_REG_D:
		if(index >= D_REG_COUNT-1)
			return false;
		memcpy(&m_d_reg[index], &value, 2);
		break;
	case PMC_REG_C:
		if(index >= C_REG_COUNT*4-1)
			return false;
		memcpy(&m_c_reg[index], &value, 2);
		break;
	case PMC_REG_T:
		if(index >= T_REG_COUNT*2-1)
			return false;
		memcpy(&m_t_reg[index], &value, 2);
		break;
#endif
	default:
		g_ptr_trace->PrintTrace(TRACE_WARNING, PMC_REGISTER, "�Ĵ�����[%d]����˫�ֽڼĴ�����", sec);
		break;
	}

	//��MI������������
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_WR_PMC_REG;
	cmd.data.axis_index = 0xFF;
	cmd.data.data[0] = sec;
	cmd.data.data[1] = index;
	cmd.data.data[2] = value;
	this->m_p_mi_comm->WriteCmd(cmd);

	return true;
}

/**
 * @brief ���üĴ���bitֵ
 * @param sec : pmc�μĴ���
 * @param index �� ����ƫ��
 * @param bit �� λƫ��
 * @param value �� ֵ
 * @return
 */
bool PmcRegister::SetRegBitValue(PmcRegSection sec, uint16_t index, uint8_t bit, uint8_t count, uint32_t value){
#ifdef USES_PMC_2_0
	if(bit > 7)
		return false;
#endif

	//��MI������������
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_SET_PMC_REG_BIT;
	cmd.data.axis_index = 0xFF;
	cmd.data.data[0] = sec;
	cmd.data.data[1] = index;
	cmd.data.data[2] = bit;
	cmd.data.data[3] = count;
	cmd.data.data[4] = value&0xFFFF;
	cmd.data.data[5] = ((value>>16)&0xFFFF);
	this->m_p_mi_comm->WriteCmd(cmd);

	//�޸ı��ؼĴ���
	uint8_t bit_value = 0;
	if(sec == PMC_REG_F){  //ֻ��Ҫֱ���޸�F�Ĵ���
		for(uint8_t i = 0; i < count; i++){
			if(index >= F_REG_COUNT)
				break;
			bit_value = ((value>>i)&0x01);
//			printf("index=%hu, bit=%hhu, bit_value=%hhu\n", index, bit, bit_value);
			if(bit_value == 0)
				this->m_f_reg.all[index] &= ~(0x01<<bit);
			else if(bit_value == 1)
				this->m_f_reg.all[index] |= bit_value<<bit;

			if(++bit > 7){
				bit = 0;
				index++;
			}
		}
//		printf("SetRegBitValue, F[%hu]=0x%hhx\n", index, m_f_reg.all[index]);
	}
	return true;
}


/**
 * @brief ��ȡ�Ĵ���ֵ
 * @param sec
 * @param index
 * @param value
 * @return
 */
bool PmcRegister::GetRegValue(PmcRegSection sec, uint16_t index, uint8_t &value){
	switch(sec){
	case PMC_REG_X:
#ifdef USES_PMC_2_0
		if(index < 128)   //IO-LINK ͨ��1
			value = this->m_x_reg[index];
		else if(index >= 200 && index < 328)    //IO-LINK ͨ��2
			value = this->m_x_reg[index-200+128];
		else if(index >= 1000 && index < 1007)
			value = this->m_x_reg[index-1000+256];
		else if(index == 2000)   //����DI
			value = this->m_x_reg[264];
		else
			return false;
#else
		if(index >= X_REG_COUNT)
			return false;
		value = this->m_x_reg[index];
#endif
		break;
	case PMC_REG_Y:
#ifdef USES_PMC_2_0
		if(index < 128)   //IO-LINK ͨ��1
			value = this->m_y_reg[index];
		else if(index >= 200 && index < 328)    //IO-LINK ͨ��2
			value = this->m_y_reg[index-200+128];
		else if(index >= 1000 && index < 1007)
			value = this->m_y_reg[index-1000+256];
		else
			return false;
#else
		if(index >= Y_REG_COUNT)
			return false;
		value = this->m_y_reg[index];
#endif
		break;
	case PMC_REG_F:
		if(index >= F_REG_COUNT)
			return false;
		value = this->m_f_reg.all[index];
		break;
	case PMC_REG_G:
		if(index >= G_REG_COUNT)
			return false;
		value = this->m_g_reg.all[index];
		break;
	case PMC_REG_R:
		if(index >= R_REG_COUNT)
			return false;
		value = this->m_r_reg[index];
		break;
	case PMC_REG_K:
		if(index >= K_REG_COUNT)
			return false;
		value = this->m_k_reg[index];
		break;
	case PMC_REG_A:
		if(index >= A_REG_COUNT)
			return false;
		value = this->m_a_reg[index];
		break;
#ifdef USES_PMC_2_0
	case PMC_REG_D:
		if(index >= D_REG_COUNT)
			return false;
		value = this->m_d_reg[index];
		break;
	case PMC_REG_C:
		if(index >= C_REG_COUNT*4)
			return false;
		value = this->m_c_reg[index];
		break;
	case PMC_REG_T:
		if(index >= T_REG_COUNT*2)
			return false;
		value = this->m_t_reg[index];
		break;
	case PMC_REG_E:
		if(index >= E_REG_COUNT)
			return false;
		value = this->m_e_reg[index];
		break;
#endif
	default:
		g_ptr_trace->PrintTrace(TRACE_WARNING, PMC_REGISTER, "�Ĵ�����[%d]���ǵ��ֽڼĴ�����", sec);
		return false;
	}

	return true;
}

/**
 * @brief ��ȡ�Ĵ���ֵ
 * @param sec
 * @param index
 * @param value
 * @return
 */
bool PmcRegister::GetRegValue(PmcRegSection sec, uint16_t index, uint16_t &value){
	switch(sec){
#ifndef USES_PMC_2_0
	case PMC_REG_D:
		if(index >= D_REG_COUNT)
			return false;
		value = this->m_d_reg[index];
		break;
	case PMC_REG_C:
		if(index >= C_REG_COUNT)
			return false;
		value = this->m_c_reg[index];
		break;
	case PMC_REG_T:
		if(index >= T_REG_COUNT)
			return false;
		value = this->m_t_reg[index];
		break;
	case PMC_REG_DC:
		if(index >= C_REG_COUNT)
			return false;
		value = this->m_dc_reg[index];
		break;
	case PMC_REG_DT:
		if(index >= T_REG_COUNT)
			return false;
		value = this->m_dt_reg[index];
		break;
#else
	case PMC_REG_D:
		if(index >= D_REG_COUNT-1)
			return false;
		memcpy(&value, &m_d_reg[index], 2);
		break;
	case PMC_REG_C:
		if(index >= C_REG_COUNT*4-1)
			return false;
		memcpy(&value, &m_c_reg[index], 2);
		break;
	case PMC_REG_T:
		if(index >= T_REG_COUNT*2-1)
			return false;
		memcpy(&value, &m_t_reg[index], 2);
		break;
#endif
	default:
		g_ptr_trace->PrintTrace(TRACE_WARNING, PMC_REGISTER, "�Ĵ�����[%d]����˫�ֽڼĴ�����", sec);
		break;
	}

	return true;
}

/**
 * @brief ��ȡ���ֽڼĴ���ֵ
 */
bool PmcRegister::GetRegValue(PmcRegSection sec, uint16_t index, int32_t &value){
	if(sec != PMC_REG_D || index >= D_REG_COUNT-4)
		return false;


	memcpy(&value, &m_d_reg[index], 4);
	return true;
}

/**
 * @brief ��ȡ����Ĵ���ֵ
 * @param sec
 * @param index
 * @param count
 * @param value
 * @return
 */
bool PmcRegister::GetRegValueMulti(PmcRegSection sec, uint16_t index, uint16_t count, uint8_t *value){
	//
	switch(sec){
	case PMC_REG_X:
#ifdef USES_PMC_2_0
		if(index < 128 && index+count <= 128)   //IO-LINK ͨ��1
			memcpy(value, &m_x_reg[index], count);
		else if(index >= 200 && index+count > 200 && index+count <= 328)    //IO-LINK ͨ��2
			memcpy(value, &m_x_reg[index-200+128], count);
		else if(index >= 1000 && index+count > 1000 && index+count <= 1008)
			memcpy(value, &m_x_reg[index-1000+256], count);
		else if(index == 2000 && count == 1)   //����DI
			memcpy(value, &m_x_reg[264], count);
		else
			return false;
#else
		if(index+count > X_REG_COUNT)
			return false;

		memcpy(value, &m_x_reg[index], count);
#endif
		break;
	case PMC_REG_Y:
#ifdef USES_PMC_2_0
		if(index < 128 && index+count <= 128)   //IO-LINK ͨ��1
			memcpy(value, &m_y_reg[index], count);
		else if(index >= 200 && index+count > 200 && index+count <= 328)    //IO-LINK ͨ��2
			memcpy(value, &m_y_reg[index-200+128], count);
		else if(index >= 1000 && index+count > 1000 && index+count <= 1008)
			memcpy(value, &m_y_reg[index-1000+256], count);
		else
			return false;
#else
		if(index+count > Y_REG_COUNT)
			return false;

		memcpy(value, &m_y_reg[index], count);
#endif
		break;
	case PMC_REG_F:
		if(index+count > F_REG_COUNT)
			return false;

		memcpy(value, &m_f_reg.all[index], count);
		break;
	case PMC_REG_G:
		if(index+count > G_REG_COUNT)
			return false;
		memcpy(value, &m_g_reg.all[index], count);
		break;
	case PMC_REG_R:
		if(index+count > R_REG_COUNT)
			return false;

		memcpy(value, &m_r_reg[index], count);
		break;
	case PMC_REG_K:
		if(index+count > K_REG_COUNT)
			return false;

		memcpy(value, &m_k_reg[index], count);
		break;
	case PMC_REG_A:
		if(index+count > A_REG_COUNT)
			return false;

		memcpy(value, &m_a_reg[index], count);
		break;
#ifdef USES_PMC_2_0
	case PMC_REG_D:
		if(index+count > D_REG_COUNT)
			return false;
		memcpy(value, &m_d_reg[index], count);
		break;
	case PMC_REG_C:
		if(index+count > C_REG_COUNT*4)
			return false;
		memcpy(value, &m_c_reg[index], count);
		break;
	case PMC_REG_T:
		if(index+count > T_REG_COUNT*2)
			return false;
		memcpy(value, &m_t_reg[index], count);
		break;
	case PMC_REG_E:
		if(index+count > E_REG_COUNT)
			return false;
		memcpy(value, &m_e_reg[index], count);
		break;
#endif
	default:
		g_ptr_trace->PrintTrace(TRACE_WARNING, PMC_REGISTER, "�Ĵ�����[%d]���ǵ��ֽڼĴ�����", sec);
		return false;
	}

	return true;
}

/**
 * @brief ��ȡ����Ĵ���ֵ
 * @param sec
 * @param index
 * @param count
 * @param value
 * @return
 */
bool PmcRegister::GetRegValueMulti(PmcRegSection sec, uint16_t index, uint16_t count, uint16_t *value){
	//
	switch(sec){
#ifndef USES_PMC_2_0
	case PMC_REG_D:
		if(index+count > D_REG_COUNT)
			return false;

		memcpy(value, &m_d_reg[index], count*2);
	//	value = this->m_d_reg[index];
		break;
	case PMC_REG_C:
		if(index+count > C_REG_COUNT)
			return false;

		memcpy(value, &m_c_reg[index], count*2);
	//	value = this->m_c_reg[index];
		break;
	case PMC_REG_T:
		if(index+count > T_REG_COUNT)
			return false;

		memcpy(value, &m_t_reg[index], count*2);
	//	value = this->m_t_reg[index];
		break;
	case PMC_REG_DC:
		if(index+count > C_REG_COUNT)
			return false;

		memcpy(value, &m_dc_reg[index], count*2);
	//	value = this->m_dc_reg[index];
		break;
	case PMC_REG_DT:
		if(index+count > T_REG_COUNT)
			return false;

		memcpy(value, &m_dt_reg[index], count*2);
	//	value = this->m_dt_reg[index];
		break;
#else
	case PMC_REG_D:
		if(index+count*2 > D_REG_COUNT)
			return false;

		memcpy(value, &m_d_reg[index], count*2);
	//	value = this->m_d_reg[index];
		break;
	case PMC_REG_C:
		if(index+count*2 > C_REG_COUNT*4)
			return false;

		memcpy(value, &m_c_reg[index], count*2);
	//	value = this->m_c_reg[index];
		break;
	case PMC_REG_T:
		if(index+count*2 > T_REG_COUNT*2)
			return false;

		memcpy(value, &m_t_reg[index], count*2);
	//	value = this->m_t_reg[index];
		break;
#endif
	default:
		g_ptr_trace->PrintTrace(TRACE_WARNING, PMC_REGISTER, "�Ĵ�����[%d]����˫�ֽڼĴ�����", sec);
		break;
	}

	return true;
}

/**
 * @brief �������ʧ������
 */
void PmcRegister::SaveRegData(){
//	if(this->m_mask_save_keep == 0x00)
//		return;
	if(this->m_n_fp == -1){
		m_n_fp = open(PATH_PMC_REG, O_RDWR);
		if(m_n_fp < 0){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��pmc�Ĵ����ļ�ʧ�ܣ�");
			return;//�ļ���ʧ��
		}
	}

	//����K����
	uint16_t size = K_REG_COUNT;
	ssize_t res = 0;
//	if(this->m_mask_save_keep & (0x01<<K_REG_BIT)){
		if(-1 == lseek(m_n_fp, 2, SEEK_SET)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תK�Ĵ���д���ַʧ�ܣ�");
			return;
		}

		res = write(m_n_fp, this->m_k_reg, size);
		if(res == -1){//д��ʧ��
			close(m_n_fp);
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "д��K�Ĵ���ʧ�ܣ�");
			return;
		}
//		this->m_mask_save_keep &= ~(0x01<<K_REG_BIT);
//	}

#ifndef USES_PMC_2_0

	//����D����
//	if(this->m_mask_save_keep & (0x01<<D_REG_BIT)){
		size = D_REG_COUNT*2;
		if(-1 == lseek(m_n_fp, 2, SEEK_CUR)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תD�Ĵ���д���ַʧ�ܣ�");
			return;
		}

		res = write(m_n_fp, this->m_d_reg, size);
		if(res == -1){//д��ʧ��
			close(m_n_fp);
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "д��D�Ĵ���ʧ�ܣ�");
			return;
		}
//		this->m_mask_save_keep &= ~(0x01<<D_REG_BIT);
//	}

	//����DC����
//	if(this->m_mask_save_keep & (0x01<<DC_REG_BIT)){
		size = C_REG_COUNT*2;
		if(-1 == lseek(m_n_fp, 2, SEEK_CUR)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תDC�Ĵ���д���ַʧ�ܣ�");
			return;
		}

		res = write(m_n_fp, this->m_dc_reg, size);
		if(res == -1){//д��ʧ��
			close(m_n_fp);
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "д��DC�Ĵ���ʧ�ܣ�");
			return;
		}

//		this->m_mask_save_keep &= ~(0x01<<DC_REG_BIT);
//	}

	//����DT����
//	if(this->m_mask_save_keep & (0x01<<DT_REG_BIT)){
		size = T_REG_COUNT*2;
		if(-1 == lseek(m_n_fp, 2, SEEK_CUR)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תDT�Ĵ���д���ַʧ�ܣ�");
			return;
		}


		res = write(m_n_fp, this->m_dt_reg, size);
		if(res == -1){//д��ʧ��
			close(m_n_fp);
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "д��DT�Ĵ���ʧ�ܣ�");
			return;
		}

//		this->m_mask_save_keep &= ~(0x01<<DT_REG_BIT);
//	}

	//����C�Ĵ���
		size = C_REG_COUNT*2;
		if(-1 == lseek(m_n_fp, 2, SEEK_CUR)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תC�Ĵ���д���ַʧ�ܣ�");
			return;
		}

		res = write(m_n_fp, this->m_c_reg, size);
		if(res == -1){//д��ʧ��
			close(m_n_fp);
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "д��C�Ĵ���ʧ�ܣ�");
			return;
		}
#else
		//����D����
		size = D_REG_COUNT;
		if(-1 == lseek(m_n_fp, 2, SEEK_CUR)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תD�Ĵ���д���ַʧ�ܣ�");
			return;
		}

		res = write(m_n_fp, this->m_d_reg, size);
		if(res == -1){//д��ʧ��
			close(m_n_fp);
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "д��D�Ĵ���ʧ�ܣ�");
			return;
		}


		//����C�Ĵ���
		size = C_REG_COUNT*4;
		if(-1 == lseek(m_n_fp, 2, SEEK_CUR)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תC�Ĵ���д���ַʧ�ܣ�");
			return;
		}

		res = write(m_n_fp, this->m_c_reg, size);
		if(res == -1){//д��ʧ��
			close(m_n_fp);
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "д��C�Ĵ���ʧ�ܣ�");
			return;
		}

		//����T�Ĵ���
		size = T_REG_COUNT*2;
		if(-1 == lseek(m_n_fp, 2, SEEK_CUR)){
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��תT�Ĵ���д���ַʧ�ܣ�");
			return;
		}

		res = write(m_n_fp, this->m_t_reg, size);
		if(res == -1){//д��ʧ��
			close(m_n_fp);
			g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "д��T�Ĵ���ʧ�ܣ�");
			return;
		}
#endif

	fsync(m_n_fp);
	close(m_n_fp);
	m_n_fp = -1;

}
