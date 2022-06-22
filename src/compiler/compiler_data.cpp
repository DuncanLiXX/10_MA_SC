/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file Compiler_data.cpp
 *@author gonghao
 *@date 2020/03/31
 *@brief ��ͷ�ļ�ΪG���������ʹ�õ����ݽṹ�Ķ���
 *@version
 */

#include "compiler_data.h"


const int kMaxGCodeCount = 120; //ϵͳ����Gָ�����ֵ

const int kMaxFileMapSize = 20*1024*1024;   //�ļ��ڴ�ӳ�������ֵ  20M

//G����ģ̬ӳ�䣬�±�ΪG����������ȡ�������֣�����G02��G02.3��������Ϊ2
//���д���G120���Զ���Gָ����ڵ�39��ģ̬,�Զ���Gָ���赥��һ��
const unsigned char GCode2Mode[kMaxGCodeCount] = {1, 1, 1, 1, 0, 0, 1, 0, 0, 0,     		//G0~G9
										0, 0, 21, 21, 0, 17, 17, 2, 2, 2, 		//G10~G19
										6, 6, 4, 4, 0, 19, 19, 0, 0, 0,			//G20~G29
										0, 0, 0, 1, 1, 1, 1, 0, 0, 0,   		//G30~G39
										7, 7, 7, 8, 8, 0, 0, 0, 0, 8,			//G40~G49
										11, 11, 0, 0, 14, 14, 14, 14, 14, 14, 	//G50~G59
										0, 15, 15, 15, 15, 0, 12, 12, 16, 16,   //G60~G69
										0, 0, 0, 9, 9, 0, 9, 0, 0, 0, 			//G70~G79
										9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 			//G80~G89
										3, 3, 0, 5, 5, 5, 13, 13, 10, 10,		//G90~G99
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			//G100~G109
										0, 0, 21, 21, 0, 0, 0, 0, 0, kMaxGModeCount-1};  		//G110~G119,

//����������Ӧ�ľֲ���������, ˳���ӦA/B/C/D/E...X/Y/Z��0��ʾ�Ƿ�
const int kMacroParamToLocalVarIndex[] = {1,2,3,7,8,9,0,11,4,5,6,0,13,0,0,0,17,18,19,20,21,22,23,24,25,26};

//ѭ��ָ�������Ӧ�ľֲ���������, ˳���ӦA/B/C/D/E...X/Y/Z��0��ʾ�Ƿ��� ��Ժ�����Ա���������P����
const int kLoopParamToLocalVarIndex[] = {1,2,3,7,8,9,0,11,4,5,6,0,13,0,15,0,17,18,19,20,21,22,23,24,25,26};

//ѭ��ָ�������Ӧȫ�ֱ���������˳���ӦA/B/C/D/E...X/Y/Z��0��ʾ�Ƿ�
const int kLoopParamToGlobalVarIndex[] = {171,172,173,177,178,179,0,181,174,175,176,0,183,0,185,0,187,188,189,190,191,192,193,194,195,196};

/**
 * @brief ��һ�����ļ������ڴ�ӳ��
 * @param name : �ļ���
 * @param sub_flag : �ӳ����־
 * @return true--�ɹ�   false--ʧ��
 */
bool AsFileMapInfo::OpenFile(const char *name, bool sub_flag){
	printf("AsFileMapInfo::OpenFile(),  [%s], [%s]\n", this->str_file_name, name);
	if(sub_flag){
		printf("open sub file: 0x%x\n", (int)ptr_map_file);
		this->Clear();
	}
	else
		CloseFile();


	strcpy(str_file_name, name);

	if(strlen(str_file_name) == 0)
		return false;


//	struct stat statbuf;
//	if(stat(str_file_name, &statbuf) == 0)
//		ln_file_size = statbuf.st_size;  //��ȡ�ļ��ܴ�С
//	else{
//		g_ptr_trace->PrintLog(LOG_ALARM, "��ȡ�ļ�[%s]��Сʧ�ܣ�", str_file_name);
//		return false;
//	}

//	printf("page size = %ld\n", sysconf(_SC_PAGE_SIZE));

	fp_cur = open(str_file_name, O_RDONLY); //ֻ�����ļ�
	if(fp_cur < 0){
		g_ptr_trace->PrintLog(LOG_ALARM, "���ļ�[%s]ʧ�ܣ�", str_file_name);
		return false;//�ļ���ʧ��
	}
	else{
		g_ptr_trace->PrintLog(LOG_MACHINING_INFO, "�򿪼ӹ��ļ�[%s]\n", str_file_name);
	}

	ln_file_size = lseek(fp_cur, 0L, SEEK_END);

	lseek(fp_cur, 0L, SEEK_SET);


	if(ln_file_size < kMaxFileMapSize)
		ln_map_blocksize = ln_file_size;
	else
		ln_map_blocksize = kMaxFileMapSize;

	//����ӳ���ʱ
	struct timeval tvStart;
	struct timeval tvNow;
	unsigned int nTimeDelay = 0;

	gettimeofday(&tvStart, NULL);

	if(ln_file_size > 0){  //�ļ��ǿ�ӳ���������
		this->ptr_map_file = (char *)mmap(nullptr, ln_map_blocksize, PROT_READ, MAP_PRIVATE, fp_cur, ln_map_start);
		if(ptr_map_file == (char *)MAP_FAILED){
	//	    printf("failed to map the file[%s], size = %lld, errno = %d\n", name, ln_map_blocksize, errno);
			g_ptr_trace->PrintLog(LOG_ALARM, "ӳ���ļ�[%s]ʧ�ܣ�size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);

			//�ر��ļ�
			close(fp_cur);
			fp_cur = -1;

			memset(str_file_name, 0x00, kMaxPathLen);
			return false;  //ӳ��ʧ��
		}
		else{
			gettimeofday(&tvNow, NULL);
			nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;

			printf("map file ok�� time: %d us, fp_cur = %d, ptr_map_file=%d\n", nTimeDelay, fp_cur, (int)this->ptr_map_file);
		}
	}


	return true;
}

/**
 * @brief �ر��ļ�������λ����
 */
void AsFileMapInfo::CloseFile(){
	printf("AsFileMapInfo::CloseFile(), ptr_map_file = %d, [%s]\n", (int)ptr_map_file, this->str_file_name);
	if(ptr_map_file != (char *)MAP_FAILED){
		munmap(ptr_map_file, ln_map_blocksize);   //ȡ��ӳ��
		ptr_map_file = (char *)MAP_FAILED;
	}
	if(fp_cur > 0){
		close(fp_cur);
		fp_cur = -1;
	}
	ln_file_size = 0;
	ln_map_start = 0;
	ln_map_blocksize = 0;
	memset(str_file_name, 0x00, kMaxPathLen);
}

/**
 * @brief �ر��ļ�ӳ��
 */
void AsFileMapInfo::UnmapFile(){
	if(ptr_map_file != (char *)MAP_FAILED){
		munmap(ptr_map_file, ln_map_blocksize);   //ȡ��ӳ��
		ptr_map_file = (char *)MAP_FAILED;
	}

	if(fp_cur > 0){
		close(fp_cur);
		fp_cur = -1;
	}

	ln_file_size = 0;
	ln_map_start = 0;
	ln_map_blocksize = 0;
}

/**
 * @brief ����ӳ���ļ�
 */
bool AsFileMapInfo::RemapFile(){
	printf("AsFileMapInfo::RemapFile(), 0x%x, [%s]\n", ptr_map_file, this->str_file_name);
	if(strlen(str_file_name) == 0)
		return false;
//	struct stat statbuf;
//	if(stat(str_file_name, &statbuf) == 0)
//		ln_file_size = statbuf.st_size;  //��ȡ�ļ��ܴ�С
//	else{
//		g_ptr_trace->PrintLog(LOG_ALARM, "��ȡ�ļ�[%s]��Сʧ�ܣ�", str_file_name);
//		return false;
//	}

//	printf("page size = %ld\n", sysconf(_SC_PAGE_SIZE));

	fp_cur = open(str_file_name, O_RDONLY); //ֻ�����ļ�
	if(fp_cur < 0){
		g_ptr_trace->PrintLog(LOG_ALARM, "���ļ�[%s]ʧ�ܣ�", str_file_name);
		return false;//�ļ���ʧ��
	}
	else{
		g_ptr_trace->PrintLog(LOG_MACHINING_INFO, "�򿪼ӹ��ļ�[%s]\n", str_file_name);
	}

	ln_file_size = lseek(fp_cur, 0L, SEEK_END);

	lseek(fp_cur, 0L, SEEK_SET);

	if(ln_file_size < kMaxFileMapSize)
		ln_map_blocksize = ln_file_size;
	else
		ln_map_blocksize = kMaxFileMapSize;

	//����ӳ���ʱ
	struct timeval tvStart;
	struct timeval tvNow;
	unsigned int nTimeDelay = 0;

	gettimeofday(&tvStart, NULL);

	if(ln_file_size > 0){  //�ļ��ǿ�ӳ���������
		this->ptr_map_file = (char *)mmap(nullptr, ln_map_blocksize, PROT_READ, MAP_PRIVATE, fp_cur, ln_map_start);
		if(ptr_map_file == (char *)MAP_FAILED){
	//	    printf("failed to map the file[%s], size = %lld, errno = %d\n", name, ln_map_blocksize, errno);
			g_ptr_trace->PrintLog(LOG_ALARM, "ӳ���ļ�[%s]ʧ�ܣ�size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
			return false;  //ӳ��ʧ��
		}
		else{
			gettimeofday(&tvNow, NULL);
			nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;

			printf("map file ok�� time: %d us, fp_cur = %d, map_ptr = 0x%x\n", nTimeDelay, fp_cur, (uint32_t)ptr_map_file);
		}
	}
	return true;
}

/**
 * @brief ���·�ҳ
 */
bool AsFileMapInfo::Swapdown(){
	printf("file swap down\n");
	if(ln_map_start+ln_map_blocksize >= ln_file_size)
		return true;

	if(-1 == munmap(ptr_map_file, ln_map_blocksize)){   //ȡ��ӳ��
		g_ptr_trace->PrintLog(LOG_ALARM, "ȡ��ӳ���ļ�[%s]ʧ�ܣ�size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;
	}

	ln_map_start += ln_map_blocksize;

	ln_map_blocksize = ln_file_size - ln_map_start;

	if(ln_map_blocksize > kMaxFileMapSize)
		ln_map_blocksize = kMaxFileMapSize;

	ptr_map_file = (char *)mmap(nullptr, (size_t)ln_map_blocksize, PROT_READ, MAP_PRIVATE, fp_cur, (off_t)ln_map_start);

	if(ptr_map_file == (char *)MAP_FAILED){
		g_ptr_trace->PrintLog(LOG_ALARM, "���·�ҳӳ���ļ�[%s]ʧ�ܣ�size=%hld, start = %hld, fp = %d, errno=%d", str_file_name, ln_map_blocksize,
				ln_map_start, fp_cur, errno);
		return false;  //ӳ��ʧ��
	}

//	DropCaches(1);

	return true;
}

/**
 * @brief ���Ϸ�ҳ
 */
bool AsFileMapInfo::Swapup(){
	if(ln_map_start == 0)
		return true;

	if(-1 == munmap(ptr_map_file, ln_map_blocksize)){   //ȡ��ӳ��
		g_ptr_trace->PrintLog(LOG_ALARM, "ȡ��ӳ���ļ�[%s]ʧ�ܣ�size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;
	}

	ln_map_blocksize = ln_map_start;

	if(ln_map_blocksize > kMaxFileMapSize)
		ln_map_blocksize = kMaxFileMapSize;

	ln_map_start -= ln_map_blocksize;


	ptr_map_file = (char *)mmap(nullptr, ln_map_blocksize, PROT_READ, MAP_PRIVATE, fp_cur, ln_map_start);

	if(ptr_map_file == (char *)MAP_FAILED){
		g_ptr_trace->PrintLog(LOG_ALARM, "���Ϸ�ҳӳ���ļ�[%s]ʧ�ܣ�size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;  //ӳ��ʧ��
	}



	return true;
}

/**
 * @brief ��λ����ǰ�ļ�ͷ
 */
bool AsFileMapInfo::ResetFile(){
	printf("AsFileMapInfo::ResetFile, [%s]\n", this->str_file_name);
	if(ln_map_start == 0 || ptr_map_file == (char *)MAP_FAILED)
		return true;
	if(-1 == munmap(ptr_map_file, ln_map_blocksize)){   //ȡ��ӳ��
		g_ptr_trace->PrintLog(LOG_ALARM, "ȡ��ӳ���ļ�[%s]ʧ�ܣ�size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;
	}

	ln_map_start = 0;
	ln_map_blocksize = 0;

	if(ln_file_size < kMaxFileMapSize)
		ln_map_blocksize = ln_file_size;
	else
		ln_map_blocksize = kMaxFileMapSize;

	this->ptr_map_file = (char *)mmap(nullptr, ln_map_blocksize, PROT_READ,MAP_SHARED, fp_cur, ln_map_start);
	if(ptr_map_file == (char *)MAP_FAILED && errno != 0){
		g_ptr_trace->PrintLog(LOG_ALARM, "��λӳ���ļ�[%s]ʧ�ܣ�size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;  //ӳ��ʧ��
	}

//	DropCaches(1);

	return true;
}

/**
 * @brief Ϊ�˽���ǰ��ȡλ����ת��pos������Ҫ����ӳ���ڴ��
 * @param pos : Ŀ��ƫ�Ƶ�ַ
 * @return
 */
bool AsFileMapInfo::JumpTo(uint64_t pos){
	if(pos >= this->ln_file_size)
		return false;

	if(pos >= this->ln_map_start && pos < this->ln_map_start+this->ln_map_blocksize){  //���ڵ�ǰӳ������
//		printf("jumpto3 , read : %c, file_size = %lld, block = %lld\n", *this->ptr_map_file, ln_file_size, ln_map_blocksize);
		return true;
	}


	//��Ҫ����ӳ������
	int block = pos/kMaxFileMapSize;  //ӳ�������

	if(-1 == munmap(ptr_map_file, ln_map_blocksize)){   //ȡ��ӳ��
		g_ptr_trace->PrintLog(LOG_ALARM, "ȡ����תӳ���ļ�[%s]ʧ�ܣ�size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;
	}

	ln_map_start = kMaxFileMapSize * block;
	ln_map_blocksize = ln_file_size - ln_map_start;

	if(ln_map_blocksize > kMaxFileMapSize)
		ln_map_blocksize = kMaxFileMapSize;

	this->ptr_map_file = (char *)mmap(nullptr, ln_map_blocksize, PROT_READ,MAP_SHARED, fp_cur, ln_map_start);
	if(ptr_map_file == (char *)MAP_FAILED){
		g_ptr_trace->PrintLog(LOG_ALARM, "��תӳ���ļ�[%s]ʧ�ܣ�size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;  //ӳ��ʧ��
	}

	return true;
}

/**
 * @brief �ָ�Ĭ��ֵ�����ر��ļ�
 */
void AsFileMapInfo::Clear(){
	fp_cur = -1;
	ln_file_size = 0;
	ln_map_start = 0;
	ln_map_blocksize = 0;
	ptr_map_file = (char *)MAP_FAILED;
	memset(str_file_name, 0x00, kMaxPathLen);
}

/*******************************************************************ModeCollect*****************************************************************/
/**
 * @breif: ��ʼ��ģʽ����
 */
void ModeCollect::Initialize(){
	Reset();
	d_mode = 0;
	h_mode = 0;
	t_mode = 0;
}

/**
 * @brief ��λģʽ���ϵ�Ĭ��״̬
 */
void ModeCollect::Reset(){
	gmode[1] = G00_CMD;  //01��Ĭ��G00
	gmode[2] = G17_CMD;  //02��Ĭ��G17
	gmode[3] = G90_CMD;  //03��Ĭ��G90
//	gmode[4] = G23_CMD;
	gmode[5] = G94_CMD;  //05��Ĭ��G94  ÿ���ӽ���
	gmode[6] = G21_CMD;  //06��Ĭ��G21  ���Ƶ�λ

	gmode[7] = G40_CMD;  //07��Ĭ��G40
	gmode[8] = G49_CMD;  //08��Ĭ��G49
	gmode[9] = G80_CMD;  //09��Ĭ��G80
	gmode[10] = G98_CMD;  //10��Ĭ��G98  ���س�ʼƽ��
	gmode[11] = G50_CMD;  //11��Ĭ��G50  ��������ȡ��
	gmode[14] = G54_CMD;  //14��Ĭ��G54  G54��������ϵ
	gmode[15] = G64_CMD;  //15��Ĭ��G64  ������ʽ
	gmode[16] = G69_CMD;  //16��Ĭ��G69  ������ת������ά����任��ʽOFF

	gmode[17] = G15_CMD;  //17��Ĭ��15   ������ָ��ȡ��
	gmode[19] = G26_CMD;  //19��Ĭ��G26  �����ٶȱ䶯���ON

	f_mode = 0.0;
	s_mode = 0.0;
	h_mode = 0;
    d_mode = 0;
}

/**
 * @brief ���ظ�ֵ�����
 * @param c
 * @return
 */
ModeCollect& ModeCollect::operator=( const ModeCollect& c){
	if(&c == this)
		return *this;
	memcpy(gmode, c.gmode, sizeof(uint16_t)*kMaxGModeCount);
	h_mode = c.h_mode;
	d_mode = c.d_mode;
	t_mode = c.t_mode;
	f_mode = c.f_mode;
	s_mode = c.s_mode;
	return *this;
}

/**
 * @brief ���ظ�ֵ�����
 * @param c
 * @return
 */
CompileStatusCollect& CompileStatusCollect::operator=( const CompileStatusCollect& c){
	if(&c == this)
		return *this;

	this->cur_pos = c.cur_pos;

//#ifdef USES_SPEED_TORQUE_CTRL
//	this->cur_velocity = c.cur_velocity;
//	this->cur_torque = c.cur_torque;
//#endif
	
	this->mode = c.mode;

	this->exact_stop_flag = c.exact_stop_flag;
	this->jump_flag = c.jump_flag;

	return *this;
}

/**
 * @brief ���ظ�ֵ�����
 * @param c
 * @return
 */
AsFileMapInfo& AsFileMapInfo::operator=( const AsFileMapInfo& c){
	if(&c == this)
		return *this;

	this->fp_cur = c.fp_cur;
	this->ln_file_size = c.ln_file_size;
	this->ln_map_blocksize = c.ln_map_blocksize;
	this->ln_map_start = c.ln_map_start;
	this->ptr_map_file = c.ptr_map_file;
	strcpy(str_file_name, c.str_file_name);


	return *this;
}

/**
 * @brief ��ֵ�����
 * @param c
 * @return
 */
LabelOffset& LabelOffset::operator=( const LabelOffset& c){
	if(&c == this)
		return *this;
	this->line_no = c.line_no;
	this->offset = c.offset;
	this->label = c.label;

	return *this;
}

/**
 * @brief ��ֵ�����
 * @param c
 * @return
 */
SubProgOffset& SubProgOffset::operator=( const SubProgOffset& c){
	if(&c == this)
		return *this;
	this->line_no = c.line_no;
	this->offset = c.offset;
	this->sub_index = c.sub_index;

	return *this;
}

/**
 * @brief ��ֵ�����
 * @param c
 * @return
 */
LoopRec& LoopRec::operator =(const LoopRec &c){
	if(&c == this)
		return *this;
	this->start_line_no = c.start_line_no;
	this->start_offset = c.start_offset;
	this->end_line_no = c.end_line_no;
	this->end_offset = c.end_offset;

	return *this;
}

/**
 * @brief ��ֵ�����
 * @param c
 * @return
 */
LoopOffset& LoopOffset::operator=( const LoopOffset& c){
	if(&c == this)
		return *this;
	this->line_no = c.line_no;
	this->offset = c.offset;
	this->loop_index = c.loop_index;

	return *this;
}

/**
 * @brief ��ֵ�����
 * @param c
 * @return
 */
IfElseOffset& IfElseOffset::operator=( const IfElseOffset& c){
	if(&c == this)
		return *this;
	this->line_no = c.line_no;
	this->offset = c.offset;
	return *this;
}


#ifdef USES_WOOD_MACHINE
SpindleStartOffset& SpindleStartOffset::operator=( const SpindleStartOffset& c){
	if(&c == this)
		return *this;
	this->line_no = c.line_no;
	this->m_code = c.m_code;
	this->s_code = c.s_code;
	this->jump_line = c.jump_line;
//	this->exec_step = c.exec_step;

	return *this;
}
#endif


/**
 * @brief ���ظ�ֵ�����
 * @param c
 * @return
 */
CompilerScene& CompilerScene::operator=( const CompilerScene& c){
	if(&c == this)
		return *this;

	this->n_sub_program = c.n_sub_program;
	this->b_eof = c.b_eof;
	this->compiler_status = c.compiler_status;
	this->file_map_info = c.file_map_info;
	this->file_state = c.file_state;
	this->head_state = c.head_state;
	this->ln_cur_line_no = c.ln_cur_line_no;
	this->ln_read_size = c.ln_read_size;
	this->ptr_cur_file_pos = c.ptr_cur_file_pos;
	this->p_last_move_msg = c.p_last_move_msg;
	this->work_mode = c.work_mode;
//	this->thread_state = c.thread_state;

	this->n_sub_call_times = c.n_sub_call_times;

	this->list_label = c.list_label;
	this->list_subprog = c.list_subprog;
	this->stack_loop = c.stack_loop;

	/***********if else ����***************/

	this->ifelse_vector.clear();
	ifelse_vector = c.ifelse_vector;
	/***********if else ����***************/


#ifdef USES_WOOD_MACHINE
	this->list_spd_start = c.list_spd_start;
#endif

	return *this;
}


/**
 * @brief ��ֵ�����
 * @param rec
 * @return
 */
MacroRec& MacroRec::operator=( const MacroRec& rec){
	if(&rec == this)
		return *this;
	this->opt = rec.opt;
	this->value = rec.value;
	return *this;
}

/**
 * @brief �ж������
 * @param one
 * @param two
 * @return
 */
bool operator ==( const MacroRec &one, const MacroRec &two){
	if(one.opt == two.opt &&
		one.value == two.value)
		return true;
	return false;
}

bool operator !=(const MacroRec &one, const MacroRec &two){
	if(one.opt == two.opt &&
		one.value == two.value)
		return false;
	return true;
}

/**
 * @brief ������ֵ����ʽ��ջ
 * @param des : Ŀ�ĵ�ַ
 * @param src ��Դ��ַ
 */
void CopyMacroExp(MacroExpression &des, MacroExpression &src){
	stack<MacroRec> temp;   //��ʱ�м�洢stack
	MacroRec rec;

	//��des���
	while(!des.empty()){
		des.pop();
	}

	//��src����������洢��temp��
	while(!src.empty()){
		rec = src.top();
		temp.push(rec);
		src.pop();
	}

	//��temp�е������ٵ���src��des����ɸ���
	while(!temp.empty()){
		rec = temp.top();
		des.push(rec);
		src.push(rec);
		temp.pop();
	}
}

/**
 * @brief �Ƚ���������ʽ�Ƿ�һ��
 * @param one
 * @param two
 * @return  һ�·���true����һ�·���false
 */
bool IsEqualMacroExp(MacroExpression &one, MacroExpression &two){
	if(one.size() != two.size())
		return false;
	stack<MacroRec> tmp;   //��ʱ�м�洢stack
	MacroRec rec1, rec2;

	bool res = true;
	while(!one.empty()){
		rec1 = one.top();
		rec2 = two.top();
		if(rec1 != rec2){
			res = false;
			goto END;
		}
		tmp.push(rec1);
		one.pop();
		two.pop();
	}

	END:
	//�ָ�����
	while(!tmp.empty()){
		rec1 = tmp.top();
		one.push(rec1);
		two.push(rec1);
		tmp.pop();
	}

	return res;
}

void LexerResult::Reset(){
	this->line_no = 0;
	this->offset = 0;
	this->macro_flag = false;
	this->error_code = ERR_NONE;
	this->nc_code.Reset();
}

void NcCode::Reset(){
	this->gcode.Reset();
	this->macro_cmd.Reset();
}

void LexerGCode::Reset(){
	this->mask_value = 0;
	this->mask_dot = 0;
	this->mask_macro = 0;

	memset(this->g_value, 0x00, sizeof(g_value));
	memset(this->value, 0x00, sizeof(value));
	memset(m_value, 0x00, sizeof(m_value));
	memset(t_value, 0x00, sizeof(t_value));
	memset(mask_pos, 0x00, sizeof(mask_pos));
	memset(mask_pos_macro, 0x00, sizeof(mask_pos_macro));
	memset(pos_value, 0x00, sizeof(pos_value));
	memset(mask_dot_ex, 0x00, sizeof(mask_dot_ex));

	this->gcode_count = 0;
	this->mcode_count = 0;
	this->tcode_count = 0;
	for(int i =0; i < kMaxMacroInLine; i++){
		while(!macro_expression[i].empty())
			macro_expression[i].pop();
	}
}

void LexerMacroCmd::Reset(){
	this->cmd = MACRO_CMD_INVALID;
	for(int i =0; i < 2; i++){
		while(!macro_expression[i].empty())
			macro_expression[i].pop();
	}
}

/**
 * @brief ��д��ֵ�����
 * @param v
 * @return
 */
MacroVarValue &MacroVarValue::operator=(const MacroVarValue &v){
	if(this == &v)
		return *this;
	this->value = v.value;
	this->init = v.init;
	return *this;
}
