/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file Compiler_data.cpp
 *@author gonghao
 *@date 2020/03/31
 *@brief 本头文件为G代码编译器使用的数据结构的定义
 *@version
 */

#include "compiler_data.h"


const int kMaxGCodeCount = 120; //系统定义G指令最大值

const int kMaxFileMapSize = 20*1024*1024;   //文件内存映射区最大值  20M

//G代码模态映射，下标为G代码索引，取整数部分，比如G02和G02.3的索引都为2
//所有大于G120的自定义G指令都属于第39组模态,自定义G指令需单独一行
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

//宏程序参数对应的局部变量索引, 顺序对应A/B/C/D/E...X/Y/Z，0表示非法
const int kMacroParamToLocalVarIndex[] = {1,2,3,7,8,9,0,11,4,5,6,0,13,0,0,0,17,18,19,20,21,22,23,24,25,26};

//循环指令参数对应的局部变量索引, 顺序对应A/B/C/D/E...X/Y/Z，0表示非法， 相对宏程序自变量增加了P参数
const int kLoopParamToLocalVarIndex[] = {1,2,3,7,8,9,0,11,4,5,6,0,13,0,15,0,17,18,19,20,21,22,23,24,25,26};

//循环指令参数对应全局变量索引，顺序对应A/B/C/D/E...X/Y/Z，0表示非法
const int kLoopParamToGlobalVarIndex[] = {171,172,173,177,178,179,0,181,174,175,176,0,183,0,185,0,187,188,189,190,191,192,193,194,195,196};

/**
 * @brief 打开一个新文件并做内存映射
 * @param name : 文件名
 * @param sub_flag : 子程序标志
 * @return true--成功   false--失败
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
//		ln_file_size = statbuf.st_size;  //获取文件总大小
//	else{
//		g_ptr_trace->PrintLog(LOG_ALARM, "获取文件[%s]大小失败！", str_file_name);
//		return false;
//	}

//	printf("page size = %ld\n", sysconf(_SC_PAGE_SIZE));

	fp_cur = open(str_file_name, O_RDONLY); //只读打开文件
	if(fp_cur < 0){
		g_ptr_trace->PrintLog(LOG_ALARM, "打开文件[%s]失败！", str_file_name);
		return false;//文件打开失败
	}
	else{
		g_ptr_trace->PrintLog(LOG_MACHINING_INFO, "打开加工文件[%s]\n", str_file_name);
	}

	ln_file_size = lseek(fp_cur, 0L, SEEK_END);

	lseek(fp_cur, 0L, SEEK_SET);


	if(ln_file_size < kMaxFileMapSize)
		ln_map_blocksize = ln_file_size;
	else
		ln_map_blocksize = kMaxFileMapSize;

	//测试映射耗时
	struct timeval tvStart;
	struct timeval tvNow;
	unsigned int nTimeDelay = 0;

	gettimeofday(&tvStart, NULL);

	if(ln_file_size > 0){  //文件非空映射才有意义
		this->ptr_map_file = (char *)mmap(nullptr, ln_map_blocksize, PROT_READ, MAP_PRIVATE, fp_cur, ln_map_start);
		if(ptr_map_file == (char *)MAP_FAILED){
	//	    printf("failed to map the file[%s], size = %lld, errno = %d\n", name, ln_map_blocksize, errno);
			g_ptr_trace->PrintLog(LOG_ALARM, "映射文件[%s]失败！size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);

			//关闭文件
			close(fp_cur);
			fp_cur = -1;

			memset(str_file_name, 0x00, kMaxPathLen);
			return false;  //映射失败
		}
		else{
			gettimeofday(&tvNow, NULL);
			nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;

			printf("map file ok， time: %d us, fp_cur = %d, ptr_map_file=%d\n", nTimeDelay, fp_cur, (int)this->ptr_map_file);
		}
	}


	return true;
}

/**
 * @brief 关闭文件，并复位参数
 */
void AsFileMapInfo::CloseFile(){
	printf("AsFileMapInfo::CloseFile(), ptr_map_file = %d, [%s]\n", (int)ptr_map_file, this->str_file_name);
	if(ptr_map_file != (char *)MAP_FAILED){
		munmap(ptr_map_file, ln_map_blocksize);   //取消映射
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
 * @brief 关闭文件映射
 */
void AsFileMapInfo::UnmapFile(){
	if(ptr_map_file != (char *)MAP_FAILED){
		munmap(ptr_map_file, ln_map_blocksize);   //取消映射
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
 * @brief 重新映射文件
 */
bool AsFileMapInfo::RemapFile(){
	printf("AsFileMapInfo::RemapFile(), 0x%x, [%s]\n", ptr_map_file, this->str_file_name);
	if(strlen(str_file_name) == 0)
		return false;
//	struct stat statbuf;
//	if(stat(str_file_name, &statbuf) == 0)
//		ln_file_size = statbuf.st_size;  //获取文件总大小
//	else{
//		g_ptr_trace->PrintLog(LOG_ALARM, "获取文件[%s]大小失败！", str_file_name);
//		return false;
//	}

//	printf("page size = %ld\n", sysconf(_SC_PAGE_SIZE));

	fp_cur = open(str_file_name, O_RDONLY); //只读打开文件
	if(fp_cur < 0){
		g_ptr_trace->PrintLog(LOG_ALARM, "打开文件[%s]失败！", str_file_name);
		return false;//文件打开失败
	}
	else{
		g_ptr_trace->PrintLog(LOG_MACHINING_INFO, "打开加工文件[%s]\n", str_file_name);
	}

	ln_file_size = lseek(fp_cur, 0L, SEEK_END);

	lseek(fp_cur, 0L, SEEK_SET);

	if(ln_file_size < kMaxFileMapSize)
		ln_map_blocksize = ln_file_size;
	else
		ln_map_blocksize = kMaxFileMapSize;

	//测试映射耗时
	struct timeval tvStart;
	struct timeval tvNow;
	unsigned int nTimeDelay = 0;

	gettimeofday(&tvStart, NULL);

	if(ln_file_size > 0){  //文件非空映射才有意义
		this->ptr_map_file = (char *)mmap(nullptr, ln_map_blocksize, PROT_READ, MAP_PRIVATE, fp_cur, ln_map_start);
		if(ptr_map_file == (char *)MAP_FAILED){
	//	    printf("failed to map the file[%s], size = %lld, errno = %d\n", name, ln_map_blocksize, errno);
			g_ptr_trace->PrintLog(LOG_ALARM, "映射文件[%s]失败！size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
			return false;  //映射失败
		}
		else{
			gettimeofday(&tvNow, NULL);
			nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;

			printf("map file ok， time: %d us, fp_cur = %d, map_ptr = 0x%x\n", nTimeDelay, fp_cur, (uint32_t)ptr_map_file);
		}
	}
	return true;
}

/**
 * @brief 向下翻页
 */
bool AsFileMapInfo::Swapdown(){
	printf("file swap down\n");
	if(ln_map_start+ln_map_blocksize >= ln_file_size)
		return true;

	if(-1 == munmap(ptr_map_file, ln_map_blocksize)){   //取消映射
		g_ptr_trace->PrintLog(LOG_ALARM, "取消映射文件[%s]失败！size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;
	}

	ln_map_start += ln_map_blocksize;

	ln_map_blocksize = ln_file_size - ln_map_start;

	if(ln_map_blocksize > kMaxFileMapSize)
		ln_map_blocksize = kMaxFileMapSize;

	ptr_map_file = (char *)mmap(nullptr, (size_t)ln_map_blocksize, PROT_READ, MAP_PRIVATE, fp_cur, (off_t)ln_map_start);

	if(ptr_map_file == (char *)MAP_FAILED){
		g_ptr_trace->PrintLog(LOG_ALARM, "向下翻页映射文件[%s]失败！size=%hld, start = %hld, fp = %d, errno=%d", str_file_name, ln_map_blocksize,
				ln_map_start, fp_cur, errno);
		return false;  //映射失败
	}

//	DropCaches(1);

	return true;
}

/**
 * @brief 向上翻页
 */
bool AsFileMapInfo::Swapup(){
	if(ln_map_start == 0)
		return true;

	if(-1 == munmap(ptr_map_file, ln_map_blocksize)){   //取消映射
		g_ptr_trace->PrintLog(LOG_ALARM, "取消映射文件[%s]失败！size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;
	}

	ln_map_blocksize = ln_map_start;

	if(ln_map_blocksize > kMaxFileMapSize)
		ln_map_blocksize = kMaxFileMapSize;

	ln_map_start -= ln_map_blocksize;


	ptr_map_file = (char *)mmap(nullptr, ln_map_blocksize, PROT_READ, MAP_PRIVATE, fp_cur, ln_map_start);

	if(ptr_map_file == (char *)MAP_FAILED){
		g_ptr_trace->PrintLog(LOG_ALARM, "向上翻页映射文件[%s]失败！size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;  //映射失败
	}



	return true;
}

/**
 * @brief 复位到当前文件头
 */
bool AsFileMapInfo::ResetFile(){
	printf("AsFileMapInfo::ResetFile, [%s]\n", this->str_file_name);
	if(ln_map_start == 0 || ptr_map_file == (char *)MAP_FAILED)
		return true;
	if(-1 == munmap(ptr_map_file, ln_map_blocksize)){   //取消映射
		g_ptr_trace->PrintLog(LOG_ALARM, "取消映射文件[%s]失败！size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
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
		g_ptr_trace->PrintLog(LOG_ALARM, "复位映射文件[%s]失败！size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;  //映射失败
	}

//	DropCaches(1);

	return true;
}

/**
 * @brief 为了将当前读取位置跳转到pos处，需要重新映射内存块
 * @param pos : 目的偏移地址
 * @return
 */
bool AsFileMapInfo::JumpTo(uint64_t pos){
	if(pos >= this->ln_file_size)
		return false;

	if(pos >= this->ln_map_start && pos < this->ln_map_start+this->ln_map_blocksize){  //处于当前映射区间
//		printf("jumpto3 , read : %c, file_size = %lld, block = %lld\n", *this->ptr_map_file, ln_file_size, ln_map_blocksize);
		return true;
	}


	//需要重新映射区块
	int block = pos/kMaxFileMapSize;  //映射块索引

	if(-1 == munmap(ptr_map_file, ln_map_blocksize)){   //取消映射
		g_ptr_trace->PrintLog(LOG_ALARM, "取消跳转映射文件[%s]失败！size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;
	}

	ln_map_start = kMaxFileMapSize * block;
	ln_map_blocksize = ln_file_size - ln_map_start;

	if(ln_map_blocksize > kMaxFileMapSize)
		ln_map_blocksize = kMaxFileMapSize;

	this->ptr_map_file = (char *)mmap(nullptr, ln_map_blocksize, PROT_READ,MAP_SHARED, fp_cur, ln_map_start);
	if(ptr_map_file == (char *)MAP_FAILED){
		g_ptr_trace->PrintLog(LOG_ALARM, "跳转映射文件[%s]失败！size=%hld, errno=%d", str_file_name, ln_map_blocksize, errno);
		return false;  //映射失败
	}

	return true;
}

/**
 * @brief 恢复默认值，不关闭文件
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
 * @breif: 初始化模式集合
 */
void ModeCollect::Initialize(){
	Reset();
	d_mode = 0;
	h_mode = 0;
	t_mode = 0;
}

/**
 * @brief 复位模式集合到默认状态
 */
void ModeCollect::Reset(){
	gmode[1] = G00_CMD;  //01组默认G00
	gmode[2] = G17_CMD;  //02组默认G17
	gmode[3] = G90_CMD;  //03组默认G90
//	gmode[4] = G23_CMD;
	gmode[5] = G94_CMD;  //05组默认G94  每分钟进给
	gmode[6] = G21_CMD;  //06组默认G21  公制单位

	gmode[7] = G40_CMD;  //07组默认G40
	gmode[8] = G49_CMD;  //08组默认G49
	gmode[9] = G80_CMD;  //09组默认G80
	gmode[10] = G98_CMD;  //10组默认G98  返回初始平面
	gmode[11] = G50_CMD;  //11组默认G50  比例缩放取消
	gmode[14] = G54_CMD;  //14组默认G54  G54工件坐标系
	gmode[15] = G64_CMD;  //15组默认G64  切削方式
	gmode[16] = G69_CMD;  //16组默认G69  坐标旋转或者三维坐标变换方式OFF

	gmode[17] = G15_CMD;  //17组默认15   极坐标指令取消
	gmode[19] = G26_CMD;  //19组默认G26  主轴速度变动检测ON

	f_mode = 0.0;
	s_mode = 0.0;
	h_mode = 0;
    d_mode = 0;
}

/**
 * @brief 重载赋值运算符
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
 * @brief 重载赋值运算符
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
 * @brief 重载赋值运算符
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
 * @brief 赋值运算符
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
 * @brief 赋值运算符
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
 * @brief 赋值运算符
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
 * @brief 赋值运算符
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
 * @brief 赋值运算符
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
 * @brief 重载赋值运算符
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

	/***********if else 处理***************/

	this->ifelse_vector.clear();
	ifelse_vector = c.ifelse_vector;
	/***********if else 处理***************/


#ifdef USES_WOOD_MACHINE
	this->list_spd_start = c.list_spd_start;
#endif

	return *this;
}


/**
 * @brief 赋值运算符
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
 * @brief 判断运算符
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
 * @brief 拷贝赋值宏表达式堆栈
 * @param des : 目的地址
 * @param src ：源地址
 */
void CopyMacroExp(MacroExpression &des, MacroExpression &src){
	stack<MacroRec> temp;   //临时中间存储stack
	MacroRec rec;

	//将des清空
	while(!des.empty()){
		des.pop();
	}

	//将src的数据逆序存储到temp中
	while(!src.empty()){
		rec = src.top();
		temp.push(rec);
		src.pop();
	}

	//将temp中的数据再倒回src和des，完成复制
	while(!temp.empty()){
		rec = temp.top();
		des.push(rec);
		src.push(rec);
		temp.pop();
	}
}

/**
 * @brief 比较两个宏表达式是否一致
 * @param one
 * @param two
 * @return  一致返回true，不一致返回false
 */
bool IsEqualMacroExp(MacroExpression &one, MacroExpression &two){
	if(one.size() != two.size())
		return false;
	stack<MacroRec> tmp;   //临时中间存储stack
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
	//恢复数据
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
 * @brief 重写赋值运算符
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
