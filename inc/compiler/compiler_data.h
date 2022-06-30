/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file Compiler_data.h
 *@author gonghao
 *@date 2020/03/31
 *@brief ��ͷ�ļ�ΪG���������ʹ�õ����ݽṹ������
 *@version
 */

#ifndef INC_COMPILER_COMPILER_DATA_H_
#define INC_COMPILER_COMPILER_DATA_H_

#include "global_include.h"

class RecordMsg;

const int kMaxTagCount = 26;     //����ַ��ǩ����26��Ӣ����ĸ
const int kMaxGCodeInLine = 16;  //һ�����G�������
#ifdef USES_WOOD_MACHINE
const int kMaxMCodeInLine = 16;   //һ�����M�������
const int kMaxTCodeInLine = 16;   //32;   //һ�����T�������
const int kMaxMacroInLine = 16;   //32;   //һ�����Macro���ʽ����
#else
const int kMaxMCodeInLine = 3;   //һ�����M�������
const int kMaxTCodeInLine = 1;   //һ�����T�������
const int kMaxMacroInLine = 8;   //һ�����Macro���ʽ����
#endif

const int kMaxLineSize = 256;   //һ��G��������ֽ���

const int kMaxAxisNameCount = 9;   //ͨ�������Ƹ��������9����X/Y/Z/A/B/C/U/V/W��

const int kMaxAlphaBufLen = 16;  //���ʽ������峤��
const int kMaxDigitBufLen = 16;  //��ֵ�������峤��

extern const int kMaxFileMapSize;   //�ļ��ڴ�ӳ�������ֵ
extern const int kMaxGCodeCount;     //ϵͳ����Gָ�����ֵ
extern const char *MacroKeys[];  //��ָ��
extern const char *MacroFunc[];  //����������
extern const unsigned char GCode2Mode[];   //G����ģ̬ӳ�䣬�±�ΪG����������ȡ�������֣�����G02��G02.3��������Ϊ2
extern const int kMacroParamToLocalVarIndex[]; //����������Ӧ�ľֲ���������, ˳���ӦA/B/C/D/E...X/Y/Z��0��ʾ�Ƿ�
extern const int kLoopParamToLocalVarIndex[];  //ѭ��ָ�������Ӧ�ľֲ���������, ˳���ӦA/B/C/D/E...X/Y/Z��0��ʾ�Ƿ��� ��Ժ�����Ա���������P����
extern const int kLoopParamToGlobalVarIndex[]; //ѭ��ָ�������Ӧȫ�ֱ���������˳���ӦA/B/C/D/E...X/Y/Z��0��ʾ�Ƿ�


//�������߳�״̬
enum CompilerState{
    IDLE = 0,   		//����
    RUN,    			//����
	WAIT_EXECUTE,		//�ȴ�ָ��executeִ�н���
	WAIT_RUN,			//�ȴ�ָ��RUN���н���
    PAUSE,    			//��ͣ
    STOP,     			//ֹͣ
    ERROR     			//����
};


//������ѭ��״̬
//enum CompilerMainState{
//    doEnd        =   0,
//    doRead       =   1,
//    doCompile    =   2,
//    doHead       =   3,
//    doHeadInfo   =   4,
//    doSubprogram =   5,
//    doMain       =   6,
//    doLineReady  =   7,
//    doLineStart  =   8,
//    doMacro      =   9,
//    doFind       =   10,
//    doReadSub    =   11,
//    doCallSub    =   12,
//    doJumpReady  =   13,
//    doJumpStart  =   14,
//    doCompiPause =   15,
//    doCompiWaitResume = 16
//};

//�����ļ�״̬
enum CompileFileState{
    FILE_HEAD = 0,    //�����ļ�ͷ����Ϣ
    FILE_MAIN,        //�����ļ���������
    FILE_FIND         //��λ�ļ��Ӻ���λ��
};

//�ļ�ͷ������״̬
enum CompileHeadState{
    HEAD_INFO = 0,   //ͷ����Ϣ
    SUBPROGRAM       //�ӳ���
};

//����������ģʽ
enum CompilerWorkMode{
	AUTO_COMPILER = 1,  //�Զ�������ģʽ
	MDA_COMPILER		//MDA������ģʽ
};

enum SimulateMode{
	SIM_NONE = 0,		//�Ƿ���ģʽ
	SIM_OUTLINE,		//��������
	SIM_TOOLPATH,		//��·����
	SIM_MACHINING		//�ӹ�����
};

//ָ����Ϣ����
enum CodeMsgType{
	NORMAL_MSG = 0,		//0	��ͨ��Ϣ
	ERROR_MSG = 2,  		// 2	������Ϣ
	LINE_MSG = 4,      		// 4 G01��Ϣ
	ARC_MSG,       		//5	G02/G03��Ϣ
	REF_RETURN_MSG = 8,     //8 ���زο�����Ϣ
	RAPID_MSG = 10,    		//10 G00��Ϣ
	FEED_MSG,      		//11 �����ٶ���Ϣ
	SPEED_MSG,          //12 ����ת����Ϣ

	TOOL_MSG = 14,      		//14 ������Ϣ
	CBS_MSG,        	//15 ������Ϣ
	MODE_MSG,			//16 ģʽ��Ϣ
	AUX_MSG,			//17 Mָ����Ϣ
	SPD_CHK_MSG,        //18 ����ת�ټ����Ϣ
	COMPENSATE_MSG,		//19 ������Ϣ
	LOOP_MSG,			//20 ѭ��ָ����Ϣ
	COORD_MSG,          //21 ����ϵ��Ϣ
	SUBPROG_CALL_MSG,   //22 �ӳ��������Ϣ
	MACRO_MSG,          //23 ��ָ����Ϣ
	SUBPROG_RETURN_MSG, //24 �ӳ��򷵻���Ϣ
	POLAR_INTP_MSG,		//25 ������岹����Ϣ
	CLEAR_CIRCLE_POS_MSG, //26 ������Ȧλ����Ϣ
	TIME_WAIT_MSG,		//27 ��ʱ��Ϣ
	SPEED_CTRL_MSG,     //28 �ٶ�ָ����Ϣ
	TORQUE_CTRL_MSG,    //29 ����ָ����Ϣ
	SKIP_MSG,            //30 ��תָ����Ϣ
	MACRO_PROG_CALL_MSG,  //31 ����������Ϣ
	AUTO_TOOL_MEASURE_MSG, //32  �Զ��Ե�ָ����Ϣ
	RESTART_OVER_MSG,     //33   �ӹ���λ�����Ϣ
	MOVE_MSG,             //34   �ƶ�ָ�������Ϣ
	GUARD_MSG           //��Ϣ��������
};

//�����ָ�����
enum MacroOpt{
	MACRO_OPT_INVALID = -1, //�Ƿ�������

	//����
	MACRO_OPT_FIX = 0,  //��ȡ������ȥС������
	MACRO_OPT_FUP,		//��ȡ������С�����ֽ�λ����������
	MACRO_OPT_ROUND,	//�������룬���������߼�����ָ����ʹ��ʱ���ڵ�1��С��λ���������룬��NC����ַ��ʹ��ʱ�����ݵ�ַ��С�趨��λ��������
	MACRO_OPT_ABS,		//����ֵ
	MACRO_OPT_SIN,		//����
	MACRO_OPT_COS,		//����
	MACRO_OPT_TAN,		//����
	MACRO_OPT_ASIN,		//������
	MACRO_OPT_ACOS,		//������
	MACRO_OPT_ATAN,		//�����У�֧�����ָ�ʽ��#i=ATAN[#j]   #i=ATAN[#j]/[#k]    #i=ATAN[#j,#k]
	MACRO_OPT_SQRT,		//ƽ����
	MACRO_OPT_POW,      //�˷�  #1=POW[#2,#3]
	MACRO_OPT_BIN,      //��BCD��תΪ��������   #1=BIN[#2]
	MACRO_OPT_BCD,      //�Ӷ�������תΪBCD��   #1=BCD[#2]
	MACRO_OPT_LN,		//��Ȼ����
	MACRO_OPT_EXP,		//ָ������
	MACRO_OPT_EQ,		//����
	MACRO_OPT_NE,		//������
	MACRO_OPT_GT,		//����
	MACRO_OPT_LT,		//С��
	MACRO_OPT_GE,		//���ڵ���
	MACRO_OPT_LE,		//С�ڵ���
	MACRO_OPT_OR,		//��
	MACRO_OPT_XOR,		//���
	MACRO_OPT_AND,		//��
	MACRO_OPT_MOD,   	//������  #i=#j MOD #k (#j��#kȡ������ȡ������#jΪ��ʱ��#iҲΪ��)

	//����
	MACRO_OPT_RD,       //��ȡ��#
	MACRO_OPT_WR,       //��ֵ
	MACRO_OPT_ADD,      //��
	MACRO_OPT_SUB,      //��
	MACRO_OPT_MULTIPLY, //��
	MACRO_OPT_DIVIDE,   //��


	//�������
	MACRO_OPT_LEFT_BRACKET,  //������
	MACRO_OPT_RIGHT_BRACKET, //������
	MACRO_OPT_VALUE,    //������

	MACRO_OPT_COMMA,    //���ţ�������ATAN/POW�����
	MACRO_OPT_GUARD		//����
};

//���������
enum MacroCmd{
	MACRO_CMD_INVALID = -1,  //�Ƿ�������
	MACRO_CMD_IF = 0,
	MACRO_CMD_THEN,
	MACRO_CMD_GOTO,		    //ǿ����ת
	MACRO_CMD_WHILE,
	MACRO_CMD_DO,
	MACRO_CMD_ELSE,
	MACRO_CMD_ELSEIF,
	MACRO_CMD_ENDIF,
	MACRO_CMD_END,          //ѭ��β��
	MACRO_CMD_IF_GOTO,		//������ת
	MACRO_CMD_IF_THEN,		//����ִ��
	MACRO_CMD_WHILE_DO,     //ѭ��
	MACRO_CMD_EXP,			//�����ʽ��
	MACRO_CMD_GUARD			//����
};

//G����ģʽ�鶨��
enum GModeIndex{
	GMODE_NONE = 0x00,              //��
	GMODE_00 = 0X01, 				//00��
	GMODE_01 = 0X02,				//01��
	GMODE_02 = 0X04,				//02��
	GMODE_03 = 0X08,
	GMODE_04 = 0X10,
	GMODE_05 = 0X20,
	GMODE_06 = 0X40,
	GMODE_07 = 0X80,
	GMODE_08 = 0X0100,
	GMODE_09 = 0X0200,
	GMODE_10 = 0X0400,
	GMODE_11 = 0X0800,
	GMODE_12 = 0X1000,
	GMODE_13 = 0X2000,
	GMODE_14 = 0X4000,
	GMODE_15 = 0X8000,
	GMODE_16 = 0X010000,
	GMODE_17 = 0X020000,
	GMODE_18 = 0X040000,
	GMODE_19 = 0X080000,
	GMODE_20 = 0X100000,
	GMODE_21 = 0X200000,
	GMODE_22 = 0X400000,
	GMODE_23 = 0X800000,
	GMODE_24 = 0X01000000,
	GMODE_25 = 0X02000000,
	GMODE_26 = 0X04000000,
	GMODE_27 = 0X08000000,
	GMODE_28 = 0X10000000,
	GMODE_29 = 0X20000000,
	GMODE_30 = 0X40000000,
	GMODE_31 = 0X80000000,
	GMODE_32 = 0X0100000000,
	GMODE_33 = 0X0200000000,		//33��
	GMODE_34 = 0X0400000000,		//34��
	GMODE_35 = 0X0800000000,		//35��
	GMODE_36 = 0x1000000000,		//36��
	GMODE_37 = 0x2000000000,		//37��
	GMODE_38 = 0x4000000000,        //38��
	GMODE_39 = 0x8000000000         //39��
};



//Gָ�����ݵ�ַ����
enum DataAddr{
	A_DATA = 0,
	B_DATA,
	C_DATA,
	D_DATA,
	E_DATA,
	F_DATA,
	G_DATA,
	H_DATA,
	I_DATA,
	J_DATA,
	K_DATA,
	L_DATA,
	M_DATA,
	N_DATA,
	O_DATA,
	P_DATA,
	Q_DATA,
	R_DATA,
	S_DATA,
	T_DATA,
	U_DATA,
	V_DATA,
	W_DATA,
	X_DATA,
	Y_DATA,
	Z_DATA
};

//������������
enum ProgramType{
	MAIN_PROG = 0,    	//������
	SUB_PROG,			//�ӳ���
	MACRO_PROG			//�����
};

//��ָ�Ԫ����������ʽ������϶���
struct MacroRec{
	MacroOpt opt;          //�����
    double value;      //����������ϸ˵������
    					//1. ���ڲ������䱣��ֵ��
    					//2. �������������ֵ����Ϊ0��
    					//3. ����ATAN/POW�������value=2��ʾΪ����������
    					//4. ����+��-��*��/��AND��MOD��OR��XOR��Щ�����������������������value=1��ʾ��ǰ��Ĳ��������и����ȼ�����ջ��

    MacroRec& operator=( const MacroRec& rec);  //��ֵ�����
    friend bool operator ==( const MacroRec &one, const MacroRec &two);  //�ж������
    friend bool operator !=(const MacroRec &one, const MacroRec &two);    //
};
typedef stack<MacroRec> MacroExpression;  //����ʽ����

void CopyMacroExp(MacroExpression &des, MacroExpression &src);   //������ֵ����ʽ��ջ
bool IsEqualMacroExp(MacroExpression &one, MacroExpression &two);   //�Ƚ���������ʽ�Ƿ�һ��

//һ��Gָ�����Ĵʷ����������
struct LexerGCode{
	void Reset();
	uint32_t mask_value;           //�����ֶ����룬���һ���г�������Щ���ݶΣ��ܼ�32bit��ʹ��ǰ26bit��������ĸ˳�����С�A=0,B=1,C=2,D=3......X=23,Y=24,Z=25��
	uint32_t mask_macro;           //�����Щ������Ϊ����ʽ
	uint32_t mask_dot;             //�����Щ�ֶε����ݴ�С����
	double value[kMaxTagCount];  //���������ֶε�ֵ������Ǻ���ʽ��洢macro_expression�е�������
	int g_value[kMaxGCodeInLine];   //��¼һ���г��ֵ�G����ֵ���˴���ŵ�G����Ŵ�ʮ���Լ���G43.4��ָ��,����Ǻ���ʽ��洢
								   //macro_expression�е������ŵĸ�ֵ,�����Ŵ�1��ʼ
	int m_value[kMaxMCodeInLine];   //��¼һ���г��ֵ�M����ֵ������Ǻ���ʽ��洢macro_expression�е������ŵĸ�ֵ,�����Ŵ�1��ʼ
	uint8_t gcode_count;          //һ���е�G�������
	uint8_t mcode_count;          //һ���е�M�������

	int t_value[kMaxTCodeInLine];    //��¼һ���г��ֵ�T����ֵ������Ǻ���ʽ��洢macro_expression�е������ŵĸ�ֵ,�����Ŵ�1��ʼ
	uint8_t tcode_count;          //һ����T����ĸ���

	double pos_value[kMaxAxisNameCount][16];    //���±����Ŀ��λ������,[X/Y/Z/A/B/C/U/V/W][1~9]
	uint16_t mask_pos[kMaxAxisNameCount];        //���±����Ŀ����������:X=0,Y=1,Z=2,A=3,B=4,C=5,U=6,V=7,W=8, BIT0~BIT15�����±�1~16
	uint16_t mask_pos_macro[kMaxAxisNameCount];  //�����Щ��չ������������Ϊ����ʽ
	uint16_t mask_dot_ex[kMaxAxisNameCount];             //�����Щ��������չ�ֶε����ݴ�С����

	MacroExpression macro_expression[kMaxMacroInLine];  //��¼����ʽ

};


//һ�к�ָ��ʷ��������,IF/WHILE/GOTO
struct LexerMacroCmd{
	void Reset();
	MacroCmd cmd;		//��ָ������
//	bool is_const;     //�Ƿ�������
	MacroExpression macro_expression[2];      //����ʽ
//	int data;          //���ݣ����ڱ���ѭ����Ż�����ת���
};

//NC����������
struct NcCode{
	NcCode(){}
	~NcCode(){}
	void Reset();
	LexerGCode gcode;          //Gָ����
	LexerMacroCmd macro_cmd;   //��ָ����
};

//һ�д���Ĵʷ��������
struct LexerResult{
	LexerResult(){}
	~LexerResult(){}
	void Reset();
	uint64_t line_no;         		//�к�
	uint64_t offset;   				//����ʼƫ�ƣ�����ѭ��ָ�����ת
	NcCode nc_code;				    //G����
	bool macro_flag;               //�Ƿ��ָ����,����IF/WHILE/GOTO�Ⱥ�ָ��
	ErrorType error_code;    		//������
};

//ģ̬���ϣ����й����������
struct ModeCollect{
	void Initialize();  //��ʼ������
	void Reset();        //��λ����
	ModeCollect& operator=( const ModeCollect& c);


	uint16_t gmode[kMaxGModeCount];   //G����ģ̬
	double f_mode;		//F����ģֵ̬   ��λ��mm/min
	double s_mode;		//S����ģֵ̬   ��λ��rpm
//	uint8_t move_mode;  //���ƶ�ģ̬��1��01��ģ̬  9:09��ģ̬
	uint8_t h_mode;		//H����ģֵ̬,���籣��
	uint8_t d_mode;		//D����ģֵ̬,���籣��
	uint8_t t_mode;		//T����ģֵ̬,���籣��


};



//����״̬��,���Ϲ����������еĸ���״̬������ģʽ�����ߡ����ᵱǰλ�õ�
struct CompileStatusCollect{
	ModeCollect mode;			//��ǰģ̬
	DPointChn cur_pos;             //��ǰ��λ��
	
	//�ٶȿ��ƺ����ؿ��Ʋ���Ҫ��¼��ǰ��Ŀ���ٶȺ�Ŀ������
//#ifdef USES_SPEED_TORQUE_CTRL
//	DPoint cur_velocity;             //��ǰ���ٶ�
//	DPoint cur_torque;             //��ǰ������
//#endif

//	double cur_feed;			//��ǰ�����ٶ� ��λmm/min

	bool exact_stop_flag;    	//׼ͣ��־   false-��׼ͣ     true--׼ͣ
	bool jump_flag;        		//�Ƿ��������


	CompileStatusCollect& operator=( const CompileStatusCollect& c);
};

/**
* @brief��AS�ļ���Ϣ�ṹ�������˶Գ����ļ���֧��
*/
struct AsFileMapInfo
{
    AsFileMapInfo()
    {
    	Clear();
    }
    ~AsFileMapInfo()
    {
//        if(ptr_map_file){
//        //	printf("unmap file:0x%x\n", ptr_map_file);
//            munmap(ptr_map_file, ln_map_blocksize);   //ȡ��ӳ��
//			ptr_map_file = nullptr;
//        }
//        if(fp_cur > 0)
//        	close(fp_cur);
    }
    void Clear();    //�ָ�Ĭ��ֵ�����ر��ļ�
    bool OpenFile(const char *name, bool sub_flag=false);  //���ļ�
    void CloseFile();    //�رյ�ǰ�ļ�
    void UnmapFile();		//�ر��ļ�ӳ��
    bool RemapFile();		//����ӳ���ļ�
    bool Swapdown();     //���·�ҳ
    bool Swapup();		   //���Ϸ�ҳ
    bool ResetFile();         //��λ����ǰ�ļ�ͷ
    bool JumpTo(uint64_t pos);   //����ǰ��ȡλ����ת��pos��
    AsFileMapInfo& operator=( const AsFileMapInfo& c);


    //�ļ�����
    int fp_cur;    //��ǰ�������ļ����
    uint64_t ln_file_size;   //�ļ���С
//    uint64_t ln_line_count;  //�ļ�������
    char str_file_name[kMaxPathLen];  //�ļ�����

    //�ڴ�ӳ��������
    uint64_t ln_map_start;      //ӳ�仺����ʼλ�ö�Ӧ�ļ�ƫ��
    uint64_t ln_map_blocksize;    //ӳ�������С
    //uint64_t ln_map_start_line;  //ӳ������ʼ�к�
    char *ptr_map_file;       //�ڴ�ӳ����ָ��
};

/**
 * @brief ��ǩλ�ü�¼
 */
struct LabelOffset{
	int label;		//��ǩ
	uint64_t offset;   //λ��ƫ��
	uint64_t line_no;	//�к�

	LabelOffset& operator=( const LabelOffset& c);
	bool operator ==( const LabelOffset &one){  //�ж������
		if(one.label == this->label)
			return true;
		return false;
	}
};
typedef ListBuffer<LabelOffset> LabelOffsetList;   //��ǩλ�ö���



/**
 * @brief �ӳ���ƫ��λ�ü�¼
 */
struct SubProgOffset{
	int sub_index;		//�ӳ����
	uint64_t offset;    //λ��ƫ��
	uint64_t line_no;	//�к�

	SubProgOffset& operator=( const SubProgOffset& c);
	bool operator ==( const SubProgOffset &one){  //�ж������
		if(one.sub_index == this->sub_index)
			return true;
		return false;
	}
};
typedef ListBuffer<SubProgOffset> SubProgOffsetList;     //�ӳ���λ�ö���

/**
 * @brief IF ELSE OFFSET
 */
struct IfElseOffset{
	uint64_t offset;     //�ļ�ָ��ƫ��
	uint64_t line_no;	 //�к�
	uint16_t vec_index;  // �˽ڵ������� vector�е�����
	uint16_t node_index; //�˽ڵ��� ifelse �����е�λ��

	IfElseOffset& operator=( const IfElseOffset& c);
	bool operator ==( const IfElseOffset &one){  //�ж������
		if(one.offset == this->offset and one.line_no == this->line_no)
			return true;
		return false;
	}
};

typedef ListBuffer<IfElseOffset> IfElseOffsetList;


/**
 * @brief ѭ������ĩ��λ�ü�¼
 */
struct LoopRec{
	uint64_t start_offset;   //���λ��ƫ��
	uint64_t start_line_no;	//����к�
	uint64_t end_offset;	//����λ��ƫ��
	uint64_t end_line_no;	//����λ���к�

	LoopRec& operator=( const LoopRec& c);
	bool operator ==( const LoopRec &one){  //�ж������
		if(one.start_line_no == this->start_line_no &&
				one.end_line_no ==  this->end_line_no)
			return true;
		return false;
	}
};
typedef ListBuffer<LoopRec> LoopRecList;    //ѭ����λ�ö���

/**
 * @brief  ѭ�������ʼλ�ü�¼
 */
struct LoopOffset{
	uint8_t loop_index;   //DO��ţ����DO��ʡ����������Ϊ0�����Ϊ8
	uint64_t offset;	  //λ��ƫ��
	uint64_t line_no;    //�к�

	LoopOffset& operator=( const LoopOffset& c);
	bool operator ==( const LoopOffset &one){  //�ж������
		if(one.loop_index == this->loop_index &&
				one.line_no == this->line_no)
			return true;
		return false;
	}
};
typedef DataStack<LoopOffset> LoopOffsetStack;     //ѭ��λ�ö�ջ

#ifdef USES_WOOD_MACHINE
/**
 * @brief �����������M03/M04/S��λ��ƫ��
 */
struct SpindleStartOffset{
	int m_code;   //03:M03   4:M04
	int s_code;   //ת��
	uint64_t line_no;   //�к�
    bool jump_line;   //�Ƿ�������α�־
//	int exec_step;   //��ǰִ�в���

	SpindleStartOffset& operator=( const SpindleStartOffset& c);
	bool operator ==( const SpindleStartOffset &one){  //�ж������
		if(one.line_no == this->line_no &&
				one.m_code == this->m_code &&
				one.s_code == this->s_code &&
				one.jump_line == this->jump_line)
			return true;
		return false;
	}
};
typedef ListBuffer<SpindleStartOffset> SpindleStartList;    //��������λ�ö���
#endif

/**
 * @brief ������״̬����,���ڻ��������״̬������ģʽ�л�
 */
struct CompilerScene{
	uint64_t ln_cur_line_no;  //��ǰ�кţ���1��ʼ
	ProgramType n_sub_program;       //��ǰ������������
	bool b_eof;					//�����ļ�β
//	bool b_comment;           //��ǰ�Ƿ�ע��״̬,�������ע��
//	bool b_left;              //�Ƿ񣨣�ע����ʽ,���Ƿ��ҵ�������
	int n_sub_call_times;		//��ǰ�ӳ�����ô���

	CompileFileState file_state;   //����״̬
	CompileHeadState head_state;     //ͷ������״̬

	CompilerWorkMode work_mode;   //���빤��ģʽģʽ
	RecordMsg *p_last_move_msg;    //��ǰ�ѱ������������һ�����ƶ�ָ��


	//�ļ�����
	char *ptr_cur_file_pos;     //��ǰ�ļ���ȡλ��ָ��
	uint64_t ln_read_size;  //��ǰ�Ѷ�ȡ�ֽ���

	CompileStatusCollect compiler_status;  //������״̬����
	AsFileMapInfo file_map_info;    //�ļ�����

	LabelOffsetList list_label;	//�б�ǩ��ƫ��λ�ô洢����
	SubProgOffsetList list_subprog;	//���ļ��ڲ��ӳ���ƫ��λ�ô洢����
	LoopRecList list_loop;		//ѭ�������,��¼���ļ��ڵ�����ѭ������ʼλ��
	LoopOffsetStack stack_loop;     //ѭ��λ������

	/*****if else ���� ******/
	vector<IfElseOffsetList> ifelse_vector;  // ��������
	DataStack<ListNode<IfElseOffset>> stack_ifelse_node;  //����ifelseʱ�ڵ���ջ
	bool meet_else_jump;         // ��if ����Ϊ��  ������else elseif ��Ҫֱ����ת�� endif�С�
	DataStack<bool> stack_else_jump_record;
	/*****if else ���� ******/

#ifdef USES_WOOD_MACHINE
	SpindleStartList list_spd_start;   //���������������
#endif

	CompilerScene& operator=( const CompilerScene& c);

};

typedef DataStack<CompilerScene> CompilerSceneStack;   //��������������ջ

/**
 * @brief �����ֵ
 */
struct MacroVarValue{
	double value;  //����ֵ
	bool init;		//�Ƿ��ʼ����false--δ��ʼ��������ֵ   true--�ѳ�ʼ����valueֵ��Ч
	MacroVarValue &operator=(const MacroVarValue &v);  //��д��ֵ�����
	MacroVarValue(){value = 0, init = false;}   //���캯��
};


//��������,����������
struct CompilerSimData{
	int type;   //ָ�����ͣ� 00-G00Ŀ��λ�� 01--G01Ŀ��λ��  02--G02Ŀ��λ��   03--G03Ŀ��λ��
	int plane;            //��ǰƽ��    170--G17    180--G18    190--G19
	double target[3];    //�������ݣ�����������ΪԲ���뾶ʱ��pos[0]���ڴ�Ű뾶
	double center[3];    //Բ��
	double radius;       //�뾶, 0��ʾ������>0��ʾ�ӻ���<0��ʾ�Ż�
};

struct ToolRec
{
    int G41G42dir;  // 40  41  42
//    int offset;  // ���Ȳ���ֵ
	int D;       // D���
	int H;       // H���
	double Radius;  //�뾶����ֵ
	int plane;  // ����ƽ��
	double feed;    // �����ٶ�
};

#endif /* INC_COMPILER_COMPILER_DATA_H_ */
