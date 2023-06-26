/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file Compiler.h
 *@author gonghao
 *@date 2020/03/19
 *@brief ��ͷ�ļ�ΪG����������������
 *@version
 */

#ifndef INC_COMPILER_COMPILER_H_
#define INC_COMPILER_COMPILER_H_

//#include "global_include.h"
#include "compiler_data.h"
#include "lexer.h"
#include "parser.h"
#include "compile_message.h"
#include "parm_definition.h"
#include "tool_compensate.h"


class ChannelControl;  //ͨ������
class Variable;        //�����
/**
 * @brief NC���������
 */
class Compiler{
public://�����ӿ�
	Compiler(ChannelControl *chn_ctrl);
	virtual ~Compiler();

	void InitCompiler();       //��������ʼ��
//	void SetFileMap(AsFileMapInfo *pFile);  //�����ļ�ӳ�����
	void Reset();               //���������帴λ
	void ResetState();    //��λ������״̬

	void DoIdle();     //���д�����


	void SetMode(CompilerWorkMode mode);   	//���ñ��빤��ģʽ
	CompilerWorkMode GetMode(){return m_work_mode;}     //���ر���ģʽ

    ErrorType GetErrorCode(){return m_error_code;}   //���ش�����

	bool IsCompileOver(){return m_b_compile_over;}  //�����Ƿ�������


	bool SetCurPos(const DPoint &cur_pos);   //���õ�ǰ��λ��
	bool SetCurPos(const DPointChn &cur_pos);  //���õ�ǰ��λ��

	bool SetCurGMode();   //���³�ʼ��Gģ̬

	bool OpenFile(const char *file, bool sub_flag=false);		//�򿪴������ļ�
	bool OpenFileInScene(const char *file);   //�ڱ���ĳ����д��ļ�

	bool GetLineData();       //��ȡһ������

	bool CompileLine();   //����һ�д���
	bool RunMessage();    //ִ�б��������Msg

	bool CompileOver();   //�����������

	void RecycleCompile();		//ѭ�����룬����M99ָ��
	void GetCurNcFile(char *file);   //��ȡ��ǰ�ӹ���NC�ļ�����

	bool SaveScene();   //���������״̬
	bool ReloadScene(bool bRecPos = true);   //�ָ�������״̬

	bool IsExactStop(){return m_compiler_status.exact_stop_flag;}   //�Ƿ���׼ͣ״̬
	bool IsEndOfFile(){return m_b_eof;}    //�Ƿ񵽴��ļ�β
	bool IsPreScaning(){return compiler_lock;}  // �Ƿ�����Ԥɨ��
	bool IsSubProgram(){return (bool)m_n_sub_program;}   	//�Ƿ����ӳ�����

	void SetOutputMsgList(OutputMsgList *output_msg){m_p_tool_compensate->SetOutputMsgList(output_msg);}  //����ָ����Ϣ���������ָ��
	uint16_t GetCurPlane(){return m_compiler_status.mode.gmode[2];}    //��ȡ��ǰƽ��ģʽ

	void UnmapCurNcFile();	//�رյ�ǰ�ļ�ӳ��
	bool RemapCurNcFile();   //����ӳ��nc�ļ�

	bool IsBlockListEmpty(){return m_p_block_msg_list->GetLength()==0?true:false;}   //�ֿ�����Ƿ�Ϊ��
	bool RefreshBlockMovePos(DPointChn &pos);   //ͬ�����λ��


	void SetAxisNameEx(bool flag);   //������������չ�±�ʹ��

    void RefreshPmcAxis();

	void SetRestartPara(const uint64_t line, const uint8_t mode);    //���üӹ���λ����

	void SetSimulateMode(SimulateMode *mode){this->m_p_simulate_mode = mode;}   //���÷���ģʽ

	int CallMarcoProgWithNoPara(int macro_index, bool flag = true);    //�޲������ú����

	// @add zk
	double GetFValue(){
		return m_compiler_status.mode.f_mode;
	}

    // sub_name:�ӳ����
    // file_only:ֻ����ʵ���ļ�
    int FindSubProgram(int sub_name, bool file_only = false, uint8_t scanMode = 0);   //���Ҳ����ӳ���


    int getSubCallTimes(){ return m_n_sub_call_times;}
    bool needJumpUpper(){return isJumpUpper;}

    void setCompensationPos(const DPointChn &pos);

    void GetMacroSubProgPath(int macro_group, int macro_index, bool abs_path, char *name);   //��ȡ������·������

    bool m_n_cur_dir_sub_prog = false;    // �Ƿ�Ϊ��ǰĿ¼�µ��ӳ���

#ifdef USES_WOOD_MACHINE
	bool FindPreStartSpdCmd(uint64_t line_min , uint64_t line_max, SpindleStartOffset &spd_cmd);   //�����Ƿ���ڿ�Ԥ����������ָ��
#endif

private://˽�нӿ�
    static void *PreScanThread(void *args);  //G������ɨ�������̺߳���
	void PreScan();     //����Ԥɨ�裬�ҵ��ӳ���ͱ�ǩ��
	void PreScanLine1(char *buf, uint64_t offset, uint64_t line_no, bool &flag, LoopOffsetStack &loop_stack, CompilerScene *scene);  //Ԥɨ���һ�飬����������
	void PreScanLine2(char *buf, uint64_t offset, uint64_t line_no, bool &flag, CompilerScene *scene);  //Ԥɨ��ڶ��飬����������
	int GetPreScanLine(char *line, uint64_t &read_total, AsFileMapInfo &map);   //��ȡԤɨ���һ������


//	void SwapDown();          //���·�ҳ

	bool RunAuxMsg(RecordMsg *msg);    	//�����������и���ָ����Ϣ
	bool RunSubProgCallMsg(RecordMsg *msg);  //�������������ӳ������ָ����Ϣ
	bool RunLineMsg(RecordMsg *msg);   	//������������ֱ������ָ����Ϣ
	bool RunRapidMsg(RecordMsg *msg);  	//�����������п��ٶ�λָ����Ϣ
	bool RunArcMsg(RecordMsg *msg);  		//������������Բ������ָ����Ϣ
	bool RunCoordMsg(RecordMsg *msg);  	//����������������ϵָ��ָ����Ϣ
	bool RunToolMsg(RecordMsg *msg);  	//�����������е���ָ����Ϣ
	bool RunModeMsg(RecordMsg *msg);  	//������������һ��ģָ̬����Ϣ������������ģָ̬��
	bool RunFeedMsg(RecordMsg *msg);  	//�����������н����ٶ�ָ��ָ����Ϣ
	bool RunSpeedMsg(RecordMsg *msg);  	//����������������ת��ָ����Ϣ
	bool RunLoopMsg(RecordMsg *msg);  	//������������ѭ��ָ����Ϣ
	bool RunCompensateMsg(RecordMsg *msg);  //�����������е���ָ����Ϣ
	bool RunMacroMsg(RecordMsg *msg);     //�����������к�ָ����Ϣ
	bool RunErrorMsg(RecordMsg *msg);  	//�����������д���ָ����Ϣ
	bool RunPolarIntpMsg(RecordMsg *msg);		//�����������м�����岹��Ϣ
	bool RunClearCirclePosMsg(RecordMsg *msg); 	//������������������Ȧλ����Ϣ
	bool RunTimeWaitMsg(RecordMsg *msg); 	//��������������ʱ��Ϣ
	bool RunRefReturnMsg(RecordMsg *msg);		//�����������лزο�����Ϣ
	bool RunSkipMsg(RecordMsg *msg);		   //��������������ת��Ϣ	
	bool RunMacroProgCallMsg(RecordMsg *msg);  //�����������к�������ָ����Ϣ
	bool RunAutoToolMeasureMsg(RecordMsg *msg);   //�������������Զ��Ե�ָ����Ϣ
	bool RunM06Msg(RecordMsg *msg);       //�����������л���ָ����Ϣ


	bool JumpToMacroProg(int macro_index);   //��ת��ָ�������
	
#ifdef USES_SPEED_TORQUE_CTRL	
	bool RunSpeedCtrlMsg(RecordMsg *msg);    //�������������ٶȿ�����Ϣ
	bool RunTorqueCtrlMsg(RecordMsg *msg);    //���������������ؿ�����Ϣ
#endif	


	bool CreateErrorMsg(const ErrorType err, uint64_t line_no);   //������������Ϣ����������һ��


	bool ProcessHead();      //�������ͷ����Ϣ
	bool ProcessMain(); //�����������
	bool CompileHead();     //����ͷ����Ϣ
//	bool CompileMain();      //�����������
	bool ProcessJumpLine();  //��������

	bool GetHeadInfo();   //������ȡͷ���ĸ�����Ϣ��������·�ϲ���Ϣ


	bool DoLexer();    //���дʷ�����
	bool DoParser();   //�����﷨����
	bool GetNumber(char *ps);   //�Ӹ����ַ����õ���ֵ

	bool CheckHead();     //������ͷ���Ϸ���
	char *GetConditionJumpMask(char *ps);  //��ȡ���κ�
	char *GetSerialNo(char *ps);   //��ȡ˳���

//	char *GetGMessage(char *ps);   //����G������Ϣ
//	char *GetMMessage(char *ps);   //����M������Ϣ
//	char *GetSMessage(char *ps);   //����S������Ϣ
//	char *GetFMessage(char *ps);   //����F������Ϣ
//	char *GetTMessage(char *ps);   //����T������Ϣ

	bool IsNewLineChar(char *pc);  //�ж��Ƿ��з�

//	bool CallSubProgram(int sub_name);   //�����ӳ���
	bool ReturnFromSubProg();				//�ӳ��򷵻ش���

	int FindJumpLabel(int label, uint64_t &offset, uint64_t &line_no);    //������ת��λ��

	bool JumpToLoopHead(LoopOffset &loop);    //������ת��ѭ��ͷ��λ��

	bool JumpToLoopEnd(LoopOffset &loop);    //������ת����Ӧ��ENDָ��

	bool CheckJumpGoto(uint64_t line_src, uint64_t line_des);     //�����ת��Ŀ���еĺϷ���

	uint8_t MapAxisNameToParamIdx(uint8_t axis_name);   //��������ӳ��Ϊ����ID

	void SaveLoopParam(LoopMsg *loop);    //����ѭ��ָ��Ĳ���������� ����global������ȫ�ֱ������Ǿֲ�����
	void ResetLoopParam();                //��λѭ��ָ�������Ӧ�ĺ����#171~#195

	bool JumpLine(int line_no, uint64_t offset, MacroCmdMsg *tmp);

#ifdef USES_FIVE_AXIS_FUNC
	void ProcessFiveAxisRotPos(DPoint &tar, DPoint &src, uint32_t mask);    //��������������ת��ͽ�·������
	void ProcessFiveAxisRotPos(DPointChn &tar, DPointChn &src, uint32_t mask);    //��������������ת��ͽ�·������
#endif

    void ProcessRotateAxisPos(DPointChn &tar, DPointChn &src, uint32_t mask);

private://˽�г�Ա
	uint8_t m_n_channel_index;              //����ͨ����
	AsFileMapInfo *m_p_file_map_info;       //��ǰ�����ļ�
//	AsFileMapInfo *m_p_file_map_info_auto;   //AUTO�������ļ�
//	AsFileMapInfo *m_p_file_map_info_mda;	//MDA�������ļ�

	SCChannelConfig *m_p_channel_config;   //����ͨ������
//#ifdef USES_FIVE_AXIS_FUNC
//	FiveAxisConfig *m_p_five_axis_config;  //����ͨ������������
//#endif

	MCCommunication *m_p_mc_communication;   //MCͨѶ�ӿ�
	ChannelControl *m_p_channel_control;	//������ͨ������

//	ToolCompensate m_tool_compensate;  //�����������
	ToolCompensate *m_p_tool_compensate;  //�����������
	ToolCompensate *m_p_tool_compensate_auto;  //�����������
	ToolCompensate *m_p_tool_compensate_mda;  //�����������

	char m_line_buf[kMaxLineSize];     //��ǰ�����л���

	char m_c_end_line;  //G�����н�����־�ַ�

//	OutputMsgList *m_p_output_msg_list;   //�������MC��ָ����Ϣ����
	OutputMsgList *m_p_block_msg_list;	//����ֶα�־�Ļ������
	RecordMsg *m_p_last_move_msg;    //��ǰ�ѱ������������һ�����ƶ�ָ��

	OutputMsgList *m_p_block_msg_list_auto;	//AUTOģʽ����ֶα�־�Ļ������
	OutputMsgList *m_p_block_msg_list_mda;	//MDAģʽ����ֶα�־�Ļ������

	bool m_b_check;    //�Ƿ�����﷨���

	bool m_b_has_over_code;   //�Ƿ���M30/M02/M99�������ָ�û����Ҫ�澯

	uint64_t m_ln_cur_line_no;  //��ǰ�кţ���1��ʼ
//	uint64_t m_ln_cur_line_offset;   //��ǰ������������ļ�ͷ��ƫ��
	ProgramType m_n_sub_program;       //��ǰ�����������ͣ�0--������   1--�ӳ���   2--�����
	bool m_b_comment;           //��ǰ�Ƿ�ע��״̬,�������ע��
	bool m_b_left;              //�Ƿ񣨣�ע����ʽ,���Ƿ��ҵ�������
	int m_n_sub_call_times{0};		//��ǰ�ӳ�����ô���
	ErrorType m_error_code;    //������

//	CompilerMainState m_n_main_state;   //������ѭ��״̬
	CompileFileState m_n_compile_state;   //����״̬
	CompileHeadState m_n_head_state;     //ͷ������״̬
//	unsigned char m_n_line_state;      //

	SimulateMode *m_p_simulate_mode;  //����ģʽ

	bool isJumpUpper{false};
	bool isSubInSameFile{false};      // �ӳ�����ͬһ��������
	uint64_t ln_read_size;
	char * p_cur_file_pos;
	uint64_t ln_cur_line_no;

	//�ļ�����
	char *m_p_cur_file_pos;     //��ǰ�ļ���ȡλ��ָ��
	uint64_t m_ln_read_size;  //��ǰ�Ѷ�ȡ�ֽ���
	bool m_b_eof;         //�����ļ�β
	bool m_b_compile_over;     //���������־��������M30/M02

	LexerResult m_lexer_result;   //�ʷ��������
	CompileRecList *m_p_parser_result;    //��ǰ�﷨�����������ָ�룬����ģʽָ��auto�Ķ��л���mda�Ķ���

	CompileRecList *m_p_parser_res_auto;	//AUTOģʽ�﷨�����������
	CompileRecList *m_p_parser_res_mda;		//MDAģʽ�﷨�����������

	Lexer *m_p_lexer;         //�ʷ�������
	Parser *m_p_parser;       //�﷨������

	CompilerWorkMode m_work_mode;   //���빤��ģʽģʽ

	CompileStatusCollect m_compiler_status;  //������״̬����

	CompilerSceneStack m_stack_scene;   //����״̬����ջ

	pthread_t m_thread_prescan;   			//G����Ԥɨ���߳�
	LabelOffsetList *m_p_list_label;	//�б�ǩ��ƫ��λ�ô洢����
	SubProgOffsetList *m_p_list_subprog;	//���ļ��ڲ��ӳ���ƫ��λ�ô洢����
	LoopRecList *m_p_list_loop;			//ѭ������ĩ��λ�ü�¼���У�Ԥɨ��ʱ����
	LoopOffsetStack m_stack_loop;        //ѭ��λ�ô洢ջ������ʱʹ��

	/************Ԥɨ��**********************/
	// ��¼��ǰ�����IF���������е�����
	int m_node_vector_index = 0;
	// ��¼ vector<IfElseOffset> ����
	int m_node_vector_len = 0;
	// ��¼ELSE���ִ���
	int m_else_count_prescan = 0;
	// ������ ÿ��vector<IfElseOffset> ����һ��IF.......ENDIF��¼
	vector<vector<IfElseOffset>> m_node_vectors_vector;
	// ����Ƕ�׹�ϵʱ ��¼���vector<IfElseOffset>����������������ջ
	vector<int> m_stack_vector_index_prescan;
	// ��¼ÿ��IF�� ELSE ������ջ
	vector<int> m_stack_else_count_prescan;
	/***************����ʱ********************/
	// ����ʱ ��¼��ǰ����Ľڵ�ջ
	vector<IfElseOffset> m_node_stack_run;
	// ����ʱ ��¼�Ƿ�����else��תջ
	vector<int> m_else_jump_stack_run;
	// �� else ��ת��־λ
	bool m_b_else_jump = false;

#ifdef USES_WOOD_MACHINE
	SpindleStartList *m_p_list_spd_start;         //���������������

	int m_n_s_code_in_prescan;      //Ԥɨ���м�¼��ǰSָ��ֵ
#endif
	bool m_b_prescan_over;         //Ԥɨ�����
	bool m_b_breakout_prescan;     //�ж�Ԥɨ���̱߳�־
	bool m_b_prescan_in_stack;     //Ԥɨ��ջ�е��ļ�
	Variable *m_p_variable;        //�����ָ��
	bool m_b_axis_name_ex;         //������������չ�±�

    uint8_t m_n_restart_mode;     //�ӹ���λģʽ��0--�Ǽӹ���λ   1--�����ӹ���λ    2--���ټӹ���λ
    uint64_t m_n_restart_line;    //�ӹ���λĿ���к�
    char m_last_main_file[kMaxPathLen];        //�����صĳ���

    // @add zk
    bool compiler_lock = false;  // ��ֹ�ӳ������ ����������ɨ��ִ��
};


#endif /* INC_COMPILER_COMPILER_H_ */
