/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file Compiler_data.h
 *@author gonghao
 *@date 2020/04/13
 *@brief ��ͷ�ļ�����G������������ɵ�������Ϣ��InfoMessage
 *@version
 */

#ifndef INC_COMPILER_COMPILE_MESSAGE_H_
#define INC_COMPILER_COMPILE_MESSAGE_H_

//#include "global_include.h"
#include "compiler_data.h"
#include "comm_data_definition.h"

//��Ϣ���־����
struct RecordMsgFlagBits{
	uint16_t     last_rec:1;   		//bit0 �Ƿ�ͬ�����һ��ָ��
	uint16_t     axis_move:1;		//bit1 �Ƿ�������ƶ�ָ�1��ʾ�ƶ���0��ʾ���ƶ�
	uint16_t     step_flag:1;   	//bit2 ����ģʽ��ִ����ָ����Ƿ���ͣ��1��ʾ��ͣ��0��ʾ����ͣ
	uint16_t     wait_move_over:1;	//bit3 ��Ҫ�ȴ�ǰ����˶�ָ���˶���λ
	uint16_t	 eof_flag:1;		//bit4 ���������־
	uint16_t     jump_flag:1;		//bit5 ���α�־  0--��Ч   1--��Ч
	uint16_t     block_over:1;		//bit6 �ֿ������־   0--��Ч   1--��Ч
	uint16_t     rsvd:9;    		//bit7-bit15 	����
};

//��Ϣ���־�ṹ
typedef union _RecordMsgFlag{
	uint16_t    all;
	struct  	RecordMsgFlagBits bits;
}RecordMsgFlag;

//��־��ö�ٶ���
enum RecordFlag{
	FLAG_LAST_REC = 0x01,	//�Ƿ�ͬ�����һ��ָ��
	FLAG_AXIS_MOVE = 0x02,	//�Ƿ�������ƶ�ָ��
	FLAG_STEP = 0x04,		//������־,����ģʽ�Ƿ���Ҫ��ͣ
	FLAG_WAIT_MOVE_OVER = 0x08,	//�ȴ��˶���λ��־
	FLAG_EOF = 0x10,			//���������־
	FLAG_JUMP = 0x20,			//���α�־
	FLAG_BLOCK_OVER = 0x40,		//�ֿ������־,�����һ�����������ƶ�ָ��
	FLAG_POINT_MOVE = 0x80		//�Ƿ�㶯�ƶ����������յ��غ�
};

/**
 * @brief ָ����Ϣ��Ļ��࣬��������ָ����Ϣ���Ӵ�������
 */
class RecordMsg{
public:
	RecordMsg();
	virtual ~RecordMsg();

//	void AppendMsg(RecordMsg *msg);  	//�����µ���Ϣ
//	void RemoveSelf();                //������ժ������
//	RecordMsg *GetPreMsg(){return m_p_prev;}   //������һ����Ϣָ��
//	RecordMsg *GetNextMsg(){return m_p_next;}  //������һ����Ϣ��ָ��

    void SetMsgType(CodeMsgType type){m_n_type = type;}   //������Ϣ����
	CodeMsgType GetMsgType(){return m_n_type;}           //��ȡ��Ϣ����
	void SetLineNo(uint64_t line){m_n_line_no = line;}    //�����к�
	uint64_t GetLineNo(){return m_n_line_no;}             //��ȡ�к�

	bool CheckFlag(RecordFlag flag){return (m_n_flags.all & flag)?true:false;}  //��ȡ��Ӧ��־״̬
	void SetFlag(RecordFlag flag, bool value);   //���ö�Ӧ��־��״̬

	// @modified by zk
	RecordMsgFlag GetFlags(){return m_n_flags;}
	void SetFlags(RecordMsgFlag flags){m_n_flags = flags;}
    // @modified by zk

	bool IsNeedWaitMsg(){return (m_n_flags.all & FLAG_WAIT_MOVE_OVER)?true:false;}    //�ж��Ƿ�����Ҫ�ȴ����˶���λ����Ϣ
	bool IsEndMsg(){return (m_n_flags.all & FLAG_EOF)?true:false;}	//�ж��Ƿ�������ָ��
	bool IsMoveMsg(){return (m_n_flags.all & FLAG_AXIS_MOVE)?true:false;} //�ж��Ƿ����ƶ�ָ��
	bool HasJumpFlag(){return (m_n_flags.all & FLAG_JUMP)?true:false;} //�ж��Ƿ������������
	bool IsPointMsg(){return (m_n_flags.all & FLAG_POINT_MOVE)?true:false;}  //�Ƿ�㶯�ƶ���Ϣ
	bool IsLineLastMsg(){return (m_n_flags.all & FLAG_LAST_REC)?true:false;}  //�Ƿ�ͬ�е����һ��ָ��

	void SetFrameIndex(uint16_t index){this->m_n_frame_index = index;}   //��������˳���
	uint16_t GetFrameIndex(){return this->m_n_frame_index;}     //��ȡ����˳���

	virtual void Execute() = 0;		//ִ�к���
	virtual void GetData(void* rec) = 0;		//��ȡ����
	virtual void SetData(void* rec) = 0;		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag) = 0;   	//���ɴ������MC���˶��������ݰ�
	virtual int GetSimulateData(CompilerSimData &data){return 1;}    //���ɷ������ݰ�

	virtual void PrintString() = 0;   //���ڳ������

	RecordMsg& operator=( const RecordMsg& msg);  //��ֵ�����
	friend bool operator ==( const RecordMsg &one, RecordMsg &two);  //�ж������
protected:
	//	RecordMsg* m_p_next;  //��һ������
	//	RecordMsg* m_p_prev;  //��һ������
		CodeMsgType m_n_type;	//��Ϣ����
		uint64_t m_n_line_no;  //�к�
		RecordMsgFlag m_n_flags; //״̬��־���ϣ���bit��ʽ��ʡ�ռ�
		uint16_t m_n_frame_index;   //����˳��ţ�0~65535ѭ��
private:


};

/**
 * @brief ������Ϣ
 */
class ErrorMsg : public RecordMsg{
public:
	ErrorMsg(int err_code);  //���캯��
	ErrorMsg();        //���캯��
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	int GetErrorCode(){return m_error_code;}   //���ش������
	void SetErrorCode(int err){m_error_code = err;}   //���ô������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   //���ɴ������MC���˶��������ݰ�

//	void SetClearInfoFlag(bool flag){this->m_b_clear_info = flag;}   //������Ϣ�����־
//	bool GetClearInfoFlag(){return this->m_b_clear_info;}      //������Ϣ�����־

	void SetInfoType(int type){this->m_n_info_type = type;}   //������Ϣ����
	int GetInfoType(){return this->m_n_info_type;}   //������Ϣ����

	virtual void PrintString();   //���ڳ������

	ErrorMsg& operator=( const ErrorMsg& msg);  //��ֵ�����
	friend bool operator ==( const ErrorMsg &one, ErrorMsg &two);  //�ж������
private:
	int m_error_code;    //�������
	int m_n_info_type;        //��Ϣ���ͣ� 0��ʾ��ʾ��Ϣ��1��ʾ�澯��Ϣ
//	bool m_b_clear_info;      //�����ʾ��Ϣ

};

/**
 * @brief �ӹ���λ�����Ϣ
 */
class RestartOverMsg : public RecordMsg{
public:
	RestartOverMsg();  //���캯��
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������


	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	RestartOverMsg& operator=( const RestartOverMsg& msg);  //��ֵ�����
	friend bool operator ==( const RestartOverMsg &one, RestartOverMsg &two);  //�ж������
};

//@test zk ����D Hָ�����
//class




/**
 * @brief ģʽ��Ϣ
 */
class ModeMsg : public RecordMsg{
public:
	ModeMsg(int gcode);		//���캯��
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	int GetGCode(){return m_n_g_code;}   //����Gָ�����
	void SetLastGCode(int gcode_last){this->m_n_last_g_code = gcode_last;}   //������ʷģֵ̬
	int GetLastGCode(){return this->m_n_last_g_code;}    //��ȡ��ʷģֵ̬

	ToolRec GetAToolRec(){return this->m_tool_info;}
	void SetAToolRec(ToolRec tool){this->m_tool_info = tool;}

	ModeMsg& operator=( const ModeMsg& msg);  //��ֵ�����
	friend bool operator ==( const ModeMsg &one, ModeMsg &two);  //�ж������
protected:
	int m_n_g_code;   //Gָ����룬�Ŵ�ʮ���洢������G43.4�洢ֵΪ434
	int m_n_last_g_code;    //ǰһ��ģ̬��Ϊ��֧�����ַ�����������

	ToolRec m_tool_info;
};

/**
 * @brief �˶�ָ����Ϣ�������˶�ָ����̳��Դ�Msg
 */
class MoveMsg : public ModeMsg{
public:
	MoveMsg(int gcode);		//���캯��
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	void SetMachCoord(bool flag){this->m_b_mach_coord = flag;}   //���û�е����ϵ��־
	bool IsMachCoord(){return this->m_b_mach_coord;}     //�Ƿ��е����ϵ

	MoveMsg& operator=( const MoveMsg& msg);  //��ֵ�����
	friend bool operator ==( const MoveMsg &one, MoveMsg &two);  //�ж������

	DPointChn &GetTargetPos(){return m_point_target;}  //�����յ�����
	DPointChn &GetSourcePos(){return m_point_source;}  //�����������
	void SetTargetPos(DPointChn pos){ m_point_target = pos;}  //
	void SetSourcePos(DPointChn pos){ m_point_source = pos;}  //
	void setCancelG80(bool flag){cancel_g80 = flag;}
	bool NeedCancelG80(){return cancel_g80;};

protected:
	bool m_b_mach_coord;    //��ǰ�Ƿ��е����ϵ
	DPointChn m_point_target;   //Ŀ���
	DPointChn m_point_source;   //��ʼ��
	bool cancel_g80 = false;
};


/**
 * @brief ����ϵ������Ϣ������G92/G52/G53/G54~G59/G5401~G5499
 */
class CoordMsg : public ModeMsg{
public:
	CoordMsg(const DPointChn &pos, const DPointChn &src, int gcode, uint32_t axis_mask);    //���캯��
	virtual void Execute();   	//ִ�к���
	virtual void GetData(void *rec);   //��ȡ����
	virtual void SetData(void *rec);   //��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�
	virtual int GetSimulateData(CompilerSimData &data);    //���ɷ������ݰ�

	virtual void PrintString();    //���ڳ������

	uint8_t GetExecStep(){return m_n_exec_step;}		//��ȡ��ǰ����
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //����ִ�в���
	DPointChn &GetTargetPos(){return m_pos;}    //����Ŀ��λ��
//	void GetTargetPosEx(DPointChn &pos);     //��ȡ��ά����ʽ������ֵ
	uint32_t GetAxisMask(){return m_n_axis_mask;}    //����������

	CoordMsg &operator=(const CoordMsg &msg);   //��ֵ�����
	friend bool operator ==( const CoordMsg &one, CoordMsg &two);  //�ж������
protected:
	DPointChn m_pos;   //��ǰλ�����¶��������ϵ�µ����ֻ꣬��Բ岹��
	DPointChn m_pos_src;   //�������
	uint32_t m_n_axis_mask;  //ָ��������
	uint8_t m_n_exec_step;  //ִ�н׶α�־

};



/**
 * @brief �����ٶ���Ϣ
 */
class FeedMsg : public RecordMsg{
public:
	FeedMsg(double feed, double last_feed);		//���캯��
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	double GetFeed(){return m_df_feed;}   //���ؽ����ٶ�

	FeedMsg& operator=( const FeedMsg& msg);  //��ֵ�����
	friend bool operator ==( const FeedMsg &one, FeedMsg &two);  //�ж������
protected:
	double m_df_feed;   //�����ٶȣ���λ:mm/min
	double m_df_last_feed;   //��һ��ָ�������ٶȣ� Ϊ��֧�����ַ�����������
};

/**
 * @brief ����ת����Ϣ
 */
class SpeedMsg : public RecordMsg{
public:
	SpeedMsg(double speed);		//���캯��
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	double GetSpeed(){return m_df_speed;}   //��������ת��

	SpeedMsg& operator=( const SpeedMsg& msg);  //��ֵ�����
	friend bool operator ==( const SpeedMsg &one, SpeedMsg &two);  //�ж������
protected:
	double m_df_speed;   //����ת�٣���λ:rpm
    double m_df_last_speed;    //��һ��ָ��������ת�٣�Ϊ��֧�����ַ�����������
};

/**
 * @brief ������Ϣ
 */
class ToolMsg : public RecordMsg{
public:
	ToolMsg(int *tool, uint8_t total);		//���캯��
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	uint16_t GetTool(uint8_t index){return m_n_tool[index];}  //���ص���
	uint8_t GetCount(){return m_n_tool_count;}   //��ȡTָ�����

	bool IsFirstExec();    //�Ƿ��״�ִ�д�����

	uint8_t GetExecStep(uint8_t index){return index<m_n_tool_count?m_n_tool_exec_segment[index]:-1;}		//��ȡ��ǰ���裬ʧ�ܷ���-1
	void SetExecStep(uint8_t index, uint8_t step){if(index < m_n_tool_count) m_n_tool_exec_segment[index] = step;} //����ִ�в���
	void IncreaseExecStep(uint8_t index){if(index<m_n_tool_count) m_n_tool_exec_segment[index]++;}   //�����Լ�

	int GetSubProgIndex(){return m_n_sub_prog_name;}  //�����ӳ����
    //void GetSubProgName(char *name, bool abs_path);	 //�����ӳ�����

	void SetSubProgType(uint8_t type){m_n_sub_prog_type = type;} //�����ӳ�������
	uint8_t GetSubProgType(){return m_n_sub_prog_type;}   	//�����ӳ�������

	uint16_t GetLastTool(){return this->m_n_last_tool;}   //������һ�ѵ���
	void SetLastTool(uint16_t last_tool){this->m_n_last_tool = last_tool;}   //������һ�ѵ���

	ToolMsg& operator=( const ToolMsg& msg);  //��ֵ�����
	friend bool operator ==( const ToolMsg &one, ToolMsg &two);  //�ж������
protected:
	uint16_t m_n_tool[kMaxTCodeInLine];   //����

	uint16_t m_n_last_tool;   //��һ�ѵ���

	int m_n_sub_prog_name;   //�ӳ�����
	uint8_t m_n_sub_prog_type;  //�ӳ�������  1--�ڱ�������   2--ͬĿ¼��nc�ļ�    3--ϵͳ�ӳ���Ŀ¼nc�ļ�     4--ͬĿ¼��iso�ļ�    5--ϵͳ�ӳ���Ŀ¼iso�ļ�

	uint8_t m_n_tool_count;   //Tָ������
	uint8_t m_n_tool_exec_segment[kMaxTCodeInLine];   //Tָ��ִ�н׶�   0--�޸Ĵ������ִ���߳�״̬   1--��T���뷢�͸�PMC    2--�ȴ�PMC����ִ����ɱ�־   0xFF--��ʾִ�н���
};

/**
 * @brief ��ʱ��Ϣ������G04
 */
class TimeWaitMsg : public ModeMsg{
public:
	TimeWaitMsg(uint32_t time);    //���캯��
	virtual void Execute();   	//ִ�к���
	virtual void GetData(void *rec);   //��ȡ����
	virtual void SetData(void *rec);   //��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();    //���ڳ������

	uint32_t GetDelayTime(){return m_n_time_delay;}		//��ȡ��ʱʱ��
	void SetStartTime(uint64_t start){ m_n_start_time = start;} 			//������ʼʱ��
	uint64_t GetStartTime(){return m_n_start_time;}      //���ص�ǰ��ʼʱ��

	TimeWaitMsg &operator=(const TimeWaitMsg &msg);   //��ֵ�����
	friend bool operator ==( const TimeWaitMsg &one, TimeWaitMsg &two);  //�ж������
private:
	uint32_t m_n_time_delay;   //��ʱʱ�䣬��λms
	uint64_t m_n_start_time;  //��ʱ��ʼʱ��,��λms��

};

/**
 * @brief �ο��㷵����Ϣ������G27 G28��G29��G30
 */
class RefReturnMsg : public ModeMsg{
public:
	RefReturnMsg(int gcode, uint32_t axis_mask, DPointChn &mid);    //���캯��
	virtual void Execute();   	//ִ�к���
	virtual void GetData(void *rec);   //��ȡ����
	virtual void SetData(void *rec);   //��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�
	virtual int GetSimulateData(CompilerSimData &data);    //���ɷ������ݰ�

	virtual void PrintString();    //���ڳ������

	uint8_t GetExecStep(){return m_n_exec_step;}		//��ȡ��ǰ����
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //����ִ�в���
	uint32_t GetAxisMask(){return m_n_axis_mask;} 	//������mask
	DPointChn &GetMiddlePos(){return m_pos_middle;}     //�����м������
	RefReturnMsg &operator=(const RefReturnMsg &msg);   //��ֵ�����
	friend bool operator ==( const RefReturnMsg &one, RefReturnMsg &two);  //�ж������

	double ref_id;      // �صڼ����ο���  �� G30 PX ��

private:
	uint32_t m_n_axis_mask;   //��ǻزο�����ᣬͨ����
	DPointChn m_pos_middle;  //�м��λ��
	uint8_t m_n_exec_step;  //ִ�н׶α�־

};


/**
 * @brief M����ָ����Ϣ
 */
class AuxMsg : public RecordMsg{
public:
	AuxMsg(int mcode);    //��Mָ��캯��
	AuxMsg(int *mcode, uint8_t total);		//���캯��
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	int GetMCode(uint8_t index=0){return index<m_n_m_count?m_n_m_code[index]:-1;}   //����M����ֵ,ʧ�ܷ���-1
	uint8_t GetMCount(){return m_n_m_count;}   //����M�������

	uint8_t GetExecStep(uint8_t index){return index<m_n_m_count?m_n_aux_exec_segment[index]:-1;}		//��ȡ��ǰ���裬ʧ�ܷ���-1
	void SetExecStep(uint8_t index, uint8_t step){if(index < m_n_m_count) m_n_aux_exec_segment[index] = step;} //����ִ�в���
	void IncreaseExecStep(uint8_t index){if(index<m_n_m_count) m_n_aux_exec_segment[index]++;}   //�����Լ�

	bool IsFirstExec();    //�Ƿ��״�ִ�д�����

	AuxMsg& operator=( const AuxMsg& msg);  //��ֵ�����
	friend bool operator ==( const AuxMsg &one, AuxMsg &two);  //�ж������
protected:
	int m_n_m_code[kMaxMCodeInLine];   //Mָ�����
//	uint8_t m_n_m_index;		//Mָ��˳��ţ�һ�����3��Mָ���1-3
	uint8_t m_n_m_count;      //Mָ�����
	uint8_t m_n_aux_exec_segment[kMaxMCodeInLine];   //����ָ��ִ�н׶�   0--�޸Ĵ������ִ���߳�״̬   1--��M���뷢�͸�PMC    2--�ȴ�PMC����ִ����ɱ�־
};

/**
 * @brief M98�ӳ������ָ����Ϣ
 */
class SubProgCallMsg : public AuxMsg{
public:
    SubProgCallMsg(int pcode, int lcode, uint8_t scan = 0);    //���캯��

	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	int GetSubProgIndex(){return m_n_sub_prog_name;}  //�����ӳ����
    //void GetSubProgName(char *name, bool abs_path);					//�����ӳ�����
	int GetCallTimes(){return m_n_call_times;}		//���ص��ô���

	void SetSubProgType(uint8_t type){m_n_sub_prog_type = type;} //�����ӳ�������
	uint8_t GetSubProgType(){return m_n_sub_prog_type;}   	//�����ӳ�������

	void SetLastProgFile(char *file);   //������һ���ļ��ľ���·�����������ַ�������
	void GetLastProgFile(char *file);   //��ȡ��һ���ļ��ľ���·�����������ַ�������

    uint8_t GetScanMode() const;

	SubProgCallMsg& operator=( const SubProgCallMsg& msg);  //��ֵ�����
	friend bool operator ==( const SubProgCallMsg &one, SubProgCallMsg &two);  //�ж������
private:
	int m_n_sub_prog_name;   //�ӳ�����
	int m_n_call_times;     //���ô���
	uint8_t m_n_sub_prog_type;  //�ӳ�������  1--�ڱ�������   2--ͬĿ¼��nc�ļ�    3--ϵͳ�ӳ���Ŀ¼nc�ļ�     4--ͬĿ¼��iso�ļ�    5--ϵͳ�ӳ���Ŀ¼iso�ļ�
    uint8_t m_n_scan_mode = 0;//�ӳ�����ҹ��� 0--> (1.sys_sub����, 2.mac_sub����)  1--> (1.��Ŀ¼����, 2.sys_sub����, 3.mac_sub����)

	char m_str_last_prog_file[kMaxPathLen];   //ǰһ���ļ�����·�������ڷ�������
};

/**
 * @brief G65��������ָ����Ϣ
 */
class MacroProgCallMsg : public ModeMsg{
public:
    MacroProgCallMsg(int pcode, int lcode, double *param, uint8_t count, uint32_t mask, uint8_t scan = 0);    //���캯��
	~MacroProgCallMsg();  //��������

	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	int GetMacroProgIndex(){return m_n_macro_prog_name;}  //�����ӳ����
    //void GetMacroProgName(char *name, bool abs_path);					//�����ӳ�����
	int GetCallTimes(){return m_n_call_times;}		//���ص��ô���
	double *GetParameter(uint32_t &mask, uint8_t &count);     //���ز�������


	void SetMacroProgType(uint8_t type){m_n_macro_prog_type = type;} //���ú��������
	uint8_t GetMacroProgType(){return m_n_macro_prog_type;}   	//���غ��������

	void SetLastProgFile(char *file);   //������һ���ļ��ľ���·�����������ַ�������
	void GetLastProgFile(char *file);   //��ȡ��һ���ļ��ľ���·�����������ַ�������

    uint8_t GetScanMode() const;

	MacroProgCallMsg& operator=( const MacroProgCallMsg& msg);  //��ֵ�����
	friend bool operator ==( const MacroProgCallMsg &one, MacroProgCallMsg &two);  //�ж������
private:
	int m_n_macro_prog_name;   //�������
	int m_n_call_times;     //���ô���
	double *m_p_df_param;   //���ò���
	uint32_t m_mask_param;   //��Ч����MASK
	uint8_t m_n_param_count;  //��������
	uint8_t m_n_macro_prog_type;  //�ӳ�������  1--�ڱ�������   2--ͬĿ¼��nc�ļ�    3--ϵͳ�ӳ���Ŀ¼nc�ļ�     4--ͬĿ¼��iso�ļ�    5--ϵͳ�ӳ���Ŀ¼iso�ļ�
    uint8_t m_n_scan_mode = 0;//�������ҹ��� 0--> (1.sys_sub����, 2.mac_sub����)  1--> (1.��Ŀ¼����, 2.sys_sub����, 3.mac_sub����)

	char m_str_last_prog_file[kMaxPathLen];   //ǰһ���ļ�����·�������ڷ�������
};

/**
 * @brief �ӳ��򷵻���Ϣ
 */
class SubProgReturnMsg : public AuxMsg{
public:
	SubProgReturnMsg(char *file_name, bool ret_macro);    //���캯��

	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	void GetReturnFileName(char *file){strcpy(file, m_str_file_name);}  //�����ϲ��������
	bool IsRetFromMacroProg(){return this->m_b_ret_from_macroprog;}    //�Ƿ�Ӻ��������з���

	SubProgReturnMsg& operator=( const SubProgReturnMsg& msg);  //��ֵ�����
	friend bool operator ==( const SubProgReturnMsg &one, SubProgReturnMsg &two);  //�ж������
private:
	char m_str_file_name[kMaxFileNameLen];   //�ӳ�����
	bool m_b_ret_from_macroprog;        //�Ƿ�Ӻ���򷵻�

};


/**
 * @brief ѭ��ָ����Ϣ
 */
class LoopMsg : public ModeMsg{
public:
	LoopMsg(const int gcode, double *param, uint8_t count, uint32_t mask);   //���캯��
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	int GetMacroProgIndex();  //�����ӳ����
    //void GetMacroProgName(char *name, bool abs_path);					//�����ӳ�����
	double *GetParameter(uint32_t &mask, uint8_t &count);     //���ز�������


	void SetMacroProgType(uint8_t type){m_n_macro_prog_type = type;} //���ú��������
	uint8_t GetMacroProgType(){return m_n_macro_prog_type;}   	//���غ��������

	void SetLastProgFile(char *file);   //������һ���ļ��ľ���·�����������ַ�������
	void GetLastProgFile(char *file);   //��ȡ��һ���ļ��ľ���·�����������ַ�������

	LoopMsg& operator=( const LoopMsg& msg);  //��ֵ�����
	friend bool operator ==( const LoopMsg &one, LoopMsg &two);  //�ж������

private:
	double *m_p_df_param;   //���ò���
	uint32_t m_mask_param;   //��Ч����MASK
	uint8_t m_n_param_count;  //��������
	uint8_t m_n_macro_prog_type;  //�ӳ�������  1--�ڱ�������   2--ͬĿ¼��nc�ļ�    3--ϵͳ�ӳ���Ŀ¼nc�ļ�     4--ͬĿ¼��iso�ļ�    5--ϵͳ�ӳ���Ŀ¼iso�ļ�

	char m_str_last_prog_file[kMaxPathLen];   //ǰһ���ļ�����·�������ڷ�������
};

/**
 * @brief G00���ٶ�λ��Ϣ
 */
class RapidMsg : public MoveMsg{
public:
	RapidMsg(const DPointChn &source, const DPointChn &target, const uint32_t axis_mask);		//���캯��
	virtual ~RapidMsg();       //��������
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�
	virtual int GetSimulateData(CompilerSimData &data);    //���ɷ������ݰ�

	virtual void PrintString();   //���ڳ������
	
	uint32_t GetAxisMoveMask(){return m_axis_move_mask;}   //�����ƶ���mask

	void RefreshTargetPos(DPointChn &pos);	//ͬ��Ŀ��λ��

	void SetIoData(uint8_t data){this->m_io_data = data;}   //����IO����

    void SetPmcAxisCount(uint8_t count);   //����PMC����˶�����
    uint8_t GetPmcAxisCount(){return this->m_n_pmc_count;}   //��ȡ�˶�PMC����
//	double *GetPmcTarget(uint8_t &count, uint32_t &mask, bool &inc);   //��ȡPMC����˶�Ŀ������
	uint8_t GetExecStep(){return m_n_exec_step;}		//��ȡ��ǰ����
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //����ִ�в���

	RapidMsg& operator=( const RapidMsg& msg);  //��ֵ�����
	friend bool operator ==( const RapidMsg &one, RapidMsg &two);  //�ж������
private:
	uint32_t m_axis_move_mask;  //�ƶ���mask��bit0-bit15�ֱ��־ͨ����1-16���Ƿ����ƶ�ָ��
	uint8_t m_io_data;       //io������ݣ�0x00��ʾ��IO���   ��1��ʼ����Ч��Χ1~127

//	double *m_p_pmc_target;  //PMC���Ŀ��λ��
//	uint32_t m_pmc_move_mask;   //PMC�ƶ���mask
    uint8_t m_n_pmc_count{0};     //PMC�ƶ�������
//	bool m_b_inc_pos;          //PMC��Ŀ���ַ�Ƿ�����ģʽ
	uint8_t m_n_exec_step;     //ִ�н׶α�־
};

/**
 * @brief G01ֱ��������Ϣ
 */
class LineMsg : public MoveMsg{
public:
	LineMsg(const DPointChn &source, const DPointChn &target, const double feed, const uint32_t axis_mask);    //���캯��
	virtual ~LineMsg();
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�
	virtual int GetSimulateData(CompilerSimData &data);    //���ɷ������ݰ�

	virtual void PrintString();   //���ڳ������

	void SetFeed(const double feed){m_df_feed = feed;}  //���ý����ٶ�
	double GetFeed(){return m_df_feed;};    //��ȡ�����ٶ�
	
	uint32_t GetAxisMoveMask(){return m_axis_move_mask;}   //�����ƶ���mask

	void RefreshTargetPos(DPointChn &pos);	//ͬ��Ŀ��λ��

	void SetIoData(uint8_t data){this->m_io_data = data;}   //����IO����

    void SetPmcAxisCount(uint8_t pmc_mask);   //����PMC����˶�����
    uint8_t GetPmcAxisCount(){return this->m_n_pmc_count;}   //��ȡ�˶�PMC����
//	double *GetPmcTarget(uint8_t &count, uint32_t &mask, bool &inc);   //��ȡPMC����˶�Ŀ������
	uint8_t GetExecStep(){return m_n_exec_step;}		//��ȡ��ǰ����
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //����ִ�в���

	LineMsg& operator=( const LineMsg& msg);  //��ֵ�����
	friend bool operator ==( const LineMsg &one, LineMsg &two);  //�ж������

protected:
	double m_df_feed;		//�����ٶ�   ��λ��mm/min
	uint32_t m_axis_move_mask;  //�ƶ���mask��bit0-bit15�ֱ��־ͨ����1-16���Ƿ����ƶ�ָ��
	uint8_t m_io_data;       //io������ݣ�0x00��ʾ��IO���   ��1��ʼ����Ч��Χ1~127

    uint8_t m_n_pmc_count{0};     //PMC�ƶ�������
	uint8_t m_n_exec_step;     //ִ�н׶α�־
    //	double *m_p_pmc_target;  //PMC���Ŀ��λ��
    //	uint32_t m_pmc_move_mask;   //PMC�ƶ���mask
    //	bool m_b_inc_pos;          //PMC��Ŀ���ַ�Ƿ�����ģʽ
};

/**
 * @brief ��ת��Ϣ������G31
 */
class SkipMsg : public LineMsg{
public:
	SkipMsg(const DPointChn &source, const DPointChn &target, const double feed, const uint32_t axis_mask);    //���캯��
	virtual ~SkipMsg(){}    //��������
	virtual void Execute();   	//ִ�к���
	virtual void GetData(void *rec);   //��ȡ����
	virtual void SetData(void *rec);   //��������

//	virtual int GetOutputData(GCodeFrame *data, uint32_t mask);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();    //���ڳ������

	uint8_t GetExecStep(){return m_n_exec_step;}		//��ȡ��ǰ����
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //����ִ�в���


	SkipMsg &operator=(const SkipMsg &msg);   //��ֵ�����
	friend bool operator ==( const SkipMsg &one, SkipMsg &two);  //�ж������
private:
	uint8_t m_n_exec_step;  //ִ�н׶α�־

};

/**
 * @brief �Զ��Ե�
 * G37  ...
 */
class AutoToolMeasureMsg : public ModeMsg{
public:
	AutoToolMeasureMsg(const DPointChn &target, const uint32_t axis_mask, const int h_code, const int times);    //���캯��
	virtual ~AutoToolMeasureMsg(){}       //��������
	virtual void Execute();   	//ִ�к���
	virtual void GetData(void *rec);   //��ȡ����
	virtual void SetData(void *rec);   //��������

	virtual void PrintString();    //���ڳ������

	DPointChn &GetTarget(){return this->m_pos_target;}   //����Ŀ��λ��
	uint32_t GetAxisMoveMask(){return this->m_mask_axis_move;}   //�������ƶ�mask
	int GetTimes(){return this->m_n_times;}    //���ضԵ�����
	int GetHIndex(){return this->m_n_h_index;}   //����д���Hֵ��

	int GetMacroProgIndex(){return m_n_macro_prog_name;}  //���ض�Ӧ������
    //void GetMacroProgName(char *name, bool abs_path);	   //���غ������

	void SetMacroProgType(uint8_t type){m_n_macro_prog_type = type;} //���ú��������
	uint8_t GetMacroProgType(){return m_n_macro_prog_type;}   	//���غ��������

	AutoToolMeasureMsg &operator=(const AutoToolMeasureMsg &msg);   //��ֵ�����
	friend bool operator ==( const AutoToolMeasureMsg &one, AutoToolMeasureMsg &two);  //�ж������
private:
	DPointChn m_pos_target;    //Ŀ��λ��
	uint32_t m_mask_axis_move;   //���ƶ�mask
	int m_n_times;            //ִ�д���
	int m_n_h_index;          //д���Hֵ

	int m_n_macro_prog_name;   //������
	uint8_t m_n_macro_prog_type;  //�ӳ�������  1--�ڱ�������   2--ͬĿ¼��nc�ļ�    3--ϵͳ�ӳ���Ŀ¼nc�ļ�     4--ͬĿ¼��iso�ļ�    5--ϵͳ�ӳ���Ŀ¼iso�ļ�

};

/**
 * @brief ������Ϣ��G40/G41/G42/G43/G44/G49��
 */
class CompensateMsg : public LineMsg{
public:
	CompensateMsg(int gcode, uint16_t data, const DPointChn &source, const DPointChn &target, const uint32_t axis_mask, const double feed, uint8_t move_type = 0);		//���캯��
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�
	virtual int GetSimulateData(CompilerSimData &data);    //���ɷ������ݰ�

	virtual void PrintString();   //���ڳ������

	int GetCompValue(){return m_n_data;}  //��ȡ����ֵ
	void SetCompLastValue(int value){this->m_n_data_last = value;}   //������ʷ����ֵ
	int GetLastCompValue(){return m_n_data_last;}     //��ȡ��ʷ����ֵ

	uint8_t GetExecStep(){return m_n_exec_step;}		//��ȡ��ǰ����
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //����ִ�в���
    uint8_t GetMoveType(){return m_n_move_type;};       //�ƶ�����


	CompensateMsg& operator=( const CompensateMsg& msg);  //��ֵ�����
	friend bool operator ==( const CompensateMsg &one, CompensateMsg &two);  //�ж������
	int GetGcode(){return m_n_g_code;};

private:
	uint16_t m_n_data;   //���ݣ�����G41/G42ΪDֵ������G43/G44ΪHֵ
	uint16_t m_n_data_last;   //֮ǰ�����ݣ�����G41/G42ΪDֵ������G43/G44ΪHֵ���������ַ�������
	uint8_t m_n_move_type;   //0--��ʾG00�� 1--��ʾG01
	uint8_t m_n_exec_step;  //ִ�н׶α�־

};

/**
 * @brief G02/G03Բ����Ϣ
 */
class ArcMsg : public MoveMsg{
public:
	int arc_id;  // @test zk
	ArcMsg(int code,
			const DPointChn& source,
	        const DPointChn& target,
		    const DPointChn& center,
		    const double& radius,
			const double feed,
			const uint32_t axis_mask,
		    const int8_t dir_flag = -1 ,
			const int8_t major_flag = 1,
			const int8_t circle_flag = 0);

	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�
	virtual int GetSimulateData(CompilerSimData &data);    //���ɷ������ݰ�

	virtual void PrintString();   //���ڳ������

	void SetFeed(const double feed){m_df_feed = feed;}  //���ý����ٶ�
	double GetFeed(){return m_df_feed;};    //��ȡ�����ٶ�
	void SetPlaneMode(const uint16_t gmode2){m_gmode_2 = gmode2;}    //���üӹ�ƽ��ģ̬

	DPointChn &GetCenterPos(){return m_point_center;}  //�����������
	
	void SetCenterPos(DPointChn pos){m_point_center = pos;}  
	
	void getArcFlags(int8_t &dir, int8_t &major, int8_t &circle){dir=m_flag_direct,major=m_flag_major,circle=m_flag_circle;}    // ����Բ�Ľ� 
//	double getAngle(void);    // ����Բ�Ľ� 
	uint32_t GetAxisMoveMask(){return m_axis_move_mask;}   //�����ƶ���mask

	void RefreshTargetPos(DPointChn &pos);	//ͬ��Ŀ��λ��

	bool CalArcCenter(const DPointChn &src, const DPointChn &tar, const double &radius, const int8_t flag, const uint16_t gmode_2, DPointChn &center);  //����Բ����������

	void SetIoData(uint8_t data){this->m_io_data = data;}   //����IO����

	int getDirect(){return m_flag_direct;}

	void setI(double i){ i_number = i;}
	void setJ(double j){ j_number = j;}
	void setR(double r){ r_number = r;}
    double getI(){ return i_number;}
    double getJ(){ return j_number;}
    double getR(){ return r_number;}

	ArcMsg& operator=( const ArcMsg& msg);  //��ֵ�����
	friend bool operator ==( const ArcMsg &one, ArcMsg &two);  //�ж������
    void SetPmcAxisCount(uint8_t count);   //����PMC����˶�����
    uint8_t GetPmcAxisCount(){return this->m_n_pmc_count;}   //��ȡ�˶�PMC����

private:
	int8_t m_flag_direct;  	//�켣�����־��-1:clockwise,1:anticlockwise  //˳ʱ��(-1)����ʱ��(1)
	int8_t m_flag_major;   	//�Ż���־�� 1:�ӻ�   -1:�Ż�
	int8_t m_flag_circle;	//��Բ��־�� 0��Բ��	1����Բ
	DPointChn  m_point_center;	//Բ������
	double  m_df_radius;	//�뾶����λmm
	double m_df_feed;		//�����ٶ�   ��λ��mm/min
	uint32_t m_axis_move_mask;  //�ƶ���mask��bit0-bit15�ֱ��־ͨ����1-16���Ƿ����ƶ�ָ��
	uint8_t m_io_data;       //io������ݣ�0x00��ʾ��IO���   ��1��ʼ����Ч��Χ1~127
    uint16_t m_gmode_2;     //�ӹ�ƽ��ģ̬
	double i_number, j_number, r_number;
    uint8_t m_n_pmc_count{0};     //PMC�ƶ�������
};

/**
 * @brief G25/G26����ת�ټ����Ϣ
 */
class SpindleCheckMsg : public ModeMsg{
public:
	SpindleCheckMsg(int gcode, int p, int q, int r, int i);

	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	SpindleCheckMsg& operator=( const SpindleCheckMsg& msg);  //��ֵ�����
	friend bool operator ==( const SpindleCheckMsg &one, SpindleCheckMsg &two);  //�ж������

private:
	int m_delay_time;   //P����������Sָ�����ʱ���ת�٣���λ��ms
	int m_allowed_tolerance;   //Q������ָ��������ת�����ޣ���λ��%
	int m_warn_ratio;      //R�����������澯������ת�ٱ䶯�ʣ���λ��%
	int m_warn_speed;      //I�����������澯������ת�ٲ��λ��תÿ��

};

/**
 * @brief ��ָ����Ϣ
 */
class MacroCmdMsg : public RecordMsg{
public:
	MacroCmdMsg(LexerMacroCmd *macro);		//���캯��
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	int GetMacroCmd(){return m_macro_cmd;}   //����Gָ�����
	MacroExpression &GetMacroExp(int index){return m_macro_expression[index];}   //��ȡ����ʽ
	MacroVarValue &GetMacroExpResult(int index){return m_macro_exp_res[index];}   //��ȡ����ʽ���
	bool GetMacroExpCalFlag(int index){return this->m_b_exp_cal[index];}    //��ȡ����ʽ�����־
	void SetMacroExpCalFlag(int index, bool flag){this->m_b_exp_cal[index] = flag;}  //���ú���ʽ�����־
	void SetRunStep(uint8_t step){m_n_run_step = step;}   //����ִ�н��ȱ�־
	uint8_t GetRunStep(){return m_n_run_step;}   //��ȡִ�н��ȱ�־
	void SetOffset(uint64_t offset){m_ln_offset = offset;}    //�����е�ַƫ��
	uint64_t GetOffset(){return m_ln_offset;}    //��ȡ�е�ַ����

	MacroCmdMsg& operator=(MacroCmdMsg& msg);  //��ֵ�����
	friend bool operator ==(MacroCmdMsg &one, MacroCmdMsg &two);  //�ж������
protected:
	MacroCmd m_macro_cmd;   //��ָ������
	MacroExpression m_macro_expression[2];  //����ʽ
	MacroVarValue m_macro_exp_res[2];     //����ʽ������
	bool m_b_exp_cal[2];         //����ʽ�Ƿ��Ѿ�����
	uint8_t m_n_run_step;     //��ָ��ִ�еĽ��ȣ� 0--��ʾ������ʽ0    1--��ʾ������ʽ1
	uint64_t m_ln_offset;     //ָ��������ƫ�Ƶ�ַ
};

/**
 * @brief ������岹�����Ϣ
 */
class PolarIntpMsg : public ModeMsg{
public:
	PolarIntpMsg(int gcode, int p = 0, int r = 0, int l = 0, int q = 0, int x = 0, int y = 0, int i = 0, int j = 0);

	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	uint8_t GetExecStep(){return m_n_exec_step;}		//��ȡ��ǰ����
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //����ִ�в���
	int GetParamP(){return m_n_p;}
	int GetParamR(){return m_n_r;}
	int GetParamL(){return m_n_l;}
	int GetParamQ(){return m_n_q;}
	int GetParamX(){return m_n_start_x;}
	int GetParamY(){return m_n_start_y;}
	int GetParamI(){return m_n_vect_i;}
	int GetParamJ(){return m_n_vect_j;}

	PolarIntpMsg& operator=( const PolarIntpMsg& msg);  //��ֵ�����
	friend bool operator ==( const PolarIntpMsg &one, PolarIntpMsg &two);  //�ж������

private:
	int m_n_p;   	//P������G12.2ָ������ָ��ĥ��ָ��ƽ��ʱ�䣬��λ��ms
	int m_n_r;   	//R������G12.2ָ������ָ��ɰ�ְ뾶����
	int m_n_l;      //L��������δʹ��
	int m_n_q;      //Q��������δʹ��

	int m_n_start_x;   //ĥ�����X����, ��λ��um
	int m_n_start_y;	//ĥ�����Y���꣬��λ��um
	int m_n_vect_i;		//ĥ������ʸ��I���Ŵ�1000000��
	int m_n_vect_j;		//ĥ������ʸ��J���Ŵ�1000000��


	uint8_t m_n_exec_step;  //ִ�н׶α�־
};

/**
 * @brief ������Ȧλ����Ϣ
 */
class ClearCirclePosMsg : public ModeMsg{
public:
	ClearCirclePosMsg(int gcode, uint32_t axis_mask, int loop);		//���캯��
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	uint8_t GetExecStep(){return m_n_exec_step;}		//��ȡ��ǰ����
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //����ִ�в���
	int GetCircleMode(){return m_n_circle_loop;};
	uint32_t GetAxisMask(){return m_axis_mask;};


	ClearCirclePosMsg& operator=( const ClearCirclePosMsg& msg);  //��ֵ�����
	friend bool operator ==( const ClearCirclePosMsg &one, ClearCirclePosMsg &two);  //�ж������
private:
	uint32_t m_axis_mask;  //������
	int m_n_circle_loop;  //ģ

	uint8_t m_n_exec_step;  //ִ�н׶α�־
};

#ifdef USES_SPEED_TORQUE_CTRL
/**
 * @brief �ٶ�ָ����Ϣ��G1000/G1001��
 */
class SpeedCtrlMsg : public ModeMsg{
public:
	SpeedCtrlMsg(int gcode, double *target, const uint8_t count, const uint32_t axis_mask);    //���캯��
	virtual ~SpeedCtrlMsg();    //��������
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	double *GetTargetSpeed(){return m_p_target_speed;}  //�����յ�����

	void  ChangeUint();

	uint32_t GetAxisMoveMask(){return m_axis_move_mask;}   //������mask
	
	uint8_t GetExecStep(){return m_n_exec_step;}		//��ȡ��ǰ����
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //����ִ�в���

//	void SetPmcAxisData(double *target, uint8_t count, uint32_t axis_mask);   //����PMC����˶�����
	double *GetTargetValue(uint8_t &count, uint32_t &mask);   //��ȡPMC����˶�Ŀ������

	SpeedCtrlMsg& operator=( const SpeedCtrlMsg& msg);  //��ֵ�����
	friend bool operator ==( const SpeedCtrlMsg &one, SpeedCtrlMsg &two);  //�ж������

protected:
	double *m_p_target_speed;   //Ŀ���ٶ�

	uint32_t m_axis_move_mask;  //�˶���mask��bit0-bit31�ֱ��־ͨ����1-32���Ƿ����ƶ�ָ��

	uint8_t m_n_axis_count;     //�ƶ�������

	uint8_t m_n_exec_step;  //ִ�н׶α�־
};

/**
 * @brief ����ָ����Ϣ��G2000/G2001��
 */
class TorqueCtrlMsg : public ModeMsg{
public:
	TorqueCtrlMsg(int gcode, double *target, const uint8_t count, const uint32_t axis_mask, uint32_t time = 0);    //���캯��
	virtual ~TorqueCtrlMsg();  //��������
	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������

	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);   	//���ɴ������MC���˶��������ݰ�

	virtual void PrintString();   //���ڳ������

	double *GetTargetTorque(){return m_p_target_torque;}  //����Ŀ������

	uint32_t GetAxisMoveMask(){return m_axis_move_mask;}   //������mask

	uint8_t GetMoveAxisCount(){return m_n_axis_count;}   //�����˶�������

	uint32_t GetTimeoverValue(){return m_n_timeover;}   //���س�ʱʱ��
	struct timeval *GetStartTimePtr(){return &m_start_time;}  //������ʼʱ��ָ��

	uint8_t GetCheckCount(){return m_n_check_count;}     //�������ص���ȷ�ϼ���
	void ResetCheckCount(){m_n_check_count = 0;}         //�����ص���ȷ�ϼ�������Ϊ0
	void IncCheckCount(){m_n_check_count++;}        //����ȷ�ϼ�����һ

//	void SetPmcAxisData(double *target, uint8_t count, uint32_t axis_mask);   //����PMC����˶�����
	double *GetTargetValue(uint8_t &count, uint32_t &mask);   //��ȡĿ������

	void SetSpeedLmtValue(int16_t val){ m_axis_speed_lmt = val;}
	
	int16_t GetSpeedLmtValue(){return m_axis_speed_lmt;}
	
	uint8_t GetExecStep(){return m_n_exec_step;}		//��ȡ��ǰ����
	void SetExecStep(uint8_t step){m_n_exec_step = step;} //����ִ�в���

	TorqueCtrlMsg& operator=( const TorqueCtrlMsg& msg);  //��ֵ�����
	friend bool operator ==( const TorqueCtrlMsg &one, TorqueCtrlMsg &two);  //�ж������

protected:
	double *m_p_target_torque;   //Ŀ������
	uint32_t m_axis_move_mask;  //�˶���mask��bit0-bit31�ֱ��־ͨ����1-32���Ƿ����ٶ�ָ��
	uint32_t m_n_timeover;     //��ʱʱ�䣬��λ������   0-��ʾ�޳�ʱ
	int16_t m_axis_speed_lmt;  // ���ؿ���ʱ�����ٶ�����ֵ
	uint8_t m_n_exec_step;     //ִ�н׶α�־
	uint8_t m_n_axis_count;     //�ƶ�������

	uint8_t m_n_check_count;  //���ص���ȷ�ϼ���

	struct timeval m_start_time;    //����ִ����ʼʱ�䣬���ڳ�ʱ�ж�


};
#endif

/**
 * @brief ����ָ����Ϣ��G10��
 */
class InputMsg : public RecordMsg{
public:
	InputMsg();    //���캯��
	virtual ~InputMsg();  //��������

	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������
	//���ɴ������MC���˶��������ݰ�
	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);
	virtual void PrintString();   //���ڳ������
	InputMsg& operator=( const InputMsg& msg);  //��ֵ�����

	double LData;
	double PData;
	double RData;
	double HData;
	double DData;
	double XData;
	double YData;
	double ZData;
	double QData;

};

// ׼ͣ��Ϣ  G09
class ExactStopMsg: public RecordMsg{
public:
	ExactStopMsg(const DPointChn &source, const DPointChn &target, const uint32_t axis_mask);    //���캯��
	virtual ~ExactStopMsg();  //��������

	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������
	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);
	virtual void PrintString();   //���ڳ������
	ExactStopMsg& operator=( const ExactStopMsg& msg);  //��ֵ�����

	DPointChn &GetTargetPos(){return m_point_target;}  //�����յ�����
	DPointChn &GetSourcePos(){return m_point_source;}  //�����������
	uint32_t GetAxisMask(){return m_axis_mask;}

	DPointChn m_point_target;
	DPointChn m_point_source;
	uint32_t  m_axis_mask;
};

class OpenFileMsg :public RecordMsg{
public:
	OpenFileMsg();    //���캯��
	virtual ~OpenFileMsg();  //��������

	virtual void Execute();		//ִ�к���
	virtual void GetData(void* rec );	//��ȡ����
	virtual void SetData(void* rec);		//��������
	//���ɴ������MC���˶��������ݰ�
	virtual int GetOutputData(GCodeFrame *data, uint32_t mask, bool flag);
	virtual void PrintString();   //���ڳ������
	OpenFileMsg& operator=( const OpenFileMsg& msg);  //��ֵ�����
	double OData;
};



//class FiveAxisMsg:public ModeMsg{
//public:
//	FiveAxisMsg(int gcode); 		//���캯��
//
//private:
//
//};

typedef ListBuffer<RecordMsg *> CompileRecList;  //�﷨���������������
typedef ListBuffer<RecordMsg *> OutputMsgList;   //�������ָ����Ϣ����
typedef ListBuffer<RecordMsg *> ToolCompMsgList;   //������ָ����Ϣ����



#endif /* INC_COMPILER_COMPILE_MESSAGE_H_ */
