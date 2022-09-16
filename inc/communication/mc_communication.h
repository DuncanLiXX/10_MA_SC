/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file mc_communication.h
 *@author gonghao
 *@date 2020/06/15
 *@brief ��ͷ�ļ�ΪSC-MCͨѶ�������
 *@version
 */

#ifndef MC_COMMUNICATION_H_
#define MC_COMMUNICATION_H_
#include <stdint.h>
#include <pthread.h>
#include "list_buffer.h"
#include "comm_data_definition.h"
#include "hmi_shared_data.h"
#include "geometry_data.h"

class ChannelEngine;

//�Ĵ�������
#define MC_REGISTER_BASE		(0x43C00000)		//SC-MC���ݽ����Ĵ�������ַ

//��������ͨ��
#define MC_CMD_DOWN_FIFO_COUNT  (MC_REGISTER_BASE+0x00)	//FIFO����,
#define MC_CMD_DOWN_WRITE_OVER  (MC_REGISTER_BASE+0x04)	//SCд����ɱ�־�����λ��Ч��д��ǰ��0����ɺ���1
#define MC_CMD_DOWN_RESET  	 (MC_REGISTER_BASE+0x08)	//ȫ�ָ�λ�źţ����λ��Ч���͵�ƽ��Ч��0��Ч
#define MC_CMD_DOWN_DATA0	 	 (MC_REGISTER_BASE+0x0C)	//����֡����0�Ĵ���
#define MC_CMD_DOWN_DATA1	 	 (MC_REGISTER_BASE+0x10)	//����֡����1�Ĵ���
#define MC_CMD_DOWN_DATA2	 	 (MC_REGISTER_BASE+0x14)	//����֡����2�Ĵ���
#define MC_CMD_DOWN_DATA3	 	 (MC_REGISTER_BASE+0x18)	//����֡����3�Ĵ���
#define MC_CMD_DOWN_DATA4	 	 (MC_REGISTER_BASE+0x1C)	//����֡����4�Ĵ���

//��������Ӧ��ͨ��
#define MC_CMD_UP_FIFO_COUNT    (MC_REGISTER_BASE+0x30)		//FIFO����,
#define MC_CMD_UP_READ_REQ  	 (MC_REGISTER_BASE+0x34)		//SC�����ȡ�����־�����λ��Ч����ȡǰ��1����ɺ���0
#define MC_CMD_UP_DATA0	 	 (MC_REGISTER_BASE+0x38)		//����֡����0�Ĵ���
#define MC_CMD_UP_DATA1	 	 (MC_REGISTER_BASE+0x3C)		//����֡����1�Ĵ���
#define MC_CMD_UP_DATA2	 	 (MC_REGISTER_BASE+0x40)		//����֡����2�Ĵ���
#define MC_CMD_UP_DATA3	 	 (MC_REGISTER_BASE+0x44)		//����֡����3�Ĵ���
#define MC_CMD_UP_DATA4	 	 (MC_REGISTER_BASE+0x48)		//����֡����4�Ĵ���

//����״̬����ͨ����512bytes,n=0,1,ֻ֧����ͨ��
#define MC_STATUS_READ_REQ		 (MC_REGISTER_BASE+0x90)		//SC״̬��ȡ�����־�����λ��Ч����ȡǰ��1����ɺ���0
#define MC_STATUS_DATA_BASE	 (MC_REGISTER_BASE+0x94)        //MC״̬���ݻ���ַ
#define MC_STATUS_DATA(n)       (MC_STATUS_DATA_BASE+0x04*n)   //MC״̬���ݵ�n������(32bits)�ļĴ�����ַ��0<=n<kMaxMcStatusCount
#define MC_CUR_LINE_NO(n)		 (MC_STATUS_DATA_BASE+0x100+0x04*n) //��nͨ���ĵ�ǰָ���к�,n=0,1,ֻ֧����ͨ��
#define MC_CUR_COORD(n)         (MC_STATUS_DATA_BASE+0x108+0x02*n) //��nͨ���ĵ�ǰ����ϵ,n=0,1,ֻ֧����ͨ��
#define MC_CUR_TOOL_OFFSET(n)   (MC_STATUS_DATA_BASE+0x10C+0x02*n) //��nͨ���ĵ�ǰ��ƫ��,n=0,1,ֻ֧����ͨ��
#define MC_CUR_FEED(n)          (MC_STATUS_DATA_BASE+0x110+0x04*n) //��nͨ���ĵ�ǰ�����ٶ�,n=0,1,ֻ֧����ͨ��
#define MC_RATED_FEED(n)        (MC_STATUS_DATA_BASE+0x118+0x04*n) //��nͨ���ĸ��������ٶ�,n=0,1,ֻ֧����ͨ��
#define MC_WORK_MODE(n)		 (MC_STATUS_DATA_BASE+0x120+0x04*n)	//��nͨ��������ģʽ������ָ��Ĵ�����ַ����16bitΪ����ģʽ����16bitΪ����ָ��
#define MC_MODE_INFO(n)         (MC_STATUS_DATA_BASE+0x128+0x04*n) //��nͨ����ģ̬������Ϣ��n=0,1,ֻ֧����ͨ��
#define MC_AXIS_INTPOVER_MASK(n) (MC_STATUS_DATA_BASE+0x130+0x04*n) //��nͨ������岹��λmask��n=0,1,ֻ֧����ͨ��
#define MC_CUR_INTP_MODE(n)      (MC_STATUS_DATA_BASE+0x138+0x02*n) //��nͨ���ĵ�ǰ�����岹ģʽ��3-3��������5-5��������6-��б�棬7-G12.2����n=0,1,ֻ֧����ͨ��
#define MC_CUR_FRAME_INDEX(n)    (MC_STATUS_DATA_BASE+0x13C+0x02*n) //��nͨ���ĵ�ǰ��������֡����ţ�n=0,1,ֻ֧����ͨ��
#define MC_AUTO_BUF_DATA_COUNT(n) 	(MC_STATUS_DATA_BASE+0x160+0x02*n) //��nͨ�����Զ����ݻ�������,n=0,1,ֻ֧����ͨ��
#define MC_MDA_BUF_DATA_COUNT(n)   (MC_STATUS_DATA_BASE+0x170+0x02*n) //��nͨ����MDA���ݻ�������,n=0,1,ֻ֧����ͨ��
#define MC_SOFT_LIMIT_MASK(n)   (MC_STATUS_DATA_BASE+0x180+0x04*n)    //��nͨ���ĳ�����λ����Mask,n=0,1,ֻ֧����ͨ������16bitΪ������λ����16bitΪ������λ
#define MC_POS_ERR_MASK(n)      (MC_STATUS_DATA_BASE+0x188+0x02*n)    //��nͨ����λ��ָ��������mask,n=0,1,ֻ֧����ͨ��
#define MC_ERR_FLAG(n) 		     (MC_STATUS_DATA_BASE+0x1A0+0x04*n) 		//MCͨ�������־��, n=0,1,ֻ֧����ͨ��
//#define MC_BLOCK_RUN_OVER(n)   (MC_STATUS_DATA_BASE+0x150+0x04*n/2) //��nͨ���ķֿ�岹��λ��־
//#define MC_STEP_RUN_OVER(n)	 (MC_STATUS_DATA_BASE+0x140+0x04*n/2)	//��nͨ���ĵ��β岹��λ��־
#define MC_RUN_OVER_FLAG		 (MC_STATUS_DATA_BASE+0x140)	//��16bitΪ���β岹��λ��־����16bitΪ�ֿ�岹��λ��־������һ��bit��ʾһ��ͨ����
																//��bit0��bit15�ɵ͵���˳������
#define MC_BLOCK_RUN_OVER_MDA   (MC_STATUS_DATA_BASE+0x144)	//��16bitΪMDAģʽ�Ŀ鵽λ��־
#define MC_CHN_CUR_TOOL_LIFE(n) (MC_STATUS_DATA_BASE+0x148+0x04*n)    //��nͨ���ĵ�ǰ����������n=0,1,ֻ֧����ͨ��
//#define MC_MDA_DATA_COUNT		 (MC_STATUS_DATA_BASE+0x170)		//��nͨ����MDA���ݻ�������,n=0,1,ֻ֧����ͨ��
#define MC_AXIS_CUR_POS_L(n,m)	 (MC_STATUS_DATA_BASE+0x80*n+0x10*m) 	//��nͨ����m��ĵ�ǰ�岹λ�õ�32λ��ַ
#define MC_AXIS_CUR_POS_H(n,m)  (MC_AXIS_CUR_POS_L(n,m)+0x04)		//��nͨ����m��ĵ�ǰ�岹λ�ø�32λ��ַ
#define MC_AXIS_TAR_POS_L(n,m)	 (MC_STATUS_DATA_BASE+0x08+0x80*n+0x10*m) 	//��nͨ����m��ĵ�ǰ�岹Ŀ��λ�õ�32λ��ַ
#define MC_AXIS_TAR_POS_H(n,m)  (MC_AXIS_TAR_POS_L(n,m)+0x04)		//��nͨ����m��ĵ�ǰ�岹Ŀ��λ�ø�32λ��ַ

#define MC_DEBUG_DATA(n)        (MC_STATUS_DATA_BASE + 0x1C0+0x04*n)   //��ȡ��n��debug���ݣ�ÿ��Ϊһ��32λ��


//�����˶�����ͨ����֧��2ͨ����ÿͨ��120bytes
#define MC_GCODE_FIFO_COUNT(n)    (MC_REGISTER_BASE+0x02B0+0x1A0*n)		//FIFO����,n=0,1,ֻ֧����ͨ��
#define MC_GCODE_WRITE_OVER(n)  	(MC_REGISTER_BASE+0x02B4+0x1A0*n)		//SCд����ɱ�־�����λ��Ч��д��ǰ��0����ɺ���1,n=0,1,ֻ֧����ͨ��
#define MC_GCODE_DATA_BASE(n)		(MC_REGISTER_BASE+0x2B8+0x1A0*n)		//�˶����ݻ���ַ, n=0,1,ֻ֧����ͨ��
#define MC_GCODE_DATA(n,m)		(MC_GCODE_DATA_BASE(n)+0x04*m)		//��n���˶����ݵ�ַ�� n=0,1,ֻ֧����ͨ��, 0<=m<kMaxGCodeDataCount


#define PL_PROG_VERSION        (MC_REGISTER_BASE+0x07F0)       //PL��ǰ����汾

//Spartan6����ͨ��
#define SP6_INIT_DATA_FIFO     (MC_REGISTER_BASE+0x07D0)       //FIFO��ǰ�����������16
#define SP6_INIT_DATA(n)		  (MC_REGISTER_BASE+0x07D4+0x04*n)	//������������0<=n<=3
#define SP6_INIT_DATA_WRITE	  (MC_REGISTER_BASE+0x07E4)     //PSд��������ɣ�bit0Ϊ1��־д��������ɣ�PL���Զ�ȡ������
#define SP6_INIT_DATA_WR_TOTAL    (MC_REGISTER_BASE+0x07E8)		//��������д�����
#define SP6_INIT_DATA_CHECK_RES  (MC_REGISTER_BASE+0x07EC)     //����У����


//fpga�汾��Ϣ
#define PL_VERSION     (MC_REGISTER_BASE+0x076C)      //PL�汾
#define SP6_VERSION	(MC_REGISTER_BASE+0x0768)		//SPARTAN6�汾

//Ƿѹ�澯�Ĵ��������λ��Ч������Ч
#define UNDER_VOLTAGE_ALARM     (MC_REGISTER_BASE+0x07A4)    //Ƿѹ�澯�Ĵ���

typedef ListBuffer<McCmdFrame> McCmdBuffer;   //�������ָ����Ϣ����


/**
 * @brief SCģ����MCģ����ͨѶ�࣬����fpga��չEMIF�ӿ�
 */
class MCCommunication {
public:

	// @test zk
	void test(int16_t frame_index);

	virtual ~MCCommunication();	//��������

	static MCCommunication *GetInstance();   //����ģʽ����ȡ����ʵ����Ψһ���ʵ�

	void SetInterface();   //���ýӿ�

	//�����˶���������ͨ���ӿ�
	bool WriteGCodeData(uint8_t chn, GCodeFrame &data);   	//д��G�����������˶���������
	bool CanWriteGCode(uint8_t chn);            //�Ƿ���д���˶���������
	uint32_t ReadGCodeFifoCount(uint8_t chn);              //��ȡ�����˿�����FIFO�����ݸ���


	//����ͨ���ӿ�
	bool WriteCmd(McCmdFrame &data, bool resend = false); 	//д��ָ��
	bool ReadCmdRsp(McCmdFrame &data);    //��ȡָ��
	uint32_t ReadCmdFifoCount();        //��ȡ����ͨ��FIFO�е�ǰ���ݸ���
	uint32_t ReadCmdBufferLen();          //��ȡ��������ݸ���

	//��ȡ��岹λ��
	void ReadAxisIntpPos(const uint8_t chn_index, DPoint &cur_pos, DPoint &tar_pos);

	//��ȡͨ������ģʽ������ָ��
	void ReadWorkMode(const uint8_t chn_index, uint16_t &work_mode);

	//��ȡͨ����ǰ����ָ��
	void ReadCurCmd(const uint8_t chn_index, uint16_t &cur_cmd);

	//��ȡͨ����ǰ�����к�
	void ReadCurLineNo(const uint8_t chn_index, uint32_t &cur_lineno);

	//��ȡͨ����ǰʵ�ʽ����ٶ�
	void ReadCurFeed(const uint8_t chn_index, uint32_t &cur_feed);

	//��ȡͨ����ǰ���������ٶ�
	void ReadRatedFeed(const uint8_t chn_index, uint32_t &rated_feed);

	//��ȡͨ����ǰ�Զ����ݻ�������
	void ReadAutoBufDataCount(const uint8_t chn_index, uint16_t &count);

	//��ȡͨ����ǰMDA���ݻ�������
	void ReadMdaBufDataCount(const uint8_t chn_index, uint16_t &count);

	//��ȡͨ��ģ̬��Ϣ
	void ReadMcModeInfo(const uint8_t chn_index, uint32_t &mode_info);

	//��ȡAUTO�ֿ�岹��λ��־
	bool ReadAutoBlockRunOverFlag(const uint8_t chn_index);

	//��ȡ��λ��־�ֶ�
	uint32_t ReadRunOverValue();

	//��ȡ���β岹��λ��־
	bool ReadStepRunOverFlag(const uint8_t chn_index);

	//��ȡMDAģʽ�鵽λ��־
	bool ReadMdaBlockRunOverFlag(const uint8_t chn_index);

	//����ָ��ͨ���ᵽλ��־
	void ReadChnAxisRunoverMask(const uint8_t chn_index, uint32_t &axis_mask);

	//��ȡͨ����ǰ����ϵ
	void ReadChnCurCoord(uint8_t chn_index, uint16_t &coord);

	//��ȡͨ����ǰ��ƫ��
	void ReadChnCurToolOffset(uint8_t chn_index, uint16_t &tool_offset);

	//��ȡ��ǰ�����岹ģʽ
	void ReadChnCurIntpMode(uint8_t chn_index, uint16_t &intp_mode);

	//��ȡ��ǰMC�����־
	void ReadMcErrFlag(uint8_t chn_index, uint32_t &mc_err);

	//��ȡͨ���ĵ�ǰ����λ������mask
	void ReadChnSoftLimtMask(uint8_t chn_index, uint32_t &axis_mask);

	//��ȡͨ���ĵ�ǰλ��ָ�����澯��mask
	void ReadChnPosErrMask(uint8_t chn_index, uint16_t &axis_mask);

	//��ȡͨ���ĵ�ǰ����֡�����
	void ReadChnCurFrameIndex(uint8_t chn_index, uint16_t &frame_index);


	//��ȡͨ����ǰ�����ۼ�����
	void ReadChnCurToolLife(uint8_t chn_index, int32_t &tool_life);

	//��ȡZynq-pl�İ汾
	bool ReadPlVer();

	//��ȡsp6�İ汾
	bool ReadSp6Ver();

	//��ȡMC�ĵ�������
	void ReadDebugData(uint32_t *debug_data);

	//��ȡǷѹ�澯�ź�
	bool ReadUnderVoltWarn();

private:
	MCCommunication();    //���캯��

	static void *ProcessCmdThread(void *args); //����������̺߳���
	bool ProcessCmdFun();  	//�������

	bool Initialize();    //��ʼ������
	bool Clean();          //������

	void CalMcCmdCrc(McCmdFrame &cmd);     //����MC�������CRC
	void CalMcGCodeFrameCrc(GCodeFrame &data);   //����G��������֡��CRC

private://˽�б���
	static MCCommunication *m_p_instance;    //��ʵ������

	ChannelEngine *m_p_channel_engine;   //ͨ������ָ��

	McCmdBuffer *m_list_cmd;	//��������������

	int m_n_mem_file;           //�����ļ��򿪾��
	uint8_t *m_p_addr_base;		//����ַ

	ErrorType m_error_code;    //���һ�εĴ�����

	pthread_mutex_t m_mutex_cmd;   // ��������ͨ��������,д�������

	pthread_t m_thread_process_cmd;    //������߳�

	uint8_t m_n_read_status_sem;    //״̬��ȡ�ź���

	uint64_t m_n_crc_err_count;		//CRCУ������

};

#endif /* MC_COMMUNICATION_H_ */
