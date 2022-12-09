/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file mc_communication_arm.h
 *@author gonghao
 *@date 2021/11/22
 *@brief ��ͷ�ļ�ΪSC-MC-ARMͨѶ���������������SC��������ARM�ϵ�MCͨѶ
 *@version
 */

#ifndef MC_COMMUNICATION_ARM_H_
#define MC_COMMUNICATION_ARM_H_
#include <stdint.h>
#include <pthread.h>
#include "list_buffer.h"
#include "comm_data_definition.h"
#include "hmi_shared_data.h"
#include "geometry_data.h"

class ChannelEngine;

//����ͨ���������
#define MC_ARM_CMD_FIFO_DEPTH  (64)	//64������֡���

//����֡�ֽ���
#define MC_ARM_CMD_FRAME_SIZE (32)

//Gָ���˶����ݰ���С���ֽ���
#define MC_ARM_GCODE_FRAME_SIZE   (128)   //ÿ���˶����ݰ�128�ֽ�

//ÿ��ͨ�����˶�����FIFO���
#define MC_ARM_CHN_GCODE_FIFO_DEPTH   (16)   //ÿ��ͨ�����˶����ݻ������Ϊ16

//ÿ��ͨ�����˶����ݻ�����ռ�ֽ���
#define MC_ARM_CHN_GCODE_FIFO_SIZE   (MC_ARM_GCODE_FRAME_SIZE*MC_ARM_CHN_GCODE_FIFO_DEPTH)    //2k�ֽ�

//�Ĵ�������
#define MC_ARM_REGISTER_BASE		(0x1FE00000)		//SC-MC���ݽ��������ڴ����ַ

//��������ͨ��
#define MC_ARM_CMD_DOWN_BASE        (MC_ARM_REGISTER_BASE+0xB4000) //��������ͨ������ַ
#define MC_ARM_CMD_DOWN_WF(n)       (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n)	//��n������֡��д��־
#define MC_ARM_CMD_DOWN_RF(n)       (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x02)	//��n������֡�Ķ���־
#define MC_ARM_CMD_DOWN_DATA0(n)	 (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x04)	//��n������֡����0�Ĵ���
#define MC_ARM_CMD_DOWN_DATA1(n)	 (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x08)	//��n������֡����1�Ĵ���
#define MC_ARM_CMD_DOWN_DATA2(n)	 (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x0C)	//��n������֡����2�Ĵ���
#define MC_ARM_CMD_DOWN_DATA3(n)	 (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x10)	//��n������֡����3�Ĵ���
#define MC_ARM_CMD_DOWN_DATA4(n)	 (MC_ARM_CMD_DOWN_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x14)	//��n������֡����4�Ĵ���

//��������Ӧ��ͨ��
#define MC_ARM_CMD_UP_BASE          (MC_ARM_REGISTER_BASE+0xB4800)     //��������ͨ������ַ
#define MC_ARM_CMD_UP_WF(n)         (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n)		//��n������֡��д��־
#define MC_ARM_CMD_UP_RF(n)  	     (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x02)		//��n������֡�Ķ���־
#define MC_ARM_CMD_UP_DATA0(n)	 	 (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x04)		//��n������֡����0�Ĵ���
#define MC_ARM_CMD_UP_DATA1(n)	 	 (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x08)		//��n������֡����1�Ĵ���
#define MC_ARM_CMD_UP_DATA2(n)	 	 (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x0C)		//��n������֡����2�Ĵ���
#define MC_ARM_CMD_UP_DATA3(n)	 	 (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x10)		//��n������֡����3�Ĵ���
#define MC_ARM_CMD_UP_DATA4(n)	 	 (MC_ARM_CMD_UP_BASE+MC_ARM_CMD_FRAME_SIZE*n+0x14)		//��n������֡����4�Ĵ���

//����״̬����ͨ����512bytes,n=0,1,ֻ֧����ͨ��
#define MC_ARM_STATUS_BASE(n)       (MC_ARM_REGISTER_BASE+0xB5000+2048*n)     //����״̬ͨ������ַ��ÿ��ͨ��״̬ռ2K�ֽ�
#define MC_ARM_STATUS_POS_WF(n)	 (MC_ARM_STATUS_BASE(n))		    //��n��ͨ��λ�����ݵ�д��־
#define MC_ARM_STATUS_POS_RF(n)     (MC_ARM_STATUS_BASE(n) + 0x02)		//��n��ͨ��λ�����ݵĶ���־
#define MC_ARM_AXIS_CUR_POS(n,m)	 (MC_ARM_STATUS_BASE(n)+0x10*m+0x08) 	//��nͨ����m��ĵ�ǰ�岹λ�ã�m��0��ʼ
#define MC_ARM_AXIS_TAR_POS(n,m)	 (MC_ARM_STATUS_BASE(n)+0x10*m+0x10) 	//��nͨ����m��ĵ�ǰ�岹Ŀ��λ�ã�m��0��ʼ

#define MC_ARM_STATUS_DATA_BASE(n)	 (MC_ARM_STATUS_BASE(n)+1024)        //��nͨ��״̬���ݻ���ַ
#define MC_ARM_STATUS_WF(n)	 	 (MC_ARM_STATUS_DATA_BASE(n))		//��n��ͨ��״̬���ݵ�д��־
#define MC_ARM_STATUS_RF(n)     	 (MC_ARM_STATUS_DATA_BASE(n)+0x02)	//��n��ͨ��״̬���ݵĶ���־
#define MC_ARM_CUR_FRAME_INDEX(n)   (MC_ARM_STATUS_DATA_BASE(n)+0x04) //��nͨ���ĵ�ǰ��������֡����ţ�n=0,1,ֻ֧����ͨ��
#define MC_ARM_CUR_LINE_NO(n)		 (MC_ARM_STATUS_DATA_BASE(n)+0x08) //��nͨ���ĵ�ǰָ���к�,n=0,1,ֻ֧����ͨ��
#define MC_ARM_CUR_FEED(n)          (MC_ARM_STATUS_DATA_BASE(n)+0x0C) //��nͨ���ĵ�ǰ�����ٶ�,n=0,1,ֻ֧����ͨ��
#define MC_ARM_RATED_FEED(n)        (MC_ARM_STATUS_DATA_BASE(n)+0x10) //��nͨ���ĸ��������ٶ�,n=0,1,ֻ֧����ͨ��
#define MC_ARM_CUR_COORD(n)         (MC_ARM_STATUS_DATA_BASE(n)+0x14) //��nͨ���ĵ�ǰ����ϵ,n=0,1,ֻ֧����ͨ��
#define MC_ARM_CUR_TOOL_OFFSET(n)   (MC_ARM_STATUS_DATA_BASE(n)+0x18) //��nͨ���ĵ�ǰ��ƫ��,n=0,1,ֻ֧����ͨ��

#define MC_ARM_WORK_MODE(n)		   (MC_ARM_STATUS_DATA_BASE(n)+0x1C)	//��nͨ��������ģʽ������ָ��Ĵ�����ַ
#define MC_ARM_CUR_CMD(n)             (MC_ARM_STATUS_DATA_BASE(n)+0x20)    //��nͨ����ǰ����ָ�G00��G01��G02��G03
#define MC_ARM_MODE_INFO(n)           (MC_ARM_STATUS_DATA_BASE(n)+0x24)    //��nͨ����ģ̬������Ϣ��n=0,1,ֻ֧����ͨ��
#define MC_ARM_AXIS_INTPOVER_MASK(n)  (MC_ARM_STATUS_DATA_BASE(n)+0x28)    //��nͨ������岹��λmask��n=0,1,ֻ֧����ͨ��
#define MC_ARM_CUR_INTP_MODE(n)       (MC_ARM_STATUS_DATA_BASE(n)+0x2C)    //��nͨ���ĵ�ǰ�����岹ģʽ��3-3��������5-5��������6-��б�棬7-G12.2����n=0,1,ֻ֧����ͨ��

#define MC_ARM_BLOCK_RUN_OVER(n)      (MC_ARM_STATUS_DATA_BASE(n)+0x30)    //��nͨ���ķֿ�岹��λ��־
#define MC_ARM_STEP_RUN_OVER(n)	   (MC_ARM_STATUS_DATA_BASE(n)+0x34)    //��nͨ���ĵ��β岹��λ��־
#define MC_ARM_BLOCK_RUN_OVER_MDA(n)  (MC_ARM_STATUS_DATA_BASE(n)+0x38)	//��nͨ��MDAģʽ�Ŀ鵽λ��־
#define MC_ARM_AUTO_BUF_DATA_COUNT(n) (MC_ARM_STATUS_DATA_BASE(n)+0x3C)    //��nͨ�����Զ����ݻ�������,n=0,1,ֻ֧����ͨ��
#define MC_ARM_MDA_BUF_DATA_COUNT(n)  (MC_ARM_STATUS_DATA_BASE(n)+0x40)    //��nͨ����MDA���ݻ�������,n=0,1,ֻ֧����ͨ��
#define MC_ARM_ERR_FLAG(n) 		   (MC_ARM_STATUS_DATA_BASE(n)+0x60)    //MCͨ�������־��, n=0,1,ֻ֧����ͨ��
#define MC_ARM_NEG_SOFT_LIMIT_MASK(n) (MC_ARM_STATUS_DATA_BASE(n)+0x64)    //��nͨ���ĳ���������λ����Mask,n=0,1,ֻ֧����ͨ��
#define MC_ARM_POS_SOFT_LIMIT_MASK(n) (MC_ARM_STATUS_DATA_BASE(n)+0x68)    //��nͨ���ĳ���������λ����Mask,n=0,1,ֻ֧����ͨ��
#define MC_ARM_POS_ERR_MASK(n)        (MC_ARM_STATUS_DATA_BASE(n)+0x6C)    //��nͨ����λ��ָ��������mask,n=0,1,ֻ֧����ͨ��


//#define MC_ARM_DEBUG_DATA(n)        (MC_ARM_STATUS_DATA_BASE + 0x1C0+0x04*n)   //��ȡ��n��debug���ݣ�ÿ��Ϊһ��32λ��


//�����˶�����ͨ����֧��2ͨ����ÿͨ��2K�ֽ�
#define MC_ARM_GCODE_BASE              (MC_ARM_REGISTER_BASE+0xB0000)                          //�����˶�����ͨ������ַ
#define MC_ARM_GCODE_DATA_BASE(n,m)	(MC_ARM_GCODE_BASE+MC_ARM_CHN_GCODE_FIFO_SIZE*n+MC_ARM_GCODE_FRAME_SIZE*m)		//�˶����ݻ���ַ, n=0,1,ֻ֧����ͨ��, mΪ�ڼ��黺������
#define MC_ARM_GCODE_WF(n,m)           (MC_ARM_GCODE_DATA_BASE(n,m))		    //��nͨ���ĵ�m������֡��д��־
#define MC_ARM_GCODE_RF(n,m)  	        (MC_ARM_GCODE_DATA_BASE(n,m)+0x02)		//��nͨ���ĵ�m������֡�Ķ���־
#define MC_ARM_GCODE_DATA(n,m,k)		(MC_ARM_GCODE_DATA_BASE(n,m)+0x04+0x04*k)    //��nͨ���ĵ�m���˶�����֡�еĵ�k��32λ���ݵ�ַ�� n=0,1,ֻ֧����ͨ��,
                                                                                //mΪ�ڼ��黺�����ݣ�0<=m<MC_ARM_CHN_GCODE_FIFO_DEPTH
                                                                                //kΪ���ڵ�32bit����ƫ�ƣ�0<=k<kMaxGCodeDataCount


typedef ListBuffer<McCmdFrame> McArmCmdBuffer;   //�������ָ����Ϣ����


/**
 * @brief SCģ����MCģ����ͨѶ�࣬����fpga��չEMIF�ӿ�
 */
class MCArmCommunication {
public:

	virtual ~MCArmCommunication();	//��������

	static MCArmCommunication *GetInstance();   //����ģʽ����ȡ����ʵ����Ψһ���ʵ�

	void SetInterface();   //���ýӿ�

	//�����˶���������ͨ���ӿ�
	bool WriteGCodeData(uint8_t chn, GCodeFrame &data);   	//д��G�����������˶���������
	bool CanWriteGCode(uint8_t chn);            //�Ƿ���д���˶���������


	//����ͨ���ӿ�
	bool WriteCmd(McCmdFrame &data, bool resend = false); 	//д��ָ��
	bool ReadCmdRsp(McCmdFrame &data);    //��ȡָ��
//	uint32_t ReadCmdFifoCount();        //��ȡ����ͨ��FIFO�е�ǰ���ݸ���
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
    void ReadCurFeed(const uint8_t chn_index, int32_t &cur_feed);

	//��ȡͨ����ǰ���������ٶ�
    void ReadRatedFeed(const uint8_t chn_index, int32_t &rated_feed);

	//��ȡͨ����ǰ�Զ����ݻ�������
	void ReadAutoBufDataCount(const uint8_t chn_index, uint16_t &count);

	//��ȡͨ����ǰMDA���ݻ�������
	void ReadMdaBufDataCount(const uint8_t chn_index, uint16_t &count);


	//��ȡͨ��ģ̬��Ϣ
	void ReadMcModeInfo(const uint8_t chn_index, uint32_t &mode_info);

	//��ȡAUTO�ֿ�岹��λ��־
	bool ReadAutoBlockRunOverFlag(const uint8_t chn_index);


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

	//��ȡͨ���ĵ�ǰ��������λ������mask
	void ReadChnPosSoftLimtMask(uint8_t chn_index, uint32_t &axis_mask);

	//��ȡͨ���ĵ�ǰ��������λ������mask
	void ReadChnNegSoftLimtMask(uint8_t chn_index, uint32_t &axis_mask);

	//��ȡͨ���ĵ�ǰλ��ָ�����澯��mask
	void ReadChnPosErrMask(uint8_t chn_index, uint16_t &axis_mask);

	//��ȡͨ���ĵ�ǰ����֡�����
	void ReadChnCurFrameIndex(uint8_t chn_index, uint16_t &frame_index);


	//��ȡMC�ĵ�������
	void ReadDebugData(uint32_t *debug_data);

	//��ȡǷѹ�澯�ź�
	bool ReadUnderVoltWarn();

private:
	MCArmCommunication();    //���캯��

	static void *ProcessCmdThread(void *args); //����������̺߳���
	bool ProcessCmdFun();  	//�������

	bool Initialize();    //��ʼ������
	bool Clean();          //������

	void CalMcCmdCrc(McCmdFrame &cmd);     //����MC�������CRC
	void CalMcGCodeFrameCrc(GCodeFrame &data);   //����G��������֡��CRC

private://˽�б���
	static MCArmCommunication *m_p_instance;    //��ʵ������

	ChannelEngine *m_p_channel_engine;   //ͨ������ָ��

	McArmCmdBuffer *m_list_cmd;	//��������������

	int m_n_mem_file;           //�����ļ��򿪾��
	uint8_t *m_p_addr_base;		//����ַ

	ErrorType m_error_code;    //���һ�εĴ�����

	pthread_mutex_t m_mutex_cmd;   // ��������ͨ��������,д�������

	pthread_t m_thread_process_cmd;    //������߳�

	uint8_t m_n_read_status_sem;    //״̬��ȡ�ź���

	uint64_t m_n_crc_err_count;		//CRCУ������

	uint8_t m_n_cur_gcode_index[kMaxChnCount];    //��ǰ�����˶�����֡���   0~MC_ARM_CHN_GCODE_FIFO_DEPTHѭ��
	uint8_t m_n_cur_cmd_send_index;      //��ǰ��������֡���  0~MC_ARM_CMD_FIFO_DEPTHѭ��
	uint8_t m_n_cur_cmd_recv_index;      //��ǰ��Ӧ����֡���  0~MC_ARM_CMD_FIFO_DEPTHѭ��

};

#endif /* MC_COMMUNICATION_ARM_H_ */
