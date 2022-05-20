/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file channel_mode_group.h
 *@author gonghao
 *@date 2020/03/19
 *@brief ��ͷ�ļ�Ϊͨ����ʽ���������
 *@version
 */

#ifndef INC_CHANNEL_CHANNEL_MODE_GROUP_H_
#define INC_CHANNEL_CHANNEL_MODE_GROUP_H_

#include "global_include.h"

#define MAX_GROUP_CHN_COUNT  (16)   //����ʽ�����ͨ������

class ChannelModeGroup {
public:
	ChannelModeGroup(uint8_t index = 0);
	virtual ~ChannelModeGroup();

	void SetChnGroupIndex(uint8_t index){m_n_mode_group_index = index;}   //���÷�ʽ��������
	uint8_t GetGroupIndex(){return m_n_mode_group_index;}      //���ط�ʽ���ţ���0��ʼ
	uint8_t GetChannelCount(){return m_n_chn_count;}           //���ط�ʽ�鵱ǰͨ������
	bool AddChannel(uint8_t chn_index);     //����ָ��ͨ��
	void RemoveChannel(uint8_t chn_index);   //�Ƴ�ָ��ͨ��

	bool HasChannel(uint8_t chn_index);    //ָ��ͨ���Ƿ��ڷ�ʽ����

	uint8_t GetChannel(uint8_t idx);  //����ָ��˳��ŵ�ͨ����


//˽�нӿ�
private:
	uint8_t GetChannelIndex(uint8_t chn_index);    //��ȡָ��ͨ����˳���



//��Ա����
private:
	uint8_t m_n_mode_group_index;   //��ʽ�������ţ���0��ʼ
	uint8_t m_n_chn_count;   	 //����ʽ��ͨ������
	uint8_t m_n_chn_index[MAX_GROUP_CHN_COUNT];   //ͨ�������ţ���0��ʼ��0xFF��ʾδ���ã���ʼֵ��Ϊ0xFF
};

#endif /* INC_CHANNEL_CHANNEL_MODE_GROUP_H_ */
