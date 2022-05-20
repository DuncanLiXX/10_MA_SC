/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file ChannelModeGroup.cpp
 *@author gonghao
 *@date 2020/03/19
 *@brief ���ļ�Ϊͨ����ʽ�����ʵ��
 *@version
 */

#include <channel_mode_group.h>

/**
 * @brief ���캯��
 * @param index : ��ʽ����
 */
ChannelModeGroup::ChannelModeGroup(uint8_t index) {
	// TODO Auto-generated constructor stub
	this->m_n_mode_group_index = index;
	this->m_n_chn_count = 0;
	memset(this->m_n_chn_index, 0xFF, MAX_GROUP_CHN_COUNT*sizeof(uint8_t));  //��ʼ��Ϊ��Чͨ����

}

/**
 * @brief ��������
 */
ChannelModeGroup::~ChannelModeGroup() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief ����ָ��ͨ��
 * @param chn_index : ָ����ͨ����������0��ʼ
 * @return ture--�ɹ�   false--ʧ��
 */
bool ChannelModeGroup::AddChannel(uint8_t chn_index){
	if(this->m_n_chn_count >= MAX_GROUP_CHN_COUNT)
		return false;
	if(this->HasChannel(chn_index))
		return true;

	this->m_n_chn_index[m_n_chn_count] = chn_index;
	this->m_n_chn_count++;
	return true;
}

/**
 * @brief �Ƴ�ָ��ͨ��
 * @param chn_index : ָ����ͨ����������0��ʼ
 */
void ChannelModeGroup::RemoveChannel(uint8_t chn_index){
	uint8_t index = this->GetChannelIndex(chn_index);
	if(index == 0xFF)
		return;  //�޴�ͨ��

	//�����ͨ��ǰ��
	if(index == this->m_n_chn_count-1)
		this->m_n_chn_index[index] = 0xFF;
	else{
		memcpy(&m_n_chn_index[index], &m_n_chn_index[index+1], sizeof(uint8_t)*(m_n_chn_count-index-1));
		this->m_n_chn_index[m_n_chn_count-1] = 0xFF;
	}
	this->m_n_chn_count--;
}


/**
 * @brief ָ��ͨ���Ƿ��ڷ�ʽ����
 * @param chn_index : ָ����ͨ����������0��ʼ
 */
bool ChannelModeGroup::HasChannel(uint8_t chn_index){
	bool res = false;

	for(uint8_t index = 0; index < this->m_n_chn_count; index++){
		if(this->m_n_chn_index[index] == chn_index){
			res = true;
			break;
		}
	}

	return res;
}

/**
 * @brief ��ȡָ��ͨ����˳���
 * @param chn_index : ָ����ͨ����������0��ʼ
 * @return ����ָ��ͨ���ڷ�ʽ���е�˳��ţ��ɹ�����0~15��ʧ�ܷ���0xFF
 */
uint8_t ChannelModeGroup::GetChannelIndex(uint8_t chn_index){
	uint8_t index = 0;
	for(; index < this->m_n_chn_count; index++){
		if(this->m_n_chn_index[index] == chn_index)
			return index;
	}

	index = 0xFF;
	return index;
}

/**
 * @brief ����ָ��˳��ŵ�ͨ����
 * @param idx : ָ����˳���
 * @return �ɹ�����ͨ���ţ���0��ʼ    ʧ���򷵻�0xFF
 */
uint8_t ChannelModeGroup::GetChannel(uint8_t idx){
	if(idx >= this->m_n_chn_count)
		return 0xFF;

	return m_n_chn_index[idx];
}
