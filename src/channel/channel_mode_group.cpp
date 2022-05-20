/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file ChannelModeGroup.cpp
 *@author gonghao
 *@date 2020/03/19
 *@brief 本文件为通道方式组类的实现
 *@version
 */

#include <channel_mode_group.h>

/**
 * @brief 构造函数
 * @param index : 方式组编号
 */
ChannelModeGroup::ChannelModeGroup(uint8_t index) {
	// TODO Auto-generated constructor stub
	this->m_n_mode_group_index = index;
	this->m_n_chn_count = 0;
	memset(this->m_n_chn_index, 0xFF, MAX_GROUP_CHN_COUNT*sizeof(uint8_t));  //初始化为无效通道号

}

/**
 * @brief 析构函数
 */
ChannelModeGroup::~ChannelModeGroup() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief 增加指定通道
 * @param chn_index : 指定的通道索引，从0开始
 * @return ture--成功   false--失败
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
 * @brief 移除指定通道
 * @param chn_index : 指定的通道索引，从0开始
 */
void ChannelModeGroup::RemoveChannel(uint8_t chn_index){
	uint8_t index = this->GetChannelIndex(chn_index);
	if(index == 0xFF)
		return;  //无此通道

	//后面的通道前移
	if(index == this->m_n_chn_count-1)
		this->m_n_chn_index[index] = 0xFF;
	else{
		memcpy(&m_n_chn_index[index], &m_n_chn_index[index+1], sizeof(uint8_t)*(m_n_chn_count-index-1));
		this->m_n_chn_index[m_n_chn_count-1] = 0xFF;
	}
	this->m_n_chn_count--;
}


/**
 * @brief 指定通道是否在方式组中
 * @param chn_index : 指定的通道索引，从0开始
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
 * @brief 获取指定通道的顺序号
 * @param chn_index : 指定的通道索引，从0开始
 * @return 返回指定通道在方式组中的顺序号，成功返回0~15，失败返回0xFF
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
 * @brief 返回指定顺序号的通道号
 * @param idx : 指定的顺序号
 * @return 成功返回通道号，从0开始    失败则返回0xFF
 */
uint8_t ChannelModeGroup::GetChannel(uint8_t idx){
	if(idx >= this->m_n_chn_count)
		return 0xFF;

	return m_n_chn_index[idx];
}
