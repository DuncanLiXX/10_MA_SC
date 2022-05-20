/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file channel_mode_group.h
 *@author gonghao
 *@date 2020/03/19
 *@brief 本头文件为通道方式组类的声明
 *@version
 */

#ifndef INC_CHANNEL_CHANNEL_MODE_GROUP_H_
#define INC_CHANNEL_CHANNEL_MODE_GROUP_H_

#include "global_include.h"

#define MAX_GROUP_CHN_COUNT  (16)   //单方式组最大通道数量

class ChannelModeGroup {
public:
	ChannelModeGroup(uint8_t index = 0);
	virtual ~ChannelModeGroup();

	void SetChnGroupIndex(uint8_t index){m_n_mode_group_index = index;}   //设置方式组索引号
	uint8_t GetGroupIndex(){return m_n_mode_group_index;}      //返回方式组编号，从0开始
	uint8_t GetChannelCount(){return m_n_chn_count;}           //返回方式组当前通道数量
	bool AddChannel(uint8_t chn_index);     //增加指定通道
	void RemoveChannel(uint8_t chn_index);   //移除指定通道

	bool HasChannel(uint8_t chn_index);    //指定通道是否在方式组中

	uint8_t GetChannel(uint8_t idx);  //返回指定顺序号的通道号


//私有接口
private:
	uint8_t GetChannelIndex(uint8_t chn_index);    //获取指定通道的顺序号



//成员变量
private:
	uint8_t m_n_mode_group_index;   //方式组索引号，从0开始
	uint8_t m_n_chn_count;   	 //本方式组通道数量
	uint8_t m_n_chn_index[MAX_GROUP_CHN_COUNT];   //通道索引号，从0开始，0xFF表示未配置，初始值均为0xFF
};

#endif /* INC_CHANNEL_CHANNEL_MODE_GROUP_H_ */
