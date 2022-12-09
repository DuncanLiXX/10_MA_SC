#ifndef SYNC_AXIS_CTRL_H
#define SYNC_AXIS_CTRL_H

#include <stdint.h>
#include "hmi_shared_data.h"

class MICommunication;
struct SCAxisConfig;
struct SCChannelConfig;

/*
 *@date 2022/11/7
 *@brief:实现同步轴功能逻辑
 */

class SyncAxisCtrl
{
public:
    SyncAxisCtrl();
    void SendMiSyncParams();        // 把同步轴相关参数发送给mi

    void InputSync(uint8_t SYNC);   // 输入SYNC信号
    void InputSyncJ(uint8_t SYNCJ);  // 输入SYNCJ信号
    void InputMode(ChnWorkMode mode);   // 输入模式

    void RspSyncAxis(int64_t mask);    // 轴同步回复
    void RspEnSyncAxis(bool enable, bool success);  // 轴同步使能回复

    // 检测某个轴的同步状态
    // 返回值 0:不在同步状态  1:主动轴   2:从动轴
    int CheckSyncState(uint8_t axis_index);

    // 获取某个轴的从动轴（前提是处于同步状态）
    // 返回值 : 每一bit代表一个轴是否为同步轴
    uint8_t GetSlaveAxis(uint8_t master_index);

private:
    void UpdateMask(int64_t mask);

// 为了方便检测数据，暂时设置为public
public:
    SCChannelConfig *chn_config{nullptr};
    SCAxisConfig *axis_config{nullptr};
    MICommunication *mi{nullptr};

    int64_t sync_mask{0};   // 轴同步mask
    int64_t sync_en{0};     // 同步使能
    uint8_t SYNC{0};           // 自动模式是否需要同步的标记信号   G138
    uint8_t SYNCJ{0};          // 非自动模式是否需要同步的标记信号  G140
    ChnWorkMode mode{MDA_MODE}; // 模式
    int64_t wait_sync_mask{0};     // 等待同步的mask
    int wait_en_index{-1};      // 等待使能的轴  从0开始
};

#endif
