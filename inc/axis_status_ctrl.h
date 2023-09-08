#ifndef AXIS_STATUS_H_
#define AXIS_STATUS_H_

#include <stdint.h>
#include <pmc_register.h>

/*
 *@date 2022/12/20
 *@brief:控制轴是否使能
 */

class MICommunication;
struct FRegBits;
struct SCChannelConfig;
struct SCAxisConfig;

class AxisStatusCtrl{
public:
    AxisStatusCtrl();
    void Init(MICommunication *comm,
              SCChannelConfig *channel,
              SCAxisConfig *axis_config,
              FRegBits *f_reg);
    // 设置急停状态
    // ESP  0:处于急停状态  1：不在急停状态
    void InputEsp(uint8_t _ESP);

    // 输入同步轴异常警告
    void InputSyncWarn(uint64_t flag);

    // 输入伺服关断信号
    void InputSVF(uint64_t SVF);
    void WaitingEnable();
    bool IsWaitingEnable(){return is_waiting_enable;}

    // 输入伺服告警
    void InputServoWarn(bool is_warn);

    // 收到MI的准备好命令
    void RspMiReady();

    // 更新伺服状态
    // force 0:需要更改使能时才往mi发命令 1:一定会重发使能状态
    void UpdateServoState(bool force = false);

    void InitRealPhyAxisMask();
    // 更新SA(F0.6)伺服就绪状态
    void UpdateSA(uint64_t srvon_mask);

private:
    uint8_t _ESP{1};    // 急停状态 低有效
    uint64_t sync_warn{0x00}; // 同步轴警告 每一bit代表一个轴
    uint64_t SVF{0x00}; // 伺服关断信号
    bool mi_ready{false};
    MICommunication *mi{nullptr};
    SCChannelConfig *channel{nullptr};
    SCAxisConfig *axis{nullptr};
    FRegBits *F{nullptr};

    bool last_enable[kMaxAxisNum]{false};
    bool last_svf[kMaxAxisNum]{false};
    bool axis_mask[kMaxAxisNum]{false};

    uint64_t real_phy_axis{0x00};
    bool is_waiting_enable{false};
    bool SA_Processing{false};
};



#endif
