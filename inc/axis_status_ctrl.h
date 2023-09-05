#ifndef AXIS_STATUS_H_
#define AXIS_STATUS_H_

#include <stdint.h>
#include <pmc_register.h>

/*
 *@date 2022/12/20
 *@brief:�������Ƿ�ʹ��
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
    // ���ü�ͣ״̬
    // ESP  0:���ڼ�ͣ״̬  1�����ڼ�ͣ״̬
    void InputEsp(uint8_t _ESP);

    // ����ͬ�����쳣����
    void InputSyncWarn(uint64_t flag);

    // �����ŷ��ض��ź�
    void InputSVF(uint64_t SVF);
    void WaitingEnable();
    bool IsWaitingEnable(){return is_waiting_enable;}

    // �����ŷ��澯
    void InputServoWarn(bool is_warn);

    // �յ�MI��׼��������
    void RspMiReady();

    // �����ŷ�״̬
    // force 0:��Ҫ����ʹ��ʱ����mi������ 1:һ�����ط�ʹ��״̬
    void UpdateServoState(bool force = false);

    void InitRealPhyAxisMask();
    // ����SA(F0.6)�ŷ�����״̬
    void UpdateSA(uint64_t srvon_mask);

private:
    uint8_t _ESP{1};    // ��ͣ״̬ ����Ч
    uint64_t sync_warn{0x00}; // ͬ���ᾯ�� ÿһbit����һ����
    uint64_t SVF{0x00}; // �ŷ��ض��ź�
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
