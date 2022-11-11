#ifndef SYNC_AXIS_CTRL_H
#define SYNC_AXIS_CTRL_H

#include <stdint.h>
#include "hmi_shared_data.h"

class MICommunication;
struct SCAxisConfig;
struct SCChannelConfig;

/*
 *@date 2022/11/7
 *@brief:ʵ��ͬ���Ṧ���߼�
 */

class SyncAxisCtrl
{
public:
    SyncAxisCtrl();
    void SendMiSyncParams();        // ��ͬ������ز������͸�mi

    void InputSync(uint8_t SYNC);   // ����SYNC�ź�
    void InputSyncJ(uint8_t SYNCJ);  // ����SYNCJ�ź�
    void InputMode(ChnWorkMode mode);   // ����ģʽ

    void RspSyncAxis(int64_t mask);    // ��ͬ���ظ�
    void RspEnSyncAxis(bool enable, bool success);  // ��ͬ��ʹ�ܻظ�

private:
    void UpdateMask(int64_t mask);

// Ϊ�˷��������ݣ���ʱ����Ϊpublic
public:
    SCChannelConfig *chn_config{nullptr};
    SCAxisConfig *axis_config{nullptr};
    MICommunication *mi{nullptr};

    int64_t sync_mask{0};   // ��ͬ��mask
    int64_t sync_en{0};     // ͬ��ʹ��
    uint8_t SYNC{0};           // �Զ�ģʽ�Ƿ���Ҫͬ���ı���ź�   G138
    uint8_t SYNCJ{0};          // ���Զ�ģʽ�Ƿ���Ҫͬ���ı���ź�  G140
    ChnWorkMode mode{MDA_MODE}; // ģʽ
    int64_t wait_sync_mask{0};     // �ȴ�ͬ����mask
    int wait_en_index{-1};      // �ȴ�ʹ�ܵ���  ��0��ʼ
};

#endif
