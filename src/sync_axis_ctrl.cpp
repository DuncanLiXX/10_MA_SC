#include "sync_axis_ctrl.h"
#include "parameter/parm_definition.h"
#include "parameter/parm_manager.h"
#include "mi_communication.h"
#include "showsc.h"

SyncAxisCtrl::SyncAxisCtrl()
{
    mi = MICommunication::GetInstance();
    chn_config = ParmManager::GetInstance()->GetChannelConfig(0);
    axis_config = ParmManager::GetInstance()->GetAxisConfig();
}

void SyncAxisCtrl::SendMiSyncParams()
{
    if(!chn_config || !axis_config)
        return;
    for(int i = 0; i < chn_config->chn_axis_count; i++){
        mi->SendMiParam<uint8_t>(i+1, 1650, axis_config->sync_axis);   //�Ƿ�ͬ����
        mi->SendMiParam<uint8_t>(i+1, 1651, axis_config->master_axis_no); 	//�������
        mi->SendMiParam<double>(i+1, 1653, axis_config->benchmark_offset); 	//��׼ƫ��
        mi->SendMiParam<uint8_t>(i+1, 1654, axis_config->series_ctrl_axis); //�Ƿ�������
        mi->SendMiParam<uint32_t>(i+1, 1655, axis_config->sync_err_max_pos); 	//λ��ͬ��������ֵ
        mi->SendMiParam<uint8_t>(i+1, 1657, axis_config->sync_pre_load_torque); //Ԥ�ص���ƫ��
        mi->SendMiParam<uint8_t>(i+1, 1658, axis_config->sync_err_max_torque); 	//Ť��ͬ��������ֵ
        mi->SendMiParam<uint32_t>(i+1, 1659, axis_config->sync_err_max_mach); 	//����ͬ��������ֵ
        mi->SendMiParam<uint8_t>(i+1, 1660, axis_config->sync_pos_detect);      //�Ƿ����λ��ͬ�������
        mi->SendMiParam<uint8_t>(i+1, 1661, axis_config->sync_mach_detect); 	//�Ƿ��������ͬ�������
        mi->SendMiParam<uint8_t>(i+1, 1662, axis_config->sync_torque_detect); 	//�Ƿ����Ť��ͬ�������

    }
}

void SyncAxisCtrl::InputSync(uint8_t SYNC)
{
    if(mode == AUTO_MODE || mode == MDA_MODE){
        UpdateMask(SYNC);
    }
    this->SYNC = SYNC;
}

void SyncAxisCtrl::InputSyncJ(uint8_t SYNCJ)
{
    bool is_auto = (mode == AUTO_MODE || mode == MDA_MODE);
    if(!is_auto){
        UpdateMask(static_cast<int64_t>(SYNCJ));
    }
    this->SYNCJ = SYNCJ;
}

void SyncAxisCtrl::InputMode(ChnWorkMode mode)
{
    if(mode == AUTO_MODE || mode == MDA_MODE){
        UpdateMask(SYNC);
    }else{
        UpdateMask(SYNCJ);
    }
    this->mode = mode;
}

void SyncAxisCtrl::UpdateMask(int64_t mask){
    if(mask == sync_mask)
        return;

    for(int i = 0; i < chn_config->chn_axis_count; i++){
        bool need_sync = (mask & (0x01 << i));
        bool is_sync = axis_config[i].sync_axis;

        // ��һ����ͬ�������ͬ��������
        if(!is_sync && need_sync){
            CreateError(ERR_SYNC_INVALID_OPT, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                        0, CHANNEL_ENGINE_INDEX, i);
        }
    }

    // ����ͬ����Ҫ�������裺
    // 1.��mi����ͬ��mask
    // 2.��ͬ��ʹ��

    // ������ִ�е�һ����������ظ����ٴ�ʹ��
    wait_sync_mask = mask;
    mi->SendSyncAxis(mask);
    ScPrintf("UpdateMask: mask = %lld\n",mask);
}

void SyncAxisCtrl::RspSyncAxis(int64_t mask)
{
    ScPrintf("RspSyncAxis: mask = %lld\n",mask);

    for(int i = 0; i < chn_config->chn_axis_count; i++){
        bool sync_now = (mask & (0x01<<i));
        bool sync_last = (sync_mask & (0x01<<i));
        bool sync_wait = (wait_sync_mask & (0x01<<1));

        // ����ͬ��ʧ��
        if(sync_wait != sync_now && sync_wait){
            CreateError(ERR_EN_SYNC_AXIS, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                        0, CHANNEL_ENGINE_INDEX, i);
            return;
        }
        // ȡ��ͬ��ʧ��
        if(sync_wait != sync_now && !sync_wait){
            CreateError(ERR_DIS_SYNC_AXIS, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                        0, CHANNEL_ENGINE_INDEX, i);
            return;
        }

        // ͬ��״̬�����˸ı䣬��Ҫ�ı�ʹ��״̬
        if(sync_now ^ sync_last){
            wait_en_index = i;
            mi->SendEnSyncAxis(axis_config[i].master_axis_no,i+1,sync_now);
        }
    }

    sync_mask = mask;

}

void SyncAxisCtrl::RspEnSyncAxis(bool enable, bool success)
{
    ScPrintf("RspSyncAxis: enable = %d,success = %d\n",enable, success);

    if(!success){
        wait_en_index = -1;
        return;
    }
    if(wait_en_index < 0 || wait_en_index >= chn_config->chn_axis_count)
        return;

    if(enable){
        sync_en |= 0x01 << wait_en_index;
    }else{
        sync_en &= ~(0x01 << wait_en_index);
    }

    wait_en_index = -1;
}
