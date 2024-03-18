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
        mi->SendMiParam<uint8_t>(i+1, 20700, axis_config[i].sync_axis);   //�Ƿ�ͬ����
        mi->SendMiParam<uint8_t>(i+1, 20701, axis_config[i].master_axis_no); 	//�������
        mi->SendMiParam<uint8_t>(i+1, 20702, axis_config[i].disp_coord); 	//ͬ��У׼����
        mi->SendMiParam<int64_t>(i+1, 20703, (int64_t)(axis_config[i].benchmark_offset*1000)); 	//У׼���ƫ��
        mi->SendMiParam<uint32_t>(i+1, 20704, (int64_t)axis_config[i].sync_err_max_pos*1000);     //λ��ͬ��������ֵ
        mi->SendMiParam<uint32_t>(i+1, 20705, (int64_t)axis_config[i].sync_err_max_mach*1000); 	//����ͬ��������ֵ
        mi->SendMiParam<uint8_t>(i+1, 20720, axis_config[i].series_ctrl_axis); //�Ƿ�������

        mi->SendMiParam<uint8_t>(i+1, 20721, axis_config[i].sync_pre_load_torque); //Ԥ�ص���ƫ��
        mi->SendMiParam<uint8_t>(i+1, 20725, axis_config[i].sync_err_max_torque); 	//Ť��ͬ��������ֵ

        mi->SendMiParam<uint8_t>(i+1, 20711, axis_config[i].sync_pos_detect);      //�Ƿ����λ��ͬ�������
        mi->SendMiParam<uint8_t>(i+1, 20712, axis_config[i].sync_mach_detect); 	//�Ƿ��������ͬ�������
        mi->SendMiParam<uint8_t>(i+1, 20724, axis_config[i].sync_torque_detect); 	//�Ƿ����Ť��ͬ�������
        mi->SendMiParam<uint16_t>(i+1, 20723, axis_config[i].serial_torque_ratio); 	//��������ϵ�� ��λ: 1%
        mi->SendMiParam<uint16_t>(i+1, 20722, axis_config[i].serial_pre_speed); 	//Ԥ�ش����ٶ� ��λ��rpm
    }
}

void SyncAxisCtrl::InputSync(uint8_t SYNC)
{

    mi->SendSyncAxis(SYNC);

    /*
    if(mode == AUTO_MODE || mode == MDA_MODE){
        UpdateMask(SYNC);
    }*/
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
      UpdateMask(SYNC);

//    if(mode == AUTO_MODE || mode == MDA_MODE){
//        UpdateMask(SYNC);
//    }else{
//        UpdateMask(SYNCJ);
//    }
    this->mode = mode;
}

void SyncAxisCtrl::UpdateMask(int64_t mask){


    if (mask == sync_mask)
    {
        return;
    }

    for (int i = 0; i < chn_config->chn_axis_count; i++)
    {
        bool need_sync = (mask & (0x01 << i));
        bool is_sync = axis_config[i].sync_axis;

        // ��һ����ͬ�������ͬ��������
        /*
        if(!is_sync && need_sync){
            CreateError(ERR_SYNC_INVALID_OPT, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                        0, CHANNEL_ENGINE_INDEX, i);
            return;
        }
        */

        int masterId = axis_config[i].master_axis_no;
        bool in_wait = wait_en_index & (0x01 << masterId);
        if (is_sync && !in_wait)
        {
            bool last_sync = sync_state & (0x01 << masterId);
            if (last_sync != need_sync)
            {
                wait_en_index |= (0x01 << masterId);         //����ȴ�״̬
                mi->SendEnSyncAxis(axis_config[i].master_axis_no, i+1, need_sync);
            }
        }
    }

    sync_mask = mask;
}

void SyncAxisCtrl::RspEnSyncAxis(int axisId, bool enable, bool success)
{
    ScPrintf("RspSyncAxis: axisId = %d, enable = %d, success = %d\n", axisId, enable, success);

    wait_en_index &= ~(0x01 << axisId);
    if (!success)
    {
        if(enable){
            CreateError(ERR_EN_SYNC_AXIS, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                0, CHANNEL_ENGINE_INDEX, axisId);
            return;
        }
        else
        {
            CreateError(ERR_DIS_SYNC_AXIS, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                0, CHANNEL_ENGINE_INDEX, axisId);
            return;
        }
    }

    if (enable)
    {
        sync_state |= (0x01 << axisId);
    }
    else
    {
        sync_state &= ~(0x01 << axisId);
    }
}

int SyncAxisCtrl::CheckSyncState(uint8_t axis_index)
{
    if(!axis_config)
        return 0;

    for(int i = 0; i<chn_config->chn_axis_count; i++){
        bool flag = (sync_mask >> i) & 0x01;
        if(!flag)
            continue;
        if (sync_state & (0x01 << axis_config[i].master_axis_no))//�Ѿ�������ϵ
        {
            if(axis_config[i].master_axis_no - 1 == axis_index)
                return 1;
            else if(i == axis_index)
                return 2;
        }
    }

    return 0;
}

uint8_t SyncAxisCtrl::GetSlaveAxis(uint8_t master_index)
{
    if(!axis_config)
        return 0;
    uint8_t mask = 0x00;
    for(int i = 0; i<chn_config->chn_axis_count; i++){
        bool flag = (sync_mask >> i) & 0x01;
        if(!flag)
            continue;
        if (sync_state & (0x01 << axis_config[i].master_axis_no))//�Ѿ�������ϵ
        {
            if(axis_config[i].master_axis_no - 1 == master_index)
                mask |= (0x01 << i);
        }
    }
    return mask;
}
