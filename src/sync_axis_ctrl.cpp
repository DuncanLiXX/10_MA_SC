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
        mi->SendMiParam<uint8_t>(i+1, 1650, axis_config[i].sync_axis);   //是否同步轴
        mi->SendMiParam<uint8_t>(i+1, 1651, axis_config[i].master_axis_no); 	//主动轴号
        mi->SendMiParam<double>(i+1, 1653, axis_config[i].benchmark_offset); 	//基准偏差
        mi->SendMiParam<uint8_t>(i+1, 1654, axis_config[i].series_ctrl_axis); //是否串联控制
        mi->SendMiParam<uint32_t>(i+1, 1655, axis_config[i].sync_err_max_pos); 	//位置同步误差报警阈值
        mi->SendMiParam<uint8_t>(i+1, 1657, axis_config[i].sync_pre_load_torque); //预载电流偏置
        mi->SendMiParam<uint8_t>(i+1, 1658, axis_config[i].sync_err_max_torque); 	//扭矩同步误差报警阈值
        mi->SendMiParam<uint32_t>(i+1, 1659, axis_config[i].sync_err_max_mach); 	//坐标同步误差报警阈值
        mi->SendMiParam<uint8_t>(i+1, 1660, axis_config[i].sync_pos_detect);      //是否进行位置同步误差检测
        mi->SendMiParam<uint8_t>(i+1, 1661, axis_config[i].sync_mach_detect); 	//是否进行坐标同步误差检测
        mi->SendMiParam<uint8_t>(i+1, 1662, axis_config[i].sync_torque_detect); 	//是否进行扭矩同步误差检测
        mi->SendMiParam<uint16_t>(i+1, 1663, axis_config[i].serial_torque_ratio); 	//串联力矩系数 单位: 1%
        mi->SendMiParam<uint16_t>(i+1, 1664, axis_config[i].serial_pre_speed); 	//预载串联速度 单位：rpm
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

    if (mask == sync_mask)
    {
        return;
    }

    for (int i = 0; i < chn_config->chn_axis_count; i++)
    {
        bool need_sync = (mask & (0x01 << i));
        bool is_sync = axis_config[i].sync_axis;

        // 对一个非同步轴进行同步，报警
        if(!is_sync && need_sync){
            CreateError(ERR_SYNC_INVALID_OPT, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                        0, CHANNEL_ENGINE_INDEX, i);
            return;
        }

        int masterId = axis_config[i].master_axis_no;
        bool in_wait = wait_en_index & (0x01 << masterId);
        if (is_sync && !in_wait)
        {
            bool last_sync = sync_state & (0x01 << masterId);
            if (last_sync != need_sync)
            {
                wait_en_index |= (0x01 << masterId);         //进入等待状态
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
        if (sync_state & (0x01 << axis_config[i].master_axis_no))//已经建立关系
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
        if (sync_state & (0x01 << axis_config[i].master_axis_no))//已经建立关系
        {
            if(axis_config[i].master_axis_no - 1 == master_index)
                mask |= (0x01 << i);
        }
    }
    return mask;
}
