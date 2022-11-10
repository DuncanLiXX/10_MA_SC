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
        mi->SendMiParam<uint8_t>(i+1, 1650, axis_config->sync_axis);   //是否同步轴
        mi->SendMiParam<uint8_t>(i+1, 1651, axis_config->master_axis_no); 	//主动轴号
        mi->SendMiParam<double>(i+1, 1653, axis_config->benchmark_offset); 	//基准偏差
        mi->SendMiParam<uint8_t>(i+1, 1654, axis_config->series_ctrl_axis); //是否串联控制
        mi->SendMiParam<uint32_t>(i+1, 1655, axis_config->sync_err_max_pos); 	//位置同步误差报警阈值
        mi->SendMiParam<uint8_t>(i+1, 1657, axis_config->sync_pre_load_torque); //预载电流偏置
        mi->SendMiParam<uint8_t>(i+1, 1658, axis_config->sync_err_max_torque); 	//扭矩同步误差报警阈值
        mi->SendMiParam<uint32_t>(i+1, 1659, axis_config->sync_err_max_mach); 	//坐标同步误差报警阈值
        mi->SendMiParam<uint8_t>(i+1, 1660, axis_config->sync_pos_detect);      //是否进行位置同步误差检测
        mi->SendMiParam<uint8_t>(i+1, 1661, axis_config->sync_mach_detect); 	//是否进行坐标同步误差检测
        mi->SendMiParam<uint8_t>(i+1, 1662, axis_config->sync_torque_detect); 	//是否进行扭矩同步误差检测

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

        // 对一个非同步轴进行同步，报警
        if(!is_sync && need_sync){
            CreateError(ERR_SYNC_INVALID_OPT, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                        0, CHANNEL_ENGINE_INDEX, i);
        }
    }

    // 建立同步需要两个步骤：
    // 1.给mi发送同步mask
    // 2.打开同步使能

    // 这里先执行第一步，等命令回复后再打开使能
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

        // 建立同步失败
        if(sync_wait != sync_now && sync_wait){
            CreateError(ERR_EN_SYNC_AXIS, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                        0, CHANNEL_ENGINE_INDEX, i);
            return;
        }
        // 取消同步失败
        if(sync_wait != sync_now && !sync_wait){
            CreateError(ERR_DIS_SYNC_AXIS, ERROR_LEVEL, CLEAR_BY_MCP_RESET,
                        0, CHANNEL_ENGINE_INDEX, i);
            return;
        }

        // 同步状态发生了改变，还要改变使能状态
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
