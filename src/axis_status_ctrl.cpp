#include "axis_status_ctrl.h"
#include "mi_communication.h"
#include "alarm_processor.h"
#include "hmi_shared_data.h"
#include "parm_definition.h"
#include <vector>
#include <algorithm>


// 以下错误出现时会关闭伺服使能
const std::vector<int> CRITS =
{
    // 同步轴相关报警
    150,151,152,1009,1016,1017,

    // 20184-20199为PMC发出的需要关闭的伺服的报警
    20184,20185,20186,20187,20188,20189,20190,20191,
    20192,20193,20194,20195,20196,20197,20198,20199,
};



AxisStatusCtrl::AxisStatusCtrl(){

}

void AxisStatusCtrl::Init(MICommunication *comm,
                               SCChannelConfig *channel_config,
                               SCAxisConfig *axis_config,
                                FRegBits *f_reg){
    mi = comm;
    channel = channel_config;
    axis = axis_config;
    F = f_reg;

    for(int i=0; i<channel_config->chn_axis_count; i++){
        if(axis[i].axis_type == AXIS_SPINDLE)
            continue;
        axis_mask[i] = true;
    }
}

void AxisStatusCtrl::InputEsp(uint8_t _ESP){
    if(!axis)
        return;
    this->_ESP = _ESP;
    UpdateServoState();
}

void AxisStatusCtrl::InputSyncWarn(uint64_t flag){
    if(!axis)
        return;
    this->sync_warn = flag;
    UpdateServoState();
}

void AxisStatusCtrl::InputSVF(uint64_t SVF){
    if(!axis)
        return;
    this->SVF = SVF;
    UpdateServoState();
}

void AxisStatusCtrl::RspMiReady(){
    if(!axis)
        return;
    mi_ready = true;
    UpdateServoState(true);
}

void AxisStatusCtrl::UpdateServoState(bool force){
    if(!axis)
        return;
    if(!mi_ready)
        return;
    bool enable; // 是否使能
    bool svf;    // 是否伺服关断（关断的状态下如果拧动轴，会在恢复之后移动回来）
    bool servo_warn = false; // 是否有需要关闭伺服的报警
    AlarmProcessor *alarms = AlarmProcessor::GetInstance();
    CircularBuffer<ErrorInfo>* list = alarms->GetWarningList();
    int count = list->BufLen();
    for(int i = 0; i < count; i++){
        ErrorInfo *info = list->ReadDataPtr(i);
        if(std::find(CRITS.begin(),CRITS.end(),info->error_code) != CRITS.end()){
             servo_warn = true;
             break;
        }
    }

    // 修改使能状态
    bool change = false;
    for(int i=0; i<channel->chn_axis_count; i++){
        // 主轴的使能不在这里控制
        if(axis[i].axis_type == AXIS_SPINDLE)
            continue;

        svf = SVF & (0x01<<i);
        enable = _ESP && !servo_warn && (sync_warn & (0x01<<i)) == 0 && !svf;

        if(enable == last_enable[i] && last_svf[i] == svf && !force){
            last_enable[i] = enable;
            last_svf[i] = svf;
            continue;
        }

        mi->SendAxisEnableCmd(i+1, enable, svf);
        last_enable[i] = enable;
        last_svf[i] = svf;
        change = true;
    }

//    // 判断是否所有轴都上了使能，如果是，打开伺服就绪信号
//    if(change){
//        bool ready = true;
//        for(int i=0; i<channel->chn_axis_count; i++){
//            if(last_enable[i] != axis_mask[i]){
//                ready = false;
//                break;
//            }
//        }
//        if(ready){
//            F->SA = true;
//        }else{
//            F->SA = false;
//        }
//    }
}
