#include "axis_status_ctrl.h"
#include "mi_communication.h"
#include "alarm_processor.h"
#include "hmi_shared_data.h"
#include "parm_definition.h"
#include <vector>
#include <algorithm>


// ���´������ʱ��ر��ŷ�ʹ��
const std::vector<int> CRITS =
{
    // ͬ������ر���
    150,151,152,1009,1016,1017,

    // 20184-20199ΪPMC��������Ҫ�رյ��ŷ��ı���
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
    bool enable; // �Ƿ�ʹ��
    bool svf;    // �Ƿ��ŷ��ضϣ��ضϵ�״̬�����š���ᣬ���ڻָ�֮���ƶ�������
    bool servo_warn = false; // �Ƿ�����Ҫ�ر��ŷ��ı���
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

    // �޸�ʹ��״̬
    bool change = false;
    for(int i=0; i<channel->chn_axis_count; i++){
        // �����ʹ�ܲ����������
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

//    // �ж��Ƿ������ᶼ����ʹ�ܣ�����ǣ����ŷ������ź�
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
