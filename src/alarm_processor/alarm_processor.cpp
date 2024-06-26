
/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file alarm_processor.cpp
 *@author gonghao
 *@date 2020/05/15
 *@brief 本头文件为告警处理类的实现
 *@version
 */
#include "alarm_processor.h"
#include "hmi_communication.h"
#include "channel_engine.h"
#include "channel_control.h"
#include "axis_status_ctrl.h"

const int kMaxAlarmCount = 40;   //告警缓冲区最大值

//m_instance初始化为nullptr
AlarmProcessor* AlarmProcessor::m_instance = nullptr;
TraceLogProcess* TraceLogProcess::m_instance = nullptr;

//错误处理类构造函数
AlarmProcessor::AlarmProcessor(){
    try{
        m_error_info_input_list = new CircularBuffer<ErrorInfo>(
                kMaxAlarmCount);
    } catch (std::bad_alloc& e) {
        g_ptr_trace->PrintTrace(TRACE_ERROR, ERROR_PROCESS_SC,
                "failed to malloc buffer");
    }
    m_latest_error.error_code = ERR_NONE;
//	m_p_hmi_comm = nullptr;
//	m_p_chn_engine = nullptr;
    pthread_mutex_init(&m_mutex, NULL);
}

/*
 * @brief 告警处理类析构函数
 * @param 无
 * @return 无
 */
AlarmProcessor::~AlarmProcessor() {
    if (m_error_info_input_list != nullptr) {
        delete m_error_info_input_list;
        m_error_info_input_list = nullptr;
    }


    pthread_mutex_destroy(&m_mutex);
}

/*
 * @brief 写入错误信息
 * @param error_info 需要写入第错误信息指针
 * @return 无
 */
void AlarmProcessor::SetErrorInfo(ErrorInfo* error_info) {
    //printf("enter AlarmProcessor::SetErrorInfo()\n");
    assert(error_info != NULL);

    pthread_mutex_lock(&m_mutex);
    if (m_error_info_input_list->EmptyBufLen() == 0) {
        ErrorInfo err;
        m_error_info_input_list->ReadData(&err, 1);
    }

    SendToHmi(error_info);
    ProcessAlarm(error_info);
    m_timePoint = std::chrono::steady_clock::now();
    if (error_info->clear_type == CLEAR_BY_AUTO)
        m_auto_error_list.push_back(make_pair(*error_info, steady_clock::now()));    //记录PMC报警产生时间

    m_error_info_input_list->WriteData(error_info, 1);

    g_ptr_trace->PrintAlarm(error_info);
    g_ptr_trace->PrintLog(LOG_ALARM, "ALARM:%d, %d, %d, %d",
        error_info->error_code, error_info->channel_index, error_info->axis_index, error_info->error_info);
    pthread_mutex_unlock(&m_mutex);
    printf("AlarmProcessor unlocked!!!\n");

    NotifyToHmi();
    Singleton<AxisStatusCtrl>::instance().UpdateServoState();
    return;
}

/*
 * @brief 读取错误信息
 * @param error_info 错误信息数据结构指针，用来存放读取第错误信息
 * @return 错误信息个数
 */
int AlarmProcessor::GetErrorInfo(ErrorInfo* error_info) {
    assert(error_info != NULL);

    pthread_mutex_lock(&m_mutex);

    int err_num = m_error_info_input_list->BufLen();

    if (err_num) {
        m_error_info_input_list->ReadData(error_info, err_num);
        memcpy(&m_latest_error, error_info + err_num - 1, sizeof(ErrorInfo)); //保存最新的错误信息
    }

    pthread_mutex_unlock(&m_mutex);
    return err_num;
}

/**
 * @brief 读取通道内所有错误信息
 * @param chn_index 通道号
 * @return 通道内的所有错误信息
 */
deque<ErrorInfo> AlarmProcessor::GetErrorInfo()
{
    ErrorInfo *err = nullptr;
    deque<ErrorInfo> infoList;

    pthread_mutex_lock(&m_mutex);
    int err_num = m_error_info_input_list->BufLen();
    for(int i = 0; i < err_num; ++i)
    {
        err = m_error_info_input_list->ReadDataPtr(i);
        infoList.push_back(*err);
    }
    pthread_mutex_unlock(&m_mutex);

    return infoList;
}

/**
 * @brief 判断是否有出错信息
 * @param level 错误等级
 * @param reSend 重发错误
 * @return 处理结果,true-有未处理的出错信息;false-无未处理的出错信息
 */
bool AlarmProcessor::HasErrorInfo(int level) {
    pthread_mutex_lock(&m_mutex);
    bool res = false;

    ErrorInfo *err = nullptr;
    int count = m_error_info_input_list->BufLen();

    if (count > 0) {
        for(int i = 0; i < count; i++){
            err = m_error_info_input_list->ReadDataPtr(i);
            if(err != nullptr){

                if(err->error_level <= level){ //错误级别
                    res = true;
                    break;
                }
            }
        }
    } else {
        res = false;
    }

    pthread_mutex_unlock(&m_mutex);
    return res;
}

/**
 * @brief 判断是否有属于指定通道的告警，通道引擎的告警属于任意通道
 * @param chn_index ： 指定通道
 * @param reSend: 是否重发错误
 * @return 处理结果,true-有未处理的出错信息;false-无未处理的出错信息
 */
bool AlarmProcessor::HasErrorInfo(uint8_t chn_index){
    pthread_mutex_lock(&m_mutex);
    bool res = false;

    ErrorInfo *err = nullptr;
    int count = m_error_info_input_list->BufLen();

    if(count == 0){
        res = false;
    }else{
        for(int i = 0; i < count; i++){
            err = m_error_info_input_list->ReadDataPtr(i);
            if(err != nullptr){
                if(err->channel_index == chn_index || err->channel_index == CHANNEL_ENGINE_INDEX){
                    if(err->error_level <= ERROR_LEVEL){ //错误级别
                        res = true;
                        break;
                    }
                }
            }
        }
    }

    pthread_mutex_unlock(&m_mutex);
    return res;
}

/**
 * @brief 判断是否有重复告警
 * @param error_code : 错误码
 * @param error_level ：错误级别
 * @param clear_type ： 清除方法
 * @param error_info ：附带信息
 * @param channel_index ： 通道号
 * @param axis_index ：轴号
 * @return
 */
bool AlarmProcessor::ContainErrorInfo(uint16_t error_code, uint8_t error_level,
        uint8_t clear_type, int32_t error_info, uint8_t channel_index, uint8_t axis_index)
{
    bool res = false;
    pthread_mutex_lock(&m_mutex);
    ErrorInfo *info;
    int count = m_error_info_input_list->BufLen();
    for(int i = 0; i < count; i++){
        info = m_error_info_input_list->ReadDataPtr(i);
        if(info == nullptr)
            continue;
//		printf("error[%hhu, %hhu, %hhu, %hu, %hhu, %u], info[%hhu, %hhu, %hhu, %hu, %hhu, %u]\n", axis_index, channel_index, clear_type, error_code,
//				error_level, error_info, info->axis_index, info->channel_index, info->clear_type, info->error_code, info->error_level,info->error_info);
        if(info->axis_index == axis_index && info->channel_index == channel_index &&
            info->clear_type == clear_type && info->error_code == error_code &&
            info->error_level == error_level && info->error_info == error_info){
            res = true;
            if (clear_type == CLEAR_BY_AUTO)
            {
                UpdateAutoAlarm(error_code, error_level, clear_type, error_info, channel_index, axis_index);
            }

            break;
        }
    }

    pthread_mutex_unlock(&m_mutex);
//	printf("AlarmProcessor::HasErrorInfo, return %hhu\n", (uint8_t)res);
    return res;
}

/**
 * @brief 读取最新的错误信息
 * @param error_info 错误信息数据结构指针，用来存放读取第错误信息
 * @return 处理结果,0--处理成功;非0--错误码
 */
int AlarmProcessor::GetLatestErrorInfo(ErrorInfo* error_info) {
    pthread_mutex_lock(&m_mutex);
    int res = ERR_NONE;

    assert(error_info != NULL);
    memcpy(error_info, &m_latest_error, sizeof(ErrorInfo));

    pthread_mutex_unlock(&m_mutex);
    return res;
}

/**
 * @brief 获取最近的错误代码
 * @return 错误代码
 */
uint16_t AlarmProcessor::GetLatestErrorCode(){
    pthread_mutex_lock(&m_mutex);

    return m_latest_error.error_code;

    pthread_mutex_unlock(&m_mutex);
}

/**
 * @brief 打印输出所有错误信息
 */
void AlarmProcessor::PrintError(){
    pthread_mutex_lock(&m_mutex);

    ErrorInfo *info;
    int count = m_error_info_input_list->BufLen();
    for(int i = 0; i < count; i++){
        info = m_error_info_input_list->ReadDataPtr(i);
        if(info == nullptr)
            continue;
        printf("error info[%d]: chn=%hhu, axis=%hhu, err_code=%hu, err_info=0x%x\n", i,
                info->channel_index, info->axis_index, info->error_code, info->error_info);
    }

    pthread_mutex_unlock(&m_mutex);
}

/**
 * @brief  清空告警队列
 */
void AlarmProcessor::Clear(){

    vector<int> delVec;
    ErrorInfo *info;

    pthread_mutex_lock(&m_mutex);

    //this->m_error_info_input_list->ClearBuf();


    int count = m_error_info_input_list->BufLen();

    for(int i = 0; i < count; i++){
        info = m_error_info_input_list->ReadDataPtr(i);
        if(info == nullptr)
            continue;
        if(info->clear_type == CLEAR_BY_MCP_RESET)
            delVec.push_back(i);

    }

    for(auto itr = delVec.rbegin(); itr != delVec.rend(); ++itr)
        m_error_info_input_list->RemoveData(*itr);

    pthread_mutex_unlock(&m_mutex);
    Singleton<AxisStatusCtrl>::instance().UpdateServoState();
    if (delVec.size())
        NotifyToHmi();
}

/**
 * @brief  清空提示信息
 */
void AlarmProcessor::ClearTips()
{
    vector<int> delVec;
    ErrorInfo *info;

    pthread_mutex_lock(&m_mutex);

    int count = m_error_info_input_list->BufLen();

    for(int i = 0; i < count; i++){
        info = m_error_info_input_list->ReadDataPtr(i);
        if(info == nullptr)
            continue;
        if(info->error_level == INFO_LEVEL)
            delVec.push_back(i);
    }

    for(auto itr = delVec.rbegin(); itr != delVec.rend(); ++itr)
        m_error_info_input_list->RemoveData(*itr);

    pthread_mutex_unlock(&m_mutex);
    Singleton<AxisStatusCtrl>::instance().UpdateServoState();
    if (delVec.size())
        NotifyToHmi();
}

/**
 * @brief 清空告警即以下等级的消息
 * @param chn : 通道号
 */
void AlarmProcessor::ClearWarning(uint8_t chn){
    pthread_mutex_lock(&m_mutex);

    ErrorInfo *info;
    int count = m_error_info_input_list->BufLen();
    int del_count = 0;
    for(int i = 0; i < count; i++){
        info = m_error_info_input_list->ReadDataPtr(i);
        if(info == nullptr)
            continue;
        if((chn == info->channel_index || chn == 0xFF) && info->error_level >= WARNING_LEVEL){
            m_error_info_input_list->RemoveData(i);
            count--;
            i--;
            del_count++;
        }
    }

    pthread_mutex_unlock(&m_mutex);
    if (del_count)
        NotifyToHmi();
    Singleton<AxisStatusCtrl>::instance().UpdateServoState();
//	printf("clear warning:%d\n", del_count);
}

/**
 * @brief 清空指定告警
 * @param chn
 * @param error_code
 */
void AlarmProcessor::RemoveWarning(uint8_t chn, uint16_t error_code){
    pthread_mutex_lock(&m_mutex);

    ErrorInfo *info;
    int count = m_error_info_input_list->BufLen();
    int del_count = 0;
    for(int i = 0; i < count; i++){
        info = m_error_info_input_list->ReadDataPtr(i);
        if(info == nullptr)
            continue;
        if((chn == info->channel_index || chn == 0xFF) && info->error_code == error_code){
            m_error_info_input_list->RemoveData(i);
            count--;
            i--;
            del_count++;
        }
    }

    pthread_mutex_unlock(&m_mutex);
    Singleton<AxisStatusCtrl>::instance().UpdateServoState();
    //	printf("remove warning:%d\n", del_count);
}

vector<ErrorInfo> AlarmProcessor::GetWarningList()
{
    vector<ErrorInfo> vecInfo;
    pthread_mutex_lock(&m_mutex);
    int err_num = m_error_info_input_list->BufLen();
    for(int i = 0; i < err_num; ++i)
    {
        ErrorInfo *infoPtr = m_error_info_input_list->ReadDataPtr(i);
        ErrorInfo info;
        info.time = infoPtr->time;
        info.error_info = infoPtr->error_code;
        info.error_code = infoPtr->error_code;
        info.channel_index = infoPtr->channel_index;
        info.axis_index = infoPtr->axis_index;
        info.error_level = infoPtr->error_level;
        info.clear_type = infoPtr->clear_type;
        vecInfo.push_back(info);
    }
    pthread_mutex_unlock(&m_mutex);
    return vecInfo;
}


/**
 * @brief 设置接口
 * @param pHmiComm ： HMI通讯接口
 * @param pEngine : 通道引擎接口
 */
void AlarmProcessor::SetInterfaces(){
    this->m_p_hmi_comm = HMICommunication::GetInstance();
    this->m_p_chn_engine = ChannelEngine::GetInstance();
}

/**
 * @brief 将告警信息发送至HMI
 * @param err
 * @return
 */
bool AlarmProcessor::SendToHmi(ErrorInfo *err){
    assert(err != nullptr);
    HMICmdFrame hmi_cmd;

    hmi_cmd.channel_index = err->channel_index;
    hmi_cmd.cmd = CMD_SC_NEW_ALARM;
    hmi_cmd.cmd_extension = 0;
    hmi_cmd.data_len = sizeof(ErrorInfo);
    memcpy(hmi_cmd.data, err, hmi_cmd.data_len);

    m_p_hmi_comm->SendCmd(hmi_cmd);

//	printf("ALARM %d\n", err->error_code);
    return true;
}

/**
 * @brief 将当前错误都输出到HMI
 * @return
 */
void AlarmProcessor::SendToHmi(){

    pthread_mutex_lock(&m_mutex);

    ErrorInfo *info;
    int count = m_error_info_input_list->BufLen();
    for(int i = 0; i < count; i++){
        info = m_error_info_input_list->ReadDataPtr(i);
        if(info == nullptr)
            continue;

        this->SendToHmi(info);
    }

    pthread_mutex_unlock(&m_mutex);
}

void AlarmProcessor::NotifyToHmi()
{
    HMICmdFrame hmi_cmd;
    hmi_cmd.channel_index = CHANNEL_ENGINE_INDEX;
    hmi_cmd.cmd = CMD_SC_NOTIFY_ALARM_CHANGE;
    hmi_cmd.cmd_extension = 0;
    hmi_cmd.data_len = 0;

    auto infolist = GetErrorInfo();
    size_t infoSize = sizeof(ErrorInfo);
    for (auto itr = infolist.begin(); itr != infolist.end(); ++itr)
    {
        memcpy(hmi_cmd.data + hmi_cmd.data_len, &*itr, infoSize);
        int data_len = hmi_cmd.data_len + infoSize;
        if (data_len < kMaxHmiDataLen)
            hmi_cmd.data_len = data_len;
        else
            break;
    }

    m_p_hmi_comm->SendCmd(hmi_cmd);
    std::cout << "Notify To Hmi, Error Lists Changed" << std::endl;

//	printf("ALARM %d\n", err->error_code);
}

/**
 * @brief 告警的统一处理函数
 * @param err : 错误信息结构指针
 */
void AlarmProcessor::ProcessAlarm(ErrorInfo *err){
    assert(err != nullptr);

    if(err->error_level <= ERROR_LEVEL){
        m_p_chn_engine->ResetOPSignal(err->channel_index);
        m_p_chn_engine->SetALSignal(err->channel_index, true);
    }

    switch(err->error_code){
    case ERR_HEARTBEAT_HMI_LOST: //HMI心跳丢失
    case ERR_HMI_MONITOR_DISCON: //HMI监控连接中断

        m_p_hmi_comm->DisconnectToHmi();  //关闭HMI连接
        break;
    case ERR_FILE_TRANS:	//文件传输失败
        m_p_hmi_comm->ProcessFileTransError();   //处理文件传输失败
        break;
    case ERR_ENCODER:    //编码器错误
    case ERR_SERVO:     //伺服告警
    case ERR_SOFT_LIMIT_NEG:     //负向软限位告警
    case ERR_SOFT_LIMIT_POS: 			//正向软限位告警
    case ERR_POS_ERR:					//位置指令过大告警
    case ERR_ARC_DATA:					//圆弧数据错误告警
    case ERR_CMD_CRC:					//指令校验错
    case ERR_DATA_CRC:					//数据校验错
    case ERR_OPEN_FILE:                //打开文件失败

        this->m_p_chn_engine->Stop(false);
        break;
    case ERR_AXIS_CTRL_MODE_SWITCH:    //轴控制模式切换超时
        this->m_p_chn_engine->Stop(err->channel_index, false);
        break;
#ifdef USES_GRIND_MACHINE
    case ERR_WORK_VAC_OPEN:			//工位吸真空超时
    case ERR_WORK_VAC_CLOSE:		//工位破真空超时
    case ERR_WORK_CYLINDER_UP:		//工位气缸上升超时
    case ERR_WORK_CYLINDER_DOWN:   //工位气缸下降超时
        this->m_p_chn_engine->Stop(err->channel_index, false);
        break;
#endif
    default:

#ifdef USES_PMC_2_0
        if(err->error_code >=20000 && err->error_code < 20999){
        }
        else if(err->error_code >=21000 && err->error_code < 21999){
        }
        else if (err->error_code >= 22000 && err->error_code < 22999) {
            this->m_p_chn_engine->Stop(false);   //终止加工
        }
        else if (err->error_code >= 23000 && err->error_code < 23999) {
            this->m_p_chn_engine->Stop(false);  //急停处理
        }

#else
        if(err->error_code >=20000 && err->error_code < 20240){ //PMC的A地址提示信息 A0~A29

        }else if(err->error_code >= 20240 && err->error_code < 20400){

        }else if(err->error_code >=20240 && err->error_code < 20400){ //PMC的A地址告警信息 A30~A49

        }else if(err->error_code >=20400 && err->error_code < 20480){ //PMC的A地址错误信息 A50~A59
            this->m_p_chn_engine->Stop(false);   //终止加工

        }else if(err->error_code >=20480 && err->error_code < 20512){ //PMC的A地址严重错误信息 A60~A63
            //通知MI下伺服
            this->m_p_chn_engine->ServoOff();
            this->m_p_chn_engine->Stop(false);  //急停处理
#endif
        else if(err->error_code >= 30000 && err->error_code < 40000){//总线通讯告警
            this->m_p_chn_engine->Stop(false);
        }
        break;
    }
}

/**
 * @brief 刷新自动类型告警的状态
 * @param error_code : 错误码
 * @param error_level ：错误级别
 * @param clear_type ： 清除方法
 * @param error_info ：附带信息
 * @param channel_index ： 通道号
 * @param axis_index ：轴号
 * @return
 */
void AlarmProcessor::UpdateAutoAlarm(uint16_t error_code, uint8_t error_level, uint8_t clear_type, int32_t error_info, uint8_t channel_index, uint8_t axis_index)
{
    ErrorInfo err_info;
    memset(&err_info, 0x00, sizeof(ErrorInfo));
    err_info.channel_index = channel_index;
    err_info.axis_index = axis_index;
    err_info.error_code = (uint16_t) error_code;
    err_info.error_level = (uint8_t) error_level;
    err_info.clear_type = (uint8_t) clear_type;
    err_info.error_info = error_info;

    auto itrRet = find_if(m_auto_error_list.begin(), m_auto_error_list.end(), [err_info](const ErrorPair &elem){
        return err_info == elem.first;
    });

    if (itrRet != m_auto_error_list.end())
    {// 存在于列表中就更新时间
        itrRet->second = steady_clock::now();
    }
}

/**
 * @brief 处理自动类型的告警
 * @param error_code : 错误码
 * @param error_level ：错误级别
 * @param clear_type ： 清除方法
 * @param error_info ：附带信息
 * @param channel_index ： 通道号
 * @param axis_index ：轴号
 * @return
 */
void AlarmProcessor::ProcessAutoAlarm()
{
    pthread_mutex_lock(&m_mutex);
    int size = m_auto_error_list.size();
    auto beginItr = m_auto_error_list.begin();
    pthread_mutex_unlock(&m_mutex);

    if (size)
    {
        time_point<steady_clock> temp_clock = steady_clock::now();
        ErrorInfo info = beginItr->first;
        int timeMs = duration_cast<milliseconds>(temp_clock - beginItr->second).count();
        if (timeMs > IMM_INTERVAL)
        {
            pthread_mutex_lock(&m_mutex);
            m_auto_error_list.pop_front();
            int count = m_error_info_input_list->BufLen();

            for(int i = 0; i < count; i++){
                ErrorInfo *err = m_error_info_input_list->ReadDataPtr(i);
                if (*err == info)
                {
                    m_error_info_input_list->RemoveData(i);
                    std::cout << "clear pmc alarm info: " << info.error_code << std::endl;

                    bool hasError = false;
                    count = m_error_info_input_list->BufLen();
                    for (int j = 0; j < count; ++j) {
                        ErrorInfo *err = m_error_info_input_list->ReadDataPtr(j);
                        if (err->error_level <= ERROR_LEVEL)
                        {
                            hasError = true;
                            break;
                        }
                    }
                    if (!hasError)
                    {
                        for(int i = 0; i < g_ptr_chn_engine->GetChnCount(); ++i)
                        {
                            g_ptr_chn_engine->GetChnControl()[i].Pmc_AutoReset();
                        }
                    }

                    break;
                }
                else
                {
                    std::cout << "error " << " error_code: " << (uint16_t)err->error_code << " error_info " << (int32_t)err->error_info
                        << " err_level : " << (int)err->error_level << " channel_index: " << (int)err->channel_index
                        << " axis_index: " << (int)err->axis_index << " clear_type: " << (int)err->clear_type << std::endl;

                    //std::cout << "info " << " error_code: " << (uint16_t)info.error_code << " error_info " << (int32_t)info.error_info
                    //    << " err_level : " << (int)info.error_level << " channel_index: " << (int)info.channel_index
                    //    << " axis_index: " << (int)info.axis_index << " clear_type: " << (int)info.clear_type << std::endl;
                }
            }

            pthread_mutex_unlock(&m_mutex);
            NotifyToHmi();

        }
        else
        {
            pthread_mutex_lock(&m_mutex);
            m_auto_error_list.push_back(m_auto_error_list.front());
            m_auto_error_list.pop_front();
            pthread_mutex_unlock(&m_mutex);
        }
    }
}


TraceLogProcess::~TraceLogProcess()
{

}

void TraceLogProcess::SetInterfaces()
{
    this->m_p_hmi_comm = HMICommunication::GetInstance();
}

void TraceLogProcess::SendToHmi(OptType optType, MsgType msgType, string strMsg)
{
    HMICmdFrame hmi_cmd;
    hmi_cmd.channel_index = CHANNEL_ENGINE_INDEX;
    hmi_cmd.cmd = CMD_SC_NOTIFY_TRACELOG;
    hmi_cmd.cmd_extension = 0;
    hmi_cmd.data_len = 0;

    int opt = optType;
    int msg = msgType;
    int len = sizeof(int);
    int strLen = strMsg.size();
    if (strLen > 512)
        strLen = 512;

    memset(&hmi_cmd.data[0], 0, kMaxHmiDataLen);
    memcpy(&hmi_cmd.data[0], &opt, len);
    memcpy(&hmi_cmd.data[0+len], &msg, len);
    strcpy(&hmi_cmd.data[0+2*len], strMsg.c_str());

    hmi_cmd.data_len = 2*len + strLen;

    std::cout << "type: " << (int)msgType << "content: " << strMsg << std::endl;
    m_p_hmi_comm->SendCmd(hmi_cmd);
}

TraceLogProcess::TraceLogProcess()
{

}
