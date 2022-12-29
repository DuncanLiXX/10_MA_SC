
/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file alarm_processor.cpp
 *@author gonghao
 *@date 2020/05/15
 *@brief ��ͷ�ļ�Ϊ�澯�������ʵ��
 *@version
 */
#include "alarm_processor.h"
#include "hmi_communication.h"
#include "channel_engine.h"
#include "axis_status_ctrl.h"

const int kMaxAlarmCount = 200;   //�澯���������ֵ

//m_instance��ʼ��Ϊnullptr
AlarmProcessor* AlarmProcessor::m_instance = nullptr;
TraceLogProcess* TraceLogProcess::m_instance = nullptr;

//�������๹�캯��
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
 * @brief �澯��������������
 * @param ��
 * @return ��
 */
AlarmProcessor::~AlarmProcessor() {
    if (m_error_info_input_list != nullptr) {
        delete m_error_info_input_list;
        m_error_info_input_list = nullptr;
    }


    pthread_mutex_destroy(&m_mutex);
}

/*
 * @brief д�������Ϣ
 * @param error_info ��Ҫд��ڴ�����Ϣָ��
 * @return ��
 */
void AlarmProcessor::SetErrorInfo(ErrorInfo* error_info) {
//	printf("enter AlarmProcessor::SetErrorInfo()\n");
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
        m_auto_error_list.push_back(make_pair(*error_info, steady_clock::now()));    //��¼PMC��������ʱ��

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
 * @brief ��ȡ������Ϣ
 * @param error_info ������Ϣ���ݽṹָ�룬������Ŷ�ȡ�ڴ�����Ϣ
 * @return ������Ϣ����
 */
int AlarmProcessor::GetErrorInfo(ErrorInfo* error_info) {
    assert(error_info != NULL);

    pthread_mutex_lock(&m_mutex);

    int err_num = m_error_info_input_list->BufLen();

    if (err_num) {
        m_error_info_input_list->ReadData(error_info, err_num);
        memcpy(&m_latest_error, error_info + err_num - 1, sizeof(ErrorInfo)); //�������µĴ�����Ϣ
    }

    pthread_mutex_unlock(&m_mutex);
    return err_num;
}

/**
 * @brief ��ȡͨ�������д�����Ϣ
 * @param chn_index ͨ����
 * @return ͨ���ڵ����д�����Ϣ
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
 * @brief �ж��Ƿ��г�����Ϣ
 * @param level ����ȼ�
 * @param reSend �ط�����
 * @return ������,true-��δ����ĳ�����Ϣ;false-��δ����ĳ�����Ϣ
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

                if(err->error_level <= level){ //���󼶱�
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
 * @brief �ж��Ƿ�������ָ��ͨ���ĸ澯��ͨ������ĸ澯��������ͨ��
 * @param chn_index �� ָ��ͨ��
 * @param reSend: �Ƿ��ط�����
 * @return ������,true-��δ����ĳ�����Ϣ;false-��δ����ĳ�����Ϣ
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
                    if(err->error_level <= ERROR_LEVEL){ //���󼶱�
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
 * @brief �ж��Ƿ����ظ��澯
 * @param error_code : ������
 * @param error_level �����󼶱�
 * @param clear_type �� �������
 * @param error_info ��������Ϣ
 * @param channel_index �� ͨ����
 * @param axis_index �����
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
 * @brief ��ȡ���µĴ�����Ϣ
 * @param error_info ������Ϣ���ݽṹָ�룬������Ŷ�ȡ�ڴ�����Ϣ
 * @return ������,0--����ɹ�;��0--������
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
 * @brief ��ȡ����Ĵ������
 * @return �������
 */
uint16_t AlarmProcessor::GetLatestErrorCode(){
    pthread_mutex_lock(&m_mutex);

    return m_latest_error.error_code;

    pthread_mutex_unlock(&m_mutex);
}

/**
 * @brief ��ӡ������д�����Ϣ
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
 * @brief  ��ո澯����
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
 * @brief ��ո澯�����µȼ�����Ϣ
 * @param chn : ͨ����
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
 * @brief ���ָ���澯
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
 * @brief ���ýӿ�
 * @param pHmiComm �� HMIͨѶ�ӿ�
 * @param pEngine : ͨ������ӿ�
 */
void AlarmProcessor::SetInterfaces(){
    this->m_p_hmi_comm = HMICommunication::GetInstance();
    this->m_p_chn_engine = ChannelEngine::GetInstance();
}

/**
 * @brief ���澯��Ϣ������HMI
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
 * @brief ����ǰ���������HMI
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
 * @brief �澯��ͳһ������
 * @param err : ������Ϣ�ṹָ��
 */
void AlarmProcessor::ProcessAlarm(ErrorInfo *err){
    assert(err != nullptr);

    if(err->error_level <= ERROR_LEVEL){
        m_p_chn_engine->ResetOPSignal(err->channel_index);
        m_p_chn_engine->SetALSignal(err->channel_index, true);
    }

    switch(err->error_code){
    case ERR_HEARTBEAT_HMI_LOST: //HMI������ʧ
    case ERR_HMI_MONITOR_DISCON: //HMI��������ж�

        m_p_hmi_comm->DisconnectToHmi();  //�ر�HMI����
        break;
    case ERR_FILE_TRANS:	//�ļ�����ʧ��
        m_p_hmi_comm->ProcessFileTransError();   //�����ļ�����ʧ��
        break;
    case ERR_ENCODER:    //����������
    case ERR_SERVO:     //�ŷ��澯
    case ERR_SOFT_LIMIT_NEG:     //��������λ�澯
    case ERR_SOFT_LIMIT_POS: 			//��������λ�澯
    case ERR_POS_ERR:					//λ��ָ�����澯
    case ERR_ARC_DATA:					//Բ�����ݴ���澯
    case ERR_CMD_CRC:					//ָ��У���
    case ERR_DATA_CRC:					//����У���
    case ERR_OPEN_FILE:                //���ļ�ʧ��

        this->m_p_chn_engine->Stop(false);
        break;
    case ERR_AXIS_CTRL_MODE_SWITCH:    //�����ģʽ�л���ʱ
        this->m_p_chn_engine->Stop(err->channel_index, false);
        break;
#ifdef USES_GRIND_MACHINE
    case ERR_WORK_VAC_OPEN:			//��λ����ճ�ʱ
    case ERR_WORK_VAC_CLOSE:		//��λ����ճ�ʱ
    case ERR_WORK_CYLINDER_UP:		//��λ����������ʱ
    case ERR_WORK_CYLINDER_DOWN:   //��λ�����½���ʱ
        this->m_p_chn_engine->Stop(err->channel_index, false);
        break;
#endif
    default:

#ifdef USES_PMC_2_0
        if(err->error_code >=20000 && err->error_code < 20080){
        }
        else if(err->error_code >=20080 && err->error_code < 20144){
        }
        else if (err->error_code >= 20144 && err->error_code < 20184) {
            this->m_p_chn_engine->Stop(false);   //��ֹ�ӹ�
        }
        else if (err->error_code >= 20184 && err->error_code < 20199) {
            this->m_p_chn_engine->Stop(false);  //��ͣ����
        }

#else
        if(err->error_code >=20000 && err->error_code < 20240){ //PMC��A��ַ��ʾ��Ϣ A0~A29

        }else if(err->error_code >= 20240 && err->error_code < 20400){

        }else if(err->error_code >=20240 && err->error_code < 20400){ //PMC��A��ַ�澯��Ϣ A30~A49

        }else if(err->error_code >=20400 && err->error_code < 20480){ //PMC��A��ַ������Ϣ A50~A59
            this->m_p_chn_engine->Stop(false);   //��ֹ�ӹ�

        }else if(err->error_code >=20480 && err->error_code < 20512){ //PMC��A��ַ���ش�����Ϣ A60~A63
            //֪ͨMI���ŷ�
            this->m_p_chn_engine->ServoOff();
            this->m_p_chn_engine->Stop(false);  //��ͣ����
#endif
        else if(err->error_code >= 30000 && err->error_code < 40000){//����ͨѶ�澯
            this->m_p_chn_engine->Stop(false);
        }
        break;
    }
}

/**
 * @brief ˢ���Զ����͸澯��״̬
 * @param error_code : ������
 * @param error_level �����󼶱�
 * @param clear_type �� �������
 * @param error_info ��������Ϣ
 * @param channel_index �� ͨ����
 * @param axis_index �����
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
    {// �������б��о͸���ʱ��
        itrRet->second = steady_clock::now();
    }
}

/**
 * @brief �����Զ����͵ĸ澯
 * @param error_code : ������
 * @param error_level �����󼶱�
 * @param clear_type �� �������
 * @param error_info ��������Ϣ
 * @param channel_index �� ͨ����
 * @param axis_index �����
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
                    break;
                }
                else
                {
                    std::cout << "error " << " error_code: " << (uint16_t)err->error_code << " error_info " << (int32_t)err->error_info
                        << " err_level : " << (int)err->error_level << " channel_index: " << (int)err->channel_index
                        << " axis_index: " << (int)err->axis_index << " clear_type: " << (int)err->clear_type << std::endl;

                    std::cout << "info " << " error_code: " << (uint16_t)info.error_code << " error_info " << (int32_t)info.error_info
                        << " err_level : " << (int)info.error_level << " channel_index: " << (int)info.channel_index
                        << " axis_index: " << (int)info.axis_index << " clear_type: " << (int)info.clear_type << std::endl;
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

    m_p_hmi_comm->SendCmd(hmi_cmd);
}

TraceLogProcess::TraceLogProcess()
{

}
