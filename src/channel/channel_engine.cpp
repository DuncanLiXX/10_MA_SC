/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file ChannelEngine.cpp
 *@author gonghao
 *@date 2020/03/19
 *@brief 本文件为通道引擎类的实现
 *@version
 */

#include <dirent.h>
#include <channel_engine.h>
#include "channel_control.h"
#include "parm_manager.h"
#include "hmi_communication.h"
#include "mi_communication.h"
#include "mc_communication.h"
#include "mc_communication_arm.h"
#include "pmc_register.h"
#include "alarm_processor.h"
#include "channel_mode_group.h"
#include "spindle_control.h"
#include "showsc.h"
#include "sync_axis_ctrl.h"
#include "axis_status_ctrl.h"
#include <future>
#include <functional>


//#include <unistd.h>
//#include <stropts.h>

using namespace Spindle;

ChannelEngine* ChannelEngine::m_p_instance = nullptr;  //初始化单例对象指正为空
const map<int, SDLINK_SPEC> ChannelEngine::m_SDLINK_MAP =
{
    //id.SA1, 3.SC1, 4.SD1, 5.SE1
    //对于有手轮扩展的板卡，梯图默认分配了手轮空间，需要去掉，要使用手轮时，再由SC分配
    //id    name    inbytes outBytes withHandWheel
    {1,     {"SA1",  9-3,    4,      true}},
    {3,     {"SC1",  16,     16,     false}},
    {4,     {"SD1",  12-3,   8,      true}},
    {5,     {"SE1",  7-3,    8,      true}}
};

const uint32_t MC_UPDATE_BLOCK_SIZE = 100;		//MC升级文件帧，每帧包含100字节数据,50个uint16

/**
 * @brief 构造函数
 */
ChannelEngine::ChannelEngine() {
    // TODO Auto-generated constructor stub

    m_p_channel_mode_group = nullptr;
    m_p_channel_control = nullptr;
    m_p_hmi_comm = nullptr;
    m_p_general_config = nullptr;
    m_p_channel_config = nullptr;
    m_df_phy_axis_pos_feedback = nullptr;
    m_df_phy_axis_pos_intp = nullptr;
    m_df_phy_axis_pos_intp_after = nullptr;
    m_df_pmc_axis_remain = nullptr;

    this->m_p_io_remap = nullptr;

    //#ifdef USES_SPEED_TORQUE_CTRL
    m_df_phy_axis_speed_feedback = nullptr;
    m_df_phy_axis_torque_feedback = nullptr;
    //#endif

    m_p_pmc_reg = nullptr;

    memset(this->m_device_sn, 0x00, SN_COUNT+1);
    strcpy(this->m_device_sn, "LIT0123456789");   //默认序列号"LIT0123456789"
    this->m_lic_info.InitData();     //初始化数据


    m_thread_update = 0;

    m_error_code = ERR_NONE;
    m_n_cur_channle_index = 0;
    m_n_cur_chn_group_index = 0;

    m_b_recv_mi_heartbeat = false;
    m_b_init_mi_over = false;
    m_b_emergency = false;
    m_b_power_off = false;
    m_b_reset_rst_signal = false;

    this->m_n_cur_pmc_axis = 0xFF;      //默认没有当前轴
    this->m_pmc_axis_active_mask = 0;   //默认所有轴都没有激活PMC轴

    this->m_hard_limit_negative = 0;
    this->m_hard_limit_postive = 0;

    this->m_n_pmc_axis_count = 0;
    m_n_run_axis_mask = 0x00;  //当前运行的轴的mask
    m_n_runover_axis_mask = 0x00;   //当前运行完成的轴的mask

    m_mask_import_param = 0;    //初始化导入参数标志

    //	m_n_mc_auto_buf_max = 200;   //默认200

    //初始化回参考点变量
    this->m_b_ret_ref = false;
    this->m_b_ret_ref_auto = false;
    m_n_mask_ret_ref_over = 0;
    this->m_n_mask_ret_ref = 0;
    memset(this->m_n_ret_ref_step, 0x00, kMaxAxisNum*sizeof(int));

    //初始化mc通道运行标志
    m_mc_run_on_arm[0] = false;
    for(uint16_t i=1; i<kMaxChnCount; i++)
        m_mc_run_on_arm[i] = false;

    m_mc_run_dsp_flag = false;
    m_mc_run_mi_flag = false;

    this->CheckTmpDir();

    this->InitBdioDev();

    ShowSc& showSc = Singleton<ShowSc>::instance();
    showSc.SetPrintType(TypePrintOutput);
    showSc.SetInterval(200);
}

/**
 * @brief 析构函数
 */
ChannelEngine::~ChannelEngine(){
    // TODO Auto-generated destructor stub

    void* thread_result;
    int res = ERR_NONE;

    //退出编译运行线程
    if(this->m_thread_update != 0){
        res = pthread_cancel(m_thread_update);
        if (res != ERR_NONE) {
            printf("Update thread cancel failed\n");
        }

        //	usleep(1000);
        res = pthread_join(m_thread_update, &thread_result);//等待编译器线程退出完成
        if (res != ERR_NONE) {
            printf("Update thread join failed\n");
        }
        m_thread_update = 0;
    }

    //销毁方式组对象
    if(this->m_p_channel_mode_group){
        delete []m_p_channel_mode_group;
        this->m_p_channel_mode_group = nullptr;
    }

    //销毁通道控制对象
    if(m_p_channel_control != nullptr){
        delete []m_p_channel_control;
        m_p_channel_control = nullptr;
    }

    if(m_df_phy_axis_pos_feedback){
        delete []m_df_phy_axis_pos_feedback;
        m_df_phy_axis_pos_feedback = nullptr;
    }

    if(m_df_phy_axis_pos_intp){
        delete []m_df_phy_axis_pos_intp;
        m_df_phy_axis_pos_intp = nullptr;
    }

    if(m_df_phy_axis_pos_intp_after){
        delete []m_df_phy_axis_pos_intp_after;
        m_df_phy_axis_pos_intp_after = nullptr;
    }

    if(m_df_pmc_axis_remain){
        delete []m_df_pmc_axis_remain;
        m_df_pmc_axis_remain = nullptr;
    }

    if(m_df_phy_axis_speed_feedback){
        delete []m_df_phy_axis_speed_feedback;
        m_df_phy_axis_speed_feedback = nullptr;
    }

    if(m_df_phy_axis_torque_feedback){
        delete []m_df_phy_axis_torque_feedback;
        m_df_phy_axis_torque_feedback = nullptr;
    }

    //释放PMC寄存器对象
    if(this->m_p_pmc_reg){
        delete m_p_pmc_reg;
        m_p_pmc_reg = nullptr;
    }
}

/**
 * @brief 获取类实例的唯一访问接口函数，单例模式
 */
ChannelEngine* ChannelEngine::GetInstance(){
    if(nullptr == m_p_instance){
        m_p_instance = new ChannelEngine();
    }
    return m_p_instance;
}

/**
 * @brief 初始化物理轴与通道的映射
 */
void ChannelEngine::InitPhyAxisChn(){
    memset(m_map_phy_axis_chn, 0xFF, kMaxAxisNum);
    for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
        for(uint8_t j = 0; j < this->m_p_channel_config[i].chn_axis_count; j++){
            if(m_p_channel_config[i].chn_axis_phy[j] == 0)
                continue;
            this->m_map_phy_axis_chn[m_p_channel_config[i].chn_axis_phy[j]-1] = i;
        }
    }
}

#ifdef USES_PMC_2_0
/**
 * @brief PMC2.0版本，读取SD_LINK从站设备配置
 */
void ChannelEngine::ReadIoDev_pmc2(){
    this->m_list_bdio_dev.Clear();

    std::ifstream ifs;
    ifs.open(PATH_PMC_LDR, std::ios::binary | std::ios::in);
    if (ifs.is_open())
    {
        //#SDLINKIO@# 从梯图文件中找到SD-LINK配置
        char readChar[11] = {0};
        while(!ifs.eof())
        {
            char ch = ifs.get();
            if (ch == '#')
            {
                ifs.read(&readChar[0], 10);
                if (!strcmp(readChar, "SDLINKIO@#"))
                    break;
            }
        }

        if (!ifs.eof())
        {
            auto ReadSequenceValue = [&]()->int {//从梯图中读取数据
                    int val = 0;
                    ifs.read(reinterpret_cast<char *>(&val), sizeof(val));
                    val = BigLittleSwitch32(val);
                    return val;
        };
            HandWheelMapInfoVec infoVec;
            bool selectedHandWheel = false;

            int num = ReadSequenceValue();
            for(int i = 0; i < num; ++i)
            {
                BdioDevInfo devInfo;

                devInfo.device_type = ReadSequenceValue();
                if (m_SDLINK_MAP.find(devInfo.device_type) != m_SDLINK_MAP.end())
                {
                    devInfo.group_index = ReadSequenceValue();
                    devInfo.base_index = ReadSequenceValue();
                    devInfo.slot_index = ReadSequenceValue();
                    devInfo.in_bytes = ReadSequenceValue();
                    devInfo.out_bytes = ReadSequenceValue();
                    devInfo.input_start = ReadSequenceValue();
                    devInfo.output_start = ReadSequenceValue();

                    SDLINK_SPEC sdlink_spec = m_SDLINK_MAP.at(devInfo.device_type);
                    if (devInfo.in_bytes && sdlink_spec.withHandWheel)//梯图默认分配了手轮空间，需要去掉，选择手轮时，再由SC分配
                    {
                        devInfo.in_bytes -= HANDWHEEL_BYTES;
                        ++devInfo.handwheel_num;
                    }
                    else
                    {
                        devInfo.handwheel_num = 0;
                    }
                    devInfo.handwheel_map = 0;
                    devInfo.info_name = sdlink_spec.info_name + "_" + std::to_string(devInfo.group_index);

                    // 序号检查,序号必须连续
                    if (devInfo.group_index != m_list_bdio_dev.GetLength() + 1)
                    {
                        m_list_bdio_dev.Clear();
                        std::cout << static_cast<int>(devInfo.group_index) << " not sequence " << std::endl;
                        CreateError(ERR_PMC_SDLINK_CONFIG, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
                        break;
                    }
                }

                if (!CheckIoDev_pmc2(devInfo))
                {
                    m_list_bdio_dev.Clear();
                    CreateError(ERR_PMC_SDLINK_CONFIG, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
                    break;
                }

                m_list_bdio_dev.Append(devInfo);
                if (!selectedHandWheel && SelectHandleWheel(devInfo.group_index, 1))
                {//默认使用第一个找到的手轮
                    selectedHandWheel = true;
                    devInfo.handwheel_map = 1;
                }
                HandWheelMapInfo info(devInfo.group_index, 0, devInfo.handwheel_map, devInfo.info_name);
                infoVec.push_back(info);
            }

            HandWheelMapInfoVec configInfo = g_ptr_parm_manager->GetHandWheelVec();
            if (configInfo == infoVec)
            {//梯图没有改变ini结构（扩展板卡个数或者基本配置没有发生变化），直接复制即可
                for (auto itr = configInfo.begin(); itr != configInfo.end(); ++itr)
                {
                    SelectHandleWheel(itr->devNum, itr->channelMap);
                }
            }
            else
            {//梯图改变了ini结构（扩展板卡个数或者基本配置发生变化），需要重写ini文件
                g_ptr_parm_manager->SyncHandWheelInfo(infoVec);
            }
        }

        //test
        ListNode<BdioDevInfo> *node = this->m_list_bdio_dev.HeadNode();
        while(node != nullptr)
        {
            std::cout << "info_name " << node->data.info_name << std::endl;
            std::cout << "device_type " << static_cast<int>(node->data.device_type) << std::endl;
            std::cout << "group " << static_cast<int>(node->data.group_index) << std::endl;
            std::cout << "base " << static_cast<int>(node->data.base_index) << std::endl;
            std::cout << "slot " << static_cast<int>(node->data.slot_index) << std::endl;
            std::cout << "inBytes " << static_cast<int>(node->data.in_bytes) << std::endl;
            std::cout << "outBytes " << static_cast<int>(node->data.out_bytes) << std::endl;
            std::cout << "inAddr " << static_cast<int>(node->data.input_start) << std::endl;
            std::cout << "outAddr " << static_cast<int>(node->data.output_start) << std::endl;
            std::cout << "handwheel_map " << static_cast<int>(node->data.handwheel_map) << std::endl;

            std::cout << std::endl;

            node = node->next;
        }
        ifs.close();
    }
    else
    {
        printf("初始化SD-LINK设备失败，无法打开梯图文件\n");
    }
    return;

}

/**
 * @brief PMC2.0版本，检测SD_LINK参数是否合理
 * @return true--合法     false--非法
 */
bool ChannelEngine::CheckIoDev_pmc2(const BdioDevInfo &info)
{
    // 设备类型检查
    if (m_SDLINK_MAP.find(info.device_type) == m_SDLINK_MAP.end())
    {
        std::cout << info.group_index << " config error, device_type: " << static_cast<int>(info.device_type) << std::endl;
        return false;
    }

    SDLINK_SPEC sdLink_spec = m_SDLINK_MAP.at(info.device_type);
    // 输入点检查
    int standard_InBytes = sdLink_spec.inBytes;
    if (info.handwheel_map != 0)
    {
        if (sdLink_spec.withHandWheel)
        {
            standard_InBytes += HANDWHEEL_BYTES;
        }
        else
        {   // 不支持手轮的板卡
            std::cout << info.group_index << " device_type " << static_cast<int>(info.device_type)
                      << " handwheel_map " << static_cast<int>(info.handwheel_map) << std::endl;
            return false;
        }

        if (info.handwheel_map != 1)//手轮映射暂时只支持通道1
        {
            std::cout << "handwheel_map " << static_cast<int>(info.handwheel_map) << std::endl;
            return false;
        }
    }
    int real_InBytes = info.in_bytes;
    if (real_InBytes)
    {
        if (standard_InBytes != real_InBytes)
        {
            std::cout << info.group_index << "SD_LINK config error, in_byetes: " << real_InBytes
                      << " stand: " << standard_InBytes << std::endl;
            return false;
        }

        if (info.input_start + real_InBytes - 1 > 127 || info.input_start < 0)
        {
            std::cout << info.group_index << "SD_LINK config error, input_start: " << static_cast<int>(info.input_start) << std::endl;
            return false;
        }
    }
    else if (info.input_start < 0 || info.input_start > 127)
    {
        std::cout << info.group_index << "SD_LINK config error, input_start: " << static_cast<int>(info.input_start) << std::endl;
        return false;
    }

    // 输出点检查
    int stanard_OutBytes = sdLink_spec.outBytes;
    int real_OutBytes = info.out_bytes;
    if (real_OutBytes)
    {
        if (stanard_OutBytes != real_OutBytes)
        {
            std::cout << info.group_index << "SD_LINK config error, outBytes: " << info.out_bytes << std::endl;
            return false;
        }

        if (info.output_start + real_OutBytes -1 > 127 || info.output_start < 0)
        {
            std::cout << info.group_index << "SD_LINK config error, output_start: " << static_cast<int>(info.output_start) << std::endl;
            return false;
        }
    }
    else if (info.output_start < 0 || info.output_start > 127)
    {
        std::cout << info.group_index << "SD_LINK config error, outBytes: " << info.out_bytes << std::endl;
        return false;
    }

    return true;
}

/**
 * @brief PMC2.0版本，配置手轮通道映射
 * @param indexId   : 手轮所在的扩展板号
 * @param channelId : 通道号 1,2,3,4
 * @return true -- 成功   false -- 失败
 */
bool ChannelEngine::SelectHandleWheel(int indexId, int channelId)
{
    if (channelId - 1 != 0)//暂时只支持单通道
        return false;

    ListNode<BdioDevInfo> *node = this->m_list_bdio_dev.HeadNode();
    bool findIndex = false;
    while(node != nullptr)
    {
        if (node->data.group_index == indexId)
        {
            if ( m_SDLINK_MAP.at(node->data.device_type).withHandWheel
                 && node->data.in_bytes != 0)
            {
                findIndex = true;
            }
            break;
        }
        node = node->next;
    }

    if (findIndex)
    {
        node = this->m_list_bdio_dev.HeadNode();
        while(node != nullptr)
        {
            if (node->data.group_index == indexId)
            {
                node->data.handwheel_map = channelId;
                node->data.in_bytes = m_SDLINK_MAP.at(node->data.device_type).inBytes + HANDWHEEL_BYTES;
            }
            else if (node->data.handwheel_map == channelId) // 一个通道暂时只支持一个手轮
            {
                node->data.handwheel_map = 0;
                node->data.in_bytes -= HANDWHEEL_BYTES;
            }
            node = node->next;
        }
        return true;
    }
    else
    {
        return false;
    }
}
#else
/**
 * @breif PMC1.0版本，读取SD_LINK从站设备配置
 */
void ChannelEngine::ReadIoDev_pmc1(){
    this->m_list_bdio_dev.Clear();

    //打开梯图文件
    int fd = open(PATH_PMC_LDR, O_RDONLY);
    if(fd == -1){
        printf("初始化SD-LINK设备失败，无法打开梯图文件\n");
        return;
    }

    //	//首先读取文件大小
    //	uint64_t file_size = lseek(fd, 0, SEEK_END);
    //
    //	lseek(fd, 0, SEEK_SET);  //跳回文件头

    int doc_ver;  //文档版本
    char uuid[17];  //16字节长度的UUID
    int machine_type;   //设备类型
    BdioDevInfo dev_info;    //从站设备
    int dev_count = 0;   //bdio设备数量
    int sub_index = 0, dev_type = 0, in_count = 0, out_count = 0;
    int i = 0;
    int tt = 0;

    //读取文档版本
    int count = read(fd, &doc_ver, sizeof(int));
    if(count != sizeof(int)){
        printf("读取文档版本失败[%d, %d]\n", count, sizeof(int));
        goto END;
    }
    doc_ver = BigLittleSwitch32(doc_ver);

    printf("read ldr file ver: %d\n", doc_ver);

    //读取UUID
    memset(uuid, 0x00, 17);
    count = read(fd, uuid, 16);
    if(count != 16){
        printf("读取梯图UUID失败[%d, 16]\n", count);
        goto END;
    }

    //读取设备类型
    count = read(fd, &machine_type, sizeof(int));
    if(count != sizeof(int)){
        printf("读取设备类型失败[%d, %d]\n", count, sizeof(int));
        goto END;
    }
    machine_type = BigLittleSwitch32(machine_type);
    printf("machine type = %d\n", machine_type);

    //读取BDIO设备
    count = read(fd, &dev_count, sizeof(int));
    if(count != sizeof(int)){
        printf("读取BDIO设备数失败[%d, %d]\n", count, sizeof(int));
        goto END;
    }
    dev_count = BigLittleSwitch32(dev_count);
    printf("slave count = %d\n", dev_count);

    //读取具体从站数据
    for(i = 0; i < dev_count; i++){
        //从站号
        count = read(fd, &sub_index, sizeof(int));
        if(count != sizeof(int)){
            printf("读取BDIO设备从站号失败[%d]\n", count);
            goto END;
        }
        sub_index = BigLittleSwitch32(sub_index);

        //从站类型
        count = read(fd, &dev_type, sizeof(int));
        if(count != sizeof(int)){
            printf("读取BDIO设备类型失败[%d]\n", count);
            goto END;
        }
        dev_type = BigLittleSwitch32(dev_type);

        //输入字节数
        count = read(fd, &in_count, sizeof(int));
        if(count != sizeof(int)){
            printf("读取BDIO设备输入字节失败[%d]\n", count);
            goto END;
        }
        in_count = BigLittleSwitch32(in_count);

        //输出字节数
        count = read(fd, &out_count, sizeof(int));
        if(count != sizeof(int)){
            printf("读取BDIO设备输出字节失败[%d]\n", count);
            goto END;
        }
        out_count = BigLittleSwitch32(out_count);

        //跳过输入地址，输出地址
        if(-1 == lseek(fd, 2*sizeof(int), SEEK_CUR)){
            printf("跳过输入输出地址失败\n");
            goto END;
        }

        //跳过设备名称
        count = read(fd, &tt, sizeof(int));
        if(count != sizeof(int)){
            printf("读取设备名称长度失败[%d]\n", count);
            goto END;
        }
        tt = BigLittleSwitch32(tt);
        if(-1 == lseek(fd, tt, SEEK_CUR)){
            printf("跳过设备名称失败\n");
            goto END;
        }

        //跳过地址字符串
        count = read(fd, &tt, sizeof(int));
        if(count != sizeof(int)){
            printf("读取地址字符串长度失败[%d]\n", count);
            goto END;
        }
        tt = BigLittleSwitch32(tt);
        if(-1 == lseek(fd, tt, SEEK_CUR)){
            printf("跳过地址字符串失败\n");
            goto END;
        }

        dev_info.slave_index = sub_index;
        dev_info.device_type = dev_type;
        dev_info.in_bytes = in_count;
        dev_info.out_bytes = out_count;
        printf("slave[%d, %d, %d, %d]\n", sub_index, dev_type, in_count, out_count);

        this->m_list_bdio_dev.Append(dev_info);
    }

END:
    close(fd);   //关闭文件
}
#endif

/**
 * @brief 初始化SD-LINK从站设备配置
 */
void ChannelEngine::InitBdioDev(){

#ifdef USES_PMC_2_0
    this->ReadIoDev_pmc2();
#else
    ReadIoDev_pmc1();
#endif

}

/**
 * @brief 检查轴上伺服信号
 */
void ChannelEngine::CheckAxisSrvOn(){
    for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
        bool svo_on = this->m_p_channel_control[i].CheckAxisSrvOn(m_n_phy_axis_svo_on);
        if(svo_on && !m_servo_ready){
            ScPrintf("servo ready");
            m_servo_ready = true;
        }
    }
}

/**
 * @brief 向MI发送各轴回参考点标志
 */
void ChannelEngine::SetAxisRetRefFlag(){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_RET_REF_FLAG;

    cmd.data.axis_index = NO_AXIS;

    memcpy(cmd.data.data, &this->m_n_mask_ret_ref_over, sizeof(uint64_t));

    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 获取指定的物理轴是否已经建立参考点
 * @param phy_axis, 物理轴号0
 * @return true 建立， flase 未建立
 */
bool ChannelEngine::GetAxisRetRefFlag(uint64_t phy_axis)
{
    bool flag = m_n_mask_ret_ref_over & (0x01 << phy_axis);
    return flag;
}

/**
 * @brief 向MI发送各轴机械锁住状态
 */
void ChannelEngine::SetMLKState(uint8_t MLK, uint8_t MLKI)
{
    static uint8_t last_MLK = 0x00;
    //static std::future<void> ans;
    ScPrintf("SetMLKState: MLK = %u MLKI=%u",MLK, MLKI);

    uint8_t mask = 0x00;
    for(int i = 0; i < m_p_channel_config->chn_axis_count; i++){
        // 旋转轴不用锁住
        if(m_p_axis_config[i].axis_type == AXIS_SPINDLE)
            continue;

        bool en_now = (MLKI & (0x01 << i)) || (MLK == 1);
        if(en_now)
            mask |= (0x01 << i);

        bool en_last = (last_MLK & (0x01 << i));
        if(en_now == en_last)
            continue;

        m_p_mi_comm->SendAxisMLK(i+1, en_now);
        if(en_now){
           m_MLK_mask |= (0x01 << i);
        }else{
            m_MLK_mask &= ~(0x01 << i);
        }
//        if(en_now){
//            this->m_p_mi_comm->ReadPhyAxisCurFedBckPos(m_df_phy_axis_pos_feedback,
//                                                       m_df_phy_axis_pos_intp,
//                                                       m_df_phy_axis_speed_feedback,
//                                                       m_df_phy_axis_torque_feedback,
//                                                       m_p_general_config->axis_count);
//           m_MLK_pos[i] = m_df_phy_axis_pos_feedback[i];
//           m_p_mi_comm->SendAxisMLK(i+1, en_now);
//           m_MLK_mask |= (0x01 << i);
//        }else{
//            clr_mask |= (0x01 << i);
//        }

    }

//    // 执行机械锁住恢复动作
//    if(clr_mask != 0x00){
//        auto func = std::bind(&ChannelEngine::ProcessRecoverMLK,
//                              this, std::placeholders::_1, std::placeholders::_2);
//        ans = std::async(std::launch::async, func, clr_mask,m_MLK_pos);
//    }
    last_MLK = mask;
}

void ChannelEngine::ProcessRecoverMLK(uint8_t mask, double *mach_pos)
{
    // 机械锁住恢复执行流程
    // 1:将机械锁住恢复标记 置为 1，通知PMC此时不要作轴移动操作
    // 2:发生轴移动指令，让轴回到之前进入锁住状态前的坐标
    // 3:等待所有轴移动到位，给MI发送取消锁住的命令
    // 4:机械锁住标记 清除
    m_p_pmc_reg->FReg().bits->CLMLK = 1;
    //ScPrintf("ProcessRecoverMLK mask = %u", mask);

    for(int i = 0; i < m_p_channel_config->chn_axis_count; i++){
        if(((mask >> i) & 0x01) == 0)
            continue;

        ManualMoveAbs(i, 3000, mach_pos[i]);

    }

    for(int i = 0; i < m_p_channel_config->chn_axis_count; i++){
        if(((mask >> i) & 0x01) == 0)
            continue;

        while(fabs(this->GetPhyAxisMachPosFeedback(i) - mach_pos[i]) >= 0.010){
            std::this_thread::sleep_for(std::chrono::microseconds(100 * 1000));
//            ScPrintf("RecoverMLK: axis = %u,fabs = %lf", i,
//                     fabs(this->GetPhyAxisMachPosFeedback(i) - mach_pos[i]));

        }
    }

    std::this_thread::sleep_for(std::chrono::microseconds(200 * 1000));

    for(int i = 0; i < m_p_channel_config->chn_axis_count; i++){
        if(((mask >> i) & 0x01) == 0)
            continue;

        m_p_mi_comm->SendAxisMLK(i+1, 0);
        m_MLK_mask &= ~(0x01 << i);
    }
    m_p_pmc_reg->FReg().bits->CLMLK = 0;

}

void ChannelEngine::SetSoftLimitSignal(uint8_t EXLM, uint8_t RLSOT){
    bool check1 = true;
    bool check2 = false;
    for(int i=0; i<m_p_channel_config->chn_axis_count; i++){
        check1 = (!EXLM && !RLSOT);
        check2 = (EXLM && !RLSOT);
        if(m_p_axis_config[i].axis_type == AXIS_SPINDLE){ // 主轴不需软限位
            m_p_axis_config[i].soft_limit_check_1 = 0;
            m_p_axis_config[i].soft_limit_check_2 = 0;
        }else{
            m_p_axis_config[i].soft_limit_check_1 = check1;
            m_p_axis_config[i].soft_limit_check_2 = check2;
        }
        m_p_channel_control->SetChnAxisSoftLimit(i);
    }
}

/**
 * @brief 向MI发送物理轴的反馈
 */
void ChannelEngine::SendMiPhyAxisEncoder(){
    FILE *fp = fopen(PATH_PHY_AXIS_ENCODER, "rb");//打开文件

    if(fp == nullptr){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "打开物理轴当前反馈数据失败！");
        return;//文件打开失败
    }

    int64_t data[m_p_general_config->axis_count];

    size_t count  = fread(data, 8, m_p_general_config->axis_count, fp);

    if(count != m_p_general_config->axis_count)
        printf("error in read phy axis encoder value!%d, %d\n", count, m_p_general_config->axis_count);

    fclose(fp);

    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_INIT_ENCODER;
    for(uint8_t i = 0; i < m_p_general_config->axis_count; i++){
        cmd.data.axis_index = i+1;

        memcpy(cmd.data.data, &data[i], 8);

        this->m_p_mi_comm->WriteCmd(cmd);
    }
}

/**
 * @brief 设置MI模块工作模式
 * @param value : 工作模式表示， 0x00--自动模式    0x10--进入手轮模式     0x20--手动模式
 */
void ChannelEngine::SetMiWorkMode(uint8_t value){
    //	printf("SetMiWorkMode: %hhu\n", value);
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_WORK_MODE;

    cmd.data.data[0] = value;

    uint8_t chn_count = this->m_p_channel_mode_group[this->m_n_cur_chn_group_index].GetChannelCount();
    for(uint i = 0; i < chn_count; i++){
        cmd.data.axis_index = NO_AXIS;
        cmd.data.reserved = this->m_p_channel_mode_group[this->m_n_cur_chn_group_index].GetChannel(i);  //通道号，从0开始

        if(cmd.data.reserved == 0xFF)
            continue;

        this->m_p_mi_comm->WriteCmd(cmd);
    }

}

/**
 * @brief 设置MI模块手轮跟踪模式
 * @param flag : true -- 打开手轮跟踪  false--关闭手轮跟踪
 * @param chn : 通道号，从0开始
 */
void ChannelEngine::UpdateHandwheelState(uint8_t chn){
    bool trace_enable = m_p_channel_control[chn].CheckFuncState(FS_HANDWHEEL_CONTOUR);
    bool trace_reverse = this->m_p_general_config->hw_rev_trace;
    bool insert_enable;
    const GRegBits *g_reg = &m_p_pmc_reg->GReg().bits[chn];
    uint8_t axis_no = 0;
    for(int axis = 0; axis < m_p_channel_config[chn].chn_axis_count; axis++){
        if((g_reg->HSIA >> axis) & 0x01){
            axis_no = axis + 1;
            break;
        }
    }
    if(axis_no == 0){
        insert_enable = false;
    }else{
        insert_enable = true;
    }
    if(trace_enable){
        insert_enable = false;
    }

    m_p_mi_comm->SendHandwheelState(chn,trace_enable,trace_reverse,insert_enable);

//    MiCmdFrame cmd;
//    memset(&cmd, 0x00, sizeof(cmd));
//    cmd.data.cmd = CMD_MI_SET_HANDWHEEL_TRACE;

//    cmd.data.data[0] = flag?0x10:0x00;
//    cmd.data.data[1] = this->m_p_general_config->hw_rev_trace;   //手轮反向引导使能
//    cmd.data.data[2] = 0;

//    cmd.data.axis_index = NO_AXIS;
//    cmd.data.reserved = chn;   //通道，从0开始


//    this->m_p_mi_comm->WriteCmd(cmd);

    //	uint8_t chn_count = this->m_p_channel_mode_group[this->m_n_cur_chn_group_index].GetChannelCount();
    //	for(uint i = 0; i < chn_count; i++){
    //		cmd.data.axis_index = NO_AXIS;
    //		cmd.data.reserved = this->m_p_channel_mode_group[this->m_n_cur_chn_group_index].GetChannel(i);    //通道，从0开始
    //
    //		if(cmd.data.reserved == 0xFF)
    //			continue;
    //
    //		this->m_p_mi_comm->WriteCmd(cmd);
    ////		printf("SetMiHandwheelTrace 2 : %hhu\n", cmd.data.reserved);
    //	}

}

void ChannelEngine::SetMiHandwheelInsert(bool flag, uint8_t chn){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "SetMiHandwheelTrace: %hhu， curgroup=%hhu\n", flag, m_n_cur_chn_group_index);
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_HANDWHEEL_TRACE;

    cmd.data.data[0] = 0x00;
    cmd.data.data[1] = this->m_p_general_config->hw_rev_trace;
    cmd.data.data[2] = flag?0x10:0x00;

    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = chn;   //通道，从0开始


    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 设置MI当前通道号
 */
void ChannelEngine::SetMiCurChannel(){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_CUR_CHN;
    cmd.data.axis_index = NO_AXIS;


    cmd.data.data[0] = this->m_n_cur_channle_index;

    this->m_p_mi_comm->WriteCmd(cmd);

}


/**
 * @brief 初始化设置外部接口以及创建通道控制对象
 * @param hmi_comm ：与HMI的通讯接口类指针
 * @param parm ：参数访问接口
 */
void ChannelEngine::Initialize(HMICommunication *hmi_comm, MICommunication *mi_comm,
                               MCCommunication *mc_comm, ParmManager *parm){
    printf("start to initialize channel engine\n");
    if(hmi_comm == nullptr ||
            mi_comm == nullptr ||
            mc_comm == nullptr ||
            parm == nullptr){
        m_error_code = ERR_SC_INIT;  //初始化失败，告警
        return;
    }

    this->m_n_update_state = MODULE_UPDATE_NONE;  //默认非升级状态

    this->m_p_hmi_comm = hmi_comm;
    this->m_p_mi_comm = mi_comm;
    this->m_p_mc_comm = mc_comm;
    this->m_p_general_config = parm->GetSystemConfig();
    this->m_p_channel_config = parm->GetChannelConfig();
    this->m_p_axis_config = parm->GetAxisConfig();
    this->m_p_pc_table = parm->GetPitchCompData();
    this->m_p_io_remap = parm->GetIoRemapList();
    this->m_p_chn_proc_param = parm->GetChnProcParam();
    this->m_p_axis_proc_param = parm->GetAxisProcParam();


    //创建PMC寄存器类对象
    this->m_p_pmc_reg = new PmcRegister();
    if(m_p_pmc_reg == nullptr){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "通道引擎创建PMC寄存器对象失败!");
        m_error_code = ERR_MEMORY_NEW;  //初始化失败
        return;
    }
    memset(m_g_reg_last.all, 0x00, sizeof(m_g_reg_last.all));
    memset(m_f_reg_last.all, 0x00, sizeof(m_f_reg_last.all));

    this->m_n_pmc_axis_count = parm->GetPmcAxisCount();
    this->m_sync_axis_ctrl = new SyncAxisCtrl;
    this->m_axis_status_ctrl = &Singleton<AxisStatusCtrl>::instance();
    this->m_axis_status_ctrl->Init(mi_comm, m_p_channel_config,
                                   m_p_axis_config,&m_p_pmc_reg->FReg().bits[0]);

#ifdef USES_FIVE_AXIS_FUNC
    this->m_p_chn_5axis_config = parm->GetFiveAxisConfig(0);
#endif

    m_n_phy_axis_svo_on = 0;

    this->InitPhyAxisChn();

    this->InitPcAllocList();

    this->m_n_da_prec = pow(2, this->m_p_general_config->da_prec-1);

    this->m_df_phy_axis_pos_feedback = new double[m_p_general_config->axis_count];  //分配物理轴当前反馈机械坐标存储区
    memset(m_df_phy_axis_pos_feedback, 0, sizeof(double)*m_p_general_config->axis_count);

    this->m_df_phy_axis_pos_intp = new double[m_p_general_config->axis_count];  //分配物理轴当前插补机械坐标存储区
    memset(m_df_phy_axis_pos_intp, 0, sizeof(double)*m_p_general_config->axis_count);

    this->m_df_phy_axis_pos_intp_after = new double[m_p_general_config->axis_count];  //分配物理轴当前插补后输出的机械坐标存储区
    memset(m_df_phy_axis_pos_intp_after, 0, sizeof(double)*m_p_general_config->axis_count);

    this->m_df_pmc_axis_remain = new double[m_p_general_config->axis_count];  //分配PMC轴当前余移动量存储区
    memset(m_df_pmc_axis_remain, 0, sizeof(double)*m_p_general_config->axis_count);

    this->m_df_phy_axis_speed_feedback = new double[m_p_general_config->axis_count];  //分配物理轴当前反馈机械坐标存储区
    memset(m_df_phy_axis_speed_feedback, 0, sizeof(double)*m_p_general_config->axis_count);

    this->m_df_phy_axis_torque_feedback = new double[m_p_general_config->axis_count];  //分配物理轴当前反馈机械坐标存储区
    memset(m_df_phy_axis_torque_feedback, 0, sizeof(double)*m_p_general_config->axis_count);

    this->m_df_spd_angle = new double[m_p_general_config->axis_count];  //分配物理轴当前反馈机械坐标存储区
    memset(m_df_spd_angle, 0, sizeof(double)*m_p_general_config->axis_count);

    //创建通道对象
    this->m_p_channel_control = new ChannelControl[m_p_general_config->chn_count];
    if(m_p_channel_control == nullptr){
        //内存分配失败，告警
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "通道引擎创建通道控制对象失败!");
        m_error_code = ERR_MEMORY_NEW;
        return;
    }

    //	printf("channel count :%d\n", m_p_general_config->chn_count);
    for(uint8_t i = 0; i < m_p_general_config->chn_count; i++){
        if(true == m_mc_run_on_arm[i])
            m_mc_run_mi_flag = true;
        else
            m_mc_run_dsp_flag = true;

        m_p_channel_control[i].SetMcArmComm(this->m_p_mc_arm_comm);
        m_p_channel_control[i].Initialize(i, this, hmi_comm, mi_comm, mc_comm, parm, m_p_pmc_reg);
    }

    printf("init pmc axis ctrl \n");
    //初始设置PMC通道控制对象
    for(uint8_t i = 0; i < kMaxPmcAxisCtrlGroup; i++)
        this->m_pmc_axis_ctrl[i].SetGroupIndex(i);
    printf("succeed to init pmc axis ctrl\n");

    //初始化方式组配置
    this->InitChnModeGroup();

    //初始化当前通道为第一通道
    this->SetCurWorkChanl(0);

    //初始化回参考点标志
    this->m_n_mask_ret_ref_over = 0;
    for(int i = 0; i < this->m_p_general_config->axis_count; i++){
        if(m_p_axis_config[i].axis_interface == VIRTUAL_AXIS    //虚拟轴不用建立参考点
            || m_p_axis_config[i].axis_type == AXIS_SPINDLE     //主轴不用建立参数点
            || (m_p_axis_config[i].feedback_mode != INCREMENTAL_ENCODER && m_p_axis_config[i].feedback_mode != NO_ENCODER && m_p_axis_config[i].ref_encoder != kAxisRefNoDef))    //已经建立参考点的绝对式编码器，不用再次建立参考点
        {
            //			m_n_mask_ret_ref_over |= (0x01<<i);
            this->SetRetRefFlag(i, true);
        }
    }
    printf("Initialize PhyAxis Ref Flag:0x%llx \n", this->m_n_mask_ret_ref_over);

    pthread_attr_t attr;
    struct sched_param param;
    int res = 0;

    //初始化授权相关
    if(!ReadDevSn(m_device_sn)){  //读取设备序列号失败
        printf("Failed to read device sn\n");
    }else{
        printf("device SN: %s\n",m_device_sn);
    }
#ifdef USES_LICENSE_FUNC
    m_ln_local_time = ReadLocalTime(m_device_sn);//读取本地计时文件
    if(m_ln_local_time < 0){//读取本地计时文件异常
        if(m_ln_local_time == -1){
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "本地计时文件不存在!");
        }else if(m_ln_local_time == -2){
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "本地计时文件已损坏!");
        }else if(m_ln_local_time == -3){
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "非法本地计时文件!");
        }
        m_error_code = ERR_SYSTEM_FILE;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
    }
    if(!ReadLicense(this->m_device_sn, &this->m_lic_info)){    //读取授权信息
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "读取授权文件失败!");
    }

    if(-4 == CheckLocalTime(&m_lic_info, m_ln_local_time)){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "系统时间异常!");
        m_error_code = ERR_SYSTEM_TIME;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
    }

    if(1 == this->CheckLicense(true)){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "系统授权非法!");
    }

    printf("license info: flag = %c, deadline=%04d-%02d-%02d\n", this->m_lic_info.licflag, this->m_lic_info.dead_line.year,
           this->m_lic_info.dead_line.month, this->m_lic_info.dead_line.day);
#endif


    //创建状态刷新刷新线程
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//
    param.__sched_priority = 36; //96;
    pthread_attr_setschedparam(&attr, &param);
    res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
    if (res) {
        printf("pthread setinheritsched failed\n");
    }

    res = pthread_create(&m_thread_refresh_mi_status, &attr,
                         ChannelEngine::RefreshMiStatusThread, this);    //开启通道状态刷新线程
    if (res != 0) {
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "通道引擎创建MI状态刷新线程失败！");
        m_error_code = ERR_SC_INIT;
        return;
    }

    this->InitPoweroffHandler();
    printf("succeed to initialize channel engine\n");

}

/**
 * @brief 初始化方式组数据，必须在读取通道配置之后调用
 */
void ChannelEngine::InitChnModeGroup(){
    if(this->m_p_channel_config == nullptr)
        return;

    this->m_p_channel_mode_group = new ChannelModeGroup[this->m_p_general_config->chn_count];    //方式组最大数量等于通道数量

    if(this->m_p_channel_mode_group == nullptr){
        CreateError(ERR_SC_INIT, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
        return;
    }

    for(uint8_t i = 0; i < m_p_general_config->chn_count; i++){
        this->m_p_channel_mode_group[i].SetChnGroupIndex(i);

        if(m_p_channel_config[i].chn_group_index < m_p_general_config->chn_count){
            this->m_p_channel_mode_group[m_p_channel_config[i].chn_group_index].AddChannel(i);
        }
    }

}

/**
 * @brief 初始化螺补数据分布表
 */
void ChannelEngine::InitPcAllocList(){
    this->m_list_pc_alloc.Clear();

    AxisPcDataAlloc pc_alloc;
    for(int i = 0; i < m_p_general_config->axis_count; i++){
        uint16_t flag = this->m_p_axis_config[i].pc_type; // 0 单向螺补  1 双向螺补
        if(this->m_p_axis_config[i].pc_count == 0)
            continue;
        pc_alloc.axis_index = i;

        if(flag == 1){
            pc_alloc.pc_count = m_p_axis_config[i].pc_count*2;  // 双向螺补
        }else{
            pc_alloc.pc_count = m_p_axis_config[i].pc_count;  // 单向螺补
        }

        pc_alloc.start_index = m_p_axis_config[i].pc_offset>0?m_p_axis_config[i].pc_offset-1:0;
        pc_alloc.end_index = pc_alloc.start_index+pc_alloc.pc_count-1;


        printf("axis %d -- start_index %d  -- end_index %d\n", i, pc_alloc.start_index, pc_alloc.end_index);

        //按起始偏移从小到大的顺序插入list
        ListNode<AxisPcDataAlloc> *node = this->m_list_pc_alloc.HeadNode();

        while(node != nullptr){
            if(node->data.start_index > pc_alloc.start_index)
                break;
            node = node->next;
        }
        if(node == nullptr){
            this->m_list_pc_alloc.Append(pc_alloc);
        }else
            this->m_list_pc_alloc.InsertBefore(pc_alloc, node);

    }
}

/**
 * @brief 改变指定通道的所属方式组
 * @param chn : 通道号，从0开始
 * @param group_old ：原所属方式组号，从0开始
 * @param group_new ：新所属方式组号，从0开始
 */
void ChannelEngine::ChangeChnGroupIndex(uint8_t chn, uint8_t group_old, uint8_t group_new){
    if(chn >= m_p_general_config->chn_count ||
            group_old >= m_p_general_config->chn_count ||
            group_new >= m_p_general_config->chn_count)
        return;

    if(group_old == group_new)
        return;

    this->m_p_channel_mode_group[group_old].RemoveChannel(chn);    //从原方式组中移除通道chn
    this->m_p_channel_mode_group[group_new].AddChannel(chn);       //
}

/**
 * @brief 获取指定通道的F寄存器指针
 * @param chn_index
 * @return
 */
FRegBits *ChannelEngine::GetChnFRegBits(uint8_t chn_index){
    if(chn_index >= kMaxChnCount)
        return nullptr;
    return &m_p_pmc_reg->FReg().bits[chn_index];
}

/**
 * @brief 获取指定通道的G寄存器指针
 * @param chn_index
 * @return
 */
const GRegBits *ChannelEngine::GetChnGRegBits(uint8_t chn_index){
    if(chn_index >= kMaxChnCount)
        return nullptr;
    return &m_p_pmc_reg->GReg().bits[chn_index];
}

PmcAxisCtrl *ChannelEngine::GetChnPmcAxisCtrl(uint8_t chn_index){
    if(chn_index > kMaxChnCount)
        return nullptr;
    return &m_pmc_axis_ctrl[chn_index*4];
}


//void ChannelEnginePoweroffHandler(int signo, siginfo_t *info, void *context){
//	//
//
//	printf("########system power off\n");
//}

/**
 * @brief 空闲处理函数
 */
void ChannelEngine::DoIdle(){
    uint8_t i = 0;
    this->m_n_idle_count++;
    //
    //	//空闲时保存PMC寄存器数据
    //	if(m_n_idle_count%100 == 0){
    //		bool idle = true;
    //		for(i = 0; i < this->m_p_general_config->chn_count; i++){
    //			if(this->m_p_channel_control[i].IsMachinRunning()){
    //				idle = false;
    //				break;
    //			}
    //		}
    //		if(idle && this->m_p_pmc_reg->IsKeepRegChanged()){//保存寄存器数据
    //			this->m_p_pmc_reg->SaveRegData();
    //		}
    //	}
    //
    //
    //调用通道的空闲函数
    for(i = 0; i < this->m_p_general_config->chn_count; i++){
        this->m_p_channel_control[i].DoIdle(m_n_idle_count);
    }
}

/**
 * @brief 初始关联掉电信号和掉电处理函数
 */
void ChannelEngine::InitPoweroffHandler(){
    struct sigaction sa;

    sa.sa_handler = (void (*)(int))PoweroffHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO;
    int ret = sigaction(SIGUSR1, &sa, NULL);
    if (ret == -1) {
        printf("#####Failed to initialize poweroff signal handler!\n");
    }

    int fd = open("/dev/aradex_gpio", O_RDWR);
    if (fd < 0) {
        printf("Can't open /dev/aradex_gpio!\n");
        return;
    }

    ioctl(fd, SET_NOTIFY_PID, getpid()); //设置通知pid为自己
    ioctl(fd, SET_NOTIFY_INTERVAL, 0);	//设置间隔为0

    close(fd);

    printf("########succeed to init power off \n");
}

/**
 * @brief 掉电处理函数
 * @param signo
 * @param info
 * @param context
 */
void ChannelEngine::PoweroffHandler(int signo, siginfo_t *info, void *context){
    printf("IN\n");

    //	static bool power_off_flag = false;
    //	if(power_off_flag)
    //		return;
    //	power_off_flag = true;

    ChannelEngine *p_chn_engine = ChannelEngine::GetInstance();
    if(p_chn_engine == nullptr)
        return;
    p_chn_engine->SetPoweoffFlag(true);

    p_chn_engine->SaveDataPoweroff();

    //	power_off_flag = false;

    printf("OUT\n");
}

/**
 * @brief 掉电时保存数据
 */
void ChannelEngine::SaveDataPoweroff(){

    //保存PMC寄存器数据
    if((this->m_mask_import_param & (0x01<<CONFIG_PMC_REG)) == 0)
        this->m_p_pmc_reg->SaveRegData();

    //保存非易失性宏变量
    if((this->m_mask_import_param & (0x01<<CONFIG_MACRO_VAR)) == 0)
        this->SaveKeepMacroVar();


    //保存各通道当前需保存的状态
    //	g_ptr_parm_manager->SaveParm(CHN_STATE_SCENE);    //注释原因：此为低频操作，改为在通道状态修改时即时保存，不再掉电时增加耗时

    //保存各轴当前位置
    this->SaveCurPhyAxisEncoder();

    //保存刀具寿命信息
#ifdef USES_WOOD_MACHINE
    this->SaveToolInfo();
#endif

    sync();

    delete g_ptr_trace;
    g_ptr_trace = nullptr;
}

/**
 * @brief 掉电保存非易失性宏变量
 */
void ChannelEngine::SaveKeepMacroVar(){

    for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
        this->m_p_channel_control[i].SaveKeepMacroVar();
    }

}

/**
 * @brief 同步保存文件
 */
void ChannelEngine::SyncKeepVar(){
    //保存PMC寄存器数据
    this->m_p_pmc_reg->SaveRegData();

    for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
        this->m_p_channel_control[i].SyncMacroVar();
    }


}

/**
 * @brief 设置参数导入标志
 * @param param_type : 参数类型
 */
void ChannelEngine::SetParamImportMask(int param_type){
    if(param_type >= CONFIG_TYPE_COUNT)
        return;

    this->m_mask_import_param |= (0x01<<param_type);
}


/**
 * @brief 掉电保存当前所有物理轴的编码器反馈
 */
void ChannelEngine::SaveCurPhyAxisEncoder(){

    FILE *fp = fopen(PATH_PHY_AXIS_ENCODER, "wb");//打开文件

    if(fp == nullptr){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "打开物理轴当前反馈数据失败！");
        return;//文件打开失败
    }

    int64_t data[m_p_general_config->axis_count];
    this->m_p_mi_comm->ReadPhyAxisEncoder(data, m_p_general_config->axis_count);

    size_t count  = fwrite(data, 8, m_p_general_config->axis_count, fp);

    if(count != m_p_general_config->axis_count)
        printf("error in save phy axis encoder value!\n");

    fsync(fileno(fp));
    fclose(fp);

}


/**
 * @brief 向MC模块发送握手命令
 */
void ChannelEngine::ShakeHandWithMc(){
    McCmdFrame cmd;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SHAKEHANDS;
    cmd.data.data[0] = 0xADAD;
    cmd.data.data[1] = 0x5678;

    if(true == this->m_mc_run_dsp_flag)
        m_p_mc_comm->WriteCmd(cmd);
    if(true == this->m_mc_run_mi_flag)
        m_p_mc_arm_comm->WriteCmd(cmd);
}

/**
 * @brief 向MI模块发送握手命令
 */
void ChannelEngine::ShakeHandWithMi(){
    MiCmdFrame cmd;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = 0;

    cmd.data.cmd = CMD_MI_SHAKEHAND;
    cmd.data.data[0] = 0xA0A1;
    cmd.data.data[1] = 0x1357;

    m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 向MC发送插补周期参数
 */
void ChannelEngine::SendIntepolateCycle(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));
    cmd.data.axis_index = NO_AXIS;
    cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SET_INIT_PARAM;
    //	cmd.data.data[0] = 0x0001;   	//250us周期
    cmd.data.data[0] = this->m_p_general_config->bus_cycle; // this->m_p_channel_config[0].intep_cycle;   //插补周期
    //	cmd.data.data[1] = 0x0002;		//-T模式


    if(true == this->m_mc_run_dsp_flag)
        m_p_mc_comm->WriteCmd(cmd);

    if(true == this->m_mc_run_mi_flag)
        m_p_mc_arm_comm->WriteCmd(cmd);

    printf("send intep cycle to mc : %hu\n", cmd.data.data[0]);
}

/**
 * @brief 发送MC复位指令
 * @return
 */
void ChannelEngine::SendMcResetCmd(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));
    cmd.data.axis_index = NO_AXIS;
    cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SYS_RESET;

    if(true == this->m_mc_run_dsp_flag)
        m_p_mc_comm->WriteCmd(cmd);

    if(true == this->m_mc_run_mi_flag)
        m_p_mc_arm_comm->WriteCmd(cmd);   // ??????????????????????????????????????????????  二选一  是否加一个宏
}

/**
 * @brief 发送MC清除告警指令
 * @return
 */
void ChannelEngine::ClearMcAlarm(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));
    cmd.data.axis_index = NO_AXIS;
    cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_RESET_ALARM;

    if(true == this->m_mc_run_dsp_flag)
        m_p_mc_comm->WriteCmd(cmd);

    if(true == this->m_mc_run_mi_flag)
        m_p_mc_arm_comm->WriteCmd(cmd);  // ??????????????????????????????????????????????  二选一  是否加一个宏
}

/**
 * @brief 初始化MC的数据缓冲区，设置默认非单段模式
 */
void ChannelEngine::InitMcDataBuffer(){
    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        this->m_p_channel_control[i].InitMcIntpAutoBuf();
        this->m_p_channel_control[i].InitMcIntpMdaBuf();
        this->m_p_channel_control[i].SetMcStepMode(false);
    }
}

/**
 * @brief 初始化设置MC工件坐标系
 */
void ChannelEngine::InitMcCoord(){
    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        this->m_p_channel_control[i].SetMcCoord(true);
    }
}

/**
 * @brief 设置MC的相关参数
 */
void ChannelEngine::InitMcParam(){
    SendIntepolateCycle();  //发送插补周期

    for(int i = 0; i < this->m_p_general_config->chn_count; i++){ //初始化通道参数
        this->m_p_channel_control[i].InitMcIntpAutoBuf();
        this->m_p_channel_control[i].InitMcIntpMdaBuf();
        this->m_p_channel_control[i].SetMcStepMode(false);
        this->m_p_channel_control[i].SetMcChnPlanMode();
        this->m_p_channel_control[i].SetMcChnPlanParam();
        this->m_p_channel_control[i].SetMcChnPlanFun();
        this->m_p_channel_control[i].SetMcChnCornerStopParam();
        this->m_p_channel_control[i].SetChnAxisName();

        this->m_p_channel_control[i].SetMcCoord(true);  //初始化工件坐标系偏移

        this->m_p_channel_control[i].SetChnAllAxisParam(); //初始化轴参数
#ifdef USES_FIVE_AXIS_FUNC
        this->m_p_channel_control[i].SetMcChnFiveAxisParam();  //初始化五轴参数
#endif
#ifdef USES_WOOD_MACHINE
        this->m_p_channel_control[i].SetMcFlipCompParam();  //发送挑角补偿
        this->m_p_channel_control[i].SetMcDebugParam(0);
        this->m_p_channel_control[i].SetMcDebugParam(1);
        this->m_p_channel_control[i].SetMcDebugParam(2);
        this->m_p_channel_control[i].SetMcDebugParam(3);
        this->m_p_channel_control[i].SetMcDebugParam(4);
#endif
    }

}

/**
 * @brief 获取MC模块的版本信息
 */
void ChannelEngine::SendGetMcVersionCmd(){
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));
    cmd.data.axis_index = NO_AXIS;
    cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_GET_VERSION;


    if(true == this->m_mc_run_dsp_flag)
        m_p_mc_comm->WriteCmd(cmd);
    if(true == this->m_mc_run_mi_flag)
        m_p_mc_arm_comm->WriteCmd(cmd);  // ??????????????????????????????????????????????  二选一  是否加一个宏
}

#ifdef USES_TIANJIN_PROJ
/**
 * @brief PMC轴机械坐标系到工件坐标系的转换
 * @param pos ：机械坐标
 * @param pmc_axis_index ： PMC轴序号
 * @return 工件坐标
 */
double ChannelEngine::TransMachToWorkCoord(double &pos, uint8_t pmc_axis_index){
    double work_pos = 0;



    return work_pos;
}

/**
 * @brief PMC轴工件坐标系到机械坐标系的转换
 * @param pos ： 工件坐标
 * @param pmc_axis_index ： PMC轴序号
 * @return ： 机械坐标
 */
double ChannelEngine::TransWorkToMachCoord(double &pos, uint8_t pmc_axis_index){
    double mach_pos = 0;


    return mach_pos;
}
#endif

/**
 * @brief 发送PMC轴位置给HMI
 */
void ChannelEngine::SendPmcAxisToHmi(){
    if(this->m_n_pmc_axis_count == 0)
        return;

    uint8_t count = 0;
    uint8_t data_type = MDT_CHN_PMC_AXIS_POS;
    uint8_t chn_index = 0xFF;
    uint16_t data_len = sizeof(PmcAxisPos)*m_n_pmc_axis_count;
    uint16_t buf_len = 4+data_len;
    char buffer[buf_len+1];
    memset(buffer, 0x00, buf_len+1);
    memcpy(buffer, &data_type, 1);
    memcpy(buffer+1, &chn_index, 1);
    memcpy(buffer+2, &data_len, 2);

    PmcAxisPos axis;
    int size = sizeof(PmcAxisPos);
    for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
        if (!GetPmcActive(i))
            continue;
        axis.axis_no = i;  //从0开始编号
        axis.mach_pos = this->m_df_phy_axis_pos_feedback[i];
        axis.remain_dis = this->m_df_pmc_axis_remain[i];

#ifdef USES_TIANJIN_PROJ
        //机械坐标转换为工件坐标
        axis.work_pos = this->TransMachToWorkCoord(axis.mach_pos, count);
#endif

        memcpy(buffer+4+size*count, &axis, size);
        if(++count >= this->m_n_pmc_axis_count)
            break;
    }


    this->m_p_hmi_comm->SendMonitorData(buffer, buf_len);//send(socket, buffer, buf_len, MSG_NOSIGNAL);
}


/**
 * @brief 发送监控数据
 * @param bAxis : 是否发送坐标轴实时位置
 * @param btime : 是否更新加工时间
 */
void ChannelEngine::SendMonitorData(bool bAxis, bool btime){
    //从MI读取所有轴的位置
    this->m_p_mi_comm->ReadPhyAxisCurFedBckPos(m_df_phy_axis_pos_feedback, m_df_phy_axis_pos_intp,m_df_phy_axis_speed_feedback,
                                               m_df_phy_axis_torque_feedback,m_df_spd_angle, m_p_general_config->axis_count);

    //#ifdef USES_SPEED_TORQUE_CTRL
    //	this->m_p_mi_comm->ReadPhyAxisCurFedBckSpeedTorque(m_df_phy_axis_speed_feedback, m_df_phy_axis_torque_feedback, m_p_general_config->axis_count);
    //#endif

    if(m_n_pmc_axis_count > 0)
    {
        this->m_p_mi_comm->ReadPmcAxisRemainDis(m_df_pmc_axis_remain, m_p_general_config->axis_count);   // 读取余移动量
        //std::cout << "m_df_pmc_axis_remain: " << m_df_pmc_axis_remain[3] << std::endl;
    }

    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        this->m_p_channel_control[i].SendMonitorData(bAxis, btime);
    }

    this->SendPmcAxisToHmi();   //发送PMC轴数据
    //	printf("send monitor data\n");
}

/**
 * @brief 处理MC模块的指令回复
 * @param rsp
 */
void ChannelEngine::ProcessMcCmdRsp(McCmdFrame &rsp){
    //TODO
    switch(rsp.data.cmd){
    case CMD_MC_SHAKEHANDS:
        if(rsp.data.data[0] == 0x1234 && rsp.data.data[1] == 0x5678){
            //握手成功
            g_sys_state.module_ready_mask |= MC_READY;  //MC模块就绪


            this->SetMcAutoBufMax(rsp.data.data[2]);   //设置MC自动运动数据缓冲大小

            //			SendIntepolateCycle();
            //
            //			//初始化自动与MDA的数据缓冲
            //			InitMcDataBuffer();
            //
            //			//初始化工件坐标系
            //			InitMcCoord();
            //
            //			//初始化软限位
            //			for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++)
            //				this->SetAxisSoftLimit(i, 0);
            InitMcParam();

            this->SendGetMcVersionCmd();//发送获取版本指令
            if(m_p_mc_comm->ReadPlVer()){
                g_sys_state.module_ready_mask |= PL_READY;  //PL模块就绪
            }
            if(m_p_mc_comm->ReadSp6Ver()){
                g_sys_state.module_ready_mask |= SPANTAN_READY;  //SP6模块就绪
            }

            if((g_sys_state.module_ready_mask & NC_READY) == NC_READY)
                g_sys_state.system_ready = true;

            g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "@#@#@Get MC shake hand cmd, module_mask=0x%hhu\n", g_sys_state.module_ready_mask);


        }
        else{//握手失败
            g_sys_state.module_ready_mask &= (~MC_READY);  //MC模块启动失败
            m_error_code = ERR_MC_INIT;
            CreateError(ERR_MC_INIT, FATAL_LEVEL, CLEAR_BY_RESET_POWER);

        }
        break;
    case CMD_MC_GET_UPDATE_STATE:
        //		this->m_n_mc_update_status = rsp.data_crc[1]&0x00FF;	//老版本返回值放在axis_index上
        this->m_n_mc_update_status = rsp.data.data[0]&0x00FF;
        this->m_b_get_mc_update_status = true;
        printf("get CMD_MC_GET_UPDATE_STATE rsp, %hu\n", m_n_mc_update_status);
        break;
    case CMD_MC_GET_VERSION:
        this->ProcessMcVersionCmd(rsp);
        break;
    default:
        break;
    }
}


/**
 * @brief 处理MC模块返回的版本信息
 */
void ChannelEngine::ProcessMcVersionCmd(McCmdFrame &cmd){
    //处理MC模块返回的版本信息
    uint16_t data1 = cmd.data.data[0], data2 = cmd.data.data[1], data3 = cmd.data.data[2], data4 = cmd.data.data[3];
    uint16_t data5 = cmd.data.data[4];
    uint8_t v_a = (data2>>8)&0xFF;
    uint8_t v_b = (data2)&0xFF;
    uint8_t v_c = (data1>>8)&0xFF;
    uint8_t v_d = (data1)&0xFF;
    uint8_t v_h = (data5)/100;
    uint8_t v_m = (data5)%100;

    sprintf(g_sys_info.sw_version_info.mc, "%c%hhu.%hhu.%hhu.%hu%04hu%02hu%02hu", v_a==0?'P':'V', v_b, v_c, v_d, data3, data4,
            v_h,v_m);
}

/**
 * @brief 处理MI模块返回的版本信息
 * @param cmd
 */
void ChannelEngine::ProcessMiVersionCmd(MiCmdFrame &cmd){

    uint8_t v_a = (cmd.data.data[2]>>8)&0xFF;
    uint8_t v_b = (cmd.data.data[2])&0xFF;
    uint8_t v_c = (cmd.data.data[1]>>8)&0xFF;
    uint8_t v_d = (cmd.data.data[1])&0xFF;

    sprintf(g_sys_info.sw_version_info.mi, "%c%hhu.%hhu.%hhu", v_a==0?'P':'V', v_b, v_c, v_d);

    printf("get MI Version: %s\n", g_sys_info.sw_version_info.mi);
}

/**
 * @brief 处理MI模块的指令
 * @param cmd
 */
void ChannelEngine::ProcessMiCmd(MiCmdFrame &cmd){
    uint16_t cmd_no = cmd.data.cmd;

    //	bool rsp = (cmd_no & 0x8000) == 0 ? false:true;  //是否响应包

    cmd_no &= 0x7FFF;

    switch(cmd_no){
    case CMD_MI_SHAKEHAND:
        this->ProcessMiShakehand(cmd);

        break;
        //	case CMD_MI_GET_VERSION:
        //		this->ProcessMiVersionCmd(cmd);
        //		break;
    case CMD_MI_PMC_CODE_REQ:
        this->ProcessMiPmcLadderReq(cmd);
        break;
    case CMD_MI_GET_ESB:
        this->ProcessMiGetESBCmd(cmd);
        break;
    case CMD_MI_READY:	//MI准备好
        ScPrintf("get CMD_MI_READY, module_mask=0x%hhu\n", g_sys_state.module_ready_mask);
        g_sys_state.module_ready_mask |= MI_READY;
        if((g_sys_state.module_ready_mask & NC_READY) == NC_READY){
            g_sys_state.system_ready = true;
            m_axis_status_ctrl->RspMiReady();
        }

        this->SetWorkMode(this->m_p_channel_control[0].GetChnWorkMode());
        this->SetMiCurChannel();

        break;
    case CMD_MI_ALARM:	//MI告警
        this->ProcessMiAlarm(cmd);
        break;
    case CMD_MI_BUS_ALARM:		//总线告警
        ProcessMiBusError(cmd);
        break;
    case CMD_MI_HEARTBEAT:	//MI心跳
        cmd.data.cmd |= 0x8000;
        this->m_p_mi_comm->WriteCmd(cmd);
        if(!m_b_recv_mi_heartbeat){
            m_b_recv_mi_heartbeat = true;
            this->InitMiParam();
        }
        break;
    case CMD_MI_OPERATE:
        this->ProcessMiOperateCmdRsp(cmd); // 轴操作指令回复
        break;
    case CMD_MI_SET_REF_CUR:	//返回了编码器值
        this->ProcessMiSetRefCurRsp(cmd);
        break;
    case CMD_MI_CLEAR_ROT_AXIS_POS:		//处理位置清整数圈指令回复
        this->ProcessMiClearPosRsp(cmd);
        break;
    case CMD_MI_SET_SPD_SPEED:
        this->ProcessMiSpindleSpeedRsp(cmd); // 设置主轴转速指令回复
        break;
    case CMD_MI_DO_SYNC_AXIS:		//处理同步轴同步结果消息
        this->ProcessMiSyncAxis(cmd);
        break;
    case CMD_MI_PMC_AXIS_RUNOVER:   //PMC轴运行到位
        this->PmcAxisRunOver(cmd);
        break;
    case CMD_MI_REFRESH_AXIS_ZERO:  //刷新指定轴的机械零点编码器值
        this->ProcessRefreshAxisZeroEncoder(cmd);
        break;
    case CMD_MI_GET_CUR_ENCODER:   //获取当前编码器反馈单圈绝对值的反馈
        this->ProcessGetCurAxisEncodeRsp(cmd);
        break;
    case CMD_MI_SET_REF_POINT:   //设置参考点命令，回复
        this->ProcessSetAxisRefRsp(cmd);
        break;
    case CMD_MI_GET_ZERO_ENCODER:  //获取指定轴机械零点对应的编码器值
        this->ProcessGetAxisZeroEncoderRsp(cmd);
        break;
    case CMD_MI_ACTIVE_SKIP:    //G31跳转
        this->ProcessSkipCmdRsp(cmd);
        break;
    case CMD_MI_HW_TRACE_STATE_CHANGED:  //手轮跟踪状态切换
        this->ProcessMiHWTraceStateChanged(cmd);
        break;
    case CMD_MI_SET_AXIS_CTRL_MODE:
        this->ProcessMiAxisCtrlModeRsp(cmd); //修改轴控制模式回复
        break;
    case CMD_MI_SET_AXIS_MACH_POS:   //设置轴当前机械坐标
        this->ProcessSetAxisCurMachPosRsp(cmd);
        break;
    case CMD_MI_EN_SYNC_AXIS:    //使能同步轴
        this->ProcessMiEnSyncAxisRsp(cmd);
        break;
    case CMD_MI_SPD_LOCATE:
        this->ProcessMiSpdLocateRsp(cmd); // 主轴定位指令回复
        break;
    case CMD_MI_ACTIVE_Z_CAPT:  //SC激活指定轴的Z信号捕获功能
        this->ProcessMiActiveAxisZCaptureRsp(cmd);
        break;
    default:
        printf("Get unsupported mi cmd[%hu]\n", cmd_no);
        break;
    }
}

/**
 * @brief 处理MI返回的当前编码器单圈绝对值，回参考点时使用
 * @param cmd : mi指令包
 */
void ChannelEngine::ProcessGetCurAxisEncodeRsp(MiCmdFrame &cmd){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "ChannelEngine::ProcessGetCurAxisEncodeRsp: axis = %hhu\n", cmd.data.axis_index-1);
    uint8_t phy_axis = cmd.data.axis_index-1;
    uint64_t encoder = 0;
    memcpy(&encoder, cmd.data.data, sizeof(encoder));
    uint16_t ref_flag = 0x01;    //当前精基准信号位置
    uint16_t ret_dir = this->m_p_axis_config[phy_axis].ret_ref_dir;

    printf("get cur encoder : %llu\n", encoder);

    //	if(encoder < this->m_p_axis_config[phy_axis].motor_count_pr)
    //		ref_flag = 0x10;    //取上一个精基准信号位置

    //	printf("ref_flag = 0x%hx\n", ref_flag);

    //	m_n_get_cur_encoder_count++;
    //	if(m_n_get_cur_encoder_count == 1){//第一次获取
    //		this->m_n_ret_ref_encoder = encoder;
    //
    //		m_n_ret_ref_step[phy_axis] = 7;   //再获取一次
    //		return;
    //	}else if(m_n_get_cur_encoder_count == 2){ //第二次获取，与第一次比较，相同则进行下一步，不同则获取第三次
    //		if(encoder != m_n_ret_ref_encoder){
    //			printf("###cur encoder not match: 1st=%llu, 2nd=%llu\n", m_n_ret_ref_encoder,
    //					encoder);
    //			m_n_ret_ref_step[phy_axis] = 7;   //再获取一次
    //			return;
    //		}
    //	}else if(m_n_get_cur_encoder_count >= 3){
    //		printf("get 3rd encoder : %llu\n", encoder);
    //	}

    //发送设置参考点命令
    MiCmdFrame send;
    memset(&send, 0x00, sizeof(send));
    send.data.cmd = CMD_MI_SET_REF_POINT;
    send.data.axis_index = phy_axis+1;
    send.data.data[0] = ref_flag;
    int64_t pos = m_p_axis_config[phy_axis].axis_home_pos[0]*1e7;   //单位转换,0.1nm
    memcpy(&send.data.data[1], &pos, sizeof(int64_t));
    send.data.data[5] = this->m_p_axis_config[phy_axis].ref_signal;
    send.data.data[6] = ret_dir;


    this->m_p_mi_comm->WriteCmd(send);

}

/**
 * @brief 处理MI返回的设置轴参考点回复指令，回参考点时使用
 * @param cmd : mi指令包
 */
void ChannelEngine::ProcessSetAxisRefRsp(MiCmdFrame &cmd){
    uint8_t  axis = cmd.data.axis_index-1;
    printf("ChannelEngine::ProcessSetAxisRefRsp, axis = 0x%hu\n", axis);
    if(cmd.data.data[0] == FAILED)
    {
        printf("axis [%hhu] set ref point failed!\n", cmd.data.axis_index);
        m_error_code = ERR_RET_REF_FAILED;
        this->m_n_ret_ref_step[axis] = 20;   //跳转到失败处理
    }
    else {
//        if(this->m_p_axis_config[axis].feedback_mode == NO_ENCODER){  //无反馈轴，直接结束
//            m_n_ret_ref_step[axis] = 11;   //跳转到成功结束
//        }else{//其它类型都移动到第一参考点位置
//            //让轴运动到第一参考点位置
//            //	int64_t pos = 0;
//            //	int8_t dir = DIR_POSITIVE;
//            //	memcpy(&pos, &cmd.data.data[1], 8);
//            //		this->m_df_phy_axis_pos_feedback[axis] = (double)pos*1e-7;   //单位转换：0.1nm --> mm
//            //		double dis = this->m_p_axis_config[axis].axis_home_pos[0]-(double)pos*1e-7;   //移动距离
//            //		if(dis < 0)
//            //			dir = DIR_NEGATIVE;

//            //		printf("axis %hhu dir = %hhd, dis = %lf, curpos = %lld\n", axis, dir, dis, pos);

//            //计算粗精基准偏差
//            if(this->m_p_axis_config[axis].ref_base_diff_check){
//                int64_t pos = 0;
//                double df_pos = 0;
//                memcpy(&pos, &cmd.data.data[1], 8);
//                df_pos = pos;
//                df_pos *= 1e-7;   //单位转换：0.1nm-->mm
//                this->m_p_axis_config[axis].ref_base_diff = df_pos;    //粗精基准误差
//                ParamValue value;
//                value.value_double = df_pos;
//                g_ptr_parm_manager->UpdateAxisParam(axis, 1312, value);
//                this->NotifyHmiAxisRefBaseDiffChanged(axis, df_pos);   //通知HMI轴参数改变
//                printf("axis %hhu ref base diff:%lf, %llu\n", axis, df_pos, pos);
//            }

//            /*llx todo
//            if (GetSyncAxisCtrl()->CheckSyncState(axis) == 1)
//            {//从动轴建立机械坐标
//                SetSubAxisRefPoint(axis);
//                usleep(200000);
//            }
//            */


//#ifdef USES_RET_REF_TO_MACH_ZERO
//            this->ManualMoveAbs(axis, m_p_axis_config[axis].ret_ref_speed, 0);
//#else
//            this->ManualMoveAbs(axis, m_p_axis_config[axis].ret_ref_speed, m_p_axis_config[axis].axis_home_pos[0]);
//#endif
//            m_n_ret_ref_step[axis] = 10;   // 跳转下一步
//        }
       int64_t pos = 0;
       double df_pos = 0;
       memcpy(&pos, &cmd.data.data[1], 8);
       df_pos = pos;
       df_pos *= 1e-7;   //单位转换：0.1nm-->mm
       this->m_p_axis_config[axis].ref_base_diff = df_pos;    //粗精基准误差
       ParamValue value;
       value.value_double = df_pos;
       g_ptr_parm_manager->UpdateAxisParam(axis, 1312, value);
       this->NotifyHmiAxisRefBaseDiffChanged(axis, df_pos);   //通知HMI轴参数改变
       printf("axis %hhu ref base diff:%lf, %llu\n", axis, df_pos, pos);

       gettimeofday(&this->m_time_ret_ref[axis], NULL);

       double move_length = 0;
       if (this->m_p_axis_config[axis].ref_z_distance_max == 0)
           move_length = this->m_p_axis_config[axis].move_pr*2;
       else
           move_length = this->m_p_axis_config[axis].ref_z_distance_max;

       if (df_pos <= move_length)
           m_n_ret_ref_step[axis] = 10;
       else
           m_n_ret_ref_step[axis] = 20;
    }
}

/**
 * @brief 处理MI返回的获取机械零点编码器值指令，回参考点时使用
 * @param cmd : MI指令包
 */
void ChannelEngine::ProcessGetAxisZeroEncoderRsp(MiCmdFrame &cmd){
    uint8_t axis = cmd.data.axis_index-1;
    int64_t encoder_value = 0;
    printf("ChannelEngine::ProcessGetAxisZeroEncoderRsp, axis = 0x%hu\n", axis);
    if(cmd.data.data[0] == FAILED){
        printf("axis [%hhu] get zero encoder failed!\n", cmd.data.axis_index);

    }else{
        //读取编码器值并保存
        memcpy(&encoder_value, &cmd.data.data[1], sizeof(int64_t));
        this->m_p_axis_config[axis].ref_encoder = encoder_value;
        g_ptr_parm_manager->UpdateAxisRef(axis, encoder_value); //写入文件
        this->NotifyHmiAxisRefChanged(axis);  //通知HMI
        std::cout << "encoder: " << encoder_value << std::endl;
    }
}

/**
 * @brief 处理MI刷新轴零点编码器值命令
 * @param cmd : MI指令包
 */
void ChannelEngine::ProcessRefreshAxisZeroEncoder(MiCmdFrame &cmd){
    uint8_t axis = cmd.data.axis_index-1;
    int64_t encoder_value = 0;

    //读取编码器值并保存
    memcpy(&encoder_value, &cmd.data.data[0], sizeof(int64_t));
    this->m_p_axis_config[axis].ref_encoder = encoder_value;
    g_ptr_parm_manager->UpdateAxisRef(axis, encoder_value); //写入文件
    this->NotifyHmiAxisRefChanged(axis);
    printf("ChannelEngine::ProcessRefreshAxisZeroEncoder, axis = 0x%hu, encoder = %lld\n", axis, encoder_value);
}

/**
 * @brief 处理MI返回的跳转命令响应
 * @param cmd : MI响应命令包
 */
void ChannelEngine::ProcessSkipCmdRsp(MiCmdFrame &cmd){
    uint8_t chn = cmd.data.data[4]-1;   //通道号从1开始
    if(chn < this->m_p_general_config->chn_count){
        this->m_p_channel_control[chn].ProcessSkipCmdRsp(cmd);
    }
}

/**
 * @brief 处理MI使能同步轴指令的响应
 * @param cmd : MI响应命令包
 */
void ChannelEngine::ProcessMiEnSyncAxisRsp(MiCmdFrame &cmd){
    bool enable = cmd.data.data[1];
    bool success = (cmd.data.data[2]==0);
    m_sync_axis_ctrl->RspEnSyncAxis(enable,success);
}

void ChannelEngine::ProcessMiSpdLocateRsp(MiCmdFrame &cmd)
{
    bool success = cmd.data.data[0];
    m_p_channel_control[0].GetSpdCtrl()->RspORCMA(success);
}

void ChannelEngine::ProcessMiOperateCmdRsp(MiCmdFrame &cmd)
{
    MiCtrlOperate type = (MiCtrlOperate)cmd.data.data[0];
    uint8_t axis = cmd.data.axis_index;
    //printf("cmd.data.data[1] = %d\n",cmd.data.data[1]);
    bool enable = cmd.data.data[1];
    if(type == MOTOR_ON_FLAG){
        m_p_channel_control[0].GetSpdCtrl()->RspAxisEnable(axis,enable);
    }
}

void ChannelEngine::ProcessMiAxisCtrlModeRsp(MiCmdFrame &cmd)
{
    uint8_t axis = cmd.data.axis_index;
    uint8_t mode = cmd.data.data[0];
    if(mode == 1){
        m_p_channel_control[0].GetSpdCtrl()->RspCtrlMode(axis,Position);
    }else if(mode == 2){
        m_p_channel_control[0].GetSpdCtrl()->RspCtrlMode(axis,Speed);
    }
    printf("ProcessMiAxisCtrlModeRsp mode=%d\n",cmd.data.data[0]);
}

void ChannelEngine::ProcessMiSpindleSpeedRsp(MiCmdFrame &cmd)
{
    uint8_t axis = cmd.data.axis_index;
    bool success = cmd.data.data[0];
    m_p_channel_control[0].GetSpdCtrl()->RspSpindleSpeed(axis,success);
}

/**
 * @brief 处理MI捕获Z信号指令响应
 * @param cmd : Mi响应包
 */
void ChannelEngine::ProcessMiActiveAxisZCaptureRsp(MiCmdFrame &cmd){
    int64_t encoder_value = 0;
    uint8_t axis = cmd.data.axis_index-1;  //MI的轴号从1开始，MC中从0开始

    if(axis >= this->m_p_general_config->axis_count){
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_ENGINE_SC, "ChannelEngine::ProcessMiActiveAxisZCaptureRsp(),非法轴号：%hhu", axis);
        return;
    }

    this->ManualMoveStop(axis);
    gettimeofday(&this->m_time_ret_ref[axis], NULL);   //记录起始时间

    if(cmd.data.data[1] == FAILED){
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_ENGINE_SC, "ChannelEngine::ProcessMiActiveAxisZCaptureRsp(), 捕获Z信号失败");
        return;
    }

    if(cmd.data.data[0] == 1){ //总线轴
        memcpy(&encoder_value, &cmd.data.data[2], sizeof(int64_t));
        this->m_p_axis_config[axis].ref_encoder = encoder_value;
        g_ptr_parm_manager->UpdateAxisRef(axis, encoder_value); //写入文件

        //		this->NotifyHmiAxisRefChanged(axis);
        printf("ChannelEngine::ProcessMiActiveAxisZCaptureRsp(), encoder:%lld\n", encoder_value);

        this->m_n_ret_ref_step[axis] = 10;  //回零步骤跳转到第10步
    }else if(cmd.data.data[0] == 0){ //非总线轴

    }
}

/**
 * @brief 处理MI对设置轴当前位置机械坐标命令的响应
 * @param cmd : MI响应命令包
 */
void ChannelEngine::ProcessSetAxisCurMachPosRsp(MiCmdFrame &cmd){
    uint8_t phy_axis = cmd.data.axis_index-1;   //通道号从1开始
    int64_t encoder_value = 0;
    printf("ChannelEngine::ProcessSetAxisCurMachPosRsp, phy_axis = 0x%hhu, res = %hu\n", phy_axis, cmd.data.data[0]);

    //读取编码器值并保存
    memcpy(&encoder_value, &cmd.data.data[1], sizeof(int64_t));
    this->m_p_axis_config[phy_axis].ref_encoder = encoder_value;
    g_ptr_parm_manager->UpdateAxisRef(phy_axis, encoder_value); //写入文件
    this->NotifyHmiAxisRefChanged(phy_axis);
    std::cout << "encoder_value: " << encoder_value << std::endl;
}

/**
 * @brief 通知HMI轴参数参考点粗精基准位置偏差发生变更
 * @param axis ： 物理轴号，从0开始
 * @param diff ： 粗精基准位置偏差
 * @return true--成功    false--失败
 */
bool ChannelEngine::NotifyHmiAxisRefBaseDiffChanged(uint8_t axis, double diff){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.cmd = CMD_SC_PARAM_CHANGED;
    cmd.cmd_extension = AXIS_CONFIG;
    cmd.data_len = 14;
    cmd.data[0] = axis;
    uint32_t param_no = 1312;
    memcpy(&cmd.data[1], &param_no, sizeof(uint32_t));
    cmd.data[5] = 8;    //值类型，double
    memcpy(&cmd.data[6], &diff, sizeof(double));

    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 通知HMI轴参考点对应的编码器值变更
 * @param phy_axis ： 物理轴号，从0开始
 * @return true--成功    false--失败
 */
bool ChannelEngine::NotifyHmiAxisRefChanged(uint8_t phy_axis){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.cmd = CMD_SC_PARAM_CHANGED;
    cmd.cmd_extension = AXIS_CONFIG;
    cmd.data_len = 14;
    cmd.data[0] = phy_axis;
    uint32_t param_no = 1314;
    memcpy(&cmd.data[1], &param_no, sizeof(uint32_t));
    cmd.data[5] = 7;    //值类型，int64
    memcpy(&cmd.data[6], &this->m_p_axis_config[phy_axis].ref_encoder, sizeof(int64_t));

    return this->m_p_hmi_comm->SendCmd(cmd);
}


/**
 * @brief 通知HMI螺补数据更改
 * @return true--成功    false--失败
 */
bool ChannelEngine::NotifyHmiPitchCompDataChanged(){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.cmd = CMD_SC_PARAM_CHANGED;
    cmd.cmd_extension = PITCH_COMP_DATA;
    cmd.data_len = 0;


    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 授权校验
 * @param force ： 强制校验
 * @return 0--合法授权   1--非法授权
 */
int ChannelEngine::CheckLicense(bool force){

    LitronucLicInfo *lpLicInfo = &this->m_lic_info;

    char cursn[SN_COUNT+1];
    memset(cursn, 0x00, sizeof(cursn));
    memcpy(cursn, m_device_sn, SN_COUNT);


    int nLen1 = strlen(cursn);
    int nLen2 = strlen(lpLicInfo->sn);
    if((nLen1 != nLen2) || (nLen1 != SN_COUNT) ||
            memcmp(cursn, lpLicInfo->sn, nLen1) != 0)
    {
        lpLicInfo->licflag = 'v';

        CreateError(ERR_LICENSE_INVALID, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
        return 1;   //非法授权
    }

    ////////////////////////////////////////////////////

    if(!force && (lpLicInfo->licflag == 'n' || lpLicInfo->licflag == 'o' || lpLicInfo->licflag == 'v'))
    {
        if(lpLicInfo->licflag == 'v')
            CreateError(ERR_LICENSE_INVALID, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
        else if(lpLicInfo->licflag == 'o')
            CreateError(ERR_LIC_OVERTIME, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
        else
            CreateError(ERR_NO_LIC, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
        return 1; //非法授权
    }

    if(!force && lpLicInfo->licflag == 'f')
    {
        return 0;
    }

    if(lpLicInfo->dead_line.year == 9999 && lpLicInfo->dead_line.month == 99 && lpLicInfo->dead_line.day == 99){
        lpLicInfo->licflag = 'f';
        return 0;
    }



    time_t now = time(nullptr);  //获取当前日期
    struct tm nowday;
    localtime_r(&now, &nowday);
    long curdaycount = (nowday.tm_year+1900) * 365L + nowday.tm_mon * 30 + nowday.tm_mday;
    long deaddaycount = lpLicInfo->dead_line.year * 365L + (lpLicInfo->dead_line.month - 1) * 30
            + lpLicInfo->dead_line.day;

    curdaycount = (curdaycount - 1) * 24 + nowday.tm_hour;   //小时数
    deaddaycount = (deaddaycount - 1) * 24 + 12;

    long leave = deaddaycount - curdaycount;

    if(deaddaycount < curdaycount)
    {
        //许可过期
        lpLicInfo->licflag = 'o';
        CreateError(ERR_LIC_OVERTIME, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
        return 1;
    }
    else if(leave <= 48)
    {
        //许可即将过期
        lpLicInfo->licflag = 'm';

        if(leave != lpLicInfo->nWarn){
            lpLicInfo->nInfo = 1;
            lpLicInfo->nWarn = leave;   //剩余小时数

            CreateError(ERR_LIC_DUE_SOON, INFO_LEVEL, CLEAR_BY_MCP_RESET, leave);
        }
    }
    else if((deaddaycount - curdaycount) > 48)
    {
        //许可期内
        lpLicInfo->licflag = 'p';
        lpLicInfo->nInfo = 0;
        lpLicInfo->nWarn = 0;
    }


    return 0;
}

/**
 * @brief 向HMI发送提示信息
 * @param msg_type : 消息类型
 * @param msg_id ： 消息ID
 * @param msg_param : 消息参数
 * @return true--发送成功  false--发送失败
 */
bool ChannelEngine::SendMessageToHmi(uint16_t msg_type, uint32_t msg_id, uint32_t msg_param){
    HMICmdFrame cmd;
    memset((void *)&cmd, 0x00, sizeof(HMICmdFrame));
    cmd.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.cmd = CMD_SC_MESSAGE;
    cmd.cmd_extension = msg_type;
    cmd.data_len = sizeof(uint32_t)*2;
    memcpy(cmd.data, &msg_id, sizeof(uint32_t));
    memcpy(cmd.data+sizeof(uint32_t), &msg_param, sizeof(uint32_t));

    return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 处理MI握手指令
 * @param cmd
 */
void ChannelEngine::ProcessMiShakehand(MiCmdFrame &cmd){
    //	bool rsp = (cmd.data.cmd & 0x8000) == 0 ? false:true;  //是否响应包

    if(cmd.data.data[0] == 0x9696){
        g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "@#@#Get MI Shakehand cmd, module=0x%hhx\n", g_sys_state.module_ready_mask);
        this->ProcessMiVersionCmd(cmd);   //获取MI版本号
        cmd.data.cmd |= 0x8000;
        if(g_sys_state.module_ready_mask & SC_READY){
            cmd.data.data[0] = 0x8888;
            cmd.data.data[1] = this->m_p_general_config->bus_cycle+1;  //MI的参数从1开始,1表示125us，2表示250us
            cmd.data.data[2] = (this->m_p_general_config->chn_count << 8) + this->m_p_general_config->axis_count;   //实际物理轴数量         实际通道数量
            //			cmd.data.data[2] = this->m_p_general_config->axis_count;   //实际物理轴数量  // ??????????????????????????????????????????????  二选一  是否加一个宏

            cmd.data.data[3] = this->GetBusAxisCount();

            printf("reply shakehand cmd %hu, %hu, %hu\n", cmd.data.data[1], cmd.data.data[2], cmd.data.data[3]);
        }
        else
            cmd.data.data[0] = 0;
        this->m_p_mi_comm->WriteCmd(cmd);

    }
    else
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC,"Get invalid mi shakehand cmd: 0x%x\n", cmd.data.data[0]);
}

/**
 * @brief 处理PMC梯形图请求命令
 * @param cmd : 待处理MI命令
 */
void ChannelEngine::ProcessMiPmcLadderReq(MiCmdFrame &cmd){

    uint16_t index = cmd.data.data[0];

    uint32_t res = LoadPmcLadderData(index, cmd.data.data[3]); //加载梯形图数据
    memcpy(cmd.data.data, &res, sizeof(uint32_t));
    cmd.data.data[2] = index;

    cmd.data.cmd |= 0x8000;
    this->m_p_mi_comm->WriteCmd(cmd);

}

/**
 * @brief 处理MI获取伺服描述文件命令
 * @param cmd : 待处理MI命令
 */
void ChannelEngine::ProcessMiGetESBCmd(MiCmdFrame &cmd){
    uint16_t index = cmd.data.data[0];

    uint32_t res = LoadEsbData(index, cmd.data.data[3]); //加载ESB文件数据

    memcpy(cmd.data.data, &res, sizeof(uint32_t));
    cmd.data.data[2] = index;

    cmd.data.cmd |= 0x8000;
    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 处理MI告警信息
 * @param cmd
 */
void ChannelEngine::ProcessMiAlarm(MiCmdFrame &cmd){
    uint32_t alarm_id = 0;
    //uint16_t alarm_level = WARNING_LEVEL;
    //uint8_t clear_type = CLEAR_BY_MCP_RESET;
    uint16_t alarm_level = FATAL_LEVEL;
    uint8_t clear_type = CLEAR_BY_RESET_POWER;

    alarm_id = cmd.data.data[1]; //高16位
    alarm_id = alarm_id<<16;
    alarm_id += cmd.data.data[0];   //低16位
    alarm_level = cmd.data.data[2];

    //    if(alarm_level >= WARNING_LEVEL)
    //        clear_type = CLEAR_BY_CLEAR_BUTTON;

    g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_ENGINE_SC, "ChannelEngine::ProcessMiAlarm, alarm_id=%u, level=%hu", alarm_id, alarm_level);

    CreateError(alarm_id, alarm_level, clear_type, 0, CHANNEL_ENGINE_INDEX, cmd.data.axis_index);
}

/**
 * @brief 处理MI返回的编码器值
 * @param cmd
 */
void ChannelEngine::ProcessMiSetRefCurRsp(MiCmdFrame &cmd){
    int64_t encoder_value = 0;
    uint8_t axis = cmd.data.axis_index-1;  //MI的轴号从1开始，MC中从0开始

    if(axis >= this->m_p_general_config->axis_count){
        printf("ChannelEngine::ProcessMiSetRefCurRsp(),非法轴号：%hhu\n", axis);
        return;
    }

    this->SetRetRefFlag(axis, true);
    this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<axis);   //置位到参考点标志

    memcpy(&encoder_value, cmd.data.data, sizeof(int64_t));
    this->m_p_axis_config[axis].ref_encoder = encoder_value;
    g_ptr_parm_manager->UpdateAxisRef(axis, encoder_value); //写入文件

    this->NotifyHmiAxisRefChanged(axis);

    printf("ProcessMiSetRefCurRsp , axis = %hhu, encoder= %lld\n", axis, encoder_value);

}

/**
 * @brief 处理MI返回的清整数圈位置指令回复
 * @param cmd
 */
void ChannelEngine::ProcessMiClearPosRsp(MiCmdFrame &cmd){
    int64_t encoder_value = 0;
    uint8_t axis = cmd.data.axis_index-1;  //MI的轴号从1开始，MC中从0开始

    printf("ProcessMiClearPosRsp , axis = %hhu\n", axis);

    memcpy(&encoder_value, cmd.data.data, sizeof(int64_t));
    this->m_p_axis_config[axis].ref_encoder = encoder_value;
    g_ptr_parm_manager->UpdateAxisRef(axis, encoder_value); //写入文件

    this->NotifyHmiAxisRefChanged(axis);

    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        this->m_p_channel_control[i].SetClearPosAxisMask(cmd.data.axis_index);
    }
}

/**
 * @brief 处理MI返回的同步轴同步结果
 * @param cmd
 */
void ChannelEngine::ProcessMiSyncAxis(MiCmdFrame &cmd){
    int64_t mask = 0;
    memcpy(&mask, cmd.data.data, 8);
    m_sync_axis_ctrl->RspSyncAxis(mask);
}

/**
 * @brief 处理MI发送的手轮跟踪状态切换指令
 * @param cmd : MI的命令
 */
void ChannelEngine::ProcessMiHWTraceStateChanged(MiCmdFrame &cmd){
    uint8_t chn_idx = cmd.data.reserved;   //通道号
    this->m_p_channel_control[chn_idx].ProcessMiHWTraceStateChanged(cmd);
}

/**
 * @brief 处理MI返回的总线错误
 * @param cmd : MI命令包
 */
void ChannelEngine::ProcessMiBusError(MiCmdFrame &cmd){
    uint8_t slave_no = 0;	//出错从站号
    uint8_t err_sub_index = 0;	//子索引
    uint16_t err_index = 0;	//索引
    uint16_t err_code = 0; //错误码

    slave_no = (cmd.data.data[0] & 0xFF);
    err_sub_index = ((cmd.data.data[0]>>8)&0xFF);
    err_index = cmd.data.data[1];
    err_code = cmd.data.data[2];

    uint32_t err_info = err_index;
    err_info <<= 8;
    err_info |= err_sub_index;
    err_info <<= 8;
    err_info |= slave_no;

    g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "receive Mi Bus err: no=%hhu, sub_idx=%hhu, idx=%hu, code=%hu\n",
                            slave_no, err_sub_index, err_index, err_code);
    this->m_error_code = static_cast<ErrorType>(err_code);
    CreateError(err_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, err_info, CHANNEL_ENGINE_INDEX, slave_no);
}

/**
 * @brief 加载PMC梯形图至共享区
 * @param index : 梯形图数据帧号，一帧数据最大128KB
 * @param flag[out] : 是否还有后续数据， 1--还有后续帧    0--没有后续帧
 * @return 返回加载的数据字节数
 */
int32_t ChannelEngine::LoadPmcLadderData(uint16_t index, uint16_t &flag){
    int32_t res = 0;

    //打开梯形图文件
    struct stat statbuf;
    int file_size = 0;
    if(stat(PATH_PMC_DATA, &statbuf) == 0)
        file_size = statbuf.st_size;  //获取文件总大小
    else{
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "获取文件[%s]大小失败！", PATH_PMC_DATA);
        return res;			//获取文件大小失败
    }

    printf("read pmc file size : %d\n", file_size);

    int fp = open(PATH_PMC_DATA, O_RDONLY); //只读打开文件
    if(fp < 0){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "打开文件[%s]失败！", PATH_PMC_DATA);
        return res;//文件打开失败
    }

    //计算文件总帧数
    int block_total = file_size/MI_PMC_DATA_SIZE;
    if(file_size%MI_PMC_DATA_SIZE)
        block_total++;

    if(index >= block_total){
        close(fp);
        printf("invalid parameter, index = %d, block_total = %d!\n", index, block_total);
        return res;  	//index不合法
    }

    uint64_t offset = MI_PMC_DATA_SIZE*index;

    if(-1 == lseek(fp, offset, SEEK_SET)){
        printf("lseek function failed, errno = %d\n", errno);
        close(fp);
        return res;
    }



    //读取数据
    int32_t read_size = file_size - offset;
    if(read_size > MI_PMC_DATA_SIZE)
        read_size = MI_PMC_DATA_SIZE;
    char *ptr_data = this->m_p_mi_comm->GetPmcDataAddr();
    res = read(fp, ptr_data, read_size);


    if(res == -1 || res != read_size){  //read失败
        printf("read pmc file failed, errno = %d, block = %d\n", errno, block_total);
        close(fp);
        return 0;
    }

    if(index < block_total-1)
        flag = 1;
    else
        flag = 0;


    close(fp);
    printf("load pmc file block %d succeed, %d!\n", index, flag);
    return res;
}

/**
 * @brief 加载ESB文件数据，伺服描述文件
 * @param index : 文件顺序号，从0开始
 * @param flag ：是否还有后续文件标志，0表示无后续文件，1表示有后续文件
 * @return 返回读取文件字节数
 */
int32_t ChannelEngine::LoadEsbData(uint16_t index, uint16_t &flag){
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "Enter ChannelEngine::LoadEsbData(), index=%hu", index);
    int32_t res = 0;

    flag = 0;  //默认后续无文件


    DIR *pdir = nullptr;
    struct dirent *ent = nullptr;
    char file_path[kMaxPathLen] = {0};
    int idx = 0;
    int len = 0;
    char *pt = nullptr;
    bool bfind = false;   //找到对应文件


    pdir  = opendir(PATH_ESB_FILE);
    if(pdir == nullptr){//路径打开失败
        //创建目录
        if(mkdir(PATH_ESB_FILE, 0755) == -1){//创建目录失败
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "创建目录失败！[%s]", PATH_ESB_FILE);
        }else{
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "打开目录[%s]失败！自动创建改目录！", PATH_ESB_FILE);
        }

        //返回读取失败
        return res;
    }

    //遍历ESB文件目录
    while((ent = readdir(pdir)) != nullptr){
        if(ent->d_type &DT_DIR){//目录
            continue;
        }else if(ent->d_type & DT_REG){//普通文件
            //校验文件后缀是不是.esb
            len = strlen(ent->d_name);
            if(len > 4){
                pt = strrchr(ent->d_name, '.');
                if(pt && strcasecmp(pt, "esb")){  //后缀匹配
                    if(idx >= index){//顺序号匹配
                        //生成当前加载文件路径
                        if(!bfind){
                            bfind = true;
                            strcpy(file_path, PATH_ESB_FILE);
                            strcat(file_path, ent->d_name);
                            printf("open esb file:%s\n", file_path);
                        }else{
                            flag = 1;   //后续还有文件
                            break;
                        }

                    }
                    idx++;
                }
            }
        }
    }

    //加载当前指定顺序号的文件
    struct stat statbuf;
    int file_size = 0;
    if(stat(file_path, &statbuf) == 0)
        file_size = statbuf.st_size;  //获取文件总大小
    else{
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "获取文件[%s]大小失败！", file_path);
        closedir(pdir);
        return res;			//获取文件大小失败
    }

    if(file_size < kEsbDataSize){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ESB文件[%s]大小不匹配[size=%d]！", file_path, file_size);
        closedir(pdir);
        return res;   //文件大小不匹配
    }

    //printf("read esb file size : %d\n", file_size);

    int fp = open(file_path, O_RDONLY); //只读打开文件
    if(fp < 0){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "打开文件[%s]失败！", file_path);
        closedir(pdir);
        return res;//文件打开失败
    }

    //读取数据
    int32_t read_size = kEsbDataSize;
    char *ptr_data = this->m_p_mi_comm->GetPmcDataAddr();   //复用PMC梯图的传输区域
    res = read(fp, ptr_data, read_size);


    if(res == -1 || res != read_size){  //read失败
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "读取文件[%s]失败！实际读取大小[%d]字节", file_path, res);
        close(fp);
        closedir(pdir);
        return 0;
    }

    close(fp);
    closedir(pdir);
    g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "Exit ChannelEngine::LoadEsbData(), flag=%hu, bytes=%d", flag, res);

    //printf("===== read esb file size: %d\n", res);
    return res;
}

/**
 * @brief 处理HMI的IO请求
 * @param cmd
 */
void ChannelEngine::ProcessHmiIOCmd(HMICmdFrame &cmd){

    //	uint8_t value = 0;
    switch(cmd.cmd_extension){
    default:
        break;
    }
}

/**
 * @brief 处理HMI模块发送的命令
 * @param cmd ：待处理的HMI命令包
 */
void ChannelEngine::ProcessHmiCmd(HMICmdFrame &cmd){

    int i = 0;
    switch(cmd.cmd){
    case CMD_HMI_GET_CHN_STATE:   //HMI获取通道当前状态
        if(cmd.channel_index >= this->m_p_general_config->chn_count){
            cmd.cmd_extension = FAILED;
            cmd.frame_number |= 0x8000;
            this->m_p_hmi_comm->SendCmd(cmd);
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "命令[%d]通道号非法！%d", cmd.cmd, cmd.channel_index);
        }else
            this->m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
        break;
    case CMD_HMI_RESTART:               //加工复位 11
    case CMD_HMI_SIMULATE:				//仿真 12
        if(cmd.channel_index == CHANNEL_ENGINE_INDEX){
            for(i = 0; i < m_p_general_config->chn_count; i++){
                m_p_channel_control[i].ProcessHmiCmd(cmd);
            }
        }
        else if(cmd.channel_index >= this->m_p_general_config->chn_count){
            cmd.frame_number |= 0x8000;
            cmd.data_len = 1;
            cmd.data[0] = FAILED;
            this->m_p_hmi_comm->SendCmd(cmd);
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "命令[%d]通道号非法！%d", cmd.cmd, cmd.channel_index);
        }
        else{
            m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
        }
        break;
    case CMD_HMI_SET_NC_FILE:	    //设置当前加工文件 13
        if(cmd.channel_index == CHANNEL_ENGINE_INDEX){
            for(i = 0; i < m_p_general_config->chn_count; i++){
                m_p_channel_control[i].ProcessHmiCmd(cmd);
            }
        }
        else if(cmd.channel_index >= this->m_p_general_config->chn_count){
            cmd.frame_number |= 0x8000;
            cmd.cmd_extension = FAILED;
            this->m_p_hmi_comm->SendCmd(cmd);
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "命令[%d]通道号非法！%d", cmd.cmd, cmd.channel_index);
        }
        else{
            m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
        }
        break;
    case CMD_HMI_FIND_REF_POINT:		//确定参考点 14
        this->ProcessHmiFindRefCmd(cmd);
        break;
    case CMD_HMI_SET_REF_POINT:			//设置轴原点，针对绝对值编码器
        if(this->m_n_cur_pmc_axis != 0xFF){
            this->ProcessHmiSetRefCmd(cmd);
        }else{
            this->m_p_channel_control[this->m_n_cur_channle_index].ProcessHmiCmd(cmd);
        }

        break;
    case CMD_SC_MDA_DATA_REQ:			//MDA代码请求 115
    case CMD_HMI_GET_MACRO_VAR:			//HMI向SC请求宏变量的值   30
    case CMD_HMI_SET_MACRO_VAR:        //HMI向SC设置宏变量寄存器的值   31
    case CMD_HMI_SET_CALIBRATION:      //HMI向SC发出调高器标定指令 32
    case CMD_HMI_MANUAL_TOOL_MEASURE:     //HMI向SC发起手动对刀操作  0x29
        if(cmd.channel_index < this->m_p_general_config->chn_count)
            m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
        else
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "命令[%d]通道号非法！%d", cmd.cmd, cmd.channel_index);
        break;
    case CMD_HMI_CLEAR_WORKPIECE:      //HMI请求SC将加工计数清零,临时计数(区分白夜班)
    case CMD_HMI_CLEAR_TOTAL_PIECE:    //总共计数清零
    case CMD_HMI_SET_REQUIRE_PIECE:    //设置需求件数
        if(cmd.channel_index < this->m_p_general_config->chn_count)
            m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
        else if(cmd.channel_index == CHANNEL_ENGINE_INDEX){
            for(int i = 0; i < this->m_p_general_config->chn_count; i++){
                this->m_p_channel_control[i].ProcessHmiCmd(cmd);
            }
        }else
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "命令[%d]通道号非法！%d", cmd.cmd, cmd.channel_index);
        break;
    case CMD_HMI_SET_PARA:		//设置参数
        this->ProcessHmiSetParam(cmd);
        break;
    case CMD_HMI_GET_PARA:		//获取参数
        this->ProcessHmiGetParam(cmd);
        break;
    case CMD_HMI_UPDATE:		//HMI通知SC请求进行升级文件传送
        this->ProcessHmiUpdateReq(cmd);
        break;
    case CMD_HMI_GET_PMC_REG:			//获取PMC寄存器
        this->ProcessHmiGetPmcReg(cmd);
        break;
    case CMD_HMI_SET_PMC_REG:			//设置PMC寄存器
        this->ProcessHmiSetPmcReg(cmd);
        break;
    case CMD_HMI_GET_PMC_UUID:
        this->ProcessHmiGetPmcUuid(cmd);	//获取pmc的uuid
        break;
    case CMD_HMI_AXIS_MANUAL_MOVE:     //HMI指令轴移动
        this->ProcessHmiAxisMoveCmd(cmd);
        break;
    case CMD_HMI_GET_LIC_INFO:            //HMI向SC请求授权信息   0x27
        this->ProcessHmiGetLicInfoCmd(cmd);
        break;
    case CMD_HMI_SEND_LICENSE:            //HMI向SC发送授权码     0x28
        this->ProcessHmiRegLicCmd(cmd);
        break;
    case CMD_HMI_GET_ESB_INFO:            //HMI向SC获取ESB文件数据   0x2A

        break;
    case CMD_HMI_ESB_OPERATE:            //HMI通知SC对指定ESB文件进行操作  0x2B

        break;
    case CMD_HMI_GET_IO_REMAP:			//HMI向SC获取IO重定向数据  0x2C
        this->ProcessHmiGetIoRemapInfoCmd(cmd);
        break;
    case CMD_HMI_SET_IO_REMAP:         //HMI向SC设置IO重定向数据  0x2D
        this->ProcessHmiSetIoRemapInfoCmd(cmd);
        break;
    case CMD_HMI_SET_PROC_PARAM:      //HMI向SC设置工艺相关参数  0x2E
        this->ProcessHmiSetProcParamCmd(cmd);
        break;
    case CMD_HMI_GET_PROC_PARAM:          //HMI向SC获取工艺相关参数  0x2F
        this->ProcessHmiGetProcParamCmd(cmd);
        break;
    case CMD_HMI_SET_PROC_GROUP:          //HMI向SC设置当前工艺参数组号  0x30
        this->ProcessHmiSetCurProcIndex(cmd);
        break;
    case CMD_HMI_GET_PROC_GROUP:          //HMI向SC获取当前工艺参数组号  0x31
        this->ProcessHmiGetCurProcIndex(cmd);
        break;
    case CMD_HMI_SET_CUR_MACH_POS:        //HMI向SC重设指定轴的机械坐标  0x32
        if(cmd.channel_index >= this->m_p_general_config->chn_count){
            cmd.frame_number |= 0x8000;
            cmd.data[9] = FAILED;
            cmd.data_len = 10;
            this->m_p_hmi_comm->SendCmd(cmd);
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "命令[%d]通道号非法！%d", cmd.cmd, cmd.channel_index);
        }else
            this->m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
        break;
    case CMD_HMI_CLEAR_MSG:               //HMI通知SC清除消息  0x33
        printf("process CMD_HMI_CLEAR_MSG:%hhu, cmd_ex=%hhu \n", cmd.channel_index, cmd.cmd_extension);
        if(cmd.channel_index == CHANNEL_ENGINE_INDEX){
            this->ProcessHmiClearMsgCmd(cmd);
        }else if(cmd.channel_index < this->m_p_general_config->chn_count){
            this->m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
        }else{
            cmd.frame_number |= 0x8000;
            cmd.cmd_extension = FAILED;
            cmd.data_len = 0;
            this->m_p_hmi_comm->SendCmd(cmd);
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "命令[%d]通道号非法！%d", cmd.cmd, cmd.channel_index);
        }
        break;
    case CMD_HMI_NOTIFY_GRAPH:            //HMI通知SC进入图形模式    0x35
        this->ProcessHmiNotifyGraphCmd(cmd);
        break;
        //    case CMD_HMI_CHECK_SYNC_EN:           //HMI向SC查询同步轴状态 0x37
        //        this->ProcessHmiCheckSyncCmd(cmd);
        //        break;
    case CMD_HMI_GET_SYS_INFO:
    {FILE *stream;
        float val;
        float temp_value;
        char buf[20] = "";
        if ((stream = fopen("/sys/bus/iio/devices/iio:device0/in_temp0_raw", "w+"))== NULL)
        {
            fprintf(stderr,"Cannot open output file.\n");
        }

        fseek(stream, SEEK_SET, 0);
        fread(buf, sizeof(char), sizeof(buf), stream);
        val=atof(buf);
        temp_value=((val * 503.975)/(1<<12) )-273.15;
        printf("The cpu temp is %.2f\r\n",temp_value);
        fclose(stream);
        break;}
    case CMD_HMI_CLEAR_MACHINETIME_TOTAL:
        if(cmd.channel_index < this->m_p_general_config->chn_count)
        {
            m_p_channel_control[cmd.channel_index].ClearMachineTimeTotal();
            g_ptr_parm_manager->SetCurTotalMachiningTime(cmd.channel_index, 0);
        }
        else if(cmd.channel_index == CHANNEL_ENGINE_INDEX){
            for(int i = 0; i < this->m_p_general_config->chn_count; i++){
                this->m_p_channel_control[i].ClearMachineTimeTotal();
                g_ptr_parm_manager->SetCurTotalMachiningTime(i, 0);
            }
        }
        break;
    case CMD_HMI_GET_HANDWHEEL_INFO:           // hmi向sc查询手轮映射关系
        ProcessHmiHandWheelCmd(cmd);
        break;
    case CMD_HMI_SET_HANDWHEEL_INFO:           // hmi向sc设置手轮信息
    {
        HandWheelMapInfoVec configInfo = g_ptr_parm_manager->GetHandWheelVec();
        std::cout << cmd.cmd_extension << " " << configInfo.size() << std::endl;
        if (cmd.cmd_extension - 1 < configInfo.size() && cmd.channel_index <= 1)//暂时只支持单通道
        {
            for(auto itr = configInfo.begin(); itr != configInfo.end(); ++itr)
            {
                itr->devNum == cmd.cmd_extension
                        ? itr->channelMap = cmd.channel_index : itr->channelMap = 0;
            }
            g_ptr_parm_manager->SyncHandWheelInfo(configInfo, true);
        }
        else
        {
            CreateError(ERR_PMC_SDLINK_CONFIG, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
        }
    }
        break;
    case CMD_HMI_GET_ERROR_INFO:
        this->ProcessHmiGetErrorCmd(cmd);
        break;
    case CMD_HMI_SET_ALL_COORD: //设置当前通道的所有工件坐标系
        ProcessHmiSetAllCoordCmd(cmd);
        break;
    case CMD_HMI_ABSOLUTE_REF_SET:
        ProcessHmiAbsoluteRefSet(cmd);
        break;
    case CMD_HMI_SET_ALL_TOOL_OFFSET: //HMI向SC请求设置所有刀偏值
        HmiToolOffsetConfig cfg;
        memcpy(&cfg, cmd.data, cmd.data_len);
        if(cmd.channel_index < this->m_p_general_config->chn_count)
        {
            m_p_channel_control[cmd.channel_index].UpdateAllToolOffset(cfg);
        }
        else if(cmd.channel_index == CHANNEL_ENGINE_INDEX){
            for(int i = 0; i < this->m_p_general_config->chn_count; i++){
                this->m_p_channel_control[i].UpdateAllToolOffset(cfg);
            }
        }
        break;
    default:
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_ENGINE_SC, "不支持的HMI指令[%d]", cmd.cmd);
        break;

    }
}

/**
 * @brief 处理HMI获取授权信息指令
 * @param cmd : HMI指令包
 */
void ChannelEngine::ProcessHmiGetLicInfoCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;  //设置回复标志
    cmd.cmd_extension = SUCCEED;
    cmd.data_len = 24 + sizeof(int);

    memcpy(cmd.data, this->m_device_sn, SN_COUNT);  //13字节长度的设备SN号
    cmd.data[SN_COUNT] = this->m_lic_info.licflag;    //授权状态
    if(this->m_lic_info.licflag == 'f'){
        sprintf(cmd.data+SN_COUNT+1, "%s", "永久");    //授权到期时间,10字节长度字符串，格式如“2021-12-21”
    }else{
        sprintf(cmd.data+SN_COUNT+1, "%d-%02d-%02d", this->m_lic_info.dead_line.year, this->m_lic_info.dead_line.month,
                this->m_lic_info.dead_line.day);    //授权到期时间,10字节长度字符串，格式如“2021-12-21”
    }

    int remainDays = GetRemainDay();
    memcpy(cmd.data+SN_COUNT+10+1, (char *)&remainDays, sizeof(remainDays));


    this->m_p_hmi_comm->SendCmd(cmd);

}

/**
 * @brief 处理HMI注册授权指令
 * @param cmd : HMI指令包
 */
void ChannelEngine::ProcessHmiRegLicCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;  //设置回复标志

    if(cmd.data_len != LICCODECOUNT){//授权码长度错误，返回失败
        cmd.cmd_extension = FAILED;
        cmd.data_len = 24 + sizeof(int);

        memcpy(cmd.data, this->m_device_sn, SN_COUNT);  //13字节长度的设备SN号
        cmd.data[SN_COUNT] = this->m_lic_info.licflag;    //授权状态

        sprintf(cmd.data+SN_COUNT+1, "%d-%02d-%02d", this->m_lic_info.dead_line.year, this->m_lic_info.dead_line.month,
                this->m_lic_info.dead_line.day);    //授权到期时间,10字节长度字符串，格式如“2021-12-21”

        int remainDays = 0;
        memcpy(cmd.data+SN_COUNT+10+1, &remainDays, sizeof(remainDays));

        this->m_p_hmi_comm->SendCmd(cmd);
        printf("register failed 1\n");
        return;
    }

    char lic_code[LICCODECOUNT+1];   //授权码
    memset(lic_code, 0x00, LICCODECOUNT+1);
    memcpy(lic_code, cmd.data, LICCODECOUNT);  //24字节长度的设备授权码
    printf("Get lic code:%s\n", lic_code);


#ifdef USES_LICENSE_FUNC
    char lic[25] = {};
    memcpy(lic, lic_code, 25);
    if (!DecryptLicense(lic))
    {
        std::cout << "illegal code" << std::endl;
        cmd.cmd_extension = FAILED;
        this->m_p_hmi_comm->SendCmd(cmd);
        return;
    }
    std::cout << lic_code << std::endl;
#endif

    //注册授权码
    if(RegisterLicWithCode(this->m_device_sn, lic_code, &this->m_lic_info)){
        cmd.cmd_extension = FAILED;
        cmd.data_len = 24 + sizeof(int);

        memcpy(cmd.data, m_device_sn, SN_COUNT);  //13字节长度的设备SN号
        cmd.data[SN_COUNT] = this->m_lic_info.licflag;    //授权状态
        sprintf(cmd.data+SN_COUNT+1, "%d-%02d-%02d", this->m_lic_info.dead_line.year, this->m_lic_info.dead_line.month,
                this->m_lic_info.dead_line.day);    //授权到期时间,10字节长度字符串，格式如“2021-12-21”

        int remainDays = 0;
        memcpy(cmd.data+SN_COUNT+10+1, &remainDays, sizeof(remainDays));

        this->m_p_hmi_comm->SendCmd(cmd);
        printf("register failed 2\n");
        return;
    }

    this->CheckLicense(true);

    cmd.data_len = 24 + sizeof(int);

    memcpy(cmd.data, m_device_sn, SN_COUNT);  //13字节长度的设备SN号
    cmd.data[SN_COUNT] = this->m_lic_info.licflag;    //授权状态
    if(this->m_lic_info.licflag == 'f'){
        sprintf(cmd.data+SN_COUNT+1, "%s", "永久");    //授权到期时间,10字节长度字符串，格式如“2021-12-21”

    }else{
        sprintf(cmd.data+SN_COUNT+1, "%d-%02d-%02d", this->m_lic_info.dead_line.year, this->m_lic_info.dead_line.month,
                this->m_lic_info.dead_line.day);    //授权到期时间,10字节长度字符串，格式如“2021-12-21”
    }

    int remainDays = GetRemainDay();
    memcpy(cmd.data+SN_COUNT+10+1, &remainDays, sizeof(remainDays));

    printf("register lic result: %s\n", cmd.data);

    //发送回复
    cmd.cmd_extension = SUCCEED;
    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 处理HMI获取IO重映射信息命令
 * @param cmd : HMI指令包
 */
void ChannelEngine::ProcessHmiGetIoRemapInfoCmd(HMICmdFrame &cmd){
    int count = this->m_p_io_remap->GetLength();

    int count_per = count;   //单次发送数据量
    int i = 0;
    ListNode<IoRemapInfo> *info_node = this->m_p_io_remap->HeadNode();

    cmd.frame_number |= 0x8000;  //设置回复标志
    if(count == 0){
        cmd.cmd_extension = 0x00;
        cmd.data_len = 0;
        this->m_p_hmi_comm->SendCmd(cmd);
        return;
    }
    while(count > 0){
        if(count > 100)
            count_per = 100;
        else
            count_per = count;

        count -= count_per;

        if(count > 0)
            cmd.cmd_extension = 0x01;
        else
            cmd.cmd_extension = 0x00;

        cmd.data_len = 9*count_per;
        for(i = 0; i < count_per && info_node != nullptr; i++){
            cmd.data[9*i] = info_node->data.iotype;
            memcpy(&cmd.data[9*i+1], &info_node->data.addr, 2);
            cmd.data[9*i+3] = info_node->data.bit;
            cmd.data[9*i+4] = info_node->data.addrmap;
            memcpy(&cmd.data[9*i+5], &info_node->data.newaddr, 2);
            cmd.data[9*i+7] = info_node->data.newbit;
            cmd.data[9*i+8] = info_node->data.valtype;

            info_node = info_node->next;
        }

        this->m_p_hmi_comm->SendCmd(cmd);
    }

}

/**
 * @brief 处理HMI设置IO重映射信息命令
 * @param cmd : HMI指令包
 */
void ChannelEngine::ProcessHmiSetIoRemapInfoCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;  //设置回复标志

    IoRemapInfo info;
    info.iotype = cmd.data[0];
    memcpy(&info.addr, &cmd.data[1], 2);
    info.bit = cmd.data[3];
    info.addrmap = cmd.data[4];
    memcpy(&info.newaddr, &cmd.data[5], 2);
    info.newbit = cmd.data[7];
    info.valtype = cmd.data[8];

    ListNode<IoRemapInfo> *info_node = this->m_p_io_remap->HeadNode();
    bool flag = false;
    while(info_node != nullptr){
        if(info_node->data.iotype == info.iotype &&
                info_node->data.addr == info.addr &&
                info_node->data.bit == info.bit){
            info_node->data = info;
            flag = true;
            break;
        }
        info_node = info_node->next;
    }

    if(!flag){  //新增
        this->m_p_io_remap->Append(info);
    }

    if(!g_ptr_parm_manager->UpdateIoRemapInfo(info)){
        cmd.cmd_extension = 0x01;  //更新失败
    }

    this->SendMiIoRemapInfo(info);   //更新MI数据

    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 处理HMI设置工艺相关参数的命令
 * @param cmd : HMI指令包
 */
void ChannelEngine::ProcessHmiSetProcParamCmd(HMICmdFrame &cmd){
    //	printf("enter ProcessHmiSetProcParamCmd\n");
    cmd.frame_number |= 0x8000;  //设置回复标志

    uint8_t active_type = ACTIVE_BY_POWEROFF;	//激活类型
    ProcParamUpdate update;
    update.param_data.param_type = cmd.cmd_extension;  //参数类型，2--通道参数  3--轴参数
    update.group_index = cmd.data[0];       //工艺参数组号

    switch(cmd.cmd_extension){
    case CHN_CONFIG:
        update.param_data.chn_index = cmd.channel_index;
        memcpy(&update.param_data.param_no, &cmd.data[1], 4);
        active_type = cmd.data[5];
        update.param_data.value_type = cmd.data[6];
        //		printf("value_type:%hhu, %hhu\n", data.value_type, cmd.data[5]);
        //		printf("data: %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu \n", cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4],
        //				cmd.data[5], cmd.data[6], cmd.data[7]);
        this->GetParamValueFromCmd(&update.param_data, &cmd.data[7]);

        break;
    case AXIS_CONFIG:
        update.param_data.axis_index = cmd.data[1];
        memcpy(&update.param_data.param_no, &cmd.data[2], 4);
        active_type = cmd.data[6];
        update.param_data.value_type = cmd.data[7];
        this->GetParamValueFromCmd(&update.param_data, &cmd.data[8]);

        //	printf("update axis param, axis = %d\n", data.axis_index);
        break;
    }


    if(g_ptr_parm_manager->UpdateProcParam(&update, active_type))
        cmd.data[0] = SUCCEED;
    else
        cmd.data[0] = FAILED;

    cmd.data_len = 1;

    this->m_p_hmi_comm->SendCmd(cmd);
    //	printf("exit ProcessHmiSetProcParamCmd\n");
}

/**
 * @brief 处理HMI获取工艺相关参数的命令
 * @param cmd : HMI指令包
 */
void ChannelEngine::ProcessHmiGetProcParamCmd(HMICmdFrame &cmd){
    //	printf("enter ProcessHmiGetProcParamCmd\n");
    uint8_t index = 0;
    uint8_t axis = 0;
    switch(cmd.cmd_extension){
    case CHN_CONFIG:
        index = cmd.data[0];    //工艺参数组号
        //		printf("get chn proc config :chn = %hhu, proc_idx=%hhu\n", cmd.channel_index, index);
        if(cmd.channel_index >= m_p_general_config->max_chn_count || index >= kMaxProcParamCount)//通道索引超范围
            cmd.data_len = 0;
        else{
            cmd.data_len = sizeof(ProcessParamChn);
            memcpy(&cmd.data[1], &this->m_p_chn_proc_param[cmd.channel_index].chn_param[index], cmd.data_len);
            cmd.data_len += 1;  //数据长度加一，加上组号
        }
        break;
    case AXIS_CONFIG:
        index = cmd.data[0];
        axis = cmd.data[1];
        //		printf("get axis proc config: axis =  %hhu, idx = %hhu\n", axis, index);
        if(axis >= m_p_general_config->max_axis_count || index >= kMaxProcParamCount){//轴号超范围，支持参数修改后一次重启
            printf("get axis proc config failed, index=%hhu, axis_count:%hhu\n", axis, m_p_general_config->max_axis_count);
            cmd.data_len = 0;
        }
        else{
            cmd.data_len = sizeof(ProcessParamAxis);
            memcpy(cmd.data+2, &this->m_p_axis_proc_param[axis].axis_param[index], cmd.data_len);
            cmd.data_len += 2;   // 数据长度加上组号和轴号
            //		printf("get axis config succeed, data_len:%hu\n", cmd.data_len);
        }

        break;
    default:
        cmd.data_len = 0;
        break;
    }
    cmd.frame_number |= 0x8000;
    this->m_p_hmi_comm->SendCmd(cmd);	//发送响应

    //	printf("exit ProcessHmiGetProcParamCmd\n");
}

/**
 * @brief 处理HMI设置通道当前工艺组号的命令
 * @param cmd : HMI指令包
 */
void ChannelEngine::ProcessHmiSetCurProcIndex(HMICmdFrame &cmd){
    //	printf("enter ProcessHmiSetCurProcIndex\n");
    cmd.frame_number |= 0x8000;

    if(cmd.channel_index == CHANNEL_ENGINE_INDEX){
        bool res = true;
        for(uint8_t i = 0; i < m_p_general_config->chn_count; i++){
            if(!m_p_channel_control[i].SetCurProcParamIndex(cmd.data[0])){
                res = false;
                break;
            }
        }
        if(res)
            cmd.cmd_extension = SUCCEED;
        else
            cmd.cmd_extension = FAILED;
    }
    else if(cmd.channel_index >= this->m_p_general_config->chn_count){
        cmd.cmd_extension = FAILED;

        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "命令[%d]通道号非法！%d", cmd.cmd, cmd.channel_index);
    }
    else{
        if(m_p_channel_control[cmd.channel_index].SetCurProcParamIndex(cmd.data[0]))
            cmd.cmd_extension = SUCCEED;
        else
            cmd.cmd_extension = FAILED;
    }
    this->m_p_hmi_comm->SendCmd(cmd);
    //	printf("exit ProcessHmiSetCurProcIndex\n");
}

/**
 * @brief 处理HMI获取工艺参数组号的命令
 * @param cmd : HMI指令包
 */
void ChannelEngine::ProcessHmiGetCurProcIndex(HMICmdFrame &cmd){
    //	printf("enter ProcessHmiGetCurProcIndex\n");
    cmd.frame_number |= 0x8000;

    if(cmd.channel_index < this->m_p_general_config->chn_count){

        cmd.data[0] = m_p_channel_control[cmd.channel_index].GetCurProcParamIndex();
        cmd.data_len = 1;
    }
    else{
        cmd.data_len = 0;

        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "命令[%d]通道号非法！%d", cmd.cmd, cmd.channel_index);
    }
    this->m_p_hmi_comm->SendCmd(cmd);
    //	printf("exit ProcessHmiGetCurProcIndex\n");
}

/**
 * @brief 处理HMI清除消息命令
 * @param cmd : HMI指令包
 */
void ChannelEngine::ProcessHmiClearMsgCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;
    cmd.data_len = 0;



    if(cmd.cmd_extension == 0xFF){  //清空告警队列
        g_ptr_alarm_processor->Clear();
        cmd.cmd_extension = SUCCEED;
    }else if(cmd.cmd_extension == 0xEE){  //清除警告及提示信息
        g_ptr_alarm_processor->ClearWarning(CHANNEL_ENGINE_INDEX);
        cmd.cmd_extension = SUCCEED;
    }else{
        cmd.cmd_extension = FAILED;
    }

    this->m_p_hmi_comm->SendCmd(cmd);
}

void ChannelEngine::ProcessHmiGetErrorCmd(HMICmdFrame &cmd)
{
    cmd.frame_number |= 0x8000;
    cmd.data_len = 0;
    memset(cmd.data, 0x00, kMaxHmiDataLen);

    auto infolist = g_ptr_alarm_processor->GetErrorInfo();
    size_t infoSize = sizeof(ErrorInfo);
    for (auto itr = infolist.begin(); itr != infolist.end(); ++itr)
    {
        memcpy(cmd.data + cmd.data_len, &*itr, infoSize);
        int data_len = cmd.data_len + infoSize;
        if (data_len < kMaxHmiDataLen)
            cmd.data_len = data_len;
        else
            break;
    }

    std::cout << "Process Hmi cmd, err cnt: " << infolist.size() << std::endl;

    this->m_p_hmi_comm->SendCmd(cmd);
}


/**
 * @brief 处理HMI通知图形模式命令
 * @param cmd : HMI指令包
 */
void ChannelEngine::ProcessHmiNotifyGraphCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;

    if(cmd.channel_index < this->m_p_general_config->chn_count){
        this->m_p_channel_control[cmd.channel_index].SetHmiGraphMode(cmd.cmd_extension);
    }else if(cmd.channel_index == CHANNEL_ENGINE_INDEX){
        for(uint8_t chn = 0; chn < m_p_general_config->chn_count; chn++){
            this->m_p_channel_control[cmd.channel_index].SetHmiGraphMode(cmd.cmd_extension);
        }
    }

    this->m_p_hmi_comm->SendCmd(cmd);
}

void ChannelEngine::ProcessHmiHandWheelCmd(HMICmdFrame &cmd)
{
    cmd.frame_number |= 0x8000;

    HandWheelMapInfoVec infoVec = g_ptr_parm_manager->GetHandWheelVec();
    cmd.data[0] = 0;
    cmd.data_len = 0;

    size_t infoLen = sizeof(HandWheelMapInfo);
    for (auto itr = infoVec.begin(); itr != infoVec.end(); ++itr)
    {
        cmd.data[0 + cmd.data_len] = itr->devNum;
        cmd.data[1 + cmd.data_len] = itr->wheelID;
        cmd.data[2 + cmd.data_len] = itr->channelMap;
        cmd.data[3 + cmd.data_len] = itr->reserve;
        memcpy(&cmd.data[4 + cmd.data_len], itr->devName, sizeof(itr->devName));
        cmd.data_len += infoLen;
    }
    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief HMI向SC设置当前通道的所有工件坐标系
 * @param cmd : HMI指令包
 */
void ChannelEngine::ProcessHmiSetAllCoordCmd(HMICmdFrame &cmd)
{
    if(cmd.channel_index < this->m_p_general_config->chn_count)
    {
        cmd.frame_number |= 0x8000;
        double dVal;
        memcpy(&dVal, cmd.data, cmd.data_len);

        bool ret1, ret2;
        ret1 = m_p_channel_control[cmd.channel_index].UpdateAllCoord(dVal);
        ret2 = m_p_channel_control[cmd.channel_index].UpdateAllExCoord(dVal, m_p_channel_config[cmd.channel_index].ex_coord_count);
        m_p_channel_control[cmd.channel_index].SetMcCoord(true);
        if (ret1 && ret2)
            this->m_p_hmi_comm->SendCmd(cmd);
    }
    else
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "命令[%d]通道号非法！%d", cmd.cmd, cmd.channel_index);

}

void ChannelEngine::ProcessHmiAbsoluteRefSet(HMICmdFrame &cmd)
{
    cmd.frame_number |= 0x8000;

    uint8_t chn_axis = cmd.data[0];
    uint8_t phy_axis = 0;
    int errCode = 0;

    if (chn_axis != 0xFF)
        phy_axis = this->GetChnAxistoPhyAixs(m_n_cur_channle_index, chn_axis);

    HmiChannelStatus status;
    GetChnStatus(m_n_cur_channle_index, status);

    cmd.cmd_extension = 1;
    std::cout << "chn_axis: " << (int)chn_axis << " phy_axis: " << (int)phy_axis << std::endl;
    if (((phy_axis >= 0 && phy_axis < m_p_general_config->axis_count) || chn_axis == 0xFF)   //轴在合理范围
            && status.chn_work_mode == REF_MODE)                                             //回零模式
    {
        if (!m_b_emergency && !m_b_ret_ref)//不在急停和当前不处于回零动作中
        {
            if (chn_axis == 0xFF)//所有绝对式编码器回零
            {
                int index = 0;
                for(int i = 0; i < m_p_general_config->axis_count; ++i)
                {
                    if ((m_p_axis_config[i].absolute_ref_mode == 0 && m_p_axis_config[i].feedback_mode == 1)
                        || m_p_axis_config[i].axis_interface == 0)
                    {
                        if (GetSyncAxisCtrl()->CheckSyncState(i) != 2)
                        {//不为从动轴
                            m_n_mask_ret_ref |= (0x01<<i);   //设置需要回零的轴
                            m_p_axis_config[i].ret_ref_index = index++;
                        }
                    }
                }
                if (m_n_mask_ret_ref != 0)
                {
                    cmd.cmd_extension = 0;
                    m_n_ret_ref_auto_cur = 0;
                    m_b_ret_ref_auto = true;
                    m_b_ret_ref = true;     //开启回零
                }
            }
            else
            {
                if ((m_p_axis_config[phy_axis].absolute_ref_mode == 0 && m_p_axis_config[phy_axis].feedback_mode == 1)         //绝对式
                    || m_p_axis_config[phy_axis].axis_interface == 0)
                {
                    if (GetSyncAxisCtrl()->CheckSyncState(phy_axis) != 2)
                    {//不为从动轴
                        cmd.cmd_extension = 0;
                        m_b_ret_ref = true;
                        m_n_mask_ret_ref = (0x01<<phy_axis);
                    }
                    else
                    {
                        errCode = 1;
                    }
                }
            }
        }
    }

    if (cmd.cmd_extension)
    {
        if (errCode == 0)
            CreateError(ERR_RET_REF_FAILED, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, 0);
        else if(errCode == 1)
            CreateError(ERR_RET_SYNC_ERR, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, 0);
    }
    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 处理HMI轴移动指令
 * @param cmd : HMI指令包
 */
void ChannelEngine::ProcessHmiAxisMoveCmd(HMICmdFrame &cmd){
    cmd.frame_number |= 0x8000;  //设置回复标志
    uint8_t axis = 0;
    uint32_t speed = 0;
    int8_t dir = 0;
    double tar_pos = 0;

    if(cmd.data_len == 6){ //未指定目标位置
        axis = cmd.data[0];
        memcpy(&speed, &cmd.data[1], 4);
        dir = cmd.data[5];
        double vel = speed;

        if (GetPmcActive(axis)) {
            //if(speed > 0)
            //    this->ManualMovePmc(axis, 99999, vel, true);
            //else
            //    this->ManualMoveStop(axis);
            m_error_code = ERR_PMC_IVALID_USED;
            CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, 0xFF);
            return;
        }else{
            if(speed > 0)
                this->ManualMove(axis, dir, vel, 99999);
            else
                this->ManualMoveStop(axis);
        }

        printf("ChannelEngine::ProcessHmiAxisMoveCmd:axis=%hhu, vel=%lf, dir=%hhu\n", axis, vel, dir);

        cmd.cmd_extension = SUCCEED;

    }else if(cmd.data_len == 14){//指定目标位置
        axis = cmd.data[0];
        memcpy(&speed, &cmd.data[1], 4);
        dir = cmd.data[5];
        memcpy(&tar_pos, &cmd.data[6], sizeof(double));  //绝对目标位置
        double vel = speed;
        if(speed == 0)
            vel = this->m_p_axis_config[axis].rapid_speed;  //速度给0则使用轴参数中的定位速度

        if (GetPmcActive(axis)) {
            //this->ManualMovePmc(axis, tar_pos, vel, false);
            m_error_code = ERR_PMC_IVALID_USED;
            CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, 0xFF);
        }else{
            this->ManualMove(axis, dir, vel, tar_pos-this->GetPhyAxisMachPosFeedback(axis));
        }
        printf("ChannelEngine::ProcessHmiAxisMoveCmd:axis=%hhu, vel=%lf, tar=%lf\n", axis, vel, tar_pos);
        cmd.cmd_extension = SUCCEED;
    }else{//命令包格式错误
        cmd.cmd_extension = FAILED;
    }

    this->m_p_hmi_comm->SendCmd(cmd);
}


/**
 * @brief 从命令包中获取参数数据
 * @param data : 参数升级结构
 * @param src : 数据源
 */
void ChannelEngine::GetParamValueFromCmd(ParamUpdate *data, char *src){

    //	double ff = 66.0;
    //	char *pc = (char *)&ff;
	switch(data->value_type){
    case 0:
        memcpy(&data->value.value_uint8, src, 1);
        break;
    case 1:
        memcpy(&data->value.value_int8, src, 1);
        break;
    case 2:
        memcpy(&data->value.value_uint16, src, 2);
        break;
    case 3:
        memcpy(&data->value.value_int16, src, 2);
        break;
    case 4:
        memcpy(&data->value.value_uint32, src, 4);
        break;
    case 5:
        memcpy(&data->value.value_int32, src, 4);
        break;
    case 6:
        memcpy(&data->value.value_uint64, src, 8);
        break;
    case 7:
        memcpy(&data->value.value_int64, src, 8);
        break;
    case 8:
        memcpy(&data->value.value_double, src, 8);
        //		for(int i = 0; i < 8; i++)
        //			printf("double : %hhx | %hhx\n", src[i], pc[i]);
        //
        //		printf("get param set double value:%f, %f\n", data->value.value_double, ff);
        break;
    default:
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "参数设置指令，参数值类型非法：%hhu", data->value_type);
        break;
    }
}

/**
 * @brief 处理HMI设置参数指令
 * @param cmd : 指令
 */
void ChannelEngine::ProcessHmiSetParam(HMICmdFrame &cmd){
    //	uint32_t param_no = 0;	//参数号
    uint8_t active_type = ACTIVE_BY_POWEROFF;	//激活类型
    //	uint8_t value_type = VALUE_UINT8;		//值类型
    ParamUpdate data;
    data.param_type = cmd.cmd_extension;
    data.chn_index = cmd.channel_index;
    switch(cmd.cmd_extension){
    case SYS_CONFIG:
    case CHN_CONFIG:
        memcpy(&data.param_no, cmd.data, 4);
        memcpy(&active_type, &cmd.data[4], 1);
        memcpy(&data.value_type, &cmd.data[5], 1);
        //		printf("value_type:%hhu, %hhu\n", data.value_type, cmd.data[5]);
        //		printf("data: %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu \n", cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4],
        //				cmd.data[5], cmd.data[6], cmd.data[7]);
        this->GetParamValueFromCmd(&data, &cmd.data[6]);
        if(g_ptr_parm_manager->UpdateParameter(&data, active_type))
            cmd.data[0] = SUCCEED;
        else
            cmd.data[0] = FAILED;
        break;
    case AXIS_CONFIG:
        memcpy(&data.axis_index, cmd.data, 1);
        memcpy(&data.param_no, &cmd.data[1], 4);
        memcpy(&active_type, &cmd.data[5], 1);
        memcpy(&data.value_type, &cmd.data[6], 1);
        this->GetParamValueFromCmd(&data, &cmd.data[7]);
        if(g_ptr_parm_manager->UpdateParameter(&data, active_type))
            cmd.data[0] = SUCCEED;
        else
            cmd.data[1] = FAILED;
        //	printf("update axis param, axis = %d\n", data.axis_index);
        break;
    case TOOL_OFFSET_CONFIG:	//刀具偏置
        if(cmd.channel_index >= m_p_general_config->chn_count ||     //通道索引超范围
                (uint8_t)cmd.data[0] >= kMaxToolCount){
            cmd.data[0] = FAILED;
        }
        else{
            HmiToolOffsetConfig cfg;
            memcpy(&cfg, cmd.data+1, cmd.data_len-1);
            this->m_p_channel_control[cmd.channel_index].UpdateToolOffset(cmd.data[0], cfg);
            cmd.data[0] = SUCCEED;
        }
        break;
    case TOOL_POT_CONFIG:		//刀具位置    立即生效
        if(cmd.channel_index >= m_p_general_config->chn_count){//通道索引超范围
            cmd.data[0] = FAILED;
        }
        else{
            g_ptr_parm_manager->UpdateToolPotConfig(cmd.channel_index, *((SCToolPotConfig*)cmd.data));
            cmd.data[0] = SUCCEED;
        }
        break;
    case COORD_CONFIG:			//工件坐标系
        if(cmd.channel_index >= m_p_general_config->chn_count ||        //通道索引超范围
                (uint8_t)cmd.data[0] >= kWorkCoordCount){						//坐标索引超范围
            cmd.data[0] = FAILED;
            printf("update coord failed! chn = %hhu, coord=%hhu\n", cmd.channel_index, (uint8_t)cmd.data[0]);
        }
        else{
            HmiCoordConfig cfg;

            memcpy(&cfg, cmd.data+1, cmd.data_len-1);
            this->m_p_channel_control[cmd.channel_index].UpdateCoord(cmd.data[0], cfg);


            cmd.data[0] = SUCCEED;
            //			printf("update coord succeed! chn = %hhu, coord=%hhu, datalen=%hu\n", cmd.channel_index, (uint8_t)cmd.data[0], cmd.data_len);

        }
        break;
    case EX_COORD_CONFIG:		//扩展工件坐标系
        if(cmd.channel_index >= m_p_general_config->chn_count ||			//通道索引超范围
                (uint8_t)cmd.data[0] >= m_p_channel_config[cmd.channel_index].ex_coord_count){	//坐标索引超范围
            cmd.data[0] = FAILED;
        }
        else{
            HmiCoordConfig cfg;

            memcpy(&cfg, cmd.data+1, cmd.data_len-1);
            this->m_p_channel_control[cmd.channel_index].UpdateExCoord(cmd.data[0], cfg);
            cmd.data[0] = SUCCEED;

        }
        break;
    case PITCH_COMP_DATA:   //螺补数据
        if(this->UpdateHmiPitchCompData(cmd))
            cmd.data[0] = SUCCEED;
        else
            cmd.data[0] = FAILED;
        break;
#ifdef USES_FIVE_AXIS_FUNC
    case FIVE_AXIS_CONFIG:
        memcpy(&data.param_no, cmd.data, 4);
        memcpy(&active_type, &cmd.data[4], 1);
        memcpy(&data.value_type, &cmd.data[5], 1);
        printf("value_type:%hhu, %hhu, id=%u\n", data.value_type, cmd.data[5], data.param_no);
        //		printf("data: %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu \n", cmd.data[0], cmd.data[1], cmd.data[2], cmd.data[3], cmd.data[4],
        //				cmd.data[5], cmd.data[6], cmd.data[7]);
        this->GetParamValueFromCmd(&data, &cmd.data[6]);
        if(g_ptr_parm_manager->UpdateParameter(&data, active_type))
            cmd.data[0] = SUCCEED;
        else
            cmd.data[0] = FAILED;
        break;
#endif
    default:
        break;
    }

    cmd.data_len = 1;
    cmd.frame_number |= 0x8000;
    this->m_p_hmi_comm->SendCmd(cmd);	//发送响应
}

/**
 * @brief 处理HMI获取参数指令
 * @param cmd : 指令
 */
void ChannelEngine::ProcessHmiGetParam(HMICmdFrame &cmd){
    uint8_t index = 0;

    switch(cmd.cmd_extension){
    case SYS_CONFIG:
        cmd.data_len = sizeof(HmiSystemConfig);
        memcpy(cmd.data, m_p_general_config, cmd.data_len);
        break;
    case CHN_CONFIG:
        if(cmd.channel_index >= m_p_general_config->max_chn_count)//通道索引超范围, 支持参数修改后一次重启
            cmd.data_len = 0;
        else{
            cmd.data_len = sizeof(HmiChnConfig);
            memcpy(cmd.data, &m_p_channel_config[cmd.channel_index], cmd.data_len);
        }
        break;
    case AXIS_CONFIG:
        index = cmd.data[0];
        //printf("get axis config %hhu, frameindex=%hu\n", index, cmd.frame_number);
        if(index >= this->m_p_general_config->max_axis_count){//轴号超范围, 支持参数修改后一次重启
            printf("get axis config failed, index=%hhu, axis_count:%hhu\n", index, m_p_general_config->max_axis_count);
            cmd.data_len = 0;
        }
        else{
            cmd.data_len = sizeof(HmiAxisConfig);
            memcpy(cmd.data+1, &this->m_p_axis_config[index], cmd.data_len);
            cmd.data_len += 1;
            //printf("get axis config succeed, data_len:%hu\n", cmd.data_len);
        }

        break;
    case TOOL_OFFSET_CONFIG:{
        index = cmd.data[0];
        //		printf("get cmd TOOL_OFFSET_CONFIG, chn_index =  %hu, tool_index = %hhu\n", cmd.channel_index, index);

#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
        if((index != 0xFF && index >= kMaxToolCount) || cmd.channel_index >= m_p_general_config->chn_count){
            cmd.data_len = 0;
        }else if(index == 0xFF){   //读取基准刀数据
            cmd.data_len = sizeof(HmiToolOffsetConfig);
            HmiToolOffsetConfig cfg;
            SCToolOffsetConfig *pp = g_ptr_parm_manager->GetToolConfig(cmd.channel_index);
            cfg.geometry_compensation[0] = pp->geometry_comp_basic[0];
            cfg.geometry_compensation[1] = pp->geometry_comp_basic[1];
            cfg.geometry_compensation[2] = pp->geometry_comp_basic[2];
            cfg.geometry_wear = 0;
            cfg.radius_compensation = 0;
            cfg.radius_wear = 0;
            memcpy(cmd.data+1, &cfg, cmd.data_len);
            cmd.data_len += 1;
        }
#else
        if((index >= kMaxToolCount) || cmd.channel_index >= m_p_general_config->chn_count){
            cmd.data_len = 0;
        }
#endif
        else{
            cmd.data_len = sizeof(HmiToolOffsetConfig);
            HmiToolOffsetConfig cfg;
            SCToolOffsetConfig *pp = g_ptr_parm_manager->GetToolConfig(cmd.channel_index);
            cfg.geometry_compensation[0] = pp->geometry_compensation[index][0];
            cfg.geometry_compensation[1] = pp->geometry_compensation[index][1];
            cfg.geometry_compensation[2] = pp->geometry_compensation[index][2];
            cfg.geometry_wear = pp->geometry_wear[index];
            cfg.radius_compensation = pp->radius_compensation[index];
            cfg.radius_wear = pp->radius_wear[index];
            memcpy(cmd.data+1, &cfg, cmd.data_len);
            cmd.data_len += 1;

        }
        break;
    }
    case TOOL_POT_CONFIG:
        std::cout << "Get Pot Config" << std::endl;
        if(cmd.channel_index >= m_p_general_config->chn_count)//通道索引超范围
            cmd.data_len = 0;
        else{
            cmd.data_len = sizeof(HmiToolPotConfig);
            memcpy(cmd.data, g_ptr_parm_manager->GetToolPotConfig(cmd.channel_index), cmd.data_len);
        }
        break;
    case COORD_CONFIG:
        index = cmd.data[0];
        if(cmd.channel_index >= m_p_general_config->chn_count ||
                index >= kWorkCoordCount)//通道索引超范围
            cmd.data_len = 0;
        else{
            cmd.data_len = sizeof(HmiCoordConfig);
            memcpy(cmd.data+1, &g_ptr_parm_manager->GetCoordConfig(cmd.channel_index)[index], cmd.data_len);
            cmd.data_len += 1;
        }
        break;
    case EX_COORD_CONFIG:
        index = cmd.data[0];
        if(cmd.channel_index >= m_p_general_config->chn_count ||
                index >= m_p_channel_config[cmd.channel_index].ex_coord_count)//通道索引超范围
            cmd.data_len = 0;
        else{
            cmd.data_len = sizeof(HmiCoordConfig);
            memcpy(cmd.data+1, &g_ptr_parm_manager->GetExCoordConfig(cmd.channel_index)[index], cmd.data_len);
            cmd.data_len += 1;
        }
        break;

    case PITCH_COMP_DATA:  //螺补数据
        this->ProcessHmiGetPcDataCmd(cmd);
        break;
#ifdef USES_FIVE_AXIS_FUNC
    case FIVE_AXIS_CONFIG:
        if(cmd.channel_index >= m_p_general_config->max_chn_count)//通道索引超范围
            cmd.data_len = 0;
        else{
            cmd.data_len = sizeof(FiveAxisConfig);
            memcpy(cmd.data, &m_p_chn_5axis_config[cmd.channel_index], cmd.data_len);
        }
        break;
#endif
    default:
        cmd.data_len = 0;
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "收到HMI参数获取指令，参数类型非法[%d]", cmd.cmd_extension);
        break;
    }

    cmd.frame_number |= 0x8000;
    this->m_p_hmi_comm->SendCmd(cmd);	//发送响应
}


/**
 * @brief 处理HMI获取螺补数据指令
 * @param cmd : HMI发送的指令
 */
void ChannelEngine::ProcessHmiGetPcDataCmd(HMICmdFrame &cmd){
    if(cmd.data_len != 6){  //格式不匹配，返回失败
        printf("ProcessHmiGetPcDataCmd return, data_len=%hu\n", cmd.data_len);
        cmd.data_len = 0;
        return;
    }
    uint8_t axis_index = cmd.data[0];
    bool dir = (cmd.data[1]==0)?true:false;
    uint16_t offset = 0, count = 0;
    memcpy(&offset, &cmd.data[2], 2);
    memcpy(&count, &cmd.data[4], 2);

    //	printf("ProcessHmiGetPcDataCmd, axis_index=%hhu, dir=%hhu, offset=%hu, count=%hu\n", axis_index, dir, offset, count);

    if(axis_index == 0xFF){//不按轴顺序获取
        uint16_t tmp_count = count, tmp_start = offset-1;
        uint16_t tmp_end = tmp_start + count -1;
        uint16_t tc = 0;  //单次拷贝数量
        uint16_t tmp_copy = 0;  //已拷贝数量
        int dc = sizeof(double);   //double类型字节大小

        double *pp = nullptr;
        ListNode<AxisPcDataAlloc> *node = this->m_list_pc_alloc.HeadNode();
        while(node != nullptr && tmp_count > 0){
            pp = m_p_pc_table->pc_table[node->data.axis_index];
            if(tmp_start < node->data.start_index){
                if(tmp_end < node->data.start_index){
                    tc = tmp_count;
                }else{
                    tc = node->data.start_index-tmp_start;
                }

                memset(&cmd.data[6+tmp_copy*dc], 0, dc*tc);
                tmp_count -= tc;
                tmp_copy += tc;
                tmp_start += tc;

                if(tmp_count > 0){//还需要拷贝数据
                    if(tmp_end <= node->data.end_index){
                        tc = tmp_count;
                    }else{
                        tc =node->data.end_index-tmp_start+1;
                    }
                    memcpy(&cmd.data[6+tmp_copy*dc], &pp[tmp_start-node->data.start_index], dc*tc);
                    tmp_count -= tc;
                    tmp_copy += tc;
                    tmp_start += tc;
                }

            }else if(node->data.start_index <= tmp_start && node->data.end_index >= tmp_start){
                if(tmp_end <= node->data.end_index){
                    tc = tmp_count;
                }else{
                    tc =node->data.end_index-tmp_start+1;
                }
                memcpy(&cmd.data[6+tmp_copy*dc], &pp[tmp_start-node->data.start_index], dc*tc);
                tmp_count -= tc;
                tmp_copy += tc;
                tmp_start += tc;
            }

            node = node->next;
        }

        if(tmp_count > 0){
            memset(&cmd.data[6+tmp_copy*dc], 0, dc*tmp_count);
        }

    }else{
        if(axis_index >= this->m_p_general_config->axis_count ||              //轴序号不对
                offset+count-1 > m_p_axis_config[axis_index].pc_count){  //获取的数量不对
            cmd.data_len = 0;
            printf("ProcessHmiGetPcDataCmd return, axis_index=%hhu, offset=%hu, count=%hu\n", axis_index, offset, count);
            return;
        }

        double *pp = m_p_pc_table->pc_table[axis_index];
        if(dir)  //正向螺补
            memcpy(&cmd.data[6], &pp[offset-1], sizeof(double)*count);
        else if(this->m_p_axis_config[axis_index].pc_type == 1)  //负向螺补
            memcpy(&cmd.data[6], &pp[m_p_axis_config[axis_index].pc_count+offset-1], sizeof(double)*count);
        else{ //单向螺补模式，获取负向螺补失败
            cmd.data_len = 0;
            printf("ProcessHmiGetPcDataCmd return, dir=%hhu\n", dir);
            return;
        }

        //	printf("read axis %hhu pc data, pos = %lf, neg=%lf\n", axis_index, pp[199], pp[399]);

    }

    cmd.data_len += sizeof(double)*count;
    return;
}

/**
 * @brief 处理螺补导入数据
 * @return true--成功   false--失败
 */
bool ChannelEngine::ProcessPcDataImport(){
    bool res = true;

    int fd = -1;                    //文件句柄

    struct stat statbuf;
    int file_size = 0;
    uint8_t phy_axis = 0;   //物理轴号
    uint8_t pc_type = 0;    //螺补类型
    uint16_t point_count = 0;  //螺补点数
    double inter_dis = 0;   //螺补间隔  单位：mm
    double *data = nullptr;

    if(stat(PATH_PC_DATA_TMP, &statbuf) == 0)
        file_size = statbuf.st_size;  //获取文件总大小
    else{
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "获取文件[%s]大小失败！", PATH_PC_DATA_TMP);
        CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 1);   //获取文件大小失败
        return false;
    }

    if(file_size < 12){ //螺补导入文件头有12字节固定数据
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "文件[%s]大小[%d字节]不匹配！", PATH_PC_DATA_TMP, file_size);
        CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 2);   //获取文件大小失败
        return false;
    }


    fd = open(PATH_PC_DATA_TMP, O_RDONLY);

    if(fd == -1){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "打开文件失败[%s]", PATH_PC_DATA_TMP);
        CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 3);   //打开文件失败
        return false;
    }


    read(fd, &phy_axis, 1);   //读取物理轴编号
    read(fd, &pc_type, 1);    //读取螺补类型，0--单向  1--双向  螺补文件读出
    read(fd, &point_count, 2);  //读取螺补点数
    read(fd, &inter_dis, 8);    //读取补偿间隔
    /***********************************************************************/
    if(this->m_p_axis_config[phy_axis].pc_type != pc_type){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "螺补类型与设置不符[%hhu : %hhu]！", m_p_axis_config[phy_axis].pc_type, pc_type);
        CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 4);   //螺补类型不匹配
        close(fd);
        return false;
    }

    if(fabs(m_p_axis_config[phy_axis].pc_inter_dist) != fabs(inter_dis)){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "螺补测量间隔与设置不符[%lf : %lf]！", fabs(m_p_axis_config[phy_axis].pc_inter_dist), fabs(inter_dis));
        CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 5);   //螺补间隔不匹配
        close(fd);
        return false;
    }

    if(this->m_p_axis_config[phy_axis].pc_count != point_count){
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_ENGINE_SC, "螺补点数与设置不符[%hu : %hu]！", m_p_axis_config[phy_axis].pc_count, point_count);
        CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 6);   //螺补间隔不匹配
        close(fd);
        return false;
    }


    if(pc_type == 1){//双向螺补
        point_count *= 2;   //双向螺补则数据量为点数乘2
    }

    if((file_size-12) != (int)(point_count*sizeof(double))){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "螺补数据大小[%d字节]与补偿点数[%hhu]不匹配！", file_size-12, point_count);
        CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 7);   //获取文件大小失败
        close(fd);
        return false;
    }

    bool dir_pos = true;  //初始螺补方向为正向，默认正向
    if(pc_type == 1 && inter_dis < 0)
        dir_pos = false;

    data = new double[point_count];
    if(data == nullptr){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "螺补数据导入，分配缓冲失败！");
        CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 8);   //分配缓冲失败
        close(fd);
        return false;
    }

    //读取数据
    read(fd, data, sizeof(double)*point_count);

    close(fd);

    //写入数据
    if(pc_type == 0){  //单向螺补
        res = g_ptr_parm_manager->UpdatePcData(phy_axis, dir_pos, 1, point_count, data);
    }else if(pc_type == 1){  //双向螺补
        int pcs = point_count/2;
        res = g_ptr_parm_manager->UpdatePcData(phy_axis, dir_pos, 1, pcs, data);
        if(res){
            int pcs_half = pcs/2;
            double tb = 0;
            for(int i =0; i < pcs_half; i++){  //调换顺序
                tb = data[point_count-1-i];
                data[point_count-1-i] = data[pcs+i];
                data[pcs+i] = tb;
            }
            res = g_ptr_parm_manager->UpdatePcData(phy_axis, !dir_pos, 1, pcs, data+pcs);
        }
    }

    delete []data;  //释放缓冲

    if(!res){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "螺补数据导入，分配缓冲失败！");
        CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 8);   //分配缓冲失败
        close(fd);
        return false;
    }

    //	if(res == -1 || res != read_size){  //read失败
    //		printf("read pmc file failed, errno = %d, block = %d\n", errno, block_total);
    //		close(fp);
    //		return 0;
    //	}

    this->NotifyHmiPitchCompDataChanged();  //通知HMI重新获取螺补数据

    //更新数据给MC
    //发送轴螺补数据表
    this->SendMiPcData(phy_axis);

    //发送轴螺补设置参数
    this->SendMiPcParam(phy_axis);
    this->SendMiPcParam2(phy_axis);

    g_ptr_trace->PrintLog(LOG_CONFIG_MODIFY, "第%hhu轴螺补数据导入成功！", phy_axis+1);
    return true;
}

#ifdef USES_WOOD_MACHINE
/**
 * @brief 保存刀具信息数据
 */
void ChannelEngine::SaveToolInfo(){
    bool flag = false;
    for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
        if(this->m_p_channel_control[i].IsToolLifeChanged()){
            flag = true;
            g_ptr_parm_manager->UpdateToolPotConfig(i, false);
        }
    }
    if(flag)
        g_ptr_parm_manager->SaveParm(TOOL_POT_CONFIG);
}
#endif

void ChannelEngine::SetProgProtect(bool flag)
{
    HMICmdFrame hmi_cmd;
    hmi_cmd.channel_index = CHANNEL_ENGINE_INDEX;
    hmi_cmd.cmd = CMD_SC_NOTIFY_PROTECT_STATUS;
    hmi_cmd.cmd_extension = 0;
    hmi_cmd.data_len = sizeof(flag);
    memcpy(&hmi_cmd.data[0], &flag, hmi_cmd.data_len);

    std::cout << "SetProgProtect: " << flag << std::endl;
    m_p_hmi_comm->SendCmd(hmi_cmd);
}

bool ChannelEngine::UpdateMcModel(const string &mcPath)
{
    std::cout << "ChannelEngine::UpdateMcModel" << std::endl;
    if(access(mcPath.c_str(), F_OK) == -1)	//升级文件不存在，MC模块不需要升级
    {
        std::cout << "file not exist " << mcPath << std::endl;
        return false;
    }

    std::cout << "step1: SendMcUpdateStartCmd()" << std::endl;
    //通知MC开始升级
    if(!this->SendMcUpdateStartCmd()){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "发送MC模块升级开始命令失败！");
        return false;
    }

    std::cout << "step2: clear flash" << std::endl;
    //等待MC擦除备份区flash
    do{
        if(!QueryMcUpdateStatus()){
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "查询MC模块升级状态失败！");
            return false;
        }
        while(!m_b_get_mc_update_status)
            usleep(100);

        if(m_n_mc_update_status == 4)
            break;

        usleep(100000);

    }while(1);

    std::cout << "step3: get file size" << std::endl;
    //通知MC升级文件的大小
    struct stat statbuf;
    uint32_t file_size = 0;
    if(stat(mcPath.c_str(), &statbuf) == 0)
        file_size = statbuf.st_size;
    else{
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "获取文件[%s]大小失败！", PATH_PMC_DATA);
        return false;
    }
    if(!this->SendMcUpdateFileSize(file_size)){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "发送升级文件大小失败！");
        return false;
    }

    std::cout << "step4: open file" << std::endl;
    //打开MC文件
    int fp = 0;
    fp = open(mcPath.c_str(), O_RDONLY);
    if(fp < 0){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "打开文件[%s]失败！", mcPath.c_str());
        return false;
    }

    //计算文件总帧数
    uint16_t file_frame_count = 0;
    file_frame_count = file_size/MC_UPDATE_BLOCK_SIZE;
    if(file_size % MC_UPDATE_BLOCK_SIZE)
        file_frame_count++;

    std::cout << "step5: write file" << std::endl;
    //写入文件
    GCodeFrame data_frame;
    uint32_t send_size = 0, read_size = 0;
    uint32_t cc = 0, i = 0;
    uint16_t file_crc = 0xffff;
    while(file_size > 0){
        bzero(data_frame.data_crc, sizeof(data_frame));
        send_size = file_size>MC_UPDATE_BLOCK_SIZE?MC_UPDATE_BLOCK_SIZE:file_size;
        read_size = read(fp, (void *)&data_frame.data_crc[1], send_size);
        if(read_size != send_size){
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "读取文件[%s]失败，%u, %u！", mcPath.c_str(), send_size, read_size);
            goto END;//文件读取失败
        }

        cc = send_size/2;
        if(send_size%2)
            cc++;
        for(i = 0; i < cc; i++){
            file_crc ^= data_frame.data_crc[i+1];  //计算CRC
        }

        while(!this->m_p_mc_comm->WriteGCodeData(0, data_frame)){  //固定用通道0数据通道更新MC
            usleep(100);
        }
        file_size -= send_size;
    }

    //等待文件写入完成
    while(this->m_p_mc_comm->ReadGCodeFifoCount(0) > 0)
        usleep(10000);
    std::cout << "step6, write finish" << std::endl;

    //发送升级文件CRC
    printf("send crc, block = %hu, crc = 0x%hx\n", file_frame_count, file_crc);
    this->SendMcUpdateFileCrc(file_frame_count, file_crc);
    std::cout << "step7, send crc" << std::endl;

    //查询MC状态，等待MC升级接收完成
    //printf("wait update result!\n");
    do{
        if(!QueryMcUpdateStatus()){
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "查询MC模块升级状态失败！");
            goto END;
        }
        while(!m_b_get_mc_update_status)
            usleep(100); //等待查询结果

        if(m_n_mc_update_status == 0x02){ //报错
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MC升级失败！");
            goto END;
        }
        else if(m_n_mc_update_status == 0x13){ //CRC校验错
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MC升级CRC校验失败！");
            goto END;
        }
        else if(m_n_mc_update_status == 0x11){//升级成功
            printf("Succeed to update the MC module!\n");
            break;
        }else
            usleep(100000);  //等待100ms

    }while(1);
    std::cout << "step8, finish" << std::endl;

    if(fp > 0)
        close(fp);
    return true;

END:
    if(fp > 0)
        close(fp);

    return false;

}

bool ChannelEngine::CheckSoftLimit(ManualMoveDir dir, uint8_t phy_axis, double pos){
    // 如果没回零，认为限位不超限
    if(!(this->m_n_mask_ret_ref_over & (0x01<<phy_axis)))
        return false;
    double positive[3];
    double negative[3];
    bool enable[3];
    positive[0] = m_p_axis_config[phy_axis].soft_limit_max_1;
    positive[1] = m_p_axis_config[phy_axis].soft_limit_max_2;
    positive[2] = m_p_axis_config[phy_axis].soft_limit_max_3;
    negative[0] = m_p_axis_config[phy_axis].soft_limit_min_1;
    negative[1] = m_p_axis_config[phy_axis].soft_limit_min_2;
    negative[2] = m_p_axis_config[phy_axis].soft_limit_min_3;
    enable[0] = m_p_axis_config[phy_axis].soft_limit_check_1;
    enable[1] = m_p_axis_config[phy_axis].soft_limit_check_2;
    enable[2] = m_p_axis_config[phy_axis].soft_limit_check_3;
    uint8_t chn_axis = 0, chn = 0;
    chn = this->GetAxisChannel(phy_axis, chn_axis);
    for(int i=0; i<3; i++){
        if(dir == DIR_POSITIVE && enable[i] && positive[i] <= pos){
            CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, chn, chn_axis);
            return true;

        }
        if(dir == DIR_NEGATIVE && enable[i] && negative[i] >= pos){
            CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, chn, chn_axis);
            return true;
        }
    }
    return false;
}

bool ChannelEngine::GetSoftLimt(ManualMoveDir dir, uint8_t phy_axis, double &limit){
    // 如果没回零，认为限位不超限
    if(!(this->m_n_mask_ret_ref_over & (0x01<<phy_axis)))
        return false;
    double positive[3];
    double negative[3];
    bool enable[3];
    positive[0] = m_p_axis_config[phy_axis].soft_limit_max_1;
    positive[1] = m_p_axis_config[phy_axis].soft_limit_max_2;
    positive[2] = m_p_axis_config[phy_axis].soft_limit_max_3;
    negative[0] = m_p_axis_config[phy_axis].soft_limit_min_1;
    negative[1] = m_p_axis_config[phy_axis].soft_limit_min_2;
    negative[2] = m_p_axis_config[phy_axis].soft_limit_min_3;
    enable[0] = m_p_axis_config[phy_axis].soft_limit_check_1;
    enable[1] = m_p_axis_config[phy_axis].soft_limit_check_2;
    enable[2] = m_p_axis_config[phy_axis].soft_limit_check_3;
    uint8_t chn_axis = 0, chn = 0;
    chn = this->GetAxisChannel(phy_axis, chn_axis);
    bool flag = false; // 是否找到了限位位置
    for(int i=0; i<2; i++){//第三限位去掉
        if(dir == DIR_POSITIVE && enable[i] &&
                (!flag || (positive[i] < limit))){
            flag = true;
            limit = positive[i];
        }
        if(dir == DIR_NEGATIVE && enable[i] &&
                (!flag || (negative[i] > limit))){
            flag = true;
            limit = negative[i];
        }
    }
    return flag;
}

/**
 * @brief 处理HMI更新螺补数据
 * @param cmd : HMI发送的数据更新包
 * @return true--执行成功   false--执行失败
 */
bool ChannelEngine::UpdateHmiPitchCompData(HMICmdFrame &cmd){
    uint8_t axis_index = cmd.data[0];
    bool dir = (cmd.data[1]==0)?true:false;   //正向螺补标志
    uint16_t offset, count;
    memcpy(&offset, &cmd.data[2], 2);
    memcpy(&count, &cmd.data[4], 2);

    if(cmd.data_len != 6+sizeof(double)*count ){
        printf("UpdateHmiPitchCompData: ERROR in data format, datalen=%hu, count=%hu\n", cmd.data_len, count);
        return false;
    }

    double *data = (double *)&cmd.data[6];

    if(axis_index == 0xFF){//不按轴顺序设置
        uint16_t tmp_start = offset-1;

        ListNode<AxisPcDataAlloc> *node = this->m_list_pc_alloc.HeadNode();
        while(node != nullptr){

            //printf("node->data.start_index : %d node->data.end_index: %d tmp_start: %d \n",
            //    node->data.start_index, node->data.end_index, tmp_start);

            if(node->data.start_index <= tmp_start && node->data.end_index >= tmp_start){
                if((tmp_start - node->data.start_index) < this->m_p_axis_config[node->data.axis_index].pc_count)
                    dir = true;
                else
                    dir = false;
                break;
            }
            node = node->next;
        }

        if(node == nullptr){
            printf("UpdateHmiPitchCompData::failed to find the axis node!\n");
            return false;
        }

        uint16_t offset_tmp = tmp_start - node->data.start_index+1;

        printf("tmp start %d,   node_start_index %d offset_tmp %d\n", tmp_start, node->data.start_index, offset_tmp);

        if(!g_ptr_parm_manager->UpdatePcData(node->data.axis_index, dir, offset_tmp, count, data)){
            printf("UpdateHmiPitchCompData: failed to update pc data1\n");
            return false;
        }
        axis_index = node->data.axis_index;

    }else{//轴顺序设置
        if(!g_ptr_parm_manager->UpdatePcData(axis_index, dir, offset, count, data)){
            printf("UpdateHmiPitchCompData: failed to update pc data2\n");
            return false;
        }
    }

     printf("UpdateHmiPitchCompData::axis=%hhu, dir=%hhu, offset=%hu, count=%hu\n", axis_index, dir, offset, count);

    //更新数据给MC
    //发送轴螺补数据表
    this->SendMiPcData(axis_index);

    //发送轴螺补设置参数
    this->SendMiPcParam(axis_index);
    this->SendMiPcParam2(axis_index);

    return true;
}

/**
 * @brief 处理HMI获取PMC寄存器值得指令
 * @param cmd
 */
void ChannelEngine::ProcessHmiGetPmcReg(HMICmdFrame &cmd){
    uint16_t reg_sec = 0, reg_index = 0;
    uint16_t reg_count = 0;
    uint16_t *reg_value16 = nullptr;
    uint8_t *reg_value8 = nullptr;


    memcpy(&reg_sec, cmd.data, 2);   //寄存器段
    memcpy(&reg_index, &cmd.data[2], 2);   	//寄存器段内索引号
    if(cmd.data_len == 6)
        memcpy(&reg_count, &cmd.data[4], 2);   //寄存器地址个数
    else
        reg_count = 1;   //兼容老版本协议

    //	printf("read pmc reg, sec = %hu, index= %hu, count = %hu\n", reg_sec, reg_index, reg_count);
    cmd.cmd_extension = SUCCEED;
    switch(reg_sec){
    case PMC_REG_X:
    case PMC_REG_Y:
    case PMC_REG_F:
    case PMC_REG_G:
    case PMC_REG_R:
    case PMC_REG_K:
    case PMC_REG_A:
#ifdef USES_PMC_2_0
    case PMC_REG_D:
    case PMC_REG_C:
    case PMC_REG_T:
    case PMC_REG_E:
#endif
        reg_value8 = new uint8_t[reg_count];
        if(reg_value8 == nullptr || !this->m_p_pmc_reg->GetRegValueMulti(static_cast<PmcRegSection>(reg_sec), reg_index, reg_count, reg_value8))
            cmd.cmd_extension = FAILED;
        else{
            if(cmd.data_len == 4){
                cmd.data_len = 5;
                memcpy(&cmd.data[4], reg_value8, reg_count);
                //				if(reg_sec == PMC_REG_K)
                //					printf("read kreg, index= %hu, value = 0x%hhx\n", reg_index, reg_value8[0]);
            }
            else{
                cmd.data_len = 0x06+reg_count;
                memcpy(&cmd.data[6], reg_value8, reg_count);
            }
            //			printf("get pmc reg: sec = %hu, index = %hu, value = 0x%hhx\n", reg_sec, reg_index, reg_value8[0]);
        }
        break;

#ifndef USES_PMC_2_0
    case PMC_REG_D:
    case PMC_REG_C:
    case PMC_REG_T:
    case PMC_REG_DC:
    case PMC_REG_DT:
        reg_value16 = new uint16_t[reg_count];
        if(reg_value16 == nullptr || !this->m_p_pmc_reg->GetRegValueMulti(static_cast<PmcRegSection>(reg_sec), reg_index, reg_count, reg_value16)){
            cmd.cmd_extension = FAILED;
            printf("failed to get pmc value\n");
        }else{
            if(cmd.data_len == 4){
                cmd.data_len = 6;
                memcpy(&cmd.data[4], reg_value16, 2*reg_count);
            }
            else{
                cmd.data_len = 0x06+reg_count*2;
                memcpy(&cmd.data[6], reg_value16, 2*reg_count);
            }
            //	printf("get pmc reg: sec = %hu, index = %hu, value = %hu\n", reg_sec, reg_index, reg_value16);
        }
        break;
#endif

    default:
        cmd.cmd_extension = FAILED;  //失败
        break;
    }

    if(reg_value8 )
        delete []reg_value8;
    if(reg_value16)
        delete []reg_value16;
    cmd.frame_number |= 0x8000;

    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 处理HMI请求PMC的UUID指令
 * @param cmd
 */
void ChannelEngine::ProcessHmiGetPmcUuid(HMICmdFrame &cmd){
    char path[kMaxPathLen];
    char uuid[20];
    strcpy(path, PATH_PMC_DATA);
    memset(uuid, 0x00, 20);

    if(access(path, F_OK) == -1){	//文件不存在
        cmd.cmd_extension = FAILED;
        printf("#####ProcessHmiGetPmcUuid:failed!\n");
    }else{
        cmd.cmd_extension = SUCCEED;

        FILE *file_src = fopen(path, "r+");
        if (nullptr == file_src)
        {
            cmd.cmd_extension = FAILED;
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMC执行文件打开失败！errno = %d", errno);
        }else{
            int read = fread(uuid, 1, 16, file_src);
            if(read != 16){
                cmd.cmd_extension = FAILED;
                g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMC执行文件读取UUID失败！errno = %d", errno);
            }else{
                memcpy(cmd.data, uuid, 16);
                cmd.data_len = 16;
            }
            fclose(file_src);
        }

    }


    cmd.frame_number |= 0x8000;

    this->m_p_hmi_comm->SendCmd(cmd);

}

/**
 * @brief 处理HMI设置PMC寄存器值得指令
 * @param cmd
 */
void ChannelEngine::ProcessHmiSetPmcReg(HMICmdFrame &cmd){

    uint16_t reg_sec = 0, reg_index = 0;
    uint8_t reg_value8 = 0;
    uint8_t bit_index = 0;
    uint8_t bit_count = 0;
    uint32_t bit_value32 = 0;
    bool bit_opt = cmd.cmd_extension==1?true:false;  //是否位操作

#ifndef USES_PMC_2_0
    uint16_t reg_value16 = 0;
#endif

    memcpy(&reg_sec, cmd.data, 2);   		//寄存器段
    memcpy(&reg_index, &cmd.data[2], 2);   	//寄存器段内索引号

    cmd.cmd_extension = SUCCEED;
    switch(reg_sec){
    case PMC_REG_X:
    case PMC_REG_Y:
    case PMC_REG_F:
    case PMC_REG_G:
    case PMC_REG_R:
    case PMC_REG_K:
    case PMC_REG_A:
#ifdef USES_PMC_2_0
    case PMC_REG_D:
    case PMC_REG_C:
    case PMC_REG_T:
    case PMC_REG_E:
#endif
        if(!bit_opt){
            memcpy(&reg_value8, &cmd.data[4], 1);
            if(!this->m_p_pmc_reg->SetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, reg_value8)){
                printf("failed to set pmc register\n");
                cmd.cmd_extension = FAILED;
            }
            printf("set pmc reg: sec = %hu, index = %hu, value = %hhu\n", reg_sec, reg_index, reg_value8);
        }else{
            bit_index = cmd.data[4];
            bit_count = cmd.data[5];
            memcpy(&bit_value32, &cmd.data[6], 4);

            if(!this->m_p_pmc_reg->SetRegBitValue(static_cast<PmcRegSection>(reg_sec), reg_index, bit_index, bit_count, bit_value32)){
                printf("failed to set pmc register bit value\n");
                cmd.cmd_extension = FAILED;
            }
            printf("set pmc reg bit: sec = %hu, index = %hu, bit = %hhu, count = %hhu, value = %u\n", reg_sec, reg_index, bit_index, bit_count, bit_value32);

            // @test zk
            if(reg_sec == 2 and reg_index == 82 and bit_index == 1 and bit_value32 == 1){
                printf("冷却！！！\n");
                this->m_p_channel_control[0].test();
            }
            // @test zk
        }
        break;
#ifndef USES_PMC_2_0
    case PMC_REG_D:
    case PMC_REG_C:
    case PMC_REG_T:
    case PMC_REG_DC:
    case PMC_REG_DT:
        if(!bit_opt){
            memcpy(&reg_value16, &cmd.data[4], 2);
            if(!this->m_p_pmc_reg->SetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, reg_value16))
                cmd.cmd_extension = FAILED;

            printf("set pmc reg: sec = %hu, index = %hu, value = 0x%hx, data[4] = 0x%hhx, data[5] = 0x%hhx\n", reg_sec, reg_index, reg_value16,
                   cmd.data[4], cmd.data[5]);
        }else{
            bit_index = cmd.data[4];
            bit_count = cmd.data[5];
            memcpy(&bit_value32, &cmd.data[6], 4);

            if(!this->m_p_pmc_reg->SetRegBitValue(static_cast<PmcRegSection>(reg_sec), reg_index, bit_index, bit_count, bit_value32)){
                printf("failed to set pmc register bit value\n");
                cmd.cmd_extension = FAILED;
            }
            printf("set pmc2.0 reg bit: sec = %hu, index = %hu, bit = %hhu, count = %hhu, value = %u\n", reg_sec, reg_index, bit_index, bit_count, bit_value32);
        }

        break;
#endif
    default:
        cmd.cmd_extension = FAILED;  //失败
        break;
    }

    cmd.frame_number |= 0x8000;

    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 处理HMI升级请求
 * @param cmd :  命令数据包
 */
void ChannelEngine::ProcessHmiUpdateReq(HMICmdFrame &cmd){
    bool flag = true;//能否升级

    if(this->m_n_update_state == MODULE_UPDATE_NONE){
        HmiChannelStatus chn_status;

        //轮询各通道当前加工状态
        for(int i = 0; i < this->m_p_general_config->chn_count; i++){
            this->m_p_channel_control[i].GetChnStatus(chn_status);
            if(chn_status.machining_state != MS_READY &&
                    chn_status.machining_state != MS_PAUSED &&
                    chn_status.machining_state != MS_WARNING){
                flag = false;
                break;
            }
        }
    }

    //清空update目录下的文件
    //组合升级文件名字
    char filepath[kMaxPathLen] = {0};
    strcpy(filepath, PATH_UPDATE_PATH);
    strcat(filepath, "*.*");
    if( 0 != remove(filepath)){
        //清空所有文件失败
        g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_ENGINE_SC, "模块升级前清空升级文件夹失败！errno = %d", errno);
    }

    cmd.frame_number |= 0x8000;
    cmd.data_len = 1;
    cmd.data[0] = flag?APPROVE:REFUSE;
    this->m_p_hmi_comm->SendCmd(cmd);	//发送响应
}

/**
 * @brief 处理PMC轴确认参考点指令
 * @param phy_axis : 物理轴号，从0开始
 */
void ChannelEngine::ProcessPmcAxisFindRef(uint8_t phy_axis){
    if(m_p_axis_config[phy_axis].axis_interface != VIRTUAL_AXIS       //非虚拟轴
            && m_p_axis_config[phy_axis].axis_type != AXIS_SPINDLE)   //非主轴
        {	//非禁止回参考点

#ifdef USES_PMC_PROCESS
        if(phy_axis == 5)
            this->m_p_pmc_reg->FReg().bits[0].OUT8 = 1;  //pmc X轴回零
        else if(phy_axis == 6)
            this->m_p_pmc_reg->FReg().bits[0].OUT9 = 1;  //pmc Y轴回零
        else if(phy_axis == 7)
            this->m_p_pmc_reg->FReg().bits[0].OUT10 = 1;  //pmc 左Z轴回零
        else if(phy_axis == 8)
            this->m_p_pmc_reg->FReg().bits[0].OUT11 = 1;  //pmc 右Z轴回零
#else
        //系统处理
        this->m_n_mask_ret_ref |= (0x01<<phy_axis);
        this->m_b_ret_ref = true;
        this->m_b_ret_ref_auto = false;
        ScPrintf("ProcessPmcAxisFindRef axis=%u",phy_axis);

#endif
    }else{
        if (GetPmcActive(phy_axis)) {
            this->m_pmc_axis_ctrl[m_p_axis_config[phy_axis].axis_pmc-1].ExecCmdOver(true);
        }
    }
}

/**
 * @brief 处理HMI回参考点指令，适用于增量式编码器
 * @param cmd ： HMI指令
 */
void ChannelEngine::ProcessHmiFindRefCmd(HMICmdFrame &cmd){
    printf("ChannelEngine::ProcessHmiFindRefCmd, ext = 0x%hx, chn=0x%hx\n", cmd.cmd_extension, cmd.channel_index);
    if(this->m_b_ret_ref){ //已经在回参考点
        printf("已处于回参考点流程中,拒绝！ret_ref_mask = 0x%llx\n", this->m_n_mask_ret_ref);
        cmd.data[0] = FAILED;

    }else if(g_ptr_alarm_processor->HasErrorInfo()){ //有告警，拒绝回零
        printf("有告警，拒绝回零！\n");
        cmd.data[0] = FAILED;
    }else if(cmd.cmd_extension == 0x00){//当前轴回零
        if(this->m_n_cur_pmc_axis != 0xFF){
            this->ProcessPmcAxisFindRef(m_n_cur_pmc_axis);
        }else{
            this->m_p_channel_control[this->m_n_cur_channle_index].ProcessHmiReturnRefCmd(false);
            this->m_b_ret_ref = true;
            this->m_b_ret_ref_auto = false;
        }


    }else if(cmd.cmd_extension == 0x10){ //通道轴回零
        this->m_p_channel_control[cmd.channel_index].ProcessHmiReturnRefCmd(true);
        this->m_b_ret_ref = true;
        this->m_b_ret_ref_auto = true;

    }else if(cmd.cmd_extension == 0xFF){  //所有轴回零
        for(int i = 0; i < this->m_p_general_config->axis_count; i++){
            if(m_p_axis_config[i].axis_interface != VIRTUAL_AXIS		//非虚拟轴
                && m_p_axis_config[i].axis_type != AXIS_SPINDLE)		//非主轴
            {	 //回参考点方式非禁止

                this->m_n_mask_ret_ref |= (0x01<<i);
            }
        }
        m_n_ret_ref_auto_cur = 0;
        this->m_b_ret_ref = true;
        this->m_b_ret_ref_auto = true;
    }else{//非法
        printf("非法的cmd_ext=%hu, 拒绝！\n", cmd.cmd_extension);
        cmd.data[0] = FAILED;
    }

    cmd.frame_number |= 0x8000;
    cmd.data_len = 1;
    this->m_p_hmi_comm->SendCmd(cmd);

}

/**
 * @brief 处理PMC指令的物理轴回零
 * @param phy_axis : 物理轴序号，从0开始
 */
void ChannelEngine::ProcessPmcRefRet(uint8_t phy_axis){
    printf("ChannelEngine::ProcessPmcRefRet, phy_axis=0x%hhx\n", phy_axis);
    if(phy_axis >= this->m_p_general_config->axis_count){
        printf("phy_axis %hhu over count %hhu, return\n", phy_axis, m_p_general_config->axis_count);
        return;
    }

    if(m_b_ret_ref || this->IsRefReturnning(phy_axis)){
        printf("axis %hhu is returnning ref, return\n", phy_axis);
        return;   //已在回参考点过程中，返回
    }

    HmiChannelStatus chn_status;
    this->GetChnStatus(this->m_n_cur_channle_index, chn_status);
    if(chn_status.machining_state != MS_READY){   //非准备好模式不执行回零
        printf("machine state is not ready:%hhu, return \n", chn_status.machining_state);
        return;
    }
    if(g_ptr_alarm_processor->HasErrorInfo()){ //有告警，拒绝回零
        CreateError(ERR_RET_REF_FAILED, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, 0);
        return;
    }

    if(m_p_axis_config[phy_axis].axis_interface == VIRTUAL_AXIS || m_p_axis_config[phy_axis].axis_type == AXIS_SPINDLE)	//主轴和虚拟轴不用回参考点
    {  //增量编码器，禁止回参考点
        printf("no ret ref, return\n");
        return;   //不用回参考点的轴禁止将回零标志复位
    }

    this->SetRetRefMask(phy_axis);
    this->SetRetRefFlag(phy_axis, false);   //复位回参考点完成标志

    this->m_b_ret_ref = true;
    this->m_b_ret_ref_auto = false;
}

/**
 * @brief 设置回参考点轴mask
 * @param phy_axis : 物理轴序号，0开始
 */
void ChannelEngine::SetRetRefMask(uint8_t phy_axis){
    if(m_p_axis_config[phy_axis].axis_interface != VIRTUAL_AXIS		//非虚拟轴
            && m_p_axis_config[phy_axis].axis_type != AXIS_SPINDLE				//非主轴
            /*	&& (m_p_axis_config[phy_axis].feedback_mode == INCREMENTAL_ENCODER ||
                        m_p_axis_config[phy_axis].feedback_mode == NO_ENCODER)*/){	//所有反馈类型都支持回参考点
        this->m_n_mask_ret_ref |= (0x01<<phy_axis);
    }
}

/**
 * @brief 设置轴回零中信号
 * @param phy_axis : 物理轴序号，0开始
 * @param flag : true--置1  flase--置0
 */
void ChannelEngine::SetInRetRefFlag(uint8_t phy_axis, bool flag){
    if(phy_axis >= this->m_p_general_config->axis_count){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ChannelEngine::SetInRetRefFlag() return, 物理轴号[%hhu: %hhu]非法!", phy_axis, m_p_general_config->axis_count);
        return;
    }
    uint8_t chn = phy_axis/16;
    uint8_t bit = phy_axis%16;

    if(flag)
        this->m_p_pmc_reg->FReg().bits[chn].IRF |= (0x01<<bit);
    else
        this->m_p_pmc_reg->FReg().bits[chn].IRF &= ~(0x01<<bit);
}

/**
 * @brief 指定轴是否正在回零中
 * @param phy_axis  : 物理轴序号，0开始
 * @return true--指定轴phy_axis正在回零中    false--未在回零过程中
 */
bool ChannelEngine::IsRefReturnning(uint8_t phy_axis){
    if(phy_axis >= this->m_p_general_config->axis_count)
        return false;

    uint64_t flag = 0x01;
    if(this->m_n_mask_ret_ref & (flag<<phy_axis))
        return true;

    return false;
}

/**
 * @brief 处理HMI设置参考点指令，适用于绝对值编码器
 * @param cmd ： HMI指令
 */
void ChannelEngine::ProcessHmiSetRefCmd(HMICmdFrame &cmd){

    printf("ChannelEngine::ProcessHmiSetRefCmd\n");

    if(this->m_n_cur_pmc_axis != 0xff){

        MiCmdFrame mi_cmd;
        memset(&mi_cmd, 0x00, sizeof(mi_cmd));
        mi_cmd.data.cmd = CMD_MI_SET_REF_CUR;
        mi_cmd.data.axis_index = m_n_cur_pmc_axis+1;
        int64_t pos = m_p_axis_config[m_n_cur_pmc_axis].axis_home_pos[0] * 1e7;   //单位转换，0.1nm
        memcpy(mi_cmd.data.data, &pos, sizeof(int64_t));

        this->m_p_mi_comm->WriteCmd(mi_cmd);
    }

    cmd.data[0] = SUCCEED;
    cmd.data_len = 0x01;
    cmd.frame_number |= 0x8000;
    this->m_p_hmi_comm->SendCmd(cmd);

}



/**
 * @brief 给HMI发送升级状态
 * @param total_step : 总步数
 * @param cur_step ：当前状态
 */
void ChannelEngine::SendHmiUpdateStatus(uint8_t total_step, uint8_t cur_step){
    //给HMI发送升级状态
    HMICmdFrame cmd;
    cmd.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.cmd = CMD_SC_UPDATE_MODULE_STATUS;

    cmd.cmd_extension = this->m_n_update_state;
    cmd.data_len = 2;
    cmd.data[0] = total_step;
    cmd.data[1] = cur_step;

    this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief 获取指定的通道控制对象指针
 * @param index
 * @return
 */
ChannelControl *ChannelEngine::GetChnControl(uint8_t index){
    return index < m_p_general_config->chn_count? &m_p_channel_control[index]:nullptr;
}

/**
 * @brief 获取通道状态
 * @param chn_index[in] : 通道索引
 * @param status[out] ：输出的通道状态
 * @return true--成功  false--失败
 */
bool ChannelEngine::GetChnStatus(uint8_t chn_index, HmiChannelStatus &status){
    if(chn_index >= this->m_p_general_config->chn_count){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "通道索引号[%d]非法！", chn_index);
        return false;
    }

    this->m_p_channel_control[chn_index].GetChnStatus(status);
    return true;
}

/**
 * @brief 获取指定通道内轴所对应的物理轴号
 * @param chn_index[in] : 通道索引
 * @param chn_axis[in] ：通道轴号
 * @return 返回物理轴号,不成功返回0xff，成功返回0开始的物理轴索引号
 */
uint8_t ChannelEngine::GetChnAxistoPhyAixs(uint8_t chn_index, uint8_t chn_axis){
    if(chn_index >= this->m_p_general_config->chn_count){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "通道索引号[%d]非法！", chn_index);
        return 0xff;
    }

    uint8_t axis = this->m_p_channel_control[chn_index].GetPhyAxis(chn_axis);
    return axis;
}

/**
 * @brief 启动程序，用于响应循环启动
 * @return true--成功   false--失败
 */
bool ChannelEngine::Start(){
    printf("Start :m_b_emergency=%d\n",m_b_emergency);
    if(this->m_b_emergency)
        return false;

#ifdef USES_LICENSE_FUNC
    //检查系统时间
    if(m_ln_local_time < 0){//读取本地计时文件异常
        if(m_ln_local_time == -1){
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "本地计时文件不存在!");
        }else if(m_ln_local_time == -2){
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "本地计时文件已损坏!");
        }else if(m_ln_local_time == -3){
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "非法本地计时文件!");
        }
        m_error_code = ERR_SYSTEM_FILE;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
        return false;
    }else if(-4 == CheckLocalTime(&m_lic_info, m_ln_local_time)){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "系统时间异常!");
        m_error_code = ERR_SYSTEM_TIME;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
        return false;
    }

    //检查授权
    if(1 == this->CheckLicense()){  //非法授权
        return false;
    }
#endif

    //检查是否导入过配置数据，提示重启
    if(this->m_mask_import_param != 0){
        CreateError(ERR_IMPORT_CONFIG, WARNING_LEVEL, CLEAR_BY_MCP_RESET);
        return false;
    }

    ChnWorkMode work_mode = (ChnWorkMode)m_p_channel_control[m_n_cur_channle_index].GetChnWorkMode();

    if(work_mode == AUTO_MODE){
        //		m_p_channel_control[m_n_cur_channle_index].StartRunGCode();
        //		for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        //			m_p_channel_control[i].StartRunGCode();
        //		}


        uint8_t chn = 0;
        for(uint8_t i = 0; i < m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount(); i++){
            chn = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i);
            UpdateHandwheelState(chn);

            m_p_channel_control[chn].StartRunGCode();
        }

    }else if(work_mode == MDA_MODE){
        m_p_channel_control[m_n_cur_channle_index].StartRunGCode();
    }

    return true;
}

/**
 * @brief 暂停程序，用于响应进给保持
 * @return true--成功   false--失败
 */
bool ChannelEngine::Pause(){
    ChnWorkMode work_mode = (ChnWorkMode)m_p_channel_control[m_n_cur_channle_index].GetChnWorkMode();

    if(work_mode == AUTO_MODE){
        //		m_p_channel_control[m_n_cur_channle_index].Pause();
        //		for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        //			m_p_channel_control[i].Pause();
        //		}

        for(uint8_t i = 0; i < m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount(); i++){
            m_p_channel_control[m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i)].Pause();
        }

    }else if(work_mode == MDA_MODE){
        m_p_channel_control[m_n_cur_channle_index].Pause();
    }

    return true;
}

/**
 * @brief 停止程序执行，用于处理系统告警下的程序停止
 * @param reset : 是否复位数据和行号， true--复位   false--不复位
 * @return true--成功   false--失败
 */
bool ChannelEngine::Stop(bool reset){
    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        m_p_channel_control[i].StopRunGCode(reset);
    }
    return true;
}

/**
 * @brief 停止程序执行，用于处理系统告警下的程序停止
 * @param chn : 通道号,从0开始
 * @param reset : 是否复位数据和行号， true--复位   false--不复位
 * @return true--成功   false--失败
 */
bool ChannelEngine::Stop(uint8_t chn, bool reset){
    if(chn < this->m_p_general_config->chn_count){
        m_p_channel_control[chn].StopRunGCode(reset);
    }else if(chn == CHANNEL_ENGINE_INDEX){
        for(int i = 0; i < this->m_p_general_config->chn_count; i++){
            m_p_channel_control[i].StopRunGCode(reset);
        }
    }
    return true;
}
/**
 * @brief 设置当前通道号
 * @param work_chan
 * @return
 */
bool ChannelEngine::SetCurWorkChanl(uint8_t work_chan){

    //	int chn_count = this->m_p_general_config->chn_count;
    if(work_chan == CHANNEL_ENGINE_INDEX)
        return true;

    if(work_chan >= m_p_general_config->chn_count)
        return false;

    m_n_cur_channle_index = work_chan;

    // 通道变化时 将F219 前4位赋值
    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        m_p_pmc_reg->FReg().bits[i].CHNC = work_chan;
    }

    this->m_n_cur_chn_group_index = this->m_p_channel_config[work_chan].chn_group_index;

    this->SetMiCurChannel();
    return true;
}

/**
 * @brief 设置工作模式, 自动、MDA、手动、手轮
 * @param work_mode
 * @return
 */
bool ChannelEngine::SetWorkMode(uint8_t work_mode){

    //	int chn_count = this->m_p_general_config->chn_count;

    /*
    for(int i = 0; i < chn_count; i++){
        this->m_p_channel_control[i].SetWorkMode(work_mode);
    }
*/

    if(work_mode != REF_MODE && this->m_b_ret_ref){
        CreateError(ERR_SWITCH_MODE_IN_RET_REF, WARNING_LEVEL, CLEAR_BY_MCP_RESET);
        return false;
    }

    uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();
    for(uint8_t i = 0; i < chn_count; i++)
        this->m_p_channel_control[m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i)].SetWorkMode(work_mode);

    if(work_mode == MPG_MODE)
        this->SetMiWorkMode(0x10);
    else if(work_mode == AUTO_MODE || work_mode == MDA_MODE)
        this->SetMiWorkMode(0x00);
    else if(work_mode == MANUAL_STEP_MODE || work_mode == MANUAL_MODE || work_mode == REF_MODE)   //参考点模式对MI等效于手动模式
        this->SetMiWorkMode(0x20);

    return true;
}

/**
 * @brief 设置功能状态，例如：单段，选停等等
 * @param chn ： 通道号， 从0开始, 0xFF表示对所有通道
 * @param state : 设置的状态
 * @param mode : 状态的开关   0--关闭   1--打开    10--点动（即根据当前状态取反）
 */
void ChannelEngine::SetFuncState(uint8_t chn, int state, uint8_t mode){

    if(chn >= this->m_p_general_config->chn_count && chn != CHANNEL_ENGINE_INDEX)
        return;  //通道号非法

    if(chn < m_p_general_config->chn_count){
        this->m_p_channel_control[chn].SetFuncState(state, mode);

        if(state == FS_HANDWHEEL_CONTOUR){ //手轮跟踪，通知MI切换状态
            this->UpdateHandwheelState(chn);
        }
    }else{
        for(uint8_t i = 0; i < m_p_general_config->chn_count; i++){
            this->m_p_channel_control[i].SetFuncState(state, mode);
            if(state == FS_HANDWHEEL_CONTOUR){ //手轮跟踪，通知MI切换状态
                this->UpdateHandwheelState(chn);
            }
        }
    }

}

/**
 * @brief 向MI更新手轮反向跟踪使能
 */
void ChannelEngine::EnableHWTraceToMi(){
    uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();
    for(uint8_t i = 0; i < chn_count; i++){
        this->UpdateHandwheelState(i);
    }
}

/**
 * @brief 设置自动倍率
 * @param ratio
 */
void ChannelEngine::SetAutoRatio(uint8_t ratio){
    //	this->m_p_channel_control[m_n_cur_channle_index].SetAutoRatio(ratio);

    //	int chn_count = this->m_p_general_config->chn_count;
    //	for(int i = 0; i < chn_count; i++){
    //		this->m_p_channel_control[i].SetAutoRatio(ratio);
    //	}

    uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();
    for(uint8_t i = 0; i < chn_count; i++)
        this->m_p_channel_control[m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i)].SetAutoRatio(ratio);
}

/**
 * @brief 设置自动倍率
 * @param chn : 通道号，从0开始
 * @param ratio ： 倍率值
 */
void ChannelEngine::SetAutoRatio(uint8_t chn, uint8_t ratio){
    this->m_p_channel_control[chn].SetAutoRatio(ratio);
}

/**
 * @brief 设置手动倍率
 * @param ratio
 */
void ChannelEngine::SetManualRatio(uint8_t ratio){
    //	this->m_p_channel_control[m_n_cur_channle_index].SetManualRatio(ratio);


    //	int chn_count = this->m_p_general_config->chn_count;
    //	for(int i = 0; i < chn_count; i++){
    //		this->m_p_channel_control[i].SetManualRatio(ratio);
    //	}

    uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();
    for(uint8_t i = 0; i < chn_count; i++)
        this->m_p_channel_control[m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i)].SetManualRatio(ratio);
}

/**
 * @brief 设置手动倍率
 * @param chn : 通道号，从0开始
 * @param ratio : 倍率值
 */
void ChannelEngine::SetManualRatio(uint8_t chn, uint8_t ratio){
    printf("ChannelEngine::SetManualRatio, old = %hhu, new = %hhu\n", this->m_p_channel_control[chn].GetManualRatio(), ratio);
    if(this->m_p_channel_control[chn].GetManualRatio() == ratio)
        return;
    this->m_p_channel_control[chn].SetManualRatio(ratio);
}

/**
 * @brief 设置快速进给倍率
 * @param ratio
 */
void ChannelEngine::SetRapidRatio(uint8_t ratio){
    //	printf("chn_engine set rapid ratio:%d\n", ratio);
    //	this->m_p_channel_control[m_n_cur_channle_index].SetRapidRatio(ratio);

    /*
    int chn_count = this->m_p_general_config->chn_count;
    for(int i = 0; i < chn_count; i++){
        this->m_p_channel_control[i].SetRapidRatio(ratio);
    }*/

    uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();
    for(uint8_t i = 0; i < chn_count; i++)
        this->m_p_channel_control[m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i)].SetRapidRatio(ratio);
}

/**
 * @brief 设置快速进给倍率
 * @param chn : 通道号，从0开始
 * @param ratio : 倍率值
 */
void ChannelEngine::SetRapidRatio(uint8_t chn, uint8_t ratio){
    //倍率值映射
    //	switch(ratio){
    //	case 0:
    //		ratio = 100;
    //		break;
    //	case 2:
    //		ratio = 50;
    //		break;
    //	case 1:
    //		ratio = 25;
    //		break;
    //	case 3:
    //		ratio = 0;
    //		break;
    //	}
    this->m_p_channel_control[chn].SetRapidRatio(ratio);
}

/**
 * @brief 设置手动步长
 * @param step
 */
void ChannelEngine::SetManualStep(uint16_t step){
    //	int chn_count = this->m_p_general_config->chn_count;
    uint8_t tt = 0;
    switch(step){
    case 1:
        tt = MANUAL_STEP_1;
        break;
    case 10:
        tt = MANUAL_STEP_10;
        break;
    case 100:
        tt = MANUAL_STEP_100;
        break;
    case 1000:
        tt = MANUAL_STEP_1000;
        break;
    default:
        tt = MANUAL_STEP_INVALID;
        break;
    }
    //	this->m_p_channel_control[m_n_cur_channle_index].SetManualStep(tt);
    //	for(int i = 0; i < chn_count; i++){
    //		this->m_p_channel_control[i].SetManualStep(tt);
    //	}

    g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "[手动步长]切换为 " + to_string(step));

    uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();
    for(uint8_t i = 0; i < chn_count; i++)
        this->m_p_channel_control[m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i)].SetManualStep(tt);
}

/**
 * @brief 设置手动步长
 * @param chn : 通道号，从0开始
 * @param step : 步长代码
 */
void ChannelEngine::SetManualStep(uint8_t chn, uint8_t step){
    switch(step){
    case 0:
        step = MANUAL_STEP_1;
        m_p_mi_comm->SendMpgStep(chn,true,1);
        break;
    case 1:
        step = MANUAL_STEP_10;
        m_p_mi_comm->SendMpgStep(chn,true,10);
        break;
    case 2:
        step = MANUAL_STEP_100;
        m_p_mi_comm->SendMpgStep(chn,true,m_p_channel_config[chn].mpg_level3_step);
        break;
    case 3:
        step = MANUAL_STEP_1000;
        m_p_mi_comm->SendMpgStep(chn,true,m_p_channel_config[chn].mpg_level4_step);
        break;
    }

    const vector<string> table = {"1", "10", "100", "1000"};
    if (step > 0 && step < table.size())
        g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "[手动步长]切换为 " + table[step]);
    this->m_p_channel_control[chn].SetManualStep(step);
}

/**
 * @brief 设置手动快速移动状态
 * @param mode : 状态的开关   0--关闭   1--打开    10--点动（即根据当前状态取反）
 */
void ChannelEngine::SetManualRapidMove(uint8_t mode){
    //	this->m_p_channel_control[m_n_cur_channle_index].SetManualRapidMove();
    /*
    int chn_count = this->m_p_general_config->chn_count;
    for(int i = 0; i < chn_count; i++){
        this->m_p_channel_control[i].SetManualRapidMove();
    }*/

    uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();   //当前方式组通道数量
    for(uint8_t i = 0; i < chn_count; i++)
        this->m_p_channel_control[m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i)].SetManualRapidMove(mode);
}

/**
 * @brief 设置当前轴
 * @param axis : 当前通道轴号
 */
void ChannelEngine::SetCurAxis(uint8_t axis){
    //	int chn_count = this->m_p_general_config->chn_count;

    this->m_n_cur_pmc_axis = 0xFF;  //取消当前PMC轴
    //	this->m_p_channel_control[m_n_cur_channle_index].SetCurAxis(axis);
    /*
    for(int i = 0; i < chn_count; i++){
        this->m_p_channel_control[i].SetCurAxis(axis);
    }*/

    uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();
    for(uint8_t i = 0; i < chn_count; i++)
        this->m_p_channel_control[m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i)].SetCurAxis(axis);
}

/**
 * @brief 设置当前轴
 * @param chn : 通道号，从0开始
 * @param axis ： 通道轴号， 从0开始
 */
void ChannelEngine::SetCurAxis(uint8_t chn, uint8_t axis){
    this->m_n_cur_pmc_axis = 0xFF;  //取消当前PMC轴
    this->m_p_channel_control[chn].SetCurAxis(axis);
}

/**
 * @brief 设置当前PMC轴
 * @param axis : 物理轴号，从0开始
 */
void ChannelEngine::SetCurPmcAxis(uint8_t axis){
    if(this->m_p_axis_config[axis].axis_pmc > 0)
        this->m_n_cur_pmc_axis = axis;
    else
        this->m_n_cur_pmc_axis = 0xFF;
}

/**
 * @brief 设置通道加工状态
 * @param chn : 通道索引号，0开始，0xFF表示通道引擎
 * @param mach_state : 加工状态
 */
void ChannelEngine::SetChnMachineState(uint8_t chn, uint8_t mach_state){
    if(chn < m_p_general_config->chn_count){
        this->m_p_channel_control[chn].SetMachineState(mach_state);
    }else if(chn == CHANNEL_ENGINE_INDEX){
        for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
            this->m_p_channel_control[i].SetMachineState(mach_state);
        }
    }
}

/**
 * @brief 设置轴使能，以及插补模式（1--NC轴插补   2--PMC轴插补）
 * @param index : 轴号索引，从0开始
 */
/*
void ChannelEngine::SetAxisOn(uint8_t index){
    McCmdFrame cmd;
    cmd.data.axis_index = index+1;
    cmd.data.channel_index = m_n_cur_channle_index;   // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SET_AXIS_ON;

    //轴使能  0--禁止该轴   1--使能该轴
    SCAxisConfig &axis_config = this->m_p_axis_config[index];
    if(axis_config.axis_type != AXIS_SPINDLE)
        cmd.data.data[0] = 1;
    else
        cmd.data.data[0] = 0;


    //轴插补类型   1--NC轴插补（自动、手动、MDI）  2--PMC轴插补   其它值无效  （注意10MC的DSP暂不支持 纯粹PMC轴）
    cmd.data.data[1] = 1;

    m_p_mc_comm->WriteCmd(cmd);
}*/

/**
 * @brief 设置指定轴的螺距及最大速度等基本信息
 * @param index : 轴索引号, 从0开始
 */
/*
void ChannelEngine::SetAxisBaseParam(uint8_t index){
    McCmdFrame cmd;
    cmd.data.axis_index = index+1;
    cmd.data.channel_index = m_n_cur_channle_index;  // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SET_AXIS_BASE_INFO;

    SCAxisConfig &axis_config = this->m_p_axis_config[index];
    //轴类型   1--直线轴   2--旋转轴   3--主轴
    cmd.data.data[0] = axis_config.axis_type+1;

    //螺距，单位：1um
    uint32_t data = axis_config.move_pr * 1e3;  //mm->1um
    cmd.data.data[1] = (data&0xFFFF);
    cmd.data.data[2] = ((data>>16)&0xFFFF);

    //最大速度限制， 单位：um/s
    data = axis_config.move_pr * axis_config.motor_speed_max * 1000 /60;   //mm/min -> um/s
    cmd.data.data[3] = (data&0xFFFF);
    cmd.data.data[4] = ((data>>16)&0xFFFF);

    m_p_mc_comm->WriteCmd(cmd);

}*/

/**
 * @brief 设置指定轴的速度相关信息
 * @param index : 轴索引号, 从0开始
 */
/*
void ChannelEngine::SetAxisSpeedParam(uint8_t index){
    McCmdFrame cmd;
    cmd.data.axis_index = index+1;
    cmd.data.channel_index = m_n_cur_channle_index;  // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SET_AXIS_SPEED_LIMIT;

    SCAxisConfig &axis_config = this->m_p_axis_config[index];
    //G00速度，单位：um/s
    uint32_t data = axis_config.rapid_speed * 1000 / 60;  //mm/min -> um/s
//	printf("set axis %hhu g00 speed: %u\n", index, data);
    cmd.data.data[0] = (data&0xFFFF);
    cmd.data.data[1] = ((data>>16)&0xFFFF);
//	printf("set axis %hhu g00 speed: 0x%hx,  0x%hx\n", index, cmd.data.data[0], cmd.data.data[1]);

    //转角速度差限制， 单位：um/s
    data = axis_config.corner_acc_limit * 1000 /60;   //mm/min -> um/s
    cmd.data.data[2] = (data&0xFFFF);
    cmd.data.data[3] = ((data>>16)&0xFFFF);

    //手动过渡速度，单位：um/s  //TODO 此参数后续将启用，此处传输一个固定值100mm/min
    data = 100 * 1000 / 60;    //mm/min -> um/s
    cmd.data.data[4] = (data&0xFFFF);
    cmd.data.data[5] = ((data>>16)&0xFFFF);

    m_p_mc_comm->WriteCmd(cmd);
}*/

/**
 * @brief 设置指定轴加速度相关信息
 * @param index : 轴索引号, 从0开始
 */
/*
void ChannelEngine::SetAxisAccParam(uint8_t index){
    McCmdFrame cmd;
    cmd.data.axis_index = index+1;
    cmd.data.channel_index = m_n_cur_channle_index;  // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

    cmd.data.cmd = CMD_MC_SET_AXIS_ACC;
//	printf("set axis[%hhu] acc:rap = %lf, start = %lf\n", index, m_p_axis_config[index].rapid_acc,
//			m_p_axis_config[index].start_acc);

    //G00加速度，单位：10mm/s^2
    uint16_t data = m_p_axis_config[index].rapid_acc / 10;
    cmd.data.data[0] = data;

    //手动加速度， 单位：10mm/s^2
    data = m_p_axis_config[index].manual_acc /10;
    cmd.data.data[1] = data;

    //手动过渡加速度，单位：10mm/s^2
    data = m_p_axis_config[index].start_acc /10;
    cmd.data.data[2] = data;

    //G00 S型时间常数
    cmd.data.data[3] = m_p_axis_config[index].rapid_s_plan_filter_time;

    m_p_mc_comm->WriteCmd(cmd);
}*/

/**
 * @brief 发送PMC轴运动指令
 * @param cmd : PMC轴运动指令
 */
void ChannelEngine::SendPmcAxisCmd(PmcCmdFrame &cmd){
    uint8_t phy_axis = cmd.data.axis_index-1;

    this->m_n_run_axis_mask |= 0x01L<<phy_axis;  //设置当前运行轴

    this->m_p_mi_comm->SendPmcCmd(cmd);
}

/**
 * @brief 获取pmc轴的设定定位速度
 * @param axis : 物理轴号， 从0开始
 * @return 返回此轴参数中设置的定位速度
 */
uint32_t ChannelEngine::GetPmcAxisRapidSpeed(uint8_t axis){
    if(axis >= this->m_p_general_config->axis_count)
        return 0;
    return this->m_p_axis_config[axis].rapid_speed;
}

void ChannelEngine::SetJPState(uint8_t chn, uint8_t JP, uint8_t last_JP, ChnWorkMode mode){

    for(int i = 0; i < m_p_channel_config[chn].chn_axis_count; i++){
        bool flag_now = (JP & (0x01 << i));
        bool flag_last = (last_JP & (0x01 << i));
        if(flag_now == flag_last)
            continue;

        uint8_t chn_axis;
        GetAxisChannel(m_p_channel_config[chn].chn_axis_phy[i]-1,chn_axis);
        // 轴正向移动按下
        if(flag_now){
            this->m_p_mi_comm->ReadPhyAxisCurFedBckPos(m_df_phy_axis_pos_feedback, m_df_phy_axis_pos_intp,m_df_phy_axis_speed_feedback,
                                                       m_df_phy_axis_torque_feedback, m_df_spd_angle, m_p_general_config->axis_count);
            SetCurAxis(chn, chn_axis);
            ManualMove(DIR_POSITIVE);
            g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "点动[轴" + to_string(chn_axis) + "]+");
        }else if(mode == MANUAL_MODE){ // 轴正向移动松开，并且为手动连续模式
            ManualMoveStop(m_p_channel_config[chn].chn_axis_phy[i]-1);
            //g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "点动释放[轴" + to_string(chn_axis) + "]+");
            usleep(200000);
        }
    }
}

void ChannelEngine::SetJNState(uint8_t chn, uint8_t JN, uint8_t last_JN, ChnWorkMode mode){
    for(int i = 0; i < m_p_channel_config[chn].chn_axis_count; i++){
        bool flag_now = (JN & (0x01 << i));
        bool flag_last = (last_JN & (0x01 << i));
        if(flag_now == flag_last)
            continue;

        uint8_t chn_axis;
        GetAxisChannel(m_p_channel_config[chn].chn_axis_phy[i]-1,chn_axis);
        // 轴负向移动按下
        if(flag_now){
            this->m_p_mi_comm->ReadPhyAxisCurFedBckPos(m_df_phy_axis_pos_feedback, m_df_phy_axis_pos_intp,m_df_phy_axis_speed_feedback,
                                                       m_df_phy_axis_torque_feedback, m_df_spd_angle, m_p_general_config->axis_count);

            SetCurAxis(chn, chn_axis);
            ManualMove(DIR_NEGATIVE);

            g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "点动[轴" + to_string(chn_axis) + "]-");
        }else if(mode == MANUAL_MODE){ // 轴正向移动松开，并且为手动连续模式
            ManualMoveStop(m_p_channel_config[chn].chn_axis_phy[i]-1);
            //g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "点动释放[轴" + to_string(chn_axis) + "]-");
            usleep(200000);
        }
    }
}

/**
 * @brief 手动移动
 * @param dir : 运动方向
 */
void ChannelEngine::ManualMove(int8_t dir){
    if(this->m_b_emergency)
        return;
//    if(this->m_n_cur_pmc_axis != 0xFF){  //移动PMC轴
//        this->ManualMovePmc(dir);
//    }else
     m_p_channel_control[m_n_cur_channle_index].ManualMove(dir);
}

/**
 * @brief 手动以目标速度vel移动至绝对目标位置
 * @param phy_axis : 物理轴号，从0开始
 * @param vel : 运动速度, 单位：mm/min
 * @param pos : 绝对位置
 */
void ChannelEngine::ManualMoveAbs(uint8_t phy_axis, double vel, double pos){
    int64_t cur_pos = this->m_df_phy_axis_pos_feedback[phy_axis]*1e7;  //当前位置
    //设置目标位置
    int64_t tar_pos = pos * 1e7;   //单位转换：mm-->0.1nms
    ManualMoveDir dir = (tar_pos > cur_pos)?DIR_POSITIVE:DIR_NEGATIVE;  //移动方向

    //检查硬限位
    if(CheckAxisHardLimit(phy_axis, dir)){   //硬限位告警，直接返回
        printf("hard limit active, manual move abs return \n");
        return;
    }
    //检查软限位
    double limit = 0;
    if(CheckSoftLimit(dir, phy_axis, this->m_df_phy_axis_pos_feedback[phy_axis])){
        printf("soft limit active, manual move abs return \n");
        return;
    }else if(GetSoftLimt(dir, phy_axis, limit) && dir == DIR_POSITIVE && pos > limit * 1e7){
        tar_pos = limit * 1e7;
    }else if(GetSoftLimt(dir, phy_axis, limit) && dir == DIR_NEGATIVE && pos < limit * 1e7){
        tar_pos = limit * 1e7;
    }


    uint8_t chn_axis = 0, chn = 0;
    chn = this->GetAxisChannel(phy_axis, chn_axis);

    if (!GetPmcActive(phy_axis) && chn != CHANNEL_ENGINE_INDEX) {
        McCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(McCmdFrame));

        cmd.data.channel_index = chn;
        cmd.data.axis_index = chn_axis+1;   //轴号从1开始
        cmd.data.cmd = CMD_MC_AXIS_MAUAL_MOVE;

        //设置速度
        uint32_t feed = vel*1000/60;   //转换单位为um/s

        //	memcpy(cmd.data.data, &feed, sizeof(feed));
        cmd.data.data[0] = (feed & 0xFFFF);
        cmd.data.data[1] = ((feed>>16)&0xFFFF);
        cmd.data.data[2] = (tar_pos & 0xFFFF);
        cmd.data.data[3] = ((tar_pos>>16)&0xFFFF);
        cmd.data.data[4] = ((tar_pos>>32) & 0xFFFF);
        cmd.data.data[5] = ((tar_pos>>48)&0xFFFF);

        cmd.data.data[6] = 0x00;   //绝对目标位置，机械坐标系

        if(!this->m_mc_run_on_arm[chn])
            m_p_mc_comm->WriteCmd(cmd);
        else
            m_p_mc_arm_comm->WriteCmd(cmd);

        printf("ChannelEngine::ManualMoveAbs: axis = %d, tar_pos = %lld\n", phy_axis, tar_pos);
    }else if (GetPmcActive(phy_axis)) {
#ifdef USES_GRIND_MACHINE
        //动作前夹爪必须上到位，保护措施
        if(this->m_p_pmc_reg->GReg().bits[0].left_claw_up_check == 0 ||
                this->m_p_pmc_reg->GReg().bits[0].right_claw_up_check == 0){//夹爪位上到位
            return;
        }
#endif
        //PmcCmdFrame cmd;
        //memset(&cmd, 0x00, sizeof(PmcCmdFrame));

        //cmd.data.axis_index = phy_axis+1;   //轴号从1开始
        //cmd.data.axis_index |= 0xFF00;      //标志通道引擎
        //cmd.data.cmd = 0;   //绝对位置

        //设置速度
        //uint32_t feed = vel*1000/60;   //转换单位为um/s

        //memcpy(&cmd.data.data[1], &tar_pos, sizeof(tar_pos));  //设置目标位置
        //memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //设置速度

        //this->m_n_run_axis_mask |= 0x01L<<phy_axis;  //设置当前运行轴

        //this->m_p_mi_comm->SendPmcCmd(cmd);

        //printf("ChannelEngine::ManualMove_pmc: axis = %d, tar_pos = %lld\n", phy_axis, tar_pos);
        m_error_code = ERR_PMC_IVALID_USED;
        CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, 0xFF);
        return;
    }
}

/**
 * @brief 手动移动，向dir方向移动dis距离
 * @param chn : 通道号， 从0开始
 * @param phy_axis : 物理轴号，从0开始
 * @param dir : 运动方向
 * @param vel : 运动速度, 单位：mm/min
 * @param inc_dis ： 增量位置, 不管正负，在函数内部根据dir决定正负
 */
void ChannelEngine::ManualMove(uint8_t phy_axis, int8_t dir, double vel, double inc_dis){
    uint8_t chn_axis = 0, chn = 0;
    chn = this->GetAxisChannel(phy_axis, chn_axis);
    //检查硬限位
    if(CheckAxisHardLimit(phy_axis, dir)){   //硬限位告警，直接返回
        printf("hard limit active, manual move return 3 \n");
        return;
    }
    double cur_pos = this->m_df_phy_axis_pos_feedback[phy_axis]*1e7;
    //检查软限位
    double tar_pos = (fabs(inc_dis)*dir + m_p_channel_control[chn].GetAxisCurIntpTarPos(chn_axis, true))*1e7;
    double limit = 0;
    if(CheckSoftLimit((ManualMoveDir)dir, phy_axis, this->m_df_phy_axis_pos_feedback[phy_axis])){
        printf("soft limit active, manual move abs return \n");
        return;
    }else if(GetSoftLimt((ManualMoveDir)dir, phy_axis, limit) && dir == DIR_POSITIVE && tar_pos > limit * 1e7){
        tar_pos = limit * 1e7;
    }else if(GetSoftLimt((ManualMoveDir)dir, phy_axis, limit) && dir == DIR_NEGATIVE && tar_pos < limit * 1e7){
        tar_pos = limit * 1e7;
    }

    int64_t n_inc_dis = tar_pos -  m_p_channel_control[chn].GetAxisCurIntpTarPos(chn_axis, true)*1e7;

    if (!GetPmcActive(phy_axis) && chn != CHANNEL_ENGINE_INDEX) {
        McCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(McCmdFrame));

        cmd.data.channel_index = chn;
        cmd.data.axis_index = chn_axis+1;   //轴号从1开始
        cmd.data.cmd = CMD_MC_AXIS_MAUAL_MOVE;

        //设置速度
        uint32_t feed = vel*1000/60;   //转换单位为um/s

        //	memcpy(cmd.data.data, &feed, sizeof(feed));
        cmd.data.data[0] = (feed & 0xFFFF);
        cmd.data.data[1] = ((feed>>16)&0xFFFF);
        cmd.data.data[2] = (n_inc_dis& 0xFFFF);
        cmd.data.data[3] = ((n_inc_dis>>16)&0xFFFF);
        cmd.data.data[4] = ((n_inc_dis>>32) & 0xFFFF);
        cmd.data.data[5] = ((n_inc_dis>>48)&0xFFFF);
        cmd.data.data[6] = 0x02;   //增量目标位置

        if(!this->m_mc_run_on_arm[chn])
            m_p_mc_comm->WriteCmd(cmd);
        else
            m_p_mc_arm_comm->WriteCmd(cmd);

        //	printf("ChannelEngine::ManualMove: axis = %d, tar_pos = %lld\n", phy_axis, tar_pos);
    }else if (GetPmcActive(phy_axis)) {
        //PmcCmdFrame cmd;
        //memset(&cmd, 0x00, sizeof(PmcCmdFrame));

        //cmd.data.axis_index = phy_axis+1;   //轴号从1开始
        //cmd.data.axis_index |= 0xFF00;      //标志通道引擎
        //cmd.data.cmd = 0x100;   //增量位置

        //设置速度
        //uint32_t feed = vel*1000/60;   //转换单位为um/s
        //memcpy(&cmd.data.data[1], &n_inc_dis, sizeof(n_inc_dis));  //设置目标位置
        //memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //设置速度

        //this->m_n_run_axis_mask |= 0x01L<<phy_axis;  //设置当前运行轴

        //this->m_p_mi_comm->SendPmcCmd(cmd);

        //		printf("ChannelEngine::ManualMove_pmc: axis = %d, tar_pos = %lld\n", phy_axis, tar_pos);
        m_error_code = ERR_PMC_IVALID_USED;
        CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, 0xFF);
    }
}

/**
 * @brief 手动停止
 * @param 无
 */
void ChannelEngine::ManualMoveStop(){
    //停止通道轴
    m_p_channel_control[m_n_cur_channle_index].ManualMoveStop();

    //停止PMC轴移动
    this->ManualMovePmcStop();


    //	if(this->m_n_run_axis_mask){
    //		MiCmdFrame cmd;
    //		memset(&cmd, 0x00, sizeof(MiCmdFrame));
    //
    //		cmd.data.axis_index = 0xFF;
    //		cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
    //		cmd.data.data[0] = 0x20;   //停止当前轴运动，并抛弃当前运动指令
    //
    //		m_p_mi_comm->WriteCmd(cmd);
    //
    //		this->m_n_run_axis_mask = 0;
    //		this->m_n_runover_axis_mask = 0;
    //		printf("send manual pmc stop : runmask = 0x%x\n", this->m_n_run_axis_mask);
    //	}

}

/**
 * @brief 指定轴手动移动
 */
void ChannelEngine::ManualMovePmc(uint8_t phy_axis, int8_t dir){
    uint8_t chn_axis = 0, chn = 0;
    chn = this->GetAxisChannel(phy_axis, chn_axis);
    //检查硬限位
    if(CheckAxisHardLimit(phy_axis, dir)){   //硬限位告警，直接返回
        printf("hard limit active, manual move pmc return \n");
        return;
    }
    //设置目标位置
    int64_t cur_pos = m_df_phy_axis_pos_feedback[phy_axis]*1e7;  //当前位置
    int64_t tar_pos = 0;
    if(this->m_p_channel_control[0].GetChnWorkMode() == MANUAL_STEP_MODE){ //手动单步
        tar_pos = cur_pos + m_p_channel_control[chn].GetCurManualStep()*1e4*dir;		//转换单位为0.1nm

    }else{
        tar_pos = cur_pos + 99999*1e7*dir;    //手动连续模式，将目标位置设置的很远
    }
    //检查软限位
    double limit = 0;
    if(CheckSoftLimit((ManualMoveDir)dir, phy_axis, this->m_df_phy_axis_pos_feedback[phy_axis])){
        printf("soft limit active, manual move abs return \n");
        return;
    }else if(GetSoftLimt((ManualMoveDir)dir, phy_axis, limit) && dir == DIR_POSITIVE && tar_pos > limit * 1e7){
        tar_pos = limit * 1e7;
    }else if(GetSoftLimt((ManualMoveDir)dir, phy_axis, limit) && dir == DIR_NEGATIVE && tar_pos < limit * 1e7){
        tar_pos = limit * 1e7;
    }
    int64_t n_inc_dis = tar_pos - m_p_channel_control[chn].GetAxisCurIntpTarPos(chn_axis, true)*1e7;
    if((GetMlkMask() & (0x01<<chn_axis)) && m_p_channel_control[chn].GetChnWorkMode() == MANUAL_STEP_MODE){
        n_inc_dis = m_p_channel_control[chn].GetCurManualStep()*1e4*dir;
    }

    //	m_channel_status.cur_manual_move_dir = dir;   //保存方向

    PmcCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(PmcCmdFrame));

    cmd.data.axis_index = phy_axis+1;   //轴号从1开始
    cmd.data.axis_index |= 0xFF00;      //标志通道引擎
    cmd.data.cmd = 0x100;   //增量位置

    //设置速度
    uint32_t feed = this->m_p_axis_config[phy_axis].manual_speed*1000/60;   //转换单位为um/s
    if(this->m_p_channel_control[0].IsRapidManualMove()){
        feed *= 2;  //速度翻倍
        printf("double manual feed\n");
    }
    memcpy(&cmd.data.data[1], &n_inc_dis, sizeof(n_inc_dis));  //设置增量目标位置
    memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //设置速度

    this->m_n_run_axis_mask = 0x01L<<phy_axis;  //设置当前运行轴

    this->m_p_mi_comm->SendPmcCmd(cmd);

    printf("manual move pmc: axis = %d, tar_pos = %lld\n", phy_axis, tar_pos);
}

/**
 * @brief 手动移动
 */
void ChannelEngine::ManualMovePmc(int8_t dir){

    this->ManualMovePmc(this->m_n_cur_pmc_axis, dir);
}

/**
 * @brief 指定轴以指定速度移动到指定位置
 * param phy_axis : 指定的物理轴， 从0开始
 * param tar_pos : 目标位置，单位：mm
 * param vel : 目标速度， 单位：mm/min
 * param inc : true--增量式    false--绝对式
 *
 */
void ChannelEngine::ManualMovePmc(uint8_t phy_axis, double pos, double vel, bool inc){
    ManualMoveDir dir = DIR_POSITIVE;  //默认正向
    if((!inc && pos - this->GetPhyAxisMachPosFeedback(phy_axis) < 0) ||
            (inc && pos < 0))
        dir = DIR_NEGATIVE;

    int64_t cur_pos = this->GetPhyAxisMachPosFeedback(phy_axis) * 1e7;;
    int64_t tar_pos;
    if(inc){
        tar_pos = cur_pos + pos * 1e7;
    }else{
        tar_pos = pos * 1e7;
    }

    //检查硬限位
    if(CheckAxisHardLimit(phy_axis, dir)){   //硬限位告警，直接返回
        printf("hard limit active, manual move pmc return \n");
        return;
    }
    //检查软限位
    double limit = 0;
    if(CheckSoftLimit(dir, phy_axis, this->GetPhyAxisMachPosFeedback(phy_axis))){
        printf("soft limit active, manual move abs return \n");
        return;
    }else if(GetSoftLimt(dir, phy_axis, limit) && dir == DIR_POSITIVE && tar_pos > limit * 1e7){
        tar_pos = limit * 1e7;
    }else if(GetSoftLimt(dir, phy_axis, limit) && dir == DIR_NEGATIVE && tar_pos < limit * 1e7){
        tar_pos = limit * 1e7;
    }

    if(inc)
        tar_pos = tar_pos - cur_pos;

    PmcCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(PmcCmdFrame));

    cmd.data.axis_index = phy_axis+1;   //轴号从1开始
    cmd.data.axis_index |= 0xFF00;      //标志通道引擎
    cmd.data.cmd = inc?0x100:0;   //绝对位置/增量位置

    //设置速度
    uint32_t feed = vel*1000/60;   //转换单位为um/s
    memcpy(&cmd.data.data[1], &pos, sizeof(tar_pos));  //设置目标位置
    memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //设置速度

    this->m_n_run_axis_mask = 0x01L<<phy_axis;  //设置当前运行轴

    this->m_p_mi_comm->SendPmcCmd(cmd);

    printf("manual move pmc vel: axis = %d, tar_pos = %lf, vel = %lf\n", phy_axis, pos, vel);
}

/**
 * @brief 手动停止
 */
void ChannelEngine::ManualMovePmcStop(){
    if(this->m_p_channel_control[0].GetChnWorkMode() == MANUAL_STEP_MODE){  //手动单步模式，不做响应，退出
        return;
    }
    printf("send manualmove pmc stop to mi \n");

#ifdef USES_PMC_PROCESS
    //PMC控制
    this->m_p_pmc_reg->FReg().bits[0].OUT0 = 0;
    this->m_p_pmc_reg->FReg().bits[0].OUT1 = 0;
    this->m_p_pmc_reg->FReg().bits[0].OUT2 = 0;
    this->m_p_pmc_reg->FReg().bits[0].OUT3 = 0;
    this->m_p_pmc_reg->FReg().bits[0].OUT4 = 0;
    this->m_p_pmc_reg->FReg().bits[0].OUT5 = 0;
    this->m_p_pmc_reg->FReg().bits[0].OUT6 = 0;
    this->m_p_pmc_reg->FReg().bits[0].OUT7 = 0;
#else
    //系统控制
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(MiCmdFrame));

    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = CHANNEL_ENGINE_INDEX;
    cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
    cmd.data.data[0] = 0x20;   //停止当前轴运动，并抛弃当前运动指令

    m_p_mi_comm->WriteCmd(cmd);

    this->m_n_run_axis_mask = 0;
    this->m_n_runover_axis_mask = 0;
#endif
}

/**
 * @brief 暂停PMC轴移动
 * @param phy_axis : 物理轴号, 从0开始
 * @param flag : true--暂停     false--取消暂停
 */
void ChannelEngine::PausePmcAxis(uint8_t phy_axis, bool flag){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(MiCmdFrame));

    uint16_t data = 0x01;
    if(!flag)
        data = 0x10;   //取消暂停
    cmd.data.axis_index = phy_axis+1;
    cmd.data.reserved = CHANNEL_ENGINE_INDEX;
    cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
    cmd.data.data[0] = data;   //暂停当前轴运动

    m_p_mi_comm->WriteCmd(cmd);

    printf("ChannelEngine::PausePmcAxis, phy_axis=%hhu, flag = %hhu\n", phy_axis, flag);
}

/**
 * @brief 手动停止指定轴
 * @param phy_axis ： 物理轴好， 从0开始
 */
void ChannelEngine::ManualMoveStop(uint8_t phy_axis){

    uint8_t chn = 0, chn_axis = 0;
    chn = this->GetAxisChannel(phy_axis, chn_axis);
    if(!GetPmcActive(phy_axis) && chn != CHANNEL_ENGINE_INDEX){
        this->m_p_channel_control[chn].ManualMoveStop(0x01<<chn_axis);
    }else if (!GetPmcActive(phy_axis)) {
        //ScPrintf("ManualMoveStop axis=%u",phy_axis);
        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(MiCmdFrame));

        cmd.data.axis_index = phy_axis+1;
        cmd.data.reserved = chn+1;
        cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
        cmd.data.data[0] = 0x30;   //停止指定轴运动，并抛弃当前运动指令

        m_p_mi_comm->WriteCmd(cmd);

        this->m_n_run_axis_mask &= ~(0x01L<<phy_axis);
        if(this->m_n_run_axis_mask == this->m_n_runover_axis_mask){
            m_n_run_axis_mask = 0;
            m_n_runover_axis_mask = 0;
        }

        //m_error_code = ERR_PMC_IVALID_USED;
        //CreateError(ERR_PMC_IVALID_USED, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, 0xFF);
    }
}

/**
 * @brief PMC轴运行到位
 * @param cmd : MI发送过来的命令帧
 */
void ChannelEngine::PmcAxisRunOver(MiCmdFrame &cmd){
    printf("PmcAxisRunOver: axis = %hhu\n",cmd.data.axis_index);
    uint64_t mask = 0;
    uint8_t chn = cmd.data.reserved-1;

    if((cmd.data.data[0]&0xFF) == 0x01){//G01指令

        memcpy(&mask, &cmd.data.data[1], 4);
        printf("PmcAxisRunOver1: axis_mask = 0x%llx\n",mask);
        if((this->m_n_run_axis_mask & mask) == 0){  //不是通道引擎控制的PMC轴
            if(chn < this->m_p_general_config->chn_count){
                if(this->m_p_channel_control[chn].PmcAxisRunOver(cmd))
                    return;
            }
            return;
        }else if(this->m_n_run_axis_mask == 0)
            return;
    }else{ //单轴指令  G00
        uint8_t phy_axis = cmd.data.axis_index-1;  //物理轴号，从0开始
        mask = 0x01L<<phy_axis;

        printf("PmcAxisRunOver2: axis_mask = 0x%llx\n",mask);
        if((this->m_n_run_axis_mask & mask) == 0){  //不是通道引擎控制的PMC轴
            if(chn < this->m_p_general_config->chn_count){
                if(this->m_p_channel_control[chn].PmcAxisRunOver(cmd))
                    return;
            }
            return;
        }else if(this->m_n_run_axis_mask == 0)
            return;

        if (GetPmcActive(phy_axis)) {
            if(m_pmc_axis_ctrl[m_p_axis_config[phy_axis].axis_pmc-1].GetCmdCount() > 0 &&
                    !m_pmc_axis_ctrl[m_p_axis_config[phy_axis].axis_pmc-1].IsPaused()){   //有数据，非暂停状态
                m_pmc_axis_ctrl[m_p_axis_config[phy_axis].axis_pmc-1].ExecCmdOver(true);
            }
        }

        this->m_n_runover_axis_mask |= mask;

        if(this->m_n_run_axis_mask == this->m_n_runover_axis_mask){  //运行结束
            this->m_n_run_axis_mask = 0;
            this->m_n_runover_axis_mask = 0;

            printf("pmc axis run over\n");
        }
    }

}

/**
 * @brief 检查物理轴限位告警情况
 * @param phy_axis : 物理轴号，由0开始
 * @param dir ： 限位方向, -1表示负向， 1表示正向
 * @return true--触发限位   false--没有触发
 */
bool ChannelEngine::CheckAxisHardLimit(uint8_t phy_axis, int8_t dir){
    printf("cur phy axis: %hhu, dir = %hhu, post_mask = 0x%llx, neg_mask = 0x%llx\n", phy_axis, dir, this->m_hard_limit_postive, m_hard_limit_negative);
    if(this->m_b_ret_ref || this->m_b_ret_ref_auto)   //回参考点时屏蔽硬限位
        return false;
    if(dir == DIR_POSITIVE){
        if(this->m_hard_limit_postive & (0x01<<phy_axis))
            return true;
    }else if(dir == DIR_NEGATIVE){
        if(this->m_hard_limit_negative & (0x01<<phy_axis))
            return true;
    }else{
        return true;
    }

    return false;
}

/**
 * @brief 设置指定轴的软限位开关
 * @param axis : 指定轴号，从0开始
 */
/*
void ChannelEngine::SetAxisSoftLimit(uint8_t axis){
    //发送MC轴软限位数据
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    uint16_t tmp = 0;

    SCAxisConfig &axis_config = this->m_p_axis_config[axis];
    if(axis_config.soft_limit_check_1)
        tmp |= 0x01;
    if(axis_config.soft_limit_check_2)
            tmp |= 0x02;
    if(axis_config.soft_limit_check_3)
            tmp |= 0x04;


    cmd.data.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.data.axis_index = axis+1;
    cmd.data.cmd = CMD_MC_AXIS_ENABLE_SOFT_LIMIT;


    cmd.data.data[0] = tmp;

    m_p_mc_comm->WriteCmd(cmd);
}*/

/**
 * @brief 设置指定轴的软限位值
 * @param axis : 指定轴号，从0开始
 * @param index : 软限位组号，总共3组，0-2
 */
/*
void ChannelEngine::SetAxisSoftLimitValue(uint8_t axis, uint8_t index){
    //发送MC轴软限位数据
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));
    uint16_t tmp = index;

    double max = 0, min = 0;
    if(index == 0){
        max = this->m_p_axis_config[axis].soft_limit_max_1;
        min = this->m_p_axis_config[axis].soft_limit_min_1;
    }else if(index ==1){
        max = this->m_p_axis_config[axis].soft_limit_max_2;
        min = this->m_p_axis_config[axis].soft_limit_min_2;
    }else if(index ==2){
        max = this->m_p_axis_config[axis].soft_limit_max_3;
        min = this->m_p_axis_config[axis].soft_limit_min_3;
    }

    cmd.data.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.data.axis_index = axis+1;
    cmd.data.cmd = CMD_MC_SET_AXIS_SOFT_LIMIT;


    cmd.data.data[0] = tmp;


    int32_t softlimit = max*1e3;   //单位由mm转换为1um
    cmd.data.data[1] = (softlimit & 0xFFFF);
    cmd.data.data[2] = ((softlimit>>16) & 0xFFFF);

    softlimit = min*1e3;   //单位由mm转换为1um
    cmd.data.data[3] = (softlimit & 0xFFFF);
    cmd.data.data[4] = ((softlimit>>16) & 0xFFFF);

    m_p_mc_comm->WriteCmd(cmd);
}*/

/**
 * @brief 开始模块升级操作
 */
void ChannelEngine::StartUpdateProcess(){
    //创建升级线程
    int res = 0;
    pthread_attr_t attr;
    struct sched_param param;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setstacksize(&attr, kThreadStackSize);	//
    param.__sched_priority = 30; //90;
    pthread_attr_setschedparam(&attr, &param);
    //	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
    //	if (res) {
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "模块升级处理线程设置线程继承模式失败！");
    //		m_error_code = ERR_INIT_UPDATE;
    //		goto END;
    //	}
    res = pthread_create(&m_thread_update, &attr,
                         ChannelEngine::UpdateThread, this);    //开启模块升级运行线程
    if (res != 0) {
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "模块升级处理线程创建失败!");
        m_error_code = ERR_INIT_UPDATE;
        CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
        goto END;
    }

END:
    pthread_attr_destroy(&attr);
}

/**
 * @brief 模块升级执行线程函数
 * @param void *args: ChannelEngine对象指针
 */
void *ChannelEngine::UpdateThread(void *args){
    printf("start module update thread, threadid = %ld!\n", syscall(SYS_gettid));
    ChannelEngine *p_chn_engine = static_cast<ChannelEngine *>(args);

    int res = ERR_NONE;
    res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
    if (res != ERR_NONE) {
        printf("Quit ChannelEngine::UpdateThread with error 1!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }
    res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
    if (res != ERR_NONE) {
        printf("Quit ChannelEngine::UpdateThread with error 2!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }

    res = p_chn_engine->UpdateProcess();

    printf("Exit ChannelEngine::UpdateThread!\n");
    pthread_exit(NULL);
}

/**
 * @brief
 * @return
 */
int ChannelEngine::UpdateProcess(){
    int res = ERR_NONE;

    //一键升级
    res = this->UpdateDisk();
    if(res != ERR_NONE)
        goto END;

    //升级SC
    res = this->UpdateSC();
    if(res != ERR_NONE)
        goto END;


    //升级MC
    res = this->UpdateMC();
    if(res != ERR_NONE)
        goto END;

    //升级MI
    //	res = this->UpdateMI();
    res = this->UpdateMI_2();
    if(res != ERR_NONE)
        goto END;

    //升级PMC
    res = this->UpdatePMC();
    if(res != ERR_NONE)
        goto END;

    //升级PL
    res = this->UpdatePL();
    if(res != ERR_NONE)
        goto END;

    //升级SPARTAN
    res = this->UpdateSpartan();
    if(res != ERR_NONE)
        goto END;

    //升级Modbus模块
    res =this->UpdateModbus();
    if(res != ERR_NONE)
        goto END;

END:
    this->SendHmiUpdateStatus(0, 0);//全部模块升级完成

    m_thread_update = 0;
    if(res != ERR_NONE){
        this->m_error_code = static_cast<ErrorType>(res);
        CreateError(res, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
    }
    return res;
}

/**
 * @brief 升级MC模块
 * @return
 */
int ChannelEngine::UpdateMC(){
    //升级MC模块
    int res = ERR_NONE;
    //	int retry = 0;
    uint32_t i = 0;

    struct stat statbuf;
    uint32_t file_size = 0;
    uint32_t send_size = 0, read_size = 0;
    uint16_t file_crc = 0xffff;  //文件crc
    uint16_t file_frame_count = 0;  //总帧数
    GCodeFrame data_frame;     //数据帧
    int fp = 0; //文件句柄
    uint32_t cc = 0;

    //组合升级文件名字
    char filepath[kMaxPathLen] = {0};
    strcpy(filepath, PATH_UPDATE_PATH);
    strcat(filepath, "module_mc.ldr");

    if(access(filepath, F_OK) == -1)	//升级文件不存在，MC模块不需要升级
        return res;

    printf("start to update MC module\n");
    m_n_update_state = MODULE_UPDATE_MC;
    m_n_mc_update_status = 0;  //初始状态为无错误

    this->SendHmiUpdateStatus(5, 0);

    if(!this->SendMcUpdateStartCmd()){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "发送MC模块升级开始命令失败！");
        res = ERR_UPDATE_MC;
        goto END;
    }

    printf("Succeed to send mc update start cmd\n");

    //TODO 清除下行运控数据FIFO中的当前数据

    //等待MC擦除备份区flash
    printf("wait mc to erase flash!\n");
    do{
        if(!QueryMcUpdateStatus()){
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "查询MC模块升级状态失败！");
            res = ERR_UPDATE_MC;
            goto END;
        }
        while(!m_b_get_mc_update_status)
            usleep(100); //等待查询结果

        if(m_n_mc_update_status == 4)
            break;

        usleep(100000);  //等待100ms

    }while(1);

    //传输文件大小
    printf("send file size!\n");
    this->SendHmiUpdateStatus(5, 1);
    if(stat(filepath, &statbuf) == 0)
        file_size = statbuf.st_size;  //获取文件总大小
    else{
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "获取文件[%s]大小失败！", PATH_PMC_DATA);
        res = ERR_UPDATE_MC;
        goto END;		//获取文件大小失败
    }
    if(!this->SendMcUpdateFileSize(file_size)){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "发送升级文件大小失败！");
        res = ERR_UPDATE_MC;
        goto END;
    }

    //发送文件内容
    printf("send file content!\n");
    this->SendHmiUpdateStatus(5, 2);
    fp = open(filepath, O_RDONLY); //只读打开文件
    if(fp < 0){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "打开文件[%s]失败！", filepath);
        res = ERR_UPDATE_MC;
        goto END;//文件打开失败
    }

    //计算文件总帧数
    file_frame_count = file_size/MC_UPDATE_BLOCK_SIZE;
    if(file_size%MC_UPDATE_BLOCK_SIZE)
        file_frame_count++;

    while(file_size > 0){
        bzero(data_frame.data_crc, sizeof(data_frame));
        send_size = file_size>MC_UPDATE_BLOCK_SIZE?MC_UPDATE_BLOCK_SIZE:file_size;
        read_size = read(fp, (void *)&data_frame.data_crc[1], send_size);
        if(read_size != send_size){
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "读取文件[%s]失败，%u, %u！", filepath, send_size, read_size);
            res = ERR_UPDATE_MC;
            goto END;//文件读取失败
        }

        cc = send_size/2;
        if(send_size%2)
            cc++;
        for(i = 0; i < cc; i++){
            file_crc ^= data_frame.data_crc[i+1];  //计算CRC
        }

        while(!this->m_p_mc_comm->WriteGCodeData(0, data_frame)){  //固定用通道0数据通道更新MC
            usleep(100);
        }
        file_size -= send_size;

    }

    //等待数据传输完成
    printf("wait data receive!\n");
    while(this->m_p_mc_comm->ReadGCodeFifoCount(0) > 0)
        usleep(10000);

    printf("send crc, block = %hu, crc = 0x%hx\n", file_frame_count, file_crc);
    this->SendHmiUpdateStatus(5, 3);
    this->SendMcUpdateFileCrc(file_frame_count, file_crc);

    //查询MC状态，等待MC升级接收完成
    printf("wait update result!\n");
    do{
        if(!QueryMcUpdateStatus()){
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "查询MC模块升级状态失败！");
            res = ERR_UPDATE_MC;
            goto END;
        }
        while(!m_b_get_mc_update_status)
            usleep(100); //等待查询结果

        if(m_n_mc_update_status == 0x02){ //报错
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MC升级失败！");
            res = ERR_UPDATE_MC;
            goto END;
        }
        else if(m_n_mc_update_status == 0x13){ //CRC校验错
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MC升级CRC校验失败！");
            res = ERR_UPDATE_MC;
            goto END;
        }
        else if(m_n_mc_update_status == 0x11){//升级成功
            this->SendHmiUpdateStatus(5, 4);
            printf("Succeed to update the MC module!\n");
            break;
        }else
            usleep(100000);  //等待100ms

    }while(1);




END:
    if(fp > 0)
        close(fp);
    if(-1 == remove(filepath)){
        //删除升级文件失败
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MC模块删除升级文件失败！");
    }

    if(res != ERR_NONE)
        this->SendHmiUpdateStatus(4, 0xFF);

    m_n_update_state = 0;
    m_n_update_state = MODULE_UPDATE_NONE;
    return res;
}

/**
 * @brief 升级MI模块
 * @return
 */
int ChannelEngine::UpdateMI(){
    //升级MI模块
    int res = ERR_NONE;

    char cmd_buf[256];

    //组合升级文件名字
    char filepath[kMaxPathLen] = {0};
    strcpy(filepath, PATH_UPDATE_PATH);
    strcat(filepath, "module_mi.bin");

    if(access(filepath, F_OK) == -1)	//升级文件不存在，MI模块不需要升级
        return res;

    printf("start to update mi module\n");
    m_n_update_state = MODULE_UPDATE_MI;

    this->SendHmiUpdateStatus(3, 0);

    printf("WAIT ERASE MI FLASH...\n");

    int ret = system("flash_erase /dev/mtd0 0 1");

    if(ret == -1){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MI升级擦除Flash失败！errno = %d", errno);
        res = ERR_UPDATE_MI;
        goto END;
    }

    bzero(cmd_buf, 256);
    strcpy(cmd_buf, "dd if=");
    strcat(cmd_buf, filepath);
    strcat(cmd_buf, " of=/dev/mtdblock0");

    printf("WAIT FOR MI UPDATING THE PROGRAM[%s]...\n", cmd_buf);
    this->SendHmiUpdateStatus(3, 1);

    ret = system(cmd_buf);

    if(ret == -1){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MI升级写入数据失败！errno = %d", errno);
        res = ERR_UPDATE_MI;
    }
    else{
        sync();

        this->SendHmiUpdateStatus(3, 2);
        printf("SUCCEED TO UPDATE MI MODULE\n");

    }

END:
    if(-1 == remove(filepath)){
        //删除升级文件失败
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MI模块删除升级文件失败！");
    }
    if(res != ERR_NONE)
        this->SendHmiUpdateStatus(3, 0xFF);

    m_n_update_state = MODULE_UPDATE_NONE;
    return res;
}

/**
 * @brief 升级MI模块，MI文件与boot分开加载
 * @return
 */
int ChannelEngine::UpdateMI_2(){
    //升级MI模块
    int res = ERR_NONE;

    //组合升级文件名字
    char filepath[kMaxPathLen] = {0};
    char filedes[kMaxPathLen] = {0};
    strcpy(filepath, PATH_UPDATE_PATH);
    strcat(filepath, "module_mi.bin");

    if(access(filepath, F_OK) == -1)	//升级文件不存在，MI模块不需要升级
        return res;

    printf("start to update mi module\n");
    m_n_update_state = MODULE_UPDATE_MI;

    this->SendHmiUpdateStatus(3, 0);

    printf("wait copy file for mi module...\n");


    //拷贝MI升级文件
    strcpy(filedes, PATH_MI_PROGRAM);

    int nn = CopyFile(filepath, filedes);
    if(0 != nn){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MI模块拷贝文件失败！errno = %d", nn);
        res = ERR_UPDATE_MI;
        goto END;
    }

    //拷贝备份文件
    this->SendHmiUpdateStatus(3, 1);
    bzero(filedes, kMaxPathLen);
    strcpy(filedes, PATH_MI_PROGRAM_BAK);
    nn = CopyFile(filepath, filedes);
    if(0 != nn){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MI模块拷贝备份文件失败！errno = %d", nn);
        res = ERR_UPDATE_MI;
        goto END;
    }


    sync();
    this->SendHmiUpdateStatus(3, 2);
    printf("SUCCEED TO UPDATE MI MODULE\n");


END:
    if(-1 == remove(filepath)){
        //删除升级文件失败
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MI模块删除升级文件失败！");
    }
    if(res != ERR_NONE)
        this->SendHmiUpdateStatus(3, 0xFF);

    m_n_update_state = MODULE_UPDATE_NONE;
    return res;
}

/**
 * @brief 升级Modbus模块
 * @return
 */
int ChannelEngine::UpdateModbus(){
    //升级Modbus模块
    int res = ERR_NONE;

    //组合升级文件名字
    char filepath[kMaxPathLen] = {0};
    char filedes[kMaxPathLen] = {0};
    strcpy(filepath, PATH_UPDATE_PATH);
    strcat(filepath, "module_modbus.elf");

    if(access(filepath, F_OK) == -1)	//升级文件不存在，Modbus模块不需要升级
        return res;

    printf("start to update Modbus module\n");
    m_n_update_state = MODULE_UPDATE_MODBUS;

    this->SendHmiUpdateStatus(3, 0);

    printf("wait copy file for Modbus module...\n");


    //拷贝Modbus升级文件
    strcpy(filedes, PATH_MODBUS_PROGRAM);

    int nn = CopyFile(filepath, filedes);
    if(0 != nn){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "Modbus模块拷贝文件失败！errno = %d", nn);
        res = ERR_UPDATE_MI;
        goto END;
    }


    sync();
    this->SendHmiUpdateStatus(3, 2);
    printf("SUCCEED TO UPDATE MODBUS MODULE\n");


END:
    if(-1 == remove(filepath)){
        //删除升级文件失败
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MODBUS模块删除升级文件失败！");
    }
    if(res != ERR_NONE)
        this->SendHmiUpdateStatus(3, 0xFF);

    m_n_update_state = MODULE_UPDATE_NONE;
    return res;
}

/**
 * @brief 一键升级
 * @return
 */
int ChannelEngine::UpdateDisk(){
    //升级所有模块
    int res = ERR_NONE;
    int ret;
    char cmd_buf[100];

    //组合升级文件名字
    char filepath[kMaxPathLen] = {0};
    char filedes[kMaxPathLen] = {0};
    strcpy(filepath, PATH_UPDATE_PATH);
    strcat(filepath, "module_disk.zip");

    if(access(filepath, F_OK) == -1)	//升级文件不存在，Modbus模块不需要升级
        return res;

    printf("start to update Disk module\n");
    m_n_update_state = MODULE_UPDATE_DISK;

    this->SendHmiUpdateStatus(3, 0);

    printf("wait copy file for Disk module...\n");

    //创建diskup目录
    ret = system("mkdir -p /cnc/diskup");
    if (ret != 0) {
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "DISK模块创建diskup目录失败！");
        printf("mkdir diskup error, return %d\n", ret);
        res = ERR_UPDATE_DISK;
        goto END;
    }

    //清空原来的文件
    system("rm -rf /cnc/diskup/sc/*");
    system("rm -rf /cnc/diskup/scroot/*");

    //解压到diskup目录
    sprintf(cmd_buf, "unzip %s -o -d /cnc/diskup", filepath);
    ret = system(cmd_buf);
    if (ret != 0) {
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "DISK模块解压升级包失败！");
        printf("unzip %s error, return %d\n", ret);
        res = ERR_UPDATE_DISK;
        goto END;
    }

    this->SendHmiUpdateStatus(3, 1);

    //检测升级脚本是否存在
    if(access(PATH_UPDATE_DISK_CMD, F_OK) == -1)	{
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "DISK模块未发现升级脚本！");
        printf("can't update disk module, no scup.sh\n");
        res = ERR_UPDATE_DISK;
        goto END;
    }

    sprintf(cmd_buf, "chmod a+x %s", PATH_UPDATE_DISK_CMD);
    system(cmd_buf);

    //运行脚本
    ret = system(PATH_UPDATE_DISK_CMD);
    if (ret != 0) {
        printf("scup.sh return low=%d high=%d\n", ret&0xFF, ret>>8);
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "DISK模块运行升级脚本失败！");
        res = ERR_UPDATE_DISK;
        goto END;
    }

    sync();
    this->SendHmiUpdateStatus(3, 2);
    printf("SUCCEED TO UPDATE DISK MODULE\n");

END:
    if(-1 == remove(filepath)){
        //删除升级文件失败
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "DISK模块删除升级文件失败！");
    }
    if(res != ERR_NONE)
        this->SendHmiUpdateStatus(3, 0xFF);

    sync();
    m_n_update_state = MODULE_UPDATE_NONE;
    return res;
}

/**
 * @brief 升级SC模块
 * @return
 */
int ChannelEngine::UpdateSC(){
    //升级SC模块
    int res = ERR_NONE;
    int ret = 0;
    char cmd_buf[256];

    //组合升级文件名字
    char filepath[kMaxPathLen] = {0};
    char filedes[kMaxPathLen] = {0};
    strcpy(filepath, PATH_UPDATE_PATH);
    strcat(filepath, "module_sc.elf");

    if(access(filepath, F_OK) == -1)	//升级文件不存在，SC模块不需要升级
        return res;

    printf("start to update sc module\n");
    m_n_update_state = MODULE_UPDATE_SC;

    this->SendHmiUpdateStatus(4, 0);

    //读取当前运行序号
    char c;
    FILE* fp = fopen(PATH_BOOT_CONFIG_FILE, "rb");
    if (fp != nullptr) {
        fread(&c, 1, 1, fp);
        fclose(fp);
    }else{
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "SC模块升级查询启动配置文件失败！errno = %d", errno);
        res = ERR_UPDATE_SC;
        goto END;
    }

    if (c == '1'){
        c = '2';
        strcpy(filedes, "/cnc/bin/10MA_SC_Module_b.elf");
    }else{
        c = '1';
        strcpy(filedes, "/cnc/bin/10MA_SC_Module_a.elf");
    }

    printf("wait copy file for sc module...\n");
    this->SendHmiUpdateStatus(4, 1);

    //拷贝复制文件
    bzero(cmd_buf, 256);
    strcpy(cmd_buf, "cp ");
    strcat(cmd_buf, filepath);
    strcat(cmd_buf, " ");
    strcat(cmd_buf, filedes);

    ret = system(cmd_buf);

    if(ret == -1){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "SC模块升级写入数据失败！errno = %d", errno);
        res = ERR_UPDATE_SC;
        goto END;
    }

    printf("waiting for changing the file attributes...\n");
    this->SendHmiUpdateStatus(4, 2);
    //修改文件属性
    bzero(cmd_buf, 256);
    strcpy(cmd_buf, "chmod a+x ");
    strcat(cmd_buf, filedes);
    ret = system(cmd_buf);

    if(ret == -1){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "SC模块升级修改文件属性失败！errno = %d", errno);
        res = ERR_UPDATE_SC;
        goto END;
    }
    else{
        sync();
    }

    //修改启动序号
    fp = fopen(PATH_BOOT_CONFIG_FILE, "wb");
    if(fp != nullptr) {
        fwrite(&c, 1, 1, fp);
        sync();
        fclose(fp);
        printf("succeed to update sc module\n");

    }else{
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "SC模块升级配置启动文件失败！errno = %d", errno);
        res = ERR_UPDATE_SC;
    }
    this->SendHmiUpdateStatus(4, 3);

END:
    if(-1 == remove(filepath)){
        //删除升级文件失败
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MC模块删除升级文件失败！");
    }
    if(res != ERR_NONE)
        this->SendHmiUpdateStatus(4, 0xFF);

    m_n_update_state = MODULE_UPDATE_NONE;
    return res;
}

/**
 * @brief 升级PL模块
 * @return
 */
int ChannelEngine::UpdatePL(){
    //升级PL模块
    int res = ERR_NONE;

    char cmd_buf[256];

    //组合升级文件名字
    char filepath[kMaxPathLen] = {0};
    strcpy(filepath, PATH_UPDATE_PATH);
    strcat(filepath, "module_pl.bin");

    if(access(filepath, F_OK) == -1)	//升级文件不存在，PL模块不需要升级
        return res;
    printf("start to update PL module\n");
    m_n_update_state = MODULE_UPDATE_PL;

    this->SendHmiUpdateStatus(3, 0);

    printf("WAIT ERASE BOOT FLASH...\n");

    int ret = system("flash_erase /dev/mtd0 0 1");

    if(ret == -1){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PL升级擦除Flash失败！errno = %d", errno);
        res = ERR_UPDATE_PL;
        goto END;
    }

    bzero(cmd_buf, 256);
    strcpy(cmd_buf, "dd if=");
    strcat(cmd_buf, filepath);
    strcat(cmd_buf, " of=/dev/mtdblock0");

    printf("WAIT FOR PL UPDATING THE PROGRAM...\n");
    this->SendHmiUpdateStatus(3, 1);

    ret = system(cmd_buf);

    if(ret == -1){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PL升级写入数据失败！errno = %d", errno);
        res = ERR_UPDATE_PL;
    }
    else{
        sync();

        this->SendHmiUpdateStatus(3, 2);
        printf("SUCCEED TO UPDATE PL MODULE\n");


    }

END:
    if(-1 == remove(filepath)){
        //删除升级文件失败
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PL模块删除升级文件失败！");
    }
    if(res != ERR_NONE)
        this->SendHmiUpdateStatus(3, 0xFF);

    m_n_update_state = MODULE_UPDATE_NONE;
    return res;
}

/**
 * @brief 升级SPARTAN模块
 * @return
 */
int ChannelEngine::UpdateSpartan(){
    //升级SPARTAN模块
    int res = ERR_NONE;

    char cmd_buf[256];

    //组合升级文件名字
    char filepath[kMaxPathLen] = {0};
    strcpy(filepath, PATH_UPDATE_PATH);
    strcat(filepath, "module_spartan.bin");

    if(access(filepath, F_OK) == -1)	//升级文件不存在，MI模块不需要升级
        return res;

    printf("start to update spartan module\n");
    m_n_update_state = MODULE_UPDATE_FPGA;

    this->SendHmiUpdateStatus(3, 0);

    bzero(cmd_buf, 256);
    strcpy(cmd_buf, "mtdwrite /dev/mtd4 ");
    strcat(cmd_buf, filepath);

    printf("WAIT FOR SPARTAN UPDATING THE PROGRAM[%s]...\n", cmd_buf);
    this->SendHmiUpdateStatus(3, 1);

    int ret = system(cmd_buf);

    if(ret != 0){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "SPARTAN升级写入数据失败！errno = %d", errno);
        res = ERR_UPDATE_SPARTAN;
    }
    else{
        sync();

        this->SendHmiUpdateStatus(3, 2);
        printf("SUCCEED TO UPDATE SPARTAN MODULE\n");


    }

    if(-1 == remove(filepath)){
        //删除升级文件失败
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "SPARTAN模块删除升级文件失败！");
    }
    if(res != ERR_NONE)
        this->SendHmiUpdateStatus(3, 0xFF);

    m_n_update_state = MODULE_UPDATE_NONE;
    return res;
}

/**
 * @brief 升级PMC模块
 * @return
 */
int ChannelEngine::UpdatePMC(){
    //升级PMC模块
    int res = ERR_NONE;

    //组合升级文件名字
    char filepath[kMaxPathLen] = {0};

    strcpy(filepath, PATH_UPDATE_PATH);
    strcat(filepath, "module_pmc.upd");


    if(access(filepath, F_OK) == -1)	//升级文件不存在，PMC模块不需要升级
        return res;

    printf("start to update PMC module\n");
    m_n_update_state = MODULE_UPDATE_PMC;

    this->SendHmiUpdateStatus(3, 0);  //总共分三步骤

    FILE *file_src = fopen(filepath, "r+");
    if (nullptr == file_src)
    {
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMC模块升级失败，升级文件打开失败[%s]！errno = %d", filepath, errno);
        return res;
    }

    uint64_t file_len = 0;   //文件长度
    uint64_t total_size = 0;	//升级文件总长度
    fseek(file_src, 0L, SEEK_END);
    total_size = ftell(file_src);   //获取文件长度

    fseek(file_src, 0L, SEEK_SET);   //回到文件头


    //升级梯形图文件
    char buffer[1024] = {0};   //缓冲
    int read_cur_real = 0;		//单次实际读取字节数
    int read_cur_plan = 0;		//单词计划读取字节数
    FILE *file_des = fopen(PATH_PMC_LDR, "w+");
    if (nullptr == file_des)
    {
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMC梯形图文件创建失败[%s]！errno = %d", PATH_PMC_LDR, errno);
        res = ERR_UPDATE_PMC;
        goto END;
    }

    fread(&file_len, 8, 1, file_src);
    file_len = BigLittleSwitch64(file_len);

    if(file_len >= total_size){	//文件长度不合法
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "升级文件可能损坏[0x%llx, 0x%llx]！", total_size, file_len);
        res = ERR_UPDATE_PMC;
        fclose(file_des);
        goto END;
    }
    while(file_len > 0 && !feof(file_src))
    {
        read_cur_plan = file_len>1024?1024:file_len;
        read_cur_real = fread(buffer, 1, read_cur_plan, file_src);
        if(read_cur_plan != read_cur_real){	//错误
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMC模块升级，复制梯形图文件出错1！%llu, %d, %d", file_len, read_cur_plan,
                                    read_cur_real);
            res = ERR_UPDATE_PMC;
            fclose(file_des);
            goto END;
        }else{
            fwrite(buffer, read_cur_real, 1, file_des);
            file_len -= read_cur_real;
        }

        memset(buffer, 0, 1024);
    }

    if(file_len != 0){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMC模块升级，复制梯形图文件出错2！");
        res = ERR_UPDATE_PMC;
        fclose(file_des);
        goto END;
    }


    fclose(file_des); 	//赋值梯形图文件成功
    sync();
    printf("Succeed to copy ldr file!\n");

    this->SendHmiUpdateStatus(3, 1);  //总共分三步骤

    //升级PMC执行文件
    file_des = fopen(PATH_PMC_DATA, "w+");
    if (nullptr == file_des)
    {
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMC执行文件创建失败[%s]！errno = %d", PATH_PMC_DATA, errno);
        res = ERR_UPDATE_PMC;
        goto END;
    }
    fread(buffer, 6, 1, file_src);
    if(strcmp(buffer, "#DAT@#") != 0){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "升级文件非法，请检查！");
        fclose(file_des);
        res = ERR_UPDATE_PMC;
        goto END;
    }
    fread(&file_len, 8, 1, file_src);
    file_len = BigLittleSwitch64(file_len);

    printf("data file length: %llu\n", file_len);

    memset(buffer, 0, 1024);
    while(file_len > 0 && !feof(file_src))
    {
        read_cur_plan = file_len>1024?1024:file_len;
        read_cur_real = fread(buffer, 1, read_cur_plan, file_src);
        if(read_cur_plan != read_cur_real){	//错误
            g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMC模块升级，复制执行文件出错1！");
            res = ERR_UPDATE_PMC;
            fclose(file_des);
            goto END;
        }else{
            fwrite(buffer, read_cur_real, 1, file_des);
            file_len -= read_cur_real;
        }

        memset(buffer, 0, 1024);
    }

    if(file_len != 0){
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMC模块升级，复制执行文件出错2！");
        res = ERR_UPDATE_PMC;
        fclose(file_des);
        goto END;
    }

    fclose(file_des);

    this->SendHmiUpdateStatus(3, 2);  //总共分三步骤

    printf("Succeed to update pmc module!\n");
END:
    fclose(file_src);
    if(-1 == remove(filepath)){
        //删除升级文件失败
        g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMC模块删除升级文件失败！");
    }
    sync();

    if(res != ERR_NONE)
        this->SendHmiUpdateStatus(3, 0xFF);
    return res;
}

/**
 * @brief 发送MC升级开始指令
 * @return true--成功   false--失败
 */
bool ChannelEngine::SendMcUpdateStartCmd(){
    //发送MC升级开始指令
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_UPDATE_START;

    return m_p_mc_comm->WriteCmd(cmd);
}

/**
 * @brief 发送MC升级文件大小
 * @param size : 升级文件16bits双字节数量
 * @return true--成功   false--失败
 */
bool ChannelEngine::SendMcUpdateFileSize(uint32_t size){
    //发送MC升级文件大小
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_UPDATE_FILE_SIZE;

    cmd.data.data[0] = (size & 0x0000FFFF);
    cmd.data.data[1] = ((size>>16) & 0x0000FFFF);

    return m_p_mc_comm->WriteCmd(cmd);
}

/**
 * @brief 发送MC升级文件CRC
 * @param frame_count :  发送帧总数
 * @param crc : 升级文件整体CRC
 * @return true--成功   false--失败
 */
bool ChannelEngine::SendMcUpdateFileCrc(uint16_t frame_count, uint16_t crc){
    //发送MC升级文件CRC
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_UPDATE_CRC;

    cmd.data.data[0] = frame_count;
    cmd.data.data[1] = crc;

    return m_p_mc_comm->WriteCmd(cmd);
}

/**
 * @brief 查询MC升级状态
 * @return
 */
bool ChannelEngine::QueryMcUpdateStatus(){
    //查询MC升级状态
    McCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(McCmdFrame));

    cmd.data.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MC_GET_UPDATE_STATE;

    m_b_get_mc_update_status = false;

    return m_p_mc_comm->WriteCmd(cmd);
}

/**
 * @brief 获取实际总线轴数量
 * @return
 */
uint16_t ChannelEngine::GetBusAxisCount(){
    uint16_t count = 0;

    for(int i = 0; i < this->m_p_general_config->axis_count; i++){
        if(m_p_axis_config[i].axis_interface == BUS_AXIS){
            count++;
        }
    }

    return count;
}

/**
 * @brief 初始化PMC的非易失性寄存器
 */
void ChannelEngine::InitPmcReg(){
    printf("init pmc register\n");
    //	//打开文件
    //	int fp = open(PATH_PMC_REG, O_RDONLY); //打开文件
    //
    //	if(fp < 0){
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "打开pmc寄存器保存文件失败！");
    //		return;//文件打开失败
    //	}
    //	uint16_t size = 0;   //寄存器字节数
    //
    //	//读取K寄存器
    //	ssize_t read_size = read(fp, &size, 2);
    //	if(read_size != 2){ //读取失败
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "读取K寄存器字节数失败！");
    //		return;
    //	}
    //
    //	if(size != K_REG_COUNT){
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "K寄存器大小数不匹配[%hu,%hu]！", size, K_REG_COUNT);
    //		return;
    //	}
    //	read_size = read(fp, this->m_p_pmc_reg->GetRegPtr8(PMC_REG_K), size);
    //	if(read_size != size){
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "读取K寄存器数据失败[%hu,%hu]！", read_size, size);
    //		return;
    //	}
    //
    //	//读取D寄存器
    //	read_size = read(fp, &size, 2);
    //	if(read_size != 2){ //读取失败
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "读取D寄存器字节数失败！");
    //		return;
    //	}
    //
    //	if(size != D_REG_COUNT*2){
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "K寄存器大小数不匹配[%hu,%hu]！", size, D_REG_COUNT*2);
    //		return;
    //	}
    //	read_size = read(fp, this->m_p_pmc_reg->GetRegPtr16(PMC_REG_D), size);
    //	if(read_size != size){
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "读取D寄存器数据失败[%hu,%hu]！", read_size, size);
    //		return;
    //	}
    //
    //	//读取DC寄存器
    //	read_size = read(fp, &size, 2);
    //	if(read_size != 2){ //读取失败
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "读取DC寄存器字节数失败！");
    //		return;
    //	}
    //
    //	if(size != C_REG_COUNT*2){
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "DC寄存器大小数不匹配[%hu,%hu]！", size, C_REG_COUNT*2);
    //		return;
    //	}
    //	read_size = read(fp, this->m_p_pmc_reg->GetRegPtr16(PMC_REG_DC), size);
    //	if(read_size != size){
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "读取DC寄存器数据失败[%hu,%hu]！", read_size, size);
    //		return;
    //	}
    //
    //	//读取DT寄存器
    //	read_size = read(fp, &size, 2);
    //	if(read_size != 2){ //读取失败
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "读取DT寄存器字节数失败！");
    //		return;
    //	}
    //
    //	if(size != T_REG_COUNT*2){
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "DT寄存器大小数不匹配[%hu,%hu]！", size, T_REG_COUNT*2);
    //		return;
    //	}
    //	read_size = read(fp, this->m_p_pmc_reg->GetRegPtr16(PMC_REG_DT), size);
    //	if(read_size != size){
    //		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "读取DT寄存器数据失败[%hu,%hu]！", read_size, size);
    //		return;
    //	}


    //将数据发送给MI
    uint16_t i = 0, count = 0;
    uint8_t *pn8 = nullptr;

    //发送K寄存器
    pn8 = this->m_p_pmc_reg->GetRegPtr8(PMC_REG_K);
    for(i = 0; i < K_REG_COUNT; i++){
        if(pn8[i] != 0){
            this->SendPmcRegValue(PMC_REG_K, i, pn8[i]);
            //printf("Init K reg k%hu = %hhu\n", i, pn8[i]);
        }
    }

#ifndef USES_PMC_2_0
    uint16_t *pn16 = nullptr;
    //发送D寄存器
    pn16 = this->m_p_pmc_reg->GetRegPtr16(PMC_REG_D);
    for(i = 0; i < D_REG_COUNT; i++){
        if(pn16[i] != 0){
            this->SendPmcRegValue(PMC_REG_D, i, pn16[i]);
            //printf("Init D reg d%hu = %hu\n", i, pn16[i]);
        }
    }

    //发送DC寄存器
    pn16 = this->m_p_pmc_reg->GetRegPtr16(PMC_REG_DC);
    for(i = 0; i < C_REG_COUNT; i++){
        if(pn16[i] != 0){
            this->SendPmcRegValue(PMC_REG_DC, i, pn16[i]);
            //printf("Init DC reg dc%hu = %hu\n", i, pn16[i]);
        }
    }

    //发送DT寄存器
    pn16 = this->m_p_pmc_reg->GetRegPtr16(PMC_REG_DT);
    for(i = 0; i < T_REG_COUNT; i++){
        if(pn16[i] != 0){
            this->SendPmcRegValue(PMC_REG_DT, i, pn16[i]);
            //printf("Init DT reg dt%hu = %hu\n", i, pn16[i]);
        }
    }

    //发送C寄存器
    pn16 = this->m_p_pmc_reg->GetRegPtr16(PMC_REG_C);
    for(i = 0; i < C_REG_COUNT; i++){
        if(pn16[i] != 0){
            this->SendPmcRegValue(PMC_REG_C, i, pn16[i]);
            //	printf("Init C reg c%hu = %hu\n", i, pn16[i]);
        }
    }
#else
    //发送D寄存器
    pn8 = this->m_p_pmc_reg->GetRegPtr8(PMC_REG_D);
    for(i = 0; i < D_REG_COUNT; i++){
        if(pn8[i] != 0){
            this->SendPmcRegValue(PMC_REG_D, i, pn8[i]);
            //printf("Init D reg d%hu = %hu\n", i, pn16[i]);
        }
    }

    //发送C寄存器
    pn8 = this->m_p_pmc_reg->GetRegPtr8(PMC_REG_C);
    count = C_REG_COUNT*4;
    for(i = 0; i < count; i++){
        if(pn8[i] != 0){
            this->SendPmcRegValue(PMC_REG_C, i, pn8[i]);
            //	printf("Init C reg c%hu = %hu\n", i, pn8[i]);
        }
    }

    //发送T寄存器
    pn8 = this->m_p_pmc_reg->GetRegPtr8(PMC_REG_T);
    count = T_REG_COUNT*2;
    for(i = 0; i < count; i++){
        if(pn8[i] != 0){
            this->SendPmcRegValue(PMC_REG_T, i, pn8[i]);
            //	printf("Init T reg c%hu = %hu\n", i, pn8[i]);
        }
    }


#endif
}

/**
 * @brief 给MI发送初始化参数
 */
void ChannelEngine::InitMiParam(){

    printf("start init mi parameters\n");
    //	uint8_t mi_axis_interface[] = {0xFF, 0x10, 0x01};   //虚拟轴、总线轴、非总线轴

    //初始化PMC寄存器
    this->InitPmcReg();

    //初始化SD-LINK从站数据
    SetSlaveInfo();

    //发送手轮编码格式
   m_p_mi_comm->SendMiParam<uint8_t>(0xFF, 10, this->m_p_general_config->hw_code_type);   //手轮编码类型

    //发送IO重映射数据
    ListNode<IoRemapInfo> *info_node = this->m_p_io_remap->HeadNode();
    while(info_node != nullptr){
        this->SendMiIoRemapInfo(info_node->data);

        info_node = info_node->next;
    }

    //所有轴依次发送
    SCAxisConfig *axis_config = nullptr;
    uint8_t index = 0;
    double tmp = 0.0;    //临时变量
    uint8_t tmp_8 = 0;   //临时变量
    uint64_t tmp_64 = 0;   //临时变量
    for(int i = 0; i < this->m_p_general_config->axis_count; i++){
        axis_config = &m_p_axis_config[i];
        index = i+1;   //MI中的轴号从1开始

        //发送插补后加减速相关参数
       m_p_mi_comm->SendMiParam<uint8_t>(index, 1140, axis_config->post_filter_type);   //滤波器类型
       m_p_mi_comm->SendMiParam<uint16_t>(index, 1141, axis_config->post_filter_time_1);   //一级滤波器时间常数
       m_p_mi_comm->SendMiParam<uint16_t>(index, 1142, axis_config->post_filter_time_2);   //二级滤波器时间常数

       m_p_mi_comm->SendMiParam<uint8_t>(index, 1002, axis_config->axis_interface);   //轴接口类型
       m_p_mi_comm->SendMiParam<uint8_t>(index, 1001, axis_config->axis_type);		//轴类型
       m_p_mi_comm->SendMiParam<uint8_t>(index, 1003, axis_config->axis_port);		//从站号或者对应轴口号
       //m_p_mi_comm->SendMiParam<uint8_t>(index, 1006, axis_config->axis_pmc);			//是否PMC轴
       m_p_mi_comm->SendMiParam<uint8_t>(index, 1006, 0);                               //PMC轴由梯图来激活
       m_p_mi_comm->SendMiParam<double>(index, 1100, axis_config->kp1);			//kp1
       m_p_mi_comm->SendMiParam<double>(index, 1101, axis_config->kp2);			//kp2
       m_p_mi_comm->SendMiParam<double>(index, 1102, axis_config->ki);			//ki
       m_p_mi_comm->SendMiParam<double>(index, 1103, axis_config->kil);			//kil
       m_p_mi_comm->SendMiParam<double>(index, 1104, axis_config->kd);			//kd
        tmp = static_cast<double>(axis_config->kvff);
       m_p_mi_comm->SendMiParam<double>(index, 1105, tmp);			//kvff
        tmp = static_cast<double>(axis_config->kaff);
       m_p_mi_comm->SendMiParam<double>(index, 1106, tmp);			//kaff
        tmp_64 = static_cast<uint64_t>(axis_config->track_err_limit) * 1e4;   //转换单位：um-->0.1nm
       m_p_mi_comm->SendMiParam<uint64_t>(index, 1107, tmp_64);	//跟踪误差限
        tmp_64 = static_cast<uint64_t>(axis_config->location_err_limit) * 1e4;   //转换单位：um-->0.1nm
       m_p_mi_comm->SendMiParam<uint64_t>(index, 1108, tmp_64);			//定位误差限
       m_p_mi_comm->SendMiParam<uint32_t>(index, 1200, axis_config->motor_count_pr);	//电机每转计数
       m_p_mi_comm->SendMiParam<uint32_t>(index, 1201, axis_config->motor_speed_max);	//电机最大转速
        tmp_64 = static_cast<uint64_t>(axis_config->move_pr * 1e7);   //转换单位：mm-->0.1nm
        // 		printf("write axis param idx = %d, value = %ld\n", index, tmp_64);
       m_p_mi_comm->SendMiParam<uint64_t>(index, 1202, tmp_64);	//每转移动量
       m_p_mi_comm->SendMiParam<uint8_t>(index, 1203, axis_config->motor_dir);	//电机旋转方向
        tmp_8 = axis_config->feedback_mode;
        //		if(axis_config->axis_interface == VIRTUAL_AXIS)
        //			tmp_8 = 0xFF;    //虚拟轴
        //		else if(axis_config->axis_interface == BUS_AXIS)
        //			tmp_8 = 0x10;   //ethercat
       m_p_mi_comm->SendMiParam<uint8_t>(index, 1204, tmp_8);	//反馈类型
        //		printf("send axis[%hhu] feedback_mode %hhu #####\n", index, tmp_8);

        tmp_8 = axis_config->ctrl_mode;
        if(axis_config->axis_interface == VIRTUAL_AXIS)
            tmp_8 = 0xFF;    //虚拟轴
        else if(axis_config->axis_interface == BUS_AXIS)
            tmp_8 = 0x10;   //ethercat
       m_p_mi_comm->SendMiParam<uint8_t>(index, 1205, tmp_8);	//轴控制方式
       m_p_mi_comm->SendMiParam<uint32_t>(index, 1206, axis_config->pulse_count_pr);	//控制每转输出脉冲数
       m_p_mi_comm->SendMiParam<uint8_t>(index, 1207, axis_config->encoder_lines);	//单圈编码器线数

       m_p_mi_comm->SendMiParam<uint16_t>(index, 1208, axis_config->encoder_max_cycle);   //旋转轴编码器整圈最大值

        //		this->SendMiParam<uint8_t>(index, 1209, axis_config->axis_alarm_level);	//轴告警电平

        if(axis_config->axis_interface != VIRTUAL_AXIS &&
                axis_config->feedback_mode != NO_ENCODER &&
                axis_config->feedback_mode != INCREMENTAL_ENCODER &&
                axis_config->ref_encoder != kAxisRefNoDef){ //写入参考点数据
            this->m_p_mi_comm->SetAxisRef(index, axis_config->ref_encoder);
            printf("send mi ref encoder : %lld\n", axis_config->ref_encoder);
        }

        //软限位
       m_p_mi_comm->SendMiParam<double>(index, 1500, axis_config->soft_limit_max_1);
       m_p_mi_comm->SendMiParam<double>(index, 1501, axis_config->soft_limit_min_1);
       m_p_mi_comm->SendMiParam<uint8_t>(index, 1502, axis_config->soft_limit_check_1);
       m_p_mi_comm->SendMiParam<double>(index, 1503, axis_config->soft_limit_max_2);
       m_p_mi_comm->SendMiParam<double>(index, 1504, axis_config->soft_limit_min_2);
       m_p_mi_comm->SendMiParam<uint8_t>(index, 1505, axis_config->soft_limit_check_2);
       m_p_mi_comm->SendMiParam<double>(index, 1506, axis_config->soft_limit_max_3);
       m_p_mi_comm->SendMiParam<double>(index, 1507, axis_config->soft_limit_min_3);
       m_p_mi_comm->SendMiParam<uint8_t>(index, 1508, axis_config->soft_limit_check_3);

        //发送主轴启动时间、制动时间
       m_p_mi_comm->SendMiParam<uint16_t>(index, 1605, axis_config->spd_start_time);
       m_p_mi_comm->SendMiParam<uint16_t>(index, 1606, axis_config->spd_stop_time);

        //发送反向间隙参数
        this->SendMiBacklash(i);

        //发送轴螺补数据表
        this->SendMiPcData(i);

        //发送轴螺补设置参数
        this->SendMiPcParam(i);
        this->SendMiPcParam2(i);

        //发送轴参考点对应的机械坐标
        this->SendMiRefMachPos(i);
    }

    //发送同步轴相关参数
    m_sync_axis_ctrl->SendMiSyncParams();

    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        this->m_p_channel_control[i].SendMiChnAxisMap();  //发送通道轴物理轴映射关系信息
    }

    this->SendMiPhyAxisEncoder();

    this->SetAxisRetRefFlag();   //发送回参考点结束标志

    //发送轴参数设置完成命令
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_INIT_COMPLETE;
    cmd.data.axis_index = 0xFF;
    this->m_p_mi_comm->WriteCmd(cmd);

    m_b_init_mi_over = true;
    printf("exit init mi parameters\n");

}

/**
 * @brief 向MI发送螺补数据表
 * @param axis : 指定物理轴号，从0开始
 */
void ChannelEngine::SendMiPcData(uint8_t axis){
    if(this->m_p_pc_table == nullptr)
        return;

    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_PC_DATA;
    cmd.data.axis_index = NO_AXIS;

    //放置数据
    uint16_t count = m_p_axis_config[axis].pc_count;
    uint16_t offset = m_p_axis_config[axis].pc_offset-1;  //起始编号，0开始

    //std::cout << "offset :" << offset << std::endl;
    if(this->m_p_axis_config[axis].pc_type == 1){//双向螺补
        count *= 2;   //数据量翻倍
    }

    int32_t value = 0;
    uint8_t dc = 0;    //每个数据包的数据个数

    for(uint16_t i = 0; i < count; i += 3, offset += 3){
        memset(cmd.data.data, 0x00, 14);
        dc = count-i;
        if(dc > 3)
            dc = 3;
        memcpy(&cmd.data.reserved, &dc, 1);   //螺补数据个数

        memcpy(cmd.data.data, &offset, 2);  //起始编号

        //第一组数据
        value = this->m_p_pc_table->pc_table[axis][i] * 1e7;   //单位0.1nm
        memcpy(&cmd.data.data[1], &value, 4);   //起始编号

        //第二组数据
        if(dc>=2){
            value = this->m_p_pc_table->pc_table[axis][i+1] * 1e7;   //单位0.1nm
            memcpy(&cmd.data.data[3], &value, 4);   //起始编号
        }

        //第三组数据
        if(dc == 3){
            value = this->m_p_pc_table->pc_table[axis][i+2] * 1e7;   //单位0.1nm
            memcpy(&cmd.data.data[5], &value, 4);   //起始编号
        }

        this->m_p_mi_comm->WriteCmd(cmd);
    }
}

/**
 * @brief 向MI发送指定轴螺补参数设置
 * @param axis : 指定物理轴号，从0开始
 */
void ChannelEngine::SendMiPcParam(uint8_t axis){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_PC_PARAM;

    cmd.data.axis_index = axis+1;

    //放置数据
    uint16_t count = m_p_axis_config[axis].pc_count;
    uint16_t offset = m_p_axis_config[axis].pc_offset-1;  //起始编号，0开始
    uint32_t inter = m_p_axis_config[axis].pc_inter_dist*1000;   //转换为微米单位
    uint16_t ref_index = m_p_axis_config[axis].pc_ref_index-1;   //参考点对应位置，0开始
    //uint16_t ref_index = 0;   //参考点对应位置，0开始
    uint16_t pc_type = this->m_p_axis_config[axis].pc_type;
    uint16_t pc_enable = m_p_axis_config[axis].pc_enable;

    //std::cout << "SendMiPcParam() " << std::endl;
    //std::cout << "axis: " << (int)cmd.data.axis_index << std::endl;
    //std::cout << "count: " << (int)count << std::endl;
    //std::cout << "offset: " << (int)offset << std::endl;
    //std::cout << "inter: " << (int)inter << std::endl;
    //std::cout << "ref_index: " << (int)ref_index << std::endl;
    //std::cout << "pc_type: " << (int)pc_type << std::endl;
    //std::cout << "pc_enable: " << (int)pc_enable << std::endl;

    memcpy(cmd.data.data, &count, 2);  //补偿数据个数
    memcpy(&cmd.data.data[1], &offset, 2);   //起始编号
    memcpy(&cmd.data.data[2], &inter, 4); 	 //补偿间隔 ， um单位，32位整型
    memcpy(&cmd.data.data[4], &ref_index, 2);     //参考点对应位置
    memcpy(&cmd.data.data[5], &pc_type, 1);     //补偿类型  0--单向螺补   1--双向螺补
    memcpy(&cmd.data.data[6], &pc_enable, 1);
    this->m_p_mi_comm->WriteCmd(cmd);
}


/**
 * @brief 向MI发送指定轴的参考点位置机械坐标
 * @param axis_index : 物理轴号，从0开始
 */
void ChannelEngine::SendMiRefMachPos(uint8_t axis_index){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_REF_MACH_POS;

    cmd.data.axis_index = axis_index+1;

    //放置数据
    uint32_t pos = 0;
    for(uint16_t i = 0; i <10; i++){
        pos = m_p_axis_config[axis_index].axis_home_pos[i]*1000;   //转换为微米单位
        memcpy(cmd.data.data, &pos, 4);  //
        cmd.data.data[2] = i;

        this->m_p_mi_comm->WriteCmd(cmd);
    }
}

/**
 * @brief 向MI发送指定轴螺补参数设置2
 * @param axis : 物理轴号，从0开始
 */
void ChannelEngine::SendMiPcParam2(uint8_t axis){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_PC_PARAM2;

    cmd.data.axis_index = axis+1;

    //放置数据
    //int32_t pos = m_p_axis_config[axis].axis_home_pos[0]*1000;   //转换为微米单位
    int32_t pos = 0;//固定传0，传非零测试出问题，暂时规避
    memcpy(cmd.data.data, &pos, 4);  //

    this->m_p_mi_comm->WriteCmd(cmd);

}

/**
 * @brief 向MI发送设置IO重定向的配置
 * @param info
 */
void ChannelEngine::SendMiIoRemapInfo(IoRemapInfo &info){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_IO_REMAP;

    cmd.data.axis_index = 0xFF;

    //放置数据
    cmd.data.data[0] = info.iotype;
    cmd.data.data[1] = info.addr;
    cmd.data.data[2] = info.bit;
    cmd.data.data[3] = info.addrmap;
    cmd.data.data[4] = info.newaddr;
    cmd.data.data[5] = info.newbit;
    cmd.data.data[6] = info.valtype;

    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 向MI发送指定轴的滤波器相关参数
 * @param phy_axis : 物理轴号，从0开始
 */
void ChannelEngine::SendMiAxisFilterParam(uint8_t phy_axis){
   m_p_mi_comm->SendMiParam<uint8_t>(phy_axis+1, 1140, m_p_axis_config[phy_axis].post_filter_type);   //滤波器类型
   m_p_mi_comm->SendMiParam<uint16_t>(phy_axis+1, 1141, m_p_axis_config[phy_axis].post_filter_time_1);   //一级滤波器时间常数
   m_p_mi_comm->SendMiParam<uint16_t>(phy_axis+1, 1142, m_p_axis_config[phy_axis].post_filter_time_2);   //二级滤波器时间常数
}

/**
 * @brief 向MI发送指定轴的反向间隙参数
 * @param axis : 物理轴号，从0开始
 */
void ChannelEngine::SendMiBacklash(uint8_t axis){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_AXIS_BACKLASH;

    cmd.data.axis_index = axis+1;

    //放置数据
    if(m_p_axis_config[axis].backlash_enable){
        int32_t data = m_p_axis_config[axis].backlash_forward * 1000;
        memcpy(cmd.data.data, &data, 4);
        data = m_p_axis_config[axis].backlash_negative * 1000;
        memcpy(&cmd.data.data[2], &data, 4);
        data = m_p_axis_config[axis].backlash_step;
        memcpy(&cmd.data.data[4], &data, 4);
        uint16_t enable = m_p_axis_config[axis].backlash_enable;
        memcpy(&cmd.data.data[6], &enable, 2);
    }else{
        int32_t data = 0.0;
        memcpy(cmd.data.data, &data, 4);
        data = 0.0;
        memcpy(&cmd.data.data[2], &data, 4);
        data = 0.0;
        memcpy(&cmd.data.data[4], &data, 4);
        uint16_t enable = 0;
        memcpy(&cmd.data.data[6], &enable, 2);
    }
    //std::cout << "axis: " << (int)cmd.data.axis_index << std::endl;
    //std::cout << "backlash_enable: " << (int)m_p_axis_config[axis].backlash_enable << std::endl;
    //std::cout << "backlash_forward: " << (double)m_p_axis_config[axis].backlash_forward << std::endl;
    //std::cout << "backlash_negative: " << (double)m_p_axis_config[axis].backlash_negative << std::endl;
    //std::cout << "backlash_step: " << (int)m_p_axis_config[axis].backlash_step << std::endl;
    this->m_p_mi_comm->WriteCmd(cmd);
}


/**
 * @brief 向MI设置PMC寄存器
 * @param sec :
 * @param index
 * @param data
 */
void ChannelEngine::SendPmcRegValue(PmcRegSection sec, uint16_t index, uint16_t data){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_WR_PMC_REG;
    cmd.data.axis_index = 0xFF;
    cmd.data.data[0] = sec;
    cmd.data.data[1] = index;
    cmd.data.data[2] = data;
    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 发送MI复位指令
 */
void ChannelEngine::SendMiReset(){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_RESET;
    cmd.data.axis_index = 0xFF;

    this->m_p_mi_comm->WriteCmd(cmd);

}

/**
 * @brief 设置MC单通道自动数据缓冲数量
 * @param count : 缓冲数量
 */
void ChannelEngine::SetMcAutoBufMax(uint16_t count){
    //	this->m_n_mc_auto_buf_max = count;
    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        this->m_p_channel_control[i].SetMcAutoBufMax(count-kMaxGCodeFifoCount-10);   //减少2个，从稳定性考虑，不放满
    }
}

/**
 * @brief 设置SD-LINK从站数据
 */
void ChannelEngine::SetSlaveInfo(){
#ifdef USES_PMC_2_0
    MiCmdFrame cmd;

    //配置扩展IO板卡信息命令
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_SDLINK_SLAVE;
    cmd.data.axis_index = NO_AXIS;

    int dev_count = 0;          //从站总数
    int total_in = 0;           //总输入字节数
    int total_out = 0;          //总输出字节数
    int handwheel_count = 0;    //手轮个数

    if (!m_list_bdio_dev.IsEmpty())
    {
        ListNode<BdioDevInfo> *node = this->m_list_bdio_dev.HeadNode();
        while(node != nullptr){
            //参数拼接
            char16_t data0 = (node->data.device_type << 8) | (node->data.group_index);
            char16_t data1 = 0;
            char16_t data2 = node->data.in_bytes;
            char16_t data3 = node->data.out_bytes;
            char16_t data4 = (node->data.output_start << 8) | (node->data.input_start);
            char16_t data5 = node->data.handwheel_map;

            cmd.data.data[0] = data0;
            cmd.data.data[1] = data1;
            cmd.data.data[2] = data2;
            cmd.data.data[3] = data3;
            cmd.data.data[4] = data4;
            cmd.data.data[5] = data5;

            //test
            printf("node data0 %d------------------\n", cmd.data.data[0]);
            printf("node data1 %d------------------\n", cmd.data.data[1]);
            printf("node data2 %d------------------\n", cmd.data.data[2]);
            printf("node data3 %d------------------\n", cmd.data.data[3]);
            printf("node data4 %d------------------\n", cmd.data.data[4]);
            printf("node data5 %d------------------\n", cmd.data.data[5]);
            printf("node data6 %d------------------\n", cmd.data.data[6]);
            printf("\n");

            //数据发送
            this->m_p_mi_comm->WriteCmd(cmd);
            total_in += node->data.in_bytes;
            total_out += node->data.out_bytes;
            dev_count++;
            if (node->data.handwheel_map)
                ++handwheel_count;

            node = node->next;
        }
    }

    //扩展IO板卡配置完成命令
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.axis_index = NO_AXIS;
    cmd.data.cmd = CMD_MI_SET_SDLINK_COMPLETE;
    cmd.data.data[0] = dev_count;
    cmd.data.data[1] = total_in;
    cmd.data.data[2] = total_out;
    cmd.data.data[3] = handwheel_count;

    //test
    printf("node data0 %d------------------\n", cmd.data.data[0]);
    printf("node data1 %d------------------\n", cmd.data.data[1]);
    printf("node data2 %d------------------\n", cmd.data.data[2]);
    printf("node data3 %d------------------\n", cmd.data.data[3]);
    printf("node data4 %d------------------\n", cmd.data.data[4]);
    printf("node data5 %d------------------\n", cmd.data.data[5]);
    printf("node data6 %d------------------\n", cmd.data.data[6]);
    printf("\n");

    this->m_p_mi_comm->WriteCmd(cmd);

#else
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_SDLINK_SLAVE;
    cmd.data.axis_index = NO_AXIS;

    int total_in = 0;  //总输入字节数
    int total_out = 0;  //总输出字节数
    int dev_count = 0;  //从站总数

    ListNode<BdioDevInfo> *node = this->m_list_bdio_dev.HeadNode();
    while(node != nullptr){
        cmd.data.data[0] = node->data.slave_index;
        cmd.data.data[1] = node->data.device_type;
        cmd.data.data[2] = node->data.in_bytes;
        cmd.data.data[3] = node->data.out_bytes;

        this->m_p_mi_comm->WriteCmd(cmd);
        total_in += node->data.in_bytes;
        total_out += node->data.out_bytes;

        node = node->next;
        dev_count++;
    }

    cmd.data.cmd = CMD_MI_SET_SDLINK_COMPLETE;
    cmd.data.data[0] = dev_count;
    cmd.data.data[1] = total_in;
    cmd.data.data[2] = total_out;
    cmd.data.data[3] = 0;

    this->m_p_mi_comm->WriteCmd(cmd);

#endif

}

/**
 * @brief 发送获取MI版本指令
 */
//void ChannelEngine::SendMiGetVerCmd(){
//	MiCmdFrame cmd;
//	memset(&cmd, 0x00, sizeof(cmd));
//	cmd.data.cmd = CMD_MI_GET_VERSION;
//	cmd.data.axis_index = 0xFF;
//
//	this->m_p_mi_comm->WriteCmd(cmd);
//	printf("get mi version!!!!\n");
//}


/**
 * @brief 清除告警
 */
void ChannelEngine::ClearAlarm(){
    //清除告警
    g_ptr_alarm_processor->Clear();
    m_axis_status_ctrl->UpdateServoState();
}

/**
 * @brief 清空MI中的PMC轴移动数据
 */
void ChannelEngine::ClearPmcAxisMoveData(){
    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(MiCmdFrame));

    cmd.data.axis_index = NO_AXIS;
    cmd.data.reserved = CHANNEL_ENGINE_INDEX;
    cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
    cmd.data.data[0] = 0x20;   //停止当前所有PMC轴运动，并抛弃当前运动指令

    m_p_mi_comm->WriteCmd(cmd);
    this->m_n_run_axis_mask = 0;
    this->m_n_runover_axis_mask = 0;
}

/**
 * @brief 系统复位
 */
void ChannelEngine::SystemReset(){

    printf("system reset\n");
    //各通道复位
    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        //        // 攻丝状态禁止复位
        //        if(this->m_p_channel_control[0].GetSpdCtrl()->isTapEnable()){
        //            CreateError(ERR_SPD_RESET_IN_TAP,
        //                        WARNING_LEVEL,
        //                        CLEAR_BY_MCP_RESET);
        //            return;
        //        }

        this->m_p_pmc_reg->FReg().bits[i].RST = 1;  //复位信号
        this->m_p_channel_control[i].Reset();
    }

    //通知MC模块复位
    //	SendMcResetCmd();    //修改为按通道复位


    //通知MI模块复位
    this->SendMiReset();

    //通知HMI清除告警，复位
    HMICmdFrame cmd;
    cmd.channel_index = CHANNEL_ENGINE_INDEX;
    cmd.cmd = CMD_SC_RESET;
    cmd.cmd_extension = 0;
    cmd.data_len = 0;
    this->m_p_hmi_comm->SendCmd(cmd);

    //复位告警码
    this->m_error_code = ERR_NONE;


    //清空告警队列
    ClearAlarm();


    //TODO 激活参数
    printf("reset update param \n");
    g_ptr_parm_manager->ActiveResetParam();
    g_ptr_parm_manager->ActiveNewStartParam();

    //	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
    //		this->m_p_channel_control[i].SetMcChnPlanMode();
    //		this->m_p_channel_control[i].SetMcChnPlanParam();
    //		this->m_p_channel_control[i].SetMcChnPlanFun();
    //		this->m_p_channel_control[i].SetMcChnCornerStopParam();
    //	}

    m_b_power_off = false;

    this->m_hard_limit_negative = 0;
    this->m_hard_limit_postive = 0;

    //初始化回参考点变量
    if(m_b_ret_ref){
        for(uint8_t id = 0; id < this->m_p_general_config->axis_count; id++){
            if(this->m_n_mask_ret_ref & (0x01<<id)){
                this->SetInRetRefFlag(id, false);   //复位回零中标志
            }
        }
    }
    this->m_b_ret_ref = false;
    this->m_b_ret_ref_auto = false;
    this->m_n_mask_ret_ref = 0;
    //	m_n_get_cur_encoder_count = 0;
    memset(this->m_n_ret_ref_step, 0x00, kMaxAxisNum*sizeof(int));

    this->ClearPmcAxisMoveData();   //清空PMC轴运动数据

    //	m_n_run_axis_mask = 0x00;  //当前运行的轴的mask
    //	m_n_runover_axis_mask = 0x00;   //当前运行完成的轴的mask   //ClearPmcAxisMoveData()中清空


    //记录复位操作结束时间，供RST信号延时后复位
    gettimeofday(&this->m_time_rst_over, NULL);
    m_b_reset_rst_signal = true;

    printf("channel engine reset finished !!!\n");
}

/**
 * @brief 急停处理
 * @param chn : 通道号
 */
void ChannelEngine::Emergency(uint8_t chn){
    m_b_emergency = true;

    //急停处理
    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        this->m_p_channel_control[i].EmergencyStop();
    }

    //初始化回参考点变量
    if(m_b_ret_ref){
        for(uint8_t id = 0; id < this->m_p_general_config->axis_count; id++){
            if(this->m_n_mask_ret_ref & (0x01<<id)){
                this->SetInRetRefFlag(id, false);   //复位回零中标志
            }
        }
    }
    this->m_b_ret_ref = false;
    this->m_b_ret_ref_auto = false;
    this->m_n_mask_ret_ref = 0;
    //	m_n_get_cur_encoder_count = 0;
    memset(this->m_n_ret_ref_step, 0x00, kMaxAxisNum*sizeof(int));

    this->ClearPmcAxisMoveData();   //清空PMC轴运动数据


    //记录复位操作结束时间，供RST信号延时后复位
    gettimeofday(&this->m_time_rst_over, NULL);
    // lidianqiang:急停后，rst信号需要保持为1。等解除急停并且按下复位后才恢复
    // m_b_reset_rst_signal = true;


    //置位F寄存器


    //复位增量式编码器轴的回参考点完成标志
#ifndef USES_WOOD_MACHINE
    for(int i = 0; i < this->m_p_general_config->axis_count; i++){
        if(m_p_axis_config[i].axis_interface != VIRTUAL_AXIS && m_p_axis_config[i].axis_type != AXIS_SPINDLE	//非主轴并且非虚拟轴
                && m_p_axis_config[i].feedback_mode == INCREMENTAL_ENCODER )
        {
            //			m_n_mask_ret_ref_over &= ~(0x01<<i);   //
            this->SetRetRefFlag(i, false);
        }
    }
#endif

    //生成急停错误信息
    m_error_code = ERR_EMERGENCY;
    //CreateError(ERR_EMERGENCY, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, chn);//梯图有报警

}

/**
 * @brief  MI状态更新线程函数
 * @param args
 */
void *ChannelEngine::RefreshMiStatusThread(void *args){
    ChannelEngine *chn_engine = static_cast<ChannelEngine *>(args);
    printf("Start ChannelEnigine::RefreshMiStatusThread!thread id = %ld\n", syscall(SYS_gettid));

    int res = ERR_NONE;
    res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //设置线程为可退出模式
    if (res != ERR_NONE) {
        printf("Quit ChannelEnigine::RefreshMiStatusThread!thread!thread with error 1!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }
    res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //线程退出方式为异步退出，不用等到cancellation point
    if (res != ERR_NONE) {
        printf("Quit ChannelEnigine::RefreshMiStatusThread!thread!thread with error 2!\n");
        pthread_exit((void*) EXIT_FAILURE);
    }

    while(!g_sys_state.system_ready){  //等待系统启动
        //printf("ChannelEngine::RefreshMiStatusThread wait system ready[0x%hhx]!\n", g_sys_state.module_ready_mask);
        usleep(10000);
    }

    chn_engine->RefreshMiStatusFun();

    printf("Quit ChannelEnigine::RefreshMiStatusThread!!\n");
    pthread_exit(NULL);
}

/**
 * @brief 更新MI的状态
 * @return
 */
bool ChannelEngine::RefreshMiStatusFun(){
    uint64_t count = 0;   //计数
    uint64_t warn_flag = 0;

    uint64_t value64 = 0;
    uint32_t value32 = 0;

    printf("start ChannelEngine::RefreshMiStatusFun(), sys_quit=%hhu\n", g_sys_state.system_quit);

    uint8_t *p_f_reg = m_p_pmc_reg->FReg().all;
    uint8_t *p_g_reg = m_p_pmc_reg->GetRegPtr8(PMC_REG_G);
    uint8_t *p_k_reg = m_p_pmc_reg->GetRegPtr8(PMC_REG_K);
#ifndef USES_PMC_2_0
    uint8_t *p_d_reg = (uint8_t *)m_p_pmc_reg->GetRegPtr16(PMC_REG_D);
    uint8_t *p_c_reg = (uint8_t *)m_p_pmc_reg->GetRegPtr16(PMC_REG_C);
    uint8_t *p_t_reg = (uint8_t *)m_p_pmc_reg->GetRegPtr16(PMC_REG_T);
    uint8_t *p_dc_reg = (uint8_t *)m_p_pmc_reg->GetRegPtr16(PMC_REG_DC);
    uint8_t *p_dt_reg = (uint8_t *)m_p_pmc_reg->GetRegPtr16(PMC_REG_DT);
#else
    uint8_t *p_d_reg = m_p_pmc_reg->GetRegPtr8(PMC_REG_D);
    uint8_t *p_c_reg = m_p_pmc_reg->GetRegPtr8(PMC_REG_C);
    uint8_t *p_t_reg = m_p_pmc_reg->GetRegPtr8(PMC_REG_T);
    uint8_t *p_e_reg = m_p_pmc_reg->GetRegPtr8(PMC_REG_E);
#endif


    while(!g_sys_state.system_quit){

        if((g_sys_state.module_ready_mask & MI_READY) == 0){  //MI未准备好则等待
            printf("wait MI_READY signal!\n");
            usleep(8000);
            continue;
        }

        //读取欠压信号
        if(!m_b_power_off && g_sys_state.system_ready && this->m_p_mc_comm->ReadUnderVoltWarn()){
        	printf("OFF\n");
            m_b_power_off = true;

            SaveDataPoweroff();
            g_sys_state.system_quit = true;   //程序退出
            printf("OUT\n");
            return true;
        }

        //更新写入F寄存器， 更新周期8ms
        this->m_p_mi_comm->WritePmcReg(PMC_REG_F, p_f_reg);
        memcpy(m_g_reg_last.all, p_g_reg, sizeof(m_g_reg_last.all));  //备份G寄存器
        memcpy(m_f_reg_last.all, p_f_reg, sizeof(m_f_reg_last.all));
        this->m_p_mi_comm->ReadPmcReg(PMC_REG_G, p_g_reg);
        this->m_p_mi_comm->ReadPmcReg(PMC_REG_K, p_k_reg);

#ifndef USES_PMC_2_0
        this->m_p_mi_comm->ReadPmcReg(PMC_REG_D, p_d_reg);
        this->m_p_mi_comm->ReadPmcReg(PMC_REG_C, p_c_reg);
        this->m_p_mi_comm->ReadPmcReg(PMC_REG_T, p_t_reg);
        this->m_p_mi_comm->ReadPmcReg(PMC_REG_DC, p_dc_reg);
        this->m_p_mi_comm->ReadPmcReg(PMC_REG_DT, p_dt_reg);
#else
        this->m_p_mi_comm->ReadPmcReg(PMC_REG_D, p_d_reg);
        this->m_p_mi_comm->ReadPmcReg(PMC_REG_C, p_c_reg);
        this->m_p_mi_comm->ReadPmcReg(PMC_REG_T, p_t_reg);
        this->m_p_mi_comm->ReadPmcReg(PMC_REG_E, p_e_reg);
#endif


        //读取最新的PMC寄存器值
        if(count % 10 == 0){	//更新周期80ms
            this->m_p_mi_comm->ReadPmcReg(PMC_REG_X, m_p_pmc_reg->GetRegPtr8(PMC_REG_X));
            this->m_p_mi_comm->ReadPmcReg(PMC_REG_Y, m_p_pmc_reg->GetRegPtr8(PMC_REG_Y));
            this->m_p_mi_comm->ReadPmcReg(PMC_REG_R, m_p_pmc_reg->GetRegPtr8(PMC_REG_R));
            this->m_p_mi_comm->ReadPmcReg(PMC_REG_A, m_p_pmc_reg->GetRegPtr8(PMC_REG_A));
            this->m_p_mi_comm->ReadPmcPeriod();


            //			if(count %200 == 0)
            //				printf("k register : %hhu\n",m_p_pmc_reg->GetRegPtr8(PMC_REG_K)[0]);
            //			if(count %310 == 0)
            //				printf("D register : %hu\n",m_p_pmc_reg->GetRegPtr16(PMC_REG_D)[300]);

        }


        this->m_p_mi_comm->ReadServoOnState(m_n_phy_axis_svo_on);
        Singleton<AxisStatusCtrl>::instance().UpdateSA(m_n_phy_axis_svo_on);

        if(!this->m_b_power_off){  //掉电后不处理
            this->ProcessPmcSignal();

            //首先读取轴告警标志
            this->m_p_mi_comm->ReadAxisWarnFlag(warn_flag);
            if(warn_flag != 0x00){//有告警，再看具体告警位
                if(warn_flag & (0x01 << 0)){//正限位告警
                    uint64_t hlimtflag = 0;
                    this->m_p_mi_comm->ReadServoHLimitFlag(true, hlimtflag);   //读取正限位数据
                    if(m_hard_limit_postive == 0){
                        this->m_hard_limit_postive |= hlimtflag;
                        this->ProcessAxisHardLimit(0);
                    }else{
                        this->m_hard_limit_postive |= hlimtflag;
                    }
                }
                if(warn_flag & (0x01 << 1)){ //负限位告警
                    uint64_t hlimtflag = 0;
                    this->m_p_mi_comm->ReadServoHLimitFlag(false, hlimtflag);   //读取负限位数据
                    if(m_hard_limit_negative == 0){
                        this->m_hard_limit_negative |= hlimtflag;
                        this->ProcessAxisHardLimit(1);
                    }else{
                        this->m_hard_limit_negative |= hlimtflag;
                    }
                }
                if(warn_flag & (0x01 << 2)){ //编码器告警
                    this->m_p_mi_comm->ReadEncoderWarn(value64);  //读取编码器告警数据
                    if(value64 != 0){
                        //有编码器告警
                        uint64_t flag = 0x01;
                        for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
                            if(value64 & flag)
                                CreateError(ERR_ENCODER, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
                            flag = flag << 1;
                        }
                    }
                }
                if(warn_flag & (0x01 << 3)){ //伺服告警
                    this->m_p_mi_comm->ReadServoWarn(value64);   //读取伺服告警
                    if(value64 != 0){
                        //有伺服告警
                        uint64_t flag = 0x01;
                        for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
                            if(value64 & flag){
                                this->m_p_mi_comm->ReadServoWarnCode(i, value32);   //读取告警码
                                //产生告警
                                CreateError(ERR_SERVO, ERROR_LEVEL, CLEAR_BY_MCP_RESET, value32, CHANNEL_ENGINE_INDEX, i);
                            }
                            flag = flag << 1;
                        }
                    }
                }
                if(warn_flag & (0x01 << 4)){  //跟随误差过大告警
                    this->m_p_mi_comm->ReadTrackErr(value64);
                    if(value64 != 0){
                        //有跟随误差过大告警
                        uint64_t flag = 0x01;
                        for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
                            if(value64 & flag)
                                CreateError(ERR_TRACK_LIMIT, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
                            flag = flag << 1;
                        }
                    }

                }
                if(warn_flag & (0x01 << 5)){  //同步轴位置指令偏差过大告警
                    this->m_p_mi_comm->ReadSyncPosErr(value64);
                    if(value64 != 0){
                        //有同步轴位置指令偏差过大告警
                        uint64_t flag = 0x01;
                        for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
                            if(value64 & flag)
                                CreateError(ERR_SYNC_POS, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
                            flag = flag << 1;
                        }
                    }
                }
                if(warn_flag & (0x01 << 6)){  //轴位置指令过大告警
                    this->m_p_mi_comm->ReadIntpPosErr(value64);
                    if(value64 != 0){
                        //有轴位置指令过大告警
                        uint64_t flag = 0x01;
                        for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
                            if(value64 & flag)
                                CreateError(ERR_INTP_POS, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
                            flag = flag << 1;
                        }
                    }
                }
                if(warn_flag & (0x01 << 7)){  //同步轴力矩偏差过大告警
                    this->m_p_mi_comm->ReadSyncTorqueErr(value64);
                    if(value64 != 0){
                        uint64_t flag = 0x01;
                        for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
                            if(value64 & flag)
                                CreateError(ERR_SYNC_TORQUE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
                            flag = flag << 1;
                        }
                    }
                }
                if(warn_flag & (0x01 << 8)){  //同步轴机床坐标偏差过大告警
                    this->m_p_mi_comm->ReadSyncMachErr(value64);
                    if(value64 != 0){
                        uint64_t flag = 0x01;
                        for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
                            if(value64 & flag)
                                CreateError(ERR_SYNC_MACH, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
                            flag = flag << 1;
                        }
                    }
                }
            }


            //进行过压，欠压、RTC电压低告警扫描
#ifndef USES_MAIN_BOARD_10MA_OLD    //老板没有此功能
            if(count%1000 == 0){
                this->CheckBattery();
            }
#endif

            if(this->m_b_ret_ref && this->m_p_pmc_reg->FReg().bits[0].SA ==1){//回参考点操作
                this->ReturnRefPoint();
            }
        }

        //更新轴限位数据给MI
        this->m_p_mi_comm->WriteAxisHLimitFlag(true, m_hard_limit_postive);
        this->m_p_mi_comm->WriteAxisHLimitFlag(false, m_hard_limit_negative);

        //TODO 更新轴的位置及速度数据
        if(count%7 == 0){
            if(!g_sys_state.hmi_comm_ready){  //未连接HMI时在此更新轴当前反馈位置
                this->SendMonitorData(false, false);
            }
        }

#ifdef USES_LICENSE_FUNC
        //时间校验以及授权校验
        if(count % 450000 == 0){
            //一小时检查一次

            //检查系统时间
            if(m_ln_local_time < 0){//读取本地计时文件异常
                if(m_ln_local_time == -1){
                    g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "本地计时文件不存在!");
                }else if(m_ln_local_time == -2){
                    g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "本地计时文件已损坏!");
                }else if(m_ln_local_time == -3){
                    g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "非法本地计时文件!");
                }
                m_error_code = ERR_SYSTEM_FILE;
                CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
            }else if(-4 == CheckLocalTime(&m_lic_info, m_ln_local_time)){
                g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "系统时间异常!");
                m_error_code = ERR_SYSTEM_TIME;
                CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
            }


            //检查授权
            this->CheckLicense();

        }
#endif

        usleep(8000);  //8ms周期，比PMC周期相同
        count++;
    }

    //	printf("exit ChannelEngine::RefreshMiStatusFun()\n");

    return true;
}

/**
 * @brief 电压相关告警检查
 */
void ChannelEngine::CheckBattery(){

    unsigned char value;
    int fd = open("/dev/aradex_gpio", O_RDWR);

    if (fd < 0) {
        printf("Can't open /dev/aradex_gpio!\n");
        return;
    }

    ioctl(fd, GET_GPIO_VALUE, &value);
    //	printf("Read GPIO value is %x\n", value);


    if(value & 0x01){//欠压
        CreateError(ERR_VOLTAGE_OVER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, NO_AXIS);

    }

    if(value & 0x02){//过压
        CreateError(ERR_VOLTAGE_UNDER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, NO_AXIS);

    }
    if(value & 0x04){ //电池电压低
        CreateError(ERR_BATTERY_VOL_UNDER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, NO_AXIS);

    }
    if((value & 0x08) == 0x00){//轴过流,低有效
        CreateError(ERR_AXIS_CURRENT_OVER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, NO_AXIS);

    }
    if((value & 0x10) == 0x00){ //USB过流,低有效
        CreateError(ERR_USB_CURRENT_OVER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, NO_AXIS);
    }

    close(fd);

}

/**
 * @brief 处理PMC的信号
 */
void ChannelEngine::ProcessPmcSignal(){

    static bool inited = false;
    const GRegBits *g_reg = nullptr;
    GRegBits *g_reg_last = nullptr;
    FRegBits *f_reg = nullptr;
    FRegBits *f_reg_last = nullptr;
    ChannelControl *ctrl = nullptr;
    uint64_t flag = 0;
    uint8_t chn = 0;
    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        g_reg = &m_p_pmc_reg->GReg().bits[i];
        g_reg_last = &m_g_reg_last.bits[i];
        f_reg = &m_p_pmc_reg->FReg().bits[i];
        f_reg_last = &m_f_reg_last.bits[i];
        ctrl = &m_p_channel_control[i];

        ChnWorkMode mode = INVALID_MODE;
        if(g_reg->MD == 0){  //MDA
            mode = MDA_MODE;
        }else if(g_reg->MD == 1){   //自动
            mode = AUTO_MODE;
        }else if(g_reg->MD == 2){   //手轮
            mode = MPG_MODE;
        }else if(g_reg->MD == 3){   //编辑
            mode = EDIT_MODE;
        }else if(g_reg->MD == 4){   //步进
            mode = MANUAL_STEP_MODE;
        }else if(g_reg->MD == 5){   //JOG
            mode = MANUAL_MODE;
        }else if(g_reg->MD == 6){   //原点模式
            mode = REF_MODE;
        }else{
            printf("ERROR:Invalid work mode signal:MD=%hhu\n", g_reg->MD);
        }
        //方式选择信号
        if(g_reg->MD != g_reg_last->MD || !inited){
            if(mode != INVALID_MODE){
                this->SetWorkMode(mode);
                m_sync_axis_ctrl->InputMode(mode);
            }
        }

        //cnc就绪
        if(!f_reg->MA)
        {
            if (g_sys_state.system_ready && f_reg->SA/*MI需要用伺服就绪信号来判断是否正确初始化*/)
            {
                f_reg->MA = 1;
            }
        }

#ifdef USES_PHYSICAL_MOP
        if(g_reg->_ESP == 0 && !m_b_emergency){ //进入急停
            f_reg->RST = 1;
            m_axis_status_ctrl->InputEsp(g_reg->_ESP);
            g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug, "进入急停");
            this->Emergency();
            m_b_emergency = 1;
        }else if(g_reg->_ESP == 1 && m_b_emergency){ // 取消急停
            m_b_emergency = false;
            m_axis_status_ctrl->InputEsp(g_reg->_ESP);
            g_ptr_tracelog_processor->SendToHmi(kProcessInfo, kDebug, "解除急停");
            f_reg->RST = 0;
        }

        // 伺服关断信号
        if(g_reg_last->SVF != g_reg->SVF){
            m_axis_status_ctrl->InputSVF(g_reg->SVF);
            m_cur_svf = g_reg->SVF;
        }

        if(g_reg_last->KEY != g_reg->KEY){
            SetProgProtect(g_reg->KEY);
        }

        // 处理G信号 切换当前通道
        if(g_reg_last->CHNC != g_reg->CHNC)
        {
            printf("pmc siganl change chn : %d\n", g_reg->CHNC);
            this->SetCurWorkChanl(g_reg->CHNC);
        }

        if((g_reg->ST && g_reg_last->ST == 0) && g_reg->_SP == 1 && f_reg->STL == 0){ //循环启动
            this->Start();
        }

        if(g_reg->_SP == 0 && g_reg_last->_SP == 1 && f_reg->SPL == 0 && f_reg->STL == 1){ //循环保持
            this->Pause();
        }

        // @test zk
        if(g_reg->RRW == 1 && g_reg_last->RRW == 0){
            this->SystemReset();
        }
        // @test

#else
        if(g_reg->_ESP == 1 && !m_b_emergency){ //急停有效
            this->Emergency();
        }

        if((g_reg->ST && g_reg_last->ST == 0) && g_reg->_SP == 0){ //循环启动
            this->Start();
        }

        if(g_reg->_SP == 1 && g_reg_last->_SP == 0){ //循环保持
            //printf("1212\n");
            this->Pause();
        }
#endif

        // 攻丝状态()不让复位
        if(g_reg->ERS == 1 && g_reg_last->ERS == 0){
            this->SystemReset();
        }

        // 某个轴机械锁住信号发生了改变
        if(g_reg->MLK != g_reg_last->MLK || g_reg->MLKI != g_reg_last->MLKI){
            f_reg->MMLK = g_reg->MLK;
            SetMLKState(g_reg->MLK, g_reg->MLKI);
            //日志记录
            if (g_reg->MLK != g_reg_last->MLK)
            {
                if (g_reg->MLK)
                    g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "开启[机械锁住]");
                else
                    g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "关闭[机械锁住]");
            }
            else
            {
                for (int i = 0; i < m_p_channel_config->chn_axis_count; i++)
                {
                    bool curState = g_reg->MLKI & (0x01 << i);
                    bool lastState = g_reg_last->MLKI & (0x01 << i);
                    if (curState && !lastState)
                        g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "轴 " + to_string(i) + "开启[机械锁]");
                    if (!curState && lastState)
                        g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "轴 " + to_string(i) + "关闭[机械锁]");
                }
            }
        }

        // 限位开关选择信号
        if(g_reg->EXLM != g_reg_last->EXLM || g_reg->RLSOT != g_reg_last->RLSOT){
            SetSoftLimitSignal(g_reg->EXLM, g_reg->RLSOT);
        }

        if(g_reg->SYNC != g_reg_last->SYNC){
            m_sync_axis_ctrl->InputSync(g_reg->SYNC);
        }
        if(g_reg->SYNCJ != g_reg_last->SYNCJ){
            m_sync_axis_ctrl->InputSyncJ(g_reg->SYNCJ);
        }

        // 主轴正转，主轴反转信号
        if(g_reg->SRV != g_reg_last->SRV || g_reg->SFR != g_reg_last->SFR){
            if(g_reg->SRV == 0 && g_reg->SFR == 0){ // 主轴停
                ctrl->GetSpdCtrl()->InputPolar(Spindle::Stop);
            }else if(g_reg->SFR == 1){  // 主轴正转
                ctrl->GetSpdCtrl()->InputPolar(Spindle::Positive);
            }else if(g_reg->SRV == 1){  // 主轴反转
                ctrl->GetSpdCtrl()->InputPolar(Spindle::Negative);
            }
        }

        // 主轴停止输出信号
        if(g_reg->_SSTP != g_reg_last->_SSTP){
            ctrl->GetSpdCtrl()->InputSSTP(g_reg->_SSTP);
        }
        // 主轴准停信号
        if(g_reg->SOR != g_reg_last->SOR){
            ctrl->GetSpdCtrl()->InputSOR(g_reg->SOR);
        }
        // 主轴转速外部输入
        if(g_reg->RI != g_reg_last->RI){
            ctrl->GetSpdCtrl()->InputRI(g_reg->RI);
        }
        // PMC来源的主轴方向
        if(g_reg->SGN != g_reg_last->SGN){
            ctrl->GetSpdCtrl()->InputSGN(g_reg->SGN);
        }
        // 设置主轴方向来源
        if(g_reg->SSIN != g_reg_last->SSIN){
            ctrl->GetSpdCtrl()->InputSSIN(g_reg->SSIN);
        }
        // 设置主轴转速来源
        if(g_reg->SIND != g_reg_last->SIND){
            ctrl->GetSpdCtrl()->InputSIND(g_reg->SIND);
        }
        // 刚性攻丝信号
        if(g_reg->RGTAP != g_reg_last->RGTAP){
            ctrl->GetSpdCtrl()->InputRGTAP(g_reg->RGTAP);
        }
        // 主轴控制模式切换
        if(g_reg->RGMD != g_reg_last->RGMD){
            ctrl->GetSpdCtrl()->InputRGMD(g_reg->RGMD);
        }
        // 定位信号
        if(g_reg->ORCMA != g_reg_last->ORCMA){
            ctrl->GetSpdCtrl()->InputORCMA(g_reg->ORCMA);
        }
        // 主轴选通信号
        if(f_reg->SF == 1 && g_reg->FIN == 1){
            f_reg->SF = 0;
        }
        // 攻丝回退
        if(g_reg->RTNT != g_reg_last->RTNT){
            ctrl->GetSpdCtrl()->InputRTNT(g_reg->RTNT);
        }
        // 换刀信号
        if(g_reg->GTC != g_reg_last->GTC) {
            ctrl->SetSysVarValue(4320, g_reg->GTC);
        }

        //通知类型的信号，只保留一个周期
        {
            if(f_reg_last->RTPT == 1)   // 攻丝回退结束信号
                f_reg->RTPT = 0;

            if(f_reg_last->MERS == 1)   // MDI复位请求
                f_reg->MERS = 0;
        }

        //选停信号 SBS
        if(g_reg->SBS != g_reg_last->SBS){
            if(g_reg->SBS)
            {
                this->SetFuncState(i, FS_OPTIONAL_STOP, 1);
            }
            else
                this->SetFuncState(i, FS_OPTIONAL_STOP, 0);
        }


        //单段信号  SBK
        if(g_reg->SBK != g_reg_last->SBK){
            if(g_reg->SBK)
                this->SetFuncState(i, FS_SINGLE_LINE, 1);
            else
                this->SetFuncState(i, FS_SINGLE_LINE, 0);
        }


        //跳段信号   BDT1
        if(g_reg->BDT1 != g_reg_last->BDT1){
            if(g_reg->BDT1)
            {
                this->SetFuncState(i, FS_BLOCK_SKIP, 1);
            }
            else
                this->SetFuncState(i, FS_BLOCK_SKIP, 0);
        }

        //手轮跟踪  HWT
        if(g_reg->HWT != g_reg_last->HWT){
            if(g_reg->HWT)
                this->SetFuncState(i, FS_HANDWHEEL_CONTOUR, 1);
            else
                this->SetFuncState(i, FS_HANDWHEEL_CONTOUR, 0);
        }

        if(g_reg->HSIA != g_reg_last->HSIA){
            MachiningState state = (MachiningState)ctrl->GetChnStatus().machining_state;
            // 运行或者暂停状态才能手轮插入
            if(g_reg->HSIA != 0 && state != MS_RUNNING && state != MS_PAUSED){
                CreateError(ERR_HW_INSERT_INVALID, INFO_LEVEL, CLEAR_BY_MCP_RESET, 0, i);
            }else{
                if(g_reg->HSIA != 0){
                    CreateError(ERR_HW_INSERT_INFO, INFO_LEVEL, CLEAR_BY_MCP_RESET, 0, i);
                }
                for(int axis = 0; axis < m_p_channel_config[chn].chn_axis_count; axis++){
                    if( ((g_reg->HSIA >> axis) & 0x01) && GetPmcActive(axis) ){
                        CreateError(ERR_HW_INSERT_INVALID, INFO_LEVEL, CLEAR_BY_MCP_RESET, 0, i);
                    }
                }
                UpdateHandwheelState(i);
            }
        }


        //手动步长信号  MP
        if(g_reg->MP != g_reg_last->MP){
            this->SetManualStep(i, g_reg->MP);
        }

        //手动快速进给选择信号  RT
        if(g_reg->RT != g_reg_last->RT){
            if(g_reg->RT)
            {
                g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "开启[快速移动]");
                this->SetManualRapidMove(1);
            }
            else
            {
                g_ptr_tracelog_processor->SendToHmi(kPanelOper, kDebug, "关闭[快速移动]");
                this->SetManualRapidMove(0);
            }
        }

        //倍率信号处理

        if(g_reg->_JV != g_reg_last->_JV || !inited){
            uint16_t ratio= ~g_reg->_JV;
            if(g_reg->_JV == 0)
                ratio = 0;
            this->SetManualRatio(i, ratio/100);
        }
        if(g_reg->_FV != g_reg_last->_FV || !inited)
            this->SetAutoRatio(i, ~g_reg->_FV);
        if(g_reg->SOV != g_reg_last->SOV || !inited)
            ctrl->SetSpindleRatio(g_reg->SOV);
        if(g_reg->ROV != g_reg_last->ROV || !inited)
            this->SetRapidRatio(i, g_reg->ROV);


        if(g_reg->JP != g_reg_last->JP){
            // 如果在增量模式下并且手动倍率为0，则不发指令
            bool valid = !(mode == MANUAL_STEP_MODE && ctrl[i].GetManualRatio() == 0);
            if(valid){
                SetJPState(i, g_reg->JP, g_reg_last->JP, mode);
            }
        }
        if(g_reg->JN != g_reg_last->JN){
            // 如果在增量模式下并且手动倍率为0，则不发指令
            bool valid = !(mode == MANUAL_STEP_MODE && ctrl[i].GetManualRatio() == 0);
            if(valid){
                SetJNState(i, g_reg->JN, g_reg_last->JN, mode);
            }
        }

        if(g_reg->MD == 6){  //原点模式
            for(int j = 0; j < 16; j++){
                if((g_reg->REFE & (0x01<<j)) != 0 && (g_reg_last->REFE & (0x01<<j)) == 0){// 启动轴回零
                    this->ProcessPmcRefRet(j+16*i);
                }
            }
        }


        //#endif

        //处理工艺模式切换
#ifdef USES_WOOD_MACHINE
        if(g_reg_last->BDM == 0 && g_reg->BDM == 1){
            this->m_p_channel_control[i].SetCurProcParamIndex(0);
        }else if(g_reg_last->BOXM == 0 && g_reg->BOXM == 1){
            this->m_p_channel_control[i].SetCurProcParamIndex(1);
        }
#endif

        //处理PMC宏调用功能
        if(g_reg_last->EMPC == 0 && g_reg->EMPC == 1){  //处理PMC宏调用
            // @test zk
            printf("PMC CALL SUB PROG : %d\n", g_reg->MPCS);
            this->m_p_channel_control[i].CallMacroProgram(g_reg->MPCS);
            f_reg->MPCO = 1;   //调用结束
        }else if(g_reg_last->EMPC == 1 && g_reg->EMPC == 0){
            f_reg->MPCO = 0;   //信号复位
        }

#ifdef USES_WUXI_BLOOD_CHECK
        if(g_reg->ret_home){//回零
            printf("wuxi, ret home\n");
            if(!m_p_channel_control[0].IsReturnHome()){
                m_p_channel_control[0].SetRetHomeFlag();  //置位标志
            }
        }

        if(g_reg->reset){//复位
            printf("wuxi, reset\n");
            for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
                this->m_p_channel_control[i].Reset();
            }
        }
#endif
    }
    //处理轴的限位信号，虽然64个轴的限位信号平均分布在四个通道中，但是处理时不分通道
    if(!this->m_b_ret_ref){  //回参考点过程中屏蔽限位
        chn = m_p_general_config->axis_count/16;
        if(m_p_general_config->axis_count%16)
            chn++;
        for(int i = 0; i < chn; i++){

            g_reg = &m_p_pmc_reg->GReg().bits[i];
            if(g_reg->axis_limit_postive1 != 0x00 || g_reg->axis_limit_postive2 != 0x00){  //正向硬限位触发

                flag = g_reg->axis_limit_postive2;
                flag <<= 8;
                flag |= g_reg->axis_limit_postive1;
                flag <<= 16*i;
                if(m_hard_limit_postive == 0){

                    this->m_hard_limit_postive |= flag;
                    this->ProcessAxisHardLimit(0);
                    //		printf("positive limit : 0x%llx\n", flag);
                }else{
                    this->m_hard_limit_postive |= flag;
                    //		printf("positive limit22 : 0x%llx\n", flag);
                }

            }

            if(g_reg->axis_limit_negative1 != 0x00 || g_reg->axis_limit_negative2 != 0x00){  //负向硬限位触发
                flag = g_reg->axis_limit_negative2;
                flag <<= 8;
                flag |= g_reg->axis_limit_negative1;
                flag <<= 16*i;
                if( m_hard_limit_negative == 0){

                    this->m_hard_limit_negative |= flag;
                    this->ProcessAxisHardLimit(1);
                    //		printf("negative limit : 0x%llx\n", m_hard_limit_negative);
                }else{
                    this->m_hard_limit_negative |= flag;
                    //		printf("negative limit222 : 0x%llx\n", m_hard_limit_negative);
                }
            }
            if(m_pos_hard_limit_last != m_hard_limit_postive){
                m_pos_hard_limit_last = m_hard_limit_postive;
                m_p_mi_comm->SendHardLimitState(true,m_pos_hard_limit_last);
            }
            if(m_neg_hard_limit_last != m_hard_limit_negative){
                m_neg_hard_limit_last = m_hard_limit_negative;
                m_p_mi_comm->SendHardLimitState(false,m_neg_hard_limit_last);
            }
        }
    }


    //复位RST信号
    if(this->m_b_reset_rst_signal){
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_rst_over.tv_sec)*1000000+time_now.tv_usec-m_time_rst_over.tv_usec;
        if(time_elpase >= this->m_p_channel_config[0].rst_hold_time * 1000){
            for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
                //延时时间到，复位RST信号
                this->m_p_pmc_reg->FReg().bits[i].RST = 0;
                //	this->m_p_channel_control[i].ResetRSTSignal();

            }
            this->m_b_reset_rst_signal = false;
        }
    }

    //PMC轴控制处理
    this->ProcessPmcAxisCtrl();

    //处理PMC数据窗口
    this->ProcessPmcDataWnd();


    //给出轴在参考点信号
    int byte = 0, bit = 0;
    FRegBits *freg = nullptr;
    double prec = 0.1;  //在位精度
    for(int i = 0; i < this->m_p_general_config->axis_count; i++){
        chn = i/16;
        byte = i%16;
        bit = byte%8;
        freg = &m_p_pmc_reg->FReg().bits[chn];
        if((m_n_mask_ret_ref_over & (0x01<<i)) == 0){//未回参考点，都赋零
            if(byte < 8){
                freg->ZP11 &= ~(0x01<<bit);
                freg->ZP21 &= ~(0x01<<bit);
                freg->ZP31 &= ~(0x01<<bit);
                freg->ZP41 &= ~(0x01<<bit);
            }
            else{
                freg->ZP12 &= ~(0x01<<bit);
                freg->ZP22 &= ~(0x01<<bit);
                freg->ZP32 &= ~(0x01<<bit);
                freg->ZP42 &= ~(0x01<<bit);
            }
            continue;
        }
        //第一参考点
        if(fabs(m_df_phy_axis_pos_feedback[i]-m_p_axis_config[i].axis_home_pos[0]) < prec){
            if(byte < 8)
                freg->ZP11 |= (0x01<<bit);
            else
                freg->ZP12 |= (0x01<<bit);
        }else{
            if(byte < 8)
                freg->ZP11 &= ~(0x01<<bit);
            else
                freg->ZP12 &= ~(0x01<<bit);
        }
        //第二参考点
        if(fabs(m_df_phy_axis_pos_feedback[i]-m_p_axis_config[i].axis_home_pos[1]) < prec){
            if(byte < 8)
                freg->ZP21 |= (0x01<<bit);
            else
                freg->ZP22 |= (0x01<<bit);
        }else{
            if(byte < 8)
                freg->ZP21 &= ~(0x01<<bit);
            else
                freg->ZP22 &= ~(0x01<<bit);
        }
        //第三参考点
        if(fabs(m_df_phy_axis_pos_feedback[i]-m_p_axis_config[i].axis_home_pos[2]) < prec){
            if(byte < 8)
                freg->ZP31 |= (0x01<<bit);
            else
                freg->ZP32 |= (0x01<<bit);
        }else{
            if(byte < 8)
                freg->ZP31 &= ~(0x01<<bit);
            else
                freg->ZP32 &= ~(0x01<<bit);
        }
        //第四参考点
        if(fabs(m_df_phy_axis_pos_feedback[i]-m_p_axis_config[i].axis_home_pos[3]) < prec){
            if(byte < 8)
                freg->ZP41 |= (0x01<<bit);
            else
                freg->ZP42 |= (0x01<<bit);
        }else{
            if(byte < 8)
                freg->ZP41 &= ~(0x01<<bit);
            else
                freg->ZP42 &= ~(0x01<<bit);
        }
    }
    inited = true;

#ifdef TAP_TEST
    static string pathname = string(PATH_LOG_FILE) + "data.txt";
    for (int i = 0; i < m_p_general_config->chn_count; ++i)
    {
        if (m_p_channel_control[i].IsMachinRunning())
        {
            if (!m_fd)
            {
                m_fd = fopen(pathname.c_str(), "a+");
                if (!m_fd)
                    std::cout << ">>>>>> open err" << std::endl;
            }

            if (m_fd && g_reg->RGTAP == 1)
            {
                this->m_p_mi_comm->ReadPhyAxisCurFedBckPos(m_df_phy_axis_pos_feedback, m_df_phy_axis_pos_intp,m_df_phy_axis_speed_feedback,
                                                           m_df_phy_axis_torque_feedback, m_df_spd_angle, m_p_general_config->axis_count);

                //X轴数据
                //string X_tar_pos = to_string(m_df_phy_axis_pos_feedback[2]) + '\t';
                //string X_real_pos = to_string(m_df_phy_axis_pos_intp[2]) +'\t';

                //Y轴数据
                //string y_tar_pos = to_string(m_df_phy_axis_pos_feedback[2]) + '\t';
                //string y_real_pos = to_string(m_df_phy_axis_pos_intp[2]) +'\t';

                //Z轴数据
                string z_tar_pos = to_string(m_df_phy_axis_pos_feedback[2]) + '\t';
                string z_real_pos = to_string(m_df_phy_axis_pos_intp[2]) +'\t';

                //主轴数据
                string spd_tar_pos = to_string(m_df_phy_axis_pos_feedback[m_p_channel_control[i].GetSpdCtrl()->GetPhyAxis()]) + '\t';
                string spd_real_pos = to_string(m_df_phy_axis_pos_intp[m_p_channel_control[i].GetSpdCtrl()->GetPhyAxis()]) + '\t';

                string msg = z_tar_pos + z_real_pos + spd_tar_pos + spd_real_pos + '\n';

                //string msg = X_tar_pos + X_real_pos + y_tar_pos + y_real_pos + z_tar_pos + z_real_pos + '\n';
                fwrite(msg.c_str(), sizeof(char), msg.size(), m_fd);
            }
        }
        else
        {
            if (m_fd)
            {
                fflush(m_fd);
                fclose(m_fd);
                m_fd = nullptr;
            }
        }
    }
#endif

    //处理PMC的告警，即A地址寄存器
    this->ProcessPmcAlarm();

}

/**
 * @brief 处理PMC数据窗口
 */
void ChannelEngine::ProcessPmcDataWnd(){

    FRegBits *freg = &m_p_pmc_reg->FReg().bits[0];
    const GRegBits *greg = &m_p_pmc_reg->GReg().bits[0];

    if(greg->ESTB && (freg->EREND==0)){  //ESTB信号为1，且EREND信号为0时，执行数据写入
        uint8_t cmd = greg->ED1;      //命令号
        uint8_t axis = greg->ED2;    //物理轴号

        if(cmd == 0x01){ //读取轴机械位置坐标
            freg->wnd_data = this->GetPhyAxisMachPosFeedback(axis)*1000;   //单位：um

            freg->EREND = 1;   //通知PMC读取数据

        }
    }else if((greg->ESTB == 0) && freg->EREND){
        freg->EREND = 0;   //复位PMC读取数据信号
    }
}

/**
 * @brief 处理PMC告警
 *
 */
void ChannelEngine::ProcessPmcAlarm(){
    uint8_t *alarmreg = this->m_p_pmc_reg->GetRegPtr8(PMC_REG_A);
    if(alarmreg == nullptr)
        return;

    //	uint64_t *pt = (uint64_t *)alarmreg;
    uint8_t bit = 0;
    uint8_t errtype = INFO_LEVEL;
    uint16_t errorno = 0;

#ifdef USES_PMC_2_0
    uint8_t value = 0;
    int count = A_REG_COUNT;
    for(int i = 0; i < count; i++){
        value = *alarmreg;
        if(value != 0){
            bit = 0;
            while(value != 0 && bit <8){
                if(value & 0x01){
                    errorno = 20000+i*8+bit;
                    if(errorno < 20080)
                        errtype = INFO_LEVEL;
                    else if(errorno < 20144)
                        errtype = WARNING_LEVEL;
                    else if(errorno < 20184)
                        errtype = ERROR_LEVEL;
                    else
                        errtype = FATAL_LEVEL;
                    CreateError(errorno, errtype, CLEAR_BY_AUTO, 0, CHANNEL_ENGINE_INDEX);
                }

                value = value>>1;  //右移一位
                ++bit;
            }
        }
        alarmreg += 1;  //pt++;
    }
#else
    uint64_t value = 0;
    int count = A_REG_COUNT/8;
    for(int i = 0; i < count; i++){
        //		value = *pt;
        memcpy(&value, alarmreg, 8);
        if(value != 0){
            bit = 0;
            while(value != 0 && bit <64){
                if(value & 0x01){
                    errorno = 20000+i*64+bit;
                    if(errorno < 20240)
                        errtype = INFO_LEVEL;
                    else if(errorno < 20400)
                        errtype = WARNING_LEVEL;
                    else if(errorno < 20480)
                        errtype = ERROR_LEVEL;
                    else
                        errtype = FATAL_LEVEL
                                CreateError(errorno, errtype, CLEAR_BY_AUTO, 0, CHANNEL_ENGINE_INDEX);
                }

                value = value>>1;  //右移一位
                ++bit;
            }
        }
        alarmreg += 8;  //pt++;
    }
#endif
    g_ptr_alarm_processor->ProcessAutoAlarm();
}

/**
 * @brief 处理PMC轴控制信号
 */
void ChannelEngine::ProcessPmcAxisCtrl(){
    //	FRegBits *freg0 = GetChnFRegBits(0);

    FRegBits *freg = nullptr;
    const GRegBits *greg = nullptr;
    //	uint8_t phy_axis = 0xFF;
    PmcAxisCtrlCmd pmc_cmd;

    for(uint8_t i = 0; i < kMaxChnCount; i++){
        greg = &m_p_pmc_reg->GReg().bits[i];
        freg = &m_p_pmc_reg->FReg().bits[i];

        //处理PMC轴通道激活
        static bool last_eax[kMaxChnCount][4] = {{false}, {false}, {false}, {false}};
        bool eax[4] = {greg->EAX1, greg->EAX2, greg->EAX3, greg->EAX4};
        bool embuf[4] = {greg->EMBUFA, greg->EMBUFB, greg->EMBUFC, greg->EMBUFD};
        uint8_t ebuf[4] = {greg->EBUFA, greg->EBUFB, greg->EBUFC, greg->EBUFD};
        uint8_t ebsy[4] = {freg->EBSYA, freg->EBSYB, freg->EBSYC, freg->EBSYD};
        bool eclr[4] = {greg->ECLRA, greg->ECLRB, greg->ECLRC, greg->ECLRD};
        bool estp[4] = {greg->ESTPA, greg->ESTPB, greg->ESTPC, greg->ESTPD};
        uint8_t cmd[4] = {greg->ECA, greg->ECB, greg->ECC, greg->ECD};
        int32_t dis[4] = {greg->EIDA, greg->EIDB, greg->EIDC, greg->EIDD};
        uint16_t spd[4] = {greg->EIFA, greg->EIFB, greg->EIFC, greg->EIFD};
        for(uint8_t j = 0; j < 4; j++){
            if(m_pmc_axis_ctrl[i*4+j].axis_list.size() == 0)
                continue;

            // 当前存在待读入的命令
            // 避免选通信号一打开就直接执行命令，需要报警
            if(ebuf[j] != ebsy[j] && !m_pmc_axis_ctrl[4*i+j].IsActive()
                    && eax[j]){
                this->m_error_code = ERR_PMC_AXIS_CTRL_CHANGE;
                CreateError(ERR_PMC_AXIS_CTRL_CHANGE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, i*4+j+1, CHANNEL_ENGINE_INDEX);
                memcpy(last_eax[i], eax, sizeof(eax));
                return;
            }

            bool ret = true;
            //0->1 选通PMC通道
            if (last_eax[i][j] == 0 && eax[j] == 1)
            {
                ret = m_pmc_axis_ctrl[i*4+j].Active(true);
            }
            //1->0 关闭PMC通道
            else if (last_eax[i][j] == 1 && eax[j] == 0)
            {
                ret = m_pmc_axis_ctrl[i*4+j].Active(false);
            }

            if(!ret){
                this->m_error_code = ERR_PMC_AXIS_CTRL_CHANGE;
                CreateError(ERR_PMC_AXIS_CTRL_CHANGE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, i*4+j+1, CHANNEL_ENGINE_INDEX);
                memcpy(last_eax[i], eax, sizeof(eax));
                return;
            }

            if(!m_pmc_axis_ctrl[4*i+j].IsActive()){

                continue;  //未激活
            }

            //EMBUFg缓冲无效信号
            this->m_pmc_axis_ctrl[4*i+j].SetBuffState(!embuf[j]);

            //ESBKg/EMSBKg  程序段停止信号/程序段停止无效信号

            //数据读取
            if(ebuf[j] != ebsy[j]){
                pmc_cmd.cmd = cmd[j];
                pmc_cmd.distance = dis[j];
                pmc_cmd.speed = spd[j];
                this->m_pmc_axis_ctrl[4*i+j].WriteCmd(pmc_cmd);
            }

            //ECLRg复位信号
            if(eclr[j])
                this->m_pmc_axis_ctrl[4*i+j].Reset();


            //ESTPg轴控制暂停信号
            this->m_pmc_axis_ctrl[4*i+j].Pause(estp[j]);

        }
        memcpy(last_eax[i], eax, sizeof(eax));
    }

}

/**
 * @brief 检查轴原点信号
 * @param phy_axis ： 物理轴号，0开始
 * @param dir ： 方向， 1--正向   -1--负向
 * @return  true--有信号    false--无信号
 */
bool ChannelEngine::CheckAxisRefBaseSignal(uint8_t phy_axis, int8_t dir){

    bool res = false;
    uint8_t chn = phy_axis/16;

    uint8_t index = phy_axis%16;

    const GRegBits *g_reg = &m_p_pmc_reg->GReg().bits[chn];

    if(g_reg->DEC & (0x01<<index)){
        res = true;
    }

    //	if(dir == DIR_POSITIVE){
    //		if(index < 8){
    //			if(g_reg->AXIS_PDEC1 & (0x01<<index)){
    //				res = true;
    //			}
    //		}else if(g_reg->AXIS_PDEC2 & (0x01<<(index-8))){
    //			res = true;
    //		}
    //	}else if(dir == DIR_NEGATIVE){
    //		if(index < 8){
    //			if(g_reg->AXIS_NDEC1 & (0x01<<index)){
    //				res = true;
    //			}
    //		}else if(g_reg->AXIS_NDEC2 & (0x01<<(index-8))){
    //			res = true;
    //		}
    //	}

    return res;

}

/**
 * @brief 检查轴原点精基准信号（IO信号）
 * @param phy_axis : 物理轴序号，0开始
 * @return  true--有信号    false--无信号
 */
bool ChannelEngine::CheckAxisRefSignal(uint8_t phy_axis){
    bool res = false;
    uint8_t chn = phy_axis/16;

    uint8_t index = phy_axis%16;

    const GRegBits *g_reg = &m_p_pmc_reg->GReg().bits[chn];

    if(g_reg->REF & (0x01<<index)){
        res = true;
    }
    return res;
}


/**
 * @brief 处理轴硬限位告警,所有轴减速停
 * @param dir : 硬限位方向，0--正向   1--负向
 */
void ChannelEngine::ProcessAxisHardLimit(uint8_t dir){

    //减速停处理
    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        this->m_p_channel_control[i].Pause();
    }

    //生成硬限位错误信息
    uint32_t info = 0x00;
    if(dir == 0){
        m_error_code = ERR_HARDLIMIT_POS;
        info = this->m_hard_limit_postive;
    }
    else{
        m_error_code = ERR_HARDLIMIT_NEG;
        info = m_hard_limit_negative;
    }
    CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, info);
    //	printf("create axis hard limit error \n");
}

/**
 * @brief 设置PMC中F寄存器的OUT输出点位
 * @param index : OUT输出的点位序号，从0开始
 * @param value : 0--开关属性，每次取反      1--有信号      2--无信号
 */
void ChannelEngine::SetPmcOutSignal(uint8_t index, uint8_t value){
    if(index >= 32 || this->m_b_emergency)
        return;
    uint8_t sec = index/8;
    uint8_t bit = index%8;

    //	uint8_t cur_val = this->m_p_pmc_reg->FReg().all[72+sec];
    //	printf("SetPmcOutSignal: index = %hhu, value=0x%hhx\n", index, cur_val);
    //	cur_val ^= (0x01<<bit);  //取反

    //	this->m_p_pmc_reg->FReg().all[72+sec] = cur_val;
    //	printf("SetPmcOutSignal new : index = %hhu, value=0x%hhx\n", index, cur_val);
    if(value == 1){
        this->m_p_pmc_reg->FReg().all[72+sec] |= (0x01<<bit);
    }else if(value == 2){
        this->m_p_pmc_reg->FReg().all[72+sec] &= ~(0x01<<bit);
    }else if(value == 0){
        uint8_t cur_val = this->m_p_pmc_reg->FReg().all[72+sec];
        //	printf("SetPmcOutSignal: index = %hhu, value=0x%hhx\n", index, cur_val);
        cur_val ^= (0x01<<bit);  //取反

        this->m_p_pmc_reg->FReg().all[72+sec] = cur_val;

    }

}

/**
 * @brief 获取宏变量的值
 * @param chn[in] : 通道号
 * @param index[in]	：变量索引号
 * @param init[out] : 返回变量初始化状态，0--未初始化   1--已初始化
 * @param value[out] : 返回的变量值
 * @return
 */
bool ChannelEngine::GetMacroVarValue(uint8_t chn, uint32_t index, bool &init, double &value){
    //
    if(chn >= m_p_general_config->chn_count)
        return false;

    if(!m_p_channel_control[chn].GetMacroVar()->GetVarValue(index, value, init))
        return false;


    return true;
}

/**
 * @brief 设置宏变量的值
 * @param chn
 * @param index
 * @param init : 初始化状态，0--未初始化   1--已初始化
 * @param value : 变量值
 * @return
 */
bool ChannelEngine::SetMacroVarValue(uint8_t chn, uint32_t index, bool init, double value){
    if(chn >= m_p_general_config->chn_count)
        return false;

    if((this->m_mask_import_param & (0x01<<CONFIG_MACRO_VAR)) != 0)
        return false;

    if(init){//已初始化
        if(!m_p_channel_control[chn].GetMacroVar()->SetVarValue(index, value))
            return false;
    }else{//未初始化
        if(!m_p_channel_control[chn].GetMacroVar()->ResetVariable(index))
            return false;
    }

    return true;
}

/**
 * @brief 获取PMC单字节寄存器的值
 * @param reg_sec
 * @param reg_index
 * @param value
 * @return  true--成功   false--失败
 */
bool ChannelEngine::GetPmcRegValue_8(uint16_t reg_sec, uint16_t reg_index, uint8_t &value){
    bool res = false;

    res = this->m_p_pmc_reg->GetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, value);

    return res;
}

/**
 * @brief 获取PMC双字节寄存器的值
 * @param reg_sec
 * @param reg_index
 * @param value
 * @return true--成功   false--失败
 */
bool ChannelEngine::GetPmcRegValue_16(uint16_t reg_sec, uint16_t reg_index, uint16_t &value){
    bool res = false;

    res = this->m_p_pmc_reg->GetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, value);

    return res;
}

/**
 * @brief 设置PMC单字节寄存器的值
 * @param reg_sec
 * @param reg_index
 * @param value
 * @return true--成功   false--失败
 */
bool ChannelEngine::SetPmcRegValue_8(uint16_t reg_sec, uint16_t reg_index, uint8_t value){
    bool res = false;

    res = this->m_p_pmc_reg->SetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, value);

    return res;
}

/**
 * @brief 按位设置寄存器值
 * @param reg_sec
 * @param reg_index
 * @param bit_index : bit位序号，0开始
 * @param value
 * @return true--成功   false--失败
 */
bool ChannelEngine::SetPmcRegBitValue_8(uint16_t reg_sec, uint16_t reg_index, uint8_t bit_index, bool value){
    bool res = false;

    if(bit_index > 7)
        return res;

    uint8_t data = 0;
    res = this->m_p_pmc_reg->GetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, data);

    if(!res)
        return res;  //失败返回

    if(value){
        data |= 0x01<<bit_index;      //置1
    }else{
        data &= ~(0x01<<bit_index);   //置0
    }

    res = this->m_p_pmc_reg->SetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, data);
    return res;
}

/**
 * @brief 设置PMC双字节寄存器的值
 * @param reg_sec
 * @param reg_index
 * @param value
 * @return true--成功   false--失败
 */
bool ChannelEngine::SetPmcRegValue_16(uint16_t reg_sec, uint16_t reg_index, uint16_t value){
    bool res = false;

    res = this->m_p_pmc_reg->SetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, value);

    return res;
}

/**
 * @brief 设置通道的PMC信号
 * @param chn_index : 通道号
 * @param signal ：信号定义
 * @param flag ： true--置一    false--置零
 */
void ChannelEngine::SetPmcSignal(uint8_t chn_index, int signal, bool flag){

}

/**
 * @brief 刷新nc文件file
 * @param file
 */
void ChannelEngine::RefreshFile(char *file){
    for(int i = 0; i < this->m_p_general_config->chn_count; i++)
        this->m_p_channel_control[i].RefreshFile(file);
}

/**
 * @brief 重新加载文件file
 * @param file
 */
void ChannelEngine::RemapFile(char *file){
    for(int i = 0; i < this->m_p_general_config->chn_count; i++)
        this->m_p_channel_control[i].RemapFile(file);
}

/**
 * @brief 复位OP信号
 * @param chn_index
 */
void ChannelEngine::ResetOPSignal(uint8_t chn_index){
    if(chn_index != CHANNEL_ENGINE_INDEX){
        this->m_p_channel_control[chn_index].ResetOPSignal();
    }else{
        for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++)
            this->m_p_channel_control[i].ResetOPSignal();
    }
}

/**
 * @brief 设置告警信号
 * @param chn_index
 * @param value
 */
void ChannelEngine::SetALSignal(uint8_t chn_index, bool value){

    if(chn_index != CHANNEL_ENGINE_INDEX){
        this->m_p_channel_control[chn_index].SetALSignal(value);
    }else{
        for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
            this->m_p_channel_control[i].SetALSignal(value);
        }
    }
}


/**
 * @brief 获取指定PMC通道的PMC轴序号
 * @param pmc_chn : PMC通道号，即寄存器组序号，从0开始
 * @return 成功返回物理轴号，失败则返回0xFF
 */
uint8_t ChannelEngine::GetPmcAxis(uint8_t pmc_chn){

    uint8_t phy_axis = 0xff;
    if(this->m_n_pmc_axis_count == 0)
        return phy_axis;

    uint8_t count = 0;
    for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
        if(this->m_p_axis_config[i].axis_pmc == 0)
            continue;
        if(m_p_axis_config[i].axis_pmc == pmc_chn+1){
            phy_axis = i;
            break;
        }
        if(++count == m_n_pmc_axis_count)
            break;
    }

    return phy_axis;
}

/**
 * @brief 仅根据原点信号设置参考点 包括步进电机根据原点信号设置参考点
 */
void ChannelEngine::AxisFindRefWithZeroSignal(uint8_t phy_axis){

    int8_t dir = 0, dir_opt;   //回参考点方向
    //	uint8_t ret_mode = 0;   //回参考点方式
    uint8_t chn = 0, chn_axis = 0;

    dir = this->m_p_axis_config[phy_axis].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;  // 回参考点找粗基准方向
    dir_opt = (m_p_axis_config[phy_axis].ret_ref_change_dir==0)?dir*-1:dir;       //回参考点找精基准方向

    switch(this->m_n_ret_ref_step[phy_axis]){
    case 0://检查原点信号
        printf("return ref step 0, dir = %hhd, dir_opt=%hhd\n", dir, dir_opt);
        this->SetRetRefFlag(phy_axis, false);   //复位回参考点完成标志
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point &= ~(0x01<<phy_axis);

        if(this->CheckAxisRefBaseSignal(phy_axis, dir)){  // 已经触发粗基准信号，直接开开始回退
            double move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;
            if(move_length>10.0) {
                move_length = 10.0;
            }
            else if(move_length < 1.0){
                move_length = 1.0;
            }
            m_df_ret_ref_tar_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir*-1;
            this->ManualMove(phy_axis, dir*-1,
                             this->m_p_axis_config[phy_axis].ret_ref_speed, move_length);

            m_n_ret_ref_step[phy_axis] = 1;
        }
        else{
            m_n_ret_ref_step[phy_axis] = 2;
        }

        break;
    case 1:{//等待回退到位，并再次检查原点信号
        printf("return ref step 1\n");
        chn = this->GetAxisChannel(phy_axis, chn_axis);
        bool flag = CheckAxisRefBaseSignal(phy_axis, dir);

        if(chn != CHANNEL_ENGINE_INDEX){
            if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - m_df_ret_ref_tar_pos[phy_axis]) <= 0.010){
                if(true == flag){
                    m_n_ret_ref_step[phy_axis] = 0;
                    break;
                }
            }
            if(flag == false){
                m_n_ret_ref_step[phy_axis] = 2;  //跳转下一步
            }

        }else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC轴到位
            if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
                if(true == flag){
                    m_n_ret_ref_step[phy_axis] = 0;
                    break;
                }else{
                    m_n_ret_ref_step[phy_axis] = 2;  //跳转下一步
                }
            }
        }
    }
        break;
    case 2:  //以回参考点速度向回参考点方向运动
        //		printf("return ref step 2\n");
        this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed, 90000.0);
        m_n_ret_ref_step[phy_axis] = 3;  //跳转下一步
        printf("return ref, goto step 3\n");
        break;
    case 3:  //触发原点信号，停止
        //		printf("return ref step 3\n");
        if(this->CheckAxisRefBaseSignal(phy_axis, dir)){
            this->ManualMoveStop(phy_axis);
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //记录起始时间
            m_n_ret_ref_step[phy_axis] = 4;  //跳转下一步
            printf("return ref, goto step 4: pos = %lf\n", this->GetPhyAxisMachPosFeedback(phy_axis));
        }
        else{
            //	this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed,3.0);
        }
        break;
    case 4:{//等待停止到位
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 500000){ //延时500ms
            m_n_ret_ref_step[phy_axis] = 5;  //跳转下一步
            printf("return ref, goto step 5: pos = %lf, time=%u\n", this->GetPhyAxisMachPosFeedback(phy_axis), time_elpase);
        }
        if(!this->CheckAxisRefBaseSignal(phy_axis, dir)){
            printf("回参考点过冲了！！！！！！！\n");
        }
    }
        break;
    case 5:  //有基准， 低速回退至原点信号消失,
        printf("return ref step 5\n");
        this->ManualMove(phy_axis, dir_opt, this->m_p_axis_config[phy_axis].ret_ref_speed_second, this->m_p_axis_config[phy_axis].move_pr*2);//60
        m_n_ret_ref_step[phy_axis] = 6;  //跳转下一步
        printf("return ref, goto step 6\n");

        break;
    case 6:  //有基准，等待粗基准信号消失
        //	printf("return ref step 6\n");
        if(!this->CheckAxisRefBaseSignal(phy_axis, dir)){
            this->ManualMoveStop(phy_axis);

            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //记录起始时间，延时300ms

            m_n_ret_ref_step[phy_axis] = 7;  //跳转下一步
        }
        break;
    case 7:{ //等待停止到位
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 500000){ //延时500ms
            //				m_n_get_cur_encoder_count = 0;  //复位获取当前编码器次数
            m_n_ret_ref_step[phy_axis] = 8;  //跳转下一步
            printf("return ref, goto step 8\n");
        }
    }
        break;

    case 8:{ //  设置原点
        printf("return ref step 8: send cmd to mi, set ref point\n");
        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.axis_index = phy_axis+1;
        cmd.data.cmd = CMD_MI_SET_REF_CUR;

        int64_t pos = m_p_axis_config[phy_axis].axis_home_pos[0] * 1e7;   //单位转换，0.1nm
        memcpy(cmd.data.data, &pos, sizeof(int64_t));
        this->m_p_mi_comm->WriteCmd(cmd);
        gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //记录起始时间，延时300ms

        m_n_ret_ref_step[phy_axis] = 9;  //跳转下一步
        printf("return ref, goto step 9\n");

    }
        break;
    case 9: { //等待设置参考点完成，在Mi指令响应处理函数中处理
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 300000){ //延时300ms
            m_n_ret_ref_step[phy_axis] = 10;  //跳转下一步
            printf("return ref, goto step 10\n");
        }
    }
        break;
    case 10:  //回参考点完成
        printf("axis %u return ref over\n", phy_axis);
        this->SetRetRefFlag(phy_axis, true);
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //置位到参考点标志


        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        break;
    case 20: //失败处理
        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        m_error_code = ERR_RET_REF_FAILED;
        CreateError(ERR_RET_REF_FAILED, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis+1);
        break;
    default:
        break;
    }
}

/**
 * @brief 设置轴名称扩展下标使能
 * @param flag : 允许轴名称下标
 */
void ChannelEngine::SetAxisNameEx(bool flag){
    for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
        this->m_p_channel_control[i].SetAxisNameEx(flag);
    }
}

/**
 * @brief 当前位置设置为参考点   虚拟轴回参考点  步进电机设置参考点
 */
void ChannelEngine::AxisFindRefNoZeroSignal(uint8_t phy_axis){

    switch(this->m_n_ret_ref_step[phy_axis]){
    case 0: {// 设置原点
        std::cout << "ChannelEngine::AxisFindRefNoZeroSignal " << (int)phy_axis << std::endl;
        std::cout << "step 0: CMD_MI_SET_REF_CUR 0x0108\n";
        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.axis_index = phy_axis+1;
        cmd.data.cmd = CMD_MI_SET_REF_CUR;

        int64_t pos = m_p_axis_config[phy_axis].axis_home_pos[0] * 1e7;   //单位转换，0.1nm
        memcpy(cmd.data.data, &pos, sizeof(int64_t));
        this->m_p_mi_comm->WriteCmd(cmd);
        gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //记录起始时间，延时300ms

        if (GetSyncAxisCtrl()->CheckSyncState(phy_axis) == 1)
        {//主动轴相应的从动轴也需要清除回零标志
            ClearSubAxisRefFlag(phy_axis);
        }
        m_n_ret_ref_step[phy_axis] = 1;  //跳转下一步
    }
        break;
    case 1: { //等待设置参考点完成，在Mi指令响应处理函数中处理
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            m_n_ret_ref_step[phy_axis] = 2;  //跳转下一步
            std::cout << "step 1, waitfor 200ms\n";
        }
    }
        break;
    case 2:  //回参考点完成
        this->SetRetRefFlag(phy_axis, true);
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //置位到参考点标志
        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if (GetSyncAxisCtrl()->CheckSyncState(phy_axis) == 1)
        {//从动轴建立机械坐标
            SetSubAxisRefPoint(phy_axis, 0);
        }

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        std::cout << "step 2, ref finish\n";

        break;
    case 20: //失败处理
        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        std::cout << "step 20, ref failed\n";
        m_error_code = ERR_RET_REF_FAILED;
        CreateError(ERR_RET_REF_FAILED, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis+1);
        break;
    default:
        break;
    }
}

/**
 * @brief 总线增量式轴有基准回参考点，根据Z信号作为精基准
 * @param phy_axis : 物理轴号，从0开始
 */
void ChannelEngine::EcatIncAxisFindRefWithZeroSignal(uint8_t phy_axis){
    int8_t dir = 0, dir_opt;   //回参考点方向
    uint8_t chn = 0, chn_axis = 0;
    double dis = 0;      //移动距离

    dir = this->m_p_axis_config[phy_axis].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;  // 回参考点找粗基准方向
    dir_opt = (m_p_axis_config[phy_axis].ret_ref_change_dir==0)?dir*-1:dir;       //回参考点找精基准方向

    switch(this->m_n_ret_ref_step[phy_axis]){
    case 0://检查原点信号
        std::cout << "ChannelEngine::EcatIncAxisFindRefWithZeroSignal " << (int)phy_axis << std::endl;
        this->SetRetRefFlag(phy_axis, false);//清除建立参考点标志
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point &= ~(0x01<<phy_axis);

        if (GetSyncAxisCtrl()->CheckSyncState(phy_axis) == 1)
        {//主动轴相应的从动轴也需要清除回零标志
            ClearSubAxisRefFlag(phy_axis);
        }

        if(this->CheckAxisRefBaseSignal(phy_axis, dir))
        {// 已经触发粗基准信号
            double move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;
            if(move_length>10.0) {
                move_length = 10.0;
            }
            else if(move_length < 1.0){
                move_length = 1.0;
            }
            m_df_ret_ref_tar_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir*-1;
            this->ManualMove(phy_axis, dir*-1,
                             this->m_p_axis_config[phy_axis].ret_ref_speed, move_length);

            std::cout << "step 0, dir: " << (int)dir << " dir_opt: " << dir_opt << " goto step 1 " << std::endl;
            m_n_ret_ref_step[phy_axis] = 1;
        }
        else
        {
            std::cout << "step 0, dir: " << (int)dir << " dir_opt: " << dir_opt << " goto step 2 " << std::endl;
            m_n_ret_ref_step[phy_axis] = 2;
        }

        break;
    case 1:{//等待回退到位，并再次检查原点信号
        chn = this->GetAxisChannel(phy_axis, chn_axis);
        bool flag = CheckAxisRefBaseSignal(phy_axis, dir);

        if(chn != CHANNEL_ENGINE_INDEX){
            if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - m_df_ret_ref_tar_pos[phy_axis]) <= 0.010){
                if(true == flag){   //还处于粗基准信号上，继续回退
                    m_n_ret_ref_step[phy_axis] = 0;
                    std::cout << "step 1, goback 0" << std::endl;
                    break;
                }
            }
            if(flag == false){
                m_n_ret_ref_step[phy_axis] = 2;
                std::cout << "step 1" << std::endl;
            }

        }else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC轴到位
            if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
                if(true == flag){
                    m_n_ret_ref_step[phy_axis] = 0;
                    break;
                }else{
                    m_n_ret_ref_step[phy_axis] = 2;
                }
            }
        }
    }
        break;
    case 2:  //以回参考点速度向回参考点方向运动
        this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed, 90000.0);
        m_n_ret_ref_step[phy_axis] = 3;
        std::cout << "step 2, ret_ref_speed: " << this->m_p_axis_config[phy_axis].ret_ref_speed << "dir: " << (int)dir << std::endl;
        break;
    case 3:  //触发粗基准信号，停止
        if(this->CheckAxisRefBaseSignal(phy_axis, dir)){
            this->ManualMoveStop(phy_axis);
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            m_n_ret_ref_step[phy_axis] = 4;
            this->SendMonitorData(false, false);
            std::cout << "step 3, find signal pos: " << this->GetPhyAxisMachPosIntp(phy_axis) << std::endl;
        }
        else{
            //	this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed,1.0);
        }
        break;
    case 4:{//等待停止到位
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 1000000){ //延时1000ms
            m_n_ret_ref_step[phy_axis] = 5;
            this->SendMonitorData(false, false);
            std::cout << "step 4, wait for 1000ms, get signal pos: " << this->GetPhyAxisMachPosIntp(phy_axis) << std::endl;
        }
    }
        break;
    case 5:{  //有基准， 低速回退至粗基准信号消失,
        double move_length = this->m_p_axis_config[phy_axis].move_pr*2;
        if(move_length>10.0) {
            move_length = 10.0;
        }else if(move_length < 1.0){
            move_length = 1.0;
        }

        double val = 60;
        if (this->m_p_axis_config[phy_axis].ret_ref_speed_second > 0)
            val = this->m_p_axis_config[phy_axis].ret_ref_speed_second;
        m_df_ret_ref_tar_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir_opt;
        this->ManualMove(phy_axis, dir_opt, val, move_length); //有基准， 低速回退至粗基准信号消失
        m_n_ret_ref_step[phy_axis] = 6;
        std::cout << "step 5," << " speed: " << val << std::endl;

        break;
    }
    case 6:  //有基准，等待粗基准信号消失
        if(!this->CheckAxisRefBaseSignal(phy_axis, dir)){
            this->ManualMoveStop(phy_axis);

            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);

            m_n_ret_ref_step[phy_axis] = 7;
            std::cout << "step 6, goto 7" << std::endl;
        }else if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - m_df_ret_ref_tar_pos[phy_axis]) <= 0.010){  //接近目标位置，粗基准信号还在，则继续回退
            m_n_ret_ref_step[phy_axis] = 5;
            std::cout << "step 6, goback 5" << std::endl;
        }
        break;
    case 7:{ //等待停止到位，并向MI发送激活捕获Z信号命令
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            if(!this->CheckAxisRefBaseSignal(phy_axis, dir)){
                this->SendMiAxisZCaptureCmd(phy_axis, true);  //激活Z信号捕获
                m_n_ret_ref_step[phy_axis] = 8;
                std::cout << "step 7, wait for 600ms, CMD_MI_ACTIVE_Z_CAPT 0x0109" << std::endl;
            }else
            {//如果又发现粗基准有信号，则继续回退
                m_n_ret_ref_step[phy_axis] = 5;
                std::cout << "step 7, wait for 600ms, goback 5" << std::endl;
            }

        }
    }
        break;
    case 8:{ //向寻找精基准方向运动，以捕获Z信号
        double move_length = 0;
        if (this->m_p_axis_config[phy_axis].ref_z_distance_max == 0)
            move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;
        else
            move_length = this->m_p_axis_config[phy_axis].ref_z_distance_max;

        m_df_ret_ref_tar_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis) + move_length * dir_opt;

        double val = 60;
        if (this->m_p_axis_config[phy_axis].ret_ref_speed_second > 0)
            val = this->m_p_axis_config[phy_axis].ret_ref_speed_second;
        this->ManualMove(phy_axis, dir_opt, val, move_length);

        m_n_ret_ref_step[phy_axis] = 9;
        std::cout << "step 8, moveLength: " << move_length << " val " << val << std::endl;
    }
        break;
    case 9:{ //等待Z信号捕获完成或者移动到位捕获失败，在Mi指令响应处理函数中处理
        if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - m_df_ret_ref_tar_pos[phy_axis]) <= 0.010){  //运行到位还是未找到Z信号
            std::cout << "step 9, not find z singal" << std::endl;
            this->SendMiAxisZCaptureCmd(phy_axis, false);
            m_n_ret_ref_step[phy_axis] = 20;   //跳转失败处理
        }
    }
        break;
    case 10:{ //延时1000ms，等待停止到位
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 1000000){ //延时1000ms
            MiCmdFrame cmd;
            memset(&cmd, 0x00, sizeof(cmd));
            cmd.data.axis_index = phy_axis+1;
            cmd.data.cmd = CMD_MI_SET_REF;
            memcpy(cmd.data.data, &m_p_axis_config[phy_axis].ref_encoder, sizeof(int64_t));

            this->m_p_mi_comm->WriteCmd(cmd);

            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 10, get z signal, wait for 2500ms, CMD_MI_SET_REF 0x0107" << std::endl;
        }
        break;
    }

    case 11:{
        if (GetSyncAxisCtrl()->CheckSyncState(phy_axis) == 1)
        {
            struct timeval time_now;
            gettimeofday(&time_now, NULL);
            unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
            if (time_elpase >= 1000000) //延时1000ms,等待轴停稳
            {
                SetSubAxisRefPoint(phy_axis, 0);
                gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
                m_n_ret_ref_step[phy_axis]++;
                std::cout << "step 11, wait for 1000ms" << std::endl;
            }
        }
        else
        {
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 11" << std::endl;
        }
    }
        break;

    case 12:{   // 零点偏移量
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            if (this->m_p_axis_config[phy_axis].ref_offset_pos)
            {//零点偏移量
                double val = 60;
                if (this->m_p_axis_config[phy_axis].ret_ref_speed_second > 0)
                    val = this->m_p_axis_config[phy_axis].ret_ref_speed_second;
                this->ManualMoveAbs(phy_axis, val, this->m_p_axis_config[phy_axis].ref_offset_pos);
                m_n_ret_ref_step[phy_axis] = 13;
            }
            else
            {
                this->ManualMoveAbs(phy_axis, m_p_axis_config[phy_axis].ret_ref_speed, m_p_axis_config[phy_axis].axis_home_pos[0]);
                m_n_ret_ref_step[phy_axis] = 17;
            }
            std::cout << "step 12, wait for 200ms, ref_offset_pos: " << this->m_p_axis_config[phy_axis].ref_offset_pos << std::endl;
        }
        break;
    }

    case 13: {  // 等待运动停止
        if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - this->m_p_axis_config[phy_axis].ref_offset_pos) <= 0.010){  //到位
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 13, move over" << std::endl;
        }
    }
        break;
    case 14: {
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 600000){ //延时600ms
            MiCmdFrame mi_cmd;
            memset(&mi_cmd, 0x00, sizeof(mi_cmd));
            mi_cmd.data.cmd = CMD_MI_SET_AXIS_MACH_POS;
            mi_cmd.data.axis_index = phy_axis+1;
            mi_cmd.data.data[0] = 0;
            mi_cmd.data.data[1] = 0;
            this->m_p_mi_comm->WriteCmd(mi_cmd);
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 14, wait for 600ms, CMD_MI_SET_AXIS_MACH_POS 0x136" << std::endl;
        }
    }
        break;

    case 15: {
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000) {//延时200ms,等待MI处理
            if (GetSyncAxisCtrl()->CheckSyncState(phy_axis) == 1)
            {//从动轴建立机械坐标
                SetSubAxisRefPoint(phy_axis, 0);
            }
            m_n_ret_ref_step[phy_axis]++;
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            std::cout << "step 15, wait for 200ms" << std::endl;
        }
    }
        break;

    case 16:{
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
#ifdef USES_RET_REF_TO_MACH_ZERO
            this->ManualMoveAbs(phy_axis, m_p_axis_config[phy_axis].ret_ref_speed, 0);
#else
            this->ManualMoveAbs(phy_axis, m_p_axis_config[phy_axis].ret_ref_speed, m_p_axis_config[phy_axis].axis_home_pos[0]);
#endif
            m_n_ret_ref_step[phy_axis]++;  //跳转下一步
            std::cout << "step 16, axis_home_pos:" << m_p_axis_config[phy_axis].axis_home_pos[0] << std::endl;
        }
        break;
    }
    case 17:
#ifdef USES_RET_REF_TO_MACH_ZERO
        dis = 0;    //机械零
#else
        dis = this->m_p_axis_config[phy_axis].axis_home_pos[0];   // 原点坐标
#endif
        this->SendMonitorData(false, false);  //再次读取实时位置

        if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis)- dis) <= 0.010){  //到位
            m_n_ret_ref_step[phy_axis]++;  //跳转下一步
            std::cout << "step 17" << std::endl;
        }
        break;
    case 18:  //回参考点完成
        printf("axis %u return ref over\n", phy_axis);
//        if(this->m_p_axis_config[phy_axis].feedback_mode == ABSOLUTE_ENCODER ||
//            this->m_p_axis_config[phy_axis].feedback_mode == LINEAR_ENCODER){  //绝对值需获取机械零点的编码器值并保存，方便断点后恢复坐标
//                MiCmdFrame mi_cmd;
//                memset(&mi_cmd, 0x00, sizeof(mi_cmd));
//                mi_cmd.data.cmd = CMD_MI_GET_ZERO_ENCODER;
//                mi_cmd.data.axis_index = phy_axis+1;

//                this->m_p_mi_comm->WriteCmd(mi_cmd);
//        }
        this->SetRetRefFlag(phy_axis, true);
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //置位到参考点标志

        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        std::cout << "step 18, home finish" << std::endl;
        break;
    case 20: //失败处理
        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        std::cout << "step 20, home failed" << std::endl;
        m_error_code = ERR_RET_REF_FAILED;
        CreateError(ERR_RET_REF_FAILED, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis+1);
        break;
    default:
        break;
    }
}

/**
 * @brief 总线增量式无基准回参考点，仅根据Z信号作为精基准回零
 * @param phy_axis : 物理轴号，从0开始
 */
void ChannelEngine::EcatIncAxisFindRefNoZeroSignal(uint8_t phy_axis){
    int8_t dir = 0;   //回参考点方向
    double dis = 0;      //移动距离

    dir = this->m_p_axis_config[phy_axis].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;  // 回参考点方向

    switch(this->m_n_ret_ref_step[phy_axis]){
    case 0: {// 激活Z信号捕获，并向回零方向运动1.2个螺距，以捕获Z信号
        printf("return ref step 0: send cmd to mi, set ref point\n");
        this->SendMiAxisZCaptureCmd(phy_axis, true);  //激活Z信号捕获

        double move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;

        m_df_ret_ref_tar_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir;
        this->ManualMove(phy_axis, dir, 60, move_length);

        m_n_ret_ref_step[phy_axis] = 1;  //跳转下一步
        printf("return ref, goto step 1\n");

    }
        break;
    case 1:  //等待Z信号捕获完成或者移动到位捕获失败，在Mi指令响应处理函数中处理
        if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - m_df_ret_ref_tar_pos[phy_axis]) <= 0.010){  //运行到位还是未找到Z信号
            this->SendMiAxisZCaptureCmd(phy_axis, false);
            m_n_ret_ref_step[phy_axis] = 20;   //跳转失败处理
        }

        break;

    case 10:{ //延时300ms，等待停止到位
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 300000){ //延时300ms
            MiCmdFrame cmd;
            memset(&cmd, 0x00, sizeof(cmd));
            cmd.data.axis_index = phy_axis+1;
            cmd.data.cmd = CMD_MI_SET_REF;
            memcpy(cmd.data.data, &m_p_axis_config[phy_axis].ref_encoder, sizeof(int64_t));

            this->m_p_mi_comm->WriteCmd(cmd);
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //记录起始时间
            m_n_ret_ref_step[phy_axis]++;  //跳转下一步
        }
        break;
    }
    case 11:{  //延时100ms，保证机械坐标已更新
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 100000){ //延时100ms
#ifdef USES_RET_REF_TO_MACH_ZERO
            this->ManualMoveAbs(phy_axis, m_p_axis_config[phy_axis].ret_ref_speed, 0);
#else
            this->ManualMoveAbs(phy_axis, m_p_axis_config[phy_axis].ret_ref_speed, m_p_axis_config[phy_axis].axis_home_pos[0]);
#endif
            m_n_ret_ref_step[phy_axis]++;  //跳转下一步
        }
        break;
    }
    case 12:
#ifdef USES_RET_REF_TO_MACH_ZERO
        dis = 0;    //机械零
#else
        dis = this->m_p_axis_config[phy_axis].axis_home_pos[0];   // 原点坐标
#endif
        this->SendMonitorData(false, false);  //再次读取实时位置

        if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis)- dis) <= 0.010){  //到位
            m_n_ret_ref_step[phy_axis]++;  //跳转下一步
        }
        //	printf("return ref, goto step 11\n");
        break;
    case 13:  //回参考点完成
        printf("axis %u return ref over\n", phy_axis);
        if(this->m_p_axis_config[phy_axis].feedback_mode == ABSOLUTE_ENCODER ||
                this->m_p_axis_config[phy_axis].feedback_mode == LINEAR_ENCODER){  //绝对值需要发送设置参考点指令
            MiCmdFrame mi_cmd;
            memset(&mi_cmd, 0x00, sizeof(mi_cmd));
            mi_cmd.data.cmd = CMD_MI_GET_ZERO_ENCODER;
            mi_cmd.data.axis_index = phy_axis+1;

            this->m_p_mi_comm->WriteCmd(mi_cmd);
        }
        this->SetRetRefFlag(phy_axis, true);
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //置位到参考点标志

        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        break;
    case 20: //失败处理
        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        m_error_code = ERR_RET_REF_FAILED;
        CreateError(ERR_RET_REF_FAILED, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis+1);
        break;
    default:
        break;
    }
}

/**
 * @brief 设置从动轴回零
 * @param axisID : 主动轴Id
 */
void ChannelEngine::SetSubAxisRefPoint(int axisID, double refPos)
{
    int axisMask = GetSyncAxisCtrl()->GetSlaveAxis(axisID);
    for (int i = 0; i < this->m_p_general_config->axis_count; ++i) {
        if(axisMask & (0x01<<i)){
            MiCmdFrame mi_cmd;//主动轴相应的从动轴也需要设置当前位置为零点
            memset(&mi_cmd, 0x00, sizeof(mi_cmd));
            mi_cmd.data.cmd = CMD_MI_SET_AXIS_MACH_POS;
            mi_cmd.data.axis_index = i+1;

            int64_t pos = (refPos + m_p_axis_config[i].benchmark_offset) * 1000;
            mi_cmd.data.data[0] = pos&0xFFFF;
            mi_cmd.data.data[1] = (pos>>16)&0xFFFF;

            std::cout << "sub axis_index: " << (int)mi_cmd.data.axis_index << std::endl;
            std::cout << "CMD_MI_SET_AXIS_MACH_POS 0x136 " << "pos-> : " << pos << std::endl;
            this->m_p_mi_comm->WriteCmd(mi_cmd);

            this->SetRetRefFlag(i, true);
            this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<i);   //置位到参考点标志
        }
    }
}

/**
 * @brief 清除重动轴回零标志
 * @param axisID : 主动轴Id
 */
void ChannelEngine::ClearSubAxisRefFlag(int axisID)
{
    int axisMask = GetSyncAxisCtrl()->GetSlaveAxis(axisID);
    for (int i = 0; i < this->m_p_general_config->axis_count; ++i) {
        if(axisMask & (0x01<<i)){
            this->SetRetRefFlag(i, false);
        }
    }
}


/**
 * @brief 清除绝对式零点编码器值
 * @param axisID : 轴ID
 */
void ChannelEngine::ClearAxisRefEncoder(int axisID)
{
    if (axisID > 0 && axisID <m_p_general_config->axis_count)
    {
        this->m_p_axis_config[axisID].ref_encoder = kAxisRefNoDef;
        g_ptr_parm_manager->UpdateAxisRef(axisID, kAxisRefNoDef);
        this->NotifyHmiAxisRefChanged(axisID);

        int axisMask = GetSyncAxisCtrl()->GetSlaveAxis(axisID);
        for (int i = 0; i < this->m_p_general_config->axis_count; ++i) {
            if(axisMask & (0x01<<i)){
                std::cout << "ChannelEngine::ClearAxisRefEncoder " << axisID << std::endl;
                this->m_p_axis_config[i].ref_encoder = kAxisRefNoDef;
                g_ptr_parm_manager->UpdateAxisRef(i, kAxisRefNoDef);
                this->NotifyHmiAxisRefChanged(i);
            }
        }
    }
}

/**
 * @brief 向MI发送激活捕获Z信号命令
 * @param phy_axis : 物理轴号， 从0开始
 * @param active ： true--激活   false--关闭
 */
void ChannelEngine::SendMiAxisZCaptureCmd(uint8_t phy_axis, bool active){
    if(phy_axis >= this->m_p_general_config->axis_count)  //超出了轴数量上限
        return;

    MiCmdFrame cmd;
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_ACTIVE_Z_CAPT;

    cmd.data.axis_index = phy_axis+1;
    cmd.data.reserved = active?1:0;

    cmd.data.data[0] = this->m_p_axis_config[phy_axis].axis_interface==BUS_AXIS?1:0;   //1:总线轴   0：非总线轴

    this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief 脉冲输出有基准轴回参考点
 */
void ChannelEngine::PulseAxisFindRefWithZeroSignal(uint8_t phy_axis){

    int8_t dir = 0, dir_opt;   //回参考点方向
    //	uint8_t ret_mode = 0;   //回参考点方式
    uint8_t chn = 0, chn_axis = 0;
    double dis = 0;      //移动距离
    static double ret_ref_record_pos[kMaxAxisNum];

    //	ret_mode = m_p_axis_config[phy_axis].ret_ref_mode;
    dir = this->m_p_axis_config[phy_axis].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;  // 回参考点找粗基准方向
    dir_opt = (m_p_axis_config[phy_axis].ret_ref_change_dir==0)?dir*-1:dir;       //回参考点找精基准方向

    switch(this->m_n_ret_ref_step[phy_axis]){
    case 0://检查原点信号
        printf("return ref step 0, dir = %hhd, dir_opt=%hhd\n", dir, dir_opt);
        this->SetRetRefFlag(phy_axis, false);   //复位回参考点完成标志
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point &= ~(0x01<<phy_axis);

        if(this->CheckAxisRefBaseSignal(phy_axis, dir)){  // 已经触发粗基准信号，直接开开始回退
            double move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;
            if(move_length>5.0) {
                move_length = 5.0;
            }
            else if(move_length < 1.0){
                move_length = 1.0;
            }
            ret_ref_record_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir*-1;
            this->ManualMove(phy_axis, dir*-1,
                             this->m_p_axis_config[phy_axis].ret_ref_speed, move_length);

            m_n_ret_ref_step[phy_axis] = 1;
        }
        else{
            m_n_ret_ref_step[phy_axis] = 2;
        }

        break;
    case 1:{//等待回退到位，并再次检查原点信号
        printf("return ref step 1\n");
        chn = this->GetAxisChannel(phy_axis, chn_axis);
        bool flag = CheckAxisRefBaseSignal(phy_axis, dir);

        if(chn != CHANNEL_ENGINE_INDEX){
            if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - ret_ref_record_pos[phy_axis]) <= 0.010){
                if(true == flag){
                    m_n_ret_ref_step[phy_axis] = 0;
                    break;
                }
                else{
                    m_n_ret_ref_step[phy_axis] = 2;  //跳转下一步
                }
            }

        }else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC轴到位
            if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
                if(true == flag){
                    m_n_ret_ref_step[phy_axis] = 0;
                    break;
                }else{
                    m_n_ret_ref_step[phy_axis] = 2;  //跳转下一步
                }
            }
        }
    }
        break;
    case 2:  //以回参考点速度向回参考点方向运动
        //		printf("return ref step 2\n");
        this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed, 900.0);
        m_n_ret_ref_step[phy_axis] = 3;  //跳转下一步
        printf("return ref, goto step 3\n");
        break;
    case 3:  //触发粗基准信号，停止
        //		printf("return ref step 3\n");
        if(this->CheckAxisRefBaseSignal(phy_axis, dir)){
            this->ManualMoveStop(phy_axis);
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //记录起始时间
            m_n_ret_ref_step[phy_axis] = 4;  //跳转下一步
            printf("return ref, goto step 4\n");
        }
        else{
            this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed,3.0);
        }
        break;
    case 4:{//等待停止到位
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 300000){ //延时300ms
            m_n_ret_ref_step[phy_axis] = 5;  //跳转下一步
            printf("return ref, goto step 5\n");
        }
    }
        break;
    case 5:  //有基准， 低速回退至粗基准信号消失,
        printf("return ref step 5\n");
        this->ManualMove(phy_axis, dir_opt, 100, this->m_p_axis_config[phy_axis].move_pr*2); //有基准， 低速回退至粗基准信号消失, 速度：100mm/min
        m_n_ret_ref_step[phy_axis] = 6;  //跳转下一步
        printf("return ref, goto step 6\n");

        break;
    case 6:  //有基准，等待粗基准信号消失
        //	printf("return ref step 6\n");
        if(!this->CheckAxisRefBaseSignal(phy_axis, dir)){
            this->ManualMoveStop(phy_axis);

            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //记录起始时间，延时300ms

            m_n_ret_ref_step[phy_axis] = 7;  //跳转下一步
        }
        break;
    case 7:{ //等待停止到位
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时500ms
            //				m_n_get_cur_encoder_count = 0;  //复位获取当前编码器次数
            m_n_ret_ref_step[phy_axis] = 8;  //跳转下一步
            printf("return ref, goto step 8\n");
        }
    }
        break;
    case 8:{ //  发送开始捕获精基准信号的命令
        printf("return ref step 8: send cmd to mi, set ref point\n");

        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.axis_index = phy_axis+1;
        cmd.data.cmd = CMD_MI_ACTIVE_Z_CAPT;
        this->m_p_mi_comm->WriteCmd(cmd);

        m_n_ret_ref_step[phy_axis] = 9;  //跳转下一步
        printf("return ref, goto step 9\n");

    }
        break;

    case 9:{ //  缓慢移动，并检测精基准
        printf("return ref step 9: move to index pos\n");

        double move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;
        ret_ref_record_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir_opt;
        this->ManualMove(phy_axis, dir_opt,
                         this->m_p_axis_config[phy_axis].ret_ref_speed, move_length);

        m_n_ret_ref_step[phy_axis] = 10;  //跳转下一步
        printf("return ref, goto step 10\n");

    }
        break;

    case 10:{
        printf("return ref step 1\n");
        chn = this->GetAxisChannel(phy_axis, chn_axis);
        bool flag = CheckAxisRefBaseSignal(phy_axis, dir);  // 查询是否已经捕获到精基准    ?????????????????????????

        if(chn != CHANNEL_ENGINE_INDEX){
            if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - ret_ref_record_pos[phy_axis]) <= 0.010){
                if(false == flag){
                    m_n_ret_ref_step[phy_axis] = 20;
                    break;
                }
            }
            if(true == flag){
                this->ManualMoveStop(phy_axis);
                gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
                m_n_ret_ref_step[phy_axis] = 11;
            }

        }else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC轴到位
            if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
                if(false == flag){
                    m_n_ret_ref_step[phy_axis] = 20;
                    break;
                }else{
                    this->ManualMoveStop(phy_axis);
                    gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
                    m_n_ret_ref_step[phy_axis] = 11;  //跳转下一步
                }
            }
        }
    }
        break;


    case 11: { //等待设置参考点完成，在Mi指令响应处理函数中处理
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            m_n_ret_ref_step[phy_axis] = 12;  //跳转下一步
            printf("return ref, goto step 12\n");
        }
    }
        break;

    case 12:       // 设置基准
        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.axis_index = phy_axis+1;
        cmd.data.cmd = CMD_MI_SET_REF;
        this->m_p_mi_comm->WriteCmd(cmd);
        gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
        m_n_ret_ref_step[phy_axis] = 13;  //跳转下一步

        break;

    case 13: {
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 100000){ //延时100ms
            m_n_ret_ref_step[phy_axis] = 14;  //跳转下一步
            printf("return ref, goto step 14\n");
        }
    }

        break;
    case 14: //等待设置完成，移动到轴参考点1
#ifdef USES_RET_REF_TO_MACH_ZERO
        dis = 0;   //机械零
#else
        dis = this->m_p_axis_config[phy_axis].axis_home_pos[0];   // 原点坐标
#endif
        this->ManualMove(phy_axis, dis, m_p_axis_config[phy_axis].ret_ref_speed, 0);
        m_n_ret_ref_step[phy_axis] = 15;  //跳转下一步
        printf("return ref, goto step 15\n");
        break;

    case 15:  //等待轴移动到位,完成！
#ifdef USES_RET_REF_TO_MACH_ZERO
        dis = 0;
#else
        dis = m_p_axis_config[phy_axis].axis_home_pos[0];
#endif
        if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis)- dis) <= 0.010){  //到位
            m_n_ret_ref_step[phy_axis] = 16;  //跳转下一步
        }
        break;
    case 16:  //回参考点完成
        printf("axis %u return ref over\n", phy_axis);
        this->SetRetRefFlag(phy_axis, true);
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //置位到参考点标志
        //			printf("return ref over flag : 0x%llx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx\n", this->m_p_pmc_reg->FReg().bits[0].in_ref_point,
        //					m_p_pmc_reg->FReg().all[200], m_p_pmc_reg->FReg().all[201], m_p_pmc_reg->FReg().all[202], m_p_pmc_reg->FReg().all[203],
        //					m_p_pmc_reg->FReg().all[204], m_p_pmc_reg->FReg().all[205], m_p_pmc_reg->FReg().all[206], m_p_pmc_reg->FReg().all[207]);
        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;
        break;

    case 20: //失败处理
        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        m_error_code = ERR_RET_REF_FAILED;
        CreateError(ERR_RET_REF_FAILED, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis);
        break;
    default:
        break;
    }
}


/**
 * @brief 脉冲输出无基准轴回参考点
 */
void ChannelEngine::PulseAxisFindRefNoZeroSignal(uint8_t phy_axis){

    int8_t dir = 0, dir_opt;   //回参考点方向
    //	uint8_t ret_mode = 0;   //回参考点方式
    uint8_t chn = 0, chn_axis = 0;
    double dis = 0;      //移动距离
    static double ret_ref_record_pos[kMaxAxisNum];

    //	ret_mode = m_p_axis_config[phy_axis].ret_ref_mode;
    dir = this->m_p_axis_config[phy_axis].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;  // 回参考点找粗基准方向
    dir_opt = (m_p_axis_config[phy_axis].ret_ref_change_dir==0)?dir*-1:dir;       //回参考点找精基准方向

    switch(this->m_n_ret_ref_step[phy_axis]){
    case 0: {//向精基准开始移动
        printf("return ref step 0, dir = %hhd, dir_opt=%hhd\n", dir, dir_opt);
        this->SetRetRefFlag(phy_axis, false);   //复位回参考点完成标志
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point &= ~(0x01<<phy_axis);

        double move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;
        ret_ref_record_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir_opt;
        this->ManualMove(phy_axis, dir_opt,
                         this->m_p_axis_config[phy_axis].ret_ref_speed, move_length);

        m_n_ret_ref_step[phy_axis] = 2;
    }
        break;
    case 2:{ //  发送开始捕获精基准信号的命令
        printf("return ref step 2: send cmd to mi, set ref point\n");

        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.axis_index = phy_axis+1;
        cmd.data.cmd = CMD_MI_ACTIVE_Z_CAPT;
        this->m_p_mi_comm->WriteCmd(cmd);

        gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //记录起始时间，延时300ms

        m_n_ret_ref_step[phy_axis] = 3;  //跳转下一步
        printf("return ref, goto step 3\n");

    }
        break;

    case 3:{ //  缓慢移动，并检测精基准
        printf("return ref step 3: move to index pos\n");

        double move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;
        ret_ref_record_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir_opt;
        this->ManualMove(phy_axis, dir_opt,
                         this->m_p_axis_config[phy_axis].ret_ref_speed, move_length);

        m_n_ret_ref_step[phy_axis] = 4;  //跳转下一步
        printf("return ref, goto step 4\n");

    }
        break;

    case 4:{
        printf("return ref step 4\n");
        chn = this->GetAxisChannel(phy_axis, chn_axis);
        bool flag = CheckAxisRefBaseSignal(phy_axis, dir);  // 查询是否已经捕获到精基准    ?????????????????????????

        if(chn != CHANNEL_ENGINE_INDEX){
            if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - ret_ref_record_pos[phy_axis]) <= 0.010){
                if(false == flag){
                    m_n_ret_ref_step[phy_axis] = 20;
                    break;
                }
            }
            if(true == flag){
                this->ManualMoveStop(phy_axis);
                gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
                m_n_ret_ref_step[phy_axis] = 5;
            }

        }else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC轴到位
            if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
                if(false == flag){
                    m_n_ret_ref_step[phy_axis] = 20;
                    break;
                }else{
                    this->ManualMoveStop(phy_axis);
                    gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
                    m_n_ret_ref_step[phy_axis] = 5;  //跳转下一步
                }
            }
        }
    }
        break;


    case 5: { //等待设置参考点完成，在Mi指令响应处理函数中处理
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            m_n_ret_ref_step[phy_axis] = 6;  //跳转下一步
            printf("return ref, goto step 6\n");
        }
    }
        break;

    case 6:       // 设置基准
        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.axis_index = phy_axis+1;
        cmd.data.cmd = CMD_MI_SET_REF;
        this->m_p_mi_comm->WriteCmd(cmd);
        gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
        m_n_ret_ref_step[phy_axis] = 7;  //跳转下一步

        break;

    case 7: {
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 100000){ //延时100ms
            m_n_ret_ref_step[phy_axis] = 8;  //跳转下一步
            printf("return ref, goto step 8\n");
        }
    }
        break;

    case 8: //等待设置完成，移动到轴参考点1
#ifdef USES_RET_REF_TO_MACH_ZERO
        dis = 0;    //机械零
#else
        dis = this->m_p_axis_config[phy_axis].axis_home_pos[0];   // 原点坐标
#endif
        this->ManualMove(phy_axis, dis, m_p_axis_config[phy_axis].ret_ref_speed, 0);
        m_n_ret_ref_step[phy_axis] = 9;  //跳转下一步
        printf("return ref, goto step 9\n");
        break;

    case 9:  //等待轴移动到位,完成！
#ifdef USES_RET_REF_TO_MACH_ZERO
        dis = 0;    //机械零
#else
        dis = this->m_p_axis_config[phy_axis].axis_home_pos[0];   // 原点坐标
#endif
        if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis)- dis) <= 0.010){  //到位
            m_n_ret_ref_step[phy_axis] = 10;  //跳转下一步
        }
        break;
    case 11:  //回参考点完成
        printf("axis %u return ref over\n", phy_axis);
        this->SetRetRefFlag(phy_axis, true);
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //置位到参考点标志
        //			printf("return ref over flag : 0x%llx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx\n", this->m_p_pmc_reg->FReg().bits[0].in_ref_point,
        //					m_p_pmc_reg->FReg().all[200], m_p_pmc_reg->FReg().all[201], m_p_pmc_reg->FReg().all[202], m_p_pmc_reg->FReg().all[203],
        //					m_p_pmc_reg->FReg().all[204], m_p_pmc_reg->FReg().all[205], m_p_pmc_reg->FReg().all[206], m_p_pmc_reg->FReg().all[207]);
        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;
        break;

    case 20: //失败处理
        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        m_error_code = ERR_RET_REF_FAILED;
        CreateError(ERR_RET_REF_FAILED, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis);
        break;
    default:
        break;
    }
}


/**
 * @brief  ethcat轴有基准轴回参考点
 */
void ChannelEngine::EcatAxisFindRefWithZeroSignal(uint8_t phy_axis){

    int8_t dir = 0, dir_opt;   //回参考点方向
    //	uint8_t ret_mode = 0;   //回参考点方式
    uint8_t chn = 0, chn_axis = 0;
    double dis = 0;      //移动距离

    dir = this->m_p_axis_config[phy_axis].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;  // 回参考点找粗基准方向
    dir_opt = (m_p_axis_config[phy_axis].ret_ref_change_dir==0)?dir*-1:dir;       //回参考点找精基准方向
    switch(this->m_n_ret_ref_step[phy_axis]){
    case 0://检查原点信号
        std::cout << "step 0, dir: " << (int)dir << " dir_opt: " << dir_opt << std::endl;
        this->SetRetRefFlag(phy_axis, false);   //复位回参考点完成标志
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point &= ~(0x01<<phy_axis);

        if (GetSyncAxisCtrl()->CheckSyncState(phy_axis) == 1)
        {//主动轴相应的从动轴也需要清除回零标志
            ClearSubAxisRefFlag(phy_axis);
        }

        if(this->CheckAxisRefBaseSignal(phy_axis, dir)){  // 已经触发粗基准信号，直接开开始回退
            double move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;
            if(move_length>10.0) {
                move_length = 10.0;
            }
            else if(move_length < 1.0){
                move_length = 1.0;
            }
            m_df_ret_ref_tar_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir*-1;
            this->ManualMove(phy_axis, dir*-1,
                             this->m_p_axis_config[phy_axis].ret_ref_speed, move_length);

            std::cout << "step 0, dir: " << (int)dir << " dir_opt: " << dir_opt << " goto step 1 " << std::endl;
            m_n_ret_ref_step[phy_axis] = 1;
        }
        else{
            std::cout << "step 0, dir: " << (int)dir << " dir_opt: " << dir_opt << " goto step 2 " << std::endl;
            m_n_ret_ref_step[phy_axis] = 2;
        }

        break;
    case 1:{//等待回退到位，并再次检查原点信号
        chn = this->GetAxisChannel(phy_axis, chn_axis);
        bool flag = CheckAxisRefBaseSignal(phy_axis, dir);

        if(chn != CHANNEL_ENGINE_INDEX){
            if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - m_df_ret_ref_tar_pos[phy_axis]) <= 0.010){
                if(true == flag){   //回到step0，继续回退
                    m_n_ret_ref_step[phy_axis] = 0;
                    std::cout << "step 1, goback 0" << std::endl;
                    break;
                }
            }
            if(flag == false){
                m_n_ret_ref_step[phy_axis] = 2;  //跳转下一步
                std::cout << "step 1" << std::endl;
            }

        }else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC轴到位
            if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
                if(true == flag){
                    m_n_ret_ref_step[phy_axis] = 0;
                    break;
                }else{
                    m_n_ret_ref_step[phy_axis] = 2;  //跳转下一步
                }
            }
        }
    }
        break;
    case 2:  //以回参考点速度向回参考点方向运动
        this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed, 90000.0);
        m_n_ret_ref_step[phy_axis] = 3;
        std::cout << "step 2, ret_ref_speed: " << this->m_p_axis_config[phy_axis].ret_ref_speed << "dir: " << (int)dir << std::endl;
        break;
    case 3:  //触发粗基准信号，停止
        if(this->CheckAxisRefBaseSignal(phy_axis, dir)){
            this->ManualMoveStop(phy_axis);
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            m_n_ret_ref_step[phy_axis] = 4;
            this->SendMonitorData(false, false);
            std::cout << "step 3, find signal pos: " << this->GetPhyAxisMachPosIntp(phy_axis) << std::endl;
        }
        else{
            //	this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed,1.0);
        }
        break;
    case 4:{//等待停止到位
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 1000000){ //延时1000ms
            m_n_ret_ref_step[phy_axis] = 5;
            this->SendMonitorData(false, false);
            std::cout << "step 4, wait for 1000ms, axisPos: " << this->GetPhyAxisMachPosIntp(phy_axis) << std::endl;
        }
    }
        break;
    case 5:{  //有基准， 低速回退至粗基准信号消失,
        double move_length = this->m_p_axis_config[phy_axis].move_pr*2;
        if(move_length>10.0) {
            move_length = 10.0;
        }else if(move_length < 1.0){
            move_length = 1.0;
        }
        double val = 60;
        if (this->m_p_axis_config[phy_axis].ret_ref_speed_second > 0)
            val = this->m_p_axis_config[phy_axis].ret_ref_speed_second;
        m_df_ret_ref_tar_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir_opt;
        this->ManualMove(phy_axis, dir_opt, val, move_length); //有基准， 低速回退至粗基准信号消失//60
        m_n_ret_ref_step[phy_axis] = 6;  //跳转下一步
        std::cout << "step 5," << " speed: " << val << std::endl;

        break;
    }
    case 6:  //有基准，等待粗基准信号消失
        if(!this->CheckAxisRefBaseSignal(phy_axis, dir)){
            this->ManualMoveStop(phy_axis);

            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //记录起始时间，延时600ms

            m_n_ret_ref_step[phy_axis] = 7;  //跳转下一步
            std::cout << "step 6, goto 7" << std::endl;
        }else if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - m_df_ret_ref_tar_pos[phy_axis]) <= 0.010){  //接近目标位置，粗基准信号还在，则继续回退
            m_n_ret_ref_step[phy_axis] = 5;  //跳转第5步
            std::cout << "step 6, goback 5" << std::endl;
        }
        break;
    case 7:{ //等待停止到位
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 600000){ //延时600ms
            //				m_n_get_cur_encoder_count = 0;  //复位获取当前编码器次数
            if(!this->CheckAxisRefBaseSignal(phy_axis, dir)){
                m_n_ret_ref_step[phy_axis] = 8;  //跳转下一步
                std::cout << "step 7, wait for 600" << std::endl;
            }else{      //如果又发现粗基准有信号，则继续回退
                m_n_ret_ref_step[phy_axis] = 5;  //回跳至第5步
                std::cout << "step 7, goback 5, wait for 600" << std::endl;
            }

        }
    }
        break;
    case 8:{ // 设置原点
        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.axis_index = phy_axis+1;
        cmd.data.cmd = CMD_MI_SET_REF_POINT;

        uint16_t err = m_p_axis_config[phy_axis].ref_mark_err * 1e3;     //单位转换， mm->um
        cmd.data.data[0] = err;    //参考点基准误差
        int64_t pos = m_p_axis_config[phy_axis].axis_home_pos[0]*1e7;   //单位转换,0.1nm
        memcpy(&cmd.data.data[1], &pos, sizeof(int64_t));
        cmd.data.data[5] = this->m_p_axis_config[phy_axis].ref_signal;
        cmd.data.data[6] = this->m_p_axis_config[phy_axis].ret_ref_dir;


        this->m_p_mi_comm->WriteCmd(cmd);
        //			gettimeofday(&this->m_time_ret_ref[phy_axis], nullptr);   //记录起始时间，延时300ms

        m_n_ret_ref_step[phy_axis] = 9;  //跳转下一步
        std::cout << "step 9, CMD_MI_SET_REF_POINT 0x011A " << std::endl;
    }
        break;
    case 9: //等待设置参考点完成，在Mi指令响应处理函数中处理
        break;

    case 10:{
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if (time_elpase >= 200000)
        {
            this->m_p_mi_comm->ReadPhyAxisCurFedBckPos(m_df_phy_axis_pos_feedback, m_df_phy_axis_pos_intp,m_df_phy_axis_speed_feedback,
                                                       m_df_phy_axis_torque_feedback, m_df_spd_angle, m_p_general_config->axis_count);
            if (GetSyncAxisCtrl()->CheckSyncState(phy_axis) == 1)
            {//重新建立主从轴关系
                double machPos = this->GetPhyAxisMachPosFeedback(phy_axis);
                std::cout << "machPos:" << machPos << std::endl;
                SetSubAxisRefPoint(phy_axis, machPos);
            }
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 10\n";
        }
    }
        break;
    case 11:{ //延时完成
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 11, wait for 200ms" << std::endl;
        }
    }
        break;
    case 12: { //运动到偏移坐标
        this->ManualMoveAbs(phy_axis, m_p_axis_config[phy_axis].ret_ref_speed, m_p_axis_config[phy_axis].ref_offset_pos);
        m_n_ret_ref_step[phy_axis]++;
        std::cout << "step 12, move to 0" << std::endl;
    }
        break;
    case 13: { //等待轴运动完成
        double dis = m_p_axis_config[phy_axis].ref_offset_pos;
        if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - dis) <= 0.010){  //到位
            m_n_ret_ref_step[phy_axis]++;  //跳转下一步
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            std::cout << "step 13\n";
        }
    }
        break;
    case 14: { //等待轴停稳
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 1000000){ //延时1000ms
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            //if (m_p_axis_config[phy_axis].ref_offset_pos)
            if(1)
            { //设置零点偏移
                MiCmdFrame mi_cmd;
                memset(&mi_cmd, 0x00, sizeof(mi_cmd));
                mi_cmd.data.cmd = CMD_MI_SET_AXIS_MACH_POS;
                mi_cmd.data.axis_index = phy_axis+1;
                mi_cmd.data.data[0] = 0;
                mi_cmd.data.data[1] = 0;
                this->m_p_mi_comm->WriteCmd(mi_cmd);
                m_n_ret_ref_step[phy_axis]++;
            }
            else
            {
                m_n_ret_ref_step[phy_axis] = 16;
            }

            std::cout << "step 14, wait for 2000ms" << std::endl;
            printf("step 14, return ref[%hhu, %lf, %lf]\n", phy_axis, this->GetPhyAxisMachPosFeedback(phy_axis), 0);
        }
    }
        break;
    case 15: { //设置零点偏移后，重新建立主丛轴关系
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            if (GetSyncAxisCtrl()->CheckSyncState(phy_axis) == 1)
            {
                SetSubAxisRefPoint(phy_axis, 0);
            }
            m_n_ret_ref_step[phy_axis]++;
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            std::cout << "step 15, wait for 200ms" << std::endl;
        }
    }
        break;
    case 16: {
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
//            if(this->m_p_axis_config[phy_axis].feedback_mode == ABSOLUTE_ENCODER ||
//                    this->m_p_axis_config[phy_axis].feedback_mode == LINEAR_ENCODER){  //绝对值需获取机械零点的编码器值并保存，方便断点后恢复坐标
//                MiCmdFrame mi_cmd;
//                memset(&mi_cmd, 0x00, sizeof(mi_cmd));
//                mi_cmd.data.cmd = CMD_MI_GET_ZERO_ENCODER;
//                mi_cmd.data.axis_index = phy_axis+1;

//                this->m_p_mi_comm->WriteCmd(mi_cmd);
//            }
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 16, get encoder" << std::endl;
        }
    }
        break;

    case 17: {
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            this->ManualMoveAbs(phy_axis, m_p_axis_config[phy_axis].ret_ref_speed, m_p_axis_config[phy_axis].axis_home_pos[0]);
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 17, move to " << m_p_axis_config[phy_axis].axis_home_pos[0] << std::endl;
        }
    }
        break;

    case 18: //等待轴移动到位,完成
#ifdef USES_RET_REF_TO_MACH_ZERO
        dis = 0;    //机械零
#else
        dis = this->m_p_axis_config[phy_axis].axis_home_pos[0];   // 原点坐标
#endif
        this->SendMonitorData(false, false);  //再次读取实时位置
        if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis)- dis) <= 0.010){  //到位
                m_n_ret_ref_step[phy_axis]++;  //跳转下一步
                printf("step 18, return ref[%hhu, %lf, %lf]\n", phy_axis, this->GetPhyAxisMachPosFeedback(phy_axis), dis);
        }
        break;
    case 19:
        this->SetRetRefFlag(phy_axis, true);
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //置位到参考点标志

        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        std::cout << "step 19, ref finish" << std::endl;
        break;
    case 20: //失败处理
        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        CreateError(m_error_code, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis);
        break;
    default:
        break;
    }
}

/**
 * @brief ethcat轴有基准回参考点，粗基准与精基准均为IO信号，直线电机
 * @param phy_axis
 */
void ChannelEngine::EcatAxisFindRefWithZeroSignal2(uint8_t phy_axis){
    int8_t dir = 0, dir_opt;   //回参考点方向
    //	uint8_t ret_mode = 0;   //回参考点方式
    uint8_t chn = 0, chn_axis = 0;
    //	double dis = 0;      //移动距离
    //	static double ret_ref_start_pos[kMaxAxisNum];

    //	ret_mode = m_p_axis_config[phy_axis].ret_ref_mode;
    dir = this->m_p_axis_config[phy_axis].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;  // 回参考点找粗基准方向
    dir_opt = (m_p_axis_config[phy_axis].ret_ref_change_dir==0)?dir*-1:dir;       //回参考点找精基准方向

    switch(this->m_n_ret_ref_step[phy_axis]){
    case 0://检查原点信号
        printf("return ref step 0, dir = %hhd, dir_opt=%hhd\n", dir, dir_opt);
        this->SetRetRefFlag(phy_axis, false);   //复位回参考点完成标志
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point &= ~(0x01<<phy_axis);

        if(this->CheckAxisRefBaseSignal(phy_axis, dir)){  // 已经触发粗基准信号，直接开始回退
            double move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;
            if(move_length>10.0){
                move_length = 10.0;
            }else if(move_length < 1.0){
                move_length = 1.0;
            }
            m_df_ret_ref_tar_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir*-1;
            this->ManualMove(phy_axis, dir*-1,
                             this->m_p_axis_config[phy_axis].ret_ref_speed, move_length);

            m_n_ret_ref_step[phy_axis] = 1;
        }
        else{
            m_n_ret_ref_step[phy_axis] = 2;
        }

        break;
    case 1:{//等待回退到位，并再次检查原点信号
        printf("return ref step 1\n");
        chn = this->GetAxisChannel(phy_axis, chn_axis);
        bool flag = CheckAxisRefBaseSignal(phy_axis, dir);

        if(chn != CHANNEL_ENGINE_INDEX){
            if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - m_df_ret_ref_tar_pos[phy_axis]) <= 0.010){
                if(true == flag){
                    m_n_ret_ref_step[phy_axis] = 0;
                    break;
                }
            }
            if(flag == false){
                m_n_ret_ref_step[phy_axis] = 2;  //跳转下一步
            }

        }else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC轴到位
            if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
                if(true == flag){
                    m_n_ret_ref_step[phy_axis] = 0;
                    break;
                }else{
                    m_n_ret_ref_step[phy_axis] = 2;  //跳转下一步
                }
            }
        }
    }
        break;
    case 2:  //以回参考点速度向回参考点方向运动
        //		printf("return ref step 2\n");
        this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed, 90000.0);
        m_n_ret_ref_step[phy_axis] = 3;  //跳转下一步
        printf("return ref, goto step 3\n");
        break;
    case 3:  //触发粗基准信号，停止
        //		printf("return ref step 3\n");
        if(this->CheckAxisRefBaseSignal(phy_axis, dir)){  //触发粗基准
            if(m_p_axis_config[phy_axis].ret_ref_change_dir){  //反向寻找精基准，则先停止到位
                this->ManualMoveStop(phy_axis);
                gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //记录起始时间
                m_n_ret_ref_step[phy_axis] = 4;  //跳转下一步
                printf("return ref, goto step 4\n");
            }else{  //同向减速寻找精基准
                this->ManualMove(phy_axis, dir_opt, this->m_p_axis_config[phy_axis].ret_ref_speed_second,3.0);//20
                m_n_ret_ref_step[phy_axis] = 5;  //跳转下一步
            }
        }
        else{
            //	this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed,3.0);
        }
        break;
    case 4:{//等待停止到位
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 300000){ //延时300ms
            m_n_ret_ref_step[phy_axis] = 5;  //跳转下一步
            printf("return ref, goto step 5\n");
        }
    }
        break;
    case 5:  //低速寻找精基准运动
        printf("return ref step 5\n");
        this->ManualMove(phy_axis, dir_opt, this->m_p_axis_config[phy_axis].ret_ref_speed_second, 3); //低速寻找精基准//20
        m_n_ret_ref_step[phy_axis] = 6;  //跳转下一步
        printf("return ref, goto step 6\n");

        break;
    case 6:  //等待精基准信号
        //	printf("return ref step 6\n");
        if(this->CheckAxisRefSignal(phy_axis)){
            this->ManualMoveStop(phy_axis);
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //记录起始时间，延时300ms
            m_n_ret_ref_step[phy_axis] = 7;  //跳转下一步
        }else{
            this->ManualMove(phy_axis, dir_opt, this->m_p_axis_config[phy_axis].ret_ref_speed_second, 3); //低速寻找精基准//20
        }
        break;
    case 7:{ //等待停止到位
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            //				m_n_get_cur_encoder_count = 0;  //复位获取当前编码器次数
            m_n_ret_ref_step[phy_axis] = 8;  //跳转下一步
            printf("return ref, goto step 8\n");
        }
    }
        break;
    case 8:{ // 设置当前位置为参考点
        printf("return ref step 8: send cmd to mi, set ref point\n");
        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.axis_index = phy_axis+1;
        cmd.data.cmd = CMD_MI_SET_REF_CUR;

        int64_t pos = m_p_axis_config[phy_axis].axis_home_pos[0] * 1e7;   //单位转换，0.1nm
        memcpy(cmd.data.data, &pos, sizeof(int64_t));
        this->m_p_mi_comm->WriteCmd(cmd);
        gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //记录起始时间，延时300ms

        m_n_ret_ref_step[phy_axis] = 9;  //跳转下一步
        printf("return ref, goto step 9\n");

    }
        break;
    case 9: { //等待设置参考点完成，在Mi指令响应处理函数中处理
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            m_n_ret_ref_step[phy_axis] = 10;  //跳转下一步
            printf("return ref, goto step 8\n");
        }
    }
        break;
    case 10:  //回参考点完成
        printf("axis %u return ref over\n", phy_axis);

        this->SetRetRefFlag(phy_axis, true);
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //置位到参考点标志

        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        break;
    case 20: //失败处理
        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        m_error_code = ERR_RET_REF_FAILED;
        CreateError(ERR_RET_REF_FAILED, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis);
        break;
    default:
        break;
    }
}


/**
 * @brief ethcat轴无基准轴回参考点
 */
void ChannelEngine::EcatAxisFindRefNoZeroSignal(uint8_t phy_axis){
    switch(this->m_n_ret_ref_step[phy_axis]){
    case 0: {// 计算原点偏移，设置原点
        std::cout << "ChannelEngine::EcatAxisFindRefNoZeroSignal" << std::endl;

        if (GetSyncAxisCtrl()->CheckSyncState(phy_axis) == 1)
        {//主动轴相应的从动轴也需要清除回零标志
            ClearSubAxisRefFlag(phy_axis);
        }

        MiCmdFrame cmd;
        memset(&cmd, 0x00, sizeof(cmd));
        cmd.data.axis_index = phy_axis+1;
        cmd.data.cmd = CMD_MI_SET_REF_POINT;

        cmd.data.data[0] = 0x01;    //当前精基准
        int64_t pos = m_p_axis_config[phy_axis].axis_home_pos[0]*1e7;   //单位转换,0.1nm
        memcpy(&cmd.data.data[1], &pos, sizeof(int64_t));
        cmd.data.data[5] = this->m_p_axis_config[phy_axis].ref_signal;
        cmd.data.data[6] = this->m_p_axis_config[phy_axis].ret_ref_dir;

        this->m_p_mi_comm->WriteCmd(cmd);


        m_n_ret_ref_step[phy_axis]++;  //跳转下一步
        std::cout << "Step 0, CMD_MI_SET_REF_POINT 0x011A" << std::endl;
    }
        break;
    case 1:  //等待设置参考点完成，在Mi指令响应处理函数中处理
        break;

    case 10:{
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if (time_elpase >= 200000)
        {
            this->m_p_mi_comm->ReadPhyAxisCurFedBckPos(m_df_phy_axis_pos_feedback, m_df_phy_axis_pos_intp,m_df_phy_axis_speed_feedback,
                                                       m_df_phy_axis_torque_feedback, m_df_spd_angle, m_p_general_config->axis_count);
            if (GetSyncAxisCtrl()->CheckSyncState(phy_axis) == 1)
            {//重新建立主从关系
                double machPos = this->GetPhyAxisMachPosFeedback(phy_axis);
                std::cout << "machPos:" << machPos << std::endl;
                SetSubAxisRefPoint(phy_axis, machPos);
            }
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 10\n";
        }
    }
        break;
    case 11:{ //延时完成
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 11, wait for 200ms" << std::endl;
        }
    }
        break;
    case 12:{ //运动到偏移坐标
        //this->ManualMoveAbs(phy_axis, m_p_axis_config[phy_axis].ret_ref_speed, m_p_axis_config[phy_axis].ref_offset_pos);
        this->ManualMoveAbs(phy_axis, m_p_axis_config[phy_axis].ret_ref_speed, 0);
        m_n_ret_ref_step[phy_axis]++;
        std::cout << "step 12, move to " << m_p_axis_config[phy_axis].ref_offset_pos << std::endl;
    }
        break;
    case 13:{ //等待轴运动完成
        double dis = 0;
        if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - dis) <= 0.010){  //到位
            m_n_ret_ref_step[phy_axis]++;  //跳转下一步
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            std::cout << "step 13\n";
        }
    }
        break;
    case 14:{ //等待轴停稳
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 1000000){ //延时1000ms
            printf("step 14, return ref[%hhu, %lf, %lf]\n", phy_axis, this->GetPhyAxisMachPosFeedback(phy_axis), 0.0);

            if (1)
            {
                MiCmdFrame mi_cmd;//设置主轴偏移坐标
                memset(&mi_cmd, 0x00, sizeof(mi_cmd));
                mi_cmd.data.cmd = CMD_MI_SET_AXIS_MACH_POS;
                mi_cmd.data.axis_index = phy_axis+1;
                mi_cmd.data.data[0] = 0;
                mi_cmd.data.data[1] = 0;
                this->m_p_mi_comm->WriteCmd(mi_cmd);
                m_n_ret_ref_step[phy_axis]++;
                gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            }
            else
            {
                m_n_ret_ref_step[phy_axis] = 16;
            }
            std::cout << "step 14, wait for 2000ms, CMD_MI_SET_AXIS_MACH_POS 0x136" << std::endl;
        }
    }
        break;
    case 15:{ //设置从轴偏移坐标
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            if (GetSyncAxisCtrl()->CheckSyncState(phy_axis) == 1)
            {
                SetSubAxisRefPoint(phy_axis, 0);
            }
            gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 15, wait for 200ms" << std::endl;
        }
    }
        break;
    case 16:{ //运动到第一参考点
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
//            if(this->m_p_axis_config[phy_axis].feedback_mode == ABSOLUTE_ENCODER ||
//                this->m_p_axis_config[phy_axis].feedback_mode == LINEAR_ENCODER){  //绝对值需要发送设置参考点指令
//                MiCmdFrame mi_cmd;
//                memset(&mi_cmd, 0x00, sizeof(mi_cmd));
//                mi_cmd.data.cmd = CMD_MI_GET_ZERO_ENCODER;
//                mi_cmd.data.axis_index = phy_axis+1;

//                this->m_p_mi_comm->WriteCmd(mi_cmd);
//                gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
//            }
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 16, get encoder" << std::endl;
        }
    }
        break;

    case 17:{
        struct timeval time_now;
        gettimeofday(&time_now, NULL);
        unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
        if(time_elpase >= 200000){ //延时200ms
            this->ManualMoveAbs(phy_axis, m_p_axis_config[phy_axis].ret_ref_speed, m_p_axis_config[phy_axis].axis_home_pos[0]);
            m_n_ret_ref_step[phy_axis]++;
            std::cout << "step 17, move to " << m_p_axis_config[phy_axis].axis_home_pos[0] << std::endl;
        }
    }
        break;

    case 18:{  //等待轴移动到位,完成！
#ifdef USES_RET_REF_TO_MACH_ZERO
        double dis = 0;    //机械零
#else
        double dis = this->m_p_axis_config[phy_axis].axis_home_pos[0];   // 原点坐标
#endif
        this->SendMonitorData(false, false);  //再次读取实时位置
        if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis)- dis) <= 0.010){  //到位
            m_n_ret_ref_step[phy_axis]++;  //跳转下一步
            printf("step 18, return ref[%hhu, %lf, %lf]\n", phy_axis, this->GetPhyAxisMachPosFeedback(phy_axis), dis);
        }
        break;
    }
    case 19:  //回参考点完成
//        if(this->m_p_axis_config[phy_axis].feedback_mode == ABSOLUTE_ENCODER ||
//                this->m_p_axis_config[phy_axis].feedback_mode == LINEAR_ENCODER){  //绝对值需要发送设置参考点指令
//            MiCmdFrame mi_cmd;
//            memset(&mi_cmd, 0x00, sizeof(mi_cmd));
//            mi_cmd.data.cmd = CMD_MI_GET_ZERO_ENCODER;
//            mi_cmd.data.axis_index = phy_axis+1;

//            this->m_p_mi_comm->WriteCmd(mi_cmd);
//        }
        this->SetRetRefFlag(phy_axis, true);
        this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //置位到参考点标志

        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        std::cout << "step 19, ref finish\n";
        break;
    case 20: //失败处理
        this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
        m_n_ret_ref_step[phy_axis] = 0;

        if(m_n_mask_ret_ref == 0){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
        }
        std::cout << "step 20, ref failed\n";
        m_error_code = ERR_RET_REF_FAILED;
        CreateError(ERR_RET_REF_FAILED, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis);
        break;
    default:
        break;
    }
}


/**
 * @brief 回参考点执行函数
 */
void ChannelEngine::ReturnRefPoint(){

    int count = 0;
    for(uint i = 0; i < this->m_p_general_config->axis_count; i++){
        if((m_n_mask_ret_ref & (0x01<<i)) == 0 /*||
                (m_p_axis_config[i].ret_ref_mode == 0 && m_p_axis_config[i].feedback_mode == INCREMENTAL_ENCODER) 为什么增量式要禁止回参考点*/){  //禁止回参考点
            continue;
        }

        if (GetSyncAxisCtrl()->CheckSyncState(i) == 2)
        {//从动轴不能回零
            CreateError(ERR_RET_SYNC_ERR, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i); 
            continue;
        }
        if(this->m_b_ret_ref_auto){
            if(m_p_axis_config[i].ret_ref_index > m_n_ret_ref_auto_cur)
                continue;
            count++;
        }
        if(this->m_p_axis_config[i].axis_interface == VIRTUAL_AXIS)
        {// 虚拟轴
            this->AxisFindRefNoZeroSignal(i);
        }
        else if(this->m_p_axis_config[i].axis_interface == BUS_AXIS)
        {// 总线轴
            this->SetInRetRefFlag(i, true);

            if(this->m_p_axis_config[i].feedback_mode == NO_ENCODER)
            {   // 步进电机，无反馈
                CreateError(ERR_RET_SYNC_ERR, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
            }else if(this->m_p_axis_config[i].feedback_mode == INCREMENTAL_ENCODER)
            {   //增量式编码器
                if (this->m_p_axis_config[i].ret_ref_mode == 1)
                    this->EcatIncAxisFindRefWithZeroSignal(i);//增量式编码器只支持有挡块回零
                else //不支持无挡块回零
                    CreateError(ERR_RET_NOT_SUPPORT, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
            }
            else
            {  //绝对式编码器
                if (this->m_p_axis_config[i].ret_ref_mode == 1)//绝对式有挡块回零
                {
                    this->EcatAxisFindRefWithZeroSignal(i);
                }
                else
                {
                    if (this->m_p_axis_config[i].absolute_ref_mode == 0) //回零标记点设定方式
                    {
                        this->AxisFindRefNoZeroSignal(i);
                    }
                    else if (this->m_p_axis_config[i].absolute_ref_mode == 1) //无挡块回零方式
                    {
                        this->EcatAxisFindRefNoZeroSignal(i);
                    }
                }
            }
        }else if(this->m_p_axis_config[i].axis_interface == ANALOG_AXIS){   // 非总线轴
            //不支持非总线轴
            CreateError(ERR_RET_SYNC_ERR, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
        }
    }

    if(m_n_mask_ret_ref == 0){
        this->m_b_ret_ref = false;
        this->m_b_ret_ref_auto = false;
        m_n_ret_ref_auto_cur = 0;
    }

    if(this->m_b_ret_ref_auto && count==0){
        if(++m_n_ret_ref_auto_cur > 11){
            this->m_b_ret_ref = false;
            this->m_b_ret_ref_auto = false;
            m_n_ret_ref_auto_cur = 0;
            printf("自动回零异常！\n");
            std::cout << "m_n_mask_ret_ref: " << (int)m_n_mask_ret_ref << std::endl;
        }
    }
}

/**
 * @brief 设置轴回参考点完成标志
 * @param phy_axis : 物理轴号，从0开始
 * @param flag ： true--设置完成标志    false--取消完成标志
 */
void ChannelEngine::SetRetRefFlag(uint8_t phy_axis, bool flag){
    //	printf("SetRetRefFlag: axis=%hhu, flag =%hhu\n", phy_axis, flag);
    uint8_t chn = phy_axis/16;
    uint8_t sec = phy_axis%16;
    uint8_t bit = phy_axis%8;

    if(flag){
        this->m_n_mask_ret_ref_over |= (0x01<<phy_axis);
        this->m_p_pmc_reg->FReg().bits[chn].IRF &= ~(0x01<<sec);

        if(sec < 8)
            m_p_pmc_reg->FReg().bits[chn].ZRF1 |= (0x01<<bit);
        else
            m_p_pmc_reg->FReg().bits[chn].ZRF2 |= (0x01<<bit);

    }else{
        if(m_p_axis_config[phy_axis].axis_interface == VIRTUAL_AXIS || m_p_axis_config[phy_axis].axis_type == AXIS_SPINDLE	//主轴和虚拟轴不用回参考点
            || (m_p_axis_config[phy_axis].feedback_mode == ABSOLUTE_ENCODER && m_p_axis_config[phy_axis].ref_encoder != kAxisRefNoDef))    //绝对值编码器，已设定参考点
        {
            return;   //不用回参考点的轴禁止将回零标志复位
        }
        this->m_n_mask_ret_ref_over &= ~(0x01<<phy_axis);

        if(sec < 8)
            m_p_pmc_reg->FReg().bits[chn].ZRF1 &= ~(0x01<<bit);
        else
            m_p_pmc_reg->FReg().bits[chn].ZRF2 &= ~(0x01<<bit);


    }

    //	if(this->m_p_axis_config[phy_axis].axis_pmc == 0){//非PMC轴    //PMC轴也能加载到通道
    uint8_t chn_axis = 0;
    chn = this->GetAxisChannel(phy_axis, chn_axis);
    if(chn != CHANNEL_ENGINE_INDEX){
        if(this->m_p_channel_control != nullptr)
            this->m_p_channel_control[chn].SetRefPointFlag(chn_axis, flag);
    }
    //	}

    this->SetAxisRetRefFlag();
}

/**
 * @brief 获取物理轴所属通道及通道轴编号
 * @param phy_axis : 物理轴号，从0开始
 * @param chn_axis[out] ：  通道轴号，从0开始
 * @return 返回所属通道号，从0开始,0xFF表示没有所属通道
 */
uint8_t ChannelEngine::GetAxisChannel(uint8_t phy_axis, uint8_t &chn_axis){
    uint8_t chn = this->m_map_phy_axis_chn[phy_axis];

    if(chn != CHANNEL_ENGINE_INDEX){
        for(uint8_t j = 0; j < this->m_p_channel_config[chn].chn_axis_count; j++){
            if(this->m_p_channel_control[chn].GetPhyAxis(j) == phy_axis){
                chn_axis = j;
                break;
            }
        }
    }

    return chn;
}

/**
 * @brief 测试tmp目录是否存在，不存在则创建
 */
void ChannelEngine::CheckTmpDir(){
    DIR *dir = opendir("/cnc/tmp/");   //打开临时目录
    if(dir == nullptr){
        //创建目录
        if(mkdir("/cnc/tmp/", 0755) == -1){//创建目录失败
            g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "创建目录失败！[/cnc/tmp/]");
        }else{
            g_ptr_trace->PrintTrace(TRACE_INFO, HMI_COMMUNICATION, "打开目录[/cnc/tmp/]失败！自动创建改目录！");
        }
    }else{
        closedir(dir);
    }
}

/**
 * @brief 获取注册剩余时间
 * @return 剩余时间
 */
int ChannelEngine::GetRemainDay()
{
    //获取系统时间
    time_t now_time=time(NULL);
    //获取本地时间
    tm*  t_tm = localtime(&now_time);

    int startYear  = t_tm->tm_year + 1900;
    int startMonth = t_tm->tm_mon + 1;
    int startDay   = t_tm->tm_mday;

    int endYear  = this->m_lic_info.dead_line.year;
    int endMonth = this->m_lic_info.dead_line.month;
    int endDay   = this->m_lic_info.dead_line.day;

    std::cout << "startYear: " << startYear << " startMonth: " << startMonth << " startDay: " << startDay << std::endl;
    std::cout << "endYear: "   << endYear   << " endMonth: "   << endMonth   << " endDay:   " << endDay   << std::endl;

    if (startYear > endYear)
    {
        std::cout << "startYear > endYear" << std::endl;
        return 0;
    }

    int h[13] = { 0, 31, 28, 31, 30, 31, 30 , 31, 31, 30, 31, 30, 31 };
    //一年的总天数
    auto yearDay = [&h](int year) -> int{
        if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)
            h[2] = 29;
        else
            h[2] = 28;
        int sum = 0;
            for (int i = 1; i<=12 ; ++i)
                sum += h[i];
            return sum;
    };

    //一年过去的天数
    auto elapseDay = [&h](int year, int month, int day) -> int {
        if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)
            h[2] = 29;
        else
                h[2] = 28;
            int elapseSum = 0;
            for (int i = 1; i < month; ++i)
            {
                elapseSum += h[i];
            }
            if (day > 0)
                day = day - 1;
            elapseSum += day;
            return elapseSum;
    };

    int remainDay = 0;
    if (startYear != endYear)
    {
        while (startYear != endYear)
        {
            remainDay += yearDay(startYear) - elapseDay(startYear, startMonth, startDay);
            startYear++;
            startMonth = 1;
            startDay = 1;
        }
        remainDay += elapseDay(endYear, endMonth, endDay);
    }
    else
    {
        remainDay = elapseDay(endYear, endMonth, endDay) - elapseDay(startYear, startMonth, startDay);
    }
    std::cout << "get remainDay: " << remainDay << std::endl;
    if (remainDay < 0)
        remainDay = 0;
    return remainDay;
}

/**
 * @brief 激活PMC轴
 * @param  phy_axis 物理轴号,从0开始
 * @return
 */
bool ChannelEngine::SetPmcActive(uint64_t phy_axis)
{
    if (phy_axis >= max_axis_cnt) //最多支持64个轴
        return false;
    m_pmc_axis_active_mask |= (0x01 << phy_axis);
    return true;
}

/**
 * @brief 取消激活PMC轴
 * @param  phy_axis 物理轴号,从0开始
 * @return
 */
bool ChannelEngine::RstPmcActive(uint64_t phy_axis)
{
    if (phy_axis >= max_axis_cnt) //最多支持64个轴
        return false;
    m_pmc_axis_active_mask &= ~(0x01 << phy_axis);
    return true;
}

/**
 * @brief 获取PMC轴激活状态
 * @param  phy_axis 物理轴号,从0开始
 * @return
 */
bool ChannelEngine::GetPmcActive(uint64_t phy_axis)
{
    if (phy_axis >= max_axis_cnt) //最多支持64个轴
        return false;
    if ( m_pmc_axis_active_mask & (0x01 << phy_axis) )
        return true;
    else
        return false;
}


/**
 * @brief 输出调试数据
 */
void ChannelEngine::PrintDebugInfo(){
    printf("CHN ENGINE DEBUG INFO: module_ready_mask=0x%hhu\n", g_sys_state.module_ready_mask);

    printf("MC CMD FIFO INFO: fifo_count = %u, buffer_count = %u\n", this->m_p_mc_comm->ReadCmdFifoCount(), this->m_p_mc_comm->ReadCmdBufferLen());

    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
        if(true == m_mc_run_on_arm[i])	{
            this->m_p_channel_control[i].PrintDebugInfo1();
        }else{
            this->m_p_channel_control[i].PrintDebugInfo();
        }
    }
}

