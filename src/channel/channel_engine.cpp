/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file ChannelEngine.cpp
 *@author gonghao
 *@date 2020/03/19
 *@brief ���ļ�Ϊͨ���������ʵ��
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


//#include <unistd.h>
//#include <stropts.h>

ChannelEngine* ChannelEngine::m_p_instance = nullptr;  //��ʼ����������ָ��Ϊ��
const map<int, SDLINK_SPEC> ChannelEngine::m_SDLINK_MAP =
{
    //id.SA1, 3.SC1, 4.SD1, 5.SE1
    //������������չ�İ忨����ͼĬ�Ϸ��������ֿռ䣬��Ҫȥ���Ҫʹ������ʱ������SC����
    //id    name    inbytes outBytes withHandWheel
    {1,     {"SA1",  9-3,    4,      true}},
    {3,     {"SC1",  16,     16,     false}},
    {4,     {"SD1",  12-3,   8,      true}},
    {5,     {"SE1",  7-3,    8,      true}}
};

const uint32_t MC_UPDATE_BLOCK_SIZE = 100;		//MC�����ļ�֡��ÿ֡����100�ֽ�����,50��uint16

/**
 * @brief ���캯��
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
	m_df_phy_axis_speed_feedback = nullptr;
//#endif

	m_p_pmc_reg = nullptr;

	memset(this->m_device_sn, 0x00, SN_COUNT+1);
	strcpy(this->m_device_sn, "LIT0123456789");   //Ĭ�����к�"LIT0123456789"
	this->m_lic_info.InitData();     //��ʼ������


	m_thread_update = 0;

	m_error_code = ERR_NONE;
	m_n_cur_channle_index = 0;
	m_n_cur_chn_group_index = 0;

	m_b_recv_mi_heartbeat = false;
	m_b_init_mi_over = false;
	m_b_emergency = false;
	m_b_power_off = false;
	m_b_reset_rst_signal = false;

#ifdef USES_EMERGENCY_DEC_STOP
	this->m_b_delay_servo_off = false;
#endif

	this->m_n_cur_pmc_axis = 0xFF;   //Ĭ��û�е�ǰ��

	this->m_hard_limit_negative = 0;
	this->m_hard_limit_postive = 0;

	this->m_n_pmc_axis_count = 0;
	m_n_run_axis_mask = 0x00;  //��ǰ���е����mask
	m_n_runover_axis_mask = 0x00;   //��ǰ������ɵ����mask

	m_mask_import_param = 0;    //��ʼ�����������־

//	m_n_mc_auto_buf_max = 200;   //Ĭ��200

	//��ʼ���زο������
	this->m_b_ret_ref = false;
	this->m_b_ret_ref_auto = false;
	m_n_mask_ret_ref_over = 0;
	this->m_n_mask_ret_ref = 0;
	memset(this->m_n_ret_ref_step, 0x00, kMaxAxisNum*sizeof(int));
	
	//��ʼ��mcͨ�����б�־
	m_mc_run_on_arm[0] = false;
	for(uint16_t i=1; i<kMaxChnCount; i++)
	    m_mc_run_on_arm[i] = false;

	m_mc_run_dsp_flag = false;
	m_mc_run_mi_flag = false;
 
	this->CheckTmpDir();

	this->InitBdioDev();
}

/**
 * @brief ��������
 */
ChannelEngine::~ChannelEngine(){
	// TODO Auto-generated destructor stub

	void* thread_result;
	int res = ERR_NONE;

	//�˳����������߳�
	if(this->m_thread_update != 0){
		res = pthread_cancel(m_thread_update);
		if (res != ERR_NONE) {
			printf("Update thread cancel failed\n");
		}

		//	usleep(1000);
		res = pthread_join(m_thread_update, &thread_result);//�ȴ��������߳��˳����
		if (res != ERR_NONE) {
			printf("Update thread join failed\n");
		}
		m_thread_update = 0;
	}

	//���ٷ�ʽ�����
	if(this->m_p_channel_mode_group){
		delete []m_p_channel_mode_group;
		this->m_p_channel_mode_group = nullptr;
	}

	//����ͨ�����ƶ���
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

#ifdef USES_SPEED_TORQUE_CTRL	
	if(m_df_phy_axis_speed_feedback){
		delete []m_df_phy_axis_speed_feedback;
		m_df_phy_axis_speed_feedback = nullptr;
	}
	
	if(m_df_phy_axis_torque_feedback){
		delete []m_df_phy_axis_torque_feedback;
		m_df_phy_axis_torque_feedback = nullptr;
	}
#endif	

	//�ͷ�PMC�Ĵ�������
	if(this->m_p_pmc_reg){
		delete m_p_pmc_reg;
		m_p_pmc_reg = nullptr;
	}
}

/**
 * @brief ��ȡ��ʵ����Ψһ���ʽӿں���������ģʽ
 */
ChannelEngine* ChannelEngine::GetInstance(){
	if(nullptr == m_p_instance){
		m_p_instance = new ChannelEngine();
	}
	return m_p_instance;
}

/**
 * @brief ��ʼ����������ͨ����ӳ��
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

/**
 * @brief ��ʼ��ͬ������ر���
 */
void ChannelEngine::InitSyncAxis(){
	this->m_n_sync_axis_mask = 0;
	this->m_n_sync_over = 0;
	this->m_n_sync_axis_enable_mask = 0;
	m_b_send_sync_cmd = false;
	int64_t flag = 0x01;
	for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
		if(this->m_p_axis_config[i].sync_axis == 1 && this->m_p_axis_config[i].master_axis_no != 0){
			m_n_sync_axis_mask |= (flag<<i);
		}
	}

	this->m_n_sync_axis_enable_mask = this->m_n_sync_axis_mask;  //Ĭ��ͬ���ᶼ��ʹ��

	g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "Init syn axis mask :0x%llx\n", m_n_sync_axis_mask);
}

/**
 * @brief ͬ����ִ��ͬ��
 */
void ChannelEngine::DoSyncAxis(){
	if(this->m_b_send_sync_cmd)
		return;
	int64_t mask = ((m_n_sync_axis_mask & m_n_sync_axis_enable_mask) ^ this->m_n_sync_over);  //�������δͬ�������
	if(mask == 0)
		return;

	if((mask & this->m_n_phy_axis_svo_on) != mask)  //�ȴ����ŷ����
		return;

	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_DO_SYNC_AXIS;

	cmd.data.axis_index = 0xFF;

	memcpy(cmd.data.data, &mask, sizeof(int64_t));

	this->m_p_mi_comm->WriteCmd(cmd);

	this->m_b_send_sync_cmd = true;

	printf("send sync cmd\n");

}

#ifdef USES_PMC_2_0
/**
 * @brief PMC2.0�汾����ȡSD_LINK��վ�豸����
 */
void ChannelEngine::ReadIoDev_pmc2(){
    this->m_list_bdio_dev.Clear();

    std::ifstream ifs;
    ifs.open(PATH_PMC_LDR, std::ios::binary | std::ios::in);

    if (ifs.is_open())
    {
        // #SDLINKIO@# ����ͼ�ļ����ҵ�SD-LINK����
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
            auto ReadSequenceValue = [&]()->int {// ����ͼ�ж�ȡ����
                int val = 0;
                ifs.read(reinterpret_cast<char *>(&val), sizeof(val));
                val = BigLittleSwitch32(val);
                return val;
            };
            HandWheelMapInfoVec infoVec;
            bool findHandWheel = false; // ���ֹ��ܣ�Ĭ��ֻ���ò��ҵ��ĵ�һ������ֹ��ܵ���չ�忨

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
                    // ��ͼĬ�Ϸ��������ֿռ䣬�����ֵ���չ�忨��Ҫȥ�����ֿռ䣬����Ҫʹ������ʱ������SC����
                    if (devInfo.in_bytes && sdlink_spec.withHandWheel)
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

                    // ��ż��,��ű�������
                    if (devInfo.group_index != m_list_bdio_dev.GetLength() + 1)
                    {
                        m_list_bdio_dev.Clear();
                        std::cout << static_cast<int>(devInfo.group_index) << " not sequence " << std::endl;
                        CreateError(ERR_PMC_SDLINK_CONFIG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON);
                        break;
                    }
                }

                m_list_bdio_dev.Append(devInfo);
                if (!findHandWheel && SelectHandleWheel(devInfo.group_index, 1))
                {// Ĭ��ʹ�õ�һ���ҵ�������
                    findHandWheel = true;
                    devInfo.handwheel_map = 1;
                }
                HandWheelMapInfo info(devInfo.group_index, 0, devInfo.handwheel_map, devInfo.info_name);
                infoVec.push_back(info);
            }

            HandWheelMapInfoVec configInfo = g_ptr_parm_manager->GetHandWheelVec();
            if (configInfo == infoVec)
            {// ��ͼ��չ�忨������Ϣ����忨������δ�����仯��ʹ���û����ò���
                for (auto itr = configInfo.begin(); itr != configInfo.end(); ++itr)
                {
                    SelectHandleWheel(itr->devNum, itr->channelMap);
                }
            }
            else
            {// ��ͼ��չ�忨������Ϣ����忨�����������仯��ʹ��Ĭ�������������������ļ�
                g_ptr_parm_manager->SyncHandWheelInfo(infoVec);
            }
        }

        ListNode<BdioDevInfo> *node = this->m_list_bdio_dev.HeadNode();
        while(node != nullptr)
        {
            if (!CheckIoDev_pmc2(node->data))
            {
                m_list_bdio_dev.Clear();
                CreateError(ERR_PMC_SDLINK_CONFIG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON);
                break;
            }
            //test
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
        printf("��ʼ��SD-LINK�豸ʧ�ܣ��޷�����ͼ�ļ�\n");
    }
    return;

}

/**
 * @brief PMC2.0�汾�����SD_LINK�����Ƿ����
 * @return true--�Ϸ�     false--�Ƿ�
 */
bool ChannelEngine::CheckIoDev_pmc2(const BdioDevInfo &info)
{
    // �豸���ͼ��
    if (m_SDLINK_MAP.find(info.device_type) == m_SDLINK_MAP.end())
    {
       std::cout << info.group_index << " config error, device_type: " << static_cast<int>(info.device_type) << std::endl;
       return false;
    }

    SDLINK_SPEC sdLink_spec = m_SDLINK_MAP.at(info.device_type);
    // �������
    int standard_InBytes = sdLink_spec.inBytes;
    if (info.handwheel_map != 0)
    {
        if (sdLink_spec.withHandWheel)
        {
            standard_InBytes += HANDWHEEL_BYTES;
        }
        else
        {   // ��֧�����ֵİ忨
            std::cout << info.group_index << " device_type " << static_cast<int>(info.device_type)
                      << " handwheel_map " << static_cast<int>(info.handwheel_map) << std::endl;
            return false;
        }

        if (info.handwheel_map != 1)//����ӳ����ʱֻ֧��ͨ��1
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

    // �������
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
 * @brief PMC2.0�汾����������ͨ��ӳ��
 * @param indexId   : �������ڵ���չ���
 * @param channelId : ͨ���� 1,2,3,4
 * @return true -- �ɹ�   false -- ʧ��
 */
bool ChannelEngine::SelectHandleWheel(int indexId, int channelId)
{
    if (channelId - 1 != 0)//��ʱֻ֧�ֵ�ͨ��
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
            else if (node->data.handwheel_map == channelId) // һ��ͨ����ʱֻ֧��һ������
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
 * @breif PMC1.0�汾����ȡSD_LINK��վ�豸����
 */
void ChannelEngine::ReadIoDev_pmc1(){
	this->m_list_bdio_dev.Clear();

	//����ͼ�ļ�
	int fd = open(PATH_PMC_LDR, O_RDONLY);
	if(fd == -1){
		printf("��ʼ��SD-LINK�豸ʧ�ܣ��޷�����ͼ�ļ�\n");
		return;
	}

//	//���ȶ�ȡ�ļ���С
//	uint64_t file_size = lseek(fd, 0, SEEK_END);
//
//	lseek(fd, 0, SEEK_SET);  //�����ļ�ͷ

	int doc_ver;  //�ĵ��汾
	char uuid[17];  //16�ֽڳ��ȵ�UUID
	int machine_type;   //�豸����
	BdioDevInfo dev_info;    //��վ�豸
	int dev_count = 0;   //bdio�豸����
	int sub_index = 0, dev_type = 0, in_count = 0, out_count = 0;
	int i = 0;
	int tt = 0;

	//��ȡ�ĵ��汾
	int count = read(fd, &doc_ver, sizeof(int));
	if(count != sizeof(int)){
		printf("��ȡ�ĵ��汾ʧ��[%d, %d]\n", count, sizeof(int));
		goto END;
	}
	doc_ver = BigLittleSwitch32(doc_ver);

	printf("read ldr file ver: %d\n", doc_ver);

	//��ȡUUID
	memset(uuid, 0x00, 17);
	count = read(fd, uuid, 16);
	if(count != 16){
		printf("��ȡ��ͼUUIDʧ��[%d, 16]\n", count);
		goto END;
	}

	//��ȡ�豸����
	count = read(fd, &machine_type, sizeof(int));
	if(count != sizeof(int)){
		printf("��ȡ�豸����ʧ��[%d, %d]\n", count, sizeof(int));
		goto END;
	}
	machine_type = BigLittleSwitch32(machine_type);
	printf("machine type = %d\n", machine_type);

	//��ȡBDIO�豸
	count = read(fd, &dev_count, sizeof(int));
	if(count != sizeof(int)){
		printf("��ȡBDIO�豸��ʧ��[%d, %d]\n", count, sizeof(int));
		goto END;
	}
	dev_count = BigLittleSwitch32(dev_count);
	printf("slave count = %d\n", dev_count);

	//��ȡ�����վ����
	for(i = 0; i < dev_count; i++){
		//��վ��
		count = read(fd, &sub_index, sizeof(int));
		if(count != sizeof(int)){
			printf("��ȡBDIO�豸��վ��ʧ��[%d]\n", count);
			goto END;
		}
		sub_index = BigLittleSwitch32(sub_index);

		//��վ����
		count = read(fd, &dev_type, sizeof(int));
		if(count != sizeof(int)){
			printf("��ȡBDIO�豸����ʧ��[%d]\n", count);
			goto END;
		}
		dev_type = BigLittleSwitch32(dev_type);

		//�����ֽ���
		count = read(fd, &in_count, sizeof(int));
		if(count != sizeof(int)){
			printf("��ȡBDIO�豸�����ֽ�ʧ��[%d]\n", count);
			goto END;
		}
		in_count = BigLittleSwitch32(in_count);

		//����ֽ���
		count = read(fd, &out_count, sizeof(int));
		if(count != sizeof(int)){
			printf("��ȡBDIO�豸����ֽ�ʧ��[%d]\n", count);
			goto END;
		}
		out_count = BigLittleSwitch32(out_count);

		//���������ַ�������ַ
		if(-1 == lseek(fd, 2*sizeof(int), SEEK_CUR)){
			printf("�������������ַʧ��\n");
			goto END;
		}

		//�����豸����
		count = read(fd, &tt, sizeof(int));
		if(count != sizeof(int)){
			printf("��ȡ�豸���Ƴ���ʧ��[%d]\n", count);
			goto END;
		}
		tt = BigLittleSwitch32(tt);
		if(-1 == lseek(fd, tt, SEEK_CUR)){
			printf("�����豸����ʧ��\n");
			goto END;
		}

		//������ַ�ַ���
		count = read(fd, &tt, sizeof(int));
		if(count != sizeof(int)){
			printf("��ȡ��ַ�ַ�������ʧ��[%d]\n", count);
			goto END;
		}
		tt = BigLittleSwitch32(tt);
		if(-1 == lseek(fd, tt, SEEK_CUR)){
			printf("������ַ�ַ���ʧ��\n");
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
	close(fd);   //�ر��ļ�
}
#endif

/**
 * @brief ��ʼ��SD-LINK��վ�豸����
 */
void ChannelEngine::InitBdioDev(){

#ifdef USES_PMC_2_0
	this->ReadIoDev_pmc2();
#else
	ReadIoDev_pmc1();
#endif

}

/**
 * @brief ��������ŷ��ź�
 */
void ChannelEngine::CheckAxisSrvOn(){
	for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].CheckAxisSrvOn(m_n_phy_axis_svo_on);
	}
}

/**
 * @brief ��MI���͸���زο����־
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
 * @brief ��MI����������ķ���
 */
void ChannelEngine::SendMiPhyAxisEncoder(){
	FILE *fp = fopen(PATH_PHY_AXIS_ENCODER, "rb");//���ļ�

	if(fp == nullptr){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�������ᵱǰ��������ʧ�ܣ�");
		return;//�ļ���ʧ��
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
 * @brief ����MIģ�鹤��ģʽ
 * @param value : ����ģʽ��ʾ�� 0x00--�Զ�ģʽ    0x10--��������ģʽ     0x20--�ֶ�ģʽ
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
		cmd.data.reserved = this->m_p_channel_mode_group[this->m_n_cur_chn_group_index].GetChannel(i);  //ͨ���ţ���0��ʼ

		if(cmd.data.reserved == 0xFF)
			continue;

		this->m_p_mi_comm->WriteCmd(cmd);
	}

}

/**
 * @brief ����MIģ�����ָ���ģʽ
 * @param flag : true -- �����ָ���  false--�ر����ָ���
 * @param chn : ͨ���ţ���0��ʼ
 */
void ChannelEngine::SetMiHandwheelTrace(bool flag, uint8_t chn){
	g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "SetMiHandwheelTrace: %hhu�� curgroup=%hhu\n", flag, m_n_cur_chn_group_index);
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_SET_HANDWHEEL_TRACE;

	cmd.data.data[0] = flag?0x10:0x00;
	cmd.data.data[1] = this->m_p_general_config->hw_rev_trace;   //���ַ�������ʹ��
						 
	cmd.data.axis_index = NO_AXIS;
	cmd.data.reserved = chn;   //ͨ������0��ʼ

							   
	this->m_p_mi_comm->WriteCmd(cmd);

//	uint8_t chn_count = this->m_p_channel_mode_group[this->m_n_cur_chn_group_index].GetChannelCount();
//	for(uint i = 0; i < chn_count; i++){
//		cmd.data.axis_index = NO_AXIS;
//		cmd.data.reserved = this->m_p_channel_mode_group[this->m_n_cur_chn_group_index].GetChannel(i);    //ͨ������0��ʼ
//
//		if(cmd.data.reserved == 0xFF)
//			continue;
//
//		this->m_p_mi_comm->WriteCmd(cmd);
////		printf("SetMiHandwheelTrace 2 : %hhu\n", cmd.data.reserved);
//	}

}

/**
 * @brief ����MI��ǰͨ����
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
 * @brief ��ʼ�������ⲿ�ӿ��Լ�����ͨ�����ƶ���
 * @param hmi_comm ����HMI��ͨѶ�ӿ���ָ��
 * @param parm ���������ʽӿ�
 */
void ChannelEngine::Initialize(HMICommunication *hmi_comm, MICommunication *mi_comm,
		MCCommunication *mc_comm, ParmManager *parm){
	printf("start to initialize channel engine\n");
	if(hmi_comm == nullptr ||
			mi_comm == nullptr ||
			mc_comm == nullptr ||
			parm == nullptr){
		m_error_code = ERR_SC_INIT;  //��ʼ��ʧ�ܣ��澯
		return;
	}

	this->m_n_update_state = MODULE_UPDATE_NONE;  //Ĭ�Ϸ�����״̬

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

	this->m_n_pmc_axis_count = parm->GetPmcAxisCount();

	//����PMC�Ĵ��������
	this->m_p_pmc_reg = new PmcRegister();
	if(m_p_pmc_reg == nullptr){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ͨ�����洴��PMC�Ĵ�������ʧ��!");
		m_error_code = ERR_MEMORY_NEW;  //��ʼ��ʧ��
		return;
	}
	memset(m_g_reg_last.all, 0x00, sizeof(m_g_reg_last.all));

#ifdef USES_FIVE_AXIS_FUNC
	this->m_p_chn_5axis_config = parm->GetFiveAxisConfig(0);
#endif

	m_n_phy_axis_svo_on = 0;

	this->InitPhyAxisChn();

	this->InitPcAllocList();

	InitSyncAxis();  //��ʼ��ͬ����

	this->m_n_da_prec = pow(2, this->m_p_general_config->da_prec-1);

	this->m_df_phy_axis_pos_feedback = new double[m_p_general_config->axis_count];  //���������ᵱǰ������е����洢��
	memset(m_df_phy_axis_pos_feedback, 0, sizeof(double)*m_p_general_config->axis_count);

	this->m_df_phy_axis_pos_intp = new double[m_p_general_config->axis_count];  //���������ᵱǰ�岹��е����洢��
	memset(m_df_phy_axis_pos_intp, 0, sizeof(double)*m_p_general_config->axis_count);

	this->m_df_phy_axis_pos_intp_after = new double[m_p_general_config->axis_count];  //���������ᵱǰ�岹������Ļ�е����洢��
	memset(m_df_phy_axis_pos_intp_after, 0, sizeof(double)*m_p_general_config->axis_count);

	this->m_df_pmc_axis_remain = new double[m_p_general_config->axis_count];  //����PMC�ᵱǰ���ƶ����洢��
	memset(m_df_pmc_axis_remain, 0, sizeof(double)*m_p_general_config->axis_count);

#ifdef USES_SPEED_TORQUE_CTRL	
	this->m_df_phy_axis_speed_feedback = new double[m_p_general_config->axis_count];  //���������ᵱǰ������е����洢��
	memset(m_df_phy_axis_speed_feedback, 0, sizeof(double)*m_p_general_config->axis_count);
	
	this->m_df_phy_axis_torque_feedback = new double[m_p_general_config->axis_count];  //���������ᵱǰ������е����洢��
	memset(m_df_phy_axis_torque_feedback, 0, sizeof(double)*m_p_general_config->axis_count);
#endif

	//����ͨ������
	this->m_p_channel_control = new ChannelControl[m_p_general_config->chn_count];
	if(m_p_channel_control == nullptr){
		//�ڴ����ʧ�ܣ��澯
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ͨ�����洴��ͨ�����ƶ���ʧ��!");
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
	//��ʼ����PMCͨ�����ƶ���
	for(uint8_t i = 0; i < kMaxPmcAxisCtrlGroup; i++)
		this->m_pmc_axis_ctrl[i].SetGroupIndex(i);
	printf("succeed to init pmc axis ctrl\n");

	//��ʼ����ʽ������
	this->InitChnModeGroup();

	//��ʼ����ǰͨ��Ϊ��һͨ��
	this->SetCurWorkChanl(0);

	//��ʼ���زο����־
	this->m_n_mask_ret_ref_over = 0;
	for(int i = 0; i < this->m_p_general_config->axis_count; i++){
		if(m_p_axis_config[i].axis_interface == VIRTUAL_AXIS || m_p_axis_config[i].axis_type == AXIS_SPINDLE	//����������᲻�ûزο���
			|| (m_p_axis_config[i].feedback_mode == NO_ENCODER && m_p_axis_config[i].ret_ref_mode == 0)    //�޷��������ҽ�ֹ�زο���
			|| (m_p_axis_config[i].feedback_mode != INCREMENTAL_ENCODER && m_p_axis_config[i].feedback_mode != NO_ENCODER && m_p_axis_config[i].ref_encoder != kAxisRefNoDef)    //����ֵ�����������趨�ο���
			|| (m_p_axis_config[i].feedback_mode == INCREMENTAL_ENCODER && m_p_axis_config[i].ret_ref_mode == 0)){  //��������������ֹ�زο���
//			m_n_mask_ret_ref_over |= (0x01<<i);
			this->SetRetRefFlag(i, true);
		}
	}
	printf("Initialize PhyAxis Ref Flag:0x%llx \n", this->m_n_mask_ret_ref_over);

	pthread_attr_t attr;
	struct sched_param param;
	int res = 0;

	//��ʼ����Ȩ���
	if(!ReadDevSn(m_device_sn)){  //��ȡ�豸���к�ʧ��
		printf("Failed to read device sn\n");
	}else{
		printf("device SN: %s\n",m_device_sn);
	}
#ifdef USES_LICENSE_FUNC
	m_ln_local_time = ReadLocalTime(m_device_sn);//��ȡ���ؼ�ʱ�ļ�
	if(m_ln_local_time < 0){//��ȡ���ؼ�ʱ�ļ��쳣
		if(m_ln_local_time == -1){
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "���ؼ�ʱ�ļ�������!");
		}else if(m_ln_local_time == -2){
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "���ؼ�ʱ�ļ�����!");
		}else if(m_ln_local_time == -3){
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�Ƿ����ؼ�ʱ�ļ�!");
		}
		m_error_code = ERR_SYSTEM_FILE;
		CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
	}
	if(!ReadLicense(this->m_device_sn, &this->m_lic_info)){    //��ȡ��Ȩ��Ϣ
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "��ȡ��Ȩ�ļ�ʧ��!");
	}

	if(-4 == CheckLocalTime(&m_lic_info, m_ln_local_time)){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ϵͳʱ���쳣!");
		m_error_code = ERR_SYSTEM_TIME;
		CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
	}

	if(1 == this->CheckLicense(true)){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ϵͳ��Ȩ�Ƿ�!");
	}

	printf("license info: flag = %c, deadline=%04d-%02d-%02d\n", this->m_lic_info.licflag, this->m_lic_info.dead_line.year,
			this->m_lic_info.dead_line.month, this->m_lic_info.dead_line.day);
#endif


	//����״̬ˢ��ˢ���߳�
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 36; //96;
	pthread_attr_setschedparam(&attr, &param);
	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
	if (res) {
		printf("pthread setinheritsched failed\n");
	}

	res = pthread_create(&m_thread_refresh_mi_status, &attr,
			ChannelEngine::RefreshMiStatusThread, this);    //����ͨ��״̬ˢ���߳�
	if (res != 0) {
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ͨ�����洴��MI״̬ˢ���߳�ʧ�ܣ�");
		m_error_code = ERR_SC_INIT;
		return;
	}

	this->InitPoweroffHandler();
	printf("succeed to initialize channel engine\n");

}

/**
 * @brief ��ʼ����ʽ�����ݣ������ڶ�ȡͨ������֮�����
 */
void ChannelEngine::InitChnModeGroup(){
	if(this->m_p_channel_config == nullptr)
		return;

	this->m_p_channel_mode_group = new ChannelModeGroup[this->m_p_general_config->chn_count];    //��ʽ�������������ͨ������

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
 * @brief ��ʼ���ݲ����ݷֲ���
 */
void ChannelEngine::InitPcAllocList(){
	this->m_list_pc_alloc.Clear();

	AxisPcDataAlloc pc_alloc;
	for(int i = 0; i < m_p_general_config->axis_count; i++){
		uint16_t flag = this->m_p_axis_config[i].pc_type; // 0 �����ݲ�  1 ˫���ݲ�
		if(this->m_p_axis_config[i].pc_count == 0)
			continue;
		pc_alloc.axis_index = i;

		if(flag == 1){
			pc_alloc.pc_count = m_p_axis_config[i].pc_count*2;  // ˫���ݲ�
		}else{
			pc_alloc.pc_count = m_p_axis_config[i].pc_count;  // �����ݲ�
		}

		pc_alloc.start_index = m_p_axis_config[i].pc_offset>0?m_p_axis_config[i].pc_offset-1:0;
		pc_alloc.end_index = pc_alloc.start_index+pc_alloc.pc_count-1;


		printf("axis %d -- start_index %d  -- end_index %d\n", i, pc_alloc.start_index, pc_alloc.end_index);

		//����ʼƫ�ƴ�С�����˳�����list
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
 * @brief �ı�ָ��ͨ���������ʽ��
 * @param chn : ͨ���ţ���0��ʼ
 * @param group_old ��ԭ�����ʽ��ţ���0��ʼ
 * @param group_new ���������ʽ��ţ���0��ʼ
 */
void ChannelEngine::ChangeChnGroupIndex(uint8_t chn, uint8_t group_old, uint8_t group_new){
	if(chn >= m_p_general_config->chn_count ||
			group_old >= m_p_general_config->chn_count ||
			group_new >= m_p_general_config->chn_count)
		return;

	if(group_old == group_new)
		return;

	this->m_p_channel_mode_group[group_old].RemoveChannel(chn);    //��ԭ��ʽ�����Ƴ�ͨ��chn
	this->m_p_channel_mode_group[group_new].AddChannel(chn);       //
}

/**
 * @brief ��ȡָ��ͨ����F�Ĵ���ָ��
 * @param chn_index
 * @return
 */
FRegBits *ChannelEngine::GetChnFRegBits(uint8_t chn_index){
	if(chn_index >= kMaxChnCount)
		return nullptr;
	return &m_p_pmc_reg->FReg().bits[chn_index];
}

/**
 * @brief ��ȡָ��ͨ����G�Ĵ���ָ��
 * @param chn_index
 * @return
 */
const GRegBits *ChannelEngine::GetChnGRegBits(uint8_t chn_index){
	if(chn_index >= kMaxChnCount)
		return nullptr;
	return &m_p_pmc_reg->GReg().bits[chn_index];
}


//void ChannelEnginePoweroffHandler(int signo, siginfo_t *info, void *context){
//	//
//
//	printf("########system power off\n");
//}

/**
 * @brief ���д������
 */
void ChannelEngine::DoIdle(){
	uint8_t i = 0;
	this->m_n_idle_count++;
//
//	//����ʱ����PMC�Ĵ�������
//	if(m_n_idle_count%100 == 0){
//		bool idle = true;
//		for(i = 0; i < this->m_p_general_config->chn_count; i++){
//			if(this->m_p_channel_control[i].IsMachinRunning()){
//				idle = false;
//				break;
//			}
//		}
//		if(idle && this->m_p_pmc_reg->IsKeepRegChanged()){//����Ĵ�������
//			this->m_p_pmc_reg->SaveRegData();
//		}
//	}
//
//
	//����ͨ���Ŀ��к���
	for(i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].DoIdle(m_n_idle_count);
	}
}

/**
 * @brief ��ʼ��������źź͵�紦�����
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

	ioctl(fd, SET_NOTIFY_PID, getpid()); //����֪ͨpidΪ�Լ�
	ioctl(fd, SET_NOTIFY_INTERVAL, 0);	//���ü��Ϊ0

	close(fd);

	printf("########succeed to init power off \n");
}

/**
 * @brief ��紦�����
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
 * @brief ���ʱ��������
 */
void ChannelEngine::SaveDataPoweroff(){

	//����PMC�Ĵ�������
	if((this->m_mask_import_param & (0x01<<CONFIG_PMC_REG)) == 0)
		this->m_p_pmc_reg->SaveRegData();

	//�������ʧ�Ժ����
	if((this->m_mask_import_param & (0x01<<CONFIG_MACRO_VAR)) == 0)
		this->SaveKeepMacroVar();


	//�����ͨ����ǰ�豣���״̬
//	g_ptr_parm_manager->SaveParm(CHN_STATE_SCENE);    //ע��ԭ�򣺴�Ϊ��Ƶ��������Ϊ��ͨ��״̬�޸�ʱ��ʱ���棬���ٵ��ʱ���Ӻ�ʱ

	//������ᵱǰλ��
	this->SaveCurPhyAxisEncoder();

	//���浶��������Ϣ
#ifdef USES_WOOD_MACHINE
	this->SaveToolInfo();
#endif

	sync();

	delete g_ptr_trace;
	g_ptr_trace = nullptr;
}

/**
 * @brief ��籣�����ʧ�Ժ����
 */
void ChannelEngine::SaveKeepMacroVar(){

	for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].SaveKeepMacroVar();
	}

}

/**
 * @brief ͬ�������ļ�
 */
void ChannelEngine::SyncKeepVar(){
	//����PMC�Ĵ�������
	this->m_p_pmc_reg->SaveRegData();

	for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].SyncMacroVar();
	}


}

/**
 * @brief ���ò��������־
 * @param param_type : ��������
 */
void ChannelEngine::SetParamImportMask(int param_type){
	if(param_type >= CONFIG_TYPE_COUNT)
		return;

	this->m_mask_import_param |= (0x01<<param_type);
}


/**
 * @brief ��籣�浱ǰ����������ı���������
 */
void ChannelEngine::SaveCurPhyAxisEncoder(){

	FILE *fp = fopen(PATH_PHY_AXIS_ENCODER, "wb");//���ļ�

	if(fp == nullptr){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�������ᵱǰ��������ʧ�ܣ�");
		return;//�ļ���ʧ��
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
 * @brief ��MCģ�鷢����������
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
 * @brief ��MIģ�鷢����������
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
 * @brief ��MC���Ͳ岹���ڲ���
 */
void ChannelEngine::SendIntepolateCycle(){
	McCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(McCmdFrame));
	cmd.data.axis_index = NO_AXIS;
	cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

	cmd.data.cmd = CMD_MC_SET_INIT_PARAM;
//	cmd.data.data[0] = 0x0001;   	//250us����
	cmd.data.data[0] = this->m_p_general_config->bus_cycle; // this->m_p_channel_config[0].intep_cycle;   //�岹����
//	cmd.data.data[1] = 0x0002;		//-Tģʽ


	if(true == this->m_mc_run_dsp_flag)
	    m_p_mc_comm->WriteCmd(cmd);
	
    if(true == this->m_mc_run_mi_flag)
	    m_p_mc_arm_comm->WriteCmd(cmd);
	
	printf("send intep cycle to mc : %hu\n", cmd.data.data[0]);
}

/**
 * @brief ����MC��λָ��
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
	    m_p_mc_arm_comm->WriteCmd(cmd);   // ??????????????????????????????????????????????  ��ѡһ  �Ƿ��һ����
}

/**
 * @brief ����MC����澯ָ��
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
	    m_p_mc_arm_comm->WriteCmd(cmd);  // ??????????????????????????????????????????????  ��ѡһ  �Ƿ��һ����
}

/**
 * @brief ��ʼ��MC�����ݻ�����������Ĭ�Ϸǵ���ģʽ
 */
void ChannelEngine::InitMcDataBuffer(){
	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].InitMcIntpAutoBuf();
		this->m_p_channel_control[i].InitMcIntpMdaBuf();
		this->m_p_channel_control[i].SetMcStepMode(false);
	}
}

/**
 * @brief ��ʼ������MC��������ϵ
 */
void ChannelEngine::InitMcCoord(){
	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].SetMcCoord(true);
	}
}

/**
 * @brief ����MC����ز���
 */
void ChannelEngine::InitMcParam(){
	SendIntepolateCycle();  //���Ͳ岹����

	for(int i = 0; i < this->m_p_general_config->chn_count; i++){ //��ʼ��ͨ������
		this->m_p_channel_control[i].InitMcIntpAutoBuf();
		this->m_p_channel_control[i].InitMcIntpMdaBuf();
		this->m_p_channel_control[i].SetMcStepMode(false);
		this->m_p_channel_control[i].SetMcChnPlanMode();
		this->m_p_channel_control[i].SetMcChnPlanParam();
		this->m_p_channel_control[i].SetMcChnPlanFun();
		this->m_p_channel_control[i].SetMcChnCornerStopParam();
		this->m_p_channel_control[i].SetChnAxisName();

		this->m_p_channel_control[i].SetMcCoord(true);  //��ʼ����������ϵƫ��

		this->m_p_channel_control[i].SetChnAllAxisParam(); //��ʼ�������
#ifdef USES_FIVE_AXIS_FUNC
		this->m_p_channel_control[i].SetMcChnFiveAxisParam();  //��ʼ���������
#endif
#ifdef USES_WOOD_MACHINE
		this->m_p_channel_control[i].SetMcFlipCompParam();  //������ǲ���
		this->m_p_channel_control[i].SetMcDebugParam(0);
		this->m_p_channel_control[i].SetMcDebugParam(1);
		this->m_p_channel_control[i].SetMcDebugParam(2);
		this->m_p_channel_control[i].SetMcDebugParam(3);
		this->m_p_channel_control[i].SetMcDebugParam(4);
#endif
	}

}

/**
 * @brief ��ȡMCģ��İ汾��Ϣ
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
	    m_p_mc_arm_comm->WriteCmd(cmd);  // ??????????????????????????????????????????????  ��ѡһ  �Ƿ��һ����
}

#ifdef USES_TIANJIN_PROJ
/**
 * @brief PMC���е����ϵ����������ϵ��ת��
 * @param pos ����е����
 * @param pmc_axis_index �� PMC�����
 * @return ��������
 */
double ChannelEngine::TransMachToWorkCoord(double &pos, uint8_t pmc_axis_index){
	double work_pos = 0;



	return work_pos;
}

/**
 * @brief PMC�Ṥ������ϵ����е����ϵ��ת��
 * @param pos �� ��������
 * @param pmc_axis_index �� PMC�����
 * @return �� ��е����
 */
double ChannelEngine::TransWorkToMachCoord(double &pos, uint8_t pmc_axis_index){
	double mach_pos = 0;


	return mach_pos;
}
#endif

/**
 * @brief ����PMC��λ�ø�HMI
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
		if(this->m_p_axis_config[i].axis_pmc == 0)
			continue;
		axis.axis_no = i;  //��0��ʼ���
		axis.mach_pos = this->m_df_phy_axis_pos_feedback[i];
		axis.remain_dis = this->m_df_pmc_axis_remain[i];

#ifdef USES_TIANJIN_PROJ
		//��е����ת��Ϊ��������
		axis.work_pos = this->TransMachToWorkCoord(axis.mach_pos, count);
#endif

		memcpy(buffer+4+size*count, &axis, size);
		if(++count >= this->m_n_pmc_axis_count)
			break;
	}


	this->m_p_hmi_comm->SendMonitorData(buffer, buf_len);//send(socket, buffer, buf_len, MSG_NOSIGNAL);
}


/**
 * @brief ���ͼ������
 * @param bAxis : �Ƿ���������ʵʱλ��
 * @param btime : �Ƿ���¼ӹ�ʱ��
 */
void ChannelEngine::SendMonitorData(bool bAxis, bool btime){
	//��MI��ȡ�������λ��
	this->m_p_mi_comm->ReadPhyAxisCurFedBckPos(m_df_phy_axis_pos_feedback, m_df_phy_axis_pos_intp,m_df_phy_axis_speed_feedback,
			m_df_phy_axis_torque_feedback, m_p_general_config->axis_count);

//#ifdef USES_SPEED_TORQUE_CTRL
//	this->m_p_mi_comm->ReadPhyAxisCurFedBckSpeedTorque(m_df_phy_axis_speed_feedback, m_df_phy_axis_torque_feedback, m_p_general_config->axis_count);
//#endif

	if(m_n_pmc_axis_count > 0)
		this->m_p_mi_comm->ReadPmcAxisRemainDis(m_df_pmc_axis_remain, m_n_pmc_axis_count);   // ��ȡ���ƶ���

	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].SendMonitorData(bAxis, btime);
	}

	this->SendPmcAxisToHmi();   //����PMC������
//	printf("send monitor data\n");
}

/**
 * @brief ����MCģ���ָ��ظ�
 * @param rsp
 */
void ChannelEngine::ProcessMcCmdRsp(McCmdFrame &rsp){
//TODO
	switch(rsp.data.cmd){
	case CMD_MC_SHAKEHANDS:
		if(rsp.data.data[0] == 0x1234 && rsp.data.data[1] == 0x5678){
			//���ֳɹ�
			g_sys_state.module_ready_mask |= MC_READY;  //MCģ�����


			this->SetMcAutoBufMax(rsp.data.data[2]);   //����MC�Զ��˶����ݻ����С

//			SendIntepolateCycle();
//
//			//��ʼ���Զ���MDA�����ݻ���
//			InitMcDataBuffer();
//
//			//��ʼ����������ϵ
//			InitMcCoord();
//
//			//��ʼ������λ
//			for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++)
//				this->SetAxisSoftLimit(i, 0);
			InitMcParam();

			this->SendGetMcVersionCmd();//���ͻ�ȡ�汾ָ��
			if(m_p_mc_comm->ReadPlVer()){
				g_sys_state.module_ready_mask |= PL_READY;  //PLģ�����
			}
			if(m_p_mc_comm->ReadSp6Ver()){
				g_sys_state.module_ready_mask |= SPANTAN_READY;  //SP6ģ�����
			}

			if((g_sys_state.module_ready_mask & NC_READY) == NC_READY)
				g_sys_state.system_ready = true;

			g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "@#@#@Get MC shake hand cmd, module_mask=0x%hhu\n", g_sys_state.module_ready_mask);


		}
		else{//����ʧ��
			g_sys_state.module_ready_mask &= (~MC_READY);  //MCģ�����ʧ��
			m_error_code = ERR_MC_INIT;
			CreateError(ERR_MC_INIT, FATAL_LEVEL, CLEAR_BY_RESET_POWER);

		}
		break;
	case CMD_MC_GET_UPDATE_STATE:
//		this->m_n_mc_update_status = rsp.data_crc[1]&0x00FF;	//�ϰ汾����ֵ����axis_index��
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
 * @brief ����MCģ�鷵�صİ汾��Ϣ
 */
void ChannelEngine::ProcessMcVersionCmd(McCmdFrame &cmd){
	//����MCģ�鷵�صİ汾��Ϣ
	uint16_t data1 = cmd.data.data[0], data2 = cmd.data.data[1], data3 = cmd.data.data[2], data4 = cmd.data.data[3];
	uint8_t v_a = (data2>>8)&0xFF;
	uint8_t v_b = (data2)&0xFF;
	uint8_t v_c = (data1>>8)&0xFF;
	uint8_t v_d = (data1)&0xFF;

    sprintf(g_sys_info.sw_version_info.mc, "%c%hhu.%hhu.%hhu.%hu%04hu", v_a==0?'P':'V', v_b, v_c, v_d, data3, data4);
}

/**
 * @brief ����MIģ�鷵�صİ汾��Ϣ
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
 * @brief ����MIģ���ָ��
 * @param cmd
 */
void ChannelEngine::ProcessMiCmd(MiCmdFrame &cmd){
	uint16_t cmd_no = cmd.data.cmd;

//	bool rsp = (cmd_no & 0x8000) == 0 ? false:true;  //�Ƿ���Ӧ��

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
	case CMD_MI_READY:	//MI׼����
		printf("get CMD_MI_READY, module_mask=0x%hhu\n", g_sys_state.module_ready_mask);
		g_sys_state.module_ready_mask |= MI_READY;
		if((g_sys_state.module_ready_mask & NC_READY) == NC_READY){
			g_sys_state.system_ready = true;
			this->ServoOn();
		}

		this->SetWorkMode(this->m_p_channel_control[0].GetChnWorkMode());
		this->SetMiCurChannel();

		break;
	case CMD_MI_ALARM:	//MI�澯
		this->ProcessMiAlarm(cmd);
		break;
	case CMD_MI_BUS_ALARM:		//���߸澯
		ProcessMiBusError(cmd);
		break;
	case CMD_MI_HEARTBEAT:	//MI����
		cmd.data.cmd |= 0x8000;
		this->m_p_mi_comm->WriteCmd(cmd);
		if(!m_b_recv_mi_heartbeat){
			m_b_recv_mi_heartbeat = true;
			this->InitMiParam();
		}
		break;
	case CMD_MI_SET_REF_CUR:	//�����˱�����ֵ
		this->ProcessMiSetRefCurRsp(cmd);
		break;
	case CMD_MI_CLEAR_ROT_AXIS_POS:		//����λ��������Ȧָ��ظ�
		this->ProcessMiClearPosRsp(cmd);
		break;
	case CMD_MI_DO_SYNC_AXIS:		//����ͬ����ͬ�������Ϣ
		this->ProcessMiSyncAxis(cmd);
		break;
	case CMD_MI_PMC_AXIS_RUNOVER:   //PMC�����е�λ
		this->PmcAxisRunOver(cmd);
		break;
	case CMD_MI_REFRESH_AXIS_ZERO:  //ˢ��ָ����Ļ�е��������ֵ
		this->ProcessRefreshAxisZeroEncoder(cmd);
		break;
	case CMD_MI_GET_CUR_ENCODER:   //��ȡ��ǰ������������Ȧ����ֵ�ķ���
		this->ProcessGetCurAxisEncodeRsp(cmd);
		break;
	case CMD_MI_SET_REF_POINT:   //���òο�������ظ�
		this->ProcessSetAxisRefRsp(cmd);
		break;
	case CMD_MI_GET_ZERO_ENCODER:  //��ȡָ�����е����Ӧ�ı�����ֵ
		this->ProcessGetAxisZeroEncoderRsp(cmd);
		break;
	case CMD_MI_ACTIVE_SKIP:    //G31��ת
		this->ProcessSkipCmdRsp(cmd);
		break;
	case CMD_MI_HW_TRACE_STATE_CHANGED:  //���ָ���״̬�л�
		this->ProcessMiHWTraceStateChanged(cmd);
		break;
	case CMD_MI_SET_AXIS_MACH_POS:   //�����ᵱǰ��е����
		this->ProcessSetAxisCurMachPosRsp(cmd);
	case CMD_MI_EN_SYNC_AXIS:    //ʹ��ͬ����
		this->ProcessMiEnSyncAxisRsp(cmd);
		break;
	default:
		printf("Get unsupported mi cmd[%hu]\n", cmd_no);
		break;
	}
}

/**
 * @brief ����MI���صĵ�ǰ��������Ȧ����ֵ���زο���ʱʹ��
 * @param cmd : miָ���
 */
void ChannelEngine::ProcessGetCurAxisEncodeRsp(MiCmdFrame &cmd){
	g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "ChannelEngine::ProcessGetCurAxisEncodeRsp: axis = %hhu\n", cmd.data.axis_index-1);
	uint8_t phy_axis = cmd.data.axis_index-1;
	uint64_t encoder = 0;
	memcpy(&encoder, cmd.data.data, sizeof(encoder));
	uint16_t ref_flag = 0x01;    //��ǰ����׼�ź�λ��
	uint16_t ret_dir = this->m_p_axis_config[phy_axis].ret_ref_dir;

	printf("get cur encoder : %llu\n", encoder);

//	if(encoder < this->m_p_axis_config[phy_axis].motor_count_pr)
//		ref_flag = 0x10;    //ȡ��һ������׼�ź�λ��

//	printf("ref_flag = 0x%hx\n", ref_flag);

//	m_n_get_cur_encoder_count++;
//	if(m_n_get_cur_encoder_count == 1){//��һ�λ�ȡ
//		this->m_n_ret_ref_encoder = encoder;
//
//		m_n_ret_ref_step[phy_axis] = 7;   //�ٻ�ȡһ��
//		return;
//	}else if(m_n_get_cur_encoder_count == 2){ //�ڶ��λ�ȡ�����һ�αȽϣ���ͬ�������һ������ͬ���ȡ������
//		if(encoder != m_n_ret_ref_encoder){
//			printf("###cur encoder not match: 1st=%llu, 2nd=%llu\n", m_n_ret_ref_encoder,
//					encoder);
//			m_n_ret_ref_step[phy_axis] = 7;   //�ٻ�ȡһ��
//			return;
//		}
//	}else if(m_n_get_cur_encoder_count >= 3){
//		printf("get 3rd encoder : %llu\n", encoder);
//	}

	//�������òο�������
	MiCmdFrame send;
	memset(&send, 0x00, sizeof(send));
	send.data.cmd = CMD_MI_SET_REF_POINT;
	send.data.axis_index = phy_axis+1;
	send.data.data[0] = ref_flag;
	int64_t pos = m_p_axis_config[phy_axis].axis_home_pos[0]*1e7;   //��λת��,0.1nm
	memcpy(&send.data.data[1], &pos, sizeof(int64_t));
	send.data.data[5] = this->m_p_axis_config[phy_axis].ref_signal;
	send.data.data[6] = ret_dir;


	this->m_p_mi_comm->WriteCmd(send);

}

/**
 * @brief ����MI���ص�������ο���ظ�ָ��زο���ʱʹ��
 * @param cmd : miָ���
 */
void ChannelEngine::ProcessSetAxisRefRsp(MiCmdFrame &cmd){
	uint8_t axis = cmd.data.axis_index-1;
	printf("ChannelEngine::ProcessSetAxisRefRsp, axis = 0x%hu\n", axis);
	if(cmd.data.data[0] == FAILED){
		printf("axis [%hhu] set ref point failed!\n", cmd.data.axis_index);
		this->m_n_ret_ref_step[axis] = 20;   //��ת��ʧ�ܴ���
	}else{
		if(this->m_p_axis_config[axis].feedback_mode == NO_ENCODER){  //�޷����ᣬֱ�ӽ���
			m_n_ret_ref_step[axis] = 11;   //��ת���ɹ�����
		}else{//�������Ͷ��ƶ�����һ�ο���λ��
			//�����˶�����һ�ο���λ��
		//	int64_t pos = 0;
		//	int8_t dir = DIR_POSITIVE;
		//	memcpy(&pos, &cmd.data.data[1], 8);
	//		this->m_df_phy_axis_pos_feedback[axis] = (double)pos*1e-7;   //��λת����0.1nm --> mm
	//		double dis = this->m_p_axis_config[axis].axis_home_pos[0]-(double)pos*1e-7;   //�ƶ�����
	//		if(dis < 0)
	//			dir = DIR_NEGATIVE;

	//		printf("axis %hhu dir = %hhd, dis = %lf, curpos = %lld\n", axis, dir, dis, pos);

			//����־���׼ƫ��
			if(this->m_p_axis_config[axis].ref_base_diff_check){
				int64_t pos = 0;
				double df_pos = 0;
				memcpy(&pos, &cmd.data.data[1], 8);
				df_pos = pos;
				df_pos *= 1e-7;   //��λת����0.1nm-->mm
				this->m_p_axis_config[axis].ref_base_diff = df_pos;    //�־���׼���
				ParamValue value;
				value.value_double = df_pos;
				g_ptr_parm_manager->UpdateAxisParam(axis, 1312, value);
				this->NotifyHmiAxisRefBaseDiffChanged(axis, df_pos);   //֪ͨHMI������ı�
				printf("axis %hhu ref base diff:%lf, %llu\n", axis, df_pos, pos);
			}



#ifdef USES_RET_REF_TO_MACH_ZERO
			this->ManualMoveAbs(axis, m_p_axis_config[axis].ret_ref_speed, 0);
#else
			this->ManualMoveAbs(axis, m_p_axis_config[axis].ret_ref_speed, m_p_axis_config[axis].axis_home_pos[0]);
#endif
			m_n_ret_ref_step[axis] = 10;   // ��ת��һ��
		}
	}
}

/**
 * @brief ����MI���صĻ�ȡ��е��������ֵָ��زο���ʱʹ��
 * @param cmd : MIָ���
 */
void ChannelEngine::ProcessGetAxisZeroEncoderRsp(MiCmdFrame &cmd){
	uint8_t axis = cmd.data.axis_index-1;
	int64_t encoder_value = 0;
	printf("ChannelEngine::ProcessGetAxisZeroEncoderRsp, axis = 0x%hu\n", axis);
	if(cmd.data.data[0] == FAILED){
		printf("axis [%hhu] get zero encoder failed!\n", cmd.data.axis_index);

	}else{
		//��ȡ������ֵ������
		memcpy(&encoder_value, &cmd.data.data[1], sizeof(int64_t));
		this->m_p_axis_config[axis].ref_encoder = encoder_value;
		g_ptr_parm_manager->UpdateAxisRef(axis, encoder_value); //д���ļ�
		this->NotifyHmiAxisRefChanged(axis);  //֪ͨHMI
	}
}

/**
 * @brief ����MIˢ������������ֵ����
 * @param cmd : MIָ���
 */
void ChannelEngine::ProcessRefreshAxisZeroEncoder(MiCmdFrame &cmd){
	uint8_t axis = cmd.data.axis_index-1;
	int64_t encoder_value = 0;
	printf("ChannelEngine::ProcessRefreshAxisZeroEncoder, axis = 0x%hu\n", axis);

	//��ȡ������ֵ������
	memcpy(&encoder_value, &cmd.data.data[0], sizeof(int64_t));
	this->m_p_axis_config[axis].ref_encoder = encoder_value;
	g_ptr_parm_manager->UpdateAxisRef(axis, encoder_value); //д���ļ�
	this->NotifyHmiAxisRefChanged(axis);
}

/**
 * @brief ����MI���ص���ת������Ӧ
 * @param cmd : MI��Ӧ�����
 */
void ChannelEngine::ProcessSkipCmdRsp(MiCmdFrame &cmd){
	uint8_t chn = cmd.data.data[4]-1;   //ͨ���Ŵ�1��ʼ
	if(chn < this->m_p_general_config->chn_count){
		this->m_p_channel_control[chn].ProcessSkipCmdRsp(cmd);
	}
}

/**
 * @brief ����MIʹ��ͬ����ָ�����Ӧ
 * @param cmd : MI��Ӧ�����
 */
void ChannelEngine::ProcessMiEnSyncAxisRsp(MiCmdFrame &cmd){
	printf("ProcessMiEnSyncAxisRsp, axis=%hhu, enable=%hhu, res=%hhu\n", cmd.data.axis_index, cmd.data.data[1], cmd.data.data[2]);
	if(cmd.data.data[2] == FAILED){  //ʧ��
		if(cmd.data.data[1] == 0){  //���ͬ��
			this->m_error_code = ERR_DIS_SYNC_AXIS;
			CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, cmd.data.data[0], CHANNEL_ENGINE_INDEX, cmd.data.axis_index);
		}else if(cmd.data.data[1] == 1){ //����ͬ��
			this->m_error_code = ERR_EN_SYNC_AXIS;
			CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, cmd.data.data[0], CHANNEL_ENGINE_INDEX, cmd.data.axis_index);
		}
		return;
	}else if(cmd.data.data[2] == SUCCEED){  //�ɹ�
		uint8_t phy_axis = cmd.data.axis_index-1;   //�Ӷ���ţ�MI���ص�ֵ��1��ʼ�������Ҫ��һ

		int64_t mask = 1;
		if(cmd.data.data[1] == 0){  //���ͬ��
			this->m_n_sync_axis_enable_mask &= ~(mask<<phy_axis);
			this->m_n_sync_over &= ~(mask<<phy_axis);

		}else if(cmd.data.data[1] == 1){ //����ͬ��
			this->m_n_sync_axis_enable_mask |= (mask<<phy_axis);
			this->m_n_sync_over |= (mask<<phy_axis);
		}
	}
}

/**
 * @brief ����MI�������ᵱǰλ�û�е�����������Ӧ
 * @param cmd : MI��Ӧ�����
 */
void ChannelEngine::ProcessSetAxisCurMachPosRsp(MiCmdFrame &cmd){
	uint8_t phy_axis = cmd.data.axis_index-1;   //ͨ���Ŵ�1��ʼ
	int64_t encoder_value = 0;
	printf("ChannelEngine::ProcessSetAxisCurMachPosRsp, phy_axis = 0x%hhu, res = %hu\n", phy_axis, cmd.data.data[0]);

	//��ȡ������ֵ������
	memcpy(&encoder_value, &cmd.data.data[1], sizeof(int64_t));
	this->m_p_axis_config[phy_axis].ref_encoder = encoder_value;
	g_ptr_parm_manager->UpdateAxisRef(phy_axis, encoder_value); //д���ļ�
	this->NotifyHmiAxisRefChanged(phy_axis);
}

/**
 * @brief ֪ͨHMI������ο���־���׼λ��ƫ������
 * @param axis �� ������ţ���0��ʼ
 * @param diff �� �־���׼λ��ƫ��
 * @return true--�ɹ�    false--ʧ��
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
	cmd.data[5] = 8;    //ֵ���ͣ�double
	memcpy(&cmd.data[6], &diff, sizeof(double));

	return this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief ֪ͨHMI��ο����Ӧ�ı�����ֵ���
 * @param phy_axis �� ������ţ���0��ʼ
 * @return true--�ɹ�    false--ʧ��
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
	cmd.data[5] = 7;    //ֵ���ͣ�int64
	memcpy(&cmd.data[6], &this->m_p_axis_config[phy_axis].ref_encoder, sizeof(int64_t));

	return this->m_p_hmi_comm->SendCmd(cmd);
}


/**
 * @brief ֪ͨHMI�ݲ����ݸ���
 * @return true--�ɹ�    false--ʧ��
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
 * @brief ��ȨУ��
 * @param force �� ǿ��У��
 * @return 0--�Ϸ���Ȩ   1--�Ƿ���Ȩ
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
		return 1;   //�Ƿ���Ȩ
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
		return 1; //�Ƿ���Ȩ
	}

	if(!force && lpLicInfo->licflag == 'f')
	{
		return 0;
	}

	if(lpLicInfo->dead_line.year == 9999 && lpLicInfo->dead_line.month == 99 && lpLicInfo->dead_line.day == 99){
		lpLicInfo->licflag = 'f';
		return 0;
	}



	time_t now = time(nullptr);  //��ȡ��ǰ����
	struct tm nowday;
	localtime_r(&now, &nowday);
	long curdaycount = (nowday.tm_year+1900) * 365L + nowday.tm_mon * 30 + nowday.tm_mday;
	long deaddaycount = lpLicInfo->dead_line.year * 365L + (lpLicInfo->dead_line.month - 1) * 30
	                    + lpLicInfo->dead_line.day;

	curdaycount = (curdaycount - 1) * 24 + nowday.tm_hour;   //Сʱ��
	deaddaycount = (deaddaycount - 1) * 24 + 12;

	long leave = deaddaycount - curdaycount;

	if(deaddaycount < curdaycount)
	{
		//��ɹ���
		lpLicInfo->licflag = 'o';
		CreateError(ERR_LIC_OVERTIME, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
		return 1;
	}
	else if(leave <= 48)
	{
		//��ɼ�������
		lpLicInfo->licflag = 'm';

		if(leave != lpLicInfo->nWarn){
			lpLicInfo->nInfo = 1;
			lpLicInfo->nWarn = leave;   //ʣ��Сʱ��

			CreateError(ERR_LIC_DUE_SOON, INFO_LEVEL, CLEAR_BY_CLEAR_BUTTON, leave);
		}
	}
	else if((deaddaycount - curdaycount) > 48)
	{
		//�������
		lpLicInfo->licflag = 'p';
		lpLicInfo->nInfo = 0;
		lpLicInfo->nWarn = 0;
	}


	return 0;
}

/**
 * @brief ��HMI������ʾ��Ϣ
 * @param msg_type : ��Ϣ����
 * @param msg_id �� ��ϢID
 * @param msg_param : ��Ϣ����
 * @return true--���ͳɹ�  false--����ʧ��
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
 * @brief ����MI����ָ��
 * @param cmd
 */
void ChannelEngine::ProcessMiShakehand(MiCmdFrame &cmd){
//	bool rsp = (cmd.data.cmd & 0x8000) == 0 ? false:true;  //�Ƿ���Ӧ��

	if(cmd.data.data[0] == 0x9696){
		g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "@#@#Get MI Shakehand cmd, module=0x%hhx\n", g_sys_state.module_ready_mask);
		this->ProcessMiVersionCmd(cmd);   //��ȡMI�汾��
		cmd.data.cmd |= 0x8000;
		if(g_sys_state.module_ready_mask & SC_READY){
			cmd.data.data[0] = 0x8888;
			cmd.data.data[1] = this->m_p_general_config->bus_cycle+1;  //MI�Ĳ�����1��ʼ,1��ʾ125us��2��ʾ250us
			cmd.data.data[2] = (this->m_p_general_config->chn_count << 8) + this->m_p_general_config->axis_count;   //ʵ������������         ʵ��ͨ������
//			cmd.data.data[2] = this->m_p_general_config->axis_count;   //ʵ������������  // ??????????????????????????????????????????????  ��ѡһ  �Ƿ��һ����
			
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
 * @brief ����PMC����ͼ��������
 * @param cmd : ������MI����
 */
void ChannelEngine::ProcessMiPmcLadderReq(MiCmdFrame &cmd){

	uint16_t index = cmd.data.data[0];

	uint32_t res = LoadPmcLadderData(index, cmd.data.data[3]); //��������ͼ����
	memcpy(cmd.data.data, &res, sizeof(uint32_t));
	cmd.data.data[2] = index;

	cmd.data.cmd |= 0x8000;
	this->m_p_mi_comm->WriteCmd(cmd);

}

/**
 * @brief ����MI��ȡ�ŷ������ļ�����
 * @param cmd : ������MI����
 */
void ChannelEngine::ProcessMiGetESBCmd(MiCmdFrame &cmd){
	uint16_t index = cmd.data.data[0];

	uint32_t res = LoadEsbData(index, cmd.data.data[3]); //����ESB�ļ�����
	memcpy(cmd.data.data, &res, sizeof(uint32_t));
	cmd.data.data[2] = index;

	cmd.data.cmd |= 0x8000;
	this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief ����MI�澯��Ϣ
 * @param cmd
 */
void ChannelEngine::ProcessMiAlarm(MiCmdFrame &cmd){
	uint32_t alarm_id = 0;
	uint16_t alarm_level = INFO_LEVEL;
	uint8_t clear_type = CLEAR_BY_MCP_RESET;

	alarm_id = cmd.data.data[1]; //��16λ
	alarm_id = alarm_id<<16;
	alarm_id += cmd.data.data[0];   //��16λ
	alarm_level = cmd.data.data[2];

	if(alarm_level >= WARNING_LEVEL)
		clear_type = CLEAR_BY_CLEAR_BUTTON;

	g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_ENGINE_SC, "ChannelEngine::ProcessMiAlarm, alarm_id=%u, level=%hu", alarm_id, alarm_level);

	CreateError(alarm_id, alarm_level, clear_type, 0, CHANNEL_ENGINE_INDEX, cmd.data.axis_index);
}

/**
 * @brief ����MI���صı�����ֵ
 * @param cmd
 */
void ChannelEngine::ProcessMiSetRefCurRsp(MiCmdFrame &cmd){
	int64_t encoder_value = 0;
	uint8_t axis = cmd.data.axis_index-1;  //MI����Ŵ�1��ʼ��MC�д�0��ʼ

	if(axis >= this->m_p_general_config->axis_count){
		printf("ChannelEngine::ProcessMiSetRefCurRsp(),�Ƿ���ţ�%hhu\n", axis);
		return;
	}

	this->SetRetRefFlag(axis, true);
	this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<axis);   //��λ���ο����־

	memcpy(&encoder_value, cmd.data.data, sizeof(int64_t));
	this->m_p_axis_config[axis].ref_encoder = encoder_value;
	g_ptr_parm_manager->UpdateAxisRef(axis, encoder_value); //д���ļ�

	this->NotifyHmiAxisRefChanged(axis);

	printf("ProcessMiSetRefCurRsp , axis = %hhu\n", axis);

}

/**
 * @brief ����MI���ص�������Ȧλ��ָ��ظ�
 * @param cmd
 */
void ChannelEngine::ProcessMiClearPosRsp(MiCmdFrame &cmd){
	int64_t encoder_value = 0;
	uint8_t axis = cmd.data.axis_index-1;  //MI����Ŵ�1��ʼ��MC�д�0��ʼ

	printf("ProcessMiClearPosRsp , axis = %hhu\n", axis);

	memcpy(&encoder_value, cmd.data.data, sizeof(int64_t));
	this->m_p_axis_config[axis].ref_encoder = encoder_value;
	g_ptr_parm_manager->UpdateAxisRef(axis, encoder_value); //д���ļ�

	this->NotifyHmiAxisRefChanged(axis);

	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].SetClearPosAxisMask(cmd.data.axis_index);
	}
}

/**
 * @brief ����MI���ص�ͬ����ͬ�����
 * @param cmd
 */
void ChannelEngine::ProcessMiSyncAxis(MiCmdFrame &cmd){
	int64_t mask = 0;

	memcpy(&mask, cmd.data.data, 8);

	this->m_n_sync_over = mask;
	if((this->m_n_sync_axis_mask & this->m_n_sync_axis_enable_mask) != mask){
		mask = (this->m_n_sync_axis_mask & this->m_n_sync_axis_enable_mask)^mask;  //Ϊ���ͬ������
		this->m_error_code = ERR_INIT_SYNC_AXIS;
		CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET, mask);
		return;
	}

	g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "Succeed to sync axis: 0x%llx\n", m_n_sync_over);
}

/**
 * @brief ����MI���͵����ָ���״̬�л�ָ��
 * @param cmd : MI������
 */
void ChannelEngine::ProcessMiHWTraceStateChanged(MiCmdFrame &cmd){
	uint8_t chn_idx = cmd.data.reserved;   //ͨ����
	this->m_p_channel_control[chn_idx].ProcessMiHWTraceStateChanged(cmd);
}

/**
 * @brief ����MI���ص����ߴ���
 * @param cmd : MI�����
 */
void ChannelEngine::ProcessMiBusError(MiCmdFrame &cmd){
	uint8_t slave_no = 0;	//�����վ��
	uint8_t err_sub_index = 0;	//������
	uint16_t err_index = 0;	//����
	uint16_t err_code = 0; //������

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
 * @brief ����PMC����ͼ��������
 * @param index : ����ͼ����֡�ţ�һ֡�������128KB
 * @param flag[out] : �Ƿ��к������ݣ� 1--���к���֡    0--û�к���֡
 * @return ���ؼ��ص������ֽ���
 */
int32_t ChannelEngine::LoadPmcLadderData(uint16_t index, uint16_t &flag){
	int32_t res = 0;

	//������ͼ�ļ�
	struct stat statbuf;
	int file_size = 0;
	if(stat(PATH_PMC_DATA, &statbuf) == 0)
		file_size = statbuf.st_size;  //��ȡ�ļ��ܴ�С
	else{
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "��ȡ�ļ�[%s]��Сʧ�ܣ�", PATH_PMC_DATA);
		return res;			//��ȡ�ļ���Сʧ��
	}

	printf("read pmc file size : %d\n", file_size);

	int fp = open(PATH_PMC_DATA, O_RDONLY); //ֻ�����ļ�
	if(fp < 0){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "���ļ�[%s]ʧ�ܣ�", PATH_PMC_DATA);
		return res;//�ļ���ʧ��
	}

	//�����ļ���֡��
	int block_total = file_size/MI_PMC_DATA_SIZE;
	if(file_size%MI_PMC_DATA_SIZE)
		block_total++;

	if(index >= block_total){
		close(fp);
		printf("invalid parameter, index = %d, block_total = %d!\n", index, block_total);
		return res;  	//index���Ϸ�
	}

	uint64_t offset = MI_PMC_DATA_SIZE*index;

	if(-1 == lseek(fp, offset, SEEK_SET)){
		printf("lseek function failed, errno = %d\n", errno);
		close(fp);
		return res;
	}



	//��ȡ����
	int32_t read_size = file_size - offset;
	if(read_size > MI_PMC_DATA_SIZE)
		read_size = MI_PMC_DATA_SIZE;
	char *ptr_data = this->m_p_mi_comm->GetPmcDataAddr();
	res = read(fp, ptr_data, read_size);


	if(res == -1 || res != read_size){  //readʧ��
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
 * @brief ����ESB�ļ����ݣ��ŷ������ļ�
 * @param index : �ļ�˳��ţ���0��ʼ
 * @param flag ���Ƿ��к����ļ���־��0��ʾ�޺����ļ���1��ʾ�к����ļ�
 * @return ���ض�ȡ�ļ��ֽ���
 */
int32_t ChannelEngine::LoadEsbData(uint16_t index, uint16_t &flag){
	g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "Enter ChannelEngine::LoadEsbData(), index=%hu", index);
	int32_t res = 0;

	flag = 0;  //Ĭ�Ϻ������ļ�


	DIR *pdir = nullptr;
	struct dirent *ent = nullptr;
	char file_path[kMaxPathLen] = {0};
	int idx = 0;
	int len = 0;
	char *pt = nullptr;
	bool bfind = false;   //�ҵ���Ӧ�ļ�


	pdir  = opendir(PATH_ESB_FILE);
	if(pdir == nullptr){//·����ʧ��
		//����Ŀ¼
		if(mkdir(PATH_ESB_FILE, 0755) == -1){//����Ŀ¼ʧ��
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "����Ŀ¼ʧ�ܣ�[%s]", PATH_ESB_FILE);
		}else{
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "��Ŀ¼[%s]ʧ�ܣ��Զ�������Ŀ¼��", PATH_ESB_FILE);
		}

		//���ض�ȡʧ��
		return res;
	}

	//����ESB�ļ�Ŀ¼
	while((ent = readdir(pdir)) != nullptr){
		if(ent->d_type &DT_DIR){//Ŀ¼
			continue;
		}else if(ent->d_type & DT_REG){//��ͨ�ļ�
			//У���ļ���׺�ǲ���.esb
			len = strlen(ent->d_name);
			if(len > 4){
				pt = strrchr(ent->d_name, '.');
				if(pt && strcasecmp(pt, "esb")){  //��׺ƥ��
					if(idx >= index){//˳���ƥ��
						//���ɵ�ǰ�����ļ�·��
						if(!bfind){
							bfind = true;
							strcpy(file_path, PATH_ESB_FILE);
							strcat(file_path, ent->d_name);
							printf("open esb file:%s\n", file_path);
						}else{
							flag = 1;   //���������ļ�
							break;
						}

					}
					idx++;
				}
			}
		}
	}

	//���ص�ǰָ��˳��ŵ��ļ�
	struct stat statbuf;
	int file_size = 0;
	if(stat(file_path, &statbuf) == 0)
		file_size = statbuf.st_size;  //��ȡ�ļ��ܴ�С
	else{
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "��ȡ�ļ�[%s]��Сʧ�ܣ�", file_path);
		closedir(pdir);
		return res;			//��ȡ�ļ���Сʧ��
	}

	if(file_size < kEsbDataSize){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ESB�ļ�[%s]��С��ƥ��[size=%d]��", file_path, file_size);
		closedir(pdir);
		return res;   //�ļ���С��ƥ��
	}

	printf("read esb file size : %d\n", file_size);

	int fp = open(file_path, O_RDONLY); //ֻ�����ļ�
	if(fp < 0){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "���ļ�[%s]ʧ�ܣ�", file_path);
		closedir(pdir);
		return res;//�ļ���ʧ��
	}

	//��ȡ����
	int32_t read_size = kEsbDataSize;
	char *ptr_data = this->m_p_mi_comm->GetPmcDataAddr();   //����PMC��ͼ�Ĵ�������
	res = read(fp, ptr_data, read_size);


	if(res == -1 || res != read_size){  //readʧ��
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "��ȡ�ļ�[%s]ʧ�ܣ�ʵ�ʶ�ȡ��С[%d]�ֽ�", file_path, res);
		close(fp);
		closedir(pdir);
		return 0;
	}

	close(fp);
	closedir(pdir);
	g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "Exit ChannelEngine::LoadEsbData(), flag=%hu, bytes=%d", flag, res);
	return res;
}

/**
 * @brief ����HMI��IO����
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
 * @brief ����HMIģ�鷢�͵�����
 * @param cmd ���������HMI�����
 */
void ChannelEngine::ProcessHmiCmd(HMICmdFrame &cmd){

	int i = 0;
	switch(cmd.cmd){
	case CMD_HMI_GET_CHN_STATE:   //HMI��ȡͨ����ǰ״̬
		if(cmd.channel_index >= this->m_p_general_config->chn_count){
			cmd.cmd_extension = FAILED;
			cmd.frame_number |= 0x8000;
			this->m_p_hmi_comm->SendCmd(cmd);
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "����[%d]ͨ���ŷǷ���%d", cmd.cmd, cmd.channel_index);
		}else
			this->m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
		break;
	case CMD_HMI_RESTART:               //�ӹ���λ 11
	case CMD_HMI_SIMULATE:				//���� 12
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
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "����[%d]ͨ���ŷǷ���%d", cmd.cmd, cmd.channel_index);
		}
		else{
			m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
		}
		break;
	case CMD_HMI_SET_NC_FILE:	    //���õ�ǰ�ӹ��ļ� 13
		if(cmd.channel_index == CHANNEL_ENGINE_INDEX){
			for(i = 0; i < m_p_general_config->chn_count; i++){
				m_p_channel_control[i].ProcessHmiCmd(cmd);
			}
		}
		else if(cmd.channel_index >= this->m_p_general_config->chn_count){
			cmd.frame_number |= 0x8000;
			cmd.cmd_extension = FAILED;
			this->m_p_hmi_comm->SendCmd(cmd);
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "����[%d]ͨ���ŷǷ���%d", cmd.cmd, cmd.channel_index);
		}
		else{
			m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
		}
		break;
	case CMD_HMI_FIND_REF_POINT:		//ȷ���ο��� 14
		this->ProcessHmiFindRefCmd(cmd);
		break;
	case CMD_HMI_SET_REF_POINT:			//������ԭ�㣬��Ծ���ֵ������
		if(this->m_n_cur_pmc_axis != 0xFF){
			this->ProcessHmiSetRefCmd(cmd);
		}else{
			this->m_p_channel_control[this->m_n_cur_channle_index].ProcessHmiCmd(cmd);
		}

		break;
	case CMD_SC_MDA_DATA_REQ:			//MDA�������� 115
	case CMD_HMI_GET_MACRO_VAR:			//HMI��SC����������ֵ   30
	case CMD_HMI_SET_MACRO_VAR:        //HMI��SC���ú�����Ĵ�����ֵ   31
	case CMD_HMI_SET_CALIBRATION:      //HMI��SC�����������궨ָ�� 32
	case CMD_HMI_MANUAL_TOOL_MEASURE:     //HMI��SC�����ֶ��Ե�����  0x29
		if(cmd.channel_index < this->m_p_general_config->chn_count)
			m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
		else
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "����[%d]ͨ���ŷǷ���%d", cmd.cmd, cmd.channel_index);
		break;
    case CMD_HMI_CLEAR_WORKPIECE:      //HMI����SC���ӹ���������,��ʱ����(���ְ�ҹ��)
    case CMD_HMI_CLEAR_TOTAL_PIECE:    //�ܹ���������
		if(cmd.channel_index < this->m_p_general_config->chn_count)
			m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
		else if(cmd.channel_index == CHANNEL_ENGINE_INDEX){
			for(int i = 0; i < this->m_p_general_config->chn_count; i++){
				this->m_p_channel_control[i].ProcessHmiCmd(cmd);
			}
		}else
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "����[%d]ͨ���ŷǷ���%d", cmd.cmd, cmd.channel_index);
		break;
	case CMD_HMI_SET_PARA:		//���ò���
		this->ProcessHmiSetParam(cmd);
		break;
	case CMD_HMI_GET_PARA:		//��ȡ����
		this->ProcessHmiGetParam(cmd);
		break;
	case CMD_HMI_UPDATE:		//HMI֪ͨSC������������ļ�����
		this->ProcessHmiUpdateReq(cmd);
		break;
	case CMD_HMI_GET_PMC_REG:			//��ȡPMC�Ĵ���
		this->ProcessHmiGetPmcReg(cmd);
		break;
	case CMD_HMI_SET_PMC_REG:			//����PMC�Ĵ���
		this->ProcessHmiSetPmcReg(cmd);
		break;
	case CMD_HMI_GET_PMC_UUID:
		this->ProcessHmiGetPmcUuid(cmd);	//��ȡpmc��uuid
		break;
	case CMD_HMI_AXIS_MANUAL_MOVE:     //HMIָ�����ƶ�
		this->ProcessHmiAxisMoveCmd(cmd);
		break;
	case CMD_HMI_GET_LIC_INFO:            //HMI��SC������Ȩ��Ϣ   0x27
		this->ProcessHmiGetLicInfoCmd(cmd);
		break;
	case CMD_HMI_SEND_LICENSE:            //HMI��SC������Ȩ��     0x28
		this->ProcessHmiRegLicCmd(cmd);
		break;
	case CMD_HMI_GET_ESB_INFO:            //HMI��SC��ȡESB�ļ�����   0x2A

		break;
	case CMD_HMI_ESB_OPERATE:            //HMI֪ͨSC��ָ��ESB�ļ����в���  0x2B

		break;
	case CMD_HMI_GET_IO_REMAP:			//HMI��SC��ȡIO�ض�������  0x2C
		this->ProcessHmiGetIoRemapInfoCmd(cmd);
		break;
	case CMD_HMI_SET_IO_REMAP:         //HMI��SC����IO�ض�������  0x2D
		this->ProcessHmiSetIoRemapInfoCmd(cmd);
		break;
	case CMD_HMI_SET_PROC_PARAM:      //HMI��SC���ù�����ز���  0x2E
		this->ProcessHmiSetProcParamCmd(cmd);
		break;
	case CMD_HMI_GET_PROC_PARAM:          //HMI��SC��ȡ������ز���  0x2F
		this->ProcessHmiGetProcParamCmd(cmd);
		break;
	case CMD_HMI_SET_PROC_GROUP:          //HMI��SC���õ�ǰ���ղ������  0x30
		this->ProcessHmiSetCurProcIndex(cmd);
		break;
	case CMD_HMI_GET_PROC_GROUP:          //HMI��SC��ȡ��ǰ���ղ������  0x31
		this->ProcessHmiGetCurProcIndex(cmd);
		break;
	case CMD_HMI_SET_CUR_MACH_POS:        //HMI��SC����ָ����Ļ�е����  0x32
		if(cmd.channel_index >= this->m_p_general_config->chn_count){
			cmd.frame_number |= 0x8000;
			cmd.data[9] = FAILED;
			cmd.data_len = 10;
			this->m_p_hmi_comm->SendCmd(cmd);
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "����[%d]ͨ���ŷǷ���%d", cmd.cmd, cmd.channel_index);
		}else
			this->m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
		break;
	case CMD_HMI_CLEAR_MSG:               //HMI֪ͨSC�����Ϣ  0x33
		printf("process CMD_HMI_CLEAR_MSG:%hhu, cmd_ex=%hhu \n", cmd.channel_index, cmd.cmd_extension);
//		if(cmd.channel_index == CHANNEL_ENGINE_INDEX){
//			this->ProcessHmiClearMsgCmd(cmd);
//		}else if(cmd.channel_index < this->m_p_general_config->chn_count){
//			this->m_p_channel_control[cmd.channel_index].ProcessHmiCmd(cmd);
//		}else{
//			cmd.frame_number |= 0x8000;
//			cmd.cmd_extension = FAILED;
//			cmd.data_len = 0;
//			this->m_p_hmi_comm->SendCmd(cmd);
//			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "����[%d]ͨ���ŷǷ���%d", cmd.cmd, cmd.channel_index);
//		}
		break;
	case CMD_HMI_SYNC_AXIS_OPT:           //HMI֪ͨHMI����ͬ����ʹ�ܲ��� 0x34
		this->ProcessHmiEnableSyncAxisCmd(cmd);
		break;
	case CMD_HMI_NOTIFY_GRAPH:            //HMI֪ͨSC����ͼ��ģʽ    0x35
		this->ProcessHmiNotifyGraphCmd(cmd);
		break;
	case CMD_HMI_CHECK_SYNC_EN:           //HMI��SC��ѯͬ����״̬ 0x37
		this->ProcessHmiCheckSyncCmd(cmd);
		break;
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
            m_p_channel_control[cmd.channel_index].ClearMachineTimeTotal();
        else if(cmd.channel_index == CHANNEL_ENGINE_INDEX){
            for(int i = 0; i < this->m_p_general_config->chn_count; i++){
                this->m_p_channel_control[i].ClearMachineTimeTotal();
            }
        }
        break;
    case CMD_HMI_GET_HANDWHEEL_INFO:           // hmi��sc��ѯ����ӳ���ϵ
        ProcessHmiHandWheelCmd(cmd);
        break;
    case CMD_HMI_SET_HANDWHEEL_INFO:           // hmi��sc����������Ϣ
    {
        HandWheelMapInfoVec configInfo = g_ptr_parm_manager->GetHandWheelVec();
        if (cmd.cmd_extension - 1 < configInfo.size() && cmd.channel_index <= 1)//��ʱֻ֧�ֵ�ͨ��
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
            CreateError(ERR_PMC_SDLINK_CONFIG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON);
        }
    }
        break;
	default:
		g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_ENGINE_SC, "��֧�ֵ�HMIָ��[%d]", cmd.cmd);
		break;

	}
}

/**
 * @brief ����HMI��ȡ��Ȩ��Ϣָ��
 * @param cmd : HMIָ���
 */
void ChannelEngine::ProcessHmiGetLicInfoCmd(HMICmdFrame &cmd){
	cmd.frame_number |= 0x8000;  //���ûظ���־
	cmd.cmd_extension = SUCCEED;
	cmd.data_len = 24;

	memcpy(cmd.data, this->m_device_sn, SN_COUNT);  //13�ֽڳ��ȵ��豸SN��
	cmd.data[SN_COUNT] = this->m_lic_info.licflag;    //��Ȩ״̬
	if(this->m_lic_info.licflag == 'f'){
		sprintf(cmd.data+SN_COUNT+1, "%s", "����");    //��Ȩ����ʱ��,10�ֽڳ����ַ�������ʽ�硰2021-12-21��
	}else{
		sprintf(cmd.data+SN_COUNT+1, "%d-%02d-%02d", this->m_lic_info.dead_line.year, this->m_lic_info.dead_line.month,
				this->m_lic_info.dead_line.day);    //��Ȩ����ʱ��,10�ֽڳ����ַ�������ʽ�硰2021-12-21��
	}


	this->m_p_hmi_comm->SendCmd(cmd);

}

/**
 * @brief ����HMIע����Ȩָ��
 * @param cmd : HMIָ���
 */
void ChannelEngine::ProcessHmiRegLicCmd(HMICmdFrame &cmd){
	cmd.frame_number |= 0x8000;  //���ûظ���־

	if(cmd.data_len != LICCODECOUNT){//��Ȩ�볤�ȴ��󣬷���ʧ��
		cmd.cmd_extension = FAILED;
		cmd.data_len = 24;

		memcpy(cmd.data, this->m_device_sn, SN_COUNT);  //13�ֽڳ��ȵ��豸SN��
		cmd.data[SN_COUNT] = this->m_lic_info.licflag;    //��Ȩ״̬
		sprintf(cmd.data+SN_COUNT+1, "%d-%02d-%02d", this->m_lic_info.dead_line.year, this->m_lic_info.dead_line.month,
				this->m_lic_info.dead_line.day);    //��Ȩ����ʱ��,10�ֽڳ����ַ�������ʽ�硰2021-12-21��
		this->m_p_hmi_comm->SendCmd(cmd);
		printf("register failed 1\n");
		return;
	}

	char lic_code[LICCODECOUNT+1];   //��Ȩ��
	memset(lic_code, 0x00, LICCODECOUNT+1);
	memcpy(lic_code, cmd.data, LICCODECOUNT);  //24�ֽڳ��ȵ��豸��Ȩ��
	printf("Get lic code:%s\n", lic_code);

	//ע����Ȩ��
	if(RegisterLicWithCode(this->m_device_sn, lic_code, &this->m_lic_info)){
		cmd.cmd_extension = FAILED;
		cmd.data_len = 24;

		memcpy(cmd.data, m_device_sn, SN_COUNT);  //13�ֽڳ��ȵ��豸SN��
		cmd.data[SN_COUNT] = this->m_lic_info.licflag;    //��Ȩ״̬
		sprintf(cmd.data+SN_COUNT+1, "%d-%02d-%02d", this->m_lic_info.dead_line.year, this->m_lic_info.dead_line.month,
				this->m_lic_info.dead_line.day);    //��Ȩ����ʱ��,10�ֽڳ����ַ�������ʽ�硰2021-12-21��
		this->m_p_hmi_comm->SendCmd(cmd);
		printf("register failed 2\n");
		return;
	}

	this->CheckLicense(true);

	cmd.data_len = 24;

	memcpy(cmd.data, m_device_sn, SN_COUNT);  //13�ֽڳ��ȵ��豸SN��
	cmd.data[SN_COUNT] = this->m_lic_info.licflag;    //��Ȩ״̬
	if(this->m_lic_info.licflag == 'f'){
		sprintf(cmd.data+SN_COUNT+1, "%s", "����");    //��Ȩ����ʱ��,10�ֽڳ����ַ�������ʽ�硰2021-12-21��

	}else{
		sprintf(cmd.data+SN_COUNT+1, "%d-%02d-%02d", this->m_lic_info.dead_line.year, this->m_lic_info.dead_line.month,
				this->m_lic_info.dead_line.day);    //��Ȩ����ʱ��,10�ֽڳ����ַ�������ʽ�硰2021-12-21��
	}

	printf("register lic result: %s\n", cmd.data);

	//���ͻظ�
	cmd.cmd_extension = SUCCEED;
	this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief ����HMI��ȡIO��ӳ����Ϣ����
 * @param cmd : HMIָ���
 */
void ChannelEngine::ProcessHmiGetIoRemapInfoCmd(HMICmdFrame &cmd){
	int count = this->m_p_io_remap->GetLength();

	int count_per = count;   //���η���������
	int i = 0;
	ListNode<IoRemapInfo> *info_node = this->m_p_io_remap->HeadNode();

	cmd.frame_number |= 0x8000;  //���ûظ���־
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
 * @brief ����HMI����IO��ӳ����Ϣ����
 * @param cmd : HMIָ���
 */
void ChannelEngine::ProcessHmiSetIoRemapInfoCmd(HMICmdFrame &cmd){
	cmd.frame_number |= 0x8000;  //���ûظ���־

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

	if(!flag){  //����
		this->m_p_io_remap->Append(info);
	}

	if(!g_ptr_parm_manager->UpdateIoRemapInfo(info)){
		cmd.cmd_extension = 0x01;  //����ʧ��
	}

	this->SendMiIoRemapInfo(info);   //����MI����

	this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief ����HMI���ù�����ز���������
 * @param cmd : HMIָ���
 */
void ChannelEngine::ProcessHmiSetProcParamCmd(HMICmdFrame &cmd){
//	printf("enter ProcessHmiSetProcParamCmd\n");
	cmd.frame_number |= 0x8000;  //���ûظ���־

	uint8_t active_type = ACTIVE_BY_POWEROFF;	//��������
	ProcParamUpdate update;
	update.param_data.param_type = cmd.cmd_extension;  //�������ͣ�2--ͨ������  3--�����
	update.group_index = cmd.data[0];       //���ղ������

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
 * @brief ����HMI��ȡ������ز���������
 * @param cmd : HMIָ���
 */
void ChannelEngine::ProcessHmiGetProcParamCmd(HMICmdFrame &cmd){
//	printf("enter ProcessHmiGetProcParamCmd\n");
	uint8_t index = 0;
	uint8_t axis = 0;
	switch(cmd.cmd_extension){
	case CHN_CONFIG:
		index = cmd.data[0];    //���ղ������
//		printf("get chn proc config :chn = %hhu, proc_idx=%hhu\n", cmd.channel_index, index);
		if(cmd.channel_index >= m_p_general_config->max_chn_count || index >= kMaxProcParamCount)//ͨ����������Χ
			cmd.data_len = 0;
		else{
			cmd.data_len = sizeof(ProcessParamChn);
			memcpy(&cmd.data[1], &this->m_p_chn_proc_param[cmd.channel_index].chn_param[index], cmd.data_len);
			cmd.data_len += 1;  //���ݳ��ȼ�һ���������
		}
		break;
	case AXIS_CONFIG:
		index = cmd.data[0];
		axis = cmd.data[1];
//		printf("get axis proc config: axis =  %hhu, idx = %hhu\n", axis, index);
		if(axis >= m_p_general_config->max_axis_count || index >= kMaxProcParamCount){//��ų���Χ��֧�ֲ����޸ĺ�һ������
			printf("get axis proc config failed, index=%hhu, axis_count:%hhu\n", axis, m_p_general_config->max_axis_count);
			cmd.data_len = 0;
		}
		else{
			cmd.data_len = sizeof(ProcessParamAxis);
			memcpy(cmd.data+2, &this->m_p_axis_proc_param[axis].axis_param[index], cmd.data_len);
			cmd.data_len += 2;   // ���ݳ��ȼ�����ź����
	//		printf("get axis config succeed, data_len:%hu\n", cmd.data_len);
		}

		break;
	default:
		cmd.data_len = 0;
		break;
	}
	cmd.frame_number |= 0x8000;
	this->m_p_hmi_comm->SendCmd(cmd);	//������Ӧ

//	printf("exit ProcessHmiGetProcParamCmd\n");
}

/**
 * @brief ����HMI����ͨ����ǰ������ŵ�����
 * @param cmd : HMIָ���
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

		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "����[%d]ͨ���ŷǷ���%d", cmd.cmd, cmd.channel_index);
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
 * @brief ����HMI��ȡ���ղ�����ŵ�����
 * @param cmd : HMIָ���
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

		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "����[%d]ͨ���ŷǷ���%d", cmd.cmd, cmd.channel_index);
	}
	this->m_p_hmi_comm->SendCmd(cmd);
//	printf("exit ProcessHmiGetCurProcIndex\n");
}

/**
 * @brief ����HMI�����Ϣ����
 * @param cmd : HMIָ���
 */
void ChannelEngine::ProcessHmiClearMsgCmd(HMICmdFrame &cmd){
	cmd.frame_number |= 0x8000;
	cmd.data_len = 0;



	if(cmd.cmd_extension == 0xFF){  //��ո澯����
		g_ptr_alarm_processor->Clear();
		cmd.cmd_extension = SUCCEED;
	}else if(cmd.cmd_extension == 0xEE){  //������漰��ʾ��Ϣ
		g_ptr_alarm_processor->ClearWarning(CHANNEL_ENGINE_INDEX);
		cmd.cmd_extension = SUCCEED;
	}else{
		cmd.cmd_extension = FAILED;
	}

	this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief ����HMIʹ��ͬ��������
 * @param cmd  : HMIָ���
 */
void ChannelEngine::ProcessHmiEnableSyncAxisCmd(HMICmdFrame &cmd){
	cmd.frame_number |= 0x8000;

	MiCmdFrame mi_cmd;
	memset(&mi_cmd, 0x00, sizeof(mi_cmd));
	mi_cmd.data.cmd = CMD_MI_EN_SYNC_AXIS;

//
	if(cmd.cmd_extension == 0x00){  //���ͬ����ϵ
		mi_cmd.data.axis_index = cmd.data[0]+1;   //�Ӷ���ţ���1��ʼ
		mi_cmd.data.data[0] = cmd.data[1]+1;      //������ţ���1��ʼ
		mi_cmd.data.data[1] = 0;

		this->m_p_mi_comm->WriteCmd(mi_cmd);

		cmd.data[cmd.data_len] = SUCCEED;
		cmd.data_len++;
	}else if(cmd.cmd_extension == 0x01){  //����ͬ����ϵ
		mi_cmd.data.axis_index = cmd.data[0]+1;   //�Ӷ���ţ���1��ʼ
		mi_cmd.data.data[0] = cmd.data[1]+1;      //������ţ���1��ʼ
		mi_cmd.data.data[1] = 1;

		this->m_p_mi_comm->WriteCmd(mi_cmd);

		cmd.data[cmd.data_len] = SUCCEED;
		cmd.data_len++;
	}else{
		cmd.cmd_extension = FAILED;
	}

	this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief ����HMI��ѯͬ����ʹ������
 * @param cmd : HMIָ���
 */
void ChannelEngine::ProcessHmiCheckSyncCmd(HMICmdFrame &cmd){
	cmd.frame_number |= 0x8000;

	uint8_t axis_index = cmd.data[0];

	if(this->m_n_sync_axis_enable_mask & (0x01<<axis_index))
		cmd.data[1] = 1;
	else
		cmd.data[1] = 0;

	cmd.data_len = 2;
	this->m_p_hmi_comm->SendCmd(cmd);
}


/**
 * @brief ����HMI֪ͨͼ��ģʽ����
 * @param cmd : HMIָ���
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
 * @brief ����HMI���ƶ�ָ��
 * @param cmd : HMIָ���
 */
void ChannelEngine::ProcessHmiAxisMoveCmd(HMICmdFrame &cmd){
	cmd.frame_number |= 0x8000;  //���ûظ���־
	uint8_t axis = 0;
	uint32_t speed = 0;
	int8_t dir = 0;
	double tar_pos = 0;

	if(cmd.data_len == 6){ //δָ��Ŀ��λ��
		axis = cmd.data[0];
		memcpy(&speed, &cmd.data[1], 4);
		dir = cmd.data[5];
		double vel = speed;

		if(this->m_p_axis_config[axis].axis_pmc){ //PMC��
			if(speed > 0)
				this->ManualMovePmc(axis, 99999, vel, true);
			else
				this->ManualMoveStop(axis);
		}else{
			if(speed > 0)
				this->ManualMove(axis, dir, vel, 99999);
			else
				this->ManualMoveStop(axis);
		}

		printf("ChannelEngine::ProcessHmiAxisMoveCmd:axis=%hhu, vel=%lf, dir=%hhu\n", axis, vel, dir);

		cmd.cmd_extension = SUCCEED;

	}else if(cmd.data_len == 14){//ָ��Ŀ��λ��
		axis = cmd.data[0];
		memcpy(&speed, &cmd.data[1], 4);
		dir = cmd.data[5];
		memcpy(&tar_pos, &cmd.data[6], sizeof(double));  //����Ŀ��λ��
		double vel = speed;
		if(speed == 0)
			vel = this->m_p_axis_config[axis].rapid_speed;  //�ٶȸ�0��ʹ��������еĶ�λ�ٶ�

		if(this->m_p_axis_config[axis].axis_pmc){ //PMC��
			this->ManualMovePmc(axis, tar_pos, vel, false);
		}else{
			this->ManualMove(axis, dir, vel, tar_pos-this->GetPhyAxisMachPosFeedback(axis));
		}
		printf("ChannelEngine::ProcessHmiAxisMoveCmd:axis=%hhu, vel=%lf, tar=%lf\n", axis, vel, tar_pos);
		cmd.cmd_extension = SUCCEED;
	}else{//�������ʽ����
		cmd.cmd_extension = FAILED;
	}

	this->m_p_hmi_comm->SendCmd(cmd);
}


/**
 * @brief ��������л�ȡ��������
 * @param data : ���������ṹ
 * @param src : ����Դ
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
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "��������ָ�����ֵ���ͷǷ���%hhu", data->value_type);
		break;
	}
}

/**
 * @brief ����HMI���ò���ָ��
 * @param cmd : ָ��
 */
void ChannelEngine::ProcessHmiSetParam(HMICmdFrame &cmd){
//	uint32_t param_no = 0;	//������
	uint8_t active_type = ACTIVE_BY_POWEROFF;	//��������
//	uint8_t value_type = VALUE_UINT8;		//ֵ����
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
	case TOOL_OFFSET_CONFIG:	//����ƫ��
		if(cmd.channel_index >= m_p_general_config->chn_count ||     //ͨ����������Χ
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
	case TOOL_POT_CONFIG:		//����λ��    ������Ч
		if(cmd.channel_index >= m_p_general_config->chn_count){//ͨ����������Χ
			cmd.data[0] = FAILED;
		}
		else{
			g_ptr_parm_manager->UpdateToolPotConfig(cmd.channel_index, *((SCToolPotConfig*)cmd.data));
			cmd.data[0] = SUCCEED;
		}
		break;
	case COORD_CONFIG:			//��������ϵ
		if(cmd.channel_index >= m_p_general_config->chn_count ||        //ͨ����������Χ
				(uint8_t)cmd.data[0] >= kWorkCoordCount){						//������������Χ
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
	case EX_COORD_CONFIG:		//��չ��������ϵ
		if(cmd.channel_index >= m_p_general_config->chn_count ||			//ͨ����������Χ
				(uint8_t)cmd.data[0] >= m_p_channel_config[cmd.channel_index].ex_coord_count){	//������������Χ
			cmd.data[0] = FAILED;
		}
		else{
			HmiCoordConfig cfg;

			memcpy(&cfg, cmd.data+1, cmd.data_len-1);
			this->m_p_channel_control[cmd.channel_index].UpdateExCoord(cmd.data[0], cfg);


			cmd.data[0] = SUCCEED;

		}
		break;
	case PITCH_COMP_DATA:   //�ݲ�����
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
	this->m_p_hmi_comm->SendCmd(cmd);	//������Ӧ
}

/**
 * @brief ����HMI��ȡ����ָ��
 * @param cmd : ָ��
 */
void ChannelEngine::ProcessHmiGetParam(HMICmdFrame &cmd){
	uint8_t index = 0;

	switch(cmd.cmd_extension){
	case SYS_CONFIG:
		cmd.data_len = sizeof(HmiSystemConfig);
		memcpy(cmd.data, m_p_general_config, cmd.data_len);
		break;
	case CHN_CONFIG:
		if(cmd.channel_index >= m_p_general_config->max_chn_count)//ͨ����������Χ, ֧�ֲ����޸ĺ�һ������
			cmd.data_len = 0;
		else{
			cmd.data_len = sizeof(HmiChnConfig);
			memcpy(cmd.data, &m_p_channel_config[cmd.channel_index], cmd.data_len);
		}
		break;
	case AXIS_CONFIG:
		index = cmd.data[0];
//		printf("get axis config %hhu, frameindex=%hu\n", index, cmd.frame_number);
		if(index >= this->m_p_general_config->max_axis_count){//��ų���Χ, ֧�ֲ����޸ĺ�һ������
			printf("get axis config failed, index=%hhu, axis_count:%hhu\n", index, m_p_general_config->max_axis_count);
			cmd.data_len = 0;
		}
		else{
			cmd.data_len = sizeof(HmiAxisConfig);
			memcpy(cmd.data+1, &this->m_p_axis_config[index], cmd.data_len);
			cmd.data_len += 1;
	//		printf("get axis config succeed, data_len:%hu\n", cmd.data_len);
		}

		break;
	case TOOL_OFFSET_CONFIG:{
		index = cmd.data[0];
//		printf("get cmd TOOL_OFFSET_CONFIG, chn_index =  %hu, tool_index = %hhu\n", cmd.channel_index, index);

#ifdef USES_INDEPEND_BASE_TOOL_OFFSET
		if((index != 0xFF && index >= kMaxToolCount) || cmd.channel_index >= m_p_general_config->chn_count){
			cmd.data_len = 0;
		}else if(index == 0xFF){   //��ȡ��׼������
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
		if(cmd.channel_index >= m_p_general_config->chn_count)//ͨ����������Χ
			cmd.data_len = 0;
		else{
			cmd.data_len = sizeof(HmiToolPotConfig);
			memcpy(cmd.data, g_ptr_parm_manager->GetToolPotConfig(cmd.channel_index), cmd.data_len);
		}
		break;
	case COORD_CONFIG:
		index = cmd.data[0];
		if(cmd.channel_index >= m_p_general_config->chn_count ||
				index >= kWorkCoordCount)//ͨ����������Χ
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
				index >= m_p_channel_config[cmd.channel_index].ex_coord_count)//ͨ����������Χ
			cmd.data_len = 0;
		else{
			cmd.data_len = sizeof(HmiCoordConfig);
			memcpy(cmd.data+1, &g_ptr_parm_manager->GetExCoordConfig(cmd.channel_index)[index], cmd.data_len);
			cmd.data_len += 1;
		}
		break;

	case PITCH_COMP_DATA:  //�ݲ�����
		this->ProcessHmiGetPcDataCmd(cmd);
		break;
#ifdef USES_FIVE_AXIS_FUNC
	case FIVE_AXIS_CONFIG:
		if(cmd.channel_index >= m_p_general_config->max_chn_count)//ͨ����������Χ
			cmd.data_len = 0;
		else{
			cmd.data_len = sizeof(FiveAxisConfig);
			memcpy(cmd.data, &m_p_chn_5axis_config[cmd.channel_index], cmd.data_len);
		}
		break;
#endif
	default:
		cmd.data_len = 0;
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�յ�HMI������ȡָ��������ͷǷ�[%d]", cmd.cmd_extension);
		break;
	}

	cmd.frame_number |= 0x8000;
	this->m_p_hmi_comm->SendCmd(cmd);	//������Ӧ
}


/**
 * @brief ����HMI��ȡ�ݲ�����ָ��
 * @param cmd : HMI���͵�ָ��
 */
void ChannelEngine::ProcessHmiGetPcDataCmd(HMICmdFrame &cmd){
	if(cmd.data_len != 6){  //��ʽ��ƥ�䣬����ʧ��
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

	if(axis_index == 0xFF){//������˳���ȡ
		uint16_t tmp_count = count, tmp_start = offset-1;
		uint16_t tmp_end = tmp_start + count -1;
		uint16_t tc = 0;  //���ο�������
		uint16_t tmp_copy = 0;  //�ѿ�������
		int dc = sizeof(double);   //double�����ֽڴ�С

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

				if(tmp_count > 0){//����Ҫ��������
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
		if(axis_index >= this->m_p_general_config->axis_count ||              //����Ų���
			offset+count-1 > m_p_axis_config[axis_index].pc_count){  //��ȡ����������
			cmd.data_len = 0;
			printf("ProcessHmiGetPcDataCmd return, axis_index=%hhu, offset=%hu, count=%hu\n", axis_index, offset, count);
			return;
		}

		double *pp = m_p_pc_table->pc_table[axis_index];
		if(dir)  //�����ݲ�
			memcpy(&cmd.data[6], &pp[offset-1], sizeof(double)*count);
		else if(this->m_p_axis_config[axis_index].pc_type == 1)  //�����ݲ�
			memcpy(&cmd.data[6], &pp[m_p_axis_config[axis_index].pc_count+offset-1], sizeof(double)*count);
		else{ //�����ݲ�ģʽ����ȡ�����ݲ�ʧ��
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
 * @brief �����ݲ���������
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelEngine::ProcessPcDataImport(){
	bool res = true;

	int fd = -1;                    //�ļ����

	struct stat statbuf;
	int file_size = 0;
	uint8_t phy_axis = 0;   //�������
	uint8_t pc_type = 0;    //�ݲ�����
	uint16_t point_count = 0;  //�ݲ�����
	double inter_dis = 0;   //�ݲ����  ��λ��mm
	double *data = nullptr;

	if(stat(PATH_PC_DATA_TMP, &statbuf) == 0)
		file_size = statbuf.st_size;  //��ȡ�ļ��ܴ�С
	else{
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "��ȡ�ļ�[%s]��Сʧ�ܣ�", PATH_PC_DATA_TMP);
		CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 1);   //��ȡ�ļ���Сʧ��
		return false;
	}

	if(file_size < 12){ //�ݲ������ļ�ͷ��12�ֽڹ̶�����
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�ļ�[%s]��С[%d�ֽ�]��ƥ�䣡", PATH_PC_DATA_TMP, file_size);
		CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 2);   //��ȡ�ļ���Сʧ��
		return false;
	}


	fd = open(PATH_PC_DATA_TMP, O_RDONLY);

	if(fd == -1){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "���ļ�ʧ��[%s]", PATH_PC_DATA_TMP);
		CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 3);   //���ļ�ʧ��
		return false;
	}


	read(fd, &phy_axis, 1);   //��ȡ��������
	read(fd, &pc_type, 1);    //��ȡ�ݲ����ͣ�0--����  1--˫��  �ݲ��ļ�����
	read(fd, &point_count, 2);  //��ȡ�ݲ�����
	read(fd, &inter_dis, 8);    //��ȡ�������
/***********************************************************************/
	if(this->m_p_axis_config[phy_axis].pc_type != pc_type){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�ݲ����������ò���[%hhu : %hhu]��", m_p_axis_config[phy_axis].pc_type, pc_type);
		CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 4);   //�ݲ����Ͳ�ƥ��
		close(fd);
		return false;
	}

	if(fabs(m_p_axis_config[phy_axis].pc_inter_dist) != fabs(inter_dis)){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�ݲ�������������ò���[%lf : %lf]��", fabs(m_p_axis_config[phy_axis].pc_inter_dist), fabs(inter_dis));
		CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 5);   //�ݲ�����ƥ��
		close(fd);
		return false;
	}

	if(this->m_p_axis_config[phy_axis].pc_count != point_count){
		g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_ENGINE_SC, "�ݲ����������ò���[%hu : %hu]��", m_p_axis_config[phy_axis].pc_count, point_count);
		CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 6);   //�ݲ�����ƥ��
		close(fd);
		return false;
	}


	if(pc_type == 1){//˫���ݲ�
		point_count *= 2;   //˫���ݲ���������Ϊ������2
	}

	if((file_size-12) != (int)(point_count*sizeof(double))){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�ݲ����ݴ�С[%d�ֽ�]�벹������[%hhu]��ƥ�䣡", file_size-12, point_count);
		CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 7);   //��ȡ�ļ���Сʧ��
		close(fd);
		return false;
	}

	bool dir_pos = true;  //��ʼ�ݲ�����Ϊ����Ĭ������
	if(pc_type == 1 && inter_dis < 0)
		dir_pos = false;

	data = new double[point_count];
	if(data == nullptr){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�ݲ����ݵ��룬���仺��ʧ�ܣ�");
		CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 8);   //���仺��ʧ��
		close(fd);
		return false;
	}

	//��ȡ����
	read(fd, data, sizeof(double)*point_count);

	close(fd);

	//д������
	if(pc_type == 0){  //�����ݲ�
		res = g_ptr_parm_manager->UpdatePcData(phy_axis, dir_pos, 1, point_count, data);
	}else if(pc_type == 1){  //˫���ݲ�
		int pcs = point_count/2;
	    res = g_ptr_parm_manager->UpdatePcData(phy_axis, dir_pos, 1, pcs, data);
	    if(res){
	    	int pcs_half = pcs/2;
	    	double tb = 0;
	    	for(int i =0; i < pcs_half; i++){  //����˳��
	    		tb = data[point_count-1-i];
	    		data[point_count-1-i] = data[pcs+i];
	    		data[pcs+i] = tb;
	    	}
	    	res = g_ptr_parm_manager->UpdatePcData(phy_axis, !dir_pos, 1, pcs, data+pcs);
	    }
	}

	delete []data;  //�ͷŻ���

	if(!res){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�ݲ����ݵ��룬���仺��ʧ�ܣ�");
		CreateError(ERR_IMPORT_PC_DATA, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 8);   //���仺��ʧ��
		close(fd);
		return false;
	}

//	if(res == -1 || res != read_size){  //readʧ��
//		printf("read pmc file failed, errno = %d, block = %d\n", errno, block_total);
//		close(fp);
//		return 0;
//	}

	this->NotifyHmiPitchCompDataChanged();  //֪ͨHMI���»�ȡ�ݲ�����

	//�������ݸ�MC
	//�������ݲ����ݱ�
	this->SendMiPcData(phy_axis);

	//�������ݲ����ò���
	this->SendMiPcParam(phy_axis);
	this->SendMiPcParam2(phy_axis);

	g_ptr_trace->PrintLog(LOG_CONFIG_MODIFY, "��%hhu���ݲ����ݵ���ɹ���", phy_axis+1);
	return true;
}

#ifdef USES_WOOD_MACHINE
/**
 * @brief ���浶����Ϣ����
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


/**
 * @brief ����HMI�����ݲ�����
 * @param cmd : HMI���͵����ݸ��°�
 * @return true--ִ�гɹ�   false--ִ��ʧ��
 */
bool ChannelEngine::UpdateHmiPitchCompData(HMICmdFrame &cmd){


	uint8_t axis_index = cmd.data[0];
	bool dir = (cmd.data[1]==0)?true:false;   //�����ݲ���־
	uint16_t offset, count;
	memcpy(&offset, &cmd.data[2], 2);
	memcpy(&count, &cmd.data[4], 2);

	printf("UpdateHmiPitchCompData::axis=%hhu, dir=%hhu, offset=%hu, count=%hu\n", axis_index, dir, offset, count);

	if(cmd.data_len != 6+sizeof(double)*count ){
		printf("UpdateHmiPitchCompData: ERROR in data format, datalen=%hu, count=%hu\n", cmd.data_len, count);
		return false;
	}

	double *data = (double *)&cmd.data[6];

	if(axis_index == 0xFF){//������˳������
		uint16_t tmp_start = offset-1;

		ListNode<AxisPcDataAlloc> *node = this->m_list_pc_alloc.HeadNode();
		while(node != nullptr){

			//printf("node->data.start_index : %d node->data.end_index: %d tmp_start: %d\n",
			//	node->data.start_index, node->data.end_index, tmp_start);

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

	}else{//��˳������
		if(!g_ptr_parm_manager->UpdatePcData(axis_index, dir, offset, count, data)){
			printf("UpdateHmiPitchCompData: failed to update pc data2\n");
			return false;
		}
	}

	//�������ݸ�MC
	//�������ݲ����ݱ�
	this->SendMiPcData(axis_index);

	//�������ݲ����ò���
	this->SendMiPcParam(axis_index);
	this->SendMiPcParam2(axis_index);


	return true;
}

/**
 * @brief ����HMI��ȡPMC�Ĵ���ֵ��ָ��
 * @param cmd
 */
void ChannelEngine::ProcessHmiGetPmcReg(HMICmdFrame &cmd){
	uint16_t reg_sec = 0, reg_index = 0;
	uint16_t reg_count = 0;
	uint16_t *reg_value16 = nullptr;
	uint8_t *reg_value8 = nullptr;


	memcpy(&reg_sec, cmd.data, 2);   //�Ĵ�����
	memcpy(&reg_index, &cmd.data[2], 2);   	//�Ĵ�������������
	if(cmd.data_len == 6)
		memcpy(&reg_count, &cmd.data[4], 2);   //�Ĵ�����ַ����
	else
		reg_count = 1;   //�����ϰ汾Э��

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
		cmd.cmd_extension = FAILED;  //ʧ��
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
 * @brief ����HMI����PMC��UUIDָ��
 * @param cmd
 */
void ChannelEngine::ProcessHmiGetPmcUuid(HMICmdFrame &cmd){
	char path[kMaxPathLen];
	char uuid[20];
	strcpy(path, PATH_PMC_DATA);
	memset(uuid, 0x00, 20);

	if(access(path, F_OK) == -1){	//�ļ�������
		cmd.cmd_extension = FAILED;
		printf("#####ProcessHmiGetPmcUuid:failed!\n");
	}else{
		cmd.cmd_extension = SUCCEED;

		FILE *file_src = fopen(path, "r+");
		if (nullptr == file_src)
		{
			cmd.cmd_extension = FAILED;
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMCִ���ļ���ʧ�ܣ�errno = %d", errno);
		}else{
			int read = fread(uuid, 1, 16, file_src);
			if(read != 16){
				cmd.cmd_extension = FAILED;
				g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMCִ���ļ���ȡUUIDʧ�ܣ�errno = %d", errno);
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
 * @brief ����HMI����PMC�Ĵ���ֵ��ָ��
 * @param cmd
 */
void ChannelEngine::ProcessHmiSetPmcReg(HMICmdFrame &cmd){

	uint16_t reg_sec = 0, reg_index = 0;
	uint8_t reg_value8 = 0;
	uint8_t bit_index = 0;
	uint8_t bit_count = 0;
	uint32_t bit_value32 = 0;
	bool bit_opt = cmd.cmd_extension==1?true:false;  //�Ƿ�λ����

#ifndef USES_PMC_2_0
	uint16_t reg_value16 = 0;
#endif

	memcpy(&reg_sec, cmd.data, 2);   		//�Ĵ�����
	memcpy(&reg_index, &cmd.data[2], 2);   	//�Ĵ�������������

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
				printf("��ȴ������\n");
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
		cmd.cmd_extension = FAILED;  //ʧ��
		break;
	}

	cmd.frame_number |= 0x8000;

	this->m_p_hmi_comm->SendCmd(cmd);
}

/**
 * @brief ����HMI��������
 * @param cmd :  �������ݰ�
 */
void ChannelEngine::ProcessHmiUpdateReq(HMICmdFrame &cmd){
	bool flag = true;//�ܷ�����

	if(this->m_n_update_state == MODULE_UPDATE_NONE){
		HmiChannelStatus chn_status;

		//��ѯ��ͨ����ǰ�ӹ�״̬
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

	//���updateĿ¼�µ��ļ�
	//��������ļ�����
	char filepath[kMaxPathLen] = {0};
	strcpy(filepath, PATH_UPDATE_PATH);
	strcat(filepath, "*.*");
	if( 0 != remove(filepath)){
		//��������ļ�ʧ��
		g_ptr_trace->PrintTrace(TRACE_WARNING, CHANNEL_ENGINE_SC, "ģ������ǰ��������ļ���ʧ�ܣ�errno = %d", errno);
	}

	cmd.frame_number |= 0x8000;
	cmd.data_len = 1;
	cmd.data[0] = flag?APPROVE:REFUSE;
	this->m_p_hmi_comm->SendCmd(cmd);	//������Ӧ
}

/**
 * @brief ����PMC��ȷ�ϲο���ָ��
 * @param phy_axis : ������ţ���0��ʼ
 */
void ChannelEngine::ProcessPmcAxisFindRef(uint8_t phy_axis){
	if(m_p_axis_config[phy_axis].axis_interface != VIRTUAL_AXIS		//��������
		&& m_p_axis_config[phy_axis].axis_type != AXIS_SPINDLE				//������
		&& /*(m_p_axis_config[phy_axis].feedback_mode == INCREMENTAL_ENCODER ||
				m_p_axis_config[phy_axis].feedback_mode == NO_ENCODER)   //����ʽ�����������޷���
		&& */m_p_axis_config[phy_axis].ret_ref_mode > 0){	//�ǽ�ֹ�زο���

#ifdef USES_PMC_PROCESS
		if(phy_axis == 5)
			this->m_p_pmc_reg->FReg().bits[0].OUT8 = 1;  //pmc X�����
		else if(phy_axis == 6)
			this->m_p_pmc_reg->FReg().bits[0].OUT9 = 1;  //pmc Y�����
		else if(phy_axis == 7)
			this->m_p_pmc_reg->FReg().bits[0].OUT10 = 1;  //pmc ��Z�����
		else if(phy_axis == 8)
			this->m_p_pmc_reg->FReg().bits[0].OUT11 = 1;  //pmc ��Z�����
#else
		//ϵͳ����
		this->m_n_mask_ret_ref |= (0x01<<phy_axis);
		this->m_b_ret_ref = true;
		this->m_b_ret_ref_auto = false;
#endif
	}else{
		if(this->m_p_axis_config[phy_axis].axis_pmc > 0){
			this->m_pmc_axis_ctrl[m_p_axis_config[phy_axis].axis_pmc-1].ExecCmdOver(true);
		}
	}
}

/**
 * @brief ����HMI�زο���ָ�����������ʽ������
 * @param cmd �� HMIָ��
 */
void ChannelEngine::ProcessHmiFindRefCmd(HMICmdFrame &cmd){
	printf("ChannelEngine::ProcessHmiFindRefCmd, ext = 0x%hx, chn=0x%hx\n", cmd.cmd_extension, cmd.channel_index);
	if(this->m_b_ret_ref){ //�Ѿ��ڻزο���
		printf("�Ѵ��ڻزο���������,�ܾ���ret_ref_mask = 0x%llx\n", this->m_n_mask_ret_ref);
		cmd.data[0] = FAILED;

	}else if(g_ptr_alarm_processor->HasErrorInfo()){ //�и澯���ܾ�����
		printf("�и澯���ܾ����㣡\n");
		cmd.data[0] = FAILED;
	}else if(cmd.cmd_extension == 0x00){//��ǰ�����
		if(this->m_n_cur_pmc_axis != 0xFF){
			this->ProcessPmcAxisFindRef(m_n_cur_pmc_axis);
		}else{
			this->m_p_channel_control[this->m_n_cur_channle_index].ProcessHmiReturnRefCmd(false);
			this->m_b_ret_ref = true;
			this->m_b_ret_ref_auto = false;
		}


	}else if(cmd.cmd_extension == 0x10){ //ͨ�������
		this->m_p_channel_control[cmd.channel_index].ProcessHmiReturnRefCmd(true);
		this->m_b_ret_ref = true;
		this->m_b_ret_ref_auto = true;

	}else if(cmd.cmd_extension == 0xFF){  //���������
		for(int i = 0; i < this->m_p_general_config->axis_count; i++){
			if(m_p_axis_config[i].axis_interface != VIRTUAL_AXIS		//��������
				&& m_p_axis_config[i].axis_type != AXIS_SPINDLE				//������
				&& /*(m_p_axis_config[i].feedback_mode == INCREMENTAL_ENCODER ||
					m_p_axis_config[i].feedback_mode == NO_ENCODER)//����ʽ�����������޷���
				&& */m_p_axis_config[i].ret_ref_mode > 0){	 //�زο��㷽ʽ�ǽ�ֹ

				this->m_n_mask_ret_ref |= (0x01<<i);
			}
		}
		m_n_ret_ref_auto_cur = 0;
		this->m_b_ret_ref = true;
		this->m_b_ret_ref_auto = true;
	}else{//�Ƿ�
		printf("�Ƿ���cmd_ext=%hu, �ܾ���\n", cmd.cmd_extension);
		cmd.data[0] = FAILED;
	}

	cmd.frame_number |= 0x8000;
	cmd.data_len = 1;
	this->m_p_hmi_comm->SendCmd(cmd);

}

/**
 * @brief ����PMCָ������������
 * @param phy_axis : ��������ţ���0��ʼ
 */
void ChannelEngine::ProcessPmcRefRet(uint8_t phy_axis){
	printf("ChannelEngine::ProcessPmcRefRet, phy_axis=0x%hhx\n", phy_axis);
	if(phy_axis >= this->m_p_general_config->axis_count){
		printf("phy_axis %hhu over count %hhu, return\n", phy_axis, m_p_general_config->axis_count);
		return;
	}

	if(m_b_ret_ref || this->IsRefReturnning(phy_axis)){
		printf("axis %hhu is returnning ref, return\n", phy_axis);
		return;   //���ڻزο�������У�����
	}

	HmiChannelStatus chn_status;
	this->GetChnStatus(this->m_n_cur_channle_index, chn_status);
	if(chn_status.machining_state != MS_READY){   //��׼����ģʽ��ִ�л���
		printf("machine state is not ready:%hhu, return \n", chn_status.machining_state);
		return;
	}
	if(g_ptr_alarm_processor->HasErrorInfo()){ //�и澯���ܾ�����
		CreateError(ERR_RET_REF_FAILED, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, 0);
		return;
	}

	if(m_p_axis_config[phy_axis].axis_interface == VIRTUAL_AXIS || m_p_axis_config[phy_axis].axis_type == AXIS_SPINDLE	//����������᲻�ûزο���
		|| (m_p_axis_config[phy_axis].feedback_mode == NO_ENCODER && m_p_axis_config[phy_axis].ret_ref_mode == 0)    //�޷��������ҽ�ֹ�زο���
		|| (m_p_axis_config[phy_axis].feedback_mode == INCREMENTAL_ENCODER && m_p_axis_config[phy_axis].ret_ref_mode == 0)){  //��������������ֹ�زο���
		printf("no ret ref, return\n");
		return;   //���ûزο�������ֹ�������־��λ
	}

	this->SetRetRefMask(phy_axis);
	this->SetRetRefFlag(phy_axis, false);   //��λ�زο�����ɱ�־

	this->m_b_ret_ref = true;
	this->m_b_ret_ref_auto = false;
}

/**
 * @brief ���ûزο�����mask
 * @param phy_axis : ��������ţ�0��ʼ
 */
void ChannelEngine::SetRetRefMask(uint8_t phy_axis){
	if(m_p_axis_config[phy_axis].axis_interface != VIRTUAL_AXIS		//��������
		&& m_p_axis_config[phy_axis].axis_type != AXIS_SPINDLE				//������
	/*	&& (m_p_axis_config[phy_axis].feedback_mode == INCREMENTAL_ENCODER ||
				m_p_axis_config[phy_axis].feedback_mode == NO_ENCODER)*/){	//���з������Ͷ�֧�ֻزο���
		this->m_n_mask_ret_ref |= (0x01<<phy_axis);
	}
}

/**
 * @brief ������������ź�
 * @param phy_axis : ��������ţ�0��ʼ
 * @param flag : true--��1  flase--��0
 */
void ChannelEngine::SetInRetRefFlag(uint8_t phy_axis, bool flag){
	if(phy_axis >= this->m_p_general_config->axis_count){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ChannelEngine::SetInRetRefFlag() return, �������[%hhu: %hhu]�Ƿ�!", phy_axis, m_p_general_config->axis_count);
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
 * @brief ָ�����Ƿ����ڻ�����
 * @param phy_axis  : ��������ţ�0��ʼ
 * @return true--ָ����phy_axis���ڻ�����    false--δ�ڻ��������
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
 * @brief ����HMI���òο���ָ������ھ���ֵ������
 * @param cmd �� HMIָ��
 */
void ChannelEngine::ProcessHmiSetRefCmd(HMICmdFrame &cmd){

	printf("ChannelEngine::ProcessHmiSetRefCmd\n");

	if(this->m_n_cur_pmc_axis != 0xff){

		MiCmdFrame mi_cmd;
		memset(&mi_cmd, 0x00, sizeof(mi_cmd));
		mi_cmd.data.cmd = CMD_MI_SET_REF_CUR;
		mi_cmd.data.axis_index = m_n_cur_pmc_axis+1;
		int64_t pos = m_p_axis_config[m_n_cur_pmc_axis].axis_home_pos[0] * 1e7;   //��λת����0.1nm
		memcpy(mi_cmd.data.data, &pos, sizeof(int64_t));

		this->m_p_mi_comm->WriteCmd(mi_cmd);
	}

	cmd.data[0] = SUCCEED;
	cmd.data_len = 0x01;
	cmd.frame_number |= 0x8000;
	this->m_p_hmi_comm->SendCmd(cmd);

}



/**
 * @brief ��HMI��������״̬
 * @param total_step : �ܲ���
 * @param cur_step ����ǰ״̬
 */
void ChannelEngine::SendHmiUpdateStatus(uint8_t total_step, uint8_t cur_step){
	//��HMI��������״̬
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
 * @brief ��ȡָ����ͨ�����ƶ���ָ��
 * @param index
 * @return
 */
ChannelControl *ChannelEngine::GetChnControl(uint8_t index){
	return index < m_p_general_config->chn_count? &m_p_channel_control[index]:nullptr;
}

/**
 * @brief ��ȡͨ��״̬
 * @param chn_index[in] : ͨ������
 * @param status[out] �������ͨ��״̬
 * @return true--�ɹ�  false--ʧ��
 */
bool ChannelEngine::GetChnStatus(uint8_t chn_index, HmiChannelStatus &status){
	if(chn_index >= this->m_p_general_config->chn_count){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ͨ��������[%d]�Ƿ���", chn_index);
		return false;
	}

	this->m_p_channel_control[chn_index].GetChnStatus(status);
	return true;
}

/**
 * @brief ��ȡָ��ͨ����������Ӧ���������
 * @param chn_index[in] : ͨ������
 * @param chn_axis[in] ��ͨ�����
 * @return �����������,���ɹ�����0xff���ɹ�����0��ʼ��������������
 */
uint8_t ChannelEngine::GetChnAxistoPhyAixs(uint8_t chn_index, uint8_t chn_axis){
	if(chn_index >= this->m_p_general_config->chn_count){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ͨ��������[%d]�Ƿ���", chn_index);
		return 0xff;
	}

	uint8_t axis = this->m_p_channel_control[chn_index].GetPhyAxis(chn_axis);
	return axis;
}

/**
 * @brief �������������Ӧѭ�����
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelEngine::Start(){
	if(this->m_b_emergency)
		return false;

#ifdef USES_LICENSE_FUNC
	//���ϵͳʱ��
	if(m_ln_local_time < 0){//��ȡ���ؼ�ʱ�ļ��쳣
		if(m_ln_local_time == -1){
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "���ؼ�ʱ�ļ�������!");
		}else if(m_ln_local_time == -2){
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "���ؼ�ʱ�ļ�����!");
		}else if(m_ln_local_time == -3){
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�Ƿ����ؼ�ʱ�ļ�!");
		}
		m_error_code = ERR_SYSTEM_FILE;
		CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
		return false;
	}else if(-4 == CheckLocalTime(&m_lic_info, m_ln_local_time)){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ϵͳʱ���쳣!");
		m_error_code = ERR_SYSTEM_TIME;
		CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
		return false;
	}


	//�����Ȩ
	if(1 == this->CheckLicense()){  //�Ƿ���Ȩ
		return false;
	}
#endif

	//���ͬ����״̬
	if((m_n_sync_axis_mask & m_n_sync_axis_enable_mask) != this->m_n_sync_over){
		CreateError(ERR_SYNC_AXIS, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
		return false;
	}

	//����Ƿ�����������ݣ���ʾ����
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
			if(m_p_channel_control[chn].CheckFuncState(FS_HANDWHEEL_CONTOUR))   //�ٴη������ָ���״̬��MI����ֹMC��λʱ���״̬
				this->SetMiHandwheelTrace(true, chn);
			else
				this->SetMiHandwheelTrace(false, chn);

			m_p_channel_control[chn].StartRunGCode();
		}

	}else if(work_mode == MDA_MODE){
		m_p_channel_control[m_n_cur_channle_index].StartRunGCode();
	}

	return true;
}

/**
 * @brief ��ͣ����������Ӧ��������
 * @return true--�ɹ�   false--ʧ��
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
 * @brief ֹͣ����ִ�У����ڴ���ϵͳ�澯�µĳ���ֹͣ
 * @param reset : �Ƿ�λ���ݺ��кţ� true--��λ   false--����λ
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelEngine::Stop(bool reset){
	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
		m_p_channel_control[i].StopRunGCode(reset);
	}
	return true;
}

/**
 * @brief ֹͣ����ִ�У����ڴ���ϵͳ�澯�µĳ���ֹͣ
 * @param chn : ͨ����,��0��ʼ
 * @param reset : �Ƿ�λ���ݺ��кţ� true--��λ   false--����λ
 * @return true--�ɹ�   false--ʧ��
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
 * @brief ���õ�ǰͨ����
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

    // ͨ���仯ʱ ��F219 ǰ4λ��ֵ
    for(int i = 0; i < this->m_p_general_config->chn_count; i++){
    	m_p_pmc_reg->FReg().bits[i].CHNC = work_chan;
    }

    this->m_n_cur_chn_group_index = this->m_p_channel_config[work_chan].chn_group_index;

    this->SetMiCurChannel();
	return true;
}

/**
 * @brief ���ù���ģʽ, �Զ���MDA���ֶ�������
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
		CreateError(ERR_SWITCH_MODE_IN_RET_REF, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON);
		return false;
	}

	uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();
	for(uint8_t i = 0; i < chn_count; i++)
		this->m_p_channel_control[m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i)].SetWorkMode(work_mode);

	if(work_mode == MPG_MODE)
		this->SetMiWorkMode(0x10);
	else if(work_mode == AUTO_MODE || work_mode == MDA_MODE)
		this->SetMiWorkMode(0x00);
	else if(work_mode == MANUAL_STEP_MODE || work_mode == MANUAL_MODE || work_mode == REF_MODE)   //�ο���ģʽ��MI��Ч���ֶ�ģʽ
		this->SetMiWorkMode(0x20);

	return true;
}

/**
 * @brief ���ù���״̬�����磺���Σ�ѡͣ�ȵ�
 * @param chn �� ͨ���ţ� ��0��ʼ, 0xFF��ʾ������ͨ��
 * @param state : ���õ�״̬
 * @param mode : ״̬�Ŀ���   0--�ر�   1--��    10--�㶯�������ݵ�ǰ״̬ȡ����
 */
void ChannelEngine::SetFuncState(uint8_t chn, int state, uint8_t mode){
																		   
	if(chn >= this->m_p_general_config->chn_count && chn != CHANNEL_ENGINE_INDEX)
		return;  //ͨ���ŷǷ�
												   
	if(chn < m_p_general_config->chn_count){								   
		this->m_p_channel_control[chn].SetFuncState(state, mode);

		if(state == FS_HANDWHEEL_CONTOUR){ //���ָ��٣�֪ͨMI�л�״̬
			if(m_p_channel_control[chn].CheckFuncState(FS_HANDWHEEL_CONTOUR))
				this->SetMiHandwheelTrace(true, chn);
			else
				this->SetMiHandwheelTrace(false, chn);
		}
	}else{
		for(uint8_t i = 0; i < m_p_general_config->chn_count; i++){
			this->m_p_channel_control[i].SetFuncState(state, mode);
			if(state == FS_HANDWHEEL_CONTOUR){ //���ָ��٣�֪ͨMI�л�״̬
				if(m_p_channel_control[i].CheckFuncState(FS_HANDWHEEL_CONTOUR))
					this->SetMiHandwheelTrace(true, i);
				else
					this->SetMiHandwheelTrace(false, i);
			}
		}
	}

}

/**
 * @brief ��MI�������ַ������ʹ��
 */
void ChannelEngine::EnableHWTraceToMi(){
	uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();
	for(uint8_t i = 0; i < chn_count; i++){
		if(m_p_channel_control[i].CheckFuncState(FS_HANDWHEEL_CONTOUR))
			this->SetMiHandwheelTrace(true, i);
		else
			this->SetMiHandwheelTrace(false, i);
	}
}

/**
 * @brief �����Զ�����
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
 * @brief �����Զ�����
 * @param chn : ͨ���ţ���0��ʼ
 * @param ratio �� ����ֵ
 */
void ChannelEngine::SetAutoRatio(uint8_t chn, uint8_t ratio){
	if(this->m_p_channel_control[chn].GetAutoRatio() == ratio)
		return;
	this->m_p_channel_control[chn].SetAutoRatio(ratio);
}

/**
 * @brief �����ֶ�����
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
 * @brief �����ֶ�����
 * @param chn : ͨ���ţ���0��ʼ
 * @param ratio : ����ֵ
 */
void ChannelEngine::SetManualRatio(uint8_t chn, uint8_t ratio){
	printf("ChannelEngine::SetManualRatio, old = %hhu, new = %hhu\n", this->m_p_channel_control[chn].GetManualRatio(), ratio);
	if(this->m_p_channel_control[chn].GetManualRatio() == ratio)
		return;
	this->m_p_channel_control[chn].SetManualRatio(ratio);
}

/**
 * @brief ���ÿ��ٽ�������
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
 * @brief ���ÿ��ٽ�������
 * @param chn : ͨ���ţ���0��ʼ
 * @param ratio : ����ֵ
 */
void ChannelEngine::SetRapidRatio(uint8_t chn, uint8_t ratio){
	//����ֵӳ��
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
	if(this->m_p_channel_control[chn].GetRapidRatio() == ratio)
		return;
	this->m_p_channel_control[chn].SetRapidRatio(ratio);
}

/**
 * @brief �������ᱶ��
 * @param ratio
 */
void ChannelEngine::SetSpindleRatio(uint8_t ratio){
//	this->m_p_channel_control[m_n_cur_channle_index].SetSpindleRatio(ratio);
/*
	int chn_count = this->m_p_general_config->chn_count;
	for(int i = 0; i < chn_count; i++){
		this->m_p_channel_control[i].SetSpindleRatio(ratio);
	}*/

	uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();
	for(uint8_t i = 0; i < chn_count; i++)
		this->m_p_channel_control[m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i)].SetSpindleRatio(ratio);
}

/**
 * @brief �������ᱶ��
 * @param chn : ͨ���ţ���0��ʼ
 * @param ratio : ����ֵ
 */
void ChannelEngine::SetSpindleRatio(uint8_t chn, uint8_t ratio){
	if(this->m_p_channel_control[chn].GetSpindleRatio() == ratio)
		return;
	this->m_p_channel_control[chn].SetSpindleRatio(ratio);
}

/**
 * @brief �����ֶ�����
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

	uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();
	for(uint8_t i = 0; i < chn_count; i++)
		this->m_p_channel_control[m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i)].SetManualStep(tt);
}

/**
 * @brief �����ֶ�����
 * @param chn : ͨ���ţ���0��ʼ
 * @param step : ��������
 */
void ChannelEngine::SetManualStep(uint8_t chn, uint8_t step){
	switch(step){
	case 0:
		step = MANUAL_STEP_1;
		break;
	case 1:
		step = MANUAL_STEP_10;
		break;
	case 2:
		step = MANUAL_STEP_100;
		break;
	case 3:
		step = MANUAL_STEP_1000;
		break;
	}

	this->m_p_channel_control[chn].SetManualStep(step);
}

/**
 * @brief �����ֶ������ƶ�״̬
 * @param mode : ״̬�Ŀ���   0--�ر�   1--��    10--�㶯�������ݵ�ǰ״̬ȡ����
 */
void ChannelEngine::SetManualRapidMove(uint8_t mode){
//	this->m_p_channel_control[m_n_cur_channle_index].SetManualRapidMove();
/*
	int chn_count = this->m_p_general_config->chn_count;
	for(int i = 0; i < chn_count; i++){
		this->m_p_channel_control[i].SetManualRapidMove();
	}*/

	uint8_t chn_count = m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannelCount();   //��ǰ��ʽ��ͨ������
	for(uint8_t i = 0; i < chn_count; i++)
		this->m_p_channel_control[m_p_channel_mode_group[m_n_cur_chn_group_index].GetChannel(i)].SetManualRapidMove(mode);
}

/**
 * @brief ���õ�ǰ��
 * @param axis : ��ǰͨ�����
 */
void ChannelEngine::SetCurAxis(uint8_t axis){
//	int chn_count = this->m_p_general_config->chn_count;

	this->m_n_cur_pmc_axis = 0xFF;  //ȡ����ǰPMC��
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
 * @brief ���õ�ǰ��
 * @param chn : ͨ���ţ���0��ʼ
 * @param axis �� ͨ����ţ� ��0��ʼ
 */
void ChannelEngine::SetCurAxis(uint8_t chn, uint8_t axis){
	this->m_n_cur_pmc_axis = 0xFF;  //ȡ����ǰPMC��
	this->m_p_channel_control[chn].SetCurAxis(axis);
}

/**
 * @brief ���õ�ǰPMC��
 * @param axis : ������ţ���0��ʼ
 */
void ChannelEngine::SetCurPmcAxis(uint8_t axis){
	if(this->m_p_axis_config[axis].axis_pmc > 0)
		this->m_n_cur_pmc_axis = axis;
	else
		this->m_n_cur_pmc_axis = 0xFF;
}

/**
 * @brief ����ͨ���ӹ�״̬
 * @param chn : ͨ�������ţ�0��ʼ��0xFF��ʾͨ������
 * @param mach_state : �ӹ�״̬
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
 * @brief ������ʹ�ܣ��Լ��岹ģʽ��1--NC��岹   2--PMC��岹��
 * @param index : �����������0��ʼ
 */
 /*
void ChannelEngine::SetAxisOn(uint8_t index){
	McCmdFrame cmd;
	cmd.data.axis_index = index+1;
	cmd.data.channel_index = m_n_cur_channle_index;   // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

	cmd.data.cmd = CMD_MC_SET_AXIS_ON;

	//��ʹ��  0--��ֹ����   1--ʹ�ܸ���
	SCAxisConfig &axis_config = this->m_p_axis_config[index];
	if(axis_config.axis_type != AXIS_SPINDLE)
		cmd.data.data[0] = 1;
	else
		cmd.data.data[0] = 0;


	//��岹����   1--NC��岹���Զ����ֶ���MDI��  2--PMC��岹   ����ֵ��Ч  ��ע��10MC��DSP�ݲ�֧�� ����PMC�ᣩ
	cmd.data.data[1] = 1;

	m_p_mc_comm->WriteCmd(cmd);
}*/

/**
 * @brief ����ָ������ݾ༰����ٶȵȻ�����Ϣ
 * @param index : ��������, ��0��ʼ
 */
 /*
void ChannelEngine::SetAxisBaseParam(uint8_t index){
	McCmdFrame cmd;
	cmd.data.axis_index = index+1;
	cmd.data.channel_index = m_n_cur_channle_index;  // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

	cmd.data.cmd = CMD_MC_SET_AXIS_BASE_INFO;

	SCAxisConfig &axis_config = this->m_p_axis_config[index];
	//������   1--ֱ����   2--��ת��   3--����
	cmd.data.data[0] = axis_config.axis_type+1;

	//�ݾ࣬��λ��1um
	uint32_t data = axis_config.move_pr * 1e3;  //mm->1um
	cmd.data.data[1] = (data&0xFFFF);
	cmd.data.data[2] = ((data>>16)&0xFFFF);

	//����ٶ����ƣ� ��λ��um/s
	data = axis_config.move_pr * axis_config.motor_speed_max * 1000 /60;   //mm/min -> um/s
	cmd.data.data[3] = (data&0xFFFF);
	cmd.data.data[4] = ((data>>16)&0xFFFF);

	m_p_mc_comm->WriteCmd(cmd);

}*/

/**
 * @brief ����ָ������ٶ������Ϣ
 * @param index : ��������, ��0��ʼ
 */
 /*
void ChannelEngine::SetAxisSpeedParam(uint8_t index){
	McCmdFrame cmd;
	cmd.data.axis_index = index+1;
	cmd.data.channel_index = m_n_cur_channle_index;  // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

	cmd.data.cmd = CMD_MC_SET_AXIS_SPEED_LIMIT;

	SCAxisConfig &axis_config = this->m_p_axis_config[index];
	//G00�ٶȣ���λ��um/s
	uint32_t data = axis_config.rapid_speed * 1000 / 60;  //mm/min -> um/s
//	printf("set axis %hhu g00 speed: %u\n", index, data);
	cmd.data.data[0] = (data&0xFFFF);
	cmd.data.data[1] = ((data>>16)&0xFFFF);
//	printf("set axis %hhu g00 speed: 0x%hx,  0x%hx\n", index, cmd.data.data[0], cmd.data.data[1]);

	//ת���ٶȲ����ƣ� ��λ��um/s
	data = axis_config.corner_acc_limit * 1000 /60;   //mm/min -> um/s
	cmd.data.data[2] = (data&0xFFFF);
	cmd.data.data[3] = ((data>>16)&0xFFFF);

	//�ֶ������ٶȣ���λ��um/s  //TODO �˲������������ã��˴�����һ���̶�ֵ100mm/min
	data = 100 * 1000 / 60;    //mm/min -> um/s
	cmd.data.data[4] = (data&0xFFFF);
	cmd.data.data[5] = ((data>>16)&0xFFFF);

	m_p_mc_comm->WriteCmd(cmd);
}*/

/**
 * @brief ����ָ������ٶ������Ϣ
 * @param index : ��������, ��0��ʼ
 */
 /*
void ChannelEngine::SetAxisAccParam(uint8_t index){
	McCmdFrame cmd;
	cmd.data.axis_index = index+1;
	cmd.data.channel_index = m_n_cur_channle_index;  // cmd.data.channel_index = CHANNEL_ENGINE_INDEX;

	cmd.data.cmd = CMD_MC_SET_AXIS_ACC;
//	printf("set axis[%hhu] acc:rap = %lf, start = %lf\n", index, m_p_axis_config[index].rapid_acc,
//			m_p_axis_config[index].start_acc);

	//G00���ٶȣ���λ��10mm/s^2
	uint16_t data = m_p_axis_config[index].rapid_acc / 10;
	cmd.data.data[0] = data;

	//�ֶ����ٶȣ� ��λ��10mm/s^2
	data = m_p_axis_config[index].manual_acc /10;
	cmd.data.data[1] = data;

	//�ֶ����ɼ��ٶȣ���λ��10mm/s^2
	data = m_p_axis_config[index].start_acc /10;
	cmd.data.data[2] = data;

	//G00 S��ʱ�䳣��
	cmd.data.data[3] = m_p_axis_config[index].rapid_s_plan_filter_time;

	m_p_mc_comm->WriteCmd(cmd);
}*/

/**
 * @brief ����PMC���˶�ָ��
 * @param cmd : PMC���˶�ָ��
 */
void ChannelEngine::SendPmcAxisCmd(PmcCmdFrame &cmd){
	uint8_t phy_axis = cmd.data.axis_index-1;

	this->m_n_run_axis_mask |= 0x01L<<phy_axis;  //���õ�ǰ������

	this->m_p_mi_comm->SendPmcCmd(cmd);
}

/**
 * @brief ��ȡpmc����趨��λ�ٶ�
 * @param axis : ������ţ� ��0��ʼ
 * @return ���ش�����������õĶ�λ�ٶ�
 */
uint32_t ChannelEngine::GetPmcAxisRapidSpeed(uint8_t axis){
	if(axis >= this->m_p_general_config->axis_count)
		return 0;
	return this->m_p_axis_config[axis].rapid_speed;
}

/**
 * @brief �ֶ��ƶ�
 * @param dir : �˶�����
 */
void ChannelEngine::ManualMove(int8_t dir){
	if(this->m_b_emergency)
		return;
	if(this->m_n_cur_pmc_axis != 0xFF){  //�ƶ�PMC��
		this->ManualMovePmc(dir);
	}else
		m_p_channel_control[m_n_cur_channle_index].ManualMove(dir);
}

/**
 * @brief �ֶ���Ŀ���ٶ�vel�ƶ�������Ŀ��λ��
 * @param phy_axis : ������ţ���0��ʼ
 * @param vel : �˶��ٶ�, ��λ��mm/min
 * @param pos : ����λ��
 */
void ChannelEngine::ManualMoveAbs(uint8_t phy_axis, double vel, double pos){
	int64_t cur_pos = this->m_df_phy_axis_pos_feedback[phy_axis]*1e7;  //��ǰλ��
	//����Ŀ��λ��
	int64_t tar_pos = pos * 1e7;   //��λת����mm-->0.1nms
	int8_t dir = (tar_pos > cur_pos)?DIR_POSITIVE:DIR_NEGATIVE;  //�ƶ�����

	//���Ӳ��λ
	if(CheckAxisHardLimit(phy_axis, dir)){   //Ӳ��λ�澯��ֱ�ӷ���
		printf("hard limit active, manual move abs return \n");
		return;
	}


	uint8_t chn_axis = 0, chn = 0;
	chn = this->GetAxisChannel(phy_axis, chn_axis);

	if(m_p_axis_config[phy_axis].axis_pmc == 0 && chn != CHANNEL_ENGINE_INDEX){ //ͨ����
		McCmdFrame cmd;
		memset(&cmd, 0x00, sizeof(McCmdFrame));

		cmd.data.channel_index = chn;
		cmd.data.axis_index = chn_axis+1;   //��Ŵ�1��ʼ
		cmd.data.cmd = CMD_MC_AXIS_MAUAL_MOVE;

		//�����ٶ�
		uint32_t feed = vel*1000/60;   //ת����λΪum/s

	//	memcpy(cmd.data.data, &feed, sizeof(feed));
		cmd.data.data[0] = (feed & 0xFFFF);
		cmd.data.data[1] = ((feed>>16)&0xFFFF);


		if((this->m_n_mask_ret_ref_over & (0x01<<phy_axis))
				&& m_p_axis_config[phy_axis].soft_limit_check_1 == 1){//����λ1��Ч

			int64_t limit_pos = 0;

			if(dir == DIR_POSITIVE){//����
				limit_pos = m_p_axis_config[phy_axis].soft_limit_max_1*1e7;


				if(limit_pos < cur_pos){//�Ѿ�����λ�⣬�澯
					CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, chn, chn_axis);
				//	this->m_error_code = ERR_SOFTLIMIT_POS;
					return;
				}else if(limit_pos == cur_pos){  //�Ѿ���������λ����λ��
					CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, chn, chn_axis);
					return;
				}else if(tar_pos > limit_pos){
					tar_pos = limit_pos;

				}
			}else if(dir == DIR_NEGATIVE){//����
				limit_pos = m_p_axis_config[phy_axis].soft_limit_min_1*1e7;

				if(limit_pos > cur_pos){//�Ѿ�����λ�⣬�澯
					CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON, 0, chn, chn_axis);
				//	this->m_error_code = ERR_SOFTLIMIT_NEG;
					return;
				}else if(limit_pos == cur_pos){  //�Ѿ����︺��λ����λ��
					CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON, 0, chn, chn_axis);
					return;
				}else if(tar_pos < limit_pos){
					tar_pos = limit_pos;
				}
			}
		}

		cmd.data.data[2] = (tar_pos & 0xFFFF);
		cmd.data.data[3] = ((tar_pos>>16)&0xFFFF);
		cmd.data.data[4] = ((tar_pos>>32) & 0xFFFF);
		cmd.data.data[5] = ((tar_pos>>48)&0xFFFF);

		cmd.data.data[6] = 0x00;   //����Ŀ��λ�ã���е����ϵ

		if(!this->m_mc_run_on_arm[chn])
			m_p_mc_comm->WriteCmd(cmd);
		else
			m_p_mc_arm_comm->WriteCmd(cmd);
		
		printf("ChannelEngine::ManualMoveAbs: axis = %d, tar_pos = %lld\n", phy_axis, tar_pos);
	}else if(m_p_axis_config[phy_axis].axis_pmc){  //PMC��
#ifdef USES_GRIND_MACHINE
		//����ǰ��צ�����ϵ�λ��������ʩ
		if(this->m_p_pmc_reg->GReg().bits[0].left_claw_up_check == 0 ||
				this->m_p_pmc_reg->GReg().bits[0].right_claw_up_check == 0){//��צλ�ϵ�λ
			return;
		}
#endif
		PmcCmdFrame cmd;
		memset(&cmd, 0x00, sizeof(PmcCmdFrame));

		cmd.data.axis_index = phy_axis+1;   //��Ŵ�1��ʼ
		cmd.data.axis_index |= 0xFF00;      //��־ͨ������
		cmd.data.cmd = 0;   //����λ��

		//�����ٶ�
		uint32_t feed = vel*1000/60;   //ת����λΪum/s


		if((this->m_n_mask_ret_ref_over & (0x01<<phy_axis))
				&& m_p_axis_config[phy_axis].soft_limit_check_1 == 1){//����λ1��Ч
			int64_t cur_pos = this->m_df_phy_axis_pos_feedback[phy_axis]*1e7;  //��ǰλ��
			int64_t limit_pos = 0;

			if(dir == DIR_POSITIVE){//����
				limit_pos = m_p_axis_config[phy_axis].soft_limit_max_1*1e7;

				if(limit_pos < cur_pos){//�Ѿ�����λ�⣬�澯
					CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis);
				//	this->m_error_code = ERR_SOFTLIMIT_POS;
					return;
				}else if(limit_pos == cur_pos){  //�Ѿ���������λ����λ��
					CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis);
					return;
				}else if(tar_pos > limit_pos){
					tar_pos = limit_pos;

				}
			}else if(dir == DIR_NEGATIVE){//����
				limit_pos = m_p_axis_config[phy_axis].soft_limit_min_1*1e7;

				if(limit_pos > cur_pos){//�Ѿ�����λ�⣬�澯
					CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON, 0, CHANNEL_ENGINE_INDEX, phy_axis);
				//	this->m_error_code = ERR_SOFTLIMIT_NEG;
					return;
				}else if(limit_pos == cur_pos){  //�Ѿ����︺��λ����λ��
					CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON, 0, CHANNEL_ENGINE_INDEX, phy_axis);
					return;
				}else if(tar_pos < limit_pos){
					tar_pos = limit_pos;
				}
			}
		}


		memcpy(&cmd.data.data[1], &tar_pos, sizeof(tar_pos));  //����Ŀ��λ��

		memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //�����ٶ�



		this->m_n_run_axis_mask |= 0x01L<<phy_axis;  //���õ�ǰ������

		this->m_p_mi_comm->SendPmcCmd(cmd);

		printf("ChannelEngine::ManualMove_pmc: axis = %d, tar_pos = %lld\n", phy_axis, tar_pos);

	}
}

/**
 * @brief �ֶ��ƶ�����dir�����ƶ�dis����
 * @param chn : ͨ���ţ� ��0��ʼ
 * @param phy_axis : ������ţ���0��ʼ
 * @param dir : �˶�����
 * @param vel : �˶��ٶ�, ��λ��mm/min
 * @param inc_dis �� ����λ��, �����������ں����ڲ�����dir��������
 */
void ChannelEngine::ManualMove(uint8_t phy_axis, int8_t dir, double vel, double inc_dis){
	//���Ӳ��λ
	if(CheckAxisHardLimit(phy_axis, dir)){   //Ӳ��λ�澯��ֱ�ӷ���
		printf("hard limit active, manual move return 3 \n");
		return;
	}


	uint8_t chn_axis = 0, chn = 0;
	chn = this->GetAxisChannel(phy_axis, chn_axis);

	if(m_p_axis_config[phy_axis].axis_pmc == 0 && chn != CHANNEL_ENGINE_INDEX){ //ͨ����
		McCmdFrame cmd;
		memset(&cmd, 0x00, sizeof(McCmdFrame));

		cmd.data.channel_index = chn;
		cmd.data.axis_index = chn_axis+1;   //��Ŵ�1��ʼ
		cmd.data.cmd = CMD_MC_AXIS_MAUAL_MOVE;

		//�����ٶ�
		uint32_t feed = vel*1000/60;   //ת����λΪum/s

	//	memcpy(cmd.data.data, &feed, sizeof(feed));
		cmd.data.data[0] = (feed & 0xFFFF);
		cmd.data.data[1] = ((feed>>16)&0xFFFF);

		//����Ŀ��λ��
		int64_t tar_pos = fabs(inc_dis) * 1e7 *dir;   //��λת����mm-->0.1nms

		if((this->m_n_mask_ret_ref_over & (0x01<<phy_axis))
				&& m_p_axis_config[phy_axis].soft_limit_check_1 == 1){//����λ1��Ч
			int64_t cur_pos = this->m_df_phy_axis_pos_feedback[phy_axis]*1e7;  //��ǰλ��
			int64_t limit_pos = 0;
		//	int8_t dir = (tar_pos > cur_pos)?DIR_POSITIVE:DIR_NEGATIVE;
			if(dir == DIR_POSITIVE){//����
				limit_pos = m_p_axis_config[phy_axis].soft_limit_max_1*1e7;


				if(limit_pos < cur_pos){//�Ѿ�����λ�⣬�澯
					CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, chn, chn_axis);
				//	this->m_error_code = ERR_SOFTLIMIT_POS;
					return;
				}else if(limit_pos == cur_pos){  //�Ѿ���������λ����λ��
					CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, chn, chn_axis);
					return;
				}else if(tar_pos > (limit_pos-cur_pos)){
					tar_pos = limit_pos-this->m_p_channel_control[chn].GetAxisCurIntpTarPos(chn_axis, true)*1e7;

				}
			}else if(dir == DIR_NEGATIVE){//����
				limit_pos = m_p_axis_config[phy_axis].soft_limit_min_1*1e7;

				if(limit_pos > cur_pos){//�Ѿ�����λ�⣬�澯
					CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON, 0, chn, chn_axis);
				//	this->m_error_code = ERR_SOFTLIMIT_NEG;
					return;
				}else if(limit_pos == cur_pos){  //�Ѿ����︺��λ����λ��
					CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON, 0, chn, chn_axis);
					return;
				}else if(tar_pos < (limit_pos-cur_pos)){
					tar_pos = limit_pos-this->m_p_channel_control[chn].GetAxisCurIntpTarPos(chn_axis, true)*1e7;
				}
			}
		}

		cmd.data.data[2] = (tar_pos & 0xFFFF);
		cmd.data.data[3] = ((tar_pos>>16)&0xFFFF);
		cmd.data.data[4] = ((tar_pos>>32) & 0xFFFF);
		cmd.data.data[5] = ((tar_pos>>48)&0xFFFF);

		cmd.data.data[6] = 0x02;   //����Ŀ��λ��

		if(!this->m_mc_run_on_arm[chn])
			m_p_mc_comm->WriteCmd(cmd);
		else
			m_p_mc_arm_comm->WriteCmd(cmd);
		
	//	printf("ChannelEngine::ManualMove: axis = %d, tar_pos = %lld\n", phy_axis, tar_pos);
	}else if(m_p_axis_config[phy_axis].axis_pmc){  //PMC��
		PmcCmdFrame cmd;
		memset(&cmd, 0x00, sizeof(PmcCmdFrame));

		cmd.data.axis_index = phy_axis+1;   //��Ŵ�1��ʼ
		cmd.data.axis_index |= 0xFF00;      //��־ͨ������
		cmd.data.cmd = 0x100;   //����λ��

		//�����ٶ�
		uint32_t feed = vel*1000/60;   //ת����λΪum/s

		//����Ŀ��λ��
		int64_t tar_pos = fabs(inc_dis)*1e7*dir;    //ת����λΪ0.1nm

		if((this->m_n_mask_ret_ref_over & (0x01<<phy_axis))
				&& m_p_axis_config[phy_axis].soft_limit_check_1 == 1){//����λ1��Ч
			int64_t cur_pos = this->m_df_phy_axis_pos_feedback[phy_axis]*1e7;  //��ǰλ��
			int64_t limit_pos = 0;
			int8_t dir = (tar_pos > cur_pos)?DIR_POSITIVE:DIR_NEGATIVE;
			if(dir == DIR_POSITIVE){//����
				limit_pos = m_p_axis_config[phy_axis].soft_limit_max_1*1e7;

				if(limit_pos < cur_pos){//�Ѿ�����λ�⣬�澯
					CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis);
				//	this->m_error_code = ERR_SOFTLIMIT_POS;
					return;
				}else if(limit_pos == cur_pos){  //�Ѿ���������λ����λ��
					CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis);
					return;
				}else if(tar_pos > (limit_pos-cur_pos)){
					tar_pos = limit_pos-cur_pos;

				}
			}else if(dir == DIR_NEGATIVE){//����
				limit_pos = m_p_axis_config[phy_axis].soft_limit_min_1*1e7;

				if(limit_pos > cur_pos){//�Ѿ�����λ�⣬�澯
					CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON, 0, CHANNEL_ENGINE_INDEX, phy_axis);
				//	this->m_error_code = ERR_SOFTLIMIT_NEG;
					return;
				}else if(limit_pos == cur_pos){  //�Ѿ����︺��λ����λ��
					CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON, 0, CHANNEL_ENGINE_INDEX, phy_axis);
					return;
				}else if(tar_pos < (limit_pos-cur_pos)){
					tar_pos = limit_pos-cur_pos;
				}
			}
		}



		memcpy(&cmd.data.data[1], &tar_pos, sizeof(tar_pos));  //����Ŀ��λ��

		memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //�����ٶ�



		this->m_n_run_axis_mask |= 0x01L<<phy_axis;  //���õ�ǰ������

		this->m_p_mi_comm->SendPmcCmd(cmd);

//		printf("ChannelEngine::ManualMove_pmc: axis = %d, tar_pos = %lld\n", phy_axis, tar_pos);

	}
}

/**
 * @brief �ֶ�ֹͣ
 * @param ��
 */
void ChannelEngine::ManualMoveStop(){
	//ֹͣͨ����
	m_p_channel_control[m_n_cur_channle_index].ManualMoveStop();

	//ֹͣPMC���ƶ�
	this->ManualMovePmcStop();


//	if(this->m_n_run_axis_mask){
//		MiCmdFrame cmd;
//		memset(&cmd, 0x00, sizeof(MiCmdFrame));
//
//		cmd.data.axis_index = 0xFF;
//		cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
//		cmd.data.data[0] = 0x20;   //ֹͣ��ǰ���˶�����������ǰ�˶�ָ��
//
//		m_p_mi_comm->WriteCmd(cmd);
//
//		this->m_n_run_axis_mask = 0;
//		this->m_n_runover_axis_mask = 0;
//		printf("send manual pmc stop : runmask = 0x%x\n", this->m_n_run_axis_mask);
//	}

}

/**
 * @brief ָ�����ֶ��ƶ�
 */
void ChannelEngine::ManualMovePmc(uint8_t phy_axis, int8_t dir){
	//���Ӳ��λ
	if(CheckAxisHardLimit(phy_axis, dir)){   //Ӳ��λ�澯��ֱ�ӷ���
		printf("hard limit active, manual move pmc return \n");
		return;
	}

//	m_channel_status.cur_manual_move_dir = dir;   //���淽��

	PmcCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(PmcCmdFrame));

	cmd.data.axis_index = phy_axis+1;   //��Ŵ�1��ʼ
	cmd.data.axis_index |= 0xFF00;      //��־ͨ������
	cmd.data.cmd = 0x100;   //����λ��

	//�����ٶ�
	uint32_t feed = this->m_p_axis_config[phy_axis].manual_speed*1000/60;   //ת����λΪum/s
	if(this->m_p_channel_control[0].IsRapidManualMove()){
		feed *= 2;  //�ٶȷ���
		printf("double manual feed\n");
	}


	//����Ŀ��λ��
	int64_t tar_pos = 0;

	if(this->m_p_channel_control[0].GetChnWorkMode() == MANUAL_STEP_MODE){ //�ֶ�����
		tar_pos = this->m_p_channel_control[0].GetCurManualStep()*1e4*dir;		//ת����λΪ0.1nm

	}else{
		tar_pos = 99999*1e7*dir;    //�ֶ�����ģʽ����Ŀ��λ�����õĺ�Զ
	}

	if((this->m_n_mask_ret_ref_over & (0x01<<phy_axis))
			&& this->m_p_axis_config[phy_axis].soft_limit_check_1 == 1){//����λ1��Ч
		int64_t tar_inc_max = 0;
		int64_t cur_pos = this->GetPhyAxisMachPosFeedback(phy_axis)*1e7;  //��ǰλ��
		if(dir == DIR_POSITIVE){//����
			tar_inc_max = m_p_axis_config[phy_axis].soft_limit_max_1*1e7 - cur_pos;

			if(tar_inc_max < 0){//�Ѿ�����λ�⣬�澯
				CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis);
				this->m_error_code = ERR_SOFTLIMIT_POS;
				return;
			}else if(tar_inc_max == 0){  //�Ѿ���������λ����λ��
				CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis);
				this->m_error_code = ERR_SOFTLIMIT_POS;
				return;
			}else if(tar_pos > tar_inc_max){
				tar_pos = tar_inc_max;
			}
		}else if(dir == DIR_NEGATIVE){//����
			tar_inc_max = m_p_axis_config[phy_axis].soft_limit_min_1*1e7 - cur_pos;

			if(tar_inc_max > 0){//�Ѿ�����λ�⣬�澯
				CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON, 0, CHANNEL_ENGINE_INDEX, phy_axis);
		//		this->m_error_code = ERR_SOFTLIMIT_NEG;
				printf("manual move out of soft negative limit\n");
				return;
			}else if(tar_inc_max == 0){  //�Ѿ����︺��λ����λ��
				CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, phy_axis);
		//		this->m_error_code = ERR_SOFTLIMIT_NEG;
				return;
			}else if(tar_pos < tar_inc_max){
				tar_pos = tar_inc_max;
			}
		}
	}

	memcpy(&cmd.data.data[1], &tar_pos, sizeof(tar_pos));  //��������Ŀ��λ��

	memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //�����ٶ�



	this->m_n_run_axis_mask = 0x01L<<phy_axis;  //���õ�ǰ������

	this->m_p_mi_comm->SendPmcCmd(cmd);

	printf("manual move pmc: axis = %d, tar_pos = %lld\n", phy_axis, tar_pos);
}

/**
 * @brief �ֶ��ƶ�
 */
void ChannelEngine::ManualMovePmc(int8_t dir){

	this->ManualMovePmc(this->m_n_cur_pmc_axis, dir);
}

/**
 * @brief ָ������ָ���ٶ��ƶ���ָ��λ��
 * param phy_axis : ָ���������ᣬ ��0��ʼ
 * param tar_pos : Ŀ��λ�ã���λ��mm
 * param vel : Ŀ���ٶȣ� ��λ��mm/min
 * param inc : true--����ʽ    false--����ʽ
 *
 */
void ChannelEngine::ManualMovePmc(uint8_t phy_axis, double tar_pos, double vel, bool inc){
	uint8_t dir = DIR_POSITIVE;  //Ĭ������

	double cur_pos = this->GetPhyAxisMachPosFeedback(phy_axis);
	if((!inc && tar_pos - cur_pos < 0) ||
			(inc && tar_pos < 0))
		dir = DIR_NEGATIVE;


	//���Ӳ��λ
	if(CheckAxisHardLimit(phy_axis, dir)){   //Ӳ��λ�澯��ֱ�ӷ���
		printf("hard limit active, manual move pmc return \n");
		return;
	}

	PmcCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(PmcCmdFrame));

	cmd.data.axis_index = phy_axis+1;   //��Ŵ�1��ʼ
	cmd.data.axis_index |= 0xFF00;      //��־ͨ������
	cmd.data.cmd = inc?0x100:0;   //����λ��/����λ��

	//�����ٶ�
	uint32_t feed = vel*1000/60;   //ת����λΪum/s

	//����Ŀ��λ��
	int64_t target = tar_pos*1e7;    //ת����λΪ0.1nm


	if((this->m_n_mask_ret_ref_over & (0x01<<phy_axis))
			&& this->m_p_axis_config[phy_axis].soft_limit_check_1 == 1){//����λ1��Ч
		int64_t tar_inc_max = 0;

		if(dir == DIR_POSITIVE){//����
			tar_inc_max = m_p_axis_config[phy_axis].soft_limit_max_1*1e7 - cur_pos*1e7;

			if(tar_inc_max <= 0){//�Ѿ�����λ�⣬�澯
				CreateError(ERR_SOFTLIMIT_POS, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON, 0, CHANNEL_ENGINE_INDEX, phy_axis);
	//			this->m_error_code = ERR_SOFTLIMIT_POS;

				return;
			}else if(inc && target > tar_inc_max){
				target = tar_inc_max;
			}else if(!inc && target > m_p_axis_config[phy_axis].soft_limit_max_1*1e7){
				target = m_p_axis_config[phy_axis].soft_limit_max_1*1e7;
			}
		}else if(dir == DIR_NEGATIVE){//����
			tar_inc_max = m_p_axis_config[phy_axis].soft_limit_min_1*1e7 - cur_pos*1e7;

			if(tar_inc_max > 0){//�Ѿ�����λ�⣬�澯
				CreateError(ERR_SOFTLIMIT_NEG, WARNING_LEVEL, CLEAR_BY_CLEAR_BUTTON, 0, CHANNEL_ENGINE_INDEX, phy_axis);
		//		this->m_error_code = ERR_SOFTLIMIT_NEG;

				return;
			}else if(inc && target < tar_inc_max){
				target = tar_inc_max;
			}else if(!inc && target < m_p_axis_config[phy_axis].soft_limit_min_1*1e7){
				target = m_p_axis_config[phy_axis].soft_limit_min_1*1e7;
			}
		}
	}

	memcpy(&cmd.data.data[1], &target, sizeof(target));  //����Ŀ��λ��

	memcpy(&cmd.data.data[5], &feed, sizeof(feed)); //�����ٶ�



	this->m_n_run_axis_mask = 0x01L<<phy_axis;  //���õ�ǰ������

	this->m_p_mi_comm->SendPmcCmd(cmd);

	printf("manual move pmc vel: axis = %d, tar_pos = %lf, vel = %lf\n", phy_axis, tar_pos, vel);
}

/**
 * @brief �ֶ�ֹͣ
 */
void ChannelEngine::ManualMovePmcStop(){
	if(this->m_p_channel_control[0].GetChnWorkMode() == MANUAL_STEP_MODE){  //�ֶ�����ģʽ��������Ӧ���˳�
		return;
	}
	printf("send manualmove pmc stop to mi \n");

#ifdef USES_PMC_PROCESS
	//PMC����
	this->m_p_pmc_reg->FReg().bits[0].OUT0 = 0;
	this->m_p_pmc_reg->FReg().bits[0].OUT1 = 0;
	this->m_p_pmc_reg->FReg().bits[0].OUT2 = 0;
	this->m_p_pmc_reg->FReg().bits[0].OUT3 = 0;
	this->m_p_pmc_reg->FReg().bits[0].OUT4 = 0;
	this->m_p_pmc_reg->FReg().bits[0].OUT5 = 0;
	this->m_p_pmc_reg->FReg().bits[0].OUT6 = 0;
	this->m_p_pmc_reg->FReg().bits[0].OUT7 = 0;
#else
	//ϵͳ����
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(MiCmdFrame));

	cmd.data.axis_index = NO_AXIS;
	cmd.data.reserved = CHANNEL_ENGINE_INDEX;
	cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
	cmd.data.data[0] = 0x20;   //ֹͣ��ǰ���˶�����������ǰ�˶�ָ��

	m_p_mi_comm->WriteCmd(cmd);

	this->m_n_run_axis_mask = 0;
	this->m_n_runover_axis_mask = 0;
#endif
}

/**
 * @brief ��ͣPMC���ƶ�
 * @param phy_axis : �������, ��0��ʼ
 * @param flag : true--��ͣ     false--ȡ����ͣ
 */
void ChannelEngine::PausePmcAxis(uint8_t phy_axis, bool flag){
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(MiCmdFrame));

	uint16_t data = 0x01;
	if(!flag)
		data = 0x10;   //ȡ����ͣ
	cmd.data.axis_index = phy_axis+1;
	cmd.data.reserved = CHANNEL_ENGINE_INDEX;
	cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
	cmd.data.data[0] = data;   //��ͣ��ǰ���˶�

	m_p_mi_comm->WriteCmd(cmd);

	printf("ChannelEngine::PausePmcAxis, phy_axis=%hhu, flag = %hhu\n", phy_axis, flag);
}

/**
 * @brief �ֶ�ָֹͣ����
 * @param phy_axis �� ������ã� ��0��ʼ
 */
void ChannelEngine::ManualMoveStop(uint8_t phy_axis){

	uint8_t chn = 0, chn_axis = 0;
	chn = this->GetAxisChannel(phy_axis, chn_axis);
	if(this->m_p_axis_config[phy_axis].axis_pmc == 0 && chn != CHANNEL_ENGINE_INDEX){
		this->m_p_channel_control[chn].ManualMoveStop(0x01<<chn_axis);
	}else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC��
		MiCmdFrame cmd;
		memset(&cmd, 0x00, sizeof(MiCmdFrame));

		cmd.data.axis_index = phy_axis+1;
		cmd.data.reserved = CHANNEL_ENGINE_INDEX;
		cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
		cmd.data.data[0] = 0x30;   //ָֹͣ�����˶�����������ǰ�˶�ָ��

		m_p_mi_comm->WriteCmd(cmd);

		this->m_n_run_axis_mask &= ~(0x01L<<phy_axis);
		if(this->m_n_run_axis_mask == this->m_n_runover_axis_mask){
			m_n_run_axis_mask = 0;
			m_n_runover_axis_mask = 0;
		}
	}
}

/**
 * @brief PMC�����е�λ
 * @param cmd : MI���͹���������֡
 */
void ChannelEngine::PmcAxisRunOver(MiCmdFrame &cmd){
	printf("PmcAxisRunOver: axis = %hhu\n",cmd.data.axis_index);
	uint64_t mask = 0;
	uint8_t chn = cmd.data.reserved-1;

	if((cmd.data.data[0]&0xFF) == 0x01){//G01ָ��

		memcpy(&mask, &cmd.data.data[1], 4);
		printf("PmcAxisRunOver1: axis_mask = 0x%llx\n",mask);
		if((this->m_n_run_axis_mask & mask) == 0){  //����ͨ��������Ƶ�PMC��
			if(chn < this->m_p_general_config->chn_count){
				if(this->m_p_channel_control[chn].PmcAxisRunOver(cmd))
					return;
			}
			return;
		}else if(this->m_n_run_axis_mask == 0)
			return;
	}else{ //����ָ��  G00
		uint8_t phy_axis = cmd.data.axis_index-1;  //������ţ���0��ʼ
		mask = 0x01L<<phy_axis;

		printf("PmcAxisRunOver2: axis_mask = 0x%llx\n",mask);
		if((this->m_n_run_axis_mask & mask) == 0){  //����ͨ��������Ƶ�PMC��
			if(chn < this->m_p_general_config->chn_count){
				if(this->m_p_channel_control[chn].PmcAxisRunOver(cmd))
					return;
			}
			return;
		}else if(this->m_n_run_axis_mask == 0)
			return;

		if(this->m_p_axis_config[phy_axis].axis_pmc > 0){
			if(m_pmc_axis_ctrl[m_p_axis_config[phy_axis].axis_pmc-1].GetCmdCount() > 0 &&
					!m_pmc_axis_ctrl[m_p_axis_config[phy_axis].axis_pmc-1].IsPaused()){   //�����ݣ�����ͣ״̬
				m_pmc_axis_ctrl[m_p_axis_config[phy_axis].axis_pmc-1].ExecCmdOver(true);
			}
		}

		this->m_n_runover_axis_mask |= mask;

		if(this->m_n_run_axis_mask == this->m_n_runover_axis_mask){  //���н���
			this->m_n_run_axis_mask = 0;
			this->m_n_runover_axis_mask = 0;

			printf("pmc axis run over\n");
		}
	}

}



/**
 * @brief �������
 * @param dir : ������ת����
 */
void ChannelEngine::SpindleOut(int dir){
	//printf("channelengine::spindleout : %d, cur_chn=%hhu\n", dir, m_n_cur_channle_index);
	g_ptr_trace->PrintTrace(TRACE_INFO, CHANNEL_ENGINE_SC, "channelengine::spindleout : %d, cur_chn=%hhu\n", dir, m_n_cur_channle_index);																																  
	m_p_channel_control[m_n_cur_channle_index].SpindleOut(dir);
}

/**
 * @brief �����������λ�澯���
 * @param phy_axis : ������ţ���0��ʼ
 * @param dir �� ��λ����, -1��ʾ���� 1��ʾ����
 * @return true--������λ   false--û�д���
 */
bool ChannelEngine::CheckAxisHardLimit(uint8_t phy_axis, int8_t dir){
    printf("cur phy axis: %hhu, dir = %hhu, post_mask = 0x%llx, neg_mask = 0x%llx\n", phy_axis, dir, this->m_hard_limit_postive, m_hard_limit_negative);
	if(this->m_b_ret_ref || this->m_b_ret_ref_auto)   //�زο���ʱ����Ӳ��λ
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
 * @brief ����ָ���������λ����
 * @param axis : ָ����ţ���0��ʼ
 */
 /*
void ChannelEngine::SetAxisSoftLimit(uint8_t axis){
	//����MC������λ����
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
 * @brief ����ָ���������λֵ
 * @param axis : ָ����ţ���0��ʼ
 * @param index : ����λ��ţ��ܹ�3�飬0-2
 */
 /*
void ChannelEngine::SetAxisSoftLimitValue(uint8_t axis, uint8_t index){
	//����MC������λ����
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


	int32_t softlimit = max*1e3;   //��λ��mmת��Ϊ1um
	cmd.data.data[1] = (softlimit & 0xFFFF);
	cmd.data.data[2] = ((softlimit>>16) & 0xFFFF);

	softlimit = min*1e3;   //��λ��mmת��Ϊ1um
	cmd.data.data[3] = (softlimit & 0xFFFF);
	cmd.data.data[4] = ((softlimit>>16) & 0xFFFF);

	m_p_mc_comm->WriteCmd(cmd);
}*/

/**
 * @brief ��ʼģ����������
 */
void ChannelEngine::StartUpdateProcess(){
	//���������߳�
	int res = 0;
	pthread_attr_t attr;
	struct sched_param param;
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, kThreadStackSize);	//
	param.__sched_priority = 30; //90;
	pthread_attr_setschedparam(&attr, &param);
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
//	if (res) {
//		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ģ�����������߳������̼̳߳�ģʽʧ�ܣ�");
//		m_error_code = ERR_INIT_UPDATE;
//		goto END;
//	}
	res = pthread_create(&m_thread_update, &attr,
			ChannelEngine::UpdateThread, this);    //����ģ�����������߳�
	if (res != 0) {
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ģ�����������̴߳���ʧ��!");
		m_error_code = ERR_INIT_UPDATE;
		CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_RESET_POWER);
		goto END;
	}

END:
	pthread_attr_destroy(&attr);
}

/**
 * @brief ģ������ִ���̺߳���
 * @param void *args: ChannelEngine����ָ��
 */
void *ChannelEngine::UpdateThread(void *args){
	printf("start module update thread, threadid = %ld!\n", syscall(SYS_gettid));
	ChannelEngine *p_chn_engine = static_cast<ChannelEngine *>(args);

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
	if (res != ERR_NONE) {
		printf("Quit ChannelEngine::UpdateThread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
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

	//һ������
	res = this->UpdateDisk();
	if(res != ERR_NONE)
		goto END;
	
	//����SC
	res = this->UpdateSC();
	if(res != ERR_NONE)
		goto END;


	//����MC
	res = this->UpdateMC();
	if(res != ERR_NONE)
		goto END;

	//����MI
//	res = this->UpdateMI();
	res = this->UpdateMI_2();
	if(res != ERR_NONE)
		goto END;

	//����PMC
	res = this->UpdatePMC();
	if(res != ERR_NONE)
		goto END;

	//����PL
	res = this->UpdatePL();
	if(res != ERR_NONE)
		goto END;

	//����SPARTAN
	res = this->UpdateSpartan();
	if(res != ERR_NONE)
		goto END;

	//����Modbusģ��
	res =this->UpdateModbus();
	if(res != ERR_NONE)
		goto END;

END:
	this->SendHmiUpdateStatus(0, 0);//ȫ��ģ���������

	m_thread_update = 0;
	if(res != ERR_NONE){
		this->m_error_code = static_cast<ErrorType>(res);
		CreateError(res, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
	}
	return res;
}

/**
 * @brief ����MCģ��
 * @return
 */
int ChannelEngine::UpdateMC(){
	//����MCģ��
	int res = ERR_NONE;
//	int retry = 0;
	uint32_t i = 0;

	struct stat statbuf;
	uint32_t file_size = 0;
	uint32_t send_size = 0, read_size = 0;
	uint16_t file_crc = 0xffff;  //�ļ�crc
	uint16_t file_frame_count = 0;  //��֡��
	GCodeFrame data_frame;     //����֡
	int fp = 0; //�ļ����
	uint32_t cc = 0;

	//��������ļ�����
	char filepath[kMaxPathLen] = {0};
	strcpy(filepath, PATH_UPDATE_PATH);
	strcat(filepath, "module_mc.ldr");

	if(access(filepath, F_OK) == -1)	//�����ļ������ڣ�MCģ�鲻��Ҫ����
		return res;

	printf("start to update MC module\n");
	m_n_update_state = MODULE_UPDATE_MC;
	m_n_mc_update_status = 0;  //��ʼ״̬Ϊ�޴���

	this->SendHmiUpdateStatus(5, 0);

	if(!this->SendMcUpdateStartCmd()){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "����MCģ��������ʼ����ʧ�ܣ�");
		res = ERR_UPDATE_MC;
		goto END;
	}

	printf("Succeed to send mc update start cmd\n");

	//TODO ��������˿�����FIFO�еĵ�ǰ����

	//�ȴ�MC����������flash
	printf("wait mc to erase flash!\n");
	do{
		if(!QueryMcUpdateStatus()){
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "��ѯMCģ������״̬ʧ�ܣ�");
			res = ERR_UPDATE_MC;
			goto END;
		}
		while(!m_b_get_mc_update_status)
			usleep(100); //�ȴ���ѯ���

		if(m_n_mc_update_status == 4)
			break;

		usleep(100000);  //�ȴ�100ms

	}while(1);

	//�����ļ���С
	printf("send file size!\n");
	this->SendHmiUpdateStatus(5, 1);
	if(stat(filepath, &statbuf) == 0)
		file_size = statbuf.st_size;  //��ȡ�ļ��ܴ�С
	else{
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "��ȡ�ļ�[%s]��Сʧ�ܣ�", PATH_PMC_DATA);
		res = ERR_UPDATE_MC;
		goto END;		//��ȡ�ļ���Сʧ��
	}
	if(!this->SendMcUpdateFileSize(file_size)){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "���������ļ���Сʧ�ܣ�");
		res = ERR_UPDATE_MC;
		goto END;
	}

	//�����ļ�����
	printf("send file content!\n");
	this->SendHmiUpdateStatus(5, 2);
	fp = open(filepath, O_RDONLY); //ֻ�����ļ�
	if(fp < 0){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "���ļ�[%s]ʧ�ܣ�", filepath);
		res = ERR_UPDATE_MC;
		goto END;//�ļ���ʧ��
	}

	//�����ļ���֡��
	file_frame_count = file_size/MC_UPDATE_BLOCK_SIZE;
	if(file_size%MC_UPDATE_BLOCK_SIZE)
		file_frame_count++;

	while(file_size > 0){
		bzero(data_frame.data_crc, sizeof(data_frame));
		send_size = file_size>MC_UPDATE_BLOCK_SIZE?MC_UPDATE_BLOCK_SIZE:file_size;
		read_size = read(fp, (void *)&data_frame.data_crc[1], send_size);
		if(read_size != send_size){
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "��ȡ�ļ�[%s]ʧ�ܣ�%u, %u��", filepath, send_size, read_size);
			res = ERR_UPDATE_MC;
			goto END;//�ļ���ȡʧ��
		}

		cc = send_size/2;
		if(send_size%2)
			cc++;
		for(i = 0; i < cc; i++){
			file_crc ^= data_frame.data_crc[i+1];  //����CRC
		}

		while(!this->m_p_mc_comm->WriteGCodeData(0, data_frame)){  //�̶���ͨ��0����ͨ������MC
			usleep(100);
		}
		file_size -= send_size;

	}

	//�ȴ����ݴ������
	printf("wait data receive!\n");
	while(this->m_p_mc_comm->ReadGCodeFifoCount(0) > 0)
		usleep(10000);

	printf("send crc, block = %hu, crc = 0x%hx\n", file_frame_count, file_crc);
	this->SendHmiUpdateStatus(5, 3);
	this->SendMcUpdateFileCrc(file_frame_count, file_crc);

	//��ѯMC״̬���ȴ�MC�����������
	printf("wait update result!\n");
	do{
		if(!QueryMcUpdateStatus()){
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "��ѯMCģ������״̬ʧ�ܣ�");
			res = ERR_UPDATE_MC;
			goto END;
		}
		while(!m_b_get_mc_update_status)
			usleep(100); //�ȴ���ѯ���

		if(m_n_mc_update_status == 0x02){ //����
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MC����ʧ�ܣ�");
			res = ERR_UPDATE_MC;
			goto END;
		}
		else if(m_n_mc_update_status == 0x13){ //CRCУ���
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MC����CRCУ��ʧ�ܣ�");
			res = ERR_UPDATE_MC;
			goto END;
		}
		else if(m_n_mc_update_status == 0x11){//�����ɹ�
			this->SendHmiUpdateStatus(5, 4);
			printf("Succeed to update the MC module!\n");
			break;
		}else
			usleep(100000);  //�ȴ�100ms

	}while(1);




END:
	if(fp > 0)
		close(fp);
	if(-1 == remove(filepath)){
		//ɾ�������ļ�ʧ��
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MCģ��ɾ�������ļ�ʧ�ܣ�");
	}

	if(res != ERR_NONE)
		this->SendHmiUpdateStatus(4, 0xFF);

	m_n_update_state = 0;
	m_n_update_state = MODULE_UPDATE_NONE;
	return res;
}

/**
 * @brief ����MIģ��
 * @return
 */
int ChannelEngine::UpdateMI(){
	//����MIģ��
	int res = ERR_NONE;

	char cmd_buf[256];

	//��������ļ�����
	char filepath[kMaxPathLen] = {0};
	strcpy(filepath, PATH_UPDATE_PATH);
	strcat(filepath, "module_mi.bin");

	if(access(filepath, F_OK) == -1)	//�����ļ������ڣ�MIģ�鲻��Ҫ����
		return res;

	printf("start to update mi module\n");
	m_n_update_state = MODULE_UPDATE_MI;

	this->SendHmiUpdateStatus(3, 0);

	printf("WAIT ERASE MI FLASH...\n");

	int ret = system("flash_erase /dev/mtd0 0 1");

	if(ret == -1){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MI��������Flashʧ�ܣ�errno = %d", errno);
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
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MI����д������ʧ�ܣ�errno = %d", errno);
		res = ERR_UPDATE_MI;
	}
	else{
		sync();

		this->SendHmiUpdateStatus(3, 2);
		printf("SUCCEED TO UPDATE MI MODULE\n");


	}

END:
	if(-1 == remove(filepath)){
		//ɾ�������ļ�ʧ��
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MIģ��ɾ�������ļ�ʧ�ܣ�");
	}
	if(res != ERR_NONE)
		this->SendHmiUpdateStatus(3, 0xFF);

	m_n_update_state = MODULE_UPDATE_NONE;
	return res;
}

/**
 * @brief ����MIģ�飬MI�ļ���boot�ֿ�����
 * @return
 */
int ChannelEngine::UpdateMI_2(){
	//����MIģ��
	int res = ERR_NONE;

	//��������ļ�����
	char filepath[kMaxPathLen] = {0};
	char filedes[kMaxPathLen] = {0};
	strcpy(filepath, PATH_UPDATE_PATH);
	strcat(filepath, "module_mi.bin");

	if(access(filepath, F_OK) == -1)	//�����ļ������ڣ�MIģ�鲻��Ҫ����
		return res;

	printf("start to update mi module\n");
	m_n_update_state = MODULE_UPDATE_MI;

	this->SendHmiUpdateStatus(3, 0);

	printf("wait copy file for mi module...\n");


	//����MI�����ļ�
	strcpy(filedes, PATH_MI_PROGRAM);

	int nn = CopyFile(filepath, filedes);
	if(0 != nn){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MIģ�鿽���ļ�ʧ�ܣ�errno = %d", nn);
		res = ERR_UPDATE_MI;
		goto END;
	}

	//���������ļ�
	this->SendHmiUpdateStatus(3, 1);
	bzero(filedes, kMaxPathLen);
	strcpy(filedes, PATH_MI_PROGRAM_BAK);
	nn = CopyFile(filepath, filedes);
	if(0 != nn){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MIģ�鿽�������ļ�ʧ�ܣ�errno = %d", nn);
		res = ERR_UPDATE_MI;
		goto END;
	}


	sync();
	this->SendHmiUpdateStatus(3, 2);
	printf("SUCCEED TO UPDATE MI MODULE\n");


END:
	if(-1 == remove(filepath)){
		//ɾ�������ļ�ʧ��
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MIģ��ɾ�������ļ�ʧ�ܣ�");
	}
	if(res != ERR_NONE)
		this->SendHmiUpdateStatus(3, 0xFF);

	m_n_update_state = MODULE_UPDATE_NONE;
	return res;
}

/**
 * @brief ����Modbusģ��
 * @return
 */
int ChannelEngine::UpdateModbus(){
	//����Modbusģ��
	int res = ERR_NONE;

	//��������ļ�����
	char filepath[kMaxPathLen] = {0};
	char filedes[kMaxPathLen] = {0};
	strcpy(filepath, PATH_UPDATE_PATH);
	strcat(filepath, "module_modbus.elf");

	if(access(filepath, F_OK) == -1)	//�����ļ������ڣ�Modbusģ�鲻��Ҫ����
		return res;

	printf("start to update Modbus module\n");
	m_n_update_state = MODULE_UPDATE_MODBUS;

	this->SendHmiUpdateStatus(3, 0);

	printf("wait copy file for Modbus module...\n");


	//����Modbus�����ļ�
	strcpy(filedes, PATH_MODBUS_PROGRAM);

	int nn = CopyFile(filepath, filedes);
	if(0 != nn){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "Modbusģ�鿽���ļ�ʧ�ܣ�errno = %d", nn);
		res = ERR_UPDATE_MI;
		goto END;
	}


	sync();
	this->SendHmiUpdateStatus(3, 2);
	printf("SUCCEED TO UPDATE MODBUS MODULE\n");


END:
	if(-1 == remove(filepath)){
		//ɾ�������ļ�ʧ��
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MODBUSģ��ɾ�������ļ�ʧ�ܣ�");
	}
	if(res != ERR_NONE)
		this->SendHmiUpdateStatus(3, 0xFF);

	m_n_update_state = MODULE_UPDATE_NONE;
	return res;
}

/**
 * @brief һ������
 * @return
 */
int ChannelEngine::UpdateDisk(){
	//��������ģ��
	int res = ERR_NONE;
	int ret;
	char cmd_buf[100];

	//��������ļ�����
	char filepath[kMaxPathLen] = {0};
	char filedes[kMaxPathLen] = {0};
	strcpy(filepath, PATH_UPDATE_PATH);
	strcat(filepath, "module_disk.zip");

	if(access(filepath, F_OK) == -1)	//�����ļ������ڣ�Modbusģ�鲻��Ҫ����
		return res;

	printf("start to update Disk module\n");
	m_n_update_state = MODULE_UPDATE_DISK;

	this->SendHmiUpdateStatus(3, 0);

	printf("wait copy file for Disk module...\n");

	//����diskupĿ¼
	ret = system("mkdir -p /cnc/diskup");
	if (ret != 0) {
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "DISKģ�鴴��diskupĿ¼ʧ�ܣ�");
		printf("mkdir diskup error, return %d\n", ret);
		res = ERR_UPDATE_DISK;
		goto END;
	}

	//���ԭ�����ļ�
	system("rm -rf /cnc/diskup/sc/*");
	system("rm -rf /cnc/diskup/scroot/*");

	//��ѹ��diskupĿ¼
	sprintf(cmd_buf, "unzip %s -o -d /cnc/diskup", filepath);
	ret = system(cmd_buf);
	if (ret != 0) {
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "DISKģ���ѹ������ʧ�ܣ�");
		printf("unzip %s error, return %d\n", ret);
		res = ERR_UPDATE_DISK;
		goto END;
	}

	this->SendHmiUpdateStatus(3, 1);

	//��������ű��Ƿ����
	if(access(PATH_UPDATE_DISK_CMD, F_OK) == -1)	{
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "DISKģ��δ���������ű���");
		printf("can't update disk module, no scup.sh\n");
		res = ERR_UPDATE_DISK;
		goto END;
	}

	sprintf(cmd_buf, "chmod a+x %s", PATH_UPDATE_DISK_CMD);
	system(cmd_buf);

	//���нű�
	ret = system(PATH_UPDATE_DISK_CMD);
	if (ret != 0) {
		printf("scup.sh return low=%d high=%d\n", ret&0xFF, ret>>8);
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "DISKģ�����������ű�ʧ�ܣ�");
		res = ERR_UPDATE_DISK;
		goto END;
	}

	sync();
	this->SendHmiUpdateStatus(3, 2);
	printf("SUCCEED TO UPDATE DISK MODULE\n");

END:
	if(-1 == remove(filepath)){
		//ɾ�������ļ�ʧ��
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "DISKģ��ɾ�������ļ�ʧ�ܣ�");
	}
	if(res != ERR_NONE)
		this->SendHmiUpdateStatus(3, 0xFF);

	sync();
	m_n_update_state = MODULE_UPDATE_NONE;
	return res;
}

/**
 * @brief ����SCģ��
 * @return
 */
int ChannelEngine::UpdateSC(){
	//����SCģ��
	int res = ERR_NONE;
	int ret = 0;
	char cmd_buf[256];

	//��������ļ�����
	char filepath[kMaxPathLen] = {0};
	char filedes[kMaxPathLen] = {0};
	strcpy(filepath, PATH_UPDATE_PATH);
	strcat(filepath, "module_sc.elf");

	if(access(filepath, F_OK) == -1)	//�����ļ������ڣ�SCģ�鲻��Ҫ����
		return res;

	printf("start to update sc module\n");
	m_n_update_state = MODULE_UPDATE_SC;

	this->SendHmiUpdateStatus(4, 0);

	//��ȡ��ǰ�������
	char c;
	FILE* fp = fopen(PATH_BOOT_CONFIG_FILE, "rb");
	if (fp != nullptr) {
		fread(&c, 1, 1, fp);
		fclose(fp);
	}else{
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "SCģ��������ѯ��������ļ�ʧ�ܣ�errno = %d", errno);
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

	//���������ļ�
	bzero(cmd_buf, 256);
	strcpy(cmd_buf, "cp ");
	strcat(cmd_buf, filepath);
	strcat(cmd_buf, " ");
	strcat(cmd_buf, filedes);

	ret = system(cmd_buf);

	if(ret == -1){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "SCģ������д������ʧ�ܣ�errno = %d", errno);
		res = ERR_UPDATE_SC;
		goto END;
	}

	printf("waiting for changing the file attributes...\n");
	this->SendHmiUpdateStatus(4, 2);
	//�޸��ļ�����
	bzero(cmd_buf, 256);
	strcpy(cmd_buf, "chmod a+x ");
	strcat(cmd_buf, filedes);
	ret = system(cmd_buf);

	if(ret == -1){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "SCģ�������޸��ļ�����ʧ�ܣ�errno = %d", errno);
		res = ERR_UPDATE_SC;
		goto END;
	}
	else{
		sync();
	}

	//�޸�������
	fp = fopen(PATH_BOOT_CONFIG_FILE, "wb");
	if(fp != nullptr) {
		fwrite(&c, 1, 1, fp);
		sync();
		fclose(fp);
		printf("succeed to update sc module\n");

	}else{
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "SCģ��������������ļ�ʧ�ܣ�errno = %d", errno);
		res = ERR_UPDATE_SC;
	}
	this->SendHmiUpdateStatus(4, 3);

END:
	if(-1 == remove(filepath)){
		//ɾ�������ļ�ʧ��
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "MCģ��ɾ�������ļ�ʧ�ܣ�");
	}
	if(res != ERR_NONE)
		this->SendHmiUpdateStatus(4, 0xFF);

	m_n_update_state = MODULE_UPDATE_NONE;
	return res;
}

/**
 * @brief ����PLģ��
 * @return
 */
int ChannelEngine::UpdatePL(){
	//����PLģ��
	int res = ERR_NONE;

	char cmd_buf[256];

	//��������ļ�����
	char filepath[kMaxPathLen] = {0};
	strcpy(filepath, PATH_UPDATE_PATH);
	strcat(filepath, "module_pl.bin");

	if(access(filepath, F_OK) == -1)	//�����ļ������ڣ�PLģ�鲻��Ҫ����
		return res;
	printf("start to update PL module\n");
	m_n_update_state = MODULE_UPDATE_PL;

	this->SendHmiUpdateStatus(3, 0);

	printf("WAIT ERASE BOOT FLASH...\n");

	int ret = system("flash_erase /dev/mtd0 0 1");

	if(ret == -1){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PL��������Flashʧ�ܣ�errno = %d", errno);
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
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PL����д������ʧ�ܣ�errno = %d", errno);
		res = ERR_UPDATE_PL;
	}
	else{
		sync();

		this->SendHmiUpdateStatus(3, 2);
		printf("SUCCEED TO UPDATE PL MODULE\n");


	}

END:
	if(-1 == remove(filepath)){
		//ɾ�������ļ�ʧ��
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PLģ��ɾ�������ļ�ʧ�ܣ�");
	}
	if(res != ERR_NONE)
		this->SendHmiUpdateStatus(3, 0xFF);

	m_n_update_state = MODULE_UPDATE_NONE;
	return res;
}

/**
 * @brief ����SPARTANģ��
 * @return
 */
int ChannelEngine::UpdateSpartan(){
	//����SPARTANģ��
	int res = ERR_NONE;

	char cmd_buf[256];

	//��������ļ�����
	char filepath[kMaxPathLen] = {0};
	strcpy(filepath, PATH_UPDATE_PATH);
	strcat(filepath, "module_spartan.bin");

	if(access(filepath, F_OK) == -1)	//�����ļ������ڣ�MIģ�鲻��Ҫ����
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
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "SPARTAN����д������ʧ�ܣ�errno = %d", errno);
		res = ERR_UPDATE_SPARTAN;
	}
	else{
		sync();

		this->SendHmiUpdateStatus(3, 2);
		printf("SUCCEED TO UPDATE SPARTAN MODULE\n");


	}

	if(-1 == remove(filepath)){
		//ɾ�������ļ�ʧ��
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "SPARTANģ��ɾ�������ļ�ʧ�ܣ�");
	}
	if(res != ERR_NONE)
		this->SendHmiUpdateStatus(3, 0xFF);

	m_n_update_state = MODULE_UPDATE_NONE;
	return res;
}

/**
 * @brief ����PMCģ��
 * @return
 */
int ChannelEngine::UpdatePMC(){
	//����PMCģ��
	int res = ERR_NONE;

	//��������ļ�����
	char filepath[kMaxPathLen] = {0};

	strcpy(filepath, PATH_UPDATE_PATH);
	strcat(filepath, "module_pmc.upd");


	if(access(filepath, F_OK) == -1)	//�����ļ������ڣ�PMCģ�鲻��Ҫ����
		return res;

	printf("start to update PMC module\n");
	m_n_update_state = MODULE_UPDATE_PMC;

	this->SendHmiUpdateStatus(3, 0);  //�ܹ���������

	FILE *file_src = fopen(filepath, "r+");
	if (nullptr == file_src)
	{
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMCģ������ʧ�ܣ������ļ���ʧ��[%s]��errno = %d", filepath, errno);
		return res;
	}

	uint64_t file_len = 0;   //�ļ�����
	uint64_t total_size = 0;	//�����ļ��ܳ���
	fseek(file_src, 0L, SEEK_END);
	total_size = ftell(file_src);   //��ȡ�ļ�����

	fseek(file_src, 0L, SEEK_SET);   //�ص��ļ�ͷ


	//��������ͼ�ļ�
	char buffer[1024] = {0};   //����
	int read_cur_real = 0;		//����ʵ�ʶ�ȡ�ֽ���
	int read_cur_plan = 0;		//���ʼƻ���ȡ�ֽ���
	FILE *file_des = fopen(PATH_PMC_LDR, "w+");
	if (nullptr == file_des)
	{
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMC����ͼ�ļ�����ʧ��[%s]��errno = %d", PATH_PMC_LDR, errno);
		res = ERR_UPDATE_PMC;
		goto END;
	}

	fread(&file_len, 8, 1, file_src);
	file_len = BigLittleSwitch64(file_len);

	if(file_len >= total_size){	//�ļ����Ȳ��Ϸ�
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�����ļ�������[0x%llx, 0x%llx]��", total_size, file_len);
		res = ERR_UPDATE_PMC;
		fclose(file_des);
		goto END;
	}
	while(file_len > 0 && !feof(file_src))
	{
		read_cur_plan = file_len>1024?1024:file_len;
		read_cur_real = fread(buffer, 1, read_cur_plan, file_src);
		if(read_cur_plan != read_cur_real){	//����
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMCģ����������������ͼ�ļ�����1��%llu, %d, %d", file_len, read_cur_plan,
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
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMCģ����������������ͼ�ļ�����2��");
		res = ERR_UPDATE_PMC;
		fclose(file_des);
		goto END;
	}


	fclose(file_des); 	//��ֵ����ͼ�ļ��ɹ�
	sync();
	printf("Succeed to copy ldr file!\n");

	this->SendHmiUpdateStatus(3, 1);  //�ܹ���������

	//����PMCִ���ļ�
	file_des = fopen(PATH_PMC_DATA, "w+");
	if (nullptr == file_des)
	{
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMCִ���ļ�����ʧ��[%s]��errno = %d", PATH_PMC_DATA, errno);
		res = ERR_UPDATE_PMC;
		goto END;
	}
	fread(buffer, 6, 1, file_src);
	if(strcmp(buffer, "#DAT@#") != 0){
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�����ļ��Ƿ������飡");
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
		if(read_cur_plan != read_cur_real){	//����
			g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMCģ������������ִ���ļ�����1��");
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
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMCģ������������ִ���ļ�����2��");
		res = ERR_UPDATE_PMC;
		fclose(file_des);
		goto END;
	}

	fclose(file_des);

	this->SendHmiUpdateStatus(3, 2);  //�ܹ���������

	printf("Succeed to update pmc module!\n");
	END:
	fclose(file_src);
	if(-1 == remove(filepath)){
		//ɾ�������ļ�ʧ��
		g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "PMCģ��ɾ�������ļ�ʧ�ܣ�");
	}
	sync();

	if(res != ERR_NONE)
		this->SendHmiUpdateStatus(3, 0xFF);
	return res;
}

/**
 * @brief ����MC������ʼָ��
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelEngine::SendMcUpdateStartCmd(){
	//����MC������ʼָ��
	McCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(McCmdFrame));

	cmd.data.channel_index = CHANNEL_ENGINE_INDEX;
	cmd.data.axis_index = NO_AXIS;
	cmd.data.cmd = CMD_MC_UPDATE_START;

	return m_p_mc_comm->WriteCmd(cmd);
}

/**
 * @brief ����MC�����ļ���С
 * @param size : �����ļ�16bits˫�ֽ�����
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelEngine::SendMcUpdateFileSize(uint32_t size){
	//����MC�����ļ���С
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
 * @brief ����MC�����ļ�CRC
 * @param frame_count :  ����֡����
 * @param crc : �����ļ�����CRC
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelEngine::SendMcUpdateFileCrc(uint16_t frame_count, uint16_t crc){
	//����MC�����ļ�CRC
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
 * @brief ��ѯMC����״̬
 * @return
 */
bool ChannelEngine::QueryMcUpdateStatus(){
	//��ѯMC����״̬
	McCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(McCmdFrame));

	cmd.data.channel_index = CHANNEL_ENGINE_INDEX;
	cmd.data.axis_index = NO_AXIS;
	cmd.data.cmd = CMD_MC_GET_UPDATE_STATE;

	m_b_get_mc_update_status = false;

	return m_p_mc_comm->WriteCmd(cmd);
}

/**
 * @brief ��ȡʵ������������
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
 * @brief ��ʼ��PMC�ķ���ʧ�ԼĴ���
 */
void ChannelEngine::InitPmcReg(){
	printf("init pmc register\n");
//	//���ļ�
//	int fp = open(PATH_PMC_REG, O_RDONLY); //���ļ�
//
//	if(fp < 0){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��pmc�Ĵ��������ļ�ʧ�ܣ�");
//		return;//�ļ���ʧ��
//	}
//	uint16_t size = 0;   //�Ĵ����ֽ���
//
//	//��ȡK�Ĵ���
//	ssize_t read_size = read(fp, &size, 2);
//	if(read_size != 2){ //��ȡʧ��
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡK�Ĵ����ֽ���ʧ�ܣ�");
//		return;
//	}
//
//	if(size != K_REG_COUNT){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "K�Ĵ�����С����ƥ��[%hu,%hu]��", size, K_REG_COUNT);
//		return;
//	}
//	read_size = read(fp, this->m_p_pmc_reg->GetRegPtr8(PMC_REG_K), size);
//	if(read_size != size){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡK�Ĵ�������ʧ��[%hu,%hu]��", read_size, size);
//		return;
//	}
//
//	//��ȡD�Ĵ���
//	read_size = read(fp, &size, 2);
//	if(read_size != 2){ //��ȡʧ��
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡD�Ĵ����ֽ���ʧ�ܣ�");
//		return;
//	}
//
//	if(size != D_REG_COUNT*2){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "K�Ĵ�����С����ƥ��[%hu,%hu]��", size, D_REG_COUNT*2);
//		return;
//	}
//	read_size = read(fp, this->m_p_pmc_reg->GetRegPtr16(PMC_REG_D), size);
//	if(read_size != size){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡD�Ĵ�������ʧ��[%hu,%hu]��", read_size, size);
//		return;
//	}
//
//	//��ȡDC�Ĵ���
//	read_size = read(fp, &size, 2);
//	if(read_size != 2){ //��ȡʧ��
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡDC�Ĵ����ֽ���ʧ�ܣ�");
//		return;
//	}
//
//	if(size != C_REG_COUNT*2){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "DC�Ĵ�����С����ƥ��[%hu,%hu]��", size, C_REG_COUNT*2);
//		return;
//	}
//	read_size = read(fp, this->m_p_pmc_reg->GetRegPtr16(PMC_REG_DC), size);
//	if(read_size != size){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡDC�Ĵ�������ʧ��[%hu,%hu]��", read_size, size);
//		return;
//	}
//
//	//��ȡDT�Ĵ���
//	read_size = read(fp, &size, 2);
//	if(read_size != 2){ //��ȡʧ��
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡDT�Ĵ����ֽ���ʧ�ܣ�");
//		return;
//	}
//
//	if(size != T_REG_COUNT*2){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "DT�Ĵ�����С����ƥ��[%hu,%hu]��", size, T_REG_COUNT*2);
//		return;
//	}
//	read_size = read(fp, this->m_p_pmc_reg->GetRegPtr16(PMC_REG_DT), size);
//	if(read_size != size){
//		g_ptr_trace->PrintTrace(TRACE_ERROR, PMC_REGISTER, "��ȡDT�Ĵ�������ʧ��[%hu,%hu]��", read_size, size);
//		return;
//	}


	//�����ݷ��͸�MI
	uint16_t i = 0, count = 0;
	uint8_t *pn8 = nullptr;


	//����K�Ĵ���
	pn8 = this->m_p_pmc_reg->GetRegPtr8(PMC_REG_K);
	for(i = 0; i < K_REG_COUNT; i++){
		if(pn8[i] != 0){
			this->SendPmcRegValue(PMC_REG_K, i, pn8[i]);
			//printf("Init K reg k%hu = %hhu\n", i, pn8[i]);
		}
	}

#ifndef USES_PMC_2_0
	uint16_t *pn16 = nullptr;
	//����D�Ĵ���
	pn16 = this->m_p_pmc_reg->GetRegPtr16(PMC_REG_D);
	for(i = 0; i < D_REG_COUNT; i++){
		if(pn16[i] != 0){
			this->SendPmcRegValue(PMC_REG_D, i, pn16[i]);
			//printf("Init D reg d%hu = %hu\n", i, pn16[i]);
		}
	}

	//����DC�Ĵ���
	pn16 = this->m_p_pmc_reg->GetRegPtr16(PMC_REG_DC);
	for(i = 0; i < C_REG_COUNT; i++){
		if(pn16[i] != 0){
			this->SendPmcRegValue(PMC_REG_DC, i, pn16[i]);
			//printf("Init DC reg dc%hu = %hu\n", i, pn16[i]);
		}
	}

	//����DT�Ĵ���
	pn16 = this->m_p_pmc_reg->GetRegPtr16(PMC_REG_DT);
	for(i = 0; i < T_REG_COUNT; i++){
		if(pn16[i] != 0){
			this->SendPmcRegValue(PMC_REG_DT, i, pn16[i]);
			//printf("Init DT reg dt%hu = %hu\n", i, pn16[i]);
		}
	}

	//����C�Ĵ���
	pn16 = this->m_p_pmc_reg->GetRegPtr16(PMC_REG_C);
	for(i = 0; i < C_REG_COUNT; i++){
		if(pn16[i] != 0){
			this->SendPmcRegValue(PMC_REG_C, i, pn16[i]);
		//	printf("Init C reg c%hu = %hu\n", i, pn16[i]);
		}
	}
#else
	//����D�Ĵ���
	pn8 = this->m_p_pmc_reg->GetRegPtr8(PMC_REG_D);
	for(i = 0; i < D_REG_COUNT; i++){
		if(pn8[i] != 0){
			this->SendPmcRegValue(PMC_REG_D, i, pn8[i]);
			//printf("Init D reg d%hu = %hu\n", i, pn16[i]);
		}
	}

	//����C�Ĵ���
	pn8 = this->m_p_pmc_reg->GetRegPtr8(PMC_REG_C);
	count = C_REG_COUNT*4;
	for(i = 0; i < count; i++){
		if(pn8[i] != 0){
			this->SendPmcRegValue(PMC_REG_C, i, pn8[i]);
		//	printf("Init C reg c%hu = %hu\n", i, pn8[i]);
		}
	}

	//����T�Ĵ���
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
 * @brief ����MI����
 * @param axis : ��ţ� MI����Ŵ�1��ʼ, 0xFF��ʾϵͳ����
 * @param para_no : ������
 * @param data ������ָ��
 * @param size ������������ռ���ֽ���
 */
template<typename T>
void ChannelEngine::SendMiParam(uint8_t axis, uint32_t para_no, T data){
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_SET_PARAM;

	cmd.data.axis_index = axis;

	memcpy(cmd.data.data, &para_no, 4);
	memcpy(&cmd.data.data[2], &data, sizeof(T));

	this->m_p_mi_comm->WriteCmd(cmd);
}

/**
 * @brief ��MI���ͳ�ʼ������
 */
void ChannelEngine::InitMiParam(){

	printf("start init mi parameters\n");
//	uint8_t mi_axis_interface[] = {0xFF, 0x10, 0x01};   //�����ᡢ�����ᡢ��������

	//��ʼ��PMC�Ĵ���
	this->InitPmcReg();

	//��ʼ��SD-LINK��վ����
	SetSlaveInfo();

	//�������ֱ����ʽ
	this->SendMiParam<uint8_t>(0xFF, 10, this->m_p_general_config->hw_code_type);   //���ֱ�������

	//����IO��ӳ������
	ListNode<IoRemapInfo> *info_node = this->m_p_io_remap->HeadNode();
	while(info_node != nullptr){
		this->SendMiIoRemapInfo(info_node->data);

		info_node = info_node->next;
	}

	//���������η���
	SCAxisConfig *axis_config = nullptr;
	uint8_t index = 0;
	double tmp = 0.0;    //��ʱ����
	uint8_t tmp_8 = 0;   //��ʱ����
	uint64_t tmp_64 = 0;   //��ʱ����
	for(int i = 0; i < this->m_p_general_config->axis_count; i++){
		axis_config = &m_p_axis_config[i];
		index = i+1;   //MI�е���Ŵ�1��ʼ

		//���Ͳ岹��Ӽ�����ز���
		this->SendMiParam<uint8_t>(index, 1140, axis_config->post_filter_type);   //�˲�������
		this->SendMiParam<uint16_t>(index, 1141, axis_config->post_filter_time_1);   //һ���˲���ʱ�䳣��
		this->SendMiParam<uint16_t>(index, 1142, axis_config->post_filter_time_2);   //�����˲���ʱ�䳣��

		this->SendMiParam<uint8_t>(index, 1002, axis_config->axis_interface);   //��ӿ�����
		this->SendMiParam<uint8_t>(index, 1001, axis_config->axis_type);		//������
		this->SendMiParam<uint8_t>(index, 1003, axis_config->axis_port);		//��վ�Ż��߶�Ӧ��ں�
		this->SendMiParam<uint8_t>(index, 1006, axis_config->axis_pmc);			//�Ƿ�PMC��
		this->SendMiParam<double>(index, 1100, axis_config->kp1);			//kp1
		this->SendMiParam<double>(index, 1101, axis_config->kp2);			//kp2
		this->SendMiParam<double>(index, 1102, axis_config->ki);			//ki
		this->SendMiParam<double>(index, 1103, axis_config->kil);			//kil
		this->SendMiParam<double>(index, 1104, axis_config->kd);			//kd
		tmp = static_cast<double>(axis_config->kvff);
		this->SendMiParam<double>(index, 1105, tmp);			//kvff
		tmp = static_cast<double>(axis_config->kaff);
		this->SendMiParam<double>(index, 1106, tmp);			//kaff
		tmp_64 = static_cast<uint64_t>(axis_config->track_err_limit) * 1e4;   //ת����λ��um-->0.1nm
		this->SendMiParam<uint64_t>(index, 1107, tmp_64);	//���������
		tmp_64 = static_cast<uint64_t>(axis_config->location_err_limit) * 1e4;   //ת����λ��um-->0.1nm
		this->SendMiParam<uint64_t>(index, 1108, tmp_64);			//��λ�����
		this->SendMiParam<uint32_t>(index, 1200, axis_config->motor_count_pr);	//���ÿת����
		this->SendMiParam<uint32_t>(index, 1201, axis_config->motor_speed_max);	//������ת��
		tmp_64 = static_cast<uint64_t>(axis_config->move_pr * 1e7);   //ת����λ��mm-->0.1nm
// 		printf("write axis param idx = %d, value = %ld\n", index, tmp_64);
		this->SendMiParam<uint64_t>(index, 1202, tmp_64);	//ÿת�ƶ���
		this->SendMiParam<uint8_t>(index, 1203, axis_config->motor_dir);	//�����ת����
		tmp_8 = axis_config->feedback_mode;
//		if(axis_config->axis_interface == VIRTUAL_AXIS)
//			tmp_8 = 0xFF;    //������
//		else if(axis_config->axis_interface == BUS_AXIS)
//			tmp_8 = 0x10;   //ethercat
		this->SendMiParam<uint8_t>(index, 1204, tmp_8);	//��������
//		printf("send axis[%hhu] feedback_mode %hhu #####\n", index, tmp_8);

		tmp_8 = axis_config->ctrl_mode;
		if(axis_config->axis_interface == VIRTUAL_AXIS)
			tmp_8 = 0xFF;    //������
		else if(axis_config->axis_interface == BUS_AXIS)
			tmp_8 = 0x10;   //ethercat
		this->SendMiParam<uint8_t>(index, 1205, tmp_8);	//����Ʒ�ʽ
		this->SendMiParam<uint32_t>(index, 1206, axis_config->pulse_count_pr);	//����ÿת���������
		this->SendMiParam<uint8_t>(index, 1207, axis_config->encoder_lines);	//��Ȧ����������

		this->SendMiParam<uint16_t>(index, 1208, axis_config->encoder_max_cycle);   //��ת���������Ȧ���ֵ

//		this->SendMiParam<uint8_t>(index, 1209, axis_config->axis_alarm_level);	//��澯��ƽ

		if(axis_config->axis_interface != VIRTUAL_AXIS &&
				axis_config->feedback_mode != NO_ENCODER &&
				axis_config->feedback_mode != INCREMENTAL_ENCODER &&
				axis_config->ref_encoder != kAxisRefNoDef){ //д��ο�������
			this->m_p_mi_comm->SetAxisRef(index, axis_config->ref_encoder);
			printf("send mi ref encoder : %lld\n", axis_config->ref_encoder);
		}

		//����λ
		this->SendMiParam<double>(index, 1500, axis_config->soft_limit_max_1);
		this->SendMiParam<double>(index, 1501, axis_config->soft_limit_min_1);
		this->SendMiParam<uint8_t>(index, 1502, axis_config->soft_limit_check_1);
		this->SendMiParam<double>(index, 1503, axis_config->soft_limit_max_2);
		this->SendMiParam<double>(index, 1504, axis_config->soft_limit_min_2);
		this->SendMiParam<uint8_t>(index, 1505, axis_config->soft_limit_check_2);
		this->SendMiParam<double>(index, 1506, axis_config->soft_limit_max_3);
		this->SendMiParam<double>(index, 1507, axis_config->soft_limit_min_3);
		this->SendMiParam<uint8_t>(index, 1508, axis_config->soft_limit_check_3);

		//�����������ʱ�䡢�ƶ�ʱ��
		this->SendMiParam<uint16_t>(index, 1605, axis_config->spd_start_time);
		this->SendMiParam<uint16_t>(index, 1606, axis_config->spd_stop_time);

		//����ͬ������ز���
		this->SendMiParam<uint8_t>(index, 1650, axis_config->sync_axis);   //�Ƿ�ͬ����
		this->SendMiParam<uint8_t>(index, 1651, axis_config->master_axis_no); 	//�������
		this->SendMiParam<double>(index, 1653, axis_config->benchmark_offset); 	//��׼ƫ��
		this->SendMiParam<uint32_t>(index, 1655, axis_config->sync_err_max); 	//�����ͬ��������

		//���ͷ����϶����
		this->SendMiBacklash(i);

		//�������ݲ����ݱ�
		this->SendMiPcData(i);

		//�������ݲ����ò���
		this->SendMiPcParam(i);
		this->SendMiPcParam2(i);

		//������ο����Ӧ�Ļ�е����
		this->SendMiRefMachPos(i);
	}

	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].SendMiChnAxisMap();  //����ͨ����������ӳ���ϵ��Ϣ
	}

	this->SendMiPhyAxisEncoder();

	this->SetAxisRetRefFlag();   //���ͻزο��������־

	//��������������������
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_INIT_COMPLETE;
	cmd.data.axis_index = 0xFF;
	this->m_p_mi_comm->WriteCmd(cmd);

	m_b_init_mi_over = true;
	printf("exit init mi parameters\n");

}

/**
 * @brief ��MI�����ݲ����ݱ�
 * @param axis : ָ��������ţ���0��ʼ
 */
void ChannelEngine::SendMiPcData(uint8_t axis){
	if(this->m_p_pc_table == nullptr)
		return;
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_SET_AXIS_PC_DATA;

	cmd.data.axis_index = NO_AXIS;

	//��������
	uint16_t count = m_p_axis_config[axis].pc_count;
	uint16_t offset = m_p_axis_config[axis].pc_offset-1;  //��ʼ��ţ�0��ʼ
	if(this->m_p_axis_config[axis].pc_type == 1){//˫���ݲ�
		count *= 2;   //����������
	}

	int32_t value = 0;
	uint8_t dc = 0;    //ÿ�����ݰ������ݸ���

	for(uint16_t i = 0; i < count; i += 3, offset += 3){
		memset(cmd.data.data, 0x00, 14);
		dc = count-i;
		if(dc > 3)
			dc = 3;
		memcpy(&cmd.data.reserved, &dc, 1);   //�ݲ����ݸ���

		memcpy(cmd.data.data, &offset, 2);  //��ʼ���

		//��һ������
		value = this->m_p_pc_table->pc_table[axis][i] * 1e7;   //��λ0.1nm
		memcpy(&cmd.data.data[1], &value, 4);   //��ʼ���

		//�ڶ�������
		if(dc>=2){
			value = this->m_p_pc_table->pc_table[axis][i+1] * 1e7;   //��λ0.1nm
			memcpy(&cmd.data.data[3], &value, 4);   //��ʼ���
		}

		//����������
		if(dc == 3){
			value = this->m_p_pc_table->pc_table[axis][i+2] * 1e7;   //��λ0.1nm
			memcpy(&cmd.data.data[5], &value, 4);   //��ʼ���
		}

		this->m_p_mi_comm->WriteCmd(cmd);
	}
}

/**
 * @brief ��MI����ָ�����ݲ���������
 * @param axis : ָ��������ţ���0��ʼ
 */
void ChannelEngine::SendMiPcParam(uint8_t axis){
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_SET_AXIS_PC_PARAM;

	cmd.data.axis_index = axis+1;

	//��������
	uint16_t count = m_p_axis_config[axis].pc_count;
	uint16_t offset = m_p_axis_config[axis].pc_offset-1;  //��ʼ��ţ�0��ʼ
	uint32_t inter = m_p_axis_config[axis].pc_inter_dist*1000;   //ת��Ϊ΢�׵�λ
	uint16_t ref_index = m_p_axis_config[axis].pc_ref_index-1;   //�ο����Ӧλ�ã�0��ʼ
 	uint16_t pc_type = this->m_p_axis_config[axis].pc_type;
 	uint16_t pc_enable = m_p_axis_config[axis].pc_enable;

	memcpy(cmd.data.data, &count, 2);  //�������ݸ���
	memcpy(&cmd.data.data[1], &offset, 2);   //��ʼ���
	memcpy(&cmd.data.data[2], &inter, 4); 	 //������� �� um��λ��32λ����
	memcpy(&cmd.data.data[4], &ref_index, 2);     //�ο����Ӧλ��
	memcpy(&cmd.data.data[5], &pc_type, 1);     //��������  0--�����ݲ�   1--˫���ݲ�
	memcpy(&cmd.data.data[6], &pc_enable, 1);
	this->m_p_mi_comm->WriteCmd(cmd);
}


/**
 * @brief ��MI����ָ����Ĳο���λ�û�е����
 * @param axis_index : ������ţ���0��ʼ
 */
void ChannelEngine::SendMiRefMachPos(uint8_t axis_index){
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_SET_AXIS_REF_MACH_POS;

	cmd.data.axis_index = axis_index+1;

	//��������
	uint32_t pos = 0;
	for(uint16_t i = 0; i <10; i++){
		pos = m_p_axis_config[axis_index].axis_home_pos[i]*1000;   //ת��Ϊ΢�׵�λ
		memcpy(cmd.data.data, &pos, 4);  //
		cmd.data.data[2] = i;

		this->m_p_mi_comm->WriteCmd(cmd);
	}
}

/**
 * @brief ��MI����ָ�����ݲ���������2
 * @param axis : ������ţ���0��ʼ
 */
void ChannelEngine::SendMiPcParam2(uint8_t axis){
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_SET_AXIS_PC_PARAM2;

	cmd.data.axis_index = axis+1;

	//��������
	uint32_t pos = m_p_axis_config[axis].axis_home_pos[0]*1000;   //ת��Ϊ΢�׵�λ
	memcpy(cmd.data.data, &pos, 4);  //

	this->m_p_mi_comm->WriteCmd(cmd);

}

/**
 * @brief ��MI��������IO�ض��������
 * @param info
 */
void ChannelEngine::SendMiIoRemapInfo(IoRemapInfo &info){
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_SET_IO_REMAP;

	cmd.data.axis_index = 0xFF;

	//��������
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
 * @brief ��MI����ָ������˲�����ز���
 * @param phy_axis : ������ţ���0��ʼ
 */
void ChannelEngine::SendMiAxisFilterParam(uint8_t phy_axis){
	this->SendMiParam<uint8_t>(phy_axis+1, 1140, m_p_axis_config[phy_axis].post_filter_type);   //�˲�������
	this->SendMiParam<uint16_t>(phy_axis+1, 1141, m_p_axis_config[phy_axis].post_filter_time_1);   //һ���˲���ʱ�䳣��
	this->SendMiParam<uint16_t>(phy_axis+1, 1142, m_p_axis_config[phy_axis].post_filter_time_2);   //�����˲���ʱ�䳣��
}

/**
 * @brief ��MI����ָ����ķ����϶����
 * @param axis : ������ţ���0��ʼ
 */
void ChannelEngine::SendMiBacklash(uint8_t axis){
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_SET_AXIS_BACKLASH;

	cmd.data.axis_index = axis+1;

	//��������
	if(m_p_axis_config[axis].backlash_enable){
		uint32_t data = m_p_axis_config[axis].backlash_forward;
		memcpy(cmd.data.data, &data, 4);  //
		data = m_p_axis_config[axis].backlash_negative;
		memcpy(&cmd.data.data[2], &data, 4);
	}else{
		uint32_t data = 0.0;
		memcpy(cmd.data.data, &data, 4);  //
		data = 0.0;
		memcpy(&cmd.data.data[2], &data, 4);
	}

	this->m_p_mi_comm->WriteCmd(cmd);
}


/**
 * @brief ��MI����PMC�Ĵ���
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
 * @brief ���ŷ�
 */
void ChannelEngine::ServoOn(){
	//TODO �Ȱ�����ʹ�ܣ�������Ҫ�Ż���ͨ����ʹ��

	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_OPERATE;
	cmd.data.axis_index = 0xff;	  //����������ʹ��
	cmd.data.data[0] = AXIS_ON_FLAG;
	cmd.data.data[1] = 1;

	this->m_p_mi_comm->WriteCmd(cmd);

//	for(int i = 0; i < this->m_p_general_config->axis_count; i++){
//		cmd.data.axis_index = i+1;
//		this->m_p_mi_comm->WriteCmd(cmd);
//
//	}
}

/**
 * @brief ���ŷ�
 */
void ChannelEngine::ServoOff(){
	//TODO �Ȱ�����ʹ�ܣ�������Ҫ�Ż���ͨ����ʹ��

	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_OPERATE;
	cmd.data.axis_index = 0xff;	  //����������ʹ��
	cmd.data.data[0] = AXIS_ON_FLAG;
	cmd.data.data[1] = 0;

	this->m_p_mi_comm->WriteCmd(cmd);

//	for(int i = 0; i < this->m_p_general_config->axis_count; i++){
//		cmd.data.axis_index = i+1;
//		this->m_p_mi_comm->WriteCmd(cmd);
//	}
}


#ifdef USES_EMERGENCY_DEC_STOP
/**
 * @brief �ӳ����ŷ�
 * @param chn_index : ͨ���ţ�0xFF��ʾ����ͨ��
 */
void ChannelEngine::DelayToServoOff(uint8_t chn_index){
	printf("enter DelayToServoOff:%hhu\n", chn_index);
	if(chn_index == CHANNEL_ENGINE_INDEX){
		this->m_mask_delay_svo_off = 0;
		this->m_mask_delay_svo_off_over = 0;
		for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
			this->m_p_channel_control[i].DelayToServoOff();
			this->m_mask_delay_svo_off |= (0x01<<i);
		}

		this->m_b_delay_servo_off = true;

	}else if(chn_index < this->m_p_general_config->chn_count){
		this->m_p_channel_control[chn_index].DelayToServoOff();
	}
}

/**
 * @brief ����ͨ��ֹͣ��λ��־
 * @param chn_index : ͨ����
 */
void ChannelEngine::SetChnStoppedMask(uint8_t chn_index){
	if(chn_index < this->m_p_general_config->chn_count){
		this->m_mask_delay_svo_off_over |= (0x01<<chn_index);

		printf("SetChnStoppedMask: mask=0x%hhx, over = 0x%hhu", this->m_mask_delay_svo_off, this->m_mask_delay_svo_off_over);
	}
}
#endif

/**
 * @brief ����MI��λָ��
 */
void ChannelEngine::SendMiReset(){
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_RESET;
	cmd.data.axis_index = 0xFF;

	this->m_p_mi_comm->WriteCmd(cmd);

}

/**
 * @brief ����MC��ͨ���Զ����ݻ�������
 * @param count : ��������
 */
void ChannelEngine::SetMcAutoBufMax(uint16_t count){
//	this->m_n_mc_auto_buf_max = count;
	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].SetMcAutoBufMax(count-kMaxGCodeFifoCount-10);   //����2�������ȶ��Կ��ǣ�������
	}
}

/**
 * @brief ����SD-LINK��վ����
 */
void ChannelEngine::SetSlaveInfo(){
#ifdef USES_PMC_2_0
    MiCmdFrame cmd;

    //������չIO�忨��Ϣ����
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.data.cmd = CMD_MI_SET_SDLINK_SLAVE;
    cmd.data.axis_index = NO_AXIS;

    int dev_count = 0;          //��վ����
    int total_in = 0;           //�������ֽ���
    int total_out = 0;          //������ֽ���
    int handwheel_count = 0;    //���ָ���

    if (!m_list_bdio_dev.IsEmpty())
    {
        ListNode<BdioDevInfo> *node = this->m_list_bdio_dev.HeadNode();
        while(node != nullptr){
            //����ƴ��
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

			//���ݷ���
            this->m_p_mi_comm->WriteCmd(cmd);
            total_in += node->data.in_bytes;
            total_out += node->data.out_bytes;
            dev_count++;
            if (node->data.handwheel_map)
                ++handwheel_count;

            node = node->next;
        }

        //��չIO�忨�����������
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
    }

#else
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(cmd));
	cmd.data.cmd = CMD_MI_SET_SDLINK_SLAVE;
	cmd.data.axis_index = NO_AXIS;

	int total_in = 0;  //�������ֽ���
	int total_out = 0;  //������ֽ���
	int dev_count = 0;  //��վ����

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
 * @brief ���ͻ�ȡMI�汾ָ��
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
 * @brief ����澯
 */
void ChannelEngine::ClearAlarm(){
	//����澯
	g_ptr_alarm_processor->Clear();
}

/**
 * @brief ���MI�е�PMC���ƶ�����
 */
void ChannelEngine::ClearPmcAxisMoveData(){
	MiCmdFrame cmd;
	memset(&cmd, 0x00, sizeof(MiCmdFrame));

	cmd.data.axis_index = NO_AXIS;
	cmd.data.reserved = CHANNEL_ENGINE_INDEX;
	cmd.data.cmd = CMD_MI_PAUSE_PMC_AXIS;
	cmd.data.data[0] = 0x20;   //ֹͣ��ǰ����PMC���˶�����������ǰ�˶�ָ��

	m_p_mi_comm->WriteCmd(cmd);
	this->m_n_run_axis_mask = 0;
	this->m_n_runover_axis_mask = 0;
}

/**
 * @brief ϵͳ��λ
 */
void ChannelEngine::SystemReset(){

	printf("system reset aaa\n");
	//��ͨ����λ��λ
	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
	//	printf("@@@@@@@@@RST=====1\n");
		this->m_p_pmc_reg->FReg().bits[i].RST = 1;  //��λ�ź�
		this->m_p_channel_control[i].Reset();
	}

	printf("system reset bbb\n");

	//֪ͨMCģ�鸴λ
//	SendMcResetCmd();    //�޸�Ϊ��ͨ����λ


	//֪ͨMIģ�鸴λ
	this->SendMiReset();

	//֪ͨHMI����澯����λ
	HMICmdFrame cmd;
	cmd.channel_index = CHANNEL_ENGINE_INDEX;
	cmd.cmd = CMD_SC_RESET;
	cmd.cmd_extension = 0;
	cmd.data_len = 0;
	this->m_p_hmi_comm->SendCmd(cmd);

	//��λ�澯��
	this->m_error_code = ERR_NONE;


	//��ո澯����
	ClearAlarm();


	//TODO �������
	printf("reset update param \n");
	g_ptr_parm_manager->ActiveResetParam();
	g_ptr_parm_manager->ActiveNewStartParam();

//	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
//		this->m_p_channel_control[i].SetMcChnPlanMode();
//		this->m_p_channel_control[i].SetMcChnPlanParam();
//		this->m_p_channel_control[i].SetMcChnPlanFun();
//		this->m_p_channel_control[i].SetMcChnCornerStopParam();
//	}

	//��λͬ�����־
	if(m_b_emergency){  //��ͣ�������ͬ��
		m_b_send_sync_cmd = false;
		m_n_sync_over = 0;
	}

	m_b_emergency = false;  //��λ��ͣ��־
	m_b_power_off = false;


#ifdef USES_EMERGENCY_DEC_STOP
	this->m_b_delay_servo_off = false;
#endif

	this->m_hard_limit_negative = 0;
	this->m_hard_limit_postive = 0;

	//��ʼ���زο������
	if(m_b_ret_ref){
		for(uint8_t id = 0; id < this->m_p_general_config->axis_count; id++){
			if(this->m_n_mask_ret_ref & (0x01<<id)){
				this->SetInRetRefFlag(id, false);   //��λ�����б�־
			}
		}
	}
	this->m_b_ret_ref = false;
	this->m_b_ret_ref_auto = false;
	this->m_n_mask_ret_ref = 0;
//	m_n_get_cur_encoder_count = 0;
	memset(this->m_n_ret_ref_step, 0x00, kMaxAxisNum*sizeof(int));

	this->ClearPmcAxisMoveData();   //���PMC���˶�����

//	m_n_run_axis_mask = 0x00;  //��ǰ���е����mask
//	m_n_runover_axis_mask = 0x00;   //��ǰ������ɵ����mask   //ClearPmcAxisMoveData()�����


	//��¼��λ��������ʱ�䣬��RST�ź���ʱ��λ
	gettimeofday(&this->m_time_rst_over, NULL);
	m_b_reset_rst_signal = true;

	printf("channel engine reset finished !!!\n");
}

/**
 * @brief ��ͣ����
 * @param chn : ͨ����
 */
void ChannelEngine::Emergency(uint8_t chn){
	if(m_b_emergency)
		return;
	else
		m_b_emergency = true;

#ifndef USES_WOOD_MACHINE
	//֪ͨMI���ŷ�
	this->ServoOff();
#endif

	//��ͣ����
	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].EmergencyStop();
	}
	
	//��ʼ���زο������
	if(m_b_ret_ref){
		for(uint8_t id = 0; id < this->m_p_general_config->axis_count; id++){
			if(this->m_n_mask_ret_ref & (0x01<<id)){
				this->SetInRetRefFlag(id, false);   //��λ�����б�־
			}
		}
	}
	this->m_b_ret_ref = false;
	this->m_b_ret_ref_auto = false;
	this->m_n_mask_ret_ref = 0;
//	m_n_get_cur_encoder_count = 0;
	memset(this->m_n_ret_ref_step, 0x00, kMaxAxisNum*sizeof(int));

	this->ClearPmcAxisMoveData();   //���PMC���˶�����


	//��¼��λ��������ʱ�䣬��RST�ź���ʱ��λ
	gettimeofday(&this->m_time_rst_over, NULL);
	m_b_reset_rst_signal = true;


	//��λF�Ĵ���


	//��λ����ʽ��������Ļزο�����ɱ�־
#ifndef USES_WOOD_MACHINE
	for(int i = 0; i < this->m_p_general_config->axis_count; i++){
		if(m_p_axis_config[i].axis_interface != VIRTUAL_AXIS && m_p_axis_config[i].axis_type != AXIS_SPINDLE	//�����Ტ�ҷ�������
				&& (m_p_axis_config[i].feedback_mode == INCREMENTAL_ENCODER ||
						m_p_axis_config[i].feedback_mode == NO_ENCODER)   //����������
				&& m_p_axis_config[i].ret_ref_mode > 0){
//			m_n_mask_ret_ref_over &= ~(0x01<<i);   //
			this->SetRetRefFlag(i, false);
		}
	}
#endif

	//���ɼ�ͣ������Ϣ
	m_error_code = ERR_EMERGENCY;
	CreateError(ERR_EMERGENCY, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, chn);

}

/**
 * @brief  MI״̬�����̺߳���
 * @param args
 */
void *ChannelEngine::RefreshMiStatusThread(void *args){
	ChannelEngine *chn_engine = static_cast<ChannelEngine *>(args);
	printf("Start ChannelEnigine::RefreshMiStatusThread!thread id = %ld\n", syscall(SYS_gettid));

	int res = ERR_NONE;
	res = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //�����߳�Ϊ���˳�ģʽ
	if (res != ERR_NONE) {
		printf("Quit ChannelEnigine::RefreshMiStatusThread!thread!thread with error 1!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}
	res = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);  //�߳��˳���ʽΪ�첽�˳������õȵ�cancellation point
	if (res != ERR_NONE) {
		printf("Quit ChannelEnigine::RefreshMiStatusThread!thread!thread with error 2!\n");
		pthread_exit((void*) EXIT_FAILURE);
	}

	while(!g_sys_state.system_ready){  //�ȴ�ϵͳ���
//		printf("ChannelEngine::RefreshMiStatusThread wait system ready[0x%hhx]!\n", g_sys_state.module_ready_mask);
		usleep(10000);
	}

	chn_engine->RefreshMiStatusFun();

	printf("Quit ChannelEnigine::RefreshMiStatusThread!!\n");
	pthread_exit(NULL);
}

/**
 * @brief ����MI��״̬
 * @return
 */
bool ChannelEngine::RefreshMiStatusFun(){
	uint64_t count = 0;   //����

	int8_t value8 = 0;

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

		if((g_sys_state.module_ready_mask & MI_READY) == 0){  //MIδ׼������ȴ�
			printf("wait MI_READY signal!\n");
			usleep(8000);
			continue;
		}


		//��ȡǷѹ�ź�
		if(!m_b_power_off && g_sys_state.system_ready && this->m_p_mc_comm->ReadUnderVoltWarn()){
			printf("OFF\n");
			m_b_power_off = true;

			SaveDataPoweroff();
			g_sys_state.system_quit = true;   //�����˳�
			printf("OUT\n");
			return true;
		}

		//����д��F�Ĵ����� ��������8ms
		this->m_p_mi_comm->WritePmcReg(PMC_REG_F, p_f_reg);
		memcpy(m_g_reg_last.all, p_g_reg, sizeof(m_g_reg_last.all));  //����G�Ĵ���
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


		//��ȡ���µ�PMC�Ĵ���ֵ
		if(count % 10 == 0){	//��������80ms
			this->m_p_mi_comm->ReadPmcReg(PMC_REG_X, m_p_pmc_reg->GetRegPtr8(PMC_REG_X));
			this->m_p_mi_comm->ReadPmcReg(PMC_REG_Y, m_p_pmc_reg->GetRegPtr8(PMC_REG_Y));
			this->m_p_mi_comm->ReadPmcReg(PMC_REG_R, m_p_pmc_reg->GetRegPtr8(PMC_REG_R));
			this->m_p_mi_comm->ReadPmcReg(PMC_REG_A, m_p_pmc_reg->GetRegPtr8(PMC_REG_A));


//			if(count %200 == 0)
//				printf("k register : %hhu\n",m_p_pmc_reg->GetRegPtr8(PMC_REG_K)[0]);
//			if(count %310 == 0)
//				printf("D register : %hu\n",m_p_pmc_reg->GetRegPtr16(PMC_REG_D)[300]);

		}


		this->m_p_mi_comm->ReadServoOnState(this->m_n_phy_axis_svo_on);
		this->DoSyncAxis();
		this->CheckAxisSrvOn();


		if(!this->m_b_power_off){  //���󲻴���
			this->ProcessPmcSignal();

			//���ȶ�ȡ��澯��־
			this->m_p_mi_comm->ReadAxisWarnFlag(value8);
			if(value8 != 0x00){//�и澯���ٿ�����澯λ
				if(value8 & 0x01){//����λ�澯
					uint64_t hlimtflag = 0;
					this->m_p_mi_comm->ReadServoHLimitFlag(true, hlimtflag);   //��ȡ����λ����
					if(m_hard_limit_postive == 0){
						this->m_hard_limit_postive |= hlimtflag;
						this->ProcessAxisHardLimit(0);
					}else{
						this->m_hard_limit_postive |= hlimtflag;
					}
				}
				if(value8 & 0x02){ //����λ�澯
					uint64_t hlimtflag = 0;
					this->m_p_mi_comm->ReadServoHLimitFlag(false, hlimtflag);   //��ȡ����λ����
					if(m_hard_limit_negative == 0){
						this->m_hard_limit_negative |= hlimtflag;
						this->ProcessAxisHardLimit(1);
					}else{
						this->m_hard_limit_negative |= hlimtflag;
					}
				}
				if(value8 & 0x04){ //�������澯
					this->m_p_mi_comm->ReadEncoderWarn(value64);  //��ȡ�������澯����
					if(value64 != 0){
						//�б������澯
						uint64_t flag = 0x01;
						for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
							if(value64 & flag)
								CreateError(ERR_ENCODER, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
							flag = flag << 1;
						}

					}
				}
				if(value8 & 0x08){ //�ŷ��澯
					this->m_p_mi_comm->ReadServoWarn(value64);   //��ȡ�ŷ��澯
					if(value64 != 0){
						//���ŷ��澯
						uint64_t flag = 0x01;
						for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
							if(value64 & flag){
								this->m_p_mi_comm->ReadServoWarnCode(i, value32);   //��ȡ�澯��
								//�����澯
								CreateError(ERR_SERVO, ERROR_LEVEL, CLEAR_BY_MCP_RESET, value32, CHANNEL_ENGINE_INDEX, i);
							}
							flag = flag << 1;
						}
					}
				}
				if(value8 & 0x10){  //����������澯
					this->m_p_mi_comm->ReadTrackErr(value64);
					if(value64 != 0){
						//�и���������澯
						uint64_t flag = 0x01;
						for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
							if(value64 & flag)
								CreateError(ERR_TRACK_LIMIT, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
							flag = flag << 1;
						}
					}

				}
				if(value8 & 0x20){  //ͬ����λ��ָ��ƫ�����澯
					this->m_p_mi_comm->ReadSyncPosErr(value64);
					if(value64 != 0){
						//��ͬ����λ��ָ��ƫ�����澯
						uint64_t flag = 0x01;
						for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
							if(value64 & flag)
								CreateError(ERR_SYNC_POS, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
							flag = flag << 1;
						}
					}
				}
				if(value8 & 0x40){  //��λ��ָ�����澯
					this->m_p_mi_comm->ReadIntpPosErr(value64);
					if(value64 != 0){
						//����λ��ָ�����澯
						uint64_t flag = 0x01;
						for(uint8_t i = 0; i < this->m_p_general_config->axis_count; i++){
							if(value64 & flag)
								CreateError(ERR_INTP_POS, ERROR_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i);
							flag = flag << 1;
						}
					}
				}
			}


			//���й�ѹ��Ƿѹ��RTC��ѹ�͸澯ɨ��
#ifndef USES_MAIN_BOARD_10MA_OLD    //�ϰ�û�д˹���
			if(count%1000 == 0){
				this->CheckBattery();
			}
#endif

			if(this->m_b_ret_ref && this->m_p_pmc_reg->FReg().bits[0].SA ==1){//�زο������
				this->ReturnRefPoint();
			}
		}

		//��������λ���ݸ�MI
		this->m_p_mi_comm->WriteAxisHLimitFlag(true, m_hard_limit_postive);
		this->m_p_mi_comm->WriteAxisHLimitFlag(false, m_hard_limit_negative);

		//TODO �������λ�ü��ٶ�����
		if(count%7 == 0){
			if(!g_sys_state.hmi_comm_ready){  //δ����HMIʱ�ڴ˸����ᵱǰ����λ��
				this->SendMonitorData(false, false);
			}
		}

#ifdef USES_LICENSE_FUNC
		//ʱ��У���Լ���ȨУ��
		if(count % 450000 == 0){
			//һСʱ���һ��

			//���ϵͳʱ��
			if(m_ln_local_time < 0){//��ȡ���ؼ�ʱ�ļ��쳣
				if(m_ln_local_time == -1){
					g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "���ؼ�ʱ�ļ�������!");
				}else if(m_ln_local_time == -2){
					g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "���ؼ�ʱ�ļ�����!");
				}else if(m_ln_local_time == -3){
					g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "�Ƿ����ؼ�ʱ�ļ�!");
				}
				m_error_code = ERR_SYSTEM_FILE;
				CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
			}else if(-4 == CheckLocalTime(&m_lic_info, m_ln_local_time)){
				g_ptr_trace->PrintTrace(TRACE_ERROR, CHANNEL_ENGINE_SC, "ϵͳʱ���쳣!");
				m_error_code = ERR_SYSTEM_TIME;
				CreateError(m_error_code, ERROR_LEVEL, CLEAR_BY_MCP_RESET);
			}


			//�����Ȩ
			this->CheckLicense();

		}
#endif

#ifdef USES_EMERGENCY_DEC_STOP
		if(this->m_b_delay_servo_off){
			if(this->m_mask_delay_svo_off == this->m_mask_delay_svo_off_over){
				this->ServoOff();
				this->m_b_delay_servo_off = false;
				this->m_mask_delay_svo_off = 0;
				this->m_mask_delay_svo_off_over = 0;
			}
		}
#endif

		usleep(8000);  //8ms���ڣ���PMC������ͬ
		count++;
	}

//	printf("exit ChannelEngine::RefreshMiStatusFun()\n");

	return true;
}

/**
 * @brief ��ѹ��ظ澯���
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


	if(value & 0x01){//Ƿѹ
		CreateError(ERR_VOLTAGE_OVER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, NO_AXIS);

	}

	if(value & 0x02){//��ѹ
		CreateError(ERR_VOLTAGE_UNDER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, NO_AXIS);

	}
	if(value & 0x04){ //��ص�ѹ��
		CreateError(ERR_BATTERY_VOL_UNDER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, NO_AXIS);

	}
	if((value & 0x08) == 0x00){//�����,����Ч
		CreateError(ERR_AXIS_CURRENT_OVER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, NO_AXIS);

	}
	if((value & 0x10) == 0x00){ //USB����,����Ч
		CreateError(ERR_USB_CURRENT_OVER, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, NO_AXIS);
	}

	close(fd);

}

/**
 * @brief ����PMC���ź�
 */
void ChannelEngine::ProcessPmcSignal(){
	const GRegBits *g_reg = nullptr;
	GRegBits *g_reg_last = nullptr;
	FRegBits *f_reg = nullptr;
	uint64_t flag = 0;
	uint8_t chn = 0;
	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
		g_reg = &m_p_pmc_reg->GReg().bits[i];
		g_reg_last = &m_g_reg_last.bits[i];
		f_reg = &m_p_pmc_reg->FReg().bits[i];
#ifdef USES_PHYSICAL_MOP
		if(g_reg->_ESP == 0 && !m_b_emergency){ //��ͣ��Ч
//            printf("ChannelEngine::ProcessPmcSignal(), emergency stop!\n");
			this->Emergency();
		}

		// ����G�ź� �л���ǰͨ��
		if(g_reg_last->CHNC != g_reg->CHNC)
		{
			printf("pmc siganl change chn : %d\n", g_reg->CHNC);
			this->SetCurWorkChanl(g_reg->CHNC);
		}

		if((g_reg->ST && g_reg_last->ST == 0) && g_reg->_SP == 1 && f_reg->STL == 0){ //ѭ�����
			this->Start();
		}

		if(g_reg->_SP == 0 && g_reg_last->_SP == 1 && f_reg->SPL == 0 && f_reg->STL == 1){ //ѭ������
			this->Pause();
		}
#else
		if(g_reg->_ESP == 1 && !m_b_emergency){ //��ͣ��Ч
			this->Emergency();
		}

		if((g_reg->ST && g_reg_last->ST == 0) && g_reg->_SP == 0){ //ѭ�����
			this->Start();
		}

		if(g_reg->_SP == 1 && g_reg_last->_SP == 0){ //ѭ������
			//printf("1212\n");
			this->Pause();
		}
#endif

		if(g_reg->RRW == 1 && g_reg_last->RRW == 0){
			this->SystemReset();
		}


//#ifdef USES_PHYSICAL_MOP
		//��ʽѡ���ź�
		if(g_reg->MD != g_reg_last->MD){
//			printf("PMC SIGNAL -----> %d\n", g_reg->MD);
			if(g_reg->MD == 0){  //MDA
				this->SetWorkMode(MDA_MODE);
			}else if(g_reg->MD == 1){   //�Զ�
				this->SetWorkMode(AUTO_MODE);
			}else if(g_reg->MD == 2){   //����
				this->SetWorkMode(MPG_MODE);
			}else if(g_reg->MD == 3){   //�༭
				this->SetWorkMode(EDIT_MODE);
			}else if(g_reg->MD == 4){   //����
				this->SetWorkMode(MANUAL_STEP_MODE);
			}else if(g_reg->MD == 5){   //JOG
				this->SetWorkMode(MANUAL_MODE);
			}else if(g_reg->MD == 6){   //ԭ��ģʽ
				this->SetWorkMode(REF_MODE);
			}else{
				printf("ERROR:Invalid work mode signal:MD=%hhu\n", g_reg->MD);
			}
		}

		//������ת����
		if(g_reg->MD == 2 || g_reg->MD == 4 || g_reg->MD == 5){  //�ֶ�ģʽ
			if(g_reg->SPOS == 1 && g_reg_last->SPOS == 0 /*&& g_reg->SNEG == 0 && g_reg->_SSTP == 1 */&& f_reg->SPS != 1){ //������ת
				this->SpindleOut(SPD_DIR_POSITIVE);
			}
			if(g_reg->SNEG == 1 && g_reg_last->SNEG == 0 /*&& g_reg->SPOS == 0 && g_reg->_SSTP == 1 */&& f_reg->SPS != 2){ //���ᷴת
				this->SpindleOut(SPD_DIR_NEGATIVE);
			}
			if(g_reg->_SSTP == 0 && g_reg_last->_SSTP == 1 /*&& g_reg->SNEG == 0 && g_reg->SPOS == 0 */&& f_reg->SPS != 0){ //����ͣת
				this->SpindleOut(SPD_DIR_STOP);
			}
		}

		//ѡͣ�ź� SBS
		if(g_reg->SBS != g_reg_last->SBS){
			if(g_reg->SBS)
				this->SetFuncState(i, FS_OPTIONAL_STOP, 1);
			else
				this->SetFuncState(i, FS_OPTIONAL_STOP, 0);
		}


		//�����ź�  SBK
		if(g_reg->SBK != g_reg_last->SBK){
			if(g_reg->SBK)
				this->SetFuncState(i, FS_SINGLE_LINE, 1);
			else
				this->SetFuncState(i, FS_SINGLE_LINE, 0);
		}


		//�����ź�   BDT1
		if(g_reg->BDT1 != g_reg_last->BDT1){
			if(g_reg->BDT1)
				this->SetFuncState(i, FS_BLOCK_SKIP, 1);
			else
				this->SetFuncState(i, FS_BLOCK_SKIP, 0);
		}

		//���ָ���  HWT
		if(g_reg->HWT != g_reg_last->HWT){
			if(g_reg->HWT)
				this->SetFuncState(i, FS_HANDWHEEL_CONTOUR, 1);
			else
				this->SetFuncState(i, FS_HANDWHEEL_CONTOUR, 0);
		}


		//�ֶ������ź�  MP
		if(g_reg->MP != g_reg_last->MP){
			this->SetManualStep(i, g_reg->MP);
		}

		//�ֶ����ٽ���ѡ���ź�  RT
		if(g_reg->RT != g_reg_last->RT){
			if(g_reg->RT)
				this->SetManualRapidMove(1);
			else
				this->SetManualRapidMove(0);
		}

		//�����źŴ���
		if(g_reg->_JV != g_reg_last->_JV){
			uint16_t ratio= ~g_reg->_JV;
			if(g_reg->_JV == 0)
				ratio = 0;
			this->SetManualRatio(i, ratio/100);
		}
		if(g_reg->_FV != g_reg_last->_FV)
			this->SetAutoRatio(i, ~g_reg->_FV);
		if(g_reg->SOV != g_reg_last->SOV)
			this->SetSpindleRatio(i, g_reg->SOV);
		if(g_reg->ROV != g_reg_last->ROV)
			this->SetRapidRatio(i, g_reg->ROV);

		//�ֶ������
		if(g_reg->MD == 4){   //����
			//��1
			if(g_reg->JP1 && g_reg_last->JP1 == 0){
				this->SetCurAxis(i, 0);
				this->ManualMove(DIR_POSITIVE);
			}
			if(g_reg->JN1 && g_reg_last->JN1 == 0){
				this->SetCurAxis(i, 0);
				this->ManualMove(DIR_NEGATIVE);
			}
			//��2
			if(g_reg->JP2 && g_reg_last->JP2 == 0){
				this->SetCurAxis(i, 1);
				this->ManualMove(DIR_POSITIVE);
			}
			if(g_reg->JN2 && g_reg_last->JN2 == 0){
				this->SetCurAxis(i, 1);
				this->ManualMove(DIR_NEGATIVE);
			}
			//��3
			if(g_reg->JP3 && g_reg_last->JP3 == 0){
				this->SetCurAxis(i, 2);
				this->ManualMove(DIR_POSITIVE);
			}
			if(g_reg->JN3 && g_reg_last->JN3 == 0){
				this->SetCurAxis(i, 2);
				this->ManualMove(DIR_NEGATIVE);
			}
			//��4
			if(g_reg->JP4 && g_reg_last->JP4 == 0){
				this->SetCurAxis(i, 3);
				this->ManualMove(DIR_POSITIVE);
			}
			if(g_reg->JN4 && g_reg_last->JN4 == 0){
				this->SetCurAxis(i, 3);
				this->ManualMove(DIR_NEGATIVE);
			}
			//��5
			if(g_reg->JP5 && g_reg_last->JP5 == 0){
				this->SetCurAxis(i, 4);
				this->ManualMove(DIR_POSITIVE);
			}
			if(g_reg->JN5 && g_reg_last->JN5 == 0){
				this->SetCurAxis(i, 4);
				this->ManualMove(DIR_NEGATIVE);
			}
			//��6
			if(g_reg->JP6 && g_reg_last->JP6 == 0){
				this->SetCurAxis(i, 5);
				this->ManualMove(DIR_POSITIVE);
			}
			if(g_reg->JN6 && g_reg_last->JN6 == 0){
				this->SetCurAxis(i, 5);
				this->ManualMove(DIR_NEGATIVE);
			}
		}else if(g_reg->MD == 5){  //JOG
			//��1
			if(g_reg->JP1 != g_reg_last->JP1){
				if(g_reg->JP1){
					this->SetCurAxis(i, 0);
					this->ManualMove(DIR_POSITIVE);
				}else{
					this->ManualMoveStop();
				}
			}
			if(g_reg->JN1 != g_reg_last->JN1){
				if(g_reg->JN1){
					this->SetCurAxis(i, 0);
					this->ManualMove(DIR_NEGATIVE);
				}else{
					this->ManualMoveStop();
				}
			}
			//��2
			if(g_reg->JP2 != g_reg_last->JP2){
				if(g_reg->JP2){
					this->SetCurAxis(i, 1);
					this->ManualMove(DIR_POSITIVE);
				}else{
					this->ManualMoveStop();
				}
			}
			if(g_reg->JN2 != g_reg_last->JN2){
				if(g_reg->JN2){
					this->SetCurAxis(i, 1);
					this->ManualMove(DIR_NEGATIVE);
				}else{
					this->ManualMoveStop();
				}
			}
			//��3
			if(g_reg->JP3 != g_reg_last->JP3){
				if(g_reg->JP3){
					this->SetCurAxis(i, 2);
					this->ManualMove(DIR_POSITIVE);
				}else{
					this->ManualMoveStop();
				}
			}
			if(g_reg->JN3 != g_reg_last->JN3){
				if(g_reg->JN3){
					this->SetCurAxis(i, 2);
					this->ManualMove(DIR_NEGATIVE);
				}else{
					this->ManualMoveStop();
				}
			}
			//��4
			if(g_reg->JP4 != g_reg_last->JP4){
				if(g_reg->JP4){
					this->SetCurAxis(i, 3);
					this->ManualMove(DIR_POSITIVE);
				}else{
					this->ManualMoveStop();
				}
			}
			if(g_reg->JN4 != g_reg_last->JN4){
				if(g_reg->JN4){
					this->SetCurAxis(i, 3);
					this->ManualMove(DIR_NEGATIVE);
				}else{
					this->ManualMoveStop();
				}
			}
			//��5
			if(g_reg->JP5 != g_reg_last->JP5){
				if(g_reg->JP5){
					this->SetCurAxis(i, 4);
					this->ManualMove(DIR_POSITIVE);
				}else{
					this->ManualMoveStop();
				}
			}
			if(g_reg->JN5 != g_reg_last->JN5){
				if(g_reg->JN5){
					this->SetCurAxis(i, 4);
					this->ManualMove(DIR_NEGATIVE);
				}else{
					this->ManualMoveStop();
				}
			}
			//��6
			if(g_reg->JP6 != g_reg_last->JP6){
				if(g_reg->JP6){
					this->SetCurAxis(i, 5);
					this->ManualMove(DIR_POSITIVE);
				}else{
					this->ManualMoveStop();
				}
			}
			if(g_reg->JN6 != g_reg_last->JN6){
				if(g_reg->JN6){
					this->SetCurAxis(i, 5);
					this->ManualMove(DIR_NEGATIVE);
				}else{
					this->ManualMoveStop();
				}
			}
		}else if(g_reg->MD == 6){  //ԭ��ģʽ

			for(int j = 0; j < 16; j++){
				if((g_reg->REFE & (0x01<<j)) != 0 && (g_reg_last->REFE & (0x01<<j)) == 0){// ��������
					this->ProcessPmcRefRet(j+16*i);
				}
			}
		}


//#endif

		//�������ģʽ�л�
#ifdef USES_WOOD_MACHINE
		if(g_reg_last->BDM == 0 && g_reg->BDM == 1){
			this->m_p_channel_control[i].SetCurProcParamIndex(0);
		}else if(g_reg_last->BOXM == 0 && g_reg->BOXM == 1){
			this->m_p_channel_control[i].SetCurProcParamIndex(1);
		}
#endif

		//ʹ��FIN�źŵ������أ���λSF�ź�
		if(g_reg_last->FIN == 0 && g_reg->FIN == 1){
			if(f_reg->SF == 1)
				f_reg->SF = 0;
		}

		//����PMC����ù���
		if(g_reg_last->EMPC == 0 && g_reg->EMPC == 1){  //����PMC�����
			this->m_p_channel_control[i].CallMacroProgram(g_reg->MPCS);
			f_reg->MPCO = 1;   //���ý���
		}else if(g_reg_last->EMPC == 1 && g_reg->EMPC == 0){
			f_reg->MPCO = 0;   //�źŸ�λ
		}

#ifdef USES_WUXI_BLOOD_CHECK
		if(g_reg->ret_home){//����
			printf("wuxi, ret home\n");
			if(!m_p_channel_control[0].IsReturnHome()){
				m_p_channel_control[0].SetRetHomeFlag();  //��λ��־
			}
		}

		if(g_reg->reset){//��λ
			printf("wuxi, reset\n");
			for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
				this->m_p_channel_control[i].Reset();
			}
		}
#endif
	}

	//���������λ�źţ���Ȼ64�������λ�ź�ƽ���ֲ����ĸ�ͨ���У����Ǵ���ʱ����ͨ��
	if(!this->m_b_ret_ref){  //�زο��������������λ
		chn = m_p_general_config->axis_count/16;
		if(m_p_general_config->axis_count%16)
			chn++;
		for(int i = 0; i < chn; i++){
			g_reg = &m_p_pmc_reg->GReg().bits[i];
			if(g_reg->axis_limit_postive1 != 0x00 || g_reg->axis_limit_postive2 != 0x00){  //����Ӳ��λ����

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

			if(g_reg->axis_limit_negative1 != 0x00 || g_reg->axis_limit_negative2 != 0x00){  //����Ӳ��λ����
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
		}
	}

	//��λRST�ź�
	if(this->m_b_reset_rst_signal){
		struct timeval time_now;
		gettimeofday(&time_now, NULL);
		unsigned int time_elpase = (time_now.tv_sec-m_time_rst_over.tv_sec)*1000000+time_now.tv_usec-m_time_rst_over.tv_usec;
		if(time_elpase >= 16000){
			for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
				//��ʱʱ�䵽����λRST�ź�
				this->m_p_pmc_reg->FReg().bits[i].RST = 0;
			//	this->m_p_channel_control[i].ResetRSTSignal();

			}
			this->m_b_reset_rst_signal = false;
		}
	}

	//PMC����ƴ���
	this->ProcessPmcAxisCtrl();

	//����PMC���ݴ���
	this->ProcessPmcDataWnd();


	//�������ڲο����ź�
	int byte = 0, bit = 0;
	FRegBits *freg = nullptr;
	double prec = 0.1;  //��λ����
	for(int i = 0; i < this->m_p_general_config->axis_count; i++){
		chn = i/16;
		byte = i%16;
		bit = byte%8;
		freg = &m_p_pmc_reg->FReg().bits[chn];
		if((m_n_mask_ret_ref_over & (0x01<<i)) == 0){//δ�زο��㣬������
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
		//��һ�ο���
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
		//�ڶ��ο���
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
		//�����ο���
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
		//���Ĳο���
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

	//����PMC�ĸ澯����A��ַ�Ĵ���
	this->ProcessPmcAlarm();
	
}

/**
 * @brief ����PMC���ݴ���
 */
void ChannelEngine::ProcessPmcDataWnd(){

	FRegBits *freg = &m_p_pmc_reg->FReg().bits[0];
	const GRegBits *greg = &m_p_pmc_reg->GReg().bits[0];

	if(greg->ESTB && (freg->EREND==0)){  //ESTB�ź�Ϊ1����EREND�ź�Ϊ0ʱ��ִ������д��
		uint8_t cmd = greg->ED1;      //�����
		uint8_t axis = greg->ED2;    //�������

		if(cmd == 0x01){ //��ȡ���еλ������
			freg->wnd_data = this->GetPhyAxisMachPosFeedback(axis)*1000;   //��λ��um

			freg->EREND = 1;   //֪ͨPMC��ȡ����

		}
	}else if((greg->ESTB == 0) && freg->EREND){
		freg->EREND = 0;   //��λPMC��ȡ�����ź�
	}
}

/**
 * @brief ����PMC�澯
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
					CreateError(errorno, errtype, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX);
				}

				value = value>>1;  //����һλ
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
						errtype = FATAL_LEVEL;
					CreateError(errorno, errtype, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX);
				}

				value = value>>1;  //����һλ
				++bit;
			}
		}
		alarmreg += 8;  //pt++;
	}
#endif
}

/**
 * @brief ����PMC������ź�
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

		//����PMC��ͨ������
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

			if(!m_pmc_axis_ctrl[i*4+j].Active(eax[j])){//ת��ʧ�ܣ��澯
				this->m_error_code = ERR_PMC_AXIS_CTRL_CHANGE;
				CreateError(ERR_PMC_AXIS_CTRL_CHANGE, ERROR_LEVEL, CLEAR_BY_MCP_RESET, i*4+j+1, CHANNEL_ENGINE_INDEX);
				return;
			}

			if(!m_pmc_axis_ctrl[4*i+j].IsActive()){

				continue;  //δ����
			}

			//EMBUFg������Ч�ź�
			this->m_pmc_axis_ctrl[4*i+j].SetBuffState(!embuf[j]);

			//ESBKg/EMSBKg  �����ֹͣ�ź�/�����ֹͣ��Ч�ź�

			//���ݶ�ȡ
			if(ebuf[j] != ebsy[j]){
				pmc_cmd.cmd = cmd[j];
				pmc_cmd.distance = dis[j];
				pmc_cmd.speed = spd[j];
				this->m_pmc_axis_ctrl[4*i+j].WriteCmd(pmc_cmd);
			}

			//ECLRg��λ�ź�
			if(eclr[j])
				this->m_pmc_axis_ctrl[4*i+j].Reset();


			//ESTPg�������ͣ�ź�
			this->m_pmc_axis_ctrl[4*i+j].Pause(estp[j]);

		}
	}
}

/**
 * @brief �����ԭ���ź�
 * @param phy_axis �� ������ţ�0��ʼ
 * @param dir �� ���� 1--����   -1--����
 * @return  true--���ź�    false--���ź�
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
 * @brief �����ԭ�㾫��׼�źţ�IO�źţ�
 * @param phy_axis : ��������ţ�0��ʼ
 * @return  true--���ź�    false--���ź�
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
 * @brief ������Ӳ��λ�澯,���������ͣ
 * @param dir : Ӳ��λ����0--����   1--����
 */
void ChannelEngine::ProcessAxisHardLimit(uint8_t dir){

	//����ͣ����
	for(int i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].Pause();
	}

	//����Ӳ��λ������Ϣ
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
 * @brief ����PMC��F�Ĵ�����OUT�����λ
 * @param index : OUT����ĵ�λ��ţ���0��ʼ
 * @param value : 0--�������ԣ�ÿ��ȡ��      1--���ź�      2--���ź�
 */
void ChannelEngine::SetPmcOutSignal(uint8_t index, uint8_t value){
	if(index >= 32 || this->m_b_emergency)
		return;
	uint8_t sec = index/8;
	uint8_t bit = index%8;

//	uint8_t cur_val = this->m_p_pmc_reg->FReg().all[72+sec];
//	printf("SetPmcOutSignal: index = %hhu, value=0x%hhx\n", index, cur_val);
//	cur_val ^= (0x01<<bit);  //ȡ��

//	this->m_p_pmc_reg->FReg().all[72+sec] = cur_val;
//	printf("SetPmcOutSignal new : index = %hhu, value=0x%hhx\n", index, cur_val);
	if(value == 1){
		this->m_p_pmc_reg->FReg().all[72+sec] |= (0x01<<bit);
	}else if(value == 2){
		this->m_p_pmc_reg->FReg().all[72+sec] &= ~(0x01<<bit);
	}else if(value == 0){
		uint8_t cur_val = this->m_p_pmc_reg->FReg().all[72+sec];
	//	printf("SetPmcOutSignal: index = %hhu, value=0x%hhx\n", index, cur_val);
		cur_val ^= (0x01<<bit);  //ȡ��

		this->m_p_pmc_reg->FReg().all[72+sec] = cur_val;

	}

}

/**
 * @brief ��ȡ�������ֵ
 * @param chn[in] : ͨ����
 * @param index[in]	������������
 * @param init[out] : ���ر�����ʼ��״̬��0--δ��ʼ��   1--�ѳ�ʼ��
 * @param value[out] : ���صı���ֵ
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
 * @brief ���ú������ֵ
 * @param chn
 * @param index
 * @param init : ��ʼ��״̬��0--δ��ʼ��   1--�ѳ�ʼ��
 * @param value : ����ֵ
 * @return
 */
bool ChannelEngine::SetMacroVarValue(uint8_t chn, uint32_t index, bool init, double value){
	if(chn >= m_p_general_config->chn_count)
		return false;

	if((this->m_mask_import_param & (0x01<<CONFIG_MACRO_VAR)) != 0)
		return false;

	if(init){//�ѳ�ʼ��
		if(!m_p_channel_control[chn].GetMacroVar()->SetVarValue(index, value))
			return false;
	}else{//δ��ʼ��
		if(!m_p_channel_control[chn].GetMacroVar()->ResetVariable(index))
			return false;
	}

	return true;
}

/**
 * @brief ��ȡPMC���ֽڼĴ�����ֵ
 * @param reg_sec
 * @param reg_index
 * @param value
 * @return  true--�ɹ�   false--ʧ��
 */
bool ChannelEngine::GetPmcRegValue_8(uint16_t reg_sec, uint16_t reg_index, uint8_t &value){
	bool res = false;

	res = this->m_p_pmc_reg->GetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, value);

	return res;
}

/**
 * @brief ��ȡPMC˫�ֽڼĴ�����ֵ
 * @param reg_sec
 * @param reg_index
 * @param value
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelEngine::GetPmcRegValue_16(uint16_t reg_sec, uint16_t reg_index, uint16_t &value){
	bool res = false;

	res = this->m_p_pmc_reg->GetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, value);

	return res;
}

/**
 * @brief ����PMC���ֽڼĴ�����ֵ
 * @param reg_sec
 * @param reg_index
 * @param value
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelEngine::SetPmcRegValue_8(uint16_t reg_sec, uint16_t reg_index, uint8_t value){
	bool res = false;

	res = this->m_p_pmc_reg->SetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, value);

	return res;
}

/**
 * @brief ��λ���üĴ���ֵ
 * @param reg_sec
 * @param reg_index
 * @param bit_index : bitλ��ţ�0��ʼ
 * @param value
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelEngine::SetPmcRegBitValue_8(uint16_t reg_sec, uint16_t reg_index, uint8_t bit_index, bool value){
	bool res = false;

	if(bit_index > 7)
		return res;

	uint8_t data = 0;
	res = this->m_p_pmc_reg->GetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, data);

	if(!res)
		return res;  //ʧ�ܷ���

	if(value){
		data |= 0x01<<bit_index;      //��1
	}else{
		data &= ~(0x01<<bit_index);   //��0
	}

	res = this->m_p_pmc_reg->SetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, data);
	return res;
}

/**
 * @brief ����PMC˫�ֽڼĴ�����ֵ
 * @param reg_sec
 * @param reg_index
 * @param value
 * @return true--�ɹ�   false--ʧ��
 */
bool ChannelEngine::SetPmcRegValue_16(uint16_t reg_sec, uint16_t reg_index, uint16_t value){
	bool res = false;

	res = this->m_p_pmc_reg->SetRegValue(static_cast<PmcRegSection>(reg_sec), reg_index, value);

	return res;
}

/**
 * @brief ����ͨ����PMC�ź�
 * @param chn_index : ͨ����
 * @param signal ���źŶ���
 * @param flag �� true--��һ    false--����
 */
void ChannelEngine::SetPmcSignal(uint8_t chn_index, int signal, bool flag){

}

/**
 * @brief ˢ��nc�ļ�file
 * @param file
 */
void ChannelEngine::RefreshFile(char *file){
	for(int i = 0; i < this->m_p_general_config->chn_count; i++)
		this->m_p_channel_control[i].RefreshFile(file);
}

/**
 * @brief ���¼����ļ�file
 * @param file
 */
void ChannelEngine::RemapFile(char *file){
	for(int i = 0; i < this->m_p_general_config->chn_count; i++)
		this->m_p_channel_control[i].RemapFile(file);
}

/**
 * @brief ��λOP�ź�
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
 * @brief ���ø澯�ź�
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
 * @brief ��ȡָ��PMCͨ����PMC�����
 * @param pmc_chn : PMCͨ���ţ����Ĵ�������ţ���0��ʼ
 * @return �ɹ�����������ţ�ʧ���򷵻�0xFF
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
 * @brief ������ԭ���ź����òο��� ���������������ԭ���ź����òο���
 */
void ChannelEngine::AxisFindRefWithZeroSignal(uint8_t phy_axis){

	int8_t dir = 0, dir_opt;   //�زο��㷽��
//	uint8_t ret_mode = 0;   //�زο��㷽ʽ
	uint8_t chn = 0, chn_axis = 0;

	dir = this->m_p_axis_config[phy_axis].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;  // �زο����Ҵֻ�׼����
	dir_opt = (m_p_axis_config[phy_axis].ret_ref_change_dir==0)?dir*-1:dir;       //�زο����Ҿ���׼����

	switch(this->m_n_ret_ref_step[phy_axis]){
		case 0://���ԭ���ź�
			printf("return ref step 0, dir = %hhd, dir_opt=%hhd\n", dir, dir_opt);
			this->SetRetRefFlag(phy_axis, false);   //��λ�زο�����ɱ�־
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point &= ~(0x01<<phy_axis);

			if(this->CheckAxisRefBaseSignal(phy_axis, dir)){  // �Ѿ������ֻ�׼�źţ�ֱ�ӿ���ʼ���� 
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
		case 1:{//�ȴ����˵�λ�����ٴμ��ԭ���ź�
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
						m_n_ret_ref_step[phy_axis] = 2;  //��ת��һ��
				}
					
			}else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC�ᵽλ
				if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
					if(true == flag){
						m_n_ret_ref_step[phy_axis] = 0;
						break;
					}else{
							m_n_ret_ref_step[phy_axis] = 2;  //��ת��һ��
					}
				}
			}
			}
			break;
		case 2:  //�Իزο����ٶ���زο��㷽���˶�
	//		printf("return ref step 2\n");
			this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed, 90000.0);
			m_n_ret_ref_step[phy_axis] = 3;  //��ת��һ��
			printf("return ref, goto step 3\n");
			break;
		case 3:  //����ԭ���źţ�ֹͣ
	//		printf("return ref step 3\n");
			if(this->CheckAxisRefBaseSignal(phy_axis, dir)){
				this->ManualMoveStop(phy_axis);
				gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ��
				m_n_ret_ref_step[phy_axis] = 4;  //��ת��һ��
				printf("return ref, goto step 4: pos = %lf\n", this->GetPhyAxisMachPosFeedback(phy_axis));
			}
			else{
			//	this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed,3.0);
			}
			break;
		case 4:{//�ȴ�ֹͣ��λ
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 500000){ //��ʱ500ms
				m_n_ret_ref_step[phy_axis] = 5;  //��ת��һ��
				printf("return ref, goto step 5: pos = %lf, time=%u\n", this->GetPhyAxisMachPosFeedback(phy_axis), time_elpase);
			}
			if(!this->CheckAxisRefBaseSignal(phy_axis, dir)){
				printf("�زο�������ˣ�������������\n");
			}
		}
			break;
		case 5:  //�л�׼�� ���ٻ�����ԭ���ź���ʧ,
			printf("return ref step 5\n");
			this->ManualMove(phy_axis, dir_opt, 60, this->m_p_axis_config[phy_axis].move_pr*2);
			m_n_ret_ref_step[phy_axis] = 6;  //��ת��һ��
			printf("return ref, goto step 6\n");

			break;
		case 6:  //�л�׼���ȴ��ֻ�׼�ź���ʧ
		//	printf("return ref step 6\n");
			if(!this->CheckAxisRefBaseSignal(phy_axis, dir)){
				this->ManualMoveStop(phy_axis);

				gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ�䣬��ʱ300ms

				m_n_ret_ref_step[phy_axis] = 7;  //��ת��һ��
			}
			break;
		case 7:{ //�ȴ�ֹͣ��λ
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 500000){ //��ʱ500ms
//				m_n_get_cur_encoder_count = 0;  //��λ��ȡ��ǰ����������
				m_n_ret_ref_step[phy_axis] = 8;  //��ת��һ��
				printf("return ref, goto step 8\n");
			}
		}
			break;

		case 8:{ //  ����ԭ��
			printf("return ref step 8: send cmd to mi, set ref point\n");
			MiCmdFrame cmd;
			memset(&cmd, 0x00, sizeof(cmd));
			cmd.data.axis_index = phy_axis+1;
			cmd.data.cmd = CMD_MI_SET_REF_CUR;

			int64_t pos = m_p_axis_config[phy_axis].axis_home_pos[0] * 1e7;   //��λת����0.1nm
			memcpy(cmd.data.data, &pos, sizeof(int64_t));
			this->m_p_mi_comm->WriteCmd(cmd);
			gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ�䣬��ʱ300ms
			
			m_n_ret_ref_step[phy_axis] = 9;  //��ת��һ��
			printf("return ref, goto step 9\n");

		}
			break;
		case 9: { //�ȴ����òο�����ɣ���Miָ����Ӧ��������д���
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 300000){ //��ʱ300ms
				m_n_ret_ref_step[phy_axis] = 10;  //��ת��һ��
				printf("return ref, goto step 10\n");
			}
			}
			break;
		case 10:  //�زο������
			printf("axis %u return ref over\n", phy_axis);
			this->SetRetRefFlag(phy_axis, true);
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //��λ���ο����־
			

			this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
			m_n_ret_ref_step[phy_axis] = 0;

			break;
		case 20: //ʧ�ܴ���
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
 * @brief ������������չ�±�ʹ��
 * @param flag : �����������±�
 */
void ChannelEngine::SetAxisNameEx(bool flag){
	for(uint8_t i = 0; i < this->m_p_general_config->chn_count; i++){
		this->m_p_channel_control[i].SetAxisNameEx(flag);
	}
}

/**
 * @brief ��ǰλ������Ϊ�ο���   ������زο���  ����������òο���
 */
void ChannelEngine::AxisFindRefNoZeroSignal(uint8_t phy_axis){

	switch(this->m_n_ret_ref_step[phy_axis]){
		case 0: {// ����ԭ��
			printf("return ref step 0: send cmd to mi, set ref point\n");
			MiCmdFrame cmd;
			memset(&cmd, 0x00, sizeof(cmd));
			cmd.data.axis_index = phy_axis+1;
			cmd.data.cmd = CMD_MI_SET_REF_CUR;

			int64_t pos = m_p_axis_config[phy_axis].axis_home_pos[0] * 1e7;   //��λת����0.1nm
			memcpy(cmd.data.data, &pos, sizeof(int64_t));
			this->m_p_mi_comm->WriteCmd(cmd);
			gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ�䣬��ʱ300ms
			
			m_n_ret_ref_step[phy_axis] = 1;  //��ת��һ��
			printf("return ref, goto step 1\n");

		}
			break;
		case 1: { //�ȴ����òο�����ɣ���Miָ����Ӧ��������д���
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 300000){ //��ʱ300ms
				m_n_ret_ref_step[phy_axis] = 2;  //��ת��һ��
				printf("return ref, goto step 2\n");
			}
			}
			break;
		case 2:  //�زο������
			printf("axis %u return ref over\n", phy_axis);
			this->SetRetRefFlag(phy_axis, true);
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //��λ���ο����־
//			printf("return ref over flag : 0x%llx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx\n", this->m_p_pmc_reg->FReg().bits[0].in_ref_point,
//					m_p_pmc_reg->FReg().all[200], m_p_pmc_reg->FReg().all[201], m_p_pmc_reg->FReg().all[202], m_p_pmc_reg->FReg().all[203],
//					m_p_pmc_reg->FReg().all[204], m_p_pmc_reg->FReg().all[205], m_p_pmc_reg->FReg().all[206], m_p_pmc_reg->FReg().all[207]);
			this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
			m_n_ret_ref_step[phy_axis] = 0;


			break;
		case 20: //ʧ�ܴ���
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
 * @brief ��������л�׼��زο���
 */
void ChannelEngine::PulseAxisFindRefWithZeroSignal(uint8_t phy_axis){

	int8_t dir = 0, dir_opt;   //�زο��㷽��
//	uint8_t ret_mode = 0;   //�زο��㷽ʽ
	uint8_t chn = 0, chn_axis = 0;
	double dis = 0;      //�ƶ�����
	static double ret_ref_record_pos[kMaxAxisNum];

//	ret_mode = m_p_axis_config[phy_axis].ret_ref_mode;
	dir = this->m_p_axis_config[phy_axis].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;  // �زο����Ҵֻ�׼����
	dir_opt = (m_p_axis_config[phy_axis].ret_ref_change_dir==0)?dir*-1:dir;       //�زο����Ҿ���׼����

	switch(this->m_n_ret_ref_step[phy_axis]){
		case 0://���ԭ���ź�
			printf("return ref step 0, dir = %hhd, dir_opt=%hhd\n", dir, dir_opt);
			this->SetRetRefFlag(phy_axis, false);   //��λ�زο�����ɱ�־
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point &= ~(0x01<<phy_axis);

			if(this->CheckAxisRefBaseSignal(phy_axis, dir)){  // �Ѿ������ֻ�׼�źţ�ֱ�ӿ���ʼ���� 
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
		case 1:{//�ȴ����˵�λ�����ٴμ��ԭ���ź�
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
						m_n_ret_ref_step[phy_axis] = 2;  //��ת��һ��
					}
				}
					
			}else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC�ᵽλ
				if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
					if(true == flag){
						m_n_ret_ref_step[phy_axis] = 0;
						break;
					}else{
							m_n_ret_ref_step[phy_axis] = 2;  //��ת��һ��
					}
				}
			}
			}
			break;
		case 2:  //�Իزο����ٶ���زο��㷽���˶�
	//		printf("return ref step 2\n");
			this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed, 900.0);
			m_n_ret_ref_step[phy_axis] = 3;  //��ת��һ��
			printf("return ref, goto step 3\n");
			break;
		case 3:  //�����ֻ�׼�źţ�ֹͣ
	//		printf("return ref step 3\n");
			if(this->CheckAxisRefBaseSignal(phy_axis, dir)){
				this->ManualMoveStop(phy_axis);
				gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ��
				m_n_ret_ref_step[phy_axis] = 4;  //��ת��һ��
				printf("return ref, goto step 4\n");
			}
			else{
				this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed,3.0);
			}
			break;
		case 4:{//�ȴ�ֹͣ��λ
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 300000){ //��ʱ300ms
				m_n_ret_ref_step[phy_axis] = 5;  //��ת��һ��
				printf("return ref, goto step 5\n");
			}
		}
			break;
		case 5:  //�л�׼�� ���ٻ������ֻ�׼�ź���ʧ,
			printf("return ref step 5\n");
			this->ManualMove(phy_axis, dir_opt, 100, this->m_p_axis_config[phy_axis].move_pr*2); //�л�׼�� ���ٻ������ֻ�׼�ź���ʧ, �ٶȣ�100mm/min
			m_n_ret_ref_step[phy_axis] = 6;  //��ת��һ��
			printf("return ref, goto step 6\n");

			break;
		case 6:  //�л�׼���ȴ��ֻ�׼�ź���ʧ
		//	printf("return ref step 6\n");
			if(!this->CheckAxisRefBaseSignal(phy_axis, dir)){
				this->ManualMoveStop(phy_axis);

				gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ�䣬��ʱ300ms

				m_n_ret_ref_step[phy_axis] = 7;  //��ת��һ��
			}
			break;
		case 7:{ //�ȴ�ֹͣ��λ
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 200000){ //��ʱ500ms
//				m_n_get_cur_encoder_count = 0;  //��λ��ȡ��ǰ����������
				m_n_ret_ref_step[phy_axis] = 8;  //��ת��һ��
				printf("return ref, goto step 8\n");
			}
		}
			break;
		case 8:{ //  ���Ϳ�ʼ���񾫻�׼�źŵ�����
			printf("return ref step 8: send cmd to mi, set ref point\n");

			MiCmdFrame cmd;
			memset(&cmd, 0x00, sizeof(cmd));
			cmd.data.axis_index = phy_axis+1;
			cmd.data.cmd = CMD_MI_ACTIVE_Z_CAPT;
			this->m_p_mi_comm->WriteCmd(cmd);			
			
			m_n_ret_ref_step[phy_axis] = 9;  //��ת��һ��
			printf("return ref, goto step 9\n");

		}
			break;
			
		case 9:{ //  �����ƶ�������⾫��׼
			printf("return ref step 9: move to index pos\n");

			double move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;
			ret_ref_record_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir_opt;
			this->ManualMove(phy_axis, dir_opt,
						this->m_p_axis_config[phy_axis].ret_ref_speed, move_length);
			
			m_n_ret_ref_step[phy_axis] = 10;  //��ת��һ��
			printf("return ref, goto step 10\n");

		}
			break;

		case 10:{
			printf("return ref step 1\n");
			chn = this->GetAxisChannel(phy_axis, chn_axis);
			bool flag = CheckAxisRefBaseSignal(phy_axis, dir);  // ��ѯ�Ƿ��Ѿ����񵽾���׼    ?????????????????????????
			
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
					
			}else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC�ᵽλ
				if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
					if(false == flag){
						m_n_ret_ref_step[phy_axis] = 20;
						break;
					}else{
						this->ManualMoveStop(phy_axis);
						gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
						m_n_ret_ref_step[phy_axis] = 11;  //��ת��һ��
					}
				}
			}
		}	
			break;

			
		case 11: { //�ȴ����òο�����ɣ���Miָ����Ӧ��������д���
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 200000){ //��ʱ200ms			    
				m_n_ret_ref_step[phy_axis] = 12;  //��ת��һ��
				printf("return ref, goto step 12\n");
			}
			}
			break;

		case 12:       // ���û�׼
			MiCmdFrame cmd;
			memset(&cmd, 0x00, sizeof(cmd));
			cmd.data.axis_index = phy_axis+1;
			cmd.data.cmd = CMD_MI_SET_REF;
			this->m_p_mi_comm->WriteCmd(cmd);
			gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
			m_n_ret_ref_step[phy_axis] = 13;  //��ת��һ��
		    
			break;
			
		case 13: {
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 100000){ //��ʱ100ms			    
				m_n_ret_ref_step[phy_axis] = 14;  //��ת��һ��
				printf("return ref, goto step 14\n");
			}
			}
			
			break;			
		case 14: //�ȴ�������ɣ��ƶ�����ο���1
#ifdef USES_RET_REF_TO_MACH_ZERO
			dis = 0;   //��е��
#else
			dis = this->m_p_axis_config[phy_axis].axis_home_pos[0];   // ԭ������
#endif
			this->ManualMove(phy_axis, dis, m_p_axis_config[phy_axis].ret_ref_speed, 0);
			m_n_ret_ref_step[phy_axis] = 15;  //��ת��һ��
			printf("return ref, goto step 15\n");
			break;
			
		case 15:  //�ȴ����ƶ���λ,��ɣ�
#ifdef USES_RET_REF_TO_MACH_ZERO
			dis = 0;
#else
			dis = m_p_axis_config[phy_axis].axis_home_pos[0];
#endif
			if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis)- dis) <= 0.010){  //��λ
				m_n_ret_ref_step[phy_axis] = 16;  //��ת��һ��
			}
			break;
		case 16:  //�زο������
			printf("axis %u return ref over\n", phy_axis);
			this->SetRetRefFlag(phy_axis, true);
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //��λ���ο����־
//			printf("return ref over flag : 0x%llx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx\n", this->m_p_pmc_reg->FReg().bits[0].in_ref_point,
//					m_p_pmc_reg->FReg().all[200], m_p_pmc_reg->FReg().all[201], m_p_pmc_reg->FReg().all[202], m_p_pmc_reg->FReg().all[203],
//					m_p_pmc_reg->FReg().all[204], m_p_pmc_reg->FReg().all[205], m_p_pmc_reg->FReg().all[206], m_p_pmc_reg->FReg().all[207]);
			this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
			m_n_ret_ref_step[phy_axis] = 0;
			break;
			
		case 20: //ʧ�ܴ���
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
 * @brief ��������޻�׼��زο���
 */
void ChannelEngine::PulseAxisFindRefNoZeroSignal(uint8_t phy_axis){

	int8_t dir = 0, dir_opt;   //�زο��㷽��
//	uint8_t ret_mode = 0;   //�زο��㷽ʽ
	uint8_t chn = 0, chn_axis = 0;
	double dis = 0;      //�ƶ�����
	static double ret_ref_record_pos[kMaxAxisNum];

//	ret_mode = m_p_axis_config[phy_axis].ret_ref_mode;
	dir = this->m_p_axis_config[phy_axis].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;  // �زο����Ҵֻ�׼����
	dir_opt = (m_p_axis_config[phy_axis].ret_ref_change_dir==0)?dir*-1:dir;       //�زο����Ҿ���׼����

	switch(this->m_n_ret_ref_step[phy_axis]){
		case 0: {//�򾫻�׼��ʼ�ƶ�
			printf("return ref step 0, dir = %hhd, dir_opt=%hhd\n", dir, dir_opt);
			this->SetRetRefFlag(phy_axis, false);   //��λ�زο�����ɱ�־
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point &= ~(0x01<<phy_axis);

			double move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;
			ret_ref_record_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir_opt;
				this->ManualMove(phy_axis, dir_opt,
						this->m_p_axis_config[phy_axis].ret_ref_speed, move_length);
		
			m_n_ret_ref_step[phy_axis] = 2;
			}
			break;
		case 2:{ //  ���Ϳ�ʼ���񾫻�׼�źŵ�����
			printf("return ref step 2: send cmd to mi, set ref point\n");

			MiCmdFrame cmd;
			memset(&cmd, 0x00, sizeof(cmd));
			cmd.data.axis_index = phy_axis+1;
			cmd.data.cmd = CMD_MI_ACTIVE_Z_CAPT;
			this->m_p_mi_comm->WriteCmd(cmd);
			
			gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ�䣬��ʱ300ms
			
			m_n_ret_ref_step[phy_axis] = 3;  //��ת��һ��
			printf("return ref, goto step 3\n");

		}
			break;
			
		case 3:{ //  �����ƶ�������⾫��׼
			printf("return ref step 3: move to index pos\n");

			double move_length = this->m_p_axis_config[phy_axis].move_pr*1.2;
			ret_ref_record_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir_opt;
			this->ManualMove(phy_axis, dir_opt,
						this->m_p_axis_config[phy_axis].ret_ref_speed, move_length);						
			
			m_n_ret_ref_step[phy_axis] = 4;  //��ת��һ��
			printf("return ref, goto step 4\n");

		}
			break;

		case 4:{
			printf("return ref step 4\n");
			chn = this->GetAxisChannel(phy_axis, chn_axis);
			bool flag = CheckAxisRefBaseSignal(phy_axis, dir);  // ��ѯ�Ƿ��Ѿ����񵽾���׼    ?????????????????????????
			
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
					
			}else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC�ᵽλ
				if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
					if(false == flag){
						m_n_ret_ref_step[phy_axis] = 20;
						break;
					}else{
				            this->ManualMoveStop(phy_axis);
							gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
							m_n_ret_ref_step[phy_axis] = 5;  //��ת��һ��
					}
				}
			}
		}	
			break;

			
		case 5: { //�ȴ����òο�����ɣ���Miָ����Ӧ��������д���
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 200000){ //��ʱ200ms			    
				m_n_ret_ref_step[phy_axis] = 6;  //��ת��һ��
				printf("return ref, goto step 6\n");
			}
			}
			break;

		case 6:       // ���û�׼
			MiCmdFrame cmd;
			memset(&cmd, 0x00, sizeof(cmd));
			cmd.data.axis_index = phy_axis+1;
			cmd.data.cmd = CMD_MI_SET_REF;
			this->m_p_mi_comm->WriteCmd(cmd);
			gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);
			m_n_ret_ref_step[phy_axis] = 7;  //��ת��һ��
		    
			break;
			
		case 7: {
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 100000){ //��ʱ100ms			    
				m_n_ret_ref_step[phy_axis] = 8;  //��ת��һ��
				printf("return ref, goto step 8\n");
			}
			}			
			break;		
			
		case 8: //�ȴ�������ɣ��ƶ�����ο���1
#ifdef USES_RET_REF_TO_MACH_ZERO
			dis = 0;    //��е��
#else
			dis = this->m_p_axis_config[phy_axis].axis_home_pos[0];   // ԭ������
#endif
			this->ManualMove(phy_axis, dis, m_p_axis_config[phy_axis].ret_ref_speed, 0);
			m_n_ret_ref_step[phy_axis] = 9;  //��ת��һ��
			printf("return ref, goto step 9\n");
			break;
			
		case 9:  //�ȴ����ƶ���λ,��ɣ�
#ifdef USES_RET_REF_TO_MACH_ZERO
			dis = 0;    //��е��
#else
			dis = this->m_p_axis_config[phy_axis].axis_home_pos[0];   // ԭ������
#endif
			if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis)- dis) <= 0.010){  //��λ
				m_n_ret_ref_step[phy_axis] = 10;  //��ת��һ��
			}
			break;
		case 11:  //�زο������
			printf("axis %u return ref over\n", phy_axis);
			this->SetRetRefFlag(phy_axis, true);
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //��λ���ο����־
//			printf("return ref over flag : 0x%llx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx\n", this->m_p_pmc_reg->FReg().bits[0].in_ref_point,
//					m_p_pmc_reg->FReg().all[200], m_p_pmc_reg->FReg().all[201], m_p_pmc_reg->FReg().all[202], m_p_pmc_reg->FReg().all[203],
//					m_p_pmc_reg->FReg().all[204], m_p_pmc_reg->FReg().all[205], m_p_pmc_reg->FReg().all[206], m_p_pmc_reg->FReg().all[207]);
			this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
			m_n_ret_ref_step[phy_axis] = 0;
			break;
			
		case 20: //ʧ�ܴ���
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
 * @brief  ethcat���л�׼��زο���
 */
void ChannelEngine::EcatAxisFindRefWithZeroSignal(uint8_t phy_axis){

	int8_t dir = 0, dir_opt;   //�زο��㷽��
//	uint8_t ret_mode = 0;   //�زο��㷽ʽ
	uint8_t chn = 0, chn_axis = 0;
	double dis = 0;      //�ƶ�����
//	static double ret_ref_start_pos[kMaxAxisNum];

//	ret_mode = m_p_axis_config[phy_axis].ret_ref_mode;
	dir = this->m_p_axis_config[phy_axis].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;  // �زο����Ҵֻ�׼����
	dir_opt = (m_p_axis_config[phy_axis].ret_ref_change_dir==0)?dir*-1:dir;       //�زο����Ҿ���׼����

	switch(this->m_n_ret_ref_step[phy_axis]){
		case 0://���ԭ���ź�
			printf("return ref step 0, dir = %hhd, dir_opt=%hhd\n", dir, dir_opt);
			this->SetRetRefFlag(phy_axis, false);   //��λ�زο�����ɱ�־
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point &= ~(0x01<<phy_axis);

			if(this->CheckAxisRefBaseSignal(phy_axis, dir)){  // �Ѿ������ֻ�׼�źţ�ֱ�ӿ���ʼ���� 
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
		case 1:{//�ȴ����˵�λ�����ٴμ��ԭ���ź�
		//	printf("return ref step 1\n");
			chn = this->GetAxisChannel(phy_axis, chn_axis);
			bool flag = CheckAxisRefBaseSignal(phy_axis, dir);
			
			if(chn != CHANNEL_ENGINE_INDEX){
				if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - m_df_ret_ref_tar_pos[phy_axis]) <= 0.010){
					if(true == flag){   //�ص�step0����������
						m_n_ret_ref_step[phy_axis] = 0;
						break;
					}
				}
				if(flag == false){
						m_n_ret_ref_step[phy_axis] = 2;  //��ת��һ��
				}
					
			}else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC�ᵽλ
				if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
					if(true == flag){
						m_n_ret_ref_step[phy_axis] = 0;
						break;
					}else{
							m_n_ret_ref_step[phy_axis] = 2;  //��ת��һ��
					}
				}
			}
			}
			break;
		case 2:  //�Իزο����ٶ���زο��㷽���˶�
	//		printf("return ref step 2\n");
			this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed, 90000.0);
			m_n_ret_ref_step[phy_axis] = 3;  //��ת��һ��
			printf("return ref, goto step 3\n");
			break;
		case 3:  //�����ֻ�׼�źţ�ֹͣ
	//		printf("return ref step 3\n");
			if(this->CheckAxisRefBaseSignal(phy_axis, dir)){
				this->ManualMoveStop(phy_axis);
				gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ��
				m_n_ret_ref_step[phy_axis] = 4;  //��ת��һ��
				this->SendMonitorData(false, false);
				printf("return ref, goto step 4, curpos=%lf\n", this->GetPhyAxisMachPosIntp(phy_axis));
			}
			else{
			//	this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed,1.0);
			}
			break;
		case 4:{//�ȴ�ֹͣ��λ
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 1000000){ //��ʱ1000ms
				m_n_ret_ref_step[phy_axis] = 5;  //��ת��һ��
				this->SendMonitorData(false, false);
				printf("return ref, goto step 5, curpos = %lf, time=%u\n", this->GetPhyAxisMachPosIntp(phy_axis), time_elpase);
			}
		}
			break;
		case 5:{  //�л�׼�� ���ٻ������ֻ�׼�ź���ʧ,
		//	printf("return ref step 5\n");
			double move_length = this->m_p_axis_config[phy_axis].move_pr*2;
			if(move_length>10.0) {
				move_length = 10.0;
			}else if(move_length < 1.0){
				move_length = 1.0;
			}
			m_df_ret_ref_tar_pos[phy_axis] = this->GetPhyAxisMachPosFeedback(phy_axis)+move_length* dir_opt;
			this->ManualMove(phy_axis, dir_opt, 60, move_length); //�л�׼�� ���ٻ������ֻ�׼�ź���ʧ, �ٶȣ�60mm/min
			m_n_ret_ref_step[phy_axis] = 6;  //��ת��һ��
			printf("return ref, goto step 6\n");

			break;
		}
		case 6:  //�л�׼���ȴ��ֻ�׼�ź���ʧ
		//	printf("return ref step 6\n");
			if(!this->CheckAxisRefBaseSignal(phy_axis, dir)){
				this->ManualMoveStop(phy_axis);

				gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ�䣬��ʱ600ms

				m_n_ret_ref_step[phy_axis] = 7;  //��ת��һ��
			}else if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis) - m_df_ret_ref_tar_pos[phy_axis]) <= 0.010){  //�ӽ�Ŀ��λ�ã��ֻ�׼�źŻ��ڣ����������
				m_n_ret_ref_step[phy_axis] = 5;  //��ת��5��
			}
			break;
		case 7:{ //�ȴ�ֹͣ��λ
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 600000){ //��ʱ600ms
//				m_n_get_cur_encoder_count = 0;  //��λ��ȡ��ǰ����������
				if(!this->CheckAxisRefBaseSignal(phy_axis, dir)){
					m_n_ret_ref_step[phy_axis] = 8;  //��ת��һ��
					printf("return ref, goto step 8\n");
				}else{      //����ַ��ִֻ�׼���źţ����������
					m_n_ret_ref_step[phy_axis] = 5;  //��������5��
					printf("return ref, goto step 5\n");
				}

			}
		}
			break;
		case 8:{ // ����ԭ��
	//		printf("return ref step 8: send cmd to mi, set ref point\n");
			MiCmdFrame cmd;
			memset(&cmd, 0x00, sizeof(cmd));
			cmd.data.axis_index = phy_axis+1;
			cmd.data.cmd = CMD_MI_SET_REF_POINT;


			uint16_t err = m_p_axis_config[phy_axis].ref_mark_err * 1e3;     //��λת���� mm->um
			cmd.data.data[0] = err;    //�ο����׼���
			int64_t pos = m_p_axis_config[phy_axis].axis_home_pos[0]*1e7;   //��λת��,0.1nm
			memcpy(&cmd.data.data[1], &pos, sizeof(int64_t));
			cmd.data.data[5] = this->m_p_axis_config[phy_axis].ref_signal;
			cmd.data.data[6] = this->m_p_axis_config[phy_axis].ret_ref_dir;

			this->m_p_mi_comm->WriteCmd(cmd);
//			gettimeofday(&this->m_time_ret_ref[phy_axis], nullptr);   //��¼��ʼʱ�䣬��ʱ300ms
			
			m_n_ret_ref_step[phy_axis] = 9;  //��ת��һ��
			printf("return ref, goto step 9\n");

		}
			break;
		case 9: //�ȴ����òο�����ɣ���Miָ����Ӧ��������д���

			break;
		case 10: //�ȴ����ƶ���λ,���
#ifdef USES_RET_REF_TO_MACH_ZERO
			dis = 0;    //��е��
#else
			dis = this->m_p_axis_config[phy_axis].axis_home_pos[0];   // ԭ������
#endif
			this->SendMonitorData(false, false);  //�ٴζ�ȡʵʱλ��

			if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis)- dis) <= 0.010){  //��λ
				m_n_ret_ref_step[phy_axis] = 11;  //��ת��һ��
			}
		//	printf("return ref, goto step 11\n");
			break;
		case 11:  //�زο������
			printf("axis %u return ref over\n", phy_axis);
			if(this->m_p_axis_config[phy_axis].feedback_mode == ABSOLUTE_ENCODER_YASAKAWA ||
				this->m_p_axis_config[phy_axis].feedback_mode == ABSOLUTE_ENCODER_PANASONIC ||
				this->m_p_axis_config[phy_axis].feedback_mode == LINEAR_ENCODER){  //����ֵ���ȡ��е���ı�����ֵ�����棬����ϵ��ָ�����
				MiCmdFrame mi_cmd;
				memset(&mi_cmd, 0x00, sizeof(mi_cmd));
				mi_cmd.data.cmd = CMD_MI_GET_ZERO_ENCODER;
				mi_cmd.data.axis_index = phy_axis+1;

				this->m_p_mi_comm->WriteCmd(mi_cmd);
			}
			this->SetRetRefFlag(phy_axis, true);
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //��λ���ο����־

//			printf("return ref over flag : 0x%llx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx\n", this->m_p_pmc_reg->FReg().bits[0].in_ref_point,
//					m_p_pmc_reg->FReg().all[200], m_p_pmc_reg->FReg().all[201], m_p_pmc_reg->FReg().all[202], m_p_pmc_reg->FReg().all[203],
//					m_p_pmc_reg->FReg().all[204], m_p_pmc_reg->FReg().all[205], m_p_pmc_reg->FReg().all[206], m_p_pmc_reg->FReg().all[207]);
			this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
			m_n_ret_ref_step[phy_axis] = 0;

			if(m_n_mask_ret_ref == 0){
				this->m_b_ret_ref = false;
				this->m_b_ret_ref_auto = false;
				m_n_ret_ref_auto_cur = 0;
			}
			break;
		case 20: //ʧ�ܴ���
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
 * @brief ethcat���л�׼�زο��㣬�ֻ�׼�뾫��׼��ΪIO�źţ�ֱ�ߵ��
 * @param phy_axis
 */
void ChannelEngine::EcatAxisFindRefWithZeroSignal2(uint8_t phy_axis){
	int8_t dir = 0, dir_opt;   //�زο��㷽��
//	uint8_t ret_mode = 0;   //�زο��㷽ʽ
	uint8_t chn = 0, chn_axis = 0;
//	double dis = 0;      //�ƶ�����
//	static double ret_ref_start_pos[kMaxAxisNum];

//	ret_mode = m_p_axis_config[phy_axis].ret_ref_mode;
	dir = this->m_p_axis_config[phy_axis].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;  // �زο����Ҵֻ�׼����
	dir_opt = (m_p_axis_config[phy_axis].ret_ref_change_dir==0)?dir*-1:dir;       //�زο����Ҿ���׼����

	switch(this->m_n_ret_ref_step[phy_axis]){
		case 0://���ԭ���ź�
			printf("return ref step 0, dir = %hhd, dir_opt=%hhd\n", dir, dir_opt);
			this->SetRetRefFlag(phy_axis, false);   //��λ�زο�����ɱ�־
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point &= ~(0x01<<phy_axis);

			if(this->CheckAxisRefBaseSignal(phy_axis, dir)){  // �Ѿ������ֻ�׼�źţ�ֱ�ӿ�ʼ����
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
		case 1:{//�ȴ����˵�λ�����ٴμ��ԭ���ź�
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
					m_n_ret_ref_step[phy_axis] = 2;  //��ת��һ��
				}

			}else if(this->m_p_axis_config[phy_axis].axis_pmc){  //PMC�ᵽλ
				if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<phy_axis))){
					if(true == flag){
						m_n_ret_ref_step[phy_axis] = 0;
						break;
					}else{
						m_n_ret_ref_step[phy_axis] = 2;  //��ת��һ��
					}
				}
			}
			}
			break;
		case 2:  //�Իزο����ٶ���زο��㷽���˶�
	//		printf("return ref step 2\n");
			this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed, 90000.0);
			m_n_ret_ref_step[phy_axis] = 3;  //��ת��һ��
			printf("return ref, goto step 3\n");
			break;
		case 3:  //�����ֻ�׼�źţ�ֹͣ
	//		printf("return ref step 3\n");
			if(this->CheckAxisRefBaseSignal(phy_axis, dir)){  //�����ֻ�׼
				if(m_p_axis_config[phy_axis].ret_ref_change_dir){  //����Ѱ�Ҿ���׼������ֹͣ��λ
					this->ManualMoveStop(phy_axis);
					gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ��
					m_n_ret_ref_step[phy_axis] = 4;  //��ת��һ��
					printf("return ref, goto step 4\n");
				}else{  //ͬ�����Ѱ�Ҿ���׼
					this->ManualMove(phy_axis, dir_opt, 20,3.0);
					m_n_ret_ref_step[phy_axis] = 5;  //��ת��һ��
				}
			}
			else{
			//	this->ManualMove(phy_axis, dir, this->m_p_axis_config[phy_axis].ret_ref_speed,3.0);
			}
			break;
		case 4:{//�ȴ�ֹͣ��λ
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 300000){ //��ʱ300ms
				m_n_ret_ref_step[phy_axis] = 5;  //��ת��һ��
				printf("return ref, goto step 5\n");
			}
		}
			break;
		case 5:  //����Ѱ�Ҿ���׼�˶�
			printf("return ref step 5\n");
			this->ManualMove(phy_axis, dir_opt, 20, 3); //����Ѱ�Ҿ���׼
			m_n_ret_ref_step[phy_axis] = 6;  //��ת��һ��
			printf("return ref, goto step 6\n");

			break;
		case 6:  //�ȴ�����׼�ź�
		//	printf("return ref step 6\n");
			if(this->CheckAxisRefSignal(phy_axis)){
				this->ManualMoveStop(phy_axis);
				gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ�䣬��ʱ300ms
				m_n_ret_ref_step[phy_axis] = 7;  //��ת��һ��
			}else{
				this->ManualMove(phy_axis, dir_opt, 20, 3); //����Ѱ�Ҿ���׼
			}
			break;
		case 7:{ //�ȴ�ֹͣ��λ
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 200000){ //��ʱ200ms
//				m_n_get_cur_encoder_count = 0;  //��λ��ȡ��ǰ����������
				m_n_ret_ref_step[phy_axis] = 8;  //��ת��һ��
				printf("return ref, goto step 8\n");
			}
		}
			break;
		case 8:{ // ���õ�ǰλ��Ϊ�ο���
			printf("return ref step 8: send cmd to mi, set ref point\n");
			MiCmdFrame cmd;
			memset(&cmd, 0x00, sizeof(cmd));
			cmd.data.axis_index = phy_axis+1;
			cmd.data.cmd = CMD_MI_SET_REF_CUR;

			int64_t pos = m_p_axis_config[phy_axis].axis_home_pos[0] * 1e7;   //��λת����0.1nm
			memcpy(cmd.data.data, &pos, sizeof(int64_t));
			this->m_p_mi_comm->WriteCmd(cmd);
			gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ�䣬��ʱ300ms

			m_n_ret_ref_step[phy_axis] = 9;  //��ת��һ��
			printf("return ref, goto step 9\n");

		}
			break;
		case 9: { //�ȴ����òο�����ɣ���Miָ����Ӧ��������д���
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 200000){ //��ʱ200ms
				m_n_ret_ref_step[phy_axis] = 10;  //��ת��һ��
				printf("return ref, goto step 8\n");
			}
			}
			break;
		case 10:  //�زο������
			printf("axis %u return ref over\n", phy_axis);

			this->SetRetRefFlag(phy_axis, true);
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //��λ���ο����־

			this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
			m_n_ret_ref_step[phy_axis] = 0;

			if(m_n_mask_ret_ref == 0){
				this->m_b_ret_ref = false;
				this->m_b_ret_ref_auto = false;
				m_n_ret_ref_auto_cur = 0;
			}
			break;
		case 20: //ʧ�ܴ���
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
 * @brief ethcat���޻�׼��زο���
 */
void ChannelEngine::EcatAxisFindRefNoZeroSignal(uint8_t phy_axis){
	switch(this->m_n_ret_ref_step[phy_axis]){
		case 0: {// ����ԭ��ƫ�ƣ�����ԭ��
			printf("return ref step 0: send cmd to mi, set ref point\n");
			MiCmdFrame cmd;
			memset(&cmd, 0x00, sizeof(cmd));
			cmd.data.axis_index = phy_axis+1;
			cmd.data.cmd = CMD_MI_SET_REF_POINT;

			cmd.data.data[0] = 0x01;    //��ǰ����׼
			int64_t pos = m_p_axis_config[phy_axis].axis_home_pos[0]*1e7;   //��λת��,0.1nm
			memcpy(&cmd.data.data[1], &pos, sizeof(int64_t));
			cmd.data.data[5] = this->m_p_axis_config[phy_axis].ref_signal;
			cmd.data.data[6] = this->m_p_axis_config[phy_axis].ret_ref_dir;

			this->m_p_mi_comm->WriteCmd(cmd);

			
			m_n_ret_ref_step[phy_axis] = 1;  //��ת��һ��
			printf("return ref, goto step 1\n");

		}
			break;
		case 1:  //�ȴ����òο�����ɣ���Miָ����Ӧ��������д���

			break;

		case 10:{  //�ȴ����ƶ���λ,��ɣ�
#ifdef USES_RET_REF_TO_MACH_ZERO
			double dis = 0;    //��е��
#else
			double dis = this->m_p_axis_config[phy_axis].axis_home_pos[0];   // ԭ������
#endif
			this->SendMonitorData(false, false);  //�ٴζ�ȡʵʱλ��
			if(fabs(this->GetPhyAxisMachPosFeedback(phy_axis)- dis) <= 0.010){  //��λ
				m_n_ret_ref_step[phy_axis] = 11;  //��ת��һ��
				printf("return ref[%hhu, %lf, %lf], goto step 11\n", phy_axis, this->GetPhyAxisMachPosFeedback(phy_axis), dis);
			}
			break;
		}
		case 11:  //�زο������
			printf("axis %u return ref over\n", phy_axis);
			if(this->m_p_axis_config[phy_axis].feedback_mode == ABSOLUTE_ENCODER_YASAKAWA ||
				this->m_p_axis_config[phy_axis].feedback_mode == ABSOLUTE_ENCODER_PANASONIC ||
				this->m_p_axis_config[phy_axis].feedback_mode == LINEAR_ENCODER){  //����ֵ��Ҫ�������òο���ָ��
				MiCmdFrame mi_cmd;
				memset(&mi_cmd, 0x00, sizeof(mi_cmd));
				mi_cmd.data.cmd = CMD_MI_GET_ZERO_ENCODER;
				mi_cmd.data.axis_index = phy_axis+1;

				this->m_p_mi_comm->WriteCmd(mi_cmd);
			}
			this->SetRetRefFlag(phy_axis, true);
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<phy_axis);   //��λ���ο����־

			this->m_n_mask_ret_ref &= ~(0x01<<phy_axis);
			m_n_ret_ref_step[phy_axis] = 0;

			if(m_n_mask_ret_ref == 0){
				this->m_b_ret_ref = false;
				this->m_b_ret_ref_auto = false;
				m_n_ret_ref_auto_cur = 0;
			}
			break;
		case 20: //ʧ�ܴ���
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
 * @brief �زο���ִ�к���
 */
void ChannelEngine::ReturnRefPoint(){

//	int8_t dir = 0, dir_opt;   //�زο��㷽��
//	uint8_t ret_mode = 0;   //�زο��㷽ʽ
//	uint8_t chn = 0, chn_axis = 0;
//	double dis = 0;      //�ƶ�����
//	static double ret_ref_start_pos[kMaxAxisNum];
//	static uint8_t ret_ref_auto_cur = 0;    //�Զ��زο���ʱ����ǰ˳���
	int count = 0;
	for(uint i = 0; i < this->m_p_general_config->axis_count; i++){
		if((m_n_mask_ret_ref & (0x01<<i)) == 0 ||
				(m_p_axis_config[i].ret_ref_mode == 0 && m_p_axis_config[i].feedback_mode == INCREMENTAL_ENCODER)){  //��ֹ�زο���
			continue;
		}
		if(this->m_b_ret_ref_auto){
			if(m_p_axis_config[i].ret_ref_index > m_n_ret_ref_auto_cur)
				continue;
			count++;
		}

		if(this->m_p_axis_config[i].axis_interface == VIRTUAL_AXIS){// ������
			this->AxisFindRefNoZeroSignal(i);   // ֱ���������

		}else if(this->m_p_axis_config[i].axis_interface == BUS_AXIS){     // ������
       	    //���û����־
			this->SetInRetRefFlag(i, true);
			
			if(this->m_p_axis_config[i].feedback_mode == NO_ENCODER){   // ����������޷���
			    if(this->m_p_axis_config[i].ret_ref_mode == 1){
			         this->AxisFindRefWithZeroSignal(i); // ��������ź��������
				}else{
			    	this->AxisFindRefNoZeroSignal(i);  // ֱ���������
				}
       	    }else if(this->m_p_axis_config[i].ret_ref_mode == 1){
			    this->EcatAxisFindRefWithZeroSignal(i);    //  �����л�׼����
	   		}else if(this->m_p_axis_config[i].ret_ref_mode == 2){
				this->EcatAxisFindRefNoZeroSignal(i);    //  �����޻�׼����
			}else if(this->m_p_axis_config[i].ret_ref_mode == 4){  //����˫��׼���㣬�֡�����׼����IO�����ź�
				this->EcatAxisFindRefWithZeroSignal2(i);
			}else if(this->m_p_axis_config[i].ret_ref_mode == 0 &&
					(this->m_p_axis_config[i].feedback_mode == ABSOLUTE_ENCODER_YASAKAWA || this->m_p_axis_config[i].feedback_mode == ABSOLUTE_ENCODER_PANASONIC)){  //����ֵ���������ҽ�ֹ����
				this->AxisFindRefNoZeroSignal(i);   // ֱ���������
			}
       	}else if(this->m_p_axis_config[i].axis_interface == ANALOG_AXIS){   // ��������
       	    //���û����־
			this->SetInRetRefFlag(i, true);
			
			if(this->m_p_axis_config[i].feedback_mode == NO_ENCODER){   // ����������޷���
			    if(this->m_p_axis_config[i].ret_ref_mode == 1){
			         this->AxisFindRefWithZeroSignal(i); // ��������ź��������
				}
			    else{
			         this->AxisFindRefNoZeroSignal(i);  // ֱ���������
				}
       	    }
			else if(this->m_p_axis_config[i].ret_ref_mode == 1){
			    this->PulseAxisFindRefWithZeroSignal(i);    //  ����ģ�������л�׼����
	   		}
			else if(this->m_p_axis_config[i].ret_ref_mode == 2){
			    this->PulseAxisFindRefNoZeroSignal(i);    // ����ģ�������޻�׼����
			}
       	}

/*
		ret_mode = m_p_axis_config[i].ret_ref_mode;
		dir = this->m_p_axis_config[i].ret_ref_dir?DIR_POSITIVE:DIR_NEGATIVE;
		dir_opt = (m_p_axis_config[i].ret_ref_change_dir==0)?dir*-1:dir;  //�زο��㷽���෴�ķ���

		switch(this->m_n_ret_ref_step[i]){
		case 0://���ԭ���ź�
			printf("return ref step 0, dir = %hhd, dir_opt=%hhd\n", dir, dir_opt);
			this->SetRetRefFlag(i, false);   //��λ�زο�����ɱ�־
			this->m_p_pmc_reg->FReg().bits[0].in_ref_point &= ~(0x01<<i);

			if(this->CheckAxisRefBaseSignal(i, dir)){  //����1.2���ݾ�
				ret_ref_start_pos[i] = this->GetPhyAxisMachPosFeedback(i)+this->m_p_axis_config[i].move_pr*1.2;
				this->ManualMove(i, dir*-1,
						this->m_p_axis_config[i].ret_ref_speed, this->m_p_axis_config[i].move_pr*1.2);

				m_n_ret_ref_step[i] = 1;
			}
			else{
				m_n_ret_ref_step[i] = 2;
			}

			break;
		case 1://�ȴ����˵�λ�����ٴμ��ԭ���ź�
			printf("return ref step 1\n");
			chn = this->GetAxisChannel(i, chn_axis);
			if(chn != CHANNEL_ENGINE_INDEX){
				if(fabs(this->GetPhyAxisMachPosFeedback(i) - ret_ref_start_pos[i]) <= 0.001){
					if(CheckAxisRefBaseSignal(i, dir)){
						m_n_ret_ref_step[i] = 0;
						break;
					}else{
						if(ret_mode == 2)  //�޻�׼
							m_n_ret_ref_step[i] = 5;  //��ת��һ��
						else if(ret_mode == 1)   //�л�׼
							m_n_ret_ref_step[i] = 2;  //��ת��һ��
					}
				}
			}else if(this->m_p_axis_config[i].axis_pmc){  //PMC�ᵽλ
				if(this->m_n_run_axis_mask == 0 || (this->m_n_runover_axis_mask & (0x01L<<i))){
					if(CheckAxisRefBaseSignal(i, dir)){
						m_n_ret_ref_step[i] = 0;
						break;
					}else{
						if(ret_mode == 2)  //�޻�׼
							m_n_ret_ref_step[i] = 5;  //��ת��һ��
						else if(ret_mode == 1)   //�л�׼
							m_n_ret_ref_step[i] = 2;  //��ת��һ��
					}
				}
			}
			break;
		case 2:{   //��MI���Ϳ�ʼ�زο�������
	//		printf("return ref step 2\n");
			MiCmdFrame cmd;
			memset(&cmd, 0x00, sizeof(cmd));
			cmd.data.cmd = CMD_MI_START_RET_REF;

			cmd.data.axis_index = i+1;

			this->m_p_mi_comm->WriteCmd(cmd);
			m_n_ret_ref_step[i] = 3;  //��ת��һ��
			printf("return ref, goto step 3\n");
		}
			break;
		case 3:  //�Իزο����ٶ���زο��㷽���˶�
	//		printf("return ref step 3\n");
			this->ManualMove(i, dir, this->m_p_axis_config[i].ret_ref_speed, 90000.);
			m_n_ret_ref_step[i] = 4;  //��ת��һ��
			printf("return ref, goto step 4\n");
			break;
		case 4:  //�����ֻ�׼�źţ�ֹͣ
	//		printf("return ref step 4\n");
			if(this->CheckAxisRefBaseSignal(i, dir)){
				this->ManualMoveStop(i);
				gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ�䣬��ʱ200ms
				m_n_ret_ref_step[i] = 50;  //��ת��һ��
				printf("return ref, goto step 50\n");
			}
			break;
		case 50:{//�ȴ�ֹͣ��λ
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 300000){ //��ʱ300ms
				m_n_ret_ref_step[i] = 5;  //��ת��һ��
				printf("return ref, goto step 5\n");
			}
		}
			break;
		case 5:
			printf("return ref step 5\n");
			if(ret_mode == 1)//�л�׼�� ���ٻ������ֻ�׼�ź���ʧ, �ٶȣ�100mm/min
				this->ManualMove(i, dir_opt, 100, this->m_p_axis_config[i].move_pr*2);
			else if(ret_mode ==2){ //�޻�׼���زο��㷽���ƶ�1.1���ݾ�
				ret_ref_start_pos[i] = this->GetPhyAxisMachPosFeedback(i)+this->m_p_axis_config[i].move_pr*1.1;
				this->ManualMove(i, dir, m_p_axis_config[i].ret_ref_speed, this->m_p_axis_config[i].move_pr*1.1);
			}
			m_n_ret_ref_step[i] = 6;  //��ת��һ��
			printf("return ref, goto step 6\n");

			break;
		case 6:
		//	printf("return ref step 6\n");
			if(ret_mode == 1){//�л�׼���ȴ��ֻ�׼�ź���ʧ
				if(!this->CheckAxisRefBaseSignal(i, dir)){
					this->ManualMoveStop(i);

					gettimeofday(&this->m_time_ret_ref[phy_axis], NULL);   //��¼��ʼʱ�䣬��ʱ300ms

					m_n_ret_ref_step[i] = 70;  //��ת��һ��
				}
			}else if(ret_mode == 2){  //�޻�׼�� λ�õ�λ
				if(fabs(this->GetPhyAxisMachPosFeedback(i) - ret_ref_start_pos[i]) <= 0.001){
					m_n_ret_ref_step[i] = 7;  //��ת��һ��
				}
			}
			break;
		case 70:{ //�ȴ�ֹͣ��λ
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			unsigned int time_elpase = (time_now.tv_sec-m_time_ret_ref[phy_axis].tv_sec)*1000000+time_now.tv_usec-m_time_ret_ref[phy_axis].tv_usec;
			if(time_elpase >= 800000){ //��ʱ800ms
				m_n_get_cur_encoder_count = 0;  //��λ��ȡ��ǰ����������
				m_n_ret_ref_step[i] = 7;  //��ת��һ��
				printf("return ref, goto step 7\n");
			}
		}
			break;
		case 7:{ //��ȡ��ǰ������ֵ
			printf("return ref step 7: get_encoder_count = %hhu\n",
					this->m_n_get_cur_encoder_count);
			MiCmdFrame cmd;
			memset(&cmd, 0x00, sizeof(cmd));
			cmd.data.axis_index = i+1;

			if(m_p_axis_config[i].feedback_mode == NO_ENCODER){   //�޷����򽫵�ǰλ������Ϊ���
				cmd.data.cmd = CMD_MI_SET_REF_POINT;
			}else{
				cmd.data.cmd = CMD_MI_GET_CUR_ENCODER;   //�����������ȡ��ǰ��Ȧ����ֵ
			}


			this->m_p_mi_comm->WriteCmd(cmd);
			m_n_ret_ref_step[i] = 8;  //��ת��һ��
			printf("return ref, goto step 8\n");

		}
			break;
		case 8: //�ȴ����òο�����ɣ���Miָ����Ӧ��������д���


			break;
		case 9: //�ȴ�������ɣ��ƶ�����ο���1

			dis = this->m_p_axis_config[i].axis_home_pos[0]-this->GetPhyAxisMachPosFeedback(i);   //�ƶ�����
			if(dis >= 0)
				dir = DIR_POSITIVE;
			else
				dir = DIR_NEGATIVE;

			printf("axis %u dir = %hhd, dis = %lf, curpos = %lf\n", i, dir, dis, GetPhyAxisMachPosFeedback(i));

			this->ManualMove(i, dir, m_p_axis_config[i].ret_ref_speed, dis);
			m_n_ret_ref_step[i] = 10;  //��ת��һ��
			printf("return ref, goto step 10\n");
			break;
		case 10:  //�ȴ����ƶ���λ,��ɣ�

			if(fabs(this->GetPhyAxisMachPosFeedback(i)- m_p_axis_config[i].axis_home_pos[0]) <= 0.005){  //��λ
				m_n_ret_ref_step[i] = 11;  //��ת��һ��
			}
			break;
		case 11:  //�زο������
			printf("axis %u return ref over\n", i);
			if(this->m_p_axis_config[i].feedback_mode == ABSOLUTE_ENCODER_YASAKAWA ||
				this->m_p_axis_config[i].feedback_mode == ABSOLUTE_ENCODER_PANASONIC ||
				this->m_p_axis_config[i].feedback_mode == LINEAR_ENCODER){  //����ֵ��Ҫ�������òο���ָ��
				MiCmdFrame mi_cmd;
				memset(&mi_cmd, 0x00, sizeof(mi_cmd));
				mi_cmd.data.cmd = CMD_MI_SET_REF_CUR;
				mi_cmd.data.axis_index = i+1;
				int64_t pos = m_p_axis_config[i].axis_home_pos[0] * 1e7;   //��λת����0.1nm
			    memcpy(cmd.data.data, &pos, sizeof(int64_t));

				this->m_p_mi_comm->WriteCmd(mi_cmd);
			}else{
				this->SetRetRefFlag(i, true);
				this->m_p_pmc_reg->FReg().bits[0].in_ref_point |= (0x01<<i);   //��λ���ο����־
			}
//			printf("return ref over flag : 0x%llx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx, 0x%hhx\n", this->m_p_pmc_reg->FReg().bits[0].in_ref_point,
//					m_p_pmc_reg->FReg().all[200], m_p_pmc_reg->FReg().all[201], m_p_pmc_reg->FReg().all[202], m_p_pmc_reg->FReg().all[203],
//					m_p_pmc_reg->FReg().all[204], m_p_pmc_reg->FReg().all[205], m_p_pmc_reg->FReg().all[206], m_p_pmc_reg->FReg().all[207]);
			this->m_n_mask_ret_ref &= ~(0x01<<i);
			m_n_ret_ref_step[i] = 0;



//#ifdef USES_PMC_PROCESS
			//PMC��ָ��زο����������
			if(this->m_p_axis_config[i].axis_pmc > 0 &&
					this->m_pmc_axis_ctrl[m_p_axis_config[i].axis_pmc-1].GetCmdCount() > 0 &&
					!this->m_pmc_axis_ctrl[m_p_axis_config[i].axis_pmc-1].IsPaused()){
				m_pmc_axis_ctrl[m_p_axis_config[i].axis_pmc-1].ExecCmdOver(true);
			}

			if(i == 5)
				this->m_p_pmc_reg->FReg().bits[0].OUT8 = 0;  //pmc X����㸴λ
			else if(i == 6)
				this->m_p_pmc_reg->FReg().bits[0].OUT9 = 0;  //pmc Y����㸴λ
			else if(i == 7)
				this->m_p_pmc_reg->FReg().bits[0].OUT10 = 0;  //pmc ��Z����㸴λ
			else if(i == 8)
				this->m_p_pmc_reg->FReg().bits[0].OUT11 = 0;  //pmc ��Z����㸴λ
//#endif

			if(m_n_mask_ret_ref == 0){
				this->m_b_ret_ref = false;
				this->m_b_ret_ref_auto = false;
				m_n_ret_ref_auto_cur = 0;
			}
			break;
		case 20: //ʧ�ܴ���
			this->m_n_mask_ret_ref &= ~(0x01<<i);
			m_n_ret_ref_step[i] = 0;
//#ifdef USES_PMC_PROCESS
			//PMC��ָ��زο����������
			if(this->m_p_axis_config[i].axis_pmc > 0 &&
					this->m_pmc_axis_ctrl[m_p_axis_config[i].axis_pmc-1].GetCmdCount() > 0 &&
					!this->m_pmc_axis_ctrl[m_p_axis_config[i].axis_pmc-1].IsPaused()){
				m_pmc_axis_ctrl[m_p_axis_config[i].axis_pmc-1].ExecCmdOver(false);
			}

			if(i == 5)
				this->m_p_pmc_reg->FReg().bits[0].OUT8 = 0;  //pmc X����㸴λ
			else if(i == 6)
				this->m_p_pmc_reg->FReg().bits[0].OUT9 = 0;  //pmc Y����㸴λ
			else if(i == 7)
				this->m_p_pmc_reg->FReg().bits[0].OUT10 = 0;  //pmc ��Z����㸴λ
			else if(i == 8)
				this->m_p_pmc_reg->FReg().bits[0].OUT11 = 0;  //pmc ��Z����㸴λ
//#endif
			if(m_n_mask_ret_ref == 0){
				this->m_b_ret_ref = false;
				this->m_b_ret_ref_auto = false;
				m_n_ret_ref_auto_cur = 0;
			}
			m_error_code = ERR_RET_REF_FAILED;
			CreateError(ERR_RET_REF_FAILED, WARNING_LEVEL, CLEAR_BY_MCP_RESET, 0, CHANNEL_ENGINE_INDEX, i+1);
			break;
		default:
			break;
		}
		*/
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
			printf("�Զ������쳣��\n");
		}
	}
}

/**
 * @brief ������زο�����ɱ�־
 * @param phy_axis : ������ţ���0��ʼ
 * @param flag �� true--������ɱ�־    false--ȡ����ɱ�־
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
		if(m_p_axis_config[phy_axis].axis_interface == VIRTUAL_AXIS || m_p_axis_config[phy_axis].axis_type == AXIS_SPINDLE	//����������᲻�ûزο���
			|| (m_p_axis_config[phy_axis].feedback_mode == NO_ENCODER && m_p_axis_config[phy_axis].ret_ref_mode == 0)    //�޷��������ҽ�ֹ�زο���
			|| (m_p_axis_config[phy_axis].feedback_mode != INCREMENTAL_ENCODER && m_p_axis_config[phy_axis].feedback_mode != NO_ENCODER && m_p_axis_config[phy_axis].ref_encoder != kAxisRefNoDef)    //����ֵ�����������趨�ο���
			|| (m_p_axis_config[phy_axis].feedback_mode == INCREMENTAL_ENCODER && m_p_axis_config[phy_axis].ret_ref_mode == 0)){  //��������������ֹ�زο���
			return;   //���ûزο�������ֹ�������־��λ
		}
		this->m_n_mask_ret_ref_over &= ~(0x01<<phy_axis);

		if(sec < 8)
			m_p_pmc_reg->FReg().bits[chn].ZRF1 &= ~(0x01<<bit);
		else
			m_p_pmc_reg->FReg().bits[chn].ZRF2 &= ~(0x01<<bit);
	}

//	if(this->m_p_axis_config[phy_axis].axis_pmc == 0){//��PMC��    //PMC��Ҳ�ܼ��ص�ͨ��
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
 * @brief ��ȡ����������ͨ����ͨ������
 * @param phy_axis : ������ţ���0��ʼ
 * @param chn_axis[out] ��  ͨ����ţ���0��ʼ
 * @return ��������ͨ���ţ���0��ʼ,0xFF��ʾû������ͨ��
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
 * @brief ����tmpĿ¼�Ƿ���ڣ��������򴴽�
 */
void ChannelEngine::CheckTmpDir(){
	DIR *dir = opendir("/cnc/tmp/");   //����ʱĿ¼
	if(dir == nullptr){
		//����Ŀ¼
		if(mkdir("/cnc/tmp/", 0755) == -1){//����Ŀ¼ʧ��
			g_ptr_trace->PrintTrace(TRACE_ERROR, HMI_COMMUNICATION, "����Ŀ¼ʧ�ܣ�[/cnc/tmp/]");
		}else{
			g_ptr_trace->PrintTrace(TRACE_INFO, HMI_COMMUNICATION, "��Ŀ¼[/cnc/tmp/]ʧ�ܣ��Զ�������Ŀ¼��");
		}
	}else{
		closedir(dir);
	}
}


/**
 * @brief �����������
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

