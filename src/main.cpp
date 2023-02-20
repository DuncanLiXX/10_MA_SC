/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file global_alg.h
 *@author gonghao
 *@date 2020/03/04
 *@brief ��Դ�ļ�ΪSC���ģ������
 *@version
 */

#include "global_include.h"
#include "parm_manager.h"
#include "channel_engine.h"
#include "hmi_communication.h"
#include "mi_communication.h"
#include "mc_communication.h"
#include "mc_communication_arm.h"
#include "ad_communication.h"
#include "trace.h"
#include "alarm_processor.h"
#include "channel_data.h"

//�Ĵ�������
#define MI_LOAD_BASE		(0x18000000)		//MIģ�������ػ���ַ, 512M�ڴ�
//#define MI_LOAD_BASE		(0x08000000)		//MIģ�������ػ���ַ, 256M�ڴ�

//�Ĵ�����д�궨�庯������߷���Ч��
#define WriteReg(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( p_addr_base + ((int32_t)x & reg_mem_map_mask));\
    *(virt_addr) = (int32_t)y;}


#define ReadReg(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( p_addr_base + ((int32_t)x & reg_mem_map_mask));\
    y = *(virt_addr);}

//����bitλ�����ڼ���spartan6�Ĵ���
#define TRANS_BIT(x)  {lseek(fp_clk, 0, SEEK_SET);\
                        write(fp_clk, "0", 1);\
                        if(data_buf[i] & (0x80>>x)){\
                            lseek(fp_data, 0, SEEK_SET);\
                            write(fp_data, "1", 1);\
                        }else{\
                            lseek(fp_data, 0, SEEK_SET);\
                            write(fp_data, "0", 1);}\
                        lseek(fp_clk, 0, SEEK_SET);\
                        write(fp_clk, "1", 1);}


//��ȡ��ǰ�ļ�������޸�ʱ��
#define YEAR ((((__DATE__[7]-'0')*10+(__DATE__[8]-'0'))*10 +(__DATE__[9]-'0'))*10+(__DATE__[10]-'0'))
#define MONTH (__DATE__[2]=='n'?(__DATE__[1]=='a'?1:6) \
:__DATE__[2]=='b'?2 \
:__DATE__[2]=='r'?(__DATE__[0]=='M'?3:4) \
:__DATE__[2]=='y'?5 \
:__DATE__[2]=='l'?7 \
:__DATE__[2]=='g'?8 \
:__DATE__[2]=='p'?9 \
:__DATE__[2]=='t'?10 \
:__DATE__[2]=='v'?11:12)
#define DAY ((__DATE__[4]==' '?0:__DATE__[4]-'0')*10 + (__DATE__[5]-'0'))
#define DATE_AS_INT (((YEAR - 2000) * 12 + MONTH) * 31 + DAY)
#define HOUR ((__TIME__[0]-'0')*10+(__TIME__[1]-'0'))
#define MINUTE ((__TIME__[3]-'0')*10+(__TIME__[4]-'0'))
#define SECOND ((__TIME__[6]-'0')*10+(__TIME__[7]-'0'))
#define TIME_AS_INT (((((YEAR - 2000) * 12 + MONTH) * 31 + DAY)*12+HOUR)*60+MINUTE)*60+SECOND


/**
 * @brief ��ʼ��ϵͳ״̬�ṹ
 */
void InitSysState(){
    g_sys_state.system_ready = false;    //ϵͳ������־��ʼ��Ϊfalse
    g_sys_state.module_ready_mask = NONE_READY;  //ģ�����mask����ʼ��Ϊ��δ����
    g_sys_state.system_quit = false;     //ϵͳ�˳���־��ʼ��Ϊfalse
    g_sys_state.hmi_comm_ready = false;  //��ʼ��HMIͨѶ����״̬
//	g_sys_state.hmi_ready = false;       //��ʼ��HMIΪ�Ǿ���״̬
    g_sys_state.system_boot_stage = STEP_POWER_ON;    //��ʼ����������Ϊ0
    g_sys_state.eth0_running = CheckNetState("eth0");  //��ʼ����������״̬
    memset(g_sys_state.hmi_host_addr, 0x00, 16);       //
    memset(g_sys_state.local_host_addr, 0x00, 16);
    GetLocalIP("eth0", g_sys_state.local_host_addr);    //��ȡ����IP��ַ
}

/**
 * @brief ΪSPARTAN6��ʼ�����ش���,ͨ��PL���м���
 * @return
 */
int LoadSp6Data(){
    int res = ERR_NONE;
    struct stat statbuf;
    uint32_t file_size = 0;
    uint32_t send_size_total = 0, send_size_block = 0;
    uint32_t fifo_count = 0, check_res = 0;
    int wait_count = 0;   //�ȴ�����
    const uint32_t reg_map_size = (uint32_t) 2*1024;   	//ӳ������С 4K
    const uint32_t reg_mem_map_mask = (reg_map_size - 1);	 //��ַ���� 0x07FF
    uint8_t *p_addr_base = (uint8_t *)MAP_FAILED;
    int mem_file = -1, fp = -1;
    Sp6Data data_buf;  //���ݶ��뻺��

    uint32_t pl_ver = 0;

    char file_path[kMaxPathLen];
    bzero(file_path, kMaxPathLen);
    strcpy(file_path, PATH_SPARTAN6_PROGRAM);

    bool bak_flag = false;  //�Ƿ�ʹ�ñ��ݰ汾

    LOAD:
    if(access(file_path, F_OK) == -1){	//�ļ������ڣ��澯
        g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data()��file do not exist[%s]!", file_path);
        res = ERR_LOAD_SP6;
        goto END;
    }

    printf("��ʼ����SPARTAN6...\n");
    if(stat(file_path, &statbuf) == 0)
        file_size = statbuf.st_size;  //��ȡ�ļ��ܴ�С
    else{
        g_ptr_trace->PrintLog(LOG_ALARM, "��ȡ�ļ�[%s]��Сʧ�ܣ�", file_path);
        res = ERR_LOAD_SP6;
        goto END;		//��ȡ�ļ���Сʧ��
    }

    //���豸�ļ�
    mem_file = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_file == -1) {
        g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data() ���ļ�ʧ��[/dev/mem]! errno = %d", errno);
        res = ERR_LOAD_SP6;
        goto END;
    }

    //ӳ���ڴ��ַ���õ�����ַ
    p_addr_base = (uint8_t *) mmap(nullptr, reg_map_size, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem_file, MC_REGISTER_BASE & (~reg_mem_map_mask));
    if (p_addr_base == MAP_FAILED) {
        g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data() failed to map memory! errno = %d", errno);
        res = ERR_LOAD_SP6;
        goto END;
    }

    //��ȡPL�汾��

    ReadReg(PL_PROG_VERSION, pl_ver);
    printf("PL program version:%u\n", pl_ver);


    WriteReg(SP6_INIT_DATA_WRITE, 0);  //��ʼ��д���־Ϊ0
    WriteReg(SP6_INIT_DATA_WR_TOTAL, 0);  //��ʼ��д���־Ϊ0

    fp = open(file_path, O_RDONLY); //ֻ�����ļ�
    if(fp < 0){
        g_ptr_trace->PrintLog(LOG_ALARM, "���ļ�[%s]ʧ�ܣ�errno = %d", file_path, errno);
        res = ERR_LOAD_SP6;
        goto END;//�ļ���ʧ��
    }


    while(send_size_total < file_size){
        wait_count = 0;
        ReadReg(SP6_INIT_DATA_FIFO, fifo_count);
        while(fifo_count >= kSp6FifoDepth && wait_count<50000){
            wait_count++;
            usleep(5);
            ReadReg(SP6_INIT_DATA_FIFO, fifo_count);
        }
//		printf("sp6 fifo: %u\n", fifo_count);

        if(fifo_count >= kSp6FifoDepth && wait_count >= 50000){  //�ȴ���ʱ
            res = ERR_LOAD_SP6;
            g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data() �ȴ�FIFO��������ʱ!��д���ֽ�����%u, ���ֽ�����%u", send_size_total, file_size);
            goto END;
        }

        WriteReg(SP6_INIT_DATA_WRITE, 0);  //д���־Ϊ0

        //д��16bytes����
        memset(data_buf.buffer, 0x00, 16);
        send_size_block = read(fp, (void *)data_buf.buffer, 16);
        send_size_total += send_size_block;

        WriteReg(SP6_INIT_DATA(0), data_buf.data_32[0]);
        WriteReg(SP6_INIT_DATA(1), data_buf.data_32[1]);
        WriteReg(SP6_INIT_DATA(2), data_buf.data_32[2]);
        WriteReg(SP6_INIT_DATA(3), data_buf.data_32[3]);

        //д����ɱ�־
        WriteReg(SP6_INIT_DATA_WRITE, 0x01);
    }
    //д������������ɱ�־
    WriteReg(SP6_INIT_DATA_WR_TOTAL, 0x01);
    usleep(200);


    //TODO �������Ƿ���ȷ�����У�����ͻ��ñ��ݰ汾���¼���
    wait_count = 0;
    ReadReg(SP6_INIT_DATA_CHECK_RES, check_res);
    while(check_res == 0  && wait_count<10000){
        wait_count++;
        usleep(200);
        ReadReg(SP6_INIT_DATA_CHECK_RES, check_res);
    }

    if(check_res == 0 && wait_count >= 10000){//�ȴ���ʱ
        res = ERR_LOAD_SP6;
        g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data() ��ȡУ������ʱ!");
        goto END;
    }else if(check_res == 1){	//У��ʧ��
        g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data() У��ʧ�ܣ�ʹ�ñ����ļ��ָ���");
        strcpy(file_path, PATH_SPARTAN6_PROGRAM_BAK);
        bak_flag = true;  //ʹ�ñ����ļ��ٴμ���
        goto LOAD;
    }else if(check_res == 3){ //У��ɹ�
        printf("Succeed to load spartan6!\n");
        if(bak_flag){
            if(0 != CopyFile(PATH_SPARTAN6_PROGRAM_BAK, PATH_SPARTAN6_PROGRAM)){
                g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data() ʹ�ñ����ļ��ָ�ʧ�ܣ�");
            }
        }
    }

    END:
    //ȡ���ڴ�ӳ��
    if(p_addr_base != MAP_FAILED)
        munmap(p_addr_base, reg_map_size);

    //�ر��豸�ļ�
    if(mem_file != -1) {
        close(mem_file);
    }
    if(fp != -1){
        close(fp);
    }
    return res;
}


/**
 * @brief ΪSPARTAN6��ʼ�����ش���, ��PSֱ�Ӽ���
 * @return
 */
int LoadSp6Data_2(){
    int res = ERR_NONE;
    struct stat statbuf;
    int file_size = 0;
    int send_size_total = 0, send_size_block = 0;
    int fp = -1;
    char data_buf[1024];  //���ݶ��뻺��

    char file_path[kMaxPathLen];
    bzero(file_path, kMaxPathLen);
    strcpy(file_path, PATH_SPARTAN6_PROGRAM);

//	bool bak_flag = false;  //�Ƿ�ʹ�ñ��ݰ汾

    int i = 0;
    int fp_prog = 0, fp_init = 0, fp_done = 0, fp_data = 0, fp_clk = 0;

    //���Ժ�ʱ
    struct timeval tvStart;
    struct timeval tvNow;
    unsigned int nTimeDelay = 0;

    if(access(file_path, F_OK) == -1){	//�ļ������ڣ��澯
        g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data()��file do not exist[%s]!", file_path);
        res = ERR_LOAD_SP6;
        goto END;
    }

    printf("��ʼ����SPARTAN6...\n");
    if(stat(file_path, &statbuf) == 0)
        file_size = statbuf.st_size;  //��ȡ�ļ��ܴ�С
    else{
        g_ptr_trace->PrintLog(LOG_ALARM, "��ȡ�ļ�[%s]��Сʧ�ܣ�", file_path);
        res = ERR_LOAD_SP6;
        goto END;		//��ȡ�ļ���Сʧ��
    }


    fp = open(file_path, O_RDONLY); //ֻ�����ļ�
    if(fp < 0){
        g_ptr_trace->PrintLog(LOG_ALARM, "���ļ�[%s]ʧ�ܣ�errno = %d", file_path, errno);
        res = ERR_LOAD_SP6;
        goto END;//�ļ���ʧ��
    }

    printf("�������ģʽ\n");
    //�������ģʽ
    FILE * fpp;
    fpp = fopen("/sys/class/gpio/export", "w");
    if(fpp){
        fputs(SP6_CLK, fpp);
        fclose(fpp);
    }
    fpp = fopen(SP6_CLK_DIR, "w");
    if(fpp){
        fputs("out", fpp);
        fclose(fpp);
    }

    fpp = fopen("/sys/class/gpio/export", "w");
    if(fpp){
     fputs(SP6_PROGRAM, fpp);
     fclose(fpp);
    }
    fpp = fopen(SP6_PROG_DIR, "w");
    if(fpp){
        fputs("out", fpp);
        fclose(fpp);
    }

    fpp = fopen("/sys/class/gpio/export", "w");
    if(fpp){
        fputs(SP6_INIT, fpp);
        fclose(fpp);
    }
    fpp = fopen(SP6_INIT_DIR, "w");
    if(fpp){
        fputs("out", fpp);
        fclose(fpp);
    }

    fpp = fopen("/sys/class/gpio/export", "w");
    if(fpp){
        fputs(SP6_DATA, fpp);
        fclose(fpp);
    }
    fpp = fopen(SP6_DATA_DIR, "w");
    if(fpp){
        fputs("out", fpp);
        fclose(fpp);
    }

    fpp = fopen("/sys/class/gpio/export", "w");
    if(fpp){
        fputs(SP6_DONE, fpp);
        fclose(fpp);
    }
    fpp = fopen(SP6_DONE_DIR, "w");
    if(fpp){
        fputs("in", fpp);
        fclose(fpp);
    }

    //��PROGRAM��INIT���ͣ�ά��1ms
    fp_prog = open(SP6_PROG_VALUE, O_WRONLY);
    fp_init = open(SP6_INIT_VALUE, O_WRONLY);
    lseek(fp_prog, 0, SEEK_SET);
    lseek(fp_init, 0, SEEK_SET);
    write(fp_prog, "0", 2);
    fsync(fp_prog);
    write(fp_init, "0", 2);
    fsync(fp_init);
    usleep(1000);
    lseek(fp_prog, 0, SEEK_SET);
    lseek(fp_init, 0, SEEK_SET);
    write(fp_prog, "1", 2);
    fsync(fp_prog);
    write(fp_init, "1", 2);
    fsync(fp_init);
    close(fp_prog);
    close(fp_init);


    printf("��ʼ���ݴ���\n");

    gettimeofday(&tvStart, NULL);

    //��������
    fp_data = open(SP6_DATA_VALUE, O_WRONLY);
    fp_clk = open(SP6_CLK_VALUE, O_WRONLY);
    while(send_size_total < file_size){
        //��ȡ1024bytes����
        memset(data_buf, 0x00, 1024);
        send_size_block = read(fp, (void *)data_buf, 1024);
        send_size_total += send_size_block;

        for(i = 0; i < send_size_block; i++){
//			for(j = 0; j < 8; j++){
//				//����ʱ��
//				lseek(fp_clk, 0, SEEK_SET);
//				write(fp_clk, "0", 2);
//				fsync(fp_clk);
//
//				//���bit����
//				if(data_buf[i] & (0x80>>j)){
//					lseek(fp_data, 0, SEEK_SET);
//					write(fp_data, "1", 2);
//					fsync(fp_data);
//				}else{
//					lseek(fp_data, 0, SEEK_SET);
//					write(fp_data, "0", 2);
//					fsync(fp_data);
//				}
//
//				//����ʱ��
//				lseek(fp_clk, 0, SEEK_SET);
//				write(fp_clk, "1", 2);
//				fsync(fp_clk);
//			}
            TRANS_BIT(0);
            TRANS_BIT(1);
            TRANS_BIT(2);
            TRANS_BIT(3);
            TRANS_BIT(4);
            TRANS_BIT(5);
            TRANS_BIT(6);
            TRANS_BIT(7);
        }
    }
    close(fp_data);
    close(fp_clk);

    gettimeofday(&tvNow, NULL);
    nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;
    printf("���ݴ������,��ʱ��%u us\n", nTimeDelay);


    //TODO �������Ƿ���ȷ�����У�����ͻ��ñ��ݰ汾���¼���
    usleep(500000);
    fp_done = open(SP6_DONE_VALUE, O_RDONLY);
    memset(data_buf, 0x00, 1024);
    if(read(fp_done, data_buf, 3) < 0){
        printf("read gpio_done signal failed!\n");
    }else{
        if(atoi(data_buf) == 1){
            printf("Succeed to load spartan6 program\n");
        }
        else
            printf("Failed to load spartan6 program\n");
    }
    close(fp_done);


    END:


    //�ر��豸�ļ�
    close(fp);

    return res;
}

/**
 * @brief ΪMI��ʼ�����ش���
 * @return
 */
int LoadMiData(){
    int res = ERR_NONE;
    struct stat statbuf;
    uint32_t file_size = 0;
    uint32_t send_size_total = 0, send_size_block = 0;
    uint32_t reg_map_size = 0;   	//ӳ������С
    uint32_t reg_mem_map_mask = 0;	 //��ַ����
    char *p_addr_base = (char *)MAP_FAILED;
    char *load_pos = nullptr;    //����д��λ��
    int mem_file = -1, fp = -1;
    char buf[1024];   //�ļ���ȡ����


    char file_path[kMaxPathLen];
    bzero(file_path, kMaxPathLen);
    strcpy(file_path, PATH_MI_PROGRAM);

    bool bak_flag = false;  //�Ƿ�ʹ�ñ��ݰ汾

    if(access(file_path, F_OK) == -1){	//�ļ������ڣ��澯
        if(!bak_flag){
            strcpy(file_path, PATH_MI_PROGRAM_BAK);//���ر����ļ�
            if(access(file_path, F_OK) == -1){	//�ļ������ڣ��澯
                g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "LoadSp6Data()��file do not exist[%s]!", file_path);
                res = ERR_LOAD_MI;
                goto END;
            }
            bak_flag = true;
        }else{
            g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "LoadSp6Data()��file do not exist[%s]!", file_path);
            res = ERR_LOAD_MI;
            goto END;
        }
    }

    printf("��ʼ����MIģ��...\n");
    if(stat(file_path, &statbuf) == 0)
        file_size = statbuf.st_size;  //��ȡ�ļ��ܴ�С
    else{
        g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "��ȡ�ļ�[%s]��Сʧ�ܣ�", file_path);
        res = ERR_LOAD_MI;
        goto END;		//��ȡ�ļ���Сʧ��
    }
    reg_map_size = PAGE_SIZE * (file_size/PAGE_SIZE + 1);   //�����ļ���С����ȡ����ҳ��������
    reg_mem_map_mask = (reg_map_size - 1);


    //���豸�ļ�
    printf("ӳ���ڴ��ַ�ռ�\n");
    mem_file = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_file == -1) {
        g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "LoadMiData() ���ļ�ʧ��[/dev/mem]! errno = %d", errno);
        res = ERR_LOAD_MI;
        goto END;
    }

    //ӳ���ڴ��ַ���õ�����ַ
    p_addr_base = (char *) mmap(nullptr, reg_map_size, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem_file, MI_LOAD_BASE & (~reg_mem_map_mask));
    if (p_addr_base == MAP_FAILED) {
        g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "LoadMiData() failed to map memory! errno = %d", errno);
        res = ERR_LOAD_MI;
        goto END;
    }


    fp = open(file_path, O_RDONLY); //ֻ�����ļ�
    if(fp < 0){
        g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "���ļ�[%s]ʧ�ܣ�errno = %d", file_path, errno);
        res = ERR_LOAD_MI;
        goto END;//�ļ���ʧ��
    }

    load_pos = static_cast<char *>(p_addr_base);
    while(send_size_total < file_size){
        //д��16bytes����
        memset(buf, 0x00, 1024);
        send_size_block = read(fp, (void *)buf, 1024);
        send_size_total += send_size_block;

        memcpy(load_pos, buf, send_size_block);
        load_pos += send_size_block;
    }

    printf("����MIģ��ɹ���\n");


    END:
    //ȡ���ڴ�ӳ��
    if(p_addr_base != MAP_FAILED)
        munmap(p_addr_base, reg_map_size);

    //�ر��豸�ļ�
    if(mem_file != -1) {
        close(mem_file);
    }
    if(fp != -1){
        close(fp);
    }
    sync();
    return res;
}

/**
 * @brief ��ȡ���ͳһ�汾��
 */
int GetDiskVersion(char* buf, int len)
{
    FILE* fp = fopen("/cnc/disk/version.txt", "rb");
    if (!fp) {
        strcpy(buf, "V1.0.0");
        return -1;
    }
    int readlen = fread(buf, 1, len, fp);
    if (readlen > 0)
        buf[readlen-1] = 0;
    else {
        strcpy(buf, "V1.0.0");
        fclose(fp);
        return -2;
    }

    for (int i=0; i<readlen; i++) {
        if (buf[i]==10 || buf[i]==13) {
            buf[i] = 0;
            break;
        }
    }
    fclose(fp);
    printf("Disk Version %s\n", buf);
    return 0;
}

/**
 * @brief ϵͳ��ʼ������
 */
int Initialize(){
    int res = ERR_NONE;

    InitSysState();
    memset(&g_sys_info, 0x00, sizeof(g_sys_info));  //��ʼ��ϵͳ��Ϣ�ṹ
    //sprintf(g_sys_info.sw_version_info.sc, "%s.%02d%02d%02d%02d%02d", ADX_SC_VERSION, YEAR-2000, MONTH, DAY, HOUR, MINUTE);
    sprintf(g_sys_info.sw_version_info.sc, "SC-00.00.02");
    strcpy(g_sys_info.sw_version_info.mc, "P0.0.0");
    strcpy(g_sys_info.sw_version_info.mi, "P0.0.0");

    GetDiskVersion(g_sys_info.sw_version_info.platform, kMaxVersionLen); //��ȡͳһ�汾��
    g_sys_state.system_boot_stage = STEP_INIT_TRACE;



    //������־���Խӿڶ���
    g_ptr_trace = TraceInfo::GetInstance();
    if(g_ptr_trace == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create TraceInfo object!\n");
        return res;
    }
    else
        printf("succeed to create TraceInfo object!\n");

    //����spartan6����
    g_sys_state.system_boot_stage = STEP_LOAD_SP6_DATA;
    //LoadSp6Data_2();

    //����MI����
    g_sys_state.system_boot_stage = STEP_LOAD_MI;
    LoadMiData();

    g_sys_state.system_boot_stage = STEP_INIT_PARM_MANAGER;
    //������������������ȡ��������
    g_ptr_parm_manager = ParmManager::GetInstance();
    if(g_ptr_parm_manager == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create ParmManager object!\n");
        return res;
    }
    else
        printf("Succeed to create ParmManager object!\n");

    g_ptr_parm_manager->InitParm();

    g_ptr_trace->set_trace_level(g_ptr_parm_manager->GetSystemConfig()->trace_level);  //����TRACE����

    g_sys_state.system_boot_stage = STEP_INIT_HMI_COMM;
    //����HMIͨѶ����
    g_ptr_hmi_comm = HMICommunication::GetInstance();
    if(g_ptr_hmi_comm == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create HMICommunication object!\n");
        return res;
    }
    else
        printf("succeed to create HMICommunication object!\n");

    g_sys_state.system_boot_stage = STEP_INIT_MI_COMM;

    //����MIͨѶ����
    g_ptr_mi_comm = MICommunication::GetInstance();
    if(g_ptr_mi_comm == nullptr || g_ptr_mi_comm->GetErrorCode() != ERR_NONE){
        res = ERR_SC_INIT;
        printf("Failed to create MICommunication object!\n");
        return res;
    }
    else
        printf("succeed to create MICommunication object!\n");


    usleep(1000000);
    StartMi();  //����MI

    g_sys_state.system_boot_stage = STEP_INIT_MC_COMM;
    //����MCͨѶ����
    g_ptr_mc_comm = MCCommunication::GetInstance();
    if(g_ptr_mc_comm == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create MCCommunication object!\n");
        return res;
    }
    else
        printf("succeed to create MCCommunication object!\n");

    //����MC-ARMͨѶ����
    g_ptr_mc_arm_comm = MCArmCommunication::GetInstance();
    if(g_ptr_mc_arm_comm == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create MCArmCommunication object!\n");
        return res;
    }
    else
        printf("succeed to create MCArmCommunication object!\n");

    g_sys_state.system_boot_stage = STEP_INIT_ALARM_PROC;
    //�����澯������ģ��
    g_ptr_alarm_processor = AlarmProcessor::GetInstance();
    if(g_ptr_alarm_processor == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create AlarmProcessor object!\n");
        return res;
    }
    else
        printf("Succeed to create AlarmProcessor object!\n");

    g_ptr_tracelog_processor = TraceLogProcess::GetInstance();
    if (g_ptr_tracelog_processor == nullptr) {
        res = ERR_SC_INIT;
        printf("Failed to create TraceLogProcess object!\n");
        return res;
    }
    else
        printf("Succeed to create TraceLogProcess object!\n");


    g_sys_state.system_boot_stage = STEP_INIT_CHN_ENGINER;
    //����ͨ������
    g_ptr_chn_engine = ChannelEngine::GetInstance();
    if(g_ptr_chn_engine == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create ChannelEngine object!\n");
        return res;
    }
    else
        printf("Succeed to create ChannelEngine object!\n");


    g_ptr_hmi_comm->SetInterface();  //
    g_ptr_mi_comm->SetInterface();
    g_ptr_mc_comm->SetInterface();
    g_ptr_mc_arm_comm->SetInterface();
    g_ptr_alarm_processor->SetInterfaces();  //���ø澯����ģ��Ľӿ�
    g_ptr_tracelog_processor->SetInterfaces();

    g_ptr_chn_engine->SetMcArmComm(g_ptr_mc_arm_comm);
    g_ptr_chn_engine->Initialize(g_ptr_hmi_comm, g_ptr_mi_comm, g_ptr_mc_comm, g_ptr_parm_manager);

    //��MC��������
    g_ptr_chn_engine->ShakeHandWithMc();

#ifdef USES_WUXI_BLOOD_CHECK
    //���������豸ͨѶ�ӿ�
    g_ptr_ad_comm = ADCommunication::GetInstance();
    if(g_ptr_ad_comm == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create ADCommunication object!\n");
        return res;
    }
    else
        printf("Succeed to create ADCommunication object!\n");
#endif


//	g_sys_state.system_ready = true; //for test

    return res;
}


//�����˳�������
void ProgramQuit(int sig) {
    g_sys_state.system_quit = true;
}


//�ͷ����ж���
void Clean(){
    printf("Enter Clean function\n");
    //�ͷ�ͨ���������
    if(g_ptr_chn_engine != nullptr){
        delete g_ptr_chn_engine;
        g_ptr_chn_engine = nullptr;
    }

    printf("succeed to delete channel engine\n");

    //�ͷŲ�������������
    if(g_ptr_parm_manager != nullptr){
        delete g_ptr_parm_manager;
        g_ptr_parm_manager = nullptr;
    }

    printf("succeed to delete parameter manager\n");

    //�ͷŸ澯����ģ��
    if(g_ptr_alarm_processor != nullptr){
        delete g_ptr_alarm_processor;
        g_ptr_alarm_processor = nullptr;
    }
    printf("succeed to delete alarm processor!\n");

    //�ͷ�MCͨѶ�ӿڶ���
    if(g_ptr_mc_comm != nullptr){
        delete g_ptr_mc_comm;
        g_ptr_mc_comm = nullptr;
    }
    printf("succeed to delete mc communication\n");

    //�ͷ�MC-ARMͨѶ�ӿڶ���
    if(g_ptr_mc_arm_comm != nullptr){
        delete g_ptr_mc_arm_comm;
        g_ptr_mc_arm_comm = nullptr;
    }
//	printf("succeed to delete mc-arm communication\n");

    //�ͷ�MIͨѶ�ӿڶ���
    if(g_ptr_mi_comm != nullptr){
        delete g_ptr_mi_comm;
        g_ptr_mi_comm = nullptr;
    }
    printf("succeed to delete mi communication\n");

    //�ͷ�HMIͨѶ�ӿڶ���
    if(g_ptr_hmi_comm != nullptr){
        delete g_ptr_hmi_comm;
        g_ptr_hmi_comm = nullptr;
    }
    printf("succeed to delete hmi communication\n");

    //�ͷ���־���Խӿڶ���
    if(g_ptr_trace != nullptr){
        delete g_ptr_trace;
        g_ptr_trace = nullptr;
    }

    printf("Exit Clean function\n");
}

/**
 * @brief ��������ں���
 * @return
 */
int main()
{
    int res = ERR_NONE;
    printf("Welcome to ARADEX 10MA CNC system! Version : [%s, %d-%d-%d]\n", ADX_SC_VERSION, YEAR,MONTH, DAY);

    printf("Enter the main loop, thread id = %ld!\n", syscall(SYS_gettid));

//	�޸����߳����ȼ�
//	pthread_attr_t attr;
//	struct sched_param param;
//	int policy;
//	pthread_attr_init(&attr);  //�߳����Գ�ʼ��
//	pthread_attr_getschedparam(&attr, &param);
//	pthread_attr_getschedpolicy(&attr, &policy);
//	pthread_attr_setschedpolicy(&attr, SCHED_RR);
//
//
//
//	printf("main thread policy: %d, priority: %d\n", policy, param.__sched_priority);
//	param.__sched_priority = 96;
//	pthread_attr_setschedparam(&attr, &param);
//
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //���̳и��̵߳��ȷ�ʽ���������ϵ����ò���Ч
//
//	pthread_attr_destroy(&attr);

    res = Initialize();

    signal(SIGINT, ProgramQuit);
    signal(SIGTERM, ProgramQuit);

    g_sys_state.system_boot_stage = STEP_ALL_DONE;
    g_sys_state.module_ready_mask |= SC_READY;  //SCģ�����

    if((g_sys_state.module_ready_mask & NC_READY) == NC_READY)
        g_sys_state.system_ready = true;

    printf("sys = %d, chn = %d, axis = %d, errorinfo = %d, toolPotconfig=%d\n", sizeof(SCSystemConfig), sizeof(SCChannelConfig), sizeof(SCAxisConfig),
            sizeof(ErrorInfo), sizeof(HmiToolPotConfig));
    g_ptr_trace->PrintTrace(TRACE_INFO, MAIN_ENTRANCE_SC, "@#@#@Start SC Module!");

    //������ʱ����
//	struct timeval tvStart;
//	struct timeval tvNow;
//	unsigned int nTimeDelay = 0;
//    struct timespec req;
//    struct timeval tv;
//	unsigned int delay[10] = {500000, 100000, 50000, 10000, 5000, 1000, 500, 100, 10, 1};
//	int nError = 0;
//	int ret = 0;
// //	fd_set rfds;
// //	int fd = 1;
//	int max_err[3][10] = {0}; 		//������
//	int min_err[3][10] = {10000};	//��С���
//	int total_err[3][10] = {0};   	//�����
//	int avg_err[3][10] = {0};		//ƽ�����
//	int count = 0;
//	memset(&min_err[0][0], 10000, sizeof(int)*10*3);
    while (1) {
        if (g_sys_state.system_quit) {
            g_ptr_trace->PrintTrace(TRACE_INFO, MAIN_ENTRANCE_SC, "Break from the main dead loop!\n");
            break;//�˳����߳�
        }
//		for(int i = 0; i < 10; i++){
//			//test usleep()
//			gettimeofday(&tvStart, NULL);
//			ret = usleep(delay[i]);
// //			req.tv_sec = delay[i]/1000000;
// //			req.tv_nsec = (delay[i]%1000000)*1000;
// //			ret = nanosleep(&req, NULL);
// //			FD_ZERO(&rfds);
// //			FD_SET(fd, &rfds);
// //			tv.tv_sec = 0;
// //			tv.tv_usec = delay[i];
// //			ret = select(0, NULL, NULL, NULL, &tv);
// //			if(-1 == ret){
// //				printf("delay error!");
// //			}
//			gettimeofday(&tvNow, NULL);
//			nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;
//			nError = nTimeDelay - delay[i];
//			if(nError < min_err[0][i])
//				min_err[0][i] = nError;
//			if(nError > max_err[0][i])
//				max_err[0][i] = nError;
//			total_err[0][i] += nError;
//			printf("select = %u, act = %u, error = %d\n", delay[i], nTimeDelay, nError);
//
//			//test nanosleep()
//			gettimeofday(&tvStart, NULL);
//			req.tv_sec = delay[i]/1000000;
//			req.tv_nsec = (delay[i]%1000000)*1000;
//			ret = nanosleep(&req, NULL);
//			gettimeofday(&tvNow, NULL);
//			nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;
//			nError = nTimeDelay - delay[i];
//			if(nError < min_err[1][i])
//				min_err[1][i] = nError;
//			if(nError > max_err[1][i])
//				max_err[1][i] = nError;
//			total_err[1][i] += nError;
//
//			//test select()
//			gettimeofday(&tvStart, NULL);
//			tv.tv_sec = 0;
//			tv.tv_usec = delay[i];
//			ret = select(0, NULL, NULL, NULL, &tv);
//			if(-1 == ret){
//				printf("delay error!");
//			}
//			gettimeofday(&tvNow, NULL);
//			nTimeDelay = (tvNow.tv_sec-tvStart.tv_sec)*1000000+tvNow.tv_usec-tvStart.tv_usec;
//			nError = nTimeDelay - delay[i];
//			if(nError < min_err[2][i])
//				min_err[2][i] = nError;
//			if(nError > max_err[2][i])
//				max_err[2][i] = nError;
//			total_err[2][i] += nError;
//		}
//
//		if(++count == 50)
//		{
//			for(int i = 0; i < 10; i++){
//				avg_err[0][i] = total_err[0][i]/count;
//				avg_err[1][i] = total_err[1][i]/count;
//				avg_err[2][i] = total_err[2][i]/count;
//				printf("%d  :   usleep = %u, nanosleep = %u, select = %u \n", delay[i], avg_err[0][i], avg_err[1][i], avg_err[2][i]);
//				printf("max: usleep = %u, nanosleep = %u, select = %u \n", max_err[0][i], max_err[1][i], max_err[2][i]);
//				printf("min: usleep = %u, nanosleep = %u, select = %u \n", min_err[0][i], min_err[1][i], min_err[2][i]);
//			}
//
//			break;
//		}

//		CheckNetState("eth0");  //�������״̬

        g_ptr_chn_engine->DoIdle();

        usleep(100000);
    //	printf("main wait~~~~\n");
    }

    Clean();

    usleep(500000);    //�ȴ�

    g_ptr_trace->PrintTrace(TRACE_INFO, MAIN_ENTRANCE_SC, "Quit the SC Module!\n");
    return res;
}

//#define SET_SECTION8_LED _IO('x', 1) //����8�ι�
//#define GET_SECTION8_LED _IO('x', 2) //��ȡ8�ι�
//#define GET_ALARM_GPIO _IO('x', 3) //��ȡ������ƽ
//#define SET_NOTIFY_PID _IO('x', 4) //����֪ͨPID
//
//void alarm_signal_handler(int signo, siginfo_t *info, void *context)
//{
//	printf("sigcode=%d\n", info->si_code);
//}
//
//void connect_signal_handler(void)
//{
//	struct sigaction sa;
//
//	sa.sa_handler = (void (*)(int))alarm_signal_handler;
//	sigemptyset(&sa.sa_mask);
//	sa.sa_flags = SA_SIGINFO;
//	int ret = sigaction(SIGUSR1, &sa, NULL);
//	if (ret == -1) {
//		printf("Failed to caught signal!\n");
//	}
//}
//
//int main()
//{
//	int fd;
//	unsigned char value;
//
//	connect_signal_handler();
//
//	fd = open("/dev/aradex_gpio", O_RDWR);
//	if (fd < 0) {
//		printf("Can't open /dev/aradex_gpio!\n");
//		return -1;
//	}
//
//	ioctl(fd, SET_NOTIFY_PID, getpid()); //����֪ͨpidΪ�Լ�
//
//	for (int i=0; i<8; i++) {
//		ioctl(fd, SET_SECTION8_LED, 1<<i);
//		sleep(1);
//	}
//
//	ioctl(fd, GET_SECTION8_LED, &value);
//	printf("Read led is %x\n", value);
//
//	ioctl(fd, GET_ALARM_GPIO, &value);
//	printf("Read alarm is %x\n", value);
//
//	close(fd);
//
//
//	printf("start loop\n");
//
//	while(1)
//		usleep(300000);
//
//	return 0;
//}

//#include <stdio.h>
//		#include <unistd.h>
//
//		//906+
//
//		int main()
//		{
//			FILE * fp;
//			fp = fopen("/sys/class/gpio/export", "w");
//			fputs("921", fp);
//			fclose(fp);
//			fp = fopen("/sys/class/gpio/gpio921/direction", "w");
//			fputs("out", fp);
//			fclose(fp);
//
//			int ff = open("/sys/class/gpio/gpio921/value", O_WRONLY);
//			while (1)
//		   	{
//				lseek(ff, 0, SEEK_SET);
//				write(ff, "1", 2);
//				fsync(ff);
//				usleep(500000);
//				lseek(ff, 0, SEEK_SET);
//				write(ff, "0", 2);
//
//				fsync(ff);
//				usleep(500000);
//			}
//			close(ff);
//
//			return 0;
//		}
