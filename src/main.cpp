/**Copyright(C) 2020 ADX. All Rights Reserved.
 *Information in this file is the intellectual property of ADX, and may
 *contains trade secrets that must be stored and viewed confidentially.
 *
 *@file global_alg.h
 *@author gonghao
 *@date 2020/03/04
 *@brief 本源文件为SC软件模块的入口
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

//寄存器定义
#define MI_LOAD_BASE		(0x18000000)		//MI模块代码加载基地址, 512M内存
//#define MI_LOAD_BASE		(0x08000000)		//MI模块代码加载基地址, 256M内存

//寄存器读写宏定义函数，提高访问效率
#define WriteReg(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( p_addr_base + ((int32_t)x & reg_mem_map_mask));\
    *(virt_addr) = (int32_t)y;}


#define ReadReg(x,y) {volatile int32_t *virt_addr = (volatile int32_t*) ( p_addr_base + ((int32_t)x & reg_mem_map_mask));\
    y = *(virt_addr);}

//传输bit位，用于加载spartan6的代码
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


//获取当前文件的最后修改时间
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
 * @brief 初始化系统状态结构
 */
void InitSysState(){
    g_sys_state.system_ready = false;    //系统启动标志初始化为false
    g_sys_state.module_ready_mask = NONE_READY;  //模块就绪mask，初始化为均未就绪
    g_sys_state.system_quit = false;     //系统退出标志初始化为false
    g_sys_state.hmi_comm_ready = false;  //初始化HMI通讯连接状态
//	g_sys_state.hmi_ready = false;       //初始化HMI为非就绪状态
    g_sys_state.system_boot_stage = STEP_POWER_ON;    //初始化启动步骤为0
    g_sys_state.eth0_running = CheckNetState("eth0");  //初始化网络连接状态
    memset(g_sys_state.hmi_host_addr, 0x00, 16);       //
    memset(g_sys_state.local_host_addr, 0x00, 16);
    GetLocalIP("eth0", g_sys_state.local_host_addr);    //获取本地IP地址
}

/**
 * @brief 为SPARTAN6初始化加载代码,通过PL进行加载
 * @return
 */
int LoadSp6Data(){
    int res = ERR_NONE;
    struct stat statbuf;
    uint32_t file_size = 0;
    uint32_t send_size_total = 0, send_size_block = 0;
    uint32_t fifo_count = 0, check_res = 0;
    int wait_count = 0;   //等待计数
    const uint32_t reg_map_size = (uint32_t) 2*1024;   	//映射区大小 4K
    const uint32_t reg_mem_map_mask = (reg_map_size - 1);	 //地址掩码 0x07FF
    uint8_t *p_addr_base = (uint8_t *)MAP_FAILED;
    int mem_file = -1, fp = -1;
    Sp6Data data_buf;  //数据读入缓冲

    uint32_t pl_ver = 0;

    char file_path[kMaxPathLen];
    bzero(file_path, kMaxPathLen);
    strcpy(file_path, PATH_SPARTAN6_PROGRAM);

    bool bak_flag = false;  //是否使用备份版本

    LOAD:
    if(access(file_path, F_OK) == -1){	//文件不存在，告警
        g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data()，file do not exist[%s]!", file_path);
        res = ERR_LOAD_SP6;
        goto END;
    }

    printf("开始加载SPARTAN6...\n");
    if(stat(file_path, &statbuf) == 0)
        file_size = statbuf.st_size;  //获取文件总大小
    else{
        g_ptr_trace->PrintLog(LOG_ALARM, "获取文件[%s]大小失败！", file_path);
        res = ERR_LOAD_SP6;
        goto END;		//获取文件大小失败
    }

    //打开设备文件
    mem_file = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_file == -1) {
        g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data() 打开文件失败[/dev/mem]! errno = %d", errno);
        res = ERR_LOAD_SP6;
        goto END;
    }

    //映射内存地址，得到基地址
    p_addr_base = (uint8_t *) mmap(nullptr, reg_map_size, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem_file, MC_REGISTER_BASE & (~reg_mem_map_mask));
    if (p_addr_base == MAP_FAILED) {
        g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data() failed to map memory! errno = %d", errno);
        res = ERR_LOAD_SP6;
        goto END;
    }

    //读取PL版本号

    ReadReg(PL_PROG_VERSION, pl_ver);
    printf("PL program version:%u\n", pl_ver);


    WriteReg(SP6_INIT_DATA_WRITE, 0);  //初始化写入标志为0
    WriteReg(SP6_INIT_DATA_WR_TOTAL, 0);  //初始化写入标志为0

    fp = open(file_path, O_RDONLY); //只读打开文件
    if(fp < 0){
        g_ptr_trace->PrintLog(LOG_ALARM, "打开文件[%s]失败！errno = %d", file_path, errno);
        res = ERR_LOAD_SP6;
        goto END;//文件打开失败
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

        if(fifo_count >= kSp6FifoDepth && wait_count >= 50000){  //等待超时
            res = ERR_LOAD_SP6;
            g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data() 等待FIFO空闲区域超时!已写入字节数：%u, 总字节数：%u", send_size_total, file_size);
            goto END;
        }

        WriteReg(SP6_INIT_DATA_WRITE, 0);  //写入标志为0

        //写入16bytes数据
        memset(data_buf.buffer, 0x00, 16);
        send_size_block = read(fp, (void *)data_buf.buffer, 16);
        send_size_total += send_size_block;

        WriteReg(SP6_INIT_DATA(0), data_buf.data_32[0]);
        WriteReg(SP6_INIT_DATA(1), data_buf.data_32[1]);
        WriteReg(SP6_INIT_DATA(2), data_buf.data_32[2]);
        WriteReg(SP6_INIT_DATA(3), data_buf.data_32[3]);

        //写入完成标志
        WriteReg(SP6_INIT_DATA_WRITE, 0x01);
    }
    //写入所有数据完成标志
    WriteReg(SP6_INIT_DATA_WR_TOTAL, 0x01);
    usleep(200);


    //TODO 检查加载是否正确，如果校验出错就换用备份版本重新加载
    wait_count = 0;
    ReadReg(SP6_INIT_DATA_CHECK_RES, check_res);
    while(check_res == 0  && wait_count<10000){
        wait_count++;
        usleep(200);
        ReadReg(SP6_INIT_DATA_CHECK_RES, check_res);
    }

    if(check_res == 0 && wait_count >= 10000){//等待超时
        res = ERR_LOAD_SP6;
        g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data() 读取校验结果超时!");
        goto END;
    }else if(check_res == 1){	//校验失败
        g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data() 校验失败，使用备份文件恢复！");
        strcpy(file_path, PATH_SPARTAN6_PROGRAM_BAK);
        bak_flag = true;  //使用备份文件再次加载
        goto LOAD;
    }else if(check_res == 3){ //校验成功
        printf("Succeed to load spartan6!\n");
        if(bak_flag){
            if(0 != CopyFile(PATH_SPARTAN6_PROGRAM_BAK, PATH_SPARTAN6_PROGRAM)){
                g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data() 使用备份文件恢复失败！");
            }
        }
    }

    END:
    //取消内存映射
    if(p_addr_base != MAP_FAILED)
        munmap(p_addr_base, reg_map_size);

    //关闭设备文件
    if(mem_file != -1) {
        close(mem_file);
    }
    if(fp != -1){
        close(fp);
    }
    return res;
}


/**
 * @brief 为SPARTAN6初始化加载代码, 由PS直接加载
 * @return
 */
int LoadSp6Data_2(){
    int res = ERR_NONE;
    struct stat statbuf;
    int file_size = 0;
    int send_size_total = 0, send_size_block = 0;
    int fp = -1;
    char data_buf[1024];  //数据读入缓冲

    char file_path[kMaxPathLen];
    bzero(file_path, kMaxPathLen);
    strcpy(file_path, PATH_SPARTAN6_PROGRAM);

//	bool bak_flag = false;  //是否使用备份版本

    int i = 0;
    int fp_prog = 0, fp_init = 0, fp_done = 0, fp_data = 0, fp_clk = 0;

    //测试耗时
    struct timeval tvStart;
    struct timeval tvNow;
    unsigned int nTimeDelay = 0;

    if(access(file_path, F_OK) == -1){	//文件不存在，告警
        g_ptr_trace->PrintLog(LOG_ALARM, "LoadSp6Data()，file do not exist[%s]!", file_path);
        res = ERR_LOAD_SP6;
        goto END;
    }

    printf("开始加载SPARTAN6...\n");
    if(stat(file_path, &statbuf) == 0)
        file_size = statbuf.st_size;  //获取文件总大小
    else{
        g_ptr_trace->PrintLog(LOG_ALARM, "获取文件[%s]大小失败！", file_path);
        res = ERR_LOAD_SP6;
        goto END;		//获取文件大小失败
    }


    fp = open(file_path, O_RDONLY); //只读打开文件
    if(fp < 0){
        g_ptr_trace->PrintLog(LOG_ALARM, "打开文件[%s]失败！errno = %d", file_path, errno);
        res = ERR_LOAD_SP6;
        goto END;//文件打开失败
    }

    printf("设置输出模式\n");
    //设置输出模式
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

    //将PROGRAM和INIT拉低，维持1ms
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


    printf("开始数据传输\n");

    gettimeofday(&tvStart, NULL);

    //传输数据
    fp_data = open(SP6_DATA_VALUE, O_WRONLY);
    fp_clk = open(SP6_CLK_VALUE, O_WRONLY);
    while(send_size_total < file_size){
        //读取1024bytes数据
        memset(data_buf, 0x00, 1024);
        send_size_block = read(fp, (void *)data_buf, 1024);
        send_size_total += send_size_block;

        for(i = 0; i < send_size_block; i++){
//			for(j = 0; j < 8; j++){
//				//拉低时钟
//				lseek(fp_clk, 0, SEEK_SET);
//				write(fp_clk, "0", 2);
//				fsync(fp_clk);
//
//				//输出bit数据
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
//				//拉高时钟
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
    printf("数据传输完成,耗时：%u us\n", nTimeDelay);


    //TODO 检查加载是否正确，如果校验出错就换用备份版本重新加载
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


    //关闭设备文件
    close(fp);

    return res;
}

/**
 * @brief 为MI初始化加载代码
 * @return
 */
int LoadMiData(){
    int res = ERR_NONE;
    struct stat statbuf;
    uint32_t file_size = 0;
    uint32_t send_size_total = 0, send_size_block = 0;
    uint32_t reg_map_size = 0;   	//映射区大小
    uint32_t reg_mem_map_mask = 0;	 //地址掩码
    char *p_addr_base = (char *)MAP_FAILED;
    char *load_pos = nullptr;    //加载写入位置
    int mem_file = -1, fp = -1;
    char buf[1024];   //文件读取缓冲


    char file_path[kMaxPathLen];
    bzero(file_path, kMaxPathLen);
    strcpy(file_path, PATH_MI_PROGRAM);

    bool bak_flag = false;  //是否使用备份版本

    if(access(file_path, F_OK) == -1){	//文件不存在，告警
        if(!bak_flag){
            strcpy(file_path, PATH_MI_PROGRAM_BAK);//加载备份文件
            if(access(file_path, F_OK) == -1){	//文件不存在，告警
                g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "LoadSp6Data()，file do not exist[%s]!", file_path);
                res = ERR_LOAD_MI;
                goto END;
            }
            bak_flag = true;
        }else{
            g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "LoadSp6Data()，file do not exist[%s]!", file_path);
            res = ERR_LOAD_MI;
            goto END;
        }
    }

    printf("开始加载MI模块...\n");
    if(stat(file_path, &statbuf) == 0)
        file_size = statbuf.st_size;  //获取文件总大小
    else{
        g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "获取文件[%s]大小失败！", file_path);
        res = ERR_LOAD_MI;
        goto END;		//获取文件大小失败
    }
    reg_map_size = PAGE_SIZE * (file_size/PAGE_SIZE + 1);   //加载文件大小向上取整到页的整数倍
    reg_mem_map_mask = (reg_map_size - 1);


    //打开设备文件
    printf("映射内存地址空间\n");
    mem_file = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_file == -1) {
        g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "LoadMiData() 打开文件失败[/dev/mem]! errno = %d", errno);
        res = ERR_LOAD_MI;
        goto END;
    }

    //映射内存地址，得到基地址
    p_addr_base = (char *) mmap(nullptr, reg_map_size, PROT_READ | PROT_WRITE, MAP_SHARED,
            mem_file, MI_LOAD_BASE & (~reg_mem_map_mask));
    if (p_addr_base == MAP_FAILED) {
        g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "LoadMiData() failed to map memory! errno = %d", errno);
        res = ERR_LOAD_MI;
        goto END;
    }


    fp = open(file_path, O_RDONLY); //只读打开文件
    if(fp < 0){
        g_ptr_trace->PrintTrace(TRACE_ERROR, MAIN_ENTRANCE_SC, "打开文件[%s]失败！errno = %d", file_path, errno);
        res = ERR_LOAD_MI;
        goto END;//文件打开失败
    }

    load_pos = static_cast<char *>(p_addr_base);
    while(send_size_total < file_size){
        //写入16bytes数据
        memset(buf, 0x00, 1024);
        send_size_block = read(fp, (void *)buf, 1024);
        send_size_total += send_size_block;

        memcpy(load_pos, buf, send_size_block);
        load_pos += send_size_block;
    }

    printf("加载MI模块成功！\n");


    END:
    //取消内存映射
    if(p_addr_base != MAP_FAILED)
        munmap(p_addr_base, reg_map_size);

    //关闭设备文件
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
 * @brief 获取软件统一版本号
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
 * @brief 系统初始化函数
 */
int Initialize(){
    int res = ERR_NONE;

    InitSysState();
    memset(&g_sys_info, 0x00, sizeof(g_sys_info));  //初始化系统信息结构
    //sprintf(g_sys_info.sw_version_info.sc, "%s.%02d%02d%02d%02d%02d", ADX_SC_VERSION, YEAR-2000, MONTH, DAY, HOUR, MINUTE);
    sprintf(g_sys_info.sw_version_info.sc, "SC-00.00.02");
    strcpy(g_sys_info.sw_version_info.mc, "P0.0.0");
    strcpy(g_sys_info.sw_version_info.mi, "P0.0.0");

    GetDiskVersion(g_sys_info.sw_version_info.platform, kMaxVersionLen); //获取统一版本号
    g_sys_state.system_boot_stage = STEP_INIT_TRACE;



    //创建日志调试接口对象
    g_ptr_trace = TraceInfo::GetInstance();
    if(g_ptr_trace == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create TraceInfo object!\n");
        return res;
    }
    else
        printf("succeed to create TraceInfo object!\n");

    //加载spartan6代码
    g_sys_state.system_boot_stage = STEP_LOAD_SP6_DATA;
    //LoadSp6Data_2();

    //加载MI代码
    g_sys_state.system_boot_stage = STEP_LOAD_MI;
    LoadMiData();

    g_sys_state.system_boot_stage = STEP_INIT_PARM_MANAGER;
    //创建参数管理器，读取参数配置
    g_ptr_parm_manager = ParmManager::GetInstance();
    if(g_ptr_parm_manager == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create ParmManager object!\n");
        return res;
    }
    else
        printf("Succeed to create ParmManager object!\n");

    g_ptr_parm_manager->InitParm();

    g_ptr_trace->set_trace_level(g_ptr_parm_manager->GetSystemConfig()->trace_level);  //设置TRACE级别

    g_sys_state.system_boot_stage = STEP_INIT_HMI_COMM;
    //创建HMI通讯对象
    g_ptr_hmi_comm = HMICommunication::GetInstance();
    if(g_ptr_hmi_comm == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create HMICommunication object!\n");
        return res;
    }
    else
        printf("succeed to create HMICommunication object!\n");

    g_sys_state.system_boot_stage = STEP_INIT_MI_COMM;

    //创建MI通讯对象
    g_ptr_mi_comm = MICommunication::GetInstance();
    if(g_ptr_mi_comm == nullptr || g_ptr_mi_comm->GetErrorCode() != ERR_NONE){
        res = ERR_SC_INIT;
        printf("Failed to create MICommunication object!\n");
        return res;
    }
    else
        printf("succeed to create MICommunication object!\n");


    usleep(1000000);
    StartMi();  //启动MI

    g_sys_state.system_boot_stage = STEP_INIT_MC_COMM;
    //创建MC通讯对象
    g_ptr_mc_comm = MCCommunication::GetInstance();
    if(g_ptr_mc_comm == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create MCCommunication object!\n");
        return res;
    }
    else
        printf("succeed to create MCCommunication object!\n");

    //创建MC-ARM通讯对象
    g_ptr_mc_arm_comm = MCArmCommunication::GetInstance();
    if(g_ptr_mc_arm_comm == nullptr){
        res = ERR_SC_INIT;
        printf("Failed to create MCArmCommunication object!\n");
        return res;
    }
    else
        printf("succeed to create MCArmCommunication object!\n");

    g_sys_state.system_boot_stage = STEP_INIT_ALARM_PROC;
    //创建告警处理器模块
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
    //创建通道引擎
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
    g_ptr_alarm_processor->SetInterfaces();  //设置告警处理模块的接口
    g_ptr_tracelog_processor->SetInterfaces();

    g_ptr_chn_engine->SetMcArmComm(g_ptr_mc_arm_comm);
    g_ptr_chn_engine->Initialize(g_ptr_hmi_comm, g_ptr_mi_comm, g_ptr_mc_comm, g_ptr_parm_manager);

    //与MC进行握手
    g_ptr_chn_engine->ShakeHandWithMc();

#ifdef USES_WUXI_BLOOD_CHECK
    //创建辅助设备通讯接口
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


//程序退出处理函数
void ProgramQuit(int sig) {
    g_sys_state.system_quit = true;
}


//释放所有对象
void Clean(){
    printf("Enter Clean function\n");
    //释放通道引擎对象
    if(g_ptr_chn_engine != nullptr){
        delete g_ptr_chn_engine;
        g_ptr_chn_engine = nullptr;
    }

    printf("succeed to delete channel engine\n");

    //释放参数管理器对象
    if(g_ptr_parm_manager != nullptr){
        delete g_ptr_parm_manager;
        g_ptr_parm_manager = nullptr;
    }

    printf("succeed to delete parameter manager\n");

    //释放告警处理模块
    if(g_ptr_alarm_processor != nullptr){
        delete g_ptr_alarm_processor;
        g_ptr_alarm_processor = nullptr;
    }
    printf("succeed to delete alarm processor!\n");

    //释放MC通讯接口对象
    if(g_ptr_mc_comm != nullptr){
        delete g_ptr_mc_comm;
        g_ptr_mc_comm = nullptr;
    }
    printf("succeed to delete mc communication\n");

    //释放MC-ARM通讯接口对象
    if(g_ptr_mc_arm_comm != nullptr){
        delete g_ptr_mc_arm_comm;
        g_ptr_mc_arm_comm = nullptr;
    }
//	printf("succeed to delete mc-arm communication\n");

    //释放MI通讯接口对象
    if(g_ptr_mi_comm != nullptr){
        delete g_ptr_mi_comm;
        g_ptr_mi_comm = nullptr;
    }
    printf("succeed to delete mi communication\n");

    //释放HMI通讯接口对象
    if(g_ptr_hmi_comm != nullptr){
        delete g_ptr_hmi_comm;
        g_ptr_hmi_comm = nullptr;
    }
    printf("succeed to delete hmi communication\n");

    //释放日志调试接口对象
    if(g_ptr_trace != nullptr){
        delete g_ptr_trace;
        g_ptr_trace = nullptr;
    }

    printf("Exit Clean function\n");
}

/**
 * @brief 主程序入口函数
 * @return
 */
int main()
{
    int res = ERR_NONE;
    printf("Welcome to ARADEX 10MA CNC system! Version : [%s, %d-%d-%d]\n", ADX_SC_VERSION, YEAR,MONTH, DAY);

    printf("Enter the main loop, thread id = %ld!\n", syscall(SYS_gettid));

//	修改主线程优先级
//	pthread_attr_t attr;
//	struct sched_param param;
//	int policy;
//	pthread_attr_init(&attr);  //线程属性初始化
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
//	res = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); //不继承父线程调度方式，否则以上的设置不生效
//
//	pthread_attr_destroy(&attr);

    res = Initialize();

    signal(SIGINT, ProgramQuit);
    signal(SIGTERM, ProgramQuit);

    g_sys_state.system_boot_stage = STEP_ALL_DONE;
    g_sys_state.module_ready_mask |= SC_READY;  //SC模块就绪

    if((g_sys_state.module_ready_mask & NC_READY) == NC_READY)
        g_sys_state.system_ready = true;

    printf("sys = %d, chn = %d, axis = %d, errorinfo = %d, toolPotconfig=%d\n", sizeof(SCSystemConfig), sizeof(SCChannelConfig), sizeof(SCAxisConfig),
            sizeof(ErrorInfo), sizeof(HmiToolPotConfig));
    g_ptr_trace->PrintTrace(TRACE_INFO, MAIN_ENTRANCE_SC, "@#@#@Start SC Module!");

    //测试延时函数
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
//	int max_err[3][10] = {0}; 		//最大误差
//	int min_err[3][10] = {10000};	//最小误差
//	int total_err[3][10] = {0};   	//总误差
//	int avg_err[3][10] = {0};		//平均误差
//	int count = 0;
//	memset(&min_err[0][0], 10000, sizeof(int)*10*3);
    while (1) {
        if (g_sys_state.system_quit) {
            g_ptr_trace->PrintTrace(TRACE_INFO, MAIN_ENTRANCE_SC, "Break from the main dead loop!\n");
            break;//退出主线程
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

//		CheckNetState("eth0");  //检测网络状态

        g_ptr_chn_engine->DoIdle();

        usleep(100000);
    //	printf("main wait~~~~\n");
    }

    Clean();

    usleep(500000);    //等待

    g_ptr_trace->PrintTrace(TRACE_INFO, MAIN_ENTRANCE_SC, "Quit the SC Module!\n");
    return res;
}

//#define SET_SECTION8_LED _IO('x', 1) //设置8段管
//#define GET_SECTION8_LED _IO('x', 2) //获取8段管
//#define GET_ALARM_GPIO _IO('x', 3) //获取报警电平
//#define SET_NOTIFY_PID _IO('x', 4) //设置通知PID
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
//	ioctl(fd, SET_NOTIFY_PID, getpid()); //设置通知pid为自己
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
