#ifndef SHOWSC_H
#define SHOWSC_H

#include <set>
#include <mutex>
#include "singleton.h"
#include "global_include.h"

class ChannelControl;
class SpindleControl;
class SCSystemConfig;
class SCChannelConfig;
class SCAxisConfig;
class SCCoordConfig;
class SCToolOffsetConfig;
class SCToolPotConfig;
class FiveAxisConfig;
class FRegBits;
class GRegBits;
class TraceInfo;
class PmcAxisCtrl;
class SyncAxisCtrl;

struct ChannelStatusCollect;
struct ChannelRealtimeStatus;
struct ChannelMcStatus;
struct SCFiveAxisV2Config;

void ScPrintf(const char * fmt,...);
void ScPrintfTime(const char *s);

const std::string SwitchTopic = "/sc/switch";  // ��ӡ����ѡ��
const std::string MarcoSelect = "/sc/marco/select"; //��ѡ��
const std::string PrintTopic = "/sc/print";    // ��ӡ���

class ShowSc{
public:

    ShowSc();
    ~ShowSc();

    // ���ô�ӡ��� 0����ֻ��ӡһ��
    void SetInterval(int ms){interval = ms;}

    // ������Ҫ��ӡ���������ͣ���ӡ�̻߳������������ӡ
    void SetPrintType(PrintType type);
    PrintType GetPrintType(){return print_type;}
    void MarcoSelect(const std::string &content);    // ��ӡ��ѡ��

    // ��Ӹ�����
    void AddComponent(ChannelStatusCollect *p){chn_status = p;}
    void AddComponent(ChannelRealtimeStatus *p){chn_rt_status = p;}
    void AddComponent(ChannelMcStatus *p){mc_status = p;}
    void AddComponent(SpindleControl *p){spindle = p;}
    void AddComponent(SCSystemConfig *p){sys_config = p;}
    void AddComponent(SCChannelConfig *p){chn_config = p;}
    void AddComponent(SCAxisConfig *p){axis_config = p;}
    void AddComponent(SCCoordConfig *p1,
                      SCCoordConfig *p2,
                      SCCoordConfig *p3){
        coord = p1;
        ex_coord = p2;
        global_coord = p3;
    }
    void AddComponent(SCToolOffsetConfig *p){tool_offset = p;}
    void AddComponent(SCToolPotConfig *p){tool_pot = p;}
    void AddComponent(FiveAxisConfig *p){fiveaxis_config = p;}
    void AddComponent(FRegBits *p){F = p;}
    void AddComponent(const GRegBits *p){G = p;}
    void AddComponent(PmcAxisCtrl *p){pmc_axis_ctrl = p;}
    void AddComponent(SyncAxisCtrl *p){sync_axis_ctrl = p;}
    void AddComponent(SCFiveAxisV2Config *p){fiveaxisV2_config = p;}

    // ������Ϣ
    void SendMsg(string &s);
    void NewMsg(){msg_new_count++;}
    void DeleteMsg(){msg_delete_count++;}

private:
    // ��ӡ�����߳�
    void ProcessPrintThread();

    // key��������
    void KeyFormat(string &key);
    // ����key��value����ʽ����׷�ӵ�s��
    void AddPair(string &s,string key,uint64_t value);
    void AddPair(string &s,string key,int64_t value);
    void AddPair(string &s,string key,uint32_t value);
    void AddPair(string &s,string key,int32_t value);
    void AddPair(string &s,string key,uint16_t value);
    void AddPair(string &s,string key,int16_t value);
    void AddPair(string &s,string key,uint8_t value);
    void AddPair(string &s,string key,int8_t value);
    void AddPair(string &s,string key,double value);
    void AddPair(string &s,string key,string value);

    // ���ݴ�ӡ���ͷֱ���

    void PrintChnStatus();  // ��ӡͨ��״̬
    void PrintMcStatus();   // ��ӡMC״̬
    void PrintRealtimeData();   // ��ӡʵʱ����
    void PrintSpindle();    // ��ӡ����״̬
    void PrintSysConfig();  // ��ӡϵͳ����
    void PrintChnConfig();  // ��ӡͨ������
    void PrintAxisConfig(int axis); // ��ӡ������ axis��0��ʼ
    void PrintCoordConfig();    // ��ӡ��������ϵ����
    void PrintExCoordConfig();  // ��ӡ��չ��������ϵ����
    void PrintGrbCoordConfig(); // ��ӡȫ�ֹ�������ϵ����
    void PrintToolOffsetConfig();   // ��ӡ����ƫ������
    void PrintToolPotConfig();  // ��ӡ������Ϣ����
    void PrintFiveAxisCoinfig();    // ��ӡ�����������
    void PrintMode();   // ��ӡģ̬
    void PrintWarning();    // ��ӡ����
    void PrintFRegState();  // ��ӡF�Ĵ���
    void PrintGRegState();  // ��ӡG�Ĵ���
    void PrintPmcAxisCtrl();    // ��ӡPMC����Ϣ
    void PrintSyncAxisCtrl();   // ��ӡͬ������Ϣ
    void PrintMarcoValue();     // ��ӡ�����
    void PrintFiveAxisV2Config(); // ��ӡ�������������

private:
    PrintType print_type{TypePrintOutput};
    int interval{1000};

    TraceInfo *trace{nullptr};

    ChannelStatusCollect *chn_status{nullptr};  // ͨ��״̬
    ChannelRealtimeStatus *chn_rt_status{nullptr}; // ʵʱ����
    ChannelMcStatus *mc_status{nullptr};    // MC״̬
    SpindleControl *spindle{nullptr}; // ����״̬
    SCSystemConfig *sys_config{nullptr}; // ϵͳ����
    SCChannelConfig *chn_config{nullptr}; // ͨ������
    SCAxisConfig *axis_config{nullptr}; // ������
    SCCoordConfig *coord{nullptr}; // ����ϵ����
    SCCoordConfig *ex_coord{nullptr}; // ��չ����ϵ����
    SCCoordConfig *global_coord{nullptr}; // ȫ������ϵ����
    SCToolOffsetConfig *tool_offset{nullptr}; // ����ƫ������
    SCToolPotConfig *tool_pot{nullptr}; // ������Ϣ����
    FiveAxisConfig *fiveaxis_config{nullptr}; // ��������
    const FRegBits *F{nullptr}; // F�Ĵ���
    const GRegBits *G{nullptr}; //G�Ĵ���
    PmcAxisCtrl *pmc_axis_ctrl{nullptr};
    SyncAxisCtrl *sync_axis_ctrl{nullptr};
    SCFiveAxisV2Config *fiveaxisV2_config{nullptr};

    bool exit_flag{false};
    int msg_new_count{0};
    int msg_delete_count{0};
    std::set<int> m_marco_select;
    std::mutex m_marco_mutex;
};

#endif
