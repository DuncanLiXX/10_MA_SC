#ifndef SHOWSC_H
#define SHOWSC_H

#include "singleton.h"
#include "channel_data.h"

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

void ScPrintf(const char *__format,va_list args);


class ShowSc{
public:
    enum PrintType{
        TypeNone =                  0,//����ӡ
        TypeChnStatus =             1,//ͨ��״̬
        TypeMcStatus =              2,//MC״̬
        TypeRealtimeData =          3,//ʵʱ����
        TypeSpindle =               4,//����״̬
        TypeSysConfig =             5,//ϵͳ����
        TypeChnConfig =             6,//ͨ������
        TypeAxisConfig =            7,//�������
        TypeCoordConfig =           8,//��������ϵƫ��
        TypeExCoordConfig =         9,//��չ��������ϵƫ��
        TypeGrbCoordConfig =        10,//ȫ�ֹ�������ϵƫ��
        TypeTooOffsetlConfig =      11,//����ƫ������
        TypeToolPotConfig =         12,//������Ϣ
        TypeFiveAxisConfig =        13,//�������
        TypeMode =                  14,//ģ̬
        TypeWarning =               15,//����
        TypeFRegState =             16,//F�Ĵ���
        TypeGRegState =             17,//G�Ĵ���
        TypePrintOutput =           18,//print�����Ĵ�ӡ���
    };

    ShowSc();
    ~ShowSc();

    // ���ô�ӡ��� 0����ֻ��ӡһ��
    void SetInterval(int ms){interval = ms;}

    // ������Ҫ��ӡ���������ͣ���ӡ�̻߳������������ӡ
    void SetPrintType(PrintType type){print_type = type;}

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

private:
    // ��ӡ�����߳�
    void ProcessPrintThread();

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

private:
    PrintType print_type{TypeNone};
    int interval{1000};

    ChannelStatusCollect *chn_status{nullptr};
    ChannelRealtimeStatus *chn_rt_status{nullptr};
    ChannelMcStatus *mc_status{nullptr};
    SpindleControl *spindle{nullptr};
    SCSystemConfig *sys_config{nullptr};
    SCChannelConfig *chn_config{nullptr};
    SCAxisConfig *axis_config{nullptr};
    SCCoordConfig *coord{nullptr};
    SCCoordConfig *ex_coord{nullptr};
    SCCoordConfig *global_coord{nullptr};
    SCToolOffsetConfig *tool_offset{nullptr};
    SCToolPotConfig *tool_pot{nullptr};
    FiveAxisConfig *fiveaxis_config{nullptr};
    FRegBits *F;
    const GRegBits *G;

    bool exit_flag{false};
};

#endif
