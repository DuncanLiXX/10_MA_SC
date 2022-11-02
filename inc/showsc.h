#ifndef SHOWSC_H
#define SHOWSC_H

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

struct ChannelStatusCollect;
struct ChannelRealtimeStatus;
struct ChannelMcStatus;

void ScPrintf(const char *__format,va_list args);

const std::string SwitchTopic = "/sc/switch";  // 打印类型选择
const std::string PrintTopic = "/sc/print";    // 打印输出

class ShowSc{
public:

    ShowSc();
    ~ShowSc();

    // 设置打印间隔 0代表只打印一次
    void SetInterval(int ms){interval = ms;}

    // 设置需要打印的数据类型，打印线程会根据类型来打印
    void SetPrintType(PrintType type){print_type = type;}

    // 添加各部件
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

    // 发送信息
    void SendMsg(string &s);

private:
    // 打印处理线程
    void ProcessPrintThread();

    // 根据key和value，格式化后追加到s中
    void AddPair(string &s,string key,int64_t value);
    void AddPair(string &s,string key,uint32_t value);
    void AddPair(string &s,string key,int32_t value);
    void AddPair(string &s,string key,uint16_t value);
    void AddPair(string &s,string key,int16_t value);
    void AddPair(string &s,string key,uint8_t value);
    void AddPair(string &s,string key,int8_t value);
    void AddPair(string &s,string key,double value);
    void AddPair(string &s,string key,string value);

    // 根据打印类型分别处理

    void PrintChnStatus();  // 打印通道状态
    void PrintMcStatus();   // 打印MC状态
    void PrintRealtimeData();   // 打印实时数据
    void PrintSpindle();    // 打印主轴状态
    void PrintSysConfig();  // 打印系统配置
    void PrintChnConfig();  // 打印通道配置
    void PrintAxisConfig(int axis); // 打印轴配置 axis从0开始
    void PrintCoordConfig();    // 打印工件坐标系配置
    void PrintExCoordConfig();  // 打印拓展工件坐标系配置
    void PrintGrbCoordConfig(); // 打印全局工件坐标系配置
    void PrintToolOffsetConfig();   // 打印刀具偏置配置
    void PrintToolPotConfig();  // 打印刀具信息配置
    void PrintFiveAxisCoinfig();    // 打印五轴参数配置
    void PrintMode();   // 打印模态
    void PrintWarning();    // 打印警告
    void PrintFRegState();  // 打印F寄存器
    void PrintGRegState();  // 打印G寄存器
    void PrintPmcAxisCtrl();    // 打印PMC轴信息

private:
    PrintType print_type{TypeNone};
    int interval{1000};

    TraceInfo *trace;

    ChannelStatusCollect *chn_status{nullptr};  // 通道状态
    ChannelRealtimeStatus *chn_rt_status{nullptr}; // 实时数据
    ChannelMcStatus *mc_status{nullptr};    // MC状态
    SpindleControl *spindle{nullptr}; // 主轴状态
    SCSystemConfig *sys_config{nullptr}; // 系统配置
    SCChannelConfig *chn_config{nullptr}; // 通道配置
    SCAxisConfig *axis_config{nullptr}; // 轴配置
    SCCoordConfig *coord{nullptr}; // 坐标系配置
    SCCoordConfig *ex_coord{nullptr}; // 拓展坐标系配置
    SCCoordConfig *global_coord{nullptr}; // 全局坐标系配置
    SCToolOffsetConfig *tool_offset{nullptr}; // 刀具偏置配置
    SCToolPotConfig *tool_pot{nullptr}; // 刀具信息配置
    FiveAxisConfig *fiveaxis_config{nullptr}; // 五轴配置
    const FRegBits *F; // F寄存器
    const GRegBits *G; //G寄存器
    PmcAxisCtrl *pmc_axis_ctrl{nullptr};

    bool exit_flag{false};
};

#endif
