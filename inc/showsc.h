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
        TypeNone =                  0,//不打印
        TypeChnStatus =             1,//通道状态
        TypeMcStatus =              2,//MC状态
        TypeRealtimeData =          3,//实时数据
        TypeSpindle =               4,//主轴状态
        TypeSysConfig =             5,//系统配置
        TypeChnConfig =             6,//通道配置
        TypeAxisConfig =            7,//轴参配置
        TypeCoordConfig =           8,//工件坐标系偏置
        TypeExCoordConfig =         9,//拓展工件坐标系偏置
        TypeGrbCoordConfig =        10,//全局工件坐标系偏置
        TypeTooOffsetlConfig =      11,//刀具偏置配置
        TypeToolPotConfig =         12,//刀具信息
        TypeFiveAxisConfig =        13,//五轴参数
        TypeMode =                  14,//模态
        TypeWarning =               15,//警告
        TypeFRegState =             16,//F寄存器
        TypeGRegState =             17,//G寄存器
        TypePrintOutput =           18,//print函数的打印输出
    };

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

private:
    // 打印处理线程
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
    void PrintToolPotConfig();  // 打印道具信息配置
    void PrintFiveAxisCoinfig();    // 打印五轴参数配置
    void PrintMode();   // 打印模态
    void PrintWarning();    // 打印警告
    void PrintFRegState();  // 打印F寄存器
    void PrintGRegState();  // 打印G寄存器

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
