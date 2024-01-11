#ifndef SPINDLE_CONTROL_H_
#define SPINDLE_CONTROL_H_

#include <stdint.h>

class MCCommunication;
class MICommunication;
class Variable;
class FRegBits;
class GRegBits;

struct SCAxisConfig;

/*
 *@date 2022/09/19
 *@brief:实现主轴功能逻辑
 */

namespace Spindle {
enum Polar
{
    Stop = -1,
    Positive = 0,
    Negative = 1,
};
enum Level  // 档位
{
    Low,    // 档位1
    Middle, // 档位2
    High    // 档位3
};
enum Mode
{
    Speed,
    Position
};

typedef struct
{
    bool tap_flag{false};       // 攻丝标记
    Polar polar{Stop};       // 主轴方向
    double F{0.0};              // 攻丝进给
    uint16_t S{0};              // 主轴转速
    uint8_t phy_axis{0};        // 主轴轴号
    uint8_t z_axis{0};          // z轴轴号
    double R{0.0};              // R平面
    int dir{1};					// G84  G74
}TapState;
}

class SpindleControl
{
public:
    SpindleControl();

    // 设置主轴相关组件
    void SetComponent(MICommunication *mi,
                    MCCommunication *mc,
                    FRegBits *f_reg,
                    const GRegBits *g_reg,
                    Variable *variable);
    // 设置主轴相关配置
    void SetSpindleParams(SCAxisConfig *spindle,
                          uint32_t da_prec,
                          uint8_t phy_axis,
                          uint8_t z_axis);

    void Reset();

    void InputSCode(uint32_t s_code);            // S指令输入
    uint32_t GetSCode();

    // 极性输入由 反转信号SRV(G70.4),正转信号SFR(G70.5) 决定
    // 1.主轴停: SRV==0&&SFR==0
    // 2.主轴正转: SRV==0&&SFR==1
    // 3.主轴反转: SRV==1&&SFR==0
    void InputPolar(Spindle::Polar polar);

    // 主轴模式由 刚性攻丝信号RGTAP(G61.0) 决定
    // 1.速度模式: RGTAP==0
    // 2.位置模式: RGTAP==1
    void SetMode(Spindle::Mode mode);
    Spindle::Mode GetMode();             // 获取主轴控制模式

    uint8_t Type();  // 0:没有主轴  1：主轴为虚拟轴 2：主轴为总线轴
    uint8_t GetPhyAxis(){return phy_axis;}

    bool isTapEnable();     // 刚性攻丝是否使能

    void SetTapFeed(double feed);       // 设置攻丝进给
    void StartRigidTap(double feed);    // 建立同步
    void CancelRigidTap();              // 取消同步
    void ResetTapFlag();

    void InputSSTP(bool _SSTP);     // _SSTP信号输入 低有效
    void InputSOR(bool SOR);        // SOR信号输入
    void InputSOV(uint8_t SOV);     // SOV信号输入
    void InputRI(uint16_t RI);      // RI信号输入
    void InputSGN(bool SGN);        // SGN信号输入
    void InputSSIN(bool SSIN);      // SSIN信号输入
    void InputSIND(bool SIND);      // SIND信号输入
    void InputORCMA(bool ORCMA);    // 定位信号信号输入
    void InputRGTAP(bool RGTAP);    // 刚性攻丝信号输入
    void InputRGMD(bool RGMD);      // 刚性攻丝模式切换
    void InputRTNT(bool RTNT);      // 攻丝回退
    void RspORCMA(bool success);    // 定位信号命令回复
    void RspCtrlMode(uint8_t axis, Spindle::Mode mode);  // 模式切换回复
    void RspAxisEnable(uint8_t axis, bool enable);       // 轴使能回复
    void RspSpindleSpeed(uint8_t axis, bool success);    // 0:速度未到达  1:速度到达

    void InputFIN(bool FIN);

    int32_t GetSpindleSpeed();  // 获取真实主轴转速，带有方向，用于hmi显示转速 单位:rpm

    double GetSpdAngle();  // 获取主轴角度 单位：度
    Spindle::Polar CalPolar();    // 根据当前状态获取主轴转向

    void EStop();

    double Get_TapRatio() const { return tap_ratio; }   //获取攻丝比例

private:
    void UpdateParams();        // 更新常用主轴参数到成员变量中
    void UpdateSpindleState();  // 根据当前状态更新转速
    uint32_t GetMaxSpeed();     // 获取当前档位最大转速
    int32_t CalDaOutput();     // 根据当前状态获取DA电平值

    // 根据当前速度更新档位输出
    //参数： speed: 主轴转速 单位：rpm
    //返回值： 0：不换挡  1：换挡
    bool UpdateSpindleLevel(uint16_t speed);

    // 根据当前状态获取目标档位
    void CalLevel(uint8_t &GR1O, uint8_t &GR2O, uint8_t &GR3O);

    // 根据当前状态发送主轴转速给Mi
    void SendSpdSpeedToMi();

    void SendSpdSpeedToMi(int16_t speed);

    // 异步处理主轴定向逻辑
    void ProcessORCMA(bool ORCMA);
    // 异步处理模式切换完成
    void ProcessModeChanged(Spindle::Mode mode);
    // 异步处理换挡逻辑
    void ProcessSwitchLevel();

    // 向MC发送刚性攻丝状态命令，MC会切换到刚性攻丝的速度规划参数
    void SendMcRigidTapFlag(bool enable);

    // 保存/加载攻丝状态
    void SaveTapState();
    bool LoadTapState(Spindle::TapState &state);

    // 异步处理攻丝回退
    void ProcessRTNT();

public:
    MICommunication *mi{nullptr};
    MCCommunication *mc{nullptr};
    int chn{0};
    SCAxisConfig *spindle{nullptr}; // 主轴1
    uint8_t phy_axis{0};       // 主轴物理轴号,从0开始
    uint8_t z_axis{2};         // z轴物理轴号,从0开始
    uint32_t da_prec{2048};    // DA精度

    FRegBits *F{nullptr};            // F寄存器
    const GRegBits *G{nullptr};            // G寄存器

    Variable *variable;

    Spindle::Level to_level{Spindle::Low};        // 目标档位
    Spindle::Level level{Spindle::Low};           // 当前档位
    Spindle::Polar cnc_polar{Spindle::Stop};    // 主轴方向
    uint32_t cnc_speed{0};              // S代码的转速 单位:rpm
    int32_t cnc_speed_virtual{0};
    double tap_feed{0.0};                    // 攻丝进给

    Spindle::Mode mode{Spindle::Speed};             // 控制模式
    bool tap_enable{false};     // 攻丝状态  false:不在攻丝状态 true:处于攻丝状态
    double  tap_ratio{1.0};       // 攻丝比例
    bool motor_enable{false};   // 电机使能状态
    bool wait_sar{false};       // 等待速度到达 0:不在等待 1:正在等待
    bool wait_off{false};       // 等待电机下使能 0:不在等待 1:正在等待
    bool wait_on{false};        // 等待电机上使能 0:不在等待 1:正在等待

    // 信号
    bool _SSTP;  //主轴停止信号     G29.6  低有效
    bool SOR;    //主轴准停信号    G29.5
    bool SAR;    //主轴到达信号    G29.4
    uint8_t SOV;    //主轴倍率       G30
    uint16_t RI; //主轴转速输入    G31~G32
    bool SGN;    //PMC输入的主轴方向    G33.5   0：正 1：负
    bool SSIN;   //主轴方向由CNC决定还是PMC决定  G33.6   0：CNC 1：PMC
    bool SIND;   //主轴速度由CNC决定还是PMC决定 G33.7   0：CNC 1：PMC
    bool ORCMA{0};  // 主轴定向信号 G70.6
    bool RGTAP;  //刚攻状态  0：退出刚攻状态  1：进入刚攻状态 G61.0
    bool RGMD;   //主轴控制模式 0：速度模式  1：位置模式 G61.1
    bool RTNT;   //攻丝回退信号 G62.6

    // 参数
    uint8_t GST{0};    //(1657)SOR信号用于： 0：主轴定向 1：齿轮换档
    uint8_t SGB{0};    //(1658)齿轮换档方式 0：A方式 1：B方式
    uint8_t SFA{0};    //(1659)换挡功能开关： 0：关闭 1：打开
    uint8_t ORM{0};    //(1660)主轴定向时的转向 0：正 1：负
    uint8_t TCW{1};    //(1661)主轴转向是否受M03/M04影响 0：不受影响 1：受影响
    uint8_t CWM{0};    //(1662)主轴转向取反 0：否  1：是
    uint8_t TSO{0};    //(1663)螺纹切削和刚性攻丝时，主轴倍率  0：强制100%  1：有效

    uint16_t TM{16000};    //SF信号输出延时 单位:us
    uint16_t TMF{16000};   //SF选通信号打开后，数据的输出延时 单位:us

    Spindle::TapState tap_state;    // 攻丝数据记录
    bool running_rtnt{false};       // 是否正在攻丝回退
    double pos_zero_ang{0.0};            // 0度机械坐标
    int TapDir = 1;        // 攻丝方向
};

#endif
