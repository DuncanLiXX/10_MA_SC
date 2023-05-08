#ifndef SERVO_GUIDE_H
#define SERVO_GUIDE_H

#include <chrono>
#include <queue>
#include <utility>
#include <mutex>
#include <memory>

#include "hmi_shared_data.h"
#include "geometry_data.h"

struct SG_Type;

using SG_DATA = std::tuple<double, double, double, double>; //最终发送的数据类型，现阶段固定为4个double类型，方便通讯库用固定类型存储
using SG_Type_Ptr = std::shared_ptr<SG_Type>;               //伺服引导类型，具体类型参见 E_SG_Type

enum class E_SG_Type {
    SG_None = -1,
    SG_Rect,            //矩形
    SG_Circle,          //圆
    SG_Rect_Circle,     //圆方
    SG_Tapping,         //刚性攻丝
};

/**
 * @brief 伺服引导类型基类，不直接使用
 */
struct SG_Type {
    SG_Type(int8_t interval, int8_t axis_one, int8_t axis_two, E_SG_Type type)
        : type_(type), axis_one_(axis_one), axis_two_(axis_two), interval_(interval)
            { }

    E_SG_Type type_  = E_SG_Type::SG_None;      //类型
    int8_t axis_one_ = -1;                      //需要监听的轴号No.1
    int8_t axis_two_ = -1;                      //需要监听的轴号No.2
    int8_t interval_ = 8;                       //采样周期

    DPoint origin_point_;                       //相对坐标起始点

    double *feedback_pos_   = nullptr;
    double *intp_pos_       = nullptr;
    double *feedback_speed_ = nullptr;

    void SetInstance(double *intp, double *feedback, double *speed);    //数据实例化

    void SetOriginPoint(DPoint);    //设置相对起点坐标
    virtual bool Verify() const;
    virtual SG_DATA GenData() = 0;
};

/**
 * @brief 矩形轨迹测量
 */
struct SG_Rect_Type : public SG_Type {
    SG_Rect_Type(SG_Rect_Config cfg);
    SG_DATA GenData() override;
};

/**
 * @brief 圆型轨迹测量
 */
struct SG_Circle_Type : public SG_Type {
//    enum class E_SG_CType {
//        SG_None = -1,
//        SG_CW,           //顺时针
//        SG_CCW,          //逆时针
//    };
    SG_Circle_Type(SG_Circle_Config cfg);

    bool Verify() const override;
    SG_DATA GenData() override;

    //顺圆，逆圆
    //E_SG_CType circle_type_ = E_SG_CType::SG_None;
    //半径
    double radius_ = 0;     //>=0
};

/**
 * @brief 圆方轨迹测量
 */
struct SG_RecCir_Type : public SG_Type {

    SG_RecCir_Type(SG_RecCir_Config cfg);

    bool Verify() const override;
    SG_DATA GenData() override;

    int GetQuadrant(DPlane plane);// 理论坐标

    //长方形宽度(不包括倒角半径)
    double width_ = 0;
    //长方形高度(不包括倒角半径)
    double height_ = 0;
    //倒角半径
    double radius_ = 0;
};

/**
 * @brief 刚性攻丝轨迹测量
 */
struct SG_Tapping_Type : public SG_Type {
    SG_Tapping_Type(SG_Tapping_Config cfg);

    bool Verify() const override;
    SG_DATA GenData() override;
};


/**
 * @brief 伺服引导
 * 用于实现cnc系统的数据采集，数据发送等功能
 */
class ServeGuide {
public:

    ServeGuide();

    bool ReadyRecord();                                 // 打开数据传输通道
    bool StartRecord();                                 // 开始数据记录
    void PauseRecord();                                 // 结束数据记录
    void ResetRecord();                                 // 复位
    bool RefreshRecording();                            // 更新数据采集状态

    void RstOriginPoint();                              //重置起始点
    void SetOriginPoint(DPoint origin_point);           //设置起始点

    bool IsIdle() const;                                // 是否处于空闲状态
    bool IsRecord() const;
    bool IsReady() const;
    bool IsEmpty() const;
    int  CurState() const;

    bool SetType(SG_Type_Ptr type);                     // 设置采集类型
    bool SetInterval(unsigned interval);                // 设置采样周期
    bool IsTimeout();                                   // 周期到达

    virtual void RecordData(const double *feedback, const double *interp);  // 记录数据 //虚函数或者模板函数

    bool InitSocket();  // 初始化Socket
    bool Accept();
    bool Close();
    void ProcessData();

    bool IsDataReady(); // 是否可以发送数据
    void SendData();    // 发送数据

private:

    enum class E_SG_RunState{
        IDLE = 0,           //空闲
        READY = 1,          //准备好
        RECORDING = 2,      //采集中
        STOPPING = 3,       //停止中
    };

    constexpr static int MAX_INTERVAL = 10000;
    std::chrono::time_point<std::chrono::steady_clock> scan_cycle_;     // 扫描周期
    int scan_interval_ = 8;                                             // 扫描周期间隔

    E_SG_RunState state_ = E_SG_RunState::IDLE;

    // 数据存放(可能需要继承)
    bool origin_inited = false;         //是否已经记录起始点（用于增量计算）
    std::queue<SG_DATA> data_;
    mutable std::mutex data_mut_;

    int data_socket_ = -1;              //连接socket
    int data_send_fd = -1;              //数据传输socket

    SG_Type_Ptr type_ptr_;              //采集数据类型
};

#endif
