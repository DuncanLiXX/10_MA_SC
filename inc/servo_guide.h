#ifndef SERVO_GUIDE_H
#define SERVO_GUIDE_H

#include <chrono>
#include <queue>
#include <utility>
#include <mutex>
#include <memory>

#include "hmi_shared_data.h"
#include "geometry_data.h"

class SG_Type;

using SG_DATA = std::tuple<double, double, double, double>; //最终发送的数据类型，现阶段写死4个double类型，方便通讯库用固定类型存储
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
        : interval_(interval), type_(type),
          axis_one_(axis_one), axis_two_(axis_two) { }

    E_SG_Type type_  = E_SG_Type::SG_None;      //类型
    int8_t axis_one_ = -1;                      //需要监听的轴号No.1
    int8_t axis_two_ = -1;                      //需要监听的轴号No.2
    int8_t interval_ = 8;                       //采样周期

    virtual bool Verify() const;
    virtual SG_DATA GenData(const double *feedback, const double *interp) = 0;
};

/**
 * @brief 矩形轨迹测量
 */
struct SG_Rect_Type : public SG_Type {
    SG_Rect_Type(SG_Rect_Config cfg);
    SG_DATA GenData(const double *feedback, const double *interp) override;
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
    SG_DATA GenData(const double *feedback, const double *interp) override;

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
    SG_DATA GenData(const double *feedback, const double *interp) override;

    int GetQuadrant(DPlane plane);// 理论坐标

    //长方形宽度(不包括倒角半径)
    double width_ = 0;
    //长方形高度(不包括倒角半径)
    double height_ = 0;
    //倒角半径
    double radius_ = 0;

    //一象限
    //double area_1_x_min = 0;
    //double area_1_x_max = 0;
    //double area_1_y = 0;

    //二象限
    //double area_2_x_min = 0;
    //double area_2_x_max = 0;
    //double area_2_y_min = 0;
    //double area_2_y_max = 0;

    //三象限
    //double area_3_x = 0;
    //double area_3_y_min = 0;
    //double area_3_y_max = 0;

    //四象限
    //double area_4_x_min = 0;
    //double area_4_x_max = 0;
    //double area_4_y_min = 0;
    //double area_4_y_max = 0;

    //五象限
    //double area_5_x_min = 0;
    //double area_5_x_max = 0;
    //double area_5_y = 0;

    //六象限
    //double area_6_x_min = 0;
    //double area_6_x_max = 0;
    //double area_6_y_min = 0;
    //double area_6_y_max = 0;

    //七象限
    //double area_7_x = 0;
    //double area_7_y_min = 0;
    //double area_7_y_max = 0;

    //八象限
    //double area_8_x_min = 0;
    //double area_8_x_max = 0;
    //double area_8_y_min = 0;
    //double area_8_y_max = 0;
};

/**
 * @brief 刚性攻丝轨迹测量
 */
struct SG_Tapping_Type : public SG_Type {
    SG_Tapping_Type(SG_Tapping_Config cfg);

    bool Verify() const override;
    SG_DATA GenData(const double *feedback, const double *interp) override;
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
    bool RefreshRecording();

    bool IsIdle() const;                                // 是否处于空闲状态
    bool IsRecord() const;
    bool IsReady() const;
    bool IsEmpty() const;

    bool SetType(SG_Type_Ptr type);
    bool SetInterval(unsigned interval);
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
        //Paused = 3,         //暂停
        STOPPING = 4,       //停止中
    };

    constexpr static int MAX_INTERVAL = 10000;
    std::chrono::time_point<std::chrono::steady_clock> scan_cycle_;     // 扫描周期
    int scan_interval_ = 8;                                             // 扫描周期间隔

    E_SG_RunState state_ = E_SG_RunState::IDLE;

    // 数据存放(可能需要继承)
    std::queue<SG_DATA> data_;
    mutable std::mutex data_mut_;

    // 数据传输 socket
    int data_socket_ = -1;              //tcp socket
    //bool connect_ = false;              //是否连接
    int  data_send_fd = -1;

    SG_Type_Ptr type_ptr_;
};

#endif
